#!/usr/bin/env python3
"""Read-only acceptance checker for the PV-A mimic-aware Table 4 rerun.

The checker walks the sealed source roster and the result database in ordinal
order.  It independently re-aggregates the published metrics, recomputes each
URDF sampling-plan binding, and checks the canonical state stream without
loading the full release into memory.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
from functools import lru_cache
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import stat
import sys
from typing import Any, Mapping, Sequence
import xml.etree.ElementTree as ET
import zlib


SCRIPT = Path(__file__).resolve()
SCRIPT_DIR = SCRIPT.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import check_table4_full_release as generic
import run_table4_full_release as table4


RUN_SCHEMA = "pva_table4_mimic_aware_full_release_run_v2"
RECEIPT_SCHEMA = "pva_table4_mimic_aware_full_release_receipt_v1"
RESULT_DB_SCHEMA = "pva_table4_mimic_aware_results_db_v1"
SOURCE_RECEIPT_SCHEMA = "pva_table1234_full_release_receipt_v1"
SOURCE_DB_SCHEMA = "pva_table1234_results_db_v1"
PROTOCOL_ID = "urdf_sim_ready_table4_pva_full_release_v2"
SAMPLING_PROTOCOL = generic.SAMPLING_PROTOCOL_V2
PACKAGE_ROOT_BINDING_SCHEMA = "pva_package_root_override_v1"
PACKAGE_ROOT_MAPPING_POLICY = "relative_prefix_substitution_v1"
PACKAGE_VERIFICATION = "frozen_package_files_sha256_v1"


class CheckError(ValueError):
    """Raised when the v2 release evidence does not close."""


def _canonical_text(value: Any) -> str:
    return json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    )


def _canonical_sha256(value: Any) -> str:
    return hashlib.sha256(_canonical_text(value).encode("utf-8")).hexdigest()


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _cached_sha256(path: Path, cache: dict[Path, str]) -> str:
    resolved = Path(path).resolve(strict=True)
    if resolved not in cache:
        cache[resolved] = _sha256_file(resolved)
    return cache[resolved]


def _load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise CheckError(f"cannot read JSON {path}: {error}") from error
    if not isinstance(value, dict):
        raise CheckError(f"JSON artifact is not an object: {path}")
    return value


def _self_hash(value: Mapping[str, Any], field: str) -> str:
    payload = dict(value)
    payload.pop(field, None)
    return _canonical_sha256(payload)


def _require_self_hash(value: Mapping[str, Any], field: str, label: str) -> None:
    if value.get(field) != _self_hash(value, field):
        raise CheckError(f"{label} self-hash mismatch")


def _read_meta(connection: sqlite3.Connection) -> dict[str, Any]:
    return {
        str(key): json.loads(value)
        for key, value in connection.execute("SELECT key, value FROM meta")
    }


def _sealed_sidecar_snapshot(database: Path) -> dict[str, tuple[int, int, int, int] | None]:
    snapshot: dict[str, tuple[int, int, int, int] | None] = {}
    for suffix in ("-wal", "-journal", "-shm"):
        sidecar = Path(f"{database}{suffix}")
        try:
            info = sidecar.lstat()
        except FileNotFoundError:
            snapshot[suffix] = None
            continue
        if stat.S_ISLNK(info.st_mode) or not stat.S_ISREG(info.st_mode):
            raise CheckError(f"unsafe sealed SQLite sidecar: {sidecar}")
        if suffix in {"-wal", "-journal"} and info.st_size != 0:
            raise CheckError(f"sealed SQLite {suffix[1:]} sidecar is non-empty: {sidecar}")
        snapshot[suffix] = (
            int(info.st_dev),
            int(info.st_ino),
            int(info.st_size),
            int(info.st_mtime_ns),
        )
    return snapshot


def _verify_sealed_database(path: Path, expected_sha256: str) -> Path:
    database = Path(path)
    try:
        before = database.lstat()
    except FileNotFoundError as error:
        raise CheckError(f"sealed SQLite database is missing: {database}") from error
    if stat.S_ISLNK(before.st_mode) or not stat.S_ISREG(before.st_mode):
        raise CheckError(f"sealed SQLite database is not a regular non-symlink file: {database}")
    if (
        not isinstance(expected_sha256, str)
        or len(expected_sha256) != 64
        or any(character not in "0123456789abcdef" for character in expected_sha256)
    ):
        raise CheckError("sealed SQLite expected SHA-256 is invalid")
    before_sidecars = _sealed_sidecar_snapshot(database)
    if _sha256_file(database) != expected_sha256:
        raise CheckError(f"sealed SQLite database SHA-256 mismatch: {database}")
    after = database.lstat()
    after_sidecars = _sealed_sidecar_snapshot(database)
    identity = lambda info: (
        int(info.st_dev),
        int(info.st_ino),
        int(info.st_size),
        int(info.st_mtime_ns),
    )
    if identity(after) != identity(before) or after_sidecars != before_sidecars:
        raise CheckError(f"sealed SQLite snapshot changed during verification: {database}")
    return database.resolve(strict=True)


def _connect_sealed_immutable(path: Path, expected_sha256: str) -> sqlite3.Connection:
    resolved = _verify_sealed_database(path, expected_sha256)
    connection = sqlite3.connect(
        f"{resolved.as_uri()}?mode=ro&immutable=1",
        uri=True,
    )
    try:
        connection.execute("PRAGMA query_only=ON")
        query_only = connection.execute("PRAGMA query_only").fetchone()
        if query_only is None or int(query_only[0]) != 1:
            raise CheckError("failed to enforce query_only on sealed SQLite database")
    except BaseException:
        connection.close()
        raise
    return connection


def _plain_absolute_root(value: Any, label: str) -> Path:
    if not isinstance(value, str) or not value:
        raise CheckError(f"{label} is missing")
    path = Path(value)
    if not path.is_absolute() or ".." in path.parts:
        raise CheckError(f"{label} is not a safe absolute path")
    current = Path(path.anchor)
    for component in path.parts[1:]:
        current /= component
        try:
            info = current.lstat()
        except OSError as error:
            raise CheckError(f"{label} is unavailable: {path}") from error
        if stat.S_ISLNK(info.st_mode):
            raise CheckError(f"{label} contains a symlink: {path}")
    try:
        resolved = path.resolve(strict=True)
    except OSError as error:
        raise CheckError(f"{label} is unavailable: {path}") from error
    if resolved != path or not path.is_dir():
        raise CheckError(f"{label} is not a plain directory: {path}")
    try:
        next(path.iterdir(), None)
    except OSError as error:
        raise CheckError(f"{label} is not readable: {path}") from error
    return path


def _plain_descendant(
    path: Path,
    root: Path,
    label: str,
    *,
    kind: str,
) -> Path:
    try:
        relative = path.relative_to(root)
    except ValueError as error:
        raise CheckError(f"{label} escapes the physical package root") from error
    if relative == Path(".") or ".." in relative.parts:
        raise CheckError(f"{label} has an unsafe mirror mapping")
    current = root
    for component in relative.parts:
        current /= component
        try:
            info = current.lstat()
        except OSError as error:
            raise CheckError(f"{label} is unavailable: {path}") from error
        if stat.S_ISLNK(info.st_mode):
            raise CheckError(f"{label} contains a symlink: {path}")
    try:
        resolved = path.resolve(strict=True)
    except OSError as error:
        raise CheckError(f"{label} is unavailable: {path}") from error
    if resolved != path:
        raise CheckError(f"{label} is not a plain path: {path}")
    if kind == "directory" and not path.is_dir():
        raise CheckError(f"{label} is not a directory: {path}")
    if kind == "file" and not stat.S_ISREG(path.lstat().st_mode):
        raise CheckError(f"{label} is not a regular file: {path}")
    return path


def _package_root_context(
    manifest: Mapping[str, Any],
    receipt: Mapping[str, Any],
    roster: Mapping[str, Any],
    source_context: Mapping[str, Any],
) -> dict[str, Any] | None:
    binding = manifest.get("package_root_binding")
    if receipt.get("package_root_binding") != binding:
        raise CheckError("receipt package-root binding mismatch")
    if binding is None:
        return None
    if not isinstance(binding, Mapping):
        raise CheckError("manifest package-root binding is invalid")
    expected_fields = {
        "schema_version",
        "mapping_policy",
        "logical_root",
        "physical_root",
        "roster_manifest_content_sha256",
        "package_verification",
        "binding_content_sha256",
    }
    if set(binding) != expected_fields:
        raise CheckError("manifest package-root binding fields mismatch")
    if binding.get("schema_version") != PACKAGE_ROOT_BINDING_SCHEMA:
        raise CheckError("package-root binding schema mismatch")
    if binding.get("mapping_policy") != PACKAGE_ROOT_MAPPING_POLICY:
        raise CheckError("package-root mapping policy mismatch")
    if binding.get("package_verification") != PACKAGE_VERIFICATION:
        raise CheckError("package-root verification policy mismatch")
    _require_self_hash(binding, "binding_content_sha256", "package-root binding")
    source_bindings = roster.get("source_bindings")
    if not isinstance(source_bindings, Mapping):
        raise CheckError("source roster has no source bindings")
    logical_value = source_bindings.get("extracted_root")
    if not isinstance(logical_value, str) or not logical_value:
        raise CheckError("source roster has no frozen extracted root")
    logical_root = Path(logical_value)
    if not logical_root.is_absolute() or ".." in logical_root.parts:
        raise CheckError("source roster extracted root is not a safe absolute path")
    if binding.get("logical_root") != logical_value:
        raise CheckError("package-root logical root differs from the sealed roster")
    roster_hash = source_context.get("roster_manifest_content_sha256")
    if (
        binding.get("roster_manifest_content_sha256") != roster_hash
        or roster.get("manifest_content_sha256") != roster_hash
    ):
        raise CheckError("package-root binding has the wrong source roster")
    physical_root = _plain_absolute_root(
        binding.get("physical_root"), "physical package root"
    )
    return {
        "binding": dict(binding),
        "logical_root": logical_root,
        "physical_root": physical_root,
    }


def _normalized_frozen_package_files(
    row: Mapping[str, Any], ordinal: int
) -> list[dict[str, Any]]:
    expected = row.get("package_files")
    if not isinstance(expected, list):
        raise CheckError(f"asset {ordinal} has no frozen package file binding")
    if _canonical_sha256(expected) != row.get("package_binding_sha256"):
        raise CheckError(f"asset {ordinal} package binding self-hash mismatch")
    normalized: list[dict[str, Any]] = []
    names: set[str] = set()
    for index, item in enumerate(expected):
        if not isinstance(item, Mapping):
            raise CheckError(f"asset {ordinal} package file {index} is invalid")
        name = item.get("path")
        size = item.get("size", item.get("bytes"))
        digest = item.get("sha256")
        if not isinstance(name, str) or not name:
            raise CheckError(f"asset {ordinal} package file {index} has no path")
        relative = Path(name)
        if (
            relative.is_absolute()
            or relative == Path(".")
            or ".." in relative.parts
            or relative.as_posix() != name
            or name in names
        ):
            raise CheckError(f"asset {ordinal} package file path is unsafe: {name}")
        if isinstance(size, bool) or not isinstance(size, int) or size < 0:
            raise CheckError(f"asset {ordinal} package file size is invalid: {name}")
        if (
            not isinstance(digest, str)
            or len(digest) != 64
            or any(character not in "0123456789abcdef" for character in digest)
        ):
            raise CheckError(f"asset {ordinal} package file hash is invalid: {name}")
        names.add(name)
        normalized.append({"path": name, "size": size, "sha256": digest})
    return sorted(normalized, key=lambda item: str(item["path"]))


def _observed_package_files(package: Path, ordinal: int) -> list[dict[str, Any]]:
    observed: list[dict[str, Any]] = []
    try:
        candidates = sorted(package.rglob("*"), key=lambda item: item.as_posix())
    except OSError as error:
        raise CheckError(f"asset {ordinal} mirror package cannot be scanned") from error
    for candidate in candidates:
        try:
            info = candidate.lstat()
        except OSError as error:
            raise CheckError(f"asset {ordinal} mirror package changed during scan") from error
        if stat.S_ISLNK(info.st_mode):
            raise CheckError(f"asset {ordinal} mirror package contains a symlink")
        if stat.S_ISDIR(info.st_mode):
            continue
        if not stat.S_ISREG(info.st_mode):
            raise CheckError(f"asset {ordinal} mirror package contains a special file")
        relative = candidate.relative_to(package).as_posix()
        observed.append(
            {
                "path": relative,
                "size": info.st_size,
                "sha256": _sha256_file(candidate),
            }
        )
    return observed


def _verified_mirror_paths(
    row: Mapping[str, Any],
    context: Mapping[str, Any],
    ordinal: int,
) -> dict[str, Any]:
    logical_root = Path(context["logical_root"])
    physical_root = Path(context["physical_root"])
    logical_package = Path(str(row.get("source_path", "")))
    logical_urdf = Path(str(row.get("primary_urdf_path", "")))
    if (
        not logical_package.is_absolute()
        or not logical_urdf.is_absolute()
        or ".." in logical_package.parts
        or ".." in logical_urdf.parts
    ):
        raise CheckError(f"asset {ordinal} has unsafe frozen logical paths")
    try:
        package_relative = logical_package.relative_to(logical_root)
        urdf_relative = logical_urdf.relative_to(logical_package)
    except ValueError as error:
        raise CheckError(f"asset {ordinal} escapes the frozen logical root") from error
    if (
        package_relative == Path(".")
        or urdf_relative == Path(".")
        or ".." in package_relative.parts
        or ".." in urdf_relative.parts
    ):
        raise CheckError(f"asset {ordinal} has an unsafe mirror mapping")
    package = _plain_descendant(
        physical_root / package_relative,
        physical_root,
        f"asset {ordinal} mirror package",
        kind="directory",
    )
    urdf = _plain_descendant(
        package / urdf_relative,
        physical_root,
        f"asset {ordinal} mirror URDF",
        kind="file",
    )
    try:
        urdf.relative_to(package)
    except ValueError as error:
        raise CheckError(f"asset {ordinal} mirror URDF escapes its package") from error
    expected = _normalized_frozen_package_files(row, ordinal)
    observed = _observed_package_files(package, ordinal)
    if observed != expected:
        raise CheckError(f"asset {ordinal} mirror package binding drift")
    return {
        "logical_package": logical_package,
        "logical_urdf": logical_urdf,
        "evaluation_package": package,
        "evaluation_urdf": urdf,
        "evaluation_package_relative_path": package_relative.as_posix(),
        "evaluation_urdf_relative_path": urdf_relative.as_posix(),
    }


def _artifact_closure(
    output: Path,
    receipt: Mapping[str, Any],
    hash_cache: dict[Path, str],
) -> None:
    artifact_path = output / "artifact_manifest.json"
    artifact = _load_json(artifact_path)
    _require_self_hash(
        artifact, "artifact_manifest_content_sha256", "artifact manifest"
    )
    rows = artifact.get("artifacts")
    if not isinstance(rows, list):
        raise CheckError("artifact manifest has no artifact list")
    names: set[str] = set()
    for index, row in enumerate(rows):
        if not isinstance(row, Mapping) or not isinstance(row.get("path"), str):
            raise CheckError(f"artifact manifest row {index} is invalid")
        relative = Path(str(row["path"]))
        if relative.is_absolute() or ".." in relative.parts:
            raise CheckError(f"artifact manifest row {index} escapes output")
        name = relative.as_posix()
        if name in names:
            raise CheckError(f"duplicate artifact: {name}")
        names.add(name)
        path = (output / relative).resolve(strict=True)
        try:
            path.relative_to(output)
        except ValueError as error:
            raise CheckError(f"artifact escapes output: {name}") from error
        if not path.is_file() or path.is_symlink():
            raise CheckError(f"artifact is not a regular file: {name}")
        if int(row.get("bytes", -1)) != path.stat().st_size:
            raise CheckError(f"artifact byte count mismatch: {name}")
        if row.get("sha256") != _cached_sha256(path, hash_cache):
            raise CheckError(f"artifact hash mismatch: {name}")
    required = {
        "manifest.json",
        "protocol_snapshot.md",
        "records.jsonl",
        "asset_records.jsonl",
        "state_records.jsonl",
        "summary.json",
        "summary.md",
        "checkpoint.json",
        "results.sqlite3",
    }
    if not required.issubset(names):
        raise CheckError(f"artifact manifest is incomplete: {sorted(required - names)}")
    if receipt.get("artifact_manifest_sha256") != _cached_sha256(
        artifact_path, hash_cache
    ):
        raise CheckError("receipt artifact-manifest hash mismatch")


def _source_context(
    manifest: Mapping[str, Any], n_eval: int
) -> tuple[dict[str, Any], sqlite3.Connection, dict[str, Any], Path, str]:
    source = manifest.get("source")
    if not isinstance(source, Mapping):
        raise CheckError("manifest source binding is missing")
    receipt_path = Path(str(source.get("source_receipt", ""))).resolve(strict=True)
    source_receipt = _load_json(receipt_path)
    if source_receipt.get("schema_version") != SOURCE_RECEIPT_SCHEMA:
        raise CheckError("source receipt schema mismatch")
    _require_self_hash(source_receipt, "receipt_content_sha256", "source receipt")
    if source.get("source_receipt_sha256") != _sha256_file(receipt_path):
        raise CheckError("source receipt file hash mismatch")
    if source.get("source_receipt_content_sha256") != source_receipt.get(
        "receipt_content_sha256"
    ):
        raise CheckError("source receipt content binding mismatch")
    source_root = receipt_path.parent
    database_path = source_root / str(source_receipt["result_database"])
    if str(database_path.resolve()) != str(
        Path(str(source.get("source_result_database", ""))).resolve()
    ):
        raise CheckError("source database path binding mismatch")
    source_database_sha256 = source_receipt.get("result_database_sha256")
    if source.get("source_result_database_declared_sha256") != source_database_sha256:
        raise CheckError("source database digest declaration mismatch")
    roster_path = Path(str(source_receipt["roster_manifest"])).resolve(strict=True)
    if str(roster_path) != str(Path(str(source.get("roster_manifest", ""))).resolve()):
        raise CheckError("source roster path binding mismatch")
    if source.get("roster_manifest_sha256") != _sha256_file(roster_path):
        raise CheckError("source roster file hash mismatch")
    roster = _load_json(roster_path)
    _require_self_hash(roster, "manifest_content_sha256", "source roster")
    if source.get("roster_manifest_content_sha256") != roster.get(
        "manifest_content_sha256"
    ):
        raise CheckError("source roster content binding mismatch")
    if n_eval > int(source_receipt["N_eval"]):
        raise CheckError("v2 run exceeds the sealed source cohort")
    connection = _connect_sealed_immutable(database_path, source_database_sha256)
    meta = _read_meta(connection)
    if meta.get("schema_version") != SOURCE_DB_SCHEMA:
        connection.close()
        raise CheckError("source database schema mismatch")
    if meta.get("asset_import_state") != "COMPLETE":
        connection.close()
        raise CheckError("source database asset import is incomplete")
    return dict(source), connection, roster, database_path, source_database_sha256


def _expected_input_identity(
    row: Mapping[str, Any], plan: Mapping[str, Any]
) -> str:
    independent = int(plan["independent_dof_count"])
    package = Path(str(row["source_path"])).resolve()
    urdf = Path(str(row["primary_urdf_path"])).resolve()
    values = {
        "dataset": "pva",
        "dataset_id": str(row["asset_id"]),
        "category": str(row.get("raw_category", row.get("category", ""))),
        "urdf_path": str(urdf),
        "primary_urdf_relative_path": str(
            row.get("primary_urdf_relative_path", "model.urdf")
        ),
        "expected_primary_urdf_sha256": row.get("primary_urdf_sha256"),
        "expected_movable_joints": int(row.get("joint_count", 0)),
        "package_binding_sha256": row.get("package_binding_sha256"),
        "sampling_protocol": SAMPLING_PROTOCOL,
        "independent_dof_count": independent,
        "range_evaluable_independent_dof_count": int(
            plan["range_evaluable_independent_dof_count"]
        ),
        "mimic_joint_count": int(plan["mimic_joint_count"]),
        "joint_sampling_plan_sha256": plan["joint_sampling_plan_sha256"],
        "single_state_expected": generic.SINGLE_SAMPLES * independent,
        "sobol_state_expected": generic.SOBOL_SAMPLES if independent else 0,
    }
    del package  # Resolving it above retains the runner's path semantics.
    return _canonical_sha256(values)


@lru_cache(maxsize=1)
def _collision_core() -> Any:
    return table4._core()


def _plan_metadata(
    row: Mapping[str, Any], *, urdf_path: Path | None = None
) -> dict[str, Any]:
    urdf = (
        Path(str(row.get("primary_urdf_path", ""))).resolve(strict=True)
        if urdf_path is None
        else Path(urdf_path)
    )
    expected_hash = str(row.get("primary_urdf_sha256", ""))
    if _sha256_file(urdf) != expected_hash:
        raise CheckError(f"source URDF hash drift: {row.get('asset_id')}")
    core = _collision_core()
    try:
        joints = core.parse_urdf_joints(urdf)
        if len(joints) != int(row.get("joint_count", 0)):
            raise ValueError("declared joint count mismatch")
        compiled = core.compile_joint_sampling_plan(joints)
        return {
            "independent_dof_count": int(compiled["independent_dof_count"]),
            "range_evaluable_independent_dof_count": int(
                compiled["range_evaluable_independent_dof_count"]
            ),
            "mimic_joint_count": int(compiled["mimic_joint_count"]),
            "fixed_root_joint_count": int(compiled["fixed_root_joint_count"]),
            "joint_sampling_plan_sha256": str(compiled["plan_sha256"]),
            "sampling_plan_error": None,
            "_independent_joint_names": [
                str(joint["name"]) for joint in compiled["independent_joints"]
            ],
            "_range_evaluable_joint_names": [
                str(joint["name"])
                for joint in compiled["independent_joints"]
                if bool(joint.get("sampling_range_evaluable"))
            ],
            "_compiled_plan": compiled,
        }
    except (OSError, ET.ParseError, TypeError, ValueError):
        # Preserve the evaluator's explicit fail-closed metadata for malformed
        # plans while exposing no executable independent names.
        metadata = core.sampling_plan_metadata(
            urdf,
            declared_dof=int(row.get("joint_count", 0)),
            expected_sha256=expected_hash,
        )
        return {
            **metadata,
            "_independent_joint_names": [],
            "_range_evaluable_joint_names": [],
            "_compiled_plan": None,
        }


@lru_cache(maxsize=None)
def _sobol_unit(dimension: int) -> tuple[tuple[float, ...], ...]:
    from scipy.stats import qmc

    values = qmc.Sobol(
        d=dimension, scramble=True, seed=table4.SOBOL_SEED
    ).random_base2(m=6)
    return tuple(tuple(float(value) for value in row) for row in values)


def _expected_joint_values(
    plan: Mapping[str, Any],
    phase: str,
    sample_index: int,
    joint_name: str | None,
) -> list[float]:
    compiled = plan.get("_compiled_plan")
    if not isinstance(compiled, Mapping):
        raise CheckError("state exists for an invalid sampling plan")
    independent_rows = list(compiled["independent_joints"])
    independent = [0.0] * len(independent_rows)
    if phase == "single_joint_sweep":
        names = [str(row["name"]) for row in independent_rows]
        if joint_name not in names:
            raise CheckError(f"single state names a non-independent joint: {joint_name}")
        position = names.index(str(joint_name))
        row = independent_rows[position]
        lower = float(row["sampling_lower"])
        upper = float(row["sampling_upper"])
        independent[position] = (
            lower
            + sample_index * (upper - lower) / (generic.SINGLE_SAMPLES - 1)
        )
    elif phase == "multi_joint_sobol":
        unit = _sobol_unit(len(independent_rows))[sample_index]
        independent = [
            float(
                float(row["sampling_lower"])
                + scalar
                * (float(row["sampling_upper"]) - float(row["sampling_lower"]))
            )
            for scalar, row in zip(unit, independent_rows, strict=True)
        ]
    elif phase != "rest":
        raise CheckError(f"unknown state phase: {phase}")
    root_positions = {
        str(row["name"]): position for position, row in enumerate(independent_rows)
    }
    fixed_roots = {
        int(index): float(value)
        for index, value in compiled.get("fixed_root_values", {}).items()
    }
    values: list[float] = []
    for binding in compiled["binding_rows"]:
        root_index = int(binding["root_index"])
        if root_index in fixed_roots:
            root_value = fixed_roots[root_index]
        else:
            root_value = independent[root_positions[str(binding["root_name"])]]
        values.append(
            float(binding["multiplier"]) * root_value + float(binding["offset"])
        )
    return values


def _state_rows(payload: bytes, expected: int, ordinal: int) -> list[dict[str, Any]]:
    if payload and not payload.endswith(b"\n"):
        raise CheckError(f"asset {ordinal} compressed states lack final newline")
    lines = payload.splitlines()
    if len(lines) != expected:
        raise CheckError(f"asset {ordinal} compressed state count mismatch")
    rows: list[dict[str, Any]] = []
    for line in lines:
        try:
            value = json.loads(line)
        except json.JSONDecodeError as error:
            raise CheckError(f"asset {ordinal} has invalid compressed state JSON") from error
        if not isinstance(value, dict):
            raise CheckError(f"asset {ordinal} has a non-object state")
        rows.append(value)
    return rows


def _state_nonnegative_int(
    state: Mapping[str, Any], field: str, ordinal: int, state_index: int
) -> int:
    value = state.get(field)
    if isinstance(value, bool) or not isinstance(value, int):
        raise CheckError(f"asset {ordinal} state {state_index} {field} is not an integer")
    parsed = int(value)
    if parsed < 0:
        raise CheckError(f"asset {ordinal} state {state_index} {field} is negative")
    return parsed


def _state_nonnegative_float(
    state: Mapping[str, Any], field: str, ordinal: int, state_index: int
) -> float:
    try:
        parsed = float(state.get(field))
    except (TypeError, ValueError) as error:
        raise CheckError(
            f"asset {ordinal} state {state_index} {field} is not numeric"
        ) from error
    if not math.isfinite(parsed) or parsed < 0.0:
        raise CheckError(f"asset {ordinal} state {state_index} {field} is invalid")
    return parsed


def _record_bool(
    record: Mapping[str, Any],
    field: str,
    expected: bool,
    ordinal: int,
    *,
    required: bool,
    exact: bool,
) -> None:
    if field not in record:
        if required:
            raise CheckError(f"asset {ordinal} is missing derived field: {field}")
        return
    try:
        observed = generic._bool(record, field)
    except (TypeError, ValueError) as error:
        raise CheckError(f"asset {ordinal} has invalid derived field: {field}") from error
    if observed is None:
        if required:
            raise CheckError(f"asset {ordinal} is missing derived field: {field}")
        return
    if (exact and bool(observed) != bool(expected)) or (
        not exact and bool(observed) and not bool(expected)
    ):
        raise CheckError(f"asset {ordinal} derived field mismatch: {field}")


def _record_number(
    record: Mapping[str, Any],
    field: str,
    expected: float,
    ordinal: int,
    *,
    required: bool,
) -> None:
    if field not in record or record.get(field) is None:
        if required:
            raise CheckError(f"asset {ordinal} is missing derived field: {field}")
        return
    try:
        observed = float(record.get(field))
    except (TypeError, ValueError) as error:
        raise CheckError(f"asset {ordinal} has invalid derived field: {field}") from error
    if not math.isfinite(observed) or not math.isclose(
        observed, expected, rel_tol=0.0, abs_tol=1e-15
    ):
        raise CheckError(f"asset {ordinal} derived field mismatch: {field}")


def _validate_states(
    record: Mapping[str, Any],
    states: Sequence[Mapping[str, Any]],
    ordinal: int,
    plan: Mapping[str, Any],
    *,
    require_normalization: bool = False,
) -> None:
    if int(record.get("state_records_count", -1)) != len(states):
        raise CheckError(f"asset {ordinal} state count binding mismatch")
    if record.get("state_records_sha256") != _canonical_sha256(states):
        raise CheckError(f"asset {ordinal} state hash binding mismatch")
    phase_counts = {"rest": 0, "single": 0, "sobol": 0}
    rest: set[int] = set()
    single: set[tuple[str, int]] = set()
    single_names: set[str] = set()
    sobol: set[int] = set()
    phase_free = {"rest": 0, "single": 0, "sobol": 0}
    phase_all_pair_free = False
    metric_max: float | None = None
    reset_max: float | None = None
    single_groups: dict[str, list[bool]] = defaultdict(list)
    for state_index, state in enumerate(states):
        if state.get("schema_version") != "table4_state_v2":
            raise CheckError(f"asset {ordinal} state {state_index} schema mismatch")
        for field in (
            "dataset_id",
            "order",
            "sampling_protocol",
            "joint_sampling_plan_sha256",
            "input_identity_sha256",
        ):
            expected = {
                "dataset_id": record.get("dataset_id"),
                "order": record.get("order"),
                "sampling_protocol": SAMPLING_PROTOCOL,
                "joint_sampling_plan_sha256": record.get(
                    "joint_sampling_plan_sha256"
                ),
                "input_identity_sha256": record.get("input_identity_sha256"),
            }[field]
            if state.get(field) != expected:
                raise CheckError(
                    f"asset {ordinal} state {state_index} {field} binding mismatch"
                )
        phase = state.get("phase")
        try:
            sample = int(state.get("sample_index"))
        except (TypeError, ValueError) as error:
            raise CheckError(f"asset {ordinal} has invalid sample index") from error
        if phase == "rest":
            phase_counts["rest"] += 1
            if sample != 0 or sample in rest or state.get("joint_name") not in {None, ""}:
                raise CheckError(f"asset {ordinal} has invalid rest coverage")
            rest.add(sample)
            phase_key = "rest"
        elif phase == "single_joint_sweep":
            phase_counts["single"] += 1
            name = state.get("joint_name")
            if not isinstance(name, str) or not name or not 0 <= sample < generic.SINGLE_SAMPLES:
                raise CheckError(f"asset {ordinal} has invalid single-state coverage")
            key = (name, sample)
            if key in single:
                raise CheckError(f"asset {ordinal} has duplicate single state")
            single.add(key)
            single_names.add(name)
            phase_key = "single"
        elif phase == "multi_joint_sobol":
            phase_counts["sobol"] += 1
            if (
                not 0 <= sample < generic.SOBOL_SAMPLES
                or sample in sobol
                or state.get("joint_name") not in {None, ""}
            ):
                raise CheckError(f"asset {ordinal} has invalid Sobol coverage")
            sobol.add(sample)
            phase_key = "sobol"
        else:
            raise CheckError(f"asset {ordinal} state {state_index} phase mismatch")

        all_contact_count = _state_nonnegative_int(
            state, "all_pair_contact_count", ordinal, state_index
        )
        all_illegal_count = _state_nonnegative_int(
            state, "all_pair_illegal_penetration_count", ordinal, state_index
        )
        non_adjacent_contact_count = _state_nonnegative_int(
            state, "non_adjacent_contact_count", ordinal, state_index
        )
        non_adjacent_illegal_count = _state_nonnegative_int(
            state, "non_adjacent_illegal_penetration_count", ordinal, state_index
        )
        if all_illegal_count > all_contact_count:
            raise CheckError(f"asset {ordinal} state {state_index} all-pair counters are invalid")
        if non_adjacent_illegal_count > non_adjacent_contact_count:
            raise CheckError(
                f"asset {ordinal} state {state_index} non-adjacent counters are invalid"
            )
        all_max = _state_nonnegative_float(
            state, "all_pair_max_penetration_m", ordinal, state_index
        )
        non_adjacent_max = _state_nonnegative_float(
            state, "non_adjacent_max_penetration_m", ordinal, state_index
        )
        metric = _state_nonnegative_float(
            state, "metric_max_penetration_m", ordinal, state_index
        )
        expected_metric = all_max if phase == "rest" else non_adjacent_max
        if not math.isclose(metric, expected_metric, rel_tol=0.0, abs_tol=1e-15):
            raise CheckError(
                f"asset {ordinal} state {state_index} metric penetration policy mismatch"
            )
        readback = _state_nonnegative_float(
            state, "reset_readback_max_abs_error", ordinal, state_index
        )
        phase_free[phase_key] += int(non_adjacent_illegal_count == 0)
        if phase == "single_joint_sweep":
            single_groups[str(state["joint_name"])].append(
                non_adjacent_illegal_count == 0
            )
        if phase == "rest":
            phase_all_pair_free = all_illegal_count == 0
        metric_max = metric if metric_max is None else max(metric_max, metric)
        reset_max = readback if reset_max is None else max(reset_max, readback)
        expected_values = _expected_joint_values(
            plan,
            str(phase),
            sample,
            state.get("joint_name") if isinstance(state.get("joint_name"), str) else None,
        )
        if state.get("joint_values_sha256") != _canonical_sha256(expected_values):
            raise CheckError(
                f"asset {ordinal} state {state_index} expanded joint-value hash mismatch"
            )
    for phase in phase_counts:
        executed = int(record.get(f"{phase}_state_executed", 0))
        if phase_counts[phase] != executed:
            raise CheckError(f"asset {ordinal} {phase} execution coverage mismatch")
    range_independent = int(record["range_evaluable_independent_dof_count"])
    if len(single_names) > range_independent:
        raise CheckError(f"asset {ordinal} sampled a mimic/non-evaluable joint")
    allowed_names = set(plan.get("_range_evaluable_joint_names", []))
    if not single_names.issubset(allowed_names):
        raise CheckError(f"asset {ordinal} sampled a mimic/non-evaluable joint")
    if phase_counts["single"] == int(record["single_state_expected"]) and single:
        expected_samples = set(range(generic.SINGLE_SAMPLES))
        if len(single_names) != int(record["independent_dof_count"]):
            raise CheckError(f"asset {ordinal} omitted an independent joint")
        if any(
            {sample for joint, sample in single if joint == name} != expected_samples
            for name in single_names
        ):
            raise CheckError(f"asset {ordinal} single-state grid is incomplete")
    if phase_counts["sobol"] == int(record["sobol_state_expected"]) and sobol:
        if sobol != set(range(generic.SOBOL_SAMPLES)):
            raise CheckError(f"asset {ordinal} Sobol grid is incomplete")

    # State rows are the source of truth for all phase-level counters and
    # collision maxima.  Missing fields remain tolerated only for historical
    # partial records; completed/new records must publish the derived values.
    completed = str(record.get("status")) == "completed"
    executed = {
        "rest": phase_counts["rest"],
        "single": phase_counts["single"],
        "sobol": phase_counts["sobol"],
    }
    expected = {
        "rest": int(record.get("rest_state_expected", 1)),
        "single": int(record.get("single_state_expected", 0)),
        "sobol": int(record.get("sobol_state_expected", 0)),
    }
    _record_number(
        record,
        "rest_non_adjacent_free",
        float(phase_free["rest"]),
        ordinal,
        required=completed,
    )
    _record_number(
        record,
        "single_non_adjacent_free",
        float(phase_free["single"]),
        ordinal,
        required=completed,
    )
    _record_number(
        record,
        "sobol_non_adjacent_free",
        float(phase_free["sobol"]),
        ordinal,
        required=completed,
    )
    rest_all_pair_expected = phase_counts["rest"] == 1 and phase_all_pair_free
    rest_non_adjacent_expected = phase_counts["rest"] == 1 and phase_free["rest"] == 1
    single_pass_expected = (
        phase_counts["single"] == expected["single"]
        and phase_free["single"] == expected["single"]
    )
    range_independent = int(record["range_evaluable_independent_dof_count"])
    independent = int(record["independent_dof_count"])
    sobol_pass_expected = (
        independent > 0
        and range_independent == independent
        and phase_counts["sobol"] == expected["sobol"]
        and phase_free["sobol"] == expected["sobol"]
    )
    measurement_expected = (
        range_independent == independent
        and all(executed[key] == expected[key] for key in executed)
    )
    strict_expected = bool(
        independent > 0
        and measurement_expected
        and rest_non_adjacent_expected
        and single_pass_expected
        and sobol_pass_expected
    )
    _record_bool(
        record,
        "rest_all_pair_cf",
        rest_all_pair_expected,
        ordinal,
        required=completed,
        exact=completed or "rest_all_pair_cf" in record,
    )
    _record_bool(
        record,
        "rest_non_adjacent_cf",
        rest_non_adjacent_expected,
        ordinal,
        required=completed,
        exact=completed or "rest_non_adjacent_cf" in record,
    )
    _record_bool(
        record,
        "single_joint_sweep_cf",
        single_pass_expected,
        ordinal,
        required=completed,
        exact=completed,
    )
    _record_bool(
        record,
        "multi_joint_sobol_cf",
        sobol_pass_expected,
        ordinal,
        required=completed,
        exact=completed,
    )
    _record_bool(
        record,
        "strict_collision_pass",
        strict_expected,
        ordinal,
        required=completed,
        exact=completed,
    )
    observed_measurement = record.get("measurement_complete")
    if observed_measurement is True and not measurement_expected:
        raise CheckError(f"asset {ordinal} measurement_complete is not state-derived")
    if completed and observed_measurement is not True:
        raise CheckError(f"asset {ordinal} completed without measurement_complete")
    joint_passes = sum(
        len(group) == generic.SINGLE_SAMPLES
        and all(group)
        for group in single_groups.values()
    )
    if "joint_single_sweep_cf_passed" in record or completed:
        try:
            observed_joint_passes = int(record.get("joint_single_sweep_cf_passed"))
        except (TypeError, ValueError) as error:
            raise CheckError(f"asset {ordinal} has invalid joint sweep pass count") from error
        if observed_joint_passes != joint_passes:
            raise CheckError(f"asset {ordinal} derived field mismatch: joint_single_sweep_cf_passed")
    if metric_max is not None:
        _record_number(
            record,
            "max_penetration_m",
            metric_max,
            ordinal,
            required=completed or record.get("max_penetration_m") is not None,
        )
    if reset_max is not None:
        _record_number(
            record,
            "max_reset_readback_error",
            reset_max,
            ordinal,
            required=completed or record.get("max_reset_readback_error") is not None,
        )
    if metric_max is not None:
        scale_value = record.get("object_bbox_diagonal_m")
        normalized_value = record.get("max_penetration_normalized")
        needs_normalized = completed or normalized_value is not None
        if scale_value is None:
            if needs_normalized:
                raise CheckError(f"asset {ordinal} is missing object_bbox_diagonal_m")
        else:
            try:
                scale = float(scale_value)
            except (TypeError, ValueError) as error:
                raise CheckError(f"asset {ordinal} has invalid object_bbox_diagonal_m") from error
            if not math.isfinite(scale) or scale <= 0.0:
                raise CheckError(f"asset {ordinal} has invalid object_bbox_diagonal_m")
            _record_number(
                record,
                "max_penetration_normalized",
                metric_max / scale,
                ordinal,
                required=needs_normalized,
            )
    elif any(
        record.get(field) is not None
        for field in (
            "max_penetration_m",
            "max_penetration_normalized",
            "max_reset_readback_error",
        )
    ):
        raise CheckError(f"asset {ordinal} publishes maxima without state evidence")

    # New evaluator records carry an explicit receipt for the pose used to
    # derive the normalization AABB.  Keep old parent records readable, but
    # never accept a partial or incorrect receipt on a new/corrected record.
    has_normalization_receipt = any(
        field in record
        for field in ("normalization_configuration", "normalization_joint_values_sha256")
    )
    if require_normalization or has_normalization_receipt:
        if record.get("normalization_configuration") != "expanded_rest":
            raise CheckError(f"asset {ordinal} normalization configuration mismatch")
        try:
            expected_rest_hash = _canonical_sha256(
                _expected_joint_values(plan, "rest", 0, None)
            )
        except (KeyError, TypeError, ValueError) as error:
            raise CheckError(f"asset {ordinal} cannot derive expanded rest state") from error
        if record.get("normalization_joint_values_sha256") != expected_rest_hash:
            raise CheckError(f"asset {ordinal} normalization rest-state hash mismatch")


def _new_aggregate() -> dict[str, Any]:
    return {
        "status_counts": defaultdict(int),
        "expected": defaultdict(int),
        "executed": defaultdict(int),
        "free": defaultdict(int),
        "passes": defaultdict(int),
        "collision_assets": 0,
        "measured_assets": 0,
        "max_values": [],
        "declared": 0,
        "independent": 0,
        "range_independent": 0,
        "mimic": 0,
        "fixed_roots": 0,
        "categories": set(),
    }


def _accumulate(acc: dict[str, Any], record: Mapping[str, Any], ordinal: int) -> None:
    sampling = generic._sampling_metadata(
        record, ordinal, expected_protocol=SAMPLING_PROTOCOL
    )
    status = str(record.get("status", ""))
    if status not in generic.VALID_STATUS:
        raise CheckError(f"asset {ordinal} has invalid status")
    acc["status_counts"][status] += 1
    acc["declared"] += int(sampling["declared_dof"])
    acc["independent"] += int(sampling["independent_dof"])
    acc["range_independent"] += int(sampling["range_independent_dof"] or 0)
    acc["mimic"] += int(sampling["mimic_joint_count"])
    acc["fixed_roots"] += int(sampling["fixed_root_joint_count"])
    acc["categories"].add(str(record.get("category") or "__UNSPECIFIED__"))
    for phase in ("rest", "single", "sobol"):
        expected = generic._asset_expected(
            record, phase, int(sampling["independent_dof"])
        )
        executed = generic._asset_executed(record, phase)
        free = generic._asset_free(record, phase)
        if executed > expected or (free is not None and free > executed):
            raise CheckError(f"asset {ordinal} has invalid {phase} accounting")
        acc["expected"][phase] += expected
        acc["executed"][phase] += executed
        acc["free"][phase] += int(free or 0)
    native = int(record.get("native_collision_elements", 0))
    collision_ne = native == 0 or generic._collision_status(record) in {
        "N/E",
        "NE",
        "BLOCKED",
        "NO_NATIVE_COLLISION",
        "NO_COLLISION_GEOMETRY",
    }
    if not collision_ne:
        acc["collision_assets"] += 1
    measured = bool(record.get("measurement_complete")) and not collision_ne
    acc["measured_assets"] += int(measured)
    for key in (
        "rest_all_pair_cf",
        "rest_non_adjacent_cf",
        "single_joint_sweep_cf",
        "multi_joint_sobol_cf",
        "strict_collision_pass",
    ):
        value = generic._bool(record, key)
        if value and collision_ne:
            raise CheckError(f"asset {ordinal} publishes a collision pass while N/E")
        acc["passes"][key] += int(bool(value))
    maximum = record.get("max_penetration_normalized")
    if maximum is not None:
        value = float(maximum)
        if not math.isfinite(value) or value < 0:
            raise CheckError(f"asset {ordinal} has invalid normalized penetration")
        acc["max_values"].append(value)


def _expected_metrics(acc: Mapping[str, Any], n_eval: int) -> dict[str, Any]:
    metrics = {
        key: {
            "numerator": int(acc["passes"][key]),
            "denominator": n_eval,
        }
        for key in (
            "rest_all_pair_cf",
            "rest_non_adjacent_cf",
            "single_joint_sweep_cf",
            "multi_joint_sobol_cf",
            "strict_collision_pass",
        )
    }
    total_expected = sum(int(acc["expected"][phase]) for phase in ("rest", "single", "sobol"))
    total_executed = sum(int(acc["executed"][phase]) for phase in ("rest", "single", "sobol"))
    total_free = sum(int(acc["free"][phase]) for phase in ("rest", "single", "sobol"))
    if int(acc["collision_assets"]) == 0:
        for key in metrics:
            metrics[key] = {"status": "N/E"}
        metrics["collision_state_rate"] = {"status": "N/E"}
        metrics["collision_free_range"] = {"status": "N/E"}
    else:
        metrics["collision_state_rate"] = {
            "numerator": total_expected - total_free,
            "denominator": total_expected,
            "executed_states": total_executed,
            "unexecuted_states": total_expected - total_executed,
        }
        metrics["collision_free_range"] = {
            "numerator": int(acc["free"]["single"]),
            "denominator": int(acc["expected"]["single"]),
        }
    metrics["max_penetration"] = {
        "status": (
            "N/E"
            if int(acc["collision_assets"]) == 0 or not acc["max_values"]
            else ("COMPLETE" if int(acc["measured_assets"]) == n_eval else "PARTIAL")
        ),
        "maximum_observed_normalized": (
            max(acc["max_values"])
            if acc["max_values"] and int(acc["collision_assets"])
            else None
        ),
        "observed_assets": len(acc["max_values"]),
        "measured_assets": int(acc["measured_assets"]),
        "denominator": n_eval,
    }
    metrics["aor"] = {"status": "N/E"}
    return metrics


def _compare_metrics(
    published: Mapping[str, Any], expected: Mapping[str, Any]
) -> None:
    for key, wanted in expected.items():
        observed = published.get(key)
        if not isinstance(observed, Mapping):
            raise CheckError(f"summary metric is missing: {key}")
        if str(wanted.get("status", "")).upper() in {"N/E", "NE"}:
            if str(observed.get("status", "")).upper() not in {"N/E", "NE"}:
                raise CheckError(f"summary metric must be N/E: {key}")
            continue
        expected_pair = generic._metric_pair(wanted)
        observed_pair = generic._metric_pair(observed)
        if expected_pair is not None and observed_pair != expected_pair:
            raise CheckError(f"summary metric fraction mismatch: {key}")
        for field in (
            "executed_states",
            "unexecuted_states",
            "maximum_observed_normalized",
            "observed_assets",
            "measured_assets",
        ):
            if field in wanted and observed.get(field) != wanted[field]:
                raise CheckError(f"summary metric mismatch: {key}.{field}")


def check(output: Path) -> dict[str, Any]:
    output = Path(output).resolve(strict=True)
    hash_cache: dict[Path, str] = {}
    receipt = _load_json(output / "full_release_receipt.json")
    if receipt.get("schema_version") != RECEIPT_SCHEMA:
        raise CheckError("receipt schema mismatch")
    _require_self_hash(receipt, "receipt_content_sha256", "receipt")
    manifest = _load_json(output / "manifest.json")
    summary = _load_json(output / "summary.json")
    checkpoint = _load_json(output / "checkpoint.json")
    if manifest.get("schema_version") != RUN_SCHEMA:
        raise CheckError("manifest schema mismatch")
    _require_self_hash(manifest, "manifest_content_sha256", "manifest")
    _require_self_hash(summary, "summary_content_sha256", "summary")
    _require_self_hash(checkpoint, "checkpoint_content_sha256", "checkpoint")
    for value, label in ((receipt, "receipt"), (manifest, "manifest"), (summary, "summary")):
        if value.get("sampling_protocol") != SAMPLING_PROTOCOL:
            raise CheckError(f"{label} sampling protocol mismatch")
    if manifest.get("protocol_id") != PROTOCOL_ID or receipt.get("protocol_id") != PROTOCOL_ID:
        raise CheckError("protocol id mismatch")
    n_eval = int(manifest["N_eval"])
    j_eval = int(manifest["J_eval"])
    if (int(receipt["N_eval"]), int(receipt["J_eval"])) != (n_eval, j_eval):
        raise CheckError("receipt N/J mismatch")
    if receipt.get("manifest_sha256") != _cached_sha256(
        output / "manifest.json", hash_cache
    ):
        raise CheckError("receipt manifest hash mismatch")
    if receipt.get("summary_sha256") != _cached_sha256(
        output / "summary.json", hash_cache
    ):
        raise CheckError("receipt summary hash mismatch")
    if receipt.get("records_sha256") != _cached_sha256(
        output / "records.jsonl", hash_cache
    ):
        raise CheckError("receipt records hash mismatch")
    if receipt.get("state_records_sha256") != _cached_sha256(
        output / "state_records.jsonl", hash_cache
    ):
        raise CheckError("receipt state-record hash mismatch")
    if receipt.get("result_database_sha256") != _cached_sha256(
        output / "results.sqlite3", hash_cache
    ):
        raise CheckError("receipt result-database hash mismatch")
    if _cached_sha256(
        output / "asset_records.jsonl", hash_cache
    ) != receipt["records_sha256"]:
        raise CheckError("asset-record alias differs from canonical records")
    _artifact_closure(output, receipt, hash_cache)
    source_context, source, roster, source_database, source_database_sha256 = (
        _source_context(manifest, n_eval)
    )
    result_database = output / "results.sqlite3"
    result_database_sha256 = receipt.get("result_database_sha256")
    try:
        package_root_context = _package_root_context(
            manifest, receipt, roster, source_context
        )
        result = _connect_sealed_immutable(
            result_database,
            result_database_sha256,
        )
    except BaseException:
        source.close()
        raise
    try:
        meta = _read_meta(result)
        expected_meta = {
            "schema_version": RESULT_DB_SCHEMA,
            "manifest_content_sha256": manifest["manifest_content_sha256"],
            "source_receipt_content_sha256": source_context[
                "source_receipt_content_sha256"
            ],
            "source_result_database_declared_sha256": source_context[
                "source_result_database_declared_sha256"
            ],
            "roster_manifest_content_sha256": source_context[
                "roster_manifest_content_sha256"
            ],
            "N_eval": n_eval,
            "J_eval": j_eval,
            "limit": manifest.get("limit"),
            "sampling_protocol": SAMPLING_PROTOCOL,
            "protocol_id": PROTOCOL_ID,
            "package_root_binding_content_sha256": (
                package_root_context["binding"]["binding_content_sha256"]
                if package_root_context is not None
                else None
            ),
        }
        for key, value in expected_meta.items():
            if meta.get(key) != value:
                raise CheckError(f"result database meta mismatch: {key}")
        source_cursor = source.execute(
            "SELECT ordinal, asset_id, category, joint_count, row_sha256, row_json "
            "FROM assets WHERE ordinal < ? ORDER BY ordinal",
            (n_eval,),
        )
        result_cursor = result.execute(
            "SELECT ordinal, asset_id, record_json, states_zlib, state_count, "
            "worker_status, worker_seconds FROM results ORDER BY ordinal"
        )
        aggregate = _new_aggregate()
        total_states = 0
        with (output / "records.jsonl").open("rb") as record_stream, (
            output / "state_records.jsonl"
        ).open("rb") as state_stream:
            for ordinal in range(n_eval):
                source_values = source_cursor.fetchone()
                result_values = result_cursor.fetchone()
                if source_values is None or result_values is None:
                    raise CheckError(f"database ended before ordinal {ordinal}")
                source_ordinal, source_id, category, source_joints, row_hash, row_json = source_values
                result_ordinal, result_id, record_json, blob, state_count, worker_status, worker_seconds = result_values
                if int(source_ordinal) != ordinal or int(result_ordinal) != ordinal:
                    raise CheckError(f"non-contiguous ordinal at {ordinal}")
                if str(source_id) != str(result_id):
                    raise CheckError(f"source/result identity mismatch at {ordinal}")
                row = json.loads(row_json)
                if not isinstance(row, dict) or _canonical_sha256(row) != str(row_hash):
                    raise CheckError(f"source row hash mismatch at {ordinal}")
                if (
                    int(row.get("ordinal", -1)) != ordinal
                    or str(row.get("asset_id")) != str(source_id)
                    or int(row.get("joint_count", -1)) != int(source_joints)
                    or str(row.get("raw_category", row.get("category", ""))) != str(category)
                ):
                    raise CheckError(f"source row binding mismatch at {ordinal}")
                record_line = record_stream.readline()
                if not record_line or record_line.rstrip(b"\n") != str(record_json).encode("utf-8"):
                    raise CheckError(f"record JSONL/database mismatch at {ordinal}")
                record = json.loads(record_json)
                if not isinstance(record, dict):
                    raise CheckError(f"record is not an object at {ordinal}")
                mirror_paths = (
                    _verified_mirror_paths(row, package_root_context, ordinal)
                    if package_root_context is not None
                    else None
                )
                plan = _plan_metadata(
                    row,
                    urdf_path=(
                        Path(mirror_paths["evaluation_urdf"])
                        if mirror_paths is not None
                        else None
                    ),
                )
                for field in (
                    "independent_dof_count",
                    "range_evaluable_independent_dof_count",
                    "mimic_joint_count",
                    "fixed_root_joint_count",
                    "joint_sampling_plan_sha256",
                    "sampling_plan_error",
                ):
                    observed = (
                        record.get(field, 0)
                        if field == "fixed_root_joint_count"
                        else record.get(field)
                    )
                    if observed != plan.get(field):
                        raise CheckError(f"asset {ordinal} sampling plan mismatch: {field}")
                if (
                    str(record.get("dataset_id")) != str(source_id)
                    or int(record.get("order", -1)) != ordinal
                    or int(record.get("expected_movable_joints", -1)) != int(source_joints)
                    or record.get("expected_primary_urdf_sha256") != row.get(
                        "primary_urdf_sha256"
                    )
                    or record.get("package_binding_sha256") != row.get(
                        "package_binding_sha256"
                    )
                ):
                    raise CheckError(f"asset {ordinal} source identity mismatch")
                if record.get("input_identity_sha256") != _expected_input_identity(row, plan):
                    raise CheckError(f"asset {ordinal} input identity mismatch")
                if mirror_paths is not None:
                    binding = package_root_context["binding"]
                    binding_hash = binding["binding_content_sha256"]
                    expected_execution = _canonical_sha256(
                        {
                            "input_identity_sha256": _expected_input_identity(
                                row, plan
                            ),
                            "package_root_binding_content_sha256": binding_hash,
                            "evaluation_package_relative_path": mirror_paths[
                                "evaluation_package_relative_path"
                            ],
                            "evaluation_urdf_relative_path": mirror_paths[
                                "evaluation_urdf_relative_path"
                            ],
                            "package_binding_sha256": row.get(
                                "package_binding_sha256"
                            ),
                            "expected_primary_urdf_sha256": row.get(
                                "primary_urdf_sha256"
                            ),
                        }
                    )
                    expected_record_fields = {
                        "package": str(mirror_paths["logical_package"]),
                        "urdf_path": str(mirror_paths["logical_urdf"]),
                        "evaluation_package_relative_path": mirror_paths[
                            "evaluation_package_relative_path"
                        ],
                        "evaluation_urdf_relative_path": mirror_paths[
                            "evaluation_urdf_relative_path"
                        ],
                        "package_root_binding_content_sha256": binding_hash,
                        "execution_input_sha256": expected_execution,
                        "package_binding_verified": True,
                    }
                    for field, expected in expected_record_fields.items():
                        if record.get(field) != expected:
                            raise CheckError(
                                f"asset {ordinal} mirror binding mismatch: {field}"
                            )
                    if (
                        "evaluation_package_path" in record
                        or "evaluation_urdf_path" in record
                    ):
                        raise CheckError(
                            f"asset {ordinal} publishes a physical mirror path"
                        )
                try:
                    states_payload = zlib.decompress(blob)
                except zlib.error as error:
                    raise CheckError(f"asset {ordinal} state decompression failed") from error
                states = _state_rows(states_payload, int(state_count), ordinal)
                if state_stream.read(len(states_payload)) != states_payload:
                    raise CheckError(f"asset {ordinal} canonical state stream mismatch")
                _validate_states(record, states, ordinal, plan)
                _accumulate(aggregate, record, ordinal)
                total_states += len(states)
                if float(worker_seconds) < 0 or not str(worker_status):
                    raise CheckError(f"asset {ordinal} worker receipt is invalid")
            if source_cursor.fetchone() is not None or result_cursor.fetchone() is not None:
                raise CheckError("database contains rows beyond N_eval")
            if record_stream.read(1) or state_stream.read(1):
                raise CheckError("canonical JSONL stream contains trailing rows")
    finally:
        result.close()
        source.close()
    _verify_sealed_database(source_database, source_database_sha256)
    _verify_sealed_database(result_database, result_database_sha256)
    if aggregate["declared"] != j_eval:
        raise CheckError("declared DoF total does not equal J_eval")
    expected_states = {
        phase: int(aggregate["expected"][phase])
        for phase in ("rest", "single", "sobol")
    }
    executed_states = {
        phase: int(aggregate["executed"][phase])
        for phase in ("rest", "single", "sobol")
    }
    if (
        int(aggregate["collision_assets"]) == 0
        and dict(aggregate["status_counts"]) == {"blocked": n_eval}
    ):
        expected_status = "BLOCKED"
    elif dict(aggregate["status_counts"]) == {"completed": n_eval}:
        expected_status = "COMPLETE"
    else:
        expected_status = "COMPLETE_WITH_RETAINED_FAILURES"
    expected_summary_fields = {
        "n_eval": n_eval,
        "j_eval": j_eval,
        "declared_dof_count": j_eval,
        "independent_dof_count": int(aggregate["independent"]),
        "range_evaluable_independent_dof_count": int(
            aggregate["range_independent"]
        ),
        "mimic_joint_count": int(aggregate["mimic"]),
        "category_count": len(aggregate["categories"]),
        "state_records_executed": total_states,
        "sampling_protocol": SAMPLING_PROTOCOL,
        "expected_states": expected_states,
        "executed_states": executed_states,
        "status_counts": dict(sorted(aggregate["status_counts"].items())),
        "status": expected_status,
    }
    if aggregate["fixed_roots"] or "fixed_root_joint_count" in summary:
        expected_summary_fields["fixed_root_joint_count"] = int(
            aggregate["fixed_roots"]
        )
    for key, value in expected_summary_fields.items():
        if summary.get(key) != value:
            raise CheckError(f"summary field mismatch: {key}")
    published_metrics = summary.get("metrics")
    if not isinstance(published_metrics, Mapping):
        raise CheckError("summary metrics are missing")
    _compare_metrics(published_metrics, _expected_metrics(aggregate, n_eval))
    if receipt.get("metrics") != published_metrics or receipt.get("status") != summary.get(
        "status"
    ):
        raise CheckError("receipt summary duplication mismatch")
    if checkpoint.get("state") != "complete" or int(
        checkpoint.get("records", -1)
    ) != n_eval:
        raise CheckError("checkpoint is incomplete")
    if (
        checkpoint.get("records_sha256") != receipt.get("records_sha256")
        or checkpoint.get("state_records_sha256")
        != receipt.get("state_records_sha256")
        or checkpoint.get("manifest_content_sha256")
        != manifest.get("manifest_content_sha256")
    ):
        raise CheckError("checkpoint binding mismatch")
    return {
        "schema_version": "pva_table4_mimic_aware_automation_check_v1",
        "all_pass": True,
        "output": str(output),
        "N_eval": n_eval,
        "J_eval": j_eval,
        "independent_dof_count": int(aggregate["independent"]),
        "mimic_joint_count": int(aggregate["mimic"]),
        "fixed_root_joint_count": int(aggregate["fixed_roots"]),
        "state_records": total_states,
        "status": expected_status,
        "metrics": published_metrics,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--json-out", type=Path)
    args = parser.parse_args(argv)
    try:
        report = check(args.output)
    except Exception as error:  # noqa: BLE001
        report = {
            "schema_version": "pva_table4_mimic_aware_automation_check_v1",
            "all_pass": False,
            "output": str(args.output.resolve()),
            "errors": [f"{type(error).__name__}: {error}"],
        }
    payload = json.dumps(report, indent=2, sort_keys=True, ensure_ascii=True) + "\n"
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(payload, encoding="utf-8")
    print(payload, end="")
    return 0 if report["all_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
