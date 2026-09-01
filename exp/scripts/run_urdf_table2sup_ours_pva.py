#!/usr/bin/env python3
"""Formal Table 2 supplementary audit for the frozen PV-A/Ours cohort.

This adapter owns only the PV-A cohort and provenance contract.  The four
supplementary atoms are evaluated by ``table2_supplementary_static`` and the
receipt/aggregation helpers are reused from the established Ours runner.
Every selected package is checked against the frozen per-file binding before
and after evaluation; failures remain in the asset and joint denominators.
"""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
import datetime as dt
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import platform
import shutil
import sys
import time
from typing import Any, Mapping


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

DEFAULT_INPUT = REPO / "exp/PV-A-per-class-n5-max-joints/manifest.json"
DEFAULT_TABLE1_RECEIPT = REPO / "exp/runtime/table1_pva_per_class_n5_max_joints"
DEFAULT_PROTOCOL = REPO / "exp/URDF-Sim-Ready-Automatic-Evaluation.md"
PVA_ROOT = REPO / "exp/PV-A-per-class-n5"
PVA_ROOTS = (
    REPO / "exp/PV-A-per-class-n5",
    REPO / "exp/PV-A-per-class-n5-max-joints",
)
TABLE2_BASE_PATH = REPO / "exp/scripts/run_urdf_table2sup_ours_500k.py"
STATIC_PATH = REPO / "exp/scripts/table2_supplementary_static.py"
VERIFIER_PATH = REPO / "exp/scripts/verify_table2_supplementary_v1.py"

PROTOCOL_ID = "table2_supplementary_ours_pva_per_class_n5_max_joints_v1"
DATASET = "PV-A-per-class-n5-max-joints"
SCHEMA_VERSION = "table2-supplementary-ours-pva/v1"
COHORT_TYPE = "CATEGORY_STRATIFIED_N5_WITH_FENCE_FERRIS_MAX_JOINT_OVERRIDES"
EXPECTED_N = 2_655
EXPECTED_CATEGORIES = 531
PER_CLASS = 5
N_RELEASE = 302_440
EXPECTED_J_EVAL = 14_968
ASSET_TIMEOUT_SECONDS = 900.0
DEFAULT_WORKERS = 16

EXPECTED_SOURCE_MANIFEST_SHA256 = (
    "e78f4b767023f8a5c1517d96bfab35a39482d6eee28238820a9b91ac3ea8d293"
)
EXPECTED_SOURCE_CONTENT_SHA256 = (
    "eea55287dd70b710a7c03b11b16c6685208bbaa63cde925232293cb9012c8158"
)
EXPECTED_TABLE1_MANIFEST_SHA256 = (
    "4b0360a398a5efba3532e9ca87c37bbedc5a3416783679f9f26f89e930e19644"
)
EXPECTED_TABLE1_RECORDS_SHA256 = (
    "8ccb7ef5e34545b72865396d1f71ca6fcb368e16f3fcab9f13820c985d488b05"
)
EXPECTED_TABLE1_SUMMARY_SHA256 = (
    "c61270cb14ba83b0ed18e35e62c4b4a982c4cf465751516f91f5384736f9fad7"
)
EXPECTED_TABLE1_ARTIFACT_MANIFEST_SHA256 = (
    "eabcc9bb7dc88e2c210577a73e91f6cc47bd088926e23e1190082323389ded1f"
)

PLACEHOLDER_REGISTRY: list[dict[str, Any]] = []
PLACEHOLDER_REGISTRY_RATIONALE = (
    "frozen empty: no validated PV-A placeholder-mass registry; incidence is N/E "
    "while complete-inertial coverage remains reported"
)

INPUT_IDENTITY_FIELDS = (
    "selection_index",
    "asset_id",
    "asset_root",
    "raw_category",
    "seed_name",
    "selection_rank",
    "package",
    "primary_urdf_relative_path",
    "expected_declared_joint_count",
    "model_urdf_sha256_expected",
    "package_content_manifest_sha256_expected",
    "package_binding_files_expected",
)


def _load_module(path: Path, name: str):
    existing = sys.modules.get(name)
    if existing is not None:
        return existing
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load module: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


# The base runner supplies the proven timeout-safe worker pool, child receipt
# schema, aggregation, and record writer.  Its cohort constants are replaced
# below; no Ours-500K input is read by this adapter.
BASE = _load_module(TABLE2_BASE_PATH, "table2sup_ours_pva_static_shared")
STATIC = _load_module(STATIC_PATH, "table2sup_ours_pva_static_atoms")
VERIFIER = _load_module(VERIFIER_PATH, "table2sup_ours_pva_verifier")

# Configure the shared static helpers for this adapter's protocol identity.
BASE.SCRIPT = SCRIPT
BASE.OURS_ROOT = PVA_ROOT
BASE.PROTOCOL_ID = PROTOCOL_ID
BASE.DATASET = DATASET
BASE.SCHEMA_VERSION = SCHEMA_VERSION
BASE.PLACEHOLDER_REGISTRY = PLACEHOLDER_REGISTRY
BASE.PLACEHOLDER_REGISTRY_RATIONALE = PLACEHOLDER_REGISTRY_RATIONALE


def canonical_json(value: Any) -> str:
    return json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    )


def canonical_sha256(value: Any) -> str:
    return hashlib.sha256(canonical_json(value).encode("utf-8")).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def package_binding(package: Path) -> dict[str, Any]:
    """Return the frozen PV-A per-file package binding without optional deps."""

    package = package.resolve(strict=True)
    files: list[dict[str, Any]] = []
    for current_raw, directory_names, file_names in os.walk(package, followlinks=False):
        current = Path(current_raw)
        directory_names.sort()
        file_names.sort()
        for name in directory_names:
            if (current / name).is_symlink():
                raise ValueError(f"package contains directory symlink: {current / name}")
        for name in file_names:
            path = current / name
            if path.is_symlink() or not path.is_file():
                raise ValueError(f"package contains non-regular file: {path}")
            files.append(
                {"path": path.relative_to(package).as_posix(), "bytes": path.stat().st_size, "sha256": sha256_file(path)}
            )
    return {
        "file_count": len(files),
        "total_bytes": sum(row["bytes"] for row in files),
        "files": files,
        "content_manifest_sha256": canonical_sha256(files),
    }


def atomic_write_text(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(value, encoding="utf-8")
    temporary.replace(path)


def atomic_json(path: Path, value: Any) -> None:
    atomic_write_text(path, canonical_json(value) + "\n")


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def timestamp_tag() -> str:
    return dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def input_identity(item: Mapping[str, Any]) -> str:
    return canonical_sha256({field: item[field] for field in INPUT_IDENTITY_FIELDS})


@dataclass(frozen=True)
class FrozenInputs:
    source_path: Path
    source_manifest: dict[str, Any]
    source_sha256: str
    source_content_sha256: str
    table1_path: Path
    table1_manifest: dict[str, Any]
    table1_sha256: str
    table1_records_sha256: str
    table1_summary_sha256: str
    table1_artifact_manifest_sha256: str
    items: list[dict[str, Any]]
    category_counts: dict[str, int]
    j_eval: int

    @property
    def categories(self) -> tuple[str, ...]:
        return tuple(self.category_counts)


class ProtocolViolation(RuntimeError):
    """Raised when a frozen input or live package binding is invalid."""


def _load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _check_artifact_manifest(receipt: Path) -> None:
    artifact_path = receipt / "artifact_manifest.json"
    if sha256_file(artifact_path) != EXPECTED_TABLE1_ARTIFACT_MANIFEST_SHA256:
        raise ProtocolViolation("Table 1 artifact_manifest.json byte hash mismatch")
    artifact = _load_json(artifact_path)
    expected_names = {
        "manifest.json",
        "asset_records.jsonl",
        "summary.json",
        "report.md",
        "timing.json",
    }
    files = artifact.get("files")
    if set(files or {}) != expected_names:
        raise ProtocolViolation("Table 1 artifact file set changed")
    for name, expected in files.items():
        path = receipt / name
        if not path.is_file() or path.stat().st_size != expected.get("bytes"):
            raise ProtocolViolation(f"Table 1 artifact size mismatch: {name}")
        if sha256_file(path) != expected.get("sha256"):
            raise ProtocolViolation(f"Table 1 artifact hash mismatch: {name}")


def _load_table1_records(receipt: Path) -> tuple[list[dict[str, Any]], int]:
    records_path = receipt / "asset_records.jsonl"
    if sha256_file(records_path) != EXPECTED_TABLE1_RECORDS_SHA256:
        raise ProtocolViolation("Table 1 asset_records.jsonl byte hash mismatch")
    records: list[dict[str, Any]] = []
    seen: set[str] = set()
    with records_path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream):
            if not line.strip():
                continue
            row = _load_json_line(line)
            asset_id = str(row.get("asset_id", ""))
            if not asset_id or asset_id in seen:
                raise ProtocolViolation(f"duplicate Table 1 asset record: {asset_id!r}")
            if row.get("status") != "EVALUATED":
                raise ProtocolViolation(f"Table 1 record is not evaluated: {asset_id}")
            count = row.get("non_fixed_joint_count")
            if isinstance(count, bool) or not isinstance(count, int) or count < 0:
                raise ProtocolViolation(
                    f"invalid non_fixed_joint_count at Table 1 record line {line_number + 1}"
                )
            seen.add(asset_id)
            records.append(row)
    if len(records) != EXPECTED_N:
        raise ProtocolViolation(f"Table 1 records cover {len(records)} assets, expected {EXPECTED_N}")
    records.sort(key=lambda row: int(row.get("selection_index", -1)))
    if [row.get("selection_index") for row in records] != list(range(EXPECTED_N)):
        raise ProtocolViolation("Table 1 record selection indices are not contiguous")
    total = sum(int(row["non_fixed_joint_count"]) for row in records)
    if total != EXPECTED_J_EVAL:
        raise ProtocolViolation(f"Table 1 J_eval {total} != frozen {EXPECTED_J_EVAL}")
    return records, total


def _load_json_line(line: str) -> dict[str, Any]:
    row = json.loads(line)
    if not isinstance(row, dict):
        raise ProtocolViolation("JSONL row is not an object")
    return row


def _validate_source_manifest(path: Path) -> tuple[dict[str, Any], str]:
    observed_sha = sha256_file(path)
    if observed_sha != EXPECTED_SOURCE_MANIFEST_SHA256:
        raise ProtocolViolation(
            f"source cohort byte hash mismatch: {observed_sha} != {EXPECTED_SOURCE_MANIFEST_SHA256}"
        )
    manifest = _load_json(path)
    body = {key: value for key, value in manifest.items() if key != "manifest_content_sha256"}
    if manifest.get("manifest_content_sha256") != EXPECTED_SOURCE_CONTENT_SHA256:
        raise ProtocolViolation("source cohort declared content hash changed")
    if canonical_sha256(body) != EXPECTED_SOURCE_CONTENT_SHA256:
        raise ProtocolViolation("source cohort canonical content hash mismatch")
    if (
        manifest.get("schema_version") != "pva-per-class-extracted-cohort/v2"
        or manifest.get("protocol_id") != "pva-per-class-n5-fence-ferris-max-movable-joints-v1"
        or manifest.get("dataset") != "PV-A-per-class-n5"
        or manifest.get("n_eval") != EXPECTED_N
        or manifest.get("class_count") != EXPECTED_CATEGORIES
        or manifest.get("per_class") != PER_CLASS
    ):
        raise ProtocolViolation("source cohort protocol/layout changed")
    assets = manifest.get("assets")
    if not isinstance(assets, list) or len(assets) != EXPECTED_N:
        raise ProtocolViolation("source cohort asset count changed")
    return manifest, observed_sha


def load_frozen_inputs(
    *,
    source_path: Path = DEFAULT_INPUT,
    table1_receipt: Path = DEFAULT_TABLE1_RECEIPT,
    validate_packages: bool = False,
) -> FrozenInputs:
    """Load and cross-check the immutable source cohort and Table 1 receipt."""

    source_path = source_path.resolve(strict=True)
    table1_receipt = table1_receipt.resolve(strict=True)
    source, source_sha = _validate_source_manifest(source_path)
    table1_path = table1_receipt / "manifest.json"
    table1_sha = sha256_file(table1_path)
    if table1_sha != EXPECTED_TABLE1_MANIFEST_SHA256:
        raise ProtocolViolation("Table 1 manifest byte hash mismatch")
    table1 = _load_json(table1_path)
    if (
        table1.get("schema_version") != "table1_pva_manifest_v1"
        or table1.get("dataset") != DATASET
        or table1.get("N_release") != N_RELEASE
        or table1.get("N_eval") != EXPECTED_N
        or table1.get("source_manifest_sha256") != source_sha
        or table1.get("source_manifest_content_sha256") != EXPECTED_SOURCE_CONTENT_SHA256
    ):
        raise ProtocolViolation("Table 1 receipt provenance or denominator changed")
    table1_assets = table1.get("assets")
    source_assets = source.get("assets")
    if not isinstance(table1_assets, list) or len(table1_assets) != EXPECTED_N:
        raise ProtocolViolation("Table 1 manifest asset count changed")
    if [row.get("selection_index") for row in table1_assets] != list(range(EXPECTED_N)):
        raise ProtocolViolation("Table 1 manifest order changed")
    if sha256_file(table1_receipt / "summary.json") != EXPECTED_TABLE1_SUMMARY_SHA256:
        raise ProtocolViolation("Table 1 summary byte hash mismatch")
    summary = _load_json(table1_receipt / "summary.json")
    if summary.get("status_counts") != {"EVALUATED": EXPECTED_N}:
        raise ProtocolViolation("Table 1 summary has non-evaluated assets")
    if summary.get("cohort", {}).get("N_parse") != EXPECTED_N:
        raise ProtocolViolation("Table 1 parse denominator changed")
    _check_artifact_manifest(table1_receipt)
    records, j_eval = _load_table1_records(table1_receipt)
    records_by_id = {str(row["asset_id"]): row for row in records}
    category_counts: Counter[str] = Counter()
    items: list[dict[str, Any]] = []
    seen_ids: set[str] = set()

    for index, (source_row, table1_row) in enumerate(zip(source_assets, table1_assets)):
        if not isinstance(source_row, dict) or not isinstance(table1_row, dict):
            raise ProtocolViolation(f"non-object asset row at index {index}")
        dataset_id = str(source_row.get("dataset_id", ""))
        if not dataset_id or dataset_id in seen_ids:
            raise ProtocolViolation(f"invalid or duplicate dataset_id: {dataset_id!r}")
        seen_ids.add(dataset_id)
        category = str(source_row.get("category", ""))
        category_counts[category] += 1
        if (
            table1_row.get("dataset_id") != dataset_id
            or table1_row.get("asset_id") != source_row.get("asset_id")
        ):
            raise ProtocolViolation(f"Table 1 identity mismatch at index {index}")
        record = records_by_id.get(dataset_id)
        if record is None or record.get("selection_index") != index:
            raise ProtocolViolation(f"Table 1 record order mismatch at {dataset_id}")
        for field in ("package", "package_binding"):
            if table1_row.get(field) != source_row.get(field):
                raise ProtocolViolation(f"{field} mismatch at {dataset_id}")
        source_urdf_sha = str(source_row.get("urdf_sha256", ""))
        if table1_row.get("primary_urdf_sha256") != source_urdf_sha:
            raise ProtocolViolation(f"Table 1 URDF hash mismatch at {dataset_id}")
        if record.get("package") != source_row.get("package"):
            raise ProtocolViolation(f"Table 1 record package mismatch at {dataset_id}")
        if record.get("primary_urdf_sha256") != source_urdf_sha:
            raise ProtocolViolation(f"Table 1 record URDF hash mismatch at {dataset_id}")
        package = Path(str(source_row.get("package", "")))
        binding = source_row.get("package_binding")
        if not package.is_absolute() or not isinstance(binding, dict):
            raise ProtocolViolation(f"invalid package binding fields at {dataset_id}")
        item = {
            "selection_index": index,
            "asset_id": dataset_id,
            "asset_root": str(package),
            "raw_category": category,
            "seed_name": str(source_row.get("asset_id", "")),
            "selection_rank": index + 1,
            "package": str(package),
            "primary_urdf_relative_path": str(source_row.get("primary_urdf_relative_path", "model.urdf")),
            "expected_declared_joint_count": int(record["non_fixed_joint_count"]),
            "model_urdf_sha256_expected": source_urdf_sha,
            "package_content_manifest_sha256_expected": str(binding.get("content_manifest_sha256", "")),
            "package_binding_files_expected": binding.get("files"),
            "package_binding_expected": binding,
        }
        item["input_identity_sha256"] = input_identity(item)
        items.append(item)

    if len(category_counts) != EXPECTED_CATEGORIES or set(category_counts.values()) != {PER_CLASS}:
        raise ProtocolViolation("source category stratification changed")
    frozen = FrozenInputs(
        source_path=source_path,
        source_manifest=source,
        source_sha256=source_sha,
        source_content_sha256=EXPECTED_SOURCE_CONTENT_SHA256,
        table1_path=table1_path,
        table1_manifest=table1,
        table1_sha256=table1_sha,
        table1_records_sha256=EXPECTED_TABLE1_RECORDS_SHA256,
        table1_summary_sha256=EXPECTED_TABLE1_SUMMARY_SHA256,
        table1_artifact_manifest_sha256=EXPECTED_TABLE1_ARTIFACT_MANIFEST_SHA256,
        items=items,
        category_counts=dict(sorted(category_counts.items())),
        j_eval=j_eval,
    )
    if validate_packages:
        for item in frozen.items:
            result = verify_binding(item)
            if not result["verified"]:
                raise ProtocolViolation(
                    f"package binding failed for {item['asset_id']}: {'; '.join(result['issues'])}"
                )
    return frozen


def verify_binding(item: Mapping[str, Any]) -> dict[str, Any]:
    """Recompute exact package and primary-URDF bindings for one item."""

    package_raw = str(item.get("package", ""))
    package = Path(package_raw)
    issues: list[str] = []
    observed_binding: dict[str, Any] | None = None
    observed_urdf: str | None = None
    if not package.is_absolute():
        issues.append("package_not_absolute")
    elif package.is_symlink():
        issues.append("package_is_symlink")
    elif not package.is_dir():
        issues.append("package_missing")
    else:
        try:
            resolved = package.resolve(strict=True)
            if not any(
                _is_relative_to(resolved, root.resolve(strict=True)) for root in PVA_ROOTS
            ):
                raise ValueError("package is outside allowed PV-A roots")
        except (OSError, ValueError):
            issues.append("package_escapes_pva_root")
        else:
            try:
                observed_binding = package_binding(resolved)
            except Exception as exc:  # noqa: BLE001
                issues.append(f"package_binding_error:{type(exc).__name__}:{exc}")
            urdf_relative = str(item.get("primary_urdf_relative_path", ""))
            urdf = resolved / urdf_relative
            if urdf_relative != "model.urdf":
                issues.append("primary_urdf_path_not_model_urdf")
            elif not urdf.is_file() or urdf.is_symlink():
                issues.append("primary_urdf_missing")
            else:
                observed_urdf = sha256_file(urdf)
                if observed_urdf != item.get("model_urdf_sha256_expected"):
                    issues.append(
                        "model_urdf_sha256_mismatch: expected "
                        f"{item.get('model_urdf_sha256_expected')}, observed {observed_urdf}"
                    )
    expected_binding = item.get("package_binding_expected")
    if observed_binding is not None and observed_binding != expected_binding:
        issues.append("package_binding_exact_mismatch")
    expected_content = item.get("package_content_manifest_sha256_expected")
    if observed_binding is not None and observed_binding.get("content_manifest_sha256") != expected_content:
        issues.append("package_content_manifest_sha256_mismatch")
    return {
        "verified": not issues,
        "issues": issues,
        "expected_content_manifest_sha256": expected_content,
        "observed_content_manifest_sha256": (
            observed_binding.get("content_manifest_sha256") if observed_binding else None
        ),
        "expected_urdf_sha256": item.get("model_urdf_sha256_expected"),
        "observed_urdf_sha256": observed_urdf,
        "verified_at_utc": utc_now(),
    }


def _is_relative_to(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
    except ValueError:
        return False
    return True


def _configure_base() -> None:
    BASE.ASSET_TIMEOUT_SECONDS = ASSET_TIMEOUT_SECONDS
    BASE.DEFAULT_WORKERS = DEFAULT_WORKERS
    BASE.PROTOCOL_ID = PROTOCOL_ID
    BASE.DATASET = DATASET
    BASE.SCHEMA_VERSION = SCHEMA_VERSION
    BASE.OURS_ROOT = PVA_ROOT
    BASE.PLACEHOLDER_REGISTRY = PLACEHOLDER_REGISTRY
    BASE.PLACEHOLDER_REGISTRY_RATIONALE = PLACEHOLDER_REGISTRY_RATIONALE


def _write_frozen_manifest(
    output: Path,
    frozen: FrozenInputs,
    items: list[dict[str, Any]],
    *,
    args: argparse.Namespace,
    started_at: str,
) -> dict[str, Any]:
    provenance = {
        "protocol_id": PROTOCOL_ID,
        "dataset": DATASET,
        "schema_version": SCHEMA_VERSION,
        "cohort_type": COHORT_TYPE,
        "selection_policy": "existing frozen source manifest order; no resampling/reselection",
        "source_cohort_manifest_path": str(frozen.source_path),
        "source_cohort_manifest_sha256": frozen.source_sha256,
        "source_cohort_manifest_content_sha256": frozen.source_content_sha256,
        "table1_manifest_path": str(frozen.table1_path),
        "table1_manifest_sha256": frozen.table1_sha256,
        "table1_asset_records_sha256": frozen.table1_records_sha256,
        "table1_summary_sha256": frozen.table1_summary_sha256,
        "table1_artifact_manifest_sha256": frozen.table1_artifact_manifest_sha256,
        "N_release": N_RELEASE,
        "N_eval_full": EXPECTED_N,
        "J_eval_full": EXPECTED_J_EVAL,
        "category_count_full": EXPECTED_CATEGORIES,
        "per_class": PER_CLASS,
        "n_eval": len(items),
        "j_eval": sum(item["expected_declared_joint_count"] for item in items),
        "package_root": str(PVA_ROOT),
        "protocol_document_path": str(DEFAULT_PROTOCOL),
        "protocol_document_sha256": sha256_file(DEFAULT_PROTOCOL),
        "placeholder_mass_registry": PLACEHOLDER_REGISTRY,
        "placeholder_mass_registry_rationale": PLACEHOLDER_REGISTRY_RATIONALE,
        "asset_timeout_seconds": args.asset_timeout_seconds,
        "workers": args.workers,
        "static_evaluator_path": str(STATIC_PATH),
        "static_evaluator_sha256": sha256_file(STATIC_PATH),
        "shared_runner_path": str(TABLE2_BASE_PATH),
        "shared_runner_sha256": sha256_file(TABLE2_BASE_PATH),
        "runner_path": str(SCRIPT),
        "runner_sha256": sha256_file(SCRIPT),
        "command": [sys.executable, *sys.argv],
        "created_at_utc": started_at,
    }
    manifest = {
        "protocol_id": PROTOCOL_ID,
        "schema_version": SCHEMA_VERSION,
        "dataset": DATASET,
        "classification": "CUSTOM_COHORT_EVIDENCE" if args.limit is None else "SMOKE",
        "provenance": provenance,
        "items": items,
    }
    manifest["manifest_content_sha256"] = canonical_sha256(
        {key: value for key, value in manifest.items() if key != "manifest_content_sha256"}
    )
    atomic_json(output / "frozen_manifest.json", manifest)
    return manifest


def _write_environment(output: Path, args: argparse.Namespace) -> None:
    import importlib.metadata

    dependencies = {}
    for name in ("numpy", "urdfpy", "trimesh"):
        try:
            dependencies[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            dependencies[name] = None
    atomic_json(
        output / "environment.json",
        {
            "python": platform.python_version(),
            "python_implementation": platform.python_implementation(),
            "executable": sys.executable,
            "platform": platform.platform(),
            "workers": args.workers,
            "asset_timeout_seconds": args.asset_timeout_seconds,
            "gpu_required": False,
            "dependencies": dependencies,
            "runner_sha256": sha256_file(SCRIPT),
            "static_evaluator_sha256": sha256_file(STATIC_PATH),
            "recorded_at_utc": utc_now(),
        },
    )


def _write_child_manifest(output: Path, items: list[dict[str, Any]]) -> None:
    files: dict[str, dict[str, Any]] = {}
    for item in items:
        index = int(item["selection_index"])
        path = BASE.child_receipt_path(output, index)
        if not path.is_file():
            raise ProtocolViolation(f"missing child receipt: {path}")
        files[path.relative_to(output).as_posix()] = {
            "bytes": path.stat().st_size,
            "sha256": sha256_file(path),
        }
    atomic_json(
        output / "child_receipt_manifest.json",
        {"schema_version": 1, "files": files, "created_at_utc": utc_now()},
    )


def _verify_artifacts(output: Path, names: tuple[str, ...]) -> dict[str, Any]:
    files: dict[str, dict[str, Any]] = {}
    for name in names:
        path = output / name
        if not path.is_file():
            raise ProtocolViolation(f"missing output artifact: {name}")
        files[name] = {"bytes": path.stat().st_size, "sha256": sha256_file(path)}
    artifact = {"schema_version": 1, "files": files, "created_at_utc": utc_now()}
    atomic_json(output / "artifact_manifest.json", artifact)
    return artifact


def _load_existing_manifest(output: Path) -> dict[str, Any]:
    manifest = _load_json(output / "frozen_manifest.json")
    declared = manifest.get("manifest_content_sha256")
    body = {key: value for key, value in manifest.items() if key != "manifest_content_sha256"}
    if declared != canonical_sha256(body):
        raise ProtocolViolation("existing frozen manifest self-hash is invalid")
    return manifest


def render_summary_md(summary: Mapping[str, Any]) -> str:
    return BASE.render_summary_md(summary)


def run(args: argparse.Namespace) -> dict[str, Any]:
    _configure_base()
    wall_started = time.perf_counter()
    started_at = utc_now()
    output = args.output.resolve()
    if args.resume:
        if not output.is_dir() or not (output / "frozen_manifest.json").is_file():
            raise SystemExit("--resume requires an existing output with frozen_manifest.json")
    elif output.exists():
        raise FileExistsError(f"refusing to overwrite existing output: {output}")
    else:
        output.mkdir(parents=True, exist_ok=False)

    input_started = time.perf_counter()
    frozen = load_frozen_inputs(
        source_path=args.input_manifest,
        table1_receipt=args.table1_receipt,
        validate_packages=args.validate_packages,
    )
    items = frozen.items[: args.limit] if args.limit is not None else frozen.items
    input_seconds = time.perf_counter() - input_started
    if args.resume:
        manifest = _load_existing_manifest(output)
        existing_ids = [row.get("input_identity_sha256") for row in manifest.get("items", [])]
        if existing_ids != [row["input_identity_sha256"] for row in items]:
            raise ProtocolViolation("frozen cohort identity changed; refusing to resume")
        if not (output / "protocol_snapshot.md").is_file():
            raise ProtocolViolation("protocol snapshot missing; refusing to resume")
    else:
        manifest = _write_frozen_manifest(output, frozen, items, args=args, started_at=started_at)
        atomic_write_text(output / "protocol_snapshot.md", DEFAULT_PROTOCOL.read_text(encoding="utf-8"))
        _write_environment(output, args)

    binding_started = time.perf_counter()
    binding: dict[str, dict[str, Any]] = {}
    for index, item in enumerate(items, start=1):
        result = verify_binding(item)
        binding[item["asset_id"]] = result
        if not result["verified"]:
            print(f"[binding] FAIL {index}/{len(items)} {item['asset_id']}: {'; '.join(result['issues'][:3])}")
    binding_seconds = time.perf_counter() - binding_started
    print(f"[binding] verified {sum(int(row['verified']) for row in binding.values())} / {len(items)}")
    evaluation_started = time.perf_counter()
    receipts = BASE.run_evaluation(output, items, binding, workers=args.workers)
    evaluation_seconds = time.perf_counter() - evaluation_started

    # Detect package changes that occurred while the worker pool was running.
    post_binding_failures: dict[str, Any] = {}
    for item in items:
        result = verify_binding(item)
        if not result["verified"]:
            post_binding_failures[item["asset_id"]] = result
    if post_binding_failures:
        atomic_json(output / "post_binding_failures.json", post_binding_failures)
        raise ProtocolViolation("package binding changed during evaluation")

    j_eval = sum(item["expected_declared_joint_count"] for item in items)
    summary = BASE.aggregate(items, receipts, j_eval)
    summary.update(
        {
            "protocol_id": PROTOCOL_ID,
            "schema_version": SCHEMA_VERSION,
            "dataset": DATASET,
            "classification": manifest["classification"],
            "frozen_manifest_sha256": manifest["manifest_content_sha256"],
            "protocol_snapshot_sha256": sha256_file(output / "protocol_snapshot.md"),
            "cohort": {
                "N_release": N_RELEASE,
                "N_eval": len(items),
                "N_eval_full": EXPECTED_N,
                "J_eval": j_eval,
                "J_eval_full": EXPECTED_J_EVAL,
                "categories": len({item["raw_category"] for item in items}),
                "categories_full": EXPECTED_CATEGORIES,
                "cohort_type": COHORT_TYPE,
                "source_manifest_sha256": frozen.source_sha256,
                "table1_manifest_sha256": frozen.table1_sha256,
            },
        }
    )
    atomic_json(output / "summary.json", summary)
    BASE.write_asset_records(output, items, receipts)
    atomic_write_text(output / "summary.md", render_summary_md(summary))
    _write_child_manifest(output, items)
    timing = {
        "schema_version": "experiment-timing/v1",
        "table": "Table 2 supplementary",
        "protocol_id": PROTOCOL_ID,
        "started_at_utc": started_at,
        "completed_at_utc": utc_now(),
        "wall_time_seconds": time.perf_counter() - wall_started,
        "input_validation_seconds": input_seconds,
        "binding_validation_seconds": binding_seconds,
        "evaluation_seconds": evaluation_seconds,
        "workers": args.workers,
        "asset_timeout_seconds": args.asset_timeout_seconds,
        "n_eval": len(items),
        "j_eval": j_eval,
        "measurement_endpoint": "before_artifact_manifest",
        "command": [sys.executable, *sys.argv],
    }
    atomic_json(output / "timing.json", timing)

    verification = VERIFIER.verify_run(
        output,
        table1_manifest=frozen.table1_path,
        expected_table1_sha256=frozen.table1_sha256,
        identity_fields=INPUT_IDENTITY_FIELDS,
        table1_id_key="dataset_id",
    )
    atomic_json(output / "verification.json", verification)
    artifact_names = (
        "frozen_manifest.json",
        "protocol_snapshot.md",
        "environment.json",
        "summary.json",
        "summary.md",
        "asset_records.jsonl",
        "timing.json",
        "child_receipt_manifest.json",
        "verification.json",
    )
    artifact = _verify_artifacts(output, artifact_names)
    if verification.get("status") != "PASS":
        raise ProtocolViolation("independent supplementary verifier failed")
    return {
        "output": output,
        "summary": summary,
        "timing": timing,
        "verification": verification,
        "artifact_manifest": artifact,
    }


def default_output_root(limit: int | None) -> Path:
    suffix = f"smoke_n{limit}_" if limit is not None else ""
    return REPO / f"exp/runtime/table2sup_urdf_ours_pva_per_class_n5_max_joints_{suffix}{timestamp_tag()}"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-manifest", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--table1-receipt", type=Path, default=DEFAULT_TABLE1_RECEIPT)
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--expected-n", type=int, default=EXPECTED_N)
    parser.add_argument("--workers", type=int, default=DEFAULT_WORKERS)
    parser.add_argument("--asset-timeout-seconds", type=float, default=ASSET_TIMEOUT_SECONDS)
    parser.add_argument("--limit", type=int)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument(
        "--validate-packages",
        action="store_true",
        help="recompute every frozen package binding before evaluation",
    )
    args = parser.parse_args(argv)
    if args.expected_n != EXPECTED_N:
        parser.error(f"--expected-n is frozen at {EXPECTED_N}")
    if args.workers < 1 or args.asset_timeout_seconds <= 0:
        parser.error("workers and asset timeout must be positive")
    if args.limit is not None and (args.limit < 1 or args.limit > EXPECTED_N):
        parser.error(f"--limit must be between 1 and {EXPECTED_N}")
    if args.protocol.resolve() != DEFAULT_PROTOCOL.resolve():
        parser.error("custom protocol path is unsupported; use the frozen protocol path")
    if args.output is None:
        args.output = default_output_root(args.limit)
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    result = run(args)
    metrics = result["summary"]["metrics"]
    print(
        json.dumps(
            {
                "status": "COMPLETE",
                "output": str(result["output"]),
                "n_eval": result["timing"]["n_eval"],
                "j_eval": result["timing"]["j_eval"],
                "wall_time_seconds": result["timing"]["wall_time_seconds"],
                "metrics": metrics,
                "artifact_manifest": str(result["output"] / "artifact_manifest.json"),
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
