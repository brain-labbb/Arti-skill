#!/usr/bin/env python3
"""Independent verifier for an Articraft-10K Supplementary Table S1 run."""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import re
import sys
from typing import Any, Mapping
import xml.etree.ElementTree as ET


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from exp.scripts import lam_supplementary_static as static_atoms  # noqa: E402


SCHEMA_VERSION = "s1-articraft10k-run/v1"
PROTOCOL_ID = "s1_articraft10k_table2cohort_n800_seed20260813_v1"
DATASET = "Articraft-10K"
FORMAL_N_EVAL = 800
FORMAL_SOURCE_MANIFEST_SHA256 = (
    "13c47e2b2affadb951a01cab826bae139852fca5769e99ec081cc916ffa6373d"
)
FORMAL_SOURCE_MANIFEST_CONTENT_SHA256 = (
    "576852cb6da00775e1c51360b82b4be40e0a614e4fb0cfb1bae066912eed56a3"
)
FORMAL_ORDERED_ASSET_IDS_SHA256 = (
    "79c44441600077513d3cde1cda8fef38324e1a0ee660730b860d5313f0ae9784"
)
FORMAL_TABLE4_MANIFEST_SHA256 = (
    "6b4275cf3da29244af70c04acecd87094f0c158dee992db20b04e90c05292c20"
)
FORMAL_TABLE4_ASSET_RECORDS_SHA256 = (
    "b732a53a464a8aeebb74799d5ec737de75f3cca377c9a5b274a5dd35adbe301b"
)
FORMAL_TABLE4_STATE_RECORDS_SHA256 = (
    "6efd4031ecebf74f30f8d3ec3c312ae2faf1b521322b5d4a8b57bb732177ac8b"
)
FORMAL_TABLE4_PROTOCOL_ID = "urdf_sim_ready_table4_articraft10k_n800_v1"
FORMAL_STATIC_ATOMS_SHA256 = (
    "4701415dad8a5c0a434c16887979bcb70c250ba0b25772014e8db73789098e5f"
)
FORMAL_CATEGORY_RECORDS_ROOT = (
    REPO / "exp/baselines/Articraft-10K-official/records"
).resolve()
FORMAL_OFFICIAL_SOURCE_MANIFEST_SHA256 = (
    "11a37014d2d73782f502f2043b589915f663e16094470366529d5e944d777f47"
)
FORMAL_ELIGIBLE_PAIRS = 3040
FORMAL_STRICT_PASSED = 147
RECEIPT_NAME_RE = re.compile(
    r"(?:^|[-_])(?:mechanical[-_])?receipt(?:[-_]|\.|$)", re.IGNORECASE
)
ALLOWANCE_NAME_RE = re.compile(
    r"allow(?:ance|list|ed)|exclu(?:de|sion)", re.IGNORECASE
)
REBUILD_RECIPE_NAMES = frozenset(
    {
        "build_recipe.json",
        "build-recipe.json",
        "rebuild_recipe.json",
        "rebuild-recipe.json",
        "deterministic_rebuild.json",
    }
)
EXPECTED_ARTIFACTS = frozenset(
    {
        "protocol_snapshot.md",
        "frozen_config.json",
        "asset_records.jsonl",
        "summary.json",
        "summary.md",
    }
)


class VerificationError(RuntimeError):
    pass


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(4 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_sha256(value: Any) -> str:
    payload = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def package_content_manifest_sha256(package: Path) -> str:
    package = package.resolve(strict=True)
    if not package.is_dir():
        raise VerificationError(f"package is not a directory: {package}")
    files: list[dict[str, Any]] = []
    for current_raw, directory_names, file_names in os.walk(package, followlinks=False):
        current = Path(current_raw)
        directory_names.sort()
        file_names.sort()
        for name in directory_names:
            child = current / name
            if child.is_symlink():
                raise VerificationError(
                    f"package contains directory symlink: {child.relative_to(package)}"
                )
        for name in file_names:
            path = current / name
            relative = path.relative_to(package).as_posix()
            if path.is_symlink():
                raise VerificationError(f"package contains file symlink: {relative}")
            resolved = path.resolve(strict=True)
            try:
                resolved.relative_to(package)
            except ValueError as exc:
                raise VerificationError(f"package file escapes package: {relative}") from exc
            files.append(
                {
                    "path": relative,
                    "bytes": resolved.stat().st_size,
                    "sha256": sha256_file(resolved),
                }
            )
    return canonical_sha256(files)


def load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise VerificationError(f"{label} is unavailable or invalid: {path}") from exc
    if not isinstance(value, dict):
        raise VerificationError(f"{label} root must be an object")
    return value


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    try:
        with path.open("r", encoding="utf-8") as handle:
            for line_number, line in enumerate(handle, start=1):
                line = line.strip()
                if not line:
                    continue
                value = json.loads(line)
                if not isinstance(value, dict):
                    raise VerificationError(
                        f"asset record at line {line_number} is not an object"
                    )
                rows.append(value)
    except (OSError, json.JSONDecodeError) as exc:
        raise VerificationError(f"asset_records.jsonl is invalid: {exc}") from exc
    return rows


def load_json_array(path: Path, label: str) -> list[dict[str, Any]]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise VerificationError(f"{label} is unavailable or invalid: {path}") from exc
    if not isinstance(value, list) or not all(isinstance(row, dict) for row in value):
        raise VerificationError(f"{label} root must be an array of objects")
    return value


def nonnegative_int(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise VerificationError(f"{label} must be a non-negative integer")
    return value


def rate(numerator: int, denominator: int) -> float | None:
    return numerator / denominator if denominator else None


def _local_tag(node: ET.Element) -> str:
    return node.tag.rsplit("}", 1)[-1]


def _children(node: ET.Element, tag: str) -> list[ET.Element]:
    return [child for child in node if _local_tag(child) == tag]


def eligible_nonadjacent_pairs(urdf_path: Path) -> set[tuple[str, str]]:
    try:
        root = ET.parse(urdf_path).getroot()
    except (OSError, ET.ParseError) as exc:
        raise VerificationError(f"allowance topology is unavailable: {urdf_path}") from exc
    links = _children(root, "link")
    names = [link.attrib.get("name", "").strip() for link in links]
    if any(not name for name in names) or len(set(names)) != len(names):
        raise VerificationError(f"invalid link identity in allowance topology: {urdf_path}")
    adjacent: set[tuple[str, str]] = set()
    for joint in _children(root, "joint"):
        parents = _children(joint, "parent")
        children = _children(joint, "child")
        if len(parents) != 1 or len(children) != 1:
            raise VerificationError(f"invalid joint topology: {urdf_path}")
        parent = parents[0].attrib.get("link", "").strip()
        child = children[0].attrib.get("link", "").strip()
        if parent not in names or child not in names or parent == child:
            raise VerificationError(f"invalid joint endpoint: {urdf_path}")
        adjacent.add(tuple(sorted((parent, child))))
    collision_names = [
        link.attrib.get("name", "").strip()
        for link in links
        if _children(link, "collision")
    ]
    all_pairs = {
        tuple(sorted((collision_names[left], collision_names[right])))
        for left in range(len(collision_names))
        for right in range(left + 1, len(collision_names))
    }
    return all_pairs - adjacent


def eligible_nonadjacent_pair_count(urdf_path: Path) -> int:
    return len(eligible_nonadjacent_pairs(urdf_path))


def _first_present(payload: Any, paths: tuple[tuple[str, ...], ...]) -> Any:
    for path in paths:
        current = payload
        for key in path:
            if not isinstance(current, Mapping) or key not in current:
                current = None
                break
            current = current[key]
        if current is not None:
            return current
    return None


def extract_allowance_pairs(payload: Any) -> set[tuple[str, str]]:
    raw_pairs = _first_present(
        payload,
        (
            ("excluded_non_adjacent_pairs",),
            ("allowances", "excluded_non_adjacent_pairs"),
            ("pair_policy", "excluded_non_adjacent_pairs"),
        ),
    )
    if not isinstance(raw_pairs, list):
        raise VerificationError("allowance pair list is missing")
    pairs: set[tuple[str, str]] = set()
    for index, raw in enumerate(raw_pairs):
        if isinstance(raw, (list, tuple)) and len(raw) == 2:
            left, right = raw
        elif isinstance(raw, Mapping):
            left, right = raw.get("link_a"), raw.get("link_b")
        else:
            raise VerificationError(f"allowance pair {index} has invalid shape")
        if (
            not isinstance(left, str)
            or not isinstance(right, str)
            or not left
            or not right
            or left == right
        ):
            raise VerificationError(f"allowance pair {index} has invalid names")
        pairs.add(tuple(sorted((left, right))))
    return pairs


def aggregate(rows: list[Mapping[str, Any]]) -> dict[str, Any]:
    denominator = len(rows)
    receipt_bound = sum(bool(row["release_receipt_bound"]) for row in rows)
    receipt_replayed = sum(bool(row["release_receipt_replay_pass"]) for row in rows)
    rebuild_eligible = sum(bool(row["deterministic_rebuild_eligible"]) for row in rows)
    rebuild_matched = sum(bool(row["deterministic_rebuild_match"]) for row in rows)
    registered_pairs = sum(
        nonnegative_int(
            row["registered_method_allowance_pair_count"],
            "registered allowance pair count",
        )
        for row in rows
    )
    eligible_pairs = sum(
        nonnegative_int(row["eligible_non_adjacent_pair_count"], "eligible pair count")
        for row in rows
    )
    no_allowance = sum(
        bool(row["strict_collision_pass_no_method_allowance"]) for row in rows
    )
    registered = sum(
        bool(row["strict_collision_pass_registered_allowance"]) for row in rows
    )
    rebuild: dict[str, Any] = {
        "status": "N/E" if rebuild_eligible == 0 else "NOT_RUN",
        "passed": None if rebuild_eligible == 0 else rebuild_matched,
        "denominator": rebuild_eligible,
        "rate": None if rebuild_eligible == 0 else rate(rebuild_matched, rebuild_eligible),
        "eligible_assets": rebuild_eligible,
        "asset_denominator": denominator,
    }
    if rebuild_eligible == 0:
        rebuild["reason"] = (
            "no frozen public build recipe with complete inputs in the release evidence"
        )
    return {
        "receipt_bound_assets": {
            "passed": receipt_bound,
            "denominator": denominator,
            "rate": rate(receipt_bound, denominator),
        },
        "receipt_replay_pass": {
            "passed": receipt_replayed,
            "denominator": denominator,
            "rate": rate(receipt_replayed, denominator),
        },
        "deterministic_rebuild_match": rebuild,
        "allowance_density": {
            "registered_pairs": registered_pairs,
            "eligible_pairs": eligible_pairs,
            "rate": rate(registered_pairs, eligible_pairs),
        },
        "strict_pass_no_method_allowance": {
            "passed": no_allowance,
            "denominator": denominator,
            "rate": rate(no_allowance, denominator),
        },
        "registered_allowance_gain_pp": {
            "value": (
                (registered - no_allowance) * 100.0 / denominator
                if denominator
                else None
            ),
            "registered_passed": registered,
            "no_allowance_passed": no_allowance,
            "denominator": denominator,
        },
    }


def official_source_manifest_sha256(rows: list[Mapping[str, Any]]) -> str:
    bindings: list[dict[str, Any]] = []
    for row in rows:
        release_evidence = row.get("release_evidence")
        rebuild = (
            release_evidence.get("rebuild")
            if isinstance(release_evidence, Mapping)
            else None
        )
        official = (
            rebuild.get("official_model_py")
            if isinstance(rebuild, Mapping)
            else None
        )
        if not isinstance(official, Mapping):
            raise VerificationError(
                f"official source evidence missing: {row.get('asset_id')}"
            )
        bindings.append(
            {
                "asset_id": row.get("asset_id"),
                "selection_index": row.get("selection_index"),
                "record_json_sha256": official.get("record_json_sha256"),
                "model_py_path": official.get("path"),
                "model_py_exists": official.get("exists"),
                "model_py_sha256": official.get("observed_sha256"),
            }
        )
    return canonical_sha256(bindings)


def official_model_py_evidence(
    category_records_root: Path,
    asset_id: str,
    compile_report: Mapping[str, Any],
) -> dict[str, Any]:
    record_root = (category_records_root / asset_id).resolve(strict=True)
    try:
        record_root.relative_to(category_records_root)
    except ValueError as exc:
        raise VerificationError(f"official record escapes records root: {asset_id}") from exc
    record_path = record_root / "record.json"
    record = load_json(record_path, "official record")
    if record.get("record_id") != asset_id:
        raise VerificationError(f"official record identity mismatch: {asset_id}")
    artifacts = record.get("artifacts")
    hashes = record.get("hashes")
    raw_model_path = artifacts.get("model_py") if isinstance(artifacts, Mapping) else None
    expected_hash = (
        hashes.get("model_py_sha256") if isinstance(hashes, Mapping) else None
    )
    compile_metrics = compile_report.get("metrics")
    fingerprint_inputs = (
        compile_metrics.get("fingerprint_inputs")
        if isinstance(compile_metrics, Mapping)
        else None
    )
    compile_hash = (
        fingerprint_inputs.get("model_py_sha256")
        if isinstance(fingerprint_inputs, Mapping)
        else None
    )
    declared = isinstance(raw_model_path, str) and bool(raw_model_path)
    exists = False
    observed_hash: str | None = None
    normalized_path: str | None = None
    if declared:
        try:
            relative = static_atoms.safe_package_relative_path(
                raw_model_path, field="official_model_py"
            )
        except ValueError as exc:
            raise VerificationError(f"official model.py path is invalid: {asset_id}") from exc
        candidate = record_root.joinpath(*relative.parts)
        normalized_path = relative.as_posix()
        if candidate.is_symlink():
            raise VerificationError(f"official model.py is a symlink: {asset_id}")
        if candidate.is_file():
            resolved = candidate.resolve(strict=True)
            try:
                resolved.relative_to(record_root)
            except ValueError as exc:
                raise VerificationError(
                    f"official model.py escapes record root: {asset_id}"
                ) from exc
            exists = True
            observed_hash = sha256_file(resolved)
    return {
        "record_json_path": str(record_path.resolve(strict=True)),
        "record_json_sha256": sha256_file(record_path),
        "declared": declared,
        "path": normalized_path,
        "exists": exists,
        "expected_sha256": expected_hash,
        "observed_sha256": observed_hash,
        "compile_report_model_py_sha256": compile_hash,
        "metadata_hash_match": bool(
            isinstance(expected_hash, str)
            and isinstance(compile_hash, str)
            and expected_hash == compile_hash
        ),
        "content_hash_match": bool(
            exists
            and isinstance(expected_hash, str)
            and observed_hash == expected_hash
        ),
    }


def verify_release_evidence(
    row: Mapping[str, Any],
    category_records_root: Path,
) -> None:
    asset_id = str(row["asset_id"])
    package = Path(str(row.get("package", ""))).resolve(strict=True)
    package_files = sorted(
        path
        for path in package.rglob("*")
        if path.is_file() and not path.is_symlink()
    )
    receipt_paths = [
        path
        for path in package_files
        if path.suffix.lower() == ".json" and RECEIPT_NAME_RE.search(path.name)
    ]
    rebuild_paths = [
        path for path in package_files if path.name.lower() in REBUILD_RECIPE_NAMES
    ]
    allowance_paths = [
        path
        for path in package_files
        if path.suffix.lower() == ".json" and ALLOWANCE_NAME_RE.search(path.name)
    ]
    if receipt_paths:
        raise VerificationError(
            f"release receipt candidate cannot be independently replayed: {asset_id}"
        )
    if rebuild_paths:
        raise VerificationError(
            f"release rebuild candidate cannot be independently executed: {asset_id}"
        )

    compile_path = package / "compile_report.json"
    compile_report = load_json(compile_path, "compile report")
    if compile_report.get("record_id") != asset_id:
        raise VerificationError(f"compile report identity mismatch: {asset_id}")
    overlap_allowances = compile_report.get("overlap_allowances")
    if not isinstance(overlap_allowances, list):
        raise VerificationError(
            f"compile report overlap_allowances is invalid: {asset_id}"
        )
    compile_pairs = extract_allowance_pairs(
        {"excluded_non_adjacent_pairs": overlap_allowances}
    )
    eligible_pairs = eligible_nonadjacent_pairs(package / "model.urdf")
    if not compile_pairs <= eligible_pairs:
        raise VerificationError(f"compile allowance pair is not eligible: {asset_id}")
    registered_pairs = set(compile_pairs)
    registry_paths: list[str] = []
    for allowance_path in allowance_paths:
        pairs = extract_allowance_pairs(load_json(allowance_path, "allowance registry"))
        if not pairs <= eligible_pairs:
            raise VerificationError(f"release allowance pair is not eligible: {asset_id}")
        registered_pairs.update(pairs)
        registry_paths.append(allowance_path.relative_to(package).as_posix())

    release_evidence = row.get("release_evidence")
    if not isinstance(release_evidence, Mapping):
        raise VerificationError(f"release evidence missing: {asset_id}")
    receipt = release_evidence.get("receipt")
    rebuild = release_evidence.get("rebuild")
    allowance = release_evidence.get("allowance")
    compile_evidence = release_evidence.get("compile_report")
    if not all(
        isinstance(value, Mapping)
        for value in (receipt, rebuild, allowance, compile_evidence)
    ):
        raise VerificationError(f"release evidence structure mismatch: {asset_id}")
    assert isinstance(receipt, Mapping)
    assert isinstance(rebuild, Mapping)
    assert isinstance(allowance, Mapping)
    assert isinstance(compile_evidence, Mapping)

    if (
        row.get("release_receipt_bound") is not False
        or row.get("release_receipt_replay_pass") is not False
        or row.get("receipt_replay_status") != "NO_VALID_RECEIPT"
        or receipt.get("candidate_count") != 0
        or receipt.get("valid_mechanical_receipt_count") != 0
        or bool(receipt.get("receipt_bound_asset"))
    ):
        raise VerificationError(f"release receipt evidence mismatch: {asset_id}")
    if (
        row.get("deterministic_rebuild_eligible") is not False
        or row.get("deterministic_rebuild_match") is not False
        or row.get("deterministic_rebuild_status") != "N/E"
        or rebuild.get("candidate_recipe_count") != 0
        or rebuild.get("valid_recipe_count") != 0
        or bool(rebuild.get("eligible_asset"))
    ):
        raise VerificationError(f"release rebuild evidence mismatch: {asset_id}")
    expected_registry_sources = ["compile_report.json", *registry_paths]
    if (
        row.get("eligible_non_adjacent_pair_count") != len(eligible_pairs)
        or row.get("registered_method_allowance_pair_count")
        != len(registered_pairs)
        or allowance.get("eligible_nonadjacent_pair_count") != len(eligible_pairs)
        or allowance.get("registered_excluded_pair_count") != len(registered_pairs)
        or allowance.get("registry_sources") != expected_registry_sources
    ):
        raise VerificationError(f"release allowance evidence mismatch: {asset_id}")
    if (
        compile_evidence.get("path") != "compile_report.json"
        or compile_evidence.get("sha256") != sha256_file(compile_path)
        or compile_evidence.get("mechanical_receipt") is not False
        or compile_evidence.get("status") != compile_report.get("status")
        or compile_evidence.get("overlap_allowances") != overlap_allowances
    ):
        raise VerificationError(f"compile report evidence mismatch: {asset_id}")

    static_record = static_atoms.audit_lam_package(
        package,
        urdf_relative_path="model.urdf",
        asset_id=asset_id,
    )
    if static_record.get("status") != "completed":
        raise VerificationError(f"static release evidence failed: {asset_id}")
    if static_record.get("urdf_sha256") != row.get("model_urdf_sha256"):
        raise VerificationError(f"static release URDF identity mismatch: {asset_id}")
    s1 = static_record.get("s1_evidence")
    if not isinstance(s1, Mapping):
        raise VerificationError(f"static S1 evidence is missing: {asset_id}")
    receipt_raw = s1.get("receipt")
    rebuild_raw = s1.get("rebuild")
    allowance_raw = s1.get("allowance")
    if not all(
        isinstance(value, Mapping)
        for value in (receipt_raw, rebuild_raw, allowance_raw)
    ):
        raise VerificationError(f"static S1 evidence is incomplete: {asset_id}")
    assert isinstance(receipt_raw, Mapping)
    assert isinstance(rebuild_raw, Mapping)
    assert isinstance(allowance_raw, Mapping)
    rebuild_eligible = bool(rebuild_raw.get("eligible_asset"))
    expected_release_evidence = {
        "asset_id": asset_id,
        "resource_closure": static_record["resource_closure"],
        "receipt": {
            **dict(receipt_raw),
            "receipt_bound_asset": bool(receipt_raw.get("receipt_bound_asset")),
        },
        "compile_report": {
            "path": "compile_report.json",
            "sha256": sha256_file(compile_path),
            "mechanical_receipt": False,
            "status": compile_report.get("status"),
            "overlap_allowances": overlap_allowances,
        },
        "rebuild": {
            **dict(rebuild_raw),
            "status": "ELIGIBLE_NOT_RUN" if rebuild_eligible else "N/E",
            "eligible_asset": rebuild_eligible,
            "official_model_py": official_model_py_evidence(
                category_records_root, asset_id, compile_report
            ),
        },
        "allowance": {
            **dict(allowance_raw),
            "status": "COMPLETE",
            "registered_excluded_pair_count": len(registered_pairs),
            "registry_sources": ["compile_report.json", *registry_paths],
        },
        "issues": list(static_record.get("issues", [])),
    }
    if release_evidence != expected_release_evidence:
        raise VerificationError(f"nested release evidence mismatch: {asset_id}")
    if row.get("resource_closure_sha256") != static_record.get(
        "resource_closure", {}
    ).get("sha256"):
        raise VerificationError(f"resource closure evidence mismatch: {asset_id}")


def _validate_source_binding(
    row: Mapping[str, Any],
    source: Mapping[str, Any],
    index: int,
) -> None:
    asset_id = str(row["asset_id"])
    if source.get("asset_id") != asset_id or source.get("selection_index") != index:
        raise VerificationError(f"source cohort identity mismatch: {asset_id}")
    source_binding = source.get("package_binding")
    if not isinstance(source_binding, Mapping):
        raise VerificationError(f"source cohort package binding missing: {asset_id}")
    expected_package_hash = source_binding.get("content_manifest_sha256")
    expected_urdf_hash = source.get("model_urdf_sha256")
    if not isinstance(expected_package_hash, str) or not isinstance(
        expected_urdf_hash, str
    ):
        raise VerificationError(f"source cohort hashes missing: {asset_id}")
    try:
        source_package = Path(str(source.get("package", ""))).resolve(strict=True)
        row_package = Path(str(row.get("package", ""))).resolve(strict=True)
    except OSError as exc:
        raise VerificationError(
            f"source cohort package binding mismatch: {asset_id}: {exc}"
        ) from exc
    if source_package != row_package:
        raise VerificationError(f"source cohort package path mismatch: {asset_id}")
    observed_package_hash = package_content_manifest_sha256(source_package)
    if (
        observed_package_hash != expected_package_hash
        or row.get("package_content_manifest_sha256") != expected_package_hash
    ):
        raise VerificationError(f"source cohort package binding mismatch: {asset_id}")
    urdf_path = source_package / "model.urdf"
    if (
        not urdf_path.is_file()
        or sha256_file(urdf_path) != expected_urdf_hash
        or row.get("model_urdf_sha256") != expected_urdf_hash
    ):
        raise VerificationError(f"source cohort URDF binding mismatch: {asset_id}")
    augmented_source = {
        **dict(source),
        "package": str(source_package),
        "package_content_manifest_sha256": expected_package_hash,
    }
    if row.get("source_cohort_record_sha256") != canonical_sha256(augmented_source):
        raise VerificationError(f"source cohort record SHA256 mismatch: {asset_id}")
    if row.get("eligible_non_adjacent_pair_count") != eligible_nonadjacent_pair_count(
        urdf_path
    ):
        raise VerificationError(f"allowance eligible-pair mismatch: {asset_id}")


def _validate_asset_rows(
    rows: list[dict[str, Any]],
    n_eval: int,
    source_records: list[dict[str, Any]],
    category_records_root: Path,
) -> None:
    if len(rows) != n_eval:
        raise VerificationError("asset record denominator mismatch")
    if len(source_records) < n_eval:
        raise VerificationError("source cohort is shorter than asset records")
    seen: set[str] = set()
    for index, row in enumerate(rows):
        asset_id = row.get("asset_id")
        if not isinstance(asset_id, str) or not asset_id or asset_id in seen:
            raise VerificationError(f"invalid or duplicate asset_id at index {index}")
        seen.add(asset_id)
        if row.get("selection_index") != index:
            raise VerificationError(f"asset record order mismatch at index {index}")
        if row.get("terminal") is not True or row.get("status") != "completed":
            raise VerificationError(f"asset record is not terminal: {asset_id}")
        boolean_fields = (
            "release_receipt_bound",
            "release_receipt_replay_pass",
            "deterministic_rebuild_eligible",
            "deterministic_rebuild_match",
            "strict_collision_pass_no_method_allowance",
            "strict_collision_pass_registered_allowance",
        )
        for field in boolean_fields:
            if not isinstance(row.get(field), bool):
                raise VerificationError(f"asset {field} is not boolean: {asset_id}")
        if row["release_receipt_replay_pass"] and not row["release_receipt_bound"]:
            raise VerificationError(f"receipt replay passed without receipt: {asset_id}")
        if row["deterministic_rebuild_match"] and not row["deterministic_rebuild_eligible"]:
            raise VerificationError(f"rebuild matched without eligibility: {asset_id}")
        registered_pairs = nonnegative_int(
            row.get("registered_method_allowance_pair_count"),
            f"registered allowance count for {asset_id}",
        )
        nonnegative_int(
            row.get("eligible_non_adjacent_pair_count"),
            f"eligible pair count for {asset_id}",
        )
        if registered_pairs != 0:
            raise VerificationError(
                f"non-empty allowance lacks pair-specific replay: {asset_id}"
            )
        if (
            row["strict_collision_pass_registered_allowance"]
            != row["strict_collision_pass_no_method_allowance"]
        ):
            raise VerificationError(f"empty allowance changed strict result: {asset_id}")
        if row.get("schema_version") != "s1-articraft10k-asset/v1":
            raise VerificationError(f"asset schema mismatch: {asset_id}")
        _validate_source_binding(row, source_records[index], index)
        verify_release_evidence(row, category_records_root)
        release_evidence = row.get("release_evidence")
        rebuild = (
            release_evidence.get("rebuild")
            if isinstance(release_evidence, Mapping)
            else None
        )
        official = (
            rebuild.get("official_model_py")
            if isinstance(rebuild, Mapping)
            else None
        )
        if not isinstance(official, Mapping):
            raise VerificationError(f"official rebuild evidence missing: {asset_id}")
        official_record_path = Path(str(official.get("record_json_path", "")))
        try:
            observed_record_hash = sha256_file(
                official_record_path.resolve(strict=True)
            )
        except OSError as exc:
            raise VerificationError(
                f"official record SHA256 mismatch: {asset_id}: {exc}"
            ) from exc
        if observed_record_hash != official.get("record_json_sha256"):
            raise VerificationError(f"official record SHA256 mismatch: {asset_id}")
        official_record = load_json(official_record_path, "official record")
        if official_record.get("record_id") != asset_id:
            raise VerificationError(f"official record identity mismatch: {asset_id}")


def _require_identity(
    expected: Mapping[str, Any],
    observed: Mapping[str, Any],
    fields: tuple[str, ...],
    label: str,
) -> None:
    for field in fields:
        if observed.get(field) != expected.get(field):
            raise VerificationError(f"{label} {field} mismatch")


def _reaggregate_table4_asset(
    item: Mapping[str, Any],
    asset: Mapping[str, Any],
    states: list[dict[str, Any]],
) -> dict[str, Any]:
    asset_id = str(item["asset_id"])
    if canonical_sha256(states) != asset.get("state_records_sha256"):
        raise VerificationError(f"Table 4 state records SHA256 mismatch: {asset_id}")
    expected_by_phase = {
        "rest": nonnegative_int(item.get("rest_state_expected"), "rest expected"),
        "single_joint_sweep": nonnegative_int(
            item.get("single_state_expected"), "single expected"
        ),
        "multi_joint_sobol": nonnegative_int(
            item.get("sobol_state_expected"), "Sobol expected"
        ),
    }
    rows_by_phase: dict[str, list[dict[str, Any]]] = {
        phase: [] for phase in expected_by_phase
    }
    for state in states:
        _require_identity(
            item,
            state,
            ("protocol_id", "asset_id", "selection_index", "order"),
            f"Table 4 state {asset_id}",
        )
        phase = state.get("phase")
        if phase not in rows_by_phase:
            raise VerificationError(
                f"unknown Table 4 state phase for {asset_id}: {phase!r}"
            )
        nonnegative_int(
            state.get("non_adjacent_illegal_penetration_count"),
            f"illegal penetration count for {asset_id}",
        )
        rows_by_phase[str(phase)].append(state)
    executed_by_phase = {
        phase: len(phase_rows) for phase, phase_rows in rows_by_phase.items()
    }
    free_by_phase = {
        phase: sum(
            row["non_adjacent_illegal_penetration_count"] == 0
            for row in phase_rows
        )
        for phase, phase_rows in rows_by_phase.items()
    }
    movable = nonnegative_int(item.get("movable_dof_count"), "movable DoF count")
    range_evaluable = nonnegative_int(
        item.get("range_evaluable_dof_count"), "range-evaluable DoF count"
    )
    rest_pass = bool(
        executed_by_phase["rest"] == expected_by_phase["rest"]
        and free_by_phase["rest"] == expected_by_phase["rest"]
    )
    single_pass = bool(
        executed_by_phase["single_joint_sweep"]
        == expected_by_phase["single_joint_sweep"]
        and free_by_phase["single_joint_sweep"]
        == expected_by_phase["single_joint_sweep"]
    )
    sobol_pass = bool(
        movable > 0
        and range_evaluable == movable
        and executed_by_phase["multi_joint_sobol"]
        == expected_by_phase["multi_joint_sobol"]
        and free_by_phase["multi_joint_sobol"]
        == expected_by_phase["multi_joint_sobol"]
    )
    measurement_complete = bool(
        range_evaluable == movable
        and sum(executed_by_phase.values()) == sum(expected_by_phase.values())
    )
    strict_pass = bool(
        measurement_complete and rest_pass and single_pass and sobol_pass
    )
    recorded_fields = {
        "rest_state_executed": executed_by_phase["rest"],
        "rest_non_adjacent_free": free_by_phase["rest"],
        "rest_non_adjacent_cf": rest_pass,
        "single_state_executed": executed_by_phase["single_joint_sweep"],
        "single_non_adjacent_free": free_by_phase["single_joint_sweep"],
        "single_joint_sweep_cf": single_pass,
        "sobol_state_executed": executed_by_phase["multi_joint_sobol"],
        "sobol_non_adjacent_free": free_by_phase["multi_joint_sobol"],
        "multi_joint_sobol_cf": sobol_pass,
        "measurement_complete": measurement_complete,
        "strict_collision_pass": strict_pass,
    }
    for field, expected in recorded_fields.items():
        if asset.get(field) != expected:
            raise VerificationError(f"Table 4 asset {field} mismatch: {asset_id}")
    for phase, field in (
        ("rest", "rest_state_expected"),
        ("single_joint_sweep", "single_state_expected"),
        ("multi_joint_sobol", "sobol_state_expected"),
    ):
        if asset.get(field) != expected_by_phase[phase]:
            raise VerificationError(f"Table 4 asset {field} mismatch: {asset_id}")
    return {
        "strict_collision_pass": strict_pass,
        "measurement_complete": measurement_complete,
        "state_record_count": len(states),
        "state_records_sha256": asset["state_records_sha256"],
        "asset_record_sha256": canonical_sha256(asset),
    }


def verify_table4_evidence(
    table4_source: Mapping[str, Any],
    source_records: list[dict[str, Any]],
    output_rows: list[dict[str, Any]],
    n_eval: int,
    *,
    formal: bool,
) -> int:
    bindings = (
        (
            "manifest_path",
            "manifest_file_sha256",
            "manifest",
            FORMAL_TABLE4_MANIFEST_SHA256,
        ),
        (
            "asset_records_path",
            "asset_records_file_sha256",
            "asset records",
            FORMAL_TABLE4_ASSET_RECORDS_SHA256,
        ),
        (
            "state_records_path",
            "state_records_file_sha256",
            "state records",
            FORMAL_TABLE4_STATE_RECORDS_SHA256,
        ),
    )
    paths: dict[str, Path] = {}
    for path_field, hash_field, label, formal_hash in bindings:
        try:
            path = Path(str(table4_source.get(path_field, ""))).resolve(strict=True)
        except OSError as exc:
            raise VerificationError(f"Table 4 {label} is unavailable") from exc
        observed_hash = sha256_file(path)
        if observed_hash != table4_source.get(hash_field):
            raise VerificationError(f"Table 4 {label} SHA256 mismatch")
        if formal and observed_hash != formal_hash:
            raise VerificationError(f"formal Table 4 {label} identity mismatch")
        paths[path_field] = path

    manifest = load_json(paths["manifest_path"], "Table 4 manifest")
    assets = load_json_array(paths["asset_records_path"], "Table 4 asset records")
    states = load_jsonl(paths["state_records_path"])
    items = manifest.get("items")
    if not isinstance(items, list) or not all(isinstance(item, dict) for item in items):
        raise VerificationError("Table 4 manifest items are invalid")
    protocol_id = manifest.get("protocol_id")
    if protocol_id != table4_source.get("protocol_id"):
        raise VerificationError("Table 4 protocol identity mismatch")
    if formal and protocol_id != FORMAL_TABLE4_PROTOCOL_ID:
        raise VerificationError("formal Table 4 protocol identity mismatch")
    if len(items) < n_eval or len(assets) < n_eval:
        raise VerificationError("Table 4 evidence is shorter than the cohort")
    if formal and (len(items) != n_eval or len(assets) != n_eval):
        raise VerificationError("formal Table 4 denominator mismatch")
    items = items[:n_eval]
    assets = assets[:n_eval]
    selected_states = [
        state
        for state in states
        if isinstance(state.get("selection_index"), int)
        and not isinstance(state.get("selection_index"), bool)
        and 0 <= state["selection_index"] < n_eval
    ]
    if table4_source.get("state_record_count") != len(selected_states):
        raise VerificationError("Table 4 state record count mismatch")
    states_by_index: dict[int, list[dict[str, Any]]] = {
        index: [] for index in range(n_eval)
    }
    previous_index = -1
    for state in selected_states:
        index = nonnegative_int(state.get("selection_index"), "state selection_index")
        if index < previous_index:
            raise VerificationError("Table 4 state records are not in cohort order")
        previous_index = index
        states_by_index[index].append(state)

    strict_passed = 0
    for index, (source, item, asset, output_row) in enumerate(
        zip(source_records[:n_eval], items, assets, output_rows, strict=True)
    ):
        asset_id = str(source.get("asset_id", ""))
        source_binding = source.get("package_binding")
        if not isinstance(source_binding, Mapping):
            raise VerificationError(f"source package binding missing: {asset_id}")
        expected_item = {
            "asset_id": asset_id,
            "selection_index": index,
            "model_urdf_sha256": source.get("model_urdf_sha256"),
            "package_content_manifest_sha256": source_binding.get(
                "content_manifest_sha256"
            ),
        }
        _require_identity(
            expected_item,
            item,
            tuple(expected_item),
            f"Table 4 manifest item {asset_id}",
        )
        _require_identity(
            item,
            asset,
            (
                "protocol_id",
                "asset_id",
                "selection_index",
                "order",
                "model_urdf_sha256",
                "package_content_manifest_sha256",
                "movable_dof_count",
                "range_evaluable_dof_count",
            ),
            f"Table 4 asset record {asset_id}",
        )
        reaggregated = _reaggregate_table4_asset(
            item, asset, states_by_index[index]
        )
        if output_row.get("strict_collision_pass_no_method_allowance") != reaggregated[
            "strict_collision_pass"
        ]:
            raise VerificationError(f"Table 4 strict collision mismatch: {asset_id}")
        if output_row.get("strict_collision_pass_registered_allowance") != reaggregated[
            "strict_collision_pass"
        ]:
            raise VerificationError(f"Table 4 registered strict mismatch: {asset_id}")
        output_bindings = {
            "table4_measurement_complete": reaggregated["measurement_complete"],
            "table4_state_record_count": reaggregated["state_record_count"],
            "table4_state_records_sha256": reaggregated["state_records_sha256"],
            "table4_asset_record_sha256": reaggregated["asset_record_sha256"],
        }
        for field, expected in output_bindings.items():
            if output_row.get(field) != expected:
                raise VerificationError(f"Table 4 output {field} mismatch: {asset_id}")
        strict_passed += int(reaggregated["strict_collision_pass"])
    return strict_passed


def verify_run(run_directory: Path, *, formal: bool) -> dict[str, Any]:
    run_directory = Path(run_directory).resolve(strict=True)
    checks: list[dict[str, Any]] = []

    def passed(name: str, detail: str = "") -> None:
        checks.append({"check": name, "pass": True, "detail": detail})

    manifest = load_json(run_directory / "manifest.json", "manifest")
    expected_content_hash = manifest.get("manifest_content_sha256")
    manifest_without_hash = {
        key: value
        for key, value in manifest.items()
        if key != "manifest_content_sha256"
    }
    if expected_content_hash != canonical_sha256(manifest_without_hash):
        raise VerificationError("manifest content SHA256 mismatch")
    passed("manifest_content_sha256")
    if (
        manifest.get("schema_version") != SCHEMA_VERSION
        or manifest.get("protocol_id") != PROTOCOL_ID
        or manifest.get("dataset") != DATASET
    ):
        raise VerificationError("manifest protocol identity mismatch")
    passed("manifest_protocol_identity")

    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, Mapping) or set(artifacts) != EXPECTED_ARTIFACTS:
        raise VerificationError("manifest artifact set mismatch")
    for name in sorted(EXPECTED_ARTIFACTS):
        binding = artifacts[name]
        path = run_directory / name
        if not isinstance(binding, Mapping) or not path.is_file():
            raise VerificationError(f"artifact is unavailable: {name}")
        if sha256_file(path) != binding.get("sha256"):
            raise VerificationError(f"artifact SHA256 mismatch: {name}")
        if path.stat().st_size != binding.get("bytes"):
            raise VerificationError(f"artifact byte count mismatch: {name}")
    passed("artifact_bindings", str(len(artifacts)))

    config = load_json(run_directory / "frozen_config.json", "frozen config")
    summary = load_json(run_directory / "summary.json", "summary")
    if (
        config.get("protocol_id") != PROTOCOL_ID
        or summary.get("protocol_id") != PROTOCOL_ID
        or summary.get("status") != "completed"
    ):
        raise VerificationError("config or summary protocol identity mismatch")
    passed("config_summary_protocol_identity")
    snapshot_hash = sha256_file(run_directory / "protocol_snapshot.md")
    if snapshot_hash != config.get("protocol_document", {}).get("sha256_at_freeze"):
        raise VerificationError("protocol snapshot SHA256 mismatch")
    passed("protocol_snapshot_binding")

    code_identity = config.get("code_identity")
    if not isinstance(code_identity, Mapping):
        raise VerificationError("code identity is missing")
    for prefix in ("runner", "verifier", "static_atoms"):
        path = Path(str(code_identity.get(f"{prefix}_path", ""))).resolve(strict=True)
        if sha256_file(path) != code_identity.get(f"{prefix}_sha256"):
            raise VerificationError(f"{prefix} code SHA256 mismatch")
    if formal and code_identity.get("static_atoms_sha256") != FORMAL_STATIC_ATOMS_SHA256:
        raise VerificationError("formal static-atoms identity mismatch")
    passed("code_identity")

    table4_source = config.get("table4_source")
    if not isinstance(table4_source, Mapping):
        raise VerificationError("Table 4 source binding is missing")
    if manifest.get("table4_source") != table4_source:
        raise VerificationError("manifest/config Table 4 binding mismatch")

    cohort = config.get("cohort")
    if not isinstance(cohort, Mapping):
        raise VerificationError("cohort binding is missing")
    if manifest.get("cohort") != cohort:
        raise VerificationError("manifest/config cohort binding mismatch")
    release_sources = config.get("release_sources")
    if not isinstance(release_sources, Mapping):
        raise VerificationError("release source binding is missing")
    if manifest.get("release_sources") != release_sources:
        raise VerificationError("manifest/config release source binding mismatch")
    try:
        category_records_root = Path(
            str(release_sources.get("category_records_root", ""))
        ).resolve(strict=True)
    except OSError as exc:
        raise VerificationError("category records root is unavailable") from exc
    if not category_records_root.is_dir():
        raise VerificationError("category records root is not a directory")
    if formal and category_records_root != FORMAL_CATEGORY_RECORDS_ROOT:
        raise VerificationError("formal category records root identity mismatch")
    source_manifest = Path(str(cohort.get("source_manifest", ""))).resolve(strict=True)
    if sha256_file(source_manifest) != cohort.get("source_manifest_file_sha256"):
        raise VerificationError("source cohort manifest SHA256 mismatch")
    source_payload = load_json(source_manifest, "source cohort manifest")
    source_records = source_payload.get("records")
    if not isinstance(source_records, list) or not all(
        isinstance(row, dict) for row in source_records
    ):
        raise VerificationError("source cohort records are unavailable")
    if source_payload.get("manifest_content_sha256") != cohort.get(
        "source_manifest_content_sha256"
    ):
        raise VerificationError("source cohort content SHA256 mismatch")
    cohort_n_eval = nonnegative_int(cohort.get("n_eval"), "cohort n_eval")
    if cohort_n_eval > len(source_records):
        raise VerificationError("source cohort is shorter than the frozen denominator")
    ordered_hash = canonical_sha256(
        [row.get("asset_id") for row in source_records[:cohort_n_eval]]
    )
    if ordered_hash != cohort.get("ordered_asset_ids_sha256"):
        raise VerificationError("source cohort ordered IDs SHA256 mismatch")
    passed("source_cohort_binding")

    n_eval = nonnegative_int(summary.get("n_eval"), "summary n_eval")
    if n_eval != manifest.get("n_eval") or n_eval != cohort.get("n_eval"):
        raise VerificationError("run denominator mismatch")
    rows = load_jsonl(run_directory / "asset_records.jsonl")
    _validate_asset_rows(rows, n_eval, source_records, category_records_root)
    observed_official_source_manifest = official_source_manifest_sha256(rows)
    if observed_official_source_manifest != release_sources.get(
        "official_source_manifest_sha256"
    ):
        raise VerificationError("official source manifest mismatch")
    if (
        formal
        and observed_official_source_manifest
        != FORMAL_OFFICIAL_SOURCE_MANIFEST_SHA256
    ):
        raise VerificationError("formal official source manifest identity mismatch")
    if [row["asset_id"] for row in rows] != [
        row.get("asset_id") for row in source_records[:n_eval]
    ]:
        raise VerificationError("asset records do not preserve cohort order")
    passed("asset_record_closure", str(n_eval))

    table4_strict_passed = verify_table4_evidence(
        table4_source,
        source_records,
        rows,
        n_eval,
        formal=formal,
    )
    passed("table4_raw_state_reaggregation", str(table4_strict_passed))

    recomputed = aggregate(rows)
    if summary.get("metrics") != recomputed:
        raise VerificationError("summary metrics do not recompute from asset records")
    passed("summary_recomputation")

    if formal:
        allowance = recomputed["allowance_density"]
        strict = recomputed["strict_pass_no_method_allowance"]
        if (
            n_eval != FORMAL_N_EVAL
            or manifest.get("classification") != "FORMAL"
            or config.get("classification") != "FORMAL"
            or summary.get("classification") != "FORMAL"
            or cohort.get("source_manifest_file_sha256")
            != FORMAL_SOURCE_MANIFEST_SHA256
            or cohort.get("source_manifest_content_sha256")
            != FORMAL_SOURCE_MANIFEST_CONTENT_SHA256
            or cohort.get("ordered_asset_ids_sha256")
            != FORMAL_ORDERED_ASSET_IDS_SHA256
            or recomputed["receipt_bound_assets"]["passed"] != 0
            or recomputed["receipt_replay_pass"]["passed"] != 0
            or recomputed["deterministic_rebuild_match"]["eligible_assets"] != 0
            or allowance["registered_pairs"] != 0
            or allowance["eligible_pairs"] != FORMAL_ELIGIBLE_PAIRS
            or strict["passed"] != FORMAL_STRICT_PASSED
            or table4_strict_passed != FORMAL_STRICT_PASSED
        ):
            raise VerificationError("formal frozen result contract mismatch")
        passed("formal_result_contract")

    return {
        "schema_version": "s1-articraft10k-verification/v1",
        "protocol_id": PROTOCOL_ID,
        "run_directory": str(run_directory),
        "verified_at": dt.datetime.now(dt.timezone.utc).isoformat().replace(
            "+00:00", "Z"
        ),
        "formal": formal,
        "all_pass": True,
        "check_count": len(checks),
        "checks": checks,
    }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--formal", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        verification = verify_run(args.run, formal=args.formal)
    except Exception as exc:  # noqa: BLE001
        print(f"verification failed: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    path = Path(args.run).resolve() / "verification.json"
    path.write_text(
        json.dumps(
            verification,
            indent=2,
            sort_keys=True,
            ensure_ascii=True,
            allow_nan=False,
        )
        + "\n",
        encoding="utf-8",
    )
    print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
