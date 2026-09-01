#!/usr/bin/env python3
"""Independently verify a PhysX-Mobility Supplementary Table S1 run."""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path, PurePosixPath, PureWindowsPath
import re
import sys
from typing import Any, Mapping
import xml.etree.ElementTree as ET
import zipfile


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
PROTOCOL_ID = "s1_physx_mobility_table5cohort_n800_v1"
DATASET = "PhysX-Mobility"
N_EVAL = 800
DATASET_ROOT = REPO / "exp/PhysX-Mobility/extracted/PhysX_mobility"
SOURCE_MANIFEST = REPO / "exp/runtime/table5_physx_mobility_n800_v2/manifest.json"
TABLE4_RUN = REPO / "exp/runtime/urdf_table4_physx_mobility_table5cohort_n800_20260819T143442Z"
TABLE4_MANIFEST = TABLE4_RUN / "manifest.json"
TABLE4_ASSET_RECORDS = TABLE4_RUN / "asset_records.json"
TABLE4_STATE_RECORDS = TABLE4_RUN / "state_records.jsonl"
TABLE4_ARTIFACT_MANIFEST = TABLE4_RUN / "artifact_manifest.json"

EXPECTED_SOURCE_MANIFEST_SHA256 = "ccb54f4b726fe717efd28a37948e6b92bac994a2c0ba8fb4ea9ac4548d3a9882"
EXPECTED_ORDERED_IDS_SHA256 = "12dddf242e3e65dbe5d41b047752ed9b59d7ecb4387b81c9327bc0bc533172ad"
EXPECTED_TABLE4_MANIFEST_SHA256 = "671d107836fd1344fee34565aa9b16439b208598d179ee5bb59bbfb9bdbfef87"
EXPECTED_TABLE4_ASSET_RECORDS_SHA256 = "1efcfea5920e8db21f7b8afe9de0713a6bb478577d8b4417a6feb2b207397a3d"
EXPECTED_TABLE4_STATE_RECORDS_SHA256 = "eaa461e205e96b9bc2db4538de4295ba5efdd9bce0ac5d52ed0524648b6ec5fa"
EXPECTED_TABLE4_ARTIFACT_MANIFEST_SHA256 = "29193c31078ebe67c2b7202a092627c6e90a40db82d051c4d8dd02c2aa4e16f5"
EXPECTED_TABLE4_PROTOCOL_ID = "urdf_sim_ready_table4_physx_mobility_n800_v1"
EXPECTED_FORMAL_STRICT_PASSED = 786
EXPECTED_FORMAL_STATE_COUNT = 104453
REQUIRED_MANIFEST_ARTIFACTS = frozenset(
    {
        "protocol_snapshot.md",
        "frozen_config.json",
        "asset_records.jsonl",
        "summary.json",
        "summary.md",
    }
)
POST_MANIFEST_RECEIPTS = {
    "verification.json": (
        "independent verifier receipt generated after manifest finalization; "
        "excluded from the non-circular manifest artifact closure"
    )
}

RECEIPT_NAME_RE = re.compile(r"(?:^|[-_])(?:mechanical[-_])?receipt(?:[-_]|\.|$)", re.IGNORECASE)
ALLOWANCE_NAME_RE = re.compile(r"allow(?:ance|list|ed)|exclu(?:de|sion)", re.IGNORECASE)
REBUILD_RECIPE_NAMES = frozenset(
    {
        "build_recipe.json",
        "build-recipe.json",
        "rebuild_recipe.json",
        "rebuild-recipe.json",
        "deterministic_rebuild.json",
    }
)


class VerificationError(RuntimeError):
    """Raised when any frozen binding or recomputed result differs."""


def canonical_json(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False)


def canonical_sha256(value: Any) -> str:
    return hashlib.sha256(canonical_json(value).encode("utf-8")).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, raw in enumerate(stream, start=1):
            if not raw.strip():
                continue
            value = json.loads(raw)
            if not isinstance(value, dict):
                raise VerificationError(f"JSONL row {line_number} is not an object: {path}")
            records.append(value)
    return records


def _ratio(passed: int, denominator: int) -> dict[str, int | float | None]:
    return {
        "passed": passed,
        "denominator": denominator,
        "rate": None if denominator == 0 else passed / denominator,
        "percentage": None if denominator == 0 else 100.0 * passed / denominator,
    }


def _nonnegative_int(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise VerificationError(f"{label} must be a non-negative integer")
    return value


def _expect(observed: Any, expected: Any, label: str) -> None:
    if observed != expected:
        raise VerificationError(f"{label} mismatch: expected {expected!r}, observed {observed!r}")


def _safe_relative(raw: str) -> PurePosixPath:
    if not isinstance(raw, str) or not raw or "\\" in raw or "\x00" in raw:
        raise VerificationError(f"invalid package-relative path: {raw!r}")
    if raw.startswith("/") or PureWindowsPath(raw).is_absolute():
        raise VerificationError(f"absolute package-relative path: {raw!r}")
    parts: list[str] = []
    for part in raw.split("/"):
        if part in ("", "."):
            if part == "":
                raise VerificationError(f"noncanonical package-relative path: {raw!r}")
            continue
        if part == "..":
            if parts and parts[-1] != "..":
                parts.pop()
            else:
                parts.append(part)
        else:
            parts.append(part)
    if not parts:
        raise VerificationError(f"empty normalized package-relative path: {raw!r}")
    return PurePosixPath(*parts)


def _local_tag(node: ET.Element) -> str:
    return node.tag.rsplit("}", 1)[-1]


def _children(node: ET.Element, name: str) -> list[ET.Element]:
    return [child for child in node if _local_tag(child) == name]


def _eligible_pairs(root: ET.Element) -> int:
    links = _children(root, "link")
    names = [link.attrib.get("name", "").strip() for link in links]
    if not names or any(not name for name in names) or len(set(names)) != len(names):
        raise VerificationError("URDF link identities are incomplete or duplicated")
    collision_links = [
        link.attrib["name"].strip() for link in links if _children(link, "collision")
    ]
    adjacent: set[tuple[str, str]] = set()
    for joint in _children(root, "joint"):
        parents = _children(joint, "parent")
        children = _children(joint, "child")
        if len(parents) != 1 or len(children) != 1:
            raise VerificationError("joint parent/child cardinality is invalid")
        parent = parents[0].attrib.get("link", "").strip()
        child = children[0].attrib.get("link", "").strip()
        if parent not in names or child not in names or parent == child:
            raise VerificationError("joint parent/child identity is invalid")
        adjacent.add(tuple(sorted((parent, child))))
    pairs = {
        tuple(sorted((collision_links[left], collision_links[right])))
        for left in range(len(collision_links))
        for right in range(left + 1, len(collision_links))
    }
    return len(pairs - adjacent)


def _source_entries(row: Mapping[str, Any]) -> list[tuple[str, str]]:
    entries = [(str(row["urdf_relative_path"]), str(row["urdf_sha256"]))]
    entries.extend((str(value["relative_path"]), str(value["sha256"])) for value in row["resources"])
    entries.append((str(row["finaljson_relative_path"]), str(row["finaljson_sha256"])))
    if len({relative for relative, _ in entries}) != len(entries):
        raise VerificationError(f"duplicate source entry for {row['dataset_id']}")
    return entries


def verify_archive_binding(
    source_receipt: Mapping[str, Any],
    rows: list[Mapping[str, Any]],
    *,
    archive_path: Path | None = None,
) -> dict[str, Any]:
    """Independently bind selected release bytes to unique ZIP members."""

    raw_path = source_receipt.get("archive_path")
    if archive_path is None:
        if not isinstance(raw_path, str) or not raw_path:
            raise VerificationError("archive path is absent from source receipt")
        archive = Path(raw_path).resolve(strict=True)
    else:
        archive = archive_path.resolve(strict=True)
    expected_bytes = source_receipt.get("archive_bytes")
    expected_sha256 = source_receipt.get("archive_sha256")
    prefix = source_receipt.get("archive_member_prefix")
    if (
        isinstance(expected_bytes, bool)
        or not isinstance(expected_bytes, int)
        or expected_bytes <= 0
        or not isinstance(expected_sha256, str)
        or not re.fullmatch(r"[0-9a-f]{64}", expected_sha256)
        or not isinstance(prefix, str)
        or not prefix.strip("/")
    ):
        raise VerificationError("source archive receipt is incomplete")
    _expect(archive.stat().st_size, expected_bytes, "source archive byte size")
    _expect(sha256_file(archive), expected_sha256, "source archive SHA-256")
    normalized_prefix = prefix.strip("/")
    checked = 0
    with zipfile.ZipFile(archive, "r") as stream:
        members: dict[str, zipfile.ZipInfo] = {}
        for info in stream.infolist():
            if "\\" in info.filename or info.filename in members:
                raise VerificationError(f"duplicate or non-POSIX archive member: {info.filename!r}")
            members[info.filename] = info
        for row in rows:
            for relative, expected in _source_entries(row):
                member_name = f"{normalized_prefix}/{relative}"
                info = members.get(member_name)
                if info is None or info.is_dir():
                    raise VerificationError(f"archive member missing: {member_name}")
                digest = hashlib.sha256()
                with stream.open(info, "r") as member:
                    for block in iter(lambda: member.read(4 * 1024 * 1024), b""):
                        digest.update(block)
                _expect(digest.hexdigest(), expected, f"archive member hash {member_name}")
                checked += 1
    return {
        "archive_path": str(archive),
        "archive_bytes": expected_bytes,
        "archive_sha256": expected_sha256,
        "checked_member_count": checked,
    }


def _table4_package_binding(row: Mapping[str, Any]) -> dict[str, Any]:
    entries = _source_entries(row)[:-1]
    file_rows = [
        {
            "path": relative,
            "bytes": (DATASET_ROOT / relative).stat().st_size,
            "sha256": digest,
        }
        for relative, digest in sorted(entries)
    ]
    return {
        "file_count": len(file_rows),
        "total_bytes": sum(record["bytes"] for record in file_rows),
        "content_manifest_sha256": canonical_sha256(file_rows),
    }


def verify_source_row(row: Mapping[str, Any], record: Mapping[str, Any], index: int) -> dict[str, Any]:
    dataset_id = str(row["dataset_id"])
    _expect(record.get("selection_index"), index, f"{dataset_id} selection index")
    _expect(record.get("dataset_id"), dataset_id, f"{dataset_id} record identity")
    _expect(record.get("rank"), index + 1, f"{dataset_id} rank")
    _expect(record.get("rank_sha256"), row["rank_sha256"], f"{dataset_id} rank hash")
    _expect(record.get("manifest_row_sha256"), canonical_sha256(row), f"{dataset_id} row hash")
    for relative, expected_hash in _source_entries(row):
        path = DATASET_ROOT.joinpath(*_safe_relative(relative).parts)
        try:
            resolved = path.resolve(strict=True)
            resolved.relative_to(DATASET_ROOT.resolve(strict=True))
        except (FileNotFoundError, ValueError, OSError) as exc:
            raise VerificationError(f"{dataset_id} source path invalid: {relative}") from exc
        if resolved.is_symlink() or not resolved.is_file():
            raise VerificationError(f"{dataset_id} source path is not a regular file: {relative}")
        _expect(sha256_file(resolved), expected_hash, f"{dataset_id} source hash {relative}")
    urdf = DATASET_ROOT / str(row["urdf_relative_path"])
    root = ET.parse(urdf).getroot()
    if _local_tag(root) != "robot":
        raise VerificationError(f"{dataset_id} URDF root is not robot")
    declared_collision = sum(len(_children(link, "collision")) for link in _children(root, "link"))
    eligible_pairs = _eligible_pairs(root)
    names = [Path(relative).name.lower() for relative, _ in _source_entries(row)]
    receipt_candidates = sum(name.endswith(".json") and RECEIPT_NAME_RE.search(name) is not None for name in names)
    rebuild_candidates = sum(name in REBUILD_RECIPE_NAMES for name in names)
    allowance_candidates = sum(name.endswith(".json") and ALLOWANCE_NAME_RE.search(name) is not None for name in names)
    _expect(record.get("status"), "completed", f"{dataset_id} status")
    _expect(record.get("binding_verified"), True, f"{dataset_id} binding")
    _expect(record.get("declared_collision_elements"), declared_collision, f"{dataset_id} collision count")
    _expect(record.get("receipt_candidate_count"), receipt_candidates, f"{dataset_id} receipt candidate count")
    _expect(record.get("rebuild_recipe_candidate_count"), rebuild_candidates, f"{dataset_id} rebuild candidate count")
    _expect(record.get("allowance_candidate_count"), allowance_candidates, f"{dataset_id} allowance candidate count")
    _expect(record.get("eligible_nonadjacent_pairs"), eligible_pairs, f"{dataset_id} eligible pairs")
    _expect(record.get("package_binding"), _table4_package_binding(row), f"{dataset_id} package binding")
    if receipt_candidates == 0:
        _expect(record.get("receipt_bound"), False, f"{dataset_id} receipt outcome")
        _expect(record.get("receipt_replay_pass"), False, f"{dataset_id} replay outcome")
    if rebuild_candidates == 0:
        _expect(record.get("rebuild_eligible"), False, f"{dataset_id} rebuild eligibility")
        _expect(record.get("rebuild_match"), None, f"{dataset_id} rebuild match")
    if allowance_candidates == 0:
        _expect(record.get("allowance_complete"), True, f"{dataset_id} allowance completeness")
        _expect(record.get("registered_allowance_pairs"), 0, f"{dataset_id} registered pairs")
    return {
        "declared_collision_elements": declared_collision,
        "eligible_nonadjacent_pairs": eligible_pairs,
        "receipt_candidate_count": receipt_candidates,
        "rebuild_candidate_count": rebuild_candidates,
        "allowance_candidate_count": allowance_candidates,
    }


def reaggregate_table4_asset(
    item: Mapping[str, Any],
    asset: Mapping[str, Any],
    states: list[dict[str, Any]],
) -> dict[str, Any]:
    dataset_id = str(item["dataset_id"])
    _expect(asset.get("state_records_sha256"), canonical_sha256(states), f"{dataset_id} state hash")
    phases = {"rest": [], "single_joint_sweep": [], "multi_joint_sobol": []}
    for state in states:
        _expect(str(state.get("dataset_id")), dataset_id, f"{dataset_id} state identity")
        _expect(state.get("order"), item.get("order"), f"{dataset_id} state order")
        _expect(state.get("protocol_id"), item.get("protocol_id"), f"{dataset_id} state protocol")
        phase = state.get("phase")
        if phase not in phases:
            raise VerificationError(f"{dataset_id} unknown state phase: {phase!r}")
        _nonnegative_int(state.get("non_adjacent_illegal_penetration_count"), f"{dataset_id} illegal count")
        phases[str(phase)].append(state)
    expected = {
        "rest": _nonnegative_int(item.get("rest_state_expected"), "rest expected"),
        "single_joint_sweep": _nonnegative_int(item.get("single_state_expected"), "single expected"),
        "multi_joint_sobol": _nonnegative_int(item.get("sobol_state_expected"), "sobol expected"),
    }
    executed = {phase: len(rows) for phase, rows in phases.items()}
    free = {phase: sum(row["non_adjacent_illegal_penetration_count"] == 0 for row in rows) for phase, rows in phases.items()}
    movable = _nonnegative_int(item.get("movable_dof_count"), "movable DoF")
    range_evaluable = _nonnegative_int(asset.get("range_evaluable_dof_count"), "range-evaluable DoF")
    rest_pass = executed["rest"] == expected["rest"] == free["rest"]
    single_pass = executed["single_joint_sweep"] == expected["single_joint_sweep"] == free["single_joint_sweep"]
    sobol_pass = bool(movable > 0 and range_evaluable == movable and executed["multi_joint_sobol"] == expected["multi_joint_sobol"] == free["multi_joint_sobol"])
    complete = bool(range_evaluable == movable and sum(executed.values()) == sum(expected.values()))
    strict = bool(complete and rest_pass and single_pass and sobol_pass)
    expected_fields = {
        "rest_state_executed": executed["rest"],
        "rest_non_adjacent_free": free["rest"],
        "rest_non_adjacent_cf": rest_pass,
        "single_state_executed": executed["single_joint_sweep"],
        "single_non_adjacent_free": free["single_joint_sweep"],
        "single_joint_sweep_cf": single_pass,
        "sobol_state_executed": executed["multi_joint_sobol"],
        "sobol_non_adjacent_free": free["multi_joint_sobol"],
        "multi_joint_sobol_cf": sobol_pass,
        "measurement_complete": complete,
        "strict_collision_pass": strict,
    }
    for field, value in expected_fields.items():
        _expect(asset.get(field), value, f"{dataset_id} Table 4 {field}")
    return {
        "strict_collision_pass": strict,
        "measurement_complete": complete,
        "state_record_count": len(states),
        "state_records_sha256": asset["state_records_sha256"],
        "table4_asset_record_sha256": canonical_sha256(asset),
    }


def load_table4(rows: list[Mapping[str, Any]], *, formal: bool) -> list[dict[str, Any]]:
    for path, expected in (
        (TABLE4_MANIFEST, EXPECTED_TABLE4_MANIFEST_SHA256),
        (TABLE4_ASSET_RECORDS, EXPECTED_TABLE4_ASSET_RECORDS_SHA256),
        (TABLE4_STATE_RECORDS, EXPECTED_TABLE4_STATE_RECORDS_SHA256),
        (TABLE4_ARTIFACT_MANIFEST, EXPECTED_TABLE4_ARTIFACT_MANIFEST_SHA256),
    ):
        _expect(sha256_file(path), expected, f"Table 4 {path.name} hash")
    artifact_receipt = load_json(TABLE4_ARTIFACT_MANIFEST)
    for name, digest in (
        ("manifest.json", EXPECTED_TABLE4_MANIFEST_SHA256),
        ("asset_records.json", EXPECTED_TABLE4_ASSET_RECORDS_SHA256),
        ("state_records.jsonl", EXPECTED_TABLE4_STATE_RECORDS_SHA256),
    ):
        _expect(artifact_receipt.get("files", {}).get(name, {}).get("sha256"), digest, f"artifact receipt {name}")
    manifest = load_json(TABLE4_MANIFEST)
    assets = load_json(TABLE4_ASSET_RECORDS)
    states = load_jsonl(TABLE4_STATE_RECORDS)
    _expect(manifest.get("protocol_id"), EXPECTED_TABLE4_PROTOCOL_ID, "Table 4 protocol")
    _expect(manifest.get("manifest_content_sha256"), canonical_sha256({key: value for key, value in manifest.items() if key != "manifest_content_sha256"}), "Table 4 self-hash")
    items = manifest.get("items")
    if not isinstance(items, list) or not isinstance(assets, list) or len(items) != N_EVAL or len(assets) != N_EVAL:
        raise VerificationError("Table 4 item/asset denominator is not 800")
    states_by_order = {index: [] for index in range(N_EVAL)}
    previous = -1
    for state in states:
        order = _nonnegative_int(state.get("order"), "state order")
        if order not in states_by_order or order < previous:
            raise VerificationError("Table 4 state order is invalid")
        previous = order
        states_by_order[order].append(state)
    if formal:
        _expect(len(states), EXPECTED_FORMAL_STATE_COUNT, "formal state count")
    results: list[dict[str, Any]] = []
    for index, row in enumerate(rows):
        item, asset = items[index], assets[index]
        dataset_id = str(row["dataset_id"])
        _expect(str(item.get("dataset_id")), dataset_id, f"{dataset_id} Table 4 item identity")
        _expect(str(asset.get("dataset_id")), dataset_id, f"{dataset_id} Table 4 asset identity")
        _expect(item.get("order"), index, f"{dataset_id} Table 4 order")
        _expect(item.get("manifest_row_sha256"), canonical_sha256(row), f"{dataset_id} Table 4 row binding")
        _expect(item.get("package_binding"), _table4_package_binding(row), f"{dataset_id} Table 4 package binding")
        embedded = asset.get("state_records")
        if not isinstance(embedded, list) or embedded != states_by_order[index]:
            raise VerificationError(f"{dataset_id} embedded/external states mismatch")
        results.append(reaggregate_table4_asset(item, asset, states_by_order[index]))
    if formal:
        _expect(sum(result["strict_collision_pass"] for result in results), EXPECTED_FORMAL_STRICT_PASSED, "formal strict count")
    return results


def recompute_metrics(records: list[Mapping[str, Any]], *, intended_assets: int) -> dict[str, Any]:
    completed = [record for record in records if record.get("status") == "completed" and record.get("binding_verified") is True]
    receipt = sum(bool(record.get("receipt_bound")) for record in completed)
    replay = sum(bool(record.get("receipt_replay_pass")) for record in completed)
    rebuild_eligible = sum(bool(record.get("rebuild_eligible")) for record in completed)
    rebuild_match = sum(bool(record.get("rebuild_match")) for record in completed if record.get("rebuild_eligible"))
    allowance_complete = len(completed) == intended_assets and all(record.get("allowance_complete") is True for record in completed)
    measured = sum(record.get("allowance_complete") is True for record in completed)
    registered = sum(int(record.get("registered_allowance_pairs") or 0) for record in completed if record.get("allowance_complete") is True)
    eligible = sum(int(record.get("eligible_nonadjacent_pairs") or 0) for record in completed if record.get("allowance_complete") is True)
    strict = sum(bool(record.get("strict_pass_no_method_allowance")) for record in completed)
    status_counts: dict[str, int] = {}
    for record in records:
        status = str(record.get("status", "missing"))
        status_counts[status] = status_counts.get(status, 0) + 1
    if not allowance_complete:
        allowance = {"status": "PARTIAL", "registered_pairs": registered, "eligible_pairs": eligible, "rate": None, "percentage": None, "measured_assets": measured, "intended_assets": intended_assets, "reason": "allowance extraction incomplete"}
    elif eligible == 0:
        allowance = {"status": "N/E", "registered_pairs": registered, "eligible_pairs": 0, "rate": None, "percentage": None, "measured_assets": measured, "intended_assets": intended_assets, "reason": "no eligible non-adjacent collision-bearing source-link pairs"}
    else:
        allowance = {"status": "COMPLETE", "registered_pairs": registered, "eligible_pairs": eligible, "rate": registered / eligible, "percentage": 100.0 * registered / eligible, "measured_assets": measured, "intended_assets": intended_assets}
    return {
        "status_counts": dict(sorted(status_counts.items())),
        "claim_boundary": {"declared_collision_element_total": sum(int(record.get("declared_collision_elements", 0)) for record in completed), "strict_collision_outcome_is_vacuous": True},
        "metrics": {
            "receipt_bound_assets": _ratio(receipt, intended_assets),
            "receipt_replay_pass": _ratio(replay, intended_assets),
            "deterministic_rebuild_match": {"status": "N/E" if rebuild_eligible == 0 else "COMPLETE", "passed": None if rebuild_eligible == 0 else rebuild_match, "denominator": rebuild_eligible, "rate": None if rebuild_eligible == 0 else rebuild_match / rebuild_eligible, "percentage": None if rebuild_eligible == 0 else 100.0 * rebuild_match / rebuild_eligible, "eligible_assets": rebuild_eligible, "asset_denominator": intended_assets},
            "allowance_density": allowance,
            "strict_pass_no_method_allowance": _ratio(strict, intended_assets),
            "registered_allowance_gain_pp": {"status": "COMPLETE" if registered == 0 and allowance_complete else "NOT_EVALUABLE", "value": 0.0 if registered == 0 and allowance_complete else None, "registered_passed": strict if registered == 0 and allowance_complete else None, "no_allowance_passed": strict, "denominator": intended_assets},
        },
    }


def verify_aggregates(records: list[Mapping[str, Any]], summary: Mapping[str, Any]) -> None:
    intended = summary.get("n_eval")
    if isinstance(intended, bool) or not isinstance(intended, int) or intended < 1:
        raise VerificationError("summary n_eval is invalid")
    recomputed = recompute_metrics(records, intended_assets=intended)
    for key in ("status_counts", "claim_boundary", "metrics"):
        if summary.get(key) != recomputed[key]:
            raise VerificationError(f"aggregate mismatch: {key}")


def verify_output_manifest(
    output: Path,
    manifest: Mapping[str, Any],
    frozen: Mapping[str, Any],
    summary: Mapping[str, Any],
) -> None:
    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, Mapping) or set(artifacts) != REQUIRED_MANIFEST_ARTIFACTS:
        raise VerificationError("manifest artifact key set mismatch")
    expected_metadata = {
        "protocol_id": frozen.get("protocol_id"),
        "classification": frozen.get("classification"),
        "dataset": frozen.get("dataset"),
        "n_eval": frozen.get("cohort", {}).get("n_eval"),
        "cohort": frozen.get("cohort"),
        "table4_source": frozen.get("table4_source"),
        "code_identity": frozen.get("code_identity"),
    }
    for key, expected in expected_metadata.items():
        _expect(manifest.get(key), expected, f"manifest/frozen {key}")
    for key in ("protocol_id", "classification", "dataset", "n_eval"):
        _expect(summary.get(key), expected_metadata[key], f"summary/frozen {key}")
    _expect(manifest.get("post_manifest_receipts"), POST_MANIFEST_RECEIPTS, "post-manifest receipt policy")
    for name in sorted(REQUIRED_MANIFEST_ARTIFACTS):
        path = output / name
        if not path.is_file():
            raise VerificationError(f"missing manifest artifact: {name}")
        _expect(
            artifacts[name],
            {"bytes": path.stat().st_size, "sha256": sha256_file(path)},
            f"manifest artifact {name}",
        )
    _expect(
        manifest.get("manifest_content_sha256"),
        canonical_sha256(
            {key: value for key, value in manifest.items() if key != "manifest_content_sha256"}
        ),
        "manifest self-hash",
    )


def verify_run(run_directory: Path, *, formal: bool) -> dict[str, Any]:
    output = run_directory.resolve(strict=True)
    required = ("protocol_snapshot.md", "frozen_config.json", "asset_records.jsonl", "summary.json", "summary.md", "manifest.json")
    for name in required:
        if not (output / name).is_file():
            raise VerificationError(f"missing run artifact: {name}")
    frozen = load_json(output / "frozen_config.json")
    summary = load_json(output / "summary.json")
    manifest = load_json(output / "manifest.json")
    records = load_jsonl(output / "asset_records.jsonl")
    checks: list[dict[str, Any]] = []

    def passed(name: str) -> None:
        checks.append({"check": name, "pass": True})

    _expect(frozen.get("protocol_id"), PROTOCOL_ID, "frozen protocol")
    _expect(summary.get("protocol_id"), PROTOCOL_ID, "summary protocol")
    _expect(manifest.get("protocol_id"), PROTOCOL_ID, "manifest protocol")
    _expect(frozen.get("dataset"), DATASET, "frozen dataset")
    passed("run_identity")

    expected_n = N_EVAL if formal else summary.get("n_eval")
    _expect(len(records), expected_n, "record denominator")
    _expect(summary.get("n_eval"), expected_n, "summary denominator")
    _expect(manifest.get("n_eval"), expected_n, "manifest denominator")
    if formal:
        _expect(summary.get("classification"), "FORMAL", "formal classification")
        _expect(frozen.get("cohort", {}).get("archive_binding_verified"), True, "archive binding")
    passed("denominator_and_classification")

    _expect(sha256_file(SOURCE_MANIFEST), EXPECTED_SOURCE_MANIFEST_SHA256, "source manifest hash")
    source = load_json(SOURCE_MANIFEST)
    rows_all = source.get("rows")
    if not isinstance(rows_all, list) or len(rows_all) != N_EVAL:
        raise VerificationError("source manifest does not contain 800 rows")
    _expect(canonical_sha256([str(row["dataset_id"]) for row in rows_all]), EXPECTED_ORDERED_IDS_SHA256, "ordered IDs")
    rows = rows_all[:expected_n]
    passed("source_cohort_binding")

    if formal:
        verify_archive_binding(source.get("source_receipt", {}), rows)
        passed("independent_archive_binding")

    independent_source = [verify_source_row(row, record, index) for index, (row, record) in enumerate(zip(rows, records, strict=True))]
    if formal:
        _expect(sum(value["declared_collision_elements"] for value in independent_source), 0, "formal collision element total")
        _expect(sum(value["eligible_nonadjacent_pairs"] for value in independent_source), 0, "formal eligible pair total")
        _expect(sum(value["receipt_candidate_count"] for value in independent_source), 0, "formal receipt candidates")
        _expect(sum(value["rebuild_candidate_count"] for value in independent_source), 0, "formal rebuild candidates")
        _expect(sum(value["allowance_candidate_count"] for value in independent_source), 0, "formal allowance candidates")
    passed("independent_release_evidence")

    table4_results = load_table4(rows, formal=formal)
    for record, table4 in zip(records, table4_results, strict=True):
        dataset_id = record["dataset_id"]
        _expect(record.get("strict_pass_no_method_allowance"), table4["strict_collision_pass"], f"{dataset_id} strict binding")
        _expect(record.get("table4_measurement_complete"), table4["measurement_complete"], f"{dataset_id} completeness binding")
        _expect(record.get("table4_state_record_count"), table4["state_record_count"], f"{dataset_id} state count binding")
        _expect(record.get("table4_state_records_sha256"), table4["state_records_sha256"], f"{dataset_id} state hash binding")
        _expect(record.get("table4_asset_record_sha256"), table4["table4_asset_record_sha256"], f"{dataset_id} asset hash binding")
    passed("independent_table4_reaggregation")

    verify_aggregates(records, summary)
    passed("aggregate_recomputation")
    if formal:
        metrics = summary["metrics"]
        _expect(metrics["receipt_bound_assets"]["passed"], 0, "formal receipt result")
        _expect(metrics["receipt_replay_pass"]["passed"], 0, "formal replay result")
        _expect(metrics["deterministic_rebuild_match"]["eligible_assets"], 0, "formal rebuild eligibility")
        _expect(metrics["allowance_density"]["status"], "N/E", "formal allowance status")
        _expect(metrics["allowance_density"]["eligible_pairs"], 0, "formal allowance denominator")
        _expect(metrics["strict_pass_no_method_allowance"]["passed"], EXPECTED_FORMAL_STRICT_PASSED, "formal strict result")
        _expect(metrics["registered_allowance_gain_pp"]["value"], 0.0, "formal gain")
        passed("formal_result_contract")

    code_identity = frozen.get("code_identity", {})
    _expect(code_identity.get("verifier_sha256"), sha256_file(SCRIPT), "verifier identity")
    for key in ("runner", "static_atoms", "cohort_loader", "table5_contract"):
        path = Path(str(code_identity.get(f"{key}_path", ""))).resolve(strict=True)
        _expect(code_identity.get(f"{key}_sha256"), sha256_file(path), f"{key} identity")
    passed("code_identity")

    snapshot = output / "protocol_snapshot.md"
    _expect(frozen.get("protocol_document", {}).get("sha256_at_freeze"), sha256_file(snapshot), "protocol snapshot")
    passed("protocol_snapshot_binding")

    verify_output_manifest(output, manifest, frozen, summary)
    allowed_output_names = set(REQUIRED_MANIFEST_ARTIFACTS) | {
        "manifest.json",
        *POST_MANIFEST_RECEIPTS,
    }
    observed_output_names = {path.name for path in output.iterdir()}
    if not observed_output_names <= allowed_output_names:
        raise VerificationError(
            f"unexpected output entries: {sorted(observed_output_names - allowed_output_names)}"
        )
    passed("output_artifact_closure")
    return {
        "schema_version": "s1-physx-mobility-verification/v1",
        "protocol_id": PROTOCOL_ID,
        "run_directory": str(output),
        "verified_at": dt.datetime.now(dt.timezone.utc).isoformat().replace("+00:00", "Z"),
        "formal": formal,
        "all_pass": True,
        "status": "PASS",
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
        result = verify_run(args.run, formal=args.formal)
    except Exception as exc:  # noqa: BLE001
        print(f"verification failed: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    path = args.run.resolve() / "verification.json"
    path.write_text(json.dumps(result, indent=2, sort_keys=True, ensure_ascii=True, allow_nan=False) + "\n", encoding="utf-8")
    print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
