#!/usr/bin/env python3
"""Independent verifier for the Artiverse Supplementary Table S1 run."""

from __future__ import annotations

import argparse
from collections import deque
import hashlib
import json
from pathlib import Path, PurePosixPath, PureWindowsPath
import re
import shlex
from typing import Any, Mapping
import xml.etree.ElementTree as ET


PROTOCOL_ID = "s1_artiverse_table1cohort_n800_seed20260813_v1"
RECEIPT_NAME_RE = re.compile(r"(?:^|[-_])(?:mechanical[-_])?receipt(?:[-_]|\.|$)", re.IGNORECASE)
ALLOWANCE_NAME_RE = re.compile(r"allow(?:ance|list|ed)|exclu(?:de|sion)", re.IGNORECASE)
REBUILD_RECIPE_NAMES = frozenset({
    "build_recipe.json", "build-recipe.json", "rebuild_recipe.json",
    "rebuild-recipe.json", "deterministic_rebuild.json",
})
MAX_EVIDENCE_JSON_BYTES = 8 * 1024 * 1024
S1_IDENTITY_FIELDS = (
    "selection_index", "asset_id", "manifest_root", "dataset_id", "model_id",
    "raw_category", "source", "selection_rank", "package",
    "primary_urdf_relative_path", "urdf_sha256_expected",
    "collision_mesh_files_expected", "table4_input_identity_sha256",
    "strict_pass_no_method_allowance",
)


def canonical_json(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            if line.strip():
                row = json.loads(line)
                if not isinstance(row, dict):
                    raise ValueError("asset record is not an object")
                rows.append(row)
    return rows


def same_resolved_path(left: Path, right: Path) -> bool:
    try:
        return left.resolve(strict=True) == right.resolve(strict=True)
    except OSError:
        return False


def ratio(passed: int, denominator: int) -> dict[str, int | float | None]:
    rate = passed / denominator if denominator else None
    return {
        "passed": passed,
        "denominator": denominator,
        "rate": rate,
        "percentage": None if rate is None else rate * 100.0,
    }


def local_tag(node: ET.Element) -> str:
    return node.tag.rsplit("}", 1)[-1]


def children(node: ET.Element, name: str) -> list[ET.Element]:
    return [child for child in node if local_tag(child) == name]


def independently_eligible_pairs(root: ET.Element) -> set[tuple[str, str]]:
    links = children(root, "link")
    collision_names = [
        link.attrib.get("name", "").strip()
        for link in links
        if children(link, "collision")
    ]
    pairs = {
        tuple(sorted((collision_names[left], collision_names[right])))
        for left in range(len(collision_names))
        for right in range(left + 1, len(collision_names))
    }
    adjacent: set[tuple[str, str]] = set()
    declared_names = {link.attrib.get("name", "").strip() for link in links}
    for joint in children(root, "joint"):
        parent = children(joint, "parent")
        child = children(joint, "child")
        if len(parent) != 1 or len(child) != 1:
            continue
        names = (parent[0].attrib.get("link", "").strip(), child[0].attrib.get("link", "").strip())
        if names[0] in declared_names and names[1] in declared_names and names[0] != names[1]:
            adjacent.add(tuple(sorted(names)))
    return pairs - adjacent


def scan_evidence_candidates(package: Path) -> dict[str, list[dict[str, Any]]]:
    result: dict[str, list[dict[str, Any]]] = {
        "receipt_candidates": [],
        "rebuild_recipe_candidates": [],
        "allowance_candidates": [],
    }
    for path in sorted(package.rglob("*")):
        try:
            if path.is_symlink() or not path.is_file():
                continue
            path.resolve(strict=True).relative_to(package.resolve(strict=True))
        except (OSError, ValueError):
            continue
        lower = path.name.lower()
        entry = {
            "path": path.relative_to(package).as_posix(),
            "sha256": sha256_file(path),
            "size_bytes": path.stat().st_size,
        }
        if path.suffix.lower() == ".json" and RECEIPT_NAME_RE.search(lower):
            result["receipt_candidates"].append(entry)
        if lower in REBUILD_RECIPE_NAMES:
            result["rebuild_recipe_candidates"].append(entry)
        if path.suffix.lower() == ".json" and ALLOWANCE_NAME_RE.search(lower):
            result["allowance_candidates"].append(entry)
    return result


def safe_relative_path(raw: str, *, field: str) -> PurePosixPath:
    if not isinstance(raw, str) or not raw or "\\" in raw or "\x00" in raw:
        raise ValueError(f"invalid_{field}: {raw!r}")
    if raw.startswith("/") or PureWindowsPath(raw).is_absolute():
        raise ValueError(f"unsafe_{field}: {raw!r}")
    parts: list[str] = []
    for part in raw.split("/"):
        if part == "":
            raise ValueError(f"noncanonical_{field}: {raw!r}")
        if part == ".":
            continue
        if part == "..":
            if parts and parts[-1] != "..":
                parts.pop()
            else:
                parts.append(part)
        else:
            parts.append(part)
    if not parts:
        raise ValueError(f"invalid_{field}: {raw!r}")
    return PurePosixPath(*parts)


def resolve_resource(
    package: Path,
    declaring_file: Path,
    raw: str,
    *,
    field: str,
    permit_generation_config: bool = False,
) -> tuple[Path | None, str | None]:
    try:
        relative = safe_relative_path(raw.strip(), field=field)
    except (AttributeError, ValueError) as exc:
        return None, str(exc)
    if re.fullmatch(r"generation_config(?:\..+)?", relative.name, re.IGNORECASE) and not permit_generation_config:
        return None, f"generation_config_not_readable_as_resource: {relative.as_posix()}"
    candidate = declaring_file.parent.joinpath(*relative.parts)
    if candidate.is_symlink():
        return None, f"symlink_{field}: {relative.as_posix()}"
    try:
        resolved = candidate.resolve(strict=True)
        resolved.relative_to(package)
    except FileNotFoundError:
        return None, f"missing_{field}: {relative.as_posix()}"
    except (OSError, ValueError):
        return None, f"escaping_{field}: {relative.as_posix()}"
    if not resolved.is_file():
        return None, f"not_file_{field}: {relative.as_posix()}"
    try:
        if resolved.stat().st_size <= 0:
            return None, f"empty_{field}: {relative.as_posix()}"
    except OSError as exc:
        return None, f"unreadable_{field}: {type(exc).__name__}: {exc}"
    return resolved, None


def descendants(node: ET.Element, name: str) -> list[ET.Element]:
    return [item for item in node.iter() if local_tag(item) == name]


def nested_resource_specs(path: Path) -> tuple[list[tuple[str, str]], str | None]:
    try:
        if path.suffix.lower() == ".obj":
            specs: list[tuple[str, str]] = []
            for line in path.read_text(encoding="utf-8").splitlines():
                tokens = shlex.split(line, comments=True)
                if tokens and tokens[0].lower() == "mtllib":
                    specs.extend(("obj_mtl", token) for token in tokens[1:])
            return specs, None
        if path.suffix.lower() == ".mtl":
            specs = []
            for line in path.read_text(encoding="utf-8").splitlines():
                tokens = shlex.split(line, comments=True)
                if len(tokens) >= 2 and tokens[0].lower() in {
                    "map_ka", "map_kd", "map_ks", "map_ke", "map_d", "bump",
                    "map_bump", "disp", "decal", "norm",
                }:
                    specs.append(("mtl_resource", tokens[-1]))
            return specs, None
        if path.suffix.lower() == ".gltf":
            payload = load_json(path)
            if not isinstance(payload, dict):
                return [], "gltf_root_not_object"
            specs = []
            for section in ("buffers", "images"):
                entries = payload.get(section, [])
                if not isinstance(entries, list):
                    return [], f"gltf_{section}_not_list"
                for entry in entries:
                    uri = entry.get("uri") if isinstance(entry, dict) else None
                    if isinstance(uri, str) and not uri.startswith("data:"):
                        specs.append((f"gltf_{section[:-1]}", uri))
            return specs, None
        if path.suffix.lower() == ".dae":
            dae_root = ET.parse(path).getroot()
            specs = []
            for image_node in descendants(dae_root, "image"):
                for node in image_node.iter():
                    if local_tag(node) == "init_from" and node.text and node.text.strip():
                        raw = node.text.strip()
                        if not raw.startswith("#"):
                            specs.append(("dae_image", raw))
            return specs, None
    except Exception as exc:  # noqa: BLE001
        return [], f"nested_resource_parse_failed: {type(exc).__name__}: {exc}"
    return [], None


def independently_resource_closure(root: ET.Element, package: Path, urdf: Path) -> dict[str, Any]:
    package = package.resolve(strict=True)
    urdf = urdf.resolve(strict=True)
    queue: deque[tuple[str, str, Path]] = deque()
    queue.extend(("urdf_mesh", node.attrib.get("filename", ""), urdf) for node in descendants(root, "mesh"))
    queue.extend(("urdf_texture", node.attrib.get("filename", ""), urdf) for node in descendants(root, "texture"))
    relative_urdf = urdf.relative_to(package).as_posix()
    records: dict[str, dict[str, Any]] = {
        relative_urdf: {"path": relative_urdf, "sha256": sha256_file(urdf)}
    }
    issues: list[str] = []
    visited_edges: set[tuple[str, str, str]] = set()
    expanded: set[Path] = set()
    while queue:
        kind, raw, declaring = queue.popleft()
        edge = (kind, raw, declaring.relative_to(package).as_posix())
        if edge in visited_edges:
            continue
        visited_edges.add(edge)
        resolved, issue = resolve_resource(package, declaring, raw, field=kind)
        if issue is not None:
            issues.append(f"{edge[2]}:{kind}: {issue}")
            continue
        assert resolved is not None
        relative = resolved.relative_to(package).as_posix()
        records.setdefault(relative, {"path": relative, "sha256": sha256_file(resolved)})
        if resolved in expanded:
            continue
        expanded.add(resolved)
        nested, nested_issue = nested_resource_specs(resolved)
        if nested_issue:
            issues.append(f"{relative}: {nested_issue}")
        else:
            queue.extend((nested_kind, nested_raw, resolved) for nested_kind, nested_raw in nested)
    ordered = [records[key] for key in sorted(records)]
    complete = not issues
    return {
        "status": "COMPLETE" if complete else "PARTIAL",
        "complete": complete,
        "file_count": len(ordered),
        "sha256": hashlib.sha256(canonical_json(ordered).encode("utf-8")).hexdigest() if complete else None,
        "files": ordered,
        "issues": issues,
    }


def first_present(payload: Any, paths: tuple[tuple[str, ...], ...]) -> Any:
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


def load_evidence_json(path: Path) -> tuple[Any | None, str | None]:
    try:
        size = path.stat().st_size
        if size <= 0 or size > MAX_EVIDENCE_JSON_BYTES:
            return None, f"evidence_json_size_invalid: {size}"
        return load_json(path), None
    except Exception as exc:  # noqa: BLE001
        return None, f"evidence_json_parse_failed: {type(exc).__name__}: {exc}"


def independently_validate_receipt(payload: Any, urdf_sha: str, closure_sha: str | None) -> dict[str, Any]:
    if not isinstance(payload, Mapping):
        return {"valid": False, "missing": ["object_root"], "mismatches": []}
    fields = {
        "asset_binding": first_present(payload, (("asset_sha256",), ("urdf_sha256",), ("asset", "urdf_sha256"), ("bindings", "asset_sha256"))),
        "resource_closure_binding": first_present(payload, (("resource_closure_sha256",), ("bindings", "resource_closure_sha256"), ("resources", "closure_sha256"))),
        "protocol_identity": first_present(payload, (("protocol_id",), ("protocol", "id"))),
        "runner_identity": first_present(payload, (("runner_sha256",), ("runner_id",), ("runner", "sha256"), ("runner", "id"))),
        "pair_policy": first_present(payload, (("pair_policy",), ("protocol", "pair_policy"))),
        "thresholds": first_present(payload, (("thresholds",), ("protocol", "thresholds"))),
        "conclusion": first_present(payload, (("conclusion",), ("verdict",), ("result", "pass"))),
    }
    missing = [name for name, value in fields.items() if value is None or value == "" or value == {}]
    mismatches: list[str] = []
    if fields["asset_binding"] is not None and fields["asset_binding"] != urdf_sha:
        mismatches.append("asset_sha256_mismatch")
    if fields["resource_closure_binding"] is not None and fields["resource_closure_binding"] != closure_sha:
        mismatches.append("resource_closure_sha256_mismatch")
    if closure_sha is None:
        mismatches.append("resource_closure_incomplete")
    return {"valid": not missing and not mismatches, "missing": missing, "mismatches": mismatches}


def independently_validate_rebuild(payload: Any, package: Path) -> dict[str, Any]:
    if not isinstance(payload, Mapping):
        return {"eligible": False, "issues": ["recipe_root_not_object"]}
    issues: list[str] = []
    runner = first_present(payload, (("runner", "sha256"), ("runner", "id"), ("runner_sha256",), ("runner_id",)))
    inputs = payload.get("inputs")
    output = payload.get("output")
    if not runner:
        issues.append("runner_identity_missing")
    if not isinstance(inputs, list) or not inputs:
        issues.append("complete_inputs_missing")
        inputs = []
    if not isinstance(output, Mapping) or not first_present(output, (("canonical_fingerprint_sha256",), ("sha256",))):
        issues.append("output_fingerprint_missing")
    checked = 0
    for index, item in enumerate(inputs):
        if not isinstance(item, Mapping):
            issues.append(f"input_{index}_not_object")
            continue
        raw_path, expected_hash = item.get("path"), item.get("sha256")
        if not isinstance(raw_path, str) or not isinstance(expected_hash, str):
            issues.append(f"input_{index}_binding_incomplete")
            continue
        resolved, issue = resolve_resource(
            package, package / "recipe-anchor", raw_path,
            field=f"rebuild_input_{index}", permit_generation_config=True,
        )
        if issue:
            issues.append(f"input_{index}: {issue}")
        elif resolved is not None and sha256_file(resolved) != expected_hash:
            issues.append(f"input_{index}_sha256_mismatch")
        else:
            checked += 1
    return {
        "eligible": not issues and checked == len(inputs),
        "checked_input_count": checked,
        "declared_input_count": len(inputs),
        "issues": issues,
    }


def independently_evidence_results(
    package: Path,
    urdf: Path,
    closure: Mapping[str, Any],
    inventory: Mapping[str, list[dict[str, Any]]],
) -> dict[str, Any]:
    receipts: list[dict[str, Any]] = []
    for entry in inventory["receipt_candidates"]:
        payload, issue = load_evidence_json(package / entry["path"])
        validation = (
            {"valid": False, "missing": [], "mismatches": [issue]}
            if issue else independently_validate_receipt(payload, sha256_file(urdf), closure.get("sha256"))
        )
        receipts.append({"path": entry["path"], **validation})
    valid_receipts = sum(int(row["valid"]) for row in receipts)
    recipes: list[dict[str, Any]] = []
    for entry in inventory["rebuild_recipe_candidates"]:
        payload, issue = load_evidence_json(package / entry["path"])
        validation = (
            {"eligible": False, "issues": [issue]}
            if issue else independently_validate_rebuild(payload, package)
        )
        recipes.append({"path": entry["path"], **validation})
    rebuild_eligible = any(row["eligible"] for row in recipes)
    return {
        "receipt": {
            "candidate_count": len(receipts),
            "valid_mechanical_receipt_count": valid_receipts,
            "receipt_bound_asset": int(valid_receipts > 0),
            "records": receipts,
            "issues": [
                f"{row['path']}: {issue}"
                for row in receipts
                for issue in [*row.get("missing", []), *row.get("mismatches", [])]
            ],
        },
        "receipt_replay": {
            "eligible_receipt_count": valid_receipts,
            "attempted": 0,
            "passed": False,
            "status": "NOT_RUN_NO_VALID_RECEIPT" if valid_receipts == 0 else "NOT_EVALUABLE_NO_REGISTERED_REPLAY_BACKEND",
        },
        "rebuild": {
            "status": "ELIGIBLE_NOT_RUN" if rebuild_eligible else "N/E",
            "eligible_asset": int(rebuild_eligible),
            "candidate_recipe_count": len(recipes),
            "valid_recipe_count": sum(int(row["eligible"]) for row in recipes),
            "recipes": recipes,
        },
    }


def verify_atomic_record(
    record: Mapping[str, Any],
    frozen: Mapping[str, Any],
    dataset_root: Path,
    expected_inventory: Mapping[str, Any] | None = None,
) -> dict[str, bool]:
    package = dataset_root / str(record.get("manifest_root")) / "urdf_w_collider"
    expected_package = package.as_posix()
    binding = record.get("binding")
    identity_payload = {field: record.get(field) for field in S1_IDENTITY_FIELDS}
    identity_matches = (
        record.get("package") == expected_package
        and record.get("dataset_id") == frozen.get("dataset_id")
        and record.get("manifest_root") == frozen.get("manifest_root")
        and record.get("table4_input_identity_sha256") == frozen.get("input_identity_sha256")
        and record.get("selection_index") == frozen.get("order")
        and record.get("s1_input_identity_sha256")
        == hashlib.sha256(canonical_json(identity_payload).encode("utf-8")).hexdigest()
    )
    urdf = package / str(record.get("primary_urdf_relative_path"))
    source_bytes_match = (
        isinstance(binding, Mapping)
        and binding.get("verified") is True
        and urdf.is_file()
        and not urdf.is_symlink()
        and sha256_file(urdf) == frozen.get("urdf_sha256") == record.get("urdf_sha256_expected")
        and record.get("collision_mesh_files_expected") == frozen.get("collision_mesh_files", [])
    )
    if source_bytes_match:
        for mesh in frozen.get("collision_mesh_files", []):
            if not mesh.get("safe") or not mesh.get("exists"):
                continue
            path = dataset_root / str(mesh.get("resolved_relpath"))
            if not path.is_file() or path.is_symlink() or sha256_file(path) != mesh.get("sha256"):
                source_bytes_match = False
                break

    closure = record.get("resource_closure") if isinstance(record.get("resource_closure"), Mapping) else {}
    try:
        urdf_root = ET.parse(urdf).getroot()
        independent_closure = independently_resource_closure(urdf_root, package, urdf)
        closure_bytes_match = dict(closure) == independent_closure
    except (OSError, ValueError, ET.ParseError):
        urdf_root = None
        independent_closure = {
            "status": "PARTIAL", "complete": False, "file_count": 0,
            "sha256": None, "files": [], "issues": ["independent_closure_failed"],
        }
        closure_bytes_match = False

    live_inventory = scan_evidence_candidates(package)
    evidence = record.get("s1_evidence") if isinstance(record.get("s1_evidence"), Mapping) else {}
    receipt = evidence.get("receipt") if isinstance(evidence.get("receipt"), Mapping) else {}
    rebuild = evidence.get("rebuild") if isinstance(evidence.get("rebuild"), Mapping) else {}
    allowance = evidence.get("allowance") if isinstance(evidence.get("allowance"), Mapping) else {}
    record_inventory = {
        "receipt_candidates": sorted(record.get("path") for record in receipt.get("records", [])),
        "rebuild_recipe_candidates": sorted(record.get("path") for record in rebuild.get("recipes", [])),
        "allowance_candidates": sorted(record.get("path") for record in allowance.get("records", [])),
    }
    live_paths = {key: sorted(entry["path"] for entry in value) for key, value in live_inventory.items()}
    evidence_matches = record_inventory == live_paths
    if expected_inventory is not None:
        evidence_matches = evidence_matches and live_inventory == {
            key: expected_inventory.get(key, []) for key in live_inventory
        }

    independent_evidence = independently_evidence_results(
        package, urdf, independent_closure, live_inventory,
    )
    replay = evidence.get("receipt_replay") if isinstance(evidence.get("receipt_replay"), Mapping) else {}

    def matches_fields(actual: Mapping[str, Any], expected: Mapping[str, Any]) -> bool:
        return all(actual.get(key) == value for key, value in expected.items())

    evidence_atomic_results_match = (
        matches_fields(receipt, independent_evidence["receipt"])
        and matches_fields(replay, independent_evidence["receipt_replay"])
        and matches_fields(rebuild, independent_evidence["rebuild"])
    )

    eligible_count = len(independently_eligible_pairs(urdf_root)) if urdf_root is not None else -1
    return {
        "source_bytes_match": source_bytes_match,
        "s1_input_identity_matches": identity_matches,
        "resource_closure_bytes_match": closure_bytes_match,
        "evidence_inventory_matches": evidence_matches,
        "evidence_atomic_results_match": evidence_atomic_results_match,
        "eligible_pair_count_matches": eligible_count == allowance.get("eligible_nonadjacent_pair_count"),
        "empty_registry_enforced": (
            allowance.get("registration_status") == "NO_PREREGISTERED_METHOD_SPECIFIC_REGISTRY"
            and allowance.get("registered_excluded_pair_count") == 0
        ),
    }


def recompute_metrics(records: list[Mapping[str, Any]], *, intended_assets: int) -> dict[str, Any]:
    receipt_bound = 0
    receipt_replayed = 0
    rebuild_eligible = 0
    rebuild_matched = 0
    rebuild_complete = True
    registered_pairs = 0
    eligible_pairs = 0
    allowance_complete = True
    allowance_measured_assets = 0
    strict_passed = 0
    registered_passed = 0
    registered_outcomes_complete = True

    for record in records:
        status = str(record.get("status", "missing"))
        binding = record.get("binding")
        result_eligible = status == "completed" and (
            not isinstance(binding, Mapping) or binding.get("verified") is True
        )
        evidence = record.get("s1_evidence") if isinstance(record.get("s1_evidence"), Mapping) else {}
        receipt = evidence.get("receipt") if isinstance(evidence.get("receipt"), Mapping) else {}
        replay = evidence.get("receipt_replay") if isinstance(evidence.get("receipt_replay"), Mapping) else {}
        rebuild = evidence.get("rebuild") if isinstance(evidence.get("rebuild"), Mapping) else {}
        allowance = evidence.get("allowance") if isinstance(evidence.get("allowance"), Mapping) else {}
        receipt_bound += int(bool(receipt.get("receipt_bound_asset")))
        receipt_replayed += int(bool(replay.get("passed")))
        eligible = int(bool(rebuild.get("eligible_asset")))
        rebuild_eligible += eligible
        if eligible:
            rebuild_matched += int(bool(record.get("deterministic_rebuild_match")))
            rebuild_complete = rebuild_complete and record.get("rebuild_replay_status") == "COMPLETE"
        if allowance.get("status") != "COMPLETE":
            allowance_complete = False
        else:
            allowance_measured_assets += 1
            registered_pairs += int(allowance.get("registered_excluded_pair_count", 0))
            eligible_pairs += int(allowance.get("eligible_nonadjacent_pair_count", 0))
        strict_passed += int(result_eligible and bool(record.get("strict_pass_no_method_allowance")))
        registered_outcome = record.get("registered_allowance_strict_pass")
        if result_eligible and isinstance(registered_outcome, bool):
            registered_passed += int(registered_outcome)
        else:
            registered_outcomes_complete = False

    rebuild_metric: dict[str, Any] = {
        "status": "N/E" if rebuild_eligible == 0 else ("COMPLETE" if rebuild_complete else "NOT_EVALUABLE"),
        "passed": None if rebuild_eligible == 0 else rebuild_matched,
        "denominator": rebuild_eligible,
        "rate": None if rebuild_eligible == 0 else rebuild_matched / rebuild_eligible,
        "percentage": None if rebuild_eligible == 0 else 100.0 * rebuild_matched / rebuild_eligible,
        "eligible_assets": rebuild_eligible,
        "asset_denominator": intended_assets,
    }
    allowance_rate = registered_pairs / eligible_pairs if eligible_pairs else (0.0 if allowance_complete else None)
    allowance_metric = {
        "status": "COMPLETE" if allowance_complete else "PARTIAL",
        "registered_pairs": registered_pairs,
        "eligible_pairs": eligible_pairs,
        "rate": allowance_rate,
        "percentage": None if allowance_rate is None else allowance_rate * 100.0,
        "measured_assets": allowance_measured_assets,
        "intended_assets": intended_assets,
    }
    if registered_pairs == 0 and allowance_complete:
        gain_metric = {
            "status": "COMPLETE",
            "value": 0.0,
            "registered_passed": strict_passed,
            "no_allowance_passed": strict_passed,
            "denominator": intended_assets,
        }
    elif registered_outcomes_complete:
        gain_metric = {
            "status": "COMPLETE",
            "value": 100.0 * (registered_passed - strict_passed) / intended_assets,
            "registered_passed": registered_passed,
            "no_allowance_passed": strict_passed,
            "denominator": intended_assets,
        }
    else:
        gain_metric = {
            "status": "NOT_EVALUABLE",
            "value": None,
            "registered_passed": None,
            "no_allowance_passed": strict_passed,
            "denominator": intended_assets,
            "reason": "registered allowance exists but no frozen sensitivity replay is available",
        }
    return {
        "receipt_bound_assets": ratio(receipt_bound, intended_assets),
        "receipt_replay_pass": ratio(receipt_replayed, intended_assets),
        "deterministic_rebuild_match": rebuild_metric,
        "allowance_density": allowance_metric,
        "strict_pass_no_method_allowance": ratio(strict_passed, intended_assets),
        "registered_allowance_gain_pp": gain_metric,
    }


def verify_aggregates(
    records: list[Mapping[str, Any]],
    summary: Mapping[str, Any],
    *,
    expected_n: int,
) -> dict[str, bool]:
    n_eval = summary.get("n_eval")
    valid_n = isinstance(n_eval, int) and not isinstance(n_eval, bool) and n_eval >= 0
    expected_metrics = recompute_metrics(records, intended_assets=n_eval if valid_n else len(records))
    return {
        "record_count_matches_n_eval": valid_n and n_eval == expected_n and len(records) == expected_n,
        "selection_order_is_exact": [row.get("selection_index") for row in records] == list(range(len(records))),
        "asset_ids_are_unique": len({row.get("asset_id") for row in records}) == len(records),
        "summary_metrics_recompute_exactly": summary.get("metrics") == expected_metrics,
        "receipt_replay_never_exceeds_bound_assets": (
            expected_metrics["receipt_replay_pass"]["passed"]
            <= expected_metrics["receipt_bound_assets"]["passed"]
        ),
    }


def verify_run(output: Path) -> dict[str, Any]:
    required = (
        "frozen_config.json",
        "environment.json",
        "evidence_inventory.json",
        "protocol_snapshot.md",
        "asset_records.jsonl",
        "summary.json",
        "summary.md",
    )
    checks: dict[str, bool] = {
        "required_outputs_present": all((output / name).is_file() for name in required),
    }
    if not checks["required_outputs_present"]:
        return {"status": "FAIL", "checks": checks}

    config = load_json(output / "frozen_config.json")
    summary = load_json(output / "summary.json")
    records = load_jsonl(output / "asset_records.jsonl")
    cohort = config.get("cohort", {})
    classification = config.get("classification")
    expected_n = 800 if classification == "FORMAL" else cohort.get("n_eval", -1)
    checks.update(verify_aggregates(records, summary, expected_n=expected_n))
    checks["classification_and_cohort_size_match"] = (
        classification in {"FORMAL", "SMOKE"}
        and summary.get("classification") == classification
        and cohort.get("n_eval") == expected_n
        and cohort.get("intended_full_cohort") == 800
        and (classification != "FORMAL" or expected_n == 800)
    )
    checks["protocol_identity_matches"] = (
        config.get("protocol_id") == PROTOCOL_ID and summary.get("protocol_id") == PROTOCOL_ID
    )
    checks["frozen_config_hash_matches_summary"] = (
        summary.get("frozen_config_sha256") == sha256_file(output / "frozen_config.json")
    )
    checks["protocol_snapshot_hash_matches"] = (
        config.get("protocol_snapshot_sha256") == sha256_file(output / "protocol_snapshot.md")
    )
    inventory_config = config.get("evidence_inventory", {})
    try:
        inventory_path = Path(inventory_config["path"])
        frozen_inventory = load_json(inventory_path)
        checks["frozen_evidence_inventory_hash_matches"] = (
            same_resolved_path(inventory_path, output / "evidence_inventory.json")
            and sha256_file(inventory_path) == inventory_config["sha256"]
            and frozen_inventory.get("allowance_registry", {}).get("status") == "ABSENT_FROZEN_EMPTY"
            and frozen_inventory.get("allowance_registry", {}).get("registered_pair_count") == 0
        )
    except (KeyError, OSError, TypeError, ValueError, json.JSONDecodeError):
        frozen_inventory = {"assets": []}
        checks["frozen_evidence_inventory_hash_matches"] = False
    runner = config.get("runner", {})
    static_module = config.get("static_module", {})
    try:
        checks["runner_identity_matches"] = sha256_file(Path(runner["path"])) == runner["sha256"]
    except (KeyError, OSError, TypeError):
        checks["runner_identity_matches"] = False
    try:
        checks["static_module_identity_matches"] = (
            sha256_file(Path(static_module["path"])) == static_module["sha256"]
        )
    except (KeyError, OSError, TypeError):
        checks["static_module_identity_matches"] = False

    try:
        table1_path = Path(cohort["table1_manifest"])
        table4_path = Path(cohort["table4_manifest"])
        table4_records_path = Path(cohort["table4_records"])
        table4_verification_path = Path(cohort["table4_verification"])
        checks["table1_source_hash_matches"] = sha256_file(table1_path) == cohort["table1_manifest_sha256"]
        checks["table4_source_hash_matches"] = sha256_file(table4_path) == cohort["table4_manifest_sha256"]
        checks["table4_records_hash_matches"] = (
            sha256_file(table4_records_path) == cohort["table4_records_sha256"]
        )
        checks["table4_verification_hash_matches"] = (
            sha256_file(table4_verification_path) == cohort["table4_verification_sha256"]
        )
        table1_roots = [row.get("manifest_root") for row in load_json(table1_path).get("assets", [])]
        observed_roots = [row.get("manifest_root") for row in records]
        checks["record_order_matches_table1"] = observed_roots == (
            table1_roots if classification == "FORMAL" else table1_roots[:expected_n]
        )
        table4_payload = load_json(table4_path)
        frozen_by_root = {row.get("manifest_root"): row for row in table4_payload.get("items", [])}
        strict_by_id = {row.get("dataset_id"): row for row in load_json(table4_records_path)}
        checks["strict_results_match_table4"] = all(
            isinstance(strict_by_id.get(row.get("dataset_id")), Mapping)
            and strict_by_id[row.get("dataset_id")].get("strict_collision_pass")
            == row.get("strict_pass_no_method_allowance")
            for row in records
        )
        checks["table4_result_identities_match_manifest"] = all(
            isinstance(strict_by_id.get(frozen.get("dataset_id")), Mapping)
            and {
                key: strict_by_id[frozen.get("dataset_id")].get(key)
                for key in ("manifest_root", "input_identity_sha256", "order", "protocol_id")
            }
            == {
                "manifest_root": frozen.get("manifest_root"),
                "input_identity_sha256": frozen.get("input_identity_sha256"),
                "order": frozen.get("order"),
                "protocol_id": table4_payload.get("protocol_id"),
            }
            for frozen in table4_payload.get("items", [])
        )
        receipt = load_json(table4_verification_path)
        checks["table4_receipt_binds_source_artifacts"] = (
            receipt.get("status") == "PASS"
            and receipt.get("artifact_sha256", {}).get("frozen_manifest.json")
            == cohort["table4_manifest_sha256"]
            and receipt.get("artifact_sha256", {}).get("asset_records.json")
            == cohort["table4_records_sha256"]
        )

        dataset_root = Path(config["dataset_root"])
        inventory_by_id = {
            row.get("asset_id"): row for row in frozen_inventory.get("assets", [])
        }
        atomic_checks = [
            verify_atomic_record(
                row,
                frozen_by_root.get(row.get("manifest_root"), {}),
                dataset_root,
                inventory_by_id.get(row.get("asset_id")),
            )
            for row in records
        ]
        for name in (
            "source_bytes_match",
            "s1_input_identity_matches",
            "resource_closure_bytes_match",
            "evidence_inventory_matches",
            "evidence_atomic_results_match",
            "eligible_pair_count_matches",
            "empty_registry_enforced",
        ):
            checks[f"all_records_{name}"] = len(atomic_checks) == expected_n and all(
                row.get(name) is True for row in atomic_checks
            )
    except (KeyError, OSError, TypeError, ValueError, json.JSONDecodeError):
        checks["table1_source_hash_matches"] = False
        checks["table4_source_hash_matches"] = False
        checks["table4_records_hash_matches"] = False
        checks["table4_verification_hash_matches"] = False
        checks["record_order_matches_table1"] = False
        checks["strict_results_match_table4"] = False
        checks["table4_result_identities_match_manifest"] = False
        checks["table4_receipt_binds_source_artifacts"] = False
        for name in (
            "source_bytes_match", "s1_input_identity_matches", "resource_closure_bytes_match",
            "evidence_inventory_matches", "evidence_atomic_results_match",
            "eligible_pair_count_matches", "empty_registry_enforced",
        ):
            checks[f"all_records_{name}"] = False

    source_verification = config.get("source_verification", {})
    try:
        verification_path = Path(source_verification["table4_verification"])
        verification = load_json(verification_path)
        checks["table4_verification_receipt_matches"] = (
            sha256_file(verification_path) == source_verification["table4_verification_sha256"]
            and verification.get("status") == "PASS"
            and verification.get("artifact_sha256", {}).get("frozen_manifest.json")
            == cohort.get("table4_manifest_sha256")
            and verification.get("artifact_sha256", {}).get("asset_records.json")
            == cohort.get("table4_records_sha256")
            and source_verification.get("table4_verification_status") == "PASS"
        )
    except (KeyError, OSError, TypeError, ValueError, json.JSONDecodeError):
        checks["table4_verification_receipt_matches"] = False

    metrics = summary.get("metrics", {})
    allowance = metrics.get("allowance_density", {})
    gain = metrics.get("registered_allowance_gain_pp", {})
    checks["zero_gain_requires_zero_registered_allowance"] = not (
        gain.get("status") == "COMPLETE"
        and gain.get("value") == 0.0
        and allowance.get("registered_pairs") != 0
    )
    manifest_path = output / "manifest.json"
    if manifest_path.is_file():
        try:
            manifest = load_json(manifest_path)
            checks["manifest_closes_output_bundle"] = all(
                (output / name).is_file() and sha256_file(output / name) == digest
                for name, digest in manifest.get("outputs", {}).items()
            ) and {
                "frozen_config.json", "environment.json", "evidence_inventory.json",
                "asset_records.jsonl", "summary.json", "summary.md", "protocol_snapshot.md",
                "verification.json",
            }.issubset(manifest.get("outputs", {}))
            verifier_identity = manifest.get("verifier", {})
            checks["manifest_binds_verifier"] = (
                Path(verifier_identity.get("path", "")) == Path(__file__).resolve()
                and verifier_identity.get("sha256") == sha256_file(Path(__file__).resolve())
            )
        except (OSError, TypeError, ValueError, json.JSONDecodeError):
            checks["manifest_closes_output_bundle"] = False
            checks["manifest_binds_verifier"] = False
    return {
        "status": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks,
        "recomputed_metrics": recompute_metrics(records, intended_assets=len(records)),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    result = verify_run(args.output)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
