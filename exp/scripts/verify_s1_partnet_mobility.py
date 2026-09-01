#!/usr/bin/env python3
"""Independently verify a PartNet-Mobility Supplementary Table S1 run."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
from typing import Any, Mapping


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from exp.scripts import lam_supplementary_static as static_atoms  # noqa: E402


PROTOCOL_ID = "s1_partnet_mobility_table4cohort_n800_salt20260813_v1"
SCHEMA_VERSION = "supplementary-s1-partnet-mobility/v1"
DATASET = "PartNet-Mobility"
TABLE4_PROTOCOL_ID = "urdf_sim_ready_table4_partnet_mobility_n800_v1"
N_EVAL = 800
EXPECTED_MANIFEST_SHA256 = "2ff015ee6bb377ce693126b52dd632a7565a3eaa9f0007e26122a1bb4ab99900"
EXPECTED_ASSETS_SHA256 = "bdbfa385a74e44bd7662cba8f2c15ffbe3d664dfc0953722b8b37c44400430dc"
EXPECTED_STATES_SHA256 = "c72728ecfde2b0b6248da7048936b5bf52cb4f4cef0ae7438cf960e27895618c"
EXPECTED_ORDERED_IDS_SHA256 = "ef6cb964e50dc712280256c5b2f675cc2c957095c3553b21845d3562a5011883"
EXPECTED_STRICT_PASSED = 567
EXPECTED_STATIC_ATOMS_SHA256 = "4701415dad8a5c0a434c16887979bcb70c250ba0b25772014e8db73789098e5f"
PAIR_POLICY = {
    "eligible_pairs": "distinct source-URDF links with collision geometry",
    "shared_topology_exclusion": "exclude_direct_parent_child",
    "method_specific_allowance": "none in headline",
    "surface_contact_allowed": True,
    "penetration_threshold_m": 1e-6,
}


class VerificationError(RuntimeError):
    """Raised when any frozen binding or independently derived result differs."""


def canonical_json(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False)


def canonical_sha256(value: Any) -> str:
    return hashlib.sha256(canonical_json(value).encode("utf-8")).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def ordered_ids_sha256(values: list[str]) -> str:
    return canonical_sha256(values)


def load_object(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise VerificationError(f"invalid {label}: {path}") from exc
    if not isinstance(value, dict):
        raise VerificationError(f"{label} root is not an object")
    return value


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            if not line.strip():
                continue
            try:
                value = json.loads(line)
            except json.JSONDecodeError as exc:
                raise VerificationError(f"invalid JSONL at {path}:{line_number}") from exc
            if not isinstance(value, dict):
                raise VerificationError(f"non-object JSONL row at {path}:{line_number}")
            rows.append(value)
    return rows


def nonnegative_int(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise VerificationError(f"{label} must be a non-negative integer")
    return value


def _resolved_file(package: Path, relative: str, field: str) -> Path:
    try:
        safe = static_atoms.safe_package_relative_path(relative, field=field)
    except ValueError as exc:
        raise VerificationError(f"unsafe {field}: {relative!r}") from exc
    candidate = package.joinpath(*safe.parts)
    if candidate.is_symlink():
        raise VerificationError(f"{field} is a symlink: {relative}")
    try:
        resolved = candidate.resolve(strict=True)
        resolved.relative_to(package)
    except (OSError, ValueError) as exc:
        raise VerificationError(f"{field} escapes or is missing: {relative}") from exc
    if not resolved.is_file():
        raise VerificationError(f"{field} is not a regular file: {relative}")
    return resolved


def _evidence_inventory_for_package(
    *, selection_index: int, dataset_id: str, package: Path
) -> dict[str, Any]:
    candidates: list[dict[str, Any]] = []
    for current_raw, directory_names, file_names in os.walk(package, followlinks=False):
        current = Path(current_raw)
        directory_names.sort()
        file_names.sort()
        for name in directory_names:
            if (current / name).is_symlink():
                raise VerificationError(f"package contains directory symlink: {dataset_id}: {name}")
        for name in file_names:
            path = current / name
            lower = name.lower()
            kinds: list[str] = []
            if path.suffix.lower() == ".json" and static_atoms.RECEIPT_NAME_RE.search(lower):
                kinds.append("mechanical_receipt")
            if lower in static_atoms.REBUILD_RECIPE_NAMES:
                kinds.append("rebuild_recipe")
            if path.suffix.lower() == ".json" and static_atoms.ALLOWANCE_NAME_RE.search(lower):
                kinds.append("allowance_registry")
            if not kinds:
                continue
            if path.is_symlink():
                raise VerificationError(f"evidence candidate is a symlink: {dataset_id}: {name}")
            resolved = path.resolve(strict=True)
            try:
                relative = resolved.relative_to(package)
            except ValueError as exc:
                raise VerificationError(f"evidence candidate escapes package: {dataset_id}: {name}") from exc
            candidates.append(
                {
                    "path": relative.as_posix(),
                    "kinds": kinds,
                    "sha256": sha256_file(resolved),
                    "bytes": resolved.stat().st_size,
                }
            )
    candidates.sort(key=lambda row: (row["path"], row["kinds"]))
    return {
        "selection_index": selection_index,
        "dataset_id": dataset_id,
        "package": str(package),
        "candidates": candidates,
        "candidate_count": len(candidates),
        "candidates_sha256": canonical_sha256(candidates),
    }


def _derive_release_evidence(record: Mapping[str, Any]) -> dict[str, Any]:
    package = Path(str(record.get("package", ""))).resolve(strict=True)
    audit = static_atoms.audit_lam_package(
        package,
        urdf_relative_path=str(record.get("primary_urdf_relative_path")),
        asset_id=str(record.get("dataset_id")),
        expected_movable_joints=None,
    )
    if audit.get("status") != "completed" or audit.get("urdf_sha256") != record.get("urdf_sha256"):
        raise VerificationError(f"release audit or URDF binding mismatch: {record.get('dataset_id')}")
    closure = audit.get("resource_closure")
    evidence = audit.get("s1_evidence")
    if not isinstance(closure, Mapping) or not isinstance(evidence, Mapping):
        raise VerificationError(f"release evidence incomplete: {record.get('dataset_id')}")
    receipt = evidence.get("receipt")
    rebuild = evidence.get("rebuild")
    allowance = evidence.get("allowance")
    if not all(isinstance(value, Mapping) for value in (receipt, rebuild, allowance)):
        raise VerificationError(f"release evidence component missing: {record.get('dataset_id')}")
    receipt_bound = bool(receipt.get("receipt_bound_asset"))
    rebuild_eligible = bool(rebuild.get("eligible_asset"))
    if closure.get("complete") is not True and receipt_bound:
        raise VerificationError(f"valid receipt binds incomplete resource closure: {record.get('dataset_id')}")
    return {
        "resource_closure": dict(closure),
        "receipt": dict(receipt),
        "receipt_replay": {
            "eligible_receipt_count": int(receipt.get("valid_mechanical_receipt_count", 0)),
            "attempted": 0,
            "passed": False,
            "status": "VALID_RECEIPT_NOT_REPLAYED" if receipt_bound else "NO_VALID_RECEIPT",
        },
        "rebuild": {**dict(rebuild), "status": "ELIGIBLE_NOT_RUN" if rebuild_eligible else "N/E"},
        "allowance": dict(allowance),
        "issues": list(audit.get("issues", [])),
    }


def _derive_strict(item: Mapping[str, Any], asset: Mapping[str, Any], states: list[dict[str, Any]]) -> dict[str, Any]:
    dataset_id = str(item.get("dataset_id"))
    if canonical_sha256(states) != asset.get("state_records_sha256"):
        raise VerificationError(f"Table 4 state records SHA256 mismatch: {dataset_id}")
    expected = {
        "rest": nonnegative_int(item.get("rest_state_expected"), "rest expected"),
        "single_joint_sweep": nonnegative_int(item.get("single_state_expected"), "single expected"),
        "multi_joint_sobol": nonnegative_int(item.get("sobol_state_expected"), "sobol expected"),
    }
    by_phase: dict[str, list[dict[str, Any]]] = {phase: [] for phase in expected}
    state_identities: set[tuple[Any, Any, Any]] = set()
    for state in states:
        if state.get("dataset_id") != dataset_id or state.get("category") != item.get("category"):
            raise VerificationError(f"Table 4 state identity mismatch: {dataset_id}")
        phase = state.get("phase")
        if phase not in by_phase:
            raise VerificationError(f"unknown Table 4 phase: {dataset_id}: {phase!r}")
        nonnegative_int(state.get("non_adjacent_illegal_penetration_count"), "illegal penetration count")
        sample_index = nonnegative_int(state.get("sample_index"), "state sample_index")
        identity = (phase, state.get("joint_name"), sample_index)
        if identity in state_identities:
            raise VerificationError(f"duplicate Table 4 state identity: {dataset_id}: {identity}")
        state_identities.add(identity)
        by_phase[str(phase)].append(state)
    executed = {phase: len(rows) for phase, rows in by_phase.items()}
    free = {
        phase: sum(row["non_adjacent_illegal_penetration_count"] == 0 for row in rows)
        for phase, rows in by_phase.items()
    }
    movable = nonnegative_int(item.get("movable_dof_count"), "movable DoF count")
    evaluable = nonnegative_int(item.get("range_evaluable_dof_count"), "range-evaluable DoF count")
    rest_pass = executed["rest"] == expected["rest"] and free["rest"] == expected["rest"]
    single_pass = executed["single_joint_sweep"] == expected["single_joint_sweep"] and free["single_joint_sweep"] == expected["single_joint_sweep"]
    sobol_pass = movable > 0 and evaluable == movable and executed["multi_joint_sobol"] == expected["multi_joint_sobol"] and free["multi_joint_sobol"] == expected["multi_joint_sobol"]
    complete = evaluable == movable and sum(executed.values()) == sum(expected.values())
    strict = bool(complete and rest_pass and single_pass and sobol_pass)
    observed = {
        "rest_state_executed": executed["rest"],
        "rest_state_expected": expected["rest"],
        "rest_non_adjacent_free": free["rest"],
        "rest_non_adjacent_cf": rest_pass,
        "single_state_executed": executed["single_joint_sweep"],
        "single_state_expected": expected["single_joint_sweep"],
        "single_non_adjacent_free": free["single_joint_sweep"],
        "single_joint_sweep_cf": single_pass,
        "sobol_state_executed": executed["multi_joint_sobol"],
        "sobol_state_expected": expected["multi_joint_sobol"],
        "sobol_non_adjacent_free": free["multi_joint_sobol"],
        "multi_joint_sobol_cf": sobol_pass,
        "measurement_complete": complete,
        "strict_collision_pass": strict,
    }
    for field, value in observed.items():
        if asset.get(field) != value:
            raise VerificationError(f"Table 4 asset {field} mismatch: {dataset_id}")
    return {
        "strict": strict,
        "complete": complete,
        "state_count": len(states),
        "state_hash": asset.get("state_records_sha256"),
        "asset_hash": canonical_sha256(asset),
    }


def _ratio(passed: int, denominator: int) -> dict[str, int | float]:
    return {"passed": passed, "denominator": denominator, "rate": passed / denominator if denominator else 0.0}


def _aggregate(records: list[Mapping[str, Any]]) -> dict[str, Any]:
    denominator = len(records)
    receipt = sum(bool(row.get("release_receipt_bound")) for row in records)
    replay = sum(bool(row.get("release_receipt_replay_pass")) for row in records)
    eligible_rebuild = sum(bool(row.get("deterministic_rebuild_eligible")) for row in records)
    rebuild_match = sum(bool(row.get("deterministic_rebuild_match")) for row in records)
    registered = sum(nonnegative_int(row.get("registered_excluded_pair_count"), "registered pair count") for row in records)
    eligible_pairs = sum(nonnegative_int(row.get("eligible_nonadjacent_pair_count"), "eligible pair count") for row in records)
    strict = sum(bool(row.get("strict_collision_pass_no_method_allowance")) for row in records)
    registered_strict = sum(bool(row.get("strict_collision_pass_registered_allowance")) for row in records)
    return {
        "receipt_bound_assets": _ratio(receipt, denominator),
        "receipt_replay_pass": _ratio(replay, denominator),
        "deterministic_rebuild_match": {
            "status": "N/E" if eligible_rebuild == 0 else "NOT_RUN",
            "passed": None if eligible_rebuild == 0 else rebuild_match,
            "denominator": eligible_rebuild,
            "rate": None if eligible_rebuild == 0 else rebuild_match / eligible_rebuild,
            "eligible_assets": eligible_rebuild,
            "asset_denominator": denominator,
        },
        "allowance_density": {
            "registered_pairs": registered,
            "eligible_pairs": eligible_pairs,
            "rate": registered / eligible_pairs if eligible_pairs else None,
        },
        "strict_pass_no_method_allowance": _ratio(strict, denominator),
        "registered_allowance_gain_pp": {
            "value": 100.0 * (registered_strict - strict) / denominator if denominator else 0.0,
            "registered_passed": registered_strict,
            "no_allowance_passed": strict,
            "denominator": denominator,
        },
    }


def _render_summary(summary: Mapping[str, Any]) -> str:
    metrics = summary["metrics"]

    def fraction(metric: Mapping[str, Any]) -> str:
        return f"{metric['passed']} / {metric['denominator']} ({100.0 * metric['rate']:.2f}%)"

    rebuild = metrics["deterministic_rebuild_match"]
    allowance = metrics["allowance_density"]
    return "\n".join(
        [
            "# Supplementary Table S1: PartNet-Mobility",
            "",
            f"- Protocol: `{summary['protocol_id']}`",
            f"- Status: `{summary['status']}`",
            f"- N_eval: {summary['n_eval']}",
            "",
            "| Metric | Result |",
            "|---|---:|",
            f"| Receipt-bound Assets | {fraction(metrics['receipt_bound_assets'])} |",
            f"| Receipt Replay Pass | {fraction(metrics['receipt_replay_pass'])} |",
            f"| Deterministic Rebuild Match | {rebuild['status']} ({rebuild['eligible_assets']} / {rebuild['asset_denominator']} eligible) |",
            f"| Allowance Density | {allowance['registered_pairs']} / {allowance['eligible_pairs']} ({100.0 * (allowance['rate'] or 0.0):.2f}%) |",
            f"| Strict Pass (No Method-specific Allowance) | {fraction(metrics['strict_pass_no_method_allowance'])} |",
            f"| Registered-allowance Gain | {metrics['registered_allowance_gain_pp']['value']:.2f} pp |",
            "",
        ]
    )


def verify_run(run_dir: Path, *, formal: bool) -> dict[str, Any]:
    if run_dir.is_symlink():
        raise VerificationError("run directory is a symlink")
    run_dir = run_dir.resolve(strict=True)
    if not run_dir.is_dir():
        raise VerificationError("run directory is not a directory")
    manifest_path = run_dir / "manifest.json"
    if (
        manifest_path.is_symlink()
        or not manifest_path.is_file()
        or manifest_path.resolve(strict=True).parent != run_dir
    ):
        raise VerificationError("run manifest is not a safe regular child")
    manifest = load_object(manifest_path, "run manifest")
    checks: list[dict[str, Any]] = []

    def passed(name: str, detail: Any = True) -> None:
        checks.append({"name": name, "pass": True, "detail": detail})

    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, Mapping):
        raise VerificationError("run manifest artifacts are missing")
    expected_artifacts = {
        "frozen_config.json",
        "protocol_snapshot.md",
        "evidence_inventory.json",
        "asset_records.jsonl",
        "summary.json",
        "summary.md",
    }
    if set(artifacts) != expected_artifacts:
        raise VerificationError("artifact set mismatch")
    for name in sorted(expected_artifacts):
        binding = artifacts[name]
        if not isinstance(binding, Mapping):
            raise VerificationError("invalid artifact binding")
        path = run_dir / name
        if path.is_symlink() or path.resolve(strict=True).parent != run_dir or not path.is_file():
            raise VerificationError(f"artifact is not a safe regular child: {name}")
        if sha256_file(path) != binding.get("sha256"):
            raise VerificationError(f"artifact SHA256 mismatch: {name}")
        if path.stat().st_size != binding.get("bytes"):
            raise VerificationError(f"artifact size mismatch: {name}")
    passed("artifact_closure", len(artifacts))
    unbound_manifest = dict(manifest)
    declared_content_hash = unbound_manifest.pop("manifest_content_sha256", None)
    if canonical_sha256(unbound_manifest) != declared_content_hash:
        raise VerificationError("manifest content SHA256 mismatch")
    passed("manifest_self_hash")

    config = load_object(run_dir / "frozen_config.json", "frozen config")
    summary = load_object(run_dir / "summary.json", "summary")
    records = load_jsonl(run_dir / "asset_records.jsonl")

    classification = "FORMAL" if formal else "SMOKE"
    for label, value in (("manifest", manifest), ("config", config), ("summary", summary)):
        if (
            value.get("schema_version") != SCHEMA_VERSION
            or value.get("protocol_id") != PROTOCOL_ID
            or value.get("dataset") != DATASET
            or value.get("classification") != classification
        ):
            raise VerificationError(f"{label} identity mismatch")
    if manifest.get("n_eval") != len(records):
        raise VerificationError("manifest denominator mismatch")
    if config.get("pair_policy") != PAIR_POLICY:
        raise VerificationError("pair policy mismatch")
    if config.get("n_eval") != len(records) or summary.get("n_eval") != len(records):
        raise VerificationError("S1 output denominator mismatch")
    if summary.get("status") != "COMPLETE":
        raise VerificationError("summary status mismatch")
    passed("protocol_and_denominator", len(records))

    source = config.get("source")
    code = config.get("code_identity")
    if not isinstance(source, Mapping) or not isinstance(code, Mapping):
        raise VerificationError("frozen source or code identity missing")
    source_paths = {
        "cohort_manifest": Path(str(source.get("cohort_manifest"))).resolve(strict=True),
        "table4_manifest": Path(str(source.get("table4_manifest"))).resolve(strict=True),
        "table4_assets": Path(str(source.get("table4_asset_records"))).resolve(strict=True),
        "table4_states": Path(str(source.get("table4_state_records"))).resolve(strict=True),
        "protocol": Path(str(source.get("protocol_document"))).resolve(strict=True),
        "evidence_inventory": Path(str(source.get("evidence_inventory"))).resolve(strict=True),
    }
    source_hash_fields = {
        "cohort_manifest": "cohort_manifest_sha256",
        "table4_manifest": "table4_manifest_sha256",
        "table4_assets": "table4_asset_records_sha256",
        "table4_states": "table4_state_records_sha256",
    }
    for key, field in source_hash_fields.items():
        if sha256_file(source_paths[key]) != source.get(field):
            raise VerificationError(f"frozen source SHA256 mismatch: {key}")
    if sha256_file(run_dir / "protocol_snapshot.md") != source.get("protocol_snapshot_sha256"):
        raise VerificationError("protocol snapshot SHA256 mismatch")
    if (run_dir / "protocol_snapshot.md").read_bytes() != source_paths["protocol"].read_bytes():
        raise VerificationError("protocol snapshot differs from source")
    if source_paths["evidence_inventory"] != run_dir / "evidence_inventory.json":
        raise VerificationError("evidence inventory path mismatch")
    if sha256_file(source_paths["evidence_inventory"]) != source.get("evidence_inventory_sha256"):
        raise VerificationError("evidence inventory SHA256 mismatch")
    expected_code_paths = (
        ("runner_path", SCRIPT.with_name("run_s1_partnet_mobility.py")),
        ("verifier_path", SCRIPT),
        ("static_atoms_path", Path(static_atoms.__file__).resolve()),
    )
    for field, canonical_path in expected_code_paths:
        recorded_path = Path(str(code.get(field))).resolve(strict=True)
        if recorded_path != canonical_path.resolve(strict=True):
            raise VerificationError(f"code path mismatch: {field}")
        hash_field = field.replace("_path", "_sha256")
        if sha256_file(recorded_path) != code.get(hash_field):
            raise VerificationError(f"code identity mismatch: {field}")
    if formal and code.get("static_atoms_sha256") != EXPECTED_STATIC_ATOMS_SHA256:
        raise VerificationError("formal static S1 atom SHA256 mismatch")
    passed("frozen_source_and_code_bindings")

    cohort_manifest = load_object(source_paths["cohort_manifest"], "cohort manifest")
    table4_manifest = load_object(source_paths["table4_manifest"], "Table 4 manifest")
    items = table4_manifest.get("items")
    assets = json.loads(source_paths["table4_assets"].read_text(encoding="utf-8"))
    states = load_jsonl(source_paths["table4_states"])
    if not isinstance(items, list) or not isinstance(assets, list):
        raise VerificationError("Table 4 source roots are invalid")
    if not formal:
        items = items[: len(records)]
        assets = assets[: len(records)]
        ids = {str(row.get("dataset_id")) for row in records}
        states = [row for row in states if row.get("dataset_id") in ids]
    if len(items) != len(records) or len(assets) != len(records):
        raise VerificationError("Table 4 source denominator mismatch")
    if cohort_manifest.get("items", [])[: len(records)] != items:
        raise VerificationError("cohort and Table 4 manifest items differ")
    manifest_dataset_root_raw = Path(str(cohort_manifest.get("dataset_root", "")))
    config_dataset_root_raw = Path(str(source.get("dataset_root", "")))
    if manifest_dataset_root_raw.is_symlink() or config_dataset_root_raw.is_symlink():
        raise VerificationError("dataset root is a symlink")
    dataset_root = manifest_dataset_root_raw.resolve(strict=True)
    if config_dataset_root_raw.resolve(strict=True) != dataset_root:
        raise VerificationError("dataset root binding mismatch")

    inventory_records: list[dict[str, Any]] = []
    for index, item in enumerate(items):
        if not isinstance(item, Mapping):
            raise VerificationError(f"invalid Table 4 item at index {index}")
        dataset_id = item.get("dataset_id")
        if (
            not isinstance(dataset_id, str)
            or not dataset_id
            or Path(dataset_id).name != dataset_id
            or "/" in dataset_id
            or "\\" in dataset_id
        ):
            raise VerificationError(f"unsafe dataset_id at index {index}")
        package_candidate = dataset_root / dataset_id
        if package_candidate.is_symlink():
            raise VerificationError(f"package is a symlink: {dataset_id}")
        package = package_candidate.resolve(strict=True)
        try:
            package.relative_to(dataset_root)
        except ValueError as exc:
            raise VerificationError(f"package escapes dataset root: {dataset_id}") from exc
        if not package.is_dir() or package.name != dataset_id:
            raise VerificationError(f"package identity mismatch: {dataset_id}")
        inventory_records.append(
            _evidence_inventory_for_package(
                selection_index=index, dataset_id=dataset_id, package=package
            )
        )
    expected_inventory = {
        "schema_version": "s1-evidence-candidate-inventory/v1",
        "protocol_id": PROTOCOL_ID,
        "record_count": len(inventory_records),
        "candidate_count": sum(row["candidate_count"] for row in inventory_records),
        "records": inventory_records,
        "records_sha256": canonical_sha256(inventory_records),
    }
    if load_object(source_paths["evidence_inventory"], "evidence inventory") != expected_inventory:
        raise VerificationError("evidence candidate inventory mismatch")
    passed("evidence_candidate_inventory", expected_inventory["candidate_count"])

    states_by_id: dict[str, list[dict[str, Any]]] = {str(item.get("dataset_id")): [] for item in items}
    seen_state_order: list[str] = []
    for state in states:
        dataset_id = state.get("dataset_id")
        if dataset_id not in states_by_id:
            raise VerificationError(f"unknown state dataset_id: {dataset_id!r}")
        if not seen_state_order or seen_state_order[-1] != dataset_id:
            if dataset_id in seen_state_order:
                raise VerificationError("Table 4 states are not grouped in cohort order")
            seen_state_order.append(str(dataset_id))
        states_by_id[str(dataset_id)].append(state)

    strict_passed = 0
    for index, (item, asset, record) in enumerate(zip(items, assets, records, strict=True)):
        if not isinstance(item, Mapping) or not isinstance(asset, Mapping):
            raise VerificationError(f"invalid source row at index {index}")
        dataset_id = str(item.get("dataset_id"))
        package = (dataset_root / dataset_id).resolve(strict=True)
        urdf = _resolved_file(package, "mobility.urdf", "primary_urdf")
        if sha256_file(urdf) != item.get("urdf_sha256"):
            raise VerificationError(f"URDF source binding mismatch: {dataset_id}")
        meshes = item.get("collision_mesh_files")
        if not isinstance(meshes, list):
            raise VerificationError(f"collision mesh inventory missing: {dataset_id}")
        for mesh in meshes:
            if not isinstance(mesh, Mapping) or not isinstance(mesh.get("exists"), bool) or not isinstance(mesh.get("path"), str):
                raise VerificationError(f"invalid collision mesh source binding: {dataset_id}")
            if mesh["exists"]:
                path = _resolved_file(package, mesh["path"], "collision_mesh")
                if sha256_file(path) != mesh.get("sha256") or path.stat().st_size != mesh.get("size_bytes"):
                    raise VerificationError(f"collision mesh source binding mismatch: {dataset_id}")
            else:
                try:
                    safe = static_atoms.safe_package_relative_path(mesh["path"], field="collision_mesh")
                except ValueError as exc:
                    raise VerificationError(f"unsafe declared-missing collision mesh: {dataset_id}") from exc
                if os.path.lexists(package.joinpath(*safe.parts)) or mesh.get("sha256") is not None or mesh.get("size_bytes") is not None:
                    raise VerificationError(f"declared-missing collision mesh source mismatch: {dataset_id}")
        source_record = {
            "selection_index": index,
            "asset_id": dataset_id,
            "dataset_id": dataset_id,
            "category": item.get("category"),
            "package": str(package),
            "primary_urdf_relative_path": "mobility.urdf",
            "urdf_sha256_expected": item.get("urdf_sha256"),
            "collision_mesh_files_expected": [dict(mesh) for mesh in meshes],
            "table4_input_identity_sha256": item.get("input_identity_sha256"),
            "movable_dof_count": item.get("movable_dof_count"),
            "range_evaluable_dof_count": item.get("range_evaluable_dof_count"),
            "rest_state_expected": item.get("rest_state_expected"),
            "single_state_expected": item.get("single_state_expected"),
            "sobol_state_expected": item.get("sobol_state_expected"),
            "table4_protocol_id": item.get("protocol_id", table4_manifest.get("protocol_id")),
        }
        source_record["s1_input_identity_sha256"] = canonical_sha256(source_record)
        derived_release = _derive_release_evidence(
            {
                "package": str(package),
                "primary_urdf_relative_path": "mobility.urdf",
                "dataset_id": dataset_id,
                "urdf_sha256": item.get("urdf_sha256"),
            }
        )
        allowance = derived_release["allowance"]
        receipt = derived_release["receipt"]
        replay = derived_release["receipt_replay"]
        rebuild = derived_release["rebuild"]
        derived_strict = _derive_strict(item, asset, states_by_id[dataset_id])
        strict = derived_strict["strict"]
        strict_passed += int(strict)
        expected_record = {
            "selection_index": index,
            "asset_id": dataset_id,
            "dataset_id": dataset_id,
            "category": item.get("category"),
            "package": str(package),
            "primary_urdf_relative_path": "mobility.urdf",
            "urdf_sha256": item.get("urdf_sha256"),
            "table4_input_identity_sha256": item.get("input_identity_sha256"),
            "s1_input_identity_sha256": source_record["s1_input_identity_sha256"],
            "resource_closure": derived_release["resource_closure"],
            "release_receipt_bound": bool(receipt.get("receipt_bound_asset")),
            "release_receipt_replay_pass": bool(replay.get("passed")),
            "receipt_replay_status": replay.get("status"),
            "deterministic_rebuild_eligible": bool(rebuild.get("eligible_asset")),
            "deterministic_rebuild_match": False,
            "deterministic_rebuild_status": rebuild.get("status"),
            "registered_excluded_pair_count": allowance.get("registered_excluded_pair_count"),
            "eligible_nonadjacent_pair_count": allowance.get("eligible_nonadjacent_pair_count"),
            "strict_collision_pass_no_method_allowance": strict,
            "strict_collision_pass_registered_allowance": strict,
            "table4_measurement_complete": derived_strict["complete"],
            "table4_state_record_count": derived_strict["state_count"],
            "table4_state_records_sha256": derived_strict["state_hash"],
            "table4_asset_record_sha256": derived_strict["asset_hash"],
            "release_evidence": derived_release,
            "status": "completed",
        }
        if record != expected_record:
            raise VerificationError(f"asset record mismatch: {dataset_id}")
    passed("source_assets_and_release_evidence", len(records))
    passed("independent_table4_reaggregation", strict_passed)

    metrics = _aggregate(records)
    if summary.get("metrics") != metrics:
        raise VerificationError("summary metrics mismatch")
    if summary.get("terminal_records") != len(records) or summary.get("error_records") != 0:
        raise VerificationError("summary terminal accounting mismatch")
    if summary.get("table4_state_record_count") != len(states):
        raise VerificationError("summary state record count mismatch")
    if (run_dir / "summary.md").read_text(encoding="utf-8") != _render_summary(summary):
        raise VerificationError("summary markdown mismatch")
    passed("independent_summary_reaggregation")

    ids = [str(row.get("dataset_id")) for row in records]
    if source.get("ordered_dataset_ids_sha256") != ordered_ids_sha256(ids):
        raise VerificationError("ordered dataset IDs SHA256 mismatch")
    if formal:
        if len(records) != N_EVAL:
            raise VerificationError("formal N_eval mismatch")
        if table4_manifest.get("protocol_id") != TABLE4_PROTOCOL_ID:
            raise VerificationError("formal Table 4 protocol mismatch")
        if source.get("cohort_manifest_sha256") != EXPECTED_MANIFEST_SHA256:
            raise VerificationError("formal cohort manifest mismatch")
        if source.get("table4_asset_records_sha256") != EXPECTED_ASSETS_SHA256:
            raise VerificationError("formal Table 4 asset records mismatch")
        if source.get("table4_state_records_sha256") != EXPECTED_STATES_SHA256:
            raise VerificationError("formal Table 4 state records mismatch")
        if source.get("ordered_dataset_ids_sha256") != EXPECTED_ORDERED_IDS_SHA256:
            raise VerificationError("formal ordered dataset IDs mismatch")
        if strict_passed != EXPECTED_STRICT_PASSED:
            raise VerificationError("formal strict-pass aggregate mismatch")
        if metrics["receipt_bound_assets"]["passed"] != 0:
            raise VerificationError("formal receipt result is not zero")
        if metrics["receipt_replay_pass"]["passed"] != 0:
            raise VerificationError("formal receipt replay result is not zero")
        if metrics["deterministic_rebuild_match"]["eligible_assets"] != 0:
            raise VerificationError("formal rebuild eligibility is not zero")
        if metrics["allowance_density"]["registered_pairs"] != 0:
            raise VerificationError("formal registered allowance is not zero")
        passed("formal_frozen_expectations")
    return {
        "status": "PASS",
        "all_pass": True,
        "check_count": len(checks),
        "checks": checks,
        "run_dir": str(run_dir),
        "formal": formal,
    }


def atomic_write_json(path: Path, value: Any) -> None:
    descriptor, name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    temporary = Path(name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            json.dump(value, handle, indent=2, sort_keys=True, ensure_ascii=True, allow_nan=False)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--smoke", action="store_true")
    parser.add_argument("--write", action="store_true")
    args = parser.parse_args(argv)
    result = verify_run(args.run_dir, formal=not args.smoke)
    if args.write:
        atomic_write_json(args.run_dir.resolve() / "verification.json", result)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
