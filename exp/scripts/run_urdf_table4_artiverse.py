#!/usr/bin/env python3
"""Run the frozen URDF Sim-Ready Table 4 protocol on Artiverse."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from concurrent.futures import as_completed, ThreadPoolExecutor
from functools import lru_cache
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
from typing import Any, Iterable
import xml.etree.ElementTree as ET


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
CORE_SCRIPT = REPO / "exp/scripts/run_urdf_table4_partnet_mobility.py"
DEFAULT_DATASET = REPO / "exp/artiverse"
DEFAULT_TABLE1_MANIFEST = REPO / "exp/runtime/table1_artiverse/manifest.json"
DEFAULT_OUTPUT = REPO / "exp/runtime/urdf_table4_artiverse_table1_n800_20260814"

PROTOCOL_ID = "urdf_sim_ready_table4_artiverse_table1_n800_v1"
QUALIFICATION_PROTOCOL_ID = "urdf_sim_ready_table4_artiverse_qualification_v1"
VERIFY_PROTOCOL_ID = "urdf_sim_ready_table4_artiverse_verify_v1"
SAMPLE_SIZE = 800
EXPECTED_RELEASE_ASSETS = 3544
EXPECTED_TABLE1_MANIFEST_SHA256 = (
    "f74575692b87605699c4f349186c4660d691c91bef39562bb976baf22ae72a8c"
)
EXPECTED_ORDERED_ASSET_IDENTITIES_SHA256 = (
    "c9078092dcb50644975815cfceee4d0c06d6387d745a206e8a29fb3c19452f28"
)
EXPECTED_CORE_SHA256 = (
    "e710d15cb79c50506487ff1335a88591bb58c11cf726c71198103c05f6d01ff0"
)
SCALE_PROTOCOL = "pybullet_q0_collision_shape_union_aabb_v1"
SINGLE_SAMPLES = 21
SOBOL_SAMPLES = 64
SOBOL_SEED = 20260813
PENETRATION_THRESHOLD_M = 1e-6
IDENTITY_FIELDS = (
    "asset_id",
    "manifest_root",
    "model_id",
    "raw_category",
    "source",
    "chunk_archive",
    "selection_hash",
    "selection_rank",
)
FROZEN_INPUT_FIELDS = (
    "protocol_id",
    "order",
    "dataset_id",
    *IDENTITY_FIELDS,
    "asset_root_relpath",
    "category",
    "package_audit_success",
    "audit_issue",
    "primary_urdf_relpath",
    "urdf_sha256",
    "valid_tree",
    "movable_dof_count",
    "range_evaluable_dof_count",
    "joint_specs_sha256",
    "collision_mesh_inventory_sha256",
    "missing_collision_mesh_reference_count",
    "unsafe_collision_mesh_reference_count",
    "scale_derivation_sha256",
    "object_bbox_diagonal_m",
    "runtime_identity_sha256",
    "rest_state_expected",
    "single_state_expected",
    "sobol_state_expected",
)


@lru_cache(maxsize=1)
def _load_core() -> Any:
    if sha256_file(CORE_SCRIPT) != EXPECTED_CORE_SHA256:
        raise RuntimeError("PartNet Table 4 core SHA256 does not match the frozen pin")
    spec = importlib.util.spec_from_file_location("urdf_table4_partnet_core", CORE_SCRIPT)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import collision core: {CORE_SCRIPT}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def sha256_file(path: Path) -> str:
    import hashlib

    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_sha256(value: Any) -> str:
    import hashlib

    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _input_identity_sha256(item: dict[str, Any]) -> str:
    return canonical_sha256({key: item.get(key) for key in FROZEN_INPUT_FIELDS})


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def atomic_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def atomic_jsonl(path: Path, rows: Iterable[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        for row in rows:
            stream.write(json.dumps(row, sort_keys=True, ensure_ascii=True) + "\n")
    temporary.replace(path)


def atomic_text(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(value, encoding="utf-8")
    temporary.replace(path)


def _ordered_identities(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [{key: row[key] for key in IDENTITY_FIELDS} for row in rows]


def load_table1_cohort(
    manifest_path: Path,
    *,
    sample_size: int = SAMPLE_SIZE,
    qualification_smoke: bool = False,
) -> dict[str, Any]:
    manifest_path = manifest_path.resolve(strict=True)
    observed_hash = sha256_file(manifest_path)
    if observed_hash != EXPECTED_TABLE1_MANIFEST_SHA256:
        raise RuntimeError("Table 1 manifest SHA256 does not match the frozen pin")
    source = read_json(manifest_path)
    rows = source.get("assets")
    if not isinstance(rows, list) or len(rows) != SAMPLE_SIZE:
        raise RuntimeError("Table 1 manifest does not contain exactly 800 assets")
    if source.get("N_eval") != SAMPLE_SIZE or source.get("N_release") != EXPECTED_RELEASE_ASSETS:
        raise RuntimeError("Table 1 cohort denominator mismatch")
    identities = _ordered_identities(rows)
    if len({row["asset_id"] for row in identities}) != SAMPLE_SIZE:
        raise RuntimeError("Table 1 cohort asset identities are not unique")
    if [row["selection_rank"] for row in identities] != list(range(1, 801)):
        raise RuntimeError("Table 1 cohort selection ranks are not the frozen sequence")
    ordered_hash = canonical_sha256(identities)
    if ordered_hash != EXPECTED_ORDERED_ASSET_IDENTITIES_SHA256:
        raise RuntimeError("Table 1 ordered asset identity hash mismatch")
    if sample_size <= 0 or sample_size > SAMPLE_SIZE:
        raise ValueError("sample_size must be in [1, 800]")
    if not qualification_smoke and sample_size != SAMPLE_SIZE:
        raise ValueError(
            f"formal protocol requires sample_size={SAMPLE_SIZE}, got {sample_size}"
        )
    selected = identities[:sample_size]
    return {
        "protocol_id": QUALIFICATION_PROTOCOL_ID if qualification_smoke else PROTOCOL_ID,
        "qualification_smoke": qualification_smoke,
        "cohort_label": (
            f"Artiverse Table 1 qualification N={sample_size}"
            if qualification_smoke
            else "Artiverse Table 1 fixed N=800 cohort"
        ),
        "source_manifest_path": str(manifest_path),
        "source_manifest_sha256": observed_hash,
        "release_asset_count": EXPECTED_RELEASE_ASSETS,
        "source_cohort_type": source.get("cohort_type"),
        "source_release_manifest_sha256": source.get("release_manifest_sha256"),
        "source_release_universe_sha256": source.get("release_universe_sha256"),
        "source_selection_protocol": source.get("selection_protocol"),
        "ordered_asset_identities_sha256": ordered_hash,
        "selected": selected,
    }


def _contained_asset_root(dataset_root: Path, manifest_root: str) -> Path:
    if Path(manifest_root).is_absolute():
        raise ValueError("absolute manifest_root is not allowed")
    dataset_root = dataset_root.resolve(strict=True)
    candidate = (dataset_root / manifest_root).resolve(strict=False)
    try:
        candidate.relative_to(dataset_root)
    except ValueError as exc:
        raise ValueError("manifest_root escapes dataset root") from exc
    return candidate


def _valid_tree(urdf_path: Path) -> bool:
    root = ET.parse(urdf_path).getroot()
    links = [node.get("name", "").strip() for node in root.findall("link")]
    if not links or "" in links or len(links) != len(set(links)):
        return False
    link_set = set(links)
    adjacency: dict[str, list[str]] = {name: [] for name in links}
    indegree: Counter[str] = Counter()
    joints = root.findall("joint")
    for joint in joints:
        parent_node = joint.find("parent")
        child_node = joint.find("child")
        parent = parent_node.get("link", "") if parent_node is not None else ""
        child = child_node.get("link", "") if child_node is not None else ""
        if parent not in link_set or child not in link_set or parent == child:
            return False
        adjacency[parent].append(child)
        indegree[child] += 1
    roots = [name for name in links if indegree[name] == 0]
    if len(roots) != 1 or any(value > 1 for value in indegree.values()):
        return False
    active: set[str] = set()
    visited: set[str] = set()

    def visit(name: str) -> bool:
        if name in active or name in visited:
            return False
        active.add(name)
        for child in adjacency[name]:
            if not visit(child):
                return False
        active.remove(name)
        visited.add(name)
        return True

    return bool(
        len(joints) == len(links) - 1
        and visit(roots[0])
        and len(visited) == len(links)
    )


def _resolve_primary_urdf(dataset_root: Path, source_row: dict[str, Any]) -> tuple[Path, Path]:
    asset_root = _contained_asset_root(dataset_root, str(source_row["manifest_root"]))
    if not asset_root.is_dir():
        raise FileNotFoundError("missing Artiverse asset root")
    package = asset_root / "urdf_w_collider"
    if not package.is_dir():
        raise FileNotFoundError("missing urdf_w_collider directory")
    candidates = sorted(path for path in package.glob("*.urdf") if path.is_file())
    if len(candidates) != 1:
        raise ValueError(f"expected exactly one top-level URDF; found {len(candidates)}")
    return asset_root, candidates[0]


def collision_mesh_inventory(
    dataset_root: Path, asset_root: Path, urdf_path: Path
) -> tuple[list[dict[str, Any]], int]:
    dataset_root = dataset_root.resolve(strict=True)
    package_root = urdf_path.parent.resolve(strict=True)
    root = ET.parse(urdf_path).getroot()
    references = sorted(
        {
            mesh.get("filename", "").replace("\\", "/")
            for mesh in root.findall("link/collision/geometry/mesh")
            if mesh.get("filename")
        }
    )
    inventory: list[dict[str, Any]] = []
    unsafe = 0
    for reference in references:
        path: Path | None = None
        safe = bool(reference) and "://" not in reference and not Path(reference).is_absolute()
        if safe:
            candidate = (package_root / reference).resolve(strict=False)
            try:
                candidate.relative_to(package_root)
                candidate.relative_to(asset_root.resolve(strict=True))
                candidate.relative_to(dataset_root)
                path = candidate
            except ValueError:
                safe = False
        if not safe:
            unsafe += 1
        exists = bool(path is not None and path.is_file())
        inventory.append(
            {
                "path": reference,
                "safe": safe,
                "resolved_relpath": (
                    path.relative_to(dataset_root).as_posix() if path is not None else None
                ),
                "exists": exists,
                "is_symlink": bool(path is not None and path.is_symlink()),
                "size_bytes": path.stat().st_size if exists and path is not None else None,
                "sha256": sha256_file(path) if exists and path is not None else None,
            }
        )
    return inventory, unsafe


def derive_collision_aabb(urdf_path: Path) -> dict[str, Any]:
    import pybullet as bullet

    client = bullet.connect(bullet.DIRECT)
    body: int | None = None
    try:
        flags = int(
            bullet.URDF_USE_INERTIA_FROM_FILE
            | bullet.URDF_IGNORE_VISUAL_SHAPES
        )
        body = bullet.loadURDF(
            str(urdf_path),
            useFixedBase=True,
            flags=flags,
            physicsClientId=client,
        )
        for joint_index in range(bullet.getNumJoints(body, physicsClientId=client)):
            bullet.resetJointState(
                body,
                joint_index,
                0.0,
                targetVelocity=0.0,
                physicsClientId=client,
            )
        collision_links = []
        bounds = []
        for link_index in range(-1, bullet.getNumJoints(body, physicsClientId=client)):
            shapes = bullet.getCollisionShapeData(
                body, link_index, physicsClientId=client
            )
            if not shapes:
                continue
            lower, upper = bullet.getAABB(body, link_index, physicsClientId=client)
            values = [*lower, *upper]
            if not all(math.isfinite(float(value)) for value in values):
                raise ValueError("collision AABB contains non-finite values")
            collision_links.append(link_index)
            bounds.append((tuple(map(float, lower)), tuple(map(float, upper))))
        if not bounds:
            raise ValueError("loaded URDF has no collision shapes")
        minimum = [min(row[0][axis] for row in bounds) for axis in range(3)]
        maximum = [max(row[1][axis] for row in bounds) for axis in range(3)]
        diagonal = math.sqrt(
            sum((high - low) ** 2 for low, high in zip(minimum, maximum))
        )
        if not math.isfinite(diagonal) or diagonal <= 0.0:
            raise ValueError("collision AABB diagonal is not positive finite")
        return {
            "protocol": SCALE_PROTOCOL,
            "status": "PASS",
            "joint_state": "q=0 for every simulator joint",
            "minimum_m": minimum,
            "maximum_m": maximum,
            "diagonal_m": diagonal,
            "collision_link_indices": collision_links,
            "load_flags": flags,
        }
    finally:
        if body is not None:
            bullet.removeBody(body, physicsClientId=client)
        bullet.disconnect(client)


def _empty_audit() -> dict[str, Any]:
    return {
        "package_audit_success": False,
        "audit_issue": None,
        "primary_urdf_relpath": None,
        "urdf_sha256": None,
        "valid_tree": False,
        "movable_dof_count": 0,
        "range_evaluable_dof_count": 0,
        "joint_specs": [],
        "joint_specs_sha256": canonical_sha256([]),
        "collision_mesh_files": [],
        "collision_mesh_inventory_sha256": canonical_sha256([]),
        "missing_collision_mesh_reference_count": 0,
        "unsafe_collision_mesh_reference_count": 0,
        "scale_derivation": {"protocol": SCALE_PROTOCOL, "status": "N/E"},
        "scale_derivation_sha256": None,
        "object_bbox_diagonal_m": None,
    }


def audit_asset(dataset_root: Path, source_row: dict[str, Any]) -> dict[str, Any]:
    result = _empty_audit()
    try:
        asset_root, urdf_path = _resolve_primary_urdf(dataset_root, source_row)
        result["primary_urdf_relpath"] = urdf_path.relative_to(
            dataset_root.resolve(strict=True)
        ).as_posix()
        result["urdf_sha256"] = sha256_file(urdf_path)
        core = _load_core()
        joints = core.parse_urdf_joints(urdf_path)
        result["joint_specs"] = joints
        result["joint_specs_sha256"] = canonical_sha256(joints)
        result["movable_dof_count"] = len(joints)
        result["range_evaluable_dof_count"] = sum(
            bool(row["range_evaluable"]) for row in joints
        )
        result["valid_tree"] = _valid_tree(urdf_path)
        inventory, unsafe = collision_mesh_inventory(
            dataset_root, asset_root, urdf_path
        )
        result["collision_mesh_files"] = inventory
        result["collision_mesh_inventory_sha256"] = canonical_sha256(inventory)
        result["unsafe_collision_mesh_reference_count"] = unsafe
        result["missing_collision_mesh_reference_count"] = sum(
            not row["exists"] for row in inventory
        )
        if unsafe:
            raise ValueError(f"unsafe collision mesh reference count: {unsafe}")
        if result["missing_collision_mesh_reference_count"]:
            raise FileNotFoundError(
                "missing collision mesh reference count: "
                f"{result['missing_collision_mesh_reference_count']}"
            )
        if not result["valid_tree"]:
            raise ValueError("URDF joint graph is not a valid rooted tree")
        scale = derive_collision_aabb(urdf_path)
        result["scale_derivation"] = scale
        result["scale_derivation_sha256"] = canonical_sha256(scale)
        result["object_bbox_diagonal_m"] = scale["diagonal_m"]
        result["package_audit_success"] = True
    except Exception as exc:  # noqa: BLE001
        result["audit_issue"] = f"{type(exc).__name__}: {exc}"
    return result


def current_runtime_identity() -> dict[str, Any]:
    core = _load_core()
    return {
        **core.current_runtime_identity(),
        "adapter_runner": str(SCRIPT),
        "adapter_runner_sha256": sha256_file(SCRIPT),
        "collision_core": str(CORE_SCRIPT),
        "collision_core_sha256": sha256_file(CORE_SCRIPT),
    }


def failure_record(
    item: dict[str, Any], issue: str, *, timed_out: bool = False
) -> dict[str, Any]:
    core = _load_core()
    result = core.failure_record(item, issue, timed_out=timed_out)
    _add_source_identity(result, item)
    result["runner_sha256"] = sha256_file(SCRIPT)
    result["collision_core_sha256"] = sha256_file(CORE_SCRIPT)
    return result


def result_matches_item(
    result: dict[str, Any],
    item: dict[str, Any],
    runner_hash: str,
    state_records: list[dict[str, Any]] | None = None,
) -> bool:
    core = _load_core()
    frozen_fields = (
        "protocol_id",
        "order",
        "dataset_id",
        "category",
        "input_identity_sha256",
        "movable_dof_count",
        "range_evaluable_dof_count",
        "rest_state_expected",
        "single_state_expected",
        "sobol_state_expected",
        "object_bbox_diagonal_m",
        "runtime_identity",
        *IDENTITY_FIELDS,
    )
    matches = bool(
        all(result.get(key) == item.get(key) for key in frozen_fields)
        and result.get("runner_sha256") == runner_hash
        and result.get("collision_core_sha256") == EXPECTED_CORE_SHA256
        and core._result_counters_valid(result, state_records, item)
    )
    if not matches:
        return False
    try:
        validate_state_closure(result, state_records, item)
    except (KeyError, RuntimeError, TypeError, ValueError):
        return False
    return True


def validate_state_closure(
    record: dict[str, Any],
    state_records: list[dict[str, Any]] | None = None,
    item: dict[str, Any] | None = None,
) -> None:
    states = record.get("state_records", []) if state_records is None else state_records
    expected = record if item is None else item
    for state in states:
        if any(
            state.get(key) != expected.get(key)
            for key in (*IDENTITY_FIELDS, "protocol_id", "order", "input_identity_sha256")
        ):
            raise RuntimeError("state source identity mismatch")
    _load_core().validate_state_closure(record, states, item)


def summarize_records(
    manifest: dict[str, Any], records: list[dict[str, Any]]
) -> dict[str, Any]:
    core = _load_core()
    summary = core.summarize_records(manifest, records)

    def fix_normalization(node: dict[str, Any]) -> None:
        metrics = node.get("metrics")
        if isinstance(metrics, dict) and isinstance(
            metrics.get("max_penetration"), dict
        ):
            metrics["max_penetration"]["normalization"] = (
                "PyBullet q=0 collision-shape union AABB diagonal"
            )
        for child in node.get("category_results", {}).values():
            if isinstance(child, dict):
                fix_normalization({"metrics": child})

    fix_normalization(summary)
    return summary


def build_manifest(
    dataset_root: Path,
    table1_manifest: Path,
    *,
    sample_size: int = SAMPLE_SIZE,
    qualification_smoke: bool = False,
    child_runtime: dict[str, Any] | None = None,
    workers: int = 1,
) -> dict[str, Any]:
    if workers <= 0:
        raise ValueError("workers must be positive")
    dataset_root = dataset_root.resolve(strict=True)
    table1_manifest = table1_manifest.resolve(strict=True)
    contract = load_table1_cohort(
        table1_manifest,
        sample_size=sample_size,
        qualification_smoke=qualification_smoke,
    )
    frozen_runtime = child_runtime or current_runtime_identity()
    _load_core().require_runtime_match(current_runtime_identity(), frozen_runtime)
    selected = contract["selected"]
    if workers == 1:
        audits = [audit_asset(dataset_root, row) for row in selected]
    else:
        with ThreadPoolExecutor(max_workers=workers) as executor:
            audits = list(
                executor.map(
                    lambda row: audit_asset(dataset_root, row),
                    selected,
                )
            )
    items = [
        freeze_item(
            source_row,
            audit,
            order=order,
            protocol_id=contract["protocol_id"],
            runtime_identity=frozen_runtime,
        )
        for order, (source_row, audit) in enumerate(zip(selected, audits))
    ]
    release_manifest_relpath = str(
        read_json(table1_manifest).get("release_manifest", "dataset_chunks/manifest.json")
    )
    release_manifest = (dataset_root / release_manifest_relpath).resolve(strict=True)
    release_manifest_hash = sha256_file(release_manifest)
    if release_manifest_hash != contract["source_release_manifest_sha256"]:
        raise RuntimeError("Artiverse release manifest SHA256 does not match Table 1")
    manifest = {
        "protocol_id": contract["protocol_id"],
        "status": "FROZEN",
        "created_at_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "cohort_label": contract["cohort_label"],
        "qualification_smoke": qualification_smoke,
        "cohort_boundary": {
            "is_full_release_cohort": False,
            "is_shared_category_balanced_cohort": False,
            "paper_table_role": (
                "qualification only" if qualification_smoke else "fixed Table 1 sample"
            ),
        },
        "dataset_root": str(dataset_root),
        "release_asset_count": contract["release_asset_count"],
        "sample_size": sample_size,
        "source": {
            "table1_manifest_path": str(table1_manifest),
            "table1_manifest_sha256": contract["source_manifest_sha256"],
            "release_manifest_path": str(release_manifest),
            "release_manifest_sha256": release_manifest_hash,
            "release_universe_sha256": contract["source_release_universe_sha256"],
            "source_cohort_type": contract["source_cohort_type"],
            "source_selection_protocol": contract["source_selection_protocol"],
            "ordered_asset_identities_sha256": contract[
                "ordered_asset_identities_sha256"
            ],
        },
        "selection_policy": {
            "algorithm": "exact ordered prefix of the frozen Table 1 manifest",
            "identity_fields_used": list(IDENTITY_FIELDS),
            "outcome_based_filtering": False,
            "selected_failures_retained_without_replacement": True,
        },
        "sampling": {
            "rest_state": "native URDF/PyBullet q=0",
            "single_joint_states_per_declared_nonfixed_joint": 21,
            "single_joint_other_joint_state": 0.0,
            "continuous_joint_interval": [-math.pi, math.pi],
            "sobol_states_per_asset_with_nonfixed_joint": 64,
            "sobol_scramble": True,
            "sobol_seed": 20260813,
        },
        "collision_policy": {
            "penetration_threshold_m": 1e-6,
            "surface_contact_allowed": True,
            "rest_panels": ["all_pair", "exclude_direct_parent_child"],
            "sweep_sobol_strict_policy": "exclude_direct_parent_child",
            "continuous_collision_detection": "not_run",
            "aor": "N/E: no stable exact overlap-volume implementation",
            "object_scale": "PyBullet q=0 collision-shape union AABB diagonal",
            "object_scale_protocol": SCALE_PROTOCOL,
        },
        "runtime": {
            "runner_sha256": sha256_file(SCRIPT),
            "collision_core_sha256": sha256_file(CORE_SCRIPT),
            "manifest_builder": current_runtime_identity(),
            "child": frozen_runtime,
        },
        "items": items,
    }
    manifest["ordered_selected_asset_ids_sha256"] = canonical_sha256(
        [item["asset_id"] for item in items]
    )
    manifest["items_sha256"] = canonical_sha256(items)
    return manifest


def validate_manifest(
    manifest: dict[str, Any],
    dataset_root: Path,
    table1_manifest: Path,
    *,
    qualification_smoke: bool,
    child_runtime: dict[str, Any] | None = None,
) -> None:
    dataset_root = dataset_root.resolve(strict=True)
    table1_manifest = table1_manifest.resolve(strict=True)
    sample_size = int(manifest.get("sample_size", -1))
    contract = load_table1_cohort(
        table1_manifest,
        sample_size=sample_size,
        qualification_smoke=qualification_smoke,
    )
    if manifest.get("protocol_id") != contract["protocol_id"]:
        raise RuntimeError("manifest protocol mismatch")
    if manifest.get("qualification_smoke") is not qualification_smoke:
        raise RuntimeError("manifest qualification mode mismatch")
    if Path(str(manifest.get("dataset_root"))).resolve() != dataset_root:
        raise RuntimeError("manifest dataset root mismatch")
    source = manifest.get("source", {})
    if Path(str(source.get("table1_manifest_path"))).resolve() != table1_manifest:
        raise RuntimeError("manifest Table 1 source path mismatch")
    if source.get("table1_manifest_sha256") != contract["source_manifest_sha256"]:
        raise RuntimeError("manifest Table 1 source hash mismatch")
    release_manifest = (
        dataset_root
        / str(read_json(table1_manifest).get("release_manifest", "dataset_chunks/manifest.json"))
    ).resolve(strict=True)
    if Path(str(source.get("release_manifest_path"))).resolve() != release_manifest:
        raise RuntimeError("manifest release source path mismatch")
    release_hash = sha256_file(release_manifest)
    if (
        release_hash != contract["source_release_manifest_sha256"]
        or source.get("release_manifest_sha256") != release_hash
    ):
        raise RuntimeError("manifest release source hash mismatch")
    if source.get("release_universe_sha256") != contract[
        "source_release_universe_sha256"
    ]:
        raise RuntimeError("manifest release universe mismatch")
    items = manifest.get("items", [])
    if not isinstance(items, list) or len(items) != sample_size:
        raise RuntimeError("manifest item count mismatch")
    selected = contract["selected"]
    runtime = manifest.get("runtime", {})
    for order, (item, source_row) in enumerate(zip(items, selected)):
        if any(item.get(key) != source_row.get(key) for key in IDENTITY_FIELDS):
            raise RuntimeError(f"manifest source identity mismatch at order {order}")
        if (
            item.get("order") != order
            or item.get("dataset_id") != f"artiverse_{order:04d}"
            or item.get("asset_root_relpath") != source_row["manifest_root"]
            or item.get("category") != source_row["raw_category"]
            or item.get("protocol_id") != contract["protocol_id"]
        ):
            raise RuntimeError(f"manifest item identity mismatch at order {order}")
        if item.get("input_identity_sha256") != _input_identity_sha256(item):
            raise RuntimeError(f"manifest frozen input hash mismatch at order {order}")
        if canonical_sha256(item.get("joint_specs", [])) != item.get(
            "joint_specs_sha256"
        ):
            raise RuntimeError(f"manifest joint specification hash mismatch at order {order}")
        meshes = item.get("collision_mesh_files", [])
        if canonical_sha256(meshes) != item.get("collision_mesh_inventory_sha256"):
            raise RuntimeError(f"manifest mesh inventory hash mismatch at order {order}")
        scale = item.get("scale_derivation", {})
        expected_scale_hash = (
            canonical_sha256(scale) if scale.get("status") == "PASS" else None
        )
        if item.get("scale_derivation_sha256") != expected_scale_hash:
            raise RuntimeError(f"manifest scale derivation hash mismatch at order {order}")
        if item.get("runtime_identity") != runtime.get("child"):
            raise RuntimeError(f"manifest item runtime mismatch at order {order}")
        if item.get("runtime_identity_sha256") != canonical_sha256(
            item.get("runtime_identity")
        ):
            raise RuntimeError(f"manifest item runtime hash mismatch at order {order}")
        movable = int(item.get("movable_dof_count", -1))
        if (
            item.get("rest_state_expected") != 1
            or item.get("single_state_expected") != SINGLE_SAMPLES * movable
            or item.get("sobol_state_expected")
            != (SOBOL_SAMPLES if movable > 0 else 0)
        ):
            raise RuntimeError(f"manifest frozen state denominator mismatch at order {order}")
    if canonical_sha256(items) != manifest.get("items_sha256"):
        raise RuntimeError("manifest items hash mismatch")
    expected_order_hash = canonical_sha256([row["asset_id"] for row in selected])
    if manifest.get("ordered_selected_asset_ids_sha256") != expected_order_hash:
        raise RuntimeError("manifest selected asset order mismatch")
    if runtime.get("runner_sha256") != sha256_file(SCRIPT):
        raise RuntimeError("Artiverse runner changed after cohort freeze")
    if runtime.get("collision_core_sha256") != EXPECTED_CORE_SHA256:
        raise RuntimeError("collision core hash mismatch")
    observed_runtime = child_runtime or current_runtime_identity()
    _load_core().require_runtime_match(runtime.get("child", {}), observed_runtime)


def freeze_item(
    source_row: dict[str, Any],
    audit: dict[str, Any],
    *,
    order: int,
    protocol_id: str,
    runtime_identity: dict[str, Any],
) -> dict[str, Any]:
    identity = {key: source_row[key] for key in IDENTITY_FIELDS}
    movable = int(audit.get("movable_dof_count", 0))
    item = {
        "protocol_id": protocol_id,
        "order": order,
        "dataset_id": f"artiverse_{order:04d}",
        **identity,
        "asset_root_relpath": str(source_row["manifest_root"]),
        "category": str(source_row["raw_category"]),
        **audit,
        "runtime_identity": runtime_identity,
        "runtime_identity_sha256": canonical_sha256(runtime_identity),
        "rest_state_expected": 1,
        "single_state_expected": 21 * movable,
        "sobol_state_expected": 64 if movable > 0 else 0,
    }
    item["input_identity_sha256"] = _input_identity_sha256(item)
    return item


def _add_source_identity(result: dict[str, Any], item: dict[str, Any]) -> None:
    for key in IDENTITY_FIELDS:
        result[key] = item.get(key)
    result["manifest_root"] = item.get("manifest_root")
    result["input_identity_sha256"] = item.get("input_identity_sha256")


def evaluate_asset(item: dict[str, Any], dataset_root: Path) -> dict[str, Any]:
    import pybullet as bullet

    core = _load_core()
    result = core.failure_record(item, "evaluation_not_completed")
    _add_source_identity(result, item)
    result["issues"] = []
    result["state_records"] = []
    result["runner_sha256"] = sha256_file(SCRIPT)
    result["collision_core_sha256"] = sha256_file(CORE_SCRIPT)
    result["runtime_identity"] = current_runtime_identity()
    try:
        validate_frozen_source_snapshot(item, dataset_root)
    except Exception as exc:  # noqa: BLE001
        result["issues"] = [f"{type(exc).__name__}: {exc}"]
        return result
    if not item.get("package_audit_success"):
        result["issues"] = [str(item.get("audit_issue") or "package_audit_failed")]
        return result
    urdf_path = dataset_root.resolve(strict=True) / str(item["primary_urdf_relpath"])
    joints = core.parse_urdf_joints(urdf_path)
    result["movable_dof_count"] = len(joints)
    result["range_evaluable_dof_count"] = sum(
        bool(row["range_evaluable"]) for row in joints
    )
    client = bullet.connect(bullet.DIRECT)
    body: int | None = None
    try:
        flags = int(
            bullet.URDF_USE_INERTIA_FROM_FILE
            | bullet.URDF_USE_SELF_COLLISION
            | bullet.URDF_USE_SELF_COLLISION_INCLUDE_PARENT
            | bullet.URDF_IGNORE_VISUAL_SHAPES
        )
        body = bullet.loadURDF(
            str(urdf_path),
            useFixedBase=True,
            flags=flags,
            physicsClientId=client,
        )
        result["load_success"] = True
        simulator_by_name: dict[str, int] = {}
        for index in range(bullet.getNumJoints(body, physicsClientId=client)):
            info = bullet.getJointInfo(body, index, physicsClientId=client)
            name = info[1].decode("utf-8") if isinstance(info[1], bytes) else str(info[1])
            simulator_by_name[name] = index
        missing_names = [row["name"] for row in joints if row["name"] not in simulator_by_name]
        if missing_names:
            raise RuntimeError(f"simulator joint mapping missing: {missing_names}")
        joint_indices = [simulator_by_name[row["name"]] for row in joints]
        for index in joint_indices:
            bullet.setJointMotorControl2(
                body,
                index,
                controlMode=bullet.VELOCITY_CONTROL,
                targetVelocity=0.0,
                force=0.0,
                physicsClientId=client,
            )
        direct_pairs = core._direct_parent_pairs(bullet, body, client)
        rest_values = [0.0] * len(joints)

        def observe(
            values: list[float],
            phase: str,
            sample_index: int,
            joint_name: str | None = None,
        ) -> dict[str, Any]:
            collision, readback_error = core._reset_and_observe(
                bullet,
                body,
                client,
                joint_indices,
                values,
                direct_pairs,
            )
            metric_key = (
                "all_pair_max_penetration_m"
                if phase == "rest"
                else "non_adjacent_max_penetration_m"
            )
            state = {
                "dataset_id": item["dataset_id"],
                **{key: item[key] for key in IDENTITY_FIELDS},
                "category": item["category"],
                "protocol_id": item["protocol_id"],
                "order": item["order"],
                "input_identity_sha256": item["input_identity_sha256"],
                "phase": phase,
                "sample_index": sample_index,
                "joint_name": joint_name,
                "joint_values_sha256": canonical_sha256(values),
                "reset_readback_max_abs_error": readback_error,
                "metric_max_penetration_m": float(collision[metric_key]),
                **collision,
            }
            result["state_records"].append(state)
            return state

        rest = observe(rest_values, "rest", 0)
        result["rest_state_executed"] = 1
        result["rest_all_pair_cf"] = rest["all_pair_illegal_penetration_count"] == 0
        result["rest_non_adjacent_cf"] = (
            rest["non_adjacent_illegal_penetration_count"] == 0
        )
        result["rest_non_adjacent_free"] = int(result["rest_non_adjacent_cf"])

        joint_sweep_passes = 0
        for joint_position, row in enumerate(joints):
            if not row["range_evaluable"]:
                result["issues"].append(
                    f"joint_range_not_evaluable:{row['name']}"
                )
                continue
            joint_free = True
            for sample_index, value in enumerate(core.single_joint_values(row)):
                values = list(rest_values)
                values[joint_position] = value
                state = observe(
                    values,
                    "single_joint_sweep",
                    sample_index,
                    str(row["name"]),
                )
                result["single_state_executed"] += 1
                free = state["non_adjacent_illegal_penetration_count"] == 0
                result["single_non_adjacent_free"] += int(free)
                joint_free = joint_free and free
            joint_sweep_passes += int(joint_free)
        result["joint_single_sweep_cf_passed"] = joint_sweep_passes

        if joints and all(row["range_evaluable"] for row in joints):
            for sample_index, values in enumerate(core.sobol_joint_values(joints)):
                state = observe(values, "multi_joint_sobol", sample_index)
                result["sobol_state_executed"] += 1
                result["sobol_non_adjacent_free"] += int(
                    state["non_adjacent_illegal_penetration_count"] == 0
                )

        result["single_joint_sweep_cf"] = bool(
            result["single_state_executed"] == result["single_state_expected"]
            and result["single_non_adjacent_free"] == result["single_state_expected"]
        )
        result["multi_joint_sobol_cf"] = bool(
            result["movable_dof_count"] > 0
            and result["range_evaluable_dof_count"] == result["movable_dof_count"]
            and result["sobol_state_executed"] == result["sobol_state_expected"]
            and result["sobol_non_adjacent_free"] == result["sobol_state_expected"]
        )
        expected_total = sum(
            int(result[f"{phase}_state_expected"])
            for phase in ("rest", "single", "sobol")
        )
        executed_total = sum(
            int(result[f"{phase}_state_executed"])
            for phase in ("rest", "single", "sobol")
        )
        result["measurement_complete"] = bool(
            result["range_evaluable_dof_count"] == result["movable_dof_count"]
            and executed_total == expected_total
        )
        result["strict_collision_pass"] = bool(
            result["measurement_complete"]
            and result["rest_non_adjacent_cf"]
            and result["single_joint_sweep_cf"]
            and result["multi_joint_sobol_cf"]
        )
    except Exception as exc:  # noqa: BLE001
        result["issues"].append(f"{type(exc).__name__}: {exc}")
    finally:
        if body is not None:
            bullet.removeBody(body, physicsClientId=client)
        bullet.disconnect(client)
    states = result["state_records"]
    if states:
        result["max_penetration_m"] = max(
            float(state["metric_max_penetration_m"]) for state in states
        )
        result["max_penetration_normalized"] = (
            float(result["max_penetration_m"])
            / float(result["object_bbox_diagonal_m"])
        )
        result["max_reset_readback_error"] = max(
            float(state["reset_readback_max_abs_error"]) for state in states
        )
    result["state_records_sha256"] = canonical_sha256(states)
    return result


def validate_frozen_asset_files(item: dict[str, Any], dataset_root: Path) -> None:
    asset_root = _contained_asset_root(dataset_root, str(item["asset_root_relpath"]))
    urdf_path = dataset_root.resolve(strict=True) / str(item["primary_urdf_relpath"])
    if not urdf_path.is_file() or sha256_file(urdf_path) != item.get("urdf_sha256"):
        raise RuntimeError("URDF content drift after freeze")
    observed, unsafe = collision_mesh_inventory(dataset_root, asset_root, urdf_path)
    if unsafe or observed != item.get("collision_mesh_files") or canonical_sha256(
        observed
    ) != item.get("collision_mesh_inventory_sha256"):
        raise RuntimeError("collision mesh inventory drift after freeze")


def validate_frozen_source_snapshot(
    item: dict[str, Any],
    dataset_root: Path,
    *,
    rederive_scale: bool = False,
) -> None:
    source_row = {key: item[key] for key in IDENTITY_FIELDS}
    if not item.get("package_audit_success"):
        observed = audit_asset(dataset_root, source_row)
        keys = tuple(_empty_audit())
        if any(observed.get(key) != item.get(key) for key in keys):
            raise RuntimeError("package audit snapshot drift after freeze")
        return
    asset_root, urdf_path = _resolve_primary_urdf(dataset_root, source_row)
    observed_relpath = urdf_path.relative_to(dataset_root.resolve(strict=True)).as_posix()
    if observed_relpath != item.get("primary_urdf_relpath"):
        raise RuntimeError("primary URDF selection drift after freeze")
    validate_frozen_asset_files(item, dataset_root)
    joints = _load_core().parse_urdf_joints(urdf_path)
    if (
        canonical_sha256(joints) != item.get("joint_specs_sha256")
        or len(joints) != int(item.get("movable_dof_count", -1))
        or sum(bool(row["range_evaluable"]) for row in joints)
        != int(item.get("range_evaluable_dof_count", -1))
        or _valid_tree(urdf_path) is not bool(item.get("valid_tree"))
    ):
        raise RuntimeError("URDF kinematic snapshot drift after freeze")
    if asset_root.relative_to(dataset_root.resolve(strict=True)).as_posix() != item.get(
        "asset_root_relpath"
    ):
        raise RuntimeError("asset root snapshot drift after freeze")
    if rederive_scale and item.get("package_audit_success"):
        scale = derive_collision_aabb(urdf_path)
        if (
            scale != item.get("scale_derivation")
            or canonical_sha256(scale) != item.get("scale_derivation_sha256")
            or not math.isclose(
                float(scale["diagonal_m"]),
                float(item["object_bbox_diagonal_m"]),
                rel_tol=0.0,
                abs_tol=0.0,
            )
        ):
            raise RuntimeError("collision scale derivation drift after freeze")


def normalize_executable_path(path: Path, cwd: Path) -> Path:
    return _load_core().normalize_executable_path(path, cwd)


def probe_runtime_identity(python: Path, result_path: Path) -> dict[str, Any]:
    result_path.unlink(missing_ok=True)
    environment = dict(os.environ)
    environment.update(
        {
            "OPENBLAS_NUM_THREADS": "1",
            "OMP_NUM_THREADS": "1",
            "MKL_NUM_THREADS": "1",
            "NUMEXPR_NUM_THREADS": "1",
            "VECLIB_MAXIMUM_THREADS": "1",
        }
    )
    completed = subprocess.run(
        [
            str(python),
            str(SCRIPT),
            "--phase",
            "runtime",
            "--runtime-result",
            str(result_path),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=120,
        env=environment,
        check=False,
    )
    if completed.returncode != 0 or not result_path.is_file():
        raise RuntimeError(
            "child runtime probe failed: "
            f"returncode={completed.returncode}, output={completed.stdout[-2000:]}"
        )
    return read_json(result_path)


def prepare(
    dataset_root: Path,
    table1_manifest: Path,
    output: Path,
    *,
    sample_size: int,
    qualification_smoke: bool,
    child_python: Path,
    workers: int,
) -> dict[str, Any]:
    output.mkdir(parents=True, exist_ok=True)
    runtime_path = output / "child_runtime_probe.json"
    child_runtime = probe_runtime_identity(child_python, runtime_path)
    _load_core().require_runtime_match(current_runtime_identity(), child_runtime)
    manifest_path = output / "frozen_manifest.json"
    if manifest_path.exists():
        manifest = read_json(manifest_path)
        if int(manifest.get("sample_size", -1)) != sample_size:
            raise RuntimeError("existing manifest requested sample size mismatch")
        validate_manifest(
            manifest,
            dataset_root,
            table1_manifest,
            qualification_smoke=qualification_smoke,
            child_runtime=child_runtime,
        )
        return manifest
    manifest = build_manifest(
        dataset_root,
        table1_manifest,
        sample_size=sample_size,
        qualification_smoke=qualification_smoke,
        child_runtime=child_runtime,
        workers=workers,
    )
    atomic_json(manifest_path, manifest)
    return manifest


def run_pair_policy_smoke(output: Path) -> dict[str, Any]:
    return _load_core().run_pair_policy_smoke(output)


def run_child(item_path: Path, dataset_root: Path, result_path: Path) -> None:
    item = read_json(item_path)
    result = evaluate_asset(item, dataset_root)
    atomic_json(result_path, result)


def _job_prefix(item: dict[str, Any]) -> str:
    return f"{int(item['order']):04d}_{item['dataset_id']}"


def child_result_path(output: Path, item: dict[str, Any]) -> Path:
    return output / "children" / f"{_job_prefix(item)}.json"


def _valid_cached_child(
    path: Path, item: dict[str, Any], runner_hash: str
) -> bool:
    if not path.is_file():
        return False
    try:
        result = read_json(path)
    except (OSError, json.JSONDecodeError):
        return False
    return result_matches_item(result, item, runner_hash)


def run_one_subprocess(
    item_path: Path,
    item: dict[str, Any],
    dataset_root: Path,
    child_result: Path,
    child_log: Path,
    timeout: int,
    python: Path,
    runner_hash: str,
) -> dict[str, Any]:
    try:
        validate_frozen_source_snapshot(item, dataset_root)
    except Exception as exc:  # noqa: BLE001
        result = failure_record(
            item, f"frozen_asset_files_drift:{type(exc).__name__}: {exc}"
        )
        result.update(
            {
                "runner_sha256": runner_hash,
                "child_returncode": None,
                "child_timed_out": False,
                "child_log": str(child_log),
                "cache_reused": False,
            }
        )
        atomic_json(child_result, result)
        return result
    if _valid_cached_child(child_result, item, runner_hash):
        result = read_json(child_result)
        result["cache_reused"] = True
        return result
    environment = dict(os.environ)
    environment.update(
        {
            "OPENBLAS_NUM_THREADS": "1",
            "OMP_NUM_THREADS": "1",
            "MKL_NUM_THREADS": "1",
            "NUMEXPR_NUM_THREADS": "1",
            "VECLIB_MAXIMUM_THREADS": "1",
        }
    )
    command = [
        str(python),
        str(SCRIPT),
        "--phase",
        "child",
        "--dataset-root",
        str(dataset_root),
        "--child-item",
        str(item_path),
        "--child-result",
        str(child_result),
    ]
    child_log.parent.mkdir(parents=True, exist_ok=True)
    timed_out = False
    with child_log.open("wb") as log:
        try:
            completed = subprocess.run(
                command,
                stdout=log,
                stderr=subprocess.STDOUT,
                timeout=timeout,
                env=environment,
                check=False,
            )
            returncode = completed.returncode
        except subprocess.TimeoutExpired:
            returncode = -9
            timed_out = True
    if returncode == 0 and _valid_cached_child(child_result, item, runner_hash):
        result = read_json(child_result)
    else:
        issue = (
            "child_timeout"
            if timed_out
            else (
                "child_invalid_result_returncode_0"
                if returncode == 0
                else f"child_exit_{returncode}"
            )
        )
        result = failure_record(item, issue, timed_out=timed_out)
        result["runner_sha256"] = runner_hash
    result.update(
        {
            "child_returncode": returncode,
            "child_timed_out": timed_out,
            "child_log": str(child_log),
            "cache_reused": False,
        }
    )
    atomic_json(child_result, result)
    return result


def execute(
    manifest: dict[str, Any],
    dataset_root: Path,
    output: Path,
    *,
    workers: int,
    timeout: int,
    python: Path,
) -> list[dict[str, Any]]:
    if workers <= 0:
        raise ValueError("workers must be positive")
    run_pair_policy_smoke(output)
    runtime_probe = probe_runtime_identity(python, output / "child_runtime_probe.json")
    _load_core().require_runtime_match(manifest["runtime"]["child"], runtime_probe)
    inputs = output / "inputs"
    children = output / "children"
    logs = output / "child_logs"
    for directory in (inputs, children, logs):
        directory.mkdir(parents=True, exist_ok=True)
    jobs = []
    for item in manifest["items"]:
        prefix = _job_prefix(item)
        item_path = inputs / f"{prefix}.json"
        atomic_json(item_path, item)
        jobs.append(
            (
                item,
                item_path,
                children / f"{prefix}.json",
                logs / f"{prefix}.log",
            )
        )
    runner_hash = str(manifest["runtime"]["runner_sha256"])
    by_order: dict[int, dict[str, Any]] = {}
    started = time.time()
    with ThreadPoolExecutor(max_workers=workers) as executor:
        futures = {
            executor.submit(
                run_one_subprocess,
                item_path,
                item,
                dataset_root,
                child_path,
                log_path,
                timeout,
                python,
                runner_hash,
            ): item
            for item, item_path, child_path, log_path in jobs
        }
        for completed, future in enumerate(as_completed(futures), 1):
            item = futures[future]
            result = future.result()
            by_order[int(item["order"])] = result
            atomic_json(
                output / "progress.json",
                {
                    "protocol_id": manifest["protocol_id"],
                    "status": (
                        "RUNNING" if completed < len(jobs) else "CHILDREN_COMPLETE"
                    ),
                    "completed_assets": completed,
                    "total_assets": len(jobs),
                    "load_successes": sum(
                        bool(row["load_success"]) for row in by_order.values()
                    ),
                    "measurement_complete_assets": sum(
                        bool(row["measurement_complete"])
                        for row in by_order.values()
                    ),
                    "timeouts": sum(
                        bool(row["child_timed_out"]) for row in by_order.values()
                    ),
                    "elapsed_seconds": time.time() - started,
                    "last_completed_asset_id": item["asset_id"],
                },
            )
            print(
                f"table4 {completed}/{len(jobs)} rank={item['selection_rank']} "
                f"load={int(result['load_success'])} "
                f"complete={int(result['measurement_complete'])}",
                flush=True,
            )
    return [by_order[index] for index in range(len(jobs))]


def summarize(manifest: dict[str, Any], output: Path) -> dict[str, Any]:
    records = []
    state_records = []
    runner_hash = str(manifest["runtime"]["runner_sha256"])
    for item in manifest["items"]:
        path = child_result_path(output, item)
        if not _valid_cached_child(path, item, runner_hash):
            raise RuntimeError(f"missing or stale child result: {path}")
        row = read_json(path)
        asset_states = row.pop("state_records", [])
        if not result_matches_item(row, item, runner_hash, asset_states):
            raise RuntimeError(f"child state closure mismatch: {path}")
        state_records.extend(asset_states)
        records.append(row)
    summary = summarize_records(manifest, records)
    atomic_json(output / "asset_records.json", records)
    atomic_jsonl(output / "state_records.jsonl", state_records)
    atomic_json(output / "summary.json", summary)
    render_report(summary, output)
    return summary


def _format_metric(row: dict[str, Any], numerator_key: str = "passed") -> str:
    numerator = row[numerator_key]
    denominator = row["denominator"]
    percentage = 100.0 * row["rate"] if row["rate"] is not None else float("nan")
    return f"{numerator}/{denominator} ({percentage:.3f}%)"


def report_text(summary: dict[str, Any]) -> str:
    metrics = summary["metrics"]
    maximum = metrics["max_penetration"]
    lines = [
        f"# {summary['cohort']['label']}: URDF Sim-Ready Table 4",
        "",
        f"Status: **{summary['status']}**",
        "",
        "This is the globally fixed Table 1 sample from the Artiverse pre-release 3,544-asset universe. It is not a full-release or category-balanced result.",
        "",
        "| Metric | Result |",
        "|---|---:|",
        f"| Rest All-pair CF | {_format_metric(metrics['rest_all_pair_cf'])} |",
        f"| Rest Non-adjacent CF | {_format_metric(metrics['rest_non_adjacent_cf'])} |",
        f"| Single-joint Sweep CF | {_format_metric(metrics['single_joint_sweep_cf'])} |",
        f"| Multi-joint Sobol CF | {_format_metric(metrics['multi_joint_sobol_cf'])} |",
        f"| Collision-state Rate | {_format_metric(metrics['collision_state_rate'], 'collision_states')} |",
        "| AOR | N/E |",
        f"| Max Penetration | {maximum['maximum_observed_normalized']} (fully measured {maximum['fully_measured_assets']}/{maximum['denominator']}; observed {maximum['observed_assets']}/{maximum['denominator']}; {maximum['status']}) |",
        f"| Collision-free Range | {_format_metric(metrics['collision_free_range'], 'passed_states')} |",
        f"| Strict Collision Pass | {_format_metric(metrics['strict_collision_pass'])} |",
        "",
        "Collision-state Rate is fail-closed. Unexecuted configurations remain in the frozen denominator and count as non-free.",
        "",
        "AOR is N/E because no stable exact overlap-volume calculation was run. Sweeps are discrete; no CCD, semantic-joint, or physical-dynamics claim is made.",
    ]
    return "\n".join(lines) + "\n"


def render_report(summary: dict[str, Any], output: Path) -> None:
    atomic_text(output / "report.md", report_text(summary))


def _read_jsonl_strict(path: Path) -> list[dict[str, Any]]:
    rows = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                raise RuntimeError(
                    f"blank line in {path.name} at line {line_number}"
                )
            value = json.loads(line)
            if not isinstance(value, dict):
                raise RuntimeError(
                    f"non-object row in {path.name} at line {line_number}"
                )
            rows.append(value)
    return rows


def verify(manifest: dict[str, Any], output: Path) -> dict[str, Any]:
    checks: dict[str, bool] = {}
    errors: dict[str, str] = {}

    def check(name: str, operation: Any) -> Any:
        try:
            value = operation()
            passed = bool(value) if isinstance(value, bool) else True
            checks[name] = passed
            if not passed:
                errors[name] = "check returned false"
            return value
        except Exception as exc:  # noqa: BLE001
            checks[name] = False
            errors[name] = f"{type(exc).__name__}: {exc}"
            return None

    dataset_root = Path(str(manifest.get("dataset_root", "")))
    source = manifest.get("source", {})
    table1_manifest = Path(str(source.get("table1_manifest_path", "")))
    qualification_smoke = bool(manifest.get("qualification_smoke"))
    runtime_probe = check(
        "child_runtime_receipt_readable",
        lambda: read_json(output / "child_runtime_probe.json"),
    )
    check(
        "manifest_revalidates_against_authoritative_sources",
        lambda: validate_manifest(
            manifest,
            dataset_root,
            table1_manifest,
            qualification_smoke=qualification_smoke,
            child_runtime=runtime_probe,
        ),
    )
    check(
        "runner_sha256_matches_manifest",
        lambda: sha256_file(SCRIPT) == manifest["runtime"]["runner_sha256"],
    )
    check(
        "collision_core_sha256_matches_pin",
        lambda: sha256_file(CORE_SCRIPT)
        == manifest["runtime"]["collision_core_sha256"]
        == EXPECTED_CORE_SHA256,
    )
    check(
        "current_runtime_matches_manifest",
        lambda: current_runtime_identity() == manifest["runtime"]["child"],
    )

    records = check(
        "asset_records_readable",
        lambda: read_json(output / "asset_records.json"),
    )
    states = check(
        "state_records_jsonl_readable",
        lambda: _read_jsonl_strict(output / "state_records.jsonl"),
    )
    summary = check("summary_readable", lambda: read_json(output / "summary.json"))
    if not isinstance(records, list):
        records = []
    if not isinstance(states, list):
        states = []
    items = manifest.get("items", []) if isinstance(manifest.get("items"), list) else []
    check(
        "sample_size_exact",
        lambda: len(records) == int(manifest["sample_size"]) == len(items),
    )
    check(
        "record_order_matches_manifest",
        lambda: [row.get("dataset_id") for row in records]
        == [item.get("dataset_id") for item in items],
    )

    states_by_id: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for state in states:
        states_by_id[str(state.get("dataset_id"))].append(state)
    known_ids = {str(item.get("dataset_id")) for item in items}
    check(
        "no_unknown_state_asset_ids",
        lambda: set(states_by_id).issubset(known_ids),
    )
    closure_ok = len(records) == len(items)
    if closure_ok:
        runner_hash = str(manifest.get("runtime", {}).get("runner_sha256"))
        for item, record in zip(items, records):
            asset_states = states_by_id.get(str(item["dataset_id"]), [])
            if not result_matches_item(record, item, runner_hash, asset_states):
                closure_ok = False
                break
    checks["state_records_close_against_assets"] = closure_ok
    if not closure_ok:
        errors["state_records_close_against_assets"] = "result/state closure mismatch"
    expected_state_order = [
        state
        for item in items
        for state in states_by_id.get(str(item.get("dataset_id")), [])
    ]
    check("state_global_order_matches_manifest", lambda: expected_state_order == states)
    check(
        "state_record_count_matches_assets",
        lambda: len(states)
        == sum(
            int(record.get(f"{phase}_state_executed", -1))
            for record in records
            for phase in ("rest", "single", "sobol")
        ),
    )

    frozen_sources_ok = True
    frozen_source_error = None
    for item in items:
        try:
            validate_frozen_source_snapshot(
                item, dataset_root, rederive_scale=True
            )
        except Exception as exc:  # noqa: BLE001
            frozen_sources_ok = False
            frozen_source_error = f"{item.get('asset_id')}: {type(exc).__name__}: {exc}"
            break
    checks["frozen_source_snapshots_match"] = frozen_sources_ok
    if frozen_source_error is not None:
        errors["frozen_source_snapshots_match"] = frozen_source_error

    recomputed = None
    if len(records) == len(items):
        recomputed = check(
            "summary_recomputable", lambda: summarize_records(manifest, records)
        )
    else:
        checks["summary_recomputable"] = False
        errors["summary_recomputable"] = "record count mismatch"
    check("summary_recomputes_exactly", lambda: summary == recomputed)
    check(
        "report_recomputes_exactly",
        lambda: (output / "report.md").read_text(encoding="utf-8")
        == report_text(recomputed),
    )
    check(
        "single_state_denominator_frozen",
        lambda: sum(int(row["single_state_expected"]) for row in records)
        == SINGLE_SAMPLES * sum(int(item["movable_dof_count"]) for item in items),
    )
    check(
        "sobol_state_denominator_frozen",
        lambda: sum(int(row["sobol_state_expected"]) for row in records)
        == SOBOL_SAMPLES * sum(int(item["movable_dof_count"]) > 0 for item in items),
    )
    check(
        "aor_remains_not_evaluable",
        lambda: recomputed["metrics"]["aor"]["status"] == "N/E",
    )
    check(
        "claim_boundary_preserved",
        lambda: recomputed["claim_boundary"]
        == {
            "continuous_collision_detection": "not_run",
            "semantic_joint_correctness": "not_evaluated",
            "physical_dynamics_validity": "not_evaluated",
            "full_release_result": False,
            "shared_category_balanced_result": False,
        },
    )
    pair_receipt = check(
        "pair_policy_receipt_readable",
        lambda: read_json(output / "pair_policy_smoke.json"),
    )
    check(
        "pair_policy_smoke_semantics_pass",
        lambda: (
            pair_receipt["protocol_id"]
            == "urdf_table4_pybullet_pair_policy_smoke_v1"
            and pair_receipt["status"] == "PASS"
            and int(pair_receipt["all_pair_illegal_penetration_count"]) > 0
            and int(pair_receipt["non_adjacent_illegal_penetration_count"]) == 0
            and int(pair_receipt["pybullet_api_version"])
            == int(manifest["runtime"]["child"]["pybullet_api_version"])
        ),
    )

    artifact_names = (
        "frozen_manifest.json",
        "child_runtime_probe.json",
        "pair_policy_smoke.json",
        "asset_records.json",
        "state_records.jsonl",
        "summary.json",
        "report.md",
    )
    artifact_hashes = {}
    for name in artifact_names:
        path = output / name
        if path.is_file():
            artifact_hashes[name] = sha256_file(path)
    receipt = {
        "protocol_id": VERIFY_PROTOCOL_ID,
        "status": "PASS" if checks and all(checks.values()) else "FAIL",
        "checked_at_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "checks": checks,
        "errors": errors,
        "artifact_sha256": artifact_hashes,
        "runner_sha256": sha256_file(SCRIPT),
        "collision_core_sha256": sha256_file(CORE_SCRIPT),
    }
    atomic_json(output / "verification.json", receipt)
    if receipt["status"] != "PASS":
        failed = sorted(name for name, passed in checks.items() if not passed)
        raise RuntimeError(f"verification failed: {failed}")
    return receipt


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--phase",
        choices=("all", "prepare", "run", "summarize", "verify", "child", "runtime"),
        default="all",
    )
    parser.add_argument("--dataset-root", type=Path, default=DEFAULT_DATASET)
    parser.add_argument("--table1-manifest", type=Path, default=DEFAULT_TABLE1_MANIFEST)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--sample-size", type=int, default=SAMPLE_SIZE)
    parser.add_argument("--qualification-smoke", action="store_true")
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--timeout", type=int, default=900)
    parser.add_argument("--python", type=Path, default=Path(sys.executable))
    parser.add_argument("--child-item", type=Path)
    parser.add_argument("--child-result", type=Path)
    parser.add_argument("--runtime-result", type=Path)
    args = parser.parse_args()

    if args.phase == "runtime":
        if args.runtime_result is None:
            parser.error("runtime phase requires --runtime-result")
        atomic_json(args.runtime_result, current_runtime_identity())
        return 0

    dataset_root = args.dataset_root.resolve(strict=True)
    if args.phase == "child":
        if args.child_item is None or args.child_result is None:
            parser.error("child phase requires --child-item and --child-result")
        run_child(args.child_item, dataset_root, args.child_result)
        return 0

    table1_manifest = args.table1_manifest.resolve(strict=True)
    output = args.output.resolve()
    child_python = normalize_executable_path(args.python, Path.cwd())
    manifest: dict[str, Any]
    if args.phase in {"all", "prepare"}:
        manifest = prepare(
            dataset_root,
            table1_manifest,
            output,
            sample_size=args.sample_size,
            qualification_smoke=args.qualification_smoke,
            child_python=child_python,
            workers=args.workers,
        )
        print(
            json.dumps(
                {
                    "manifest": str(output / "frozen_manifest.json"),
                    "sample_size": manifest["sample_size"],
                    "ordered_selected_asset_ids_sha256": manifest[
                        "ordered_selected_asset_ids_sha256"
                    ],
                },
                indent=2,
            ),
            flush=True,
        )
        if args.phase == "prepare":
            return 0
    else:
        manifest = read_json(output / "frozen_manifest.json")
        runtime_probe = probe_runtime_identity(
            child_python, output / "child_runtime_probe.json"
        )
        validate_manifest(
            manifest,
            dataset_root,
            table1_manifest,
            qualification_smoke=args.qualification_smoke,
            child_runtime=runtime_probe,
        )

    if args.phase in {"all", "run"}:
        execute(
            manifest,
            dataset_root,
            output,
            workers=args.workers,
            timeout=args.timeout,
            python=child_python,
        )
        if args.phase == "run":
            return 0
    if args.phase in {"all", "summarize"}:
        summary = summarize(manifest, output)
        print(json.dumps(summary["cohort"], indent=2), flush=True)
        if args.phase == "summarize":
            return 0
    if args.phase in {"all", "verify"}:
        receipt = verify(manifest, output)
        print(json.dumps(receipt, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
