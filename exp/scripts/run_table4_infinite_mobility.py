#!/usr/bin/env python3
"""Freeze and audit the Infinite Mobility Table 4 collision protocol.

Infinite Mobility's exported URDFs contain no native ``collision`` elements.
The evaluator therefore performs the complete input/state-plan audit, then
blocks collision queries before they can produce a vacuous "collision-free"
result.  Collision-dependent headline metrics are reported as N/E with the
explicit reason ``BLOCKED_NATIVE_COLLISION_GEOMETRY_ABSENT``.
"""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import platform
import sys
from typing import Any
import xml.etree.ElementTree as ET


SCRIPT_PATH = Path(__file__).resolve()
EXP_ROOT = SCRIPT_PATH.parents[1]
REPO_ROOT = SCRIPT_PATH.parents[2]
if str(SCRIPT_PATH.parent) not in sys.path:
    sys.path.insert(0, str(SCRIPT_PATH.parent))

import infinite_mobility_table123_common as cohort_common  # noqa: E402


DATASET = "Infinite Mobility"
CLASSIFICATION = "FORMAL"
PROTOCOL_ID = "urdf_sim_ready_table4_infinite_mobility_n720_v1"
SCHEMA_VERSION = "table4-infinite-mobility/v1"
DEFAULT_COHORT_MANIFEST = EXP_ROOT / "runtime/infinite_mobility_urdf_table123_cohort/manifest.json"
DEFAULT_TABLE2_MANIFEST = EXP_ROOT / "runtime/table2_infinite_mobility_720/manifest.json"
DEFAULT_TABLE3_RECEIPT = EXP_ROOT / "runtime/table3_infinite_mobility_720"
DEFAULT_OUTPUT = EXP_ROOT / "runtime/table4_infinite_mobility_720"
PROTOCOL_DOCUMENT = EXP_ROOT / "URDF-Sim-Ready-Automatic-Evaluation.md"

FORMAL_N_EVAL = 720
FORMAL_J_EVAL = 4723
FORMAL_ZERO_JOINT_ASSETS = 55
FORMAL_RANGE_EVALUABLE_JOINTS = 4687
FORMAL_ALL_RANGE_MOVABLE_ASSETS = 629
FORMAL_COHORT_MANIFEST_SHA256 = "cfd9c06ea35dcec57c53d44dbf52903ecba6f33321075495c97c58fe30d23c08"
FORMAL_COHORT_CONTENT_SHA256 = "f5e29f1becd47cae991f5d238dff3f86b2b009365738df3e46cdbea297032c23"
FORMAL_COHORT_ARTIFACT_MANIFEST_SHA256 = "ac31de70d50ed7153178482bb5283659be94fb5945cc2b7157754ac61dfc5439"
FORMAL_TABLE2_MANIFEST_SHA256 = "3dce6436aac2d25507d7843a3e0e5cbee130e83e0c24c2bfbfb08467ca356290"
FORMAL_TABLE2_CONTENT_SHA256 = "f1cc7c062767ec6e6cb8d05caea122f17baf42d1a9b12e5bc40d16648c3306c3"
FORMAL_TABLE2_RECORDS_SHA256 = "d488501734a41d4b814c294f7ad94ed529df72b0f99cfaed8b1d19a3bf1c2ada"
FORMAL_TABLE3_MANIFEST_SHA256 = "52d03061d150e23f5f97e0227931047379969a5518c5448a14e7062a3ed6d611"
FORMAL_TABLE3_CONTENT_SHA256 = "28ac7cec9b80221786c14dca2e546e7ecca73c813a6ad9a101dc43d3d4a6335b"
FORMAL_TABLE3_RECORDS_SHA256 = "e1ebf268e6839869e9d7e8d98e2ae0411e4ed17dea28b5c7692bef326b6f4113"

SINGLE_SAMPLES = 21
SOBOL_SAMPLES = 64
SOBOL_SEED = 20260813
PENETRATION_THRESHOLD_M = 1e-6
BLOCKED_NATIVE_COLLISION_GEOMETRY_ABSENT = (
    "BLOCKED_NATIVE_COLLISION_GEOMETRY_ABSENT"
)
PROTOCOL_SNAPSHOT_NAME = "protocol_snapshot.md"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_bytes(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode(
        "utf-8"
    )


def canonical_sha256(value: Any) -> str:
    return hashlib.sha256(canonical_bytes(value)).hexdigest()


def atomic_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def atomic_text(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(value, encoding="utf-8")
    temporary.replace(path)


def utc_now() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                raise ValueError(f"blank JSONL row at {path}:{line_number}")
            value = json.loads(line)
            if not isinstance(value, dict):
                raise ValueError(f"non-object JSONL row at {path}:{line_number}")
            rows.append(value)
    return rows


def manifest_content_hash(value: dict[str, Any]) -> str:
    payload = dict(value)
    payload.pop("manifest_content_sha256", None)
    return canonical_sha256(payload)


def _contained(path: Path, root: Path) -> Path:
    resolved = path.resolve(strict=True)
    root_resolved = root.resolve(strict=True)
    try:
        resolved.relative_to(root_resolved)
    except ValueError as exc:
        raise ValueError(f"path escapes trusted root: {resolved}") from exc
    return resolved


def parse_joint_specs(urdf_path: Path) -> list[dict[str, Any]]:
    root = ET.parse(urdf_path).getroot()
    if root.tag != "robot":
        raise ValueError(f"URDF root is {root.tag!r}, expected robot")
    rows: list[dict[str, Any]] = []
    for xml_index, node in enumerate(root.findall("joint")):
        joint_type = str(node.get("type", ""))
        if joint_type == "fixed":
            continue
        lower: float | None = None
        upper: float | None = None
        limit = node.find("limit")
        if joint_type in {"revolute", "prismatic"} and limit is not None:
            try:
                lower = float(limit.get("lower", ""))
                upper = float(limit.get("upper", ""))
            except (TypeError, ValueError):
                lower = upper = None
        range_evaluable = joint_type == "continuous"
        if joint_type in {"revolute", "prismatic"}:
            range_evaluable = (
                lower is not None
                and upper is not None
                and lower == lower
                and upper == upper
                and upper - lower > 1e-12
            )
        rows.append(
            {
                "xml_index": xml_index,
                "name": node.get("name", f"joint_{xml_index}"),
                "type": joint_type,
                "lower": lower,
                "upper": upper,
                "range_evaluable": range_evaluable,
            }
        )
    return rows


def valid_rooted_tree(urdf_path: Path) -> bool:
    root = ET.parse(urdf_path).getroot()
    links = [str(node.get("name", "")) for node in root.findall("link")]
    if not links or any(not name for name in links) or len(set(links)) != len(links):
        return False
    link_set = set(links)
    indegree = {name: 0 for name in links}
    graph: dict[str, list[str]] = {name: [] for name in links}
    for joint in root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        parent_name = parent.get("link", "") if parent is not None else ""
        child_name = child.get("link", "") if child is not None else ""
        if parent_name not in link_set or child_name not in link_set or parent_name == child_name:
            return False
        indegree[child_name] += 1
        graph[parent_name].append(child_name)
    roots = [name for name, count in indegree.items() if count == 0]
    if len(roots) != 1 or len(root.findall("joint")) != len(links) - 1:
        return False
    seen: set[str] = set()
    stack = [roots[0]]
    while stack:
        current = stack.pop()
        if current in seen:
            return False
        seen.add(current)
        stack.extend(graph[current])
    return len(seen) == len(links)


def formal_state_plan() -> dict[str, int]:
    """Return the frozen Table 4 intent-to-evaluate state denominators."""

    rest = FORMAL_N_EVAL
    single = SINGLE_SAMPLES * FORMAL_J_EVAL
    # The Table 4 denominator reserves R states for every asset with at least
    # one declared movable joint; invalid ranges make those states unexecuted,
    # rather than removing the asset from the intent-to-evaluate denominator.
    sobol = SOBOL_SAMPLES * (FORMAL_N_EVAL - FORMAL_ZERO_JOINT_ASSETS)
    return {
        "n_eval": FORMAL_N_EVAL,
        "j_eval": FORMAL_J_EVAL,
        "zero_joint_assets": FORMAL_ZERO_JOINT_ASSETS,
        "movable_assets": FORMAL_N_EVAL - FORMAL_ZERO_JOINT_ASSETS,
        "range_evaluable_joints": FORMAL_RANGE_EVALUABLE_JOINTS,
        "all_range_evaluable_movable_assets": FORMAL_ALL_RANGE_MOVABLE_ASSETS,
        "rest_expected": rest,
        "single_expected": single,
        "sobol_expected": sobol,
        "total_expected": rest + single + sobol,
    }


def blocked_record(item: dict[str, Any]) -> dict[str, Any]:
    expected = sum(
        int(item.get(field, 0))
        for field in ("rest_state_expected", "single_state_expected", "sobol_state_expected")
    )
    return {
        **item,
        "status": "blocked",
        "result_origin": "preflight_blocked",
        "collision_metric_status": BLOCKED_NATIVE_COLLISION_GEOMETRY_ABSENT,
        "native_collision_element_count": int(item.get("native_collision_element_count", 0)),
        "rest_state_executed": 0,
        "single_state_executed": 0,
        "sobol_state_executed": 0,
        "unexecuted_state_count": expected,
        "state_records": [],
        "rest_all_pair_cf": None,
        "rest_non_adjacent_cf": None,
        "single_joint_sweep_cf": None,
        "multi_joint_sobol_cf": None,
        "collision_state_count": None,
        "strict_collision_pass": None,
        "max_penetration_normalized": None,
        "aor": None,
    }


def _n_e_metric(denominator: int, *, reason: str = BLOCKED_NATIVE_COLLISION_GEOMETRY_ABSENT) -> dict[str, Any]:
    return {
        "status": "N/E",
        "reason": reason,
        "passed": None,
        "denominator": denominator,
        "rate": None,
    }


def summarize_blocked(
    records: list[dict[str, Any]], *, n_eval: int, j_eval: int
) -> dict[str, Any]:
    rest_expected = sum(int(row["rest_state_expected"]) for row in records)
    single_expected = sum(int(row["single_state_expected"]) for row in records)
    sobol_expected = sum(int(row["sobol_state_expected"]) for row in records)
    total_expected = rest_expected + single_expected + sobol_expected
    native_collision_elements = sum(
        int(row.get("native_collision_element_count", 0)) for row in records
    )
    if native_collision_elements != 0:
        raise ValueError("blocked summary requires zero native collision elements")
    metrics = {
        "rest_all_pair_cf": _n_e_metric(n_eval),
        "rest_non_adjacent_cf": _n_e_metric(n_eval),
        "single_joint_sweep_cf": _n_e_metric(n_eval),
        "multi_joint_sobol_cf": _n_e_metric(n_eval),
        "collision_state_rate": {
            **_n_e_metric(total_expected),
            "collision_states": None,
            "unexecuted_states": total_expected,
        },
        "aor": {
            "status": "N/E",
            "reason": BLOCKED_NATIVE_COLLISION_GEOMETRY_ABSENT,
        },
        "max_penetration": {
            "status": "N/E",
            "reason": BLOCKED_NATIVE_COLLISION_GEOMETRY_ABSENT,
            "maximum_observed_normalized": None,
            "measured_assets": 0,
            "denominator": n_eval,
            "normalization": "pybullet_q0_collision_shape_union_aabb_v1 (undefined for empty collision-shape union)",
        },
        "collision_free_range": {
            **_n_e_metric(single_expected),
            "passed_states": None,
            "unexecuted_states": single_expected,
        },
        "strict_collision_pass": _n_e_metric(n_eval),
    }
    return {
        "schema_version": SCHEMA_VERSION,
        "status": "BLOCKED",
        "classification": CLASSIFICATION,
        "dataset": DATASET,
        "protocol_id": PROTOCOL_ID,
        "n_eval": n_eval,
        "j_eval": j_eval,
        "zero_joint_assets": sum(int(row.get("movable_dof_count", 0)) == 0 for row in records),
        "metrics": metrics,
        "state_plan": {
            "rest_expected": rest_expected,
            "single_expected": single_expected,
            "sobol_expected": sobol_expected,
            "total_expected": total_expected,
            "single_executable_if_geometry_available": sum(
                int(row.get("range_evaluable_dof_count", 0)) * SINGLE_SAMPLES
                for row in records
            ),
            "sobol_executable_if_geometry_available": sum(
                SOBOL_SAMPLES
                for row in records
                if int(row.get("movable_dof_count", 0)) > 0
                and int(row.get("range_evaluable_dof_count", 0))
                == int(row.get("movable_dof_count", 0))
            ),
        },
        "claim_boundary": {
            "native_collision_element_total": native_collision_elements,
            "collision_metric_status": BLOCKED_NATIVE_COLLISION_GEOMETRY_ABSENT,
            "collision_queries_executed": False,
            "visual_geometry_fallback": False,
            "vacuous_no_contact_is_not_pass": True,
            "aor": "N/E",
            "max_penetration": "N/E",
            "continuous_collision_detection": "not_run",
            "semantic_joint_correctness": "not_evaluated",
            "physical_dynamics_validity": "not_evaluated",
        },
        "fail_closed_projection": {
            "label": "not a headline metric; hypothetical if blocked states were counted as failures",
            "strict_collision_pass": {"passed": 0, "denominator": n_eval},
            "collision_state_rate": {"collision_states": total_expected, "denominator": total_expected},
            "collision_free_range": {"passed_states": 0, "denominator": single_expected},
        },
        "records_present": len(records),
        "records_missing_counted_as_failures": 0,
        "native_collision_element_total": native_collision_elements,
    }


def audit_asset(
    source_row: dict[str, Any],
    table2_row: dict[str, Any],
    table3_row: dict[str, Any],
    *,
    dataset_root: Path,
) -> dict[str, Any]:
    asset_id = str(source_row["asset_id"])
    package = _contained(Path(str(source_row["package_path"])), dataset_root)
    urdf_relpath = str(source_row["urdf_relpath"])
    urdf = _contained(package / urdf_relpath, package)
    observed_binding = cohort_common.package_binding(package)
    expected_binding = source_row.get("package_binding")
    if observed_binding != expected_binding:
        raise ValueError(f"package binding drift: {asset_id}")
    observed_urdf_sha = sha256_file(urdf)
    if observed_urdf_sha != source_row.get("primary_urdf_sha256"):
        raise ValueError(f"URDF hash drift: {asset_id}")
    if table2_row.get("asset_id") != asset_id or table3_row.get("asset_id") != asset_id:
        raise ValueError(f"upstream order drift: {asset_id}")
    if table2_row.get("expected_package_path") != str(package):
        raise ValueError(f"Table 2 package path drift: {asset_id}")
    if table2_row.get("declared_joint_count_hint") != source_row.get("declared_joint_count_hint"):
        raise ValueError(f"Table 2 joint hint drift: {asset_id}")
    if int(table3_row.get("declared_joint_count", -1)) != int(source_row.get("declared_joint_count_hint", -2)):
        raise ValueError(f"Table 3 joint count drift: {asset_id}")

    root = ET.parse(urdf).getroot()
    joints = parse_joint_specs(urdf)
    table3_joints = table3_row.get("joints", [])
    expected_joint_identity = [
        (str(row["name"]), str(row["type"])) for row in joints
    ]
    observed_joint_identity = [
        (str(row.get("joint_name")), str(row.get("joint_type")))
        for row in table3_joints
    ]
    if expected_joint_identity != observed_joint_identity:
        raise ValueError(f"Table 3 joint identity drift: {asset_id}")
    native_collision_count = len(root.findall(".//collision"))
    links = len(root.findall("link"))
    range_count = sum(bool(row["range_evaluable"]) for row in joints)
    movable_count = len(joints)
    return blocked_record(
        {
            "asset_id": asset_id,
            "asset_key": asset_id,
            # The frozen cohort's selection_index is already 1-based.
            "order": int(source_row["selection_index"]),
            "selection_index": int(source_row["selection_index"]),
            "factory": str(source_row["factory"]),
            "raw_category": str(source_row.get("raw_category", source_row["factory"])),
            "seed": int(source_row["seed"]),
            "source": str(source_row.get("source", "generated")),
            "original_status": str(source_row.get("original_status")),
            "recovery_used": bool(source_row.get("recovery_used")),
            "urdf_relpath": urdf_relpath,
            "urdf_sha256": observed_urdf_sha,
            "package_path": str(package),
            "package_content_manifest_sha256": observed_binding["content_manifest_sha256"],
            "package_file_count": observed_binding["file_count"],
            "package_total_bytes": observed_binding["total_bytes"],
            "link_count": links,
            "joint_specs": joints,
            "movable_dof_count": movable_count,
            "range_evaluable_dof_count": range_count,
            "native_collision_element_count": native_collision_count,
            "valid_tree": valid_rooted_tree(urdf),
            "table2_collision_coverage_pass": bool(
                table2_row.get("metrics", {}).get("collision_coverage", {}).get("pass")
            ),
            "table3_strict_kinematic_pass": bool(table3_row.get("strict_kinematic_pass")),
            "rest_state_expected": 1,
            "single_state_expected": SINGLE_SAMPLES * movable_count,
            "sobol_state_expected": SOBOL_SAMPLES if movable_count > 0 else 0,
        }
    )


def _environment() -> dict[str, Any]:
    return {
        "python": sys.version,
        "python_executable": sys.executable,
        "python_version": platform.python_version(),
        "platform": platform.platform(),
        "runner_path": str(SCRIPT_PATH),
        "runner_sha256": sha256_file(SCRIPT_PATH),
        "thread_environment": {
            key: os.environ.get(key)
            for key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS")
            if key in os.environ
        },
    }


def _load_and_validate_inputs(
    cohort_manifest_path: Path,
    table2_manifest_path: Path,
    table3_receipt: Path,
    *,
    formal: bool,
) -> tuple[dict[str, Any], list[dict[str, Any]], list[dict[str, Any]], list[dict[str, Any]]]:
    cohort_manifest_path = cohort_manifest_path.resolve(strict=True)
    table2_manifest_path = table2_manifest_path.resolve(strict=True)
    table3_receipt = table3_receipt.resolve(strict=True)
    cohort = read_json(cohort_manifest_path)
    table2_manifest = read_json(table2_manifest_path)
    table3_manifest = read_json(table3_receipt / "manifest.json")
    table2_records_path = table2_manifest_path.parent / "records.jsonl"
    table3_records_path = table3_receipt / "asset_records.jsonl"
    table2_records = read_jsonl(table2_records_path)
    table3_records = read_jsonl(table3_records_path)
    if formal:
        expected_files = {
            cohort_manifest_path: FORMAL_COHORT_MANIFEST_SHA256,
            table2_manifest_path: FORMAL_TABLE2_MANIFEST_SHA256,
            table2_records_path: FORMAL_TABLE2_RECORDS_SHA256,
            table3_receipt / "manifest.json": FORMAL_TABLE3_MANIFEST_SHA256,
            table3_records_path: FORMAL_TABLE3_RECORDS_SHA256,
        }
        for path, expected in expected_files.items():
            observed = sha256_file(path)
            if observed != expected:
                raise ValueError(f"formal input hash drift: {path} {observed} != {expected}")
        if cohort.get("manifest_content_sha256") != FORMAL_COHORT_CONTENT_SHA256:
            raise ValueError("formal cohort content hash drift")
        if table2_manifest.get("manifest_content_sha256") != FORMAL_TABLE2_CONTENT_SHA256:
            raise ValueError("formal Table 2 manifest content hash drift")
        if table3_manifest.get("manifest_content_sha256") != FORMAL_TABLE3_CONTENT_SHA256:
            raise ValueError("formal Table 3 manifest content hash drift")
        artifact_manifest = EXP_ROOT / "runtime/infinite_mobility_urdf_table123_cohort/artifact_manifest.json"
        if sha256_file(artifact_manifest) != FORMAL_COHORT_ARTIFACT_MANIFEST_SHA256:
            raise ValueError("formal cohort artifact manifest hash drift")
    assets = cohort.get("assets")
    if not isinstance(assets, list) or len(assets) != len(table2_records) or len(assets) != len(table3_records):
        raise ValueError("upstream record count mismatch")
    if formal and len(assets) != FORMAL_N_EVAL:
        raise ValueError("formal cohort size mismatch")
    if cohort.get("dataset") != DATASET or table2_manifest.get("dataset") != DATASET or table3_manifest.get("dataset") != DATASET:
        raise ValueError("upstream dataset identity mismatch")
    if table3_manifest.get("classification") != CLASSIFICATION:
        raise ValueError("Table 3 receipt is not formal")
    return cohort, assets, table2_records, table3_records


def _compact_items(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    fields = (
        "asset_id", "order", "factory", "seed", "original_status", "recovery_used",
        "package_content_manifest_sha256", "package_file_count", "package_total_bytes",
        "urdf_sha256", "link_count", "movable_dof_count", "range_evaluable_dof_count",
        "native_collision_element_count", "valid_tree", "rest_state_expected",
        "single_state_expected", "sobol_state_expected", "collision_metric_status",
    )
    return [{key: row.get(key) for key in fields} for row in records]


def report_text(summary: dict[str, Any], manifest: dict[str, Any]) -> str:
    plan = summary["state_plan"]
    lines = [
        "# Infinite Mobility Table 4: Collision and Mechanical Clearance",
        "",
        f"Run classification: **{manifest['classification']}**; status: **{summary['status']}**.",
        "",
        f"Frozen cohort: N_eval={summary['n_eval']}, J_eval={summary['j_eval']}; "
        f"rest q=0; single-joint K={SINGLE_SAMPLES}; Sobol R={SOBOL_SAMPLES} "
        f"(seed {SOBOL_SEED}); penetration threshold {PENETRATION_THRESHOLD_M:g} m.",
        "",
        "| Metric | Result |",
        "|---|---:|",
        "| Rest All-pair CF | N/E |",
        "| Rest Non-adjacent CF | N/E |",
        "| Single-joint Sweep CF | N/E |",
        "| Multi-joint Sobol CF | N/E |",
        "| Collision-state Rate | N/E |",
        "| AOR | N/E |",
        "| Max Penetration | N/E |",
        "| Collision-free Range | N/E |",
        "| Strict Collision Pass | N/E |",
        "",
        f"The preflight found {summary['native_collision_element_total']} native URDF collision elements across "
        f"{summary['n_eval']} assets. Collision queries were not executed: "
        f"`{BLOCKED_NATIVE_COLLISION_GEOMETRY_ABSENT}`. Treating empty contact queries as passes would be a "
        "vacuous result, not mechanical-clearance evidence.",
        "",
        f"Intent-to-evaluate state plan: rest {plan['rest_expected']}, single {plan['single_expected']}, "
        f"Sobol {plan['sobol_expected']}, total {plan['total_expected']}. No state receipt is emitted because "
        "the collision oracle is inapplicable. If a future release adds native collision geometry, the full "
        "state schedule must be rerun under a new protocol snapshot.",
        "",
        "AOR and max penetration remain N/E; no visual or bounding-box fallback was used.",
        "",
    ]
    return "\n".join(lines)


def write_artifact_manifest(output: Path) -> dict[str, Any]:
    names = (
        "manifest.json",
        "asset_records.jsonl",
        "state_records.jsonl",
        "summary.json",
        "report.md",
        "protocol_snapshot.md",
        "environment.json",
    )
    payload = {
        "schema_version": 1,
        "files": {
            name: {"bytes": (output / name).stat().st_size, "sha256": sha256_file(output / name)}
            for name in names
        },
    }
    atomic_json(output / "artifact_manifest.json", payload)
    return payload


def _verify_artifact_manifest(output: Path) -> None:
    manifest = read_json(output / "artifact_manifest.json")
    files = manifest.get("files")
    if not isinstance(files, dict):
        raise ValueError("artifact manifest files missing")
    expected = set(files) | {"artifact_manifest.json"}
    if (output / "verification.json").is_file():
        expected.add("verification.json")
    observed = {path.name for path in output.iterdir() if path.is_file()}
    if observed != expected:
        raise ValueError(f"artifact set drift: {sorted(observed)} != {sorted(expected)}")
    for name, entry in files.items():
        path = output / name
        if path.stat().st_size != int(entry["bytes"]) or sha256_file(path) != entry["sha256"]:
            raise ValueError(f"artifact hash drift: {name}")


def verify_output(output: Path) -> dict[str, Any]:
    output = output.resolve(strict=True)
    manifest = read_json(output / "manifest.json")
    summary = read_json(output / "summary.json")
    records = read_jsonl(output / "asset_records.jsonl")
    state_lines = (output / "state_records.jsonl").read_text(encoding="utf-8")
    checks: dict[str, bool] = {}
    checks["manifest_self_hash"] = manifest.get("manifest_content_sha256") == manifest_content_hash(manifest)
    checks["formal_status_blocked"] = manifest.get("classification") == CLASSIFICATION and summary.get("status") == "BLOCKED"
    checks["record_count"] = len(records) == FORMAL_N_EVAL
    checks["record_order"] = [row.get("order") for row in records] == list(range(1, FORMAL_N_EVAL + 1))
    checks["all_collision_counts_zero"] = sum(int(row.get("native_collision_element_count", -1)) for row in records) == 0
    checks["all_records_blocked"] = all(
        row.get("collision_metric_status") == BLOCKED_NATIVE_COLLISION_GEOMETRY_ABSENT
        and row.get("strict_collision_pass") is None
        and not row.get("state_records")
        for row in records
    )
    checks["state_records_empty"] = state_lines == ""
    # Recompute while preserving the run's completion timestamp and manifest binding.
    recomputed = summarize_blocked(
        records, n_eval=len(records), j_eval=sum(int(row["movable_dof_count"]) for row in records)
    )
    checks["summary_recomputes"] = True
    for key in ("schema_version", "status", "classification", "dataset", "protocol_id", "n_eval", "j_eval", "zero_joint_assets", "metrics", "state_plan", "claim_boundary", "fail_closed_projection", "records_present", "records_missing_counted_as_failures", "native_collision_element_total"):
        if summary.get(key) != recomputed.get(key):
            checks["summary_recomputes"] = False
            break
    checks["report_recomputes"] = (output / "report.md").read_text(encoding="utf-8") == report_text(summary, manifest)
    checks["artifact_manifest"] = True
    try:
        _verify_artifact_manifest(output)
    except Exception:
        checks["artifact_manifest"] = False
    receipt = {
        "schema_version": 1,
        "protocol_id": f"{PROTOCOL_ID}:verify",
        "status": "PASS" if all(checks.values()) else "FAIL",
        "checked_at": utc_now(),
        "checks": checks,
        "artifact_sha256": {
            name: sha256_file(output / name)
            for name in (
                "manifest.json", "asset_records.jsonl", "state_records.jsonl", "summary.json",
                "report.md", "protocol_snapshot.md", "environment.json", "artifact_manifest.json",
            )
        },
    }
    atomic_json(output / "verification.json", receipt)
    if receipt["status"] != "PASS":
        raise RuntimeError(f"verification failed: {checks}")
    return receipt


def run(
    *,
    cohort_manifest: Path = DEFAULT_COHORT_MANIFEST,
    table2_manifest: Path = DEFAULT_TABLE2_MANIFEST,
    table3_receipt: Path = DEFAULT_TABLE3_RECEIPT,
    output: Path = DEFAULT_OUTPUT,
    workers: int = 4,
    formal: bool = True,
) -> dict[str, Any]:
    if workers <= 0:
        raise ValueError("workers must be positive")
    output = output.resolve()
    if output.exists():
        raise FileExistsError(f"refusing to overwrite existing output: {output}")
    cohort, assets, table2_records, table3_records = _load_and_validate_inputs(
        cohort_manifest, table2_manifest, table3_receipt, formal=formal
    )
    output.mkdir(parents=True, exist_ok=False)
    protocol_snapshot = output / PROTOCOL_SNAPSHOT_NAME
    protocol_snapshot.write_bytes(PROTOCOL_DOCUMENT.read_bytes())
    environment = _environment()
    protocol_sha = sha256_file(protocol_snapshot)
    manifest_stub = {
        "schema_version": SCHEMA_VERSION,
        "dataset": DATASET,
        "classification": CLASSIFICATION if formal else "NON_FORMAL_SMOKE",
        "status": "BLOCKED",
        "protocol_id": PROTOCOL_ID,
        "created_at": utc_now(),
        "cohort_type": cohort.get("cohort_type"),
        "source": {
            "cohort_manifest": str(Path(cohort_manifest).resolve(strict=True)),
            "cohort_manifest_sha256": sha256_file(Path(cohort_manifest).resolve(strict=True)),
            "cohort_manifest_content_sha256": cohort.get("manifest_content_sha256"),
            "cohort_artifact_manifest_sha256": FORMAL_COHORT_ARTIFACT_MANIFEST_SHA256,
            "table2_manifest": str(Path(table2_manifest).resolve(strict=True)),
            "table2_manifest_sha256": sha256_file(Path(table2_manifest).resolve(strict=True)),
            "table2_records_sha256": sha256_file(Path(table2_manifest).resolve(strict=True).parent / "records.jsonl"),
            "table3_receipt": str(Path(table3_receipt).resolve(strict=True)),
            "table3_manifest_sha256": sha256_file(Path(table3_receipt).resolve(strict=True) / "manifest.json"),
            "table3_records_sha256": sha256_file(Path(table3_receipt).resolve(strict=True) / "asset_records.jsonl"),
            "n_release_claim": "not finite; this is the frozen 720-case operational cohort",
        },
        "evaluation": {
            "protocol_snapshot": str(protocol_snapshot),
            "protocol_snapshot_sha256": protocol_sha,
            "state_policy": {
                "rest": "q=0",
                "single_joint_samples": SINGLE_SAMPLES,
                "sobol_samples": SOBOL_SAMPLES,
                "sobol_seed": SOBOL_SEED,
            },
            "penetration_threshold_m": PENETRATION_THRESHOLD_M,
            "pair_policy": "all-pair and non-adjacent; direct parent-child excluded for headline",
            "geometry_policy": "native collision geometry only; no visual fallback",
            "scale_protocol": "pybullet_q0_collision_shape_union_aabb_v1",
            "runner_path": str(SCRIPT_PATH),
            "runner_sha256": sha256_file(SCRIPT_PATH),
            "workers": workers,
        },
        "claim_boundary": {
            "native_collision_geometry_required": True,
            "zero_native_collision_is_blocking": True,
            "blocked_reason": BLOCKED_NATIVE_COLLISION_GEOMETRY_ABSENT,
            "vacuous_no_contact_is_not_pass": True,
        },
        "environment": environment,
    }
    manifest_stub["manifest_content_sha256"] = manifest_content_hash(manifest_stub)
    atomic_json(output / "manifest.json", manifest_stub)
    atomic_json(output / "environment.json", environment)

    records: list[dict[str, Any]] = [None] * len(assets)  # type: ignore[list-item]
    with ThreadPoolExecutor(max_workers=workers) as executor:
        futures = {
            executor.submit(
                audit_asset,
                asset,
                table2_row,
                table3_row,
                dataset_root=Path(asset["package_path"]).parents[4],
            ): index
            for index, (asset, table2_row, table3_row) in enumerate(
                zip(assets, table2_records, table3_records, strict=True)
            )
        }
        for completed, future in enumerate(as_completed(futures), 1):
            index = futures[future]
            records[index] = future.result()
            print(f"table4 audit {completed}/{len(assets)} {records[index]['asset_id']}", flush=True)
    if any(row is None for row in records):
        raise RuntimeError("missing audit record")
    records = [row for row in records if row is not None]
    if formal:
        if sum(int(row["movable_dof_count"]) for row in records) != FORMAL_J_EVAL:
            raise ValueError("formal joint denominator mismatch")
        if sum(int(row["movable_dof_count"]) == 0 for row in records) != FORMAL_ZERO_JOINT_ASSETS:
            raise ValueError("formal zero-joint asset denominator mismatch")
        if sum(int(row["range_evaluable_dof_count"]) for row in records) != FORMAL_RANGE_EVALUABLE_JOINTS:
            raise ValueError("formal range-evaluable joint denominator mismatch")
        if sum(
            int(row["movable_dof_count"]) > 0
            and int(row["range_evaluable_dof_count"]) == int(row["movable_dof_count"])
            for row in records
        ) != FORMAL_ALL_RANGE_MOVABLE_ASSETS:
            raise ValueError("formal all-range movable asset denominator mismatch")
        if sum(int(row["native_collision_element_count"]) for row in records) != 0:
            raise ValueError("Infinite Mobility collision preflight is not blocked-safe")
    summary = summarize_blocked(
        records,
        n_eval=len(records),
        j_eval=sum(int(row["movable_dof_count"]) for row in records),
    )
    summary["manifest_content_sha256"] = manifest_stub["manifest_content_sha256"]
    summary["completed_at"] = utc_now()
    atomic_json(output / "summary.json", summary)
    # Keep the canonical JSONL spelling used by the other Table 4 runners.
    atomic_text(
        output / "asset_records.jsonl",
        "".join(json.dumps(row, sort_keys=True, ensure_ascii=True) + "\n" for row in records),
    )
    atomic_text(output / "state_records.jsonl", "")
    atomic_text(output / "report.md", report_text(summary, manifest_stub))
    write_artifact_manifest(output)
    verify_output(output)
    return summary


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cohort-manifest", type=Path, default=DEFAULT_COHORT_MANIFEST)
    parser.add_argument("--table2-manifest", type=Path, default=DEFAULT_TABLE2_MANIFEST)
    parser.add_argument("--table3-receipt", type=Path, default=DEFAULT_TABLE3_RECEIPT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--non-formal", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    summary = run(
        cohort_manifest=args.cohort_manifest,
        table2_manifest=args.table2_manifest,
        table3_receipt=args.table3_receipt,
        output=args.output,
        workers=args.workers,
        formal=not args.non_formal,
    )
    print(
        json.dumps(
            {
                "status": summary["status"],
                "dataset": DATASET,
                "n_eval": summary["n_eval"],
                "j_eval": summary["j_eval"],
                "native_collision_element_total": summary["native_collision_element_total"],
                "output": str(args.output.resolve()),
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
