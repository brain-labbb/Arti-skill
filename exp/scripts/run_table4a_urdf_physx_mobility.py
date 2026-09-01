#!/usr/bin/env python3
"""Fail-closed Table 4a runner for PhysX-Mobility (Genesis contact-penetration oracle).

Runs the proposed Table 4a DoF-aware Mechanical Safety metrics over the frozen
Table 5 PhysX-Mobility N=800 receipt-set cohort, reusing the version-pinned
``GenesisTable4aAdapter`` from the LAM supplementary runner so that the engine
protocol (``genesis_contact_penetration_v1``) is identical across methods.

Scope: Table 4a only (joint_full_range states; K=21 frozen Table 4 sweep
states per movable joint, other joints at historical q=0). Sobol/rest strict
states are not re-executed; the existing Table 4 ``Strict Collision Pass`` is
reported per DoF bin from the frozen PhysX Table 4 asset records.

Normalized Clearance P5 is N/E under this oracle: the frozen adapter reports
no signed clearance for separated pairs and this run registers no independent
exact-distance backend.

Claim boundary: official PhysX-Mobility URDFs declare zero collision elements,
so every asset maps to zero eligible collision pairs and all collision-free
outcomes are vacuous (consistent with the frozen Table 5 receipt set's
``strict_collision: N/E`` adjudication and the Table 4 run's claim boundary).
"""

from __future__ import annotations

import argparse
import datetime as _dt
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import shutil
import signal
import statistics
import subprocess
import sys
import time
from typing import Any, Mapping, Sequence
import xml.etree.ElementTree as ET

SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from exp.scripts import run_urdf_lam_supplementary_v1 as lam4a  # noqa: E402

SCHEMA_VERSION = "table4a-physx-mobility/v1"
PROTOCOL_ID = "table4a_physx_mobility_table5cohort_n800_v1"
ENGINE_PROTOCOL_ID = lam4a.ENGINE_PROTOCOL_ID
DATASET = "PhysX-Mobility"
CLASSIFICATION = "FORMAL"

DEFAULT_DATASET_ROOT = REPO / "exp/PhysX-Mobility/extracted/PhysX_mobility"
DEFAULT_RECEIPT_SET = REPO / "exp/runtime/table5_physx_mobility_n800_v2"
TABLE3_RECORDS = REPO / "exp/runtime/urdf_table3_physx_mobility_table5cohort_n800_20260819T102939Z/asset_records.jsonl"
TABLE4_ASSET_RECORDS = REPO / "exp/runtime/urdf_table4_physx_mobility_table5cohort_n800_20260819T143442Z/asset_records.json"
TABLE4_STATE_RECORDS = REPO / "exp/runtime/urdf_table4_physx_mobility_table5cohort_n800_20260819T143442Z/state_records.jsonl"
PROTOCOL_DOCUMENT = REPO / "exp/URDF-Sim-Ready-Automatic-Evaluation.md"

N_EVAL = 800
J_EVAL = 3809
SINGLE_SAMPLES = 21
ZERO_WIDTH_TOLERANCE = 1e-12
CONTINUOUS_INTERVAL = (-math.pi, math.pi)
CHILD_TIMEOUT_SECONDS = 3600
WORKERS = 16
LAUNCH_STAGGER_SECONDS = 1.5
DOF_BINS: list[tuple[str, int, int]] = [
    ("0", 0, 0),
    ("1", 1, 1),
    ("2--3", 2, 3),
    ("4--7", 4, 7),
    (">=8", 8, 10**9),
]

METRIC_RULES = {
    "joint_level_full_range_cf": (
        "关节级 passed / J_eval。对每个 movable joint 使用既有 Table 4 的冻结测试区间和 K = 21 "
        "状态，其他关节保持历史 q = 0；全部 intended states 均须成功执行，并在统一、无 "
        "method-specific allowance 的 headline pair policy 与穿透阈值下无非法碰撞。任何未执行状态、"
        "无效区间、加载失败或资源失败均使该关节 fail closed。该离散指标不证明连续配置空间无碰撞。"
    ),
    "executable_cf_dof_per_asset": (
        "每个资产中同时通过 Table 3 Joint-level Pass 和本表 Joint-level Full-range CF 的关节数；"
        "跨 N_eval 报告 mean / median / P90。解析、加载或测量失败资产的安全可执行 DoF 计为 0，"
        "不得从资产分母删除。"
    ),
    "collision_safe_dof_retention": (
        "上述安全可执行关节总数除以冻结 J_eval，写为 passed / J_eval (percentage)。它是与 raw DoF "
        "数配套的保留率，不得以只统计成功加载资产的条件分母替代。"
    ),
    "normalized_clearance_p5": (
        "对每个实际测得的 intended state，取 headline pair policy 下所有 eligible non-adjacent "
        "collision-surface pair 的最小 signed clearance，再除以统一 D_visual；负值表示穿透，零表示接触。"
        "先对每个资产取 state-level minimum 的 P5，数据集单元格报告这些 asset-level P5 的中位数，"
        "并同时写出 measured / intended state 和 asset coverage，以及 COMPLETE 或 PARTIAL。未测状态"
        "不伪造距离，但对应 pass metrics 仍 fail closed。"
    ),
    "limit_reachability": (
        "bounded joint 的 lower 和 upper endpoint 都可执行、transform 有限且在 headline pair policy "
        "下无非法碰撞时通过；报告 passed / J_bounded (percentage)。continuous joint 不进入 J_bounded，"
        "其分母必须显式报告；endpoint 未执行或不可加载计为失败。"
    ),
}

OPERATIONALIZATION = {
    "engine": (
        "Genesis contact-penetration backend, engine protocol genesis_contact_penetration_v1, "
        "version-pinned by the frozen lam_supplementary GenesisTable4aAdapter: Genesis "
        f"{lam4a.GENESIS_VERSION}, trimesh {lam4a.TRIMESH_VERSION}, rtree {lam4a.RTREE_VERSION}, "
        "CPU backend, precision 64, one asset per fresh Genesis process."
    ),
    "state_detection": (
        "每个冻结 q-state 只做 direct kinematic detection（set_dofs_position + detect_collision + "
        "get_contacts，不 scene.step）；q readback 必须在 1e-9 容差内。"
    ),
    "illegal_rule": (
        "仅当 eligible source-URDF link pair 的 Genesis-reported penetration 严格大于 1e-6 m 时判为非法；"
        "表面接触允许。"
    ),
    "pair_policy": (
        "Headline pair policy = eligible geom pairs on distinct source links excluding the URDF "
        "direct parent-child graph (exclude_direct_parent_child); self / neutral / adjacent "
        "collision candidate generation enabled; Genesis default filtering or visual fallback 不得改变 "
        "pair policy; eligible pairs must be present in the Genesis valid-pair table or the asset "
        "fails closed."
    ),
    "state_plan": (
        "严格沿用既有 PhysX Table 4 的冻结单关节 sweep：每关节 K=21 个状态，取值 lower + i*(upper-lower)/20，"
        "i=0..20（含两端 endpoint）；bounded joint 使用声明 lower/upper（宽度须 > 1e-12），continuous "
        "joint 使用冻结区间 [-pi, pi]；其余关节保持 q=0。每个状态的全 DoF 向量先按 Genesis 内部 DoF 顺序读出，"
        "再重排为冻结 manifest 关节顺序后取 canonical SHA256，与既有 PhysX Table 4 state_records.joint_values_sha256 "
        "逐项核对；PhysX Table 4 中未执行的 single-joint 状态（12 个 getJointState 失败资产）无参照，"
        "仅按冻结规则重生成并计入 no_reference。"
    ),
    "limit_reachability_endpoints": (
        "endpoint = 同一 K=21 sweep 的 sample_index 0（lower）与 20（upper）；'可执行、transform 有限' "
        "operationalize 为该状态成功执行、q readback 有限且观测 status COMPLETE（Genesis FK/接触管线有限）。"
    ),
    "j_bounded": "revolute/prismatic joints with finite declared lower < upper; continuous joints excluded and counted explicitly.",
    "normalized_clearance_p5": (
        "N/E under this oracle: the frozen adapter sets clearance_status N/E because Genesis contact "
        "penetration has no signed clearance for separated pairs; no independent exact-distance "
        "backend is registered for this run. measured/intended state coverage is still reported."
    ),
    "cohort_staging": (
        "官方发布几何中资源位于扁平 urdf/ 目录的 sibling 目录 partseg/；每个资产先被 staging 为复刻发布几何的"
        "自包含 package（逐字节哈希校验，绑定冻结 Table 5 manifest 行与官方归档字节级绑定），Genesis 从 staging "
        "package 加载；审计前后重算 package binding。"
    ),
    "claim_boundary": (
        "官方 PhysX-Mobility URDF 未声明任何 collision 元素（冻结 Table 5 manifest xml_counts.collision_elements "
        "总和为 0）：每个资产的 eligible collision pair 数为 0，全部 collision-free 结果均为空判定（vacuous），"
        "不得解读为已验证的机械间隙；Table 5 receipt set 对该方法预注册 strict_collision: N/E"
        "（reason official_urdf_zero_collision_elements），Table 4 运行已记录同一 claim boundary。"
    ),
    "table3_joint_pass_source": "PhysX Table 3 asset_records joints[].joint_level_pass (frozen run urdf_table3_physx_mobility_table5cohort_n800_20260819T102939Z).",
    "existing_strict_collision_pass_source": "PhysX Table 4 asset_records strict_collision_pass (frozen run urdf_table4_physx_mobility_table5cohort_n800_20260819T143442Z; PyBullet result, reported per DoF bin only).",
    "percentile_policy": "median/P90 of per-asset safe-DoF counts with linear interpolation (numpy-style), over all 800 assets including zero-DoF failed assets.",
}


def utc_now_iso() -> str:
    return _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")


def sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def atomic_write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    with tmp.open("w", encoding="utf-8") as handle:
        handle.write(text)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(tmp, path)


def atomic_write_json(path: Path, value: Any) -> None:
    atomic_write_text(
        path,
        json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True, allow_nan=False) + "\n",
    )


def _load_module(path: Path, name: str):
    existing = sys.modules.get(name)
    if existing is not None:
        return existing
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load required module: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


TABLE1P = _load_module(
    REPO / "exp/scripts/run_table1_physx_mobility.py",
    "run_table4a_physx_table1_cohort_shared",
)


def joint_interval(row: Mapping[str, Any]) -> tuple[float, float]:
    if row["type"] == "continuous":
        return CONTINUOUS_INTERVAL
    lower = row.get("lower")
    upper = row.get("upper")
    if not isinstance(lower, (int, float)) or not isinstance(upper, (int, float)):
        raise ValueError(f"joint {row.get('name', '<unnamed>')} has no finite range")
    lower_value = float(lower)
    upper_value = float(upper)
    if (
        not math.isfinite(lower_value)
        or not math.isfinite(upper_value)
        or upper_value - lower_value <= ZERO_WIDTH_TOLERANCE
    ):
        raise ValueError(f"joint {row.get('name', '<unnamed>')} has invalid range")
    return lower_value, upper_value


def single_joint_values(row: Mapping[str, Any]) -> list[float]:
    lower, upper = joint_interval(row)
    return [
        lower + index * (upper - lower) / (SINGLE_SAMPLES - 1)
        for index in range(SINGLE_SAMPLES)
    ]


def parse_urdf_joints(urdf_path: Path) -> list[dict[str, Any]]:
    root = ET.parse(urdf_path).getroot()
    if root.tag != "robot":
        raise ValueError(f"URDF root is {root.tag!r}, expected 'robot'")
    rows: list[dict[str, Any]] = []
    for xml_index, node in enumerate(root.findall("joint")):
        joint_type = str(node.get("type", ""))
        if joint_type == "fixed":
            continue
        limit = node.find("limit")
        lower: float | None = None
        upper: float | None = None
        if joint_type in {"revolute", "prismatic"} and limit is not None:
            try:
                lower = float(limit.get("lower", ""))
                upper = float(limit.get("upper", ""))
            except ValueError:
                lower = upper = None
        row = {
            "xml_index": xml_index,
            "name": node.get("name", f"joint_{xml_index}"),
            "type": joint_type,
            "lower": lower,
            "upper": upper,
        }
        try:
            joint_interval(row)
            row["range_evaluable"] = True
        except ValueError:
            row["range_evaluable"] = False
        rows.append(row)
    return rows


def package_binding(package: Path) -> dict[str, Any]:
    rows: list[dict[str, Any]] = []
    for current_raw, directory_names, file_names in os.walk(package, followlinks=False):
        current = Path(current_raw)
        directory_names.sort()
        file_names.sort()
        for name in file_names:
            path = current / name
            if path.is_symlink():
                raise ValueError(f"package contains symlink: {path.relative_to(package)}")
            rows.append(
                {
                    "path": path.relative_to(package).as_posix(),
                    "bytes": path.stat().st_size,
                    "sha256": lam4a.sha256_file(path),
                }
            )
    rows.sort(key=lambda item: item["path"])
    return {
        "file_count": len(rows),
        "total_bytes": sum(item["bytes"] for item in rows),
        "content_manifest_sha256": lam4a.canonical_sha256(rows),
    }


def stage_package(dataset_root: Path, row: Mapping[str, Any], staging_root: Path) -> dict[str, Any]:
    dataset_root = dataset_root.resolve()
    dataset_id = int(row["dataset_id"])
    package = staging_root / str(dataset_id)
    items = [(str(row["urdf_relative_path"]), str(row["urdf_sha256"]))]
    items.extend(
        (str(resource["relative_path"]), str(resource["sha256"]))
        for resource in row["resources"]
    )
    seen: set[str] = set()
    for relative, expected_sha256 in items:
        if relative in seen:
            raise ValueError(f"duplicate resource binding: {relative}")
        seen.add(relative)
        source = dataset_root / relative
        target = package / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)
        if lam4a.sha256_file(target) != expected_sha256:
            raise ValueError(f"staged byte mismatch: {dataset_id}: {relative}")
    return {"package": package, "package_binding": package_binding(package)}


def load_table3_joint_pass() -> tuple[dict[str, dict[str, bool]], int]:
    result: dict[str, dict[str, bool]] = {}
    joints_total = 0
    with TABLE3_RECORDS.open(encoding="utf-8") as handle:
        for line in handle:
            record = json.loads(line)
            dataset_id = str(record["asset_key"])
            passes: dict[str, bool] = {}
            for joint in record.get("joints") or []:
                passes[str(joint["joint_name"])] = bool(joint["joint_level_pass"])
                joints_total += 1
            result[dataset_id] = passes
    return result, joints_total


def load_table4_strict_pass() -> dict[str, bool]:
    data = json.loads(TABLE4_ASSET_RECORDS.read_text(encoding="utf-8"))
    return {str(r["dataset_id"]): bool(r["strict_collision_pass"]) for r in data}


def load_table4_state_hashes() -> dict[tuple[str, str, int], str]:
    index: dict[tuple[str, str, int], str] = {}
    with TABLE4_STATE_RECORDS.open(encoding="utf-8") as handle:
        for line in handle:
            record = json.loads(line)
            if record.get("phase") != "single_joint_sweep":
                continue
            index[
                (str(record["dataset_id"]), str(record["joint_name"]), int(record["sample_index"]))
            ] = str(record["joint_values_sha256"])
    return index


def build_jobs(
    rows: Sequence[Mapping[str, Any]],
    staged: Mapping[int, Mapping[str, Any]],
    table3_pass: Mapping[str, Mapping[str, bool]],
    state_hashes: Mapping[tuple[str, str, int], str],
) -> list[dict[str, Any]]:
    jobs: list[dict[str, Any]] = []
    for order, row in enumerate(rows):
        dataset_id = int(row["dataset_id"])
        entry = staged[dataset_id]
        package = Path(str(entry["package"]))
        urdf_path = package / str(row["urdf_relative_path"])
        joints = sorted(parse_urdf_joints(urdf_path), key=lambda item: int(item["xml_index"]))
        joint_jobs = []
        for position, joint_row in enumerate(joints):
            values = single_joint_values(joint_row) if joint_row["range_evaluable"] else []
            references = []
            for sample_index in range(SINGLE_SAMPLES):
                references.append(
                    state_hashes.get((str(dataset_id), str(joint_row["name"]), sample_index))
                )
            joint_jobs.append(
                {
                    "name": str(joint_row["name"]),
                    "type": str(joint_row["type"]),
                    "lower": joint_row.get("lower"),
                    "upper": joint_row.get("upper"),
                    "xml_index": int(joint_row["xml_index"]),
                    "dof_position": position,
                    "range_evaluable": bool(joint_row["range_evaluable"]),
                    "values": values,
                    "state_hash_references": references,
                    "table3_joint_level_pass": bool(
                        table3_pass.get(str(dataset_id), {}).get(str(joint_row["name"]), False)
                    ),
                }
            )
        jobs.append(
            {
                "selection_index": order,
                "dataset_id": str(dataset_id),
                "rank": int(row["rank"]),
                "category": str(row["category"]),
                "package": str(package),
                "package_binding": entry["package_binding"],
                "urdf_path": str(urdf_path),
                "expected_urdf_sha256": str(row["urdf_sha256"]),
                "manifest_row_sha256": TABLE1P.TABLE5.canonical_sha256(dict(row)),
                "input_identity_sha256": TABLE1P.TABLE5.canonical_sha256(dict(row)),
                "expected_movable_dof": len(joint_jobs),
                "expected_state_count": SINGLE_SAMPLES * len(joint_jobs),
                "joints": joint_jobs,
            }
        )
    return jobs


def _failed_asset_record(job: Mapping[str, Any], issue: str) -> dict[str, Any]:
    joints = []
    for joint in job["joints"]:
        joints.append(
            {
                "joint_name": joint["name"],
                "joint_type": joint["type"],
                "dof_position": int(joint["dof_position"]),
                "xml_index": int(joint["xml_index"]),
                "states_intended": SINGLE_SAMPLES,
                "states_executed": 0,
                "illegal_states": 0,
                "full_range_cf_pass": False,
                "limit_endpoints_intended": 2 if joint["type"] != "continuous" else 0,
                "limit_endpoints_executed": 0,
                "limit_reachable": False,
                "table3_joint_level_pass": bool(joint["table3_joint_level_pass"]),
                "safe_dof": 0,
                "issues": [issue],
            }
        )
    return {
        "schema_version": SCHEMA_VERSION,
        "protocol_id": PROTOCOL_ID,
        "engine_protocol_id": ENGINE_PROTOCOL_ID,
        "selection_index": int(job["selection_index"]),
        "dataset_id": str(job["dataset_id"]),
        "category": str(job["category"]),
        "package": str(job["package"]),
        "urdf_sha256": None,
        "expected_urdf_sha256": str(job["expected_urdf_sha256"]),
        "expected_movable_dof": int(job["expected_movable_dof"]),
        "status": "error",
        "load_success": False,
        "mapping_status": "NOT_EVALUABLE",
        "eligible_pair_count": None,
        "states_intended": int(job["expected_state_count"]),
        "states_executed": 0,
        "state_hash_cross_check": {"verified": 0, "mismatch": 0, "no_reference": 0},
        "joint_records": joints,
        "issues": [issue],
    }


def run_child(job_path: Path, result_path: Path) -> int:
    job = json.loads(job_path.read_text(encoding="utf-8"))
    cache_path = Path(str(job["genesis_cache_path"]))
    template_raw = job.get("template_cache_path")
    if template_raw:
        template_path = Path(str(template_raw))
        if template_path.is_dir() and not cache_path.exists():
            shutil.copytree(template_path, cache_path)
    cache_path.mkdir(parents=True, exist_ok=True)
    record: dict[str, Any]
    try:
        package = Path(str(job["package"]))
        urdf_path = Path(str(job["urdf_path"]))
        binding_before = package_binding(package)
        if binding_before != job["package_binding"]:
            raise lam4a.GenesisAdapterError("package changed before evaluation")
        urdf_sha256 = lam4a.sha256_file(urdf_path)
        if urdf_sha256 != str(job["expected_urdf_sha256"]):
            raise lam4a.GenesisAdapterError(
                f"urdf_sha256_mismatch: got {urdf_sha256}, expected {job['expected_urdf_sha256']}"
            )
        runtime = lam4a.genesis_runtime_binding(expected_cache_path=cache_path)
        adapter = lam4a.GenesisTable4aAdapter(urdf_path, runtime)
        adapter.build()
        if int(adapter.entity.n_dofs) != int(job["expected_movable_dof"]):
            raise lam4a.GenesisAdapterError(
                f"genesis_dof_count_mismatch: genesis={int(adapter.entity.n_dofs)}, "
                f"frozen={int(job['expected_movable_dof'])}"
            )
        mapping_status = str(adapter.mapping.get("status"))
        if mapping_status != "COMPLETE":
            raise lam4a.GenesisAdapterError(f"pair_mapping_incomplete: {adapter.mapping}")
        genesis_index_by_name = {
            str(entry["joint_name"]): int(entry["dof_index"]) for entry in adapter.dof_order
        }
        manifest_joint_order = [str(joint["name"]) for joint in job["joints"]]
        for name in manifest_joint_order:
            if name not in genesis_index_by_name:
                raise lam4a.GenesisAdapterError(f"manifest joint {name!r} missing from Genesis DoF map")
        synthetic_item = {
            "asset_key": str(job["dataset_id"]),
            "selection_rank": int(job["selection_index"]) + 1,
            "input_identity_sha256": str(job["input_identity_sha256"]),
        }
        joint_records = []
        states_executed = 0
        cross = {"verified": 0, "mismatch": 0, "no_reference": 0}
        asset_failed = False
        failure_reason: str | None = None
        for joint in job["joints"]:
            name = str(joint["name"])
            executed = 0
            illegal = 0
            endpoint_ok = {0: False, SINGLE_SAMPLES - 1: False}
            state_summaries = []
            if not joint["range_evaluable"]:
                asset_failed = True
                if failure_reason is None:
                    failure_reason = f"joint_range_not_evaluable: {name}"
                joint_records.append(
                    {
                        "joint_name": name,
                        "joint_type": joint["type"],
                        "dof_position": int(joint["dof_position"]),
                        "xml_index": int(joint["xml_index"]),
                        "states_intended": SINGLE_SAMPLES,
                        "states_executed": 0,
                        "illegal_states": 0,
                        "full_range_cf_pass": False,
                        "limit_endpoints_intended": 2 if joint["type"] != "continuous" else 0,
                        "limit_endpoints_executed": 0,
                        "limit_reachable": False,
                        "table3_joint_level_pass": bool(joint["table3_joint_level_pass"]),
                        "safe_dof": 0,
                        "issues": [f"joint_range_not_evaluable: {name}"],
                        "state_summaries": [
                            {"sample_index": i, "executed": False,
                             "issue": f"joint_range_not_evaluable: {name}"}
                            for i in range(SINGLE_SAMPLES)
                        ],
                    }
                )
                continue
            for sample_index, value in enumerate(joint["values"]):
                if asset_failed:
                    state_summaries.append(
                        {"sample_index": sample_index, "executed": False,
                         "issue": f"asset_fail_closed: {failure_reason}"}
                    )
                    continue
                try:
                    engine_record = adapter.state(
                        item=synthetic_item,
                        joint_name=name,
                        sample_index=sample_index,
                        value=float(value),
                    )
                except Exception as exc:  # noqa: BLE001
                    asset_failed = True
                    failure_reason = f"state_execution_failed: {type(exc).__name__}: {exc}"
                    state_summaries.append(
                        {"sample_index": sample_index, "executed": False, "issue": failure_reason}
                    )
                    continue
                intended_values = list(engine_record["q_intended_values"])
                engine_hash = lam4a.canonical_sha256(intended_values)
                canonical_vector = [
                    float(intended_values[genesis_index_by_name[manifest_name]])
                    for manifest_name in manifest_joint_order
                ]
                intended_hash = lam4a.canonical_sha256(canonical_vector)
                reference = joint["state_hash_references"][sample_index]
                if reference is None:
                    cross["no_reference"] += 1
                elif intended_hash == reference:
                    cross["verified"] += 1
                else:
                    cross["mismatch"] += 1
                    asset_failed = True
                    failure_reason = (
                        f"state_identity_mismatch: {name}#{sample_index} "
                        f"got {intended_hash}, expected {reference}"
                    )
                illegal_here = bool(engine_record.get("illegal_collision"))
                executed += 1
                illegal += int(illegal_here)
                if sample_index in endpoint_ok:
                    endpoint_ok[sample_index] = (
                        engine_record.get("executed") is True
                        and engine_record.get("observation_status") == "COMPLETE"
                        and engine_record.get("readback", {}).get("finite") is True
                    )
                state_summaries.append(
                    {
                        "sample_index": sample_index,
                        "executed": True,
                        "illegal_collision": illegal_here,
                        "max_eligible_penetration_m": engine_record.get("max_eligible_penetration_m"),
                        "q_readback_max_abs_error": engine_record.get("q_readback_max_abs_error"),
                        "raw_contact_count": engine_record.get("raw_contact_count"),
                        "eligible_contact_count": engine_record.get("eligible_contact_count"),
                        "excluded_direct_parent_child_contact_count": engine_record.get(
                            "excluded_direct_parent_child_contact_count"
                        ),
                        "q_intended_values_sha256": intended_hash,
                        "q_engine_order_values_sha256": engine_hash,
                    }
                )
            states_executed += executed
            full_range_pass = (
                executed == SINGLE_SAMPLES and illegal == 0 and not asset_failed
            )
            is_bounded = joint["type"] != "continuous"
            limit_reachable = bool(
                is_bounded
                and full_range_pass
                and endpoint_ok[0]
                and endpoint_ok[SINGLE_SAMPLES - 1]
            )
            joint_records.append(
                {
                    "joint_name": name,
                    "joint_type": joint["type"],
                    "dof_position": int(joint["dof_position"]),
                    "xml_index": int(joint["xml_index"]),
                    "states_intended": SINGLE_SAMPLES,
                    "states_executed": executed,
                    "illegal_states": illegal,
                    "full_range_cf_pass": bool(full_range_pass),
                    "limit_endpoints_intended": 2 if is_bounded else 0,
                    "limit_endpoints_executed": (
                        int(endpoint_ok[0]) + int(endpoint_ok[SINGLE_SAMPLES - 1]) if is_bounded else 0
                    ),
                    "limit_reachable": limit_reachable,
                    "table3_joint_level_pass": bool(joint["table3_joint_level_pass"]),
                    "safe_dof": int(bool(full_range_pass) and bool(joint["table3_joint_level_pass"])),
                    "issues": [],
                    "state_summaries": state_summaries,
                }
            )
        try:
            binding_after = package_binding(package)
        except Exception:  # noqa: BLE001
            binding_after = None
        if binding_after != job["package_binding"]:
            raise lam4a.GenesisAdapterError("package changed during evaluation")
        record = {
            "schema_version": SCHEMA_VERSION,
            "protocol_id": PROTOCOL_ID,
            "engine_protocol_id": ENGINE_PROTOCOL_ID,
            "selection_index": int(job["selection_index"]),
            "dataset_id": str(job["dataset_id"]),
            "category": str(job["category"]),
            "package": str(job["package"]),
            "urdf_sha256": urdf_sha256,
            "expected_urdf_sha256": str(job["expected_urdf_sha256"]),
            "expected_movable_dof": int(job["expected_movable_dof"]),
            "status": "completed" if not asset_failed else "error",
            "load_success": True,
            "load_time_seconds": adapter.load_time_seconds,
            "mapping_status": mapping_status,
            "eligible_pair_count": adapter.mapping.get("eligible_pair_count"),
            "mapped_pair_count": adapter.mapping.get("mapped_pair_count"),
            "source_collision_element_count": adapter.mapping.get("source_collision_element_count"),
            "states_intended": int(job["expected_state_count"]),
            "states_executed": states_executed,
            "state_hash_cross_check": cross,
            "joint_records": joint_records,
            "issues": [] if not asset_failed else [str(failure_reason)],
        }
    except Exception as exc:  # noqa: BLE001
        record = _failed_asset_record(job, issue=f"{type(exc).__name__}: {exc}")
    atomic_write_json(result_path, record)
    return 0


def spawn_children(
    jobs: Sequence[Mapping[str, Any]],
    outdir: Path,
    *,
    workers: int,
    timeout_seconds: float,
) -> list[dict[str, Any]]:
    genesis_python = lam4a.DEFAULT_GENESIS_PYTHON.resolve(strict=True)
    job_dir = outdir / "jobs"
    child_dir = outdir / "children"
    log_dir = outdir / "child_logs"
    cache_root = outdir / "genesis_caches"
    for directory in (job_dir, child_dir, log_dir, cache_root):
        directory.mkdir(parents=True, exist_ok=True)
    results: list[dict[str, Any]] = []
    pending: dict[int, dict[str, Any]] = {}
    template_cache: dict[str, Any] = {"path": None}
    total = len(jobs)
    next_index = 0
    cpu_count = os.cpu_count() or 1

    def launch(index: int) -> None:
        nonlocal next_index
        job = dict(jobs[index])
        slot = index
        width = max(1, min(lam4a.CPU_AFFINITY_WIDTH, cpu_count))
        base = (slot * width) % cpu_count
        affinity = ",".join(str((base + offset) % cpu_count) for offset in range(width))
        job["genesis_cache_path"] = str(cache_root / f"rank_{index + 1:04d}")
        job["template_cache_path"] = template_cache["path"]
        job_path = job_dir / f"rank_{index + 1:04d}.json"
        result_path = child_dir / f"rank_{index + 1:04d}.json"
        atomic_write_json(job_path, job)
        env = os.environ.copy()
        env["GS_CACHE_FILE_PATH"] = job["genesis_cache_path"]
        for key, value in lam4a.THREAD_ENV_VALUES.items():
            env[key] = value
        env[lam4a.CPU_AFFINITY_ENV] = affinity
        env["PYTHONDONTWRITEBYTECODE"] = "1"
        log_path = log_dir / f"rank_{index + 1:04d}.log"
        log_handle = log_path.open("w", encoding="utf-8")
        proc = subprocess.Popen(
            [str(genesis_python), str(SCRIPT), "--child", "--job", str(job_path), "--result", str(result_path)],
            env=env,
            stdout=log_handle,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            cwd=str(REPO),
        )
        pending[index] = {
            "proc": proc,
            "result_path": result_path,
            "log_handle": log_handle,
            "deadline": time.time() + timeout_seconds,
        }

    def finalize(index: int, entry: dict[str, Any], reason: str | None) -> None:
        try:
            entry.get("log_handle").close()
        except Exception:  # noqa: BLE001
            pass
        result_path: Path = entry["result_path"]
        record = None
        if result_path.is_file():
            try:
                record = json.loads(result_path.read_text(encoding="utf-8"))
            except Exception:  # noqa: BLE001
                record = None
        if record is None:
            record = _failed_asset_record(jobs[index], issue=reason or "child_result_missing")
            atomic_write_json(result_path, record)
        results.append(record)

    def kill_entry(entry: dict[str, Any]) -> None:
        proc = entry["proc"]
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except OSError:
            pass
        try:
            proc.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except OSError:
                pass
            try:
                proc.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                pass

    if total > 0 and workers > 1:
        launch(0)
        next_index = 1
        while len(results) < 1:
            time.sleep(0.1)
            for windex in sorted(pending):
                entry = pending[windex]
                proc = entry["proc"]
                rc = proc.poll()
                if rc is not None:
                    del pending[windex]
                    finalize(windex, entry, None if rc == 0 else f"child_process_failed: rc={rc}")
                elif time.time() >= entry["deadline"]:
                    kill_entry(entry)
                    del pending[windex]
                    finalize(windex, entry, f"asset_timeout after {timeout_seconds}s")
        if template_cache["path"] is None:
            template_cache["path"] = str(cache_root / "rank_0001")
    while len(results) < total:
        launched_now = False
        while len(pending) < workers and next_index < total:
            launch(next_index)
            next_index += 1
            launched_now = True
        if launched_now:
            time.sleep(LAUNCH_STAGGER_SECONDS)
        time.sleep(0.1)
        for index in sorted(pending):
            entry = pending[index]
            proc = entry["proc"]
            rc = proc.poll()
            if rc is not None:
                del pending[index]
                finalize(index, entry, None if rc == 0 else f"child_process_failed: rc={rc}")
            elif time.time() >= entry["deadline"]:
                kill_entry(entry)
                del pending[index]
                finalize(index, entry, f"asset_timeout after {timeout_seconds}s")
    results.sort(key=lambda r: int(r["selection_index"]))
    return results


def rate(numerator: float, denominator: float) -> dict[str, Any]:
    return {
        "numerator": int(numerator),
        "denominator": int(denominator),
        "percent": round(100.0 * numerator / denominator, 4) if denominator else None,
    }


def percentile_linear(values: Sequence[float], q: float) -> float:
    if not values:
        raise ValueError("empty values")
    ordered = sorted(values)
    if len(ordered) == 1:
        return float(ordered[0])
    position = q * (len(ordered) - 1)
    lo = int(math.floor(position))
    hi = int(math.ceil(position))
    if lo == hi:
        return float(ordered[lo])
    fraction = position - lo
    return float(ordered[lo] + (ordered[hi] - ordered[lo]) * fraction)


def aggregate(records: Sequence[Mapping[str, Any]], table4_strict: Mapping[str, bool]) -> dict[str, Any]:
    n = len(records)
    completed = sum(1 for r in records if r.get("status") == "completed")
    joints_intended = 0
    joints_passed = 0
    safe_total = 0
    bounded_intended = 0
    bounded_passed = 0
    continuous_count = 0
    states_intended = 0
    states_executed = 0
    cross = {"verified": 0, "mismatch": 0, "no_reference": 0}
    safe_per_asset: list[int] = []
    bins: dict[str, dict[str, Any]] = {
        label: {
            "n_eval": 0,
            "joints_intended": 0,
            "joints_passed": 0,
            "safe": 0,
            "strict_pass": 0,
        }
        for label, _, _ in DOF_BINS
    }
    bins["unknown/unparseable"] = {"n_eval": 0, "joints_intended": 0, "joints_passed": 0, "safe": 0, "strict_pass": 0}
    for record in records:
        dataset_id = str(record["dataset_id"])
        dof = record.get("expected_movable_dof")
        bin_label = "unknown/unparseable"
        if isinstance(dof, int) and not isinstance(dof, bool):
            for label, lo, hi in DOF_BINS:
                if lo <= dof <= hi:
                    bin_label = label
                    break
        bucket = bins[bin_label]
        bucket["n_eval"] += 1
        bucket["strict_pass"] += int(bool(table4_strict.get(dataset_id, False)))
        asset_safe = 0
        for joint in record.get("joint_records", []):
            joints_intended += 1
            joints_passed += int(bool(joint["full_range_cf_pass"]))
            bucket["joints_intended"] += 1
            bucket["joints_passed"] += int(bool(joint["full_range_cf_pass"]))
            safe = int(joint["safe_dof"])
            asset_safe += safe
            safe_total += safe
            bucket["safe"] += safe
            if joint["joint_type"] == "continuous":
                continuous_count += 1
            else:
                bounded_intended += 1
                bounded_passed += int(bool(joint["limit_reachable"]))
        safe_per_asset.append(asset_safe)
        states_intended += int(record.get("states_intended", 0))
        states_executed += int(record.get("states_executed", 0))
        for key in cross:
            cross[key] += int(record.get("state_hash_cross_check", {}).get(key, 0))
    return {
        "status_counts": {"completed": completed, "error": n - completed, "total": n},
        "joint_level_full_range_cf": rate(joints_passed, joints_intended),
        "collision_safe_dof_retention": rate(safe_total, J_EVAL),
        "executable_cf_dof_per_asset": {
            "mean": round(statistics.fmean(safe_per_asset), 4),
            "median": percentile_linear(safe_per_asset, 0.5),
            "p90": percentile_linear(safe_per_asset, 0.9),
            "n_assets": n,
            "total_safe_dof": safe_total,
        },
        "limit_reachability": rate(bounded_passed, bounded_intended)
        | {"continuous_excluded": continuous_count},
        "normalized_clearance_p5": {
            "status": "N/E",
            "reason": "genesis_contact_penetration_has_no_signed_clearance_no_independent_exact_distance_backend_registered",
            "state_coverage": rate(states_executed, states_intended),
            "asset_coverage": rate(completed, n),
            "coverage_status": "COMPLETE" if (states_executed == states_intended and completed == n) else "PARTIAL",
        },
        "state_counts": {
            "intended": states_intended,
            "executed": states_executed,
            "hash_cross_check": cross,
        },
        "dof_bins": bins,
    }


def verify_run(
    rows: Sequence[Mapping[str, Any]],
    records: Sequence[Mapping[str, Any]],
    aggregates: Mapping[str, Any],
    table4_strict: Mapping[str, bool],
) -> dict[str, Any]:
    checks: list[dict[str, Any]] = []

    def check(name: str, ok: bool, detail: str = "") -> None:
        checks.append({"check": name, "pass": bool(ok), "detail": detail})

    check("record_count", len(records) == N_EVAL, f"{len(records)}")
    order_ok = all(
        int(r["selection_index"]) == i and str(r["dataset_id"]) == str(rows[i]["dataset_id"])
        for i, r in enumerate(records)
    )
    check("frozen_order_preserved", order_ok)
    intended = aggregates["joint_level_full_range_cf"]["denominator"]
    check("j_eval_denominator", intended == J_EVAL, f"{intended}")
    retention_den = aggregates["collision_safe_dof_retention"]["denominator"]
    check("retention_denominator", retention_den == J_EVAL, f"{retention_den}")
    sha_ok = all(
        (r.get("urdf_sha256") is None and r.get("status") != "completed")
        or r.get("urdf_sha256") == str(rows[int(r["selection_index"])]["urdf_sha256"])
        for r in records
    )
    check("urdf_identity_matches_frozen_manifest", sha_ok)
    cross = aggregates["state_counts"]["hash_cross_check"]
    check("state_hash_cross_check_no_mismatch", cross["mismatch"] == 0, f"{cross}")
    states_intended = aggregates["state_counts"]["intended"]
    check("state_intended_count", states_intended == SINGLE_SAMPLES * J_EVAL, f"{states_intended}")
    eligible = [r.get("eligible_pair_count") for r in records if r.get("eligible_pair_count") is not None]
    check(
        "claim_boundary_zero_collision_elements",
        all(count == 0 for count in eligible),
        f"assets_with_eligible_pairs_reported={len(eligible)}",
    )
    recomputed = aggregate(records, table4_strict)
    check(
        "aggregate_recomputation_matches",
        lam4a.canonical_sha256(recomputed) == lam4a.canonical_sha256(dict(aggregates)),
    )
    all_pass = all(c["pass"] for c in checks)
    return {"all_pass": all_pass, "check_count": len(checks), "checks": checks}


def render_summary_md(summary: Mapping[str, Any]) -> str:
    m = summary["metrics"]
    cf = m["joint_level_full_range_cf"]
    ret = m["collision_safe_dof_retention"]
    exe = m["executable_cf_dof_per_asset"]
    lim = m["limit_reachability"]
    p5 = m["normalized_clearance_p5"]
    lines = [
        "# Table 4a — PhysX-Mobility (frozen Table 5 receipt-set cohort, N=800; Genesis contact-penetration oracle)",
        "",
        f"- Protocol ID: `{summary['protocol_id']}` (engine `{summary['engine_protocol_id']}`)",
        f"- Run directory: `{summary['run_directory']}`",
        f"- N_eval = {summary['cohort']['n_eval']}, J_eval = {summary['cohort']['j_eval']}",
        f"- Status: completed = {summary['status_counts']['completed']}, error = {summary['status_counts']['error']}",
        "- Claim boundary: official PhysX-Mobility URDFs declare zero collision elements; every asset has 0 eligible collision pairs, so all collision-free outcomes are vacuous.",
        "",
        "| Dataset / Outputs | Joint-level Full-range CF ↑ | Executable CF DoF/Asset ↑ | Collision-safe DoF Retention ↑ | Normalized Clearance P5 ↑ | Limit Reachability ↑ |",
        "|---|---:|---:|---:|---:|---:|",
        (
            f"| PhysX-Mobility "
            f"| {cf['numerator']} / {cf['denominator']} ({cf['percent']:.4f}%) "
            f"| mean {exe['mean']} / median {exe['median']} / P90 {exe['p90']} "
            f"| {ret['numerator']} / {ret['denominator']} ({ret['percent']:.4f}%) "
            f"| N/E ({p5['coverage_status']}; states {p5['state_coverage']['numerator']} / {p5['state_coverage']['denominator']}) "
            f"| {lim['numerator']} / {lim['denominator']} ({lim['percent']:.4f}%; continuous excluded: {lim['continuous_excluded']}) |"
        ),
        "",
        "## DoF bins (declared movable DoF; existing Strict Collision Pass from PhysX Table 4)",
        "",
        "| DoF bin | N_eval | Joint-level Full-range CF ↑ | Collision-safe DoF Retention ↑ | Existing Strict Collision Pass ↑ |",
        "|---|---:|---:|---:|---:|",
    ]
    for label, _, _ in DOF_BINS:
        b = m["dof_bins"][label]
        cf_rate = rate(b["joints_passed"], b["joints_intended"])
        ret_rate = rate(b["safe"], b["joints_intended"])
        strict_rate = rate(b["strict_pass"], b["n_eval"])
        lines.append(
            f"| {label} | {b['n_eval']} "
            f"| {cf_rate['numerator']} / {cf_rate['denominator']} ({cf_rate['percent'] if cf_rate['percent'] is not None else 'N/A'}%) "
            f"| {ret_rate['numerator']} / {ret_rate['denominator']} ({ret_rate['percent'] if ret_rate['percent'] is not None else 'N/A'}%) "
            f"| {strict_rate['numerator']} / {strict_rate['denominator']} ({strict_rate['percent'] if strict_rate['percent'] is not None else 'N/A'}%) |"
        )
    b = m["dof_bins"]["unknown/unparseable"]
    if b["n_eval"]:
        lines.append(f"| unknown/unparseable | {b['n_eval']} | - | - | {b['strict_pass']} / {b['n_eval']} |")
    lines.append("")
    return "\n".join(lines) + "\n"


def run_scope(args: argparse.Namespace) -> int:
    mode = args.mode
    timestamp = _dt.datetime.now(_dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    if mode == "formal":
        outdir = REPO / f"exp/runtime/table4a_urdf_physx_mobility_table5cohort_n800_{timestamp}"
    else:
        outdir = REPO / f"exp/runtime/table4a_urdf_physx_mobility_smoke_n{args.n}_{timestamp}"
    if outdir.exists():
        raise SystemExit(f"output directory already exists: {outdir}")
    outdir.mkdir(parents=True)

    dataset_root = (args.dataset_root or DEFAULT_DATASET_ROOT).resolve(strict=True)
    receipt_dir = (args.receipt_set or DEFAULT_RECEIPT_SET).resolve(strict=True)
    cohort = TABLE1P.load_formal_cohort(
        receipt_dir,
        dataset_root,
        expected_n=N_EVAL,
        verify_archive=not args.skip_archive_binding,
    )
    manifest5 = cohort["manifest"]
    rows = manifest5["rows"]

    table3_pass, table3_joint_total = load_table3_joint_pass()
    if table3_joint_total != J_EVAL:
        raise SystemExit(f"Table 3 joint count {table3_joint_total} != J_EVAL {J_EVAL}")
    table4_strict = load_table4_strict_pass()
    state_hashes = load_table4_state_hashes()

    staging_root = outdir / "staging"
    staging_root.mkdir(parents=True)
    staged: dict[int, dict[str, Any]] = {}
    for row in rows:
        staged[int(row["dataset_id"])] = stage_package(dataset_root, row, staging_root)

    jobs = build_jobs(rows, staged, table3_pass, state_hashes)
    if mode == "smoke":
        jobs = jobs[: args.n]

    protocol_snapshot_text = PROTOCOL_DOCUMENT.read_text(encoding="utf-8")
    atomic_write_text(outdir / "protocol_snapshot.md", protocol_snapshot_text)

    frozen_config = {
        "protocol_id": PROTOCOL_ID,
        "engine_protocol_id": ENGINE_PROTOCOL_ID,
        "frozen_at_utc": utc_now_iso(),
        "mode": mode,
        "dataset": DATASET,
        "classification": CLASSIFICATION if mode == "formal" else "SMOKE",
        "protocol_document_sha256": sha256_bytes(protocol_snapshot_text.encode("utf-8")),
        "cohort": {
            "receipt_set": str(receipt_dir),
            "receipt_manifest_sha256": lam4a.sha256_file(receipt_dir / "manifest.json"),
            "manifest_cohort_sha256": manifest5["cohort_sha256"],
            "n_eval": len(jobs),
            "j_eval": sum(job["expected_movable_dof"] for job in jobs),
            "selection_policy": "all Table 5 frozen receipt-set rows in existing rank order; no resampling or result-based filtering",
        },
        "metric_rules": METRIC_RULES,
        "operationalization": OPERATIONALIZATION,
        "execution": {
            "child_timeout_seconds": CHILD_TIMEOUT_SECONDS,
            "workers": WORKERS if args.workers is None else args.workers,
            "child_interpreter": str(lam4a.DEFAULT_GENESIS_PYTHON.resolve(strict=True)),
            "genesis_cache_policy": "per-rank private GS_CACHE_FILE_PATH under output root; rank 1 warmup cache is copied as a read template into each later rank cache",
            "denominator_policy": "all frozen assets, joints and states including failures and unexecuted items",
            "dof_bins": [label for label, _, _ in DOF_BINS] + ["unknown/unparseable"],
            "table3_source": {"records": str(TABLE3_RECORDS), "sha256": lam4a.sha256_file(TABLE3_RECORDS)},
            "table4_state_records": str(TABLE4_STATE_RECORDS),
            "table4_state_records_sha256": lam4a.sha256_file(TABLE4_STATE_RECORDS),
            "table4_asset_records": str(TABLE4_ASSET_RECORDS),
            "table4_asset_records_sha256": lam4a.sha256_file(TABLE4_ASSET_RECORDS),
            "archive_binding_verified": not args.skip_archive_binding,
        },
        "runner_identity": {
            "runner_script": str(SCRIPT),
            "runner_script_sha256": lam4a.sha256_file(SCRIPT),
            "lam_adapter_module": str(Path(lam4a.__file__).resolve()),
            "lam_adapter_module_sha256": lam4a.sha256_file(Path(lam4a.__file__).resolve()),
            "static_module_sha256": lam4a.sha256_file(Path(lam4a.static.__file__).resolve()),
        },
    }
    atomic_write_json(outdir / "frozen_config.json", frozen_config)
    frozen_config_sha256 = lam4a.sha256_file(outdir / "frozen_config.json")

    started_at = utc_now_iso()
    wall_start = time.time()
    records = spawn_children(
        jobs,
        outdir,
        workers=WORKERS if args.workers is None else args.workers,
        timeout_seconds=CHILD_TIMEOUT_SECONDS,
    )
    wall_seconds = round(time.time() - wall_start, 3)

    slim_rows = []
    joint_rows = []
    for record in records:
        slim = {k: v for k, v in record.items() if k != "joint_records"}
        slim_rows.append(slim)
        for joint in record.get("joint_records", []):
            joint_row = {
                "dataset_id": record["dataset_id"],
                "category": record["category"],
                "selection_index": record["selection_index"],
                "asset_status": record["status"],
            }
            joint_rows.append({**joint_row, **{k: v for k, v in joint.items() if k != "state_summaries"}})
    atomic_write_text(
        outdir / "asset_records.jsonl",
        "\n".join(json.dumps(r, sort_keys=True, ensure_ascii=True, allow_nan=False) for r in slim_rows) + "\n",
    )
    atomic_write_text(
        outdir / "joint_records.jsonl",
        "\n".join(json.dumps(r, sort_keys=True, ensure_ascii=True, allow_nan=False) for r in joint_rows) + "\n",
    )

    aggregates = aggregate(records, table4_strict)
    completed_at = utc_now_iso()
    summary = {
        "protocol_id": PROTOCOL_ID,
        "engine_protocol_id": ENGINE_PROTOCOL_ID,
        "mode": mode,
        "dataset": DATASET,
        "classification": CLASSIFICATION if mode == "formal" else "SMOKE",
        "run_directory": str(outdir),
        "started_at_utc": started_at,
        "completed_at_utc": completed_at,
        "wall_seconds": wall_seconds,
        "cohort": {
            "receipt_set": str(receipt_dir),
            "receipt_manifest_sha256": lam4a.sha256_file(receipt_dir / "manifest.json"),
            "manifest_cohort_sha256": manifest5["cohort_sha256"],
            "n_eval": len(jobs),
            "j_eval": sum(job["expected_movable_dof"] for job in jobs),
        },
        "status_counts": aggregates["status_counts"],
        "metrics": aggregates,
        "frozen_config_sha256": frozen_config_sha256,
        "claim_boundary": {
            "official_urdf_zero_collision_elements": True,
            "vacuous_collision_free_outcomes": True,
            "table5_receipt_strict_collision": "N/E",
            "table5_receipt_strict_collision_reason": "official_urdf_zero_collision_elements",
        },
        "notes": [
            "Joint-level Full-range CF reuses the exact frozen PhysX Table 4 single-joint sweep states (K=21, endpoints included, other joints q=0); per-state q-vector hashes are cross-checked against the PhysX Table 4 state_records where available.",
            "State collision oracle = Genesis contact-penetration backend (genesis_contact_penetration_v1), direct kinematic detection only, illegal iff eligible-pair penetration > 1e-6 m.",
            "Headline pair policy = distinct source links excluding direct parent-child; no method-specific allowance.",
            "Normalized Clearance P5 is N/E under this oracle (no signed clearance for separated pairs; no independent exact-distance backend registered).",
            "Existing Strict Collision Pass values in DoF bins are historical PhysX Table 4 (PyBullet) results reported alongside, not re-executed.",
            "Official PhysX-Mobility URDFs declare zero collision elements: every asset maps to 0 eligible collision pairs; all collision-free outcomes are vacuous and must not be read as verified mechanical clearance.",
        ],
    }
    atomic_write_json(outdir / "summary.json", summary)
    atomic_write_text(outdir / "summary.md", render_summary_md(summary))

    verification = (
        verify_run(rows, records, aggregates, table4_strict)
        if mode == "formal"
        else {"all_pass": None, "note": "smoke mode: formal verification skipped"}
    )
    atomic_write_json(outdir / "verification.json", verification)
    run_manifest = {
        "protocol_id": PROTOCOL_ID,
        "engine_protocol_id": ENGINE_PROTOCOL_ID,
        "mode": mode,
        "dataset": DATASET,
        "created_at_utc": completed_at,
        "command": sys.argv,
        "frozen_config_sha256": frozen_config_sha256,
        "record_count": len(records),
        "status_counts": aggregates["status_counts"],
        "wall_seconds": wall_seconds,
        "outputs": {
            "asset_records_sha256": lam4a.sha256_file(outdir / "asset_records.jsonl"),
            "joint_records_sha256": lam4a.sha256_file(outdir / "joint_records.jsonl"),
            "summary_sha256": lam4a.sha256_file(outdir / "summary.json"),
            "summary_md_sha256": lam4a.sha256_file(outdir / "summary.md"),
            "verification_sha256": lam4a.sha256_file(outdir / "verification.json"),
        },
        "verification": verification,
    }
    atomic_write_json(outdir / "manifest.json", run_manifest)
    run_manifest["manifest_self_sha256_at_write"] = lam4a.sha256_file(outdir / "manifest.json")
    atomic_write_json(outdir / "manifest.json", run_manifest)

    print(json.dumps(
        {
            "mode": mode,
            "run_directory": str(outdir),
            "status_counts": aggregates["status_counts"],
            "joint_level_full_range_cf": aggregates["joint_level_full_range_cf"],
            "collision_safe_dof_retention": aggregates["collision_safe_dof_retention"],
            "executable_cf_dof_per_asset": aggregates["executable_cf_dof_per_asset"],
            "limit_reachability": aggregates["limit_reachability"],
            "normalized_clearance_p5": aggregates["normalized_clearance_p5"],
            "state_counts": aggregates["state_counts"],
            "verification_all_pass": verification.get("all_pass"),
        },
        indent=2,
        sort_keys=True,
        ensure_ascii=True,
    ))
    if mode == "formal" and not verification["all_pass"]:
        return 2
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("smoke", "formal"), default=None)
    parser.add_argument("--n", type=int, default=3, help="smoke sample size (smoke mode only)")
    parser.add_argument("--workers", type=int, default=None)
    parser.add_argument("--dataset-root", type=Path, default=None)
    parser.add_argument("--receipt-set", type=Path, default=None)
    parser.add_argument("--skip-archive-binding", action="store_true")
    parser.add_argument("--child", action="store_true")
    parser.add_argument("--job", type=Path)
    parser.add_argument("--result", type=Path)
    args = parser.parse_args()
    if args.child:
        if args.job is None or args.result is None:
            raise SystemExit("--child requires --job and --result")
        return run_child(args.job, args.result)
    if args.mode is None:
        raise SystemExit("--mode is required unless --child is given")
    return run_scope(args)


if __name__ == "__main__":
    raise SystemExit(main())
