#!/usr/bin/env python3
"""Fail-closed Table 4a runner for Articraft-10K (Genesis contact-penetration oracle).

Runs the proposed Table 4a DoF-aware Mechanical Safety metrics over the frozen
Articraft-10K Table 2 cohort (reused unchanged by the frozen Articraft Table 4
manifest), reusing the version-pinned ``GenesisTable4aAdapter`` from the LAM
supplementary runner so that the engine protocol
(``genesis_contact_penetration_v1``) is identical across methods.

Scope: Table 4a only (joint_full_range states; K=21 frozen Table 4 sweep
states per movable joint, other joints at historical q=0).  Sobol/rest strict
states are not re-executed; the existing Table 4 ``Strict Collision Pass`` is
reported per DoF bin from the frozen Table 4 asset records.

Normalized Clearance P5 is N/E under this oracle: the frozen adapter reports
no signed clearance for separated pairs and this run registers no independent
exact-distance backend.
"""

from __future__ import annotations

import argparse
import datetime as _dt
import json
import math
import os
from pathlib import Path
import platform
import signal
import statistics
import subprocess
import sys
import time
from typing import Any, Mapping, Sequence

SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from exp.scripts import run_urdf_lam_supplementary_v1 as lam4a  # noqa: E402

SCHEMA_VERSION = "table4a-articraft10k/v1"
PROTOCOL_ID = "table4a_articraft10k_table2cohort_n800_v1"
ENGINE_PROTOCOL_ID = lam4a.ENGINE_PROTOCOL_ID
DATASET = "Articraft-10K"
CLASSIFICATION = "FORMAL"

DATASET_ROOT = REPO / "exp/Articraft-10K"
SOURCE_MANIFEST = REPO / "exp/runtime/urdf_table4_articraft10k_n800_20260814/frozen_manifest.json"
EXPECTED_SOURCE_MANIFEST_FILE_SHA256 = "6b4275cf3da29244af70c04acecd87094f0c158dee992db20b04e90c05292c20"
EXPECTED_SOURCE_MANIFEST_CONTENT_SHA256 = "1c6ba7d9e19818580fe8573cf95bb1d065bf2235d0699070516888520f86d7b6"
EXPECTED_ORDERED_IDS_SHA256 = "8417b0874f111bfe16b63c2702358ea501ced723b22fd328c9ad21e80e6115de"
TABLE4_STATE_RECORDS = SOURCE_MANIFEST.with_name("state_records.jsonl")
EXPECTED_TABLE4_STATE_RECORDS_SHA256 = "6efd4031ecebf74f30f8d3ec3c312ae2faf1b521322b5d4a8b57bb732177ac8b"
TABLE4_ASSET_RECORDS = SOURCE_MANIFEST.with_name("asset_records.json")
EXPECTED_TABLE4_ASSET_RECORDS_SHA256 = "b732a53a464a8aeebb74799d5ec737de75f3cca377c9a5b274a5dd35adbe301b"
TABLE3_RECORDS = (
    REPO / "exp/runtime/urdf_table3_articraft10k_table2_n800_20260814T040300Z/asset_records.jsonl"
)
EXPECTED_TABLE3_RECORDS_SHA256 = "2dbb09fab36fe60b469eb38439708250f4af3fe75fb0d6dcd118e49c8febf103"
TABLE2_COHORT_MANIFEST = (
    REPO / "exp/runtime/table2_urdf_articraft10k_n800_seed20260813_20260813T145915Z/manifest.json"
)
EXPECTED_TABLE2_COHORT_FILE_SHA256 = "13c47e2b2affadb951a01cab826bae139852fca5769e99ec081cc916ffa6373d"
EXPECTED_TABLE2_COHORT_CONTENT_SHA256 = "576852cb6da00775e1c51360b82b4be40e0a614e4fb0cfb1bae066912eed56a3"
CATEGORY_RECORDS_ROOT = REPO / "exp/baselines/Articraft-10K-official/records"
EXPECTED_CATEGORY_MAPPING_SHA256 = "0305569f49d2aa1acb72fbb7bc8dcaf68ca3dd4a5bd7eba140b5bac4c8c0f449"
EXPECTED_CATEGORY_RECORDS_REVISION = "677ca9722427dce500873730255874c8c3f07eb2"
PROTOCOL_DOCUMENT = REPO / "exp/URDF-Sim-Ready-Automatic-Evaluation.md"

N_EVAL = 800
J_EVAL = 2865
EXPECTED_CATEGORY_COUNT = 222
SINGLE_SAMPLES = 21
ZERO_WIDTH_TOLERANCE = 1e-12
CONTINUOUS_INTERVAL = (-math.pi, math.pi)
# Timeout is a run-safety parameter, not a metric definition; timeouts remain
# fail-closed and stay in every denominator.  Frozen at 1800 s (the PartNet
# Table 4a precedent) after Articraft smoke calibration on the idle machine:
# legitimate collision-bearing assets take 549-1256 s cold (rec_radial_arm
# 549 s, rec_washing_machine_variant 824 s, rec_13-dof stack with 369
# collision elements 1256 s), so the historical 900 s would fail-closed
# legitimate assets under any realistic concurrent load.
CHILD_TIMEOUT_SECONDS = 1800
WORKERS = 24
PRIVATE_GENESIS_CACHES = False
LAUNCH_STAGGER_SECONDS = 1.5
EARLY_CPU_AFFINITY_LAUNCHER: Path | None = None
EXPECTED_EARLY_CPU_AFFINITY_LAUNCHER_RECEIPT: dict[str, Any] | None = None
GENESIS_CACHE_POLICY = (
    "single shared GS_CACHE_FILE_PATH under output root (LAM sequential precedent); "
    "rank 1 runs alone as warmup before parallel children"
)
DOF_BINS: list[tuple[str, int, int]] = [
    ("0", 0, 0),
    ("1", 1, 1),
    ("2--3", 2, 3),
    ("4--7", 4, 7),
    (">=8", 8, 10**9),
]

SELECTION_POLICY = (
    "exact Table 2 manifest .records[].package cohort, reused unchanged by the frozen "
    "Articraft Table 4 manifest; no resampling or result-based filtering"
)

RUN_NOTES = [
    "Joint-level Full-range CF reuses the exact frozen Table 4 single-joint sweep states (K=21, endpoints included, other joints q=0); per-state q-vector hashes are cross-checked against Table 4 state_records where available (legacy Articraft Table 4 executed 18,648 / 60,165 sweep states; the rest are no_reference).",
    "State collision oracle = Genesis contact-penetration backend (genesis_contact_penetration_v1), direct kinematic detection only, illegal iff eligible-pair penetration > 1e-6 m.",
    "Headline pair policy = distinct source links excluding direct parent-child; no method-specific allowance. Assets with zero declared collision geometry have zero eligible pairs and pass vacuously once all states execute.",
    "Normalized Clearance P5 is N/E under this oracle (no signed clearance for separated pairs; no independent exact-distance backend registered).",
    "Existing Strict Collision Pass values in DoF bins are historical Table 4 (PyBullet) results reported alongside, not re-executed.",
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
        "fails closed. Assets with zero declared collision geometry have zero eligible pairs and "
        "vacuously satisfy the collision rule once all states execute (fail-closed execution still "
        "applies)."
    ),
    "state_plan": (
        "严格沿用既有 Table 4 的冻结单关节 sweep：每关节 K=21 个状态，取值 lower + i*(upper-lower)/20，"
        "i=0..20（含两端 endpoint）；bounded joint 使用声明 lower/upper（宽度须 > 1e-12），continuous "
        "joint 使用冻结区间 [-pi, pi]；其余关节保持 q=0（与既有 Articraft Table 4 rest_state "
        "'native URDF/PyBullet q=0' 一致）。每个状态的全 DoF 向量 canonical SHA256 与既有 Table 4 "
        "state_records.joint_values_sha256 逐项核对；核对向量按 source-URDF movable-joint 枚举顺序构造"
        "（fixed joints 除外；被扫关节取其 movable_rank 位置的取值，其余为 0），与既有 Table 4 哈希所用的 "
        "PyBullet 关节顺序约定一致，独立于原始 XML joint index 与引擎内部 DOF 排序（Genesis 内部 DOF 顺序"
        "另存于 q_intended_values_sha256 仅作 provenance）；该约定已对全部 18,648 条既有 sweep 参照离线"
        "验证为 18,648 / 18,648 命中。既有 Table 4 运行仅执行了 18,648 / 60,165 个 sweep 状态，无参照状态"
        "仅按冻结规则重生成并计为 no_reference。"
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
    "table3_joint_pass_source": "Table 3 asset_records joints[].joint_level_pass (frozen run urdf_table3_articraft10k_table2_n800_20260814T040300Z), joined by asset_id.",
    "existing_strict_collision_pass_source": "Table 4 asset_records strict_collision_pass (frozen run urdf_table4_articraft10k_n800_20260814; historical PyBullet result, reported per DoF bin only).",
    "category_source": (
        "exact asset_id join to official record.json category_slug "
        "(baselines/Articraft-10K-official/records); used for record provenance only."
    ),
    "percentile_policy": "median/P90 of per-asset safe-DoF counts with linear interpolation (numpy-style), over all 800 assets including zero-DoF failed assets.",
}


def utc_now_iso() -> str:
    return _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")


def sha256_bytes(payload: bytes) -> str:
    import hashlib

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


def load_source_manifest() -> dict[str, Any]:
    payload = SOURCE_MANIFEST.read_bytes()
    digest = sha256_bytes(payload)
    if digest != EXPECTED_SOURCE_MANIFEST_FILE_SHA256:
        raise SystemExit(f"source manifest file sha256 mismatch: {digest}")
    manifest = json.loads(payload)
    if manifest.get("manifest_content_sha256") != EXPECTED_SOURCE_MANIFEST_CONTENT_SHA256:
        raise SystemExit("source manifest content sha256 mismatch")
    items = manifest.get("items")
    if not isinstance(items, list) or len(items) != N_EVAL:
        raise SystemExit(f"source manifest must contain exactly {N_EVAL} items")
    ids = [item["dataset_id"] for item in items]
    ordered = sha256_bytes(json.dumps(ids, separators=(",", ":"), ensure_ascii=True).encode())
    if ordered != EXPECTED_ORDERED_IDS_SHA256:
        raise SystemExit(f"ordered asset id sha256 mismatch: {ordered}")
    for index, item in enumerate(items):
        if item.get("order") != index:
            raise SystemExit(f"item order field mismatch at index {index}")
        if int(item["movable_dof_count"]) != len(item["joint_specs"]):
            raise SystemExit(f"item {item['dataset_id']} dof/spec mismatch")
    return manifest


def load_table2_cohort_identity() -> None:
    payload = TABLE2_COHORT_MANIFEST.read_bytes()
    if sha256_bytes(payload) != EXPECTED_TABLE2_COHORT_FILE_SHA256:
        raise SystemExit("Table 2 cohort manifest file sha256 mismatch")
    cohort = json.loads(payload)
    if cohort.get("manifest_content_sha256") != EXPECTED_TABLE2_COHORT_CONTENT_SHA256:
        raise SystemExit("Table 2 cohort manifest content sha256 mismatch")


def load_table3_joint_pass() -> tuple[dict[str, dict[str, bool]], int]:
    if lam4a.sha256_file(TABLE3_RECORDS) != EXPECTED_TABLE3_RECORDS_SHA256:
        raise SystemExit("Table 3 asset records sha256 mismatch")
    result: dict[str, dict[str, bool]] = {}
    joints_total = 0
    with TABLE3_RECORDS.open(encoding="utf-8") as handle:
        for line in handle:
            record = json.loads(line)
            asset_id = str(record.get("asset_key") or record.get("asset_id"))
            passes: dict[str, bool] = {}
            for joint in record.get("joints") or []:
                passes[str(joint["joint_name"])] = bool(joint["joint_level_pass"])
                joints_total += 1
            result[asset_id] = passes
    return result, joints_total


def load_table4_strict_pass() -> dict[str, bool]:
    if lam4a.sha256_file(TABLE4_ASSET_RECORDS) != EXPECTED_TABLE4_ASSET_RECORDS_SHA256:
        raise SystemExit("Table 4 asset records sha256 mismatch")
    data = json.loads(TABLE4_ASSET_RECORDS.read_text(encoding="utf-8"))
    return {str(r["dataset_id"]): bool(r["strict_collision_pass"]) for r in data}


def load_table4_state_hashes() -> dict[tuple[str, str, int], str]:
    if lam4a.sha256_file(TABLE4_STATE_RECORDS) != EXPECTED_TABLE4_STATE_RECORDS_SHA256:
        raise SystemExit("Table 4 state records sha256 mismatch")
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


def _category_revision() -> str | None:
    checkout = CATEGORY_RECORDS_ROOT.parent
    result = subprocess.run(
        ["git", "-C", str(checkout), "rev-parse", "HEAD"],
        check=False,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip() if result.returncode == 0 else None


def _read_category(asset_id: str) -> str:
    path = CATEGORY_RECORDS_ROOT / asset_id / "record.json"
    try:
        record = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"category record is unavailable or invalid: {asset_id}") from exc
    if record.get("record_id") != asset_id:
        raise ValueError(f"category record identity mismatch: {asset_id}")
    category = str(record.get("category_slug", "")).strip()
    if not category:
        raise ValueError(f"category_slug is missing: {asset_id}")
    return category


def build_jobs(
    manifest: Mapping[str, Any],
    table3_pass: Mapping[str, Mapping[str, bool]],
    state_hashes: Mapping[tuple[str, str, int], str],
    *,
    formal: bool,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    dataset_root = DATASET_ROOT.resolve(strict=True)
    jobs: list[dict[str, Any]] = []
    category_rows: list[dict[str, str]] = []
    for index, item in enumerate(manifest["items"]):
        dataset_id = str(item["dataset_id"])
        asset_id = str(item["asset_id"])
        category = _read_category(asset_id)
        category_rows.append({"asset_id": asset_id, "category_slug": category})
        joints = sorted(item["joint_specs"], key=lambda row: int(row["xml_index"]))
        joint_jobs = []
        for position, row in enumerate(joints):
            values = single_joint_values(row)
            references = []
            for sample_index in range(SINGLE_SAMPLES):
                references.append(state_hashes.get((dataset_id, str(row["name"]), sample_index)))
            joint_jobs.append(
                {
                    "name": str(row["name"]),
                    "type": str(row["type"]),
                    "lower": row.get("lower"),
                    "upper": row.get("upper"),
                    "xml_index": int(row["xml_index"]),
                    "movable_rank": position,
                    "dof_position": position,
                    "values": values,
                    "state_hash_references": references,
                    "table3_joint_level_pass": bool(
                        table3_pass.get(asset_id, {}).get(str(row["name"]), False)
                    ),
                    "table3_joint_present": str(row["name"]) in table3_pass.get(asset_id, {}),
                }
            )
        urdf_path = dataset_root / str(item["primary_urdf_relpath"])
        jobs.append(
            {
                "selection_index": index,
                "dataset_id": dataset_id,
                "asset_id": asset_id,
                "category": category,
                "package": str(dataset_root / str(item["package_relpath"])),
                "urdf_path": str(urdf_path),
                "expected_urdf_sha256": str(item["urdf_sha256"]),
                "input_identity_sha256": str(item["input_identity_sha256"]),
                "expected_movable_dof": int(item["movable_dof_count"]),
                "joints": joint_jobs,
                "expected_state_count": SINGLE_SAMPLES * len(joint_jobs),
            }
        )
    category_mapping_hash = lam4a.canonical_sha256(category_rows)
    category_info = {
        "category_records_root": str(CATEGORY_RECORDS_ROOT),
        "category_records_revision": _category_revision(),
        "category_mapping_policy": "exact asset_id join to official record.json category_slug",
        "category_mapping_sha256": category_mapping_hash,
        "eval_category_count": len({row["category_slug"] for row in category_rows}),
    }
    if formal:
        if category_mapping_hash != EXPECTED_CATEGORY_MAPPING_SHA256:
            raise SystemExit("formal category mapping sha256 mismatch")
        if category_info["category_records_revision"] != EXPECTED_CATEGORY_RECORDS_REVISION:
            raise SystemExit("formal category records revision mismatch")
        if category_info["eval_category_count"] != EXPECTED_CATEGORY_COUNT:
            raise SystemExit(f"formal cohort must cover exactly {EXPECTED_CATEGORY_COUNT} categories")
    return jobs, category_info


def _failed_asset_record(job: Mapping[str, Any], issue: str) -> dict[str, Any]:
    joints = []
    for joint in job["joints"]:
        joints.append(
            {
                "joint_name": joint["name"],
                "joint_type": joint["type"],
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
        "asset_id": str(job["asset_id"]),
        "category": str(job["category"]),
        "package": str(job["package"]),
        "urdf_sha256": None,
        "expected_urdf_sha256": str(job["expected_urdf_sha256"]),
        "expected_movable_dof": int(job["expected_movable_dof"]),
        "status": "error",
        "load_success": False,
        "mapping_status": "NOT_EVALUABLE",
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
            import shutil

            shutil.copytree(template_path, cache_path)
    cache_path.mkdir(parents=True, exist_ok=True)
    record: dict[str, Any]
    try:
        urdf_path = Path(str(job["urdf_path"]))
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
                intended_hash = lam4a.canonical_sha256(engine_record["q_intended_values"])
                # Cross-check convention: the legacy Table 4 joint_values_sha256
                # hashes the full movable-joint q-vector in source-URDF
                # movable-joint enumeration order (fixed joints excluded; the
                # legacy runner's PyBullet joint order), not the raw XML joint
                # index and not the engine-internal DOF order.  Reconstruct
                # that canonical vector (swept value at movable_rank, zeros
                # elsewhere) so identity is compared order-independently
                # against the frozen reference.  Verified offline against all
                # 18,648 legacy sweep references (223 assets): 18,648/18,648
                # matches under this convention.
                xml_order_vector = [0.0] * int(job["expected_movable_dof"])
                xml_order_vector[int(joint["movable_rank"])] = float(value)
                xml_order_hash = lam4a.canonical_sha256(xml_order_vector)
                reference = joint["state_hash_references"][sample_index]
                if reference is None:
                    cross["no_reference"] += 1
                elif xml_order_hash == reference:
                    cross["verified"] += 1
                else:
                    cross["mismatch"] += 1
                    asset_failed = True
                    failure_reason = (
                        f"state_identity_mismatch: {name}#{sample_index} "
                        f"got {xml_order_hash}, expected {reference}"
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
                        "q_xml_order_sha256": xml_order_hash,
                    }
                )
            states_executed += executed
            full_range_pass = bool(
                executed == SINGLE_SAMPLES
                and illegal == 0
                and not asset_failed
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
        record = {
            "schema_version": SCHEMA_VERSION,
            "protocol_id": PROTOCOL_ID,
            "engine_protocol_id": ENGINE_PROTOCOL_ID,
            "selection_index": int(job["selection_index"]),
            "dataset_id": str(job["dataset_id"]),
            "asset_id": str(job["asset_id"]),
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
            "states_intended": int(job["expected_state_count"]),
            "states_executed": states_executed,
            "state_hash_cross_check": cross,
            "joint_records": joint_records,
            "issues": [failure_reason] if failure_reason else [],
        }
        try:
            adapter.close()
        except Exception:  # noqa: BLE001
            pass
    except Exception as exc:  # noqa: BLE001
        record = _failed_asset_record(job, issue=f"load_failed: {type(exc).__name__}: {exc}")
    record["child"] = {
        "finished_at_utc": utc_now_iso(),
        "python_executable": sys.executable,
        "python_version": platform.python_version(),
    }
    atomic_write_json(result_path, record)
    return 0


def early_cpu_affinity_launcher_receipt() -> dict[str, Any] | None:
    if EARLY_CPU_AFFINITY_LAUNCHER is None:
        return None
    launcher = EARLY_CPU_AFFINITY_LAUNCHER
    if launcher.is_symlink() or not launcher.is_file():
        raise ValueError(f"unsafe early CPU affinity launcher: {launcher}")
    launcher = launcher.resolve(strict=True)
    return {
        "path": str(launcher),
        "bytes": launcher.stat().st_size,
        "sha256": lam4a.sha256_file(launcher),
    }


def _child_launch_command(
    genesis_python: str,
    job_path: Path,
    result_path: Path,
) -> list[str]:
    child_arguments = [
        str(SCRIPT),
        "--child",
        "--job",
        str(job_path),
        "--result",
        str(result_path),
    ]
    receipt = early_cpu_affinity_launcher_receipt()
    if receipt is None:
        return [genesis_python, *child_arguments]
    if (
        EXPECTED_EARLY_CPU_AFFINITY_LAUNCHER_RECEIPT is not None
        and receipt != EXPECTED_EARLY_CPU_AFFINITY_LAUNCHER_RECEIPT
    ):
        raise ValueError("early CPU affinity launcher drifted after run freeze")
    return [genesis_python, "-S", str(receipt["path"]), "--", *child_arguments]


def _cpu_affinity_for_slot(
    available_cpus: Sequence[int], slot: int, width: int
) -> str:
    if (
        slot < 0
        or width < 1
        or list(available_cpus) != sorted(set(available_cpus))
    ):
        raise ValueError("invalid CPU affinity allocation input")
    start = slot * width
    selected = list(available_cpus[start : start + width])
    if len(selected) != width:
        raise ValueError("insufficient CPUs for non-overlapping child affinity")
    return ",".join(str(cpu) for cpu in selected)


def spawn_children(
    jobs: Sequence[Mapping[str, Any]],
    outdir: Path,
    *,
    workers: int,
    timeout_seconds: float,
) -> list[dict[str, Any]]:
    job_dir = outdir / "child_jobs"
    child_dir = outdir / "children"
    job_dir.mkdir(parents=True, exist_ok=True)
    child_dir.mkdir(parents=True, exist_ok=True)
    cache_root = outdir / "genesis-cache"
    cache_root.mkdir(parents=True, exist_ok=True)
    shared_cache = cache_root / "shared"
    if not PRIVATE_GENESIS_CACHES:
        shared_cache.mkdir(parents=True, exist_ok=True)
    template_cache: dict[str, str | None] = {"path": None}
    genesis_python = str(lam4a.DEFAULT_GENESIS_PYTHON.resolve(strict=True))
    results: list[dict[str, Any]] = []
    pending: dict[int, dict[str, Any]] = {}
    next_index = 0
    total = len(jobs)
    available_cpus = sorted(int(cpu) for cpu in os.sched_getaffinity(0))
    cpu_count = len(available_cpus)
    width = max(1, min(lam4a.CPU_AFFINITY_WIDTH, cpu_count))
    if workers * width > cpu_count:
        raise ValueError(
            f"workers={workers} require {workers * width} CPUs, only {cpu_count} available"
        )

    def launch(index: int) -> None:
        job = dict(jobs[index])
        used_slots = {int(entry["slot"]) for entry in pending.values()}
        slot = next(slot for slot in range(workers) if slot not in used_slots)
        affinity = _cpu_affinity_for_slot(available_cpus, slot, width)
        job["genesis_cache_path"] = str(
            cache_root / f"rank_{index + 1:04d}" if PRIVATE_GENESIS_CACHES else shared_cache
        )
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
        proc = subprocess.Popen(
            _child_launch_command(genesis_python, job_path, result_path),
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
            cwd=str(REPO),
        )
        pending[index] = {
            "proc": proc,
            "result_path": result_path,
            "deadline": time.time() + timeout_seconds,
            "slot": slot,
        }

    def finalize(index: int, entry: dict[str, Any], reason: str | None) -> None:
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
        print(
            f"[{len(results)}/{total}] {jobs[index]['dataset_id']} "
            f"{record.get('status')} load={record.get('load_time_seconds')}",
            flush=True,
        )

    if total > 0 and workers > 1:
        # Warmup: run the first rank alone so the shared Genesis cache
        # (taichi kernel cache etc.) is warm before parallel children start.
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
                        proc.wait(timeout=10.0)
                    del pending[windex]
                    finalize(windex, entry, f"asset_timeout after {timeout_seconds}s")
        if PRIVATE_GENESIS_CACHES and template_cache["path"] is None:
            template_cache["path"] = str(cache_root / "rank_0001")
    while len(results) < total:
        launched_now = False
        while len(pending) < workers and next_index < total:
            launch(next_index)
            next_index += 1
            launched_now = True
        if launched_now and PRIVATE_GENESIS_CACHES:
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
                    proc.wait(timeout=10.0)
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
            bucket["joints_intended"] += 1
            joints_passed += int(bool(joint["full_range_cf_pass"]))
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
    manifest: Mapping[str, Any],
    records: Sequence[Mapping[str, Any]],
    aggregates: Mapping[str, Any],
    table4_strict: Mapping[str, bool],
    category_info: Mapping[str, Any],
) -> dict[str, Any]:
    checks: list[dict[str, Any]] = []

    def check(name: str, ok: bool, detail: str = "") -> None:
        checks.append({"check": name, "pass": bool(ok), "detail": detail})

    items = manifest["items"]
    check(
        "source_manifest_file_sha256",
        lam4a.sha256_file(SOURCE_MANIFEST) == EXPECTED_SOURCE_MANIFEST_FILE_SHA256,
        EXPECTED_SOURCE_MANIFEST_FILE_SHA256,
    )
    ids = [str(item["dataset_id"]) for item in items]
    ordered = sha256_bytes(json.dumps(ids, separators=(",", ":"), ensure_ascii=True).encode())
    check("ordered_ids_sha256", ordered == EXPECTED_ORDERED_IDS_SHA256, ordered)
    check(
        "table2_cohort_file_sha256",
        lam4a.sha256_file(TABLE2_COHORT_MANIFEST) == EXPECTED_TABLE2_COHORT_FILE_SHA256,
        EXPECTED_TABLE2_COHORT_FILE_SHA256,
    )
    t2 = json.loads(TABLE2_COHORT_MANIFEST.read_text(encoding="utf-8"))
    t2_asset_ids = [str(r["asset_id"]) for r in t2["records"]]
    t4_asset_ids = [str(item["asset_id"]) for item in items]
    check("cohort_is_exact_table2_sample", t2_asset_ids == t4_asset_ids)
    check("record_count", len(records) == N_EVAL, f"{len(records)}")
    order_ok = all(
        int(r["selection_index"]) == i and str(r["dataset_id"]) == str(items[i]["dataset_id"])
        for i, r in enumerate(records)
    )
    check("frozen_order_preserved", order_ok)
    intended = aggregates["joint_level_full_range_cf"]["denominator"]
    check("j_eval_denominator", intended == J_EVAL, f"{intended}")
    retention_den = aggregates["collision_safe_dof_retention"]["denominator"]
    check("retention_denominator", retention_den == J_EVAL, f"{retention_den}")
    sha_ok = all(
        (r.get("urdf_sha256") is None and r.get("status") != "completed")
        or r.get("urdf_sha256") == str(items[int(r["selection_index"])]["urdf_sha256"])
        for r in records
    )
    check("urdf_identity_matches_frozen_manifest", sha_ok)
    cross = aggregates["state_counts"]["hash_cross_check"]
    check("state_hash_cross_check_no_mismatch", cross["mismatch"] == 0, f"{cross}")
    states_intended = aggregates["state_counts"]["intended"]
    check("state_intended_count", states_intended == SINGLE_SAMPLES * J_EVAL, f"{states_intended}")
    check(
        "category_mapping_sha256",
        category_info.get("category_mapping_sha256") == EXPECTED_CATEGORY_MAPPING_SHA256,
        str(category_info.get("category_mapping_sha256")),
    )
    check(
        "category_records_revision",
        category_info.get("category_records_revision") == EXPECTED_CATEGORY_RECORDS_REVISION,
        str(category_info.get("category_records_revision")),
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
        f"# Table 4a - {DATASET} (frozen cohort, N={summary['cohort']['n_eval']}; Genesis contact-penetration oracle)",
        "",
        f"- Protocol ID: `{summary['protocol_id']}` (engine `{summary['engine_protocol_id']}`)",
        f"- Run directory: `{summary['run_directory']}`",
        f"- N_eval = {summary['cohort']['n_eval']}, J_eval = {summary['cohort']['j_eval']}",
        f"- Status: completed = {summary['status_counts']['completed']}, error = {summary['status_counts']['error']}",
        "",
        "| Dataset / Outputs | Joint-level Full-range CF ↑ | Executable CF DoF/Asset ↑ | Collision-safe DoF Retention ↑ | Normalized Clearance P5 ↑ | Limit Reachability ↑ |",
        "|---|---:|---:|---:|---:|---:|",
        (
            f"| {DATASET} "
            f"| {cf['numerator']} / {cf['denominator']} ({cf['percent']:.4f}%) "
            f"| mean {exe['mean']} / median {exe['median']} / P90 {exe['p90']} "
            f"| {ret['numerator']} / {ret['denominator']} ({ret['percent']:.4f}%) "
            f"| N/E ({p5['coverage_status']}; states {p5['state_coverage']['numerator']} / {p5['state_coverage']['denominator']}) "
            f"| {lim['numerator']} / {lim['denominator']} ({lim['percent']:.4f}%; continuous excluded: {lim['continuous_excluded']}) |"
        ),
        "",
        "## DoF bins (declared movable DoF; existing Strict Collision Pass from Table 4)",
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
    global EXPECTED_EARLY_CPU_AFFINITY_LAUNCHER_RECEIPT
    mode = args.mode
    formal = mode == "formal"
    timestamp = _dt.datetime.now(_dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    if args.output_dir is not None:
        outdir = args.output_dir.resolve()
    elif formal:
        outdir = REPO / f"exp/runtime/table4a_urdf_articraft10k_table2cohort_n800_seed20260813_{timestamp}"
    else:
        outdir = REPO / f"exp/runtime/table4a_urdf_articraft10k_smoke_n{args.n}_{timestamp}"
    if outdir.exists():
        raise SystemExit(f"output directory already exists: {outdir}")
    outdir.mkdir(parents=True)
    early_affinity_launcher = early_cpu_affinity_launcher_receipt()
    EXPECTED_EARLY_CPU_AFFINITY_LAUNCHER_RECEIPT = early_affinity_launcher

    manifest = load_source_manifest()
    load_table2_cohort_identity()
    table3_pass, table3_joint_total = load_table3_joint_pass()
    if table3_joint_total != J_EVAL:
        raise SystemExit(f"Table 3 joint count {table3_joint_total} != J_EVAL {J_EVAL}")
    table4_strict = load_table4_strict_pass()
    state_hashes = load_table4_state_hashes()
    jobs, category_info = build_jobs(manifest, table3_pass, state_hashes, formal=formal)
    if formal and sum(job["expected_movable_dof"] for job in jobs) != J_EVAL:
        raise SystemExit("formal cohort J_eval mismatch")
    if mode == "smoke":
        jobs = jobs[: args.n]

    protocol_snapshot_text = PROTOCOL_DOCUMENT.read_text(encoding="utf-8")
    atomic_write_text(outdir / "protocol_snapshot.md", protocol_snapshot_text)

    workers = WORKERS if args.workers is None else args.workers
    frozen_config = {
        "protocol_id": PROTOCOL_ID,
        "engine_protocol_id": ENGINE_PROTOCOL_ID,
        "frozen_at_utc": utc_now_iso(),
        "mode": mode,
        "dataset": DATASET,
        "classification": CLASSIFICATION if formal else "SMOKE",
        "protocol_document_sha256": sha256_bytes(protocol_snapshot_text.encode("utf-8")),
        "cohort": {
            "dataset_root": str(DATASET_ROOT),
            "source_manifest": str(SOURCE_MANIFEST),
            "source_manifest_file_sha256": EXPECTED_SOURCE_MANIFEST_FILE_SHA256,
            "source_manifest_content_sha256": EXPECTED_SOURCE_MANIFEST_CONTENT_SHA256,
            "ordered_ids_sha256": EXPECTED_ORDERED_IDS_SHA256,
            "table2_cohort_manifest": str(TABLE2_COHORT_MANIFEST),
            "table2_cohort_file_sha256": EXPECTED_TABLE2_COHORT_FILE_SHA256,
            "table2_cohort_content_sha256": EXPECTED_TABLE2_COHORT_CONTENT_SHA256,
            "n_eval": len(jobs),
            "j_eval": sum(job["expected_movable_dof"] for job in jobs),
            "selection_policy": SELECTION_POLICY,
            **category_info,
        },
        "metric_rules": METRIC_RULES,
        "operationalization": OPERATIONALIZATION,
        "execution": {
            "child_timeout_seconds": CHILD_TIMEOUT_SECONDS,
            "workers": workers,
            "child_interpreter": str(lam4a.DEFAULT_GENESIS_PYTHON.resolve(strict=True)),
            "early_cpu_affinity_launcher": early_affinity_launcher,
            "child_cpu_affinity_width": lam4a.CPU_AFFINITY_WIDTH,
            "genesis_cache_policy": GENESIS_CACHE_POLICY,
            "denominator_policy": "all frozen assets, joints and states including failures and unexecuted items",
            "dof_bins": [label for label, _, _ in DOF_BINS] + ["unknown/unparseable"],
            "table3_source": {"records": str(TABLE3_RECORDS), "sha256": EXPECTED_TABLE3_RECORDS_SHA256},
            "table4_state_records": {"path": str(TABLE4_STATE_RECORDS), "sha256": EXPECTED_TABLE4_STATE_RECORDS_SHA256},
            "table4_asset_records": {"path": str(TABLE4_ASSET_RECORDS), "sha256": EXPECTED_TABLE4_ASSET_RECORDS_SHA256},
        },
        "runner_identity": {
            "runner_path": str(SCRIPT),
            "runner_sha256": lam4a.sha256_file(SCRIPT),
            "lam_supplementary_runner_path": str(lam4a.SCRIPT),
            "lam_supplementary_runner_sha256": lam4a.sha256_file(lam4a.SCRIPT),
            "static_atoms_path": str(SCRIPT.with_name("lam_supplementary_static.py")),
            "static_atoms_sha256": lam4a.sha256_file(SCRIPT.with_name("lam_supplementary_static.py")),
        },
    }
    frozen_config_sha256 = lam4a.canonical_sha256(frozen_config)
    frozen_config["frozen_config_sha256"] = frozen_config_sha256
    atomic_write_json(outdir / "frozen_config.json", frozen_config)

    started_at = utc_now_iso()
    t0 = time.monotonic()
    records = spawn_children(
        jobs, outdir, workers=workers, timeout_seconds=CHILD_TIMEOUT_SECONDS
    )
    wall_seconds = round(time.monotonic() - t0, 3)

    asset_lines = [json.dumps(r, sort_keys=True, ensure_ascii=True, allow_nan=False) for r in records]
    atomic_write_text(outdir / "asset_records.jsonl", "\n".join(asset_lines) + "\n")
    joint_rows = []
    for record in records:
        for joint in record.get("joint_records", []):
            joint_rows.append(
                {
                    "dataset_id": record["dataset_id"],
                    "asset_id": record.get("asset_id"),
                    "category": record.get("category"),
                    "selection_index": record["selection_index"],
                    "asset_status": record.get("status"),
                    **{k: v for k, v in joint.items() if k != "state_summaries"},
                }
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
        "classification": CLASSIFICATION if formal else "SMOKE",
        "run_directory": str(outdir),
        "started_at_utc": started_at,
        "completed_at_utc": completed_at,
        "wall_seconds": wall_seconds,
        "cohort": {
            "source_manifest": str(SOURCE_MANIFEST),
            "source_manifest_file_sha256": EXPECTED_SOURCE_MANIFEST_FILE_SHA256,
            "ordered_ids_sha256": EXPECTED_ORDERED_IDS_SHA256,
            "n_eval": len(jobs),
            "j_eval": sum(job["expected_movable_dof"] for job in jobs),
        },
        "status_counts": aggregates["status_counts"],
        "metrics": aggregates,
        "frozen_config_sha256": frozen_config_sha256,
        "notes": RUN_NOTES,
    }
    atomic_write_json(outdir / "summary.json", summary)
    atomic_write_text(outdir / "summary.md", render_summary_md(summary))

    verification = (
        verify_run(manifest, records, aggregates, table4_strict, category_info)
        if formal
        else {"all_pass": None, "note": "smoke mode: formal verification skipped"}
    )
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
    if formal and not verification["all_pass"]:
        return 2
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("smoke", "formal"), default=None)
    parser.add_argument("--n", type=int, default=3, help="smoke sample size (smoke mode only)")
    parser.add_argument("--workers", type=int, default=None)
    parser.add_argument("--output-dir", type=Path, default=None)
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
