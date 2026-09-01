#!/usr/bin/env python3
"""Fail-closed Table 2 supplementary runner for PartNet-Mobility.

Runs the four proposed Table 2 supplementary metrics
(Visual-bearing Collision Coverage, Joint-limit Portability,
Joint Dynamics Coverage, Placeholder-mass Incidence) over the frozen
Table 4 PartNet-Mobility N=800 cohort, reusing the frozen static atom
implementations from ``lam_supplementary_static`` so that all methods are
scored by the same metric code version.

This module performs no metric tuning.  All thresholds, the placeholder
registry, the per-type joint mapping and the cohort identity are frozen in
``frozen_config.json`` before any asset is evaluated.
"""

from __future__ import annotations

import argparse
import datetime as _dt
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import shutil
import signal
import subprocess
import sys
import time
from typing import Any, Mapping, Sequence
import xml.etree.ElementTree as ET

SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from exp.scripts import lam_supplementary_static as static  # noqa: E402

SCHEMA_VERSION = "table2sup-partnet-mobility/v1"
PROTOCOL_ID = "table2_supplementary_partnet_mobility_table4cohort_n800_v1"
DATASET = "PartNet-Mobility"
CLASSIFICATION = "FORMAL"

SOURCE_MANIFEST = REPO / "exp/runtime/urdf_table4_partnet_mobility_n800_20260813/frozen_manifest.json"
EXPECTED_SOURCE_MANIFEST_SHA256 = "2ff015ee6bb377ce693126b52dd632a7565a3eaa9f0007e26122a1bb4ab99900"
EXPECTED_ORDERED_IDS_SHA256 = "ef6cb964e50dc712280256c5b2f675cc2c957095c3553b21845d3562a5011883"
TABLE3_RECORDS = REPO / "exp/runtime/urdf_table3_partnet_mobility_table4_n800_20260814T070118Z/asset_records.jsonl"
TABLE3_J_EVAL = 4078
PROTOCOL_DOCUMENT = REPO / "exp/URDF-Sim-Ready-Automatic-Evaluation.md"
URDF_RELATIVE_PATH = "mobility.urdf"

N_EVAL = 800
J_EVAL = 4078
EXPECTED_CATEGORY_COUNT = 46
ASSET_TIMEOUT_SECONDS = 120.0
WORKERS = 4
CHILD_THREAD_ENV = {
    "MKL_NUM_THREADS": "1",
    "NUMEXPR_NUM_THREADS": "1",
    "OMP_NUM_THREADS": "1",
    "OPENBLAS_NUM_THREADS": "1",
}
# Frozen pre-registered placeholder template registry.  Empty by design: this
# matches the frozen LAM supplementary precedent (no exporter/simulator
# default template has been pre-registered for this comparison), which keeps
# the metric uniform across methods.  Empty registry -> formal N/E cell.
PLACEHOLDER_REGISTRY: list[dict[str, Any]] = []
PLACEHOLDER_REGISTRY_RATIONALE = (
    "Empty registry frozen before any PartNet-Mobility result was inspected. "
    "Identical to the frozen LAM supplementary precedent "
    "(run_urdf_lam_supplementary_v1.py passes placeholder_registry=[]). No "
    "exporter/simulator default template is pre-registered for this "
    "comparison; the cell is therefore N/E with reason placeholder_registry_empty, "
    "reported together with complete-inertial / dynamic-link coverage."
)

METRIC_RULES = {
    "visual_bearing_collision_coverage": (
        "主值为资产级 passed / N_eval：资产必须可解析、至少包含一个在 XML 中声明 <visual> "
        "geometry 的 visual-bearing link，且每个此类 link 都至少包含一个资源可解析、可加载的 "
        "collision geometry；解析失败、visual/collision 资源失败或零 visual-bearing link 均 "
        "fail closed。另补充报告 covered visual-bearing links / L_visual_declared 的 link-micro "
        "值及 link extraction coverage。该指标补充而不替换既有按全部声明 link 计算的 Collision Coverage。"
    ),
    "joint_limit_portability": (
        "关节级 passed / J_eval。bounded revolute/prismatic joint 必须具有有限的 lower < upper、"
        "有限且非负的 effort 和有限且为正的 velocity；continuous joint 不要求有限 lower/upper，"
        "但仍须满足冻结 adapter 共同要求的 effort/velocity 字段。其他 joint type 按查看结果前冻结的 "
        "per-type mapping 处理；缺字段、unsupported mapping 和未执行项均保留为失败。"
    ),
    "joint_dynamics_coverage": (
        "同时声明有限、非负 damping 与 friction 的 movable joints 数除以 J_eval；缺失任一字段计为未覆盖。"
        "该项只衡量字段覆盖，不证明数值经过动力学校准，也不进入既有 Strict URDF Pass。"
    ),
    "placeholder_mass_incidence": (
        "在具有 complete inertial 的动态 link 中，mass 或完整 inertial tuple 命中预注册 "
        "exporter/simulator 默认模板的 link 数除以 complete-inertial link 数，并同时报告 "
        "complete-inertial links / dynamic links coverage。默认模板或 sentinel 只能来自冻结工具默认值"
        "或公开文档，禁止查看方法结果后添加；分母为零时，正式运行后记为 N/E 而不是 0。该项是诊断 flag，"
        "不证明被标记质量一定错误，也不进入既有 Strict URDF Pass。"
    ),
}

OPERATIONALIZATION = {
    "parse_gate": (
        "XML well-formed parse plus root element named robot (frozen precedent: "
        "lam_supplementary_static.audit_lam_package). This is not the urdfpy "
        "standard-parser gate used by Table 2 Parse Rate."
    ),
    "visual_bearing_link_eligibility": (
        "A link is visual-bearing iff it declares at least one <visual> element "
        "containing at least one <geometry> child (frozen atom: _visual_collision_atoms)."
    ),
    "link_covered_rule": (
        "A visual-bearing link is covered iff it declares at least one collision "
        "element whose single geometry child passes the frozen "
        "_collision_geometry_loadable checks (primitive validity or mesh resource "
        "resolution plus byte-level loadability)."
    ),
    "link_extraction_coverage": (
        "link_extraction_complete per asset requires all declared links to be "
        "named and uniquely named; asset_pass additionally requires it (frozen atom)."
    ),
    "portability_per_type_mapping": {
        "revolute": "bounded: exactly one <limit> with finite lower < upper; finite effort >= 0; finite velocity > 0",
        "prismatic": "bounded: exactly one <limit> with finite lower < upper; finite effort >= 0; finite velocity > 0",
        "continuous": "exactly one <limit> element required (cardinality), no finite lower/upper requirement; finite effort >= 0; finite velocity > 0",
        "fixed": "excluded from J_eval",
        "other": "unsupported_mapping fail (planar/floating/missing type retained as failures)",
    },
    "dynamics_rule": (
        "exactly one <dynamics> element with finite damping >= 0 and finite "
        "friction >= 0; missing element or either field counts as uncovered."
    ),
    "dynamic_link_policy": "all_declared_links (frozen precedent)",
    "expected_movable_joints_source": (
        "Per-asset expected denominator = Table 4 frozen manifest "
        "items[].movable_dof_count (identical to len(items[].joint_specs)); "
        "cross-checked at runtime against Table 3 asset_records "
        "declared_joint_count; mismatch fails the asset closed."
    ),
    "urdf_identity_gate": (
        "Per-asset mobility.urdf SHA-256 must equal the Table 4 frozen manifest "
        "items[].urdf_sha256; mismatch fails the asset closed."
    ),
    "placeholder_registry": PLACEHOLDER_REGISTRY_RATIONALE,
}


def utc_now_iso() -> str:
    return _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")


def sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def sha256_file(path: Path) -> str:
    return static.sha256_file(path)


def canonical_sha256(value: Any) -> str:
    return static.canonical_sha256(value)


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


def load_source_manifest() -> dict[str, Any]:
    payload = SOURCE_MANIFEST.read_bytes()
    digest = sha256_bytes(payload)
    if digest != EXPECTED_SOURCE_MANIFEST_SHA256:
        raise SystemExit(
            f"source manifest sha256 mismatch: got {digest}, expected {EXPECTED_SOURCE_MANIFEST_SHA256}"
        )
    manifest = json.loads(payload)
    items = manifest.get("items")
    if not isinstance(items, list) or len(items) != N_EVAL:
        raise SystemExit(f"source manifest must contain exactly {N_EVAL} items")
    ids = [item["dataset_id"] for item in items]
    ordered_digest = sha256_bytes(
        json.dumps(ids, separators=(",", ":"), ensure_ascii=True).encode("utf-8")
    )
    if ordered_digest != EXPECTED_ORDERED_IDS_SHA256:
        raise SystemExit(
            f"ordered asset id sha256 mismatch: got {ordered_digest}, expected {EXPECTED_ORDERED_IDS_SHA256}"
        )
    if manifest.get("dataset_root") != str(REPO / "exp/PartNet-Mobility/data/dataset"):
        raise SystemExit("unexpected dataset_root in source manifest")
    for index, item in enumerate(items):
        if item.get("order") != index:
            raise SystemExit(f"item order field mismatch at index {index}")
    return manifest


def load_table3_joint_counts() -> dict[str, int]:
    counts: dict[str, int] = {}
    with TABLE3_RECORDS.open("r", encoding="utf-8") as handle:
        for line in handle:
            record = json.loads(line)
            counts[str(record["dataset_id"])] = int(record["declared_joint_count"])
    return counts


def environment_identity() -> dict[str, Any]:
    identity: dict[str, Any] = {
        "python_executable": sys.executable,
        "python_version": platform.python_version(),
        "platform": platform.platform(),
        "runner_script": str(SCRIPT),
        "runner_script_sha256": sha256_file(SCRIPT),
        "static_module": str(Path(static.__file__).resolve()),
        "static_module_sha256": sha256_file(Path(static.__file__).resolve()),
    }
    try:
        import numpy as np

        identity["numpy_version"] = np.__version__
    except Exception:  # noqa: BLE001
        identity["numpy_version"] = None
    try:
        import trimesh

        identity["trimesh_version"] = trimesh.__version__
    except Exception:  # noqa: BLE001
        identity["trimesh_version"] = None
    return identity


def _failed_record(
    *,
    job: Mapping[str, Any],
    issue: str,
) -> dict[str, Any]:
    intended = int(job["expected_movable_joints"])
    return {
        "schema_version": SCHEMA_VERSION,
        "selection_index": int(job["selection_index"]),
        "asset_id": str(job["dataset_id"]),
        "dataset_id": str(job["dataset_id"]),
        "category": str(job["category"]),
        "package": str(job["package"]),
        "urdf_relative_path": URDF_RELATIVE_PATH,
        "urdf_sha256": None,
        "expected_urdf_sha256": str(job["expected_urdf_sha256"]),
        "expected_movable_joints": intended,
        "table3_declared_joint_count": job.get("table3_declared_joint_count"),
        "status": "error",
        "parse": {"success": False, "issues": [issue]},
        "table2_supplementary": {
            "visual_bearing_collision_coverage": {
                "status": "NOT_EVALUABLE",
                "asset_intended": 1,
                "asset_passed": 0,
                "asset_pass": False,
                "visual_bearing_links_declared": 0,
                "covered_visual_bearing_links": 0,
                "link_extraction_complete": False,
                "collision_elements_declared_on_visual_links": 0,
                "loadable_collision_elements_on_visual_links": 0,
                "link_records": [],
                "issues": [issue],
            },
            "joint_limit_portability": {
                "status": "NOT_EVALUABLE",
                "joints_intended": intended,
                "joints_extracted": 0,
                "joints_passed": 0,
                "extraction_complete": False,
                "joint_records": [],
                "issues": [issue],
            },
            "joint_dynamics_coverage": {
                "status": "NOT_EVALUABLE",
                "joints_intended": intended,
                "joints_extracted": 0,
                "joints_covered": 0,
                "extraction_complete": False,
                "joint_records": [],
                "issues": [issue],
            },
            "placeholder_mass_incidence": {
                "status": "N/E",
                "dynamic_link_policy": "all_declared_links",
                "dynamic_links": 0,
                "complete_inertial_links": 0,
                "complete_inertial_coverage_numerator": 0,
                "complete_inertial_coverage_denominator": 0,
                "classified_complete_inertial_links": 0,
                "unclassified_complete_inertial_links": 0,
                "placeholder_links": None,
                "incidence_numerator": None,
                "incidence_denominator": 0,
                "registry_ids": [],
                "link_records": [],
                "incomplete_inertial_links": [],
                "issues": [issue],
            },
        },
        "resource_closure": {
            "status": "NOT_EVALUABLE",
            "complete": False,
            "file_count": 0,
            "sha256": None,
            "files": [],
            "issues": [issue],
        },
        "issues": [issue],
    }


def audit_partnet_mobility_asset(job: Mapping[str, Any]) -> dict[str, Any]:
    """Fail-closed Table 2 supplementary audit for one PartNet-Mobility asset."""

    package_raw = Path(str(job["package"]))
    dataset_id = str(job["dataset_id"])
    intended = int(job["expected_movable_joints"])
    try:
        registry = static.normalize_placeholder_registry(PLACEHOLDER_REGISTRY)
        if package_raw.is_symlink():
            raise ValueError("package_is_symlink")
        package = package_raw.resolve(strict=True)
        if not package.is_dir():
            raise NotADirectoryError(str(package))
        urdf_path = static._primary_urdf(package, URDF_RELATIVE_PATH)
        urdf_sha256 = static.sha256_file(urdf_path)
        if urdf_sha256 != str(job["expected_urdf_sha256"]):
            raise ValueError(
                f"urdf_sha256_mismatch: got {urdf_sha256}, "
                f"expected {job['expected_urdf_sha256']}"
            )
        t3 = job.get("table3_declared_joint_count")
        if t3 is not None and int(t3) != intended:
            raise ValueError(
                f"table3_joint_count_mismatch: table3={t3}, frozen_manifest={intended}"
            )
    except Exception as exc:  # noqa: BLE001
        record = _failed_record(job=job, issue=f"preflight_failed: {type(exc).__name__}: {exc}")
        return record

    try:
        root = ET.parse(urdf_path).getroot()
        if static.local_tag(root) != "robot":
            raise ValueError(f"root_element_not_robot: {static.local_tag(root)}")
    except Exception as exc:  # noqa: BLE001
        record = _failed_record(job=job, issue=f"xml_parse_failed: {type(exc).__name__}: {exc}")
        record["urdf_sha256"] = urdf_sha256
        return record

    link_names = [link.attrib.get("name", "").strip() for link in static.children(root, "link")]
    link_issues: list[str] = []
    if not link_names:
        link_issues.append("no_declared_links")
    if any(not name for name in link_names):
        link_issues.append("unnamed_link")
    if len(set(link_names)) != len(link_names):
        link_issues.append("duplicate_link_name")
    link_extraction_complete = not link_issues

    visual_collision = static._visual_collision_atoms(
        root, package, urdf_path, link_extraction_complete=link_extraction_complete
    )
    joint_limit, joint_dynamics, joint_extraction_issues = static._joint_atoms(
        root, expected_movable_joints=intended
    )
    placeholder = static._placeholder_mass_atoms(root, registry)
    resource_closure = static._resource_closure(root, package, urdf_path)

    issues = [*link_issues, *joint_extraction_issues, *resource_closure["issues"]]
    return {
        "schema_version": SCHEMA_VERSION,
        "selection_index": int(job["selection_index"]),
        "asset_id": dataset_id,
        "dataset_id": dataset_id,
        "category": str(job["category"]),
        "package": str(package),
        "urdf_relative_path": URDF_RELATIVE_PATH,
        "urdf_sha256": urdf_sha256,
        "expected_urdf_sha256": str(job["expected_urdf_sha256"]),
        "expected_movable_joints": intended,
        "table3_declared_joint_count": job.get("table3_declared_joint_count"),
        "status": "completed",
        "parse": {"success": True, "issues": []},
        "table2_supplementary": {
            "visual_bearing_collision_coverage": visual_collision,
            "joint_limit_portability": joint_limit,
            "joint_dynamics_coverage": joint_dynamics,
            "placeholder_mass_incidence": placeholder,
        },
        "resource_closure": resource_closure,
        "issues": issues,
    }


def run_child(job_path: Path, result_path: Path) -> int:
    started = time.time()
    job = json.loads(job_path.read_text(encoding="utf-8"))
    try:
        record = audit_partnet_mobility_asset(job)
    except Exception as exc:  # noqa: BLE001 - defense in depth; fail closed
        record = _failed_record(
            job=job, issue=f"unexpected_child_failure: {type(exc).__name__}: {exc}"
        )
    record["child"] = {
        "started_at_utc": utc_now_iso(),
        "duration_seconds": round(time.time() - started, 6),
        "python_executable": sys.executable,
        "python_version": platform.python_version(),
    }
    atomic_write_json(result_path, record)
    return 0


def slim_record(record: Mapping[str, Any]) -> dict[str, Any]:
    t2 = record.get("table2_supplementary", {})
    vb = t2.get("visual_bearing_collision_coverage", {})
    port = t2.get("joint_limit_portability", {})
    dyn = t2.get("joint_dynamics_coverage", {})
    ph = t2.get("placeholder_mass_incidence", {})
    rc = record.get("resource_closure", {})
    return {
        "schema_version": record.get("schema_version"),
        "selection_index": record.get("selection_index"),
        "asset_id": record.get("asset_id"),
        "dataset_id": record.get("dataset_id"),
        "category": record.get("category"),
        "package": record.get("package"),
        "urdf_relative_path": record.get("urdf_relative_path"),
        "urdf_sha256": record.get("urdf_sha256"),
        "expected_urdf_sha256": record.get("expected_urdf_sha256"),
        "urdf_sha256_matches_frozen": bool(
            record.get("urdf_sha256") is not None
            and record.get("urdf_sha256") == record.get("expected_urdf_sha256")
        ),
        "expected_movable_joints": record.get("expected_movable_joints"),
        "table3_declared_joint_count": record.get("table3_declared_joint_count"),
        "status": record.get("status"),
        "parse_success": bool(record.get("parse", {}).get("success", False)),
        "visual_bearing_collision_coverage": {
            "status": vb.get("status"),
            "asset_pass": bool(vb.get("asset_pass", False)),
            "visual_bearing_links_declared": int(vb.get("visual_bearing_links_declared", 0)),
            "covered_visual_bearing_links": int(vb.get("covered_visual_bearing_links", 0)),
            "link_extraction_complete": bool(vb.get("link_extraction_complete", False)),
            "collision_elements_declared_on_visual_links": int(
                vb.get("collision_elements_declared_on_visual_links", 0)
            ),
            "loadable_collision_elements_on_visual_links": int(
                vb.get("loadable_collision_elements_on_visual_links", 0)
            ),
        },
        "joint_limit_portability": {
            "status": port.get("status"),
            "joints_intended": int(port.get("joints_intended", 0)),
            "joints_extracted": int(port.get("joints_extracted", 0)),
            "joints_passed": int(port.get("joints_passed", 0)),
            "extraction_complete": bool(port.get("extraction_complete", False)),
        },
        "joint_dynamics_coverage": {
            "status": dyn.get("status"),
            "joints_intended": int(dyn.get("joints_intended", 0)),
            "joints_extracted": int(dyn.get("joints_extracted", 0)),
            "joints_covered": int(dyn.get("joints_covered", 0)),
            "extraction_complete": bool(dyn.get("extraction_complete", False)),
        },
        "placeholder_mass_incidence": {
            "status": ph.get("status"),
            "dynamic_links": int(ph.get("dynamic_links", 0)),
            "complete_inertial_links": int(ph.get("complete_inertial_links", 0)),
            "placeholder_links": ph.get("placeholder_links"),
            "incidence_numerator": ph.get("incidence_numerator"),
            "incidence_denominator": ph.get("incidence_denominator"),
            "registry_ids": ph.get("registry_ids", []),
        },
        "resource_closure": {
            "status": rc.get("status"),
            "complete": bool(rc.get("complete", False)),
            "file_count": int(rc.get("file_count", 0)),
            "sha256": rc.get("sha256"),
        },
        "issues": list(record.get("issues", [])),
        "child_duration_seconds": (record.get("child") or {}).get("duration_seconds"),
    }


def build_jobs(manifest: Mapping[str, Any], table3_counts: Mapping[str, int]) -> list[dict[str, Any]]:
    dataset_root = Path(str(manifest["dataset_root"]))
    jobs: list[dict[str, Any]] = []
    for index, item in enumerate(manifest["items"]):
        dataset_id = str(item["dataset_id"])
        jobs.append(
            {
                "selection_index": index,
                "dataset_id": dataset_id,
                "category": str(item.get("category")),
                "package": str(dataset_root / dataset_id),
                "expected_movable_joints": int(item["movable_dof_count"]),
                "expected_urdf_sha256": str(item["urdf_sha256"]),
                "table3_declared_joint_count": table3_counts.get(dataset_id),
                "frozen_joint_spec_count": len(item.get("joint_specs", [])),
            }
        )
    return jobs


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
    results: list[dict[str, Any]] = []
    env = dict(os.environ)
    env.update(CHILD_THREAD_ENV)
    pending: dict[int, dict[str, Any]] = {}
    next_index = 0
    total = len(jobs)

    def launch(index: int) -> None:
        job = jobs[index]
        job_path = job_dir / f"rank_{index + 1:04d}.json"
        result_path = child_dir / f"rank_{index + 1:04d}.json"
        atomic_write_json(job_path, dict(job))
        proc = subprocess.Popen(
            [sys.executable, str(SCRIPT), "--child", "--job", str(job_path), "--result", str(result_path)],
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        pending[index] = {
            "proc": proc,
            "result_path": result_path,
            "deadline": time.time() + timeout_seconds,
        }

    def finalize(index: int, entry: dict[str, Any], reason: str | None) -> None:
        result_path: Path = entry["result_path"]
        record: dict[str, Any] | None = None
        if result_path.is_file():
            try:
                record = json.loads(result_path.read_text(encoding="utf-8"))
            except Exception:  # noqa: BLE001
                record = None
        if record is None:
            record = _failed_record(
                job=jobs[index],
                issue=(reason or "child_result_missing"),
            )
            atomic_write_json(result_path, record)
        results.append(record)

    while len(results) < total:
        while len(pending) < workers and next_index < total:
            launch(next_index)
            next_index += 1
        time.sleep(0.05)
        for index in sorted(pending):
            entry = pending[index]
            proc = entry["proc"]
            rc = proc.poll()
            if rc is not None:
                del pending[index]
                reason = None if rc == 0 else f"child_process_failed: rc={rc}"
                finalize(index, entry, reason)
            elif time.time() >= entry["deadline"]:
                try:
                    os.killpg(proc.pid, signal.SIGTERM)
                except OSError:
                    pass
                try:
                    proc.wait(timeout=0.2)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(proc.pid, signal.SIGKILL)
                    except OSError:
                        pass
                    proc.wait(timeout=5.0)
                del pending[index]
                finalize(index, entry, "asset_timeout")
    results.sort(key=lambda record: int(record["selection_index"]))
    return results


def rate(numerator: int, denominator: int) -> dict[str, Any]:
    return {
        "numerator": int(numerator),
        "denominator": int(denominator),
        "percent": round(100.0 * numerator / denominator, 2) if denominator else None,
    }


def aggregate(records: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    n = len(records)
    completed = sum(1 for r in records if r.get("status") == "completed")
    errors = n - completed
    parse_passed = sum(1 for r in records if r.get("parse", {}).get("success"))
    vb_assets_passed = 0
    vb_declared_total = 0
    vb_covered_total = 0
    vb_link_extraction_complete_assets = 0
    vb_zero_declared_assets = 0
    port_intended = 0
    port_extracted = 0
    port_passed = 0
    dyn_intended = 0
    dyn_extracted = 0
    dyn_covered = 0
    dynamic_links_total = 0
    complete_inertial_total = 0
    for record in records:
        t2 = record.get("table2_supplementary", {})
        vb = t2.get("visual_bearing_collision_coverage", {})
        port = t2.get("joint_limit_portability", {})
        dyn = t2.get("joint_dynamics_coverage", {})
        ph = t2.get("placeholder_mass_incidence", {})
        if vb.get("asset_pass"):
            vb_assets_passed += 1
        vb_declared_total += int(vb.get("visual_bearing_links_declared", 0))
        vb_covered_total += int(vb.get("covered_visual_bearing_links", 0))
        if vb.get("link_extraction_complete"):
            vb_link_extraction_complete_assets += 1
        if record.get("status") == "completed" and int(vb.get("visual_bearing_links_declared", 0)) == 0:
            vb_zero_declared_assets += 1
        port_intended += int(port.get("joints_intended", 0))
        port_extracted += int(port.get("joints_extracted", 0))
        port_passed += int(port.get("joints_passed", 0))
        dyn_intended += int(dyn.get("joints_intended", 0))
        dyn_extracted += int(dyn.get("joints_extracted", 0))
        dyn_covered += int(dyn.get("joints_covered", 0))
        dynamic_links_total += int(ph.get("dynamic_links", 0))
        complete_inertial_total += int(ph.get("complete_inertial_links", 0))

    categories: dict[str, dict[str, Any]] = {}
    for record in records:
        category = str(record.get("category"))
        bucket = categories.setdefault(
            category,
            {
                "assets": 0,
                "vb_passed": 0,
                "port_intended": 0,
                "port_passed": 0,
                "dyn_intended": 0,
                "dyn_covered": 0,
            },
        )
        t2 = record.get("table2_supplementary", {})
        bucket["assets"] += 1
        if t2.get("visual_bearing_collision_coverage", {}).get("asset_pass"):
            bucket["vb_passed"] += 1
        bucket["port_intended"] += int(t2.get("joint_limit_portability", {}).get("joints_intended", 0))
        bucket["port_passed"] += int(t2.get("joint_limit_portability", {}).get("joints_passed", 0))
        bucket["dyn_intended"] += int(t2.get("joint_dynamics_coverage", {}).get("joints_intended", 0))
        bucket["dyn_covered"] += int(t2.get("joint_dynamics_coverage", {}).get("joints_covered", 0))

    def unweighted_mean(values: Sequence[float]) -> float | None:
        return round(100.0 * sum(values) / len(values), 2) if values else None

    vb_macro = unweighted_mean([b["vb_passed"] / b["assets"] for b in categories.values() if b["assets"]])
    port_macro = unweighted_mean(
        [b["port_passed"] / b["port_intended"] for b in categories.values() if b["port_intended"]]
    )
    dyn_macro = unweighted_mean(
        [b["dyn_covered"] / b["dyn_intended"] for b in categories.values() if b["dyn_intended"]]
    )

    return {
        "status_counts": {"completed": completed, "error": errors, "total": n},
        "parse_passed_assets": parse_passed,
        "metrics": {
            "visual_bearing_collision_coverage": {
                "asset": rate(vb_assets_passed, n),
                "link_micro": rate(vb_covered_total, vb_declared_total),
                "link_extraction_complete_assets": vb_link_extraction_complete_assets,
                "zero_visual_bearing_assets_completed": vb_zero_declared_assets,
            },
            "joint_limit_portability": rate(port_passed, port_intended)
            | {"joints_extracted": port_extracted},
            "joint_dynamics_coverage": rate(dyn_covered, dyn_intended)
            | {"joints_extracted": dyn_extracted},
            "placeholder_mass_incidence": {
                "status": "N/E",
                "reason": "placeholder_registry_empty",
                "registry_ids": [],
                "complete_inertial_links": complete_inertial_total,
                "dynamic_links_measured": dynamic_links_total,
                "coverage": rate(complete_inertial_total, dynamic_links_total),
            },
        },
        "category_macro": {
            "category_count": len(categories),
            "visual_bearing_asset_rate_mean_percent": vb_macro,
            "portability_joint_rate_mean_percent": port_macro,
            "dynamics_joint_rate_mean_percent": dyn_macro,
            "categories": {
                name: {
                    "assets": b["assets"],
                    "visual_bearing_asset_rate": rate(b["vb_passed"], b["assets"]),
                    "portability_joint_rate": rate(b["port_passed"], b["port_intended"]),
                    "dynamics_joint_rate": rate(b["dyn_covered"], b["dyn_intended"]),
                }
                for name, b in sorted(categories.items())
            },
        },
    }


def verify_run(
    outdir: Path,
    manifest: Mapping[str, Any],
    records: Sequence[Mapping[str, Any]],
    aggregates: Mapping[str, Any],
) -> dict[str, Any]:
    checks: list[dict[str, Any]] = []

    def check(name: str, ok: bool, detail: str = "") -> None:
        checks.append({"check": name, "pass": bool(ok), "detail": detail})

    items = manifest["items"]
    check(
        "source_manifest_sha256",
        sha256_file(SOURCE_MANIFEST) == EXPECTED_SOURCE_MANIFEST_SHA256,
        EXPECTED_SOURCE_MANIFEST_SHA256,
    )
    ids = [str(item["dataset_id"]) for item in items]
    ordered = sha256_bytes(json.dumps(ids, separators=(",", ":"), ensure_ascii=True).encode("utf-8"))
    check("ordered_ids_sha256", ordered == EXPECTED_ORDERED_IDS_SHA256, ordered)
    check("record_count", len(records) == N_EVAL, f"{len(records)}")
    order_ok = all(
        int(record["selection_index"]) == index
        and str(record["dataset_id"]) == str(items[index]["dataset_id"])
        for index, record in enumerate(records)
    )
    check("frozen_order_preserved", order_ok)
    intended_total = sum(int(r.get("expected_movable_joints", 0)) for r in records)
    check("j_eval_denominator", intended_total == J_EVAL, f"{intended_total}")
    port_den = aggregates["metrics"]["joint_limit_portability"]["denominator"]
    dyn_den = aggregates["metrics"]["joint_dynamics_coverage"]["denominator"]
    check("joint_denominators_preserved", port_den == J_EVAL and dyn_den == J_EVAL, f"{port_den}/{dyn_den}")
    vb_den = aggregates["metrics"]["visual_bearing_collision_coverage"]["asset"]["denominator"]
    check("asset_denominator_preserved", vb_den == N_EVAL, f"{vb_den}")
    sha_ok = all(
        (r.get("urdf_sha256") is None and r.get("status") != "completed")
        or r.get("urdf_sha256") == str(items[int(r["selection_index"])]["urdf_sha256"])
        for r in records
    )
    check("urdf_identity_matches_frozen_manifest", sha_ok)
    completed = sum(1 for r in records if r.get("status") == "completed")
    check("all_assets_completed", completed == N_EVAL, f"completed={completed}")
    recomputed = aggregate(records)
    check(
        "aggregate_recomputation_matches",
        canonical_sha256(recomputed) == canonical_sha256(dict(aggregates)),
    )
    category_count = aggregates["category_macro"]["category_count"]
    check("category_count", category_count == EXPECTED_CATEGORY_COUNT, f"{category_count}")
    all_pass = all(c["pass"] for c in checks)
    return {"all_pass": all_pass, "check_count": len(checks), "checks": checks}


def render_summary_md(summary: Mapping[str, Any]) -> str:
    m = summary["metrics"]
    vb = m["visual_bearing_collision_coverage"]
    port = m["joint_limit_portability"]
    dyn = m["joint_dynamics_coverage"]
    ph = m["placeholder_mass_incidence"]
    macro = summary["category_macro"]
    lines = [
        "# Table 2 supplementary — PartNet-Mobility (frozen Table 4 cohort, N=800)",
        "",
        f"- Protocol ID: `{summary['protocol_id']}`",
        f"- Run directory: `{summary['run_directory']}`",
        f"- Cohort source: `{summary['cohort']['source_manifest']}` (SHA256 `{summary['cohort']['source_manifest_sha256']}`)",
        f"- N_eval = {summary['cohort']['n_eval']}, J_eval = {summary['cohort']['j_eval']}, observed categories = {macro['category_count']}",
        f"- Status: completed = {summary['status_counts']['completed']}, error = {summary['status_counts']['error']}",
        "",
        "## Overall micro averages",
        "",
        "| Dataset / Outputs | Visual-bearing Collision Coverage ↑ | Joint-limit Portability ↑ | Joint Dynamics Coverage ↑ | Placeholder-mass Incidence ↓ |",
        "|---|---:|---:|---:|---:|",
        (
            f"| PartNet-Mobility "
            f"| {vb['asset']['numerator']} / {vb['asset']['denominator']} ({vb['asset']['percent']:.2f}%) "
            f"| {port['numerator']} / {port['denominator']} ({port['percent']:.2f}%) "
            f"| {dyn['numerator']} / {dyn['denominator']} ({dyn['percent']:.2f}%) "
            f"| N/E ({ph['reason']}) |"
        ),
        "",
        "Companion detail:",
        "",
        (
            f"- Visual-bearing link-micro coverage: {vb['link_micro']['numerator']} / {vb['link_micro']['denominator']}"
            f" ({'%.2f%%' % vb['link_micro']['percent'] if vb['link_micro']['percent'] is not None else 'N/A'});"
            f" link extraction complete on {vb['link_extraction_complete_assets']} / {summary['cohort']['n_eval']} assets;"
            f" zero visual-bearing link assets (completed): {vb['zero_visual_bearing_assets_completed']}."
        ),
        (
            f"- Joint extraction: portability extracted {port['joints_extracted']} / intended {port['denominator']};"
            f" dynamics extracted {dyn['joints_extracted']} / intended {dyn['denominator']}."
        ),
        (
            f"- Placeholder-mass: complete-inertial links {ph['complete_inertial_links']} / measured dynamic links "
            f"{ph['dynamic_links_measured']}"
            f" ({'%.2f%%' % ph['coverage']['percent'] if ph['coverage']['percent'] is not None else 'N/A'})."
        ),
        "",
        "## Category macro (unweighted mean of per-category rates)",
        "",
        f"- Visual-bearing asset rate mean: {macro['visual_bearing_asset_rate_mean_percent']}",
        f"- Portability joint rate mean: {macro['portability_joint_rate_mean_percent']}",
        f"- Dynamics joint rate mean: {macro['dynamics_joint_rate_mean_percent']}",
        "",
    ]
    return "\n".join(lines) + "\n"


def run_scope(args: argparse.Namespace) -> int:
    mode = args.mode
    timestamp = _dt.datetime.now(_dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    if mode == "formal":
        outdir = REPO / f"exp/runtime/table2sup_urdf_partnet_mobility_table4cohort_n800_salt20260813_{timestamp}"
    else:
        outdir = REPO / f"exp/runtime/table2sup_urdf_partnet_mobility_smoke_n{args.n}_{timestamp}"
    if outdir.exists():
        raise SystemExit(f"output directory already exists: {outdir}")
    outdir.mkdir(parents=True)

    manifest = load_source_manifest()
    table3_counts = load_table3_joint_counts()
    jobs = build_jobs(manifest, table3_counts)
    if mode == "smoke":
        jobs = jobs[: args.n]

    protocol_snapshot_text = PROTOCOL_DOCUMENT.read_text(encoding="utf-8")
    atomic_write_text(outdir / "protocol_snapshot.md", protocol_snapshot_text)

    frozen_config = {
        "protocol_id": PROTOCOL_ID,
        "frozen_at_utc": utc_now_iso(),
        "mode": mode,
        "dataset": DATASET,
        "classification": CLASSIFICATION if mode == "formal" else "SMOKE",
        "protocol_document": str(PROTOCOL_DOCUMENT),
        "protocol_document_sha256": sha256_bytes(protocol_snapshot_text.encode("utf-8")),
        "cohort": {
            "source_manifest": str(SOURCE_MANIFEST),
            "source_manifest_sha256": EXPECTED_SOURCE_MANIFEST_SHA256,
            "ordered_ids_sha256": EXPECTED_ORDERED_IDS_SHA256,
            "dataset_root": str(manifest["dataset_root"]),
            "n_eval": len(jobs),
            "j_eval": sum(job["expected_movable_joints"] for job in jobs),
            "selection_policy": (
                "all items[].dataset_id from the Table 4 frozen manifest in existing "
                "order; no resampling, replacement or result-based filtering"
            ),
            "items": jobs,
        },
        "asset_resolution": {
            "urdf_relative_path": URDF_RELATIVE_PATH,
            "package_is_asset_directory": True,
        },
        "metric_rules": METRIC_RULES,
        "operationalization": OPERATIONALIZATION,
        "placeholder_registry": PLACEHOLDER_REGISTRY,
        "placeholder_registry_rationale": PLACEHOLDER_REGISTRY_RATIONALE,
        "execution": {
            "asset_timeout_seconds": ASSET_TIMEOUT_SECONDS,
            "workers": WORKERS,
            "child_process": {
                "interpreter": "sys.executable fresh interpreter",
                "job_protocol": "per-job JSON input and atomic per-job JSON result in output-owned directories",
                "start_new_session": True,
                "termination": "SIGTERM owned process group, grace 0.2 seconds, then SIGKILL",
                "thread_environment": CHILD_THREAD_ENV,
            },
            "denominator_policy": "all frozen selected assets and joints, including failures, timeouts and non-executed items",
        },
        "table3_cross_check": {
            "asset_records": str(TABLE3_RECORDS),
            "asset_records_sha256": sha256_file(TABLE3_RECORDS),
            "j_eval": TABLE3_J_EVAL,
        },
        "environment": environment_identity(),
    }
    atomic_write_json(outdir / "frozen_config.json", frozen_config)
    frozen_config_sha256 = sha256_file(outdir / "frozen_config.json")

    started_at = utc_now_iso()
    wall_start = time.time()
    records = spawn_children(
        jobs, outdir, workers=args.workers or WORKERS, timeout_seconds=ASSET_TIMEOUT_SECONDS
    )
    wall_seconds = round(time.time() - wall_start, 3)

    slim = [slim_record(record) for record in records]
    records_lines = "\n".join(
        json.dumps(record, sort_keys=True, ensure_ascii=True, allow_nan=False) for record in slim
    ) + "\n"
    atomic_write_text(outdir / "asset_records.jsonl", records_lines)

    aggregates = aggregate(records)
    completed_at = utc_now_iso()
    summary = {
        "protocol_id": PROTOCOL_ID,
        "mode": mode,
        "dataset": DATASET,
        "classification": CLASSIFICATION if mode == "formal" else "SMOKE",
        "run_directory": str(outdir),
        "started_at_utc": started_at,
        "completed_at_utc": completed_at,
        "wall_seconds": wall_seconds,
        "cohort": {
            "source_manifest": str(SOURCE_MANIFEST),
            "source_manifest_sha256": EXPECTED_SOURCE_MANIFEST_SHA256,
            "ordered_ids_sha256": EXPECTED_ORDERED_IDS_SHA256,
            "n_eval": len(jobs),
            "j_eval": sum(job["expected_movable_joints"] for job in jobs),
        },
        "status_counts": aggregates["status_counts"],
        "parse_passed_assets": aggregates["parse_passed_assets"],
        "metrics": aggregates["metrics"],
        "category_macro": aggregates["category_macro"],
        "frozen_config_sha256": frozen_config_sha256,
        "notes": [
            "Placeholder-mass Incidence is N/E because the frozen placeholder registry is empty (uniform with the LAM supplementary precedent); complete-inertial coverage is reported alongside.",
            "Parse gate for these supplementary metrics is XML well-formedness plus robot root (frozen precedent), not the urdfpy standard parser used by Table 2 Parse Rate.",
            "All denominators are intent-to-evaluate: failures, timeouts and non-executed items remain in the denominator.",
        ],
    }
    atomic_write_json(outdir / "summary.json", summary)
    atomic_write_text(outdir / "summary.md", render_summary_md(summary))

    verification = verify_run(outdir, manifest, records, aggregates) if mode == "formal" else {
        "all_pass": None,
        "note": "smoke mode: formal verification skipped",
    }
    run_manifest = {
        "protocol_id": PROTOCOL_ID,
        "mode": mode,
        "dataset": DATASET,
        "created_at_utc": completed_at,
        "command": sys.argv,
        "frozen_config_sha256": frozen_config_sha256,
        "record_count": len(records),
        "status_counts": aggregates["status_counts"],
        "wall_seconds": wall_seconds,
        "outputs": {
            "asset_records_sha256": sha256_file(outdir / "asset_records.jsonl"),
            "summary_sha256": sha256_file(outdir / "summary.json"),
            "summary_md_sha256": sha256_file(outdir / "summary.md"),
        },
        "verification": verification,
    }
    atomic_write_json(outdir / "manifest.json", run_manifest)
    manifest_self_hash = sha256_file(outdir / "manifest.json")
    run_manifest["manifest_self_sha256_at_write"] = manifest_self_hash
    atomic_write_json(outdir / "manifest.json", run_manifest)

    print(json.dumps(
        {
            "mode": mode,
            "run_directory": str(outdir),
            "status_counts": aggregates["status_counts"],
            "metrics": aggregates["metrics"],
            "category_macro_means": {
                "visual_bearing": aggregates["category_macro"]["visual_bearing_asset_rate_mean_percent"],
                "portability": aggregates["category_macro"]["portability_joint_rate_mean_percent"],
                "dynamics": aggregates["category_macro"]["dynamics_joint_rate_mean_percent"],
            },
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
    parser.add_argument("--n", type=int, default=5, help="smoke sample size (smoke mode only)")
    parser.add_argument("--workers", type=int, default=None)
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
