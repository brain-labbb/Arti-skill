#!/usr/bin/env python3
"""Formal Table 2 supplementary runner for LAM released outputs (static, fail-closed).

Cohort
------
The sample is the frozen Table 3 LAM cohort
(``exp/runtime/urdf_table3_lam_n800_seed20260813_20260814T022400Z_v3``):
all 800 ``manifest.json .records[]`` entries in ``selection_rank = 1..800``
order (the manifest is already stored in that order; the runner re-verifies
it).  The cohort comes from the ``VERIFIED_RELEASE_COMPLETE``
``YipengGao/Articulated-Object-Code@28cec4f5be7e34fd4d586879ecfcb67f7c5e4cc0``
released outputs (``N_release = 3,217``), seed ``20260813``: 621 ``viable``,
75 ``loads_only`` and 104 ``broken`` assets covering 305 observed categories.
There is no resampling, no replacement of failed assets and no outcome-based
selection (identical policy to the formal Table 2 LAM run).

Package binding
---------------
Each asset is bound to ``<source_root>/<rel_path>`` with primary URDF
``generated.urdf`` exactly as in the frozen Table 3 manifest.  Before
evaluation the runner re-verifies every primary URDF SHA-256 against the
frozen Table 3 manifest ``records[].urdf_sha256`` and re-checks the frozen
absolute ``urdf_path`` binding; any mismatch is fail-closed and the asset is
never evaluated.

Denominators
------------
- Asset-level denominator: ``N_eval = 800`` (parse/binding failures retained).
- Joint-level denominator: ``J_eval = 2,395`` taken from the frozen Table 3
  manifest ``records[].declared_joint_count_hint``, cross-checked per asset
  against the Table 3 ``asset_records.jsonl`` ``declared_joint_count``.
  Assets that fail preflight keep their intended joint count as failed
  denominator.
- Placeholder-mass registry: frozen empty.  No LAM exporter/simulator default
  mass or inertia template has been validated from frozen tool defaults or
  public documentation before result inspection, matching the frozen LAM
  supplementary precedent (``run_urdf_lam_supplementary_v1.py`` passes
  ``placeholder_registry=[]``) and the Table 2 supplementary precedent of the
  other methods.  Placeholder-mass Incidence is therefore reported ``N/E``
  together with complete-inertial / dynamic-link coverage.

Evaluator version
-----------------
All metric atoms are imported from ``lam_supplementary_static.py`` (via the
method-agnostic ``table2_supplementary_static.audit_worker`` wrapper).  The
runner records the byte-level SHA-256 of both modules in the frozen manifest
and environment receipt before any asset is evaluated.
"""

from __future__ import annotations

import argparse
import datetime
import hashlib
import importlib.util
import json
import multiprocessing
import os
from pathlib import Path
import platform
import sys
import tempfile
import time
from typing import Any, Mapping

SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

PROTOCOL_ID = "table2_supplementary_lam_table3cohort_n800_seed20260813_v1"
DATASET = "LAM released outputs"
SCHEMA_VERSION = "table2sup-lam/v1"

LAM_SOURCE_ROOT = REPO / "exp/Articulated-Object-Code/released_outputs"
TABLE3_MANIFEST = REPO / "exp/runtime/urdf_table3_lam_n800_seed20260813_20260814T022400Z_v3/manifest.json"
TABLE3_RECORDS = REPO / "exp/runtime/urdf_table3_lam_n800_seed20260813_20260814T022400Z_v3/asset_records.jsonl"
PROTOCOL_DOCUMENT = REPO / "exp/URDF-Sim-Ready-Automatic-Evaluation.md"

# Frozen pre-result cohort identity hashes (computed 2026-08-19 before this run).
EXPECTED_TABLE3_MANIFEST_FILE_SHA256 = "7e16683bfe4e4f37d7972082d8512713c1d8d1ae4ce142b75bf7dfb0509b9951"
EXPECTED_TABLE3_MANIFEST_SELF_SHA256 = "f8f7fe4da5634d4f806e793c0da919689eab25be1ce0bbed7e2232f3453d15c2"
EXPECTED_TABLE3_RECORDS_FILE_SHA256 = "7ef1c38d61bc780e41f62c7dd359e66f0bfeabe655c7453c93e2ea9830122d94"
EXPECTED_ORDERED_ASSET_KEYS_SHA256 = "643aa5b76ac61f57dd943bee26444a3525c01201a8dff3443763a7fd8d8267d3"

EXPECTED_COHORT_SIZE = 800
EXPECTED_J_EVAL = 2395
EXPECTED_CATEGORY_COUNT = 305
EXPECTED_TIER_COUNTS = {"viable": 621, "loads_only": 75, "broken": 104}
N_RELEASE = 3217
UPSTREAM_REVISION = "28cec4f5be7e34fd4d586879ecfcb67f7c5e4cc0"
SELECTION_SEED = 20260813
URDF_RELATIVE_PATH = "generated.urdf"
JOINT_COUNT_BINS = ("0", "1", "2-3", "4-7", ">=8")

PLACEHOLDER_REGISTRY: list[dict[str, Any]] = []  # frozen empty; see module docstring
PLACEHOLDER_REGISTRY_RATIONALE = (
    "frozen empty: no LAM exporter/simulator default mass or inertia template was "
    "validated from frozen tool defaults or public documentation before result "
    "inspection; identical to the frozen LAM supplementary precedent "
    "(run_urdf_lam_supplementary_v1.py passes placeholder_registry=[]) and the "
    "Table 2 supplementary precedent of the other methods; incidence therefore "
    "reported N/E with complete-inertial coverage"
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
        "Per-asset expected denominator = Table 3 frozen manifest "
        "records[].declared_joint_count_hint; cross-checked at runtime against "
        "Table 3 asset_records.jsonl declared_joint_count; mismatch fails the "
        "asset closed."
    ),
    "urdf_identity_gate": (
        "Per-asset generated.urdf SHA-256 must equal the Table 3 frozen manifest "
        "records[].urdf_sha256 and the frozen absolute urdf_path must equal "
        "<package>/generated.urdf; mismatch fails the asset closed."
    ),
    "placeholder_registry": PLACEHOLDER_REGISTRY_RATIONALE,
}

DEFAULT_WORKERS = 8
ASSET_TIMEOUT_SECONDS = 300

INPUT_IDENTITY_FIELDS = (
    "selection_index", "asset_id", "tier", "rel_path", "object_release_id",
    "category", "release_order", "selection_rank", "selection_hash",
    "package", "primary_urdf_relative_path", "expected_declared_joint_count",
    "model_urdf_sha256_expected", "source_root",
)


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


def atomic_write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    temporary = Path(name)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            handle.write(text)
        temporary.replace(path)
    finally:
        if temporary.exists():
            temporary.unlink()


def atomic_json(path: Path, value: Any) -> None:
    atomic_write_text(path, canonical_json(value) + "\n")


def utc_now_iso() -> str:
    return datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def timestamp_tag() -> str:
    return datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def module_sha256(module_name: str) -> str:
    target = SCRIPT.with_name(f"{module_name}.py")
    return sha256_file(target)


from exp.scripts import table2_supplementary_static as static_evaluator  # noqa: E402


class ProtocolViolation(RuntimeError):
    """Raised when a frozen input or binding check fails."""


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


# --------------------------------------------------------------------------
# Cohort freezing
# --------------------------------------------------------------------------

def verify_table3_manifest() -> dict[str, Any]:
    observed_file_sha = sha256_file(TABLE3_MANIFEST)
    if observed_file_sha != EXPECTED_TABLE3_MANIFEST_FILE_SHA256:
        raise ProtocolViolation(
            f"Table 3 LAM manifest file SHA-256 mismatch: expected "
            f"{EXPECTED_TABLE3_MANIFEST_FILE_SHA256}, observed {observed_file_sha}"
        )
    payload = load_json(TABLE3_MANIFEST)
    declared = payload.get("manifest_content_sha256")
    body = {key: value for key, value in payload.items() if key != "manifest_content_sha256"}
    computed = canonical_sha256(body)
    if declared != EXPECTED_TABLE3_MANIFEST_SELF_SHA256 or computed != EXPECTED_TABLE3_MANIFEST_SELF_SHA256:
        raise ProtocolViolation(
            "Table 3 LAM manifest self-hash mismatch: declared "
            f"{declared}, computed {computed}, expected {EXPECTED_TABLE3_MANIFEST_SELF_SHA256}"
        )
    records = payload.get("records")
    if not isinstance(records, list) or len(records) != EXPECTED_COHORT_SIZE:
        raise ProtocolViolation(
            f"Table 3 manifest must contain {EXPECTED_COHORT_SIZE} records, found "
            f"{len(records) if isinstance(records, list) else type(records).__name__}"
        )
    if payload.get("dataset") != "LAM released outputs (Articulated-Object-Code)":
        raise ProtocolViolation(f"Table 3 manifest dataset is {payload.get('dataset')!r}")
    source = payload.get("source", {})
    if source.get("n_release") != N_RELEASE:
        raise ProtocolViolation(f"Table 3 manifest n_release is {source.get('n_release')!r}")
    if source.get("upstream_revision") != UPSTREAM_REVISION:
        raise ProtocolViolation(f"Table 3 manifest upstream_revision is {source.get('upstream_revision')!r}")
    if source.get("source_root") != str(LAM_SOURCE_ROOT):
        raise ProtocolViolation(f"Table 3 manifest source_root is {source.get('source_root')!r}")
    selection = payload.get("selection", {})
    if selection.get("seed") != SELECTION_SEED:
        raise ProtocolViolation(f"Table 3 manifest selection seed is {selection.get('seed')!r}")
    if selection.get("n_eval") != EXPECTED_COHORT_SIZE:
        raise ProtocolViolation(f"Table 3 manifest selection n_eval is {selection.get('n_eval')!r}")
    return payload


def load_table3_expected_joints() -> dict[str, int]:
    observed_file_sha = sha256_file(TABLE3_RECORDS)
    if observed_file_sha != EXPECTED_TABLE3_RECORDS_FILE_SHA256:
        raise ProtocolViolation(
            f"Table 3 LAM asset_records file SHA-256 mismatch: expected "
            f"{EXPECTED_TABLE3_RECORDS_FILE_SHA256}, observed {observed_file_sha}"
        )
    expected: dict[str, int] = {}
    with TABLE3_RECORDS.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            row = json.loads(line)
            asset_key = row.get("asset_key")
            count = row.get("declared_joint_count")
            if not isinstance(asset_key, str) or isinstance(count, bool) or not isinstance(count, int) or count < 0:
                raise ProtocolViolation(f"invalid Table 3 declared_joint_count row for {asset_key!r}")
            if asset_key in expected:
                raise ProtocolViolation(f"duplicate Table 3 asset_key: {asset_key}")
            expected[asset_key] = count
    if len(expected) != EXPECTED_COHORT_SIZE:
        raise ProtocolViolation(
            f"Table 3 records must cover {EXPECTED_COHORT_SIZE} assets, found {len(expected)}"
        )
    total = sum(expected.values())
    if total != EXPECTED_J_EVAL:
        raise ProtocolViolation(f"Table 3 declared_joint_count total {total} != frozen J_eval {EXPECTED_J_EVAL}")
    return expected


def freeze_cohort(*, limit: int | None) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    table3 = verify_table3_manifest()
    expected_joints = load_table3_expected_joints()

    records = table3["records"]
    ranks = [record["selection_rank"] for record in records]
    if ranks != list(range(1, EXPECTED_COHORT_SIZE + 1)):
        raise ProtocolViolation("Table 3 manifest records are not in selection_rank 1..800 order")

    ordered_keys = [record["asset_key"] for record in records]
    ordered_sha = hashlib.sha256(
        json.dumps(ordered_keys, separators=(",", ":"), ensure_ascii=True).encode("utf-8")
    ).hexdigest()
    if ordered_sha != EXPECTED_ORDERED_ASSET_KEYS_SHA256:
        raise ProtocolViolation(
            f"ordered asset_key SHA-256 mismatch: expected {EXPECTED_ORDERED_ASSET_KEYS_SHA256}, "
            f"observed {ordered_sha}"
        )
    if len(set(ordered_keys)) != EXPECTED_COHORT_SIZE:
        raise ProtocolViolation("Table 3 manifest asset_key values are not unique")

    tier_counts: dict[str, int] = {}
    categories: set[str] = set()
    for record in records:
        tier_counts[record["tier"]] = tier_counts.get(record["tier"], 0) + 1
        categories.add(record["category"])
    if tier_counts != EXPECTED_TIER_COUNTS:
        raise ProtocolViolation(f"tier counts mismatch: {tier_counts}")
    if len(categories) != EXPECTED_CATEGORY_COUNT:
        raise ProtocolViolation(f"category count mismatch: {len(categories)}")
    joint_total = sum(record["declared_joint_count_hint"] for record in records)
    if joint_total != EXPECTED_J_EVAL:
        raise ProtocolViolation(f"declared_joint_count_hint total {joint_total} != frozen J_eval {EXPECTED_J_EVAL}")

    items: list[dict[str, Any]] = []
    for index, record in enumerate(records):
        asset_key = record["asset_key"]
        if asset_key not in expected_joints:
            raise ProtocolViolation(f"asset missing from Table 3 asset_records: {asset_key}")
        if expected_joints[asset_key] != record["declared_joint_count_hint"]:
            raise ProtocolViolation(
                f"joint count mismatch for {asset_key}: manifest hint "
                f"{record['declared_joint_count_hint']} != asset_records {expected_joints[asset_key]}"
            )
        package = (LAM_SOURCE_ROOT / record["rel_path"]).as_posix()
        expected_urdf_path = f"{package}/{URDF_RELATIVE_PATH}"
        if record["urdf_path"] != expected_urdf_path:
            raise ProtocolViolation(
                f"urdf_path binding mismatch for {asset_key}: {record['urdf_path']} != {expected_urdf_path}"
            )
        item = {
            "selection_index": index,
            "asset_id": asset_key,
            "tier": record["tier"],
            "rel_path": record["rel_path"],
            "object_release_id": record["object_release_id"],
            "category": record["category"],
            "release_order": record["release_order"],
            "selection_rank": record["selection_rank"],
            "selection_hash": record["selection_hash"],
            "package": package,
            "primary_urdf_relative_path": URDF_RELATIVE_PATH,
            "expected_declared_joint_count": record["declared_joint_count_hint"],
            "model_urdf_sha256_expected": record["urdf_sha256"],
            "source_root": str(LAM_SOURCE_ROOT),
        }
        item["input_identity_sha256"] = canonical_sha256(
            {field: item[field] for field in INPUT_IDENTITY_FIELDS}
        )
        items.append(item)

    if limit is not None:
        items = items[:limit]

    provenance = {
        "dataset": DATASET,
        "cohort_source_manifest": str(TABLE3_MANIFEST),
        "cohort_source_manifest_file_sha256": EXPECTED_TABLE3_MANIFEST_FILE_SHA256,
        "cohort_source_manifest_self_sha256": EXPECTED_TABLE3_MANIFEST_SELF_SHA256,
        "cohort_source_records": str(TABLE3_RECORDS),
        "cohort_source_records_file_sha256": EXPECTED_TABLE3_RECORDS_FILE_SHA256,
        "ordered_asset_keys_sha256": EXPECTED_ORDERED_ASSET_KEYS_SHA256,
        "selection_policy": (
            "all Table 3 manifest records[] in selection_rank 1..800 order; "
            "no resampling, replacement or result-based filtering"
        ),
        "selection_seed": SELECTION_SEED,
        "n_release": N_RELEASE,
        "upstream_revision": UPSTREAM_REVISION,
        "source_root": str(LAM_SOURCE_ROOT),
        "tier_counts": dict(sorted(tier_counts.items())),
        "category_count": len(categories),
        "n_eval": len(items),
        "j_eval": sum(item["expected_declared_joint_count"] for item in items),
        "protocol_id": PROTOCOL_ID,
        "schema_version": SCHEMA_VERSION,
        "metric_rules": METRIC_RULES,
        "operationalization": OPERATIONALIZATION,
        "placeholder_registry": PLACEHOLDER_REGISTRY,
        "placeholder_registry_rationale": PLACEHOLDER_REGISTRY_RATIONALE,
    }
    return items, provenance


def write_frozen_manifest(output_root: Path, items: list[dict[str, Any]], provenance: dict[str, Any]) -> dict[str, Any]:
    manifest = {
        "schema_version": SCHEMA_VERSION,
        "protocol_id": PROTOCOL_ID,
        "dataset": DATASET,
        "created_at_utc": utc_now_iso(),
        "provenance": provenance,
        "items": items,
    }
    manifest["manifest_content_sha256"] = canonical_sha256(
        {key: value for key, value in manifest.items() if key != "manifest_content_sha256"}
    )
    atomic_json(output_root / "frozen_manifest.json", manifest)
    return manifest


# --------------------------------------------------------------------------
# Package binding
# --------------------------------------------------------------------------

def verify_binding(item: Mapping[str, Any]) -> dict[str, Any]:
    package = Path(item["package"])
    issues: list[str] = []
    if package.is_symlink():
        issues.append("package_is_symlink")
    elif not package.is_dir():
        issues.append("package_missing")
    else:
        try:
            resolved = package.resolve(strict=True)
            resolved.relative_to(LAM_SOURCE_ROOT.resolve(strict=True))
        except (OSError, ValueError):
            issues.append("package_escapes_source_root")
        if not issues:
            urdf_path = resolved / item["primary_urdf_relative_path"]
            if not urdf_path.is_file() or urdf_path.is_symlink():
                issues.append("primary_urdf_missing")
            else:
                observed_urdf_sha = sha256_file(urdf_path)
                if observed_urdf_sha != item["model_urdf_sha256_expected"]:
                    issues.append(
                        "model_urdf_sha256_mismatch: expected "
                        f"{item['model_urdf_sha256_expected']}, observed {observed_urdf_sha}"
                    )
    return {
        "verified": not issues,
        "issues": issues,
        "verified_at_utc": utc_now_iso(),
    }


# --------------------------------------------------------------------------
# Evaluation
# --------------------------------------------------------------------------

def timeout_record(item: Mapping[str, Any], reason: str) -> dict[str, Any]:
    intended = int(item["expected_declared_joint_count"])
    return {
        "schema_version": "table2-supplementary-static/v1",
        "asset_id": item["asset_id"],
        "package": item["package"],
        "urdf_relative_path": item["primary_urdf_relative_path"],
        "urdf_sha256": None,
        "status": "error",
        "parse": {"success": False, "issues": [reason]},
        "table2_supplementary": {
            "visual_bearing_collision_coverage": {
                "status": "NOT_EVALUABLE", "asset_intended": 1, "asset_passed": 0,
                "asset_pass": False, "visual_bearing_links_declared": 0,
                "covered_visual_bearing_links": 0, "link_extraction_complete": False,
                "collision_elements_declared_on_visual_links": 0,
                "loadable_collision_elements_on_visual_links": 0,
                "link_records": [], "issues": [reason],
            },
            "joint_limit_portability": {
                "status": "NOT_EVALUABLE",
                "joints_intended": intended,
                "joints_extracted": 0, "joints_passed": 0, "extraction_complete": False,
                "joint_records": [], "issues": [reason],
            },
            "joint_dynamics_coverage": {
                "status": "NOT_EVALUABLE",
                "joints_intended": intended,
                "joints_extracted": 0, "joints_covered": 0, "extraction_complete": False,
                "joint_records": [], "issues": [reason],
            },
            "placeholder_mass_incidence": {
                "status": "N/E", "dynamic_link_policy": "all_declared_links",
                "dynamic_links": 0, "complete_inertial_links": 0,
                "complete_inertial_coverage_numerator": 0,
                "complete_inertial_coverage_denominator": 0,
                "classified_complete_inertial_links": 0,
                "unclassified_complete_inertial_links": 0,
                "placeholder_links": None, "incidence_numerator": None,
                "incidence_denominator": 0, "registry_ids": [],
                "link_records": [], "incomplete_inertial_links": [], "issues": [reason],
            },
        },
        "resource_closure": {
            "status": "NOT_EVALUABLE", "complete": False, "file_count": 0,
            "sha256": None, "files": [], "issues": [reason],
        },
        "issues": [reason],
    }


def child_receipt_path(output_root: Path, selection_index: int) -> Path:
    return output_root / "children" / f"rank_{selection_index:04d}.json"


def run_evaluation(
    output_root: Path,
    items: list[dict[str, Any]],
    binding: dict[str, dict[str, Any]],
    *,
    workers: int,
) -> dict[int, dict[str, Any]]:
    receipts: dict[int, dict[str, Any]] = {}

    # Resume: load existing child receipts and validate identity.
    for item in items:
        path = child_receipt_path(output_root, item["selection_index"])
        if not path.exists():
            continue
        payload = load_json(path)
        if payload.get("input_identity_sha256") != item["input_identity_sha256"]:
            raise ProtocolViolation(
                f"stale child receipt for selection_index={item['selection_index']} "
                "(frozen input identity changed); refusing to resume"
            )
        receipts[item["selection_index"]] = payload

    pending = [item for item in items if item["selection_index"] not in receipts]
    if not pending:
        return receipts

    context = multiprocessing.get_context("fork")
    pool = context.Pool(processes=workers)
    try:
        for item in pending:
            index = item["selection_index"]
            bind = binding[item["asset_id"]]
            if not bind["verified"]:
                reason = "binding_failed: " + "; ".join(bind["issues"])
                receipt = {
                    "input_identity_sha256": item["input_identity_sha256"],
                    "selection_index": index,
                    "asset_id": item["asset_id"],
                    "status": "binding_failed",
                    "binding": bind,
                    "audit": timeout_record(item, reason),
                    "completed_at_utc": utc_now_iso(),
                }
                atomic_json(child_receipt_path(output_root, index), receipt)
                receipts[index] = receipt
                continue

            async_result = pool.apply_async(
                static_evaluator.audit_worker,
                ({
                    "package": item["package"],
                    "primary_urdf_relative_path": item["primary_urdf_relative_path"],
                    "asset_id": item["asset_id"],
                    "expected_declared_joint_count": item["expected_declared_joint_count"],
                    "placeholder_registry": PLACEHOLDER_REGISTRY,
                },),
            )
            try:
                audit = async_result.get(timeout=ASSET_TIMEOUT_SECONDS)
                status = "completed" if audit.get("status") == "completed" else "error"
            except multiprocessing.TimeoutError:
                audit = timeout_record(item, f"asset_timeout_after_{ASSET_TIMEOUT_SECONDS}s")
                status = "timeout"
                pool.terminate()
                pool.join()
                pool = context.Pool(processes=workers)
            except Exception as exc:  # noqa: BLE001
                audit = timeout_record(
                    item, f"worker_exception: {type(exc).__name__}: {exc}"
                )
                status = "error"

            receipt = {
                "input_identity_sha256": item["input_identity_sha256"],
                "selection_index": index,
                "asset_id": item["asset_id"],
                "status": status,
                "binding": bind,
                "audit": audit,
                "completed_at_utc": utc_now_iso(),
            }
            atomic_json(child_receipt_path(output_root, index), receipt)
            receipts[index] = receipt
    finally:
        pool.terminate()
        pool.join()
    return receipts


# --------------------------------------------------------------------------
# Aggregation
# --------------------------------------------------------------------------

def joint_count_bin(count: int) -> str:
    if count <= 0:
        return "0"
    if count == 1:
        return "1"
    if count <= 3:
        return "2-3"
    if count <= 7:
        return "4-7"
    return ">=8"


def ratio(numerator: int, denominator: int) -> dict[str, Any]:
    return {
        "numerator": numerator,
        "denominator": denominator,
        "percentage": (100.0 * numerator / denominator) if denominator else None,
    }


def aggregate(items: list[dict[str, Any]], receipts: dict[int, dict[str, Any]]) -> dict[str, Any]:
    n_eval = len(items)
    j_eval = sum(item["expected_declared_joint_count"] for item in items)

    asset_pass = 0
    link_declared = 0
    link_covered = 0
    link_extraction_complete_assets = 0
    portability_passed = 0
    portability_intended = 0
    portability_extracted = 0
    dynamics_covered = 0
    dynamics_intended = 0
    placeholder_status_counts: dict[str, int] = {}
    complete_inertial_links = 0
    dynamic_links = 0
    status_counts: dict[str, int] = {}
    parse_success = 0

    category: dict[str, dict[str, int]] = {}
    tier: dict[str, dict[str, int]] = {}
    joint_type: dict[str, dict[str, int]] = {}
    joint_bin: dict[str, dict[str, int]] = {bin_name: {"assets": 0, "asset_pass": 0} for bin_name in JOINT_COUNT_BINS}

    for item in items:
        receipt = receipts[item["selection_index"]]
        audit = receipt["audit"]
        status = receipt["status"]
        status_counts[status] = status_counts.get(status, 0) + 1
        table2 = audit.get("table2_supplementary", {})
        visual = table2.get("visual_bearing_collision_coverage", {})
        portability = table2.get("joint_limit_portability", {})
        dynamics = table2.get("joint_dynamics_coverage", {})
        placeholder = table2.get("placeholder_mass_incidence", {})

        passed = int(visual.get("asset_passed", 0))
        asset_pass += passed
        link_declared += int(visual.get("visual_bearing_links_declared", 0))
        link_covered += int(visual.get("covered_visual_bearing_links", 0))
        if visual.get("link_extraction_complete"):
            link_extraction_complete_assets += 1
        portability_passed += int(portability.get("joints_passed", 0))
        portability_intended += int(portability.get("joints_intended", 0))
        portability_extracted += int(portability.get("joints_extracted", 0))
        dynamics_covered += int(dynamics.get("joints_covered", 0))
        dynamics_intended += int(dynamics.get("joints_intended", 0))
        placeholder_status_counts[str(placeholder.get("status"))] = (
            placeholder_status_counts.get(str(placeholder.get("status")), 0) + 1
        )
        complete_inertial_links += int(placeholder.get("complete_inertial_links", 0))
        dynamic_links += int(placeholder.get("dynamic_links", 0))
        if audit.get("parse", {}).get("success"):
            parse_success += 1

        for key, buckets in ((item["category"], category), (item["tier"], tier)):
            bucket = buckets.setdefault(key, {
                "assets": 0, "asset_pass": 0,
                "portability_passed": 0, "portability_intended": 0,
                "dynamics_covered": 0, "dynamics_intended": 0,
            })
            bucket["assets"] += 1
            bucket["asset_pass"] += passed
            bucket["portability_passed"] += int(portability.get("joints_passed", 0))
            bucket["portability_intended"] += int(portability.get("joints_intended", 0))
            bucket["dynamics_covered"] += int(dynamics.get("joints_covered", 0))
            bucket["dynamics_intended"] += int(dynamics.get("joints_intended", 0))

        for joint_record in portability.get("joint_records", []):
            joint_type_bucket = joint_type.setdefault(str(joint_record.get("joint_type")), {
                "joints": 0, "portability_passed": 0, "dynamics_covered": 0,
            })
            joint_type_bucket["joints"] += 1
            joint_type_bucket["portability_passed"] += int(bool(joint_record.get("limit_portability_pass")))
        for joint_record in dynamics.get("joint_records", []):
            joint_type_bucket = joint_type.setdefault(str(joint_record.get("joint_type")), {
                "joints": 0, "portability_passed": 0, "dynamics_covered": 0,
            })
            joint_type_bucket["dynamics_covered"] += int(bool(joint_record.get("covered")))

        bin_name = joint_count_bin(item["expected_declared_joint_count"])
        joint_bin[bin_name]["assets"] += 1
        joint_bin[bin_name]["asset_pass"] += passed

    def macro(buckets: Mapping[str, dict[str, int]]) -> dict[str, Any]:
        macro_visual: list[float] = []
        macro_portability: list[float] = []
        macro_dynamics: list[float] = []
        for bucket in buckets.values():
            if bucket["assets"]:
                macro_visual.append(100.0 * bucket["asset_pass"] / bucket["assets"])
            if bucket["portability_intended"]:
                macro_portability.append(100.0 * bucket["portability_passed"] / bucket["portability_intended"])
            if bucket["dynamics_intended"]:
                macro_dynamics.append(100.0 * bucket["dynamics_covered"] / bucket["dynamics_intended"])
        return {
            "group_count": len(buckets),
            "visual_bearing_collision_coverage_pct": (sum(macro_visual) / len(macro_visual)) if macro_visual else None,
            "joint_limit_portability_pct": (sum(macro_portability) / len(macro_portability)) if macro_portability else None,
            "joint_dynamics_coverage_pct": (sum(macro_dynamics) / len(macro_dynamics)) if macro_dynamics else None,
        }

    def breakdown(buckets: Mapping[str, dict[str, int]]) -> dict[str, Any]:
        return {
            key: {
                "assets": value["assets"],
                "visual_bearing_collision_coverage": ratio(value["asset_pass"], value["assets"]),
                "joint_limit_portability": ratio(value["portability_passed"], value["portability_intended"]),
                "joint_dynamics_coverage": ratio(value["dynamics_covered"], value["dynamics_intended"]),
            }
            for key, value in sorted(buckets.items())
        }

    summary = {
        "protocol_id": PROTOCOL_ID,
        "schema_version": SCHEMA_VERSION,
        "dataset": DATASET,
        "n_eval": n_eval,
        "j_eval": j_eval,
        "status_counts": dict(sorted(status_counts.items())),
        "parse_success_assets": parse_success,
        "metrics": {
            "visual_bearing_collision_coverage": {
                "asset_level": ratio(asset_pass, n_eval),
                "link_micro": ratio(link_covered, link_declared),
                "link_extraction_coverage": ratio(link_extraction_complete_assets, n_eval),
            },
            "joint_limit_portability": {
                "joint_level": ratio(portability_passed, j_eval),
                "intended_from_frozen_table3": ratio(portability_intended, j_eval),
                "extracted": portability_extracted,
            },
            "joint_dynamics_coverage": {
                "joint_level": ratio(dynamics_covered, j_eval),
                "intended_from_frozen_table3": ratio(dynamics_intended, j_eval),
            },
            "placeholder_mass_incidence": {
                "status": "N/E" if not PLACEHOLDER_REGISTRY else "COMPLETE",
                "reason": PLACEHOLDER_REGISTRY_RATIONALE if not PLACEHOLDER_REGISTRY else None,
                "registry_ids": [],
                "placeholder_status_counts": dict(sorted(placeholder_status_counts.items())),
                "complete_inertial_coverage": ratio(complete_inertial_links, dynamic_links),
            },
        },
        "category_macro": macro(category),
        "category_breakdown": breakdown(category),
        "tier_macro": macro(tier),
        "tier_breakdown": breakdown(tier),
        "joint_type_breakdown": {
            key: {
                "joints": value["joints"],
                "portability_passed": value["portability_passed"],
                "dynamics_covered": value["dynamics_covered"],
            }
            for key, value in sorted(joint_type.items())
        },
        "joint_count_bin_breakdown": {
            bin_name: ratio(value["asset_pass"], value["assets"]) | {"assets": value["assets"]}
            for bin_name, value in joint_bin.items()
        },
        "completed_at_utc": utc_now_iso(),
    }
    return summary


def write_asset_records(
    output_root: Path,
    items: list[dict[str, Any]],
    receipts: dict[int, dict[str, Any]],
) -> None:
    lines: list[str] = []
    for item in items:
        receipt = receipts[item["selection_index"]]
        audit = receipt["audit"]
        table2 = audit.get("table2_supplementary", {})
        visual = table2.get("visual_bearing_collision_coverage", {})
        portability = table2.get("joint_limit_portability", {})
        dynamics = table2.get("joint_dynamics_coverage", {})
        placeholder = table2.get("placeholder_mass_incidence", {})
        record = {
            "selection_index": item["selection_index"],
            "asset_id": item["asset_id"],
            "tier": item["tier"],
            "rel_path": item["rel_path"],
            "object_release_id": item["object_release_id"],
            "category": item["category"],
            "release_order": item["release_order"],
            "selection_rank": item["selection_rank"],
            "selection_hash": item["selection_hash"],
            "package": item["package"],
            "primary_urdf_relative_path": item["primary_urdf_relative_path"],
            "expected_declared_joint_count": item["expected_declared_joint_count"],
            "input_identity_sha256": item["input_identity_sha256"],
            "binding_verified": receipt["binding"]["verified"],
            "binding_issues": receipt["binding"]["issues"],
            "status": receipt["status"],
            "parse_success": bool(audit.get("parse", {}).get("success")),
            "urdf_sha256": audit.get("urdf_sha256"),
            "visual_bearing_collision_coverage_asset_pass": bool(visual.get("asset_pass")),
            "visual_bearing_links_declared": int(visual.get("visual_bearing_links_declared", 0)),
            "covered_visual_bearing_links": int(visual.get("covered_visual_bearing_links", 0)),
            "joint_limit_portability_passed": int(portability.get("joints_passed", 0)),
            "joint_limit_portability_intended": int(portability.get("joints_intended", 0)),
            "joint_dynamics_covered": int(dynamics.get("joints_covered", 0)),
            "joint_dynamics_intended": int(dynamics.get("joints_intended", 0)),
            "placeholder_mass_status": placeholder.get("status"),
            "complete_inertial_links": int(placeholder.get("complete_inertial_links", 0)),
            "dynamic_links": int(placeholder.get("dynamic_links", 0)),
            "issues": audit.get("issues", []),
            "audit": audit,
            "completed_at_utc": receipt["completed_at_utc"],
        }
        lines.append(canonical_json(record))
    atomic_write_text(output_root / "asset_records.jsonl", "\n".join(lines) + "\n")


def render_summary_md(summary: Mapping[str, Any]) -> str:
    metrics = summary["metrics"]
    visual = metrics["visual_bearing_collision_coverage"]
    portability = metrics["joint_limit_portability"]
    dynamics = metrics["joint_dynamics_coverage"]
    placeholder = metrics["placeholder_mass_incidence"]

    def pct(cell: Mapping[str, Any]) -> str:
        if cell.get("denominator") and cell.get("percentage") is not None:
            return f"{cell['numerator']} / {cell['denominator']} ({cell['percentage']:.2f}%)"
        return f"{cell.get('numerator')} / {cell.get('denominator')} (N/E)"

    lines = [
        f"# Table 2 supplementary — {summary['dataset']} (protocol {summary['protocol_id']})",
        "",
        f"- N_eval = {summary['n_eval']}, J_eval = {summary['j_eval']}",
        f"- status counts: {summary['status_counts']}",
        f"- parse success assets: {summary['parse_success_assets']}",
        "",
        "| Metric | Result |",
        "|---|---|",
        f"| Visual-bearing Collision Coverage (asset) | {pct(visual['asset_level'])} |",
        f"| Visual-bearing Collision Coverage (link-micro) | {pct(visual['link_micro'])} |",
        f"| Link extraction coverage | {pct(visual['link_extraction_coverage'])} |",
        f"| Joint-limit Portability | {pct(portability['joint_level'])} |",
        f"| Joint Dynamics Coverage | {pct(dynamics['joint_level'])} |",
        f"| Placeholder-mass Incidence | {placeholder['status']} ({placeholder['reason']}) |",
        f"| Complete-inertial coverage | {pct(placeholder['complete_inertial_coverage'])} |",
        "",
        f"Category macro ({summary['category_macro']['group_count']} categories): "
        f"visual {summary['category_macro']['visual_bearing_collision_coverage_pct']}, "
        f"portability {summary['category_macro']['joint_limit_portability_pct']}, "
        f"dynamics {summary['category_macro']['joint_dynamics_coverage_pct']}",
        "",
        f"Tier macro: "
        f"visual {summary['tier_macro']['visual_bearing_collision_coverage_pct']}, "
        f"portability {summary['tier_macro']['joint_limit_portability_pct']}, "
        f"dynamics {summary['tier_macro']['joint_dynamics_coverage_pct']}",
        "",
    ]
    return "\n".join(lines)


def write_environment(output_root: Path, workers: int) -> None:
    environment = {
        "python": platform.python_version(),
        "python_implementation": platform.python_implementation(),
        "executable": sys.executable,
        "platform": platform.platform(),
        "worker_count": workers,
        "gpu_required": False,
        "evaluator_modules": {
            "table2_supplementary_static_sha256": module_sha256("table2_supplementary_static"),
            "lam_supplementary_static_sha256": module_sha256("lam_supplementary_static"),
        },
        "runner_sha256": sha256_file(SCRIPT),
        "recorded_at_utc": utc_now_iso(),
    }
    atomic_json(output_root / "environment.json", environment)


def write_protocol_snapshot(output_root: Path) -> str:
    snapshot = PROTOCOL_DOCUMENT.read_text(encoding="utf-8")
    atomic_write_text(output_root / "protocol_snapshot.md", snapshot)
    return sha256_file(PROTOCOL_DOCUMENT)


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------

def default_output_root(*, limit: int | None) -> Path:
    if limit is None:
        return REPO / f"exp/runtime/table2sup_urdf_lam_table3cohort_n800_seed20260813_{timestamp_tag()}"
    return REPO / f"exp/runtime/table2sup_urdf_lam_smoke_n{limit}_{timestamp_tag()}"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--workers", type=int, default=DEFAULT_WORKERS)
    parser.add_argument("--limit", type=int, default=None, help="smoke mode: evaluate only the first N frozen items")
    parser.add_argument("--skip-verify", action="store_true", help="do not run the independent verifier at the end")
    args = parser.parse_args(argv)

    if args.limit is not None and (args.limit <= 0 or args.limit > EXPECTED_COHORT_SIZE):
        raise SystemExit(f"--limit must be within 1..{EXPECTED_COHORT_SIZE}")

    output_root = args.output or default_output_root(limit=args.limit)
    if output_root.exists() and any(output_root.iterdir()):
        raise SystemExit(f"output root already exists and is not empty: {output_root}")
    output_root.mkdir(parents=True, exist_ok=True)

    started = time.monotonic()
    items, provenance = freeze_cohort(limit=args.limit)
    provenance["output_root"] = str(output_root)
    manifest = write_frozen_manifest(output_root, items, provenance)
    snapshot_sha = write_protocol_snapshot(output_root)
    write_environment(output_root, args.workers)
    print(f"[freeze] {len(items)} items, J_eval={provenance['j_eval']}, manifest sha256={manifest['manifest_content_sha256']}")
    print(f"[freeze] protocol snapshot sha256={snapshot_sha}")

    print("[binding] verifying package bindings ...")
    binding: dict[str, dict[str, Any]] = {}
    binding_failures = 0
    for item in items:
        result = verify_binding(item)
        binding[item["asset_id"]] = result
        if not result["verified"]:
            binding_failures += 1
            print(f"[binding] FAIL {item['asset_id']}: {'; '.join(result['issues'])}")
    print(f"[binding] verified {len(items) - binding_failures} / {len(items)}")

    receipts = run_evaluation(output_root, items, binding, workers=args.workers)

    summary = aggregate(items, receipts)
    summary["frozen_manifest_sha256"] = manifest["manifest_content_sha256"]
    summary["protocol_snapshot_sha256"] = snapshot_sha
    summary["elapsed_seconds"] = time.monotonic() - started
    atomic_json(output_root / "summary.json", summary)
    atomic_write_text(output_root / "summary.md", render_summary_md(summary))
    write_asset_records(output_root, items, receipts)

    print(f"[done] summary written to {output_root / 'summary.json'}")
    metrics = summary["metrics"]
    print(json.dumps(metrics, indent=2, sort_keys=True))

    if not args.skip_verify:
        verifier_path = SCRIPT.with_name("verify_table2_supplementary_lam_v1.py")
        spec = importlib.util.spec_from_file_location("verify_table2_supplementary_lam_v1", verifier_path)
        if spec is None or spec.loader is None:
            raise RuntimeError("cannot load verifier")
        verifier = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(verifier)
        verification = verifier.verify_run(output_root)
        atomic_json(output_root / "verification.json", verification)
        if verification["status"] != "PASS":
            print(f"[verify] FAILED: {output_root / 'verification.json'}")
            return 1
        print(f"[verify] PASS ({len(verification['checks'])} checks)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
