#!/usr/bin/env python3
"""Cross-simulator Table 5 summary.

Computes from existing per-simulator asset records (no re-runs):
  - per-simulator legacy diagnostic rates (native import/reset/settling/
    actuation/limit_enforcement/constraint_drift/simulator_pass), denominator = 200
  - All-3 Load: asset imports natively in all three simulators
  - All-3 Runtime: simulator_pass in all three simulators
  - Cross-sim Joint RMSE: worst-pair RMSE of 31-sample normalized joint
    trajectories (protocol cross_simulator.joint_rmse)
  - Cross-sim Pose Error: worst-pair endpoint pose disagreement
    (translation / bbox diagonal, quaternion angular error)

Articraft-10K uses the diagnostic resample re-run; all other datasets use
the formal eight-dataset run.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
FORMAL_ROOT = REPO_ROOT / "exp/runtime/table5_v2_r2_formal_eight_datasets"
DIAG_ROOT = REPO_ROOT / "exp/runtime/table5_v2_articraft_diagnostic_resample_20260830"
FORMAL_MANIFEST = REPO_ROOT / (
    "exp/runtime/table5_v2_core200_prepared_five_full_release_"
    "articraft10787_infinigen_paired_official_metrics_r2/manifest.json"
)
DIAG_MANIFEST = DIAG_ROOT / "prepared/manifest.json"
OUT_DIR = REPO_ROOT / "exp/runtime/table5_cross_sim_summary_articraft_resample_20260830"

SIMULATORS = ("genesis", "pybullet", "mujoco")
RUNTIME_METRICS = (
    "load",
    "reset",
    "settling",
    "actuation",
    "limit_enforcement",
    "constraint_drift",
    "simulator_pass",
)
DATASET_ORDER = (
    "articraft_10k",
    "lam_released_outputs",
    "artiverse",
    "partnet_mobility",
    "physx_mobility",
    "sketchmobility",
    "infinigen_sim",
    "pva",
)
RMSE_THRESHOLD = 0.1


def canonical_sha256(obj, exclude_fields=()):
    def prune(value):
        if isinstance(value, dict):
            return {k: prune(v) for k, v in sorted(value.items()) if k not in exclude_fields}
        if isinstance(value, list):
            return [prune(v) for v in value]
        return value
    blob = json.dumps(prune(obj), ensure_ascii=True, allow_nan=False, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(blob.encode("utf-8")).hexdigest()


def load_record(root: Path, slug: str, sim: str, dataset_id: str):
    path = root / slug / sim / "assets" / f"{dataset_id}.json"
    if not path.is_file():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None


def import_passed(record) -> bool:
    if not isinstance(record, dict):
        return False
    imported = (record.get("evaluation") or {}).get("v2", {}).get("import")
    return bool(isinstance(imported, dict) and imported.get("passed") is True)


def runtime_metrics(record) -> dict:
    if not isinstance(record, dict):
        return {name: False for name in RUNTIME_METRICS}
    metrics = (record.get("evaluation") or {}).get("metrics") or {}
    corrected = {name: bool(metrics.get(name)) for name in RUNTIME_METRICS}
    corrected["load"] = import_passed(record)
    corrected["simulator_pass"] = all(
        corrected[name]
        for name in RUNTIME_METRICS
        if name != "simulator_pass"
    )
    return corrected


def actuation_trajectories(record) -> dict:
    if not isinstance(record, dict):
        return {}
    diagnostics = (record.get("evaluation") or {}).get("diagnostics") or {}
    out = {}
    for row in diagnostics.get("actuation") or []:
        if not isinstance(row, dict):
            continue
        name = row.get("joint_name")
        trajectory = row.get("trajectory")
        if not isinstance(name, str) or not isinstance(trajectory, dict):
            continue
        measured = trajectory.get("normalized_positions")
        if not isinstance(measured, list) or len(measured) != 31:
            continue
        if any(not (isinstance(v, (int, float)) and math.isfinite(v)) for v in measured):
            continue
        out[name] = [float(v) for v in measured]
    return out


def endpoint_poses(record) -> dict:
    if not isinstance(record, dict):
        return {}
    diagnostics = (record.get("evaluation") or {}).get("diagnostics") or {}
    out = {}
    for row in diagnostics.get("actuation") or []:
        if not isinstance(row, dict):
            continue
        joint = row.get("joint_name")
        poses = row.get("final_descendant_root_frame_poses")
        if not isinstance(joint, str) or not isinstance(poses, dict):
            continue
        for link, pose in poses.items():
            if not isinstance(pose, dict):
                continue
            translation = pose.get("translation")
            rotation = pose.get("rotation")
            if not isinstance(translation, list) or not isinstance(rotation, list):
                continue
            if len(translation) != 3 or len(rotation) != 4:
                continue
            if not all(isinstance(x, (int, float)) and math.isfinite(x) for x in translation + rotation):
                continue
            out[(joint, link)] = (
                [float(x) for x in translation],
                [float(x) for x in rotation],
            )
    return out


def quaternion_angle(q1, q2) -> float:
    norm1 = math.sqrt(sum(x * x for x in q1)) or 1.0
    norm2 = math.sqrt(sum(x * x for x in q2)) or 1.0
    dot = sum(a * b for a, b in zip(q1, q2)) / (norm1 * norm2)
    return 2.0 * math.acos(min(1.0, abs(dot)))


def percentile(values, pct):
    if not values:
        return None
    ordered = sorted(values)
    idx = min(len(ordered) - 1, int(math.ceil(pct / 100.0 * len(ordered))) - 1)
    return ordered[max(idx, 0)]


def summarize_dataset(slug, rows, roots):
    n = len(rows)
    per_sim_rates = {}
    all3_load = all3_runtime = 0
    joint_rmse_values = []
    joint_pairs_evaluated = 0
    translation_values = []
    rotation_values = []
    pose_links_evaluated = 0

    for row in rows:
        dataset_id = row["dataset_id"]
        records = {sim: load_record(roots[sim], slug, sim, dataset_id) for sim in SIMULATORS}

        for sim in SIMULATORS:
            rates = per_sim_rates.setdefault(sim, {name: 0 for name in RUNTIME_METRICS})
            for name, passed in runtime_metrics(records[sim]).items():
                rates[name] += int(passed)

        imports = {sim: import_passed(records[sim]) for sim in SIMULATORS}
        passes = {sim: runtime_metrics(records[sim])["simulator_pass"] for sim in SIMULATORS}
        all3_load += int(all(imports.values()))
        all3_runtime += int(all(passes.values()))

        trajectories = {sim: actuation_trajectories(records[sim]) for sim in SIMULATORS}
        joint_names = set().union(*[set(t) for t in trajectories.values()]) if any(trajectories.values()) else set()
        for joint in joint_names:
            holders = [sim for sim in SIMULATORS if joint in trajectories[sim]]
            if len(holders) < 2:
                continue
            joint_pairs_evaluated += 1
            worst = max(
                math.sqrt(
                    sum((a - b) ** 2 for a, b in zip(trajectories[si][joint], trajectories[sj][joint])) / 31.0
                )
                for i, si in enumerate(holders)
                for sj in holders[i + 1:]
            )
            joint_rmse_values.append(worst)

        poses = {sim: endpoint_poses(records[sim]) for sim in SIMULATORS}
        bbox_diag = float(row.get("bounding_box_diagonal") or 0.0)
        keys = set().union(*[set(p) for p in poses.values()]) if any(poses.values()) else set()
        for key in keys:
            holders = [sim for sim in SIMULATORS if key in poses[sim]]
            if len(holders) < 2 or bbox_diag <= 0.0:
                continue
            pose_links_evaluated += 1
            worst_translation = 0.0
            worst_rotation = 0.0
            for i, si in enumerate(holders):
                for sj in holders[i + 1:]:
                    t1, r1 = poses[si][key]
                    t2, r2 = poses[sj][key]
                    delta = math.sqrt(sum((a - b) ** 2 for a, b in zip(t1, t2)))
                    worst_translation = max(worst_translation, delta / bbox_diag)
                    worst_rotation = max(worst_rotation, quaternion_angle(r1, r2))
            translation_values.append(worst_translation)
            rotation_values.append(worst_rotation)

    def rate(count):
        return {"passed": count, "denominator": n, "percentage": 100.0 * count / n if n else 0.0}

    joint_pass = sum(1 for v in joint_rmse_values if v <= RMSE_THRESHOLD)
    return {
        "dataset_slug": slug,
        "n": n,
        "per_simulator_runtime": {
            sim: {name: rate(per_sim_rates.get(sim, {}).get(name, 0)) for name in RUNTIME_METRICS}
            for sim in SIMULATORS
        },
        "all_three_load": rate(all3_load),
        "all_three_runtime": rate(all3_runtime),
        "cross_sim_joint_rmse": {
            "evaluated_joints": joint_pairs_evaluated,
            "median": percentile(joint_rmse_values, 50),
            "p95": percentile(joint_rmse_values, 95),
            "pass_threshold": RMSE_THRESHOLD,
            "pass_rate": (
                None if not joint_rmse_values else {
                    "passed": joint_pass,
                    "denominator": len(joint_rmse_values),
                    "percentage": 100.0 * joint_pass / len(joint_rmse_values),
                }
            ),
        },
        "cross_sim_pose_error": {
            "evaluated_links": pose_links_evaluated,
            "translation_over_bbox_median": percentile(translation_values, 50),
            "translation_over_bbox_p95": percentile(translation_values, 95),
            "rotation_deg_median": None if not rotation_values else math.degrees(percentile(rotation_values, 50)),
            "rotation_deg_p95": None if not rotation_values else math.degrees(percentile(rotation_values, 95)),
        },
    }


def main():
    formal_manifest = json.loads(FORMAL_MANIFEST.read_text(encoding="utf-8"))
    diag_manifest = json.loads(DIAG_MANIFEST.read_text(encoding="utf-8"))
    formal_rows = {d["dataset_slug"]: d["rows"] for d in formal_manifest["datasets"]}
    diag_rows = {d["dataset_slug"]: d["rows"] for d in diag_manifest["datasets"]}

    datasets = []
    for slug in DATASET_ORDER:
        if slug == "articraft_10k":
            roots = {sim: DIAG_ROOT / "runs" / sim for sim in SIMULATORS}
            rows = diag_rows["articraft_10k"]
            source = str(DIAG_ROOT)
        else:
            roots = {sim: FORMAL_ROOT / sim for sim in SIMULATORS}
            rows = formal_rows[slug]
            source = str(FORMAL_ROOT)
        block = summarize_dataset(slug, rows, roots)
        block["record_source"] = source
        datasets.append(block)
        print(f"done {slug}: all3_load={block['all_three_load']['passed']}/200 "
              f"all3_runtime={block['all_three_runtime']['passed']}/200 "
              f"joints={block['cross_sim_joint_rmse']['evaluated_joints']}")

    summary = {
        "schema_version": "table5_cross_sim_summary_v2",
        "load_definition": "native simulator import success (evaluation.v2.import.passed)",
        "legacy_diagnostic_note": (
            "reset/settling/actuation/limit_enforcement/constraint_drift are "
            "legacy stress-test diagnostics and are not the paper's primary Table 5a metrics"
        ),
        "articraft_source": "diagnostic_resample_20260830",
        "other_datasets_source": "formal_eight_datasets",
        "rmse_threshold": RMSE_THRESHOLD,
        "datasets": datasets,
    }
    summary["summary_sha256"] = canonical_sha256(summary, exclude_fields=("summary_sha256",))

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    (OUT_DIR / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (OUT_DIR / "report.md").write_text(render_report(summary), encoding="utf-8")
    print("out:", OUT_DIR)


def pct_cell(value):
    return "N/E" if value is None else f"{value * 100.0:.2f}"


def render_report(summary):
    lines = ["# Cross-simulator Table 5 summary", ""]
    lines.append("Articraft-10K = diagnostic resample re-run (20260830); other datasets = formal eight-dataset run.")
    lines.append("Load means native simulator import success only; preserving every declared link or joint is not required.")
    lines.append("Reset/Settling/Act/Limits/Drift/SimPass are legacy stress-test diagnostics, not the primary Table 5a metrics.")
    lines.append("")
    lines.append("## All-3 metrics")
    lines.append("")
    lines.append("| Dataset | N | All-3 Load (%) ↑ | All-3 Runtime (%) ↑ | Joint RMSE joints | Joint RMSE median ↓ | Joint RMSE P95 ↓ | Joint RMSE ≤0.1 pass (%) ↑ | Pose links | Pose trans/bbox P95 (%) ↓ | Pose rot P95 (deg) ↓ |")
    lines.append("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
    for d in summary["datasets"]:
        rmse = d["cross_sim_joint_rmse"]
        pose = d["cross_sim_pose_error"]
        pass_cell = "N/E" if rmse["pass_rate"] is None else f"{rmse['pass_rate']['percentage']:.2f}"
        lines.append(
            f"| {d['dataset_slug']} | {d['n']} | {d['all_three_load']['percentage']:.2f} "
            f"| {d['all_three_runtime']['percentage']:.2f} | {rmse['evaluated_joints']} "
            f"| {'N/E' if rmse['median'] is None else f'{rmse['median']:.4f}'} "
            f"| {'N/E' if rmse['p95'] is None else f'{rmse['p95']:.4f}'} | {pass_cell} "
            f"| {pose['evaluated_links']} | {pct_cell(pose['translation_over_bbox_p95'])} "
            f"| {'N/E' if pose['rotation_deg_p95'] is None else f'{pose['rotation_deg_p95']:.3f}'} |"
        )
    lines.append("")
    for sim in SIMULATORS:
        lines.append(f"## Per-simulator legacy diagnostics ({sim}), % of 200")
        lines.append("")
        lines.append("| Dataset | Load | Reset | Settling | Act | Limits | Drift | SimPass |")
        lines.append("|---|---:|---:|---:|---:|---:|---:|---:|")
        for d in summary["datasets"]:
            rates = d["per_simulator_runtime"][sim]
            lines.append(
                f"| {d['dataset_slug']} | "
                + " | ".join(f"{rates[name]['percentage']:.2f}" for name in RUNTIME_METRICS)
                + " |"
            )
        lines.append("")
    return "\n".join(lines) + "\n"


if __name__ == "__main__":
    raise SystemExit(main())
