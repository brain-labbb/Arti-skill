#!/usr/bin/env python3
"""Compare the diagnostic Articraft resample against the formal Table 5 results.

Reads the formal eight-dataset summary, the diagnostic resample summary and
per-asset records, and emits:

  * overall Table 5a / 5b side-by-side comparison (formal sample vs
    diagnostic sample of Articraft-200);
  * per-stratum (F export failures / W defective / R random) breakdown of
    import, passive stability, actuation tracking NRMSE (same formula as the
    official aggregation) and limit enforcement for each simulator.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence

SCRIPT_PATH = Path(__file__).resolve()
REPO_ROOT = SCRIPT_PATH.parents[2]
EXP_ROOT = REPO_ROOT / "exp"
FORMAL_FINAL = EXP_ROOT / "runtime/table5_v2_r2_formal_eight_datasets/final"
DIAG_ROOT = EXP_ROOT / "runtime/table5_v2_articraft_diagnostic_resample_20260830"
SIMULATORS = ("genesis", "pybullet", "mujoco")


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def minimum_jerk(progress: float) -> float:
    progress = max(0.0, min(1.0, progress))
    return 10.0 * progress**3 - 15.0 * progress**4 + 6.0 * progress**5


def tracking_nrmse(
    diagnostic: Mapping[str, Any], sample_steps: Sequence[int], ramp_steps: int
) -> float | None:
    trajectory = diagnostic.get("trajectory")
    if not isinstance(trajectory, Mapping):
        return None
    measured = trajectory.get("normalized_positions")
    if (
        trajectory.get("sample_steps") != list(sample_steps)
        or not isinstance(measured, list)
        or len(measured) != len(sample_steps)
        or any(not finite(value) for value in measured)
    ):
        return None
    target = [
        minimum_jerk(step / ramp_steps) if step <= ramp_steps else 1.0
        for step in sample_steps
    ]
    return math.sqrt(
        sum((float(a) - float(b)) ** 2 for a, b in zip(measured, target)) / len(target)
    )


def percentile(values: Sequence[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, int(math.ceil(fraction * len(ordered))) - 1)
    return ordered[max(index, 0)]


def import_pass(record: Mapping[str, Any]) -> bool:
    imported = record.get("evaluation", {}).get("v2", {}).get("import", {})
    return (
        imported.get("passed") is True
        and imported.get("criterion") == "native_simulator_asset_load"
    )


def stable_pass(record: Mapping[str, Any]) -> bool:
    stable = record.get("evaluation", {}).get("v2", {}).get("stable_rollout", {})
    if not import_pass(record) or stable.get("passed") is not True:
        return False
    checks = stable.get("checks")
    return (
        isinstance(checks, Mapping)
        and bool(checks)
        and all(value is True for value in checks.values())
        and stable.get("steps_completed") == stable.get("required_steps")
        and stable.get("finite_state_steps") == stable.get("required_steps")
        and stable.get("finite_pose_steps") == stable.get("required_steps")
    )


def fmt(value: Any, nd: int = 2) -> str:
    if value is None:
        return "N/E"
    return f"{value:.{nd}f}"


def dataset_block(summary: Mapping[str, Any], slug: str) -> Mapping[str, Any]:
    for dataset in summary["datasets"]:
        if dataset["dataset_slug"] == slug:
            return dataset
    raise SystemExit(f"dataset missing from summary: {slug}")


def main() -> int:
    formal = load_json(FORMAL_FINAL / "summary.json")
    diag = load_json(DIAG_ROOT / "final/summary.json")
    prepared = load_json(DIAG_ROOT / "prepared/manifest.json")
    protocol = prepared["protocol"]
    sample_steps = protocol["cross_simulator"]["joint_rmse"]["sample_steps"]
    ramp_steps = int(protocol["runtime"]["actuation"]["trajectory"]["ramp_steps"])
    strata: dict[str, str] = {}
    with (DIAG_ROOT / "selection/selected_assets.jsonl").open() as handle:
        for line in handle:
            row = json.loads(line)
            strata[row["asset_id"]] = row["stratum"]

    lines: list[str] = []
    lines.append("# Articraft diagnostic resample vs formal Table 5")
    lines.append("")
    formal_art = dataset_block(formal, "articraft_10k")
    diag_art = dataset_block(diag, "articraft_10k")
    lines.append("## Table 5a (genesis)")
    lines.append("")
    lines.append("| Metric | Formal sample | Diagnostic sample |")
    lines.append("|---|---:|---:|")
    for key, label in (
        ("import_success", "Import success (%)"),
        ("stable_rollout", "Stable rollout (%)"),
        ("dof_coverage", "DoF coverage (%)"),
        ("trajectory_coverage", "Trajectory coverage (%)"),
    ):
        lines.append(
            f"| {label} | {fmt(formal_art['table5a'][key]['percentage'])} "
            f"| {fmt(diag_art['table5a'][key]['percentage'])} |"
        )
    for key, label in (
        ("tracking_nrmse_p95", "Tracking NRMSE P95 (%)"),
        ("limit_violation_p95", "Limit violation P95 (%)"),
    ):
        f = formal_art["table5a"][key]
        d = diag_art["table5a"][key]
        lines.append(
            f"| {label} | {fmt(f['p95'])} ({f['evaluated_units']}/{f['candidate_units']}) "
            f"| {fmt(d['p95'])} ({d['evaluated_units']}/{d['candidate_units']}) |"
        )
    lines.append("")
    lines.append("## Table 5b (per simulator)")
    lines.append("")
    lines.append("| Simulator | Metric | Formal | Diagnostic |")
    lines.append("|---|---|---:|---:|")
    for sim in SIMULATORS:
        for key, label in (
            ("import_success", "import %"),
            ("stable_rollout", "stable %"),
        ):
            f = formal_art["table5b"][sim][key]
            d = diag_art["table5b"][sim][key]
            lines.append(
                f"| {sim} | {label} | {fmt(f['percentage'])} | {fmt(d['percentage'])} |"
            )
    lines.append("")
    lines.append("## Per-stratum breakdown (diagnostic sample)")
    lines.append("")
    for sim in SIMULATORS:
        assets_dir = DIAG_ROOT / "runs" / sim / "articraft_10k" / sim / "assets"
        records: dict[str, Mapping[str, Any]] = {}
        for path in sorted(assets_dir.glob("*.json")):
            record = load_json(path)
            records[str(record["identity"]["asset_id"])] = record
        lines.append(f"### {sim}")
        lines.append("")
        lines.append(
            "| Stratum | n | import % | stable % | actuated joints | act pass % | "
            "NRMSE median % | NRMSE P95 % | limit joints | limit pass % |"
        )
        lines.append("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
        for stratum in ("F", "C", "I", "R"):
            members = [
                asset_id for asset_id, member in strata.items() if member == stratum
            ]
            n = len(members)
            n_import = 0
            n_stable = 0
            actuated = 0
            act_passed = 0
            nrmse_values: list[float] = []
            limit_joints = 0
            limit_passed = 0
            for asset_id in members:
                record = records.get(asset_id)
                if record is None:
                    continue
                if import_pass(record):
                    n_import += 1
                if stable_pass(record):
                    n_stable += 1
                diagnostics = record.get("evaluation", {}).get("diagnostics", {})
                for row in diagnostics.get("actuation", []) or []:
                    actuated += 1
                    if row.get("passed") is True:
                        act_passed += 1
                    value = tracking_nrmse(row, sample_steps, ramp_steps)
                    if value is not None:
                        nrmse_values.append(value * 100.0)
                for row in diagnostics.get("limit_enforcement", []) or []:
                    limit_joints += 1
                    targets = row.get("targets") or []
                    if targets and all(
                        target.get("passed") is True for target in targets
                    ):
                        limit_passed += 1
            pct = lambda count: 100.0 * count / n if n else None
            lines.append(
                f"| {stratum} | {n} | {fmt(pct(n_import), 1)} | {fmt(pct(n_stable), 1)} "
                f"| {actuated} | {fmt(100.0 * act_passed / actuated if actuated else None, 1)} "
                f"| {fmt(percentile(nrmse_values, 0.5))} | {fmt(percentile(nrmse_values, 0.95))} "
                f"| {limit_joints} | {fmt(100.0 * limit_passed / limit_joints if limit_joints else None, 1)} |"
            )
        lines.append("")
    report = "\n".join(lines) + "\n"
    out_path = DIAG_ROOT / "comparison_report.md"
    out_path.write_text(report, encoding="utf-8")
    print(report)
    print(f"report written to {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
