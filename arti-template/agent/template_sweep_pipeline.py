"""Incremental multi-stage sweep pipeline for procedural templates."""

from __future__ import annotations

import json
import re
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Iterable

from agent.template_axes import (
    DEFAULT_MAX_CORNER_FAILURES,
    CornerSeedPlan,
    build_axis_realization,
    select_corner_seeds,
)
from agent.template_sweep import (
    DEFAULT_PASS_THRESHOLD,
    SeedOutcome,
    SweepReport,
    build_sweep_report_from_outcomes,
    record_sweep_state,
    run_seed_outcomes,
)

PIPELINE_NEXT_ACTION = (
    "Fix the largest failure cluster first, rerun the pipeline from seed 0 after editing "
    "the template, and do not run viewer batch until the pipeline passes."
)

NONFIXED_JOINT_HINTS: tuple[str, ...] = (
    "ArticulationType.REVOLUTE",
    "ArticulationType.PRISMATIC",
    "ArticulationType.CONTINUOUS",
    "REVOLUTE",
    "PRISMATIC",
    "CONTINUOUS",
)

SAMPLED_POSE_EXEMPTION_MARKERS: tuple[str, ...] = (
    "sampled-pose exemption",
    "sampled pose exemption",
    "sampled_pose_exemption",
)


@dataclass(slots=True, frozen=True)
class PipelineStageSpec:
    name: str
    added_seeds: list[int]
    cumulative_seeds: list[int]


# Two ladder stages + corner. fast (0-15) runs fully parallel, so its
# wall-clock ~= one seed's compile; agent-side smoke probes (AUTHORING §C)
# replace the old seed0 compute guard. final ends at 36 cumulative seeds —
# statistically sufficient for the per-key coverage floor, which is evaluated
# there (the old ≥20-seed activation threshold is retired with the ladder).
# Corner seeds (constructed extremes) stay: they are the long-tail catchers.
PIPELINE_STAGES: tuple[PipelineStageSpec, ...] = (
    PipelineStageSpec("fast", list(range(0, 16)), list(range(0, 16))),
    PipelineStageSpec("final", list(range(16, 36)), list(range(0, 36))),
)


@dataclass(slots=True)
class PipelineStageResult:
    name: str
    added_seeds: list[int]
    cumulative_seeds: list[int]
    status: str
    report: SweepReport | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "added_seeds": list(self.added_seeds),
            "cumulative_seeds": list(self.cumulative_seeds),
            "status": self.status,
            "report": None if self.report is None else self.report.to_dict(),
        }


@dataclass(slots=True)
class PipelineReport:
    slug: str
    stem: str
    verdict: str
    stopped_at: str | None
    pass_threshold: float
    stages: list[PipelineStageResult]
    motion_test_audit: dict[str, Any] = field(default_factory=dict)
    repair_summary: dict[str, Any] = field(default_factory=dict)
    corner_seed_plan: dict[str, Any] = field(default_factory=dict)
    axis_realization: dict[str, Any] = field(default_factory=dict)
    failure_triage: list[dict[str, Any]] = field(default_factory=list)
    allowance_audit: dict[str, Any] = field(default_factory=dict)
    elapsed_s: float = 0.0

    def to_dict(self) -> dict[str, Any]:
        return {
            "slug": self.slug,
            "stem": self.stem,
            "verdict": self.verdict,
            "stopped_at": self.stopped_at,
            "pass_threshold": self.pass_threshold,
            "stages": [stage.to_dict() for stage in self.stages],
            "motion_test_audit": dict(self.motion_test_audit),
            "repair_summary": dict(self.repair_summary),
            "corner_seed_plan": dict(self.corner_seed_plan),
            "axis_realization": dict(self.axis_realization),
            "failure_triage": [dict(group) for group in self.failure_triage],
            "allowance_audit": dict(self.allowance_audit),
            "elapsed_s": self.elapsed_s,
        }


def pipeline_report_to_json(report: PipelineReport, *, indent: int | None = 2) -> str:
    return json.dumps(report.to_dict(), indent=indent, sort_keys=False, default=str)


def write_pipeline_report(report: PipelineReport, *, out_path: Path) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(pipeline_report_to_json(report), encoding="utf-8")


def build_repair_summary(stage_name: str, report: SweepReport) -> dict[str, Any]:
    report_payload = report.to_dict()
    coverage_gates = report_payload.get("coverage_gates") or {}
    failing_gates = {
        name: gate
        for name, gate in coverage_gates.items()
        if isinstance(gate, dict) and gate.get("status") not in {"pass", "skipped"}
    }
    failed_seeds = [outcome.seed for outcome in report.failed_outcomes]
    return {
        "failed_stage": stage_name,
        "failed_seeds": failed_seeds,
        "failing_coverage_gates": failing_gates,
        "top_failure_clusters": report_payload.get("failure_clusters", [])[:3],
        "failure_triage": build_failure_triage(report.failed_outcomes)[:5],
        "escalation": report_payload.get("escalation"),
        "next_action": PIPELINE_NEXT_ACTION,
    }


# Repair priority: what blocks everything first, structure next, placement and
# motion after — the order an agent should fix in.
_TRIAGE_PRIORITY: tuple[tuple[str, int], ...] = (
    ("compile_timeout", 0),
    ("subprocess_crash", 0),
    ("fail_if_isolated_parts", 1),
    ("fail_if_part_contains_disconnected_geometry_islands", 2),
    ("fail_if_parts_overlap_in_current_pose", 3),
    ("harness_motion_qc", 4),
    ("fail_if_articulation_origin_far_from_geometry", 5),
    ("fail_if_joint_mating_has_gap", 6),
)

_SUBJECT_PATTERNS = (
    re.compile(r"pair=\('([^']+)',\s*'([^']+)'\)"),
    re.compile(r"part='([^']+)'"),
    re.compile(r"joint='([^']+)'"),
)
_MIN_DEPTH_RE = re.compile(r"min_depth=([0-9.eE+-]+)")
_BUILT_BY_RE = re.compile(r"built_by=(\w+)")
_POSE_RE = re.compile(r"pose=(\{[^}]*\})")


def _triage_priority(check_normalized: str) -> int:
    for prefix, rank in _TRIAGE_PRIORITY:
        if check_normalized.startswith(prefix):
            return rank
    if check_normalized.startswith("COMPILE") or check_normalized.endswith("Error"):
        return 0
    return 7


def _triage_subject(details: str) -> str:
    for pattern in _SUBJECT_PATTERNS:
        match = pattern.search(details)
        if match:
            return "↔".join(match.groups()) if len(match.groups()) > 1 else match.group(1)
    return ""


def build_failure_triage(outcomes: Iterable[SeedOutcome]) -> list[dict[str, Any]]:
    """Group every structured failure across seeds by (check, subject).

    Each group keeps the seeds it hits, the worst penetration depth, the pose
    and builder attribution when present, and a representative detail excerpt
    — ordered by repair priority (build-blockers first) then breadth.
    """
    groups: dict[tuple[str, str], dict[str, Any]] = {}
    for outcome in outcomes:
        entries = outcome.failures or ()
        if not entries and outcome.failure_type:
            entries = (
                {
                    "check": outcome.failure_type,
                    "check_normalized": outcome.failure_type_normalized or outcome.failure_type,
                    "details": outcome.failure_details or "",
                },
            )
        for entry in entries:
            details = str(entry.get("details") or "")
            normalized = str(entry.get("check_normalized") or entry.get("check") or "unknown")
            subject = _triage_subject(details)
            key = (normalized, subject)
            group = groups.get(key)
            if group is None:
                group = groups[key] = {
                    "check": entry.get("check"),
                    "check_normalized": normalized,
                    "subject": subject,
                    "seeds": [],
                    "priority": _triage_priority(normalized),
                    "worst_min_depth": None,
                    "pose": None,
                    "built_by": None,
                    "example": details[:600],
                }
            if outcome.seed not in group["seeds"]:
                group["seeds"].append(outcome.seed)
            for raw_depth in _MIN_DEPTH_RE.findall(details):
                try:
                    depth = float(raw_depth)
                except ValueError:
                    continue
                if group["worst_min_depth"] is None or depth > group["worst_min_depth"]:
                    group["worst_min_depth"] = depth
                    pose_match = _POSE_RE.search(details)
                    if pose_match:
                        group["pose"] = pose_match.group(1)
            if group["built_by"] is None:
                built_by = _BUILT_BY_RE.search(details)
                if built_by:
                    group["built_by"] = built_by.group(1)
    ordered = sorted(
        groups.values(), key=lambda g: (g["priority"], -len(g["seeds"]), g["check_normalized"])
    )
    for group in ordered:
        group["seed_count"] = len(group["seeds"])
        group["seeds"] = sorted(group["seeds"])[:20]
    return ordered


_ALLOWANCE_REASON_RE = re.compile(r"\):\s*(.*)$")


def build_allowance_audit(
    outcomes: Iterable[SeedOutcome],
    *,
    baseline: set[str] | None = None,
) -> dict[str, Any]:
    """Aggregate declared allowances across seeds and flag likely abuse.

    Report-only: `allow_overlap` is legitimate for captured/coaxial mechanism
    contact, but it is also the cheapest way to silence a genuine 穿模. The
    audit lists every distinct allowance with its seed reach, flags overlap
    allowances with weak reasons, and — when a last-passing ``baseline`` set
    is available from sweep state — flags every allowance that is NEW since
    the template last passed (the prime whitewash signal: a repair that adds
    an allowance instead of fixing geometry shows up here).
    """
    counts: dict[str, int] = {}
    for outcome in outcomes:
        for allowance in outcome.allowances or ():
            counts[allowance] = counts.get(allowance, 0) + 1
    suspicious: list[dict[str, str]] = []
    for allowance in counts:
        if not allowance.startswith("allow_overlap("):
            continue
        reason_match = _ALLOWANCE_REASON_RE.search(allowance)
        reason = (reason_match.group(1) if reason_match else "").strip()
        if len(reason) < 12:
            suspicious.append({"allowance": allowance, "flag": "weak_reason"})
    audit: dict[str, Any] = {
        "distinct": len(counts),
        "allowances": [
            {"allowance": allowance, "seed_count": count}
            for allowance, count in sorted(counts.items(), key=lambda kv: (-kv[1], kv[0]))
        ][:100],
        "suspicious": suspicious,
    }
    if baseline is None:
        audit["baseline"] = "unavailable"
    else:
        new_allowances = sorted(set(counts) - baseline)
        removed_allowances = sorted(baseline - set(counts))
        audit["baseline"] = "last_passing_run"
        audit["new_allowances"] = new_allowances
        audit["removed_allowances_count"] = len(removed_allowances)
        for allowance in new_allowances:
            if allowance.startswith("allow_overlap("):
                suspicious.append({"allowance": allowance, "flag": "new_since_last_pass"})
    return audit


def audit_template_motion_tests(*, slug: str, repo_root: Path | None = None) -> dict[str, Any]:
    """Lightweight source audit for new-template dynamic motion test coverage."""
    repo_root = repo_root or Path(__file__).resolve().parents[1]
    template_path = repo_root / "agent" / "templates" / f"{slug}.py"
    if not template_path.exists():
        return {
            "status": "missing_template",
            "template_path": str(template_path),
            "has_nonfixed_joint_hint": False,
            "has_sampled_collision_check": False,
            "has_targeted_pose_check": False,
            "has_sampled_pose_exemption": False,
            "details": "template source was not found; motion test coverage could not be audited",
        }

    source = template_path.read_text(encoding="utf-8")
    lowered = source.lower()
    has_nonfixed_joint_hint = any(hint in source for hint in NONFIXED_JOINT_HINTS)
    has_sampled_collision_check = "fail_if_parts_overlap_in_sampled_poses" in source
    has_targeted_pose_check = "ctx.pose(" in source
    has_sampled_pose_exemption = any(marker in lowered for marker in SAMPLED_POSE_EXEMPTION_MARKERS)

    if not has_nonfixed_joint_hint:
        status = "not_applicable"
        details = "no non-fixed joint hint was found in the template source"
    elif has_sampled_collision_check and has_targeted_pose_check:
        status = "pass"
        details = "non-fixed joint hints have sampled collision and targeted pose coverage"
    elif has_sampled_pose_exemption and has_targeted_pose_check:
        status = "pass"
        details = "sampled collision is explicitly exempted and targeted pose coverage is present"
    else:
        status = "warn"
        missing: list[str] = []
        if not has_sampled_collision_check and not has_sampled_pose_exemption:
            missing.append("sampled collision check or sampled-pose exemption")
        if not has_targeted_pose_check:
            missing.append("targeted ctx.pose(...) check")
        details = "non-fixed joint hints found, but missing " + " and ".join(missing)

    return {
        "status": status,
        "template_path": str(template_path),
        "has_nonfixed_joint_hint": has_nonfixed_joint_hint,
        "has_sampled_collision_check": has_sampled_collision_check,
        "has_targeted_pose_check": has_targeted_pose_check,
        "has_sampled_pose_exemption": has_sampled_pose_exemption,
        "details": details,
    }


def run_sweep_pipeline(
    *,
    slug: str,
    stem: str,
    sdk_package: str = "sdk",
    pass_threshold: float = DEFAULT_PASS_THRESHOLD,
    max_workers: int | None = None,
    repo_root: Path | None = None,
    progress: Callable[[SeedOutcome], None] | None = None,
    stage_progress: Callable[[str, PipelineStageResult], None] | None = None,
    state_dir: Path | None = None,
    compile_timeout_s: float = 0.0,
    motion_qc: bool = True,
    corner_seeds: bool = True,
) -> PipelineReport:
    repo_root = repo_root or Path(__file__).resolve().parents[1]
    started = time.monotonic()
    motion_test_audit = audit_template_motion_tests(slug=slug, repo_root=repo_root)
    # Capture the last-passing allowance baseline BEFORE any stage writes
    # state (the terminal stage refreshes it on a pass, which would make the
    # diff vacuously empty).
    allowance_baseline: set[str] | None = None
    if state_dir is not None:
        try:
            from agent.template_sweep_state import load_state

            previous_state = load_state(state_dir, slug)
            if previous_state is not None:
                allowance_baseline = set(previous_state.passing_allowances)
        except Exception:  # noqa: BLE001 - audit input only, never sinks the run
            allowance_baseline = None
    corner_plan: CornerSeedPlan | None = None
    if corner_seeds:
        try:
            corner_plan = select_corner_seeds(slug, base_seeds=PIPELINE_STAGES[-1].cumulative_seeds)
        except Exception as exc:  # noqa: BLE001 - selection must never sink the pipeline
            corner_plan = CornerSeedPlan(status="error", reason=f"{type(exc).__name__}: {exc}")
    has_corner_stage = (
        corner_plan is not None and corner_plan.status == "ok" and bool(corner_plan.seeds)
    )
    outcomes_by_seed: dict[int, SeedOutcome] = {}
    stages: list[PipelineStageResult] = []
    stopped_at: str | None = None
    repair_summary: dict[str, Any] = {}

    for index, spec in enumerate(PIPELINE_STAGES):
        if stage_progress is not None:
            stage_progress(
                "start",
                PipelineStageResult(
                    name=spec.name,
                    added_seeds=list(spec.added_seeds),
                    cumulative_seeds=list(spec.cumulative_seeds),
                    status="running",
                    report=None,
                ),
            )
        stage_started = time.monotonic()
        added_outcomes = run_seed_outcomes(
            slug=slug,
            stem=stem,
            seeds=spec.added_seeds,
            sdk_package=sdk_package,
            max_workers=max_workers,
            repo_root=repo_root,
            progress=progress,
            compile_timeout_s=compile_timeout_s,
            motion_qc=motion_qc,
        )
        outcomes_by_seed.update({outcome.seed: outcome for outcome in added_outcomes})
        cumulative_outcomes = [outcomes_by_seed[seed] for seed in spec.cumulative_seeds]
        is_final_stage = index == len(PIPELINE_STAGES) - 1
        applies_state = is_final_stage and not has_corner_stage
        report = build_sweep_report_from_outcomes(
            slug=slug,
            stem=stem,
            seeds=spec.cumulative_seeds,
            outcomes=cumulative_outcomes,
            pass_threshold=pass_threshold,
            repo_root=repo_root,
            state_dir=state_dir if applies_state else None,
            elapsed_s=time.monotonic() - stage_started,
        )
        if report.verdict == "fail" and not applies_state and state_dir is not None:
            report = build_sweep_report_from_outcomes(
                slug=slug,
                stem=stem,
                seeds=spec.cumulative_seeds,
                outcomes=cumulative_outcomes,
                pass_threshold=pass_threshold,
                repo_root=repo_root,
                state_dir=state_dir,
                elapsed_s=time.monotonic() - stage_started,
            )
        status = report.verdict
        stage_result = PipelineStageResult(
            name=spec.name,
            added_seeds=list(spec.added_seeds),
            cumulative_seeds=list(spec.cumulative_seeds),
            status=status,
            report=report,
        )
        stages.append(stage_result)
        if stage_progress is not None:
            stage_progress("end", stage_result)
        if status == "fail":
            stopped_at = spec.name
            repair_summary = build_repair_summary(spec.name, report)
            for skipped in PIPELINE_STAGES[index + 1 :]:
                skipped_result = PipelineStageResult(
                    name=skipped.name,
                    added_seeds=list(skipped.added_seeds),
                    cumulative_seeds=list(skipped.cumulative_seeds),
                    status="skipped",
                    report=None,
                )
                stages.append(skipped_result)
                if stage_progress is not None:
                    stage_progress("end", skipped_result)
            break

    if stopped_at is None and has_corner_stage and corner_plan is not None:
        corner_added = list(corner_plan.seeds)
        corner_cumulative = list(PIPELINE_STAGES[-1].cumulative_seeds) + corner_added
        if stage_progress is not None:
            stage_progress(
                "start",
                PipelineStageResult(
                    name="corner",
                    added_seeds=list(corner_added),
                    cumulative_seeds=list(corner_cumulative),
                    status="running",
                    report=None,
                ),
            )
        stage_started = time.monotonic()
        added_outcomes = run_seed_outcomes(
            slug=slug,
            stem=stem,
            seeds=corner_added,
            sdk_package=sdk_package,
            max_workers=max_workers,
            repo_root=repo_root,
            progress=progress,
            compile_timeout_s=compile_timeout_s,
            motion_qc=motion_qc,
        )
        outcomes_by_seed.update({outcome.seed: outcome for outcome in added_outcomes})
        cumulative_outcomes = [outcomes_by_seed[seed] for seed in corner_cumulative]
        # Build WITHOUT state_dir: the corner gate below may flip the verdict,
        # and streak/allowance state must record the FINAL verdict (a
        # corner-only failure recorded as a pass would refresh the allowance
        # baseline with unvetted allowances).
        report = build_sweep_report_from_outcomes(
            slug=slug,
            stem=stem,
            seeds=corner_cumulative,
            outcomes=cumulative_outcomes,
            pass_threshold=pass_threshold,
            repo_root=repo_root,
            state_dir=None,
            elapsed_s=time.monotonic() - stage_started,
        )
        corner_failed = sorted(
            outcome.seed for outcome in added_outcomes if outcome.verdict == "fail"
        )
        corner_gate = {
            "corner_seed_count": len(corner_added),
            "failed_corner_seeds": corner_failed,
            "max_corner_failures": DEFAULT_MAX_CORNER_FAILURES,
            "status": ("fail" if len(corner_failed) > DEFAULT_MAX_CORNER_FAILURES else "pass"),
        }
        report.extra["corner_gate"] = corner_gate
        if corner_gate["status"] == "fail" and report.verdict == "pass":
            # Corner seeds are deliberately extreme: systematic breakage there
            # must gate even when the cumulative rate stays above threshold.
            report.verdict = "fail"
        if state_dir is not None:
            record_sweep_state(report, cumulative_outcomes, state_dir=state_dir)
        corner_result = PipelineStageResult(
            name="corner",
            added_seeds=list(corner_added),
            cumulative_seeds=list(corner_cumulative),
            status=report.verdict,
            report=report,
        )
        stages.append(corner_result)
        if stage_progress is not None:
            stage_progress("end", corner_result)
        if report.verdict == "fail":
            stopped_at = "corner"
            repair_summary = build_repair_summary("corner", report)

    try:
        axis_realization = build_axis_realization(slug, list(outcomes_by_seed.values()))
    except Exception as exc:  # noqa: BLE001 - report-only section never sinks the run
        axis_realization = {"status": "error", "reason": f"{type(exc).__name__}: {exc}"}

    all_outcomes = list(outcomes_by_seed.values())
    try:
        failure_triage = build_failure_triage(
            [outcome for outcome in all_outcomes if outcome.verdict != "pass"]
        )
    except Exception as exc:  # noqa: BLE001 - report-only section never sinks the run
        failure_triage = [{"status": "error", "reason": f"{type(exc).__name__}: {exc}"}]
    try:
        allowance_audit = build_allowance_audit(all_outcomes, baseline=allowance_baseline)
    except Exception as exc:  # noqa: BLE001 - report-only section never sinks the run
        allowance_audit = {"status": "error", "reason": f"{type(exc).__name__}: {exc}"}

    verdict = "fail" if stopped_at is not None else "pass"
    return PipelineReport(
        slug=slug,
        stem=stem,
        verdict=verdict,
        stopped_at=stopped_at,
        pass_threshold=pass_threshold,
        stages=stages,
        motion_test_audit=motion_test_audit,
        repair_summary=repair_summary,
        corner_seed_plan=(
            corner_plan.to_dict() if corner_plan is not None else {"status": "disabled"}
        ),
        axis_realization=axis_realization,
        failure_triage=failure_triage,
        allowance_audit=allowance_audit,
        elapsed_s=round(time.monotonic() - started, 3),
    )
