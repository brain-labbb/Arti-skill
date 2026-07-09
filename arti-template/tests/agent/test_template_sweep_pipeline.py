from __future__ import annotations

import json
from pathlib import Path

from agent import template_sweep_pipeline
from agent.template_sweep import SeedOutcome
from agent.template_sweep_coverage import CoverageGateResult, CoverageGates
from agent.template_sweep_pipeline import audit_template_motion_tests, run_sweep_pipeline


def _fake_template(repo_root: Path, *, lines: int = 1200, source: str | None = None) -> None:
    path = repo_root / "agent" / "templates" / "stub_slug.py"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(source if source is not None else "x = 1\n" * lines, encoding="utf-8")


def _outcome(seed: int, *, verdict: str = "pass") -> SeedOutcome:
    if verdict == "fail":
        return SeedOutcome(
            seed=seed,
            verdict="fail",
            config={"seed": seed},
            failure_type="fail_if_parts_overlap_in_current_pose()",
            failure_type_normalized="fail_if_parts_overlap_in_current_pose",
            failure_details=f"seed-{seed}-overlap",
            elapsed_s=0.01,
        )
    return SeedOutcome(
        seed=seed,
        verdict="pass",
        config={"seed": seed},
        failure_type=None,
        failure_type_normalized=None,
        failure_details=None,
        elapsed_s=0.01,
    )


def _stub_seed_runner(monkeypatch, *, failing_seeds: set[int] | None = None):
    calls: list[list[int]] = []
    failures = failing_seeds or set()

    def fake_run_seed_outcomes(**kwargs):
        seeds = list(kwargs["seeds"])
        calls.append(seeds)
        return [_outcome(seed, verdict="fail" if seed in failures else "pass") for seed in seeds]

    monkeypatch.setattr(template_sweep_pipeline, "run_seed_outcomes", fake_run_seed_outcomes)
    return calls


def _stub_coverage_gate(monkeypatch, *, status: str = "pass") -> None:
    monkeypatch.setattr(
        "agent.template_sweep_coverage.evaluate_gates",
        lambda **kwargs: CoverageGates(
            module_topology_diversity=CoverageGateResult(
                name="module_topology_diversity",
                status=status,
                details={"distinct_count": 10 if status == "pass" else 1},
                reason="" if status == "pass" else "not enough distinct modular topology",
            )
        ),
    )


def test_pipeline_all_stages_pass_and_does_not_recompile_prior_seeds(monkeypatch, tmp_path: Path):
    _fake_template(tmp_path)
    calls = _stub_seed_runner(monkeypatch)
    _stub_coverage_gate(monkeypatch)

    report = run_sweep_pipeline(
        slug="stub_slug",
        stem="stub",
        repo_root=tmp_path,
        max_workers=1,
        state_dir=None,
        pass_threshold=1.0,
    )

    assert report.verdict == "pass"
    assert report.stopped_at is None
    assert report.motion_test_audit["status"] == "not_applicable"
    assert report.to_dict()["motion_test_audit"]["status"] == "not_applicable"
    assert [stage.status for stage in report.stages] == ["pass", "pass"]
    assert calls == [list(range(0, 16)), list(range(16, 36))]
    # The stub template exposes no config_from_seed, so the corner stage is
    # skipped (status unavailable) and never appears among the stages.
    payload = report.to_dict()
    assert payload["corner_seed_plan"]["status"] == "unavailable"
    assert "corner" not in [stage["name"] for stage in payload["stages"]]
    assert "axis_realization" in payload


def _stub_corner_plan(monkeypatch, *, seeds: list[int], status: str = "ok") -> None:
    from agent.template_axes import CornerSeedPlan

    monkeypatch.setattr(
        template_sweep_pipeline,
        "select_corner_seeds",
        lambda slug, base_seeds: CornerSeedPlan(status=status, seeds=list(seeds)),
    )


def test_pipeline_appends_corner_stage_and_passes_when_corners_clean(monkeypatch, tmp_path: Path):
    _fake_template(tmp_path)
    calls = _stub_seed_runner(monkeypatch)
    _stub_coverage_gate(monkeypatch)
    _stub_corner_plan(monkeypatch, seeds=[101, 202])

    report = run_sweep_pipeline(
        slug="stub_slug",
        stem="stub",
        repo_root=tmp_path,
        max_workers=1,
        state_dir=None,
        pass_threshold=1.0,
    )

    assert report.verdict == "pass"
    assert report.stopped_at is None
    assert [stage.name for stage in report.stages] == [
        "fast",
        "final",
        "corner",
    ]
    corner = report.stages[-1]
    assert corner.added_seeds == [101, 202]
    assert corner.cumulative_seeds == list(range(0, 36)) + [101, 202]
    assert corner.report.extra["corner_gate"]["status"] == "pass"
    assert calls[-1] == [101, 202]
    assert report.corner_seed_plan["seeds"] == [101, 202]


def test_pipeline_corner_failure_gates_even_when_rate_above_threshold(monkeypatch, tmp_path: Path):
    _fake_template(tmp_path)
    # Two corner seeds both fail; base 36 all pass, so the cumulative rate stays
    # at 36/38 ~= 0.947 (>= 0.90), but the corner gate must still fail the run.
    _stub_seed_runner(monkeypatch, failing_seeds={101, 202})
    _stub_coverage_gate(monkeypatch)
    _stub_corner_plan(monkeypatch, seeds=[101, 202])

    report = run_sweep_pipeline(
        slug="stub_slug",
        stem="stub",
        repo_root=tmp_path,
        max_workers=1,
        state_dir=None,
        pass_threshold=0.90,
    )

    assert report.verdict == "fail"
    assert report.stopped_at == "corner"
    corner_gate = report.stages[-1].report.extra["corner_gate"]
    assert corner_gate["status"] == "fail"
    assert corner_gate["failed_corner_seeds"] == [101, 202]
    assert report.repair_summary["failed_stage"] == "corner"


def test_pipeline_fast_stage_seed0_fail_skips_remaining_stages(monkeypatch, tmp_path: Path):
    # Seed 0 (the smoke seed) lives in the fast stage now; its failure stops
    # the ladder before the final stage compiles.
    _fake_template(tmp_path)
    calls = _stub_seed_runner(monkeypatch, failing_seeds={0})
    _stub_coverage_gate(monkeypatch)

    report = run_sweep_pipeline(
        slug="stub_slug",
        stem="stub",
        repo_root=tmp_path,
        max_workers=1,
        state_dir=None,
        pass_threshold=1.0,
    )

    assert report.verdict == "fail"
    assert report.stopped_at == "fast"
    assert [stage.status for stage in report.stages] == ["fail", "skipped"]
    assert calls == [list(range(0, 16))]
    assert report.repair_summary["failed_stage"] == "fast"
    assert report.repair_summary["failed_seeds"] == [0]
    assert report.repair_summary["top_failure_clusters"]


def test_pipeline_fast_fail_skips_later_stages(monkeypatch, tmp_path: Path):
    _fake_template(tmp_path)
    calls = _stub_seed_runner(monkeypatch, failing_seeds={2})
    _stub_coverage_gate(monkeypatch)

    report = run_sweep_pipeline(
        slug="stub_slug",
        stem="stub",
        repo_root=tmp_path,
        max_workers=1,
        state_dir=None,
        pass_threshold=1.0,
    )

    assert report.verdict == "fail"
    assert report.stopped_at == "fast"
    assert [stage.status for stage in report.stages] == ["fail", "skipped"]
    assert calls == [list(range(0, 16))]


def test_pipeline_final_fail_skips_corner_stage(monkeypatch, tmp_path: Path):
    # A failure in the final ladder stage stops the run before the corner
    # stage is compiled, even though a corner plan is available.
    _fake_template(tmp_path)
    calls = _stub_seed_runner(monkeypatch, failing_seeds={20})
    _stub_coverage_gate(monkeypatch)
    _stub_corner_plan(monkeypatch, seeds=[101, 202])

    report = run_sweep_pipeline(
        slug="stub_slug",
        stem="stub",
        repo_root=tmp_path,
        max_workers=1,
        state_dir=None,
        pass_threshold=1.0,
    )

    assert report.verdict == "fail"
    assert report.stopped_at == "final"
    assert [stage.status for stage in report.stages] == ["pass", "fail"]
    assert "corner" not in [stage.name for stage in report.stages]
    assert calls == [list(range(0, 16)), list(range(16, 36))]


def test_pipeline_final_fail_returns_fail(monkeypatch, tmp_path: Path):
    _fake_template(tmp_path)
    calls = _stub_seed_runner(monkeypatch, failing_seeds={35})
    _stub_coverage_gate(monkeypatch)

    report = run_sweep_pipeline(
        slug="stub_slug",
        stem="stub",
        repo_root=tmp_path,
        max_workers=1,
        state_dir=None,
        pass_threshold=1.0,
    )

    assert report.verdict == "fail"
    assert report.stopped_at == "final"
    assert [stage.status for stage in report.stages] == ["pass", "fail"]
    assert calls == [list(range(0, 16)), list(range(16, 36))]


def test_pipeline_coverage_gate_is_report_only_and_does_not_flip_verdict(
    monkeypatch, tmp_path: Path
):
    # module_topology_diversity is REPORT-ONLY: a failing gate must NOT flip the
    # verdict (verdict follows pass_rate), but the failing gate must still be
    # surfaced in the stage report's coverage_gates for visibility.
    _fake_template(tmp_path)
    _stub_seed_runner(monkeypatch)
    _stub_coverage_gate(monkeypatch, status="fail")

    report = run_sweep_pipeline(
        slug="stub_slug",
        stem="stub",
        repo_root=tmp_path,
        max_workers=1,
        state_dir=None,
    )

    assert report.verdict == "pass"
    assert report.stopped_at is None
    assert report.repair_summary == {}
    final_gates = report.stages[-1].report.to_dict()["coverage_gates"]
    assert final_gates["module_topology_diversity"]["status"] == "fail"


def test_pipeline_state_tracking_updates_once_per_run(monkeypatch, tmp_path: Path):
    _fake_template(tmp_path)
    _stub_seed_runner(monkeypatch, failing_seeds={2})
    _stub_coverage_gate(monkeypatch)
    state_dir = tmp_path / "state"

    report = run_sweep_pipeline(
        slug="stub_slug",
        stem="stub",
        repo_root=tmp_path,
        max_workers=1,
        state_dir=state_dir,
        pass_threshold=1.0,
    )

    assert report.stopped_at == "fast"
    state = json.loads((state_dir / "stub_slug.json").read_text(encoding="utf-8"))
    assert len(state["sweep_history"]) == 1


def test_motion_audit_passes_with_sampled_collision_and_targeted_pose(tmp_path: Path):
    _fake_template(
        tmp_path,
        source="""
from sdk import ArticulationType

def build():
    joint_type = ArticulationType.REVOLUTE
    return joint_type

def run_stub_slug_tests(model, config):
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=32)
    with ctx.pose({"door_hinge": 1.0}):
        pass
""",
    )

    audit = audit_template_motion_tests(slug="stub_slug", repo_root=tmp_path)

    assert audit["status"] == "pass"
    assert audit["has_nonfixed_joint_hint"] is True
    assert audit["has_sampled_collision_check"] is True
    assert audit["has_targeted_pose_check"] is True


def test_motion_audit_warns_when_nonfixed_joint_has_no_motion_tests(tmp_path: Path):
    _fake_template(
        tmp_path,
        source="""
from sdk import ArticulationType

def build():
    return ArticulationType.PRISMATIC
""",
    )

    audit = audit_template_motion_tests(slug="stub_slug", repo_root=tmp_path)

    assert audit["status"] == "warn"
    assert "sampled collision check" in audit["details"]
    assert "targeted ctx.pose" in audit["details"]


def test_motion_audit_passes_with_exemption_and_targeted_pose(tmp_path: Path):
    _fake_template(
        tmp_path,
        source="""
from sdk import ArticulationType

# sampled-pose exemption: high-DOF copied slats make broad Cartesian sampling noisy.
def build():
    return ArticulationType.REVOLUTE

def run_stub_slug_tests(model, config):
    with ctx.pose({"panel_hinge": 0.8}):
        pass
""",
    )

    audit = audit_template_motion_tests(slug="stub_slug", repo_root=tmp_path)

    assert audit["status"] == "pass"
    assert audit["has_sampled_collision_check"] is False
    assert audit["has_sampled_pose_exemption"] is True
    assert "exempted" in audit["details"]


def test_motion_audit_is_not_applicable_without_nonfixed_joint_hint(tmp_path: Path):
    _fake_template(tmp_path, source="x = 1\n")

    audit = audit_template_motion_tests(slug="stub_slug", repo_root=tmp_path)

    assert audit["status"] == "not_applicable"
    assert audit["has_nonfixed_joint_hint"] is False


def test_failure_triage_groups_and_prioritizes() -> None:
    from agent.template_sweep import SeedOutcome
    from agent.template_sweep_pipeline import build_allowance_audit, build_failure_triage

    def _fail(seed, failures, allowances=()):
        return SeedOutcome(
            seed=seed,
            verdict="fail",
            config={},
            failure_type=failures[0]["check"],
            failure_type_normalized=failures[0]["check_normalized"],
            failure_details=failures[0]["details"],
            elapsed_s=0.1,
            failures=tuple(failures),
            allowances=tuple(allowances),
        )

    overlap = {
        "check": "harness_motion_qc(joints=2,samples=32)",
        "check_normalized": "harness_motion_qc",
        "details": (
            "pair=('barrel','wedge') min_depth=0.09 vol=0.007 "
            "pose={'elev': 0.38} built_by=_build_wedge"
        ),
    }
    origin = {
        "check": "fail_if_articulation_origin_far_from_geometry(tol=0.015)",
        "check_normalized": "fail_if_articulation_origin_far_from_geometry",
        "details": "joint='barrel_elevation' dist_parent=0.35",
    }
    outcomes = [
        _fail(0, [origin, overlap], ["allow_overlap('a', 'b'): ok"]),
        _fail(1, [overlap], ["allow_overlap('a', 'b'): ok"]),
    ]
    triage = build_failure_triage(outcomes)
    assert [g["check_normalized"] for g in triage] == [
        "harness_motion_qc",
        "fail_if_articulation_origin_far_from_geometry",
    ]  # motion (priority 4) before origin (priority 5)
    motion = triage[0]
    assert motion["subject"] == "barrel↔wedge"
    assert motion["seeds"] == [0, 1] and motion["seed_count"] == 2
    assert motion["worst_min_depth"] == 0.09
    assert motion["built_by"] == "_build_wedge"
    assert "elev" in motion["pose"]
    assert triage[1]["subject"] == "barrel_elevation"

    audit = build_allowance_audit(outcomes)
    assert audit["distinct"] == 1
    assert audit["allowances"][0]["seed_count"] == 2
    assert audit["suspicious"][0]["flag"] == "weak_reason"  # "ok" is too short


def test_allowance_audit_flags_new_since_last_pass() -> None:
    from agent.template_sweep import SeedOutcome
    from agent.template_sweep_pipeline import build_allowance_audit

    outcome = SeedOutcome(
        seed=0,
        verdict="pass",
        config={},
        failure_type=None,
        failure_type_normalized=None,
        failure_details=None,
        elapsed_s=0.1,
        allowances=(
            "allow_overlap('hub', 'axle'): captured coaxial axle through the wheel hub bore",
            "allow_overlap('tray', 'frame'): whitewash",
        ),
    )
    baseline = {"allow_overlap('hub', 'axle'): captured coaxial axle through the wheel hub bore"}
    audit = build_allowance_audit([outcome], baseline=baseline)
    assert audit["baseline"] == "last_passing_run"
    assert audit["new_allowances"] == ["allow_overlap('tray', 'frame'): whitewash"]
    flags = {(s["allowance"], s["flag"]) for s in audit["suspicious"]}
    assert ("allow_overlap('tray', 'frame'): whitewash", "new_since_last_pass") in flags
    assert ("allow_overlap('tray', 'frame'): whitewash", "weak_reason") in flags

    no_baseline = build_allowance_audit([outcome], baseline=None)
    assert no_baseline["baseline"] == "unavailable"
    assert "new_allowances" not in no_baseline


def test_sweep_state_round_trips_passing_allowances(tmp_path) -> None:
    from agent.template_sweep_state import SweepStateRecord, load_state, save_state

    record = SweepStateRecord(
        slug="demo",
        last_updated_at="2026-07-03T00:00:00Z",
        passing_allowances=["allow_overlap('a', 'b'): captured pin"],
    )
    save_state(tmp_path, record)
    loaded = load_state(tmp_path, "demo")
    assert loaded is not None
    assert loaded.passing_allowances == ["allow_overlap('a', 'b'): captured pin"]
