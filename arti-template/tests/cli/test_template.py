from __future__ import annotations

from pathlib import Path

import pytest

from agent.template_sweep_pipeline import PipelineReport
from cli import template as template_cli


def test_parse_seed_spec_supports_ranges_and_lists() -> None:
    assert template_cli.parse_seed_spec("1-3") == [1, 2, 3]
    assert template_cli.parse_seed_spec("1,3,5-7") == [1, 3, 5, 6, 7]


def test_parse_seed_spec_rejects_empty_input() -> None:
    with pytest.raises(ValueError, match="At least one seed"):
        template_cli.parse_seed_spec("")


def test_batch_ferris_wheel_dry_run(tmp_path: Path) -> None:
    exit_code = template_cli.batch_ferris_wheel(
        tmp_path,
        seeds=[1, 2],
        agent="codex",
        category_slug=None,
        dry_run=True,
    )

    assert exit_code == 0
    assert not (tmp_path / "data" / "records").exists()


def _pipeline_report(verdict: str) -> PipelineReport:
    return PipelineReport(
        slug="turntable",
        stem="turntable",
        verdict=verdict,
        stopped_at=None if verdict == "pass" else "fast",
        pass_threshold=0.95,
        stages=[],
        repair_summary={},
        elapsed_s=0.12,
    )


def test_sweep_pipeline_cli_pass_returns_zero(monkeypatch, tmp_path: Path, capsys) -> None:
    captured = {}

    def fake_run_sweep_pipeline(**kwargs):
        captured.update(kwargs)
        return _pipeline_report("pass")

    monkeypatch.setattr(template_cli, "run_sweep_pipeline", fake_run_sweep_pipeline)

    exit_code = template_cli.main(
        ["--repo-root", str(tmp_path), "sweep-pipeline", "turntable", "--quiet"]
    )

    assert exit_code == 0
    assert captured["slug"] == "turntable"
    assert captured["stem"] == "turntable"
    assert captured["state_dir"] == tmp_path / ".articraft" / "template_sweep_state"
    assert '"verdict": "pass"' in capsys.readouterr().out


def test_sweep_pipeline_cli_empty_state_dir_disables_tracking(monkeypatch, tmp_path: Path) -> None:
    captured = {}

    def fake_run_sweep_pipeline(**kwargs):
        captured.update(kwargs)
        return _pipeline_report("pass")

    monkeypatch.setattr(template_cli, "run_sweep_pipeline", fake_run_sweep_pipeline)

    exit_code = template_cli.main(
        ["--repo-root", str(tmp_path), "sweep-pipeline", "turntable", "--state-dir", "", "--quiet"]
    )

    assert exit_code == 0
    assert captured["state_dir"] is None


def test_sweep_pipeline_cli_fail_returns_one(monkeypatch, tmp_path: Path) -> None:
    monkeypatch.setattr(
        template_cli,
        "run_sweep_pipeline",
        lambda **kwargs: _pipeline_report("fail"),
    )

    exit_code = template_cli.main(
        ["--repo-root", str(tmp_path), "sweep-pipeline", "turntable", "--quiet"]
    )

    assert exit_code == 1


def test_sweep_pipeline_cli_invalid_config_returns_two(monkeypatch, tmp_path: Path) -> None:
    def fake_run_sweep_pipeline(**kwargs):
        raise ValueError("bad template config")

    monkeypatch.setattr(template_cli, "run_sweep_pipeline", fake_run_sweep_pipeline)

    exit_code = template_cli.main(
        ["--repo-root", str(tmp_path), "sweep-pipeline", "turntable", "--quiet"]
    )

    assert exit_code == 2


def test_sweep_pipeline_cli_out_writes_json(monkeypatch, tmp_path: Path) -> None:
    monkeypatch.setattr(
        template_cli,
        "run_sweep_pipeline",
        lambda **kwargs: _pipeline_report("pass"),
    )
    out = tmp_path / "pipeline.json"

    exit_code = template_cli.main(
        [
            "--repo-root",
            str(tmp_path),
            "sweep-pipeline",
            "turntable",
            "--out",
            str(out),
            "--quiet",
        ]
    )

    assert exit_code == 0
    assert '"verdict": "pass"' in out.read_text(encoding="utf-8")


def test_sweep_pipeline_cli_quiet_keeps_final_summary(monkeypatch, tmp_path: Path, capsys) -> None:
    monkeypatch.setattr(
        template_cli,
        "run_sweep_pipeline",
        lambda **kwargs: _pipeline_report("pass"),
    )

    exit_code = template_cli.main(
        ["--repo-root", str(tmp_path), "sweep-pipeline", "turntable", "--quiet"]
    )

    assert exit_code == 0
    captured = capsys.readouterr()
    assert "sweep-pipeline turntable PASS" in captured.err
