from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys

import pytest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import run_table2sup_full_release as runner  # noqa: E402


def _row(
    tmp_path: Path,
    *,
    asset_id: str = "box/1001/box",
    source_root_relative: str = "box/urdf/box/1001/box.urdf",
    joint_count: int = 2,
) -> dict[str, object]:
    package = tmp_path / "box" / "urdf" / "box" / "1001"
    package.mkdir(parents=True)
    urdf = package / "box.urdf"
    urdf.write_text(
        """<robot name="fixture">
  <link name="base"><visual><geometry><box size="1 1 1"/></geometry></visual>
    <collision><geometry><box size="1 1 1"/></geometry></collision></link>
  <link name="door"><visual><geometry><box size="1 1 1"/></geometry></visual>
    <collision><geometry><box size="1 1 1"/></geometry></collision></link>
  <joint name="hinge" type="revolute"><parent link="base"/><child link="door"/>
    <limit lower="0" upper="1" effort="1" velocity="1"/>
    <dynamics damping="0" friction="0"/></joint>
</robot>
""",
        encoding="utf-8",
    )
    digest = hashlib.sha256(urdf.read_bytes()).hexdigest()
    return {
        "asset_id": asset_id,
        "ordinal": 0,
        "category": "box",
        "raw_category": "box",
        "source_path": str(package),
        "primary_urdf_path": str(urdf),
        "primary_urdf_relative_path": source_root_relative,
        "primary_urdf_sha256": digest,
        "joint_count": joint_count,
    }


def test_package_relative_urdf_uses_absolute_binding_for_source_root_paths(
    tmp_path: Path,
) -> None:
    row = _row(tmp_path)
    assert runner.package_relative_urdf(row) == "box.urdf"


def test_build_jobs_supports_full_release_rows_and_preserves_denominators(
    tmp_path: Path,
) -> None:
    row = _row(tmp_path, joint_count=3)
    manifest = {
        "schema_version": "table123_full_release_manifest_v1",
        "dataset": "Infinigen-Sim",
        "N_eval": 1,
        "J_eval": 3,
        "rows": [row],
    }
    jobs = runner.build_jobs_from_manifest(manifest, source_kind="infinigen")
    assert jobs == [
        {
            "selection_index": 0,
            "asset_id": "box/1001/box",
            "category": "box",
            "package": row["source_path"],
            "primary_urdf_relative_path": "box.urdf",
            "expected_primary_urdf_sha256": row["primary_urdf_sha256"],
            "expected_movable_joints": 3,
        }
    ]


def test_aggregate_records_keeps_error_assets_and_joint_denominator() -> None:
    def record(asset_id: str, status: str, *, joints: int, passed: int) -> dict[str, object]:
        return {
            "asset_id": asset_id,
            "status": status,
            "parse": {"success": status == "completed"},
            "table2_supplementary": {
                "visual_bearing_collision_coverage": {
                    "asset_pass": status == "completed",
                    "visual_bearing_links_declared": 2 if status == "completed" else 0,
                    "covered_visual_bearing_links": 2 if status == "completed" else 0,
                    "link_extraction_complete": status == "completed",
                },
                "joint_limit_portability": {
                    "joints_intended": joints,
                    "joints_extracted": joints if status == "completed" else 0,
                    "joints_passed": passed,
                },
                "joint_dynamics_coverage": {
                    "joints_intended": joints,
                    "joints_extracted": joints if status == "completed" else 0,
                    "joints_covered": passed,
                },
                "placeholder_mass_incidence": {
                    "dynamic_links": 2 if status == "completed" else 0,
                    "complete_inertial_links": 1 if status == "completed" else 0,
                },
            },
            "category": "fixture",
        }

    summary = runner.aggregate_records(
        [record("ok", "completed", joints=3, passed=2), record("bad", "error", joints=2, passed=0)],
        n_eval=2,
        j_eval=5,
    )
    assert summary["n_eval"] == 2
    assert summary["j_eval"] == 5
    assert summary["status_counts"] == {"completed": 1, "error": 1}
    assert summary["metrics"]["visual_bearing_collision_coverage"]["asset"]["denominator"] == 2
    assert summary["metrics"]["joint_limit_portability"]["denominator"] == 5
    assert summary["metrics"]["joint_dynamics_coverage"]["denominator"] == 5


def test_parts_zip_binding_records_hash_and_size(tmp_path: Path) -> None:
    archive = tmp_path / "parts.zip"
    archive.write_bytes(b"fixture archive")
    binding = runner.source_binding_for_parts_zip(archive)
    assert binding["path"] == str(archive.resolve())
    assert binding["sha256"] == hashlib.sha256(archive.read_bytes()).hexdigest()
    assert binding["bytes"] == archive.stat().st_size


def test_checker_self_hash_selects_schema_specific_field() -> None:
    checker = __import__("check_table2sup_full_release")
    # Published summaries/checkpoints carry the run-manifest hash alongside
    # their own content hash.  The checker must validate the latter.
    summary = {
        "schema_version": "table2sup_full_release_summary_v1",
        "manifest_content_sha256": "bound-run",
        "metrics": {"ok": 1},
    }
    summary["summary_content_sha256"] = checker.common.canonical_sha256(summary)
    assert checker._check_self_hash(
        summary,
        "fixture summary",
        field="summary_content_sha256",
        required=True,
    ) == summary["summary_content_sha256"]

    checkpoint = {
        "schema_version": "table2sup_checkpoint_v1",
        "manifest_content_sha256": "bound-run",
        "state": "complete",
    }
    checkpoint["checkpoint_content_sha256"] = checker.common.canonical_sha256(checkpoint)
    assert checker._check_self_hash(
        checkpoint,
        "fixture checkpoint",
        field="checkpoint_content_sha256",
        required=True,
    ) == checkpoint["checkpoint_content_sha256"]


def test_checker_allows_extraction_count_above_frozen_joint_denominator() -> None:
    checker = __import__("check_table2sup_full_release")
    record = {
        "asset_id": "physx-extra-joint",
        "status": "completed",
        "table2_supplementary": {
            "visual_bearing_collision_coverage": {
                "asset_pass": False,
                "visual_bearing_links_declared": 0,
                "covered_visual_bearing_links": 0,
                "link_extraction_complete": False,
            },
            "joint_limit_portability": {
                "joints_intended": 2,
                "joints_extracted": 3,
                "joints_passed": 2,
            },
            "joint_dynamics_coverage": {
                "joints_intended": 2,
                "joints_extracted": 3,
                "joints_covered": 0,
            },
            "placeholder_mass_incidence": {
                "dynamic_links": 0,
                "complete_inertial_links": 0,
            },
        },
    }
    aggregate = checker._aggregate([record], n_eval=1, j_eval=2)
    assert aggregate["metrics"]["joint_limit_portability"]["joints_extracted"] == 3

    record["table2_supplementary"]["joint_limit_portability"]["joints_passed"] = 4
    with pytest.raises(checker.AutomationError, match="intended/extracted"):
        checker._aggregate([record], n_eval=1, j_eval=2)


def test_checker_binds_combined_receipt_to_live_summary_and_evidence(tmp_path: Path) -> None:
    checker = __import__("check_table2sup_full_release")
    item = checker.DATASETS[0]
    root = tmp_path / "run-root"
    summary_path = root / item["slug"] / "summary.json"
    summary_path.parent.mkdir(parents=True)
    metrics = {
        "visual_bearing_collision_coverage": {"asset": {"numerator": 1, "denominator": item["n_eval"]}},
        "joint_limit_portability": {"numerator": 1, "denominator": item["j_eval"]},
        "joint_dynamics_coverage": {"numerator": 0, "denominator": item["j_eval"]},
        "placeholder_mass_incidence": {"status": "N/E"},
    }
    summary_path.write_text("{}\n", encoding="utf-8")
    checkpoint = {"state": "complete"}
    entry = {
        "N_eval": item["n_eval"],
        "J_eval": item["j_eval"],
        "dataset": item["display"],
        "display": item["display"],
        "status": "complete",
        "metrics": metrics,
        "evidence": {"summary": f"{item['slug']}/summary.json"},
    }
    checker._verify_receipt_entry(
        root,
        item,
        entry,
        dataset=item["display"],
        n_eval=item["n_eval"],
        j_eval=item["j_eval"],
        summary={"metrics": metrics},
        summary_path=summary_path,
        checkpoint=checkpoint,
    )

    entry["metrics"] = {**metrics, "extra": {"stale": True}}
    with pytest.raises(checker.AutomationError, match="metrics mismatch"):
        checker._verify_receipt_entry(
            root,
            item,
            entry,
            dataset=item["display"],
            n_eval=item["n_eval"],
            j_eval=item["j_eval"],
            summary={"metrics": metrics},
            summary_path=summary_path,
            checkpoint=checkpoint,
        )

    entry["metrics"] = metrics
    (summary_path.parent / "other-summary.json").write_text("{}\n", encoding="utf-8")
    entry["evidence"] = {"summary": "articraft/other-summary.json"}
    with pytest.raises(checker.AutomationError, match="evidence path mismatch"):
        checker._verify_receipt_entry(
            root,
            item,
            entry,
            dataset=item["display"],
            n_eval=item["n_eval"],
            j_eval=item["j_eval"],
            summary={"metrics": metrics},
            summary_path=summary_path,
            checkpoint=checkpoint,
        )


def test_checker_resolves_relative_root_once(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    checker = __import__("check_table2sup_full_release")
    root = tmp_path / "run-root"
    (root / "articraft").mkdir(parents=True)
    monkeypatch.chdir(tmp_path)
    with pytest.raises(checker.AutomationError, match="missing run manifest"):
        checker._live_dataset(Path("run-root"), checker.DATASETS[0], "none")


def test_checker_rejects_stale_markdown_numerator(tmp_path: Path) -> None:
    checker = __import__("check_table2sup_full_release")
    lines = [
        "### Table 2 supplementary. Collision, Joint, and Inertial Diagnostics",
        "| Dataset / Outputs | Visual-bearing Collision Coverage | Joint-limit Portability | Joint Dynamics Coverage | Placeholder-mass Incidence |",
        "|---|---:|---:|---:|---:|",
        "| Ours-500K | unchanged | unchanged | unchanged | N/E |",
        "| Ours per-class N=5 (supplementary) | unchanged | unchanged | unchanged | N/E |",
    ]
    results = {}
    for item in checker.DATASETS:
        n, j = item["n_eval"], item["j_eval"]
        display = item["display"]
        # The first visual numerator is intentionally stale (0 instead of 1).
        visual_numerator = 0 if item["slug"] == "articraft" else 1
        visual_pct = 100.0 * visual_numerator / n
        joint_pct = 100.0 / j
        lines.append(
            f"| {display} | {visual_numerator} / {n:,} ({visual_pct:.2f}%) | "
            f"1 / {j:,} ({joint_pct:.2f}%) | 1 / {j:,} ({joint_pct:.2f}%) | N/E |"
        )
        results[item["slug"]] = {
            "aggregate": {
                "metrics": {
                    "visual_bearing_collision_coverage": {"asset": {"numerator": 1, "denominator": n}},
                    "joint_limit_portability": {"numerator": 1, "denominator": j},
                    "joint_dynamics_coverage": {"numerator": 1, "denominator": j},
                    "placeholder_mass_incidence": {"status": "N/E"},
                }
            }
        }
    lines.append("\n#### Table 2 supplementary metric definitions\n")
    path = tmp_path / "evaluation.md"
    path.write_text("\n".join(lines), encoding="utf-8")
    with pytest.raises(checker.AutomationError, match="numerator"):
        checker.validate_supplementary_markdown(path, results)


def test_checker_rejects_stale_markdown_percentage(tmp_path: Path) -> None:
    checker = __import__("check_table2sup_full_release")
    lines = [
        "### Table 2 supplementary. Collision, Joint, and Inertial Diagnostics",
        "| Dataset / Outputs | Visual-bearing Collision Coverage | Joint-limit Portability | Joint Dynamics Coverage | Placeholder-mass Incidence |",
        "|---|---:|---:|---:|---:|",
        "| Ours-500K | unchanged | unchanged | unchanged | N/E |",
        "| Ours per-class N=5 (supplementary) | unchanged | unchanged | unchanged | N/E |",
    ]
    results = {}
    for item in checker.DATASETS:
        n, j = item["n_eval"], item["j_eval"]
        display = item["display"]
        visual_pct = 99.99 if item["slug"] == "articraft" else round(100.0 / n, 2)
        joint_pct = round(100.0 / j, 2)
        lines.append(
            f"| {display} | 1 / {n:,} ({visual_pct:.2f}%) | "
            f"1 / {j:,} ({joint_pct:.2f}%) | 1 / {j:,} ({joint_pct:.2f}%) | N/E |"
        )
        results[item["slug"]] = {
            "aggregate": {
                "metrics": {
                    "visual_bearing_collision_coverage": {"asset": {"numerator": 1, "denominator": n}},
                    "joint_limit_portability": {"numerator": 1, "denominator": j},
                    "joint_dynamics_coverage": {"numerator": 1, "denominator": j},
                    "placeholder_mass_incidence": {"status": "N/E"},
                }
            }
        }
    lines.append("\n#### Table 2 supplementary metric definitions\n")
    path = tmp_path / "evaluation.md"
    path.write_text("\n".join(lines), encoding="utf-8")
    with pytest.raises(checker.AutomationError, match="percentage"):
        checker.validate_supplementary_markdown(path, results)


def test_checker_binds_child_metrics_and_runtime_fields_to_parent(tmp_path: Path) -> None:
    checker = __import__("check_table2sup_full_release")
    children = tmp_path / "children"
    children.mkdir()
    expected_hash = "a" * 64
    job = {
        "selection_index": 0,
        "asset_id": "asset-0",
        "expected_primary_urdf_sha256": expected_hash,
        "expected_movable_joints": 2,
    }
    parent = {
        "asset_id": "asset-0",
        "selection_index": 0,
        "expected_primary_urdf_sha256": expected_hash,
        "expected_movable_joints": 2,
        "status": "completed",
        "package": "/tmp/package",
        "urdf_relative_path": "model.urdf",
        "table2_supplementary": {"joint_limit_portability": {"joints_passed": 2}},
    }
    child = {
        **parent,
        "status": "error",
        "package": "/tmp/other-package",
        "urdf_relative_path": "other.urdf",
        "table2_supplementary": {"joint_limit_portability": {"joints_passed": 0}},
    }
    (children / "000000.json").write_text(json.dumps(child), encoding="utf-8")
    with pytest.raises(checker.AutomationError, match="child.*binding"):
        checker._verify_children(tmp_path, [parent], [job])


def test_checker_requires_new_full_release_rows(tmp_path: Path) -> None:
    checker = __import__("check_table2sup_full_release")
    markdown = tmp_path / "evaluation.md"
    markdown.write_text(
        "## Table 2 supplementary. Collision, Joint, and Inertial Diagnostics\n"
        "| Dataset / Outputs | Visual-bearing Collision Coverage | Joint-limit Portability | Joint Dynamics Coverage | Placeholder-mass Incidence |\n"
        "|---|---:|---:|---:|---:|\n"
        "| Infinite Mobility | 720 / 720 (100.00%) | 1 / 1 (100.00%) | 1 / 1 (100.00%) | N/E |\n",
        encoding="utf-8",
    )
    with pytest.raises(checker.AutomationError, match="Infinigen-Sim"):
        checker.validate_supplementary_markdown(markdown, {"infinite": {}})


def test_checker_validates_nested_visual_denominator_and_rejects_stale_value(tmp_path: Path) -> None:
    checker = __import__("check_table2sup_full_release")
    lines = [
        "### Table 2 supplementary. Collision, Joint, and Inertial Diagnostics",
        "| Dataset / Outputs | Visual-bearing Collision Coverage | Joint-limit Portability | Joint Dynamics Coverage | Placeholder-mass Incidence |",
        "|---|---:|---:|---:|---:|",
        "| Ours-500K | unchanged | unchanged | unchanged | N/E |",
        "| Ours per-class N=5 (supplementary) | unchanged | unchanged | unchanged | N/E |",
    ]
    results = {}
    for item in checker.DATASETS:
        n, j = item["n_eval"], item["j_eval"]
        display = item["display"]
        visual_pct = 100.0 / n
        joint_pct = 100.0 / j
        lines.append(
            f"| {display} | 1 / {n:,} ({visual_pct:.2f}%) | "
            f"1 / {j:,} ({joint_pct:.2f}%) | 1 / {j:,} ({joint_pct:.2f}%) | N/E |"
        )
        results[item["slug"]] = {
            "aggregate": {
                "metrics": {
                    "visual_bearing_collision_coverage": {"asset": {"numerator": 1, "denominator": n}},
                    "joint_limit_portability": {"numerator": 1, "denominator": j},
                    "joint_dynamics_coverage": {"numerator": 1, "denominator": j},
                    "placeholder_mass_incidence": {"status": "N/E"},
                }
            }
        }
    lines.append("\n#### Table 2 supplementary metric definitions\n")
    path = tmp_path / "evaluation.md"
    path.write_text("\n".join(lines), encoding="utf-8")
    assert checker.validate_supplementary_markdown(path, results)["comparison_rows"] == 8
    path.write_text(path.read_text().replace("1 / 9,996", "1 / 800", 1), encoding="utf-8")
    with pytest.raises(checker.AutomationError):
        checker.validate_supplementary_markdown(path, results)
