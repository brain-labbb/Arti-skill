from __future__ import annotations

import copy
import importlib.util
from pathlib import Path
import sys

import pytest


REPO = Path(__file__).resolve().parents[2]
RUNNER = REPO / "exp/scripts/run_urdf_table4b_partnet_mobility.py"


def _load_runner():
    spec = importlib.util.spec_from_file_location(
        "run_urdf_table4b_partnet_mobility", RUNNER
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_frozen_manifest_builds_exact_ordered_jobs() -> None:
    runner = _load_runner()

    manifest = runner.load_source_manifest()
    jobs = runner.build_jobs(manifest)

    assert len(jobs) == 800
    assert [job["selection_index"] for job in jobs] == list(range(800))
    assert jobs[0]["asset_id"] == "16832"
    assert jobs[0]["expected_urdf_sha256"] == (
        "4c6c3993cf3c1d9630317fecb2300da7b60c4724aa90f8fe172d57830b492cb6"
    )
    assert jobs[0]["urdf_path"].endswith(
        "/PartNet-Mobility/data/dataset/16832/mobility.urdf"
    )
    assert jobs[-1]["asset_id"] == "100782"
    assert jobs[-1]["expected_urdf_sha256"] == (
        "60d38f7d3a5339fc6ede36f59ef63b8fdaa0ccb38b8fa9e6ae5cf8897435279d"
    )


def test_build_jobs_rejects_dataset_id_that_escapes_root() -> None:
    runner = _load_runner()
    manifest = runner.load_source_manifest()
    tampered = copy.deepcopy(manifest)
    tampered["items"][0]["dataset_id"] = "../dataset"

    with pytest.raises(SystemExit, match="dataset_id is not a package basename"):
        runner.build_jobs(tampered)


def test_build_jobs_rejects_urdf_symlink_that_escapes_package(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    runner = _load_runner()
    dataset_root = tmp_path / "dataset"
    package = dataset_root / "123"
    package.mkdir(parents=True)
    external_urdf = tmp_path / "external.urdf"
    external_urdf.write_text(
        '<robot name="external"><link name="base"/></robot>\n', encoding="utf-8"
    )
    (package / "mobility.urdf").symlink_to(external_urdf)
    monkeypatch.setattr(runner, "DATASET_ROOT", dataset_root)
    manifest = {
        "dataset_root": str(dataset_root),
        "items": [
            {
                "dataset_id": "123",
                "category": "Fixture",
                "urdf_sha256": runner.sha256_bytes(external_urdf.read_bytes()),
                "input_identity_sha256": "fixture-input",
            }
        ],
    }

    with pytest.raises(SystemExit, match="URDF escapes frozen PartNet-Mobility package"):
        runner.build_jobs(manifest)


def test_verify_run_rejects_dataset_id_drift() -> None:
    runner = _load_runner()
    manifest = runner.load_source_manifest()

    def unavailable() -> dict[str, object]:
        return {"status": "N/E", "value": None, "reason": "fixture"}

    records = []
    for index, item in enumerate(manifest["items"]):
        records.append(
            {
                "selection_index": index,
                "dataset_id": str(item["dataset_id"]),
                "asset_id": str(item["dataset_id"]),
                "status": "completed",
                "urdf_sha256": str(item["urdf_sha256"]),
                "analytic_collision_element_count": 0,
                "loadable_collision_element_count": 0,
                "visual_to_collision_p95_normalized": unavailable(),
                "collision_to_visual_p95_normalized": unavailable(),
                "shapes_per_visual_bearing_link": unavailable(),
                "collision_mesh_triangles_per_asset": unavailable(),
                "collision_load_time_seconds": unavailable(),
                "intra_link_redundancy": {
                    "status": "N/E",
                    "value": None,
                    "shape_volume_m3": None,
                    "redundant_volume_m3": None,
                    "intended_link_count": 0,
                    "measured_link_count": 0,
                    "reason": "fixture",
                },
            }
        )
    aggregates = runner.base.aggregate(records)
    records[0]["dataset_id"] = "drifted"

    verification = runner.verify_run(manifest, records, aggregates)
    frozen_order = next(
        check for check in verification["checks"] if check["check"] == "frozen_order_preserved"
    )

    assert frozen_order["pass"] is False
    assert verification["all_pass"] is False


@pytest.mark.parametrize(
    ("mode", "n", "expected"),
    [
        (
            "formal",
            800,
            "table4b_urdf_partnet_mobility_table4cohort_n800_salt20260813_20260821T120000Z",
        ),
        ("smoke", 3, "table4b_urdf_partnet_mobility_smoke_n3_20260821T120000Z"),
    ],
)
def test_output_directory_name_identifies_partnet(mode: str, n: int, expected: str) -> None:
    runner = _load_runner()

    assert runner.output_directory_name(mode, n, "20260821T120000Z") == expected
