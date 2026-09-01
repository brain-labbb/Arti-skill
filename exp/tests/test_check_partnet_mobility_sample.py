from __future__ import annotations

import hashlib
import importlib.util
import json
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
CHECKER = REPO / "exp/scripts/check_partnet_mobility_sample.py"


def _load_checker():
    spec = importlib.util.spec_from_file_location(
        "check_partnet_mobility_sample", CHECKER
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _canonical_sha256(value: object) -> str:
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _fixture_input_identity_fields() -> tuple[str, ...]:
    return (
        "protocol_id",
        "order",
        "dataset_id",
        "selection_digest",
        "category",
        "movable_dof_count",
        "range_evaluable_dof_count",
        "joint_specs_sha256",
        "runtime_identity_sha256",
        "urdf_sha256",
        "bounding_box_sha256",
        "collision_mesh_inventory_sha256",
        "object_bbox_diagonal_m",
        "rest_state_expected",
        "single_state_expected",
        "sobol_state_expected",
    )


def _fixture_manifest(tmp_path: Path, ids: tuple[str, ...] = ("20", "10")) -> Path:
    dataset_root = tmp_path / "dataset"
    items: list[dict[str, object]] = []
    for order, dataset_id in enumerate(ids):
        package = dataset_root / dataset_id
        package.mkdir(parents=True)
        mesh = package / "mesh.obj"
        mesh.write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n", encoding="utf-8")
        urdf = package / "mobility.urdf"
        urdf.write_text(
            f'<robot name="fixture-{dataset_id}"><link name="base"><collision>'
            '<geometry><mesh filename="mesh.obj"/></geometry></collision></link></robot>\n',
            encoding="utf-8",
        )
        for filename in (
            "meta.json",
            "mobility_v2.json",
            "semantics.txt",
            "result.json",
            "bounding_box.json",
        ):
            (package / filename).write_text("{}\n", encoding="utf-8")
        inventory = [
            {
                "path": "mesh.obj",
                "exists": True,
                "size_bytes": mesh.stat().st_size,
                "sha256": hashlib.sha256(mesh.read_bytes()).hexdigest(),
            }
        ]
        item = {
            "protocol_id": "fixture_protocol_v1",
            "order": order,
            "dataset_id": dataset_id,
            "selection_digest": f"fixture-{dataset_id}",
            "category": "fixture",
            "movable_dof_count": 0,
            "range_evaluable_dof_count": 0,
            "joint_specs_sha256": _canonical_sha256([]),
            "runtime_identity_sha256": _canonical_sha256({}),
            "object_bbox_diagonal_m": 1.0,
            "rest_state_expected": 1,
            "single_state_expected": 0,
            "sobol_state_expected": 0,
            "urdf_sha256": hashlib.sha256(urdf.read_bytes()).hexdigest(),
            "bounding_box_sha256": hashlib.sha256(
                (package / "bounding_box.json").read_bytes()
            ).hexdigest(),
            "collision_mesh_files": inventory,
            "collision_mesh_inventory_sha256": _canonical_sha256(inventory),
        }
        item["input_identity_sha256"] = _canonical_sha256(
            {key: item[key] for key in _fixture_input_identity_fields()}
        )
        items.append(item)

    manifest = {
        "status": "FROZEN",
        "protocol_id": "fixture_protocol_v1",
        "dataset_root": str(dataset_root),
        "sample_size": len(items),
        "items": items,
        "ordered_selected_ids_sha256": _canonical_sha256(list(ids)),
        "items_sha256": _canonical_sha256(items),
    }
    path = tmp_path / "frozen_manifest.json"
    path.write_text(
        json.dumps(manifest, sort_keys=True, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )
    return path


def _write_runner_receipt(
    checker,
    output: Path,
    record: dict[str, object],
    *,
    summary_status_counts: dict[str, int] | None = None,
    manifest_status_counts: dict[str, int] | None = None,
) -> None:
    output.mkdir()
    summary = {
        "protocol_id": checker.EXPECTED_RUNNER_PROTOCOL_ID,
        "mode": "smoke",
        "dataset": "PartNet-Mobility",
        "cohort": {
            "n_eval": 1,
            "source_manifest_sha256": checker.EXPECTED_MANIFEST_SHA256,
            "ordered_ids_sha256": checker.EXPECTED_ORDERED_IDS_SHA256,
        },
        "status_counts": summary_status_counts
        or {"completed": 1, "error": 0, "total": 1},
    }
    summary_path = output / "summary.json"
    records_path = output / "asset_records.jsonl"
    summary_path.write_text(json.dumps(summary), encoding="utf-8")
    records_path.write_text(json.dumps(record) + "\n", encoding="utf-8")
    run_manifest = {
        "protocol_id": checker.EXPECTED_RUNNER_PROTOCOL_ID,
        "mode": "smoke",
        "dataset": "PartNet-Mobility",
        "record_count": 1,
        "status_counts": manifest_status_counts
        or {"completed": 1, "error": 0, "total": 1},
        "outputs": {
            "summary_sha256": hashlib.sha256(summary_path.read_bytes()).hexdigest(),
            "asset_records_sha256": hashlib.sha256(records_path.read_bytes()).hexdigest(),
        },
    }
    (output / "manifest.json").write_text(json.dumps(run_manifest), encoding="utf-8")


def test_frozen_manifest_paths_preserve_order_and_hash_contract() -> None:
    checker = _load_checker()

    report = checker.check_sample()
    paths = checker.load_sample_paths()

    assert report["status"] == "PASS"
    assert report["sample_size"] == 800
    assert report["paths_checked"] == 800
    assert report["first_path"].endswith(
        "/PartNet-Mobility/data/dataset/16832/mobility.urdf"
    )
    assert report["last_path"].endswith(
        "/PartNet-Mobility/data/dataset/100782/mobility.urdf"
    )
    assert len(paths) == 800
    assert paths[0].name == "mobility.urdf"
    assert paths[-1].parent.name == "100782"


def test_fixture_paths_follow_manifest_items_without_sorting(tmp_path: Path) -> None:
    checker = _load_checker()
    manifest = _fixture_manifest(tmp_path)

    paths = checker.load_sample_paths(
        manifest,
        expected_count=2,
        expected_dataset_root=None,
    )

    assert paths == [
        (tmp_path / "dataset/20/mobility.urdf").resolve(),
        (tmp_path / "dataset/10/mobility.urdf").resolve(),
    ]


def test_fixture_package_paths_match_jq_semantics(tmp_path: Path) -> None:
    checker = _load_checker()
    manifest_path = _fixture_manifest(tmp_path)

    packages = checker.load_sample_package_paths(
        manifest_path,
        expected_count=2,
        expected_dataset_root=None,
    )

    assert packages == [
        (tmp_path / "dataset/20").resolve(),
        (tmp_path / "dataset/10").resolve(),
    ]


def test_cli_print_paths_is_jq_compatible_and_urdf_output_is_explicit(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    checker = _load_checker()
    manifest_path = _fixture_manifest(tmp_path)

    assert checker.main(
        [
            "--manifest",
            str(manifest_path),
            "--expected-root",
            str(tmp_path / "dataset"),
            "--expected-count",
            "2",
            "--print-paths",
        ]
    ) == 0
    package_lines = capsys.readouterr().out.splitlines()
    assert package_lines == [
        str((tmp_path / "dataset/20").resolve()),
        str((tmp_path / "dataset/10").resolve()),
    ]

    assert checker.main(
        [
            "--manifest",
            str(manifest_path),
            "--expected-root",
            str(tmp_path / "dataset"),
            "--expected-count",
            "2",
            "--print-urdf-paths",
        ]
    ) == 0
    urdf_lines = capsys.readouterr().out.splitlines()
    assert urdf_lines == [
        str((tmp_path / "dataset/20/mobility.urdf").resolve()),
        str((tmp_path / "dataset/10/mobility.urdf").resolve()),
    ]


def test_manifest_rejects_dataset_id_path_escape(tmp_path: Path) -> None:
    checker = _load_checker()
    manifest_path = _fixture_manifest(tmp_path)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["items"][0]["dataset_id"] = "../outside"
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

    with pytest.raises(checker.SampleValidationError, match="dataset_id"):
        checker.load_sample_paths(
            manifest_path,
            expected_count=2,
            expected_dataset_root=None,
        )


def test_manifest_rejects_urdf_content_drift(tmp_path: Path) -> None:
    checker = _load_checker()
    manifest_path = _fixture_manifest(tmp_path)
    (tmp_path / "dataset/20/mobility.urdf").write_text(
        '<robot name="changed"><link name="base"/></robot>\n',
        encoding="utf-8",
    )

    with pytest.raises(checker.SampleValidationError, match="URDF SHA-256"):
        checker.load_sample_paths(
            manifest_path,
            expected_count=2,
            expected_dataset_root=None,
        )


def test_manifest_rejects_missing_required_core_file(tmp_path: Path) -> None:
    checker = _load_checker()
    manifest_path = _fixture_manifest(tmp_path)
    (tmp_path / "dataset/10/bounding_box.json").unlink()

    with pytest.raises(checker.SampleValidationError, match="bounding_box.json"):
        checker.load_sample_paths(
            manifest_path,
            expected_count=2,
            expected_dataset_root=None,
        )


def test_manifest_rejects_bounding_box_content_drift(tmp_path: Path) -> None:
    checker = _load_checker()
    manifest_path = _fixture_manifest(tmp_path)
    (tmp_path / "dataset/20/bounding_box.json").write_text(
        '{"changed": true}\n', encoding="utf-8"
    )

    with pytest.raises(checker.SampleValidationError, match="bounding-box SHA-256"):
        checker.load_sample_paths(
            manifest_path,
            expected_count=2,
            expected_dataset_root=None,
        )


def test_manifest_rejects_collision_mesh_content_drift(tmp_path: Path) -> None:
    checker = _load_checker()
    manifest_path = _fixture_manifest(tmp_path)
    (tmp_path / "dataset/20/mesh.obj").write_text(
        "v 0 0 0\nv 2 0 0\nv 0 1 0\nf 1 2 3\n", encoding="utf-8"
    )

    with pytest.raises(checker.SampleValidationError, match="collision mesh inventory"):
        checker.load_sample_paths(
            manifest_path,
            expected_count=2,
            expected_dataset_root=None,
        )


def test_manifest_rejects_input_identity_metadata_drift(tmp_path: Path) -> None:
    checker = _load_checker()
    manifest_path = _fixture_manifest(tmp_path)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["items"][0]["category"] = "tampered"
    manifest["items_sha256"] = _canonical_sha256(manifest["items"])
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

    with pytest.raises(checker.SampleValidationError, match="input identity SHA-256"):
        checker.load_sample_paths(
            manifest_path,
            expected_count=2,
            expected_dataset_root=None,
        )


def test_formal_copy_with_metadata_tampering_is_rejected(tmp_path: Path) -> None:
    checker = _load_checker()
    copied = tmp_path / "copied-frozen-manifest.json"
    shutil.copyfile(checker.DEFAULT_MANIFEST, copied)
    manifest = json.loads(copied.read_text(encoding="utf-8"))
    manifest["protocol_id"] = "tampered"
    copied.write_text(json.dumps(manifest), encoding="utf-8")

    with pytest.raises(checker.SampleValidationError, match="frozen manifest SHA-256"):
        checker.load_manifest(
            copied,
            expected_count=checker.DEFAULT_SAMPLE_SIZE,
            expected_dataset_root=checker.DEFAULT_DATASET_ROOT,
        )


def test_manifest_rejects_non_integer_order(tmp_path: Path) -> None:
    checker = _load_checker()
    manifest_path = _fixture_manifest(tmp_path)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["items"][0]["order"] = 0.5
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

    with pytest.raises(checker.SampleValidationError, match="order"):
        checker.load_sample_paths(
            manifest_path,
            expected_count=2,
            expected_dataset_root=None,
        )


def test_direct_asset_resolution_rejects_order_drift() -> None:
    checker = _load_checker()
    manifest = checker.load_manifest()[0]
    altered = json.loads(json.dumps(manifest))
    altered["items"][0]["order"] = 1

    with pytest.raises(checker.SampleValidationError, match="order"):
        checker.resolve_sample_assets(altered)


def test_runner_command_binds_mode_sample_and_output() -> None:
    checker = _load_checker()

    command = checker.build_runner_command(
        runner_python=Path("/opt/python"),
        runner=Path("runner.py"),
        mode="smoke",
        sample_size=3,
        workers=1,
        output_dir=Path("/tmp/table4b-smoke"),
    )

    assert command == [
        "/opt/python",
        "runner.py",
        "--mode",
        "smoke",
        "--n",
        "3",
        "--workers",
        "1",
        "--output-dir",
        "/tmp/table4b-smoke",
    ]


def test_run_binding_rejects_untrusted_runner() -> None:
    checker = _load_checker()

    with pytest.raises(checker.SampleValidationError, match="canonical runner"):
        checker.validate_runner_binding(Path("/bin/true"))


def test_runner_result_requires_a_bound_output_directory(tmp_path: Path) -> None:
    checker = _load_checker()
    completed = subprocess.CompletedProcess(
        ["runner"],
        0,
        stdout="{}\n",
        stderr="",
    )

    with pytest.raises(checker.SampleValidationError, match="run directory"):
        checker.validate_runner_result(
            completed,
            mode="smoke",
            expected_count=1,
            expected_dataset_ids=["20"],
            output_dir=tmp_path / "missing-output",
        )


def test_runner_result_rejects_completed_record_without_urdf_binding(
    tmp_path: Path,
) -> None:
    checker = _load_checker()
    output = tmp_path / "output"
    output.mkdir()
    summary = {
        "protocol_id": checker.EXPECTED_RUNNER_PROTOCOL_ID,
        "mode": "smoke",
        "dataset": "PartNet-Mobility",
        "cohort": {
            "n_eval": 1,
            "source_manifest_sha256": checker.EXPECTED_MANIFEST_SHA256,
            "ordered_ids_sha256": checker.EXPECTED_ORDERED_IDS_SHA256,
        },
        "status_counts": {"completed": 1, "error": 0, "total": 1},
    }
    (output / "summary.json").write_text(
        json.dumps(summary), encoding="utf-8"
    )
    (output / "asset_records.jsonl").write_text(
        json.dumps(
            {
                "protocol_id": checker.EXPECTED_RUNNER_PROTOCOL_ID,
                "selection_index": 0,
                "dataset_id": "20",
                "asset_id": "20",
                "status": "completed",
            }
        )
        + "\n",
        encoding="utf-8",
    )
    record_hash = hashlib.sha256(
        (output / "asset_records.jsonl").read_bytes()
    ).hexdigest()
    summary_hash = hashlib.sha256((output / "summary.json").read_bytes()).hexdigest()
    (output / "manifest.json").write_text(
        json.dumps(
            {
                "protocol_id": checker.EXPECTED_RUNNER_PROTOCOL_ID,
                "mode": "smoke",
                "dataset": "PartNet-Mobility",
                "record_count": 1,
                "outputs": {
                    "summary_sha256": summary_hash,
                    "asset_records_sha256": record_hash,
                },
            }
        ),
        encoding="utf-8",
    )

    completed = subprocess.CompletedProcess(
        ["runner"],
        0,
        stdout=json.dumps({"run_directory": str(output)}),
        stderr="",
    )
    with pytest.raises(checker.SampleValidationError, match="URDF hash"):
        checker.validate_runner_result(
            completed,
            mode="smoke",
            expected_count=1,
            expected_dataset_ids=["20"],
            expected_urdf_sha256=["a" * 64],
            expected_package_paths=[(tmp_path / "package-20").resolve()],
            output_dir=output,
        )


def test_runner_result_requires_completed_record_package_binding(tmp_path: Path) -> None:
    checker = _load_checker()
    output = tmp_path / "output"
    package = tmp_path / "package-20"
    package.mkdir()
    expected_hash = "a" * 64
    _write_runner_receipt(
        checker,
        output,
        {
            "protocol_id": checker.EXPECTED_RUNNER_PROTOCOL_ID,
            "selection_index": 0,
            "dataset_id": "20",
            "asset_id": "20",
            "status": "completed",
            "expected_urdf_sha256": expected_hash,
            "urdf_sha256": expected_hash,
        },
    )

    completed = subprocess.CompletedProcess(
        ["runner"],
        0,
        stdout=json.dumps({"run_directory": str(output)}),
        stderr="",
    )
    with pytest.raises(checker.SampleValidationError, match="package binding"):
        checker.validate_runner_result(
            completed,
            mode="smoke",
            expected_count=1,
            expected_dataset_ids=["20"],
            expected_urdf_sha256=[expected_hash],
            expected_package_paths=[package],
            output_dir=output,
        )


def test_runner_result_cross_checks_status_counts(tmp_path: Path) -> None:
    checker = _load_checker()
    output = tmp_path / "output"
    package = tmp_path / "package-20"
    package.mkdir()
    expected_hash = "a" * 64
    record = {
        "protocol_id": checker.EXPECTED_RUNNER_PROTOCOL_ID,
        "selection_index": 0,
        "dataset_id": "20",
        "asset_id": "20",
        "status": "completed",
        "package": str(package),
        "expected_urdf_sha256": expected_hash,
        "urdf_sha256": expected_hash,
    }
    _write_runner_receipt(
        checker,
        output,
        record,
        summary_status_counts={"completed": 0, "error": 1, "total": 1},
    )
    completed = subprocess.CompletedProcess(
        ["runner"],
        0,
        stdout=json.dumps({"run_directory": str(output)}),
        stderr="",
    )
    with pytest.raises(checker.SampleValidationError, match="status counts"):
        checker.validate_runner_result(
            completed,
            mode="smoke",
            expected_count=1,
            expected_dataset_ids=["20"],
            expected_urdf_sha256=[expected_hash],
            expected_package_paths=[package],
            output_dir=output,
        )
