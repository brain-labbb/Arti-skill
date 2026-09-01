from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
import sys
from xml.etree import ElementTree as ET

import numpy as np
import pytest
import trimesh


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/run_table1_diversity_metrics.py"
SPEC = importlib.util.spec_from_file_location("table1_diversity_tested", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
METRICS = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = METRICS
SPEC.loader.exec_module(METRICS)


def _robot(body: str) -> ET.Element:
    return ET.fromstring(f"<robot name='fixture'>{body}</robot>")


def _box_urdf(size: str, origin: str = "0 0 0", mesh: str | None = None) -> str:
    geometry = f"<mesh filename='{mesh}'/>" if mesh else f"<box size='{size}'/>"
    return (
        "<robot name='fixture'><link name='base'><visual>"
        f"<origin xyz='{origin}'/><geometry>{geometry}</geometry>"
        "</visual></link></robot>"
    )


def test_joint_variance_is_population_macro_and_retains_failures() -> None:
    records = [
        {"asset_id": "a0", "raw_category": "A", "parse_success": True, "non_fixed_joint_count": 0},
        {"asset_id": "a1", "raw_category": "A", "parse_success": True, "non_fixed_joint_count": 2},
        {"asset_id": "a2", "raw_category": "A", "parse_success": False, "non_fixed_joint_count": None},
        {"asset_id": "b0", "raw_category": "B", "parse_success": True, "non_fixed_joint_count": 7},
    ]
    summary, categories = METRICS.aggregate_joint_variance("fixture", records, expected_n=4)

    # A has population variance 1; singleton B has variance 0; macro is 0.5.
    assert summary["macro_population_variance_exact"] == "1/2"
    assert summary["macro_population_variance"] == 0.5
    assert summary["n_valid"] == 3
    assert summary["asset_coverage"] == 0.75
    assert [row["population_variance_exact"] for row in categories] == ["1/1", "0/1"]


def test_variance_rejects_bool_and_sketch_uses_source_category() -> None:
    records = [
        {"asset_id": "Agentic/Chair/a", "raw_category": "Chair", "parse_success": True, "non_fixed_joint_count": 1},
        {"asset_id": "Seed3D/Chair/b", "raw_category": "Chair", "parse_success": True, "non_fixed_joint_count": 3},
        {"asset_id": "Seed3D/Chair/c", "raw_category": "Chair", "parse_success": True, "non_fixed_joint_count": True},
    ]
    summary, categories = METRICS.aggregate_joint_variance("sketch", records)
    assert summary["category_count"] == 2
    assert summary["n_valid"] == 2
    assert [row["category"] for row in categories] == ["Agentic/Chair", "Seed3D/Chair"]


def test_fixed_contraction_name_and_sibling_order_invariance() -> None:
    direct = _robot(
        "<link name='root'/><link name='left'/><link name='right'/>"
        "<joint name='j1' type='revolute'><parent link='root'/><child link='left'/></joint>"
        "<joint name='j2' type='prismatic'><parent link='root'/><child link='right'/></joint>"
    )
    dummy = _robot(
        "<link name='renamed_root'/><link name='dummy'/><link name='b'/><link name='a'/>"
        "<joint name='fixed' type='fixed'><parent link='renamed_root'/><child link='dummy'/></joint>"
        "<joint name='second' type='prismatic'><parent link='dummy'/><child link='b'/></joint>"
        "<joint name='first' type='revolute'><parent link='dummy'/><child link='a'/></joint>"
    )
    direct_signature, direct_hash = METRICS.canonical_kinematic_graph(direct)
    dummy_signature, dummy_hash = METRICS.canonical_kinematic_graph(dummy)
    assert direct_signature == dummy_signature
    assert direct_hash == dummy_hash

    changed = ET.fromstring(ET.tostring(direct, encoding="unicode").replace("prismatic", "revolute"))
    assert METRICS.canonical_kinematic_graph(changed)[1] != direct_hash

    continuous = ET.fromstring(
        ET.tostring(direct, encoding="unicode").replace(
            'type="revolute"', 'type="continuous"', 1
        )
    )
    assert METRICS.canonical_kinematic_graph(continuous)[1] != direct_hash


def test_graph_rejects_mimic_until_dependency_canonicalization_is_frozen() -> None:
    root = _robot(
        "<link name='root'/><link name='child'/>"
        "<joint name='j' type='revolute'><parent link='root'/><child link='child'/><mimic joint='x'/></joint>"
    )
    with pytest.raises(METRICS.MetricError, match="mimic dependency"):
        METRICS.canonical_kinematic_graph(root)


def test_q0_unit_normalization_removes_translation_and_uniform_scale(tmp_path: Path) -> None:
    first = tmp_path / "first"
    second = tmp_path / "second"
    first.mkdir(); second.mkdir()
    (first / "model.urdf").write_text(_box_urdf("1 2 3", "4 5 6"), encoding="utf-8")
    (second / "model.urdf").write_text(_box_urdf("10 20 30", "40 50 60"), encoding="utf-8")
    asset_a = METRICS.Asset(
        0, "same-seed", "fixture", str(first / "model.urdf"), str(first),
        METRICS.sha256_file(first / "model.urdf"),
    )
    asset_b = METRICS.Asset(
        0, "same-seed", "fixture", str(second / "model.urdf"), str(second),
        METRICS.sha256_file(second / "model.urdf"),
    )
    record_a, points_a, descriptor_a = METRICS.extract_point_cloud(asset_a, 512)
    record_b, points_b, descriptor_b = METRICS.extract_point_cloud(asset_b, 512)

    assert record_a["status"] == record_b["status"] == "EVALUATED"
    np.testing.assert_allclose(points_a, points_b, atol=2e-7)
    np.testing.assert_allclose(descriptor_a, descriptor_b, atol=2e-7)
    assert np.linalg.norm(np.ptp(points_a, axis=0)) <= 1.01


def test_scene_instances_are_preserved_for_glb(tmp_path: Path) -> None:
    package = tmp_path / "package"
    package.mkdir()
    scene = trimesh.Scene()
    scene.add_geometry(trimesh.creation.box(extents=(1, 1, 1)), node_name="left")
    transform = np.eye(4)
    transform[0, 3] = 3.0
    scene.add_geometry(
        trimesh.creation.box(extents=(1, 1, 1)), node_name="right", transform=transform
    )
    (package / "pair.glb").write_bytes(scene.export(file_type="glb"))
    instances = METRICS._mesh_instances(package / "pair.glb", np.asarray((2.0, 1.0, 1.0)))
    all_vertices = np.vstack([np.asarray(mesh.vertices) for mesh in instances])
    assert float(np.max(all_vertices[:, 0])) == pytest.approx(7.0)
    (package / "model.urdf").write_text(_box_urdf("", mesh="pair.glb"), encoding="utf-8")
    asset = METRICS.Asset(
        0, "scene", "fixture", str(package / "model.urdf"), str(package),
        METRICS.sha256_file(package / "model.urdf"), None,
        (("pair.glb", METRICS.sha256_file(package / "pair.glb")),),
    )

    record, points, _ = METRICS.extract_point_cloud(asset, 512)
    assert record["loaded_mesh_instance_count"] == 2
    assert float(np.ptp(points[:, 0])) > 0.8


def test_mesh_resolution_supports_physx_sibling_and_rejects_escape(tmp_path: Path) -> None:
    package = tmp_path / "dataset"
    urdf_dir = package / "urdf"
    mesh_dir = package / "partseg/1"
    urdf_dir.mkdir(parents=True); mesh_dir.mkdir(parents=True)
    mesh = mesh_dir / "part.obj"
    mesh.write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n", encoding="utf-8")
    urdf = urdf_dir / "1.urdf"
    urdf.write_text("<robot name='x'/>", encoding="utf-8")
    assert METRICS._contained_file("../partseg/1/part.obj", urdf, package) == mesh.resolve()

    outside = tmp_path / "outside.obj"
    outside.write_text("v 0 0 0\n", encoding="utf-8")
    with pytest.raises(METRICS.MetricError, match="escapes package"):
        METRICS._contained_file("../../outside.obj", urdf, package)


def test_surface_sampler_is_deterministic_and_area_weighted() -> None:
    vertices = np.asarray(
        ((0, 0, 0), (1, 0, 0), (0, 1, 0), (10, 0, 0), (12, 0, 0), (10, 2, 0)),
        dtype=float,
    )
    faces = np.asarray(((0, 1, 2), (3, 4, 5)), dtype=int)  # areas 0.5 and 2.0
    left = METRICS.sample_surface(vertices, faces, 2000, np.random.Generator(np.random.PCG64DXSM(7)))
    right = METRICS.sample_surface(vertices, faces, 2000, np.random.Generator(np.random.PCG64DXSM(7)))
    np.testing.assert_array_equal(left, right)
    assert 0.75 < float(np.mean(left[:, 0] > 5)) < 0.85


def test_symmetric_chamfer_matches_brute_force_and_is_permutation_invariant() -> None:
    left = np.asarray(((0, 0, 0), (1, 0, 0), (0, 2, 0)), dtype=float)
    right = np.asarray(((0, 0, 0), (2, 0, 0)), dtype=float)
    distances = np.linalg.norm(left[:, None, :] - right[None, :, :], axis=2)
    expected = 0.5 * (np.min(distances, axis=1).mean() + np.min(distances, axis=0).mean())
    assert METRICS.symmetric_chamfer(left, right) == pytest.approx(expected)
    assert METRICS.symmetric_chamfer(right[::-1], left[::-1]) == pytest.approx(expected)
    assert METRICS.symmetric_chamfer(left, left) == 0.0


def test_worker_retries_only_transient_failures(monkeypatch: pytest.MonkeyPatch) -> None:
    asset = METRICS.Asset(0, "asset", "category", "/tmp/model.urdf", "/tmp", None)
    attempts = 0

    def transient_then_success(_asset: METRICS.Asset, _count: int):
        nonlocal attempts
        attempts += 1
        if attempts < 3:
            raise OSError("temporary read failure")
        return ({"ordinal": 0}, np.zeros((64, 3)), np.zeros(94))

    monkeypatch.setattr(METRICS, "extract_point_cloud", transient_then_success)
    record, _, _ = METRICS._extract_worker((asset, 64))
    assert record["attempt_count"] == 3
    assert record["error_kind"] is None

    def semantic_failure(_asset: METRICS.Asset, _count: int):
        raise METRICS.MetricError("invalid graph")

    monkeypatch.setattr(METRICS, "extract_point_cloud", semantic_failure)
    record, _, _ = METRICS._extract_worker((asset, 64))
    assert record["attempt_count"] == 1
    assert record["error_kind"] == "SEMANTIC_OR_UNSUPPORTED"


def test_candidate_generation_and_duplicate_rate_contracts() -> None:
    descriptors = np.asarray(((0, 0), (0.1, 0), (5, 0), (5.1, 0)), dtype=float)
    exhaustive = METRICS.candidate_pairs_for_group(descriptors, top_k=1, exhaustive_limit=4)
    assert len(exhaustive) == 6
    retrieved = METRICS.candidate_pairs_for_group(descriptors, top_k=1, exhaustive_limit=2)
    assert {(left, right) for left, right, _ in retrieved} == {(0, 1), (2, 3)}

    rates = METRICS.duplicate_rates(5, ((0, 1), (1, 2), (3, 4)))
    assert rates["neighbor_asset_rate"] == 1.0
    assert rates["component_count"] == 2
    assert rates["duplicate_excess_count"] == 3
    assert rates["cluster_excess_rate"] == 0.6

    macro, rows = METRICS.category_macro_duplicate_rates(
        ("A", "A", "A", "B", "B"),
        ((0, 1), (1, 2), (3, 4)),
        seed_key="fixture",
    )
    assert [row["cluster_excess_rate"] for row in rows] == [pytest.approx(2 / 3), 0.5]
    assert macro["category_macro_cluster_excess_rate"] == pytest.approx(7 / 12)
    assert len(macro["category_macro_cluster_excess_rate_ci95"]) == 2


def test_calibration_receipt_is_required_and_heldout_gated(tmp_path: Path) -> None:
    output = tmp_path / "output"
    calibration = output / "near_duplicate/calibration"
    calibration.mkdir(parents=True)
    audits = []
    labels = []
    for index in range(400):
        pair_id = f"pair_{index:04d}"
        duplicate = index < 250
        audits.append({
            "pair_id": pair_id,
            "dataset_key": "fixture",
            "left_asset_id": f"left_{index:04d}",
            "right_asset_id": f"right_{index:04d}",
            "chamfer_distance": 0.01 + index * 1e-6 if duplicate else 0.2 + index * 1e-6,
        })
        labels.append({"pair_id": pair_id, "label": "duplicate" if duplicate else "not_duplicate"})
    METRICS.write_jsonl(
        calibration / "annotation_tasks.jsonl",
        ({"pair_id": row["pair_id"], "label": None} for row in audits),
    )
    METRICS.write_jsonl(calibration / "annotation_audit.jsonl", audits)
    labels_path = tmp_path / "labels.jsonl"
    METRICS.write_jsonl(labels_path, labels)
    METRICS.write_json(
        output / "manifest.json",
        {"schema_version": "fixture"},
        self_hash="manifest_content_sha256",
    )
    METRICS.write_json(
        calibration / "annotation_packet.json",
        {
            "schema_version": "fixture",
            "run_manifest_sha256": METRICS.sha256_file(output / "manifest.json"),
            "annotation_audit_sha256": METRICS.sha256_file(
                calibration / "annotation_audit.jsonl"
            ),
            "annotation_tasks_sha256": METRICS.sha256_file(
                calibration / "annotation_tasks.jsonl"
            ),
        },
        self_hash="packet_content_sha256",
    )

    with pytest.raises(METRICS.MetricError, match="threshold receipt"):
        METRICS.score_all((), output)
    receipt = METRICS.calibrate_threshold(
        output, labels_path, target_precision=0.98, min_resolved=300
    )
    assert receipt["status"] == "PASS"
    assert 0.01 < receipt["tau"] < 0.2
    assert receipt["heldout_metrics"]["precision"] == 1.0
    assert Path(receipt["labels_path"]) == calibration / "completed_labels.jsonl"
    assert METRICS.sha256_file(Path(receipt["labels_path"])) == receipt["labels_sha256"]


def test_calibration_rejects_selectively_omitted_labels(tmp_path: Path) -> None:
    output = tmp_path / "output"
    calibration = output / "near_duplicate/calibration"
    calibration.mkdir(parents=True)
    tasks = [{"pair_id": "a", "label": None}, {"pair_id": "b", "label": None}]
    audits = [
        {"pair_id": "a", "dataset_key": "fixture", "left_asset_id": "a0", "right_asset_id": "a1", "chamfer_distance": 0.01},
        {"pair_id": "b", "dataset_key": "fixture", "left_asset_id": "b0", "right_asset_id": "b1", "chamfer_distance": 0.2},
    ]
    METRICS.write_jsonl(calibration / "annotation_tasks.jsonl", tasks)
    METRICS.write_jsonl(calibration / "annotation_audit.jsonl", audits)
    METRICS.write_json(output / "manifest.json", {"schema_version": "fixture"}, self_hash="manifest_content_sha256")
    METRICS.write_json(
        calibration / "annotation_packet.json",
        {
            "run_manifest_sha256": METRICS.sha256_file(output / "manifest.json"),
            "annotation_tasks_sha256": METRICS.sha256_file(calibration / "annotation_tasks.jsonl"),
            "annotation_audit_sha256": METRICS.sha256_file(calibration / "annotation_audit.jsonl"),
        },
        self_hash="packet_content_sha256",
    )
    labels = tmp_path / "labels.jsonl"
    METRICS.write_jsonl(labels, [{"pair_id": "a", "label": "duplicate"}])
    with pytest.raises(METRICS.MetricError, match="every annotation task"):
        METRICS.calibrate_threshold(output, labels, target_precision=0.98, min_resolved=1)


def test_resume_rejects_configuration_drift(tmp_path: Path) -> None:
    output, scratch = tmp_path / "out", tmp_path / "scratch"

    def args(workers: int, resume: bool) -> argparse.Namespace:
        return argparse.Namespace(
            output=output,
            scratch=scratch,
            workers=workers,
            point_count=2048,
            top_k=64,
            exhaustive_group_limit=256,
            annotation_pairs=1000,
            target_precision=0.98,
            min_resolved_labels=800,
            pva_root_override=None,
            resume=resume,
        )

    METRICS.initialize_run(args(2, False), ())
    METRICS.initialize_run(args(2, True), ())
    with pytest.raises(METRICS.MetricError, match="workers"):
        METRICS.initialize_run(args(3, True), ())


def test_resume_rejects_bound_input_drift(tmp_path: Path) -> None:
    output, scratch = tmp_path / "out", tmp_path / "scratch"
    records, roster = tmp_path / "records.jsonl", tmp_path / "roster.jsonl"
    records.write_text("{}\n", encoding="utf-8")
    roster.write_text("{}\n", encoding="utf-8")
    spec = METRICS.DatasetSpec("fixture", "Fixture", records, roster, "jsonl", 1)
    args = argparse.Namespace(
        output=output,
        scratch=scratch,
        workers=1,
        point_count=64,
        top_k=8,
        exhaustive_group_limit=32,
        annotation_pairs=20,
        target_precision=0.98,
        min_resolved_labels=10,
        pva_root_override=None,
        resume=False,
    )
    METRICS.initialize_run(args, (spec,))
    records.write_text('{"changed":true}\n', encoding="utf-8")
    args.resume = True
    with pytest.raises(METRICS.MetricError, match="dataset_inputs"):
        METRICS.initialize_run(args, (spec,))
