from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path, PurePosixPath

import pytest


REPO = Path(__file__).resolve().parents[2]
SOURCE_AUDITOR = REPO / "exp/scripts/audit_partnet_mobility_table4_source.py"


def load_source_auditor():
    spec = importlib.util.spec_from_file_location("partnet_table4_source_auditor", SOURCE_AUDITOR)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def test_archive_member_maps_under_data_extraction_root() -> None:
    module = load_source_auditor()
    member = PurePosixPath("dataset/100013/mobility.urdf")
    extracted = module.archive_member_to_extracted(member)
    assert extracted == module.EXTRACTION_ROOT / "dataset/100013/mobility.urdf"
    assert extracted.is_file()
    assert module.extracted_archive_key(extracted) == member.as_posix()


def test_private_geometry_closure_rejects_unreferenced_blob(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    module = load_source_auditor()
    monkeypatch.setattr(module, "WORKSPACE", tmp_path)
    private_root = tmp_path / "private"
    scenes = private_root / "geometry_scenes"
    blobs = private_root / "geometry_blobs"
    scenes.mkdir(parents=True)
    blobs.mkdir()

    vertices = module.np.asarray([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype="<f8")
    faces = module.np.asarray([[0, 1, 2]], dtype="<i8")
    blob_bytes = module.deterministic_npz_bytes(vertices, faces)
    blob_hash = sha256_bytes(blob_bytes)
    blob_path = blobs / f"{blob_hash}.npz"
    blob_path.write_bytes(blob_bytes)
    candidate_key = "a" * 64
    scene_path = scenes / f"{candidate_key}.json"
    write_json(
        scene_path,
        {
            "schema_version": 2,
            "camera_frame": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            "mesh_instances": [
                {
                    "mesh_blob": f"geometry_blobs/{blob_hash}.npz",
                    "mesh_sha256": blob_hash,
                    "transform": [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
                }
            ],
        },
    )
    plan = [
        {
            "candidate_key": candidate_key,
            "geometry_scene": f"geometry_scenes/{candidate_key}.json",
            "geometry_scene_sha256": module.sha256_file(scene_path),
        }
    ]

    closure = module.validate_private_geometry_closure(private_root, plan)
    assert closure["geometry_scene_count"] == 1
    assert closure["geometry_blob_count"] == 1

    (blobs / "unreferenced.npz").write_bytes(blob_bytes)
    with pytest.raises(RuntimeError, match="geometry blob exact closure"):
        module.validate_private_geometry_closure(private_root, plan)


def test_obj_labels_do_not_change_numeric_npz(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    module = load_source_auditor()
    monkeypatch.setattr(module, "WORKSPACE", tmp_path)
    first = tmp_path / "first.obj"
    second = tmp_path / "second.obj"
    numeric = "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1/7/3 2/8/3 3/9/3\n"
    first.write_text("# label one\nmtllib secret-one.mtl\no component-door\nusemtl brass\n" + numeric, encoding="utf-8")
    second.write_text("# unrelated comment\ng semantic-handle\nusemtl plastic\nmtllib other.mtl\n" + numeric, encoding="utf-8")

    vertices1, faces1 = module.parse_obj_numeric(first)
    vertices2, faces2 = module.parse_obj_numeric(second)
    assert module.np.array_equal(vertices1, vertices2)
    assert module.np.array_equal(faces1, faces2)
    assert module.deterministic_npz_bytes(vertices1, faces1) == module.deterministic_npz_bytes(vertices2, faces2)


def test_obj_homogeneous_vertex_is_divided_by_w(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    module = load_source_auditor()
    monkeypatch.setattr(module, "WORKSPACE", tmp_path)
    source = tmp_path / "homogeneous.obj"
    source.write_text(
        "v 2 4 6 2\n"
        "v 4 0 0 2\n"
        "v 0 8 0 2\n"
        "f 1 2 3\n",
        encoding="utf-8",
    )

    vertices, faces = module.parse_obj_numeric(source)

    assert module.np.array_equal(
        vertices,
        module.np.asarray([[1.0, 2.0, 3.0], [2.0, 0.0, 0.0], [0.0, 4.0, 0.0]], dtype="<f8"),
    )
    assert module.np.array_equal(faces, module.np.asarray([[0, 1, 2]], dtype="<i8"))


def test_obj_homogeneous_vertex_rejects_zero_w(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    module = load_source_auditor()
    monkeypatch.setattr(module, "WORKSPACE", tmp_path)
    source = tmp_path / "zero-w.obj"
    source.write_text("v 1 2 3 0\nv 0 0 0\nv 0 1 0\nf 1 2 3\n", encoding="utf-8")

    with pytest.raises(RuntimeError, match="zero OBJ homogeneous vertex weight"):
        module.parse_obj_numeric(source)


def test_obj_repeated_index_triangle_is_deterministically_removed(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    module = load_source_auditor()
    monkeypatch.setattr(module, "WORKSPACE", tmp_path)
    source = tmp_path / "repeated-index.obj"
    source.write_text(
        "v 0 0 0\n"
        "v 1 0 0\n"
        "v 0 1 0\n"
        "f 1 2 3\n"
        "f 1 1 2\n",
        encoding="utf-8",
    )

    vertices, faces, stats = module.parse_obj_numeric_with_stats(source)

    assert module.np.array_equal(vertices, module.np.asarray([[0, 0, 0], [1, 0, 0], [0, 1, 0]], dtype="<f8"))
    assert module.np.array_equal(faces, module.np.asarray([[0, 1, 2]], dtype="<i8"))
    assert stats == {
        "input_triangle_count": 2,
        "degenerate_index_triangle_count": 1,
        "retained_triangle_count": 1,
    }
