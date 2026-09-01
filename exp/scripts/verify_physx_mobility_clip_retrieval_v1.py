#!/usr/bin/env python3
"""Independently verify the shared PhysX render/retrieval/materialization chain."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import stat
import zipfile
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path
from pathlib import PurePosixPath
from typing import Any

import numpy as np
import trimesh


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
WORKSPACE = Path("/mnt/zsn/lyb").resolve()
EXTERNAL_WORKSPACE = Path("/mnt/zsn/zsn_workspace").resolve()
REFERENCE = REPO / "exp/reference/table4_constraints_v2"
ADDENDUM = REFERENCE / "amendment_physx_mobility_clip_retrieval_v1_addendum1.json"
PROMPTS = REFERENCE / "prompts.jsonl"
PRIVATE_SOURCE = REPO / "exp/runtime/table4_constraints_v2/physx_mobility_clip_retrieval_v1_private_source"
RENDER_BUILD = REPO / "exp/runtime/table4_constraints_v2/physx_mobility_clip_retrieval_v1_render_build"
SNAPSHOT = REPO / "exp/runtime/table4_constraints_v2/physx_mobility_clip_retrieval_v1_snapshot"
DEFAULT_ROOT = REPO / "exp/runtime/table4_constraints_v2/physx_mobility_clip_retrieval_v1"
SELECTION_BUNDLE_NAME = "selection_bundle"
MATERIALIZATION_BUNDLE_NAME = "materialization_bundle"
SOURCE_AUDITOR = REPO / "exp/scripts/audit_physx_mobility_table4_source.py"
RENDER_WORKER = REPO / "exp/scripts/render_partnet_mobility_table4_opaque.py"
RENDER_BUILDER = REPO / "exp/scripts/build_partnet_mobility_table4_render_snapshot.py"
SNAPSHOT_LOCKER = REPO / "exp/scripts/lock_partnet_mobility_table4_render_snapshot.py"
SELECTION_LAUNCHER = REPO / "exp/scripts/launch_mobility_table4_clip_retrieval_selection.py"
SELECTOR = REPO / "exp/scripts/run_partnet_mobility_clip_retrieval_selection.py"
MATERIALIZER = REPO / "exp/scripts/run_physx_mobility_clip_retrieval_materialize.py"
CANONICALIZER = REPO / "exp/scripts/canonicalize_table4_artifact.py"
SELECTION_CONTRACT = REFERENCE / "selection_contract_physx_mobility_clip_retrieval_v1.json"
DATASET = Path("/mnt/zsn/zsn_workspace/Ctrl-3D-trellis2-controlnet-dev/demo/physical_edit_demo/third_party/physx_mobility/extracted/PhysX_mobility")
ARCHIVE = DATASET.parents[1] / "PhysX-Mobility.zip"
EXPECTED_ARCHIVE_BYTES = 937_374_668
EXPECTED_ARCHIVE_SHA256 = "88308cc2a4cc6177c59e32c2de51e881e6b961737295e5082d7ed01cca221908"
EXPECTED_ASSETS = 2024
EXPECTED_TASKS = [f"T4C{index:03d}" for index in range(1, 19)]
EXPECTED_PROMPTS_SHA256 = "0b46c15c65c46550cc25654ccca10e4de87f1b97e6bde89796ea3cc9b439065e"
SELECTION_LOCKED_FILES = {
    "asset_embeddings_run1.npy", "asset_embeddings_run2.npy",
    "prompt_embeddings_run1.npy", "prompt_embeddings_run2.npy",
    "embedding_index.jsonl", "prompt_tokenization.jsonl", "selection.jsonl",
    "selection_replay.jsonl", "selection_preflight.json",
}


def safe(path: Path, *, must_exist: bool = True) -> Path:
    resolved = path.resolve(strict=must_exist)
    if not any(
        resolved == root or root in resolved.parents
        for root in (WORKSPACE, EXTERNAL_WORKSPACE)
    ):
        raise RuntimeError(f"verifier path outside workspace: {resolved}")
    return resolved


def regular_file(path: Path) -> Path:
    resolved = safe(path)
    if path.is_symlink() or not stat.S_ISREG(path.lstat().st_mode):
        raise RuntimeError(f"not a regular non-symlink file: {path}")
    return resolved


def regular_dir(path: Path) -> Path:
    resolved = safe(path)
    if path.is_symlink() or not stat.S_ISDIR(path.lstat().st_mode):
        raise RuntimeError(f"not a regular non-symlink directory: {path}")
    return resolved


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with regular_file(path).open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def sha256_text(value: str) -> str:
    return hashlib.sha256(value.encode("utf-8")).hexdigest()


def read_json(path: Path) -> Any:
    return json.loads(regular_file(path).read_text(encoding="utf-8"))


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in regular_file(path).read_text(encoding="utf-8").splitlines() if line.strip()]


def archive_relative(info: zipfile.ZipInfo) -> str | None:
    filename = info.filename
    if "\\" in filename or not filename.startswith("PhysX_mobility/"):
        raise RuntimeError(f"archive member outside expected root: {filename!r}")
    relative = filename[len("PhysX_mobility/"):]
    if not relative:
        if info.is_dir():
            return None
        raise RuntimeError("archive has empty regular-file member path")
    normalized = PurePosixPath(relative)
    expected_text = normalized.as_posix() + ("/" if info.is_dir() else "")
    if (
        normalized.is_absolute()
        or any(part in {"", ".", ".."} for part in normalized.parts)
        or expected_text != relative
    ):
        raise RuntimeError(f"unsafe or non-normalized archive path: {filename!r}")
    mode = (info.external_attr >> 16) & 0o170000
    if mode == stat.S_IFLNK:
        raise RuntimeError(f"archive symlink forbidden: {filename}")
    return normalized.as_posix()


def stream_sha256(stream: Any, block_size: int = 8 * 1024 * 1024) -> tuple[str, int]:
    digest = hashlib.sha256()
    size = 0
    for block in iter(lambda: stream.read(block_size), b""):
        digest.update(block)
        size += len(block)
    return digest.hexdigest(), size


def verify_archive_extraction(manifest_path: Path, lock: dict[str, Any]) -> dict[str, str]:
    archive = regular_file(ARCHIVE)
    dataset = regular_dir(DATASET)
    if archive.stat().st_size != EXPECTED_ARCHIVE_BYTES or sha256_file(archive) != EXPECTED_ARCHIVE_SHA256:
        raise RuntimeError("pinned PhysX archive identity drifted")
    actual: set[str] = set()
    for directory, dirnames, filenames in os.walk(dataset, followlinks=False):
        current = Path(directory)
        for dirname in dirnames:
            child = current / dirname
            if child.is_symlink() or not stat.S_ISDIR(child.lstat().st_mode):
                raise RuntimeError(f"non-regular extracted directory: {child}")
        for filename in filenames:
            child = current / filename
            if child.is_symlink() or not stat.S_ISREG(child.lstat().st_mode):
                raise RuntimeError(f"non-regular extracted file: {child}")
            actual.add(child.relative_to(dataset).as_posix())
    observed_rows = read_jsonl(manifest_path)
    recomputed_rows: list[dict[str, Any]] = []
    hashes: dict[str, str] = {}
    with zipfile.ZipFile(archive, "r") as bundle:
        members: dict[str, zipfile.ZipInfo] = {}
        normalized_names: set[str] = set()
        for info in bundle.infolist():
            relative = archive_relative(info)
            normalized_key = (relative or "") + ("/" if info.is_dir() else "")
            if normalized_key in normalized_names:
                raise RuntimeError(f"duplicate normalized archive member: {normalized_key!r}")
            normalized_names.add(normalized_key)
            if info.is_dir():
                continue
            if relative is None or relative in members:
                raise RuntimeError(f"duplicate regular archive member: {relative!r}")
            members[relative] = info
        if actual != set(members):
            raise RuntimeError("archive/extraction path closure drifted")
        for index, relative in enumerate(sorted(members), 1):
            info = members[relative]
            extracted = regular_file(dataset / relative)
            if extracted.stat().st_size != info.file_size:
                raise RuntimeError(f"archive/extraction size mismatch: {relative}")
            with bundle.open(info, "r") as source_stream, extracted.open("rb") as extracted_stream:
                source_hash, source_size = stream_sha256(source_stream)
                extracted_hash, extracted_size = stream_sha256(extracted_stream)
            if (
                source_size != info.file_size
                or extracted_size != info.file_size
                or source_hash != extracted_hash
            ):
                raise RuntimeError(f"archive/extraction byte mismatch: {relative}")
            recomputed_rows.append({
                "relative_path": relative,
                "bytes": info.file_size,
                "crc32": f"{info.CRC:08x}",
                "sha256": source_hash,
            })
            hashes[relative] = source_hash
            if index % 10000 == 0:
                print(f"[verify-archive] {index}/{len(members)}", flush=True)
    if recomputed_rows != observed_rows:
        raise RuntimeError("archive member manifest differs from independent recomputation")
    if (
        lock.get("archive_regular_file_count") != len(recomputed_rows)
        or lock.get("archive_uncompressed_bytes") != sum(row["bytes"] for row in recomputed_rows)
    ):
        raise RuntimeError("archive aggregate evidence drifted")
    return hashes


def parse_obj_numeric(path: Path) -> tuple[np.ndarray, np.ndarray]:
    vertices: list[tuple[float, float, float]] = []
    faces: list[tuple[int, int, int]] = []
    with regular_file(path).open("r", encoding="utf-8", errors="strict", newline=None) as stream:
        for line_number, line in enumerate(stream, 1):
            stripped = line.lstrip()
            if not stripped or stripped.startswith("#"):
                continue
            fields = stripped.split()
            if fields[0] == "v":
                if len(fields) not in {4, 5}:
                    raise RuntimeError(f"unsupported OBJ vertex at {path}:{line_number}")
                values = tuple(float(value) for value in fields[1:])
                if not all(math.isfinite(value) for value in values):
                    raise RuntimeError(f"non-finite OBJ vertex at {path}:{line_number}")
                if len(values) == 4:
                    if values[3] == 0.0:
                        raise RuntimeError(f"zero homogeneous OBJ vertex weight at {path}:{line_number}")
                    values = tuple(value / values[3] for value in values[:3])
                vertices.append((values[0], values[1], values[2]))
            elif fields[0] == "f":
                if len(fields) < 4:
                    raise RuntimeError(f"degenerate OBJ face at {path}:{line_number}")
                polygon: list[int] = []
                for token in fields[1:]:
                    vertex_token = token.split("/", 1)[0]
                    if not vertex_token:
                        raise RuntimeError(f"OBJ face lacks vertex index at {path}:{line_number}")
                    raw_index = int(vertex_token)
                    if raw_index == 0:
                        raise RuntimeError(f"OBJ vertex index zero at {path}:{line_number}")
                    index = raw_index - 1 if raw_index > 0 else len(vertices) + raw_index
                    if index < 0 or index >= len(vertices):
                        raise RuntimeError(f"OBJ vertex index out of range at {path}:{line_number}")
                    polygon.append(index)
                for offset in range(1, len(polygon) - 1):
                    faces.append((polygon[0], polygon[offset], polygon[offset + 1]))
    vertex_array = np.ascontiguousarray(vertices, dtype="<f8")
    face_array = np.ascontiguousarray(faces, dtype="<i8")
    if (
        vertex_array.ndim != 2
        or vertex_array.shape[1:] != (3,)
        or face_array.ndim != 2
        or face_array.shape[1:] != (3,)
        or len(vertex_array) == 0
        or len(face_array) == 0
        or not np.isfinite(vertex_array).all()
    ):
        raise RuntimeError(f"invalid numeric OBJ geometry: {path}")
    return vertex_array, face_array


def urdf_vector(text: str | None, default: tuple[float, float, float]) -> tuple[float, float, float]:
    values = default if text is None else tuple(float(token) for token in text.split())
    if len(values) == 1:
        values = values * 3
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise RuntimeError(f"invalid URDF vector: {text!r}")
    return values


def urdf_origin(node: ET.Element | None) -> np.ndarray:
    if node is None:
        return np.eye(4)
    xyz = urdf_vector(node.attrib.get("xyz"), (0.0, 0.0, 0.0))
    roll, pitch, yaw = urdf_vector(node.attrib.get("rpy"), (0.0, 0.0, 0.0))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    result = np.eye(4)
    result[:3, :3] = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])
    result[:3, 3] = xyz
    return result


def verify_urdf_source_binding(
    binding: dict[str, Any],
    derivation: dict[str, Any],
    archive_hashes: dict[str, str],
) -> None:
    identity = binding["source_asset_id"]
    urdf = regular_file(DATASET / "urdf" / f"{identity}.urdf")
    finaljson = regular_file(DATASET / "finaljson" / f"{identity}.json")
    if (
        Path(binding["source_urdf"]).resolve() != urdf
        or Path(binding["source_finaljson"]).resolve() != finaljson
        or binding["source_urdf_sha256"] != sha256_file(urdf)
        or binding["source_finaljson_sha256"] != sha256_file(finaljson)
    ):
        raise RuntimeError(f"private source identity/hash binding drifted: {identity}")
    root = ET.parse(urdf).getroot()
    link_nodes = root.findall("link")
    links = {link.attrib["name"]: link for link in link_nodes}
    if not links or len(links) != len(link_nodes):
        raise RuntimeError(f"missing or duplicate URDF link name: {identity}")
    parents: dict[str, tuple[str, np.ndarray]] = {}
    for joint in root.findall("joint"):
        parent, child = joint.find("parent"), joint.find("child")
        parent_name = parent.attrib.get("link", "") if parent is not None else ""
        child_name = child.attrib.get("link", "") if child is not None else ""
        if (
            parent is None
            or child is None
            or parent_name not in links
            or child_name not in links
            or parent_name == child_name
            or child_name in parents
        ):
            raise RuntimeError(f"invalid URDF tree: {identity}")
        parents[child_name] = (parent_name, urdf_origin(joint.find("origin")))
    if len(set(links) - set(parents)) != 1:
        raise RuntimeError(f"URDF root-link closure drifted: {identity}")
    pending = set(links)
    world: dict[str, np.ndarray] = {}
    while pending:
        progressed = False
        for name in sorted(pending):
            if name in parents:
                parent, transform = parents[name]
                if parent not in world:
                    continue
                world[name] = world[parent] @ transform
            else:
                world[name] = np.eye(4)
            pending.remove(name)
            progressed = True
        if not progressed:
            raise RuntimeError(f"unresolved URDF graph: {identity}")
    expected_instance_rows: list[tuple[str, str]] = []
    mesh_links = 0
    visuals = 0
    for name in sorted(links):
        link_visuals = links[name].findall("visual")
        if link_visuals:
            mesh_links += 1
            if not np.array_equal(world[name], np.eye(4)):
                raise RuntimeError(f"mesh-link q0 transform is not identity: {identity}/{name}")
        for visual in link_visuals:
            visuals += 1
            visual_transform = urdf_origin(visual.find("origin"))
            if not np.array_equal(visual_transform, np.eye(4)):
                raise RuntimeError(f"visual origin is not identity: {identity}/{name}")
            mesh = visual.find("geometry/mesh")
            if mesh is None or urdf_vector(mesh.attrib.get("scale"), (1.0, 1.0, 1.0)) != (1.0, 1.0, 1.0):
                raise RuntimeError(f"non-mesh or scaled visual: {identity}/{name}")
            reference = Path(mesh.attrib.get("filename", ""))
            mesh_path = (urdf.parent / reference).resolve()
            object_root = (DATASET / "partseg" / identity / "objs").resolve()
            if reference.is_absolute() or ".." not in reference.parts or object_root not in mesh_path.parents:
                raise RuntimeError(f"visual reference escaped identity OBJ root: {identity}/{reference}")
            relative = mesh_path.relative_to(DATASET).as_posix()
            if archive_hashes.get(relative) != sha256_file(mesh_path):
                raise RuntimeError(f"URDF visual is not archive-bound: {identity}/{relative}")
            transform = np.ascontiguousarray(world[name] @ visual_transform, dtype="<f8")
            expected_instance_rows.append((
                relative,
                hashlib.sha256(transform.tobytes(order="C")).hexdigest(),
            ))
    if mesh_links != binding["mesh_bearing_links"] or visuals != binding["visual_count"]:
        raise RuntimeError(f"private source hierarchy counts drifted: {identity}")
    observed_instance_rows = [
        (row["source_member_path"], row["q0_transform_sha256"])
        for row in derivation["ordered_source_instances"]
    ]
    if Counter(expected_instance_rows) != Counter(observed_instance_rows):
        raise RuntimeError(f"URDF visual/derivation multiset drifted: {identity}")


def reconstruct_numeric_derivation(
    derivation: dict[str, Any],
    archive_hashes: dict[str, str],
) -> None:
    source_instances = derivation["ordered_source_instances"]
    identity_transform = np.ascontiguousarray(np.eye(4), dtype="<f8")
    identity_transform_bytes = identity_transform.tobytes(order="C")
    identity_transform_sha256 = hashlib.sha256(identity_transform_bytes).hexdigest()
    all_vertices: list[np.ndarray] = []
    all_faces: list[np.ndarray] = []
    vertex_offset = 0
    ordering: list[str] = []
    for row in source_instances:
        expected_fields = {
            "source_member_path", "source_member_path_sha256", "source_obj_sha256",
            "q0_transform_sha256", "numeric_vertices_sha256", "numeric_faces_sha256",
            "numeric_instance_order_sha256", "source_vertex_count", "source_face_count",
        }
        if set(row) != expected_fields:
            raise RuntimeError(
                f"numeric source-instance schema drifted: {derivation['candidate_key']}"
            )
        relative = row["source_member_path"]
        member = PurePosixPath(relative)
        if (
            member.is_absolute()
            or any(part in {"", ".", ".."} for part in member.parts)
            or member.parts[:1] != ("partseg",)
            or member.suffix.lower() != ".obj"
            or row["source_member_path_sha256"] != sha256_text(relative)
            or archive_hashes.get(relative) != row["source_obj_sha256"]
            or row["q0_transform_sha256"] != identity_transform_sha256
        ):
            raise RuntimeError(
                f"numeric source-instance provenance/q0 drifted: {derivation['candidate_key']}"
            )
        source = regular_file(DATASET / Path(*member.parts))
        if sha256_file(source) != row["source_obj_sha256"]:
            raise RuntimeError(f"numeric source OBJ bytes drifted: {relative}")
        vertices, faces = parse_obj_numeric(source)
        vertex_bytes = vertices.tobytes(order="C")
        face_bytes = faces.tobytes(order="C")
        expected = {
            "numeric_vertices_sha256": hashlib.sha256(vertex_bytes).hexdigest(),
            "numeric_faces_sha256": hashlib.sha256(face_bytes).hexdigest(),
            "numeric_instance_order_sha256": hashlib.sha256(
                vertex_bytes + face_bytes + identity_transform_bytes
            ).hexdigest(),
            "source_vertex_count": int(len(vertices)),
            "source_face_count": int(len(faces)),
        }
        for field, value in expected.items():
            if row.get(field) != value:
                raise RuntimeError(
                    f"numeric source-instance field drifted: "
                    f"{derivation['candidate_key']}/{field}"
                )
        ordering.append(row["numeric_instance_order_sha256"])
        all_vertices.append(vertices)
        all_faces.append(np.ascontiguousarray(faces + vertex_offset, dtype="<i8"))
        vertex_offset += len(vertices)
    if ordering != sorted(ordering):
        raise RuntimeError(f"numeric instance order drifted: {derivation['candidate_key']}")
    combined_vertices = np.ascontiguousarray(np.concatenate(all_vertices, axis=0), dtype="<f8")
    combined_faces = np.ascontiguousarray(np.concatenate(all_faces, axis=0), dtype="<i8")
    expected_combined = {
        "vertex_count": int(len(combined_vertices)),
        "face_count": int(len(combined_faces)),
        "vertices_content_sha256": hashlib.sha256(
            combined_vertices.tobytes(order="C")
        ).hexdigest(),
        "faces_content_sha256": hashlib.sha256(
            combined_faces.tobytes(order="C")
        ).hexdigest(),
    }
    for field, value in expected_combined.items():
        if derivation.get(field) != value:
            raise RuntimeError(
                f"reconstructed numeric candidate differs from derived NPZ: "
                f"{derivation['candidate_key']}/{field}"
            )


def exact_regular_files(directory: Path) -> dict[str, Path]:
    directory = regular_dir(directory)
    result: dict[str, Path] = {}
    for entry in directory.iterdir():
        if entry.is_symlink() or not stat.S_ISREG(entry.lstat().st_mode):
            raise RuntimeError(f"directory contains non-regular file: {entry}")
        result[entry.name] = entry
    return result


def validate_numeric_npz(path: Path, derivation: dict[str, Any]) -> None:
    path = regular_file(path)
    with zipfile.ZipFile(path, "r") as bundle:
        infos = bundle.infolist()
        if [info.filename for info in infos] != ["vertices.npy", "faces.npy"]:
            raise RuntimeError(f"numeric NPZ entry closure/order drifted: {path}")
        arrays: dict[str, np.ndarray] = {}
        for info in infos:
            if (
                info.is_dir()
                or info.compress_type != zipfile.ZIP_STORED
                or info.date_time != (1980, 1, 1, 0, 0, 0)
                or info.create_system != 3
                or ((info.external_attr >> 16) & 0o177777) != (stat.S_IFREG | 0o644)
                or info.flag_bits & 0x1
            ):
                raise RuntimeError(f"numeric NPZ metadata drifted: {path}/{info.filename}")
            with bundle.open(info, "r") as stream:
                arrays[info.filename] = np.load(stream, allow_pickle=False)
    vertices = arrays["vertices.npy"]
    faces = arrays["faces.npy"]
    if vertices.dtype.str != "<f8" or vertices.ndim != 2 or vertices.shape[1:] != (3,):
        raise RuntimeError(f"numeric vertices schema drifted: {path}")
    if faces.dtype.str != "<i8" or faces.ndim != 2 or faces.shape[1:] != (3,):
        raise RuntimeError(f"numeric faces schema drifted: {path}")
    if (
        len(vertices) == 0
        or len(faces) == 0
        or not np.isfinite(vertices).all()
        or faces.min() < 0
        or faces.max() >= len(vertices)
    ):
        raise RuntimeError(f"numeric NPZ geometry closure drifted: {path}")
    expected = {
        "vertices_dtype": vertices.dtype.str,
        "faces_dtype": faces.dtype.str,
        "vertex_count": int(len(vertices)),
        "face_count": int(len(faces)),
        "vertices_content_sha256": hashlib.sha256(
            np.ascontiguousarray(vertices).tobytes(order="C")
        ).hexdigest(),
        "faces_content_sha256": hashlib.sha256(
            np.ascontiguousarray(faces).tobytes(order="C")
        ).hexdigest(),
        "npz_entries": ["vertices.npy", "faces.npy"],
        "npz_compression": "ZIP_STORED",
    }
    for field, expected_value in expected.items():
        if derivation.get(field) != expected_value:
            raise RuntimeError(f"numeric derivation field drifted: {path}/{field}")


def implementation_hashes() -> dict[str, str]:
    paths = {
        "source_auditor_sha256": SOURCE_AUDITOR,
        "render_worker_sha256": RENDER_WORKER,
        "render_builder_sha256": RENDER_BUILDER,
        "snapshot_locker_sha256": SNAPSHOT_LOCKER,
        "selection_launcher_sha256": SELECTION_LAUNCHER,
        "selector_sha256": SELECTOR,
        "materializer_sha256": MATERIALIZER,
        "integrity_verifier_sha256": SCRIPT,
        "canonicalizer_sha256": CANONICALIZER,
    }
    return {field: sha256_file(path) for field, path in paths.items()}


def validate_amendment() -> tuple[dict[str, Any], str, dict[str, str]]:
    addendum = read_json(ADDENDUM)
    observed = implementation_hashes()
    addendum_sha = sha256_file(ADDENDUM)
    if addendum.get("status") != "pre_result_frozen" or addendum.get("implementation") != observed:
        raise RuntimeError("PhysX addendum implementation binding drifted")
    if sha256_file(PROMPTS) != EXPECTED_PROMPTS_SHA256:
        raise RuntimeError("prompt manifest drifted")
    shared = addendum.get("shared_pipeline", {})
    if shared.get("selection_contract_sha256") != sha256_file(SELECTION_CONTRACT):
        raise RuntimeError("PhysX selection execution contract binding drifted")
    return addendum, addendum_sha, observed


def verify_source(addendum_sha: str, implementation: dict[str, str]) -> tuple[dict[str, Any], dict[str, dict[str, Any]]]:
    lock_path = PRIVATE_SOURCE / "source_audit.lock.json"
    lock = read_json(lock_path)
    if lock.get("status") != "PASS" or lock.get("asset_count") != EXPECTED_ASSETS:
        raise RuntimeError("private source lock is not full PASS")
    if lock.get("amendment_sha256") != addendum_sha or lock.get("implementation") != implementation:
        raise RuntimeError("private source implementation/addendum binding drifted")
    if lock.get("mesh_bearing_link_count") != 14096 or lock.get("visual_count") != 91855:
        raise RuntimeError("private zero-pose aggregate counts drifted")
    invariant = lock.get("zero_pose_geometry_invariant", {})
    if invariant != {
        "all_mesh_bearing_link_world_transforms_identity_at_q0": True,
        "all_visual_origins_identity": True,
        "all_mesh_scales_one": True,
        "maximum_transform_error": 0.0,
    }:
        raise RuntimeError("private zero-pose invariant drifted")
    expected_camera_frame = [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]]
    if lock.get("camera_frame_source_to_canonical") != expected_camera_frame:
        raise RuntimeError("PhysX official camera frame drifted")
    source_binding_path = PRIVATE_SOURCE / "source_binding.jsonl"
    render_plan_path = PRIVATE_SOURCE / "render_plan.jsonl"
    derivation_path = PRIVATE_SOURCE / "geometry_derivation_manifest.jsonl"
    archive_manifest_path = PRIVATE_SOURCE / "archive_member_manifest.jsonl"
    if (
        sha256_file(source_binding_path) != lock.get("source_binding_sha256")
        or sha256_file(render_plan_path) != lock.get("render_plan_sha256")
        or sha256_file(derivation_path) != lock.get("geometry_derivation_manifest_sha256")
        or sha256_file(archive_manifest_path) != lock.get("archive_member_manifest_sha256")
    ):
        raise RuntimeError("private source manifests drifted")
    archive_hashes = verify_archive_extraction(archive_manifest_path, lock)
    bindings = read_jsonl(source_binding_path)
    plans = read_jsonl(render_plan_path)
    derivations = read_jsonl(derivation_path)
    if len(bindings) != EXPECTED_ASSETS or len(plans) != EXPECTED_ASSETS or len(derivations) != EXPECTED_ASSETS:
        raise RuntimeError("private source manifest cardinality drifted")
    by_key = {row["candidate_key"]: row for row in bindings}
    ordered_keys = [row["candidate_key"] for row in plans]
    if (
        len(by_key) != EXPECTED_ASSETS
        or ordered_keys != sorted(by_key)
        or [row.get("candidate_key") for row in derivations] != ordered_keys
    ):
        raise RuntimeError("private source opaque key order/uniqueness drifted")
    derivation_by_key = {row["candidate_key"]: row for row in derivations}
    if len(derivation_by_key) != EXPECTED_ASSETS:
        raise RuntimeError("private geometry derivation key uniqueness drifted")
    expected_top = {
        "archive_member_manifest.jsonl", "geometry_blobs", "geometry_scenes",
        "geometry_derivation_manifest.jsonl", "render_plan.jsonl", "source_binding.jsonl",
        "source_audit.lock.json",
    }
    if {entry.name for entry in regular_dir(PRIVATE_SOURCE).iterdir()} != expected_top:
        raise RuntimeError("private source top-level closure drifted")
    scene_files = exact_regular_files(PRIVATE_SOURCE / "geometry_scenes")
    blob_files = exact_regular_files(PRIVATE_SOURCE / "geometry_blobs")
    if set(scene_files) != {f"{key}.json" for key in ordered_keys}:
        raise RuntimeError("private geometry scene file closure drifted")
    referenced_blobs: dict[str, str] = {}
    for row in plans:
        if set(row) != {"candidate_key", "geometry_scene", "geometry_scene_sha256"}:
            raise RuntimeError(f"private render-plan schema drifted: {row.get('candidate_key')}")
        scene = PRIVATE_SOURCE / row["geometry_scene"]
        if sha256_file(scene) != row["geometry_scene_sha256"]:
            raise RuntimeError(f"private geometry scene drifted: {row['candidate_key']}")
        payload = read_json(scene)
        if set(payload) != {"schema_version", "camera_frame", "mesh_instances"} or payload["schema_version"] != 2 or payload["camera_frame"] != expected_camera_frame:
            raise RuntimeError(f"private geometry scene schema/frame drifted: {row['candidate_key']}")
        if not isinstance(payload["mesh_instances"], list) or len(payload["mesh_instances"]) != 1:
            raise RuntimeError(f"private geometry scene must contain one numeric instance: {row['candidate_key']}")
        instance = payload["mesh_instances"][0]
        if set(instance) != {"mesh_blob", "mesh_sha256", "transform"} or instance["transform"] != np.eye(4).tolist():
            raise RuntimeError(f"private numeric instance schema/transform drifted: {row['candidate_key']}")
        blob = Path(instance["mesh_blob"])
        if (
            blob.is_absolute()
            or ".." in blob.parts
            or blob.parts[:1] != ("geometry_blobs",)
            or len(blob.parts) != 2
            or blob.suffix != ".npz"
            or blob.name != f"{instance['mesh_sha256']}.npz"
        ):
            raise RuntimeError(f"private numeric blob path/content address drifted: {row['candidate_key']}")
        previous = referenced_blobs.setdefault(blob.name, instance["mesh_sha256"])
        if previous != instance["mesh_sha256"]:
            raise RuntimeError(f"private numeric blob hash conflict: {blob.name}")
        derivation = derivation_by_key[row["candidate_key"]]
        if (
            derivation.get("schema_version") != 1
            or derivation.get("derived_blob") != instance["mesh_blob"]
            or derivation.get("derived_blob_sha256") != instance["mesh_sha256"]
        ):
            raise RuntimeError(f"private numeric derivation/scene binding drifted: {row['candidate_key']}")
        source_instances = derivation.get("ordered_source_instances")
        if not isinstance(source_instances, list) or not source_instances:
            raise RuntimeError(f"private numeric source derivation missing: {row['candidate_key']}")
        ordering = [item.get("numeric_instance_order_sha256") for item in source_instances]
        if (
            ordering != sorted(ordering)
            or any(not isinstance(value, str) or len(value) != 64 for value in ordering)
            or sum(item.get("source_vertex_count", 0) for item in source_instances) != derivation.get("vertex_count")
            or sum(item.get("source_face_count", 0) for item in source_instances) != derivation.get("face_count")
        ):
            raise RuntimeError(f"private numeric source ordering/count closure drifted: {row['candidate_key']}")
        binding = by_key[row["candidate_key"]]
        verify_urdf_source_binding(binding, derivation, archive_hashes)
        reconstruct_numeric_derivation(derivation, archive_hashes)
        if (len(referenced_blobs) % 100) == 0:
            print(f"[verify-source] {len(referenced_blobs)}/{EXPECTED_ASSETS}", flush=True)
    if set(blob_files) != set(referenced_blobs):
        raise RuntimeError("private numeric blob file/reference closure drifted")
    for derivation in derivations:
        blob_name = Path(derivation["derived_blob"]).name
        if sha256_file(blob_files[blob_name]) != derivation["derived_blob_sha256"]:
            raise RuntimeError(f"private numeric blob bytes drifted: {blob_name}")
        validate_numeric_npz(blob_files[blob_name], derivation)
    return lock, by_key


def verify_render_and_snapshot(source_lock: dict[str, Any], addendum_sha: str, implementation: dict[str, str]) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    build_lock = read_json(RENDER_BUILD / "render_build.lock.json")
    if build_lock.get("status") != "PASS" or build_lock.get("formal") is not True:
        raise RuntimeError("render build is not formal PASS")
    if build_lock.get("candidate_assets") != EXPECTED_ASSETS or build_lock.get("eligible_assets") != EXPECTED_ASSETS or build_lock.get("ineligible_assets") != 0:
        raise RuntimeError("render build candidate closure drifted")
    if build_lock.get("run_workers") != {"run1": 1, "run2": 4} or build_lock.get("fresh_render_runs") != 2 or build_lock.get("cross_worker_full_render_replay_byte_identical") is not True:
        raise RuntimeError("render replay policy drifted")
    if build_lock.get("private_source_audit_lock_sha256") != sha256_file(PRIVATE_SOURCE / "source_audit.lock.json"):
        raise RuntimeError("render/private source binding drifted")
    if build_lock.get("source_binding_sha256") != source_lock["source_binding_sha256"]:
        raise RuntimeError("render/source identity binding drifted")
    if build_lock.get("render_worker_sha256") != implementation["render_worker_sha256"] or build_lock.get("snapshot_builder_sha256") != implementation["render_builder_sha256"]:
        raise RuntimeError("render implementation binding drifted")
    inventory_path = RENDER_BUILD / "candidate_inventory.jsonl"
    if sha256_file(inventory_path) != build_lock.get("candidate_inventory_sha256"):
        raise RuntimeError("render candidate inventory drifted")
    inventory = read_jsonl(inventory_path)
    if len(inventory) != EXPECTED_ASSETS or inventory != sorted(inventory, key=lambda row: row["candidate_key"]):
        raise RuntimeError("render candidate count/order drifted")
    worker_fingerprint = build_lock.get("all_worker_runtime_fingerprint")
    required_worker_fingerprint = {
        "driver_version", "egl_device_id", "egl_client_apis", "egl_vendor", "egl_version",
        "gl_renderer", "gl_vendor", "gl_version", "glsl_version", "gl_device_uuid",
        "numpy", "openblas_num_threads", "omp_num_threads", "mkl_num_threads",
        "numexpr_num_threads", "pillow", "python_zlib_compile", "python_zlib_runtime",
        "pillow_zlib", "pillow_zlib_ng", "pyopengl", "pyopengl_accelerate",
        "pyopengl_platform", "python", "physical_gpu_name", "physical_gpu_uuid",
        "physical_gpu_memory_mib", "physical_gpu_compute_capability", "pyrender", "trimesh",
    }
    if (
        not isinstance(worker_fingerprint, dict)
        or set(worker_fingerprint) != required_worker_fingerprint
        or worker_fingerprint.get("egl_device_id") != "1"
        or worker_fingerprint.get("physical_gpu_uuid") != "GPU-7390aec1-d177-6672-4136-d998c85f489d"
        or worker_fingerprint.get("gl_device_uuid") != "7390aec1d17766724136d998c85f489d"
        or not all(worker_fingerprint.values())
    ):
        raise RuntimeError("render worker runtime/GPU fingerprint drifted")
    parent_fingerprint = build_lock.get("runtime_fingerprint")
    if (
        not isinstance(parent_fingerprint, dict)
        or parent_fingerprint.get("EGL_DEVICE_ID") != "1"
        or parent_fingerprint.get("gl_device_uuid") != worker_fingerprint["gl_device_uuid"]
        or parent_fingerprint.get("python_zlib_compile") != worker_fingerprint["python_zlib_compile"]
        or parent_fingerprint.get("python_zlib_runtime") != worker_fingerprint["python_zlib_runtime"]
        or parent_fingerprint.get("pillow_zlib") != worker_fingerprint["pillow_zlib"]
        or parent_fingerprint.get("pillow_zlib_ng") != worker_fingerprint["pillow_zlib_ng"]
        or not all(parent_fingerprint.values())
    ):
        raise RuntimeError("render parent runtime/GPU fingerprint drifted")
    required_fields = {"candidate_key", "identity_sha256", "snapshot_render_dir", "eligible", "views", "render_sha256"}
    keys = {row["candidate_key"] for row in inventory}
    if len(keys) != EXPECTED_ASSETS or any(
        len(key) != 64 or any(character not in "0123456789abcdef" for character in key)
        for key in keys
    ):
        raise RuntimeError("render opaque candidate-key closure drifted")
    for run_name in ("run1", "run2"):
        run_root = regular_dir(RENDER_BUILD / run_name)
        observed: set[str] = set()
        for entry in os.scandir(run_root):
            path = Path(entry.path)
            if entry.is_symlink() or not entry.is_dir(follow_symlinks=False):
                raise RuntimeError(f"render run contains non-directory: {path}")
            observed.add(entry.name)
        if observed != keys:
            raise RuntimeError(f"render run candidate-directory closure drifted: {run_name}")
    for index, row in enumerate(inventory, 1):
        key = row["candidate_key"]
        if set(row) != required_fields or row["identity_sha256"] != key or row["eligible"] is not True or row["views"] != 8:
            raise RuntimeError(f"render candidate schema drifted: {key}")
        names = [f"{view:03d}.png" for view in range(8)]
        if set(row["render_sha256"]) != set(names):
            raise RuntimeError(f"render view hash closure drifted: {key}")
        expected_directory_names = set(names) | {"render.record.json"}
        run_directories = [regular_dir(RENDER_BUILD / run_name / key) for run_name in ("run1", "run2")]
        for directory in run_directories:
            if {entry.name for entry in directory.iterdir()} != expected_directory_names:
                raise RuntimeError(f"render candidate file closure drifted: {directory}")
            for entry in directory.iterdir():
                regular_file(entry)
        record1_path = regular_file(run_directories[0] / "render.record.json")
        record2_path = regular_file(run_directories[1] / "render.record.json")
        if record1_path.read_bytes() != record2_path.read_bytes():
            raise RuntimeError(f"fresh render record bytes differ: {key}")
        record = read_json(record1_path)
        if record.get("runtime_fingerprint") != worker_fingerprint:
            raise RuntimeError(f"render record worker fingerprint drifted: {key}")
        public_record = {field: value for field, value in record.items() if field != "runtime_fingerprint"}
        if public_record != row:
            raise RuntimeError(f"render record/inventory row differs: {key}")
        for name in names:
            run1 = regular_file(RENDER_BUILD / "run1" / key / name)
            run2 = regular_file(RENDER_BUILD / "run2" / key / name)
            expected = row["render_sha256"][name]
            if sha256_file(run1) != expected or sha256_file(run2) != expected or run1.read_bytes() != run2.read_bytes():
                raise RuntimeError(f"fresh render replay differs: {key}/{name}")
        if index % 100 == 0:
            print(f"[verify-render] {index}/{len(inventory)}", flush=True)
    snapshot_lock = read_json(SNAPSHOT / "snapshot.lock.json")
    snapshot_inventory_path = SNAPSHOT / "candidate_inventory.jsonl"
    if snapshot_lock.get("status") != "PASS" or snapshot_lock.get("candidate_assets") != EXPECTED_ASSETS or snapshot_lock.get("eligible_assets") != EXPECTED_ASSETS:
        raise RuntimeError("opaque snapshot is not full PASS")
    if snapshot_lock.get("render_build_lock_sha256") != sha256_file(RENDER_BUILD / "render_build.lock.json"):
        raise RuntimeError("snapshot/render build binding drifted")
    if snapshot_lock.get("render_worker_sha256") != implementation["render_worker_sha256"] or snapshot_lock.get("snapshot_locker_sha256") != implementation["snapshot_locker_sha256"]:
        raise RuntimeError("snapshot implementation binding drifted")
    if snapshot_lock.get("candidate_inventory_sha256") != sha256_file(snapshot_inventory_path):
        raise RuntimeError("snapshot candidate inventory drifted")
    if snapshot_lock.get("selection_execution_contract_sha256") != sha256_file(SELECTION_CONTRACT):
        raise RuntimeError("snapshot execution-contract binding drifted")
    if sha256_file(SNAPSHOT / "selection.execution_contract.json") != sha256_file(SELECTION_CONTRACT):
        raise RuntimeError("selector-visible execution contract drifted")
    if regular_file(snapshot_inventory_path).read_bytes() != regular_file(inventory_path).read_bytes():
        raise RuntimeError("snapshot/render candidate inventories differ")
    for row in inventory:
        key = row["candidate_key"]
        snapshot_dir = regular_dir(SNAPSHOT / "renders" / key)
        if sorted(entry.name for entry in snapshot_dir.iterdir()) != [f"{view:03d}.png" for view in range(8)]:
            raise RuntimeError(f"snapshot render closure drifted: {key}")
        for name, expected in row["render_sha256"].items():
            if sha256_file(snapshot_dir / name) != expected:
                raise RuntimeError(f"snapshot render hash drifted: {key}/{name}")
    return snapshot_lock, inventory


def verify_selection(root: Path, snapshot_lock: dict[str, Any], inventory: list[dict[str, Any]], addendum_sha: str, implementation: dict[str, str]) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    selection_bundle = regular_dir(root / SELECTION_BUNDLE_NAME)
    lock = read_json(selection_bundle / "selection.lock.json")
    if lock.get("phase") != "selection_locked_before_source_binding_or_geometry_access":
        raise RuntimeError("selection lock phase drifted")
    shared_selection_hashes = {
        "selection_launcher_sha256": implementation["selection_launcher_sha256"],
        "selector_sha256": implementation["selector_sha256"],
    }
    if lock.get("implementation") != shared_selection_hashes:
        raise RuntimeError("selection implementation binding drifted")
    if lock.get("selection_execution_contract_sha256") != sha256_file(SELECTION_CONTRACT):
        raise RuntimeError("selection execution-contract binding drifted")
    if lock.get("snapshot_lock_sha256") != sha256_file(SNAPSHOT / "snapshot.lock.json"):
        raise RuntimeError("selection/snapshot binding drifted")
    if lock.get("candidate_assets") != EXPECTED_ASSETS or lock.get("eligible_assets") != EXPECTED_ASSETS or lock.get("task_count") != 18:
        raise RuntimeError("selection cardinality drifted")
    if lock.get("full_embedding_replays") != 2 or lock.get("full_embedding_replay_byte_identical") is not True or lock.get("repair_attempts") != 0 or lock.get("rank_fallbacks") != 0:
        raise RuntimeError("selection replay/repair/fallback policy drifted")
    if lock.get("geometry_access_during_selection") is not False:
        raise RuntimeError("selection geometry-access declaration drifted")
    if set(lock.get("locked_file_sha256", {})) != SELECTION_LOCKED_FILES:
        raise RuntimeError("selection locked-file set drifted")
    if {entry.name for entry in selection_bundle.iterdir()} != SELECTION_LOCKED_FILES | {"selection.lock.json"}:
        raise RuntimeError("selection bundle exact closure drifted")
    for entry in selection_bundle.iterdir():
        if entry.is_symlink() or not stat.S_ISREG(entry.lstat().st_mode):
            raise RuntimeError(f"selection bundle contains non-regular file: {entry}")
    for name, expected in lock["locked_file_sha256"].items():
        if Path(name).name != name or sha256_file(selection_bundle / name) != expected:
            raise RuntimeError(f"selection locked evidence drifted: {name}")
    selections = read_jsonl(selection_bundle / "selection.jsonl")
    replay = read_jsonl(selection_bundle / "selection_replay.jsonl")
    index = read_jsonl(selection_bundle / "embedding_index.jsonl")
    token_rows = read_jsonl(selection_bundle / "prompt_tokenization.jsonl")
    prompts = read_jsonl(PROMPTS)
    expected_fields = {
        "task_id", "prompt_sha256", "selected_embedding_row", "selected_candidate_key",
        "clip_cosine_similarity", "tie_break", "fallback_allowed",
    }
    if selections != replay or len(selections) != 18 or [row.get("task_id") for row in selections] != EXPECTED_TASKS:
        raise RuntimeError("selection/replay task closure drifted")
    if len(index) != EXPECTED_ASSETS or [row["candidate_key"] for row in index] != [row["candidate_key"] for row in inventory]:
        raise RuntimeError("embedding index/candidate inventory binding drifted")
    if len(token_rows) != 18 or [row["task_id"] for row in token_rows] != EXPECTED_TASKS:
        raise RuntimeError("prompt tokenization task closure drifted")
    assets1 = np.load(regular_file(selection_bundle / "asset_embeddings_run1.npy"), allow_pickle=False)
    assets2 = np.load(regular_file(selection_bundle / "asset_embeddings_run2.npy"), allow_pickle=False)
    text1 = np.load(regular_file(selection_bundle / "prompt_embeddings_run1.npy"), allow_pickle=False)
    text2 = np.load(regular_file(selection_bundle / "prompt_embeddings_run2.npy"), allow_pickle=False)
    if assets1.shape != (EXPECTED_ASSETS, 512) or text1.shape != (18, 512):
        raise RuntimeError("selection embedding shapes drifted")
    if not np.array_equal(assets1, assets2) or not np.array_equal(text1, text2):
        raise RuntimeError("selection embedding replay differs")
    if not np.isfinite(assets1).all() or not np.isfinite(text1).all():
        raise RuntimeError("selection embeddings contain non-finite values")
    similarities = text1 @ assets1.T
    for task_index, row in enumerate(selections):
        prompt = prompts[task_index]
        if set(row) != expected_fields or row["fallback_allowed"] is not False:
            raise RuntimeError(f"selection schema/fallback drifted: {row.get('task_id')}")
        if row["prompt_sha256"] != sha256_text(prompt["prompt"]):
            raise RuntimeError(f"selection prompt binding drifted: {row['task_id']}")
        winner = min(range(EXPECTED_ASSETS), key=lambda offset: (-float(similarities[task_index, offset]), index[offset]["candidate_key"]))
        if row["selected_embedding_row"] != winner or row["selected_candidate_key"] != index[winner]["candidate_key"]:
            raise RuntimeError(f"selection top-1 recomputation differs: {row['task_id']}")
        if row["clip_cosine_similarity"] != float(similarities[task_index, winner]):
            raise RuntimeError(f"selection score recomputation differs: {row['task_id']}")
        if row["tie_break"] != "descending exact float64 score, ascending opaque candidate key":
            raise RuntimeError(f"selection tie rule drifted: {row['task_id']}")
    return lock, selections


def verify_materialization(root: Path, selection_lock: dict[str, Any], selections: list[dict[str, Any]], bindings: dict[str, dict[str, Any]], addendum_sha: str, implementation: dict[str, str]) -> tuple[dict[str, Any], list[dict[str, Any]], int]:
    bundle = regular_dir(root / MATERIALIZATION_BUNDLE_NAME)
    expected_bundle = {
        "canonical", "materialized", "selected_metadata_audit.jsonl", "artifact_manifest.jsonl",
        "numeric_constraints_na.jsonl", "numeric_summary.json", "provenance.json",
        "result_report.md", "materialization.lock.json",
    }
    if {entry.name for entry in bundle.iterdir()} != expected_bundle:
        raise RuntimeError("materialization bundle exact closure drifted")
    for name in expected_bundle - {"canonical", "materialized"}:
        regular_file(bundle / name)
    lock = read_json(bundle / "materialization.lock.json")
    if lock.get("status") not in {"PASS", "PARTIAL"} or lock.get("tasks") != 18:
        raise RuntimeError("materialization lock status/task closure drifted")
    if lock.get("amendment_sha256") != addendum_sha or lock.get("implementation") != implementation:
        raise RuntimeError("materialization implementation/addendum binding drifted")
    if lock.get("selection_lock_sha256") != sha256_file(root / SELECTION_BUNDLE_NAME / "selection.lock.json"):
        raise RuntimeError("materialization/selection binding drifted")
    if lock.get("private_source_audit_lock_sha256") != sha256_file(PRIVATE_SOURCE / "source_audit.lock.json"):
        raise RuntimeError("materialization/private source binding drifted")
    if lock.get("materializer_sha256") != implementation["materializer_sha256"] or lock.get("canonicalizer_sha256") != implementation["canonicalizer_sha256"]:
        raise RuntimeError("materialization script binding drifted")
    if lock.get("repair_attempts") != 0 or lock.get("rank_fallbacks") != 0 or lock.get("numeric_constraints") != "N/A" or lock.get("count_proxy") != "N/R":
        raise RuntimeError("materialization repair/fallback/reporting policy drifted")
    locked_outputs = {
        "artifact_manifest.jsonl": "artifact_manifest_sha256",
        "numeric_constraints_na.jsonl": "numeric_constraints_na_sha256",
        "numeric_summary.json": "numeric_summary_sha256",
        "selected_metadata_audit.jsonl": "selected_metadata_audit_sha256",
        "provenance.json": "provenance_sha256",
        "result_report.md": "result_report_sha256",
    }
    for filename, field in locked_outputs.items():
        if sha256_file(bundle / filename) != lock.get(field):
            raise RuntimeError(f"materialization evidence drifted: {filename}")
    numeric_records = read_jsonl(bundle / "numeric_constraints_na.jsonl")
    numeric_summary = read_json(bundle / "numeric_summary.json")
    if len(numeric_records) != 20 or len({row.get("constraint_id") for row in numeric_records}) != 20:
        raise RuntimeError("numeric N/A record closure drifted")
    if any(row.get("status") != "N/A" or row.get("passed") is not None or row.get("evaluable") is not False for row in numeric_records):
        raise RuntimeError("numeric N/A row semantics drifted")
    if numeric_summary != {
        "schema_version": 1, "status": "N/A", "passed": None,
        "protocol_numeric_constraints": 20, "evaluable_numeric_constraints": 0,
        "not_applicable_numeric_constraints": 20, "headline": "N/A",
        "zero_over_twenty_is_forbidden": True,
    }:
        raise RuntimeError("numeric N/A summary semantics drifted")
    if lock.get("numeric_protocol_constraints") != 20 or lock.get("numeric_evaluable_constraints") != 0 or lock.get("numeric_passed") is not None:
        raise RuntimeError("materialization numeric denominator semantics drifted")
    manifest = read_jsonl(bundle / "artifact_manifest.jsonl")
    if len(manifest) != 18 or [row.get("task_id") for row in manifest] != EXPECTED_TASKS:
        raise RuntimeError("artifact manifest task closure drifted")
    selection_by_task = {row["task_id"]: row for row in selections}
    success = 0
    for row in manifest:
        task_id = row["task_id"]
        selected = selection_by_task[task_id]
        binding = bindings[selected["selected_candidate_key"]]
        if row.get("status") not in {"success", "failed"}:
            raise RuntimeError(f"unknown artifact status: {task_id}")
        if row.get("selected_candidate_key") != selected["selected_candidate_key"] or row.get("selected_identity") != binding["source_asset_id"]:
            raise RuntimeError(f"artifact selected source binding drifted: {task_id}")
        if row.get("fallback_used") is not False or row.get("repair_attempts") != 0:
            raise RuntimeError(f"artifact fallback/repair contract drifted: {task_id}")
        if not str(row.get("numeric_constraints", "")).startswith("N/A") or not str(row.get("count_proxy", "")).startswith("N/R"):
            raise RuntimeError(f"artifact unsupported score claim: {task_id}")
        if row["status"] != "success":
            continue
        success += 1
        if row.get("source_sha256") != binding["source_urdf_sha256"]:
            raise RuntimeError(f"artifact source hash binding drifted: {task_id}")
        glb = regular_file(Path(row["canonical_glb"]))
        if sha256_file(glb) != row["canonical_glb_sha256"]:
            raise RuntimeError(f"canonical GLB drifted: {task_id}")
        artifact_path = regular_file(Path(row["canonical_dir"]) / "artifact.json")
        artifact = read_json(artifact_path)
        metric_fields = {key for key in artifact if key == "unit_scale_to_m" or key.endswith("_m")}
        if metric_fields:
            raise RuntimeError(f"metric artifact field forbidden for nonmetric PhysX source: {task_id}/{sorted(metric_fields)}")
        if (
            artifact.get("coordinate_units") != "dataset_units_metric_binding_unestablished"
            or artifact.get("metric_eligible") is not False
            or artifact.get("numeric_constraint_status") != "N/A"
            or artifact.get("geometry_scale_applied") != 1.0
            or artifact.get("extents_dataset_units") != row.get("extents_dataset_units")
            or row.get("metric_eligible") is not False
        ):
            raise RuntimeError(f"nonmetric artifact declaration drifted: {task_id}")
        scene = trimesh.load(glb, force="scene", process=False)
        bounds = np.asarray(scene.bounds, dtype=np.float64)
        if bounds.shape != (2, 3) or not np.isfinite(bounds).all() or not (bounds[1] > bounds[0]).any():
            raise RuntimeError(f"invalid canonical AABB: {task_id}")
        semantic = regular_file(Path(row["semantic_nodes"]))
        if sha256_file(semantic) != row["semantic_nodes_sha256"]:
            raise RuntimeError(f"semantic nodes drifted: {task_id}")
        nodes = json.loads(semantic.read_text(encoding="utf-8"))
        if sum(node["level"] == "link" for node in nodes) != binding["mesh_bearing_links"]:
            raise RuntimeError(f"semantic link count drifted: {task_id}")
        if sum(node["level"] == "visual" for node in nodes) != binding["visual_count"]:
            raise RuntimeError(f"semantic visual count drifted: {task_id}")
    if success != lock.get("success") or len(manifest) - success != lock.get("failed"):
        raise RuntimeError("materialization success/failure accounting drifted")
    return lock, manifest, success


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    root = regular_dir(args.root)
    destination = safe(args.output, must_exist=False) if args.output else safe(root / "integrity.json", must_exist=False)
    if destination.exists():
        raise RuntimeError("integrity output already exists; verifier never overwrites")
    _, addendum_sha, implementation = validate_amendment()
    source_lock, bindings = verify_source(addendum_sha, implementation)
    snapshot_lock, inventory = verify_render_and_snapshot(source_lock, addendum_sha, implementation)
    selection_lock, selections = verify_selection(root, snapshot_lock, inventory, addendum_sha, implementation)
    material_lock, manifest, success = verify_materialization(root, selection_lock, selections, bindings, addendum_sha, implementation)
    result = {
        "schema_version": 1,
        "status": "PASS",
        "method": "physx_mobility_clip_retrieval_v1",
        "candidate_assets": EXPECTED_ASSETS,
        "fresh_full_render_runs": 2,
        "fresh_render_replay_byte_identical": True,
        "full_embedding_replays": 2,
        "embeddings_bitwise_identical": True,
        "selection_byte_identical": True,
        "artifact_rows": len(manifest),
        "artifact_success": success,
        "numeric_constraints": "N/A",
        "count_proxy": "N/R",
        "source_audit_lock_sha256": sha256_file(PRIVATE_SOURCE / "source_audit.lock.json"),
        "render_build_lock_sha256": sha256_file(RENDER_BUILD / "render_build.lock.json"),
        "snapshot_lock_sha256": sha256_file(SNAPSHOT / "snapshot.lock.json"),
        "selection_lock_sha256": sha256_file(root / SELECTION_BUNDLE_NAME / "selection.lock.json"),
        "selection_execution_contract_sha256": sha256_file(SELECTION_CONTRACT),
        "materialization_lock_sha256": sha256_file(root / MATERIALIZATION_BUNDLE_NAME / "materialization.lock.json"),
        "artifact_manifest_sha256": sha256_file(root / MATERIALIZATION_BUNDLE_NAME / "artifact_manifest.jsonl"),
        "integrity_verifier_sha256": sha256_file(SCRIPT),
        "amendment_sha256": addendum_sha,
    }
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(destination.suffix + ".tmp")
    temporary.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(destination)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
