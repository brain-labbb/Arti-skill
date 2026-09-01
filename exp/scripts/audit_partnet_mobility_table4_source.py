#!/usr/bin/env python3
"""Privately bind every PartNet-Mobility archive member to the extracted tree."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import shutil
import stat
import io
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path, PurePosixPath
from typing import Any

import numpy as np


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
WORKSPACE = REPO.parent.resolve()
REFERENCE = REPO / "exp/reference/table4_constraints_v2"
AMENDMENT = REFERENCE / "amendment_partnet_mobility_clip_retrieval_v1.json"
FORMAL_ROOT = REPO / "exp/runtime/table4_constraints_v2/partnet_mobility_clip_retrieval_v1"
DATA_PARENT = WORKSPACE / "PartNet_Mobility"
EXTRACTION_ROOT = DATA_PARENT / "data"
DATASET = EXTRACTION_ROOT / "dataset"
ARCHIVE = DATA_PARENT / "partnet-mobility-v0.zip"
DEFAULT_OUTPUT = REPO / "exp/runtime/table4_constraints_v2/partnet_mobility_clip_retrieval_v1_private_source"
EXPECTED_ARCHIVE_BYTES = 3268124298
EXPECTED_ARCHIVE_SHA256 = "b47247a44246111e8d09f2c0e64b4012ae35e0dcf4bb55f68a05b604455119ff"
EXPECTED_ASSETS = 2347
EXPECTED_CATEGORIES = 46
EXPECTED_ROOT_RPY = "1.570796326794897 0 -1.570796326794897"
SOURCE_AUDIT_PEER_FILES = {
    "render_worker_sha256": REPO / "exp/scripts/render_partnet_mobility_table4_opaque.py",
    "render_builder_sha256": REPO / "exp/scripts/build_partnet_mobility_table4_render_snapshot.py",
    "snapshot_locker_sha256": REPO / "exp/scripts/lock_partnet_mobility_table4_render_snapshot.py",
    "selection_launcher_sha256": REPO / "exp/scripts/launch_mobility_table4_clip_retrieval_selection.py",
    "selector_sha256": REPO / "exp/scripts/run_partnet_mobility_clip_retrieval_selection.py",
    "materializer_sha256": REPO / "exp/scripts/run_partnet_mobility_clip_retrieval_materialize.py",
    "integrity_verifier_sha256": REPO / "exp/scripts/verify_partnet_mobility_table4_retrieval.py",
    "canonicalizer_sha256": REPO / "exp/scripts/canonicalize_table4_artifact.py",
    "count_scorer_sha256": REPO / "exp/scripts/score_table4_constraints_v2.py",
    "generic_integrity_verifier_sha256": REPO / "exp/scripts/verify_table4_constraints_v2_integrity.py",
}


def safe(path: Path, *, must_exist: bool = True) -> Path:
    resolved = path.resolve(strict=must_exist)
    if resolved != WORKSPACE and WORKSPACE not in resolved.parents:
        raise RuntimeError(f"outside authorized workspace: {resolved}")
    return resolved


def sha256_stream(stream: Any, block_size: int = 8 * 1024 * 1024) -> tuple[str, int]:
    digest = hashlib.sha256()
    size = 0
    for block in iter(lambda: stream.read(block_size), b""):
        digest.update(block)
        size += len(block)
    return digest.hexdigest(), size


def sha256_file(path: Path) -> str:
    with safe(path).open("rb") as stream:
        return sha256_stream(stream)[0]


def sha256_text(value: str) -> str:
    return hashlib.sha256(value.encode("utf-8")).hexdigest()


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def parse_obj_numeric_with_stats(path: Path) -> tuple[np.ndarray, np.ndarray, dict[str, int]]:
    source = safe(path)
    vertices: list[tuple[float, float, float]] = []
    faces: list[tuple[int, int, int]] = []
    input_triangle_count = 0
    degenerate_index_triangle_count = 0
    with source.open("r", encoding="utf-8", errors="strict", newline=None) as stream:
        for line_number, raw in enumerate(stream, 1):
            stripped = raw.strip()
            if not stripped or stripped.startswith("#"):
                continue
            fields = stripped.split()
            if fields[0] == "v":
                if len(fields) not in (4, 5):
                    raise RuntimeError(f"unsupported OBJ vertex arity: {source}:{line_number}")
                values = tuple(float(value) for value in fields[1:])
                if not all(math.isfinite(value) for value in values):
                    raise RuntimeError(f"nonfinite OBJ vertex: {source}:{line_number}")
                xyz = values[:3]
                if len(values) == 4:
                    weight = values[3]
                    if weight == 0.0:
                        raise RuntimeError(f"zero OBJ homogeneous vertex weight: {source}:{line_number}")
                    xyz = tuple(value / weight for value in xyz)
                vertices.append(xyz)
            elif fields[0] == "f":
                if len(fields) != 4:
                    raise RuntimeError(f"non-triangular OBJ face: {source}:{line_number}")
                input_triangle_count += 1
                indices = []
                for token in fields[1:]:
                    head = token.split("/", 1)[0]
                    if not head:
                        raise RuntimeError(f"missing OBJ vertex index: {source}:{line_number}")
                    index = int(head)
                    if index == 0:
                        raise RuntimeError(f"zero OBJ vertex index: {source}:{line_number}")
                    resolved = index - 1 if index > 0 else len(vertices) + index
                    if resolved < 0 or resolved >= len(vertices):
                        raise RuntimeError(f"OBJ vertex index out of range: {source}:{line_number}")
                    indices.append(resolved)
                if len(set(indices)) != 3:
                    degenerate_index_triangle_count += 1
                    continue
                faces.append(tuple(indices))
            elif fields[0] in {"vn", "vt", "vp", "l", "p", "o", "g", "s", "mtllib", "usemtl"}:
                continue
            else:
                raise RuntimeError(f"unsupported OBJ directive: {source}:{line_number}:{fields[0]}")
    vertex_array = np.ascontiguousarray(vertices, dtype="<f8")
    face_array = np.ascontiguousarray(faces, dtype="<i8")
    if vertex_array.ndim != 2 or vertex_array.shape[1:] != (3,) or face_array.ndim != 2 or face_array.shape[1:] != (3,) or not len(face_array):
        raise RuntimeError(f"OBJ has no triangular numeric geometry: {source}")
    return vertex_array, face_array, {
        "input_triangle_count": input_triangle_count,
        "degenerate_index_triangle_count": degenerate_index_triangle_count,
        "retained_triangle_count": len(faces),
    }


def parse_obj_numeric(path: Path) -> tuple[np.ndarray, np.ndarray]:
    vertices, faces, _ = parse_obj_numeric_with_stats(path)
    return vertices, faces


def npy_bytes(array: np.ndarray) -> bytes:
    buffer = io.BytesIO()
    np.lib.format.write_array(buffer, np.ascontiguousarray(array), allow_pickle=False, version=(1, 0))
    return buffer.getvalue()


def deterministic_npz_bytes(vertices: np.ndarray, faces: np.ndarray) -> bytes:
    members = (("vertices.npy", npy_bytes(np.asarray(vertices, dtype="<f8"))), ("faces.npy", npy_bytes(np.asarray(faces, dtype="<i8"))))
    output = io.BytesIO()
    with zipfile.ZipFile(output, "w", compression=zipfile.ZIP_STORED, allowZip64=False) as bundle:
        for name, payload in members:
            info = zipfile.ZipInfo(name, date_time=(1980, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_STORED
            info.create_system = 3
            info.external_attr = 0o100644 << 16
            bundle.writestr(info, payload)
    return output.getvalue()


def write_geometry_npz(blob_root: Path, vertices: np.ndarray, faces: np.ndarray) -> tuple[Path, str, dict[str, Any]]:
    payload = deterministic_npz_bytes(vertices, faces)
    file_hash = sha256_bytes(payload)
    destination = safe(blob_root / f"{file_hash}.npz", must_exist=False)
    if not destination.exists():
        destination.write_bytes(payload)
    if destination.is_symlink() or not destination.is_file() or sha256_file(destination) != file_hash:
        raise RuntimeError("deterministic numeric geometry blob binding failed")
    return destination, file_hash, {
        "vertices_sha256": sha256_bytes(np.ascontiguousarray(vertices, dtype="<f8").tobytes(order="C")),
        "faces_sha256": sha256_bytes(np.ascontiguousarray(faces, dtype="<i8").tobytes(order="C")),
        "vertex_count": int(len(vertices)),
        "face_count": int(len(faces)),
    }


def atomic_text(path: Path, value: str) -> None:
    destination = safe(path, must_exist=False)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(destination.suffix + ".tmp")
    temporary.write_text(value, encoding="utf-8")
    temporary.replace(destination)


def write_json(path: Path, value: Any) -> None:
    atomic_text(path, json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True) + "\n")


def write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    atomic_text(path, "".join(json.dumps(row, sort_keys=True, ensure_ascii=True) + "\n" for row in rows))


def validate_pre_result_gate() -> dict[str, Any]:
    amendment = json.loads(safe(AMENDMENT).read_text(encoding="utf-8"))
    if amendment.get("status") != "pre_result_frozen":
        raise RuntimeError("amendment is not pre-result frozen")
    if amendment.get("implementation", {}).get("source_auditor_sha256") != sha256_file(SCRIPT):
        raise RuntimeError("amendment source-auditor binding drifted")
    for field, path in SOURCE_AUDIT_PEER_FILES.items():
        if amendment.get("implementation", {}).get(field) != sha256_file(path):
            raise RuntimeError(f"amendment implementation binding drifted: {field}")
    if not FORMAL_ROOT.is_dir() or {entry.name for entry in os.scandir(FORMAL_ROOT)} != {"protocol_audit_pre_result.json", "report.md"}:
        raise RuntimeError("formal root must contain exactly the independent pre-result audit and report")
    audit = json.loads(safe(FORMAL_ROOT / "protocol_audit_pre_result.json").read_text(encoding="utf-8"))
    if str(audit.get("verdict") or audit.get("status") or "").upper() != "PASS" or audit.get("protocol_ready") is not True:
        raise RuntimeError("independent pre-result audit is not ready PASS")
    required_audit = {"amendment_sha256": sha256_file(AMENDMENT), "source_auditor_sha256": sha256_file(SCRIPT)}
    required_audit.update({field: sha256_file(path) for field, path in SOURCE_AUDIT_PEER_FILES.items()})
    for field, expected in required_audit.items():
        if audit.get(field) != expected:
            raise RuntimeError(f"independent pre-result audit binding drifted: {field}")
    if audit.get("report_sha256") != sha256_file(FORMAL_ROOT / "report.md"):
        raise RuntimeError("independent pre-result report binding drifted")
    return {
        "amendment_sha256": sha256_file(AMENDMENT),
        "audit_sha256": sha256_file(FORMAL_ROOT / "protocol_audit_pre_result.json"),
        "implementation": amendment["implementation"],
    }


def validate_member(info: zipfile.ZipInfo) -> PurePosixPath:
    if "\\" in info.filename:
        raise RuntimeError(f"unsafe archive member backslash: {info.filename!r}")
    name = PurePosixPath(info.filename)
    expected_raw = name.as_posix().rstrip("/") + ("/" if info.is_dir() else "")
    if info.filename != expected_raw or name.is_absolute() or any(part in ("", ".", "..") for part in name.parts) or not name.parts or name.parts[0] != "dataset":
        raise RuntimeError(f"unsafe archive member: {info.filename!r}")
    mode = info.external_attr >> 16
    if stat.S_ISLNK(mode):
        raise RuntimeError(f"archive symlink forbidden: {info.filename}")
    return name


def archive_member_to_extracted(member: PurePosixPath) -> Path:
    if member.is_absolute() or ".." in member.parts or not member.parts or member.parts[0] != "dataset":
        raise RuntimeError(f"unsafe archive member path: {member}")
    return safe(EXTRACTION_ROOT / Path(*member.parts))


def extracted_archive_key(path: Path) -> str:
    extracted = safe(path)
    if extracted == EXTRACTION_ROOT or EXTRACTION_ROOT not in extracted.parents:
        raise RuntimeError(f"extracted path is outside extraction root: {extracted}")
    return extracted.relative_to(EXTRACTION_ROOT).as_posix()


def exact_regular_files(directory: Path) -> set[str]:
    root = safe(directory)
    if directory.is_symlink() or not stat.S_ISDIR(directory.lstat().st_mode):
        raise RuntimeError(f"geometry closure root is not a regular non-symlink directory: {directory}")
    observed: set[str] = set()
    for entry in os.scandir(root):
        path = Path(entry.path)
        if entry.is_symlink() or not entry.is_file(follow_symlinks=False):
            raise RuntimeError(f"geometry closure permits only flat regular non-symlink files: {path}")
        observed.add(path.relative_to(root).as_posix())
    return observed


def validate_private_geometry_closure(private_root: Path, render_plan: list[dict[str, Any]]) -> dict[str, int]:
    root = safe(private_root)
    scene_root = safe(root / "geometry_scenes")
    blob_root = safe(root / "geometry_blobs")
    expected_scenes: set[str] = set()
    expected_blobs: dict[str, str] = {}
    candidate_keys: set[str] = set()
    for row in render_plan:
        if set(row) != {"candidate_key", "geometry_scene", "geometry_scene_sha256"}:
            raise RuntimeError("render plan row schema drifted")
        candidate_key = row["candidate_key"]
        if not isinstance(candidate_key, str) or len(candidate_key) != 64 or any(char not in "0123456789abcdef" for char in candidate_key):
            raise RuntimeError("invalid opaque candidate key")
        if candidate_key in candidate_keys:
            raise RuntimeError(f"duplicate opaque candidate key: {candidate_key}")
        candidate_keys.add(candidate_key)
        relative_scene = Path(row["geometry_scene"])
        expected_relative = Path("geometry_scenes") / f"{candidate_key}.json"
        if relative_scene != expected_relative or relative_scene.is_absolute() or ".." in relative_scene.parts:
            raise RuntimeError(f"unsafe or noncanonical relative geometry scene: {relative_scene}")
        scene_name = relative_scene.name
        expected_scenes.add(scene_name)
        scene_path = safe(scene_root / scene_name)
        if scene_path.is_symlink() or not stat.S_ISREG(scene_path.lstat().st_mode):
            raise RuntimeError(f"geometry scene is not a regular non-symlink file: {scene_path}")
        if row["geometry_scene_sha256"] != sha256_file(scene_path):
            raise RuntimeError(f"geometry scene hash drifted: {scene_name}")
        scene = json.loads(scene_path.read_text(encoding="utf-8"))
        if set(scene) != {"schema_version", "camera_frame", "mesh_instances"} or scene.get("schema_version") != 2:
            raise RuntimeError(f"geometry scene schema drifted: {scene_name}")
        camera_frame = np.asarray(scene["camera_frame"], dtype=float)
        if camera_frame.shape != (3, 3) or not np.isfinite(camera_frame).all() or not np.allclose(camera_frame.T @ camera_frame, np.eye(3), atol=1e-12, rtol=0.0) or not np.isclose(np.linalg.det(camera_frame), 1.0, atol=1e-12, rtol=0.0):
            raise RuntimeError(f"camera frame is not a proper rotation: {scene_name}")
        instances = scene["mesh_instances"]
        if not isinstance(instances, list) or not instances:
            raise RuntimeError(f"geometry scene has no mesh instances: {scene_name}")
        for instance in instances:
            if set(instance) != {"mesh_blob", "mesh_sha256", "transform"}:
                raise RuntimeError(f"geometry instance schema drifted: {scene_name}")
            blob_relative = Path(instance["mesh_blob"])
            mesh_hash = instance["mesh_sha256"]
            if blob_relative.is_absolute() or ".." in blob_relative.parts or len(blob_relative.parts) != 2 or blob_relative.parts[0] != "geometry_blobs":
                raise RuntimeError(f"unsafe opaque geometry blob path: {blob_relative}")
            if not isinstance(mesh_hash, str) or len(mesh_hash) != 64 or any(char not in "0123456789abcdef" for char in mesh_hash) or blob_relative.stem != mesh_hash:
                raise RuntimeError(f"opaque geometry blob/hash identity drifted: {blob_relative}")
            if blob_relative.suffix != ".npz":
                raise RuntimeError(f"opaque geometry blob is not numeric NPZ: {blob_relative}")
            transform = np.asarray(instance["transform"], dtype=float)
            if transform.shape != (4, 4) or not np.array_equal(transform, np.eye(4)):
                raise RuntimeError(f"invalid geometry transform: {scene_name}")
            previous = expected_blobs.setdefault(blob_relative.name, mesh_hash)
            if previous != mesh_hash:
                raise RuntimeError(f"conflicting geometry blob hash: {blob_relative.name}")
    actual_scenes = exact_regular_files(scene_root)
    if actual_scenes != expected_scenes:
        raise RuntimeError(f"geometry scene exact closure mismatch missing={sorted(expected_scenes - actual_scenes)} extra={sorted(actual_scenes - expected_scenes)}")
    actual_blobs = exact_regular_files(blob_root)
    if actual_blobs != set(expected_blobs):
        raise RuntimeError(f"geometry blob exact closure mismatch missing={sorted(set(expected_blobs) - actual_blobs)} extra={sorted(actual_blobs - set(expected_blobs))}")
    for blob_name, mesh_hash in expected_blobs.items():
        blob_path = safe(blob_root / blob_name)
        if sha256_file(blob_path) != mesh_hash:
            raise RuntimeError(f"opaque geometry blob hash drifted: {blob_name}")
        with zipfile.ZipFile(blob_path) as bundle:
            infos = bundle.infolist()
            if [info.filename for info in infos] != ["vertices.npy", "faces.npy"] or any(info.compress_type != zipfile.ZIP_STORED for info in infos):
                raise RuntimeError(f"numeric geometry NPZ closure drifted: {blob_name}")
        with np.load(blob_path, allow_pickle=False) as payload:
            if payload.files != ["vertices", "faces"]:
                raise RuntimeError(f"numeric geometry array closure drifted: {blob_name}")
            vertices, faces = payload["vertices"], payload["faces"]
        if vertices.dtype != np.dtype("<f8") or faces.dtype != np.dtype("<i8") or vertices.ndim != 2 or vertices.shape[1:] != (3,) or faces.ndim != 2 or faces.shape[1:] != (3,):
            raise RuntimeError(f"numeric geometry dtype/shape drifted: {blob_name}")
        if not np.isfinite(vertices).all() or not len(faces) or np.any(faces < 0) or np.any(faces >= len(vertices)):
            raise RuntimeError(f"invalid numeric geometry arrays: {blob_name}")
    return {"geometry_scene_count": len(expected_scenes), "geometry_blob_count": len(expected_blobs)}


def validate_private_top_level(private_root: Path, expected: set[str]) -> None:
    root = safe(private_root)
    if root.is_symlink() or not stat.S_ISDIR(root.lstat().st_mode):
        raise RuntimeError("private source root is not a regular non-symlink directory")
    observed = {entry.name for entry in os.scandir(root)}
    if observed != expected:
        raise RuntimeError(f"private source top-level closure mismatch missing={sorted(expected - observed)} extra={sorted(observed - expected)}")


def floats(value: str | None, default: tuple[float, ...]) -> tuple[float, ...]:
    if not value:
        return default
    parsed = tuple(float(token) for token in value.split())
    if len(parsed) == 1 and len(default) == 3:
        return parsed * 3
    if len(parsed) != len(default):
        raise RuntimeError(f"bad numeric tuple: {value!r}")
    return parsed


def origin(node: ET.Element | None) -> np.ndarray:
    if node is None:
        return np.eye(4)
    xyz = floats(node.attrib.get("xyz"), (0.0, 0.0, 0.0))
    roll, pitch, yaw = floats(node.attrib.get("rpy"), (0.0, 0.0, 0.0))
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


def geometry_scene(asset: Path, archive_hashes: dict[str, str], blob_root: Path, candidate_key: str) -> tuple[dict[str, Any], dict[str, Any]]:
    root = ET.parse(asset / "mobility.urdf").getroot()
    base_joints = [joint for joint in root.findall("joint") if joint.find("parent") is not None and joint.find("parent").attrib.get("link") == "base"]
    if len(base_joints) != 1 or base_joints[0].attrib.get("type") != "fixed":
        raise RuntimeError("frozen PartNet root-joint closure drifted")
    base_origin = base_joints[0].find("origin")
    if base_origin is None or base_origin.attrib.get("rpy") != EXPECTED_ROOT_RPY or base_origin.attrib.get("xyz") != "0 0 0":
        raise RuntimeError("frozen PartNet root-frame transform drifted")
    links = {link.attrib["name"]: link for link in root.findall("link")}
    parents: dict[str, tuple[str, np.ndarray]] = {}
    for joint in root.findall("joint"):
        parent, child = joint.find("parent"), joint.find("child")
        if parent is None or child is None or child.attrib["link"] in parents:
            raise RuntimeError("invalid URDF tree")
        parents[child.attrib["link"]] = (parent.attrib["link"], origin(joint.find("origin")))
    pending, world = set(links), {}
    while pending:
        progressed = False
        for name in sorted(pending):
            if name in parents:
                parent, matrix = parents[name]
                if parent not in world:
                    continue
                world[name] = world[parent] @ matrix
            else:
                world[name] = np.eye(4)
            pending.remove(name)
            progressed = True
        if not progressed:
            raise RuntimeError("unresolved URDF graph")
    numeric_instances: list[dict[str, Any]] = []
    for name in sorted(links):
        for visual in links[name].findall("visual"):
            geometry = visual.find("geometry")
            shape = next(iter(geometry), None) if geometry is not None else None
            if shape is None or shape.tag != "mesh":
                raise RuntimeError("only mesh visuals are permitted")
            reference = Path(shape.attrib["filename"])
            if reference.is_absolute() or ".." in reference.parts:
                raise RuntimeError("unsafe mesh reference")
            mesh = safe(asset / reference)
            if asset not in mesh.parents or mesh.is_symlink() or not mesh.is_file():
                raise RuntimeError("mesh reference escaped or is not regular")
            relative = extracted_archive_key(mesh)
            if relative not in archive_hashes:
                raise RuntimeError("mesh is not bound to archive member manifest")
            scale = floats(shape.attrib.get("scale"), (1.0, 1.0, 1.0))
            transform = world[name] @ origin(visual.find("origin")) @ np.diag([*scale, 1.0])
            raw_hash = archive_hashes[relative]
            if sha256_file(mesh) != raw_hash:
                raise RuntimeError("archive-bound source OBJ hash drifted")
            vertices, faces, face_stats = parse_obj_numeric_with_stats(mesh)
            vertices_hash = sha256_bytes(vertices.tobytes(order="C"))
            faces_hash = sha256_bytes(faces.tobytes(order="C"))
            transform_array = np.ascontiguousarray(transform, dtype="<f8")
            transform_hash = sha256_bytes(transform_array.tobytes(order="C"))
            numeric_instances.append({
                "sort_key": (vertices_hash, faces_hash, transform_hash),
                "vertices": vertices,
                "faces": faces,
                "transform": transform_array,
                "private_source_member": relative,
                "private_source_obj_sha256": raw_hash,
                "vertices_sha256": vertices_hash,
                "faces_sha256": faces_hash,
                "transform_sha256": transform_hash,
                **face_stats,
            })
    if not numeric_instances:
        raise RuntimeError("geometry-only scene has no visual mesh instances")
    numeric_instances.sort(key=lambda item: item["sort_key"])
    merged_vertices: list[np.ndarray] = []
    merged_faces: list[np.ndarray] = []
    vertex_offset = 0
    private_sources = []
    for item in numeric_instances:
        vertices = item["vertices"]
        homogeneous = np.column_stack((vertices, np.ones(len(vertices), dtype="<f8")))
        transformed = np.ascontiguousarray((homogeneous @ item["transform"].T)[:, :3], dtype="<f8")
        faces = np.ascontiguousarray(item["faces"] + vertex_offset, dtype="<i8")
        merged_vertices.append(transformed)
        merged_faces.append(faces)
        vertex_offset += len(transformed)
        private_sources.append({
            "source_member": item["private_source_member"],
            "source_obj_sha256": item["private_source_obj_sha256"],
            "vertices_sha256": item["vertices_sha256"],
            "faces_sha256": item["faces_sha256"],
            "transform_sha256": item["transform_sha256"],
            "input_triangle_count": item["input_triangle_count"],
            "degenerate_index_triangle_count": item["degenerate_index_triangle_count"],
            "retained_triangle_count": item["retained_triangle_count"],
        })
    vertices = np.ascontiguousarray(np.concatenate(merged_vertices, axis=0), dtype="<f8")
    faces = np.ascontiguousarray(np.concatenate(merged_faces, axis=0), dtype="<i8")
    blob, blob_hash, numeric_summary = write_geometry_npz(blob_root, vertices, faces)
    blob_relative = blob.relative_to(blob_root.parent).as_posix()
    scene = {
        "schema_version": 2,
        "camera_frame": np.eye(3).tolist(),
        "mesh_instances": [{
            "mesh_blob": blob_relative,
            "mesh_sha256": blob_hash,
            "transform": np.eye(4).tolist(),
        }],
    }
    derivation = {
        "candidate_key": candidate_key,
        "source_instance_count": len(numeric_instances),
        "numeric_sort_policy": "ascending vertices_sha256, faces_sha256, transform_sha256 only; source path/raw hash/labels are provenance-only and never sorting inputs",
        "private_sources_in_numeric_order": private_sources,
        "derived_geometry_blob": blob_relative,
        "derived_geometry_blob_sha256": blob_hash,
        "input_triangle_count": sum(item["input_triangle_count"] for item in numeric_instances),
        "degenerate_index_triangle_count": sum(item["degenerate_index_triangle_count"] for item in numeric_instances),
        "retained_triangle_count": sum(item["retained_triangle_count"] for item in numeric_instances),
        **numeric_summary,
    }
    return scene, derivation


def run(output: Path) -> None:
    gate = validate_pre_result_gate()
    output = safe(output, must_exist=False)
    if output.exists():
        raise RuntimeError(f"output exists: {output}")
    staging = safe(output.with_name(output.name + ".staging"), must_exist=False)
    if staging.exists():
        raise RuntimeError(f"staging exists: {staging}")
    staging.mkdir(parents=True)
    try:
        archive = safe(ARCHIVE)
        if archive.is_symlink() or not archive.is_file():
            raise RuntimeError("archive must be a regular non-symlink file")
        if archive.stat().st_size != EXPECTED_ARCHIVE_BYTES or sha256_file(archive) != EXPECTED_ARCHIVE_SHA256:
            raise RuntimeError("archive identity mismatch")
        archive_rows: list[dict[str, Any]] = []
        archive_hashes: dict[str, str] = {}
        archive_names: set[str] = set()
        archive_ids: set[str] = set()
        with zipfile.ZipFile(archive) as bundle:
            for info in bundle.infolist():
                member = validate_member(info)
                normalized = member.as_posix().rstrip("/") + ("/" if info.is_dir() else "")
                if normalized in archive_names:
                    raise RuntimeError(f"duplicate normalized archive member: {normalized}")
                archive_names.add(normalized)
                if len(member.parts) >= 2:
                    archive_ids.add(member.parts[1])
                if info.is_dir():
                    continue
                extracted = archive_member_to_extracted(member)
                if extracted.is_symlink() or not extracted.is_file():
                    raise RuntimeError(f"missing regular extracted member: {member}")
                with bundle.open(info, "r") as source, extracted.open("rb") as target:
                    source_hash, source_size = sha256_stream(source)
                    target_hash, target_size = sha256_stream(target)
                if source_size != info.file_size or target_size != info.file_size or source_hash != target_hash:
                    raise RuntimeError(f"archive/extracted byte mismatch: {member}")
                archive_rows.append({
                    "relative_path": member.as_posix(),
                    "bytes": info.file_size,
                    "crc32": f"{info.CRC:08x}",
                    "sha256": source_hash,
                })
                archive_hashes[member.as_posix()] = source_hash
                if len(archive_rows) % 10000 == 0:
                    print(f"[source-audit] {len(archive_rows)} members", flush=True)
        actual_names: set[str] = set()
        actual_ids: set[str] = set()
        for directory, directory_names, file_names in os.walk(safe(DATASET), followlinks=False):
            current = Path(directory)
            for name in list(directory_names):
                path = current / name
                if path.is_symlink():
                    raise RuntimeError(f"extracted symlink forbidden: {path}")
                relative = path.relative_to(EXTRACTION_ROOT).as_posix() + "/"
                actual_names.add(relative)
            for name in file_names:
                path = current / name
                if path.is_symlink() or not path.is_file():
                    raise RuntimeError(f"extracted non-regular file forbidden: {path}")
                actual_names.add(path.relative_to(EXTRACTION_ROOT).as_posix())
            relative_parts = current.relative_to(DATASET).parts
            if len(relative_parts) == 1:
                actual_ids.add(relative_parts[0])
        if archive_names != actual_names | {"dataset/"}:
            raise RuntimeError(
                f"archive/extracted path closure mismatch missing={len(archive_names - actual_names - {'dataset/'})} "
                f"extra={len(actual_names - archive_names)}"
            )
        if archive_ids != actual_ids or len(actual_ids) != EXPECTED_ASSETS:
            raise RuntimeError("asset identity closure mismatch")
        categories: set[str] = set()
        render_plan: list[dict[str, Any]] = []
        source_binding: list[dict[str, Any]] = []
        geometry_derivations: list[dict[str, Any]] = []
        geometry_root = staging / "geometry_scenes"
        geometry_root.mkdir()
        blob_root = staging / "geometry_blobs"
        blob_root.mkdir()
        for asset_id in sorted(actual_ids):
            asset = safe(DATASET / asset_id)
            required = [asset / "mobility.urdf", asset / "meta.json", asset / "textured_objs"]
            if not all(path.exists() and not path.is_symlink() for path in required):
                raise RuntimeError(f"required package component missing: {asset_id}")
            meta = json.loads((asset / "meta.json").read_text(encoding="utf-8"))
            categories.add(meta["model_cat"])
            candidate_key = sha256_text(f"partnet-mobility-v0:{asset_id}")
            scene_path = geometry_root / f"{candidate_key}.json"
            scene, derivation = geometry_scene(asset, archive_hashes, blob_root, candidate_key)
            write_json(scene_path, scene)
            geometry_derivations.append(derivation)
            render_plan.append({
                "candidate_key": candidate_key,
                "geometry_scene": scene_path.relative_to(staging).as_posix(),
                "geometry_scene_sha256": sha256_file(scene_path),
            })
            source_binding.append({
                "candidate_key": candidate_key,
                "source_asset_id": asset_id,
                "source_urdf": str(asset / "mobility.urdf"),
                "source_urdf_sha256": sha256_file(asset / "mobility.urdf"),
            })
        for row in render_plan:
            relative_scene = Path(row["geometry_scene"])
            if relative_scene.is_absolute() or ".." in relative_scene.parts or relative_scene.parts[:1] != ("geometry_scenes",):
                raise RuntimeError(f"unsafe relative geometry scene: {relative_scene}")
        if len(categories) != EXPECTED_CATEGORIES:
            raise RuntimeError(f"category closure drifted: {len(categories)}")
        render_plan.sort(key=lambda row: row["candidate_key"])
        source_binding.sort(key=lambda row: row["candidate_key"])
        geometry_derivations.sort(key=lambda row: row["candidate_key"])
        write_jsonl(staging / "archive_member_manifest.jsonl", archive_rows)
        write_jsonl(staging / "geometry_derivation_manifest.jsonl", geometry_derivations)
        write_jsonl(staging / "render_plan.jsonl", render_plan)
        write_jsonl(staging / "source_binding.jsonl", source_binding)
        expected_before_lock = {
            "archive_member_manifest.jsonl",
            "geometry_blobs",
            "geometry_derivation_manifest.jsonl",
            "geometry_scenes",
            "render_plan.jsonl",
            "source_binding.jsonl",
        }
        validate_private_top_level(staging, expected_before_lock)
        geometry_closure = validate_private_geometry_closure(staging, render_plan)
        lock = {
            "schema_version": 1,
            "status": "PASS",
            "private": True,
            "archive_sha256": EXPECTED_ARCHIVE_SHA256,
            "archive_bytes": EXPECTED_ARCHIVE_BYTES,
            "archive_regular_file_count": len(archive_rows),
            "archive_uncompressed_bytes": sum(row["bytes"] for row in archive_rows),
            "archive_member_manifest_sha256": sha256_file(staging / "archive_member_manifest.jsonl"),
            "geometry_derivation_manifest_sha256": sha256_file(staging / "geometry_derivation_manifest.jsonl"),
            "asset_count": len(render_plan),
            "category_count_source_audit_only": len(categories),
            "render_plan_sha256": sha256_file(staging / "render_plan.jsonl"),
            "source_binding_sha256": sha256_file(staging / "source_binding.jsonl"),
            "geometry_scene_count": geometry_closure["geometry_scene_count"],
            "geometry_blob_count": geometry_closure["geometry_blob_count"],
            "private_geometry_exact_closure": True,
            "partnet_root_frame_gate": {"assets_checked": len(render_plan), "root_parent": "base", "joint_type": "fixed", "rpy": EXPECTED_ROOT_RPY, "xyz": "0 0 0", "all_exact": True},
            "geometry_scene_policy": "q0 world transforms include the frozen URDF root fixed transform for all assets; private source phase parses only OBJ v/f numeric directives and ignores all labels/materials/comments; instances sort only by numeric vertices/faces/transform hashes and are merged into one deterministic label-free NPZ; the separate source-to-canonical camera rotation is identity and never modifies score geometry",
            "source_auditor_sha256": sha256_file(SCRIPT),
            "amendment_sha256": gate["amendment_sha256"],
            "pre_result_audit_sha256": gate["audit_sha256"],
            "closure": "every regular ZIP member was stream-SHA256 compared to the corresponding extracted regular file; exact path set was checked",
        }
        write_json(staging / "source_audit.lock.json", lock)
        validate_private_top_level(staging, expected_before_lock | {"source_audit.lock.json"})
        validate_private_geometry_closure(staging, render_plan)
        staging.replace(output)
        print(json.dumps({"status": "SOURCE_AUDIT_LOCKED", "assets": len(render_plan), "members": len(archive_rows), "lock_sha256": sha256_file(output / "source_audit.lock.json")}, indent=2), flush=True)
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    run(parser.parse_args().output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
