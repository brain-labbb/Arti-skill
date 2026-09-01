#!/usr/bin/env python3
"""Privately bind PhysX-Mobility source geometry for the shared Table 4 renderer."""

from __future__ import annotations

import argparse
import hashlib
import io
import json
import math
import os
import shutil
import stat
import zipfile
import xml.etree.ElementTree as ET
from pathlib import Path, PurePosixPath
from typing import Any, BinaryIO

import numpy as np


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
WORKSPACE = Path("/mnt/zsn/lyb").resolve()
EXTERNAL_WORKSPACE = Path("/mnt/zsn/zsn_workspace").resolve()
REFERENCE = REPO / "exp/reference/table4_constraints_v2"
ADDENDUM = REFERENCE / "amendment_physx_mobility_clip_retrieval_v1_addendum1.json"
FORMAL_ROOT = REPO / "exp/runtime/table4_constraints_v2/physx_mobility_clip_retrieval_v1"
DATASET = Path("/mnt/zsn/zsn_workspace/Ctrl-3D-trellis2-controlnet-dev/demo/physical_edit_demo/third_party/physx_mobility/extracted/PhysX_mobility")
ARCHIVE = DATASET.parents[1] / "PhysX-Mobility.zip"
HF_METADATA = DATASET.parents[1] / ".cache/huggingface/download/PhysX-Mobility.zip.metadata"
DEFAULT_OUTPUT = REPO / "exp/runtime/table4_constraints_v2/physx_mobility_clip_retrieval_v1_private_source"
RENDER_WORKER = REPO / "exp/scripts/render_partnet_mobility_table4_opaque.py"
RENDER_BUILDER = REPO / "exp/scripts/build_partnet_mobility_table4_render_snapshot.py"
SNAPSHOT_LOCKER = REPO / "exp/scripts/lock_partnet_mobility_table4_render_snapshot.py"
SELECTION_LAUNCHER = REPO / "exp/scripts/launch_mobility_table4_clip_retrieval_selection.py"
SELECTOR = REPO / "exp/scripts/run_partnet_mobility_clip_retrieval_selection.py"
MATERIALIZER = REPO / "exp/scripts/run_physx_mobility_clip_retrieval_materialize.py"
VERIFIER = REPO / "exp/scripts/verify_physx_mobility_clip_retrieval_v1.py"
CANONICALIZER = REPO / "exp/scripts/canonicalize_table4_artifact.py"

EXPECTED_ARCHIVE_BYTES = 937_374_668
EXPECTED_ARCHIVE_SHA256 = "88308cc2a4cc6177c59e32c2de51e881e6b961737295e5082d7ed01cca221908"
EXPECTED_HF_REVISION = "d0768ee9e1415f6be8db78d6389ba018b85134c0"
EXPECTED_HF_METADATA_SHA256 = "2e9fe98c3a42737ba25d6791d39fc6aa7201cc595ea761b6023d99645479f439"
EXPECTED_ASSETS = 2024
EXPECTED_MESH_BEARING_LINKS = 14_096
EXPECTED_VISUALS = 91_855
CAMERA_FRAME_SOURCE_TO_CANONICAL = np.array([
    [1.0, 0.0, 0.0],
    [0.0, 0.0, -1.0],
    [0.0, 1.0, 0.0],
])


def safe_read(path: Path, *, must_exist: bool = True) -> Path:
    resolved = path.resolve(strict=must_exist)
    if not any(resolved == root or root in resolved.parents for root in (WORKSPACE, EXTERNAL_WORKSPACE)):
        raise RuntimeError(f"read path outside authorized roots: {resolved}")
    return resolved


def safe_write(path: Path) -> Path:
    resolved = path.resolve(strict=False)
    if resolved != WORKSPACE and WORKSPACE not in resolved.parents:
        raise RuntimeError(f"write path outside primary workspace: {resolved}")
    return resolved


def regular_file(path: Path) -> Path:
    resolved = safe_read(path)
    if path.is_symlink() or not stat.S_ISREG(path.lstat().st_mode):
        raise RuntimeError(f"not a regular non-symlink file: {path}")
    return resolved


def regular_dir(path: Path) -> Path:
    resolved = safe_read(path)
    if path.is_symlink() or not stat.S_ISDIR(path.lstat().st_mode):
        raise RuntimeError(f"not a regular non-symlink directory: {path}")
    return resolved


def sha256_stream(stream: BinaryIO, block_size: int = 8 * 1024 * 1024) -> tuple[str, int]:
    digest = hashlib.sha256()
    size = 0
    for block in iter(lambda: stream.read(block_size), b""):
        digest.update(block)
        size += len(block)
    return digest.hexdigest(), size


def sha256_file(path: Path) -> str:
    with regular_file(path).open("rb") as stream:
        return sha256_stream(stream)[0]


def sha256_text(value: str) -> str:
    return hashlib.sha256(value.encode("utf-8")).hexdigest()


def atomic_text(path: Path, value: str) -> None:
    destination = safe_write(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(destination.suffix + ".tmp")
    temporary.write_text(value, encoding="utf-8")
    temporary.replace(destination)


def write_json(path: Path, value: Any) -> None:
    atomic_text(path, json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True) + "\n")


def write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    atomic_text(path, "".join(json.dumps(row, sort_keys=True, ensure_ascii=True) + "\n" for row in rows))


def implementation_hashes() -> dict[str, str]:
    paths = {
        "source_auditor_sha256": SCRIPT,
        "render_worker_sha256": RENDER_WORKER,
        "render_builder_sha256": RENDER_BUILDER,
        "snapshot_locker_sha256": SNAPSHOT_LOCKER,
        "selection_launcher_sha256": SELECTION_LAUNCHER,
        "selector_sha256": SELECTOR,
        "materializer_sha256": MATERIALIZER,
        "integrity_verifier_sha256": VERIFIER,
        "canonicalizer_sha256": CANONICALIZER,
    }
    return {field: sha256_file(path) for field, path in paths.items()}


def validate_pre_result_gate() -> dict[str, Any]:
    amendment_path = regular_file(ADDENDUM)
    amendment = json.loads(amendment_path.read_text(encoding="utf-8"))
    observed = implementation_hashes()
    if amendment.get("status") != "pre_result_frozen" or amendment.get("implementation") != observed:
        raise RuntimeError("PhysX addendum implementation binding drifted")
    output = regular_dir(FORMAL_ROOT)
    if {entry.name for entry in os.scandir(output)} != {"protocol_audit_pre_result.json", "report.md"}:
        raise RuntimeError("formal root must contain exactly independent pre-result audit and report")
    audit_path = regular_file(output / "protocol_audit_pre_result.json")
    report_path = regular_file(output / "report.md")
    audit = json.loads(audit_path.read_text(encoding="utf-8"))
    verdict = str(audit.get("verdict") or audit.get("status") or "").upper()
    if verdict != "PASS" or audit.get("protocol_ready") is not True:
        raise RuntimeError("independent pre-result audit is not ready PASS")
    required = {
        "amendment_sha256": sha256_file(amendment_path),
        "report_sha256": sha256_file(report_path),
        **observed,
    }
    for field, expected in required.items():
        if audit.get(field) != expected:
            raise RuntimeError(f"independent pre-result audit binding drifted: {field}")
    return {
        "amendment_sha256": required["amendment_sha256"],
        "audit_sha256": sha256_file(audit_path),
        "implementation": observed,
    }


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
    if normalized.is_absolute() or any(part in {"", ".", ".."} for part in normalized.parts) or expected_text != relative:
        raise RuntimeError(f"unsafe or non-normalized archive path: {filename!r}")
    mode = (info.external_attr >> 16) & 0o170000
    if mode == stat.S_IFLNK:
        raise RuntimeError(f"archive symlink forbidden: {filename}")
    return normalized.as_posix()


def archive_extraction_closure(archive: Path, dataset: Path, manifest: Path) -> tuple[dict[str, str], dict[str, Any]]:
    archive = regular_file(archive)
    dataset = regular_dir(dataset)
    if archive.stat().st_size != EXPECTED_ARCHIVE_BYTES or sha256_file(archive) != EXPECTED_ARCHIVE_SHA256:
        raise RuntimeError("PhysX-Mobility archive identity mismatch")
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
    rows: list[dict[str, Any]] = []
    hashes: dict[str, str] = {}
    with zipfile.ZipFile(archive) as bundle:
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
            missing = sorted(set(members) - actual)
            extra = sorted(actual - set(members))
            raise RuntimeError(f"archive/extraction file closure mismatch: missing={missing[:5]} extra={extra[:5]}")
        for index, relative in enumerate(sorted(members), 1):
            info = members[relative]
            extracted = regular_file(dataset / relative)
            if extracted.stat().st_size != info.file_size:
                raise RuntimeError(f"archive/extraction size mismatch: {relative}")
            with bundle.open(info, "r") as source_stream, extracted.open("rb") as extracted_stream:
                source_hash, source_size = sha256_stream(source_stream)
                extracted_hash, extracted_size = sha256_stream(extracted_stream)
            if source_size != info.file_size or extracted_size != info.file_size or source_hash != extracted_hash:
                raise RuntimeError(f"archive/extraction byte mismatch: {relative}")
            rows.append({
                "relative_path": relative,
                "bytes": info.file_size,
                "crc32": f"{info.CRC:08x}",
                "sha256": source_hash,
            })
            hashes[relative] = source_hash
            if index % 10000 == 0:
                print(f"[source-archive] {index}/{len(members)}", flush=True)
    write_jsonl(manifest, rows)
    return hashes, {
        "archive_regular_file_count": len(rows),
        "archive_uncompressed_bytes": sum(row["bytes"] for row in rows),
        "archive_member_manifest_sha256": sha256_file(manifest),
    }


def vector(text: str | None, default: tuple[float, float, float]) -> tuple[float, float, float]:
    values = default if text is None else tuple(float(token) for token in text.split())
    if len(values) == 1:
        values = values * 3
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise RuntimeError(f"invalid URDF vector: {text!r}")
    return values


def origin(node: ET.Element | None) -> np.ndarray:
    if node is None:
        return np.eye(4)
    xyz = vector(node.attrib.get("xyz"), (0.0, 0.0, 0.0))
    roll, pitch, yaw = vector(node.attrib.get("rpy"), (0.0, 0.0, 0.0))
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


def parse_obj_numeric(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Parse only OBJ vertex and face numbers; all labels/material records are ignored."""
    vertices: list[tuple[float, float, float]] = []
    faces: list[tuple[int, int, int]] = []
    with regular_file(path).open("r", encoding="utf-8", errors="strict", newline=None) as stream:
        for line_number, line in enumerate(stream, 1):
            stripped = line.lstrip()
            if not stripped or stripped.startswith("#"):
                continue
            fields = stripped.split()
            if not fields:
                continue
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
    if vertex_array.ndim != 2 or vertex_array.shape[1:] != (3,) or face_array.ndim != 2 or face_array.shape[1:] != (3,):
        raise RuntimeError(f"invalid numeric OBJ array shapes: {path}/{vertex_array.shape}/{face_array.shape}")
    if len(vertex_array) == 0 or len(face_array) == 0 or not np.isfinite(vertex_array).all():
        raise RuntimeError(f"empty or invalid numeric OBJ geometry: {path}")
    return vertex_array, face_array


def npy_bytes(array: np.ndarray) -> bytes:
    buffer = io.BytesIO()
    np.lib.format.write_array(buffer, np.ascontiguousarray(array), version=(1, 0), allow_pickle=False)
    return buffer.getvalue()


def deterministic_npz_bytes(vertices: np.ndarray, faces: np.ndarray) -> bytes:
    entries = {
        "vertices.npy": npy_bytes(np.ascontiguousarray(vertices, dtype="<f8")),
        "faces.npy": npy_bytes(np.ascontiguousarray(faces, dtype="<i8")),
    }
    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, "w", compression=zipfile.ZIP_STORED, strict_timestamps=True) as bundle:
        for name in ("vertices.npy", "faces.npy"):
            info = zipfile.ZipInfo(name, date_time=(1980, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_STORED
            info.create_system = 3
            info.external_attr = (stat.S_IFREG | 0o644) << 16
            bundle.writestr(info, entries[name])
    return buffer.getvalue()


def derive_candidate_geometry(
    candidate_key: str,
    source_instances: list[dict[str, Any]],
    blob_root: Path,
) -> tuple[dict[str, Any], dict[str, Any]]:
    prepared: list[dict[str, Any]] = []
    cache: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    for item in source_instances:
        cache_key = item["source_obj_sha256"]
        if cache_key not in cache:
            cache[cache_key] = parse_obj_numeric(Path(item["source_path"]))
        vertices, faces = cache[cache_key]
        transform = np.ascontiguousarray(item["q0_transform"], dtype="<f8")
        if transform.shape != (4, 4) or not np.isfinite(transform).all():
            raise RuntimeError(f"invalid q0 transform for numeric derivation: {candidate_key}")
        vertex_bytes = np.ascontiguousarray(vertices, dtype="<f8").tobytes(order="C")
        face_bytes = np.ascontiguousarray(faces, dtype="<i8").tobytes(order="C")
        transform_bytes = transform.tobytes(order="C")
        transform_sha256 = hashlib.sha256(transform_bytes).hexdigest()
        if item["q0_transform_sha256"] != transform_sha256:
            raise RuntimeError(f"q0 transform hash drifted before numeric derivation: {candidate_key}")
        prepared.append({
            "item": item,
            "vertices": vertices,
            "faces": faces,
            "transform": transform,
            "numeric_vertices_sha256": hashlib.sha256(vertex_bytes).hexdigest(),
            "numeric_faces_sha256": hashlib.sha256(face_bytes).hexdigest(),
            "numeric_instance_order_sha256": hashlib.sha256(
                vertex_bytes + face_bytes + transform_bytes
            ).hexdigest(),
            "numeric_instance_bytes": vertex_bytes + face_bytes + transform_bytes,
        })
    prepared.sort(key=lambda entry: entry["numeric_instance_order_sha256"])
    for previous, current in zip(prepared, prepared[1:]):
        if (
            previous["numeric_instance_order_sha256"]
            == current["numeric_instance_order_sha256"]
            and previous["numeric_instance_bytes"] != current["numeric_instance_bytes"]
        ):
            raise RuntimeError(f"numeric instance ordering hash collision: {candidate_key}")
    all_vertices: list[np.ndarray] = []
    all_faces: list[np.ndarray] = []
    vertex_offset = 0
    derivation_instances: list[dict[str, Any]] = []
    for entry in prepared:
        item = entry["item"]
        vertices = entry["vertices"]
        faces = entry["faces"]
        transform = entry["transform"]
        homogeneous = np.column_stack((vertices, np.ones(len(vertices), dtype="<f8")))
        transformed = np.ascontiguousarray((homogeneous @ transform.T)[:, :3], dtype="<f8")
        all_vertices.append(transformed)
        all_faces.append(np.ascontiguousarray(faces + vertex_offset, dtype="<i8"))
        derivation_instances.append({
            "source_member_path": item["source_member_path"],
            "source_member_path_sha256": item["source_member_path_sha256"],
            "source_obj_sha256": item["source_obj_sha256"],
            "q0_transform_sha256": item["q0_transform_sha256"],
            "numeric_vertices_sha256": entry["numeric_vertices_sha256"],
            "numeric_faces_sha256": entry["numeric_faces_sha256"],
            "numeric_instance_order_sha256": entry["numeric_instance_order_sha256"],
            "source_vertex_count": int(len(vertices)),
            "source_face_count": int(len(faces)),
        })
        vertex_offset += len(vertices)
    if not all_vertices or not all_faces:
        raise RuntimeError(f"no numeric geometry instances: {candidate_key}")
    combined_vertices = np.ascontiguousarray(np.concatenate(all_vertices, axis=0), dtype="<f8")
    combined_faces = np.ascontiguousarray(np.concatenate(all_faces, axis=0), dtype="<i8")
    if combined_faces.min() < 0 or combined_faces.max() >= len(combined_vertices):
        raise RuntimeError(f"derived face index closure drifted: {candidate_key}")
    payload = deterministic_npz_bytes(combined_vertices, combined_faces)
    blob_sha256 = hashlib.sha256(payload).hexdigest()
    blob = blob_root / f"{blob_sha256}.npz"
    if blob.exists():
        if blob.is_symlink() or not stat.S_ISREG(blob.lstat().st_mode) or blob.read_bytes() != payload:
            raise RuntimeError(f"derived numeric blob collision: {blob}")
    else:
        blob.write_bytes(payload)
    identity = np.eye(4, dtype="<f8")
    scene_instance = {
        "mesh_blob": f"geometry_blobs/{blob.name}",
        "mesh_sha256": blob_sha256,
        "transform": identity.tolist(),
    }
    derivation = {
        "schema_version": 1,
        "candidate_key": candidate_key,
        "ordered_source_instances": derivation_instances,
        "derived_blob": scene_instance["mesh_blob"],
        "derived_blob_sha256": blob_sha256,
        "vertices_dtype": "<f8",
        "faces_dtype": "<i8",
        "vertex_count": int(len(combined_vertices)),
        "face_count": int(len(combined_faces)),
        "vertices_content_sha256": hashlib.sha256(combined_vertices.tobytes(order="C")).hexdigest(),
        "faces_content_sha256": hashlib.sha256(combined_faces.tobytes(order="C")).hexdigest(),
        "npz_entries": ["vertices.npy", "faces.npy"],
        "npz_compression": "ZIP_STORED",
    }
    return scene_instance, derivation


def geometry_scene(
    dataset: Path,
    identity: str,
    archive_hashes: dict[str, str],
    private_root: Path,
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    urdf = regular_file(dataset / "urdf" / f"{identity}.urdf")
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
            parent is None or child is None or parent_name not in links or child_name not in links
            or parent_name == child_name or child_name in parents
        ):
            raise RuntimeError(f"invalid URDF tree: {identity}")
        parents[child_name] = (parent_name, origin(joint.find("origin")))
    root_links = set(links) - set(parents)
    if len(root_links) != 1:
        raise RuntimeError(f"URDF must have exactly one root link: {identity}/{sorted(root_links)}")
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
    if len(world) != len(links):
        raise RuntimeError(f"URDF world-transform closure drifted: {identity}")
    source_instances: list[dict[str, Any]] = []
    mesh_links = 0
    visuals = 0
    for name in sorted(links):
        link_visuals = links[name].findall("visual")
        if link_visuals:
            mesh_links += 1
            if not np.array_equal(world[name], np.eye(4)):
                error = float(np.max(np.abs(world[name] - np.eye(4))))
                raise RuntimeError(f"non-identity mesh-link q0 world transform: {identity}/{name} error={error}")
        for visual in link_visuals:
            visuals += 1
            visual_origin = origin(visual.find("origin"))
            if not np.array_equal(visual_origin, np.eye(4)):
                raise RuntimeError(f"non-identity visual origin: {identity}/{name}")
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                raise RuntimeError(f"non-mesh visual: {identity}/{name}")
            scale = vector(mesh.attrib.get("scale"), (1.0, 1.0, 1.0))
            if scale != (1.0, 1.0, 1.0):
                raise RuntimeError(f"non-unit mesh scale: {identity}/{name}")
            reference = Path(mesh.attrib.get("filename", ""))
            if reference.is_absolute() or ".." not in reference.parts:
                raise RuntimeError(f"unexpected PhysX visual reference: {identity}/{reference}")
            mesh_path = safe_read((urdf.parent / reference).resolve())
            object_root = regular_dir(dataset / "partseg" / identity / "objs")
            if mesh_path.suffix.lower() != ".obj" or object_root not in mesh_path.parents:
                raise RuntimeError(f"visual reference escaped identity OBJ root: {identity}/{reference}")
            relative = mesh_path.relative_to(dataset).as_posix()
            mesh_hash = archive_hashes.get(relative)
            if mesh_hash is None or sha256_file(mesh_path) != mesh_hash:
                raise RuntimeError(f"visual OBJ is not archive-bound: {identity}/{reference}")
            transform = np.ascontiguousarray(world[name] @ visual_origin @ np.diag([*scale, 1.0]), dtype="<f8")
            source_instances.append({
                "source_path": str(mesh_path),
                "source_member_path": relative,
                "source_member_path_sha256": sha256_text(relative),
                "source_obj_sha256": mesh_hash,
                "q0_transform": transform.tolist(),
                "q0_transform_sha256": hashlib.sha256(transform.tobytes(order="C")).hexdigest(),
            })
    if not source_instances:
        raise RuntimeError(f"no renderable visuals: {identity}")
    candidate_key = sha256_text(f"physx-mobility-{EXPECTED_HF_REVISION}:{identity}")
    derived_instance, derivation = derive_candidate_geometry(candidate_key, source_instances, private_root / "geometry_blobs")
    scene = {
        "schema_version": 2,
        "camera_frame": CAMERA_FRAME_SOURCE_TO_CANONICAL.tolist(),
        "mesh_instances": [derived_instance],
    }
    source = {
        "source_asset_id": identity,
        "source_urdf": str(urdf),
        "source_urdf_sha256": sha256_file(urdf),
        "source_finaljson": str(regular_file(dataset / "finaljson" / f"{identity}.json")),
        "source_finaljson_sha256": sha256_file(dataset / "finaljson" / f"{identity}.json"),
        "mesh_bearing_links": mesh_links,
        "visual_count": visuals,
    }
    return scene, source, derivation


def validate_hf_metadata() -> dict[str, str]:
    metadata = regular_file(HF_METADATA)
    lines = metadata.read_text(encoding="utf-8").splitlines()
    if len(lines) != 3 or lines[0] != EXPECTED_HF_REVISION or lines[1] != EXPECTED_ARCHIVE_SHA256:
        raise RuntimeError("local Hugging Face metadata drifted")
    if sha256_file(metadata) != EXPECTED_HF_METADATA_SHA256:
        raise RuntimeError("local Hugging Face metadata hash drifted")
    return {"revision": lines[0], "archive_etag_sha256": lines[1], "download_timestamp": lines[2], "sha256": EXPECTED_HF_METADATA_SHA256}


def exact_regular_files(directory: Path) -> dict[str, Path]:
    directory = regular_dir(directory)
    result: dict[str, Path] = {}
    for entry in directory.iterdir():
        if entry.is_symlink() or not stat.S_ISREG(entry.lstat().st_mode):
            raise RuntimeError(f"private closure contains non-regular file: {entry}")
        result[entry.name] = entry
    return result


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    return [
        json.loads(line)
        for line in regular_file(path).read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]


def validate_numeric_npz(path: Path, derivation: dict[str, Any]) -> None:
    path = regular_file(path)
    with zipfile.ZipFile(path, "r") as bundle:
        infos = bundle.infolist()
        if [info.filename for info in infos] != ["vertices.npy", "faces.npy"]:
            raise RuntimeError(f"numeric NPZ entry closure/order drifted: {path}")
        arrays: dict[str, np.ndarray] = {}
        for info in infos:
            expected_mode = stat.S_IFREG | 0o644
            if (
                info.is_dir()
                or info.compress_type != zipfile.ZIP_STORED
                or info.date_time != (1980, 1, 1, 0, 0, 0)
                or info.create_system != 3
                or ((info.external_attr >> 16) & 0o177777) != expected_mode
                or info.flag_bits & 0x1
            ):
                raise RuntimeError(f"numeric NPZ metadata drifted: {path}/{info.filename}")
            with bundle.open(info, "r") as stream:
                arrays[info.filename] = np.load(stream, allow_pickle=False)
    vertices = arrays["vertices.npy"]
    faces = arrays["faces.npy"]
    if vertices.dtype.str != "<f8" or vertices.ndim != 2 or vertices.shape[1:] != (3,):
        raise RuntimeError(f"numeric NPZ vertices schema drifted: {path}")
    if faces.dtype.str != "<i8" or faces.ndim != 2 or faces.shape[1:] != (3,):
        raise RuntimeError(f"numeric NPZ faces schema drifted: {path}")
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
    for field, value in expected.items():
        if derivation.get(field) != value:
            raise RuntimeError(f"numeric derivation field drifted: {path}/{field}")


def validate_private_closure(private_root: Path, render_plan: list[dict[str, Any]], *, lock_written: bool) -> dict[str, Any]:
    private_root = regular_dir(private_root)
    expected_top = {
        "archive_member_manifest.jsonl", "geometry_blobs", "geometry_scenes",
        "geometry_derivation_manifest.jsonl", "render_plan.jsonl", "source_binding.jsonl",
    }
    if lock_written:
        expected_top.add("source_audit.lock.json")
    actual_top = {entry.name for entry in private_root.iterdir()}
    if actual_top != expected_top:
        raise RuntimeError(f"private source top-level closure drifted: {sorted(actual_top)}")
    for name in expected_top - {"geometry_blobs", "geometry_scenes"}:
        regular_file(private_root / name)
    scene_files = exact_regular_files(private_root / "geometry_scenes")
    candidate_keys = [row.get("candidate_key") for row in render_plan]
    if (
        len(set(candidate_keys)) != len(candidate_keys)
        or any(
            not isinstance(key, str)
            or len(key) != 64
            or any(character not in "0123456789abcdef" for character in key)
            for key in candidate_keys
        )
    ):
        raise RuntimeError("private render-plan opaque candidate-key closure drifted")
    expected_scene_names = {f"{row['candidate_key']}.json" for row in render_plan}
    if set(scene_files) != expected_scene_names or len(expected_scene_names) != len(render_plan):
        raise RuntimeError("private geometry-scene set differs from render plan")
    derivations = read_jsonl(private_root / "geometry_derivation_manifest.jsonl")
    if (
        len(derivations) != len(render_plan)
        or [row.get("candidate_key") for row in derivations]
        != [row.get("candidate_key") for row in render_plan]
    ):
        raise RuntimeError("private geometry-derivation candidate closure/order drifted")
    derivation_by_key = {row["candidate_key"]: row for row in derivations}
    if len(derivation_by_key) != len(derivations):
        raise RuntimeError("duplicate private geometry-derivation candidate key")
    archive_rows = read_jsonl(private_root / "archive_member_manifest.jsonl")
    archive_hashes = {row["relative_path"]: row["sha256"] for row in archive_rows}
    if len(archive_hashes) != len(archive_rows):
        raise RuntimeError("duplicate archive member in private manifest")
    numeric_source_audit: dict[tuple[str, str], dict[str, Any]] = {}
    referenced_blobs: dict[str, str] = {}
    for row in render_plan:
        if set(row) != {"candidate_key", "geometry_scene", "geometry_scene_sha256"}:
            raise RuntimeError(f"render plan schema drifted: {row.get('candidate_key')}")
        relative = Path(row["geometry_scene"])
        if relative != Path("geometry_scenes") / f"{row['candidate_key']}.json":
            raise RuntimeError(f"render plan scene path drifted: {row['candidate_key']}")
        scene_path = scene_files[relative.name]
        if sha256_file(scene_path) != row["geometry_scene_sha256"]:
            raise RuntimeError(f"private geometry-scene hash drifted: {row['candidate_key']}")
        scene = json.loads(scene_path.read_text(encoding="utf-8"))
        if set(scene) != {"schema_version", "camera_frame", "mesh_instances"} or scene["schema_version"] != 2:
            raise RuntimeError(f"private geometry-scene schema drifted: {row['candidate_key']}")
        if scene["camera_frame"] != CAMERA_FRAME_SOURCE_TO_CANONICAL.tolist():
            raise RuntimeError(f"private camera frame drifted: {row['candidate_key']}")
        if not isinstance(scene["mesh_instances"], list) or len(scene["mesh_instances"]) != 1:
            raise RuntimeError(f"private numeric scene must contain exactly one instance: {row['candidate_key']}")
        instance = scene["mesh_instances"][0]
        if set(instance) != {"mesh_blob", "mesh_sha256", "transform"}:
            raise RuntimeError(f"private mesh-instance schema drifted: {row['candidate_key']}")
        if instance["transform"] != np.eye(4).tolist():
            raise RuntimeError(f"private numeric scene transform must be identity: {row['candidate_key']}")
        blob = Path(instance["mesh_blob"])
        if (
            blob.is_absolute()
            or ".." in blob.parts
            or blob.parts[:1] != ("geometry_blobs",)
            or len(blob.parts) != 2
            or blob.suffix != ".npz"
            or blob.name != f"{instance['mesh_sha256']}.npz"
        ):
            raise RuntimeError(f"unsafe or non-content-addressed private geometry blob: {blob}")
        previous = referenced_blobs.setdefault(blob.name, instance["mesh_sha256"])
        if previous != instance["mesh_sha256"]:
            raise RuntimeError(f"conflicting private geometry-blob hash: {blob.name}")
        derivation = derivation_by_key[row["candidate_key"]]
        expected_derivation_fields = {
            "schema_version", "candidate_key", "ordered_source_instances", "derived_blob",
            "derived_blob_sha256", "vertices_dtype", "faces_dtype", "vertex_count",
            "face_count", "vertices_content_sha256", "faces_content_sha256", "npz_entries",
            "npz_compression",
        }
        if set(derivation) != expected_derivation_fields or derivation["schema_version"] != 1:
            raise RuntimeError(f"private geometry-derivation schema drifted: {row['candidate_key']}")
        if (
            derivation["derived_blob"] != instance["mesh_blob"]
            or derivation["derived_blob_sha256"] != instance["mesh_sha256"]
        ):
            raise RuntimeError(f"private geometry-derivation scene binding drifted: {row['candidate_key']}")
        source_instances = derivation["ordered_source_instances"]
        if not isinstance(source_instances, list) or not source_instances:
            raise RuntimeError(f"empty private source derivation: {row['candidate_key']}")
        expected_source_fields = {
            "source_member_path", "source_member_path_sha256", "source_obj_sha256",
            "q0_transform_sha256", "numeric_vertices_sha256", "numeric_faces_sha256",
            "numeric_instance_order_sha256", "source_vertex_count", "source_face_count",
        }
        expected_identity_transform_sha = hashlib.sha256(
            np.ascontiguousarray(np.eye(4), dtype="<f8").tobytes(order="C")
        ).hexdigest()
        ordering: list[str] = []
        reconstructed_vertices: list[np.ndarray] = []
        reconstructed_faces: list[np.ndarray] = []
        reconstructed_vertex_offset = 0
        for source_instance in source_instances:
            if set(source_instance) != expected_source_fields:
                raise RuntimeError(f"private source-derivation schema drifted: {row['candidate_key']}")
            member = source_instance["source_member_path"]
            if (
                source_instance["source_member_path_sha256"] != sha256_text(member)
                or archive_hashes.get(member) != source_instance["source_obj_sha256"]
                or source_instance["q0_transform_sha256"] != expected_identity_transform_sha
            ):
                raise RuntimeError(f"private source-derivation archive/q0 binding drifted: {row['candidate_key']}")
            member_path = PurePosixPath(member)
            if (
                member_path.is_absolute()
                or any(part in {"", ".", ".."} for part in member_path.parts)
                or member_path.suffix.lower() != ".obj"
                or member_path.parts[:1] != ("partseg",)
            ):
                raise RuntimeError(f"unsafe source member in numeric derivation: {member!r}")
            audit_key = (
                source_instance["source_obj_sha256"],
                source_instance["q0_transform_sha256"],
            )
            if audit_key not in numeric_source_audit:
                source_path = regular_file(DATASET / Path(*member_path.parts))
                if sha256_file(source_path) != source_instance["source_obj_sha256"]:
                    raise RuntimeError(f"source OBJ changed after archive closure: {member}")
                vertices, faces = parse_obj_numeric(source_path)
                vertex_bytes = np.ascontiguousarray(vertices, dtype="<f8").tobytes(order="C")
                face_bytes = np.ascontiguousarray(faces, dtype="<i8").tobytes(order="C")
                transform_bytes = np.ascontiguousarray(np.eye(4), dtype="<f8").tobytes(order="C")
                numeric_source_audit[audit_key] = {
                    "numeric_vertices_sha256": hashlib.sha256(vertex_bytes).hexdigest(),
                    "numeric_faces_sha256": hashlib.sha256(face_bytes).hexdigest(),
                    "numeric_instance_order_sha256": hashlib.sha256(
                        vertex_bytes + face_bytes + transform_bytes
                    ).hexdigest(),
                    "source_vertex_count": int(len(vertices)),
                    "source_face_count": int(len(faces)),
                    "_vertices": vertices,
                    "_faces": faces,
                }
            else:
                vertices = numeric_source_audit[audit_key]["_vertices"]
                faces = numeric_source_audit[audit_key]["_faces"]
            for field, expected in numeric_source_audit[audit_key].items():
                if field.startswith("_"):
                    continue
                if source_instance.get(field) != expected:
                    raise RuntimeError(
                        f"source numeric derivation field drifted: {row['candidate_key']}/{field}"
                    )
            ordering.append(source_instance["numeric_instance_order_sha256"])
            reconstructed_vertices.append(np.ascontiguousarray(vertices, dtype="<f8"))
            reconstructed_faces.append(
                np.ascontiguousarray(faces + reconstructed_vertex_offset, dtype="<i8")
            )
            reconstructed_vertex_offset += len(vertices)
        if ordering != sorted(ordering):
            raise RuntimeError(f"private source-derivation ordering drifted: {row['candidate_key']}")
        combined_vertices = np.ascontiguousarray(
            np.concatenate(reconstructed_vertices, axis=0), dtype="<f8"
        )
        combined_faces = np.ascontiguousarray(
            np.concatenate(reconstructed_faces, axis=0), dtype="<i8"
        )
        reconstructed = {
            "vertex_count": int(len(combined_vertices)),
            "face_count": int(len(combined_faces)),
            "vertices_content_sha256": hashlib.sha256(
                combined_vertices.tobytes(order="C")
            ).hexdigest(),
            "faces_content_sha256": hashlib.sha256(
                combined_faces.tobytes(order="C")
            ).hexdigest(),
        }
        for field, expected in reconstructed.items():
            if derivation.get(field) != expected:
                raise RuntimeError(
                    f"derived NPZ differs from reconstructed source numeric geometry: "
                    f"{row['candidate_key']}/{field}"
                )
    blob_files = exact_regular_files(private_root / "geometry_blobs")
    if set(blob_files) != set(referenced_blobs):
        raise RuntimeError("private geometry-blob set differs from scene references")
    for name, expected in referenced_blobs.items():
        if sha256_file(blob_files[name]) != expected:
            raise RuntimeError(f"private geometry-blob hash drifted: {name}")
    for derivation in derivations:
        validate_numeric_npz(
            blob_files[Path(derivation["derived_blob"]).name],
            derivation,
        )
    return {
        "top_level_entries": sorted(expected_top),
        "geometry_scene_count": len(scene_files),
        "referenced_geometry_blob_count": len(blob_files),
        "all_entries_regular_non_symlink": True,
        "no_extra_or_unreferenced_geometry": True,
    }


def run(output: Path) -> None:
    gate = validate_pre_result_gate()
    output = safe_write(output)
    staging = safe_write(output.with_name(output.name + ".staging"))
    if output.exists() or staging.exists():
        raise RuntimeError(f"private source final/staging roots must both be absent: {output}, {staging}")
    staging.mkdir(parents=True, exist_ok=False)
    try:
        dataset = regular_dir(DATASET)
        hf_metadata = validate_hf_metadata()
        archive_manifest = staging / "archive_member_manifest.jsonl"
        archive_hashes, archive_summary = archive_extraction_closure(ARCHIVE, dataset, archive_manifest)
        part_ids = {entry.name for entry in regular_dir(dataset / "partseg").iterdir() if entry.is_dir() and not entry.is_symlink()}
        json_ids = {entry.stem for entry in regular_dir(dataset / "finaljson").glob("*.json") if entry.is_file() and not entry.is_symlink()}
        urdf_ids = {
            entry.stem for entry in regular_dir(dataset / "urdf").glob("*.urdf")
            if entry.is_file() and not entry.is_symlink() and not entry.stem.endswith(("_collision", "_sim"))
        }
        if part_ids != json_ids or part_ids != urdf_ids or len(part_ids) != EXPECTED_ASSETS:
            raise RuntimeError(f"identity closure drifted: part={len(part_ids)} json={len(json_ids)} urdf={len(urdf_ids)}")
        geometry_root = staging / "geometry_scenes"
        blob_root = staging / "geometry_blobs"
        geometry_root.mkdir()
        blob_root.mkdir()
        render_plan: list[dict[str, Any]] = []
        source_binding: list[dict[str, Any]] = []
        derivation_rows: list[dict[str, Any]] = []
        mesh_links = 0
        visuals = 0
        for index, identity in enumerate(sorted(part_ids), 1):
            candidate_key = sha256_text(f"physx-mobility-{EXPECTED_HF_REVISION}:{identity}")
            scene, source, derivation = geometry_scene(dataset, identity, archive_hashes, staging)
            scene_path = geometry_root / f"{candidate_key}.json"
            write_json(scene_path, scene)
            render_plan.append({
                "candidate_key": candidate_key,
                "geometry_scene": scene_path.relative_to(staging).as_posix(),
                "geometry_scene_sha256": sha256_file(scene_path),
            })
            source_binding.append({"candidate_key": candidate_key, **source})
            derivation_rows.append(derivation)
            mesh_links += source["mesh_bearing_links"]
            visuals += source["visual_count"]
            if index % 25 == 0 or index == len(part_ids):
                print(f"[source-geometry] {index}/{len(part_ids)}", flush=True)
        if mesh_links != EXPECTED_MESH_BEARING_LINKS or visuals != EXPECTED_VISUALS:
            raise RuntimeError(f"zero-pose aggregate invariant drifted: links={mesh_links} visuals={visuals}")
        render_plan.sort(key=lambda row: row["candidate_key"])
        source_binding.sort(key=lambda row: row["candidate_key"])
        derivation_rows.sort(key=lambda row: row["candidate_key"])
        write_jsonl(staging / "render_plan.jsonl", render_plan)
        write_jsonl(staging / "source_binding.jsonl", source_binding)
        write_jsonl(staging / "geometry_derivation_manifest.jsonl", derivation_rows)
        private_closure = validate_private_closure(staging, render_plan, lock_written=False)
        lock = {
            "schema_version": 3,
            "status": "PASS",
            "private": True,
            "method": "physx_mobility_clip_retrieval_v1",
            "dataset_id": "Caoza/PhysX-Mobility",
            "dataset_revision": EXPECTED_HF_REVISION,
            "archive_sha256": EXPECTED_ARCHIVE_SHA256,
            "archive_bytes": EXPECTED_ARCHIVE_BYTES,
            **archive_summary,
            "local_huggingface_metadata": hf_metadata,
            "asset_count": len(render_plan),
            "render_plan_sha256": sha256_file(staging / "render_plan.jsonl"),
            "source_binding_sha256": sha256_file(staging / "source_binding.jsonl"),
            "geometry_derivation_manifest_sha256": sha256_file(staging / "geometry_derivation_manifest.jsonl"),
            "geometry_scene_count": len(render_plan),
            "geometry_blob_count": len(list(blob_root.iterdir())),
            "private_geometry_closure": private_closure,
            "mesh_bearing_link_count": mesh_links,
            "visual_count": visuals,
            "zero_pose_geometry_invariant": {
                "all_mesh_bearing_link_world_transforms_identity_at_q0": True,
                "all_visual_origins_identity": True,
                "all_mesh_scales_one": True,
                "maximum_transform_error": 0.0,
            },
            "camera_frame_convention": "camera_frame maps source coordinates to canonical camera coordinates; renderer maps canonical camera/light poses back with inverse(camera_frame); geometry vertices and q0 transforms remain unchanged",
            "camera_frame_source_to_canonical": CAMERA_FRAME_SOURCE_TO_CANONICAL.tolist(),
            "camera_frame_official_evidence": {
                "repository_commit": "e221826e6176d940905126d1894f9c1c933b70a8",
                "path": "dataset/1voxel_mobility.py",
                "sha256": "db24f80e230f8f2e3d2f53fd575b47fdfd180252d2cd80d3440bf56a51a6e7bc",
                "rule": "R_x(+pi/2) is the official fixed preprocessing rotation; it is used only as a camera frame and is not applied to source geometry",
                "semantic_up_claim": False,
            },
            "geometry_scene_policy": "one deterministic label-stripped numeric NPZ per candidate with pre-applied raw q0 transforms and a fixed camera_frame; scene exposes only the numeric NPZ, identity transform, and camera frame; no source OBJ text, identity, link, joint, visual, semantic, category, prompt, spec, dimension, or prior score",
            "source_auditor_sha256": sha256_file(SCRIPT),
            "implementation": gate["implementation"],
            "amendment_sha256": gate["amendment_sha256"],
            "pre_result_audit_sha256": gate["audit_sha256"],
            "closure": "every regular ZIP member was stream-SHA256 compared with the corresponding extracted regular file and the regular-file sets were exactly equal",
        }
        write_json(staging / "source_audit.lock.json", lock)
        validate_private_closure(staging, render_plan, lock_written=True)
        staging.replace(output)
        print(json.dumps({"status": "SOURCE_AUDIT_LOCKED", "assets": len(render_plan), "lock_sha256": sha256_file(output / "source_audit.lock.json")}, indent=2), flush=True)
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    run(args.output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
