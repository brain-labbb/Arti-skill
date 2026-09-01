#!/usr/bin/env python3
"""Build a locked render-only PhysX-Mobility snapshot for Table 4 retrieval."""

from __future__ import annotations

import argparse
import concurrent.futures
import hashlib
import json
import math
import os
import platform
import multiprocessing
import shutil
import subprocess
import stat
import zipfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

os.environ["PYOPENGL_PLATFORM"] = "egl"
os.environ["EGL_DEVICE_ID"] = "1"
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["NUMEXPR_NUM_THREADS"] = "1"

import numpy as np
import PIL
from PIL import Image
import OpenGL
from OpenGL import EGL, GL
import pyrender
import trimesh


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
WORKSPACE = Path("/mnt/zsn/lyb").resolve()
EXTERNAL_WORKSPACE = Path("/mnt/zsn/zsn_workspace").resolve()
HF_CACHE_ROOT = Path("/root/.cache/huggingface").resolve()
DEFAULT_SOURCE = Path("/mnt/zsn/zsn_workspace/Ctrl-3D-trellis2-controlnet-dev/demo/physical_edit_demo/third_party/physx_mobility/extracted/PhysX_mobility")
DEFAULT_ARCHIVE = DEFAULT_SOURCE.parents[1] / "PhysX-Mobility.zip"
DEFAULT_HF_METADATA = DEFAULT_SOURCE.parents[1] / ".cache/huggingface/download/PhysX-Mobility.zip.metadata"
DEFAULT_SNAPSHOT = REPO / "exp/runtime/table4_constraints_v2/physx_mobility_clip_retrieval_v1_snapshot"
DEFAULT_AUDIT = REPO / "exp/runtime/table4_constraints_v2/physx_mobility_clip_retrieval_v1/protocol_audit_pre_result.json"
DEFAULT_AMENDMENT = REPO / "exp/reference/table4_constraints_v2/amendment_physx_mobility_clip_retrieval_v1.json"
MODEL_SOURCE = HF_CACHE_ROOT / "hub/models--openai--clip-vit-base-patch32/snapshots/3d74acf9a28c67741b2f4f2ea7635f0aaf6f0268"
SOURCE_INVENTORY = REPO / "exp/baselines/physx_mobility/source_inventory.jsonl"
SOURCE_SUMMARY = REPO / "exp/baselines/physx_mobility/source_audit.json"
ARCHIVE_EXTRACTION_MANIFEST = REPO / "exp/baselines/physx_mobility/archive_extraction_manifest.jsonl"

EXPECTED_ARCHIVE_BYTES = 937_374_668
EXPECTED_ARCHIVE_SHA256 = "88308cc2a4cc6177c59e32c2de51e881e6b961737295e5082d7ed01cca221908"
EXPECTED_HF_REVISION = "d0768ee9e1415f6be8db78d6389ba018b85134c0"
EXPECTED_IDENTITIES = 2024
EXPECTED_MESH_BEARING_LINKS = 14_096
EXPECTED_VISUALS = 91_855
VIEWS = tuple((float(azimuth), 22.5) for azimuth in range(0, 360, 45))
RESOLUTION = 256
EXPECTED_WORKERS = 4
BACKGROUND = (255, 255, 255)
BASE_COLOR = (0.62, 0.66, 0.70, 1.0)
_RENDERER: pyrender.OffscreenRenderer | None = None
_WORKER_RUNTIME_SHA256: str | None = None
EXPECTED_THREAD_ENV = {
    "OPENBLAS_NUM_THREADS": "1",
    "OMP_NUM_THREADS": "1",
    "MKL_NUM_THREADS": "1",
    "NUMEXPR_NUM_THREADS": "1",
    "PYOPENGL_PLATFORM": "egl",
    "EGL_DEVICE_ID": "1",
}
EXPECTED_RUNTIME = {
    "python": "3.13.2",
    "numpy": "2.4.4",
    "pillow": "12.3.0",
    "pyrender": "0.1.45",
    "trimesh": "4.12.2",
    "pyopengl": "3.1.10",
    "gl_vendor": "NVIDIA Corporation",
    "gl_renderer": "NVIDIA L20X/PCIe/SSE2",
    "gl_version": "4.1.0 NVIDIA 570.172.08",
    "glsl_version": "4.10 NVIDIA via Cg compiler",
    "egl_version": "1.5",
    "egl_vendor": "NVIDIA",
    "mesa_version": None,
}
EXPECTED_GPU = {
    "physical_index": "1",
    "uuid": "GPU-7390aec1-d177-6672-4136-d998c85f489d",
    "name": "NVIDIA L20X",
    "driver_version": "570.172.08",
}
EXPECTED_MODEL_FILES = {
    "config.json": "b575ef3c36f2a057fa19e221650105052d61cc9c1a972ec15019c6261ec98770",
    "merges.txt": "f526393189112391ce6f9795d4695f704121ce452c3aad1f5335cc41337eba85",
    "preprocessor_config.json": "910e70b3956ac9879ebc90b22fb3bc8a75b6a0677814500101a4c072bd7857bd",
    "pytorch_model.bin": "a63082132ba4f97a80bea76823f544493bffa8082296d62d71581a4feff1576f",
    "special_tokens_map.json": "f8c0d6c39aee3f8431078ef6646567b0aba7f2246e9c54b8b99d55c22b707cbf",
    "tokenizer.json": "b556ac8c99757ffb677208af34bc8c6721572114111a6e0aaf5fa69ff0b8d842",
    "tokenizer_config.json": "34b7336e4bee12e0a9730eaf5189f582ef3c3eea5027f65730e5717256755aad",
    "vocab.json": "5047b556ce86ccaf6aa22b3ffccfc52d391ea4accdab9c2f2407da5b742d4363",
}


def safe_read(path: Path, *, must_exist: bool = True) -> Path:
    resolved = path.resolve(strict=must_exist)
    allowed = (WORKSPACE, EXTERNAL_WORKSPACE, HF_CACHE_ROOT)
    if not any(resolved == root or root in resolved.parents for root in allowed):
        raise RuntimeError(f"read path outside authorized roots: {resolved}")
    return resolved


def safe_write(path: Path) -> Path:
    resolved = path.resolve(strict=False)
    if resolved != WORKSPACE and WORKSPACE not in resolved.parents:
        raise RuntimeError(f"write path outside primary workspace: {resolved}")
    return resolved


def sha256_file(path: Path, block_size: int = 8 * 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with safe_read(path).open("rb") as stream:
        for block in iter(lambda: stream.read(block_size), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_json(value: Any) -> bytes:
    return (json.dumps(value, sort_keys=True, ensure_ascii=True, separators=(",", ":")) + "\n").encode()


def write_json(path: Path, value: Any) -> None:
    destination = safe_write(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(destination.suffix + ".tmp")
    temporary.write_bytes(json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True).encode() + b"\n")
    temporary.replace(destination)


def write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    destination = safe_write(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    payload = b"".join(canonical_json(row) for row in rows)
    temporary = destination.with_suffix(destination.suffix + ".tmp")
    temporary.write_bytes(payload)
    temporary.replace(destination)


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


def validate_protocol(amendment_path: Path, audit_path: Path) -> dict[str, Any]:
    amendment_path = regular_file(amendment_path)
    amendment = json.loads(amendment_path.read_text())
    if amendment.get("status") != "pre_result_frozen":
        raise RuntimeError("amendment is not pre_result_frozen")
    if amendment["implementation"]["snapshot_builder_sha256"] != sha256_file(SCRIPT):
        raise RuntimeError("snapshot builder hash differs from amendment")
    audit_path = regular_file(audit_path)
    audit = json.loads(audit_path.read_text())
    if str(audit.get("verdict", "")).upper() != "PASS" or audit.get("protocol_ready") is not True:
        raise RuntimeError("independent pre-result protocol audit is not PASS")
    if audit.get("amendment_sha256") != sha256_file(amendment_path):
        raise RuntimeError("audit is not bound to the frozen amendment")
    for key in ("snapshot_builder_sha256", "selector_sha256", "materializer_sha256", "canonicalizer_sha256", "verifier_sha256"):
        if audit.get(key) != amendment["implementation"][key]:
            raise RuntimeError(f"audit implementation binding mismatch: {key}")
    return {"amendment": amendment, "audit_sha256": sha256_file(audit_path)}


def parse_vector(value: str | None, default: tuple[float, float, float]) -> tuple[float, float, float]:
    parsed = default if value is None else tuple(float(item) for item in value.split())
    if len(parsed) != 3 or not all(math.isfinite(item) for item in parsed):
        raise RuntimeError(f"invalid URDF vector: {value!r}")
    return parsed


def referenced_objs(source: Path, identity: str) -> tuple[list[Path], dict[str, int]]:
    urdf = regular_file(source / "urdf" / f"{identity}.urdf")
    root = ET.parse(urdf).getroot()
    paths: set[Path] = set()
    mesh_links = 0
    visual_count = 0
    parents: dict[str, tuple[str, np.ndarray]] = {}
    for joint in root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            raise RuntimeError(f"joint without parent/child: {identity}")
        origin = joint.find("origin")
        xyz = parse_vector(origin.attrib.get("xyz") if origin is not None else None, (0.0, 0.0, 0.0))
        roll, pitch, yaw = parse_vector(origin.attrib.get("rpy") if origin is not None else None, (0.0, 0.0, 0.0))
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        transform = np.eye(4)
        transform[:3, :3] = np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ])
        transform[:3, 3] = xyz
        child_name = child.attrib["link"]
        if child_name in parents:
            raise RuntimeError(f"multiple parents: {identity}/{child_name}")
        parents[child_name] = (parent.attrib["link"], transform)
    links = {link.attrib["name"]: link for link in root.findall("link")}
    world: dict[str, np.ndarray] = {}
    pending = set(links)
    while pending:
        progressed = False
        for link_name in sorted(pending):
            if link_name not in parents:
                world[link_name] = np.eye(4)
            else:
                parent_name, transform = parents[link_name]
                if parent_name not in world:
                    continue
                world[link_name] = world[parent_name] @ transform
            pending.remove(link_name)
            progressed = True
        if not progressed:
            raise RuntimeError(f"unresolved URDF hierarchy: {identity}/{sorted(pending)}")
    for link in root.findall("link"):
        visuals = link.findall("visual")
        if visuals:
            mesh_links += 1
            if not np.array_equal(world[link.attrib["name"]], np.eye(4)):
                error = float(np.max(np.abs(world[link.attrib["name"]] - np.eye(4))))
                raise RuntimeError(f"non-identity mesh-link zero-pose world transform: {identity}/{link.attrib['name']} error={error}")
        for visual in visuals:
            visual_count += 1
            origin = visual.find("origin")
            xyz = parse_vector(origin.attrib.get("xyz") if origin is not None else None, (0.0, 0.0, 0.0))
            rpy = parse_vector(origin.attrib.get("rpy") if origin is not None else None, (0.0, 0.0, 0.0))
            if xyz != (0.0, 0.0, 0.0) or rpy != (0.0, 0.0, 0.0):
                raise RuntimeError(f"non-identity visual origin: {identity}/{link.attrib.get('name')}")
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                raise RuntimeError(f"non-mesh visual geometry: {identity}/{link.attrib.get('name')}")
            if parse_vector(mesh.attrib.get("scale"), (1.0, 1.0, 1.0)) != (1.0, 1.0, 1.0):
                raise RuntimeError(f"non-unit visual mesh scale: {identity}/{link.attrib.get('name')}")
            reference = mesh.attrib.get("filename", "")
            candidate = (urdf.parent / reference).resolve()
            expected_root = regular_dir(source / "partseg" / identity / "objs")
            if candidate.suffix.lower() != ".obj" or expected_root not in candidate.parents:
                raise RuntimeError(f"unexpected visual mesh reference for {identity}: {reference}")
            paths.add(regular_file(candidate))
    if not paths:
        raise RuntimeError(f"no visual OBJ references: {identity}")
    return sorted(paths), {"mesh_bearing_links": mesh_links, "visuals": visual_count}


def archive_gate(archive: Path, source: Path) -> dict[str, Any]:
    archive = regular_file(archive)
    if archive.stat().st_size != EXPECTED_ARCHIVE_BYTES or sha256_file(archive) != EXPECTED_ARCHIVE_SHA256:
        raise RuntimeError("PhysX-Mobility archive size/hash mismatch")
    actual_files: set[str] = set()
    for directory, dirnames, filenames in os.walk(source, followlinks=False):
        directory_path = Path(directory)
        for dirname in dirnames:
            child = directory_path / dirname
            if child.is_symlink() or not stat.S_ISDIR(child.lstat().st_mode):
                raise RuntimeError(f"non-regular directory in extraction: {child}")
        for filename in filenames:
            child = directory_path / filename
            if child.is_symlink() or not stat.S_ISREG(child.lstat().st_mode):
                raise RuntimeError(f"non-regular file in extraction: {child}")
            actual_files.add(child.relative_to(source).as_posix())
    extracted_bytes = 0
    manifest_path = safe_write(ARCHIVE_EXTRACTION_MANIFEST)
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = manifest_path.with_suffix(manifest_path.suffix + ".tmp")
    with zipfile.ZipFile(archive) as bundle:
        members = [row for row in bundle.infolist() if not row.is_dir()]
        expected: dict[str, zipfile.ZipInfo] = {}
        for row in members:
            prefix = "PhysX_mobility/"
            if not row.filename.startswith(prefix):
                raise RuntimeError(f"archive member outside expected root: {row.filename}")
            relative = row.filename[len(prefix):]
            relative_path = Path(relative)
            if relative_path.is_absolute() or not relative or any(part in {"", ".", ".."} for part in relative_path.parts):
                raise RuntimeError(f"unsafe archive member path: {relative!r}")
            if relative_path.as_posix() != relative:
                raise RuntimeError(f"non-normalized archive member path: {relative!r}")
            mode = (row.external_attr >> 16) & 0o170000
            if mode == stat.S_IFLNK:
                raise RuntimeError(f"symlink member in archive: {relative}")
            if not relative or relative in expected:
                raise RuntimeError(f"empty or duplicate archive member: {relative!r}")
            expected[relative] = row
        if actual_files != set(expected):
            missing = sorted(set(expected) - actual_files)
            extra = sorted(actual_files - set(expected))
            raise RuntimeError(f"archive/extraction file-set mismatch: missing={missing[:5]} extra={extra[:5]}")
        try:
            with temporary.open("w", encoding="utf-8") as manifest:
                for index, relative in enumerate(sorted(expected), 1):
                    row = expected[relative]
                    candidate = regular_file(source / relative)
                    if candidate.stat().st_size != row.file_size:
                        raise RuntimeError(f"archive/extraction size mismatch: {relative}")
                    digest = hashlib.sha256()
                    with bundle.open(row, "r") as archived, candidate.open("rb") as extracted:
                        while True:
                            archive_block = archived.read(1024 * 1024)
                            extracted_block = extracted.read(1024 * 1024)
                            if archive_block != extracted_block:
                                raise RuntimeError(f"archive/extraction byte mismatch: {relative}")
                            if not archive_block:
                                break
                            digest.update(archive_block)
                    extracted_bytes += row.file_size
                    manifest.write(json.dumps({
                        "path": relative,
                        "bytes": row.file_size,
                        "sha256": digest.hexdigest(),
                    }, sort_keys=True, ensure_ascii=True) + "\n")
                    if index % 10000 == 0:
                        print(f"[archive] {index}/{len(expected)}", flush=True)
            if manifest_path.exists():
                if regular_file(manifest_path).read_bytes() != temporary.read_bytes():
                    raise RuntimeError("archive extraction manifest differs; replay refuses overwrite")
                temporary.unlink()
            else:
                temporary.replace(manifest_path)
        except Exception:
            temporary.unlink(missing_ok=True)
            raise
    return {
        "archive_bytes": archive.stat().st_size,
        "archive_sha256": sha256_file(archive),
        "archive_members": len(members),
        "extracted_files": len(actual_files),
        "extracted_bytes": extracted_bytes,
        "archive_extraction_manifest": str(manifest_path),
        "archive_extraction_manifest_sha256": sha256_file(manifest_path),
        "closure": "every extracted regular file was compared byte-for-byte with the pinned ZIP member; file sets are exactly equal",
    }


def hf_metadata_gate(metadata_path: Path) -> dict[str, Any]:
    metadata_path = regular_file(metadata_path)
    lines = metadata_path.read_text(encoding="utf-8").splitlines()
    if len(lines) != 3 or lines[0] != EXPECTED_HF_REVISION or lines[1] != EXPECTED_ARCHIVE_SHA256:
        raise RuntimeError(f"local Hugging Face metadata drifted: {lines!r}")
    try:
        timestamp = float(lines[2])
    except ValueError as error:
        raise RuntimeError("invalid local Hugging Face metadata timestamp") from error
    if not math.isfinite(timestamp) or timestamp <= 0:
        raise RuntimeError("invalid local Hugging Face metadata timestamp")
    return {
        "path": str(metadata_path),
        "sha256": sha256_file(metadata_path),
        "revision": lines[0],
        "archive_etag_sha256": lines[1],
        "download_timestamp": lines[2],
    }


def build_source_inventory(source: Path) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    source = regular_dir(source)
    part_ids = {row.name for row in regular_dir(source / "partseg").iterdir() if row.is_dir() and not row.is_symlink()}
    json_ids = {row.stem for row in regular_dir(source / "finaljson").glob("*.json") if row.is_file() and not row.is_symlink()}
    urdf_ids = {
        row.stem for row in regular_dir(source / "urdf").glob("*.urdf")
        if row.is_file() and not row.is_symlink() and not row.stem.endswith(("_collision", "_sim"))
    }
    identities = sorted(part_ids & json_ids & urdf_ids)
    if len(part_ids) != EXPECTED_IDENTITIES or part_ids != json_ids or len(identities) != EXPECTED_IDENTITIES:
        raise RuntimeError(
            f"identity closure mismatch: part={len(part_ids)} json={len(json_ids)} base_urdf={len(urdf_ids)} "
            f"intersection={len(identities)}"
        )
    rows = []
    for index, identity in enumerate(identities, 1):
        urdf = regular_file(source / "urdf" / f"{identity}.urdf")
        objects, zero_pose = referenced_objs(source, identity)
        object_rows = [
            {
                "path": path.relative_to(source).as_posix(),
                "bytes": path.stat().st_size,
                "sha256": sha256_file(path),
            }
            for path in objects
        ]
        geometry_digest = hashlib.sha256(b"".join(canonical_json(row) for row in object_rows)).hexdigest()
        rows.append({
            "identity": identity,
            "identity_sha256": hashlib.sha256(identity.encode()).hexdigest(),
            "urdf_path": urdf.relative_to(source).as_posix(),
            "urdf_bytes": urdf.stat().st_size,
            "urdf_sha256": sha256_file(urdf),
            "visual_obj_count": len(object_rows),
            "visual_obj_digest_sha256": geometry_digest,
            "visual_objs": object_rows,
            "mesh_bearing_links": zero_pose["mesh_bearing_links"],
            "visual_count": zero_pose["visuals"],
        })
        if index % 100 == 0:
            print(f"[inventory] {index}/{len(identities)}", flush=True)
    collision = len(list((source / "urdf").glob("*_collision.urdf")))
    sim = len(list((source / "urdf").glob("*_sim.urdf")))
    summary = {
        "status": "PASS",
        "identity_count": len(rows),
        "partseg_identity_count": len(part_ids),
        "finaljson_identity_count": len(json_ids),
        "base_urdf_identity_count": len(urdf_ids),
        "collision_urdf_count": collision,
        "sim_urdf_count": sim,
        "visual_obj_count": sum(row["visual_obj_count"] for row in rows),
        "mesh_bearing_link_count": sum(row["mesh_bearing_links"] for row in rows),
        "urdf_visual_count": sum(row["visual_count"] for row in rows),
        "zero_pose_geometry_equivalence": {
            "status": "PASS",
            "all_mesh_bearing_link_world_transforms_identity_at_q0": True,
            "all_visual_origins_identity": True,
            "all_mesh_scales_one": True,
            "raw_obj_concat_equals_zero_pose_urdf_geometry": True,
        },
    }
    if summary["mesh_bearing_link_count"] != EXPECTED_MESH_BEARING_LINKS or summary["urdf_visual_count"] != EXPECTED_VISUALS:
        raise RuntimeError(f"zero-pose source invariant count drifted: {summary}")
    return rows, summary


def load_obj_fast(path: str) -> trimesh.Trimesh:
    vertices: list[tuple[float, float, float]] = []
    faces: list[tuple[int, int, int]] = []
    with open(path, "r", encoding="utf-8", errors="ignore") as stream:
        for line in stream:
            if line.startswith("v "):
                fields = line.split()
                vertices.append((float(fields[1]), float(fields[2]), float(fields[3])))
            elif line.startswith("f "):
                raw = [int(token.split("/", 1)[0]) for token in line.split()[1:]]
                resolved = [(value - 1 if value > 0 else len(vertices) + value) for value in raw]
                for offset in range(1, len(resolved) - 1):
                    faces.append((resolved[0], resolved[offset], resolved[offset + 1]))
    if not vertices or not faces:
        raise RuntimeError(f"empty OBJ: {path}")
    return trimesh.Trimesh(np.asarray(vertices), np.asarray(faces), process=False)


def camera_pose(
    center: np.ndarray,
    azimuth_deg: float,
    elevation_deg: float,
    radius: float,
) -> np.ndarray:
    azimuth = math.radians(azimuth_deg)
    elevation = math.radians(elevation_deg)
    eye = center + radius * np.array([
        math.cos(elevation) * math.cos(azimuth),
        math.cos(elevation) * math.sin(azimuth),
        math.sin(elevation),
    ])
    backward = (eye - center) / np.linalg.norm(eye - center)
    right = np.cross(np.array([0.0, 0.0, 1.0]), backward)
    right /= np.linalg.norm(right)
    up = np.cross(backward, right)
    pose = np.eye(4)
    pose[:3, 0] = right
    pose[:3, 1] = up
    pose[:3, 2] = backward
    pose[:3, 3] = eye
    return pose


def get_renderer() -> pyrender.OffscreenRenderer:
    global _RENDERER
    if _RENDERER is None:
        _RENDERER = pyrender.OffscreenRenderer(RESOLUTION, RESOLUTION)
    return _RENDERER


def decode_gl_string(field: int) -> str | None:
    value = GL.glGetString(field)
    return value.decode("utf-8") if value else None


def decode_egl_string(field: int) -> str | None:
    value = EGL.eglQueryString(EGL.eglGetCurrentDisplay(), field)
    return value.decode("utf-8") if value else None


def runtime_fingerprint() -> dict[str, Any]:
    get_renderer()
    fingerprint = {
        "python": platform.python_version(),
        "numpy": np.__version__,
        "pillow": PIL.__version__,
        "pyrender": pyrender.__version__,
        "trimesh": trimesh.__version__,
        "pyopengl": OpenGL.__version__,
        "gl_vendor": decode_gl_string(GL.GL_VENDOR),
        "gl_renderer": decode_gl_string(GL.GL_RENDERER),
        "gl_version": decode_gl_string(GL.GL_VERSION),
        "glsl_version": decode_gl_string(GL.GL_SHADING_LANGUAGE_VERSION),
        "egl_version": decode_egl_string(EGL.EGL_VERSION),
        "egl_vendor": decode_egl_string(EGL.EGL_VENDOR),
        "mesa_version": None,
    }
    if fingerprint != EXPECTED_RUNTIME:
        raise RuntimeError(f"renderer runtime fingerprint drifted: {fingerprint!r} != {EXPECTED_RUNTIME!r}")
    observed_env = {key: os.environ.get(key) for key in EXPECTED_THREAD_ENV}
    if observed_env != EXPECTED_THREAD_ENV:
        raise RuntimeError(f"renderer environment drifted: {observed_env!r}")
    return fingerprint


def ensure_worker_runtime() -> str:
    global _WORKER_RUNTIME_SHA256
    if _WORKER_RUNTIME_SHA256 is None:
        _WORKER_RUNTIME_SHA256 = hashlib.sha256(canonical_json(runtime_fingerprint())).hexdigest()
    return _WORKER_RUNTIME_SHA256


def render_one(payload: tuple[dict[str, Any], str, str]) -> dict[str, Any]:
    row, source_text, snapshot_text = payload
    source = Path(source_text)
    snapshot = Path(snapshot_text)
    identity = row["identity"]
    worker_runtime_sha256 = ensure_worker_runtime()
    output = snapshot / "renders" / identity
    if output.exists():
        raise RuntimeError(f"formal renderer refuses pre-existing asset output: {output}")
    output.mkdir(parents=True, exist_ok=True)
    required = [output / f"{index:03d}.png" for index in range(len(VIEWS))]
    try:
        meshes = [load_obj_fast(str(source / item["path"])) for item in row["visual_objs"]]
        combined = trimesh.util.concatenate(meshes)
        bounds = np.asarray(combined.bounds, dtype=np.float64)
        extents = bounds[1] - bounds[0]
        maximum = float(extents.max())
        if not math.isfinite(maximum) or maximum <= 0:
            raise RuntimeError("non-positive geometry extent")
        if not np.isfinite(combined.vertices).all():
            raise RuntimeError("non-finite geometry vertices")
        center = bounds.mean(axis=0)
        half_diagonal = float(np.linalg.norm(extents) / 2.0)
        if not math.isfinite(half_diagonal) or half_diagonal <= 0:
            raise RuntimeError("non-positive geometry half diagonal")
        half_frame = half_diagonal * 1.15
        camera_radius = half_diagonal * 3.2
        near = max(camera_radius - half_diagonal * 1.5, half_diagonal * 1e-4)
        far = camera_radius + half_diagonal * 1.5
        material = pyrender.MetallicRoughnessMaterial(
            baseColorFactor=BASE_COLOR,
            metallicFactor=0.0,
            roughnessFactor=0.82,
            doubleSided=True,
        )
        render_mesh = pyrender.Mesh.from_trimesh(combined, material=material, smooth=False)
        camera = pyrender.OrthographicCamera(xmag=half_frame, ymag=half_frame, znear=near, zfar=far)
        renderer = get_renderer()
        for index, (azimuth, elevation) in enumerate(VIEWS):
            scene = pyrender.Scene(bg_color=np.array([1.0, 1.0, 1.0, 1.0]), ambient_light=np.array([0.42, 0.42, 0.42]))
            scene.add(render_mesh)
            pose = camera_pose(center, azimuth, elevation, camera_radius)
            scene.add(camera, pose=pose)
            key = pyrender.DirectionalLight(color=np.ones(3), intensity=2.4)
            fill = pyrender.DirectionalLight(color=np.array([0.86, 0.91, 1.0]), intensity=1.2)
            scene.add(key, pose=pose)
            scene.add(fill, pose=camera_pose(center, azimuth + 135.0, 35.0, camera_radius))
            color, _ = renderer.render(scene, flags=pyrender.RenderFlags.RGBA)
            rgb = Image.fromarray(color, mode="RGBA").convert("RGB")
            temporary = required[index].with_suffix(".png.tmp")
            rgb.save(temporary, format="PNG", compress_level=6)
            temporary.replace(required[index])
        views = [
            {"name": path.name, "bytes": path.stat().st_size, "sha256": sha256_file(path)}
            for path in required
        ]
        return {
            "status": "eligible",
            "identity": identity,
            "identity_sha256": row["identity_sha256"],
            "views": views,
            "worker_runtime_sha256": worker_runtime_sha256,
        }
    except Exception as error:
        shutil.rmtree(output, ignore_errors=True)
        return {
            "status": "ineligible",
            "identity": identity,
            "identity_sha256": row["identity_sha256"],
            "reason": f"{type(error).__name__}: {error}",
            "worker_runtime_sha256": worker_runtime_sha256,
        }


def copy_model(snapshot: Path) -> dict[str, str]:
    destination = snapshot / "model"
    destination.mkdir(parents=True, exist_ok=True)
    hashes = {}
    source_root = regular_dir(MODEL_SOURCE)
    if {source.name for source in source_root.iterdir() if source.is_file()} != set(EXPECTED_MODEL_FILES):
        raise RuntimeError("pinned CLIP cache snapshot file closure drifted")
    for name, expected_hash in sorted(EXPECTED_MODEL_FILES.items()):
        source = safe_read(source_root / name)
        if not source.is_file() or sha256_file(source) != expected_hash:
            raise RuntimeError(f"pinned CLIP cache file hash drifted: {name}")
        target = destination / source.name
        expected = expected_hash
        if not target.is_file() or sha256_file(target) != expected:
            temporary = target.with_suffix(target.suffix + ".tmp")
            with source.open("rb") as inp, temporary.open("wb") as out:
                for block in iter(lambda: inp.read(8 * 1024 * 1024), b""):
                    out.write(block)
            temporary.replace(target)
        hashes[target.name] = sha256_file(target)
    return hashes


def physical_gpu_gate() -> dict[str, str]:
    fields = subprocess.run(
        ["nvidia-smi", "--query-gpu=index,uuid,name,driver_version", "--format=csv,noheader,nounits", "-i", "1"],
        text=True, capture_output=True, check=True,
    ).stdout.strip().split(", ")
    observed = dict(zip(("physical_index", "uuid", "name", "driver_version"), fields))
    if observed != EXPECTED_GPU:
        raise RuntimeError(f"formal render GPU identity drifted: {observed!r}")
    return observed


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--archive", type=Path, default=DEFAULT_ARCHIVE)
    parser.add_argument("--hf-metadata", type=Path, default=DEFAULT_HF_METADATA)
    parser.add_argument("--snapshot", type=Path, default=DEFAULT_SNAPSHOT)
    parser.add_argument("--amendment", type=Path, default=DEFAULT_AMENDMENT)
    parser.add_argument("--audit", type=Path, default=DEFAULT_AUDIT)
    parser.add_argument("--workers", type=int, default=4)
    args = parser.parse_args()
    if args.workers != EXPECTED_WORKERS:
        raise RuntimeError(f"formal render requires --workers {EXPECTED_WORKERS}")
    protocol_binding = validate_protocol(args.amendment, args.audit)
    source = regular_dir(args.source)
    final_snapshot = safe_write(args.snapshot)
    snapshot = safe_write(final_snapshot.with_name(final_snapshot.name + ".staging"))
    if final_snapshot.exists() or snapshot.exists():
        raise RuntimeError(f"formal snapshot and staging path must both be absent: {final_snapshot}, {snapshot}")
    snapshot.mkdir(parents=True, exist_ok=True)
    spawn_context = multiprocessing.get_context("spawn")
    physical_gpu = physical_gpu_gate()
    with concurrent.futures.ProcessPoolExecutor(max_workers=1, mp_context=spawn_context) as probe_pool:
        runtime = probe_pool.submit(runtime_fingerprint).result(timeout=120)
    runtime_sha256 = hashlib.sha256(canonical_json(runtime)).hexdigest()
    archive = archive_gate(args.archive, source)
    hf_metadata = hf_metadata_gate(args.hf_metadata)
    source_rows, source_summary = build_source_inventory(source)
    inventory_payload = b"".join(canonical_json(row) for row in source_rows)
    if SOURCE_INVENTORY.exists():
        if regular_file(SOURCE_INVENTORY).read_bytes() != inventory_payload:
            raise RuntimeError("pre-existing source inventory differs; replay refuses overwrite")
    else:
        write_jsonl(SOURCE_INVENTORY, source_rows)
    source_inventory_sha = sha256_file(SOURCE_INVENTORY)
    source_summary.update({
        "dataset_id": "Caoza/PhysX-Mobility",
        "hf_revision": EXPECTED_HF_REVISION,
        "license": "cc-by-nc-4.0",
        "archive": archive,
        "local_hf_metadata": hf_metadata,
        "source_inventory_sha256": source_inventory_sha,
    })
    summary_payload = json.dumps(source_summary, indent=2, sort_keys=True, ensure_ascii=True).encode() + b"\n"
    if SOURCE_SUMMARY.exists():
        if regular_file(SOURCE_SUMMARY).read_bytes() != summary_payload:
            raise RuntimeError("pre-existing source audit differs; replay refuses overwrite")
    else:
        write_json(SOURCE_SUMMARY, source_summary)
    (snapshot / "renders").mkdir(exist_ok=True)
    payloads = [(row, str(source), str(snapshot)) for row in source_rows]
    results = []
    with concurrent.futures.ProcessPoolExecutor(
        max_workers=args.workers, mp_context=spawn_context
    ) as pool:
        for index, result in enumerate(pool.map(render_one, payloads, chunksize=1), 1):
            results.append(result)
            if index % 25 == 0 or index == len(payloads):
                eligible = sum(row["status"] == "eligible" for row in results)
                print(f"[render] {index}/{len(payloads)} eligible={eligible}", flush=True)
    results.sort(key=lambda row: row["identity"])
    if {row.get("worker_runtime_sha256") for row in results} != {runtime_sha256}:
        raise RuntimeError("one or more render workers had a different runtime fingerprint")
    eligible = [row for row in results if row["status"] == "eligible"]
    ineligible = [row for row in results if row["status"] != "eligible"]
    if not eligible:
        raise RuntimeError("no renderable PhysX-Mobility candidates")
    eligible_snapshot = [
        {key: row[key] for key in ("status", "identity", "identity_sha256", "views")}
        for row in eligible
    ]
    ineligible_snapshot = [
        {key: row[key] for key in ("status", "identity", "identity_sha256", "reason")}
        for row in ineligible
    ]
    write_jsonl(snapshot / "candidate_inventory.jsonl", eligible_snapshot)
    write_jsonl(snapshot / "ineligible.jsonl", ineligible_snapshot)
    model_hashes = copy_model(snapshot)
    summary = {
        "status": "PASS",
        "source_assets": len(source_rows),
        "eligible_assets": len(eligible),
        "ineligible_assets": len(ineligible),
        "view_count": len(VIEWS),
        "resolution": [RESOLUTION, RESOLUTION],
    }
    write_json(snapshot / "candidate_summary.json", summary)
    locked = {
        "snapshot_type": "render_only_identity_snapshot",
        "status": "PASS",
        "method": "physx_mobility_geometry_clip_retrieval_v1",
        "is_generation_method": False,
        "dataset_id": "Caoza/PhysX-Mobility",
        "dataset_revision": EXPECTED_HF_REVISION,
        "license": "cc-by-nc-4.0",
        "archive": archive,
        "local_hf_metadata": hf_metadata,
        "source_inventory_sha256": source_inventory_sha,
        "source_summary_sha256": sha256_file(SOURCE_SUMMARY),
        "source_assets": len(source_rows),
        "eligible_assets": len(eligible),
        "ineligible_assets": len(ineligible),
        "render_policy": {
            "geometry_only": True,
            "geometry_vertices_unchanged": True,
            "source_textures_used": False,
            "source_json_used": False,
            "source_categories_used": False,
            "geometry_scale_policy": "raw source vertices and coordinates are unchanged",
            "camera_framing": "raw AABB center; half diagonal times 1.15 orthographic half-frame; camera radius half diagonal times 3.2; near/far derived from half diagonal",
            "views_azimuth_elevation_degrees": VIEWS,
            "projection": "orthographic xmag=ymag=raw AABB half diagonal times 1.15",
            "resolution": [RESOLUTION, RESOLUTION],
            "background_rgb": BACKGROUND,
            "base_color_rgba": BASE_COLOR,
            "ambient_light_rgb": [0.42, 0.42, 0.42],
            "key_light": {"type": "directional", "color_rgb": [1.0, 1.0, 1.0], "intensity": 2.4, "pose": "camera pose"},
            "fill_light": {"type": "directional", "color_rgb": [0.86, 0.91, 1.0], "intensity": 1.2, "azimuth_offset_deg": 135.0, "elevation_deg": 35.0},
            "workers": EXPECTED_WORKERS,
            "multiprocessing_start_method": "spawn",
            "png_compress_level": 6,
        },
        "prompt_data_accessed_by_snapshot_builder": False,
        "spec_data_accessed_by_snapshot_builder": False,
        "source_json_content_used": False,
        "finaljson_filename_existence_used_for_identity_closure": True,
        "candidate_inventory_sha256": sha256_file(snapshot / "candidate_inventory.jsonl"),
        "ineligible_sha256": sha256_file(snapshot / "ineligible.jsonl"),
        "candidate_summary_sha256": sha256_file(snapshot / "candidate_summary.json"),
        "model_files_sha256": model_hashes,
        "amendment_sha256": sha256_file(args.amendment),
        "independent_audit_sha256": protocol_binding["audit_sha256"],
        "snapshot_builder_sha256": sha256_file(SCRIPT),
        "runtime_fingerprint": runtime,
        "runtime_fingerprint_sha256": runtime_sha256,
        "thread_environment": EXPECTED_THREAD_ENV,
        "physical_gpu": physical_gpu,
    }
    write_json(snapshot / "snapshot.lock.json", locked)
    snapshot.replace(final_snapshot)
    print(json.dumps(summary, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
