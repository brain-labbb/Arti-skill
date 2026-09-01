#!/usr/bin/env python3
"""Render one opaque mobility candidate without reading benchmark metadata."""

from __future__ import annotations

import argparse
import ctypes
import hashlib
import importlib.metadata
import json
import math
import os
import stat
import subprocess
import sys
import zipfile
import zlib
from pathlib import Path

EXPECTED_ENVIRONMENT = {
    "PYOPENGL_PLATFORM": "egl",
    "EGL_DEVICE_ID": "1",
    "OPENBLAS_NUM_THREADS": "1",
    "OMP_NUM_THREADS": "1",
    "MKL_NUM_THREADS": "1",
    "NUMEXPR_NUM_THREADS": "1",
}
for _name, _value in EXPECTED_ENVIRONMENT.items():
    if os.environ.get(_name) not in (None, _value):
        raise RuntimeError(f"renderer environment drifted: {_name}={os.environ.get(_name)!r}")
    os.environ[_name] = _value

import numpy as np
from OpenGL import EGL, GL
from OpenGL.GL.EXT.memory_object import GL_DEVICE_UUID_EXT, glGetUnsignedBytevEXT
from PIL import Image, features as pillow_features
import pyrender
import trimesh


WORKSPACE = Path("/mnt/zsn/lyb").resolve()
VIEWS = 8
IMAGE_SIZE = 256
ELEVATION_DEGREES = 22.5
FRAME_HALF_DIAGONAL_MULTIPLIER = 1.15
CAMERA_RADIUS_HALF_DIAGONAL_MULTIPLIER = 3.2
ZNEAR_HALF_DIAGONAL_MULTIPLIER = 0.01
ZFAR_HALF_DIAGONAL_MULTIPLIER = 8.0


def safe(path: Path, *, must_exist: bool = True) -> Path:
    resolved = path.resolve(strict=must_exist)
    if resolved != WORKSPACE and WORKSPACE not in resolved.parents:
        raise RuntimeError(f"outside authorized workspace: {resolved}")
    return resolved


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with safe(path).open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def numeric_mesh(path: Path) -> trimesh.Trimesh:
    source = safe(path)
    if source.suffix != ".npz":
        raise RuntimeError("renderer accepts only label-free numeric NPZ geometry")
    with zipfile.ZipFile(source) as bundle:
        infos = bundle.infolist()
        if [info.filename for info in infos] != ["vertices.npy", "faces.npy"] or any(info.compress_type != zipfile.ZIP_STORED for info in infos):
            raise RuntimeError("numeric geometry NPZ member closure drifted")
    with np.load(source, allow_pickle=False) as payload:
        if payload.files != ["vertices", "faces"]:
            raise RuntimeError("numeric geometry NPZ array closure drifted")
        vertices = payload["vertices"]
        faces = payload["faces"]
    if vertices.dtype != np.dtype("<f8") or faces.dtype != np.dtype("<i8"):
        raise RuntimeError("numeric geometry NPZ dtype drifted")
    if vertices.ndim != 2 or vertices.shape[1:] != (3,) or faces.ndim != 2 or faces.shape[1:] != (3,):
        raise RuntimeError("numeric geometry NPZ shape drifted")
    if not np.isfinite(vertices).all() or not len(faces) or np.any(faces < 0) or np.any(faces >= len(vertices)):
        raise RuntimeError("invalid numeric geometry NPZ arrays")
    return trimesh.Trimesh(vertices=vertices, faces=faces, process=False, validate=False)


def load_geometry_scene(private_root: Path, path: Path, expected_sha256: str) -> tuple[trimesh.Trimesh, np.ndarray]:
    private_root = safe(private_root)
    path = safe(path)
    geometry_scene_root = safe(private_root / "geometry_scenes")
    if path.parent != geometry_scene_root or path.suffix != ".json":
        raise RuntimeError("geometry-only scene escaped frozen geometry_scenes closure")
    if path.is_symlink() or not stat.S_ISREG(path.lstat().st_mode) or sha256_file(path) != expected_sha256:
        raise RuntimeError("geometry-only scene identity mismatch")
    scene = json.loads(path.read_text(encoding="utf-8"))
    if set(scene) != {"schema_version", "camera_frame", "mesh_instances"} or scene["schema_version"] != 2:
        raise RuntimeError("geometry-only scene schema drifted")
    camera_frame = np.asarray(scene["camera_frame"], dtype=float)
    if camera_frame.shape != (3, 3) or not np.isfinite(camera_frame).all():
        raise RuntimeError("invalid camera-frame rotation")
    if not np.allclose(camera_frame.T @ camera_frame, np.eye(3), atol=1e-12, rtol=0.0) or not np.isclose(np.linalg.det(camera_frame), 1.0, atol=1e-12, rtol=0.0):
        raise RuntimeError("camera frame must be a proper rigid rotation")
    instances = scene["mesh_instances"]
    if not isinstance(instances, list) or len(instances) != 1:
        raise RuntimeError("geometry scene must bind exactly one merged numeric blob")
    meshes = []
    for instance in instances:
        if set(instance) != {"mesh_blob", "mesh_sha256", "transform"}:
            raise RuntimeError("geometry instance schema drifted")
        relative_blob = Path(instance["mesh_blob"])
        if relative_blob.is_absolute() or ".." in relative_blob.parts or len(relative_blob.parts) != 2 or relative_blob.parts[0] != "geometry_blobs" or relative_blob.suffix != ".npz":
            raise RuntimeError("unsafe opaque geometry blob path")
        mesh_path = safe(private_root / relative_blob)
        if private_root not in mesh_path.parents:
            raise RuntimeError("opaque geometry blob escaped private root")
        if mesh_path.is_symlink() or not mesh_path.is_file() or sha256_file(mesh_path) != instance["mesh_sha256"]:
            raise RuntimeError("bound mesh identity mismatch")
        transform = np.asarray(instance["transform"], dtype=float)
        if transform.shape != (4, 4) or not np.array_equal(transform, np.eye(4)):
            raise RuntimeError("merged numeric geometry transform must be identity")
        mesh = numeric_mesh(mesh_path)
        if len(mesh.faces):
            meshes.append(mesh)
    if not meshes:
        raise RuntimeError("no triangular geometry")
    combined = trimesh.util.concatenate(meshes)
    if not np.isfinite(combined.vertices).all() or not np.isfinite(combined.bounds).all() or np.any(combined.extents <= 0):
        raise RuntimeError("invalid geometry")
    return combined, camera_frame


def runtime_fingerprint() -> dict[str, str]:
    result = {}
    for name, enum in (("gl_vendor", GL.GL_VENDOR), ("gl_renderer", GL.GL_RENDERER), ("gl_version", GL.GL_VERSION), ("glsl_version", GL.GL_SHADING_LANGUAGE_VERSION)):
        value = GL.glGetString(enum)
        result[name] = value.decode("utf-8", errors="replace") if value else ""
    display = EGL.eglGetCurrentDisplay()
    for name, enum in (("egl_vendor", EGL.EGL_VENDOR), ("egl_version", EGL.EGL_VERSION), ("egl_client_apis", EGL.EGL_CLIENT_APIS)):
        value = EGL.eglQueryString(display, enum)
        result[name] = value.decode("utf-8", errors="replace") if value else ""
    gpu_query = subprocess.run(
        ["nvidia-smi", "-i", "1", "--query-gpu=uuid,name,driver_version,memory.total,compute_cap", "--format=csv,noheader,nounits"],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if gpu_query.returncode != 0:
        raise RuntimeError(f"cannot query renderer GPU identity: {gpu_query.stderr}")
    gpu_tokens = [token.strip() for token in gpu_query.stdout.strip().split(",")]
    if len(gpu_tokens) != 5:
        raise RuntimeError(f"invalid renderer GPU identity: {gpu_query.stdout!r}")
    uuid_buffer = (ctypes.c_ubyte * 16)()
    glGetUnsignedBytevEXT(GL_DEVICE_UUID_EXT, uuid_buffer)
    gl_device_uuid = bytes(uuid_buffer).hex()
    expected_gl_device_uuid = gpu_tokens[0].removeprefix("GPU-").replace("-", "").lower()
    if gl_device_uuid != expected_gl_device_uuid:
        raise RuntimeError(f"current GL context is not physical GPU 1: gl={gl_device_uuid} expected={expected_gl_device_uuid}")
    result.update({
        "egl_device_id": os.environ["EGL_DEVICE_ID"],
        "physical_gpu_uuid": gpu_tokens[0],
        "physical_gpu_name": gpu_tokens[1],
        "driver_version": gpu_tokens[2],
        "physical_gpu_memory_mib": gpu_tokens[3],
        "physical_gpu_compute_capability": gpu_tokens[4],
        "gl_device_uuid": gl_device_uuid,
        "pyopengl_platform": os.environ.get("PYOPENGL_PLATFORM", ""),
        "python": sys.version.split()[0],
        "openblas_num_threads": os.environ["OPENBLAS_NUM_THREADS"],
        "omp_num_threads": os.environ["OMP_NUM_THREADS"],
        "mkl_num_threads": os.environ["MKL_NUM_THREADS"],
        "numexpr_num_threads": os.environ["NUMEXPR_NUM_THREADS"],
        "numpy": np.__version__,
        "pillow": Image.__version__,
        "python_zlib_compile": zlib.ZLIB_VERSION,
        "python_zlib_runtime": zlib.ZLIB_RUNTIME_VERSION,
        "pillow_zlib": str(pillow_features.version("zlib")),
        "pillow_zlib_ng": str(pillow_features.version("zlib_ng")),
        "trimesh": trimesh.__version__,
        "pyrender": pyrender.__version__,
        "pyopengl": importlib.metadata.version("PyOpenGL"),
        "pyopengl_accelerate": importlib.metadata.version("PyOpenGL-accelerate"),
    })
    if not all(result.values()):
        raise RuntimeError(f"incomplete worker runtime fingerprint: {result}")
    return result


def camera_pose(center: np.ndarray, radius: float, azimuth: float) -> np.ndarray:
    elevation = math.radians(ELEVATION_DEGREES)
    eye = center + radius * np.array([
        math.cos(elevation) * math.cos(azimuth),
        math.cos(elevation) * math.sin(azimuth),
        math.sin(elevation),
    ])
    backward = (eye - center) / np.linalg.norm(eye - center)
    right = np.cross(np.array([0.0, 0.0, 1.0]), backward)
    right /= np.linalg.norm(right)
    up = np.cross(backward, right)
    result = np.eye(4)
    result[:3, 0], result[:3, 1], result[:3, 2], result[:3, 3] = right, up, backward, eye
    return result


def source_camera_pose(canonical_pose: np.ndarray, camera_frame: np.ndarray) -> np.ndarray:
    """Map a canonical camera-to-world pose into the source coordinate frame."""
    homogeneous = np.eye(4)
    homogeneous[:3, :3] = camera_frame
    return np.linalg.inv(homogeneous) @ canonical_pose


def assert_paired_upright_camera_invariant() -> None:
    """Fail if view-frame rotation changes geometry or its canonical silhouette."""
    source_vertices = np.array([
        [-0.2, -0.1, 0.0], [0.2, -0.1, 0.0], [0.2, 0.1, 0.0], [-0.2, 0.1, 0.0],
        [-0.2, -0.1, 1.0], [0.2, -0.1, 1.0], [0.2, 0.1, 1.0], [-0.2, 0.1, 1.0],
    ])
    rotations = (
        np.eye(3),
        np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]]),
    )
    canonical_pose = camera_pose(np.array([0.0, 0.0, 0.5]), 3.2, 0.0)
    for camera_frame in rotations:
        before = source_vertices.copy()
        source_pose = source_camera_pose(canonical_pose, camera_frame)
        if not np.array_equal(source_vertices, before):
            raise RuntimeError("camera-frame invariant modified score geometry")
        canonical_vertices = source_vertices @ camera_frame.T
        source_h = np.column_stack((source_vertices, np.ones(len(source_vertices))))
        canonical_h = np.column_stack((canonical_vertices, np.ones(len(canonical_vertices))))
        source_camera = source_h @ np.linalg.inv(source_pose).T
        canonical_camera = canonical_h @ np.linalg.inv(canonical_pose).T
        if not np.allclose(source_camera, canonical_camera, atol=1e-12, rtol=0.0):
            raise RuntimeError("camera-frame invariant changed canonical silhouette")


def render(private_root: Path, scene_path: Path, scene_sha256: str, candidate_key: str, output: Path) -> None:
    if len(candidate_key) != 64 or any(char not in "0123456789abcdef" for char in candidate_key):
        raise RuntimeError("candidate key must be lowercase SHA-256")
    assert_paired_upright_camera_invariant()
    output = safe(output, must_exist=False)
    if output.exists():
        raise RuntimeError(f"output exists: {output}")
    output.mkdir(parents=True)
    mesh, camera_frame = load_geometry_scene(private_root, scene_path, scene_sha256)
    canonical_vertices = np.asarray(mesh.vertices, dtype=float) @ camera_frame.T
    canonical_minimum = canonical_vertices.min(axis=0)
    canonical_maximum = canonical_vertices.max(axis=0)
    canonical_extents = canonical_maximum - canonical_minimum
    if not np.isfinite(canonical_extents).all() or np.any(canonical_extents <= 0):
        raise RuntimeError("invalid canonical camera bounds")
    half_diagonal = float(np.linalg.norm(canonical_extents)) * 0.5
    canonical_center = (canonical_minimum + canonical_maximum) * 0.5
    half_frame = max(half_diagonal * FRAME_HALF_DIAGONAL_MULTIPLIER, 1e-6)
    camera_radius = max(half_diagonal * CAMERA_RADIUS_HALF_DIAGONAL_MULTIPLIER, 1e-4)
    znear = max(half_diagonal * ZNEAR_HALF_DIAGONAL_MULTIPLIER, 1e-6)
    zfar = max(half_diagonal * ZFAR_HALF_DIAGONAL_MULTIPLIER, 1e-3)
    scene = pyrender.Scene(bg_color=np.ones(4), ambient_light=np.array([0.42, 0.42, 0.42]))
    material = pyrender.MetallicRoughnessMaterial(baseColorFactor=(0.56, 0.62, 0.68, 1.0), metallicFactor=0.0, roughnessFactor=0.78, doubleSided=True)
    scene.add(pyrender.Mesh.from_trimesh(mesh, material=material, smooth=False))
    camera = pyrender.OrthographicCamera(xmag=half_frame, ymag=half_frame, znear=znear, zfar=zfar)
    light = pyrender.DirectionalLight(color=np.ones(3), intensity=2.6)
    renderer = pyrender.OffscreenRenderer(IMAGE_SIZE, IMAGE_SIZE)
    hashes = {}
    try:
        fingerprint = runtime_fingerprint()
        for index in range(VIEWS):
            canonical_pose = camera_pose(canonical_center, camera_radius, 2 * math.pi * index / VIEWS)
            pose = source_camera_pose(canonical_pose, camera_frame)
            camera_node = scene.add(camera, pose=pose)
            light_node = scene.add(light, pose=pose)
            pixels, _ = renderer.render(scene, flags=pyrender.RenderFlags.RGBA)
            scene.remove_node(camera_node)
            scene.remove_node(light_node)
            path = output / f"{index:03d}.png"
            rgb = pixels[:, :, :3]
            foreground_pixels = int(np.count_nonzero(np.any(rgb < 250, axis=2)))
            if foreground_pixels < 32:
                raise RuntimeError(f"empty or near-empty render view: {index} foreground={foreground_pixels}")
            Image.fromarray(rgb, mode="RGB").save(path, compress_level=9)
            hashes[path.name] = sha256_file(path)
    finally:
        renderer.delete()
    record = {
        "candidate_key": candidate_key,
        "identity_sha256": candidate_key,
        "snapshot_render_dir": f"renders/{candidate_key}",
        "eligible": True,
        "views": VIEWS,
        "render_sha256": hashes,
        "runtime_fingerprint": fingerprint,
    }
    (output / "render.record.json").write_text(json.dumps(record, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--private-root", type=Path, required=True)
    parser.add_argument("--geometry-scene", type=Path, required=True)
    parser.add_argument("--geometry-scene-sha256", required=True)
    parser.add_argument("--candidate-key", required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    render(args.private_root, args.geometry_scene, args.geometry_scene_sha256, args.candidate_key, args.output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
