#!/usr/bin/env python3
"""Build two fresh deterministic mobility render cohorts from an opaque plan."""

from __future__ import annotations

import argparse
import concurrent.futures
import hashlib
import importlib.metadata
import json
import os
import shutil
import stat
import subprocess
import sys
import zipfile
import zlib
from pathlib import Path
from typing import Any

EXPECTED_RENDER_ENV = {
    "PYOPENGL_PLATFORM": "egl",
    "EGL_DEVICE_ID": "1",
    "OPENBLAS_NUM_THREADS": "1",
    "OMP_NUM_THREADS": "1",
    "MKL_NUM_THREADS": "1",
    "NUMEXPR_NUM_THREADS": "1",
}
for _name, _value in EXPECTED_RENDER_ENV.items():
    if os.environ.get(_name) not in (None, _value):
        raise RuntimeError(f"render builder environment drifted: {_name}={os.environ.get(_name)!r}")
    os.environ[_name] = _value

from OpenGL import GL
import numpy as np
from PIL import Image, features as pillow_features
import pyrender
import trimesh


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
WORKSPACE = REPO.parent.resolve()
WORKER = REPO / "exp/scripts/render_partnet_mobility_table4_opaque.py"
DEFAULT_PRIVATE_SOURCE = REPO / "exp/runtime/table4_constraints_v2/partnet_mobility_clip_retrieval_v1_private_source"
DEFAULT_AMENDMENT = REPO / "exp/reference/table4_constraints_v2/amendment_partnet_mobility_clip_retrieval_v1.json"
DEFAULT_FORMAL_ROOT = REPO / "exp/runtime/table4_constraints_v2/partnet_mobility_clip_retrieval_v1"
DEFAULT_OUTPUT = REPO / "exp/runtime/table4_constraints_v2/partnet_mobility_clip_retrieval_v1_render_build"
RUN_WORKERS = {"run1": 1, "run2": 4}
VIEWS = 8
EXPECTED_RUNTIME_FINGERPRINT = {
    "EGL_DEVICE_ID": "1",
    "PYOPENGL_PLATFORM": "egl",
    "numpy": "2.4.4",
    "pillow": "12.3.0",
    "trimesh": "4.12.2",
    "pyrender": "0.1.45",
    "pyopengl": "3.1.10",
    "pyopengl_accelerate": "3.1.10",
    "python": "3.13.2",
    "python_zlib_compile": "1.3.1",
    "python_zlib_runtime": "1.3.2",
    "pillow_zlib": "1.3.2",
    "pillow_zlib_ng": "2.3.3",
    "vendor": "NVIDIA Corporation",
    "renderer": "NVIDIA L20X/PCIe/SSE2",
    "version": "4.1.0 NVIDIA 570.172.08",
    "shading_language_version": "4.10 NVIDIA via Cg compiler",
}
EXPECTED_WORKER_FINGERPRINT = {
    "driver_version": "570.172.08",
    "egl_device_id": "1",
    "egl_client_apis": "OpenGL_ES OpenGL",
    "egl_vendor": "NVIDIA",
    "egl_version": "1.5",
    "gl_renderer": "NVIDIA L20X/PCIe/SSE2",
    "gl_vendor": "NVIDIA Corporation",
    "gl_version": "4.1.0 NVIDIA 570.172.08",
    "glsl_version": "4.10 NVIDIA via Cg compiler",
    "gl_device_uuid": "7390aec1d17766724136d998c85f489d",
    "numpy": "2.4.4",
    "openblas_num_threads": "1",
    "omp_num_threads": "1",
    "mkl_num_threads": "1",
    "numexpr_num_threads": "1",
    "pillow": "12.3.0",
    "python_zlib_compile": "1.3.1",
    "python_zlib_runtime": "1.3.2",
    "pillow_zlib": "1.3.2",
    "pillow_zlib_ng": "2.3.3",
    "pyopengl": "3.1.10",
    "pyopengl_accelerate": "3.1.10",
    "pyopengl_platform": "egl",
    "python": "3.13.2",
    "physical_gpu_name": "NVIDIA L20X",
    "physical_gpu_uuid": "GPU-7390aec1-d177-6672-4136-d998c85f489d",
    "physical_gpu_memory_mib": "143771",
    "physical_gpu_compute_capability": "8.9",
    "pyrender": "0.1.45",
    "trimesh": "4.12.2",
}


def safe(path: Path, *, must_exist: bool = True) -> Path:
    resolved = path.resolve(strict=must_exist)
    if resolved != WORKSPACE and WORKSPACE not in resolved.parents:
        raise RuntimeError(f"outside workspace: {resolved}")
    return resolved


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with safe(path).open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_json(path: Path) -> Any:
    return json.loads(safe(path).read_text(encoding="utf-8"))


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in safe(path).read_text(encoding="utf-8").splitlines() if line.strip()]


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


def flat_regular_files(directory: Path) -> dict[str, Path]:
    root = regular_dir(directory)
    files: dict[str, Path] = {}
    for entry in os.scandir(root):
        path = Path(entry.path)
        if entry.is_symlink() or not entry.is_file(follow_symlinks=False):
            raise RuntimeError(f"private geometry closure contains a non-regular file: {path}")
        files[entry.name] = path
    return files


def validate_private_source_closure(private_source: Path, plan: list[dict[str, Any]], source_lock: dict[str, Any]) -> dict[str, int]:
    root = regular_dir(private_source)
    expected_top = {
        "archive_member_manifest.jsonl",
        "geometry_blobs",
        "geometry_derivation_manifest.jsonl",
        "geometry_scenes",
        "render_plan.jsonl",
        "source_audit.lock.json",
        "source_binding.jsonl",
    }
    observed_top = {entry.name for entry in os.scandir(root)}
    if observed_top != expected_top:
        raise RuntimeError(f"private source top-level exact closure mismatch missing={sorted(expected_top - observed_top)} extra={sorted(observed_top - expected_top)}")
    for name in expected_top - {"geometry_blobs", "geometry_scenes"}:
        regular_file(root / name)
    if source_lock.get("render_plan_sha256") != sha256_file(root / "render_plan.jsonl"):
        raise RuntimeError("private source render-plan hash drifted")
    if source_lock.get("source_binding_sha256") != sha256_file(root / "source_binding.jsonl"):
        raise RuntimeError("private source binding hash drifted")
    if source_lock.get("geometry_derivation_manifest_sha256") != sha256_file(root / "geometry_derivation_manifest.jsonl"):
        raise RuntimeError("private geometry derivation manifest hash drifted")

    scene_files = flat_regular_files(root / "geometry_scenes")
    expected_scenes: set[str] = set()
    referenced_blobs: dict[str, str] = {}
    candidate_keys: set[str] = set()
    for row in plan:
        if set(row) != {"candidate_key", "geometry_scene", "geometry_scene_sha256"}:
            raise RuntimeError("private render-plan row schema drifted")
        candidate_key = row["candidate_key"]
        if not isinstance(candidate_key, str) or len(candidate_key) != 64 or any(char not in "0123456789abcdef" for char in candidate_key):
            raise RuntimeError("invalid opaque candidate key in private render plan")
        if candidate_key in candidate_keys:
            raise RuntimeError(f"duplicate opaque candidate key: {candidate_key}")
        candidate_keys.add(candidate_key)
        relative_scene = Path(row["geometry_scene"])
        expected_relative = Path("geometry_scenes") / f"{candidate_key}.json"
        if relative_scene != expected_relative or relative_scene.is_absolute() or ".." in relative_scene.parts:
            raise RuntimeError(f"noncanonical private geometry scene path: {relative_scene}")
        expected_scenes.add(relative_scene.name)
        scene_path = scene_files.get(relative_scene.name)
        if scene_path is None or sha256_file(scene_path) != row["geometry_scene_sha256"]:
            raise RuntimeError(f"private geometry scene hash drifted: {candidate_key}")
        scene = read_json(scene_path)
        if set(scene) != {"schema_version", "camera_frame", "mesh_instances"} or scene.get("schema_version") != 2:
            raise RuntimeError(f"private geometry scene schema drifted: {candidate_key}")
        camera_frame = np.asarray(scene["camera_frame"], dtype=float)
        if camera_frame.shape != (3, 3) or not np.isfinite(camera_frame).all() or not np.allclose(camera_frame.T @ camera_frame, np.eye(3), atol=1e-12, rtol=0.0) or not np.isclose(np.linalg.det(camera_frame), 1.0, atol=1e-12, rtol=0.0):
            raise RuntimeError(f"private camera frame is not a proper rotation: {candidate_key}")
        instances = scene["mesh_instances"]
        if not isinstance(instances, list) or not instances:
            raise RuntimeError(f"private geometry scene is empty: {candidate_key}")
        for instance in instances:
            if set(instance) != {"mesh_blob", "mesh_sha256", "transform"}:
                raise RuntimeError(f"private mesh-instance schema drifted: {candidate_key}")
            blob = Path(instance["mesh_blob"])
            mesh_hash = instance["mesh_sha256"]
            if blob.is_absolute() or ".." in blob.parts or len(blob.parts) != 2 or blob.parts[0] != "geometry_blobs":
                raise RuntimeError(f"unsafe private geometry blob path: {blob}")
            if not isinstance(mesh_hash, str) or len(mesh_hash) != 64 or any(char not in "0123456789abcdef" for char in mesh_hash) or blob.stem != mesh_hash:
                raise RuntimeError(f"private geometry blob/hash identity drifted: {blob}")
            if blob.suffix != ".npz":
                raise RuntimeError(f"private geometry blob is not numeric NPZ: {blob}")
            transform = np.asarray(instance["transform"], dtype=float)
            if transform.shape != (4, 4) or not np.array_equal(transform, np.eye(4)):
                raise RuntimeError(f"invalid private geometry transform: {candidate_key}")
            previous = referenced_blobs.setdefault(blob.name, mesh_hash)
            if previous != mesh_hash:
                raise RuntimeError(f"conflicting private geometry blob hash: {blob.name}")
    if set(scene_files) != expected_scenes:
        raise RuntimeError(f"geometry scene exact closure mismatch missing={sorted(expected_scenes - set(scene_files))} extra={sorted(set(scene_files) - expected_scenes)}")
    blob_files = flat_regular_files(root / "geometry_blobs")
    if set(blob_files) != set(referenced_blobs):
        raise RuntimeError(f"geometry blob exact closure mismatch missing={sorted(set(referenced_blobs) - set(blob_files))} extra={sorted(set(blob_files) - set(referenced_blobs))}")
    for name, expected_hash in referenced_blobs.items():
        if sha256_file(blob_files[name]) != expected_hash:
            raise RuntimeError(f"private geometry blob hash drifted: {name}")
        with zipfile.ZipFile(blob_files[name]) as bundle:
            infos = bundle.infolist()
            if [info.filename for info in infos] != ["vertices.npy", "faces.npy"] or any(info.compress_type != zipfile.ZIP_STORED for info in infos):
                raise RuntimeError(f"numeric geometry NPZ closure drifted: {name}")
        with np.load(blob_files[name], allow_pickle=False) as payload:
            if payload.files != ["vertices", "faces"]:
                raise RuntimeError(f"numeric geometry array closure drifted: {name}")
            vertices, faces = payload["vertices"], payload["faces"]
        if vertices.dtype != np.dtype("<f8") or faces.dtype != np.dtype("<i8") or vertices.ndim != 2 or vertices.shape[1:] != (3,) or faces.ndim != 2 or faces.shape[1:] != (3,):
            raise RuntimeError(f"numeric geometry dtype/shape drifted: {name}")
        if not np.isfinite(vertices).all() or not len(faces) or np.any(faces < 0) or np.any(faces >= len(vertices)):
            raise RuntimeError(f"invalid numeric geometry arrays: {name}")
    if source_lock.get("geometry_scene_count") != len(expected_scenes) or source_lock.get("geometry_blob_count") != len(referenced_blobs):
        raise RuntimeError("private source geometry closure counts drifted")
    return {"geometry_scene_count": len(expected_scenes), "geometry_blob_count": len(referenced_blobs)}


def write_json(path: Path, value: Any) -> None:
    destination = safe(path, must_exist=False)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True) + "\n", encoding="utf-8")


def write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    destination = safe(path, must_exist=False)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text("".join(json.dumps(row, sort_keys=True, ensure_ascii=True) + "\n" for row in rows), encoding="utf-8")


def gl_fingerprint() -> dict[str, Any]:
    renderer = pyrender.OffscreenRenderer(8, 8)
    try:
        values = {}
        for name, enum in (("vendor", GL.GL_VENDOR), ("renderer", GL.GL_RENDERER), ("version", GL.GL_VERSION), ("shading_language_version", GL.GL_SHADING_LANGUAGE_VERSION)):
            raw = GL.glGetString(enum)
            values[name] = raw.decode("utf-8", errors="replace") if raw else None
    finally:
        renderer.delete()
    values.update({
        "EGL_DEVICE_ID": os.environ.get("EGL_DEVICE_ID"),
        "PYOPENGL_PLATFORM": os.environ.get("PYOPENGL_PLATFORM"),
        "python": sys.version.split()[0],
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
    if not all(values.values()):
        raise RuntimeError(f"incomplete OpenGL fingerprint: {values}")
    return values


def render_one(private_source: Path, scene_path: Path, scene_sha256: str, candidate_key: str, destination: str) -> dict[str, Any]:
    result = subprocess.run(
        [sys.executable, str(safe(WORKER)), "--private-root", str(safe(private_source)), "--geometry-scene", str(safe(scene_path)), "--geometry-scene-sha256", scene_sha256, "--candidate-key", candidate_key, "--output-dir", destination],
        cwd=str(safe(REPO)), text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=False,
        env={**os.environ, "PYTHONHASHSEED": "0", **EXPECTED_RENDER_ENV},
    )
    return {"candidate_key": candidate_key, "returncode": result.returncode, "log": result.stdout}


def render_run(private_source: Path, run_root: Path, plan: list[dict[str, Any]], workers: int) -> list[dict[str, Any]]:
    run_root.mkdir(parents=True, exist_ok=False)
    results = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=workers) as pool:
        futures = {}
        for row in plan:
            destination = run_root / row["candidate_key"]
            relative_scene = Path(row["geometry_scene"])
            if relative_scene.is_absolute() or ".." in relative_scene.parts or relative_scene.parts[:1] != ("geometry_scenes",):
                raise RuntimeError(f"unsafe geometry scene binding: {relative_scene}")
            future = pool.submit(render_one, private_source, private_source / relative_scene, row["geometry_scene_sha256"], row["candidate_key"], str(destination))
            futures[future] = row["candidate_key"]
        for completed, future in enumerate(concurrent.futures.as_completed(futures), 1):
            result = future.result()
            results.append(result)
            if result["returncode"] != 0:
                raise RuntimeError(f"render failed {result['candidate_key']}: {result['log'][-1000:]}")
            if completed % 25 == 0 or completed == len(plan):
                print(f"[render-snapshot] workers={workers} {completed}/{len(plan)}", flush=True)
    return sorted(results, key=lambda row: row["candidate_key"])


def record(path: Path) -> dict[str, Any]:
    row = read_json(path / "render.record.json")
    expected_names = [f"{index:03d}.png" for index in range(VIEWS)] + ["render.record.json"]
    if sorted(item.name for item in path.iterdir()) != sorted(expected_names):
        raise RuntimeError(f"render closure drifted: {path}")
    for name, expected in row["render_sha256"].items():
        if sha256_file(path / name) != expected:
            raise RuntimeError(f"render hash drifted: {path}/{name}")
    return row


def configured_path(value: str) -> Path:
    path = Path(value)
    return safe(path if path.is_absolute() else REPO / path, must_exist=False)


def build(private_source: Path, amendment_path: Path, formal_root: Path, output: Path, expected_assets: int, limit: int) -> None:
    private_source = safe(private_source)
    amendment_path = safe(amendment_path)
    formal_root = safe(formal_root)
    output = safe(output, must_exist=False)
    if output.exists():
        raise RuntimeError(f"output exists: {output}")
    source_lock = read_json(private_source / "source_audit.lock.json")
    amendment = read_json(amendment_path)
    shared = amendment.get("shared_pipeline", {})
    expected_configuration = {
        "private_source": private_source,
        "render_build": output,
        "formal_root": formal_root,
    }
    for field, observed in expected_configuration.items():
        if configured_path(shared.get(field, "")) != observed:
            raise RuntimeError(f"amendment shared pipeline path drifted: {field}")
    if shared.get("expected_assets") != expected_assets or shared.get("run_workers") != RUN_WORKERS:
        raise RuntimeError("amendment shared pipeline cardinality/worker policy drifted")
    if configured_path(shared.get("render_worker", "")) != safe(WORKER) or configured_path(shared.get("render_builder", "")) != safe(SCRIPT):
        raise RuntimeError("amendment shared renderer implementation path drifted")
    audit = read_json(formal_root / "protocol_audit_pre_result.json")
    if amendment.get("implementation", {}).get("render_builder_sha256") != sha256_file(SCRIPT) or amendment.get("implementation", {}).get("render_worker_sha256") != sha256_file(WORKER):
        raise RuntimeError("amendment render implementation binding drifted")
    if audit.get("protocol_ready") is not True or audit.get("amendment_sha256") != sha256_file(amendment_path) or audit.get("render_builder_sha256") != sha256_file(SCRIPT) or audit.get("render_worker_sha256") != sha256_file(WORKER):
        raise RuntimeError("independent pre-result render audit binding drifted")
    if audit.get("report_sha256") != sha256_file(formal_root / "report.md"):
        raise RuntimeError("independent pre-result audit report drifted")
    plan = read_jsonl(private_source / "render_plan.jsonl")
    if source_lock.get("status") != "PASS" or len(plan) != expected_assets:
        raise RuntimeError("private source audit is not complete")
    if source_lock.get("render_plan_sha256") != sha256_file(private_source / "render_plan.jsonl"):
        raise RuntimeError("render plan hash drifted")
    if plan != sorted(plan, key=lambda row: row["candidate_key"]):
        raise RuntimeError("render plan order drifted")
    validate_private_source_closure(private_source, plan, source_lock)
    formal = limit == 0
    if limit:
        plan = plan[:limit]
    output.mkdir(parents=True, exist_ok=False)
    try:
        fingerprint_before = gl_fingerprint()
        if fingerprint_before != EXPECTED_RUNTIME_FINGERPRINT:
            raise RuntimeError(f"pre-frozen parent renderer fingerprint drifted: {fingerprint_before}")
        run1, run2 = output / "run1", output / "run2"
        render_run(private_source, run1, plan, RUN_WORKERS["run1"])
        render_run(private_source, run2, plan, RUN_WORKERS["run2"])
        inventory = []
        worker_fingerprint = None
        for row in plan:
            key = row["candidate_key"]
            first, second = record(run1 / key), record(run2 / key)
            if first != second:
                raise RuntimeError(f"cross-worker fresh render replay differs: {key}")
            observed = first["runtime_fingerprint"]
            if second["runtime_fingerprint"] != observed:
                raise RuntimeError(f"run1/run2 worker runtime fingerprint differs: {key}")
            if worker_fingerprint is None:
                worker_fingerprint = observed
            elif worker_fingerprint != observed:
                raise RuntimeError(f"worker runtime fingerprint differs: {key}")
            first = {name: value for name, value in first.items() if name != "runtime_fingerprint"}
            inventory.append(first)
        fingerprint_after = gl_fingerprint()
        if fingerprint_before != fingerprint_after:
            raise RuntimeError("renderer runtime fingerprint drifted during full replay")
        if worker_fingerprint != EXPECTED_WORKER_FINGERPRINT:
            raise RuntimeError(f"pre-frozen worker renderer fingerprint drifted: {worker_fingerprint}")
        write_jsonl(output / "candidate_inventory.jsonl", inventory)
        lock = {
            "schema_version": 1,
            "status": "PASS",
            "formal": formal,
            "candidate_assets": len(inventory),
            "eligible_assets": len(inventory),
            "ineligible_assets": 0,
            "task_order": "ascending opaque candidate_key",
            "fresh_render_runs": 2,
            "run_workers": RUN_WORKERS,
            "cross_worker_full_render_replay_byte_identical": True,
            "views_per_asset": VIEWS,
            "image_size": [256, 256],
            "renderer_policy": "private source audit emits zero-pose geometry-only scene plus a frozen source-to-canonical camera rotation; worker leaves mesh geometry unchanged and uses the inverse rotation only for camera/light poses; canonical +Z camera convention, eight 45-degree azimuths, elevation 22.5 degrees, orthographic half-frame 1.15 times AABB half-diagonal, camera radius 3.2 times half-diagonal, znear 0.01 and zfar 8.0 times half-diagonal, fixed material/light/white background",
            "renderer_forbidden_inputs": "worker contains and reads no URDF, link/joint/visual names, benchmark prompt/protocol/spec, meta.json, semantics.txt, category, target dimension, or prior score",
            "private_source_audit_lock_sha256": sha256_file(private_source / "source_audit.lock.json"),
            "render_plan_sha256": source_lock["render_plan_sha256"],
            "source_binding_sha256": source_lock["source_binding_sha256"],
            "source_auditor_sha256": source_lock["source_auditor_sha256"],
            "render_worker_sha256": sha256_file(WORKER),
            "snapshot_builder_sha256": sha256_file(SCRIPT),
            "candidate_inventory_sha256": sha256_file(output / "candidate_inventory.jsonl"),
            "runtime_fingerprint": fingerprint_before,
            "all_worker_runtime_fingerprint": worker_fingerprint,
        }
        if formal and len(inventory) != expected_assets:
            raise RuntimeError("formal render cohort incomplete")
        write_json(output / "render_build.lock.json", lock)
        print(json.dumps({"status": "RENDER_BUILD_LOCKED", "formal": formal, "candidates": len(inventory), "lock_sha256": sha256_file(output / "render_build.lock.json")}, indent=2), flush=True)
    except Exception:
        shutil.rmtree(output, ignore_errors=True)
        raise


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--private-source", type=Path, default=DEFAULT_PRIVATE_SOURCE)
    parser.add_argument("--amendment", type=Path, default=DEFAULT_AMENDMENT)
    parser.add_argument("--formal-root", type=Path, default=DEFAULT_FORMAL_ROOT)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--expected-assets", type=int, default=2347)
    parser.add_argument("--limit", type=int, default=0)
    args = parser.parse_args()
    if args.limit < 0:
        raise ValueError("limit must be nonnegative")
    if args.expected_assets <= 0:
        raise ValueError("expected assets must be positive")
    build(args.private_source, args.amendment, args.formal_root, args.output_dir, args.expected_assets, args.limit)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
