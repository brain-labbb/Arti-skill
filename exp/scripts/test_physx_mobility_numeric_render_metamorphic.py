#!/usr/bin/env python3
"""Prove PhysX path/OBJ labels cannot affect numeric blobs or shared renders."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any

import numpy as np


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
WORKSPACE = REPO.parent.resolve()
SOURCE_ADAPTER = REPO / "exp/scripts/audit_physx_mobility_table4_source.py"
WORKER = REPO / "exp/scripts/render_partnet_mobility_table4_opaque.py"
CANDIDATE_KEY = "a" * 64
CAMERA_FRAME = [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]]

FIXTURE_A = """o SECRET_DOOR
mtllib classified.mtl
v -1 -0.5 -0.3
v 1 -0.5 -0.3
v 1 0.5 -0.3
v -1 0.5 -0.3
v -1 -0.5 0.3
v 1 -0.5 0.3
v 1 0.5 0.3
v -1 0.5 0.3
usemtl SECRET
f 1 2 3 4
f 5 8 7 6
f 1 5 6 2
f 2 6 7 3
f 3 7 8 4
f 5 1 4 8
"""

FIXTURE_B = """# prompt-like comment
g RENAMED_CONTAINER
v -1 -0.5 -0.3
v 1 -0.5 -0.3
v 1 0.5 -0.3
v -1 0.5 -0.3
v -1 -0.5 0.3
v 1 -0.5 0.3
v 1 0.5 0.3
v -1 0.5 0.3
f 1/1 2/2 3/3 4/4
f 5/5 8/8 7/7 6/6
f 1/1 5/5 6/6 2/2
f 2/2 6/6 7/7 3/3
f 3/3 7/7 8/8 4/4
f 5/5 1/1 4/4 8/8
"""


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256_file(path: Path) -> str:
    return sha256_bytes(path.read_bytes())


def load_source_adapter() -> Any:
    spec = importlib.util.spec_from_file_location("physx_source_adapter", SOURCE_ADAPTER)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot import PhysX source adapter")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def source_item(adapter: Any, path: Path, member: str, transform: np.ndarray) -> dict[str, Any]:
    return {
        "source_path": str(path),
        "source_member_path": member,
        "source_member_path_sha256": adapter.sha256_text(member),
        "source_obj_sha256": sha256_file(path),
        "q0_transform": transform.tolist(),
        "q0_transform_sha256": sha256_bytes(transform.tobytes(order="C")),
    }


def build_fixture(adapter: Any, root: Path, path: Path, member: str) -> tuple[Path, dict[str, Any]]:
    (root / "geometry_blobs").mkdir(parents=True)
    (root / "geometry_scenes").mkdir()
    identity = np.ascontiguousarray(np.eye(4), dtype="<f8")
    instance, derivation = adapter.derive_candidate_geometry(
        CANDIDATE_KEY,
        [source_item(adapter, path, member, identity)],
        root / "geometry_blobs",
    )
    scene = root / "geometry_scenes" / f"{CANDIDATE_KEY}.json"
    adapter.write_json(scene, {
        "schema_version": 2,
        "camera_frame": CAMERA_FRAME,
        "mesh_instances": [instance],
    })
    return scene, derivation


def render(root: Path, scene: Path, output: Path) -> None:
    environment = dict(os.environ)
    environment.update({
        "PYOPENGL_PLATFORM": "egl",
        "EGL_DEVICE_ID": "1",
        "OPENBLAS_NUM_THREADS": "1",
        "OMP_NUM_THREADS": "1",
        "MKL_NUM_THREADS": "1",
        "NUMEXPR_NUM_THREADS": "1",
    })
    completed = subprocess.run(
        [
            sys.executable,
            str(WORKER),
            "--private-root", str(root),
            "--geometry-scene", str(scene),
            "--geometry-scene-sha256", sha256_file(scene),
            "--candidate-key", CANDIDATE_KEY,
            "--output-dir", str(output),
        ],
        cwd=str(REPO),
        env=environment,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=180,
        check=False,
    )
    if completed.returncode != 0:
        raise RuntimeError(f"shared worker failed ({completed.returncode}):\n{completed.stdout}")


def run() -> dict[str, Any]:
    adapter = load_source_adapter()
    with tempfile.TemporaryDirectory(prefix="physx-metamorphic-", dir=str(WORKSPACE)) as temporary:
        temporary_root = Path(temporary)
        first_obj = temporary_root / "secret_component.obj"
        second_obj = temporary_root / "renamed_prompt_like_component.obj"
        first_obj.write_text(FIXTURE_A, encoding="ascii")
        second_obj.write_text(FIXTURE_B, encoding="ascii")
        first_root = temporary_root / "private_first"
        second_root = temporary_root / "private_second"
        first_scene, first_derivation = build_fixture(
            adapter, first_root, first_obj, "partseg/SECRET/objs/secret_component.obj"
        )
        second_scene, second_derivation = build_fixture(
            adapter, second_root, second_obj, "partseg/RENAMED/objs/prompt_like_component.obj"
        )
        first_instance = first_derivation["ordered_source_instances"][0]
        second_instance = second_derivation["ordered_source_instances"][0]
        if first_instance["source_member_path_sha256"] == second_instance["source_member_path_sha256"]:
            raise RuntimeError("metamorphic source paths did not differ")
        if first_instance["source_obj_sha256"] == second_instance["source_obj_sha256"]:
            raise RuntimeError("metamorphic raw OBJ bytes did not differ")
        if first_instance["numeric_instance_order_sha256"] != second_instance["numeric_instance_order_sha256"]:
            raise RuntimeError("label/path edit changed numeric instance ordering key")
        first_blob = next((first_root / "geometry_blobs").iterdir())
        second_blob = next((second_root / "geometry_blobs").iterdir())
        if first_blob.read_bytes() != second_blob.read_bytes():
            raise RuntimeError("label/path edit changed derived numeric NPZ bytes")
        first_output = temporary_root / "render_first"
        second_output = temporary_root / "render_second"
        render(first_root, first_scene, first_output)
        render(second_root, second_scene, second_output)
        png_sha256: dict[str, str] = {}
        for index in range(8):
            name = f"{index:03d}.png"
            first = (first_output / name).read_bytes()
            second = (second_output / name).read_bytes()
            if first != second:
                raise RuntimeError(f"label/path edit changed rendered PNG bytes: {name}")
            png_sha256[name] = sha256_bytes(first)
        first_record_path = first_output / "render.record.json"
        second_record_path = second_output / "render.record.json"
        if first_record_path.read_bytes() != second_record_path.read_bytes():
            raise RuntimeError("label/path edit changed render record bytes")
        record = json.loads(first_record_path.read_text(encoding="utf-8"))
        return {
            "schema_version": 1,
            "status": "PASS",
            "test": "physx_path_obj_label_material_comment_numeric_npz_and_8view_png_metamorphic",
            "formal_output_written": False,
            "fixture_a_sha256": sha256_bytes(FIXTURE_A.encode("ascii")),
            "fixture_b_sha256": sha256_bytes(FIXTURE_B.encode("ascii")),
            "fixture_a_member": "partseg/SECRET/objs/secret_component.obj",
            "fixture_b_member": "partseg/RENAMED/objs/prompt_like_component.obj",
            "source_adapter_sha256": sha256_file(SOURCE_ADAPTER),
            "render_worker_sha256": sha256_file(WORKER),
            "metamorphic_test_sha256": sha256_file(SCRIPT),
            "raw_obj_sha256_different": True,
            "source_path_sha256_different": True,
            "numeric_instance_order_sha256": first_instance["numeric_instance_order_sha256"],
            "derived_npz_sha256": sha256_file(first_blob),
            "derived_npz_byte_identical": True,
            "eight_png_byte_identical": True,
            "png_sha256": png_sha256,
            "render_record_sha256": sha256_file(first_record_path),
            "runtime_fingerprint": record["runtime_fingerprint"],
        }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args()
    print(json.dumps(run(), indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
