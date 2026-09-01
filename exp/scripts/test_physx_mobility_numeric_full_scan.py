#!/usr/bin/env python3
"""Run the frozen numeric OBJ parser across the complete pinned PhysX cohort."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
from typing import Any


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
SOURCE_ADAPTER = REPO / "exp/scripts/audit_physx_mobility_table4_source.py"
DATASET = Path("/mnt/zsn/zsn_workspace/Ctrl-3D-trellis2-controlnet-dev/demo/physical_edit_demo/third_party/physx_mobility/extracted/PhysX_mobility")
ARCHIVE = DATASET.parents[1] / "PhysX-Mobility.zip"
EXPECTED_ARCHIVE_SHA256 = "88308cc2a4cc6177c59e32c2de51e881e6b961737295e5082d7ed01cca221908"
EXPECTED_OBJ_FILES = 91_898
EXPECTED_VERTICES = 18_391_114
EXPECTED_TRIANGULATED_FACES = 33_005_720


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_source_adapter() -> Any:
    spec = importlib.util.spec_from_file_location("physx_source_adapter", SOURCE_ADAPTER)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot import PhysX source adapter")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def run() -> dict[str, Any]:
    if sha256_file(ARCHIVE) != EXPECTED_ARCHIVE_SHA256:
        raise RuntimeError("pinned PhysX archive SHA drifted")
    adapter = load_source_adapter()
    paths = sorted(DATASET.glob("partseg/*/objs/*.obj"))
    if len(paths) != EXPECTED_OBJ_FILES:
        raise RuntimeError(f"PhysX OBJ file count drifted: {len(paths)}")
    vertices = 0
    triangulated_faces = 0
    for index, path in enumerate(paths, 1):
        vertex_array, face_array = adapter.parse_obj_numeric(path)
        vertices += len(vertex_array)
        triangulated_faces += len(face_array)
        if index % 10_000 == 0:
            print(f"[physx-numeric-scan] {index}/{len(paths)}", flush=True)
    if vertices != EXPECTED_VERTICES or triangulated_faces != EXPECTED_TRIANGULATED_FACES:
        raise RuntimeError(
            f"PhysX numeric parser totals drifted: vertices={vertices}, "
            f"triangulated_faces={triangulated_faces}"
        )
    return {
        "schema_version": 1,
        "status": "PASS",
        "test": "complete_pinned_physx_obj_numeric_vf_only_scan",
        "formal_output_written": False,
        "archive_sha256": EXPECTED_ARCHIVE_SHA256,
        "source_adapter_sha256": sha256_file(SOURCE_ADAPTER),
        "numeric_full_scan_test_sha256": sha256_file(SCRIPT),
        "obj_files": len(paths),
        "vertices": vertices,
        "triangulated_faces": triangulated_faces,
        "rules": {
            "read_records": ["v", "f"],
            "ignored_records_include": ["o", "g", "mtllib", "usemtl", "comments"],
            "homogeneous_vertices": "divide xyz by finite nonzero w",
            "polygons": "deterministic fan triangulation",
            "indices": "OBJ positive and negative vertex indices; zero/out-of-range rejected",
            "arrays": "finite nonempty little-endian float64 vertices and int64 triangular faces",
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args()
    print(json.dumps(run(), indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
