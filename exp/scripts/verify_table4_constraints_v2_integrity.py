#!/usr/bin/env python3
"""Verify the immutable inputs and artifact chain for Table 4 Constraints v2."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

import numpy as np
import trimesh


WORKSPACE = Path("/mnt/zsn/lyb").resolve()
EXP_ROOT = Path(__file__).resolve().parents[1]
REFERENCE = EXP_ROOT / "reference/table4_constraints_v2"
PROTOCOL = REFERENCE / "protocol.json"
PROMPTS = REFERENCE / "prompts.jsonl"
HASHED_SCRIPTS = (
    EXP_ROOT / "scripts/prepare_table4_constraints_v2.py",
    EXP_ROOT / "scripts/canonicalize_table4_artifact.py",
    EXP_ROOT / "scripts/score_table4_constraints_v2.py",
)


def contained(path: Path) -> Path:
    resolved = path.resolve()
    if resolved != WORKSPACE and WORKSPACE not in resolved.parents:
        raise ValueError(f"path escapes workspace: {resolved}")
    return resolved


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with contained(path).open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_json(path: Path) -> Any:
    return json.loads(contained(path).read_text(encoding="utf-8"))


def load_manifest(path: Path) -> list[dict[str, Any]]:
    rows = []
    for line_number, line in enumerate(contained(path).read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        row = json.loads(line)
        if not isinstance(row, dict):
            raise ValueError(f"manifest line {line_number} is not an object")
        rows.append(row)
    return rows


def parse_manifest(value: str) -> tuple[str, Path]:
    if "=" not in value:
        raise argparse.ArgumentTypeError("expected METHOD=MANIFEST_JSONL")
    method, raw_path = value.split("=", 1)
    if not method or not raw_path:
        raise argparse.ArgumentTypeError("expected METHOD=MANIFEST_JSONL")
    return method, contained(Path(raw_path))


def scene_extents(path: Path) -> np.ndarray:
    scene = trimesh.load(contained(path), force="scene", process=False)
    if not isinstance(scene, trimesh.Scene) or not scene.geometry:
        raise ValueError("canonical GLB has no scene geometry")
    bounds = np.asarray(scene.bounds, dtype=float)
    if bounds.shape != (2, 3) or not np.isfinite(bounds).all():
        raise ValueError("canonical GLB bounds are invalid")
    return bounds[1] - bounds[0]


def verify_method(
    method: str,
    manifest_path: Path,
    expected_tasks: set[str],
) -> dict[str, Any]:
    rows = load_manifest(manifest_path)
    errors: list[str] = []
    task_ids = [str(row.get("task_id") or "") for row in rows]
    if len(rows) != len(expected_tasks):
        errors.append(f"row_count={len(rows)} expected={len(expected_tasks)}")
    if len(set(task_ids)) != len(task_ids):
        errors.append("duplicate task_id")
    if set(task_ids) != expected_tasks:
        errors.append(
            f"task_set mismatch missing={sorted(expected_tasks - set(task_ids))} "
            f"extra={sorted(set(task_ids) - expected_tasks)}"
        )
    success = 0
    artifact_checks = []
    for row in rows:
        task_id = str(row.get("task_id") or "")
        row_method = row.get("method")
        if row_method is not None and row_method != method:
            errors.append(f"{task_id}: row method {row_method!r} != {method!r}")
        if row.get("status") != "success":
            continue
        success += 1
        task_errors = []
        try:
            canonical_dir = contained(Path(row["canonical_dir"]))
            artifact_path = canonical_dir / "artifact.json"
            nodes_path = canonical_dir / "semantic_nodes.json"
            artifact = load_json(artifact_path)
            nodes = load_json(nodes_path)
            if not isinstance(nodes, list):
                raise ValueError("semantic_nodes.json is not a list")
            canonical_glb = contained(Path(artifact["canonical_glb"]))
            if canonical_glb.parent != canonical_dir:
                raise ValueError("artifact canonical_glb is outside its canonical_dir")
            actual_glb_hash = sha256(canonical_glb)
            if actual_glb_hash != artifact.get("canonical_glb_sha256"):
                task_errors.append("canonical GLB hash mismatch")
            if len(nodes) != artifact.get("semantic_node_count"):
                task_errors.append("semantic node count mismatch")
            extents = scene_extents(canonical_glb)
            stored = np.asarray(artifact.get("extents_m"), dtype=float)
            if stored.shape != (3,) or not np.allclose(extents, stored, atol=1e-6, rtol=1e-6):
                task_errors.append(
                    f"GLB AABB mismatch observed={extents.tolist()} stored={stored.tolist()}"
                )
            source_value = row.get("source_artifact")
            source_hash = row.get("source_artifact_sha256")
            if source_value and source_hash and sha256(Path(source_value)) != source_hash:
                task_errors.append("source artifact hash mismatch")
            artifact_checks.append({
                "task_id": task_id,
                "canonical_glb_sha256": actual_glb_hash,
                "semantic_node_count": len(nodes),
                "extents_m": extents.tolist(),
                "errors": task_errors,
            })
        except Exception as exc:  # fail closed per task
            task_errors.append(f"{type(exc).__name__}: {exc}")
            artifact_checks.append({"task_id": task_id, "errors": task_errors})
        errors.extend(f"{task_id}: {error}" for error in task_errors)
    return {
        "method": method,
        "manifest": str(manifest_path),
        "manifest_sha256": sha256(manifest_path),
        "row_count": len(rows),
        "success_count": success,
        "artifact_checks": artifact_checks,
        "errors": errors,
        "passed": not errors,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", action="append", type=parse_manifest, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    protocol = load_json(PROTOCOL)
    errors: list[str] = []
    actual_prompt_hash = sha256(PROMPTS)
    if actual_prompt_hash != protocol.get("prompt_manifest_sha256"):
        errors.append("prompt manifest hash mismatch")
    expected_tasks = set(protocol.get("spec_sha256") or {})
    if len(expected_tasks) != protocol.get("task_count"):
        errors.append("protocol task_count/spec set mismatch")
    spec_checks = []
    constraint_count = 0
    for task_id in sorted(expected_tasks):
        path = REFERENCE / "specs" / f"{task_id}.json"
        actual_hash = sha256(path)
        expected_hash = protocol["spec_sha256"][task_id]
        spec = load_json(path)
        constraint_count += len(spec.get("constraints") or [])
        passed = actual_hash == expected_hash
        spec_checks.append({
            "task_id": task_id,
            "sha256": actual_hash,
            "expected_sha256": expected_hash,
            "passed": passed,
        })
        if not passed:
            errors.append(f"spec hash mismatch: {task_id}")
    if constraint_count != protocol.get("constraint_count"):
        errors.append(
            f"constraint_count={constraint_count} expected={protocol.get('constraint_count')}"
        )
    seen_methods = set()
    methods = []
    for method, manifest in args.manifest:
        if method in seen_methods:
            errors.append(f"duplicate method argument: {method}")
            continue
        seen_methods.add(method)
        result = verify_method(method, manifest, expected_tasks)
        methods.append(result)
        errors.extend(f"{method}: {error}" for error in result["errors"])
    payload = {
        "schema_version": 1,
        "benchmark_id": protocol.get("benchmark_id"),
        "passed": not errors,
        "protocol": {
            "path": str(PROTOCOL),
            "sha256": sha256(PROTOCOL),
            "prompt_manifest_path": str(PROMPTS),
            "prompt_manifest_sha256": actual_prompt_hash,
            "expected_prompt_manifest_sha256": protocol.get("prompt_manifest_sha256"),
            "task_count": len(expected_tasks),
            "constraint_count": constraint_count,
            "spec_checks": spec_checks,
        },
        "implementation_sha256": {str(path): sha256(path) for path in HASHED_SCRIPTS},
        "methods": methods,
        "errors": errors,
    }
    output = contained(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps({
        "passed": payload["passed"],
        "methods": len(methods),
        "artifacts": sum(row["success_count"] for row in methods),
        "errors": len(errors),
        "output": str(output),
    }, indent=2))
    return 0 if payload["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
