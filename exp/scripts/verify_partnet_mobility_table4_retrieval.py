#!/usr/bin/env python3
"""Rebuild integrity evidence for the PartNet-Mobility Table 4 retrieval control."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
from typing import Any

import numpy as np
import trimesh


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
WORKSPACE = REPO.parent.resolve()
REFERENCE = REPO / "exp/reference/table4_constraints_v2"
PROMPTS = REFERENCE / "prompts.jsonl"
PROTOCOL = REFERENCE / "protocol.json"
AMENDMENT = REFERENCE / "amendment_partnet_mobility_clip_retrieval_v1.json"
CANONICALIZER = REPO / "exp/scripts/canonicalize_table4_artifact.py"
COUNT_SCORER = REPO / "exp/scripts/score_table4_constraints_v2.py"
GENERIC_INTEGRITY = REPO / "exp/scripts/verify_table4_constraints_v2_integrity.py"
PRIVATE_SOURCE = REPO / "exp/runtime/table4_constraints_v2/partnet_mobility_clip_retrieval_v1_private_source"
SNAPSHOT = REPO / "exp/runtime/table4_constraints_v2/partnet_mobility_clip_retrieval_v1_snapshot"
DEFAULT_ROOT = REPO / "exp/runtime/table4_constraints_v2/partnet_mobility_clip_retrieval_v1"
EXPECTED_PROMPTS_SHA256 = "0b46c15c65c46550cc25654ccca10e4de87f1b97e6bde89796ea3cc9b439065e"
EXPECTED_PROTOCOL_SHA256 = "6857194072ccb0ba3943d14a62f29c2364920ec01381a5475311535c1831031f"


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


def read_json(path: Path) -> Any:
    return json.loads(safe(path).read_text(encoding="utf-8"))


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in safe(path).read_text(encoding="utf-8").splitlines() if line.strip()]


def verify(root: Path) -> dict[str, Any]:
    root = safe(root)
    errors: list[str] = []
    if sha256_file(PROMPTS) != EXPECTED_PROMPTS_SHA256:
        errors.append("prompt manifest hash mismatch")
    if sha256_file(PROTOCOL) != EXPECTED_PROTOCOL_SHA256:
        errors.append("protocol hash mismatch")
    amendment = read_json(AMENDMENT)
    tooling_expected = {
        "canonicalizer_sha256": sha256_file(CANONICALIZER),
        "count_scorer_sha256": sha256_file(COUNT_SCORER),
        "generic_integrity_verifier_sha256": sha256_file(GENERIC_INTEGRITY),
    }
    for field, expected in tooling_expected.items():
        if amendment.get("implementation", {}).get(field) != expected:
            errors.append(f"amendment shared tooling binding mismatch: {field}")
    selection_lock = read_json(root / "selection.lock.json")
    materialization_lock = read_json(root / "materialization.lock.json")
    source_lock = read_json(PRIVATE_SOURCE / "source_audit.lock.json")
    snapshot_lock = read_json(SNAPSHOT / "snapshot.lock.json")
    if selection_lock.get("amendment_sha256") != sha256_file(AMENDMENT):
        errors.append("selection amendment hash mismatch")
    for name, expected in selection_lock.get("locked_file_sha256", {}).items():
        if sha256_file(root / name) != expected:
            errors.append(f"selection locked file mismatch: {name}")
    if selection_lock.get("private_source_audit_lock_sha256") != sha256_file(PRIVATE_SOURCE / "source_audit.lock.json"):
        errors.append("private source audit lock mismatch")
    if selection_lock.get("snapshot_lock_sha256") != sha256_file(SNAPSHOT / "snapshot.lock.json"):
        errors.append("opaque snapshot lock mismatch")
    if snapshot_lock.get("source_binding_sha256") != source_lock.get("source_binding_sha256"):
        errors.append("opaque snapshot/source binding hash mismatch")
    inventory = read_jsonl(SNAPSHOT / "candidate_inventory.jsonl")
    if len(inventory) != 2347 or sha256_file(SNAPSHOT / "candidate_inventory.jsonl") != snapshot_lock.get("candidate_inventory_sha256"):
        errors.append("opaque inventory closure mismatch")
    selection = read_jsonl(root / "selection.jsonl")
    replay = read_jsonl(root / "selection_replay.jsonl")
    index = read_jsonl(root / "embedding_index.jsonl")
    assets1 = np.load(root / "asset_embeddings_run1.npy", allow_pickle=False)
    assets2 = np.load(root / "asset_embeddings_run2.npy", allow_pickle=False)
    text1 = np.load(root / "prompt_embeddings_run1.npy", allow_pickle=False)
    text2 = np.load(root / "prompt_embeddings_run2.npy", allow_pickle=False)
    if not np.array_equal(assets1, assets2) or not np.array_equal(text1, text2) or selection != replay:
        errors.append("embedding/selection replay differs")
    if assets1.shape != (2347, 512) or text1.shape != (18, 512):
        errors.append("embedding shape mismatch")
    similarities = text1 @ assets1.T
    for prompt_offset, row in enumerate(selection):
        expected = min(range(len(index)), key=lambda offset: (-float(similarities[prompt_offset, offset]), index[offset]["candidate_key"]))
        if row.get("selected_embedding_row") != expected or row.get("selected_candidate_key") != index[expected]["candidate_key"]:
            errors.append(f"non-top1 selection: {row.get('task_id')}")
    manifest = read_jsonl(root / "artifact_manifest.jsonl")
    if len(manifest) != 18 or [row.get("task_id") for row in manifest] != [f"T4C{number:03d}" for number in range(1, 19)]:
        errors.append("artifact manifest task closure mismatch")
    selection_by_task = {row["task_id"]: row for row in selection}
    artifact_checks = []
    for row in manifest:
        locked_selection = selection_by_task.get(row.get("task_id"), {})
        if row.get("selection_candidate_key") != locked_selection.get("selected_candidate_key"):
            errors.append(f"selected candidate binding mismatch: {row.get('task_id')}")
        if row.get("fallback_used") is not False or row.get("repair_attempts") != 0:
            errors.append(f"repair/fallback policy mismatch: {row.get('task_id')}")
        if row.get("status") != "success":
            artifact_checks.append({"task_id": row.get("task_id"), "status": "failure"})
            continue
        directory = safe(Path(row["canonical_dir"]))
        artifact = read_json(directory / "artifact.json")
        nodes = read_json(directory / "semantic_nodes.json")
        glb = safe(Path(artifact["canonical_glb"]))
        loaded = trimesh.load(glb, force="scene", process=False)
        scene = loaded if isinstance(loaded, trimesh.Scene) else trimesh.Scene(loaded)
        extents = np.asarray(scene.bounds[1] - scene.bounds[0], dtype=float)
        stored = np.asarray(artifact["extents_m"], dtype=float)
        item_errors = []
        if sha256_file(glb) != artifact.get("canonical_glb_sha256") or row.get("canonical_glb_sha256") != artifact.get("canonical_glb_sha256"):
            item_errors.append("canonical GLB hash mismatch")
        if len(nodes) != artifact.get("semantic_node_count"):
            item_errors.append("semantic node count mismatch")
        if not np.allclose(extents, stored, atol=1e-6, rtol=1e-6):
            item_errors.append("canonical GLB AABB mismatch")
        if artifact.get("unit_scale_to_m") != 1.0 or row.get("source_dataset_units") != "normalized_unit_sphere_non_metric":
            item_errors.append("normalized non-metric unit declaration mismatch")
        errors.extend(f"{row['task_id']}: {error}" for error in item_errors)
        artifact_checks.append({"task_id": row["task_id"], "status": "success", "canonical_glb_sha256": sha256_file(glb), "errors": item_errors})
    numeric1 = read_json(root / "score_numeric_na_run1/summary.json")
    numeric2 = read_json(root / "score_numeric_na_run2/summary.json")
    if numeric1 != numeric2 or numeric1.get("numeric_pass") is not None or numeric1.get("numeric_applicable_count") != 0 or numeric1.get("numeric_constraint_count") != 20:
        errors.append("numeric N/A replay or declaration mismatch")
    count1 = read_json(root / "score_count_proxy_run1/summary.json")
    count2 = read_json(root / "score_count_proxy_run2/summary.json")
    if count1 != count2 or count1.get("panel") != "image_count":
        errors.append("count proxy replay mismatch")
    if materialization_lock.get("artifact_manifest_sha256") != sha256_file(root / "artifact_manifest.jsonl"):
        errors.append("materialization artifact manifest hash mismatch")
    if materialization_lock.get("selection_lock_sha256") != sha256_file(root / "selection.lock.json"):
        errors.append("materialization selection lock hash mismatch")
    if materialization_lock.get("amendment_sha256") != sha256_file(AMENDMENT):
        errors.append("materialization amendment hash mismatch")
    if materialization_lock.get("implementation") != amendment.get("implementation"):
        errors.append("materialization implementation binding mismatch")
    if selection_lock.get("implementation") != amendment.get("implementation"):
        errors.append("selection implementation binding mismatch")
    if amendment.get("reporting", {}).get("numeric_dimension") != "N/A":
        errors.append("amendment numeric N/A policy drifted")
    return {
        "schema_version": 1,
        "method": "partnet_mobility_clip_retrieval_v1",
        "passed": not errors,
        "candidate_assets": len(inventory),
        "selection_tasks": len(selection),
        "artifact_successes": sum(row.get("status") == "success" for row in manifest),
        "numeric_pass": None,
        "numeric_display": "N/A",
        "count_proxy": count1.get("count_pass"),
        "artifact_checks": artifact_checks,
        "errors": errors,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    payload = verify(args.root)
    output = safe(args.output or (args.root / "dedicated_integrity.json"), must_exist=False)
    if output.exists():
        raise RuntimeError(f"integrity output exists; refusing overwrite: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2, sort_keys=True, ensure_ascii=True) + "\n", encoding="utf-8")
    print(json.dumps({"passed": payload["passed"], "artifacts": payload["artifact_successes"], "errors": len(payload["errors"]), "output": str(output)}, indent=2))
    return 0 if payload["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
