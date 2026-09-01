#!/usr/bin/env python3
"""Materialize locked PartNet top-1 selections and report count proxy plus numeric N/A."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any

import numpy as np


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
WORKSPACE = REPO.parent.resolve()
REFERENCE = REPO / "exp/reference/table4_constraints_v2"
PROMPTS = REFERENCE / "prompts.jsonl"
PROTOCOL = REFERENCE / "protocol.json"
AMENDMENT = REFERENCE / "amendment_partnet_mobility_clip_retrieval_v1.json"
PRIVATE_SOURCE = REPO / "exp/runtime/table4_constraints_v2/partnet_mobility_clip_retrieval_v1_private_source"
SOURCE_AUDITOR = REPO / "exp/scripts/audit_partnet_mobility_table4_source.py"
RENDER_WORKER = REPO / "exp/scripts/render_partnet_mobility_table4_opaque.py"
RENDER_BUILDER = REPO / "exp/scripts/build_partnet_mobility_table4_render_snapshot.py"
SNAPSHOT_LOCKER = REPO / "exp/scripts/lock_partnet_mobility_table4_render_snapshot.py"
SELECTION_LAUNCHER = REPO / "exp/scripts/launch_mobility_table4_clip_retrieval_selection.py"
SELECTOR = REPO / "exp/scripts/run_partnet_mobility_clip_retrieval_selection.py"
CANONICALIZER = REPO / "exp/scripts/canonicalize_table4_artifact.py"
SCORER = REPO / "exp/scripts/score_table4_constraints_v2.py"
INTEGRITY = REPO / "exp/scripts/verify_table4_constraints_v2_integrity.py"
DEDICATED_VERIFIER = REPO / "exp/scripts/verify_partnet_mobility_table4_retrieval.py"
DEFAULT_ROOT = REPO / "exp/runtime/table4_constraints_v2/partnet_mobility_clip_retrieval_v1"
METHOD = "partnet_mobility_clip_retrieval_v1"
EXPECTED_PROMPTS_SHA256 = "0b46c15c65c46550cc25654ccca10e4de87f1b97e6bde89796ea3cc9b439065e"
EXPECTED_TASKS = [f"T4C{number:03d}" for number in range(1, 19)]


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


def run_command(command: list[str], log: Path) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(command, cwd=str(safe(REPO)), text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=False, env={**os.environ, "PYTHONHASHSEED": "0"})
    atomic_text(log, result.stdout)
    return result


def implementation_hashes() -> dict[str, str]:
    return {
        "source_auditor_sha256": sha256_file(SOURCE_AUDITOR),
        "render_worker_sha256": sha256_file(RENDER_WORKER),
        "render_builder_sha256": sha256_file(RENDER_BUILDER),
        "snapshot_locker_sha256": sha256_file(SNAPSHOT_LOCKER),
        "selection_launcher_sha256": sha256_file(SELECTION_LAUNCHER),
        "selector_sha256": sha256_file(SELECTOR),
        "materializer_sha256": sha256_file(SCRIPT),
        "integrity_verifier_sha256": sha256_file(DEDICATED_VERIFIER),
        "canonicalizer_sha256": sha256_file(CANONICALIZER),
        "count_scorer_sha256": sha256_file(SCORER),
        "generic_integrity_verifier_sha256": sha256_file(INTEGRITY),
    }


def validate_selection(root: Path) -> tuple[dict[str, Any], list[dict[str, Any]], dict[str, dict[str, Any]]]:
    selection_root = safe(root / "selection_bundle")
    lock = read_json(selection_root / "selection.lock.json")
    amendment = read_json(AMENDMENT)
    if amendment.get("status") != "pre_result_frozen" or amendment.get("implementation") != implementation_hashes():
        raise RuntimeError("amendment implementation binding drifted")
    shared_selection = {field: implementation_hashes()[field] for field in ("selection_launcher_sha256", "selector_sha256")}
    contract_path = safe(REPO / read_json(AMENDMENT)["shared_pipeline"]["selection_contract"])
    contract = read_json(contract_path)
    if lock.get("selection_execution_contract_sha256") != sha256_file(contract_path) or lock.get("implementation") != shared_selection:
        raise RuntimeError("selection/current execution-contract binding drifted")
    if lock.get("contract_id") != contract.get("contract_id") or lock.get("phase") != "selection_locked_before_source_binding_or_geometry_access" or lock.get("task_count") != 18:
        raise RuntimeError("selection lock is invalid")
    for name, expected in lock["locked_file_sha256"].items():
        if sha256_file(selection_root / name) != expected:
            raise RuntimeError(f"selection locked file drifted: {name}")
    selection = read_jsonl(selection_root / "selection.jsonl")
    replay = read_jsonl(selection_root / "selection_replay.jsonl")
    index = read_jsonl(selection_root / "embedding_index.jsonl")
    prompts = read_jsonl(PROMPTS)
    assets = np.load(selection_root / "asset_embeddings_run1.npy", allow_pickle=False)
    text = np.load(selection_root / "prompt_embeddings_run1.npy", allow_pickle=False)
    if selection != replay or len(selection) != 18 or [row["task_id"] for row in selection] != EXPECTED_TASKS:
        raise RuntimeError("selection task closure/replay drifted")
    if assets.shape != (2347, 512) or text.shape != (18, 512) or not np.isfinite(assets).all() or not np.isfinite(text).all():
        raise RuntimeError("locked embedding matrices are invalid")
    similarities = text @ assets.T
    for prompt_offset, row in enumerate(selection):
        selected = min(range(len(index)), key=lambda offset: (-float(similarities[prompt_offset, offset]), index[offset]["candidate_key"]))
        if row["selected_embedding_row"] != selected or row["selected_candidate_key"] != index[selected]["candidate_key"]:
            raise RuntimeError(f"selection is not deterministic global top-1: {row['task_id']}")
        if row["clip_cosine_similarity"] != float(similarities[prompt_offset, selected]) or row["fallback_allowed"] is not False:
            raise RuntimeError(f"selection score/policy drifted: {row['task_id']}")
        if row["prompt_sha256"] != hashlib.sha256(prompts[prompt_offset]["prompt"].encode()).hexdigest():
            raise RuntimeError(f"selection prompt binding drifted: {row['task_id']}")
    source_lock = read_json(PRIVATE_SOURCE / "source_audit.lock.json")
    if sha256_file(PRIVATE_SOURCE / "source_audit.lock.json") != lock["private_source_audit_lock_sha256"]:
        raise RuntimeError("private source audit lock drifted")
    if sha256_file(PRIVATE_SOURCE / "source_binding.jsonl") != lock["source_binding_sha256"] or source_lock["source_binding_sha256"] != lock["source_binding_sha256"]:
        raise RuntimeError("private source binding drifted")
    bindings = read_jsonl(PRIVATE_SOURCE / "source_binding.jsonl")
    by_key = {row["candidate_key"]: row for row in bindings}
    if len(bindings) != 2347 or len(by_key) != 2347:
        raise RuntimeError("private source binding closure drifted")
    return lock, selection, by_key


def materialize(root: Path, selection: list[dict[str, Any]], bindings: dict[str, dict[str, Any]]) -> list[dict[str, Any]]:
    canonical_root = safe(root / "canonical", must_exist=False)
    if canonical_root.exists():
        raise RuntimeError("canonical output exists; refusing overwrite")
    canonical_root.mkdir()
    rows = []
    for selected in selection:
        task_id = selected["task_id"]
        binding = bindings[selected["selected_candidate_key"]]
        source = safe(Path(binding["source_urdf"]))
        destination = canonical_root / task_id
        row: dict[str, Any] = {
            "task_id": task_id,
            "method": METHOD,
            "status": "failure",
            "repair_attempts": 0,
            "fallback_used": False,
            "selection_candidate_key": selected["selected_candidate_key"],
            "clip_cosine_similarity": selected["clip_cosine_similarity"],
            "source_artifact": str(source),
            "source_artifact_sha256": sha256_file(source),
            "source_dataset_units": "normalized_unit_sphere_non_metric",
            "canonical_dir": str(destination),
        }
        try:
            if row["source_artifact_sha256"] != binding["source_urdf_sha256"]:
                raise RuntimeError("selected source URDF hash drifted")
            result = run_command([
                sys.executable, str(safe(CANONICALIZER)), "--input", str(source), "--artifact-type", "urdf",
                "--unit-scale-to-m", "1.0", "--output-dir", str(destination),
            ], canonical_root / f"{task_id}.canonicalize.log")
            if result.returncode != 0:
                raise RuntimeError(f"canonicalizer_exit_{result.returncode}")
            artifact = read_json(destination / "artifact.json")
            if artifact["source_sha256"] != row["source_artifact_sha256"] or artifact["unit_scale_to_m"] != 1.0:
                raise RuntimeError("canonical artifact source/unit binding drifted")
            row.update({
                "status": "success",
                "canonical_glb_sha256": artifact["canonical_glb_sha256"],
                "semantic_node_count": artifact["semantic_node_count"],
                "normalized_extents_dataset_units": artifact["extents_m"],
            })
        except Exception as exc:
            row["failure_reason"] = f"{type(exc).__name__}:{exc}"
        rows.append(row)
        print(f"[materialize] {task_id}: {row['status']}", flush=True)
    write_jsonl(root / "artifact_manifest.jsonl", rows)
    return rows


def compare_outputs(first: Path, second: Path, names: tuple[str, ...]) -> dict[str, str]:
    hashes = {}
    for name in names:
        first_hash, second_hash = sha256_file(first / name), sha256_file(second / name)
        if first_hash != second_hash:
            raise RuntimeError(f"replay differs: {name}")
        hashes[name] = first_hash
    return hashes


def score_count(root: Path) -> tuple[dict[str, Any], dict[str, str]]:
    outputs = [root / "score_count_proxy_run1", root / "score_count_proxy_run2"]
    for index, output in enumerate(outputs, 1):
        result = run_command([
            sys.executable, str(safe(SCORER)), "--method", METHOD, "--panel", "image_count",
            "--artifact-manifest", str(root / "artifact_manifest.jsonl"), "--output-dir", str(output),
        ], root / f"score_count_proxy_run{index}.log")
        if result.returncode != 0:
            raise RuntimeError(f"count proxy scorer failed run {index}")
    hashes = compare_outputs(outputs[0], outputs[1], ("records.json", "summary.json", "report.md"))
    return read_json(outputs[0] / "summary.json"), hashes


def numeric_na(root: Path, rows: list[dict[str, Any]]) -> tuple[dict[str, Any], dict[str, str]]:
    outputs = [root / "score_numeric_na_run1", root / "score_numeric_na_run2"]
    protocol = read_json(PROTOCOL)
    artifacts = {row["task_id"]: row for row in rows}
    for output in outputs:
        output.mkdir()
        records = []
        numeric_count = 0
        for task_id in EXPECTED_TASKS:
            spec = read_json(REFERENCE / "specs" / f"{task_id}.json")
            constraints = []
            for constraint in spec["constraints"]:
                if constraint["type"] != "numeric_dimension":
                    continue
                numeric_count += 1
                constraints.append({
                    "task_id": task_id,
                    "constraint_id": constraint["constraint_id"],
                    "type": "numeric_dimension",
                    "applicable": False,
                    "measurable": False,
                    "passed": False,
                    "reason": "PartNet-Mobility geometry is normalized to a unit sphere and has no recoverable metric scale; target-driven rescaling is forbidden",
                    "artifact_registered": artifacts[task_id]["status"] == "success",
                })
            records.append({"task_id": task_id, "constraints": constraints})
        if numeric_count != 20:
            raise RuntimeError(f"numeric constraint count drifted: {numeric_count}")
        summary = {
            "schema_version": 1,
            "benchmark_id": protocol["benchmark_id"],
            "prompt_manifest_sha256": protocol["prompt_manifest_sha256"],
            "method": METHOD,
            "panel": "dataset_normalized_numeric_not_applicable",
            "task_count": 18,
            "artifact_count": sum(row["status"] == "success" for row in rows),
            "numeric_constraint_count": 20,
            "numeric_applicable_count": 0,
            "numeric_pass": None,
            "display_numeric_pass": "N/A",
            "reason": "normalized_unit_sphere_without_metric_scale",
        }
        write_json(output / "records.json", records)
        write_json(output / "summary.json", summary)
        atomic_text(output / "report.md", f"# {METHOD}: numeric dimensions\n\nNumeric pass: **N/A**. All 20 numeric constraints are not applicable because the fixed dataset geometry is unit-sphere normalized and no metric scale is recoverable. No target-driven rescaling was performed.\n")
    return read_json(outputs[0] / "summary.json"), compare_outputs(outputs[0], outputs[1], ("records.json", "summary.json", "report.md"))


def generic_integrity(root: Path) -> dict[str, Any]:
    output = root / "table4_artifact_integrity.json"
    result = run_command([sys.executable, str(safe(INTEGRITY)), "--manifest", f"{METHOD}={root / 'artifact_manifest.jsonl'}", "--output", str(output)], root / "table4_artifact_integrity.log")
    payload = read_json(output)
    if result.returncode != 0 or payload.get("passed") is not True:
        raise RuntimeError("generic Table4 artifact integrity failed")
    return payload


def run(root: Path) -> None:
    root = safe(root)
    lock, selection, bindings = validate_selection(root)
    rows = materialize(root, selection, bindings)
    count, count_hashes = score_count(root)
    numeric, numeric_hashes = numeric_na(root, rows)
    generic = generic_integrity(root)
    provenance = {
        "schema_version": 1,
        "method": METHOD,
        "display_name": "PartNet-Mobility prompt-only CLIP retrieval (fixed-dataset reference; not generation)",
        "method_type": "fixed_dataset_prompt_only_retrieval_reference_control",
        "same_prompt_generation_method": False,
        "dataset": {
            "id": "sapien-sim/PartNetMobility",
            "official_huggingface_revision": "ee0aa3ef1df16181d76d83f7415aa8c94ed1da8f",
            "local_archive_sha256": "b47247a44246111e8d09f2c0e64b4012ae35e0dcf4bb55f68a05b604455119ff",
            "local_archive_binding_to_official_object": "unverified",
            "license": "other; gated; non-commercial research and education only; ShapeNet terms also apply",
            "redistribution": "raw selected assets and renders remain internal; report only metrics and cryptographic hashes",
            "units": "unit-sphere normalized, non-metric",
        },
        "selection": {
            "model": "openai/clip-vit-base-patch32",
            "revision": "3d74acf9a28c67741b2f4f2ea7635f0aaf6f0268",
            "global_top1": True,
            "repair_attempts": 0,
            "rank_fallbacks": 0,
            "selection_lock_sha256": sha256_file(root / "selection_bundle/selection.lock.json"),
        },
        "benchmark": {
            "prompt_manifest_sha256": EXPECTED_PROMPTS_SHA256,
            "amendment_sha256": sha256_file(AMENDMENT),
            "numeric": "N/A because normalized dataset units are not metric",
            "count": "supplementary name-matched renderable-node proxy only",
        },
    }
    write_json(root / "provenance.json", provenance)
    success = sum(row["status"] == "success" for row in rows)
    atomic_text(root / "result_report.md", f"""# PartNet-Mobility prompt-only CLIP retrieval: Table 4 Constraints v2

Status: **COMPLETE**. This is a fixed-dataset retrieval/reference control, not same-prompt generation.

| Candidates | Eligible renders | Selected artifacts | Numeric pass | Named-node count proxy |
|---:|---:|---:|---:|---:|
| 2347 | 2347 | {success}/18 | N/A | {count['count_pass']} |

The exact 18 frozen prompts selected global top-1 assets from an opaque eight-view render snapshot. Selection had no category, source path, geometry, part name, dimensions, specs, or prior scores. There was no repair or rank fallback.

Numeric dimensions are N/A: PartNet-Mobility geometry is unit-sphere normalized and no metric scale is recoverable. No target-driven rescaling was performed. The count value is only the frozen name-matched renderable-node proxy and is not semantic exact-count ground truth.
""")
    materialization_lock = {
        "schema_version": 1,
        "method": METHOD,
        "selection_lock_sha256": sha256_file(root / "selection_bundle/selection.lock.json"),
        "amendment_sha256": sha256_file(AMENDMENT),
        "implementation": implementation_hashes(),
        "materializer_sha256": sha256_file(SCRIPT),
        "canonicalizer_sha256": sha256_file(CANONICALIZER),
        "count_scorer_sha256": sha256_file(SCORER),
        "generic_integrity_verifier_sha256": sha256_file(INTEGRITY),
        "dedicated_integrity_verifier_sha256": sha256_file(DEDICATED_VERIFIER),
        "artifact_manifest_sha256": sha256_file(root / "artifact_manifest.jsonl"),
        "success_count": success,
        "failure_count": 18 - success,
        "repair_attempts": 0,
        "rank_fallbacks": 0,
        "numeric_summary": numeric,
        "count_proxy_summary": count,
        "numeric_replay_sha256": numeric_hashes,
        "count_proxy_replay_sha256": count_hashes,
        "generic_integrity_passed": generic["passed"],
        "provenance_sha256": sha256_file(root / "provenance.json"),
        "result_report_sha256": sha256_file(root / "result_report.md"),
    }
    write_json(root / "materialization.lock.json", materialization_lock)
    dedicated_output = root / "dedicated_integrity.json"
    dedicated_result = run_command([
        sys.executable, str(safe(DEDICATED_VERIFIER)), "--root", str(root), "--output", str(dedicated_output),
    ], root / "dedicated_integrity.log")
    dedicated = read_json(dedicated_output)
    if dedicated_result.returncode != 0 or dedicated.get("passed") is not True:
        raise RuntimeError("dedicated PartNet retrieval integrity failed")
    print(json.dumps({"status": "MATERIALIZED", "artifacts": f"{success}/18", "numeric_pass": "N/A", "count_proxy": count["count_pass"]}, indent=2), flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    run(parser.parse_args().root)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
