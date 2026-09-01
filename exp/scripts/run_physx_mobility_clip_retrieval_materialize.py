#!/usr/bin/env python3
"""Materialize locked PhysX retrievals after opaque shared selection."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import shutil
import stat
import subprocess
import sys
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path
from typing import Any


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
WORKSPACE = Path("/mnt/zsn/lyb").resolve()
EXTERNAL_WORKSPACE = Path("/mnt/zsn/zsn_workspace").resolve()
REFERENCE = REPO / "exp/reference/table4_constraints_v2"
ADDENDUM = REFERENCE / "amendment_physx_mobility_clip_retrieval_v1_addendum1.json"
PROMPTS = REFERENCE / "prompts.jsonl"
PROTOCOL = REFERENCE / "protocol.json"
SPECS = REFERENCE / "specs"
PRIVATE_SOURCE = REPO / "exp/runtime/table4_constraints_v2/physx_mobility_clip_retrieval_v1_private_source"
DEFAULT_ROOT = REPO / "exp/runtime/table4_constraints_v2/physx_mobility_clip_retrieval_v1"
SELECTION_BUNDLE_NAME = "selection_bundle"
MATERIALIZATION_BUNDLE_NAME = "materialization_bundle"
SOURCE_AUDITOR = REPO / "exp/scripts/audit_physx_mobility_table4_source.py"
RENDER_WORKER = REPO / "exp/scripts/render_partnet_mobility_table4_opaque.py"
RENDER_BUILDER = REPO / "exp/scripts/build_partnet_mobility_table4_render_snapshot.py"
SNAPSHOT_LOCKER = REPO / "exp/scripts/lock_partnet_mobility_table4_render_snapshot.py"
SELECTION_LAUNCHER = REPO / "exp/scripts/launch_mobility_table4_clip_retrieval_selection.py"
SELECTOR = REPO / "exp/scripts/run_partnet_mobility_clip_retrieval_selection.py"
SELECTION_CONTRACT = REFERENCE / "selection_contract_physx_mobility_clip_retrieval_v1.json"
CANONICALIZER = REPO / "exp/scripts/canonicalize_table4_artifact.py"
VERIFIER = REPO / "exp/scripts/verify_physx_mobility_clip_retrieval_v1.py"
EXPECTED_PROMPTS_SHA256 = "0b46c15c65c46550cc25654ccca10e4de87f1b97e6bde89796ea3cc9b439065e"
EXPECTED_PROTOCOL_SHA256 = "6857194072ccb0ba3943d14a62f29c2364920ec01381a5475311535c1831031f"
EXPECTED_TASKS = [f"T4C{index:03d}" for index in range(1, 19)]
EXPECTED_ASSETS = 2024
SELECTION_LOCKED_FILES = {
    "asset_embeddings_run1.npy", "asset_embeddings_run2.npy",
    "prompt_embeddings_run1.npy", "prompt_embeddings_run2.npy",
    "embedding_index.jsonl", "prompt_tokenization.jsonl", "selection.jsonl",
    "selection_replay.jsonl", "selection_preflight.json",
}


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


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with regular_file(path).open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_json(path: Path) -> Any:
    return json.loads(regular_file(path).read_text(encoding="utf-8"))


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in regular_file(path).read_text(encoding="utf-8").splitlines() if line.strip()]


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
        "source_auditor_sha256": SOURCE_AUDITOR,
        "render_worker_sha256": RENDER_WORKER,
        "render_builder_sha256": RENDER_BUILDER,
        "snapshot_locker_sha256": SNAPSHOT_LOCKER,
        "selection_launcher_sha256": SELECTION_LAUNCHER,
        "selector_sha256": SELECTOR,
        "materializer_sha256": SCRIPT,
        "integrity_verifier_sha256": VERIFIER,
        "canonicalizer_sha256": CANONICALIZER,
    }
    return {field: sha256_file(path) for field, path in paths.items()}


def tree_manifest(root: Path) -> list[dict[str, Any]]:
    root = regular_dir(root)
    rows: list[dict[str, Any]] = []
    for directory, dirnames, filenames in os.walk(root, followlinks=False):
        current = Path(directory)
        for dirname in dirnames:
            child = current / dirname
            if child.is_symlink() or not child.is_dir():
                raise RuntimeError(f"non-regular directory in selected tree: {child}")
        for filename in filenames:
            child = regular_file(current / filename)
            rows.append({
                "path": child.relative_to(root).as_posix(),
                "bytes": child.stat().st_size,
                "sha256": sha256_file(child),
            })
    return sorted(rows, key=lambda row: row["path"])


def numeric_na_records() -> tuple[list[dict[str, Any]], dict[str, Any]]:
    if sha256_file(PROTOCOL) != EXPECTED_PROTOCOL_SHA256:
        raise RuntimeError("numeric N/A protocol binding drifted")
    protocol = read_json(PROTOCOL)
    records: list[dict[str, Any]] = []
    for task_id in EXPECTED_TASKS:
        spec_path = regular_file(SPECS / f"{task_id}.json")
        if sha256_file(spec_path) != protocol.get("spec_sha256", {}).get(task_id):
            raise RuntimeError(f"numeric N/A spec binding drifted: {task_id}")
        spec = read_json(spec_path)
        if spec.get("task_id") != task_id:
            raise RuntimeError(f"numeric N/A spec task drifted: {task_id}")
        for constraint in spec.get("constraints", []):
            if constraint.get("type") == "numeric_dimension":
                records.append({
                    "task_id": task_id,
                    "constraint_id": constraint["constraint_id"],
                    "status": "N/A",
                    "passed": None,
                    "evaluable": False,
                    "reason": "PhysX-Mobility metric unit and dimension-axis mapping are unestablished",
                })
    if len(records) != 20 or len({row["constraint_id"] for row in records}) != 20:
        raise RuntimeError("numeric N/A constraint closure drifted")
    summary = {
        "schema_version": 1,
        "status": "N/A",
        "passed": None,
        "protocol_numeric_constraints": 20,
        "evaluable_numeric_constraints": 0,
        "not_applicable_numeric_constraints": 20,
        "headline": "N/A",
        "zero_over_twenty_is_forbidden": True,
    }
    return records, summary


def validate(root: Path) -> tuple[dict[str, Any], list[dict[str, Any]], dict[str, dict[str, Any]], dict[str, Any]]:
    addendum = read_json(ADDENDUM)
    observed = implementation_hashes()
    if addendum.get("status") != "pre_result_frozen" or addendum.get("implementation") != observed:
        raise RuntimeError("PhysX addendum implementation binding drifted")
    if observed["canonicalizer_sha256"] != sha256_file(CANONICALIZER):
        raise RuntimeError("canonicalizer hash differs from addendum before invocation")
    shared = addendum.get("shared_pipeline", {})
    if shared.get("selection_contract_sha256") != sha256_file(SELECTION_CONTRACT):
        raise RuntimeError("PhysX selection execution contract binding drifted")
    root = regular_dir(root)
    selection_bundle = regular_dir(root / SELECTION_BUNDLE_NAME)
    selection_lock = read_json(selection_bundle / "selection.lock.json")
    shared_selection_hashes = {
        "selection_launcher_sha256": observed["selection_launcher_sha256"],
        "selector_sha256": observed["selector_sha256"],
    }
    if selection_lock.get("implementation") != shared_selection_hashes:
        raise RuntimeError("selection lock implementation binding drifted")
    if selection_lock.get("selection_execution_contract_sha256") != sha256_file(SELECTION_CONTRACT):
        raise RuntimeError("selection lock execution-contract binding drifted")
    if selection_lock.get("task_count") != 18:
        raise RuntimeError("selection lock task closure drifted")
    if set(selection_lock.get("locked_file_sha256", {})) != SELECTION_LOCKED_FILES:
        raise RuntimeError("selection locked-file set drifted")
    if {entry.name for entry in selection_bundle.iterdir()} != SELECTION_LOCKED_FILES | {"selection.lock.json"}:
        raise RuntimeError("selection bundle exact closure drifted")
    for entry in selection_bundle.iterdir():
        if entry.is_symlink() or not stat.S_ISREG(entry.lstat().st_mode):
            raise RuntimeError(f"selection bundle contains non-regular file: {entry}")
    for name, expected in selection_lock["locked_file_sha256"].items():
        if Path(name).name != name or sha256_file(selection_bundle / name) != expected:
            raise RuntimeError(f"selection locked file drifted: {name}")
    selections = read_jsonl(selection_bundle / "selection.jsonl")
    replay = read_jsonl(selection_bundle / "selection_replay.jsonl")
    expected_fields = {
        "task_id", "prompt_sha256", "selected_embedding_row", "selected_candidate_key",
        "clip_cosine_similarity", "tie_break", "fallback_allowed",
    }
    if selections != replay or len(selections) != 18 or [row.get("task_id") for row in selections] != EXPECTED_TASKS:
        raise RuntimeError("selection task/replay closure drifted")
    for row in selections:
        if set(row) != expected_fields or row["fallback_allowed"] is not False:
            raise RuntimeError(f"selection schema/fallback drifted: {row.get('task_id')}")
        if not isinstance(row["clip_cosine_similarity"], float) or not math.isfinite(row["clip_cosine_similarity"]):
            raise RuntimeError(f"selection score invalid: {row['task_id']}")
    source_lock = read_json(PRIVATE_SOURCE / "source_audit.lock.json")
    binding_path = PRIVATE_SOURCE / "source_binding.jsonl"
    if source_lock.get("status") != "PASS" or source_lock.get("asset_count") != EXPECTED_ASSETS:
        raise RuntimeError("private source audit is not full PASS")
    if source_lock.get("amendment_sha256") != sha256_file(ADDENDUM) or source_lock.get("implementation") != observed:
        raise RuntimeError("private source implementation/addendum binding drifted")
    if source_lock.get("source_binding_sha256") != sha256_file(binding_path):
        raise RuntimeError("private source binding drifted")
    if selection_lock.get("private_source_audit_lock_sha256") != sha256_file(PRIVATE_SOURCE / "source_audit.lock.json"):
        raise RuntimeError("selection/private source audit binding drifted")
    if selection_lock.get("source_binding_sha256") != source_lock["source_binding_sha256"]:
        raise RuntimeError("selection/private source identity binding drifted")
    bindings = read_jsonl(binding_path)
    by_key = {row["candidate_key"]: row for row in bindings}
    if len(bindings) != EXPECTED_ASSETS or len(by_key) != EXPECTED_ASSETS:
        raise RuntimeError("source binding cardinality/uniqueness drifted")
    for row in selections:
        if row["selected_candidate_key"] not in by_key:
            raise RuntimeError(f"selected opaque key missing from source binding: {row['task_id']}")
    if sha256_file(PROMPTS) != EXPECTED_PROMPTS_SHA256:
        raise RuntimeError("prompt manifest drifted")
    return selection_lock, selections, by_key, observed


def materialize(root: Path) -> None:
    root = safe_write(root)
    selection_lock, selections, bindings, implementation = validate(root)
    expected_root = {"protocol_audit_pre_result.json", "report.md", SELECTION_BUNDLE_NAME}
    if {entry.name for entry in root.iterdir()} != expected_root:
        raise RuntimeError("formal root must contain only audit/report and atomic selection bundle before materialization")
    bundle = safe_write(root / MATERIALIZATION_BUNDLE_NAME)
    staging = safe_write(root.parent / f".{root.name}.{MATERIALIZATION_BUNDLE_NAME}.staging")
    if bundle.exists() or staging.exists():
        raise RuntimeError("materialization bundle/staging must both be absent; never overwrite or resume")
    staging.mkdir(exist_ok=False)
    canonical_root = staging / "canonical"
    materialized_root = staging / "materialized"
    canonical_root.mkdir(parents=True, exist_ok=False)
    materialized_root.mkdir(exist_ok=False)
    manifest: list[dict[str, Any]] = []
    metadata_rows: list[dict[str, Any]] = []
    for selection in selections:
        task_id = selection["task_id"]
        key = selection["selected_candidate_key"]
        binding = bindings[key]
        identity = binding["source_asset_id"]
        row: dict[str, Any] = {
            "task_id": task_id,
            "method": "physx_mobility_clip_retrieval_v1",
            "status": "failed",
            "attempts": 1,
            "repair_attempts": 0,
            "selected_candidate_key": key,
            "selected_identity": identity,
            "fallback_used": False,
            "numeric_constraints": "N/A: official source does not establish dimension unit or XYZ mapping",
            "count_proxy": "N/R: source link names are structural l_N identifiers, not Table 4 role semantics",
        }
        try:
            source_urdf = regular_file(Path(binding["source_urdf"]))
            source_partseg = regular_dir(source_urdf.parents[1] / "partseg" / identity)
            source_json = regular_file(Path(binding["source_finaljson"]))
            if sha256_file(source_urdf) != binding["source_urdf_sha256"] or sha256_file(source_json) != binding["source_finaljson_sha256"]:
                raise RuntimeError("selected source binding hash drifted")
            package = materialized_root / task_id
            copied_urdf_dir = package / "urdf"
            copied_partseg = package / "partseg" / identity
            copied_urdf_dir.mkdir(parents=True, exist_ok=False)
            shutil.copytree(source_partseg, copied_partseg, symlinks=False)
            copied_urdf = copied_urdf_dir / f"{identity}.urdf"
            shutil.copy2(source_urdf, copied_urdf)
            if sha256_file(copied_urdf) != binding["source_urdf_sha256"]:
                raise RuntimeError("copied URDF differs from private source binding")
            if tree_manifest(source_partseg) != tree_manifest(copied_partseg):
                raise RuntimeError("copied selected partseg tree is not byte-identical")
            if implementation["canonicalizer_sha256"] != sha256_file(CANONICALIZER):
                raise RuntimeError("canonicalizer hash drifted immediately before invocation")
            canonical_dir = canonical_root / task_id
            completed = subprocess.run(
                [
                    sys.executable,
                    str(regular_file(CANONICALIZER)),
                    "--input", str(copied_urdf),
                    "--artifact-type", "urdf",
                    "--nonmetric-dataset-units",
                    "--output-dir", str(canonical_dir),
                ],
                cwd=str(REPO), text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                check=False, timeout=600, env={**os.environ, "PYTHONHASHSEED": "0"},
            )
            atomic_text(canonical_root / f"{task_id}.canonicalize.log", completed.stdout)
            if completed.returncode != 0:
                raise RuntimeError(f"canonicalizer exit {completed.returncode}")
            artifact_path = canonical_dir / "artifact.json"
            artifact = read_json(artifact_path)
            metric_fields = {key for key in artifact if key == "unit_scale_to_m" or key.endswith("_m")}
            if metric_fields:
                raise RuntimeError(f"nonmetric canonical artifact contains metric fields: {sorted(metric_fields)}")
            if (
                artifact.get("coordinate_units") != "dataset_units_metric_binding_unestablished"
                or artifact.get("metric_eligible") is not False
                or artifact.get("numeric_constraint_status") != "N/A"
                or artifact.get("geometry_scale_applied") != 1.0
                or "bounds_dataset_units" not in artifact
                or "extents_dataset_units" not in artifact
            ):
                raise RuntimeError("nonmetric canonical artifact declaration drifted")
            semantic_path = canonical_dir / "semantic_nodes.json"
            semantic_nodes = read_json(semantic_path)
            source_xml = ET.parse(copied_urdf).getroot()
            source_links = [link.attrib["name"] for link in source_xml.findall("link") if link.findall("visual")]
            source_visual_links = [link.attrib["name"] for link in source_xml.findall("link") for _ in link.findall("visual")]
            canonical_links = [node["name"] for node in semantic_nodes if node["level"] == "link"]
            canonical_visuals = [node["link_name"] for node in semantic_nodes if node["level"] == "visual"]
            if Counter(canonical_links) != Counter(source_links) or Counter(canonical_visuals) != Counter(source_visual_links):
                raise RuntimeError("canonical semantic nodes did not preserve source link/visual mapping")
            if len(canonical_links) != binding["mesh_bearing_links"] or len(canonical_visuals) != binding["visual_count"]:
                raise RuntimeError("canonical semantic counts differ from private source binding")
            final_package = bundle / "materialized" / task_id
            final_canonical = bundle / "canonical" / task_id
            final_source = final_package / "urdf" / f"{identity}.urdf"
            final_glb = final_canonical / "artifact.glb"
            final_semantic = final_canonical / "semantic_nodes.json"
            artifact.update({
                "source": str(final_source),
                "canonical_glb": str(final_glb),
                "semantic_nodes": str(final_semantic),
            })
            write_json(artifact_path, artifact)
            metadata = json.loads(source_json.read_text(encoding="utf-8"))
            metadata_rows.append({
                "task_id": task_id,
                "selected_candidate_key": key,
                "metadata_sha256": binding["source_finaljson_sha256"],
                "dimension_raw": metadata.get("dimension"),
                "dimension_used_for_materialization_or_scoring": False,
                "object_name_used_for_retrieval_or_scoring": False,
                "category_used_for_retrieval_or_scoring": False,
                "parts_used_for_retrieval_or_scoring": False,
            })
            row.update({
                "status": "success",
                "artifact_type": "urdf",
                "source": str(final_source),
                "source_sha256": binding["source_urdf_sha256"],
                "original_source": str(source_urdf),
                "canonical_dir": str(final_canonical),
                "canonical_glb": str(final_glb),
                "canonical_glb_sha256": artifact["canonical_glb_sha256"],
                "source_scale_policy": "raw released OBJ/URDF dataset coordinates; explicit nonmetric canonicalizer mode; geometry scale 1.0; no metric claim",
                "metric_eligible": False,
                "extents_dataset_units": artifact["extents_dataset_units"],
                "source_hierarchy_preserved": True,
                "source_link_visual_names_preserved": True,
                "source_mesh_bearing_links": binding["mesh_bearing_links"],
                "source_visual_count": binding["visual_count"],
                "semantic_nodes": str(final_semantic),
                "semantic_nodes_sha256": sha256_file(semantic_path),
            })
        except Exception as error:
            row["error"] = f"{type(error).__name__}: {error}"
            shutil.rmtree(materialized_root / task_id, ignore_errors=True)
            shutil.rmtree(canonical_root / task_id, ignore_errors=True)
        manifest.append(row)
    write_jsonl(staging / "selected_metadata_audit.jsonl", metadata_rows)
    write_jsonl(staging / "artifact_manifest.jsonl", manifest)
    numeric_records, numeric_summary = numeric_na_records()
    write_jsonl(staging / "numeric_constraints_na.jsonl", numeric_records)
    write_json(staging / "numeric_summary.json", numeric_summary)
    success = sum(row["status"] == "success" for row in manifest)
    provenance = {
        "schema_version": 1,
        "method": "physx_mobility_clip_retrieval_v1",
        "display_name": "PhysX-Mobility prompt-only CLIP retrieval (fixed-dataset reference; not generation)",
        "method_type": "fixed_dataset_prompt_only_retrieval_reference_control",
        "same_prompt_generation_method": False,
        "dataset": {
            "id": "Caoza/PhysX-Mobility",
            "revision": "d0768ee9e1415f6be8db78d6389ba018b85134c0",
            "archive_sha256": "88308cc2a4cc6177c59e32c2de51e881e6b961737295e5082d7ed01cca221908",
            "license": "CC-BY-NC-4.0 from official dataset API; local archive lacks license text",
            "units": "raw source coordinates; metric unit and dimension-axis mapping unestablished",
        },
        "selection": {
            "model": "openai/clip-vit-base-patch32",
            "revision": "3d74acf9a28c67741b2f4f2ea7635f0aaf6f0268",
            "global_top1": True,
            "repair_attempts": 0,
            "rank_fallbacks": 0,
            "selection_lock_sha256": sha256_file(root / SELECTION_BUNDLE_NAME / "selection.lock.json"),
            "selection_execution_contract_sha256": sha256_file(SELECTION_CONTRACT),
        },
        "benchmark": {
            "prompt_manifest_sha256": EXPECTED_PROMPTS_SHA256,
            "amendment_sha256": sha256_file(ADDENDUM),
            "numeric": "N/A",
            "numeric_protocol_constraints": 20,
            "numeric_evaluable_constraints": 0,
            "numeric_passed": None,
            "count": "N/R",
        },
    }
    write_json(staging / "provenance.json", provenance)
    atomic_text(staging / "result_report.md", f"""# PhysX-Mobility prompt-only CLIP retrieval: Table 4 Constraints v2

Status: **{'COMPLETE' if success == 18 else 'PARTIAL'}**. This is a fixed-dataset retrieval/reference control, not same-prompt generation.

| Candidates | Selected artifacts | Numeric constraints | Count proxy |
|---:|---:|---:|---:|
| {EXPECTED_ASSETS} | {success}/18 | N/A (0 evaluable / 20 protocol constraints) | N/R |

The exact 18 frozen prompts selected global top-1 assets from the shared opaque eight-view renderer and CLIP selector used by the paired PartNet-Mobility reference. Selection had no category, source path, geometry, part name, dimensions, specs, or prior scores. There was no repair or rank fallback.

Numeric dimensions are N/A because official evidence does not establish dimension units or annotation-axis mapping. Count is N/R because structural `l_N` source names are not frozen Table 4 semantic-role mappings.
""")
    lock = {
        "schema_version": 1,
        "status": "PASS" if success == 18 else "PARTIAL",
        "method": "physx_mobility_clip_retrieval_v1",
        "is_generation_method": False,
        "dataset_reference_only": True,
        "tasks": len(manifest),
        "success": success,
        "failed": len(manifest) - success,
        "repair_attempts": 0,
        "rank_fallbacks": 0,
        "numeric_constraints": "N/A",
        "numeric_protocol_constraints": 20,
        "numeric_evaluable_constraints": 0,
        "numeric_passed": None,
        "count_proxy": "N/R",
        "metric_scale_recovery_attempted": False,
        "selection_lock_sha256": sha256_file(root / SELECTION_BUNDLE_NAME / "selection.lock.json"),
        "selection_execution_contract_sha256": sha256_file(SELECTION_CONTRACT),
        "private_source_audit_lock_sha256": sha256_file(PRIVATE_SOURCE / "source_audit.lock.json"),
        "selected_metadata_audit_sha256": sha256_file(staging / "selected_metadata_audit.jsonl"),
        "artifact_manifest_sha256": sha256_file(staging / "artifact_manifest.jsonl"),
        "numeric_constraints_na_sha256": sha256_file(staging / "numeric_constraints_na.jsonl"),
        "numeric_summary_sha256": sha256_file(staging / "numeric_summary.json"),
        "provenance_sha256": sha256_file(staging / "provenance.json"),
        "result_report_sha256": sha256_file(staging / "result_report.md"),
        "implementation": implementation,
        "materializer_sha256": sha256_file(SCRIPT),
        "canonicalizer_sha256": sha256_file(CANONICALIZER),
        "amendment_sha256": sha256_file(ADDENDUM),
    }
    write_json(staging / "materialization.lock.json", lock)
    staging.replace(bundle)
    print(json.dumps(lock, indent=2, sort_keys=True))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    args = parser.parse_args()
    materialize(args.root)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
