#!/usr/bin/env python3
"""Fail-closed preflight for the LAM row of Nano3D Table 6.

The script does not download LAM and never treats another method's assets as
LAM output. It records whether the authorized workspace has the official
checkout, frozen prompt-to-output manifest, attributed URDF packages, and
joint gold needed by the Table 6 protocol.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
import re
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable


REPO_ROOT = Path(__file__).resolve().parents[2]
WORKSPACE_ROOT = REPO_ROOT.parent.resolve()
DEFAULT_OUTPUT = REPO_ROOT / "exp" / "runtime" / "table6_lam"
CHECKOUT_NAMES = {
    "lam",
    "lam3d",
    "large-articulation-model",
    "large_articulation_model",
    "language-articulated-object-modelers",
    "language_articulated_object_modelers",
}
ATTRIBUTION_RE = re.compile(
    r'"(?:method|source_method|generator|baseline)"\s*:\s*"LAM"', re.IGNORECASE
)
LAM_PATH_TOKEN_RE = re.compile(r"(?:^|[/_.-])lam(?:[/_.-]|$)", re.IGNORECASE)
PROMPT_ID_RE = re.compile(
    r'"(?:prompt_id|task_id|input_id|prompt_sha256|input_sha256)"\s*:', re.IGNORECASE
)
JOINT_GOLD_RE = re.compile(
    r'"(?:joint_gold|expected_joints|gold_joints|joint_spec)"\s*:', re.IGNORECASE
)
CHECKPOINT_SUFFIXES = {".pt", ".pth", ".ckpt", ".safetensors", ".bin"}


def contained(path: Path) -> Path:
    resolved = path.resolve()
    if resolved != WORKSPACE_ROOT and WORKSPACE_ROOT not in resolved.parents:
        raise ValueError(f"path is outside authorized workspace: {resolved}")
    return resolved


def iter_files(root: Path, *, max_depth: int | None = None) -> Iterable[Path]:
    root = contained(root)
    for dirpath, dirnames, filenames in os.walk(root, followlinks=False):
        current = contained(Path(dirpath))
        depth = len(current.relative_to(root).parts)
        dirnames[:] = [name for name in dirnames if not (current / name).is_symlink()]
        if max_depth is not None and depth >= max_depth:
            dirnames[:] = []
        for filename in filenames:
            path = current / filename
            if not path.is_symlink():
                yield contained(path)


def iter_dirs(root: Path, *, max_depth: int) -> Iterable[Path]:
    root = contained(root)
    for dirpath, dirnames, _ in os.walk(root, followlinks=False):
        current = contained(Path(dirpath))
        depth = len(current.relative_to(root).parts)
        dirnames[:] = [name for name in dirnames if not (current / name).is_symlink()]
        if depth >= max_depth:
            dirnames[:] = []
            continue
        for dirname in dirnames:
            yield contained(current / dirname)


def read_small_text(path: Path, limit: int = 16 * 1024 * 1024) -> str | None:
    path = contained(path)
    if path.stat().st_size > limit:
        return None
    try:
        return path.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError):
        return None


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with contained(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def relative(path: Path) -> str:
    return str(contained(path).relative_to(WORKSPACE_ROOT))


def dependency_status() -> dict[str, bool]:
    return {
        name: importlib.util.find_spec(name) is not None
        for name in ("numpy", "scipy", "urdfpy", "pybullet")
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    output_dir = contained(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    checkout_candidates = sorted(
        relative(path)
        for path in iter_dirs(WORKSPACE_ROOT, max_depth=2)
        if path.name.lower() in CHECKOUT_NAMES
    )

    checkpoint_candidates: list[Path] = []
    for checkout in checkout_candidates:
        for path in iter_files(WORKSPACE_ROOT / checkout):
            if path.suffix.lower() in CHECKPOINT_SUFFIXES:
                checkpoint_candidates.append(path)

    structured_assets: list[str] = []
    attributed_records: list[str] = []
    attributed_manifests: list[str] = []
    prompt_linked_manifests: list[str] = []
    joint_gold_manifests: list[str] = []
    scan_roots = [REPO_ROOT / "exp" / "reference", REPO_ROOT / "exp" / "runtime"]
    for scan_root in scan_roots:
        for path in iter_files(scan_root, max_depth=5):
            if output_dir == path.parent or output_dir in path.parents:
                continue
            rel = relative(path)
            if path.suffix.lower() == ".urdf" and LAM_PATH_TOKEN_RE.search(rel):
                structured_assets.append(rel)
            if path.suffix.lower() not in {".json", ".jsonl"}:
                continue
            if "manifest" not in path.name.lower() and not LAM_PATH_TOKEN_RE.search(rel):
                continue
            text = read_small_text(path)
            if text is None or not ATTRIBUTION_RE.search(text):
                continue
            attributed_records.append(rel)
            if "manifest" not in path.name.lower():
                continue
            attributed_manifests.append(rel)
            if PROMPT_ID_RE.search(text):
                prompt_linked_manifests.append(rel)
            if JOINT_GOLD_RE.search(text):
                joint_gold_manifests.append(rel)

    checkout_present = bool(checkout_candidates)
    checkpoint_present = bool(checkpoint_candidates)
    assets_present = bool(structured_assets)
    output_manifest_present = bool(attributed_manifests)
    prompt_link_present = bool(prompt_linked_manifests)
    joint_gold_present = bool(joint_gold_manifests)
    dependencies = dependency_status()
    runtime_dependencies_present = all(dependencies.values())

    blockers: list[str] = []
    if not checkout_present:
        blockers.append("official LAM checkout not found in the authorized workspace")
    if not checkpoint_present:
        blockers.append("no checkpoint file attributable to LAM found in the authorized workspace")
    if not assets_present:
        blockers.append("no LAM-attributed URDF package found")
    if not output_manifest_present:
        blockers.append("no manifest explicitly attributes structured outputs to LAM")
    if not prompt_link_present:
        blockers.append("no frozen common prompt-to-LAM-output identity mapping found")
    if not runtime_dependencies_present:
        missing = sorted(name for name, present in dependencies.items() if not present)
        blockers.append(f"articulation evaluator dependencies missing: {', '.join(missing)}")

    table_metrics = {
        "articulable": "N/R",
        "joints_per_asset": "N/R",
        "native_joint_exposure": "N/R",
        "joint_type_accuracy": "N/R",
        "joint_recall": "N/R",
        "parent_child_accuracy": "N/R",
        "axis_valid": "N/R",
        "origin_valid": "N/R",
        "limit_valid": "N/R",
        "joint_geometric_valid": "N/R",
        "asset_geometric_valid": "N/R",
        "full_range_collision_free": "N/R",
        "generic_range": "N/R",
    }
    evaluator = REPO_ROOT / "exp" / "scripts" / "run_nano3d_articulation_paper.py"
    summary = {
        "protocol_id": "nano3d_table6_lam_preflight_v1",
        "method": "LAM",
        "status": "BLOCKED" if blockers else "READY_NOT_RUN",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "workspace_root": str(WORKSPACE_ROOT),
        "repository_root": str(REPO_ROOT),
        "network_accessed": False,
        "gpu_task_launched": False,
        "evaluated_asset_count": 0,
        "evaluation_policy": {
            "comparison_unit": "one final LAM structured asset per frozen shared text prompt",
            "method_attribution_required": True,
            "other_method_assets_reused": False,
            "single_joint_samples": 11,
            "multi_joint_sobol_configurations": 64,
            "collision_policy": "URDF_USE_SELF_COLLISION_EXCLUDE_PARENT",
            "ccd": False,
            "paper_values_reused": False,
            "missing_evidence_is_zero": False,
        },
        "requirements": {
            "official_lam_checkout": checkout_present,
            "lam_checkpoint": checkpoint_present,
            "lam_attributed_urdf_packages": assets_present,
            "lam_output_manifest": output_manifest_present,
            "frozen_prompt_output_mapping": prompt_link_present,
            "joint_semantic_gold_for_accuracy_columns": joint_gold_present,
            "runtime_dependencies": runtime_dependencies_present,
        },
        "runtime_dependencies": dependencies,
        "artifact_gates": {
            "lam_code": {
                "present": checkout_present,
                "candidates": checkout_candidates,
                "missing_evidence": None
                if checkout_present
                else "0 exact-name official checkout candidates in the non-symlink workspace scan (depth <= 2)",
            },
            "lam_checkpoint": {
                "present": checkpoint_present,
                "candidates": [
                    {"path": relative(path), "sha256": sha256(path)}
                    for path in sorted(checkpoint_candidates)
                ],
                "missing_evidence": None
                if checkpoint_present
                else "0 checkpoint candidates inside the 0 identified official LAM checkouts",
            },
            "lam_generated_urdf": {
                "present": assets_present,
                "candidates": [
                    {"path": path, "sha256": sha256(WORKSPACE_ROOT / path)}
                    for path in sorted(structured_assets)
                ],
                "missing_evidence": None
                if assets_present
                else "0 .urdf files with an explicit LAM path token under exp/reference and exp/runtime",
            },
            "common_prompt_manifest": {
                "present": prompt_link_present,
                "candidates": [
                    {"path": path, "sha256": sha256(WORKSPACE_ROOT / path)}
                    for path in sorted(prompt_linked_manifests)
                ],
                "missing_evidence": None
                if prompt_link_present
                else "0 manifests jointly containing explicit method=LAM attribution and frozen prompt/task identity",
            },
        },
        "search": {
            "checkout_scope": "all non-symlink directories under workspace, depth <= 2",
            "checkpoint_scope": "checkpoint suffixes recursively inside identified official LAM checkouts only",
            "asset_scope": [relative(path) for path in scan_roots],
            "checkout_name_candidates": sorted(CHECKOUT_NAMES),
            "checkout_candidates": checkout_candidates,
            "lam_checkpoint_candidates": [
                {"path": relative(path), "sha256": sha256(path)}
                for path in sorted(checkpoint_candidates)
            ],
            "lam_urdf_assets": sorted(structured_assets),
            "lam_urdf_asset_hashes": [
                {"path": path, "sha256": sha256(WORKSPACE_ROOT / path)}
                for path in sorted(structured_assets)
            ],
            "lam_attributed_records": sorted(attributed_records),
            "lam_attributed_manifests": sorted(attributed_manifests),
            "prompt_linked_manifests": sorted(prompt_linked_manifests),
            "prompt_linked_manifest_hashes": [
                {"path": path, "sha256": sha256(WORKSPACE_ROOT / path)}
                for path in sorted(prompt_linked_manifests)
            ],
            "joint_gold_manifests": sorted(joint_gold_manifests),
        },
        "blockers": blockers,
        "non_blocking_metric_gap": None
        if joint_gold_present
        else "Joint type/recall/parent-child/axis/origin/limit semantic accuracy cannot be scored without independent frozen joint gold.",
        "metrics": table_metrics,
        "evidence": {
            "table_document": relative(REPO_ROOT / "exp" / "Nano3dresults.md"),
            "table_document_sha256": sha256(REPO_ROOT / "exp" / "Nano3dresults.md"),
            "protocol_document": relative(REPO_ROOT / "exp" / "Nano3d.md"),
            "protocol_document_sha256": sha256(REPO_ROOT / "exp" / "Nano3d.md"),
            "local_articulation_evaluator": relative(evaluator),
            "local_articulation_evaluator_sha256": sha256(evaluator),
            "evaluator_main_is_frozen_n33_specific": True,
        },
        "minimum_next_step": [
            "Place the official LAM checkout and its required model/runtime files inside the authorized workspace.",
            "Freeze the common text-prompt manifest and run LAM once per prompt with the declared repair budget.",
            "Save each final URDF plus meshes and a manifest with method=LAM, prompt identity, package path, and hashes.",
            "Adapt the existing articulation evaluator's manifest loader from its hard-coded 33-asset PV-A cohort, then run 11 single-joint states and 64 Sobol multi-joint states.",
            "Provide independent joint gold to score semantic accuracy columns; otherwise keep those columns N/A while reporting only direct metadata and collision proxies.",
        ],
    }

    summary_path = output_dir / "summary.json"
    summary_path.write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    report = f"""# Table 6 LAM articulation baseline

Status: **{summary['status']}**

No LAM asset was evaluated. The authorized workspace contains no official LAM
checkout or attributable checkpoint, no LAM-attributed URDF package, and no frozen common manifest mapping
shared prompts to LAM outputs. Existing PV-A and Articraft packages were not
relabeled as LAM outputs. Therefore missing evidence remains `N/R`, not zero.

## Local preflight

- Official checkout candidates: {len(checkout_candidates)}.
- LAM-attributed checkpoint candidates: {len(checkpoint_candidates)}.
- LAM-attributed URDF packages: {len(structured_assets)}.
- LAM-attributed output manifests: {len(attributed_manifests)}.
- Frozen prompt-output mappings: {len(prompt_linked_manifests)}.
- Independent joint-gold manifests: {len(joint_gold_manifests)}.
- Evaluator dependencies: {json.dumps(dependencies, sort_keys=True)}.
- Existing paper-aligned evaluator: `{relative(evaluator)}`; its current main
  manifest loader is fixed to the PV-A N=33 cohort and must be adapted for LAM.
- GPU task launched: no. Network accessed: no.

## Table 6 row

| Method | Articulable | Joints/Asset | Native Joint Exposure | Joint Type Accuracy | Joint Recall | Parent-Child Accuracy | Axis Valid | Origin Valid | Limit Valid | Joint Geom. Valid | Asset Geom. Valid | Full-Range Collision-Free | Generic Range |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| LAM (common articulation benchmark) | N/R | N/R | N/R | N/R | N/R | N/R | N/R | N/R | N/R | N/R | N/R | N/R | N/R |

## Minimum unblock sequence

1. Add the official checkout and required model/runtime files inside the authorized workspace.
2. Freeze the common prompt manifest and produce one final LAM URDF package per prompt.
3. Add explicit LAM attribution, prompt identity, package paths, and hashes to the result manifest.
4. Run the frozen 11-state single-joint and 64-configuration Sobol sweep through the local evaluator adapter.
5. Supply independent joint gold for semantic accuracy; without it, those columns stay `N/A` even after collision sweeps run.
"""
    (output_dir / "report.md").write_text(report, encoding="utf-8")
    print(
        json.dumps(
            {"status": summary["status"], "output_dir": str(output_dir), "blockers": blockers},
            ensure_ascii=False,
        )
    )
    return 2 if blockers else 0


if __name__ == "__main__":
    raise SystemExit(main())
