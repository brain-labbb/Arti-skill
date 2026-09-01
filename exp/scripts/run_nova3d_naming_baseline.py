#!/usr/bin/env python3
"""Fail-closed local preflight for Nova3D in Nano3D Naming Table 2.

The runner does not download Nova3D and does not relabel this repository's
``nano3d_*`` evaluation artifacts as Nova3D outputs.  A local direct Naming
evaluation requires an identifiable Nova3D checkout, Nova3D-attributed GLB or
URDF artifacts, and a shared manifest linking those artifacts to the frozen
baseline Naming protocol.  Semantic metrics additionally require independent
output-blind gold and three complete blind-judge verdict sets.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable


REPO_ROOT = Path(__file__).resolve().parents[2]
WORKSPACE_ROOT = REPO_ROOT.parent.resolve()
DEFAULT_OUTPUT = REPO_ROOT / "exp" / "runtime" / "nova3d_naming_v1"
CHECKOUT_NAMES = {
    "nova3d",
    "nova-3d",
    "nova_3d",
}
REMOTE_RE = re.compile(r"nova[-_]?3d", re.IGNORECASE)
ATTRIBUTION_RE = re.compile(
    r'"(?:method|source_method|generator|baseline)"\s*:\s*"Nova3D"',
    re.IGNORECASE,
)
NOVA_PATH_TOKEN_RE = re.compile(r"(?:^|[/_.-])nova[-_]?3d(?:[/_.-]|$)", re.IGNORECASE)
STRUCTURED_SUFFIXES = {".glb", ".gltf", ".urdf"}
NON_RESULT_STATUSES = {"BLOCKED", "NOT_RUN", "READY_NOT_RUN", "STARTING", "NOT_VERIFIED"}


def contained(path: Path) -> Path:
    resolved = path.resolve()
    if resolved != WORKSPACE_ROOT and WORKSPACE_ROOT not in resolved.parents:
        raise ValueError(f"path is outside authorized workspace: {resolved}")
    return resolved


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with contained(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_small_text(path: Path, limit: int = 16 * 1024 * 1024) -> str | None:
    path = contained(path)
    if path.stat().st_size > limit:
        return None
    try:
        return path.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError):
        return None


def iter_files(root: Path, *, max_depth: int) -> Iterable[Path]:
    root = contained(root)
    for dirpath, dirnames, filenames in os.walk(root, followlinks=False):
        current = Path(dirpath)
        depth = len(current.relative_to(root).parts)
        dirnames[:] = [name for name in dirnames if not (current / name).is_symlink()]
        if depth >= max_depth:
            dirnames[:] = []
        for filename in filenames:
            path = current / filename
            if not path.is_symlink():
                yield contained(path)


def direct_directories(root: Path) -> Iterable[Path]:
    root = contained(root)
    if not root.is_dir() or root.is_symlink():
        return
    for entry in os.scandir(root):
        if not entry.is_symlink() and entry.is_dir(follow_symlinks=False):
            yield contained(Path(entry.path))


def is_non_result_record(path: Path, text: str) -> bool:
    if "preflight" in path.name.lower():
        return True
    if path.suffix.lower() != ".json":
        return False
    try:
        payload = json.loads(text)
    except json.JSONDecodeError:
        return False
    if not isinstance(payload, dict):
        return False
    if str(payload.get("status", "")).upper() in NON_RESULT_STATUSES:
        return True
    if str(payload.get("evidence_class", "")).upper() == "PAPER_ONLY":
        return True
    local_evaluation = payload.get("local_evaluation")
    return (
        isinstance(local_evaluation, dict)
        and str(local_evaluation.get("status", "")).upper() in NON_RESULT_STATUSES
    )


def checkout_audit() -> tuple[list[dict[str, str]], list[str]]:
    search_roots = [WORKSPACE_ROOT, REPO_ROOT]
    repo_cache = REPO_ROOT / ".cache"
    if repo_cache.is_dir() and not repo_cache.is_symlink():
        search_roots.append(contained(repo_cache))

    candidates: dict[str, dict[str, str]] = {}
    remote_configs: list[str] = []
    for search_root in search_roots:
        for directory in direct_directories(search_root):
            relative = str(directory.relative_to(WORKSPACE_ROOT))
            if directory.name.lower() in CHECKOUT_NAMES:
                candidates[relative] = {"path": relative, "evidence": "directory_name"}

            config = directory / ".git" / "config"
            if not config.is_file() or config.is_symlink():
                continue
            text = read_small_text(config)
            if text is not None and REMOTE_RE.search(text):
                config_relative = str(contained(config).relative_to(WORKSPACE_ROOT))
                remote_configs.append(config_relative)
                candidates[relative] = {"path": relative, "evidence": "git_remote"}

    return sorted(candidates.values(), key=lambda row: row["path"]), sorted(remote_configs)


def artifact_audit(output_dir: Path) -> tuple[list[str], list[str], list[str]]:
    structured_assets: list[str] = []
    attributed_records: list[str] = []
    manifests: list[str] = []
    scan_roots = [REPO_ROOT / "exp" / "reference", REPO_ROOT / "exp" / "runtime"]
    for scan_root in scan_roots:
        for path in iter_files(scan_root, max_depth=5):
            if output_dir == path.parent or output_dir in path.parents:
                continue
            relative = str(path.relative_to(WORKSPACE_ROOT))
            if path.suffix.lower() in STRUCTURED_SUFFIXES and NOVA_PATH_TOKEN_RE.search(relative):
                structured_assets.append(relative)
            if path.suffix.lower() not in {".json", ".jsonl"}:
                continue
            text = read_small_text(path)
            if text is None or not ATTRIBUTION_RE.search(text):
                continue
            if is_non_result_record(path, text):
                continue
            attributed_records.append(relative)
            if "manifest" in path.name.lower():
                manifests.append(relative)
    return sorted(structured_assets), sorted(attributed_records), sorted(manifests)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    output_dir = contained(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    protocol_path = REPO_ROOT / "exp" / "reference" / "baseline_naming_protocol_v1.json"
    judge_protocol_path = REPO_ROOT / "exp" / "reference" / "naming_protocol_v2.json"
    table_path = REPO_ROOT / "exp" / "Nano3dresults.md"
    existing_manifest = REPO_ROOT / "exp" / "runtime" / "nano3d_glb_n33" / "input_packages" / "input_manifest.json"
    protocol = json.loads(contained(protocol_path).read_text(encoding="utf-8"))

    checkout_candidates, remote_configs = checkout_audit()
    structured_assets, attributed_records, manifest_candidates = artifact_audit(output_dir)
    official_checkout_present = bool(checkout_candidates)
    attributed_assets_present = bool(structured_assets or attributed_records)
    shared_manifest_present = bool(manifest_candidates)

    independent_gold_complete = False
    three_blind_judges_complete = False
    blockers = []
    if not official_checkout_present:
        blockers.append("official Nova3D checkout or Nova3D git remote not found in the authorized workspace")
    if not attributed_assets_present:
        blockers.append("no Nova3D-attributed GLB/URDF artifacts or generation records found")
    if not shared_manifest_present:
        blockers.append("no shared Naming manifest attributes frozen inputs and outputs to Nova3D")
    if not independent_gold_complete:
        blockers.append("no output-independent Nova3D role gold is linked to a shared artifact manifest")
    if not three_blind_judges_complete:
        blockers.append("three complete independent blind-judge verdict sets are not available for Nova3D outputs")

    summary = {
        "protocol_id": "nano3d_nova3d_naming_preflight_v1.1",
        "status": "BLOCKED" if blockers else "READY_NOT_RUN",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "workspace_root": str(WORKSPACE_ROOT),
        "repository_root": str(REPO_ROOT),
        "network_accessed": False,
        "evaluation_policy": {
            "urdf_part": "one link with at least one valid renderable visual geometry; multiple visuals on one link are merged",
            "glb_part": "one mesh-bearing GLB scene node",
            "representations_reported_separately": True,
            "method_attribution_required": True,
            "semantic_metrics_require_output_independent_gold_and_three_blind_judges": True,
            "cross_seed_consistency": "N/A for per-asset Nova3D generation",
            "paper_values_reused": False,
            "existing_nano3d_outputs_relabelled": False,
        },
        "search": {
            "checkout_scope": [
                ". (direct non-symlink directories and their git remotes)",
                "arti-skill (direct non-symlink directories and their git remotes)",
                "arti-skill/.cache (direct non-symlink directories and their git remotes)",
            ],
            "artifact_scope": [
                "arti-skill/exp/reference (non-symlink files, depth <= 5)",
                "arti-skill/exp/runtime (non-symlink files, depth <= 5)",
            ],
            "checkout_name_candidates": sorted(CHECKOUT_NAMES),
            "checkout_candidates": checkout_candidates,
            "matching_git_remote_configs": remote_configs,
            "nova3d_structured_assets": structured_assets,
            "nova3d_attributed_records": attributed_records,
            "nova3d_manifest_candidates": manifest_candidates,
        },
        "requirements": {
            "official_checkout": official_checkout_present,
            "nova3d_attributed_structured_assets": attributed_assets_present,
            "shared_naming_manifest": shared_manifest_present,
            "independent_output_blind_role_gold": independent_gold_complete,
            "three_complete_blind_judges": three_blind_judges_complete,
        },
        "blockers": blockers,
        "metrics": {
            "assets": 0,
            "parts": None,
            "named_nameability": None,
            "semantic_precision": None,
            "semantic_recall": None,
            "naming_richness": None,
            "functional_core_coverage": None,
            "instance_discriminability": None,
            "cross_seed_consistency": None,
            "over_segmentation_rate": None,
        },
        "evidence": {
            "table_document": str(table_path.relative_to(WORKSPACE_ROOT)),
            "table_document_sha256": sha256(table_path),
            "baseline_naming_protocol": str(protocol_path.relative_to(WORKSPACE_ROOT)),
            "baseline_naming_protocol_id": protocol["protocol_id"],
            "baseline_naming_protocol_sha256": sha256(protocol_path),
            "blind_judge_protocol": str(judge_protocol_path.relative_to(WORKSPACE_ROOT)),
            "blind_judge_protocol_sha256": sha256(judge_protocol_path),
            "existing_arti_skill_manifest": str(existing_manifest.relative_to(WORKSPACE_ROOT)),
            "existing_arti_skill_manifest_sha256": sha256(existing_manifest),
            "existing_manifest_method": "arti-skill seed_exports / seed_exports_physics_10",
            "existing_manifest_is_nova3d_attributed": False,
        },
    }
    summary_path = output_dir / "summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    report = f"""# Nova3D Naming baseline preflight

Status: **{summary['status']}**

The local Nova3D row for Table 2 was not evaluated. The authorized workspace
contains no identifiable official Nova3D checkout, no Nova3D-attributed GLB or
URDF output package, and no shared manifest connecting Nova3D outputs to the
frozen Naming inputs. Existing `nano3d_*` runtime artifacts come from this
repository's `seed_exports` or `seed_exports_physics_10` pipeline and are not
relabelled as Nova3D outputs.

## Evidence

- Frozen common protocol: `{protocol['protocol_id']}` at
  `arti-skill/exp/reference/baseline_naming_protocol_v1.json`.
- Checkout candidates or matching git remotes: {len(checkout_candidates)}.
- Nova3D-attributed structured assets or records: {len(structured_assets) + len(attributed_records)}.
- Nova3D-attributed shared manifests: {len(manifest_candidates)}.
- Output-independent Nova3D role gold linked to those assets: no.
- Complete independent blind-judge verdict sets linked to those assets: 0/3.
- Network access: none.

## Required to unblock

1. Place the official Nova3D checkout inside the authorized workspace.
2. Generate or provide Nova3D GLB/URDF outputs for the frozen shared inputs.
3. Add a manifest that explicitly attributes every artifact to Nova3D and
   preserves shared asset/category identity.
4. Run direct Parts/Nameability on mesh-bearing GLB nodes (or report URDF links
   separately); complete output-independent gold and all three blind judges
   before reporting semantic metrics.

Until these inputs exist, all local Nova3D Table 2 metrics remain `N/R` (JSON
`null`); the separate Nova3D paper row is contextual evidence, not a local
rerun. Reporting zeros would conflate missing evidence with measured failure.
"""
    (output_dir / "report.md").write_text(report, encoding="utf-8")
    print(json.dumps({"status": summary["status"], "output_dir": str(output_dir), "blockers": blockers}))
    return 2 if blockers else 0


if __name__ == "__main__":
    raise SystemExit(main())
