#!/usr/bin/env python3
"""Fail-closed Table 5 Editability preflight for the Nova3D baseline.

The public Nova3D checkout contains clients and integrations.  It is not, by
itself, a locally runnable generation/editing backend and it does not contain
the paper's original 18 edit cases.  This runner only permits local Nova3D
metrics when an immutable, Nova3D-attributed 18-case manifest links the source
programs, base assets, and edit prompts.  Paper values and this repository's
protocol-aligned procedural-asset run are recorded as separate evidence
classes and are never promoted to local Nova3D measurements.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable


REPO_ROOT = Path(__file__).resolve().parents[2]
WORKSPACE_ROOT = REPO_ROOT.parent.resolve()
EXP_ROOT = REPO_ROOT / "exp"
DEFAULT_OUTPUT = (
    EXP_ROOT / "runtime" / "nano3d_editability_baselines" / "nova3d"
)
OFFICIAL_REMOTE = re.compile(
    r"(?:github\.com[/:])RareSense/Nova3D(?:\.git)?$", re.IGNORECASE
)
NOVA_TOKEN = re.compile(r"(?:^|[/_.-])nova[-_]?3d(?:[/_.-]|$)", re.IGNORECASE)
ATTRIBUTION_KEYS = {"method", "source_method", "generator", "baseline"}
ASSET_SUFFIXES = {".blend", ".glb", ".gltf"}
SOURCE_SUFFIXES = {".blend", ".py"}
RECORD_SUFFIXES = {".json", ".jsonl", ".yaml", ".yml"}
NON_RESULT_STATUSES = {"BLOCKED", "NOT_RUN", "READY_NOT_RUN", "NOT_VERIFIED"}


def contained(path: Path, *, strict: bool = False) -> Path:
    resolved = path.resolve(strict=strict)
    if resolved != WORKSPACE_ROOT and WORKSPACE_ROOT not in resolved.parents:
        raise ValueError(f"path is outside authorized workspace: {resolved}")
    return resolved


def relative(path: Path) -> str:
    return str(contained(path).relative_to(WORKSPACE_ROOT))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with contained(path, strict=True).open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_small_text(path: Path, *, limit: int = 8 * 1024 * 1024) -> str | None:
    path = contained(path, strict=True)
    if path.stat().st_size > limit:
        return None
    try:
        return path.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError):
        return None


def git_value(checkout: Path, *args: str) -> str | None:
    checkout = contained(checkout, strict=True)
    result = subprocess.run(
        ["git", "-C", str(checkout), *args],
        check=False,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip() if result.returncode == 0 else None


def iter_files(roots: Iterable[Path]) -> Iterable[Path]:
    for root in roots:
        root = contained(root)
        if not root.is_dir() or root.is_symlink():
            continue
        for path in root.rglob("*"):
            if ".git" in path.parts or not path.is_file() or path.is_symlink():
                continue
            yield contained(path, strict=True)


def discover_checkouts() -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    seen: set[Path] = set()
    candidates = (
        WORKSPACE_ROOT / "Nova3D",
        WORKSPACE_ROOT / "nova3d",
        REPO_ROOT / "Nova3D",
        REPO_ROOT / "nova3d",
        REPO_ROOT / ".cache" / "Nova3D",
        REPO_ROOT / ".cache" / "nova3d",
        REPO_ROOT / ".cache" / "table6_sources" / "nova3d" / "code",
    )
    for candidate in candidates:
        current = contained(candidate)
        if current in seen or not current.is_dir() or current.is_symlink():
            continue
        seen.add(current)
        git_dir = current / ".git"
        if git_dir.is_dir() and not git_dir.is_symlink():
            remote = git_value(current, "config", "--get", "remote.origin.url")
            name_match = bool(NOVA_TOKEN.search(relative(current)))
            remote_match = bool(remote and OFFICIAL_REMOTE.search(remote))
            if name_match or remote_match:
                assets = sorted(
                    relative(path)
                    for path in current.rglob("*")
                    if path.is_file()
                    and not path.is_symlink()
                    and path.suffix.lower() in ASSET_SUFFIXES
                )
                examples = current / "examples"
                rows.append(
                    {
                        "path": relative(current),
                        "remote": remote,
                        "official_remote": remote_match,
                        "commit": git_value(current, "rev-parse", "HEAD"),
                        "branch": git_value(current, "branch", "--show-current"),
                        "worktree_clean": git_value(current, "status", "--short") == "",
                        "examples_directory_present": examples.is_dir()
                        and not examples.is_symlink(),
                        "structured_asset_count": len(assets),
                        "structured_assets": assets,
                    }
                )
    return sorted(rows, key=lambda row: str(row["path"]))


def is_nova_value(value: Any) -> bool:
    return isinstance(value, str) and value.strip().lower().replace("-", "").replace(
        "_", ""
    ) == "nova3d"


def has_exact_attribution(value: Any) -> bool:
    if isinstance(value, dict):
        for key, item in value.items():
            if str(key).lower() in ATTRIBUTION_KEYS and is_nova_value(item):
                return True
            if has_exact_attribution(item):
                return True
    elif isinstance(value, list):
        return any(has_exact_attribution(item) for item in value)
    return False


def record_is_non_result(value: Any) -> bool:
    if not isinstance(value, dict):
        return False
    status = str(value.get("status", "")).upper()
    evidence_class = str(value.get("evidence_class", "")).upper()
    return status in NON_RESULT_STATUSES or evidence_class in {
        "PAPER_ONLY",
        "PREFLIGHT_ONLY",
    }


def parse_json_record(path: Path) -> Any | None:
    if path.suffix.lower() != ".json":
        return None
    text = read_small_text(path)
    if text is None:
        return None
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        return None


def manifest_case_rows(payload: Any) -> list[dict[str, Any]] | None:
    if not isinstance(payload, dict):
        return None
    for key in ("tasks", "items", "cases", "assets"):
        rows = payload.get(key)
        if isinstance(rows, list) and all(isinstance(row, dict) for row in rows):
            return rows
    return None


def referenced_path(manifest: Path, row: dict[str, Any], keys: tuple[str, ...]) -> Path | None:
    for key in keys:
        value = row.get(key)
        if not isinstance(value, str) or not value.strip():
            continue
        candidate = Path(value)
        if not candidate.is_absolute():
            candidate = manifest.parent / candidate
        try:
            candidate = contained(candidate)
        except ValueError:
            return None
        return candidate if candidate.is_file() and not candidate.is_symlink() else None
    return None


def audit_attributed_inputs(output_dir: Path) -> dict[str, Any]:
    structured_assets: list[str] = []
    source_programs: list[str] = []
    attributed_records: list[str] = []
    eligible_manifests: list[dict[str, Any]] = []
    output_dir = contained(output_dir)

    scan_roots = (
        REPO_ROOT / ".cache" / "table6_sources" / "nova3d" / "code",
        EXP_ROOT / "reference",
        EXP_ROOT / "runtime" / "nano3d_paper_editability",
        EXP_ROOT / "runtime" / "table6_nova3d",
    )
    for path in iter_files(scan_roots):
        if output_dir == path.parent or output_dir in path.parents:
            continue
        rel = relative(path)
        suffix = path.suffix.lower()
        if NOVA_TOKEN.search(rel) and suffix in ASSET_SUFFIXES:
            structured_assets.append(rel)
        if (
            NOVA_TOKEN.search(rel)
            and suffix in SOURCE_SUFFIXES
            and any(token in path.parts for token in ("examples", "assets", "outputs", "paper"))
        ):
            source_programs.append(rel)
        if suffix not in RECORD_SUFFIXES:
            continue
        if not (
            NOVA_TOKEN.search(rel)
            or any(token in path.name.lower() for token in ("manifest", "edit", "prompt"))
        ):
            continue
        payload = parse_json_record(path)
        if payload is None or not has_exact_attribution(payload) or record_is_non_result(payload):
            continue
        attributed_records.append(rel)
        rows = manifest_case_rows(payload)
        if rows is None or len(rows) != 18:
            continue
        sources = [
            referenced_path(
                path,
                row,
                ("source_program", "program_path", "blender_source", "source_path"),
            )
            for row in rows
        ]
        bases = [
            referenced_path(path, row, ("base_asset", "base_glb", "asset_path"))
            for row in rows
        ]
        prompts = [
            row.get("edit_prompt", row.get("edit_instruction", row.get("instruction")))
            for row in rows
        ]
        eligible_manifests.append(
            {
                "path": rel,
                "sha256": sha256(path),
                "cases": len(rows),
                "source_programs_present": sum(item is not None for item in sources),
                "base_assets_present": sum(item is not None for item in bases),
                "edit_prompts_present": sum(
                    isinstance(item, str) and bool(item.strip()) for item in prompts
                ),
                "complete": (
                    all(item is not None for item in sources)
                    and all(item is not None for item in bases)
                    and all(isinstance(item, str) and item.strip() for item in prompts)
                ),
            }
        )

    return {
        "scan_scope": [
            relative(contained(root)) for root in scan_roots if contained(root).is_dir()
        ],
        "strict_attribution_rule": (
            "structured record has method/source_method/generator/baseline exactly Nova3D; "
            "BLOCKED, NOT_RUN, PAPER_ONLY, and PREFLIGHT_ONLY records are excluded"
        ),
        "nova3d_path_structured_assets": sorted(set(structured_assets)),
        "nova3d_example_or_output_source_programs": sorted(set(source_programs)),
        "nova3d_attributed_result_records": sorted(set(attributed_records)),
        "eligible_18_case_manifests": sorted(
            eligible_manifests, key=lambda row: str(row["path"])
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    output_dir = contained(args.output_dir)

    table = contained(EXP_ROOT / "Nano3dresults.md", strict=True)
    local_runner = contained(
        EXP_ROOT / "scripts" / "run_nano3d_paper_editability.py", strict=True
    )
    local_summary = contained(
        EXP_ROOT / "runtime" / "nano3d_paper_editability" / "summary.json",
        strict=True,
    )
    local_manifest = contained(
        EXP_ROOT / "runtime" / "nano3d_paper_editability" / "manifest.json",
        strict=True,
    )
    prior_nova_preflight = contained(
        EXP_ROOT / "runtime" / "table6_nova3d" / "preflight.json", strict=True
    )

    checkouts = discover_checkouts()
    official = [row for row in checkouts if row["official_remote"]]
    inputs = audit_attributed_inputs(output_dir)
    complete_manifests = [
        row for row in inputs["eligible_18_case_manifests"] if row["complete"]
    ]
    official_assets = sorted(
        asset for checkout in official for asset in checkout["structured_assets"]
    )

    readme = None
    readme_text = None
    if official:
        candidate = contained(WORKSPACE_ROOT / official[0]["path"] / "README.md")
        if candidate.is_file() and not candidate.is_symlink():
            readme = candidate
            readme_text = read_small_text(candidate)
    closed_backend_declared = bool(
        readme_text and "hosted generation backend is (currently) closed-source" in readme_text
    )
    examples_coming_soon = bool(
        readme_text
        and "examples/" in readme_text
        and "source programs (coming soon)" in readme_text
    )

    requirements = {
        "official_public_client_checkout": bool(official),
        "locally_runnable_official_generation_and_edit_backend": False,
        "original_paper_18_case_manifest": bool(complete_manifests),
        "original_18_source_programs": bool(
            complete_manifests
            and all(row["source_programs_present"] == 18 for row in complete_manifests)
        ),
        "original_18_base_assets": bool(
            complete_manifests
            and all(row["base_assets_present"] == 18 for row in complete_manifests)
        ),
        "original_18_edit_prompts": bool(
            complete_manifests
            and all(row["edit_prompts_present"] == 18 for row in complete_manifests)
        ),
        "nova3d_attributed_local_result_assets": bool(
            inputs["nova3d_path_structured_assets"] or official_assets
        ),
        "paper_compatible_editability_scorer_and_render_backend": False,
        "two_blind_reviewers_and_adjudication": False,
    }
    blocker_codes = []
    if not requirements["locally_runnable_official_generation_and_edit_backend"]:
        blocker_codes.append("CLOSED_HOSTED_BACKEND")
    if not requirements["original_paper_18_case_manifest"]:
        blocker_codes.append("ORIGINAL_18_CASE_MANIFEST_UNAVAILABLE")
    if not requirements["original_18_source_programs"]:
        blocker_codes.append("ORIGINAL_18_SOURCE_PROGRAMS_UNAVAILABLE")
    if not requirements["original_18_base_assets"]:
        blocker_codes.append("ORIGINAL_18_BASE_ASSETS_UNAVAILABLE")
    if not requirements["original_18_edit_prompts"]:
        blocker_codes.append("ORIGINAL_18_EDIT_PROMPTS_UNAVAILABLE")
    if not requirements["paper_compatible_editability_scorer_and_render_backend"]:
        blocker_codes.append("PAPER_SCORER_AND_RENDER_BACKEND_UNAVAILABLE")
    if not requirements["two_blind_reviewers_and_adjudication"]:
        blocker_codes.append("HUMAN_REVIEW_UNAVAILABLE")

    summary = {
        "schema_version": 1,
        "protocol_id": "nano3d_table5_nova3d_editability_preflight_v1",
        "baseline": "Nova3D",
        "table": "Table 5: Editability",
        "status": "BLOCKED" if blocker_codes else "READY_NOT_RUN",
        "evidence_class": "PREFLIGHT_ONLY",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "safety": {
            "workspace_root": str(WORKSPACE_ROOT),
            "network_accessed": False,
            "hosted_nova3d_api_called": False,
            "paid_generation_called": False,
            "secret_read": False,
            "gpu_job_started": False,
            "existing_job_touched": False,
            "paper_values_reused_as_local_metrics": False,
            "existing_pva_assets_relabelled_as_nova3d": False,
        },
        "official_checkout_audit": {
            "checkouts": checkouts,
            "official_checkout_count": len(official),
            "public_code_scope": "clients and integrations",
            "closed_backend_declared_in_readme": closed_backend_declared,
            "examples_source_programs_marked_coming_soon": examples_coming_soon,
            "official_checkout_structured_assets": official_assets,
            "interpretation": (
                "The public checkout is real Nova3D code, but it is client/integration "
                "code and does not make the hosted generation/editing method locally runnable."
            ),
        },
        "workspace_input_audit": inputs,
        "requirements": requirements,
        "blocker_codes": blocker_codes,
        "local_evaluation": {
            "status": "NOT_RUN",
            "display_value": "N/R",
            "assets": 0,
            "edits": 0,
            "metrics": {
                "target_fulfilled": None,
                "anchor": None,
                "scale": None,
                "non_target_preserved": None,
                "geometry_locality": None,
                "structural_locality": None,
                "post_edit_constraint_pass": None,
                "sixteen_seed_propagation": None,
                "regression_preservation": None,
                "final_pass": None,
                "edit_cost": None,
            },
            "sixteen_seed_policy": {
                "value": "N/A",
                "reason": (
                    "Nova3D Table 5 evaluates one per-asset source-program edit, not "
                    "propagation of one reusable template edit across 16 seeds."
                ),
            },
            "reason": (
                "No complete attributable 18-case Nova3D input manifest and no locally "
                "runnable official backend/scorer are available in the workspace."
            ),
        },
        "paper_reported_only": {
            "evidence_class": "PAPER_ONLY",
            "must_not_be_presented_as_local_reproduction": True,
            "source": "Existing transcription in exp/Nano3dresults.md Table 5",
            "cases": 18,
            "edit_mix": {"additive": 13, "modified_existing": 5},
            "target_fulfilled_reviewer_agreement": "88.9%",
            "anchor_reviewer_agreement": "94.4%",
            "scale_reviewer_agreement": "94.4%",
            "non_target_preserved": "18/18",
            "geometry_locality": "18/18",
            "final_pass": "14/18",
            "sixteen_seed_propagation": "N/A",
            "interpretation": (
                "The three agreement percentages are reviewer agreement, not field pass "
                "rates. These values have not been reproduced by this runner."
            ),
        },
        "excluded_protocol_aligned_local_run": {
            "method": "PV-A / arti-skill existing selected procedural assets",
            "protocol_alignment": "Nova3D Section 9 edit mix and deterministic front-half gates",
            "is_nova3d_method_output": False,
            "accepted_as_nova3d_baseline": False,
            "runner": relative(local_runner),
            "runner_sha256": sha256(local_runner),
            "manifest": relative(local_manifest),
            "manifest_sha256": sha256(local_manifest),
            "summary": relative(local_summary),
            "summary_sha256": sha256(local_summary),
            "reason": (
                "Its 18 assets are selected existing arti-skill procedural assets and its "
                "edits are direct template configuration changes; protocol alignment does "
                "not establish Nova3D method provenance."
            ),
        },
        "minimum_unblock_inputs": [
            "An immutable Nova3D-attributed manifest for the original 18 edit cases.",
            "All 18 pre-edit Blender source programs and base assets with hashes and stable IDs.",
            "The original edit prompt/instruction for every case and the 13 additive / 5 modified-existing labels.",
            "A locally runnable official edit backend, or separately authorized hosted execution with retained source and output artifacts.",
            "The paper-compatible Blender render/scoring contract, two independent blind reviews, and adjudication records.",
        ],
        "evidence": {
            "table_document": relative(table),
            "table_document_sha256": sha256(table),
            "official_readme": relative(readme) if readme else None,
            "official_readme_sha256": sha256(readme) if readme else None,
            "official_readme_relevant_lines": "26,31,38-40,117" if readme else None,
            "prior_nova3d_release_preflight": relative(prior_nova_preflight),
            "prior_nova3d_release_preflight_sha256": sha256(prior_nova_preflight),
        },
        "run_command": "python exp/scripts/run_nova3d_editability_baseline.py",
    }

    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )

    official_line = "0"
    if official:
        first = official[0]
        official_line = (
            f"1 (`{first['path']}`, commit `{first['commit']}`; "
            f"structured assets {first['structured_asset_count']})"
        )
    report = f"""# Nova3D Table 5 Editability baseline preflight

Status: **{summary['status']}** (`PREFLIGHT_ONLY`; local metrics **N/R**)

The official public Nova3D checkout is present, but it contains clients and
integrations rather than the locally runnable hosted generation/editing backend.
The README marks the generated-asset/source-program `examples/` directory as
coming soon. This distinction is important: public client code exists, while
the original 18-case source programs and benchmark inputs do not exist locally.

| Requirement | Available | Required |
|---|---:|---:|
| Official public client checkout | {official_line} | 1 |
| Locally runnable official backend | 0 | 1 |
| Complete original 18-case manifest | {len(complete_manifests)} | 1 |
| Original source programs | 0 | 18 |
| Original base assets | 0 | 18 |
| Original edit prompts | 0 | 18 |
| Paper-compatible scorer/render backend | 0 | 1 |
| Completed blind-review/adjudication set | 0 | 1 |

No hosted API, paid generation, secret, GPU, or external job was used. No local
Nova3D edit was run: `N=0` means missing attributable inputs, not a measured
failure or a zero score. The per-asset Nova3D comparison has **16-Seed
Propagation = N/A**.

## Evidence separation

The values already transcribed in Table 5 (18 cases; final pass 14/18) remain
`PAPER_ONLY`. The 88.9%, 94.4%, and 94.4% target/anchor/scale values are reviewer
agreement percentages, not pass rates, and were not reproduced here.

`exp/scripts/run_nano3d_paper_editability.py` and
`exp/runtime/nano3d_paper_editability/` are a protocol-aligned local run on 18
selected existing PV-A/arti-skill procedural assets. They are not Nova3D method
outputs and are excluded from this baseline.

Blockers: {', '.join(blocker_codes)}.
"""
    (output_dir / "report.md").write_text(report, encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 2 if blocker_codes else 0


if __name__ == "__main__":
    raise SystemExit(main())
