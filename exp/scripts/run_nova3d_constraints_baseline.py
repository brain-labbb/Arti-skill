#!/usr/bin/env python3
"""Fail-closed Table 4 Constraints preflight for the Nova3D baseline.

This runner only evaluates locally attributable Nova3D artifacts against the
frozen paper-aligned protocol. It never derives targets from generated geometry
and never imports paper-reported values as local measurements.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import subprocess
from datetime import datetime, timezone
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
WORKSPACE_ROOT = REPO_ROOT.parent.resolve()
EXP_ROOT = REPO_ROOT / "exp"
PROTOCOL_PATH = EXP_ROOT / "reference" / "paper_constraints_protocol_v1.json"
COMMON_INPUT_ROOT = EXP_ROOT / "reference" / "paper_constraints"
DEFAULT_OUTPUT = (
    EXP_ROOT / "runtime" / "table4_constraints_baselines" / "nova3d"
)
STRUCTURED_SUFFIXES = {".blend", ".glb", ".gltf"}
NOVA_TOKEN = re.compile(r"(?:^|[/_.-])nova[-_]?3d(?:[/_.-]|$)", re.IGNORECASE)


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
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_text(path: Path, *, limit: int = 16 * 1024 * 1024) -> str | None:
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


def checkout_audit() -> list[dict[str, object]]:
    candidates = [
        WORKSPACE_ROOT / "Nova3D",
        WORKSPACE_ROOT / "nova3d",
        REPO_ROOT / "Nova3D",
        REPO_ROOT / "nova3d",
        REPO_ROOT / ".cache" / "Nova3D",
        REPO_ROOT / ".cache" / "nova3d",
        REPO_ROOT / ".cache" / "table6_sources" / "nova3d" / "code",
    ]
    rows: list[dict[str, object]] = []
    seen: set[Path] = set()
    for candidate in candidates:
        if candidate.is_symlink():
            continue
        resolved = contained(candidate)
        if resolved in seen or not resolved.is_dir():
            continue
        seen.add(resolved)
        remote = git_value(resolved, "config", "--get", "remote.origin.url")
        commit = git_value(resolved, "rev-parse", "HEAD")
        status = git_value(resolved, "status", "--short")
        assets = sorted(
            relative(path)
            for path in resolved.rglob("*")
            if not path.is_symlink()
            and path.is_file()
            and path.suffix.lower() in STRUCTURED_SUFFIXES
        )
        paper_inputs = sorted(
            relative(path)
            for path in resolved.rglob("*")
            if not path.is_symlink()
            and path.is_file()
            and path.name.lower()
            in {"prompt_manifest.jsonl", "spec.yaml", "spec.yml"}
        )
        scorer_candidates = sorted(
            relative(path)
            for path in resolved.rglob("*")
            if not path.is_symlink()
            and path.is_file()
            and "scor" in path.name.lower()
            and path.suffix.lower() in {".py", ".dart", ".js", ".ts"}
        )
        readme = resolved / "README.md"
        readme_text = read_text(readme) if readme.is_file() and not readme.is_symlink() else None
        rows.append(
            {
                "path": relative(resolved),
                "remote": remote,
                "commit": commit,
                "worktree_clean": status == "",
                "official_remote": bool(
                    remote and "github.com/raresense/nova3d" in remote.lower()
                ),
                "structured_asset_count": len(assets),
                "structured_assets": assets,
                "paper_input_file_count": len(paper_inputs),
                "paper_input_files": paper_inputs,
                "scorer_candidate_count": len(scorer_candidates),
                "scorer_candidates": scorer_candidates,
                "readme": relative(readme) if readme_text is not None else None,
                "readme_sha256": sha256(readme) if readme_text is not None else None,
                "readme_declares_hosted_backend_closed_source": bool(
                    readme_text
                    and "hosted generation backend is (currently) closed-source"
                    in readme_text
                ),
                "readme_marks_examples_coming_soon": bool(
                    readme_text
                    and "examples/" in readme_text
                    and "coming soon" in readme_text
                ),
            }
        )
    return rows


def local_attributed_artifact_audit(output_dir: Path) -> list[str]:
    roots = [EXP_ROOT / "reference", EXP_ROOT / "runtime"]
    hits: list[str] = []
    for root in roots:
        root = contained(root, strict=True)
        for entry in root.iterdir():
            if entry.is_symlink() or not NOVA_TOKEN.search(entry.name):
                continue
            candidates = entry.rglob("*") if entry.is_dir() else (entry,)
            for path in candidates:
                if path.is_symlink() or not path.is_file():
                    continue
                if output_dir == path.parent or output_dir in path.parents:
                    continue
                if path.suffix.lower() in STRUCTURED_SUFFIXES:
                    hits.append(relative(path))
    return sorted(hits)


def common_input_audit(protocol: dict[str, object]) -> dict[str, object]:
    root = contained(COMMON_INPUT_ROOT)
    prompt_path = root / "prompt_manifest.jsonl"
    prompt_rows: list[object] = []
    prompt_errors: list[str] = []
    if not prompt_path.is_symlink() and prompt_path.is_file():
        text = read_text(prompt_path)
        if text is not None:
            for index, line in enumerate(text.splitlines(), start=1):
                if not line.strip():
                    continue
                try:
                    prompt_rows.append(json.loads(line))
                except json.JSONDecodeError as error:
                    prompt_errors.append(f"line {index}: {error.msg}")

    specs_root = root / "specs"
    specs = []
    if not specs_root.is_symlink() and specs_root.is_dir():
        specs = sorted(
            path
            for path in specs_root.iterdir()
            if not path.is_symlink()
            and path.is_file()
            and path.suffix.lower() in {".yaml", ".yml"}
        )

    glbs_root = root / "glbs"
    glbs = []
    if not glbs_root.is_symlink() and glbs_root.is_dir():
        glbs = sorted(
            path
            for path in glbs_root.iterdir()
            if not path.is_symlink()
            and path.is_file()
            and path.suffix.lower() == ".glb"
        )

    scorer_names = (
        "score_constraints.py",
        "score_paper_constraints.py",
        "constraint_scorer.py",
    )
    scorer_paths = [root / name for name in scorer_names]
    scorers = [path for path in scorer_paths if not path.is_symlink() and path.is_file()]
    required_items = int(protocol["required_item_count"])
    return {
        "root": relative(root),
        "root_exists": root.is_dir(),
        "prompt_manifest": relative(prompt_path),
        "prompt_manifest_exists": prompt_path.is_file(),
        "prompt_rows_valid_json": len(prompt_rows),
        "prompt_json_errors": prompt_errors,
        "frozen_spec_count": len(specs),
        "frozen_specs": [relative(path) for path in specs],
        "final_glb_count": len(glbs),
        "final_glbs": [relative(path) for path in glbs],
        "paper_scorer_count": len(scorers),
        "paper_scorers": [relative(path) for path in scorers],
        "required_item_count": required_items,
        "complete_item_counts": (
            not prompt_errors
            and len(prompt_rows) == required_items
            and len(specs) == required_items
            and len(glbs) == required_items
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    output_dir = contained(args.output_dir)

    protocol = json.loads(contained(PROTOCOL_PATH, strict=True).read_text(encoding="utf-8"))
    checkouts = checkout_audit()
    official_checkouts = [row for row in checkouts if row["official_remote"]]
    common_inputs = common_input_audit(protocol)
    attributed_assets = local_attributed_artifact_audit(output_dir)

    existing_preflight = EXP_ROOT / "scripts" / "preflight_nano3d_paper_constraints.py"
    operational_scorer = EXP_ROOT / "scripts" / "run_nano3d_constraints.py"
    previous_preflight = (
        EXP_ROOT / "runtime" / "nano3d_paper_constraints_preflight" / "preflight.json"
    )

    requirements = {
        "official_nova3d_checkout": bool(official_checkouts),
        "original_constrained_prompt_manifest_18_items": (
            common_inputs["prompt_rows_valid_json"] == protocol["required_item_count"]
            and not common_inputs["prompt_json_errors"]
        ),
        "frozen_specs_18_items": (
            common_inputs["frozen_spec_count"] == protocol["required_item_count"]
        ),
        "nova3d_attributed_final_glbs_18_items": (
            common_inputs["final_glb_count"] == protocol["required_item_count"]
            and len(attributed_assets) >= protocol["required_item_count"]
        ),
        "paper_measure_recipes_and_scorer": common_inputs["paper_scorer_count"] > 0,
    }
    blockers = []
    if not requirements["original_constrained_prompt_manifest_18_items"]:
        blockers.append("MISSING_FROZEN_PROMPTS")
    if not requirements["frozen_specs_18_items"]:
        blockers.append("MISSING_FROZEN_SPECS")
    if not requirements["nova3d_attributed_final_glbs_18_items"]:
        blockers.append("MISSING_NOVA3D_FINAL_GLBS")
    if not requirements["paper_measure_recipes_and_scorer"]:
        blockers.append("MISSING_PAPER_SCORER")
    if official_checkouts and not any(row["structured_asset_count"] for row in official_checkouts):
        blockers.append("OFFICIAL_CHECKOUT_HAS_NO_STRUCTURED_ASSETS")

    status = "BLOCKED" if blockers else "READY_NOT_RUN"
    summary = {
        "schema_version": 1,
        "protocol_id": "nano3d_table4_constraints_nova3d_preflight_v1",
        "baseline": "Nova3D",
        "table": "Table 4A: Paper-aligned constraint satisfaction",
        "status": status,
        "evidence_class": "PREFLIGHT_ONLY",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "safety": {
            "workspace_root": str(WORKSPACE_ROOT),
            "network_accessed": False,
            "hosted_nova3d_api_called": False,
            "secret_read": False,
            "gpu_job_started": False,
            "existing_job_touched": False,
            "paper_values_reused_as_local_metrics": False,
            "targets_inferred_from_outputs": False,
        },
        "protocol": {
            "source": relative(PROTOCOL_PATH),
            "id": protocol["protocol"],
            "required_items": protocol["required_item_count"],
            "required_constraints": protocol["required_constraint_count"],
            "required_count_constraints": protocol["required_count_constraint_count"],
            "required_numeric_constraints": protocol["required_numeric_constraint_count"],
            "sha256": sha256(PROTOCOL_PATH),
        },
        "official_checkout_audit": checkouts,
        "common_input_audit": common_inputs,
        "local_nova3d_attributed_structured_assets": attributed_assets,
        "existing_harness_audit": {
            "paper_preflight": relative(existing_preflight),
            "paper_preflight_sha256": sha256(existing_preflight),
            "previous_preflight": relative(previous_preflight),
            "previous_preflight_sha256": sha256(previous_preflight),
            "operational_constraints_runner": relative(operational_scorer),
            "operational_constraints_runner_sha256": sha256(operational_scorer),
            "operational_runner_is_paper_compatible": False,
            "reason": "It scores source-derived URDF/config/QC clauses, not frozen prompt-stated GLB dimensions and exact counts.",
        },
        "requirements": requirements,
        "blocker_codes": blockers,
        "local_evaluation": {
            "status": "NOT_RUN" if blockers else "READY_NOT_RUN",
            "display_value": "N/R",
            "items": 0,
            "constraints_scored": 0,
            "metrics": {
                "constraints": None,
                "measurable": None,
                "passed": None,
                "coverage": None,
                "satisfaction": None,
                "conditional_accuracy": None,
                "count_pass": None,
            },
            "reason": "The frozen common inputs and compatible scorer are incomplete; no output-derived spec was fabricated.",
        },
        "paper_reference_policy": {
            "paper_values_imported": False,
            "reason": "Numbers transcribed in Nano3dresults.md are paper-reported context, not a local rerun.",
        },
        "minimum_unblock_inputs": [
            "The original frozen 18-item constrained prompt manifest.",
            "One output-independent spec.yaml per item containing all 52 constraints and frozen measure recipes/tolerances.",
            "Eighteen attributable final Nova3D GLBs linked by immutable item IDs and hashes.",
            "A locally runnable paper-compatible semantic-node measurement scorer.",
        ],
        "run_command": "python exp/scripts/run_nova3d_constraints_baseline.py",
    }

    output_dir.mkdir(parents=True, exist_ok=True)
    summary_path = output_dir / "summary.json"
    summary_path.write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )

    official = official_checkouts[0] if official_checkouts else None
    checkout_line = "0"
    if official:
        checkout_line = (
            f"1 (`{official['path']}`, commit `{official['commit']}`; "
            f"structured assets {official['structured_asset_count']})"
        )
    report = f"""# Nova3D Constraints baseline

Status: **{status}** (`PREFLIGHT_ONLY`; local metrics **N/R**)

No Table 4A constraint score was produced. The run did not infer targets or
tolerances from generated geometry and did not reuse paper-reported numbers as
local measurements.

| Requirement | Available | Required |
|---|---:|---:|
| Official Nova3D checkout | {checkout_line} | 1 |
| Original constrained prompts | {common_inputs['prompt_rows_valid_json']} | 18 |
| Frozen `spec.yaml` files | {common_inputs['frozen_spec_count']} | 18 |
| Attributable final Nova3D GLBs | {common_inputs['final_glb_count']} | 18 |
| Paper-compatible scorer | {common_inputs['paper_scorer_count']} | 1 |

The frozen protocol expects 52 constraints: 32 exact-count and 20 numeric. The
official checkout is only evidence that the public client/integration source is
present; it contains no structured result assets or paper benchmark inputs. The
existing `run_nano3d_constraints.py` result is a source-derived operational
audit and is not compatible with Table 4A.

Blockers: {', '.join(blockers)}.
"""
    (output_dir / "report.md").write_text(report, encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 2 if blockers else 0


if __name__ == "__main__":
    raise SystemExit(main())
