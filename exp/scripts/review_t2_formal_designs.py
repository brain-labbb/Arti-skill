#!/usr/bin/env python3
"""Complete and validate the 12 frozen T2 TemplateDesign scaffolds with Codex."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


EXP_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = EXP_ROOT.parent
TEMPLATE_ROOT = PROJECT_ROOT / "arti-template"
PREP = EXP_ROOT / "runtime/t2_formal_v1/preparation"
DESIGNS = PREP / "designs"
SCAFFOLDS = PREP / "design_scaffolds"
LOGS = PREP / "design_logs"
PYTHON = TEMPLATE_ROOT / ".venv/bin/python"
CODEX = Path("/mnt/zsn/miniconda3/bin/codex")
SLUGS = (
    "flip_phone",
    "glove_compartment_door",
    "flatbed_scanner_with_hinged_lid",
    "clamp_meter_with_hinged_jaw_and_rotary_selector",
    "garden_gate",
    "bicycle_dropper_seatpost_assembly",
    "air_purifier_with_filter_door",
    "instrument_case_with_hinged_lid",
    "adjustable_weight_bench_with_hinged_backrest",
    "folding_kick_scooter",
    "extension_ladder",
    "dock_loading_ramp",
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def protected_hashes() -> dict[str, str]:
    paths = sorted((PREP / "source_maps").glob("*.md"))
    paths.extend(sorted((PREP / "records").rglob("*.json")))
    return {str(path.relative_to(PREP)): sha256(path) for path in paths}


def validate(design_path: Path, scaffold_path: Path) -> dict[str, Any]:
    import sys

    sys.path.insert(0, str(TEMPLATE_ROOT))
    from agent.template_design import load_template_design

    design = load_template_design(design_path)
    payload = json.loads(design_path.read_text(encoding="utf-8"))
    scaffold = json.loads(scaffold_path.read_text(encoding="utf-8"))
    problems: list[str] = []
    scaffold_slots = {
        slot["name"]: {
            candidate["name"]: {
                key: candidate.get(key)
                for key in (
                    "record_id",
                    "revision",
                    "source_spans",
                    "evidence",
                    "implementation_function",
                )
            }
            for candidate in slot["candidates"]
        }
        for slot in scaffold["slots"]
    }
    design_slots = {slot["name"]: slot for slot in payload["slots"]}
    if set(design_slots) != set(scaffold_slots):
        problems.append("slot names differ from frozen scaffold")
    candidate_pairs: set[tuple[str, str]] = set()
    interface_names: set[str] = set()
    parameter_count = 0
    interface_count = 0
    for slot_name, frozen_candidates in scaffold_slots.items():
        slot = design_slots.get(slot_name, {})
        actual_candidates = {
            candidate["name"]: candidate for candidate in slot.get("candidates", [])
        }
        if set(actual_candidates) != set(frozen_candidates):
            problems.append(f"{slot_name}: candidate names differ from frozen scaffold")
            continue
        for candidate_name, frozen_fields in frozen_candidates.items():
            candidate = actual_candidates[candidate_name]
            candidate_pairs.add((slot_name, candidate_name))
            for key, expected in frozen_fields.items():
                if candidate.get(key) != expected:
                    problems.append(f"{slot_name}.{candidate_name}: protected {key} changed")
            if not candidate.get("parameters"):
                problems.append(f"{slot_name}.{candidate_name}: no grounded parameters")
            parameter_count += len(candidate.get("parameters") or [])
            for interface in candidate.get("interfaces") or []:
                interface_names.add(f"{slot_name}.{interface.get('name')}")
                interface_count += 1
    covered: set[tuple[str, str]] = set()
    for anchor in payload.get("category_anchors") or []:
        applies = anchor.get("applies_to") or {}
        for candidate in applies.get("candidates") or []:
            covered.add((str(applies.get("slot") or ""), str(candidate)))
    missing = sorted(candidate_pairs - covered)
    if missing:
        problems.append(f"category anchors do not cover {len(missing)} candidates")
    if not payload.get("category_anchors"):
        problems.append("category_anchors is empty")
    if not str(payload.get("assembly_notes") or "").strip():
        problems.append("assembly_notes is empty")
    for binding in payload.get("bindings") or []:
        if binding.get("provider") not in interface_names:
            problems.append(f"binding {binding.get('binding_id')}: unknown provider")
        if binding.get("consumer") not in interface_names:
            problems.append(f"binding {binding.get('binding_id')}: unknown consumer")
    return {
        "valid": not problems,
        "problems": problems,
        "slot_count": len(design.slots),
        "candidate_count": len(candidate_pairs),
        "parameter_count": parameter_count,
        "interface_count": interface_count,
        "binding_count": len(design.bindings),
        "multiplicity_count": len(design.multiplicities),
        "anchor_count": len(design.category_anchors),
    }


def prompt(slug: str, design: Path, scaffold: Path) -> str:
    return f"""
Complete the frozen TemplateDesign for the formal unseen-category experiment `{slug}`.
This is an experiment-model action, and your trace will be retained. Work only inside
`{PREP}`. Read these authoritative references first:
- `{TEMPLATE_ROOT / 'articraft_template_authoring/AUTHORING.md'}` section 3
- `{TEMPLATE_ROOT / 'articraft_template_authoring/MECHANICAL_PRIORS.md'}`
- `{TEMPLATE_ROOT / 'articraft_template_authoring/VISUAL_DIVERSITY_MODEL.md'}`
- `{TEMPLATE_ROOT / 'agent/template_design.py'}`
Then read `source_maps/{slug}.md`, its referenced frozen records under `records/`, the
immutable scaffold `{scaffold}`, and the working file `{design}`.

Edit only `{design}`. Preserve the exact slots, candidates, record_id, revision,
source_spans, evidence, and implementation_function from the scaffold. Ground all new
decisions in the frozen records; do not inspect existing templates or experiment results.
For every candidate add real independent/derived parameters with units/ranges/expressions
and rationale. Add mechanically explicit plane/axis interfaces where components connect,
bindings across separable slots, true multiplicities where count variation is supported,
and category anchors covering every candidate with roles, joint relations, and named
author checks. Whole-assembly-family slots may encode internal interfaces and require no
cross-slot binding. Fill concise assembly_notes including frame convention, articulation
layout, adaptation rules, and collision/clearance invariants. Never copy source code.

Before finishing, run this exact validation import:
`PYTHONPATH={TEMPLATE_ROOT} {PYTHON} -c "from pathlib import Path; from agent.template_design import load_template_design; d=load_template_design(Path(r'{design}')); print(len(d.slots))"`
Report counts and uncertainties only. Do not modify any other file.
""".strip()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--only",
        help="comma-separated non-overlapping slug shard; defaults to all slugs",
    )
    args = parser.parse_args()
    selected = tuple(args.only.split(",")) if args.only else SLUGS
    unknown = sorted(set(selected) - set(SLUGS))
    if unknown:
        parser.error(f"unknown slugs: {unknown}")
    LOGS.mkdir(parents=True, exist_ok=True)
    SCAFFOLDS.mkdir(parents=True, exist_ok=True)
    before = protected_hashes()
    results: list[dict[str, Any]] = []
    for slug in selected:
        design = DESIGNS / f"{slug}.json"
        scaffold = SCAFFOLDS / f"{slug}.json"
        if not scaffold.exists():
            shutil.copy2(design, scaffold)
        run_path = LOGS / f"{slug}_run.json"
        if run_path.is_file():
            prior = json.loads(run_path.read_text(encoding="utf-8"))
            if prior.get("passed") and validate(design, scaffold)["valid"]:
                results.append(prior)
                print(json.dumps({"slug": slug, "status": "already_passed"}), flush=True)
                continue
        started = datetime.now(timezone.utc).isoformat()
        print(json.dumps({"slug": slug, "status": "starting"}), flush=True)
        completed = subprocess.run(
            [
                str(CODEX),
                "exec",
                "--json",
                "--model",
                "gpt-5.6-sol",
                "-c",
                "model_reasoning_effort='high'",
                "--dangerously-bypass-approvals-and-sandbox",
                "--skip-git-repo-check",
                prompt(slug, design, scaffold),
            ],
            cwd=PREP,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
            timeout=7200,
        )
        (LOGS / f"{slug}_events.jsonl").write_text(completed.stdout, encoding="utf-8")
        (LOGS / f"{slug}_stderr.txt").write_text(completed.stderr, encoding="utf-8")
        validation: dict[str, Any]
        try:
            validation = validate(design, scaffold)
        except Exception as exc:  # noqa: BLE001
            validation = {"valid": False, "problems": [f"{type(exc).__name__}: {exc}"]}
        changed = [
            path for path, digest in before.items()
            if not (PREP / path).is_file() or sha256(PREP / path) != digest
        ]
        record = {
            "slug": slug,
            "model": "gpt-5.6-sol",
            "reasoning_effort": "high",
            "started_at": started,
            "finished_at": datetime.now(timezone.utc).isoformat(),
            "process_exit_code": completed.returncode,
            "design_sha256": sha256(design),
            "protected_files_changed": changed,
            "validation": validation,
            "passed": completed.returncode == 0 and validation["valid"] and not changed,
        }
        run_path.write_text(json.dumps(record, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        results.append(record)
        print(json.dumps(record, ensure_ascii=False), flush=True)
        if not record["passed"]:
            break
    all_results = []
    for slug in SLUGS:
        run_path = LOGS / f"{slug}_run.json"
        if run_path.is_file():
            all_results.append(json.loads(run_path.read_text(encoding="utf-8")))
    summary = {
        "schema_version": 1,
        "reviewed": len(all_results),
        "passed": sum(row.get("passed", False) for row in all_results),
        "expected": len(SLUGS),
        "all_passed": len(all_results) == len(SLUGS) and all(row.get("passed") for row in all_results),
        "selected_shard": selected,
        "results": all_results,
    }
    (LOGS / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0 if summary["all_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
