#!/usr/bin/env python3
"""Fail-closed preflight for a Nova3D-paper-aligned constraint experiment."""

from __future__ import annotations

import json
from pathlib import Path


EXP = Path("/mnt/zsn/lyb/arti-skill/exp").resolve()
PROTOCOL = EXP / "reference/paper_constraints_protocol_v1.json"
INPUT = EXP / "reference/paper_constraints"
OUTPUT = EXP / "runtime/nano3d_paper_constraints_preflight"


def main() -> int:
    protocol = json.loads(PROTOCOL.read_text(encoding="utf-8"))
    prompts = INPUT / "prompt_manifest.jsonl"
    specs = sorted((INPUT / "specs").glob("*.yaml")) if (INPUT / "specs").is_dir() else []
    glbs = sorted((INPUT / "glbs").glob("*.glb")) if (INPUT / "glbs").is_dir() else []
    prompt_rows = []
    if prompts.is_file():
        prompt_rows = [json.loads(line) for line in prompts.read_text(encoding="utf-8").splitlines() if line.strip()]
    checks = {
        "prompt_manifest_exists": prompts.is_file(),
        "prompt_item_count": len(prompt_rows),
        "spec_count": len(specs),
        "glb_count": len(glbs),
        "required_item_count": protocol["required_item_count"],
        "required_constraint_count": protocol["required_constraint_count"],
        "required_count_constraint_count": protocol["required_count_constraint_count"],
        "required_numeric_constraint_count": protocol["required_numeric_constraint_count"],
    }
    ready = (
        len(prompt_rows) == protocol["required_item_count"]
        and len(specs) == protocol["required_item_count"]
        and len(glbs) == protocol["required_item_count"]
    )
    result = {
        "schema_version": 1,
        "protocol": protocol["protocol"],
        "status": "ready" if ready else "blocked_missing_paper_benchmark_inputs",
        "checks": checks,
        "missing": [
            name for name, present in {
                "original constrained prompt manifest": len(prompt_rows) == protocol["required_item_count"],
                "18 frozen spec.yaml files": len(specs) == protocol["required_item_count"],
                "18 matching final GLB files": len(glbs) == protocol["required_item_count"],
                "paper scorer/measure recipes": False,
            }.items() if not present
        ],
        "paper_metrics": {
            "constraints": None, "measurable": None, "passed": None,
            "coverage": None, "satisfaction": None,
            "conditional_accuracy": None, "count_pass": None,
        },
        "reason": "Targets cannot be inferred from generated outputs without violating the paper's pre-freeze protocol.",
    }
    OUTPUT.mkdir(parents=True, exist_ok=True)
    (OUTPUT / "preflight.json").write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    report = [
        "# Nova3D paper-aligned Constraints preflight", "",
        f"Status: **{result['status']}**", "",
        "| Requirement | Available | Required |", "|---|---:|---:|",
        f"| Original constrained prompts | {len(prompt_rows)} | 18 |",
        f"| Frozen `spec.yaml` | {len(specs)} | 18 |",
        f"| Matching final GLB | {len(glbs)} | 18 |",
        "| Public paper scorer/measure recipes | 0 | 1 release |", "",
        "Paper-aligned metrics remain N/A. Generated geometry was not used to invent targets.", "",
    ]
    (OUTPUT / "report.md").write_text("\n".join(report), encoding="utf-8")
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0 if ready else 2


if __name__ == "__main__":
    raise SystemExit(main())
