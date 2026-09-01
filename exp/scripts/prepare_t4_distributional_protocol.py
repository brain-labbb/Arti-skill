#!/usr/bin/env python3
"""Freeze valid 18-task x 16-seed edit cases before expensive T4 execution."""

from __future__ import annotations

import argparse
import dataclasses
import hashlib
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
EXP_ROOT = SCRIPT_DIR.parent
PROJECT_ROOT = EXP_ROOT.parent
TEMPLATE_ROOT = PROJECT_ROOT / "arti-template"
DEFAULT_OUT = EXP_ROOT / "runtime" / "t4_distributional_protocol_v1"
SEEDS_PER_TASK = 16
MAX_SEED_SEARCH = 4096

sys.path.insert(0, str(TEMPLATE_ROOT))
sys.path.insert(0, str(SCRIPT_DIR))

from run_nano3d_paper_editability import TASKS, task_module  # noqa: E402


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def domain_values(module: Any, field: str) -> list[Any]:
    for slot in module.TEMPLATE_DOMAIN.slots:
        if slot.name == field:
            return list(slot.values)
    raise ValueError(f"domain field not found: {field}")


def proposed_edit(module: Any, task: dict[str, Any], seed: int) -> dict[str, Any] | None:
    config = module.config_from_seed(seed)
    old = getattr(config, task["field"])
    values = domain_values(module, task["field"])
    if task["edit_class"] == "additive":
        if task["task_id"] == "E001":
            if old != "none":
                return None
            new = task["target_value"]
            policy = "none_to_frozen_additive_component"
        else:
            if isinstance(old, bool) or not isinstance(old, int):
                raise TypeError(f"{task['task_id']}: additive count field is not integer")
            new = old + 1
            if new not in values:
                return None
            policy = "increment_count_by_one"
    else:
        new = task["target_value"]
        if old == new:
            return None
        policy = "replace_with_frozen_candidate"
    if new not in values:
        raise ValueError(f"{task['task_id']}: proposed value absent from domain: {new!r}")
    return {
        "seed": seed,
        "base_value": old,
        "edited_value": new,
        "edit_policy": policy,
        "base_config": dataclasses.asdict(config),
    }


def freeze_cases() -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    tasks: list[dict[str, Any]] = []
    cases: list[dict[str, Any]] = []
    for task in TASKS:
        module = task_module(task)
        selected: list[dict[str, Any]] = []
        for seed in range(MAX_SEED_SEARCH):
            edit = proposed_edit(module, task, seed)
            if edit is None:
                continue
            selected.append(edit)
            if len(selected) == SEEDS_PER_TASK:
                break
        if len(selected) != SEEDS_PER_TASK:
            raise RuntimeError(
                f"{task['task_id']}: found only {len(selected)} valid seeds before {MAX_SEED_SEARCH}"
            )
        template_path = TEMPLATE_ROOT / "agent" / "templates" / f"{task['slug']}.py"
        task_row = {
            **task,
            "selected_seeds": [row["seed"] for row in selected],
            "template_path": str(template_path.relative_to(PROJECT_ROOT)),
            "template_sha256": sha256_file(template_path),
            "semantic_partition_status": "pending_independent_freeze",
            "target_roles": None,
            "allowed_dependent_roles": None,
            "true_non_target_roles": None,
        }
        tasks.append(task_row)
        for row in selected:
            cases.append(
                {
                    "case_id": f"{task['task_id']}__seed_{row['seed']}",
                    "task_id": task["task_id"],
                    "slug": task["slug"],
                    "edit_class": task["edit_class"],
                    "field": task["field"],
                    **row,
                }
            )
    return tasks, cases


def dump_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True, default=str) + "\n",
        encoding="utf-8",
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    args = parser.parse_args()
    output = args.out.resolve()
    output.relative_to(EXP_ROOT.resolve())
    tasks, cases = freeze_cases()
    tasks = json.loads(json.dumps(tasks, ensure_ascii=False, default=str))
    manifest_without_time = {
        "schema_version": 1,
        "protocol": "t4_distributional_editability_18x16_v1",
        "task_count": len(tasks),
        "seeds_per_task": SEEDS_PER_TASK,
        "case_count": len(cases),
        "seed_selection": (
            "First 16 non-negative seeds satisfying the edit precondition. Additive integer "
            "tasks increment the base count by one; replacement tasks exclude seeds already "
            "containing the target candidate."
        ),
        "tasks": tasks,
        "source_task_script": str(
            (SCRIPT_DIR / "run_nano3d_paper_editability.py").relative_to(PROJECT_ROOT)
        ),
        "source_task_script_sha256": sha256_file(
            SCRIPT_DIR / "run_nano3d_paper_editability.py"
        ),
    }
    manifest_path = output / "protocol_manifest.json"
    if manifest_path.exists():
        existing = json.loads(manifest_path.read_text(encoding="utf-8"))
        comparable = {key: value for key, value in existing.items() if key != "frozen_at"}
        if comparable != manifest_without_time:
            raise RuntimeError("existing protocol manifest differs from current frozen inputs")
    else:
        dump_json(
            manifest_path,
            {**manifest_without_time, "frozen_at": datetime.now(timezone.utc).isoformat()},
        )
    cases_path = output / "cases.jsonl"
    expected_cases = "".join(
        json.dumps(row, ensure_ascii=False, sort_keys=True, default=str) + "\n"
        for row in cases
    )
    if cases_path.exists():
        if cases_path.read_text(encoding="utf-8") != expected_cases:
            raise RuntimeError("existing case manifest differs from current frozen inputs")
    else:
        cases_path.parent.mkdir(parents=True, exist_ok=True)
        cases_path.write_text(expected_cases, encoding="utf-8")
    readiness = {
        "schema_version": 1,
        "protocol": "t4_distributional_editability_18x16_v1",
        "valid_edit_cases": len(cases),
        "expected_edit_cases": 18 * 16,
        "edit_preconditions_ready": len(cases) == 18 * 16,
        "execution_ready": False,
        "blocking_inputs": [
            "Independent per-task target/allowed-dependent/true-non-target role partitions",
            "Frozen pre-edit regression manifest",
            "Two blinded reviewers plus adjudication protocol",
        ],
        "note": (
            "Compilation before semantic partitions are frozen would produce artifacts but would "
            "not make Non-Target Preservation, Locality, Regression, or Final Pass measurable."
        ),
    }
    dump_json(output / "readiness.json", readiness)
    print(json.dumps(readiness, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
