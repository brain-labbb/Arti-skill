#!/usr/bin/env python3
"""Run the nested PV-A Table 3 expansion (30 seeds/category, N=150).

Generation is deliberately isolated from the moving ``arti-template`` checkout.
The subprocess workers import the historical worktree pinned below.  Every
terminal seed outcome is appended immediately to a journal and can be resumed.
Seeds 6--29 are never attempted until an exact replay of seeds 0--5 passes the
frozen Main30 compatibility gate.
"""

from __future__ import annotations

import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import random
from statistics import mean
import subprocess
import sys
import threading
from typing import Any, Callable


WORKSPACE = Path("/mnt/zsn/lyb").resolve(strict=True)
ARTI_SKILL = WORKSPACE / "arti-skill"
EXP_ROOT = ARTI_SKILL / "exp"
HISTORICAL_ROOT = EXP_ROOT / "baselines/arti-template-pva-expanded-vintage"
HISTORICAL_COMMIT = "9d522c2feae942a37baeafa7a808f658e20423f1"
PYTHON = ARTI_SKILL / "arti-template/.venv/bin/python"
DEFAULT_OUTPUT = EXP_ROOT / "runtime/nano3d_hierarchy_expanded_n150/pva"
OLD_ROOT = EXP_ROOT / "runtime/nano3d_hierarchy_correctness/pva"
OLD_RECORDS = OLD_ROOT / "records.json"
OLD_RECORDS_SHA256 = "b67146bd17de028f8319a9f7f6b8a2ee134deadeac53f86b2a64a652b6d1e7d6"
SHARED_TREE = EXP_ROOT / "scripts/run_nano3d_hierarchy.py"
SHARED_EXTENDED = EXP_ROOT / "scripts/hierarchy_extended_metrics.py"
SHARED_PARTNET = EXP_ROOT / "scripts/partnet_hierarchy_correctness.py"
PARTNET_PROTOCOL = EXP_ROOT / "reference/partnet_hierarchy_correctness_v1.json"
PROTOCOL_ID = "nano3d_hierarchy_pva_expanded_n150_v1"
PER_CATEGORY = 30
OLD_PER_CATEGORY = 6
BOOTSTRAP_REPLICATES = 10_000
BOOTSTRAP_SEED = 20260812
CATEGORY_TEMPLATES = {
    "storage_furniture": ("drawer_cabinet_with_sliding_drawers", "drawer_cabinet_with_sliding_drawers"),
    "table": ("folding_camp_table", "folding_camp_table"),
    "refrigerator": ("refrigerator_with_hinged_doors", "refrigerator"),
    "dishwasher": (
        "dishwasher_with_dropdown_door_and_sliding_racks",
        "dishwasher_with_dropdown_door_and_sliding_racks",
    ),
    "microwave": ("Kitchen_Microwave", "microwave"),
}
EXPECTED_TEMPLATE_SHA256 = {
    "storage_furniture": "5ae6f2673ca9d60a4a7fe6281207e241612ce404f98a0d349cdc5ddb5513a733",
    "table": "1f569e0f19d92b9ad2fd6a22cf1c325acdfe19c4ae303c5e7bd6ff02741ff410",
    "refrigerator": "5e6618c9ad99c11326a2263d7b4ef4890e99becd6f9decedb8500c106d7a042e",
    "dishwasher": "607b1e45bc8c17d16db0d9526e8104e944bf2e09600dbb05c21a74ac53cde4b3",
    "microwave": "020f661750038dcd7ea9f9041262a802e9246df23726c5b4dffb61a689079446",
}


def contained(path: Path, *, exists: bool = False) -> Path:
    resolved = path.resolve(strict=exists)
    if not resolved.is_relative_to(WORKSPACE):
        raise ValueError(f"path escapes workspace: {resolved}")
    return resolved


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    return [
        json.loads(line)
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]


def atomic_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def atomic_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in rows),
        encoding="utf-8",
    )
    temporary.replace(path)


def load_module(name: str, path: Path) -> Any:
    spec = importlib.util.spec_from_file_location(name, contained(path, exists=True))
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def git_head(path: Path) -> str:
    result = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=path,
        check=True,
        text=True,
        capture_output=True,
    )
    return result.stdout.strip()


def preflight() -> dict[str, Any]:
    historical = contained(HISTORICAL_ROOT, exists=True)
    if not PYTHON.is_file():
        raise FileNotFoundError(PYTHON)
    python = PYTHON
    if git_head(historical) != HISTORICAL_COMMIT:
        raise ValueError("historical PV-A worktree commit changed")
    status = subprocess.run(
        ["git", "status", "--porcelain=v1"],
        cwd=historical,
        check=True,
        text=True,
        capture_output=True,
    ).stdout
    if status.strip():
        raise ValueError("historical PV-A worktree is dirty")
    templates = []
    for category, (slug, stem) in CATEGORY_TEMPLATES.items():
        path = contained(historical / "agent/templates" / f"{slug}.py", exists=True)
        digest = sha256(path)
        if digest != EXPECTED_TEMPLATE_SHA256[category]:
            raise ValueError(f"historical template hash mismatch: {category}")
        templates.append(
            {
                "category": category,
                "slug": slug,
                "stem": stem,
                "template_path": str(path),
                "template_sha256": digest,
            }
        )
    old_records = contained(OLD_RECORDS, exists=True)
    old_digest = sha256(old_records)
    # This constant is checked only after it has been set from the frozen file.
    # Keeping it explicit makes later drift fail closed.
    if OLD_RECORDS_SHA256 and old_digest != OLD_RECORDS_SHA256:
        raise ValueError(
            f"frozen PV-A Main30 records changed: expected {OLD_RECORDS_SHA256}, got {old_digest}"
        )
    return {
        "historical_root": str(historical),
        "historical_commit": HISTORICAL_COMMIT,
        "historical_clean": True,
        "python": str(python),
        "old_records": str(old_records),
        "old_records_sha256": old_digest,
        "templates": templates,
        "shared_modules": {
            "tree": {"path": str(SHARED_TREE), "sha256": sha256(SHARED_TREE)},
            "extended": {"path": str(SHARED_EXTENDED), "sha256": sha256(SHARED_EXTENDED)},
            "partnet": {"path": str(SHARED_PARTNET), "sha256": sha256(SHARED_PARTNET)},
            "ontology": {"path": str(PARTNET_PROTOCOL), "sha256": sha256(PARTNET_PROTOCOL)},
        },
    }


def selection() -> list[dict[str, Any]]:
    return [
        {"category": category, "slug": slug, "stem": stem, "seed": seed}
        for category, (slug, stem) in CATEGORY_TEMPLATES.items()
        for seed in range(PER_CATEGORY)
    ]


def key(row: dict[str, Any]) -> tuple[str, int]:
    return str(row["category"]), int(row["seed"])


def terminal_path(output: Path, category: str, seed: int) -> Path:
    return output / "terminal" / category / f"seed_{seed}.json"


def valid_terminal(path: Path) -> dict[str, Any] | None:
    if not path.is_file():
        return None
    try:
        row = load_json(path)
        if not isinstance(row, dict):
            return None
        if row.get("parseable_final_urdf"):
            urdf = contained(Path(str(row["model_urdf"])), exists=True)
            if sha256(urdf) != row.get("model_urdf_sha256"):
                return None
        return row
    except (OSError, ValueError, KeyError, json.JSONDecodeError):
        return None


def worker_command(case: dict[str, Any], output: Path, timeout_seconds: float) -> dict[str, Any]:
    category = str(case["category"])
    seed = int(case["seed"])
    artifact_root = contained(output / "packages" / category)
    artifact_root.mkdir(parents=True, exist_ok=True)
    code = """
import json
from agent.template_sweep import _compile_one
outcome = _compile_one(
    %r, %r, %d, "sdk", motion_qc=True, artifact_root=%r
)
print(json.dumps(outcome.to_dict()))
""" % (case["slug"], case["stem"], seed, str(artifact_root))
    env = os.environ.copy()
    env.update(
        {
            "PYTHONPATH": str(HISTORICAL_ROOT),
            "OPENBLAS_NUM_THREADS": "1",
            "OMP_NUM_THREADS": "1",
            "MKL_NUM_THREADS": "1",
            "NUMEXPR_NUM_THREADS": "1",
        }
    )
    started = datetime.now(timezone.utc).isoformat()
    try:
        proc = subprocess.run(
            [str(PYTHON), "-c", code],
            cwd=HISTORICAL_ROOT,
            env=env,
            capture_output=True,
            text=True,
            timeout=timeout_seconds,
        )
        if proc.returncode == 0:
            outcome = json.loads(proc.stdout)
        else:
            outcome = {
                "seed": seed,
                "verdict": "fail",
                "config": {},
                "failure_type": f"subprocess_crash(rc={proc.returncode})",
                "failure_type_normalized": "subprocess_crash",
                "failure_details": proc.stderr[:4000],
                "elapsed_s": 0.0,
                "failures": [],
                "allowances": [],
                "artifact_dir": None,
                "motion_qc_report": None,
            }
    except subprocess.TimeoutExpired:
        outcome = {
            "seed": seed,
            "verdict": "fail",
            "config": {},
            "failure_type": f"compile_timeout({timeout_seconds:.0f}s)",
            "failure_type_normalized": "compile_timeout",
            "failure_details": f"per-seed compile exceeded {timeout_seconds:.0f}s",
            "elapsed_s": timeout_seconds,
            "failures": [],
            "allowances": [],
            "artifact_dir": None,
            "motion_qc_report": None,
        }
    row: dict[str, Any] = {
        "category": category,
        "slug": case["slug"],
        "stem": case["stem"],
        "seed": seed,
        "started_at_utc": started,
        "finished_at_utc": datetime.now(timezone.utc).isoformat(),
        "compile_status": "PASS" if outcome["verdict"] == "pass" else "FAIL",
        "compile_elapsed_seconds": float(outcome.get("elapsed_s") or 0.0),
        "failure_type": outcome.get("failure_type"),
        "failure_type_normalized": outcome.get("failure_type_normalized"),
        "failure_details": outcome.get("failure_details"),
        "failures": outcome.get("failures") or [],
        "allowances": outcome.get("allowances") or [],
        "config": outcome.get("config") or {},
        "artifact_dir": outcome.get("artifact_dir"),
        "motion_qc_report": outcome.get("motion_qc_report"),
        "parseable_final_urdf": False,
        "valid_tree": False,
        "has_tree": False,
        "materialization_mode": "fresh_historical_compile",
    }
    if outcome["verdict"] == "pass" and outcome.get("artifact_dir"):
        artifact = contained(Path(str(outcome["artifact_dir"])), exists=True)
        urdf = contained(artifact / "model.urdf", exists=True)
        tree = load_module("pva_expanded_tree_worker", SHARED_TREE).parse_hierarchy(urdf)
        row.update(tree)
        row.update(
            {
                "parseable_final_urdf": True,
                "model_urdf": str(urdf),
                "model_urdf_sha256": sha256(urdf),
            }
        )
    return row


def write_state(output: Path, terminal: dict[tuple[str, int], dict[str, Any]], phase: str) -> None:
    rows = sorted(terminal.values(), key=key)
    atomic_jsonl(output / "records.progress.jsonl", rows)
    atomic_json(
        output / "state.json",
        {
            "protocol_id": PROTOCOL_ID,
            "phase": phase,
            "updated_at_utc": datetime.now(timezone.utc).isoformat(),
            "terminal_count": len(rows),
            "pass_count": sum(row["compile_status"] == "PASS" for row in rows),
            "failure_count": sum(row["compile_status"] != "PASS" for row in rows),
            "remaining_count": len(CATEGORY_TEMPLATES) * PER_CATEGORY - len(rows),
            "per_category_terminal": dict(Counter(row["category"] for row in rows)),
        },
    )


def run_cases(
    cases: list[dict[str, Any]],
    output: Path,
    terminal: dict[tuple[str, int], dict[str, Any]],
    *,
    workers: int,
    timeout_seconds: float,
    phase: str,
) -> None:
    pending = [case for case in cases if key(case) not in terminal]
    if not pending:
        write_state(output, terminal, phase)
        return
    lock = threading.Lock()
    with ThreadPoolExecutor(max_workers=max(1, workers)) as executor:
        futures = {
            executor.submit(worker_command, case, output, timeout_seconds): case
            for case in pending
        }
        for future in as_completed(futures):
            case = futures[future]
            try:
                row = future.result()
            except Exception as exc:  # noqa: BLE001
                row = {
                    **case,
                    "compile_status": "FAIL",
                    "compile_elapsed_seconds": 0.0,
                    "failure_type": f"runner_error:{type(exc).__name__}",
                    "failure_type_normalized": "runner_error",
                    "failure_details": str(exc),
                    "config": {},
                    "artifact_dir": None,
                    "parseable_final_urdf": False,
                    "valid_tree": False,
                    "has_tree": False,
                    "materialization_mode": "fresh_historical_compile",
                    "finished_at_utc": datetime.now(timezone.utc).isoformat(),
                }
            with lock:
                atomic_json(terminal_path(output, str(case["category"]), int(case["seed"])), row)
                terminal[key(row)] = row
                write_state(output, terminal, phase)
                print(
                    f"[{len(terminal)}/150] {row['category']} seed={row['seed']} "
                    f"{row['compile_status']} {row.get('failure_type') or ''}",
                    flush=True,
                )


def compatibility_gate(
    output: Path, terminal: dict[tuple[str, int], dict[str, Any]]
) -> dict[str, Any]:
    old_rows = load_json(OLD_RECORDS)
    old = {key(row): row for row in old_rows}
    checks: list[dict[str, Any]] = []
    mismatch: list[dict[str, Any]] = []
    for category in CATEGORY_TEMPLATES:
        for seed in range(OLD_PER_CATEGORY):
            observed = terminal[(category, seed)]
            expected = old[(category, seed)]
            row = {
                "category": category,
                "seed": seed,
                "expected_status": expected["compile_status"],
                "observed_status": observed["compile_status"],
                "expected_failure_type": expected.get("failure_type"),
                "observed_failure_type": observed.get("failure_type"),
                "expected_urdf_sha256": expected.get("model_urdf_sha256"),
                "observed_urdf_sha256": observed.get("model_urdf_sha256"),
            }
            row["match"] = (
                row["expected_status"] == row["observed_status"]
                and row["expected_failure_type"] == row["observed_failure_type"]
                and (
                    row["expected_status"] != "PASS"
                    or row["expected_urdf_sha256"] == row["observed_urdf_sha256"]
                )
            )
            checks.append(row)
            if not row["match"]:
                mismatch.append(row)
    payload = {
        "status": "PASS" if not mismatch else "FAIL",
        "protocol_id": PROTOCOL_ID,
        "required_before_new_seeds": True,
        "checked_count": len(checks),
        "match_count": len(checks) - len(mismatch),
        "mismatch_count": len(mismatch),
        "frozen_old_records": str(OLD_RECORDS),
        "frozen_old_records_sha256": sha256(OLD_RECORDS),
        "checks": checks,
        "mismatches": mismatch,
    }
    atomic_json(output / "old_n30_compatibility_gate.json", payload)
    if mismatch:
        raise RuntimeError(f"old N30 compatibility gate failed for {len(mismatch)} cases")
    return payload


def summarize_rows(rows: list[dict[str, Any]], requested: int) -> dict[str, Any]:
    valid = [row for row in rows if row.get("parseable_final_urdf")]
    return {
        "requested_count": requested,
        "compile_pass_count": sum(row["compile_status"] == "PASS" for row in rows),
        "available_count": len(valid),
        "valid_tree_count": sum(bool(row.get("valid_tree")) for row in rows),
        "valid_tree_rate_requested": sum(bool(row.get("valid_tree")) for row in rows) / requested,
        "has_hierarchy_count": sum(bool(row.get("has_tree")) for row in rows),
        "semantic_depth_mean_available": (
            mean(float(row["semantic_depth"]) for row in valid) if valid else None
        ),
        "named_groups_mean_available": (
            mean(float(row["named_group_count"]) for row in valid) if valid else None
        ),
        "movable_joints_mean_available": (
            mean(float(row["pivot_count"]) for row in valid) if valid else None
        ),
        "failure_type_counts": dict(
            Counter(
                str(row.get("failure_type") or "unknown")
                for row in rows
                if row["compile_status"] != "PASS"
            )
        ),
    }


def quantile(values: list[float], probability: float) -> float:
    ordered = sorted(values)
    position = (len(ordered) - 1) * probability
    low = int(position)
    high = min(low + 1, len(ordered) - 1)
    weight = position - low
    return ordered[low] * (1 - weight) + ordered[high] * weight


def stratified_bootstrap(
    records: list[dict[str, Any]], functions: dict[str, Callable[[dict[str, Any]], float]]
) -> dict[str, Any]:
    populations = {
        category: [row for row in records if row["category"] == category]
        for category in CATEGORY_TEMPLATES
    }
    if any(len(rows) != PER_CATEGORY for rows in populations.values()):
        raise ValueError("bootstrap requires exactly 30 rows per category")
    estimates = {
        name: mean(mean(fn(row) for row in populations[c]) for c in CATEGORY_TEMPLATES)
        for name, fn in functions.items()
    }
    samples = {name: [] for name in functions}
    rng = random.Random(BOOTSTRAP_SEED)
    for _ in range(BOOTSTRAP_REPLICATES):
        selected = {
            category: [rng.choice(populations[category]) for _ in range(PER_CATEGORY)]
            for category in CATEGORY_TEMPLATES
        }
        for name, fn in functions.items():
            samples[name].append(
                mean(mean(fn(row) for row in selected[c]) for c in CATEGORY_TEMPLATES)
            )
    return {
        "design": "category-stratified asset bootstrap; 30 draws/category; equal category macro",
        "replicates": BOOTSTRAP_REPLICATES,
        "seed": BOOTSTRAP_SEED,
        "interval": "95% percentile",
        "metrics": {
            name: {
                "estimate": estimates[name],
                "ci95_percentile": [
                    quantile(samples[name], 0.025),
                    quantile(samples[name], 0.975),
                ],
            }
            for name in functions
        },
    }


def finalize(output: Path, terminal: dict[tuple[str, int], dict[str, Any]], pre: dict[str, Any]) -> None:
    rows = sorted(terminal.values(), key=key)
    if len(rows) != len(CATEGORY_TEMPLATES) * PER_CATEGORY:
        raise ValueError("cannot finalize before all 150 terminal outcomes exist")
    records_path = output / "records.jsonl"
    atomic_jsonl(records_path, rows)
    evaluation: list[dict[str, Any]] = []
    for row in rows:
        available = bool(row.get("parseable_final_urdf"))
        score_row = {
            "method": "PV-A",
            "sample_id": f"{row['category']}/seed_{row['seed']}",
            "category": row["category"],
            "urdf_path": row.get("model_urdf") if available else None,
            "urdf_sha256": row.get("model_urdf_sha256") if available else None,
            "available": available,
            "selection_rank": int(row["seed"]) + 1,
        }
        if not available:
            score_row["unavailable_reason"] = (
                f"{row.get('failure_type', 'compile failure')}: "
                f"{row.get('failure_details', 'no final URDF')}"
            )
        evaluation.append(score_row)
    evaluation_path = output / "evaluation_manifest.jsonl"
    atomic_jsonl(evaluation_path, evaluation)

    extended = load_module("pva_expanded_extended", SHARED_EXTENDED)
    extended_rows = []
    for row in rows:
        if not row.get("parseable_final_urdf"):
            continue
        extended_rows.append(
            {
                "sample_id": f"{row['category']}/seed_{row['seed']}",
                "category": row["category"],
                "seed": row["seed"],
                "model_urdf": row["model_urdf"],
                "model_urdf_sha256": row["model_urdf_sha256"],
                **extended.analyze_urdf(Path(row["model_urdf"])),
            }
        )
    extended_path = output / "extended_structure_records.jsonl"
    atomic_jsonl(extended_path, extended_rows)
    per_category_extended = {
        category: {
            "metrics": extended.aggregate(
                [row for row in extended_rows if row["category"] == category], PER_CATEGORY
            ),
            "topology": extended.topology_consistency(
                [row for row in extended_rows if row["category"] == category]
            ),
        }
        for category in CATEGORY_TEMPLATES
    }
    extended_summary = {
        "protocol_id": PROTOCOL_ID,
        "requested_count": len(rows),
        "requested_per_category": PER_CATEGORY,
        "available_parseable_count": len(extended_rows),
        "conditioning": "descriptor means use valid trees; requested rates retain failures",
        "overall": extended.aggregate(extended_rows, len(rows)),
        "per_category": per_category_extended,
        "records_path": str(extended_path),
        "records_sha256": sha256(extended_path),
    }
    atomic_json(output / "extended_structure_summary.json", extended_summary)

    partnet = load_module("pva_expanded_partnet", SHARED_PARTNET)
    protocol = partnet.load_protocol(PARTNET_PROTOCOL)
    score_records = []
    for source in evaluation:
        score = dict(source)
        score["evaluation_complete"] = False
        if source["available"]:
            try:
                score.update(
                    partnet.evaluate_urdf(
                        Path(str(source["urdf_path"])), str(source["category"]), protocol
                    )
                )
                score["evaluation_complete"] = True
                score["evaluation_error"] = None
            except Exception as exc:  # noqa: BLE001
                score["evaluation_error"] = f"{type(exc).__name__}: {exc}"
        else:
            score["evaluation_error"] = source["unavailable_reason"]
        score_records.append(score)
    score_dir = output / "partnet_scores"
    score_dir.mkdir(parents=True, exist_ok=True)
    score_records_path = score_dir / "records.jsonl"
    atomic_jsonl(score_records_path, score_records)
    score_bootstrap = stratified_bootstrap(
        score_records,
        {
            "role_coverage_requested": lambda row: float(
                row.get("semantic_role_coverage") or 0.0
            ),
            "scorable_coverage_requested": lambda row: float(bool(row.get("scorable"))),
            "induced_edge_f1_requested": lambda row: float(
                row.get("parent_child_edge_f1") or 0.0
            ),
            "coverage_weighted_induced_edge_f1": lambda row: float(
                row.get("parent_child_edge_f1") or 0.0
            )
            * float(row.get("semantic_role_coverage") or 0.0),
            "induced_exact_requested": lambda row: float(
                bool(row.get("hierarchy_exact_match"))
            ),
            "semantic_parent_alignment_requested": lambda row: float(
                row.get("semantic_nesting_accuracy") or 0.0
            ),
        },
    )
    score_summary = {
        "protocol_id": PROTOCOL_ID,
        "method": "PV-A",
        "claim_boundary": protocol["claim_boundary"],
        "requested_per_category": PER_CATEGORY,
        "requested_count": len(score_records),
        "selection_manifest_sha256": sha256(evaluation_path),
        "partnet_protocol_sha256": sha256(PARTNET_PROTOCOL),
        "scorer_core": str(SHARED_PARTNET),
        "scorer_sha256": sha256(SHARED_PARTNET),
        "runner_mode": "direct import of frozen shared scorer core",
        "overall": partnet.aggregate(score_records),
        "per_category": {
            category: partnet.aggregate(
                [row for row in score_records if row["category"] == category]
            )
            for category in CATEGORY_TEMPLATES
        },
        "bootstrap": score_bootstrap,
    }
    score_summary_path = score_dir / "summary.json"
    atomic_json(score_summary_path, score_summary)

    old_rows = load_json(OLD_RECORDS)
    old_keys = {key(row) for row in old_rows}
    nested_rows = [row for row in rows if key(row) in old_keys]
    old_n30 = {
        "status": "PASS" if len(nested_rows) == 30 else "FAIL",
        "nested_count": len(nested_rows),
        "expected_count": 30,
        "summary_recomputed_from_expanded_prefix": summarize_rows(nested_rows, 30),
        "frozen_old_summary": load_json(OLD_ROOT / "summary.json"),
        "compatibility_gate_sha256": sha256(output / "old_n30_compatibility_gate.json"),
    }
    atomic_json(output / "old_n30_nested_validation.json", old_n30)

    structure_bootstrap = stratified_bootstrap(
        rows,
        {
            "compile_success_requested": lambda row: float(row["compile_status"] == "PASS"),
            "valid_tree_requested": lambda row: float(bool(row.get("valid_tree"))),
            "has_hierarchy_requested": lambda row: float(bool(row.get("has_tree"))),
        },
    )
    summary = {
        "status": "COMPLETE_WITH_FAILURES"
        if any(row["compile_status"] != "PASS" for row in rows)
        else "COMPLETE",
        "protocol_id": PROTOCOL_ID,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "claim_boundary": (
            "expanded deterministic PV-A template-seed audit from the pinned historical "
            "code snapshot; PartNet measurements remain ontology proxies"
        ),
        "evaluation_unit": "one frozen PV-A category-template and seed",
        "selection": {
            "categories": list(CATEGORY_TEMPLATES),
            "seeds": list(range(PER_CATEGORY)),
            "requested_per_category": PER_CATEGORY,
            "requested_count": len(rows),
            "original_n30_is_exact_seed_prefix": True,
            "failures_not_replaced": True,
        },
        "provenance": pre,
        "materialization": {
            "overall": summarize_rows(rows, len(rows)),
            "per_category": {
                category: summarize_rows(
                    [row for row in rows if row["category"] == category], PER_CATEGORY
                )
                for category in CATEGORY_TEMPLATES
            },
            "timeout_seconds": 180,
        },
        "structure": extended_summary,
        "structure_bootstrap": structure_bootstrap,
        "alignment": score_summary,
        "old_n30_nested_validation": old_n30,
        "hashes": {
            "records": sha256(records_path),
            "evaluation_manifest": sha256(evaluation_path),
            "extended_records": sha256(extended_path),
            "extended_summary": sha256(output / "extended_structure_summary.json"),
            "alignment_records": sha256(score_records_path),
            "alignment_summary": sha256(score_summary_path),
            "runner": sha256(Path(__file__)),
        },
    }
    atomic_json(output / "summary.json", summary)
    report = [
        "# PV-A Table 3 expanded-N audit",
        "",
        f"Status: **{summary['status']}**",
        "",
        "The frozen nested cohort contains seeds 0--29 in each of five categories (N=150).",
        "Seeds 0--5 replay the original Main30 exactly; failures are retained without replacement.",
        "",
        "| Category | Requested | Compile pass | Parseable | Valid tree |",
        "|---|---:|---:|---:|---:|",
    ]
    for category in CATEGORY_TEMPLATES:
        row = summary["materialization"]["per_category"][category]
        report.append(
            f"| {category} | 30 | {row['compile_pass_count']} | {row['available_count']} | "
            f"{row['valid_tree_count']} |"
        )
    weighted = score_bootstrap["metrics"]["coverage_weighted_induced_edge_f1"]
    report.extend(
        [
            "",
            f"Coverage-Weighted Induced Edge F1: {100 * weighted['estimate']:.2f}% "
            f"[{100 * weighted['ci95_percentile'][0]:.2f}%, "
            f"{100 * weighted['ci95_percentile'][1]:.2f}%].",
            "",
            "PartNet scores are category-ontology alignment proxies, not instance-level kinematic gold.",
            "",
        ]
    )
    (output / "report.md").write_text("\n".join(report), encoding="utf-8")
    write_state(output, terminal, "complete")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--timeout-seconds", type=float, default=180.0)
    parser.add_argument("--preflight-only", action="store_true")
    parser.add_argument("--gate-only", action="store_true")
    parser.add_argument("--finalize-only", action="store_true")
    args = parser.parse_args()
    output = contained(args.output)
    output.mkdir(parents=True, exist_ok=True)
    pre = preflight()
    atomic_json(output / "preflight.json", pre)
    frozen = selection()
    manifest_path = output / "frozen_manifest.jsonl"
    if manifest_path.exists() and load_jsonl(manifest_path) != frozen:
        raise ValueError("existing selection manifest differs from deterministic selection")
    atomic_jsonl(manifest_path, frozen)
    if args.preflight_only:
        print(json.dumps(pre, indent=2, sort_keys=True))
        return 0

    terminal: dict[tuple[str, int], dict[str, Any]] = {}
    for case in frozen:
        row = valid_terminal(terminal_path(output, str(case["category"]), int(case["seed"])))
        if row is not None:
            terminal[key(row)] = row
    if args.finalize_only:
        compatibility_gate(output, terminal)
        finalize(output, terminal, pre)
        return 0

    old_cases = [case for case in frozen if int(case["seed"]) < OLD_PER_CATEGORY]
    run_cases(
        old_cases,
        output,
        terminal,
        workers=args.workers,
        timeout_seconds=args.timeout_seconds,
        phase="old_n30_compatibility",
    )
    compatibility_gate(output, terminal)
    if args.gate_only:
        print(json.dumps(load_json(output / "old_n30_compatibility_gate.json"), indent=2))
        return 0

    new_cases = [case for case in frozen if int(case["seed"]) >= OLD_PER_CATEGORY]
    run_cases(
        new_cases,
        output,
        terminal,
        workers=args.workers,
        timeout_seconds=args.timeout_seconds,
        phase="expanded_generation",
    )
    finalize(output, terminal, pre)
    print(json.dumps(load_json(output / "summary.json"), indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
