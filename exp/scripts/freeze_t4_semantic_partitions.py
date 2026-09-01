#!/usr/bin/env python3
"""Freeze T4 semantic partitions and baseline tests before artifact compilation.

The frozen unit is one of the already selected 18 x 16 edit cases.  Both SDK
object graphs are built in isolated processes, but no URDF/GLB artifact is
compiled here.  Target membership follows the predeclared task tokens plus
added/removed source parts.  Stable source parts whose SDK declarations change
as a direct consequence of the one-field edit are frozen as allowed
dependents; all remaining stable parts are true non-targets.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import dataclasses
import hashlib
import importlib
import json
import os
import subprocess
import sys
import time
from datetime import datetime, timezone
from enum import Enum
from pathlib import Path
from typing import Any


EXP_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = EXP_ROOT.parent
TEMPLATE_ROOT = PROJECT_ROOT / "arti-template"
PROTOCOL_ROOT = EXP_ROOT / "runtime/t4_distributional_protocol_v1"
DEFAULT_OUT = EXP_ROOT / "runtime/t4_formal_v1/frozen_partitions_v8"
PYTHON = TEMPLATE_ROOT / ".venv/bin/python"

sys.path.insert(0, str(TEMPLATE_ROOT))
sys.path.insert(0, str(EXP_ROOT / "scripts"))


def dump_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True, default=str) + "\n",
        encoding="utf-8",
    )


def digest_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def normalize(value: Any) -> Any:
    try:
        import numpy as np

        if isinstance(value, np.ndarray):
            contiguous = np.ascontiguousarray(value)
            return {
                "ndarray_shape": list(contiguous.shape),
                "ndarray_dtype": str(contiguous.dtype),
                "ndarray_sha256": hashlib.sha256(contiguous.tobytes()).hexdigest(),
            }
    except ImportError:
        pass
    if hasattr(value, "vertices") and hasattr(value, "faces"):
        # trimesh and SDK mesh-data objects: retain exact in-memory geometry
        # without serializing millions of coordinates into the manifest.
        return {
            "mesh_vertices": normalize(value.vertices),
            "mesh_faces": normalize(value.faces),
        }
    if dataclasses.is_dataclass(value):
        result = {}
        for field in dataclasses.fields(value):
            # AssetContext.root is deliberately different for the isolated
            # baseline and edited source graphs.  It is an evaluation scratch
            # location, not a semantic or geometric property of the Part.
            # Including it makes every stable Part look edited and invalidates
            # the frozen partition.
            if field.name in {"assets", "materialized_path"}:
                continue
            result[field.name] = normalize(getattr(value, field.name))
        return result
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, dict):
        return {str(key): normalize(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [normalize(item) for item in value]
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return str(value)


def file_references(value: Any) -> list[str]:
    refs: list[str] = []
    if dataclasses.is_dataclass(value):
        for field in dataclasses.fields(value):
            if field.name in {"assets", "materialized_path", "source_geometry"}:
                continue
            refs.extend(file_references(getattr(value, field.name)))
    elif isinstance(value, dict):
        for item in value.values():
            refs.extend(file_references(item))
    elif isinstance(value, (list, tuple)):
        for item in value:
            refs.extend(file_references(item))
    elif isinstance(value, str) and value.lower().endswith(
        (".obj", ".stl", ".ply", ".glb", ".gltf", ".png", ".jpg", ".jpeg")
    ):
        refs.append(value)
    return refs


def source_signature(value: Any, source_dir: Path) -> str:
    payload = normalize(value)
    hashes: dict[str, str | None] = {}
    for ref in sorted(set(file_references(value))):
        candidate = Path(ref)
        if not candidate.is_absolute():
            candidate = source_dir / candidate
        hashes[ref] = digest_file(candidate) if candidate.is_file() else None
    encoded = json.dumps(
        {"declaration": payload, "referenced_file_sha256": hashes},
        ensure_ascii=False,
        sort_keys=True,
        default=str,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def load_cases() -> list[dict[str, Any]]:
    return [
        json.loads(line)
        for line in (PROTOCOL_ROOT / "cases.jsonl").read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]


def task_by_id() -> dict[str, dict[str, Any]]:
    manifest = json.loads((PROTOCOL_ROOT / "protocol_manifest.json").read_text(encoding="utf-8"))
    return {row["task_id"]: row for row in manifest["tasks"]}


def build_one(module: Any, task: dict[str, Any], config: Any, source_dir: Path) -> Any:
    from sdk import AssetContext

    source_dir.mkdir(parents=True, exist_ok=True)
    source_path = source_dir / "source.py"
    source_path.touch(exist_ok=True)
    model = getattr(module, f"build_{task['stem']}")(
        config, assets=AssetContext.from_script(source_path)
    )
    return model


def observe_test(module: Any, task: dict[str, Any], model: Any, config: Any) -> dict[str, Any]:
    try:
        report = getattr(module, f"run_{task['stem']}_tests")(model, config)
        return report_payload(report)
    except Exception as exc:  # noqa: BLE001
        return {
            "passed": None,
            "checks_run": None,
            "checks": [],
            "failures": [],
            "warnings": [],
            "allowances": [],
            "precompile_unavailable": f"{type(exc).__name__}: {exc}",
            "deferred_to_formal_compile": True,
        }


def report_payload(report: Any) -> dict[str, Any]:
    return {
        "passed": bool(getattr(report, "passed", False)),
        "checks_run": int(getattr(report, "checks_run", 0)),
        "checks": list(getattr(report, "checks", ()) or ()),
        "failures": [normalize(item) for item in (getattr(report, "failures", ()) or ())],
        "warnings": [normalize(item) for item in (getattr(report, "warnings", ()) or ())],
        "allowances": [normalize(item) for item in (getattr(report, "allowances", ()) or ())],
    }


def worker(case_path: Path, task_path: Path, output: Path, source_root: Path) -> int:
    case = json.loads(case_path.read_text(encoding="utf-8"))
    task = json.loads(task_path.read_text(encoding="utf-8"))
    started = time.monotonic()
    try:
        module = importlib.import_module(f"agent.templates.{case['slug']}")
        base_config = module.config_from_seed(int(case["seed"]))
        edited_config = dataclasses.replace(base_config, **{case["field"]: case["edited_value"]})
        case_source = source_root / case["case_id"]
        base = build_one(module, task, base_config, case_source / "base")
        edited = build_one(module, task, edited_config, case_source / "edited")
        base_parts = {part.name: part for part in base.parts}
        edit_parts = {part.name: part for part in edited.parts}
        names = set(base_parts) | set(edit_parts)
        tokens = tuple(str(token).lower() for token in task["target_tokens"])
        symmetric = set(base_parts) ^ set(edit_parts)
        # Targets can be nested visual/collision elements inside a stable SDK
        # Part (divider inside basket_tub, screen rows inside press_frame, etc.).
        # Apply the already frozen task tokens to the complete source
        # declaration, not only to the top-level part name.
        token_targets = set()
        for name in names:
            declarations = [base_parts.get(name), edit_parts.get(name)]
            searchable = " ".join(
                json.dumps(normalize(item), ensure_ascii=False, sort_keys=True, default=str)
                for item in declarations
                if item is not None
            ).lower()
            if any(token in searchable for token in tokens):
                token_targets.add(name)
        targets = symmetric | token_targets
        common = set(base_parts) & set(edit_parts)
        base_signatures = {
            name: source_signature(part, case_source / "base")
            for name, part in base_parts.items()
        }
        edit_signatures = {
            name: source_signature(part, case_source / "edited")
            for name, part in edit_parts.items()
        }
        # A fused edit can change the host mesh without leaving a role token in
        # the Part declaration (for example, a divider fused into basket_tub).
        # If the predeclared tokens find no target at all, the changed stable
        # source declaration is the only precompile evidence of the target.
        # Promote it rather than misclassifying the host as a dependent.
        if not targets:
            targets |= {
                name
                for name in common
                if base_signatures[name] != edit_signatures[name]
            }
        allowed = {
            name
            for name in common - targets
            if base_signatures[name] != edit_signatures[name]
        }
        non_targets = common - targets - allowed

        base_articulations = {item.name: item for item in base.articulations}
        edit_articulations = {item.name: item for item in edited.articulations}
        articulation_names = set(base_articulations) | set(edit_articulations)
        target_joints: set[str] = set()
        allowed_joints: set[str] = set()
        non_target_joints: set[str] = set()
        for name in articulation_names:
            candidates = [base_articulations.get(name), edit_articulations.get(name)]
            endpoints = {
                endpoint
                for item in candidates
                if item is not None
                for endpoint in (item.parent, item.child)
            }
            if name in (set(base_articulations) ^ set(edit_articulations)) or endpoints & targets or any(
                token in name.lower() for token in tokens
            ):
                target_joints.add(name)
            elif endpoints & allowed:
                allowed_joints.add(name)
            else:
                non_target_joints.add(name)
        payload = {
            "schema_version": 1,
            "case_id": case["case_id"],
            "task_id": case["task_id"],
            "slug": case["slug"],
            "seed": case["seed"],
            "field": case["field"],
            "base_value": case["base_value"],
            "edited_value": case["edited_value"],
            "partition_policy": {
                "target_parts": "predeclared target-token match plus source-graph additions/removals",
                "allowed_dependent_parts": "stable non-target-token parts whose SDK declaration or emitted source mesh changes under the frozen one-field edit",
                "true_non_target_parts": "remaining stable parts",
                "timing": "frozen from SDK source graphs before any artifact compilation",
            },
            "target_parts": sorted(targets),
            "allowed_dependent_parts": sorted(allowed),
            "true_non_target_parts": sorted(non_targets),
            "target_joints": sorted(target_joints),
            "allowed_dependent_joints": sorted(allowed_joints),
            "true_non_target_joints": sorted(non_target_joints),
            "base_part_source_signatures": dict(sorted(base_signatures.items())),
            "edited_part_source_signatures": dict(sorted(edit_signatures.items())),
            "base_articulation_declarations": {
                name: normalize(item) for name, item in sorted(base_articulations.items())
            },
            "edited_articulation_declarations": {
                name: normalize(item) for name, item in sorted(edit_articulations.items())
            },
            "base_part_count": len(base_parts),
            "edited_part_count": len(edit_parts),
            "base_articulation_count": len(base_articulations),
            "edited_articulation_count": len(edit_articulations),
            "baseline_test_manifest": observe_test(module, task, base, base_config),
            "edited_precompile_test_observation": observe_test(
                module, task, edited, edited_config
            ),
            "elapsed_s": time.monotonic() - started,
        }
        dump_json(output, payload)
        return 0
    except BaseException as exc:  # noqa: BLE001
        import traceback

        dump_json(
            output,
            {
                "case_id": case.get("case_id"),
                "error": f"{type(exc).__name__}: {exc}",
                "traceback": traceback.format_exc()[-8000:],
                "elapsed_s": time.monotonic() - started,
            },
        )
        return 1


def run_isolated(case: dict[str, Any], task: dict[str, Any], out: Path, timeout: float) -> dict[str, Any]:
    record = out / "cases" / f"{case['case_id']}.json"
    if record.is_file():
        payload = json.loads(record.read_text(encoding="utf-8"))
        if not payload.get("error"):
            return payload
    inputs = out / "worker_inputs"
    case_path = inputs / f"{case['case_id']}__case.json"
    task_path = inputs / f"{case['case_id']}__task.json"
    dump_json(case_path, case)
    dump_json(task_path, task)
    env = os.environ.copy()
    for key in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
        env[key] = "1"
    try:
        completed = subprocess.run(
            [
                str(PYTHON),
                str(Path(__file__).resolve()),
                "--worker",
                str(case_path),
                str(task_path),
                str(record),
                str(out / "source_graphs"),
            ],
            cwd=TEMPLATE_ROOT,
            env=env,
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
    except subprocess.TimeoutExpired:
        return {"case_id": case["case_id"], "error": f"timeout({timeout}s)"}
    if record.is_file():
        payload = json.loads(record.read_text(encoding="utf-8"))
        if completed.returncode and not payload.get("error"):
            payload["error"] = f"worker_exit_{completed.returncode}"
        return payload
    return {
        "case_id": case["case_id"],
        "error": f"worker_exit_{completed.returncode}",
        "stderr": completed.stderr[-4000:],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--timeout", type=float, default=900.0)
    parser.add_argument("--worker", nargs=4, metavar=("CASE", "TASK", "OUTPUT", "SOURCE_ROOT"))
    args = parser.parse_args()
    if args.worker:
        return worker(*(Path(value) for value in args.worker))
    out = args.out.resolve()
    out.relative_to(EXP_ROOT.resolve())
    cases = load_cases()
    tasks = task_by_id()
    results: list[dict[str, Any]] = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, args.workers)) as pool:
        futures = {
            pool.submit(run_isolated, case, tasks[case["task_id"]], out, args.timeout): case
            for case in cases
        }
        for index, future in enumerate(concurrent.futures.as_completed(futures), 1):
            results.append(future.result())
            if index % 16 == 0 or index == len(futures):
                print(f"frozen {index}/{len(futures)}", flush=True)
    results.sort(key=lambda row: row.get("case_id", ""))
    failures = [row for row in results if row.get("error")]
    manifest = {
        "schema_version": 1,
        "protocol": "t4_distributional_editability_18x16_v1",
        "frozen_at": datetime.now(timezone.utc).isoformat(),
        "frozen_before_artifact_compilation": True,
        "case_count": len(results),
        "successful_freezes": len(results) - len(failures),
        "failed_freezes": len(failures),
        "baseline_precompile_test_pass": sum(
            bool(row.get("baseline_test_manifest", {}).get("passed")) for row in results
        ),
        "edited_precompile_test_pass": sum(
            bool(row.get("edited_precompile_test_observation", {}).get("passed")) for row in results
        ),
        "case_records": [str((out / "cases" / f"{row['case_id']}.json").relative_to(out)) for row in results],
        "failures": failures,
    }
    dump_json(out / "partition_manifest.json", manifest)
    print(json.dumps(manifest, indent=2, ensure_ascii=False))
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
