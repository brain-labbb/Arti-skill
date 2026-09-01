#!/usr/bin/env python3
"""Freeze and execute one out-of-domain negative configuration per T3 template."""

from __future__ import annotations

import argparse
import concurrent.futures
import hashlib
import importlib
import json
import os
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


EXP_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = EXP_ROOT.parent
TEMPLATE_ROOT = PROJECT_ROOT / "arti-template"
PYTHON = TEMPLATE_ROOT / ".venv/bin/python"
GOLD = EXP_ROOT / "reference/naming_gold_v2.json"
DEFAULT_OUT = EXP_ROOT / "runtime/t3_formal_v1/invalid_combination_rejection_v2"

sys.path.insert(0, str(TEMPLATE_ROOT))


def dump_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True, default=str) + "\n",
        encoding="utf-8",
    )


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def invalid_value(values: tuple[Any, ...]) -> Any:
    if values and all(isinstance(value, str) for value in values):
        return "__formal_out_of_domain_candidate__"
    if values and all(isinstance(value, int) and not isinstance(value, bool) for value in values):
        candidate = min(values) - 1
        return candidate if candidate not in values else max(values) + 1
    if values and all(isinstance(value, float) for value in values):
        return max(values) + max(1.0, abs(max(values)))
    raise ValueError(f"cannot create type-compatible invalid value for {values!r}")


def freeze(out: Path) -> list[dict[str, Any]]:
    gold = json.loads(GOLD.read_text(encoding="utf-8"))
    cases: list[dict[str, Any]] = []
    inapplicable: list[dict[str, str]] = []
    for slug in sorted(gold["assets"]):
        module = importlib.import_module(f"agent.templates.{slug}")
        if not hasattr(module, "TEMPLATE_DOMAIN"):
            inapplicable.append(
                {"slug": slug, "reason": "legacy template exposes no finite TEMPLATE_DOMAIN"}
            )
            continue
        chosen = None
        for slot in module.TEMPLATE_DOMAIN.slots:
            try:
                invalid = invalid_value(tuple(slot.values))
            except ValueError:
                continue
            chosen = (slot, invalid)
            break
        if chosen is None:
            raise RuntimeError(f"{slug}: no negative-capable domain slot")
        slot, invalid = chosen
        template = TEMPLATE_ROOT / "agent/templates" / f"{slug}.py"
        cases.append(
            {
                "case_id": f"{slug}__invalid_{slot.name}",
                "slug": slug,
                "seed": 0,
                "field": slot.name,
                "valid_values": list(slot.values),
                "invalid_value": invalid,
                "template": str(template),
                "template_sha256": sha256(template),
            }
        )
    manifest = {
        "schema_version": 1,
        "protocol": "t3_invalid_combination_rejection_v1",
        "frozen_at": datetime.now(timezone.utc).isoformat(),
        "case_count": len(cases),
        "inapplicable_count": len(inapplicable),
        "inapplicable": inapplicable,
        "selection_policy": "first TEMPLATE_DOMAIN slot with a type-compatible value provably absent from its frozen value set",
        "cases": cases,
    }
    dump_json(out / "frozen_negative_manifest.json", manifest)
    return cases


def worker(case_path: Path, output: Path) -> int:
    import dataclasses
    import importlib.util

    from sdk import AssetContext

    case = json.loads(case_path.read_text(encoding="utf-8"))
    started = time.monotonic()
    rejected = False
    error = None
    stage = "load"
    try:
        path = Path(case["template"])
        spec = importlib.util.spec_from_file_location("_t3_negative_template", path)
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        stage = "construct_invalid_config"
        base = module.config_from_seed(int(case["seed"]))
        config = dataclasses.replace(base, **{case["field"]: case["invalid_value"]})
        stage = "resolve_config"
        resolved = module.resolve_config(config) if hasattr(module, "resolve_config") else config
        stage = "build"
        build_names = [
            name
            for name in module.__all__
            if name.startswith("build_") and "seeded" not in name
        ]
        if len(build_names) != 1:
            raise RuntimeError(f"contract has {len(build_names)} build functions")
        root = output.parent
        root.mkdir(parents=True, exist_ok=True)
        getattr(module, build_names[0])(
            resolved, assets=AssetContext.from_script(root / "source.py")
        )
    except (ValueError, TypeError, AssertionError) as exc:
        rejected = True
        error = f"{type(exc).__name__}: {exc}"
    except Exception as exc:  # noqa: BLE001
        # A validation/domain exception wrapped by the SDK still counts only
        # when it occurs before or during build and explicitly rejects the
        # invalid value. Preserve the exact class/message for audit.
        error = f"{type(exc).__name__}: {exc}"
        lowered = error.lower()
        rejected = stage in {"resolve_config", "build"} and any(
            token in lowered
            for token in ("invalid", "unsupported", "unknown", "domain", "candidate", "must be")
        )
    payload = {
        **case,
        "rejected": rejected,
        "stage": stage,
        "exception": error,
        "elapsed_s": time.monotonic() - started,
    }
    dump_json(output, payload)
    return 0 if rejected else 1


def run_one(case: dict[str, Any], out: Path, timeout: float) -> dict[str, Any]:
    input_path = out / "worker_inputs" / f"{case['case_id']}.json"
    output = out / "cases" / f"{case['case_id']}.json"
    dump_json(input_path, case)
    env = os.environ.copy()
    for key in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
        env[key] = "1"
    try:
        subprocess.run(
            [str(PYTHON), str(Path(__file__).resolve()), "--worker", str(input_path), str(output)],
            cwd=TEMPLATE_ROOT,
            env=env,
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
    except subprocess.TimeoutExpired:
        return {**case, "rejected": False, "stage": "timeout", "exception": f"timeout({timeout}s)"}
    return json.loads(output.read_text(encoding="utf-8")) if output.is_file() else {
        **case,
        "rejected": False,
        "stage": "worker_crash",
        "exception": "worker produced no record",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--timeout", type=float, default=300.0)
    parser.add_argument("--worker", nargs=2, metavar=("CASE", "OUTPUT"))
    args = parser.parse_args()
    if args.worker:
        return worker(Path(args.worker[0]), Path(args.worker[1]))
    out = args.out.resolve()
    out.relative_to(EXP_ROOT.resolve())
    cases = freeze(out)
    frozen_finished = datetime.now(timezone.utc).isoformat()
    records: list[dict[str, Any]] = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, args.workers)) as pool:
        futures = [pool.submit(run_one, case, out, args.timeout) for case in cases]
        for future in concurrent.futures.as_completed(futures):
            records.append(future.result())
    records.sort(key=lambda row: row["case_id"])
    summary = {
        "schema_version": 1,
        "protocol": "t3_invalid_combination_rejection_v1",
        "frozen_manifest_completed_at": frozen_finished,
        "execution_completed_at": datetime.now(timezone.utc).isoformat(),
        "chronology_valid": True,
        "rejected": sum(row["rejected"] for row in records),
        "total": len(records),
        "rate": sum(row["rejected"] for row in records) / len(records),
        "records": records,
    }
    dump_json(out / "summary.json", summary)
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
