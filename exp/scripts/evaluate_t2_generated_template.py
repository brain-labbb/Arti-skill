#!/usr/bin/env python3
"""Hidden compile/full-QC evaluator for one generated T2 template revision."""

from __future__ import annotations

import argparse
import concurrent.futures
import hashlib
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


def wrapper_text(template: Path, case_kind: str, case_value: int) -> str:
    if case_kind == "seed":
        config = f"CONFIG = _module.config_from_seed({case_value})"
    elif case_kind == "corner":
        config = (
            "_base = _module.config_from_seed(0)\n"
            f"_corner = _module.TEMPLATE_CORNERS[{case_value}]\n"
            "CONFIG = dataclasses.replace(_base, **dict(_corner.overrides))"
        )
    else:
        raise ValueError(case_kind)
    return f'''from __future__ import annotations

import dataclasses
import importlib.util
import sys
from pathlib import Path
from sdk import AssetContext

_template_path = Path(r"{template.resolve()}")
_spec = importlib.util.spec_from_file_location("_formal_generated_template", _template_path)
if _spec is None or _spec.loader is None:
    raise RuntimeError(f"cannot load {{_template_path}}")
_module = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = _module
_spec.loader.exec_module(_module)
_exported = tuple(getattr(_module, "__all__", ()))
_build_names = [name for name in _exported if name.startswith("build_")]
_test_names = [name for name in _exported if name.startswith("run_") and name.endswith("_tests")]
if len(_build_names) != 1 or len(_test_names) != 1:
    raise RuntimeError(f"expected exactly one exported build/test function, got {{_build_names}} / {{_test_names}}")
if not hasattr(_module, "TEMPLATE_DOMAIN") or not hasattr(_module, "config_from_seed"):
    raise RuntimeError("missing TEMPLATE_DOMAIN or config_from_seed")
{config}
ASSETS = AssetContext.from_script(__file__)
object_model = getattr(_module, _build_names[0])(CONFIG, assets=ASSETS)

def run_tests():
    return getattr(_module, _test_names[0])(object_model, CONFIG)
'''


def package_digest(case_dir: Path) -> str:
    digest = hashlib.sha256()
    for path in sorted([case_dir / "model.urdf", *(case_dir / "assets").rglob("*")]):
        if not path.is_file():
            continue
        digest.update(str(path.relative_to(case_dir)).encode("utf-8"))
        digest.update(path.read_bytes())
    return digest.hexdigest()


def worker(template: Path, case_kind: str, case_value: int, case_dir: Path, output: Path) -> int:
    from agent.compiler import compile_urdf_report

    started = time.monotonic()
    payload: dict[str, Any] = {
        "case_kind": case_kind,
        "case_value": case_value,
        "verdict": "fail",
        "error": None,
        "traceback": None,
    }
    try:
        case_dir.mkdir(parents=True, exist_ok=True)
        wrapper = case_dir / "source.py"
        wrapper.write_text(wrapper_text(template, case_kind, case_value), encoding="utf-8")
        report = compile_urdf_report(
            wrapper,
            sdk_package="sdk",
            run_checks=True,
            target="full",
            rewrite_visual_glb=False,
            motion_qc=True,
        )
        urdf = report.urdf_xml or ""
        if not urdf.strip():
            raise RuntimeError("compiler returned empty URDF")
        urdf_path = case_dir / "model.urdf"
        urdf_path.write_text(urdf, encoding="utf-8")
        asset_files = [path for path in (case_dir / "assets").rglob("*") if path.is_file()]
        signal = getattr(report, "signal_bundle", None)
        payload.update(
            {
                "verdict": "pass",
                "artifact_saved": urdf_path.is_file() and bool(asset_files),
                "urdf_sha256": sha256(urdf_path),
                "package_sha256": package_digest(case_dir),
                "asset_file_count": len(asset_files),
                "signal_status": getattr(signal, "status", None),
                "warnings": [str(item) for item in (getattr(report, "warnings", None) or [])],
            }
        )
    except BaseException as exc:  # noqa: BLE001
        import traceback

        payload.update(
            {
                "error": f"{type(exc).__name__}: {exc}",
                "traceback": traceback.format_exc()[-10000:],
                "artifact_saved": bool(list((case_dir / "assets").rglob("*")))
                if (case_dir / "assets").exists()
                else False,
            }
        )
    payload["elapsed_s"] = time.monotonic() - started
    dump_json(output, payload)
    return 0 if payload["verdict"] == "pass" and payload["artifact_saved"] else 1


def isolated(
    template: Path,
    case_kind: str,
    case_value: int,
    root: Path,
    timeout: float,
) -> dict[str, Any]:
    case_id = f"{case_kind}_{case_value:03d}"
    case_dir = root / "cases" / case_id
    output = case_dir / "result.json"
    if output.is_file():
        return json.loads(output.read_text(encoding="utf-8"))
    env = os.environ.copy()
    for key in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
        env[key] = "1"
    try:
        completed = subprocess.run(
            [
                str(PYTHON),
                str(Path(__file__).resolve()),
                "--worker",
                str(template),
                case_kind,
                str(case_value),
                str(case_dir),
                str(output),
            ],
            cwd=TEMPLATE_ROOT,
            env=env,
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
    except subprocess.TimeoutExpired:
        return {
            "case_kind": case_kind,
            "case_value": case_value,
            "verdict": "fail",
            "artifact_saved": False,
            "error": f"timeout({timeout}s)",
            "elapsed_s": timeout,
        }
    if output.is_file():
        return json.loads(output.read_text(encoding="utf-8"))
    return {
        "case_kind": case_kind,
        "case_value": case_value,
        "verdict": "fail",
        "artifact_saved": False,
        "error": f"worker_exit_{completed.returncode}: {completed.stderr[-4000:]}",
    }


def discover_corners(template: Path, output: Path, timeout: float) -> dict[str, Any]:
    probe = output / "corner_probe.json"
    code = f'''
import importlib.util, json, sys
from pathlib import Path
p=Path(r"{template.resolve()}")
s=importlib.util.spec_from_file_location("_corner_probe_template",p)
m=importlib.util.module_from_spec(s); sys.modules[s.name]=m; s.loader.exec_module(m)
corners=getattr(m,"TEMPLATE_CORNERS",())
print(json.dumps({{"count":len(corners),"names":[str(getattr(x,"name",i)) for i,x in enumerate(corners)]}}))
'''
    try:
        completed = subprocess.run(
            [str(PYTHON), "-c", code],
            cwd=TEMPLATE_ROOT,
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
        payload = json.loads(completed.stdout.strip().splitlines()[-1]) if completed.returncode == 0 else {
            "count": 0,
            "names": [],
            "error": completed.stderr[-4000:],
        }
    except Exception as exc:  # noqa: BLE001
        payload = {"count": 0, "names": [], "error": f"{type(exc).__name__}: {exc}"}
    dump_json(probe, payload)
    return payload


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--template", type=Path)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--seeds", default="0-15")
    parser.add_argument("--include-corners", action="store_true")
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--timeout", type=float, default=600.0)
    parser.add_argument(
        "--worker", nargs=5, metavar=("TEMPLATE", "KIND", "VALUE", "CASE_DIR", "OUTPUT")
    )
    args = parser.parse_args()
    if args.worker:
        return worker(
            Path(args.worker[0]),
            args.worker[1],
            int(args.worker[2]),
            Path(args.worker[3]),
            Path(args.worker[4]),
        )
    if args.template is None or args.out is None:
        parser.error("--template and --out are required")
    template = args.template.resolve()
    out = args.out.resolve()
    out.relative_to(EXP_ROOT.resolve())
    start, end = (int(item) for item in args.seeds.split("-", 1))
    cases: list[tuple[str, int]] = [("seed", seed) for seed in range(start, end + 1)]
    corner_probe = {"count": 0, "names": []}
    if args.include_corners:
        corner_probe = discover_corners(template, out, args.timeout)
        cases.extend(("corner", index) for index in range(int(corner_probe.get("count", 0))))
    records: list[dict[str, Any]] = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, args.workers)) as pool:
        futures = {
            pool.submit(isolated, template, kind, value, out, args.timeout): (kind, value)
            for kind, value in cases
        }
        for future in concurrent.futures.as_completed(futures):
            records.append(future.result())
    records.sort(key=lambda row: (row["case_kind"], row["case_value"]))
    seed_records = [row for row in records if row["case_kind"] == "seed"]
    corner_records = [row for row in records if row["case_kind"] == "corner"]
    passed = lambda row: row.get("verdict") == "pass" and row.get("artifact_saved") is True
    summary = {
        "schema_version": 1,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "template": str(template),
        "template_sha256": sha256(template),
        "seed_range": [start, end],
        "seed_compile_and_full_qc_pass": sum(passed(row) for row in seed_records),
        "seed_total": len(seed_records),
        "all_seeds_pass": bool(seed_records) and all(passed(row) for row in seed_records),
        "artifact_saved_any": any(row.get("artifact_saved") for row in records),
        "corner_count": len(corner_records),
        "corner_pass": sum(passed(row) for row in corner_records),
        "all_corners_pass": (
            (not args.include_corners)
            or (bool(corner_records) and all(passed(row) for row in corner_records))
        ),
        "corner_probe": corner_probe,
        "elapsed_s_total": sum(float(row.get("elapsed_s") or 0.0) for row in records),
        "records": records,
    }
    dump_json(out / "summary.json", summary)
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0 if summary["all_seeds_pass"] and summary["all_corners_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
