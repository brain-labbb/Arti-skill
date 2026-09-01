#!/usr/bin/env python3
"""Freeze movable-joint gold first, then freshly compile and score the 33-asset T5 cohort."""

from __future__ import annotations

import argparse
import contextlib
import hashlib
import io
import json
import runpy
import shutil
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
EXP_ROOT = SCRIPT_DIR.parent
PROJECT_ROOT = EXP_ROOT.parent
TEMPLATE_ROOT = PROJECT_ROOT / "arti-template"
DEFAULT_INPUT = EXP_ROOT / "runtime/nano3d_glb_n33/input_packages/input_manifest.json"
DEFAULT_GLB = EXP_ROOT / "runtime/nano3d_glb_n33/output/records.json"
DEFAULT_OUT = EXP_ROOT / "runtime/t5_formal_v1/gold_then_compile"

sys.path.insert(0, str(TEMPLATE_ROOT))
sys.path.insert(0, str(SCRIPT_DIR))

import run_nano3d_editability as compiler  # noqa: E402
import run_t5_source_program_gold as scoring  # noqa: E402


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def dump_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def git_value(*args: str) -> str | None:
    result = subprocess.run(
        ["git", "-C", str(TEMPLATE_ROOT), *args],
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
    )
    return result.stdout.strip() if result.returncode == 0 else None


def stage_package(source: Path, target: Path) -> Path:
    target.mkdir(parents=True, exist_ok=True)
    source_wrapper = source / "model.py"
    target_wrapper = target / "model.py"
    if target_wrapper.exists():
        if sha256(source_wrapper) != sha256(target_wrapper):
            raise RuntimeError(f"resume refused; wrapper differs: {target_wrapper}")
    else:
        shutil.copy2(source_wrapper, target_wrapper)
    source_assets = source / "assets"
    target_assets = target / "assets"
    if source_assets.is_dir() and not target_assets.exists():
        shutil.copytree(source_assets, target_assets)
    return target_wrapper


def freeze_one(row: dict[str, Any], glb_row: dict[str, Any], packages: Path) -> dict[str, Any]:
    asset_id = row["asset_id"]
    wrapper = stage_package(Path(row["copied_package"]), packages / asset_id)
    template = scoring.imported_template_path(wrapper, TEMPLATE_ROOT)
    captured = io.StringIO()
    with contextlib.redirect_stdout(captured), contextlib.redirect_stderr(captured):
        namespace = runpy.run_path(str(wrapper))
    expected = scoring.expected_joints(namespace["object_model"])
    return {
        "asset_id": asset_id,
        "frozen_at": datetime.now(timezone.utc).isoformat(),
        "wrapper": str(wrapper),
        "wrapper_sha256": sha256(wrapper),
        "template_path": str(template.relative_to(PROJECT_ROOT)),
        "template_sha256": sha256(template),
        "bbox_diagonal": scoring.diagonal(glb_row["urdf_scene_bounds"]),
        "expected_joint_count": len(expected),
        "expected_joints": expected,
        "captured_output": captured.getvalue()[-4000:],
    }


def compare(gold: dict[str, Any], actual: list[dict[str, Any]]) -> dict[str, Any]:
    expected = gold["expected_joints"]
    expected_by_name = {joint["name"]: joint for joint in expected}
    actual_by_name = {joint["name"]: joint for joint in actual}
    comparisons: list[dict[str, Any]] = []
    for name in sorted(set(expected_by_name) | set(actual_by_name)):
        wanted = expected_by_name.get(name)
        observed = actual_by_name.get(name)
        if wanted is None or observed is None:
            comparisons.append(
                {"name": name, "expected_present": wanted is not None, "actual_present": observed is not None}
            )
            continue
        comparisons.append(
            {
                "name": name,
                "expected_present": True,
                "actual_present": True,
                "type_correct": observed["type"] == wanted["type"],
                "parent_child_correct": (observed["parent"], observed["child"]) == (wanted["parent"], wanted["child"]),
                "axis_error_degrees": scoring.axis_error_degrees(wanted["axis"], observed["axis"]),
                "origin_error_bbox_normalized": scoring.euclidean(wanted["origin_xyz"], observed["origin_xyz"]) / max(gold["bbox_diagonal"], 1e-12),
                "origin_rpy_error_radians": scoring.euclidean(wanted["origin_rpy"], observed["origin_rpy"]),
                "limit_error_range_normalized": scoring.limit_error(wanted, observed),
                "expected": wanted,
                "actual": observed,
            }
        )
    matched = [item for item in comparisons if item.get("expected_present") and item.get("actual_present")]
    return {
        **gold,
        "rebuild_success": True,
        "actual_joint_count": len(actual),
        "matched_joint_count": len(matched),
        "comparisons": comparisons,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-manifest", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--glb-records", type=Path, default=DEFAULT_GLB)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--max-workers", type=int, default=4)
    parser.add_argument("--timeout", type=float, default=600.0)
    args = parser.parse_args()
    output = args.out.resolve()
    output.relative_to(EXP_ROOT.resolve())
    inputs = json.loads(args.input_manifest.read_text(encoding="utf-8"))
    glb_rows = json.loads(args.glb_records.read_text(encoding="utf-8"))
    glb_by_id = {row["asset_id"]: row for row in glb_rows}
    if set(glb_by_id) != {row["asset_id"] for row in inputs}:
        raise RuntimeError("input and GLB cohorts differ")

    gold_path = output / "frozen_gold.json"
    if gold_path.exists():
        gold_payload = json.loads(gold_path.read_text(encoding="utf-8"))
        gold_records = gold_payload["assets"]
    else:
        gold_records = [
            freeze_one(row, glb_by_id[row["asset_id"]], output / "packages") for row in inputs
        ]
        gold_payload = {
            "schema_version": 1,
            "protocol": "t5_gold_before_fresh_compile_v1",
            "frozen_at": datetime.now(timezone.utc).isoformat(),
            "asset_count": len(gold_records),
            "git_head": git_value("rev-parse", "HEAD"),
            "tracked_worktree_status": git_value("status", "--porcelain", "--untracked-files=no"),
            "gold_definition": "Movable SDK articulation declarations constructed and persisted before any formal URDF compilation.",
            "assets": gold_records,
        }
        dump_json(gold_path, gold_payload)
    frozen_hash = sha256(gold_path)

    gold_by_id = {row["asset_id"]: row for row in gold_records}
    compile_results: dict[str, dict[str, Any]] = {}
    compile_started: dict[str, str] = {}
    with ThreadPoolExecutor(max_workers=args.max_workers) as executor:
        futures = {}
        for row in inputs:
            asset_id = row["asset_id"]
            package = output / "packages" / asset_id
            result_path = package / "formal_compile_result.json"
            if result_path.is_file():
                compile_results[asset_id] = json.loads(result_path.read_text(encoding="utf-8"))
                compile_started[asset_id] = datetime.fromtimestamp(
                    result_path.stat().st_mtime, tz=timezone.utc
                ).isoformat()
                continue
            compile_started[asset_id] = datetime.now(timezone.utc).isoformat()
            future = executor.submit(compiler.run_one, package / "model.py", result_path, args.timeout)
            futures[future] = asset_id
        for future in as_completed(futures):
            asset_id = futures[future]
            compile_results[asset_id] = future.result()
            print(asset_id, compile_results[asset_id].get("verdict"), flush=True)

    records: list[dict[str, Any]] = []
    for row in inputs:
        asset_id = row["asset_id"]
        package = output / "packages" / asset_id
        compile_result = compile_results[asset_id]
        urdf = package / "model.urdf"
        if compile_result.get("verdict") == "pass" and urdf.is_file():
            record = compare(gold_by_id[asset_id], scoring.parse_urdf_joints(urdf))
        else:
            record = {
                **gold_by_id[asset_id],
                "rebuild_success": False,
                "actual_joint_count": 0,
                "matched_joint_count": 0,
                "comparisons": [],
            }
        record["compile_result"] = compile_result
        record["formal_compile_started_at"] = compile_started[asset_id]
        record["formal_urdf_sha256"] = sha256(urdf) if urdf.is_file() else None
        record["frozen_gold_manifest_sha256"] = frozen_hash
        records.append(record)
    dump_json(output / "asset_records.json", records)
    formal_input_manifest = [
        {
            "asset_id": row["asset_id"],
            "source_package": row["source_package"],
            "copied_package": str(output / "packages" / row["asset_id"]),
            "model_urdf_sha256": sha256(output / "packages" / row["asset_id"] / "model.urdf"),
        }
        for row in inputs
    ]
    dump_json(output / "formal_input_manifest.json", formal_input_manifest)
    summary = scoring.summarize(records)
    summary.update(
        {
            "protocol": "t5_gold_before_fresh_compile_v1",
            "frozen_gold_manifest": str(gold_path),
            "frozen_gold_manifest_sha256": frozen_hash,
            "compile_success": sum(row["compile_result"].get("verdict") == "pass" for row in records),
            "chronology_valid": all(
                row["frozen_at"] <= row["formal_compile_started_at"] for row in records
            ),
        }
    )
    dump_json(output / "summary.json", summary)
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0 if summary["compile_success"] == len(records) else 1


if __name__ == "__main__":
    raise SystemExit(main())
