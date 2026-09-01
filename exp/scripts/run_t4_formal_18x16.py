#!/usr/bin/env python3
"""Execute and score the frozen formal T4 18-task x 16-seed protocol."""

from __future__ import annotations

import argparse
import concurrent.futures
import hashlib
import json
import math
import re
import sys
import time
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


EXP_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = EXP_ROOT.parent
TEMPLATE_ROOT = PROJECT_ROOT / "arti-template"
PROTOCOL_ROOT = EXP_ROOT / "runtime/t4_distributional_protocol_v1"
PARTITION_ROOT = EXP_ROOT / "runtime/t4_formal_v1/frozen_partitions_v8"
DEFAULT_OUT = EXP_ROOT / "runtime/t4_formal_v1/execution_v3"

sys.path.insert(0, str(TEMPLATE_ROOT))
sys.path.insert(0, str(EXP_ROOT / "scripts"))

import run_nano3d_editability as local  # noqa: E402


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


def load_inputs() -> tuple[list[dict[str, Any]], dict[str, dict[str, Any]]]:
    cases = [
        json.loads(line)
        for line in (PROTOCOL_ROOT / "cases.jsonl").read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    manifest = json.loads((PROTOCOL_ROOT / "protocol_manifest.json").read_text(encoding="utf-8"))
    partitions = json.loads((PARTITION_ROOT / "partition_manifest.json").read_text(encoding="utf-8"))
    if partitions.get("successful_freezes") != 288 or not partitions.get("frozen_before_artifact_compilation"):
        raise RuntimeError("formal source partitions are not complete/frozen")
    problems: list[str] = []
    for case in cases:
        path = PARTITION_ROOT / "cases" / f"{case['case_id']}.json"
        row = json.loads(path.read_text(encoding="utf-8"))
        targets = set(row["target_parts"])
        allowed = set(row["allowed_dependent_parts"])
        non_targets = set(row["true_non_target_parts"])
        base = row["base_part_source_signatures"]
        edited = row["edited_part_source_signatures"]
        names = set(base) | set(edited)
        changed = {name for name in names if base.get(name) != edited.get(name)}
        if not targets or not (targets & changed):
            problems.append(f"{case['case_id']}: no changed target")
        if targets & allowed or targets & non_targets or allowed & non_targets:
            problems.append(f"{case['case_id']}: partition overlap")
        if targets | allowed | non_targets != names:
            problems.append(f"{case['case_id']}: partition does not cover source parts")
        if any(base.get(name) != edited.get(name) for name in non_targets):
            problems.append(f"{case['case_id']}: changed true non-target")
        if any(base.get(name) == edited.get(name) for name in allowed):
            problems.append(f"{case['case_id']}: unchanged allowed dependent")
    if problems:
        raise RuntimeError("invalid formal source partitions: " + "; ".join(problems[:20]))
    return cases, {row["task_id"]: row for row in manifest["tasks"]}


def write_wrapper(path: Path, task: dict[str, Any], case: dict[str, Any], edited: bool) -> None:
    changes = {case["field"]: case["edited_value"]} if edited else {}
    path.parent.mkdir(parents=True, exist_ok=True)
    expected = local.wrapper_text(
        task["slug"], task["stem"], int(case["seed"]), case["edit_class"], changes
    )
    if path.is_file() and path.read_text(encoding="utf-8") != expected:
        raise RuntimeError(f"frozen wrapper differs: {path}")
    if not path.exists():
        path.write_text(expected, encoding="utf-8")


def compile_one(wrapper: Path, result: Path, timeout: float) -> dict[str, Any]:
    if result.is_file():
        return json.loads(result.read_text(encoding="utf-8"))
    return local.run_one(wrapper, result, timeout)


def run_phase(jobs: list[tuple[Path, Path]], workers: int, timeout: float, label: str) -> dict[str, dict[str, Any]]:
    results: dict[str, dict[str, Any]] = {}
    with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, workers)) as pool:
        futures = {
            pool.submit(compile_one, wrapper, result, timeout): (wrapper, result)
            for wrapper, result in jobs
        }
        for index, future in enumerate(concurrent.futures.as_completed(futures), 1):
            wrapper, _ = futures[future]
            results[str(wrapper)] = future.result()
            if index % 16 == 0 or index == len(futures):
                print(f"{label} {index}/{len(futures)}", flush=True)
    return results


def canonical(node: ET.Element) -> str:
    clone = ET.fromstring(ET.tostring(node, encoding="unicode"))
    return ET.tostring(clone, encoding="unicode", short_empty_elements=True)


def referenced_hashes(node: ET.Element, package: Path) -> dict[str, str | None]:
    hashes: dict[str, str | None] = {}
    for element in node.iter():
        for field in ("filename", "texture"):
            ref = element.attrib.get(field)
            if not ref:
                continue
            path = Path(ref)
            if not path.is_absolute():
                path = package / path
            hashes[ref] = sha256(path) if path.is_file() else None
    return hashes


def artifact_graph(urdf: Path) -> dict[str, Any]:
    root = ET.parse(urdf).getroot()
    package = urdf.parent
    links: dict[str, str] = {}
    nonempty_geometry: dict[str, bool] = {}
    for link in root.findall("link"):
        name = link.attrib.get("name", "")
        encoded = json.dumps(
            {"xml": canonical(link), "referenced_hashes": referenced_hashes(link, package)},
            ensure_ascii=False,
            sort_keys=True,
        ).encode("utf-8")
        links[name] = hashlib.sha256(encoded).hexdigest()
        nonempty_geometry[name] = any(
            geometry.find("mesh") is not None
            or geometry.find("box") is not None
            or geometry.find("cylinder") is not None
            or geometry.find("sphere") is not None
            for parent_tag in ("visual", "collision")
            for parent in link.findall(parent_tag)
            for geometry in [parent.find("geometry")]
            if geometry is not None
        )
    joints: dict[str, dict[str, Any]] = {}
    for joint in root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        joints[joint.attrib.get("name", "")] = {
            "type": joint.attrib.get("type", ""),
            "parent": parent.attrib.get("link", "") if parent is not None else "",
            "child": child.attrib.get("link", "") if child is not None else "",
            "signature": hashlib.sha256(canonical(joint).encode("utf-8")).hexdigest(),
        }
    return {"links": links, "joints": joints, "nonempty_geometry": nonempty_geometry}


def source_joint_triplet(declaration: dict[str, Any]) -> tuple[str, str, str]:
    joint_type = declaration.get("articulation_type", "")
    if isinstance(joint_type, dict):
        joint_type = joint_type.get("value", "")
    return str(joint_type), str(declaration.get("parent", "")), str(declaration.get("child", ""))


def score_case(
    case: dict[str, Any],
    task: dict[str, Any],
    partition: dict[str, Any],
    base_run: dict[str, Any],
    edit_run: dict[str, Any],
    template_hash_after: str,
) -> dict[str, Any]:
    base_urdf = Path(base_run.get("urdf_path") or "")
    edit_urdf = Path(edit_run.get("urdf_path") or "")
    base_graph = artifact_graph(base_urdf) if base_urdf.is_file() else {"links": {}, "joints": {}, "nonempty_geometry": {}}
    edit_graph = artifact_graph(edit_urdf) if edit_urdf.is_file() else {"links": {}, "joints": {}, "nonempty_geometry": {}}
    expected_base_links = set(partition["base_part_source_signatures"])
    expected_edit_links = set(partition["edited_part_source_signatures"])
    expected_base_joints = set(partition["base_articulation_declarations"])
    expected_edit_joints = set(partition["edited_articulation_declarations"])
    changed_source_targets = {
        name
        for name in set(partition["target_parts"])
        if partition["base_part_source_signatures"].get(name)
        != partition["edited_part_source_signatures"].get(name)
    }
    changed_artifact_links = {
        name
        for name in set(base_graph["links"]) | set(edit_graph["links"])
        if base_graph["links"].get(name) != edit_graph["links"].get(name)
    }
    target_reflected = bool(changed_source_targets) and changed_source_targets <= changed_artifact_links
    source_graph_complete = (
        set(base_graph["links"]) == expected_base_links
        and set(edit_graph["links"]) == expected_edit_links
        and set(base_graph["joints"]) == expected_base_joints
        and set(edit_graph["joints"]) == expected_edit_joints
    )
    target_fulfilled = bool(
        base_run.get("verdict") == "pass"
        and edit_run.get("verdict") == "pass"
        and source_graph_complete
        and target_reflected
    )
    edited_target_joints = set(partition["target_joints"]) & expected_edit_joints
    anchor_checks = []
    for name in edited_target_joints:
        declaration = partition["edited_articulation_declarations"][name]
        expected = source_joint_triplet(declaration)
        actual_row = edit_graph["joints"].get(name)
        actual = (
            str(actual_row.get("type", "")),
            str(actual_row.get("parent", "")),
            str(actual_row.get("child", "")),
        ) if actual_row else None
        anchor_checks.append(actual == expected)
    added_target_links = (expected_edit_links - expected_base_links) & set(partition["target_parts"])
    anchor_pass = bool(
        target_fulfilled
        and all(anchor_checks)
        and all(
            any(row["child"] == name for row in edit_graph["joints"].values())
            for name in added_target_links
        )
    )
    edited_changed_targets = changed_source_targets & expected_edit_links
    scale_pass = bool(
        target_fulfilled
        and edited_changed_targets
        and all(edit_graph["nonempty_geometry"].get(name, False) for name in edited_changed_targets)
    )
    true_non_targets = set(partition["true_non_target_parts"])
    non_target_preserved = all(
        name in base_graph["links"]
        and name in edit_graph["links"]
        and base_graph["links"][name] == edit_graph["links"][name]
        for name in true_non_targets
    )
    allowed_parts = set(partition["target_parts"]) | set(partition["allowed_dependent_parts"])
    geometry_locality = changed_artifact_links <= allowed_parts
    true_non_target_joints = set(partition["true_non_target_joints"])
    structural_locality = all(
        name in base_graph["joints"]
        and name in edit_graph["joints"]
        and base_graph["joints"][name]["signature"] == edit_graph["joints"][name]["signature"]
        for name in true_non_target_joints
    )
    post_edit_constraint_pass = edit_run.get("verdict") == "pass"
    regression_preservation = bool(
        base_run.get("verdict") == "pass"
        and set(base_graph["links"]) == expected_base_links
        and set(base_graph["joints"]) == expected_base_joints
        and template_hash_after == task["template_sha256"]
    )
    final_pass = all(
        (
            target_fulfilled,
            anchor_pass,
            scale_pass,
            non_target_preserved,
            geometry_locality,
            structural_locality,
            post_edit_constraint_pass,
            regression_preservation,
        )
    )
    return {
        "case_id": case["case_id"],
        "task_id": case["task_id"],
        "slug": case["slug"],
        "seed": case["seed"],
        "edit_class": case["edit_class"],
        "field": case["field"],
        "base_value": case["base_value"],
        "edited_value": case["edited_value"],
        "partition_sha256": sha256(PARTITION_ROOT / "cases" / f"{case['case_id']}.json"),
        "changed_source_target_parts": sorted(changed_source_targets),
        "changed_artifact_links": sorted(changed_artifact_links),
        "unexpected_changed_links": sorted(changed_artifact_links - allowed_parts),
        "gates": {
            "target_fulfilled": target_fulfilled,
            "anchor": anchor_pass,
            "scale": scale_pass,
            "non_target_preserved": non_target_preserved,
            "geometry_locality": geometry_locality,
            "structural_locality": structural_locality,
            "post_edit_constraint_pass": post_edit_constraint_pass,
            "regression_preservation": regression_preservation,
            "final_pass": final_pass,
        },
        "diagnostics": {
            "source_graph_complete": source_graph_complete,
            "target_reflected_in_artifact": target_reflected,
            "target_joint_anchor_checks": anchor_checks,
            "base_compile_verdict": base_run.get("verdict"),
            "edit_compile_verdict": edit_run.get("verdict"),
            "base_error": base_run.get("error"),
            "edit_error": edit_run.get("error"),
        },
        "cost": {
            "baseline_wall_time_s": base_run.get("elapsed_s"),
            "edit_wall_time_s": edit_run.get("elapsed_s"),
            "api_tokens": 0,
            "api_cost_usd": 0.0,
            "edit_operation": "one frozen dataclass field replacement",
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--timeout", type=float, default=600.0)
    args = parser.parse_args()
    out = args.out.resolve()
    out.relative_to(EXP_ROOT.resolve())
    cases, tasks = load_inputs()
    base_jobs: list[tuple[Path, Path]] = []
    edit_jobs: list[tuple[Path, Path]] = []
    wrappers: dict[str, tuple[Path, Path]] = {}
    for case in cases:
        task = tasks[case["task_id"]]
        case_root = out / "cases" / case["case_id"]
        base_wrapper = case_root / "base/source.py"
        edit_wrapper = case_root / "edited/source.py"
        write_wrapper(base_wrapper, task, case, edited=False)
        write_wrapper(edit_wrapper, task, case, edited=True)
        base_jobs.append((base_wrapper, base_wrapper.parent / "compile_result.json"))
        edit_jobs.append((edit_wrapper, edit_wrapper.parent / "compile_result.json"))
        wrappers[case["case_id"]] = (base_wrapper, edit_wrapper)
    started = datetime.now(timezone.utc).isoformat()
    # Deliberately phase-separated: every historical baseline is compiled and
    # recorded before any edited artifact is compiled.
    base_results = run_phase(base_jobs, args.workers, args.timeout, "baseline")
    baseline_phase_finished = datetime.now(timezone.utc).isoformat()
    edit_results = run_phase(edit_jobs, args.workers, args.timeout, "edited")
    edited_phase_finished = datetime.now(timezone.utc).isoformat()
    template_hashes_after = {
        task_id: sha256(PROJECT_ROOT / task["template_path"])
        for task_id, task in tasks.items()
    }
    records: list[dict[str, Any]] = []
    for case in cases:
        base_wrapper, edit_wrapper = wrappers[case["case_id"]]
        partition_path = PARTITION_ROOT / "cases" / f"{case['case_id']}.json"
        partition = json.loads(partition_path.read_text(encoding="utf-8"))
        record = score_case(
            case,
            tasks[case["task_id"]],
            partition,
            base_results[str(base_wrapper)],
            edit_results[str(edit_wrapper)],
            template_hashes_after[case["task_id"]],
        )
        dump_json(out / "records" / f"{case['case_id']}.json", record)
        records.append(record)
    records.sort(key=lambda row: row["case_id"])
    gates = tuple(next(iter(records))["gates"])
    task_rows = []
    for task_id in sorted(tasks):
        subset = [row for row in records if row["task_id"] == task_id]
        task_rows.append(
            {
                "task_id": task_id,
                "slug": tasks[task_id]["slug"],
                "case_count": len(subset),
                "final_pass_count": sum(row["gates"]["final_pass"] for row in subset),
                "propagation_16_of_16": all(row["gates"]["final_pass"] for row in subset),
            }
        )
    edit_times = [
        row["cost"]["edit_wall_time_s"]
        for row in records
        if isinstance(row["cost"]["edit_wall_time_s"], (int, float))
    ]
    summary = {
        "schema_version": 1,
        "protocol": "t4_distributional_editability_18x16_v1",
        "started_at": started,
        "baseline_phase_finished_at": baseline_phase_finished,
        "edited_phase_finished_at": edited_phase_finished,
        "phase_order_valid": started <= baseline_phase_finished <= edited_phase_finished,
        "task_count": len(tasks),
        "seeds_per_task": 16,
        "case_count": len(records),
        "gates": {
            gate: {
                "pass": sum(row["gates"][gate] for row in records),
                "total": len(records),
                "rate": sum(row["gates"][gate] for row in records) / len(records),
            }
            for gate in gates
        },
        "propagation_16_of_16": {
            "pass": sum(row["propagation_16_of_16"] for row in task_rows),
            "total": len(task_rows),
            "rate": sum(row["propagation_16_of_16"] for row in task_rows) / len(task_rows),
        },
        "task_results": task_rows,
        "edit_cost": {
            "mean_wall_time_s": sum(edit_times) / len(edit_times) if edit_times else None,
            "median_wall_time_s": sorted(edit_times)[len(edit_times) // 2] if edit_times else None,
            "api_tokens": 0,
            "api_cost_usd": 0.0,
        },
        "frozen_partition_manifest": str(PARTITION_ROOT / "partition_manifest.json"),
        "frozen_partition_manifest_sha256": sha256(PARTITION_ROOT / "partition_manifest.json"),
        "metric_scope_note": (
            "Anchor/type/parent-child and locality are scored against SDK declarations and "
            "in-memory geometry hashes frozen before artifact compilation; scale requires a "
            "changed target declaration to survive as nonempty target artifact geometry."
        ),
    }
    dump_json(out / "records.json", records)
    dump_json(out / "summary.json", summary)
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
