#!/usr/bin/env python3
"""Artifact-only PV-A V2 rescore for the S05 local-visual-AABB erratum."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import run_pva_editability_v2 as base


EXP_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RUN = EXP_ROOT / "runtime/nano3d_editability_v2/pva_retry1_d8d6e47a"
DEFAULT_BASE_MAPPING = EXP_ROOT / "reference/editability_v2/pva_task_mapping.json"
DEFAULT_MAPPING = EXP_ROOT / "reference/editability_v2/pva_task_mapping_erratum_v2.json"
DEFAULT_SCORER = EXP_ROOT / "reference/editability_v2/pva_scorer_erratum_v2.json"
DEFAULT_OUT = DEFAULT_RUN / "contract_corrected_erratum_v2"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def object_sha256(value: object) -> str:
    payload = json.dumps(value, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(payload).hexdigest()


def load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def task_by_id(mapping: dict) -> dict[str, dict]:
    return {task["task_id"]: task for task in mapping["tasks"]}


def validate_erratum(base_mapping: dict, mapping: dict, scorer: dict) -> None:
    if sha256(DEFAULT_BASE_MAPPING) != scorer["base_mapping_sha256"]:
        raise RuntimeError("base mapping SHA mismatch")
    if mapping["source_manifest_sha256"] != scorer["source_manifest_sha256"]:
        raise RuntimeError("manifest binding mismatch")
    if mapping["source_scorer_sha256"] != scorer["source_shared_scorer_sha256"]:
        raise RuntimeError("shared scorer binding mismatch")
    original = task_by_id(base_mapping)
    corrected = task_by_id(mapping)
    if set(original) != set(corrected):
        raise RuntimeError("erratum changed the task cohort")
    for task_id in original:
        if task_id != "S05" and original[task_id] != corrected[task_id]:
            raise RuntimeError(f"non-S05 mapping changed: {task_id}")
    lhs = copy.deepcopy(original["S05"])
    rhs = copy.deepcopy(corrected["S05"])
    for row in (lhs, rhs):
        row.pop("numeric_predicates")
        row.pop("compiled_predicates")
    if lhs != rhs:
        raise RuntimeError("S05 changed outside numeric/compiled scoring selectors")
    rules = corrected["S05"]["compiled_predicates"]
    if len(rules) != 10 or any(r.get("coordinate_space") != "visual_geometry_local" for r in rules):
        raise RuntimeError("S05 erratum must contain ten visual-local axis predicates")


def raw_visual_bounds(path: Path, mapping: dict) -> dict:
    root = ET.parse(path).getroot()
    replacements = dict(mapping["locality"].get("canonical_replacements") or {})
    result = {}
    for link in root.findall("link"):
        link_name = base.replace_names(link.attrib.get("name", ""), replacements)
        visuals = {}
        for index, visual in enumerate(link.findall("visual")):
            geometry = visual.find("geometry")
            if geometry is None:
                continue
            low, high = base._geometry_bounds(geometry, path.parent)
            name = base.replace_names(visual.attrib.get("name", f"visual_{index}"), replacements)
            visuals[name] = {
                "min": list(low),
                "max": list(high),
                "extent": [high[axis] - low[axis] for axis in range(3)],
            }
        result[link_name] = visuals
    return result


def select(bounds: dict, rule: dict) -> tuple[float, list[str]]:
    links = tuple(token.lower() for token in rule["link_tokens"])
    visuals = tuple(token.lower() for token in rule["visual_tokens"])
    chosen = []
    for link_name, rows in bounds.items():
        if not all(token in link_name.lower() for token in links):
            continue
        for visual_name, row in rows.items():
            if all(token in visual_name.lower() for token in visuals):
                chosen.append((f"{link_name}/{visual_name}", row))
    if len(chosen) != 1:
        raise RuntimeError(f"selector must resolve exactly one local visual: {rule}, got {chosen}")
    axis = {"x": 0, "y": 1, "z": 2}[rule["axis"]]
    return chosen[0][1][rule["measure"]][axis], [chosen[0][0]]


def local_predicates(parent_urdf: Path, edited_urdf: Path, mapping: dict) -> list[dict]:
    parent = raw_visual_bounds(parent_urdf, mapping)
    edited = raw_visual_bounds(edited_urdf, mapping)
    rows = []
    for rule in mapping["compiled_predicates"]:
        parent_value, parent_matches = select(parent, rule)
        edited_value, edited_matches = select(edited, rule)
        expected = parent_value * float(rule["factor"])
        tolerance = float(rule.get("tolerance", 1.0e-6))
        rows.append({
            **rule,
            "parent_actual": parent_value,
            "edited_actual": edited_value,
            "comparison_actual": edited_value,
            "expected_boundary": expected,
            "parent_matches": parent_matches,
            "edited_matches": edited_matches,
            "passed": base._op_pass(edited_value, rule["op"], expected, tolerance),
        })
    return rows


def corrected_numeric(preflight: dict, mapping: dict) -> dict:
    result = copy.deepcopy(preflight)
    old = preflight["numeric_predicates"]
    rows = []
    for rule in mapping["numeric_predicates"]:
        match = next(row for row in old if all(row[key] == rule[key] for key in ("edited_field", "parent_field", "op")))
        parent_value = float(match["parent_actual"])
        edited_value = float(match["edited_actual"])
        expected = parent_value * float(rule["factor"])
        rows.append({
            **rule,
            "parent_actual": parent_value,
            "edited_actual": edited_value,
            "expected_boundary": expected,
            "passed": base._op_pass(edited_value, rule["op"], expected, 1.0e-9),
        })
    result["numeric_predicates"] = rows
    result["numeric_pass"] = all(row["passed"] for row in rows)
    result["preflight_pass"] = bool(
        result["field_locality"]
        and result["numeric_pass"]
        and result["source_rules_pass"]
        and result["source_locality_pass"]
    )
    return result


def wilson(passed: int, total: int) -> list[float]:
    z = 1.959963984540054
    p = passed / total
    denominator = 1 + z * z / total
    center = (p + z * z / (2 * total)) / denominator
    half = z * math.sqrt((p * (1 - p) + z * z / (4 * total)) / total) / denominator
    return [center - half, center + half]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-root", type=Path, default=DEFAULT_RUN)
    parser.add_argument("--base-mapping", type=Path, default=DEFAULT_BASE_MAPPING)
    parser.add_argument("--mapping", type=Path, default=DEFAULT_MAPPING)
    parser.add_argument("--scorer", type=Path, default=DEFAULT_SCORER)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    args = parser.parse_args()
    run_root = base.contained(args.run_root)
    base_mapping_path = base.contained(args.base_mapping)
    mapping_path = base.contained(args.mapping)
    scorer_path = base.contained(args.scorer)
    output = base.contained(args.out, must_exist=False)
    output.relative_to(EXP_ROOT.resolve(strict=True))
    output.mkdir(parents=True, exist_ok=True)

    base_mapping = load(base_mapping_path)
    mapping = load(mapping_path)
    scorer = load(scorer_path)
    validate_erratum(base_mapping, mapping, scorer)
    original_summary = load(run_root / "summary.json")
    if original_summary["source_manifest_sha256"] != mapping["source_manifest_sha256"]:
        raise RuntimeError("run artifacts do not match the frozen manifest")
    mappings = task_by_id(mapping)
    mapping_sha = sha256(mapping_path)
    scorer_sha = sha256(scorer_path)
    runner_sha = sha256(Path(__file__).resolve())
    scores = []
    changed = []

    for task_id in sorted(mappings):
        task = mappings[task_id]
        for seed in mapping["seeds"]:
            record_path = run_root / "records" / task_id / f"seed_{seed:02d}.json"
            original = load(record_path)
            preflight = original["preflight"]
            scorer_task = task
            if task_id == "S05":
                preflight = corrected_numeric(preflight, task)
                scorer_task = copy.deepcopy(task)
                scorer_task["compiled_predicates"] = []
            rescored = base.score_compiled_case(
                preflight,
                scorer_task,
                original["parent_compile"],
                original["edited_compile"],
                "frozen-template-hash",
                "frozen-template-hash",
            )
            if task_id == "S05":
                predicates = local_predicates(
                    Path(original["parent_compile"]["urdf_path"]),
                    Path(original["edited_compile"]["urdf_path"]),
                    task,
                )
                compile_pass = original["parent_compile"]["verdict"] == "pass" and original["edited_compile"]["verdict"] == "pass"
                target = bool(
                    compile_pass
                    and preflight["field_locality"]
                    and preflight["numeric_pass"]
                    and preflight["source_rules_pass"]
                    and preflight["source_locality_pass"]
                    and all(row["passed"] for row in predicates)
                )
                rescored["compiled_predicates"] = predicates
                rescored["gates"]["target_fulfilled"] = target
                rescored["gates"]["anchor"] = target and rescored["gates"]["structural_locality"]
                rescored["gates"]["scale"] = target and all(row["passed"] for row in preflight["numeric_predicates"] + predicates)
                rescored["gates"]["final_pass"] = all(rescored["gates"].values())

            old_final = original["gates"]["final_pass"]
            new_final = rescored["gates"]["final_pass"]
            if rescored["gates"] != original["gates"]:
                changed.append({"task_id": task_id, "seed": seed, "before": old_final, "after": new_final})
            parent_graph = base.artifact_graph(Path(original["parent_compile"].get("urdf_path") or ""), task) if original["parent_compile"].get("urdf_path") else {"links": {}, "joints": {}}
            edited_graph = base.artifact_graph(Path(original["edited_compile"].get("urdf_path") or ""), task) if original["edited_compile"].get("urdf_path") else {"links": {}, "joints": {}}
            score = {
                "protocol_id": scorer["protocol_id"],
                "task_id": task_id,
                "seed": seed,
                "hashes": {
                    "source_manifest_sha256": mapping["source_manifest_sha256"],
                    "base_mapping_sha256": sha256(base_mapping_path),
                    "erratum_mapping_sha256": mapping_sha,
                    "erratum_scorer_sha256": scorer_sha,
                    "rescore_runner_sha256": runner_sha,
                    "source_record_sha256": sha256(record_path),
                    "parent_compile_result_sha256": sha256(run_root / "cases" / task_id / f"seed_{seed:02d}" / "parent/compile_result.json"),
                    "edited_compile_result_sha256": sha256(run_root / "cases" / task_id / f"seed_{seed:02d}" / "edited/compile_result.json"),
                    "parent_urdf_sha256": original["parent_compile"].get("urdf_sha256"),
                    "edited_urdf_sha256": original["edited_compile"].get("urdf_sha256"),
                    "parent_artifact_graph_sha256": object_sha256(parent_graph),
                    "edited_artifact_graph_sha256": object_sha256(edited_graph),
                },
                "numeric_predicates": preflight["numeric_predicates"],
                "compiled_predicates": rescored["compiled_predicates"],
                "changed_artifact_links": rescored["changed_artifact_links"],
                "changed_artifact_joints": rescored["changed_artifact_joints"],
                "unexpected_changed_links": rescored["unexpected_changed_links"],
                "unexpected_changed_joints": rescored["unexpected_changed_joints"],
                "compile_verdicts": {"parent": original["parent_compile"]["verdict"], "edited": original["edited_compile"]["verdict"]},
                "frozen_as_run_gates": original["gates"],
                "gates": rescored["gates"],
                "final_pass": new_final,
            }
            target = output / "scores" / task_id / f"seed_{seed:02d}" / "score.json"
            base.dump_json(target, score)
            scores.append(score)

    non_s05_changes = [row for row in changed if row["task_id"] != "S05"]
    if non_s05_changes:
        raise RuntimeError(f"non-S05 scores changed: {non_s05_changes}")
    expected_changes = [{"task_id": "S05", "seed": seed, "before": False, "after": True} for seed in mapping["seeds"]]
    if changed != expected_changes:
        raise RuntimeError(f"unexpected changed-case set: {changed}")

    task_rows = []
    for task_id in sorted(mappings):
        subset = [row for row in scores if row["task_id"] == task_id]
        task_rows.append({
            "task_id": task_id,
            "final_pass_count": sum(row["final_pass"] for row in subset),
            "main_seed_pass": next(row["final_pass"] for row in subset if row["seed"] == 0),
            "propagation_16_of_16": all(row["final_pass"] for row in subset),
        })
    classes = []
    for name in ("A", "M", "S"):
        subset = [row for row in scores if row["task_id"].startswith(name)]
        tasks = [row for row in task_rows if row["task_id"].startswith(name)]
        passed = sum(row["final_pass"] for row in subset)
        classes.append({
            "class": name,
            "case_pass": passed,
            "case_total": len(subset),
            "case_rate": passed / len(subset),
            "case_wilson_95": wilson(passed, len(subset)),
            "main_asset_pass": sum(row["main_seed_pass"] for row in tasks),
            "main_asset_total": len(tasks),
            "propagation_pass": sum(row["propagation_16_of_16"] for row in tasks),
            "propagation_total": len(tasks),
        })
    passed = sum(row["final_pass"] for row in scores)
    main_pass = sum(row["main_seed_pass"] for row in task_rows)
    propagation_pass = sum(row["propagation_16_of_16"] for row in task_rows)
    summary = {
        "protocol_id": scorer["protocol_id"],
        "status": "CONTRACT_CORRECTED_COMPLETE",
        "artifact_only_rescore": True,
        "compile_invocations": 0,
        "hashes": {
            "source_manifest_sha256": mapping["source_manifest_sha256"],
            "base_mapping_sha256": sha256(base_mapping_path),
            "erratum_mapping_sha256": mapping_sha,
            "erratum_scorer_sha256": scorer_sha,
            "rescore_runner_sha256": runner_sha,
        },
        "frozen_as_run": {"case_pass": 255, "case_total": 288, "main_asset_pass": 16, "main_asset_total": 18, "propagation_pass": 15, "propagation_total": 18},
        "contract_corrected": {
            "case_pass": passed,
            "case_total": len(scores),
            "case_rate": passed / len(scores),
            "case_wilson_95": wilson(passed, len(scores)),
            "main_asset_pass": main_pass,
            "main_asset_total": len(task_rows),
            "main_asset_wilson_95": wilson(main_pass, len(task_rows)),
            "propagation_pass": propagation_pass,
            "propagation_total": len(task_rows),
            "propagation_wilson_95": wilson(propagation_pass, len(task_rows)),
        },
        "class_results": classes,
        "task_results": task_rows,
        "verification": {
            "score_files": len(scores),
            "changed_case_count": len(changed),
            "changed_cases": changed,
            "non_s05_changed_case_count": 0,
            "s05_all_16_changed_false_to_true": True,
            "a01_unchanged_0_of_16": next(row["final_pass_count"] for row in task_rows if row["task_id"] == "A01") == 0,
            "s01_unchanged_15_of_16": next(row["final_pass_count"] for row in task_rows if row["task_id"] == "S01") == 15,
        },
    }
    base.dump_json(output / "summary.json", summary)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
