#!/usr/bin/env python3
"""Freeze the output-independent Table 4 Constraints v2 benchmark."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path


EXP_ROOT = Path(__file__).resolve().parents[1]
OUT_ROOT = EXP_ROOT / "reference" / "table4_constraints_v2"
FROZEN_AT = "2026-08-11T00:00:00Z"


TASKS = [
    ("T4C001", "workshop_stool", "simple", "a freestanding four-legged workshop stool", [
        ("leg", ["leg", "support_leg"], 4),
        ("footrest_ring", ["footrest_ring", "foot_ring"], 1),
    ], [("z_extent", 0.60), ("horizontal_max", 0.36)]),
    ("T4C002", "six_drawer_cabinet", "medium", "a cabinet with six separate sliding drawers", [
        ("drawer_front", ["drawer", "drawer_front"], 6),
        ("drawer_pull", ["drawer_pull", "drawer_handle", "pull_handle"], 6),
    ], [("horizontal_max", 0.90)]),
    ("T4C003", "step_ladder", "simple", "an upright step ladder", [
        ("side_rail", ["side_rail", "ladder_rail", "rail"], 2),
        ("rung", ["rung", "step"], 7),
    ], [("z_extent", 1.80)]),
    ("T4C004", "park_bench", "medium", "a park bench with a slatted seat", [
        ("leg", ["leg", "support_leg"], 4),
        ("seat_slat", ["seat_slat", "seat_board", "slat"], 5),
    ], [("horizontal_max", 1.50)]),
    ("T4C005", "utility_cart", "medium", "an upright rolling utility cart", [
        ("wheel", ["wheel", "caster", "caster_wheel"], 4),
        ("shelf", ["shelf", "tray", "platform"], 3),
    ], [("horizontal_max", 0.90)]),
    ("T4C006", "office_chair", "complex", "an upright office chair", [
        ("caster_wheel", ["caster_wheel", "caster", "wheel"], 5),
        ("armrest", ["armrest", "arm_rest", "chair_arm"], 2),
    ], [("z_extent", 1.10)]),
    ("T4C007", "work_table", "simple", "a rectangular work table", [
        ("leg", ["leg", "table_leg", "support_leg"], 4),
        ("tabletop", ["tabletop", "table_top", "top"], 1),
    ], [("horizontal_max", 1.20)]),
    ("T4C008", "storage_rack", "medium", "an upright open storage rack", [
        ("vertical_post", ["vertical_post", "upright_post", "post"], 4),
        ("shelf", ["shelf", "shelf_board"], 4),
    ], [("z_extent", 1.80)]),
    ("T4C009", "dumbbell", "simple", "a symmetric plate-loaded dumbbell", [
        ("weight_plate", ["weight_plate", "plate", "disc"], 4),
        ("collar", ["collar", "locking_collar"], 2),
    ], [("horizontal_max", 0.45)]),
    ("T4C010", "wall_hook_rack", "medium", "a horizontal wall-mounted hook rack", [
        ("hook", ["hook", "coat_hook"], 8),
        ("mounting_rail", ["mounting_rail", "back_rail", "rail"], 2),
    ], [("horizontal_max", 0.80)]),
    ("T4C011", "cable_spool", "simple", "a cable spool standing on its circular flanges", [
        ("flange", ["flange", "side_flange", "spool_flange"], 2),
        ("hub", ["hub", "spool_hub", "drum"], 1),
    ], [("horizontal_max", 0.40)]),
    ("T4C012", "valve_manifold", "complex", "a four-outlet valve manifold", [
        ("valve_handle", ["valve_handle", "handle", "knob"], 4),
        ("outlet_pipe", ["outlet_pipe", "outlet", "port"], 4),
    ], [("horizontal_max", 0.60)]),
    ("T4C013", "two_door_cabinet", "medium", "an upright two-door storage cabinet", [
        ("door", ["door", "cabinet_door"], 2),
        ("internal_shelf", ["internal_shelf", "shelf", "shelf_board"], 3),
    ], [("horizontal_max", 1.00)]),
    ("T4C014", "rolling_toolbox", "complex", "an upright rolling mechanic's toolbox", [
        ("drawer", ["drawer", "drawer_front"], 3),
        ("caster_wheel", ["caster_wheel", "caster", "wheel"], 4),
    ], [("horizontal_max", 0.70)]),
    ("T4C015", "bike_parking_rack", "medium", "a floor-mounted bicycle parking rack", [
        ("parking_loop", ["parking_loop", "bike_loop", "wheel_loop", "loop"], 5),
    ], [("horizontal_max", 1.20)]),
    ("T4C016", "pipe_manifold", "complex", "a horizontal pipe manifold with separately modeled outlet nozzles", [
        ("outlet_nozzle", ["outlet_nozzle", "outlet", "nozzle", "port"], 4),
    ], [("horizontal_max", 0.40)]),
    ("T4C017", "platform_trolley", "simple", "a low platform trolley", [
        ("wheel", ["wheel", "caster", "caster_wheel"], 4),
    ], [("horizontal_max", 0.80), ("horizontal_min", 0.50)]),
    ("T4C018", "wall_shelf", "simple", "an upright wall shelf unit", [
        ("shelf_board", ["shelf_board", "shelf", "board"], 3),
    ], [("z_extent", 0.90)]),
]


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def canonical_json(payload: object) -> bytes:
    return (json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True) + "\n").encode("utf-8")


def prompt_for(description: str, counts: list[tuple[str, list[str], int]], numerics: list[tuple[str, float]]) -> str:
    count_text = ", ".join(f"exactly {target} distinct {role.replace('_', ' ')} part(s)" for role, _, target in counts)
    dimension_labels = {
        "z_extent": "overall upright height",
        "horizontal_max": "larger horizontal overall extent",
        "horizontal_min": "smaller horizontal overall extent",
    }
    numeric_text = ", ".join(
        f"{dimension_labels[measure]} {target_m:.2f} m ({target_m * 1000:.0f} mm)"
        for measure, target_m in numerics
    )
    return (
        f"Create {description}. The final object must contain {count_text}. "
        f"Its {numeric_text}. Model every counted role as a separately named object, part, link, or visual. "
        "Keep the object upright on the ground plane, use Z as up, center it near the origin, and preserve real-world scale."
    )


def build_task(task: tuple) -> dict:
    task_id, category, difficulty, description, counts, numerics = task
    constraints = []
    for index, (role, aliases, target) in enumerate(counts, 1):
        constraints.append({
            "constraint_id": f"{task_id}-C{index:02d}",
            "type": "exact_count",
            "role": role,
            "aliases": aliases,
            "target": target,
            "measurement": {
                "source": "final_artifact_only",
                "units": "mesh-bearing named nodes_or_visuals",
                "matcher": "normalized token-boundary alias match",
                "deduplication": "one final renderable node_or_visual counts once",
                "unmeasurable_policy": "not passed and excluded from conditional denominator",
            },
        })
    for index, (measure, target_m) in enumerate(numerics, len(counts) + 1):
        constraints.append({
            "constraint_id": f"{task_id}-C{index:02d}",
            "type": "numeric_dimension",
            "measure": measure,
            "target_m": target_m,
            "relative_tolerance": 0.10,
            "absolute_tolerance_m": 0.005,
            "measurement": {
                "source": "final_artifact_world_aabb",
                "glb_urdf_units": "m",
                "step_units": "mm converted to m",
                "z_up_required": True,
            },
        })
    return {
        "schema_version": 2,
        "benchmark_id": "table4_constraints_v2",
        "task_id": task_id,
        "category": category,
        "difficulty": difficulty,
        "input_modality": "text",
        "prompt": prompt_for(description, counts, numerics),
        "constraints": constraints,
    }


def main() -> None:
    OUT_ROOT.mkdir(parents=True, exist_ok=True)
    specs_root = OUT_ROOT / "specs"
    specs_root.mkdir(parents=True, exist_ok=True)
    tasks = [build_task(task) for task in TASKS]
    count_constraints = sum(c["type"] == "exact_count" for task in tasks for c in task["constraints"])
    numeric_constraints = sum(c["type"] == "numeric_dimension" for task in tasks for c in task["constraints"])
    assert len(tasks) == 18
    assert count_constraints == 32
    assert numeric_constraints == 20
    assert count_constraints + numeric_constraints == 52

    prompt_rows = []
    spec_hashes = {}
    for task in tasks:
        spec_path = specs_root / f"{task['task_id']}.json"
        spec_bytes = canonical_json(task)
        if spec_path.exists() and spec_path.read_bytes() != spec_bytes:
            raise RuntimeError(f"frozen spec differs: {spec_path}")
        spec_path.write_bytes(spec_bytes)
        spec_hashes[task["task_id"]] = sha256_bytes(spec_bytes)
        prompt_rows.append({
            "task_id": task["task_id"],
            "category": task["category"],
            "difficulty": task["difficulty"],
            "input_modality": task["input_modality"],
            "prompt": task["prompt"],
            "spec_sha256": spec_hashes[task["task_id"]],
        })

    prompts_bytes = b"".join(
        (json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n").encode("utf-8")
        for row in prompt_rows
    )
    prompts_path = OUT_ROOT / "prompts.jsonl"
    if prompts_path.exists() and prompts_path.read_bytes() != prompts_bytes:
        raise RuntimeError(f"frozen prompts differ: {prompts_path}")
    prompts_path.write_bytes(prompts_bytes)

    protocol = {
        "schema_version": 2,
        "benchmark_id": "table4_constraints_v2",
        "frozen_at": FROZEN_AT,
        "task_count": len(tasks),
        "constraint_count": count_constraints + numeric_constraints,
        "count_constraint_count": count_constraints,
        "numeric_constraint_count": numeric_constraints,
        "prompt_manifest_sha256": sha256_bytes(prompts_bytes),
        "spec_sha256": spec_hashes,
        "generation_policy": {
            "one_final_artifact_per_method_task": True,
            "method_native_default_decoding": True,
            "repair_budget": 0,
            "timeout_seconds": 1800,
            "specs_hidden_from_generation": False,
            "note": "Every scored target is stated in the prompt; measurement recipes and tolerances are frozen before generation.",
        },
        "panels": {
            "structured_main": ["ours", "naive_same_llm", "articraft", "lam", "blenderllm"],
            "cad_numeric": ["text_to_cadquery", "text2cad"],
            "image_count": ["cad_coder"],
            "paper_reference_only": ["nova3d"],
        },
        "metrics": {
            "coverage": "measurable / all applicable constraints",
            "satisfaction": "passed / all applicable constraints",
            "conditional_accuracy": "passed / measurable constraints",
            "count_pass": "passed / all exact-count constraints",
            "numeric_pass": "passed / all applicable numeric constraints",
        },
        "anti_leakage": [
            "Do not infer targets or tolerances from generated artifacts.",
            "Do not edit prompts/specs after any common-task generation starts.",
            "Score final exported artifacts rather than source constants or self-reports.",
            "Keep modality-incompatible metrics N/A rather than zero.",
        ],
    }
    protocol_bytes = canonical_json(protocol)
    protocol_path = OUT_ROOT / "protocol.json"
    if protocol_path.exists() and protocol_path.read_bytes() != protocol_bytes:
        raise RuntimeError(f"frozen protocol differs: {protocol_path}")
    protocol_path.write_bytes(protocol_bytes)
    print(json.dumps({
        "output": str(OUT_ROOT),
        "tasks": len(tasks),
        "constraints": count_constraints + numeric_constraints,
        "count": count_constraints,
        "numeric": numeric_constraints,
        "prompt_manifest_sha256": protocol["prompt_manifest_sha256"],
    }, indent=2))


if __name__ == "__main__":
    main()
