#!/usr/bin/env python3
"""Run the frozen Articraft native-fork Table 5 editability cohort.

The default mode is an offline preflight. It creates an output-local repository
snapshot, compiles every frozen parent and historical gold record, and applies
the same deterministic scorer that will be used for fresh children. Paid API
execution requires --execute and the exact manifest SHA-256.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
import trimesh


SCRIPT_DIR = Path(__file__).resolve().parent
EXP_ROOT = SCRIPT_DIR.parent
REPO_ROOT = EXP_ROOT.parent
WORKSPACE_ROOT = REPO_ROOT.parent.resolve()
ARTICRAFT_ROOT = REPO_ROOT / "articraft_data"
DEFAULT_MANIFEST = EXP_ROOT / "reference" / "editability_v2" / "articraft_parent_manifest.json"
DEFAULT_OUTPUT = EXP_ROOT / "runtime" / "nano3d_editability_v2" / "articraft_native_fork"
SCORER_CONTRACT = EXP_ROOT / "reference" / "editability_v2" / "articraft_scorer_contract_v2.json"
BUDGET_GUARD_SOURCE = SCRIPT_DIR / "articraft_budget_sitecustomize.py"
MODEL_SNAPSHOT = "qwen3.7-max-2026-05-20"
PROVIDER = "dashscope"
THINKING_LEVEL = "high"
MAX_TURNS = 30
RECORD_TIMEOUT_SECONDS = 1800
PROMPT_CONTRACT_SHA256 = "0ad1443cea1078af2dcfba49b0f2f81e6caf34bf2b4d39600e62ebc871ef3965"
FROZEN_PRICING = {
    "currency": "CNY",
    "unit": "per_1m_tokens",
    "uncached_input": 12.0,
    "cached_input": 2.4,
    "output": 36.0,
    "retrieved_at": "2026-08-11",
    "usd_conversion": None,
}
FROZEN_NATIVE_SETTINGS = {
    "execution_mode": "official_articraft_fork",
    "provider": PROVIDER,
    "model_id": MODEL_SNAPSHOT,
    "thinking_level": THINKING_LEVEL,
    "max_turns": MAX_TURNS,
    "context_window_tokens": 1_000_000,
    "max_output_tokens": 65_536,
    "output_safety_tokens": 1_024,
    "max_cost_usd": None,
    "tools": ["read_file", "replace", "write_file", "compile_model", "probe_model", "find_examples"],
    "compile_feedback": True,
    "agent_repair_across_turns": True,
    "record_level_retries": 0,
    "provider_transient_max_attempts": 1,
    "source_truncation": False,
}
FROZEN_BUDGET = {
    "currency": "CNY",
    "per_record_hard_cap": 20.0,
    "cohort_hard_cap": 360.0,
    "pre_request_fail_closed": True,
    "input_token_upper_bound": "utf8_payload_bytes_plus_4096_special_token_reserve",
    "max_output_tokens_per_request": 65_536,
    "context_window_tokens": 1_000_000,
    "output_safety_tokens": 1_024,
    "provider_max_attempts": 1,
    "record_level_retries": 0,
    "usage_log": "cny_budget/usage.jsonl",
    "total_state": "cny_budget/total.json",
}

# Machine checks are intentionally simple, deterministic source/URDF predicates.
# They are preregistered here and applied unchanged to historical gold and fresh output.
SOURCE_CHECKS: dict[str, dict[str, list[str]]] = {
    "A01": {"all": ["caddy", "JointType.PRISMATIC", "JointType.REVOLUTE"]},
    "A02": {"all": ["media_knob", "knob_boss", "JointType.REVOLUTE"]},
    "A03": {"all": ["body_to_cover", "hinge_knuckle", "JointType.REVOLUTE"]},
    "A04": {"all": ["antenna", "JointType.PRISMATIC"]},
    "A05": {"all": ["becket", "eye"]},
    "A06": {"all": ["leg_specs", "legs_2", "JointType.REVOLUTE"]},
    "M01": {"all": ["upper_leg", "lower_leg", "knee_joint", "JointType.REVOLUTE"]},
    "M02": {"all": ["carriage", "JointType.PRISMATIC"], "any": ["rail_left", "slide_rail"]},
    "M03": {"all": ["drawer", "JointType.PRISMATIC"], "any": ["guide", "runner"]},
    "M04": {"all": ["drawer", "lid", "JointType.PRISMATIC"]},
    "M05": {"all": ["cup", "plunger", "perfor"], "any": ["round", "circle"]},
    "M06": {"all": ["grille_bar"], "none": ["grille_rib_"]},
    "S01": {"all": ["basket"], "any": ["oval", "ellipse"]},
    "S02": {"all": ["bucket", "handle"], "any": ["deep", "cone"]},
    "S03": {"all": ["center_top", "center_to_leaf_", "leaf_width"]},
    "S04": {"all": ["chassis", "key_"], "any": ["high-profile", "high_profile"]},
    "S05": {"all": ["base_chassis", "screen_lid", "bumper"]},
    "S06": {"all": ["chassis", "cylinder_body", "lift_arm"], "any": ["low-profile", "low_profile"]},
}

SCALE_LINKS: dict[str, list[list[str]]] = {
    "S01": [["basket_tub"]],
    "S02": [["bucket"]],
    "S03": [["center_section"], ["leaf_0"], ["leaf_1"]],
    "S04": [["chassis"]],
    "S05": [["base_chassis"], ["screen_lid"]],
    "S06": [["chassis"]],
}

# URDF rules freeze the allowed semantic delta. Names are case-insensitive
# substrings. All visual and joint changes outside these patterns are failures.
URDF_RULES: dict[str, dict[str, Any]] = {
    "A01": {"structure": ["caddy"], "geometry": ["caddy"], "required": ["inner_caddy"], "anchors": ["basket_tub"], "preserve": ["bail_handle"], "joints": [("tub_to_caddy", "prismatic", "basket_tub", "inner_caddy", (0, 0, 1))]},
    "A02": {"structure": ["media_knob"], "geometry": ["media_knob", "knob_boss", "chassis", "keybed"], "required": ["media_knob", "knob_boss"], "anchors": ["chassis"], "preserve": ["key_"], "joints": [("chassis_to_media_knob", "revolute", "chassis", "media_knob", (0, 0, 1))]},
    "A03": {"structure": ["cover"], "geometry": ["cover", "hinge_knuckle"], "required": ["cover", "hinge_knuckle"], "anchors": ["body"], "preserve": ["lcd", "button"], "joints": [("body_to_cover", "revolute", "body", "cover", (1, 0, 0), (math.radians(90), math.radians(110)))]},
    "A04": {"structure": ["antenna"], "geometry": ["antenna", "radio_window"], "required": ["antenna_mast", "antenna_boss", "antenna_sleeve"], "anchors": ["body"], "preserve": ["flip_hinge", "lid", "key_"], "joints": [("antenna", "prismatic", "body", "antenna", None)]},
    "A05": {"structure": [], "geometry": ["becket"], "required": ["becket_eye", "becket_neck"], "anchors": ["mount_frame"], "preserve": ["sheave"], "joints": []},
    "A06": {"structure": ["legs", "front_legs", "rear_legs"], "geometry": ["legs", "leg_hinge_block", "front_legs", "rear_legs"], "required": ["legs_0", "legs_1", "legs_2"], "anchors": ["main_frame"], "preserve": ["wing"], "joints": [("frame_to_legs_0", "revolute", "main_frame", "legs_0", None), ("frame_to_legs_1", "revolute", "main_frame", "legs_1", None), ("frame_to_legs_2", "revolute", "main_frame", "legs_2", None)]},
    "M01": {"structure": ["leg", "knee"], "geometry": ["leg", "knee", "adjustment_link"], "required": ["left_upper_leg", "left_lower_leg", "right_upper_leg", "right_lower_leg"], "anchors": ["head"], "preserve": ["handle", "adjustment_link", "adjustment_wheel"], "joints": [("left_knee", "revolute", "left_upper_leg", "left_lower_leg", None), ("right_knee", "revolute", "right_upper_leg", "right_lower_leg", None)]},
    "M02": {"structure": ["tray_carriage", "slide_latch", "tray_slide", "slide_latch_pivot", "tray_tilt"], "geometry": ["tray_carriage", "slide_latch", "rail", "tray_cross_pin", "wrist_yoke"], "required": ["tray_carriage", "slide_latch", "slide_rail_0", "slide_rail_1"], "anchors": ["wrist_yoke", "laptop_tray"], "preserve": ["clamp", "arm_"], "joints": [("tray_slide", "prismatic", "wrist_yoke", "tray_carriage", (1, 0, 0)), ("slide_latch_pivot", "revolute", "tray_carriage", "slide_latch", None)]},
    "M03": {"structure": ["drawer"], "geometry": ["drawer", "cabinet", "frame", "lower_shelf", "shelf_fastener", "shelf_rim"], "required": ["drawer_0", "drawer_1"], "anchors": ["frame"], "preserve": ["caster", "top_skin"], "joints": [("drawer_slide_0", "prismatic", "frame", "drawer_0", None), ("drawer_slide_1", "prismatic", "frame", "drawer_1", None)]},
    "M04": {"structure": [], "geometry": ["floor_tray", "bin_", "drawer", "panel", "handle_bar", "front_wall", "side_"], "required": ["top_panel", "drawer_front"], "anchors": ["frame"], "preserve": ["shelf"], "joints": [("frame_to_tall_floor_tray_a", "prismatic", "frame", "tall_floor_tray_a", None), ("frame_to_low_floor_tray_d", "prismatic", "frame", "low_floor_tray_d", None)]},
    "M05": {"structure": [], "geometry": ["press_body", "basket", "cup", "plunger", "rim"], "required": ["basket_ribs", "basket_rim"], "anchors": ["press_body"], "preserve": ["plunger_arm_hinge", "handle"], "joints": []},
    "M06": {"structure": [], "geometry": ["grille"], "required": ["grille_bar_0", "grille_bar_1", "grille_bar_2"], "forbidden": ["grille_rib_"], "anchors": ["cabinet"], "preserve": ["knob", "tuning"], "joints": []},
    "S01": {"structure": [], "geometry": ["basket_tub", "handle_0", "handle_1"], "required": ["basket_tub"], "anchors": ["handle_0", "handle_1"], "preserve": ["tub_to_handle_0", "tub_to_handle_1"], "joints": []},
    "S02": {"structure": [], "geometry": ["bucket", "handle"], "required": ["bucket"], "anchors": ["handle"], "preserve": ["bucket_to_handle"], "joints": []},
    "S03": {"structure": [], "geometry": ["center_section", "leaf_"], "required": ["center_top", "leaf_0", "leaf_1"], "anchors": ["center_to_leaf_0", "center_to_leaf_1"], "preserve": ["leg_frame", "locking_brace"], "joints": []},
    "S04": {"structure": [], "geometry": ["chassis"], "required": ["chassis"], "anchors": ["key_"], "preserve": ["key_"], "joints": []},
    "S05": {"structure": [], "geometry": ["base_chassis", "screen_lid", "lower_shell", "lid_shell", "corner_bumper"], "required": ["corner_bumper_0", "corner_bumper_1", "corner_bumper_2", "corner_bumper_3"], "anchors": ["base_chassis", "screen_lid"], "preserve": ["base_to_screen", "key_", "trackpad", "display_panel"], "joints": []},
    "S06": {"structure": ["lift_arm_hinge", "piston_slide"], "geometry": ["chassis", "cylinder_body", "lift_arm", "piston", "mount_datum", "pump_link_pivot"], "required": ["chassis", "cylinder_body", "lift_arm"], "anchors": ["lift_arm_hinge", "pump_handle_hinge", "caster_swivel_0"], "preserve": ["piston", "saddle", "front_wheel", "rear_caster"], "joints": []},
}


def contained(path: Path, *, strict: bool = False) -> Path:
    resolved = path.resolve(strict=strict)
    if resolved != WORKSPACE_ROOT and WORKSPACE_ROOT not in resolved.parents:
        raise ValueError(f"path outside authorized workspace: {resolved}")
    return resolved


def relative(path: Path) -> str:
    return str(contained(path).relative_to(WORKSPACE_ROOT))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with contained(path, strict=True).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_json(path: Path) -> Any:
    return json.loads(contained(path, strict=True).read_text(encoding="utf-8"))


def dump_json(path: Path, payload: Any) -> None:
    path = contained(path)
    contained(path.parent).mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def now_utc() -> str:
    return datetime.now(timezone.utc).isoformat()


def scorer_rule_tables_sha256() -> str:
    payload = {
        "source_checks": SOURCE_CHECKS,
        "scale_links": SCALE_LINKS,
        "urdf_rules": URDF_RULES,
    }
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def validate_manifest(manifest: dict[str, Any], manifest_path: Path) -> list[str]:
    errors: list[str] = []
    tasks = manifest.get("tasks")
    if not str(manifest.get("frozen_at_utc", "")).strip():
        errors.append("FROZEN_AT_UTC_MISSING")
    try:
        scorer_contract = contained(WORKSPACE_ROOT / manifest["scorer_contract_path"], strict=True)
        budget_guard = contained(WORKSPACE_ROOT / manifest["budget_guard_path"], strict=True)
    except (KeyError, OSError, ValueError) as exc:
        errors.append(f"FROZEN_ARTIFACT_UNAVAILABLE:{exc}")
    else:
        if scorer_contract != contained(SCORER_CONTRACT, strict=True):
            errors.append("SCORER_CONTRACT_PATH_MISMATCH")
        if manifest.get("scorer_sha256") != sha256(scorer_contract):
            errors.append("SCORER_CONTRACT_SHA256_MISMATCH")
        contract = load_json(scorer_contract)
        if contract.get("semantic_rule_tables_sha256") != scorer_rule_tables_sha256():
            errors.append("SCORER_RULE_TABLES_SHA256_MISMATCH")
        if budget_guard != contained(BUDGET_GUARD_SOURCE, strict=True):
            errors.append("BUDGET_GUARD_PATH_MISMATCH")
        if manifest.get("budget_guard_sha256") != sha256(budget_guard):
            errors.append("BUDGET_GUARD_SHA256_MISMATCH")
    if manifest.get("execution_runner_sha256") != sha256(Path(__file__).resolve()):
        errors.append("EXECUTION_RUNNER_SHA256_MISMATCH")
    if manifest.get("cohort") != "curated_challenge_set":
        errors.append("COHORT_NOT_CURATED_CHALLENGE_SET")
    if manifest.get("prompt_contract_sha256") != PROMPT_CONTRACT_SHA256:
        errors.append("PROMPT_CONTRACT_SHA256_MISMATCH")
    method_protocols = manifest.get("method_protocols", {})
    if method_protocols.get("articraft_native") != FROZEN_NATIVE_SETTINGS:
        errors.append("FROZEN_NATIVE_SETTINGS_MISMATCH")
    if manifest.get("pricing_snapshot") != FROZEN_PRICING:
        errors.append("PRICING_SNAPSHOT_MISMATCH")
    if manifest.get("budget_guard") != FROZEN_BUDGET:
        errors.append("FROZEN_BUDGET_MISMATCH")
    if not isinstance(tasks, list) or len(tasks) != 18:
        return [*errors, "TASK_COUNT_NOT_18"]
    if len({task.get("task_id") for task in tasks}) != 18:
        errors.append("TASK_IDS_NOT_UNIQUE")
    if len({task.get("parent_record_id") for task in tasks}) != 18:
        errors.append("PARENT_RECORD_IDS_NOT_UNIQUE")
    counts = {key: 0 for key in ("additive", "modified", "scale")}
    for task in tasks:
        task_id = str(task.get("task_id"))
        edit_class = task.get("edit_class")
        if edit_class in counts:
            counts[edit_class] += 1
        else:
            errors.append(f"{task_id}:UNKNOWN_EDIT_CLASS")
        if task_id not in SOURCE_CHECKS:
            errors.append(f"{task_id}:MACHINE_SCORER_MISSING")
        parent_id = task.get("parent_record_id")
        gold_id = task.get("historical_gold_record_id")
        parent_dir = ARTICRAFT_ROOT / "data" / "records" / str(parent_id)
        gold_dir = ARTICRAFT_ROOT / "data" / "records" / str(gold_id)
        try:
            parent = load_json(parent_dir / "record.json")
            gold = load_json(gold_dir / "record.json")
            source_map = contained(REPO_ROOT / task["source_map"], strict=True)
            model_path = contained(WORKSPACE_ROOT / task["parent_model_path"], strict=True)
        except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
            errors.append(f"{task_id}:INPUT_UNAVAILABLE:{exc}")
            continue
        if parent.get("lineage", {}).get("parent_record_id"):
            errors.append(f"{task_id}:PARENT_NOT_ROOT")
        lineage = gold.get("lineage", {})
        if lineage.get("parent_record_id") != parent_id:
            errors.append(f"{task_id}:GOLD_NOT_DIRECT_CHILD")
        if lineage.get("parent_revision_id") != task.get("parent_revision_id"):
            errors.append(f"{task_id}:GOLD_PARENT_REVISION_MISMATCH")
        source_map_text = source_map.read_text(encoding="utf-8")
        if str(gold_id) not in source_map_text:
            errors.append(f"{task_id}:GOLD_NOT_IN_SOURCE_MAP")
        actual_model_hash = sha256(model_path)
        if actual_model_hash != task.get("parent_model_sha256"):
            errors.append(f"{task_id}:PARENT_MODEL_SHA256_MISMATCH")
        if actual_model_hash != parent.get("hashes", {}).get("model_py_sha256"):
            errors.append(f"{task_id}:PARENT_RECORD_PROVENANCE_HASH_MISMATCH")
        if not str(task.get("original_object_prompt", "")).strip():
            errors.append(f"{task_id}:ORIGINAL_OBJECT_PROMPT_EMPTY")
        if task.get("normalized_edit_instruction") != task.get("normalized_instruction"):
            errors.append(f"{task_id}:NORMALIZED_INSTRUCTION_ALIAS_MISMATCH")
        if not isinstance(task.get("gold"), dict) or not task["gold"]:
            errors.append(f"{task_id}:GOLD_OBJECT_EMPTY")
        if edit_class == "scale":
            measurement = task.get("historical_gold_measurement")
            if not isinstance(measurement, dict) or not measurement.get("measured_before_fresh_run"):
                errors.append(f"{task_id}:HISTORICAL_GOLD_MEASUREMENT_MISSING")
            elif measurement.get("scorer_version") != "urdf_semantic_v2" or measurement.get("passed") is not True:
                errors.append(f"{task_id}:HISTORICAL_GOLD_MEASUREMENT_INVALID")
            if task.get("gold", {}).get("pre_freeze_measurement") != measurement:
                errors.append(f"{task_id}:GOLD_PRE_FREEZE_MEASUREMENT_MISMATCH")
    if counts != {"additive": 6, "modified": 6, "scale": 6}:
        errors.append("CLASS_DISTRIBUTION_NOT_6_6_6")
    if manifest.get("cohort_distribution") != counts:
        errors.append("DECLARED_COHORT_DISTRIBUTION_MISMATCH")
    if sha256(manifest_path) == "":
        errors.append("UNREACHABLE_MANIFEST_HASH")
    return sorted(set(errors))


def copy_snapshot(manifest: dict[str, Any], snapshot: Path, manifest_hash: str) -> None:
    snapshot = contained(snapshot)
    stamp = snapshot / "MANIFEST.sha256"
    if snapshot.exists():
        if not stamp.is_file() or stamp.read_text(encoding="utf-8").strip() != manifest_hash:
            raise RuntimeError(f"existing snapshot does not match manifest: {snapshot}")
        return
    snapshot.mkdir(parents=True)
    for name in ("sdk", "agent", "articraft", "storage", "cli"):
        os.symlink(contained(ARTICRAFT_ROOT / name, strict=True), snapshot / name)
    for name in ("pyproject.toml", "uv.lock", ".python-version", "FORK_VARIANTS.md", "VARIANT_PIPELINE.md"):
        source = contained(ARTICRAFT_ROOT / name, strict=True)
        if source.exists():
            os.symlink(source, snapshot / name)
    suffix_source = contained(ARTICRAFT_ROOT / "picture_expansion" / "FORK_SUFFIX.txt", strict=True)
    (snapshot / "picture_expansion").mkdir()
    shutil.copy2(suffix_source, snapshot / "picture_expansion" / "FORK_SUFFIX.txt")
    budget_guard_dir = snapshot / "budget_guard"
    budget_guard_dir.mkdir()
    shutil.copy2(contained(BUDGET_GUARD_SOURCE, strict=True), budget_guard_dir / "sitecustomize.py")
    source_data = contained(ARTICRAFT_ROOT / "data", strict=True)
    target_data = snapshot / "data"
    target_data.mkdir()
    for name in (
        "CATEGORY_PROMPT_GUIDE.md",
        "CATEGORY_SELECTION_REQUIREMENTS.md",
        "REJECTED_CATEGORIES.md",
        "supercategories.json",
    ):
        source = source_data / name
        if source.exists():
            shutil.copy2(source, target_data / name)
    for name in ("categories", "system_prompts"):
        source = source_data / name
        if source.exists():
            shutil.copytree(source, target_data / name)
    for name in ("records", "cache", "local", "index"):
        (target_data / name).mkdir(exist_ok=True)
    record_ids: set[str] = set()
    for task in manifest["tasks"]:
        record_ids.add(task["parent_record_id"])
        record_ids.add(task["historical_gold_record_id"])
    for record_id in sorted(record_ids):
        source = contained(source_data / "records" / record_id, strict=True)
        shutil.copytree(source, target_data / "records" / record_id)
    stamp.write_text(manifest_hash + "\n", encoding="utf-8")


def run_command(
    command: list[str],
    *,
    cwd: Path,
    timeout: int,
    log_path: Path,
    env: dict[str, str] | None = None,
) -> dict[str, Any]:
    started = time.monotonic()
    try:
        completed = subprocess.run(
            command,
            cwd=contained(cwd, strict=True),
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=timeout,
            check=False,
            env=env,
        )
        result = {
            "returncode": completed.returncode,
            "timed_out": False,
            "elapsed_seconds": round(time.monotonic() - started, 3),
            "command": command,
        }
        output = completed.stdout
    except subprocess.TimeoutExpired as exc:
        result = {
            "returncode": 124,
            "timed_out": True,
            "elapsed_seconds": round(time.monotonic() - started, 3),
            "command": command,
        }
        output = (exc.stdout or "") + "\nTIMEOUT\n"
    contained(log_path.parent).mkdir(parents=True, exist_ok=True)
    contained(log_path).write_text(output, encoding="utf-8")
    result["log"] = relative(log_path)
    return result


def compile_record(snapshot: Path, record_id: str, timeout: int, log_dir: Path) -> dict[str, Any]:
    command = [
        "uv",
        "run",
        "articraft",
        "compile",
        "--repo-root",
        str(snapshot),
        "--target",
        "full",
        "--validate",
        record_id,
    ]
    return run_command(
        command,
        cwd=ARTICRAFT_ROOT,
        timeout=timeout,
        log_path=log_dir / f"{record_id}.log",
    )


def compile_cohort(
    manifest: dict[str, Any], snapshot: Path, timeout: int, workers: int, output: Path
) -> dict[str, dict[str, Any]]:
    record_ids = sorted(
        {
            record_id
            for task in manifest["tasks"]
            for record_id in (task["parent_record_id"], task["historical_gold_record_id"])
        }
    )
    results: dict[str, dict[str, Any]] = {}
    with ThreadPoolExecutor(max_workers=workers) as pool:
        futures = {
            pool.submit(compile_record, snapshot, record_id, timeout, output / "compile_logs"): record_id
            for record_id in record_ids
        }
        for future in as_completed(futures):
            record_id = futures[future]
            results[record_id] = future.result()
    return results


def rpy_matrix(rpy: str | None) -> np.ndarray:
    values = [float(value) for value in (rpy or "0 0 0").split()]
    roll, pitch, yaw = values
    cx, sx = math.cos(roll), math.sin(roll)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cz, sz = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return rz @ ry @ rx


def geometry_corners(geometry: ET.Element, urdf_dir: Path) -> np.ndarray:
    box = geometry.find("box")
    cylinder = geometry.find("cylinder")
    sphere = geometry.find("sphere")
    mesh = geometry.find("mesh")
    if box is not None:
        half = np.array([float(v) for v in box.attrib["size"].split()]) / 2.0
        return trimesh.bounds.corners(np.array([-half, half]))
    if cylinder is not None:
        radius = float(cylinder.attrib["radius"])
        half = np.array([radius, radius, float(cylinder.attrib["length"]) / 2.0])
        return trimesh.bounds.corners(np.array([-half, half]))
    if sphere is not None:
        radius = float(sphere.attrib["radius"])
        half = np.array([radius, radius, radius])
        return trimesh.bounds.corners(np.array([-half, half]))
    if mesh is not None:
        mesh_path = contained(urdf_dir / mesh.attrib["filename"], strict=True)
        loaded = trimesh.load(mesh_path, force="mesh", process=False)
        bounds = np.asarray(loaded.bounds, dtype=float)
        corners = trimesh.bounds.corners(bounds)
        scale = np.array([float(v) for v in mesh.attrib.get("scale", "1 1 1").split()])
        return corners * scale
    raise ValueError("unsupported URDF geometry")


def link_extents(urdf_path: Path, aliases: list[str]) -> dict[str, Any]:
    urdf_path = contained(urdf_path, strict=True)
    root = ET.parse(urdf_path).getroot()
    links = [link for link in root.findall("link") if link.attrib.get("name") in aliases]
    if not links:
        raise ValueError(f"none of links found: {aliases}")
    points: list[np.ndarray] = []
    for link in links:
        for visual in link.findall("visual"):
            geometry = visual.find("geometry")
            if geometry is None:
                continue
            corners = geometry_corners(geometry, urdf_path.parent)
            origin = visual.find("origin")
            xyz = np.array(
                [float(v) for v in ((origin.attrib.get("xyz") if origin is not None else None) or "0 0 0").split()]
            )
            rotation = rpy_matrix(origin.attrib.get("rpy") if origin is not None else None)
            points.append(corners @ rotation.T + xyz)
    if not points:
        raise ValueError(f"link has no supported visuals: {aliases}")
    merged = np.vstack(points)
    bounds = np.array([merged.min(axis=0), merged.max(axis=0)])
    return {
        "matched_links": [link.attrib["name"] for link in links],
        "bounds": bounds.tolist(),
        "extents": (bounds[1] - bounds[0]).tolist(),
    }


def rounded(values: list[float] | np.ndarray, digits: int = 7) -> list[float]:
    return [round(float(value), digits) for value in values]


def visual_signature(visual: ET.Element, urdf_dir: Path) -> dict[str, Any]:
    geometry = visual.find("geometry")
    if geometry is None:
        raise ValueError("visual without geometry")
    corners = geometry_corners(geometry, urdf_dir)
    local_bounds = np.array([corners.min(axis=0), corners.max(axis=0)])
    origin = visual.find("origin")
    xyz = ((origin.attrib.get("xyz") if origin is not None else None) or "0 0 0").split()
    rpy = ((origin.attrib.get("rpy") if origin is not None else None) or "0 0 0").split()
    primitive = next((name for name in ("box", "cylinder", "sphere", "mesh") if geometry.find(name) is not None), "unknown")
    primitive_element = geometry.find(primitive) if primitive != "unknown" else None
    primitive_parameters = dict(sorted(primitive_element.attrib.items())) if primitive_element is not None else {}
    mesh_content_sha256 = None
    if primitive == "mesh" and primitive_element is not None:
        mesh_path = contained(urdf_dir / primitive_element.attrib["filename"], strict=True)
        mesh_content_sha256 = sha256(mesh_path)
    material = visual.find("material")
    material_signature: dict[str, Any] | None = None
    if material is not None:
        color = material.find("color")
        texture = material.find("texture")
        material_signature = {
            "name": material.attrib.get("name"),
            "color_rgba": color.attrib.get("rgba") if color is not None else None,
            "texture_filename": texture.attrib.get("filename") if texture is not None else None,
        }
    return {
        "primitive": primitive,
        "primitive_parameters": primitive_parameters,
        "mesh_content_sha256": mesh_content_sha256,
        "material": material_signature,
        "local_bounds": [rounded(local_bounds[0]), rounded(local_bounds[1])],
        "local_extents": rounded(local_bounds[1] - local_bounds[0]),
        "origin_xyz": rounded([float(value) for value in xyz]),
        "origin_rpy": rounded([float(value) for value in rpy]),
    }


def urdf_descriptor(urdf_path: Path) -> dict[str, Any]:
    urdf_path = contained(urdf_path, strict=True)
    root = ET.parse(urdf_path).getroot()
    links: set[str] = set()
    visuals: dict[str, dict[str, Any]] = {}
    for link in root.findall("link"):
        link_name = link.attrib["name"]
        links.add(link_name)
        for index, visual in enumerate(link.findall("visual")):
            visual_name = visual.attrib.get("name") or f"__visual_{index}"
            visuals[f"{link_name}/{visual_name}"] = visual_signature(visual, urdf_path.parent)
    joints: dict[str, dict[str, Any]] = {}
    for joint in root.findall("joint"):
        axis = joint.find("axis")
        limit = joint.find("limit")
        joints[joint.attrib["name"]] = {
            "type": joint.attrib["type"],
            "parent": joint.find("parent").attrib["link"],
            "child": joint.find("child").attrib["link"],
            "axis": rounded([float(value) for value in axis.attrib.get("xyz", "0 0 0").split()]) if axis is not None else None,
            "lower": round(float(limit.attrib["lower"]), 7) if limit is not None and "lower" in limit.attrib else None,
            "upper": round(float(limit.attrib["upper"]), 7) if limit is not None and "upper" in limit.attrib else None,
        }
    semantic_names = sorted(links | set(joints) | {name.split("/", 1)[1] for name in visuals})
    return {"links": sorted(links), "visuals": visuals, "joints": joints, "semantic_names": semantic_names}


def visual_extents(urdf_path: Path, key: str) -> list[float]:
    descriptor = urdf_descriptor(urdf_path)
    if key not in descriptor["visuals"]:
        raise ValueError(f"visual not found: {key}")
    return descriptor["visuals"][key]["local_extents"]


def matches_any(value: str, patterns: list[str]) -> bool:
    folded = value.lower()
    return any(pattern.lower() in folded for pattern in patterns)


def descriptor_delta(parent: dict[str, Any], child: dict[str, Any]) -> dict[str, Any]:
    parent_links, child_links = set(parent["links"]), set(child["links"])
    parent_joints, child_joints = parent["joints"], child["joints"]
    parent_visuals, child_visuals = parent["visuals"], child["visuals"]
    return {
        "links_added": sorted(child_links - parent_links),
        "links_removed": sorted(parent_links - child_links),
        "joints_added": sorted(set(child_joints) - set(parent_joints)),
        "joints_removed": sorted(set(parent_joints) - set(child_joints)),
        "joints_modified": sorted(
            name for name in set(parent_joints) & set(child_joints) if parent_joints[name] != child_joints[name]
        ),
        "visuals_added": sorted(set(child_visuals) - set(parent_visuals)),
        "visuals_removed": sorted(set(parent_visuals) - set(child_visuals)),
        "visuals_modified": sorted(
            name for name in set(parent_visuals) & set(child_visuals) if parent_visuals[name] != child_visuals[name]
        ),
    }


def joint_requirement(descriptor: dict[str, Any], requirement: tuple[Any, ...]) -> dict[str, Any]:
    name_pattern, joint_type, parent_pattern, child_pattern, expected_axis, *limit_ranges = requirement
    matches = [
        (name, value)
        for name, value in descriptor["joints"].items()
        if name_pattern.lower() in name.lower()
        and value["type"] == joint_type
        and parent_pattern.lower() in value["parent"].lower()
        and child_pattern.lower() in value["child"].lower()
    ]
    axis_pass = True
    if expected_axis is not None:
        axis_pass = any(
            value["axis"] is not None
            and np.allclose(np.abs(value["axis"]), np.abs(expected_axis), atol=1e-6)
            for _, value in matches
        )
    bounded_pass = all(
        value["type"] == "continuous" or (value["lower"] is not None and value["upper"] is not None and value["upper"] > value["lower"])
        for _, value in matches
    )
    limit_range_pass = True
    if limit_ranges:
        minimum, maximum = limit_ranges[0]
        limit_range_pass = any(
            value["upper"] is not None and minimum <= value["upper"] <= maximum
            for _, value in matches
        )
    return {
        "name_pattern": name_pattern,
        "matches": [name for name, _ in matches],
        "axis_pass": axis_pass,
        "bounded_or_continuous": bounded_pass,
        "upper_limit_range": list(limit_ranges[0]) if limit_ranges else None,
        "upper_limit_range_pass": limit_range_pass,
        "passed": bool(matches) and axis_pass and bounded_pass and limit_range_pass,
    }


def urdf_score(task_id: str, parent_urdf: Path, child_urdf: Path) -> dict[str, Any]:
    rule = URDF_RULES[task_id]
    parent = urdf_descriptor(parent_urdf)
    child = urdf_descriptor(child_urdf)
    delta = descriptor_delta(parent, child)
    parent_names = parent["semantic_names"]
    child_names = child["semantic_names"]
    required = {pattern: any(pattern.lower() in name.lower() for name in child_names) for pattern in rule["required"]}
    forbidden = {pattern: not any(pattern.lower() in name.lower() for name in child_names) for pattern in rule.get("forbidden", [])}
    anchors = {
        pattern: (
            any(pattern.lower() in name.lower() for name in parent_names)
            and any(pattern.lower() in name.lower() for name in child_names)
        )
        for pattern in rule["anchors"]
    }
    preserve = {
        pattern: (
            any(pattern.lower() in name.lower() for name in parent_names)
            and any(pattern.lower() in name.lower() for name in child_names)
        )
        for pattern in rule["preserve"]
    }
    joint_checks = [joint_requirement(child, requirement) for requirement in rule["joints"]]
    structural_changes = delta["links_added"] + delta["links_removed"] + delta["joints_added"] + delta["joints_removed"] + delta["joints_modified"]
    structural_locality = all(matches_any(name, rule["structure"]) for name in structural_changes)
    if not rule["structure"]:
        structural_locality = not structural_changes
    visual_changes = delta["visuals_added"] + delta["visuals_removed"] + delta["visuals_modified"]
    geometry_locality = bool(visual_changes) and all(matches_any(name, rule["geometry"]) for name in visual_changes)
    target = all(required.values()) and all(forbidden.values()) and all(check["passed"] for check in joint_checks)
    anchor = all(anchors.values())
    non_target = all(preserve.values()) and structural_locality and geometry_locality
    return {
        "target": {"required": required, "forbidden": forbidden, "joint_requirements": joint_checks, "passed": target},
        "anchor": {"patterns": anchors, "passed": anchor},
        "non_target": {"patterns": preserve, "passed": non_target},
        "structural_locality": {"allowed_patterns": rule["structure"], "changes": structural_changes, "passed": structural_locality},
        "geometry_locality": {"allowed_patterns": rule["geometry"], "changes": visual_changes, "passed": geometry_locality},
        "delta": delta,
        "passed": target and anchor and non_target and structural_locality and geometry_locality,
    }


def source_predicates(task_id: str, source: str) -> dict[str, Any]:
    config = SOURCE_CHECKS[task_id]
    folded = source.lower()
    all_checks = {token: token.lower() in folded for token in config.get("all", [])}
    any_tokens = config.get("any", [])
    none_checks = {token: token.lower() not in folded for token in config.get("none", [])}
    any_pass = not any_tokens or any(token.lower() in folded for token in any_tokens)
    return {
        "all": all_checks,
        "any": {token: token.lower() in folded for token in any_tokens},
        "none": none_checks,
        "passed": all(all_checks.values()) and any_pass and all(none_checks.values()),
    }


def scale_measurement(task_id: str, parent_urdf: Path, child_urdf: Path) -> dict[str, Any]:
    parent_groups = [link_extents(parent_urdf, aliases) for aliases in SCALE_LINKS[task_id]]
    child_groups = [link_extents(child_urdf, aliases) for aliases in SCALE_LINKS[task_id]]
    ratios = []
    for parent, child in zip(parent_groups, child_groups, strict=True):
        ratios.append(
            [child_value / parent_value for child_value, parent_value in zip(child["extents"], parent["extents"], strict=True)]
        )
    passed = False
    extra: dict[str, Any] = {}
    if task_id == "S01":
        major_ratio = max(ratios[0][:2])
        minor_ratio = min(ratios[0][:2])
        extra = {"basket_body": {"height_ratio": ratios[0][2], "major_horizontal_ratio": major_ratio, "minor_horizontal_ratio": minor_ratio}}
        passed = ratios[0][2] >= 1.25 and 0.90 <= major_ratio <= 1.00 and minor_ratio <= 0.85
    elif task_id == "S02":
        passed = ratios[0][2] >= 1.25 and max(ratios[0][0], ratios[0][1]) <= 0.85
        passed = passed and child_groups[0]["extents"][2] / max(child_groups[0]["extents"][:2]) >= 1.25
    elif task_id == "S03":
        parent_center = visual_extents(parent_urdf, "center_section/center_top")
        child_center = visual_extents(child_urdf, "center_section/center_top")
        parent_leaves = [visual_extents(parent_urdf, f"leaf_{index}/wood_panel") for index in (0, 1)]
        child_leaves = [visual_extents(child_urdf, f"leaf_{index}/wood_panel") for index in (0, 1)]
        center_ratio = min(child_center[:2]) / min(parent_center[:2])
        leaf_ratios = [child[-1] / parent[-1] for parent, child in zip(parent_leaves, child_leaves, strict=True)]
        extra = {"center_top": {"parent": parent_center, "child": child_center, "narrow_ratio": center_ratio}, "leaf_wood_panels": {"parent": parent_leaves, "child": child_leaves, "wide_ratios": leaf_ratios}}
        passed = center_ratio <= 0.70 and all(value >= 1.08 for value in leaf_ratios)
        passed = passed and abs(leaf_ratios[0] - leaf_ratios[1]) <= 0.02
    elif task_id == "S04":
        passed = ratios[0][2] >= 1.15 and all(0.95 <= value <= 1.05 for value in ratios[0][:2])
    elif task_id == "S05":
        parent_shells = [visual_extents(parent_urdf, "base_chassis/lower_shell"), visual_extents(parent_urdf, "screen_lid/lid_shell")]
        child_shells = [visual_extents(child_urdf, "base_chassis/lower_shell"), visual_extents(child_urdf, "screen_lid/lid_shell")]
        shell_ratios = [[c / p for p, c in zip(parent, child, strict=True)] for parent, child in zip(parent_shells, child_shells, strict=True)]
        extra = {"shell_visuals": {"parent": parent_shells, "child": child_shells, "ratios": shell_ratios}}
        passed = all(group[2] >= 1.50 for group in shell_ratios) and all(0.90 <= value <= 1.10 for group in shell_ratios for value in group[:2])
    elif task_id == "S06":
        parent_tray = visual_extents(parent_urdf, "chassis/chassis_tray")
        child_tray = visual_extents(child_urdf, "chassis/chassis_tray")
        tray_ratios = [c / p for p, c in zip(parent_tray, child_tray, strict=True)]
        parent_front_xmax = parent_groups[0]["bounds"][1][0]
        child_front_xmax = child_groups[0]["bounds"][1][0]
        front_xmax_ratio = child_front_xmax / parent_front_xmax
        extra = {"front_nose_chassis_tray": {"parent": parent_tray, "child": child_tray, "ratios": tray_ratios, "parent_front_xmax": parent_front_xmax, "child_front_xmax": child_front_xmax, "front_xmax_ratio": front_xmax_ratio}}
        passed = front_xmax_ratio >= 1.20 and tray_ratios[2] <= 0.70
    return {"parent": parent_groups, "child": child_groups, "ratios": ratios, "task_specific": extra, "passed": bool(passed)}


def score_record(task: dict[str, Any], parent_id: str, child_id: str, snapshot: Path) -> dict[str, Any]:
    child_model = contained(
        snapshot / "data" / "records" / child_id / "revisions" / "rev_000001" / "model.py",
        strict=True,
    )
    parent_urdf = contained(
        snapshot / "data" / "cache" / "record_materialization" / parent_id / "model.urdf",
        strict=True,
    )
    child_urdf = contained(
        snapshot / "data" / "cache" / "record_materialization" / child_id / "model.urdf",
        strict=True,
    )
    source = child_model.read_text(encoding="utf-8")
    source_result = source_predicates(task["task_id"], source)
    urdf_result = urdf_score(task["task_id"], parent_urdf, child_urdf)
    scale_result = None
    if task["edit_class"] == "scale":
        scale_result = scale_measurement(task["task_id"], parent_urdf, child_urdf)
    return {
        "task_id": task["task_id"],
        "record_id": child_id,
        "source_predicates_auxiliary_only": source_result,
        "urdf_semantic_score": urdf_result,
        "scale_measurement": scale_result,
        "compile_reports_present": True,
        "target_fulfilled": urdf_result["target"]["passed"],
        "anchor_preserved": urdf_result["anchor"]["passed"],
        "non_target_preserved": urdf_result["non_target"]["passed"],
        "structural_locality": urdf_result["structural_locality"]["passed"],
        "geometry_locality": urdf_result["geometry_locality"]["passed"],
        "scale_passed": None if scale_result is None else scale_result["passed"],
        "passed": (
            urdf_result["passed"]
            and (scale_result is None or scale_result["passed"])
        ),
    }


def rendered_fork_prompt(task: dict[str, Any]) -> str:
    suffix = contained(ARTICRAFT_ROOT / "picture_expansion" / "FORK_SUFFIX.txt", strict=True).read_text(
        encoding="utf-8"
    )
    suffix = suffix.replace("<Category>", task["category"]).replace("<Subcategory>", task["subcategory"])
    return task["normalized_edit_instruction"].rstrip() + "\n\n" + suffix.strip() + "\n"


def budget_environment(snapshot: Path, output: Path, task_id: str) -> dict[str, str]:
    guard_dir = contained(snapshot / "budget_guard", strict=True)
    budget_dir = contained(output / "cny_budget")
    budget_dir.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    existing_pythonpath = env.get("PYTHONPATH", "").strip()
    env["PYTHONPATH"] = str(guard_dir) + (os.pathsep + existing_pythonpath if existing_pythonpath else "")
    env.update(
        {
            "DASHSCOPE_CONTEXT_TOKENS": "1000000",
            "DASHSCOPE_MAX_TOKENS": "65536",
            "DASHSCOPE_OUTPUT_SAFETY_TOKENS": "1024",
            "DASHSCOPE_MAX_ATTEMPTS": "1",
            "ARTICRAFT_CNY_TOTAL_STATE": str(budget_dir / "total.json"),
            "ARTICRAFT_CNY_USAGE_JSONL": str(budget_dir / "usage.jsonl"),
            "ARTICRAFT_BUDGET_TASK_ID": task_id,
            "ARTICRAFT_CNY_SELF_TEST_DIR": str(budget_dir / "mock_self_test"),
        }
    )
    return env


def validate_budget_guard(snapshot: Path, output: Path) -> bool:
    env = budget_environment(snapshot, output, "PREFLIGHT_NO_API")
    command = [
        "uv",
        "run",
        "python",
        "-c",
        (
            "from agent.providers.dashscope import DashScopeLLM; "
            "client=DashScopeLLM(model_id='qwen3.7-max-2026-05-20', dry_run=True); "
            "assert client._articraft_cny_budget_guard == "
            "'table5_v2_20cny_per_record_360cny_total'; "
            "assert client.max_tokens == 65536; "
            "assert client.context_tokens == 1000000; "
            "assert client.max_attempts == 1; "
            "from sitecustomize import run_mock_self_tests; "
            "tests=run_mock_self_tests(); "
            "assert tests and all(tests.values()), tests"
        ),
    ]
    try:
        completed = subprocess.run(
            command,
            cwd=contained(ARTICRAFT_ROOT, strict=True),
            env=env,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=30,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    return completed.returncode == 0


def run_forks(
    manifest: dict[str, Any], snapshot: Path, output: Path, timeout: int
) -> list[dict[str, Any]]:
    results: list[dict[str, Any]] = []
    for task in manifest["tasks"]:
        task_id = task["task_id"]
        child_id = task["fresh_record_id"]
        child_dir = snapshot / "data" / "records" / child_id
        if child_dir.exists():
            results.append({"task_id": task_id, "record_id": child_id, "status": "REFUSED_EXISTS"})
            continue
        command = [
            "uv",
            "run",
            "articraft",
            "fork",
            "--repo-root",
            str(snapshot),
            "--provider",
            PROVIDER,
            "--model",
            MODEL_SNAPSHOT,
            "--thinking-level",
            THINKING_LEVEL,
            "--max-turns",
            str(MAX_TURNS),
            "--record-id",
            child_id,
            "--label",
            f"table5-articraft-native-v2-{task_id.lower()}",
            "--tag",
            "table5-editability-v2",
            "--skip-search-index",
            task["parent_record_id"],
            rendered_fork_prompt(task),
        ]
        fork_result = run_command(
            command,
            cwd=ARTICRAFT_ROOT,
            timeout=timeout,
            log_path=output / "fork_logs" / f"{task_id}.log",
            env=budget_environment(snapshot, output, task_id),
        )
        item: dict[str, Any] = {"task_id": task_id, "record_id": child_id, "fork": fork_result}
        if fork_result["returncode"] == 0:
            item["compile"] = compile_record(snapshot, child_id, timeout, output / "compile_logs_fresh")
            if item["compile"]["returncode"] == 0:
                item["score"] = score_record(task, task["parent_record_id"], child_id, snapshot)
        results.append(item)
        dump_json(output / "fresh_results" / f"{task_id}.json", item)
    reconcile = [
        "uv",
        "run",
        "articraft",
        "data",
        "reconcile",
        "--repo-root",
        str(snapshot),
        "--with-records-index",
        "--skip-search-index",
    ]
    run_command(reconcile, cwd=ARTICRAFT_ROOT, timeout=timeout, log_path=output / "reconcile.log")
    return results


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--expected-manifest-sha256", required=True)
    parser.add_argument("--confirm-model-snapshot", default=None)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--record-timeout-seconds", type=int, default=RECORD_TIMEOUT_SECONDS)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    manifest_path = contained(args.manifest, strict=True)
    output = contained(args.output_dir)
    actual_hash = sha256(manifest_path)
    manifest = load_json(manifest_path)
    errors = validate_manifest(manifest, manifest_path)
    if actual_hash != args.expected_manifest_sha256.lower():
        errors.append("MANIFEST_SHA256_MISMATCH")
    if args.workers < 1 or args.workers > 4:
        errors.append("WORKERS_MUST_BE_1_TO_4")
    output.mkdir(parents=True, exist_ok=True)
    snapshot = output / "repo_snapshot"
    if not errors:
        copy_snapshot(manifest, snapshot, actual_hash)
    compile_results: dict[str, dict[str, Any]] = {}
    gold_scores: list[dict[str, Any]] = []
    if not errors:
        compile_results = compile_cohort(
            manifest, snapshot, args.record_timeout_seconds, args.workers, output
        )
        failed_compiles = [record_id for record_id, result in compile_results.items() if result["returncode"] != 0]
        if failed_compiles:
            errors.extend(f"COMPILE_FAILED:{record_id}" for record_id in failed_compiles)
        else:
            for task in manifest["tasks"]:
                try:
                    gold_scores.append(
                        score_record(
                            task,
                            task["parent_record_id"],
                            task["historical_gold_record_id"],
                            snapshot,
                        )
                    )
                except Exception as exc:
                    errors.append(f"{task['task_id']}:GOLD_SCORER_ERROR:{exc}")
            for score in gold_scores:
                if not score["passed"]:
                    errors.append(f"{score['task_id']}:HISTORICAL_GOLD_FAILED_SCORER")
    provider_available = False
    if not errors:
        try:
            credential_check = subprocess.run(
                [
                    "uv",
                    "run",
                    "python",
                    "-c",
                    (
                        "from pathlib import Path; "
                        "from articraft.config import load_repo_env; "
                        "from agent.providers.factory import validate_provider_credentials; "
                        "load_repo_env(Path.cwd()); "
                        f"validate_provider_credentials({PROVIDER!r})"
                    ),
                ],
                cwd=ARTICRAFT_ROOT,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=30,
                check=False,
            )
            provider_available = credential_check.returncode == 0
        except (OSError, subprocess.TimeoutExpired):
            provider_available = False
    budget_guard_available = False
    if not errors and snapshot.exists():
        budget_guard_available = validate_budget_guard(snapshot, output)
        if not budget_guard_available:
            errors.append("CNY_BUDGET_GUARD_PREFLIGHT_FAILED")
    preflight = {
        "schema_version": 1,
        "protocol_id": "table5_articraft_native_fork_v2",
        "status": "READY_NOT_RUN" if not errors else "BLOCKED_PREFLIGHT",
        "evidence_class": "PREFLIGHT_ONLY",
        "generated_at_utc": now_utc(),
        "network_accessed": False,
        "paid_api_called": False,
        "manifest": relative(manifest_path),
        "manifest_sha256": actual_hash,
        "expected_manifest_sha256": args.expected_manifest_sha256.lower(),
        "snapshot": relative(snapshot) if snapshot.exists() else None,
        "frozen_native_settings": {
            "provider": PROVIDER,
            "model_id": MODEL_SNAPSHOT,
            "thinking_level": THINKING_LEVEL,
            "max_turns": MAX_TURNS,
            "max_cost_usd": None,
            "record_timeout_seconds": args.record_timeout_seconds,
        },
        "pricing_snapshot": FROZEN_PRICING,
        "budget_guard": FROZEN_BUDGET,
        "budget_guard_available": budget_guard_available,
        "provider_available": provider_available,
        "task_count": len(manifest.get("tasks", [])),
        "unique_parent_count": len({task["parent_record_id"] for task in manifest.get("tasks", [])}),
        "compile_results": compile_results,
        "historical_gold_scores": gold_scores,
        "blockers": sorted(set(errors)),
    }
    dump_json(output / "preflight.json", preflight)
    print(json.dumps(preflight, indent=2, ensure_ascii=False))
    if errors:
        return 2
    if not args.execute:
        return 0
    if args.confirm_model_snapshot != MODEL_SNAPSHOT:
        print(f"execution blocked: pass --confirm-model-snapshot {MODEL_SNAPSHOT}", file=sys.stderr)
        return 2
    if not provider_available:
        print("execution blocked: dashscope credentials unavailable", file=sys.stderr)
        return 2
    if not budget_guard_available:
        print("execution blocked: CNY budget guard unavailable", file=sys.stderr)
        return 2
    fresh_results = run_forks(manifest, snapshot, output, args.record_timeout_seconds)
    summary = {
        "schema_version": 1,
        "protocol_id": "table5_articraft_native_fork_v2",
        "generated_at_utc": now_utc(),
        "manifest_sha256": actual_hash,
        "model_snapshot": MODEL_SNAPSHOT,
        "paid_api_called": True,
        "records": fresh_results,
        "passed": sum(bool(item.get("score", {}).get("passed")) for item in fresh_results),
        "failed": sum(not bool(item.get("score", {}).get("passed")) for item in fresh_results),
        "manual_blind_review": "N/A",
    }
    dump_json(output / "run_summary.json", summary)
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0 if summary["failed"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
