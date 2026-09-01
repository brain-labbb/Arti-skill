#!/usr/bin/env python3
"""Freeze strict LAM parents for the shared-editor Table 5 probe.

This command is offline. It uses official released LAM records only, applies
category and source-level precondition gates to all 18 shared tasks, and never
contacts the editor provider.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import pandas as pd


REPO_ROOT = Path(__file__).resolve().parents[2]
WORKSPACE_ROOT = REPO_ROOT.parent.resolve()
EXP_ROOT = REPO_ROOT / "exp"
TASK_SOURCE = EXP_ROOT / "reference/editability_v2/articraft_parent_manifest.json"
PARQUET = REPO_ROOT / ".cache/table6_sources/lam/dataset/articulated_code.parquet"
TABLE6_PREFLIGHT = EXP_ROOT / "runtime/table6_lam/preflight.json"
DEFAULT_OUT = EXP_ROOT / "runtime/nano3d_editability_v2/lam_shared_editor"
PROTOCOL = "nano3d_table5_lam_shared_external_editor_probe_v2"

# Frozen after inspecting release inputs and before any editor outcome/API call.
SELECTED = {
    "A02": "keyboard_017",
    "A03": "professional_grade_remote_control_engineered_000",
    "A05": "compound_pulley_with_multiple_sheaves_001",
    "M01": "drawing_compass_with_adjustable_legs_001",
    "M05": "garlic_press_with_hinged_handles_and_pressing_pl_000",
    "S04": "keyboard_001",
    "S05": "laptop_037",
}
EXPECTED_INSTRUCTION_MARKERS = {
    "A02": ("media_knob", "key layout"),
    "A03": ("flip cover", "button"),
    "A05": ("becket", "sheave"),
    "M01": ("fold-out lower working section", "revolute knee"),
    "M05": ("round perforated basket", "plunger"),
    "S04": ("high-profile keyboard", "chassis"),
    "S05": ("rugged laptop", "lower_shell", "lid_shell"),
}

EXACT_CATEGORIES = {
    "A02": {"keyboard"},
    "A03": {"professional_grade_remote_control_engineered"},
    "A05": {"compound_pulley_with_multiple_sheaves", "pulley_system"},
    "M01": {"drawing_compass_with_adjustable_legs"},
    "M05": {"garlic_press_with_hinged_handles_and_pressing_pl"},
    "S04": {"keyboard"},
    "S05": {"laptop"},
    "S06": {
        "automotive_hydraulic_jack_with_pumping",
        "functional_hydraulic_jack_featuring",
        "hydraulic_jack",
        "manual_hydraulic_floor_jack_with_pump_handle",
        "meticulously_crafted_hydraulic_jack",
        "precision_hydraulic_jack_with",
        "professional_grade_hydraulic_jack_engineered",
        "professional_hydraulic_jack_with_safety",
        "professional_hydraulic_jack_with_safety_valve_and",
        "well_designed_hydraulic_jack_with",
        "manual_hydraulic_floor_jack_with_pump_handle",
    },
    "S03": {"gate_leg_table_has", "gate_leg_table_has_a_fixed_top_center_section_an"},
}

REJECTION_REASONS = {
    "A01": "no exact viable shopping-bucket or shopping-basket release category",
    "A04": "no exact viable clamshell phone parent with lid screens, full keypad presses, flip hinge, and no external antenna",
    "A06": "no exact viable drying-rack release category",
    "M02": "released clamp records do not contain the required fixed wrist-to-tray interface",
    "M03": "no exact viable rolling work-table parent with open lower bay and retained casters",
    "M04": "adjacent bookshelf records do not satisfy the exact shelving-unit lower-open-tray precondition",
    "M06": "no exact viable radio/audio parent with horizontal grille ribs",
    "S01": "no exact viable shopping-basket release category",
    "S02": "no exact viable conical fire-bucket release category",
    "S03": "gate-leg table records lack the required retained locking braces and oval two-leaf precondition",
    "S06": "floor-jack records either lack casters or encode front casters plus rear fixed wheels, opposite the required front-wheel/rear-caster precondition",
}


def contained(path: Path, *, strict: bool = False) -> Path:
    resolved = path.resolve(strict=strict)
    if resolved != WORKSPACE_ROOT and WORKSPACE_ROOT not in resolved.parents:
        raise ValueError(f"path outside authorized workspace: {resolved}")
    return resolved


def sha_text(value: str) -> str:
    return hashlib.sha256(value.encode("utf-8")).hexdigest()


def sha_file(path: Path) -> str:
    return hashlib.sha256(contained(path, strict=True).read_bytes()).hexdigest()


def dump(path: Path, value: Any) -> None:
    path = contained(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, sort_keys=True) + "\n", encoding="utf-8")


def load_tasks() -> tuple[dict[str, Any], list[dict[str, Any]]]:
    payload = json.loads(contained(TASK_SOURCE, strict=True).read_text(encoding="utf-8"))
    tasks = payload.get("tasks", [])
    ids = [task.get("task_id") for task in tasks]
    classes = [task.get("edit_class") for task in tasks]
    if len(tasks) != 18 or len(set(ids)) != 18:
        raise ValueError("shared task manifest must contain exactly 18 unique tasks")
    if {name: classes.count(name) for name in set(classes)} != {"additive": 6, "modified": 6, "scale": 6}:
        raise ValueError("shared task manifest must have a 6/6/6 edit-class distribution")
    scorer_text = json.dumps([task.get("scorer") for task in tasks], ensure_ascii=False).lower()
    if "pre-freeze" in scorer_text:
        raise ValueError("shared task manifest still contains pre-freeze scorer placeholders")
    return payload, tasks


def source_assertions(task_id: str, source: str, articulation_raw: str) -> list[dict[str, Any]]:
    text = source.lower()
    articulation_text = articulation_raw.lower()
    predicates: dict[str, bool]
    if task_id == "A02":
        predicates = {
            "keyboard_chassis_present": "chassis" in text and "key" in text,
            "media_knob_absent": "media_knob" not in text and "knob" not in text,
        }
    elif task_id == "A03":
        predicates = {
            "body_display_buttons_present": "body" in text and "display" in text and "button" in text,
            "flip_cover_absent": "cover" not in text and "hinge" not in text,
        }
    elif task_id == "A05":
        predicates = {
            "sheave_mechanism_present": "sheave" in text and ("mount" in text or "frame" in text or "cheek" in text),
            "becket_absent": "becket" not in text and "rope_anchor_eye" not in text,
        }
    elif task_id == "M01":
        predicates = {
            "two_single_piece_legs_present": "leg_needle" in text and "leg_pencil" in text,
            "two_main_pivots_present": "needle_pivot" in articulation_text and "pencil_pivot" in articulation_text,
            "head_handle_adjustment_present": all(
                token in text for token in ("pivot_handle", "adjustment_assembly", "adjustment_wheel")
            ),
            "tip_roles_present": "needle_point" in text and "pencil_holder" in text,
            "knee_sections_absent": "knee" not in text and "lower_leg" not in text and "upper_leg" not in text,
        }
    elif task_id == "M05":
        predicates = {
            "rectangular_cup_plunger_present": "boxgeometry" in text and "cup" in text and "plunger" in text,
            "press_hinge_present": "hinge" in text and "handle" in text,
            "round_replacement_absent": "round_cup" not in text and "radial_rim" not in text,
        }
    elif task_id == "S04":
        predicates = {
            "keyboard_base_and_layout_present": "body" in text and "keyboardrows" in text and "keygroup" in text,
            "stable_body_geometry_present": "bodygeometry" in text and "new three.boxgeometry" in text,
        }
    elif task_id == "S05":
        predicates = {
            "clamshell_parts_present": all(token in text for token in ("base_shell", "lid_shell", "primary_hinge")),
            "retained_interfaces_present": all(token in text for token in ("keyboard", "trackpad", "screen")),
        }
    elif task_id == "S06":
        predicates = {
            "hydraulic_lift_present": "hydraulic_cylinder" in text and "lifting_arm" in text and "lifting_tray" in text,
            "wheels_and_handle_present": "wheels" in text and "pump_handle" in text,
            "base_dimensions_present": all(token in text for token in ("basewidth", "basedepth", "baseheight")),
        }
    else:
        predicates = {}
    return [{"name": key, "pass": value} for key, value in predicates.items()]


def normalized(value: str) -> str:
    return " ".join(value.strip().split())


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--expected-shared-manifest-sha256")
    args = parser.parse_args()
    out = contained(args.out)
    out.mkdir(parents=True, exist_ok=True)

    shared, tasks = load_tasks()
    shared_sha = sha_file(TASK_SOURCE)
    if args.expected_shared_manifest_sha256 and args.expected_shared_manifest_sha256.lower() != shared_sha:
        raise ValueError(
            f"shared manifest SHA mismatch: {shared_sha} != {args.expected_shared_manifest_sha256.lower()}"
        )
    binding_status = "EXACT_FINAL_MANIFEST" if args.expected_shared_manifest_sha256 else "SEMANTIC_CANDIDATE"
    frame = pd.read_parquet(contained(PARQUET, strict=True))
    viable = frame[frame["tier"] == "viable"]
    release = json.loads(contained(TABLE6_PREFLIGHT, strict=True).read_text(encoding="utf-8"))
    revision = release["official_sources"]["dataset"]["revision"]
    commit = release["official_sources"]["code"]["commit"]
    audit: list[dict[str, Any]] = []
    frozen: list[dict[str, Any]] = []

    for task in tasks:
        task_id = task["task_id"]
        exact = EXACT_CATEGORIES.get(task_id, set())
        candidates = viable[viable["category"].isin(exact)]
        audit_row: dict[str, Any] = {
            "task_id": task_id,
            "edit_class": task["edit_class"],
            "precondition": task["precondition"],
            "exact_release_categories": sorted(exact),
            "exact_viable_candidates": sorted(candidates["object_release_id"].tolist()),
        }
        markers = EXPECTED_INSTRUCTION_MARKERS.get(task_id, ())
        instruction_text = task["normalized_instruction"].lower()
        semantic_gate_passed = bool(markers) and all(marker in instruction_text for marker in markers)
        audit_row["instruction_semantic_markers"] = list(markers)
        audit_row["instruction_semantic_gate_passed"] = semantic_gate_passed
        if task_id not in SELECTED or not semantic_gate_passed:
            reason = REJECTION_REASONS.get(task_id, "no exact viable LAM category parent")
            if task_id in SELECTED and not semantic_gate_passed:
                reason = "current task semantics do not match this frozen LAM parent gate"
            audit_row.update({"status": "REJECTED_PARENT_GATE", "reason": reason})
            audit.append(audit_row)
            continue

        object_id = SELECTED[task_id]
        matches = candidates[candidates["object_release_id"] == object_id]
        if len(matches) != 1:
            raise RuntimeError(f"{task_id}: expected one viable exact-category row for {object_id}, got {len(matches)}")
        source_row = matches.iloc[0]
        source = str(source_row["threejs_code"])
        articulation_raw = str(source_row["articulation_json"])
        hierarchy_raw = str(source_row["links_hierarchy_json"])
        assertions = source_assertions(task_id, source, articulation_raw)
        if not assertions or not all(check["pass"] for check in assertions):
            raise RuntimeError(f"{task_id}: parent source assertions failed: {assertions}")
        audit_row.update(
            {
                "status": "ELIGIBLE",
                "selected_object_release_id": object_id,
                "selection_fixed_before_edit_outcomes": True,
                "source_assertions": assertions,
            }
        )
        audit.append(audit_row)
        frozen.append(
            {
                "task_id": task_id,
                "edit_class": task["edit_class"],
                "shared_category": task["category"],
                "shared_subcategory": task.get("subcategory"),
                "normalized_instruction": normalized(task["normalized_instruction"]),
                "precondition": task["precondition"],
                "roles": task["roles"],
                "scorer": task["scorer"],
                "method": "LAM",
                "probe_kind": "released-source_external-editor",
                "object_release_id": object_id,
                "release_category": str(source_row["category"]),
                "caption": str(source_row["caption"]),
                "tier": str(source_row["tier"]),
                "rel_path": str(source_row["rel_path"]),
                "source_assertions": assertions,
                "threejs_code_sha256": sha_text(source),
                "inline_urdf_sha256": sha_text(str(source_row["urdf"])),
                "articulation_json_sha256": sha_text(articulation_raw),
                "links_hierarchy_json_sha256": sha_text(hierarchy_raw),
                "provenance": {
                    "official_dataset_revision": revision,
                    "official_code_commit": commit,
                    "release_row": object_id,
                },
            }
        )

    payload = {
        "schema_version": 3,
        "protocol_id": PROTOCOL,
        "cohort_role": "strict matched released-source external-editor probe candidate; excluded from the common 18-item main table",
        "shared_manifest_binding_status": binding_status,
        "main_table_eligible": False,
        "selection_fixed_before_edit_outcomes": True,
        "generated_code_executed": False,
        "api_called": False,
        "shared_task_manifest": str(TASK_SOURCE.relative_to(WORKSPACE_ROOT)),
        "shared_task_manifest_sha256": shared_sha,
        "shared_task_protocol_id": shared["protocol_id"],
        "dataset_revision": revision,
        "code_commit": commit,
        "items": frozen,
    }
    manifest_path = out / "frozen_parent_manifest.json"
    dump(manifest_path, payload)
    cohort_sha = sha_file(manifest_path)

    contract = {
        "schema_version": 2,
        "status": (
            "FROZEN_OFFLINE_NO_API" if binding_status == "EXACT_FINAL_MANIFEST"
            else "CANDIDATE_OFFLINE_NO_API_FINAL_MANIFEST_BINDING_REQUIRED"
        ),
        "probe_label": "LAM released-source + shared frozen external-editor probe",
        "comparison_note": "Same provider/model/one-shot budget as naive; this is not a LAM-native edit API.",
        "main_table_eligible": False,
        "manifest_path": str(manifest_path.relative_to(WORKSPACE_ROOT)),
        "manifest_sha256": cohort_sha,
        "shared_task_manifest_sha256": shared_sha,
        "shared_manifest_binding_status": binding_status,
        "provider": "dashscope",
        "model_snapshot": "qwen3.7-max-2026-05-20",
        "context_window_tokens": 1_000_000,
        "thinking_level": "high",
        "max_turns": 1,
        "provider_request_limit": 1,
        "max_output_tokens": 65_536,
        "output_safety_tokens": 1_024,
        "max_cost_usd": None,
        "tools": [],
        "automatic_retries": 0,
        "repair_turns": 0,
        "compile_feedback": False,
        "lam_native_checker_enabled": False,
        "lam_native_repair_enabled": False,
        "source_truncation": False,
        "pricing_cny_per_million_tokens": {"uncached_input": 12.0, "cached_input": 2.4, "output": 36.0},
        "cost_reporting": "actual input/cached/output tokens plus estimated CNY; no USD conversion",
        "system_message": (
            "You are a one-shot source-code editor. Return only one complete replacement JavaScript module, with no markdown fences, "
            "patch, diff, excerpt, ellipsis, placeholder, explanation, or additional prose. Preserve all behavior not required by the edit. "
            "The module must remain self-contained, use only APIs/imports demonstrated by the parent source, and export createScene(). "
            "You have no tools, compiler feedback, continuation, retry, or repair turn."
        ),
        "user_message_template": (
            "ORIGINAL OBJECT PROMPT:\n{original_object_prompt}\n\nEDIT INSTRUCTION:\n{normalized_instruction}\n\n"
            "PARENT THREE.JS SHA256:\n{parent_source_sha256}\n\nPARENT THREE.JS UTF-8 BYTES:\n{parent_source_bytes}\n\n"
            "COMPLETE PARENT THREE.JS SOURCE:\n{parent_source}"
        ),
        "output_contract": "exactly one complete JavaScript module; no text outside module",
        "execution_gate": (
            "final shared manifest binding, --execute, exact cohort manifest SHA, immutable model snapshot, "
            "and paid confirmation are mandatory"
        ),
    }
    dump(out / "editor_contract.json", contract)
    dump(
        out / "category_match_audit.json",
        {
            "protocol_id": PROTOCOL,
            "created_at_utc": datetime.now(timezone.utc).isoformat(),
            "official_release_rows": int(len(frame)),
            "official_release_viable_rows": int(len(viable)),
            "shared_task_manifest_sha256": shared_sha,
            "strict_eligible_count": len(frozen),
            "strict_eligible_task_ids": [item["task_id"] for item in frozen],
            "rejected_count": 18 - len(frozen),
            "rows": audit,
        },
    )
    summary = {
        "protocol_id": PROTOCOL,
        "status": (
            "OFFLINE_COHORT_EXACTLY_BOUND_API_NOT_CALLED"
            if binding_status == "EXACT_FINAL_MANIFEST"
            else "OFFLINE_SEMANTIC_COHORT_CANDIDATE_API_NOT_CALLED"
        ),
        "shared_manifest_binding_status": binding_status,
        "strict_eligible_count": len(frozen),
        "strict_eligible_task_ids": [item["task_id"] for item in frozen],
        "manifest": str(manifest_path.relative_to(WORKSPACE_ROOT)),
        "manifest_sha256": cohort_sha,
        "shared_task_manifest_sha256": shared_sha,
        "generated_code_executed": False,
        "api_called": False,
        "main_table_eligible": False,
    }
    dump(out / "prepare_summary.json", summary)
    report_rows = []
    for row in audit:
        if row["status"] == "ELIGIBLE":
            detail = row["selected_object_release_id"]
        else:
            detail = row["reason"]
        report_rows.append(f"| {row['task_id']} | {row['status']} | {detail} |")
    report = f"""# LAM shared-editor semantic cohort candidate

Status: **{summary['status']}**

This is a strict official released-source parent cohort for an external editor,
not a LAM-native edit API and not an 18-item main-table result. Current binding
is `{binding_status}`; final paid execution is forbidden until the experiment
owner supplies the final shared-manifest SHA.

| Evidence | Value |
|---|---:|
| Shared task manifest SHA | `{shared_sha}` |
| Candidate cohort manifest SHA | `{cohort_sha}` |
| Strict eligible subset | {len(frozen)}/18 ({', '.join(item['task_id'] for item in frozen)}) |
| Official release rows / viable rows | {len(frame)} / {len(viable)} |
| Official code commit | `{commit}` |
| Official dataset revision | `{revision}` |
| API called | false |
| Main-table eligible | false |

| Task | Gate | Selected parent or rejection reason |
|---|---|---|
{chr(10).join(report_rows)}

Budget protection: immutable model `qwen3.7-max-2026-05-20`, high thinking,
1,000,000 context tokens, 65,536 max output tokens, one provider request and
one turn per item, no tools, retry, checker, compiler feedback, continuation,
or repair. Real execution additionally requires final-manifest binding,
`--execute`, exact cohort SHA, exact model snapshot, and paid confirmation.
"""
    contained(out / "semantic_cohort_report.md").write_text(report, encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0 if frozen else 1


if __name__ == "__main__":
    raise SystemExit(main())
