#!/usr/bin/env python3
"""Run the Nova3D-paper-aligned local-edit protocol on an exp-local 18-item slice.

The paper protocol is one natural edit per asset: 13 additive edits and 5
modified-existing edits.  This harness runs the deterministic front half of
that protocol on 18 selected procedural assets, converts base/edited packages
to GLB, and creates a blinded render packet for the required two-reviewer
adjudication.  It does not invent human labels.

All writes are restricted to ``exp/runtime/nano3d_paper_editability``.  The
template repository and the original selected exports are read-only.
"""

from __future__ import annotations

import argparse
import dataclasses
import hashlib
import json
import math
import os
import random
import re
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
EXP_ROOT = SCRIPT_DIR.parents[1]
PROJECT_ROOT = EXP_ROOT.parent
TEMPLATE_ROOT = PROJECT_ROOT / "arti-template"
DEFAULT_OUT = EXP_ROOT / "runtime" / "nano3d_paper_editability"

sys.path.insert(0, str(TEMPLATE_ROOT))
sys.path.insert(0, str(SCRIPT_DIR))

import run_nano3d_editability as local  # noqa: E402
import run_nano3d_urdf_glb_pilot as glb_pilot  # noqa: E402


# These are the 18 selected source-backed procedural templates from the 33-asset
# cohort.  The seeds are frozen so that multiplicity edits have a valid larger
# value where the task is additive.
TASKS: tuple[dict[str, Any], ...] = (
    {
        "task_id": "E001", "slug": "Bag_Suitcase_Shopping_bucket", "stem": "shopping_bucket", "seed": 3,
        "edit_class": "additive", "field": "secondary_storage", "target_value": "two_compartment_divider",
        "target_tokens": ("divider", "compartment", "caddy"),
        "instruction": "Add an internal divider compartment to the bucket.",
    },
    {
        "task_id": "E002", "slug": "Urban_Environment_bucket2", "stem": "bucket2", "seed": 1,
        "edit_class": "additive", "field": "hoop_count", "target_value": 6,
        "target_tokens": ("hoop", "band"), "instruction": "Add one additional reinforcement hoop around the bucket.",
    },
    {
        "task_id": "E003", "slug": "pictureX_0611_garlic_press", "stem": "pictureX_0611_garlic_press", "seed": 0,
        "edit_class": "additive", "field": "screen_rows", "target_value": 6,
        "target_tokens": ("screen", "row", "hole"), "instruction": "Add one row of holes to the press screen.",
    },
    {
        "task_id": "E004", "slug": "pictureX_0611_guitar_tuning_peg_mechanism", "stem": "pictureX_0611_guitar_tuning_peg_mechanism", "seed": 0,
        "edit_class": "additive", "field": "tuner_count", "target_value": 2,
        "target_tokens": ("tuner", "gear", "post", "worm", "lock"), "instruction": "Add one additional tuning-peg mechanism.",
    },
    {
        "task_id": "E005", "slug": "pictureX_0611_C_shaped_sofa_side_table", "stem": "picturex_0611_c_shaped_sofa_side_table", "seed": 1,
        "edit_class": "additive", "field": "side_table_count", "target_value": 2,
        "target_tokens": ("table", "side"), "instruction": "Add a second side table beside the sofa.",
    },
    {
        "task_id": "E006", "slug": "pictureX_0611_Dressing_table", "stem": "picturex_0611_dressing_table", "seed": 0,
        "edit_class": "additive", "field": "drawer_count", "target_value": 5,
        "target_tokens": ("drawer",), "instruction": "Add one drawer to the dressing table.",
    },
    {
        "task_id": "E007", "slug": "pictureX_0611_Butter_maker", "stem": "picturex_0611_butter_maker", "seed": 2,
        "edit_class": "additive", "field": "paddle_count", "target_value": 3,
        "target_tokens": ("paddle", "dasher", "blade"), "instruction": "Add one additional dasher paddle.",
    },
    {
        "task_id": "E008", "slug": "pictureX_0611_Garden_pruner", "stem": "picturex_0611_garden_pruner", "seed": 2,
        "edit_class": "additive", "field": "spring_stage_count", "target_value": 2,
        "target_tokens": ("spring",), "instruction": "Add a second return-spring stage.",
    },
    {
        "task_id": "E009", "slug": "pictureX_0611_Ice_crream_machine", "stem": "picturex_0611_ice_crream_machine", "seed": 0,
        "edit_class": "additive", "field": "dasher_blade_count", "target_value": 4,
        "target_tokens": ("dasher", "blade", "paddle"), "instruction": "Add one blade to the dasher.",
    },
    {
        "task_id": "E010", "slug": "pictureX_0611_juicer_press_with_handle", "stem": "pictureX_0611_juicer_press_with_handle", "seed": 2,
        "edit_class": "additive", "field": "base_support_count", "target_value": 6,
        "target_tokens": ("support", "foot", "mount"), "instruction": "Add one additional base support.",
    },
    {
        "task_id": "E011", "slug": "pictureX_0611_bi_fold_closet_door_system", "stem": "picturex_0611_bi_fold_closet_door_system", "seed": 0,
        "edit_class": "additive", "field": "bifold_pair_count", "target_value": 3,
        "target_tokens": ("leaf", "door", "fold"), "instruction": "Add one folding door pair to the closet system.",
    },
    {
        "task_id": "E012", "slug": "sailboat_winch_with_pawl_and_handle", "stem": "sailboat_winch_with_pawl_and_handle", "seed": 0,
        "edit_class": "additive", "field": "pawl_count", "target_value": 3,
        "target_tokens": ("pawl", "ratchet"), "instruction": "Add one additional ratchet pawl.",
    },
    {
        "task_id": "E013", "slug": "Household_Laundry_Clothes_drying_rack_Laundry_drying_rack", "stem": "household_laundry_clothes_drying_rack_laundry_drying_rack", "seed": 0,
        "edit_class": "additive", "field": "hanger_count", "target_value": 5,
        "target_tokens": ("hanger", "clip"), "instruction": "Add one additional hanger to the drying rack.",
    },
    {
        "task_id": "E014", "slug": "pictureX_0611_Industrial_rolling_work_table", "stem": "picturex_0611_industrial_rolling_work_table", "seed": 0,
        "edit_class": "modified-existing", "field": "storage_module", "target_value": "drawer_bank",
        "target_tokens": ("drawer", "storage", "module"), "instruction": "Replace the open storage rack with a drawer bank.",
    },
    {
        "task_id": "E015", "slug": "pictureX_0611_Hole_punch", "stem": "picturex_0611_hole_punch", "seed": 0,
        "edit_class": "modified-existing", "field": "punch_station", "target_value": "long_pin",
        "target_tokens": ("punch", "pin", "station"), "instruction": "Replace the collared punch pin with a long punch pin.",
    },
    {
        "task_id": "E016", "slug": "pictureX_0611_ergonomic_clamp_with_adjustable_components", "stem": "picturex_0611_ergonomic_clamp_with_adjustable_components", "seed": 0,
        "edit_class": "modified-existing", "field": "tray_head", "target_value": "sliding_rails",
        "target_tokens": ("tray", "rail"), "instruction": "Replace the fixed tray head with sliding rails.",
    },
    {
        "task_id": "E017", "slug": "pictureX_0611_Hand_crank_clothes_wringer", "stem": "picturex_0611_hand_crank_clothes_wringer", "seed": 0,
        "edit_class": "modified-existing", "field": "feed_module", "target_value": "plain_front_apron",
        "target_tokens": ("feed", "apron", "shelf"), "instruction": "Replace the fold-down feed shelf with a plain front apron.",
    },
    {
        "task_id": "E018", "slug": "pictureX_0611_Folding_table5", "stem": "picturex_0611_folding_table5", "seed": 2,
        "edit_class": "modified-existing", "field": "support_build", "target_value": "telescoping_brace",
        "target_tokens": ("brace", "support", "spine"), "instruction": "Replace the cross-trestle support with a telescoping brace.",
    },
)


def dump_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True, default=str) + "\n", encoding="utf-8")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def task_module(task: dict[str, Any]) -> Any:
    return __import__(f"agent.templates.{task['slug']}", fromlist=["*"])


def make_changes(module: Any, task: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
    base = module.config_from_seed(task["seed"])
    old = getattr(base, task["field"])
    new = task["target_value"]
    if old == new:
        raise ValueError(f"{task['task_id']}: base already has target value {new!r}")
    valid = False
    for slot in getattr(module.TEMPLATE_DOMAIN, "slots", ()):
        if slot.name == task["field"]:
            valid = new in slot.values
            break
    if not valid:
        raise ValueError(f"{task['task_id']}: target value is not in TEMPLATE_DOMAIN: {new!r}")
    return {task["field"]: new}, {
        "field": task["field"], "old": old, "new": new, "edit_class": task["edit_class"],
        "instruction": task["instruction"],
    }


def write_wrapper(path: Path, task: dict[str, Any], changes: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(local.wrapper_text(task["slug"], task["stem"], task["seed"], task["edit_class"], changes), encoding="utf-8")


def edge_set(urdf_path: Path) -> tuple[set[tuple[str, str, str]], set[str]]:
    root = ET.parse(urdf_path).getroot()
    edges = set()
    links = {node.attrib.get("name", "") for node in root.findall("link")}
    for joint in root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is not None and child is not None:
            edges.add((parent.attrib.get("link", ""), child.attrib.get("link", ""), joint.attrib.get("type", "")))
    return edges, links


def non_target_edges(edges: set[tuple[str, str, str]], tokens: tuple[str, ...]) -> set[tuple[str, str, str]]:
    return {edge for edge in edges if not any(token.lower() in f"{edge[0]} {edge[1]}".lower() for token in tokens)}


def target_handle_exists(glb_path: Path, tokens: tuple[str, ...]) -> bool:
    inspection = glb_pilot.inspect_glb(glb_path)
    names = inspection["glb_node_names"]
    return any(any(token.lower() in name.lower() for token in tokens) for name in names)


def render_preview(glb_path: Path, output: Path) -> dict[str, Any]:
    """Create a deterministic, texture-free review preview without Blender.

    The paper uses canonical Blender renders. Blender is unavailable in this
    environment, so these are packet previews only and are not treated as
    paper-equivalent visual evidence.
    """
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
        import trimesh

        scene = trimesh.load(glb_path, force="scene", process=False)
        mesh = scene.dump(concatenate=True)
        vertices = np.asarray(mesh.vertices)
        faces = np.asarray(mesh.faces)
        if len(faces) > 12000:
            indices = np.linspace(0, len(faces) - 1, 12000, dtype=int)
            faces = faces[indices]
        face_vertices = vertices[faces]
        normals = np.asarray(mesh.face_normals)
        light = np.clip(normals @ np.array([0.55, -0.35, 0.76]), -1, 1)
        shade = 0.34 + 0.58 * ((light + 1.0) / 2.0)
        colors = np.column_stack([0.45 * shade, 0.58 * shade, 0.76 * shade, np.full(len(faces), 1.0)])
        fig = plt.figure(figsize=(7.0, 5.5), dpi=140)
        ax = fig.add_subplot(111, projection="3d")
        ax.add_collection3d(Poly3DCollection(face_vertices, facecolors=colors, linewidths=0.0, antialiased=False))
        bounds = np.asarray(mesh.bounds)
        centre = bounds.mean(axis=0)
        radius = max(float((bounds[1] - bounds[0]).max()) / 2.0, 1e-6)
        ax.set_xlim(centre[0] - radius, centre[0] + radius)
        ax.set_ylim(centre[1] - radius, centre[1] + radius)
        ax.set_zlim(centre[2] - radius, centre[2] + radius)
        ax.view_init(elev=22, azim=38)
        ax.set_axis_off()
        fig.tight_layout(pad=0)
        output.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output, transparent=False, facecolor="white")
        plt.close(fig)
        return {"status": "written", "path": str(output), "faces_rendered": len(faces), "renderer": "matplotlib_preview"}
    except Exception as exc:  # noqa: BLE001
        return {"status": "unavailable", "error": f"{type(exc).__name__}: {exc}"}


def make_blinded_packet(out_root: Path, rows: list[dict[str, Any]]) -> dict[str, Any]:
    rng = random.Random(260722738)
    labels = [f"P{idx:03d}" for idx in range(1, len(rows) + 1)]
    rng.shuffle(labels)
    public = []
    private = []
    for label, row in zip(labels, rows):
        public.append({
            "panel_id": label,
            "instruction": row["instruction"],
            "base_preview": row["base_preview"],
            "edited_preview": row["edited_preview"],
            "deterministic_gates_hidden": True,
            "review_fields": ["target_fulfilled", "target_anchor_ok", "target_scale_ok", "non_target_preserved", "locality_pass"],
        })
        private.append({"panel_id": label, "task_id": row["task_id"], "slug": row["slug"], "seed": row["seed"]})
    packet = out_root / "blind_review_packet"
    packet.mkdir(parents=True, exist_ok=True)
    dump_json(packet / "public_packet.json", {"protocol": "nova3d_section9_blinded_review_v1", "panels": public})
    dump_json(packet / "private_key.json", {"protocol": "nova3d_section9_blinded_review_v1", "panels": private})
    template = {
        "reviewer_id": "reviewer_A_or_B",
        "protocol": "nova3d_section9_blinded_review_v1",
        "labels": [
            {"panel_id": p["panel_id"], "target_fulfilled": None, "target_anchor_ok": None, "target_scale_ok": None,
             "non_target_preserved": None, "locality_pass": None, "notes": ""}
            for p in public
        ],
    }
    dump_json(packet / "reviewer_template.json", template)
    return {"public": str(packet / "public_packet.json"), "private_key": str(packet / "private_key.json"), "template": str(packet / "reviewer_template.json"), "panel_count": len(public)}


def run(args: argparse.Namespace) -> int:
    out_root = args.out.resolve()
    out_root.relative_to(EXP_ROOT.resolve())
    out_root.mkdir(parents=True, exist_ok=True)
    dump_json(out_root / "manifest.json", {
        "protocol": "nova3d_section9_paper_aligned_reduced_v1",
        "paper": "Nova3D arXiv:2607.22738v1 Section 9 / Figure 12 / Tables 13-14",
        "paper_scope": "18 generated assets, one natural edit per asset, 13 additive + 5 modified-existing",
        "local_scope": "18 selected existing procedural assets; protocol-aligned edit class and gates, not paper's original generated asset IDs",
        "tasks": list(TASKS),
        "review": "two blinded reviewers plus adjudication required; labels are not generated by this harness",
    })
    rows: list[dict[str, Any]] = []
    for task in TASKS:
        module = task_module(task)
        changes, edit = make_changes(module, task)
        task_root = out_root / "tasks" / task["task_id"]
        base_dir = task_root / "base"
        edit_dir = task_root / "edited"
        base_wrapper = base_dir / "source.py"
        edit_wrapper = edit_dir / "source.py"
        write_wrapper(base_wrapper, task, {})
        write_wrapper(edit_wrapper, task, changes)
        base_result = base_dir / "compile_result.json"
        edit_result = edit_dir / "compile_result.json"
        base_run = local.run_one(base_wrapper, base_result, args.timeout)
        edit_run = local.run_one(edit_wrapper, edit_result, args.timeout)
        row: dict[str, Any] = {
            **task,
            "edit": edit,
            "changes": changes,
            "base_run": base_run,
            "edit_run": edit_run,
            "source_changed": sha256(base_wrapper) != sha256(edit_wrapper),
        }
        base_urdf = Path(base_run.get("urdf_path") or "")
        edit_urdf = Path(edit_run.get("urdf_path") or "")
        row["base_parse"] = local.parse_urdf(base_urdf) if base_urdf.is_file() else {"valid_tree": False}
        row["edit_parse"] = local.parse_urdf(edit_urdf) if edit_urdf.is_file() else {"valid_tree": False}
        base_glb = task_root / "base" / "model.glb"
        edit_glb = task_root / "edited" / "model.glb"
        for package, output in ((base_dir, base_glb), (edit_dir, edit_glb)):
            try:
                glb_pilot.export_package(package, output)
            except Exception as exc:  # noqa: BLE001
                row.setdefault("glb_errors", []).append(f"{package.name}: {type(exc).__name__}: {exc}")
        row["base_glb"] = str(base_glb) if base_glb.is_file() else None
        row["edited_glb"] = str(edit_glb) if edit_glb.is_file() else None
        row["artifact_valid"] = (
            base_run.get("verdict") == "pass" and edit_run.get("verdict") == "pass"
            and base_glb.is_file() and edit_glb.is_file()
        )
        row["target_handle"] = bool(edit_glb.is_file() and target_handle_exists(edit_glb, tuple(task["target_tokens"])))
        row["glb_changed"] = bool(base_glb.is_file() and edit_glb.is_file() and sha256(base_glb) != sha256(edit_glb))
        row["source_glb_changed"] = row["source_changed"] and row["glb_changed"]
        if base_urdf.is_file() and edit_urdf.is_file():
            base_edges, _ = edge_set(base_urdf)
            edit_edges, _ = edge_set(edit_urdf)
            tokens = tuple(task["target_tokens"])
            row["hierarchy_preserved"] = (
                row["base_parse"].get("valid_tree", False)
                and row["edit_parse"].get("valid_tree", False)
                and non_target_edges(base_edges, tokens) == non_target_edges(edit_edges, tokens)
            )
        else:
            row["hierarchy_preserved"] = False
        row["deterministic_gate_pass"] = all(row[key] for key in ("artifact_valid", "target_handle", "source_glb_changed", "hierarchy_preserved"))
        row["base_preview"] = None
        row["edited_preview"] = None
        if base_glb.is_file():
            preview = render_preview(base_glb, out_root / "blind_review_packet" / "previews" / f"{task['task_id']}_base.png")
            row["base_preview"] = preview.get("path")
        if edit_glb.is_file():
            preview = render_preview(edit_glb, out_root / "blind_review_packet" / "previews" / f"{task['task_id']}_edited.png")
            row["edited_preview"] = preview.get("path")
        dump_json(task_root / "record.json", row)
        rows.append(row)
        print(task["task_id"], task["edit_class"], "gate=", row["deterministic_gate_pass"], flush=True)

    packet = make_blinded_packet(out_root, rows)
    summary = {
        "protocol": "nova3d_section9_paper_aligned_reduced_v1",
        "paper_reference": "arXiv:2607.22738v1 Section 9 / Tables 13-14",
        "tasks": len(rows),
        "additive": sum(row["edit_class"] == "additive" for row in rows),
        "modified_existing": sum(row["edit_class"] == "modified-existing" for row in rows),
        "deterministic_gates": {
            "artifact_valid": sum(row["artifact_valid"] for row in rows),
            "target_handle": sum(row["target_handle"] for row in rows),
            "source_glb_changed": sum(row["source_glb_changed"] for row in rows),
            "hierarchy_preserved": sum(row["hierarchy_preserved"] for row in rows),
            "all_gates": sum(row["deterministic_gate_pass"] for row in rows),
        },
        "human_review": {
            "reviewers_required": 2,
            "reviewer_A_submitted": False,
            "reviewer_B_submitted": False,
            "adjudication_submitted": False,
            "final_pass": None,
            "non_target_preserved": None,
            "locality_preserved": None,
            "target_fulfilled_agreement": None,
            "target_anchor_agreement": None,
            "target_scale_agreement": None,
            "overall_agreement": None,
        },
        "packet": packet,
        "limitations": [
            "The 18 local assets are selected procedural assets, not the paper's private generated asset IDs.",
            "GLB conversion uses the exp-local URDF/trimesh converter, not the paper's unreleased Blender converter.",
            "Blender is unavailable; previews are deterministic matplotlib previews and are not paper canonical renders.",
            "Human reviewer labels and adjudication are intentionally left null.",
        ],
    }
    dump_json(out_root / "records.json", rows)
    dump_json(out_root / "summary.json", summary)
    lines = [
        "# Paper-aligned Nova3D local editability report",
        "",
        "Protocol: `nova3d_section9_paper_aligned_reduced_v1`.",
        "",
        "The paper protocol is 18 generated assets, one natural edit per asset, 13 additive and 5 modified-existing; deterministic gates precede two blinded human reviews and adjudication.",
        "",
        "## Deterministic gates",
        "",
        "| Gate | Pass | N |",
        "|---|---:|---:|",
    ]
    for key, value in summary["deterministic_gates"].items():
        lines.append(f"| {key} | {value}/{len(rows)} | {len(rows)} |")
    lines += [
        "",
        "## Human review status",
        "",
        "All human fields remain `N/A`/null until two independent reviewers score the blinded packet and an adjudicator resolves disagreements. The harness does not substitute automated proxies for those labels.",
        "",
        f"Review packet: `{packet['public']}`; private key: `{packet['private_key']}`; reviewer template: `{packet['template']}`.",
        "",
        "This is protocol-aligned but not an exact reproduction of the paper's private generated asset IDs, Blender render backend, or released reviewer panels.",
    ]
    (out_root / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False), flush=True)
    return 0 if summary["deterministic_gates"]["all_gates"] == len(rows) else 1


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--timeout", type=float, default=180.0)
    args = parser.parse_args()
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
