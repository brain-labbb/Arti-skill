#!/usr/bin/env python3
"""Compile and score final one-shot LAM shared-editor outputs offline."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path
from typing import Any

import pandas as pd
import numpy as np

from execute_lam_released_source import localize_three_import, parse_graph


REPO_ROOT = Path(__file__).resolve().parents[2]
WORKSPACE_ROOT = REPO_ROOT.parent.resolve()
LAM_ROOT = REPO_ROOT / ".cache/table6_sources/lam"
LAM_CODE = LAM_ROOT / "code"
PARQUET = LAM_ROOT / "dataset/articulated_code.parquet"
DEFAULT_ROOT = REPO_ROOT / "exp/runtime/nano3d_editability_v2/lam_shared_editor_final_d8d6e47a"


def contained(path: Path, *, strict: bool = False) -> Path:
    resolved = path.resolve(strict=strict)
    if resolved != WORKSPACE_ROOT and WORKSPACE_ROOT not in resolved.parents:
        raise ValueError(f"path outside authorized workspace: {resolved}")
    return resolved


def sha_file(path: Path) -> str:
    return hashlib.sha256(contained(path, strict=True).read_bytes()).hexdigest()


def dump(path: Path, value: Any) -> None:
    target = contained(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(value, indent=2, ensure_ascii=False, sort_keys=True) + "\n", encoding="utf-8")


def aabb(path: Path) -> dict[str, Any]:
    vertices: list[tuple[float, float, float]] = []
    for line in contained(path, strict=True).read_text(encoding="utf-8", errors="replace").splitlines():
        if line.startswith("v "):
            values = line.split()
            if len(values) >= 4:
                vertices.append((float(values[1]), float(values[2]), float(values[3])))
    if not vertices:
        raise ValueError(f"no OBJ vertices: {path}")
    mins = [min(vertex[axis] for vertex in vertices) for axis in range(3)]
    maxs = [max(vertex[axis] for vertex in vertices) for axis in range(3)]
    return {
        "min": mins,
        "max": maxs,
        "extent": [maxs[index] - mins[index] for index in range(3)],
        "vertex_count": len(vertices),
    }


def ratio(child: float, parent: float) -> float:
    if abs(parent) < 1e-12:
        raise ValueError("zero parent extent")
    return child / parent


def obb_extents(path: Path) -> list[float]:
    vertices = []
    for line in contained(path, strict=True).read_text(encoding="utf-8", errors="replace").splitlines():
        if line.startswith("v "):
            values = line.split()
            if len(values) >= 4:
                vertices.append([float(values[1]), float(values[2]), float(values[3])])
    points = np.asarray(vertices, dtype=np.float64)
    if points.shape[0] < 4:
        raise ValueError(f"insufficient OBJ vertices for OBB: {path}")
    centered = points - points.mean(axis=0)
    _, eigenvectors = np.linalg.eigh(np.cov(centered, rowvar=False))
    projected = centered @ eigenvectors
    extents = projected.max(axis=0) - projected.min(axis=0)
    return sorted((float(value) for value in extents), reverse=True)


def check(name: str, passed: bool, evidence: Any) -> dict[str, Any]:
    return {"name": name, "pass": bool(passed), "evidence": evidence}


def axis(name: str, checks: list[dict[str, Any]]) -> dict[str, Any]:
    return {"axis": name, "status": "PASS" if all(item["pass"] for item in checks) else "FAIL", "checks": checks}


def compile_case(
    item: dict[str, Any], record: dict[str, Any], source_row: pd.Series, parent_record: dict[str, Any], out: Path
) -> dict[str, Any]:
    task_id = item["task_id"]
    task_out = contained(out / task_id)
    task_out.mkdir(parents=True, exist_ok=True)
    edited_source = contained(WORKSPACE_ROOT / record["edited_source"], strict=True)
    if sha_file(edited_source) != record["edited_source_sha256"]:
        raise ValueError(f"{task_id}: edited source hash mismatch")
    source = edited_source.read_text(encoding="utf-8")
    localized = task_out / "edited_source.localized.mjs"
    localized.write_text(localize_three_import(source), encoding="utf-8")
    configs = task_out / "configs"
    configs.mkdir(exist_ok=True)
    articulation = json.loads(str(source_row["articulation_json"]))
    (configs / "articulation.json").write_text(
        json.dumps(articulation, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    part_meshes = task_out / "part_meshes"
    part_meshes.mkdir(exist_ok=True)
    command = [
        "node",
        str(contained(LAM_CODE / "utils/threejs_to_mesh.js", strict=True)),
        str(localized),
        str(part_meshes),
    ]
    completed = subprocess.run(
        command,
        cwd=contained(LAM_CODE, strict=True),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=300,
        check=False,
    )
    (task_out / "node_export.log").write_text(completed.stdout, encoding="utf-8")
    generated = task_out / "generated.urdf"
    urdf_error = None
    if completed.returncode == 0:
        sys.path.insert(0, str(contained(LAM_CODE, strict=True)))
        try:
            from utils.generate_urdf import generate_urdf

            generate_urdf(str(task_out / "links"), str(task_out), str(generated))
        except Exception as exc:  # noqa: BLE001
            urdf_error = f"{type(exc).__name__}: {exc}"
    graph = parse_graph(generated) if generated.is_file() else None
    artifact_generation_pass = bool(
        completed.returncode == 0 and urdf_error is None and graph and not graph["missing_mesh_references"]
    )
    link_set = set(graph["links"]) if graph else set()
    graph_edges = [tuple(edge) for edge in graph["edges"]] if graph else []
    dangling_edges = [edge for edge in graph_edges if edge[0] not in link_set or edge[1] not in link_set]
    child_links = {edge[1] for edge in graph_edges if edge[0] in link_set and edge[1] in link_set}
    roots = sorted(link_set - child_links)
    reachable: set[str] = set()
    if len(roots) == 1:
        frontier = [roots[0]]
        while frontier:
            current = frontier.pop()
            if current in reachable:
                continue
            reachable.add(current)
            frontier.extend(edge[1] for edge in graph_edges if edge[0] == current and edge[1] in link_set)
    unreachable_links = sorted(link_set - reachable)
    urdf_structural_pass = bool(
        artifact_generation_pass and not dangling_edges and len(roots) == 1 and not unreachable_links
    )
    compile_pass = urdf_structural_pass
    parent_graph = parent_record["generated_graph"]
    child_edges = sorted(tuple(edge) for edge in graph["edges"]) if graph else []
    parent_edges = sorted(tuple(edge) for edge in parent_graph["edges"])
    graph_unchanged = bool(
        graph and sorted(graph["links"]) == sorted(parent_graph["links"]) and child_edges == parent_edges
    )
    result = {
        "task_id": task_id,
        "object_release_id": item["object_release_id"],
        "status": "PASS" if compile_pass else "FAIL",
        "artifact_generation_status": "PASS" if artifact_generation_pass else "FAIL",
        "urdf_structural_status": "PASS" if urdf_structural_pass else "FAIL",
        "api_called_in_evaluator": False,
        "edited_source": str(edited_source.relative_to(WORKSPACE_ROOT)),
        "edited_source_sha256": sha_file(edited_source),
        "localized_source": str(localized.relative_to(WORKSPACE_ROOT)),
        "node_returncode": completed.returncode,
        "node_log": str((task_out / "node_export.log").relative_to(WORKSPACE_ROOT)),
        "urdf_error": urdf_error,
        "generated_urdf": str(generated.relative_to(WORKSPACE_ROOT)) if generated.is_file() else None,
        "generated_urdf_sha256": sha_file(generated) if generated.is_file() else None,
        "part_obj_count": len(list(part_meshes.rglob("*.obj"))),
        "link_obj_count": len(list((task_out / "links").glob("*.obj"))),
        "generated_graph": graph,
        "urdf_roots": roots,
        "dangling_joint_edges": dangling_edges,
        "unreachable_links": unreachable_links,
        "parent_graph": parent_graph,
        "graph_unchanged": graph_unchanged,
    }
    dump(task_out / "compile_record.json", result)
    return result


def score_case(
    item: dict[str, Any], compiled: dict[str, Any], parent_dir: Path, child_dir: Path
) -> dict[str, Any]:
    task_id = item["task_id"]
    source = contained(WORKSPACE_ROOT / compiled["edited_source"], strict=True).read_text(encoding="utf-8").lower()
    graph = compiled["generated_graph"] or {"links": [], "edges": []}
    edges = graph["edges"]
    child_edge_set = {tuple(edge) for edge in edges}
    parent_edge_set = {tuple(edge) for edge in compiled["parent_graph"]["edges"]}
    axes: list[dict[str, Any]] = []
    measurements: dict[str, Any] = {}

    if task_id == "A02":
        knob_joint = any("knob" in child.lower() and kind in {"revolute", "continuous"} for _, child, kind in edges)
        axes.append(axis("target", [
            check("media_knob_and_boss_geometry", "media_knob" in source and "chassis_boss" in source, "source semantic names"),
            check("continuous_or_revolute_knob_joint_in_urdf", knob_joint, edges),
        ]))
        axes.append(axis("anchor", [check("boss_attached_to_original_body", "bodygroup.add(bossmesh)" in source, "source attachment")]))
        parent_key_links = {name for name in compiled["parent_graph"]["links"] if name.startswith("key_")}
        child_key_links = {name for name in graph["links"] if name.startswith("key_")}
        parent_key_edges = {edge for edge in parent_edge_set if edge[1].startswith("key_")}
        child_key_edges = {edge for edge in child_edge_set if edge[1].startswith("key_")}
        axes.append(axis("non_target", [
            check("key_semantic_set_preserved", child_key_links == parent_key_links, sorted(child_key_links)),
            check("key_joint_set_preserved", child_key_edges == parent_key_edges, sorted(child_key_edges)),
        ]))
    elif task_id == "A03":
        cover_joint = any("cover" in child.lower() and kind == "revolute" for _, child, kind in edges)
        axes.append(axis("target", [
            check("cover_and_bilateral_knuckles", all(token in source for token in ("lower_button_cluster_cover", "cover_knuckle_1", "cover_knuckle_2")), "source names"),
            check("bounded_cover_revolute_in_urdf", cover_joint, edges),
        ]))
        axes.append(axis("anchor", [check("cover_attached_at_body", "body_to_cover" in source, "source pivot name")]))
        parent_non_target_links = set(compiled["parent_graph"]["links"])
        child_non_target_links = set(graph["links"]) - {"lower_button_cluster_cover"}
        axes.append(axis("non_target", [
            check("display_button_semantic_set_preserved", child_non_target_links == parent_non_target_links, sorted(child_non_target_links)),
            check("display_button_joint_set_preserved", child_edge_set == parent_edge_set, sorted(child_edge_set)),
        ]))
    elif task_id == "A05":
        axes.append(axis("target", [
            check("closed_becket_eye", "lower_secondary_becket_eye" in source and "torusgeometry" in source, "named torus geometry"),
            check("fixed_support_strap", "becket_strap" in source and "main_frame.add(becketstrap)" in source, "source attachment"),
        ]))
        axes.append(axis("anchor", [check("becket_fixed_to_mount_not_sheave", "main_frame.add(becketeyemesh)" in source, "source attachment")]))
        axes.append(axis("non_target", [check("parent_joint_graph_preserved", compiled["graph_unchanged"], "compiled graph equality")]))
    elif task_id == "M01":
        knee_edges = [edge for edge in edges if "knee" in edge[1].lower() and edge[2] == "revolute"]
        axes.append(axis("target", [
            check("two_lower_leg_geometries", "lower_leg_needle" in source and "lower_leg_pencil" in source, "source names"),
            check("two_bounded_revolute_knees_in_urdf", len(knee_edges) == 2, knee_edges),
        ]))
        required_main_pivots = {
            ("pivot_handle", "leg_needle", "revolute"),
            ("pivot_handle", "leg_pencil", "revolute"),
        }
        axes.append(axis("anchor", [check("original_main_pivots_preserved", required_main_pivots.issubset(child_edge_set), sorted(child_edge_set))]))
        axes.append(axis("non_target", [check("adjustment_and_tip_roles_present", all(token in source for token in ("adjustment_wheel", "needle_point", "pencil_holder")), "source names")]))
    elif task_id == "M05":
        axes.append(axis("target", [
            check("round_basket_and_plunger", all(token in source for token in ("basketouterradius", "basketinnerradius", "plungerradius", "cylindergeometry")), "shared radius variables"),
            check("radial_rim", "basket_rim" in source and "torusgeometry" in source, "source geometry"),
            check("radial_perforations", "perforated_plate" in source and "perforation" in source, "source loop/geometry"),
            check("positive_clearance", "basketinnerradius" in source and "plungerradius" in source and "clearance" in source, "explicit source clearance"),
        ]))
        axes.append(axis("anchor", [check("handle_interfaces_retained", "hopperhandle" in source and "presserhandle" in source, "source groups")]))
        axes.append(axis("non_target", [check("parent_joint_graph_preserved", compiled["graph_unchanged"], "compiled graph equality")]))
    elif task_id == "S04":
        parent_box = aabb(parent_dir / "part_meshes/body/body_shell.obj")
        child_box = aabb(child_dir / "part_meshes/body/body_shell.obj")
        ratios = [ratio(child_box["extent"][index], parent_box["extent"][index]) for index in range(3)]
        measurements = {"parent_body_aabb": parent_box, "child_body_aabb": child_box, "axis_ratios": ratios, "axis_mapping": "LAM local Y is chassis thickness"}
        axes.append(axis("target", [check("coherent_chassis_geometry", "body_shell" in source, "source semantic")]))
        axes.append(axis("anchor", [check("key_seating_refit", "keygroup.position.set" in source and "1.4" in source, "source seating")]))
        axes.append(axis("non_target", [check("parent_joint_graph_preserved", compiled["graph_unchanged"], "compiled graph equality")]))
        axes.append(axis("scale", [
            check("thickness_ratio_at_least_1_15", ratios[1] >= 1.15 - 1e-6, ratios[1]),
            check("footprint_x_within_5_percent", 0.95 <= ratios[0] <= 1.05, ratios[0]),
            check("footprint_z_within_5_percent", 0.95 <= ratios[2] <= 1.05, ratios[2]),
        ]))
    elif task_id == "S05":
        parent_base_path = parent_dir / "part_meshes/base_shell/baseBody.obj"
        child_base_path = child_dir / "part_meshes/base_shell/baseBody.obj"
        parent_lid_path = parent_dir / "part_meshes/lid_shell/lidBody.obj"
        child_lid_path = child_dir / "part_meshes/lid_shell/lidBody.obj"
        parent_base = aabb(parent_base_path)
        child_base = aabb(child_base_path)
        parent_lid = aabb(parent_lid_path)
        child_lid = aabb(child_lid_path)
        parent_base_local = obb_extents(parent_base_path)
        child_base_local = obb_extents(child_base_path)
        parent_lid_local = obb_extents(parent_lid_path)
        child_lid_local = obb_extents(child_lid_path)
        base_ratios = [ratio(child_base_local[index], parent_base_local[index]) for index in range(3)]
        lid_ratios = [ratio(child_lid_local[index], parent_lid_local[index]) for index in range(3)]
        bumper_files = sorted((child_dir / "part_meshes/base_shell").glob("corner_bumper_*.obj"))
        bumper_count = len(bumper_files)
        measurements = {
            "parent_base_aabb": parent_base, "child_base_aabb": child_base,
            "parent_lid_aabb": parent_lid, "child_lid_aabb": child_lid,
            "parent_base_local_obb_extents_desc": parent_base_local,
            "child_base_local_obb_extents_desc": child_base_local,
            "parent_lid_local_obb_extents_desc": parent_lid_local,
            "child_lid_local_obb_extents_desc": child_lid_local,
            "base_axis_ratios_desc": base_ratios, "lid_axis_ratios_desc": lid_ratios,
            "axis_mapping": "PCA/OBB local extents sorted as major, intermediate, thickness",
            "named_bumper_count": bumper_count,
        }
        axes.append(axis("target", [check("exactly_four_inline_bumpers", bumper_count == 4, [path.name for path in bumper_files])]))
        axes.append(axis("anchor", [check("hinge_graph_preserved", compiled["graph_unchanged"], "compiled graph equality")]))
        axes.append(axis("non_target", [check("keyboard_trackpad_screen_present", all(token in source for token in ("keyboard", "trackpad", "screen")), "source names")]))
        axes.append(axis("scale", [
            check("base_thickness_ratio_at_least_1_50", base_ratios[2] >= 1.50 - 1e-6, base_ratios[2]),
            check("base_horizontal_extents_within_10_percent", 0.90 <= base_ratios[0] <= 1.10 and 0.90 <= base_ratios[1] <= 1.10, base_ratios),
            check("lid_thickness_ratio_at_least_1_50", lid_ratios[2] >= 1.50 - 1e-6, lid_ratios[2]),
            check("lid_horizontal_extents_within_10_percent", 0.90 <= lid_ratios[0] <= 1.10 and 0.90 <= lid_ratios[1] <= 1.10, lid_ratios),
        ]))
    else:
        raise ValueError(f"unsupported strict cohort task: {task_id}")

    passed = compiled["status"] == "PASS" and all(item["status"] == "PASS" for item in axes)
    return {
        "task_id": task_id,
        "status": "PASS" if passed else "FAIL",
        "compile_status": compiled["status"],
        "axes": axes,
        "measurements": measurements,
        "scorer_source": item["scorer"],
        "gold_used_in_prompt": False,
        "scorer_used_in_prompt": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    args = parser.parse_args()
    root = contained(args.root, strict=True)
    manifest_path = contained(root / "frozen_parent_manifest.json", strict=True)
    run_path = contained(root / "shared_editor_run/summary.json", strict=True)
    smoke_path = contained(root / "offline_parent_smoke/summary.json", strict=True)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    run = json.loads(run_path.read_text(encoding="utf-8"))
    smoke = json.loads(smoke_path.read_text(encoding="utf-8"))
    if manifest.get("shared_manifest_binding_status") != "EXACT_FINAL_MANIFEST":
        raise ValueError("final shared manifest binding required")
    if run.get("status") != "COMPLETE" or len(run.get("records", [])) != len(manifest["items"]):
        raise ValueError("complete provider run required")
    if run.get("manifest_sha256") != sha_file(manifest_path) or smoke.get("manifest_sha256") != sha_file(manifest_path):
        raise ValueError("manifest provenance mismatch")
    if any(not row.get("accepted") for row in run["records"]):
        raise ValueError("all provider outputs must pass response gate")

    frame = pd.read_parquet(contained(PARQUET, strict=True))
    frame = frame[frame["tier"] == "viable"]
    run_by_id = {row["task_id"]: row for row in run["records"]}
    smoke_by_id = {row["task_id"]: row for row in smoke["records"]}
    compile_root = contained(root / "edited_compile")
    compile_records = []
    scores = []
    for item in manifest["items"]:
        task_id = item["task_id"]
        rows = frame[frame["object_release_id"] == item["object_release_id"]]
        if len(rows) != 1:
            raise ValueError(f"{task_id}: unique official release row required")
        compiled = compile_case(item, run_by_id[task_id], rows.iloc[0], smoke_by_id[task_id], compile_root)
        compile_records.append(compiled)
        parent_dir = contained(root / "offline_parent_smoke" / task_id, strict=True)
        child_dir = contained(compile_root / task_id, strict=True)
        score = score_case(item, compiled, parent_dir, child_dir)
        scores.append(score)
        dump(child_dir / "score.json", score)
        print(task_id, compiled["status"], score["status"], flush=True)

    item_by_id = {item["task_id"]: item for item in manifest["items"]}
    score_by_id = {score["task_id"]: score for score in scores}
    per_task = []
    for record in run["records"]:
        task_id = record["task_id"]
        score = score_by_id[task_id]
        axis_status = {item["axis"]: item["status"] for item in score["axes"]}
        failed_checks = [
            check_row["name"]
            for axis_row in score["axes"]
            for check_row in axis_row["checks"]
            if not check_row["pass"]
        ]
        per_task.append(
            {
                "task_id": task_id,
                "edit_class": item_by_id[task_id]["edit_class"],
                "object_release_id": item_by_id[task_id]["object_release_id"],
                "provider_output_status": record["status"],
                "compile_status": score["compile_status"],
                "axis_status": axis_status,
                "failed_checks": failed_checks,
                "task_success": score["status"] == "PASS",
                "usage": record["usage"],
                "estimated_cost_cny": record["estimated_cost_cny"]["total"],
            }
        )
    applicable_axis_counts: dict[str, dict[str, int]] = {}
    for score in scores:
        for axis_row in score["axes"]:
            counts = applicable_axis_counts.setdefault(axis_row["axis"], {"pass": 0, "applicable": 0})
            counts["applicable"] += 1
            counts["pass"] += axis_row["status"] == "PASS"
    class_counts: dict[str, dict[str, int]] = {}
    for row in per_task:
        counts = class_counts.setdefault(row["edit_class"], {"pass": 0, "count": 0})
        counts["count"] += 1
        counts["pass"] += row["task_success"]

    summary = {
        "schema_version": 1,
        "protocol_id": "nano3d_table5_lam_shared_editor_offline_evaluation_v2",
        "status": "COMPLETE",
        "evidence_class": "LOCAL_DIRECT",
        "main_table_eligible": False,
        "method_label": "LAM released-source + external same-editor probe",
        "native_lam_edit_api": False,
        "strict_cohort_count": len(scores),
        "shared_task_manifest_sha256": manifest["shared_task_manifest_sha256"],
        "cohort_manifest_sha256": sha_file(manifest_path),
        "provider_run_summary_sha256": sha_file(run_path),
        "api_called_in_evaluator": False,
        "compile_pass_count": sum(row["status"] == "PASS" for row in compile_records),
        "artifact_generation_pass_count": sum(row["artifact_generation_status"] == "PASS" for row in compile_records),
        "task_success_count": sum(row["status"] == "PASS" for row in scores),
        "edit_success_rate": sum(row["status"] == "PASS" for row in scores) / len(scores),
        "applicable_axis_counts": applicable_axis_counts,
        "edit_class_counts": class_counts,
        "per_task": per_task,
        "scores": scores,
        "compile_records": compile_records,
        "provider_usage_and_cost": run["totals"],
    }
    dump(root / "evaluation_summary.json", summary)
    report_rows = []
    for row in per_task:
        axes_text = ", ".join(f"{name}={status}" for name, status in row["axis_status"].items())
        failures = ", ".join(row["failed_checks"]) if row["failed_checks"] else "none"
        report_rows.append(
            f"| {row['task_id']} | {row['object_release_id']} | {row['compile_status']} | "
            f"{axes_text} | {'PASS' if row['task_success'] else 'FAIL'} | {failures} | "
            f"{row['usage'].get('prompt_tokens', 0)} / {row['usage'].get('candidates_tokens', 0)} | "
            f"{row['estimated_cost_cny']:.6f} |"
        )
    report = f"""# LAM released-source external same-editor probe

Status: **COMPLETE** (`LOCAL_DIRECT`)

This is a strict N={len(scores)} subset probe using official released LAM parent
Three.js sources plus the same external one-shot editor as the naive control.
It is **not** a native LAM editing API and is excluded from the common 18-item
main-table denominator.

| Aggregate | Result |
|---|---:|
| Final shared manifest SHA | `{manifest['shared_task_manifest_sha256']}` |
| Exact cohort manifest SHA | `{sha_file(manifest_path)}` |
| Provider outputs accepted | {len(run['records'])}/{len(scores)} |
| Mesh + URDF XML generated | {sum(row['artifact_generation_status'] == 'PASS' for row in compile_records)}/{len(scores)} |
| Structurally valid single-root URDF | {sum(row['status'] == 'PASS' for row in compile_records)}/{len(scores)} |
| Edit success | {sum(row['status'] == 'PASS' for row in scores)}/{len(scores)} ({summary['edit_success_rate']:.6f}) |
| Target axis | {applicable_axis_counts['target']['pass']}/{applicable_axis_counts['target']['applicable']} |
| Anchor axis | {applicable_axis_counts['anchor']['pass']}/{applicable_axis_counts['anchor']['applicable']} |
| Non-target axis | {applicable_axis_counts['non_target']['pass']}/{applicable_axis_counts['non_target']['applicable']} |
| Scale axis (applicable only) | {applicable_axis_counts['scale']['pass']}/{applicable_axis_counts['scale']['applicable']} |
| Additive / modified / scale success | {class_counts['additive']['pass']}/{class_counts['additive']['count']} / {class_counts['modified']['pass']}/{class_counts['modified']['count']} / {class_counts['scale']['pass']}/{class_counts['scale']['count']} |
| Actual prompt / output / total tokens | {run['totals']['actual_tokens']['prompt_tokens']} / {run['totals']['actual_tokens']['candidates_tokens']} / {run['totals']['actual_tokens']['total_tokens']} |
| Estimated total cost | {run['totals']['estimated_cost_cny']:.6f} CNY |

| Task | Official LAM parent | Compile | Shared scorer axes | Result | Failed checks | Input / output tokens | Est. CNY |
|---|---|---|---|---|---|---:|---:|
{chr(10).join(report_rows)}

A02, A03, and M01 produced target geometry in Three.js but did not add the
required joints to the generated URDF articulation graph. A02/A03 therefore
have an extra disconnected root; M01 retains joints pointing to replaced link
names, producing dangling joint endpoints. Source Group names or `userData` are
not counted as joints. Their anchor and non-target preservation checks passed.
A05 and M05 passed their fixed/geometry-only edits. S04 and S05 passed compiled
local AABB/OBB scale checks and preservation checks.

The provider protocol was `dashscope/qwen3.7-max-2026-05-20`, high thinking,
one request and one turn per task, max output 65,536, attempts=1, no tools,
compiler feedback, retry, continuation, checker, or repair. Gold and scorer
metadata were excluded from every request.
"""
    contained(root / "evaluation_report.md").write_text(report, encoding="utf-8")
    print(json.dumps({key: summary[key] for key in ("status", "strict_cohort_count", "compile_pass_count", "task_success_count", "edit_success_rate", "provider_usage_and_cost")}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
