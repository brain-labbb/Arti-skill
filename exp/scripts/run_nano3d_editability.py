#!/usr/bin/env python3
"""Run the exp-local reduced Nano3D Editability benchmark.

This harness evaluates six existing procedural templates with three edit types
(numeric parameter, component replacement, and multiplicity/structure edit) on
fixed seeds 0--15.  It never edits the template repository or the selected
source exports.  Generated wrappers, URDFs, meshes, manifests, and reports are
all written below ``exp/runtime/nano3d_editability``.

The deterministic gates are deliberately narrower than the full Nano3D paper
protocol: there is no hidden semantic anchor gold, no independent historical
regression manifest, and no human blind-review panel.  Those fields remain
explicitly unsupported in the generated report.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import dataclasses
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


EXP_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = EXP_ROOT.parent
TEMPLATE_ROOT = PROJECT_ROOT / "arti-template"
DEFAULT_OUT = EXP_ROOT / "runtime" / "nano3d_editability"
SEEDS = list(range(16))

# Each row is one selected, already-existing procedural template.  The stem is
# the function stem registered by arti-template's TEMPLATE_REGISTRY.
EDIT_SPECS: tuple[dict[str, Any], ...] = (
    {
        "slug": "pictureX_0611_Dressing_table",
        "stem": "picturex_0611_dressing_table",
        "parameter": {"field": "body_width_m", "factor": 1.10, "max": 1.299, "target": "carcass width"},
        "component": {"field": "drawer_front", "target": "drawer front hardware"},
        "structure": {"field": "drawer_count", "target": "number of drawers"},
        "target_tokens": ("drawer", "vanity__body"),
    },
    {
        "slug": "pictureX_0611_bi_fold_closet_door_system",
        "stem": "picturex_0611_bi_fold_closet_door_system",
        "parameter": {"field": "opening_width_m", "factor": 1.10, "max": 2.45, "target": "opening width"},
        "component": {"field": "leaf_face", "target": "leaf face"},
        "structure": {"field": "bifold_pair_count", "target": "folding leaf pair count"},
        "target_tokens": ("leaf", "door", "panel", "closet"),
    },
    {
        "slug": "pictureX_0611_juicer_press_with_handle",
        "stem": "pictureX_0611_juicer_press_with_handle",
        "parameter": {"field": "base_width_m", "factor": 1.10, "max": 0.30, "target": "base width"},
        "component": {"field": "grip_style", "target": "operator grip"},
        "structure": {"field": "base_support_count", "target": "base support count"},
        "target_tokens": ("grip", "handle", "actuator", "support", "foot", "mount"),
    },
    {
        "slug": "pictureX_0611_Garden_pruner",
        "stem": "picturex_0611_garden_pruner",
        "parameter": {"field": "tool_length_m", "factor": 1.10, "max": 0.259, "target": "tool length"},
        "component": {"field": "handle_build", "target": "handle build"},
        "structure": {"field": "spring_stage_count", "target": "spring stage count"},
        "target_tokens": ("handle", "spring", "catch", "blade", "jaw"),
    },
    {
        "slug": "pictureX_0611_ergonomic_clamp_with_adjustable_components",
        "stem": "picturex_0611_ergonomic_clamp_with_adjustable_components",
        "parameter": {"field": "clamp_throat_m", "factor": 1.10, "max": 0.059, "target": "clamp throat"},
        "component": {"field": "tray_head", "target": "tray head"},
        "structure": {"field": "clamp_form", "target": "clamp form"},
        "target_tokens": ("tray", "arm", "clamp", "screw", "cam"),
    },
    {
        "slug": "pictureX_0611_Hand_crank_clothes_wringer",
        "stem": "picturex_0611_hand_crank_clothes_wringer",
        "parameter": {"field": "roller_length_m", "factor": 1.10, "max": 0.419, "target": "roller length"},
        "component": {"field": "feed_module", "target": "feed module"},
        "structure": {"field": "roller_count", "target": "roller count"},
        "target_tokens": ("roller", "feed", "apron", "shelf", "crank"),
    },
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def sha256_text(value: str) -> str:
    return hashlib.sha256(value.encode("utf-8")).hexdigest()


def package_digest(case_dir: Path) -> str:
    """Hash the exported artifact, excluding wrappers and result bookkeeping."""
    digest = hashlib.sha256()
    candidates = [case_dir / "model.urdf"]
    assets_dir = case_dir / "assets"
    if assets_dir.exists():
        candidates.extend(assets_dir.rglob("*"))
    for path in sorted(candidates):
        if not path.is_file():
            continue
        digest.update(str(path.relative_to(case_dir)).encode("utf-8"))
        digest.update(path.read_bytes())
    return digest.hexdigest()


def dump_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True, default=str) + "\n",
        encoding="utf-8",
    )


def next_domain_value(module: Any, field: str, current: Any) -> Any:
    for slot in module.TEMPLATE_DOMAIN.slots:
        if slot.name == field:
            values = list(slot.values)
            if current not in values or len(values) < 2:
                raise ValueError(f"no component edit available for {field}={current!r}")
            return values[(values.index(current) + 1) % len(values)]
    raise ValueError(f"domain slot not found: {field}")


def next_multiplicity(module: Any, field: str, current: Any) -> Any:
    for slot in module.TEMPLATE_DOMAIN.slots:
        if slot.name == field:
            values = list(slot.values)
            if current not in values or len(values) < 2:
                raise ValueError(f"no structure edit available for {field}={current!r}")
            index = values.index(current)
            return values[index + 1] if index + 1 < len(values) else values[index - 1]
    raise ValueError(f"domain multiplicity slot not found: {field}")


def build_edit(module: Any, seed: int, kind: str, spec: dict[str, Any]) -> tuple[Any, dict[str, Any]]:
    base = module.config_from_seed(seed)
    if kind == "parameter":
        edit = spec[kind]
        field = edit["field"]
        old = getattr(base, field)
        if old is None:
            raise ValueError(f"seed {seed}: parameter field {field} is None")
        new = min(float(old) * float(edit["factor"]), float(edit["max"]))
        if new <= float(old):
            raise ValueError(f"seed {seed}: no valid positive edit for {field}={old}")
        new = round(new, 8)
        changes = {field: new}
    elif kind == "component":
        edit = spec[kind]
        field = edit["field"]
        old = getattr(base, field)
        new = next_domain_value(module, field, old)
        changes = {field: new}
    elif kind == "structure":
        edit = spec[kind]
        field = edit["field"]
        old = getattr(base, field)
        new = next_multiplicity(module, field, old)
        changes = {field: new}
    else:
        raise ValueError(f"unknown edit kind: {kind}")
    return dataclasses.replace(base, **changes), {
        "field": field,
        "old": old,
        "new": new,
        "changes": changes,
        "target": edit["target"],
    }


def wrapper_text(slug: str, stem: str, seed: int, kind: str, changes: dict[str, Any]) -> str:
    change_literal = repr(changes)
    build_name = f"build_{stem}"
    test_name = f"run_{stem}_tests"
    return f'''from __future__ import annotations

import dataclasses
import importlib
from sdk import AssetContext

_module = importlib.import_module("agent.templates.{slug}")
SEED = {int(seed)!r}
BASE_CONFIG = _module.config_from_seed(SEED)
CONFIG = dataclasses.replace(BASE_CONFIG, **{change_literal})
ASSETS = AssetContext.from_script(__file__)
object_model = _module.{build_name}(CONFIG, assets=ASSETS)

def run_tests():
    return _module.{test_name}(object_model, CONFIG)
'''


def worker_main(wrapper: Path, result_path: Path) -> int:
    # The parent process supplies a hard timeout.  This child only performs one
    # isolated compilation, avoiding shared SDK asset-session state.
    from agent.compiler import compile_urdf_report

    started = time.monotonic()
    payload: dict[str, Any] = {
        "wrapper": str(wrapper),
        "verdict": "fail",
        "error": None,
        "traceback": None,
        "elapsed_s": None,
        "urdf_sha256": None,
        "package_sha256": None,
        "urdf_path": None,
        "signal_status": None,
        "warnings": [],
    }
    try:
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
        urdf_path = wrapper.parent / "model.urdf"
        urdf_path.write_text(urdf, encoding="utf-8")
        signal = getattr(report, "signal_bundle", None)
        payload.update(
            {
                "verdict": "pass",
                "urdf_sha256": sha256_text(urdf),
                "package_sha256": package_digest(wrapper.parent),
                "urdf_path": str(urdf_path),
                "signal_status": getattr(signal, "status", None),
                "warnings": [str(item) for item in (getattr(report, "warnings", None) or [])],
            }
        )
    except BaseException as exc:  # noqa: BLE001
        import traceback

        payload.update(
            {
                "error": f"{type(exc).__name__}: {exc}",
                "traceback": traceback.format_exc()[-6000:],
            }
        )
    payload["elapsed_s"] = time.monotonic() - started
    dump_json(result_path, payload)
    return 0 if payload["verdict"] == "pass" else 1


def run_one(wrapper: Path, result_path: Path, timeout: float) -> dict[str, Any]:
    started = time.monotonic()
    env = os.environ.copy()
    for key in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
        env[key] = "1"
    try:
        proc = subprocess.run(
            [sys.executable, str(Path(__file__).resolve()), "--worker", str(wrapper), str(result_path)],
            cwd=TEMPLATE_ROOT,
            env=env,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
    except subprocess.TimeoutExpired:
        return {
            "verdict": "fail",
            "error": f"compile_timeout({timeout:.0f}s)",
            "elapsed_s": time.monotonic() - started,
            "wrapper": str(wrapper),
        }
    if result_path.is_file():
        payload = json.loads(result_path.read_text(encoding="utf-8"))
        if proc.returncode != 0 and payload.get("verdict") == "pass":
            payload["verdict"] = "fail"
        return payload
    return {
        "verdict": "fail",
        "error": f"worker_crash(rc={proc.returncode}): {proc.stderr[-2000:]}",
        "elapsed_s": time.monotonic() - started,
        "wrapper": str(wrapper),
    }


def parse_urdf(path: Path) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "valid_urdf": False,
        "valid_tree": False,
        "links": [],
        "joints": [],
        "link_count": 0,
        "joint_count": 0,
    }
    try:
        root = ET.parse(path).getroot()
    except Exception as exc:  # noqa: BLE001
        payload["error"] = f"{type(exc).__name__}: {exc}"
        return payload
    links = [node.attrib.get("name", "") for node in root.findall("link")]
    link_set = {name for name in links if name}
    adjacency: dict[str, list[str]] = {name: [] for name in link_set}
    children: set[str] = set()
    joints: list[dict[str, Any]] = []
    for node in root.findall("joint"):
        parent_node = node.find("parent")
        child_node = node.find("child")
        parent = parent_node.attrib.get("link", "") if parent_node is not None else ""
        child = child_node.attrib.get("link", "") if child_node is not None else ""
        joint = {
            "name": node.attrib.get("name", ""),
            "type": node.attrib.get("type", ""),
            "parent": parent,
            "child": child,
        }
        joints.append(joint)
        if parent in link_set and child in link_set:
            adjacency[parent].append(child)
            children.add(child)
    roots = sorted(link_set - children)
    visited: set[str] = set()
    active: set[str] = set()
    cycle = False

    def visit(name: str) -> None:
        nonlocal cycle
        if name in active:
            cycle = True
            return
        if name in visited:
            return
        active.add(name)
        for child in adjacency.get(name, []):
            visit(child)
        active.remove(name)
        visited.add(name)

    for name in link_set:
        visit(name)
    reachable: set[str] = set()
    queue = list(roots)
    while queue:
        name = queue.pop()
        if name in reachable:
            continue
        reachable.add(name)
        queue.extend(adjacency.get(name, []))
    payload.update(
        {
            "valid_urdf": bool(link_set),
            "valid_tree": len(roots) == 1 and not cycle and reachable == link_set and len(joints) == max(0, len(link_set) - 1),
            "links": sorted(link_set),
            "joints": joints,
            "link_count": len(link_set),
            "joint_count": len(joints),
            "roots": roots,
        }
    )
    return payload


def canonical_node(node: ET.Element) -> str:
    # Attribute ordering and whitespace are exporter details; values are kept
    # because the locality metric is intended to observe geometric changes.
    clone = ET.fromstring(ET.tostring(node, encoding="unicode"))
    return ET.tostring(clone, encoding="unicode", short_empty_elements=True)


def node_signatures(path: Path) -> tuple[dict[str, str], dict[str, tuple[str, str, str]]]:
    root = ET.parse(path).getroot()
    links: dict[str, str] = {}
    for link in root.findall("link"):
        name = link.attrib.get("name", "")
        if name:
            links[name] = canonical_node(link)
    joints: dict[str, tuple[str, str, str]] = {}
    for joint in root.findall("joint"):
        name = joint.attrib.get("name", "")
        parent = joint.find("parent")
        child = joint.find("child")
        joints[name] = (
            joint.attrib.get("type", ""),
            parent.attrib.get("link", "") if parent is not None else "",
            child.attrib.get("link", "") if child is not None else "",
        )
    return links, joints


def contains_target(name: str, tokens: tuple[str, ...]) -> bool:
    lowered = name.lower()
    return any(token.lower() in lowered for token in tokens)


def diff_metrics(
    baseline_urdf: Path,
    edited_urdf: Path,
    target_tokens: tuple[str, ...],
) -> dict[str, Any]:
    base_links, base_joints = node_signatures(baseline_urdf)
    edit_links, edit_joints = node_signatures(edited_urdf)
    all_link_names = set(base_links) | set(edit_links)
    changed_links = {name for name in all_link_names if base_links.get(name) != edit_links.get(name)}
    non_target_links = {
        name for name in all_link_names if not contains_target(name, target_tokens)
    }
    changed_non_target = {name for name in changed_links if name in non_target_links}
    all_joint_edges = set(base_joints.values()) | set(edit_joints.values())
    changed_edges = {
        edge for edge in all_joint_edges if edge not in set(base_joints.values()) or edge not in set(edit_joints.values())
    }
    non_target_edges = {
        edge for edge in changed_edges if not contains_target(" ".join(edge), target_tokens)
    }
    geometry_locality = 1.0 - len(changed_non_target) / max(1, len(changed_links))
    structural_locality = 1.0 - len(non_target_edges) / max(1, len(changed_edges))
    stable_non_target_links = {
        name for name in base_links if name in edit_links and not contains_target(name, target_tokens)
    }
    non_target_preserved = all(base_links[name] == edit_links[name] for name in stable_non_target_links)
    return {
        "changed_link_count": len(changed_links),
        "changed_non_target_link_count": len(changed_non_target),
        "changed_joint_edge_count": len(changed_edges),
        "changed_non_target_edge_count": len(non_target_edges),
        "geometry_locality": geometry_locality,
        "structural_locality": structural_locality,
        "non_target_preserved_structural_proxy": non_target_preserved,
    }


def make_case(
    out_root: Path,
    spec: dict[str, Any],
    seed: int,
    kind: str,
) -> tuple[dict[str, Any], Path, Path, dict[str, Any]]:
    module = __import__(f"agent.templates.{spec['slug']}", fromlist=["*"])
    edited_config, edit = build_edit(module, seed, kind, spec)
    base_config = module.config_from_seed(seed)
    case_dir = out_root / "cases" / spec["slug"] / f"seed_{seed}" / kind
    case_dir.mkdir(parents=True, exist_ok=True)
    base_wrapper = out_root / "wrappers" / spec["slug"] / f"seed_{seed}" / "baseline.py"
    wrapper = case_dir / "edited.py"
    base_wrapper.parent.mkdir(parents=True, exist_ok=True)
    if not base_wrapper.exists():
        base_wrapper.write_text(wrapper_text(spec["slug"], spec["stem"], seed, "baseline", {}), encoding="utf-8")
    wrapper.write_text(wrapper_text(spec["slug"], spec["stem"], seed, kind, edit["changes"]), encoding="utf-8")
    base_payload = dataclasses.asdict(base_config)
    edited_payload = dataclasses.asdict(edited_config)
    case_meta = {
        "slug": spec["slug"],
        "seed": seed,
        "edit_kind": kind,
        "base_config": base_payload,
        "edited_config": edited_payload,
        "edit": edit,
        "source_changed": sha256_file(base_wrapper) != sha256_file(wrapper),
        "baseline_wrapper": str(base_wrapper),
        "edited_wrapper": str(wrapper),
    }
    dump_json(case_dir / "case.json", case_meta)
    return case_meta, base_wrapper, wrapper, edit


def load_or_run(
    wrapper: Path,
    result_path: Path,
    timeout: float,
    force: bool,
) -> dict[str, Any]:
    if result_path.is_file() and not force:
        return json.loads(result_path.read_text(encoding="utf-8"))
    return run_one(wrapper, result_path, timeout)


def aggregate(out_root: Path, records: list[dict[str, Any]], specs: list[dict[str, Any]]) -> dict[str, Any]:
    for row in records:
        edited = row["edited_run"]
        edited_path = Path(edited.get("urdf_path") or "")
        base_path = Path(row["baseline_run"].get("urdf_path") or "")
        edited_parse = parse_urdf(edited_path) if edited_path.is_file() else {"valid_tree": False}
        base_parse = parse_urdf(base_path) if base_path.is_file() else {"valid_tree": False}
        row["baseline_parse"] = base_parse
        row["edited_parse"] = edited_parse
        edit = row["edit"]
        actual = row["edited_config"].get(edit["field"])
        row["target_fulfilled"] = actual == edit["new"]
        row["scale_contract_pass"] = (
            row["edit_kind"] == "parameter"
            and isinstance(edit["old"], (int, float))
            and isinstance(actual, (int, float))
            and float(actual) > float(edit["old"])
            and float(actual) <= float(edit["old"]) * float(row["edit"].get("factor", 1.10)) + 1e-8
        )
        row["artifact_saved"] = edited.get("verdict") == "pass" and edited_path.is_file()
        row["output_changed"] = (
            row["artifact_saved"]
            and row["baseline_run"].get("package_sha256") is not None
            and edited.get("package_sha256") != row["baseline_run"].get("package_sha256")
        )
        row["operational_constraint_pass"] = row["artifact_saved"] and edited_parse.get("valid_tree", False)
        row["target_addressable"] = row["target_fulfilled"] and row["output_changed"]
        if row["artifact_saved"] and base_path.is_file() and edited_path.is_file():
            row["diff"] = diff_metrics(base_path, edited_path, tuple(row["target_tokens"]))
        else:
            row["diff"] = {
                "geometry_locality": None,
                "structural_locality": None,
                "non_target_preserved_structural_proxy": False,
            }
        row["final_pass_proxy"] = all(
            (
                row["artifact_saved"],
                edited_parse.get("valid_tree", False),
                row["target_addressable"],
                row["operational_constraint_pass"],
            )
        )

    def mean(field: str, subset: list[dict[str, Any]]) -> float | None:
        values = [row[field] for row in subset if isinstance(row.get(field), (int, float))]
        return sum(values) / len(values) if values else None

    total = len(records)
    parameter_rows = [row for row in records if row["edit_kind"] == "parameter"]
    task_results: list[dict[str, Any]] = []
    for spec in specs:
        for kind in ("parameter", "component", "structure"):
            subset = [row for row in records if row["slug"] == spec["slug"] and row["edit_kind"] == kind]
            task_results.append(
                {
                    "slug": spec["slug"],
                    "edit_kind": kind,
                    "seed_count": len(subset),
                    "all_16_seed_gate": bool(subset) and all(row["final_pass_proxy"] for row in subset),
                    "final_pass_count": sum(row["final_pass_proxy"] for row in subset),
                }
            )
    summary = {
        "schema_version": 1,
        "protocol": "nano3d_editability_reduced_v1",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "template_count": len(specs),
        "edit_tasks": len(task_results),
        "seeds_per_edit": len(SEEDS),
        "edited_cases": total,
        "baseline_cases": len({(row["slug"], row["seed"]) for row in records}),
        "gates": {
            "target_fulfilled": sum(row["target_fulfilled"] for row in records) / total if total else None,
            "target_addressable": sum(row["target_addressable"] for row in records) / total if total else None,
            "artifact_saved": sum(row["artifact_saved"] for row in records) / total if total else None,
            "output_changed": sum(row["output_changed"] for row in records) / total if total else None,
            "valid_tree": sum(row["edited_parse"].get("valid_tree", False) for row in records) / total if total else None,
            "operational_constraint_pass": sum(row["operational_constraint_pass"] for row in records) / total if total else None,
            "final_pass_proxy": sum(row["final_pass_proxy"] for row in records) / total if total else None,
        },
        "target_scale_contract": {
            "parameter_cases": len(parameter_rows),
            "pass": sum(row["scale_contract_pass"] for row in parameter_rows),
            "rate": sum(row["scale_contract_pass"] for row in parameter_rows) / len(parameter_rows) if parameter_rows else None,
        },
        "non_target_preserved_structural_proxy": {
            "pass": sum(row["diff"].get("non_target_preserved_structural_proxy", False) for row in records),
            "total": total,
            "rate": sum(row["diff"].get("non_target_preserved_structural_proxy", False) for row in records) / total if total else None,
        },
        "geometry_locality_mean": mean("geometry_locality", [row["diff"] for row in records]),
        "structural_locality_mean": mean("structural_locality", [row["diff"] for row in records]),
        "task_results": task_results,
        "edit_cost": {
            "wrapper_config_diff": "one dataclass field assignment per edit",
            "mean_wall_time_s": mean("elapsed_s", [row["edited_run"] for row in records]),
            "median_wall_time_s": sorted(
                row["edited_run"]["elapsed_s"] for row in records if isinstance(row["edited_run"].get("elapsed_s"), (int, float))
            )[len([row for row in records if isinstance(row["edited_run"].get("elapsed_s"), (int, float))]) // 2]
            if any(isinstance(row["edited_run"].get("elapsed_s"), (int, float)) for row in records)
            else None,
            "tokens": None,
            "api_cost": None,
        },
        "unsupported": {
            "anchor": "no frozen semantic anchor gold or independent target coordinate contract",
            "regression_preservation": "no independent historical pre-edit regression manifest",
            "human_final_review": "no three-judge blind review packet/verdict",
            "full_nano3d_scope": "6 templates and 18 edit tasks, not the planned 18 templates and 54 edit tasks",
        },
    }
    dump_json(out_root / "records.json", records)
    dump_json(out_root / "summary.json", summary)
    return summary


def write_report(out_root: Path, summary: dict[str, Any]) -> None:
    gates = summary["gates"]
    proxy = summary["non_target_preserved_structural_proxy"]
    tasks = summary["task_results"]
    passed_tasks = sum(bool(row["all_16_seed_gate"]) for row in tasks)
    report = f"""# Nano3D Editability reduced benchmark report

Protocol: `nano3d_editability_reduced_v1`  
Scope: 6 existing procedural templates × 3 edit tasks × 16 seeds = {summary['edited_cases']} edited cases.  
This is a reduced benchmark; it is not the full 18-template / 54-edit Nano3D protocol.

## Deterministic results

| Gate / metric | Result |
|---|---:|
| Target fulfilled (config-level) | {gates['target_fulfilled']:.3f} |
| Edited target addressable + output changed | {gates['target_addressable']:.3f} |
| Artifact saved | {gates['artifact_saved']:.3f} |
| Output changed | {gates['output_changed']:.3f} |
| Valid hierarchy tree | {gates['valid_tree']:.3f} |
| Post-edit operational template QC | {gates['operational_constraint_pass']:.3f} |
| Final deterministic proxy pass | {gates['final_pass_proxy']:.3f} |
| Parameter scale contract ({summary['target_scale_contract']['pass']}/{summary['target_scale_contract']['parameter_cases']}) | {summary['target_scale_contract']['rate']:.3f} |
| Non-target structural preservation proxy ({proxy['pass']}/{proxy['total']}) | {proxy['rate']:.3f} |
| Mean geometry locality proxy | {summary['geometry_locality_mean']:.3f} |
| Mean structural locality proxy | {summary['structural_locality_mean']:.3f} |
| 16-seed task propagation ({passed_tasks}/{len(tasks)}) | {passed_tasks / len(tasks):.3f} |

`Post-edit operational template QC` means the template's own deterministic tests,
URDF export and motion-QC path passed. It is not a hidden-spec constraint pass.
The locality and non-target values are structural/URDF proxies, not semantic or
human-reviewed geometry judgments.

## Coverage and unsupported fields

- Anchor correctness: unsupported; no frozen semantic anchor/coordinate gold.
- Regression preservation: unsupported; no independent historical regression manifest.
- Final human review: unsupported; no three-judge blind-review packet or agreement statistic.
- Edit cost: wrapper config diff and wall time are recorded; token/API cost is unavailable.
- Full benchmark: unsupported; this run covers 6/18 planned templates and 18/54 planned edit tasks.

Per-case evidence is in [`records.json`](records.json); the frozen manifest and
edit definitions are in [`manifest.json`](manifest.json).
"""
    (out_root / "report.md").write_text(report, encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--worker", nargs=2, metavar=("WRAPPER", "RESULT"))
    args = parser.parse_args()
    if args.worker:
        return worker_main(Path(args.worker[0]), Path(args.worker[1]))

    out_root = args.out.resolve()
    out_root.relative_to(EXP_ROOT.resolve())
    out_root.mkdir(parents=True, exist_ok=True)
    manifest = {
        "schema_version": 1,
        "protocol": "nano3d_editability_reduced_v1",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "scope": "6 templates x 3 edit kinds x seeds 0-15",
        "seeds": SEEDS,
        "templates": EDIT_SPECS,
        "source_template_root": str(TEMPLATE_ROOT),
        "source_edit_policy": "read-only; wrappers and outputs are exp-local",
    }
    dump_json(out_root / "manifest.json", manifest)

    records: list[dict[str, Any]] = []
    baseline_jobs: dict[tuple[str, int], tuple[Path, Path]] = {}
    case_jobs: list[tuple[dict[str, Any], Path, Path, Path]] = []
    # Importing modules in the parent is only used to construct immutable edit
    # configs and wrappers.  Compilation itself is isolated in child processes.
    sys.path.insert(0, str(TEMPLATE_ROOT))
    for spec in EDIT_SPECS:
        for seed in SEEDS:
            for kind in ("parameter", "component", "structure"):
                case_meta, base_wrapper, wrapper, edit = make_case(out_root, spec, seed, kind)
                base_result = out_root / "wrappers" / spec["slug"] / f"seed_{seed}" / "baseline_result.json"
                edited_result = wrapper.parent / "edited_result.json"
                baseline_jobs[(spec["slug"], seed)] = (base_wrapper, base_result)
                case_jobs.append((case_meta, base_wrapper, wrapper, edited_result))

    jobs: list[tuple[Path, Path]] = list(baseline_jobs.values())
    jobs.extend((wrapper, result) for _, _, wrapper, result in case_jobs)
    print(f"Running {len(jobs)} isolated compile jobs with {args.workers} workers", flush=True)
    results: dict[tuple[str, str], dict[str, Any]] = {}
    with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, args.workers)) as pool:
        futures = {
            pool.submit(load_or_run, wrapper, result, args.timeout, args.force): (wrapper, result)
            for wrapper, result in jobs
        }
        for index, future in enumerate(concurrent.futures.as_completed(futures), 1):
            wrapper, result = futures[future]
            payload = future.result()
            results[(str(wrapper), str(result))] = payload
            if index % 20 == 0 or index == len(jobs):
                print(f"  completed {index}/{len(jobs)}", flush=True)

    for case_meta, base_wrapper, wrapper, edited_result in case_jobs:
        base_result = baseline_jobs[(case_meta["slug"], case_meta["seed"])][1]
        row = dict(case_meta)
        row["target_tokens"] = next(
            spec["target_tokens"] for spec in EDIT_SPECS if spec["slug"] == case_meta["slug"]
        )
        row["baseline_run"] = results[(str(base_wrapper), str(base_result))]
        row["edited_run"] = results[(str(wrapper), str(edited_result))]
        records.append(row)
    records.sort(key=lambda row: (row["slug"], row["seed"], row["edit_kind"]))
    summary = aggregate(out_root, records, list(EDIT_SPECS))
    write_report(out_root, summary)
    print(json.dumps(summary, indent=2, ensure_ascii=False, sort_keys=True), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
