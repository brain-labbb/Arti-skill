#!/usr/bin/env python3
"""Freeze the shared PhysX/PartNet identities and evaluate the PartNet side."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import shutil
from typing import Any, Iterable
import xml.etree.ElementTree as ET

from hierarchy_extended_metrics import aggregate as aggregate_structure
from hierarchy_extended_metrics import analyze_urdf, topology_consistency
from partnet_hierarchy_correctness import aggregate as aggregate_alignment
from partnet_hierarchy_correctness import evaluate_urdf, load_protocol


EXP_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = EXP_ROOT.parents[1]
DEFAULT_PM_ROOT = WORKSPACE / "PartNet_Mobility/data/dataset"
DEFAULT_PHYSX_ROOT = Path("/mnt/zsn/zsn_workspace/Ctrl-3D-trellis2-controlnet-dev/demo/physical_edit_demo/third_party/physx_mobility/extracted/PhysX_mobility/urdf")
DEFAULT_OUTPUT = EXP_ROOT / "runtime/nano3d_hierarchy_correctness/physx_partnet_paired_partnet_reference"
DEFAULT_PROTOCOL = EXP_ROOT / "reference/physx_partnet_paired_reference_v1.json"
DEFAULT_ONTOLOGY = EXP_ROOT / "reference/partnet_hierarchy_correctness_v1.json"
SALT = "nano3d-table3-physx-partnet-paired-v1"
CATEGORY_MAP = {
    "StorageFurniture": "storage_furniture",
    "Table": "table",
    "Refrigerator": "refrigerator",
    "Dishwasher": "dishwasher",
    "Microwave": "microwave",
}


def contained(path: Path, *, exists: bool = True) -> Path:
    resolved = path.resolve(strict=exists)
    allowed = (WORKSPACE.resolve(strict=True), Path("/mnt/zsn/zsn_workspace").resolve(strict=True))
    if not any(resolved == root or root in resolved.parents for root in allowed):
        raise ValueError(f"path outside authorized roots: {resolved}")
    return resolved


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_jsonl(path: Path, rows: Iterable[dict[str, Any]]) -> None:
    path.write_text("".join(json.dumps(row, sort_keys=True) + "\n" for row in rows), encoding="utf-8")


def parse_xyz(element: ET.Element | None, attribute: str) -> list[float] | None:
    if element is None or attribute not in element.attrib:
        return None
    return [float(value) for value in element.attrib[attribute].split()]


def paired_graph_record(urdf_path: Path, partnet_root: Path) -> dict[str, Any]:
    """Retain raw and fixed-contracted graph facts for cross-dataset comparison."""
    robot = ET.parse(urdf_path).getroot()
    links = sorted(node.attrib["name"] for node in robot.findall("link") if node.attrib.get("name"))
    visual_meshes: dict[str, list[dict[str, Any]]] = {}
    all_mesh_hashes = []
    for link in robot.findall("link"):
        link_name = link.attrib.get("name", "")
        meshes = []
        for mesh in link.findall("visual/geometry/mesh"):
            filename = mesh.attrib.get("filename", "")
            mesh_path = (urdf_path.parent / filename).resolve(strict=False)
            exists = mesh_path.is_file()
            mesh_hash = sha256_file(mesh_path) if exists else None
            meshes.append({"filename": filename, "exists": exists, "sha256": mesh_hash})
            if mesh_hash:
                all_mesh_hashes.append(mesh_hash)
        visual_meshes[link_name] = sorted(meshes, key=lambda row: row["filename"])
    joints = []
    fixed_pairs = []
    for joint in robot.findall("joint"):
        parent_node = joint.find("parent")
        child_node = joint.find("child")
        parent = parent_node.attrib.get("link", "") if parent_node is not None else ""
        child = child_node.attrib.get("link", "") if child_node is not None else ""
        joint_type = joint.attrib.get("type", "")
        record = {
            "name": joint.attrib.get("name", ""),
            "type": joint_type,
            "parent": parent,
            "child": child,
            "origin_xyz": parse_xyz(joint.find("origin"), "xyz"),
            "origin_rpy": parse_xyz(joint.find("origin"), "rpy"),
            "axis_xyz": parse_xyz(joint.find("axis"), "xyz"),
            "limit_lower": float(joint.find("limit").attrib["lower"]) if joint.find("limit") is not None and "lower" in joint.find("limit").attrib else None,
            "limit_upper": float(joint.find("limit").attrib["upper"]) if joint.find("limit") is not None and "upper" in joint.find("limit").attrib else None,
        }
        joints.append(record)
        if joint_type == "fixed":
            fixed_pairs.append((parent, child))
    representative = {link: link for link in links}

    def find(node: str) -> str:
        while representative[node] != node:
            representative[node] = representative[representative[node]]
            node = representative[node]
        return node

    def union(left: str, right: str) -> None:
        left_root, right_root = find(left), find(right)
        if left_root != right_root:
            keep, drop = sorted((left_root, right_root))
            representative[drop] = keep

    for parent, child in fixed_pairs:
        if parent in representative and child in representative:
            union(parent, child)
    components: dict[str, list[str]] = defaultdict(list)
    for link in links:
        components[find(link)].append(link)
    canonical_component = {link: "+".join(sorted(members)) for members in components.values() for link in members}
    contracted_edges = []
    for joint in joints:
        if joint["type"] == "fixed":
            continue
        contracted_edges.append(
            {
                **joint,
                "contracted_parent": canonical_component[joint["parent"]],
                "contracted_child": canonical_component[joint["child"]],
            }
        )
    return {
        "source_urdf": str(urdf_path),
        "source_urdf_sha256": sha256_file(urdf_path),
        "link_names": links,
        "raw_joints": sorted(joints, key=lambda row: (row["parent"], row["child"], row["name"])),
        "fixed_contracted_components": sorted(sorted(members) for members in components.values()),
        "fixed_contracted_mobility_edges": sorted(contracted_edges, key=lambda row: (row["contracted_parent"], row["contracted_child"], row["name"])),
        "visual_meshes_by_link": visual_meshes,
        "visual_mesh_sha256_multiset": sorted(all_mesh_hashes),
        "mesh_root": str(partnet_root),
    }


def rank_payload(category: str, dataset_id: str) -> str:
    return "\n".join((SALT, category, dataset_id))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--partnet-root", type=Path, default=DEFAULT_PM_ROOT)
    parser.add_argument("--physx-identity-root", type=Path, default=DEFAULT_PHYSX_ROOT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--ontology-protocol", type=Path, default=DEFAULT_ONTOLOGY)
    parser.add_argument("--selection", type=Path)
    args = parser.parse_args()
    pm_root = contained(args.partnet_root)
    physx_root = contained(args.physx_identity_root)
    output = contained(args.output, exists=False)
    protocol_path = contained(args.protocol)
    ontology_path = contained(args.ontology_protocol)
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"refusing to overwrite: {output}")
    output.mkdir(parents=True, exist_ok=True)
    selected_root = output / "selected_files"
    selected_root.mkdir()
    protocol = json.loads(protocol_path.read_text(encoding="utf-8"))
    ontology = load_protocol(ontology_path)
    if protocol["selection"]["rank_payload_salt"] != SALT:
        raise ValueError("paired protocol salt mismatch")
    physx_ids = {
        path.name.removesuffix("_collision.urdf").removesuffix(".urdf")
        for path in physx_root.glob("*.urdf")
    }
    candidates: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for dataset_id in sorted(physx_ids):
        meta_path = pm_root / dataset_id / "meta.json"
        if not meta_path.is_file():
            continue
        metadata = json.loads(meta_path.read_text(encoding="utf-8"))
        raw_category = str(metadata.get("model_cat", ""))
        if raw_category not in CATEGORY_MAP:
            continue
        category = CATEGORY_MAP[raw_category]
        payload = rank_payload(category, dataset_id)
        candidates[category].append(
            {
                "category": category,
                "raw_category": raw_category,
                "dataset_id": dataset_id,
                "rank_payload": payload,
                "selection_hash": sha256_bytes(payload.encode("utf-8")),
            }
        )
    candidate_counts = {category: len(candidates[category]) for category in sorted(CATEGORY_MAP.values())}
    if args.selection:
        selection = json.loads(contained(args.selection).read_text(encoding="utf-8"))
        frozen = list(selection["records"])
    else:
        frozen = []
        for category in sorted(CATEGORY_MAP.values()):
            ranked = sorted(candidates[category], key=lambda row: (row["selection_hash"], row["dataset_id"]))
            frozen.extend({**row, "selection_rank": rank} for rank, row in enumerate(ranked[:6], 1))
        selection = {
            "protocol_id": protocol["protocol_id"],
            "candidate_counts": candidate_counts,
            "selection_rule": protocol["selection"],
            "records": frozen,
        }
    frozen.sort(key=lambda row: (row["category"], int(row["selection_rank"])))
    candidate_index = {(row["category"], row["dataset_id"]): row for rows in candidates.values() for row in rows}
    if len(frozen) != 30 or Counter(row["category"] for row in frozen) != Counter({c: 6 for c in CATEGORY_MAP.values()}):
        raise ValueError("paired selection is not five categories x six")
    for row in frozen:
        expected = candidate_index.get((row["category"], row["dataset_id"]))
        if expected is None or expected["selection_hash"] != row["selection_hash"]:
            raise ValueError(f"paired selection record mismatch: {row}")
    selection_path = output / "paired_selection.json"
    write_json(selection_path, selection)
    shutil.copyfile(protocol_path, output / "paired_protocol_snapshot.json")
    shutil.copyfile(ontology_path, output / "ontology_protocol_snapshot.json")

    manifest = []
    structures = []
    raw_records = []
    semantics_records = []
    paired_graph_records = []
    for selected in frozen:
        source = pm_root / selected["dataset_id"]
        destination = selected_root / selected["category"] / selected["dataset_id"]
        destination.mkdir(parents=True)
        files = {"meta": source / "meta.json", "urdf": source / "mobility.urdf", "semantics": source / "semantics.txt"}
        missing = [key for key, path in files.items() if not path.is_file()]
        row: dict[str, Any] = {**selected, "sample_id": f"{selected['category']}/{selected['dataset_id']}", "available": False, "missing_required_files": missing, "load_error": None}
        try:
            if missing:
                raise FileNotFoundError(f"missing required files: {missing}")
            copied = {}
            for key, path in files.items():
                target = destination / path.name
                shutil.copyfile(path, target)
                copied[key] = {"path": target.relative_to(output).as_posix(), "sha256": sha256_file(target), "size_bytes": target.stat().st_size}
            row["files"] = copied
            row["urdf_path"] = copied["urdf"]["path"]
            row["urdf_sha256"] = copied["urdf"]["sha256"]
            row["available"] = True
        except Exception as exc:
            row["load_error"] = f"{type(exc).__name__}: {exc}"
        manifest.append(row)
        structure = dict(row)
        structure["evaluated"] = False
        if row["available"]:
            try:
                structure.update(analyze_urdf(output / row["urdf_path"]))
                structure["evaluated"] = True
                structure["evaluation_error"] = None
            except Exception as exc:  # noqa: BLE001
                structure["evaluation_error"] = f"{type(exc).__name__}: {exc}"
        structures.append(structure)
        if row["available"]:
            paired_graph_records.append({"sample_id": row["sample_id"], "category": row["category"], "dataset_id": row["dataset_id"], **paired_graph_record(source / "mobility.urdf", pm_root)})
        raw = dict(row)
        semantics = dict(row)
        raw["evaluation_complete"] = False
        semantics["evaluation_complete"] = False
        if row["available"]:
            urdf = output / row["urdf_path"]
            try:
                raw.update(evaluate_urdf(urdf, row["category"], ontology))
                raw["evaluation_complete"] = True
                raw["evaluation_error"] = None
            except Exception as exc:  # noqa: BLE001
                raw["evaluation_error"] = f"{type(exc).__name__}: {exc}"
            try:
                labels = {}
                for line in (output / row["files"]["semantics"]["path"]).read_text(encoding="utf-8").splitlines():
                    if line.strip():
                        link_name, _, role = line.split(maxsplit=2)
                        labels[link_name] = role
                semantics.update(evaluate_urdf(urdf, row["category"], ontology, labels))
                semantics["evaluation_complete"] = True
                semantics["evaluation_error"] = None
            except Exception as exc:  # noqa: BLE001
                semantics["evaluation_error"] = f"{type(exc).__name__}: {exc}"
        raw_records.append(raw)
        semantics_records.append(semantics)
    write_jsonl(output / "manifest.jsonl", manifest)
    write_jsonl(output / "structure_records.jsonl", structures)
    write_jsonl(output / "urdf_name_only_records.jsonl", raw_records)
    write_jsonl(output / "package_semantics_assisted_records.jsonl", semantics_records)
    write_jsonl(output / "paired_graph_records.jsonl", paired_graph_records)
    evaluated = [row for row in structures if row.get("evaluated")]
    structure = aggregate_structure(evaluated, requested_count=30)
    per_category_topology = []
    for category in sorted(CATEGORY_MAP.values()):
        per_category_topology.append(topology_consistency([row for row in evaluated if row["category"] == category]))
    summary = {
        "protocol_id": protocol["protocol_id"],
        "side": "PartNet-Mobility",
        "role": "paired same-object reference; not a generated-method ranking",
        "requested_count": 30,
        "candidate_counts": candidate_counts,
        "selection_sha256": sha256_file(selection_path),
        "structure": structure,
        "category_macro_topology_consistency": {key: sum(float(row[key]) for row in per_category_topology) / len(per_category_topology) for key in ("unique_signature_rate", "mode_rate", "pairwise_exact_rate", "normalized_entropy")},
        "urdf_name_only_sensitivity": aggregate_alignment(raw_records),
        "package_semantics_assisted_calibration": aggregate_alignment(semantics_records),
        "paired_graph_records_sha256": sha256_file(output / "paired_graph_records.jsonl"),
        "paired_graph_contract": "raw joints preserve parent/child/type/axis/origin/limits and visual mesh hashes; fixed-joint contraction unions fixed-connected links and retains every non-fixed edge",
        "same_ontology_provenance_warning": "PartNet-Mobility semantics and the PartNet-derived ontology share provenance; annotation-assisted scores are not independent gold.",
    }
    write_json(output / "summary.json", summary)
    checks = {
        "manifest_has_30_rows": len(manifest) == 30,
        "candidate_counts_expected": candidate_counts == {"dishwasher": 17, "microwave": 11, "refrigerator": 22, "storage_furniture": 339, "table": 67},
        "selection_hashes_recompute": all(row["selection_hash"] == sha256_bytes(rank_payload(row["category"], row["dataset_id"]).encode("utf-8")) for row in manifest),
        "all_frozen_ids_in_physx_identity_set": all(row["dataset_id"] in physx_ids for row in manifest),
        "all_available": all(row["available"] for row in manifest),
        "paired_graph_has_30_records": len(paired_graph_records) == 30,
        "paired_graph_preserves_all_movable_joints": all(len(row["fixed_contracted_mobility_edges"]) == sum(joint["type"] != "fixed" for joint in row["raw_joints"]) for row in paired_graph_records),
    }
    verification = {"passed": all(checks.values()), "checks": checks, "summary_sha256": sha256_file(output / "summary.json"), "runner_sha256": sha256_file(Path(__file__))}
    write_json(output / "verification.json", verification)
    if not verification["passed"]:
        raise ValueError(f"verification failed: {checks}")
    print(json.dumps({"output": str(output), "selection_sha256": summary["selection_sha256"], "valid": structure["valid_tree_count"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
