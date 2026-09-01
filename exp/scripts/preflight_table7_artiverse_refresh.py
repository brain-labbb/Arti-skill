#!/usr/bin/env python3
"""Inventory a locally unpacked Artiverse release without geometry evaluation.

The audit is intentionally read-only with respect to the dataset.  It performs
one data-tree traversal, validates the release manifest aggregates, parses the
small per-model JSON/URDF metadata, and writes reproducible readiness evidence.
It does not load meshes, run collision simulation, or turn reference-field
coverage into method accuracy.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
import os
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET
from typing import Any


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
DEFAULT_DATASET = REPO / "exp/artiverse"
DEFAULT_OUTPUT = REPO / "exp/runtime/table7_artiverse_refresh"
EXPECTED_REVISION = "8c4b120418e7cbdf9ac4c9580c5dbfdbf128a248"
EXPECTED_CODE_COMMIT = "44f3d41d015018e9b4dff2cbf01fd0892fe6b2c5"
TREE_SNAPSHOT_NAME = f"{EXPECTED_REVISION}.json"
IMAGE_SUFFIXES = {".png", ".jpg", ".jpeg", ".webp"}
MESH_SUFFIXES = {".glb", ".gltf", ".obj", ".stl", ".ply", ".off"}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def read_checkout_commit(git_dir: Path) -> str:
    head = (git_dir / "HEAD").read_text(encoding="utf-8").strip()
    if not head.startswith("ref: "):
        commit = head
    else:
        ref = head.removeprefix("ref: ").strip()
        if not re.fullmatch(r"refs/[A-Za-z0-9._/-]+", ref) or ".." in Path(ref).parts:
            raise ValueError(f"unsafe git ref: {ref!r}")
        ref_path = git_dir / ref
        if ref_path.is_file():
            commit = ref_path.read_text(encoding="utf-8").strip()
        else:
            packed = (git_dir / "packed-refs").read_text(encoding="utf-8")
            matches = [line.split()[0] for line in packed.splitlines() if line.endswith(f" {ref}")]
            commit = matches[0] if len(matches) == 1 else ""
    if not re.fullmatch(r"[0-9a-f]{40}", commit):
        raise ValueError("could not resolve official code checkout commit")
    return commit


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def write_text(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(value, encoding="utf-8")
    temporary.replace(path)


def rel(path: Path, root: Path) -> str:
    return path.relative_to(root).as_posix()


def counter_dict(counter: Counter[str]) -> dict[str, int]:
    return dict(sorted(counter.items()))


def rate(numerator: int, denominator: int) -> float | None:
    return numerator / denominator if denominator else None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, default=DEFAULT_DATASET)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--allow-incomplete",
        action="store_true",
        help="write a partial audit instead of failing when manifest aggregates differ",
    )
    return parser.parse_args()


def validate_manifest(manifest: dict[str, Any]) -> tuple[set[str], list[dict[str, Any]]]:
    checks: list[dict[str, Any]] = []

    def check(name: str, passed: bool, detail: str) -> None:
        checks.append({"name": name, "passed": passed, "detail": detail})

    chunks = manifest.get("chunks", [])
    roots = [root for chunk in chunks for root in chunk.get("roots", [])]
    check("manifest_format", manifest.get("format") == "artiverse-data-tar-gz-chunks-v1", str(manifest.get("format")))
    check("chunk_count", manifest.get("chunk_count") == len(chunks), f"declared={manifest.get('chunk_count')} actual={len(chunks)}")
    check("model_count_sum", manifest.get("model_count") == sum(c.get("model_count", 0) for c in chunks), f"declared={manifest.get('model_count')} chunk_sum={sum(c.get('model_count', 0) for c in chunks)}")
    check("file_count_sum", manifest.get("file_count") == sum(c.get("file_count", 0) for c in chunks), f"declared={manifest.get('file_count')} chunk_sum={sum(c.get('file_count', 0) for c in chunks)}")
    check("input_bytes_sum", manifest.get("input_bytes") == sum(c.get("input_bytes", 0) for c in chunks), f"declared={manifest.get('input_bytes')} chunk_sum={sum(c.get('input_bytes', 0) for c in chunks)}")
    check("root_count", len(roots) == manifest.get("model_count"), f"roots={len(roots)} declared={manifest.get('model_count')}")
    check("roots_unique", len(set(roots)) == len(roots), f"unique={len(set(roots))} roots={len(roots)}")
    check("root_shape", all(len(Path(root).parts) == 4 and Path(root).parts[0] == "data" for root in roots), "all roots must be data/category/source/model")
    return set(roots), checks


def new_record(root: str) -> dict[str, Any]:
    _, category, source, model_id = Path(root).parts
    return {
        "root": root,
        "category": category,
        "source": source,
        "model_id": model_id,
        "file_count": 0,
        "bytes": 0,
        "suffix_counts": Counter(),
        "segmented_glb": False,
        "articulation_json": False,
        "material_json": False,
        "mass_furniture_heuristic_json": False,
        "image_count": 0,
        "urdf_count": 0,
        "urdf_paths": [],
        "urdf_mesh_file_count": 0,
        "symlink_count": 0,
    }


def inventory_tree(data_root: Path, expected_roots: set[str]) -> tuple[dict[str, dict[str, Any]], dict[str, Any]]:
    records = {root: new_record(root) for root in sorted(expected_roots)}
    actual_roots: set[str] = set()
    unexpected_files: list[str] = []
    all_suffixes: Counter[str] = Counter()
    total_files = 0
    total_bytes = 0
    symlinks = 0

    for dirpath, dirnames, filenames in os.walk(data_root, followlinks=False):
        dirnames.sort()
        filenames.sort()
        directory = Path(dirpath)
        relative_directory = directory.relative_to(data_root)
        parts = relative_directory.parts
        if len(parts) >= 3:
            root = Path("data", *parts[:3]).as_posix()
            actual_roots.add(root)
        else:
            root = ""

        for filename in filenames:
            path = directory / filename
            if path.is_symlink():
                symlinks += 1
                if root in records:
                    records[root]["symlink_count"] += 1
                continue
            stat = path.stat()
            total_files += 1
            total_bytes += stat.st_size
            suffix = path.suffix.lower() or "[no_suffix]"
            all_suffixes[suffix] += 1
            if root not in records:
                if len(unexpected_files) < 100:
                    unexpected_files.append(rel(path, data_root.parent))
                continue

            record = records[root]
            record["file_count"] += 1
            record["bytes"] += stat.st_size
            record["suffix_counts"][suffix] += 1
            model_relative = path.relative_to(data_root / Path(*parts[:3]))
            expected_prefix = record["model_id"]
            model_rel = model_relative.as_posix()
            if model_rel == f"{expected_prefix}.segmented.glb":
                record["segmented_glb"] = True
            elif model_rel == f"{expected_prefix}.articulations.json":
                record["articulation_json"] = True
            elif model_rel == "material.json":
                record["material_json"] = True
            elif model_rel == "mass_furniture_heuristic.json":
                record["mass_furniture_heuristic_json"] = True
            if model_relative.parts and model_relative.parts[0] == "imgs" and suffix in IMAGE_SUFFIXES:
                record["image_count"] += 1
            if model_relative.parts and model_relative.parts[0] == "urdf_w_collider":
                if suffix == ".urdf":
                    record["urdf_count"] += 1
                    record["urdf_paths"].append(path)
                if suffix in MESH_SUFFIXES:
                    record["urdf_mesh_file_count"] += 1

    stats = {
        "actual_roots": actual_roots,
        "unexpected_files": unexpected_files,
        "file_count": total_files,
        "bytes": total_bytes,
        "symlink_count": symlinks,
        "suffix_counts": counter_dict(all_suffixes),
    }
    return records, stats


def validate_articulation(path: Path) -> dict[str, Any]:
    try:
        value = read_json(path)
        articulations = value.get("articulations")
        if not isinstance(articulations, list):
            raise ValueError("articulations is not a list")
        joint_types = Counter()
        required = Counter()
        for item in articulations:
            if not isinstance(item, dict):
                raise ValueError("articulation item is not an object")
            joint_types[str(item.get("type", "[missing]"))] += 1
            for field in ("pid", "type", "axis", "rangeMin", "rangeMax"):
                if field in item and item[field] is not None:
                    required[field] += 1
        return {
            "valid": True,
            "joint_count": len(articulations),
            "joint_types": counter_dict(joint_types),
            "joint_field_counts": counter_dict(required),
            "error": None,
        }
    except Exception as exc:  # Keep a per-asset failure rather than aborting the cohort.
        return {"valid": False, "joint_count": None, "joint_types": {}, "joint_field_counts": {}, "error": f"{type(exc).__name__}: {exc}"}


def validate_json(path: Path) -> tuple[bool, str | None]:
    try:
        read_json(path)
        return True, None
    except Exception as exc:
        return False, f"{type(exc).__name__}: {exc}"


def validate_urdf(path: Path) -> dict[str, Any]:
    try:
        root = ET.parse(path).getroot()
        if root.tag != "robot":
            raise ValueError(f"root tag is {root.tag!r}, expected 'robot'")
        links = root.findall("link")
        joints = root.findall("joint")
        movable = [joint for joint in joints if joint.get("type") != "fixed"]
        mesh_refs: list[str] = []
        missing_refs: list[str] = []
        absolute_refs: list[str] = []
        for mesh in root.findall(".//mesh"):
            filename = mesh.get("filename")
            if not filename:
                continue
            mesh_refs.append(filename)
            if Path(filename).is_absolute():
                absolute_refs.append(filename)
                continue
            normalized = filename.removeprefix("package://")
            candidate = (path.parent / normalized).resolve()
            try:
                candidate.relative_to(path.parent.resolve())
            except ValueError:
                missing_refs.append(filename)
                continue
            if not candidate.is_file():
                missing_refs.append(filename)
        return {
            "valid_xml": True,
            "link_count": len(links),
            "joint_count": len(joints),
            "movable_joint_count": len(movable),
            "mesh_ref_count": len(mesh_refs),
            "missing_mesh_ref_count": len(missing_refs),
            "absolute_mesh_ref_count": len(absolute_refs),
            "error": None,
        }
    except Exception as exc:
        return {
            "valid_xml": False,
            "link_count": None,
            "joint_count": None,
            "movable_joint_count": None,
            "mesh_ref_count": None,
            "missing_mesh_ref_count": None,
            "absolute_mesh_ref_count": None,
            "error": f"{type(exc).__name__}: {exc}",
        }


def inspect_records(dataset_root: Path, records: dict[str, dict[str, Any]]) -> list[dict[str, Any]]:
    output: list[dict[str, Any]] = []
    for root in sorted(records):
        record = records[root]
        model_path = dataset_root / root
        model_id = record["model_id"]
        articulation_path = model_path / f"{model_id}.articulations.json"
        material_path = model_path / "material.json"
        mass_path = model_path / "mass_furniture_heuristic.json"
        articulation = validate_articulation(articulation_path) if record["articulation_json"] else None
        material_valid, material_error = validate_json(material_path) if record["material_json"] else (False, "missing")
        mass_valid, mass_error = validate_json(mass_path) if record["mass_furniture_heuristic_json"] else (False, "missing")
        urdfs = [validate_urdf(path) for path in record.pop("urdf_paths")]
        record["suffix_counts"] = counter_dict(record["suffix_counts"])
        record["articulation"] = articulation
        record["material_json_valid"] = material_valid
        record["material_json_error"] = material_error
        record["mass_furniture_heuristic_json_valid"] = mass_valid
        record["mass_furniture_heuristic_json_error"] = mass_error
        record["urdf"] = urdfs
        output.append(record)
    return output


def aggregate(records: list[dict[str, Any]]) -> dict[str, Any]:
    n = len(records)
    category_counts = Counter(record["category"] for record in records)
    source_counts = Counter(record["source"] for record in records)
    joint_types: Counter[str] = Counter()
    joint_fields: Counter[str] = Counter()
    total_annotation_joints = 0
    total_urdf_links = 0
    total_urdf_joints = 0
    total_urdf_movable = 0
    total_mesh_refs = 0
    total_missing_refs = 0
    total_absolute_refs = 0
    for record in records:
        articulation = record["articulation"]
        if articulation and articulation["valid"]:
            total_annotation_joints += articulation["joint_count"]
            joint_types.update(articulation["joint_types"])
            joint_fields.update(articulation["joint_field_counts"])
        for urdf in record["urdf"]:
            if urdf["valid_xml"]:
                total_urdf_links += urdf["link_count"]
                total_urdf_joints += urdf["joint_count"]
                total_urdf_movable += urdf["movable_joint_count"]
                total_mesh_refs += urdf["mesh_ref_count"]
                total_missing_refs += urdf["missing_mesh_ref_count"]
                total_absolute_refs += urdf["absolute_mesh_ref_count"]

    def coverage(field: str) -> dict[str, Any]:
        count = sum(bool(record[field]) for record in records)
        return {"count": count, "denominator": n, "rate": rate(count, n)}

    articulation_valid = sum(bool(record["articulation"] and record["articulation"]["valid"]) for record in records)
    urdf_present = sum(record["urdf_count"] > 0 for record in records)
    urdf_valid = sum(record["urdf_count"] > 0 and all(item["valid_xml"] for item in record["urdf"]) for record in records)
    urdf_refs_resolved = sum(
        record["urdf_count"] > 0
        and all(item["valid_xml"] and item["missing_mesh_ref_count"] == 0 and item["absolute_mesh_ref_count"] == 0 for item in record["urdf"])
        for record in records
    )
    return {
        "model_count": n,
        "category_count": len(category_counts),
        "source_count": len(source_counts),
        "models_by_category": counter_dict(category_counts),
        "models_by_source": counter_dict(source_counts),
        "core_coverage": {
            "segmented_glb": coverage("segmented_glb"),
            "articulation_json": coverage("articulation_json"),
            "articulation_json_valid": {"count": articulation_valid, "denominator": n, "rate": rate(articulation_valid, n)},
            "material_json": coverage("material_json"),
            "material_json_valid": {"count": sum(record["material_json_valid"] for record in records), "denominator": n, "rate": rate(sum(record["material_json_valid"] for record in records), n)},
            "mass_furniture_heuristic_json": coverage("mass_furniture_heuristic_json"),
            "mass_furniture_heuristic_json_valid": {"count": sum(record["mass_furniture_heuristic_json_valid"] for record in records), "denominator": n, "rate": rate(sum(record["mass_furniture_heuristic_json_valid"] for record in records), n)},
            "reference_images": {"count": sum(record["image_count"] > 0 for record in records), "denominator": n, "rate": rate(sum(record["image_count"] > 0 for record in records), n), "image_file_count": sum(record["image_count"] for record in records)},
            "collision_urdf": {"count": urdf_present, "denominator": n, "rate": rate(urdf_present, n), "urdf_file_count": sum(record["urdf_count"] for record in records)},
            "collision_urdf_xml_valid": {"count": urdf_valid, "denominator": n, "rate": rate(urdf_valid, n)},
            "collision_urdf_all_mesh_refs_relative_and_resolved": {"count": urdf_refs_resolved, "denominator": n, "rate": rate(urdf_refs_resolved, n)},
        },
        "annotation_joint_count": total_annotation_joints,
        "annotation_joint_types": counter_dict(joint_types),
        "annotation_joint_field_counts": counter_dict(joint_fields),
        "urdf_totals": {
            "links": total_urdf_links,
            "joints": total_urdf_joints,
            "movable_joints": total_urdf_movable,
            "mesh_references": total_mesh_refs,
            "missing_mesh_references": total_missing_refs,
            "absolute_mesh_references": total_absolute_refs,
        },
    }


def make_report(summary: dict[str, Any]) -> str:
    release = summary["release"]
    core = summary["inventory"]["core_coverage"]
    readiness = summary["readiness"]

    def frac(name: str) -> str:
        item = core[name]
        return f"{item['count']}/{item['denominator']} ({100 * item['rate']:.2f}%)"

    return f"""# Artiverse local release refresh preflight

Status: **{summary['status']}**. This is a release-readiness audit, not a geometry-quality evaluation.

## Frozen release

- Hugging Face dataset: `3dlg-hcvc/artiverse`
- Dataset revision: `{release['dataset_revision']}`
- Manifest created UTC: `{release['manifest_created_utc']}`
- Manifest cohort: {release['manifest_model_count']:,} models, {release['manifest_file_count']:,} files, {release['manifest_input_bytes']:,} uncompressed bytes
- Coverage: {release['category_count']} categories and {release['source_count']} upstream sources
- Archive payload: {release['archive_count']} tar.gz chunks, {release['archive_bytes']:,} bytes
- Release scope: official README calls this a **pre-release subset** and says remaining models/cleanup are pending.

## Local unpack integrity

- Model roots: {release['local_model_count']:,}/{release['manifest_model_count']:,}
- Files: {release['local_file_count']:,}/{release['manifest_file_count']:,}
- Bytes: {release['local_input_bytes']:,}/{release['manifest_input_bytes']:,}
- Missing manifest roots: {release['missing_root_count']}; extra local roots: {release['extra_root_count']}
- All {summary['self_check']['passed_count']}/{summary['self_check']['check_count']} frozen checks passed.

## Core carrier coverage

| Carrier | Assets / denominator |
|---|---:|
| segmented GLB | {frac('segmented_glb')} |
| articulation JSON present | {frac('articulation_json')} |
| articulation JSON parseable | {frac('articulation_json_valid')} |
| material JSON present | {frac('material_json')} |
| heuristic-mass JSON present | {frac('mass_furniture_heuristic_json')} |
| reference images present | {frac('reference_images')} |
| collider URDF present | {frac('collision_urdf')} |
| collider URDF XML parseable | {frac('collision_urdf_xml_valid')} |
| all URDF mesh refs relative and resolved | {frac('collision_urdf_all_mesh_refs_relative_and_resolved')} |

The release contains {summary['inventory']['annotation_joint_count']:,} articulation records and {summary['inventory']['urdf_totals']['movable_joints']:,} movable URDF joints. These are annotation/package counts, not independent joint accuracy or geometric-validity scores.

## Table impact

- Prior `access-blocked`, `manual-gated but not downloaded`, and local `N=0` statements for Artiverse are stale.
- Artiverse can now enter a frozen real-data reference cohort with local N={release['local_model_count']:,} for schema/package readiness.
- Table 6 prediction-accuracy fields remain N/A for a human-authored reference. Collision-free, axis-on-moving-part, and geometry-valid fields remain N/R until their evaluators are run.
- Table 7/9 production-readiness geometry fields remain N/E or N/R until mesh topology, self-intersection, portability, and physics checks run; field presence must not be substituted for those scores.
- Table 10 should replace `access-blocked local N=0` with `local pre-release reference N={release['local_model_count']:,}; schema/package preflight complete; geometry/simulation metrics not yet evaluated`.

## Readiness boundary

`{readiness['state']}`: {readiness['reason']}
"""


def main() -> int:
    args = parse_args()
    dataset_root = args.dataset_root.resolve()
    output = args.output.resolve()
    manifest_path = dataset_root / "dataset_chunks/manifest.json"
    data_root = dataset_root / "data"
    tree_snapshot = dataset_root / ".cache/huggingface/trees" / TREE_SNAPSHOT_NAME
    unpack_log = dataset_root / "unpack.log"
    code_checkout = REPO / ".cache/table6_sources/artiverse/code"
    for path in (manifest_path, data_root, tree_snapshot, unpack_log):
        if not path.exists():
            raise FileNotFoundError(path)

    manifest = read_json(manifest_path)
    expected_roots, checks = validate_manifest(manifest)
    records_by_root, tree = inventory_tree(data_root, expected_roots)
    records = inspect_records(dataset_root, records_by_root)
    inventory = aggregate(records)
    actual_roots = {f"data/{root}" if not root.startswith("data/") else root for root in tree["actual_roots"]}
    missing_roots = sorted(expected_roots - actual_roots)
    extra_roots = sorted(actual_roots - expected_roots)

    def check(name: str, passed: bool, detail: str) -> None:
        checks.append({"name": name, "passed": passed, "detail": detail})

    chunks = manifest["chunks"]
    archive_bytes = 0
    tree_value = read_json(tree_snapshot)
    remote_files = tree_value.get("files", {})
    for chunk in chunks:
        archive = dataset_root / "dataset_chunks" / chunk["archive"]
        archive_bytes += archive.stat().st_size if archive.is_file() else 0
        remote = remote_files.get(f"dataset_chunks/{chunk['archive']}", {})
        check(f"archive_{chunk['archive']}_exists_and_sized", archive.is_file() and archive.stat().st_size == chunk["archive_bytes"], f"local={archive.stat().st_size if archive.is_file() else None} manifest={chunk['archive_bytes']}")
        check(f"archive_{chunk['archive']}_lfs_hash_bound", remote.get("lfs_sha256") == chunk["sha256"], f"tree={remote.get('lfs_sha256')} manifest={chunk['sha256']}")
    check("dataset_revision", tree_snapshot.stem == EXPECTED_REVISION, tree_snapshot.stem)
    code_commit = read_checkout_commit(code_checkout / ".git")
    check("official_code_commit", code_commit == EXPECTED_CODE_COMMIT, code_commit)
    check("local_model_roots", not missing_roots and not extra_roots, f"actual={len(actual_roots)} expected={len(expected_roots)} missing={len(missing_roots)} extra={len(extra_roots)}")
    check("local_file_count", tree["file_count"] == manifest["file_count"], f"actual={tree['file_count']} expected={manifest['file_count']}")
    check("local_input_bytes", tree["bytes"] == manifest["input_bytes"], f"actual={tree['bytes']} expected={manifest['input_bytes']}")
    check("no_symlinks", tree["symlink_count"] == 0, f"symlinks={tree['symlink_count']}")
    unpack_text = unpack_log.read_text(encoding="utf-8", errors="replace")
    check("unpack_success_marker", f"[UNPACK] reconstructed {data_root}" in unpack_text, "success marker in unpack.log")

    failed = [item for item in checks if not item["passed"]]
    complete = not failed
    if failed and not args.allow_incomplete:
        details = "; ".join(f"{item['name']}: {item['detail']}" for item in failed)
        raise RuntimeError(f"incomplete or inconsistent Artiverse extraction: {details}")

    self_check = {
        "status": "PASS" if complete else "FAIL",
        "check_count": len(checks),
        "passed_count": len(checks) - len(failed),
        "failed_count": len(failed),
        "checks": checks,
    }
    release = {
        "dataset_id": "3dlg-hcvc/artiverse",
        "dataset_revision": EXPECTED_REVISION,
        "release_kind": "pre-release subset",
        "manifest_created_utc": manifest["created_utc"],
        "manifest_sha256": sha256_file(manifest_path),
        "manifest_model_count": manifest["model_count"],
        "manifest_file_count": manifest["file_count"],
        "manifest_input_bytes": manifest["input_bytes"],
        "archive_count": manifest["chunk_count"],
        "archive_bytes": archive_bytes,
        "category_count": inventory["category_count"],
        "source_count": inventory["source_count"],
        "local_model_count": len(actual_roots),
        "local_file_count": tree["file_count"],
        "local_input_bytes": tree["bytes"],
        "missing_root_count": len(missing_roots),
        "extra_root_count": len(extra_roots),
        "missing_roots": missing_roots,
        "extra_roots": extra_roots,
    }
    summary = {
        "protocol_id": "nano3d_table7_artiverse_local_release_refresh_v1",
        "status": "READY_FOR_ADAPTER" if complete else "INCOMPLETE_EXTRACTION",
        "role": "human-authored real-data reference",
        "interpretation": "LOCAL_RELEASE_PRESENT_NOT_A_METHOD_ACCURACY_SCORE",
        "release": release,
        "inventory": inventory,
        "file_formats": tree["suffix_counts"],
        "readiness": {
            "state": "READY_FOR_ADAPTER" if complete else "BLOCKED_INCOMPLETE_EXTRACTION",
            "reason": "The local pre-release cohort is frozen and structurally inventoried; matched-category selection and task-specific geometry/simulation adapters have not run." if complete else "The local data tree does not match the frozen release manifest.",
            "geometry_evaluation_run": False,
            "collision_sweep_run": False,
            "prediction_accuracy_applicable": False,
        },
        "provenance": {
            "dataset_source": {
                "kind": "Hugging Face dataset snapshot",
                "dataset_id": "3dlg-hcvc/artiverse",
                "revision": EXPECTED_REVISION,
                "local_dataset_directory_is_git_checkout": False,
            },
            "official_code": {
                "url": "https://github.com/3dlg-hcvc/artiverse",
                "path": rel(code_checkout, REPO),
                "commit": code_commit,
            },
            "readme": {"path": rel(dataset_root / "README.md", REPO), "sha256": sha256_file(dataset_root / "README.md")},
            "manifest": {"path": rel(manifest_path, REPO), "sha256": release["manifest_sha256"]},
            "pack_script": {"path": rel(dataset_root / "pack_dataset_chunks.py", REPO), "sha256": sha256_file(dataset_root / "pack_dataset_chunks.py")},
            "hf_tree_snapshot": {"path": rel(tree_snapshot, REPO), "sha256": sha256_file(tree_snapshot)},
            "unpack_log": {"path": rel(unpack_log, REPO), "sha256": sha256_file(unpack_log)},
            "audit_script": {"path": rel(SCRIPT, REPO), "sha256": sha256_file(SCRIPT)},
            "archive_sha256_recomputed_by_this_audit": False,
            "archive_integrity_basis": "The completed official unpack command used --verify-sha256; this audit additionally binds local sizes and cached LFS hashes to the manifest without rereading 65 GB.",
        },
        "self_check": {key: self_check[key] for key in ("status", "check_count", "passed_count", "failed_count")},
    }

    output.mkdir(parents=True, exist_ok=True)
    write_json(output / "asset_records.json", records)
    write_json(output / "summary.json", summary)
    write_json(output / "self_check.json", self_check)
    write_text(output / "report.md", make_report(summary))
    print(json.dumps({"status": summary["status"], "models": len(records), "files": tree["file_count"], "bytes": tree["bytes"], "self_check": self_check["status"], "output": str(output)}, indent=2))
    return 0 if complete else 2


if __name__ == "__main__":
    raise SystemExit(main())
