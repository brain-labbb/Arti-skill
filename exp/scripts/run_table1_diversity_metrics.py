#!/usr/bin/env python3
"""Run the Table 1 joint-variance and calibrated near-duplicate pipelines.

The near-duplicate workflow is deliberately gated.  ``prepare`` extracts frozen
q=0 visual point clouds, builds graph/category-constrained retrieval candidates,
and creates a human-label packet.  ``calibrate`` must then produce a passing
threshold receipt before ``score`` will calculate any near-duplicate rate.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from fractions import Fraction
import hashlib
import heapq
import importlib.util
import json
import math
import multiprocessing
import os
from pathlib import Path
import platform
import sqlite3
import struct
import sys
import tempfile
from typing import Any, Iterable, Iterator, Mapping, Sequence
from xml.etree import ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw
import scipy
from scipy.spatial import cKDTree
import trimesh


PROJECT_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = Path(__file__).resolve()
RUNTIME_ROOT = PROJECT_ROOT / "exp/runtime"
DEFAULT_OUTPUT = RUNTIME_ROOT / "table1_var_neardup_diagnostic_20260827"
DEFAULT_SCRATCH = Path("/tmp/table1_var_neardup_diagnostic_20260827")
PVA_SOURCE_ROOT = Path("/mnt/zsn/data/particulate/datasets/PV-A/extracted")
DEFAULT_PVA_MIRROR = Path("/tmp/pva_table4_local_mirror_20260827/extracted")

RUN_SCHEMA = "table1_diversity_run_v1"
VARIANCE_PROTOCOL = "within-release-label-population-variance-macro-v1"
GRAPH_PROTOCOL = "fixed-contracted-unordered-rooted-joint-tree-v1"
POINT_PROTOCOL = "q0-visual-aabb-diagonal-pcg64dxsm-v1"
DESCRIPTOR_PROTOCOL = "axis-radial-histogram-quantile-moments-v1"
CANDIDATE_PROTOCOL = "category-graph-ckdtree-v1"
CHAMFER_PROTOCOL = "symmetric-mean-unsquared-l2-pointcloud-v1"
RATE_PROTOCOL = "connected-component-excess-over-geometry-evaluable-v1"
CALIBRATION_PROTOCOL = "balanced-hard-random-human-label-asset-disjoint-v3"
PAIR_STRUCT = struct.Struct("<IIf")
PAIR_DTYPE = np.dtype([("left", "<u4"), ("right", "<u4"), ("distance", "<f4")])
CHAMFER_STRUCT = struct.Struct("<IId")
CHAMFER_DTYPE = np.dtype([("left", "<u4"), ("right", "<u4"), ("distance", "<f8")])
DESCRIPTOR_DIM = 94
BLAS_ENVIRONMENT = {
    "OMP_NUM_THREADS": "1",
    "OPENBLAS_NUM_THREADS": "1",
    "MKL_NUM_THREADS": "1",
    "NUMEXPR_NUM_THREADS": "1",
}


class MetricError(ValueError):
    """Raised when an input or output cannot satisfy the frozen protocol."""


@dataclass(frozen=True)
class DatasetSpec:
    key: str
    label: str
    records: Path
    roster: Path
    roster_kind: str
    expected_n: int


@dataclass(frozen=True)
class Asset:
    ordinal: int
    asset_id: str
    category: str
    urdf_path: str
    package_root: str
    expected_urdf_sha256: str | None
    package_binding_sha256: str | None = None
    expected_resources: tuple[tuple[str, str], ...] = ()
    expected_package_fingerprint: str | None = None


DATASETS: tuple[DatasetSpec, ...] = (
    DatasetSpec(
        "ours_500k", "Ours-500K",
        RUNTIME_ROOT / "table1_ours_500k/asset_records.jsonl",
        RUNTIME_ROOT / "table1_ours_500k/manifest.json", "ours_manifest", 500,
    ),
    DatasetSpec(
        "ours_n5", "Ours per-class N=5 (supplementary)",
        RUNTIME_ROOT / "table1_pva_per_class_n5_max_joints/asset_records.jsonl",
        RUNTIME_ROOT / "table1_pva_per_class_n5_max_joints/manifest.json", "n5_manifest", 2655,
    ),
    DatasetSpec(
        "pva", "Ours / PV-A",
        RUNTIME_ROOT / "pva_table1234_full_release_20260826/evaluation/table1/asset_records.jsonl",
        RUNTIME_ROOT / "pva_table1234_full_release_20260826/roster/full_release_roster.jsonl",
        "jsonl", 302440,
    ),
    DatasetSpec(
        "articraft", "Articraft-10K (merged source cohort)",
        RUNTIME_ROOT / "articraft_github_merged_10787_20260827/merged/table1/asset_records.jsonl",
        RUNTIME_ROOT / "articraft_github_merged_10787_20260827/rosters/merged/full_release_roster.jsonl",
        "jsonl", 10787,
    ),
    *tuple(
        DatasetSpec(
            key,
            label,
            RUNTIME_ROOT / f"table123_full_release_20260825/{key}/table1/asset_records.jsonl",
            RUNTIME_ROOT / f"table123_full_release_20260825/{key}/full_release_roster.jsonl",
            "jsonl",
            expected,
        )
        for key, label, expected in (
            ("lam", "LAM released outputs", 3217),
            ("artiverse", "Artiverse", 3544),
            ("partnet", "PartNet-Mobility", 2347),
            ("physx", "PhysX-Mobility", 2024),
            ("sketch", "SketchMobility", 4956),
            ("infinite", "Infinite Mobility", 720),
            ("infinigen", "Infinigen-Sim", 8226),
        )
    ),
)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def canonical_bytes(value: Any) -> bytes:
    try:
        return json.dumps(
            value, sort_keys=True, separators=(",", ":"), ensure_ascii=True,
            allow_nan=False,
        ).encode("utf-8")
    except (TypeError, ValueError) as error:
        raise MetricError(f"value is not canonical JSON: {error}") from error


def canonical_sha256(value: Any) -> str:
    return hashlib.sha256(canonical_bytes(value)).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for block in iter(lambda: handle.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def atomic_write(path: Path, payload: bytes) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and path.is_symlink():
        raise MetricError(f"refusing to replace symlink: {path}")
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(descriptor, "wb") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def atomic_copy_with_sha256(source: Path, destination: Path) -> str:
    source, destination = Path(source), Path(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{destination.name}.", dir=destination.parent
    )
    digest = hashlib.sha256()
    try:
        with source.open("rb") as reader, os.fdopen(descriptor, "wb") as writer:
            for block in iter(lambda: reader.read(4 * 1024 * 1024), b""):
                digest.update(block)
                writer.write(block)
            writer.flush(); os.fsync(writer.fileno())
        os.replace(temporary, destination)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise
    return digest.hexdigest()


def fsync_file(path: Path) -> None:
    with Path(path).open("rb") as handle:
        os.fsync(handle.fileno())


_TABLE1_CORE: Any | None = None


def table1_core() -> Any:
    global _TABLE1_CORE
    if _TABLE1_CORE is None:
        core_path = SCRIPT_PATH.with_name("run_table1_artiverse.py")
        spec = importlib.util.spec_from_file_location(
            "table1_diversity_fingerprint_core", core_path
        )
        if spec is None or spec.loader is None:
            raise MetricError(f"cannot load frozen Table 1 fingerprint core: {core_path}")
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        _TABLE1_CORE = module
    return _TABLE1_CORE


def write_json(path: Path, value: Mapping[str, Any], *, self_hash: str | None = None) -> dict[str, Any]:
    result = dict(value)
    if self_hash is not None:
        result.pop(self_hash, None)
        result[self_hash] = canonical_sha256(result)
    atomic_write(path, json.dumps(result, indent=2, sort_keys=True, ensure_ascii=True).encode() + b"\n")
    return result


def read_self_hashed_json(path: Path, field: str) -> dict[str, Any]:
    try:
        value = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise MetricError(f"cannot read completed artifact {path}: {error}") from error
    if not isinstance(value, dict):
        raise MetricError(f"completed artifact is not an object: {path}")
    unsigned = dict(value)
    declared = unsigned.pop(field, None)
    if declared != canonical_sha256(unsigned):
        raise MetricError(f"completed artifact self-hash mismatch: {path}")
    return value


def write_jsonl(path: Path, rows: Iterable[Mapping[str, Any]]) -> None:
    payload = b"".join(canonical_bytes(dict(row)) + b"\n" for row in rows)
    atomic_write(path, payload)


def iter_jsonl(path: Path) -> Iterator[dict[str, Any]]:
    with Path(path).open("r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, 1):
            if not line.strip():
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError as error:
                raise MetricError(f"invalid JSONL {path}:{line_number}: {error}") from error
            if not isinstance(row, dict):
                raise MetricError(f"JSONL row is not an object: {path}:{line_number}")
            yield row


def _spec(key: str) -> DatasetSpec:
    for spec in DATASETS:
        if spec.key == key:
            return spec
    raise MetricError(f"unknown dataset: {key}")


def selected_specs(keys: Sequence[str] | None = None) -> tuple[DatasetSpec, ...]:
    if not keys:
        return DATASETS
    requested = set(keys)
    result = tuple(spec for spec in DATASETS if spec.key in requested)
    missing = requested - {spec.key for spec in result}
    if missing:
        raise MetricError(f"unknown datasets: {', '.join(sorted(missing))}")
    return result


def category_key(dataset: str, row: Mapping[str, Any]) -> str:
    raw = row.get("raw_category", row.get("category"))
    if not isinstance(raw, str) or not raw.strip():
        raise MetricError(f"{dataset} row has no non-empty category")
    raw = raw.strip()
    if dataset == "sketch":
        asset_id = row.get("asset_id")
        if not isinstance(asset_id, str) or "/" not in asset_id:
            raise MetricError("SketchMobility row cannot recover source/category label")
        source = asset_id.split("/", 1)[0].removeprefix("data/")
        if source == "data" and asset_id.count("/") >= 2:
            source = asset_id.split("/", 2)[1]
        return f"{source}/{raw}"
    return raw


def _fraction_text(value: Fraction) -> str:
    return f"{value.numerator}/{value.denominator}"


def category_bootstrap_ci95(
    values: Sequence[float], *, seed_key: str, resamples: int = 10_000
) -> list[float]:
    array = np.asarray(values, dtype=np.float64)
    if array.ndim != 1 or not len(array) or not np.all(np.isfinite(array)):
        raise MetricError("category bootstrap requires finite one-dimensional values")
    if len(array) == 1:
        return [float(array[0]), float(array[0])]
    seed = int(hashlib.sha256(seed_key.encode("utf-8")).hexdigest(), 16)
    rng = np.random.Generator(np.random.PCG64DXSM(seed))
    means = np.empty(resamples, dtype=np.float64)
    for start in range(0, resamples, 256):
        stop = min(start + 256, resamples)
        indices = rng.integers(0, len(array), size=(stop - start, len(array)))
        means[start:stop] = np.mean(array[indices], axis=1)
    lower, upper = np.quantile(means, (0.025, 0.975))
    return [float(lower), float(upper)]


def aggregate_joint_variance(
    dataset: str,
    records: Iterable[Mapping[str, Any]],
    *,
    expected_n: int | None = None,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    totals: dict[str, list[int]] = defaultdict(lambda: [0, 0, 0, 0])
    seen = 0
    valid_assets = 0
    for row in records:
        seen += 1
        category = category_key(dataset, row)
        bucket = totals[category]
        bucket[0] += 1
        value = row.get("non_fixed_joint_count")
        valid = row.get("parse_success") is True and isinstance(value, int) and not isinstance(value, bool) and value >= 0
        if valid:
            value = int(value)
            bucket[1] += 1
            bucket[2] += value
            bucket[3] += value * value
            valid_assets += 1
    if expected_n is not None and seen != expected_n:
        raise MetricError(f"{dataset} record denominator drift: {seen} != {expected_n}")
    if not totals or not valid_assets:
        raise MetricError(f"{dataset} has no valid variance inputs")

    category_rows: list[dict[str, Any]] = []
    variances: list[Fraction] = []
    for category, (n_total, n_valid, total, total_sq) in sorted(totals.items()):
        variance: Fraction | None = None
        if n_valid:
            variance = Fraction(n_valid * total_sq - total * total, n_valid * n_valid)
            variances.append(variance)
        category_rows.append({
            "category": category,
            "n_total": n_total,
            "n_valid": n_valid,
            "sum_joints": total,
            "sum_squared_joints": total_sq,
            "population_variance": None if variance is None else float(variance),
            "population_variance_exact": None if variance is None else _fraction_text(variance),
        })
    macro = sum(variances, Fraction()) / len(variances)
    summary = {
        "dataset_key": dataset,
        "protocol": VARIANCE_PROTOCOL,
        "category_policy": "frozen release label; SketchMobility uses {source}/{category}",
        "ddof": 0,
        "singleton_variance": 0,
        "n_total": seen,
        "n_valid": valid_assets,
        "asset_coverage": valid_assets / seen,
        "category_count": len(totals),
        "valid_category_count": len(variances),
        "category_coverage": len(variances) / len(totals),
        "all_invalid_category_count": len(totals) - len(variances),
        "macro_population_variance": float(macro),
        "macro_population_variance_exact": _fraction_text(macro),
        "macro_population_variance_ci95": category_bootstrap_ci95(
            [float(value) for value in variances],
            seed_key=f"{VARIANCE_PROTOCOL}|bootstrap|{dataset}",
        ),
        "bootstrap_resamples": 10_000,
    }
    return summary, category_rows


def run_variance(output: Path, specs: Sequence[DatasetSpec]) -> dict[str, Any]:
    target = output / "variance"
    category_rows: list[dict[str, Any]] = []
    summaries: list[dict[str, Any]] = []
    for spec in specs:
        print(f"[variance] {spec.key}: {spec.records}", flush=True)
        if not spec.records.is_file():
            raise MetricError(f"missing Table 1 records: {spec.records}")
        summary, rows = aggregate_joint_variance(
            spec.key, iter_jsonl(spec.records), expected_n=spec.expected_n
        )
        summary["dataset"] = spec.label
        summary["records_path"] = str(spec.records)
        summary["records_sha256"] = sha256_file(spec.records)
        summaries.append(summary)
        category_rows.extend({"dataset_key": spec.key, **row} for row in rows)
    write_jsonl(target / "category_records.jsonl", category_rows)
    result = write_json(
        target / "summary.json",
        {
            "schema_version": "table1_joint_variance_summary_v1",
            "status": "COMPLETE",
            "created_at_utc": utc_now(),
            "protocol": VARIANCE_PROTOCOL,
            "category_records_sha256": sha256_file(target / "category_records.jsonl"),
            "datasets": summaries,
        },
        self_hash="summary_content_sha256",
    )
    return result


def local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def children(node: ET.Element, name: str) -> list[ET.Element]:
    return [child for child in node if local_name(child.tag) == name]


def _finite_vector(raw: str | None, default: Sequence[float], label: str) -> np.ndarray:
    fields = list(default) if raw is None else raw.split()
    if len(fields) != len(default):
        raise MetricError(f"{label} must contain {len(default)} values")
    try:
        values = np.asarray([float(value) for value in fields], dtype=np.float64)
    except ValueError as error:
        raise MetricError(f"{label} is not numeric") from error
    if not np.all(np.isfinite(values)):
        raise MetricError(f"{label} contains a non-finite value")
    return values


def origin_transform(node: ET.Element | None) -> np.ndarray:
    result = np.eye(4, dtype=np.float64)
    if node is None:
        return result
    xyz = _finite_vector(node.get("xyz"), (0.0, 0.0, 0.0), "origin xyz")
    roll, pitch, yaw = _finite_vector(node.get("rpy"), (0.0, 0.0, 0.0), "origin rpy")
    cr, sr = math.cos(float(roll)), math.sin(float(roll))
    cp, sp = math.cos(float(pitch)), math.sin(float(pitch))
    cy, sy = math.cos(float(yaw)), math.sin(float(yaw))
    rx = np.asarray(((1, 0, 0), (0, cr, -sr), (0, sr, cr)), dtype=np.float64)
    ry = np.asarray(((cp, 0, sp), (0, 1, 0), (-sp, 0, cp)), dtype=np.float64)
    rz = np.asarray(((cy, -sy, 0), (sy, cy, 0), (0, 0, 1)), dtype=np.float64)
    result[:3, :3] = rz @ ry @ rx
    result[:3, 3] = xyz
    return result


def parse_robot(root: ET.Element) -> tuple[list[str], list[dict[str, Any]]]:
    if local_name(root.tag) != "robot":
        raise MetricError("URDF root is not <robot>")
    links = [str(node.get("name") or "").strip() for node in children(root, "link")]
    if not links or any(not name for name in links) or len(set(links)) != len(links):
        raise MetricError("URDF link names are empty or duplicated")
    link_set = set(links)
    joints: list[dict[str, Any]] = []
    joint_names: set[str] = set()
    for index, node in enumerate(children(root, "joint")):
        name = str(node.get("name") or f"__unnamed_{index}")
        if name in joint_names:
            raise MetricError("URDF joint names are duplicated")
        joint_names.add(name)
        parent_nodes = children(node, "parent")
        child_nodes = children(node, "child")
        if len(parent_nodes) != 1 or len(child_nodes) != 1:
            raise MetricError(f"joint {name} does not have one parent and child")
        parent = str(parent_nodes[0].get("link") or "")
        child = str(child_nodes[0].get("link") or "")
        if parent not in link_set or child not in link_set or parent == child:
            raise MetricError(f"joint {name} has invalid endpoints")
        joint_type = str(node.get("type") or "").strip().lower()
        if not joint_type:
            raise MetricError(f"joint {name} has no type")
        if children(node, "mimic"):
            raise MetricError("mimic dependency canonicalization is not frozen")
        origins = children(node, "origin")
        if len(origins) > 1:
            raise MetricError(f"joint {name} has multiple origins")
        joints.append({
            "name": name,
            "parent": parent,
            "child": child,
            "type": joint_type,
            "origin": origin_transform(origins[0] if origins else None),
        })
    if len(joints) != len(links) - 1:
        raise MetricError("URDF joint graph is not a tree")
    indegree: dict[str, int] = defaultdict(int)
    adjacency: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for joint in joints:
        indegree[joint["child"]] += 1
        adjacency[joint["parent"]].append(joint)
    if any(value > 1 for value in indegree.values()):
        raise MetricError("URDF link has multiple parent joints")
    roots = [link for link in links if indegree[link] == 0]
    if len(roots) != 1:
        raise MetricError(f"URDF joint graph has {len(roots)} roots")
    visited: set[str] = set()
    active: set[str] = set()

    def visit(link: str) -> None:
        if link in active:
            raise MetricError("URDF joint graph contains a cycle")
        if link in visited:
            return
        active.add(link)
        for joint in adjacency.get(link, []):
            visit(joint["child"])
        active.remove(link)
        visited.add(link)

    visit(roots[0])
    if visited != link_set:
        raise MetricError("URDF joint graph is disconnected")
    return links, joints


def canonical_kinematic_graph(root: ET.Element) -> tuple[str, str]:
    links, joints = parse_robot(root)
    parent = {link: link for link in links}

    def find(value: str) -> str:
        while parent[value] != value:
            parent[value] = parent[parent[value]]
            value = parent[value]
        return value

    def union(left: str, right: str) -> None:
        left_root, right_root = find(left), find(right)
        if left_root != right_root:
            parent[right_root] = left_root

    for joint in joints:
        if joint["type"] == "fixed":
            union(joint["parent"], joint["child"])
    root_link = next(link for link in links if all(joint["child"] != link for joint in joints))
    graph: dict[str, list[tuple[str, str]]] = defaultdict(list)
    components = {find(link) for link in links}
    for joint in joints:
        if joint["type"] == "fixed":
            continue
        source, target = find(joint["parent"]), find(joint["child"])
        if source == target:
            raise MetricError("non-fixed joint collapsed inside a fixed component")
        graph[source].append((target, joint["type"]))

    def encode(component: str, active: set[str]) -> str:
        if component in active:
            raise MetricError("contracted kinematic graph contains a cycle")
        active.add(component)
        encoded = sorted(f"{kind}:{encode(child, active)}" for child, kind in graph.get(component, []))
        active.remove(component)
        return f"({','.join(encoded)})"

    signature = encode(find(root_link), set())
    reached: set[str] = set()

    def collect(component: str) -> None:
        if component in reached:
            return
        reached.add(component)
        for child, _ in graph.get(component, []):
            collect(child)

    collect(find(root_link))
    if reached != components:
        raise MetricError("fixed-contracted graph is disconnected")
    digest = hashlib.sha256(f"{GRAPH_PROTOCOL}\0{signature}".encode()).hexdigest()
    return signature, digest


def q0_link_transforms(root: ET.Element) -> dict[str, np.ndarray]:
    links, joints = parse_robot(root)
    children_set = {joint["child"] for joint in joints}
    root_link = next(link for link in links if link not in children_set)
    adjacency: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for joint in joints:
        adjacency[joint["parent"]].append(joint)
    world = {root_link: np.eye(4, dtype=np.float64)}
    queue = [root_link]
    while queue:
        parent = queue.pop(0)
        for joint in adjacency.get(parent, []):
            child = joint["child"]
            world[child] = world[parent] @ joint["origin"]
            queue.append(child)
    if len(world) != len(links):
        raise MetricError("q=0 FK did not reach every link")
    return world


def _contained_file(reference: str, urdf_path: Path, package_root: Path) -> Path:
    reference = reference.strip()
    if not reference or "\x00" in reference or "\\" in reference:
        raise MetricError("mesh reference is empty or malformed")
    candidates: list[Path] = []
    if reference.startswith("package://"):
        remainder = reference[len("package://"):]
        parts = Path(remainder).parts
        candidates.append(package_root / remainder)
        if len(parts) > 1:
            candidates.append(package_root / Path(*parts[1:]))
    elif "://" in reference:
        raise MetricError(f"unsupported mesh URI: {reference}")
    else:
        raw = Path(reference)
        candidates.append(raw if raw.is_absolute() else urdf_path.parent / raw)
        if not raw.is_absolute():
            candidates.append(package_root / raw)
    package_root = package_root.resolve(strict=True)
    for candidate in candidates:
        try:
            resolved = candidate.resolve(strict=True)
            resolved.relative_to(package_root)
        except (FileNotFoundError, ValueError):
            continue
        if resolved.is_file():
            return resolved
    raise MetricError(f"mesh resource is missing or escapes package: {reference}")


def _mesh_instances(path: Path, scale: np.ndarray) -> list[trimesh.Trimesh]:
    loaded = trimesh.load(path, force="scene", process=False)
    result: list[trimesh.Trimesh] = []
    if isinstance(loaded, trimesh.Trimesh):
        mesh = loaded.copy()
        mesh.vertices = np.asarray(mesh.vertices, dtype=np.float64) * scale.reshape(1, 3)
        result.append(mesh)
    elif isinstance(loaded, trimesh.Scene):
        for node_name in sorted(loaded.graph.nodes_geometry):
            transform, geometry_name = loaded.graph.get(node_name)
            if geometry_name is None:
                continue
            geometry = loaded.geometry.get(geometry_name)
            if not isinstance(geometry, trimesh.Trimesh):
                continue
            mesh = geometry.copy()
            mesh.apply_transform(np.asarray(transform, dtype=np.float64))
            # URDF mesh scale acts on the imported scene result, including a
            # scene node's translation: T_visual @ S_mesh @ T_scene @ V.
            mesh.vertices = np.asarray(mesh.vertices, dtype=np.float64) * scale.reshape(1, 3)
            result.append(mesh)
    if not result:
        raise MetricError(f"mesh has no triangular scene geometry: {path}")
    return result


def _record_visual_resource(
    path: Path,
    package_root: Path,
    expected: Mapping[str, str],
    observed: dict[str, str],
) -> None:
    relative = path.resolve(strict=True).relative_to(package_root.resolve(strict=True)).as_posix()
    digest = sha256_file(path)
    if expected:
        declared = expected.get(relative)
        if declared is None:
            raise MetricError(f"loaded visual resource is absent from package binding: {relative}")
        if declared != digest:
            raise MetricError(f"loaded visual resource SHA-256 drift: {relative}")
    previous = observed.setdefault(relative, digest)
    if previous != digest:
        raise MetricError(f"visual resource changed during extraction: {relative}")


def _shape_meshes(
    shape: ET.Element,
    urdf_path: Path,
    package_root: Path,
    expected_resources: Mapping[str, str],
    observed_resources: dict[str, str],
) -> list[trimesh.Trimesh]:
    kind = local_name(shape.tag)
    if kind == "mesh":
        path = _contained_file(str(shape.get("filename") or ""), urdf_path, package_root)
        _record_visual_resource(path, package_root, expected_resources, observed_resources)
        if path.suffix.lower() == ".gltf":
            try:
                gltf = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, UnicodeError, json.JSONDecodeError) as error:
                raise MetricError(f"cannot inspect GLTF buffer closure: {path}") from error
            for buffer in gltf.get("buffers", []):
                uri = buffer.get("uri") if isinstance(buffer, Mapping) else None
                if not isinstance(uri, str) or uri.startswith("data:"):
                    continue
                dependency = _contained_file(uri, path, package_root)
                _record_visual_resource(
                    dependency, package_root, expected_resources, observed_resources
                )
        scale = _finite_vector(shape.get("scale"), (1.0, 1.0, 1.0), "mesh scale")
        if np.any(scale == 0.0):
            raise MetricError("mesh scale contains zero")
        return _mesh_instances(path, scale)
    if kind == "box":
        if shape.get("size") is None:
            raise MetricError("box size is missing")
        size = _finite_vector(shape.get("size"), (0.0, 0.0, 0.0), "box size")
        if len(size) != 3 or np.any(size <= 0):
            raise MetricError("box size must contain three positive values")
        return [trimesh.creation.box(extents=size)]
    if kind == "cylinder":
        radius, length = float(shape.get("radius", "nan")), float(shape.get("length", "nan"))
        if not math.isfinite(radius) or not math.isfinite(length) or min(radius, length) <= 0:
            raise MetricError("cylinder dimensions must be positive finite")
        return [trimesh.creation.cylinder(radius=radius, height=length, sections=32)]
    if kind == "sphere":
        radius = float(shape.get("radius", "nan"))
        if not math.isfinite(radius) or radius <= 0:
            raise MetricError("sphere radius must be positive finite")
        return [trimesh.creation.icosphere(subdivisions=3, radius=radius)]
    if kind == "capsule":
        radius, length = float(shape.get("radius", "nan")), float(shape.get("length", "nan"))
        if not math.isfinite(radius) or not math.isfinite(length) or min(radius, length) <= 0:
            raise MetricError("capsule dimensions must be positive finite")
        return [trimesh.creation.capsule(radius=radius, height=length, count=[16, 16])]
    raise MetricError(f"unsupported visual geometry kind: {kind}")


def sample_surface(
    vertices: np.ndarray, faces: np.ndarray, count: int, rng: np.random.Generator
) -> np.ndarray:
    triangles = np.asarray(vertices, dtype=np.float64)[np.asarray(faces, dtype=np.int64)]
    twice_area = np.linalg.norm(
        np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]), axis=1
    )
    keep = np.isfinite(twice_area) & (twice_area > 0)
    triangles = triangles[keep]
    area = twice_area[keep] * 0.5
    if not len(triangles):
        raise MetricError("visual surface has no positive-area triangles")
    cumulative = np.cumsum(area, dtype=np.float64)
    selected = np.searchsorted(cumulative, rng.random(count) * cumulative[-1], side="right")
    chosen = triangles[selected]
    u, v = rng.random(count), rng.random(count)
    root = np.sqrt(u)
    points = (
        (1.0 - root)[:, None] * chosen[:, 0]
        + (root * (1.0 - v))[:, None] * chosen[:, 1]
        + (root * v)[:, None] * chosen[:, 2]
    )
    if not np.all(np.isfinite(points)):
        raise MetricError("surface sampler produced non-finite points")
    return np.asarray(points, dtype=np.float32)


def point_descriptor(points: np.ndarray) -> np.ndarray:
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or not len(points) or not np.all(np.isfinite(points)):
        raise MetricError("descriptor requires finite Nx3 points")
    parts: list[np.ndarray] = []
    for axis in range(3):
        histogram, _ = np.histogram(points[:, axis], bins=16, range=(-0.5, 0.5))
        parts.append(histogram.astype(np.float64) / len(points))
    radius = np.linalg.norm(points, axis=1)
    radial, _ = np.histogram(radius, bins=16, range=(0.0, 0.5))
    parts.append(radial.astype(np.float64) / len(points))
    quantiles = np.quantile(np.abs(points), (0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0), axis=0)
    parts.append(quantiles.T.reshape(-1))
    covariance = np.cov(points, rowvar=False, ddof=0)
    parts.append(np.linalg.eigvalsh(covariance))
    parts.append(np.mean(points, axis=0))
    parts.append(np.std(points, axis=0, ddof=0))
    result = np.concatenate(parts).astype(np.float32)
    if result.shape != (DESCRIPTOR_DIM,) or not np.all(np.isfinite(result)):
        raise MetricError(f"descriptor contract drift: {result.shape}")
    return result


def extract_point_cloud(asset: Asset, point_count: int) -> tuple[dict[str, Any], np.ndarray, np.ndarray]:
    path = Path(asset.urdf_path)
    package_root = Path(asset.package_root)
    if not path.is_file() or path.is_symlink():
        raise MetricError("primary URDF is missing or is a symlink")
    if not asset.expected_urdf_sha256:
        raise MetricError("primary URDF has no frozen Table 1 SHA-256 binding")
    if sha256_file(path) != asset.expected_urdf_sha256:
        raise MetricError("primary URDF SHA-256 drift")
    observed_package_fingerprint = None
    fingerprint_resource_count = None
    if asset.expected_package_fingerprint:
        fingerprint = table1_core().fingerprint_package(path, package_root=package_root)
        if not fingerprint.get("complete"):
            raise MetricError("frozen Table 1 package fingerprint closure is now incomplete")
        observed_package_fingerprint = str(fingerprint.get("fingerprint"))
        fingerprint_resource_count = int(fingerprint.get("resource_count", 0))
        if observed_package_fingerprint != asset.expected_package_fingerprint:
            raise MetricError("frozen Table 1 package fingerprint drift")
    payload = path.read_bytes()
    if b"<!DOCTYPE" in payload.upper() or b"<!ENTITY" in payload.upper():
        raise MetricError("URDF DTD/entity declarations are forbidden")
    root = ET.fromstring(payload)
    visual_mesh_referenced = any(
        local_name(shape.tag) == "mesh"
        for link in children(root, "link")
        for visual in children(link, "visual")
        for geometry in children(visual, "geometry")
        for shape in geometry
    )
    if asset.expected_package_fingerprint:
        visual_binding_mode = "TABLE1_PACKAGE_FINGERPRINT"
    elif asset.expected_resources:
        visual_binding_mode = "ROSTER_RESOURCE_MANIFEST"
    elif visual_mesh_referenced:
        raise MetricError(
            "visual mesh has neither a frozen package fingerprint nor a resource manifest"
        )
    else:
        visual_binding_mode = "URDF_HASHED_PRIMITIVES_ONLY"
    signature, graph_hash = canonical_kinematic_graph(root)
    world = q0_link_transforms(root)
    meshes: list[trimesh.Trimesh] = []
    observed_resources: dict[str, str] = {}
    expected_resources = dict(asset.expected_resources)
    declared_visuals = 0
    for link in children(root, "link"):
        link_name = str(link.get("name") or "")
        for visual in children(link, "visual"):
            declared_visuals += 1
            geometries = children(visual, "geometry")
            if len(geometries) != 1:
                raise MetricError(f"visual {link_name} does not contain exactly one geometry")
            shapes = list(geometries[0])
            if len(shapes) != 1:
                raise MetricError(f"visual {link_name} geometry has {len(shapes)} shapes")
            origins = children(visual, "origin")
            if len(origins) > 1:
                raise MetricError(f"visual {link_name} has multiple origins")
            transform = world[link_name] @ origin_transform(origins[0] if origins else None)
            for mesh in _shape_meshes(
                shapes[0], path, package_root, expected_resources, observed_resources
            ):
                mesh.apply_transform(transform)
                meshes.append(mesh)
    if declared_visuals == 0 or len(meshes) < declared_visuals:
        raise MetricError("not every declared visual produced triangular geometry")
    vertices: list[np.ndarray] = []
    faces: list[np.ndarray] = []
    offset = 0
    for mesh in meshes:
        current_vertices = np.asarray(mesh.vertices, dtype=np.float64)
        current_faces = np.asarray(mesh.faces, dtype=np.int64)
        if current_vertices.ndim != 2 or current_vertices.shape[1] != 3 or not len(current_faces):
            raise MetricError("visual mesh is not a non-empty triangle mesh")
        if not np.all(np.isfinite(current_vertices)):
            raise MetricError("visual mesh contains non-finite vertices")
        vertices.append(current_vertices)
        faces.append(current_faces + offset)
        offset += len(current_vertices)
    union_vertices = np.vstack(vertices)
    union_faces = np.vstack(faces)
    lower, upper = np.min(union_vertices, axis=0), np.max(union_vertices, axis=0)
    diagonal = float(np.linalg.norm(upper - lower))
    if not math.isfinite(diagonal) or diagonal <= 0:
        raise MetricError("visual AABB diagonal is not positive finite")
    normalized = (union_vertices - (lower + upper) / 2.0) / diagonal
    seed_hex = hashlib.sha256(
        f"{POINT_PROTOCOL}|{asset.asset_id}".encode("utf-8")
    ).hexdigest()
    rng = np.random.Generator(np.random.PCG64DXSM(int(seed_hex, 16)))
    points = sample_surface(normalized, union_faces, point_count, rng)
    descriptor = point_descriptor(points)
    resource_bindings = [
        {"path": relative, "sha256": digest}
        for relative, digest in sorted(observed_resources.items())
    ]
    record = {
        "ordinal": asset.ordinal,
        "asset_id": asset.asset_id,
        "category": asset.category,
        "status": "EVALUATED",
        "graph_signature": signature,
        "graph_hash": graph_hash,
        "declared_visual_count": declared_visuals,
        "loaded_mesh_instance_count": len(meshes),
        "source_aabb_diagonal": diagonal,
        "primary_urdf_sha256": sha256_file(path),
        "package_binding_sha256": asset.package_binding_sha256,
        "expected_package_fingerprint": asset.expected_package_fingerprint,
        "package_fingerprint": observed_package_fingerprint,
        "fingerprint_resource_count": fingerprint_resource_count,
        "visual_binding_mode": visual_binding_mode,
        "visual_resource_bindings": resource_bindings,
        "visual_resource_content_sha256": canonical_sha256(resource_bindings),
        "error": None,
    }
    return record, points, descriptor


def _extract_worker(task: tuple[Asset, int]) -> tuple[dict[str, Any], np.ndarray | None, np.ndarray | None]:
    asset, point_count = task
    for attempt in range(1, 4):
        try:
            record, cloud, descriptor = extract_point_cloud(asset, point_count)
            record.update({"attempt_count": attempt, "error_kind": None})
            return record, cloud, descriptor
        except Exception as error:  # noqa: BLE001 - every frozen asset gets a record
            transient = isinstance(error, (OSError, EOFError))
            if transient and attempt < 3:
                continue
            return ({
                "ordinal": asset.ordinal,
                "asset_id": asset.asset_id,
                "category": asset.category,
                "status": "ERROR",
                "graph_signature": None,
                "graph_hash": None,
                "declared_visual_count": None,
                "loaded_mesh_instance_count": None,
                "source_aabb_diagonal": None,
                "primary_urdf_sha256": asset.expected_urdf_sha256,
                "package_binding_sha256": asset.package_binding_sha256,
                "expected_package_fingerprint": asset.expected_package_fingerprint,
                "package_fingerprint": None,
                "fingerprint_resource_count": None,
                "visual_binding_mode": None,
                "visual_resource_bindings": None,
                "visual_resource_content_sha256": None,
                "attempt_count": attempt,
                "error_kind": "TRANSIENT_EXHAUSTED" if transient else "SEMANTIC_OR_UNSUPPORTED",
                "error": f"{type(error).__name__}: {error}"[:2000],
            }, None, None)
    raise AssertionError("unreachable retry state")


def _row_asset(
    spec: DatasetSpec,
    ordinal: int,
    row: Mapping[str, Any],
    *,
    pva_root_override: Path | None,
    dataset_root: Path | None = None,
    record_binding: Mapping[str, Any] | None = None,
) -> Asset:
    if spec.roster_kind == "ours_manifest":
        asset_id = str(row["asset_id"])
        if dataset_root is None:
            raise MetricError("Ours manifest is missing dataset_root")
        urdf = dataset_root / str(row["primary_urdf"])
        package = dataset_root / str(row["asset_root"])
        expected = row.get("primary_urdf_sha256", row.get("urdf_sha256"))
    elif spec.roster_kind == "n5_manifest":
        asset_id = str(row.get("dataset_id", row["asset_id"]))
        package = Path(str(row["package"]))
        urdf = package / str(row.get("primary_urdf_relative_path", "model.urdf"))
        expected = row.get("primary_urdf_sha256", row.get("urdf_sha256"))
    else:
        asset_id = str(row["asset_id"])
        urdf = Path(str(row["primary_urdf_path"]))
        package = Path(str(row.get("source_path", urdf.parent)))
        expected = row.get("primary_urdf_sha256")
        if spec.key == "pva" and pva_root_override is not None:
            try:
                relative_urdf = urdf.relative_to(PVA_SOURCE_ROOT)
                relative_package = package.relative_to(PVA_SOURCE_ROOT)
            except ValueError as error:
                raise MetricError(f"PV-A roster path is outside frozen source root: {urdf}") from error
            urdf = pva_root_override / relative_urdf
            package = pva_root_override / relative_package
    record_binding = record_binding or {}
    record_urdf_hash = record_binding.get("primary_urdf_sha256")
    if expected and record_urdf_hash and str(expected) != str(record_urdf_hash):
        raise MetricError(f"{spec.key} roster/Table 1 URDF hash mismatch: {asset_id}")
    expected = record_urdf_hash or expected
    category = category_key(spec.key, {**row, "asset_id": asset_id})
    raw_files = row.get("package_files")
    if not isinstance(raw_files, list):
        binding = row.get("package_binding")
        raw_files = binding.get("files", []) if isinstance(binding, Mapping) else []
    resources: list[tuple[str, str]] = []
    for item in raw_files:
        if not isinstance(item, Mapping):
            raise MetricError(f"{spec.key} package file binding is malformed")
        relative, resource_hash = item.get("path"), item.get("sha256")
        if not isinstance(relative, str) or not isinstance(resource_hash, str):
            raise MetricError(f"{spec.key} package file binding lacks path/SHA-256")
        resources.append((relative, resource_hash))
    if len({path for path, _ in resources}) != len(resources):
        raise MetricError(f"{spec.key} package file binding contains duplicate paths")
    return Asset(
        ordinal,
        asset_id,
        category,
        str(urdf),
        str(package),
        str(expected) if expected else None,
        str(row.get("package_binding_sha256") or record_binding.get("package_binding_sha256"))
        if row.get("package_binding_sha256") or record_binding.get("package_binding_sha256")
        else None,
        tuple(sorted(resources)),
        str(record_binding.get("package_fingerprint"))
        if record_binding.get("fingerprint_complete") is True
        and record_binding.get("package_fingerprint")
        else None,
    )


def load_assets(spec: DatasetSpec, *, pva_root_override: Path | None = None) -> tuple[list[Asset], str]:
    assets: list[Asset] = []
    digest = hashlib.sha256()
    record_bindings: dict[str, dict[str, Any]] = {}
    for record in iter_jsonl(spec.records):
        record_id = str(record.get("dataset_id", record.get("asset_id"))) if spec.roster_kind == "n5_manifest" else str(record.get("asset_id"))
        if not record_id or record_id == "None" or record_id in record_bindings:
            raise MetricError(f"{spec.key} Table 1 record identities are invalid or duplicated")
        record_bindings[record_id] = {
            "primary_urdf_sha256": record.get("primary_urdf_sha256"),
            "package_binding_sha256": record.get("package_binding_sha256"),
            "fingerprint_complete": record.get("fingerprint_complete"),
            "package_fingerprint": record.get("package_fingerprint"),
        }
    if len(record_bindings) != spec.expected_n:
        raise MetricError(f"{spec.key} Table 1 record binding denominator drift")
    if spec.roster_kind in {"ours_manifest", "n5_manifest"}:
        manifest = json.loads(spec.roster.read_text(encoding="utf-8"))
        rows = manifest.get("assets")
        if not isinstance(rows, list):
            raise MetricError(f"manifest has no assets: {spec.roster}")
        dataset_root = Path(str(manifest["dataset_root"])) if spec.roster_kind == "ours_manifest" else None
        iterator: Iterable[Mapping[str, Any]] = rows
    else:
        dataset_root = None
        iterator = iter_jsonl(spec.roster)
    for ordinal, row in enumerate(iterator):
        identity = str(row.get("dataset_id", row.get("asset_id"))) if spec.roster_kind == "n5_manifest" else str(row.get("asset_id"))
        asset = _row_asset(
            spec, ordinal, row, pva_root_override=pva_root_override,
            dataset_root=dataset_root, record_binding=record_bindings.get(identity),
        )
        digest.update(canonical_bytes(asdict(asset)) + b"\n")
        assets.append(asset)
    if len(assets) != spec.expected_n:
        raise MetricError(f"{spec.key} roster denominator drift: {len(assets)} != {spec.expected_n}")
    if len({asset.asset_id for asset in assets}) != len(assets):
        raise MetricError(f"{spec.key} roster asset IDs are not unique")
    if {asset.asset_id for asset in assets} != set(record_bindings):
        raise MetricError(f"{spec.key} roster/Table 1 record identities do not close")
    return assets, digest.hexdigest()


def symmetric_chamfer(left: np.ndarray, right: np.ndarray) -> float:
    left = np.asarray(left, dtype=np.float64)
    right = np.asarray(right, dtype=np.float64)
    if left.ndim != 2 or right.ndim != 2 or left.shape[1:] != (3,) or right.shape[1:] != (3,):
        raise MetricError("Chamfer inputs must be non-empty Nx3 arrays")
    if not len(left) or not len(right) or not np.all(np.isfinite(left)) or not np.all(np.isfinite(right)):
        raise MetricError("Chamfer inputs must be non-empty and finite")
    left_to_right = cKDTree(right).query(left, k=1, workers=1)[0]
    right_to_left = cKDTree(left).query(right, k=1, workers=1)[0]
    return float((np.mean(left_to_right) + np.mean(right_to_left)) * 0.5)


def candidate_pairs_for_group(descriptors: np.ndarray, top_k: int, exhaustive_limit: int) -> list[tuple[int, int, float]]:
    values = np.asarray(descriptors, dtype=np.float64)
    n = len(values)
    if n < 2:
        return []
    pairs: dict[tuple[int, int], float] = {}
    if n <= exhaustive_limit:
        for left in range(n):
            distances = np.linalg.norm(values[left + 1:] - values[left], axis=1)
            for offset, distance in enumerate(distances, left + 1):
                pairs[(left, offset)] = float(distance)
    else:
        k = min(n, top_k + 1)
        distances, indices = cKDTree(values).query(values, k=k, workers=1)
        if k == 1:
            distances, indices = distances[:, None], indices[:, None]
        for left in range(n):
            for distance, right_value in zip(distances[left], indices[left], strict=True):
                right = int(right_value)
                if right == left:
                    continue
                pair = (min(left, right), max(left, right))
                previous = pairs.get(pair)
                pairs[pair] = float(distance) if previous is None else min(previous, float(distance))
    return [(left, right, pairs[(left, right)]) for left, right in sorted(pairs)]


def duplicate_rates(n_evaluable: int, positive_pairs: Iterable[tuple[int, int]]) -> dict[str, Any]:
    if n_evaluable < 0:
        raise MetricError("evaluable denominator cannot be negative")
    parent = list(range(n_evaluable))
    touched: set[int] = set()

    def find(value: int) -> int:
        while parent[value] != value:
            parent[value] = parent[parent[value]]
            value = parent[value]
        return value

    for left, right in positive_pairs:
        if not (0 <= left < n_evaluable and 0 <= right < n_evaluable) or left == right:
            raise MetricError("duplicate edge is outside the evaluable denominator")
        touched.update((left, right))
        left_root, right_root = find(left), find(right)
        if left_root != right_root:
            parent[right_root] = left_root
    component_count = len({find(index) for index in range(n_evaluable)})
    excess = n_evaluable - component_count
    return {
        "n_evaluable": n_evaluable,
        "neighbor_asset_count": len(touched),
        "neighbor_asset_rate": len(touched) / n_evaluable if n_evaluable else None,
        "component_count": component_count,
        "duplicate_excess_count": excess,
        "cluster_excess_rate": excess / n_evaluable if n_evaluable else None,
    }


def category_macro_duplicate_rates(
    categories: Sequence[str],
    positive_pairs: Sequence[tuple[int, int]],
    *,
    seed_key: str,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    members: dict[str, list[int]] = defaultdict(list)
    for index, category in enumerate(categories):
        members[str(category)].append(index)
    edges: dict[str, list[tuple[int, int]]] = defaultdict(list)
    for left, right in positive_pairs:
        if categories[left] != categories[right]:
            raise MetricError("near-duplicate edge crosses a category boundary")
        edges[str(categories[left])].append((left, right))
    rows: list[dict[str, Any]] = []
    for category, global_indices in sorted(members.items()):
        local = {global_index: local_index for local_index, global_index in enumerate(global_indices)}
        rates = duplicate_rates(
            len(global_indices),
            ((local[left], local[right]) for left, right in edges.get(category, [])),
        )
        rows.append({"category": category, **rates})
    cluster_values = [float(row["cluster_excess_rate"]) for row in rows]
    neighbor_values = [float(row["neighbor_asset_rate"]) for row in rows]
    return ({
        "category_count": len(rows),
        "category_macro_cluster_excess_rate": float(np.mean(cluster_values)),
        "category_macro_cluster_excess_rate_ci95": category_bootstrap_ci95(
            cluster_values, seed_key=f"{seed_key}|cluster"
        ),
        "category_macro_neighbor_asset_rate": float(np.mean(neighbor_values)),
        "category_macro_neighbor_asset_rate_ci95": category_bootstrap_ci95(
            neighbor_values, seed_key=f"{seed_key}|neighbor"
        ),
        "bootstrap_resamples": 10_000,
    }, rows)


def _db(path: Path) -> sqlite3.Connection:
    path.parent.mkdir(parents=True, exist_ok=True)
    connection = sqlite3.connect(path)
    connection.execute("PRAGMA journal_mode=WAL")
    connection.execute("PRAGMA synchronous=FULL")
    connection.execute(
        """CREATE TABLE IF NOT EXISTS assets (
        ordinal INTEGER PRIMARY KEY, asset_id TEXT NOT NULL UNIQUE, category TEXT NOT NULL,
        urdf_path TEXT NOT NULL, status TEXT NOT NULL, graph_hash TEXT,
        graph_signature TEXT, declared_visual_count INTEGER,
        loaded_mesh_instance_count INTEGER, source_aabb_diagonal REAL,
        primary_urdf_sha256 TEXT, package_binding_sha256 TEXT,
        expected_package_fingerprint TEXT, package_fingerprint TEXT,
        fingerprint_resource_count INTEGER,
        visual_binding_mode TEXT,
        visual_resource_bindings_json TEXT, visual_resource_content_sha256 TEXT,
        attempt_count INTEGER NOT NULL, error_kind TEXT, error TEXT)"""
    )
    connection.execute(
        "CREATE TABLE IF NOT EXISTS run_binding (name TEXT PRIMARY KEY, value TEXT NOT NULL)"
    )
    return connection


def _db_readonly(path: Path) -> sqlite3.Connection:
    absolute = Path(path).resolve(strict=True)
    return sqlite3.connect(f"file:{absolute}?mode=ro", uri=True)


def _geometry_records(connection: sqlite3.Connection) -> Iterator[dict[str, Any]]:
    columns = (
        "ordinal", "asset_id", "category", "urdf_path", "status", "graph_hash",
        "graph_signature", "declared_visual_count", "loaded_mesh_instance_count",
        "source_aabb_diagonal", "primary_urdf_sha256", "package_binding_sha256",
        "expected_package_fingerprint", "package_fingerprint",
        "fingerprint_resource_count", "visual_binding_mode",
        "visual_resource_bindings_json",
        "visual_resource_content_sha256", "attempt_count", "error_kind", "error",
    )
    for values in connection.execute(f"SELECT {','.join(columns)} FROM assets ORDER BY ordinal"):
        row = dict(zip(columns, values, strict=True))
        encoded = row.pop("visual_resource_bindings_json")
        row["visual_resource_bindings"] = None if encoded is None else json.loads(encoded)
        yield row


def verify_scratch_bindings(summary: Mapping[str, Any]) -> None:
    bindings = summary.get("scratch_bindings")
    if not isinstance(bindings, Mapping):
        raise MetricError("geometry summary has no scratch bindings")
    for name in ("points", "descriptors", "database"):
        binding = bindings.get(name)
        if not isinstance(binding, Mapping):
            raise MetricError(f"geometry summary lacks {name} scratch binding")
        path = Path(str(binding.get("path")))
        if not path.is_file() or path.stat().st_size != binding.get("bytes"):
            raise MetricError(f"geometry scratch artifact is missing or resized: {name}")
        if sha256_file(path) != binding.get("sha256"):
            raise MetricError(f"geometry scratch artifact hash drift: {name}")
        if name != "database":
            array = np.load(path, mmap_mode="r")
            if list(array.shape) != binding.get("shape") or str(array.dtype) != binding.get("dtype"):
                raise MetricError(f"geometry scratch array contract drift: {name}")


def verify_candidate_bindings(formal: Path, candidate: Mapping[str, Any]) -> None:
    geometry_path = formal / "geometry_summary.json"
    if candidate.get("geometry_summary_sha256") != sha256_file(geometry_path):
        raise MetricError("candidate summary is not bound to the geometry summary")
    pairs_path = Path(str(candidate.get("candidate_pairs_path")))
    if (
        not pairs_path.is_file()
        or pairs_path.stat().st_size
        != int(candidate.get("candidate_pair_count", -1)) * PAIR_STRUCT.size
        or sha256_file(pairs_path) != candidate.get("candidate_pairs_sha256")
    ):
        raise MetricError("candidate pair artifact is missing, resized, or hash-drifted")
    checkpoint_path = formal / "candidate_checkpoint.json"
    if sha256_file(checkpoint_path) != candidate.get("candidate_checkpoint_sha256"):
        raise MetricError("candidate checkpoint binding drift")
    checkpoint = read_self_hashed_json(
        checkpoint_path, "checkpoint_content_sha256"
    )
    if (
        checkpoint.get("state") != "COMPLETE"
        or checkpoint.get("candidate_pairs_sha256")
        != candidate.get("candidate_pairs_sha256")
        or checkpoint.get("candidate_pair_count")
        != candidate.get("candidate_pair_count")
    ):
        raise MetricError("candidate checkpoint completion contract drift")


def prepare_geometry_dataset(
    spec: DatasetSpec,
    assets: Sequence[Asset],
    roster_hash: str,
    output: Path,
    scratch: Path,
    *,
    workers: int,
    point_count: int,
) -> dict[str, Any]:
    formal = output / "near_duplicate" / spec.key
    local = scratch / spec.key
    local.mkdir(parents=True, exist_ok=True)
    database_path = local / "geometry.sqlite3"
    points_path = local / "points.npy"
    descriptors_path = local / "descriptors.npy"
    arrays_complete = points_path.exists() and descriptors_path.exists()
    connection = _db(database_path)
    committed_rows = int(connection.execute("SELECT COUNT(*) FROM assets").fetchone()[0])
    if arrays_complete:
        points = np.lib.format.open_memmap(points_path, mode="r+")
        descriptors = np.lib.format.open_memmap(descriptors_path, mode="r+")
        if points.shape != (len(assets), point_count, 3) or descriptors.shape != (len(assets), DESCRIPTOR_DIM):
            raise MetricError(f"{spec.key} scratch array shape does not match run manifest")
    else:
        if committed_rows:
            connection.close()
            raise MetricError(
                f"{spec.key} scratch arrays are incomplete after {committed_rows} committed rows"
            )
        # No database row points at either array yet, so an interrupted initial
        # allocation can be recreated without losing committed work.
        points = np.lib.format.open_memmap(points_path, mode="w+", dtype=np.float32, shape=(len(assets), point_count, 3))
        descriptors = np.lib.format.open_memmap(descriptors_path, mode="w+", dtype=np.float32, shape=(len(assets), DESCRIPTOR_DIM))
    run_binding = {
        "dataset_key": spec.key,
        "roster_compact_sha256": roster_hash,
        "n_total": len(assets),
        "point_count": point_count,
        "descriptor_dim": DESCRIPTOR_DIM,
        "graph_protocol": GRAPH_PROTOCOL,
        "point_protocol": POINT_PROTOCOL,
        "descriptor_protocol": DESCRIPTOR_PROTOCOL,
    }
    encoded_binding = canonical_bytes(run_binding).decode("ascii")
    frozen_binding = connection.execute(
        "SELECT value FROM run_binding WHERE name='geometry_run'"
    ).fetchone()
    if frozen_binding is None:
        connection.execute(
            "INSERT INTO run_binding(name,value) VALUES ('geometry_run',?)", (encoded_binding,)
        )
        connection.commit()
    elif frozen_binding[0] != encoded_binding:
        raise MetricError(f"{spec.key} geometry scratch run binding mismatch")
    existing = {
        int(ordinal): (str(asset_id), str(category), str(urdf_path))
        for ordinal, asset_id, category, urdf_path in connection.execute(
            "SELECT ordinal,asset_id,category,urdf_path FROM assets"
        )
    }
    for ordinal, identity in existing.items():
        asset = assets[ordinal]
        if identity != (asset.asset_id, asset.category, asset.urdf_path):
            raise MetricError(f"{spec.key} resume identity mismatch at ordinal {ordinal}")
    pending = [asset for asset in assets if asset.ordinal not in existing]
    print(f"[geometry] {spec.key}: complete={len(existing)} pending={len(pending)}", flush=True)
    completed = len(existing)

    def commit_geometry_batch() -> None:
        # SQLite rows are the commit marker. Array bytes must be durable first,
        # otherwise a crash could claim that an unpersisted cloud is complete.
        points.flush(); descriptors.flush()
        fsync_file(points_path); fsync_file(descriptors_path)
        connection.commit()

    try:
        if pending:
            context = multiprocessing.get_context("spawn")
            with context.Pool(processes=workers, maxtasksperchild=32) as pool:
                tasks = ((asset, point_count) for asset in pending)
                for record, cloud, descriptor in pool.imap_unordered(_extract_worker, tasks, chunksize=1):
                    ordinal = int(record["ordinal"])
                    asset = assets[ordinal]
                    if record["asset_id"] != asset.asset_id or record["category"] != asset.category:
                        raise MetricError("worker result identity drift")
                    if record["status"] == "EVALUATED":
                        assert cloud is not None and descriptor is not None
                        points[ordinal] = cloud
                        descriptors[ordinal] = descriptor
                    connection.execute(
                        """INSERT INTO assets (
                        ordinal,asset_id,category,urdf_path,status,graph_hash,
                        graph_signature,declared_visual_count,loaded_mesh_instance_count,
                        source_aabb_diagonal,primary_urdf_sha256,package_binding_sha256,
                        expected_package_fingerprint,package_fingerprint,
                        fingerprint_resource_count,visual_binding_mode,
                        visual_resource_bindings_json,
                        visual_resource_content_sha256,attempt_count,error_kind,error
                        ) VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)""",
                        (
                            ordinal, asset.asset_id, asset.category, asset.urdf_path,
                            record["status"], record["graph_hash"], record["graph_signature"],
                            record["declared_visual_count"], record["loaded_mesh_instance_count"],
                            record["source_aabb_diagonal"], record["primary_urdf_sha256"],
                            record["package_binding_sha256"],
                            record["expected_package_fingerprint"],
                            record["package_fingerprint"],
                            record["fingerprint_resource_count"],
                            record["visual_binding_mode"],
                            None if record["visual_resource_bindings"] is None else canonical_bytes(record["visual_resource_bindings"]).decode("ascii"),
                            record["visual_resource_content_sha256"], record["attempt_count"],
                            record["error_kind"], record["error"],
                        ),
                    )
                    completed += 1
                    if completed % 50 == 0:
                        commit_geometry_batch()
                    if completed % 250 == 0 or completed == len(assets):
                        print(f"[geometry] {spec.key}: {completed}/{len(assets)}", flush=True)
        commit_geometry_batch()
    except BaseException:
        connection.rollback()
        connection.close()
        raise
    if completed != len(assets):
        raise MetricError(f"{spec.key} geometry stage incomplete: {completed}/{len(assets)}")
    records_path = formal / "geometry_records.jsonl"
    write_jsonl(records_path, _geometry_records(connection))
    status_counts = dict(connection.execute("SELECT status,COUNT(*) FROM assets GROUP BY status").fetchall())
    retry_count = int(connection.execute(
        "SELECT COUNT(*) FROM assets WHERE attempt_count > 1"
    ).fetchone()[0])
    transient_exhausted = int(connection.execute(
        "SELECT COUNT(*) FROM assets WHERE error_kind='TRANSIENT_EXHAUSTED'"
    ).fetchone()[0])
    evaluated = int(status_counts.get("EVALUATED", 0))
    categories = int(connection.execute("SELECT COUNT(DISTINCT category) FROM assets").fetchone()[0])
    valid_categories = int(connection.execute(
        "SELECT COUNT(DISTINCT category) FROM assets WHERE status='EVALUATED'"
    ).fetchone()[0])
    connection.commit()
    connection.execute("PRAGMA wal_checkpoint(TRUNCATE)")
    connection.close()
    points.flush(); descriptors.flush()
    persistent = formal / "feature_store"
    persistent_points = persistent / "points.npy"
    persistent_descriptors = persistent / "descriptors.npy"
    persistent_database = persistent / "geometry.sqlite3"
    points_sha256 = atomic_copy_with_sha256(points_path, persistent_points)
    descriptors_sha256 = atomic_copy_with_sha256(descriptors_path, persistent_descriptors)
    database_sha256 = atomic_copy_with_sha256(database_path, persistent_database)
    scratch_bindings = {
        "points": {
            "path": str(persistent_points),
            "bytes": persistent_points.stat().st_size,
            "sha256": points_sha256,
            "shape": list(points.shape),
            "dtype": str(points.dtype),
        },
        "descriptors": {
            "path": str(persistent_descriptors),
            "bytes": persistent_descriptors.stat().st_size,
            "sha256": descriptors_sha256,
            "shape": list(descriptors.shape),
            "dtype": str(descriptors.dtype),
        },
        "database": {
            "path": str(persistent_database),
            "bytes": persistent_database.stat().st_size,
            "sha256": database_sha256,
        },
    }
    summary = write_json(
        formal / "geometry_summary.json",
        {
            "schema_version": "table1_neardup_geometry_summary_v1",
            "dataset_key": spec.key,
            "dataset": spec.label,
            "status": "COMPLETE",
            "created_at_utc": utc_now(),
            "roster_compact_sha256": roster_hash,
            "n_total": len(assets),
            "n_evaluable": evaluated,
            "coverage": evaluated / len(assets),
            "category_count": categories,
            "evaluable_category_count": valid_categories,
            "status_counts": status_counts,
            "retried_asset_count": retry_count,
            "transient_exhausted_count": transient_exhausted,
            "error_review_required": int(status_counts.get("ERROR", 0)) > 0,
            "graph_protocol": GRAPH_PROTOCOL,
            "point_protocol": POINT_PROTOCOL,
            "point_count": point_count,
            "descriptor_protocol": DESCRIPTOR_PROTOCOL,
            "descriptor_dim": DESCRIPTOR_DIM,
            "geometry_records_sha256": sha256_file(records_path),
            "scratch_bindings": scratch_bindings,
            "feature_store": "persistent_formal_output",
            "scratch_points_path": str(persistent_points),
            "scratch_descriptors_path": str(persistent_descriptors),
            "scratch_database_path": str(persistent_database),
        },
        self_hash="summary_content_sha256",
    )
    return summary


def build_candidates_dataset(
    spec: DatasetSpec,
    output: Path,
    scratch: Path,
    *,
    top_k: int,
    exhaustive_limit: int,
) -> dict[str, Any]:
    formal = output / "near_duplicate" / spec.key
    local = scratch / spec.key
    summary = read_self_hashed_json(
        formal / "geometry_summary.json", "summary_content_sha256"
    )
    verify_scratch_bindings(summary)
    descriptors = np.load(Path(summary["scratch_descriptors_path"]), mmap_mode="r")
    connection = _db_readonly(Path(summary["scratch_database_path"]))
    groups: dict[tuple[str, str], list[int]] = defaultdict(list)
    for ordinal, category, graph_hash in connection.execute(
        "SELECT ordinal,category,graph_hash FROM assets WHERE status='EVALUATED' ORDER BY ordinal"
    ):
        groups[(str(category), str(graph_hash))].append(int(ordinal))
    group_items = sorted(groups.items())
    target = formal / "candidate_pairs.bin"
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_name(f".{target.name}.tmp")
    checkpoint_path = formal / "candidate_checkpoint.json"
    checkpoint_binding = {
        "dataset_key": spec.key,
        "candidate_protocol": CANDIDATE_PROTOCOL,
        "top_k": top_k,
        "exhaustive_group_limit": exhaustive_limit,
        "eligible_group_count": len(group_items),
        "geometry_summary_sha256": sha256_file(formal / "geometry_summary.json"),
        "descriptors_sha256": summary["scratch_bindings"]["descriptors"]["sha256"],
    }

    completed_groups = pair_count = group_with_pairs = 0
    digest = hashlib.sha256()
    completed_target = False
    if checkpoint_path.is_file():
        checkpoint = read_self_hashed_json(
            checkpoint_path, "checkpoint_content_sha256"
        )
        for field, value in checkpoint_binding.items():
            if checkpoint.get(field) != value:
                raise MetricError(f"{spec.key} candidate checkpoint binding mismatch: {field}")
        completed_groups = int(checkpoint.get("completed_group_count", -1))
        pair_count = int(checkpoint.get("candidate_pair_count", -1))
        group_with_pairs = int(checkpoint.get("groups_with_pairs", -1))
        if not (
            0 <= completed_groups <= len(group_items)
            and pair_count >= 0
            and 0 <= group_with_pairs <= completed_groups
        ):
            raise MetricError(f"{spec.key} candidate checkpoint progress is invalid")
        committed_bytes = pair_count * PAIR_STRUCT.size
        if checkpoint.get("state") == "COMPLETE":
            if (
                not target.is_file()
                or target.stat().st_size != committed_bytes
                or sha256_file(target) != checkpoint.get("candidate_pairs_sha256")
            ):
                raise MetricError(f"{spec.key} completed candidate checkpoint drift")
            completed_target = True
        else:
            resume_path = temporary
            if not resume_path.is_file() and completed_groups == len(group_items) and target.is_file():
                resume_path = target
            if not resume_path.is_file() or resume_path.stat().st_size < committed_bytes:
                raise MetricError(f"{spec.key} candidate checkpoint bytes are missing")
            if resume_path.stat().st_size > committed_bytes:
                with resume_path.open("r+b") as handle:
                    handle.truncate(committed_bytes)
                    handle.flush(); os.fsync(handle.fileno())
            with resume_path.open("rb") as handle:
                for block in iter(lambda: handle.read(4 * 1024 * 1024), b""):
                    digest.update(block)
            if digest.hexdigest() != checkpoint.get("candidate_prefix_sha256"):
                raise MetricError(f"{spec.key} candidate checkpoint prefix hash drift")
            if resume_path == target:
                completed_target = True
    elif temporary.exists():
        # There is no committed JSON marker, so no byte in the temporary file
        # belongs to the resume contract.
        with temporary.open("wb") as handle:
            handle.flush(); os.fsync(handle.fileno())

    if not completed_target:
        mode = "ab" if completed_groups else "wb"
        with temporary.open(mode) as handle:
            for group_index in range(completed_groups, len(group_items)):
                (_, ordinals) = group_items[group_index]
                local_pairs = (
                    candidate_pairs_for_group(
                        descriptors[ordinals], top_k, exhaustive_limit
                    )
                    if len(ordinals) >= 2 else []
                )
                if local_pairs:
                    group_with_pairs += 1
                for left, right, distance in local_pairs:
                    packed = PAIR_STRUCT.pack(ordinals[left], ordinals[right], distance)
                    handle.write(packed)
                    digest.update(packed)
                    pair_count += 1
                completed_groups = group_index + 1
                if completed_groups % 100 == 0 or completed_groups == len(group_items):
                    handle.flush(); os.fsync(handle.fileno())
                    write_json(
                        checkpoint_path,
                        {
                            "schema_version": "table1_neardup_candidate_checkpoint_v1",
                            **checkpoint_binding,
                            "state": "IN_PROGRESS",
                            "completed_group_count": completed_groups,
                            "candidate_pair_count": pair_count,
                            "groups_with_pairs": group_with_pairs,
                            "candidate_prefix_sha256": digest.hexdigest(),
                            "updated_at_utc": utc_now(),
                        },
                        self_hash="checkpoint_content_sha256",
                    )
                    print(
                        f"[candidates] {spec.key}: groups={completed_groups}/{len(groups)} "
                        f"pairs={pair_count}",
                        flush=True,
                    )
        os.replace(temporary, target)
    target_sha256 = sha256_file(target)
    write_json(
        checkpoint_path,
        {
            "schema_version": "table1_neardup_candidate_checkpoint_v1",
            **checkpoint_binding,
            "state": "COMPLETE",
            "completed_group_count": len(group_items),
            "candidate_pair_count": pair_count,
            "groups_with_pairs": group_with_pairs,
            "candidate_prefix_sha256": target_sha256,
            "candidate_pairs_sha256": target_sha256,
            "updated_at_utc": utc_now(),
        },
        self_hash="checkpoint_content_sha256",
    )
    result = write_json(
        formal / "candidate_summary.json",
        {
            "schema_version": "table1_neardup_candidate_summary_v1",
            "dataset_key": spec.key,
            "status": "COMPLETE",
            "created_at_utc": utc_now(),
            "candidate_protocol": CANDIDATE_PROTOCOL,
            "top_k": top_k,
            "exhaustive_group_limit": exhaustive_limit,
            "eligible_group_count": len(group_items),
            "groups_with_pairs": group_with_pairs,
            "candidate_pair_count": pair_count,
            "candidate_pair_record_bytes": PAIR_STRUCT.size,
            "candidate_pairs_path": str(target),
            "candidate_pairs_sha256": target_sha256,
            "candidate_checkpoint_sha256": sha256_file(checkpoint_path),
            "geometry_summary_sha256": sha256_file(formal / "geometry_summary.json"),
        },
        self_hash="summary_content_sha256",
    )
    connection.close()
    return result


def _pair_id(dataset: str, left_id: str, right_id: str) -> str:
    left_id, right_id = sorted((left_id, right_id))
    digest = hashlib.sha256(
        f"{CALIBRATION_PROTOCOL}|{dataset}|{left_id}|{right_id}".encode()
    ).hexdigest()
    return f"pair_{digest[:24]}"


def _lowest_candidates(path: Path, count: int) -> list[tuple[int, int, float]]:
    if count <= 0 or path.stat().st_size == 0:
        return []
    values = np.memmap(path, dtype=PAIR_DTYPE, mode="r")
    heap: list[tuple[float, int, int]] = []
    for row in values:
        item = (-float(row["distance"]), int(row["left"]), int(row["right"]))
        if len(heap) < count:
            heapq.heappush(heap, item)
        elif item > heap[0]:
            heapq.heapreplace(heap, item)
    return sorted((left, right, -negative) for negative, left, right in heap)


def _random_same_gate_pairs(connection: sqlite3.Connection, count: int, dataset: str) -> list[tuple[int, int, float]]:
    groups: dict[tuple[str, str], list[int]] = defaultdict(list)
    for ordinal, category, graph_hash in connection.execute(
        "SELECT ordinal,category,graph_hash FROM assets WHERE status='EVALUATED' ORDER BY ordinal"
    ):
        groups[(str(category), str(graph_hash))].append(int(ordinal))
    heap: list[tuple[int, int, int]] = []
    for key, ordinals in sorted(groups.items()):
        if len(ordinals) < 2:
            continue
        attempts = min(len(ordinals), 8)
        for offset in range(attempts):
            digest = hashlib.sha256(
                f"{CALIBRATION_PROTOCOL}|random|{dataset}|{key}|{offset}".encode()
            ).digest()
            left = ordinals[int.from_bytes(digest[:8], "big") % len(ordinals)]
            right = ordinals[int.from_bytes(digest[8:16], "big") % len(ordinals)]
            if left == right:
                right = ordinals[(ordinals.index(left) + 1) % len(ordinals)]
            left, right = min(left, right), max(left, right)
            rank = int.from_bytes(hashlib.sha256(f"{dataset}|{left}|{right}".encode()).digest(), "big")
            item = (-rank, left, right)
            if len(heap) < count:
                heapq.heappush(heap, item)
            elif item > heap[0]:
                heapq.heapreplace(heap, item)
    return sorted((left, right, math.nan) for _, left, right in heap)


def _project(points: np.ndarray, yaw: float = -0.7, pitch: float = 0.45) -> np.ndarray:
    cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    rz = np.asarray(((cy, -sy, 0), (sy, cy, 0), (0, 0, 1)), dtype=np.float64)
    rx = np.asarray(((1, 0, 0), (0, cp, -sp), (0, sp, cp)), dtype=np.float64)
    return np.asarray(points, dtype=np.float64) @ (rx @ rz).T


def render_pair_preview(left: np.ndarray, right: np.ndarray, path: Path) -> None:
    width, height, panel = 720, 480, 240
    image = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(image)

    def paint(
        points: np.ndarray, x_offset: int, y_offset: int,
        color: tuple[int, int, int],
    ) -> None:
        xy = points[:, :2]
        pixels = np.empty((len(xy), 2), dtype=int)
        pixels[:, 0] = np.rint((xy[:, 0] + 0.55) * (panel - 24) + x_offset + 12)
        pixels[:, 1] = np.rint((0.55 - xy[:, 1]) * (panel - 24) + y_offset + 12)
        pixels[:, 0] = np.clip(pixels[:, 0], x_offset + 2, x_offset + panel - 3)
        pixels[:, 1] = np.clip(pixels[:, 1], y_offset + 2, y_offset + panel - 3)
        for x, y in pixels[::max(1, len(pixels) // 1800)]:
            draw.ellipse((x - 1, y - 1, x + 1, y + 1), fill=color)

    for view_index, (yaw, pitch) in enumerate(((-0.7, 0.45), (0.9, 0.2))):
        y_offset = view_index * panel
        projected_left = _project(left, yaw=yaw, pitch=pitch)
        projected_right = _project(right, yaw=yaw, pitch=pitch)
        paint(projected_left, 0, y_offset, (190, 55, 45))
        paint(projected_right, panel, y_offset, (45, 95, 185))
        paint(projected_left, panel * 2, y_offset, (205, 75, 55))
        paint(projected_right, panel * 2, y_offset, (45, 105, 195))
        draw.text((8, y_offset + 6), f"A view {view_index + 1}", fill=(25, 25, 25))
        draw.text((panel + 8, y_offset + 6), f"B view {view_index + 1}", fill=(25, 25, 25))
        draw.text((panel * 2 + 8, y_offset + 6), "overlay", fill=(25, 25, 25))
    path.parent.mkdir(parents=True, exist_ok=True)
    image.save(path, format="PNG", optimize=True)


def build_annotation_packet(
    specs: Sequence[DatasetSpec], output: Path, scratch: Path, *, target_pairs: int
) -> dict[str, Any]:
    available = [
        spec for spec in specs
        if (output / "near_duplicate" / spec.key / "candidate_summary.json").is_file()
    ]
    if not available:
        raise MetricError("no completed candidate datasets are available")
    base_quota, remainder = divmod(target_pairs, len(available))
    tasks: list[dict[str, Any]] = []
    audits: list[dict[str, Any]] = []
    previews: list[dict[str, Any]] = []
    dataset_bindings: dict[str, dict[str, str]] = {}
    pools: dict[str, list[tuple[int, int, str]]] = {}
    for dataset_index, spec in enumerate(available):
        quota = base_quota + int(dataset_index < remainder)
        formal = output / "near_duplicate" / spec.key
        geometry = read_self_hashed_json(
            formal / "geometry_summary.json", "summary_content_sha256"
        )
        verify_scratch_bindings(geometry)
        candidates = read_self_hashed_json(
            formal / "candidate_summary.json", "summary_content_sha256"
        )
        verify_candidate_bindings(formal, candidates)
        dataset_bindings[spec.key] = {
            "geometry_summary_sha256": sha256_file(formal / "geometry_summary.json"),
            "candidate_summary_sha256": sha256_file(formal / "candidate_summary.json"),
            "candidate_pairs_sha256": str(candidates["candidate_pairs_sha256"]),
        }
        connection = _db_readonly(Path(geometry["scratch_database_path"]))
        hard_count = math.ceil(quota * 0.6)
        hard = _lowest_candidates(
            Path(candidates["candidate_pairs_path"]), max(target_pairs, hard_count)
        )
        selected = [
            (left, right, "hard_retrieval")
            for left, right, _ in hard[:hard_count]
        ]
        selected.extend(
            (left, right, "random_same_category_graph")
            for left, right, _ in _random_same_gate_pairs(
                connection, target_pairs * 2, spec.key
            )
        )
        selected.extend(
            (left, right, "hard_retrieval_redistribution")
            for left, right, _ in hard[hard_count:]
        )
        deduplicated: dict[tuple[int, int], str] = {}
        for left, right, mode in selected:
            deduplicated.setdefault((left, right), mode)
        pools[spec.key] = [
            (left, right, mode)
            for (left, right), mode in deduplicated.items()
        ]
        connection.close()

    selected_by_dataset: dict[str, list[tuple[int, int, str]]] = {}
    cursors: dict[str, int] = {}
    for dataset_index, spec in enumerate(available):
        quota = base_quota + int(dataset_index < remainder)
        initial = min(quota, len(pools[spec.key]))
        selected_by_dataset[spec.key] = pools[spec.key][:initial]
        cursors[spec.key] = initial
    remaining = target_pairs - sum(len(rows) for rows in selected_by_dataset.values())
    while remaining:
        progressed = False
        for spec in available:
            cursor = cursors[spec.key]
            if cursor >= len(pools[spec.key]):
                continue
            selected_by_dataset[spec.key].append(pools[spec.key][cursor])
            cursors[spec.key] += 1
            remaining -= 1
            progressed = True
            if not remaining:
                break
        if not progressed:
            available_pairs = sum(len(rows) for rows in pools.values())
            raise MetricError(
                f"only {available_pairs} distinct gated calibration pairs exist; "
                f"requested {target_pairs}"
            )

    for spec in available:
        formal = output / "near_duplicate" / spec.key
        geometry = read_self_hashed_json(
            formal / "geometry_summary.json", "summary_content_sha256"
        )
        points = np.load(Path(geometry["scratch_points_path"]), mmap_mode="r")
        connection = _db_readonly(Path(geometry["scratch_database_path"]))
        by_ordinal = {
            int(row[0]): {
                "asset_id": str(row[1]), "category": str(row[2]),
                "urdf_path": str(row[3]), "graph_hash": str(row[4]),
            }
            for row in connection.execute(
                "SELECT ordinal,asset_id,category,urdf_path,graph_hash "
                "FROM assets WHERE status='EVALUATED'"
            )
        }
        for left, right, mode in selected_by_dataset[spec.key]:
            left_row, right_row = by_ordinal[left], by_ordinal[right]
            if left_row["category"] != right_row["category"] or left_row["graph_hash"] != right_row["graph_hash"]:
                raise MetricError("annotation pair crossed the category/graph gate")
            distance = symmetric_chamfer(points[left], points[right])
            pair_id = _pair_id(spec.key, left_row["asset_id"], right_row["asset_id"])
            preview = output / "near_duplicate/calibration/previews" / f"{pair_id}.png"
            render_pair_preview(points[left], points[right], preview)
            task = {
                "pair_id": pair_id,
                "dataset_key": spec.key,
                "category": left_row["category"],
                "left_asset_id": left_row["asset_id"],
                "right_asset_id": right_row["asset_id"],
                "preview_path": str(preview.relative_to(output)),
                "label": None,
                "reviewer_confidence": None,
                "notes": None,
            }
            tasks.append(task)
            audits.append({
                **task,
                "selection_mode": mode,
                "graph_hash": left_row["graph_hash"],
                "left_ordinal": left,
                "right_ordinal": right,
                "left_urdf_path": left_row["urdf_path"],
                "right_urdf_path": right_row["urdf_path"],
                "chamfer_distance": distance,
            })
            previews.append({
                "pair_id": pair_id,
                "path": str(preview.relative_to(output)),
                "sha256": sha256_file(preview),
            })
        connection.close()
    tasks.sort(key=lambda row: row["pair_id"])
    audits.sort(key=lambda row: row["pair_id"])
    previews.sort(key=lambda row: row["pair_id"])
    target = output / "near_duplicate/calibration"
    write_jsonl(target / "annotation_tasks.jsonl", tasks)
    write_jsonl(target / "annotation_labels_template.jsonl", tasks)
    write_jsonl(target / "annotation_audit.jsonl", audits)
    write_jsonl(target / "preview_manifest.jsonl", previews)
    result = write_json(
        target / "annotation_packet.json",
        {
            "schema_version": "table1_neardup_annotation_packet_v1",
            "status": "AWAITING_HUMAN_LABELS",
            "created_at_utc": utc_now(),
            "calibration_protocol": CALIBRATION_PROTOCOL,
            "chamfer_protocol": CHAMFER_PROTOCOL,
            "allowed_labels": ["duplicate", "not_duplicate", "uncertain"],
            "task_count": len(tasks),
            "requested_task_count": target_pairs,
            "dataset_counts": {
                key: sum(row["dataset_key"] == key for row in tasks)
                for key in sorted({row["dataset_key"] for row in tasks})
            },
            "dataset_bindings": dataset_bindings,
            "selection_design": "equal dataset quota with deterministic deficit redistribution; 60% descriptor-hard target plus same-gate random fill",
            "estimation_limitation": "selection probabilities are not prevalence-weighted; calibration is diagnostic only",
            "run_manifest_sha256": sha256_file(output / "manifest.json"),
            "annotation_tasks_sha256": sha256_file(target / "annotation_tasks.jsonl"),
            "annotation_audit_sha256": sha256_file(target / "annotation_audit.jsonl"),
            "labels_template_sha256": sha256_file(target / "annotation_labels_template.jsonl"),
            "preview_manifest_sha256": sha256_file(target / "preview_manifest.jsonl"),
            "review_instruction": "Fill label with duplicate/not_duplicate/uncertain; do not edit pair_id or other fields.",
        },
        self_hash="packet_content_sha256",
    )
    return result


def _load_audit_by_pair(output: Path) -> dict[str, dict[str, Any]]:
    path = output / "near_duplicate/calibration/annotation_audit.jsonl"
    rows = list(iter_jsonl(path))
    result = {str(row["pair_id"]): row for row in rows}
    if len(result) != len(rows):
        raise MetricError("annotation audit pair IDs are not unique")
    return result


def _wilson_ci95(successes: int, trials: int) -> list[float] | None:
    if trials == 0:
        return None
    z = 1.959963984540054
    probability = successes / trials
    denominator = 1.0 + z * z / trials
    center = (probability + z * z / (2.0 * trials)) / denominator
    half = z * math.sqrt(
        probability * (1.0 - probability) / trials + z * z / (4.0 * trials * trials)
    ) / denominator
    return [max(0.0, center - half), min(1.0, center + half)]


def _precision_recall(rows: Sequence[tuple[float, bool]], tau: float) -> dict[str, Any]:
    tp = sum(distance < tau and label for distance, label in rows)
    fp = sum(distance < tau and not label for distance, label in rows)
    fn = sum(distance >= tau and label for distance, label in rows)
    precision = tp / (tp + fp) if tp + fp else None
    recall = tp / (tp + fn) if tp + fn else None
    return {
        "tp": tp,
        "fp": fp,
        "fn": fn,
        "precision": precision,
        "precision_ci95": _wilson_ci95(tp, tp + fp),
        "recall": recall,
        "recall_ci95": _wilson_ci95(tp, tp + fn),
    }


def _asset_calibration_bucket(dataset: str, asset_id: str) -> int:
    digest = hashlib.sha256(
        f"{CALIBRATION_PROTOCOL}|asset-split|{dataset}|{asset_id}".encode()
    ).digest()
    return int.from_bytes(digest[:8], "big") % 10


def calibrate_threshold(
    output: Path,
    labels_path: Path,
    *,
    target_precision: float,
    min_resolved: int,
) -> dict[str, Any]:
    packet_path = output / "near_duplicate/calibration/annotation_packet.json"
    packet = read_self_hashed_json(packet_path, "packet_content_sha256")
    if packet.get("run_manifest_sha256") != sha256_file(output / "manifest.json"):
        raise MetricError("annotation packet run-manifest binding mismatch")
    audit_path = output / "near_duplicate/calibration/annotation_audit.jsonl"
    if packet.get("annotation_audit_sha256") != sha256_file(audit_path):
        raise MetricError("annotation packet audit binding mismatch")
    tasks_path = output / "near_duplicate/calibration/annotation_tasks.jsonl"
    if packet.get("annotation_tasks_sha256") != sha256_file(tasks_path):
        raise MetricError("annotation packet task binding mismatch")
    audit = _load_audit_by_pair(output)
    task_rows = list(iter_jsonl(tasks_path))
    task_ids = [str(row.get("pair_id")) for row in task_rows]
    if len(set(task_ids)) != len(task_ids) or set(task_ids) != set(audit):
        raise MetricError("annotation task/audit pair IDs are not identical and unique")
    labels = list(iter_jsonl(labels_path))
    label_ids = [str(row.get("pair_id")) for row in labels]
    if len(set(label_ids)) != len(label_ids):
        raise MetricError("label file pair IDs are not unique")
    missing = sorted(set(task_ids) - set(label_ids))
    foreign = sorted(set(label_ids) - set(task_ids))
    if missing or foreign:
        raise MetricError(
            "label file must contain every annotation task exactly once; "
            f"missing={missing[:3]} foreign={foreign[:3]}"
        )
    resolved: list[tuple[str, float, bool]] = []
    uncertain = 0
    for row in labels:
        pair_id = str(row.get("pair_id"))
        label = row.get("label")
        if label == "uncertain":
            uncertain += 1
            continue
        if label not in {"duplicate", "not_duplicate"}:
            raise MetricError(f"invalid label for {pair_id}: {label!r}")
        resolved.append((pair_id, float(audit[pair_id]["chamfer_distance"]), label == "duplicate"))
    if len(resolved) < min_resolved:
        raise MetricError(f"only {len(resolved)} resolved labels; require at least {min_resolved}")
    train: list[tuple[float, bool]] = []
    heldout: list[tuple[float, bool]] = []
    split_excluded = 0
    for pair_id, distance, label in resolved:
        row = audit[pair_id]
        dataset = str(row["dataset_key"])
        left_bucket = _asset_calibration_bucket(dataset, str(row["left_asset_id"]))
        right_bucket = _asset_calibration_bucket(dataset, str(row["right_asset_id"]))
        if left_bucket < 4 and right_bucket < 4:
            heldout.append((distance, label))
        elif left_bucket >= 4 and right_bucket >= 4:
            train.append((distance, label))
        else:
            split_excluded += 1
    if (
        not train
        or not heldout
        or not any(label for _, label in train)
        or not any(not label for _, label in train)
        or not any(label for _, label in heldout)
        or not any(not label for _, label in heldout)
    ):
        raise MetricError(
            "calibration labels do not support an asset-disjoint train/heldout assessment with both classes"
        )
    candidates = sorted({math.nextafter(distance, math.inf) for distance, _ in train})
    viable: list[tuple[float, float, int]] = []
    for tau in candidates:
        metrics = _precision_recall(train, tau)
        predicted = int(metrics["tp"] + metrics["fp"])
        if predicted >= 20 and metrics["precision"] is not None and metrics["precision"] >= target_precision:
            viable.append((float(metrics["recall"] or 0.0), tau, predicted))
    if not viable:
        raise MetricError("no threshold meets target precision with at least 20 train predictions")
    # Among thresholds with the same train recall, retain the smallest one.
    # This is the conservative high-precision tie-break and is frozen in the
    # receipt rather than expanding tau until the precision budget is spent.
    _, tau, _ = max(viable, key=lambda item: (item[0], -item[1]))
    train_metrics = _precision_recall(train, tau)
    heldout_metrics = _precision_recall(heldout, tau)
    passed = (
        heldout_metrics["precision"] is not None
        and heldout_metrics["precision"] >= target_precision
        and heldout_metrics["tp"] + heldout_metrics["fp"] >= 10
    )
    completed_labels_path = output / "near_duplicate/calibration/completed_labels.jsonl"
    labels_sha256 = atomic_copy_with_sha256(labels_path, completed_labels_path)
    receipt = write_json(
        output / "near_duplicate/calibration/threshold_receipt.json",
        {
            "schema_version": "table1_neardup_threshold_receipt_v1",
            "classification": "FULL_RELEASE_DESCRIPTIVE_DIAGNOSTIC",
            "status": "PASS" if passed else "FAILED_HELDOUT_PRECISION",
            "created_at_utc": utc_now(),
            "calibration_protocol": CALIBRATION_PROTOCOL,
            "chamfer_protocol": CHAMFER_PROTOCOL,
            "tau": tau,
            "comparison": "distance < tau",
            "target_precision": target_precision,
            "precision_gate_basis": "heldout point estimate; CI reported but diagnostic run is not headline eligible",
            "split_policy": "asset-hash buckets 0-3 heldout, 4-9 train; cross-bucket pairs excluded",
            "selection_design": packet.get("selection_design"),
            "estimation_limitation": packet.get("estimation_limitation"),
            "submitted_label_count": len(labels),
            "resolved_label_count": len(resolved),
            "uncertain_or_unlabeled_count": uncertain,
            "train_count": len(train),
            "heldout_count": len(heldout),
            "split_excluded_count": split_excluded,
            "train_metrics": train_metrics,
            "heldout_metrics": heldout_metrics,
            "labels_path": str(completed_labels_path.resolve()),
            "labels_sha256": labels_sha256,
            "annotation_audit_sha256": sha256_file(audit_path),
            "annotation_packet_sha256": sha256_file(packet_path),
            "run_manifest_sha256": sha256_file(output / "manifest.json"),
        },
        self_hash="receipt_content_sha256",
    )
    if not passed:
        raise MetricError("held-out calibration did not meet the frozen precision gate")
    write_json(
        output / "status.json",
        {
            "schema_version": "table1_diversity_status_v1",
            "status": "CALIBRATED_DIAGNOSTIC_READY_TO_SCORE",
            "updated_at_utc": utc_now(),
            "variance": "COMPLETE",
            "near_duplicate": "CALIBRATED",
            "threshold_receipt_sha256": sha256_file(
                output / "near_duplicate/calibration/threshold_receipt.json"
            ),
        },
        self_hash="status_content_sha256",
    )
    return receipt


def score_dataset(
    spec: DatasetSpec,
    output: Path,
    *,
    tau: float,
    threshold_receipt_sha256: str,
) -> dict[str, Any]:
    formal = output / "near_duplicate" / spec.key
    geometry = read_self_hashed_json(
        formal / "geometry_summary.json", "summary_content_sha256"
    )
    verify_scratch_bindings(geometry)
    candidates = read_self_hashed_json(
        formal / "candidate_summary.json", "summary_content_sha256"
    )
    verify_candidate_bindings(formal, candidates)
    points = np.load(Path(geometry["scratch_points_path"]), mmap_mode="r")
    connection = _db_readonly(Path(geometry["scratch_database_path"]))
    evaluable_rows = [
        (int(row[0]), str(row[1]))
        for row in connection.execute(
            "SELECT ordinal,category FROM assets WHERE status='EVALUATED' ORDER BY ordinal"
        )
    ]
    evaluable_ordinals = [row[0] for row in evaluable_rows]
    evaluable_categories = [row[1] for row in evaluable_rows]
    dense = {ordinal: index for index, ordinal in enumerate(evaluable_ordinals)}
    source = Path(candidates["candidate_pairs_path"])
    pairs = np.memmap(source, dtype=PAIR_DTYPE, mode="r") if source.stat().st_size else []
    destination = formal / "chamfer_pairs.bin"
    temporary = destination.with_name(f".{destination.name}.tmp")
    checkpoint_path = formal / "score_checkpoint.json"
    positives: list[tuple[int, int]] = []

    def consume_prefix(path: Path, completed: int) -> None:
        if path.stat().st_size != completed * CHAMFER_STRUCT.size:
            raise MetricError(f"{spec.key} score checkpoint byte count mismatch")
        if not completed:
            return
        frozen = np.memmap(path, dtype=CHAMFER_DTYPE, mode="r", shape=(completed,))
        for index, row in enumerate(frozen):
            source_row = pairs[index]
            left, right = int(row["left"]), int(row["right"])
            if left != int(source_row["left"]) or right != int(source_row["right"]):
                raise MetricError(f"{spec.key} score checkpoint candidate identity mismatch")
            distance = float(row["distance"])
            if not math.isfinite(distance) or distance < 0:
                raise MetricError(f"{spec.key} score checkpoint has an invalid distance")
            if distance < tau:
                positives.append((dense[left], dense[right]))

    completed = 0
    if destination.is_file():
        if destination.stat().st_size != len(pairs) * CHAMFER_STRUCT.size:
            raise MetricError(f"{spec.key} completed Chamfer artifact has the wrong size")
        consume_prefix(destination, len(pairs))
        completed = len(pairs)
    else:
        if temporary.exists() != checkpoint_path.exists():
            raise MetricError(f"{spec.key} score resume artifacts are incomplete")
        if checkpoint_path.exists():
            checkpoint = read_self_hashed_json(checkpoint_path, "checkpoint_content_sha256")
            expected = {
                "dataset_key": spec.key,
                "tau": tau,
                "threshold_receipt_sha256": threshold_receipt_sha256,
                "candidate_pairs_sha256": candidates["candidate_pairs_sha256"],
                "candidate_pair_count": len(pairs),
                "chamfer_protocol": CHAMFER_PROTOCOL,
            }
            for field, value in expected.items():
                if checkpoint.get(field) != value:
                    raise MetricError(f"{spec.key} score resume binding mismatch: {field}")
            completed = int(checkpoint.get("completed_pair_count", -1))
            if not 0 <= completed <= len(pairs):
                raise MetricError(f"{spec.key} score checkpoint progress is invalid")
            committed_bytes = completed * CHAMFER_STRUCT.size
            observed_bytes = temporary.stat().st_size
            if observed_bytes < committed_bytes or observed_bytes % CHAMFER_STRUCT.size:
                raise MetricError(f"{spec.key} score checkpoint byte count is invalid")
            if observed_bytes > committed_bytes:
                # Data fsynced after the last committed JSON checkpoint is not
                # part of the resume contract. Roll back to that atomic boundary.
                with temporary.open("r+b") as handle:
                    handle.truncate(committed_bytes)
                    handle.flush(); os.fsync(handle.fileno())
            consume_prefix(temporary, completed)
        else:
            with temporary.open("wb") as handle:
                handle.flush(); os.fsync(handle.fileno())
            write_json(
                checkpoint_path,
                {
                    "schema_version": "table1_neardup_score_checkpoint_v1",
                    "dataset_key": spec.key,
                    "tau": tau,
                    "threshold_receipt_sha256": threshold_receipt_sha256,
                    "candidate_pairs_sha256": candidates["candidate_pairs_sha256"],
                    "candidate_pair_count": len(pairs),
                    "completed_pair_count": 0,
                    "chamfer_protocol": CHAMFER_PROTOCOL,
                    "updated_at_utc": utc_now(),
                },
                self_hash="checkpoint_content_sha256",
            )
        mode = "ab"
        with temporary.open(mode) as handle:
            for offset in range(completed, len(pairs)):
                row = pairs[offset]
                left, right = int(row["left"]), int(row["right"])
                distance = symmetric_chamfer(points[left], points[right])
                handle.write(CHAMFER_STRUCT.pack(left, right, distance))
                if distance < tau:
                    positives.append((dense[left], dense[right]))
                completed = offset + 1
                if completed % 1000 == 0 or completed == len(pairs):
                    handle.flush(); os.fsync(handle.fileno())
                    write_json(
                        checkpoint_path,
                        {
                            "schema_version": "table1_neardup_score_checkpoint_v1",
                            "dataset_key": spec.key,
                            "tau": tau,
                            "threshold_receipt_sha256": threshold_receipt_sha256,
                            "candidate_pairs_sha256": candidates["candidate_pairs_sha256"],
                            "candidate_pair_count": len(pairs),
                            "completed_pair_count": completed,
                            "chamfer_protocol": CHAMFER_PROTOCOL,
                            "updated_at_utc": utc_now(),
                        },
                        self_hash="checkpoint_content_sha256",
                    )
                    print(f"[score] {spec.key}: {completed}/{len(pairs)}", flush=True)
        os.replace(temporary, destination)
        write_json(
            checkpoint_path,
            {
                "schema_version": "table1_neardup_score_checkpoint_v1",
                "dataset_key": spec.key,
                "tau": tau,
                "threshold_receipt_sha256": threshold_receipt_sha256,
                "candidate_pairs_sha256": candidates["candidate_pairs_sha256"],
                "candidate_pair_count": len(pairs),
                "completed_pair_count": len(pairs),
                "chamfer_protocol": CHAMFER_PROTOCOL,
                "state": "COMPLETE",
                "chamfer_pairs_sha256": sha256_file(destination),
                "updated_at_utc": utc_now(),
            },
            self_hash="checkpoint_content_sha256",
        )
    rates = duplicate_rates(len(evaluable_ordinals), positives)
    category_summary, category_rows = category_macro_duplicate_rates(
        evaluable_categories,
        positives,
        seed_key=f"{RATE_PROTOCOL}|bootstrap|{spec.key}",
    )
    category_path = formal / "score_categories.jsonl"
    write_jsonl(category_path, category_rows)
    rates.update({
        "schema_version": "table1_neardup_score_v1",
        "dataset_key": spec.key,
        "dataset": spec.label,
        "status": "COMPLETE",
        "created_at_utc": utc_now(),
        "n_requested": spec.expected_n,
        "coverage": len(evaluable_ordinals) / spec.expected_n,
        "positive_candidate_edge_count": len(positives),
        "candidate_pair_count": len(pairs),
        "tau": tau,
        "chamfer_protocol": CHAMFER_PROTOCOL,
        "rate_protocol": RATE_PROTOCOL,
        "classification": "FULL_RELEASE_DESCRIPTIVE_DIAGNOSTIC",
        "table1_headline_eligible": False,
        "retrieval_recall_status": "NOT_ESTABLISHED",
        "rate_interpretation": "REQUESTED_DENOMINATOR_CANDIDATE_DETECTED_LOWER_BOUND_WITH_EVALUABLE_CONDITIONAL_DIAGNOSTICS",
        "geometry_summary_sha256": sha256_file(formal / "geometry_summary.json"),
        "candidate_summary_sha256": sha256_file(formal / "candidate_summary.json"),
        "candidate_pairs_sha256": candidates["candidate_pairs_sha256"],
        "threshold_receipt_sha256": threshold_receipt_sha256,
        "chamfer_pairs_sha256": sha256_file(destination),
        "cluster_excess_rate_requested": rates["duplicate_excess_count"] / spec.expected_n,
        "candidate_detected_excess_over_requested_lower_bound": rates[
            "duplicate_excess_count"
        ] / spec.expected_n,
        "candidate_detected_cluster_excess_rate_on_evaluable_assets": rates[
            "cluster_excess_rate"
        ],
        "candidate_detected_category_macro_rate_on_evaluable_assets": category_summary[
            "category_macro_cluster_excess_rate"
        ],
        "category_macro": category_summary,
        "score_categories_sha256": sha256_file(category_path),
    })
    connection.close()
    return write_json(formal / "score.json", rates, self_hash="score_content_sha256")


def score_all(specs: Sequence[DatasetSpec], output: Path) -> dict[str, Any]:
    receipt_path = output / "near_duplicate/calibration/threshold_receipt.json"
    if not receipt_path.is_file():
        raise MetricError("threshold receipt is required; run calibrate after human labeling")
    receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    unsigned = dict(receipt); declared = unsigned.pop("receipt_content_sha256", None)
    if declared != canonical_sha256(unsigned) or receipt.get("status") != "PASS":
        raise MetricError("threshold receipt is invalid or did not pass held-out calibration")
    if receipt.get("chamfer_protocol") != CHAMFER_PROTOCOL:
        raise MetricError("threshold receipt Chamfer protocol mismatch")
    if receipt.get("run_manifest_sha256") != sha256_file(output / "manifest.json"):
        raise MetricError("threshold receipt run-manifest binding mismatch")
    receipt_sha256 = sha256_file(receipt_path)
    results = [
        score_dataset(
            spec,
            output,
            tau=float(receipt["tau"]),
            threshold_receipt_sha256=receipt_sha256,
        )
        for spec in specs
    ]
    result = write_json(
        output / "near_duplicate/summary.json",
        {
            "schema_version": "table1_neardup_summary_v1",
            "status": "COMPLETE",
            "classification": "FULL_RELEASE_DESCRIPTIVE_DIAGNOSTIC_NOT_MATCHED_COHORT",
            "table1_headline_eligible": False,
            "retrieval_recall_status": "NOT_ESTABLISHED",
            "rate_interpretation": "REQUESTED_DENOMINATOR_CANDIDATE_DETECTED_LOWER_BOUND_WITH_EVALUABLE_CONDITIONAL_DIAGNOSTICS",
            "created_at_utc": utc_now(),
            "threshold_receipt_sha256": receipt_sha256,
            "diagnostic_rate": "candidate_detected_excess_over_requested_lower_bound",
            "headline_blocker": "candidate retrieval recall and matched canonical-category cohort are not established",
            "datasets": results,
        },
        self_hash="summary_content_sha256",
    )
    write_json(
        output / "status.json",
        {
            "schema_version": "table1_diversity_status_v1",
            "status": "DIAGNOSTIC_SCORE_COMPLETE",
            "updated_at_utc": utc_now(),
            "variance": "COMPLETE",
            "near_duplicate": "COMPLETE_DESCRIPTIVE_DIAGNOSTIC",
            "near_duplicate_summary_sha256": sha256_file(
                output / "near_duplicate/summary.json"
            ),
        },
        self_hash="status_content_sha256",
    )
    return result


def _manifest_config(args: argparse.Namespace, specs: Sequence[DatasetSpec]) -> dict[str, Any]:
    def file_binding(path: Path) -> dict[str, Any]:
        resolved = path.resolve(strict=True)
        return {
            "path": str(resolved),
            "bytes": resolved.stat().st_size,
            "sha256": sha256_file(resolved),
        }

    fingerprint_core = SCRIPT_PATH.with_name("run_table1_artiverse.py")
    pva_receipts: dict[str, dict[str, Any]] = {}
    if args.pva_root_override:
        mirror_root = args.pva_root_override.resolve().parent
        for name in ("manifest.json", "artifact_manifest.json", "archive_records.jsonl"):
            receipt = mirror_root / name
            if receipt.is_file():
                pva_receipts[name] = file_binding(receipt)
    return {
        "schema_version": RUN_SCHEMA,
        "classification": "FULL_RELEASE_DESCRIPTIVE_DIAGNOSTIC",
        "table1_headline_eligible": False,
        "headline_blockers": [
            "no frozen cross-dataset canonical category mapping or C_common roster",
            "full-release cohorts do not implement matched n=20 with five resamples",
            "near-duplicate threshold requires human calibration",
            "candidate-retrieval recall is not established",
            "canonical cross-dataset categories and canonical visual-role graph are not frozen",
        ],
        "category_policy": "dataset-local frozen release label; SketchMobility uses {source}/{category}",
        "script_path": str(SCRIPT_PATH),
        "script_sha256": sha256_file(SCRIPT_PATH),
        "fingerprint_core": file_binding(fingerprint_core),
        "python_executable": sys.executable,
        "dependency_versions": {
            "python": platform.python_version(),
            "numpy": np.__version__,
            "scipy": scipy.__version__,
            "trimesh": trimesh.__version__,
        },
        "dataset_keys": [spec.key for spec in specs],
        "dataset_expected_n": {spec.key: spec.expected_n for spec in specs},
        "dataset_inputs": {
            spec.key: {
                "records": file_binding(spec.records),
                "roster": file_binding(spec.roster),
            }
            for spec in specs
        },
        "workers": args.workers,
        "point_count": args.point_count,
        "top_k": args.top_k,
        "exhaustive_group_limit": args.exhaustive_group_limit,
        "annotation_pairs": args.annotation_pairs,
        "target_precision": args.target_precision,
        "min_resolved_labels": args.min_resolved_labels,
        "scratch": str(args.scratch.resolve()),
        "pva_root_override": str(args.pva_root_override.resolve()) if args.pva_root_override else None,
        "pva_mirror_receipts": pva_receipts,
        "protocols": {
            "variance": VARIANCE_PROTOCOL,
            "graph": GRAPH_PROTOCOL,
            "points": POINT_PROTOCOL,
            "descriptor": DESCRIPTOR_PROTOCOL,
            "candidates": CANDIDATE_PROTOCOL,
            "chamfer": CHAMFER_PROTOCOL,
            "rate": RATE_PROTOCOL,
            "calibration": CALIBRATION_PROTOCOL,
        },
    }


def initialize_run(args: argparse.Namespace, specs: Sequence[DatasetSpec]) -> dict[str, Any]:
    output, scratch = args.output.resolve(), args.scratch.resolve()
    config = _manifest_config(args, specs)
    path = output / "manifest.json"
    if args.resume:
        if not path.is_file():
            raise MetricError("--resume requires an existing run manifest")
        existing = json.loads(path.read_text(encoding="utf-8"))
        unsigned = dict(existing); declared = unsigned.pop("manifest_content_sha256", None)
        if declared != canonical_sha256(unsigned):
            raise MetricError("run manifest self-hash mismatch")
        expected = {**config, "created_at_utc": existing.get("created_at_utc")}
        expected["manifest_content_sha256"] = canonical_sha256(expected)
        if existing != expected:
            differing = sorted(key for key in set(existing) | set(expected) if existing.get(key) != expected.get(key))
            raise MetricError(f"resume manifest mismatch: {', '.join(differing)}")
        return existing
    if output.exists() and any(output.iterdir()):
        raise MetricError(f"output is non-empty; use --resume only for the identical run: {output}")
    if scratch.exists() and any(scratch.iterdir()):
        raise MetricError(f"scratch is non-empty; choose a fresh path or use --resume: {scratch}")
    output.mkdir(parents=True, exist_ok=True)
    scratch.mkdir(parents=True, exist_ok=True)
    return write_json(path, {**config, "created_at_utc": utc_now()}, self_hash="manifest_content_sha256")


def run_prepare(args: argparse.Namespace, specs: Sequence[DatasetSpec]) -> None:
    output, scratch = args.output.resolve(), args.scratch.resolve()
    variance_path = output / "variance/summary.json"
    if not variance_path.is_file():
        run_variance(output, specs)
    else:
        read_self_hashed_json(variance_path, "summary_content_sha256")
    # Finish the ten smaller cohorts before the 302,440-asset PV-A release so
    # useful checkpointed coverage appears early in a long asynchronous run.
    ordered_specs = [spec for spec in specs if spec.key != "pva"] + [
        spec for spec in specs if spec.key == "pva"
    ]
    for spec in ordered_specs:
        print(f"[roster] loading {spec.key}", flush=True)
        assets, roster_hash = load_assets(spec, pva_root_override=args.pva_root_override)
        geometry_path = output / "near_duplicate" / spec.key / "geometry_summary.json"
        if not geometry_path.is_file():
            prepare_geometry_dataset(
                spec, assets, roster_hash, output, scratch,
                workers=args.workers, point_count=args.point_count,
            )
        else:
            summary = read_self_hashed_json(geometry_path, "summary_content_sha256")
            if summary.get("roster_compact_sha256") != roster_hash:
                raise MetricError(f"{spec.key} completed geometry is bound to another roster")
            records_path = geometry_path.with_name("geometry_records.jsonl")
            if sha256_file(records_path) != summary.get("geometry_records_sha256"):
                raise MetricError(f"{spec.key} completed geometry records have drifted")
            for field in ("scratch_points_path", "scratch_descriptors_path", "scratch_database_path"):
                if not Path(str(summary[field])).is_file():
                    raise MetricError(f"{spec.key} completed geometry scratch artifact is missing: {field}")
            verify_scratch_bindings(summary)
        candidate_path = output / "near_duplicate" / spec.key / "candidate_summary.json"
        if not candidate_path.is_file():
            build_candidates_dataset(
                spec, output, scratch, top_k=args.top_k,
                exhaustive_limit=args.exhaustive_group_limit,
            )
        else:
            candidate = read_self_hashed_json(candidate_path, "summary_content_sha256")
            verify_candidate_bindings(candidate_path.parent, candidate)
            expected_candidate_fields = {
                "dataset_key": spec.key,
                "candidate_protocol": CANDIDATE_PROTOCOL,
                "top_k": args.top_k,
                "exhaustive_group_limit": args.exhaustive_group_limit,
                "geometry_summary_sha256": sha256_file(geometry_path),
            }
            for field, value in expected_candidate_fields.items():
                if candidate.get(field) != value:
                    raise MetricError(
                        f"{spec.key} completed candidate binding drift: {field}"
                    )
    packet_path = output / "near_duplicate/calibration/annotation_packet.json"
    if not packet_path.is_file():
        build_annotation_packet(specs, output, scratch, target_pairs=args.annotation_pairs)
    else:
        packet = read_self_hashed_json(packet_path, "packet_content_sha256")
        calibration = packet_path.parent
        if packet.get("run_manifest_sha256") != sha256_file(output / "manifest.json"):
            raise MetricError("completed annotation packet manifest binding drift")
        expected_packet_keys = {spec.key for spec in specs}
        if set(packet.get("dataset_bindings", {})) != expected_packet_keys:
            raise MetricError("completed annotation packet dataset closure drift")
        for spec in specs:
            formal = output / "near_duplicate" / spec.key
            binding = packet["dataset_bindings"][spec.key]
            if (
                binding.get("geometry_summary_sha256")
                != sha256_file(formal / "geometry_summary.json")
                or binding.get("candidate_summary_sha256")
                != sha256_file(formal / "candidate_summary.json")
            ):
                raise MetricError(
                    f"completed annotation packet upstream binding drift: {spec.key}"
                )
        for name, field in (
            ("annotation_tasks.jsonl", "annotation_tasks_sha256"),
            ("annotation_audit.jsonl", "annotation_audit_sha256"),
            ("annotation_labels_template.jsonl", "labels_template_sha256"),
            ("preview_manifest.jsonl", "preview_manifest_sha256"),
        ):
            if sha256_file(calibration / name) != packet.get(field):
                raise MetricError(f"completed annotation packet has drifted: {name}")
    write_json(
        output / "status.json",
        {
            "schema_version": "table1_diversity_status_v1",
            "status": "BLOCKED_CALIBRATION",
            "updated_at_utc": utc_now(),
            "variance": "COMPLETE" if (output / "variance/summary.json").is_file() else "NOT_RUN",
            "near_duplicate": "AWAITING_HUMAN_LABELS",
            "next_command": f"{sys.executable} {SCRIPT_PATH} calibrate --output {output} --labels <completed-labels.jsonl> --resume",
        },
        self_hash="status_content_sha256",
    )
    print("[gate] candidate preparation complete; formal score blocked pending human labels", flush=True)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("variance", "prepare", "run", "calibrate", "score"))
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--scratch", type=Path, default=DEFAULT_SCRATCH)
    parser.add_argument("--dataset", action="append", dest="datasets")
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--point-count", type=int, default=2048)
    parser.add_argument("--top-k", type=int, default=64)
    parser.add_argument("--exhaustive-group-limit", type=int, default=256)
    parser.add_argument("--annotation-pairs", type=int, default=1000)
    parser.add_argument("--pva-root-override", type=Path, default=DEFAULT_PVA_MIRROR)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--labels", type=Path)
    parser.add_argument("--target-precision", type=float, default=0.98)
    parser.add_argument("--min-resolved-labels", type=int, default=800)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        if args.workers < 1 or args.point_count < 64 or args.top_k < 1 or args.exhaustive_group_limit < 2:
            raise MetricError("workers/point-count/top-k/exhaustive-group-limit are out of range")
        if not 0 < args.target_precision <= 1:
            raise MetricError("target precision must be in (0, 1]")
        specs = selected_specs(args.datasets)
        initialize_run(args, specs)
        if args.command in {"variance", "run"}:
            run_variance(args.output.resolve(), specs)
        if args.command in {"prepare", "run"}:
            run_prepare(args, specs)
        elif args.command == "calibrate":
            if args.labels is None:
                raise MetricError("calibrate requires --labels")
            calibrate_threshold(
                args.output.resolve(), args.labels.resolve(),
                target_precision=args.target_precision,
                min_resolved=args.min_resolved_labels,
            )
        elif args.command == "score":
            score_all(specs, args.output.resolve())
    except Exception as error:  # noqa: BLE001 - CLI emits one durable failure
        print(f"ERROR: {type(error).__name__}: {error}", file=sys.stderr, flush=True)
        return 1
    print(json.dumps({"status": "ok", "command": args.command, "output": str(args.output.resolve())}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
