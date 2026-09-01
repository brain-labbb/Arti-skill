#!/usr/bin/env python3
"""Run the frozen Table 7 production-readiness audit for Original Articraft.

This evaluator consumes the 143-record Table 6 cohort without rebuilding, repairing, or
reselecting any asset. Recovered URDFs remain strict compile failures and are reported in a
separate stratum. The audit is static except for copying each package into a fresh directory
inside the authorized workspace and parsing its copied artifact and dependencies.
"""

from __future__ import annotations

import argparse
from collections import Counter, deque
import hashlib
import json
import math
import os
from pathlib import Path, PureWindowsPath
import re
import shlex
import shutil
import sys
import tempfile
from typing import Any, Iterable
import xml.etree.ElementTree as ET

import numpy as np
import trimesh


SCRIPT_PATH = Path(__file__).resolve()
REPO_ROOT = SCRIPT_PATH.parents[2]
WORKSPACE_ROOT = REPO_ROOT.parent.resolve()
PROTOCOL_SOURCE = REPO_ROOT / "exp/reference/table7_production_readiness_protocol_v1.json"
TABLE6_ROOT = REPO_ROOT / "exp/runtime/table6_articraft_full"
TABLE6_MANIFEST = TABLE6_ROOT / "manifest.json"
TABLE6_RECORDS = TABLE6_ROOT / "asset_records.json"
DEFAULT_OUTPUT = REPO_ROOT / "exp/runtime/table7_articraft"

EXPECTED_REQUESTED = 143
EXPECTED_STRICT = 126
EXPECTED_RECOVERED_FAILURE = 17
MESH_SUFFIXES = {".obj", ".stl", ".ply", ".off", ".dae", ".glb", ".gltf"}
ONE_AXIS_JOINT_TYPES = {"revolute", "continuous", "prismatic"}
RECOGNIZED_JOINT_TYPES = {"fixed", "revolute", "continuous", "prismatic", "planar", "floating"}
PLACEHOLDER_NAME = re.compile(
    r"^(?:link|part|mesh|object|component|unnamed|default)(?:[_-]?\d+)?$", re.IGNORECASE
)
URI_SCHEME = re.compile(r"^[A-Za-z][A-Za-z0-9+.-]*:")


def ensure_within(path: Path, root: Path = WORKSPACE_ROOT, *, strict: bool = False) -> Path:
    """Resolve a path and reject any target outside the authorized workspace."""

    canonical_root = root.resolve(strict=True)
    canonical = path.resolve(strict=strict)
    if canonical != canonical_root and canonical_root not in canonical.parents:
        raise RuntimeError(f"path escapes authorized workspace: {path}")
    return canonical


def repo_relative(path: Path) -> str:
    return ensure_within(path, strict=False).relative_to(REPO_ROOT).as_posix()


def sha256_file(path: Path) -> str:
    path = ensure_within(path, strict=True)
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_json_sha256(value: Any) -> str:
    payload = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def read_json(path: Path) -> Any:
    path = ensure_within(path, strict=True)
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, value: Any) -> None:
    path = ensure_within(path, strict=False)
    ensure_within(path.parent, strict=True)
    temporary = ensure_within(path.with_suffix(path.suffix + ".tmp"), strict=False)
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def ratio(numerator: int, denominator: int) -> dict[str, Any]:
    return {
        "numerator": numerator,
        "denominator": denominator,
        "rate": numerator / denominator if denominator else None,
    }


def state_partition(rows: list[dict[str, Any]], key: str) -> dict[str, Any]:
    counts = Counter(nested_value(row, key) for row in rows)
    normalized = {state: int(counts.get(state, 0)) for state in ("pass", "fail", "not_evaluable")}
    evaluable = normalized["pass"] + normalized["fail"]
    normalized["denominator"] = len(rows)
    normalized["evaluable_denominator"] = evaluable
    normalized["pass_rate"] = normalized["pass"] / evaluable if evaluable else None
    return normalized


def gate_partition(rows: list[dict[str, Any]], key: str) -> dict[str, Any]:
    partition = state_partition(rows, f"{key}.state")
    gate_names = sorted(
        {
            gate
            for row in rows
            for gate, value in row.get(key, {}).get("gates", {}).items()
            if isinstance(value, bool)
        }
    )
    partition["gate_pass_counts"] = {
        gate: ratio(
            sum(row.get(key, {}).get("gates", {}).get(gate) is True for row in rows),
            sum(isinstance(row.get(key, {}).get("gates", {}).get(gate), bool) for row in rows),
        )
        for gate in gate_names
    }
    return partition


def nested_value(row: dict[str, Any], key: str) -> Any:
    value: Any = row
    for part in key.split("."):
        value = value[part]
    return value


def finite_vector(raw: str | None, *, default_zero: bool = False) -> bool:
    if raw is None:
        return default_zero
    try:
        values = [float(item) for item in raw.split()]
    except ValueError:
        return False
    return len(values) == 3 and all(math.isfinite(value) for value in values)


def finite_number(raw: str | None) -> float | None:
    if raw is None:
        return None
    try:
        value = float(raw)
    except ValueError:
        return None
    return value if math.isfinite(value) else None


def local_tag(node: ET.Element) -> str:
    return node.tag.rsplit("}", 1)[-1].lower()


def load_frozen_cohort() -> tuple[dict[str, Any], list[dict[str, Any]], dict[str, Any]]:
    protocol = read_json(PROTOCOL_SOURCE)
    table6_manifest = read_json(TABLE6_MANIFEST)
    table6_records = read_json(TABLE6_RECORDS)
    manifest_rows = table6_manifest.get("records", [])
    if not isinstance(manifest_rows, list) or not isinstance(table6_records, list):
        raise RuntimeError("invalid Table 6 manifest or asset-record schema")
    if len(manifest_rows) != EXPECTED_REQUESTED or len(table6_records) != EXPECTED_REQUESTED:
        raise RuntimeError("frozen Table 6 cohort is not exactly 143 records")

    manifest_ids = [row.get("record_id") for row in manifest_rows]
    record_ids = [row.get("record_id") for row in table6_records]
    if len(set(manifest_ids)) != EXPECTED_REQUESTED or set(manifest_ids) != set(record_ids):
        raise RuntimeError("Table 6 cohort identities are not unique and identical")
    if any(int(row.get("rating", 0)) < 4 for row in manifest_rows):
        raise RuntimeError("frozen cohort contains a rating below 4")

    table6_by_id = {row["record_id"]: row for row in table6_records}
    cohort: list[dict[str, Any]] = []
    for selected in manifest_rows:
        result = table6_by_id[selected["record_id"]]
        strict_success = result.get("strict_compile_success") is True
        recovered_failure = (
            not strict_success
            and result.get("strict_failure_export_recovered") is True
            and result.get("artifact_export_available") is True
        )
        if strict_success:
            stratum = "strict_compile_success"
        elif recovered_failure:
            stratum = "strict_compile_failure_recovered_urdf"
        else:
            stratum = "strict_compile_failure_no_artifact"
        cohort.append(
            {
                **selected,
                "cohort_stratum": stratum,
                "strict_compile_success": strict_success,
                "strict_failure_export_recovered": recovered_failure,
                "table6_artifact_export_available": result.get("artifact_export_available") is True,
                "table6_model_urdf_sha256": result.get("model_urdf_sha256"),
                "table6_compile_error": result.get("compile_error"),
            }
        )

    counts = Counter(row["cohort_stratum"] for row in cohort)
    if counts["strict_compile_success"] != EXPECTED_STRICT:
        raise RuntimeError(f"strict-success stratum drifted: {dict(counts)}")
    if counts["strict_compile_failure_recovered_urdf"] != EXPECTED_RECOVERED_FAILURE:
        raise RuntimeError(f"recovered strict-failure stratum drifted: {dict(counts)}")
    if counts["strict_compile_failure_no_artifact"]:
        raise RuntimeError(f"unexpected unavailable Table 6 assets: {dict(counts)}")
    return protocol, cohort, table6_manifest


def make_manifest(
    protocol: dict[str, Any], cohort: list[dict[str, Any]], table6_manifest: dict[str, Any]
) -> dict[str, Any]:
    source_manifest_path = REPO_ROOT / table6_manifest["source_manifest"]
    source_manifest_path = ensure_within(source_manifest_path, strict=True)
    source_manifest_sha256 = sha256_file(source_manifest_path)
    expected_source_hash = table6_manifest.get("source_manifest_sha256")
    if source_manifest_sha256 != expected_source_hash:
        raise RuntimeError("upstream formal source manifest hash no longer matches Table 6 freeze")

    records = []
    for row in cohort:
        package = ensure_within(REPO_ROOT / row["work_package"], strict=True)
        source = ensure_within(package / "model.py", strict=True)
        source_hash = sha256_file(source)
        if source_hash != row["source_model_sha256"]:
            raise RuntimeError(f"frozen source hash mismatch: {row['record_id']}")
        records.append(
            {
                key: row[key]
                for key in (
                    "record_id",
                    "category_slug",
                    "complexity",
                    "rating",
                    "selection_index",
                    "selection_rule",
                    "sdk_package",
                    "source_revision",
                    "source_model_sha256",
                    "source_prompt_sha256",
                    "work_package",
                    "cohort_stratum",
                    "strict_compile_success",
                    "strict_failure_export_recovered",
                    "table6_artifact_export_available",
                    "table6_model_urdf_sha256",
                    "table6_compile_error",
                )
            }
        )

    return {
        "schema_version": "1.0.0",
        "method": "Original Articraft",
        "evaluation_axis": "Table 7 Production Readiness",
        "protocol_id": protocol["protocol_id"],
        "protocol_snapshot": "exp/runtime/table7_articraft/protocol_snapshot.json",
        "protocol_snapshot_sha256": sha256_file(PROTOCOL_SOURCE),
        "selection_policy": {
            "rule": table6_manifest["selection_rule"],
            "outcome_based_reselection": False,
            "rating_threshold": ">=4",
            "requested_assets": EXPECTED_REQUESTED,
            "category_count": len({row["category_slug"] for row in cohort}),
            "strata": {
                "strict_compile_success": EXPECTED_STRICT,
                "strict_compile_failure_recovered_urdf": EXPECTED_RECOVERED_FAILURE,
            },
            "denominator_policy": "all 143 requested assets remain in every applicable state partition",
        },
        "frozen_inputs": {
            "table6_manifest": repo_relative(TABLE6_MANIFEST),
            "table6_manifest_sha256": sha256_file(TABLE6_MANIFEST),
            "table6_asset_records": repo_relative(TABLE6_RECORDS),
            "table6_asset_records_sha256": sha256_file(TABLE6_RECORDS),
            "formal_source_manifest": repo_relative(source_manifest_path),
            "formal_source_manifest_sha256": source_manifest_sha256,
        },
        "adapter": {
            "primary_artifact": "model.urdf",
            "per_asset_source": "model.py",
            "mesh_payloads": "union of packaged and URDF-referenced mesh files, deduplicated per asset",
            "manifold": "edge-manifold proxy: every undirected edge has at most two incident faces",
            "degenerate_faces": "repeated face vertex index or area <= 1e-12",
            "origin_transform": "an explicit origin element; omitted xyz/rpy attributes use URDF zero defaults",
            "self_intersection": "not_evaluable; no exact adjacent-face-excluding backend was run",
            "deterministic_build": "not_evaluable; no two fresh rebuilds are executed",
            "semantic_complete": "not_evaluable without output-independent required-part and role gold",
            "semantic_field_proxy": "non-placeholder visual-geometry-bearing link names plus one rooted connected acyclic hierarchy",
            "physical_defaults": "runtime-supplied defaults never satisfy a native metadata gate",
        },
        "records": records,
    }


def package_entries(package: Path) -> list[dict[str, Any]]:
    package = ensure_within(package, strict=True)
    entries: list[dict[str, Any]] = []
    for current_raw, directory_names, file_names in os.walk(package, followlinks=False):
        current = ensure_within(Path(current_raw), strict=True)
        directory_names.sort()
        file_names.sort()
        for name in list(directory_names):
            path = current / name
            if path.is_symlink():
                entries.append(
                    {
                        "path": path.relative_to(package).as_posix(),
                        "kind": "symlink",
                        "target": os.readlink(path),
                    }
                )
                directory_names.remove(name)
        for name in file_names:
            path = current / name
            relative = path.relative_to(package).as_posix()
            if path.is_symlink():
                entries.append({"path": relative, "kind": "symlink", "target": os.readlink(path)})
                continue
            path = ensure_within(path, strict=True)
            entries.append(
                {
                    "path": relative,
                    "kind": "file",
                    "bytes": path.stat().st_size,
                    "sha256": sha256_file(path),
                }
            )
    return sorted(entries, key=lambda item: item["path"])


def package_manifest(package: Path) -> dict[str, Any]:
    entries = package_entries(package)
    return {
        "entry_count": len(entries),
        "file_count": sum(row["kind"] == "file" for row in entries),
        "symlink_count": sum(row["kind"] == "symlink" for row in entries),
        "content_manifest_sha256": canonical_json_sha256(entries),
    }


def urdf_dependency_specs(root: ET.Element) -> list[tuple[str, str, str]]:
    specs: list[tuple[str, str, str]] = []
    for node in root.findall(".//mesh"):
        specs.append(("urdf_mesh", node.attrib.get("filename", ""), "model.urdf"))
    for node in root.findall(".//texture"):
        filename = node.attrib.get("filename")
        if filename is not None:
            specs.append(("urdf_texture", filename, "model.urdf"))
    return specs


def nested_dependency_specs(path: Path, package: Path) -> list[tuple[str, str, str]]:
    suffix = path.suffix.lower()
    relative = path.relative_to(package).as_posix()
    specs: list[tuple[str, str, str]] = []
    if suffix == ".obj":
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            stripped = line.strip()
            if not stripped.lower().startswith("mtllib "):
                continue
            try:
                values = shlex.split(stripped[len("mtllib ") :])
            except ValueError:
                values = [stripped[len("mtllib ") :].strip()]
            specs.extend(("obj_mtllib", value, relative) for value in values if value)
    elif suffix == ".mtl":
        map_keys = {"map_ka", "map_kd", "map_ks", "map_ke", "map_d", "bump", "map_bump", "disp", "decal", "refl"}
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            try:
                values = shlex.split(line, comments=True)
            except ValueError:
                continue
            if len(values) >= 2 and values[0].lower() in map_keys:
                specs.append(("mtl_texture", values[-1], relative))
    elif suffix == ".gltf":
        payload = json.loads(path.read_text(encoding="utf-8"))
        for section in ("buffers", "images"):
            for item in payload.get(section, []):
                uri = item.get("uri")
                if isinstance(uri, str) and not uri.startswith("data:"):
                    specs.append((f"gltf_{section[:-1]}", uri, relative))
    elif suffix == ".dae":
        dae_root = ET.parse(path).getroot()
        for node in dae_root.iter():
            if local_tag(node) == "init_from" and node.text and node.text.strip():
                specs.append(("dae_image", node.text.strip(), relative))
    return specs


def resolve_dependency(
    package: Path, declaring_file: Path, raw_reference: str
) -> tuple[Path | None, str | None, bool]:
    raw = raw_reference.strip()
    if not raw:
        return None, "empty dependency reference", False
    if "\\" in raw:
        return None, f"non-portable backslash dependency: {raw}", False
    if URI_SCHEME.match(raw) or Path(raw).is_absolute() or PureWindowsPath(raw).is_absolute():
        return None, f"dependency is not relative: {raw}", False
    lexical = declaring_file.parent / raw
    try:
        resolved = lexical.resolve(strict=False)
    except OSError as exc:
        return None, f"dependency cannot resolve: {raw}: {type(exc).__name__}: {exc}", False
    try:
        resolved.relative_to(package)
    except ValueError:
        return None, f"dependency escapes copied package: {raw}", False

    uses_symlink = False
    cursor = declaring_file.parent
    for part in Path(raw).parts:
        cursor = cursor / part
        if cursor.is_symlink():
            uses_symlink = True
    if not resolved.is_file():
        return None, f"dependency is missing: {raw}", uses_symlink
    return ensure_within(resolved, strict=True), None, uses_symlink


def dependency_closure(primary: Path, package: Path) -> dict[str, Any]:
    package = ensure_within(package, strict=True)
    primary = ensure_within(primary, strict=True)
    root = ET.parse(primary).getroot()
    queue: deque[tuple[str, str, Path]] = deque(
        (kind, raw, primary) for kind, raw, _declared in urdf_dependency_specs(root)
    )
    seen_specs: set[tuple[str, str, str]] = set()
    visited_files: set[Path] = set()
    records: list[dict[str, Any]] = []
    issues: list[str] = []
    mesh_paths: set[Path] = set()
    while queue:
        kind, raw, declaring_file = queue.popleft()
        declaring_relative = declaring_file.relative_to(package).as_posix()
        spec_key = (kind, raw, declaring_relative)
        if spec_key in seen_specs:
            continue
        seen_specs.add(spec_key)
        resolved, error, uses_symlink = resolve_dependency(package, declaring_file, raw)
        record = {
            "kind": kind,
            "declared_by": declaring_relative,
            "reference": raw,
            "relative": not bool(URI_SCHEME.match(raw)) and not Path(raw).is_absolute(),
            "uses_internal_symlink": uses_symlink,
            "resolved_path": resolved.relative_to(package).as_posix() if resolved else None,
            "exists": resolved is not None,
            "error": error,
        }
        records.append(record)
        if error:
            issues.append(error)
            continue
        assert resolved is not None
        if resolved.suffix.lower() in MESH_SUFFIXES:
            mesh_paths.add(resolved)
        if resolved in visited_files:
            continue
        visited_files.add(resolved)
        try:
            nested = nested_dependency_specs(resolved, package)
        except Exception as exc:  # noqa: BLE001
            issues.append(
                f"cannot parse nested dependencies in {resolved.relative_to(package)}: "
                f"{type(exc).__name__}: {exc}"
            )
            continue
        for nested_kind, nested_raw, _declared in nested:
            queue.append((nested_kind, nested_raw, resolved))
    return {
        "root": root,
        "dependency_records": records,
        "issues": issues,
        "mesh_paths": sorted(mesh_paths),
    }


def portable_package_audit(package: Path, scratch_root: Path) -> dict[str, Any]:
    if not package.is_dir() or not (package / "model.urdf").is_file():
        return {
            "state": "not_evaluable",
            "reason": "no available final URDF package",
            "fresh_workspace_copy": False,
            "scratch_removed": True,
            "issues": [],
        }

    package = ensure_within(package, strict=True)
    scratch_root = ensure_within(scratch_root, strict=True)
    original_manifest = package_manifest(package)
    temporary = Path(tempfile.mkdtemp(prefix=f"{package.name}-", dir=scratch_root))
    temporary = ensure_within(temporary, strict=True)
    copied_package = ensure_within(temporary / "package", strict=False)
    issues: list[str] = []
    dependency_records: list[dict[str, Any]] = []
    copied_manifest: dict[str, Any] | None = None
    parse_success = False
    mesh_load_success = False
    try:
        shutil.copytree(package, copied_package, symlinks=True)
        copied_package = ensure_within(copied_package, strict=True)
        copied_manifest = package_manifest(copied_package)
        if copied_manifest["content_manifest_sha256"] != original_manifest["content_manifest_sha256"]:
            issues.append("clean copy content-manifest hash differs from original package")
        primary = copied_package / "model.urdf"
        try:
            closure = dependency_closure(primary, copied_package)
            parse_success = True
            dependency_records = closure["dependency_records"]
            issues.extend(closure["issues"])
            mesh_errors: list[str] = []
            for mesh_path in closure["mesh_paths"]:
                try:
                    loaded = trimesh.load(mesh_path, force="scene", process=False)
                    geometries = list(getattr(loaded, "geometry", {}).values())
                    if not geometries:
                        raise ValueError("no readable triangle geometry")
                except Exception as exc:  # noqa: BLE001
                    mesh_errors.append(
                        f"copied mesh parse failed for {mesh_path.relative_to(copied_package)}: "
                        f"{type(exc).__name__}: {exc}"
                    )
            issues.extend(mesh_errors)
            mesh_load_success = not mesh_errors
        except Exception as exc:  # noqa: BLE001
            issues.append(f"copied primary artifact parse failed: {type(exc).__name__}: {exc}")
    finally:
        shutil.rmtree(temporary)

    state = "pass" if parse_success and mesh_load_success and not issues else "fail"
    return {
        "state": state,
        "reason": None if state == "pass" else "one or more clean-copy package gates failed",
        "fresh_workspace_copy": True,
        "scratch_location": repo_relative(scratch_root),
        "scratch_removed": not temporary.exists(),
        "primary_artifact_parsed": parse_success,
        "declared_dependency_count": len(dependency_records),
        "all_dependencies_relative": all(row["relative"] for row in dependency_records),
        "all_dependencies_exist": all(row["exists"] for row in dependency_records),
        "external_required_symlink_count": 0,
        "copied_meshes_parse": mesh_load_success,
        "copy_integrity_hash_equal": bool(
            copied_manifest
            and copied_manifest["content_manifest_sha256"]
            == original_manifest["content_manifest_sha256"]
        ),
        "original_package_manifest": original_manifest,
        "copied_package_manifest": copied_manifest,
        "dependencies": dependency_records,
        "issues": issues,
    }


def packaged_mesh_paths(package: Path, root: ET.Element | None) -> tuple[list[Path], list[str]]:
    paths: set[Path] = set()
    issues: list[str] = []
    assets = package / "assets"
    if assets.is_dir():
        for current_raw, directory_names, file_names in os.walk(assets, followlinks=False):
            current = ensure_within(Path(current_raw), strict=True)
            directory_names.sort()
            file_names.sort()
            for name in list(directory_names):
                child = current / name
                if child.is_symlink():
                    issues.append(f"packaged mesh tree contains directory symlink: {child.relative_to(package)}")
                    directory_names.remove(name)
            for name in file_names:
                path = current / name
                if path.suffix.lower() not in MESH_SUFFIXES:
                    continue
                try:
                    paths.add(ensure_within(path, strict=True))
                except Exception as exc:  # noqa: BLE001
                    issues.append(f"mesh payload escapes workspace: {path}: {type(exc).__name__}: {exc}")
    if root is not None:
        primary = package / "model.urdf"
        for _kind, raw, _declared in urdf_dependency_specs(root):
            if Path(raw).suffix.lower() not in MESH_SUFFIXES:
                continue
            resolved, error, _uses_symlink = resolve_dependency(package, primary, raw)
            if error:
                issues.append(error)
            elif resolved is not None:
                paths.add(resolved)
    return sorted(paths), issues


def mesh_geometry_audit(mesh_paths: list[Path], package: Path, initial_issues: list[str]) -> dict[str, Any]:
    records: list[dict[str, Any]] = []
    errors = list(initial_issues)
    for path in mesh_paths:
        relative = path.relative_to(package).as_posix()
        try:
            loaded = trimesh.load(path, force="scene", process=False)
            geometries = list(getattr(loaded, "geometry", {}).items())
            if not geometries:
                raise ValueError("no geometry in loaded scene")
            for geometry_name, mesh in geometries:
                faces = np.asarray(mesh.faces)
                if faces.ndim != 2 or faces.shape[1] != 3 or len(faces) == 0:
                    raise ValueError(f"non-triangle or empty geometry: {geometry_name}")
                edges = np.asarray(mesh.edges_sorted)
                _unique_edges, edge_counts = np.unique(edges, axis=0, return_counts=True)
                open_edges = int(np.count_nonzero(edge_counts == 1))
                nonmanifold_edges = int(np.count_nonzero(edge_counts > 2))
                repeated = (
                    (faces[:, 0] == faces[:, 1])
                    | (faces[:, 1] == faces[:, 2])
                    | (faces[:, 0] == faces[:, 2])
                )
                areas = np.asarray(mesh.area_faces)
                degenerate = repeated | ~np.isfinite(areas) | (areas <= 1e-12)
                records.append(
                    {
                        "mesh_path": relative,
                        "geometry_name": str(geometry_name),
                        "vertices": int(len(mesh.vertices)),
                        "faces": int(len(faces)),
                        "watertight": bool(mesh.is_watertight),
                        "edge_manifold_proxy": nonmanifold_edges == 0,
                        "open_edges": open_edges,
                        "nonmanifold_edges": nonmanifold_edges,
                        "degenerate_faces": int(np.count_nonzero(degenerate)),
                        "repeated_index_faces": int(np.count_nonzero(repeated)),
                        "area_degenerate_faces": int(np.count_nonzero(~np.isfinite(areas) | (areas <= 1e-12))),
                        "self_intersection": {
                            "state": "not_evaluable",
                            "reason": "no exact adjacent-face-excluding triangle intersection backend was run",
                        },
                    }
                )
        except Exception as exc:  # noqa: BLE001
            errors.append(f"{relative}: {type(exc).__name__}: {exc}")

    evaluable = bool(records) and not errors
    geometry_count = len(records)
    watertight_count = sum(row["watertight"] for row in records)
    manifold_count = sum(row["edge_manifold_proxy"] for row in records)
    open_edges = sum(row["open_edges"] for row in records)
    degenerate_faces = sum(row["degenerate_faces"] for row in records)
    return {
        "state": "pass" if evaluable else "not_evaluable",
        "evaluable": evaluable,
        "mesh_payload_count": len(mesh_paths),
        "readable_geometry_count": geometry_count,
        "load_errors": errors,
        "watertight": {
            "state": (
                "not_evaluable"
                if not evaluable
                else "pass" if watertight_count == geometry_count else "fail"
            ),
            "numerator": watertight_count,
            "denominator": geometry_count if evaluable else 0,
            "fraction": watertight_count / geometry_count if evaluable else None,
        },
        "manifold": {
            "state": (
                "not_evaluable"
                if not evaluable
                else "pass" if manifold_count == geometry_count else "fail"
            ),
            "numerator": manifold_count,
            "denominator": geometry_count if evaluable else 0,
            "fraction": manifold_count / geometry_count if evaluable else None,
            "definition": "edge-manifold proxy; vertex-manifold is not claimed",
        },
        "open_edges": {
            "state": "not_evaluable" if not evaluable else "pass" if open_edges == 0 else "fail",
            "count": open_edges if evaluable else None,
        },
        "degenerate_faces": {
            "state": (
                "not_evaluable" if not evaluable else "pass" if degenerate_faces == 0 else "fail"
            ),
            "count": degenerate_faces if evaluable else None,
        },
        "self_intersection": {
            "state": "not_evaluable",
            "reason": "no exact adjacent-face-excluding triangle intersection backend was run",
        },
        "geometries": records,
    }


def hierarchy_gates(root: ET.Element) -> dict[str, Any]:
    links = root.findall("link")
    link_names = [node.attrib.get("name", "").strip() for node in links]
    link_set = set(link_names)
    joints = root.findall("joint")
    adjacency: dict[str, list[str]] = {name: [] for name in link_names}
    children: set[str] = set()
    references_resolve = bool(link_names) and len(link_set) == len(link_names) and "" not in link_set
    for joint in joints:
        parent_node = joint.find("parent")
        child_node = joint.find("child")
        parent = parent_node.attrib.get("link", "") if parent_node is not None else ""
        child = child_node.attrib.get("link", "") if child_node is not None else ""
        if parent not in link_set or child not in link_set:
            references_resolve = False
            continue
        adjacency[parent].append(child)
        children.add(child)
    roots = [name for name in link_names if name not in children]
    visited: set[str] = set()
    active: set[str] = set()
    acyclic = True

    def visit(name: str) -> None:
        nonlocal acyclic
        if name in active:
            acyclic = False
            return
        if name in visited:
            return
        active.add(name)
        for child in adjacency.get(name, []):
            visit(child)
        active.remove(name)
        visited.add(name)

    if len(roots) == 1:
        visit(roots[0])
    connected = len(roots) == 1 and visited == link_set
    valid = (
        references_resolve
        and len(roots) == 1
        and connected
        and acyclic
        and len(joints) == max(0, len(link_names) - 1)
    )
    return {
        "link_count": len(link_names),
        "joint_count": len(joints),
        "unique_nonempty_link_names": len(link_set) == len(link_names) and "" not in link_set,
        "parent_child_references_resolve": references_resolve,
        "root_count": len(roots),
        "connected": connected,
        "acyclic": acyclic,
        "edge_count_is_links_minus_one": len(joints) == max(0, len(link_names) - 1),
        "valid_tree": valid,
    }


def semantic_audit(root: ET.Element | None, parse_error: str | None) -> tuple[dict[str, Any], dict[str, Any]]:
    strict = {
        "state": "not_evaluable",
        "reason": "no output-independent required-part and role specification exists for this frozen cohort",
    }
    if root is None:
        return strict, {
            "state": "not_evaluable",
            "reason": parse_error or "final URDF unavailable",
            "label": "field proxy, not semantic correctness",
        }
    hierarchy = hierarchy_gates(root)
    geometry_bearing_names = []
    for link in root.findall("link"):
        if link.findall(".//visual/geometry"):
            geometry_bearing_names.append(link.attrib.get("name", "").strip())
    invalid_names = [
        name for name in geometry_bearing_names if not name or PLACEHOLDER_NAME.fullmatch(name)
    ]
    names_pass = bool(geometry_bearing_names) and not invalid_names
    gates = {
        "visual_geometry_bearing_part_count": len(geometry_bearing_names),
        "all_visual_geometry_bearing_parts_named_nonplaceholder": names_pass,
        "invalid_or_placeholder_names": invalid_names,
        "one_rooted_connected_acyclic_hierarchy": hierarchy["valid_tree"],
        "native_role_field": {
            "state": "not_evaluable",
            "reason": "Articraft URDF contract has no separate required semantic-role field",
        },
    }
    return strict, {
        "state": "pass" if names_pass and hierarchy["valid_tree"] else "fail",
        "label": "field proxy, not semantic correctness",
        "gates": gates,
    }


def kinematic_audit(root: ET.Element | None, parse_error: str | None) -> dict[str, Any]:
    if root is None:
        return {"state": "not_evaluable", "reason": parse_error or "final URDF unavailable"}
    joints = root.findall("joint")
    if not joints:
        return {"state": "not_evaluable", "reason": "native package exposes no joints"}
    links = {node.attrib.get("name", "") for node in root.findall("link")}
    details: list[dict[str, Any]] = []
    for joint in joints:
        joint_type = joint.attrib.get("type", "")
        parent_node = joint.find("parent")
        child_node = joint.find("child")
        parent = parent_node.attrib.get("link", "") if parent_node is not None else ""
        child = child_node.attrib.get("link", "") if child_node is not None else ""
        parent_child = parent in links and child in links and parent != child
        recognized = joint_type in RECOGNIZED_JOINT_TYPES
        axis_required = joint_type in ONE_AXIS_JOINT_TYPES
        axis_node = joint.find("axis")
        axis_values: list[float] = []
        if axis_node is not None:
            try:
                axis_values = [float(value) for value in axis_node.attrib.get("xyz", "").split()]
            except ValueError:
                axis_values = []
        axis_valid = (
            not axis_required
            or (
                len(axis_values) == 3
                and all(math.isfinite(value) for value in axis_values)
                and math.sqrt(sum(value * value for value in axis_values)) > 1e-9
            )
        )
        limit_required = joint_type in {"revolute", "prismatic"}
        limit_node = joint.find("limit")
        lower = finite_number(limit_node.attrib.get("lower")) if limit_node is not None else None
        upper = finite_number(limit_node.attrib.get("upper")) if limit_node is not None else None
        limits_valid = not limit_required or (
            lower is not None and upper is not None and lower <= upper
        )
        origin_node = joint.find("origin")
        origin_valid = bool(
            origin_node is not None
            and finite_vector(origin_node.attrib.get("xyz"), default_zero=True)
            and finite_vector(origin_node.attrib.get("rpy"), default_zero=True)
        )
        details.append(
            {
                "name": joint.attrib.get("name", ""),
                "type": joint_type,
                "parent_child_resolve": parent_child,
                "recognized_type": recognized,
                "axis_valid": axis_valid,
                "limits_valid": limits_valid,
                "origin_valid": origin_valid,
            }
        )
    gates = {
        "parent_child_references": all(row["parent_child_resolve"] for row in details),
        "recognized_joint_types": all(row["recognized_type"] for row in details),
        "movable_one_axis_joint_axes": all(row["axis_valid"] for row in details),
        "revolute_prismatic_limits": all(row["limits_valid"] for row in details),
        "explicit_finite_joint_origins": all(row["origin_valid"] for row in details),
    }
    return {
        "state": "pass" if all(gates.values()) else "fail",
        "joint_count": len(details),
        "movable_joint_count": sum(row["type"] != "fixed" for row in details),
        "gates": gates,
        "joints": details,
    }


def inertia_positive_definite(inertia_node: ET.Element | None) -> bool:
    if inertia_node is None:
        return False
    names = ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
    values = {name: finite_number(inertia_node.attrib.get(name)) for name in names}
    if any(value is None for value in values.values()):
        return False
    matrix = np.array(
        [
            [values["ixx"], values["ixy"], values["ixz"]],
            [values["ixy"], values["iyy"], values["iyz"]],
            [values["ixz"], values["iyz"], values["izz"]],
        ],
        dtype=float,
    )
    try:
        eigenvalues = np.linalg.eigvalsh(matrix)
    except np.linalg.LinAlgError:
        return False
    return bool(np.all(np.isfinite(eigenvalues)) and np.all(eigenvalues > 0.0))


def native_contact_friction_present(root: ET.Element) -> bool:
    friction_tags = {"mu", "mu1", "mu2", "friction", "lateral_friction", "rolling_friction"}
    for container in root.iter():
        if local_tag(container) not in {"gazebo", "contact"}:
            continue
        for node in container.iter():
            if local_tag(node) in friction_tags:
                if node.text is None or finite_number(node.text.strip()) is not None:
                    return True
            for attribute in friction_tags:
                if finite_number(node.attrib.get(attribute)) is not None:
                    return True
    return False


def physical_audit(root: ET.Element | None, parse_error: str | None) -> dict[str, Any]:
    if root is None:
        return {"state": "not_evaluable", "reason": parse_error or "final URDF unavailable"}
    links = root.findall("link")
    if not links:
        return {"state": "not_evaluable", "reason": "native URDF has no physical links"}
    link_details: list[dict[str, Any]] = []
    for link in links:
        visual_bearing = bool(link.findall("visual"))
        collisions = link.findall("collision")
        collision_geometry = any(collision.find("geometry") is not None for collision in collisions)
        inertial = link.find("inertial")
        mass_node = inertial.find("mass") if inertial is not None else None
        mass = finite_number(mass_node.attrib.get("value")) if mass_node is not None else None
        inertia_node = inertial.find("inertia") if inertial is not None else None
        link_details.append(
            {
                "name": link.attrib.get("name", ""),
                "visual_bearing": visual_bearing,
                "native_collision_geometry": (not visual_bearing) or collision_geometry,
                "positive_finite_native_mass": mass is not None and mass > 0.0,
                "positive_definite_native_inertia": inertia_positive_definite(inertia_node),
            }
        )
    movable_joint_details = []
    for joint in root.findall("joint"):
        if joint.attrib.get("type", "") == "fixed":
            continue
        dynamics = joint.find("dynamics")
        damping = finite_number(dynamics.attrib.get("damping")) if dynamics is not None else None
        friction = finite_number(dynamics.attrib.get("friction")) if dynamics is not None else None
        movable_joint_details.append(
            {
                "name": joint.attrib.get("name", ""),
                "native_damping": damping is not None and damping >= 0.0,
                "native_friction": friction is not None and friction >= 0.0,
            }
        )
    gates = {
        "visual_links_have_native_collision": all(
            row["native_collision_geometry"] for row in link_details
        ),
        "physical_links_have_positive_native_mass": all(
            row["positive_finite_native_mass"] for row in link_details
        ),
        "physical_links_have_positive_definite_native_inertia": all(
            row["positive_definite_native_inertia"] for row in link_details
        ),
        "movable_joints_have_native_damping_and_friction": all(
            row["native_damping"] and row["native_friction"]
            for row in movable_joint_details
        ),
        "native_contact_material_or_friction_metadata": native_contact_friction_present(root),
    }
    return {
        "state": "pass" if all(gates.values()) else "fail",
        "runtime_defaults_count_as_native": False,
        "gates": gates,
        "links": link_details,
        "movable_joints": movable_joint_details,
    }


def audit_asset(row: dict[str, Any], scratch_root: Path) -> dict[str, Any]:
    package = ensure_within(REPO_ROOT / row["work_package"], strict=True)
    source = package / "model.py"
    urdf = package / "model.urdf"
    source_available = source.is_file()
    urdf_available = urdf.is_file()
    parse_error: str | None = None
    root: ET.Element | None = None
    if urdf_available:
        try:
            root = ET.parse(ensure_within(urdf, strict=True)).getroot()
        except Exception as exc:  # noqa: BLE001
            parse_error = f"{type(exc).__name__}: {exc}"
    else:
        parse_error = "model.urdf is missing"

    mesh_paths, mesh_path_issues = packaged_mesh_paths(package, root)
    geometry = mesh_geometry_audit(mesh_paths, package, mesh_path_issues)
    mesh_bytes = sum(path.stat().st_size for path in mesh_paths)
    semantic_complete, semantic_field_proxy = semantic_audit(root, parse_error)
    kinematic_complete = kinematic_audit(root, parse_error)
    physical_complete = physical_audit(root, parse_error)
    portable = portable_package_audit(package, scratch_root)
    package_hash = package_manifest(package)

    source_hash = sha256_file(source) if source_available else None
    urdf_hash = sha256_file(urdf) if urdf_available else None
    source_hash_match = source_hash == row["source_model_sha256"] if source_hash else False
    urdf_hash_match = urdf_hash == row["table6_model_urdf_sha256"] if urdf_hash else False
    artifact_entries = []
    if source_hash:
        artifact_entries.append({"path": "model.py", "sha256": source_hash})
    if urdf_hash:
        artifact_entries.append({"path": "model.urdf", "sha256": urdf_hash})
    for mesh_path in mesh_paths:
        artifact_entries.append(
            {
                "path": mesh_path.relative_to(package).as_posix(),
                "sha256": sha256_file(mesh_path),
            }
        )
    artifact_entries.sort(key=lambda item: item["path"])

    return {
        "record_id": row["record_id"],
        "category_slug": row["category_slug"],
        "complexity": row["complexity"],
        "rating": row["rating"],
        "selection_index": row["selection_index"],
        "cohort_stratum": row["cohort_stratum"],
        "strict_compile_success": row["strict_compile_success"],
        "strict_failure_export_recovered": row["strict_failure_export_recovered"],
        "recovered_urdf_is_strict_success": False,
        "work_package": row["work_package"],
        "availability": {
            "state": "pass" if urdf_available and root is not None else "fail",
            "source_available": source_available,
            "urdf_available": urdf_available,
            "urdf_parsed": root is not None,
            "urdf_parse_error": parse_error,
        },
        "integrity": {
            "source_sha256": source_hash,
            "source_matches_frozen_hash": source_hash_match,
            "urdf_sha256": urdf_hash,
            "urdf_matches_table6_hash": urdf_hash_match,
            "audited_artifact_manifest_sha256": canonical_json_sha256(artifact_entries),
            "audited_artifact_entry_count": len(artifact_entries),
            "package_content_manifest": package_hash,
        },
        "size_bytes": {
            "source": source.stat().st_size if source_available else None,
            "urdf": urdf.stat().st_size if urdf_available else None,
            "mesh": mesh_bytes if mesh_paths else None,
            "mesh_payload_count": len(mesh_paths),
        },
        "geometry": geometry,
        "portable_package": portable,
        "deterministic_build": {
            "state": "not_evaluable",
            "fresh_build_count": 0,
            "reason": "this static audit did not execute two fresh builds from frozen inputs",
            "existing_artifact_rehash_not_used_as_build_evidence": True,
        },
        "semantic_complete": semantic_complete,
        "semantic_field_proxy": semantic_field_proxy,
        "kinematic_complete": kinematic_complete,
        "physical_complete": physical_complete,
    }


def mean_or_none(values: Iterable[float]) -> float | None:
    materialized = list(values)
    return sum(materialized) / len(materialized) if materialized else None


def geometry_summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
    evaluable = [row for row in rows if row["geometry"]["evaluable"]]
    geometries = [geometry for row in evaluable for geometry in row["geometry"]["geometries"]]
    watertight = sum(geometry["watertight"] for geometry in geometries)
    manifold = sum(geometry["edge_manifold_proxy"] for geometry in geometries)
    open_edges = sum(geometry["open_edges"] for geometry in geometries)
    degenerate = sum(geometry["degenerate_faces"] for geometry in geometries)
    return {
        "requested_assets": len(rows),
        "geometry_evaluable_assets": len(evaluable),
        "geometry_evaluable_asset_rate": ratio(len(evaluable), len(rows)),
        "readable_geometries": len(geometries),
        "watertight": {
            "per_asset_mean_fraction": mean_or_none(
                row["geometry"]["watertight"]["fraction"] for row in evaluable
            ),
            "geometry_level": ratio(watertight, len(geometries)),
            "all_geometries_pass_assets": ratio(
                sum(row["geometry"]["watertight"]["state"] == "pass" for row in evaluable),
                len(evaluable),
            ),
        },
        "manifold": {
            "definition": "edge-manifold proxy; vertex-manifold is not claimed",
            "per_asset_mean_fraction": mean_or_none(
                row["geometry"]["manifold"]["fraction"] for row in evaluable
            ),
            "geometry_level": ratio(manifold, len(geometries)),
            "all_geometries_pass_assets": ratio(
                sum(row["geometry"]["manifold"]["state"] == "pass" for row in evaluable),
                len(evaluable),
            ),
        },
        "open_edges": {
            "per_asset_mean": mean_or_none(
                float(row["geometry"]["open_edges"]["count"]) for row in evaluable
            ),
            "total": open_edges,
            "zero_error_assets": ratio(
                sum(row["geometry"]["open_edges"]["state"] == "pass" for row in evaluable),
                len(evaluable),
            ),
        },
        "degenerate_faces": {
            "per_asset_mean": mean_or_none(
                float(row["geometry"]["degenerate_faces"]["count"]) for row in evaluable
            ),
            "total": degenerate,
            "zero_error_assets": ratio(
                sum(row["geometry"]["degenerate_faces"]["state"] == "pass" for row in evaluable),
                len(evaluable),
            ),
        },
        "self_intersection": {
            "state": "not_evaluable",
            "denominator": len(rows),
            "reason": "no exact adjacent-face-excluding triangle intersection backend was run",
        },
        "geometry_not_evaluable_assets": len(rows) - len(evaluable),
        "no_mesh_payload_assets": sum(
            row["geometry"]["mesh_payload_count"] == 0 for row in rows
        ),
        "mesh_load_error_assets": sum(bool(row["geometry"]["load_errors"]) for row in rows),
        "geometry_scope": (
            "packaged or URDF-referenced readable triangle mesh payloads only; "
            "primitive-only URDF geometry is not triangulated"
        ),
    }


def size_summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
    payload: dict[str, Any] = {}
    for key in ("source", "urdf", "mesh"):
        values = [row["size_bytes"][key] for row in rows if row["size_bytes"][key] is not None]
        payload[key] = {
            "total_bytes": int(sum(values)),
            "mean_bytes": mean_or_none(float(value) for value in values),
            "denominator": len(values),
        }
    return payload


def summarize(records: list[dict[str, Any]], manifest: dict[str, Any]) -> dict[str, Any]:
    strata: dict[str, Any] = {}
    for name in ("strict_compile_success", "strict_compile_failure_recovered_urdf"):
        rows = [row for row in records if row["cohort_stratum"] == name]
        strata[name] = {
            "requested_assets": len(rows),
            "available_assets": sum(row["availability"]["state"] == "pass" for row in rows),
            "geometry": geometry_summary(rows),
            "portable_package": state_partition(rows, "portable_package.state"),
            "deterministic_build": state_partition(rows, "deterministic_build.state"),
            "semantic_complete": state_partition(rows, "semantic_complete.state"),
            "semantic_field_proxy": gate_partition(rows, "semantic_field_proxy"),
            "kinematic_complete": gate_partition(rows, "kinematic_complete"),
            "physical_complete": gate_partition(rows, "physical_complete"),
            "size_bytes": size_summary(rows),
        }

    available = sum(row["availability"]["state"] == "pass" for row in records)
    geometry_evaluable = sum(row["geometry"]["evaluable"] for row in records)
    package_evaluable = sum(row["portable_package"]["state"] != "not_evaluable" for row in records)
    return {
        "schema_version": "1.0.0",
        "status": "COMPLETED_STATIC_AUDIT",
        "method": "Original Articraft",
        "scope": (
            "frozen 12-category, 143-record rating>=4 Original Articraft cohort inherited "
            "from Table 6; no outcome-based reselection"
        ),
        "protocol": {
            "protocol_id": manifest["protocol_id"],
            "protocol_snapshot": manifest["protocol_snapshot"],
            "protocol_snapshot_sha256": manifest["protocol_snapshot_sha256"],
        },
        "cohort": {
            "requested_assets": len(records),
            "available_assets": available,
            "geometry_evaluable_assets": geometry_evaluable,
            "package_evaluable_assets": package_evaluable,
            "strict_compile_success": EXPECTED_STRICT,
            "strict_compile_failure_recovered_urdf": EXPECTED_RECOVERED_FAILURE,
            "recovered_urdfs_promoted_to_strict_success": 0,
            "selection_rule": manifest["selection_policy"]["rule"],
        },
        "results": {
            "geometry": geometry_summary(records),
            "size_bytes": size_summary(records),
            "portable_package": state_partition(records, "portable_package.state"),
            "deterministic_build": state_partition(records, "deterministic_build.state"),
            "semantic_complete": state_partition(records, "semantic_complete.state"),
            "semantic_field_proxy": gate_partition(records, "semantic_field_proxy"),
            "kinematic_complete": gate_partition(records, "kinematic_complete"),
            "physical_complete": gate_partition(records, "physical_complete"),
        },
        "strata": strata,
        "integrity": {
            "source_hash_matches": ratio(
                sum(row["integrity"]["source_matches_frozen_hash"] for row in records),
                len(records),
            ),
            "urdf_hash_matches_table6": ratio(
                sum(row["integrity"]["urdf_matches_table6_hash"] for row in records),
                len(records),
            ),
        },
        "evidence_files": {
            "protocol_snapshot": "exp/runtime/table7_articraft/protocol_snapshot.json",
            "manifest": "exp/runtime/table7_articraft/manifest.json",
            "asset_records": "exp/runtime/table7_articraft/asset_records.json",
            "summary": "exp/runtime/table7_articraft/summary.json",
            "self_check": "exp/runtime/table7_articraft/self_check.json",
            "report": "exp/runtime/table7_articraft/report.md",
        },
        "caveats": [
            "Self-intersection is not evaluable because no exact adjacent-face-excluding backend was run.",
            "Deterministic build is not evaluable because this audit did not execute two fresh builds.",
            "Semantic completeness is not evaluable without output-independent required-part and role gold; the named-tree field proxy is separate.",
            "Physical completeness requires native metadata; prior simulator runtime defaults do not satisfy any gate.",
            "The 17 recovered URDFs remain strict compile failures in every result and denominator.",
            "Geometry scores cover packaged triangle meshes only; primitive-only URDF assets are not triangulated.",
        ],
    }


def percent(value: float | None) -> str:
    return "N/A" if value is None else f"{100.0 * value:.2f}%"


def kb(value: float | None) -> str:
    return "N/A" if value is None else f"{value / 1024.0:.2f}"


def report_text(summary: dict[str, Any]) -> str:
    result = summary["results"]
    geometry = result["geometry"]
    size = result["size_bytes"]
    cohort = summary["cohort"]
    lines = [
        "# Original Articraft Table 7 production-readiness audit",
        "",
        "## Scope",
        "",
        (
            f"The audit retains all {cohort['requested_assets']} frozen rating>=4 records. "
            f"The strata are {cohort['strict_compile_success']} strict compile successes and "
            f"{cohort['strict_compile_failure_recovered_urdf']} strict compile failures with "
            "recovered URDFs. Recovered exports are auditable, but none is promoted to a strict "
            "compile success."
        ),
        "",
        "## Table 7 row",
        "",
        "| Method | Watertight | Manifold | Open Edges | Degenerate Faces | Self-Intersection | Source KB | URDF KB | Mesh KB | Portable Package | Deterministic Build | Semantic Complete | Kinematic Complete | Physical Complete |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
        (
            "| Original Articraft (frozen N=143) "
            f"| {geometry['watertight']['per_asset_mean_fraction']:.6f} mean/asset; "
            f"{geometry['watertight']['geometry_level']['numerator']}/"
            f"{geometry['watertight']['geometry_level']['denominator']} geometries "
            f"| {geometry['manifold']['per_asset_mean_fraction']:.6f} mean/asset; "
            f"{geometry['manifold']['geometry_level']['numerator']}/"
            f"{geometry['manifold']['geometry_level']['denominator']} geometries "
            f"| {geometry['open_edges']['per_asset_mean']:.3f}/asset; {geometry['open_edges']['total']} total "
            f"| {geometry['degenerate_faces']['per_asset_mean']:.3f}/asset; {geometry['degenerate_faces']['total']} total "
            "| N/A "
            f"| {kb(size['source']['mean_bytes'])}/asset; {kb(size['source']['total_bytes'])} total "
            f"| {kb(size['urdf']['mean_bytes'])}/asset; {kb(size['urdf']['total_bytes'])} total "
            f"| {kb(size['mesh']['mean_bytes'])}/asset; {kb(size['mesh']['total_bytes'])} total "
            f"| {result['portable_package']['pass']}/{result['portable_package']['denominator']} "
            "| N/A (no two fresh builds) "
            "| N/A (field proxy separate) "
            f"| {result['kinematic_complete']['pass']}/{result['kinematic_complete']['denominator']} "
            f"| {result['physical_complete']['pass']}/{result['physical_complete']['denominator']} |"
        ),
        "",
        "Manifold is the frozen edge-manifold proxy (every undirected edge has at most two incident faces); vertex-manifold is not claimed.",
        "Geometry denominators contain only packaged or URDF-referenced readable triangle mesh payloads. The evaluator does not triangulate primitive-only URDF geometry.",
        (
            f"Geometry not evaluable: {geometry['geometry_not_evaluable_assets']}; "
            f"no mesh payload: {geometry['no_mesh_payload_assets']}; "
            f"mesh load errors: {geometry['mesh_load_error_assets']}."
        ),
        "",
        "## Cohort and evidence states",
        "",
        "| Cohort | Requested | Available | Geometry evaluable | Portable pass | Kinematic pass | Physical pass |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for label, data in [("All", summary), *summary["strata"].items()]:
        if label == "All":
            requested = cohort["requested_assets"]
            available = cohort["available_assets"]
            geom = cohort["geometry_evaluable_assets"]
            portable = result["portable_package"]["pass"]
            kinematic = result["kinematic_complete"]["pass"]
            physical = result["physical_complete"]["pass"]
        else:
            requested = data["requested_assets"]
            available = data["available_assets"]
            geom = data["geometry"]["geometry_evaluable_assets"]
            portable = data["portable_package"]["pass"]
            kinematic = data["kinematic_complete"]["pass"]
            physical = data["physical_complete"]["pass"]
        lines.append(
            f"| {label} | {requested} | {available} | {geom} | {portable}/{requested} | "
            f"{kinematic}/{requested} | {physical}/{requested} |"
        )
    semantic_proxy = result["semantic_field_proxy"]
    lines.extend(
        [
            "",
            "## Non-ranking field proxy",
            "",
            (
                f"The semantic named-tree field proxy passes {semantic_proxy['pass']}/"
                f"{semantic_proxy['denominator']} assets. This is not semantic completeness or "
                "semantic correctness; strict Semantic Complete remains not_evaluable for all assets."
            ),
            "",
            "## Completeness field gates",
            "",
            "| Axis | Gate | Passing assets |",
            "|---|---|---:|",
        ]
    )
    for axis in ("kinematic_complete", "physical_complete"):
        for gate, count in result[axis]["gate_pass_counts"].items():
            lines.append(
                f"| {axis} | {gate} | {count['numerator']}/{count['denominator']} |"
            )
    lines.extend(
        [
            "",
            "## Fail-closed exclusions",
            "",
            "- Self-intersection: not_evaluable; no exact backend excluding adjacent faces was run.",
            "- Deterministic build: not_evaluable; re-hashing an existing artifact was not treated as a rebuild.",
            "- Semantic completeness: not_evaluable; no output-independent required-part and role specification exists.",
            "- Physical metadata: simulator defaults from Table 6 do not count as native package fields.",
            "",
            "## Reproduction",
            "",
            "```bash",
            "python exp/scripts/run_table7_articraft.py",
            "python exp/scripts/run_table7_articraft.py --verify-only",
            "```",
            "",
            "Evidence: `protocol_snapshot.json`, `manifest.json`, `asset_records.json`, `summary.json`, and `self_check.json` in this directory.",
            "",
        ]
    )
    return "\n".join(lines)


def build_self_check(
    output: Path,
    protocol: dict[str, Any],
    manifest: dict[str, Any],
    records: list[dict[str, Any]],
    summary: dict[str, Any],
) -> dict[str, Any]:
    checks: list[dict[str, Any]] = []

    def add(name: str, passed: bool, detail: Any) -> None:
        checks.append({"name": name, "state": "pass" if passed else "fail", "detail": detail})

    ids = [row["record_id"] for row in manifest["records"]]
    record_ids = [row["record_id"] for row in records]
    add("manifest_identities_are_unique", len(ids) == len(set(ids)) == EXPECTED_REQUESTED, len(ids))
    add("asset_record_identities_match_manifest_order", record_ids == ids, len(record_ids))
    add(
        "requested_denominator_is_frozen",
        summary["cohort"]["requested_assets"] == EXPECTED_REQUESTED,
        summary["cohort"],
    )
    add(
        "strict_strata_are_126_plus_17",
        summary["cohort"]["strict_compile_success"] == EXPECTED_STRICT
        and summary["cohort"]["strict_compile_failure_recovered_urdf"]
        == EXPECTED_RECOVERED_FAILURE,
        {
            "strict_compile_success": summary["cohort"]["strict_compile_success"],
            "strict_compile_failure_recovered_urdf": summary["cohort"][
                "strict_compile_failure_recovered_urdf"
            ],
        },
    )
    add(
        "recovered_urdfs_are_never_strict_success",
        all(
            not row["strict_compile_success"]
            and not row["recovered_urdf_is_strict_success"]
            for row in records
            if row["strict_failure_export_recovered"]
        ),
        EXPECTED_RECOVERED_FAILURE,
    )
    for axis in (
        "portable_package",
        "deterministic_build",
        "semantic_complete",
        "semantic_field_proxy",
        "kinematic_complete",
        "physical_complete",
    ):
        partition = summary["results"][axis]
        total = partition["pass"] + partition["fail"] + partition["not_evaluable"]
        add(
            f"{axis}_state_partition_conserves_requested_denominator",
            total == EXPECTED_REQUESTED and partition["denominator"] == EXPECTED_REQUESTED,
            partition,
        )
    add(
        "geometry_denominators_are_explicit",
        summary["results"]["geometry"]["requested_assets"] == EXPECTED_REQUESTED
        and summary["results"]["geometry"]["geometry_evaluable_asset_rate"]["denominator"]
        == EXPECTED_REQUESTED,
        summary["results"]["geometry"]["geometry_evaluable_asset_rate"],
    )
    add(
        "all_frozen_source_hashes_match",
        summary["integrity"]["source_hash_matches"]["numerator"] == EXPECTED_REQUESTED,
        summary["integrity"]["source_hash_matches"],
    )
    add(
        "all_table6_urdf_hashes_match",
        summary["integrity"]["urdf_hash_matches_table6"]["numerator"] == EXPECTED_REQUESTED,
        summary["integrity"]["urdf_hash_matches_table6"],
    )
    protocol_hash = sha256_file(output / "protocol_snapshot.json")
    add(
        "protocol_snapshot_hash_matches_frozen_source",
        protocol_hash == sha256_file(PROTOCOL_SOURCE) == manifest["protocol_snapshot_sha256"],
        protocol_hash,
    )
    add(
        "protocol_id_matches_snapshot",
        protocol["protocol_id"] == manifest["protocol_id"],
        protocol["protocol_id"],
    )
    all_evidence_inside = True
    all_existing_evidence_present = True
    evidence_detail = []
    for relative in summary["evidence_files"].values():
        candidate = ensure_within(REPO_ROOT / relative, strict=False)
        all_evidence_inside = all_evidence_inside and (
            candidate == WORKSPACE_ROOT or WORKSPACE_ROOT in candidate.parents
        )
        if candidate.name != "self_check.json":
            all_existing_evidence_present = all_existing_evidence_present and candidate.is_file()
        evidence_detail.append(relative)
    add("all_evidence_paths_are_inside_workspace", all_evidence_inside, evidence_detail)
    add(
        "all_non_circular_evidence_files_exist",
        all_existing_evidence_present,
        evidence_detail,
    )
    scratch = output / ".portable_scratch"
    scratch_empty = scratch.is_dir() and not any(scratch.iterdir())
    add("portable_scratch_is_empty", scratch_empty, repo_relative(scratch))

    hash_names = [
        "protocol_snapshot.json",
        "manifest.json",
        "asset_records.json",
        "summary.json",
        "report.md",
    ]
    artifact_hashes = {name: sha256_file(output / name) for name in hash_names}
    overall = "pass" if all(check["state"] == "pass" for check in checks) else "fail"
    return {
        "schema_version": "1.0.0",
        "state": overall,
        "checks": checks,
        "input_hashes": manifest["frozen_inputs"],
        "artifact_hashes": artifact_hashes,
        "artifact_hash_scope": "all required outputs except self_check.json to avoid a circular hash",
        "required_outputs": protocol["required_outputs"],
    }


def verify_output(output: Path) -> dict[str, Any]:
    output = ensure_within(output, strict=True)
    self_check = read_json(output / "self_check.json")
    manifest = read_json(output / "manifest.json")
    records = read_json(output / "asset_records.json")
    summary = read_json(output / "summary.json")
    protocol = read_json(output / "protocol_snapshot.json")
    errors: list[str] = []
    for name in protocol["required_outputs"]:
        path = ensure_within(output / name, strict=False)
        if not path.is_file():
            errors.append(f"missing required output: {name}")
    for name, expected in self_check.get("artifact_hashes", {}).items():
        path = output / name
        if not path.is_file() or sha256_file(path) != expected:
            errors.append(f"artifact hash mismatch: {name}")
    manifest_ids = [row["record_id"] for row in manifest.get("records", [])]
    record_ids = [row["record_id"] for row in records]
    if len(manifest_ids) != EXPECTED_REQUESTED or len(set(manifest_ids)) != EXPECTED_REQUESTED:
        errors.append("manifest does not contain 143 unique identities")
    if record_ids != manifest_ids:
        errors.append("asset record identities do not match manifest order")
    if summary.get("cohort", {}).get("requested_assets") != EXPECTED_REQUESTED:
        errors.append("summary requested denominator is not 143")
    if summary.get("cohort", {}).get("strict_compile_success") != EXPECTED_STRICT:
        errors.append("summary strict stratum is not 126")
    if summary.get("cohort", {}).get("strict_compile_failure_recovered_urdf") != EXPECTED_RECOVERED_FAILURE:
        errors.append("summary recovered strict-failure stratum is not 17")
    if sha256_file(output / "protocol_snapshot.json") != manifest.get("protocol_snapshot_sha256"):
        errors.append("protocol snapshot hash differs from manifest")
    if self_check.get("state") != "pass":
        errors.append("generated self_check state is not pass")
    return {
        "state": "pass" if not errors else "fail",
        "errors": errors,
        "requested_assets": len(records),
        "strict_compile_success": sum(row["strict_compile_success"] for row in records),
        "strict_compile_failure_recovered_urdf": sum(
            row["strict_failure_export_recovered"] for row in records
        ),
    }


def run(output: Path) -> dict[str, Any]:
    output = ensure_within(output, strict=False)
    output.mkdir(parents=True, exist_ok=True)
    output = ensure_within(output, strict=True)
    protocol, cohort, table6_manifest = load_frozen_cohort()

    snapshot = output / "protocol_snapshot.json"
    temporary_snapshot = output / "protocol_snapshot.json.tmp"
    ensure_within(temporary_snapshot, strict=False)
    shutil.copyfile(ensure_within(PROTOCOL_SOURCE, strict=True), temporary_snapshot)
    temporary_snapshot.replace(snapshot)
    manifest = make_manifest(protocol, cohort, table6_manifest)
    write_json(output / "manifest.json", manifest)

    scratch_root = output / ".portable_scratch"
    if scratch_root.exists():
        scratch_root = ensure_within(scratch_root, strict=True)
        shutil.rmtree(scratch_root)
    scratch_root.mkdir(parents=True)
    scratch_root = ensure_within(scratch_root, strict=True)

    records: list[dict[str, Any]] = []
    for index, row in enumerate(cohort, start=1):
        record = audit_asset(row, scratch_root)
        records.append(record)
        print(
            f"[{index:03d}/{len(cohort)}] {row['record_id']} "
            f"stratum={row['cohort_stratum']} portable={record['portable_package']['state']} "
            f"geometries={record['geometry']['readable_geometry_count']}",
            flush=True,
        )
    write_json(output / "asset_records.json", records)
    summary = summarize(records, manifest)
    write_json(output / "summary.json", summary)
    report_path = output / "report.md"
    report_path.write_text(report_text(summary), encoding="utf-8")
    self_check = build_self_check(output, protocol, manifest, records, summary)
    write_json(output / "self_check.json", self_check)
    if self_check["state"] != "pass":
        failed = [row["name"] for row in self_check["checks"] if row["state"] != "pass"]
        raise RuntimeError(f"generated self-check failed: {failed}")
    verification = verify_output(output)
    if verification["state"] != "pass":
        raise RuntimeError(f"post-write verification failed: {verification['errors']}")
    return verification


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help="output directory inside the authorized workspace",
    )
    parser.add_argument(
        "--verify-only",
        action="store_true",
        help="verify existing output denominators and hashes without changing files",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output = ensure_within(args.output if args.output.is_absolute() else Path.cwd() / args.output, strict=False)
    if args.verify_only:
        result = verify_output(output)
    else:
        result = run(output)
    print(json.dumps(result, sort_keys=True))
    return 0 if result["state"] == "pass" else 1


if __name__ == "__main__":
    raise SystemExit(main())
