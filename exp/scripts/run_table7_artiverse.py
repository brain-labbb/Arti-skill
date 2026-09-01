#!/usr/bin/env python3
"""Audit the frozen Artiverse pre-release subset under the Table 7 protocol.

The official chunk manifest fixes all requested identities before scoring.  The
runner refuses to score a partial extraction.  It does not rebuild, repair, or
select assets based on their audit outcome.
"""

from __future__ import annotations

import argparse
from collections import Counter, deque
from concurrent.futures import ProcessPoolExecutor, as_completed
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


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
WORKSPACE = REPO.parent.resolve()
PROTOCOL = REPO / "exp/reference/table7_production_readiness_protocol_v1.json"
DATASET = REPO / "exp/artiverse"
DATA_ROOT = DATASET / "data"
CHUNKS = DATASET / "dataset_chunks"
CHUNK_MANIFEST = CHUNKS / "manifest.json"
OFFICIAL_CODE = REPO / ".cache/table6_sources/artiverse/code"
DEFAULT_OUTPUT = REPO / "exp/runtime/table7_artiverse"

EXPECTED_FORMAT = "artiverse-data-tar-gz-chunks-v1"
EXPECTED_REQUESTED = 3544
EXPECTED_FILE_COUNT = 531937
EXPECTED_INPUT_BYTES = 86992752890
EXPECTED_ARCHIVE_BYTES = 65334141104
EXPECTED_CODE_COMMIT = "44f3d41d015018e9b4dff2cbf01fd0892fe6b2c5"
EXPECTED_DATASET_REVISION = "8c4b120418e7cbdf9ac4c9580c5dbfdbf128a248"
EXPECTED_PROTOCOL_ID = "nano3d_table7_production_readiness_v1"
AUDIT_IDENTITY = "FROZEN_PRE_RELEASE_SUBSET_STATIC_AUDIT"
RELEASE_IDENTITY = "subset of pre-release Artiverse assets, per local README"

REQUIRED = (
    "protocol_snapshot.json",
    "manifest.json",
    "asset_records.json",
    "summary.json",
    "self_check.json",
    "report.md",
)
MESH_SUFFIXES = {".obj", ".stl", ".ply", ".off", ".dae", ".glb", ".gltf"}
SOURCE_SUFFIXES = {".py", ".sh", ".js", ".mjs", ".cjs", ".ts", ".tsx"}
ONE_AXIS_JOINT_TYPES = {"revolute", "continuous", "prismatic"}
RECOGNIZED_JOINT_TYPES = {
    "fixed", "revolute", "continuous", "prismatic", "planar", "floating"
}
PLACEHOLDER_NAME = re.compile(
    r"^(?:link|part|mesh|object|component|unnamed|default)(?:[_-]?\d+)?$",
    re.IGNORECASE,
)
URI_SCHEME = re.compile(r"^[A-Za-z][A-Za-z0-9+.-]*:")


def safe(path: Path, *, must_exist: bool = True) -> Path:
    root = WORKSPACE.resolve(strict=True)
    resolved = path.resolve(strict=must_exist)
    if resolved != root and root not in resolved.parents:
        raise RuntimeError(f"path outside authorized workspace: {resolved}")
    return resolved


def rel(path: Path) -> str:
    return safe(path, must_exist=False).relative_to(WORKSPACE).as_posix()


def read_json(path: Path) -> Any:
    return json.loads(safe(path).read_text(encoding="utf-8"))


def write_json(path: Path, value: Any) -> None:
    destination = safe(path, must_exist=False)
    safe(destination.parent).mkdir(parents=True, exist_ok=True)
    temporary = safe(destination.with_suffix(destination.suffix + ".tmp"), must_exist=False)
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(destination)


def write_text(path: Path, value: str) -> None:
    destination = safe(path, must_exist=False)
    safe(destination.parent).mkdir(parents=True, exist_ok=True)
    temporary = safe(destination.with_suffix(destination.suffix + ".tmp"), must_exist=False)
    temporary.write_text(value, encoding="utf-8")
    temporary.replace(destination)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with safe(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_sha256(value: Any) -> str:
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True)
    return hashlib.sha256(encoded.encode("utf-8")).hexdigest()


def ratio(numerator: int, denominator: int) -> dict[str, Any]:
    return {
        "numerator": numerator,
        "denominator": denominator,
        "rate": numerator / denominator if denominator else None,
    }


def finite_number(raw: str | None) -> float | None:
    if raw is None:
        return None
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def finite_vector(raw: str | None, *, default_zero: bool = False) -> bool:
    if raw is None:
        return default_zero
    try:
        values = [float(item) for item in raw.split()]
    except ValueError:
        return False
    return len(values) == 3 and all(math.isfinite(value) for value in values)


def local_tag(node: ET.Element) -> str:
    return node.tag.rsplit("}", 1)[-1].lower()


def checkout_commit() -> str:
    git = safe(OFFICIAL_CODE / ".git")
    head = safe(git / "HEAD").read_text(encoding="utf-8").strip()
    if head.startswith("ref: "):
        ref = head[5:].strip()
        if ".." in Path(ref).parts or not re.fullmatch(r"refs/[A-Za-z0-9._/-]+", ref):
            raise RuntimeError(f"unsafe git ref: {ref!r}")
        ref_path = safe(git / ref, must_exist=False)
        if ref_path.is_file():
            commit = ref_path.read_text(encoding="utf-8").strip()
        else:
            packed = safe(git / "packed-refs").read_text(encoding="utf-8")
            matches = [line.split()[0] for line in packed.splitlines() if line.endswith(f" {ref}")]
            commit = matches[0] if len(matches) == 1 else ""
    else:
        commit = head
    if not re.fullmatch(r"[0-9a-f]{40}", commit):
        raise RuntimeError("could not resolve official Artiverse checkout commit")
    if commit != EXPECTED_CODE_COMMIT:
        raise RuntimeError(f"official code commit drifted: {commit}")
    return commit


def source_inventory() -> dict[str, Any]:
    root = safe(OFFICIAL_CODE)
    entries: list[dict[str, Any]] = []
    for directory, directory_names, file_names in os.walk(root, followlinks=False):
        current = safe(Path(directory))
        if current == root and ".git" in directory_names:
            directory_names.remove(".git")
        directory_names.sort()
        for name in sorted(file_names):
            path = current / name
            if path.suffix.lower() not in SOURCE_SUFFIXES or path.is_symlink():
                continue
            path = safe(path)
            entries.append(
                {
                    "path": path.relative_to(root).as_posix(),
                    "bytes": path.stat().st_size,
                    "sha256": sha256_file(path),
                }
            )
    return {
        "scope": "shared official executable source; .git excluded",
        "code_commit": checkout_commit(),
        "file_count": len(entries),
        "total_bytes": sum(row["bytes"] for row in entries),
        "content_manifest_sha256": canonical_sha256(entries),
        "files": entries,
    }


def load_frozen_cohort() -> tuple[dict[str, Any], list[dict[str, Any]]]:
    manifest = read_json(CHUNK_MANIFEST)
    if manifest.get("format") != EXPECTED_FORMAT:
        raise RuntimeError("unexpected Artiverse chunk-manifest format")
    if manifest.get("model_count") != EXPECTED_REQUESTED:
        raise RuntimeError("Artiverse requested cohort size drifted")
    if manifest.get("file_count") != EXPECTED_FILE_COUNT:
        raise RuntimeError("Artiverse frozen file count drifted")
    if manifest.get("input_bytes") != EXPECTED_INPUT_BYTES:
        raise RuntimeError("Artiverse frozen input bytes drifted")
    chunks = manifest.get("chunks")
    if not isinstance(chunks, list) or len(chunks) != manifest.get("chunk_count"):
        raise RuntimeError("invalid chunk entries")

    roots: list[str] = []
    archive_total = 0
    for chunk in chunks:
        archive = chunk.get("archive")
        if not isinstance(archive, str) or Path(archive).name != archive:
            raise RuntimeError(f"unsafe archive name: {archive!r}")
        archive_path = safe(CHUNKS / archive)
        actual_bytes = archive_path.stat().st_size
        if actual_bytes != chunk.get("archive_bytes"):
            raise RuntimeError(f"archive size mismatch: {archive}")
        archive_total += actual_bytes
        chunk_roots = chunk.get("roots")
        if not isinstance(chunk_roots, list) or len(chunk_roots) != chunk.get("model_count"):
            raise RuntimeError(f"invalid roots for archive: {archive}")
        roots.extend(chunk_roots)
    if archive_total != EXPECTED_ARCHIVE_BYTES:
        raise RuntimeError("frozen archive-byte total drifted")
    if len(roots) != EXPECTED_REQUESTED or len(set(roots)) != EXPECTED_REQUESTED:
        raise RuntimeError("cohort root identities are not exactly 3,544 unique values")

    cohort: list[dict[str, Any]] = []
    for index, root_text in enumerate(roots):
        path = Path(root_text)
        if path.is_absolute() or ".." in path.parts or len(path.parts) != 4 or path.parts[0] != "data":
            raise RuntimeError(f"unsafe or malformed model root: {root_text!r}")
        cohort.append(
            {
                "selection_index": index,
                "asset_id": root_text,
                "category": path.parts[1],
                "source_repository": path.parts[2],
                "model_id": path.parts[3],
                "asset_root": f"arti-skill/exp/artiverse/{root_text}",
                "primary_package": f"arti-skill/exp/artiverse/{root_text}/urdf_w_collider",
            }
        )
    return manifest, cohort


def tree_preflight(*, verify_archive_hashes: bool, quick: bool = False) -> dict[str, Any]:
    chunk_manifest, cohort = load_frozen_cohort()
    present_roots = 0
    file_count = 0
    input_bytes = 0
    urdf_roots = 0
    segmented_roots = 0
    articulation_roots = 0
    material_roots = 0
    missing_roots: list[str] = []
    known_root_paths = {safe(WORKSPACE / row["asset_root"], must_exist=False) for row in cohort}
    observed_root_paths: set[Path] = set()
    if DATA_ROOT.is_dir():
        for category in sorted(path for path in safe(DATA_ROOT).iterdir() if path.is_dir()):
            category = safe(category)
            for source in sorted(path for path in category.iterdir() if path.is_dir()):
                source = safe(source)
                observed_root_paths.update(
                    safe(path) for path in source.iterdir() if path.is_dir()
                )
    unexpected_roots = sorted(
        rel(path) for path in observed_root_paths - known_root_paths
    )
    malformed_urdf_roots: list[dict[str, Any]] = []
    observed_per_root: dict[str, dict[str, int]] = {}
    for row in cohort:
        root = safe(WORKSPACE / row["asset_root"], must_exist=False)
        if not root.is_dir():
            missing_roots.append(row["asset_id"])
            continue
        present_roots += 1
        if not quick:
            root_file_count = 0
            root_input_bytes = 0
            for directory, directory_names, file_names in os.walk(root, followlinks=False):
                current = safe(Path(directory))
                directory_names.sort()
                file_names.sort()
                for name in file_names:
                    path = current / name
                    if path.is_symlink():
                        continue
                    path = safe(path)
                    file_count += 1
                    input_bytes += path.stat().st_size
                    root_file_count += 1
                    root_input_bytes += path.stat().st_size
            observed_per_root[row["asset_id"]] = {
                "file_count": root_file_count,
                "input_bytes": root_input_bytes,
            }
        package = root / "urdf_w_collider"
        urdfs = sorted(package.glob("*.urdf")) if package.is_dir() else []
        if len(urdfs) == 1:
            urdf_roots += 1
        else:
            malformed_urdf_roots.append(
                {"asset_id": row["asset_id"], "urdf_candidate_count": len(urdfs)}
            )
        if len(list(root.glob("*.segmented.glb"))) == 1:
            segmented_roots += 1
        if len(list(root.glob("*.articulations.json"))) == 1:
            articulation_roots += 1
        if (root / "material.json").is_file():
            material_roots += 1

    archives: list[dict[str, Any]] = []
    archive_hashes_match: bool | None = True if verify_archive_hashes else None
    for chunk in chunk_manifest["chunks"]:
        path = safe(CHUNKS / chunk["archive"])
        actual_hash = sha256_file(path) if verify_archive_hashes else None
        matches = actual_hash == chunk["sha256"] if verify_archive_hashes else None
        if matches is False:
            archive_hashes_match = False
        archives.append(
            {
                "path": rel(path),
                "bytes": path.stat().st_size,
                "manifest_sha256": chunk["sha256"],
                "recomputed_sha256": actual_hash,
                "sha256_matches": matches,
            }
        )

    complete = (
        present_roots == EXPECTED_REQUESTED
        and not unexpected_roots
        and (quick or file_count == EXPECTED_FILE_COUNT)
        and (quick or input_bytes == EXPECTED_INPUT_BYTES)
        and (archive_hashes_match is not False)
    )
    status = (
        "INCOMPLETE"
        if not complete
        else "ROOTS_COMPLETE_FULL_TREE_UNVERIFIED" if quick else "COMPLETE"
    )
    per_root_matches_manifest: bool | None = None
    per_root_mismatch_count = 0
    per_root_mismatch_examples: list[dict[str, Any]] = []
    if not quick:
        # The public chunk manifest records totals per archive, not per root.
        # Exact tree equivalence is therefore asserted by global totals plus
        # the unique root identity set; retain observed per-root evidence for
        # diagnostics without inventing unavailable expected values.
        per_root_matches_manifest = None
        if len(observed_per_root) != present_roots:
            per_root_mismatch_count = present_roots - len(observed_per_root)
            per_root_mismatch_examples = [
                {"reason": "root observed without complete file inventory"}
            ]
    return {
        "status": status,
        "scoring_allowed": complete and not quick,
        "manifest": {
            "path": rel(CHUNK_MANIFEST),
            "sha256": sha256_file(CHUNK_MANIFEST),
            "format": chunk_manifest["format"],
            "created_utc": chunk_manifest["created_utc"],
        },
        "requested_roots": EXPECTED_REQUESTED,
        "present_roots": present_roots,
        "missing_root_count": len(missing_roots),
        "missing_roots": missing_roots,
        "observed_root_count": len(observed_root_paths),
        "unexpected_root_count": len(unexpected_roots),
        "unexpected_roots": unexpected_roots,
        "expected_file_count": EXPECTED_FILE_COUNT,
        "observed_file_count": file_count,
        "expected_input_bytes": EXPECTED_INPUT_BYTES,
        "observed_input_bytes": input_bytes,
        "roots_with_one_primary_urdf": urdf_roots,
        "roots_with_one_segmented_glb": segmented_roots,
        "roots_with_one_articulation_json": articulation_roots,
        "roots_with_material_json": material_roots,
        "malformed_primary_urdf_roots": malformed_urdf_roots,
        "archive_hashes_recomputed": verify_archive_hashes,
        "archive_hashes_match": archive_hashes_match,
        "quick_root_only": quick,
        "per_root_inventory_count": len(observed_per_root),
        "per_root_expected_values_available": False,
        "per_root_matches_manifest": per_root_matches_manifest,
        "per_root_mismatch_count": per_root_mismatch_count,
        "per_root_mismatch_examples": per_root_mismatch_examples,
        "archives": archives,
    }


def validated_preflight_snapshot(path: Path) -> dict[str, Any]:
    """Load a completed full preflight without re-reading both 65 GB archives."""

    path = safe(path)
    value = read_json(path)
    exact = {
        "status": "COMPLETE",
        "scoring_allowed": True,
        "quick_root_only": False,
        "requested_roots": EXPECTED_REQUESTED,
        "present_roots": EXPECTED_REQUESTED,
        "observed_root_count": EXPECTED_REQUESTED,
        "missing_root_count": 0,
        "unexpected_root_count": 0,
        "observed_file_count": EXPECTED_FILE_COUNT,
        "expected_file_count": EXPECTED_FILE_COUNT,
        "observed_input_bytes": EXPECTED_INPUT_BYTES,
        "expected_input_bytes": EXPECTED_INPUT_BYTES,
        "roots_with_one_primary_urdf": EXPECTED_REQUESTED,
        "roots_with_one_segmented_glb": EXPECTED_REQUESTED,
        "roots_with_one_articulation_json": EXPECTED_REQUESTED,
        "roots_with_material_json": EXPECTED_REQUESTED,
        "per_root_inventory_count": EXPECTED_REQUESTED,
        "per_root_mismatch_count": 0,
        "archive_hashes_recomputed": True,
        "archive_hashes_match": True,
    }
    drift = {
        key: {"expected": expected, "observed": value.get(key)}
        for key, expected in exact.items()
        if value.get(key) != expected
    }
    if drift:
        raise RuntimeError(f"full-preflight snapshot is not exact: {drift}")
    if value.get("manifest", {}).get("sha256") != sha256_file(CHUNK_MANIFEST):
        raise RuntimeError("full-preflight snapshot chunk-manifest hash drifted")
    chunk_manifest = read_json(CHUNK_MANIFEST)
    expected_archives = {
        rel(CHUNKS / row["archive"]): row for row in chunk_manifest["chunks"]
    }
    observed_archives = {
        row.get("path"): row for row in value.get("archives", [])
        if isinstance(row, dict)
    }
    if set(observed_archives) != set(expected_archives):
        raise RuntimeError("full-preflight archive identities drifted")
    for archive_path, expected in expected_archives.items():
        observed = observed_archives[archive_path]
        current_archive = safe(CHUNKS / expected["archive"])
        if not (
            current_archive.is_file()
            and current_archive.stat().st_size == expected["archive_bytes"]
            and observed.get("bytes") == expected["archive_bytes"]
            and observed.get("manifest_sha256") == expected["sha256"]
            and observed.get("recomputed_sha256") == expected["sha256"]
            and observed.get("sha256_matches") is True
        ):
            raise RuntimeError(f"full-preflight archive evidence drifted: {archive_path}")
    value["snapshot_evidence"] = {"path": rel(path), "sha256": sha256_file(path)}
    return value


def preflight_snapshot_matches_manifest(manifest: dict[str, Any]) -> bool:
    try:
        frozen_inputs = manifest["frozen_inputs"]
        evidence = frozen_inputs["full_preflight_snapshot"]
        if not isinstance(evidence, dict):
            return False
        path = safe(WORKSPACE / evidence["path"])
        if sha256_file(path) != evidence["sha256"]:
            return False
        snapshot = validated_preflight_snapshot(path)
        frozen_tree = frozen_inputs["tree_preflight"]
        return (
            snapshot["snapshot_evidence"] == evidence
            and snapshot["manifest"] == frozen_inputs["chunk_manifest"]
            and snapshot["archives"] == frozen_inputs["archives"]
            and all(snapshot.get(key) == value for key, value in frozen_tree.items())
        )
    except (KeyError, OSError, RuntimeError, TypeError, ValueError):
        return False


def package_files(package: Path) -> list[Path]:
    files: list[Path] = []
    if not package.is_dir():
        return files
    for directory, directory_names, file_names in os.walk(package, followlinks=False):
        current = safe(Path(directory))
        directory_names.sort()
        file_names.sort()
        for name in file_names:
            path = current / name
            if not path.is_symlink():
                files.append(safe(path))
    return files


def inventory_signature(package: Path) -> dict[str, Any]:
    entries = [
        {"path": path.relative_to(package).as_posix(), "bytes": path.stat().st_size}
        for path in package_files(package)
    ]
    entries.sort(key=lambda row: row["path"])
    return {
        "file_count": len(entries),
        "total_bytes": sum(row["bytes"] for row in entries),
        "path_size_manifest_sha256": canonical_sha256(entries),
    }


def urdf_dependency_specs(root: ET.Element) -> list[tuple[str, str, str]]:
    """Return declared dependencies with their representation role.

    Visual and collision mesh closures stay separate so the headline geometry
    score cannot silently double-count collision or convenience representations.
    """

    specs: list[tuple[str, str, str]] = []
    for visual in root.findall(".//visual"):
        for node in visual.findall(".//geometry/mesh"):
            specs.append(("urdf_visual_mesh", node.attrib.get("filename", ""), "visual"))
    for collision in root.findall(".//collision"):
        for node in collision.findall(".//geometry/mesh"):
            specs.append(("urdf_collision_mesh", node.attrib.get("filename", ""), "collision"))
    for node in root.findall(".//texture"):
        if "filename" in node.attrib:
            specs.append(("urdf_texture", node.attrib["filename"], "visual"))
    return specs


def nested_dependency_specs(path: Path) -> list[tuple[str, str]]:
    specs: list[tuple[str, str]] = []
    suffix = path.suffix.lower()
    if suffix == ".obj":
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            stripped = line.strip()
            if not stripped.lower().startswith("mtllib "):
                continue
            try:
                values = shlex.split(stripped[len("mtllib "):])
            except ValueError:
                values = [stripped[len("mtllib "):].strip()]
            specs.extend(("obj_mtllib", value) for value in values if value)
    elif suffix == ".mtl":
        map_keys = {
            "map_ka", "map_kd", "map_ks", "map_ke", "map_d", "bump",
            "map_bump", "disp", "decal", "refl",
        }
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            try:
                values = shlex.split(line, comments=True)
            except ValueError:
                continue
            if len(values) >= 2 and values[0].lower() in map_keys:
                specs.append(("mtl_texture", values[-1]))
    elif suffix == ".gltf":
        payload = json.loads(path.read_text(encoding="utf-8"))
        for section in ("buffers", "images"):
            for item in payload.get(section, []):
                uri = item.get("uri")
                if isinstance(uri, str) and not uri.startswith("data:"):
                    specs.append((f"gltf_{section[:-1]}", uri))
    elif suffix == ".dae":
        dae = ET.parse(path).getroot()
        for node in dae.iter():
            if local_tag(node) == "init_from" and node.text and node.text.strip():
                specs.append(("dae_image", node.text.strip()))
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
    resolved = lexical.resolve(strict=False)
    try:
        resolved.relative_to(package)
    except ValueError:
        return None, f"dependency escapes package: {raw}", False
    uses_symlink = False
    cursor = declaring_file.parent
    for part in Path(raw).parts:
        cursor = cursor / part
        if cursor.is_symlink():
            uses_symlink = True
    if not resolved.is_file():
        return None, f"dependency is missing: {raw}", uses_symlink
    return safe(resolved), None, uses_symlink


def dependency_closure(primary: Path, package: Path) -> dict[str, Any]:
    root = ET.parse(primary).getroot()
    queue: deque[tuple[str, str, Path, str]] = deque(
        (kind, raw, primary, scope) for kind, raw, scope in urdf_dependency_specs(root)
    )
    seen: set[tuple[str, str, str, str]] = set()
    visited: set[tuple[Path, str]] = set()
    mesh_paths: set[Path] = set()
    visual_mesh_paths: set[Path] = set()
    collision_mesh_paths: set[Path] = set()
    issues: list[str] = []
    issues_by_scope: dict[str, list[str]] = {"visual": [], "collision": []}
    visual_mesh_issues: list[str] = []
    collision_mesh_issues: list[str] = []
    dependency_count = 0
    relative_count = 0
    existing_count = 0
    symlink_count = 0
    while queue:
        kind, raw, declaring, scope = queue.popleft()
        key = (kind, raw, declaring.relative_to(package).as_posix(), scope)
        if key in seen:
            continue
        seen.add(key)
        dependency_count += 1
        relative = not bool(URI_SCHEME.match(raw)) and not Path(raw).is_absolute() and "\\" not in raw
        relative_count += int(relative)
        resolved, error, uses_symlink = resolve_dependency(package, declaring, raw)
        symlink_count += int(uses_symlink)
        if error:
            issue = f"{key[2]}: {error}"
            issues.append(issue)
            issues_by_scope[scope].append(issue)
            if kind == "urdf_visual_mesh":
                visual_mesh_issues.append(issue)
            elif kind == "urdf_collision_mesh":
                collision_mesh_issues.append(issue)
            continue
        assert resolved is not None
        existing_count += 1
        if resolved.suffix.lower() in MESH_SUFFIXES:
            mesh_paths.add(resolved)
            if scope == "visual":
                visual_mesh_paths.add(resolved)
            elif scope == "collision":
                collision_mesh_paths.add(resolved)
        visit_key = (resolved, scope)
        if visit_key in visited:
            continue
        visited.add(visit_key)
        try:
            for nested_kind, nested_raw in nested_dependency_specs(resolved):
                queue.append((nested_kind, nested_raw, resolved, scope))
        except Exception as exc:  # noqa: BLE001
            issue = (
                f"cannot parse nested dependencies in {resolved.relative_to(package)}: "
                f"{type(exc).__name__}: {exc}"
            )
            issues.append(issue)
            issues_by_scope[scope].append(issue)
    return {
        "root": root,
        "dependency_count": dependency_count,
        "relative_dependency_count": relative_count,
        "existing_dependency_count": existing_count,
        "dependency_symlink_count": symlink_count,
        "mesh_paths": sorted(mesh_paths),
        "visual_mesh_paths": sorted(visual_mesh_paths),
        "collision_mesh_paths": sorted(collision_mesh_paths),
        "issues": issues,
        "issues_by_scope": issues_by_scope,
        "visual_mesh_issues": visual_mesh_issues,
        "collision_mesh_issues": collision_mesh_issues,
    }


def portable_package_audit(package: Path, primary: Path | None, scratch_root: Path) -> dict[str, Any]:
    if not package.is_dir():
        return {
            "state": "not_evaluable",
            "reason": "no native simulation package is available",
            "fresh_workspace_copy": False,
            "scratch_removed": True,
            "issues": [],
        }
    package = safe(package)
    primary = safe(primary) if primary is not None else None
    original_inventory = inventory_signature(package)
    prefix = hashlib.sha256(str(package).encode("utf-8")).hexdigest()[:12] + "-"
    temporary = safe(Path(tempfile.mkdtemp(prefix=prefix, dir=safe(scratch_root))))
    copied = safe(temporary / "package", must_exist=False)
    issues: list[str] = []
    closure: dict[str, Any] | None = None
    copied_inventory: dict[str, Any] | None = None
    try:
        shutil.copytree(package, copied, symlinks=True)
        copied = safe(copied)
        copied_inventory = inventory_signature(copied)
        if copied_inventory != original_inventory:
            issues.append("complete-package path/size inventory differs after clean copy")
        if primary is None:
            issues.append("copied package has no unambiguous top-level primary URDF")
        else:
            copied_primary = safe(copied / primary.relative_to(package))
            try:
                closure = dependency_closure(copied_primary, copied)
                issues.extend(closure["issues"])
            except Exception as exc:  # noqa: BLE001
                issues.append(f"copied primary parse failed: {type(exc).__name__}: {exc}")
    finally:
        shutil.rmtree(temporary)

    passed = closure is not None and not issues
    return {
        "state": "pass" if passed else "fail",
        "reason": None if passed else "one or more clean-copy package gates failed",
        "fresh_workspace_copy": True,
        "complete_primary_package_copied": True,
        "scratch_removed": not temporary.exists(),
        "copy_inventory_equal": copied_inventory == original_inventory,
        "original_inventory": original_inventory,
        "copied_inventory": copied_inventory,
        "primary_artifact_parsed": closure is not None,
        "declared_dependency_count": closure["dependency_count"] if closure else 0,
        "all_dependencies_relative": bool(
            closure and closure["relative_dependency_count"] == closure["dependency_count"]
        ),
        "all_dependencies_exist": bool(
            closure and closure["existing_dependency_count"] == closure["dependency_count"]
        ),
        "required_dependency_symlink_count": closure["dependency_symlink_count"] if closure else 0,
        "copied_dependency_declarations_parsed": closure is not None,
        "copied_declared_mesh_count": len(closure["mesh_paths"]) if closure else 0,
        "mesh_readability_scored_separately": True,
        "issues": issues,
    }


def geometry_audit(
    mesh_paths: list[Path], package: Path, initial_errors: Iterable[str] = (),
) -> dict[str, Any]:
    geometries: list[dict[str, Any]] = []
    load_errors = list(initial_errors)
    readable_meshes = 0
    for path in mesh_paths:
        relative = path.relative_to(package).as_posix()
        try:
            loaded = trimesh.load(path, force="scene", process=False)
            items = list(getattr(loaded, "geometry", {}).items())
            if not items:
                raise ValueError("no geometry in loaded scene")
            readable_meshes += 1
            for name, mesh in items:
                faces = np.asarray(mesh.faces)
                if faces.ndim != 2 or faces.shape[1] != 3 or len(faces) == 0:
                    raise ValueError(f"non-triangle or empty geometry: {name}")
                edges = np.asarray(mesh.edges_sorted)
                _unique, edge_counts = np.unique(edges, axis=0, return_counts=True)
                open_edges = int(np.count_nonzero(edge_counts == 1))
                nonmanifold_edges = int(np.count_nonzero(edge_counts > 2))
                repeated = (
                    (faces[:, 0] == faces[:, 1])
                    | (faces[:, 1] == faces[:, 2])
                    | (faces[:, 0] == faces[:, 2])
                )
                areas = np.asarray(mesh.area_faces)
                area_bad = ~np.isfinite(areas) | (areas <= 1e-12)
                degenerate = repeated | area_bad
                geometries.append(
                    {
                        "mesh_path": relative,
                        "geometry_name": str(name),
                        "vertices": int(len(mesh.vertices)),
                        "faces": int(len(faces)),
                        "watertight": bool(mesh.is_watertight),
                        "edge_manifold_proxy": nonmanifold_edges == 0,
                        "open_edges": open_edges,
                        "nonmanifold_edges": nonmanifold_edges,
                        "degenerate_faces": int(np.count_nonzero(degenerate)),
                        "repeated_index_faces": int(np.count_nonzero(repeated)),
                        "area_degenerate_faces": int(np.count_nonzero(area_bad)),
                    }
                )
        except Exception as exc:  # noqa: BLE001
            load_errors.append(f"{relative}: {type(exc).__name__}: {exc}")
    evaluable = bool(geometries) and not load_errors
    count = len(geometries)
    watertight = sum(row["watertight"] for row in geometries)
    manifold = sum(row["edge_manifold_proxy"] for row in geometries)
    open_edges = sum(row["open_edges"] for row in geometries)
    degenerate = sum(row["degenerate_faces"] for row in geometries)
    return {
        "state": "pass" if evaluable else "not_evaluable",
        "evaluable": evaluable,
        "mesh_payload_count": len(mesh_paths),
        "mesh_scope": "canonical-deduplicated native URDF visual mesh dependency closure",
        "readable_mesh_payload_count": readable_meshes,
        "readable_geometry_count": count,
        "load_errors": load_errors,
        "watertight": {
            "state": "not_evaluable" if not evaluable else "pass" if watertight == count else "fail",
            **ratio(watertight, count if evaluable else 0),
        },
        "manifold": {
            "state": "not_evaluable" if not evaluable else "pass" if manifold == count else "fail",
            **ratio(manifold, count if evaluable else 0),
            "definition": "edge-manifold proxy; vertex-manifold is not claimed",
        },
        "open_edges": {
            "state": "not_evaluable" if not evaluable else "pass" if open_edges == 0 else "fail",
            "count": open_edges if evaluable else None,
        },
        "degenerate_faces": {
            "state": "not_evaluable" if not evaluable else "pass" if degenerate == 0 else "fail",
            "count": degenerate if evaluable else None,
        },
        "self_intersection": {
            "state": "not_evaluable",
            "reason": "no exact adjacent-face-excluding triangle-intersection backend was run",
        },
        "geometries": geometries,
    }


def hierarchy_gates(root: ET.Element) -> dict[str, Any]:
    links = root.findall("link")
    names = [node.attrib.get("name", "").strip() for node in links]
    name_set = set(names)
    adjacency: dict[str, list[str]] = {name: [] for name in names}
    children: set[str] = set()
    references = bool(names) and len(name_set) == len(names) and "" not in name_set
    joints = root.findall("joint")
    for joint in joints:
        parent_node = joint.find("parent")
        child_node = joint.find("child")
        parent = parent_node.attrib.get("link", "") if parent_node is not None else ""
        child = child_node.attrib.get("link", "") if child_node is not None else ""
        if parent not in name_set or child not in name_set:
            references = False
            continue
        adjacency[parent].append(child)
        children.add(child)
    roots = [name for name in names if name not in children]
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
    connected = len(roots) == 1 and visited == name_set
    valid = references and connected and acyclic and len(joints) == max(0, len(names) - 1)
    return {
        "link_count": len(names),
        "joint_count": len(joints),
        "unique_nonempty_link_names": len(name_set) == len(names) and "" not in name_set,
        "parent_child_references_resolve": references,
        "root_count": len(roots),
        "connected": connected,
        "acyclic": acyclic,
        "edge_count_is_links_minus_one": len(joints) == max(0, len(names) - 1),
        "valid_tree": valid,
    }


def semantic_audit(root: ET.Element | None, parse_error: str | None) -> tuple[dict[str, Any], dict[str, Any]]:
    strict = {
        "state": "not_evaluable",
        "reason": "no output-independent required-part and role specification exists for this cohort",
    }
    if root is None:
        return strict, {
            "state": "not_evaluable",
            "reason": parse_error or "native URDF unavailable",
            "label": "field proxy, not semantic correctness",
        }
    hierarchy = hierarchy_gates(root)
    names = [
        link.attrib.get("name", "").strip()
        for link in root.findall("link")
        if link.findall(".//visual/geometry")
    ]
    invalid = [name for name in names if not name or PLACEHOLDER_NAME.fullmatch(name)]
    gates = {
        "all_visual_geometry_bearing_parts_named_nonplaceholder": bool(names) and not invalid,
        "one_rooted_connected_acyclic_hierarchy": hierarchy["valid_tree"],
    }
    return strict, {
        "state": "pass" if all(gates.values()) else "fail",
        "label": "field proxy, not semantic correctness",
        "gates": gates,
        "visual_geometry_bearing_part_count": len(names),
        "invalid_or_placeholder_names": invalid,
        "hierarchy": hierarchy,
    }


def kinematic_audit(root: ET.Element | None, parse_error: str | None) -> dict[str, Any]:
    if root is None:
        return {"state": "not_evaluable", "reason": parse_error or "native URDF unavailable"}
    joints = root.findall("joint")
    if not joints:
        return {"state": "not_evaluable", "reason": "native package exposes no joints"}
    movable_joints = [joint for joint in joints if joint.attrib.get("type", "") != "fixed"]
    if not movable_joints:
        return {
            "state": "not_evaluable",
            "reason": "native package exposes joints but no movable articulation",
            "joint_count": len(joints),
            "movable_joint_count": 0,
        }
    links = {node.attrib.get("name", "") for node in root.findall("link")}
    gate_failures: dict[str, list[str]] = {
        "parent_child_references": [],
        "recognized_joint_types": [],
        "movable_one_axis_joint_axes": [],
        "revolute_prismatic_limits": [],
        "explicit_finite_joint_origins": [],
    }
    movable = 0
    for index, joint in enumerate(joints):
        name = joint.attrib.get("name", "") or f"joint[{index}]"
        joint_type = joint.attrib.get("type", "")
        movable += int(joint_type != "fixed")
        parent_node = joint.find("parent")
        child_node = joint.find("child")
        parent = parent_node.attrib.get("link", "") if parent_node is not None else ""
        child = child_node.attrib.get("link", "") if child_node is not None else ""
        if parent not in links or child not in links or parent == child:
            gate_failures["parent_child_references"].append(name)
        if joint_type not in RECOGNIZED_JOINT_TYPES:
            gate_failures["recognized_joint_types"].append(name)
        if joint_type in ONE_AXIS_JOINT_TYPES:
            axis = joint.find("axis")
            raw = axis.attrib.get("xyz") if axis is not None else None
            try:
                values = [float(value) for value in raw.split()] if raw else []
            except ValueError:
                values = []
            if not (
                len(values) == 3
                and all(math.isfinite(value) for value in values)
                and math.sqrt(sum(value * value for value in values)) > 1e-9
            ):
                gate_failures["movable_one_axis_joint_axes"].append(name)
        if joint_type in {"revolute", "prismatic"}:
            limit = joint.find("limit")
            lower = finite_number(limit.attrib.get("lower")) if limit is not None else None
            upper = finite_number(limit.attrib.get("upper")) if limit is not None else None
            if lower is None or upper is None or lower > upper:
                gate_failures["revolute_prismatic_limits"].append(name)
        origin = joint.find("origin")
        if not (
            origin is not None
            and finite_vector(origin.attrib.get("xyz"), default_zero=True)
            and finite_vector(origin.attrib.get("rpy"), default_zero=True)
        ):
            gate_failures["explicit_finite_joint_origins"].append(name)
    gates = {name: not failures for name, failures in gate_failures.items()}
    return {
        "state": "pass" if all(gates.values()) else "fail",
        "joint_count": len(joints),
        "movable_joint_count": movable,
        "gates": gates,
        "failing_joints_by_gate": gate_failures,
    }


def inertia_positive_definite(node: ET.Element | None) -> bool:
    if node is None:
        return False
    keys = ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
    values = {key: finite_number(node.attrib.get(key)) for key in keys}
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


def native_contact_present(root: ET.Element) -> bool:
    fields = {"mu", "mu1", "mu2", "friction", "lateral_friction", "rolling_friction"}
    for container in root.iter():
        if local_tag(container) not in {"gazebo", "contact"}:
            continue
        for node in container.iter():
            if local_tag(node) in fields:
                if node.text is None or finite_number(node.text.strip()) is not None:
                    return True
            if any(finite_number(node.attrib.get(field)) is not None for field in fields):
                return True
    return False


def physical_audit(
    root: ET.Element | None, parse_error: str | None, native_package_present: bool,
) -> dict[str, Any]:
    if not native_package_present:
        return {"state": "not_evaluable", "reason": "native simulation package unavailable"}
    if root is None:
        gates = {
            "visual_links_have_native_collision": False,
            "physical_links_have_positive_native_mass": False,
            "physical_links_have_positive_definite_native_inertia": False,
            "movable_joints_have_native_damping_and_friction": False,
            "native_contact_material_or_friction_metadata": False,
        }
        return {
            "state": "fail",
            "reason": parse_error or "native simulation package is not parseable",
            "runtime_defaults_count_as_native": False,
            "link_count": None,
            "movable_joint_count": None,
            "gates": gates,
            "failing_links_by_gate": {},
            "movable_joints_missing_damping_or_friction": [],
        }
    links = root.findall("link")
    if not links:
        return {
            "state": "fail",
            "reason": "native URDF has no physical links",
            "runtime_defaults_count_as_native": False,
            "link_count": 0,
            "movable_joint_count": 0,
            "gates": {
                "visual_links_have_native_collision": False,
                "physical_links_have_positive_native_mass": False,
                "physical_links_have_positive_definite_native_inertia": False,
                "movable_joints_have_native_damping_and_friction": False,
                "native_contact_material_or_friction_metadata": False,
            },
            "failing_links_by_gate": {},
            "movable_joints_missing_damping_or_friction": [],
        }
    link_failures = {
        "visual_links_have_native_collision": [],
        "physical_links_have_positive_native_mass": [],
        "physical_links_have_positive_definite_native_inertia": [],
    }
    for index, link in enumerate(links):
        name = link.attrib.get("name", "") or f"link[{index}]"
        visual = bool(link.findall("visual"))
        collision = any(node.find("geometry") is not None for node in link.findall("collision"))
        if visual and not collision:
            link_failures["visual_links_have_native_collision"].append(name)
        inertial = link.find("inertial")
        mass_node = inertial.find("mass") if inertial is not None else None
        mass = finite_number(mass_node.attrib.get("value")) if mass_node is not None else None
        if mass is None or mass <= 0.0:
            link_failures["physical_links_have_positive_native_mass"].append(name)
        inertia = inertial.find("inertia") if inertial is not None else None
        if not inertia_positive_definite(inertia):
            link_failures["physical_links_have_positive_definite_native_inertia"].append(name)
    dynamics_failures: list[str] = []
    movable_count = 0
    for index, joint in enumerate(root.findall("joint")):
        if joint.attrib.get("type", "") == "fixed":
            continue
        movable_count += 1
        name = joint.attrib.get("name", "") or f"joint[{index}]"
        dynamics = joint.find("dynamics")
        damping = finite_number(dynamics.attrib.get("damping")) if dynamics is not None else None
        friction = finite_number(dynamics.attrib.get("friction")) if dynamics is not None else None
        if damping is None or damping < 0.0 or friction is None or friction < 0.0:
            dynamics_failures.append(name)
    gates = {
        **{name: not failures for name, failures in link_failures.items()},
        "movable_joints_have_native_damping_and_friction": not dynamics_failures,
        "native_contact_material_or_friction_metadata": native_contact_present(root),
    }
    return {
        "state": "pass" if all(gates.values()) else "fail",
        "runtime_defaults_count_as_native": False,
        "link_count": len(links),
        "movable_joint_count": movable_count,
        "gates": gates,
        "failing_links_by_gate": link_failures,
        "movable_joints_missing_damping_or_friction": dynamics_failures,
    }


def audit_asset(row: dict[str, Any], scratch_root_text: str) -> dict[str, Any]:
    root_path = safe(WORKSPACE / row["asset_root"], must_exist=False)
    package = safe(WORKSPACE / row["primary_package"], must_exist=False)
    candidates = sorted(package.glob("*.urdf")) if package.is_dir() else []
    primary = safe(candidates[0]) if len(candidates) == 1 else None
    parsed: ET.Element | None = None
    parse_error: str | None = None
    if primary is not None:
        try:
            parsed = ET.parse(primary).getroot()
        except Exception as exc:  # noqa: BLE001
            parse_error = f"{type(exc).__name__}: {exc}"
    else:
        parse_error = f"expected exactly one top-level URDF, found {len(candidates)}"

    files = package_files(package)
    packaged_mesh_paths = sorted({path for path in files if path.suffix.lower() in MESH_SUFFIXES})
    closure: dict[str, Any] | None = None
    if primary is not None and parsed is not None:
        try:
            closure = dependency_closure(primary, package)
        except Exception as exc:  # noqa: BLE001
            parse_error = parse_error or f"dependency closure: {type(exc).__name__}: {exc}"
    visual_mesh_paths = closure["visual_mesh_paths"] if closure else []
    collision_mesh_paths = closure["collision_mesh_paths"] if closure else []
    referenced_mesh_paths = closure["mesh_paths"] if closure else []
    visual_issues = closure["visual_mesh_issues"] if closure else []
    geometry = geometry_audit(visual_mesh_paths, package, visual_issues)
    strict_semantic, field_proxy = semantic_audit(parsed, parse_error)
    kinematic = kinematic_audit(parsed, parse_error)
    physical = physical_audit(parsed, parse_error, package.is_dir())
    if parsed is not None and (closure is None or closure["collision_mesh_issues"]):
        physical.setdefault("gates", {})["visual_links_have_native_collision"] = False
        physical["state"] = "fail"
        physical["collision_dependency_issues"] = (
            closure["collision_mesh_issues"]
            if closure is not None else ["dependency closure unavailable"]
        )
    portable = portable_package_audit(package, primary, safe(Path(scratch_root_text)))
    metadata = {
        "segmented_glb_count": len(list(root_path.glob("*.segmented.glb"))) if root_path.is_dir() else 0,
        "articulation_json_count": len(list(root_path.glob("*.articulations.json"))) if root_path.is_dir() else 0,
        "material_json_present": (root_path / "material.json").is_file() if root_path.is_dir() else False,
    }
    inventory = [
        {"path": path.relative_to(package).as_posix(), "bytes": path.stat().st_size}
        for path in files
    ]
    inventory.sort(key=lambda item: item["path"])
    return {
        **row,
        "availability": {
            "state": "pass" if root_path.is_dir() else "fail",
            "definition": "official manifest root is locally released; independent of URDF evaluability",
            "asset_root_present": root_path.is_dir(),
        },
        "package_evaluability": {
            "state": "pass" if primary is not None and parsed is not None else "fail",
            "primary_package_present": package.is_dir(),
            "primary_urdf_candidate_count": len(candidates),
            "primary_urdf": primary.relative_to(root_path).as_posix() if primary else None,
            "primary_urdf_parsed": parsed is not None,
            "parse_error": parse_error,
        },
        "release_metadata_files": metadata,
        "integrity": {
            "primary_urdf_sha256": sha256_file(primary) if primary else None,
            "package_file_count": len(files),
            "package_path_size_manifest_sha256": canonical_sha256(inventory),
            "packaged_mesh_payload_count": len(packaged_mesh_paths),
            "visual_mesh_dependency_count": len(visual_mesh_paths),
            "collision_mesh_dependency_count": len(collision_mesh_paths),
            "referenced_mesh_dependency_count": len(referenced_mesh_paths),
            "referenced_mesh_dependency_bytes": sum(
                path.stat().st_size for path in referenced_mesh_paths
            ),
            "visual_mesh_dependency_bytes": sum(
                path.stat().st_size for path in visual_mesh_paths
            ),
            "visual_collision_mesh_overlap_count": len(
                set(visual_mesh_paths) & set(collision_mesh_paths)
            ),
            "declared_dependency_issues": closure["issues"] if closure else [],
        },
        "size_bytes": {
            "source": None,
            "source_reason": "Artiverse code is shared, not emitted per asset",
            "urdf": primary.stat().st_size if primary else None,
            "mesh": (
                sum(path.stat().st_size for path in referenced_mesh_paths)
                if closure is not None else None
            ),
            "mesh_payload_count": len(referenced_mesh_paths),
            "mesh_scope": (
                "canonical-deduplicated union of all URDF-declared visual and "
                "collision mesh dependency paths"
            ),
            "visual_referenced_mesh": (
                sum(path.stat().st_size for path in visual_mesh_paths)
                if closure is not None else None
            ),
            "visual_referenced_mesh_count": len(visual_mesh_paths),
            "visual_referenced_mesh_scope": (
                "canonical-deduplicated URDF visual mesh dependency closure"
            ),
            "packaged_mesh_all_representations": (
                sum(path.stat().st_size for path in packaged_mesh_paths)
                if packaged_mesh_paths else None
            ),
            "packaged_mesh_all_representations_count": len(packaged_mesh_paths),
            "primary_package": sum(path.stat().st_size for path in files) if files else None,
        },
        "geometry": geometry,
        "portable_package": portable,
        "deterministic_build": {
            "state": "not_evaluable",
            "fresh_build_count": 0,
            "reason": "static release audit; no two fresh frozen builds were executed",
            "existing_artifact_rehash_not_used_as_build_evidence": True,
        },
        "semantic_complete": strict_semantic,
        "semantic_field_proxy": field_proxy,
        "kinematic_complete": kinematic,
        "physical_complete": physical,
    }


def nested_value(row: dict[str, Any], key: str) -> Any:
    value: Any = row
    for part in key.split("."):
        value = value[part]
    return value


def state_partition(rows: list[dict[str, Any]], key: str) -> dict[str, Any]:
    counts = Counter(nested_value(row, key) for row in rows)
    result = {state: int(counts.get(state, 0)) for state in ("pass", "fail", "not_evaluable")}
    result["denominator"] = len(rows)
    result["evaluable_denominator"] = result["pass"] + result["fail"]
    result["pass_rate"] = (
        result["pass"] / result["evaluable_denominator"]
        if result["evaluable_denominator"] else None
    )
    return result


def gate_partition(rows: list[dict[str, Any]], key: str) -> dict[str, Any]:
    result = state_partition(rows, f"{key}.state")
    names = sorted(
        {
            name
            for row in rows
            for name, value in row.get(key, {}).get("gates", {}).items()
            if isinstance(value, bool)
        }
    )
    result["gate_pass_counts"] = {
        name: ratio(
            sum(row.get(key, {}).get("gates", {}).get(name) is True for row in rows),
            sum(isinstance(row.get(key, {}).get("gates", {}).get(name), bool) for row in rows),
        )
        for name in names
    }
    return result


def mean_or_none(values: Iterable[float]) -> float | None:
    materialized = list(values)
    return sum(materialized) / len(materialized) if materialized else None


def geometry_summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
    evaluable = [row for row in rows if row["geometry"]["evaluable"]]
    geometries = [item for row in evaluable for item in row["geometry"]["geometries"]]
    watertight = sum(item["watertight"] for item in geometries)
    manifold = sum(item["edge_manifold_proxy"] for item in geometries)
    open_edges = sum(item["open_edges"] for item in geometries)
    degenerate = sum(item["degenerate_faces"] for item in geometries)
    return {
        "requested_assets": len(rows),
        "geometry_evaluable_assets": len(evaluable),
        "geometry_not_evaluable_assets": len(rows) - len(evaluable),
        "mesh_load_error_assets": sum(bool(row["geometry"]["load_errors"]) for row in rows),
        "mesh_load_error_count": sum(len(row["geometry"]["load_errors"]) for row in rows),
        "no_mesh_payload_assets": sum(not row["geometry"]["mesh_payload_count"] for row in rows),
        "readable_mesh_payloads": sum(row["geometry"]["readable_mesh_payload_count"] for row in evaluable),
        "readable_geometries": len(geometries),
        "mesh_scope": "canonical-deduplicated native URDF visual mesh dependency closure",
        "watertight": {
            "geometry_level": ratio(watertight, len(geometries)),
            "per_asset_mean_fraction": mean_or_none(
                row["geometry"]["watertight"]["rate"] for row in evaluable
            ),
            "all_geometries_pass_assets": ratio(
                sum(row["geometry"]["watertight"]["state"] == "pass" for row in evaluable),
                len(evaluable),
            ),
        },
        "manifold": {
            "definition": "edge-manifold proxy; vertex-manifold is not claimed",
            "geometry_level": ratio(manifold, len(geometries)),
            "per_asset_mean_fraction": mean_or_none(
                row["geometry"]["manifold"]["rate"] for row in evaluable
            ),
            "all_geometries_pass_assets": ratio(
                sum(row["geometry"]["manifold"]["state"] == "pass" for row in evaluable),
                len(evaluable),
            ),
        },
        "open_edges": {
            "total": open_edges,
            "per_asset_mean": open_edges / len(evaluable) if evaluable else None,
            "zero_error_assets": ratio(
                sum(row["geometry"]["open_edges"]["state"] == "pass" for row in evaluable),
                len(evaluable),
            ),
        },
        "degenerate_faces": {
            "total": degenerate,
            "per_asset_mean": degenerate / len(evaluable) if evaluable else None,
            "zero_error_assets": ratio(
                sum(row["geometry"]["degenerate_faces"]["state"] == "pass" for row in evaluable),
                len(evaluable),
            ),
        },
        "self_intersection": {
            "state": "not_evaluable",
            "denominator": len(rows),
            "reason": "no exact adjacent-face-excluding triangle-intersection backend was run",
        },
    }


def byte_summary(rows: list[dict[str, Any]], key: str) -> dict[str, Any]:
    values = [row["size_bytes"][key] for row in rows if row["size_bytes"][key] is not None]
    return {
        "denominator": len(values),
        "total_bytes": sum(values),
        "mean_bytes": sum(values) / len(values) if values else None,
    }


def build_summary(
    records: list[dict[str, Any]], manifest_hash: str, protocol_hash: str,
    source: dict[str, Any], preflight: dict[str, Any],
) -> dict[str, Any]:
    available = sum(row["availability"]["state"] == "pass" for row in records)
    return {
        "schema_version": "2.0.0",
        "protocol_id": EXPECTED_PROTOCOL_ID,
        "status": "COMPLETE",
        "audit_identity": AUDIT_IDENTITY,
        "interpretation": AUDIT_IDENTITY,
        "cohort": {
            "requested_assets": len(records),
            "available_assets": available,
            "unavailable_assets": len(records) - available,
            "geometry_evaluable_assets": sum(row["geometry"]["evaluable"] for row in records),
            "package_evaluable_assets": sum(row["package_evaluability"]["state"] == "pass" for row in records),
            "selection": "all 3,544 identities in the official chunk manifest; no outcome filtering",
        },
        "provenance": {
            "dataset_revision": EXPECTED_DATASET_REVISION,
            "release_identity": RELEASE_IDENTITY,
            "official_code_commit": EXPECTED_CODE_COMMIT,
            "chunk_manifest": preflight["manifest"],
            "archive_hashes_recomputed": preflight["archive_hashes_recomputed"],
            "archive_hashes_match": preflight["archive_hashes_match"],
            "complete_extraction": preflight["status"] == "COMPLETE",
        },
        "results": {
            "geometry": geometry_summary(records),
            "size_bytes": {
                "source": {
                    "denominator": 1,
                    "total_bytes": source["total_bytes"],
                    "mean_bytes": None,
                    "scope": "shared official executable source; no per-asset source is emitted",
                },
                "urdf": byte_summary(records, "urdf"),
                "mesh": byte_summary(records, "mesh"),
                "visual_referenced_mesh": byte_summary(records, "visual_referenced_mesh"),
                "packaged_mesh_all_representations": byte_summary(
                    records, "packaged_mesh_all_representations"
                ),
                "primary_package": byte_summary(records, "primary_package"),
            },
            "portable_package": state_partition(records, "portable_package.state"),
            "deterministic_build": state_partition(records, "deterministic_build.state"),
            "semantic_complete": state_partition(records, "semantic_complete.state"),
            "semantic_field_proxy": gate_partition(records, "semantic_field_proxy"),
            "kinematic_complete": gate_partition(records, "kinematic_complete"),
            "physical_complete": gate_partition(records, "physical_complete"),
        },
        "fail_closed": {
            "self_intersection": "not_evaluable; no exact backend",
            "deterministic_build": "not_evaluable; no two fresh builds",
            "semantic_complete": "not_evaluable; no output-independent required part/role gold",
            "runtime_physics_defaults_count_as_native": False,
        },
        "hashes": {
            "protocol_sha256": protocol_hash,
            "manifest_sha256": manifest_hash,
            "runner_sha256": sha256_file(SCRIPT),
            "official_source_manifest_sha256": source["content_manifest_sha256"],
        },
    }


def fmt(value: float | None, digits: int = 3) -> str:
    return "N/A" if value is None else f"{value:.{digits}f}"


def report_text(summary: dict[str, Any]) -> str:
    cohort = summary["cohort"]
    result = summary["results"]
    geometry = result["geometry"]
    sizes = result["size_bytes"]
    watertight = geometry["watertight"]
    manifold = geometry["manifold"]
    portable = result["portable_package"]
    semantic = result["semantic_complete"]
    proxy = result["semantic_field_proxy"]
    kinematic = result["kinematic_complete"]
    physical = result["physical_complete"]
    source_kib = sizes["source"]["total_bytes"] / 1024
    urdf_mean = sizes["urdf"]["mean_bytes"] / 1024 if sizes["urdf"]["mean_bytes"] is not None else None
    urdf_total = sizes["urdf"]["total_bytes"] / 1024
    mesh_mean = sizes["mesh"]["mean_bytes"] / 1024 if sizes["mesh"]["mean_bytes"] is not None else None
    mesh_total = sizes["mesh"]["total_bytes"] / 1024
    return f"""# Table 7: Artiverse production readiness

Status: **COMPLETE**

This static audit retains all {cohort['requested_assets']} identities in the official
two-chunk manifest (84 categories, 10 source repositories). No asset was selected,
repaired, rebuilt, or dropped based on a Table 7 outcome.

## Cohort and provenance

- Requested/available: {cohort['requested_assets']}/{cohort['available_assets']}; unavailable={cohort['unavailable_assets']}.
- Geometry-evaluable: {cohort['geometry_evaluable_assets']}; package-evaluable: {cohort['package_evaluable_assets']}.
- Dataset revision: `{summary['provenance']['dataset_revision']}`; official code commit: `{summary['provenance']['official_code_commit']}`.
- Extraction matched 3,544 roots, 531,937 files, and 86,992,752,890 input bytes.
- Geometry scope is the canonical-deduplicated URDF visual-mesh dependency closure.
- Headline Mesh KB is the canonical-deduplicated union of all URDF-declared visual and collision mesh paths. Unreferenced packaged GLB, convenience, and decomposition representations are excluded; visual-only and all-packaged byte totals are retained as auxiliary evidence.

## Table 7 row

| Method | Watertight | Manifold | Open Edges | Degenerate Faces | Self-Intersection | Source KB | URDF KB | Mesh KB | Portable Package | Deterministic Build | Semantic Complete | Kinematic Complete | Physical Complete |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Artiverse frozen pre-release subset (N={cohort['requested_assets']}) | {fmt(watertight['per_asset_mean_fraction'], 6)} mean/asset; {watertight['geometry_level']['numerator']}/{watertight['geometry_level']['denominator']} geom | {fmt(manifold['per_asset_mean_fraction'], 6)} edge-manifold mean/asset; {manifold['geometry_level']['numerator']}/{manifold['geometry_level']['denominator']} geom | {fmt(geometry['open_edges']['per_asset_mean'])}/asset; {geometry['open_edges']['total']} total | {fmt(geometry['degenerate_faces']['per_asset_mean'])}/asset; {geometry['degenerate_faces']['total']} total | N/E | shared {source_kib:.2f} total; per-asset N/E | {fmt(urdf_mean, 2)}/asset; {urdf_total:.2f} total | {fmt(mesh_mean, 2)}/asset; {mesh_total:.2f} total | {portable['pass']}/{portable['denominator']} | N/E (no two fresh builds) | N/E; field proxy {proxy['pass']}/{proxy['evaluable_denominator']} evaluable; {proxy['not_evaluable']} N/E | {kinematic['pass']}/{kinematic['evaluable_denominator']} applicable | {physical['pass']}/{physical['evaluable_denominator']} applicable |

## Evidence-state notes

- Readable geometries: {geometry['readable_geometries']}; mesh-load-error assets: {geometry['mesh_load_error_assets']}.
- Edge manifold is the <=2 incident-face proxy; vertex manifold is not claimed.
- Self-intersection is N/E because no exact adjacent-face-excluding backend ran.
- Source size is shared method code and is not divided into artificial per-asset source bytes.
- Portable Package copies the complete native simulation subpackage (`urdf_w_collider`) into fresh workspace-internal storage, then parses the copied URDF and recursively parses and resolves declared dependencies. Mesh readability is scored independently by the geometry audit. It does not claim to copy the entire Artiverse model root.
- Deterministic Build is N/E: existing hashes are provenance, not two fresh builds.
- Strict Semantic Complete is N/E for {semantic['not_evaluable']}/{semantic['denominator']}; the separately labelled name/tree field proxy is not semantic correctness.
- Physical completeness is fail-closed: simulator defaults do not satisfy native collision, mass, inertia, joint dynamics, or contact metadata gates.

## Reproduction

```bash
python arti-skill/exp/scripts/run_table7_artiverse.py --preflight-only
python arti-skill/exp/scripts/run_table7_artiverse.py --workers 2 --preflight-snapshot arti-skill/exp/artiverse/full_preflight.tmp.json
python arti-skill/exp/scripts/run_table7_artiverse.py --verify-only
```
"""


def make_manifest(
    protocol_hash: str, preflight: dict[str, Any], source: dict[str, Any],
    cohort: list[dict[str, Any]], output: Path,
) -> dict[str, Any]:
    return {
        "schema_version": "2.0.0",
        "protocol_id": EXPECTED_PROTOCOL_ID,
        "protocol_snapshot": rel(output / "protocol_snapshot.json"),
        "protocol_sha256": protocol_hash,
        "method": "Artiverse",
        "role": "real-data reference (frozen pre-release subset)",
        "audit_identity": AUDIT_IDENTITY,
        "axis": "Table 7 Production Readiness",
        "status": "FROZEN_BEFORE_SCORING",
        "manifest_frozen_before_scoring": True,
        "selection_policy": {
            "rule": "all roots from both official chunk-manifest entries in manifest order",
            "requested_assets": len(cohort),
            "outcome_based_filtering": False,
            "missing_or_failed_assets_retained": True,
        },
        "representation_adapter": {
            "asset_unit": "category/source_repository/model_id root",
            "native_simulation_package": "urdf_w_collider directory",
            "primary_artifact": "the exactly-one top-level *.urdf in that directory",
            "geometry_scope": "canonical-deduplicated URDF visual mesh dependency closure",
            "mesh_size_scope": "canonical-deduplicated union of all URDF-declared visual and collision mesh dependency paths",
            "auxiliary_mesh_inventory": "visual-only refs and all recursively packaged mesh representations",
            "portable_scope": "complete native simulation subpackage (urdf_w_collider), not entire model root",
            "source_bytes": "shared official executable source, reported separately",
            "self_intersection": "not_evaluable",
            "deterministic_build": "not_evaluable",
            "semantic_complete": "not_evaluable; field proxy reported separately",
        },
        "frozen_inputs": {
            "dataset_revision": EXPECTED_DATASET_REVISION,
            "release_identity": RELEASE_IDENTITY,
            "full_preflight_snapshot": preflight.get("snapshot_evidence"),
            "chunk_manifest": preflight["manifest"],
            "archives": preflight["archives"],
            "tree_preflight": {
                key: preflight[key]
                for key in (
                    "status", "requested_roots", "present_roots", "missing_root_count",
                    "observed_root_count", "unexpected_root_count",
                    "expected_file_count", "observed_file_count", "expected_input_bytes",
                    "observed_input_bytes", "roots_with_one_primary_urdf",
                    "roots_with_one_segmented_glb", "roots_with_one_articulation_json",
                    "roots_with_material_json", "archive_hashes_recomputed", "archive_hashes_match",
                    "quick_root_only", "per_root_inventory_count",
                    "per_root_expected_values_available", "per_root_matches_manifest",
                    "per_root_mismatch_count",
                )
            },
            "official_source": source,
            "runner": {"path": rel(SCRIPT), "sha256": sha256_file(SCRIPT)},
        },
        "assets": cohort,
    }


def current_checks(
    output: Path, protocol: dict[str, Any], manifest: dict[str, Any],
    records: list[dict[str, Any]], summary: dict[str, Any],
) -> dict[str, bool]:
    ids = [row["asset_id"] for row in manifest["assets"]]
    record_ids = [row["asset_id"] for row in records]
    cohort = summary["cohort"]
    result = summary["results"]
    return {
        "required_outputs_match_protocol": tuple(protocol.get("required_outputs", [])) == REQUIRED,
        "all_required_outputs_exist": all((output / name).is_file() for name in REQUIRED),
        "protocol_snapshot_exact": sha256_file(output / "protocol_snapshot.json") == sha256_file(PROTOCOL),
        "protocol_hash_recorded": manifest["protocol_sha256"] == summary["hashes"]["protocol_sha256"],
        "manifest_hash_recorded": sha256_file(output / "manifest.json") == summary["hashes"]["manifest_sha256"],
        "full_preflight_snapshot_evidence_valid": preflight_snapshot_matches_manifest(manifest),
        "asset_records_input_provenance_valid": (
            "asset_records_input" not in manifest["frozen_inputs"]
            or (
                manifest["frozen_inputs"]["asset_records_input"].get("path")
                == rel(output / "asset_records.json")
                and manifest["frozen_inputs"]["asset_records_input"].get("sha256")
                == sha256_file(output / "asset_records.json")
                and manifest["frozen_inputs"]["asset_records_input"].get("record_count")
                == len(records)
                and isinstance(
                    manifest["frozen_inputs"].get("asset_evaluator_runner"), dict
                )
            )
        ),
        "pre_release_identity_consistent": (
            manifest.get("audit_identity") == AUDIT_IDENTITY
            and summary.get("audit_identity") == AUDIT_IDENTITY
            and summary.get("interpretation") == AUDIT_IDENTITY
            and manifest["frozen_inputs"].get("release_identity") == RELEASE_IDENTITY
            and summary["provenance"].get("release_identity") == RELEASE_IDENTITY
            and "frozen pre-release subset" in (output / "report.md").read_text(encoding="utf-8")
        ),
        "no_full_or_final_release_identity_claim": not re.search(
            r"\b(?:full|final)[ _-]?release\b",
            json.dumps({"manifest": manifest, "summary": summary}, sort_keys=True)
            + "\n"
            + (output / "report.md").read_text(encoding="utf-8"),
            flags=re.IGNORECASE,
        ),
        "manifest_identities_unique": len(ids) == len(set(ids)) == EXPECTED_REQUESTED,
        "asset_records_match_manifest_order": record_ids == ids,
        "requested_denominator_fixed": cohort["requested_assets"] == EXPECTED_REQUESTED,
        "availability_conserved": cohort["requested_assets"] == cohort["available_assets"] + cohort["unavailable_assets"],
        "geometry_denominator_matches": cohort["geometry_evaluable_assets"] == result["geometry"]["geometry_evaluable_assets"],
        "package_denominator_matches": cohort["package_evaluable_assets"] == sum(
            row["package_evaluability"]["state"] == "pass" for row in records
        ),
        "released_availability_independent_of_urdf": all(
            row["availability"]["state"] == "pass"
            and row["availability"]["asset_root_present"] is True
            for row in records
        ),
        "headline_geometry_is_visual_closure": all(
            (
                row["geometry"]["mesh_scope"]
                == "canonical-deduplicated native URDF visual mesh dependency closure"
                and (
                    (
                        row["package_evaluability"]["state"] == "pass"
                        and row["geometry"]["mesh_payload_count"]
                        == row["integrity"]["visual_mesh_dependency_count"]
                        == row["size_bytes"]["visual_referenced_mesh_count"]
                        and row["size_bytes"]["visual_referenced_mesh"]
                        == row["integrity"]["visual_mesh_dependency_bytes"]
                    )
                    or (
                        row["package_evaluability"]["state"] == "fail"
                        and row["geometry"]["state"] == "not_evaluable"
                        and row["geometry"]["mesh_payload_count"] == 0
                        and row["integrity"]["visual_mesh_dependency_count"] == 0
                        and row["size_bytes"]["visual_referenced_mesh_count"] == 0
                        and row["size_bytes"]["visual_referenced_mesh"] is None
                    )
                )
            )
            for row in records
        ),
        "headline_mesh_size_is_referenced_union": all(
            row["size_bytes"]["mesh_scope"]
            == (
                "canonical-deduplicated union of all URDF-declared visual and "
                "collision mesh dependency paths"
            )
            and (
                (
                    row["package_evaluability"]["state"] == "pass"
                    and row["size_bytes"]["mesh_payload_count"]
                    == row["integrity"]["referenced_mesh_dependency_count"]
                    and row["size_bytes"]["mesh"]
                    == row["integrity"]["referenced_mesh_dependency_bytes"]
                )
                or (
                    row["package_evaluability"]["state"] == "fail"
                    and row["size_bytes"]["mesh_payload_count"] == 0
                    and row["integrity"]["referenced_mesh_dependency_count"] == 0
                    and row["size_bytes"]["mesh"] is None
                )
            )
            for row in records
        ),
        "manifest_mesh_scopes_recorded": (
            manifest["representation_adapter"]["geometry_scope"]
            == "canonical-deduplicated URDF visual mesh dependency closure"
            and manifest["representation_adapter"]["mesh_size_scope"]
            == (
                "canonical-deduplicated union of all URDF-declared visual and "
                "collision mesh dependency paths"
            )
        ),
        "all_fixed_kinematics_not_evaluable": all(
            not (
                row["kinematic_complete"].get("movable_joint_count") == 0
                and row["kinematic_complete"]["state"] != "not_evaluable"
            )
            for row in records
        ),
        "portable_scope_is_native_subpackage": (
            manifest["representation_adapter"]["portable_scope"]
            == "complete native simulation subpackage (urdf_w_collider), not entire model root"
        ),
        "all_state_partitions_conserved": all(
            axis["pass"] + axis["fail"] + axis["not_evaluable"] == axis["denominator"] == EXPECTED_REQUESTED
            for axis in (
                result["portable_package"], result["deterministic_build"],
                result["semantic_complete"], result["semantic_field_proxy"],
                result["kinematic_complete"], result["physical_complete"],
            )
        ),
        "self_intersection_not_evaluable": result["geometry"]["self_intersection"]["state"] == "not_evaluable",
        "determinism_not_rehash_shortcut": result["deterministic_build"]["not_evaluable"] == EXPECTED_REQUESTED,
        "strict_semantics_not_proxy": result["semantic_complete"]["not_evaluable"] == EXPECTED_REQUESTED,
        "source_commit_pinned": manifest["frozen_inputs"]["official_source"]["code_commit"] == EXPECTED_CODE_COMMIT,
        "complete_tree_recorded": manifest["frozen_inputs"]["tree_preflight"]["status"] == "COMPLETE",
        "full_tree_not_quick_only": manifest["frozen_inputs"]["tree_preflight"]["quick_root_only"] is False,
        "all_roots_have_observed_inventory": (
            manifest["frozen_inputs"]["tree_preflight"]["per_root_inventory_count"]
            == EXPECTED_REQUESTED
        ),
        "manifest_frozen_before_scoring": manifest["manifest_frozen_before_scoring"] is True,
        "outcome_filtering_disabled": manifest["selection_policy"]["outcome_based_filtering"] is False,
    }


def run_assets(
    cohort: list[dict[str, Any]], output: Path, workers: int, resume: bool,
    run_key: str,
) -> list[dict[str, Any]]:
    partial = output / "asset_records.partial.jsonl"
    meta_path = output / "asset_records.partial.meta.json"
    existing: dict[str, dict[str, Any]] = {}
    if resume:
        if not partial.is_file() or not meta_path.is_file():
            raise RuntimeError("--resume requested but compatible partial files do not exist")
        meta = read_json(meta_path)
        if meta.get("run_key") != run_key:
            raise RuntimeError("partial audit run key does not match frozen inputs/runner")
        for line in partial.read_text(encoding="utf-8").splitlines():
            if line.strip():
                record = json.loads(line)
                existing[record["asset_id"]] = record
    else:
        write_json(meta_path, {"run_key": run_key, "requested_assets": len(cohort)})
        safe(partial, must_exist=False).write_text("", encoding="utf-8")

    remaining = [row for row in cohort if row["asset_id"] not in existing]
    scratch = safe(output / ".portable_tmp", must_exist=False)
    scratch.mkdir(parents=True, exist_ok=True)
    scratch = safe(scratch)
    mode = "a"
    with safe(partial, must_exist=False).open(mode, encoding="utf-8") as stream:
        if workers == 1:
            iterator = ((row, audit_asset(row, str(scratch))) for row in remaining)
            for row, record in iterator:
                existing[row["asset_id"]] = record
                stream.write(json.dumps(record, sort_keys=True, ensure_ascii=True) + "\n")
                stream.flush()
                if len(existing) % 25 == 0 or len(existing) == len(cohort):
                    print(f"[Artiverse Table 7] {len(existing)}/{len(cohort)}", flush=True)
        else:
            with ProcessPoolExecutor(max_workers=workers) as pool:
                futures = {
                    pool.submit(audit_asset, row, str(scratch)): row
                    for row in remaining
                }
                for future in as_completed(futures):
                    row = futures[future]
                    record = future.result()
                    existing[row["asset_id"]] = record
                    stream.write(json.dumps(record, sort_keys=True, ensure_ascii=True) + "\n")
                    stream.flush()
                    if len(existing) % 25 == 0 or len(existing) == len(cohort):
                        print(f"[Artiverse Table 7] {len(existing)}/{len(cohort)}", flush=True)
    leftovers = list(scratch.iterdir())
    if leftovers:
        raise RuntimeError(f"portable scratch is not empty: {len(leftovers)} entries")
    records = [existing[row["asset_id"]] for row in cohort]
    if len(records) != EXPECTED_REQUESTED:
        raise RuntimeError("asset-record result count is incomplete")
    return records


def create(
    output: Path, *, workers: int, verify_archive_hashes: bool, resume: bool,
    preflight_snapshot: Path | None,
) -> dict[str, Any]:
    output = safe(output, must_exist=False)
    safe(output.parent).mkdir(parents=True, exist_ok=True)
    output.mkdir(parents=True, exist_ok=True)
    output = safe(output)
    protocol = read_json(PROTOCOL)
    if protocol.get("protocol_id") != EXPECTED_PROTOCOL_ID:
        raise RuntimeError("shared Table 7 protocol id drifted")
    if tuple(protocol.get("required_outputs", [])) != REQUIRED:
        raise RuntimeError("shared Table 7 required outputs drifted")
    preflight = (
        validated_preflight_snapshot(preflight_snapshot)
        if preflight_snapshot is not None
        else tree_preflight(verify_archive_hashes=verify_archive_hashes)
    )
    if not preflight["scoring_allowed"]:
        raise RuntimeError(
            "Artiverse extraction is incomplete; refusing to freeze or score a partial cohort: "
            f"present={preflight['present_roots']}/{preflight['requested_roots']}, "
            f"files={preflight['observed_file_count']}/{preflight['expected_file_count']}, "
            f"bytes={preflight['observed_input_bytes']}/{preflight['expected_input_bytes']}"
        )
    _chunk_manifest, cohort = load_frozen_cohort()
    source = source_inventory()
    snapshot = output / "protocol_snapshot.json"
    temporary = output / "protocol_snapshot.json.tmp"
    shutil.copyfile(safe(PROTOCOL), safe(temporary, must_exist=False))
    temporary.replace(snapshot)
    protocol_hash = sha256_file(snapshot)
    manifest = make_manifest(protocol_hash, preflight, source, cohort, output)
    write_json(output / "manifest.json", manifest)
    manifest_hash = sha256_file(output / "manifest.json")
    run_key = canonical_sha256(
        {
            "protocol": protocol_hash,
            "chunk_manifest": preflight["manifest"]["sha256"],
            "runner": sha256_file(SCRIPT),
            "manifest": manifest_hash,
        }
    )
    records = run_assets(cohort, output, workers, resume, run_key)
    write_json(output / "asset_records.json", records)
    summary = build_summary(records, manifest_hash, protocol_hash, source, preflight)
    write_json(output / "summary.json", summary)
    write_text(output / "report.md", report_text(summary))
    write_json(output / "self_check.json", {"status": "PROVISIONAL"})
    checks = current_checks(output, protocol, manifest, records, summary)
    hashes = {
        name: sha256_file(output / name)
        for name in REQUIRED if name != "self_check.json"
    }
    self_check = {
        "schema_version": "2.0.0",
        "protocol_id": EXPECTED_PROTOCOL_ID,
        "status": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks,
        "protocol_sha256": protocol_hash,
        "manifest_sha256": manifest_hash,
        "artifact_hashes": hashes,
        "artifact_hash_scope": "required outputs except self_check.json to avoid a circular hash",
    }
    write_json(output / "self_check.json", self_check)
    if self_check["status"] != "PASS":
        raise RuntimeError(f"self-check failed: {[key for key, value in checks.items() if not value]}")
    return verify(output, scan_tree=False)


def finalize_existing(output: Path) -> dict[str, Any]:
    output = safe(output)
    old_manifest = read_json(output / "manifest.json")
    old_check = read_json(output / "self_check.json")
    records_path = safe(output / "asset_records.json")
    records = read_json(records_path)
    record_hash = sha256_file(records_path)
    known_finalize_checks = {
        "headline_geometry_is_visual_closure",
        "headline_mesh_size_is_referenced_union",
    }
    failed = {name for name, passed in old_check.get("checks", {}).items() if not passed}
    expected_failed_run = failed == known_finalize_checks and old_check.get("status") == "FAIL"
    expected_finalized_run = (
        not failed
        and old_check.get("status") == "PASS"
        and old_manifest.get("frozen_inputs", {}).get("asset_records_input", {}).get("sha256")
        == record_hash
    )
    if not (expected_failed_run or expected_finalized_run):
        raise RuntimeError(f"existing self-check is not the expected finalize case: {sorted(failed)}")
    if old_check.get("artifact_hashes", {}).get("asset_records.json") != record_hash:
        raise RuntimeError("existing asset-record hash differs from failed-run evidence")
    _chunk_manifest, cohort = load_frozen_cohort()
    if old_manifest.get("assets") != cohort or len(records) != EXPECTED_REQUESTED:
        raise RuntimeError("existing manifest/record cohort differs from frozen cohort")
    static_keys = (
        "selection_index", "asset_id", "asset_root", "category",
        "source_repository", "model_id", "primary_package",
    )
    for expected, record in zip(cohort, records, strict=True):
        if any(record.get(key) != expected.get(key) for key in static_keys):
            raise RuntimeError(f"existing record identity drifted: {expected['asset_id']}")
    evaluator_runner = old_manifest.get("frozen_inputs", {}).get(
        "asset_evaluator_runner",
        old_manifest.get("frozen_inputs", {}).get("runner"),
    )
    if not (
        isinstance(evaluator_runner, dict)
        and evaluator_runner.get("path") == rel(SCRIPT)
        and re.fullmatch(r"[0-9a-f]{64}", evaluator_runner.get("sha256", ""))
    ):
        raise RuntimeError("existing asset-evaluator runner provenance is invalid")
    evidence = old_manifest["frozen_inputs"]["full_preflight_snapshot"]
    preflight = validated_preflight_snapshot(safe(WORKSPACE / evidence["path"]))
    if preflight["snapshot_evidence"] != evidence:
        raise RuntimeError("existing preflight snapshot evidence drifted")
    protocol = read_json(PROTOCOL)
    source = source_inventory()
    protocol_hash = sha256_file(output / "protocol_snapshot.json")
    manifest = make_manifest(protocol_hash, preflight, source, cohort, output)
    manifest["frozen_inputs"]["asset_evaluator_runner"] = evaluator_runner
    manifest["frozen_inputs"]["asset_records_input"] = {
        "path": rel(records_path),
        "sha256": record_hash,
        "record_count": len(records),
        "finalized_without_asset_rerun": True,
    }
    write_json(output / "manifest.json", manifest)
    manifest_hash = sha256_file(output / "manifest.json")
    summary = build_summary(records, manifest_hash, protocol_hash, source, preflight)
    summary["provenance"]["asset_evaluator_runner"] = evaluator_runner
    summary["provenance"]["asset_records_sha256"] = record_hash
    summary["provenance"]["finalized_without_asset_rerun"] = True
    write_json(output / "summary.json", summary)
    write_text(output / "report.md", report_text(summary))
    write_json(output / "self_check.json", {"status": "PROVISIONAL"})
    checks = current_checks(output, protocol, manifest, records, summary)
    hashes = {
        name: sha256_file(output / name)
        for name in REQUIRED if name != "self_check.json"
    }
    self_check = {
        "schema_version": "2.0.0",
        "protocol_id": EXPECTED_PROTOCOL_ID,
        "status": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks,
        "protocol_sha256": protocol_hash,
        "manifest_sha256": manifest_hash,
        "artifact_hashes": hashes,
        "artifact_hash_scope": "required outputs except self_check.json to avoid a circular hash",
    }
    write_json(output / "self_check.json", self_check)
    if self_check["status"] != "PASS":
        raise RuntimeError(f"finalize self-check failed: {[key for key, value in checks.items() if not value]}")
    return verify(output, scan_tree=False)


def verify(output: Path, *, scan_tree: bool = True) -> dict[str, Any]:
    output = safe(output)
    protocol = read_json(output / "protocol_snapshot.json")
    manifest = read_json(output / "manifest.json")
    records = read_json(output / "asset_records.json")
    summary = read_json(output / "summary.json")
    recorded = read_json(output / "self_check.json")
    checks = current_checks(output, protocol, manifest, records, summary)
    errors = [name for name, passed in checks.items() if not passed]
    for name, expected in recorded.get("artifact_hashes", {}).items():
        if sha256_file(output / name) != expected:
            errors.append(f"artifact_hash:{name}")
    if recorded.get("status") != "PASS":
        errors.append("recorded_self_check_status")
    if sha256_file(CHUNK_MANIFEST) != manifest["frozen_inputs"]["chunk_manifest"]["sha256"]:
        errors.append("chunk_manifest_hash_drift")
    if sha256_file(SCRIPT) != manifest["frozen_inputs"]["runner"]["sha256"]:
        errors.append("runner_hash_drift")
    if scan_tree:
        live = tree_preflight(verify_archive_hashes=False)
        frozen = manifest["frozen_inputs"]["tree_preflight"]
        for key in (
            "status", "present_roots", "observed_root_count", "unexpected_root_count",
            "observed_file_count", "observed_input_bytes",
            "roots_with_one_primary_urdf", "roots_with_one_segmented_glb",
            "roots_with_one_articulation_json", "roots_with_material_json",
        ):
            if live[key] != frozen[key]:
                errors.append(f"live_tree_drift:{key}")
    return {
        "status": "PASS" if not errors else "FAIL",
        "errors": errors,
        "audit_status": summary.get("status"),
        "requested_assets": len(records),
        "available_assets": summary["cohort"]["available_assets"],
        "geometry_evaluable_assets": summary["cohort"]["geometry_evaluable_assets"],
        "output": rel(output),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--workers", type=int, default=2)
    parser.add_argument("--verify-archive-hashes", action="store_true")
    parser.add_argument(
        "--preflight-snapshot", type=Path,
        help="validated COMPLETE full-preflight JSON; avoids re-reading archive payloads",
    )
    parser.add_argument("--resume", action="store_true")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--preflight-only", action="store_true")
    mode.add_argument("--full-preflight-only", action="store_true")
    mode.add_argument("--verify-only", action="store_true")
    mode.add_argument("--finalize-existing", action="store_true")
    args = parser.parse_args()
    if not 1 <= args.workers <= 8:
        parser.error("--workers must be in [1, 8]")
    cwd = safe(Path.cwd())
    if cwd != WORKSPACE:
        raise RuntimeError(f"run from authorized workspace root: {WORKSPACE}")
    output = safe(args.output if args.output.is_absolute() else cwd / args.output, must_exist=False)
    try:
        if args.preflight_only:
            result = tree_preflight(verify_archive_hashes=args.verify_archive_hashes, quick=True)
            print(json.dumps(result, sort_keys=True))
            return 0 if result["status"] == "COMPLETE" else 2
        if args.full_preflight_only:
            result = tree_preflight(
                verify_archive_hashes=args.verify_archive_hashes, quick=False
            )
            print(json.dumps(result, sort_keys=True))
            return 0 if result["status"] == "COMPLETE" else 2
        if args.verify_only:
            result = verify(output)
        elif args.finalize_existing:
            result = finalize_existing(output)
        else:
            result = create(
                output,
                workers=args.workers,
                verify_archive_hashes=args.verify_archive_hashes,
                resume=args.resume,
                preflight_snapshot=(
                    safe(args.preflight_snapshot if args.preflight_snapshot.is_absolute()
                         else cwd / args.preflight_snapshot)
                    if args.preflight_snapshot is not None else None
                ),
            )
        print(json.dumps(result, sort_keys=True))
        return 0 if result["status"] == "PASS" else 1
    except Exception as exc:  # noqa: BLE001
        print(json.dumps({"status": "ERROR", "error": f"{type(exc).__name__}: {exc}"}), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
