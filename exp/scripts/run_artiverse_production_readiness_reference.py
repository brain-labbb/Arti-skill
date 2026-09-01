#!/usr/bin/env python3
"""Run the Artiverse real-data reference under the frozen Table 7 protocol.

The official chunk manifest fixes all 3,544 requested identities.  A cheap
manifest-driven preflight refuses partial extraction without walking mesh
payloads.  A formal run additionally verifies both archive hashes and two
stable full-tree path/size inventories before freezing its output manifest.

All assets receive URDF/package portability, byte-size, semantic-field proxy,
kinematic-completeness, and physical-completeness audits.  Expensive mesh
topology is restricted to a content-blind N=100 cohort selected solely from
the frozen root strings.
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


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
PROTOCOL = REPO / "exp/reference/table7_production_readiness_protocol_v1.json"
DATASET = REPO / "exp/artiverse"
DATA_ROOT = DATASET / "data"
CHUNKS = DATASET / "dataset_chunks"
CHUNK_MANIFEST = CHUNKS / "manifest.json"
DEFAULT_OUTPUT = REPO / "exp/runtime/table7_artiverse_reference"

EXPECTED_PROTOCOL_ID = "nano3d_table7_production_readiness_v1"
EXPECTED_PROTOCOL_SHA256 = "5fc86932f35f8b66514d5747be732b5c75fef7215c987628f5dd28522f710a7c"
EXPECTED_CHUNK_MANIFEST_SHA256 = "8fa6468254a1f74c58f0c25699598bf88f622fabdaf74f0cd9268ee5663c5586"
EXPECTED_FORMAT = "artiverse-data-tar-gz-chunks-v1"
EXPECTED_REQUESTED = 3544
EXPECTED_FILE_COUNT = 531937
EXPECTED_INPUT_BYTES = 86992752890
EXPECTED_ARCHIVE_BYTES = 65334141104
GEOMETRY_COHORT_SIZE = 100
GEOMETRY_SELECTION_RULE = (
    "ascending SHA256(manifest root UTF-8), then manifest root; take first 100"
)

REQUIRED_OUTPUTS = (
    "protocol_snapshot.json",
    "manifest.json",
    "asset_records.json",
    "summary.json",
    "self_check.json",
    "report.md",
)
MESH_SUFFIXES = {".obj", ".stl", ".ply", ".off", ".dae", ".glb", ".gltf"}
ONE_AXIS_JOINT_TYPES = {"revolute", "continuous", "prismatic"}
RECOGNIZED_JOINT_TYPES = {
    "fixed",
    "revolute",
    "continuous",
    "prismatic",
    "planar",
    "floating",
}
PLACEHOLDER_NAME = re.compile(
    r"^(?:link|part|mesh|object|component|unnamed|default)(?:[_-]?\d+)?$",
    re.IGNORECASE,
)
URI_SCHEME = re.compile(r"^[A-Za-z][A-Za-z0-9+.-]*:")


def contained(path: Path, *, must_exist: bool = True) -> Path:
    """Resolve a path and require it to remain inside this repository."""

    root = REPO.resolve(strict=True)
    resolved = path.resolve(strict=must_exist)
    if resolved != root and root not in resolved.parents:
        raise RuntimeError(f"path outside repository boundary: {resolved}")
    return resolved


def relative(path: Path) -> str:
    return contained(path, must_exist=False).relative_to(REPO).as_posix()


def read_json(path: Path) -> Any:
    return json.loads(contained(path).read_text(encoding="utf-8"))


def write_json(path: Path, value: Any) -> None:
    destination = contained(path, must_exist=False)
    contained(destination.parent).mkdir(parents=True, exist_ok=True)
    temporary = contained(
        destination.with_suffix(destination.suffix + ".tmp"), must_exist=False
    )
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(destination)


def write_text(path: Path, value: str) -> None:
    destination = contained(path, must_exist=False)
    contained(destination.parent).mkdir(parents=True, exist_ok=True)
    temporary = contained(
        destination.with_suffix(destination.suffix + ".tmp"), must_exist=False
    )
    temporary.write_text(value, encoding="utf-8")
    temporary.replace(destination)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with contained(path).open("rb") as stream:
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


def load_frozen_cohort() -> tuple[dict[str, Any], list[dict[str, Any]]]:
    actual_hash = sha256_file(CHUNK_MANIFEST)
    if actual_hash != EXPECTED_CHUNK_MANIFEST_SHA256:
        raise RuntimeError(
            "Artiverse chunk manifest hash drifted: "
            f"expected={EXPECTED_CHUNK_MANIFEST_SHA256} actual={actual_hash}"
        )
    manifest = read_json(CHUNK_MANIFEST)
    scalar_checks = {
        "format": (manifest.get("format"), EXPECTED_FORMAT),
        "data_dir": (manifest.get("data_dir"), "data"),
        "model_count": (manifest.get("model_count"), EXPECTED_REQUESTED),
        "file_count": (manifest.get("file_count"), EXPECTED_FILE_COUNT),
        "input_bytes": (manifest.get("input_bytes"), EXPECTED_INPUT_BYTES),
    }
    drift = {
        name: {"actual": actual, "expected": expected}
        for name, (actual, expected) in scalar_checks.items()
        if actual != expected
    }
    if drift:
        raise RuntimeError(f"Artiverse chunk manifest fields drifted: {drift}")
    chunks = manifest.get("chunks")
    if not isinstance(chunks, list) or len(chunks) != manifest.get("chunk_count"):
        raise RuntimeError("invalid Artiverse chunk entries")

    roots: list[tuple[str, str]] = []
    archive_bytes = 0
    for chunk in chunks:
        archive = chunk.get("archive")
        if not isinstance(archive, str) or Path(archive).name != archive:
            raise RuntimeError(f"unsafe archive name: {archive!r}")
        archive_path = contained(CHUNKS / archive)
        actual_bytes = archive_path.stat().st_size
        if actual_bytes != chunk.get("archive_bytes"):
            raise RuntimeError(f"archive size mismatch: {archive}")
        archive_bytes += actual_bytes
        chunk_roots = chunk.get("roots")
        if not isinstance(chunk_roots, list) or len(chunk_roots) != chunk.get("model_count"):
            raise RuntimeError(f"invalid root list for archive: {archive}")
        roots.extend((str(root), archive) for root in chunk_roots)
    if archive_bytes != EXPECTED_ARCHIVE_BYTES:
        raise RuntimeError("frozen archive-byte total drifted")
    root_names = [root for root, _archive in roots]
    if len(root_names) != EXPECTED_REQUESTED or len(set(root_names)) != EXPECTED_REQUESTED:
        raise RuntimeError("cohort is not exactly 3,544 unique manifest roots")

    cohort: list[dict[str, Any]] = []
    for manifest_index, (root_text, archive) in enumerate(roots):
        root = Path(root_text)
        if root.is_absolute() or ".." in root.parts or len(root.parts) != 4:
            raise RuntimeError(f"unsafe or malformed model root: {root_text!r}")
        if root.parts[0] != "data":
            raise RuntimeError(f"model root is outside data/: {root_text!r}")
        contained(DATASET / root, must_exist=False)
        cohort.append(
            {
                "manifest_index": manifest_index,
                "asset_id": root_text,
                "category": root.parts[1],
                "source_repository": root.parts[2],
                "model_id": root.parts[3],
                "chunk_archive": archive,
                "asset_root": f"exp/artiverse/{root_text}",
                "primary_package": f"exp/artiverse/{root_text}/urdf_w_collider",
            }
        )
    return manifest, cohort


def geometry_cohort(cohort: list[dict[str, Any]]) -> list[dict[str, Any]]:
    ranked = sorted(
        cohort,
        key=lambda row: (
            hashlib.sha256(row["asset_id"].encode("utf-8")).hexdigest(),
            row["asset_id"],
        ),
    )[:GEOMETRY_COHORT_SIZE]
    return [
        {
            "geometry_rank": rank,
            "asset_id": row["asset_id"],
            "manifest_index": row["manifest_index"],
            "selection_sha256": hashlib.sha256(
                row["asset_id"].encode("utf-8")
            ).hexdigest(),
        }
        for rank, row in enumerate(ranked)
    ]


def observed_model_roots() -> set[str]:
    observed: set[str] = set()
    if not DATA_ROOT.is_dir():
        return observed
    for category in sorted(path for path in contained(DATA_ROOT).iterdir() if path.is_dir()):
        category = contained(category)
        for source in sorted(path for path in category.iterdir() if path.is_dir()):
            source = contained(source)
            for model in sorted(path for path in source.iterdir() if path.is_dir()):
                model = contained(model)
                observed.add(model.relative_to(DATASET).as_posix())
    return observed


def quick_preflight() -> tuple[dict[str, Any], list[dict[str, Any]]]:
    manifest, cohort = load_frozen_cohort()
    expected_roots = {row["asset_id"] for row in cohort}
    observed_roots = observed_model_roots()
    missing_roots = sorted(expected_roots - observed_roots)
    unexpected_roots = sorted(observed_roots - expected_roots)

    core_counts: Counter[str] = Counter()
    core_failures: list[dict[str, Any]] = []
    all_core_pass_count = 0
    for row in cohort:
        root = contained(DATASET / row["asset_id"], must_exist=False)
        if not root.is_dir():
            continue
        model_id = row["model_id"]
        segmented = sorted(root.glob("*.segmented.glb"))
        articulations = sorted(root.glob("*.articulations.json"))
        package = root / "urdf_w_collider"
        urdfs = sorted(package.glob("*.urdf")) if package.is_dir() else []
        gates = {
            "one_exact_segmented_glb": (
                len(segmented) == 1 and segmented[0].name == f"{model_id}.segmented.glb"
            ),
            "one_exact_articulation_json": (
                len(articulations) == 1
                and articulations[0].name == f"{model_id}.articulations.json"
            ),
            "material_json": (root / "material.json").is_file(),
            "one_primary_urdf": package.is_dir() and len(urdfs) == 1,
        }
        core_counts.update(name for name, passed in gates.items() if passed)
        if all(gates.values()):
            all_core_pass_count += 1
        else:
            core_failures.append(
                {
                    "asset_id": row["asset_id"],
                    "failed_gates": [name for name, passed in gates.items() if not passed],
                    "segmented_candidates": [path.name for path in segmented],
                    "articulation_candidates": [path.name for path in articulations],
                    "urdf_candidates": [path.name for path in urdfs],
                }
            )

    gates = {
        "chunk_manifest_sha256_exact": True,
        "requested_manifest_roots_exact_3544": len(expected_roots) == EXPECTED_REQUESTED,
        "observed_root_set_exact": not missing_roots and not unexpected_roots,
        "core_four_present_for_all_roots": (
            all_core_pass_count == EXPECTED_REQUESTED
        ),
    }
    blockers: list[str] = []
    if missing_roots:
        blockers.append(f"missing manifest roots: {len(missing_roots)}")
    if unexpected_roots:
        blockers.append(f"unexpected depth-3 roots: {len(unexpected_roots)}")
    if core_failures:
        blockers.append(f"roots failing one or more core-file gates: {len(core_failures)}")
    ready = all(gates.values())
    selected = geometry_cohort(cohort)
    result = {
        "schema_version": "1.0.0",
        "protocol_id": EXPECTED_PROTOCOL_ID,
        "status": "READY" if ready else "BLOCKED",
        "scoring_allowed": ready,
        "mode": "MANIFEST_DRIVEN_QUICK_PREFLIGHT",
        "manifest": {
            "path": relative(CHUNK_MANIFEST),
            "sha256": EXPECTED_CHUNK_MANIFEST_SHA256,
            "format": manifest["format"],
            "declared_model_count": manifest["model_count"],
            "declared_file_count": manifest["file_count"],
            "declared_input_bytes": manifest["input_bytes"],
        },
        "roots": {
            "expected": EXPECTED_REQUESTED,
            "observed": len(observed_roots),
            "missing": len(missing_roots),
            "unexpected": len(unexpected_roots),
            "missing_examples": missing_roots[:20],
            "unexpected_examples": unexpected_roots[:20],
        },
        "core_four": {
            "definition": [
                "one exact {model_id}.segmented.glb",
                "one exact {model_id}.articulations.json",
                "material.json",
                "one top-level URDF in urdf_w_collider",
            ],
            "pass_counts": {
                name: int(core_counts[name])
                for name in (
                    "one_exact_segmented_glb",
                    "one_exact_articulation_json",
                    "material_json",
                    "one_primary_urdf",
                )
            },
            "all_four_pass_assets": all_core_pass_count,
            "present_root_failure_count": len(core_failures),
            "requested_root_not_all_four_count": (
                EXPECTED_REQUESTED - all_core_pass_count
            ),
            "failure_examples": core_failures[:20],
        },
        "geometry_cohort": {
            "size": len(selected),
            "selection_rule": GEOMETRY_SELECTION_RULE,
            "content_blind": True,
            "selection_sha256": canonical_sha256(selected),
            "asset_ids": [row["asset_id"] for row in selected],
        },
        "gates": gates,
        "blockers": blockers,
        "process_detection": {
            "attempted": False,
            "reason": (
                "process-name inspection is not authoritative; formal admission instead "
                "requires exact archives plus two equal complete tree inventories"
            ),
        },
        "large_payload_scan_performed": False,
    }
    return result, cohort


def full_tree_inventory() -> dict[str, Any]:
    """Fingerprint all regular data files in deterministic path order."""

    digest = hashlib.sha256()
    file_count = 0
    total_bytes = 0
    symlink_count = 0
    special_count = 0
    if not DATA_ROOT.is_dir():
        return {
            "file_count": 0,
            "total_bytes": 0,
            "symlink_count": 0,
            "special_count": 0,
            "path_size_manifest_sha256": digest.hexdigest(),
        }
    for directory, directory_names, file_names in os.walk(
        contained(DATA_ROOT), followlinks=False
    ):
        current = contained(Path(directory))
        directory_names.sort()
        file_names.sort()
        for name in file_names:
            path = current / name
            if path.is_symlink():
                symlink_count += 1
                continue
            if not path.is_file():
                special_count += 1
                continue
            path = contained(path)
            size = path.stat().st_size
            relpath = path.relative_to(DATASET).as_posix()
            digest.update(relpath.encode("utf-8"))
            digest.update(b"\0")
            digest.update(str(size).encode("ascii"))
            digest.update(b"\n")
            file_count += 1
            total_bytes += size
    return {
        "file_count": file_count,
        "total_bytes": total_bytes,
        "symlink_count": symlink_count,
        "special_count": special_count,
        "path_size_manifest_sha256": digest.hexdigest(),
    }


def archive_inventory(*, recompute_hashes: bool) -> list[dict[str, Any]]:
    manifest = read_json(CHUNK_MANIFEST)
    records: list[dict[str, Any]] = []
    for chunk in manifest["chunks"]:
        path = contained(CHUNKS / chunk["archive"])
        actual_hash = sha256_file(path) if recompute_hashes else None
        records.append(
            {
                "path": relative(path),
                "bytes": path.stat().st_size,
                "expected_bytes": chunk["archive_bytes"],
                "manifest_sha256": chunk["sha256"],
                "recomputed_sha256": actual_hash,
                "bytes_match": path.stat().st_size == chunk["archive_bytes"],
                "sha256_matches": (
                    actual_hash == chunk["sha256"] if recompute_hashes else None
                ),
            }
        )
    return records


def formal_preflight() -> tuple[dict[str, Any], list[dict[str, Any]]]:
    quick, cohort = quick_preflight()
    if quick["status"] != "READY":
        raise RuntimeError(
            "Artiverse extraction is incomplete; formal scoring refused: "
            + "; ".join(quick["blockers"])
        )
    first = full_tree_inventory()
    archives = archive_inventory(recompute_hashes=True)
    second = full_tree_inventory()
    gates = {
        "quick_preflight_ready": True,
        "tree_file_count_exact": first["file_count"] == EXPECTED_FILE_COUNT,
        "tree_input_bytes_exact": first["total_bytes"] == EXPECTED_INPUT_BYTES,
        "tree_has_no_symlinks_or_special_files": (
            first["symlink_count"] == 0 and first["special_count"] == 0
        ),
        "archives_match_manifest": all(
            row["bytes_match"] and row["sha256_matches"] for row in archives
        ),
        "two_full_tree_inventories_equal": first == second,
    }
    if not all(gates.values()):
        failed = [name for name, passed in gates.items() if not passed]
        raise RuntimeError(f"formal extraction gates failed: {failed}")
    return {
        "status": "READY",
        "scoring_allowed": True,
        "quick": quick,
        "tree_inventory_before_archive_hash": first,
        "tree_inventory_after_archive_hash": second,
        "archives": archives,
        "gates": gates,
    }, cohort


def package_files(package: Path) -> list[Path]:
    files: list[Path] = []
    if not package.is_dir():
        return files
    for directory, directory_names, file_names in os.walk(package, followlinks=False):
        current = contained(Path(directory))
        directory_names.sort()
        file_names.sort()
        for name in file_names:
            path = current / name
            if not path.is_symlink() and path.is_file():
                files.append(contained(path))
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


def urdf_dependency_specs(root: ET.Element) -> list[tuple[str, str]]:
    specs: list[tuple[str, str]] = []
    for node in root.findall(".//mesh"):
        specs.append(("urdf_mesh", node.attrib.get("filename", "")))
    for node in root.findall(".//texture"):
        if "filename" in node.attrib:
            specs.append(("urdf_texture", node.attrib["filename"]))
    return specs


def nested_dependency_specs(path: Path) -> list[tuple[str, str]]:
    specs: list[tuple[str, str]] = []
    suffix = path.suffix.lower()
    if suffix == ".obj":
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            stripped = line.strip()
            if stripped.lower().startswith("mtllib "):
                try:
                    values = shlex.split(stripped[len("mtllib ") :])
                except ValueError:
                    values = [stripped[len("mtllib ") :].strip()]
                specs.extend(("obj_mtllib", value) for value in values if value)
    elif suffix == ".mtl":
        map_keys = {
            "map_ka",
            "map_kd",
            "map_ks",
            "map_ke",
            "map_d",
            "bump",
            "map_bump",
            "disp",
            "decal",
            "refl",
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
    resolved = (declaring_file.parent / raw).resolve(strict=False)
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
    return contained(resolved), None, uses_symlink


def dependency_closure(primary: Path, package: Path) -> dict[str, Any]:
    root = ET.parse(primary).getroot()
    queue: deque[tuple[str, str, Path]] = deque(
        (kind, raw, primary) for kind, raw in urdf_dependency_specs(root)
    )
    seen: set[tuple[str, str, str]] = set()
    visited: set[Path] = set()
    dependency_paths: set[Path] = set()
    mesh_paths: set[Path] = set()
    issues: list[str] = []
    dependency_count = 0
    relative_count = 0
    existing_count = 0
    symlink_count = 0
    while queue:
        kind, raw, declaring = queue.popleft()
        key = (kind, raw, declaring.relative_to(package).as_posix())
        if key in seen:
            continue
        seen.add(key)
        dependency_count += 1
        relative = (
            not bool(URI_SCHEME.match(raw))
            and not Path(raw).is_absolute()
            and not PureWindowsPath(raw).is_absolute()
            and "\\" not in raw
        )
        relative_count += int(relative)
        resolved, error, uses_symlink = resolve_dependency(package, declaring, raw)
        symlink_count += int(uses_symlink)
        if error:
            issues.append(f"{key[2]}: {error}")
            continue
        assert resolved is not None
        existing_count += 1
        dependency_paths.add(resolved)
        if resolved.suffix.lower() in MESH_SUFFIXES:
            mesh_paths.add(resolved)
        if resolved in visited:
            continue
        visited.add(resolved)
        try:
            for nested_kind, nested_raw in nested_dependency_specs(resolved):
                queue.append((nested_kind, nested_raw, resolved))
        except Exception as exc:  # noqa: BLE001
            issues.append(
                f"cannot parse nested dependencies in {resolved.relative_to(package)}: "
                f"{type(exc).__name__}: {exc}"
            )
    return {
        "root": root,
        "dependency_count": dependency_count,
        "relative_dependency_count": relative_count,
        "existing_dependency_count": existing_count,
        "dependency_symlink_count": symlink_count,
        "dependency_paths": sorted(dependency_paths),
        "mesh_paths": sorted(mesh_paths),
        "issues": issues,
    }


def portable_package_audit(
    package: Path, primary: Path | None, scratch_root: Path
) -> dict[str, Any]:
    if primary is None or not package.is_dir():
        return {
            "state": "not_evaluable",
            "reason": "no unambiguous native URDF package is available",
            "fresh_workspace_copy": False,
            "issues": [],
        }
    package = contained(package)
    primary = contained(primary)
    original_inventory = inventory_signature(package)
    prefix = hashlib.sha256(str(package).encode("utf-8")).hexdigest()[:12] + "-"
    temporary = contained(
        Path(tempfile.mkdtemp(prefix=prefix, dir=contained(scratch_root)))
    )
    copied = contained(temporary / "package", must_exist=False)
    issues: list[str] = []
    closure: dict[str, Any] | None = None
    copied_inventory: dict[str, Any] | None = None
    try:
        shutil.copytree(package, copied, symlinks=True)
        copied = contained(copied)
        copied_inventory = inventory_signature(copied)
        if copied_inventory != original_inventory:
            issues.append("complete-package path/size inventory differs after clean copy")
        copied_primary = contained(copied / primary.relative_to(package))
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
        "primary_artifact_parsed_after_copy": closure is not None,
        "declared_dependency_count": closure["dependency_count"] if closure else 0,
        "all_dependencies_relative": bool(
            closure and closure["relative_dependency_count"] == closure["dependency_count"]
        ),
        "all_dependencies_exist": bool(
            closure and closure["existing_dependency_count"] == closure["dependency_count"]
        ),
        "required_dependency_symlink_count": (
            closure["dependency_symlink_count"] if closure else 0
        ),
        "issues": issues,
    }


def geometry_not_selected() -> dict[str, Any]:
    return {
        "state": "not_evaluable",
        "selected": False,
        "evaluable": False,
        "reason": "outside the frozen content-blind N=100 topology cohort",
        "mesh_payload_count": None,
        "readable_mesh_payload_count": None,
        "readable_geometry_count": None,
        "load_errors": [],
        "geometries": [],
    }


def geometry_audit(mesh_paths: list[Path], package: Path) -> dict[str, Any]:
    try:
        import numpy as np
        import trimesh
    except ImportError as exc:
        raise RuntimeError("formal topology audit requires numpy and trimesh") from exc

    geometries: list[dict[str, Any]] = []
    load_errors: list[str] = []
    readable_meshes = 0
    for path in mesh_paths:
        name = path.relative_to(package).as_posix()
        try:
            loaded = trimesh.load(path, force="scene", process=False)
            items = list(getattr(loaded, "geometry", {}).items())
            if not items:
                raise ValueError("no geometry in loaded scene")
            readable_meshes += 1
            for geometry_name, mesh in items:
                faces = np.asarray(mesh.faces)
                if faces.ndim != 2 or faces.shape[1] != 3 or len(faces) == 0:
                    raise ValueError(f"non-triangle or empty geometry: {geometry_name}")
                edges = np.asarray(mesh.edges_sorted)
                _unique, edge_counts = np.unique(edges, axis=0, return_counts=True)
                repeated = (
                    (faces[:, 0] == faces[:, 1])
                    | (faces[:, 1] == faces[:, 2])
                    | (faces[:, 0] == faces[:, 2])
                )
                areas = np.asarray(mesh.area_faces)
                area_bad = ~np.isfinite(areas) | (areas <= 1e-12)
                geometries.append(
                    {
                        "mesh_path": name,
                        "geometry_name": str(geometry_name),
                        "vertices": int(len(mesh.vertices)),
                        "faces": int(len(faces)),
                        "watertight": bool(mesh.is_watertight),
                        "edge_manifold_proxy": int(np.count_nonzero(edge_counts > 2)) == 0,
                        "open_edges": int(np.count_nonzero(edge_counts == 1)),
                        "nonmanifold_edges": int(np.count_nonzero(edge_counts > 2)),
                        "degenerate_faces": int(np.count_nonzero(repeated | area_bad)),
                    }
                )
        except Exception as exc:  # noqa: BLE001
            load_errors.append(f"{name}: {type(exc).__name__}: {exc}")
    evaluable = bool(geometries) and not load_errors
    count = len(geometries)
    watertight = sum(row["watertight"] for row in geometries)
    manifold = sum(row["edge_manifold_proxy"] for row in geometries)
    open_edges = sum(row["open_edges"] for row in geometries)
    degenerate = sum(row["degenerate_faces"] for row in geometries)
    return {
        "state": "pass" if evaluable else "not_evaluable",
        "selected": True,
        "evaluable": evaluable,
        "reason": None if evaluable else "one or more packaged meshes were unreadable",
        "mesh_payload_count": len(mesh_paths),
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


def semantic_audit(
    root: ET.Element | None, parse_error: str | None
) -> tuple[dict[str, Any], dict[str, Any]]:
    strict = {
        "state": "not_evaluable",
        "reason": "no output-independent required-part and role specification exists",
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
    links = {node.attrib.get("name", "") for node in root.findall("link")}
    failures: dict[str, list[str]] = {
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
            failures["parent_child_references"].append(name)
        if joint_type not in RECOGNIZED_JOINT_TYPES:
            failures["recognized_joint_types"].append(name)
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
                failures["movable_one_axis_joint_axes"].append(name)
        if joint_type in {"revolute", "prismatic"}:
            limit = joint.find("limit")
            lower = finite_number(limit.attrib.get("lower")) if limit is not None else None
            upper = finite_number(limit.attrib.get("upper")) if limit is not None else None
            if lower is None or upper is None or lower > upper:
                failures["revolute_prismatic_limits"].append(name)
        origin = joint.find("origin")
        if not (
            origin is not None
            and finite_vector(origin.attrib.get("xyz"), default_zero=True)
            and finite_vector(origin.attrib.get("rpy"), default_zero=True)
        ):
            failures["explicit_finite_joint_origins"].append(name)
    gates = {name: not rows for name, rows in failures.items()}
    return {
        "state": "pass" if all(gates.values()) else "fail",
        "joint_count": len(joints),
        "movable_joint_count": movable,
        "gates": gates,
        "failing_joints_by_gate": failures,
    }


def inertia_positive_definite(node: ET.Element | None) -> bool:
    if node is None:
        return False
    values = {
        key: finite_number(node.attrib.get(key))
        for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
    }
    if any(value is None for value in values.values()):
        return False
    ixx = values["ixx"]
    ixy = values["ixy"]
    ixz = values["ixz"]
    iyy = values["iyy"]
    iyz = values["iyz"]
    izz = values["izz"]
    assert all(value is not None for value in (ixx, ixy, ixz, iyy, iyz, izz))
    minor2 = ixx * iyy - ixy * ixy
    determinant = (
        ixx * (iyy * izz - iyz * iyz)
        - ixy * (ixy * izz - iyz * ixz)
        + ixz * (ixy * iyz - iyy * ixz)
    )
    return bool(ixx > 0.0 and minor2 > 0.0 and determinant > 0.0)


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


def physical_audit(root: ET.Element | None, parse_error: str | None) -> dict[str, Any]:
    if root is None:
        return {"state": "not_evaluable", "reason": parse_error or "native URDF unavailable"}
    links = root.findall("link")
    if not links:
        return {"state": "not_evaluable", "reason": "native URDF has no physical links"}
    link_failures: dict[str, list[str]] = {
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
        **{name: not rows for name, rows in link_failures.items()},
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


def audit_asset(
    row: dict[str, Any], geometry_ids: set[str], scratch_root_text: str
) -> dict[str, Any]:
    root_path = contained(REPO / row["asset_root"], must_exist=False)
    package = contained(REPO / row["primary_package"], must_exist=False)
    candidates = sorted(package.glob("*.urdf")) if package.is_dir() else []
    primary = contained(candidates[0]) if len(candidates) == 1 else None
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
    mesh_paths = sorted({path for path in files if path.suffix.lower() in MESH_SUFFIXES})
    selected = row["asset_id"] in geometry_ids
    geometry = geometry_audit(mesh_paths, package) if selected else geometry_not_selected()
    strict_semantic, field_proxy = semantic_audit(parsed, parse_error)
    kinematic = kinematic_audit(parsed, parse_error)
    physical = physical_audit(parsed, parse_error)
    portable = portable_package_audit(
        package, primary, contained(Path(scratch_root_text))
    )
    inventory = [
        {"path": path.relative_to(package).as_posix(), "bytes": path.stat().st_size}
        for path in files
    ]
    inventory.sort(key=lambda item: item["path"])
    return {
        **row,
        "availability": {
            "state": "pass" if primary is not None and parsed is not None else "fail",
            "asset_root_present": root_path.is_dir(),
            "primary_package_present": package.is_dir(),
            "primary_urdf_candidate_count": len(candidates),
            "primary_urdf": primary.relative_to(root_path).as_posix() if primary else None,
            "primary_urdf_parsed": parsed is not None,
            "parse_error": parse_error,
        },
        "integrity": {
            "primary_urdf_sha256": sha256_file(primary) if primary else None,
            "package_file_count": len(files),
            "package_path_size_manifest_sha256": canonical_sha256(inventory),
        },
        "size_bytes": {
            "source": None,
            "source_reason": "released data reference has no per-asset generator source",
            "urdf": primary.stat().st_size if primary else None,
            "mesh": sum(path.stat().st_size for path in mesh_paths) if mesh_paths else None,
            "mesh_payload_count": len(mesh_paths),
            "primary_package": sum(path.stat().st_size for path in files) if files else None,
        },
        "geometry": geometry,
        "portable_package": portable,
        "deterministic_build": {
            "state": "not_evaluable",
            "fresh_build_count": 0,
            "reason": "static data reference; no two fresh frozen builds were executed",
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
        if result["evaluable_denominator"]
        else None
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
            sum(
                isinstance(row.get(key, {}).get("gates", {}).get(name), bool)
                for row in rows
            ),
        )
        for name in names
    }
    return result


def mean_or_none(values: Iterable[float]) -> float | None:
    rows = list(values)
    return sum(rows) / len(rows) if rows else None


def geometry_summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
    selected = [row for row in rows if row["geometry"]["selected"]]
    evaluable = [row for row in selected if row["geometry"]["evaluable"]]
    geometries = [item for row in evaluable for item in row["geometry"]["geometries"]]
    watertight = sum(item["watertight"] for item in geometries)
    manifold = sum(item["edge_manifold_proxy"] for item in geometries)
    open_edges = sum(item["open_edges"] for item in geometries)
    degenerate = sum(item["degenerate_faces"] for item in geometries)
    return {
        "selection": {
            "requested_assets": len(selected),
            "full_asset_cohort": len(rows),
            "rule": GEOMETRY_SELECTION_RULE,
            "content_blind": True,
        },
        "geometry_evaluable_assets": len(evaluable),
        "geometry_not_evaluable_assets": len(selected) - len(evaluable),
        "mesh_load_error_assets": sum(bool(row["geometry"]["load_errors"]) for row in selected),
        "mesh_load_error_count": sum(len(row["geometry"]["load_errors"]) for row in selected),
        "no_mesh_payload_assets": sum(not row["geometry"]["mesh_payload_count"] for row in selected),
        "readable_mesh_payloads": sum(
            row["geometry"]["readable_mesh_payload_count"] for row in evaluable
        ),
        "readable_geometries": len(geometries),
        "mesh_scope": "all packaged mesh payloads under urdf_w_collider for frozen N=100",
        "watertight": {
            "geometry_level": ratio(watertight, len(geometries)),
            "per_asset_mean_fraction": mean_or_none(
                row["geometry"]["watertight"]["rate"] for row in evaluable
            ),
        },
        "manifold": {
            "definition": "edge-manifold proxy; vertex-manifold is not claimed",
            "geometry_level": ratio(manifold, len(geometries)),
            "per_asset_mean_fraction": mean_or_none(
                row["geometry"]["manifold"]["rate"] for row in evaluable
            ),
        },
        "open_edges": {
            "total": open_edges,
            "per_asset_mean": open_edges / len(evaluable) if evaluable else None,
        },
        "degenerate_faces": {
            "total": degenerate,
            "per_asset_mean": degenerate / len(evaluable) if evaluable else None,
        },
        "self_intersection": {
            "state": "not_evaluable",
            "denominator": len(selected),
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


def make_summary(
    records: list[dict[str, Any]], manifest_hash: str, protocol_hash: str,
    formal: dict[str, Any], final_tree: dict[str, Any]
) -> dict[str, Any]:
    available = sum(row["availability"]["state"] == "pass" for row in records)
    return {
        "schema_version": "1.0.0",
        "protocol_id": EXPECTED_PROTOCOL_ID,
        "status": "COMPLETE",
        "interpretation": "ARTIVERSE_REAL_DATA_REFERENCE_STATIC_AUDIT",
        "cohort": {
            "requested_assets": len(records),
            "available_assets": available,
            "unavailable_assets": len(records) - available,
            "package_evaluable_assets": sum(
                row["portable_package"]["state"] in {"pass", "fail"} for row in records
            ),
            "geometry_requested_assets": GEOMETRY_COHORT_SIZE,
            "selection": "all 3,544 frozen manifest identities; no outcome filtering",
        },
        "provenance": {
            "chunk_manifest": formal["quick"]["manifest"],
            "archives_recomputed_and_matched": True,
            "complete_extraction": True,
            "tree_inventory_before": formal["tree_inventory_after_archive_hash"],
            "tree_inventory_after": final_tree,
        },
        "results": {
            "geometry": geometry_summary(records),
            "size_bytes": {
                "source": {
                    "state": "not_evaluable",
                    "denominator": 0,
                    "reason": "released data reference has no attributable generator source package",
                },
                "urdf": byte_summary(records, "urdf"),
                "mesh": byte_summary(records, "mesh"),
                "primary_package": byte_summary(records, "primary_package"),
            },
            "portable_package": gate_partition(records, "portable_package"),
            "deterministic_build": state_partition(records, "deterministic_build.state"),
            "semantic_complete": state_partition(records, "semantic_complete.state"),
            "semantic_field_proxy": gate_partition(records, "semantic_field_proxy"),
            "kinematic_complete": gate_partition(records, "kinematic_complete"),
            "physical_complete": gate_partition(records, "physical_complete"),
        },
        "fail_closed": {
            "self_intersection": "not_evaluable; no exact backend",
            "deterministic_build": "not_evaluable; no two fresh builds",
            "semantic_complete": "not_evaluable; no independent required-part/role gold",
            "runtime_physics_defaults_count_as_native": False,
        },
        "hashes": {
            "protocol_sha256": protocol_hash,
            "manifest_sha256": manifest_hash,
            "runner_sha256": sha256_file(SCRIPT),
            "chunk_manifest_sha256": EXPECTED_CHUNK_MANIFEST_SHA256,
        },
    }


def make_manifest(
    formal: dict[str, Any], cohort: list[dict[str, Any]], output: Path
) -> dict[str, Any]:
    selected = geometry_cohort(cohort)
    selected_ids = {row["asset_id"] for row in selected}
    assets = [
        {**row, "geometry_selected": row["asset_id"] in selected_ids}
        for row in cohort
    ]
    return {
        "schema_version": "1.0.0",
        "protocol_id": EXPECTED_PROTOCOL_ID,
        "method": "Artiverse",
        "role": "real-data reference",
        "axis": "Table 7 Production Readiness",
        "status": "FROZEN_BEFORE_SCORING",
        "manifest_frozen_before_scoring": True,
        "protocol_snapshot": relative(output / "protocol_snapshot.json"),
        "protocol_sha256": EXPECTED_PROTOCOL_SHA256,
        "selection_policy": {
            "full_reference": "all roots in official chunk-manifest order",
            "requested_assets": len(cohort),
            "outcome_based_filtering": False,
            "missing_or_failed_assets_retained": True,
        },
        "geometry_topology_cohort": {
            "requested_assets": len(selected),
            "selection_rule": GEOMETRY_SELECTION_RULE,
            "content_blind": True,
            "selected_before_scoring": True,
            "selection_sha256": canonical_sha256(selected),
            "assets": selected,
        },
        "representation_adapter": {
            "asset_unit": "category/source_repository/model_id root",
            "native_simulation_package": "urdf_w_collider directory",
            "primary_artifact": "exactly one top-level *.urdf",
            "portable_scope": "clean copy of complete urdf_w_collider plus recursive dependency checks",
            "geometry_scope": "all packaged mesh payloads for frozen content-blind N=100",
            "bytes_scope": "all 3,544 assets",
            "semantic_complete": "not_evaluable; field proxy is reported separately",
        },
        "frozen_inputs": {
            "chunk_manifest": formal["quick"]["manifest"],
            "archives": formal["archives"],
            "complete_tree_inventory": formal["tree_inventory_after_archive_hash"],
            "formal_preflight_gates": formal["gates"],
            "runner": {"path": relative(SCRIPT), "sha256": sha256_file(SCRIPT)},
        },
        "assets": assets,
    }


def run_assets(
    cohort: list[dict[str, Any]], output: Path, workers: int, resume: bool,
    run_key: str, geometry_ids: set[str]
) -> list[dict[str, Any]]:
    partial = output / "asset_records.partial.jsonl"
    meta_path = output / "asset_records.partial.meta.json"
    existing: dict[str, dict[str, Any]] = {}
    if resume:
        if not partial.is_file() or not meta_path.is_file():
            raise RuntimeError("--resume requested but compatible partial files do not exist")
        meta = read_json(meta_path)
        if meta.get("run_key") != run_key:
            raise RuntimeError("partial run key does not match current frozen inputs and runner")
        for line in partial.read_text(encoding="utf-8").splitlines():
            if line.strip():
                record = json.loads(line)
                asset_id = record["asset_id"]
                if asset_id in existing:
                    raise RuntimeError(f"duplicate partial asset record: {asset_id}")
                existing[asset_id] = record
    else:
        if partial.exists() or meta_path.exists():
            raise RuntimeError("partial output already exists; use --resume or a fresh output")
        write_json(meta_path, {"run_key": run_key, "requested_assets": len(cohort)})
        contained(partial, must_exist=False).write_text("", encoding="utf-8")

    remaining = [row for row in cohort if row["asset_id"] not in existing]
    scratch = contained(output / ".portable_tmp", must_exist=False)
    scratch.mkdir(parents=True, exist_ok=True)
    scratch = contained(scratch)
    with contained(partial, must_exist=False).open("a", encoding="utf-8") as stream:
        if workers == 1:
            for row in remaining:
                record = audit_asset(row, geometry_ids, str(scratch))
                existing[row["asset_id"]] = record
                stream.write(json.dumps(record, sort_keys=True, ensure_ascii=True) + "\n")
                stream.flush()
                if len(existing) % 25 == 0 or len(existing) == len(cohort):
                    print(f"[Artiverse Table 7 reference] {len(existing)}/{len(cohort)}", flush=True)
        else:
            with ProcessPoolExecutor(max_workers=workers) as pool:
                futures = {
                    pool.submit(audit_asset, row, geometry_ids, str(scratch)): row
                    for row in remaining
                }
                for future in as_completed(futures):
                    row = futures[future]
                    record = future.result()
                    existing[row["asset_id"]] = record
                    stream.write(json.dumps(record, sort_keys=True, ensure_ascii=True) + "\n")
                    stream.flush()
                    if len(existing) % 25 == 0 or len(existing) == len(cohort):
                        print(
                            f"[Artiverse Table 7 reference] {len(existing)}/{len(cohort)}",
                            flush=True,
                        )
    leftovers = list(scratch.iterdir())
    if leftovers:
        raise RuntimeError(f"portable scratch is not empty: {len(leftovers)} entries")
    records = [existing[row["asset_id"]] for row in cohort]
    if len(records) != EXPECTED_REQUESTED:
        raise RuntimeError("asset-record count is incomplete")
    return records


def self_checks(
    output: Path, protocol: dict[str, Any], manifest: dict[str, Any],
    records: list[dict[str, Any]], summary: dict[str, Any], final_quick: dict[str, Any],
    final_tree: dict[str, Any]
) -> dict[str, bool]:
    ids = [row["asset_id"] for row in manifest["assets"]]
    record_ids = [row["asset_id"] for row in records]
    selected = [row["asset_id"] for row in manifest["geometry_topology_cohort"]["assets"]]
    result = summary["results"]
    full_partitions = (
        result["portable_package"],
        result["deterministic_build"],
        result["semantic_complete"],
        result["semantic_field_proxy"],
        result["kinematic_complete"],
        result["physical_complete"],
    )
    return {
        "required_outputs_match_protocol": tuple(protocol.get("required_outputs", [])) == REQUIRED_OUTPUTS,
        "all_required_outputs_exist": all((output / name).is_file() for name in REQUIRED_OUTPUTS),
        "protocol_snapshot_exact": sha256_file(output / "protocol_snapshot.json") == EXPECTED_PROTOCOL_SHA256,
        "live_chunk_manifest_hash_exact": sha256_file(CHUNK_MANIFEST) == EXPECTED_CHUNK_MANIFEST_SHA256,
        "manifest_hash_recorded": sha256_file(output / "manifest.json") == summary["hashes"]["manifest_sha256"],
        "runner_hash_recorded": sha256_file(SCRIPT) == summary["hashes"]["runner_sha256"],
        "manifest_identities_unique": len(ids) == len(set(ids)) == EXPECTED_REQUESTED,
        "asset_records_match_manifest_order": record_ids == ids,
        "availability_conserved": (
            summary["cohort"]["requested_assets"]
            == summary["cohort"]["available_assets"] + summary["cohort"]["unavailable_assets"]
        ),
        "full_state_partitions_conserved": all(
            axis["pass"] + axis["fail"] + axis["not_evaluable"]
            == axis["denominator"]
            == EXPECTED_REQUESTED
            for axis in full_partitions
        ),
        "geometry_selection_exact_n100": len(selected) == len(set(selected)) == GEOMETRY_COHORT_SIZE,
        "geometry_records_match_frozen_selection": {
            row["asset_id"] for row in records if row["geometry"]["selected"]
        } == set(selected),
        "geometry_selection_hash_exact": (
            canonical_sha256(manifest["geometry_topology_cohort"]["assets"])
            == manifest["geometry_topology_cohort"]["selection_sha256"]
        ),
        "strict_semantics_not_proxy": result["semantic_complete"]["not_evaluable"] == EXPECTED_REQUESTED,
        "determinism_not_rehash_shortcut": result["deterministic_build"]["not_evaluable"] == EXPECTED_REQUESTED,
        "final_quick_preflight_ready": final_quick["status"] == "READY",
        "final_tree_matches_frozen_tree": (
            final_tree == manifest["frozen_inputs"]["complete_tree_inventory"]
        ),
        "manifest_frozen_before_scoring": manifest["manifest_frozen_before_scoring"] is True,
        "outcome_filtering_disabled": manifest["selection_policy"]["outcome_based_filtering"] is False,
    }


def fmt(value: float | None, digits: int = 3) -> str:
    return "N/A" if value is None else f"{value:.{digits}f}"


def report_text(summary: dict[str, Any]) -> str:
    cohort = summary["cohort"]
    result = summary["results"]
    geometry = result["geometry"]
    return f"""# Table 7: Artiverse real-data production-readiness reference

Status: **COMPLETE**

The full reference retains all {cohort['requested_assets']} identities from the exact
official chunk manifest. No unavailable, malformed, or failed asset is replaced or
removed. Expensive mesh topology uses a separate content-blind frozen N={GEOMETRY_COHORT_SIZE}
cohort; all other locally measurable axes use N={cohort['requested_assets']}.

## Cohorts

- Full requested/available: {cohort['requested_assets']}/{cohort['available_assets']}.
- Portable-package evaluable: {cohort['package_evaluable_assets']}.
- Geometry requested/evaluable: {geometry['selection']['requested_assets']}/{geometry['geometry_evaluable_assets']}.
- Geometry selection: {GEOMETRY_SELECTION_RULE}.

## Results

| Axis | Result |
|---|---:|
| Watertight | {geometry['watertight']['geometry_level']['numerator']}/{geometry['watertight']['geometry_level']['denominator']} readable geometries; {fmt(geometry['watertight']['per_asset_mean_fraction'], 6)} mean/asset |
| Edge-manifold proxy | {geometry['manifold']['geometry_level']['numerator']}/{geometry['manifold']['geometry_level']['denominator']} readable geometries; {fmt(geometry['manifold']['per_asset_mean_fraction'], 6)} mean/asset |
| Open edges | {geometry['open_edges']['total']} total; {fmt(geometry['open_edges']['per_asset_mean'])}/evaluable geometry asset |
| Degenerate faces | {geometry['degenerate_faces']['total']} total; {fmt(geometry['degenerate_faces']['per_asset_mean'])}/evaluable geometry asset |
| Portable Package | {result['portable_package']['pass']}/{result['portable_package']['evaluable_denominator']} evaluable |
| Semantic field proxy | {result['semantic_field_proxy']['pass']}/{result['semantic_field_proxy']['evaluable_denominator']} evaluable |
| Kinematic Complete | {result['kinematic_complete']['pass']}/{result['kinematic_complete']['evaluable_denominator']} applicable |
| Physical Complete | {result['physical_complete']['pass']}/{result['physical_complete']['evaluable_denominator']} applicable |

## Evidence boundaries

- Strict semantic completeness is not evaluable without output-independent part/role gold; the named-part/tree result is only a field proxy.
- Deterministic Build is not evaluable because two fresh builds were not run.
- Self-intersection is not evaluable because no exact adjacent-face-excluding backend ran.
- Simulator-provided defaults do not count as native physical metadata.

## Reproduction

```bash
python exp/scripts/run_artiverse_production_readiness_reference.py --preflight-only
python exp/scripts/run_artiverse_production_readiness_reference.py --workers 2
python exp/scripts/run_artiverse_production_readiness_reference.py --verify-only
```
"""


def create(output: Path, *, workers: int, resume: bool) -> dict[str, Any]:
    output = contained(output, must_exist=False)
    if output.exists() and not resume and any(output.iterdir()):
        raise RuntimeError("output is not empty; use --resume or a fresh output directory")
    protocol = read_json(PROTOCOL)
    if protocol.get("protocol_id") != EXPECTED_PROTOCOL_ID:
        raise RuntimeError("Table 7 protocol id drifted")
    if sha256_file(PROTOCOL) != EXPECTED_PROTOCOL_SHA256:
        raise RuntimeError("Table 7 protocol hash drifted")
    if tuple(protocol.get("required_outputs", [])) != REQUIRED_OUTPUTS:
        raise RuntimeError("Table 7 required-output contract drifted")

    formal, cohort = formal_preflight()
    parent = contained(output.parent, must_exist=False)
    parent.mkdir(parents=True, exist_ok=True)
    output.mkdir(parents=True, exist_ok=True)
    output = contained(output)
    snapshot = output / "protocol_snapshot.json"
    if not resume:
        temporary = output / "protocol_snapshot.json.tmp"
        shutil.copyfile(contained(PROTOCOL), contained(temporary, must_exist=False))
        temporary.replace(snapshot)
        manifest = make_manifest(formal, cohort, output)
        write_json(output / "manifest.json", manifest)
    else:
        if not snapshot.is_file() or not (output / "manifest.json").is_file():
            raise RuntimeError("--resume requires frozen protocol_snapshot.json and manifest.json")
        if sha256_file(snapshot) != EXPECTED_PROTOCOL_SHA256:
            raise RuntimeError("resume protocol snapshot drifted")
        manifest = read_json(output / "manifest.json")
        expected_manifest = make_manifest(formal, cohort, output)
        if manifest != expected_manifest:
            raise RuntimeError("resume manifest no longer matches frozen inputs and runner")
    manifest_hash = sha256_file(output / "manifest.json")
    run_key = canonical_sha256(
        {
            "protocol": EXPECTED_PROTOCOL_SHA256,
            "chunk_manifest": EXPECTED_CHUNK_MANIFEST_SHA256,
            "runner": sha256_file(SCRIPT),
            "manifest": manifest_hash,
            "tree": formal["tree_inventory_after_archive_hash"],
        }
    )
    selected_ids = {
        row["asset_id"] for row in manifest["geometry_topology_cohort"]["assets"]
    }
    records = run_assets(cohort, output, workers, resume, run_key, selected_ids)
    write_json(output / "asset_records.json", records)

    final_quick, _cohort = quick_preflight()
    final_tree = full_tree_inventory()
    if final_quick["status"] != "READY":
        raise RuntimeError("data-tree core gates changed during scoring")
    if final_tree != formal["tree_inventory_after_archive_hash"]:
        raise RuntimeError("data-tree path/size inventory changed during scoring")

    write_json(output / "self_check.json", {"status": "PROVISIONAL"})
    summary = make_summary(
        records,
        manifest_hash,
        EXPECTED_PROTOCOL_SHA256,
        formal,
        final_tree,
    )
    write_json(output / "summary.json", summary)
    write_text(output / "report.md", report_text(summary))
    checks = self_checks(
        output, protocol, manifest, records, summary, final_quick, final_tree
    )
    artifact_hashes = {
        name: sha256_file(output / name)
        for name in REQUIRED_OUTPUTS
        if name != "self_check.json"
    }
    self_check = {
        "schema_version": "1.0.0",
        "protocol_id": EXPECTED_PROTOCOL_ID,
        "status": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks,
        "protocol_sha256": EXPECTED_PROTOCOL_SHA256,
        "manifest_sha256": manifest_hash,
        "artifact_hashes": artifact_hashes,
        "artifact_hash_scope": "required outputs except self_check.json",
    }
    write_json(output / "self_check.json", self_check)
    if self_check["status"] != "PASS":
        failed = [name for name, passed in checks.items() if not passed]
        raise RuntimeError(f"self-check failed: {failed}")
    return verify(output)


def verify(output: Path) -> dict[str, Any]:
    output = contained(output)
    protocol = read_json(output / "protocol_snapshot.json")
    manifest = read_json(output / "manifest.json")
    records = read_json(output / "asset_records.json")
    summary = read_json(output / "summary.json")
    recorded = read_json(output / "self_check.json")
    final_quick, _cohort = quick_preflight()
    final_tree = full_tree_inventory()
    checks = self_checks(
        output, protocol, manifest, records, summary, final_quick, final_tree
    )
    errors = [name for name, passed in checks.items() if not passed]
    for name, expected in recorded.get("artifact_hashes", {}).items():
        if sha256_file(output / name) != expected:
            errors.append(f"artifact_hash:{name}")
    if recorded.get("status") != "PASS":
        errors.append("recorded_self_check_status")
    return {
        "status": "PASS" if not errors else "FAIL",
        "errors": errors,
        "audit_status": summary.get("status"),
        "requested_assets": len(records),
        "available_assets": summary["cohort"]["available_assets"],
        "geometry_requested_assets": summary["cohort"]["geometry_requested_assets"],
        "output": relative(output),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--workers", type=int, default=2)
    parser.add_argument("--resume", action="store_true")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--preflight-only", action="store_true")
    mode.add_argument("--verify-only", action="store_true")
    args = parser.parse_args()
    if not 1 <= args.workers <= 8:
        parser.error("--workers must be in [1, 8]")
    cwd = contained(Path.cwd())
    if cwd != REPO:
        raise RuntimeError(f"run from repository root: {REPO}")
    output = contained(
        args.output if args.output.is_absolute() else cwd / args.output,
        must_exist=False,
    )
    try:
        if args.preflight_only:
            result, _cohort = quick_preflight()
            print(json.dumps(result, sort_keys=True))
            return 0 if result["status"] == "READY" else 2
        result = verify(output) if args.verify_only else create(
            output, workers=args.workers, resume=args.resume
        )
        print(json.dumps(result, sort_keys=True))
        return 0 if result["status"] == "PASS" else 1
    except Exception as exc:  # noqa: BLE001
        print(
            json.dumps({"status": "ERROR", "error": f"{type(exc).__name__}: {exc}"}),
            file=sys.stderr,
        )
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
