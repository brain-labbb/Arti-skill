#!/usr/bin/env python3
"""Discover complete local source rosters for the Table 1/2/3 release."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import stat
import sys
import xml.etree.ElementTree as ET
from pathlib import Path, PurePosixPath
from typing import Any, Callable
from concurrent.futures import ThreadPoolExecutor

try:
    from table123_full_release_common import freeze_roster, canonical_sha256, sha256_file
except ImportError:  # pragma: no cover
    from .table123_full_release_common import freeze_roster, canonical_sha256, sha256_file


EXPECTED_COUNTS = {
    "Articraft-10K": 9_996,
    "LAM": 3_217,
    "Artiverse": 3_544,
    "PartNet-Mobility": 2_347,
    "PhysX-Mobility": 2_024,
    "SketchMobility": 4_956,
    "Infinite Mobility": 720,
    "Infinigen-Sim": 8_226,
}


def _parallel_rows(
    paths: list[Path],
    builder: Callable[[Path], dict[str, Any]],
    workers: int = 16,
) -> list[dict[str, Any]]:
    """Hash independent packages concurrently while preserving source order."""

    if workers <= 1 or len(paths) < 2:
        return [builder(path) for path in paths]
    with ThreadPoolExecutor(max_workers=workers) as executor:
        return list(executor.map(builder, paths))


def _safe_rel(path: Path, root: Path) -> str:
    try:
        return path.resolve().relative_to(root.resolve()).as_posix()
    except ValueError as exc:
        raise ValueError(f"path escapes source root: {path}") from exc


def _package_files(package: Path) -> list[dict[str, Any]]:
    result = []
    for current, dirs, names in os.walk(package, followlinks=False):
        dirs.sort(); names.sort()
        for name in dirs + names:
            path = Path(current) / name
            mode = os.lstat(path).st_mode
            if stat.S_ISLNK(mode):
                raise ValueError(f"package contains symlink: {path}")
            if not stat.S_ISREG(mode):
                continue
            result.append({"path": path.relative_to(package).as_posix(), "bytes": path.stat().st_size, "sha256": sha256_file(path)})
    return result


def _inspect(urdf: Path) -> tuple[str, dict[str, int], list[dict[str, Any]]]:
    try:
        root = ET.parse(urdf).getroot()
    except (ET.ParseError, OSError):
        return "malformed", {"links": 0, "joints": 0, "fixed_joints": 0, "movable_joints": 0}, []
    joints = []
    for joint in root.findall("joint"):
        typ = str(joint.get("type", "unknown"))
        item = {"name": str(joint.get("name", "")), "type": typ}
        joints.append(item)
    movable = [j for j in joints if j["type"] not in {"fixed"}]
    return "valid", {"links": len(root.findall("link")), "joints": len(joints), "fixed_joints": len(joints) - len(movable), "movable_joints": len(movable)}, movable


def _row(
    urdf: Path,
    *,
    source_root: Path,
    asset_id: str,
    category: str = "",
    package: Path | None = None,
    skip_package_binding: bool = False,
    **extra: Any,
) -> dict[str, Any]:
    package = package or urdf.parent
    status, counts, movable = _inspect(urdf)
    files = [] if skip_package_binding else _package_files(package)
    bound_files = [
        {"path": item["path"], "size": item["bytes"], "sha256": item["sha256"]}
        for item in files
    ]
    rel_urdf = _safe_rel(urdf, source_root)
    rel_package = _safe_rel(package, source_root)
    row: dict[str, Any] = {
        "asset_id": asset_id,
        "ordinal": 0,
        "category": category,
        "raw_category": category,
        "source_path": str(package.resolve()),
        "source_relative_path": rel_package,
        "primary_urdf_path": str(urdf.resolve()),
        "primary_urdf_relative_path": rel_urdf,
        "primary_urdf_bytes": urdf.stat().st_size,
        "primary_urdf_sha256": sha256_file(urdf),
        "package_files": bound_files,
        # The common contract normalizes the legacy `bytes` spelling to
        # `size`; carrying the binding here lets freeze_roster avoid a second
        # recursive read of the package.
        "package_binding_sha256": canonical_sha256(
            bound_files
        ),
        "package_binding": {"files": files, "file_count": len(files), "total_bytes": sum(f.get("size", f.get("bytes", 0)) for f in files), "content_manifest_sha256": canonical_sha256(bound_files)},
        "xml_parse_status": status,
        "parse_status": status,
        "xml_counts": counts,
        "non_fixed_joints": movable,
    }
    row.update(extra)
    return row


def discover_articraft(source_root: Path, *, workers: int = 16) -> list[dict[str, Any]]:
    root = Path(source_root)
    paths: list[Path] = []
    # The release layout is one package directory per asset.  Avoid a full
    # recursive walk through mesh payloads on the network filesystem.
    with os.scandir(root) as entries:
        for entry in entries:
            if not entry.is_dir(follow_symlinks=False):
                continue
            candidate = Path(entry.path) / "model.urdf"
            if candidate.is_file() and not candidate.is_symlink():
                paths.append(candidate)
    ordered = sorted(paths, key=lambda p: p.parent.name)
    return _parallel_rows(
        ordered,
        lambda p: _row(p, source_root=root, asset_id=p.parent.name, package=p.parent),
        workers,
    )


def discover_lam(source_root: Path, *, workers: int = 16) -> list[dict[str, Any]]:
    root = Path(source_root); manifest = root / "manifest.csv"
    if not manifest.is_file():
        raise ValueError(f"missing LAM manifest: {manifest}")
    entries: list[tuple[Path, Path, str, str, str, str, str]] = []
    with manifest.open(newline="", encoding="utf-8") as stream:
        for entry in csv.DictReader(stream):
            rel = str(entry.get("rel_path", "")); asset_id = str(entry.get("object_release_id", ""))
            if not rel or not asset_id:
                raise ValueError("LAM manifest row lacks object_release_id/rel_path")
            package = root / "released_outputs" / rel
            urdf = package / "generated.urdf"
            tier = str(entry.get("tier", ""))
            entries.append((urdf, package, asset_id, rel, str(entry.get("category", "")), tier, str(entry.get("status", ""))))
    raw_counts: dict[str, int] = {}
    for _urdf, _package, asset_id, _rel, _category, _tier, _status in entries:
        raw_counts[asset_id] = raw_counts.get(asset_id, 0) + 1
    entries = [
        (urdf, package, f"{tier}:{rel}" if raw_counts[asset_id] > 1 else asset_id, category, tier, status)
        for urdf, package, asset_id, rel, category, tier, status in entries
    ]
    entry_by_urdf = {item[0]: item[1:] for item in entries}
    def build(urdf: Path) -> dict[str, Any]:
        package, asset_id, category, tier, status = entry_by_urdf[urdf]
        return _row(
            urdf,
            source_root=root,
            asset_id=asset_id,
            category=category,
            package=package,
            release_tier=tier,
            manifest_status=status,
        )
    rows = _parallel_rows([item[0] for item in entries], build, workers)
    return sorted(rows, key=lambda r: (r["asset_id"], r["primary_urdf_relative_path"]))


def discover_artiverse(source_root: Path, *, workers: int = 16) -> list[dict[str, Any]]:
    root = Path(source_root)
    paths: list[Path] = []
    manifest_path = root / "dataset_chunks" / "manifest.json"
    source_base = root
    if not manifest_path.is_file():
        manifest_path = root.parent / "dataset_chunks" / "manifest.json"
        if manifest_path.is_file():
            source_base = root.parent
    if manifest_path.is_file():
        try:
            release = json.loads(manifest_path.read_text(encoding="utf-8"))
            roots = [str(item) for chunk in release.get("chunks", []) for item in chunk.get("roots", [])]
            for root_name in roots:
                package = source_base / root_name / "urdf_w_collider"
                if package.is_dir():
                    paths.extend(sorted(package.glob("*.urdf")))
        except (OSError, ValueError, json.JSONDecodeError):
            paths = []
    if not paths:
        # Fixture/fallback layout.
        paths = [Path(entry.path) for entry in os.scandir(root) if entry.is_file() and entry.name.endswith(".urdf")]
        if not paths:
            for current, _dirs, names in os.walk(root, followlinks=False):
                for name in sorted(names):
                    if name.endswith(".urdf"):
                        paths.append(Path(current) / name)
    def build(urdf: Path) -> dict[str, Any]:
        relative = urdf.relative_to(source_base)
        category_parts = relative.parts[1:] if relative.parts and relative.parts[0] == "data" else relative.parts
        category = category_parts[0] if category_parts else ""
        # The same model basename can occur under multiple source/category
        # roots; bind the complete portable path to keep IDs unique.
        asset_id = relative.with_suffix("").as_posix()
        return _row(urdf, source_root=source_base, asset_id=asset_id, category=category, package=urdf.parent)

    return _parallel_rows(sorted(paths, key=lambda p: p.as_posix()), build, workers)


def discover_partnet(source_root: Path, *, workers: int = 16) -> list[dict[str, Any]]:
    root = Path(source_root)
    paths: list[Path] = []
    with os.scandir(root) as entries:
        for entry in entries:
            if entry.is_dir(follow_symlinks=False):
                candidate = Path(entry.path) / "mobility.urdf"
                if candidate.is_file() and not candidate.is_symlink():
                    paths.append(candidate)
    def build(urdf: Path) -> dict[str, Any]:
        category = ""
        meta = urdf.parent / "meta.json"
        try:
            category = str(json.loads(meta.read_text(encoding="utf-8")).get("model_cat", ""))
        except (OSError, ValueError, json.JSONDecodeError):
            pass
        return _row(urdf, source_root=root, asset_id=urdf.parent.name, category=category, package=urdf.parent)
    return _parallel_rows(sorted(paths, key=lambda p: p.parent.name), build, workers)


def discover_physx(source_root: Path, *, workers: int = 16) -> list[dict[str, Any]]:
    root = Path(source_root); urdf_root = root / "urdf" if (root / "urdf").is_dir() else root
    shared_binding = canonical_sha256({"dataset": "PhysX-Mobility", "source_root": str(root.resolve())})
    def build(urdf: Path) -> dict[str, Any]:
        raw = urdf.stem
        metadata = root / "finaljson" / f"{urdf.stem}.json"
        try:
            declared = json.loads(metadata.read_text(encoding="utf-8")).get("category")
            if isinstance(declared, str) and declared.strip():
                raw = declared  # Exact official raw category label; no alias normalization.
        except (OSError, ValueError, json.JSONDecodeError):
            pass
        row = _row(
            urdf,
            source_root=root,
            asset_id=urdf.stem,
            category=raw,
            package=root,
            skip_package_binding=True,
        )
        row["source_relative_path"] = "."
        row["primary_urdf_relative_path"] = urdf.relative_to(root).as_posix()
        row["package_files"] = []
        row["package_binding_sha256"] = shared_binding
        row["package_binding_deferred"] = True
        # PhysX's official scalar-joint inventory excludes `floating`; retain
        # those declarations as an explicit unsupported count while using the
        # supported movable-joint roster as the Table 3 J denominator.
        parsed = row.get("non_fixed_joints", [])
        supported = [joint for joint in parsed if str(joint.get("type", "")).lower() in {"revolute", "continuous", "prismatic"}]
        row["declared_joint_count_all"] = len(parsed)
        row["unsupported_joint_count"] = len(parsed) - len(supported)
        row["non_fixed_joints"] = supported
        row["xml_counts"]["movable_joints"] = len(supported)
        row["xml_counts"]["unsupported_movable_joints"] = len(parsed) - len(supported)
        row["unsupported_joint_types"] = [str(joint.get("type", "")) for joint in parsed if joint not in supported]
        return row
    return _parallel_rows(sorted(urdf_root.glob("*.urdf")), build, workers)


def discover_sketch(source_root: Path, *, workers: int = 16) -> list[dict[str, Any]]:
    root = Path(source_root)
    paths: list[Path] = []
    # A prior complete release roster is a source-bound index and avoids
    # traversing all mesh files.  Fall back to a bounded walk for fixtures.
    indexed = root.parent.parent / "runtime" / "table1_sketch_mobility_rerun_20260821T021838Z" / "release_roster.jsonl"
    if indexed.is_file():
        try:
            for line in indexed.read_text(encoding="utf-8").splitlines():
                item = json.loads(line)
                relative = item.get("primary_urdf_relative_path") or item.get("rel_path")
                if not relative and item.get("asset_id"):
                    relative = f"{item['asset_id']}/mobility.urdf"
                if isinstance(relative, str) and relative.startswith("data/"):
                    relative = relative[len("data/"):]
                candidate = root / str(relative or "")
                if candidate.is_file():
                    paths.append(candidate)
        except (OSError, ValueError, json.JSONDecodeError):
            paths = []
    if not paths:
        for current, _dirs, names in os.walk(root, followlinks=False):
            if "mobility.urdf" in names:
                paths.append(Path(current) / "mobility.urdf")
    def build(urdf: Path) -> dict[str, Any]:
        rel = urdf.relative_to(root).parts
        category = rel[-3] if len(rel) >= 3 else ""
        asset_id = urdf.relative_to(root).with_suffix("").as_posix()
        return _row(urdf, source_root=root, asset_id=asset_id, category=category, package=urdf.parent)
    return _parallel_rows(sorted(paths, key=lambda p: p.as_posix()), build, workers)


def discover_infinite(cohort_manifest: Path) -> list[dict[str, Any]]:
    data = json.loads(Path(cohort_manifest).read_text(encoding="utf-8"))
    assets = data.get("assets") if isinstance(data, dict) else None
    if not isinstance(assets, list):
        raise ValueError("Infinite cohort manifest lacks assets")
    result = []
    for entry in assets:
        row = dict(entry)
        row.setdefault("category", row.get("factory", "")); row.setdefault("raw_category", row.get("factory", ""))
        package_path = Path(str(row.get("package_path", "")))
        urdf_relpath = str(row.get("urdf_relpath", ""))
        row.setdefault("source_path", str(package_path))
        # The operational manifest stores an absolute package path; the
        # portable roster field is identity-based and must remain relative.
        row.setdefault("source_relative_path", str(row.get("asset_id", "")))
        row.setdefault("primary_urdf_path", str(package_path / urdf_relpath)); row.setdefault("primary_urdf_relative_path", urdf_relpath)
        row.setdefault("primary_urdf_bytes", Path(row["primary_urdf_path"]).stat().st_size if Path(row["primary_urdf_path"]).is_file() else 0)
        row.setdefault("joint_count", int(row.get("declared_joint_count_hint", 0)))
        row.setdefault(
            "non_fixed_joints",
            [
                {"name": f"__declared_joint_{index}", "type": "unknown"}
                for index in range(int(row.get("declared_joint_count_hint", 0)))
            ],
        )
        binding = row.get("package_binding")
        if isinstance(binding, dict) and isinstance(binding.get("files"), list):
            files = [
                {
                    "path": item.get("path"),
                    "size": int(item.get("size", item.get("bytes", 0))),
                    "sha256": item.get("sha256"),
                }
                for item in binding["files"]
                if isinstance(item, dict)
            ]
            # Recompute the binding after normalizing the legacy cohort's
            # `bytes` spelling to the release contract's `size` field.  Do
            # not carry a hash over the pre-normalized source metadata.
            files.sort(key=lambda item: str(item.get("path", "")))
            row["package_files"] = files
            row["package_binding_sha256"] = canonical_sha256(files)
        result.append(row)
    return result


def discover_infinigen(source_root: Path, *, workers: int = 16) -> list[dict[str, Any]]:
    root = Path(source_root)
    paths: list[Path] = []
    # The secure extractor stages each source archive under `<archive>/urdf`.
    # Accept both that staging layout and a directly unpacked `urdf` root.
    candidates = []
    # Directly unpacked fixture/source root (`<root>/<category>/<id>`).
    if root.is_dir() and any(
        child.is_dir() and any(grandchild.is_dir() for grandchild in child.iterdir())
        for child in root.iterdir()
    ):
        candidates.append(root)
    if (root / "urdf").is_dir():
        candidates.append(root / "urdf")
    candidates.extend(
        child / "urdf" for child in sorted(root.iterdir() if root.is_dir() else [])
        if child.is_dir() and (child / "urdf").is_dir()
    )
    for urdf_root in candidates:
        for category in sorted(urdf_root.iterdir()):
            if not category.is_dir():
                continue
            for asset in sorted(category.iterdir()):
                if asset.is_dir():
                    paths.extend(sorted(asset.glob("*.urdf")))
    def build(urdf: Path) -> dict[str, Any]:
        rel = urdf.relative_to(root)
        parts = list(rel.parts)
        if "urdf" in parts:
            parts = parts[parts.index("urdf") + 1 :]
        portable = Path(*parts)
        asset_id = portable.with_suffix("").as_posix()
        row = _row(
            urdf,
            source_root=root,
            asset_id=asset_id,
            category=parts[0] if parts else "",
            package=urdf.parent,
            # Infinigen-Sim archives contain 39 GB of auxiliary meshes and
            # textures.  The archive-level validation receipt is the frozen
            # package binding; recursively hashing that payload once per
            # asset would duplicate the same archive inventory thousands of
            # times.  Table 1/2 still resolve referenced resources, while all
            # three adapters bind and hash the primary URDF itself.
            skip_package_binding=True,
        )
        archive_name = parts[0] if parts else ""
        row["archive_name"] = archive_name
        row["package_binding_deferred"] = True
        row["package_binding_sha256"] = canonical_sha256({"archive_name": archive_name})
        row["package_files"] = []
        return row
    return _parallel_rows(sorted(paths, key=lambda p: p.as_posix()), build, workers)


_DISCOVERERS: dict[str, Callable[[Path], list[dict[str, Any]]]] = {
    "Articraft-10K": discover_articraft, "LAM": discover_lam, "Artiverse": discover_artiverse,
    "PartNet-Mobility": discover_partnet, "PhysX-Mobility": discover_physx, "SketchMobility": discover_sketch,
    "Infinigen-Sim": discover_infinigen,
}

_DATASET_ALIASES = {
    "LAM released outputs": "LAM",
    "Infinite": "Infinite Mobility",
}


def build_roster(dataset: str, *, source_root: Path, output: Path, workers: int = 16) -> Path:
    dataset = _DATASET_ALIASES.get(dataset, dataset)
    if dataset == "Infinite Mobility":
        rows = discover_infinite(source_root)
    elif dataset in _DISCOVERERS:
        rows = _DISCOVERERS[dataset](source_root, workers=workers)
    else:
        raise ValueError(f"unknown dataset: {dataset}")
    bindings = [{"name": dataset, "path": str(Path(source_root).resolve())}]
    if dataset == "Infinigen-Sim":
        # Bind the extracted tree to the read-only archive validation receipt
        # when the standard runtime layout is used.
        receipt = Path(source_root).resolve().parent.parent / "infinigen_archive_validation_receipt.json"
        if receipt.is_file():
            bindings.append({
                "name": "infinigen_archive_validation_receipt",
                "path": str(receipt),
                "sha256": sha256_file(receipt),
            })
    expected = EXPECTED_COUNTS.get(dataset)
    if expected is not None and len(rows) != expected:
        raise ValueError(f"{dataset} release count mismatch: {len(rows)} != {expected}")
    freeze_roster(rows, Path(output), dataset=dataset, source_bindings=bindings)
    return Path(output)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", choices=sorted(EXPECTED_COUNTS))
    parser.add_argument("source_root", type=Path); parser.add_argument("output", type=Path)
    parser.add_argument("--workers", type=int, default=16)
    args = parser.parse_args(argv); build_roster(args.dataset, source_root=args.source_root, output=args.output, workers=args.workers); return 0


if __name__ == "__main__":
    raise SystemExit(main())
