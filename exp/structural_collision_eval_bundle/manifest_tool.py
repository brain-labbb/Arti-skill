#!/usr/bin/env python3
"""Create, normalize, rebase, and validate portable asset manifests."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import re
import tempfile
from typing import Any, Iterable, Mapping, Sequence
import xml.etree.ElementTree as ET


SCHEMA_VERSION = "articulated_integrity_manifest_jsonl_v1"


def _atomic_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            handle.write(text)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def _canonical_hash(rows: Sequence[Mapping[str, Any]]) -> str:
    payload = json.dumps(
        rows, ensure_ascii=True, allow_nan=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _read_rows(path: Path) -> list[dict[str, Any]]:
    if path.suffix.lower() == ".jsonl":
        return [
            json.loads(line)
            for line in path.read_text(encoding="utf-8").splitlines()
            if line.strip()
        ]
    payload = json.loads(path.read_text(encoding="utf-8"))
    if isinstance(payload, list):
        return [dict(row) for row in payload]
    if not isinstance(payload, dict) or not isinstance(payload.get("datasets"), list):
        raise ValueError("JSON input must be a row list or contain datasets[].rows[]")
    rows: list[dict[str, Any]] = []
    for dataset in payload["datasets"]:
        slug = str(dataset.get("dataset_slug") or dataset.get("slug") or "")
        name = str(dataset.get("dataset_name") or dataset.get("name") or slug)
        for source in dataset.get("rows", []):
            row = dict(source)
            row.setdefault("dataset_slug", slug)
            row.setdefault("dataset_name", name)
            rows.append(row)
    return rows


def _maps(values: Sequence[str]) -> list[tuple[str, str]]:
    result: list[tuple[str, str]] = []
    for value in values:
        if "=" not in value:
            raise ValueError(f"path map must be OLD=NEW: {value}")
        old, new = value.split("=", 1)
        if not old:
            raise ValueError("path map OLD prefix cannot be empty")
        result.append((old.rstrip("/"), new.rstrip("/")))
    return sorted(result, key=lambda item: len(item[0]), reverse=True)


def _rebase(value: str, mappings: Sequence[tuple[str, str]]) -> str:
    for old, new in mappings:
        if value == old or value.startswith(old + "/"):
            return new + value[len(old) :]
    return value


def _normal_row(source: Mapping[str, Any], mappings: Sequence[tuple[str, str]]) -> dict[str, Any]:
    slug = str(source.get("dataset_slug") or source.get("slug") or "").strip()
    asset_id = str(source.get("asset_id") or source.get("id") or "").strip()
    urdf = _rebase(str(source.get("urdf_path") or ""), mappings)
    package = _rebase(str(source.get("package_root") or Path(urdf).parent), mappings)
    if not slug or not asset_id or not urdf:
        raise ValueError(f"row requires dataset_slug, asset_id, and urdf_path: {source}")
    row = {
        "schema_version": SCHEMA_VERSION,
        "dataset_slug": slug,
        "dataset_name": str(source.get("dataset_name") or slug),
        "asset_id": asset_id,
        "urdf_path": urdf,
        "package_root": package,
    }
    for key in ("dataset_id", "category", "split"):
        if source.get(key) is not None:
            row[key] = source[key]
    return row


def _write_rows(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    text = "".join(
        json.dumps(row, ensure_ascii=True, allow_nan=False, sort_keys=True) + "\n"
        for row in rows
    )
    _atomic_text(path, text)


def _validate(rows: Sequence[Mapping[str, Any]], *, check_paths: bool) -> dict[str, Any]:
    identities: set[tuple[str, str]] = set()
    duplicate: list[str] = []
    missing_urdf: list[str] = []
    missing_root: list[str] = []
    counts: Counter[str] = Counter()
    for row in rows:
        normalized = _normal_row(row, [])
        identity = (normalized["dataset_slug"], normalized["asset_id"])
        if identity in identities:
            duplicate.append("/".join(identity))
        identities.add(identity)
        counts[identity[0]] += 1
        if check_paths:
            if not Path(normalized["urdf_path"]).is_file():
                missing_urdf.append(normalized["urdf_path"])
            if not Path(normalized["package_root"]).is_dir():
                missing_root.append(normalized["package_root"])
    return {
        "schema_version": "articulated_integrity_manifest_validation_v1",
        "row_count": len(rows),
        "dataset_counts": dict(sorted(counts.items())),
        "ordered_rows_sha256": _canonical_hash([_normal_row(row, []) for row in rows]),
        "duplicate_identity_count": len(duplicate),
        "duplicate_identity_examples": duplicate[:20],
        "missing_urdf_count": len(missing_urdf),
        "missing_urdf_examples": missing_urdf[:20],
        "missing_package_root_count": len(missing_root),
        "missing_package_root_examples": missing_root[:20],
        "valid": not duplicate and (not check_paths or (not missing_urdf and not missing_root)),
    }


def normalize(args: argparse.Namespace) -> int:
    mappings = _maps(args.path_map)
    rows = [_normal_row(row, mappings) for row in _read_rows(args.input)]
    report = _validate(rows, check_paths=args.check_paths)
    if not report["valid"]:
        print(json.dumps(report, ensure_ascii=True, indent=2, sort_keys=True))
        return 2
    _write_rows(args.out, rows)
    print(json.dumps(report, ensure_ascii=True, indent=2, sort_keys=True))
    return 0


def scan(args: argparse.Namespace) -> int:
    config = json.loads(args.config.read_text(encoding="utf-8"))
    rows: list[dict[str, Any]] = []
    for dataset in config.get("datasets", []):
        slug = str(dataset["dataset_slug"])
        name = str(dataset.get("dataset_name") or slug)
        root = Path(dataset["root"]).expanduser().resolve()
        pattern = str(dataset["urdf_glob"])
        include = re.compile(str(dataset["include_regex"])) if dataset.get("include_regex") else None
        exclude = re.compile(str(dataset["exclude_regex"])) if dataset.get("exclude_regex") else None
        package_mode = str(dataset.get("package_root_mode", "urdf_parent"))
        asset_mode = str(dataset.get("asset_id_mode", "relative_urdf"))
        for urdf in sorted(path for path in root.glob(pattern) if path.is_file()):
            relative = urdf.relative_to(root).as_posix()
            if include and not include.search(relative):
                continue
            if exclude and exclude.search(relative):
                continue
            if asset_mode == "relative_parent":
                asset_id = urdf.parent.relative_to(root).as_posix()
            elif asset_mode == "relative_urdf":
                asset_id = relative
            else:
                raise ValueError(f"unsupported asset_id_mode: {asset_mode}")
            if package_mode == "dataset_root":
                package_root = root
            elif package_mode == "urdf_parent":
                package_root = urdf.parent
            else:
                raise ValueError(f"unsupported package_root_mode: {package_mode}")
            rows.append(
                {
                    "schema_version": SCHEMA_VERSION,
                    "dataset_slug": slug,
                    "dataset_name": name,
                    "asset_id": asset_id,
                    "urdf_path": str(urdf),
                    "package_root": str(package_root),
                }
            )
    report = _validate(rows, check_paths=True)
    if not report["valid"]:
        print(json.dumps(report, ensure_ascii=True, indent=2, sort_keys=True))
        return 2
    _write_rows(args.out, rows)
    print(json.dumps(report, ensure_ascii=True, indent=2, sort_keys=True))
    return 0


def validate(args: argparse.Namespace) -> int:
    rows = _read_rows(args.input)
    report = _validate(rows, check_paths=args.check_paths)
    print(json.dumps(report, ensure_ascii=True, indent=2, sort_keys=True))
    return 0 if report["valid"] else 2


def filter_eligible(args: argparse.Namespace) -> int:
    rows = _read_rows(args.input)
    accepted: list[dict[str, Any]] = []
    rejected: list[dict[str, Any]] = []
    reason_counts: Counter[str] = Counter()
    dataset_counts: Counter[str] = Counter()
    for source in rows:
        row = _normal_row(source, [])
        raw_path = Path(row["urdf_path"]).expanduser()
        urdf_path = (raw_path if raw_path.is_absolute() else args.input.parent / raw_path).resolve()
        reason: str | None = None
        movable_count: int | None = None
        try:
            root = ET.parse(urdf_path).getroot()
            movable_count = sum(
                (joint.get("type") or "fixed").lower() != "fixed"
                for joint in root.findall("joint")
            )
            if movable_count < args.min_movable:
                reason = "movable_joint_count_lt_min"
            elif args.max_movable is not None and movable_count > args.max_movable:
                reason = "movable_joint_count_gt_max"
        except Exception as exc:
            reason = f"urdf_parse_error:{type(exc).__name__}"
        row["movable_joint_count"] = movable_count
        if reason is None:
            accepted.append(row)
            dataset_counts[row["dataset_slug"]] += 1
        else:
            reason_counts[reason] += 1
            rejected.append(
                {
                    "dataset_slug": row["dataset_slug"],
                    "asset_id": row["asset_id"],
                    "urdf_path": row["urdf_path"],
                    "movable_joint_count": movable_count,
                    "reason": reason,
                }
            )
    report = {
        "schema_version": "articulated_integrity_eligibility_report_v1",
        "input_count": len(rows),
        "eligible_count": len(accepted),
        "rejected_count": len(rejected),
        "min_movable": args.min_movable,
        "max_movable": args.max_movable,
        "eligible_dataset_counts": dict(sorted(dataset_counts.items())),
        "rejection_counts": dict(sorted(reason_counts.items())),
        "ordered_eligible_rows_sha256": _canonical_hash(accepted),
    }
    _write_rows(args.out, accepted)
    if args.rejects:
        _write_rows(args.rejects, rejected)
    report_path = args.report or args.out.with_suffix(".eligibility.json")
    _atomic_text(report_path, json.dumps(report, ensure_ascii=True, indent=2, sort_keys=True) + "\n")
    print(json.dumps(report, ensure_ascii=True, indent=2, sort_keys=True))
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    normalize_parser = subparsers.add_parser("normalize", help="convert JSON/JSONL to portable JSONL")
    normalize_parser.add_argument("--input", type=Path, required=True)
    normalize_parser.add_argument("--out", type=Path, required=True)
    normalize_parser.add_argument("--path-map", action="append", default=[], metavar="OLD=NEW")
    normalize_parser.add_argument("--check-paths", action="store_true")
    normalize_parser.set_defaults(func=normalize)

    scan_parser = subparsers.add_parser("scan", help="scan dataset roots using a JSON config")
    scan_parser.add_argument("--config", type=Path, required=True)
    scan_parser.add_argument("--out", type=Path, required=True)
    scan_parser.set_defaults(func=scan)

    validate_parser = subparsers.add_parser("validate", help="validate normalized JSONL")
    validate_parser.add_argument("--input", type=Path, required=True)
    validate_parser.add_argument("--check-paths", action="store_true")
    validate_parser.set_defaults(func=validate)

    filter_parser = subparsers.add_parser("filter-eligible", help="freeze articulated assets by movable-joint count")
    filter_parser.add_argument("--input", type=Path, required=True)
    filter_parser.add_argument("--out", type=Path, required=True)
    filter_parser.add_argument("--rejects", type=Path)
    filter_parser.add_argument("--report", type=Path)
    filter_parser.add_argument("--min-movable", type=int, default=1)
    filter_parser.add_argument("--max-movable", type=int, default=20)
    filter_parser.set_defaults(func=filter_eligible)

    args = parser.parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
