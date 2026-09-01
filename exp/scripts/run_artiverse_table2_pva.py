#!/usr/bin/env python3
"""Evaluate the Artiverse Table 2 articulation statistics on Ours / PV-A.

The paper's Table 2 is an annotation statistic, not a simulator pass rate.  A
PV-A export currently contains an URDF and physical/appearance receipts but no
semantic annotation sidecar.  This runner therefore has two deliberately
separate modes:

``structural-proxy`` (the default)
    Computes a reproducible representation-level proxy from the exported URDF:
    functional parts are renderable links, articulated parts are unique child
    links of non-fixed joints, and joints are non-fixed XML joint elements.
    The report is explicitly marked ``STRUCTURAL_PROXY``.

``semantic``
    Consumes a sidecar with final functional/articulated part instances and
    logical joint records.  This is the mode to use once PV-A has an annotation
    sidecar in the schema documented by ``artiverse_table2_pva_protocol_v1``.

Both modes write per-asset records and a deterministic Table 2-shaped summary.
No result is inferred from the existing sim-ready Table 1--4 receipts.
"""

from __future__ import annotations

import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
import csv
from dataclasses import dataclass
import hashlib
import itertools
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile
import time
from typing import Any, Iterable, Iterator, Mapping, Sequence
from xml.etree import ElementTree as ET


SCRIPT = Path(__file__).resolve()
EXP_ROOT = SCRIPT.parents[1]
DEFAULT_ROSTER = (
    EXP_ROOT / "runtime/pva_table1234_full_release_20260826/roster/roster_manifest.json"
)
DEFAULT_PROTOCOL = EXP_ROOT / "reference/artiverse_table2_pva_protocol_v1.json"
DEFAULT_OUTPUT = EXP_ROOT / "runtime/artiverse_table2_pva_full_release"
PAPER_URL = "https://arxiv.org/abs/2605.24403"
RUN_SCHEMA_VERSION = "artiverse_table2_pva_run_v1"
RECORD_SCHEMA_VERSION = "artiverse_table2_pva_asset_record_v1"
ANNOTATION_SCHEMA_VERSION = "artiverse_table2_pva_annotation_v1"
PROTOCOL_ID = "artiverse_table2_pva_v1"
EXPECTED_COLUMNS = (
    "n_obj",
    "category_total",
    "category_avg_objects",
    "functional_parts_total",
    "functional_parts_avg",
    "articulated_parts_total",
    "articulated_parts_avg",
    "joints_1dof",
    "joints_2dof",
)

# Table 2 displays 1-DoF and 2-DoF joint *counts*.  The source table has a
# commented-out 3-DoF column, so retain 3-DoF and other values in diagnostics
# instead of silently dropping them.
DOF_BY_TYPE: dict[str, int] = {
    "revolute": 1,
    "continuous": 1,
    "prismatic": 1,
    "screw": 1,
    "cylindrical": 2,
    "universal": 2,
    "planar": 3,
    "floating": 3,
    "free": 3,
    "spherical": 3,
}
TYPE_ALIASES = {
    "rotation": "revolute",
    "rotational": "revolute",
    "translation": "prismatic",
    "translational": "prismatic",
}


@dataclass(frozen=True)
class RosterInput:
    manifest_path: Path
    rows_path: Path
    manifest: dict[str, Any]
    manifest_sha256: str
    rows_sha256: str
    declared_n: int | None
    declared_categories: int | None
    declared_joints: int | None


@dataclass(frozen=True)
class AnnotationStore:
    rows: dict[str, dict[str, Any]]
    path: Path
    sha256: str
    schema_version: str


def load_protocol(path: Path) -> dict[str, Any]:
    """Load the frozen protocol and reject silently incompatible snapshots."""

    path = path.resolve(strict=True)
    value = _read_json(path)
    if not isinstance(value, dict):
        raise ValueError("protocol must be a JSON object")
    if value.get("protocol_id") != PROTOCOL_ID:
        raise ValueError(
            f"protocol id mismatch: {value.get('protocol_id')!r} != {PROTOCOL_ID!r}"
        )
    if int(value.get("protocol_version", -1)) != 1:
        raise ValueError("unsupported Artiverse Table 2 protocol version")
    paper = value.get("paper")
    if not isinstance(paper, Mapping) or paper.get("table") != "Table 2":
        raise ValueError("protocol is not bound to Artiverse Table 2")
    columns = value.get("columns")
    if tuple(columns or ()) != EXPECTED_COLUMNS:
        raise ValueError("protocol Table 2 columns do not match the runner")
    modes = value.get("modes")
    semantic = modes.get("semantic") if isinstance(modes, Mapping) else None
    if (
        not isinstance(semantic, Mapping)
        or semantic.get("sidecar_schema") != ANNOTATION_SCHEMA_VERSION
    ):
        raise ValueError("protocol semantic sidecar schema is missing or incompatible")
    return value


def validate_formal_roster_provenance(path: Path, manifest: Mapping[str, Any]) -> None:
    """Run the PV-A roster builder's canonical source/shard checks when present."""

    if manifest.get("schema_version") != "pva_table1234_full_release_roster_v1":
        return
    builder_path = SCRIPT.parent / "build_pva_full_release_roster.py"
    if not builder_path.is_file():
        raise ValueError(f"PV-A roster verifier is missing: {builder_path}")
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "_pva_roster_provenance_verifier", builder_path
    )
    if spec is None or spec.loader is None:
        raise ValueError("unable to load the PV-A roster verifier")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    try:
        spec.loader.exec_module(module)
        module.load_roster_manifest(path, verify_rows=False)
    finally:
        sys.modules.pop(spec.name, None)


def canonical_json(value: Any) -> str:
    return json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    )


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_json(path: Path, value: Any) -> None:
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def write_jsonl(path: Path, rows: Iterable[Mapping[str, Any]]) -> int:
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    count = 0
    with temporary.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(canonical_json(dict(row)) + "\n")
            count += 1
    os.replace(temporary, path)
    return count


def normalize_type(value: Any) -> str:
    raw = str(value or "").strip().lower().replace(" ", "_")
    return TYPE_ALIASES.get(raw, raw)


def dof_bucket(joint_type: Any) -> tuple[int | None, str]:
    normalized = normalize_type(joint_type)
    dof = DOF_BY_TYPE.get(normalized)
    if dof is None:
        return None, "other"
    return dof, str(dof)


def _self_hash(payload: Mapping[str, Any], field: str) -> str:
    value = dict(payload)
    value.pop(field, None)
    return hashlib.sha256(canonical_json(value).encode("utf-8")).hexdigest()


def _read_json(path: Path) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"invalid JSON {path}: {error}") from error


def _resolve_child(base: Path, raw: Any, *, field: str) -> Path:
    value = str(raw or "").strip()
    if not value:
        raise ValueError(f"{field} is empty")
    candidate = Path(value)
    if not candidate.is_absolute():
        candidate = base / candidate
    return candidate.resolve(strict=True)


def load_roster(path: Path, *, limit: int | None = None) -> RosterInput:
    """Load and hash a frozen roster manifest, or a direct JSONL fixture."""

    path = path.resolve(strict=True)
    if path.suffix.lower() == ".jsonl":
        rows_sha = sha256_file(path)
        manifest = {
            "schema_version": "direct_jsonl_fixture_v1",
            "dataset": "Ours / PV-A",
            "N_release": None,
            "N_eval": None,
            "release_category_count": None,
            "J_eval": None,
        }
        return RosterInput(
            path, path, manifest, sha256_file(path), rows_sha, None, None, None
        )

    manifest = _read_json(path)
    if not isinstance(manifest, dict):
        raise ValueError("roster manifest must be a JSON object")
    schema_version = str(manifest.get("schema_version", "")).strip()
    if schema_version == "pva_table1234_full_release_roster_v1":
        if manifest.get("dataset") != "Ours / PV-A":
            raise ValueError("PV-A roster dataset identity mismatch")
        if int(manifest.get("N_eval", -1)) <= 0:
            raise ValueError("PV-A roster N_eval is missing or invalid")
        if int(manifest.get("release_category_count", -1)) <= 0:
            raise ValueError("PV-A roster category denominator is missing or invalid")
    declared_hash = manifest.get("manifest_content_sha256")
    if declared_hash is not None and declared_hash != _self_hash(
        manifest, "manifest_content_sha256"
    ):
        raise ValueError("roster manifest self-hash mismatch")
    # A limited smoke run still checks the ordered-roster hash above, while the
    # canonical shard/source audit is reserved for a full-release run.
    if limit is None:
        validate_formal_roster_provenance(path, manifest)
    roster = manifest.get("roster")
    if not isinstance(roster, Mapping) or not isinstance(roster.get("path"), str):
        raise ValueError("roster manifest has no ordered roster path")
    rows_path = _resolve_child(path.parent, roster["path"], field="roster.path")
    rows_sha = sha256_file(rows_path)
    if roster.get("sha256") is not None and rows_sha != roster.get("sha256"):
        raise ValueError("ordered PV-A roster hash mismatch")
    if roster.get("bytes") is not None and rows_path.stat().st_size != int(
        roster["bytes"]
    ):
        raise ValueError("ordered PV-A roster byte count mismatch")
    declared_n = int(manifest["N_eval"]) if manifest.get("N_eval") is not None else None
    declared_rows = (
        int(roster["row_count"]) if roster.get("row_count") is not None else None
    )
    if (
        declared_n is not None
        and declared_rows is not None
        and declared_n != declared_rows
    ):
        raise ValueError("roster N_eval and row_count disagree")
    if limit is not None and (
        limit <= 0
        or (declared_n is not None and limit > declared_n)
        or (declared_rows is not None and limit > declared_rows)
    ):
        raise ValueError(
            f"limit must be in [1, {declared_n or declared_rows or 'roster size'}]"
        )
    return RosterInput(
        path,
        rows_path,
        manifest,
        sha256_file(path),
        rows_sha,
        declared_n,
        int(manifest["release_category_count"])
        if manifest.get("release_category_count") is not None
        else None,
        int(manifest["J_eval"]) if manifest.get("J_eval") is not None else None,
    )


def iter_roster_rows(
    roster: RosterInput, *, limit: int | None = None
) -> Iterator[dict[str, Any]]:
    with roster.rows_path.open("r", encoding="utf-8") as handle:
        for ordinal, line in enumerate(handle):
            if limit is not None and ordinal >= limit:
                break
            try:
                value = json.loads(line)
            except json.JSONDecodeError as error:
                raise ValueError(
                    f"invalid roster JSON at line {ordinal + 1}"
                ) from error
            if not isinstance(value, dict):
                raise ValueError(f"roster row {ordinal} is not an object")
            observed_ordinal = value.get("ordinal", ordinal)
            if int(observed_ordinal) != ordinal:
                raise ValueError(
                    f"roster ordinal mismatch: {observed_ordinal} != {ordinal}"
                )
            asset_id = str(value.get("asset_id", "")).strip()
            category = str(value.get("raw_category", value.get("category", ""))).strip()
            if not asset_id or not category:
                raise ValueError(f"roster row {ordinal} has empty asset_id/category")
            value["ordinal"] = ordinal
            value["asset_id"] = asset_id
            value["category"] = category
            yield value


def _annotation_rows_from_json(
    value: Any, path: Path
) -> tuple[str, Iterable[Mapping[str, Any]]]:
    if not isinstance(value, Mapping):
        raise ValueError(f"annotation file must contain an object: {path}")
    schema = str(value.get("schema_version", "")).strip()
    assets = value.get("assets")
    if isinstance(assets, Mapping):
        rows = []
        for asset_id, row in assets.items():
            if not isinstance(row, Mapping):
                raise ValueError(f"annotation asset {asset_id!r} is not an object")
            embedded_id = row.get("asset_id")
            if embedded_id is not None and str(embedded_id).strip() != str(asset_id):
                raise ValueError(
                    f"annotation map key does not match embedded asset_id: {asset_id!r}"
                )
            normalized = dict(row)
            normalized["asset_id"] = asset_id
            rows.append(normalized)
        return schema, rows
    if isinstance(assets, list):
        return schema, assets
    # A direct one-asset sidecar is convenient for package-local annotations.
    if value.get("asset_id") is not None:
        return schema, [value]
    raise ValueError("annotation JSON requires an assets list/map or asset_id")


def load_annotations(
    path: Path, *, expected_schema: str = ANNOTATION_SCHEMA_VERSION
) -> AnnotationStore:
    path = path.resolve(strict=True)
    rows: Iterable[Mapping[str, Any]]
    if path.suffix.lower() in {".jsonl", ".ndjson"}:
        parsed: list[Mapping[str, Any]] = []
        schema = ""
        with path.open("r", encoding="utf-8") as handle:
            for line_number, line in enumerate(handle, start=1):
                if not line.strip():
                    continue
                try:
                    row = json.loads(line)
                except json.JSONDecodeError as error:
                    raise ValueError(
                        f"invalid annotation JSON at line {line_number}"
                    ) from error
                if not isinstance(row, Mapping):
                    raise ValueError(f"annotation row {line_number} is not an object")
                row_schema = str(row.get("schema_version", "")).strip()
                if row_schema:
                    if schema and row_schema != schema:
                        raise ValueError(
                            f"annotation schema changes at line {line_number}"
                        )
                    schema = schema or row_schema
                parsed.append(row)
        rows = parsed
    else:
        schema, rows = _annotation_rows_from_json(_read_json(path), path)
    if not schema:
        raise ValueError("annotation sidecar is missing schema_version")
    if expected_schema and schema != expected_schema:
        raise ValueError(
            f"annotation schema mismatch: {schema!r} != {expected_schema!r}"
        )
    by_id: dict[str, dict[str, Any]] = {}
    for raw in rows:
        if not isinstance(raw, Mapping):
            raise ValueError("annotation row is not an object")
        row_schema = str(raw.get("schema_version", "")).strip()
        if row_schema and row_schema != schema:
            raise ValueError("annotation row schema does not match file schema")
        asset_id = str(raw.get("asset_id", raw.get("id", ""))).strip()
        if not asset_id:
            raise ValueError("annotation row is missing asset_id")
        if asset_id in by_id:
            raise ValueError(f"duplicate annotation asset_id: {asset_id}")
        by_id[asset_id] = dict(raw)
    return AnnotationStore(by_id, path, sha256_file(path), schema)


def _part_count(value: Any, *, field: str) -> tuple[int, list[str]]:
    if isinstance(value, bool):
        raise ValueError(f"{field} must not be boolean")
    if isinstance(value, int):
        if value < 0:
            raise ValueError(f"{field} must be non-negative")
        return value, []
    if not isinstance(value, list):
        raise ValueError(f"{field} must be a list or integer count")
    identifiers: list[str] = []
    for index, item in enumerate(value):
        if isinstance(item, Mapping):
            identifier = item.get("id", item.get("pid", item.get("name", index)))
        else:
            identifier = item
        identifiers.append(str(identifier))
    if len(set(identifiers)) != len(identifiers):
        raise ValueError(f"{field} contains duplicate part identifiers")
    return len(value), identifiers


def _joint_type_from_group(items: Sequence[Mapping[str, Any]]) -> str | None:
    explicit = [
        normalize_type(item.get("type", item.get("joint_type"))) for item in items
    ]
    explicit = [item for item in explicit if item]
    if not explicit:
        return "other"
    # Artiverse uses a `free` record as a base/placeholder in some pid groups.
    # Ignore it whenever a concrete motion record is present; otherwise a real
    # free joint remains visible as a 3-DoF diagnostic. Fixed-only groups are
    # not logical motion joints and are dropped.
    motion = [item for item in explicit if item not in {"fixed", "free"}]
    if not motion:
        return "free" if "free" in explicit else None
    if any(item in {"cylindrical", "universal", "screw"} for item in motion):
        for candidate in ("universal", "cylindrical", "screw"):
            if candidate in motion:
                return candidate
    if "revolute" in motion and "prismatic" in motion:
        return "cylindrical"
    if sum(item in {"revolute", "continuous"} for item in motion) >= 2:
        return "universal"
    return motion[0]


def semantic_joint_types(annotation: Mapping[str, Any]) -> list[str]:
    raw = annotation.get("joints", annotation.get("motion_joints"))
    if not isinstance(raw, list):
        raise ValueError("semantic annotation requires a joints list")
    groups: dict[str, list[Mapping[str, Any]]] = {}
    ordered: list[tuple[str, Mapping[str, Any]]] = []
    for index, item in enumerate(raw):
        if not isinstance(item, Mapping):
            raise ValueError("joint records must be objects")
        # A repeated pid/id denotes the components of one logical joint.  A
        # missing id means each record is its own logical joint.
        raw_identifier = item.get("pid", item.get("id"))
        identifier = str(raw_identifier).strip() if raw_identifier is not None else ""
        if not identifier:
            identifier = f"__row_{index}"
        if identifier not in groups:
            ordered.append((identifier, item))
            groups[identifier] = []
        groups[identifier].append(item)
    return [
        joint_type
        for key, _ in ordered
        if (joint_type := _joint_type_from_group(groups[key])) is not None
    ]


def _resolve_row_path(row: Mapping[str, Any], key: str, *, roster_base: Path) -> Path:
    raw = row.get(key)
    if raw is None and key == "primary_urdf_path":
        raw = row.get("urdf_path")
    if raw is None:
        source = row.get("source_path") or row.get("package_root")
        relative = row.get("primary_urdf_relative_path", "model.urdf")
        if source is None:
            raise ValueError(f"roster row has no {key}")
        raw = str(Path(str(source)) / str(relative))
    return _resolve_child(roster_base, raw, field=key)


def _valid_visual_link(link: ET.Element) -> bool:
    # Part annotations are attached to semantic instances with geometry.  A
    # link carrying at least one visual is the conservative export proxy.
    return any(child.tag.rsplit("}", 1)[-1] == "visual" for child in link)


def evaluate_xml(
    row: Mapping[str, Any],
    *,
    roster_base: Path,
    annotations: AnnotationStore | None,
    mode: str,
    verify_hash: bool,
) -> dict[str, Any]:
    ordinal = int(row["ordinal"])
    asset_id = str(row["asset_id"])
    category = str(row["category"])
    record: dict[str, Any] = {
        "schema_version": RECORD_SCHEMA_VERSION,
        "ordinal": ordinal,
        "asset_id": asset_id,
        "category": category,
        "mode": mode,
        "status": "FAILED",
        "urdf_path": None,
        "expected_urdf_sha256": row.get("primary_urdf_sha256"),
        "observed_urdf_sha256": None,
        "link_count": None,
        "renderable_link_count": None,
        "fixed_joint_count": None,
        "representation_movable_joint_count": None,
        "functional_part_count": None,
        "articulated_part_count": None,
        "logical_joint_count": None,
        "joint_1dof_count": 0,
        "joint_2dof_count": 0,
        "joint_3dof_count": 0,
        "joint_other_count": 0,
        "joint_type_counts": {},
        "annotation_raw_joint_type_counts": {},
        "semantic_representation_joint_count_delta": None,
        "functional_part_source": None,
        "articulated_part_source": None,
        "joint_source": None,
        "roster_joint_count": row.get("joint_count"),
        "roster_joint_count_match": None,
        "error": None,
    }
    try:
        urdf = _resolve_row_path(row, "primary_urdf_path", roster_base=roster_base)
        record["urdf_path"] = str(urdf)
        payload = urdf.read_bytes()
        observed_hash = hashlib.sha256(payload).hexdigest()
        record["observed_urdf_sha256"] = observed_hash
        expected_hash = row.get("primary_urdf_sha256")
        if verify_hash and expected_hash and observed_hash != expected_hash:
            raise ValueError("primary URDF hash drift")
        root = ET.fromstring(payload)
        if root.tag.rsplit("}", 1)[-1] != "robot":
            raise ValueError(f"expected robot root, got {root.tag!r}")
        links = [child for child in root if child.tag.rsplit("}", 1)[-1] == "link"]
        joints = [child for child in root if child.tag.rsplit("}", 1)[-1] == "joint"]
        names = [str(link.get("name", "")).strip() for link in links]
        if (
            not names
            or any(not name for name in names)
            or len(set(names)) != len(names)
        ):
            raise ValueError("URDF link names are empty or non-unique")
        link_set = set(names)
        renderable = sum(_valid_visual_link(link) for link in links)
        fixed = 0
        movable: list[tuple[str, str, str, str]] = []
        type_counts: Counter[str] = Counter()
        for joint in joints:
            joint_type = normalize_type(joint.get("type"))
            parent_node = next(
                (child for child in joint if child.tag.rsplit("}", 1)[-1] == "parent"),
                None,
            )
            child_node = next(
                (child for child in joint if child.tag.rsplit("}", 1)[-1] == "child"),
                None,
            )
            parent = (
                str(parent_node.get("link", "")).strip()
                if parent_node is not None
                else ""
            )
            child = (
                str(child_node.get("link", "")).strip()
                if child_node is not None
                else ""
            )
            if parent not in link_set or child not in link_set:
                raise ValueError(
                    f"joint endpoint is not a declared link: {joint.get('name')!r}"
                )
            if joint_type == "fixed":
                fixed += 1
                continue
            type_counts[joint_type] += 1
            movable.append((str(joint.get("name", "")), joint_type, parent, child))
        record["link_count"] = len(links)
        record["renderable_link_count"] = renderable
        record["fixed_joint_count"] = fixed
        record["representation_movable_joint_count"] = len(movable)
        record["joint_type_counts"] = dict(sorted(type_counts.items()))
        record["roster_joint_count_match"] = row.get("joint_count") is None or int(
            row["joint_count"]
        ) == len(movable)

        if mode == "structural-proxy":
            record["functional_part_count"] = renderable
            record["articulated_part_count"] = len(
                {child for _, _, _, child in movable}
            )
            record["logical_joint_count"] = len(movable)
            record["semantic_representation_joint_count_delta"] = 0
            record["functional_part_source"] = "renderable_urdf_links"
            record["articulated_part_source"] = (
                "unique_child_links_of_nonfixed_xml_joints"
            )
            record["joint_source"] = "nonfixed_xml_joint_elements"
        else:
            if annotations is None:
                raise ValueError("semantic mode requires --annotations")
            annotation = annotations.rows.get(asset_id)
            if annotation is None:
                raise ValueError("semantic annotation missing for asset")
            functional_value = annotation.get(
                "functional_parts", annotation.get("func_parts")
            )
            articulated_value = annotation.get(
                "articulated_parts", annotation.get("arti_parts")
            )
            if functional_value is None or articulated_value is None:
                raise ValueError(
                    "semantic annotation requires functional_parts and articulated_parts"
                )
            functional_count, _ = _part_count(
                functional_value, field="functional_parts"
            )
            articulated_count, _ = _part_count(
                articulated_value, field="articulated_parts"
            )
            joint_types = semantic_joint_types(annotation)
            raw_annotation_joints = annotation.get(
                "joints", annotation.get("motion_joints", [])
            )
            record["annotation_raw_joint_type_counts"] = dict(
                sorted(
                    Counter(
                        normalize_type(item.get("type", item.get("joint_type")))
                        for item in raw_annotation_joints
                        if isinstance(item, Mapping)
                    ).items()
                )
            )
            record["functional_part_count"] = functional_count
            record["articulated_part_count"] = articulated_count
            record["logical_joint_count"] = len(joint_types)
            record["functional_part_source"] = "semantic_annotation"
            record["articulated_part_source"] = "semantic_annotation"
            record["joint_source"] = "semantic_annotation"
            record["semantic_representation_joint_count_delta"] = len(
                joint_types
            ) - len(movable)
            type_counts = Counter(normalize_type(item) for item in joint_types)
            record["joint_type_counts"] = dict(sorted(type_counts.items()))

        for joint_type, count in type_counts.items():
            _, bucket = dof_bucket(joint_type)
            record[
                f"joint_{bucket}dof_count"
                if bucket in {"1", "2", "3"}
                else "joint_other_count"
            ] += count
        # Keep the spelling stable for machine consumers.
        record["joint_1dof_count"] = int(record.get("joint_1dof_count", 0))
        record["joint_2dof_count"] = int(record.get("joint_2dof_count", 0))
        record["joint_3dof_count"] = int(record.get("joint_3dof_count", 0))
        record["joint_other_count"] = int(record.get("joint_other_count", 0))
        record["status"] = "EVALUATED"
    except Exception as error:  # noqa: BLE001 - preserve a per-asset failure record
        record["error"] = f"{type(error).__name__}: {error}"
    return record


def _empty_aggregate() -> dict[str, Any]:
    return {
        "n_eval": 0,
        "evaluated_assets": 0,
        "failed_assets": 0,
        "categories": Counter(),
        "evaluated_categories": Counter(),
        "functional_parts_total": 0,
        "articulated_parts_total": 0,
        "logical_joints_total": 0,
        "joint_1dof_total": 0,
        "joint_2dof_total": 0,
        "joint_3dof_total": 0,
        "joint_other_total": 0,
        "representation_movable_joint_total": 0,
        "semantic_representation_joint_count_delta_total": 0,
        "joint_type_counts": Counter(),
        "annotation_raw_joint_type_counts": Counter(),
        "roster_joint_count_mismatches": 0,
    }


def update_aggregate(aggregate: dict[str, Any], record: Mapping[str, Any]) -> None:
    aggregate["n_eval"] += 1
    aggregate["categories"][str(record["category"])] += 1
    if record.get("status") != "EVALUATED":
        aggregate["failed_assets"] += 1
        return
    aggregate["evaluated_assets"] += 1
    aggregate["evaluated_categories"][str(record["category"])] += 1
    record_fields = {
        "functional_parts_total": "functional_part_count",
        "articulated_parts_total": "articulated_part_count",
        "logical_joints_total": "logical_joint_count",
        "joint_1dof_total": "joint_1dof_count",
        "joint_2dof_total": "joint_2dof_count",
        "joint_3dof_total": "joint_3dof_count",
        "joint_other_total": "joint_other_count",
        "representation_movable_joint_total": "representation_movable_joint_count",
        "semantic_representation_joint_count_delta_total": (
            "semantic_representation_joint_count_delta"
        ),
    }
    for name, field in record_fields.items():
        value = record.get(field)
        if value is not None:
            aggregate[name] += int(value)
    for joint_type, count in (record.get("joint_type_counts") or {}).items():
        aggregate["joint_type_counts"][str(joint_type)] += int(count)
    for joint_type, count in (
        record.get("annotation_raw_joint_type_counts") or {}
    ).items():
        aggregate["annotation_raw_joint_type_counts"][str(joint_type)] += int(count)
    if record.get("roster_joint_count_match") is False:
        aggregate["roster_joint_count_mismatches"] += 1


def _round(value: float | None, digits: int = 1) -> float | None:
    return None if value is None else round(value, digits)


def make_summary(
    aggregate: dict[str, Any],
    *,
    mode: str,
    roster: RosterInput,
    annotation_store: AnnotationStore | None,
    limit: int | None,
    started_at: float,
    elapsed_s: float,
    observed_row_count: int | None = None,
    observed_category_count: int | None = None,
    observed_declared_joint_sum: int | None = None,
    roster_complete: bool | None = None,
    annotation_missing_assets: int | None = None,
    annotation_extra_assets: int | None = None,
) -> dict[str, Any]:
    n = int(aggregate["n_eval"])
    evaluated = int(aggregate["evaluated_assets"])
    categories = aggregate["categories"]
    denominator_complete = evaluated == n and n > 0
    # Preserve the paper's denominator for complete runs.  For incomplete runs
    # averages are left null rather than quietly changing the denominator.
    avg_denominator = n if denominator_complete else None
    table = {
        "dataset": "Ours / PV-A",
        "mode": "SEMANTIC" if mode == "semantic" else "STRUCTURAL_PROXY",
        "n_obj": n,
        "category_total": len(categories),
        "category_avg_objects": _round(
            n / len(categories) if denominator_complete and categories else None
        ),
        "functional_parts_total": aggregate["functional_parts_total"],
        "functional_parts_avg": _round(
            aggregate["functional_parts_total"] / avg_denominator
            if avg_denominator
            else None
        ),
        "articulated_parts_total": aggregate["articulated_parts_total"],
        "articulated_parts_avg": _round(
            aggregate["articulated_parts_total"] / avg_denominator
            if avg_denominator
            else None
        ),
        "joints_1dof": aggregate["joint_1dof_total"],
        "joints_2dof": aggregate["joint_2dof_total"],
    }
    diagnostics = {
        "average_denominator": avg_denominator,
        "average_denominator_policy": (
            "selected asset count when every selected asset evaluates; otherwise null"
        ),
        "logical_joints_total": aggregate["logical_joints_total"],
        "logical_joints_avg": _round(
            aggregate["logical_joints_total"] / avg_denominator
            if avg_denominator
            else None
        ),
        "joints_3dof": aggregate["joint_3dof_total"],
        "joints_other": aggregate["joint_other_total"],
        "representation_movable_joint_total": aggregate[
            "representation_movable_joint_total"
        ],
        "representation_movable_joint_avg": _round(
            aggregate["representation_movable_joint_total"] / avg_denominator
            if avg_denominator
            else None
        ),
        "semantic_representation_joint_count_delta_total": aggregate[
            "semantic_representation_joint_count_delta_total"
        ],
        "joint_type_counts": dict(sorted(aggregate["joint_type_counts"].items())),
        "annotation_raw_joint_type_counts": dict(
            sorted(aggregate["annotation_raw_joint_type_counts"].items())
        ),
        "roster_joint_count_mismatches": aggregate["roster_joint_count_mismatches"],
    }
    summary: dict[str, Any] = {
        "schema_version": RUN_SCHEMA_VERSION,
        "dataset": "Ours / PV-A",
        "classification": (
            "SMOKE"
            if limit is not None
            else "SEMANTIC_ANNOTATION"
            if mode == "semantic"
            else "STRUCTURAL_PROXY"
        ),
        "mode": mode,
        "paper": {"url": PAPER_URL, "table": "Table 2", "page": 6},
        "n_eval": n,
        "evaluated_assets": evaluated,
        "failed_assets": aggregate["failed_assets"],
        "metric_coverage": evaluated / n if n else 0.0,
        "category_count": len(categories),
        "category_counts": dict(sorted(categories.items())),
        "table2": table,
        "diagnostics": diagnostics,
        "input": {
            "roster_manifest": str(roster.manifest_path),
            "roster_manifest_sha256": roster.manifest_sha256,
            "roster_jsonl": str(roster.rows_path),
            "roster_jsonl_sha256": roster.rows_sha256,
            "declared_N_eval": roster.declared_n,
            "declared_category_count": roster.declared_categories,
            "declared_J_eval": roster.declared_joints,
            "annotation_sidecar": str(annotation_store.path)
            if annotation_store
            else None,
            "annotation_sidecar_sha256": annotation_store.sha256
            if annotation_store
            else None,
            "annotation_schema_version": annotation_store.schema_version
            if annotation_store
            else None,
            "observed_roster_row_count": observed_row_count,
            "observed_roster_category_count": observed_category_count,
            "observed_roster_declared_joint_sum": observed_declared_joint_sum,
            "roster_complete": roster_complete,
            "annotation_missing_assets": annotation_missing_assets,
            "annotation_extra_assets": annotation_extra_assets,
        },
        "timing": {"started_epoch": started_at, "elapsed_seconds": elapsed_s},
    }
    summary["summary_content_sha256"] = _self_hash(summary, "summary_content_sha256")
    return summary


def markdown_report(
    summary: Mapping[str, Any], *, runner_sha256: str, protocol_sha256: str
) -> str:
    table = summary["table2"]
    mode = str(summary["mode"])

    def display(value: Any) -> str:
        return (
            "N/A"
            if value is None
            else f"{value:,}"
            if isinstance(value, int)
            else str(value)
        )

    lines = [
        "# Ours / PV-A Artiverse Table 2",
        "",
        f"- Classification: **{summary['classification']}**",
        f"- Mode: **{mode}**",
        f"- Paper reference: [Artiverse Table 2]({PAPER_URL}), page 6",
        f"- Evaluated assets: **{summary['evaluated_assets']:,} / {summary['n_eval']:,}**",
        f"- Runner SHA-256: `{runner_sha256}`",
        f"- Protocol SHA-256: `{protocol_sha256}`",
        "",
        "## Table 2-shaped output",
        "",
        "| Dataset | # obj | Category total | Avg # obj | # Func. Parts total | Avg | # Arti. Parts total | Avg | # Joints 1-DoF | # Joints 2-DoF |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
        "| Ours / PV-A | "
        + " | ".join(
            display(table[key])
            for key in (
                "n_obj",
                "category_total",
                "category_avg_objects",
                "functional_parts_total",
                "functional_parts_avg",
                "articulated_parts_total",
                "articulated_parts_avg",
                "joints_1dof",
                "joints_2dof",
            )
        )
        + " |",
        "",
        "## Scope and diagnostics",
        "",
    ]
    if mode == "structural-proxy":
        lines.extend(
            [
                "This row is a **STRUCTURAL_PROXY**, not a semantic annotation result.",
                "- Functional parts: URDF links with at least one visual element.",
                "- Articulated parts: unique child links of non-fixed XML joints.",
                "- Joints: non-fixed XML joint elements; XML types are mapped to DoF buckets.",
                "- A paper-comparable semantic row requires a complete `--annotations` sidecar and `--mode semantic`.",
            ]
        )
    else:
        lines.append("This row uses the supplied semantic annotation sidecar.")
    diagnostics = summary["diagnostics"]
    lines.extend(
        [
            "",
            f"- Logical joints (all DoF): {display(diagnostics['logical_joints_total'])}; 3-DoF: {display(diagnostics['joints_3dof'])}; other: {display(diagnostics['joints_other'])}.",
            f"- Representation movable XML joints: {display(diagnostics['representation_movable_joint_total'])}.",
            f"- Semantic minus representation logical-joint count: {display(diagnostics['semantic_representation_joint_count_delta_total'])}.",
            f"- Roster joint-count mismatches: {display(diagnostics['roster_joint_count_mismatches'])}.",
            f"- Average denominator: {display(diagnostics['average_denominator'])} (null when any selected asset fails).",
            "",
        ]
    )
    inputs = summary.get("input", {})
    if inputs.get("annotation_sidecar"):
        lines.insert(
            -1,
            f"- Annotation coverage: missing {display(inputs.get('annotation_missing_assets'))}, extra {display(inputs.get('annotation_extra_assets'))}.",
        )
    return "\n".join(lines)


def csv_row(summary: Mapping[str, Any]) -> dict[str, Any]:
    table = summary["table2"]
    return {
        "dataset": table["dataset"],
        "mode": table["mode"],
        "n_obj": table["n_obj"],
        "category_total": table["category_total"],
        "category_avg_objects": table["category_avg_objects"],
        "functional_parts_total": table["functional_parts_total"],
        "functional_parts_avg": table["functional_parts_avg"],
        "articulated_parts_total": table["articulated_parts_total"],
        "articulated_parts_avg": table["articulated_parts_avg"],
        "joints_1dof": table["joints_1dof"],
        "joints_2dof": table["joints_2dof"],
    }


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--roster", type=Path, default=DEFAULT_ROSTER)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--mode",
        choices=("structural-proxy", "semantic"),
        default="structural-proxy",
        help="semantic requires --annotations; proxy is runnable on the current PV-A release",
    )
    parser.add_argument("--annotations", type=Path)
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument(
        "--limit", type=int, help="evaluate only the first N roster rows"
    )
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument(
        "--no-verify-input-hashes",
        action="store_true",
        help="skip per-asset primary URDF hash checks (not recommended for formal runs)",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="return a non-zero status when any asset fails or a roster count mismatches",
    )
    return parser.parse_args(argv)


def run(args: argparse.Namespace) -> dict[str, Any]:
    if args.workers < 1:
        raise ValueError("--workers must be positive")
    if args.limit is not None and args.limit <= 0:
        raise ValueError("--limit must be positive")
    if args.mode == "semantic" and args.annotations is None:
        raise ValueError("--mode semantic requires --annotations")
    protocol = args.protocol.resolve(strict=True)
    protocol_value = load_protocol(protocol)
    roster = load_roster(args.roster, limit=args.limit)
    annotation_store = load_annotations(args.annotations) if args.annotations else None
    if args.mode == "semantic" and annotation_store is None:
        raise ValueError("semantic mode requires a loaded annotation sidecar")
    output = args.output.resolve()
    if output.exists():
        raise FileExistsError(f"output already exists: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(
        tempfile.mkdtemp(prefix=f".{output.name}.staging.", dir=output.parent)
    )
    started_epoch = time.time()
    started_monotonic = time.perf_counter()
    runner_hash = sha256_file(SCRIPT)
    protocol_hash = sha256_file(protocol)
    try:
        rows = iter_roster_rows(roster, limit=args.limit)
        aggregate = _empty_aggregate()
        selected_asset_ids: set[str] = set()
        selected_declared_joint_sum = 0
        selected_declared_joint_rows = 0
        records_path = staging / "asset_records.jsonl"

        def evaluate_one(row: Mapping[str, Any]) -> dict[str, Any]:
            return evaluate_xml(
                row,
                roster_base=roster.manifest_path.parent,
                annotations=annotation_store,
                mode=args.mode,
                verify_hash=not args.no_verify_input_hashes,
            )

        # Keep only a small batch of futures in memory.  This matters for the
        # 302,440-row release and preserves roster order in the JSONL evidence.
        batch_size = max(32, args.workers * 4)
        executor = (
            ThreadPoolExecutor(max_workers=args.workers) if args.workers > 1 else None
        )
        try:
            with records_path.open("w", encoding="utf-8") as records_handle:
                while True:
                    batch = list(itertools.islice(rows, batch_size))
                    if not batch:
                        break
                    for row in batch:
                        asset_id = str(row["asset_id"])
                        if asset_id in selected_asset_ids:
                            raise ValueError(f"duplicate roster asset_id: {asset_id}")
                        selected_asset_ids.add(asset_id)
                        if row.get("joint_count") is not None:
                            selected_declared_joint_sum += int(row["joint_count"])
                            selected_declared_joint_rows += 1
                    if executor is None:
                        evaluated = [evaluate_one(row) for row in batch]
                    else:
                        futures = [executor.submit(evaluate_one, row) for row in batch]
                        evaluated = []
                        for row, future in zip(batch, futures):
                            try:
                                evaluated.append(future.result())
                            except (
                                Exception
                            ) as error:  # pragma: no cover - defensive worker fence
                                evaluated.append(
                                    {
                                        "schema_version": RECORD_SCHEMA_VERSION,
                                        "ordinal": int(row["ordinal"]),
                                        "asset_id": str(row["asset_id"]),
                                        "category": str(row["category"]),
                                        "mode": args.mode,
                                        "status": "FAILED",
                                        "error": f"worker_exception: {type(error).__name__}: {error}",
                                        "functional_part_count": None,
                                        "articulated_part_count": None,
                                        "logical_joint_count": None,
                                        "joint_1dof_count": 0,
                                        "joint_2dof_count": 0,
                                        "joint_3dof_count": 0,
                                        "joint_other_count": 0,
                                        "joint_type_counts": {},
                                        "annotation_raw_joint_type_counts": {},
                                        "semantic_representation_joint_count_delta": None,
                                    }
                                )
                    for record in evaluated:
                        update_aggregate(aggregate, record)
                        records_handle.write(canonical_json(record) + "\n")
        finally:
            if executor is not None:
                executor.shutdown(wait=True)
        observed_rows = int(aggregate["n_eval"])
        observed_categories = len(aggregate["categories"])
        declared_rows = roster.manifest.get("roster", {}).get("row_count")
        expected_rows = args.limit
        if expected_rows is None:
            expected_rows = roster.declared_n
        if expected_rows is None and declared_rows is not None:
            expected_rows = int(declared_rows)
        rows_complete = expected_rows is None or observed_rows == expected_rows
        if not rows_complete:
            raise ValueError(
                f"roster row count mismatch: evaluated {observed_rows}, expected {expected_rows}"
            )
        if args.limit is None and roster.declared_categories is not None:
            if observed_categories != roster.declared_categories:
                raise ValueError(
                    "roster category count mismatch: "
                    f"evaluated {observed_categories}, expected {roster.declared_categories}"
                )
        if (
            args.limit is None
            and roster.declared_joints is not None
            and selected_declared_joint_rows == observed_rows
            and selected_declared_joint_sum != roster.declared_joints
        ):
            raise ValueError(
                "roster joint denominator mismatch: "
                f"observed {selected_declared_joint_sum}, expected {roster.declared_joints}"
            )
        annotation_missing = 0
        annotation_extra = 0
        if annotation_store is not None:
            annotation_missing = len(
                selected_asset_ids.difference(annotation_store.rows)
            )
            annotation_extra = len(
                set(annotation_store.rows).difference(selected_asset_ids)
            )
        elapsed = time.perf_counter() - started_monotonic
        summary = make_summary(
            aggregate,
            mode=args.mode,
            roster=roster,
            annotation_store=annotation_store,
            limit=args.limit,
            started_at=started_epoch,
            elapsed_s=elapsed,
            observed_row_count=observed_rows,
            observed_category_count=observed_categories,
            observed_declared_joint_sum=(
                selected_declared_joint_sum if selected_declared_joint_rows else None
            ),
            roster_complete=rows_complete,
            annotation_missing_assets=annotation_missing
            if annotation_store is not None
            else None,
            annotation_extra_assets=annotation_extra
            if annotation_store is not None
            else None,
        )
        summary["protocol"] = {
            "id": protocol_value["protocol_id"],
            "path": str(protocol),
            "sha256": protocol_hash,
        }
        summary["summary_content_sha256"] = _self_hash(
            summary, "summary_content_sha256"
        )
        strict_failures = []
        if summary["failed_assets"]:
            strict_failures.append(f"asset_failures={summary['failed_assets']}")
        if summary["diagnostics"]["roster_joint_count_mismatches"]:
            strict_failures.append(
                "roster_joint_count_mismatches="
                f"{summary['diagnostics']['roster_joint_count_mismatches']}"
            )
        if annotation_store is not None and annotation_extra:
            strict_failures.append(f"annotation_extra_assets={annotation_extra}")
        if annotation_store is not None and annotation_missing:
            strict_failures.append(f"annotation_missing_assets={annotation_missing}")
        summary["strict"] = {
            "requested": bool(args.strict),
            "passed": not strict_failures,
            "failures": strict_failures,
        }
        summary["summary_content_sha256"] = _self_hash(
            summary, "summary_content_sha256"
        )
        manifest = {
            "schema_version": RUN_SCHEMA_VERSION,
            "dataset": "Ours / PV-A",
            "classification": summary["classification"],
            "mode": args.mode,
            "paper_url": PAPER_URL,
            "protocol_id": protocol_value["protocol_id"],
            "protocol": str(protocol),
            "protocol_sha256": protocol_hash,
            "runner": str(SCRIPT),
            "runner_sha256": runner_hash,
            "roster_manifest": str(roster.manifest_path),
            "roster_manifest_sha256": roster.manifest_sha256,
            "roster_jsonl": str(roster.rows_path),
            "roster_jsonl_sha256": roster.rows_sha256,
            "annotation_sidecar": str(annotation_store.path)
            if annotation_store
            else None,
            "annotation_sidecar_sha256": annotation_store.sha256
            if annotation_store
            else None,
            "limit": args.limit,
            "workers": args.workers,
            "verify_input_hashes": not args.no_verify_input_hashes,
            "asset_records": "asset_records.jsonl",
            "summary": "summary.json",
            "started_epoch": started_epoch,
            "elapsed_seconds": elapsed,
        }
        manifest["strict"] = summary["strict"]
        write_json(staging / "summary.json", summary)
        (staging / "table2.md").write_text(
            markdown_report(
                summary, runner_sha256=runner_hash, protocol_sha256=protocol_hash
            ),
            encoding="utf-8",
        )
        with (staging / "table2.csv").open("w", newline="", encoding="utf-8") as handle:
            row = csv_row(summary)
            writer = csv.DictWriter(handle, fieldnames=list(row))
            writer.writeheader()
            writer.writerow(row)
        # Bind all generated evidence except the manifest itself.  This makes a
        # copied report auditable without requiring a second verifier script.
        artifact_paths = (
            staging / "asset_records.jsonl",
            staging / "summary.json",
            staging / "table2.md",
            staging / "table2.csv",
        )
        manifest["artifacts"] = {
            path.name: {
                "bytes": path.stat().st_size,
                "sha256": sha256_file(path),
            }
            for path in artifact_paths
        }
        manifest["manifest_content_sha256"] = _self_hash(
            manifest, "manifest_content_sha256"
        )
        write_json(staging / "run_manifest.json", manifest)
        os.replace(staging, output)
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    if args.strict and not summary["strict"]["passed"]:
        raise RuntimeError(
            "strict evaluation failed: " + ", ".join(summary["strict"]["failures"])
        )
    return summary


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        summary = run(args)
    except Exception as error:  # noqa: BLE001 - CLI should report a concise failure
        print(f"ERROR: {type(error).__name__}: {error}", file=sys.stderr)
        return 1
    print(
        json.dumps(
            {
                "status": "COMPLETE",
                "classification": summary["classification"],
                "output": str(args.output.resolve()),
                "n_eval": summary["n_eval"],
                "table2": summary["table2"],
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "ANNOTATION_SCHEMA_VERSION",
    "DOF_BY_TYPE",
    "DEFAULT_OUTPUT",
    "DEFAULT_PROTOCOL",
    "DEFAULT_ROSTER",
    "evaluate_xml",
    "load_annotations",
    "load_protocol",
    "load_roster",
    "make_summary",
    "parse_args",
    "run",
    "semantic_joint_types",
    "validate_formal_roster_provenance",
]
