#!/usr/bin/env python3
"""Build the PV-A Table 5 N=200 manifest from the formal release roster.

The selection is the immutable ordinal prefix ``[0:200]`` of the PV-A
full-release roster.  Upstream Table 1--4 records are bound by asset id,
ordinal, package binding, and URDF hash before they are copied into the
dataset-neutral Table 5 runtime contract.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any, Mapping


SCRIPT_PATH = Path(__file__).resolve()
REPO_ROOT = SCRIPT_PATH.parents[2]
PVA_ROOT = Path("/mnt/zsn/data/particulate/datasets/PV-A")
RUN_ROOT = REPO_ROOT / "exp/runtime/table5_pva_n200_all_20260828"
ROSTER_PATH = (
    REPO_ROOT
    / "exp/runtime/pva_table1234_full_release_20260826/roster/full_release_roster.jsonl"
)
EVALUATION_ROOT = (
    REPO_ROOT / "exp/runtime/pva_table1234_full_release_20260826/evaluation"
)
SAMPLE_SIZE = 200

# The helper module is intentionally reused so the runtime protocol and URDF
# parser cannot drift from the established six-dataset Table 5 implementation.
sys.path.insert(0, str(SCRIPT_PATH.parent))
from table5_n200_manifest import (  # noqa: E402
    _frozen_protocol,
    _parse_urdf,
    canonical_sha256,
    sha256_file,
)


class ManifestError(ValueError):
    pass


def _read_jsonl_prefix(path: Path, count: int) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                value = json.loads(line)
            except json.JSONDecodeError as error:
                raise ManifestError(f"invalid JSONL at {path}:{line_number}") from error
            if not isinstance(value, dict):
                raise ManifestError(f"non-object JSONL row at {path}:{line_number}")
            rows.append(value)
            if len(rows) == count:
                break
    if len(rows) != count:
        raise ManifestError(f"{path} contains only {len(rows)} rows; expected {count}")
    return rows


def _record_key(record: Mapping[str, Any], table: str) -> str | None:
    if table == "table4":
        value = record.get("dataset_id")
    else:
        value = record.get("asset_id")
    return value if isinstance(value, str) and value else None


def _bind_prefix(path: Path, table: str, roster: list[dict[str, Any]]) -> list[dict[str, Any]]:
    records = _read_jsonl_prefix(path, len(roster))
    bound: list[dict[str, Any]] = []
    seen: set[str] = set()
    for index, (source, record) in enumerate(zip(roster, records)):
        key = _record_key(record, table)
        expected = source["asset_id"]
        if key != expected or key in seen:
            raise ManifestError(
                f"{table} prefix identity mismatch at ordinal {index}: {key!r} != {expected!r}"
            )
        ordinal = record.get("ordinal", record.get("order"))
        if ordinal is not None and ordinal != index:
            raise ManifestError(f"{table} ordinal mismatch at {index}: {ordinal!r}")
        for field in ("primary_urdf_sha256", "expected_primary_urdf_sha256"):
            value = record.get(field)
            if value is not None and value != source["primary_urdf_sha256"]:
                raise ManifestError(f"{table} URDF hash mismatch at ordinal {index}")
        binding = record.get("package_binding_sha256")
        if binding is not None and binding != source["package_binding_sha256"]:
            raise ManifestError(f"{table} package binding mismatch at ordinal {index}")
        seen.add(key)
        bound.append(record)
    return bound


def _gate(record: Mapping[str, Any], name: str) -> bool | None:
    value = record.get(name)
    if isinstance(value, bool):
        return value
    metrics = record.get("metrics")
    if isinstance(metrics, Mapping):
        value = metrics.get(name)
        if isinstance(value, bool):
            return value
        if isinstance(value, Mapping) and isinstance(value.get("pass"), bool):
            return bool(value["pass"])
    return None


def _positive_number(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    value = float(value)
    return value if value > 0 else None


def _atomic_write_json(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(
        json.dumps(value, ensure_ascii=True, allow_nan=False, sort_keys=True, indent=2)
        + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def build_manifest(
    *,
    roster_path: Path = ROSTER_PATH,
    evaluation_root: Path = EVALUATION_ROOT,
    sample_size: int = SAMPLE_SIZE,
) -> dict[str, Any]:
    if sample_size != SAMPLE_SIZE:
        raise ManifestError("this frozen PV-A manifest is defined only for N=200")
    roster = _read_jsonl_prefix(roster_path, sample_size)
    for index, row in enumerate(roster):
        if row.get("ordinal") != index or row.get("release_dataset") != "PV-A":
            raise ManifestError(f"roster prefix is not ordinal PV-A at {index}")

    table_paths = {
        "table1": evaluation_root / "table1/asset_records.jsonl",
        "table2": evaluation_root / "table2/asset_records.jsonl",
        "table3": evaluation_root / "table3/records.jsonl",
        "table4": evaluation_root / "table4/records.jsonl",
    }
    upstream = {
        table: _bind_prefix(path, table, roster) for table, path in table_paths.items()
    }

    rows: list[dict[str, Any]] = []
    parse_issue_count = 0
    for index, source in enumerate(roster):
        urdf_path = Path(source["primary_urdf_path"]).resolve()
        package_root = Path(source["source_path"]).resolve()
        if not urdf_path.is_file() or urdf_path.parent != package_root:
            raise ManifestError(f"roster source path is not materialized at ordinal {index}")
        observed_hash = sha256_file(urdf_path)
        if observed_hash != source["primary_urdf_sha256"]:
            raise ManifestError(f"source URDF hash mismatch at ordinal {index}")
        parsed = _parse_urdf(package_root, urdf_path)
        issues = list(parsed.get("issues", []))
        table1 = upstream["table1"][index]
        if table1.get("parse_success") is not True or table1.get("valid_tree") is not True:
            issues.append("upstream_table1_parse_or_tree_failed")
        if table1.get("missing_resources"):
            issues.append("upstream_table1_missing_resources")
        if parsed.get("xml_counts") != {
            "links": table1.get("link_count"),
            "joints": table1.get("joint_count"),
            "fixed_joints": source.get("xml_counts", {}).get("fixed_joints"),
            "movable_joints": table1.get("non_fixed_joint_count"),
            "visual_elements": parsed.get("xml_counts", {}).get("visual_elements"),
            "collision_elements": parsed.get("xml_counts", {}).get("collision_elements"),
            "inertial_elements": parsed.get("xml_counts", {}).get("inertial_elements"),
        }:
            # Counts from the independent static parser are retained as the
            # authority; upstream records may intentionally omit XML subcounts.
            if parsed.get("xml_counts") is None:
                issues.append("table5_static_xml_counts_unavailable")
        if parsed.get("joint_tree") is None:
            issues.append("table5_joint_tree_unavailable")
        if issues:
            parse_issue_count += 1

        table4 = upstream["table4"][index]
        bbox = _positive_number(table4.get("object_bbox_diagonal_m"))
        strict = {
            "table2": {
                "strict_urdf_pass": _gate(upstream["table2"][index], "strict_urdf_pass"),
                "source": str(table_paths["table2"].resolve()),
                "ordinal": index,
            },
            "table3": {
                "strict_kinematic_pass": _gate(upstream["table3"][index], "strict_kinematic_pass"),
                "source": str(table_paths["table3"].resolve()),
                "ordinal": index,
            },
            "table4": {
                "strict_collision_pass": _gate(upstream["table4"][index], "strict_collision_pass"),
                "source": str(table_paths["table4"].resolve()),
                "ordinal": index,
            },
        }
        warnings: list[str] = []
        if bbox is None:
            warnings.append("bounding_box_diagonal_not_available")
        if any(value is None for value in (
            strict["table2"]["strict_urdf_pass"],
            strict["table3"]["strict_kinematic_pass"],
            strict["table4"]["strict_collision_pass"],
        )):
            warnings.append("upstream_strict_gate_not_available")
        preflight = {
            "status": "failed" if issues else "pass",
            "issues": sorted(set(issues)),
            "warnings": sorted(set(warnings)),
            "simulator_eligible": not issues,
        }
        row: dict[str, Any] = {
            "dataset_slug": "pva",
            "dataset_name": "Ours / PV-A",
            "dataset_id": f"pva_{index:04d}",
            "asset_id": source["asset_id"],
            "category": source["category"],
            "ordinal": index,
            "selection_rank": index,
            "source_asset_id": source.get("source_asset_id"),
            "seed": source.get("seed"),
            "package_root": str(package_root),
            "urdf_path": str(urdf_path),
            "urdf_sha256": source["primary_urdf_sha256"],
            "package_binding_sha256": source.get("package_binding_sha256"),
            "bounding_box_diagonal": bbox,
            "bounding_box": {
                "status": "available" if bbox is not None else "not_available",
                "diagonal_m": bbox,
                "source": str(table_paths["table4"].resolve()),
            },
            "joint_tree": parsed.get("joint_tree"),
            "scalar_joints": parsed.get("scalar_joints", []),
            "xml_counts": parsed.get("xml_counts"),
            "preflight": preflight,
            "strict_gates": strict,
            "upstream": {
                "table1": {
                    "source": str(table_paths["table1"].resolve()),
                    "record_status": table1.get("status"),
                    "package_binding_sha256": table1.get("package_binding_sha256"),
                },
                "table2": {"record_status": upstream["table2"][index].get("status")},
                "table3": {"record_status": upstream["table3"][index].get("status")},
                "table4": {"record_status": table4.get("status")},
            },
        }
        row["row_sha256"] = canonical_sha256(row, exclude_fields=("row_sha256",))
        rows.append(row)

    protocol = _frozen_protocol(sample_size)
    manifest: dict[str, Any] = {
        "schema_version": "table5_pva_n200_manifest_v1",
        "manifest_title": "PV-A formal full-release roster prefix Table 5 N=200",
        "dataset_count": 1,
        "sample_size": sample_size,
        "total_rows": len(rows),
        "ordered_dataset_slugs": ["pva"],
        "selection": {
            "rule": "formal full-release roster rows with ordinal [0:200], before Table 5 preflight",
            "replacement": False,
            "resampling": False,
            "source_roster": str(roster_path.resolve()),
            "source_roster_sha256": sha256_file(roster_path),
            "prefix_ordinals": [0, sample_size - 1],
            "prefix_asset_ids_sha256": canonical_sha256([row["asset_id"] for row in roster]),
        },
        "source": {
            "dataset_root": str(PVA_ROOT),
            "root_manifest_csv": str((PVA_ROOT / "manifest.csv").resolve()),
            "root_manifest_csv_sha256": sha256_file(PVA_ROOT / "manifest.csv"),
            "strict_preparation": str((PVA_ROOT / "particulate-strict/preparation.json").resolve()),
            "strict_runtime_acceptance": str((PVA_ROOT / "particulate-strict/runtime-safe-acceptance.json").resolve()),
            "upstream_evaluation_root": str(evaluation_root.resolve()),
            "upstream_file_sha256": {
                table: sha256_file(path) for table, path in table_paths.items()
            },
        },
        "upstream_scope": {
            "table1": "formal PV-A full-release Table 1 records, roster-bound",
            "table2": "formal PV-A full-release Table 2 records, roster-bound",
            "table3": "formal PV-A full-release Table 3 records, roster-bound",
            "table4": "formal PV-A full-release Table 4 records, roster-bound",
            "strict_gate_counts_prefix": {
                "strict_urdf_pass": sum(
                    bool(_gate(record, "strict_urdf_pass")) for record in upstream["table2"]
                ),
                "strict_kinematic_pass": sum(
                    bool(_gate(record, "strict_kinematic_pass")) for record in upstream["table3"]
                ),
                "strict_collision_pass": sum(
                    bool(_gate(record, "strict_collision_pass")) for record in upstream["table4"]
                ),
            },
        },
        "protocol": protocol,
        "protocol_sha256": protocol["protocol_sha256"],
        "datasets": [
            {
                "dataset_slug": "pva",
                "dataset_name": "Ours / PV-A",
                "parent": {
                    "path": str(roster_path.resolve()),
                    "sha256": sha256_file(roster_path),
                    "row_container": "jsonl",
                    "declared_n_eval": sample_size,
                    "prefix_start": 0,
                    "prefix_end_exclusive": sample_size,
                },
                "rows": rows,
            }
        ],
    }
    manifest["manifest_sha256"] = canonical_sha256(
        manifest, exclude_fields=("manifest_sha256",)
    )
    manifest["generation"] = {
        "script": str(SCRIPT_PATH),
        "script_sha256": sha256_file(SCRIPT_PATH),
        "static_parser_issue_rows": parse_issue_count,
    }
    # `generation` is part of the manifest authority, so update the hash after
    # recording the generator receipt.
    manifest["manifest_sha256"] = canonical_sha256(
        manifest, exclude_fields=("manifest_sha256",)
    )
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=RUN_ROOT / "manifest.json")
    parser.add_argument("--roster", type=Path, default=ROSTER_PATH)
    parser.add_argument("--evaluation-root", type=Path, default=EVALUATION_ROOT)
    args = parser.parse_args()
    manifest = build_manifest(
        roster_path=args.roster.resolve(), evaluation_root=args.evaluation_root.resolve()
    )
    _atomic_write_json(args.out.resolve(), manifest)
    print(json.dumps({
        "manifest": str(args.out.resolve()),
        "manifest_sha256": manifest["manifest_sha256"],
        "protocol_sha256": manifest["protocol_sha256"],
        "rows": manifest["total_rows"],
        "static_parser_issue_rows": manifest["generation"]["static_parser_issue_rows"],
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
