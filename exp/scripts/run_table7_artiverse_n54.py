#!/usr/bin/env python3
"""Freeze and aggregate the outcome-independent Artiverse Table 7 N=54 cohort."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import shutil
import sys
from typing import Any, Iterable


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
WORKSPACE = REPO.parent.resolve()
PROTOCOL = REPO / "exp/reference/table7_production_readiness_protocol_v1.json"
PARENT = REPO / "exp/runtime/table7_artiverse"
PARENT_MANIFEST = PARENT / "manifest.json"
PARENT_RECORDS = PARENT / "asset_records.json"
PARENT_SUMMARY = PARENT / "summary.json"
PARENT_SELF_CHECK = PARENT / "self_check.json"
DEFAULT_OUTPUT = REPO / "exp/runtime/table7_artiverse_n54"

EXPECTED_PROTOCOL_ID = "nano3d_table7_production_readiness_v1"
EXPECTED_PROTOCOL_SHA256 = "5fc86932f35f8b66514d5747be732b5c75fef7215c987628f5dd28522f710a7c"
EXPECTED_PARENT_MANIFEST_SHA256 = "fa57055d04ad0ef47256ecbc7db7f9863ecefdb7a0f82e3feca33bba7ddf5e16"
EXPECTED_PARENT_RECORDS_SHA256 = "3b57418fb31bcbec98711333efb4691e8451e13de958ea50e3f8dfb968fa9c2e"
EXPECTED_SELECTED_IDS_SHA256 = "ed2b4076dbb142d21932be3dbb715426b3d77d5c39b0c7444999d16ba8b128b6"
SELECTION_SALT = "nano3d-table7-artiverse-n54-v1"
COHORT_SIZE = 54
REQUIRED = (
    "protocol_snapshot.json",
    "manifest.json",
    "asset_records.json",
    "summary.json",
    "self_check.json",
    "report.md",
)


def safe(path: Path, *, must_exist: bool = True) -> Path:
    root = WORKSPACE.resolve(strict=True)
    resolved = path.resolve(strict=must_exist)
    if resolved != root and root not in resolved.parents:
        raise RuntimeError(f"path outside authorized workspace: {resolved}")
    return resolved


def relative(path: Path) -> str:
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
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def selected_ids_sha256(asset_ids: Iterable[str]) -> str:
    payload = "".join(f"{asset_id}\n" for asset_id in asset_ids).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def selection_key(asset_id: str) -> tuple[str, str]:
    payload = SELECTION_SALT.encode("utf-8") + b"\0" + asset_id.encode("utf-8")
    return hashlib.sha256(payload).hexdigest(), asset_id


def select_assets(parent_assets: list[dict[str, Any]]) -> list[dict[str, Any]]:
    if len(parent_assets) != 3544:
        raise RuntimeError(f"unexpected parent cohort size: {len(parent_assets)}")
    ids = [row["asset_id"] for row in parent_assets]
    if len(ids) != len(set(ids)):
        raise RuntimeError("parent manifest asset IDs are not unique")
    selected_id_set = set(sorted(ids, key=selection_key)[:COHORT_SIZE])
    by_id = {row["asset_id"]: row for row in parent_assets}
    selected_ids = sorted(selected_id_set, key=selection_key)
    selected = [by_id[asset_id] for asset_id in selected_ids]
    if selected_ids_sha256(selected_ids) != EXPECTED_SELECTED_IDS_SHA256:
        raise RuntimeError("selected Artiverse N=54 identity hash mismatch")
    return selected


def validate_parent() -> tuple[dict[str, Any], list[dict[str, Any]], dict[str, Any]]:
    observed = {
        "manifest": sha256_file(PARENT_MANIFEST),
        "asset_records": sha256_file(PARENT_RECORDS),
    }
    expected = {
        "manifest": EXPECTED_PARENT_MANIFEST_SHA256,
        "asset_records": EXPECTED_PARENT_RECORDS_SHA256,
    }
    if observed != expected:
        raise RuntimeError(f"parent Artiverse evidence drift: expected={expected} observed={observed}")
    parent_self_check = read_json(PARENT_SELF_CHECK)
    if parent_self_check.get("status") != "PASS":
        raise RuntimeError("parent Artiverse self-check is not PASS")
    parent_manifest = read_json(PARENT_MANIFEST)
    parent_records = read_json(PARENT_RECORDS)
    parent_summary = read_json(PARENT_SUMMARY)
    manifest_ids = [row["asset_id"] for row in parent_manifest["assets"]]
    record_ids = [row["asset_id"] for row in parent_records]
    if manifest_ids != record_ids:
        raise RuntimeError("parent manifest and records are not identity-aligned")
    if parent_summary.get("status") != "COMPLETE":
        raise RuntimeError("parent Artiverse audit is not COMPLETE")
    return parent_manifest, parent_records, parent_summary


def nested_value(row: dict[str, Any], key: str) -> Any:
    value: Any = row
    for part in key.split("."):
        value = value[part]
    return value


def ratio(numerator: int, denominator: int) -> dict[str, Any]:
    return {
        "numerator": numerator,
        "denominator": denominator,
        "rate": numerator / denominator if denominator else None,
    }


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
    components = [item for row in evaluable for item in row["geometry"]["geometries"]]
    watertight = sum(item["watertight"] for item in components)
    manifold = sum(item["edge_manifold_proxy"] for item in components)
    open_edges = sum(item["open_edges"] for item in components)
    degenerate = sum(item["degenerate_faces"] for item in components)
    return {
        "requested_assets": len(rows),
        "geometry_evaluable_assets": len(evaluable),
        "geometry_not_evaluable_assets": len(rows) - len(evaluable),
        "mesh_load_error_assets": sum(bool(row["geometry"]["load_errors"]) for row in rows),
        "mesh_load_error_count": sum(len(row["geometry"]["load_errors"]) for row in rows),
        "no_mesh_payload_assets": sum(not row["geometry"]["mesh_payload_count"] for row in rows),
        "readable_mesh_payloads": sum(
            row["geometry"]["readable_mesh_payload_count"] for row in evaluable
        ),
        "readable_geometries": len(components),
        "evaluation_unit_label": "mesh components",
        "mesh_scope": "canonical-deduplicated native URDF visual mesh dependency closure",
        "watertight": {
            "geometry_level": ratio(watertight, len(components)),
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
            "geometry_level": ratio(manifold, len(components)),
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


def build_manifest(
    parent_manifest: dict[str, Any], selected_assets: list[dict[str, Any]], protocol_hash: str,
) -> dict[str, Any]:
    return {
        "schema_version": "1.0.0",
        "protocol_id": EXPECTED_PROTOCOL_ID,
        "protocol_snapshot": relative(DEFAULT_OUTPUT / "protocol_snapshot.json"),
        "protocol_sha256": protocol_hash,
        "method": "Artiverse",
        "axis": "Table 7 Production Readiness",
        "role": "production-readiness control (frozen N=54 pre-release subset)",
        "manifest_frozen_before_scoring": True,
        "selection_policy": {
            "algorithm": "sha256(salt + NUL + full asset_id), ascending by (digest, asset_id)",
            "missing_or_failed_assets_retained": True,
            "outcome_based_filtering": False,
            "requested_assets": COHORT_SIZE,
            "salt": SELECTION_SALT,
        },
        "selected_asset_ids_sha256": EXPECTED_SELECTED_IDS_SHA256,
        "parent_evidence": {
            "manifest": {
                "path": relative(PARENT_MANIFEST),
                "sha256": EXPECTED_PARENT_MANIFEST_SHA256,
            },
            "asset_records": {
                "path": relative(PARENT_RECORDS),
                "sha256": EXPECTED_PARENT_RECORDS_SHA256,
            },
            "audit_identity": parent_manifest["audit_identity"],
            "dataset_revision": parent_manifest["frozen_inputs"]["dataset_revision"],
            "release_identity": parent_manifest["frozen_inputs"]["release_identity"],
        },
        "representation_adapter": parent_manifest["representation_adapter"],
        "assets": selected_assets,
    }


def build_summary(
    records: list[dict[str, Any]], manifest_hash: str, protocol_hash: str,
    parent_summary: dict[str, Any],
) -> dict[str, Any]:
    available = sum(row["availability"]["state"] == "pass" for row in records)
    return {
        "schema_version": "1.0.0",
        "protocol_id": EXPECTED_PROTOCOL_ID,
        "status": "COMPLETE",
        "interpretation": "OUTCOME_INDEPENDENT_N54_SUBSET_AUDIT",
        "cohort": {
            "requested_assets": len(records),
            "available_assets": available,
            "unavailable_assets": len(records) - available,
            "package_evaluable_assets": sum(
                row["package_evaluability"]["state"] == "pass" for row in records
            ),
            "geometry_evaluable_assets": sum(row["geometry"]["evaluable"] for row in records),
            "category_count": len({row["category"] for row in records}),
            "source_repository_count": len({row["source_repository"] for row in records}),
            "selection": (
                "fixed salted-SHA256 rank over the frozen 3,544-asset manifest; "
                "no outcome filtering"
            ),
        },
        "provenance": {
            "parent_manifest_sha256": EXPECTED_PARENT_MANIFEST_SHA256,
            "parent_asset_records_sha256": EXPECTED_PARENT_RECORDS_SHA256,
            "selected_asset_ids_sha256": EXPECTED_SELECTED_IDS_SHA256,
            "dataset_revision": parent_summary["provenance"]["dataset_revision"],
            "release_identity": parent_summary["provenance"]["release_identity"],
            "asset_measurements_reused_without_modification": True,
            "subset_aggregated_after_outcome_independent_identity_freeze": True,
        },
        "results": {
            "geometry": geometry_summary(records),
            "size_bytes": {
                "source": parent_summary["results"]["size_bytes"]["source"],
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
        "fail_closed": parent_summary["fail_closed"],
        "hashes": {
            "protocol_sha256": protocol_hash,
            "manifest_sha256": manifest_hash,
            "runner_sha256": sha256_file(SCRIPT),
        },
    }


def report_text(summary: dict[str, Any]) -> str:
    cohort = summary["cohort"]
    result = summary["results"]
    geometry = result["geometry"]
    sizes = result["size_bytes"]
    watertight = geometry["watertight"]
    manifold = geometry["manifold"]
    return f"""# Table 7: Artiverse production readiness (frozen N=54)

Status: **COMPLETE**

The cohort is the first 54 full asset IDs under a fixed salted SHA-256 rank over
the frozen 3,544-asset manifest. Selection reads no outcome field and failed or
unavailable selected assets would remain in the denominator. The selected cohort
covers {cohort['category_count']} categories and {cohort['source_repository_count']} source repositories.

## Table 7 row

| Method | Watertight | Manifold | Open Edges | Degenerate Faces | Self-Intersection | Source KB | URDF KB | Mesh KB | Portable Package | Deterministic Build | Semantic Complete | Kinematic Complete | Physical Complete |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Artiverse (frozen N=54) | {watertight['per_asset_mean_fraction']:.6f} mean/asset; {watertight['geometry_level']['numerator']}/{watertight['geometry_level']['denominator']} mesh components | {manifold['per_asset_mean_fraction']:.6f} edge-manifold mean/asset; {manifold['geometry_level']['numerator']}/{manifold['geometry_level']['denominator']} mesh components | {geometry['open_edges']['per_asset_mean']:,.2f}/asset; {geometry['open_edges']['total']:,} total | {geometry['degenerate_faces']['per_asset_mean']:.2f}/asset; {geometry['degenerate_faces']['total']:,} total | N/E | shared {sizes['source']['total_bytes'] / 1024:.2f} KiB total; per-asset N/E | {sizes['urdf']['mean_bytes'] / 1024:.2f} KiB/asset; {sizes['urdf']['total_bytes'] / 1024:.2f} KiB total | {sizes['mesh']['mean_bytes'] / 1024:.2f} KiB/asset; {sizes['mesh']['total_bytes'] / 1024:.2f} KiB total | {result['portable_package']['pass']}/{result['portable_package']['denominator']} | N/E | N/E strict; field proxy {result['semantic_field_proxy']['pass']}/{result['semantic_field_proxy']['evaluable_denominator']} | {result['kinematic_complete']['pass']}/{result['kinematic_complete']['evaluable_denominator']} | {result['physical_complete']['pass']}/{result['physical_complete']['evaluable_denominator']} |

## Denominators and boundaries

- Asset denominator: requested/available/package-evaluable/geometry-evaluable = {cohort['requested_assets']}/{cohort['available_assets']}/{cohort['package_evaluable_assets']}/{cohort['geometry_evaluable_assets']}.
- A mesh component is one independently loaded triangle-mesh object inside an asset; one asset may contain several components. Readable mesh components: {geometry['readable_geometries']}.
- Watertight and edge-manifold report an asset-macro mean plus a mesh-component numerator/denominator.
- Open edges and degenerate faces sum over the same mesh components, then divide by geometry-evaluable assets for the per-asset mean.
- Strict semantics remain N/E. The separately labelled name/tree field proxy is not semantic correctness.
- Physical Complete is a strict AND over native collision, mass, inertia, joint dynamics, and contact/friction metadata; runtime defaults do not count.

## Reproduction

```bash
python arti-skill/exp/scripts/run_table7_artiverse_n54.py
python arti-skill/exp/scripts/run_table7_artiverse_n54.py --verify-only
```
"""


def current_checks(
    output: Path, protocol: dict[str, Any], manifest: dict[str, Any],
    records: list[dict[str, Any]], summary: dict[str, Any], parent_manifest: dict[str, Any],
    parent_records: list[dict[str, Any]], parent_summary: dict[str, Any],
) -> dict[str, bool]:
    selected_assets = select_assets(parent_manifest["assets"])
    selected_ids = [row["asset_id"] for row in selected_assets]
    parent_by_id = {row["asset_id"]: row for row in parent_records}
    expected_records = [parent_by_id[asset_id] for asset_id in selected_ids]
    manifest_hash = sha256_file(output / "manifest.json")
    protocol_hash = sha256_file(output / "protocol_snapshot.json")
    expected_summary = build_summary(expected_records, manifest_hash, protocol_hash, parent_summary)
    result = summary["results"]
    return {
        "required_outputs_match_protocol": tuple(protocol.get("required_outputs", [])) == REQUIRED,
        "all_required_outputs_exist": all((output / name).is_file() for name in REQUIRED),
        "protocol_snapshot_exact": protocol_hash == sha256_file(PROTOCOL),
        "protocol_hash_expected": protocol_hash == EXPECTED_PROTOCOL_SHA256,
        "parent_manifest_hash_expected": sha256_file(PARENT_MANIFEST) == EXPECTED_PARENT_MANIFEST_SHA256,
        "parent_records_hash_expected": sha256_file(PARENT_RECORDS) == EXPECTED_PARENT_RECORDS_SHA256,
        "selection_is_outcome_independent": manifest["selection_policy"]["outcome_based_filtering"] is False,
        "selected_id_hash_expected": selected_ids_sha256(selected_ids) == EXPECTED_SELECTED_IDS_SHA256,
        "selected_identities_unique_n54": len(selected_ids) == len(set(selected_ids)) == COHORT_SIZE,
        "manifest_assets_match_selection": manifest["assets"] == selected_assets,
        "records_are_exact_parent_subset": records == expected_records,
        "summary_exactly_reaggregated": summary == expected_summary,
        "report_exactly_regenerated": (output / "report.md").read_text(encoding="utf-8") == report_text(summary),
        "requested_denominator_fixed": summary["cohort"]["requested_assets"] == COHORT_SIZE,
        "availability_conserved": summary["cohort"]["requested_assets"] == summary["cohort"]["available_assets"] + summary["cohort"]["unavailable_assets"],
        "mesh_component_denominator_explicit": result["geometry"]["readable_geometries"] == result["geometry"]["watertight"]["geometry_level"]["denominator"] == result["geometry"]["manifold"]["geometry_level"]["denominator"],
        "state_partitions_conserve_assets": all(
            result[name]["pass"] + result[name]["fail"] + result[name]["not_evaluable"] == COHORT_SIZE
            for name in (
                "portable_package", "deterministic_build", "semantic_complete",
                "semantic_field_proxy", "kinematic_complete", "physical_complete",
            )
        ),
    }


def create(output: Path) -> dict[str, Any]:
    output = safe(output, must_exist=False)
    safe(output.parent, must_exist=False).mkdir(parents=True, exist_ok=True)
    output.mkdir(parents=True, exist_ok=True)
    parent_manifest, parent_records, parent_summary = validate_parent()
    protocol = read_json(PROTOCOL)
    if protocol.get("protocol_id") != EXPECTED_PROTOCOL_ID:
        raise RuntimeError("unexpected Table 7 protocol")

    temporary = safe(output / "protocol_snapshot.json.tmp", must_exist=False)
    shutil.copyfile(safe(PROTOCOL), temporary)
    temporary.replace(output / "protocol_snapshot.json")
    protocol_hash = sha256_file(output / "protocol_snapshot.json")
    selected_assets = select_assets(parent_manifest["assets"])
    selected_ids = [row["asset_id"] for row in selected_assets]
    parent_by_id = {row["asset_id"]: row for row in parent_records}
    records = [parent_by_id[asset_id] for asset_id in selected_ids]

    manifest = build_manifest(parent_manifest, selected_assets, protocol_hash)
    manifest["protocol_snapshot"] = relative(output / "protocol_snapshot.json")
    write_json(output / "manifest.json", manifest)
    write_json(output / "asset_records.json", records)
    manifest_hash = sha256_file(output / "manifest.json")
    summary = build_summary(records, manifest_hash, protocol_hash, parent_summary)
    write_json(output / "summary.json", summary)
    write_text(output / "report.md", report_text(summary))
    write_json(output / "self_check.json", {"status": "PROVISIONAL"})

    checks = current_checks(
        output, protocol, manifest, records, summary,
        parent_manifest, parent_records, parent_summary,
    )
    artifact_hashes = {
        name: sha256_file(output / name) for name in REQUIRED if name != "self_check.json"
    }
    self_check = {
        "schema_version": "1.0.0",
        "protocol_id": EXPECTED_PROTOCOL_ID,
        "status": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks,
        "protocol_sha256": protocol_hash,
        "manifest_sha256": manifest_hash,
        "selected_asset_ids_sha256": EXPECTED_SELECTED_IDS_SHA256,
        "artifact_hashes": artifact_hashes,
        "artifact_hash_scope": "required outputs except self_check.json to avoid a circular hash",
    }
    write_json(output / "self_check.json", self_check)
    if self_check["status"] != "PASS":
        failed = [name for name, passed in checks.items() if not passed]
        raise RuntimeError(f"self-check failed: {failed}")
    return verify(output)


def verify(output: Path) -> dict[str, Any]:
    output = safe(output)
    parent_manifest, parent_records, parent_summary = validate_parent()
    protocol = read_json(output / "protocol_snapshot.json")
    manifest = read_json(output / "manifest.json")
    records = read_json(output / "asset_records.json")
    summary = read_json(output / "summary.json")
    self_check = read_json(output / "self_check.json")
    checks = current_checks(
        output, protocol, manifest, records, summary,
        parent_manifest, parent_records, parent_summary,
    )
    errors = [name for name, passed in checks.items() if not passed]
    for name, expected in self_check.get("artifact_hashes", {}).items():
        if sha256_file(output / name) != expected:
            errors.append(f"artifact hash mismatch: {name}")
    if self_check.get("status") != "PASS":
        errors.append("recorded self-check is not PASS")
    if self_check.get("checks") != checks:
        errors.append("recorded self-check values drifted")
    return {
        "status": "PASS" if not errors else "FAIL",
        "errors": errors,
        "requested_assets": len(records),
        "available_assets": summary["cohort"]["available_assets"],
        "geometry_evaluable_assets": summary["cohort"]["geometry_evaluable_assets"],
        "selected_asset_ids_sha256": EXPECTED_SELECTED_IDS_SHA256,
        "output": relative(output),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--verify-only", action="store_true")
    args = parser.parse_args()
    try:
        result = verify(args.output) if args.verify_only else create(args.output)
    except Exception as exc:
        print(json.dumps({"status": "FAIL", "errors": [str(exc)]}, indent=2, sort_keys=True))
        return 2
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["status"] == "PASS" else 2


if __name__ == "__main__":
    sys.exit(main())
