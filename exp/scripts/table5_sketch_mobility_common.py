"""Frozen source and receipt helpers for SketchMobility Table 5 N=800."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping

try:
    from table5_ours_common import (
        atomic_write_json,
        canonical_sha256,
        cleanup_new_prepare_output,
        collect_artifact_set,
        fk_link_poses,
        output_lock,
        package_binding,
        parse_urdf_metadata,
        protocol_with_hash,
        validate_output_path,
    )
except ModuleNotFoundError:
    from exp.scripts.table5_ours_common import (
        atomic_write_json,
        canonical_sha256,
        cleanup_new_prepare_output,
        collect_artifact_set,
        fk_link_poses,
        output_lock,
        package_binding,
        parse_urdf_metadata,
        protocol_with_hash,
        validate_output_path,
    )


DATASET_NAME = "SketchMobility"
SELECTION_PROTOCOL = "sketch-mobility-table1-global-sample-v1"
FORMAL_SEED = "arti-skill-table1-sketch-mobility-n800-v1"
FORMAL_N_RELEASE = 4956
FORMAL_N_EVAL = 800
FORMAL_TABLE1_MANIFEST_SHA256 = (
    "081e9e9125f8945cad67a751949e659f6d4e73817704c07cd3fcd4b657ffc696"
)
FORMAL_RELEASE_MANIFEST_SHA256 = (
    "5b4b0891bafeba5029e5e1dd71042e5be8543eccb2990edfbc9aba71a1ac56fb"
)
FORMAL_RELEASE_UNIVERSE_SHA256 = (
    "a9c128d24ab9cba03d593ecff17f1e7284ef1f74f09f69939cbef7c0cc8af346"
)
FORMAL_ORDERED_IDENTITY_SHA256 = (
    "f7cd81b2c6ef85c915582a911e3040fbc330a90e626aafdf71a9c5919ab1402f"
)
FORMAL_ORDERED_MANIFEST_ROOT_SHA256 = (
    "a88506e1da8e7e8b61a740965dea2faba4e9ab8280f47417e17550024b6dde17"
)
FORMAL_ORDERED_PACKAGE_BINDING_SHA256 = (
    "5fa3622502d74feacffd327b61c7a43f7c30d6d6109d4439d79651a39a39805d"
)
FORMAL_ORDERED_URDF_BINDING_SHA256 = (
    "4fee367147d3f83482ee34959723bf12539c22472153b10dce252d36967d2f86"
)
FORMAL_PACKAGE_FILE_COUNT = 31403
FORMAL_PACKAGE_TOTAL_BYTES = 2510671298
FORMAL_CATEGORY_COUNT = 67
FORMAL_PREFLIGHT_FAILURES = 489
FORMAL_UPSTREAM_ARTIFACT_SETS = {
    "table2": {
        "artifact_set_sha256": "f404a084b68e257222c6d6c146f67c8ea90db27ec8728702be4d9691e27fb589",
        "file_count": 9,
        "total_bytes": 13486161,
    },
    "table3": {
        "artifact_set_sha256": "506819f33acf0acc638280a419e5d9cd44280cf858dfc5fb324ea4140b009d2d",
        "file_count": 13,
        "total_bytes": 10953965,
    },
    "table4": {
        "artifact_set_sha256": "7201f8299f8fe0ec1f93a6269149e506bb75614a64eb96d1b8362c96f800aaa7",
        "file_count": 20,
        "total_bytes": 45951870,
    },
}
CANONICAL_PROTOCOL_PATH = (
    Path(__file__).parents[1]
    / "reference/table5_sketch_mobility_n800_protocol_v1.json"
)
CANONICAL_PROTOCOL_FILE_SHA256 = (
    "32632da34a5830b62e49dbafff081f346ba83c0ba9f38d99e6f2a6f7a53a3325"
)
EXPECTED_GENESIS_GPU_BINDING = {
    "cuda_visible_devices": "3",
    "physical_device_index": 3,
    "visible_device_index": 0,
    "gpu_uuid": "GPU-ebc0d328-a3fa-7e89-2733-cadb001661f7",
}


class ManifestError(ValueError):
    """Raised when a frozen input or receipt does not validate."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(4 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _nested(value: Mapping[str, Any], dotted_path: str) -> Any:
    current: Any = value
    for segment in dotted_path.split("."):
        if not isinstance(current, Mapping) or segment not in current:
            raise ManifestError(f"protocol missing required field: {dotted_path}")
        current = current[segment]
    return current


def validate_protocol_schema(protocol: dict[str, Any]) -> None:
    expected = {
        "schema_version": "table5_sketch_mobility_protocol_v1",
        "source.dataset": DATASET_NAME,
        "source.release_status": "RELEASED_MIXED_LICENSE_SECONDARY_CURATION",
        "source.required_release_count": FORMAL_N_RELEASE,
        "source.required_selected_count": FORMAL_N_EVAL,
        "source.required_raw_category_count": FORMAL_CATEGORY_COUNT,
        "source.table1_manifest_sha256": FORMAL_TABLE1_MANIFEST_SHA256,
        "source.release_manifest_sha256": FORMAL_RELEASE_MANIFEST_SHA256,
        "source.release_universe_sha256": FORMAL_RELEASE_UNIVERSE_SHA256,
        "source.ordered_identity_sha256": FORMAL_ORDERED_IDENTITY_SHA256,
        "source.ordered_manifest_root_sha256": FORMAL_ORDERED_MANIFEST_ROOT_SHA256,
        "source.ordered_package_binding_sha256": FORMAL_ORDERED_PACKAGE_BINDING_SHA256,
        "source.ordered_urdf_binding_sha256": FORMAL_ORDERED_URDF_BINDING_SHA256,
        "selection.identity_authority": "manifest_root",
        "selection.internal_id": "sketch_<zero_based_order:04d>",
        "selection.selected_count": FORMAL_N_EVAL,
        "selection.replacement": "never",
        "selection.retained_preflight_failures": FORMAL_PREFLIGHT_FAILURES,
        "runtime.base": "fixed",
        "runtime.contacts": "enabled",
        "runtime.timestep_s": {"numerator": 1, "denominator": 240},
        "runtime.child_timeout_s": 300,
        "cross_simulator.trajectory_samples": 31,
        "cross_simulator.sample_cadence_steps": 12,
        "cross_simulator.thresholds.normalized_joint_rmse": 0.1,
        "cross_simulator.thresholds.translation_over_bbox_diagonal": 0.02,
        "cross_simulator.thresholds.rotation_rad": 0.1,
        "cross_simulator.all_three_denominator": FORMAL_N_EVAL,
        "adapters.pybullet.ignore_collision": False,
        "adapters.pybullet.self_collision": True,
        "adapters.pybullet.contacts": "enabled",
        "adapters.mujoco.contacts": "enabled",
        "adapters.mujoco.self_collision": True,
        "adapters.genesis.backend": "cuda",
        "adapters.genesis.gpu_binding": EXPECTED_GENESIS_GPU_BINDING,
        "adapters.genesis.collision": True,
        "adapters.genesis.self_collision": True,
        "adapters.genesis.contacts": "enabled",
        "adapters.genesis.visualization": False,
    }
    for name, artifact_set in FORMAL_UPSTREAM_ARTIFACT_SETS.items():
        expected[f"upstream_strict_gates.{name}.artifact_set"] = artifact_set
        expected[f"upstream_strict_gates.{name}.denominator"] = FORMAL_N_EVAL
    for dotted_path, required in expected.items():
        if _nested(protocol, dotted_path) != required:
            raise ManifestError(f"protocol semantic mismatch: {dotted_path}")


def validate_canonical_protocol(protocol_path: Path) -> dict[str, Any]:
    resolved = Path(protocol_path).resolve(strict=True)
    if resolved != CANONICAL_PROTOCOL_PATH.resolve(strict=True):
        raise ManifestError(f"canonical protocol path required: {CANONICAL_PROTOCOL_PATH}")
    if sha256_file(resolved) != CANONICAL_PROTOCOL_FILE_SHA256:
        raise ManifestError("canonical protocol file hash mismatch")
    try:
        protocol = json.loads(resolved.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise ManifestError(f"invalid canonical protocol JSON: {error}") from error
    validate_protocol_schema(protocol)
    return protocol


def expected_genesis_gpu_binding(protocol: dict[str, Any]) -> dict[str, Any]:
    validate_protocol_schema(protocol)
    return dict(protocol["adapters"]["genesis"]["gpu_binding"])


def validate_genesis_gpu_binding(
    protocol: dict[str, Any], observed: Mapping[str, Any]
) -> None:
    expected = expected_genesis_gpu_binding(protocol)
    if dict(observed) != expected:
        raise ManifestError(
            f"Genesis GPU binding mismatch: observed={dict(observed)!r}"
        )


def publish_receipt_set(
    output_root: Path, protocol: dict[str, Any], manifest: dict[str, Any]
) -> None:
    output_root = Path(output_root)
    atomic_write_json(output_root / "protocol.json", protocol)
    atomic_write_json(output_root / "manifest.json", manifest)
    atomic_write_json(
        output_root / "receipt_set.json",
        {
            "schema_version": "table5_sketch_mobility_receipt_set_v1",
            "protocol_sha256": sha256_file(output_root / "protocol.json"),
            "manifest_sha256": sha256_file(output_root / "manifest.json"),
        },
    )


def validate_receipt_set(
    output_root: Path,
) -> tuple[dict[str, Any], dict[str, Any]]:
    output_root = Path(output_root)
    marker_path = output_root / "receipt_set.json"
    protocol_path = output_root / "protocol.json"
    manifest_path = output_root / "manifest.json"
    try:
        marker = json.loads(marker_path.read_text(encoding="utf-8"))
        protocol = json.loads(protocol_path.read_text(encoding="utf-8"))
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ManifestError(f"incomplete receipt set: {error}") from error
    if marker.get("schema_version") != "table5_sketch_mobility_receipt_set_v1":
        raise ManifestError("receipt marker schema mismatch")
    if marker.get("protocol_sha256") != sha256_file(protocol_path):
        raise ManifestError("receipt marker protocol hash mismatch")
    if marker.get("manifest_sha256") != sha256_file(manifest_path):
        raise ManifestError("receipt marker manifest hash mismatch")
    canonical = protocol_with_hash(
        validate_canonical_protocol(CANONICAL_PROTOCOL_PATH)
    )
    if protocol != canonical:
        raise ManifestError("published protocol does not match canonical protocol")
    return protocol, manifest


def load_table1_cohort(
    dataset_root: Path, table1_manifest: Path, *, formal: bool
) -> dict[str, Any]:
    dataset_root = dataset_root.resolve(strict=True)
    table1_manifest = table1_manifest.resolve(strict=True)
    if formal and sha256_file(table1_manifest) != FORMAL_TABLE1_MANIFEST_SHA256:
        raise ManifestError("formal Table 1 manifest hash mismatch")
    try:
        cohort = json.loads(table1_manifest.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise ManifestError(f"invalid Table 1 manifest JSON: {error}") from error

    expected = {
        "dataset": DATASET_NAME,
        "selection_protocol": SELECTION_PROTOCOL,
        "seed": FORMAL_SEED,
        "N_release": FORMAL_N_RELEASE,
        "N_eval": FORMAL_N_EVAL,
        "release_manifest_sha256": FORMAL_RELEASE_MANIFEST_SHA256,
        "release_universe_sha256": FORMAL_RELEASE_UNIVERSE_SHA256,
    }
    if formal:
        for field, required in expected.items():
            if cohort.get(field) != required:
                raise ManifestError(f"formal Table 1 {field} mismatch")

    assets = cohort.get("assets")
    if not isinstance(assets, list) or not assets:
        raise ManifestError("Table 1 assets must be a non-empty list")
    if formal and len(assets) != FORMAL_N_EVAL:
        raise ManifestError("formal Table 1 asset count mismatch")

    seen: set[str] = set()
    for index, asset in enumerate(assets):
        if not isinstance(asset, dict):
            raise ManifestError(f"Table 1 asset {index} must be an object")
        asset_id = asset.get("asset_id")
        if not isinstance(asset_id, str) or not asset_id.startswith("data/"):
            raise ManifestError(f"invalid Table 1 asset_id at index {index}")
        if asset_id in seen:
            raise ManifestError(f"duplicate Table 1 asset_id: {asset_id}")
        seen.add(asset_id)
        if asset.get("selection_rank") != index + 1:
            raise ManifestError(f"selection rank mismatch for {asset_id}")
        package = (dataset_root / asset_id).resolve(strict=True)
        try:
            package.relative_to(dataset_root)
        except ValueError as error:
            raise ManifestError(f"asset escapes dataset root: {asset_id}") from error
        urdf = package / "mobility.urdf"
        if not urdf.is_file():
            raise ManifestError(f"missing mobility.urdf: {asset_id}")
        expected_urdf = asset.get("mobility_urdf_sha256")
        if not isinstance(expected_urdf, str) or len(expected_urdf) != 64:
            raise ManifestError(f"invalid mobility URDF hash: {asset_id}")

    return {"cohort": cohort, "assets": assets, "dataset_root": dataset_root}


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, 1):
            if not line.strip():
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError as error:
                raise ManifestError(
                    f"invalid JSONL record at {path}:{line_number}: {error}"
                ) from error
            if not isinstance(record, dict):
                raise ManifestError(f"non-object JSONL record at {path}:{line_number}")
            records.append(record)
    return records


def load_upstream_records(
    upstream_roots: Mapping[str, Path], asset_ids: list[str]
) -> dict[str, dict[str, dict[str, Any]]]:
    if set(upstream_roots) != {"table2", "table3", "table4"}:
        raise ManifestError("upstream roots must contain Table 2, Table 3, and Table 4")
    if len(asset_ids) != len(set(asset_ids)):
        raise ManifestError("frozen asset IDs must be unique")
    expected = set(asset_ids)
    joined: dict[str, dict[str, dict[str, Any]]] = {}
    for name in ("table2", "table3", "table4"):
        root = Path(upstream_roots[name]).resolve(strict=True)
        records = _read_jsonl(root / "asset_records.jsonl")
        by_id: dict[str, dict[str, Any]] = {}
        key = "dataset_id" if name == "table4" else "asset_id"
        for index, record in enumerate(records):
            asset_id = record.get(key)
            if not isinstance(asset_id, str) or not asset_id:
                raise ManifestError(f"{name} record {index} has invalid {key}")
            if asset_id in by_id:
                raise ManifestError(f"duplicate {name} asset ID: {asset_id}")
            by_id[asset_id] = record
        if set(by_id) != expected:
            missing = sorted(expected - set(by_id))[:3]
            extra = sorted(set(by_id) - expected)[:3]
            raise ManifestError(
                f"{name} cohort mismatch: missing={missing!r}, extra={extra!r}"
            )
        joined[name] = by_id
    return joined


def _strict_gate_evidence(
    table2: Mapping[str, Any],
    table3: Mapping[str, Any],
    table4: Mapping[str, Any],
) -> dict[str, Any]:
    metrics = table2.get("metrics")
    if not isinstance(metrics, dict):
        raise ManifestError("Table 2 metrics are missing")
    subgates: dict[str, bool] = {}
    for name in (
        "parse_rate",
        "resource_resolution",
        "finite_fields",
        "valid_tree",
        "valid_joint_spec",
        "collision_coverage",
        "inertial_coverage",
        "inertia_validity",
    ):
        metric = metrics.get(name)
        if not isinstance(metric, dict) or not isinstance(metric.get("pass"), bool):
            raise ManifestError(f"Table 2 gate is malformed: {name}")
        subgates[name] = metric["pass"]
    required_bools = (
        (table2, "strict_urdf_pass", "Table 2"),
        (table3, "strict_kinematic_pass", "Table 3"),
        (table3, "tree_valid", "Table 3"),
        (table4, "strict_collision_pass", "Table 4"),
        (table4, "load_success", "Table 4"),
        (table4, "measurement_complete", "Table 4"),
    )
    for record, field, label in required_bools:
        if not isinstance(record.get(field), bool):
            raise ManifestError(f"{label} {field} is not boolean")
    return {
        "table2": {
            "strict_urdf_pass": table2["strict_urdf_pass"],
            "subgates": subgates,
            "record_sha256": canonical_sha256(table2),
        },
        "table3": {
            "strict_kinematic_pass": table3["strict_kinematic_pass"],
            "tree_valid": table3["tree_valid"],
            "record_sha256": canonical_sha256(table3),
        },
        "table4": {
            "strict_collision_pass": table4["strict_collision_pass"],
            "load_success": table4["load_success"],
            "measurement_complete": table4["measurement_complete"],
            "record_sha256": canonical_sha256(table4),
        },
    }


def build_manifest_row(
    dataset_root: Path,
    identity: Mapping[str, Any],
    upstream: Mapping[str, Mapping[str, Any]],
    *,
    order: int,
) -> dict[str, Any]:
    if set(upstream) != {"table2", "table3", "table4"}:
        raise ManifestError("manifest row requires Table 2, Table 3, and Table 4")
    asset_id = identity.get("asset_id")
    if not isinstance(asset_id, str) or not asset_id.startswith("data/"):
        raise ManifestError("invalid SketchMobility asset identity")
    if identity.get("selection_rank") != order + 1:
        raise ManifestError(f"selection order mismatch for {asset_id}")
    selection_hash = identity.get("selection_hash")
    if not isinstance(selection_hash, str) or len(selection_hash) != 64:
        raise ManifestError(f"invalid selection hash for {asset_id}")

    table2, table3, table4 = (
        upstream["table2"],
        upstream["table3"],
        upstream["table4"],
    )
    if table2.get("asset_id") != asset_id:
        raise ManifestError(f"Table 2 identity mismatch for {asset_id}")
    if table3.get("asset_id") != asset_id:
        raise ManifestError(f"Table 3 identity mismatch for {asset_id}")
    if table4.get("dataset_id") != asset_id:
        raise ManifestError(f"Table 4 identity mismatch for {asset_id}")
    if table3.get("selection_rank") != order + 1:
        raise ManifestError(f"Table 3 rank mismatch for {asset_id}")
    if table4.get("order") != order or table4.get("selection_rank") != order + 1:
        raise ManifestError(f"Table 4 order mismatch for {asset_id}")

    dataset = Path(dataset_root).resolve(strict=True)
    package = (dataset / asset_id).resolve(strict=True)
    urdf = package / "mobility.urdf"
    binding = package_binding(package)
    package_hash = binding["content_manifest_sha256"]
    for name, record in upstream.items():
        if record.get("package_content_manifest_sha256") != package_hash:
            raise ManifestError(f"{name} package binding mismatch for {asset_id}")
    urdf_hash = sha256_file(urdf)
    if identity.get("mobility_urdf_sha256") != urdf_hash:
        raise ManifestError(f"Table 1 URDF binding mismatch for {asset_id}")
    if table2.get("primary_urdf_sha256") != urdf_hash:
        raise ManifestError(f"Table 2 URDF binding mismatch for {asset_id}")
    if table3.get("urdf_sha256") != urdf_hash:
        raise ManifestError(f"Table 3 URDF binding mismatch for {asset_id}")

    metadata = parse_urdf_metadata(package, urdf)
    bbox = table4.get("object_bbox_diagonal_m")
    bbox_available = (
        isinstance(bbox, (int, float))
        and not isinstance(bbox, bool)
        and math.isfinite(float(bbox))
        and float(bbox) > 0
    )
    issues: list[str] = []
    if metadata["joint_tree"] is None:
        issues.append("invalid_joint_graph")
    if not bbox_available:
        issues.append("missing_bounding_box")
    raw_category = table3.get("raw_category")
    if not isinstance(raw_category, str) or not raw_category:
        raise ManifestError(f"missing category for {asset_id}")
    row = {
        "dataset_id": f"sketch_{order:04d}",
        "order": order,
        "asset_id": asset_id,
        "manifest_root": asset_id,
        "raw_category": raw_category,
        "category": raw_category,
        "source": identity.get("source"),
        "selection_hash": selection_hash,
        "selection_rank": order + 1,
        "package_relative_path": asset_id,
        "package_binding": binding,
        "package_content_manifest_sha256": package_hash,
        "urdf_relative_path": f"{asset_id}/mobility.urdf",
        "urdf_sha256": urdf_hash,
        **metadata,
        "bounding_box": {
            "status": "available" if bbox_available else "not_available",
            "diagonal_m": float(bbox) if bbox_available else None,
            "protocol": "pybullet_q0_collision_shape_union_aabb_v1",
        },
        "bounding_box_diagonal": float(bbox) if bbox_available else None,
        "strict_gates": _strict_gate_evidence(table2, table3, table4),
        "preflight": {
            "status": "pass" if not issues else "failed",
            "issues": issues,
            "simulator_eligible": not issues,
        },
    }
    row["row_sha256"] = canonical_sha256(row, exclude_fields={"row_sha256"})
    return row


def build_manifest(
    dataset_root: Path,
    table1_manifest: Path,
    upstream_roots: Mapping[str, Path],
    *,
    protocol: dict[str, Any],
    formal: bool = True,
) -> dict[str, Any]:
    if formal:
        validate_protocol_schema(protocol)
    loaded = load_table1_cohort(dataset_root, table1_manifest, formal=formal)
    assets = loaded["assets"]
    asset_ids = [row["asset_id"] for row in assets]
    upstream = load_upstream_records(upstream_roots, asset_ids)
    artifact_sets = {
        name: collect_artifact_set(Path(upstream_roots[name]))
        for name in ("table2", "table3", "table4")
    }
    rows: list[dict[str, Any]] = []
    package_meta: list[dict[str, str]] = []
    urdf_meta: list[dict[str, str]] = []
    for order, identity in enumerate(assets):
        asset_id = identity["asset_id"]
        row = build_manifest_row(
            Path(loaded["dataset_root"]),
            identity,
            {name: records[asset_id] for name, records in upstream.items()},
            order=order,
        )
        rows.append(row)
        package_meta.append(
            {
                "manifest_root": asset_id,
                "package_content_manifest_sha256": row[
                    "package_content_manifest_sha256"
                ],
            }
        )
        urdf_meta.append(
            {
                "manifest_root": asset_id,
                "urdf_relpath": row["urdf_relative_path"],
                "sha256": row["urdf_sha256"],
            }
        )

    ordered_identity_sha256 = canonical_sha256(
        [
            {
                "asset_id": row["asset_id"],
                "selection_hash": row["selection_hash"],
                "selection_rank": row["selection_rank"],
            }
            for row in rows
        ]
    )
    ordered_manifest_root_sha256 = canonical_sha256(
        [row["manifest_root"] for row in rows]
    )
    ordered_package_hash = canonical_sha256(package_meta)
    ordered_urdf_hash = canonical_sha256(urdf_meta)
    protocol_receipt = protocol_with_hash(protocol)
    preflight_failures = sum(
        row["preflight"]["status"] == "failed" for row in rows
    )
    manifest = {
        "schema_version": "table5_sketch_mobility_manifest_v1",
        "source_receipt": {
            "dataset": DATASET_NAME,
            "dataset_root": str(loaded["dataset_root"]),
            "release_status": loaded["cohort"].get("release_status"),
            "N_release": loaded["cohort"].get("N_release"),
            "N_eval": loaded["cohort"].get("N_eval"),
            "table1_manifest_path": str(Path(table1_manifest).resolve(strict=True)),
            "table1_manifest_sha256": sha256_file(
                Path(table1_manifest).resolve(strict=True)
            ),
            "release_manifest_path": loaded["cohort"].get("release_manifest"),
            "release_manifest_sha256": loaded["cohort"].get(
                "release_manifest_sha256"
            ),
            "release_universe_sha256": loaded["cohort"].get(
                "release_universe_sha256"
            ),
        },
        "selection": {
            "source": "Table 1 manifest assets exact stored order",
            "candidate_count": len(assets),
            "selected_count": len(rows),
            "identity_authority": "manifest_root",
            "internal_id": "sketch_<zero_based_order:04d>",
            "ordered_identity_sha256": ordered_identity_sha256,
            "ordered_manifest_root_sha256": ordered_manifest_root_sha256,
            "ordered_package_binding_sha256": ordered_package_hash,
            "ordered_urdf_binding_sha256": ordered_urdf_hash,
            "replacement": "never",
            "outcome_filtering": False,
            "retained_preflight_failures": preflight_failures,
        },
        "upstream_artifacts": artifact_sets,
        "protocol_sha256": protocol_receipt["protocol_sha256"],
        "rows": rows,
    }
    manifest["cohort_sha256"] = canonical_sha256(
        manifest, exclude_fields={"cohort_sha256", "generated_at"}
    )
    if formal:
        validate_formal_manifest_constants(manifest)
    return manifest


def validate_formal_manifest_constants(manifest: Mapping[str, Any]) -> None:
    rows = manifest.get("rows")
    if not isinstance(rows, list) or len(rows) != FORMAL_N_EVAL:
        raise ManifestError("formal manifest row count mismatch")
    source = manifest.get("source_receipt")
    if not isinstance(source, Mapping):
        raise ManifestError("formal manifest source receipt is missing")
    source_expected = {
        "dataset": DATASET_NAME,
        "N_release": FORMAL_N_RELEASE,
        "N_eval": FORMAL_N_EVAL,
        "table1_manifest_sha256": FORMAL_TABLE1_MANIFEST_SHA256,
        "release_manifest_sha256": FORMAL_RELEASE_MANIFEST_SHA256,
        "release_universe_sha256": FORMAL_RELEASE_UNIVERSE_SHA256,
    }
    for field, required in source_expected.items():
        if source.get(field) != required:
            raise ManifestError(f"formal source receipt mismatch: {field}")

    selection = manifest.get("selection")
    if not isinstance(selection, Mapping):
        raise ManifestError("formal manifest selection receipt is missing")
    selection_expected = {
        "candidate_count": FORMAL_N_EVAL,
        "selected_count": FORMAL_N_EVAL,
        "identity_authority": "manifest_root",
        "internal_id": "sketch_<zero_based_order:04d>",
        "ordered_identity_sha256": FORMAL_ORDERED_IDENTITY_SHA256,
        "ordered_manifest_root_sha256": FORMAL_ORDERED_MANIFEST_ROOT_SHA256,
        "ordered_package_binding_sha256": FORMAL_ORDERED_PACKAGE_BINDING_SHA256,
        "ordered_urdf_binding_sha256": FORMAL_ORDERED_URDF_BINDING_SHA256,
        "replacement": "never",
        "outcome_filtering": False,
        "retained_preflight_failures": FORMAL_PREFLIGHT_FAILURES,
    }
    for field, required in selection_expected.items():
        if selection.get(field) != required:
            raise ManifestError(f"formal selection mismatch: {field}")

    artifacts = manifest.get("upstream_artifacts")
    if not isinstance(artifacts, Mapping):
        raise ManifestError("formal upstream artifact receipts are missing")
    for name, expected in FORMAL_UPSTREAM_ARTIFACT_SETS.items():
        artifact = artifacts.get(name)
        if not isinstance(artifact, Mapping):
            raise ManifestError(f"formal upstream artifact is missing: {name}")
        for field, required in expected.items():
            if artifact.get(field) != required:
                raise ManifestError(
                    f"formal upstream {name} artifact mismatch: {field}"
                )

    if len({row.get("dataset_id") for row in rows}) != FORMAL_N_EVAL:
        raise ManifestError("formal runtime IDs are not unique")
    for order, row in enumerate(rows):
        if row.get("dataset_id") != f"sketch_{order:04d}":
            raise ManifestError(f"formal runtime ID/order mismatch at {order}")
        if row.get("selection_rank") != order + 1:
            raise ManifestError(f"formal selection rank mismatch at {order}")
        if row.get("asset_id") != row.get("manifest_root"):
            raise ManifestError(f"formal identity authority mismatch at {order}")
    if len({row.get("raw_category") for row in rows}) != FORMAL_CATEGORY_COUNT:
        raise ManifestError("formal category count mismatch")
    if sum(row["package_binding"]["file_count"] for row in rows) != FORMAL_PACKAGE_FILE_COUNT:
        raise ManifestError("formal package file count mismatch")
    if sum(row["package_binding"]["total_bytes"] for row in rows) != FORMAL_PACKAGE_TOTAL_BYTES:
        raise ManifestError("formal package byte count mismatch")
    if sum(not row["preflight"]["simulator_eligible"] for row in rows) != FORMAL_PREFLIGHT_FAILURES:
        raise ManifestError("formal preflight failure count mismatch")


def validate_manifest(
    manifest: dict[str, Any],
    dataset_root: Path,
    table1_manifest: Path,
    upstream_roots: Mapping[str, Path],
    *,
    protocol: dict[str, Any],
    formal: bool = True,
) -> None:
    if not isinstance(manifest, dict):
        raise ManifestError("manifest must be an object")
    if manifest.get("schema_version") != "table5_sketch_mobility_manifest_v1":
        raise ManifestError("manifest schema mismatch")
    validate_protocol_schema(protocol)
    protocol_receipt = protocol_with_hash(protocol)
    if manifest.get("protocol_sha256") != protocol_receipt["protocol_sha256"]:
        raise ManifestError("manifest protocol binding mismatch")
    if formal:
        validate_formal_manifest_constants(manifest)
    expected = build_manifest(
        dataset_root,
        table1_manifest,
        upstream_roots,
        protocol=protocol,
        formal=formal,
    )
    if manifest != expected:
        raise ManifestError("manifest differs from a fresh source-bound rebuild")
