"""Frozen source and receipt helpers for Artiverse Table 5 N=800."""

from __future__ import annotations

import hashlib
import json
import math
import os
import stat
import tempfile
import xml.etree.ElementTree as ET
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Iterable, Mapping

DATASET_NAME = "Ours-500K"
SELECTION_PROTOCOL = "ours500k-table1-roster-v1"
FORMAL_N_RELEASE = 500
FORMAL_N_EVAL = 500
FORMAL_CATEGORY_COUNT = 12
FORMAL_TABLE1_MANIFEST_SHA256 = (
    "bc3eb334b1fc1c57378e50e7c2fab5d765a7599db8b3e82bc1d91536570b7c06"
)
FORMAL_TABLE1_ASSET_RECORDS_SHA256 = (
    "930951dd083ad91865388213e04c03fa2acdee5ca086411380e87be5225a64fe"
)
FORMAL_ROSTER_SHA256 = (
    "ed70ebeb97f9ad8a655288e2afce96b0c3a8e26f50653e50dbbdc00238cfea3b"
)
FORMAL_ARCHIVE_SHA256 = (
    "ffedf5bd90ae5eb96a061d0e127b700915ed6c221eeb7c5afe282b7249bfbd66"
)
FORMAL_ARCHIVE_FILENAME = "arti_cabinet_drawer_geometry_500_20260813.zip"
FORMAL_ORDERED_IDENTITY_SHA256 = (
    "4c0596407f2e9ecf0c09a205d3c80019ba7b3ddcb90d54f85c589990f85aad47"
)
FORMAL_ORDERED_MANIFEST_ROOT_SHA256 = (
    "dcd19530ff3a3546fa149db58f331a042d5c3326f1b4fa1e5580914952e79289"
)
FORMAL_ORDERED_PACKAGE_BINDING_SHA256 = (
    "63a6770b1fc9f98db49ccc1ee1116155b1a726322fdfbac398e1f2fd35bead20"
)
FORMAL_ORDERED_URDF_BINDING_SHA256 = (
    "e73e417876c691e98f1111e6e087cba4a7c2d72235649f4ddd0b142427cef917"
)
FORMAL_PACKAGE_FILE_COUNT = 15662
FORMAL_PACKAGE_TOTAL_BYTES = 8842166963
FORMAL_UPSTREAM_PINS = {
    "table2": {
        "artifact_set": {
            "artifact_set_sha256": "80687caf3f0b90e377a035bdbb90ffaa034016874c6e936b36ee0452507346cb",
            "file_count": 7,
            "total_bytes": 2967785,
        },
        "required_files": [
            "artifact_manifest.json",
            "asset_records.jsonl",
            "environment.json",
            "manifest.json",
            "protocol_snapshot.md",
            "summary.json",
            "summary.md",
        ],
        "summary_expectations": {
            "classification": "FORMAL",
            "dataset": "Ours-500K",
            "n_eval": 500,
            "records_present": 500,
            "error_count": 0,
            "status_counts": {"completed": 500},
            "strict_passed": 4,
            "metric_pass_counts": {
                "collision_coverage": 500,
                "finite_fields": 500,
                "inertia_validity": 4,
                "inertial_coverage": 4,
                "parse_rate": 500,
                "resource_resolution": 500,
                "strict_urdf_pass": 4,
                "valid_joint_spec": 500,
                "valid_tree": 500,
            },
        },
    },
    "table3": {
        "artifact_set": {
            "artifact_set_sha256": "c8c4b589ddc0b049b15ca4bbc631d7a6a333d4b578aba3c0ca8cf0073c110dd5",
            "file_count": 1014,
            "total_bytes": 18198324,
        },
        "required_files": [
            "asset_records.env_error_pre_retry.jsonl",
            "asset_records.jsonl",
            "checkpoint.json",
            "manifest.json",
            "summary.json",
            "summary.md",
        ],
        "summary_expectations": {
            "schema_version": 1,
            "status": "completed",
            "classification": "FORMAL",
            "dataset": "Ours-500K",
            "n_eval": 500,
            "j_eval": 2467,
            "parse_success": 500,
            "valid_tree": 500,
            "status_counts": {"completed": 500},
            "strict_passed": 500,
        },
    },
    "table4": {
        "artifact_set": {
            "artifact_set_sha256": "2fd4a911675a32292bfbfae2be1ceed98ce86ff8b8c410b6d81565dcee896046",
            "file_count": 8,
            "total_bytes": 174988498,
        },
        "required_files": [
            "asset_records.jsonl",
            "checkpoint.json",
            "frozen_manifest.json",
            "protocol_document_at_freeze.md",
            "report.md",
            "state_records.jsonl",
            "summary.json",
            "verification.json",
        ],
        "summary_expectations": {
            "status": "COMPLETE",
            "protocol_id": "urdf_sim_ready_table4_ours_500k_table2_n500_v1",
            "selected": 500,
            "category_count": 12,
            "load_success": 500,
            "measurement_complete": 500,
            "strict_passed": 485,
            "verification_status": "PASS",
        },
    },
}
FORMAL_INVALID_GRAPH_ROOTS: frozenset[str] = frozenset()
IDENTITY_FIELDS = (
    "asset_id",
    "manifest_root",
    "raw_category",
    "seed_name",
    "selection_rank",
)
TABLE2_GATE_FIELDS = (
    "parse_rate",
    "resource_resolution",
    "finite_fields",
    "valid_tree",
    "valid_joint_spec",
    "collision_coverage",
    "inertial_coverage",
    "inertia_validity",
)
CANONICAL_PROTOCOL_PATH = (
    Path(__file__).parents[1] / "reference" / "table5_ours_n500_protocol_v1.json"
)
# File-byte pin. Update only with a reviewed canonical protocol change.
CANONICAL_PROTOCOL_FILE_SHA256 = (
    "a41d094f5130a6c0310ac7f68cc3471fd0abbaaa65b33a369e0ab664a7c679c4"
)
EXPECTED_GENESIS_GPU_BINDING = {
    "cuda_visible_devices": "0",
    "physical_device_index": 0,
    "visible_device_index": 0,
    "gpu_uuid": "GPU-3a784765-2b05-547f-1508-2a5ea43e3d27",
}
EXPECTED_RESUME_BINDING = (
    "schema_version",
    "run_phase",
    "simulator",
    "adapter_name",
    "adapter_version",
    "adapter_config_sha256",
    "effective_workers",
    "dataset_id",
    "manifest_root",
    "manifest_row_sha256",
    "urdf_sha256",
    "package_content_manifest_sha256",
    "protocol_sha256",
    "cohort_sha256",
)


class ManifestError(ValueError):
    """Raised when a frozen input or receipt does not validate."""


def canonical_json_bytes(value: Any, *, exclude_fields: Iterable[str] = ()) -> bytes:
    excluded = set(exclude_fields)

    def filtered(item: Any) -> Any:
        if isinstance(item, dict):
            return {
                key: filtered(val) for key, val in item.items() if key not in excluded
            }
        if isinstance(item, list):
            return [filtered(val) for val in item]
        return item

    return json.dumps(
        filtered(value),
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")


def canonical_sha256(value: Any, *, exclude_fields: Iterable[str] = ()) -> str:
    return hashlib.sha256(
        canonical_json_bytes(value, exclude_fields=exclude_fields)
    ).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(4 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def atomic_write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    try:
        with os.fdopen(descriptor, "wb") as handle:
            handle.write(canonical_json_bytes(value))
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def _require_fields(
    value: Mapping[str, Any], fields: Iterable[str], location: str
) -> None:
    for field in fields:
        if field not in value:
            raise ManifestError(f"protocol missing required field: {location}.{field}")


def validate_protocol_schema(protocol: dict[str, Any]) -> None:
    _require_fields(
        protocol,
        (
            "schema_version",
            "claim_boundary",
            "source",
            "selection",
            "upstream_strict_gates",
            "runtime",
            "metrics",
            "cross_simulator",
            "adapters",
            "artifacts",
        ),
        "protocol",
    )
    _require_fields(protocol["adapters"], ("pybullet", "mujoco", "genesis"), "adapters")
    _require_fields(
        protocol["adapters"]["genesis"],
        (
            "version",
            "backend",
            "gpu_binding",
            "precision",
            "substeps",
            "collision",
            "self_collision",
            "enable_neutral_collision",
            "contacts",
            "visualization",
            "fixed",
            "fixed_link_merge",
            "recompute_inertia",
            "requires_jac_and_IK",
            "logging",
            "control",
        ),
        "adapters.genesis",
    )
    expected = {
        "schema_version": "table5_ours_protocol_v1",
        "source.table1_manifest_sha256": FORMAL_TABLE1_MANIFEST_SHA256,
        "source.archive_sha256": FORMAL_ARCHIVE_SHA256,
        "source.roster_sha256": FORMAL_ROSTER_SHA256,
        "source.release_manifest_sha256": FORMAL_ARCHIVE_SHA256,
        "source.release_universe_sha256": FORMAL_ROSTER_SHA256,
        "source.ordered_identity_sha256": FORMAL_ORDERED_IDENTITY_SHA256,
        "source.ordered_manifest_root_sha256": FORMAL_ORDERED_MANIFEST_ROOT_SHA256,
        "source.ordered_package_binding_sha256": FORMAL_ORDERED_PACKAGE_BINDING_SHA256,
        "source.ordered_urdf_binding_sha256": FORMAL_ORDERED_URDF_BINDING_SHA256,
        "upstream_strict_gates.table2.artifact_set": FORMAL_UPSTREAM_PINS["table2"][
            "artifact_set"
        ],
        "upstream_strict_gates.table2.required_files": FORMAL_UPSTREAM_PINS["table2"][
            "required_files"
        ],
        "upstream_strict_gates.table2.summary_expectations": FORMAL_UPSTREAM_PINS[
            "table2"
        ]["summary_expectations"],
        "upstream_strict_gates.table3.artifact_set": FORMAL_UPSTREAM_PINS["table3"][
            "artifact_set"
        ],
        "upstream_strict_gates.table3.required_files": FORMAL_UPSTREAM_PINS["table3"][
            "required_files"
        ],
        "upstream_strict_gates.table3.summary_expectations": FORMAL_UPSTREAM_PINS[
            "table3"
        ]["summary_expectations"],
        "upstream_strict_gates.table4.artifact_set": FORMAL_UPSTREAM_PINS["table4"][
            "artifact_set"
        ],
        "upstream_strict_gates.table4.required_files": FORMAL_UPSTREAM_PINS["table4"][
            "required_files"
        ],
        "upstream_strict_gates.table4.summary_expectations": FORMAL_UPSTREAM_PINS[
            "table4"
        ]["summary_expectations"],
        "selection.identity_authority": "manifest_root",
        "selection.selected_count": 500,
        "selection.replacement": "never",
        "selection.retained_preflight_failures": 0,
        "runtime.base": "fixed",
        "runtime.contacts": "enabled",
        "runtime.timestep_s": {"numerator": 1, "denominator": 240},
        "runtime.actuation.effort_controller.formula": "tau = effort * clip(kp * normalized_position_error - kd * normalized_speed, -1, 1)",
        "runtime.actuation.effort_controller.kp": 2.0,
        "runtime.actuation.effort_controller.kd": 0.2,
        "runtime.actuation.effort_controller.clip": [-1.0, 1.0],
        "cross_simulator.pairing.asset_key": "manifest.manifest_root",
        "cross_simulator.pairing.runtime_asset_key": "manifest.dataset_id",
        "cross_simulator.pairing.joint_key": "urdf_joint_name",
        "cross_simulator.trajectory_samples": 31,
        "cross_simulator.sample_cadence_steps": 12,
        "cross_simulator.thresholds.normalized_joint_rmse": 0.1,
        "cross_simulator.thresholds.translation_over_bbox_diagonal": 0.02,
        "cross_simulator.thresholds.rotation_rad": 0.1,
        "cross_simulator.all_three_denominator": 500,
        "adapters.pybullet.ignore_collision": False,
        "adapters.pybullet.self_collision": True,
        "adapters.pybullet.contacts": "enabled",
        "adapters.mujoco.contacts": "enabled",
        "adapters.mujoco.self_collision": True,
        "adapters.genesis.backend": "cuda",
        "adapters.genesis.gpu_binding.cuda_visible_devices": EXPECTED_GENESIS_GPU_BINDING[
            "cuda_visible_devices"
        ],
        "adapters.genesis.gpu_binding.physical_device_index": EXPECTED_GENESIS_GPU_BINDING[
            "physical_device_index"
        ],
        "adapters.genesis.gpu_binding.visible_device_index": EXPECTED_GENESIS_GPU_BINDING[
            "visible_device_index"
        ],
        "adapters.genesis.gpu_binding.gpu_uuid": EXPECTED_GENESIS_GPU_BINDING[
            "gpu_uuid"
        ],
        "adapters.genesis.precision": "float32",
        "adapters.genesis.collision": True,
        "adapters.genesis.self_collision": True,
        "adapters.genesis.enable_neutral_collision": True,
        "adapters.genesis.contacts": "enabled",
        "adapters.genesis.visualization": False,
        "artifacts.resume_binding": list(EXPECTED_RESUME_BINDING),
    }
    for dotted_path, required in expected.items():
        current: Any = protocol
        for segment in dotted_path.split("."):
            if not isinstance(current, dict) or segment not in current:
                raise ManifestError(f"protocol semantic mismatch: {dotted_path}")
            current = current[segment]
        if current != required:
            raise ManifestError(f"protocol semantic mismatch: {dotted_path}")


def validate_canonical_protocol(protocol_path: Path) -> dict[str, Any]:
    resolved = protocol_path.resolve()
    if resolved != CANONICAL_PROTOCOL_PATH.resolve():
        raise ManifestError(
            f"canonical protocol path required: {CANONICAL_PROTOCOL_PATH}"
        )
    if sha256_file(resolved) != CANONICAL_PROTOCOL_FILE_SHA256:
        raise ManifestError("canonical protocol file hash mismatch")
    try:
        protocol = json.loads(resolved.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise ManifestError(f"invalid canonical protocol JSON: {error}") from error
    validate_protocol_schema(protocol)
    return protocol


def protocol_with_hash(protocol: dict[str, Any]) -> dict[str, Any]:
    receipt = dict(protocol)
    receipt["protocol_sha256"] = canonical_sha256(
        receipt, exclude_fields={"protocol_sha256", "generated_at"}
    )
    return receipt


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


def validate_output_path(
    dataset_root: Path,
    upstream_roots: Iterable[Path],
    output_root: Path,
    *,
    table1_manifest: Path | None = None,
) -> None:
    output = output_root.resolve()
    sources = [dataset_root.resolve(), *(path.resolve() for path in upstream_roots)]
    for source in sources:
        if output == source or source in output.parents or output in source.parents:
            raise ManifestError("output path overlaps source or upstream artifacts")
    if table1_manifest is not None:
        table1_artifact_root = table1_manifest.resolve(strict=True).parent
        if (
            output == table1_artifact_root
            or table1_artifact_root in output.parents
            or output in table1_artifact_root.parents
        ):
            raise ManifestError("output path overlaps the Table 1 artifact root")


@contextmanager
def output_lock(output_root: Path, *, require_empty: bool = False):
    import fcntl

    try:
        output_root.mkdir(parents=True, exist_ok=False)
        created_output = True
    except FileExistsError:
        created_output = False
        if not output_root.is_dir():
            raise ManifestError("output path exists and is not a directory")
    lock_path = output_root / ".prepare.lock"
    if os.path.lexists(lock_path) and (
        lock_path.is_symlink() or not lock_path.is_file()
    ):
        raise ManifestError("output .prepare.lock must be a regular file")
    handle = lock_path.open("a+", encoding="utf-8")
    try:
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise ManifestError(f"output lock is already held: {lock_path}") from error
        if require_empty:
            unexpected = sorted(
                path.name
                for path in output_root.iterdir()
                if path.name != lock_path.name
            )
            if unexpected:
                raise ManifestError(
                    "output directory must be empty except .prepare.lock: "
                    f"{unexpected!r}"
                )
        handle.seek(0)
        handle.truncate()
        lock_contents = f"pid={os.getpid()}\n"
        handle.write(lock_contents)
        handle.flush()
        root_stat = output_root.stat()
        lock_stat = os.fstat(handle.fileno())
        yield {
            "created_output": created_output,
            "output_root": str(output_root.resolve(strict=True)),
            "output_dev": root_stat.st_dev,
            "output_ino": root_stat.st_ino,
            "lock_dev": lock_stat.st_dev,
            "lock_ino": lock_stat.st_ino,
            "lock_contents": lock_contents,
        }
    finally:
        fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
        handle.close()


def cleanup_new_prepare_output(
    output_root: Path, lock_receipt: Mapping[str, Any]
) -> bool:
    """Remove only an unchanged output directory created by this prepare call."""
    try:
        if output_root.is_symlink():
            return False
        resolved = output_root.resolve(strict=True)
        if str(resolved) != lock_receipt.get("output_root"):
            return False
        root_stat = resolved.stat(follow_symlinks=False)
        if (
            not stat.S_ISDIR(root_stat.st_mode)
            or root_stat.st_dev != lock_receipt.get("output_dev")
            or root_stat.st_ino != lock_receipt.get("output_ino")
        ):
            return False
        entries = list(resolved.iterdir())
        if len(entries) != 1 or entries[0].name != ".prepare.lock":
            return False
        lock_path = entries[0]
        lock_stat = lock_path.stat(follow_symlinks=False)
        if (
            not stat.S_ISREG(lock_stat.st_mode)
            or lock_stat.st_dev != lock_receipt.get("lock_dev")
            or lock_stat.st_ino != lock_receipt.get("lock_ino")
            or lock_path.read_text(encoding="utf-8")
            != lock_receipt.get("lock_contents")
        ):
            return False
        lock_path.unlink()
        resolved.rmdir()
        return True
    except (FileNotFoundError, OSError, UnicodeError):
        return False


def publish_receipt_set(
    output_root: Path, protocol: dict[str, Any], manifest: dict[str, Any]
) -> None:
    atomic_write_json(output_root / "protocol.json", protocol)
    atomic_write_json(output_root / "manifest.json", manifest)
    atomic_write_json(
        output_root / "receipt_set.json",
        {
            "schema_version": "table5_ours_receipt_set_v1",
            "protocol_sha256": sha256_file(output_root / "protocol.json"),
            "manifest_sha256": sha256_file(output_root / "manifest.json"),
        },
    )


def validate_receipt_set(
    output_root: Path,
) -> tuple[dict[str, Any], dict[str, Any]]:
    marker_path = output_root / "receipt_set.json"
    if not marker_path.is_file():
        raise ManifestError("receipt marker missing")
    try:
        marker = json.loads(marker_path.read_text(encoding="utf-8"))
        protocol_path = output_root / "protocol.json"
        manifest_path = output_root / "manifest.json"
        protocol = json.loads(protocol_path.read_text(encoding="utf-8"))
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ManifestError(f"incomplete receipt set: {error}") from error
    if marker.get("schema_version") != "table5_ours_receipt_set_v1":
        raise ManifestError("receipt marker schema mismatch")
    if marker.get("protocol_sha256") != sha256_file(protocol_path):
        raise ManifestError("receipt marker protocol hash mismatch")
    if marker.get("manifest_sha256") != sha256_file(manifest_path):
        raise ManifestError("receipt marker manifest hash mismatch")
    canonical = protocol_with_hash(validate_canonical_protocol(CANONICAL_PROTOCOL_PATH))
    if protocol != canonical:
        raise ManifestError("published protocol does not match canonical protocol")
    return protocol, manifest


def _safe_relative(raw: str, *, field: str) -> Path:
    value = Path(raw)
    if value.is_absolute() or not value.parts or ".." in value.parts:
        raise ManifestError(f"unsafe {field}: {raw!r}")
    return value


def load_table1_cohort(
    dataset_root: Path, table1_manifest: Path, *, formal: bool = True
) -> dict[str, Any]:
    """Load and validate the frozen Ours-500K Table 1 full-roster receipt."""
    dataset = dataset_root.resolve(strict=True)
    cohort_path = table1_manifest.resolve(strict=True)
    cohort_hash = sha256_file(cohort_path)
    cohort = json.loads(cohort_path.read_text(encoding="utf-8"))
    if cohort.get("schema_version") != "table1_ours_500k_manifest_v1":
        raise ManifestError("Table 1 cohort schema mismatch")
    if cohort.get("dataset") != DATASET_NAME:
        raise ManifestError("Table 1 cohort dataset must be Ours-500K")
    if cohort.get("release_status") != "ACQUIRED_RELEASE_SAMPLE_BRAIN_MODELSCOPE":
        raise ManifestError("Ours-500K cohort release status mismatch")
    if cohort.get("cohort_type") != "FULL_ACQUIRED_RELEASE_SAMPLE_NO_SUBSAMPLING":
        raise ManifestError("unexpected Ours-500K cohort type")
    if cohort.get("roster_sha256") != FORMAL_ROSTER_SHA256:
        raise ManifestError("Ours-500K roster hash mismatch")
    archive_binding = cohort.get("archive_binding")
    if not isinstance(archive_binding, dict):
        raise ManifestError("Ours-500K archive binding missing")
    archive_path = Path(str(archive_binding.get("archive", "")))
    if not archive_path.is_file():
        raise ManifestError(f"acquired archive missing: {archive_path}")
    if archive_path.name != FORMAL_ARCHIVE_FILENAME:
        raise ManifestError("acquired archive filename mismatch")
    archive_hash = sha256_file(archive_path)
    if archive_binding.get("archive_sha256") != archive_hash:
        raise ManifestError("archive binding hash mismatch")
    if archive_hash != FORMAL_ARCHIVE_SHA256:
        raise ManifestError("formal archive hash mismatch")
    records_path = cohort_path.parent / "asset_records.jsonl"
    records_hash = sha256_file(records_path)
    if records_hash != FORMAL_TABLE1_ASSET_RECORDS_SHA256:
        raise ManifestError("Table 1 asset records hash mismatch")
    urdf_hashes: dict[str, str] = {}
    with records_path.open("r", encoding="utf-8") as handle:
        for line in handle:
            record = json.loads(line)
            asset_id = str(record.get("asset_id", ""))
            if record.get("status") != "EVALUATED" or not record.get("primary_urdf_sha256"):
                raise ManifestError(f"Table 1 record not fully evaluated: {asset_id}")
            urdf_hashes[asset_id] = str(record["primary_urdf_sha256"])
    raw_assets = cohort.get("assets")
    if (
        not isinstance(raw_assets, list)
        or cohort.get("N_eval") != len(raw_assets)
        or cohort.get("N_release") != len(raw_assets)
        or len(urdf_hashes) != len(raw_assets)
    ):
        raise ManifestError("Ours-500K cohort denominator mismatch")
    seen_roots: set[str] = set()
    assets: list[dict[str, Any]] = []
    for rank, raw in enumerate(raw_assets, start=1):
        if not isinstance(raw, dict):
            raise ManifestError("Ours-500K cohort asset is not an object")
        asset_id = raw.get("asset_id")
        if not isinstance(asset_id, str) or raw.get("asset_root") != asset_id:
            raise ManifestError("Ours-500K asset_id must equal asset_root")
        if asset_id in seen_roots:
            raise ManifestError(f"duplicate Ours-500K cohort asset: {asset_id}")
        seen_roots.add(asset_id)
        parts = _safe_relative(asset_id, field="asset_id").parts
        if len(parts) != 2 or not parts[0] or not parts[1]:
            raise ManifestError(f"invalid Ours-500K asset_id: {asset_id!r}")
        if (
            raw.get("raw_category") != parts[0]
            or raw.get("seed_name") != parts[1]
            or raw.get("selection_rank") != rank
        ):
            raise ManifestError(f"Ours-500K path/rank metadata mismatch: {asset_id}")
        if raw.get("primary_urdf") != f"{asset_id}/model.urdf":
            raise ManifestError(f"Ours-500K primary URDF path mismatch: {asset_id}")
        expected_hash = urdf_hashes.get(asset_id)
        if expected_hash is None:
            raise ManifestError(f"Ours-500K URDF hash missing from records: {asset_id}")
        declared_hash = raw.get("primary_urdf_sha256")
        if declared_hash is not None and declared_hash != expected_hash:
            raise ManifestError(f"Ours-500K URDF hash binding mismatch: {asset_id}")
        asset_root = dataset / _safe_relative(asset_id, field="asset_id")
        if asset_root.is_symlink() or not asset_root.is_dir():
            raise ManifestError(f"Ours-500K asset root missing or symlinked: {asset_id}")
        package = asset_root.resolve(strict=True)
        try:
            package.relative_to(dataset)
        except ValueError as error:
            raise ManifestError(
                f"Ours-500K asset root escapes dataset root: {asset_id}"
            ) from error
        urdf = package / "model.urdf"
        if urdf.is_symlink() or not urdf.is_file():
            raise ManifestError(f"Ours-500K primary URDF missing: {asset_id}")
        assets.append(
            {
                "asset_id": asset_id,
                "manifest_root": asset_id,
                "raw_category": parts[0],
                "seed_name": parts[1],
                "selection_rank": rank,
                "package": str(package),
                "package_relative_path": package.relative_to(dataset).as_posix(),
                "urdf_path": str(urdf.resolve(strict=True)),
                "urdf_relative_path": urdf.relative_to(dataset).as_posix(),
                "primary_urdf_sha256": expected_hash,
            }
        )
    identities = [{field: row[field] for field in IDENTITY_FIELDS} for row in assets]
    roots = [row["manifest_root"] for row in assets]
    ordered_identity = canonical_sha256(identities)
    ordered_roots = canonical_sha256(roots)
    if formal:
        expected = {
            "Table 1 manifest": (cohort_hash, FORMAL_TABLE1_MANIFEST_SHA256),
            "archive": (archive_hash, FORMAL_ARCHIVE_SHA256),
            "N_release": (len(raw_assets), FORMAL_N_RELEASE),
            "N_eval": (len(assets), FORMAL_N_EVAL),
            "ordered identities": (ordered_identity, FORMAL_ORDERED_IDENTITY_SHA256),
            "ordered manifest roots": (
                ordered_roots,
                FORMAL_ORDERED_MANIFEST_ROOT_SHA256,
            ),
            "raw category count": (
                len({row["raw_category"] for row in assets}),
                FORMAL_CATEGORY_COUNT,
            ),
        }
        for name, (actual, required) in expected.items():
            if actual != required:
                raise ManifestError(
                    f"formal Ours-500K {name} mismatch: {actual!r} != {required!r}"
                )
    return {
        "dataset_root": str(dataset),
        "table1_manifest_path": str(cohort_path),
        "table1_manifest_sha256": cohort_hash,
        "table1_asset_records_sha256": records_hash,
        "release_manifest_path": str(archive_path.resolve(strict=True)),
        "release_manifest_sha256": archive_hash,
        "release_universe_sha256": cohort["roster_sha256"],
        "release_status": cohort["release_status"],
        "cohort_type": cohort["cohort_type"],
        "selection_protocol": SELECTION_PROTOCOL,
        "n_release": len(raw_assets),
        "n_eval": len(assets),
        "identity_fields": list(IDENTITY_FIELDS),
        "ordered_identity_sha256": ordered_identity,
        "ordered_manifest_root_sha256": ordered_roots,
        "assets": assets,
    }


def package_binding(package: Path) -> dict[str, Any]:
    root = package.resolve(strict=True)
    if not root.is_dir():
        raise ManifestError(f"package is not a directory: {root}")
    rows: list[dict[str, Any]] = []
    for current_raw, directory_names, file_names in os.walk(root, followlinks=False):
        current = Path(current_raw)
        directory_names.sort()
        file_names.sort()
        for name in directory_names:
            if (current / name).is_symlink():
                raise ManifestError(
                    f"package contains directory symlink: {(current / name).relative_to(root)}"
                )
        for name in file_names:
            path = current / name
            relative = path.relative_to(root).as_posix()
            if path.is_symlink():
                raise ManifestError(f"package contains file symlink: {relative}")
            canonical = path.resolve(strict=True)
            canonical.relative_to(root)
            if not canonical.is_file():
                raise ManifestError(f"package entry is not a regular file: {relative}")
            rows.append(
                {
                    "path": relative,
                    "bytes": canonical.stat().st_size,
                    "sha256": sha256_file(canonical),
                }
            )
    return {
        "file_count": len(rows),
        "total_bytes": sum(row["bytes"] for row in rows),
        "files": rows,
        "content_manifest_sha256": canonical_sha256(rows),
    }


def collect_artifact_set(root: Path) -> dict[str, Any]:
    artifact_root = root.resolve(strict=True)
    if not artifact_root.is_dir():
        raise ManifestError(f"upstream artifact root is not a directory: {root}")
    files: list[dict[str, Any]] = []
    for current_raw, directory_names, file_names in os.walk(
        artifact_root, followlinks=False
    ):
        current = Path(current_raw)
        directory_names.sort()
        file_names.sort()
        for name in directory_names:
            if (current / name).is_symlink():
                raise ManifestError(
                    f"upstream artifact contains directory symlink: {name}"
                )
        for name in file_names:
            path = current / name
            if path.is_symlink() or not path.is_file():
                raise ManifestError(f"upstream artifact is not a regular file: {path}")
            files.append(
                {
                    "path": path.relative_to(artifact_root).as_posix(),
                    "bytes": path.stat().st_size,
                    "sha256": sha256_file(path),
                }
            )
    return {
        "root": str(artifact_root),
        "file_count": len(files),
        "total_bytes": sum(row["bytes"] for row in files),
        "files": files,
        "artifact_set_sha256": canonical_sha256(files),
    }


def _expect_values(
    actual: Mapping[str, Any],
    expected: Mapping[str, Any],
    location: str,
) -> None:
    for key, required in expected.items():
        if actual.get(key) != required:
            raise ManifestError(
                f"{location}.{key} mismatch: " f"{actual.get(key)!r} != {required!r}"
            )


def _validate_formal_upstream_artifacts(
    artifact_sets: Mapping[str, dict[str, Any]],
    upstream_roots: Mapping[str, Path],
) -> None:
    for name, pin in FORMAL_UPSTREAM_PINS.items():
        artifact_set = artifact_sets[name]
        _expect_values(
            artifact_set,
            pin["artifact_set"],
            f"formal upstream {name} artifact_set",
        )
        paths = {row["path"] for row in artifact_set["files"]}
        missing = set(pin["required_files"]) - paths
        if missing:
            raise ManifestError(
                f"formal upstream {name} required files missing: "
                f"{sorted(missing)!r}"
            )

    table2_summary = json.loads(
        (Path(upstream_roots["table2"]) / "summary.json").read_text(encoding="utf-8")
    )
    table2_pin = FORMAL_UPSTREAM_PINS["table2"]["summary_expectations"]
    _expect_values(
        table2_summary,
        {
            key: table2_pin[key]
            for key in (
                "classification",
                "dataset",
                "n_eval",
                "records_present",
                "error_count",
                "status_counts",
            )
        },
        "formal upstream table2 summary",
    )
    _expect_values(
        table2_summary.get("strict_urdf_pass", {}),
        {"denominator": FORMAL_N_EVAL, "passed": table2_pin["strict_passed"]},
        "formal upstream table2 summary.strict_urdf_pass",
        )
    table2_metrics = table2_summary.get("metrics")
    if not isinstance(table2_metrics, dict):
        raise ManifestError("formal upstream table2 summary.metrics malformed")
    for metric, passed in table2_pin["metric_pass_counts"].items():
        evidence = table2_metrics.get(metric)
        if not isinstance(evidence, dict):
            raise ManifestError(
                f"formal upstream table2 summary metric missing: {metric}"
            )
        _expect_values(
            evidence,
            {"denominator": FORMAL_N_EVAL, "passed": passed},
            f"formal upstream table2 summary.metrics.{metric}",
        )

    table3_summary = json.loads(
        (Path(upstream_roots["table3"]) / "summary.json").read_text(encoding="utf-8")
    )
    table3_pin = FORMAL_UPSTREAM_PINS["table3"]["summary_expectations"]
    _expect_values(
        table3_summary,
        {
            key: table3_pin[key]
            for key in (
                "schema_version",
                "status",
                "classification",
                "dataset",
                "n_eval",
                "j_eval",
                "parse_success",
                "valid_tree",
                "status_counts",
            )
        },
        "formal upstream table3 summary",
    )
    _expect_values(
        table3_summary.get("metrics", {}).get("strict_kinematic_pass", {}),
        {"denominator": FORMAL_N_EVAL, "passed": table3_pin["strict_passed"]},
        "formal upstream table3 summary.metrics.strict_kinematic_pass",
    )

    table4_summary = json.loads(
        (Path(upstream_roots["table4"]) / "summary.json").read_text(encoding="utf-8")
    )
    table4_pin = FORMAL_UPSTREAM_PINS["table4"]["summary_expectations"]
    _expect_values(
        table4_summary,
        {
            "status": table4_pin["status"],
            "protocol_id": table4_pin["protocol_id"],
        },
        "formal upstream table4 summary",
    )
    _expect_values(
        table4_summary.get("cohort", {}),
        {
            key: table4_pin[key]
            for key in (
                "selected",
                "category_count",
                "load_success",
                "measurement_complete",
            )
        },
        "formal upstream table4 summary.cohort",
    )
    _expect_values(
        table4_summary.get("metrics", {}).get("strict_collision_pass", {}),
        {"denominator": FORMAL_N_EVAL, "passed": table4_pin["strict_passed"]},
        "formal upstream table4 summary.metrics.strict_collision_pass",
    )
    verification = json.loads(
        (Path(upstream_roots["table4"]) / "verification.json").read_text(
            encoding="utf-8"
        )
    )
    _expect_values(
        verification,
        {"status": table4_pin["verification_status"]},
        "formal upstream table4 verification",
    )


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            try:
                value = json.loads(line)
            except json.JSONDecodeError as error:
                raise ManifestError(f"invalid JSONL at {path}:{line_number}") from error
            if not isinstance(value, dict):
                raise ManifestError(f"non-object JSONL row at {path}:{line_number}")
            rows.append(value)
    return rows


def _records_by_asset(
    rows: list[dict[str, Any]], name: str, key_field: str
) -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    for row in rows:
        key = row.get(key_field)
        if not isinstance(key, str):
            raise ManifestError(f"{name} record missing {key_field}")
        if key in result:
            raise ManifestError(f"duplicate {name} {key_field}: {key}")
        result[key] = row
    return result


def load_upstream_records(
    upstream_roots: Mapping[str, Path], manifest_roots: list[str]
) -> dict[str, dict[str, dict[str, Any]]]:
    if set(upstream_roots) != {"table2", "table3", "table4"}:
        raise ManifestError("upstream roots must be exactly table2/table3/table4")
    table2 = _records_by_asset(
        _read_jsonl(Path(upstream_roots["table2"]) / "asset_records.jsonl"),
        "Table 2",
        "asset_id",
    )
    table3 = _records_by_asset(
        _read_jsonl(Path(upstream_roots["table3"]) / "asset_records.jsonl"),
        "Table 3",
        "asset_id",
    )
    table4 = _records_by_asset(
        _read_jsonl(Path(upstream_roots["table4"]) / "asset_records.jsonl"),
        "Table 4",
        "dataset_id",
    )
    verification_path = Path(upstream_roots["table4"]) / "verification.json"
    if verification_path.is_file():
        verification = json.loads(verification_path.read_text(encoding="utf-8"))
        if verification.get("status") != "PASS":
            raise ManifestError("Table 4 verification status is not PASS")
    expected = set(manifest_roots)
    for name, records in (
        ("Table 2", table2),
        ("Table 3", table3),
        ("Table 4", table4),
    ):
        if set(records) != expected:
            raise ManifestError(f"{name} record membership mismatch")
    return {"table2": table2, "table3": table3, "table4": table4}


def _float_attribute(element: ET.Element | None, name: str) -> float | None:
    if element is None or element.get(name) is None:
        return None
    try:
        value = float(element.get(name, ""))
    except ValueError:
        return None
    return value if math.isfinite(value) else None


def _vector_attribute(
    element: ET.Element | None,
    name: str,
    default: tuple[float, float, float],
) -> list[float]:
    text = element.get(name) if element is not None else None
    if text is None:
        return list(default)
    try:
        values = [float(item) for item in text.split()]
    except ValueError as error:
        raise ManifestError(f"invalid {name} vector") from error
    if len(values) != 3 or not all(math.isfinite(item) for item in values):
        raise ManifestError(f"invalid {name} vector")
    return values


def _normalized(values: list[float]) -> list[float]:
    magnitude = math.sqrt(sum(item * item for item in values))
    if not math.isfinite(magnitude) or magnitude <= 0:
        raise ManifestError("joint axis must be finite and non-zero")
    return [item / magnitude for item in values]


def _contained_resource(
    package: Path, urdf_path: Path, reference: str
) -> tuple[Path, str]:
    raw = Path(reference)
    if not reference or raw.is_absolute() or "://" in reference:
        raise ManifestError(f"unsafe mesh reference: {reference!r}")
    resolved = (urdf_path.parent / raw).resolve(strict=False)
    root = package.resolve(strict=True)
    try:
        relative = resolved.relative_to(root)
    except ValueError as error:
        raise ManifestError(f"mesh reference escapes package: {reference}") from error
    if resolved.is_symlink() or not resolved.is_file():
        raise ManifestError(
            f"missing or symlinked mesh resource: {relative.as_posix()}"
        )
    return resolved, relative.as_posix()


def _joint_tree(link_names: list[str], joints: list[dict[str, Any]]) -> dict[str, Any]:
    if (
        not link_names
        or any(not name for name in link_names)
        or len(set(link_names)) != len(link_names)
    ):
        raise ManifestError("link names must be unique and non-empty")
    links = set(link_names)
    child_links: set[str] = set()
    joint_names: set[str] = set()
    for joint in joints:
        if not joint["name"] or joint["name"] in joint_names:
            raise ManifestError("joint names must be unique and non-empty")
        if (
            joint["parent"] not in links
            or joint["child"] not in links
            or joint["parent"] == joint["child"]
            or joint["child"] in child_links
        ):
            raise ManifestError("joint tree has invalid parent/child structure")
        joint_names.add(joint["name"])
        child_links.add(joint["child"])
    roots = sorted(links - child_links)
    if len(roots) != 1:
        raise ManifestError("joint tree must have exactly one root")
    reachable = {roots[0]}
    while True:
        expanded = reachable | {
            joint["child"] for joint in joints if joint["parent"] in reachable
        }
        if expanded == reachable:
            break
        reachable = expanded
    if len(reachable) != len(links):
        raise ManifestError("joint tree is disconnected or cyclic")
    return {
        "links": link_names,
        "root_links": roots,
        "joints": joints,
    }


def parse_urdf_metadata(package: Path, urdf_path: Path) -> dict[str, Any]:
    package = package.resolve(strict=True)
    urdf = urdf_path.resolve(strict=True)
    urdf.relative_to(package)
    try:
        robot = ET.parse(urdf).getroot()
    except ET.ParseError as error:
        raise ManifestError(f"invalid Artiverse URDF: {urdf}") from error
    if robot.tag != "robot":
        raise ManifestError("URDF root must be robot")
    link_nodes = list(robot.findall("link"))
    joint_nodes = list(robot.findall("joint"))
    link_names = [node.get("name", "").strip() for node in link_nodes]
    joints: list[dict[str, Any]] = []
    for joint in joint_nodes:
        parent_node = joint.find("parent")
        child_node = joint.find("child")
        joint_type = joint.get("type", "").strip().lower()
        try:
            axis = _normalized(
                _vector_attribute(joint.find("axis"), "xyz", (1.0, 0.0, 0.0))
            )
        except ManifestError:
            axis = [float("nan"), float("nan"), float("nan")]
        origin = joint.find("origin")
        limit = joint.find("limit")
        joints.append(
            {
                "name": joint.get("name", "").strip(),
                "type": joint_type,
                "parent": (
                    parent_node.get("link", "").strip()
                    if parent_node is not None
                    else ""
                ),
                "child": (
                    child_node.get("link", "").strip() if child_node is not None else ""
                ),
                "origin_xyz": _vector_attribute(origin, "xyz", (0.0, 0.0, 0.0)),
                "origin_rpy": _vector_attribute(origin, "rpy", (0.0, 0.0, 0.0)),
                "axis": axis,
                "fk_supported": joint_type
                in {"fixed", "revolute", "continuous", "prismatic"},
                "lower": _float_attribute(limit, "lower"),
                "upper": _float_attribute(limit, "upper"),
                "effort": _float_attribute(limit, "effort"),
                "velocity": _float_attribute(limit, "velocity"),
            }
        )
    tree: dict[str, Any] | None
    tree_issue: str | None
    try:
        if any(not all(math.isfinite(x) for x in row["axis"]) for row in joints):
            raise ManifestError("joint axis must be finite and non-zero")
        tree = _joint_tree(link_names, joints)
        tree_issue = None
    except ManifestError as error:
        tree = None
        tree_issue = str(error)
    resources: dict[str, dict[str, Any]] = {}
    collision_links = 0
    inertial_links = 0
    visual_links = 0
    inertial_rows: list[dict[str, Any]] = []
    for link in link_nodes:
        collisions = list(link.findall("collision"))
        visuals = list(link.findall("visual"))
        inertials = list(link.findall("inertial"))
        collision_links += int(bool(collisions))
        visual_links += int(bool(visuals))
        inertial_links += int(bool(inertials))
        for usage, elements in (("collision", collisions), ("visual", visuals)):
            for mesh in [
                node for element in elements for node in element.findall(".//mesh")
            ]:
                resource_path, relative = _contained_resource(
                    package, urdf, mesh.get("filename", "")
                )
                record = resources.setdefault(
                    relative,
                    {
                        "relative_path": relative,
                        "sha256": sha256_file(resource_path),
                        "usages": [],
                        "occurrences": 0,
                    },
                )
                if usage not in record["usages"]:
                    record["usages"].append(usage)
                record["occurrences"] += 1
        for inertial in inertials:
            inertia = inertial.find("inertia")
            inertial_rows.append(
                {
                    "link": link.get("name", ""),
                    "mass": _float_attribute(inertial.find("mass"), "value"),
                    "inertia": {
                        key: _float_attribute(inertia, key)
                        for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
                    },
                }
            )
    resource_rows = [
        {
            **resources[key],
            "usages": sorted(resources[key]["usages"]),
        }
        for key in sorted(resources)
    ]
    scalar_joints = [
        row for row in joints if row["type"] in {"revolute", "continuous", "prismatic"}
    ]
    return {
        "link_names": link_names,
        "joint_names": [row["name"] for row in joints],
        "joints": joints,
        "scalar_joints": scalar_joints,
        "joint_tree": tree,
        "joint_tree_issue": tree_issue,
        "resources": resource_rows,
        "resource_sha256": canonical_sha256(resource_rows),
        "xml_counts": {
            "links": len(link_nodes),
            "joints": len(joint_nodes),
            "fixed_joints": sum(row["type"] == "fixed" for row in joints),
            "movable_joints": len(scalar_joints),
            "visual_elements": len(robot.findall(".//visual")),
            "collision_elements": len(robot.findall(".//collision")),
            "inertial_elements": len(robot.findall(".//inertial")),
        },
        "collision": {
            "covered_links": collision_links,
            "denominator_links": len(link_nodes),
            "full_link_coverage": collision_links == len(link_nodes),
            "element_count": len(robot.findall(".//collision")),
            "resource_sha256": canonical_sha256(
                [row for row in resource_rows if "collision" in row["usages"]]
            ),
        },
        "inertia": {
            "covered_links": inertial_links,
            "denominator_links": len(link_nodes),
            "full_link_coverage": inertial_links == len(link_nodes),
            "elements": inertial_rows,
            "elements_sha256": canonical_sha256(inertial_rows),
        },
        "visual": {
            "covered_links": visual_links,
            "denominator_links": len(link_nodes),
            "full_link_coverage": visual_links == len(link_nodes),
            "element_count": len(robot.findall(".//visual")),
        },
    }


def _require_bool(value: Mapping[str, Any], field: str, location: str) -> bool:
    observed = value.get(field)
    if not isinstance(observed, bool):
        raise ManifestError(f"{location}.{field} must be a JSON boolean")
    return observed


def _validate_upstream_identity(
    name: str,
    record: Mapping[str, Any],
    identity: Mapping[str, Any],
    order: int,
    urdf_sha256: str,
) -> None:
    record_asset = record.get("asset_id")
    if record_asset is None and name == "table4":
        record_asset = record.get("dataset_id")
    if record_asset != identity["asset_id"]:
        raise ManifestError(
            f"{name} asset_id mismatch: {identity['asset_id']!r} != "
            f"{record_asset!r}"
        )
    for field in IDENTITY_FIELDS:
        if field == "asset_id":
            continue
        if field in record and record.get(field) != identity[field]:
            raise ManifestError(
                f"{name} identity mismatch for {field}: {identity['asset_id']}"
            )
    if name == "table2":
        if record.get("primary_urdf_relative_path") != "model.urdf":
            raise ManifestError(
                f"table2 primary URDF path mismatch: {identity['asset_id']}"
            )
        for field in ("primary_urdf_sha256", "model_urdf_sha256"):
            if field in record and record[field] != urdf_sha256:
                raise ManifestError(
                    f"table2 {field} mismatch: {identity['asset_id']}"
                )
        if record.get("primary_urdf_sha256") != urdf_sha256:
            raise ManifestError(
                f"table2 primary_urdf_sha256 mismatch: {identity['asset_id']}"
            )
    elif name == "table3":
        if record.get("urdf_sha256") != urdf_sha256:
            raise ManifestError(
                f"table3 urdf_sha256 mismatch: {identity['asset_id']}"
            )
    elif name == "table4":
        if (
            record.get("dataset_id") != identity["asset_id"]
            or record.get("order") != order
        ):
            raise ManifestError(
                f"table4 dataset identity/order mismatch: {identity['asset_id']}"
            )


def _strict_gate_evidence(
    table2: dict[str, Any],
    table3: dict[str, Any],
    table4: dict[str, Any],
) -> dict[str, Any]:
    table2_metrics = table2.get("metrics", {})
    if not isinstance(table2_metrics, dict):
        raise ManifestError("table2 metrics must be an object")
    table2_subgates: dict[str, bool] = {}
    for key in TABLE2_GATE_FIELDS:
        metric = table2_metrics.get(key)
        if not isinstance(metric, dict):
            raise ManifestError(f"table2 metrics.{key} must be an object")
        table2_subgates[key] = _require_bool(metric, "pass", f"table2 metrics.{key}")
    return {
        "table2": {
            "strict_urdf_pass": _require_bool(table2, "strict_urdf_pass", "table2"),
            "subgates": table2_subgates,
            "record_sha256": canonical_sha256(table2),
        },
        "table3": {
            "strict_kinematic_pass": _require_bool(
                table3, "strict_kinematic_pass", "table3"
            ),
            "tree_valid": _require_bool(table3, "tree_valid", "table3"),
            "record_sha256": canonical_sha256(table3),
        },
        "table4": {
            "strict_collision_pass": _require_bool(
                table4, "strict_collision_pass", "table4"
            ),
            "load_success": _require_bool(table4, "load_success", "table4"),
            "measurement_complete": _require_bool(
                table4, "measurement_complete", "table4"
            ),
            "record_sha256": canonical_sha256(table4),
        },
    }


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
    dataset = Path(loaded["dataset_root"])
    roots = [row["manifest_root"] for row in loaded["assets"]]
    upstream = load_upstream_records(upstream_roots, roots)
    artifact_sets = {
        name: collect_artifact_set(Path(upstream_roots[name]))
        for name in ("table2", "table3", "table4")
    }
    if formal:
        _validate_formal_upstream_artifacts(artifact_sets, upstream_roots)
    rows: list[dict[str, Any]] = []
    package_meta: list[dict[str, str]] = []
    urdf_meta: list[dict[str, str]] = []
    package_file_count = 0
    package_total_bytes = 0
    for order, identity in enumerate(loaded["assets"]):
        manifest_root = identity["manifest_root"]
        package = Path(identity["package"])
        urdf = Path(identity["urdf_path"])
        table2 = upstream["table2"][manifest_root]
        table3 = upstream["table3"][manifest_root]
        table4 = upstream["table4"][manifest_root]
        binding = package_binding(package)
        expected_package_hash = table2.get("package_content_manifest_sha256")
        if expected_package_hash is None and isinstance(
            table2.get("package_binding"), dict
        ):
            expected_package_hash = table2["package_binding"].get(
                "content_manifest_sha256"
            )
        if binding["content_manifest_sha256"] != expected_package_hash:
            raise ManifestError(
                f"source package receipt mismatch for {manifest_root}: "
                f"expected {expected_package_hash!r}, "
                f"found {binding['content_manifest_sha256']!r}"
            )
        urdf_hash = sha256_file(urdf)
        expected_urdf_hash = table2.get(
            "primary_urdf_sha256", table2.get("model_urdf_sha256")
        )
        if expected_urdf_hash != urdf_hash:
            raise ManifestError(f"source URDF drift: {manifest_root}")
        for name, record in (
            ("table2", table2),
            ("table3", table3),
            ("table4", table4),
        ):
            _validate_upstream_identity(name, record, identity, order, urdf_hash)
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
        preflight = {
            "status": "pass" if not issues else "failed",
            "issues": issues,
            "simulator_eligible": not issues,
        }
        selection_hash = hashlib.sha256(
            "\0".join(
                (SELECTION_PROTOCOL, FORMAL_ROSTER_SHA256, identity["asset_id"])
            ).encode("utf-8")
        ).hexdigest()
        row = {
            "dataset_id": f"ours_{order:04d}",
            "order": order,
            "selection_hash": selection_hash,
            **{field: identity[field] for field in IDENTITY_FIELDS},
            "category": identity["raw_category"],
            "package_relative_path": identity["package_relative_path"],
            "package_binding": binding,
            "package_content_manifest_sha256": binding["content_manifest_sha256"],
            "urdf_relative_path": identity["urdf_relative_path"],
            "urdf_sha256": urdf_hash,
            **metadata,
            "bounding_box": {
                "status": "available" if bbox_available else "not_available",
                "diagonal_m": float(bbox) if bbox_available else None,
                "protocol": "pybullet_q0_collision_shape_union_aabb_v1",
            },
            "bounding_box_diagonal": float(bbox) if bbox_available else None,
            "strict_gates": _strict_gate_evidence(table2, table3, table4),
            "preflight": preflight,
        }
        row["row_sha256"] = canonical_sha256(row, exclude_fields={"row_sha256"})
        rows.append(row)
        package_meta.append(
            {
                "manifest_root": manifest_root,
                "package_content_manifest_sha256": binding["content_manifest_sha256"],
            }
        )
        urdf_meta.append(
            {
                "manifest_root": manifest_root,
                "urdf_relpath": identity["urdf_relative_path"],
                "sha256": urdf_hash,
            }
        )
        package_file_count += binding["file_count"]
        package_total_bytes += binding["total_bytes"]
    ordered_package_hash = canonical_sha256(package_meta)
    ordered_urdf_hash = canonical_sha256(urdf_meta)
    invalid_roots = {
        row["manifest_root"] for row in rows if row["preflight"]["status"] == "failed"
    }
    if formal:
        checks = {
            "ordered package binding": (
                ordered_package_hash,
                FORMAL_ORDERED_PACKAGE_BINDING_SHA256,
            ),
            "ordered URDF binding": (
                ordered_urdf_hash,
                FORMAL_ORDERED_URDF_BINDING_SHA256,
            ),
            "package file count": (
                package_file_count,
                FORMAL_PACKAGE_FILE_COUNT,
            ),
            "package total bytes": (
                package_total_bytes,
                FORMAL_PACKAGE_TOTAL_BYTES,
            ),
            "invalid graph roots": (
                invalid_roots,
                FORMAL_INVALID_GRAPH_ROOTS,
            ),
        }
        for name, (actual, expected) in checks.items():
            if actual != expected:
                raise ManifestError(
                    f"formal Artiverse {name} mismatch: {actual!r} != {expected!r}"
                )
    protocol_receipt = protocol_with_hash(protocol)
    manifest = {
        "schema_version": "table5_ours_manifest_v1",
        "source_receipt": {
            "dataset": DATASET_NAME,
            "dataset_root": loaded["dataset_root"],
            "release_status": loaded["release_status"],
            "N_release": loaded["n_release"],
            "N_eval": loaded["n_eval"],
            "table1_manifest_path": loaded["table1_manifest_path"],
            "table1_manifest_sha256": loaded["table1_manifest_sha256"],
            "release_manifest_path": loaded["release_manifest_path"],
            "release_manifest_sha256": loaded["release_manifest_sha256"],
            "release_universe_sha256": loaded["release_universe_sha256"],
        },
        "selection": {
            "source": "Table1 manifest assets exact stored order",
            "candidate_count": loaded["n_eval"],
            "selected_count": len(rows),
            "identity_authority": "manifest_root",
            "internal_id": "ours_<zero_based_order:04d>",
            "ordered_identity_sha256": loaded["ordered_identity_sha256"],
            "ordered_manifest_root_sha256": loaded["ordered_manifest_root_sha256"],
            "ordered_package_binding_sha256": ordered_package_hash,
            "ordered_urdf_binding_sha256": ordered_urdf_hash,
            "replacement": "never",
            "outcome_filtering": False,
            "retained_preflight_failures": len(invalid_roots),
        },
        "upstream_artifacts": artifact_sets,
        "protocol_sha256": protocol_receipt["protocol_sha256"],
        "rows": rows,
    }
    manifest["cohort_sha256"] = canonical_sha256(
        manifest, exclude_fields={"cohort_sha256", "generated_at"}
    )
    return manifest


def validate_source_bindings(
    manifest: dict[str, Any],
    dataset_root: Path,
    table1_manifest: Path,
    upstream_roots: Mapping[str, Path],
    *,
    formal: bool = True,
) -> None:
    loaded = load_table1_cohort(dataset_root, table1_manifest, formal=formal)
    rows = manifest.get("rows")
    if not isinstance(rows, list) or len(rows) != len(loaded["assets"]):
        raise ManifestError("manifest row count mismatch")
    for identity, row in zip(loaded["assets"], rows):
        if row.get("manifest_root") != identity["manifest_root"]:
            raise ManifestError("manifest row order/identity mismatch")
        current = package_binding(Path(identity["package"]))
        if current != row.get("package_binding"):
            raise ManifestError(f"source package drift: {identity['manifest_root']}")
        if sha256_file(Path(identity["urdf_path"])) != row.get("urdf_sha256"):
            raise ManifestError(f"source URDF drift: {identity['manifest_root']}")
    for name in ("table2", "table3", "table4"):
        current = collect_artifact_set(Path(upstream_roots[name]))
        if current != manifest.get("upstream_artifacts", {}).get(name):
            raise ManifestError(f"upstream artifact drift: {name}")


def validate_manifest(
    manifest: dict[str, Any],
    dataset_root: Path,
    table1_manifest: Path,
    upstream_roots: Mapping[str, Path],
    *,
    protocol: dict[str, Any],
    formal: bool = True,
) -> None:
    if manifest.get("schema_version") != "table5_ours_manifest_v1":
        raise ManifestError("manifest schema mismatch")
    expected_protocol = protocol_with_hash(protocol)["protocol_sha256"]
    if manifest.get("protocol_sha256") != expected_protocol:
        raise ManifestError("protocol hash mismatch")
    rows = manifest.get("rows")
    if not isinstance(rows, list):
        raise ManifestError("manifest rows missing")
    for order, row in enumerate(rows):
        if (
            row.get("dataset_id") != f"ours_{order:04d}"
            or row.get("order") != order
            or row.get("selection_rank") != order + 1
            or row.get("asset_id") != row.get("manifest_root")
        ):
            raise ManifestError("manifest row identity/order mismatch")
        if row.get("row_sha256") != canonical_sha256(
            row, exclude_fields={"row_sha256"}
        ):
            raise ManifestError(f"manifest row hash mismatch: {row.get('dataset_id')}")
    if manifest.get("cohort_sha256") != canonical_sha256(
        manifest, exclude_fields={"cohort_sha256", "generated_at"}
    ):
        raise ManifestError("cohort hash mismatch")
    validate_source_bindings(
        manifest,
        dataset_root,
        table1_manifest,
        upstream_roots,
        formal=formal,
    )
    # Rebuild after validation so joins, preflight state, and all derived fields
    # are compared, not merely their source byte receipts.
    expected = build_manifest(
        dataset_root,
        table1_manifest,
        upstream_roots,
        protocol=protocol,
        formal=formal,
    )
    if manifest != expected:
        raise ManifestError("manifest derived evidence mismatch")


def _matmul(left: list[list[float]], right: list[list[float]]) -> list[list[float]]:
    return [
        [
            sum(left[row][index] * right[index][column] for index in range(3))
            for column in range(3)
        ]
        for row in range(3)
    ]


def _matvec(rotation: list[list[float]], vector: list[float]) -> list[float]:
    return [
        sum(rotation[row][index] * vector[index] for index in range(3))
        for row in range(3)
    ]


def _rpy_matrix(rpy: list[float]) -> list[list[float]]:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _axis_angle_matrix(axis: list[float], angle: float) -> list[list[float]]:
    x, y, z = axis
    cosine, sine, complement = (
        math.cos(angle),
        math.sin(angle),
        1 - math.cos(angle),
    )
    return [
        [
            cosine + x * x * complement,
            x * y * complement - z * sine,
            x * z * complement + y * sine,
        ],
        [
            y * x * complement + z * sine,
            cosine + y * y * complement,
            y * z * complement - x * sine,
        ],
        [
            z * x * complement - y * sine,
            z * y * complement + x * sine,
            cosine + z * z * complement,
        ],
    ]


def _quaternion(rotation: list[list[float]]) -> list[float]:
    trace = rotation[0][0] + rotation[1][1] + rotation[2][2]
    if trace > 0:
        scale = math.sqrt(trace + 1.0) * 2
        return [
            0.25 * scale,
            (rotation[2][1] - rotation[1][2]) / scale,
            (rotation[0][2] - rotation[2][0]) / scale,
            (rotation[1][0] - rotation[0][1]) / scale,
        ]
    index = max(range(3), key=lambda item: rotation[item][item])
    if index == 0:
        scale = math.sqrt(1 + rotation[0][0] - rotation[1][1] - rotation[2][2]) * 2
        return [
            (rotation[2][1] - rotation[1][2]) / scale,
            0.25 * scale,
            (rotation[0][1] + rotation[1][0]) / scale,
            (rotation[0][2] + rotation[2][0]) / scale,
        ]
    if index == 1:
        scale = math.sqrt(1 + rotation[1][1] - rotation[0][0] - rotation[2][2]) * 2
        return [
            (rotation[0][2] - rotation[2][0]) / scale,
            (rotation[0][1] + rotation[1][0]) / scale,
            0.25 * scale,
            (rotation[1][2] + rotation[2][1]) / scale,
        ]
    scale = math.sqrt(1 + rotation[2][2] - rotation[0][0] - rotation[1][1]) * 2
    return [
        (rotation[1][0] - rotation[0][1]) / scale,
        (rotation[0][2] + rotation[2][0]) / scale,
        (rotation[1][2] + rotation[2][1]) / scale,
        0.25 * scale,
    ]


def fk_link_poses(
    joint_tree: dict[str, Any], scalar_positions: Mapping[str, float]
) -> dict[str, dict[str, list[float]]]:
    identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    transforms: dict[str, tuple[list[list[float]], list[float]]] = {
        root: (identity, [0.0, 0.0, 0.0]) for root in joint_tree["root_links"]
    }
    remaining = list(joint_tree["joints"])
    while remaining:
        next_remaining: list[dict[str, Any]] = []
        advanced = False
        for joint in remaining:
            if not joint.get("fk_supported", False):
                raise ManifestError(f"unsupported joint type for FK: {joint['type']}")
            if joint["parent"] not in transforms:
                next_remaining.append(joint)
                continue
            parent_rotation, parent_translation = transforms[joint["parent"]]
            origin_rotation = _rpy_matrix(joint["origin_rpy"])
            rotation = _matmul(parent_rotation, origin_rotation)
            relative_translation = _matvec(parent_rotation, joint["origin_xyz"])
            translation = [
                parent_translation[index] + relative_translation[index]
                for index in range(3)
            ]
            position = float(scalar_positions.get(joint["name"], 0.0))
            if joint["type"] in {"revolute", "continuous"}:
                rotation = _matmul(
                    rotation,
                    _axis_angle_matrix(joint["axis"], position),
                )
            elif joint["type"] == "prismatic":
                offset = _matvec(
                    rotation,
                    [position * item for item in joint["axis"]],
                )
                translation = [translation[index] + offset[index] for index in range(3)]
            transforms[joint["child"]] = (rotation, translation)
            advanced = True
        if not advanced:
            raise ManifestError("joint tree is disconnected or cyclic")
        remaining = next_remaining
    return {
        link: {
            "translation": translation,
            "rotation": _quaternion(rotation),
        }
        for link, (rotation, translation) in transforms.items()
    }
