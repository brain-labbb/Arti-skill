"""Frozen source and receipt helpers for PartNet-Mobility Table 5 N=800."""
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


DATASET_NAME = "PartNet-Mobility"
SOURCE_RELEASE = "sapien-sim/PartNetMobility@ee0aa3ef1df16181d76d83f7415aa8c94ed1da8f"
RELEASE_STATUS = "LOCAL_COMPLETE_PROVENANCE_LIMITED"
SELECTION_SALT = "urdf-sim-ready-table4-partnet-mobility-n800-v1:20260813"
URDF_RELATIVE_NAME = "mobility.urdf"
FORMAL_N_RELEASE = 2347
FORMAL_N_EVAL = 800
FORMAL_CATEGORY_COUNT = 46
FORMAL_TABLE4_MANIFEST_SHA256 = (
    "2ff015ee6bb377ce693126b52dd632a7565a3eaa9f0007e26122a1bb4ab99900"
)
FORMAL_ORDERED_IDS_SHA256 = (
    "ef6cb964e50dc712280256c5b2f675cc2c957095c3553b21845d3562a5011883"
)
FORMAL_ORDERED_PACKAGE_BINDING_SHA256 = (
    "efdcac92772cb87631c343e92d2c2505e8fae4640e4ee9e6abbe20f28e0f4dcd"
)
FORMAL_ORDERED_URDF_BINDING_SHA256 = (
    "19b56d468ed92c3902ef0ae35b60275a89d7f7947b844b95b2dc98ac82324c8f"
)
FORMAL_PACKAGE_FILE_COUNT = 118972
FORMAL_PACKAGE_TOTAL_BYTES = 2691166823
FORMAL_ARCHIVE_SHA256 = (
    "b47247a44246111e8d09f2c0e64b4012ae35e0dcf4bb55f68a05b604455119ff"
)
FORMAL_UPSTREAM_PINS = {
    "table2": {
        "artifact_set": {
            "artifact_set_sha256": "cb4f8dce333b6dba2914c7d66e68c13abfdc5343bfde81f73c3dae201d06b90f",
            "file_count": 8,
            "total_bytes": 64907756,
        },
        "required_files": [
            "asset_records.jsonl",
            "checkpoint.json",
            "environment.json",
            "manifest.json",
            "protocol_snapshot.md",
            "summary.json",
            "summary.md",
        ],
        "summary_expectations": {
            "schema_version": "1.1.0",
            "status": "completed",
            "dataset": "PartNet-Mobility",
            "mode": "formal",
            "n_eval": 800,
            "records_present": 800,
            "error_count": 0,
            "status_counts": {"completed": 800},
            "strict_passed": 0,
            "metric_pass_counts": {
                "collision_coverage": 0,
                "finite_fields": 800,
                "inertia_validity": 0,
                "inertial_coverage": 0,
                "parse_rate": 95,
                "resource_resolution": 787,
                "strict_urdf_pass": 0,
                "valid_joint_spec": 800,
                "valid_tree": 800,
            },
        },
    },
    "table3": {
        "artifact_set": {
            "artifact_set_sha256": "30770a27722b79163255a16dc042828d76710b6493b697453e8b44bc0d6d40d8",
            "file_count": 1605,
            "total_bytes": 56147515,
        },
        "required_files": [
            "asset_records.jsonl",
            "checkpoint.json",
            "manifest.json",
            "summary.json",
            "summary.md",
        ],
        "summary_expectations": {
            "schema_version": 1,
            "status": "completed",
            "dataset": "PartNet-Mobility",
            "n_eval": 800,
            "j_eval": 4078,
            "parse_success": 800,
            "valid_tree": 800,
            "status_counts": {"completed": 800},
            "strict_passed": 793,
        },
    },
    "table4": {
        "artifact_set": {
            "artifact_set_sha256": "c25e6eedd950e2234c43d8de6d8ce2f62e519c502953ff2961339de431ff859b",
            "file_count": 2411,
            "total_bytes": 182718906,
        },
        "required_files": [
            "asset_records.json",
            "child_runtime_probe.json",
            "frozen_manifest.json",
            "pair_policy_smoke.json",
            "pair_policy_smoke.urdf",
            "state_records.jsonl",
            "summary.json",
            "verification.json",
        ],
        "summary_expectations": {
            "status": "COMPLETE_WITH_RETAINED_FAILURES",
            "protocol_id": "urdf_sim_ready_table4_partnet_mobility_n800_v1",
            "selected": 800,
            "category_count": 46,
            "load_success": 787,
            "measurement_complete": 787,
            "strict_passed": 567,
            "verification_status": "PASS",
            "verification_protocol_id": "urdf_sim_ready_table4_partnet_mobility_verify_v1",
        },
    },
}
# 13 assets whose release package is missing collision mesh references (frozen
# Table 2/Table 4 knowledge).  They remain in the cohort as retained preflight
# failures and are never handed to a simulator.
FORMAL_PREFLIGHT_FAILURE_IDS = frozenset(
    {3380, 3593, 7130, 7221, 7320, 9918, 10090, 11887, 12542, 25144, 29525, 32746, 39138}
)
IDENTITY_FIELDS = (
    "dataset_id",
    "selection_digest",
    "input_identity_sha256",
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
    Path(__file__).parents[1] / "reference" / "table5_partnet_mobility_n800_protocol_v1.json"
)
# File-byte pin. Update only with a reviewed canonical protocol change.
CANONICAL_PROTOCOL_FILE_SHA256 = (
    "da6cb9884036e73c12a20c50d74f2ae87ee9c8e3a06274b1d3686d15b33cec8e"
)
EXPECTED_GENESIS_GPU_BINDING = {
    "cuda_visible_devices": "3",
    "physical_device_index": 3,
    "visible_device_index": 0,
    "gpu_uuid": "GPU-ebc0d328-a3fa-7e89-2733-cadb001661f7",
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
    "package_relative_path",
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
        "schema_version": "table5_partnet_mobility_protocol_v1",
        "source.table4_frozen_manifest_sha256": FORMAL_TABLE4_MANIFEST_SHA256,
        "source.ordered_ids_sha256": FORMAL_ORDERED_IDS_SHA256,
        "source.ordered_package_binding_sha256": FORMAL_ORDERED_PACKAGE_BINDING_SHA256,
        "source.ordered_urdf_binding_sha256": FORMAL_ORDERED_URDF_BINDING_SHA256,
        "source.archive_sha256": FORMAL_ARCHIVE_SHA256,
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
        "selection.identity_authority": "dataset_id",
        "selection.selected_count": 800,
        "selection.replacement": "never",
        "selection.retained_preflight_failures": len(FORMAL_PREFLIGHT_FAILURE_IDS),
        "runtime.base": "fixed",
        "runtime.contacts": "enabled",
        "runtime.timestep_s": {"numerator": 1, "denominator": 240},
        "runtime.actuation.effort_controller.formula": "tau = effort * clip(kp * normalized_position_error - kd * normalized_speed, -1, 1)",
        "runtime.actuation.effort_controller.kp": 2.0,
        "runtime.actuation.effort_controller.kd": 0.2,
        "runtime.actuation.effort_controller.clip": [-1.0, 1.0],
        "cross_simulator.pairing.asset_key": "manifest.dataset_id",
        "cross_simulator.pairing.joint_key": "urdf_joint_name",
        "cross_simulator.trajectory_samples": 31,
        "cross_simulator.sample_cadence_steps": 12,
        "cross_simulator.thresholds.normalized_joint_rmse": 0.1,
        "cross_simulator.thresholds.translation_over_bbox_diagonal": 0.02,
        "cross_simulator.thresholds.rotation_rad": 0.1,
        "cross_simulator.all_three_denominator": 800,
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
            "schema_version": "table5_partnet_mobility_receipt_set_v1",
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
    if marker.get("schema_version") != "table5_partnet_mobility_receipt_set_v1":
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


def load_table4_cohort(
    dataset_root: Path, table4_manifest: Path, *, formal: bool = True
) -> dict[str, Any]:
    dataset = dataset_root.resolve(strict=True)
    cohort_path = table4_manifest.resolve(strict=True)
    cohort_hash = sha256_file(cohort_path)
    manifest = json.loads(cohort_path.read_text(encoding="utf-8"))
    items = manifest.get("items")
    if manifest.get("dataset_root") != str(dataset):
        raise ManifestError("Table 4 frozen manifest dataset_root mismatch")
    if not isinstance(items, list) or len(items) != FORMAL_N_EVAL:
        raise ManifestError("Table 4 frozen manifest must contain exactly 800 items")
    assets: list[dict[str, Any]] = []
    string_ids: list[str] = []
    for order, item in enumerate(items):
        if not isinstance(item.get("order"), int) or item["order"] != order:
            raise ManifestError(f"Table 4 item order mismatch at index {order}")
        raw_id = item.get("dataset_id")
        if not isinstance(raw_id, str) or not raw_id.isdigit():
            raise ManifestError(f"Table 4 item dataset_id is malformed: {raw_id!r}")
        dataset_id = int(raw_id)
        package_raw = dataset / raw_id
        if package_raw.is_symlink() or not package_raw.is_dir():
            raise ManifestError(f"PartNet asset directory missing or symlinked: {raw_id}")
        package = package_raw.resolve(strict=True)
        try:
            package.relative_to(dataset)
        except ValueError as error:
            raise ManifestError(f"PartNet asset directory escapes dataset root: {raw_id}") from error
        urdf_raw = package / URDF_RELATIVE_NAME
        if urdf_raw.is_symlink() or not urdf_raw.is_file():
            raise ManifestError(f"PartNet primary URDF missing or symlinked: {raw_id}")
        urdf = urdf_raw.resolve(strict=True)
        selection_digest = item.get("selection_digest")
        input_identity = item.get("input_identity_sha256")
        if not isinstance(selection_digest, str) or len(selection_digest) != 64:
            raise ManifestError(f"Table 4 item selection_digest is malformed: {raw_id}")
        if not isinstance(input_identity, str) or len(input_identity) != 64:
            raise ManifestError(f"Table 4 item input_identity_sha256 is malformed: {raw_id}")
        assets.append(
            {
                "dataset_id": dataset_id,
                "order": order,
                "selection_rank": order + 1,
                "selection_digest": selection_digest,
                "input_identity_sha256": input_identity,
                "urdf_sha256_frozen": item.get("urdf_sha256"),
                "category": item.get("category"),
                "bounding_box_diagonal_frozen": item.get("object_bbox_diagonal_m"),
                "package": str(package),
                "package_relative_path": raw_id,
                "urdf_path": str(urdf),
                "urdf_relative_path": f"{raw_id}/{URDF_RELATIVE_NAME}",
            }
        )
        string_ids.append(raw_id)
    ordered_ids = hashlib.sha256(
        json.dumps(string_ids, separators=(",", ":"), ensure_ascii=True).encode("utf-8")
    ).hexdigest()
    if formal:
        expected = {
            "Table 4 frozen manifest": (cohort_hash, FORMAL_TABLE4_MANIFEST_SHA256),
            "ordered dataset ids": (ordered_ids, FORMAL_ORDERED_IDS_SHA256),
            "N_eval": (len(assets), FORMAL_N_EVAL),
            "raw category count": (
                len({row["category"] for row in assets}),
                FORMAL_CATEGORY_COUNT,
            ),
        }
        for name, (actual, required) in expected.items():
            if actual != required:
                raise ManifestError(
                    f"formal PartNet-Mobility {name} mismatch: {actual!r} != {required!r}"
                )
    return {
        "dataset_root": str(dataset),
        "table4_manifest_path": str(cohort_path),
        "table4_manifest_sha256": cohort_hash,
        "ordered_ids_sha256": ordered_ids,
        "release_status": RELEASE_STATUS,
        "n_release": FORMAL_N_RELEASE,
        "n_eval": len(assets),
        "identity_fields": list(IDENTITY_FIELDS),
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
                "schema_version",
                "status",
                "dataset",
                "mode",
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
        {"denominator": 800, "passed": table2_pin["strict_passed"]},
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
            {"denominator": 800, "passed": passed},
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
        {"denominator": 800, "passed": table3_pin["strict_passed"]},
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
        {"denominator": 800, "passed": table4_pin["strict_passed"]},
        "formal upstream table4 summary.metrics.strict_collision_pass",
    )
    verification = json.loads(
        (Path(upstream_roots["table4"]) / "verification.json").read_text(
            encoding="utf-8"
        )
    )
    _expect_values(
        verification,
        {
            "status": table4_pin["verification_status"],
            "protocol_id": table4_pin["verification_protocol_id"],
        },
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


def _records_by_id(
    rows: list[dict[str, Any]], name: str
) -> dict[int, dict[str, Any]]:
    result: dict[int, dict[str, Any]] = {}
    for row in rows:
        raw = row.get("dataset_id")
        if isinstance(raw, bool):
            raise ManifestError(f"{name} record dataset_id is malformed")
        if isinstance(raw, str):
            if not raw.isdigit():
                raise ManifestError(f"{name} record dataset_id is malformed: {raw!r}")
            key = int(raw)
        elif isinstance(raw, int):
            key = raw
        else:
            raise ManifestError(f"{name} record missing dataset_id")
        if key in result:
            raise ManifestError(f"duplicate {name} dataset_id: {key}")
        result[key] = row
    return result


def load_upstream_records(
    upstream_roots: Mapping[str, Path], dataset_ids: list[int]
) -> dict[str, dict[int, dict[str, Any]]]:
    if set(upstream_roots) != {"table2", "table3", "table4"}:
        raise ManifestError("upstream roots must be exactly table2/table3/table4")
    table2 = _records_by_id(
        _read_jsonl(Path(upstream_roots["table2"]) / "asset_records.jsonl"),
        "Table 2",
    )
    table3 = _records_by_id(
        _read_jsonl(Path(upstream_roots["table3"]) / "asset_records.jsonl"),
        "Table 3",
    )
    table4_path = Path(upstream_roots["table4"]) / "asset_records.json"
    table4_raw = json.loads(table4_path.read_text(encoding="utf-8"))
    if not isinstance(table4_raw, list):
        raise ManifestError("Table 4 asset records must be a JSON array")
    table4 = _records_by_id(table4_raw, "Table 4")
    verification_path = Path(upstream_roots["table4"]) / "verification.json"
    if verification_path.is_file():
        verification = json.loads(verification_path.read_text(encoding="utf-8"))
        if verification.get("status") != "PASS":
            raise ManifestError("Table 4 verification status is not PASS")
    expected = set(dataset_ids)
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
    missing_resources: list[str] = []
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
                try:
                    resource_path, relative = _contained_resource(
                        package, urdf, mesh.get("filename", "")
                    )
                except ManifestError:
                    missing_resources.append(
                        f"missing_mesh_resource:{mesh.get('filename', '')}"
                    )
                    continue
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
        "preflight_issues": sorted(set(missing_resources)),
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
    raw_id = record.get("dataset_id")
    record_id = int(raw_id) if isinstance(raw_id, str) and raw_id.isdigit() else raw_id
    if record_id != identity["dataset_id"]:
        raise ManifestError(
            f"{name} dataset_id mismatch: {identity['dataset_id']}"
        )
    if name == "table2":
        if record.get("order") != order:
            raise ManifestError(f"table2 order mismatch: {identity['dataset_id']}")
        if record.get("primary_urdf_relative_path") != URDF_RELATIVE_NAME:
            raise ManifestError(
                f"table2 primary URDF path mismatch: {identity['dataset_id']}"
            )
        for field in ("primary_urdf_sha256", "model_urdf_sha256"):
            if field in record and record[field] != urdf_sha256:
                raise ManifestError(
                    f"table2 {field} mismatch: {identity['dataset_id']}"
                )
        if record.get("primary_urdf_sha256") != urdf_sha256:
            raise ManifestError(
                f"table2 primary_urdf_sha256 mismatch: {identity['dataset_id']}"
            )
        if record.get("package_content_manifest_sha256") != identity[
            "package_content_manifest_sha256"
        ]:
            raise ManifestError(
                f"table2 package binding mismatch: {identity['dataset_id']}"
            )
    elif name == "table3":
        if record.get("selection_rank") != order + 1:
            raise ManifestError(
                f"table3 selection_rank mismatch: {identity['dataset_id']}"
            )
        if record.get("urdf_sha256") != urdf_sha256:
            raise ManifestError(
                f"table3 urdf_sha256 mismatch: {identity['dataset_id']}"
            )
    elif name == "table4":
        if record.get("order") != order:
            raise ManifestError(f"table4 order mismatch: {identity['dataset_id']}")


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
    table4_manifest: Path,
    upstream_roots: Mapping[str, Path],
    *,
    protocol: dict[str, Any],
    formal: bool = True,
) -> dict[str, Any]:
    if formal:
        validate_protocol_schema(protocol)
    loaded = load_table4_cohort(dataset_root, table4_manifest, formal=formal)
    dataset = Path(loaded["dataset_root"])
    ids = [row["dataset_id"] for row in loaded["assets"]]
    upstream = load_upstream_records(upstream_roots, ids)
    artifact_sets = {
        name: collect_artifact_set(Path(upstream_roots[name]))
        for name in ("table2", "table3", "table4")
    }
    if formal:
        _validate_formal_upstream_artifacts(artifact_sets, upstream_roots)
    rows: list[dict[str, Any]] = []
    package_meta: list[dict[str, Any]] = []
    urdf_meta: list[dict[str, Any]] = []
    package_file_count = 0
    package_total_bytes = 0
    for identity in loaded["assets"]:
        order = identity["order"]
        dataset_id = identity["dataset_id"]
        package = Path(identity["package"])
        urdf = Path(identity["urdf_path"])
        table2 = upstream["table2"][dataset_id]
        table3 = upstream["table3"][dataset_id]
        table4 = upstream["table4"][dataset_id]
        binding = package_binding(package)
        expected_package_hash = table2.get("package_content_manifest_sha256")
        if binding["content_manifest_sha256"] != expected_package_hash:
            raise ManifestError(
                f"source package receipt mismatch for {dataset_id}: "
                f"expected {expected_package_hash!r}, "
                f"found {binding['content_manifest_sha256']!r}"
            )
        urdf_hash = sha256_file(urdf)
        if identity.get("urdf_sha256_frozen") != urdf_hash:
            raise ManifestError(f"source URDF drift vs frozen manifest: {dataset_id}")
        bound_identity = {
            **identity,
            "package_content_manifest_sha256": binding["content_manifest_sha256"],
        }
        for name, record in (
            ("table2", table2),
            ("table3", table3),
            ("table4", table4),
        ):
            _validate_upstream_identity(name, record, bound_identity, order, urdf_hash)
        metadata = parse_urdf_metadata(package, urdf)
        bbox = table4.get("object_bbox_diagonal_m")
        frozen_bbox = identity.get("bounding_box_diagonal_frozen")
        bbox_available = (
            isinstance(bbox, (int, float))
            and not isinstance(bbox, bool)
            and math.isfinite(float(bbox))
            and float(bbox) > 0
            and frozen_bbox == bbox
        )
        issues: list[str] = []
        if metadata["joint_tree"] is None:
            issues.append("invalid_joint_graph")
        issues.extend(metadata["preflight_issues"])
        if not bbox_available:
            issues.append("missing_bounding_box")
        preflight = {
            "status": "pass" if not issues else "failed",
            "issues": issues,
            "simulator_eligible": not issues,
        }
        row = {
            "dataset_id": dataset_id,
            "order": order,
            "selection_rank": identity["selection_rank"],
            **{field: identity[field] for field in IDENTITY_FIELDS},
            "category": identity["category"],
            "package_relative_path": identity["package_relative_path"],
            "package_binding": binding,
            "package_content_manifest_sha256": binding["content_manifest_sha256"],
            "urdf_relative_path": identity["urdf_relative_path"],
            "urdf_sha256": urdf_hash,
            **{
                key: metadata[key]
                for key in (
                    "link_names",
                    "joint_names",
                    "joints",
                    "scalar_joints",
                    "joint_tree",
                    "joint_tree_issue",
                    "resources",
                    "resource_sha256",
                    "xml_counts",
                    "collision",
                    "inertia",
                    "visual",
                )
            },
            "bounding_box": {
                "status": "available" if bbox_available else "not_available",
                "diagonal_m": float(bbox) if bbox_available else None,
                "protocol": "release_bounding_box_json_v1",
            },
            "bounding_box_diagonal": float(bbox) if bbox_available else None,
            "strict_gates": _strict_gate_evidence(table2, table3, table4),
            "preflight": preflight,
        }
        row["row_sha256"] = canonical_sha256(row, exclude_fields={"row_sha256"})
        rows.append(row)
        package_meta.append(
            {
                "dataset_id": dataset_id,
                "package_content_manifest_sha256": binding["content_manifest_sha256"],
            }
        )
        urdf_meta.append(
            {
                "dataset_id": dataset_id,
                "urdf_relpath": identity["urdf_relative_path"],
                "sha256": urdf_hash,
            }
        )
        package_file_count += binding["file_count"]
        package_total_bytes += binding["total_bytes"]
    ordered_package_hash = canonical_sha256(package_meta)
    ordered_urdf_hash = canonical_sha256(urdf_meta)
    preflight_failure_ids = {
        row["dataset_id"] for row in rows if row["preflight"]["status"] == "failed"
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
            "preflight failure ids": (
                preflight_failure_ids,
                set(FORMAL_PREFLIGHT_FAILURE_IDS),
            ),
        }
        for name, (actual, expected) in checks.items():
            if actual != expected:
                raise ManifestError(
                    f"formal PartNet-Mobility {name} mismatch: {actual!r} != {expected!r}"
                )
    protocol_receipt = protocol_with_hash(protocol)
    manifest = {
        "schema_version": "table5_partnet_mobility_manifest_v1",
        "source_receipt": {
            "dataset": DATASET_NAME,
            "dataset_root": loaded["dataset_root"],
            "source_release": SOURCE_RELEASE,
            "release_status": loaded["release_status"],
            "archive_sha256": FORMAL_ARCHIVE_SHA256,
            "N_release": loaded["n_release"],
            "N_eval": loaded["n_eval"],
            "table4_frozen_manifest_path": loaded["table4_manifest_path"],
            "table4_frozen_manifest_sha256": loaded["table4_manifest_sha256"],
            "ordered_ids_sha256": loaded["ordered_ids_sha256"],
        },
        "selection": {
            "source": "Table 4 frozen manifest items exact stored order",
            "candidate_count": loaded["n_eval"],
            "selected_count": len(rows),
            "identity_authority": "dataset_id",
            "selection_salt": SELECTION_SALT,
            "ordered_ids_sha256": loaded["ordered_ids_sha256"],
            "ordered_package_binding_sha256": ordered_package_hash,
            "ordered_urdf_binding_sha256": ordered_urdf_hash,
            "replacement": "never",
            "outcome_filtering": False,
            "retained_preflight_failures": len(preflight_failure_ids),
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
    table4_manifest: Path,
    upstream_roots: Mapping[str, Path],
    *,
    formal: bool = True,
) -> None:
    loaded = load_table4_cohort(dataset_root, table4_manifest, formal=formal)
    rows = manifest.get("rows")
    if not isinstance(rows, list) or len(rows) != len(loaded["assets"]):
        raise ManifestError("manifest row count mismatch")
    for identity, row in zip(loaded["assets"], rows):
        if row.get("dataset_id") != identity["dataset_id"]:
            raise ManifestError("manifest row order/identity mismatch")
        current = package_binding(Path(identity["package"]))
        if current != row.get("package_binding"):
            raise ManifestError(f"source package drift: {identity['dataset_id']}")
        if sha256_file(Path(identity["urdf_path"])) != row.get("urdf_sha256"):
            raise ManifestError(f"source URDF drift: {identity['dataset_id']}")
    for name in ("table2", "table3", "table4"):
        current = collect_artifact_set(Path(upstream_roots[name]))
        if current != manifest.get("upstream_artifacts", {}).get(name):
            raise ManifestError(f"upstream artifact drift: {name}")


def validate_manifest(
    manifest: dict[str, Any],
    dataset_root: Path,
    table4_manifest: Path,
    upstream_roots: Mapping[str, Path],
    *,
    protocol: dict[str, Any],
    formal: bool = True,
) -> None:
    if manifest.get("schema_version") != "table5_partnet_mobility_manifest_v1":
        raise ManifestError("manifest schema mismatch")
    expected_protocol = protocol_with_hash(protocol)["protocol_sha256"]
    if manifest.get("protocol_sha256") != expected_protocol:
        raise ManifestError("protocol hash mismatch")
    rows = manifest.get("rows")
    if not isinstance(rows, list):
        raise ManifestError("manifest rows missing")
    for order, row in enumerate(rows):
        if (
            not isinstance(row.get("dataset_id"), int)
            or isinstance(row.get("dataset_id"), bool)
            or row.get("order") != order
            or row.get("selection_rank") != order + 1
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
        table4_manifest,
        upstream_roots,
        formal=formal,
    )
    # Rebuild after validation so joins, preflight state, and all derived fields
    # are compared, not merely their source byte receipts.
    expected = build_manifest(
        dataset_root,
        table4_manifest,
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
