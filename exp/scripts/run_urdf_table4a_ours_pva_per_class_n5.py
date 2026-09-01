#!/usr/bin/env python3
"""Run Table 4a on the frozen Ours per-class N=5 supplementary cohort.

The adapter binds the shared fail-closed Genesis evaluator to an explicitly
selected Table 4 formal receipt.  No Table 4 hash is known in advance: the
receipt is validated against the canonical 2,655-asset cohort and its actual
manifest/state/asset hashes are frozen into every Table 4a run receipt.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Any, Mapping, NamedTuple, Sequence
import xml.etree.ElementTree as ET


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from exp.scripts import run_urdf_table4a_articraft10k as shared  # noqa: E402


DATASET_LABEL = "Ours per-class N=5 (supplementary)"
SCHEMA_VERSION = "table4a-ours-pva-per-class-n5/v1"
PROTOCOL_ID = "table4a_ours_pva_per_class_n5_max_joints_v1"
TABLE4_SCHEMA_VERSION = "table4_ours_pva_per_class_n5_frozen_manifest_v1"
TABLE4_PROTOCOL_ID = "urdf-sim-ready-table4-ours-per-class-n5-max-joints-v1"
TABLE4_ARTIFACT_SCHEMA_VERSION = "table4_artifact_manifest_v1"
TABLE4_RECEIPT_FILES = (
    "frozen_manifest.json",
    "asset_records.jsonl",
    "state_records.jsonl",
    "summary.json",
    "report.md",
    "verification.json",
    "checkpoint.json",
    "protocol_document_at_freeze.md",
    "timing.json",
)
TABLE4_CLOSURE_CHECKS = {
    "manifest_self_hash_valid",
    "base_verification_passed",
    "checkpoint_complete",
    "expected_states_match_verification",
    "executed_states_within_denominator",
}

COHORT_MANIFEST = REPO / "exp/PV-A-per-class-n5-max-joints/manifest.json"
COHORT_FILE_SHA256 = "e78f4b767023f8a5c1517d96bfab35a39482d6eee28238820a9b91ac3ea8d293"
COHORT_CONTENT_SHA256 = "eea55287dd70b710a7c03b11b16c6685208bbaa63cde925232293cb9012c8158"
TABLE2_COHORT_MANIFEST = REPO / "exp/runtime/table2_pva_per_class_n5_max_joints/manifest.json"
TABLE2_COHORT_FILE_SHA256 = "97b7df486b75f961978bc86eb7a2985cb8e40eba63336ed7531700a4ff234471"
TABLE2_COHORT_CONTENT_SHA256 = "afff3beb6ba320ccb4855ecb380de47b6de7ca91fdc77f2ba071a53944e03c14"
TABLE3_ROOT = REPO / "exp/runtime/urdf_table3_pva_per_class_n5_max_joints_n2655_20260824T0310Z"
TABLE3_MANIFEST = TABLE3_ROOT / "manifest.json"
TABLE3_MANIFEST_FILE_SHA256 = "49a8baee13deabc748fca589c954cb771a119b4499486dee9866a2030f3d36de"
TABLE3_MANIFEST_CONTENT_SHA256 = "1a35ba1462777239bdaf83e998a056fb228f34e434a2c083fe1a4a32fc255903"
TABLE3_RECORDS = TABLE3_ROOT / "asset_records.jsonl"
TABLE3_RECORDS_SHA256 = "1e573c5f097eef8c6a63b8de295502c17104320131110fafa432edf5edf78d86"

N_EVAL = 2655
J_EVAL = 14968
EXPECTED_CATEGORY_COUNT = 531
N_RELEASE = 302440
SINGLE_SAMPLES = 21
EXPECTED_SINGLE_STATES = SINGLE_SAMPLES * J_EVAL
EXPECTED_RANGE_EVALUABLE_JOINTS = 14943
EXPECTED_RANGE_INVALID_JOINTS = 25
EXPECTED_GENESIS_REPLAY_STATES = SINGLE_SAMPLES * EXPECTED_RANGE_EVALUABLE_JOINTS
EXPECTED_FAIL_CLOSED_WITHOUT_REPLAY_STATES = SINGLE_SAMPLES * EXPECTED_RANGE_INVALID_JOINTS
FORMAL_WORKERS = 1
CHILD_TIMEOUT_SECONDS = 1800
CPU_AFFINITY_LAUNCHER = SCRIPT.with_name("exec_with_cpu_affinity.py")
EARLY_AFFINITY_RECEIPT_ENV = "TABLE4A_EARLY_CPU_AFFINITY_RECEIPT"
EARLY_CPU_AFFINITY_POLICY = {
    "binding_point": "before importing the Table 4a child runner",
    "affinity_environment": "LAM_GENESIS_CPU_AFFINITY",
    "receipt_environment": EARLY_AFFINITY_RECEIPT_ENV,
    "child_cpu_affinity_width": 4,
    "exec_preserves_pid": True,
}
TORCH_THREAD_COUNTS = {
    "intra_op_threads": 1,
    "inter_op_threads": 1,
}
TORCH_THREADING_POLICY = {
    "expected": TORCH_THREAD_COUNTS,
    "binding": (
        "torch.set_num_threads(1) and torch.set_num_interop_threads(1) "
        "before Genesis import"
    ),
    "readback": (
        "completed children record before/after values; parent-enforced timeouts remain "
        "explicit NOT_OBSERVED fail-closed records"
    ),
}
RESUME_POLICY = (
    "Resume is unsupported; an interrupted invocation is non-public and must restart "
    "in a new output directory. Existing output directories are rejected."
)
WALL_CLOCK_MEASUREMENT_ENDPOINT = (
    "after shared run_scope and before post-run receipt publication"
)
FORMAL_VERIFICATION_CHECK_NAMES = (
    "source_manifest_file_sha256",
    "table4_state_records_sha256",
    "table4_asset_records_sha256",
    "canonical_cohort_file_sha256",
    "table2_cohort_file_sha256",
    "table3_records_sha256",
    "record_count",
    "frozen_order_preserved",
    "j_eval_denominator",
    "retention_denominator",
    "state_intended_count",
    "state_hash_cross_check_complete",
    "package_input_bindings_attested",
    "early_cpu_affinity_attested",
    "torch_thread_controls_attested",
    "category_mapping_sha256",
    "category_count",
    "aggregate_recomputation_matches",
)
EXPECTED_ORDERED_IDS_SHA256 = "b5c9262eca8e65ede90c597a16e4ed2b0d7348b4eeb326cd64a06cea518c4178"
EXPECTED_CATEGORY_MAPPING_SHA256 = "41e8ef9aa45b3fd2268ce7bccdbd1816dce61843ea6b6125d62f4170f72dcdce"
PROTOCOL_DOCUMENT = REPO / "exp/URDF-Sim-Ready-Automatic-Evaluation.md"
ARTIFACT_SCHEMA_VERSION = "table4a-ours-pva-per-class-n5-artifact-manifest/v1"
VERIFICATION_SCHEMA_VERSION = "table4a-ours-pva-per-class-n5-verification/v1"
RECEIPT_FILES = (
    "frozen_config.json",
    "protocol_snapshot.md",
    "asset_records.jsonl",
    "joint_records.jsonl",
    "summary.json",
    "summary.md",
    "manifest.json",
    "timing.json",
    "verification.json",
)
ARTIFACT_EXCLUSIONS = {
    "artifact_manifest.json": "self-referential manifest",
    "child_jobs/": "non-public child launch scratch",
    "children/": "non-public child result scratch; aggregate records are closed above",
    "genesis-cache/": "non-public regenerable runtime cache",
}


class Table4Binding(NamedTuple):
    directory: Path
    manifest_path: Path
    state_records_path: Path
    asset_records_path: Path
    artifact_manifest_path: Path
    manifest: dict[str, Any]
    cohort_assets: list[dict[str, Any]]
    frozen_manifest_sha256: str
    frozen_manifest_content_sha256: str
    state_records_sha256: str
    asset_records_sha256: str
    artifact_manifest_sha256: str
    ordered_ids_sha256: str


_BOUND_TABLE4: Table4Binding | None = None
_CANONICAL_ASSETS: list[dict[str, Any]] | None = None
_SHARED_SPAWN_CHILDREN = shared.spawn_children
_SHARED_RUN_CHILD = shared.run_child
_SHARED_FAILED_ASSET_RECORD = shared._failed_asset_record
_SHARED_RUNNER_SCRIPT = shared.SCRIPT


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat().replace("+00:00", "Z")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_sha256(value: Any) -> str:
    payload = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def manifest_self_hash(manifest: Mapping[str, Any]) -> str:
    body = dict(manifest)
    body.pop("manifest_content_sha256", None)
    return canonical_sha256(body)


def artifact_manifest_self_hash(manifest: Mapping[str, Any]) -> str:
    body = dict(manifest)
    body.pop("artifact_manifest_content_sha256", None)
    return canonical_sha256(body)


def atomic_json(path: Path, value: Any) -> None:
    shared.atomic_write_json(path, value)


def _unobserved_early_cpu_affinity() -> dict[str, Any]:
    return {
        "status": "NOT_OBSERVED",
        "pid": None,
        "requested": None,
        "observed": None,
    }


def validate_early_cpu_affinity_receipt() -> dict[str, Any]:
    affinity_raw = os.environ.get(shared.lam4a.CPU_AFFINITY_ENV, "")
    tokens = affinity_raw.split(",")
    if (
        not affinity_raw
        or any(
            not token
            or not token.isascii()
            or not token.isdecimal()
            or token != str(int(token))
            for token in tokens
        )
    ):
        raise RuntimeError("early CPU affinity request is missing or malformed")
    requested = [int(token) for token in tokens]
    if (
        requested != sorted(set(requested))
        or len(requested) != shared.lam4a.CPU_AFFINITY_WIDTH
    ):
        raise RuntimeError(f"early CPU affinity request is non-canonical: {requested}")
    observed = sorted(int(cpu) for cpu in os.sched_getaffinity(0))
    if observed != requested:
        raise RuntimeError(
            f"early CPU affinity readback mismatch: {observed} != {requested}"
        )
    pid = os.getpid()
    expected_marker = f"pid={pid};cpus={affinity_raw}"
    if os.environ.get(EARLY_AFFINITY_RECEIPT_ENV) != expected_marker:
        raise RuntimeError("early CPU affinity launcher receipt is missing or invalid")
    return {
        "status": "COMPLETE",
        "pid": pid,
        "requested": requested,
        "observed": observed,
    }


def early_cpu_affinity_attestation_valid(record: Mapping[str, Any]) -> bool:
    attestation = record.get("early_cpu_affinity")
    if not isinstance(attestation, Mapping):
        return False
    if attestation.get("status") == "COMPLETE":
        requested = attestation.get("requested")
        child = record.get("child")
        child_pid_matches = (
            record.get("status") != "completed"
            or isinstance(child, Mapping)
            and child.get("pid") == attestation.get("pid")
        )
        return (
            isinstance(attestation.get("pid"), int)
            and not isinstance(attestation.get("pid"), bool)
            and int(attestation["pid"]) > 0
            and isinstance(requested, list)
            and len(requested) == shared.lam4a.CPU_AFFINITY_WIDTH
            and requested == sorted(set(requested))
            and all(isinstance(cpu, int) and not isinstance(cpu, bool) and cpu >= 0 for cpu in requested)
            and attestation.get("observed") == requested
            and child_pid_matches
        )
    if attestation.get("status") != "NOT_OBSERVED":
        return False
    issues = [str(issue) for issue in (record.get("issues") or [])]
    return (
        record.get("status") == "error"
        and attestation.get("pid") is None
        and attestation.get("requested") is None
        and attestation.get("observed") is None
        and any("asset_timeout" in issue for issue in issues)
    )


def _torch_module() -> Any:
    try:
        import torch
    except Exception as exc:  # noqa: BLE001
        raise RuntimeError(f"PyTorch import failed: {type(exc).__name__}: {exc}") from exc
    return torch


def read_torch_threading() -> dict[str, int]:
    """Read PyTorch pool sizes without changing them."""

    torch = _torch_module()
    return {
        "intra_op_threads": int(torch.get_num_threads()),
        "inter_op_threads": int(torch.get_num_interop_threads()),
    }


def bind_torch_threading() -> dict[str, int]:
    """Limit and read back PyTorch pools before Genesis initializes them."""

    torch = _torch_module()
    try:
        if int(torch.get_num_threads()) != TORCH_THREAD_COUNTS["intra_op_threads"]:
            torch.set_num_threads(TORCH_THREAD_COUNTS["intra_op_threads"])
        # PyTorch permits this setter only before inter-op work starts. Avoid
        # calling it again when the repeated runtime identity check reads back 1.
        if int(torch.get_num_interop_threads()) != TORCH_THREAD_COUNTS["inter_op_threads"]:
            torch.set_num_interop_threads(TORCH_THREAD_COUNTS["inter_op_threads"])
        observed = read_torch_threading()
    except Exception as exc:  # noqa: BLE001
        raise RuntimeError(
            f"PyTorch thread binding failed: {type(exc).__name__}: {exc}"
        ) from exc
    if observed != TORCH_THREAD_COUNTS:
        raise RuntimeError(
            f"PyTorch thread binding readback mismatch: {observed} != {TORCH_THREAD_COUNTS}"
        )
    return observed


def _unobserved_torch_threading() -> dict[str, Any]:
    return {
        "status": "NOT_OBSERVED",
        "expected": dict(TORCH_THREAD_COUNTS),
        "before_evaluation": None,
        "after_evaluation": None,
    }


def _complete_torch_threading(
    record: dict[str, Any], before: Mapping[str, int]
) -> dict[str, Any]:
    after: dict[str, int] | None = None
    try:
        after = read_torch_threading()
        if after != TORCH_THREAD_COUNTS:
            raise RuntimeError(
                f"PyTorch thread readback mismatch: {after} != {TORCH_THREAD_COUNTS}"
            )
    except Exception as exc:  # noqa: BLE001
        record["status"] = "error"
        issues = list(record.get("issues") or [])
        issues.append(f"torch_thread_readback_failed: {type(exc).__name__}: {exc}")
        record["issues"] = issues
        record["torch_threading"] = {
            "status": "FAILED",
            "expected": dict(TORCH_THREAD_COUNTS),
            "before_evaluation": dict(before),
            "after_evaluation": after,
        }
        return record
    record["torch_threading"] = {
        "status": "COMPLETE",
        "expected": dict(TORCH_THREAD_COUNTS),
        "before_evaluation": dict(before),
        "after_evaluation": after,
    }
    return record


def torch_threading_attestation_valid(record: Mapping[str, Any]) -> bool:
    attestation = record.get("torch_threading")
    if not isinstance(attestation, Mapping):
        return False
    expected = dict(TORCH_THREAD_COUNTS)
    if attestation.get("expected") != expected:
        return False
    if attestation.get("status") == "COMPLETE":
        return (
            attestation.get("before_evaluation") == expected
            and attestation.get("after_evaluation") == expected
        )
    if attestation.get("status") != "NOT_OBSERVED":
        return False
    issues = [str(issue) for issue in (record.get("issues") or [])]
    parent_failure = any("asset_timeout" in issue for issue in issues)
    return (
        record.get("status") == "error"
        and attestation.get("before_evaluation") is None
        and attestation.get("after_evaluation") is None
        and parent_failure
    )


def _validated_package_binding(row: Mapping[str, Any]) -> Mapping[str, Any]:
    dataset_id = str(row.get("dataset_id", "<unknown>"))
    binding = row.get("package_binding")
    if not isinstance(binding, Mapping) or not isinstance(binding.get("files"), list):
        raise ValueError(f"package binding missing: {dataset_id}")
    files = binding["files"]
    if canonical_sha256(files) != binding.get("content_manifest_sha256"):
        raise ValueError(f"package binding self-hash mismatch: {dataset_id}")
    if int(binding.get("file_count", -1)) != len(files):
        raise ValueError(f"package binding file count mismatch: {dataset_id}")
    if int(binding.get("total_bytes", -1)) != sum(int(entry["bytes"]) for entry in files):
        raise ValueError(f"package binding byte count mismatch: {dataset_id}")
    urdf_rows = [entry for entry in files if entry.get("path") == "model.urdf"]
    if len(urdf_rows) != 1 or urdf_rows[0].get("sha256") != row.get("urdf_sha256"):
        raise ValueError(f"package binding URDF mismatch: {dataset_id}")
    return binding


def _load_self_hashed_manifest(
    path: Path,
    *,
    expected_file_sha256: str,
    expected_content_sha256: str,
    label: str,
) -> dict[str, Any]:
    path = path.resolve(strict=True)
    if sha256_file(path) != expected_file_sha256:
        raise ValueError(f"{label} file hash mismatch")
    manifest = json.loads(path.read_text(encoding="utf-8"))
    if manifest.get("manifest_content_sha256") != manifest_self_hash(manifest):
        raise ValueError(f"{label} self-hash mismatch")
    if manifest.get("manifest_content_sha256") != expected_content_sha256:
        raise ValueError(f"{label} content hash mismatch")
    return manifest


def load_canonical_cohort() -> list[dict[str, Any]]:
    global _CANONICAL_ASSETS
    if _CANONICAL_ASSETS is not None:
        return [dict(row) for row in _CANONICAL_ASSETS]
    cohort = _load_self_hashed_manifest(
        COHORT_MANIFEST,
        expected_file_sha256=COHORT_FILE_SHA256,
        expected_content_sha256=COHORT_CONTENT_SHA256,
        label="canonical cohort manifest",
    )
    if (
        cohort.get("protocol_id") != "pva-per-class-n5-fence-ferris-max-movable-joints-v1"
        or cohort.get("n_eval") != N_EVAL
        or cohort.get("class_count") != EXPECTED_CATEGORY_COUNT
    ):
        raise ValueError("canonical cohort metadata mismatch")
    assets = cohort.get("assets")
    if not isinstance(assets, list) or len(assets) != N_EVAL:
        raise ValueError("canonical cohort asset count mismatch")

    table2 = _load_self_hashed_manifest(
        TABLE2_COHORT_MANIFEST,
        expected_file_sha256=TABLE2_COHORT_FILE_SHA256,
        expected_content_sha256=TABLE2_COHORT_CONTENT_SHA256,
        label="Table 2 cohort receipt",
    )
    table2_assets = table2.get("assets")
    if (
        table2.get("N_eval") != N_EVAL
        or table2.get("N_release") != N_RELEASE
        or not isinstance(table2_assets, list)
        or len(table2_assets) != N_EVAL
    ):
        raise ValueError("Table 2 cohort metadata mismatch")

    normalized: list[dict[str, Any]] = []
    seen: set[str] = set()
    for index, (raw, table2_raw) in enumerate(zip(assets, table2_assets)):
        if raw.get("selection_index") != index or table2_raw.get("selection_index") != index:
            raise ValueError(f"cohort order mismatch at index {index}")
        dataset_id = str(raw.get("dataset_id", ""))
        if not dataset_id or dataset_id in seen:
            raise ValueError(f"duplicate or empty cohort identity: {dataset_id!r}")
        seen.add(dataset_id)
        binding = _validated_package_binding(raw)
        package = Path(str(raw.get("package", ""))).resolve(strict=True)
        urdf_path = package / str(raw.get("primary_urdf_relative_path", ""))
        if not urdf_path.is_file() or urdf_path.name != "model.urdf":
            raise ValueError(f"canonical URDF missing: {dataset_id}")
        if sha256_file(urdf_path) != raw.get("urdf_sha256"):
            raise ValueError(f"canonical URDF hash mismatch: {dataset_id}")
        comparable = (
            "dataset_id",
            "package",
            "primary_urdf_relative_path",
            "urdf_sha256",
            "package_binding",
        )
        if any(table2_raw.get(key) != raw.get(key) for key in comparable):
            raise ValueError(f"Table 2 cohort binding mismatch: {dataset_id}")
        normalized.append(dict(raw) | {"package": str(package), "package_binding": dict(binding)})
    ids_hash = canonical_sha256([row["dataset_id"] for row in normalized])
    if ids_hash != EXPECTED_ORDERED_IDS_SHA256:
        raise ValueError("canonical cohort ordered identity hash mismatch")
    _CANONICAL_ASSETS = normalized
    return [dict(row) for row in normalized]


def _artifact_binds(
    artifact: Mapping[str, Any], path: Path, *, name: str
) -> bool:
    try:
        entry = artifact.get("files", {}).get(name)
        return bool(
            isinstance(entry, Mapping)
            and set(entry) == {"bytes", "sha256"}
            and path.is_file()
            and not path.is_symlink()
            and isinstance(entry.get("bytes"), int)
            and not isinstance(entry.get("bytes"), bool)
            and entry.get("bytes") == path.stat().st_size
            and isinstance(entry.get("sha256"), str)
            and entry.get("sha256") == sha256_file(path)
        )
    except (OSError, TypeError):
        return False


def _validate_table4_artifact_receipt(
    directory: Path,
    artifact: Mapping[str, Any],
    *,
    manifest_content_sha256: str,
    expected_n: int,
    expected_j: int,
) -> None:
    if artifact.get("artifact_manifest_content_sha256") != artifact_manifest_self_hash(
        artifact
    ):
        raise ValueError("Table 4 artifact self-hash mismatch")
    files = artifact.get("files")
    if (
        artifact.get("schema_version") != TABLE4_ARTIFACT_SCHEMA_VERSION
        or artifact.get("protocol_id") != TABLE4_PROTOCOL_ID
        or artifact.get("dataset") != DATASET_LABEL
        or not isinstance(files, Mapping)
        or set(files) != set(TABLE4_RECEIPT_FILES)
    ):
        raise ValueError("Table 4 artifact protocol metadata mismatch")
    if artifact.get("run_manifest_content_sha256") != manifest_content_sha256:
        raise ValueError("Table 4 artifact source manifest binding mismatch")

    expected_states = expected_n + SINGLE_SAMPLES * expected_j + 64 * expected_n
    executed_states = artifact.get("executed_states")
    if (
        artifact.get("n_eval") != expected_n
        or artifact.get("j_eval") != expected_j
        or artifact.get("expected_states") != expected_states
        or not isinstance(executed_states, int)
        or isinstance(executed_states, bool)
        or not 0 <= executed_states <= expected_states
    ):
        raise ValueError("Table 4 artifact denominator mismatch")

    closure = artifact.get("closure_checks")
    if (
        not isinstance(closure, Mapping)
        or set(closure) != TABLE4_CLOSURE_CHECKS
        or not all(value is True for value in closure.values())
    ):
        raise ValueError("Table 4 artifact closure checks are not all true")

    public_files = {
        path.name
        for path in directory.iterdir()
        if path.is_file() or path.is_symlink()
    }
    if public_files != set(TABLE4_RECEIPT_FILES) | {"artifact_manifest.json"}:
        raise ValueError("Table 4 artifact has unlisted top-level files")
    for name in TABLE4_RECEIPT_FILES:
        if not _artifact_binds(artifact, directory / name, name=name):
            raise ValueError(f"Table 4 artifact manifest does not bind {name}")
    upstream_protocol = directory / "protocol_document_at_freeze.md"
    if sha256_file(upstream_protocol) != sha256_file(PROTOCOL_DOCUMENT):
        raise ValueError("Table 4 protocol snapshot does not match current protocol document")

    verification = json.loads(
        (directory / "verification.json").read_text(encoding="utf-8")
    )
    checkpoint = json.loads((directory / "checkpoint.json").read_text(encoding="utf-8"))
    timing = json.loads((directory / "timing.json").read_text(encoding="utf-8"))
    if (
        not isinstance(verification, Mapping)
        or verification.get("status") != "PASS"
        or verification.get("expected_states") != expected_states
        or verification.get("executed_states") != executed_states
    ):
        raise ValueError("Table 4 verification receipt mismatch")
    if not isinstance(checkpoint, Mapping) or checkpoint.get("state") != "complete":
        raise ValueError("Table 4 checkpoint is not complete")
    if not isinstance(timing, Mapping) or any(
        timing.get(key) != value
        for key, value in {
            "n_eval": expected_n,
            "j_eval": expected_j,
            "expected_states": expected_states,
            "executed_states": executed_states,
        }.items()
    ):
        raise ValueError("Table 4 timing denominator mismatch")


def _validate_table4_manifest_cohort(
    manifest: Any,
    cohort_assets: Sequence[Mapping[str, Any]],
    *,
    expected_n: int,
    expected_j: int,
    expected_category_count: int,
) -> tuple[str, list[str]]:
    if not isinstance(manifest, Mapping):
        raise ValueError("Table 4 source manifest must be an object")
    declared = manifest.get("manifest_content_sha256")
    if declared != manifest_self_hash(manifest):
        raise ValueError("Table 4 source manifest self-hash mismatch")
    if (
        manifest.get("schema_version") != TABLE4_SCHEMA_VERSION
        or manifest.get("protocol_id") != TABLE4_PROTOCOL_ID
        or manifest.get("dataset") != DATASET_LABEL
        or manifest.get("classification") != "FORMAL"
    ):
        raise ValueError("Table 4 source manifest protocol metadata mismatch")
    items = manifest.get("items")
    if not isinstance(items, list) or len(items) != expected_n or len(cohort_assets) != expected_n:
        raise ValueError(f"Table 4 source manifest must contain exactly {expected_n} items")

    categories: set[str] = set()
    total_joints = 0
    ordered_ids: list[str] = []
    for index, (item, cohort) in enumerate(zip(items, cohort_assets)):
        if not isinstance(item, Mapping):
            raise ValueError(f"Table 4 item {index} must be an object")
        dataset_id = str(cohort.get("dataset_id", ""))
        binding = _validated_package_binding(cohort)
        expected_package = str(Path(str(cohort["package"])).resolve(strict=True))
        if cohort.get("selection_index") != index:
            raise ValueError(f"canonical cohort order mismatch at index {index}")
        if (
            item.get("order") != index
            or item.get("dataset_id") != dataset_id
            or item.get("asset_id") != dataset_id
        ):
            raise ValueError(f"Table 4 item order/identity mismatch at index {index}")
        if str(Path(str(item.get("package", ""))).resolve(strict=True)) != expected_package:
            raise ValueError(f"Table 4 package mismatch: {dataset_id}")
        expected_urdf = Path(expected_package) / str(cohort["primary_urdf_relative_path"])
        item_urdf = Path(str(item.get("primary_urdf_relpath", "")))
        if not item_urdf.is_absolute():
            item_urdf = Path(expected_package).parent / item_urdf
        if item_urdf.resolve(strict=True) != expected_urdf.resolve(strict=True):
            raise ValueError(f"Table 4 URDF path mismatch: {dataset_id}")
        if item.get("urdf_sha256") != cohort.get("urdf_sha256"):
            raise ValueError(f"Table 4 URDF hash mismatch: {dataset_id}")
        package_fields = {
            "package_binding_content_manifest_sha256": binding["content_manifest_sha256"],
            "package_binding_file_count": binding["file_count"],
            "package_binding_total_bytes": binding["total_bytes"],
        }
        if any(item.get(key) != value for key, value in package_fields.items()):
            raise ValueError(f"Table 4 package binding mismatch: {dataset_id}")
        category = str(cohort.get("category", ""))
        if item.get("category") != category:
            raise ValueError(f"Table 4 category mismatch: {dataset_id}")
        movable = item.get("movable_dof_count")
        if not isinstance(movable, int) or isinstance(movable, bool) or movable < 0:
            raise ValueError(f"Table 4 movable DoF invalid: {dataset_id}")
        observed_movable = len(_joint_specs(expected_urdf))
        if observed_movable != movable:
            raise ValueError(
                f"Table 4 movable joint count mismatch: {dataset_id}: "
                f"URDF={observed_movable}, receipt={movable}"
            )
        total_joints += movable
        categories.add(category)
        ordered_ids.append(dataset_id)
    if total_joints != expected_j:
        raise ValueError(f"Table 4 J_eval mismatch: {total_joints} != {expected_j}")
    if len(categories) != expected_category_count:
        raise ValueError("Table 4 category count mismatch")
    return str(declared), ordered_ids


def validate_table4_receipt(
    table4_dir: Path,
    cohort_assets: Sequence[Mapping[str, Any]],
    *,
    expected_n: int,
    expected_j: int,
    expected_category_count: int,
) -> Table4Binding:
    directory = table4_dir.resolve(strict=True)
    manifest_path = directory / "frozen_manifest.json"
    state_path = directory / "state_records.jsonl"
    asset_path = directory / "asset_records.jsonl"
    artifact_path = directory / "artifact_manifest.json"
    for path in (manifest_path, state_path, asset_path, artifact_path):
        if path.is_symlink():
            raise ValueError(f"Table 4 receipt artifact must not be a symlink: {path.name}")
        if not path.is_file():
            raise ValueError(f"Table 4 receipt artifact missing: {path.name}")

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    declared, ordered_ids = _validate_table4_manifest_cohort(
        manifest,
        cohort_assets,
        expected_n=expected_n,
        expected_j=expected_j,
        expected_category_count=expected_category_count,
    )

    artifact = json.loads(artifact_path.read_text(encoding="utf-8"))
    if not isinstance(artifact, Mapping):
        raise ValueError("Table 4 artifact manifest must be an object")
    _validate_table4_artifact_receipt(
        directory,
        artifact,
        manifest_content_sha256=str(declared),
        expected_n=expected_n,
        expected_j=expected_j,
    )
    return Table4Binding(
        directory=directory,
        manifest_path=manifest_path,
        state_records_path=state_path,
        asset_records_path=asset_path,
        artifact_manifest_path=artifact_path,
        manifest=manifest,
        cohort_assets=[dict(row) for row in cohort_assets],
        frozen_manifest_sha256=str(artifact["files"]["frozen_manifest.json"]["sha256"]),
        frozen_manifest_content_sha256=declared,
        state_records_sha256=str(artifact["files"]["state_records.jsonl"]["sha256"]),
        asset_records_sha256=str(artifact["files"]["asset_records.jsonl"]["sha256"]),
        artifact_manifest_sha256=sha256_file(artifact_path),
        ordered_ids_sha256=canonical_sha256(ordered_ids),
    )


def _file_receipt(path: Path, **metadata: Any) -> dict[str, Any]:
    resolved = path.resolve(strict=True)
    return {
        "path": str(resolved),
        "bytes": resolved.stat().st_size,
        "sha256": sha256_file(resolved),
        **metadata,
    }


def table4_input_receipt(binding: Table4Binding) -> dict[str, Any]:
    return {
        "directory": str(binding.directory),
        "frozen_manifest": _file_receipt(
            binding.manifest_path,
            content_sha256=binding.frozen_manifest_content_sha256,
        ),
        "state_records": _file_receipt(binding.state_records_path),
        "asset_records": _file_receipt(binding.asset_records_path),
        "artifact_manifest": _file_receipt(binding.artifact_manifest_path),
    }


def static_input_receipt() -> dict[str, Any]:
    static_atoms = SCRIPT.with_name("lam_supplementary_static.py")
    return {
        "canonical_cohort": _file_receipt(
            COHORT_MANIFEST, content_sha256=COHORT_CONTENT_SHA256
        ),
        "table2_cohort": _file_receipt(
            TABLE2_COHORT_MANIFEST, content_sha256=TABLE2_COHORT_CONTENT_SHA256
        ),
        "table3_manifest": _file_receipt(
            TABLE3_MANIFEST, content_sha256=TABLE3_MANIFEST_CONTENT_SHA256
        ),
        "table3_records": _file_receipt(TABLE3_RECORDS),
        "protocol_document": _file_receipt(PROTOCOL_DOCUMENT),
        "adapter_runner": _file_receipt(SCRIPT),
        "cpu_affinity_launcher": _file_receipt(CPU_AFFINITY_LAUNCHER),
        "shared_table4a_runner": _file_receipt(_SHARED_RUNNER_SCRIPT),
        "genesis_runner": _file_receipt(shared.lam4a.SCRIPT),
        "static_atoms": _file_receipt(static_atoms),
    }


def _joint_specs(urdf_path: Path) -> list[dict[str, Any]]:
    root = ET.parse(urdf_path).getroot()
    if root.tag != "robot":
        raise ValueError(f"URDF root is {root.tag!r}, expected 'robot'")
    specs: list[dict[str, Any]] = []
    for xml_index, joint in enumerate(root.findall("joint")):
        joint_type = str(joint.get("type", "")).strip()
        if joint_type == "fixed":
            continue
        name = str(joint.get("name") or f"joint_{xml_index}").strip()
        lower: float | None = None
        upper: float | None = None
        if joint_type in {"prismatic", "revolute"}:
            limit = joint.find("limit")
            if limit is not None:
                try:
                    lower = float(limit.attrib["lower"])
                    upper = float(limit.attrib["upper"])
                except (KeyError, ValueError):
                    lower = upper = None
        range_evaluable = joint_type == "continuous" or bool(
            joint_type in {"prismatic", "revolute"}
            and lower is not None
            and upper is not None
            and math.isfinite(lower)
            and math.isfinite(upper)
            and upper - lower > shared.ZERO_WIDTH_TOLERANCE
        )
        specs.append(
            {
                "name": name,
                "type": joint_type,
                "lower": lower,
                "upper": upper,
                "xml_index": xml_index,
                "range_evaluable": range_evaluable,
            }
        )
    return specs


def source_manifest_from_binding(binding: Table4Binding) -> dict[str, Any]:
    manifest = json.loads(json.dumps(binding.manifest))
    total_joints = 0
    for item, cohort in zip(manifest["items"], binding.cohort_assets):
        package = Path(str(cohort["package"])).resolve(strict=True)
        urdf_path = package / str(cohort["primary_urdf_relative_path"])
        specs = _joint_specs(urdf_path)
        if len(specs) != int(item["movable_dof_count"]):
            raise ValueError(f"movable joint count mismatch: {item['dataset_id']}")
        item["joint_specs"] = specs
        item["package_relpath"] = str(package)
        item["primary_urdf_relpath"] = str(urdf_path)
        item["package_binding"] = json.loads(json.dumps(cohort["package_binding"]))
        total_joints += len(specs)
    if total_joints != sum(int(item["movable_dof_count"]) for item in manifest["items"]):
        raise ValueError("normalized Table 4 joint denominator mismatch")
    return manifest


def load_source_manifest() -> dict[str, Any]:
    if _BOUND_TABLE4 is None:
        raise RuntimeError("Table 4 receipt has not been bound")
    return source_manifest_from_binding(_BOUND_TABLE4)


def load_table2_cohort_identity() -> None:
    load_canonical_cohort()


def load_table3_joint_pass() -> tuple[dict[str, dict[str, bool]], int]:
    if sha256_file(TABLE3_MANIFEST) != TABLE3_MANIFEST_FILE_SHA256:
        raise SystemExit("Table 3 manifest file hash mismatch")
    table3_manifest = json.loads(TABLE3_MANIFEST.read_text(encoding="utf-8"))
    if (
        table3_manifest.get("manifest_content_sha256") != manifest_self_hash(table3_manifest)
        or table3_manifest.get("manifest_content_sha256") != TABLE3_MANIFEST_CONTENT_SHA256
    ):
        raise SystemExit("Table 3 manifest self-hash mismatch")
    if sha256_file(TABLE3_RECORDS) != TABLE3_RECORDS_SHA256:
        raise SystemExit("Table 3 asset records hash mismatch")
    assets = load_canonical_cohort()
    expected = {str(row["dataset_id"]): row for row in assets}
    result: dict[str, dict[str, bool]] = {}
    joints_total = 0
    with TABLE3_RECORDS.open(encoding="utf-8") as handle:
        for line in handle:
            record = json.loads(line)
            dataset_id = str(record.get("dataset_id") or record.get("asset_key"))
            cohort = expected.get(dataset_id)
            if cohort is None or dataset_id in result:
                raise SystemExit(f"Table 3 duplicate or foreign asset: {dataset_id}")
            if (
                record.get("selection_index") != cohort["selection_index"]
                or record.get("package") != cohort["package"]
                or record.get("urdf_sha256") != cohort["urdf_sha256"]
                or record.get("package_content_manifest_sha256")
                != cohort["package_binding"]["content_manifest_sha256"]
            ):
                raise SystemExit(f"Table 3 cohort binding mismatch: {dataset_id}")
            passes: dict[str, bool] = {}
            for joint in record.get("joints") or []:
                name = str(joint["joint_name"])
                if name in passes:
                    raise SystemExit(f"Table 3 duplicate joint: {dataset_id}/{name}")
                passes[name] = bool(joint["joint_level_pass"])
                joints_total += 1
            result[dataset_id] = passes
    if len(result) != N_EVAL or joints_total != J_EVAL:
        raise SystemExit(f"Table 3 denominator mismatch: N={len(result)} J={joints_total}")
    return result, joints_total


def load_table4_strict_pass() -> dict[str, bool]:
    if _BOUND_TABLE4 is None or sha256_file(_BOUND_TABLE4.asset_records_path) != _BOUND_TABLE4.asset_records_sha256:
        raise SystemExit("Table 4 asset records hash mismatch")
    try:
        return _load_table4_strict_for_verification(
            _BOUND_TABLE4.asset_records_path,
            _BOUND_TABLE4.manifest["items"],
        )
    except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
        raise SystemExit(f"Table 4 strict collision receipt invalid: {exc}") from exc


def load_table4_state_hashes() -> dict[tuple[str, str, int], str]:
    if _BOUND_TABLE4 is None or sha256_file(_BOUND_TABLE4.state_records_path) != _BOUND_TABLE4.state_records_sha256:
        raise SystemExit("Table 4 state records hash mismatch")
    source = load_source_manifest()
    allowed = {
        (str(item["dataset_id"]), str(joint["name"]), sample)
        for item in source["items"]
        for joint in item["joint_specs"]
        for sample in range(SINGLE_SAMPLES)
    }
    index: dict[tuple[str, str, int], str] = {}
    with _BOUND_TABLE4.state_records_path.open(encoding="utf-8") as handle:
        for line in handle:
            record = json.loads(line)
            if record.get("phase") != "single_joint_sweep":
                continue
            key = (
                str(record.get("dataset_id")),
                str(record.get("joint_name")),
                int(record.get("sample_index", -1)),
            )
            if key not in allowed or key in index:
                raise SystemExit(f"Table 4 sweep state identity invalid or duplicated: {key}")
            digest = str(record.get("joint_values_sha256", ""))
            if len(digest) != 64:
                raise SystemExit(f"Table 4 sweep state hash invalid: {key}")
            index[key] = digest
    return index


def _observe_package_file_binding(job: Mapping[str, Any]) -> list[dict[str, Any]]:
    package = Path(str(job["package"]))
    if package.is_symlink():
        raise ValueError("package root must not be a symlink")
    package = package.resolve(strict=True)
    expected = job.get("expected_package_file_binding")
    expected_sha256 = job.get("expected_package_file_binding_sha256")
    if (
        not isinstance(expected, list)
        or not expected
        or canonical_sha256(expected) != expected_sha256
        or expected_sha256 != job.get("expected_package_content_manifest_sha256")
    ):
        raise ValueError("frozen package file binding is invalid")

    expected_paths: list[str] = []
    for row in expected:
        if not isinstance(row, Mapping):
            raise ValueError("frozen package file row is invalid")
        relative = Path(str(row.get("path", "")))
        if relative.is_absolute() or not relative.parts or ".." in relative.parts:
            raise ValueError(f"unsafe frozen package path: {relative}")
        expected_paths.append(relative.as_posix())
    if len(set(expected_paths)) != len(expected_paths):
        raise ValueError("frozen package file binding contains duplicate paths")

    actual_paths: set[str] = set()
    for candidate in package.rglob("*"):
        if candidate.is_symlink():
            raise ValueError(f"package contains symlink: {candidate.relative_to(package)}")
        if candidate.is_dir():
            continue
        if not candidate.is_file():
            raise ValueError(f"package contains unsupported entry: {candidate.relative_to(package)}")
        actual_paths.add(candidate.relative_to(package).as_posix())
    if actual_paths != set(expected_paths):
        missing = sorted(set(expected_paths) - actual_paths)
        extra = sorted(actual_paths - set(expected_paths))
        raise ValueError(f"package file roster drift: missing={missing} extra={extra}")

    observed: list[dict[str, Any]] = []
    for row, relative_text in zip(expected, expected_paths):
        candidate = package / relative_text
        path = candidate.resolve(strict=True)
        try:
            path.relative_to(package)
        except ValueError as error:
            raise ValueError(f"package file escapes root: {relative_text}") from error
        if candidate.is_symlink() or not path.is_file():
            raise ValueError(f"invalid package file: {relative_text}")
        observed.append(
            {
                "path": relative_text,
                "bytes": path.stat().st_size,
                "sha256": sha256_file(path),
            }
        )
    return observed


def build_jobs(
    manifest: Mapping[str, Any],
    table3_pass: Mapping[str, Mapping[str, bool]],
    state_hashes: Mapping[tuple[str, str, int], str],
    *,
    formal: bool,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    jobs: list[dict[str, Any]] = []
    category_rows: list[dict[str, str]] = []
    for index, item in enumerate(manifest["items"]):
        dataset_id = str(item["dataset_id"])
        asset_id = str(item["asset_id"])
        category = dataset_id.split("/", 2)[1]
        category_rows.append({"asset_id": asset_id, "category_slug": category})
        joint_jobs: list[dict[str, Any]] = []
        joints = sorted(item["joint_specs"], key=lambda row: int(row["xml_index"]))
        seen_names: set[str] = set()
        for position, row in enumerate(joints):
            name = str(row["name"])
            if name in seen_names:
                raise SystemExit(f"duplicate source joint name: {dataset_id}/{name}")
            seen_names.add(name)
            range_evaluable = bool(row.get("range_evaluable"))
            values = shared.single_joint_values(row) if range_evaluable else []
            joint_jobs.append(
                {
                    "name": name,
                    "type": str(row["type"]),
                    "lower": row.get("lower"),
                    "upper": row.get("upper"),
                    "xml_index": int(row["xml_index"]),
                    "movable_rank": position,
                    "dof_position": position,
                    "range_evaluable": range_evaluable,
                    "values": values,
                    "state_hash_references": [
                        state_hashes.get((dataset_id, name, sample_index))
                        for sample_index in range(SINGLE_SAMPLES)
                    ],
                    "table3_joint_level_pass": bool(
                        table3_pass.get(asset_id, {}).get(name, False)
                    ),
                    "table3_joint_present": name in table3_pass.get(asset_id, {}),
                }
            )
        package_binding = item.get("package_binding")
        if not isinstance(package_binding, Mapping) or not isinstance(
            package_binding.get("files"), list
        ):
            raise SystemExit(f"package binding missing from source job: {dataset_id}")
        expected_package_files = [
            {
                "path": str(row["path"]),
                "bytes": int(row["bytes"]),
                "sha256": str(row["sha256"]),
            }
            for row in package_binding["files"]
        ]
        expected_package_sha256 = canonical_sha256(expected_package_files)
        if expected_package_sha256 != package_binding.get("content_manifest_sha256"):
            raise SystemExit(f"package binding self-hash mismatch in source job: {dataset_id}")
        job = {
            "selection_index": index,
            "dataset_id": dataset_id,
            "asset_id": asset_id,
            "category": category,
            "package": str(item["package_relpath"]),
            "urdf_path": str(item["primary_urdf_relpath"]),
            "expected_urdf_sha256": str(item["urdf_sha256"]),
            "expected_package_content_manifest_sha256": expected_package_sha256,
            "expected_package_file_binding": expected_package_files,
            "expected_package_file_binding_sha256": expected_package_sha256,
            "input_identity_sha256": str(item["input_identity_sha256"]),
            "expected_movable_dof": int(item["movable_dof_count"]),
            "joints": joint_jobs,
            "expected_state_count": SINGLE_SAMPLES * len(joint_jobs),
        }
        try:
            observed_package_files = _observe_package_file_binding(job)
        except Exception as exc:  # noqa: BLE001
            raise SystemExit(
                f"package input audit failed before scheduling: {dataset_id}: "
                f"{type(exc).__name__}: {exc}"
            ) from exc
        if canonical_sha256(observed_package_files) != expected_package_sha256:
            raise SystemExit(f"package input drift before scheduling: {dataset_id}")
        jobs.append(job)
    category_mapping_hash = canonical_sha256(category_rows)
    category_info = {
        "category_records_root": str(COHORT_MANIFEST),
        "category_records_revision": COHORT_CONTENT_SHA256,
        "category_mapping_policy": "exact canonical cohort dataset_id/category join",
        "category_mapping_sha256": category_mapping_hash,
        "eval_category_count": len({row["category_slug"] for row in category_rows}),
    }
    if formal and (
        category_mapping_hash != EXPECTED_CATEGORY_MAPPING_SHA256
        or category_info["eval_category_count"] != EXPECTED_CATEGORY_COUNT
    ):
        raise SystemExit("formal category mapping binding mismatch")
    return jobs, category_info


def _invalid_joint_record(joint: Mapping[str, Any]) -> dict[str, Any]:
    is_bounded = joint["type"] != "continuous"
    return {
        "joint_name": str(joint["name"]),
        "joint_type": str(joint["type"]),
        "dof_position": int(joint["dof_position"]),
        "xml_index": int(joint["xml_index"]),
        "states_intended": SINGLE_SAMPLES,
        "states_executed": 0,
        "illegal_states": 0,
        "full_range_cf_pass": False,
        "limit_endpoints_intended": 2 if is_bounded else 0,
        "limit_endpoints_executed": 0,
        "limit_reachable": False,
        "table3_joint_level_pass": bool(joint["table3_joint_level_pass"]),
        "safe_dof": 0,
        "issues": ["joint_range_not_evaluable"],
        "state_summaries": [
            {
                "sample_index": sample_index,
                "executed": False,
                "issue": "joint_range_not_evaluable",
            }
            for sample_index in range(SINGLE_SAMPLES)
        ],
    }


def failed_asset_record(job: Mapping[str, Any], issue: str) -> dict[str, Any]:
    record = _SHARED_FAILED_ASSET_RECORD(job, issue)
    joint_records = record.get("joint_records")
    job_joints = job.get("joints")
    if not isinstance(joint_records, list) or not isinstance(job_joints, list):
        raise ValueError("failed asset record requires frozen joint records")
    if len(joint_records) != len(job_joints):
        raise ValueError("failed asset record joint count mismatch")
    for joint_record, joint in zip(joint_records, job_joints):
        if not isinstance(joint_record, dict) or not isinstance(joint, Mapping):
            raise ValueError("failed asset record joint identity is malformed")
        joint_record["dof_position"] = int(joint["dof_position"])
        joint_record["xml_index"] = int(joint["xml_index"])
    record.update(
        {
            "schema_version": SCHEMA_VERSION,
            "protocol_id": PROTOCOL_ID,
            "input_identity_sha256": str(job["input_identity_sha256"]),
            "expected_package_content_manifest_sha256": job.get(
                "expected_package_content_manifest_sha256"
            ),
            "expected_package_file_binding": job.get("expected_package_file_binding"),
            "expected_package_file_binding_sha256": job.get(
                "expected_package_file_binding_sha256"
            ),
            "observed_package_file_binding_before": None,
            "observed_package_file_binding_before_sha256": None,
            "observed_package_file_binding_after": None,
            "observed_package_file_binding_after_sha256": None,
            "early_cpu_affinity": _unobserved_early_cpu_affinity(),
            "torch_threading": _unobserved_torch_threading(),
        }
    )
    return record


def run_child(job_path: Path, result_path: Path) -> int:
    job = json.loads(job_path.read_text(encoding="utf-8"))
    try:
        early_affinity = validate_early_cpu_affinity_receipt()
    except Exception as exc:  # noqa: BLE001
        record = failed_asset_record(
            job,
            f"early_cpu_affinity_failed: {type(exc).__name__}: {exc}",
        )
        record["early_cpu_affinity"] = {
            "status": "FAILED",
            "pid": os.getpid(),
            "requested": None,
            "observed": sorted(int(cpu) for cpu in os.sched_getaffinity(0)),
        }
        atomic_json(result_path, record)
        return 0
    try:
        torch_before = bind_torch_threading()
    except Exception as exc:  # noqa: BLE001
        record = failed_asset_record(
            job,
            f"torch_thread_binding_failed: {type(exc).__name__}: {exc}",
        )
        record["torch_threading"] = {
            "status": "FAILED",
            "expected": dict(TORCH_THREAD_COUNTS),
            "before_evaluation": None,
            "after_evaluation": None,
        }
        record["early_cpu_affinity"] = early_affinity
        atomic_json(result_path, record)
        return 0

    def finalize(record: dict[str, Any]) -> dict[str, Any]:
        record["early_cpu_affinity"] = dict(early_affinity)
        return _complete_torch_threading(record, torch_before)

    expected = job.get("expected_package_file_binding")
    expected_sha256 = job.get("expected_package_file_binding_sha256")
    if (
        not isinstance(expected, list)
        or canonical_sha256(expected) != expected_sha256
        or expected_sha256 != job.get("expected_package_content_manifest_sha256")
    ):
        record = failed_asset_record(job, "invalid_frozen_package_file_binding")
        atomic_json(result_path, finalize(record))
        return 0
    try:
        observed_before = _observe_package_file_binding(job)
        observed_before_sha256 = canonical_sha256(observed_before)
    except Exception as exc:  # noqa: BLE001
        record = failed_asset_record(
            job,
            f"input_binding_check_failed_before_evaluation: {type(exc).__name__}: {exc}",
        )
        atomic_json(result_path, finalize(record))
        return 0
    if observed_before_sha256 != expected_sha256:
        record = failed_asset_record(
            job,
            "input_binding_drift_before_evaluation: "
            f"expected {expected_sha256}, observed {observed_before_sha256}",
        )
        record["observed_package_file_binding_before"] = observed_before
        record["observed_package_file_binding_before_sha256"] = observed_before_sha256
        atomic_json(result_path, finalize(record))
        return 0

    return_code = _SHARED_RUN_CHILD(job_path, result_path)
    record = json.loads(result_path.read_text(encoding="utf-8"))
    record["input_identity_sha256"] = str(job["input_identity_sha256"])
    if isinstance(record.get("child"), dict):
        record["child"]["pid"] = os.getpid()
    try:
        observed_after = _observe_package_file_binding(job)
        observed_after_sha256 = canonical_sha256(observed_after)
    except Exception as exc:  # noqa: BLE001
        record = failed_asset_record(
            job,
            f"input_binding_check_failed_after_evaluation: {type(exc).__name__}: {exc}",
        )
        record["observed_package_file_binding_before"] = observed_before
        record["observed_package_file_binding_before_sha256"] = observed_before_sha256
        atomic_json(result_path, finalize(record))
        return 0
    if observed_after_sha256 != expected_sha256:
        record = failed_asset_record(
            job,
            "input_binding_drift_after_evaluation: "
            f"expected {expected_sha256}, observed {observed_after_sha256}",
        )
        record["observed_package_file_binding_before"] = observed_before
        record["observed_package_file_binding_before_sha256"] = observed_before_sha256
        record["observed_package_file_binding_after"] = observed_after
        record["observed_package_file_binding_after_sha256"] = observed_after_sha256
        atomic_json(result_path, finalize(record))
        return 0

    record["expected_package_content_manifest_sha256"] = job.get(
        "expected_package_content_manifest_sha256"
    )
    record["expected_package_file_binding"] = expected
    record["expected_package_file_binding_sha256"] = expected_sha256
    record["observed_package_file_binding_before"] = observed_before
    record["observed_package_file_binding_before_sha256"] = observed_before_sha256
    record["observed_package_file_binding_after"] = observed_after
    record["observed_package_file_binding_after_sha256"] = observed_after_sha256
    atomic_json(result_path, finalize(record))
    return return_code


def package_binding_attestation_valid(record: Mapping[str, Any]) -> bool:
    expected = record.get("expected_package_file_binding")
    expected_sha256 = record.get("expected_package_file_binding_sha256")
    if (
        not isinstance(expected, list)
        or not expected
        or canonical_sha256(expected) != expected_sha256
        or record.get("expected_package_content_manifest_sha256") != expected_sha256
    ):
        return False
    issues = [str(issue) for issue in (record.get("issues") or [])]
    if any(
        issue.startswith("input_binding_")
        or issue == "invalid_frozen_package_file_binding"
        for issue in issues
    ):
        return False
    pairs = (
        (
            record.get("observed_package_file_binding_before"),
            record.get("observed_package_file_binding_before_sha256"),
        ),
        (
            record.get("observed_package_file_binding_after"),
            record.get("observed_package_file_binding_after_sha256"),
        ),
    )
    for observed, observed_sha256 in pairs:
        if observed is None and observed_sha256 is None:
            continue
        if (
            observed != expected
            or observed_sha256 != expected_sha256
            or canonical_sha256(observed) != observed_sha256
        ):
            return False
    if record.get("status") == "completed":
        return all(
            observed == expected and observed_sha256 == expected_sha256
            for observed, observed_sha256 in pairs
        )
    return True


def package_binding_matches_source_item(
    record: Mapping[str, Any], item: Mapping[str, Any]
) -> bool:
    try:
        return (
            int(record.get("selection_index", -1)) == int(item.get("order", -2))
            and str(record.get("dataset_id", "")) == str(item.get("dataset_id", ""))
            and Path(str(record.get("package", ""))).resolve(strict=True)
            == Path(str(item.get("package", ""))).resolve(strict=True)
            and record.get("expected_package_content_manifest_sha256")
            == item.get("package_binding_content_manifest_sha256")
            == record.get("expected_package_file_binding_sha256")
            == canonical_sha256(record.get("expected_package_file_binding"))
        )
    except (OSError, TypeError, ValueError):
        return False


def current_package_binding_matches(record: Mapping[str, Any]) -> bool:
    try:
        observed = _observe_package_file_binding(record)
        return (
            canonical_sha256(observed)
            == record.get("expected_package_file_binding_sha256")
        )
    except Exception:  # noqa: BLE001
        return False


def spawn_children(
    jobs: Sequence[Mapping[str, Any]],
    outdir: Path,
    *,
    workers: int,
    timeout_seconds: float,
) -> list[dict[str, Any]]:
    runnable_jobs: list[dict[str, Any]] = []
    for job in jobs:
        runnable = dict(job)
        runnable["joints"] = [
            dict(joint) for joint in job["joints"] if bool(joint.get("range_evaluable"))
        ]
        runnable["expected_state_count"] = SINGLE_SAMPLES * len(runnable["joints"])
        runnable_jobs.append(runnable)
    records = _SHARED_SPAWN_CHILDREN(
        runnable_jobs,
        outdir,
        workers=workers,
        timeout_seconds=timeout_seconds,
    )
    return merge_range_failures(jobs, records)


def merge_range_failures(
    jobs: Sequence[Mapping[str, Any]],
    records: Sequence[dict[str, Any]],
) -> list[dict[str, Any]]:
    by_index = {int(record["selection_index"]): record for record in records}
    merged: list[dict[str, Any]] = []
    for job in jobs:
        record = by_index[int(job["selection_index"])]
        existing = {str(row["joint_name"]): row for row in record.get("joint_records", [])}
        invalid_names: list[str] = []
        joint_records = []
        for joint in job["joints"]:
            name = str(joint["name"])
            if bool(joint.get("range_evaluable")):
                joint_records.append(existing[name])
            else:
                invalid_names.append(name)
                joint_records.append(_invalid_joint_record(joint))
        record["joint_records"] = joint_records
        record["states_intended"] = int(job["expected_state_count"])
        if invalid_names:
            record["status"] = "error"
            issues = list(record.get("issues") or [])
            issues.extend(f"joint_range_not_evaluable:{name}" for name in invalid_names)
            record["issues"] = issues
        merged.append(record)
    merged.sort(key=lambda row: int(row["selection_index"]))
    return merged


def hash_cross_check_covers_executed_states(aggregates: Mapping[str, Any]) -> bool:
    counts = aggregates["state_counts"]
    cross = counts["hash_cross_check"]
    classified = sum(int(cross[key]) for key in ("verified", "mismatch", "no_reference"))
    return classified == int(counts["executed"]) and int(cross["mismatch"]) == 0


def verify_run(
    manifest: Mapping[str, Any],
    records: Sequence[Mapping[str, Any]],
    aggregates: Mapping[str, Any],
    table4_strict: Mapping[str, bool],
    category_info: Mapping[str, Any],
) -> dict[str, Any]:
    checks: list[dict[str, Any]] = []

    def check(name: str, passed: bool, detail: str = "") -> None:
        checks.append({"check": name, "pass": bool(passed), "detail": detail})

    assert _BOUND_TABLE4 is not None
    items = manifest["items"]
    check("source_manifest_file_sha256", sha256_file(_BOUND_TABLE4.manifest_path) == _BOUND_TABLE4.frozen_manifest_sha256)
    check("table4_state_records_sha256", sha256_file(_BOUND_TABLE4.state_records_path) == _BOUND_TABLE4.state_records_sha256)
    check("table4_asset_records_sha256", sha256_file(_BOUND_TABLE4.asset_records_path) == _BOUND_TABLE4.asset_records_sha256)
    check("canonical_cohort_file_sha256", sha256_file(COHORT_MANIFEST) == COHORT_FILE_SHA256)
    check("table2_cohort_file_sha256", sha256_file(TABLE2_COHORT_MANIFEST) == TABLE2_COHORT_FILE_SHA256)
    check("table3_records_sha256", sha256_file(TABLE3_RECORDS) == TABLE3_RECORDS_SHA256)
    check("record_count", len(records) == N_EVAL, str(len(records)))
    check(
        "frozen_order_preserved",
        all(
            int(record["selection_index"]) == index
            and str(record["dataset_id"]) == str(items[index]["dataset_id"])
            for index, record in enumerate(records)
        ),
    )
    check("j_eval_denominator", aggregates["joint_level_full_range_cf"]["denominator"] == J_EVAL)
    check("retention_denominator", aggregates["collision_safe_dof_retention"]["denominator"] == J_EVAL)
    check("state_intended_count", aggregates["state_counts"]["intended"] == EXPECTED_SINGLE_STATES)
    check("state_hash_cross_check_complete", hash_cross_check_covers_executed_states(aggregates))
    check(
        "package_input_bindings_attested",
        all(
            package_binding_attestation_valid(record)
            and 0 <= int(record.get("selection_index", -1)) < len(items)
            and package_binding_matches_source_item(
                record, items[int(record["selection_index"])]
            )
            for record in records
        ),
    )
    check(
        "early_cpu_affinity_attested",
        all(early_cpu_affinity_attestation_valid(record) for record in records),
    )
    check(
        "torch_thread_controls_attested",
        all(torch_threading_attestation_valid(record) for record in records),
    )
    check("category_mapping_sha256", category_info.get("category_mapping_sha256") == EXPECTED_CATEGORY_MAPPING_SHA256)
    check("category_count", category_info.get("eval_category_count") == EXPECTED_CATEGORY_COUNT)
    recomputed = shared.aggregate(records, table4_strict)
    check("aggregate_recomputation_matches", canonical_sha256(recomputed) == canonical_sha256(dict(aggregates)))
    return {"all_pass": all(row["pass"] for row in checks), "check_count": len(checks), "checks": checks}


def configure_shared_runner(binding: Table4Binding | None) -> None:
    shared.SCRIPT = SCRIPT
    shared.SCHEMA_VERSION = SCHEMA_VERSION
    shared.PROTOCOL_ID = PROTOCOL_ID
    shared.DATASET = DATASET_LABEL
    shared.DATASET_ROOT = REPO
    shared.PROTOCOL_DOCUMENT = PROTOCOL_DOCUMENT
    shared.N_EVAL = N_EVAL
    shared.J_EVAL = J_EVAL
    shared.EXPECTED_CATEGORY_COUNT = EXPECTED_CATEGORY_COUNT
    shared.SINGLE_SAMPLES = SINGLE_SAMPLES
    shared.CHILD_TIMEOUT_SECONDS = CHILD_TIMEOUT_SECONDS
    shared.WORKERS = FORMAL_WORKERS
    shared.PRIVATE_GENESIS_CACHES = True
    shared.EARLY_CPU_AFFINITY_LAUNCHER = CPU_AFFINITY_LAUNCHER
    shared.EXPECTED_EARLY_CPU_AFFINITY_LAUNCHER_RECEIPT = None
    shared.LAUNCH_STAGGER_SECONDS = 1.5
    shared.GENESIS_CACHE_POLICY = (
        "per-rank private GS_CACHE_FILE_PATH; rank 1 warms a read template copied to later ranks"
    )
    shared.TABLE2_COHORT_MANIFEST = TABLE2_COHORT_MANIFEST
    shared.EXPECTED_TABLE2_COHORT_FILE_SHA256 = TABLE2_COHORT_FILE_SHA256
    shared.EXPECTED_TABLE2_COHORT_CONTENT_SHA256 = TABLE2_COHORT_CONTENT_SHA256
    shared.TABLE3_RECORDS = TABLE3_RECORDS
    shared.EXPECTED_TABLE3_RECORDS_SHA256 = TABLE3_RECORDS_SHA256
    shared.CATEGORY_RECORDS_ROOT = COHORT_MANIFEST.parent
    shared.EXPECTED_CATEGORY_MAPPING_SHA256 = EXPECTED_CATEGORY_MAPPING_SHA256
    shared.EXPECTED_CATEGORY_RECORDS_REVISION = COHORT_CONTENT_SHA256
    shared.SELECTION_POLICY = (
        "exact frozen PV-A per-class N=5 cohort order with fence/ferris max-joint overrides; "
        "no resampling or result-based filtering"
    )
    shared.RUN_NOTES = [
        "The intent denominator is 314,328 states (14,968 joints x K=21). Genesis replay is attempted only for the 313,803 states of 14,943 range-evaluable joints; the 525 states belonging to 25 zero-width joints are not submitted to Genesis and remain explicit fail-closed records.",
        "For states that execute in Genesis, available frozen Table 4 q-vector hashes are cross-checked; missing references remain explicitly classified as no_reference and hash mismatches fail verification.",
        "Every package file is bound by the canonical cohort manifest and rehashed before scheduling plus immediately before and after each Genesis child; drift invalidates publication.",
        "State collision oracle = Genesis contact-penetration backend; penetration > 1e-6 m is illegal and all failures remain fail closed.",
        "PyTorch intra-op and inter-op pools are both fixed to one thread before Genesis import and read back before and after every child evaluation.",
        "Every Genesis child binds and reads back its four-core CPU affinity in a minimal -S launcher before importing the child runner, then execs the same pinned interpreter and records the same-PID receipt.",
        "Formal concurrency is fixed at one worker because both six-worker and two-worker calibration reproduced host-wide native-thread exhaustion, while an isolated replay of the same failed asset completed; this execution control does not change cohort, state, oracle, or denominator semantics.",
        "Headline pair policy excludes direct parent-child pairs and applies no method-specific allowance.",
        "Normalized Clearance P5 is N/E because this oracle has no separated-pair signed clearance.",
        "Existing Strict Collision Pass is read from the explicitly bound Table 4 receipt.",
        RESUME_POLICY,
    ]
    shared.load_source_manifest = load_source_manifest
    shared.load_table2_cohort_identity = load_table2_cohort_identity
    shared.load_table3_joint_pass = load_table3_joint_pass
    shared.load_table4_strict_pass = load_table4_strict_pass
    shared.load_table4_state_hashes = load_table4_state_hashes
    shared.build_jobs = build_jobs
    shared.spawn_children = spawn_children
    shared._failed_asset_record = failed_asset_record
    shared._read_category = lambda asset_id: str(asset_id).split("/", 2)[1]
    shared._category_revision = lambda: COHORT_CONTENT_SHA256
    shared.verify_run = verify_run
    operationalization = dict(shared.OPERATIONALIZATION)
    operationalization.update(
        {
            "state_plan": (
                "Full intent denominator = 14,968 movable joints x K=21 = 314,328 states. "
                "Only 14,943 range-evaluable joints (313,803 states) enter Genesis replay; "
                "25 zero-width joints (525 states) are not replayed and are retained fail closed. "
                "Executed states cross-check available frozen Table 4 q-vector hashes, with "
                "mismatch forbidden and no_reference reported explicitly."
            ),
            "table3_joint_pass_source": f"{TABLE3_RECORDS} (sha256 {TABLE3_RECORDS_SHA256})",
            "existing_strict_collision_pass_source": "explicit --table4-dir asset_records.jsonl",
            "category_source": "category embedded in the canonical PV-A cohort manifest",
            "percentile_policy": "linear interpolation over all 2,655 frozen assets including fail-closed zeros",
            "package_input_binding": (
                "exact recursive package file roster, byte count and SHA-256 from the canonical "
                "cohort package binding; audited before scheduling and before/after each child"
            ),
            "torch_threading": TORCH_THREADING_POLICY,
            "early_cpu_affinity": EARLY_CPU_AFFINITY_POLICY,
            "early_cpu_affinity_launcher_binding": _file_receipt(
                CPU_AFFINITY_LAUNCHER
            ),
            "resume_policy": RESUME_POLICY,
            "shared_table4a_runner_binding": _file_receipt(_SHARED_RUNNER_SCRIPT),
            "source_table4_artifact_manifest_binding": (
                _file_receipt(binding.artifact_manifest_path) if binding is not None else None
            ),
        }
    )
    shared.OPERATIONALIZATION = operationalization
    if binding is not None:
        shared.SOURCE_MANIFEST = binding.manifest_path
        shared.EXPECTED_SOURCE_MANIFEST_FILE_SHA256 = binding.frozen_manifest_sha256
        shared.EXPECTED_SOURCE_MANIFEST_CONTENT_SHA256 = binding.frozen_manifest_content_sha256
        shared.EXPECTED_ORDERED_IDS_SHA256 = binding.ordered_ids_sha256
        shared.TABLE4_STATE_RECORDS = binding.state_records_path
        shared.EXPECTED_TABLE4_STATE_RECORDS_SHA256 = binding.state_records_sha256
        shared.TABLE4_ASSET_RECORDS = binding.asset_records_path
        shared.EXPECTED_TABLE4_ASSET_RECORDS_SHA256 = binding.asset_records_sha256


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open(encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            if not line.strip():
                continue
            row = json.loads(line)
            if not isinstance(row, dict):
                raise ValueError(f"{path.name}:{line_number} is not an object")
            rows.append(row)
    return rows


def _derive_receipt_counts_from_rows(
    assets: Sequence[Mapping[str, Any]],
    joints: Sequence[Mapping[str, Any]],
) -> dict[str, int]:
    invalid = 0
    for row in joints:
        issues = row.get("issues") or []
        if "joint_range_not_evaluable" in issues:
            invalid += 1
    intended = sum(int(row.get("states_intended", 0)) for row in assets)
    executed = sum(int(row.get("states_executed", 0)) for row in assets)
    replay_intended = sum(
        int(row.get("states_intended", 0))
        for row in joints
        if "joint_range_not_evaluable" not in (row.get("issues") or [])
    )
    fail_closed_without_replay = intended - replay_intended
    return {
        "n_eval": len(assets),
        "j_eval": len(joints),
        "range_evaluable_joints": len(joints) - invalid,
        "range_invalid_joints": invalid,
        "states_intended": intended,
        "states_genesis_replay_intended": replay_intended,
        "states_fail_closed_without_replay": fail_closed_without_replay,
        "states_executed": executed,
    }


def _derive_receipt_counts(output: Path) -> dict[str, int]:
    return _derive_receipt_counts_from_rows(
        _read_jsonl(output / "asset_records.jsonl"),
        _read_jsonl(output / "joint_records.jsonl"),
    )


def _nonnegative_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0


def execution_record_invariants_valid(
    asset_rows: Sequence[Mapping[str, Any]],
    *,
    formal: bool,
) -> bool:
    total_executed = 0
    for asset in asset_rows:
        if not early_cpu_affinity_attestation_valid(
            asset
        ) or not torch_threading_attestation_valid(asset):
            return False
        joints = asset.get("joint_records")
        asset_intended = asset.get("states_intended")
        asset_executed = asset.get("states_executed")
        expected_movable = asset.get("expected_movable_dof")
        if (
            not isinstance(joints, list)
            or not _nonnegative_int(asset_intended)
            or not _nonnegative_int(asset_executed)
            or asset_executed > asset_intended
            or not _nonnegative_int(expected_movable)
            or expected_movable != len(joints)
        ):
            return False

        joint_intended_total = 0
        joint_executed_total = 0
        for joint in joints:
            if not isinstance(joint, Mapping):
                return False
            intended = joint.get("states_intended")
            executed = joint.get("states_executed")
            illegal = joint.get("illegal_states")
            safe_dof = joint.get("safe_dof")
            full_range_pass = joint.get("full_range_cf_pass")
            table3_pass = joint.get("table3_joint_level_pass")
            joint_type = joint.get("joint_type")
            endpoints_intended = joint.get("limit_endpoints_intended")
            endpoints_executed = joint.get("limit_endpoints_executed")
            limit_reachable = joint.get("limit_reachable")
            bounded = joint_type != "continuous"
            if (
                not _nonnegative_int(intended)
                or intended != SINGLE_SAMPLES
                or not _nonnegative_int(executed)
                or executed > intended
                or not _nonnegative_int(illegal)
                or illegal > executed
                or not isinstance(safe_dof, int)
                or isinstance(safe_dof, bool)
                or not 0 <= safe_dof <= 1
                or not isinstance(full_range_pass, bool)
                or (full_range_pass and (executed != intended or illegal != 0))
                or not isinstance(table3_pass, bool)
                or safe_dof != int(full_range_pass and table3_pass)
                or not isinstance(joint_type, str)
                or not _nonnegative_int(endpoints_intended)
                or endpoints_intended != (2 if bounded else 0)
                or not _nonnegative_int(endpoints_executed)
                or endpoints_executed > endpoints_intended
                or endpoints_executed > executed
                or not isinstance(limit_reachable, bool)
                or limit_reachable
                != bool(bounded and full_range_pass and endpoints_executed == 2)
            ):
                return False
            issues = joint.get("issues") or []
            if not isinstance(issues, list):
                return False
            if "joint_range_not_evaluable" in issues and executed != 0:
                return False

            summaries = joint.get("state_summaries")
            if summaries is None:
                if executed != 0:
                    return False
            elif not isinstance(summaries, list) or len(summaries) != intended:
                return False
            else:
                executed_summaries = 0
                illegal_summaries = 0
                for sample_index, state in enumerate(summaries):
                    if (
                        not isinstance(state, Mapping)
                        or state.get("sample_index") != sample_index
                        or not isinstance(state.get("executed"), bool)
                    ):
                        return False
                    if state["executed"]:
                        executed_summaries += 1
                        illegal_collision = state.get("illegal_collision", False)
                        if not isinstance(illegal_collision, bool):
                            return False
                        illegal_summaries += int(illegal_collision)
                if executed_summaries != executed or illegal_summaries != illegal:
                    return False
            joint_intended_total += intended
            joint_executed_total += executed

        cross = asset.get("state_hash_cross_check")
        if not isinstance(cross, Mapping) or set(cross) != {
            "verified",
            "mismatch",
            "no_reference",
        }:
            return False
        cross_values = [cross[key] for key in ("verified", "mismatch", "no_reference")]
        if (
            not all(_nonnegative_int(value) for value in cross_values)
            or sum(cross_values) != asset_executed
            or (formal and cross["mismatch"] != 0)
            or joint_intended_total != asset_intended
            or joint_executed_total != asset_executed
        ):
            return False
        total_executed += asset_executed
    return not formal or total_executed <= EXPECTED_GENESIS_REPLAY_STATES


def _flatten_joint_records(
    asset_rows: Sequence[Mapping[str, Any]],
) -> list[dict[str, Any]]:
    flattened: list[dict[str, Any]] = []
    for record in asset_rows:
        for joint in record.get("joint_records", []):
            flattened.append(
                {
                    "dataset_id": record["dataset_id"],
                    "asset_id": record.get("asset_id"),
                    "category": record.get("category"),
                    "selection_index": record["selection_index"],
                    "asset_status": record.get("status"),
                    **{key: value for key, value in joint.items() if key != "state_summaries"},
                }
            )
    return flattened


def _load_table4_strict_for_verification(
    path: Path,
    source_items: Sequence[Mapping[str, Any]],
) -> dict[str, bool]:
    expected_ids = [str(item["dataset_id"]) for item in source_items]
    strict: dict[str, bool] = {}
    seen_orders: set[int] = set()
    with path.open(encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            if not line.strip():
                continue
            record = json.loads(line)
            if not isinstance(record, Mapping):
                raise ValueError(f"Table 4 asset record line {line_number} is not an object")
            order = record.get("order", record.get("selection_index"))
            dataset_id = str(record.get("dataset_id", ""))
            strict_pass = record.get("strict_collision_pass")
            if (
                not isinstance(order, int)
                or isinstance(order, bool)
                or not 0 <= order < len(expected_ids)
                or order in seen_orders
                or dataset_id != expected_ids[order]
                or not isinstance(strict_pass, bool)
                or dataset_id in strict
            ):
                raise ValueError(
                    f"Table 4 asset record identity invalid at line {line_number}"
                )
            seen_orders.add(order)
            strict[dataset_id] = strict_pass
    if len(strict) != len(expected_ids) or len(seen_orders) != len(expected_ids):
        raise ValueError("Table 4 asset record count mismatch")
    return strict


def _recompute_summary_metrics(
    asset_rows: Sequence[Mapping[str, Any]],
    table4_strict: Mapping[str, bool],
) -> dict[str, Any]:
    previous_j_eval = shared.J_EVAL
    try:
        shared.J_EVAL = J_EVAL
        return shared.aggregate(asset_rows, table4_strict)
    finally:
        shared.J_EVAL = previous_j_eval


def _render_summary_markdown(summary: Mapping[str, Any]) -> str:
    previous_dataset = shared.DATASET
    try:
        shared.DATASET = DATASET_LABEL
        return shared.render_summary_md(summary)
    finally:
        shared.DATASET = previous_dataset


def output_records_bind_source(
    asset_rows: Sequence[Mapping[str, Any]],
    source_items: Sequence[Mapping[str, Any]],
    *,
    table3_pass: Mapping[str, Mapping[str, bool]] | None,
) -> bool:
    if len(asset_rows) > len(source_items):
        return False
    try:
        for index, (asset, item) in enumerate(zip(asset_rows, source_items)):
            if not isinstance(asset, Mapping) or not isinstance(item, Mapping):
                return False
            dataset_id = str(item["dataset_id"])
            package = Path(str(item["package"])).resolve(strict=True)
            urdf = Path(str(item["primary_urdf_relpath"]))
            if not urdf.is_absolute():
                urdf = package.parent / urdf
            urdf = urdf.resolve(strict=True)
            specs = _joint_specs(urdf)
            joints = asset.get("joint_records")
            if (
                asset.get("selection_index") != index
                or asset.get("dataset_id") != dataset_id
                or asset.get("asset_id") != item.get("asset_id")
                or asset.get("category") != item.get("category")
                or asset.get("input_identity_sha256")
                != item.get("input_identity_sha256")
                or Path(str(asset.get("package", ""))).resolve(strict=True) != package
                or asset.get("expected_urdf_sha256") != item.get("urdf_sha256")
                or asset.get("expected_movable_dof") != len(specs)
                or item.get("movable_dof_count") != len(specs)
                or not isinstance(joints, list)
                or len(joints) != len(specs)
            ):
                return False
            observed_urdf_sha256 = asset.get("urdf_sha256")
            if observed_urdf_sha256 is not None and observed_urdf_sha256 != item.get(
                "urdf_sha256"
            ):
                return False

            expected_table3 = table3_pass.get(dataset_id) if table3_pass is not None else None
            if table3_pass is not None and not isinstance(expected_table3, Mapping):
                return False
            for position, (joint, spec) in enumerate(zip(joints, specs)):
                if not isinstance(joint, Mapping):
                    return False
                issues = joint.get("issues") or []
                if not isinstance(issues, list):
                    return False
                if (
                    joint.get("joint_name") != spec["name"]
                    or joint.get("joint_type") != spec["type"]
                    or joint.get("xml_index") != spec["xml_index"]
                    or joint.get("dof_position") != position
                    or ("joint_range_not_evaluable" in issues)
                    == bool(spec["range_evaluable"])
                ):
                    return False
                table3_value = joint.get("table3_joint_level_pass")
                if not isinstance(table3_value, bool):
                    return False
                if expected_table3 is not None and (
                    spec["name"] not in expected_table3
                    or table3_value is not bool(expected_table3[spec["name"]])
                ):
                    return False
        return True
    except (KeyError, OSError, TypeError, ValueError, ET.ParseError):
        return False


def _receipt_matches_file(receipt: Any, *, expected_parent: Path | None = None) -> bool:
    if not isinstance(receipt, Mapping):
        return False
    try:
        byte_count = receipt["bytes"]
        expected_sha256 = receipt["sha256"]
        if (
            not isinstance(byte_count, int)
            or isinstance(byte_count, bool)
            or not isinstance(expected_sha256, str)
            or len(expected_sha256) != 64
        ):
            return False
        path = Path(str(receipt["path"]))
        if not path.is_absolute() or path.is_symlink() or not path.is_file():
            return False
        resolved = path.resolve(strict=True)
        if expected_parent is not None and resolved.parent != expected_parent:
            return False
        return (
            resolved.stat().st_size == byte_count
            and sha256_file(resolved) == expected_sha256
        )
    except (KeyError, OSError, TypeError, ValueError):
        return False


def _self_hashed_manifest_receipt_matches(
    receipt: Mapping[str, Any],
    *,
    expected_file_sha256: str,
    expected_content_sha256: str,
) -> bool:
    if (
        receipt.get("sha256") != expected_file_sha256
        or receipt.get("content_sha256") != expected_content_sha256
    ):
        return False
    manifest = json.loads(Path(str(receipt["path"])).read_text(encoding="utf-8"))
    return (
        manifest.get("manifest_content_sha256")
        == manifest_self_hash(manifest)
        == expected_content_sha256
    )


def _timestamp(value: Any) -> dt.datetime:
    parsed = dt.datetime.fromisoformat(str(value).replace("Z", "+00:00"))
    if parsed.tzinfo is None:
        raise ValueError("timestamp must include timezone")
    return parsed


def formal_verification_receipt_valid(
    receipt: Any,
    run_manifest_receipt: Any,
) -> bool:
    if (
        not isinstance(receipt, Mapping)
        or not isinstance(run_manifest_receipt, Mapping)
        or receipt != run_manifest_receipt
        or receipt.get("all_pass") is not True
        or receipt.get("check_count") != len(FORMAL_VERIFICATION_CHECK_NAMES)
    ):
        return False
    rows = receipt.get("checks")
    return bool(
        isinstance(rows, list)
        and len(rows) == len(FORMAL_VERIFICATION_CHECK_NAMES)
        and tuple(row.get("check") for row in rows if isinstance(row, Mapping))
        == FORMAL_VERIFICATION_CHECK_NAMES
        and all(
            isinstance(row, Mapping)
            and set(row) == {"check", "detail", "pass"}
            and row.get("pass") is True
            for row in rows
        )
    )


def verify_output_receipt(
    output: Path,
    *,
    expected_table4_dir: Path | None = None,
) -> dict[str, Any]:
    check_names = (
        "artifact_self_hash_valid",
        "artifact_metadata_valid",
        "artifact_files_closed",
        "frozen_config_self_hash_valid",
        "protocol_snapshot_binding",
        "source_table4_binding_consistent",
        "source_table4_files_match",
        "static_inputs_match",
        "package_input_bindings_match",
        "execution_controls_match",
        "output_hashes_match",
        "record_counts_match",
        "execution_invariants_valid",
        "output_source_bindings_match",
        "derived_outputs_match",
        "denominators_match",
        "wall_clock_valid",
        "verification_status_valid",
    )
    checks = {name: False for name in check_names}
    issues: list[str] = []
    try:
        outdir = output.resolve(strict=True)
        if not outdir.is_dir():
            raise ValueError("output is not a directory")
        artifact_path = outdir / "artifact_manifest.json"
        if artifact_path.is_symlink() or not artifact_path.is_file():
            raise ValueError("artifact manifest must be a regular non-symlink file")
        artifact = json.loads(artifact_path.read_text(encoding="utf-8"))
        if not isinstance(artifact, dict):
            raise ValueError("artifact manifest is not an object")
        checks["artifact_self_hash_valid"] = artifact.get(
            "artifact_manifest_content_sha256"
        ) == artifact_manifest_self_hash(artifact)
        mode = str(artifact.get("mode", ""))
        classification = "FORMAL" if mode == "formal" else "SMOKE"
        files = artifact.get("files")
        checks["artifact_metadata_valid"] = (
            artifact.get("schema_version") == ARTIFACT_SCHEMA_VERSION
            and artifact.get("protocol_id") == PROTOCOL_ID
            and artifact.get("dataset") == DATASET_LABEL
            and artifact.get("classification") == classification
            and mode in {"smoke", "formal"}
            and artifact.get("excludes") == ARTIFACT_EXCLUSIONS
            and isinstance(files, Mapping)
            and set(files) == set(RECEIPT_FILES)
        )
        allowed_top_level = set(RECEIPT_FILES) | {
            "artifact_manifest.json",
            "child_jobs",
            "children",
            "genesis-cache",
        }
        actual_top_level = {path.name for path in outdir.iterdir()}
        scratch_layout_valid = all(
            not (outdir / name).exists()
            or ((outdir / name).is_dir() and not (outdir / name).is_symlink())
            for name in ("child_jobs", "children", "genesis-cache")
        )
        checks["artifact_files_closed"] = bool(
            isinstance(files, Mapping)
            and set(files) == set(RECEIPT_FILES)
            and actual_top_level <= allowed_top_level
            and scratch_layout_valid
            and all(
                isinstance(receipt, Mapping)
                and set(receipt) == {"bytes", "sha256"}
                and not Path(name).is_absolute()
                and ".." not in Path(name).parts
                and _receipt_matches_file(
                    {**dict(receipt), "path": str(outdir / name)},
                    expected_parent=outdir,
                )
                for name, receipt in files.items()
            )
            and len(files) == len(RECEIPT_FILES)
        )

        frozen = json.loads((outdir / "frozen_config.json").read_text(encoding="utf-8"))
        frozen_body = dict(frozen)
        declared_frozen_hash = frozen_body.pop("frozen_config_sha256", None)
        checks["frozen_config_self_hash_valid"] = (
            declared_frozen_hash == canonical_sha256(frozen_body)
            and artifact.get("frozen_config_content_sha256") == declared_frozen_hash
        )
        current_static_inputs = static_input_receipt()
        snapshot_sha256 = sha256_file(outdir / "protocol_snapshot.md")
        checks["protocol_snapshot_binding"] = (
            snapshot_sha256 == frozen.get("protocol_document_sha256")
            and snapshot_sha256 == artifact.get("protocol_snapshot_sha256")
            and snapshot_sha256 == current_static_inputs["protocol_document"]["sha256"]
        )

        verification = json.loads((outdir / "verification.json").read_text(encoding="utf-8"))
        source = artifact.get("source_table4")
        checks["source_table4_binding_consistent"] = (
            isinstance(source, Mapping)
            and source == verification.get("table4_inputs")
            and artifact.get("source_table4_content_sha256") == canonical_sha256(source)
        )
        source_dir = Path(str(source.get("directory", ""))).resolve(strict=True)
        if expected_table4_dir is not None:
            expected_source_dir = expected_table4_dir.resolve(strict=True)
        else:
            expected_source_dir = source_dir
        source_names = {
            "frozen_manifest": "frozen_manifest.json",
            "state_records": "state_records.jsonl",
            "asset_records": "asset_records.jsonl",
            "artifact_manifest": "artifact_manifest.json",
        }
        source_paths_match = source_dir == expected_source_dir and all(
            Path(str(source[key].get("path", ""))).resolve(strict=True)
            == source_dir / filename
            for key, filename in source_names.items()
            if isinstance(source.get(key), Mapping)
        ) and all(isinstance(source.get(key), Mapping) for key in source_names)
        source_files_match = source_paths_match and all(
            _receipt_matches_file(source[key], expected_parent=source_dir) for key in source_names
        )
        source_manifest = json.loads(
            (source_dir / "frozen_manifest.json").read_text(encoding="utf-8")
        )
        source_files_match = bool(
            source_files_match
            and source_manifest.get("manifest_content_sha256")
            == manifest_self_hash(source_manifest)
            == source["frozen_manifest"].get("content_sha256")
        )
        upstream_artifact = json.loads((source_dir / "artifact_manifest.json").read_text(encoding="utf-8"))
        source_items = source_manifest.get("items")
        try:
            if not isinstance(upstream_artifact, Mapping) or not isinstance(source_items, list):
                raise ValueError("Table 4 source receipt is malformed")
            source_j_eval = sum(int(item["movable_dof_count"]) for item in source_items)
            _validate_table4_artifact_receipt(
                source_dir,
                upstream_artifact,
                manifest_content_sha256=str(source_manifest["manifest_content_sha256"]),
                expected_n=len(source_items),
                expected_j=source_j_eval,
            )
            if mode == "formal":
                canonical_assets = load_canonical_cohort()
                _, formal_ordered_ids = _validate_table4_manifest_cohort(
                    source_manifest,
                    canonical_assets,
                    expected_n=N_EVAL,
                    expected_j=J_EVAL,
                    expected_category_count=EXPECTED_CATEGORY_COUNT,
                )
                if canonical_sha256(formal_ordered_ids) != EXPECTED_ORDERED_IDS_SHA256:
                    raise ValueError("Table 4 formal ordered identity hash mismatch")
            upstream_artifact_valid = True
        except (KeyError, OSError, TypeError, ValueError, json.JSONDecodeError) as exc:
            issues.append(f"Table 4 artifact validation failed: {type(exc).__name__}: {exc}")
            upstream_artifact_valid = False
        source_files_match = bool(source_files_match and upstream_artifact_valid)
        checks["source_table4_files_match"] = source_files_match

        static_inputs = artifact.get("static_inputs")
        static_manifests = {
            "canonical_cohort": (
                COHORT_FILE_SHA256,
                COHORT_CONTENT_SHA256,
            ),
            "table2_cohort": (
                TABLE2_COHORT_FILE_SHA256,
                TABLE2_COHORT_CONTENT_SHA256,
            ),
            "table3_manifest": (
                TABLE3_MANIFEST_FILE_SHA256,
                TABLE3_MANIFEST_CONTENT_SHA256,
            ),
        }
        static_pins_match = all(
            _self_hashed_manifest_receipt_matches(
                current_static_inputs[name],
                expected_file_sha256=file_sha256,
                expected_content_sha256=content_sha256,
            )
            for name, (file_sha256, content_sha256) in static_manifests.items()
        ) and current_static_inputs["table3_records"]["sha256"] == TABLE3_RECORDS_SHA256
        checks["static_inputs_match"] = (
            isinstance(static_inputs, Mapping)
            and static_inputs == current_static_inputs
            and artifact.get("static_inputs_content_sha256")
            == canonical_sha256(static_inputs)
            and static_pins_match
        )

        cohort = frozen.get("cohort", {})
        execution = frozen.get("execution", {})
        runner_identity = frozen.get("runner_identity", {})
        shared_binding = frozen.get("operationalization", {}).get(
            "shared_table4a_runner_binding"
        )
        source_artifact_binding = frozen.get("operationalization", {}).get(
            "source_table4_artifact_manifest_binding"
        )
        resume_policy = frozen.get("operationalization", {}).get("resume_policy")
        torch_threading_policy = frozen.get("operationalization", {}).get(
            "torch_threading"
        )
        early_affinity_policy = frozen.get("operationalization", {}).get(
            "early_cpu_affinity"
        )
        early_affinity_binding = frozen.get("operationalization", {}).get(
            "early_cpu_affinity_launcher_binding"
        )
        execution_common = (
            frozen.get("protocol_id") == PROTOCOL_ID
            and frozen.get("dataset") == DATASET_LABEL
            and frozen.get("mode") == mode
            and frozen.get("classification") == classification
            and execution.get("child_timeout_seconds") == CHILD_TIMEOUT_SECONDS
            and execution.get("workers") == artifact.get("execution", {}).get("workers")
            and execution.get("early_cpu_affinity_launcher")
            == current_static_inputs["cpu_affinity_launcher"]
            and execution.get("child_cpu_affinity_width")
            == shared.lam4a.CPU_AFFINITY_WIDTH
            and cohort.get("source_manifest") == source["frozen_manifest"]["path"]
            and cohort.get("source_manifest_file_sha256") == source["frozen_manifest"]["sha256"]
            and cohort.get("source_manifest_content_sha256")
            == source["frozen_manifest"]["content_sha256"]
            and execution.get("table4_state_records")
            == {
                "path": source["state_records"]["path"],
                "sha256": source["state_records"]["sha256"],
            }
            and execution.get("table4_asset_records")
            == {
                "path": source["asset_records"]["path"],
                "sha256": source["asset_records"]["sha256"],
            }
            and runner_identity.get("runner_path")
            == current_static_inputs["adapter_runner"]["path"]
            and runner_identity.get("runner_sha256")
            == current_static_inputs["adapter_runner"]["sha256"]
            and runner_identity.get("lam_supplementary_runner_path")
            == current_static_inputs["genesis_runner"]["path"]
            and runner_identity.get("lam_supplementary_runner_sha256")
            == current_static_inputs["genesis_runner"]["sha256"]
            and runner_identity.get("static_atoms_path")
            == current_static_inputs["static_atoms"]["path"]
            and runner_identity.get("static_atoms_sha256")
            == current_static_inputs["static_atoms"]["sha256"]
            and shared_binding == current_static_inputs["shared_table4a_runner"]
            and source_artifact_binding == source["artifact_manifest"]
            and resume_policy == RESUME_POLICY
            and torch_threading_policy == TORCH_THREADING_POLICY
            and early_affinity_policy == EARLY_CPU_AFFINITY_POLICY
            and early_affinity_binding
            == current_static_inputs["cpu_affinity_launcher"]
        )
        formal_controls = mode != "formal" or (
            execution.get("workers") == FORMAL_WORKERS
            and execution.get("child_timeout_seconds") == CHILD_TIMEOUT_SECONDS
        )
        checks["execution_controls_match"] = bool(execution_common and formal_controls)

        run_manifest = json.loads((outdir / "manifest.json").read_text(encoding="utf-8"))
        summary = json.loads((outdir / "summary.json").read_text(encoding="utf-8"))
        manifest_outputs = run_manifest.get("outputs", {})
        checks["output_hashes_match"] = manifest_outputs == {
            "asset_records_sha256": sha256_file(outdir / "asset_records.jsonl"),
            "joint_records_sha256": sha256_file(outdir / "joint_records.jsonl"),
            "summary_sha256": sha256_file(outdir / "summary.json"),
            "summary_md_sha256": sha256_file(outdir / "summary.md"),
        }
        asset_rows = _read_jsonl(outdir / "asset_records.jsonl")
        joint_rows = _read_jsonl(outdir / "joint_records.jsonl")
        checks["package_input_bindings_match"] = all(
            package_binding_attestation_valid(record)
            and 0 <= int(record.get("selection_index", -1))
            < len(source_manifest.get("items", []))
            and package_binding_matches_source_item(
                record,
                source_manifest["items"][int(record["selection_index"])],
            )
            and current_package_binding_matches(record)
            for record in asset_rows
        )
        selection_indices = [int(row.get("selection_index", -1)) for row in asset_rows]
        checks["record_counts_match"] = (
            run_manifest.get("protocol_id") == PROTOCOL_ID
            and run_manifest.get("dataset") == DATASET_LABEL
            and run_manifest.get("mode") == mode
            and run_manifest.get("record_count") == len(asset_rows)
            and summary.get("protocol_id") == PROTOCOL_ID
            and summary.get("dataset") == DATASET_LABEL
            and summary.get("mode") == mode
            and summary.get("classification") == classification
            and summary.get("status_counts", {}).get("total") == len(asset_rows)
            and selection_indices == list(range(len(asset_rows)))
            and len({str(row.get("dataset_id")) for row in asset_rows}) == len(asset_rows)
        )

        table3_for_output: Mapping[str, Mapping[str, bool]] | None = None
        table3_output_binding_available = True
        if mode == "formal":
            try:
                table3_for_output, table3_joint_total = load_table3_joint_pass()
                if table3_joint_total != J_EVAL:
                    raise ValueError("Table 3 joint denominator mismatch")
            except (Exception, SystemExit) as exc:  # noqa: BLE001
                issues.append(f"Table 3 output binding failed: {type(exc).__name__}: {exc}")
                table3_output_binding_available = False
        checks["output_source_bindings_match"] = bool(
            table3_output_binding_available
            and output_records_bind_source(
                asset_rows,
                source_items,
                table3_pass=table3_for_output,
            )
        )

        table4_strict = _load_table4_strict_for_verification(
            source_dir / "asset_records.jsonl",
            source_items,
        )
        recomputed_metrics = _recompute_summary_metrics(asset_rows, table4_strict)
        checks["execution_invariants_valid"] = execution_record_invariants_valid(
            asset_rows,
            formal=mode == "formal",
        )
        checks["derived_outputs_match"] = (
            joint_rows == _flatten_joint_records(asset_rows)
            and summary.get("status_counts") == recomputed_metrics.get("status_counts")
            and summary.get("metrics") == recomputed_metrics
            and run_manifest.get("status_counts") == recomputed_metrics.get("status_counts")
            and (outdir / "summary.md").read_text(encoding="utf-8")
            == _render_summary_markdown(summary)
        )

        counts = _derive_receipt_counts_from_rows(asset_rows, joint_rows)
        timing = json.loads((outdir / "timing.json").read_text(encoding="utf-8"))
        summary_counts = summary.get("metrics", {}).get("state_counts", {})
        cross = summary_counts.get("hash_cross_check", {})
        joint_state_intended = sum(int(row.get("states_intended", 0)) for row in joint_rows)
        joint_state_executed = sum(int(row.get("states_executed", 0)) for row in joint_rows)
        denominators_common = (
            artifact.get("denominators") == counts
            and verification.get("denominators") == counts
            and timing.get("n_eval") == counts["n_eval"]
            and timing.get("j_eval") == counts["j_eval"]
            and timing.get("states_intended") == counts["states_intended"]
            and timing.get("states_genesis_replay_intended")
            == counts["states_genesis_replay_intended"]
            and timing.get("states_fail_closed_without_replay")
            == counts["states_fail_closed_without_replay"]
            and timing.get("states_executed") == counts["states_executed"]
            and summary.get("cohort", {}).get("n_eval") == counts["n_eval"]
            and summary.get("cohort", {}).get("j_eval") == counts["j_eval"]
            and summary_counts.get("intended") == counts["states_intended"]
            and summary_counts.get("executed") == counts["states_executed"]
            and joint_state_intended == counts["states_intended"]
            and joint_state_executed == counts["states_executed"]
            and sum(int(cross.get(key, 0)) for key in ("verified", "mismatch", "no_reference"))
            == counts["states_executed"]
            and int(cross.get("mismatch", -1)) == 0
        )
        formal_denominators = mode != "formal" or (
            counts
            == {
                "n_eval": N_EVAL,
                "j_eval": J_EVAL,
                "range_evaluable_joints": EXPECTED_RANGE_EVALUABLE_JOINTS,
                "range_invalid_joints": EXPECTED_RANGE_INVALID_JOINTS,
                "states_intended": EXPECTED_SINGLE_STATES,
                "states_genesis_replay_intended": EXPECTED_GENESIS_REPLAY_STATES,
                "states_fail_closed_without_replay": EXPECTED_FAIL_CLOSED_WITHOUT_REPLAY_STATES,
                "states_executed": counts["states_executed"],
            }
            and summary.get("metrics", {})
            .get("joint_level_full_range_cf", {})
            .get("denominator")
            == J_EVAL
            and summary.get("metrics", {})
            .get("collision_safe_dof_retention", {})
            .get("denominator")
            == J_EVAL
        )
        checks["denominators_match"] = bool(denominators_common and formal_denominators)

        full_wall_raw = timing.get("wall_time_seconds")
        if not isinstance(full_wall_raw, (int, float)) or isinstance(full_wall_raw, bool):
            raise ValueError("wall_time_seconds must be a JSON number")
        full_wall = float(full_wall_raw)
        child_wall = float(summary.get("wall_seconds", -1))
        wall_clock = {
            "started_at_utc": timing.get("started_at_utc"),
            "completed_at_utc": timing.get("completed_at_utc"),
            "wall_time_seconds": timing.get("wall_time_seconds"),
            "measurement_endpoint": timing.get("measurement_endpoint"),
        }
        started_timestamp = _timestamp(timing.get("started_at_utc"))
        completed_timestamp = _timestamp(timing.get("completed_at_utc"))
        utc_elapsed = (completed_timestamp - started_timestamp).total_seconds()
        duration_tolerance = max(5.0, 0.001 * full_wall)
        checks["wall_clock_valid"] = (
            timing.get("schema_version") == "experiment-timing/v1"
            and timing.get("table") == "Table 4a"
            and timing.get("protocol_id") == PROTOCOL_ID
            and timing.get("mode") == mode
            and timing.get("workers") == execution.get("workers")
            and timing.get("asset_timeout_seconds") == CHILD_TIMEOUT_SECONDS
            and timing.get("measurement_endpoint") == WALL_CLOCK_MEASUREMENT_ENDPOINT
            and math.isfinite(full_wall)
            and full_wall >= child_wall >= 0.0
            and utc_elapsed >= 0.0
            and abs(utc_elapsed - full_wall) <= duration_tolerance
            and artifact.get("wall_clock") == wall_clock
            and verification.get("wall_clock") == wall_clock
        )
        formal_verification_valid = mode != "formal" or formal_verification_receipt_valid(
            verification.get("shared_formal_verification"),
            run_manifest.get("verification"),
        )
        expected_status = (
            "PASS"
            if mode == "formal"
            and verification.get("return_code") == 0
            and formal_verification_valid
            else "FAIL" if mode == "formal" else "SMOKE"
        )
        checks["verification_status_valid"] = (
            verification.get("schema_version") == VERIFICATION_SCHEMA_VERSION
            and verification.get("protocol_id") == PROTOCOL_ID
            and verification.get("dataset") == DATASET_LABEL
            and verification.get("mode") == mode
            and verification.get("status") == expected_status
            and (mode != "formal" or expected_status == "PASS")
            and formal_verification_valid
        )
    except Exception as exc:  # noqa: BLE001
        issues.append(f"{type(exc).__name__}: {exc}")
    return {
        "schema_version": "table4a-output-verifier/v1",
        "protocol_id": PROTOCOL_ID,
        "status": "PASS" if all(checks.values()) else "FAIL",
        "check_count": len(checks),
        "checks": checks,
        "issues": issues,
        "verified_at_utc": utc_now(),
    }


def write_post_run_receipts(
    output: Path,
    *,
    mode: str,
    return_code: int,
    started_at_utc: str,
    completed_at_utc: str,
    wall_time_seconds: float,
    table4_inputs: Mapping[str, Any],
    workers: int,
) -> dict[str, Any]:
    output = output.resolve(strict=True)
    counts = _derive_receipt_counts(output)
    wall_clock = {
        "started_at_utc": started_at_utc,
        "completed_at_utc": completed_at_utc,
        "wall_time_seconds": wall_time_seconds,
        "measurement_endpoint": WALL_CLOCK_MEASUREMENT_ENDPOINT,
    }
    timing = {
        "schema_version": "experiment-timing/v1",
        "table": "Table 4a",
        "protocol_id": PROTOCOL_ID,
        "mode": mode,
        **wall_clock,
        "workers": workers,
        "asset_timeout_seconds": CHILD_TIMEOUT_SECONDS,
        **counts,
        "command": [sys.executable, *sys.argv],
    }
    atomic_json(output / "timing.json", timing)
    run_manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    shared_verification: Mapping[str, Any] | None = (
        run_manifest.get("verification") if mode == "formal" else None
    )
    status = (
        "PASS"
        if mode == "formal"
        and return_code == 0
        and shared_verification
        and shared_verification.get("all_pass") is True
        else "FAIL" if mode == "formal" else "SMOKE"
    )
    verification = {
        "schema_version": VERIFICATION_SCHEMA_VERSION,
        "protocol_id": PROTOCOL_ID,
        "dataset": DATASET_LABEL,
        "mode": mode,
        "status": status,
        "return_code": return_code,
        "table4_inputs": dict(table4_inputs),
        "denominators": counts,
        "wall_clock": wall_clock,
        "shared_formal_verification": shared_verification,
    }
    atomic_json(output / "verification.json", verification)
    files = {
        name: {
            "bytes": (output / name).stat().st_size,
            "sha256": sha256_file(output / name),
        }
        for name in RECEIPT_FILES
    }
    static_inputs = static_input_receipt()
    frozen_config = json.loads((output / "frozen_config.json").read_text(encoding="utf-8"))
    artifact = {
        "schema_version": ARTIFACT_SCHEMA_VERSION,
        "protocol_id": PROTOCOL_ID,
        "dataset": DATASET_LABEL,
        "mode": mode,
        "classification": "FORMAL" if mode == "formal" else "SMOKE",
        "created_at_utc": completed_at_utc,
        "excludes": ARTIFACT_EXCLUSIONS,
        "protocol_snapshot_sha256": sha256_file(output / "protocol_snapshot.md"),
        "frozen_config_content_sha256": frozen_config.get("frozen_config_sha256"),
        "source_table4": dict(table4_inputs),
        "source_table4_content_sha256": canonical_sha256(table4_inputs),
        "static_inputs": static_inputs,
        "static_inputs_content_sha256": canonical_sha256(static_inputs),
        "execution": {
            "workers": workers,
            "asset_timeout_seconds": CHILD_TIMEOUT_SECONDS,
        },
        "denominators": counts,
        "wall_clock": wall_clock,
        "files": files,
    }
    artifact["artifact_manifest_content_sha256"] = artifact_manifest_self_hash(artifact)
    atomic_json(output / "artifact_manifest.json", artifact)
    result = verify_output_receipt(
        output,
        expected_table4_dir=Path(str(table4_inputs["directory"])),
    )
    if result["status"] != "PASS":
        raise RuntimeError(f"Table 4a publication gate failed: {result}")
    return result


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("smoke", "formal"), default=None)
    parser.add_argument("--table4-dir", type=Path)
    parser.add_argument("--n", type=int, default=3)
    parser.add_argument("--workers", type=int, default=FORMAL_WORKERS)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--verify-output", type=Path)
    parser.add_argument("--child", action="store_true")
    parser.add_argument("--job", type=Path)
    parser.add_argument("--result", type=Path)
    return parser.parse_args(argv)


def validate_cli_contract(args: argparse.Namespace) -> None:
    if args.verify_output is not None:
        if args.child or args.mode is not None or args.output_dir is not None:
            raise ValueError("--verify-output cannot be combined with run or child mode")
        if not args.verify_output.is_dir():
            raise ValueError("--verify-output must be an existing directory")
        if args.table4_dir is not None and not args.table4_dir.is_dir():
            raise ValueError("--table4-dir must be an existing directory")
        return
    if args.child:
        if args.job is None or args.result is None:
            raise ValueError("--child requires --job and --result")
        return
    if args.mode is None or args.table4_dir is None:
        raise ValueError("--mode and --table4-dir are required")
    if args.n < 1 or args.workers < 1:
        raise ValueError("n and workers must be positive")
    if not args.table4_dir.is_dir():
        raise ValueError("--table4-dir must be an existing directory")
    if args.mode == "formal" and args.workers != FORMAL_WORKERS:
        raise ValueError(f"formal Table 4a freezes workers={FORMAL_WORKERS}")


def main(argv: list[str] | None = None) -> int:
    global _BOUND_TABLE4
    args = parse_args(argv)
    validate_cli_contract(args)
    if args.verify_output is not None:
        result = verify_output_receipt(
            args.verify_output,
            expected_table4_dir=args.table4_dir,
        )
        print(json.dumps(result, indent=2, sort_keys=True, ensure_ascii=True))
        return 0 if result["status"] == "PASS" else 2
    if args.child:
        configure_shared_runner(None)
        return run_child(args.job, args.result)

    started_at = utc_now()
    started = time.perf_counter()
    cohort = load_canonical_cohort()
    binding = validate_table4_receipt(
        args.table4_dir,
        cohort,
        expected_n=N_EVAL,
        expected_j=J_EVAL,
        expected_category_count=EXPECTED_CATEGORY_COUNT,
    )
    if binding.ordered_ids_sha256 != EXPECTED_ORDERED_IDS_SHA256:
        raise ValueError("Table 4 ordered identity hash mismatch")
    _BOUND_TABLE4 = binding
    configure_shared_runner(binding)
    if args.output_dir is None:
        timestamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        suffix = "n2655" if args.mode == "formal" else f"smoke_n{args.n}"
        args.output_dir = REPO / "exp/runtime" / f"table4a_urdf_ours_pva_per_class_n5_{suffix}_{timestamp}"
    return_code = shared.run_scope(args)
    completed_at = utc_now()
    wall_time_seconds = round(time.perf_counter() - started, 6)
    table4_inputs = table4_input_receipt(binding)
    write_post_run_receipts(
        args.output_dir.resolve(),
        mode=args.mode,
        return_code=return_code,
        started_at_utc=started_at,
        completed_at_utc=completed_at,
        wall_time_seconds=wall_time_seconds,
        table4_inputs=table4_inputs,
        workers=args.workers,
    )
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
