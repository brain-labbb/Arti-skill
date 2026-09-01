#!/usr/bin/env python3
"""Validate and optionally run the frozen PartNet-Mobility sample cohort.

The sample is defined by the ordered ``items[].dataset_id`` entries in the
Table 4 frozen manifest.  This command is the structured-data equivalent of
the ``jq`` pipeline documented in ``URDF-Sim-Ready-Automatic-Evaluation.md``.
By default it only checks the cohort and package bindings; ``--run`` delegates
to the existing Table 4b evaluator after those checks pass.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import subprocess
import sys
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET

SCRIPT = Path(__file__).resolve()
EXP_ROOT = SCRIPT.parents[1]
REPO = SCRIPT.parents[2]
DEFAULT_MANIFEST = (
    EXP_ROOT / "runtime/urdf_table4_partnet_mobility_n800_20260813/frozen_manifest.json"
)
DEFAULT_DATASET_ROOT = EXP_ROOT / "PartNet-Mobility/data/dataset"
DEFAULT_RUNNER = EXP_ROOT / "scripts/run_urdf_table4b_partnet_mobility.py"
_GENESIS_PYTHON = Path("/mnt/zsn/miniconda3/envs/genesis-main/bin/python")
DEFAULT_RUNNER_PYTHON = (
    _GENESIS_PYTHON if _GENESIS_PYTHON.is_file() else Path(sys.executable)
)
DEFAULT_SAMPLE_SIZE = 800
FORMAL_WORKERS = 16
EXPECTED_PROTOCOL_ID = "urdf_sim_ready_table4_partnet_mobility_n800_v1"
EXPECTED_RUNNER_PROTOCOL_ID = (
    "table4b_partnet_mobility_table4cohort_n800_salt20260813_v1"
)
EXPECTED_RUNNER_SHA256 = (
    "6ffc2207f6262f99c73277eac8ac97e29cc9ebd5af6c3ff49457ac4feb58a494"
)
EXPECTED_MANIFEST_SHA256 = (
    "2ff015ee6bb377ce693126b52dd632a7565a3eaa9f0007e26122a1bb4ab99900"
)
EXPECTED_ORDERED_IDS_SHA256 = (
    "ef6cb964e50dc712280256c5b2f675cc2c957095c3553b21845d3562a5011883"
)
REQUIRED_FILES = (
    "meta.json",
    "mobility.urdf",
    "mobility_v2.json",
    "semantics.txt",
    "result.json",
    "bounding_box.json",
)
RUNNER_OUTPUT_FILES = ("summary.json", "asset_records.jsonl", "manifest.json")
VALID_RECORD_STATUSES = {"completed", "error", "timeout", "blocked", "skipped"}
_NUMERIC_ID = re.compile(r"^[0-9]+$")
_HASH = re.compile(r"^[0-9a-f]{64}$")
INPUT_IDENTITY_FIELDS = (
    "protocol_id",
    "order",
    "dataset_id",
    "selection_digest",
    "category",
    "movable_dof_count",
    "range_evaluable_dof_count",
    "joint_specs_sha256",
    "runtime_identity_sha256",
    "urdf_sha256",
    "bounding_box_sha256",
    "collision_mesh_inventory_sha256",
    "object_bbox_diagonal_m",
    "rest_state_expected",
    "single_state_expected",
    "sobol_state_expected",
)


class SampleValidationError(ValueError):
    """Raised when the frozen sample or its live package binding is invalid."""


def sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
                digest.update(block)
    except OSError as error:
        raise SampleValidationError(f"cannot read {path}: {error}") from error
    return digest.hexdigest()


def canonical_sha256(value: Any) -> str:
    return sha256_bytes(
        json.dumps(
            value,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=True,
        ).encode("utf-8")
    )


def validate_runner_binding(runner: Path = DEFAULT_RUNNER) -> Path:
    """Return the frozen Table 4b runner, rejecting path or content drift."""

    requested = Path(runner)
    try:
        requested = requested.resolve(strict=True)
        canonical = DEFAULT_RUNNER.resolve(strict=True)
    except (OSError, RuntimeError) as error:
        raise SampleValidationError(f"canonical runner is unavailable: {error}") from error
    if requested != canonical:
        raise SampleValidationError(
            f"--run requires the canonical runner {canonical} (got {requested})"
        )
    if not requested.is_file() or requested.is_symlink():
        raise SampleValidationError(f"canonical runner is not a regular file: {requested}")
    observed = sha256_file(requested)
    if observed != EXPECTED_RUNNER_SHA256:
        raise SampleValidationError(
            f"canonical runner SHA-256 mismatch: {observed} != {EXPECTED_RUNNER_SHA256}"
        )
    return requested


def validate_runner_python(runner_python: Path) -> Path:
    """Resolve an executable interpreter used for the frozen runner."""

    candidate = Path(runner_python)
    if not candidate.is_absolute():
        candidate = Path.cwd() / candidate
    try:
        candidate = candidate.resolve(strict=True)
    except (OSError, RuntimeError) as error:
        raise SampleValidationError(
            f"runner Python is unavailable: {runner_python}: {error}"
        ) from error
    if candidate.is_symlink() or not candidate.is_file():
        raise SampleValidationError(f"runner Python is not a regular file: {candidate}")
    if not os.access(candidate, os.X_OK):
        raise SampleValidationError(f"runner Python is not executable: {candidate}")
    return candidate


def _parse_json_bytes(payload: bytes, path: Path) -> dict[str, Any]:
    try:
        value = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise SampleValidationError(f"cannot read manifest {path}: {error}") from error
    if not isinstance(value, dict):
        raise SampleValidationError("manifest must contain a JSON object")
    return value


def _resolve_existing(
    path: Path, label: str, *, reject_symlink: bool = False
) -> Path:
    try:
        if reject_symlink and path.is_symlink():
            raise SampleValidationError(f"{label} is a symlink: {path}")
        return path.resolve(strict=True)
    except SampleValidationError:
        raise
    except (OSError, RuntimeError) as error:
        raise SampleValidationError(f"{label} does not exist: {path}") from error


def _require_contained(path: Path, root: Path, label: str) -> Path:
    try:
        path.relative_to(root)
    except ValueError as error:
        raise SampleValidationError(f"{label} escapes frozen root: {path}") from error
    return path


def _ordered_ids(items: Sequence[Mapping[str, Any]]) -> list[str]:
    ids: list[str] = []
    for index, item in enumerate(items):
        if not isinstance(item, Mapping):
            raise SampleValidationError(f"manifest item {index} is not an object")
        dataset_id = item.get("dataset_id")
        if not isinstance(dataset_id, str) or not dataset_id:
            raise SampleValidationError(f"manifest item {index} has no dataset_id")
        if not _NUMERIC_ID.fullmatch(dataset_id):
            raise SampleValidationError(
                f"dataset_id must be numeric at index {index}: {dataset_id!r}"
            )
        if Path(dataset_id).name != dataset_id:
            raise SampleValidationError(
                f"dataset_id is not a package basename at index {index}: {dataset_id!r}"
            )
        order = item.get("order")
        if isinstance(order, bool) or not isinstance(order, int):
            raise SampleValidationError(f"invalid order at index {index}: {order!r}")
        if order != index:
            raise SampleValidationError(
                f"manifest item order mismatch at index {index}: {order}"
            )
        ids.append(dataset_id)
    if len(ids) != len(set(ids)):
        raise SampleValidationError("manifest dataset_id values are not unique")
    return ids


def _is_hash(value: Any) -> bool:
    return isinstance(value, str) and _HASH.fullmatch(value) is not None


def _collision_mesh_inventory(package: Path, urdf: Path) -> list[dict[str, Any]]:
    """Recompute the frozen collision-mesh inventory for one package."""

    try:
        root = ET.parse(urdf).getroot()
    except (OSError, ET.ParseError) as error:
        raise SampleValidationError(f"cannot parse URDF {urdf}: {error}") from error
    references = sorted(
        {
            mesh.get("filename", "").replace("\\", "/")
            for mesh in root.findall("link/collision/geometry/mesh")
            if mesh.get("filename")
        }
    )
    inventory: list[dict[str, Any]] = []
    for reference in references:
        candidate = package / reference
        if candidate.is_symlink():
            raise SampleValidationError(f"collision mesh is a symlink: {candidate}")
        resolved_candidate = (
            _resolve_existing(
                candidate, f"collision mesh {reference}", reject_symlink=True
            )
            if candidate.exists()
            else None
        )
        if resolved_candidate is not None:
            _require_contained(
                resolved_candidate, package, f"collision mesh {reference}"
            )
        else:
            try:
                unresolved = candidate.resolve(strict=False)
            except (OSError, RuntimeError) as error:
                raise SampleValidationError(
                    f"cannot resolve collision mesh {reference}: {error}"
                ) from error
            _require_contained(unresolved, package, f"collision mesh {reference}")
        exists = resolved_candidate is not None and resolved_candidate.is_file()
        inventory.append(
            {
                "path": reference,
                "exists": exists,
                "size_bytes": resolved_candidate.stat().st_size if exists else None,
                "sha256": sha256_file(resolved_candidate) if exists else None,
            }
        )
    return inventory


def _validate_live_bindings(
    item: Mapping[str, Any],
    package: Path,
    urdf: Path,
    *,
    require_frozen_bindings: bool,
) -> None:
    """Check all package hashes declared by a frozen manifest item."""

    bounding_box_hash = item.get("bounding_box_sha256")
    inventory = item.get("collision_mesh_files")
    inventory_hash = item.get("collision_mesh_inventory_sha256")
    input_identity = item.get("input_identity_sha256")
    if require_frozen_bindings:
        required = {
            "bounding_box_sha256": bounding_box_hash,
            "collision_mesh_files": inventory,
            "collision_mesh_inventory_sha256": inventory_hash,
            "input_identity_sha256": input_identity,
        }
        missing = [name for name, value in required.items() if value is None]
        if missing:
            raise SampleValidationError(
                f"frozen item is missing package bindings: {', '.join(missing)}"
            )

    if bounding_box_hash is not None:
        if not _is_hash(bounding_box_hash):
            raise SampleValidationError("invalid bounding_box_sha256")
        bounding_box = _resolve_existing(
            package / "bounding_box.json",
            f"{package.name}/bounding_box.json",
            reject_symlink=True,
        )
        _require_contained(bounding_box, package, f"{package.name}/bounding_box.json")
        observed_bbox_hash = sha256_file(bounding_box)
        if observed_bbox_hash != bounding_box_hash:
            raise SampleValidationError(
                f"bounding-box SHA-256 mismatch for {package.name}: "
                f"{observed_bbox_hash} != {bounding_box_hash}"
            )

    if inventory is not None or inventory_hash is not None:
        if not isinstance(inventory, list) or not _is_hash(inventory_hash):
            raise SampleValidationError("invalid collision mesh inventory binding")
        observed_inventory = _collision_mesh_inventory(package, urdf)
        if observed_inventory != inventory:
            raise SampleValidationError(
                f"collision mesh inventory mismatch for {package.name}"
            )
        observed_inventory_hash = canonical_sha256(observed_inventory)
        if observed_inventory_hash != inventory_hash:
            raise SampleValidationError(
                f"collision mesh inventory SHA-256 mismatch for {package.name}: "
                f"{observed_inventory_hash} != {inventory_hash}"
            )

    if input_identity is not None:
        if not _is_hash(input_identity):
            raise SampleValidationError("invalid input_identity_sha256")
        try:
            identity = {key: item[key] for key in INPUT_IDENTITY_FIELDS}
        except KeyError as error:
            raise SampleValidationError(
                f"input identity is missing field: {error.args[0]}"
            ) from error
        observed_identity = canonical_sha256(identity)
        if observed_identity != input_identity:
            raise SampleValidationError(
                f"input identity SHA-256 mismatch for {package.name}: "
                f"{observed_identity} != {input_identity}"
            )


def load_manifest(
    manifest_path: Path = DEFAULT_MANIFEST,
    *,
    expected_count: int | None = DEFAULT_SAMPLE_SIZE,
    expected_dataset_root: Path | None = DEFAULT_DATASET_ROOT,
) -> tuple[dict[str, Any], Path, str, list[str]]:
    """Load and validate the frozen manifest metadata.

    ``expected_count`` and ``expected_dataset_root`` can be set to ``None``
    for isolated fixture tests.  The default values enforce the formal N=800
    PartNet-Mobility cohort.
    """

    manifest_path = _resolve_existing(
        Path(manifest_path), "manifest", reject_symlink=True
    )
    if not manifest_path.is_file():
        raise SampleValidationError(f"manifest is not a regular file: {manifest_path}")
    try:
        manifest_bytes = manifest_path.read_bytes()
    except OSError as error:
        raise SampleValidationError(f"cannot read manifest {manifest_path}: {error}") from error
    manifest_sha256 = sha256_bytes(manifest_bytes)
    is_default_path = manifest_path == DEFAULT_MANIFEST.resolve()
    expected_root_is_default = False
    if expected_dataset_root is not None:
        try:
            expected_root_is_default = (
                Path(expected_dataset_root).resolve(strict=True)
                == DEFAULT_DATASET_ROOT.resolve(strict=True)
            )
        except (OSError, RuntimeError):
            expected_root_is_default = False
    enforce_frozen_identity = is_default_path or (
        expected_count == DEFAULT_SAMPLE_SIZE and expected_root_is_default
    )
    if enforce_frozen_identity and manifest_sha256 != EXPECTED_MANIFEST_SHA256:
        raise SampleValidationError(
            f"frozen manifest SHA-256 mismatch: {manifest_sha256}"
        )
    manifest = _parse_json_bytes(manifest_bytes, manifest_path)
    if enforce_frozen_identity and manifest.get("protocol_id") != EXPECTED_PROTOCOL_ID:
        raise SampleValidationError("frozen manifest protocol_id mismatch")
    if manifest.get("status") != "FROZEN":
        raise SampleValidationError("manifest status must be FROZEN")
    items = manifest.get("items")
    if not isinstance(items, list):
        raise SampleValidationError("manifest items must be a list")
    if not items:
        raise SampleValidationError("manifest items must not be empty")
    if expected_count is not None and len(items) != expected_count:
        raise SampleValidationError(
            f"manifest must contain exactly {expected_count} items, got {len(items)}"
        )
    declared_size = manifest.get("sample_size")
    if isinstance(declared_size, bool) or not isinstance(declared_size, int):
        raise SampleValidationError("manifest sample_size is invalid")
    if declared_size != len(items):
        raise SampleValidationError(
            f"manifest sample_size mismatch: {declared_size} != {len(items)}"
        )
    ids = _ordered_ids(items)
    if enforce_frozen_identity:
        for index, item in enumerate(items):
            if item.get("protocol_id") != EXPECTED_PROTOCOL_ID:
                raise SampleValidationError(
                    f"frozen item protocol_id mismatch at index {index}"
                )
            for field in (
                "bounding_box_sha256",
                "collision_mesh_files",
                "collision_mesh_inventory_sha256",
                "input_identity_sha256",
            ):
                if field not in item or item[field] is None:
                    raise SampleValidationError(
                        f"frozen item is missing {field} at index {index}"
                    )
            if not _is_hash(item["bounding_box_sha256"]):
                raise SampleValidationError(
                    f"invalid bounding_box_sha256 at index {index}"
                )
            if not isinstance(item["collision_mesh_files"], list):
                raise SampleValidationError(
                    f"invalid collision_mesh_files at index {index}"
                )
            if not _is_hash(item["collision_mesh_inventory_sha256"]):
                raise SampleValidationError(
                    f"invalid collision_mesh_inventory_sha256 at index {index}"
                )
            if not _is_hash(item["input_identity_sha256"]):
                raise SampleValidationError(
                    f"invalid input_identity_sha256 at index {index}"
                )
    observed_ids_sha256 = canonical_sha256(ids)
    declared_ids_sha256 = manifest.get("ordered_selected_ids_sha256")
    if declared_ids_sha256 != observed_ids_sha256:
        raise SampleValidationError(
            f"ordered dataset ID SHA-256 mismatch: {observed_ids_sha256}"
        )
    if enforce_frozen_identity and observed_ids_sha256 != EXPECTED_ORDERED_IDS_SHA256:
        raise SampleValidationError(
            f"formal ordered dataset ID SHA-256 mismatch: {observed_ids_sha256}"
        )
    declared_items_sha256 = manifest.get("items_sha256")
    if declared_items_sha256 is not None:
        observed_items_sha256 = canonical_sha256(items)
        if declared_items_sha256 != observed_items_sha256:
            raise SampleValidationError(
                f"manifest items SHA-256 mismatch: {observed_items_sha256}"
            )
    dataset_root_value = manifest.get("dataset_root")
    if not isinstance(dataset_root_value, str) or not dataset_root_value:
        raise SampleValidationError("manifest dataset_root is missing")
    dataset_root = _resolve_existing(
        Path(dataset_root_value), "dataset_root", reject_symlink=True
    )
    if not dataset_root.is_dir():
        raise SampleValidationError(
            f"dataset_root is not a regular directory: {dataset_root}"
        )
    if expected_dataset_root is not None:
        expected_root = _resolve_existing(
            Path(expected_dataset_root),
            "expected dataset_root",
            reject_symlink=True,
        )
        if dataset_root != expected_root:
            raise SampleValidationError(
                f"dataset_root mismatch: {dataset_root} != {expected_root}"
            )
    manifest["dataset_root"] = str(dataset_root)
    return manifest, dataset_root, manifest_sha256, ids


def resolve_sample_assets(
    manifest: Mapping[str, Any],
    dataset_root: Path | None = None,
    *,
    require_core_files: bool = True,
) -> list[dict[str, Any]]:
    """Resolve each manifest item to a contained package and URDF."""

    items = manifest.get("items")
    if not isinstance(items, list):
        raise SampleValidationError("manifest items must be a list")
    # Keep this function safe when called directly, rather than only through
    # ``load_manifest``.  The runner's contract is the manifest order itself.
    _ordered_ids(items)
    root_value = dataset_root or manifest.get("dataset_root")
    if not isinstance(root_value, (str, Path)):
        raise SampleValidationError("dataset_root is missing")
    root = _resolve_existing(Path(root_value), "dataset_root", reject_symlink=True)
    require_frozen_bindings = (
        manifest.get("protocol_id") == EXPECTED_PROTOCOL_ID
        and len(items) == DEFAULT_SAMPLE_SIZE
    )
    assets: list[dict[str, Any]] = []
    for index, raw_item in enumerate(items):
        if not isinstance(raw_item, Mapping):
            raise SampleValidationError(f"manifest item {index} is not an object")
        dataset_id = raw_item.get("dataset_id")
        if not isinstance(dataset_id, str) or not _NUMERIC_ID.fullmatch(dataset_id):
            raise SampleValidationError(
                f"invalid dataset_id at index {index}: {dataset_id!r}"
            )
        if Path(dataset_id).name != dataset_id:
            raise SampleValidationError(
                f"dataset_id is not a package basename at index {index}: {dataset_id!r}"
            )
        package = _resolve_existing(
            root / dataset_id, f"package {dataset_id}", reject_symlink=True
        )
        _require_contained(package, root, f"package {dataset_id}")
        if not package.is_dir():
            raise SampleValidationError(f"package is not a directory: {package}")
        if package.name != dataset_id:
            raise SampleValidationError(
                f"dataset_id/package mismatch at index {index}: {dataset_id!r}"
            )
        if require_core_files:
            for filename in REQUIRED_FILES:
                candidate = _resolve_existing(
                    package / filename,
                    f"{dataset_id}/{filename}",
                    reject_symlink=True,
                )
                _require_contained(candidate, package, f"{dataset_id}/{filename}")
                if not candidate.is_file():
                    raise SampleValidationError(
                        f"required file is not regular: {candidate}"
                    )
        urdf = _resolve_existing(
            package / "mobility.urdf",
            f"{dataset_id}/mobility.urdf",
            reject_symlink=True,
        )
        _require_contained(urdf, package, f"{dataset_id}/mobility.urdf")
        if not urdf.is_file():
            raise SampleValidationError(f"URDF is not a regular file: {urdf}")
        expected_hash = raw_item.get("urdf_sha256")
        if not isinstance(expected_hash, str) or not expected_hash:
            raise SampleValidationError(f"missing URDF SHA-256 at index {index}")
        observed_hash = sha256_file(urdf)
        if observed_hash != expected_hash:
            raise SampleValidationError(
                f"URDF SHA-256 mismatch for {dataset_id}: {observed_hash} != {expected_hash}"
            )
        _validate_live_bindings(
            raw_item,
            package,
            urdf,
            require_frozen_bindings=require_frozen_bindings,
        )
        assets.append(
            {
                "order": index,
                "dataset_id": dataset_id,
                "package": package,
                "urdf": urdf,
                "urdf_sha256": observed_hash,
            }
        )
    return assets


def load_sample_assets(
    manifest_path: Path = DEFAULT_MANIFEST,
    *,
    expected_count: int | None = DEFAULT_SAMPLE_SIZE,
    expected_dataset_root: Path | None = DEFAULT_DATASET_ROOT,
) -> list[dict[str, Any]]:
    manifest, dataset_root, _manifest_sha256, _ids = load_manifest(
        manifest_path,
        expected_count=expected_count,
        expected_dataset_root=expected_dataset_root,
    )
    return resolve_sample_assets(manifest, dataset_root)


def load_sample_paths(
    manifest_path: Path = DEFAULT_MANIFEST,
    *,
    expected_count: int | None = DEFAULT_SAMPLE_SIZE,
    expected_dataset_root: Path | None = DEFAULT_DATASET_ROOT,
) -> list[Path]:
    """Return ``dataset_root / dataset_id / mobility.urdf`` in manifest order."""

    return [
        asset["urdf"]
        for asset in load_sample_assets(
            manifest_path,
            expected_count=expected_count,
            expected_dataset_root=expected_dataset_root,
        )
    ]


def load_sample_package_paths(
    manifest_path: Path = DEFAULT_MANIFEST,
    *,
    expected_count: int | None = DEFAULT_SAMPLE_SIZE,
    expected_dataset_root: Path | None = DEFAULT_DATASET_ROOT,
) -> list[Path]:
    """Return the package roots emitted by the documented ``jq`` expression."""

    return [
        asset["package"]
        for asset in load_sample_assets(
            manifest_path,
            expected_count=expected_count,
            expected_dataset_root=expected_dataset_root,
        )
    ]


def load_sample_urdf_paths(
    manifest_path: Path = DEFAULT_MANIFEST,
    *,
    expected_count: int | None = DEFAULT_SAMPLE_SIZE,
    expected_dataset_root: Path | None = DEFAULT_DATASET_ROOT,
) -> list[Path]:
    """Explicit alias for :func:`load_sample_paths` for CLI/API callers."""

    return load_sample_paths(
        manifest_path,
        expected_count=expected_count,
        expected_dataset_root=expected_dataset_root,
    )


def _validate_sample(
    manifest_path: Path = DEFAULT_MANIFEST,
    *,
    expected_count: int | None = DEFAULT_SAMPLE_SIZE,
    expected_dataset_root: Path | None = DEFAULT_DATASET_ROOT,
) -> tuple[dict[str, Any], list[Path], list[dict[str, Any]]]:
    """Validate the sample once and return its receipt, paths, and bindings."""

    manifest, dataset_root, manifest_sha256, ids = load_manifest(
        manifest_path,
        expected_count=expected_count,
        expected_dataset_root=expected_dataset_root,
    )
    assets = resolve_sample_assets(manifest, dataset_root)
    paths = [asset["urdf"] for asset in assets]
    report = {
        "status": "PASS",
        "manifest": str(Path(manifest_path).resolve()),
        "manifest_sha256": manifest_sha256,
        "dataset_root": str(dataset_root),
        "selection_expression": (
            ".dataset_root as $root | .items[].dataset_id | \"\\($root)/\\(.)\""
        ),
        "sample_size": len(assets),
        "ordered_ids_sha256": canonical_sha256(ids),
        "paths_checked": len(paths),
        "package_paths_checked": len(paths),
        "required_files_checked": len(assets) * len(REQUIRED_FILES),
        "urdf_hashes_checked": len(paths),
        "first_package_path": str(paths[0].parent) if paths else None,
        "last_package_path": str(paths[-1].parent) if paths else None,
        "first_path": str(paths[0]) if paths else None,
        "last_path": str(paths[-1]) if paths else None,
    }
    return report, paths, assets


def check_sample(
    manifest_path: Path = DEFAULT_MANIFEST,
    *,
    expected_count: int | None = DEFAULT_SAMPLE_SIZE,
    expected_dataset_root: Path | None = DEFAULT_DATASET_ROOT,
) -> dict[str, Any]:
    """Validate the sample and return a compact machine-readable receipt."""

    report, _paths, _assets = _validate_sample(
        manifest_path,
        expected_count=expected_count,
        expected_dataset_root=expected_dataset_root,
    )
    return report


def build_runner_command(
    *,
    runner_python: Path,
    runner: Path,
    mode: str,
    sample_size: int,
    workers: int | None = None,
    output_dir: Path | None = None,
) -> list[str]:
    """Build the explicit Table 4b invocation for a validated sample."""

    if mode not in {"smoke", "formal"}:
        raise ValueError(f"unsupported runner mode: {mode!r}")
    if sample_size <= 0:
        raise ValueError("sample_size must be positive")
    if mode == "formal" and workers not in {None, FORMAL_WORKERS}:
        raise ValueError(f"formal mode requires workers={FORMAL_WORKERS}")
    command = [
        str(runner_python),
        str(runner),
        "--mode",
        mode,
    ]
    if mode == "smoke":
        command.extend(("--n", str(sample_size)))
    if workers is not None:
        if workers <= 0:
            raise ValueError("workers must be positive")
        command.extend(("--workers", str(workers)))
    if output_dir is not None:
        command.extend(("--output-dir", str(output_dir)))
    return command


def _extract_run_directory(stdout: str) -> Path | None:
    """Extract the runner's pretty-printed ``run_directory`` field."""

    # The runner emits a multi-line JSON object.  Matching the escaped JSON
    # string rather than attempting to parse arbitrary log prefixes keeps this
    # helper tolerant of child diagnostics while remaining deterministic.
    matches = re.findall(r'"run_directory"\s*:\s*"((?:\\.|[^"\\])*)"', stdout)
    for encoded in reversed(matches):
        try:
            value = json.loads(f'"{encoded}"')
        except json.JSONDecodeError:
            continue
        if isinstance(value, str) and value:
            return Path(value)
    return None


def _resolve_runner_output_directory(
    completed: subprocess.CompletedProcess[str], output_dir: Path | None
) -> Path:
    candidate = Path(output_dir) if output_dir is not None else _extract_run_directory(
        completed.stdout or ""
    )
    if candidate is None:
        raise SampleValidationError("evaluator did not report a run directory")
    if not candidate.is_absolute():
        # The child runner is launched with ``cwd=REPO``.
        candidate = REPO / candidate
    # The runner prints its temporary ``.work`` path immediately before the
    # atomic rename.  Resolve the published sibling when it exists.
    if candidate.name.endswith(".work"):
        published = candidate.with_name(candidate.name[: -len(".work")])
        if published.exists():
            candidate = published
    if candidate.is_symlink():
        raise SampleValidationError(f"evaluator output directory is a symlink: {candidate}")
    try:
        resolved = candidate.resolve(strict=True)
    except (OSError, RuntimeError) as error:
        raise SampleValidationError(
            f"evaluator run directory does not exist: {candidate}"
        ) from error
    if not resolved.is_dir():
        raise SampleValidationError(f"evaluator run directory is not a directory: {resolved}")
    return resolved


def _read_output_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise SampleValidationError(f"cannot read evaluator {label}: {error}") from error
    if not isinstance(value, dict):
        raise SampleValidationError(f"evaluator {label} must be a JSON object")
    return value


def validate_runner_result(
    completed: subprocess.CompletedProcess[str],
    *,
    mode: str,
    expected_count: int,
    expected_dataset_ids: Sequence[str],
    expected_urdf_sha256: Sequence[str] | None = None,
    expected_package_paths: Sequence[Path] | None = None,
    output_dir: Path | None = None,
) -> dict[str, Any]:
    """Fail closed unless the runner produced the bound cohort receipt."""

    if completed.returncode != 0:
        raise SampleValidationError(
            f"evaluator returned non-zero status {completed.returncode}"
        )
    if mode not in {"smoke", "formal"}:
        raise SampleValidationError(f"unsupported evaluator mode: {mode!r}")
    if expected_count <= 0 or len(expected_dataset_ids) != expected_count:
        raise SampleValidationError("invalid expected evaluator cohort")
    if len(set(str(value) for value in expected_dataset_ids)) != expected_count:
        raise SampleValidationError("expected evaluator cohort contains duplicate IDs")
    if expected_urdf_sha256 is not None:
        if len(expected_urdf_sha256) != expected_count or not all(
            _is_hash(value) for value in expected_urdf_sha256
        ):
            raise SampleValidationError("invalid expected evaluator URDF hashes")
    if expected_package_paths is not None and len(expected_package_paths) != expected_count:
        raise SampleValidationError("invalid expected evaluator package bindings")

    run_directory = _resolve_runner_output_directory(completed, output_dir)
    missing = [name for name in RUNNER_OUTPUT_FILES if not (run_directory / name).is_file()]
    if missing:
        raise SampleValidationError(
            f"evaluator run directory is missing required outputs: {', '.join(missing)}"
        )

    summary_path = run_directory / "summary.json"
    records_path = run_directory / "asset_records.jsonl"
    run_manifest_path = run_directory / "manifest.json"
    summary = _read_output_json(summary_path, "summary.json")
    run_manifest = _read_output_json(run_manifest_path, "manifest.json")
    if summary.get("protocol_id") != EXPECTED_RUNNER_PROTOCOL_ID:
        raise SampleValidationError("evaluator summary protocol_id mismatch")
    if summary.get("mode") != mode or summary.get("dataset") != "PartNet-Mobility":
        raise SampleValidationError("evaluator summary mode/dataset mismatch")
    cohort = summary.get("cohort")
    if not isinstance(cohort, Mapping):
        raise SampleValidationError("evaluator summary has no cohort binding")
    if (
        cohort.get("n_eval") != expected_count
        or cohort.get("source_manifest_sha256") != EXPECTED_MANIFEST_SHA256
        or cohort.get("ordered_ids_sha256") != EXPECTED_ORDERED_IDS_SHA256
    ):
        raise SampleValidationError("evaluator summary cohort binding mismatch")
    if run_manifest.get("protocol_id") != EXPECTED_RUNNER_PROTOCOL_ID:
        raise SampleValidationError("evaluator manifest protocol_id mismatch")
    if run_manifest.get("mode") != mode or run_manifest.get("dataset") != "PartNet-Mobility":
        raise SampleValidationError("evaluator manifest mode/dataset mismatch")
    if run_manifest.get("record_count") != expected_count:
        raise SampleValidationError("evaluator manifest record_count mismatch")

    try:
        lines = records_path.read_text(encoding="utf-8").splitlines()
    except OSError as error:
        raise SampleValidationError(f"cannot read evaluator asset records: {error}") from error
    if len(lines) != expected_count:
        raise SampleValidationError(
            f"evaluator asset record count mismatch: {len(lines)} != {expected_count}"
        )
    records: list[dict[str, Any]] = []
    for index, line in enumerate(lines):
        if not line.strip():
            raise SampleValidationError(f"blank evaluator asset record at index {index}")
        try:
            record = json.loads(line)
        except json.JSONDecodeError as error:
            raise SampleValidationError(
                f"invalid evaluator asset record at index {index}: {error}"
            ) from error
        if not isinstance(record, dict):
            raise SampleValidationError(f"evaluator asset record {index} is not an object")
        if (
            record.get("selection_index") != index
            or str(record.get("dataset_id")) != str(expected_dataset_ids[index])
            or str(record.get("asset_id")) != str(expected_dataset_ids[index])
        ):
            raise SampleValidationError(
                f"evaluator asset record {index} does not preserve frozen order"
            )
        if record.get("protocol_id") != EXPECTED_RUNNER_PROTOCOL_ID:
            raise SampleValidationError(f"evaluator asset record {index} protocol mismatch")
        if record.get("status") not in VALID_RECORD_STATUSES:
            raise SampleValidationError(f"evaluator asset record {index} has invalid status")
        if expected_urdf_sha256 is not None:
            expected_hash = expected_urdf_sha256[index]
            if record.get("expected_urdf_sha256") != expected_hash:
                raise SampleValidationError(
                    f"evaluator asset record {index} expected URDF hash mismatch"
                )
            observed_hash = record.get("urdf_sha256")
            if record.get("status") == "completed" and observed_hash != expected_hash:
                raise SampleValidationError(
                    f"evaluator asset record {index} URDF hash mismatch"
                )
            if observed_hash is not None and observed_hash != expected_hash:
                raise SampleValidationError(
                    f"evaluator asset record {index} URDF hash mismatch"
                )
        if expected_package_paths is not None and record.get("package") is not None:
            try:
                observed_package = Path(str(record["package"])).resolve(strict=True)
                expected_package = Path(expected_package_paths[index]).resolve(strict=True)
            except (OSError, RuntimeError) as error:
                raise SampleValidationError(
                    f"evaluator asset record {index} package binding is unavailable"
                ) from error
            if observed_package != expected_package:
                raise SampleValidationError(
                    f"evaluator asset record {index} package binding mismatch"
                )
        records.append(record)

    outputs = run_manifest.get("outputs")
    if not isinstance(outputs, Mapping):
        raise SampleValidationError("evaluator manifest has no output hashes")
    expected_hashes = {
        "summary_sha256": sha256_file(summary_path),
        "asset_records_sha256": sha256_file(records_path),
    }
    for field, observed in expected_hashes.items():
        if outputs.get(field) != observed:
            raise SampleValidationError(f"evaluator manifest {field} mismatch")
    return {
        "run_directory": str(run_directory),
        "summary_sha256": expected_hashes["summary_sha256"],
        "asset_records_sha256": expected_hashes["asset_records_sha256"],
        "record_count": len(records),
        "status_counts": summary.get("status_counts", {}),
    }


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--expected-root", type=Path, default=DEFAULT_DATASET_ROOT)
    parser.add_argument("--expected-count", type=int, default=DEFAULT_SAMPLE_SIZE)
    print_group = parser.add_mutually_exclusive_group()
    print_group.add_argument(
        "--print-paths",
        action="store_true",
        help="print one validated package path per line (exact jq semantics)",
    )
    print_group.add_argument(
        "--print-urdf-paths",
        action="store_true",
        help="print one validated mobility.urdf path per line",
    )
    parser.add_argument(
        "--run",
        action="store_true",
        help="run the existing Table 4b evaluator after validation",
    )
    parser.add_argument("--mode", choices=("smoke", "formal"), default="smoke")
    parser.add_argument("--n", type=int, default=3, help="smoke sample size")
    parser.add_argument("--runner", type=Path, default=DEFAULT_RUNNER)
    parser.add_argument("--runner-python", type=Path, default=DEFAULT_RUNNER_PYTHON)
    parser.add_argument("--workers", type=int)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--subprocess-timeout", type=float)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    try:
        canonical_runner: Path | None = None
        runner_python: Path | None = None
        if args.run:
            if args.manifest.resolve() != DEFAULT_MANIFEST.resolve():
                raise SampleValidationError(
                    "--run requires the canonical frozen PartNet-Mobility manifest"
                )
            canonical_runner = validate_runner_binding(args.runner)
            runner_python = validate_runner_python(args.runner_python)
            if args.mode == "formal" and args.expected_count != DEFAULT_SAMPLE_SIZE:
                raise SampleValidationError("formal mode requires --expected-count 800")
            if args.mode == "formal" and args.workers not in {None, FORMAL_WORKERS}:
                raise SampleValidationError(
                    f"formal mode requires workers={FORMAL_WORKERS}"
                )
        report, paths, assets = _validate_sample(
            args.manifest,
            expected_count=args.expected_count,
            expected_dataset_root=args.expected_root,
        )
        if args.print_paths:
            print("\n".join(str(path.parent) for path in paths))
            if not args.run:
                return 0
        elif args.print_urdf_paths:
            print("\n".join(str(path) for path in paths))
            if not args.run:
                return 0
        if args.run:
            assert canonical_runner is not None and runner_python is not None
            run_count = args.n if args.mode == "smoke" else DEFAULT_SAMPLE_SIZE
            if args.mode == "smoke" and not 1 <= run_count <= len(paths):
                raise SampleValidationError(
                    f"--n must be in [1, {len(paths)}] for smoke mode"
                )
            command = build_runner_command(
                runner_python=runner_python,
                runner=canonical_runner,
                mode=args.mode,
                sample_size=run_count,
                workers=args.workers,
                output_dir=args.output_dir,
            )
            completed = subprocess.run(
                command,
                cwd=REPO,
                text=True,
                capture_output=True,
                check=False,
                timeout=args.subprocess_timeout,
            )
            if completed.stdout:
                print(completed.stdout, file=sys.stderr, end="")
            if completed.stderr:
                print(completed.stderr, file=sys.stderr, end="")
            evaluator_report: dict[str, Any] = {
                "command": command,
                "returncode": completed.returncode,
                "status": "FAIL",
            }
            if completed.returncode == 0:
                try:
                    receipt = validate_runner_result(
                        completed,
                        mode=args.mode,
                        expected_count=run_count,
                        expected_dataset_ids=[path.parent.name for path in paths[:run_count]],
                        expected_urdf_sha256=[
                            asset["urdf_sha256"] for asset in assets[:run_count]
                        ],
                        expected_package_paths=[
                            asset["package"] for asset in assets[:run_count]
                        ],
                        output_dir=args.output_dir,
                    )
                except SampleValidationError as error:
                    evaluator_report["error"] = str(error)
                else:
                    evaluator_report.update(receipt)
                    evaluator_report["status"] = "PASS"
            else:
                evaluator_report["error"] = (
                    "evaluator returned non-zero status; see captured diagnostics"
                )
            report["evaluator"] = evaluator_report
            report["status"] = evaluator_report["status"]
        print(json.dumps(report, sort_keys=True, ensure_ascii=True, indent=2))
        return 0 if report["status"] == "PASS" else 2
    except (
        SampleValidationError,
        OSError,
        ValueError,
        subprocess.TimeoutExpired,
    ) as error:
        failure = {"status": "FAIL", "error": str(error)}
        print(json.dumps(failure, sort_keys=True, ensure_ascii=True, indent=2))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
