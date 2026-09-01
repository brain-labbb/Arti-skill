#!/usr/bin/env python3
"""Run Table 3 on the exact frozen PartNet-Mobility Table 4 cohort."""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
from typing import Any
import xml.etree.ElementTree as ET


SCRIPT_PATH = Path(__file__).resolve()
REPO_ROOT = SCRIPT_PATH.parents[2]
CORE_PATH = SCRIPT_PATH.with_name("run_urdf_table3_lam.py")
FROZEN_CONTRACT_PATH = SCRIPT_PATH.with_name(
    "run_urdf_table4_partnet_mobility.py"
)
PROTOCOL_PATH = REPO_ROOT / "exp/URDF-Sim-Ready-Automatic-Evaluation.md"
INVENTORY_PATH = REPO_ROOT / "exp/dataset_inventory.json"
DEFAULT_SOURCE_ROOT = REPO_ROOT / "exp/PartNet-Mobility/data/dataset"
DEFAULT_ARCHIVE = REPO_ROOT / "exp/PartNet-Mobility/partnet-mobility-v0.zip"
DEFAULT_COHORT_MANIFEST = (
    REPO_ROOT
    / "exp/runtime/urdf_table4_partnet_mobility_n800_20260813/frozen_manifest.json"
)
DEFAULT_OUTPUT_PARENT = REPO_ROOT / "exp/runtime"

DATASET_NAME = "PartNet-Mobility"
FORMAL_N_RELEASE = 2347
FORMAL_N_EVAL = 800
FORMAL_CATEGORY_COUNT = 46
FORMAL_J_EVAL = 4078
FORMAL_COHORT_FILE_SHA256 = (
    "2ff015ee6bb377ce693126b52dd632a7565a3eaa9f0007e26122a1bb4ab99900"
)
FORMAL_ITEMS_SHA256 = (
    "5f4d0eaa7d50087edc3491a92868950a035dabcb2ad5f8c4d4970aa4c890e5e3"
)
FORMAL_SELECTED_IDS_SHA256 = (
    "ef6cb964e50dc712280256c5b2f675cc2c957095c3553b21845d3562a5011883"
)
FORMAL_CANDIDATE_POOL_SHA256 = (
    "0203a510202510cea7e469048e84b133bd65ccbc6e1e3aa90c9bfeea7807959d"
)
FORMAL_ARCHIVE_SHA256 = (
    "b47247a44246111e8d09f2c0e64b4012ae35e0dcf4bb55f68a05b604455119ff"
)
DEFAULT_SAMPLES = 21
DEFAULT_WORKERS = 4
DEFAULT_TIMEOUT_SECONDS = 120.0
SELECTION_SALT = "urdf-sim-ready-table4-partnet-mobility-n800-v1:20260813"
SELECTION_ALGORITHM = (
    "SHA256(salt + NUL + numeric dataset_id), ascending by (digest, numeric ID)"
)
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


def _load_module(path: Path, name: str):
    existing = sys.modules.get(name)
    if existing is not None:
        return existing
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import required module: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


core = _load_module(CORE_PATH, "urdf_table3_core_partnet_mobility")


def _declared_joint_hint(urdf_path: Path) -> int:
    try:
        root = ET.parse(urdf_path).getroot()
    except Exception:  # noqa: BLE001
        return 0
    return sum(node.get("type", "") != "fixed" for node in root.findall("joint"))


def _package_file_manifest(package: Path) -> list[dict[str, Any]]:
    package = package.resolve(strict=True)
    if not package.is_dir():
        raise ValueError(f"package is not a directory: {package}")
    rows: list[dict[str, Any]] = []
    for current_raw, directory_names, file_names in os.walk(
        package, followlinks=False
    ):
        current = Path(current_raw)
        directory_names.sort()
        file_names.sort()
        for name in directory_names:
            child = current / name
            if child.is_symlink():
                raise ValueError(
                    f"package contains directory symlink: {child.relative_to(package)}"
                )
        for name in file_names:
            path = current / name
            relative = path.relative_to(package).as_posix()
            if path.is_symlink():
                raise ValueError(f"package contains file symlink: {relative}")
            canonical = path.resolve(strict=True)
            canonical.relative_to(package)
            if not canonical.is_file():
                raise ValueError(f"package entry is not a regular file: {relative}")
            rows.append(
                {
                    "path": relative,
                    "bytes": canonical.stat().st_size,
                    "sha256": core.sha256_file(canonical),
                }
            )
    return rows


def _package_binding(package: Path) -> dict[str, Any]:
    files = _package_file_manifest(package)
    return {
        "file_count": len(files),
        "total_bytes": sum(row["bytes"] for row in files),
        "files": files,
        "content_manifest_sha256": core.canonical_sha256(files),
    }


def _selection_digest(dataset_id: str) -> str:
    return hashlib.sha256(
        f"{SELECTION_SALT}\0{dataset_id}".encode("utf-8")
    ).hexdigest()


def _input_identity(item: dict[str, Any]) -> str:
    try:
        payload = {field: item[field] for field in INPUT_IDENTITY_FIELDS}
    except KeyError as exc:
        raise ValueError(
            f"PartNet-Mobility input identity field is missing: {exc.args[0]}"
        ) from exc
    return core.canonical_sha256(payload)


def _release_ids(source_root: Path) -> list[str]:
    identities: list[str] = []
    for child in source_root.iterdir():
        if child.is_symlink():
            raise ValueError(f"PartNet-Mobility release child is a symlink: {child}")
        if not child.is_dir() or not child.name.isdecimal():
            raise ValueError(f"invalid PartNet-Mobility release child: {child.name}")
        identities.append(child.name)
    return sorted(identities, key=int)


def _read_category(package: Path, dataset_id: str) -> str:
    meta_path = package / "meta.json"
    if meta_path.is_symlink() or not meta_path.is_file():
        raise ValueError(f"meta.json is missing or symlinked: {dataset_id}")
    try:
        payload = json.loads(meta_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"meta.json is unavailable or invalid: {dataset_id}") from exc
    category = str(payload.get("model_cat", "")).strip()
    if not category:
        raise ValueError(f"meta.json model_cat is missing: {dataset_id}")
    return category


def _release_categories(
    source_root: Path, release_ids: list[str]
) -> tuple[dict[str, str], str]:
    categories: dict[str, str] = {}
    rows: list[dict[str, str]] = []
    for dataset_id in release_ids:
        category = _read_category(source_root / dataset_id, dataset_id)
        categories[dataset_id] = category
        rows.append({"dataset_id": dataset_id, "category": category})
    return categories, core.canonical_sha256(rows)


def _collision_mesh_inventory(
    package: Path, urdf_path: Path
) -> list[dict[str, Any]]:
    root = ET.parse(urdf_path).getroot()
    references = sorted(
        {
            mesh.get("filename", "").replace("\\", "/")
            for mesh in root.findall("link/collision/geometry/mesh")
            if mesh.get("filename")
        }
    )
    inventory: list[dict[str, Any]] = []
    for reference in references:
        path = package / reference
        exists = path.is_file()
        inventory.append(
            {
                "path": reference,
                "exists": exists,
                "size_bytes": path.stat().st_size if exists else None,
                "sha256": core.sha256_file(path) if exists else None,
            }
        )
    return inventory


def _inventory_entry() -> tuple[dict[str, Any], str]:
    payload = json.loads(INVENTORY_PATH.read_text(encoding="utf-8"))
    for entry in payload.get("datasets", []):
        if entry.get("name") == DATASET_NAME:
            return entry, core.sha256_file(INVENTORY_PATH)
    raise RuntimeError("PartNet-Mobility is absent from dataset_inventory.json")


def _current_runtime_identity() -> dict[str, Any]:
    import numpy
    import pybullet
    import scipy
    from scipy.stats import _qmc

    python_executable = Path(os.path.abspath(sys.executable))
    bullet_module = Path(pybullet.__file__).resolve(strict=True)
    qmc_module = Path(_qmc.__file__).resolve(strict=True)
    return {
        "python_executable": str(python_executable),
        "python_executable_sha256": core.sha256_file(python_executable),
        "python_version": sys.version,
        "python_cache_tag": sys.implementation.cache_tag,
        "pybullet_api_version": pybullet.getAPIVersion(),
        "pybullet_module": str(bullet_module),
        "pybullet_module_sha256": core.sha256_file(bullet_module),
        "scipy_version": scipy.__version__,
        "scipy_qmc_module": str(qmc_module),
        "scipy_qmc_module_sha256": core.sha256_file(qmc_module),
        "numpy_version": numpy.__version__,
    }


def _require_runtime_identity(expected: dict[str, Any], *, formal: bool) -> None:
    if not formal:
        return
    observed = _current_runtime_identity()
    if observed != expected:
        differing = sorted(
            key
            for key in set(expected) | set(observed)
            if expected.get(key) != observed.get(key)
        )
        raise RuntimeError(
            f"formal runtime identity mismatch in fields: {differing}"
        )


def load_cohort(
    source_root: Path,
    cohort_manifest: Path,
    *,
    formal: bool,
) -> dict[str, Any]:
    source_root = source_root.resolve(strict=True)
    cohort_manifest = cohort_manifest.resolve(strict=True)
    cohort_file_hash = core.sha256_file(cohort_manifest)
    manifest = json.loads(cohort_manifest.read_text(encoding="utf-8"))
    expected_n = manifest.get("sample_size")
    if not isinstance(expected_n, int) or expected_n <= 0:
        raise ValueError("PartNet-Mobility frozen manifest has no positive sample_size")
    if manifest.get("protocol_id") != "urdf_sim_ready_table4_partnet_mobility_n800_v1":
        raise ValueError("unexpected PartNet-Mobility frozen protocol")
    if manifest.get("status") != "FROZEN" or manifest.get("qualification_smoke") is not False:
        raise ValueError("PartNet-Mobility cohort must be a formal frozen selection")
    selection_policy = manifest.get("selection_policy")
    if not isinstance(selection_policy, dict):
        raise ValueError("PartNet-Mobility selection policy is missing")
    if (
        selection_policy.get("algorithm") != SELECTION_ALGORITHM
        or selection_policy.get("salt") != SELECTION_SALT
        or selection_policy.get("outcome_based_filtering") is not False
        or selection_policy.get("selected_failures_retained_without_replacement")
        is not True
    ):
        raise ValueError("PartNet-Mobility selection policy mismatch")
    boundary = manifest.get("cohort_boundary")
    if not isinstance(boundary, dict) or (
        boundary.get("is_full_release_cohort") is not False
        or boundary.get("is_shared_category_balanced_cohort") is not False
    ):
        raise ValueError("PartNet-Mobility cohort boundary mismatch")
    items = manifest.get("items")
    if not isinstance(items, list) or len(items) != expected_n:
        raise ValueError("PartNet-Mobility cohort selected item count mismatch")
    if core.canonical_sha256(items) != manifest.get("items_sha256"):
        raise ValueError("PartNet-Mobility items SHA256 mismatch")
    runtime_identities: list[dict[str, Any]] = []
    for item in items:
        if not isinstance(item, dict) or not isinstance(
            item.get("runtime_identity"), dict
        ):
            raise ValueError("PartNet-Mobility frozen runtime identity is missing")
        runtime_identity = item["runtime_identity"]
        if core.canonical_sha256(runtime_identity) != item.get(
            "runtime_identity_sha256"
        ):
            raise ValueError("PartNet-Mobility frozen runtime identity hash mismatch")
        runtime_identities.append(runtime_identity)
    frozen_runtime_identity = runtime_identities[0]
    if any(identity != frozen_runtime_identity for identity in runtime_identities[1:]):
        raise ValueError("PartNet-Mobility frozen runtime identities are inconsistent")
    _require_runtime_identity(frozen_runtime_identity, formal=formal)
    if Path(str(manifest.get("dataset_root", ""))).resolve(strict=True) != source_root:
        raise ValueError("PartNet-Mobility dataset root mismatch")

    release_ids = _release_ids(source_root)
    if manifest.get("release_asset_count") != len(release_ids):
        raise ValueError("PartNet-Mobility release asset count mismatch")
    candidate_pool_hash = core.canonical_sha256(release_ids)
    if candidate_pool_hash != manifest.get("candidate_pool_identity_sha256"):
        raise ValueError("PartNet-Mobility candidate pool identity mismatch")
    release_categories, release_category_mapping_hash = _release_categories(
        source_root, release_ids
    )

    archive_payload = manifest.get("archive")
    if not isinstance(archive_payload, dict):
        raise ValueError("PartNet-Mobility archive identity is missing")
    archive_path = Path(str(archive_payload.get("path", "")))
    if archive_path.is_symlink() or not archive_path.is_file():
        raise ValueError("PartNet-Mobility archive must be a regular non-symlink file")
    archive_path = archive_path.resolve(strict=True)
    archive_bytes = archive_path.stat().st_size
    if archive_bytes != archive_payload.get("size_bytes"):
        raise ValueError("PartNet-Mobility archive size mismatch")
    archive_hash = core.sha256_file(archive_path)
    if archive_hash != archive_payload.get("sha256"):
        raise ValueError("PartNet-Mobility archive SHA256 mismatch")

    assets: list[dict[str, Any]] = []
    category_rows: list[dict[str, str]] = []
    seen_ids: set[str] = set()
    for index, source_item in enumerate(items):
        if not isinstance(source_item, dict) or source_item.get("order") != index:
            raise ValueError(f"PartNet-Mobility selection index mismatch at {index}")
        dataset_id = source_item.get("dataset_id")
        if not isinstance(dataset_id, str) or not dataset_id.isdecimal():
            raise ValueError("PartNet-Mobility dataset_id must be numeric")
        if dataset_id in seen_ids:
            raise ValueError(f"duplicate PartNet-Mobility dataset_id: {dataset_id}")
        seen_ids.add(dataset_id)
        if dataset_id not in release_categories:
            raise ValueError(f"PartNet-Mobility item is absent from release: {dataset_id}")
        if "package" in source_item:
            raise ValueError("PartNet-Mobility item must not override package path")
        if source_item.get("protocol_id") != manifest["protocol_id"]:
            raise ValueError(f"PartNet-Mobility item protocol mismatch: {dataset_id}")
        if source_item.get("selection_digest") != _selection_digest(dataset_id):
            raise ValueError(f"PartNet-Mobility selection digest mismatch: {dataset_id}")
        if _input_identity(source_item) != source_item.get("input_identity_sha256"):
            raise ValueError(f"PartNet-Mobility input identity mismatch: {dataset_id}")
        if source_item.get("package_audit_success") is not True:
            raise ValueError(f"frozen package audit failed: {dataset_id}")
        if source_item.get("missing_core_files") not in (None, []):
            raise ValueError(f"frozen package has missing core files: {dataset_id}")
        package = (source_root / dataset_id).resolve(strict=True)
        package.relative_to(source_root)
        urdf_candidates = sorted(package.glob("*.urdf"), key=lambda path: path.name)
        if len(urdf_candidates) != 1 or urdf_candidates[0].name != "mobility.urdf":
            raise ValueError(
                f"package must contain exactly one top-level mobility.urdf: {dataset_id}"
            )
        urdf_path = (package / "mobility.urdf").resolve(strict=True)
        if urdf_path.parent != package or urdf_path.is_symlink():
            raise ValueError(f"invalid PartNet-Mobility primary URDF: {dataset_id}")
        if core.sha256_file(urdf_path) != source_item.get("urdf_sha256"):
            raise ValueError(f"PartNet-Mobility URDF content drift: {dataset_id}")
        bounding_box_hash = source_item.get("bounding_box_sha256")
        if bounding_box_hash is not None:
            bounding_box = package / "bounding_box.json"
            if (
                bounding_box.is_symlink()
                or not bounding_box.is_file()
                or core.sha256_file(bounding_box) != bounding_box_hash
            ):
                raise ValueError(
                    f"PartNet-Mobility bounding-box content drift: {dataset_id}"
                )
        collision_inventory = _collision_mesh_inventory(package, urdf_path)
        if (
            collision_inventory != source_item.get("collision_mesh_files")
            or core.canonical_sha256(collision_inventory)
            != source_item.get("collision_mesh_inventory_sha256")
        ):
            raise ValueError(
                f"PartNet-Mobility collision mesh inventory drift: {dataset_id}"
            )
        category = str(source_item.get("category", "")).strip()
        if not category or release_categories[dataset_id] != category:
            raise ValueError(f"PartNet-Mobility category drift: {dataset_id}")
        declared_joint_count = _declared_joint_hint(urdf_path)
        if declared_joint_count != source_item.get("movable_dof_count"):
            raise ValueError(f"declared movable joint count drift: {dataset_id}")
        binding = _package_binding(package)
        category_rows.append({"dataset_id": dataset_id, "category": category})
        assets.append(
            {
                "asset_key": dataset_id,
                "asset_id": dataset_id,
                "dataset_id": dataset_id,
                "raw_category": category,
                "category": category,
                "selection_index": index,
                "selection_rank": index + 1,
                "selection_hash": source_item["selection_digest"],
                "package": str(package),
                "package_binding": binding,
                "package_content_manifest_sha256": binding[
                    "content_manifest_sha256"
                ],
                "urdf_path": str(urdf_path),
                "urdf_sha256": source_item["urdf_sha256"],
                "frozen_item_sha256": core.canonical_sha256(source_item),
                "input_identity_sha256": source_item["input_identity_sha256"],
                "missing_collision_mesh_reference_count": int(
                    source_item.get("missing_collision_mesh_reference_count") or 0
                ),
                "declared_joint_count_hint": declared_joint_count,
            }
        )

    selected_ids = [row["asset_id"] for row in assets]
    if core.canonical_sha256(selected_ids) != manifest.get(
        "ordered_selected_ids_sha256"
    ):
        raise ValueError("PartNet-Mobility ordered selected ID hash mismatch")
    eval_category_count = len({row["category"] for row in assets})
    j_eval = sum(row["declared_joint_count_hint"] for row in assets)
    inventory: dict[str, Any] = {}
    inventory_hash: str | None = None
    if formal:
        inventory, inventory_hash = _inventory_entry()
        inventory_source = inventory.get("source", {})
        inventory_verification = inventory.get("verification", {})
        checks = {
            "cohort file SHA256": (
                cohort_file_hash,
                FORMAL_COHORT_FILE_SHA256,
            ),
            "cohort assets": (len(assets), FORMAL_N_EVAL),
            "release assets": (len(release_ids), FORMAL_N_RELEASE),
            "release categories": (
                len(set(release_categories.values())),
                FORMAL_CATEGORY_COUNT,
            ),
            "eval categories": (eval_category_count, FORMAL_CATEGORY_COUNT),
            "declared joints": (j_eval, FORMAL_J_EVAL),
            "items SHA256": (manifest["items_sha256"], FORMAL_ITEMS_SHA256),
            "ordered selected IDs SHA256": (
                manifest["ordered_selected_ids_sha256"],
                FORMAL_SELECTED_IDS_SHA256,
            ),
            "candidate pool SHA256": (
                candidate_pool_hash,
                FORMAL_CANDIDATE_POOL_SHA256,
            ),
            "archive SHA256": (archive_hash, FORMAL_ARCHIVE_SHA256),
            "inventory status": (
                inventory.get("status"),
                "LOCAL_COMPLETE_PROVENANCE_LIMITED",
            ),
            "inventory revision": (
                inventory_source.get("revision"),
                "ee0aa3ef1df16181d76d83f7415aa8c94ed1da8f",
            ),
            "inventory release IDs": (
                inventory_verification.get("local_ids_listed_by_fixed_hf_revision"),
                FORMAL_N_RELEASE,
            ),
        }
        for name, (observed, expected) in checks.items():
            if observed != expected:
                raise RuntimeError(
                    f"formal PartNet-Mobility {name} mismatch: "
                    f"{observed!r} != {expected!r}"
                )
        if source_root != DEFAULT_SOURCE_ROOT.resolve(strict=True):
            raise RuntimeError("formal PartNet-Mobility source root mismatch")
        if cohort_manifest != DEFAULT_COHORT_MANIFEST.resolve(strict=True):
            raise RuntimeError("formal PartNet-Mobility cohort manifest path mismatch")
        if archive_path != DEFAULT_ARCHIVE.resolve(strict=True):
            raise RuntimeError("formal PartNet-Mobility archive path mismatch")
        if Path(str(inventory.get("urdf_root", ""))).resolve(strict=True) != source_root:
            raise RuntimeError("formal PartNet-Mobility inventory root mismatch")

    inventory_source = inventory.get("source", {})
    inventory_verification = inventory.get("verification", {})
    return {
        "source_root": str(source_root),
        "cohort_manifest_path": str(cohort_manifest),
        "cohort_manifest_file_sha256": cohort_file_hash,
        "cohort_protocol_id": manifest["protocol_id"],
        "frozen_runtime_identity": frozen_runtime_identity,
        "frozen_runtime_identity_sha256": core.canonical_sha256(
            frozen_runtime_identity
        ),
        "cohort_boundary": manifest["cohort_boundary"],
        "selection_policy": manifest["selection_policy"],
        "sampling": manifest.get("sampling"),
        "items_sha256": manifest["items_sha256"],
        "ordered_selected_ids_sha256": manifest[
            "ordered_selected_ids_sha256"
        ],
        "candidate_pool_identity_sha256": candidate_pool_hash,
        "archive": {
            "path": str(archive_path),
            "bytes": archive_bytes,
            "sha256": archive_hash,
        },
        "inventory_sha256": inventory_hash,
        "release_status": inventory.get(
            "status", "LOCAL_COMPLETE_PROVENANCE_LIMITED"
        ),
        "source_repo_id": inventory_source.get("repo_id"),
        "source_revision": inventory_source.get("revision"),
        "source_bytes_authenticated_against_revision": inventory_verification.get(
            "local_bytes_authenticated_against_hf_revision"
        ),
        "n_release": len(release_ids),
        "release_category_count": len(set(release_categories.values())),
        "eval_category_count": eval_category_count,
        "release_category_mapping_sha256": release_category_mapping_hash,
        "eval_category_mapping_sha256": core.canonical_sha256(category_rows),
        "j_eval": j_eval,
        "known_missing_collision_mesh_asset_count": sum(
            row["missing_collision_mesh_reference_count"] > 0 for row in assets
        ),
        "known_missing_collision_mesh_reference_count": sum(
            row["missing_collision_mesh_reference_count"] for row in assets
        ),
        "assets": assets,
    }


def _config(args: argparse.Namespace) -> dict[str, Any]:
    return {
        "protocol_id": "urdf-sim-ready-table3-partnet-mobility-table4-cohort-v1",
        "samples_per_joint": args.samples,
        "continuous_interval": list(core.CONTINUOUS_INTERVAL),
        "continuous_non_degenerate_policy": (
            "maximum_excursion_from_q0_over_frozen_interval"
        ),
        "q0_policy": "zero_clipped_to_declared_interval",
        "translation_motion_threshold_bbox_diagonal": (
            core.TRANSLATION_MOTION_THRESHOLD
        ),
        "rotation_motion_threshold_rad": core.ROTATION_MOTION_THRESHOLD_RAD,
        "unchanged_translation_tolerance_bbox_diagonal": (
            core.UNCHANGED_TRANSLATION_TOLERANCE
        ),
        "unchanged_rotation_tolerance_rad": (
            core.UNCHANGED_ROTATION_TOLERANCE_RAD
        ),
        "roundtrip_translation_tolerance_bbox_diagonal": (
            core.ROUNDTRIP_TRANSLATION_TOLERANCE
        ),
        "roundtrip_rotation_tolerance_rad": (
            core.ROUNDTRIP_ROTATION_TOLERANCE_RAD
        ),
        "strict_asset_requires_at_least_one_declared_movable_joint": True,
        "asset_timeout_seconds": args.asset_timeout_seconds,
        "workers": args.workers,
    }


def build_manifest(args: argparse.Namespace, loaded: dict[str, Any]) -> dict[str, Any]:
    selected = loaded["assets"] if args.limit is None else loaded["assets"][: args.limit]
    config = _config(args)
    environment = core._environment()
    manifest = {
        "schema_version": 1,
        "dataset": DATASET_NAME,
        "classification": (
            "FORMAL" if args.mode == "formal" else "NON_FORMAL_SMOKE"
        ),
        "created_at": core.utc_now(),
        "source": {
            "source_root": loaded["source_root"],
            "release_status": loaded["release_status"],
            "n_release": loaded["n_release"],
            "release_category_count": loaded["release_category_count"],
            "eval_category_count": loaded["eval_category_count"],
            "source_repo_id": loaded["source_repo_id"],
            "source_revision": loaded["source_revision"],
            "source_bytes_authenticated_against_revision": loaded[
                "source_bytes_authenticated_against_revision"
            ],
            "inventory_sha256": loaded["inventory_sha256"],
            "archive": loaded["archive"],
            "cohort_manifest_path": loaded["cohort_manifest_path"],
            "cohort_manifest_file_sha256": loaded[
                "cohort_manifest_file_sha256"
            ],
            "cohort_protocol_id": loaded["cohort_protocol_id"],
            "frozen_runtime_identity_sha256": loaded[
                "frozen_runtime_identity_sha256"
            ],
            "cohort_asset_count": len(loaded["assets"]),
            "cohort_boundary": loaded["cohort_boundary"],
            "items_sha256": loaded["items_sha256"],
            "ordered_selected_ids_sha256": loaded[
                "ordered_selected_ids_sha256"
            ],
            "candidate_pool_identity_sha256": loaded[
                "candidate_pool_identity_sha256"
            ],
            "release_category_mapping_sha256": loaded[
                "release_category_mapping_sha256"
            ],
            "eval_category_mapping_sha256": loaded[
                "eval_category_mapping_sha256"
            ],
            "known_missing_collision_mesh_asset_count": loaded[
                "known_missing_collision_mesh_asset_count"
            ],
            "known_missing_collision_mesh_reference_count": loaded[
                "known_missing_collision_mesh_reference_count"
            ],
        },
        "selection": {
            "algorithm": (
                "exact existing Table 4 frozen manifest .items[].dataset_id order; "
                "optional smoke prefix only"
            ),
            "source_policy": loaded["selection_policy"],
            "source_sampling": loaded["sampling"],
            "cohort_type": "FROZEN_HASH_RANKED_SAMPLE_NOT_CATEGORY_BALANCED",
            "requested_limit": args.limit,
            "n_eval": len(selected),
            "source_selected_asset_ids_sha256": loaded[
                "ordered_selected_ids_sha256"
            ],
            "selected_asset_ids_sha256": core.canonical_sha256(
                [row["asset_id"] for row in selected]
            ),
            "selected_packages_sha256": core.canonical_sha256(
                [row["package"] for row in selected]
            ),
            "selected_frozen_items_sha256": core.canonical_sha256(
                [row["frozen_item_sha256"] for row in selected]
            ),
            "selection_order_preserved": True,
            "outcome_based_reselection": False,
        },
        "evaluation": {
            "protocol_path": str(PROTOCOL_PATH.resolve(strict=True)),
            "protocol_sha256": core.sha256_file(PROTOCOL_PATH.resolve(strict=True)),
            "adapter_path": str(SCRIPT_PATH),
            "adapter_sha256": core.sha256_file(SCRIPT_PATH),
            "core_evaluator_path": str(CORE_PATH.resolve(strict=True)),
            "core_evaluator_sha256": core.sha256_file(CORE_PATH.resolve(strict=True)),
            "frozen_contract_path": str(FROZEN_CONTRACT_PATH.resolve(strict=True)),
            "frozen_contract_sha256": core.sha256_file(
                FROZEN_CONTRACT_PATH.resolve(strict=True)
            ),
            "config": config,
            "config_sha256": core.canonical_sha256(config),
            "environment": environment,
            "environment_sha256": core.canonical_sha256(environment),
            "frozen_runtime_identity": loaded["frozen_runtime_identity"],
        },
        "records": selected,
    }
    manifest["manifest_content_sha256"] = core._manifest_self_hash(manifest)
    return manifest


def _record_self_hash(record: dict[str, Any]) -> str:
    payload = dict(record)
    payload.pop("record_content_sha256", None)
    return core.canonical_sha256(payload)


def _validate_record_binding(record: dict[str, Any], job: dict[str, Any]) -> None:
    if record.get("record_content_sha256") != _record_self_hash(record):
        raise RuntimeError(f"record content SHA256 mismatch: {job['asset_key']}")
    expected_fields = {
        "asset_key": job["asset_key"],
        "asset_id": job["asset_id"],
        "dataset_id": job["dataset_id"],
        "category": job["category"],
        "raw_category": job["raw_category"],
        "selection_index": job["selection_index"],
        "selection_rank": job["selection_rank"],
        "selection_hash": job["selection_hash"],
        "package": job["package"],
        "package_content_manifest_sha256": job[
            "package_content_manifest_sha256"
        ],
        "urdf_sha256": job["urdf_sha256"],
        "frozen_item_sha256": job["frozen_item_sha256"],
        "input_identity_sha256": job["input_identity_sha256"],
        "missing_collision_mesh_reference_count": job[
            "missing_collision_mesh_reference_count"
        ],
        "manifest_content_sha256": job["manifest_content_sha256"],
    }
    mismatched = sorted(
        field
        for field, expected in expected_fields.items()
        if record.get(field) != expected
    )
    if mismatched:
        raise RuntimeError(
            f"record manifest binding mismatch for {job['asset_key']}: {mismatched}"
        )
    expected_joint_count = int(job["declared_joint_count_hint"])
    if record.get("declared_joint_count") != expected_joint_count:
        raise RuntimeError(
            f"record declared joint count mismatch: {job['asset_key']}"
        )
    joints = record.get("joints")
    if not isinstance(joints, list) or len(joints) != expected_joint_count:
        raise RuntimeError(f"record joint schema mismatch: {job['asset_key']}")
    if any(not isinstance(joint, dict) for joint in joints):
        raise RuntimeError(f"record joint schema mismatch: {job['asset_key']}")
    if record.get("status") not in {"completed", "error", "timeout"}:
        raise RuntimeError(f"record status mismatch: {job['asset_key']}")
    expected_strict = expected_joint_count > 0 and all(
        bool(joint.get("joint_level_pass")) for joint in joints
    )
    if bool(record.get("strict_kinematic_pass")) != expected_strict:
        raise RuntimeError(f"record strict pass mismatch: {job['asset_key']}")


def _bind_record(record: dict[str, Any], job: dict[str, Any]) -> dict[str, Any]:
    record.pop("record_content_sha256", None)
    record.update(
        {
            "asset_id": job["asset_id"],
            "dataset_id": job["dataset_id"],
            "category": job["category"],
            "raw_category": job["raw_category"],
            "selection_index": job["selection_index"],
            "selection_rank": job["selection_rank"],
            "selection_hash": job["selection_hash"],
            "package": job["package"],
            "package_content_manifest_sha256": job[
                "package_content_manifest_sha256"
            ],
            "urdf_sha256": job["urdf_sha256"],
            "frozen_item_sha256": job["frozen_item_sha256"],
            "input_identity_sha256": job["input_identity_sha256"],
            "missing_collision_mesh_reference_count": job[
                "missing_collision_mesh_reference_count"
            ],
            "manifest_content_sha256": job["manifest_content_sha256"],
            "completed_at": core.utc_now(),
        }
    )
    record["record_content_sha256"] = _record_self_hash(record)
    return record


def _reject_symlink_components(path: Path, *, label: str) -> None:
    absolute = Path(os.path.abspath(path))
    current = Path(absolute.anchor)
    for part in absolute.parts[1:]:
        current /= part
        if current.is_symlink():
            raise ValueError(f"{label} contains a symlink component: {absolute}")


def _validate_internal_paths(
    job_path: Path, result_path: Path
) -> tuple[Path, Path]:
    job_path = Path(os.path.abspath(job_path))
    result_path = Path(os.path.abspath(result_path))
    _reject_symlink_components(job_path, label="internal job path")
    _reject_symlink_components(result_path.parent, label="internal result path")
    if not job_path.is_file() or job_path.name != "job.json":
        raise ValueError("internal job must be a regular job.json file")
    if result_path.exists() or result_path.is_symlink():
        raise ValueError("internal result path must not already exist")
    if result_path.name != "result.json" or result_path.parent != job_path.parent:
        raise ValueError("internal result must be result.json in the job directory")
    job_root = job_path.parent
    scratch_root = job_root.parent
    if (
        not job_root.is_dir()
        or not job_root.name.startswith("job_")
        or not scratch_root.is_dir()
        or scratch_root.name != ".worker_scratch"
    ):
        raise ValueError("internal job directory is outside a worker scratch root")
    return job_path, result_path


def run_internal_job(job_path: Path, result_path: Path) -> int:
    job_path, result_path = _validate_internal_paths(job_path, result_path)
    job = json.loads(job_path.read_text(encoding="utf-8"))
    urdf_path = Path(job["urdf_path"])
    package = Path(job["package"])
    failure: str | None = None
    try:
        if not urdf_path.is_file() or urdf_path.is_symlink():
            failure = "selected URDF is missing"
        elif urdf_path.resolve(strict=True) != (package / "mobility.urdf").resolve(
            strict=True
        ):
            failure = "selected URDF package identity mismatch"
        elif core.sha256_file(urdf_path) != job["urdf_sha256"]:
            failure = "selected URDF changed after freeze"
        elif _package_binding(package) != job["package_binding"]:
            failure = "selected package changed after freeze"
    except Exception as exc:  # noqa: BLE001
        failure = f"selected package validation failed: {type(exc).__name__}: {exc}"

    if failure is not None:
        record = core.failed_record(
            job["asset_key"], job["declared_joint_count_hint"], failure
        )
    else:
        record = core.evaluate_urdf(
            urdf_path,
            job["asset_key"],
            samples=job["samples"],
            declared_joint_count_hint=job["declared_joint_count_hint"],
        )
        try:
            changed = (
                core.sha256_file(urdf_path) != job["urdf_sha256"]
                or _package_binding(package) != job["package_binding"]
            )
        except Exception:  # noqa: BLE001
            changed = True
        if changed:
            record = core.failed_record(
                job["asset_key"],
                job["declared_joint_count_hint"],
                "selected package changed during evaluation",
            )
    core.atomic_write_json(result_path, _bind_record(record, job))
    return 0


def _execute_job(
    job: dict[str, Any], scratch_root: Path, timeout_seconds: float
) -> dict[str, Any]:
    job_root = Path(tempfile.mkdtemp(prefix="job_", dir=scratch_root))
    job_path = job_root / "job.json"
    result_path = job_root / "result.json"
    core.atomic_write_json(job_path, job)
    environment = dict(os.environ)
    environment.update(
        {
            "OMP_NUM_THREADS": "1",
            "OPENBLAS_NUM_THREADS": "1",
            "MKL_NUM_THREADS": "1",
            "NUMEXPR_NUM_THREADS": "1",
        }
    )
    process = subprocess.Popen(
        [
            sys.executable,
            str(SCRIPT_PATH),
            "--internal-job",
            str(job_path),
            "--internal-result",
            str(result_path),
        ],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        start_new_session=True,
        env=environment,
    )
    try:
        _stdout, stderr = process.communicate(timeout=timeout_seconds)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.communicate(timeout=2.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.communicate()
        return _bind_record(
            core.failed_record(
                job["asset_key"],
                job["declared_joint_count_hint"],
                f"asset timeout after {timeout_seconds:g} seconds",
                status="timeout",
            ),
            job,
        )
    if process.returncode != 0 or not result_path.is_file():
        detail = stderr.decode("utf-8", errors="replace")[-4000:]
        return _bind_record(
            core.failed_record(
                job["asset_key"],
                job["declared_joint_count_hint"],
                f"worker failed with exit {process.returncode}: {detail}",
                status="error",
            ),
            job,
        )
    result = json.loads(result_path.read_text(encoding="utf-8"))
    try:
        _validate_record_binding(result, job)
    except (KeyError, TypeError, ValueError, RuntimeError) as exc:
        return _bind_record(
            core.failed_record(
                job["asset_key"],
                job["declared_joint_count_hint"],
                f"worker result validation failed: {type(exc).__name__}: {exc}",
            ),
            job,
        )
    return result


def _summary_markdown(summary: dict[str, Any], manifest: dict[str, Any]) -> str:
    metrics = summary["metrics"]
    lines = [
        "# PartNet-Mobility Table 3 Kinematic Executability",
        "",
        f"Run classification: **{manifest['classification']}**.",
        "",
        (
            f"Exact Table 4 frozen cohort: N_eval={summary['n_eval']} from "
            f"N_release={manifest['source']['n_release']}; J_eval={summary['j_eval']}. "
            "Existing manifest item order was preserved without resampling."
        ),
        "",
        "| Metric | Result |",
        "|---|---:|",
    ]
    for metric in core.JOINT_RATE_METRICS:
        value = metrics[metric]
        percentage = "N/A" if value["rate"] is None else f"{100 * value['rate']:.2f}%"
        lines.append(
            f"| {metric} | {value['passed']} / {value['denominator']} "
            f"({percentage}) |"
        )
    roundtrip = metrics["fk_roundtrip_error"]
    lines.append(
        "| fk_roundtrip_error | "
        f"translation={roundtrip['max_normalized_translation']}; "
        f"rotation_rad={roundtrip['max_rotation_rad']}; "
        f"coverage={roundtrip['measured_joint_count']} / "
        f"{roundtrip['denominator']} ({roundtrip['status']}) |"
    )
    strict = metrics["strict_kinematic_pass"]
    strict_percentage = (
        "N/A" if strict["rate"] is None else f"{100 * strict['rate']:.2f}%"
    )
    lines.append(
        f"| strict_kinematic_pass | {strict['passed']} / "
        f"{strict['denominator']} ({strict_percentage}) |"
    )
    macro = summary["category_macro"]
    lines.extend(
        [
            "",
            (
                f"Category macro average over {macro['category_count']} observed raw "
                f"categories ({macro['joint_metric_category_count']} with at least one "
                "declared movable joint):"
            ),
            "",
            "| Metric | Category macro |",
            "|---|---:|",
        ]
    )
    for metric in (*core.JOINT_RATE_METRICS, "strict_kinematic_pass"):
        value = macro["metrics"][metric]
        percentage = "N/A" if value["rate"] is None else f"{100 * value['rate']:.2f}%"
        lines.append(
            f"| {metric} | {percentage} (categories={value['category_count']}) |"
        )
    lines.extend(
        [
            "",
            (
                "Frozen input inventory records "
                f"{manifest['source']['known_missing_collision_mesh_reference_count']} "
                "missing collision-mesh references across "
                f"{manifest['source']['known_missing_collision_mesh_asset_count']} assets; "
                "these assets remain in the frozen denominators."
            ),
            "",
            (
                "This evaluates executable declared kinematics only, not semantic joints, "
                "collision safety, dynamics, or real-world fidelity."
            ),
            "",
        ]
    )
    return "\n".join(lines)


def _validate_resume_paths(output: Path) -> dict[str, Any]:
    if output.is_symlink() or not output.is_dir():
        raise RuntimeError(f"resume output must be a regular directory: {output}")
    for name in ("manifest.json", "asset_records.jsonl", "checkpoint.json"):
        path = output / name
        if path.is_symlink():
            raise RuntimeError(f"resume {name} must not be a symlink")
        if not path.is_file():
            raise RuntimeError(f"resume {name} must be a regular file")
    scratch = output / ".worker_scratch"
    if scratch.is_symlink():
        raise RuntimeError("resume .worker_scratch must not be a symlink")
    if not scratch.is_dir():
        raise RuntimeError("resume .worker_scratch must be a regular directory")
    try:
        checkpoint = json.loads(
            (output / "checkpoint.json").read_text(encoding="utf-8")
        )
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeError("resume checkpoint is unreadable or invalid") from exc
    if not isinstance(checkpoint, dict):
        raise RuntimeError("resume checkpoint must be an object")
    return checkpoint


def _prepare_output(args: argparse.Namespace, n_eval: int) -> Path:
    if args.output is None:
        timestamp = core.datetime.now(core.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        args.output = (
            DEFAULT_OUTPUT_PARENT
            / f"urdf_table3_partnet_mobility_table4_n{n_eval}_{timestamp}"
        )
    if args.output.is_symlink():
        raise ValueError(f"output must not be a symlink: {args.output}")
    output = args.output.resolve(strict=False)
    try:
        output.relative_to(REPO_ROOT.resolve(strict=True))
    except ValueError as exc:
        raise ValueError(f"output must be inside repository: {output}") from exc
    if args.resume:
        if not output.is_dir():
            raise FileNotFoundError(f"resume output does not exist: {output}")
    else:
        output.mkdir(parents=True, exist_ok=False)
    return output


def run(args: argparse.Namespace) -> Path:
    validate_contract(args)
    output: Path | None = None
    if args.resume:
        output = _prepare_output(args, 0)
        checkpoint = _validate_resume_paths(output)
        if checkpoint.get("state") == "complete":
            raise RuntimeError("completed evaluation output is immutable")
    loaded = load_cohort(
        args.source_root,
        args.cohort_manifest,
        formal=args.mode == "formal",
    )
    n_eval = len(loaded["assets"]) if args.limit is None else args.limit
    if output is None:
        output = _prepare_output(args, n_eval)
    manifest_path = output / "manifest.json"
    records_path = output / "asset_records.jsonl"
    checkpoint_path = output / "checkpoint.json"
    scratch = output / ".worker_scratch"
    if args.resume:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if manifest.get("manifest_content_sha256") != core._manifest_self_hash(
            manifest
        ):
            raise RuntimeError("resume manifest self-hash mismatch")
        fresh = build_manifest(args, loaded)
        for field in ("source", "selection", "evaluation", "records"):
            if fresh[field] != manifest[field]:
                raise RuntimeError(f"resume binding mismatch: {field}")
    else:
        scratch.mkdir(exist_ok=False)
        manifest = build_manifest(args, loaded)
        core.atomic_write_json(manifest_path, manifest)
        records_path.touch(exist_ok=False)
        core.atomic_write_json(
            checkpoint_path,
            {
                "state": "frozen",
                "completed": 0,
                "remaining": n_eval,
                "n_eval": n_eval,
                "manifest_content_sha256": manifest["manifest_content_sha256"],
                "updated_at": core.utc_now(),
            },
        )
    records = core.load_jsonl(records_path)
    selected_keys = [row["asset_key"] for row in manifest["records"]]
    jobs_by_key = {
        row["asset_key"]: {
            **row,
            "samples": args.samples,
            "manifest_content_sha256": manifest["manifest_content_sha256"],
        }
        for row in manifest["records"]
    }
    by_key: dict[str, dict[str, Any]] = {}
    for record in records:
        key = record.get("asset_key")
        if key not in selected_keys or key in by_key:
            raise RuntimeError(f"invalid or duplicate resume record: {key!r}")
        _validate_record_binding(record, jobs_by_key[key])
        by_key[key] = record
    pending = [
        row for row in manifest["records"] if row["asset_key"] not in by_key
    ]
    jobs = [jobs_by_key[row["asset_key"]] for row in pending]
    with ThreadPoolExecutor(max_workers=args.workers) as executor:
        futures = {
            executor.submit(
                _execute_job, job, scratch, args.asset_timeout_seconds
            ): job
            for job in jobs
        }
        for future in as_completed(futures):
            job = futures[future]
            try:
                record = future.result()
            except Exception as exc:  # noqa: BLE001
                record = _bind_record(
                    core.failed_record(
                        job["asset_key"],
                        job["declared_joint_count_hint"],
                        f"parent worker exception: {type(exc).__name__}: {exc}",
                    ),
                    job,
                )
            by_key[record["asset_key"]] = record
            core.append_jsonl(records_path, record)
            completed = len(by_key)
            core.atomic_write_json(
                checkpoint_path,
                {
                    "state": "running" if completed < n_eval else "aggregating",
                    "completed": completed,
                    "remaining": n_eval - completed,
                    "n_eval": n_eval,
                    "last_completed_asset_key": record["asset_key"],
                    "manifest_content_sha256": manifest[
                        "manifest_content_sha256"
                    ],
                    "updated_at": core.utc_now(),
                },
            )
            print(
                f"[{completed}/{n_eval}] {record['asset_key']} {record['status']}",
                flush=True,
            )
    ordered = [by_key[key] for key in selected_keys]
    summary = core.aggregate_records(ordered, n_eval)
    summary.update(
        {
            "schema_version": 1,
            "status": "completed",
            "classification": manifest["classification"],
            "dataset": DATASET_NAME,
            "manifest_content_sha256": manifest["manifest_content_sha256"],
            "known_missing_collision_mesh_asset_count": manifest["source"][
                "known_missing_collision_mesh_asset_count"
            ],
            "known_missing_collision_mesh_reference_count": manifest["source"][
                "known_missing_collision_mesh_reference_count"
            ],
            "completed_at": core.utc_now(),
        }
    )
    core.atomic_write_json(output / "summary.json", summary)
    core.atomic_write_text(output / "summary.md", _summary_markdown(summary, manifest))
    core.atomic_write_json(
        checkpoint_path,
        {
            "state": "complete",
            "completed": n_eval,
            "remaining": 0,
            "n_eval": n_eval,
            "manifest_content_sha256": manifest["manifest_content_sha256"],
            "updated_at": core.utc_now(),
        },
    )
    return output


def validate_contract(args: argparse.Namespace) -> None:
    if args.samples < 2 or args.workers <= 0 or args.asset_timeout_seconds <= 0:
        raise ValueError("samples must be >=2 and workers/timeout must be positive")
    if args.resume and args.output is None:
        raise ValueError("resume requires an explicit output directory")
    if args.limit is not None and not 0 < args.limit <= FORMAL_N_EVAL:
        raise ValueError("limit must be in [1, 800]")
    if args.mode == "formal":
        if (
            args.limit is not None
            or args.samples != DEFAULT_SAMPLES
            or args.workers != DEFAULT_WORKERS
            or args.asset_timeout_seconds != DEFAULT_TIMEOUT_SECONDS
            or args.source_root.resolve(strict=False)
            != DEFAULT_SOURCE_ROOT.resolve(strict=False)
            or args.cohort_manifest.resolve(strict=False)
            != DEFAULT_COHORT_MANIFEST.resolve(strict=False)
        ):
            raise ValueError(
                "formal mode freezes canonical source/cohort, N=800, K=21, "
                "workers=4, timeout=120"
            )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("formal", "smoke"), default="formal")
    parser.add_argument("--source-root", type=Path, default=DEFAULT_SOURCE_ROOT)
    parser.add_argument(
        "--cohort-manifest", type=Path, default=DEFAULT_COHORT_MANIFEST
    )
    parser.add_argument(
        "--limit", type=int, help="smoke mode: evaluate exact manifest prefix"
    )
    parser.add_argument("--samples", type=int, default=DEFAULT_SAMPLES)
    parser.add_argument("--workers", type=int, default=DEFAULT_WORKERS)
    parser.add_argument(
        "--asset-timeout-seconds", type=float, default=DEFAULT_TIMEOUT_SECONDS
    )
    parser.add_argument("--output", type=Path)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--internal-job", type=Path)
    parser.add_argument("--internal-result", type=Path)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.internal_job is not None or args.internal_result is not None:
        if args.internal_job is None or args.internal_result is None:
            raise ValueError("internal job mode requires both paths")
        return run_internal_job(args.internal_job, args.internal_result)
    output = run(args)
    print(json.dumps({"status": "completed", "output": str(output)}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
