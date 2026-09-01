#!/usr/bin/env python3
"""Run the frozen URDF Sim-Ready Table 4 protocol on LAM released outputs."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from concurrent.futures import as_completed, ThreadPoolExecutor
from functools import lru_cache
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import time
import types
from typing import Any, Iterable
import xml.etree.ElementTree as ET


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
CORE_SCRIPT = REPO / "exp/scripts/run_urdf_table4_partnet_mobility.py"
DEFAULT_DATASET = REPO / "exp/Articulated-Object-Code"
DEFAULT_SOURCE_RECORDS = (
    REPO
    / "exp/runtime/urdf_table3_lam_n800_seed20260813_20260814T022400Z_v3/asset_records.jsonl"
)
DEFAULT_SOURCE_MANIFEST = DEFAULT_SOURCE_RECORDS.parent / "manifest.json"
DEFAULT_OUTPUT = REPO / "exp/runtime/urdf_table4_lam_n800_20260814"
DEFAULT_CHILD_PYTHON = REPO / "exp/.venv_low_medium/bin/python"
PROTOCOL_DOCUMENT = REPO / "exp/URDF-Sim-Ready-Automatic-Evaluation.md"
PROTOCOL_DOCUMENT_SNAPSHOT = "protocol_document_at_freeze.md"

PROTOCOL_ID = "urdf_sim_ready_table4_lam_n800_v1"
QUALIFICATION_PROTOCOL_ID = "urdf_sim_ready_table4_lam_qualification_v1"
VERIFY_PROTOCOL_ID = "urdf_sim_ready_table4_lam_verify_v1"
SAMPLE_SIZE = 800
EXPECTED_RELEASE_ASSETS = 3217
EXPECTED_SOURCE_RECORDS_SHA256 = (
    "7ef1c38d61bc780e41f62c7dd359e66f0bfeabe655c7453c93e2ea9830122d94"
)
EXPECTED_SOURCE_MANIFEST_SHA256 = (
    "7e16683bfe4e4f37d7972082d8512713c1d8d1ae4ce142b75bf7dfb0509b9951"
)
EXPECTED_SOURCE_MANIFEST_CONTENT_SHA256 = (
    "f8f7fe4da5634d4f806e793c0da919689eab25be1ce0bbed7e2232f3453d15c2"
)
EXPECTED_SELECTED_ASSET_KEYS_SHA256 = (
    "643aa5b76ac61f57dd943bee26444a3525c01201a8dff3443763a7fd8d8267d3"
)
EXPECTED_ORDERED_ASSET_IDENTITIES_SHA256 = (
    "d2e51d1443152384636d0ff24f4947069529a9cf24850d3cb9fade8ca7100251"
)
EXPECTED_PACKAGE_AUDIT_SUCCESS_ASSET_KEYS_SHA256 = (
    "d50b1281d1152d59642c054bc0f883e0980744f6e56ffb6e0282a973e75db08d"
)
EXPECTED_CANDIDATE_POOL_SHA256 = (
    "81304d094da5b8411cbb7e0fb3f911b41a934cb3733766cfda4c2f8364f780fd"
)
EXPECTED_DATASET_API_SHA256 = (
    "4ef6d64310968fab9e280ca19545ef0647e6c5e2eb81dbd681b5ca20fbbc727d"
)
EXPECTED_RELEASE_MANIFEST_SHA256 = (
    "70216593ec02b71d596e456498ff9863ad0f8e519d5d27d2cf4f58792d412412"
)
EXPECTED_DATASET_INVENTORY_SHA256 = (
    "e281119f870bb6bae9599c3edc02de0a42a257e0d433335361d4a774592c1b5a"
)
EXPECTED_UPSTREAM_REVISION = "28cec4f5be7e34fd4d586879ecfcb67f7c5e4cc0"
EXPECTED_RELEASE_TIER_COUNTS = {"broken": 385, "loads_only": 299, "viable": 2533}
EXPECTED_ARCHIVES = {
    "broken.tar.gz": {
        "bytes": 199627935,
        "sha256": "ef0f6e8506e0432febd5ccc4159c6cbf66a9c5cdadd3bdbcd521478e4c0fda3a",
    },
    "loads_only.tar.gz": {
        "bytes": 194746559,
        "sha256": "e616dc455450ca0f8ea1c76955929f340b75b9fa51b974327e160890620f9a9e",
    },
    "viable.tar.gz": {
        "bytes": 1185271461,
        "sha256": "a582ef0aa0f3073749adcc73d289a12200e500c1a5762a4ee1530eefc2c4920d",
    },
}
EXPECTED_CORE_SHA256 = (
    "e710d15cb79c50506487ff1335a88591bb58c11cf726c71198103c05f6d01ff0"
)
EXPECTED_PROTOCOL_DOCUMENT_SHA256 = (
    "c59de12cd9b51fc8556291d4a590a36115060df259ee31f0ccf3e87fccb19d86"
)
CHILD_TIMEOUT_SECONDS = 900
LAUNCHER_BINDING_PROTOCOL = "exact_venv_launcher_binding_v1"
EXPECTED_CHILD_PYTHON_SYMLINK_TARGET = "/usr/bin/python3.12"
EXPECTED_CHILD_PYTHON_RESOLVED_PATH = Path("/usr/bin/python3.12")
EXPECTED_CHILD_PYTHON_SHA256 = (
    "c0736aec631466e7bc4f5541b67358543193b8922ec3b63f6c1b247d70716591"
)
EXPECTED_CHILD_PYTHON_BYTES = 8023232
EXPECTED_PYVENV_CFG_SHA256 = (
    "a0151eba26bcc62dbba81f521d095126e9c3ab83db1471f592c4e14961b6341f"
)
EXPECTED_PYVENV_CFG_BYTES = 113
SCALE_PROTOCOL = "pybullet_q0_collision_shape_union_aabb_v1"
SINGLE_SAMPLES = 21
SOBOL_SAMPLES = 64
SOBOL_SEED = 20260813
PENETRATION_THRESHOLD_M = 1e-6
RESET_TOLERANCE = 1e-9
EXPECTED_FORMAL_AUDIT_SUMMARY = {
    "package_audit_success": 323,
    "retained_package_failures": 477,
    "collision_coverage_complete": 394,
    "movable_dof_count": 2395,
    "range_evaluable_dof_count": 2382,
    "rest_state_expected": 800,
    "single_state_expected": 50295,
    "sobol_state_expected": 49536,
    "package_audit_success_asset_keys_sha256": (
        EXPECTED_PACKAGE_AUDIT_SUCCESS_ASSET_KEYS_SHA256
    ),
}
IDENTITY_FIELDS = (
    "asset_key",
    "selection_rank",
    "selection_hash",
    "tier",
    "rel_path",
    "object_release_id",
    "package_relpath",
    "model_urdf_sha256",
    "package_content_manifest_sha256",
    "source_record_sha256",
    "source_manifest_record_sha256",
)
FROZEN_INPUT_FIELDS = (
    "protocol_id",
    "order",
    "dataset_id",
    *IDENTITY_FIELDS,
    "package_binding_sha256",
    "category",
    "package_audit_success",
    "audit_issue",
    "primary_urdf_relpath",
    "urdf_sha256",
    "valid_tree",
    "link_count",
    "collision_element_count",
    "collision_covered_link_count",
    "collision_coverage_complete",
    "movable_dof_count",
    "range_evaluable_dof_count",
    "joint_specs_sha256",
    "collision_mesh_inventory_sha256",
    "missing_collision_mesh_reference_count",
    "unsafe_collision_mesh_reference_count",
    "scale_derivation_sha256",
    "object_bbox_diagonal_m",
    "runtime_identity_sha256",
    "rest_state_expected",
    "single_state_expected",
    "sobol_state_expected",
)


class VerificationFailed(RuntimeError):
    pass


class VerificationAborted(RuntimeError):
    pass


@lru_cache(maxsize=1)
def _load_core() -> Any:
    import hashlib

    source_bytes = CORE_SCRIPT.read_bytes()
    source_sha256 = hashlib.sha256(source_bytes).hexdigest()
    if source_sha256 != EXPECTED_CORE_SHA256:
        raise RuntimeError("PartNet Table 4 core SHA256 does not match the frozen pin")
    module = types.ModuleType("urdf_table4_partnet_core")
    module.__file__ = str(CORE_SCRIPT)
    module.__cached__ = None
    module.__source_sha256__ = source_sha256
    code = compile(source_bytes, str(CORE_SCRIPT), "exec", dont_inherit=True)
    exec(code, module.__dict__)
    return module


def sha256_file(path: Path) -> str:
    import hashlib

    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_sha256(value: Any) -> str:
    import hashlib

    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _input_identity_sha256(item: dict[str, Any]) -> str:
    return canonical_sha256({key: item.get(key) for key in FROZEN_INPUT_FIELDS})


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def atomic_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def atomic_jsonl(path: Path, rows: Iterable[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        for row in rows:
            stream.write(json.dumps(row, sort_keys=True, ensure_ascii=True) + "\n")
    temporary.replace(path)


def atomic_text(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(value, encoding="utf-8")
    temporary.replace(path)


def write_protocol_document_snapshot(output: Path, manifest: dict[str, Any]) -> None:
    import hashlib

    source = manifest.get("source", {})
    expected_sha256 = source.get("protocol_document_sha256_at_freeze")
    document_bytes = PROTOCOL_DOCUMENT.read_bytes()
    observed_sha256 = hashlib.sha256(document_bytes).hexdigest()
    if (
        expected_sha256 != EXPECTED_PROTOCOL_DOCUMENT_SHA256
        or observed_sha256 != expected_sha256
    ):
        raise RuntimeError("protocol document changed before snapshot creation")
    atomic_text(
        output / PROTOCOL_DOCUMENT_SNAPSHOT,
        document_bytes.decode("utf-8"),
    )


def validate_protocol_document_snapshot(
    output: Path, manifest: dict[str, Any]
) -> None:
    source = manifest.get("source", {})
    if source.get("protocol_document_snapshot") != PROTOCOL_DOCUMENT_SNAPSHOT:
        raise RuntimeError("protocol document snapshot name mismatch")
    snapshot = output / PROTOCOL_DOCUMENT_SNAPSHOT
    if not snapshot.is_file() or snapshot.is_symlink():
        raise RuntimeError("protocol document snapshot is missing or unsafe")
    if sha256_file(snapshot) != source.get("protocol_document_sha256_at_freeze"):
        raise RuntimeError("protocol document snapshot SHA256 mismatch")


def package_file_manifest(package: Path) -> list[dict[str, Any]]:
    package = package.resolve(strict=True)
    if not package.is_dir():
        raise NotADirectoryError(package)
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
                raise RuntimeError(
                    f"package contains directory symlink: {child.relative_to(package)}"
                )
        for name in file_names:
            path = current / name
            relative = path.relative_to(package).as_posix()
            if path.is_symlink():
                raise RuntimeError(f"package contains file symlink: {relative}")
            canonical = path.resolve(strict=True)
            canonical.relative_to(package)
            if not canonical.is_file():
                raise RuntimeError(f"package entry is not a regular file: {relative}")
            rows.append(
                {
                    "path": relative,
                    "bytes": canonical.stat().st_size,
                    "sha256": sha256_file(canonical),
                }
            )
    return rows


def package_binding(package: Path) -> dict[str, Any]:
    files = package_file_manifest(package)
    return {
        "file_count": len(files),
        "total_bytes": sum(row["bytes"] for row in files),
        "files": files,
        "content_manifest_sha256": canonical_sha256(files),
    }


def manifest_self_hash(manifest: dict[str, Any]) -> str:
    payload = dict(manifest)
    payload.pop("manifest_content_sha256", None)
    return canonical_sha256(payload)


def _ordered_identities(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [{key: row[key] for key in IDENTITY_FIELDS} for row in rows]


def _safe_lam_rel_path(value: Any) -> str:
    rel_path = str(value or "").replace("\\", "/")
    path = Path(rel_path)
    if (
        not rel_path
        or path.is_absolute()
        or ".." in path.parts
        or "." in path.parts
        or len(path.parts) != 3
        or path.parts[0] not in {"objects", "imperfect"}
    ):
        raise RuntimeError(f"invalid LAM release relative path: {rel_path!r}")
    return path.as_posix()


def _frozen_source_file(
    path: Path, expected_sha256: str, *, expected_bytes: int | None = None
) -> dict[str, Any]:
    lexical = Path(os.path.abspath(str(path)))
    if lexical.is_symlink():
        raise RuntimeError(f"source provenance file is a symlink: {lexical}")
    path = lexical.resolve(strict=True)
    if not path.is_file():
        raise RuntimeError(f"source provenance file is not a regular file: {path}")
    size = path.stat().st_size
    if expected_bytes is not None and size != expected_bytes:
        raise RuntimeError(f"source provenance file size mismatch: {path}")
    observed_sha256 = sha256_file(path)
    if observed_sha256 != expected_sha256:
        raise RuntimeError(f"source provenance file SHA256 mismatch: {path}")
    return {"path": str(path), "bytes": size, "sha256": observed_sha256}


def _lam_source_provenance(dataset_root: Path) -> dict[str, Any]:
    dataset_root = dataset_root.resolve(strict=True)
    archives = {
        name: _frozen_source_file(
            dataset_root / name,
            str(expected["sha256"]),
            expected_bytes=int(expected["bytes"]),
        )
        for name, expected in sorted(EXPECTED_ARCHIVES.items())
    }
    dataset_api = _frozen_source_file(
        dataset_root / "dataset_api.json", EXPECTED_DATASET_API_SHA256
    )
    release_manifest = _frozen_source_file(
        dataset_root / "manifest.csv", EXPECTED_RELEASE_MANIFEST_SHA256
    )
    inventory = _frozen_source_file(
        REPO / "exp/dataset_inventory.json", EXPECTED_DATASET_INVENTORY_SHA256
    )
    return {
        "dataset_api": dataset_api,
        "release_manifest": release_manifest,
        "dataset_inventory": inventory,
        "archives": archives,
    }


def load_table3_cohort(
    source_records_path: Path,
    source_manifest_path: Path,
    dataset_root: Path,
    *,
    sample_size: int = SAMPLE_SIZE,
    qualification_smoke: bool = False,
) -> dict[str, Any]:
    source_records_lexical = Path(os.path.abspath(str(source_records_path)))
    source_manifest_lexical = Path(os.path.abspath(str(source_manifest_path)))
    if source_records_lexical.is_symlink() or source_manifest_lexical.is_symlink():
        raise RuntimeError("Table 3 source authority paths must not be symlinks")
    source_records_path = source_records_lexical.resolve(strict=True)
    source_manifest_path = source_manifest_lexical.resolve(strict=True)
    dataset_root = dataset_root.resolve(strict=True)
    release_root = (dataset_root / "released_outputs").resolve(strict=True)
    records_file_sha256 = sha256_file(source_records_path)
    if records_file_sha256 != EXPECTED_SOURCE_RECORDS_SHA256:
        raise RuntimeError("Table 3 asset_records.jsonl SHA256 does not match the frozen pin")
    manifest_file_sha256 = sha256_file(source_manifest_path)
    if manifest_file_sha256 != EXPECTED_SOURCE_MANIFEST_SHA256:
        raise RuntimeError("Table 3 manifest SHA256 does not match the frozen pin")
    source_manifest = read_json(source_manifest_path)
    if not isinstance(source_manifest, dict):
        raise RuntimeError("Table 3 manifest is not an object")
    if manifest_self_hash(source_manifest) != EXPECTED_SOURCE_MANIFEST_CONTENT_SHA256:
        raise RuntimeError("Table 3 manifest content SHA256 mismatch")
    if (
        source_manifest.get("manifest_content_sha256")
        != EXPECTED_SOURCE_MANIFEST_CONTENT_SHA256
    ):
        raise RuntimeError("Table 3 embedded manifest content SHA256 mismatch")
    if (
        source_manifest.get("schema_version") != 1
        or source_manifest.get("dataset")
        != "LAM released outputs (Articulated-Object-Code)"
        or source_manifest.get("classification") != "FORMAL"
    ):
        raise RuntimeError("unexpected Table 3 source manifest identity")
    manifest_rows = source_manifest.get("records")
    jsonl_rows = _read_jsonl_strict(source_records_path)
    if not isinstance(manifest_rows, list) or len(manifest_rows) != SAMPLE_SIZE:
        raise RuntimeError("Table 3 manifest does not contain exactly 800 records")
    if len(jsonl_rows) != SAMPLE_SIZE:
        raise RuntimeError("Table 3 asset_records.jsonl does not contain exactly 800 rows")
    selection = source_manifest.get("selection", {})
    source_block = source_manifest.get("source", {})
    if (
        selection.get("n_eval") != SAMPLE_SIZE
        or selection.get("seed") != SOBOL_SEED
        or selection.get("quality_label_blind") is not True
        or selection.get("algorithm")
        != "random.Random(seed).sample(sorted(asset_key), n)"
        or selection.get("selected_asset_keys_sha256")
        != EXPECTED_SELECTED_ASSET_KEYS_SHA256
        or source_block.get("n_release") != EXPECTED_RELEASE_ASSETS
        or source_block.get("tier_counts") != EXPECTED_RELEASE_TIER_COUNTS
        or source_block.get("candidate_pool_sha256")
        != EXPECTED_CANDIDATE_POOL_SHA256
        or source_block.get("upstream_revision") != EXPECTED_UPSTREAM_REVISION
        or source_block.get("dataset_api_sha256") != EXPECTED_DATASET_API_SHA256
        or source_block.get("release_manifest_sha256")
        != EXPECTED_RELEASE_MANIFEST_SHA256
    ):
        raise RuntimeError("Table 3 cohort denominator, selection, or source mismatch")
    if Path(str(source_block.get("source_root"))).resolve(strict=True) != release_root:
        raise RuntimeError("Table 3 source root does not match configured dataset")
    if Path(str(source_block.get("dataset_api"))).resolve(strict=True) != (
        dataset_root / "dataset_api.json"
    ).resolve(strict=True):
        raise RuntimeError("Table 3 dataset_api path mismatch")
    if Path(str(source_block.get("release_manifest"))).resolve(strict=True) != (
        dataset_root / "manifest.csv"
    ).resolve(strict=True):
        raise RuntimeError("Table 3 release manifest path mismatch")
    provenance = _lam_source_provenance(dataset_root)

    manifest_by_rank: dict[int, dict[str, Any]] = {}
    for raw in manifest_rows:
        if not isinstance(raw, dict):
            raise RuntimeError("Table 3 manifest record is not an object")
        rank = raw.get("selection_rank")
        if isinstance(rank, bool) or not isinstance(rank, int) or rank in manifest_by_rank:
            raise RuntimeError("Table 3 manifest selection ranks are invalid")
        manifest_by_rank[rank] = raw
    jsonl_by_rank: dict[int, dict[str, Any]] = {}
    for raw in jsonl_rows:
        rank = raw.get("selection_rank")
        if isinstance(rank, bool) or not isinstance(rank, int) or rank in jsonl_by_rank:
            raise RuntimeError("Table 3 JSONL selection ranks are invalid")
        jsonl_by_rank[rank] = raw
    expected_ranks = set(range(1, SAMPLE_SIZE + 1))
    if set(manifest_by_rank) != expected_ranks or set(jsonl_by_rank) != expected_ranks:
        raise RuntimeError("Table 3 selection ranks are not exactly 1..800")

    cross_fields = (
        "asset_key",
        "category",
        "object_release_id",
        "rel_path",
        "selection_hash",
        "selection_rank",
        "tier",
        "urdf_sha256",
    )
    identities: list[dict[str, Any]] = []
    for rank in range(1, SAMPLE_SIZE + 1):
        manifest_row = manifest_by_rank[rank]
        jsonl_row = jsonl_by_rank[rank]
        if any(manifest_row.get(key) != jsonl_row.get(key) for key in cross_fields):
            raise RuntimeError(f"Table 3 JSONL/manifest identity mismatch at rank {rank}")
        if jsonl_row.get("manifest_content_sha256") != EXPECTED_SOURCE_MANIFEST_CONTENT_SHA256:
            raise RuntimeError(f"Table 3 JSONL manifest binding mismatch at rank {rank}")
        rel_path = _safe_lam_rel_path(jsonl_row.get("rel_path"))
        tier = str(jsonl_row.get("tier", ""))
        asset_key = str(jsonl_row.get("asset_key", ""))
        object_release_id = str(jsonl_row.get("object_release_id", ""))
        category = str(jsonl_row.get("category", ""))
        if (
            tier not in EXPECTED_RELEASE_TIER_COUNTS
            or asset_key != f"{tier}:{rel_path}"
            or not object_release_id
            or object_release_id != Path(rel_path).name
            or not category
            or category != Path(rel_path).parts[1]
        ):
            raise RuntimeError(f"invalid Table 3 identity at rank {rank}")
        package = _contained_package(dataset_root, f"released_outputs/{rel_path}")
        package = package.resolve(strict=True)
        package.relative_to(release_root)
        if not package.is_dir() or package.is_symlink():
            raise RuntimeError(f"LAM package root is not a regular directory: {asset_key}")
        expected_urdf = (package / "generated.urdf").resolve(strict=True)
        if Path(str(manifest_row.get("urdf_path"))).resolve(strict=True) != expected_urdf:
            raise RuntimeError(f"Table 3 generated.urdf path mismatch: {asset_key}")
        if not expected_urdf.is_file() or expected_urdf.is_symlink():
            raise RuntimeError(f"LAM generated.urdf is not a regular file: {asset_key}")
        observed_urdf_sha256 = sha256_file(expected_urdf)
        if (
            manifest_row.get("urdf_exists") is not True
            or observed_urdf_sha256 != str(jsonl_row.get("urdf_sha256", ""))
        ):
            raise RuntimeError(f"Table 3 generated.urdf drift: {asset_key}")
        observed_binding = package_binding(package)
        identities.append(
            {
                "asset_key": asset_key,
                "selection_rank": rank,
                "selection_hash": str(jsonl_row["selection_hash"]),
                "tier": tier,
                "rel_path": rel_path,
                "object_release_id": object_release_id,
                "category": category,
                "package": str(package),
                "package_relpath": f"released_outputs/{rel_path}",
                "model_urdf_sha256": observed_urdf_sha256,
                "package_binding": observed_binding,
                "package_content_manifest_sha256": observed_binding[
                    "content_manifest_sha256"
                ],
                "source_record_sha256": canonical_sha256(jsonl_row),
                "source_manifest_record_sha256": canonical_sha256(manifest_row),
            }
        )
    if len({row["asset_key"] for row in identities}) != SAMPLE_SIZE:
        raise RuntimeError("Table 3 cohort asset keys are not unique")
    selected_asset_keys_sha256 = canonical_sha256(
        [row["asset_key"] for row in identities]
    )
    if selected_asset_keys_sha256 != EXPECTED_SELECTED_ASSET_KEYS_SHA256:
        raise RuntimeError("Table 3 selected asset-key hash mismatch")
    ordered_hash = canonical_sha256(_ordered_identities(identities))
    if ordered_hash != EXPECTED_ORDERED_ASSET_IDENTITIES_SHA256:
        raise RuntimeError("Table 3 ordered asset identity hash mismatch")
    if sample_size <= 0 or sample_size > SAMPLE_SIZE:
        raise ValueError("sample_size must be in [1, 800]")
    if not qualification_smoke and sample_size != SAMPLE_SIZE:
        raise ValueError(
            f"formal protocol requires sample_size={SAMPLE_SIZE}, got {sample_size}"
        )
    selected = identities[:sample_size]
    return {
        "protocol_id": QUALIFICATION_PROTOCOL_ID if qualification_smoke else PROTOCOL_ID,
        "qualification_smoke": qualification_smoke,
        "cohort_label": (
            f"LAM released outputs qualification N={sample_size}"
            if qualification_smoke
            else "LAM released outputs fixed Table 3 N=800 cohort"
        ),
        "source_records_path": str(source_records_path),
        "source_records_sha256": records_file_sha256,
        "source_manifest_path": str(source_manifest_path),
        "source_manifest_sha256": manifest_file_sha256,
        "source_manifest_content_sha256": EXPECTED_SOURCE_MANIFEST_CONTENT_SHA256,
        "release_asset_count": EXPECTED_RELEASE_ASSETS,
        "source_repo_id": "YipengGao/Articulated-Object-Code",
        "source_revision": EXPECTED_UPSTREAM_REVISION,
        "source_candidate_pool_sha256": EXPECTED_CANDIDATE_POOL_SHA256,
        "source_tier_counts": dict(EXPECTED_RELEASE_TIER_COUNTS),
        "source_provenance": provenance,
        "source_selection_algorithm": selection.get("algorithm"),
        "selected_asset_keys_sha256": selected_asset_keys_sha256,
        "ordered_asset_identities_sha256": ordered_hash,
        "selected": selected,
    }


def _contained_package(dataset_root: Path, package_relpath: str) -> Path:
    relative = Path(package_relpath)
    if (
        relative.is_absolute()
        or not relative.parts
        or ".." in relative.parts
        or "." in relative.parts
    ):
        raise ValueError("absolute package_relpath is not allowed")
    dataset_root = dataset_root.resolve(strict=True)
    lexical = dataset_root / relative
    current = dataset_root
    for part in relative.parts:
        current = current / part
        if current.is_symlink():
            raise ValueError(f"package path contains symlink: {current}")
    candidate = lexical.resolve(strict=False)
    try:
        candidate.relative_to(dataset_root)
    except ValueError as exc:
        raise ValueError("package_relpath escapes dataset root") from exc
    return candidate


def _valid_tree(urdf_path: Path) -> bool:
    root = ET.parse(urdf_path).getroot()
    links = [node.get("name", "").strip() for node in root.findall("link")]
    if not links or "" in links or len(links) != len(set(links)):
        return False
    link_set = set(links)
    adjacency: dict[str, list[str]] = {name: [] for name in links}
    indegree: Counter[str] = Counter()
    joints = root.findall("joint")
    for joint in joints:
        parent_node = joint.find("parent")
        child_node = joint.find("child")
        parent = parent_node.get("link", "") if parent_node is not None else ""
        child = child_node.get("link", "") if child_node is not None else ""
        if parent not in link_set or child not in link_set or parent == child:
            return False
        adjacency[parent].append(child)
        indegree[child] += 1
    roots = [name for name in links if indegree[name] == 0]
    if len(roots) != 1 or any(value > 1 for value in indegree.values()):
        return False
    active: set[str] = set()
    visited: set[str] = set()

    def visit(name: str) -> bool:
        if name in active or name in visited:
            return False
        active.add(name)
        for child in adjacency[name]:
            if not visit(child):
                return False
        active.remove(name)
        visited.add(name)
        return True

    return bool(
        len(joints) == len(links) - 1
        and visit(roots[0])
        and len(visited) == len(links)
    )


def _resolve_primary_urdf(dataset_root: Path, source_row: dict[str, Any]) -> tuple[Path, Path]:
    package = _contained_package(dataset_root, str(source_row["package_relpath"]))
    if not package.is_dir():
        raise FileNotFoundError("missing LAM package root")
    urdf_path = package / "generated.urdf"
    if urdf_path.is_symlink() or not urdf_path.is_file():
        raise FileNotFoundError("package generated.urdf is not a regular file")
    return package, urdf_path


def collision_mesh_inventory(
    dataset_root: Path, package: Path, urdf_path: Path
) -> tuple[list[dict[str, Any]], int]:
    dataset_root = dataset_root.resolve(strict=True)
    package_root = urdf_path.parent.resolve(strict=True)
    root = ET.parse(urdf_path).getroot()
    references = sorted(
        {
            mesh.get("filename", "").replace("\\", "/")
            for mesh in root.findall("link/collision/geometry/mesh")
            if mesh.get("filename")
        }
    )
    inventory: list[dict[str, Any]] = []
    unsafe = 0
    for reference in references:
        path: Path | None = None
        safe = bool(reference) and "://" not in reference and not Path(reference).is_absolute()
        if safe:
            candidate = (package_root / reference).resolve(strict=False)
            try:
                candidate.relative_to(package_root)
                candidate.relative_to(package.resolve(strict=True))
                candidate.relative_to(dataset_root)
                path = candidate
            except ValueError:
                safe = False
        if not safe:
            unsafe += 1
        exists = bool(path is not None and path.is_file())
        inventory.append(
            {
                "path": reference,
                "safe": safe,
                "resolved_relpath": (
                    path.relative_to(dataset_root).as_posix() if path is not None else None
                ),
                "exists": exists,
                "is_symlink": bool(path is not None and path.is_symlink()),
                "size_bytes": path.stat().st_size if exists and path is not None else None,
                "sha256": sha256_file(path) if exists and path is not None else None,
            }
        )
    return inventory, unsafe


def derive_collision_aabb(urdf_path: Path) -> dict[str, Any]:
    import pybullet as bullet

    client = bullet.connect(bullet.DIRECT)
    body: int | None = None
    try:
        flags = int(
            bullet.URDF_USE_INERTIA_FROM_FILE
            | bullet.URDF_IGNORE_VISUAL_SHAPES
        )
        body = bullet.loadURDF(
            str(urdf_path),
            useFixedBase=True,
            flags=flags,
            physicsClientId=client,
        )
        for joint_index in range(bullet.getNumJoints(body, physicsClientId=client)):
            bullet.resetJointState(
                body,
                joint_index,
                0.0,
                targetVelocity=0.0,
                physicsClientId=client,
            )
        collision_links = []
        bounds = []
        for link_index in range(-1, bullet.getNumJoints(body, physicsClientId=client)):
            shapes = bullet.getCollisionShapeData(
                body, link_index, physicsClientId=client
            )
            if not shapes:
                continue
            lower, upper = bullet.getAABB(body, link_index, physicsClientId=client)
            values = [*lower, *upper]
            if not all(math.isfinite(float(value)) for value in values):
                raise ValueError("collision AABB contains non-finite values")
            collision_links.append(link_index)
            bounds.append((tuple(map(float, lower)), tuple(map(float, upper))))
        if not bounds:
            raise ValueError("loaded URDF has no collision shapes")
        minimum = [min(row[0][axis] for row in bounds) for axis in range(3)]
        maximum = [max(row[1][axis] for row in bounds) for axis in range(3)]
        diagonal = math.sqrt(
            sum((high - low) ** 2 for low, high in zip(minimum, maximum))
        )
        if not math.isfinite(diagonal) or diagonal <= 0.0:
            raise ValueError("collision AABB diagonal is not positive finite")
        return {
            "protocol": SCALE_PROTOCOL,
            "status": "PASS",
            "joint_state": "q=0 for every simulator joint",
            "minimum_m": minimum,
            "maximum_m": maximum,
            "diagonal_m": diagonal,
            "collision_link_indices": collision_links,
            "load_flags": flags,
        }
    finally:
        if body is not None:
            bullet.removeBody(body, physicsClientId=client)
        bullet.disconnect(client)


def _empty_audit() -> dict[str, Any]:
    return {
        "package_audit_success": False,
        "audit_issue": None,
        "primary_urdf_relpath": None,
        "urdf_sha256": None,
        "valid_tree": False,
        "link_count": 0,
        "collision_element_count": 0,
        "collision_covered_link_count": 0,
        "collision_coverage_complete": False,
        "movable_dof_count": 0,
        "range_evaluable_dof_count": 0,
        "joint_specs": [],
        "joint_specs_sha256": canonical_sha256([]),
        "collision_mesh_files": [],
        "collision_mesh_inventory_sha256": canonical_sha256([]),
        "missing_collision_mesh_reference_count": 0,
        "unsafe_collision_mesh_reference_count": 0,
        "scale_derivation": {"protocol": SCALE_PROTOCOL, "status": "N/E"},
        "scale_derivation_sha256": canonical_sha256(
            {"protocol": SCALE_PROTOCOL, "status": "N/E"}
        ),
        "object_bbox_diagonal_m": None,
    }


def audit_asset(dataset_root: Path, source_row: dict[str, Any]) -> dict[str, Any]:
    result = _empty_audit()
    try:
        package, urdf_path = _resolve_primary_urdf(dataset_root, source_row)
        observed_binding = package_binding(package)
        if observed_binding != source_row.get("package_binding"):
            raise RuntimeError("package binding drift")
        result["primary_urdf_relpath"] = urdf_path.relative_to(
            dataset_root.resolve(strict=True)
        ).as_posix()
        result["urdf_sha256"] = sha256_file(urdf_path)
        if result["urdf_sha256"] != source_row.get("model_urdf_sha256"):
            raise RuntimeError("generated.urdf SHA256 drift")
        core = _load_core()
        joints = core.parse_urdf_joints(urdf_path)
        result["joint_specs"] = joints
        result["joint_specs_sha256"] = canonical_sha256(joints)
        result["movable_dof_count"] = len(joints)
        result["range_evaluable_dof_count"] = sum(
            bool(row["range_evaluable"]) for row in joints
        )
        result["valid_tree"] = _valid_tree(urdf_path)
        xml_root = ET.parse(urdf_path).getroot()
        links = xml_root.findall("link")
        covered_links = [
            link
            for link in links
            if any(collision.find("geometry") is not None for collision in link.findall("collision"))
        ]
        result["link_count"] = len(links)
        result["collision_element_count"] = sum(
            collision.find("geometry") is not None
            for link in links
            for collision in link.findall("collision")
        )
        result["collision_covered_link_count"] = len(covered_links)
        result["collision_coverage_complete"] = bool(
            links and len(covered_links) == len(links)
        )
        inventory, unsafe = collision_mesh_inventory(
            dataset_root, package, urdf_path
        )
        result["collision_mesh_files"] = inventory
        result["collision_mesh_inventory_sha256"] = canonical_sha256(inventory)
        result["unsafe_collision_mesh_reference_count"] = unsafe
        result["missing_collision_mesh_reference_count"] = sum(
            not row["exists"] for row in inventory
        )
        if unsafe:
            raise ValueError(f"unsafe collision mesh reference count: {unsafe}")
        if result["missing_collision_mesh_reference_count"]:
            raise FileNotFoundError(
                "missing collision mesh reference count: "
                f"{result['missing_collision_mesh_reference_count']}"
            )
        if not result["valid_tree"]:
            raise ValueError("URDF joint graph is not a valid rooted tree")
        if not result["collision_coverage_complete"]:
            raise ValueError(
                "collision coverage incomplete: "
                f"{result['collision_covered_link_count']}/{result['link_count']}"
            )
        scale = derive_collision_aabb(urdf_path)
        result["scale_derivation"] = scale
        result["scale_derivation_sha256"] = canonical_sha256(scale)
        result["object_bbox_diagonal_m"] = scale["diagonal_m"]
        result["package_audit_success"] = True
    except Exception as exc:  # noqa: BLE001
        result["audit_issue"] = f"{type(exc).__name__}: {exc}"
    return result


def current_runtime_identity() -> dict[str, Any]:
    core = _load_core()
    return {
        **core.current_runtime_identity(),
        "adapter_runner": str(SCRIPT),
        "adapter_runner_sha256": sha256_file(SCRIPT),
        "collision_core": str(CORE_SCRIPT),
        "collision_core_sha256": sha256_file(CORE_SCRIPT),
    }


def failure_record(
    item: dict[str, Any], issue: str, *, timed_out: bool = False
) -> dict[str, Any]:
    core = _load_core()
    result = core.failure_record(item, issue, timed_out=timed_out)
    _add_source_identity(result, item)
    result["runner_sha256"] = sha256_file(SCRIPT)
    result["collision_core_sha256"] = sha256_file(CORE_SCRIPT)
    return result


def result_matches_item(
    result: dict[str, Any],
    item: dict[str, Any],
    runner_hash: str,
    state_records: list[dict[str, Any]] | None = None,
) -> bool:
    if not isinstance(result, dict) or not isinstance(item, dict):
        return False
    core = _load_core()
    frozen_fields = (
        "protocol_id",
        "order",
        "dataset_id",
        "category",
        "input_identity_sha256",
        "movable_dof_count",
        "range_evaluable_dof_count",
        "rest_state_expected",
        "single_state_expected",
        "sobol_state_expected",
        "object_bbox_diagonal_m",
        "runtime_identity",
        *IDENTITY_FIELDS,
    )
    matches = bool(
        all(result.get(key) == item.get(key) for key in frozen_fields)
        and result.get("runner_sha256") == runner_hash
        and result.get("collision_core_sha256") == EXPECTED_CORE_SHA256
        and core._result_counters_valid(result, state_records, item)
    )
    if not matches:
        return False
    try:
        validate_state_closure(result, state_records, item)
    except (KeyError, RuntimeError, TypeError, ValueError):
        return False
    return True


def validate_state_closure(
    record: dict[str, Any],
    state_records: list[dict[str, Any]] | None = None,
    item: dict[str, Any] | None = None,
) -> None:
    expected = record if item is None else item
    if not isinstance(record, dict) or not isinstance(expected, dict):
        raise RuntimeError("state closure record or item is not an object")
    states = record.get("state_records", []) if state_records is None else state_records
    if not isinstance(states, list) or not all(
        isinstance(state, dict) for state in states
    ):
        raise RuntimeError("state records are not a list of objects")
    if not states and any(
        record.get(field) is not None
        for field in (
            "max_penetration_m",
            "max_penetration_normalized",
            "max_reset_readback_error",
        )
    ):
        raise RuntimeError("empty state evidence has non-empty maximum metrics")
    for state in states:
        if any(
            state.get(key) != expected.get(key)
            for key in (*IDENTITY_FIELDS, "protocol_id", "order", "input_identity_sha256")
        ):
            raise RuntimeError("state source identity mismatch")
        observations: dict[str, tuple[int, int, float]] = {}
        for prefix in ("all_pair", "non_adjacent"):
            contact_raw = state.get(f"{prefix}_contact_count")
            illegal_raw = state.get(f"{prefix}_illegal_penetration_count")
            maximum_raw = state.get(f"{prefix}_max_penetration_m")
            if (
                isinstance(contact_raw, bool)
                or not isinstance(contact_raw, int)
                or isinstance(illegal_raw, bool)
                or not isinstance(illegal_raw, int)
                or isinstance(maximum_raw, bool)
                or not isinstance(maximum_raw, (int, float))
            ):
                raise RuntimeError("collision observation has invalid numeric types")
            maximum = float(maximum_raw)
            if (
                contact_raw < 0
                or illegal_raw < 0
                or illegal_raw > contact_raw
                or not math.isfinite(maximum)
                or maximum < 0.0
            ):
                raise RuntimeError("collision observation counters are invalid")
            if (illegal_raw > 0) != (maximum > PENETRATION_THRESHOLD_M):
                raise RuntimeError("collision penetration threshold is inconsistent")
            if maximum > 0.0 and contact_raw == 0:
                raise RuntimeError("penetration depth exists without a contact")
            observations[prefix] = (contact_raw, illegal_raw, maximum)
        all_pair = observations["all_pair"]
        non_adjacent = observations["non_adjacent"]
        if (
            non_adjacent[0] > all_pair[0]
            or non_adjacent[1] > all_pair[1]
            or non_adjacent[2] > all_pair[2] + 1e-15
        ):
            raise RuntimeError("non-adjacent observations are not an all-pair subset")
        readback_raw = state.get("reset_readback_max_abs_error")
        if (
            isinstance(readback_raw, bool)
            or not isinstance(readback_raw, (int, float))
            or not math.isfinite(float(readback_raw))
            or float(readback_raw) < 0.0
            or float(readback_raw) > RESET_TOLERANCE
        ):
            raise RuntimeError("state reset readback exceeds the frozen tolerance")
    _load_core().validate_state_closure(record, states, item)


def summarize_records(
    manifest: dict[str, Any], records: list[dict[str, Any]]
) -> dict[str, Any]:
    core = _load_core()
    summary = core.summarize_records(manifest, records)
    completed_max_values = [
        float(record["max_penetration_normalized"])
        for record in records
        if bool(record.get("measurement_complete"))
        and record.get("max_penetration_normalized") is not None
    ]
    summary["metrics"]["max_penetration"].update(
        {
            "maximum_observed_normalized": (
                max(completed_max_values) if completed_max_values else None
            ),
            "observed_assets": len(completed_max_values),
            "fully_measured_assets": len(completed_max_values),
            "denominator": len(records),
            "status": (
                "COMPLETE"
                if len(completed_max_values) == len(records)
                else "PARTIAL"
            ),
        }
    )

    by_category: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for record in records:
        by_category[str(record["category"])].append(record)
    if len(by_category) == 1 and not summary.get("category_results"):
        category, category_records = next(iter(by_category.items()))
        summary["category_results"] = {
            category: core.summarize_records(
                {"sample_size": len(category_records)}, category_records
            )["metrics"]
        }
        summary["category_macro"] = {
            key: summary["category_results"][category][key]["rate"]
            for key in (
                "rest_all_pair_cf",
                "rest_non_adjacent_cf",
                "single_joint_sweep_cf",
                "multi_joint_sobol_cf",
                "collision_free_range",
                "strict_collision_pass",
            )
        }
    for category, category_records in by_category.items():
        category_metrics = summary.get("category_results", {}).get(category)
        if not isinstance(category_metrics, dict):
            raise RuntimeError(f"missing authoritative category result: {category}")
        category_completed = [
            float(record["max_penetration_normalized"])
            for record in category_records
            if bool(record.get("measurement_complete"))
            and record.get("max_penetration_normalized") is not None
        ]
        category_metrics["max_penetration"].update(
            {
                "maximum_observed_normalized": (
                    max(category_completed) if category_completed else None
                ),
                "observed_assets": len(category_completed),
                "fully_measured_assets": len(category_completed),
                "denominator": len(category_records),
                "status": (
                    "COMPLETE"
                    if len(category_completed) == len(category_records)
                    else "PARTIAL"
                ),
                "normalization": "PyBullet q=0 collision-shape union AABB diagonal",
            }
        )
    summary["metrics"]["max_penetration"]["normalization"] = (
        "PyBullet q=0 collision-shape union AABB diagonal"
    )
    collision_state_rates = [
        category_metrics["collision_state_rate"]["rate"]
        for category_metrics in summary.get("category_results", {}).values()
        if category_metrics["collision_state_rate"]["rate"] is not None
    ]
    summary["category_macro"]["collision_state_rate"] = (
        sum(collision_state_rates) / len(collision_state_rates)
        if collision_state_rates
        else None
    )
    return summary


def _audit_summary(items: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "package_audit_success": sum(
            bool(item["package_audit_success"]) for item in items
        ),
        "retained_package_failures": sum(
            not bool(item["package_audit_success"]) for item in items
        ),
        "collision_coverage_complete": sum(
            bool(item["collision_coverage_complete"]) for item in items
        ),
        "movable_dof_count": sum(int(item["movable_dof_count"]) for item in items),
        "range_evaluable_dof_count": sum(
            int(item["range_evaluable_dof_count"]) for item in items
        ),
        "rest_state_expected": sum(int(item["rest_state_expected"]) for item in items),
        "single_state_expected": sum(
            int(item["single_state_expected"]) for item in items
        ),
        "sobol_state_expected": sum(
            int(item["sobol_state_expected"]) for item in items
        ),
        "package_audit_success_asset_keys_sha256": canonical_sha256(
            [
                item["asset_key"]
                for item in items
                if bool(item["package_audit_success"])
            ]
        ),
    }


def _cohort_boundary(qualification_smoke: bool) -> dict[str, Any]:
    return {
        "is_full_release_cohort": False,
        "is_shared_category_balanced_cohort": False,
        "authoritative_category_labels_available": True,
        "paper_table_role": (
            "qualification only"
            if qualification_smoke
            else "exact frozen LAM Table 3 N=800 sample"
        ),
    }


def _selection_policy() -> dict[str, Any]:
    return {
        "algorithm": (
            "asset_records.jsonl membership joined to the frozen Table 3 manifest "
            "and ordered by selection_rank; qualification uses an exact prefix"
        ),
        "identity_fields_used": list(IDENTITY_FIELDS),
        "outcome_based_filtering": False,
        "selected_failures_retained_without_replacement": True,
    }


def _sampling_policy() -> dict[str, Any]:
    return {
        "rest_state": "native URDF/PyBullet q=0",
        "single_joint_states_per_declared_nonfixed_joint": SINGLE_SAMPLES,
        "single_joint_other_joint_state": 0.0,
        "continuous_joint_interval": [-math.pi, math.pi],
        "sobol_states_per_asset_with_nonfixed_joint": SOBOL_SAMPLES,
        "sobol_scramble": True,
        "sobol_seed": SOBOL_SEED,
    }


def _collision_policy() -> dict[str, Any]:
    return {
        "penetration_threshold_m": PENETRATION_THRESHOLD_M,
        "surface_contact_allowed": True,
        "rest_panels": ["all_pair", "exclude_direct_parent_child"],
        "sweep_sobol_strict_policy": "exclude_direct_parent_child",
        "complete_collision_coverage_required": True,
        "visual_geometry_fallback": False,
        "unexecuted_states_fail_closed": True,
        "continuous_collision_detection": "not_run",
        "aor": "N/E: no stable exact overlap-volume implementation",
        "object_scale": "PyBullet q=0 collision-shape union AABB diagonal",
        "object_scale_protocol": SCALE_PROTOCOL,
    }


def _execution_policy() -> dict[str, Any]:
    return {
        "child_timeout_seconds": CHILD_TIMEOUT_SECONDS,
        "timeout_scope": "per_asset_subprocess",
        "timeout_failure": "retained_fail_closed",
    }


def _validate_manifest_semantics(
    manifest: dict[str, Any], *, qualification_smoke: bool
) -> None:
    if manifest.get("status") != "FROZEN":
        raise RuntimeError("manifest status mismatch")
    if manifest.get("cohort_boundary") != _cohort_boundary(qualification_smoke):
        raise RuntimeError("manifest cohort boundary mismatch")
    if manifest.get("selection_policy") != _selection_policy():
        raise RuntimeError("manifest selection policy mismatch")
    if manifest.get("sampling") != _sampling_policy():
        raise RuntimeError("manifest sampling contract mismatch")
    if manifest.get("collision_policy") != _collision_policy():
        raise RuntimeError("manifest collision policy mismatch")
    if manifest.get("execution_policy") != _execution_policy():
        raise RuntimeError("manifest execution policy mismatch")


def _validate_formal_audit_summary(
    summary: dict[str, Any], *, qualification_smoke: bool
) -> None:
    if not qualification_smoke and summary != EXPECTED_FORMAL_AUDIT_SUMMARY:
        raise RuntimeError(
            "formal static audit invariant mismatch: "
            f"expected {EXPECTED_FORMAL_AUDIT_SUMMARY}, observed {summary}"
        )


def build_manifest(
    dataset_root: Path,
    source_records: Path,
    table3_manifest: Path,
    *,
    sample_size: int = SAMPLE_SIZE,
    qualification_smoke: bool = False,
    child_runtime: dict[str, Any] | None = None,
    child_launcher: dict[str, Any] | None = None,
    workers: int = 1,
) -> dict[str, Any]:
    if workers <= 0:
        raise ValueError("workers must be positive")
    dataset_root = dataset_root.resolve(strict=True)
    source_records = source_records.resolve(strict=True)
    table3_manifest = table3_manifest.resolve(strict=True)
    protocol_document_sha256 = sha256_file(PROTOCOL_DOCUMENT)
    if protocol_document_sha256 != EXPECTED_PROTOCOL_DOCUMENT_SHA256:
        raise RuntimeError("Table 4 protocol document changed before cohort freeze")
    contract = load_table3_cohort(
        source_records,
        table3_manifest,
        dataset_root,
        sample_size=sample_size,
        qualification_smoke=qualification_smoke,
    )
    frozen_runtime = child_runtime or current_runtime_identity()
    frozen_launcher = child_launcher or frozen_launcher_binding(
        DEFAULT_CHILD_PYTHON, REPO
    )
    validate_frozen_launcher_binding(frozen_launcher)
    _load_core().require_runtime_match(current_runtime_identity(), frozen_runtime)
    selected = contract["selected"]
    if workers == 1:
        audits = [audit_asset(dataset_root, row) for row in selected]
    else:
        with ThreadPoolExecutor(max_workers=workers) as executor:
            audits = list(executor.map(lambda row: audit_asset(dataset_root, row), selected))
    items = [
        freeze_item(
            source_row,
            audit,
            order=order,
            protocol_id=contract["protocol_id"],
            runtime_identity=frozen_runtime,
        )
        for order, (source_row, audit) in enumerate(zip(selected, audits))
    ]
    # A second, ordered pass prevents a transient audit failure from being frozen
    # as a dataset failure during concurrent preparation.
    for item in items:
        validate_frozen_source_snapshot(item, dataset_root, rederive_scale=True)
    manifest = {
        "protocol_id": contract["protocol_id"],
        "status": "FROZEN",
        "created_at_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "cohort_label": contract["cohort_label"],
        "qualification_smoke": qualification_smoke,
        "cohort_boundary": _cohort_boundary(qualification_smoke),
        "dataset_root": str(dataset_root),
        "release_asset_count": contract["release_asset_count"],
        "sample_size": sample_size,
        "source": {
            "table3_asset_records_path": str(source_records),
            "table3_asset_records_sha256": contract["source_records_sha256"],
            "table3_manifest_path": str(table3_manifest),
            "table3_manifest_sha256": contract["source_manifest_sha256"],
            "table3_manifest_content_sha256": contract[
                "source_manifest_content_sha256"
            ],
            "repo_id": contract["source_repo_id"],
            "revision": contract["source_revision"],
            "candidate_pool_sha256": contract["source_candidate_pool_sha256"],
            "tier_counts": contract["source_tier_counts"],
            "provenance": contract["source_provenance"],
            "selection_algorithm": contract["source_selection_algorithm"],
            "selected_asset_keys_sha256": contract[
                "selected_asset_keys_sha256"
            ],
            "ordered_asset_identities_sha256": contract[
                "ordered_asset_identities_sha256"
            ],
            "protocol_document_path": str(PROTOCOL_DOCUMENT),
            "protocol_document_snapshot": PROTOCOL_DOCUMENT_SNAPSHOT,
            "protocol_document_sha256_at_freeze": protocol_document_sha256,
        },
        "selection_policy": _selection_policy(),
        "sampling": _sampling_policy(),
        "collision_policy": _collision_policy(),
        "execution_policy": _execution_policy(),
        "runtime": {
            "runner_sha256": sha256_file(SCRIPT),
            "collision_core_sha256": sha256_file(CORE_SCRIPT),
            "manifest_builder": current_runtime_identity(),
            "child": frozen_runtime,
            "child_launcher": frozen_launcher,
        },
        "items": items,
    }
    manifest["ordered_selected_asset_keys_sha256"] = canonical_sha256(
        [item["asset_key"] for item in items]
    )
    manifest["items_sha256"] = canonical_sha256(items)
    manifest["audit_summary"] = _audit_summary(items)
    _validate_formal_audit_summary(
        manifest["audit_summary"], qualification_smoke=qualification_smoke
    )
    manifest["manifest_content_sha256"] = manifest_self_hash(manifest)
    return manifest


def validate_manifest(
    manifest: dict[str, Any],
    dataset_root: Path,
    source_records: Path,
    table3_manifest: Path,
    *,
    qualification_smoke: bool,
    child_runtime: dict[str, Any] | None = None,
    child_launcher: dict[str, Any] | None = None,
) -> None:
    dataset_root = dataset_root.resolve(strict=True)
    source_records = source_records.resolve(strict=True)
    table3_manifest = table3_manifest.resolve(strict=True)
    sample_size = int(manifest.get("sample_size", -1))
    contract = load_table3_cohort(
        source_records,
        table3_manifest,
        dataset_root,
        sample_size=sample_size,
        qualification_smoke=qualification_smoke,
    )
    if manifest.get("protocol_id") != contract["protocol_id"]:
        raise RuntimeError("manifest protocol mismatch")
    if manifest.get("qualification_smoke") is not qualification_smoke:
        raise RuntimeError("manifest qualification mode mismatch")
    if Path(str(manifest.get("dataset_root"))).resolve() != dataset_root:
        raise RuntimeError("manifest dataset root mismatch")
    if manifest.get("release_asset_count") != EXPECTED_RELEASE_ASSETS:
        raise RuntimeError("manifest release denominator mismatch")
    if manifest.get("cohort_label") != contract["cohort_label"]:
        raise RuntimeError("manifest cohort label mismatch")
    _validate_manifest_semantics(
        manifest, qualification_smoke=qualification_smoke
    )
    source = manifest.get("source", {})
    expected_source = {
        "table3_asset_records_path": str(source_records),
        "table3_asset_records_sha256": contract["source_records_sha256"],
        "table3_manifest_path": str(table3_manifest),
        "table3_manifest_sha256": contract["source_manifest_sha256"],
        "table3_manifest_content_sha256": contract[
            "source_manifest_content_sha256"
        ],
        "repo_id": contract["source_repo_id"],
        "revision": contract["source_revision"],
        "candidate_pool_sha256": contract["source_candidate_pool_sha256"],
        "tier_counts": contract["source_tier_counts"],
        "provenance": contract["source_provenance"],
        "selection_algorithm": contract["source_selection_algorithm"],
        "selected_asset_keys_sha256": contract["selected_asset_keys_sha256"],
        "ordered_asset_identities_sha256": contract[
            "ordered_asset_identities_sha256"
        ],
        "protocol_document_path": str(PROTOCOL_DOCUMENT),
        "protocol_document_snapshot": PROTOCOL_DOCUMENT_SNAPSHOT,
        "protocol_document_sha256_at_freeze": EXPECTED_PROTOCOL_DOCUMENT_SHA256,
    }
    if source != expected_source:
        raise RuntimeError("manifest Table 3 source closure mismatch")
    items = manifest.get("items", [])
    if not isinstance(items, list) or len(items) != sample_size:
        raise RuntimeError("manifest item count mismatch")
    selected = contract["selected"]
    runtime = manifest.get("runtime", {})
    for order, (item, source_row) in enumerate(zip(items, selected)):
        if any(item.get(key) != source_row.get(key) for key in IDENTITY_FIELDS):
            raise RuntimeError(f"manifest source identity mismatch at order {order}")
        if (
            item.get("order") != order
            or item.get("dataset_id") != f"lam_{order:04d}"
            or item.get("category") != source_row["category"]
            or item.get("protocol_id") != contract["protocol_id"]
        ):
            raise RuntimeError(f"manifest item identity mismatch at order {order}")
        if (
            item.get("package_binding") != source_row["package_binding"]
            or item.get("package_binding_sha256")
            != canonical_sha256(source_row["package_binding"])
        ):
            raise RuntimeError(f"manifest package binding mismatch at order {order}")
        if item.get("input_identity_sha256") != _input_identity_sha256(item):
            raise RuntimeError(f"manifest frozen input hash mismatch at order {order}")
        if canonical_sha256(item.get("joint_specs", [])) != item.get(
            "joint_specs_sha256"
        ):
            raise RuntimeError(
                f"manifest joint specification hash mismatch at order {order}"
            )
        meshes = item.get("collision_mesh_files", [])
        if canonical_sha256(meshes) != item.get("collision_mesh_inventory_sha256"):
            raise RuntimeError(f"manifest mesh inventory hash mismatch at order {order}")
        scale = item.get("scale_derivation", {})
        if item.get("scale_derivation_sha256") != canonical_sha256(scale):
            raise RuntimeError(f"manifest scale derivation hash mismatch at order {order}")
        if bool(item.get("package_audit_success")) and not bool(
            item.get("collision_coverage_complete")
        ):
            raise RuntimeError(f"successful incomplete collision audit at order {order}")
        if item.get("runtime_identity") != runtime.get("child"):
            raise RuntimeError(f"manifest item runtime mismatch at order {order}")
        if item.get("runtime_identity_sha256") != canonical_sha256(
            item.get("runtime_identity")
        ):
            raise RuntimeError(f"manifest item runtime hash mismatch at order {order}")
        movable = int(item.get("movable_dof_count", -1))
        if (
            item.get("rest_state_expected") != 1
            or item.get("single_state_expected") != SINGLE_SAMPLES * movable
            or item.get("sobol_state_expected")
            != (SOBOL_SAMPLES if movable > 0 else 0)
        ):
            raise RuntimeError(f"manifest frozen state denominator mismatch at order {order}")
        validate_frozen_source_snapshot(item, dataset_root, rederive_scale=True)
    if canonical_sha256(items) != manifest.get("items_sha256"):
        raise RuntimeError("manifest items hash mismatch")
    if manifest.get("audit_summary") != _audit_summary(items):
        raise RuntimeError("manifest audit summary mismatch")
    _validate_formal_audit_summary(
        manifest["audit_summary"], qualification_smoke=qualification_smoke
    )
    expected_order_hash = canonical_sha256([row["asset_key"] for row in selected])
    if manifest.get("ordered_selected_asset_keys_sha256") != expected_order_hash:
        raise RuntimeError("manifest selected asset order mismatch")
    if manifest.get("manifest_content_sha256") != manifest_self_hash(manifest):
        raise RuntimeError("manifest content self-hash mismatch")
    current = current_runtime_identity()
    if runtime.get("runner_sha256") != sha256_file(SCRIPT):
        raise RuntimeError("LAM runner changed after cohort freeze")
    if runtime.get("collision_core_sha256") != EXPECTED_CORE_SHA256:
        raise RuntimeError("collision core hash mismatch")
    if runtime.get("manifest_builder") != current:
        raise RuntimeError("manifest builder runtime mismatch")
    frozen_launcher = runtime.get("child_launcher")
    validate_frozen_launcher_binding(frozen_launcher)
    if child_launcher is not None and child_launcher != frozen_launcher:
        raise RuntimeError("requested child launcher does not match frozen manifest")
    observed_runtime = child_runtime or current
    _load_core().require_runtime_match(runtime.get("child", {}), observed_runtime)


def freeze_item(
    source_row: dict[str, Any],
    audit: dict[str, Any],
    *,
    order: int,
    protocol_id: str,
    runtime_identity: dict[str, Any],
) -> dict[str, Any]:
    identity = {key: source_row[key] for key in IDENTITY_FIELDS}
    movable = int(audit.get("movable_dof_count", 0))
    item = {
        "protocol_id": protocol_id,
        "order": order,
        "dataset_id": f"lam_{order:04d}",
        **identity,
        "package_binding": source_row["package_binding"],
        "package_binding_sha256": canonical_sha256(source_row["package_binding"]),
        "category": source_row["category"],
        **audit,
        "runtime_identity": runtime_identity,
        "runtime_identity_sha256": canonical_sha256(runtime_identity),
        "rest_state_expected": 1,
        "single_state_expected": 21 * movable,
        "sobol_state_expected": 64 if movable > 0 else 0,
    }
    item["input_identity_sha256"] = _input_identity_sha256(item)
    return item


def _add_source_identity(result: dict[str, Any], item: dict[str, Any]) -> None:
    for key in IDENTITY_FIELDS:
        result[key] = item.get(key)
    result["input_identity_sha256"] = item.get("input_identity_sha256")


def evaluate_asset(item: dict[str, Any], dataset_root: Path) -> dict[str, Any]:
    import pybullet as bullet

    core = _load_core()
    result = core.failure_record(item, "evaluation_not_completed")
    _add_source_identity(result, item)
    result["issues"] = []
    result["state_records"] = []
    result["runner_sha256"] = sha256_file(SCRIPT)
    result["collision_core_sha256"] = sha256_file(CORE_SCRIPT)
    result["runtime_identity"] = current_runtime_identity()
    try:
        validate_frozen_source_snapshot(item, dataset_root)
    except Exception as exc:  # noqa: BLE001
        result["issues"] = [f"{type(exc).__name__}: {exc}"]
        return result
    if not item.get("package_audit_success"):
        result["issues"] = [str(item.get("audit_issue") or "package_audit_failed")]
        return result
    urdf_path = dataset_root.resolve(strict=True) / str(item["primary_urdf_relpath"])
    joints = core.parse_urdf_joints(urdf_path)
    result["movable_dof_count"] = len(joints)
    result["range_evaluable_dof_count"] = sum(
        bool(row["range_evaluable"]) for row in joints
    )
    client = bullet.connect(bullet.DIRECT)
    body: int | None = None
    try:
        flags = int(
            bullet.URDF_USE_INERTIA_FROM_FILE
            | bullet.URDF_USE_SELF_COLLISION
            | bullet.URDF_USE_SELF_COLLISION_INCLUDE_PARENT
            | bullet.URDF_IGNORE_VISUAL_SHAPES
        )
        body = bullet.loadURDF(
            str(urdf_path),
            useFixedBase=True,
            flags=flags,
            physicsClientId=client,
        )
        result["load_success"] = True
        simulator_by_name: dict[str, int] = {}
        for index in range(bullet.getNumJoints(body, physicsClientId=client)):
            info = bullet.getJointInfo(body, index, physicsClientId=client)
            name = info[1].decode("utf-8") if isinstance(info[1], bytes) else str(info[1])
            simulator_by_name[name] = index
        missing_names = [row["name"] for row in joints if row["name"] not in simulator_by_name]
        if missing_names:
            raise RuntimeError(f"simulator joint mapping missing: {missing_names}")
        joint_indices = [simulator_by_name[row["name"]] for row in joints]
        for index in joint_indices:
            bullet.setJointMotorControl2(
                body,
                index,
                controlMode=bullet.VELOCITY_CONTROL,
                targetVelocity=0.0,
                force=0.0,
                physicsClientId=client,
            )
        direct_pairs = core._direct_parent_pairs(bullet, body, client)
        rest_values = [0.0] * len(joints)

        def observe(
            values: list[float],
            phase: str,
            sample_index: int,
            joint_name: str | None = None,
        ) -> dict[str, Any]:
            collision, readback_error = core._reset_and_observe(
                bullet,
                body,
                client,
                joint_indices,
                values,
                direct_pairs,
            )
            metric_key = (
                "all_pair_max_penetration_m"
                if phase == "rest"
                else "non_adjacent_max_penetration_m"
            )
            state = {
                "dataset_id": item["dataset_id"],
                **{key: item[key] for key in IDENTITY_FIELDS},
                "category": item["category"],
                "protocol_id": item["protocol_id"],
                "order": item["order"],
                "input_identity_sha256": item["input_identity_sha256"],
                "phase": phase,
                "sample_index": sample_index,
                "joint_name": joint_name,
                "joint_values_sha256": canonical_sha256(values),
                "reset_readback_max_abs_error": readback_error,
                "metric_max_penetration_m": float(collision[metric_key]),
                **collision,
            }
            result["state_records"].append(state)
            return state

        rest = observe(rest_values, "rest", 0)
        result["rest_state_executed"] = 1
        result["rest_all_pair_cf"] = rest["all_pair_illegal_penetration_count"] == 0
        result["rest_non_adjacent_cf"] = (
            rest["non_adjacent_illegal_penetration_count"] == 0
        )
        result["rest_non_adjacent_free"] = int(result["rest_non_adjacent_cf"])

        joint_sweep_passes = 0
        for joint_position, row in enumerate(joints):
            if not row["range_evaluable"]:
                result["issues"].append(
                    f"joint_range_not_evaluable:{row['name']}"
                )
                continue
            joint_free = True
            for sample_index, value in enumerate(core.single_joint_values(row)):
                values = list(rest_values)
                values[joint_position] = value
                state = observe(
                    values,
                    "single_joint_sweep",
                    sample_index,
                    str(row["name"]),
                )
                result["single_state_executed"] += 1
                free = state["non_adjacent_illegal_penetration_count"] == 0
                result["single_non_adjacent_free"] += int(free)
                joint_free = joint_free and free
            joint_sweep_passes += int(joint_free)
        result["joint_single_sweep_cf_passed"] = joint_sweep_passes

        if joints and all(row["range_evaluable"] for row in joints):
            for sample_index, values in enumerate(core.sobol_joint_values(joints)):
                state = observe(values, "multi_joint_sobol", sample_index)
                result["sobol_state_executed"] += 1
                result["sobol_non_adjacent_free"] += int(
                    state["non_adjacent_illegal_penetration_count"] == 0
                )

        result["single_joint_sweep_cf"] = bool(
            result["single_state_executed"] == result["single_state_expected"]
            and result["single_non_adjacent_free"] == result["single_state_expected"]
        )
        result["multi_joint_sobol_cf"] = bool(
            result["movable_dof_count"] > 0
            and result["range_evaluable_dof_count"] == result["movable_dof_count"]
            and result["sobol_state_executed"] == result["sobol_state_expected"]
            and result["sobol_non_adjacent_free"] == result["sobol_state_expected"]
        )
        expected_total = sum(
            int(result[f"{phase}_state_expected"])
            for phase in ("rest", "single", "sobol")
        )
        executed_total = sum(
            int(result[f"{phase}_state_executed"])
            for phase in ("rest", "single", "sobol")
        )
        result["measurement_complete"] = bool(
            result["range_evaluable_dof_count"] == result["movable_dof_count"]
            and executed_total == expected_total
        )
        result["strict_collision_pass"] = bool(
            result["measurement_complete"]
            and result["rest_non_adjacent_cf"]
            and result["single_joint_sweep_cf"]
            and result["multi_joint_sobol_cf"]
        )
    except Exception as exc:  # noqa: BLE001
        result["issues"].append(f"{type(exc).__name__}: {exc}")
    finally:
        if body is not None:
            bullet.removeBody(body, physicsClientId=client)
        bullet.disconnect(client)
    states = result["state_records"]
    if states:
        result["max_penetration_m"] = max(
            float(state["metric_max_penetration_m"]) for state in states
        )
        result["max_penetration_normalized"] = (
            float(result["max_penetration_m"])
            / float(result["object_bbox_diagonal_m"])
        )
        result["max_reset_readback_error"] = max(
            float(state["reset_readback_max_abs_error"]) for state in states
        )
    result["state_records_sha256"] = canonical_sha256(states)
    return result


def validate_frozen_asset_files(item: dict[str, Any], dataset_root: Path) -> None:
    package = _contained_package(dataset_root, str(item["package_relpath"]))
    observed = package_binding(package)
    if (
        observed != item.get("package_binding")
        or canonical_sha256(observed) != item.get("package_binding_sha256")
        or observed.get("content_manifest_sha256")
        != item.get("package_content_manifest_sha256")
    ):
        raise RuntimeError("package binding drift")


def validate_frozen_source_snapshot(
    item: dict[str, Any],
    dataset_root: Path,
    *,
    rederive_scale: bool = False,
) -> None:
    validate_frozen_asset_files(item, dataset_root)
    source_row = {
        **{key: item[key] for key in IDENTITY_FIELDS},
        "package_binding": item["package_binding"],
    }
    observed = audit_asset(dataset_root, source_row)
    audit_keys = tuple(_empty_audit())
    if not rederive_scale and item.get("package_audit_success"):
        audit_keys = tuple(
            key
            for key in audit_keys
            if key
            not in {
                "scale_derivation",
                "scale_derivation_sha256",
                "object_bbox_diagonal_m",
            }
        )
    if any(observed.get(key) != item.get(key) for key in audit_keys):
        raise RuntimeError("package audit snapshot drift after freeze")


def normalize_executable_path(path: Path, cwd: Path) -> Path:
    candidate = path if path.is_absolute() else cwd / path
    normalized = Path(os.path.abspath(str(candidate)))
    if not normalized.exists() or not normalized.is_file():
        raise RuntimeError(f"child launcher is not a file: {normalized}")
    if not os.access(normalized, os.X_OK):
        raise RuntimeError(f"child launcher is not executable: {normalized}")
    return normalized


def frozen_launcher_binding(path: Path, cwd: Path) -> dict[str, Any]:
    launch_path = normalize_executable_path(path, cwd)
    if launch_path != DEFAULT_CHILD_PYTHON:
        raise RuntimeError(
            f"child launcher path is not the frozen venv entrypoint: {launch_path}"
        )
    if not launch_path.is_symlink():
        raise RuntimeError("frozen child launcher is no longer a symlink")
    symlink_target = os.readlink(launch_path)
    resolved = launch_path.resolve(strict=True)
    pyvenv_cfg = launch_path.parent.parent / "pyvenv.cfg"
    if (
        launch_path.parent.is_symlink()
        or not launch_path.parent.is_dir()
        or launch_path.parent.parent.is_symlink()
        or not launch_path.parent.parent.is_dir()
    ):
        raise RuntimeError("frozen child launcher venv directories are unsafe")
    if not resolved.is_file() or not os.access(resolved, os.X_OK):
        raise RuntimeError("resolved child launcher is not an executable regular file")
    if not pyvenv_cfg.is_file() or pyvenv_cfg.is_symlink():
        raise RuntimeError("frozen child launcher pyvenv.cfg is missing or unsafe")
    binding = {
        "protocol_id": LAUNCHER_BINDING_PROTOCOL,
        "schema_version": "1.0.0",
        "launch_path": str(launch_path),
        "entry_type": "symlink",
        "symlink_target": symlink_target,
        "resolved_executable_path": str(resolved),
        "resolved_executable_bytes": resolved.stat().st_size,
        "resolved_executable_sha256": sha256_file(resolved),
        "pyvenv_cfg_path": str(pyvenv_cfg),
        "pyvenv_cfg_bytes": pyvenv_cfg.stat().st_size,
        "pyvenv_cfg_sha256": sha256_file(pyvenv_cfg),
    }
    expected = {
        "protocol_id": LAUNCHER_BINDING_PROTOCOL,
        "schema_version": "1.0.0",
        "launch_path": str(DEFAULT_CHILD_PYTHON),
        "entry_type": "symlink",
        "symlink_target": EXPECTED_CHILD_PYTHON_SYMLINK_TARGET,
        "resolved_executable_path": str(EXPECTED_CHILD_PYTHON_RESOLVED_PATH),
        "resolved_executable_bytes": EXPECTED_CHILD_PYTHON_BYTES,
        "resolved_executable_sha256": EXPECTED_CHILD_PYTHON_SHA256,
        "pyvenv_cfg_path": str(DEFAULT_CHILD_PYTHON.parent.parent / "pyvenv.cfg"),
        "pyvenv_cfg_bytes": EXPECTED_PYVENV_CFG_BYTES,
        "pyvenv_cfg_sha256": EXPECTED_PYVENV_CFG_SHA256,
    }
    if binding != expected:
        raise RuntimeError("observed child launcher does not match the frozen launcher binding")
    return binding


def _path_stat_token(path: Path, *, follow_symlinks: bool) -> dict[str, int]:
    stat_result = path.stat() if follow_symlinks else path.lstat()
    return {
        "device": stat_result.st_dev,
        "inode": stat_result.st_ino,
        "mode": stat_result.st_mode,
        "size": stat_result.st_size,
        "mtime_ns": stat_result.st_mtime_ns,
        "ctime_ns": stat_result.st_ctime_ns,
    }


def launcher_integrity_token(binding: dict[str, Any]) -> dict[str, Any]:
    launch_path = Path(str(binding["launch_path"]))
    resolved_path = Path(str(binding["resolved_executable_path"]))
    pyvenv_cfg = Path(str(binding["pyvenv_cfg_path"]))
    return {
        "launch_entry_lstat": _path_stat_token(
            launch_path, follow_symlinks=False
        ),
        "bin_directory_lstat": _path_stat_token(
            launch_path.parent, follow_symlinks=False
        ),
        "venv_directory_lstat": _path_stat_token(
            launch_path.parent.parent, follow_symlinks=False
        ),
        "resolved_executable_stat": _path_stat_token(
            resolved_path, follow_symlinks=True
        ),
        "pyvenv_cfg_lstat": _path_stat_token(
            pyvenv_cfg, follow_symlinks=False
        ),
    }


def validate_frozen_launcher_binding(binding: dict[str, Any]) -> None:
    if not isinstance(binding, dict):
        raise RuntimeError("frozen launcher binding is not an object")
    launch_path = binding.get("launch_path")
    if not isinstance(launch_path, str):
        raise RuntimeError("frozen launcher binding has no launch path")
    observed = frozen_launcher_binding(Path(launch_path), REPO)
    if observed != binding:
        raise RuntimeError("frozen launcher binding drift")


def probe_runtime_identity(
    python: Path,
    result_path: Path,
    *,
    launcher_binding: dict[str, Any] | None = None,
) -> dict[str, Any]:
    observed_launcher = frozen_launcher_binding(python, Path.cwd())
    if launcher_binding is not None and observed_launcher != launcher_binding:
        raise RuntimeError("runtime probe launcher binding mismatch")
    before_token = launcher_integrity_token(observed_launcher)
    result_path.unlink(missing_ok=True)
    environment = dict(os.environ)
    environment.update(
        {
            "OPENBLAS_NUM_THREADS": "1",
            "OMP_NUM_THREADS": "1",
            "MKL_NUM_THREADS": "1",
            "NUMEXPR_NUM_THREADS": "1",
            "VECLIB_MAXIMUM_THREADS": "1",
        }
    )
    completed = subprocess.run(
        [
            str(python),
            str(SCRIPT),
            "--phase",
            "runtime",
            "--runtime-result",
            str(result_path),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=120,
        env=environment,
        check=False,
    )
    if completed.returncode != 0 or not result_path.is_file():
        raise RuntimeError(
            "child runtime probe failed: "
            f"returncode={completed.returncode}, output={completed.stdout[-2000:]}"
        )
    after_launcher = frozen_launcher_binding(python, Path.cwd())
    after_token = launcher_integrity_token(after_launcher)
    if after_launcher != observed_launcher or after_token != before_token:
        result_path.unlink(missing_ok=True)
        raise RuntimeError("child launcher binding changed during runtime probe")
    return read_json(result_path)


def prepare(
    dataset_root: Path,
    source_records: Path,
    table3_manifest: Path,
    output: Path,
    *,
    sample_size: int,
    qualification_smoke: bool,
    child_python: Path,
    workers: int,
    timeout: int,
) -> dict[str, Any]:
    if timeout != CHILD_TIMEOUT_SECONDS:
        raise RuntimeError("requested timeout does not match frozen execution policy")
    output.mkdir(parents=True, exist_ok=True)
    launcher_binding = frozen_launcher_binding(child_python, Path.cwd())
    runtime_path = output / "child_runtime_probe.json"
    child_runtime = probe_runtime_identity(
        child_python, runtime_path, launcher_binding=launcher_binding
    )
    _load_core().require_runtime_match(current_runtime_identity(), child_runtime)
    manifest_path = output / "frozen_manifest.json"
    if manifest_path.exists():
        manifest = read_json(manifest_path)
        if int(manifest.get("sample_size", -1)) != sample_size:
            raise RuntimeError("existing manifest requested sample size mismatch")
        validate_manifest(
            manifest,
            dataset_root,
            source_records,
            table3_manifest,
            qualification_smoke=qualification_smoke,
            child_runtime=child_runtime,
            child_launcher=launcher_binding,
        )
        validate_protocol_document_snapshot(output, manifest)
        return manifest
    manifest = build_manifest(
        dataset_root,
        source_records,
        table3_manifest,
        sample_size=sample_size,
        qualification_smoke=qualification_smoke,
        child_runtime=child_runtime,
        child_launcher=launcher_binding,
        workers=workers,
    )
    write_protocol_document_snapshot(output, manifest)
    validate_protocol_document_snapshot(output, manifest)
    atomic_json(manifest_path, manifest)
    return manifest


def run_pair_policy_smoke(output: Path) -> dict[str, Any]:
    return _load_core().run_pair_policy_smoke(output)


def _reproduce_pair_policy_smoke(output: Path) -> dict[str, Any]:
    output.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(
        prefix=".pair_policy_verify_", dir=output
    ) as temporary:
        return run_pair_policy_smoke(Path(temporary))


def run_child(item_path: Path, dataset_root: Path, result_path: Path) -> None:
    item = read_json(item_path)
    result = evaluate_asset(item, dataset_root)
    atomic_json(result_path, result)


def _job_prefix(item: dict[str, Any]) -> str:
    return f"{int(item['order']):04d}_{item['dataset_id']}"


def child_result_path(output: Path, item: dict[str, Any]) -> Path:
    return output / "children" / f"{_job_prefix(item)}.json"


def _child_transport_terminal(
    result: dict[str, Any], item: dict[str, Any], output: Path
) -> bool:
    expected_log = output / "child_logs" / f"{_job_prefix(item)}.log"
    returncode = result.get("child_returncode")
    timed_out = result.get("child_timed_out")
    return bool(
        isinstance(returncode, int)
        and not isinstance(returncode, bool)
        and isinstance(timed_out, bool)
        and (not timed_out or returncode == -9)
        and result.get("cache_reused") is False
        and result.get("child_log") == str(expected_log)
    )


def _read_child_result(path: Path) -> dict[str, Any] | None:
    if not path.is_file():
        return None
    try:
        result = read_json(path)
    except (OSError, json.JSONDecodeError):
        return None
    return result if isinstance(result, dict) else None


def _valid_raw_child(
    path: Path, item: dict[str, Any], runner_hash: str
) -> bool:
    result = _read_child_result(path)
    if result is None:
        return False
    return result_matches_item(result, item, runner_hash)


def _valid_terminal_child(
    path: Path, item: dict[str, Any], runner_hash: str
) -> bool:
    result = _read_child_result(path)
    if result is None:
        return False
    output = path.parent.parent
    return bool(
        _child_transport_terminal(result, item, output)
        and result_matches_item(result, item, runner_hash)
    )


def _valid_cached_child(
    path: Path, item: dict[str, Any], runner_hash: str
) -> bool:
    if not _valid_terminal_child(path, item, runner_hash):
        return False
    result = read_json(path)
    issues = result.get("issues", [])
    if not isinstance(issues, list):
        return False
    return bool(
        result.get("child_returncode") == 0
        and result.get("child_timed_out") is False
        and "child_invalid_result_returncode_0" not in issues
    )


def run_one_subprocess(
    item_path: Path,
    item: dict[str, Any],
    dataset_root: Path,
    child_result: Path,
    child_log: Path,
    timeout: int,
    python: Path,
    launcher_binding: dict[str, Any],
    runner_hash: str,
) -> dict[str, Any]:
    observed_launcher = frozen_launcher_binding(python, Path.cwd())
    if observed_launcher != launcher_binding:
        raise RuntimeError("child subprocess launcher binding mismatch")
    launcher_token_before = launcher_integrity_token(observed_launcher)
    try:
        validate_frozen_source_snapshot(item, dataset_root)
    except Exception as exc:  # noqa: BLE001
        child_result.unlink(missing_ok=True)
        raise RuntimeError(
            f"frozen_asset_files_drift:{type(exc).__name__}: {exc}"
        ) from exc
    if _valid_cached_child(child_result, item, runner_hash):
        result = read_json(child_result)
        result["cache_reused"] = True
        return result
    child_result.unlink(missing_ok=True)
    environment = dict(os.environ)
    environment.update(
        {
            "OPENBLAS_NUM_THREADS": "1",
            "OMP_NUM_THREADS": "1",
            "MKL_NUM_THREADS": "1",
            "NUMEXPR_NUM_THREADS": "1",
            "VECLIB_MAXIMUM_THREADS": "1",
        }
    )
    command = [
        str(python),
        str(SCRIPT),
        "--phase",
        "child",
        "--dataset-root",
        str(dataset_root),
        "--child-item",
        str(item_path),
        "--child-result",
        str(child_result),
    ]
    child_log.parent.mkdir(parents=True, exist_ok=True)
    timed_out = False
    with child_log.open("wb") as log:
        try:
            try:
                completed = subprocess.run(
                    command,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    timeout=timeout,
                    env=environment,
                    check=False,
                )
                returncode = completed.returncode
            except subprocess.TimeoutExpired:
                returncode = -9
                timed_out = True
        finally:
            try:
                after_launcher = frozen_launcher_binding(python, Path.cwd())
                after_token = launcher_integrity_token(after_launcher)
            except Exception:
                child_result.unlink(missing_ok=True)
                raise
            if (
                after_launcher != observed_launcher
                or after_token != launcher_token_before
            ):
                child_result.unlink(missing_ok=True)
                raise RuntimeError(
                    "child launcher binding changed during subprocess execution"
                )
    if returncode == 0 and _valid_raw_child(child_result, item, runner_hash):
        result = read_json(child_result)
    else:
        issue = (
            "child_timeout"
            if timed_out
            else (
                "child_invalid_result_returncode_0"
                if returncode == 0
                else f"child_exit_{returncode}"
            )
        )
        result = failure_record(item, issue, timed_out=timed_out)
        result["runner_sha256"] = runner_hash
    result.update(
        {
            "child_returncode": returncode,
            "child_timed_out": timed_out,
            "child_log": str(child_log),
            "cache_reused": False,
        }
    )
    atomic_json(child_result, result)
    return result


def execute(
    manifest: dict[str, Any],
    dataset_root: Path,
    output: Path,
    *,
    workers: int,
    timeout: int,
    python: Path,
) -> list[dict[str, Any]]:
    if workers <= 0:
        raise ValueError("workers must be positive")
    if (
        timeout != CHILD_TIMEOUT_SECONDS
        or manifest.get("execution_policy") != _execution_policy()
    ):
        raise RuntimeError("execution timeout does not match frozen policy")
    validate_protocol_document_snapshot(output, manifest)
    launcher_binding = frozen_launcher_binding(python, Path.cwd())
    if launcher_binding != manifest.get("runtime", {}).get("child_launcher"):
        raise RuntimeError("execution launcher does not match frozen manifest")
    run_pair_policy_smoke(output)
    runtime_probe = probe_runtime_identity(
        python,
        output / "child_runtime_probe.json",
        launcher_binding=launcher_binding,
    )
    _load_core().require_runtime_match(manifest["runtime"]["child"], runtime_probe)
    inputs = output / "inputs"
    children = output / "children"
    logs = output / "child_logs"
    for directory in (inputs, children, logs):
        directory.mkdir(parents=True, exist_ok=True)
    jobs = []
    for item in manifest["items"]:
        prefix = _job_prefix(item)
        item_path = inputs / f"{prefix}.json"
        atomic_json(item_path, item)
        jobs.append(
            (
                item,
                item_path,
                children / f"{prefix}.json",
                logs / f"{prefix}.log",
            )
        )
    runner_hash = str(manifest["runtime"]["runner_sha256"])
    by_order: dict[int, dict[str, Any]] = {}
    started = time.time()
    with ThreadPoolExecutor(max_workers=workers) as executor:
        futures = {
            executor.submit(
                run_one_subprocess,
                item_path,
                item,
                dataset_root,
                child_path,
                log_path,
                timeout,
                python,
                launcher_binding,
                runner_hash,
            ): item
            for item, item_path, child_path, log_path in jobs
        }
        for completed, future in enumerate(as_completed(futures), 1):
            item = futures[future]
            result = future.result()
            by_order[int(item["order"])] = result
            atomic_json(
                output / "progress.json",
                {
                    "protocol_id": manifest["protocol_id"],
                    "status": (
                        "RUNNING" if completed < len(jobs) else "CHILDREN_COMPLETE"
                    ),
                    "completed_assets": completed,
                    "total_assets": len(jobs),
                    "load_successes": sum(
                        bool(row["load_success"]) for row in by_order.values()
                    ),
                    "measurement_complete_assets": sum(
                        bool(row["measurement_complete"])
                        for row in by_order.values()
                    ),
                    "timeouts": sum(
                        bool(row["child_timed_out"]) for row in by_order.values()
                    ),
                    "elapsed_seconds": time.time() - started,
                    "last_completed_asset_key": item["asset_key"],
                },
            )
            print(
                f"table4 {completed}/{len(jobs)} index={item['selection_rank']} "
                f"load={int(result['load_success'])} "
                f"complete={int(result['measurement_complete'])}",
                flush=True,
            )
    return [by_order[index] for index in range(len(jobs))]


def summarize(manifest: dict[str, Any], output: Path) -> dict[str, Any]:
    validate_protocol_document_snapshot(output, manifest)
    records = []
    state_records = []
    runner_hash = str(manifest["runtime"]["runner_sha256"])
    for item in manifest["items"]:
        path = child_result_path(output, item)
        if not _valid_terminal_child(path, item, runner_hash):
            raise RuntimeError(f"missing or stale child result: {path}")
        row = read_json(path)
        asset_states = row.pop("state_records", [])
        if not result_matches_item(row, item, runner_hash, asset_states):
            raise RuntimeError(f"child state closure mismatch: {path}")
        state_records.extend(asset_states)
        records.append(row)
    summary = summarize_records(manifest, records)
    atomic_json(output / "asset_records.json", records)
    atomic_jsonl(output / "state_records.jsonl", state_records)
    atomic_json(output / "summary.json", summary)
    render_report(summary, output)
    return summary


def _format_metric(row: dict[str, Any], numerator_key: str = "passed") -> str:
    numerator = row[numerator_key]
    denominator = row["denominator"]
    percentage = 100.0 * row["rate"] if row["rate"] is not None else float("nan")
    return f"{numerator}/{denominator} ({percentage:.3f}%)"


def report_text(summary: dict[str, Any]) -> str:
    metrics = summary["metrics"]
    maximum = metrics["max_penetration"]
    macro = summary["category_macro"]
    macro_text = ", ".join(
        f"{key}={100.0 * value:.3f}%"
        for key, value in sorted(macro.items())
        if value is not None
    )
    lines = [
        f"# {summary['cohort']['label']}: URDF Sim-Ready Table 4",
        "",
        f"Status: **{summary['status']}**",
        "",
        "This is the exact frozen N=800 cohort identified by the supplied Table 3 asset_records.jsonl, reconstructed in selection_rank order from the 3,217-asset LAM release universe. It is not a full-release or category-balanced result.",
        "",
        f"Observed authoritative categories: {summary['cohort']['category_count']}. Unweighted category macro: {macro_text}.",
        "",
        "| Metric | Result |",
        "|---|---:|",
        f"| Rest All-pair CF | {_format_metric(metrics['rest_all_pair_cf'])} |",
        f"| Rest Non-adjacent CF | {_format_metric(metrics['rest_non_adjacent_cf'])} |",
        f"| Single-joint Sweep CF | {_format_metric(metrics['single_joint_sweep_cf'])} |",
        f"| Multi-joint Sobol CF | {_format_metric(metrics['multi_joint_sobol_cf'])} |",
        f"| Collision-state Rate | {_format_metric(metrics['collision_state_rate'], 'collision_states')} |",
        "| AOR | N/E |",
        f"| Max Penetration | {maximum['maximum_observed_normalized']} (fully measured {maximum['fully_measured_assets']}/{maximum['denominator']}; observed {maximum['observed_assets']}/{maximum['denominator']}; {maximum['status']}) |",
        f"| Collision-free Range | {_format_metric(metrics['collision_free_range'], 'passed_states')} |",
        f"| Strict Collision Pass | {_format_metric(metrics['strict_collision_pass'])} |",
        "",
        "Collision-state Rate is fail-closed. Unexecuted configurations remain in the frozen denominator and count as non-free.",
        "",
        "AOR is N/E because no stable exact overlap-volume calculation was run. Sweeps are discrete; no CCD, semantic-joint, or physical-dynamics claim is made.",
    ]
    return "\n".join(lines) + "\n"


def render_report(summary: dict[str, Any], output: Path) -> None:
    atomic_text(output / "report.md", report_text(summary))


def _read_jsonl_strict(path: Path) -> list[dict[str, Any]]:
    rows = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                raise RuntimeError(
                    f"blank line in {path.name} at line {line_number}"
                )
            value = json.loads(line)
            if not isinstance(value, dict):
                raise RuntimeError(
                    f"non-object row in {path.name} at line {line_number}"
                )
            rows.append(value)
    return rows


def _authoritative_child_aggregates(
    manifest: dict[str, Any], output: Path
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], list[dict[str, Any]]]:
    runtime = manifest.get("runtime", {})
    runner_hash = str(runtime.get("runner_sha256"))
    records: list[dict[str, Any]] = []
    states: list[dict[str, Any]] = []
    bindings: list[dict[str, Any]] = []
    for item in manifest.get("items", []):
        if not isinstance(item, dict):
            raise RuntimeError("frozen manifest item is not an object")
        path = child_result_path(output, item)
        child = read_json(path)
        if not isinstance(child, dict):
            raise RuntimeError(f"child result is not an object: {path}")
        if not _child_transport_terminal(child, item, output):
            raise RuntimeError(f"child transport receipt is not terminal: {path}")
        child_states = child.get("state_records", [])
        if not isinstance(child_states, list) or not all(
            isinstance(state, dict) for state in child_states
        ):
            raise RuntimeError(f"child state records are malformed: {path}")
        if not result_matches_item(child, item, runner_hash, child_states):
            raise RuntimeError(f"child result does not close against manifest: {path}")
        record = dict(child)
        record.pop("state_records", None)
        records.append(record)
        states.extend(child_states)
        bindings.append(
            {
                "order": item["order"],
                "dataset_id": item["dataset_id"],
                "path": path.relative_to(output).as_posix(),
                "sha256": sha256_file(path),
            }
        )
    return records, states, bindings


_CHILD_TRANSPORT_FIELDS = frozenset(
    {"child_returncode", "child_timed_out", "child_log", "cache_reused"}
)


def _measurement_payload(result: dict[str, Any]) -> dict[str, Any]:
    if not isinstance(result, dict):
        raise RuntimeError("measurement result is not an object")
    return {
        key: value
        for key, value in result.items()
        if key not in _CHILD_TRANSPORT_FIELDS
    }


def _replay_frozen_measurements(
    manifest: dict[str, Any], output: Path, dataset_root: Path
) -> list[dict[str, Any]]:
    bindings: list[dict[str, Any]] = []
    for item in manifest.get("items", []):
        if not isinstance(item, dict):
            raise RuntimeError("frozen manifest item is not an object during replay")
        path = child_result_path(output, item)
        child = read_json(path)
        if not isinstance(child, dict):
            raise RuntimeError(f"child result is not an object during replay: {path}")
        if not _child_transport_terminal(child, item, output):
            raise RuntimeError(
                f"frozen child transport provenance mismatch: {item.get('dataset_id')}"
            )
        observed_payload = _measurement_payload(child)
        returncode = int(child["child_returncode"])
        if returncode == 0:
            replay_mode = "simulator_replay"
            replayed_payload = _measurement_payload(
                evaluate_asset(item, dataset_root)
            )
        else:
            replay_mode = "canonical_terminal_failure"
            issue = "child_timeout" if child["child_timed_out"] else f"child_exit_{returncode}"
            replayed_payload = _measurement_payload(
                failure_record(
                    item, issue, timed_out=bool(child["child_timed_out"])
                )
            )
        observed_sha256 = canonical_sha256(observed_payload)
        replayed_sha256 = canonical_sha256(replayed_payload)
        if observed_sha256 != replayed_sha256:
            raise RuntimeError(
                "measurement replay mismatch for "
                f"{item.get('dataset_id')}: observed={observed_sha256}, "
                f"replayed={replayed_sha256}"
            )
        bindings.append(
            {
                "order": item["order"],
                "dataset_id": item["dataset_id"],
                "replay_mode": replay_mode,
                "child_returncode": returncode,
                "child_timed_out": bool(child["child_timed_out"]),
                "measurement_payload_sha256": replayed_sha256,
                "state_records_sha256": replayed_payload["state_records_sha256"],
            }
        )
    expected = int(manifest.get("sample_size", len(bindings)))
    if len(bindings) != expected:
        raise RuntimeError(
            f"measurement replay count mismatch: expected {expected}, observed {len(bindings)}"
        )
    return bindings


def _verification_identity(
    manifest: dict[str, Any], output: Path
) -> dict[str, Any]:
    if not isinstance(manifest, dict):
        manifest = {}
    source = manifest.get("source", {})
    if not isinstance(source, dict):
        source = {}
    try:
        sample_size = int(manifest.get("sample_size", -1))
    except (TypeError, ValueError):
        sample_size = -1
    qualification_smoke = bool(manifest.get("qualification_smoke"))
    formal_evaluation = bool(
        not qualification_smoke
        and sample_size == SAMPLE_SIZE
        and manifest.get("protocol_id") == PROTOCOL_ID
        and manifest.get("audit_summary") == EXPECTED_FORMAL_AUDIT_SUMMARY
    )
    manifest_path = output / "frozen_manifest.json"
    return {
        "evaluated_protocol_id": manifest.get("protocol_id"),
        "qualification_smoke": qualification_smoke,
        "formal_evaluation": formal_evaluation,
        "sample_size": sample_size,
        "cohort_label": manifest.get("cohort_label"),
        "manifest_content_sha256": manifest.get("manifest_content_sha256"),
        "manifest_file_sha256": (
            sha256_file(manifest_path) if manifest_path.is_file() else None
        ),
        "ordered_selected_asset_keys_sha256": manifest.get(
            "ordered_selected_asset_keys_sha256"
        ),
        "table3_asset_records_sha256": source.get(
            "table3_asset_records_sha256"
        ),
        "table3_manifest_sha256": source.get("table3_manifest_sha256"),
        "table3_manifest_content_sha256": source.get(
            "table3_manifest_content_sha256"
        ),
        "protocol_document_sha256_at_freeze": source.get(
            "protocol_document_sha256_at_freeze"
        ),
    }


def _verify_impl(manifest: dict[str, Any], output: Path) -> dict[str, Any]:
    checks: dict[str, bool] = {}
    errors: dict[str, str] = {}

    def check(name: str, operation: Any) -> Any:
        try:
            value = operation()
            passed = bool(value) if isinstance(value, bool) else True
            checks[name] = passed
            if not passed:
                errors[name] = "check returned false"
            return value
        except Exception as exc:  # noqa: BLE001
            checks[name] = False
            errors[name] = f"{type(exc).__name__}: {exc}"
            return None

    dataset_root = Path(str(manifest.get("dataset_root", "")))
    source = manifest.get("source", {})
    source_records = Path(str(source.get("table3_asset_records_path", "")))
    table3_manifest = Path(str(source.get("table3_manifest_path", "")))
    qualification_smoke = bool(manifest.get("qualification_smoke"))
    runtime_probe = check(
        "child_runtime_receipt_readable",
        lambda: read_json(output / "child_runtime_probe.json"),
    )
    check(
        "manifest_revalidates_against_authoritative_sources",
        lambda: validate_manifest(
            manifest,
            dataset_root,
            source_records,
            table3_manifest,
            qualification_smoke=qualification_smoke,
            child_runtime=runtime_probe,
        ),
    )
    check(
        "runner_sha256_matches_manifest",
        lambda: sha256_file(SCRIPT) == manifest["runtime"]["runner_sha256"],
    )
    check(
        "collision_core_sha256_matches_pin",
        lambda: sha256_file(CORE_SCRIPT)
        == manifest["runtime"]["collision_core_sha256"]
        == EXPECTED_CORE_SHA256,
    )
    check(
        "current_runtime_matches_manifest",
        lambda: current_runtime_identity() == manifest["runtime"]["child"],
    )
    check(
        "launcher_binding_matches_manifest",
        lambda: validate_frozen_launcher_binding(
            manifest["runtime"]["child_launcher"]
        ),
    )
    check(
        "protocol_document_snapshot_matches_manifest",
        lambda: validate_protocol_document_snapshot(output, manifest),
    )

    records = check(
        "asset_records_readable",
        lambda: read_json(output / "asset_records.json"),
    )
    states = check(
        "state_records_jsonl_readable",
        lambda: _read_jsonl_strict(output / "state_records.jsonl"),
    )
    summary = check("summary_readable", lambda: read_json(output / "summary.json"))
    if not isinstance(records, list):
        records = []
    if not isinstance(states, list):
        states = []
    items = manifest.get("items", []) if isinstance(manifest.get("items"), list) else []
    authoritative = check(
        "authoritative_children_readable_and_valid",
        lambda: _authoritative_child_aggregates(manifest, output),
    )
    authoritative_bindings: list[dict[str, Any]] = []
    if (
        isinstance(authoritative, tuple)
        and len(authoritative) == 3
        and isinstance(authoritative[2], list)
    ):
        authoritative_bindings = authoritative[2]
    check(
        "aggregates_match_authoritative_children",
        lambda: (
            isinstance(authoritative, tuple)
            and len(authoritative) == 3
            and records == authoritative[0]
            and states == authoritative[1]
        ),
    )
    check(
        "sample_size_exact",
        lambda: len(records) == int(manifest["sample_size"]) == len(items),
    )
    check(
        "record_order_matches_manifest",
        lambda: [row.get("dataset_id") for row in records]
        == [item.get("dataset_id") for item in items],
    )

    states_by_id: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for state in states:
        states_by_id[str(state.get("dataset_id"))].append(state)
    known_ids = {str(item.get("dataset_id")) for item in items}
    check(
        "no_unknown_state_asset_keys",
        lambda: set(states_by_id).issubset(known_ids),
    )
    closure_ok = len(records) == len(items)
    if closure_ok:
        runner_hash = str(manifest.get("runtime", {}).get("runner_sha256"))
        for item, record in zip(items, records):
            asset_states = states_by_id.get(str(item["dataset_id"]), [])
            if not result_matches_item(record, item, runner_hash, asset_states):
                closure_ok = False
                break
    checks["state_records_close_against_assets"] = closure_ok
    if not closure_ok:
        errors["state_records_close_against_assets"] = "result/state closure mismatch"
    expected_state_order = [
        state
        for item in items
        for state in states_by_id.get(str(item.get("dataset_id")), [])
    ]
    check("state_global_order_matches_manifest", lambda: expected_state_order == states)
    check(
        "state_record_count_matches_assets",
        lambda: len(states)
        == sum(
            int(record.get(f"{phase}_state_executed", -1))
            for record in records
            for phase in ("rest", "single", "sobol")
        ),
    )

    frozen_sources_ok = True
    frozen_source_error = None
    for item in items:
        try:
            validate_frozen_source_snapshot(
                item, dataset_root, rederive_scale=True
            )
        except Exception as exc:  # noqa: BLE001
            frozen_sources_ok = False
            frozen_source_error = f"{item.get('asset_key')}: {type(exc).__name__}: {exc}"
            break
    checks["frozen_source_snapshots_match"] = frozen_sources_ok
    if frozen_source_error is not None:
        errors["frozen_source_snapshots_match"] = frozen_source_error

    measurement_replays = check(
        "all_frozen_measurements_replay_exactly",
        lambda: _replay_frozen_measurements(manifest, output, dataset_root),
    )
    replay_bindings = measurement_replays if isinstance(measurement_replays, list) else []

    recomputed = None
    if len(records) == len(items):
        recomputed = check(
            "summary_recomputable", lambda: summarize_records(manifest, records)
        )
    else:
        checks["summary_recomputable"] = False
        errors["summary_recomputable"] = "record count mismatch"
    check("summary_recomputes_exactly", lambda: summary == recomputed)
    check(
        "report_recomputes_exactly",
        lambda: (output / "report.md").read_text(encoding="utf-8")
        == report_text(recomputed),
    )
    check(
        "single_state_denominator_frozen",
        lambda: sum(int(row["single_state_expected"]) for row in records)
        == SINGLE_SAMPLES * sum(int(item["movable_dof_count"]) for item in items),
    )
    check(
        "sobol_state_denominator_frozen",
        lambda: sum(int(row["sobol_state_expected"]) for row in records)
        == SOBOL_SAMPLES * sum(int(item["movable_dof_count"]) > 0 for item in items),
    )
    check(
        "aor_remains_not_evaluable",
        lambda: recomputed["metrics"]["aor"]["status"] == "N/E",
    )
    check(
        "claim_boundary_preserved",
        lambda: recomputed["claim_boundary"]
        == {
            "continuous_collision_detection": "not_run",
            "semantic_joint_correctness": "not_evaluated",
            "physical_dynamics_validity": "not_evaluated",
            "full_release_result": False,
            "shared_category_balanced_result": False,
        },
    )
    pair_receipt = check(
        "pair_policy_receipt_readable",
        lambda: read_json(output / "pair_policy_smoke.json"),
    )
    check(
        "pair_policy_smoke_semantics_pass",
        lambda: (
            pair_receipt["protocol_id"]
            == "urdf_table4_pybullet_pair_policy_smoke_v1"
            and pair_receipt["status"] == "PASS"
            and int(pair_receipt["all_pair_illegal_penetration_count"]) > 0
            and int(pair_receipt["non_adjacent_illegal_penetration_count"]) == 0
            and int(pair_receipt["pybullet_api_version"])
            == int(manifest["runtime"]["child"]["pybullet_api_version"])
        ),
    )
    reproduced_pair_receipt = check(
        "pair_policy_smoke_reruns",
        lambda: _reproduce_pair_policy_smoke(output),
    )
    check(
        "pair_policy_smoke_reproduces_exactly",
        lambda: pair_receipt == reproduced_pair_receipt,
    )

    artifact_names = (
        "frozen_manifest.json",
        PROTOCOL_DOCUMENT_SNAPSHOT,
        "child_runtime_probe.json",
        "pair_policy_smoke.json",
        "asset_records.json",
        "state_records.jsonl",
        "summary.json",
        "report.md",
    )
    artifact_hashes = {}
    for name in artifact_names:
        path = output / name
        if path.is_file():
            artifact_hashes[name] = sha256_file(path)
    receipt = {
        "protocol_id": VERIFY_PROTOCOL_ID,
        **_verification_identity(manifest, output),
        "status": "PASS" if checks and all(checks.values()) else "FAIL",
        "checked_at_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "checks": checks,
        "errors": errors,
        "artifact_sha256": artifact_hashes,
        "runner_sha256": sha256_file(SCRIPT),
        "collision_core_sha256": sha256_file(CORE_SCRIPT),
        "child_launcher_binding_sha256": canonical_sha256(
            manifest.get("runtime", {}).get("child_launcher")
        ),
        "authoritative_child_result_count": len(authoritative_bindings),
        "authoritative_child_results_sha256": canonical_sha256(
            authoritative_bindings
        ),
        "frozen_measurement_replay_count": len(replay_bindings),
        "frozen_measurement_replays_sha256": canonical_sha256(replay_bindings),
        "experiment_status": (
            summary.get("status") if isinstance(summary, dict) else None
        ),
        "retained_failure_assets": (
            int(summary["cohort"]["selected"])
            - int(summary["cohort"]["measurement_complete"])
            if isinstance(summary, dict)
            else None
        ),
        "unexecuted_states": (
            int(summary["metrics"]["collision_state_rate"]["unexecuted_states"])
            if isinstance(summary, dict)
            else None
        ),
    }
    atomic_json(output / "verification.json", receipt)
    if receipt["status"] != "PASS":
        failed = sorted(name for name, passed in checks.items() if not passed)
        raise VerificationFailed(f"verification failed: {failed}")
    return receipt


def verify(manifest: dict[str, Any], output: Path) -> dict[str, Any]:
    try:
        return _verify_impl(manifest, output)
    except VerificationFailed:
        raise
    except Exception as exc:  # noqa: BLE001
        _write_fatal_verification_receipt(manifest, output, exc)
        raise VerificationAborted(
            f"verification aborted: {type(exc).__name__}: {exc}"
        ) from exc


def _write_fatal_verification_receipt(
    manifest: dict[str, Any], output: Path, exc: BaseException
) -> dict[str, Any]:
    artifact_hashes: dict[str, str] = {}
    for name in (
        "frozen_manifest.json",
        PROTOCOL_DOCUMENT_SNAPSHOT,
        "child_runtime_probe.json",
        "pair_policy_smoke.json",
        "asset_records.json",
        "state_records.jsonl",
        "summary.json",
        "report.md",
    ):
        path = output / name
        try:
            if path.is_file():
                artifact_hashes[name] = sha256_file(path)
        except OSError:
            continue
    receipt = {
        "protocol_id": VERIFY_PROTOCOL_ID,
        **_verification_identity(manifest, output),
        "status": "FAIL",
        "checked_at_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "checks": {"verification_completed_without_uncaught_exception": False},
        "errors": {"fatal_verification_error": f"{type(exc).__name__}: {exc}"},
        "artifact_sha256": artifact_hashes,
        "runner_sha256": sha256_file(SCRIPT),
        "collision_core_sha256": sha256_file(CORE_SCRIPT),
        "child_launcher_binding_sha256": (
            canonical_sha256(manifest.get("runtime", {}).get("child_launcher"))
            if isinstance(manifest, dict)
            else None
        ),
        "authoritative_child_result_count": None,
        "authoritative_child_results_sha256": None,
        "frozen_measurement_replay_count": None,
        "frozen_measurement_replays_sha256": None,
        "experiment_status": None,
        "retained_failure_assets": None,
        "unexecuted_states": None,
    }
    atomic_json(output / "verification.json", receipt)
    return receipt


def invalidate_prior_verification(output: Path, phase: str) -> None:
    prior_path = output / "verification.json"
    prior_sha256 = None
    try:
        if prior_path.is_file():
            prior_sha256 = sha256_file(prior_path)
    except OSError:
        prior_sha256 = None
    receipt = {
        "protocol_id": VERIFY_PROTOCOL_ID,
        "status": "FAIL",
        "checked_at_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "checks": {"current_phase_has_completed_verification": False},
        "errors": {
            "prior_verification_invalidated": (
                f"phase {phase} started; no current PASS receipt exists"
            )
        },
        "phase": phase,
        "prior_verification_sha256": prior_sha256,
        "runner_sha256": sha256_file(SCRIPT),
        "collision_core_sha256": sha256_file(CORE_SCRIPT),
    }
    atomic_json(prior_path, receipt)


def verify_output(
    dataset_root_arg: Path,
    source_records_arg: Path,
    table3_manifest_arg: Path,
    output: Path,
    python_arg: Path,
    *,
    qualification_smoke: bool,
) -> dict[str, Any]:
    manifest: dict[str, Any] = {}
    try:
        dataset_root = dataset_root_arg.resolve(strict=True)
        source_records = source_records_arg.resolve(strict=True)
        table3_manifest = table3_manifest_arg.resolve(strict=True)
        child_python = normalize_executable_path(python_arg, Path.cwd())
        launcher_binding = frozen_launcher_binding(child_python, Path.cwd())
        raw_manifest = read_json(output / "frozen_manifest.json")
        if not isinstance(raw_manifest, dict):
            raise RuntimeError("frozen manifest is not an object")
        manifest = raw_manifest
        if bool(manifest.get("qualification_smoke")) is not qualification_smoke:
            raise RuntimeError("requested verification mode does not match manifest")
        if Path(
            str(manifest.get("source", {}).get("table3_asset_records_path", ""))
        ).resolve(strict=True) != source_records:
            raise RuntimeError("requested Table 3 asset records do not match manifest")
        if Path(
            str(manifest.get("source", {}).get("table3_manifest_path", ""))
        ).resolve(strict=True) != table3_manifest:
            raise RuntimeError("requested Table 3 manifest does not match frozen manifest")
        if manifest.get("runtime", {}).get("child_launcher") != launcher_binding:
            raise RuntimeError("verification launcher does not match frozen manifest")
        probe_runtime_identity(
            child_python,
            output / "child_runtime_probe.json",
            launcher_binding=launcher_binding,
        )
        return verify(manifest, output)
    except (VerificationFailed, VerificationAborted):
        raise
    except Exception as exc:  # noqa: BLE001
        _write_fatal_verification_receipt(manifest, output, exc)
        raise VerificationAborted(
            f"verification aborted: {type(exc).__name__}: {exc}"
        ) from exc


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--phase",
        choices=("all", "prepare", "run", "summarize", "verify", "child", "runtime"),
        default="all",
    )
    parser.add_argument("--dataset-root", type=Path, default=DEFAULT_DATASET)
    parser.add_argument(
        "--source-records", type=Path, default=DEFAULT_SOURCE_RECORDS
    )
    parser.add_argument(
        "--table3-manifest", type=Path, default=DEFAULT_SOURCE_MANIFEST
    )
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--sample-size", type=int, default=SAMPLE_SIZE)
    parser.add_argument("--qualification-smoke", action="store_true")
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--timeout", type=int, default=CHILD_TIMEOUT_SECONDS)
    parser.add_argument("--python", type=Path, default=DEFAULT_CHILD_PYTHON)
    parser.add_argument("--child-item", type=Path)
    parser.add_argument("--child-result", type=Path)
    parser.add_argument("--runtime-result", type=Path)
    args = parser.parse_args()

    if args.phase == "runtime":
        if args.runtime_result is None:
            parser.error("runtime phase requires --runtime-result")
        atomic_json(args.runtime_result, current_runtime_identity())
        return 0

    if args.phase == "child":
        if args.child_item is None or args.child_result is None:
            parser.error("child phase requires --child-item and --child-result")
        dataset_root = args.dataset_root.resolve(strict=True)
        run_child(args.child_item, dataset_root, args.child_result)
        return 0

    output = args.output.resolve()
    invalidate_prior_verification(output, args.phase)
    if args.phase == "verify":
        receipt = verify_output(
            args.dataset_root,
            args.source_records,
            args.table3_manifest,
            output,
            args.python,
            qualification_smoke=args.qualification_smoke,
        )
        print(json.dumps(receipt, indent=2), flush=True)
        return 0

    dataset_root = args.dataset_root.resolve(strict=True)
    source_records = args.source_records.resolve(strict=True)
    table3_manifest = args.table3_manifest.resolve(strict=True)
    child_python = normalize_executable_path(args.python, Path.cwd())
    manifest: dict[str, Any]
    if args.phase in {"all", "prepare"}:
        manifest = prepare(
            dataset_root,
            source_records,
            table3_manifest,
            output,
            sample_size=args.sample_size,
            qualification_smoke=args.qualification_smoke,
            child_python=child_python,
            workers=args.workers,
            timeout=args.timeout,
        )
        print(
            json.dumps(
                {
                    "manifest": str(output / "frozen_manifest.json"),
                    "sample_size": manifest["sample_size"],
                    "ordered_selected_asset_keys_sha256": manifest[
                        "ordered_selected_asset_keys_sha256"
                    ],
                },
                indent=2,
            ),
            flush=True,
        )
        if args.phase == "prepare":
            return 0
    else:
        manifest = read_json(output / "frozen_manifest.json")
        launcher_binding = frozen_launcher_binding(child_python, Path.cwd())
        runtime_probe = probe_runtime_identity(
            child_python,
            output / "child_runtime_probe.json",
            launcher_binding=launcher_binding,
        )
        validate_manifest(
            manifest,
            dataset_root,
            source_records,
            table3_manifest,
            qualification_smoke=args.qualification_smoke,
            child_runtime=runtime_probe,
            child_launcher=launcher_binding,
        )

    if args.phase in {"all", "run"}:
        execute(
            manifest,
            dataset_root,
            output,
            workers=args.workers,
            timeout=args.timeout,
            python=child_python,
        )
        if args.phase == "run":
            return 0
    if args.phase in {"all", "summarize"}:
        summary = summarize(manifest, output)
        print(json.dumps(summary["cohort"], indent=2), flush=True)
        if args.phase == "summarize":
            return 0
    if args.phase == "all":
        receipt = verify(manifest, output)
        print(json.dumps(receipt, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
