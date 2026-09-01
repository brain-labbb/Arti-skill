#!/usr/bin/env python3
"""Run Table 2 supplementary static diagnostics on the frozen Articraft-10K Table 2 cohort.

This adapter reuses the exact frozen Table 2 Articraft-10K cohort manifest
(`table2_urdf_articraft10k_n800_seed20260813_20260813T145915Z/manifest.json`),
preserving `.records[].package` order, and evaluates the four proposed
Table 2 supplementary metrics with the same-version static atom module
(`lam_supplementary_static.py`) that the LAM supplementary run froze:

- Visual-bearing Collision Coverage (asset level, fail closed; link-micro)
- Joint-limit Portability (joint level over frozen J_eval)
- Joint Dynamics Coverage (joint level over frozen J_eval)
- Placeholder-mass Incidence (registry frozen empty -> N/E, coverage reported)

The placeholder-mass registry is frozen empty because no Articraft tool
default mass/inertia template is documented in the frozen release docs or the
official record bundle; templates may only come from frozen tool defaults or
public documentation, never from observed results.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
from typing import Any


SCRIPT_PATH = Path(__file__).resolve()
REPO_ROOT = SCRIPT_PATH.parents[2]
STATIC_ATOMS_PATH = SCRIPT_PATH.with_name("lam_supplementary_static.py")
PROTOCOL_PATH = REPO_ROOT / "exp/URDF-Sim-Ready-Automatic-Evaluation.md"
DEFAULT_DATASET_ROOT = REPO_ROOT / "exp/Articraft-10K"
DEFAULT_COHORT_MANIFEST = (
    REPO_ROOT
    / "exp/runtime/table2_urdf_articraft10k_n800_seed20260813_20260813T145915Z/manifest.json"
)
DEFAULT_CATEGORY_RECORDS_ROOT = REPO_ROOT / "exp/baselines/Articraft-10K-official/records"
DEFAULT_TABLE3_RECORDS = (
    REPO_ROOT
    / "exp/runtime/urdf_table3_articraft10k_table2_n800_20260814T040300Z/asset_records.jsonl"
)
DEFAULT_OUTPUT_PARENT = REPO_ROOT / "exp/runtime"

DATASET_NAME = "Articraft-10K"
PROTOCOL_ID = "urdf-sim-ready-table2sup-articraft10k-table2-cohort-v1"
URDF_RELATIVE_PATH = "model.urdf"
FORMAL_N_RELEASE = 9996
FORMAL_N_EVAL = 800
FORMAL_J_EVAL = 2865
FORMAL_RELEASE_CATEGORY_COUNT = 240
FORMAL_EVAL_CATEGORY_COUNT = 222
FORMAL_SEED = 20260813
FORMAL_COHORT_FILE_SHA256 = "13c47e2b2affadb951a01cab826bae139852fca5769e99ec081cc916ffa6373d"
FORMAL_COHORT_CONTENT_SHA256 = "576852cb6da00775e1c51360b82b4be40e0a614e4fb0cfb1bae066912eed56a3"
FORMAL_SELECTED_ASSET_IDS_SHA256 = "79c44441600077513d3cde1cda8fef38324e1a0ee660730b860d5313f0ae9784"
FORMAL_RELEASE_ASSET_IDS_SHA256 = "a52fab1cc35e9948ea75e5a7cb4e99408ae39a2e5d27eca3a479c24d5c9606ff"
FORMAL_SELECTED_CATEGORY_MAPPING_SHA256 = "0305569f49d2aa1acb72fbb7bc8dcaf68ca3dd4a5bd7eba140b5bac4c8c0f449"
FORMAL_CATEGORY_RECORDS_REVISION = "677ca9722427dce500873730255874c8c3f07eb2"
FORMAL_TABLE3_RECORDS_FILE_SHA256 = "2dbb09fab36fe60b469eb38439708250f4af3fe75fb0d6dcd118e49c8febf103"
FORMAL_TABLE3_MANIFEST_FILE_SHA256 = "bd4d04de43117df39cd2e81b68d637b573397135725d3836042c6f8ef2afb864"
FORMAL_TABLE3_MANIFEST_CONTENT_SHA256 = "9cba009db52b2fc40d8e31468fd6bad9b1a6551199f4ffaf4b218dc9280b8800"
SELECTION_PROTOCOL = "exact-table2-record-package-order-v1"
# Frozen empty: no Articraft exporter/simulator default mass or inertia template
# is documented in the frozen release docs or official record bundle.  The
# protocol only permits templates from frozen tool defaults or public docs.
PLACEHOLDER_MASS_REGISTRY: list[dict[str, Any]] = []
PLACEHOLDER_REGISTRY_POLICY = (
    "frozen empty: no documented Articraft tool-default mass/inertia template; "
    "templates may only come from frozen tool defaults or public documentation"
)
DOF_BINS = ("0", "1", "2-3", "4-7", ">=8")
DEFAULT_WORKERS = 4
DEFAULT_TIMEOUT_SECONDS = 120.0


def _load_static_atoms():
    spec = importlib.util.spec_from_file_location("urdf_table2sup_static_atoms", STATIC_ATOMS_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import static atoms module: {STATIC_ATOMS_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


static_atoms = _load_static_atoms()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(4 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_json(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False)


def canonical_sha256(value: Any) -> str:
    return hashlib.sha256(canonical_json(value).encode("utf-8")).hexdigest()


def utc_now() -> str:
    from datetime import datetime, timezone

    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def atomic_write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    temporary = Path(name)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            handle.write(text)
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def atomic_write_json(path: Path, value: Any) -> None:
    atomic_write_text(path, json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n")


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        return []
    rows = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def append_jsonl(path: Path, record: dict[str, Any]) -> None:
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")


def _environment() -> dict[str, Any]:
    import platform

    environment: dict[str, Any] = {
        "python": platform.python_version(),
        "python_implementation": platform.python_implementation(),
        "platform": platform.platform(),
        "executable": sys.executable,
        "gpu_required": False,
        "standard_parser": {
            "package": "xml.etree.ElementTree (stdlib)",
            "note": "static XML atoms; urdfpy is not used by Table 2 supplementary atoms",
        },
    }
    for package in ("numpy", "trimesh"):
        try:
            module = importlib.import_module(package)
            environment[f"{package}_version"] = getattr(module, "__version__", "unknown")
        except Exception:  # noqa: BLE001
            environment[f"{package}_version"] = None
    return environment


def _package_file_manifest(package: Path) -> list[dict[str, Any]]:
    package = package.resolve(strict=True)
    if not package.is_dir():
        raise ValueError(f"package is not a directory: {package}")
    rows: list[dict[str, Any]] = []
    for current_raw, directory_names, file_names in os.walk(package, followlinks=False):
        current = Path(current_raw)
        directory_names.sort()
        file_names.sort()
        for name in directory_names:
            child = current / name
            if child.is_symlink():
                raise ValueError(f"package contains directory symlink: {child.relative_to(package)}")
        for name in file_names:
            path = current / name
            relative = path.relative_to(package).as_posix()
            if path.is_symlink():
                raise ValueError(f"package contains file symlink: {relative}")
            canonical = path.resolve(strict=True)
            canonical.relative_to(package)
            if not canonical.is_file():
                raise ValueError(f"package entry is not a regular file: {relative}")
            rows.append({"path": relative, "bytes": canonical.stat().st_size, "sha256": sha256_file(canonical)})
    return rows


def _package_binding(package: Path) -> dict[str, Any]:
    files = _package_file_manifest(package)
    return {
        "file_count": len(files),
        "total_bytes": sum(row["bytes"] for row in files),
        "files": files,
        "content_manifest_sha256": canonical_sha256(files),
    }


def _category_revision(records_root: Path) -> str | None:
    checkout = records_root.parent
    result = subprocess.run(
        ["git", "-C", str(checkout), "rev-parse", "HEAD"],
        check=False,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip() if result.returncode == 0 else None


def _read_category(records_root: Path, asset_id: str) -> str:
    path = records_root / asset_id / "record.json"
    try:
        record = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"category record is unavailable or invalid: {asset_id}") from exc
    if record.get("record_id") != asset_id:
        raise ValueError(f"category record identity mismatch: {asset_id}")
    category = str(record.get("category_slug", "")).strip()
    if not category:
        raise ValueError(f"category_slug is missing: {asset_id}")
    return category


def _load_table3_records(path: Path, formal: bool) -> dict[str, dict[str, Any]]:
    if formal:
        if sha256_file(path) != FORMAL_TABLE3_RECORDS_FILE_SHA256:
            raise RuntimeError("formal Table 3 asset-records file SHA256 mismatch")
        table3_manifest = path.with_name("manifest.json")
        if sha256_file(table3_manifest) != FORMAL_TABLE3_MANIFEST_FILE_SHA256:
            raise RuntimeError("formal Table 3 manifest file SHA256 mismatch")
        manifest_payload = json.loads(table3_manifest.read_text(encoding="utf-8"))
        if manifest_payload.get("manifest_content_sha256") != FORMAL_TABLE3_MANIFEST_CONTENT_SHA256:
            raise RuntimeError("formal Table 3 manifest content SHA256 mismatch")
    rows: dict[str, dict[str, Any]] = {}
    for record in load_jsonl(path):
        key = record.get("asset_key") or record.get("asset_id")
        if not isinstance(key, str) or key in rows:
            raise ValueError(f"invalid or duplicate Table 3 record key: {key!r}")
        declared = record.get("declared_joint_count")
        if isinstance(declared, bool) or not isinstance(declared, int) or declared < 0:
            raise ValueError(f"Table 3 declared_joint_count invalid for {key}")
        rows[key] = {
            "declared_joint_count": declared,
            "urdf_sha256": record.get("urdf_sha256"),
            "package_content_manifest_sha256": record.get("package_content_manifest_sha256"),
            "selection_index": record.get("selection_index"),
        }
    return rows


def load_cohort(
    dataset_root: Path,
    cohort_manifest: Path,
    category_records_root: Path,
    table3_records_path: Path,
    *,
    formal: bool,
) -> dict[str, Any]:
    dataset_root = dataset_root.resolve(strict=True)
    release_root = dataset_root / "released_urdf"
    if not release_root.is_dir():
        raise FileNotFoundError(f"release root missing: {release_root}")
    cohort_file_hash = sha256_file(cohort_manifest)
    cohort = json.loads(cohort_manifest.read_text(encoding="utf-8"))
    if formal:
        if cohort_file_hash != FORMAL_COHORT_FILE_SHA256:
            raise RuntimeError("formal cohort manifest file SHA256 mismatch")
        if cohort.get("manifest_content_sha256") != FORMAL_COHORT_CONTENT_SHA256:
            raise RuntimeError("formal cohort manifest content SHA256 mismatch")
    raw_records = cohort.get("records")
    if not isinstance(raw_records, list) or not raw_records:
        raise ValueError("cohort manifest has no records")
    if formal and len(raw_records) != FORMAL_N_EVAL:
        raise RuntimeError(f"formal cohort must contain exactly {FORMAL_N_EVAL} records")
    source = cohort.get("source", {})
    selection = cohort.get("selection", {})
    release_ids = sorted(path.name for path in release_root.iterdir() if path.is_dir())
    if formal:
        if source.get("release_asset_ids_sha256") != FORMAL_RELEASE_ASSET_IDS_SHA256:
            raise RuntimeError("formal release asset ID SHA256 mismatch")
        if source.get("release_asset_count") != FORMAL_N_RELEASE:
            raise RuntimeError("formal release asset count mismatch")
        if len(release_ids) != FORMAL_N_RELEASE:
            raise RuntimeError("formal release roster size mismatch")
        if canonical_sha256(release_ids) != FORMAL_RELEASE_ASSET_IDS_SHA256:
            raise RuntimeError("formal release roster disk hash mismatch")
        if selection.get("selected_asset_ids_sha256") != FORMAL_SELECTED_ASSET_IDS_SHA256:
            raise RuntimeError("formal selected asset ID SHA256 mismatch")
        if int(selection.get("seed", -1)) != FORMAL_SEED:
            raise RuntimeError("formal cohort seed mismatch")
    table3_rows = _load_table3_records(table3_records_path, formal)
    assets: list[dict[str, Any]] = []
    category_rows: list[dict[str, str]] = []
    seen_packages: set[str] = set()
    for index, raw in enumerate(raw_records):
        asset_id = str(raw.get("asset_id", ""))
        if not asset_id or "/" in asset_id or "\\" in asset_id or asset_id in {".", ".."}:
            raise ValueError(f"invalid Articraft-10K asset ID at index {index}: {asset_id!r}")
        expected_package = release_root / asset_id
        package = Path(str(raw.get("package", ""))).resolve(strict=True)
        if package != expected_package.resolve(strict=True) or package.name != asset_id:
            raise ValueError(f"Table 2 package path mismatch: {asset_id}")
        package_text = str(package)
        if package_text in seen_packages:
            raise ValueError(f"duplicate Table 2 package: {package}")
        seen_packages.add(package_text)
        binding = _package_binding(package)
        if binding != raw.get("package_binding"):
            raise ValueError(f"Table 2 package binding mismatch: {asset_id}")
        urdf_path = package / URDF_RELATIVE_PATH
        if urdf_path.is_symlink() or not urdf_path.is_file():
            raise ValueError(f"{URDF_RELATIVE_PATH} is missing or symlinked: {asset_id}")
        urdf_hash = sha256_file(urdf_path)
        if urdf_hash != raw.get("model_urdf_sha256"):
            raise ValueError(f"{URDF_RELATIVE_PATH} hash mismatch: {asset_id}")
        table3_row = table3_rows.get(asset_id)
        if table3_row is None:
            raise ValueError(f"Table 3 record missing for cohort asset: {asset_id}")
        if table3_row["urdf_sha256"] != urdf_hash:
            raise ValueError(f"Table 3 urdf binding mismatch: {asset_id}")
        if table3_row["package_content_manifest_sha256"] != binding["content_manifest_sha256"]:
            raise ValueError(f"Table 3 package binding mismatch: {asset_id}")
        category = _read_category(category_records_root, asset_id)
        category_rows.append({"asset_id": asset_id, "category_slug": category})
        selection_hash = hashlib.sha256(
            "\0".join((SELECTION_PROTOCOL, cohort_file_hash, str(index), package_text)).encode("utf-8")
        ).hexdigest()
        assets.append(
            {
                "asset_key": asset_id,
                "asset_id": asset_id,
                "raw_category": category,
                "category": category,
                "selection_index": index,
                "selection_rank": index + 1,
                "selection_hash": selection_hash,
                "package": package_text,
                "package_binding": binding,
                "package_content_manifest_sha256": binding["content_manifest_sha256"],
                "urdf_path": str(urdf_path.resolve(strict=True)),
                "urdf_sha256": urdf_hash,
                "expected_movable_joint_count": table3_row["declared_joint_count"],
            }
        )
    selected_asset_ids = [row["asset_id"] for row in assets]
    if formal and canonical_sha256(selected_asset_ids) != FORMAL_SELECTED_ASSET_IDS_SHA256:
        raise RuntimeError("formal selected asset ID list hash mismatch")
    category_mapping_hash = canonical_sha256(category_rows)
    eval_category_count = len({row["category"] for row in assets})
    category_revision = _category_revision(category_records_root)
    if formal:
        if category_mapping_hash != FORMAL_SELECTED_CATEGORY_MAPPING_SHA256:
            raise RuntimeError("formal selected category mapping SHA256 mismatch")
        if eval_category_count != FORMAL_EVAL_CATEGORY_COUNT:
            raise RuntimeError("formal cohort must cover exactly 222 categories")
        if category_revision != FORMAL_CATEGORY_RECORDS_REVISION:
            raise RuntimeError("formal category-record revision mismatch")
    return {
        "dataset_root": str(dataset_root),
        "release_root": str(release_root),
        "cohort_manifest_path": str(cohort_manifest),
        "cohort_manifest_file_sha256": cohort_file_hash,
        "cohort_manifest_content_sha256": cohort.get("manifest_content_sha256"),
        "source_repo_id": source.get("repo_id"),
        "source_revision": source.get("revision"),
        "release_asset_ids_sha256": source.get("release_asset_ids_sha256"),
        "category_records_root": str(category_records_root),
        "category_records_revision": category_revision,
        "category_mapping_sha256": category_mapping_hash,
        "release_category_count": FORMAL_RELEASE_CATEGORY_COUNT if formal else None,
        "eval_category_count": eval_category_count,
        "table3_records_path": str(table3_records_path),
        "table3_records_sha256": sha256_file(table3_records_path),
        "selected_asset_ids_sha256": canonical_sha256(selected_asset_ids),
        "seed": int(selection.get("seed")) if selection.get("seed") is not None else None,
        "n_release": len(release_ids),
        "assets": assets,
    }


def _config(args: argparse.Namespace) -> dict[str, Any]:
    return {
        "protocol_id": PROTOCOL_ID,
        "urdf_relative_path": URDF_RELATIVE_PATH,
        "metrics": [
            "visual_bearing_collision_coverage",
            "joint_limit_portability",
            "joint_dynamics_coverage",
            "placeholder_mass_incidence",
        ],
        "asset_denominator_policy": "all frozen selected assets, including failures and exceptions",
        "joint_denominator_policy": "frozen declared non-fixed joints (J_eval); extraction failures stay fail closed",
        "visual_bearing_link_policy": (
            "asset passes iff parseable, >=1 link declares <visual> geometry, and every visual-bearing "
            "link has >=1 resource-resolvable loadable collision geometry; zero visual-bearing links fail closed"
        ),
        "joint_limit_portability_mapping": {
            "revolute": "bounded: finite lower<upper, finite non-negative effort, finite positive velocity",
            "prismatic": "bounded: finite lower<upper, finite non-negative effort, finite positive velocity",
            "continuous": "no finite lower/upper required; finite non-negative effort and finite positive velocity",
            "other": "unsupported mapping; retained as failure in denominator",
        },
        "joint_dynamics_coverage_policy": "finite non-negative damping AND friction declared on the movable joint",
        "placeholder_mass_registry": PLACEHOLDER_MASS_REGISTRY,
        "placeholder_mass_registry_policy": PLACEHOLDER_REGISTRY_POLICY,
        "dof_bins": list(DOF_BINS),
        "link_count_bin_note": "link-count bins reuse frozen DoF bin edges; labelled as such in breakdown",
        "workers": args.workers,
        "asset_timeout_seconds": args.asset_timeout_seconds,
    }


def build_manifest(args: argparse.Namespace, loaded: dict[str, Any]) -> dict[str, Any]:
    selected = loaded["assets"] if args.limit is None else loaded["assets"][: args.limit]
    config = _config(args)
    environment = _environment()
    manifest: dict[str, Any] = {
        "schema_version": 1,
        "dataset": DATASET_NAME,
        "table": "table2_supplementary",
        "classification": "FORMAL" if args.mode == "formal" else "NON_FORMAL_SMOKE",
        "created_at": utc_now(),
        "source": {
            "dataset_root": loaded["dataset_root"],
            "release_root": loaded["release_root"],
            "n_release": loaded["n_release"],
            "release_asset_ids_sha256": loaded["release_asset_ids_sha256"],
            "source_repo_id": loaded["source_repo_id"],
            "source_revision": loaded["source_revision"],
            "cohort_manifest_path": loaded["cohort_manifest_path"],
            "cohort_manifest_file_sha256": loaded["cohort_manifest_file_sha256"],
            "cohort_manifest_content_sha256": loaded["cohort_manifest_content_sha256"],
            "cohort_asset_count": len(loaded["assets"]),
            "category_records_root": loaded["category_records_root"],
            "category_records_revision": loaded["category_records_revision"],
            "category_mapping_policy": "exact asset_id join to official record.json category_slug",
            "category_mapping_sha256": loaded["category_mapping_sha256"],
            "release_category_count": loaded["release_category_count"],
            "eval_category_count": loaded["eval_category_count"],
            "table3_records_path": loaded["table3_records_path"],
            "table3_records_sha256": loaded["table3_records_sha256"],
            "expected_joint_denominator_binding": "Table 3 declared_joint_count per asset (fail-closed intended)",
        },
        "selection": {
            "algorithm": "exact existing Table 2 manifest .records[].package order; optional smoke prefix only",
            "source_protocol": SELECTION_PROTOCOL,
            "seed": loaded["seed"],
            "cohort_type": "FROZEN_RANDOM_SAMPLE_NOT_CATEGORY_BALANCED",
            "requested_limit": args.limit,
            "n_eval": len(selected),
            "selected_asset_ids_sha256": canonical_sha256([row["asset_id"] for row in selected]),
            "selected_packages_sha256": canonical_sha256([row["package"] for row in selected]),
            "selection_order_preserved": True,
            "outcome_based_reselection": False,
        },
        "evaluation": {
            "protocol_path": str(PROTOCOL_PATH.resolve(strict=True)),
            "protocol_sha256": sha256_file(PROTOCOL_PATH.resolve(strict=True)),
            "adapter_path": str(SCRIPT_PATH),
            "adapter_sha256": sha256_file(SCRIPT_PATH),
            "static_atoms_path": str(STATIC_ATOMS_PATH.resolve(strict=True)),
            "static_atoms_sha256": sha256_file(STATIC_ATOMS_PATH.resolve(strict=True)),
            "static_atoms_note": "same-version Table 2 supplementary atom module shared with LAM supplementary run",
            "config": config,
            "config_sha256": canonical_sha256(config),
            "environment": environment,
            "environment_sha256": canonical_sha256(environment),
        },
        "records": selected,
    }
    manifest["manifest_content_sha256"] = canonical_sha256(
        {key: value for key, value in manifest.items() if key != "manifest_content_sha256"}
    )
    return manifest


def failed_atom_record(job: dict[str, Any], issue: str, *, status: str = "error") -> dict[str, Any]:
    """Fail-closed record shape mirroring the static atoms failed record."""

    intended = int(job.get("expected_movable_joint_count") or 0)
    return {
        "schema_version": static_atoms.SCHEMA_VERSION,
        "asset_id": job["asset_key"],
        "package": job["package"],
        "urdf_relative_path": URDF_RELATIVE_PATH,
        "urdf_sha256": job.get("urdf_sha256"),
        "status": status,
        "parse": {"success": False, "issues": [issue]},
        "table2_supplementary": {
            "visual_bearing_collision_coverage": {
                "status": "NOT_EVALUABLE",
                "asset_intended": 1,
                "asset_passed": 0,
                "asset_pass": False,
                "visual_bearing_links_declared": 0,
                "covered_visual_bearing_links": 0,
                "link_extraction_complete": False,
                "collision_elements_declared_on_visual_links": 0,
                "loadable_collision_elements_on_visual_links": 0,
                "link_records": [],
                "issues": [issue],
            },
            "joint_limit_portability": {
                "status": "NOT_EVALUABLE",
                "joints_intended": intended,
                "joints_extracted": 0,
                "joints_passed": 0,
                "extraction_complete": False,
                "joint_records": [],
                "issues": [issue],
            },
            "joint_dynamics_coverage": {
                "status": "NOT_EVALUABLE",
                "joints_intended": intended,
                "joints_extracted": 0,
                "joints_covered": 0,
                "extraction_complete": False,
                "joint_records": [],
                "issues": [issue],
            },
            "placeholder_mass_incidence": {
                "status": "N/E",
                "dynamic_link_policy": "all_declared_links",
                "dynamic_links": 0,
                "complete_inertial_links": 0,
                "complete_inertial_coverage_numerator": 0,
                "complete_inertial_coverage_denominator": 0,
                "classified_complete_inertial_links": 0,
                "unclassified_complete_inertial_links": 0,
                "placeholder_links": None,
                "incidence_numerator": None,
                "incidence_denominator": 0,
                "registry_ids": [],
                "link_records": [],
                "incomplete_inertial_links": [],
                "issues": [issue],
            },
        },
        "resource_closure": {
            "status": "NOT_EVALUABLE",
            "complete": False,
            "file_count": 0,
            "sha256": None,
            "files": [],
            "issues": [issue],
        },
        "s1_evidence": {"status": "NOT_EVALUABLE", "issues": [issue]},
        "issues": [issue],
    }


def _bind_record(record: dict[str, Any], job: dict[str, Any], manifest_hash: str) -> dict[str, Any]:
    record.update(
        {
            "asset_key": job["asset_key"],
            "asset_id": job["asset_id"],
            "category": job["category"],
            "raw_category": job["raw_category"],
            "selection_index": job["selection_index"],
            "selection_rank": job["selection_rank"],
            "selection_hash": job["selection_hash"],
            "package": job["package"],
            "package_content_manifest_sha256": job["package_content_manifest_sha256"],
            "expected_movable_joint_count": job["expected_movable_joint_count"],
            "manifest_content_sha256": manifest_hash,
            "completed_at": utc_now(),
        }
    )
    return record


def run_internal_job(job_path: Path, result_path: Path) -> int:
    job = json.loads(job_path.resolve(strict=True).read_text(encoding="utf-8"))
    urdf_path = Path(job["urdf_path"])
    package = Path(job["package"])
    failure: str | None = None
    if not urdf_path.is_file() or urdf_path.is_symlink():
        failure = "selected URDF is missing"
    elif sha256_file(urdf_path) != job["urdf_sha256"]:
        failure = "selected URDF changed after freeze"
    elif _package_binding(package) != job["package_binding"]:
        failure = "selected package changed after freeze"
    if failure is not None:
        atomic_write_json(result_path, failed_atom_record(job, failure))
        return 0
    record = static_atoms.audit_lam_package(
        package,
        urdf_relative_path=URDF_RELATIVE_PATH,
        asset_id=job["asset_key"],
        expected_movable_joints=int(job["expected_movable_joint_count"]),
        placeholder_registry=PLACEHOLDER_MASS_REGISTRY,
    )
    atomic_write_json(result_path, record)
    return 0


def _execute_job(job: dict[str, Any], scratch: Path, timeout_seconds: float, manifest_hash: str) -> dict[str, Any]:
    job_path = scratch / f"job_{job['selection_rank']:04d}.json"
    result_path = scratch / f"result_{job['selection_rank']:04d}.json"
    atomic_write_json(job_path, {**job, "manifest_content_sha256": manifest_hash})
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
            failed_atom_record(job, f"asset timeout after {timeout_seconds:g} seconds", status="timeout"),
            job,
            manifest_hash,
        )
    if process.returncode != 0 or not result_path.is_file():
        detail = stderr.decode("utf-8", errors="replace")[-4000:]
        return _bind_record(
            failed_atom_record(job, f"worker failed with exit {process.returncode}: {detail}"),
            job,
            manifest_hash,
        )
    result = json.loads(result_path.read_text(encoding="utf-8"))
    if result.get("asset_id") != job["asset_key"]:
        return _bind_record(failed_atom_record(job, "worker result asset binding mismatch"), job, manifest_hash)
    return _bind_record(result, job, manifest_hash)


def _rate(numerator: int | None, denominator: int) -> float | None:
    if numerator is None or denominator <= 0:
        return None
    return numerator / denominator


def _dof_bin(count: int) -> str:
    if count <= 0:
        return "0"
    if count == 1:
        return "1"
    if count <= 3:
        return "2-3"
    if count <= 7:
        return "4-7"
    return ">=8"


def aggregate_records(records: list[dict[str, Any]], n_eval: int) -> dict[str, Any]:
    """Aggregate Table 2 supplementary metrics (micro, category macro, breakdowns)."""

    if len(records) != n_eval:
        raise ValueError(f"record count {len(records)} != n_eval {n_eval}")

    vbcc_passed_assets = 0
    link_declared_total = 0
    link_covered_total = 0
    link_extraction_complete_assets = 0
    portability_passed = 0
    dynamics_covered = 0
    joints_intended_total = 0
    joints_extracted_total = 0
    joint_extraction_complete_assets = 0
    complete_inertial_total = 0
    dynamic_links_total = 0
    complete_inertial_asset_coverage_pairs = 0
    status_counts: Counter[str] = Counter()
    per_asset: list[dict[str, Any]] = []
    joint_type_portability: dict[str, dict[str, int]] = defaultdict(lambda: {"passed": 0, "total": 0})
    joint_type_dynamics: dict[str, dict[str, int]] = defaultdict(lambda: {"passed": 0, "total": 0})

    for record in records:
        status_counts[str(record.get("status"))] += 1
        t2s = record.get("table2_supplementary", {})
        vbcc = t2s.get("visual_bearing_collision_coverage", {})
        port = t2s.get("joint_limit_portability", {})
        dyn = t2s.get("joint_dynamics_coverage", {})
        placeholder = t2s.get("placeholder_mass_incidence", {})

        asset_pass = bool(vbcc.get("asset_pass")) and str(record.get("status")) == "completed"
        vbcc_passed_assets += int(asset_pass)
        link_declared_total += int(vbcc.get("visual_bearing_links_declared") or 0)
        link_covered_total += int(vbcc.get("covered_visual_bearing_links") or 0)
        if bool(vbcc.get("link_extraction_complete")):
            link_extraction_complete_assets += 1

        intended = int(port.get("joints_intended") or 0)
        extracted = int(port.get("joints_extracted") or 0)
        joints_intended_total += intended
        joints_extracted_total += extracted
        portability_passed += int(port.get("joints_passed") or 0)
        dynamics_covered += int(dyn.get("joints_covered") or 0)
        if bool(port.get("extraction_complete")):
            joint_extraction_complete_assets += 1
        for joint_record in port.get("joint_records", []):
            joint_type = str(joint_record.get("joint_type") or "unknown")
            joint_type_portability[joint_type]["total"] += 1
            joint_type_portability[joint_type]["passed"] += int(bool(joint_record.get("limit_portability_pass")))
        for joint_record in dyn.get("joint_records", []):
            joint_type = str(joint_record.get("joint_type") or "unknown")
            joint_type_dynamics[joint_type]["total"] += 1
            joint_type_dynamics[joint_type]["passed"] += int(bool(joint_record.get("covered")))

        complete_inertial_total += int(placeholder.get("complete_inertial_links") or 0)
        dynamic_links_total += int(placeholder.get("dynamic_links") or 0)
        if int(placeholder.get("complete_inertial_coverage_denominator") or 0) > 0:
            complete_inertial_asset_coverage_pairs += 1

        per_asset.append(
            {
                "asset_key": record.get("asset_key"),
                "category": record.get("category"),
                "status": record.get("status"),
                "vbcc_pass": asset_pass,
                "joints_intended": intended,
                "joints_portable": int(port.get("joints_passed") or 0),
                "joints_dynamics": int(dyn.get("joints_covered") or 0),
                "declared_link_count": int(placeholder.get("dynamic_links") or 0),
                "complete_inertial_links": int(placeholder.get("complete_inertial_links") or 0),
            }
        )

    j_eval = joints_intended_total
    vbcc_status = (
        "COMPLETE"
        if link_extraction_complete_assets == n_eval and joint_extraction_complete_assets == n_eval
        else "PARTIAL"
    )

    by_category: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in per_asset:
        by_category[str(row["category"])].append(row)
    category_rows: dict[str, dict[str, Any]] = {}
    vbcc_macro_values: list[float] = []
    port_macro_values: list[float] = []
    dyn_macro_values: list[float] = []
    for category in sorted(by_category):
        rows = by_category[category]
        cat_vbcc_passed = sum(int(row["vbcc_pass"]) for row in rows)
        cat_vbcc_rate = _rate(cat_vbcc_passed, len(rows))
        cat_intended = sum(int(row["joints_intended"]) for row in rows)
        cat_port = sum(int(row["joints_portable"]) for row in rows)
        cat_dyn = sum(int(row["joints_dynamics"]) for row in rows)
        cat_port_rate = _rate(cat_port, cat_intended)
        cat_dyn_rate = _rate(cat_dyn, cat_intended)
        if cat_vbcc_rate is not None:
            vbcc_macro_values.append(cat_vbcc_rate)
        if cat_port_rate is not None:
            port_macro_values.append(cat_port_rate)
        if cat_dyn_rate is not None:
            dyn_macro_values.append(cat_dyn_rate)
        category_rows[category] = {
            "assets": len(rows),
            "joints_intended": cat_intended,
            "visual_bearing_collision_coverage": {
                "passed": cat_vbcc_passed,
                "denominator": len(rows),
                "rate": cat_vbcc_rate,
            },
            "joint_limit_portability": {
                "passed": cat_port,
                "denominator": cat_intended,
                "rate": cat_port_rate,
            },
            "joint_dynamics_coverage": {
                "passed": cat_dyn,
                "denominator": cat_intended,
                "rate": cat_dyn_rate,
            },
            "placeholder_mass_incidence": {
                "status": "N/E",
                "reason": "placeholder registry is empty; no mass values are evaluable",
            },
        }

    def _macro(values: list[float]) -> float | None:
        return sum(values) / len(values) if values else None

    dof_bins: dict[str, dict[str, Any]] = {}
    link_bins: dict[str, dict[str, Any]] = {}
    for name in DOF_BINS:
        subset = [row for row in per_asset if _dof_bin(int(row["joints_intended"])) == name]
        subset_intended = sum(int(row["joints_intended"]) for row in subset)
        subset_vbcc = sum(int(row["vbcc_pass"]) for row in subset)
        subset_port = sum(int(row["joints_portable"]) for row in subset)
        subset_dyn = sum(int(row["joints_dynamics"]) for row in subset)
        dof_bins[name] = {
            "assets": len(subset),
            "joints_intended": subset_intended,
            "visual_bearing_collision_coverage": {
                "passed": subset_vbcc,
                "denominator": len(subset),
                "rate": _rate(subset_vbcc, len(subset)),
            },
            "joint_limit_portability": {
                "passed": subset_port,
                "denominator": subset_intended,
                "rate": _rate(subset_port, subset_intended),
            },
            "joint_dynamics_coverage": {
                "passed": subset_dyn,
                "denominator": subset_intended,
                "rate": _rate(subset_dyn, subset_intended),
            },
        }
        link_subset = [row for row in per_asset if _dof_bin(int(row["declared_link_count"])) == name]
        link_vbcc = sum(int(row["vbcc_pass"]) for row in link_subset)
        link_bins[name] = {
            "assets": len(link_subset),
            "visual_bearing_collision_coverage": {
                "passed": link_vbcc,
                "denominator": len(link_subset),
                "rate": _rate(link_vbcc, len(link_subset)),
            },
        }

    joint_type_breakdown = {
        "joint_limit_portability": {
            joint_type: {
                "passed": counts["passed"],
                "denominator": counts["total"],
                "rate": _rate(counts["passed"], counts["total"]),
            }
            for joint_type, counts in sorted(joint_type_portability.items())
        },
        "joint_dynamics_coverage": {
            joint_type: {
                "passed": counts["passed"],
                "denominator": counts["total"],
                "rate": _rate(counts["passed"], counts["total"]),
            }
            for joint_type, counts in sorted(joint_type_dynamics.items())
        },
    }

    return {
        "n_eval": n_eval,
        "j_eval": j_eval,
        "status_counts": dict(sorted(status_counts.items())),
        "metrics": {
            "visual_bearing_collision_coverage": {
                "passed": vbcc_passed_assets,
                "denominator": n_eval,
                "rate": _rate(vbcc_passed_assets, n_eval),
                "link_micro_covered": link_covered_total,
                "link_micro_denominator": link_declared_total,
                "link_micro_rate": _rate(link_covered_total, link_declared_total),
                "link_extraction_coverage": {
                    "complete_assets": link_extraction_complete_assets,
                    "denominator": n_eval,
                    "rate": _rate(link_extraction_complete_assets, n_eval),
                },
                "status": vbcc_status,
            },
            "joint_limit_portability": {
                "passed": portability_passed,
                "denominator": j_eval,
                "rate": _rate(portability_passed, j_eval),
                "joints_extracted": joints_extracted_total,
                "extraction_complete_assets": joint_extraction_complete_assets,
            },
            "joint_dynamics_coverage": {
                "passed": dynamics_covered,
                "denominator": j_eval,
                "rate": _rate(dynamics_covered, j_eval),
            },
            "placeholder_mass_incidence": {
                "status": "N/E",
                "placeholder": None,
                "denominator": 0,
                "rate": None,
                "reason": "placeholder registry is empty; no mass values are evaluable",
                "registry_ids": [],
                "complete_inertial_coverage": {
                    "complete_inertial_links": complete_inertial_total,
                    "dynamic_links": dynamic_links_total,
                    "rate": _rate(complete_inertial_total, dynamic_links_total),
                    "assets_with_declared_links": complete_inertial_asset_coverage_pairs,
                },
            },
        },
        "category_macro": {
            "category_count": len(by_category),
            "visual_bearing_collision_coverage": {
                "macro_rate": _macro(vbcc_macro_values),
                "categories": len(vbcc_macro_values),
            },
            "joint_limit_portability": {
                "macro_rate": _macro(port_macro_values),
                "categories": len(port_macro_values),
            },
            "joint_dynamics_coverage": {
                "macro_rate": _macro(dyn_macro_values),
                "categories": len(dyn_macro_values),
            },
            "placeholder_mass_incidence": {
                "status": "N/E",
                "reason": "placeholder registry is empty; no mass values are evaluable",
            },
            "categories": category_rows,
        },
        "breakdown": {
            "declared_dof_bins": dof_bins,
            "declared_link_count_bins_same_edges_as_dof_bins": link_bins,
            "joint_type": joint_type_breakdown,
        },
    }


def _fmt_rate(cell: dict[str, Any]) -> str:
    rate = cell.get("rate")
    if rate is None:
        return "N/A"
    return f"{100 * rate:.2f}%"


def _summary_markdown(summary: dict[str, Any], manifest: dict[str, Any]) -> str:
    metrics = summary["metrics"]
    vbcc = metrics["visual_bearing_collision_coverage"]
    port = metrics["joint_limit_portability"]
    dyn = metrics["joint_dynamics_coverage"]
    placeholder = metrics["placeholder_mass_incidence"]
    lines = [
        "# Articraft-10K Table 2 Supplementary Diagnostics",
        "",
        f"Run classification: **{manifest['classification']}**.",
        "",
        (
            f"Exact Table 2 manifest cohort: N_eval={summary['n_eval']} from "
            f"N_release={manifest['source']['n_release']}, seed={manifest['selection']['seed']}; "
            f"J_eval={summary['j_eval']}. Existing package order was preserved without resampling."
        ),
        "",
        "| Metric | Result |",
        "|---|---:|",
        f"| visual_bearing_collision_coverage (asset) | {vbcc['passed']} / {vbcc['denominator']} ({_fmt_rate(vbcc)}) |",
        (
            f"| visual_bearing_collision_coverage (link-micro) | "
            f"{vbcc['link_micro_covered']} / {vbcc['link_micro_denominator']} "
            f"({_fmt_rate({'rate': vbcc['link_micro_rate']})}) "
            f"[link extraction coverage {vbcc['link_extraction_coverage']['complete_assets']}"
            f"/{vbcc['link_extraction_coverage']['denominator']}; {vbcc['status']}] |"
        ),
        f"| joint_limit_portability | {port['passed']} / {port['denominator']} ({_fmt_rate(port)}) |",
        f"| joint_dynamics_coverage | {dyn['passed']} / {dyn['denominator']} ({_fmt_rate(dyn)}) |",
        f"| placeholder_mass_incidence | N/E ({placeholder['reason']}) |",
        (
            f"| placeholder_mass_incidence complete-inertial coverage | "
            f"{placeholder['complete_inertial_coverage']['complete_inertial_links']} / "
            f"{placeholder['complete_inertial_coverage']['dynamic_links']} "
            f"({_fmt_rate(placeholder['complete_inertial_coverage'])}) |"
        ),
        "",
    ]
    macro = summary["category_macro"]
    lines.extend(
        [
            (
                f"Category macro average over {macro['category_count']} observed categories "
                "(unweighted; joint metrics only over categories with >=1 declared movable joint):"
            ),
            "",
            "| Metric | Category macro |",
            "|---|---:|",
        ]
    )
    for key in ("visual_bearing_collision_coverage", "joint_limit_portability", "joint_dynamics_coverage"):
        value = macro[key]
        rate = value.get("macro_rate")
        text = "N/A" if rate is None else f"{100 * rate:.2f}%"
        lines.append(f"| {key} | {text} (categories={value['categories']}) |")
    lines.append("| placeholder_mass_incidence | N/E (registry empty) |")
    lines.extend(
        [
            "",
            "Declared-DoF bin breakdown (frozen bins 0 / 1 / 2-3 / 4-7 / >=8):",
            "",
            "| Bin | Assets | Joints | VBCC asset pass | Portability | Dynamics |",
            "|---|---:|---:|---:|---:|---:|",
        ]
    )
    for name in DOF_BINS:
        row = summary["breakdown"]["declared_dof_bins"][name]
        lines.append(
            f"| {name} | {row['assets']} | {row['joints_intended']} | "
            f"{row['visual_bearing_collision_coverage']['passed']} / {row['visual_bearing_collision_coverage']['denominator']} "
            f"({_fmt_rate(row['visual_bearing_collision_coverage'])}) | "
            f"{row['joint_limit_portability']['passed']} / {row['joint_limit_portability']['denominator']} "
            f"({_fmt_rate(row['joint_limit_portability'])}) | "
            f"{row['joint_dynamics_coverage']['passed']} / {row['joint_dynamics_coverage']['denominator']} "
            f"({_fmt_rate(row['joint_dynamics_coverage'])}) |"
        )
    lines.extend(
        [
            "",
            "Link-count bin breakdown uses the same bin edges as the frozen DoF bins (declared total links per asset):",
            "",
            "| Bin | Assets | VBCC asset pass |",
            "|---|---:|---:|",
        ]
    )
    for name in DOF_BINS:
        row = summary["breakdown"]["declared_link_count_bins_same_edges_as_dof_bins"][name]
        lines.append(
            f"| {name} | {row['assets']} | "
            f"{row['visual_bearing_collision_coverage']['passed']} / {row['visual_bearing_collision_coverage']['denominator']} "
            f"({_fmt_rate(row['visual_bearing_collision_coverage'])}) |"
        )
    lines.extend(["", "Joint-type breakdown:", "", "| Joint type | Portability | Dynamics |", "|---|---:|---:|"])
    port_types = summary["breakdown"]["joint_type"]["joint_limit_portability"]
    dyn_types = summary["breakdown"]["joint_type"]["joint_dynamics_coverage"]
    for joint_type in sorted(set(port_types) | set(dyn_types)):
        port_cell = port_types.get(joint_type, {"passed": 0, "denominator": 0, "rate": None})
        dyn_cell = dyn_types.get(joint_type, {"passed": 0, "denominator": 0, "rate": None})
        lines.append(
            f"| {joint_type} | {port_cell['passed']} / {port_cell['denominator']} ({_fmt_rate(port_cell)}) | "
            f"{dyn_cell['passed']} / {dyn_cell['denominator']} ({_fmt_rate(dyn_cell)}) |"
        )
    lines.extend(
        [
            "",
            (
                "These are proposed Table 2 supplementary diagnostics; they do not retroactively change "
                "the frozen Table 2 Strict URDF Pass values."
            ),
            "",
        ]
    )
    return "\n".join(lines)


def _prepare_output(args: argparse.Namespace, n_eval: int) -> Path:
    if args.output is None:
        from datetime import datetime, timezone

        timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        if args.mode == "formal":
            args.output = (
                DEFAULT_OUTPUT_PARENT
                / f"urdf_table2sup_articraft10k_table2_n{n_eval}_seed{FORMAL_SEED}_{timestamp}"
            )
        else:
            args.output = DEFAULT_OUTPUT_PARENT / f"urdf_table2sup_articraft10k_smoke_n{n_eval}_{timestamp}"
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
    loaded = load_cohort(
        args.dataset_root,
        args.cohort_manifest,
        args.category_records_root,
        args.table3_records,
        formal=args.mode == "formal",
    )
    n_eval = len(loaded["assets"]) if args.limit is None else args.limit
    output = _prepare_output(args, n_eval)
    manifest_path = output / "manifest.json"
    records_path = output / "asset_records.jsonl"
    checkpoint_path = output / "checkpoint.json"
    snapshot_path = output / "protocol_snapshot.md"
    scratch = output / ".worker_scratch"
    scratch.mkdir(exist_ok=True)
    if args.resume:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if manifest.get("manifest_content_sha256") != canonical_sha256(
            {key: value for key, value in manifest.items() if key != "manifest_content_sha256"}
        ):
            raise RuntimeError("resume manifest self-hash mismatch")
        fresh = build_manifest(args, loaded)
        for field in ("source", "selection", "evaluation", "records"):
            if fresh[field] != manifest[field]:
                raise RuntimeError(f"resume binding mismatch: {field}")
    else:
        manifest = build_manifest(args, loaded)
        shutil.copyfile(PROTOCOL_PATH, snapshot_path)
        manifest["evaluation"]["protocol_snapshot_path"] = str(snapshot_path)
        manifest["evaluation"]["protocol_snapshot_sha256"] = sha256_file(snapshot_path)
        manifest["manifest_content_sha256"] = canonical_sha256(
            {key: value for key, value in manifest.items() if key != "manifest_content_sha256"}
        )
        atomic_write_json(manifest_path, manifest)
        records_path.touch(exist_ok=False)
        atomic_write_json(
            checkpoint_path,
            {
                "state": "frozen",
                "completed": 0,
                "remaining": n_eval,
                "n_eval": n_eval,
                "manifest_content_sha256": manifest["manifest_content_sha256"],
                "updated_at": utc_now(),
            },
        )
    manifest_hash = manifest["manifest_content_sha256"]
    records = load_jsonl(records_path)
    selected_keys = [row["asset_key"] for row in manifest["records"]]
    by_key: dict[str, dict[str, Any]] = {}
    for record in records:
        key = record.get("asset_key")
        if key not in selected_keys or key in by_key:
            raise RuntimeError(f"invalid or duplicate resume record: {key!r}")
        if record.get("manifest_content_sha256") != manifest_hash:
            raise RuntimeError(f"resume record manifest binding mismatch: {key}")
        by_key[key] = record
    pending = [row for row in manifest["records"] if row["asset_key"] not in by_key]
    with ThreadPoolExecutor(max_workers=args.workers) as executor:
        futures = {
            executor.submit(_execute_job, row, scratch, args.asset_timeout_seconds, manifest_hash): row
            for row in pending
        }
        for future in as_completed(futures):
            job = futures[future]
            try:
                record = future.result()
            except Exception as exc:  # noqa: BLE001
                record = _bind_record(
                    failed_atom_record(job, f"parent worker exception: {type(exc).__name__}: {exc}"),
                    job,
                    manifest_hash,
                )
            by_key[record["asset_key"]] = record
            append_jsonl(records_path, record)
            completed = len(by_key)
            atomic_write_json(
                checkpoint_path,
                {
                    "state": "running" if completed < n_eval else "aggregating",
                    "completed": completed,
                    "remaining": n_eval - completed,
                    "n_eval": n_eval,
                    "last_completed_asset_key": record["asset_key"],
                    "manifest_content_sha256": manifest_hash,
                    "updated_at": utc_now(),
                },
            )
            print(f"[{completed}/{n_eval}] {record['asset_key']} {record.get('status')}", flush=True)
    ordered = [by_key[key] for key in selected_keys]
    summary = aggregate_records(ordered, n_eval)
    summary.update(
        {
            "schema_version": 1,
            "status": "completed",
            "classification": manifest["classification"],
            "dataset": DATASET_NAME,
            "table": "table2_supplementary",
            "manifest_content_sha256": manifest_hash,
            "completed_at": utc_now(),
        }
    )
    atomic_write_json(output / "summary.json", summary)
    atomic_write_text(output / "summary.md", _summary_markdown(summary, manifest))
    atomic_write_json(
        checkpoint_path,
        {
            "state": "complete",
            "completed": n_eval,
            "remaining": 0,
            "n_eval": n_eval,
            "manifest_content_sha256": manifest_hash,
            "updated_at": utc_now(),
        },
    )
    return output


def validate_contract(args: argparse.Namespace) -> None:
    if args.workers <= 0 or args.asset_timeout_seconds <= 0:
        raise ValueError("workers/timeout must be positive")
    if args.limit is not None and not 0 < args.limit <= FORMAL_N_EVAL:
        raise ValueError("limit must be in [1, 800]")
    if args.mode == "formal":
        if (
            args.limit is not None
            or args.workers != DEFAULT_WORKERS
            or args.asset_timeout_seconds != DEFAULT_TIMEOUT_SECONDS
            or args.dataset_root.resolve(strict=False) != DEFAULT_DATASET_ROOT.resolve(strict=False)
            or args.cohort_manifest.resolve(strict=False) != DEFAULT_COHORT_MANIFEST.resolve(strict=False)
            or args.category_records_root.resolve(strict=False)
            != DEFAULT_CATEGORY_RECORDS_ROOT.resolve(strict=False)
            or args.table3_records.resolve(strict=False) != DEFAULT_TABLE3_RECORDS.resolve(strict=False)
        ):
            raise ValueError(
                "formal mode freezes canonical dataset/cohort/categories/table3 binding, N=800, workers=4, timeout=120"
            )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("formal", "smoke"), default="formal")
    parser.add_argument("--dataset-root", type=Path, default=DEFAULT_DATASET_ROOT)
    parser.add_argument("--cohort-manifest", type=Path, default=DEFAULT_COHORT_MANIFEST)
    parser.add_argument("--category-records-root", type=Path, default=DEFAULT_CATEGORY_RECORDS_ROOT)
    parser.add_argument("--table3-records", type=Path, default=DEFAULT_TABLE3_RECORDS)
    parser.add_argument("--limit", type=int, help="smoke mode: evaluate exact manifest prefix")
    parser.add_argument("--workers", type=int, default=DEFAULT_WORKERS)
    parser.add_argument("--asset-timeout-seconds", type=float, default=DEFAULT_TIMEOUT_SECONDS)
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
