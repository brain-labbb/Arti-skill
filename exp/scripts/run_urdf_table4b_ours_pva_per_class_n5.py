#!/usr/bin/env python3
"""Run exact Table 4b geometry metrics on the frozen Ours PV-A N=5 cohort.

The upstream Table 4 directory is supplied explicitly because its timestamped
formal receipt is created immediately before this run.  This adapter freezes
that actual receipt hash, binds every item back to the immutable per-class
cohort, and delegates the exact trimesh + rtree protocol to the shared Table 4b
runner.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
from pathlib import Path
import sys
import time
from typing import Any, Mapping, Sequence
import xml.etree.ElementTree as ET


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from exp.scripts import run_urdf_table4b_artiverse as base  # noqa: E402


SCHEMA_VERSION = "table4b-ours-pva-per-class-n5/v1"
PROTOCOL_ID = "table4b-ours-pva-per-class-n5-max-joints-v1"
SOURCE_PROTOCOL_ID = "urdf-sim-ready-table4-ours-per-class-n5-max-joints-v1"
SOURCE_SCHEMA_VERSION = "table4_ours_pva_per_class_n5_frozen_manifest_v1"
DATASET = "Ours per-class N=5 (supplementary)"
CLASSIFICATION = "FORMAL"
COHORT_MANIFEST = REPO / "exp/PV-A-per-class-n5-max-joints/manifest.json"
EXPECTED_COHORT_MANIFEST_SHA256 = (
    "e78f4b767023f8a5c1517d96bfab35a39482d6eee28238820a9b91ac3ea8d293"
)
EXPECTED_COHORT_CONTENT_SHA256 = (
    "eea55287dd70b710a7c03b11b16c6685208bbaa63cde925232293cb9012c8158"
)
EXPECTED_ORDERED_IDS_SHA256 = (
    "b5c9262eca8e65ede90c597a16e4ed2b0d7348b4eeb326cd64a06cea518c4178"
)
N_RELEASE = 302440
N_EVAL = 2655
EXPECTED_CATEGORY_COUNT = 531
PER_CLASS = 5
WORKERS = 16
CHILD_TIMEOUT_SECONDS = 900
FORMAL_OUTPUT_NAME = (
    "table4b_urdf_ours_pva_per_class_n5_max_joints_n2655_{timestamp}"
)
SMOKE_OUTPUT_NAME = (
    "table4b_urdf_ours_pva_per_class_n5_max_joints_smoke_n{n}_{timestamp}"
)
SUMMARY_TITLE = (
    "Table 4b - Ours per-class N=5 (supplementary; exact collision-"
    "representation geometry)"
)
SELECTION_POLICY = (
    "all 2,655 frozen per-class N=5 Table 4 items in existing order; "
    "no resampling, replacement, or result filtering"
)

SOURCE_MANIFEST: Path | None = None
EXPECTED_SOURCE_MANIFEST_SHA256 = ""
EXPECTED_SOURCE_CONTENT_SHA256 = ""
_SOURCE_BINDING: dict[str, Any] | None = None
_COHORT_ROWS: list[dict[str, Any]] | None = None
_FULL_RUN_STARTED_AT_UTC: str | None = None
_FULL_RUN_STARTED_PERF: float | None = None
_FULL_RUN_RESUME = False
_perf_counter = time.perf_counter

if not hasattr(base, "_ours_pva_original_failed_asset_record"):
    base._ours_pva_original_failed_asset_record = base._failed_asset_record
if not hasattr(base, "_ours_pva_original_run_child"):
    base._ours_pva_original_run_child = base.run_child
_base_failed_asset_record = base._ours_pva_original_failed_asset_record
_base_run_child = base._ours_pva_original_run_child


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat().replace("+00:00", "Z")


def canonical_sha256(value: Any) -> str:
    payload = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    )
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def ordered_ids_sha256(ids: Sequence[str]) -> str:
    payload = json.dumps(list(ids), separators=(",", ":"), ensure_ascii=True)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _manifest_self_hash(manifest: Mapping[str, Any]) -> str:
    payload = dict(manifest)
    payload.pop("manifest_content_sha256", None)
    return canonical_sha256(payload)


def _validate_package_binding(row: Mapping[str, Any]) -> Mapping[str, Any]:
    asset_id = str(row.get("dataset_id", ""))
    binding = row.get("package_binding")
    if not isinstance(binding, dict) or not isinstance(binding.get("files"), list):
        raise SystemExit(f"missing package binding: {asset_id}")
    files = binding["files"]
    if canonical_sha256(files) != binding.get("content_manifest_sha256"):
        raise SystemExit(f"package binding self-hash mismatch: {asset_id}")
    if binding.get("file_count") != len(files):
        raise SystemExit(f"package binding file count mismatch: {asset_id}")
    if binding.get("total_bytes") != sum(int(item["bytes"]) for item in files):
        raise SystemExit(f"package binding byte count mismatch: {asset_id}")
    urdf_rows = [item for item in files if item.get("path") == "model.urdf"]
    if len(urdf_rows) != 1 or urdf_rows[0].get("sha256") != row.get("urdf_sha256"):
        raise SystemExit(f"package binding URDF mismatch: {asset_id}")
    return binding


def _load_cohort_rows() -> list[dict[str, Any]]:
    if sha256_file(COHORT_MANIFEST) != EXPECTED_COHORT_MANIFEST_SHA256:
        raise SystemExit("cohort manifest sha256 mismatch")
    manifest = json.loads(COHORT_MANIFEST.read_text(encoding="utf-8"))
    if manifest.get("manifest_content_sha256") != _manifest_self_hash(manifest):
        raise SystemExit("cohort manifest self-hash mismatch")
    if manifest.get("manifest_content_sha256") != EXPECTED_COHORT_CONTENT_SHA256:
        raise SystemExit("cohort manifest content hash mismatch")
    expected_metadata = {
        "schema_version": "pva-per-class-extracted-cohort/v2",
        "protocol_id": "pva-per-class-n5-fence-ferris-max-movable-joints-v1",
        "dataset": "PV-A-per-class-n5",
        "classification": "FROZEN_MIXED_STRATIFIED_SAMPLE",
        "n_eval": N_EVAL,
        "class_count": EXPECTED_CATEGORY_COUNT,
        "per_class": PER_CLASS,
    }
    if any(manifest.get(key) != value for key, value in expected_metadata.items()):
        raise SystemExit("cohort manifest protocol metadata mismatch")
    assets = manifest.get("assets")
    if not isinstance(assets, list) or len(assets) != N_EVAL:
        raise SystemExit(f"cohort manifest must contain {N_EVAL} assets")
    ids = [str(row.get("dataset_id", "")) for row in assets]
    if len(set(ids)) != N_EVAL or ordered_ids_sha256(ids) != EXPECTED_ORDERED_IDS_SHA256:
        raise SystemExit("cohort ordered dataset IDs mismatch")
    categories: set[str] = set()
    rows: list[dict[str, Any]] = []
    for index, raw in enumerate(assets):
        if raw.get("selection_index") != index:
            raise SystemExit(f"cohort selection order mismatch at {index}")
        binding = _validate_package_binding(raw)
        package = Path(str(raw.get("package", ""))).resolve(strict=True)
        if str(raw.get("primary_urdf_relative_path")) != "model.urdf":
            raise SystemExit(f"cohort canonical URDF path mismatch: {ids[index]}")
        categories.add(str(raw.get("category")))
        rows.append(
            {
                "selection_index": index,
                "dataset_id": ids[index],
                "asset_id": str(raw.get("asset_id")),
                "category": str(raw.get("category")),
                "package": str(package),
                "urdf_sha256": str(raw.get("urdf_sha256")),
                "package_binding": binding,
            }
        )
    if len(categories) != EXPECTED_CATEGORY_COUNT:
        raise SystemExit("cohort category count mismatch")
    return rows


def _validate_source_manifest(
    path: Path, *, expected_file_sha256: str | None = None
) -> tuple[dict[str, Any], list[dict[str, Any]], str]:
    payload = path.read_bytes()
    file_sha256 = hashlib.sha256(payload).hexdigest()
    if expected_file_sha256 is not None and file_sha256 != expected_file_sha256:
        raise SystemExit("source manifest sha256 mismatch")
    manifest = json.loads(payload)
    if manifest.get("manifest_content_sha256") != _manifest_self_hash(manifest):
        raise SystemExit("source manifest self-hash mismatch")
    expected_metadata = {
        "schema_version": SOURCE_SCHEMA_VERSION,
        "protocol_id": SOURCE_PROTOCOL_ID,
        "dataset": DATASET,
        "classification": CLASSIFICATION,
    }
    if any(manifest.get(key) != value for key, value in expected_metadata.items()):
        raise SystemExit("source manifest protocol metadata mismatch")

    rows = _load_cohort_rows()
    source = manifest.get("source")
    if not isinstance(source, dict):
        raise SystemExit("source manifest cohort binding missing")
    expected_source = {
        "cohort_manifest_file_sha256": EXPECTED_COHORT_MANIFEST_SHA256,
        "cohort_manifest_content_sha256": EXPECTED_COHORT_CONTENT_SHA256,
        "n_release": N_RELEASE,
        "n_eval": N_EVAL,
        "release_category_count": EXPECTED_CATEGORY_COUNT,
        "eval_category_count": EXPECTED_CATEGORY_COUNT,
        "per_class": PER_CLASS,
        "per_item_package_paths": True,
    }
    if any(source.get(key) != value for key, value in expected_source.items()):
        raise SystemExit("source manifest cohort metadata mismatch")
    cohort_path = Path(str(source.get("cohort_manifest_path", ""))).resolve(strict=True)
    if cohort_path != COHORT_MANIFEST.resolve(strict=True):
        raise SystemExit("source cohort manifest path mismatch")

    items = manifest.get("items")
    if not isinstance(items, list) or len(items) != N_EVAL:
        raise SystemExit(f"source manifest must contain {N_EVAL} items")
    ids = [str(item.get("dataset_id", "")) for item in items]
    if ordered_ids_sha256(ids) != EXPECTED_ORDERED_IDS_SHA256:
        raise SystemExit("source ordered dataset IDs mismatch")
    selection = manifest.get("selection")
    if not isinstance(selection, dict) or selection.get(
        "selected_asset_ids_sha256"
    ) != EXPECTED_ORDERED_IDS_SHA256:
        raise SystemExit("source selection identity hash mismatch")

    for index, (item, row) in enumerate(zip(items, rows)):
        asset_id = row["dataset_id"]
        if item.get("order") != index:
            raise SystemExit(f"source order mismatch at {index}")
        if item.get("dataset_id") != asset_id or item.get("asset_id") != asset_id:
            raise SystemExit(f"source dataset identity mismatch at {index}")
        if item.get("category") != row["category"]:
            raise SystemExit(f"source category binding mismatch: {asset_id}")
        if str(item.get("package", "")) != row["package"]:
            raise SystemExit(f"package path binding mismatch: {asset_id}")
        package = Path(row["package"])
        expected_relative = (Path(package.name) / "model.urdf").as_posix()
        if str(item.get("primary_urdf_relpath", "")) != expected_relative:
            raise SystemExit(f"source URDF relative path mismatch: {asset_id}")
        if item.get("urdf_sha256") != row["urdf_sha256"]:
            raise SystemExit(f"source URDF binding mismatch: {asset_id}")
        binding = row["package_binding"]
        if item.get("package_binding_content_manifest_sha256") != binding.get(
            "content_manifest_sha256"
        ):
            raise SystemExit(f"package content identity mismatch: {asset_id}")
        if item.get("package_binding_file_count") != binding.get("file_count"):
            raise SystemExit(f"package file count identity mismatch: {asset_id}")
        if item.get("package_binding_total_bytes") != binding.get("total_bytes"):
            raise SystemExit(f"package byte count identity mismatch: {asset_id}")
    return manifest, rows, file_sha256


def configure_table4_dir(table4_dir: Path) -> dict[str, Any]:
    global SOURCE_MANIFEST
    global EXPECTED_SOURCE_MANIFEST_SHA256
    global EXPECTED_SOURCE_CONTENT_SHA256
    global _SOURCE_BINDING
    global _COHORT_ROWS

    directory = table4_dir.resolve(strict=True)
    if not directory.is_dir():
        raise SystemExit(f"Table 4 receipt is not a directory: {directory}")
    source_manifest = directory / "frozen_manifest.json"
    if not source_manifest.is_file() or source_manifest.is_symlink():
        raise SystemExit(f"Table 4 frozen manifest is missing: {source_manifest}")
    manifest, rows, file_sha256 = _validate_source_manifest(source_manifest)
    SOURCE_MANIFEST = source_manifest
    EXPECTED_SOURCE_MANIFEST_SHA256 = file_sha256
    EXPECTED_SOURCE_CONTENT_SHA256 = str(manifest["manifest_content_sha256"])
    _COHORT_ROWS = rows
    _SOURCE_BINDING = {
        "table4_dir": str(directory),
        "source_manifest": str(source_manifest),
        "source_manifest_sha256": file_sha256,
        "source_manifest_content_sha256": EXPECTED_SOURCE_CONTENT_SHA256,
        "source_protocol_id": SOURCE_PROTOCOL_ID,
        "ordered_ids_sha256": EXPECTED_ORDERED_IDS_SHA256,
        "n_eval": N_EVAL,
    }
    base.SOURCE_MANIFEST = SOURCE_MANIFEST
    base.EXPECTED_SOURCE_MANIFEST_SHA256 = EXPECTED_SOURCE_MANIFEST_SHA256
    base.EXPECTED_ORDERED_IDS_SHA256 = EXPECTED_ORDERED_IDS_SHA256
    return dict(_SOURCE_BINDING)


def load_source_manifest() -> dict[str, Any]:
    if SOURCE_MANIFEST is None or not EXPECTED_SOURCE_MANIFEST_SHA256:
        raise SystemExit("--table4-dir must be configured before loading the source manifest")
    manifest, rows, _ = _validate_source_manifest(
        SOURCE_MANIFEST, expected_file_sha256=EXPECTED_SOURCE_MANIFEST_SHA256
    )
    if manifest.get("manifest_content_sha256") != EXPECTED_SOURCE_CONTENT_SHA256:
        raise SystemExit("source manifest content hash drift")
    global _COHORT_ROWS
    _COHORT_ROWS = rows
    return manifest


def _referenced_mesh_paths(package: Path, urdf: Path) -> list[Path]:
    root = ET.parse(urdf).getroot()
    references = sorted(
        {
            str(mesh.get("filename", "")).replace("\\", "/")
            for mesh in root.findall(".//visual/geometry/mesh")
            + root.findall(".//collision/geometry/mesh")
            if mesh.get("filename")
        }
    )
    paths: list[Path] = []
    package = package.resolve(strict=True)
    for reference in references:
        if "://" in reference or Path(reference).is_absolute():
            raise SystemExit(f"unsafe Table 4b mesh reference: {reference}")
        path = (package / reference).resolve(strict=True)
        try:
            path.relative_to(package)
        except ValueError as error:
            raise SystemExit(f"Table 4b mesh escapes package: {reference}") from error
        paths.append(path)
    return paths


def _validate_execution_files(
    asset_id: str, package: Path, urdf: Path, binding: Mapping[str, Any]
) -> list[dict[str, Any]]:
    files = binding["files"]
    by_path = {str(row["path"]): row for row in files}
    relevant: list[dict[str, Any]] = []
    for path in [urdf, *_referenced_mesh_paths(package, urdf)]:
        relative = path.relative_to(package).as_posix()
        expected = by_path.get(relative)
        if expected is None:
            raise SystemExit(f"unbound Table 4b input: {asset_id}:{relative}")
        if path.is_symlink() or not path.is_file():
            raise SystemExit(f"invalid Table 4b input file: {asset_id}:{relative}")
        if path.stat().st_size != expected.get("bytes") or sha256_file(path) != expected.get(
            "sha256"
        ):
            raise SystemExit(f"Table 4b input binding mismatch: {asset_id}:{relative}")
        relevant.append(
            {
                "path": relative,
                "bytes": int(expected["bytes"]),
                "sha256": str(expected["sha256"]),
            }
        )
    return relevant


def _observe_relevant_file_binding(job: Mapping[str, Any]) -> list[dict[str, Any]]:
    package = Path(str(job["package"])).resolve(strict=True)
    expected = job.get("expected_relevant_file_binding")
    if not isinstance(expected, list) or not expected:
        raise ValueError("frozen relevant-file binding is missing")
    observed: list[dict[str, Any]] = []
    for row in expected:
        if not isinstance(row, dict):
            raise ValueError("invalid frozen relevant-file row")
        relative = Path(str(row.get("path", "")))
        if relative.is_absolute() or not relative.parts or ".." in relative.parts:
            raise ValueError(f"unsafe frozen relevant-file path: {relative}")
        candidate = package / relative
        path = candidate.resolve(strict=True)
        try:
            path.relative_to(package)
        except ValueError as error:
            raise ValueError(f"relevant input escapes package: {relative}") from error
        if candidate.is_symlink() or not path.is_file():
            raise ValueError(f"invalid relevant input file: {relative}")
        observed.append(
            {
                "path": relative.as_posix(),
                "bytes": path.stat().st_size,
                "sha256": sha256_file(path),
            }
        )
    return observed


def build_jobs(manifest: Mapping[str, Any]) -> list[dict[str, Any]]:
    rows = _COHORT_ROWS if _COHORT_ROWS is not None else _load_cohort_rows()
    jobs: list[dict[str, Any]] = []
    for index, (item, row) in enumerate(zip(manifest["items"], rows)):
        asset_id = row["dataset_id"]
        if item.get("order") != index or item.get("dataset_id") != asset_id:
            raise SystemExit(f"source order or identity mismatch at {index}")
        package = Path(row["package"]).resolve(strict=True)
        urdf = (package / "model.urdf").resolve(strict=True)
        if urdf.parent != package or sha256_file(urdf) != row["urdf_sha256"]:
            raise SystemExit(f"URDF binding mismatch: {asset_id}")
        binding = row["package_binding"]
        relevant_binding = _validate_execution_files(asset_id, package, urdf, binding)
        jobs.append(
            {
                "selection_index": index,
                "dataset_id": asset_id,
                "asset_id": asset_id,
                "category": row["category"],
                "source_component": "PV-A per-class N=5",
                "package": str(package),
                "urdf_path": str(urdf),
                "expected_urdf_sha256": row["urdf_sha256"],
                "input_identity_sha256": str(item["input_identity_sha256"]),
                "package_binding_content_manifest_sha256": str(
                    binding["content_manifest_sha256"]
                ),
                "expected_package_content_manifest_sha256": str(
                    binding["content_manifest_sha256"]
                ),
                "expected_relevant_file_binding": relevant_binding,
                "expected_relevant_file_binding_sha256": canonical_sha256(
                    relevant_binding
                ),
            }
        )
    if len(jobs) != N_EVAL:
        raise SystemExit(f"Table 4b job count mismatch: {len(jobs)}")
    return jobs


def failed_asset_record(job: Mapping[str, Any], issue: str) -> dict[str, Any]:
    record = _base_failed_asset_record(job, issue)
    record.update(
        {
            "expected_package_content_manifest_sha256": job.get(
                "expected_package_content_manifest_sha256"
            ),
            "expected_relevant_file_binding_sha256": job.get(
                "expected_relevant_file_binding_sha256"
            ),
            "expected_relevant_file_binding": job.get(
                "expected_relevant_file_binding"
            ),
            "observed_relevant_file_binding_before": None,
            "observed_relevant_file_binding_before_sha256": None,
            "observed_relevant_file_binding_after": None,
            "observed_relevant_file_binding_after_sha256": None,
        }
    )
    return record


def run_child(job_path: Path, result_path: Path) -> int:
    job = json.loads(job_path.read_text(encoding="utf-8"))
    expected = job.get("expected_relevant_file_binding")
    expected_sha256 = job.get("expected_relevant_file_binding_sha256")
    if not isinstance(expected, list) or canonical_sha256(expected) != expected_sha256:
        record = failed_asset_record(job, "invalid_frozen_relevant_file_binding")
        base.atomic_write_json(result_path, record)
        return 0
    try:
        observed_before = _observe_relevant_file_binding(job)
        observed_before_sha256 = canonical_sha256(observed_before)
    except Exception as exc:  # noqa: BLE001
        record = failed_asset_record(
            job,
            f"input_binding_check_failed_before_evaluation: {type(exc).__name__}: {exc}",
        )
        base.atomic_write_json(result_path, record)
        return 0
    if observed_before_sha256 != expected_sha256:
        record = failed_asset_record(
            job,
            "input_binding_drift_before_evaluation: "
            f"expected {expected_sha256}, observed {observed_before_sha256}",
        )
        record["observed_relevant_file_binding_before"] = observed_before
        record["observed_relevant_file_binding_before_sha256"] = (
            observed_before_sha256
        )
        base.atomic_write_json(result_path, record)
        return 0

    return_code = _base_run_child(job_path, result_path)
    record = json.loads(result_path.read_text(encoding="utf-8"))
    try:
        observed_after = _observe_relevant_file_binding(job)
        observed_after_sha256 = canonical_sha256(observed_after)
    except Exception as exc:  # noqa: BLE001
        record = failed_asset_record(
            job,
            f"input_binding_check_failed_after_evaluation: {type(exc).__name__}: {exc}",
        )
        record["observed_relevant_file_binding_before"] = observed_before
        record["observed_relevant_file_binding_before_sha256"] = (
            observed_before_sha256
        )
        base.atomic_write_json(result_path, record)
        return 0
    if observed_after_sha256 != expected_sha256:
        record = failed_asset_record(
            job,
            "input_binding_drift_after_evaluation: "
            f"expected {expected_sha256}, observed {observed_after_sha256}",
        )
        record["observed_relevant_file_binding_before"] = observed_before
        record["observed_relevant_file_binding_before_sha256"] = (
            observed_before_sha256
        )
        record["observed_relevant_file_binding_after"] = observed_after
        record["observed_relevant_file_binding_after_sha256"] = observed_after_sha256
        base.atomic_write_json(result_path, record)
        return 0
    record["expected_package_content_manifest_sha256"] = job.get(
        "expected_package_content_manifest_sha256"
    )
    record["expected_relevant_file_binding"] = expected
    record["expected_relevant_file_binding_sha256"] = expected_sha256
    record["observed_relevant_file_binding_before"] = observed_before
    record["observed_relevant_file_binding_before_sha256"] = observed_before_sha256
    record["observed_relevant_file_binding_after"] = observed_after
    record["observed_relevant_file_binding_after_sha256"] = observed_after_sha256
    base.atomic_write_json(result_path, record)
    return return_code


def verify_run(
    manifest: Mapping[str, Any],
    records: Sequence[Mapping[str, Any]],
    aggregates: Mapping[str, Any],
) -> dict[str, Any]:
    checks: list[dict[str, Any]] = []

    def check(name: str, passed: bool, detail: str = "") -> None:
        checks.append({"check": name, "pass": bool(passed), "detail": detail})

    if SOURCE_MANIFEST is None:
        raise RuntimeError("Table 4 source manifest is not configured")
    check(
        "source_manifest_sha256",
        sha256_file(SOURCE_MANIFEST) == EXPECTED_SOURCE_MANIFEST_SHA256,
        EXPECTED_SOURCE_MANIFEST_SHA256,
    )
    check(
        "source_manifest_self_hash",
        manifest.get("manifest_content_sha256") == _manifest_self_hash(manifest),
        str(manifest.get("manifest_content_sha256")),
    )
    check("record_count", len(records) == N_EVAL, str(len(records)))
    check(
        "frozen_order_preserved",
        all(
            int(record.get("selection_index", -1)) == index
            and record.get("dataset_id") == manifest["items"][index]["dataset_id"]
            for index, record in enumerate(records)
        ),
    )
    check(
        "urdf_identity_matches_frozen_manifest",
        all(
            (record.get("urdf_sha256") is None and record.get("status") != "completed")
            or record.get("urdf_sha256")
            == manifest["items"][int(record["selection_index"])]["urdf_sha256"]
            for record in records
        ),
    )
    check(
        "package_identity_matches_frozen_manifest",
        all(
            record.get("expected_package_content_manifest_sha256")
            == manifest["items"][int(record["selection_index"])][
                "package_binding_content_manifest_sha256"
            ]
            for record in records
        ),
    )
    check(
        "completed_relevant_input_binding_matches_expected",
        all(
            record.get("status") != "completed"
            or (
                isinstance(record.get("expected_relevant_file_binding"), list)
                and bool(record["expected_relevant_file_binding"])
                and canonical_sha256(record["expected_relevant_file_binding"])
                == record.get("expected_relevant_file_binding_sha256")
                and record.get("observed_relevant_file_binding_before")
                == record["expected_relevant_file_binding"]
                and record.get("observed_relevant_file_binding_before_sha256")
                == record["expected_relevant_file_binding_sha256"]
                and record.get("observed_relevant_file_binding_after")
                == record["expected_relevant_file_binding"]
                and record.get("observed_relevant_file_binding_after_sha256")
                == record["expected_relevant_file_binding_sha256"]
            )
            for record in records
        ),
    )
    check(
        "aggregate_recomputation_matches",
        base.aggregate(records) == dict(aggregates),
    )
    return {
        "all_pass": all(item["pass"] for item in checks),
        "check_count": len(checks),
        "checks": checks,
    }


def output_directory_name(mode: str, n: int, timestamp: str) -> str:
    if mode == "formal":
        return FORMAL_OUTPUT_NAME.format(timestamp=timestamp, n=n)
    if mode == "smoke":
        return SMOKE_OUTPUT_NAME.format(timestamp=timestamp, n=n)
    raise ValueError(f"unsupported mode: {mode}")


def validate_jobs(jobs: Sequence[Mapping[str, Any]], workers: int) -> None:
    if not jobs:
        raise ValueError("Table 4b requires at least one job")
    if workers < 1 or workers > WORKERS:
        raise ValueError(f"workers must be between 1 and {WORKERS}")
    if any(not job.get("expected_package_content_manifest_sha256") for job in jobs):
        raise ValueError("every Table 4b job must retain its package binding")


def _snapshot_file(outdir: Path, name: str, source: Path) -> str:
    payload = source.read_bytes()
    destination = outdir / "source_snapshots" / name
    if destination.exists():
        if destination.read_bytes() != payload:
            raise RuntimeError(f"resume source snapshot drift: {name}")
    else:
        destination.parent.mkdir(parents=True, exist_ok=True)
        base.atomic_write_text(destination, payload.decode("utf-8"))
    return hashlib.sha256(payload).hexdigest()


def snapshot_sources(outdir: Path) -> dict[str, str]:
    if SOURCE_MANIFEST is None:
        raise RuntimeError("Table 4 source manifest is not configured")
    sources = {
        "adapter.py": SCRIPT,
        "base_runner.py": Path(base.__file__).resolve(),
        "geometry.py": Path(base.geometry.__file__).resolve(),
        "source_table4_manifest.json": SOURCE_MANIFEST,
        "cohort_manifest.json": COHORT_MANIFEST,
    }
    return {
        name: _snapshot_file(outdir, name, path)
        for name, path in sorted(sources.items())
    }


def _artifact_entries(outdir: Path) -> dict[str, dict[str, Any]]:
    files: dict[str, dict[str, Any]] = {}
    for path in sorted(outdir.rglob("*")):
        if path == outdir / "artifact_manifest.json" or path.is_dir():
            continue
        if path.is_symlink() or not path.is_file():
            raise RuntimeError(f"invalid artifact path: {path}")
        relative = path.relative_to(outdir).as_posix()
        files[relative] = {
            "bytes": path.stat().st_size,
            "sha256": sha256_file(path),
        }
    return files


def _artifact_input_binding() -> dict[str, Any]:
    if _SOURCE_BINDING is None:
        raise RuntimeError("Table 4 source binding is not configured")
    return {
        "source_manifest_sha256": _SOURCE_BINDING["source_manifest_sha256"],
        "source_manifest_content_sha256": _SOURCE_BINDING[
            "source_manifest_content_sha256"
        ],
        "cohort_manifest_sha256": EXPECTED_COHORT_MANIFEST_SHA256,
        "cohort_manifest_content_sha256": EXPECTED_COHORT_CONTENT_SHA256,
        "ordered_ids_sha256": EXPECTED_ORDERED_IDS_SHA256,
        "n_eval": N_EVAL,
    }


def finalize_receipt(outdir: Path) -> None:
    if _SOURCE_BINDING is None:
        raise RuntimeError("Table 4 source binding is not configured")
    summary = json.loads((outdir / "summary.json").read_text(encoding="utf-8"))
    run_manifest = json.loads((outdir / "manifest.json").read_text(encoding="utf-8"))
    evaluation_wall_seconds = summary.get("wall_seconds")
    invocation = {
        "started_at_utc": _FULL_RUN_STARTED_AT_UTC
        or summary.get("started_at_utc"),
        "completed_at_utc": utc_now(),
        "wall_seconds": round(
            _perf_counter() - _FULL_RUN_STARTED_PERF, 6
        )
        if _FULL_RUN_STARTED_PERF is not None
        else evaluation_wall_seconds,
        "resume": bool(_FULL_RUN_RESUME),
    }
    timing = {
        "schema_version": "table-wall-timing/v1",
        "protocol_id": PROTOCOL_ID,
        "table": "Table 4b",
        "mode": summary.get("mode"),
        **invocation,
        "measurement_endpoint": "after aggregation and verification, before artifact closure",
        "evaluation_child_started_at_utc": summary.get("started_at_utc"),
        "evaluation_child_completed_at_utc": summary.get("completed_at_utc"),
        "evaluation_child_wall_seconds": evaluation_wall_seconds,
        "resume_history": [],
        "resume_invocation_count": 0,
        "cumulative_wall_seconds": invocation["wall_seconds"],
    }
    timing_path = outdir / "timing.json"
    if _FULL_RUN_RESUME and timing_path.is_file():
        initial = json.loads(timing_path.read_text(encoding="utf-8"))
        binding_fields = ("schema_version", "protocol_id", "table", "mode")
        if any(initial.get(key) != timing.get(key) for key in binding_fields):
            raise RuntimeError("resume timing binding differs from the initial run")
        history = list(initial.get("resume_history", []))
        history.append(invocation)
        timing = dict(initial)
        timing["resume_history"] = history
        timing["resume_invocation_count"] = len(history)
        timing["cumulative_wall_seconds"] = round(
            float(initial.get("cumulative_wall_seconds", initial["wall_seconds"]))
            + float(invocation["wall_seconds"]),
            6,
        )
        timing["last_resume_completed_at_utc"] = invocation["completed_at_utc"]
    base.atomic_write_json(timing_path, timing)

    result = run_manifest.get("verification")
    if not isinstance(result, dict):
        raise RuntimeError("run manifest verification is missing")
    if result.get("all_pass") is True:
        status = "PASS"
    elif result.get("all_pass") is False:
        status = "FAIL"
    else:
        status = "SMOKE"
    verification = {
        "schema_version": "table4b-verification-receipt/v1",
        "protocol_id": PROTOCOL_ID,
        "status": status,
        "record_count": run_manifest.get("record_count"),
        "checks": result.get("checks", []),
        "upstream_table4": dict(_SOURCE_BINDING),
        "cohort": {
            "manifest": str(COHORT_MANIFEST.resolve(strict=True)),
            "manifest_sha256": EXPECTED_COHORT_MANIFEST_SHA256,
            "manifest_content_sha256": EXPECTED_COHORT_CONTENT_SHA256,
            "ordered_ids_sha256": EXPECTED_ORDERED_IDS_SHA256,
            "n_eval": N_EVAL,
            "category_count": EXPECTED_CATEGORY_COUNT,
            "per_class": PER_CLASS,
        },
        "run_manifest_sha256": sha256_file(outdir / "manifest.json"),
    }
    base.atomic_write_json(outdir / "verification.json", verification)

    files = _artifact_entries(outdir)
    artifact_manifest = {
        "schema_version": "artifact-manifest/v1",
        "protocol_id": PROTOCOL_ID,
        "excludes": ["artifact_manifest.json"],
        "input_binding": _artifact_input_binding(),
        "files": files,
        "content_manifest_sha256": canonical_sha256(files),
    }
    artifact_manifest["manifest_content_sha256"] = _manifest_self_hash(
        artifact_manifest
    )
    base.atomic_write_json(outdir / "artifact_manifest.json", artifact_manifest)
    if not verify_artifact_manifest(outdir):
        raise RuntimeError("artifact manifest verification failed")


def verify_artifact_manifest(outdir: Path) -> bool:
    try:
        if _SOURCE_BINDING is None:
            return False
        receipt = json.loads(
            (outdir / "artifact_manifest.json").read_text(encoding="utf-8")
        )
        if set(receipt) != {
            "schema_version",
            "protocol_id",
            "excludes",
            "input_binding",
            "files",
            "content_manifest_sha256",
            "manifest_content_sha256",
        }:
            return False
        if (
            receipt.get("schema_version") != "artifact-manifest/v1"
            or receipt.get("protocol_id") != PROTOCOL_ID
            or receipt.get("excludes") != ["artifact_manifest.json"]
            or receipt.get("input_binding") != _artifact_input_binding()
            or receipt.get("manifest_content_sha256")
            != _manifest_self_hash(receipt)
        ):
            return False
        files = receipt.get("files")
        if not isinstance(files, dict):
            return False
        if receipt.get("content_manifest_sha256") != canonical_sha256(files):
            return False
        if set(files) != set(_artifact_entries(outdir)):
            return False
        for relative, expected in files.items():
            if not isinstance(expected, dict) or set(expected) != {"bytes", "sha256"}:
                return False
            relpath = Path(relative)
            if relpath.is_absolute() or ".." in relpath.parts:
                return False
            path = outdir / relpath
            if path.is_symlink() or not path.is_file():
                return False
            if path.stat().st_size != expected.get("bytes"):
                return False
            if sha256_file(path) != expected.get("sha256"):
                return False
        return True
    except (
        KeyError,
        OSError,
        RuntimeError,
        ValueError,
        TypeError,
        json.JSONDecodeError,
    ):
        return False


def _configure_base() -> None:
    values = {
        "SCRIPT": SCRIPT,
        "SCHEMA_VERSION": SCHEMA_VERSION,
        "PROTOCOL_ID": PROTOCOL_ID,
        "DATASET": DATASET,
        "CLASSIFICATION": CLASSIFICATION,
        "N_EVAL": N_EVAL,
        "EXPECTED_CATEGORY_COUNT": EXPECTED_CATEGORY_COUNT,
        "WORKERS": WORKERS,
        "CHILD_TIMEOUT_SECONDS": CHILD_TIMEOUT_SECONDS,
        "CPU_AFFINITY_ENV": "TABLE4B_OURS_PVA_N5_CPU_AFFINITY",
        "FORMAL_OUTPUT_NAME": FORMAL_OUTPUT_NAME,
        "SMOKE_OUTPUT_NAME": SMOKE_OUTPUT_NAME,
        "SUMMARY_TITLE": SUMMARY_TITLE,
        "SELECTION_POLICY": SELECTION_POLICY,
        "load_source_manifest": load_source_manifest,
        "build_jobs": build_jobs,
        "_failed_asset_record": failed_asset_record,
        "run_child": run_child,
        "verify_run": verify_run,
        "output_directory_name": output_directory_name,
        "validate_jobs": validate_jobs,
        "snapshot_sources": snapshot_sources,
        "finalize_receipt": finalize_receipt,
    }
    for name, value in values.items():
        setattr(base, name, value)


_configure_base()


def main(argv: Sequence[str] | None = None) -> int:
    global _FULL_RUN_STARTED_AT_UTC
    global _FULL_RUN_STARTED_PERF
    global _FULL_RUN_RESUME
    _FULL_RUN_STARTED_AT_UTC = utc_now()
    _FULL_RUN_STARTED_PERF = _perf_counter()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--table4-dir", type=Path)
    parser.add_argument("--mode", choices=("smoke", "formal"), default=None)
    parser.add_argument("--n", type=int, default=3, help="smoke sample size")
    parser.add_argument("--workers", type=int, default=None)
    parser.add_argument("--smoke-receipt", type=Path)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--child", action="store_true")
    parser.add_argument("--job", type=Path)
    parser.add_argument("--result", type=Path)
    args = parser.parse_args(argv)
    _FULL_RUN_RESUME = bool(args.resume)
    if args.child:
        if args.job is None or args.result is None:
            raise SystemExit("--child requires --job and --result")
        return run_child(args.job, args.result)
    if args.mode is None:
        raise SystemExit("--mode is required unless --child is given")
    if args.table4_dir is None:
        raise SystemExit("--table4-dir is required")
    configure_table4_dir(args.table4_dir)
    return base.run_scope(args)


if __name__ == "__main__":
    raise SystemExit(main())
