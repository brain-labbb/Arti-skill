#!/usr/bin/env python3
"""Run the frozen Table 4 state evidence protocol on Brain-500 + PV-A-300.

This adapter preserves the published Ours Table 4 collision core and execution
contract while resolving every asset against the absolute package frozen in the
mixed Ours-800 cohort manifest.  The resulting state evidence is the prerequisite
for the Ours-800 Supplementary Table S1 replay.
"""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import sys
from typing import Any


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
BASE_SCRIPT = REPO / "exp/scripts/run_urdf_table4_ours_500k.py"
DEFAULT_COHORT_MANIFEST = Path(
    "/root/.cache/torch/arti-skill/ours_pva_800_cohort_v2/manifest.json"
)
DATASET_LABEL = "Ours-800"
PROTOCOL_ID = "urdf_sim_ready_table4_ours_brain500_pva300_n800_v1"
SAMPLE_SIZE = 800
EXPECTED_COHORT_FILE_SHA256 = (
    "014ac091edf84037a12b044226f384722187167ebc7c47330d51e3b717399b53"
)
EXPECTED_COHORT_CONTENT_SHA256 = (
    "5c79bc84cb92aed8306c4f9b283634159f457661c46c2b8abd2d706359994f4d"
)
EXPECTED_ORDERED_IDS_SHA256 = (
    "f297d85c0de7b00411be80fdfa3b12e71ec21723a4bc0e619df87f229624db39"
)
EXPECTED_PVA_SELECTION_SHA256 = (
    "4d3bac6ff5884311b2277c928e85e2d825970e8ddbe5d44ab436b556b7615368"
)


def _load_base():
    spec = importlib.util.spec_from_file_location("urdf_table4_ours_500k_base", BASE_SCRIPT)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import Table 4 base runner: {BASE_SCRIPT}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


base = _load_base()
_base_audit_asset = base.audit_asset
_base_build_frozen_items = base.build_frozen_items
_base_evaluate_asset = base.evaluate_asset
_base_report_text = base.report_text
_base_run = base.run


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
    )
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _manifest_self_hash(manifest: dict[str, Any]) -> str:
    payload = dict(manifest)
    payload.pop("manifest_content_sha256", None)
    return canonical_sha256(payload)


def _validate_package_binding(row: dict[str, Any]) -> str:
    binding = row.get("package_binding")
    if not isinstance(binding, dict) or not isinstance(binding.get("files"), list):
        raise ValueError(f"missing package binding: {row.get('dataset_id')}")
    files = binding["files"]
    content_hash = canonical_sha256(files)
    if content_hash != binding.get("content_manifest_sha256"):
        raise ValueError(f"package binding self-hash mismatch: {row.get('dataset_id')}")
    if binding.get("file_count") != len(files):
        raise ValueError(f"package binding file count mismatch: {row.get('dataset_id')}")
    if binding.get("total_bytes") != sum(int(item["bytes"]) for item in files):
        raise ValueError(f"package binding byte count mismatch: {row.get('dataset_id')}")
    urdf_rows = [item for item in files if item.get("path") == "model.urdf"]
    if len(urdf_rows) != 1 or urdf_rows[0].get("sha256") != row.get("urdf_sha256"):
        raise ValueError(f"package binding URDF mismatch: {row.get('dataset_id')}")
    return content_hash


def load_cohort(path: Path) -> dict[str, Any]:
    path = path.resolve(strict=True)
    file_sha = sha256_file(path)
    if file_sha != EXPECTED_COHORT_FILE_SHA256:
        raise ValueError("Ours-800 cohort manifest file hash mismatch")
    manifest = json.loads(path.read_text(encoding="utf-8"))
    if manifest.get("manifest_content_sha256") != _manifest_self_hash(manifest):
        raise ValueError("Ours-800 cohort manifest self-hash mismatch")
    if manifest.get("manifest_content_sha256") != EXPECTED_COHORT_CONTENT_SHA256:
        raise ValueError("Ours-800 cohort manifest content hash mismatch")
    if (
        manifest.get("schema_version") != "ours-pva-800-cohort/v1"
        or manifest.get("protocol_id") != "ours-brain500-pva300-cohort-v1"
        or manifest.get("dataset") != DATASET_LABEL
        or manifest.get("classification") != "FORMAL"
        or manifest.get("n_eval") != SAMPLE_SIZE
        or manifest.get("composition") != {"Brain-500": 500, "PV-A-300": 300}
    ):
        raise ValueError("Ours-800 cohort protocol metadata mismatch")
    selection = manifest.get("selection", {})
    if selection.get("ordered_dataset_ids_sha256") != EXPECTED_ORDERED_IDS_SHA256:
        raise ValueError("Ours-800 ordered identity binding mismatch")
    if selection.get("selected_pva_identities_sha256") != EXPECTED_PVA_SELECTION_SHA256:
        raise ValueError("PV-A selection binding mismatch")
    assets = manifest.get("assets")
    if not isinstance(assets, list) or len(assets) != SAMPLE_SIZE:
        raise ValueError("Ours-800 cohort asset count mismatch")

    rows: list[dict[str, Any]] = []
    seen: set[str] = set()
    for index, raw in enumerate(assets):
        if raw.get("selection_index") != index:
            raise ValueError("Ours-800 selection order mismatch")
        dataset_id = str(raw.get("dataset_id", ""))
        if not dataset_id or dataset_id in seen:
            raise ValueError(f"duplicate or empty Ours-800 identity: {dataset_id!r}")
        seen.add(dataset_id)
        package = Path(str(raw.get("package", ""))).resolve(strict=True)
        urdf_path = package / str(raw.get("primary_urdf_relative_path", ""))
        if urdf_path != package / "model.urdf" or not urdf_path.is_file():
            raise ValueError(f"missing canonical model.urdf: {dataset_id}")
        binding_hash = _validate_package_binding(raw)
        rows.append(
            {
                "asset_id": dataset_id,
                "raw_category": str(raw["raw_category"]),
                "seed_name": str(raw["seed_name"]),
                "asset_root": package.name,
                "selection_index": index,
                "primary_urdf_sha256": str(raw["urdf_sha256"]),
                "package": str(package),
                "source_component": str(raw["source"]),
                "source_identity": str(raw["source_identity"]),
                "package_binding_content_manifest_sha256": binding_hash,
            }
        )
    if [row["source_component"] for row in rows[:500]] != ["Brain-500"] * 500:
        raise ValueError("Brain component order mismatch")
    if [row["source_component"] for row in rows[500:]] != ["PV-A-300"] * 300:
        raise ValueError("PV-A component order mismatch")
    return {
        "file_sha256": file_sha,
        "content_sha256": manifest["manifest_content_sha256"],
        "source": {"cohort_type": "frozen Brain-500 + deterministic PV-A-300"},
        "rows": rows,
    }


def audit_asset(_dataset_root: Path, row: dict[str, Any]) -> dict[str, Any]:
    package = Path(row["package"]).resolve(strict=True)
    return _base_audit_asset(package.parent, row)


def build_frozen_items(
    cohort_rows: list[dict[str, Any]],
    audits: dict[str, dict[str, Any]],
    runtime_identity: dict[str, Any],
) -> list[dict[str, Any]]:
    items = _base_build_frozen_items(cohort_rows, audits, runtime_identity)
    for item, row in zip(items, cohort_rows):
        item.update(
            {
                "package": row["package"],
                "source_component": row["source_component"],
                "source_identity": row["source_identity"],
                "package_binding_content_manifest_sha256": row[
                    "package_binding_content_manifest_sha256"
                ],
            }
        )
        item["input_identity_sha256"] = canonical_sha256(
            {
                "base_fields": {
                    key: item[key] for key in base.FROZEN_INPUT_FIELDS
                },
                "package": item["package"],
                "source_component": item["source_component"],
                "source_identity": item["source_identity"],
                "package_binding_content_manifest_sha256": item[
                    "package_binding_content_manifest_sha256"
                ],
            }
        )
    return items


def evaluate_asset(item: dict[str, Any], _dataset_root: Path) -> dict[str, Any]:
    package = Path(item["package"]).resolve(strict=True)
    return _base_evaluate_asset(item, package.parent)


def report_text(summary: dict[str, Any], manifest: dict[str, Any]) -> str:
    report = _base_report_text(summary, manifest)
    report = report.replace("Ours-500K", DATASET_LABEL)
    report = report.replace(
        "full acquired roster, Table 2 manifest order",
        "frozen Brain-500 + deterministic PV-A-300 cohort order",
    )
    return report


def _finalize_mixed_metadata(output: Path, cohort_path: Path) -> None:
    manifest_path = output / "frozen_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["schema_version"] = "table4_ours_brain500_pva300_frozen_manifest_v1"
    manifest["cohort_label"] = "Ours-800: Brain-500 + deterministic PV-A-300"
    manifest["source"] = {
        "cohort_manifest_path": str(cohort_path.resolve(strict=True)),
        "cohort_manifest_file_sha256": EXPECTED_COHORT_FILE_SHA256,
        "cohort_manifest_content_sha256": EXPECTED_COHORT_CONTENT_SHA256,
        "cohort_type": "frozen Brain-500 + deterministic PV-A-300",
        "composition": {"Brain-500": 500, "PV-A-300": 300},
        "n_release": SAMPLE_SIZE,
        "n_eval": len(manifest["items"]),
        "category_count": len({item["category"] for item in manifest["items"]}),
        "per_item_package_paths": True,
    }
    manifest["selection"]["algorithm"] = (
        "exact frozen Ours-800 cohort .assets[] order; optional smoke prefix only"
    )
    manifest["selection"]["ordered_input_identities_sha256"] = canonical_sha256(
        [item["input_identity_sha256"] for item in manifest["items"]]
    )
    manifest["manifest_content_sha256"] = base._manifest_self_hash(manifest)
    base.atomic_json(manifest_path, manifest)

    summary_path = output / "summary.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    summary["manifest_content_sha256"] = manifest["manifest_content_sha256"]
    base.atomic_json(summary_path, summary)

    checkpoint_path = output / "checkpoint.json"
    checkpoint = json.loads(checkpoint_path.read_text(encoding="utf-8"))
    checkpoint["manifest_content_sha256"] = manifest["manifest_content_sha256"]
    base.atomic_json(checkpoint_path, checkpoint)


def run(args):
    output = _base_run(args)
    _finalize_mixed_metadata(output, args.table2_manifest)
    return output


def _configure_base() -> None:
    base.SCRIPT = SCRIPT
    base.DATASET_LABEL = DATASET_LABEL
    base.DATASET_ROOT = DEFAULT_COHORT_MANIFEST.parent
    base.DEFAULT_TABLE2_MANIFEST = DEFAULT_COHORT_MANIFEST
    base.PROTOCOL_ID = PROTOCOL_ID
    base.SAMPLE_SIZE = SAMPLE_SIZE
    base.EXPECTED_N_RELEASE = SAMPLE_SIZE
    base.EXPECTED_TABLE2_MANIFEST_FILE_SHA256 = EXPECTED_COHORT_FILE_SHA256
    base.EXPECTED_TABLE2_MANIFEST_CONTENT_SHA256 = EXPECTED_COHORT_CONTENT_SHA256
    base.EXPECTED_ARCHIVE_SHA256 = EXPECTED_COHORT_FILE_SHA256
    base.load_cohort = load_cohort
    base.audit_asset = audit_asset
    base.build_frozen_items = build_frozen_items
    base.evaluate_asset = evaluate_asset
    base.report_text = report_text
    base.run = run


_configure_base()


def main(argv: list[str] | None = None) -> int:
    return base.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
