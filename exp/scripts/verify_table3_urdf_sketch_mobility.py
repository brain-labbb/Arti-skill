#!/usr/bin/env python3
"""Independently verify a SketchMobility Table 3 evaluation receipt."""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import platform
import sys
from typing import Any, Callable
import uuid

import numpy


SCRIPT_PATH = Path(__file__).resolve()
FORMAL_N_RELEASE = 4956
FORMAL_N_EVAL = 800
FORMAL_J_EVAL = 1824
FORMAL_EVAL_CATEGORY_COUNT = 67
FORMAL_TABLE2_MANIFEST_FILE_SHA256 = (
    "0be3e21f079bd86ba9ab680f1d709dd676b623bea01d8e43a3db85943a64a8e5"
)
FORMAL_TABLE2_MANIFEST_CONTENT_SHA256 = (
    "a4cd711698d46ce25fa306bd1f1aa751f26d8277c62f592ba9e40b16f08ee2ff"
)
FORMAL_TABLE1_HASHES = {
    "manifest": "081e9e9125f8945cad67a751949e659f6d4e73817704c07cd3fcd4b657ffc696",
    "asset_records": "4c7dc19d2a0558e07e6a0f42ce12bd96e7a8b199849c885374a3e396f7b16cca",
    "release_roster": "9b3f3776162e59baa0b73996b398f97bc52b632b966cedef7bb83ff99acdb765",
}
FORMAL_CORE_SHA256 = "0da075f077ce13c78bb6b4ee66b0abe77668ccf7bb3c105660b321e667fc2acf"
FORMAL_PYTHON_VERSION = "3.12.3"
FORMAL_NUMPY_VERSION = "2.5.1"
EXPECTED_CHILD_ENVIRONMENT = {
    "OMP_NUM_THREADS": "1",
    "OPENBLAS_NUM_THREADS": "1",
    "MKL_NUM_THREADS": "1",
    "NUMEXPR_NUM_THREADS": "1",
}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_sha256(value: Any) -> str:
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def manifest_self_hash(manifest: dict[str, Any]) -> str:
    return canonical_sha256(
        {key: value for key, value in manifest.items() if key != "manifest_content_sha256"}
    )


def read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"JSON root is not an object: {path}")
    return value


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for number, line in enumerate(handle, start=1):
            value = json.loads(line)
            if not isinstance(value, dict):
                raise ValueError(f"JSONL row {number} is not an object: {path}")
            rows.append(value)
    return rows


def atomic_write_json(path: Path, value: Any) -> None:
    temporary = path.parent / f".{path.name}.{uuid.uuid4().hex}.tmp"
    try:
        temporary.write_text(
            json.dumps(value, sort_keys=True, indent=2) + "\n", encoding="utf-8"
        )
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def package_binding(package: Path) -> dict[str, Any]:
    package = package.resolve(strict=True)
    if not package.is_dir():
        raise ValueError(f"package is not a directory: {package}")
    files: list[dict[str, Any]] = []
    for current_raw, directory_names, file_names in os.walk(package, followlinks=False):
        current = Path(current_raw)
        directory_names.sort()
        file_names.sort()
        for name in directory_names:
            if (current / name).is_symlink():
                raise ValueError(f"package contains directory symlink: {current / name}")
        for name in file_names:
            path = current / name
            if path.is_symlink():
                raise ValueError(f"package contains file symlink: {path}")
            canonical = path.resolve(strict=True)
            canonical.relative_to(package)
            if not canonical.is_file():
                raise ValueError(f"package entry is not a regular file: {path}")
            files.append(
                {
                    "path": canonical.relative_to(package).as_posix(),
                    "bytes": canonical.stat().st_size,
                    "sha256": sha256_file(canonical),
                }
            )
    return {
        "file_count": len(files),
        "total_bytes": sum(row["bytes"] for row in files),
        "files": files,
        "content_manifest_sha256": canonical_sha256(files),
    }


@dataclass
class Checks:
    values: dict[str, bool] = field(default_factory=dict)
    errors: dict[str, str] = field(default_factory=dict)

    def run(self, name: str, check: Callable[[], Any]) -> Any:
        try:
            result = check()
            if result is False:
                raise ValueError("check returned false")
            self.values[name] = True
            return result
        except Exception as exc:  # noqa: BLE001
            self.values[name] = False
            self.errors[name] = f"{type(exc).__name__}: {exc}"
            return None


def _require(condition: bool, message: str) -> bool:
    if not condition:
        raise ValueError(message)
    return True


def _load_frozen_core(path: Path):
    spec = importlib.util.spec_from_file_location(
        f"table3_sketch_verifier_core_{uuid.uuid4().hex}", path
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import frozen core: {path}")
    module = importlib.util.module_from_spec(spec)
    previous = sys.dont_write_bytecode
    sys.dont_write_bytecode = True
    try:
        spec.loader.exec_module(module)
    finally:
        sys.dont_write_bytecode = previous
    return module


def _validate_sources(root: Path, manifest: dict[str, Any]) -> dict[str, Path]:
    evaluation = manifest["evaluation"]
    declared = evaluation["source_snapshots"]
    expected_names = {
        "adapter": "run_table3_urdf_sketch_mobility.py",
        "child_harness": "run_urdf_table3_ours_500k.py",
        "core_evaluator": "run_urdf_table3_lam.py",
        "independent_verifier": "verify_table3_urdf_sketch_mobility.py",
    }
    if set(declared) != set(expected_names):
        raise ValueError("source snapshot roles differ")
    paths: dict[str, Path] = {}
    for role, name in expected_names.items():
        row = declared[role]
        path = Path(row["path"]).resolve(strict=True)
        path.relative_to(root)
        if path != root / "source_snapshot" / name or path.is_symlink():
            raise ValueError(f"invalid source snapshot path: {role}")
        if sha256_file(path) != row["sha256"]:
            raise ValueError(f"source snapshot hash mismatch: {role}")
        paths[role] = path
    flat_fields = {
        "adapter": ("adapter_path", "adapter_sha256"),
        "child_harness": ("child_harness_path", "child_harness_sha256"),
        "core_evaluator": ("core_evaluator_path", "core_evaluator_sha256"),
        "independent_verifier": (
            "independent_verifier_path",
            "independent_verifier_sha256",
        ),
    }
    for role, (path_field, hash_field) in flat_fields.items():
        if (
            Path(evaluation[path_field]).resolve(strict=True) != paths[role]
            or evaluation[hash_field] != declared[role]["sha256"]
        ):
            raise ValueError(f"flat source binding mismatch: {role}")
    if declared["core_evaluator"]["sha256"] != FORMAL_CORE_SHA256:
        raise ValueError("shared FK core hash differs from the frozen formal core")
    if declared["independent_verifier"]["sha256"] != sha256_file(SCRIPT_PATH):
        raise ValueError("executed verifier differs from the receipt snapshot")
    return paths


def validate_formal_runtime_environment(environment: dict[str, Any]) -> bool:
    python_full = environment.get("python")
    if not isinstance(python_full, str) or python_full.split(maxsplit=1)[0] != FORMAL_PYTHON_VERSION:
        raise ValueError("formal Python version mismatch")
    if environment.get("numpy") != FORMAL_NUMPY_VERSION:
        raise ValueError("formal numpy version mismatch")
    return True


def _validate_upstream(manifest: dict[str, Any]) -> bool:
    source = manifest["source"]
    table2 = Path(source["table2_manifest_path"]).resolve(strict=True)
    table1 = Path(source["table1_receipt"]).resolve(strict=True)
    _require(
        sha256_file(table2) == FORMAL_TABLE2_MANIFEST_FILE_SHA256,
        "Table 2 manifest file hash mismatch",
    )
    table2_json = read_json(table2)
    _require(
        table2_json.get("manifest_content_sha256")
        == FORMAL_TABLE2_MANIFEST_CONTENT_SHA256,
        "Table 2 manifest content hash mismatch",
    )
    _require(
        table2_json.get("manifest_content_sha256") == manifest_self_hash(table2_json),
        "Table 2 manifest self-hash mismatch",
    )
    observed = {
        "manifest": sha256_file(table1 / "manifest.json"),
        "asset_records": sha256_file(table1 / "asset_records.jsonl"),
        "release_roster": sha256_file(table1 / "release_roster.jsonl"),
    }
    _require(observed == FORMAL_TABLE1_HASHES, "Table 1 receipt hash mismatch")
    _require(
        source["table1_receipt_hashes"] == FORMAL_TABLE1_HASHES,
        "manifest Table 1 binding mismatch",
    )
    frozen_assets = manifest["records"]
    table2_assets = table2_json.get("assets")
    _require(isinstance(table2_assets, list), "Table 2 assets are missing")
    _require(len(frozen_assets) <= len(table2_assets), "cohort is longer than Table 2")
    for rank, (asset, table2_asset) in enumerate(
        zip(frozen_assets, table2_assets, strict=False), start=1
    ):
        expected = {
            "asset_id": table2_asset.get("asset_id"),
            "selection_rank": rank,
            "selection_hash": table2_asset.get("selection_hash"),
            "package": table2_asset.get("package"),
            "urdf_sha256": table2_asset.get("primary_urdf_sha256"),
            "category": table2_asset.get("source_category"),
        }
        observed = {key: asset.get(key) for key in expected}
        _require(observed == expected, f"fixed cohort mismatch at rank {rank}")
    return True


def _validate_manifest_contract(root: Path, manifest: dict[str, Any]) -> bool:
    _require(manifest.get("dataset") == "SketchMobility", "dataset mismatch")
    classification = manifest.get("classification")
    _require(
        classification in {"FORMAL", "NON_FORMAL_SMOKE"}, "classification mismatch"
    )
    selection = manifest["selection"]
    records = manifest["records"]
    _require(selection["n_eval"] == len(records), "manifest N_eval mismatch")
    _require(selection["selection_order_preserved"] is True, "selection order flag")
    _require(selection["outcome_based_reselection"] is False, "reselection flag")
    _require(
        [row["selection_rank"] for row in records] == list(range(1, len(records) + 1)),
        "selection ranks are not contiguous",
    )
    _require(
        selection["selected_asset_ids_sha256"]
        == canonical_sha256([row["asset_id"] for row in records]),
        "selected asset identity hash mismatch",
    )
    evaluation = manifest["evaluation"]
    protocol = Path(evaluation["protocol_path"]).resolve(strict=True)
    _require(protocol == root / "protocol_snapshot.md", "protocol path mismatch")
    _require(sha256_file(protocol) == evaluation["protocol_sha256"], "protocol hash")
    _require(
        evaluation["effective_child_environment"] == EXPECTED_CHILD_ENVIRONMENT,
        "effective child environment mismatch",
    )
    _require(
        evaluation["effective_child_environment_sha256"]
        == canonical_sha256(EXPECTED_CHILD_ENVIRONMENT),
        "effective child environment hash mismatch",
    )
    config = evaluation["config"]
    _require(canonical_sha256(config) == evaluation["config_sha256"], "config hash")
    frozen_config = {
        "protocol_id": "urdf-sim-ready-table3-sketch-mobility-table1-cohort-v1",
        "samples_per_joint": 21,
        "continuous_interval": [-math.pi, math.pi],
        "continuous_non_degenerate_policy": "maximum_excursion_from_q0_over_frozen_interval",
        "q0_policy": "zero_clipped_to_declared_interval",
        "translation_motion_threshold_bbox_diagonal": 1e-6,
        "rotation_motion_threshold_rad": 1e-6,
        "unchanged_translation_tolerance_bbox_diagonal": 1e-9,
        "unchanged_rotation_tolerance_rad": 1e-9,
        "roundtrip_translation_tolerance_bbox_diagonal": 1e-9,
        "roundtrip_rotation_tolerance_rad": 1e-9,
        "strict_asset_requires_at_least_one_declared_movable_joint": True,
        "primary_urdf_relative_path": "mobility.urdf",
        "category_policy": "exact source/category pair; no semantic merging",
    }
    for key, value in frozen_config.items():
        _require(config.get(key) == value, f"frozen config mismatch: {key}")
    if classification == "FORMAL":
        _require(len(records) == FORMAL_N_EVAL, "formal N_eval mismatch")
        _require(manifest["source"]["n_release"] == FORMAL_N_RELEASE, "N_release")
        _require(
            manifest["source"]["eval_category_count"] == FORMAL_EVAL_CATEGORY_COUNT,
            "category count",
        )
        _require(config["samples_per_joint"] == 21, "formal sample count")
        _require(config["workers"] == 4, "formal worker count")
        _require(config["asset_timeout_seconds"] == 120.0, "formal timeout")
        validate_formal_runtime_environment(evaluation["environment"])
    return True


def _validate_packages(manifest: dict[str, Any]) -> bool:
    for asset in manifest["records"]:
        package = Path(asset["package"]).resolve(strict=True)
        urdf = Path(asset["urdf_path"]).resolve(strict=True)
        _require(urdf == package / "mobility.urdf", "primary URDF path mismatch")
        _require(sha256_file(urdf) == asset["urdf_sha256"], "URDF hash mismatch")
        observed = package_binding(package)
        _require(observed == asset["package_binding"], "package binding mismatch")
        _require(
            observed["content_manifest_sha256"]
            == asset["package_content_manifest_sha256"],
            "package content hash mismatch",
        )
    return True


def _validate_records(
    manifest: dict[str, Any], rows: list[dict[str, Any]]
) -> list[dict[str, Any]]:
    assets = manifest["records"]
    _require(len(rows) == len(assets), "asset record count mismatch")
    manifest_hash = manifest["manifest_content_sha256"]
    child_env_hash = manifest["evaluation"]["effective_child_environment_sha256"]
    for asset, row in zip(assets, rows, strict=True):
        expected = {
            "asset_key": asset["asset_key"],
            "asset_id": asset["asset_id"],
            "selection_rank": asset["selection_rank"],
            "selection_hash": asset["selection_hash"],
            "package_content_manifest_sha256": asset[
                "package_content_manifest_sha256"
            ],
            "urdf_sha256": asset["urdf_sha256"],
            "manifest_content_sha256": manifest_hash,
            "effective_child_environment_sha256": child_env_hash,
            "result_origin": "frozen_fresh_interpreter_child_harness",
        }
        _require(
            {key: row.get(key) for key in expected} == expected,
            f"record provenance mismatch: {asset['asset_key']}",
        )
        _require(
            row.get("status") in {"completed", "error", "timeout"},
            "invalid terminal status",
        )
        _require(
            len(row.get("joints", [])) == int(row.get("declared_joint_count", -1)),
            "record joint denominator mismatch",
        )
    return rows


def _validate_checkpoint(manifest: dict[str, Any], checkpoint: dict[str, Any]) -> bool:
    n_eval = manifest["selection"]["n_eval"]
    _require(checkpoint.get("state") == "complete", "checkpoint is not complete")
    _require(checkpoint.get("completed") == n_eval, "checkpoint completed mismatch")
    _require(checkpoint.get("remaining") == 0, "checkpoint remaining mismatch")
    _require(
        checkpoint.get("manifest_content_sha256")
        == manifest["manifest_content_sha256"],
        "checkpoint manifest binding mismatch",
    )
    return True


def _validate_summary(
    manifest: dict[str, Any], summary: dict[str, Any], recomputed: dict[str, Any]
) -> bool:
    for key, value in recomputed.items():
        _require(summary.get(key) == value, f"summary aggregate mismatch: {key}")
    _require(summary.get("manifest_content_sha256") == manifest["manifest_content_sha256"], "summary manifest binding")
    _require(summary.get("dataset") == "SketchMobility", "summary dataset")
    _require(summary.get("classification") == manifest["classification"], "summary class")
    if manifest["classification"] == "FORMAL":
        _require(summary.get("j_eval") == FORMAL_J_EVAL, "formal J_eval mismatch")
    return True


def verify_output(root: Path, *, write_receipt: bool = False) -> dict[str, Any]:
    root = root.resolve(strict=True)
    if not root.is_dir():
        raise ValueError(f"output root is not a directory: {root}")
    checks = Checks()
    required = (
        "manifest.json",
        "asset_records.jsonl",
        "summary.json",
        "summary.md",
        "environment.json",
        "checkpoint.json",
        "protocol_snapshot.md",
    )
    checks.run(
        "required_artifacts_present",
        lambda: _require(
            all((root / name).is_file() and not (root / name).is_symlink() for name in required),
            "required artifact missing or symlinked",
        ),
    )
    manifest = checks.run("manifest_readable", lambda: read_json(root / "manifest.json"))
    if isinstance(manifest, dict):
        checks.run(
            "manifest_self_hash",
            lambda: _require(
                manifest.get("manifest_content_sha256") == manifest_self_hash(manifest),
                "manifest self-hash mismatch",
            ),
        )
        checks.run("manifest_contract", lambda: _validate_manifest_contract(root, manifest))
        sources = checks.run("frozen_sources", lambda: _validate_sources(root, manifest))
        checks.run("fixed_upstream_receipts", lambda: _validate_upstream(manifest))
        checks.run("asset_package_bindings", lambda: _validate_packages(manifest))
    else:
        sources = None
        for name in (
            "manifest_self_hash",
            "manifest_contract",
            "frozen_sources",
            "fixed_upstream_receipts",
            "asset_package_bindings",
        ):
            checks.values[name] = False
            checks.errors[name] = "manifest prerequisite failed"

    rows = checks.run("asset_records_readable", lambda: read_jsonl(root / "asset_records.jsonl"))
    valid_rows = None
    if isinstance(manifest, dict) and isinstance(rows, list):
        valid_rows = checks.run(
            "asset_records_order_and_bindings",
            lambda: _validate_records(manifest, rows),
        )
    else:
        checks.values["asset_records_order_and_bindings"] = False
        checks.errors["asset_records_order_and_bindings"] = "prerequisite failed"

    checkpoint = checks.run("checkpoint_readable", lambda: read_json(root / "checkpoint.json"))
    if isinstance(manifest, dict) and isinstance(checkpoint, dict):
        checks.run("checkpoint_complete_and_bound", lambda: _validate_checkpoint(manifest, checkpoint))
    else:
        checks.values["checkpoint_complete_and_bound"] = False
        checks.errors["checkpoint_complete_and_bound"] = "prerequisite failed"

    recomputed = None
    if isinstance(valid_rows, list) and isinstance(sources, dict):
        core = checks.run("frozen_core_importable", lambda: _load_frozen_core(sources["core_evaluator"]))
        if core is not None:
            recomputed = checks.run(
                "records_reaggregate",
                lambda: core.aggregate_records(valid_rows, len(valid_rows)),
            )
    else:
        checks.values["frozen_core_importable"] = False
        checks.errors["frozen_core_importable"] = "prerequisite failed"
        checks.values["records_reaggregate"] = False
        checks.errors["records_reaggregate"] = "prerequisite failed"

    summary = checks.run("summary_readable", lambda: read_json(root / "summary.json"))
    if isinstance(manifest, dict) and isinstance(summary, dict) and isinstance(recomputed, dict):
        checks.run(
            "summary_matches_reaggregation",
            lambda: _validate_summary(manifest, summary, recomputed),
        )
    else:
        checks.values["summary_matches_reaggregation"] = False
        checks.errors["summary_matches_reaggregation"] = "prerequisite failed"

    environment = checks.run("environment_readable", lambda: read_json(root / "environment.json"))
    if isinstance(manifest, dict) and isinstance(environment, dict):
        checks.run(
            "environment_matches_manifest",
            lambda: _require(
                environment == manifest["evaluation"]["environment"]
                and canonical_sha256(environment)
                == manifest["evaluation"]["environment_sha256"],
                "environment binding mismatch",
            ),
        )
    else:
        checks.values["environment_matches_manifest"] = False
        checks.errors["environment_matches_manifest"] = "prerequisite failed"

    receipt = {
        "schema_version": 1,
        "verifier_protocol_id": "table3-sketch-mobility-independent-verifier-v1",
        "checked_at": datetime.now(timezone.utc).isoformat(),
        "status": "PASS" if all(checks.values.values()) else "FAIL",
        "formal_evaluation": bool(
            all(checks.values.values())
            and isinstance(manifest, dict)
            and manifest.get("classification") == "FORMAL"
        ),
        "output_root": str(root),
        "verifier_path": str(SCRIPT_PATH),
        "verifier_sha256": sha256_file(SCRIPT_PATH),
        "runtime": {"python": platform.python_version(), "numpy": numpy.__version__},
        "checks": checks.values,
        "errors": checks.errors,
        "artifact_sha256": {
            name: sha256_file(root / name) if (root / name).is_file() else None
            for name in required
        },
        "recomputed_aggregates_sha256": (
            canonical_sha256(recomputed) if isinstance(recomputed, dict) else None
        ),
    }
    if write_receipt:
        atomic_write_json(root / "verification.json", receipt)
    return receipt


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument(
        "--write-receipt",
        action="store_true",
        help="write verification.json; only valid before the run is published",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    receipt = verify_output(args.output_root, write_receipt=args.write_receipt)
    print(json.dumps({"status": receipt["status"], "checks": receipt["checks"]}, sort_keys=True))
    return 0 if receipt["status"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
