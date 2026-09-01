#!/usr/bin/env python3
"""Independent verifier for SketchMobility Table 2 supplementary receipts."""

from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
import importlib.util
import json
from pathlib import Path
import os
import subprocess
import sys
from typing import Any


SCRIPT = Path(__file__).resolve()
REPO = Path(os.environ.get("SKETCHMOBILITY_REPO_ROOT", SCRIPT.parents[2])).resolve()
SOURCE_ROOT = Path(os.environ.get("SKETCHMOBILITY_SOURCE_ROOT", REPO)).resolve()
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from exp.scripts import sketchmobility_supplementary_common as common  # noqa: E402


PROTOCOL_ID = "table2-supplementary-sketchmobility-table1-cohort-n800-v1"
FORMAL_SMOKE_N_EVAL = 5
FORMAL_WORKERS = 4
ASSET_TIMEOUT_SECONDS = 120.0
EXPECTED_STATIC_SHA256 = (
    "4701415dad8a5c0a434c16887979bcb70c250ba0b25772014e8db73789098e5f"
)
EXPECTED_UPSTREAM_HASHES = {
    "table2_manifest_sha256": "0be3e21f079bd86ba9ab680f1d709dd676b623bea01d8e43a3db85943a64a8e5",
    "table2_records_sha256": "03b6d5e0d335052f123664a7a85dcdbc33ffbad8143ffb4bb62560e9b44ea2d1",
    "table3_manifest_sha256": "0f90fbdec03cf4be69dc2b870b2aa7eaa3c00de93e49c005394e402907276f4a",
    "table3_records_sha256": "13124125cbdef565efc95c7526e052576aead73fa6499d7b0b81bcc0490a24f7",
    "table4_manifest_file_sha256": "71b895dea4c9ce220825928a89205a05cb7875e2bfce6372687a52eee596de17",
    "table4_asset_records_sha256": "6b51d10a094bea63d20829cf16a4a4034b5cbe31ebdc3852617fc7690ebed58a",
    "table4_state_records_sha256": "91a1b9b676436f5ff753c0fec6f1dfcc9f4e1c32b60cad1172c14f1ce5c12a40",
}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_sha256(value: Any) -> str:
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows = []
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            if not line.strip():
                raise ValueError(f"blank JSONL row: {path}")
            rows.append(json.loads(line))
    return rows


def _load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import frozen source: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    previous = sys.dont_write_bytecode
    sys.dont_write_bytecode = True
    try:
        spec.loader.exec_module(module)
    finally:
        sys.dont_write_bytecode = previous
    return module


def _load_frozen_atoms(root: Path):
    source = root / "source_snapshots/exp/scripts"
    frozen_static = _load_module(
        source / "lam_supplementary_static.py",
        "table2sup_sketch_verifier_frozen_static",
    )
    frozen_adapter = _load_module(
        source / "run_urdf_table2sup_partnet_mobility.py",
        "table2sup_sketch_verifier_frozen_adapter",
    )
    frozen_adapter.static = frozen_static
    frozen_adapter.SCHEMA_VERSION = "table2sup-sketchmobility/v1"
    frozen_adapter.PROTOCOL_ID = PROTOCOL_ID
    frozen_adapter.DATASET = "SketchMobility"
    frozen_adapter.URDF_RELATIVE_PATH = "mobility.urdf"
    frozen_adapter.PLACEHOLDER_REGISTRY = []
    return frozen_adapter


def _authoritative_items(manifest: dict[str, Any]) -> tuple[list[dict[str, Any]], bool]:
    authority = common.load_frozen_cohort(formal=True)["rows"]
    items = manifest.get("items", [])
    if not isinstance(items, list) or len(items) > len(authority):
        return [], False
    expected_items: list[dict[str, Any]] = []
    root = common.DEFAULT_DATASET_ROOT.resolve(strict=True)
    for index, row in enumerate(authority[: len(items)]):
        expected_items.append(
            {
                "selection_index": index,
                "selection_rank": index + 1,
                "asset_id": row["asset_id"],
                "dataset_id": row["asset_id"],
                "category": row["category"],
                "source": row["source"],
                "package": str(root / row["asset_id"]),
                "urdf_relative_path": "mobility.urdf",
                "expected_movable_joints": int(row["movable_dof_count"]),
                "expected_urdf_sha256": row["urdf_sha256"],
                "expected_package_content_manifest_sha256": row[
                    "package_content_manifest_sha256"
                ],
                "table3_declared_joint_count": int(row["movable_dof_count"]),
                "frozen_joint_spec_count": len(row.get("joint_specs", [])),
            }
        )
    return expected_items, items == expected_items


def _recompute_record_atoms(
    adapter: Any, item: dict[str, Any], record: dict[str, Any]
) -> bool:
    package = Path(item["package"])
    try:
        binding = common.package_binding(package)
        urdf_sha256 = sha256_file(package / "mobility.urdf")
    except Exception:  # noqa: BLE001
        return False
    if (
        binding["content_manifest_sha256"]
        != item["expected_package_content_manifest_sha256"]
        or urdf_sha256 != item["expected_urdf_sha256"]
    ):
        return False
    if record.get("expected_movable_joints") != item["expected_movable_joints"]:
        return False
    if record.get("expected_urdf_sha256") != item["expected_urdf_sha256"]:
        return False
    for key in ("asset_id", "dataset_id", "category", "package"):
        if record.get(key) != item.get(key):
            return False
    expected = adapter.audit_partnet_mobility_asset(item)
    for key in (
        "status",
        "parse",
        "table2_supplementary",
        "resource_closure",
        "issues",
        "urdf_sha256",
        "expected_urdf_sha256",
        "expected_movable_joints",
        "table3_declared_joint_count",
    ):
        if record.get(key) != expected.get(key):
            return False
    return (
        record.get("package_content_manifest_sha256")
        == item["expected_package_content_manifest_sha256"]
    )


def _verifiable_runtime_failure(
    item: dict[str, Any],
    record: dict[str, Any],
    *,
    snapshots: dict[str, str],
    manifest_hash: str,
) -> bool:
    # A receipt-local attestation cannot prove that an external process failed:
    # every value needed to forge one is public. Runtime failures therefore stay
    # in staging for resume and are never valid formal receipt atoms.
    del item, record, snapshots, manifest_hash
    return False


def _verify_formal_smoke_binding(
    binding: Any, *, snapshots: dict[str, str]
) -> bool:
    if not isinstance(binding, dict) or binding.get("source_snapshots") != snapshots:
        return False
    try:
        root = Path(str(binding["path"])).resolve(strict=True)
        paths = {
            "manifest_sha256": root / "manifest.json",
            "summary_sha256": root / "summary.json",
            "asset_records_sha256": root / "asset_records.jsonl",
            "artifact_manifest_sha256": root / "artifact_manifest.json",
            "verification_sha256": root / "verification.json",
            "receipt_digest_sha256": root / "receipt_digest.json",
            "frozen_verifier_sha256": root
            / "source_snapshots/exp/scripts/verify_table2sup_urdf_sketch_mobility.py",
        }
        if not all(
            path.is_file()
            and not path.is_symlink()
            and sha256_file(path) == binding.get(field)
            for field, path in paths.items()
        ):
            return False
        digest = json.loads(paths["receipt_digest_sha256"].read_text(encoding="utf-8"))
        if digest.get("tree_sha256") != binding.get("receipt_tree_sha256"):
            return False
        smoke_manifest = json.loads(paths["manifest_sha256"].read_text(encoding="utf-8"))
        smoke_summary = json.loads(paths["summary_sha256"].read_text(encoding="utf-8"))
        expected_items, authority_ok = _authoritative_items(smoke_manifest)
        smoke_items = smoke_manifest.get("items", [])
        expected_asset_ids = [item["asset_id"] for item in expected_items]
        expected_j_eval = sum(
            int(item["expected_movable_joints"]) for item in expected_items
        )
        selection = smoke_manifest.get("selection", {})
        execution = smoke_manifest.get("execution", {})
        if (
            smoke_manifest.get("classification") != "SMOKE"
            or smoke_manifest.get("mode") != "smoke"
            or smoke_summary.get("classification") != "SMOKE"
            or smoke_summary.get("mode") != "smoke"
            or smoke_summary.get("n_eval") != FORMAL_SMOKE_N_EVAL
            or smoke_summary.get("j_eval") != expected_j_eval
            or selection.get("n_eval") != FORMAL_SMOKE_N_EVAL
            or selection.get("j_eval") != expected_j_eval
            or len(smoke_items) != FORMAL_SMOKE_N_EVAL
            or len(expected_items) != FORMAL_SMOKE_N_EVAL
            or not authority_ok
            or selection.get("ordered_asset_ids_sha256")
            != canonical_sha256(expected_asset_ids)
            or selection.get("full_cohort_ordered_asset_ids_sha256")
            != common.EXPECTED_ORDERED_ASSET_IDS_SHA256
            or execution.get("workers") != FORMAL_WORKERS
            or execution.get("asset_timeout_seconds") != ASSET_TIMEOUT_SECONDS
            or smoke_manifest.get("source_snapshots") != snapshots
        ):
            return False
        environment = dict(os.environ)
        environment["SKETCHMOBILITY_REPO_ROOT"] = str(REPO)
        environment["SKETCHMOBILITY_SOURCE_ROOT"] = str(root / "source_snapshots")
        environment["PYTHONPATH"] = str(root / "source_snapshots")
        environment["PYTHONDONTWRITEBYTECODE"] = "1"
        process = subprocess.run(
            [
                sys.executable,
                str(paths["frozen_verifier_sha256"]),
                str(root),
                "--no-write",
            ],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
            env=environment,
        )
        replay = json.loads(process.stdout.decode("utf-8"))
        return process.returncode == 0 and replay.get("status") == "PASS"
    except Exception:  # noqa: BLE001
        return False


def _rate(numerator: int, denominator: int) -> dict[str, Any]:
    return {
        "numerator": numerator,
        "denominator": denominator,
        "percent": round(100.0 * numerator / denominator, 2) if denominator else None,
    }


def reaggregate(records: list[dict[str, Any]]) -> dict[str, Any]:
    n_eval = len(records)
    j_eval = sum(int(record.get("expected_movable_joints", 0)) for record in records)
    completed = sum(record.get("status") == "completed" for record in records)
    parse_passed = sum(bool(record.get("parse", {}).get("success")) for record in records)
    vb_passed = vb_declared = vb_covered = vb_complete = zero_vb = 0
    port_passed = port_extracted = port_intended = 0
    dyn_passed = dyn_extracted = dyn_intended = 0
    complete_inertial = dynamic_links = 0
    categories: dict[str, dict[str, int]] = defaultdict(
        lambda: {
            "assets": 0,
            "vb_passed": 0,
            "port_intended": 0,
            "port_passed": 0,
            "dyn_intended": 0,
            "dyn_passed": 0,
        }
    )
    for record in records:
        cells = record.get("table2_supplementary", {})
        vb = cells.get("visual_bearing_collision_coverage", {})
        port = cells.get("joint_limit_portability", {})
        dyn = cells.get("joint_dynamics_coverage", {})
        placeholder = cells.get("placeholder_mass_incidence", {})
        passed = bool(vb.get("asset_pass"))
        vb_passed += passed
        vb_declared += int(vb.get("visual_bearing_links_declared", 0))
        vb_covered += int(vb.get("covered_visual_bearing_links", 0))
        vb_complete += bool(vb.get("link_extraction_complete"))
        zero_vb += record.get("status") == "completed" and int(
            vb.get("visual_bearing_links_declared", 0)
        ) == 0
        port_intended += int(port.get("joints_intended", 0))
        port_extracted += int(port.get("joints_extracted", 0))
        port_passed += int(port.get("joints_passed", 0))
        dyn_intended += int(dyn.get("joints_intended", 0))
        dyn_extracted += int(dyn.get("joints_extracted", 0))
        dyn_passed += int(dyn.get("joints_covered", 0))
        complete_inertial += int(placeholder.get("complete_inertial_links", 0))
        dynamic_links += int(placeholder.get("dynamic_links", 0))
        bucket = categories[str(record.get("category"))]
        bucket["assets"] += 1
        bucket["vb_passed"] += passed
        bucket["port_intended"] += int(port.get("joints_intended", 0))
        bucket["port_passed"] += int(port.get("joints_passed", 0))
        bucket["dyn_intended"] += int(dyn.get("joints_intended", 0))
        bucket["dyn_passed"] += int(dyn.get("joints_covered", 0))

    def macro(numerator: str, denominator: str) -> float | None:
        values = [
            bucket[numerator] / bucket[denominator]
            for bucket in categories.values()
            if bucket[denominator]
        ]
        return round(100.0 * sum(values) / len(values), 2) if values else None

    category_rows = {
        name: {
            "assets": bucket["assets"],
            "visual_bearing_asset_rate": _rate(
                bucket["vb_passed"], bucket["assets"]
            ),
            "portability_joint_rate": _rate(
                bucket["port_passed"], bucket["port_intended"]
            ),
            "dynamics_joint_rate": _rate(
                bucket["dyn_passed"], bucket["dyn_intended"]
            ),
        }
        for name, bucket in sorted(categories.items())
    }
    return {
        "n_eval": n_eval,
        "j_eval": j_eval,
        "status_counts": {
            "completed": completed,
            "error": n_eval - completed,
            "total": n_eval,
        },
        "parse_passed_assets": parse_passed,
        "metrics": {
            "visual_bearing_collision_coverage": {
                "passed": vb_passed,
                "denominator": n_eval,
                "percent": _rate(vb_passed, n_eval)["percent"],
                "link_micro": _rate(vb_covered, vb_declared),
                "link_extraction_complete_assets": vb_complete,
                "zero_visual_bearing_assets_completed": zero_vb,
            },
            "joint_limit_portability": _rate(port_passed, port_intended)
            | {"joints_extracted": port_extracted},
            "joint_dynamics_coverage": _rate(dyn_passed, dyn_intended)
            | {"joints_extracted": dyn_extracted},
            "placeholder_mass_incidence": {
                "status": "N/E",
                "reason": "placeholder_registry_empty",
                "registry_ids": [],
                "complete_inertial_links": complete_inertial,
                "dynamic_links_measured": dynamic_links,
                "coverage": _rate(complete_inertial, dynamic_links),
            },
        },
        "category_macro": {
            "category_count": len(categories),
            "visual_bearing_asset_rate_mean_percent": macro(
                "vb_passed", "assets"
            ),
            "portability_joint_rate_mean_percent": macro(
                "port_passed", "port_intended"
            ),
            "dynamics_joint_rate_mean_percent": macro(
                "dyn_passed", "dyn_intended"
            ),
            "categories": category_rows,
        },
    }


def _artifact_closure(root: Path) -> bool:
    manifest_path = root / "artifact_manifest.json"
    artifact = json.loads(manifest_path.read_text(encoding="utf-8"))
    expected = {
        str(row["path"]): (int(row["bytes"]), str(row["sha256"]))
        for row in artifact.get("files", [])
    }
    actual = {
        path.relative_to(root).as_posix()
        for path in root.rglob("*")
        if path.is_file()
        and path.name
        not in {"artifact_manifest.json", "verification.json", "receipt_digest.json"}
    }
    if set(expected) != actual:
        return False
    return all(
        (root / relative).stat().st_size == size
        and sha256_file(root / relative) == digest
        and not (root / relative).is_symlink()
        for relative, (size, digest) in expected.items()
    )


def _receipt_digest_valid(root: Path, *, required: bool) -> bool:
    path = root / "receipt_digest.json"
    if not path.exists():
        return not required
    receipt = json.loads(path.read_text(encoding="utf-8"))
    files = [
        {
            "path": candidate.relative_to(root).as_posix(),
            "bytes": candidate.stat().st_size,
            "sha256": sha256_file(candidate),
        }
        for candidate in sorted(root.rglob("*"))
        if candidate.is_file()
        and not candidate.is_symlink()
        and candidate.name != "receipt_digest.json"
    ]
    return (
        receipt.get("schema_version") == "whole-receipt-digest/v1"
        and receipt.get("file_count") == len(files)
        and receipt.get("files") == files
        and receipt.get("tree_sha256") == canonical_sha256(files)
    )


def verify_output(
    output: Path,
    *,
    write_receipt: bool = True,
    require_receipt_digest: bool = True,
) -> dict[str, Any]:
    root = output.resolve(strict=True)
    manifest = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
    summary = json.loads((root / "summary.json").read_text(encoding="utf-8"))
    records = load_jsonl(root / "asset_records.jsonl")
    declared_hash = manifest.get("manifest_content_sha256")
    payload = dict(manifest)
    payload.pop("manifest_content_sha256", None)
    items = manifest.get("items", [])
    expected_items, authority_ok = _authoritative_items(manifest)
    checks: dict[str, bool] = {
        "protocol_id": manifest.get("protocol_id") == PROTOCOL_ID,
        "manifest_self_hash": declared_hash == canonical_sha256(payload),
        "artifact_closure": _artifact_closure(root),
        "whole_receipt_digest": _receipt_digest_valid(
            root, required=require_receipt_digest
        ),
        "record_count": len(records) == len(items),
        "record_order_and_identity": all(
            int(record.get("selection_index", -1)) == index
            and int(record.get("selection_rank", -1)) == index + 1
            and record.get("asset_id") == items[index].get("asset_id")
            for index, record in enumerate(records)
        ),
        "record_manifest_binding": all(
            record.get("manifest_content_sha256") == declared_hash
            for record in records
        ),
        "authoritative_cohort_items": authority_ok,
        "static_evaluator_pin": manifest.get("evaluator", {}).get(
            "static_module_sha256"
        )
        == EXPECTED_STATIC_SHA256
        and manifest.get("source_snapshots", {}).get(
            "exp/scripts/lam_supplementary_static.py"
        )
        == EXPECTED_STATIC_SHA256,
        "placeholder_registry_empty": manifest.get("execution", {}).get(
            "placeholder_registry"
        )
        == [],
        "summary_manifest_binding": summary.get("manifest_content_sha256")
        == declared_hash,
    }
    snapshots = manifest.get("source_snapshots", {})
    checks["source_snapshot_bindings"] = isinstance(snapshots, dict) and all(
        (root / "source_snapshots" / relative).is_file()
        and not (root / "source_snapshots" / relative).is_symlink()
        and sha256_file(root / "source_snapshots" / relative) == digest
        for relative, digest in snapshots.items()
    )
    source = manifest.get("source", {})
    observed_upstream = {
        "table2_manifest_sha256": sha256_file(
            common.DEFAULT_TABLE2_RECEIPT / "manifest.json"
        ),
        "table2_records_sha256": sha256_file(
            common.DEFAULT_TABLE2_RECEIPT / "asset_records.jsonl"
        ),
        "table3_manifest_sha256": sha256_file(
            common.DEFAULT_TABLE3_RECEIPT / "manifest.json"
        ),
        "table3_records_sha256": sha256_file(
            common.DEFAULT_TABLE3_RECEIPT / "asset_records.jsonl"
        ),
        "table4_manifest_file_sha256": sha256_file(
            common.DEFAULT_TABLE4_RECEIPT / "manifest.json"
        ),
        "table4_asset_records_sha256": sha256_file(
            common.DEFAULT_TABLE4_RECEIPT / "asset_records.jsonl"
        ),
        "table4_state_records_sha256": sha256_file(
            common.DEFAULT_TABLE4_RECEIPT / "state_records.jsonl"
        ),
    }
    checks["upstream_receipt_bindings"] = (
        observed_upstream == EXPECTED_UPSTREAM_HASHES
        and all(source.get(key) == value for key, value in EXPECTED_UPSTREAM_HASHES.items())
    )
    try:
        frozen_atoms = _load_frozen_atoms(root)
        checks["record_atoms_recomputed"] = authority_ok and len(records) == len(
            expected_items
        ) and all(
            _recompute_record_atoms(frozen_atoms, item, record)
            for item, record in zip(expected_items, records, strict=True)
        )
    except Exception:  # noqa: BLE001
        checks["record_atoms_recomputed"] = False
    frozen_runner_hash = snapshots.get(
        "exp/scripts/run_table2sup_urdf_sketch_mobility.py"
    )
    checks["children_executed_frozen_sources"] = all(
        record.get("child", {}).get("executed_runner_sha256")
        == frozen_runner_hash
        and record.get("child", {}).get("executed_source_snapshots") == snapshots
        for record in records
    )
    smoke_binding = source.get("smoke_receipt")
    checks["formal_smoke_source_compatibility"] = (
        manifest.get("classification") != "FORMAL"
        or _verify_formal_smoke_binding(
            smoke_binding,
            snapshots=snapshots,
        )
    )
    recomputed = reaggregate(records)
    summary_core = {
        key: summary.get(key)
        for key in (
            "n_eval",
            "j_eval",
            "status_counts",
            "parse_passed_assets",
            "metrics",
            "category_macro",
        )
    }
    checks["summary_matches_reaggregation"] = summary_core == recomputed
    checks["formal_configuration"] = (
        manifest.get("classification") != "FORMAL"
        or (
            len(records) == 800
            and recomputed["j_eval"] == 1824
            and manifest.get("execution", {}).get("workers") == 4
            and manifest.get("execution", {}).get("asset_timeout_seconds") == 120.0
            and manifest.get("selection", {}).get(
                "full_cohort_ordered_asset_ids_sha256"
            )
            == common.EXPECTED_ORDERED_ASSET_IDS_SHA256
        )
    )
    result = {
        "schema_version": "table2sup-sketchmobility-verification/v1",
        "status": "PASS" if all(checks.values()) else "FAIL",
        "formal_evaluation": manifest.get("classification") == "FORMAL",
        "checks": checks,
        "check_count": len(checks),
    }
    if write_receipt:
        (root / "verification.json").write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--no-write", action="store_true")
    parser.add_argument("--allow-missing-receipt-digest", action="store_true")
    args = parser.parse_args(argv)
    result = verify_output(
        args.output,
        write_receipt=not args.no_write,
        require_receipt_digest=not args.allow_missing_receipt_digest,
    )
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["status"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
