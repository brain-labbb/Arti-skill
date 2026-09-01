#!/usr/bin/env python3
"""Run Supplementary Table S1 on Brain-500 + deterministic PV-A-300."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
from typing import Any, Mapping


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from exp.scripts import run_s1_partnet_mobility as base  # noqa: E402

DATASET = "Ours-800"
SCHEMA_VERSION = "supplementary-s1-ours-brain500-pva300/v1"
PROTOCOL_ID = "s1_ours_brain500_pva300_n800_v1"
N_EVAL = 800
TABLE4_PROTOCOL_ID = "urdf_sim_ready_table4_ours_brain500_pva300_n800_v1"
TABLE4_RUN = Path(
    "/root/.cache/torch/arti-skill/"
    "urdf_table4_ours_800_mixed_n800_20260823T075110Z"
)
SOURCE_MANIFEST = TABLE4_RUN / "frozen_manifest.json"
TABLE4_ASSET_RECORDS = TABLE4_RUN / "asset_records.jsonl"
TABLE4_STATE_RECORDS = TABLE4_RUN / "state_records.jsonl"
TABLE4_VERIFICATION = TABLE4_RUN / "verification.json"
DATASET_ROOT = Path("/root/.cache/torch/arti-skill/ours_pva_800_cohort_v2")
PROTOCOL_DOCUMENT = REPO / "exp/URDF-Sim-Ready-Automatic-Evaluation.md"
VERIFIER_PATH = SCRIPT.with_name("verify_s1_ours_800.py")
DEFAULT_OUTPUT_PARENT = Path("/root/.cache/torch/arti-skill")
DEFAULT_WORKERS = 16
EXPECTED_SOURCE_MANIFEST_SHA256 = (
    "b06d08204019d80d00f95779757d3522d3dcb23b4fbe760c722560e4d217520c"
)
EXPECTED_ASSET_RECORDS_SHA256 = (
    "1b4f2c99dcbae277659c86fab043a6371f510526ae42df15caf06b3f5d6900c8"
)
EXPECTED_STATE_RECORDS_SHA256 = (
    "7a423737afa5a3772f1fa67c66482661df08cd3b94bb20f57f0134082b8c3472"
)
EXPECTED_TABLE4_VERIFICATION_SHA256 = (
    "794406c52a9447346501b61955b91f568d87c9f23ee03bc548c3ee5801686f21"
)
EXPECTED_ORDERED_IDS_SHA256 = (
    "f297d85c0de7b00411be80fdfa3b12e71ec21723a4bc0e619df87f229624db39"
)
EXPECTED_FORMAL_STRICT_PASSED = 680


def load_cohort(
    manifest_path: Path,
    dataset_root: Path,
    *,
    formal: bool,
    limit: int | None = None,
) -> dict[str, Any]:
    manifest_path = manifest_path.resolve(strict=True)
    dataset_root = dataset_root.resolve(strict=True)
    digest = base.sha256_file(manifest_path)
    if formal and digest != EXPECTED_SOURCE_MANIFEST_SHA256:
        raise base.ProtocolViolation("formal frozen manifest SHA256 mismatch")
    manifest = base.load_json_object(manifest_path, "frozen manifest")
    items = manifest.get("items")
    if not isinstance(items, list):
        raise ValueError("cohort manifest is not an item list")
    if formal and (
        manifest.get("protocol_id") != TABLE4_PROTOCOL_ID or len(items) != N_EVAL
    ):
        raise base.ProtocolViolation("formal cohort identity mismatch")
    if limit is not None:
        if formal or not 1 <= limit <= len(items):
            raise ValueError("invalid cohort limit")
        items = items[:limit]

    records: list[dict[str, Any]] = []
    for index, item in enumerate(items):
        if item.get("order") != index:
            raise ValueError(f"cohort order mismatch at {index}")
        dataset_id = str(item["dataset_id"])
        package = Path(str(item.get("package", ""))).resolve(strict=True)
        expected_relative = Path(package.name) / "model.urdf"
        if Path(str(item.get("primary_urdf_relpath", ""))) != expected_relative:
            raise ValueError(f"primary URDF path mismatch: {dataset_id}")
        urdf = base._resolved_child(package, "model.urdf", field="primary_urdf")
        if base.sha256_file(urdf) != item.get("urdf_sha256"):
            raise ValueError(f"URDF SHA256 mismatch: {dataset_id}")
        for mesh in item.get("collision_mesh_files", []):
            if mesh.get("exists"):
                path = base._resolved_child(
                    package,
                    str(mesh["resolved_relpath"]),
                    field="collision_mesh",
                )
                if (
                    base.sha256_file(path) != mesh.get("sha256")
                    or path.stat().st_size != mesh.get("size_bytes")
                ):
                    raise ValueError(f"collision mesh binding mismatch: {dataset_id}")
        record = {
            "selection_index": index,
            "asset_id": dataset_id,
            "dataset_id": dataset_id,
            "category": item.get("category"),
            "source_component": item.get("source_component"),
            "source_identity": item.get("source_identity"),
            "package": str(package),
            "primary_urdf_relative_path": "model.urdf",
            "urdf_sha256_expected": item.get("urdf_sha256"),
            "collision_mesh_files_expected": [
                dict(mesh) for mesh in item.get("collision_mesh_files", [])
            ],
            "table4_input_identity_sha256": item.get("input_identity_sha256"),
            "package_binding_content_manifest_sha256": item.get(
                "package_binding_content_manifest_sha256"
            ),
            "movable_dof_count": item.get("movable_dof_count"),
            "range_evaluable_dof_count": item.get("range_evaluable_dof_count"),
            "rest_state_expected": item.get("rest_state_expected"),
            "single_state_expected": item.get("single_state_expected"),
            "sobol_state_expected": item.get("sobol_state_expected"),
            "table4_protocol_id": TABLE4_PROTOCOL_ID,
        }
        record["s1_input_identity_sha256"] = base.canonical_sha256(record)
        records.append(record)
    ordered = base.ordered_ids_sha256([row["dataset_id"] for row in records])
    if formal and ordered != EXPECTED_ORDERED_IDS_SHA256:
        raise base.ProtocolViolation("formal ordered dataset IDs SHA256 mismatch")
    return {
        "manifest_path": str(manifest_path),
        "manifest_file_sha256": digest,
        "dataset_root": str(dataset_root),
        "ordered_dataset_ids_sha256": ordered,
        "records": records,
    }


def load_table4_evidence(
    manifest_path: Path,
    asset_records_path: Path,
    state_records_path: Path,
    cohort: Mapping[str, Any],
    *,
    formal: bool,
    verification_path: Path | None = None,
) -> dict[str, Any]:
    hashes = {
        "manifest": base.sha256_file(manifest_path),
        "assets": base.sha256_file(asset_records_path),
        "states": base.sha256_file(state_records_path),
    }
    expected = {
        "manifest": EXPECTED_SOURCE_MANIFEST_SHA256,
        "assets": EXPECTED_ASSET_RECORDS_SHA256,
        "states": EXPECTED_STATE_RECORDS_SHA256,
    }
    if formal and hashes != expected:
        raise base.ProtocolViolation("formal Table 4 artifact SHA256 mismatch")
    if formal:
        verification_path = (verification_path or TABLE4_VERIFICATION).resolve(
            strict=True
        )
        if (
            base.sha256_file(verification_path)
            != EXPECTED_TABLE4_VERIFICATION_SHA256
        ):
            raise base.ProtocolViolation(
                "formal Table 4 verification SHA256 mismatch"
            )
        receipt = base.load_json_object(
            verification_path, "Table 4 verification"
        )
        if receipt.get("status") != "PASS" or not all(
            receipt.get("checks", {}).values()
        ):
            raise base.ProtocolViolation("formal Table 4 verification did not pass")

    manifest = base.load_json_object(manifest_path, "Table 4 manifest")
    assets = base.load_jsonl(asset_records_path)
    states = base.load_jsonl(state_records_path)
    sources = list(cohort["records"])
    items = list(manifest["items"][: len(sources)])
    selected = {row["dataset_id"] for row in sources}
    assets = [row for row in assets if row.get("dataset_id") in selected]
    assets.sort(key=lambda row: int(row["order"]))
    states_by_id = {row["dataset_id"]: [] for row in sources}
    seen_order: list[str] = []
    for state in states:
        dataset_id = state.get("dataset_id")
        if dataset_id not in states_by_id:
            if formal:
                raise ValueError(f"unknown Table 4 state dataset_id: {dataset_id!r}")
            continue
        if not seen_order or seen_order[-1] != dataset_id:
            if dataset_id in seen_order:
                raise ValueError("Table 4 states are not grouped in cohort order")
            seen_order.append(str(dataset_id))
        states_by_id[str(dataset_id)].append(state)
    expected_order = [
        row["dataset_id"] for row in sources if states_by_id[row["dataset_id"]]
    ]
    if seen_order != expected_order:
        raise ValueError("Table 4 state records do not follow cohort order")
    if len(items) != len(sources) or len(assets) != len(sources):
        raise ValueError("Table 4 evidence denominator mismatch")

    records = []
    for index, (source, item, asset) in enumerate(
        zip(sources, items, assets, strict=True)
    ):
        if (
            item.get("dataset_id") != source["dataset_id"]
            or asset.get("dataset_id") != source["dataset_id"]
            or item.get("order") != index
            or asset.get("order") != index
        ):
            raise ValueError(f"Table 4 identity mismatch at {index}")
        for field in (
            "category",
            "movable_dof_count",
            "range_evaluable_dof_count",
        ):
            if item.get(field) != source.get(field):
                raise ValueError(f"Table 4 cohort {field} mismatch at {index}")
        records.append(
            base._reaggregate_table4_asset(
                source, asset, states_by_id[source["dataset_id"]]
            )
        )
    strict = sum(row["strict_collision_pass"] for row in records)
    if formal and strict != EXPECTED_FORMAL_STRICT_PASSED:
        raise base.ProtocolViolation("formal strict-pass result drifted")
    return {
        "protocol_id": manifest.get("protocol_id"),
        "manifest_path": str(Path(manifest_path).resolve()),
        "manifest_file_sha256": hashes["manifest"],
        "asset_records_path": str(Path(asset_records_path).resolve()),
        "asset_records_file_sha256": hashes["assets"],
        "state_records_path": str(Path(state_records_path).resolve()),
        "state_records_file_sha256": hashes["states"],
        "strict_passed": strict,
        "denominator": len(records),
        "state_record_count": sum(
            len(states_by_id[row["dataset_id"]]) for row in sources
        ),
        "records": records,
    }


def render_summary(summary: Mapping[str, Any]) -> str:
    metrics = summary["metrics"]

    def fraction(metric: Mapping[str, Any]) -> str:
        return (
            f"{metric['passed']} / {metric['denominator']} "
            f"({100.0 * metric['rate']:.2f}%)"
        )

    rebuild = metrics["deterministic_rebuild_match"]
    allowance = metrics["allowance_density"]
    return "\n".join(
        [
            f"# Supplementary Table S1: {DATASET}",
            "",
            f"- Protocol: `{summary['protocol_id']}`",
            f"- Status: `{summary['status']}`",
            f"- N_eval: {summary['n_eval']}",
            "",
            "| Metric | Result |",
            "|---|---:|",
            f"| Receipt-bound Assets | {fraction(metrics['receipt_bound_assets'])} |",
            f"| Receipt Replay Pass | {fraction(metrics['receipt_replay_pass'])} |",
            (
                "| Deterministic Rebuild Match | "
                f"{rebuild['status']} ({rebuild['eligible_assets']} / "
                f"{rebuild['asset_denominator']} eligible) |"
            ),
            (
                f"| Allowance Density | {allowance['registered_pairs']} / "
                f"{allowance['eligible_pairs']} "
                f"({100.0 * (allowance['rate'] or 0.0):.2f}%) |"
            ),
            (
                "| Strict Pass (No Method-specific Allowance) | "
                f"{fraction(metrics['strict_pass_no_method_allowance'])} |"
            ),
            (
                "| Registered-allowance Gain | "
                f"{metrics['registered_allowance_gain_pp']['value']:.2f} pp |"
            ),
            "",
        ]
    )


def configure_base() -> None:
    values = globals()
    names = (
        "SCRIPT",
        "DATASET",
        "SCHEMA_VERSION",
        "PROTOCOL_ID",
        "N_EVAL",
        "TABLE4_PROTOCOL_ID",
        "SOURCE_MANIFEST",
        "TABLE4_ASSET_RECORDS",
        "TABLE4_STATE_RECORDS",
        "TABLE4_VERIFICATION",
        "DATASET_ROOT",
        "PROTOCOL_DOCUMENT",
        "VERIFIER_PATH",
        "DEFAULT_OUTPUT_PARENT",
        "DEFAULT_WORKERS",
        "EXPECTED_SOURCE_MANIFEST_SHA256",
        "EXPECTED_ASSET_RECORDS_SHA256",
        "EXPECTED_STATE_RECORDS_SHA256",
        "EXPECTED_TABLE4_VERIFICATION_SHA256",
        "EXPECTED_ORDERED_IDS_SHA256",
        "EXPECTED_FORMAL_STRICT_PASSED",
    )
    for name in names:
        setattr(base, name, values[name])
    base.load_cohort = load_cohort
    base.load_table4_evidence = load_table4_evidence
    base.render_summary = render_summary


def main(argv: list[str] | None = None) -> int:
    configure_base()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--workers", type=int, default=DEFAULT_WORKERS)
    parser.add_argument("--limit", type=int)
    parser.add_argument("--smoke", action="store_true")
    args = parser.parse_args(argv)
    formal = not args.smoke
    if formal and args.limit is not None:
        parser.error("--limit requires --smoke")
    label = (
        "brain500_pva300_n800" if formal else f"smoke_n{args.limit or 5}"
    )
    output = args.output or (
        DEFAULT_OUTPUT_PARENT / f"s1_ours_{label}_{base.timestamp_tag()}"
    )
    summary = base.run_evaluation(
        output=output,
        cohort_manifest=SOURCE_MANIFEST,
        dataset_root=DATASET_ROOT,
        table4_manifest=SOURCE_MANIFEST,
        table4_asset_records=TABLE4_ASSET_RECORDS,
        table4_state_records=TABLE4_STATE_RECORDS,
        table4_verification=TABLE4_VERIFICATION,
        protocol_document=PROTOCOL_DOCUMENT,
        formal=formal,
        workers=args.workers,
        limit=args.limit,
    )
    print(
        base.canonical_json(
            {
                "output": str(output),
                "status": summary["status"],
                "metrics": summary["metrics"],
            }
        )
    )
    return 0


configure_base()


if __name__ == "__main__":
    raise SystemExit(main())
