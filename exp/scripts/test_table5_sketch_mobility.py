#!/usr/bin/env python3
"""Contract tests for the frozen SketchMobility Table 5 pipeline."""

from __future__ import annotations

import copy
import importlib.util
import json
from pathlib import Path
import shutil
import sys
import tempfile
import unittest


REPO = Path(__file__).resolve().parents[2]
COMMON = REPO / "exp/scripts/table5_sketch_mobility_common.py"
PREPARE = REPO / "exp/scripts/prepare_table5_sketch_mobility_n800.py"
RUNNER = REPO / "exp/scripts/run_table5_sketch_mobility.py"
AGGREGATE = REPO / "exp/scripts/aggregate_table5_sketch_mobility.py"
VERIFY = REPO / "exp/scripts/verify_table5_sketch_mobility_published.py"
PROTOCOL = REPO / "exp/reference/table5_sketch_mobility_n800_protocol_v1.json"
TABLE1_MANIFEST = (
    REPO
    / "exp/runtime/table1_sketch_mobility_rerun_20260821T021838Z/manifest.json"
)
DATASET_ROOT = REPO / "exp/SketchMobility"
UPSTREAM_ROOTS = {
    "table2": REPO
    / "exp/runtime/table2_urdf_sketch_mobility_table1cohort_n800_20260821T035015Z",
    "table3": REPO
    / "exp/runtime/urdf_table3_sketch_mobility_table1cohort_n800_20260821T062050Z",
    "table4": REPO
    / "exp/runtime/urdf_table4_sketch_mobility_table1cohort_n800_20260821T090554Z",
}


def load_module(path: Path, name: str):
    if not path.is_file():
        raise AssertionError(f"required module is missing: {path}")
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise AssertionError(f"cannot load module: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


class SketchMobilityTable5ContractsTests(unittest.TestCase):
    def test_table1_loader_preserves_the_exact_frozen_800_asset_order(self) -> None:
        common = load_module(COMMON, "table5_sketch_common_test_target")

        loaded = common.load_table1_cohort(
            DATASET_ROOT, TABLE1_MANIFEST, formal=True
        )

        self.assertEqual(800, len(loaded["assets"]))
        self.assertEqual(
            "data/Shape2Motion/Kettle/kettle_0057",
            loaded["assets"][0]["asset_id"],
        )
        self.assertEqual(1, loaded["assets"][0]["selection_rank"])
        self.assertEqual(
            "150fb5b16442ad363223d045fcddfa385d1d164851c6f37602a1c5cb64602711",
            loaded["assets"][0]["mobility_urdf_sha256"],
        )

    def test_upstream_loader_joins_all_four_tables_by_original_asset_id(self) -> None:
        common = load_module(COMMON, "table5_sketch_upstream_test_target")
        self.assertTrue(
            hasattr(common, "load_upstream_records"),
            "SketchMobility Table 5 must expose an upstream receipt joiner",
        )
        cohort = common.load_table1_cohort(
            DATASET_ROOT, TABLE1_MANIFEST, formal=True
        )
        asset_ids = [row["asset_id"] for row in cohort["assets"]]

        upstream = common.load_upstream_records(UPSTREAM_ROOTS, asset_ids)

        first = "data/Shape2Motion/Kettle/kettle_0057"
        self.assertEqual(first, upstream["table2"][first]["asset_id"])
        self.assertEqual(first, upstream["table3"][first]["asset_id"])
        self.assertEqual(first, upstream["table4"][first]["dataset_id"])
        self.assertFalse(upstream["table2"][first]["strict_urdf_pass"])
        self.assertTrue(upstream["table3"][first]["strict_kinematic_pass"])
        self.assertTrue(upstream["table4"][first]["load_success"])

    def test_manifest_row_maps_original_identity_to_stable_runtime_id(self) -> None:
        common = load_module(COMMON, "table5_sketch_row_test_target")
        self.assertTrue(
            hasattr(common, "build_manifest_row"),
            "SketchMobility Table 5 must expose a manifest-row builder",
        )
        cohort = common.load_table1_cohort(
            DATASET_ROOT, TABLE1_MANIFEST, formal=True
        )
        asset_ids = [row["asset_id"] for row in cohort["assets"]]
        upstream = common.load_upstream_records(UPSTREAM_ROOTS, asset_ids)
        identity = cohort["assets"][0]
        asset_id = identity["asset_id"]

        row = common.build_manifest_row(
            DATASET_ROOT,
            identity,
            {name: records[asset_id] for name, records in upstream.items()},
            order=0,
        )

        self.assertEqual("sketch_0000", row["dataset_id"])
        self.assertEqual(asset_id, row["asset_id"])
        self.assertEqual(asset_id, row["manifest_root"])
        self.assertEqual(asset_id, row["package_relative_path"])
        self.assertEqual(f"{asset_id}/mobility.urdf", row["urdf_relative_path"])
        self.assertEqual(
            "b1e55aa48e8120a9e94e82d4400881054adf749c2cbcf09b7dcb7a0d301c1eae",
            row["package_content_manifest_sha256"],
        )
        self.assertTrue(row["preflight"]["simulator_eligible"])
        self.assertFalse(row["strict_gates"]["table2"]["strict_urdf_pass"])
        self.assertTrue(row["strict_gates"]["table3"]["strict_kinematic_pass"])
        self.assertTrue(row["strict_gates"]["table4"]["strict_collision_pass"])

    def test_manifest_builder_retains_all_800_rows_without_outcome_filtering(self) -> None:
        common = load_module(COMMON, "table5_sketch_manifest_test_target")
        self.assertTrue(
            hasattr(common, "build_manifest"),
            "SketchMobility Table 5 must expose a full manifest builder",
        )

        manifest = common.build_manifest(
            DATASET_ROOT,
            TABLE1_MANIFEST,
            UPSTREAM_ROOTS,
            protocol={"schema_version": "test-only"},
            formal=False,
        )

        self.assertEqual(800, len(manifest["rows"]))
        self.assertEqual("sketch_0000", manifest["rows"][0]["dataset_id"])
        self.assertEqual("sketch_0799", manifest["rows"][-1]["dataset_id"])
        self.assertEqual(800, manifest["selection"]["selected_count"])
        self.assertFalse(manifest["selection"]["outcome_filtering"])
        self.assertEqual(
            311,
            sum(row["preflight"]["simulator_eligible"] for row in manifest["rows"]),
        )

    def test_canonical_protocol_freezes_gpu3_and_full_800_denominators(self) -> None:
        common = load_module(COMMON, "table5_sketch_protocol_test_target")
        self.assertTrue(
            hasattr(common, "validate_canonical_protocol"),
            "SketchMobility Table 5 must validate a canonical protocol",
        )
        self.assertTrue(PROTOCOL.is_file(), "canonical protocol is missing")

        protocol = common.validate_canonical_protocol(PROTOCOL)

        self.assertEqual("SketchMobility", protocol["source"]["dataset"])
        self.assertEqual(800, protocol["selection"]["selected_count"])
        self.assertEqual(800, protocol["cross_simulator"]["all_three_denominator"])
        self.assertEqual("cuda", protocol["adapters"]["genesis"]["backend"])
        self.assertEqual(
            "GPU-ebc0d328-a3fa-7e89-2733-cadb001661f7",
            protocol["adapters"]["genesis"]["gpu_binding"]["gpu_uuid"],
        )

    def test_formal_manifest_rebuilds_against_all_frozen_source_bindings(self) -> None:
        common = load_module(COMMON, "table5_sketch_formal_manifest_test_target")
        self.assertTrue(
            hasattr(common, "validate_manifest"),
            "SketchMobility Table 5 must expose a manifest validator",
        )
        protocol = common.validate_canonical_protocol(PROTOCOL)
        manifest = common.build_manifest(
            DATASET_ROOT,
            TABLE1_MANIFEST,
            UPSTREAM_ROOTS,
            protocol=protocol,
            formal=True,
        )

        common.validate_manifest(
            manifest,
            DATASET_ROOT,
            TABLE1_MANIFEST,
            UPSTREAM_ROOTS,
            protocol=protocol,
            formal=True,
        )

        self.assertEqual(
            "5fa3622502d74feacffd327b61c7a43f7c30d6d6109d4439d79651a39a39805d",
            manifest["selection"]["ordered_package_binding_sha256"],
        )
        self.assertEqual(489, manifest["selection"]["retained_preflight_failures"])
        self.assertEqual(
            31403, sum(row["package_binding"]["file_count"] for row in manifest["rows"])
        )

    def test_receipt_marker_rejects_manifest_tampering(self) -> None:
        common = load_module(COMMON, "table5_sketch_receipt_test_target")
        self.assertTrue(
            hasattr(common, "publish_receipt_set"),
            "SketchMobility Table 5 must publish a bound receipt set",
        )
        protocol = common.protocol_with_hash(
            common.validate_canonical_protocol(PROTOCOL)
        )
        manifest = {"schema_version": "test-fixture", "rows": []}
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as directory:
            output = Path(directory)
            common.publish_receipt_set(output, protocol, manifest)
            observed_protocol, observed_manifest = common.validate_receipt_set(output)
            self.assertEqual(protocol, observed_protocol)
            self.assertEqual(manifest, observed_manifest)

            (output / "manifest.json").write_text(
                json.dumps({"schema_version": "tampered"}), encoding="utf-8"
            )
            with self.assertRaises(common.ManifestError):
                common.validate_receipt_set(output)

    def test_prepare_cli_publishes_a_valid_source_bound_receipt_set(self) -> None:
        prepare = load_module(PREPARE, "prepare_table5_sketch_test_target")
        common = load_module(COMMON, "table5_sketch_prepare_common_test_target")
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as directory:
            output = Path(directory) / "receipt"
            arguments = [
                str(PREPARE),
                "--dataset-root",
                str(DATASET_ROOT),
                "--table1-manifest",
                str(TABLE1_MANIFEST),
                "--table2-root",
                str(UPSTREAM_ROOTS["table2"]),
                "--table3-root",
                str(UPSTREAM_ROOTS["table3"]),
                "--table4-root",
                str(UPSTREAM_ROOTS["table4"]),
                "--protocol",
                str(PROTOCOL),
                "--out",
                str(output),
            ]
            original = sys.argv
            try:
                sys.argv = arguments
                self.assertEqual(0, prepare.main())
            finally:
                sys.argv = original

            protocol, manifest = common.validate_receipt_set(output)
            self.assertEqual(800, len(manifest["rows"]))
            self.assertEqual(
                protocol["protocol_sha256"], manifest["protocol_sha256"]
            )

    def test_runner_accepts_only_sketch_runtime_ids_bound_to_original_identity(self) -> None:
        runner = load_module(RUNNER, "run_table5_sketch_test_target")
        row = {
            "dataset_id": "sketch_0000",
            "order": 0,
            "asset_id": "data/Shape2Motion/Kettle/kettle_0057",
            "manifest_root": "data/Shape2Motion/Kettle/kettle_0057",
            "selection_rank": 1,
            "selection_hash": "0" * 64,
        }

        runner._validate_row_identity(row)
        runner.validate_runtime_protocol(
            load_module(COMMON, "table5_sketch_runner_common_test_target")
            .validate_canonical_protocol(PROTOCOL)
        )

        row["dataset_id"] = "artiverse_0000"
        with self.assertRaises(runner.RuntimeContractError):
            runner._validate_row_identity(row)

    def test_runner_retains_missing_bbox_preflight_failure_with_valid_joint_tree(self) -> None:
        runner = load_module(RUNNER, "run_table5_sketch_bbox_gate_test_target")
        row = {
            "preflight": {
                "status": "failed",
                "issues": ["missing_bounding_box"],
                "simulator_eligible": False,
            },
            "joint_tree": {"links": ["base"], "joints": [], "root_links": ["base"]},
        }

        try:
            eligible = runner._preflight_eligible(row)
        except runner.RuntimeContractError as error:
            self.fail(f"valid joint tree must be retained for bbox gate failure: {error}")
        self.assertFalse(eligible)

    def test_aggregate_cli_accepts_only_canonical_sketch_ids(self) -> None:
        aggregate = load_module(AGGREGATE, "aggregate_table5_sketch_test_target")

        self.assertEqual(
            ["sketch_0000", "sketch_0799"],
            aggregate._parse_ids("sketch_0000,sketch_0799"),
        )
        with self.assertRaises(aggregate.AggregateContractError):
            aggregate._parse_ids("artiverse_0000")
        with self.assertRaises(aggregate.AggregateContractError):
            aggregate._parse_ids("sketch_0800")

    def test_published_verifier_rejects_incomplete_formal_result(self) -> None:
        verifier = load_module(VERIFY, "verify_table5_sketch_incomplete_test_target")
        source = Path(
            "/root/.cache/torch/arti-skill/"
            "table5_sketch_mobility_table1_n800_gpu_v1"
        )
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as directory:
            isolated = Path(directory)
            shutil.copy2(source / "protocol.json", isolated / "protocol.json")
            shutil.copy2(source / "manifest.json", isolated / "manifest.json")
            result = verifier.verify_publication(
                isolated,
                phase="formal",
                table1_manifest=TABLE1_MANIFEST,
            )

        self.assertEqual("FAIL", result["status"])
        self.assertFalse(result["checks"]["all_simulator_summaries_complete"])
        self.assertFalse(result["checks"]["aggregate_publication_complete"])

    def test_published_verifier_qualification_uses_declared_five_asset_intent(self) -> None:
        verifier = load_module(VERIFY, "verify_table5_sketch_intent_test_target")
        rows = [
            {
                "dataset_id": f"sketch_{index:04d}",
                "manifest_root": f"data/source/asset_{index:04d}",
            }
            for index in range(8)
        ]
        chosen = ["sketch_0000", "sketch_0001", "sketch_0004", "sketch_0006", "sketch_0007"]
        table5 = {
            "intent": {
                "count": 5,
                "dataset_ids": chosen,
                "manifest_roots": [rows[index]["manifest_root"] for index in (0, 1, 4, 6, 7)],
            }
        }

        intent_ids, intent_rows = verifier._phase_intent(
            phase="qualification", table5=table5, rows=rows
        )

        self.assertEqual(chosen, intent_ids)
        self.assertEqual(chosen, [row["dataset_id"] for row in intent_rows])

    def test_published_verifier_rejects_noncanonical_protocol_semantics(self) -> None:
        verifier = load_module(VERIFY, "verify_table5_sketch_protocol_test_target")
        canonical = json.loads(PROTOCOL.read_text(encoding="utf-8"))
        published = copy.deepcopy(canonical)
        published["protocol_sha256"] = verifier.canonical_sha256(
            published, exclude_fields={"protocol_sha256", "generated_at"}
        )
        self.assertEqual([], verifier._canonical_protocol_errors(published))

        published["selection"]["selected_count"] = 799
        published["protocol_sha256"] = verifier.canonical_sha256(
            published, exclude_fields={"protocol_sha256", "generated_at"}
        )

        self.assertTrue(verifier._canonical_protocol_errors(published))

    def test_published_verifier_rejects_extra_aggregate_marker_hash_key(self) -> None:
        verifier = load_module(VERIFY, "verify_table5_sketch_aggregate_marker_test_target")
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as directory:
            root = Path(directory)
            required = {
                "table5.json": "table5",
                "failure_inventory.json": "inventory",
                "report.md": "report",
                "self_check.json": "self-check",
            }
            for name, contents in required.items():
                (root / name).write_text(contents, encoding="utf-8")
            marker = {
                "schema_version": "table5_sketch_mobility_aggregate_publication_v1",
                "run_phase": "qualification",
                "protocol_sha256": "1" * 64,
                "cohort_sha256": "2" * 64,
                "file_hashes": {
                    name: verifier.sha256_file(root / name) for name in required
                },
            }
            self.assertEqual(
                [],
                verifier._aggregate_marker_errors(
                    root,
                    marker,
                    phase="qualification",
                    protocol_sha256="1" * 64,
                    cohort_sha256="2" * 64,
                ),
            )
            (root / "extra.json").write_text("{}", encoding="utf-8")
            marker["file_hashes"]["extra.json"] = verifier.sha256_file(
                root / "extra.json"
            )

            self.assertTrue(
                verifier._aggregate_marker_errors(
                    root,
                    marker,
                    phase="qualification",
                    protocol_sha256="1" * 64,
                    cohort_sha256="2" * 64,
                )
            )

    def test_published_verifier_terminal_contract_rejects_extra_metric(self) -> None:
        verifier = load_module(VERIFY, "verify_table5_sketch_terminal_test_target")
        receipt_root = Path(
            "/root/.cache/torch/arti-skill/"
            "table5_sketch_mobility_table1_n800_gpu_v1"
        )
        protocol = json.loads((receipt_root / "protocol.json").read_text(encoding="utf-8"))
        manifest = json.loads((receipt_root / "manifest.json").read_text(encoding="utf-8"))
        row = manifest["rows"][0]
        record = json.loads(
            (receipt_root / "formal/pybullet/assets/sketch_0000.json").read_text(
                encoding="utf-8"
            )
        )
        self.assertEqual(
            [],
            verifier._terminal_record_errors(
                record,
                row=row,
                simulator="pybullet",
                phase="formal",
                protocol=protocol,
                manifest=manifest,
            ),
        )

        record["metrics"]["invented"] = True

        self.assertTrue(
            verifier._terminal_record_errors(
                record,
                row=row,
                simulator="pybullet",
                phase="formal",
                protocol=protocol,
                manifest=manifest,
            )
        )

        record = json.loads(
            (receipt_root / "formal/pybullet/assets/sketch_0000.json").read_text(
                encoding="utf-8"
            )
        )
        record["identity"]["asset_id"] = "data/forged/asset"
        self.assertTrue(
            verifier._terminal_record_errors(
                record,
                row=row,
                simulator="pybullet",
                phase="formal",
                protocol=protocol,
                manifest=manifest,
            )
        )

    def test_published_verifier_recomputes_completed_terminal_evidence(self) -> None:
        verifier = load_module(VERIFY, "verify_table5_sketch_completed_evidence_test_target")
        receipt_root = Path(
            "/root/.cache/torch/arti-skill/"
            "table5_sketch_mobility_table1_n800_gpu_v1"
        )
        protocol = json.loads((receipt_root / "protocol.json").read_text(encoding="utf-8"))
        manifest = json.loads((receipt_root / "manifest.json").read_text(encoding="utf-8"))
        row = manifest["rows"][0]
        record_path = receipt_root / "formal/pybullet/assets/sketch_0000.json"
        record = json.loads(record_path.read_text(encoding="utf-8"))
        self.assertEqual("completed", record["terminal_status"])
        self.assertEqual(
            [],
            verifier._terminal_record_errors(
                record,
                row=row,
                simulator="pybullet",
                phase="formal",
                protocol=protocol,
                manifest=manifest,
            ),
        )

        corruptions = {
            "support": lambda value: value["support"].update(joints=[]),
            "diagnostics": lambda value: value["diagnostics"].update(reset=[]),
            "process": lambda value: value["process"].update(exit_code=None),
        }
        for label, corrupt in corruptions.items():
            with self.subTest(label=label):
                forged = copy.deepcopy(record)
                corrupt(forged)
                self.assertTrue(
                    verifier._terminal_record_errors(
                        forged,
                        row=row,
                        simulator="pybullet",
                        phase="formal",
                        protocol=protocol,
                        manifest=manifest,
                    )
                )

    def test_published_verifier_rejects_malformed_diagnostic_failure_evidence(self) -> None:
        verifier = load_module(VERIFY, "verify_table5_sketch_diagnostic_failure_test_target")
        receipt_root = Path(
            "/root/.cache/torch/arti-skill/"
            "table5_sketch_mobility_table1_n800_gpu_v1"
        )
        protocol = json.loads((receipt_root / "protocol.json").read_text(encoding="utf-8"))
        manifest = json.loads((receipt_root / "manifest.json").read_text(encoding="utf-8"))
        record = json.loads(
            (receipt_root / "formal/mujoco/assets/sketch_0049.json").read_text(
                encoding="utf-8"
            )
        )
        row = manifest["rows"][record["identity"]["order"]]
        self.assertEqual("diagnostic_failure", record["terminal_status"])
        self.assertEqual(
            [],
            verifier._terminal_record_errors(
                record,
                row=row,
                simulator="mujoco",
                phase="formal",
                protocol=protocol,
                manifest=manifest,
            ),
        )

        del record["diagnostics"]["diagnostic_failure"]["operation"]
        self.assertTrue(
            verifier._terminal_record_errors(
                record,
                row=row,
                simulator="mujoco",
                phase="formal",
                protocol=protocol,
                manifest=manifest,
            )
        )

    def test_published_verifier_enforces_genesis_parent_and_child_gpu_gates(self) -> None:
        verifier = load_module(VERIFY, "verify_table5_sketch_gpu_receipt_test_target")
        parent = {
            "schema_version": "table5_sketch_mobility_parent_gpu_gate_receipt_v1",
            **verifier.FROZEN_GENESIS_GPU_HARDWARE,
            "used_memory_mib": 76,
            "free_memory_mib": 143092,
            "utilization_percent": 0,
            "compute_pids": [],
        }
        self.assertEqual([], verifier._parent_gpu_errors(parent))
        busy = copy.deepcopy(parent)
        busy.update(
            used_memory_mib=99999,
            free_memory_mib=0,
            utilization_percent=100,
            compute_pids=[123],
        )
        self.assertTrue(verifier._parent_gpu_errors(busy))

        child = copy.deepcopy(parent)
        child["schema_version"] = (
            "table5_sketch_mobility_child_gpu_gate_receipt_v1"
        )
        child["worker_pid"] = 123
        child["compute_pids"] = [123]
        device = {
            "schema_version": "table5_sketch_mobility_genesis_device_receipt_v1",
            "backend": "cuda",
            "logical_device": "cuda:0",
            "logical_device_count": 1,
            "cuda_visible_devices": "3",
            "cuda_device_order": "PCI_BUS_ID",
            "physical_device_index": 3,
            "visible_device_index": 0,
            "nvidia_smi_gpu_uuid": verifier.FROZEN_GENESIS_GPU_HARDWARE["gpu_uuid"],
            "torch_gpu_uuid": "ebc0d328-a3fa-7e89-2733-cadb001661f7",
            "normalized_gpu_uuid": "ebc0d328a3fa7e892733cadb001661f7",
            "nvidia_smi_device_name": "NVIDIA L20X",
            "nvidia_smi_total_memory_mib": 143771,
            "driver_version": "570.172.08",
            "nvidia_smi_compute_capability": "8.9",
            "torch_device_name": "NVIDIA L20X",
            "torch_total_memory_bytes": 150121021440,
            "torch_total_memory_mib": 143166,
            "torch_compute_capability": "9.0",
            "torch_version": "2.8.0+cu128",
            "torch_cuda_version": "12.8",
            "cudnn_version": 91002,
            "quadrants_version": "1.2.0",
            "python_version": "3.12.13",
            "child_gpu_gate_receipt": child,
        }
        self.assertEqual(
            [],
            verifier._device_receipt_errors(
                device, simulator="genesis", completed=True
            ),
        )
        self.assertTrue(
            verifier._device_receipt_errors(
                None, simulator="genesis", completed=True
            )
        )
        device["child_gpu_gate_receipt"]["compute_pids"].append(456)
        self.assertTrue(
            verifier._device_receipt_errors(
                device, simulator="genesis", completed=True
            )
        )

    def test_published_verifier_fresh_row_closure_reads_package_and_upstreams(self) -> None:
        verifier = load_module(VERIFY, "verify_table5_sketch_source_row_test_target")
        receipt_root = Path(
            "/root/.cache/torch/arti-skill/"
            "table5_sketch_mobility_table1_n800_gpu_v1"
        )
        manifest = json.loads((receipt_root / "manifest.json").read_text(encoding="utf-8"))
        table1 = json.loads(TABLE1_MANIFEST.read_text(encoding="utf-8"))
        row = manifest["rows"][0]
        asset_id = row["asset_id"]
        upstream = {}
        for name, key in (("table2", "asset_id"), ("table3", "asset_id"), ("table4", "dataset_id")):
            path = Path(manifest["upstream_artifacts"][name]["root"]) / "asset_records.jsonl"
            with path.open("r", encoding="utf-8") as handle:
                upstream[name] = next(
                    record
                    for record in (json.loads(line) for line in handle if line.strip())
                    if record.get(key) == asset_id
                )

        self.assertEqual(
            [],
            verifier._fresh_row_source_errors(
                row,
                table1_asset=table1["assets"][0],
                upstream=upstream,
                dataset_root=DATASET_ROOT,
                order=0,
            ),
        )

        invented = copy.deepcopy(row)
        invented["invented"] = True
        invented["row_sha256"] = verifier.canonical_sha256(
            invented, exclude_fields={"row_sha256"}
        )
        self.assertTrue(
            verifier._fresh_row_source_errors(
                invented,
                table1_asset=table1["assets"][0],
                upstream=upstream,
                dataset_root=DATASET_ROOT,
                order=0,
            )
        )

        forged_structure = copy.deepcopy(row)
        forged_structure["joint_names"] = ["forged_joint"]
        forged_structure["joint_tree"]["joints"][0]["parent"] = "forged_link"
        forged_structure["row_sha256"] = verifier.canonical_sha256(
            forged_structure, exclude_fields={"row_sha256"}
        )
        self.assertTrue(
            verifier._fresh_row_source_errors(
                forged_structure,
                table1_asset=table1["assets"][0],
                upstream=upstream,
                dataset_root=DATASET_ROOT,
                order=0,
            )
        )

        for field, mutate in (
            ("resources", lambda value: value.append({"invented": True})),
            ("collision", lambda value: value.update(element_count=999)),
            ("bounding_box", lambda value: value.update(protocol="forged")),
        ):
            with self.subTest(fresh_metadata=field):
                forged_metadata = copy.deepcopy(row)
                mutate(forged_metadata[field])
                forged_metadata["row_sha256"] = verifier.canonical_sha256(
                    forged_metadata, exclude_fields={"row_sha256"}
                )
                self.assertTrue(
                    verifier._fresh_row_source_errors(
                        forged_metadata,
                        table1_asset=table1["assets"][0],
                        upstream=upstream,
                        dataset_root=DATASET_ROOT,
                        order=0,
                    )
                )

        row = copy.deepcopy(row)
        row["strict_gates"]["table4"]["strict_collision_pass"] = not row[
            "strict_gates"
        ]["table4"]["strict_collision_pass"]
        self.assertTrue(
            verifier._fresh_row_source_errors(
                row,
                table1_asset=table1["assets"][0],
                upstream=upstream,
                dataset_root=DATASET_ROOT,
                order=0,
            )
        )

    def test_published_verifier_runtime_receipt_binds_every_record_file(self) -> None:
        verifier = load_module(VERIFY, "verify_table5_sketch_runtime_receipt_test_target")
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as directory:
            root = Path(directory)
            assets = root / "assets"
            assets.mkdir()
            record_path = assets / "sketch_0000.json"
            record_path.write_text('{"terminal":true}', encoding="utf-8")
            summary_path = root / "summary.json"
            progress_path = root / "progress.json"
            summary_path.write_text('{"complete":true}', encoding="utf-8")
            progress_path.write_text('{"complete":true}', encoding="utf-8")
            record_hashes = {
                "sketch_0000.json": verifier.sha256_file(record_path),
            }
            runtime_input = {
                "present": True,
                "effective_workers": 1,
                "intent_count": 1,
                "terminal_count": 1,
                "complete": True,
                "record_file_hashes": record_hashes,
                "record_set_sha256": verifier.canonical_sha256(
                    [
                        {
                            "filename": "sketch_0000.json",
                            "sha256": record_hashes["sketch_0000.json"],
                        }
                    ]
                ),
                "summary_sha256": verifier.sha256_file(summary_path),
                "progress_sha256": verifier.sha256_file(progress_path),
                "adapter_implementation_sha256": "3" * 64,
            }
            self.assertEqual(
                [],
                verifier._simulator_runtime_receipt_errors(
                    root,
                    runtime_input,
                    simulator="genesis",
                    phase="qualification",
                    intent_ids=["sketch_0000"],
                    records={
                        "sketch_0000": {
                            "identity": {
                                "adapter_implementation_sha256": "3" * 64,
                                "effective_workers": 1,
                            }
                        }
                    },
                ),
            )

            runtime_input["effective_workers"] = 2
            self.assertTrue(
                verifier._simulator_runtime_receipt_errors(
                    root,
                    runtime_input,
                    simulator="genesis",
                    phase="qualification",
                    intent_ids=["sketch_0000"],
                    records={
                        "sketch_0000": {
                            "identity": {
                                "adapter_implementation_sha256": "3" * 64,
                                "effective_workers": 1,
                            }
                        }
                    },
                )
            )
            runtime_input["effective_workers"] = 1

            record_path.write_text('{"terminal":false}', encoding="utf-8")

            self.assertTrue(
                verifier._simulator_runtime_receipt_errors(
                    root,
                    runtime_input,
                    simulator="genesis",
                    phase="qualification",
                    intent_ids=["sketch_0000"],
                    records={
                        "sketch_0000": {
                            "identity": {
                                "adapter_implementation_sha256": "3" * 64,
                                "effective_workers": 1,
                            }
                        }
                    },
                )
            )

    def test_published_verifier_recomputes_all_headline_table5_cells(self) -> None:
        verifier = load_module(VERIFY, "verify_table5_sketch_tables_test_target")
        ids = ["sketch_0000", "sketch_0001"]
        rows = [
            {
                "dataset_id": dataset_id,
                "asset_id": f"data/source/{dataset_id}",
                "manifest_root": f"data/source/{dataset_id}",
                "preflight": {
                    "status": "failed",
                    "issues": ["missing_bounding_box"],
                    "simulator_eligible": False,
                },
                "joint_tree": None,
                "joints": [],
                "strict_gates": {
                    "table2": {"strict_urdf_pass": index == 0},
                    "table3": {"strict_kinematic_pass": True},
                    "table4": {"strict_collision_pass": index == 1},
                },
            }
            for index, dataset_id in enumerate(ids)
        ]
        records = {
            simulator: {
                dataset_id: {
                    "terminal_status": "preflight_failure",
                    "identity": {
                        "dataset_id": dataset_id,
                        "asset_id": row["asset_id"],
                        "manifest_root": row["manifest_root"],
                    },
                    "metrics": {metric: False for metric in verifier.METRICS},
                }
                for dataset_id, row in zip(ids, rows)
            }
            for simulator in verifier.SIMULATORS
        }
        protocol = json.loads(PROTOCOL.read_text(encoding="utf-8"))

        table5a, table5b, outcomes = verifier._recompute_headline_tables(
            protocol=protocol,
            rows_by_id={row["dataset_id"]: row for row in rows},
            records_by_simulator=records,
            intent_ids=ids,
        )

        self.assertEqual(
            {"passed": 0, "denominator": 2, "percentage": 0.0},
            table5a["pybullet"]["simulator_pass"],
        )
        self.assertEqual(
            {"passed": 1, "denominator": 2, "percentage": 50.0},
            table5b["strict_urdf_pass"],
        )
        self.assertEqual(
            {"passed": 2, "denominator": 2, "percentage": 100.0},
            table5b["strict_kinematic_pass"],
        )
        self.assertEqual(
            {"passed": 0, "denominator": 2, "percentage": 0.0},
            table5b["strict_sim_ready"],
        )
        self.assertEqual(ids, list(outcomes))

    def test_published_verifier_recomputes_categories_and_inventory_counts(self) -> None:
        verifier = load_module(VERIFY, "verify_table5_sketch_category_inventory_test_target")
        ids = ["sketch_0000", "sketch_0001"]
        rows = [
            {
                "dataset_id": dataset_id,
                "asset_id": f"data/source/{dataset_id}",
                "manifest_root": f"data/source/{dataset_id}",
                "raw_category": f"category_{index}",
                "preflight": {
                    "status": "failed",
                    "issues": ["missing_bounding_box"],
                    "simulator_eligible": False,
                },
                "joint_tree": None,
                "joints": [],
                "strict_gates": {
                    "table2": {"strict_urdf_pass": index == 0},
                    "table3": {"strict_kinematic_pass": True},
                    "table4": {"strict_collision_pass": index == 1},
                },
            }
            for index, dataset_id in enumerate(ids)
        ]
        records = {
            simulator: {
                dataset_id: {
                    "terminal_status": "preflight_failure",
                    "identity": {
                        "dataset_id": dataset_id,
                        "asset_id": row["asset_id"],
                        "manifest_root": row["manifest_root"],
                    },
                    "metrics": {metric: False for metric in verifier.METRICS},
                    "diagnostics": {"actuation": []},
                }
                for dataset_id, row in zip(ids, rows)
            }
            for simulator in verifier.SIMULATORS
        }
        protocol = json.loads(PROTOCOL.read_text(encoding="utf-8"))
        rows_by_id = {row["dataset_id"]: row for row in rows}
        table5a, table5b, outcomes = verifier._recompute_headline_tables(
            protocol=protocol,
            rows_by_id=rows_by_id,
            records_by_simulator=records,
            intent_ids=ids,
        )
        categories = verifier._recompute_categories(
            protocol=protocol,
            rows_by_id=rows_by_id,
            records_by_simulator=records,
            intent_ids=ids,
            outcomes=outcomes,
        )
        self.assertEqual("micro", categories["headline"])
        self.assertEqual(2, categories["category_count"])
        self.assertEqual(
            ["singleton", "small_group"], categories["groups"][0]["warnings"]
        )
        self.assertEqual(table5a, categories["micro"]["table5a"])
        for field in verifier.TABLE5B_RATE_FIELDS:
            self.assertEqual(table5b[field], categories["micro"]["table5b"][field])

        forged = copy.deepcopy(categories)
        forged["headline"] = "macro"
        self.assertTrue(verifier._category_errors(forged, categories))

        counts = verifier._recompute_diagnostic_counts(
            protocol=protocol,
            rows_by_id=rows_by_id,
            records_by_simulator=records,
            intent_ids=ids,
        )
        self.assertEqual(6, counts["strict_consistency_entries"])
        self.assertEqual(0, counts["constraint_drift_entries"])
        inventory = {
            "joint_diagnostics": [],
            "pose_diagnostics": [],
            "constraint_drift": [],
            "strict_consistency": [{}] * 6,
        }
        self.assertEqual(
            [], verifier._diagnostic_count_errors(counts, inventory, counts)
        )
        forged_counts = copy.deepcopy(counts)
        forged_counts["strict_consistency_entries"] = 0
        self.assertTrue(
            verifier._diagnostic_count_errors(counts, inventory, forged_counts)
        )
        record_inventory = verifier._recompute_record_inventory(
            rows_by_id=rows_by_id,
            records_by_simulator=records,
            intent_ids=ids,
        )
        self.assertEqual(6, len(record_inventory))
        self.assertEqual(
            [], verifier._record_inventory_errors(record_inventory, record_inventory)
        )
        self.assertTrue(
            verifier._record_inventory_errors(record_inventory[:-1], record_inventory)
        )

    def test_published_verifier_rejects_report_headline_tampering(self) -> None:
        verifier = load_module(VERIFY, "verify_table5_sketch_report_test_target")
        rate = {"passed": 1, "denominator": 2, "percentage": 50.0}
        table5a = {
            simulator: {
                **{metric: copy.deepcopy(rate) for metric in verifier.METRICS},
                "strict_collision_pass": copy.deepcopy(rate),
            }
            for simulator in verifier.SIMULATORS
        }
        table5b = {
            "per_simulator_pass": {
                simulator: copy.deepcopy(rate) for simulator in verifier.SIMULATORS
            },
            **{
                field: copy.deepcopy(rate)
                for field in verifier.TABLE5B_RATE_FIELDS
                if field != "per_simulator_pass"
            },
            "joint_rmse": {
                joint_type: {"evaluable_units": 0, "population_max": None}
                for joint_type in ("revolute", "prismatic")
            },
            "link_pose_error": {"evaluable_units": 0},
        }
        table5 = {
            "run_phase": "qualification",
            "report_kind": "non_formal",
            "state": "complete",
            "formal_claim_complete": False,
            "intent": {"count": 2},
            "table5a": table5a,
            "table5b": table5b,
            "categories": {"small_group_threshold": 5},
        }
        expected = verifier._expected_report_markdown(table5)
        self.assertEqual([], verifier._report_errors(expected, table5))
        self.assertTrue(
            verifier._report_errors(
                expected.replace("Strict Sim-ready: 1/2", "Strict Sim-ready: 2/2"),
                table5,
            )
        )
    def test_published_verifier_returns_structured_fail_for_malformed_nested_json(self) -> None:
        verifier = load_module(VERIFY, "verify_table5_sketch_malformed_test_target")
        receipt_root = Path(
            "/root/.cache/torch/arti-skill/"
            "table5_sketch_mobility_table1_n800_gpu_v1"
        )
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as directory:
            root = Path(directory)
            shutil.copyfile(receipt_root / "protocol.json", root / "protocol.json")
            manifest = json.loads(
                (receipt_root / "manifest.json").read_text(encoding="utf-8")
            )
            manifest["rows"][0] = []
            (root / "manifest.json").write_text(
                json.dumps(manifest, sort_keys=True, separators=(",", ":")),
                encoding="utf-8",
            )
            marker = {
                "schema_version": "table5_sketch_mobility_receipt_set_v1",
                "protocol_sha256": verifier.sha256_file(root / "protocol.json"),
                "manifest_sha256": verifier.sha256_file(root / "manifest.json"),
            }
            (root / "receipt_set.json").write_text(
                json.dumps(marker, sort_keys=True, separators=(",", ":")),
                encoding="utf-8",
            )

            result = verifier.verify_publication(
                root, phase="formal", table1_manifest=TABLE1_MANIFEST
            )

        self.assertEqual("FAIL", result["status"])
        self.assertTrue(result["errors"])


if __name__ == "__main__":
    unittest.main()
