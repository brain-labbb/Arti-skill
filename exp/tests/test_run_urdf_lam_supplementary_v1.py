from __future__ import annotations

import json
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock


SCRIPT = Path(__file__).parents[1] / "scripts" / "run_urdf_lam_supplementary_v1.py"


def load_runner():
    import importlib.util

    spec = importlib.util.spec_from_file_location("lam_supplementary_runner_test", SCRIPT)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load supplementary runner")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class RunnerSchemaTests(unittest.TestCase):
    def test_unexecuted_state_is_terminal_and_fail_closed(self):
        runner = load_runner()
        item = {
            "asset_key": "synthetic:objects/demo/demo_000",
            "selection_rank": 1,
            "input_identity_sha256": "input",
        }
        state = runner.unexecuted_state(item, "slide", 0, 0.0, {"engine": "Genesis"}, "mapping unavailable")
        self.assertTrue(state["terminal"])
        self.assertFalse(state["executed"])
        self.assertIsNone(state["illegal_collision"])
        self.assertIsNone(state["clearance_normalized"])
        self.assertEqual(state["engine_protocol_id"], runner.ENGINE_PROTOCOL_ID)
        self.assertEqual(state["mapping"]["status"], "N/E")

    def test_manifest_hash_excludes_only_self_hash(self):
        runner = load_runner()
        manifest = {"protocol_id": runner.PROTOCOL_ID, "items": [], "manifest_content_sha256": "old"}
        expected = runner.canonical_sha256({"protocol_id": runner.PROTOCOL_ID, "items": []})
        self.assertEqual(runner._manifest_hash(manifest), expected)

    def test_nonzero_child_attempt_is_preserved_but_not_promoted(self):
        runner = load_runner()
        with tempfile.TemporaryDirectory(prefix="lam_child_protocol_test_") as temporary:
            root = Path(temporary)
            output = root / "output"
            output.mkdir()
            dataset = root / "dataset"
            dataset.mkdir()
            source_records = root / "source.jsonl"
            source_manifest = root / "source_manifest.json"
            source_records.write_text("{}\n", encoding="utf-8")
            source_manifest.write_text("{}\n", encoding="utf-8")
            item = {
                "asset_key": "synthetic:objects/demo/demo_000",
                "selection_rank": 1,
                "input_identity_sha256": "input",
                "source_record_sha256": "source",
            }
            source = SimpleNamespace(joints_by_key={}, joint_order={})
            diagnostic = {"diagnostic_only": True}

            def failed_child(command, **_kwargs):
                runner.atomic_json(
                    output / "child_attempts" / "rank_0001.json", diagnostic
                )
                return runner.subprocess.CompletedProcess(command, 7, stdout="child failed")

            with mock.patch.object(runner.subprocess, "run", side_effect=failed_child):
                result = runner._run_child_rank_subprocess(
                    output_root=output,
                    dataset_root=dataset,
                    source_records=source_records,
                    source_manifest=source_manifest,
                    rank=1,
                    item=item,
                    source=source,
                    runtime={
                        "engine": "genesis",
                        "cpu_affinity": sorted(os.sched_getaffinity(0))[:1],
                        "thread_environment": dict(runner.THREAD_ENV_VALUES),
                    },
                )

            attempt = json.loads(
                (output / "child_attempts" / "rank_0001.json").read_text(encoding="utf-8")
            )
            receipt = json.loads(
                (output / "children" / "rank_0001.json").read_text(encoding="utf-8")
            )
            self.assertEqual(attempt, diagnostic)
            self.assertEqual(receipt, result)
            self.assertEqual(receipt["asset_record"]["status"], "fail_closed")
            self.assertIn("returncode=7", receipt["asset_record"]["engine_failure"])


class GenesisAdapterSmokeTests(unittest.TestCase):
    def test_synthetic_urdf_contact_readback(self):
        """Exercise the pinned Genesis API on one tiny asset, never the formal cohort."""

        runner = load_runner()
        with tempfile.TemporaryDirectory(prefix="lam_genesis_test_") as temporary:
            cache = Path(temporary) / "genesis-cache"
            os.environ["GS_CACHE_FILE_PATH"] = str(cache)
            urdf = Path(temporary) / "synthetic.urdf"
            urdf.write_text(
                """<robot name="synthetic">
  <link name="base">
    <visual><geometry><box size="1 1 1"/></geometry></visual>
    <collision><geometry><box size="1 1 1"/></geometry></collision>
  </link>
  <link name="slider">
    <visual><geometry><box size="0.2 0.2 0.2"/></geometry></visual>
    <collision><geometry><box size="0.2 0.2 0.2"/></geometry></collision>
  </link>
  <joint name="slide" type="prismatic">
    <parent link="base"/><child link="slider"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="0.1" effort="1" velocity="1"/>
    <dynamics damping="0" friction="0"/>
  </joint>
</robot>
""",
                encoding="utf-8",
            )
            runtime = runner.genesis_runtime_binding()
            adapter = runner.GenesisTable4aAdapter(urdf, runtime)
            try:
                adapter.build()
                item = {
                    "asset_key": "synthetic:objects/demo/demo_000",
                    "selection_rank": 1,
                    "input_identity_sha256": "input",
                }
                state = adapter.state(item=item, joint_name="slide", sample_index=0, value=0.0)
                self.assertTrue(state["executed"])
                self.assertTrue(state["contact_readback"]["success"])
                self.assertTrue(state["readback"]["success"])
                self.assertGreaterEqual(state["max_eligible_penetration_m"], 0.0)
                self.assertIsNone(state["clearance_normalized"])
            finally:
                adapter.close()
                os.environ.pop("GS_CACHE_FILE_PATH", None)


if __name__ == "__main__":
    unittest.main()
