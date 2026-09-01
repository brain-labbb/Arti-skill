from __future__ import annotations

import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "exp" / "scripts" / "run_t2_authoring_pilot.py"
SPEC = importlib.util.spec_from_file_location("run_t2_authoring_pilot", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
PILOT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PILOT)


class T2AuthoringPilotTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.temporary = tempfile.TemporaryDirectory()
        cls.output_root = Path(cls.temporary.name)
        cls.run_dir = PILOT.prepare(
            PILOT.DEFAULT_PROTOCOL,
            cls.output_root,
            "unit",
            PILOT.MODEL_PLACEHOLDER,
        )
        cls.rows = PILOT.read_jsonl(cls.run_dir / "run_manifest.jsonl")

    @classmethod
    def tearDownClass(cls) -> None:
        cls.temporary.cleanup()

    def test_frozen_matrix_contains_144_unique_runs(self) -> None:
        self.assertEqual(len(self.rows), 144)
        self.assertEqual(len({row["run_key"] for row in self.rows}), 144)
        self.assertEqual(
            {row["method_id"] for row in self.rows},
            {
                "naive_same_llm",
                "without_source_map",
                "without_template_design",
                "full_ours",
            },
        )

    def test_method_packets_hold_raw_records_constant_and_hide_reference(self) -> None:
        task_rows = [
            row
            for row in self.rows
            if row["task_slug"] == "pictureX_0611_garlic_press"
            and row["repeat_index"] == 0
        ]
        packets = {
            row["method_id"]: PILOT.load_json(self.run_dir / row["packet_path"])
            for row in task_rows
        }
        record_sets = {
            tuple(packet["allowed_inputs"]["source_record_ids"])
            for packet in packets.values()
        }
        self.assertEqual(len(record_sets), 1)
        for packet in packets.values():
            hidden = f"arti-template/agent/templates/{packet['task_slug']}.py"
            self.assertNotIn(hidden, packet["allowed_inputs"]["task_evidence_paths"])
            self.assertIn(hidden, packet["forbidden_inputs"]["paths"])

        source_map = "arti-template/articraft_template_authoring/source_maps/pictureX_0611_garlic_press.md"
        design = "arti-template/articraft_template_authoring/designs/pictureX_0611_garlic_press.json"
        self.assertNotIn(source_map, packets["naive_same_llm"]["allowed_inputs"]["task_evidence_paths"])
        self.assertNotIn(design, packets["naive_same_llm"]["allowed_inputs"]["task_evidence_paths"])
        self.assertIn(design, packets["without_source_map"]["allowed_inputs"]["task_evidence_paths"])
        self.assertIn(source_map, packets["without_template_design"]["allowed_inputs"]["task_evidence_paths"])
        self.assertIn(source_map, packets["full_ours"]["allowed_inputs"]["task_evidence_paths"])
        self.assertIn(design, packets["full_ours"]["allowed_inputs"]["task_evidence_paths"])

    def test_prepare_is_idempotent(self) -> None:
        resumed = PILOT.prepare(
            PILOT.DEFAULT_PROTOCOL,
            self.output_root,
            "unit",
            PILOT.MODEL_PLACEHOLDER,
        )
        self.assertEqual(resumed, self.run_dir)

    def test_status_accepts_one_valid_result_and_leaves_others_pending(self) -> None:
        row = self.rows[0]
        result_path = self.run_dir / row["result_path"]
        result_path.write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "run_key": row["run_key"],
                    "task_slug": row["task_slug"],
                    "method_id": row["method_id"],
                    "repeat_index": row["repeat_index"],
                    "status": "completed",
                    "backend": {"provider": "test", "model": "test-model"},
                    "metrics": {
                        "first_shot_pass": True,
                        "final_success": True,
                        "artifact_saved": True,
                        "repair_turns": 0,
                        "human_intervention": False,
                        "wall_seconds": 1.0,
                        "input_tokens": 10,
                        "output_tokens": 20,
                        "api_cost_usd": 0.0,
                    },
                    "output": {
                        "template_path": "output/template.py",
                        "template_sha256": "a" * 64,
                        "evaluator_report_path": "evaluator.json",
                    },
                }
            )
            + "\n",
            encoding="utf-8",
        )
        summary = PILOT.summarize(self.run_dir, write=False)
        self.assertEqual(summary["status_counts"]["completed"], 1)
        self.assertEqual(summary["status_counts"]["pending"], 143)
        self.assertEqual(summary["status_counts"]["invalid"], 0)


if __name__ == "__main__":
    unittest.main()
