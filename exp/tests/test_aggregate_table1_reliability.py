from __future__ import annotations

import importlib.util
import unittest
from pathlib import Path


EXP_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = EXP_ROOT / "scripts/aggregate_table1_reliability.py"


def load_aggregator():
    spec = importlib.util.spec_from_file_location("aggregate_table1_reliability", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


aggregator = load_aggregator()


class AggregateTable1ReliabilityTest(unittest.TestCase):
    def test_common_adapter_readiness_matches_mixed_protocol_evidence(self):
        protocol = {
            "execution_ready": False,
            "execution_readiness": {
                "method_adapters_ready": {
                    "pva": True,
                    "lam": False,
                    "articraft": True,
                }
            },
        }
        summary = {
            "execution_adapter_readiness": {
                "declared_execution_ready": False,
                "ready": False,
                "methods": {
                    method: {
                        "declared_ready": ready,
                        "ready": ready,
                    }
                    for method, ready in {
                        "pva": True,
                        "lam": False,
                        "articraft": True,
                    }.items()
                },
            }
        }
        self_check = {
            "adapter_readiness_excluded_from_gate": True,
            "checks": [
                {
                    "check_id": f"adapter.{method}.ready",
                    "scope": "adapter_readiness",
                    "passed": ready,
                }
                for method, ready in {
                    "pva": True,
                    "lam": False,
                    "articraft": True,
                }.items()
            ],
        }

        self.assertTrue(
            aggregator.common_adapter_readiness_matches_protocol(
                protocol,
                summary,
                self_check,
            )
        )

        drifted_summary = {
            **summary,
            "execution_adapter_readiness": {
                **summary["execution_adapter_readiness"],
                "methods": {
                    **summary["execution_adapter_readiness"]["methods"],
                    "pva": {"declared_ready": True, "ready": False},
                },
            },
        }
        self.assertFalse(
            aggregator.common_adapter_readiness_matches_protocol(
                protocol,
                drifted_summary,
                self_check,
            )
        )

        drifted_self_check = {
            **self_check,
            "checks": [
                {**row, "passed": False}
                if row["check_id"] == "adapter.pva.ready"
                else row
                for row in self_check["checks"]
            ],
        }
        self.assertFalse(
            aggregator.common_adapter_readiness_matches_protocol(
                protocol,
                summary,
                drifted_self_check,
            )
        )

    def test_repaired_lam_with_one_capability_blocker_matches_protocol(self):
        """Paid confirmation and execution-time credential gates are not capabilities."""

        protocol = {
            "execution_readiness": {
                "method_blockers": {
                    "lam": ["LAM result writer remains unwired"],
                }
            }
        }
        repaired_lam_summary = {
            "runtime_blockers": [
                "Paid confirmation is required before execution",
                "OpenAI credential is unavailable at execution time",
            ],
            "adapter_blockers": ["  LAM result writer\nremains unwired  "],
        }

        self.assertTrue(
            aggregator.capability_blockers_match_protocol(
                protocol,
                "lam",
                aggregator.lam_capability_blockers(repaired_lam_summary),
            )
        )

    def test_common_frozen_consistency_accepts_expanded_all_passing_checks(self):
        rows = [
            {"scope": "frozen_consistency", "passed": True}
            for _ in range(113)
        ]
        self.assertTrue(
            aggregator.common_frozen_consistency_ready(
                {
                    "status": "READY",
                    "frozen_consistency": {
                        "status": "READY",
                        "ready": True,
                        "passed_checks": 113,
                        "total_checks": 113,
                    },
                },
                {"checks": rows, "pass": True},
            )
        )

    def test_malformed_capability_blocker_record_fails_closed(self):
        self.assertFalse(
            aggregator.capability_blockers_match_protocol(
                {"execution_readiness": {"method_blockers": {"lam": ["writer"]}}},
                "lam",
                aggregator.lam_capability_blockers(
                    {"runtime_blockers": ["runtime"], "adapter_blockers": None}
                ),
            )
        )

    def test_named_boolean_self_check_accepts_expanded_exact_count(self):
        self.assertTrue(
            aggregator.named_boolean_self_check_ready(
                {
                    "status": "PASS",
                    "passed": 21,
                    "total": 21,
                    "checks": {f"check_{index}": True for index in range(21)},
                },
                minimum_checks=17,
            )
        )
        self.assertFalse(
            aggregator.named_boolean_self_check_ready(
                {
                    "status": "PASS",
                    "passed": 21,
                    "total": 21,
                    "checks": {
                        **{f"check_{index}": True for index in range(20)},
                        "check_20": False,
                    },
                },
                minimum_checks=17,
            )
        )


if __name__ == "__main__":
    unittest.main()
