from __future__ import annotations

import unittest
from pathlib import Path
import sys


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from table5_com_stability_aggregate import AggregateError, _common_protocol, _report
from table5_com_stability_runtime import signed_support_margin


class CenterOfMassStabilityTest(unittest.TestCase):
    def test_signed_margin_is_positive_inside(self) -> None:
        polygon = ((-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0))
        self.assertAlmostEqual(signed_support_margin((0.0, 0.0), polygon), 1.0)

    def test_signed_margin_is_negative_outside(self) -> None:
        polygon = ((-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0))
        self.assertAlmostEqual(signed_support_margin((2.0, 0.0), polygon), -1.0)

    def test_signed_margin_is_zero_on_boundary(self) -> None:
        polygon = ((-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0))
        self.assertAlmostEqual(signed_support_margin((1.0, 0.0), polygon), 0.0)

    def test_report_handles_missing_attributable_margins(self) -> None:
        report = _report(
            {
                "datasets": [
                    {
                        "dataset_name": "No Physics",
                        "n": 3,
                        "metrics": {
                            "physics_parameter_coverage": {"percentage": 0.0},
                            "dataset_attributable_com_support_margin": {
                                "coverage_percentage": 0.0,
                                "median_normalized_signed_margin": None,
                                "median_signed_margin_mm": None,
                            },
                            "dataset_attributable_com_static_stability": {
                                "percentage": 0.0,
                                "conditional_ready_percentage": None,
                            },
                            "engine_finalized_diagnostic": {
                                "com_support_margin": {
                                    "coverage_percentage": 100.0,
                                    "median_normalized_signed_margin": 0.25,
                                    "median_signed_margin_mm": 50.0,
                                },
                                "com_static_stability": {"percentage": 100.0},
                            },
                        },
                    }
                ]
            }
        )
        self.assertIn("| No Physics | 3 | 0.0% | 0.0% | N/A | 0.0% |", report)
        self.assertIn("| No Physics | N/A | N/A |", report)

    def test_common_protocol_uses_runtime_receipt(self) -> None:
        protocol = {
            "protocol_id": "table5-genesis-com-static-stability-20260901-v2",
            "protocol_sha256": "diagnostic-sha",
            "classification": "diagnostic",
        }
        self.assertEqual(_common_protocol([protocol, protocol]), protocol)

    def test_common_protocol_rejects_mixed_runtime_receipts(self) -> None:
        first = {"protocol_sha256": "first"}
        second = {"protocol_sha256": "second"}
        with self.assertRaises(AggregateError):
            _common_protocol([first, second])


if __name__ == "__main__":
    unittest.main()
