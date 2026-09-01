import importlib.util
from pathlib import Path
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "table5_cross_sim_summary.py"
SPEC = importlib.util.spec_from_file_location("table5_cross_sim_summary", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class RuntimeMetricsTest(unittest.TestCase):
    def test_load_uses_native_import_not_legacy_structure_match(self) -> None:
        record = {
            "evaluation": {
                "metrics": {
                    "load": False,
                    "reset": True,
                    "settling": True,
                    "actuation": True,
                    "limit_enforcement": True,
                    "constraint_drift": True,
                    "simulator_pass": False,
                },
                "v2": {"import": {"passed": True}},
            }
        }

        metrics = MODULE.runtime_metrics(record)

        self.assertTrue(metrics["load"])
        self.assertTrue(metrics["simulator_pass"])

    def test_failed_native_import_fails_load_and_simulator_pass(self) -> None:
        record = {
            "evaluation": {
                "metrics": {
                    "load": True,
                    "reset": True,
                    "settling": True,
                    "actuation": True,
                    "limit_enforcement": True,
                    "constraint_drift": True,
                    "simulator_pass": True,
                },
                "v2": {"import": {"passed": False}},
            }
        }

        metrics = MODULE.runtime_metrics(record)

        self.assertFalse(metrics["load"])
        self.assertFalse(metrics["simulator_pass"])


if __name__ == "__main__":
    unittest.main()
