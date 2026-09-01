#!/usr/bin/env python3
"""Focused tests for the expanded-N hierarchy aggregator."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import random
import unittest


MODULE_PATH = Path(__file__).with_name("summarize_hierarchy_expanded_n150.py")
SPEC = importlib.util.spec_from_file_location("expanded_aggregate", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def synthetic_rows(method: str) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for category in MODULE.CATEGORIES:
        for rank in range(30):
            available = rank != 29
            valid = available and rank != 28
            scorable = rank < 27
            role_coverage = rank / 29
            induced_f1 = 0.75 if scorable else None
            rows.append(
                {
                    "method": method,
                    "sample_id": f"{method}/{category}/{rank}",
                    "category": category,
                    "available": available,
                    "valid_tree": valid,
                    "node_count": 10 if valid else None,
                    "semantic_depth": 3 if valid else None,
                    "movable_edge_count": 4 if valid else None,
                    "scorable": scorable,
                    "semantic_role_coverage": role_coverage if available else 0.0,
                    "induced_parent_child_edge_f1": induced_f1,
                    "induced_hierarchy_exact_match": bool(scorable),
                    "semantic_nesting_accuracy": 0.5 if scorable else None,
                }
            )
    return rows


class ExpandedAggregateTest(unittest.TestCase):
    def test_verification_gate_accepts_supported_pass_schemas(self) -> None:
        self.assertTrue(
            MODULE.verification_document_passed(
                {"status": "PASS", "checks": ["audited"]}
            )
        )
        self.assertTrue(
            MODULE.verification_document_passed(
                {"passed": True, "checks": {"hashes_match": True}}
            )
        )
        self.assertTrue(
            MODULE.verification_document_passed(
                {
                    "checks": {"main": True, "replay": True},
                    "main_verification": {"status": "PASS"},
                    "replay_verification": {"status": "PASS"},
                }
            )
        )

    def test_verification_gate_rejects_failed_or_statusless_documents(self) -> None:
        self.assertFalse(
            MODULE.verification_document_passed(
                {"status": "FAIL", "checks": {"hashes_match": True}}
            )
        )
        self.assertFalse(
            MODULE.verification_document_passed(
                {"passed": True, "checks": {"hashes_match": False}}
            )
        )
        self.assertFalse(MODULE.verification_document_passed({}))

    def test_paper_validation_requires_every_expected_fragment(self) -> None:
        expected = {
            "main row": "| PV-A | 125/150 |",
            "claim boundary": "ontology alignment proxy",
        }
        self.assertEqual(
            MODULE.validate_expected_fragments(
                "| PV-A | 125/150 | ontology alignment proxy", expected
            ),
            2,
        )
        with self.assertRaisesRegex(ValueError, "claim boundary"):
            MODULE.validate_expected_fragments("| PV-A | 125/150 |", expected)

    def test_unique_index_rejects_duplicate_sample_ids(self) -> None:
        with self.assertRaisesRegex(ValueError, "duplicate sample_id"):
            MODULE.unique_index(
                [{"sample_id": "same"}, {"sample_id": "same"}],
                method="PV-A",
                source="alignment records",
            )

    def test_requested_metrics_keep_failures_in_denominator(self) -> None:
        rows = synthetic_rows("PV-A")
        metrics = MODULE.compute_metrics(rows)
        self.assertAlmostEqual(metrics["available_requested"], 29 / 30)
        self.assertAlmostEqual(metrics["valid_requested"], 28 / 30)
        self.assertAlmostEqual(metrics["scorable_requested"], 27 / 30)
        self.assertAlmostEqual(metrics["induced_edge_f1_requested"], 27 / 30 * 0.75)
        self.assertAlmostEqual(metrics["induced_exact_requested"], 27 / 30)
        self.assertEqual(metrics["node_count_mean_valid"], 10.0)

    def test_structure_metrics_use_equal_category_valid_macro(self) -> None:
        rows = synthetic_rows("PV-A")
        first_category = MODULE.CATEGORIES[0]
        for row in rows:
            if row["category"] == first_category:
                is_only_valid = row["sample_id"].endswith("/0")
                row["valid_tree"] = is_only_valid
                row["node_count"] = 100 if is_only_valid else None
                row["semantic_depth"] = 10 if is_only_valid else None
                row["movable_edge_count"] = 50 if is_only_valid else None
            elif row["valid_tree"]:
                row["node_count"] = 0
                row["semantic_depth"] = 0
                row["movable_edge_count"] = 0

        metrics = MODULE.compute_metrics(rows)
        self.assertEqual(metrics["node_count_mean_valid"], 20.0)
        self.assertEqual(metrics["semantic_depth_mean_valid"], 2.0)
        self.assertEqual(metrics["movable_joint_count_mean_valid"], 10.0)
        self.assertAlmostEqual(metrics["node_count_mean_pooled_valid"], 100 / 113)

    def test_stratified_resample_draws_thirty_per_category(self) -> None:
        rows = synthetic_rows("PV-A")
        sample = MODULE.stratified_resample(rows, random.Random(7))
        self.assertEqual(len(sample), 150)
        self.assertEqual(
            {category: sum(row["category"] == category for row in sample)
             for category in MODULE.CATEGORIES},
            {category: 30 for category in MODULE.CATEGORIES},
        )

    def test_structure_bootstrap_skips_undefined_all_invalid_category_draw(self) -> None:
        rows = synthetic_rows("PV-A")
        for row in rows:
            if row["category"] == MODULE.CATEGORIES[0]:
                row["available"] = False
                row["valid_tree"] = False
                row["node_count"] = None
                row["semantic_depth"] = None
                row["movable_edge_count"] = None
        intervals, _ = MODULE.bootstrap_methods(
            {"PV-A": rows}, replicates=3, seed=3
        )
        for metric in MODULE.STRUCTURE_BOOTSTRAP_METRICS:
            self.assertIsNotNone(intervals["PV-A"][metric]["estimate"])
            self.assertEqual(intervals["PV-A"][metric]["valid_replicate_count"], 3)
            self.assertEqual(len(intervals["PV-A"][metric]["ci95_percentile"]), 2)

    def test_pairwise_bootstrap_resamples_methods_independently(self) -> None:
        pva = synthetic_rows("PV-A")
        baseline = synthetic_rows("Articraft")
        audit: list[tuple[str, tuple[str, ...]]] = []
        intervals, _ = MODULE.bootstrap_methods(
            {"PV-A": pva, "Articraft": baseline},
            replicates=3,
            seed=11,
            resample_audit=audit,
        )
        by_rep: dict[str, tuple[str, ...]] = {}
        for label, ids in audit[:2]:
            by_rep[label] = ids
        self.assertEqual(set(by_rep), {"PV-A", "Articraft"})
        self.assertNotEqual(
            tuple(value.rsplit("/", 1)[-1] for value in by_rep["PV-A"]),
            tuple(value.rsplit("/", 1)[-1] for value in by_rep["Articraft"]),
        )
        self.assertEqual(
            set(MODULE.STRUCTURE_BOOTSTRAP_METRICS),
            {
                "valid_available",
                "node_count_mean_valid",
                "semantic_depth_mean_valid",
                "movable_joint_count_mean_valid",
            },
        )
        for metric in MODULE.STRUCTURE_BOOTSTRAP_METRICS:
            self.assertIn(metric, intervals["PV-A"])
            self.assertEqual(len(intervals["PV-A"][metric]["ci95_percentile"]), 2)

    def test_report_formats_pairwise_difference_as_percentage_points(self) -> None:
        payload = {
            "balanced_generated_method_panel": {
                "methods": {
                    method: {
                        "point_estimates": {
                            "available_count": 150,
                            "valid_tree_count": 150,
                            "valid_available": 1.0,
                            "node_count_mean_valid": 4.0,
                            "semantic_depth_mean_valid": 2.0,
                            "movable_joint_count_mean_valid": 1.0,
                        },
                        "bootstrap_intervals": {
                            metric: {"estimate": 0.5, "ci95_percentile": [0.4, 0.6]}
                            for metric in (
                                MODULE.METRICS
                                + MODULE.STRUCTURE_BOOTSTRAP_METRICS
                            )
                        },
                    }
                    for method in MODULE.METHODS
                },
                "pairwise_primary_metric_differences": {
                    "PV-A_minus_Articraft": {
                        "estimate": 0.012,
                        "ci95_percentile": [-0.034, 0.056],
                    }
                },
            }
        }
        report = MODULE.make_report(payload)
        self.assertIn("1.2 pp [-3.4, 5.6]", report)
        self.assertNotIn("percentage points / 100", report)


if __name__ == "__main__":
    unittest.main()
