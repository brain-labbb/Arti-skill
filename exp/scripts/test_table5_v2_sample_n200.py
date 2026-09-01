#!/usr/bin/env python3

from __future__ import annotations

import unittest

from table5_v2_sample_n200 import (
    _source_universe_file_sha256,
    eligibility_reason,
    normalize_label,
    selection_rank,
    semantic_exclusion,
)


class Table5V2SamplingTest(unittest.TestCase):
    def test_normalize_label(self) -> None:
        self.assertEqual(
            normalize_label("Urban Environment/Public-Toilet"),
            "urban_environment_public_toilet",
        )

    def test_declared_semantic_exclusions(self) -> None:
        self.assertEqual(semantic_exclusion("Fence", "asset-1"), "fence")
        self.assertEqual(semantic_exclusion("Cascade fences", "asset-1"), "fence")
        self.assertEqual(
            semantic_exclusion("Furniture Sofa-bed", "asset-2"), "sofa_bed"
        )
        self.assertEqual(
            semantic_exclusion("Urban Public Toilet", "asset-3"), "public_toilet"
        )
        self.assertIsNone(semantic_exclusion("Bathroom toilet", "asset-4"))
        self.assertIsNone(semantic_exclusion("Defence cabinet", "asset-5"))

    def test_joint_bounds_are_inclusive(self) -> None:
        self.assertEqual(
            eligibility_reason("cabinet", "a", 0), "movable_joint_count_lt_1"
        )
        self.assertIsNone(eligibility_reason("cabinet", "a", 1))
        self.assertIsNone(eligibility_reason("cabinet", "a", 20))
        self.assertEqual(
            eligibility_reason("cabinet", "a", 21), "movable_joint_count_gt_20"
        )
        self.assertEqual(
            eligibility_reason("cabinet", "a", None),
            "movable_joint_count_unavailable",
        )

    def test_rank_is_deterministic_and_dataset_scoped(self) -> None:
        kwargs = {"universe_sha256": "a" * 64, "seed": "fixed"}
        first = selection_rank("pva", "asset", **kwargs)
        self.assertEqual(first, selection_rank("pva", "asset", **kwargs))
        self.assertNotEqual(first, selection_rank("artiverse", "asset", **kwargs))

    def test_full_release_universe_uses_bound_jsonl_file_hash(self) -> None:
        universe = {
            "sha256": "a" * 64,
            "roster_jsonl_sha256": "b" * 64,
        }
        self.assertEqual(_source_universe_file_sha256(universe), "b" * 64)
        self.assertEqual(_source_universe_file_sha256({"sha256": "a" * 64}), "a" * 64)


if __name__ == "__main__":
    unittest.main()
