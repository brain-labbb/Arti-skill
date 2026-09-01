#!/usr/bin/env python3

from __future__ import annotations

import unittest

from exp.scripts.replace_pva_max_joint_classes import (
    count_movable_joints,
    select_max_joint_rows,
)


class CountMovableJointsTests(unittest.TestCase):
    def test_counts_only_non_fixed_joint_declarations(self) -> None:
        xml = b"""<robot name="sample">
          <joint name="a" type="fixed" />
          <joint name="b" type="revolute" />
          <joint name="c" type="prismatic" />
          <joint name="d" type="continuous" />
        </robot>"""

        self.assertEqual(count_movable_joints(xml), 3)

    def test_rejects_non_robot_xml(self) -> None:
        with self.assertRaisesRegex(ValueError, "robot"):
            count_movable_joints(b"<not_robot />")


class SelectMaxJointRowsTests(unittest.TestCase):
    def test_selects_highest_joint_counts_in_descending_order(self) -> None:
        rows = [
            {"slug": "Fence", "asset_id": f"seed_{index:04d}"}
            for index in range(7)
        ]
        counts = {
            "seed_0000": 1,
            "seed_0001": 7,
            "seed_0002": 3,
            "seed_0003": 6,
            "seed_0004": 2,
            "seed_0005": 5,
            "seed_0006": 4,
        }

        selected = select_max_joint_rows(
            rows,
            counts,
            5,
            seed="fixed-tie-seed",
        )

        self.assertEqual(
            [(row["asset_id"], row["movable_joint_count"]) for row in selected],
            [
                ("seed_0001", 7),
                ("seed_0003", 6),
                ("seed_0005", 5),
                ("seed_0006", 4),
                ("seed_0002", 3),
            ],
        )

    def test_ties_are_reproducible_independent_of_input_order(self) -> None:
        rows = [
            {"slug": "Ferris", "asset_id": f"seed_{index:04d}"}
            for index in range(8)
        ]
        counts = {row["asset_id"]: 10 for row in rows}

        first = select_max_joint_rows(rows, counts, 5, seed="fixed-tie-seed")
        second = select_max_joint_rows(
            reversed(rows), counts, 5, seed="fixed-tie-seed"
        )

        self.assertEqual(first, second)


if __name__ == "__main__":
    unittest.main()
