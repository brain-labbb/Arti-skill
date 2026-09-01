#!/usr/bin/env python3

from __future__ import annotations

from pathlib import Path
import sys
import unittest


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import table5_functional_motion as motion


def spec(name: str, depth: int) -> dict[str, object]:
    return {
        "name": name,
        "type": "revolute",
        "parent": "base",
        "child": name,
        "lower": 0.0,
        "upper": 1.0,
        "neutral": 0.0,
        "low": 0.1,
        "high": 0.9,
        "depth": depth,
    }


class FunctionalMotionPlannerTests(unittest.TestCase):
    def test_door_is_opened_before_blocked_drawer(self) -> None:
        specs = [spec("door", 0), spec("drawer", 0)]
        indices = {"door": 0, "drawer": 1}

        def observe(values: tuple[float, ...]) -> dict[str, object]:
            drawer_extended = values[1] > 0.2
            door_closed = values[0] < 0.8
            return {"valid": not (drawer_extended and door_closed)}

        result = motion.plan_dependency_aware_motion(
            specs, (0.0, 0.0), indices, observe
        )

        self.assertTrue(result["passed"])
        self.assertEqual(
            [step["joint_name"] for step in result["plan"]],
            ["door", "drawer"],
        )

    def test_fold_chain_opens_from_root_to_tip(self) -> None:
        specs = [spec("fold_0", 0), spec("fold_1", 1), spec("fold_2", 2)]
        indices = {"fold_0": 0, "fold_1": 1, "fold_2": 2}

        def observe(values: tuple[float, ...]) -> dict[str, object]:
            valid = not (
                (values[1] > 0.2 and values[0] < 0.8)
                or (values[2] > 0.2 and values[1] < 0.8)
            )
            return {"valid": valid}

        result = motion.plan_dependency_aware_motion(
            specs, (0.0, 0.0, 0.0), indices, observe
        )

        self.assertTrue(result["passed"])
        self.assertEqual(
            [step["joint_name"] for step in result["plan"]],
            ["fold_0", "fold_1", "fold_2"],
        )

    def test_both_doors_open_before_inner_slide(self) -> None:
        specs = [spec("left_door", 0), spec("right_door", 0), spec("slide", 1)]
        indices = {"left_door": 0, "right_door": 1, "slide": 2}

        def observe(values: tuple[float, ...]) -> dict[str, object]:
            slide_extended = values[2] > 0.2
            both_doors_open = values[0] >= 0.8 and values[1] >= 0.8
            return {"valid": not slide_extended or both_doors_open}

        result = motion.plan_dependency_aware_motion(
            specs, (0.0, 0.0, 0.0), indices, observe
        )

        self.assertTrue(result["passed"])
        self.assertEqual(
            [step["joint_name"] for step in result["plan"]],
            ["left_door", "right_door", "slide"],
        )

    def test_outer_lid_unlocks_multiple_inner_branches(self) -> None:
        specs = [spec("lid", 0), spec("left_tray", 1), spec("right_tray", 1)]
        indices = {"lid": 0, "left_tray": 1, "right_tray": 2}

        def observe(values: tuple[float, ...]) -> dict[str, object]:
            inner_branch_moved = values[1] > 0.2 or values[2] > 0.2
            return {"valid": not inner_branch_moved or values[0] >= 0.8}

        result = motion.plan_dependency_aware_motion(
            specs, (0.0, 0.0, 0.0), indices, observe
        )

        self.assertTrue(result["passed"])
        self.assertEqual(result["plan"][0]["joint_name"], "lid")
        self.assertEqual(
            {step["joint_name"] for step in result["plan"][1:]},
            {"left_tray", "right_tray"},
        )

    def test_search_keeps_prerequisite_at_required_endpoint(self) -> None:
        blocker = spec("blocker", 0)
        blocker["neutral"] = 0.5
        specs = [blocker, spec("drawer", 1)]
        indices = {"blocker": 0, "drawer": 1}

        def observe(values: tuple[float, ...]) -> dict[str, object]:
            drawer_extended = values[1] > 0.2
            return {"valid": not drawer_extended or values[0] <= 0.2}

        result = motion.plan_dependency_aware_motion(
            specs, (0.5, 0.0), indices, observe
        )

        self.assertTrue(result["passed"])
        self.assertEqual(
            [(step["joint_name"], step["direction"]) for step in result["plan"]],
            [("blocker", "high_to_low"), ("drawer", "low_to_high")],
        )

    def test_unmapped_joint_fails_closed(self) -> None:
        result = motion.plan_dependency_aware_motion(
            [spec("door", 0)], (0.0,), {}, lambda _: {"valid": True}
        )

        self.assertFalse(result["passed"])
        self.assertEqual(result["reason"], "eligible_joint_unmapped")

    def test_invalid_neutral_fails_before_search(self) -> None:
        result = motion.plan_dependency_aware_motion(
            [spec("door", 0)],
            (0.0,),
            {"door": 0},
            lambda _: {"valid": False, "reason": "neutral_penetration"},
        )

        self.assertFalse(result["passed"])
        self.assertEqual(result["reason"], "neutral_penetration")
        self.assertEqual(result["collision_queries"], 1)


if __name__ == "__main__":
    unittest.main()
