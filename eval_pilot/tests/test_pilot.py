from __future__ import annotations

import importlib.util
from pathlib import Path

import networkx as nx

PILOT_PATH = Path(__file__).resolve().parents[1] / "pilot.py"
SPEC = importlib.util.spec_from_file_location("eval_pilot", PILOT_PATH)
assert SPEC and SPEC.loader
pilot = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(pilot)


def test_vision_name_candidates() -> None:
    assert pilot._vision_name_candidate("gpt-4-vision-preview")
    assert pilot._vision_name_candidate("gpt-4o")
    assert not pilot._vision_name_candidate("text-embedding-3-large")


def test_blender_pilot_uses_four_stable_views() -> None:
    assert pilot.BLENDER_VIEW_NAMES == ("front_iso", "right_iso", "rear_iso", "left_iso")


def test_pilot_tree_edit_distance_uses_joint_type() -> None:
    left = nx.DiGraph()
    left.add_node("root", incoming_joint="root")
    left.add_node("door", incoming_joint="revolute")
    left.add_edge("root", "door", joint_type="revolute")

    same = nx.DiGraph()
    same.add_node("base", incoming_joint="root")
    same.add_node("panel", incoming_joint="revolute")
    same.add_edge("base", "panel", joint_type="revolute")

    different = nx.DiGraph()
    different.add_node("base", incoming_joint="root")
    different.add_node("panel", incoming_joint="prismatic")
    different.add_edge("base", "panel", joint_type="prismatic")

    assert pilot.pilot_tree_edit_distance(left, same) == 0.0
    assert pilot.pilot_tree_edit_distance(left, different) > 0.0


def test_extract_json_accepts_fenced_payload() -> None:
    parsed = pilot._extract_json(
        """```json
        {"geometry":{"winner":"A"},"appearance":{"winner":"TIE"}}
        ```"""
    )
    assert parsed["geometry"]["winner"] == "A"
    assert parsed["appearance"]["winner"] == "TIE"


def test_same_source_pair_reports_concrete_asset_winner() -> None:
    pair = {
        "pair_id": "p",
        "left_asset_id": "seed_1",
        "right_asset_id": "seed_2",
        "left_source": "ours",
        "right_source": "ours",
    }
    results = [
        {
            "pair_id": "p",
            "order": "ab",
            "parsed": {"geometry": {"winner": "A"}, "appearance": {"winner": "B"}},
        },
        {
            "pair_id": "p",
            "order": "ba",
            "parsed": {"geometry": {"winner": "B"}, "appearance": {"winner": "A"}},
        },
    ]
    counts, details = pilot.aggregate_judgments([pair], results)
    assert counts["geometry"] == {"seed_1_win": 1}
    assert counts["appearance"] == {"seed_2_win": 1}
    assert details["p"]["geometry"]["winner_asset_id"] == "seed_1"
