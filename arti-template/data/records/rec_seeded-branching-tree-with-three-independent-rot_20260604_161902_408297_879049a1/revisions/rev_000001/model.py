from __future__ import annotations

from agent.templates.branching_tree_with_three_independent_rotary_branches import (
    build_branching_tree_with_three_independent_rotary_branches,
    config_from_seed,
    run_branching_tree_with_three_independent_rotary_branches_tests,
)
from sdk import AssetContext

SEED = 9
CONFIG = config_from_seed(SEED)
ASSETS = AssetContext.from_script(__file__)


def build_object_model():
    return build_branching_tree_with_three_independent_rotary_branches(CONFIG, assets=ASSETS)


def run_tests():
    return run_branching_tree_with_three_independent_rotary_branches_tests(object_model, CONFIG)


object_model = build_object_model()
