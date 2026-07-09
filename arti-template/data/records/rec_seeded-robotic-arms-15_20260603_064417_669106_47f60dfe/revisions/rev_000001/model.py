from __future__ import annotations

from agent.templates.robotic_arms import (
    build_robotic_arms,
    config_from_seed,
    run_robotic_arms_tests,
)
from sdk import AssetContext

SEED = 15
CONFIG = config_from_seed(SEED)
ASSETS = AssetContext.from_script(__file__)


def build_object_model():
    return build_robotic_arms(CONFIG, assets=ASSETS)


def run_tests():
    return run_robotic_arms_tests(object_model, CONFIG)


object_model = build_object_model()
