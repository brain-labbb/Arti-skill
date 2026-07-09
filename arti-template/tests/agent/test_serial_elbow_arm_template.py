from __future__ import annotations

import pytest

from agent.templates.serial_elbow_arm import (
    SOURCE_ADAPTATION_MAP,
    SOURCE_IDS,
    SerialElbowArmConfig,
    build_seeded_serial_elbow_arm,
    build_serial_elbow_arm,
    config_from_seed,
    resolve_config,
    run_serial_elbow_arm_tests,
)
from sdk import TestContext as ArticraftTestContext


def _assert_basic_qc_passes(model) -> None:
    ctx = ArticraftTestContext(model)
    ctx.check_model_valid()
    ctx.fail_if_isolated_parts()
    report = ctx.report()
    assert report.passed, report.failures


def test_seed_config_is_reproducible() -> None:
    assert config_from_seed(7) == config_from_seed(7)
    assert config_from_seed(7) != config_from_seed(8)
    assert build_seeded_serial_elbow_arm(7).name == "seeded_serial_elbow_arm_7"


def test_invalid_config_rejections() -> None:
    with pytest.raises(ValueError):
        resolve_config(SerialElbowArmConfig(axis_family="mixed_pitch_yaw"))  # type: ignore[arg-type]


def test_default_identity_and_validator() -> None:
    config = SerialElbowArmConfig()
    model = build_serial_elbow_arm(config)
    report = run_serial_elbow_arm_tests(model, config)
    assert report.passed, report.failures
    assert model.get_articulation("shoulder_pitch").axis == (0.0, 1.0, 0.0)
    assert model.get_articulation("elbow_pitch").axis == (0.0, 1.0, 0.0)


def test_source_adaptation_map_covers_core_semantics() -> None:
    assert SOURCE_IDS
    assert SOURCE_ADAPTATION_MAP
    assert len(SOURCE_IDS) >= 12


def test_representative_seeded_models_pass_basic_qc() -> None:
    for seed in range(3):
        _assert_basic_qc_passes(build_seeded_serial_elbow_arm(seed))
