"""Harness-owned motion QC gate (sampled-pose overlap) fault-injection tests.

The model under test is a door on a round hinge post: a knuckle sleeve on the
door captures the post (declared via an element-scoped allow_overlap), and the
panel stays clear of the post while sweeping over the slab. With a wall in the
swing arc the model is clean in the rest pose but penetrates the wall as the
joint opens; the legacy current-pose baseline must accept it, the motion QC
gate must reject it, and the allowance-covered captured-pin contact must stay
legal at every sampled pose.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from agent.compiler import compile_urdf_report


def _write_swinging_door_model_script(script_path: Path, *, with_wall: bool) -> None:
    wall_lines = (
        [
            "base.visual(",
            "    Box((0.4, 0.02, 0.2)),",
            "    origin=Origin(xyz=(0.1, 0.12, 0.1)),",
            "    name='wall',",
            ")",
        ]
        if with_wall
        else []
    )
    script_path.write_text(
        "\n".join(
            [
                "from __future__ import annotations",
                "",
                "from sdk import (",
                "    ArticulatedObject,",
                "    ArticulationType,",
                "    Box,",
                "    Cylinder,",
                "    MotionLimits,",
                "    Origin,",
                "    TestContext,",
                ")",
                "",
                "object_model = ArticulatedObject(name='swinging_door')",
                "base = object_model.part('base')",
                "base.visual(",
                "    Box((0.5, 0.5, 0.02)),",
                "    origin=Origin(xyz=(0.1, 0.05, -0.01)),",
                "    name='slab',",
                ")",
                "base.visual(",
                "    Cylinder(0.012, 0.2),",
                "    origin=Origin(xyz=(0.0, 0.0, 0.1)),",
                "    name='hinge_post',",
                ")",
                *wall_lines,
                "door = object_model.part('door')",
                "# Knuckle sleeves the post (captured pin, allowance below); the panel",
                "# clears the post by 2mm and its bottom rides flush on the slab plane.",
                "door.visual(",
                "    Cylinder(0.016, 0.04),",
                "    origin=Origin(xyz=(0.0, 0.0, 0.0)),",
                "    name='knuckle',",
                ")",
                "door.visual(",
                "    Box((0.18, 0.02, 0.16)),",
                "    origin=Origin(xyz=(0.104, 0.0, -0.02)),",
                "    name='panel',",
                ")",
                "object_model.articulation(",
                "    'base_to_door',",
                "    ArticulationType.REVOLUTE,",
                "    parent=base,",
                "    child=door,",
                "    axis=(0.0, 0.0, 1.0),",
                "    origin=Origin(xyz=(0.0, 0.0, 0.1)),",
                "    motion_limits=MotionLimits(",
                "        effort=1.0, velocity=1.0, lower=0.0, upper=1.5708",
                "    ),",
                ")",
                "",
                "def run_tests():",
                "    ctx = TestContext(object_model)",
                "    ctx.allow_overlap(",
                "        object_model.get_part('base'),",
                "        object_model.get_part('door'),",
                "        elem_a='hinge_post',",
                "        elem_b='knuckle',",
                "        reason='door knuckle sleeves the hinge post (captured pin)',",
                "    )",
                "    return ctx.report()",
            ]
        ),
        encoding="utf-8",
    )


def test_swept_door_passes_current_pose_baseline_without_motion_qc(tmp_path: Path) -> None:
    script_path = tmp_path / "model.py"
    _write_swinging_door_model_script(script_path, with_wall=True)

    report = compile_urdf_report(script_path, run_checks=True, target="full")

    assert "<robot" in report.urdf_xml


def test_swept_door_fails_motion_qc_gate(tmp_path: Path) -> None:
    script_path = tmp_path / "model.py"
    _write_swinging_door_model_script(script_path, with_wall=True)

    with pytest.raises(RuntimeError, match="harness_motion_qc"):
        compile_urdf_report(script_path, run_checks=True, target="full", motion_qc=True)


def test_clean_door_with_allowed_captured_pin_passes_motion_qc_gate(tmp_path: Path) -> None:
    script_path = tmp_path / "model.py"
    _write_swinging_door_model_script(script_path, with_wall=False)

    report = compile_urdf_report(script_path, run_checks=True, target="full", motion_qc=True)

    assert "<robot" in report.urdf_xml
