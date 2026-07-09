from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BASE_W = 0.380
BASE_D = 0.255
BASE_T = 0.032
REAR_Y = BASE_D / 2.0 - 0.008
HINGE_Z = BASE_T + 0.006

LID_W = 0.372
LID_H = 0.235
LID_T = 0.014
DISPLAY_W = 0.344
DISPLAY_H = 0.194
OPEN_ANGLE = math.radians(105.0)

KEY_T = 0.0030
KEY_BOTTOM_Z = BASE_T + 0.00065
KEY_TRAVEL = 0.0015


def _open_lid_origin(x: float, y: float, z: float) -> Origin:
    """Place a visual authored in the closed lid frame into the open rest pose."""
    c = math.cos(-OPEN_ANGLE)
    s = math.sin(-OPEN_ANGLE)
    return Origin(xyz=(x, y * c - z * s, y * s + z * c), rpy=(-OPEN_ANGLE, 0.0, 0.0))


def _key_width(units: float, pitch: float) -> float:
    return max(0.006, units * pitch - 0.0022)


_BUMPER_W = 0.024
_BUMPER_D = 0.024
_BUMPER_EXTRA_H = 0.004


def _corner_bumper() -> Box:
    """Shared geometry for rugged corner bumper guards."""
    return Box((_BUMPER_W, _BUMPER_D, BASE_T + _BUMPER_EXTRA_H))


def _add_key(
    model: ArticulatedObject,
    base,
    index: tuple[int, int],
    x: float,
    y: float,
    width: float,
    depth: float,
    *,
    legend_width: float | None = None,
) -> None:
    key = model.part(f"key_{index[0]}_{index[1]}")
    key.visual(
        Box((width, depth, KEY_T)),
        origin=Origin(xyz=(0.0, 0.0, KEY_T / 2.0)),
        material="key_black",
        name="keycap",
    )
    key.visual(
        Box((legend_width or min(width * 0.44, 0.0075), 0.0011, 0.00016)),
        origin=Origin(xyz=(0.0, 0.0, KEY_T + 0.00008)),
        material="legend_gray",
        name="legend",
    )
    model.articulation(
        f"base_to_key_{index[0]}_{index[1]}",
        ArticulationType.PRISMATIC,
        parent=base,
        child=key,
        origin=Origin(xyz=(x, y, KEY_BOTTOM_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.7, velocity=0.18, lower=0.0, upper=KEY_TRAVEL),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_workstation_15_6_laptop")

    model.material("matte_black_plastic", rgba=(0.015, 0.016, 0.015, 1.0))
    model.material("soft_black_plastic", rgba=(0.035, 0.037, 0.036, 1.0))
    model.material("key_black", rgba=(0.005, 0.005, 0.005, 1.0))
    model.material("off_screen_black", rgba=(0.0, 0.0, 0.0, 1.0))
    model.material("trackpad_black", rgba=(0.025, 0.026, 0.025, 1.0))
    model.material("legend_gray", rgba=(0.78, 0.78, 0.74, 1.0))
    model.material("webcam_glass", rgba=(0.004, 0.004, 0.006, 1.0))
    model.material("webcam_ring", rgba=(0.55, 0.55, 0.52, 1.0))
    model.material("rugged_bumper", rgba=(0.11, 0.13, 0.07, 1.0))

    base = model.part("base_chassis")
    base.visual(
        Box((BASE_W, BASE_D, BASE_T)),
        origin=Origin(xyz=(0.0, 0.0, BASE_T / 2.0)),
        material="matte_black_plastic",
        name="lower_shell",
    )
    base.visual(
        Box((BASE_W - 0.028, BASE_D - 0.026, 0.0010)),
        origin=Origin(xyz=(0.0, -0.004, BASE_T + 0.00050)),
        material="soft_black_plastic",
        name="palm_deck_skin",
    )
    base.visual(
        Cylinder(radius=0.0046, length=BASE_W - 0.052),
        origin=Origin(xyz=(0.0, REAR_Y + 0.0010, BASE_T + 0.0010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="matte_black_plastic",
        name="hinge_barrel",
    )
    # Small rear feet and front lip make the thin shell read as a real budget chassis.
    for i, x in enumerate((-0.150, 0.150)):
        base.visual(
            Box((0.052, 0.010, 0.0030)),
            origin=Origin(xyz=(x, -BASE_D / 2.0 + 0.010, -0.0015)),
            material="soft_black_plastic",
            name=f"front_foot_{i}",
        )
    for i, x in enumerate((-0.155, 0.155)):
        base.visual(
            Box((0.060, 0.007, 0.00055)),
            origin=Origin(xyz=(x, 0.057, BASE_T + 0.00105)),
            material="key_black",
            name=f"speaker_grille_{i}",
        )
    # Reinforced corner bumpers for rugged workstation form.
    _bumper_cx = BASE_W / 2.0 - _BUMPER_W / 2.0 + 0.002
    _bumper_cy = BASE_D / 2.0 - _BUMPER_D / 2.0 + 0.002
    _bumper_cz = (BASE_T + _BUMPER_EXTRA_H) / 2.0
    _corner_signs = [(1.0, -1.0), (-1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
    for i, (sx, sy) in enumerate(_corner_signs):
        base.visual(
            _corner_bumper(),
            origin=Origin(xyz=(sx * _bumper_cx, sy * _bumper_cy, _bumper_cz)),
            material="rugged_bumper",
            name=f"corner_bumper_{i}",
        )

    screen = model.part("screen_lid")
    screen.visual(
        Box((LID_W, LID_H, LID_T)),
        origin=_open_lid_origin(0.0, -LID_H / 2.0, LID_T / 2.0),
        material="soft_black_plastic",
        name="lid_shell",
    )
    screen.visual(
        Box((DISPLAY_W, DISPLAY_H, 0.0008)),
        origin=_open_lid_origin(0.0, -LID_H + 0.018 + DISPLAY_H / 2.0, -0.0004),
        material="off_screen_black",
        name="display_panel",
    )
    screen.visual(
        Cylinder(radius=0.0042, length=0.0007),
        origin=_open_lid_origin(0.0, -LID_H + 0.0095, -0.00035),
        material="webcam_ring",
        name="webcam_ring",
    )
    screen.visual(
        Cylinder(radius=0.0024, length=0.00095),
        origin=_open_lid_origin(0.0, -LID_H + 0.0095, -0.00085),
        material="webcam_glass",
        name="webcam_lens",
    )
    screen.visual(
        Box((0.074, 0.014, 0.0045)),
        origin=_open_lid_origin(0.0, -0.0040, LID_T + 0.0018),
        material="matte_black_plastic",
        name="bottom_hinge_cover",
    )

    model.articulation(
        "base_to_screen",
        ArticulationType.REVOLUTE,
        parent=base,
        child=screen,
        origin=Origin(xyz=(0.0, REAR_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=1.2, lower=-OPEN_ANGLE, upper=0.25),
    )

    # Island keyboard: full width main block plus distinct numeric keypad.
    row_specs = [
        # units across the row; the helper turns units into isolated key widths.
        [1] * 16,
        [1] * 14 + [1, 1, 1, 1],
        [1.25] + [1] * 12 + [1, 1, 1, 1],
        [1.45] + [1] * 10 + [1.55] + [1, 1, 1, 1],
        [1.7] + [1] * 9 + [1.8] + [1, 1, 1, 1],
        [1.3, 1.05, 1.05, 1.15, 5.2, 1.15, 1.05, 1.05, 1.0, 1.0, 1.0, 1, 1, 1, 1],
    ]
    row_ys = [0.073, 0.052, 0.031, 0.010, -0.011, -0.034]
    key_depths = [0.0110, 0.0145, 0.0145, 0.0145, 0.0145, 0.0145]
    pitches = [0.0190, 0.0175, 0.0175, 0.0175, 0.0175, 0.0175]

    for r, units in enumerate(row_specs):
        pitch = pitches[r]
        total = sum(units) * pitch
        x = -total / 2.0
        for c, u in enumerate(units):
            center_x = x + u * pitch / 2.0
            width = _key_width(u, pitch)
            if r == 5 and c == 4:
                legend_width = min(width * 0.55, 0.046)
            else:
                legend_width = None
            _add_key(
                model,
                base,
                (r, c),
                center_x,
                row_ys[r],
                width,
                key_depths[r],
                legend_width=legend_width,
            )
            x += u * pitch

    trackpad = model.part("trackpad")
    trackpad.visual(
        Box((0.106, 0.062, 0.0013)),
        origin=Origin(xyz=(0.0, 0.0, 0.00065)),
        material="trackpad_black",
        name="pad_plate",
    )
    trackpad.visual(
        Box((0.103, 0.0010, 0.00018)),
        origin=Origin(xyz=(0.0, -0.0205, 0.00139)),
        material="key_black",
        name="click_seam",
    )
    model.articulation(
        "base_to_trackpad",
        ArticulationType.PRISMATIC,
        parent=base,
        child=trackpad,
        origin=Origin(xyz=(0.0, -0.086, BASE_T + 0.00022)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=0.08, lower=0.0, upper=0.0009),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_chassis")
    screen = object_model.get_part("screen_lid")
    trackpad = object_model.get_part("trackpad")
    hinge = object_model.get_articulation("base_to_screen")

    ctx.allow_overlap(
        base,
        screen,
        elem_a="hinge_barrel",
        elem_b="lid_shell",
        reason="The base hinge barrel is intentionally captured by the lower edge of the lid shell.",
    )

    key_count = len([p for p in object_model.parts if p.name.startswith("key_")])
    ctx.check("full keyboard has numeric keypad", key_count >= 85, details=f"key_count={key_count}")

    base_aabb = ctx.part_world_aabb(base)
    display_aabb = ctx.part_element_world_aabb(screen, elem="display_panel")
    ctx.check(
        "rugged thick base envelope",
        base_aabb is not None
        and 0.36 <= (base_aabb[1][0] - base_aabb[0][0]) <= 0.42
        and 0.24 <= (base_aabb[1][1] - base_aabb[0][1]) <= 0.29
        and 0.030 <= (base_aabb[1][2] - base_aabb[0][2]) <= 0.050,
        details=f"base_aabb={base_aabb}",
    )
    bumper_names = [f"corner_bumper_{i}" for i in range(4)]
    bumper_count = sum(
        1 for v in base.visuals if v.name in bumper_names
    )
    ctx.check(
        "four reinforced corner bumpers on base",
        bumper_count == 4,
        details=f"found={bumper_count}",
    )
    ctx.check(
        "display is upright at viewing angle",
        display_aabb is not None
        and display_aabb[1][2] > 0.20
        and display_aabb[0][1] > 0.09,
        details=f"display_aabb={display_aabb}",
    )

    ctx.expect_within(
        trackpad,
        base,
        axes="xy",
        margin=0.004,
        name="trackpad stays inside palm rest",
    )
    ctx.expect_overlap(
        screen,
        base,
        axes="x",
        min_overlap=0.34,
        elem_a="display_panel",
        elem_b="lower_shell",
        name="screen spans laptop width",
    )
    ctx.expect_overlap(
        base,
        screen,
        axes="x",
        min_overlap=0.30,
        elem_a="hinge_barrel",
        elem_b="lid_shell",
        name="hinge barrel spans captured lid edge",
    )

    open_aabb = ctx.part_element_world_aabb(screen, elem="display_panel")
    with ctx.pose({hinge: -OPEN_ANGLE}):
        closed_aabb = ctx.part_element_world_aabb(screen, elem="display_panel")
    ctx.check(
        "hinge closes from open rest pose",
        open_aabb is not None
        and closed_aabb is not None
        and closed_aabb[1][2] < open_aabb[1][2] - 0.14
        and closed_aabb[0][1] < 0.0,
        details=f"open={open_aabb}, closed={closed_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
