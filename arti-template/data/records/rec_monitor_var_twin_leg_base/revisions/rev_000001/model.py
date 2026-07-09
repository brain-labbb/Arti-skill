from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


# ── Stand dimensions ────────────────────────────────────────────────
FOOT_PAD_H = 0.005
JUNCTION_Z = 0.240
LEG_FOOT_X = 0.170
LEG_TOP_X = 0.018
LEG_WIDTH_Y = 0.048
LEG_THICKNESS = 0.016
COLUMN_W = 0.056
COLUMN_D = 0.042
COLUMN_TOP_Z = 0.362
CROSSHEAD_Z = 0.376

# ── Articulation anchors ────────────────────────────────────────────
HINGE_Z = 0.435
HINGE_Y = 0.026

# ── Screen dimensions ───────────────────────────────────────────────
SCREEN_OUTER_W = 0.585
SCREEN_OUTER_H = 0.375
PANEL_W = 0.512
PANEL_H = 0.320
SCREEN_CENTER_Z = 0.105

# ── Derived leg geometry ────────────────────────────────────────────
_LEG_DZ = JUNCTION_Z - FOOT_PAD_H
_LEG_DX = LEG_FOOT_X - LEG_TOP_X  # 0.152
_LEG_LENGTH = math.sqrt(_LEG_DX**2 + _LEG_DZ**2)
_LEG_ANGLE_DEG = math.degrees(math.atan2(_LEG_DX, _LEG_DZ))
_LEG_MID_Z = (FOOT_PAD_H + JUNCTION_Z) / 2.0

_COLUMN_H = COLUMN_TOP_Z - JUNCTION_Z
_COLUMN_CZ = JUNCTION_Z + _COLUMN_H / 2.0


# ── Geometry helpers ────────────────────────────────────────────────

def _leg_beam(sign: int) -> cq.Workplane:
    """One splayed A-frame leg beam, positioned in world space.

    *sign* = -1 for the left leg, +1 for the right leg.
    The beam runs from a wide foot on the desk to a narrow top at the
    central junction, rotated around Y so positive *q* tips the top
    end toward the centre line.
    """
    foot_x = sign * LEG_FOOT_X
    top_x = sign * LEG_TOP_X
    mid_x = (foot_x + top_x) / 2.0

    beam = cq.Workplane("XY").box(LEG_THICKNESS, LEG_WIDTH_Y, _LEG_LENGTH)
    # Tilt so the top end converges toward the centre column.
    beam = beam.rotate((0, 0, 0), (0, 1, 0), -sign * _LEG_ANGLE_DEG)
    beam = beam.translate((mid_x, 0.030, _LEG_MID_Z))
    return beam


def _foot_pad_xyz(sign: int) -> tuple[float, float, float]:
    """Centre of the rubber foot pad under each leg."""
    return (sign * LEG_FOOT_X, 0.030, FOOT_PAD_H / 2.0)


# ── Object model ────────────────────────────────────────────────────

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widescreen_lcd_monitor")

    # Materials
    matte_black = model.material("matte_black", rgba=(0.006, 0.007, 0.007, 1.0))
    soft_black = model.material("soft_black", rgba=(0.018, 0.020, 0.020, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.060, 0.062, 0.060, 1.0))
    lcd_white = model.material("lcd_white", rgba=(0.88, 0.92, 0.95, 1.0))
    dark_silver = model.material("dark_silver", rgba=(0.38, 0.38, 0.35, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.20, 0.20, 0.19, 1.0))
    rubber = model.material("rubber", rgba=(0.03, 0.03, 0.03, 1.0))

    # ── Stand ────────────────────────────────────────────────────────
    stand = model.part("stand")

    # Twin A-frame legs (shared geometry helper, for-loop naming)
    for i, sign in enumerate((-1, 1)):
        stand.visual(
            mesh_from_cadquery(_leg_beam(sign), f"leg_{i}"),
            material=gunmetal,
            name=f"leg_{i}",
        )
        fx, fy, fz = _foot_pad_xyz(sign)
        stand.visual(
            Box((0.046, 0.052, FOOT_PAD_H)),
            origin=Origin(xyz=(fx, fy, fz)),
            material=rubber,
            name=f"foot_pad_{i}",
        )

    # Central upright column (legs → crosshead)
    stand.visual(
        Box((COLUMN_W, COLUMN_D, _COLUMN_H)),
        origin=Origin(xyz=(0.0, 0.030, _COLUMN_CZ)),
        material=matte_black,
        name="central_column",
    )

    # Junction gusset where the two legs converge into the column
    stand.visual(
        Box((0.066, 0.054, 0.022)),
        origin=Origin(xyz=(0.0, 0.030, JUNCTION_Z + 0.011)),
        material=matte_black,
        name="junction_gusset",
    )

    # Small collar at column top (visual transition to crosshead)
    stand.visual(
        Cylinder(radius=0.034, length=0.006),
        origin=Origin(xyz=(0.0, 0.030, COLUMN_TOP_Z + 0.003)),
        material=dark_silver,
        name="column_collar",
    )

    # Top crosshead (KEEP – identical to parent)
    stand.visual(
        Box((0.126, 0.052, 0.030)),
        origin=Origin(xyz=(0.0, 0.055, CROSSHEAD_Z)),
        material=matte_black,
        name="top_crosshead",
    )

    # Yoke arms and hinge cheeks (KEEP – identical to parent)
    for i, x in enumerate((-0.053, 0.053)):
        stand.visual(
            Box((0.026, 0.034, 0.070)),
            origin=Origin(xyz=(x, HINGE_Y, HINGE_Z - 0.020)),
            material=matte_black,
            name=f"yoke_arm_{i}",
        )
        stand.visual(
            Cylinder(radius=0.018, length=0.026),
            origin=Origin(xyz=(x, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=matte_black,
            name=f"hinge_cheek_{i}",
        )

    # ── Screen ───────────────────────────────────────────────────────
    screen = model.part("screen")
    screen.visual(
        Box((0.568, 0.034, 0.360)),
        origin=Origin(xyz=(0.0, -0.047, SCREEN_CENTER_Z)),
        material=soft_black,
        name="rear_shell",
    )
    screen.visual(
        mesh_from_geometry(
            BezelGeometry(
                (PANEL_W, PANEL_H),
                (SCREEN_OUTER_W, SCREEN_OUTER_H),
                0.020,
                opening_shape="rounded_rect",
                outer_shape="rounded_rect",
                opening_corner_radius=0.004,
                outer_corner_radius=0.018,
            ),
            "screen_bezel",
        ),
        origin=Origin(xyz=(0.0, -0.067, SCREEN_CENTER_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="bezel",
    )
    screen.visual(
        Box((PANEL_W, 0.003, PANEL_H)),
        origin=Origin(xyz=(0.0, -0.076, SCREEN_CENTER_Z)),
        material=lcd_white,
        name="display_panel",
    )
    screen.visual(
        Box((0.074, 0.042, 0.100)),
        origin=Origin(xyz=(0.0, -0.030, 0.0)),
        material=dark_graphite,
        name="rear_mount",
    )
    screen.visual(
        Cylinder(radius=0.014, length=0.086),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="hinge_barrel",
    )

    # ── Articulation (KEEP – unchanged type / axis / placement) ──────
    model.articulation(
        "stand_to_screen",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=screen,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z), rpy=(-0.12, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=9.0, velocity=1.5, lower=-0.22, upper=0.20),
    )

    return model


# ── Tests ────────────────────────────────────────────────────────────

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    screen = object_model.get_part("screen")
    tilt = object_model.get_articulation("stand_to_screen")

    # ── A-frame base topology (TARGET-specific) ─────────────────────
    # Two discrete legs must be widely splayed at desk level, proving
    # the base is not a single plate.
    ctx.expect_gap(
        stand,
        stand,
        axis="x",
        min_gap=0.24,
        positive_elem="foot_pad_1",
        negative_elem="foot_pad_0",
        name="twin A-frame legs splay wide at desk level",
    )

    # Each leg must connect to the central column (XY overlap).
    for i in range(2):
        ctx.expect_overlap(
            stand,
            stand,
            axes="xy",
            min_overlap=0.008,
            elem_a=f"leg_{i}",
            elem_b="central_column",
            name=f"leg_{i} connects upward to the central column",
        )

    # ── Screen / stand relationship ──────────────────────────────────
    ctx.expect_gap(
        screen,
        stand,
        axis="z",
        min_gap=0.10,
        positive_elem="display_panel",
        negative_elem="leg_0",
        name="display sits well above the desktop legs",
    )
    ctx.expect_overlap(
        screen,
        stand,
        axes="x",
        min_overlap=0.04,
        elem_a="hinge_barrel",
        elem_b="top_crosshead",
        name="screen hinge is centered over the stand head",
    )

    # ── Panel aspect ratio ───────────────────────────────────────────
    ctx.check(
        "panel is 16:10 widescreen",
        abs((PANEL_W / PANEL_H) - 1.6) < 0.02,
        details=f"panel ratio={PANEL_W / PANEL_H:.3f}",
    )

    # ── Tilt joint behaviour ────────────────────────────────────────
    with ctx.pose({tilt: -0.20}):
        back_aabb = ctx.part_element_world_aabb(screen, elem="display_panel")
    with ctx.pose({tilt: 0.18}):
        forward_aabb = ctx.part_element_world_aabb(screen, elem="display_panel")

    def _center_y(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return (lo[1] + hi[1]) / 2.0

    back_y = _center_y(back_aabb)
    forward_y = _center_y(forward_aabb)
    ctx.check(
        "tilt joint moves screen forward and back",
        back_y is not None and forward_y is not None and forward_y < back_y - 0.025,
        details=f"back_y={back_y}, forward_y={forward_y}",
    )

    return ctx.report()


object_model = build_object_model()
