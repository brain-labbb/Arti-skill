from __future__ import annotations

"""Roller shade window covering.

Forked from the horizontal wooden venetian blind parent. The shade panel
mechanism is changed to a roller shade: a top roller tube with a single
flexible sheet winding down and a bottom bar. The headrail location and
control cord area are preserved from the parent.

Articulations:
- roller (REVOLUTE): the roller tube rotates about its long horizontal (X) axis
  to wind the shade material around itself. Limits 0..2π rad.
- shade_lift (PRISMATIC): the bottom bar and attached shade sheet rise when the
  cord is pulled. Independent driver along +Z, travel 0..0.55 m.

Both joints represent the same user action (pulling the cord to raise the
shade); cross-type mimic is not supported so they are modeled as independent
DOFs that a downstream controller would drive in coordination.

Kinematic structure: the roller tube is a REVOLUTE child of the headrail,
mounted just below the headrail box. The bottom bar (with the shade sheet
attached as a visual) is a PRISMATIC child of the headrail.

Support strategy: the headrail carries both the roller tube (through the
revolute joint at the tube center, with bearings contacting the headrail
bottom face) and the bottom bar assembly (through the prismatic lift joint,
with the shade panel contacting the headrail bottom when deployed).
"""

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------- dimensions
BLIND_WIDTH = 0.80
HEADRAIL_SIZE = (BLIND_WIDTH, 0.062, 0.060)
HEADRAIL_Z = 1.27  # center; spans 1.24..1.30

# Roller tube sits just below the headrail
ROLLER_RADIUS = 0.022
ROLLER_WALL = 0.004  # tube wall thickness
ROLLER_LENGTH = 0.76
ROLLER_Z = 1.214  # tube center Z (bearings contact headrail bottom at 1.240)

# Shade sheet dimensions (deployed)
SHADE_WIDTH = 0.74
SHADE_THICK = 0.003
SHADE_BOTTOM_Z = 0.080  # bottom bar center when fully lowered
# Shade extends from bottom bar center up to the headrail bottom face
HEADRAIL_BOTTOM_Z = HEADRAIL_Z - HEADRAIL_SIZE[2] / 2.0  # 1.240
SHADE_HEIGHT = HEADRAIL_BOTTOM_Z - SHADE_BOTTOM_Z  # 1.160 m deployed shade

# Bottom bar
BOTTOM_BAR_WIDTH = 0.76
BOTTOM_BAR_DEPTH = 0.032
BOTTOM_BAR_HEIGHT = 0.020
BOTTOM_BAR_Z = SHADE_BOTTOM_Z

# Wound shade ridge: a subtle thickened section on the rear of the tube where
# the shade fabric wraps before departing downward.
WRAP_THICK = 0.006  # radial thickness of wound layer

# Lift: the bottom bar rises to show the shade partially rolled up
SHADE_TRAVEL = 0.55  # bar rises 0.55 m

# Roller rotation: one full turn
ROLLER_UPPER = 2.0 * math.pi

# --------------- materials
HEADRAIL_WOOD = Material("headrail_wood", rgba=(0.38, 0.24, 0.15, 1.0))
ROLLER_METAL = Material("roller_metal", rgba=(0.62, 0.62, 0.65, 1.0))
SHADE_FABRIC = Material("shade_fabric", rgba=(0.93, 0.89, 0.82, 1.0))
SHADE_WRAP = Material("shade_wrap", rgba=(0.88, 0.84, 0.77, 1.0))
BOTTOM_BAR_WOOD = Material("bottom_bar_wood", rgba=(0.32, 0.20, 0.13, 1.0))
CORD_TAN = Material("cord_tan", rgba=(0.66, 0.55, 0.40, 1.0))
BRACKET_METAL = Material("bracket_metal", rgba=(0.50, 0.50, 0.53, 1.0))


def build_roller_tube() -> object:
    """Hollow cylindrical roller tube centered at origin along Z (to be
    rotated to X axis by the visual origin)."""
    outer = ROLLER_RADIUS
    inner = ROLLER_RADIUS - ROLLER_WALL
    half_len = ROLLER_LENGTH / 2.0
    tube = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, -half_len))
        .circle(outer)
        .circle(inner)
        .extrude(ROLLER_LENGTH)
    )
    return tube


def build_shade_panel() -> object:
    """Thin flat shade sheet centered at origin. Width along X, thin along Y,
    tall along Z. Height spans from the bottom bar center up to the headrail."""
    return (
        cq.Workplane("XY")
        .box(SHADE_WIDTH, SHADE_THICK, SHADE_HEIGHT)
    )


def build_wound_shade_ridge() -> object:
    """Small ridge on the front of the roller tube representing the wound
    shade fabric departure point. Thin strip along X, positioned at the
    front-bottom of the tube in the roller_tube part frame."""
    ridge_width = SHADE_WIDTH  # spans shade width
    ridge_depth = WRAP_THICK  # radial thickness
    ridge_height = 0.008  # small height along tube surface
    return (
        cq.Workplane("XY")
        .box(ridge_width, ridge_depth, ridge_height)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roller_shade")

    # ============================================================= headrail
    headrail = model.part("headrail")
    headrail.visual(
        Box(HEADRAIL_SIZE),
        origin=Origin(xyz=(0.0, 0.0, HEADRAIL_Z)),
        material=HEADRAIL_WOOD,
        name="headrail_box",
    )

    # Mounting brackets at each end, above the headrail
    for i, (side, sx) in enumerate((("left", -1), ("right", 1))):
        bx = sx * (BLIND_WIDTH / 2.0 - 0.020)
        headrail.visual(
            Box((0.040, 0.070, 0.035)),
            origin=Origin(xyz=(bx, 0.0, HEADRAIL_Z + 0.047)),
            material=BRACKET_METAL,
            name=f"bracket_{i}",
        )

    # Cord lock housing on the front-right of the headrail
    headrail.visual(
        Box((0.055, 0.016, 0.028)),
        origin=Origin(xyz=(0.32, 0.038, 1.254)),
        material=HEADRAIL_WOOD,
        name="cord_lock",
    )

    # Pull cord hanging from front of headrail
    cord_top = 1.258
    cord_bottom = 0.520
    headrail.visual(
        Cylinder(radius=0.0022, length=cord_top - cord_bottom),
        origin=Origin(xyz=(0.320, 0.040, 0.5 * (cord_top + cord_bottom))),
        material=CORD_TAN,
        name="pull_cord",
    )
    # Tassel at bottom of pull cord
    headrail.visual(
        Cylinder(radius=0.008, length=0.050),
        origin=Origin(xyz=(0.320, 0.040, cord_bottom - 0.025)),
        material=HEADRAIL_WOOD,
        name="cord_tassel",
    )

    # ========================================================= roller tube
    roller_tube = model.part("roller_tube")

    # Main tube shell (CadQuery hollow cylinder, rotated from Z to X)
    tube_mesh = mesh_from_cadquery(build_roller_tube(), "roller_tube_shell")
    roller_tube.visual(
        tube_mesh,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=ROLLER_METAL,
        name="tube_shell",
    )

    # Wound-shade ridge: small strip on the rear of the tube showing where
    # the shade fabric wraps. Positioned at the back of the tube in the part
    # frame (y = -ROLLER_RADIUS) to avoid overlapping the shade panel at Y=0.
    ridge_mesh = mesh_from_cadquery(build_wound_shade_ridge(), "wound_shade_ridge")
    roller_tube.visual(
        ridge_mesh,
        origin=Origin(xyz=(0.0, -(ROLLER_RADIUS + WRAP_THICK / 2.0 - 0.001), 0.0)),
        material=SHADE_WRAP,
        name="shade_ridge",
    )

    # End caps / bearings at each tube end
    for i, sx in enumerate((-1, 1)):
        cx = sx * (ROLLER_LENGTH / 2.0 + 0.003)
        roller_tube.visual(
            Cylinder(radius=ROLLER_RADIUS + 0.004, length=0.008),
            origin=Origin(
                xyz=(cx, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=BRACKET_METAL,
            name=f"tube_bearing_{i}",
        )

    # Roller articulation: REVOLUTE about long horizontal X axis.
    # Independent from shade_lift (cross-type mimic is not supported);
    # both joints represent the same user action of pulling the cord.
    model.articulation(
        "roller",
        ArticulationType.REVOLUTE,
        parent=headrail,
        child=roller_tube,
        origin=Origin(xyz=(0.0, 0.0, ROLLER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=0.0, upper=ROLLER_UPPER,
        ),
    )

    # ========================================================== bottom bar
    bottom_bar = model.part("bottom_bar")

    # Weighted bottom bar
    bottom_bar.visual(
        Box((BOTTOM_BAR_WIDTH, BOTTOM_BAR_DEPTH, BOTTOM_BAR_HEIGHT)),
        origin=Origin(),
        material=BOTTOM_BAR_WOOD,
        name="bottom_bar_weight",
    )

    # Shade sheet: thin panel extending upward from the bar
    # Center the shade panel so its bottom edge is at the bar center
    shade_mesh = mesh_from_cadquery(build_shade_panel(), "shade_panel")
    bottom_bar.visual(
        shade_mesh,
        origin=Origin(xyz=(0.0, 0.0, SHADE_HEIGHT / 2.0)),
        material=SHADE_FABRIC,
        name="shade_panel",
    )

    # Shade lift: PRISMATIC driver. The bottom bar rises when the cord is
    # pulled. Modeled independently from the roller (cross-type mimic is not
    # supported); both joints represent the same cord-pull action.
    model.articulation(
        "shade_lift",
        ArticulationType.PRISMATIC,
        parent=headrail,
        child=bottom_bar,
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_BAR_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0, velocity=0.5, lower=0.0, upper=SHADE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    headrail = object_model.get_part("headrail")
    roller_tube = object_model.get_part("roller_tube")
    bottom_bar = object_model.get_part("bottom_bar")
    roller = object_model.get_articulation("roller")
    shade_lift = object_model.get_articulation("shade_lift")

    # 1. Roller tube has a cylindrical shell visual (CadQuery mesh).
    tube_shell = roller_tube.get_visual("tube_shell")
    ctx.check(
        "roller_tube_has_shell",
        tube_shell is not None,
        "roller_tube missing tube_shell visual",
    )

    # 2. Shade panel exists on the bottom bar.
    shade_panel = bottom_bar.get_visual("shade_panel")
    ctx.check(
        "shade_panel_exists",
        shade_panel is not None,
        "bottom_bar missing shade_panel visual",
    )

    # 3. Bottom bar weight exists.
    bar_weight = bottom_bar.get_visual("bottom_bar_weight")
    ctx.check(
        "bottom_bar_weight_exists",
        bar_weight is not None,
        "bottom_bar missing bottom_bar_weight visual",
    )

    # 4. Roller articulation is REVOLUTE about the long horizontal X axis.
    ctx.check(
        "roller_axis_is_horizontal_x",
        tuple(roller.axis) == (1.0, 0.0, 0.0)
        and roller.articulation_type == ArticulationType.REVOLUTE,
        f"roller axis={roller.axis}, type={roller.articulation_type}",
    )
    ctx.check(
        "roller_limits_positive_full_turn",
        roller.motion_limits is not None
        and abs(roller.motion_limits.lower) < 1e-9
        and abs(roller.motion_limits.upper - 2.0 * math.pi) < 0.01,
        f"roller limits={roller.motion_limits}",
    )

    # 5. Shade lift is PRISMATIC along Z, independent driver.
    ctx.check(
        "shade_lift_is_prismatic_z",
        shade_lift.articulation_type == ArticulationType.PRISMATIC
        and tuple(shade_lift.axis) == (0.0, 0.0, 1.0),
        f"shade_lift axis={shade_lift.axis}, type={shade_lift.articulation_type}",
    )
    ctx.check(
        "shade_lift_is_independent_driver",
        shade_lift.mimic is None,
        f"shade_lift should be independent, mimic={shade_lift.mimic}",
    )

    # 6. Shade lift travel is substantial (>0.30 m).
    ctx.check(
        "shade_lift_travel_substantial",
        shade_lift.motion_limits is not None
        and shade_lift.motion_limits.upper > 0.30,
        f"shade_lift travel={shade_lift.motion_limits.upper if shade_lift.motion_limits else 0:.3f} m",
    )

    # 7. Headrail box exists and is at the expected height.
    headrail_box = headrail.get_visual("headrail_box")
    ctx.check(
        "headrail_box_exists",
        headrail_box is not None,
        "headrail missing headrail_box visual",
    )

    # 8. Lowered pose: roller tube sits just below headrail, shade extends down.
    with ctx.pose({roller: 0.0}):
        roller_aabb = ctx.part_world_aabb(roller_tube)
        headrail_aabb = ctx.part_world_aabb(headrail)
        assert roller_aabb is not None and headrail_aabb is not None
        ctx.check(
            "roller_tube_below_headrail_at_rest",
            roller_aabb[1][2] < headrail_aabb[1][2],
            f"roller top={roller_aabb[1][2]:.3f}, headrail top={headrail_aabb[1][2]:.3f}",
        )

        # Shade panel reaches the headrail at rest (deployed shade)
        shade_aabb = ctx.part_element_world_aabb(bottom_bar, elem=shade_panel)
        assert shade_aabb is not None
        ctx.check(
            "shade_reaches_headrail_at_rest",
            shade_aabb[1][2] > HEADRAIL_BOTTOM_Z - 0.01,
            f"shade top={shade_aabb[1][2]:.3f}, headrail bottom={HEADRAIL_BOTTOM_Z:.3f}",
        )

    # 9. Raised pose: driving shade_lift raises the bottom bar.
    lift_max = float(shade_lift.motion_limits.upper)
    bar_rest_z = ctx.part_world_position(bottom_bar)[2]
    with ctx.pose({shade_lift: lift_max}):
        bar_raised_z = ctx.part_world_position(bottom_bar)[2]
        ctx.check(
            "bottom_bar_rises_on_lift",
            bar_raised_z > bar_rest_z + 0.20,
            f"rest z={bar_rest_z:.3f}, raised z={bar_raised_z:.3f}",
        )

    # 10. Pull cord and tassel hang at the front, below the headrail.
    tassel = headrail.get_visual("cord_tassel")
    ctx.check(
        "cord_tassel_exists",
        tassel is not None,
        "headrail missing cord_tassel visual",
    )
    tassel_aabb = ctx.part_element_world_aabb(headrail, elem=tassel)
    headrail_box_aabb = ctx.part_element_world_aabb(headrail, elem=headrail_box)
    assert tassel_aabb is not None and headrail_box_aabb is not None
    ctx.check(
        "tassel_hangs_below_headrail_front",
        tassel_aabb[1][2] < headrail_box_aabb[0][2]
        and tassel_aabb[0][1] > 0.0,
        f"tassel aabb={tassel_aabb}",
    )

    # 11. Wound shade ridge on the roller tube.
    ridge = roller_tube.get_visual("shade_ridge")
    ctx.check(
        "wound_shade_ridge_exists",
        ridge is not None,
        "roller_tube missing shade_ridge visual",
    )

    # 12. Roller tube is supported by headrail (not floating).
    ctx.expect_contact(
        roller_tube,
        headrail,
        name="roller_tube_supported_by_headrail",
    )

    return ctx.report()


object_model = build_object_model()
