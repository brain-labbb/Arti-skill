from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Animal-shaped toddler seesaw (horse theme)
#
# World frame: beam runs along X, pivot axis along Y, Z up.
# - Two A-frame tubular legs (~32 mm dia) with rubber ground pads at feet;
#   the legs converge at a central pivot bracket holding the axle.
# - The rocking beam is a stylised horse body (~1.1 m long) moulded in bright
#   green plastic with an orange head/neck, two yellow ear handles, and a
#   small tail wedge.
# - Rubber bump stops are bolted below each beam tip to limit ground impact.
# - Single revolute joint at the apex, axis (0, 1, 0), +/-15 degrees.
#   Positive q lowers the +X (head) end.
# ---------------------------------------------------------------------------

# --- overall scale (toddler-sized) ---
PIVOT_Z = 0.42          # axle centre height above ground

# --- A-frame base ---
LEG_FOOT_X = 0.32
LEG_FOOT_Y = 0.28
LEG_TUBE_R = 0.016
AXLE_R = 0.012
AXLE_LEN = 0.14
# Legs stop below the sleeve so they don't collide with it
LEG_APEX_Z = PIVOT_Z - 0.045  # 0.375 - bracket fills the gap to axle

# --- ground pads ---
PAD_SIZE = (0.10, 0.10, 0.012)

# --- horse body (beam-local frame: origin at axle centre) ---
BODY_W = 0.20
BODY_H = 0.16
HORSE_BODY_LEN = 1.00
SLEEVE_R = 0.020
SLEEVE_LEN = 0.060

# Body center Z in beam-local frame: torso bottom overlaps sleeve top for contact
BODY_Z_OFFSET = SLEEVE_R + BODY_H / 2.0 - 0.005  # 0.095

# Neck/head (built on torso top surface)
NECK_X = 0.36
NECK_W = 0.10
NECK_D = BODY_W * 0.65
NECK_H = 0.10
HEAD_LEN = 0.16
HEAD_W = 0.12
HEAD_H = 0.09
SNOUT_LEN = 0.07
SNOUT_W = HEAD_W * 1.05
SNOUT_H = 0.06

# Tail
TAIL_LEN = 0.10
TAIL_W = BODY_W * 0.45
TAIL_H = 0.07

# Ears
EAR_R = 0.012
EAR_H = 0.07
EAR_SPACING = 0.05

# Bump stops
BUMPER_X = 0.42
BUMPER_SIZE = (0.06, 0.08, 0.04)

TILT = math.radians(15.0)


def _build_horse_body_green() -> object:
    """CadQuery: horse torso + tail as one green solid.

    Built in beam-local frame (origin at axle centre).
    """
    bz = BODY_Z_OFFSET
    torso_bottom = bz - BODY_H / 2.0
    torso_top = bz + BODY_H / 2.0

    # Torso: elongated box with rounded vertical edges only
    torso = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, bz))
        .box(HORSE_BODY_LEN, BODY_W, BODY_H)
        .edges("|Z")
        .fillet(0.04)
    )

    # Tail: box extending from the -X torso face, clearly overlapping
    tail_cx = -HORSE_BODY_LEN / 2.0 - TAIL_LEN / 2.0 + 0.04  # 0.04 overlap
    tail_cz = bz + BODY_H / 2.0 - TAIL_H / 2.0 - 0.01  # near torso top
    tail = (
        cq.Workplane("XY")
        .transformed(offset=(tail_cx, 0.0, tail_cz))
        .box(TAIL_LEN + 0.04, TAIL_W, TAIL_H)  # extra 0.04 for overlap
        .edges("|Y")
        .fillet(0.012)
    )

    return torso.union(tail)


def _build_horse_head_orange() -> object:
    """CadQuery: neck + head + snout as one orange solid.

    The neck bottom sits on the torso top surface for clear contact.
    Built in beam-local frame.
    """
    bz = BODY_Z_OFFSET
    torso_top = bz + BODY_H / 2.0

    # Neck: box rising from torso top
    neck_cz = torso_top + NECK_H / 2.0
    neck = (
        cq.Workplane("XY")
        .transformed(offset=(NECK_X, 0.0, neck_cz))
        .box(NECK_W, NECK_D, NECK_H)
        .edges("|Y")
        .fillet(0.018)
    )

    # Head: box on top of neck
    neck_top = torso_top + NECK_H
    head_cz = neck_top + HEAD_H / 2.0
    head = (
        cq.Workplane("XY")
        .transformed(offset=(NECK_X + 0.04, 0.0, head_cz))
        .box(HEAD_LEN, HEAD_W, HEAD_H)
        .edges("|Z")
        .fillet(0.02)
        .edges("|X")
        .fillet(0.015)
    )

    # Snout: box at front of head
    snout_cx = NECK_X + 0.04 + HEAD_LEN / 2.0 + SNOUT_LEN / 2.0 - 0.02
    snout_cz = head_cz - HEAD_H / 2.0 + SNOUT_H / 2.0
    snout = (
        cq.Workplane("XY")
        .transformed(offset=(snout_cx, 0.0, snout_cz))
        .box(SNOUT_LEN + 0.02, SNOUT_W, SNOUT_H)
        .edges("|Z")
        .fillet(0.015)
    )

    return neck.union(head).union(snout)


def _leg_points(foot_x: float, foot_y: float) -> list[tuple[float, float, float]]:
    """Straight-line centerline for one A-frame leg tube."""
    foot_z = PAD_SIZE[2] / 2.0  # start at pad center
    apex_y = foot_y * 0.12  # converge inward
    return [(foot_x, foot_y, foot_z), (0.0, apex_y, LEG_APEX_Z)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="animal_toddler_seesaw")

    # Materials
    bright_green = model.material("bright_green_plastic", rgba=(0.18, 0.65, 0.22, 1.0))
    orange = model.material("orange_plastic", rgba=(0.90, 0.45, 0.10, 1.0))
    steel_gray = model.material("powder_coat_steel", rgba=(0.38, 0.40, 0.42, 1.0))
    rubber = model.material("black_rubber", rgba=(0.06, 0.06, 0.06, 1.0))
    pad_rubber = model.material("gray_rubber_pad", rgba=(0.25, 0.25, 0.25, 1.0))
    ear_color = model.material("yellow_plastic", rgba=(0.92, 0.82, 0.12, 1.0))
    dark_accent = model.material("dark_brown_accent", rgba=(0.30, 0.18, 0.08, 1.0))

    # ===================================================================
    # BASE: A-frame legs + ground pads + pivot bracket + axle
    # ===================================================================
    base = model.part("support_base")

    for i, side in enumerate((1.0, -1.0)):
        # Front leg (+X foot)
        base.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _leg_points(LEG_FOOT_X, side * LEG_FOOT_Y),
                    radius=LEG_TUBE_R,
                    samples_per_segment=4,
                    radial_segments=14,
                    cap_ends=True,
                ),
                f"leg_front_{i}",
            ),
            material=steel_gray,
            name=f"leg_{i}",
        )
        # Rear leg (-X foot)
        base.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _leg_points(-LEG_FOOT_X, side * LEG_FOOT_Y),
                    radius=LEG_TUBE_R,
                    samples_per_segment=4,
                    radial_segments=14,
                    cap_ends=True,
                ),
                f"leg_rear_{i}",
            ),
            material=steel_gray,
            name=f"leg_{i}_rear",
        )

    # Pivot bracket: box from leg apex to axle level
    bracket_y_half = LEG_FOOT_Y * 0.12 + LEG_TUBE_R + 0.01
    bracket_cz = (LEG_APEX_Z + PIVOT_Z) / 2.0  # center between leg top and axle
    bracket_h = PIVOT_Z - LEG_APEX_Z + 0.01    # extends from leg top to just above axle
    bracket = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, bracket_cz))
        .box(0.05, bracket_y_half * 2.0, bracket_h)
        .edges("|Z")
        .fillet(0.006)
    )
    base.visual(
        mesh_from_cadquery(bracket, "pivot_bracket"),
        material=steel_gray,
        name="pivot_bracket",
    )

    # Pivot axle bolt
    base.visual(
        Cylinder(radius=AXLE_R, length=AXLE_LEN),
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_accent,
        name="pivot_axle",
    )

    # Rubber ground pads under each foot
    for i, (fx, fy) in enumerate([
        (LEG_FOOT_X, LEG_FOOT_Y),
        (LEG_FOOT_X, -LEG_FOOT_Y),
        (-LEG_FOOT_X, LEG_FOOT_Y),
        (-LEG_FOOT_X, -LEG_FOOT_Y),
    ]):
        base.visual(
            Box(PAD_SIZE),
            origin=Origin(xyz=(fx, fy, PAD_SIZE[2] / 2.0)),
            material=pad_rubber,
            name=f"ground_pad_{i}",
        )

    # ===================================================================
    # BEAM: horse body + ears + pivot sleeve + bump stops
    # ===================================================================
    beam = model.part("horse_beam")

    # Horse torso + tail (green)
    green_body = _build_horse_body_green()
    beam.visual(
        mesh_from_cadquery(green_body, "horse_body"),
        material=bright_green,
        name="horse_body",
    )

    # Horse head/neck/snout (orange) - sits on torso top surface
    orange_head = _build_horse_head_orange()
    beam.visual(
        mesh_from_cadquery(orange_head, "horse_head"),
        material=orange,
        name="horse_head",
    )

    # Ear handles sitting ON the head top surface (not embedded)
    bz = BODY_Z_OFFSET
    torso_top = bz + BODY_H / 2.0
    neck_top = torso_top + NECK_H
    head_top = neck_top + HEAD_H
    ear_x = NECK_X + 0.02
    for i, ey in enumerate((EAR_SPACING / 2.0, -EAR_SPACING / 2.0)):
        # Ear bottom at head top surface, extends upward
        beam.visual(
            Cylinder(radius=EAR_R, length=EAR_H),
            origin=Origin(
                xyz=(ear_x, ey, head_top + EAR_H / 2.0),
                rpy=(0.10 * (1 if i == 0 else -1), 0.0, 0.0),
            ),
            material=ear_color,
            name=f"ear_handle_{i}",
        )

    # Pivot sleeve (bushing around the axle)
    beam.visual(
        Cylinder(radius=SLEEVE_R, length=SLEEVE_LEN),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_accent,
        name="pivot_sleeve",
    )

    # Rubber bump stops below each beam end
    torso_bottom = BODY_Z_OFFSET - BODY_H / 2.0
    bump_cz = torso_bottom - BUMPER_SIZE[2] / 2.0 + 0.005  # 5mm overlap with torso
    for i, bx in enumerate((BUMPER_X, -BUMPER_X)):
        beam.visual(
            Box(BUMPER_SIZE),
            origin=Origin(xyz=(bx, 0.0, bump_cz)),
            material=rubber,
            name=f"bump_stop_{i}",
        )

    # ===================================================================
    # ARTICULATION: central revolute pivot
    # ===================================================================
    model.articulation(
        "beam_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0, velocity=2.0, lower=-TILT, upper=TILT
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("support_base")
    beam = object_model.get_part("horse_beam")
    pivot = object_model.get_articulation("beam_pivot")

    # --- Pivot sleeve captures axle bolt (intentional nesting) ---
    ctx.allow_overlap(
        beam,
        base,
        elem_a="pivot_sleeve",
        elem_b="pivot_axle",
        reason="Pivot sleeve is a bushing intentionally nested around the axle bolt.",
    )
    ctx.allow_overlap(
        beam,
        base,
        elem_a="pivot_sleeve",
        elem_b="pivot_bracket",
        reason="Pivot sleeve passes through the bracket cradle to rotate on the axle.",
    )
    ctx.expect_contact(
        beam,
        base,
        elem_a="pivot_sleeve",
        elem_b="pivot_axle",
        name="pivot sleeve seated on axle bolt",
    )
    ctx.expect_within(
        beam,
        base,
        axes="y",
        inner_elem="pivot_sleeve",
        outer_elem="pivot_axle",
        margin=0.001,
        name="pivot sleeve stays inside axle span",
    )

    # --- Joint is non-fixed revolute ---
    ax = pivot.axis
    ctx.check(
        "pivot axis is horizontal and perpendicular to beam",
        abs(ax[0]) < 1e-9 and abs(ax[1] - 1.0) < 1e-9 and abs(ax[2]) < 1e-9,
        details=f"axis={ax}",
    )
    lim = pivot.motion_limits
    ctx.check(
        "rocking limits are +/- 15 degrees",
        lim is not None
        and lim.lower is not None
        and lim.upper is not None
        and abs(lim.lower + TILT) < 1e-6
        and abs(lim.upper - TILT) < 1e-6,
        details=f"limits=({lim.lower}, {lim.upper})",
    )

    # --- Animal-shaped body exists ---
    body_box = ctx.part_element_world_aabb(beam, elem="horse_body")
    ctx.check(
        "horse body exists and has toddler-appropriate length",
        body_box is not None
        and 0.80 < (body_box[1][0] - body_box[0][0]) < 1.40,
        details=f"body aabb={body_box}",
    )

    # --- Head rises above torso ---
    head_box = ctx.part_element_world_aabb(beam, elem="horse_head")
    ctx.check(
        "horse head rises above the torso",
        head_box is not None
        and body_box is not None
        and head_box[1][2] > body_box[0][2] + 0.10,
        details=f"head aabb={head_box}",
    )

    # --- Head sits on torso (contact) ---
    ctx.expect_contact(
        beam,
        beam,
        elem_a="horse_head",
        elem_b="horse_body",
        name="horse head contacts the torso top",
    )

    # --- Ground pads exist under base feet ---
    for i in range(4):
        pad_box = ctx.part_element_world_aabb(base, elem=f"ground_pad_{i}")
        ctx.check(
            f"ground_pad_{i} exists at ground level",
            pad_box is not None and pad_box[0][2] < 0.015,
            details=f"pad aabb={pad_box}",
        )

    # --- Safety bump stops exist below horse body ---
    for i in range(2):
        bump_box = ctx.part_element_world_aabb(beam, elem=f"bump_stop_{i}")
        ctx.check(
            f"bump_stop_{i} hangs below the horse body",
            bump_box is not None
            and body_box is not None
            and bump_box[0][2] < body_box[0][2],
            details=f"bump_stop aabb={bump_box}",
        )

    # --- Base feet on the ground ---
    base_box = ctx.part_world_aabb(base)
    ctx.check(
        "base feet rest on the ground",
        base_box is not None and -0.01 <= base_box[0][2] <= 0.02,
        details=f"base aabb={base_box}",
    )

    # --- Pivot height toddler-appropriate ---
    axle_box = ctx.part_element_world_aabb(base, elem="pivot_axle")
    ctx.check(
        "pivot axle sits at toddler-appropriate height (~0.42 m)",
        axle_box is not None and 0.35 <= (axle_box[0][2] + axle_box[1][2]) / 2.0 <= 0.50,
        details=f"axle aabb={axle_box}",
    )

    # --- Decisive pose: rocking alternates ends ---
    rest_b0 = ctx.part_element_world_aabb(beam, elem="bump_stop_0")
    rest_b1 = ctx.part_element_world_aabb(beam, elem="bump_stop_1")
    with ctx.pose({pivot: TILT}):
        down0 = ctx.part_element_world_aabb(beam, elem="bump_stop_0")
        up1 = ctx.part_element_world_aabb(beam, elem="bump_stop_1")
        ctx.check(
            "positive rock lowers head end",
            rest_b0 is not None
            and down0 is not None
            and down0[0][2] < rest_b0[0][2] - 0.04,
            details=f"rest={rest_b0}, tilted={down0}",
        )
        ctx.check(
            "positive rock raises tail end",
            rest_b1 is not None
            and up1 is not None
            and up1[0][2] > rest_b1[0][2] + 0.04,
            details=f"rest={rest_b1}, tilted={up1}",
        )
    with ctx.pose({pivot: -TILT}):
        down1 = ctx.part_element_world_aabb(beam, elem="bump_stop_1")
        ctx.check(
            "negative rock lowers tail end",
            down1 is not None
            and rest_b1 is not None
            and down1[0][2] < rest_b1[0][2] - 0.04,
            details=f"tilted={down1}",
        )

    return ctx.report()


object_model = build_object_model()
