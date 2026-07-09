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
)

# ---------------------------------------------------------------------------
# Variant 15: High-arc gooseneck faucet sibling.
#
# Layout (world frame, deck plane at z = 0):
# - +X is the front (direction the gooseneck reaches over the sink), +Z is up.
# - A stepped cylindrical pedestal (broad escutcheon → mid step → upper step)
#   in polished chrome replaces the single base disc.
# - A gloss-black cylindrical column (~0.04 m dia) rises from the pedestal.
# - A single side lever is mounted on the +Y (right) side of the column at
#   mid-height.  It pivots about the X axis (front-back), tilting from
#   horizontal-off (q = 0) upward to ~60 deg open (q = 1.05 rad).
# - Two small raised tick marks (hot / cold) sit on the column surface near
#   the lever's range, modeled as thin geometric boxes — not text.
# - A thin chrome collar ring separates the column from the swan-neck
#   gooseneck spout, which swivels about the vertical axis (revolute,
#   -110..+110 deg).  The tube arcs up to an apex near z = 0.38 and ends
#   in a short chrome tip sleeve with downward outlet.
# ---------------------------------------------------------------------------

# Stepped pedestal (3 tiers, all chrome)
ESCUTCHEON_R = 0.045   # broad base disc, 0.090 m dia
ESCUTCHEON_H = 0.005
MID_STEP_R = 0.032     # intermediate step
MID_STEP_H = 0.012
UPPER_STEP_R = 0.025   # transition to column
UPPER_STEP_H = 0.010
PEDESTAL_TOP = ESCUTCHEON_H + MID_STEP_H + UPPER_STEP_H  # 0.027

# Column
COLUMN_R = 0.020       # 0.04 m diameter
COLUMN_TOP = 0.135     # column shaft top (just below collar)

# Side lever
LEVER_Z = 0.082        # lever pivot height (mid-column)
LEVER_BOSS_R = 0.012   # boss cylinder radius
LEVER_BOSS_LEN = 0.014 # boss protrusion along +Y from column surface
LEVER_ARM_LEN = 0.075  # lever arm extends outward
LEVER_ARM_R = 0.007    # lever arm cylinder radius
LEVER_TIP_R = 0.009    # slightly larger tip knob

# Tick marks (small raised boxes on column surface near lever)
TICK_W = 0.003         # width along Z (vertical extent of the mark)
TICK_H = 0.010         # height along Y (protrudes from column surface)
TICK_D = 0.003         # depth along X (narrow line mark)
TICK_HOT_Z = LEVER_Z + 0.030   # above lever range
TICK_COLD_Z = LEVER_Z - 0.025  # below lever rest position

# Lever motion
LEVER_LOWER = 0.0
LEVER_UPPER = math.radians(60.0)  # ~1.047 rad

# Swivel collar + gooseneck (spout-local frame at the collar top)
COLLAR_R = 0.0215
COLLAR_LEN = 0.010
SWIVEL_Z = 0.140

TUBE_R = 0.015
ARC_R = 0.072
RISER_TOP = 0.153
REACH_X = 2.0 * ARC_R  # 0.144 m
DROP_END = 0.124

SLEEVE_R = 0.0165
SLEEVE_LEN = 0.028
AERATOR_R = 0.0118
AERATOR_LEN = 0.003

APEX_WORLD = SWIVEL_Z + RISER_TOP + ARC_R + TUBE_R  # ~0.380 m

SWIVEL_LIMIT = math.radians(110.0)


def _gooseneck_shape() -> cq.Workplane:
    """Swan-neck tube: straight riser, high semicircular arc, short drop leg."""
    path = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0)
        .lineTo(0.0, RISER_TOP)
        .threePointArc((ARC_R, RISER_TOP + ARC_R), (REACH_X, RISER_TOP))
        .lineTo(REACH_X, DROP_END)
    )
    return cq.Workplane("XY").circle(TUBE_R).sweep(path)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="high_arc_gooseneck_faucet_v15")

    gloss_black = model.material("veined_gloss_black", rgba=(0.075, 0.075, 0.085, 1.0))
    matte_black = model.material("matte_black", rgba=(0.035, 0.035, 0.038, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    outlet_dark = model.material("outlet_dark", rgba=(0.10, 0.10, 0.10, 1.0))
    tick_mark = model.material("tick_white", rgba=(0.75, 0.75, 0.72, 1.0))

    # ------------------------------------------------------------------ column body
    column = model.part("body_column")

    # --- Stepped pedestal (3 chrome tiers) ---
    # Tier 1: broad escutcheon disc
    column.visual(
        Cylinder(radius=ESCUTCHEON_R, length=ESCUTCHEON_H),
        origin=Origin(xyz=(0.0, 0.0, ESCUTCHEON_H / 2.0)),
        material=chrome,
        name="escutcheon",
    )
    # Tier 2: mid step
    column.visual(
        Cylinder(radius=MID_STEP_R, length=MID_STEP_H),
        origin=Origin(xyz=(0.0, 0.0, ESCUTCHEON_H + MID_STEP_H / 2.0)),
        material=chrome,
        name="mid_step",
    )
    # Tier 3: upper step
    column.visual(
        Cylinder(radius=UPPER_STEP_R, length=UPPER_STEP_H),
        origin=Origin(xyz=(0.0, 0.0, ESCUTCHEON_H + MID_STEP_H + UPPER_STEP_H / 2.0)),
        material=chrome,
        name="upper_step",
    )

    # --- Column shaft ---
    col_shaft_len = COLUMN_TOP - PEDESTAL_TOP
    column.visual(
        Cylinder(radius=COLUMN_R, length=col_shaft_len),
        origin=Origin(xyz=(0.0, 0.0, PEDESTAL_TOP + col_shaft_len / 2.0)),
        material=gloss_black,
        name="column_shaft",
    )

    # --- Tick marks (raised geometric boxes on column +Y surface) ---
    # Hot tick mark (above lever range)
    column.visual(
        Box((TICK_D, TICK_H, TICK_W)),
        origin=Origin(xyz=(0.0, COLUMN_R + TICK_H / 2.0, TICK_HOT_Z)),
        material=tick_mark,
        name="tick_hot",
    )
    # Cold tick mark (below lever rest)
    column.visual(
        Box((TICK_D, TICK_H, TICK_W)),
        origin=Origin(xyz=(0.0, COLUMN_R + TICK_H / 2.0, TICK_COLD_Z)),
        material=tick_mark,
        name="tick_cold",
    )

    # --- Chrome collar ring ---
    column.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_LEN),
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z - COLLAR_LEN / 2.0)),
        material=chrome,
        name="swivel_collar",
    )

    # --------------------------------------------------------------- gooseneck spout
    spout = model.part("gooseneck_spout")
    spout.visual(
        mesh_from_cadquery(_gooseneck_shape(), "gooseneck_tube"),
        material=gloss_black,
        name="gooseneck_tube",
    )
    spout.visual(
        Cylinder(radius=SLEEVE_R, length=SLEEVE_LEN),
        origin=Origin(xyz=(REACH_X, 0.0, DROP_END + SLEEVE_LEN / 2.0)),
        material=chrome,
        name="tip_sleeve",
    )
    spout.visual(
        Cylinder(radius=AERATOR_R, length=AERATOR_LEN),
        origin=Origin(xyz=(REACH_X, 0.0, DROP_END - 0.001)),
        material=outlet_dark,
        name="outlet_aerator",
    )
    model.articulation(
        "spout_swivel",
        ArticulationType.REVOLUTE,
        parent=column,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=1.5, lower=-SWIVEL_LIMIT, upper=SWIVEL_LIMIT
        ),
    )

    # --------------------------------------------------------------- side lever
    lever = model.part("side_lever")
    # Boss: cylindrical base that seats into the column sidewall
    # Extends along +Y from the pivot (at column surface)
    lever.visual(
        Cylinder(radius=LEVER_BOSS_R, length=LEVER_BOSS_LEN),
        origin=Origin(
            xyz=(0.0, LEVER_BOSS_LEN / 2.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=matte_black,
        name="lever_boss",
    )
    # Arm: slim cylinder extending outward from boss end
    arm_y0 = LEVER_BOSS_LEN
    arm_y1 = arm_y0 + LEVER_ARM_LEN
    lever.visual(
        Cylinder(radius=LEVER_ARM_R, length=LEVER_ARM_LEN),
        origin=Origin(
            xyz=(0.0, arm_y0 + LEVER_ARM_LEN / 2.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=gloss_black,
        name="lever_arm",
    )
    # Tip: slightly larger knob at lever end
    lever.visual(
        Cylinder(radius=LEVER_TIP_R, length=0.012),
        origin=Origin(
            xyz=(0.0, arm_y1 + 0.006, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=gloss_black,
        name="lever_tip",
    )

    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=column,
        child=lever,
        # Pivot at the column surface on the +Y side
        origin=Origin(xyz=(0.0, COLUMN_R, LEVER_Z)),
        # Axis along X (front-back); positive q tilts lever tip from +Y toward +Z (up)
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=LEVER_LOWER, upper=LEVER_UPPER
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    column = object_model.get_part("body_column")
    spout = object_model.get_part("gooseneck_spout")
    lever = object_model.get_part("side_lever")

    swivel = object_model.get_articulation("spout_swivel")
    pivot = object_model.get_articulation("lever_pivot")

    # Intentional seated insertion: lever boss embeds into column sidewall
    ctx.allow_overlap(
        lever,
        column,
        elem_a="lever_boss",
        elem_b="column_shaft",
        reason="Lever boss intentionally seats into the column sidewall for mounting.",
    )

    # ----- stepped pedestal base -----
    escutcheon = ctx.part_element_world_aabb(column, elem="escutcheon")
    mid_step = ctx.part_element_world_aabb(column, elem="mid_step")
    upper_step = ctx.part_element_world_aabb(column, elem="upper_step")
    ctx.check(
        "broad escutcheon is the widest pedestal tier (~0.09 m dia, thin)",
        escutcheon is not None
        and 0.088 <= (escutcheon[1][0] - escutcheon[0][0]) <= 0.092
        and (escutcheon[1][2] - escutcheon[0][2]) <= 0.006,
        details=f"escutcheon aabb={escutcheon}",
    )
    ctx.check(
        "stepped pedestal has 3 tiers of decreasing diameter",
        escutcheon is not None
        and mid_step is not None
        and upper_step is not None
        and (escutcheon[1][0] - escutcheon[0][0])
        > (mid_step[1][0] - mid_step[0][0])
        > (upper_step[1][0] - upper_step[0][0]),
        details=f"esc={escutcheon}, mid={mid_step}, upper={upper_step}",
    )
    ctx.check(
        "pedestal tiers stack vertically without gaps",
        escutcheon is not None
        and mid_step is not None
        and upper_step is not None
        and abs(mid_step[0][2] - escutcheon[1][2]) < 0.001
        and abs(upper_step[0][2] - mid_step[1][2]) < 0.001,
        details=f"esc_top={escutcheon[1][2]}, mid_bot={mid_step[0][2]}, "
        f"mid_top={mid_step[1][2]}, upper_bot={upper_step[0][2]}",
    )

    # ----- column shaft -----
    shaft = ctx.part_element_world_aabb(column, elem="column_shaft")
    ctx.check(
        "column shaft is ~0.04 m diameter",
        shaft is not None and 0.038 <= (shaft[1][0] - shaft[0][0]) <= 0.042,
        details=f"column shaft aabb={shaft}",
    )

    # ----- tick marks as geometry (not text) -----
    tick_hot = ctx.part_element_world_aabb(column, elem="tick_hot")
    tick_cold = ctx.part_element_world_aabb(column, elem="tick_cold")
    ctx.check(
        "hot tick mark is a visible raised geometry element on the column",
        tick_hot is not None
        and (tick_hot[1][1] - tick_hot[0][1]) > 0.002
        and (tick_hot[1][2] - tick_hot[0][2]) > 0.001
        and tick_hot[0][2] > LEVER_Z,
        details=f"tick_hot aabb={tick_hot}",
    )
    ctx.check(
        "cold tick mark is a visible raised geometry element on the column",
        tick_cold is not None
        and (tick_cold[1][1] - tick_cold[0][1]) > 0.002
        and (tick_cold[1][2] - tick_cold[0][2]) > 0.001
        and tick_cold[1][2] < LEVER_Z,
        details=f"tick_cold aabb={tick_cold}",
    )
    ctx.check(
        "tick marks are separated vertically along the column",
        tick_hot is not None
        and tick_cold is not None
        and tick_cold[1][2] < tick_hot[0][2],
        details=f"hot_z={tick_hot}, cold_z={tick_cold}",
    )

    # ----- gooseneck spout and swivel -----
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "gooseneck apex near 0.38 m",
        spout_aabb is not None and 0.372 <= spout_aabb[1][2] <= 0.388,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "gooseneck arcs forward over the sink (+X reach)",
        spout_aabb is not None and spout_aabb[1][0] >= 0.150,
        details=f"spout aabb={spout_aabb}",
    )

    collar = ctx.part_element_world_aabb(column, elem="swivel_collar")
    ctx.expect_contact(
        spout,
        column,
        elem_a="gooseneck_tube",
        elem_b="swivel_collar",
        contact_tol=0.001,
        name="gooseneck riser seats on the chrome collar",
    )

    ctx.check(
        "spout swivel is revolute -110..+110 deg about the vertical column axis",
        swivel.articulation_type == ArticulationType.REVOLUTE
        and tuple(swivel.axis) == (0.0, 0.0, 1.0)
        and swivel.motion_limits is not None
        and abs(swivel.motion_limits.lower + SWIVEL_LIMIT) < 1e-6
        and abs(swivel.motion_limits.upper - SWIVEL_LIMIT) < 1e-6,
    )

    # Swivel pose: spout sweeps sideways
    rest_sleeve = ctx.part_element_world_aabb(spout, elem="tip_sleeve")
    with ctx.pose({swivel: 1.0}):
        sw_sleeve = ctx.part_element_world_aabb(spout, elem="tip_sleeve")
    ctx.check(
        "spout swivel carries the outlet sideways about the vertical axis",
        rest_sleeve is not None
        and sw_sleeve is not None
        and abs(0.5 * (rest_sleeve[0][1] + rest_sleeve[1][1])) < 1e-6
        and 0.5 * (sw_sleeve[0][1] + sw_sleeve[1][1]) > 0.08,
        details=f"rest={rest_sleeve}, swiveled={sw_sleeve}",
    )

    # ----- single side lever -----
    lever_aabb = ctx.part_world_aabb(lever)
    arm = ctx.part_element_world_aabb(lever, elem="lever_arm")
    boss = ctx.part_element_world_aabb(lever, elem="lever_boss")
    ctx.check(
        "side lever is mounted on the column (boss seats into column sidewall)",
        lever_aabb is not None
        and boss is not None
        and boss[0][1] > 0.0
        and abs(boss[0][2] - LEVER_Z + 0.001) < LEVER_BOSS_R + 0.005,
        details=f"lever aabb={lever_aabb}, boss={boss}",
    )
    ctx.check(
        "lever arm extends outward from the column along +Y at rest",
        arm is not None and arm[1][1] > COLUMN_R + LEVER_ARM_LEN * 0.7,
        details=f"arm aabb={arm}",
    )

    # Lever boss overlap proof
    ctx.expect_overlap(
        lever,
        column,
        axes="z",
        elem_a="lever_boss",
        elem_b="column_shaft",
        min_overlap=0.003,
        name="lever boss seats into the column shaft",
    )

    # Lever pivot joint check
    ctx.check(
        "lever pivot is revolute with horizontal X axis, range 0..60 deg",
        pivot.articulation_type == ArticulationType.REVOLUTE
        and tuple(pivot.axis) == (1.0, 0.0, 0.0)
        and pivot.motion_limits is not None
        and abs(pivot.motion_limits.lower) < 1e-6
        and abs(pivot.motion_limits.upper - LEVER_UPPER) < 1e-6,
    )

    # Lever pose: positive q tilts lever tip upward
    rest_tip = ctx.part_element_world_aabb(lever, elem="lever_tip")
    with ctx.pose({pivot: LEVER_UPPER}):
        tilted_tip = ctx.part_element_world_aabb(lever, elem="lever_tip")
    ctx.check(
        "lever tilts upward at max angle (positive q raises tip)",
        rest_tip is not None
        and tilted_tip is not None
        and tilted_tip[1][2] > rest_tip[1][2] + 0.02
        and tilted_tip[1][1] < rest_tip[1][1] - 0.01,
        details=f"rest_tip={rest_tip}, tilted_tip={tilted_tip}",
    )

    # ----- grounding -----
    col_aabb = ctx.part_world_aabb(column)
    ctx.check(
        "tap grounded on the deck plane",
        col_aabb is not None and abs(col_aabb[0][2]) <= 0.002,
        details=f"column aabb={col_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
