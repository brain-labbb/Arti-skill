from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# High-arc gooseneck faucet variant 10 (forked from gloss-black monobloc mixer).
#
# Layout (world frame, deck plane at z = 0):
# - +X is the front of the tap (direction the gooseneck reaches over the sink
#   and pin levers tilt toward the user), +Z is up.
# - Chrome base disc on the deck; gloss-black cylindrical column (0.04 m dia).
# - Horizontal cross-cylinder (0.045 m dia, 0.18 m end-to-end along Y) with
#   flat matte-black end caps.
# - Two slim vertical pin levers (revolute, -90..0 deg about valve Y axis).
# - Thin chrome collar ring separates column from swan-neck gooseneck spout.
# - A rear support strut braces from the base rear to the high arc region.
# - Spout swivels on a CONTINUOUS vertical joint at the base (unlimited).
# - Spray head has shallow ribbing rings and a distinct hollow outlet opening.
# ---------------------------------------------------------------------------

# Base + column
BASE_DISC_R = 0.042
BASE_DISC_H = 0.008
COLUMN_R = 0.020
COLUMN_TOP = 0.132

# Cross valve cylinder
CROSS_Z = 0.085
CROSS_R = 0.0225
CROSS_TUBE_LEN = 0.170
CAP_LEN = 0.005
CAP_R = 0.0235
CAP_Y = CROSS_TUBE_LEN / 2.0 + CAP_LEN / 2.0

# Pin levers
LEVER_Y = 0.058
BOSS_R = 0.010
BOSS_LEN = 0.016
BOSS_Z = 0.026
PIN_R = 0.006
PIN_LEN = 0.100
PIN_Z0 = 0.032

# Swivel collar + gooseneck
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
AERATOR_OUTER_R = 0.014
AERATOR_INNER_R = 0.008
AERATOR_LEN = 0.005

APEX_WORLD = SWIVEL_Z + RISER_TOP + ARC_R + TUBE_R  # 0.380 m

# Rear support strut
STRUT_WIDTH = 0.014
STRUT_THICK = 0.005
STRUT_Z_START = 0.008
STRUT_Z_END = 0.135
STRUT_X_OFFSET = -0.024  # behind column center

# Ribbing on spray head
RIB_COUNT = 3
RIB_OUTER_R = SLEEVE_R + 0.002
RIB_THICKNESS = 0.0015
RIB_SPACING = 0.007


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


def _hollow_outlet_shape() -> cq.Workplane:
    """Ring/washer shape for a distinct hollow outlet opening."""
    return (
        cq.Workplane("XY")
        .circle(AERATOR_OUTER_R)
        .circle(AERATOR_INNER_R)
        .extrude(AERATOR_LEN)
    )


def _rear_support_strut() -> cq.Workplane:
    """Rear support strut from base rear up to the collar/high-arc region.

    A flat plate behind the column that tapers from wider at the base to
    narrower near the top, providing structural bracing.
    """
    height = STRUT_Z_END - STRUT_Z_START
    # Build a tapered plate using a loft between two rectangles
    bottom_hw = STRUT_WIDTH / 2.0  # half-width at base
    top_hw = STRUT_WIDTH * 0.35  # narrower at top
    half_t = STRUT_THICK / 2.0

    # Bottom rectangle (wider)
    bottom = (
        cq.Workplane("XY")
        .transformed(offset=(STRUT_X_OFFSET, 0.0, STRUT_Z_START))
        .rect(bottom_hw * 2, STRUT_THICK)
    )
    # Top rectangle (narrower)
    top = (
        cq.Workplane("XY")
        .transformed(offset=(STRUT_X_OFFSET + 0.004, 0.0, STRUT_Z_END))
        .rect(top_hw * 2, STRUT_THICK)
    )
    # Use a simple box approximation instead of loft for reliability
    # Angled plate from base rear to collar
    mid_z = (STRUT_Z_START + STRUT_Z_END) / 2.0
    mid_x = (STRUT_X_OFFSET + STRUT_X_OFFSET + 0.004) / 2.0
    strut_length = math.sqrt(
        (STRUT_Z_END - STRUT_Z_START) ** 2 + 0.004**2
    )
    angle = math.atan2(0.004, STRUT_Z_END - STRUT_Z_START)

    strut = (
        cq.Workplane("XY")
        .transformed(offset=(mid_x, 0.0, mid_z))
        .transformed(rotate=(0.0, -math.degrees(angle), 0.0))
        .box(STRUT_WIDTH, STRUT_THICK, strut_length)
    )
    return strut


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="high_arc_gooseneck_faucet_v10")

    gloss_black = model.material("veined_gloss_black", rgba=(0.075, 0.075, 0.085, 1.0))
    matte_black = model.material("matte_black", rgba=(0.035, 0.035, 0.038, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    outlet_dark = model.material("outlet_dark", rgba=(0.10, 0.10, 0.10, 1.0))
    strut_mat = model.material("brushed_black", rgba=(0.06, 0.06, 0.07, 1.0))

    # ------------------------------------------------------------------ column
    column = model.part("body_column")
    column.visual(
        Cylinder(radius=BASE_DISC_R, length=BASE_DISC_H),
        origin=Origin(xyz=(0.0, 0.0, BASE_DISC_H / 2.0)),
        material=chrome,
        name="base_disc",
    )
    column.visual(
        Cylinder(radius=COLUMN_R, length=COLUMN_TOP - 0.004),
        origin=Origin(xyz=(0.0, 0.0, (COLUMN_TOP + 0.004) / 2.0)),
        material=gloss_black,
        name="column_shaft",
    )
    # Horizontal cross valve cylinder through the column, left-right (Y axis).
    column.visual(
        Cylinder(radius=CROSS_R, length=CROSS_TUBE_LEN),
        origin=Origin(xyz=(0.0, 0.0, CROSS_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gloss_black,
        name="cross_tube",
    )
    column.visual(
        Cylinder(radius=CAP_R, length=CAP_LEN),
        origin=Origin(xyz=(0.0, CAP_Y, CROSS_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="valve_end_cap_0",
    )
    column.visual(
        Cylinder(radius=CAP_R, length=CAP_LEN),
        origin=Origin(xyz=(0.0, -CAP_Y, CROSS_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="valve_end_cap_1",
    )
    # Thin chrome collar ring separating the column from the swivel spout.
    column.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_LEN),
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z - COLLAR_LEN / 2.0)),
        material=chrome,
        name="swivel_collar",
    )
    # Rear support strut from base to high arc
    column.visual(
        mesh_from_cadquery(_rear_support_strut(), "rear_strut"),
        material=strut_mat,
        name="rear_support_strut",
    )

    # --------------------------------------------------------------- gooseneck
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
    # Hollow outlet opening (ring/washer with visible bore)
    spout.visual(
        mesh_from_cadquery(_hollow_outlet_shape(), "hollow_outlet"),
        origin=Origin(xyz=(REACH_X, 0.0, DROP_END - AERATOR_LEN)),
        material=outlet_dark,
        name="hollow_outlet",
    )
    # Shallow ribbing rings on the spray head
    for i in range(RIB_COUNT):
        rib_z = DROP_END + 0.004 + i * RIB_SPACING
        spout.visual(
            Cylinder(radius=RIB_OUTER_R, length=RIB_THICKNESS),
            origin=Origin(xyz=(REACH_X, 0.0, rib_z)),
            material=chrome,
            name=f"spray_rib_{i}",
        )

    # Spout swivel: CONTINUOUS joint about the vertical column axis
    model.articulation(
        "spout_swivel",
        ArticulationType.CONTINUOUS,
        parent=column,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.0),
    )

    # ------------------------------------------------------------- pin levers
    for idx, y_sign in ((0, 1.0), (1, -1.0)):
        lever = model.part(f"pin_lever_{idx}")
        lever.visual(
            Cylinder(radius=BOSS_R, length=BOSS_LEN),
            origin=Origin(xyz=(0.0, 0.0, BOSS_Z)),
            material=gloss_black,
            name="lever_boss",
        )
        lever.visual(
            Cylinder(radius=PIN_R, length=PIN_LEN),
            origin=Origin(xyz=(0.0, 0.0, PIN_Z0 + PIN_LEN / 2.0)),
            material=gloss_black,
            name="lever_pin",
        )
        model.articulation(
            f"lever_pivot_{idx}",
            ArticulationType.REVOLUTE,
            parent=column,
            child=lever,
            origin=Origin(xyz=(0.0, y_sign * LEVER_Y, CROSS_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=5.0, velocity=2.0, lower=-math.pi / 2.0, upper=0.0
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    column = object_model.get_part("body_column")
    spout = object_model.get_part("gooseneck_spout")
    lever_0 = object_model.get_part("pin_lever_0")
    lever_1 = object_model.get_part("pin_lever_1")

    swivel = object_model.get_articulation("spout_swivel")
    pivot_0 = object_model.get_articulation("lever_pivot_0")
    pivot_1 = object_model.get_articulation("lever_pivot_1")

    # Intentional seated insertions
    ctx.allow_overlap(
        lever_0,
        column,
        elem_a="lever_boss",
        elem_b="cross_tube",
        reason="Lever boss intentionally seats a few mm into the valve cylinder.",
    )
    ctx.allow_overlap(
        lever_1,
        column,
        elem_a="lever_boss",
        elem_b="cross_tube",
        reason="Lever boss intentionally seats a few mm into the valve cylinder.",
    )

    # ----- grounding, scale, proportions
    col_aabb = ctx.part_world_aabb(column)
    ctx.check(
        "tap grounded on the deck plane",
        col_aabb is not None and abs(col_aabb[0][2]) <= 0.002,
        details=f"column aabb={col_aabb}",
    )
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "gooseneck apex near 0.38 m (tall arcing silhouette)",
        spout_aabb is not None and 0.370 <= spout_aabb[1][2] <= 0.390,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "gooseneck arcs forward over the sink (+X reach)",
        spout_aabb is not None and spout_aabb[1][0] >= 0.140,
        details=f"spout aabb={spout_aabb}",
    )

    # ----- rear support strut
    strut_aabb = ctx.part_element_world_aabb(column, elem="rear_support_strut")
    ctx.check(
        "rear support strut exists behind the column from base to high arc region",
        strut_aabb is not None
        and strut_aabb[0][0] < -0.015  # behind column center
        and strut_aabb[0][2] <= 0.015  # starts near base
        and strut_aabb[1][2] >= 0.120,  # reaches high arc region
        details=f"strut aabb={strut_aabb}",
    )

    # ----- hollow outlet opening
    outlet_aabb = ctx.part_element_world_aabb(spout, elem="hollow_outlet")
    ctx.check(
        "distinct hollow outlet opening at spout tip (ring with bore)",
        outlet_aabb is not None
        and outlet_aabb[0][2] < SWIVEL_Z + DROP_END + 0.001
        and 0.026 <= (outlet_aabb[1][0] - outlet_aabb[0][0]) <= 0.032,
        details=f"outlet aabb={outlet_aabb}",
    )

    # ----- spray head ribbing
    rib_aabbs = []
    for i in range(RIB_COUNT):
        rib = ctx.part_element_world_aabb(spout, elem=f"spray_rib_{i}")
        rib_aabbs.append(rib)
    ctx.check(
        "shallow ribbing rings on the spray head (3 rings)",
        all(r is not None for r in rib_aabbs)
        and all(
            (r[1][0] - r[0][0]) > 2.0 * SLEEVE_R
            for r in rib_aabbs
        ),
        details=f"ribs={rib_aabbs}",
    )

    # ----- spout swivel: CONTINUOUS joint about vertical axis
    ctx.check(
        "spout swivel is a continuous joint about the vertical column axis",
        swivel.articulation_type == ArticulationType.CONTINUOUS
        and tuple(swivel.axis) == (0.0, 0.0, 1.0),
    )

    # ----- lever joints remain revolute with proper limits
    for pivot, name in ((pivot_0, "lever_pivot_0"), (pivot_1, "lever_pivot_1")):
        ctx.check(
            f"{name} is revolute -90..0 deg about the valve's left-right axis",
            pivot.articulation_type == ArticulationType.REVOLUTE
            and tuple(pivot.axis) == (0.0, -1.0, 0.0)
            and pivot.motion_limits is not None
            and abs(pivot.motion_limits.lower + math.pi / 2.0) < 1e-6
            and abs(pivot.motion_limits.upper) < 1e-6,
        )

    # ----- swivel pose: continuous joint rotates spout outlet sideways
    rest_sleeve = ctx.part_element_world_aabb(spout, elem="tip_sleeve")
    with ctx.pose({swivel: 1.5}):
        sw_sleeve = ctx.part_element_world_aabb(spout, elem="tip_sleeve")
    ctx.check(
        "continuous spout swivel carries the outlet sideways about the vertical axis",
        rest_sleeve is not None
        and sw_sleeve is not None
        and abs(0.5 * (rest_sleeve[0][1] + rest_sleeve[1][1])) < 0.002
        and abs(0.5 * (sw_sleeve[0][1] + sw_sleeve[1][1])) > 0.08,
        details=f"rest={rest_sleeve}, swiveled={sw_sleeve}",
    )

    # ----- lever pose check
    rest_0 = ctx.part_world_aabb(lever_0)
    with ctx.pose({pivot_0: -math.pi / 2.0}):
        tilted_0 = ctx.part_world_aabb(lever_0)
    ctx.check(
        "lever 0 tilts from vertical toward the user at q=-90 deg",
        rest_0 is not None
        and tilted_0 is not None
        and tilted_0[1][0] > rest_0[1][0] + 0.08,
        details=f"rest={rest_0}, tilted={tilted_0}",
    )

    return ctx.report()


object_model = build_object_model()
