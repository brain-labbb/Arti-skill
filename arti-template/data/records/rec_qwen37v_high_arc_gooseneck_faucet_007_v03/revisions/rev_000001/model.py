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
# Variant 03: squared-bridge high-arc gooseneck faucet, ~0.38 m tall.
#
# Layout (world frame, deck plane at z = 0):
# - +X is front (spout reach direction), +Z is up.
# - Chrome base disc on the deck; gloss-black cylindrical column (0.04 m dia).
# - Horizontal cross-cylinder (0.045 m dia, 0.18 m end-to-end along Y) at
#   z = 0.085, two valve bodies with flat black end caps.
# - Each cap has visible cold/hot tick marks (raised thin bars, not text).
# - From each valve body's top a slim pin lever (0.012 m dia, 0.10 m) points
#   up.  Each is an independent revolute joint about Y axis, -90..0 deg.
# - Chrome collar ring at z 0.130..0.140 separates column from spout.
# - SQUARED BRIDGE gooseneck: straight riser, rounded elbow, horizontal
#   bridge top, rounded elbow, drop leg.  Apex ~0.38 m.  Swivel about Z.
# - Flip-down aerator at the nozzle: separate part with revolute pivot about
#   Y axis at the sleeve bottom, range 0..70 deg (tilts outlet forward).
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
CAP_Y = CROSS_TUBE_LEN / 2.0 + CAP_LEN / 2.0  # 0.0875

# Tick marks on valve end caps
TICK_W = 0.002
TICK_H = 0.0012  # proud from cap surface
TICK_L = 0.010
TICK_COUNT = 3  # marks per cap
TICK_SPACING = 0.006  # spacing between ticks along X

# Pin levers
LEVER_Y = 0.058
BOSS_R = 0.010
BOSS_LEN = 0.016
BOSS_Z = 0.026
PIN_R = 0.006
PIN_LEN = 0.100
PIN_Z0 = 0.032

# Swivel collar + squared bridge gooseneck
COLLAR_R = 0.0215
COLLAR_LEN = 0.010
SWIVEL_Z = 0.140

TUBE_R = 0.015
BRIDGE_H = 0.225  # centerline height of horizontal bridge in spout-local
CORNER_R = 0.030  # softened elbow radius
BRIDGE_REACH = 0.140  # horizontal reach of bridge
DROP_END = 0.124  # spout-local z of tube drop-leg end

SLEEVE_R = 0.0165
SLEEVE_LEN = 0.028

# Flip-down aerator
AERATOR_R = 0.013
AERATOR_LEN = 0.018  # total aerator body length
AERATOR_SCREEN_R = 0.011
AERATOR_SCREEN_LEN = 0.003

APEX_WORLD = SWIVEL_Z + BRIDGE_H + TUBE_R  # 0.380 m
SWIVEL_LIMIT = math.radians(110.0)

# Aerator pivot at the bottom of the tip sleeve (spout-local z = DROP_END)
AERATOR_FLIP_LIMIT = math.radians(70.0)


def _bridge_gooseneck_shape() -> cq.Workplane:
    """Squared bridge tube: riser, rounded elbow, bridge, rounded elbow, drop."""
    path = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0)
        .lineTo(0.0, BRIDGE_H - CORNER_R)
        .tangentArcPoint((CORNER_R, CORNER_R), relative=True)
        .lineTo(BRIDGE_REACH - CORNER_R, BRIDGE_H)
        .tangentArcPoint((CORNER_R, -CORNER_R), relative=True)
        .lineTo(BRIDGE_REACH, DROP_END)
    )
    return cq.Workplane("XY").circle(TUBE_R).sweep(path)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="squared_bridge_gooseneck_faucet")

    gloss_black = model.material("veined_gloss_black", rgba=(0.075, 0.075, 0.085, 1.0))
    matte_black = model.material("matte_black", rgba=(0.035, 0.035, 0.038, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    outlet_dark = model.material("outlet_dark", rgba=(0.10, 0.10, 0.10, 1.0))
    tick_mark = model.material("tick_silver", rgba=(0.70, 0.72, 0.74, 1.0))

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
    # Horizontal cross valve cylinder
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

    # Cold/hot tick marks as raised thin bars on each valve end cap face.
    # Cap 0 (+Y side, hot): 3 ticks; Cap 1 (-Y side, cold): 3 ticks.
    cap_outer_y = CAP_Y + CAP_LEN / 2.0
    for cap_idx, y_sign in ((0, 1.0), (1, -1.0)):
        for t in range(TICK_COUNT):
            x_off = (t - (TICK_COUNT - 1) / 2.0) * TICK_SPACING
            tick_name = f"tick_{cap_idx}_{t}"
            column.visual(
                Box((TICK_W, TICK_H, TICK_L)),
                origin=Origin(xyz=(x_off, y_sign * (cap_outer_y + TICK_H / 2.0), CROSS_Z)),
                material=tick_mark,
                name=tick_name,
            )

    # Chrome collar ring
    column.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_LEN),
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z - COLLAR_LEN / 2.0)),
        material=chrome,
        name="swivel_collar",
    )

    # --------------------------------------------------------------- gooseneck
    spout = model.part("gooseneck_spout")
    spout.visual(
        mesh_from_cadquery(_bridge_gooseneck_shape(), "bridge_gooseneck_tube"),
        material=gloss_black,
        name="gooseneck_tube",
    )
    spout.visual(
        Cylinder(radius=SLEEVE_R, length=SLEEVE_LEN),
        origin=Origin(xyz=(BRIDGE_REACH, 0.0, DROP_END + SLEEVE_LEN / 2.0)),
        material=chrome,
        name="tip_sleeve",
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

    # ------------------------------------------------------- flip-down aerator
    # The aerator pivots at the bottom of the tip sleeve.  Part origin sits at
    # the pivot point; aerator body extends downward (-Z in local frame).
    aerator = model.part("flip_aerator")
    aerator.visual(
        Cylinder(radius=AERATOR_R, length=AERATOR_LEN),
        origin=Origin(xyz=(0.0, 0.0, -AERATOR_LEN / 2.0)),
        material=chrome,
        name="aerator_body",
    )
    aerator.visual(
        Cylinder(radius=AERATOR_SCREEN_R, length=AERATOR_SCREEN_LEN),
        origin=Origin(xyz=(0.0, 0.0, -AERATOR_LEN + AERATOR_SCREEN_LEN / 2.0)),
        material=outlet_dark,
        name="aerator_screen",
    )
    # Hinge boss connecting aerator to the sleeve bottom (small cylinder)
    aerator.visual(
        Cylinder(radius=0.008, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=chrome,
        name="aerator_hinge_boss",
    )
    model.articulation(
        "aerator_pivot",
        ArticulationType.REVOLUTE,
        parent=spout,
        child=aerator,
        # Origin in spout-local frame; spout-local z=DROP_END maps to
        # world z = SWIVEL_Z + DROP_END (bottom of the tip sleeve).
        origin=Origin(xyz=(BRIDGE_REACH, 0.0, DROP_END)),
        # Axis along -Y; positive q tilts the aerator bottom forward (+X).
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0, lower=0.0, upper=AERATOR_FLIP_LIMIT
        ),
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
    aerator = object_model.get_part("flip_aerator")
    lever_0 = object_model.get_part("pin_lever_0")
    lever_1 = object_model.get_part("pin_lever_1")

    swivel = object_model.get_articulation("spout_swivel")
    aerator_pivot = object_model.get_articulation("aerator_pivot")
    pivot_0 = object_model.get_articulation("lever_pivot_0")
    pivot_1 = object_model.get_articulation("lever_pivot_1")

    # Intentional seated insertions
    ctx.allow_overlap(
        lever_0, column,
        elem_a="lever_boss", elem_b="cross_tube",
        reason="Lever boss seats a few mm into the valve cylinder.",
    )
    ctx.allow_overlap(
        lever_1, column,
        elem_a="lever_boss", elem_b="cross_tube",
        reason="Lever boss seats a few mm into the valve cylinder.",
    )
    # Aerator hinge boss embeds slightly into the spout tip sleeve
    ctx.allow_overlap(
        aerator, spout,
        elem_a="aerator_hinge_boss", elem_b="tip_sleeve",
        reason="Aerator hinge boss seats into the tip sleeve bottom for pivot mount.",
    )
    # Aerator body seats into the gooseneck tube drop-leg end (pivot connection)
    ctx.allow_overlap(
        aerator, spout,
        elem_a="aerator_body", elem_b="gooseneck_tube",
        reason="Aerator body is seated into the tube drop-leg end as a pivot housing.",
    )
    # Hinge boss also contacts the tube end at the pivot point
    ctx.allow_overlap(
        aerator, spout,
        elem_a="aerator_hinge_boss", elem_b="gooseneck_tube",
        reason="Hinge boss passes through the tube end face at the aerator pivot point.",
    )

    # ----- grounding and scale
    col_aabb = ctx.part_world_aabb(column)
    ctx.check(
        "faucet grounded on the deck plane",
        col_aabb is not None and abs(col_aabb[0][2]) <= 0.002,
        details=f"column aabb={col_aabb}",
    )

    # ----- squared bridge gooseneck silhouette
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "gooseneck apex near 0.38 m",
        spout_aabb is not None and 0.372 <= spout_aabb[1][2] <= 0.388,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "gooseneck arcs forward over the sink (+X reach)",
        spout_aabb is not None and spout_aabb[1][0] >= 0.120,
        details=f"spout aabb={spout_aabb}",
    )

    # The squared bridge should have a visible horizontal section: the bridge
    # top must be significantly wider in X than the tube diameter, proving the
    # squared profile (not a smooth arc).
    tube = ctx.part_element_world_aabb(spout, elem="gooseneck_tube")
    ctx.check(
        "squared bridge tube has a horizontal span well beyond the tube diameter",
        tube is not None and (tube[1][0] - tube[0][0]) >= 0.10,
        details=f"tube aabb={tube}",
    )
    # The bridge top centerline is at a nearly constant Z, so the tube Z extent
    # should show the bridge top near the apex (not a smooth peak).
    ctx.check(
        "bridge top sits near BRIDGE_H + SWIVEL_Z (flat top, not peaked arc)",
        tube is not None
        and abs(tube[1][2] - (SWIVEL_Z + BRIDGE_H + TUBE_R)) <= 0.005,
        details=f"tube aabb={tube}",
    )

    # ----- chrome collar ring
    collar = ctx.part_element_world_aabb(column, elem="swivel_collar")
    cross = ctx.part_element_world_aabb(column, elem="cross_tube")
    ctx.check(
        "thin chrome collar sits above the cross and below the spout",
        collar is not None and cross is not None
        and collar[0][2] >= cross[1][2]
        and spout_aabb is not None
        and collar[1][2] <= spout_aabb[0][2] + 1e-6,
        details=f"collar={collar}",
    )
    ctx.expect_contact(
        spout, column,
        elem_a="gooseneck_tube", elem_b="swivel_collar",
        contact_tol=0.001,
        name="gooseneck riser seats on the chrome collar",
    )

    # ----- cold/hot tick marks as geometry on valve end caps
    for cap_idx in (0, 1):
        for t in range(TICK_COUNT):
            tick_name = f"tick_{cap_idx}_{t}"
            tick_aabb = ctx.part_element_world_aabb(column, elem=tick_name)
            ctx.check(
                f"tick mark {tick_name} exists as raised geometry on cap {cap_idx}",
                tick_aabb is not None
                and (tick_aabb[1][2] - tick_aabb[0][2]) >= 0.008,
                details=f"{tick_name} aabb={tick_aabb}",
            )

    # Verify ticks protrude from the cap faces (y-direction)
    cap_0 = ctx.part_element_world_aabb(column, elem="valve_end_cap_0")
    cap_1 = ctx.part_element_world_aabb(column, elem="valve_end_cap_1")
    tick_0_1 = ctx.part_element_world_aabb(column, elem="tick_0_1")
    tick_1_1 = ctx.part_element_world_aabb(column, elem="tick_1_1")
    ctx.check(
        "tick marks protrude outward from valve end cap faces",
        cap_0 is not None and tick_0_1 is not None
        and tick_0_1[1][1] > cap_0[1][1] - 0.0001
        and cap_1 is not None and tick_1_1 is not None
        and tick_1_1[0][1] < cap_1[0][1] + 0.0001,
        details=f"cap_0={cap_0}, tick_0_1={tick_0_1}, cap_1={cap_1}, tick_1_1={tick_1_1}",
    )

    # ----- flip-down aerator at nozzle
    aerator_aabb = ctx.part_world_aabb(aerator)
    sleeve = ctx.part_element_world_aabb(spout, elem="tip_sleeve")
    ctx.check(
        "flip-down aerator hangs below the tip sleeve at rest",
        aerator_aabb is not None and sleeve is not None
        and aerator_aabb[0][2] < sleeve[0][2],
        details=f"aerator={aerator_aabb}, sleeve={sleeve}",
    )

    # Aerator pivot joint: revolute about Y, 0..70 deg
    ctx.check(
        "aerator pivot is revolute about Y axis, 0..70 deg",
        aerator_pivot.articulation_type == ArticulationType.REVOLUTE
        and abs(aerator_pivot.axis[1]) > 0.99
        and aerator_pivot.motion_limits is not None
        and abs(aerator_pivot.motion_limits.lower) < 1e-6
        and abs(aerator_pivot.motion_limits.upper - AERATOR_FLIP_LIMIT) < 1e-6,
    )

    # Pose check: aerator flips forward (+X) at upper limit
    rest_aerator = ctx.part_world_aabb(aerator)
    with ctx.pose({aerator_pivot: AERATOR_FLIP_LIMIT}):
        flipped_aerator = ctx.part_world_aabb(aerator)
    ctx.check(
        "aerator flips forward (+X) at upper limit",
        rest_aerator is not None and flipped_aerator is not None
        and flipped_aerator[1][0] > rest_aerator[1][0] + 0.005,
        details=f"rest={rest_aerator}, flipped={flipped_aerator}",
    )

    # Aerator hinge boss contact with spout tip sleeve
    ctx.expect_overlap(
        aerator, spout,
        axes="z",
        elem_a="aerator_hinge_boss", elem_b="tip_sleeve",
        min_overlap=0.002,
        name="aerator hinge boss seats into the tip sleeve",
    )
    # Aerator body is seated at the tube drop-leg end (proof for allow_overlap)
    ctx.expect_contact(
        aerator, spout,
        elem_a="aerator_body", elem_b="gooseneck_tube",
        contact_tol=0.020,
        name="aerator body contacts the tube drop-leg end at pivot",
    )

    # ----- spout swivel joint
    ctx.check(
        "spout swivel is revolute -110..+110 deg about the vertical axis",
        swivel.articulation_type == ArticulationType.REVOLUTE
        and tuple(swivel.axis) == (0.0, 0.0, 1.0)
        and swivel.motion_limits is not None
        and abs(swivel.motion_limits.lower + SWIVEL_LIMIT) < 1e-6
        and abs(swivel.motion_limits.upper - SWIVEL_LIMIT) < 1e-6,
    )

    # ----- pin lever joints
    for pivot, name in ((pivot_0, "lever_pivot_0"), (pivot_1, "lever_pivot_1")):
        ctx.check(
            f"{name} is revolute -90..0 deg about the valve Y axis",
            pivot.articulation_type == ArticulationType.REVOLUTE
            and tuple(pivot.axis) == (0.0, -1.0, 0.0)
            and pivot.motion_limits is not None
            and abs(pivot.motion_limits.lower + math.pi / 2.0) < 1e-6
            and abs(pivot.motion_limits.upper) < 1e-6,
        )

    # Lever pose: full tilt toward user
    rest_0 = ctx.part_world_aabb(lever_0)
    with ctx.pose({pivot_0: -math.pi / 2.0}):
        tilted_0 = ctx.part_world_aabb(lever_0)
    ctx.check(
        "lever 0 tilts from vertical to horizontal toward user at q=-90 deg",
        rest_0 is not None and tilted_0 is not None
        and tilted_0[1][0] > rest_0[1][0] + 0.08
        and tilted_0[1][2] < CROSS_Z + 0.03,
        details=f"rest={rest_0}, tilted={tilted_0}",
    )

    # Swivel pose: spout sweeps sideways
    rest_sleeve = ctx.part_element_world_aabb(spout, elem="tip_sleeve")
    with ctx.pose({swivel: 1.0}):
        sw_sleeve = ctx.part_element_world_aabb(spout, elem="tip_sleeve")
    ctx.check(
        "spout swivel carries the outlet sideways about the vertical axis",
        rest_sleeve is not None and sw_sleeve is not None
        and abs(0.5 * (rest_sleeve[0][1] + rest_sleeve[1][1])) < 0.005
        and abs(0.5 * (sw_sleeve[0][1] + sw_sleeve[1][1])) > 0.06,
        details=f"rest={rest_sleeve}, swiveled={sw_sleeve}",
    )

    return ctx.report()


object_model = build_object_model()
