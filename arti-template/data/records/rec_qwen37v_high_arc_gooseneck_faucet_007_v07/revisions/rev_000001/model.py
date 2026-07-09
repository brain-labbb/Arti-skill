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
# High-arc gooseneck faucet variant (variant 07) based on the gloss-black
# monobloc kitchen mixer tap.  ~0.38 m tall, deck-mounted.
#
# Changes from the parent asset:
#  - Removable circular deck plate under the chrome base disc.
#  - Pull-down spray head nested into the gooseneck mouth (prismatic joint).
#  - Spout swivel is now a CONTINUOUS vertical-axis joint at the collar.
#  - Visible cold/hot tick marks as raised geometry on the valve bodies.
#
# Layout (world frame, deck plane at z = 0):
# - +X is the front of the tap (the direction the gooseneck reaches over the
#   sink and the direction the pin levers tilt toward the user), +Z is up.
# ---------------------------------------------------------------------------

# Base + column
BASE_DISC_R = 0.042
BASE_DISC_H = 0.008
COLUMN_R = 0.020  # 0.04 m diameter per prompt
COLUMN_TOP = 0.132  # shaft reaches 2 mm into the collar for connectivity

# Deck plate (removable escutcheon under the base disc)
DECK_PLATE_R = 0.065
DECK_PLATE_H = 0.004

# Cross valve cylinder
CROSS_Z = 0.085
CROSS_R = 0.0225  # 0.045 m diameter per prompt
CROSS_TUBE_LEN = 0.170
CAP_LEN = 0.005  # flat end caps; total end-to-end = 0.170 + 2*0.005 = 0.180 m
CAP_R = 0.0235
CAP_Y = CROSS_TUBE_LEN / 2.0 + CAP_LEN / 2.0  # 0.0875

# Tick marks (raised hot/cold indicators on each valve cap)
TICK_W = 0.002
TICK_H = 0.003  # protrusion above cap surface
TICK_LEN = 0.012

# Pin levers (lever-local frame at the valve axis center)
LEVER_Y = 0.058  # outboard position of each lever along the cross
BOSS_R = 0.010
BOSS_LEN = 0.016
BOSS_Z = 0.026  # boss spans z 0.018..0.034; embeds ~4.5 mm into the valve
PIN_R = 0.006  # 0.012 m diameter per prompt
PIN_LEN = 0.100
PIN_Z0 = 0.032  # pin spans z 0.032..0.132 (overlaps boss top for connectivity)

# Swivel collar + gooseneck (spout-local frame at the collar top, z = SWIVEL_Z)
COLLAR_R = 0.0215
COLLAR_LEN = 0.010
SWIVEL_Z = 0.140

TUBE_R = 0.015
ARC_R = 0.072
RISER_TOP = 0.153  # centerline apex = 0.153 + 0.072 = 0.225; +TUBE_R -> 0.240
REACH_X = 2.0 * ARC_R  # 0.144 m horizontal reach
DROP_END = 0.124  # spout-local z of the open tube tip (world 0.264)

SLEEVE_R = 0.0165
SLEEVE_LEN = 0.028  # chrome tip sleeve spans local z 0.124..0.152
AERATOR_R = 0.0118
AERATOR_LEN = 0.003  # dark outlet ring, 1 mm proud below the sleeve mouth

APEX_WORLD = SWIVEL_Z + RISER_TOP + ARC_R + TUBE_R  # 0.380 m

# Pull-down spray head (spray_head-local frame at the seated position)
SPRAY_BODY_R = 0.013  # main body fits inside the tip sleeve
SPRAY_BODY_LEN = 0.040
SPRAY_GRIP_R = 0.015
SPRAY_GRIP_LEN = 0.028
SPRAY_FACE_R = 0.012
SPRAY_FACE_LEN = 0.003
PULLDOWN_RANGE = 0.10  # max pull-down travel in meters

# Spray head seated position in spout-local frame:
# The spray head hangs from the bottom of the tip sleeve.
# seated z center (spout-local) = DROP_END - SPRAY_GRIP_LEN/2
SPRAY_SEATED_Z = DROP_END - SPRAY_GRIP_LEN / 2.0


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
    model = ArticulatedObject(name="high_arc_gooseneck_faucet")

    gloss_black = model.material("veined_gloss_black", rgba=(0.075, 0.075, 0.085, 1.0))
    matte_black = model.material("matte_black", rgba=(0.035, 0.035, 0.038, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    outlet_dark = model.material("outlet_dark", rgba=(0.10, 0.10, 0.10, 1.0))
    tick_red = model.material("hot_tick_red", rgba=(0.55, 0.10, 0.08, 1.0))
    tick_blue = model.material("cold_tick_blue", rgba=(0.08, 0.12, 0.55, 1.0))
    spray_grip = model.material("spray_grip_rubber", rgba=(0.06, 0.06, 0.065, 1.0))

    # ------------------------------------------------------------------ deck plate
    deck_plate = model.part("deck_plate")
    deck_plate.visual(
        Cylinder(radius=DECK_PLATE_R, length=DECK_PLATE_H),
        origin=Origin(xyz=(0.0, 0.0, -DECK_PLATE_H / 2.0)),
        material=chrome,
        name="deck_plate_disc",
    )

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
    # Hot/cold tick marks: small raised indicators on top of each valve cap.
    # Hot (left, +Y) = red tick, Cold (right, -Y) = blue tick.
    # Placed on the top surface of the valve caps, protruding upward.
    tick_z = CROSS_Z + CROSS_R + TICK_H / 2.0
    column.visual(
        Box((TICK_LEN, TICK_W, TICK_H)),
        origin=Origin(xyz=(0.0, CAP_Y, tick_z)),
        material=tick_red,
        name="hot_tick",
    )
    column.visual(
        Box((TICK_LEN, TICK_W, TICK_H)),
        origin=Origin(xyz=(0.0, -CAP_Y, tick_z)),
        material=tick_blue,
        name="cold_tick",
    )
    # Thin chrome collar ring separating the column from the swivel spout.
    column.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_LEN),
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z - COLLAR_LEN / 2.0)),
        material=chrome,
        name="swivel_collar",
    )

    # Fixed joint: column mounted on the deck plate (deck plate sits under base)
    model.articulation(
        "deck_to_column",
        ArticulationType.FIXED,
        parent=deck_plate,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
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
    spout.visual(
        Cylinder(radius=AERATOR_R, length=AERATOR_LEN),
        origin=Origin(xyz=(REACH_X, 0.0, DROP_END - 0.001)),
        material=outlet_dark,
        name="outlet_aerator",
    )
    # CONTINUOUS spout swivel about the vertical column axis at the collar.
    model.articulation(
        "spout_swivel",
        ArticulationType.CONTINUOUS,
        parent=column,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5),
    )

    # ---------------------------------------------------------- spray head
    spray_head = model.part("spray_head")
    # Main cylindrical body that nests inside the gooseneck mouth
    spray_head.visual(
        Cylinder(radius=SPRAY_BODY_R, length=SPRAY_BODY_LEN),
        origin=Origin(xyz=(0.0, 0.0, SPRAY_BODY_LEN / 2.0)),
        material=matte_black,
        name="spray_body",
    )
    # Rubber grip section (slightly wider)
    spray_head.visual(
        Cylinder(radius=SPRAY_GRIP_R, length=SPRAY_GRIP_LEN),
        origin=Origin(xyz=(0.0, 0.0, -SPRAY_GRIP_LEN / 2.0)),
        material=spray_grip,
        name="spray_grip",
    )
    # Spray face disc at the bottom of the grip
    spray_head.visual(
        Cylinder(radius=SPRAY_FACE_R, length=SPRAY_FACE_LEN),
        origin=Origin(xyz=(0.0, 0.0, -SPRAY_GRIP_LEN - SPRAY_FACE_LEN / 2.0)),
        material=outlet_dark,
        name="spray_face",
    )
    # Prismatic pull-down joint: axis = -Z so positive q pulls the head down.
    # At q=0, the spray head frame coincides with the articulation frame
    # at the seated position inside the spout mouth.
    model.articulation(
        "spray_pulldown",
        ArticulationType.PRISMATIC,
        parent=spout,
        child=spray_head,
        origin=Origin(xyz=(REACH_X, 0.0, SPRAY_SEATED_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=10.0, velocity=0.5, lower=0.0, upper=PULLDOWN_RANGE
        ),
    )

    # ------------------------------------------------------------- pin levers
    # Two identical levers; numeric suffixes (no intrinsic left/right frame).
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
            # Axis is the valve cylinder's own left-right (Y) axis. With
            # axis -Y, negative q rotates the vertical pin toward +X (the
            # user side): q in [-pi/2, 0] tilts from vertical to horizontal.
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=5.0, velocity=2.0, lower=-math.pi / 2.0, upper=0.0
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck_plate = object_model.get_part("deck_plate")
    column = object_model.get_part("body_column")
    spout = object_model.get_part("gooseneck_spout")
    spray_head = object_model.get_part("spray_head")
    lever_0 = object_model.get_part("pin_lever_0")
    lever_1 = object_model.get_part("pin_lever_1")

    swivel = object_model.get_articulation("spout_swivel")
    pulldown = object_model.get_articulation("spray_pulldown")
    pivot_0 = object_model.get_articulation("lever_pivot_0")
    pivot_1 = object_model.get_articulation("lever_pivot_1")
    deck_to_col = object_model.get_articulation("deck_to_column")

    # Intentional seated insertions: each lever boss embeds a few mm into the
    # valve cylinder so the lever reads mounted, proven by expect_overlap below.
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

    # Spray head body nests inside the gooseneck drop leg and tip sleeve at rest.
    ctx.allow_overlap(
        spray_head,
        spout,
        elem_a="spray_body",
        elem_b="tip_sleeve",
        reason="Spray head body is intentionally nested inside the tip sleeve at rest.",
    )
    ctx.allow_overlap(
        spray_head,
        spout,
        elem_a="spray_body",
        elem_b="gooseneck_tube",
        reason="Spray head body is intentionally nested inside the gooseneck drop leg at rest.",
    )

    # ----- deck plate: removable circular plate under the base
    deck_aabb = ctx.part_world_aabb(deck_plate)
    ctx.check(
        "removable circular deck plate present under the base",
        deck_aabb is not None
        and 0.12 <= (deck_aabb[1][0] - deck_aabb[0][0]) <= 0.14
        and (deck_aabb[1][2] - deck_aabb[0][2]) <= 0.006,
        details=f"deck_plate aabb={deck_aabb}",
    )
    ctx.check(
        "deck plate sits at or below the deck plane (z <= 0)",
        deck_aabb is not None and deck_aabb[1][2] <= 0.001,
        details=f"deck_plate aabb={deck_aabb}",
    )
    ctx.check(
        "deck_to_column is a FIXED articulation",
        deck_to_col.articulation_type == ArticulationType.FIXED,
    )

    # ----- grounding, scale, proportions
    col_aabb = ctx.part_world_aabb(column)
    ctx.check(
        "tap grounded on the deck plane",
        col_aabb is not None and abs(col_aabb[0][2]) <= 0.002,
        details=f"column aabb={col_aabb}",
    )
    shaft = ctx.part_element_world_aabb(column, elem="column_shaft")
    ctx.check(
        "vertical column is ~0.04 m diameter",
        shaft is not None and 0.038 <= (shaft[1][0] - shaft[0][0]) <= 0.042,
        details=f"column shaft aabb={shaft}",
    )
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

    # ----- cross valve cylinder with flat black end caps
    cross = ctx.part_element_world_aabb(column, elem="cross_tube")
    cap_0 = ctx.part_element_world_aabb(column, elem="valve_end_cap_0")
    cap_1 = ctx.part_element_world_aabb(column, elem="valve_end_cap_1")
    ctx.check(
        "cross-cylinder is ~0.045 m diameter at mid-column height",
        cross is not None
        and 0.043 <= (cross[1][2] - cross[0][2]) <= 0.047
        and 0.06 <= 0.5 * (cross[0][2] + cross[1][2]) <= 0.11,
        details=f"cross aabb={cross}",
    )
    ctx.check(
        "valve assembly spans ~0.18 m end-to-end cap face to cap face",
        cap_0 is not None
        and cap_1 is not None
        and 0.178 <= (cap_0[1][1] - cap_1[0][1]) <= 0.182,
        details=f"cap_0={cap_0}, cap_1={cap_1}",
    )

    # ----- hot/cold tick marks as visible geometry
    hot_tick = ctx.part_element_world_aabb(column, elem="hot_tick")
    cold_tick = ctx.part_element_world_aabb(column, elem="cold_tick")
    ctx.check(
        "hot tick mark present on left valve body (+Y side)",
        hot_tick is not None
        and 0.5 * (hot_tick[0][1] + hot_tick[1][1]) > 0.04
        and hot_tick[1][2] > CROSS_Z,
        details=f"hot_tick aabb={hot_tick}",
    )
    ctx.check(
        "cold tick mark present on right valve body (-Y side)",
        cold_tick is not None
        and 0.5 * (cold_tick[0][1] + cold_tick[1][1]) < -0.04
        and cold_tick[1][2] > CROSS_Z,
        details=f"cold_tick aabb={cold_tick}",
    )
    ctx.check(
        "tick marks are small raised geometry (not large surfaces)",
        hot_tick is not None
        and cold_tick is not None
        and (hot_tick[1][0] - hot_tick[0][0]) <= 0.015
        and (hot_tick[1][1] - hot_tick[0][1]) <= 0.005
        and (hot_tick[1][2] - hot_tick[0][2]) <= 0.005,
        details=f"hot={hot_tick}, cold={cold_tick}",
    )

    # ----- chrome collar ring between column and spout; spout seats on it
    collar = ctx.part_element_world_aabb(column, elem="swivel_collar")
    ctx.check(
        "thin chrome collar sits above the cross and below the spout",
        collar is not None
        and cross is not None
        and collar[0][2] >= cross[1][2]
        and spout_aabb is not None
        and collar[1][2] <= spout_aabb[0][2] + 1e-6,
        details=f"collar={collar}, cross_top={cross[1][2] if cross else None}",
    )
    ctx.expect_contact(
        spout,
        column,
        elem_a="gooseneck_tube",
        elem_b="swivel_collar",
        contact_tol=0.001,
        name="gooseneck riser seats on the chrome collar",
    )

    # ----- chrome tip sleeve with downward outlet at the spout end
    sleeve = ctx.part_element_world_aabb(spout, elem="tip_sleeve")
    aerator = ctx.part_element_world_aabb(spout, elem="outlet_aerator")
    tube = ctx.part_element_world_aabb(spout, elem="gooseneck_tube")
    ctx.check(
        "chrome tip sleeve wraps the spout drop leg with a downward outlet",
        sleeve is not None
        and aerator is not None
        and tube is not None
        and 0.25 <= sleeve[0][2] <= 0.28
        and aerator[0][2] < sleeve[0][2]
        and aerator[0][2] <= SWIVEL_Z + DROP_END
        and abs(0.5 * (sleeve[0][0] + sleeve[1][0]) - REACH_X) <= 0.002,
        details=f"sleeve={sleeve}, aerator={aerator}, tube={tube}",
    )

    # ----- spray head: nested in gooseneck mouth with pull-down mechanism
    # Prove the spray body is centered within the gooseneck drop leg on XY
    ctx.expect_within(
        spray_head,
        spout,
        axes="xy",
        inner_elem="spray_body",
        outer_elem="gooseneck_tube",
        margin=0.005,
        name="spray body centered within the gooseneck tube on XY at rest",
    )
    spray_aabb = ctx.part_world_aabb(spray_head)
    ctx.check(
        "spray head is positioned at the gooseneck mouth",
        spray_aabb is not None
        and spray_aabb[0][0] >= 0.10
        and spray_aabb[0][2] <= 0.28
        and spray_aabb[1][2] <= 0.32,
        details=f"spray_head aabb={spray_aabb}",
    )
    spray_grip_aabb = ctx.part_element_world_aabb(spray_head, elem="spray_grip")
    spray_face_aabb = ctx.part_element_world_aabb(spray_head, elem="spray_face")
    ctx.check(
        "spray head has a grip section and a downward-facing spray face",
        spray_grip_aabb is not None
        and spray_face_aabb is not None
        and spray_face_aabb[0][2] < spray_grip_aabb[0][2],
        details=f"grip={spray_grip_aabb}, face={spray_face_aabb}",
    )

    # ----- spray pull-down joint: prismatic, axis -Z, range 0..0.10 m
    ctx.check(
        "spray_pulldown is prismatic along -Z with 0..0.10 m range",
        pulldown.articulation_type == ArticulationType.PRISMATIC
        and tuple(pulldown.axis) == (0.0, 0.0, -1.0)
        and pulldown.motion_limits is not None
        and abs(pulldown.motion_limits.lower) < 1e-6
        and abs(pulldown.motion_limits.upper - PULLDOWN_RANGE) < 1e-6,
    )
    # Prove pull-down moves the spray head downward
    rest_spray_z = ctx.part_world_position(spray_head)
    with ctx.pose({pulldown: PULLDOWN_RANGE}):
        extended_spray_z = ctx.part_world_position(spray_head)
    ctx.check(
        "spray head pulls downward (lower Z) at max pull-down",
        rest_spray_z is not None
        and extended_spray_z is not None
        and extended_spray_z[2] < rest_spray_z[2] - 0.05,
        details=f"rest_z={rest_spray_z[2] if rest_spray_z else None}, extended_z={extended_spray_z[2] if extended_spray_z else None}",
    )

    # ----- spout swivel: CONTINUOUS about vertical axis
    ctx.check(
        "spout swivel is CONTINUOUS about the vertical column axis",
        swivel.articulation_type == ArticulationType.CONTINUOUS
        and tuple(swivel.axis) == (0.0, 0.0, 1.0),
    )
    # Prove the swivel still rotates the spout outlet sideways
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

    # ----- pin levers: geometry and seating
    for lever, name in ((lever_0, "pin_lever_0"), (lever_1, "pin_lever_1")):
        pin = ctx.part_element_world_aabb(lever, elem="lever_pin")
        ctx.check(
            f"{name} pin is slim (0.012 m dia) and 0.10 m long, vertical at rest",
            pin is not None
            and 0.010 <= (pin[1][0] - pin[0][0]) <= 0.014
            and 0.098 <= (pin[1][2] - pin[0][2]) <= 0.102,
            details=f"pin aabb={pin}",
        )
        ctx.expect_overlap(
            lever,
            column,
            axes="z",
            elem_a="lever_boss",
            elem_b="cross_tube",
            min_overlap=0.003,
            name=f"{name} boss seats into the valve cylinder",
        )
    pin0 = ctx.part_element_world_aabb(lever_0, elem="lever_pin")
    pin1 = ctx.part_element_world_aabb(lever_1, elem="lever_pin")
    ctx.check(
        "the two pin levers rise from the tops of the two valve bodies",
        pin0 is not None
        and pin1 is not None
        and cross is not None
        and 0.5 * (pin0[0][1] + pin0[1][1]) > 0.04
        and 0.5 * (pin1[0][1] + pin1[1][1]) < -0.04
        and pin0[0][2] >= cross[1][2] - 0.001
        and pin1[0][2] >= cross[1][2] - 0.001,
        details=f"pin0={pin0}, pin1={pin1}",
    )

    # ----- lever pivot joints
    for pivot, name in ((pivot_0, "lever_pivot_0"), (pivot_1, "lever_pivot_1")):
        ctx.check(
            f"{name} is revolute -90..0 deg about the valve's left-right axis",
            pivot.articulation_type == ArticulationType.REVOLUTE
            and tuple(pivot.axis) == (0.0, -1.0, 0.0)
            and pivot.motion_limits is not None
            and abs(pivot.motion_limits.lower + math.pi / 2.0) < 1e-6
            and abs(pivot.motion_limits.upper) < 1e-6
            and pivot.mimic is None,
        )

    # ----- lever pose: full -90 deg tilt brings the pin toward the user (+X)
    rest_0 = ctx.part_world_aabb(lever_0)
    rest_1 = ctx.part_world_aabb(lever_1)
    with ctx.pose({pivot_0: -math.pi / 2.0}):
        tilted_0 = ctx.part_world_aabb(lever_0)
        still_1 = ctx.part_world_aabb(lever_1)
    ctx.check(
        "lever 0 tilts from vertical to horizontal toward the user at q=-90 deg",
        rest_0 is not None
        and tilted_0 is not None
        and tilted_0[1][0] > rest_0[1][0] + 0.10
        and tilted_0[1][2] < CROSS_Z + 0.03,
        details=f"rest={rest_0}, tilted={tilted_0}",
    )
    ctx.check(
        "lever 1 is independent of lever 0 (stays vertical while 0 tilts)",
        rest_1 is not None
        and still_1 is not None
        and abs(still_1[1][2] - rest_1[1][2]) < 1e-9,
        details=f"rest={rest_1}, while_0_tilted={still_1}",
    )

    # ----- joint count: at least one non-fixed joint
    non_fixed = [
        a for a in object_model.articulations
        if a.articulation_type != ArticulationType.FIXED
    ]
    ctx.check(
        "at least one non-fixed joint exists",
        len(non_fixed) >= 1,
        details=f"non-fixed joints: {[a.name for a in non_fixed]}",
    )

    return ctx.report()


object_model = build_object_model()
