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
# High-arc gooseneck faucet variant — gloss-black monobloc kitchen mixer tap
# with a ribbed spray head and flip-down outlet aerator.
#
# Layout (world frame, deck plane at z = 0):
# - +X is the front of the tap (direction the gooseneck reaches over the
#   sink), +Z is up.
# - Chrome base disc on the deck; gloss-black cylindrical column (0.04 m dia).
# - Horizontal cross-cylinder (0.045 m dia, 0.18 m end-to-end along Y) at
#   z = 0.085, forming two valve bodies with flat black end caps.
# - Pin levers from each valve body (independent revolute joints, -90..0 deg).
# - Chrome collar ring separates column from the swan-neck gooseneck spout
#   (swivel revolute, -110..+110 deg about vertical).
# - At the spout end: a ribbed cylindrical spray head (gloss black) replaces
#   the plain tip sleeve. A flip-down aerator disc pivots at the +X edge of
#   the spray head bottom (revolute, 0..70 deg about the lateral Y axis).
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

APEX_WORLD = SWIVEL_Z + RISER_TOP + ARC_R + TUBE_R  # 0.380 m
SWIVEL_LIMIT = math.radians(110.0)

# Spray head (ribbed cylinder replacing the plain tip sleeve)
SPRAY_R = 0.017
SPRAY_LEN = 0.042
SPRAY_OVERLAP = 0.003
SPRAY_BOTTOM_LOCAL = DROP_END - SPRAY_LEN + SPRAY_OVERLAP  # 0.085

# Shallow ribbing on spray head
NUM_RIBS = 6
RIB_PROUD = 0.001
RIB_WIDTH = 0.002

# Chrome transition ring at spray head top
SPRAY_COLLAR_R = SPRAY_R + 0.002
SPRAY_COLLAR_LEN = 0.004

# Flip-down aerator
AERATOR_R = 0.015
AERATOR_H = 0.006
AERATOR_PIVOT_X = REACH_X + AERATOR_R  # 0.159
AERATOR_PIVOT_Z = SPRAY_BOTTOM_LOCAL   # 0.085
AERATOR_FLIP_UPPER = math.radians(70.0)

# Hinge boss on spray head (carries the aerator pivot pin)
HINGE_R = 0.004
HINGE_LEN = 0.024

# Pivot pin on aerator
PIVOT_PIN_R = 0.003
PIVOT_PIN_LEN = 0.020


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


def _spray_head_shape() -> cq.Workplane:
    """Ribbed cylindrical spray head body with circumferential ribs."""
    body = cq.Workplane("XY").circle(SPRAY_R).extrude(SPRAY_LEN)
    rib_span = SPRAY_LEN - 0.010
    rib_spacing = rib_span / max(NUM_RIBS - 1, 1)
    for i in range(NUM_RIBS):
        z = 0.005 + i * rib_spacing
        ring = (
            cq.Workplane("XY")
            .workplane(offset=z)
            .circle(SPRAY_R + RIB_PROUD)
            .circle(SPRAY_R - 0.0005)
            .extrude(RIB_WIDTH)
        )
        body = body.union(ring)
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="high_arc_gooseneck_faucet")

    gloss_black = model.material("veined_gloss_black", rgba=(0.075, 0.075, 0.085, 1.0))
    matte_black = model.material("matte_black", rgba=(0.035, 0.035, 0.038, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    outlet_dark = model.material("outlet_dark", rgba=(0.10, 0.10, 0.10, 1.0))
    spray_grey = model.material("spray_grey", rgba=(0.22, 0.22, 0.24, 1.0))

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
    column.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_LEN),
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z - COLLAR_LEN / 2.0)),
        material=chrome,
        name="swivel_collar",
    )

    # --------------------------------------------------------------- gooseneck
    spout = model.part("gooseneck_spout")
    spout.visual(
        mesh_from_cadquery(_gooseneck_shape(), "gooseneck_tube"),
        material=gloss_black,
        name="gooseneck_tube",
    )
    # Ribbed spray head at the end of the drop leg
    spout.visual(
        mesh_from_cadquery(_spray_head_shape(), "spray_head_body"),
        origin=Origin(xyz=(REACH_X, 0.0, SPRAY_BOTTOM_LOCAL)),
        material=gloss_black,
        name="spray_head_body",
    )
    # Chrome transition ring at the top of the spray head
    spout.visual(
        Cylinder(radius=SPRAY_COLLAR_R, length=SPRAY_COLLAR_LEN),
        origin=Origin(xyz=(REACH_X, 0.0, DROP_END - SPRAY_COLLAR_LEN / 2.0)),
        material=chrome,
        name="spray_head_collar",
    )
    # Hinge boss at the +X edge of the spray head bottom (carries aerator)
    spout.visual(
        Cylinder(radius=HINGE_R, length=HINGE_LEN),
        origin=Origin(
            xyz=(AERATOR_PIVOT_X, 0.0, AERATOR_PIVOT_Z + HINGE_R),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=chrome,
        name="aerator_hinge_boss",
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

    # ---------------------------------------------------------- flip-down aerator
    aerator = model.part("flip_aerator")
    # Main aerator disc (horizontal thin cylinder, offset from pivot)
    aerator.visual(
        Cylinder(radius=AERATOR_R, length=AERATOR_H),
        origin=Origin(xyz=(-AERATOR_R, 0.0, -AERATOR_H / 2.0)),
        material=spray_grey,
        name="aerator_disc",
    )
    # Outlet screen on the bottom face of the aerator
    aerator.visual(
        Cylinder(radius=AERATOR_R * 0.75, length=0.001),
        origin=Origin(xyz=(-AERATOR_R, 0.0, -AERATOR_H - 0.0004)),
        material=outlet_dark,
        name="aerator_screen",
    )
    # Pivot pin that inserts into the hinge boss
    aerator.visual(
        Cylinder(radius=PIVOT_PIN_R, length=PIVOT_PIN_LEN),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="pivot_pin",
    )

    model.articulation(
        "aerator_flip",
        ArticulationType.REVOLUTE,
        parent=spout,
        child=aerator,
        origin=Origin(xyz=(AERATOR_PIVOT_X, 0.0, AERATOR_PIVOT_Z)),
        # axis (0,-1,0): positive q flips the -X edge of the disc downward
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0, lower=0.0, upper=AERATOR_FLIP_UPPER
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
    aerator_flip = object_model.get_articulation("aerator_flip")
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
    ctx.allow_overlap(
        aerator,
        spout,
        elem_a="pivot_pin",
        elem_b="aerator_hinge_boss",
        reason="Pivot pin intentionally inserts into the hinge boss for the flip-down aerator mechanism.",
    )

    # ----- grounding, scale, proportions
    col_aabb = ctx.part_world_aabb(column)
    ctx.check(
        "tap grounded on the deck plane",
        col_aabb is not None and abs(col_aabb[0][2]) <= 0.002,
        details=f"column aabb={col_aabb}",
    )
    disc = ctx.part_element_world_aabb(column, elem="base_disc")
    ctx.check(
        "single chrome base disc sits on the deck (wide, thin)",
        disc is not None
        and 0.080 <= (disc[1][0] - disc[0][0]) <= 0.090
        and (disc[1][2] - disc[0][2]) <= 0.010,
        details=f"base disc aabb={disc}",
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
        spout_aabb is not None and 0.372 <= spout_aabb[1][2] <= 0.392,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "gooseneck arcs forward over the sink (+X reach)",
        spout_aabb is not None and spout_aabb[1][0] >= 0.130,
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

    # ----- chrome collar ring between column and spout
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

    # ----- ribbed spray head at the spout end
    spray = ctx.part_element_world_aabb(spout, elem="spray_head_body")
    tube = ctx.part_element_world_aabb(spout, elem="gooseneck_tube")
    ctx.check(
        "ribbed spray head at the front end of the gooseneck, below the apex",
        spray is not None
        and tube is not None
        and spray[1][0] >= REACH_X + SPRAY_R - 0.005
        and spray[1][2] < tube[1][2] - 0.050
        and spray[0][2] < tube[1][2] - 0.100,
        details=f"spray={spray}, tube={tube}",
    )
    ctx.check(
        "spray head has shallow ribbing (X-width exceeds base cylinder diameter)",
        spray is not None
        and (spray[1][0] - spray[0][0]) > 2.0 * SPRAY_R + 0.0005,
        details=f"spray aabb={spray}, base_dia={2*SPRAY_R}",
    )
    ctx.check(
        "spray head body length approximately 0.042 m",
        spray is not None
        and 0.038 <= (spray[1][2] - spray[0][2]) <= 0.046,
        details=f"spray aabb={spray}",
    )

    # ----- flip-down aerator
    aerator_aabb = ctx.part_world_aabb(aerator)
    aerator_disc = ctx.part_element_world_aabb(aerator, elem="aerator_disc")
    ctx.check(
        "flip-down aerator disc exists below the spray head",
        aerator_disc is not None
        and spray is not None
        and aerator_disc[1][2] <= spray[0][2] + 0.002,
        details=f"aerator_disc={aerator_disc}, spray={spray}",
    )
    # Pivot pin inserts into hinge boss (contact/proximity)
    ctx.expect_overlap(
        aerator,
        spout,
        axes="z",
        elem_a="pivot_pin",
        elem_b="aerator_hinge_boss",
        min_overlap=0.002,
        name="aerator pivot pin overlaps the hinge boss along Z",
    )

    # ----- aerator flip joint: type, axis, limits
    ctx.check(
        "aerator_flip is revolute 0..70 deg about the lateral Y axis",
        aerator_flip.articulation_type == ArticulationType.REVOLUTE
        and tuple(aerator_flip.axis) == (0.0, -1.0, 0.0)
        and aerator_flip.motion_limits is not None
        and abs(aerator_flip.motion_limits.lower) < 1e-6
        and abs(aerator_flip.motion_limits.upper - AERATOR_FLIP_UPPER) < 1e-6,
    )

    # ----- aerator pose: flip-down tilts the disc downward
    rest_aerator = ctx.part_world_aabb(aerator)
    with ctx.pose({aerator_flip: AERATOR_FLIP_UPPER}):
        flipped_aerator = ctx.part_world_aabb(aerator)
    ctx.check(
        "aerator flips downward at max angle (z-min decreases)",
        rest_aerator is not None
        and flipped_aerator is not None
        and flipped_aerator[0][2] < rest_aerator[0][2] - 0.003,
        details=f"rest={rest_aerator}, flipped={flipped_aerator}",
    )
    with ctx.pose({aerator_flip: AERATOR_FLIP_UPPER}):
        flipped_disc = ctx.part_element_world_aabb(aerator, elem="aerator_disc")
    ctx.check(
        "aerator disc tilts forward at max flip (X reach increases)",
        flipped_disc is not None
        and aerator_disc is not None
        and flipped_disc[1][0] > aerator_disc[1][0] + 0.002,
        details=f"rest_disc={aerator_disc}, flipped_disc={flipped_disc}",
    )

    # ----- spout swivel joint
    ctx.check(
        "spout swivel is revolute -110..+110 deg about the vertical column axis",
        swivel.articulation_type == ArticulationType.REVOLUTE
        and tuple(swivel.axis) == (0.0, 0.0, 1.0)
        and swivel.motion_limits is not None
        and abs(swivel.motion_limits.lower + SWIVEL_LIMIT) < 1e-6
        and abs(swivel.motion_limits.upper - SWIVEL_LIMIT) < 1e-6,
    )

    # ----- swivel pose: spout outlet sweeps sideways
    rest_spray = ctx.part_element_world_aabb(spout, elem="spray_head_body")
    with ctx.pose({swivel: 1.0}):
        sw_spray = ctx.part_element_world_aabb(spout, elem="spray_head_body")
    ctx.check(
        "spout swivel carries the spray head sideways about the vertical axis",
        rest_spray is not None
        and sw_spray is not None
        and abs(0.5 * (rest_spray[0][1] + rest_spray[1][1])) < 0.002
        and abs(0.5 * (sw_spray[0][1] + sw_spray[1][1])) > 0.06,
        details=f"rest={rest_spray}, swiveled={sw_spray}",
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

    # ----- lever joint plan
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

    # ----- lever pose checks
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
    with ctx.pose({pivot_1: -math.pi / 2.0}):
        tilted_1 = ctx.part_world_aabb(lever_1)
    ctx.check(
        "lever 1 tilts from vertical to horizontal toward the user at q=-90 deg",
        rest_1 is not None
        and tilted_1 is not None
        and tilted_1[1][0] > rest_1[1][0] + 0.10
        and tilted_1[1][2] < CROSS_Z + 0.03,
        details=f"rest={rest_1}, tilted={tilted_1}",
    )

    return ctx.report()


object_model = build_object_model()
