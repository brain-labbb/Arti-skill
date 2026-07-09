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
# High-arc gooseneck faucet variant 02 (fork of gloss-black monobloc mixer tap).
#
# Structural changes vs parent:
#   - Spout is lower (~0.35 m apex) and wider, with a flattened oval
#     (elliptical) tube cross-section (wider side-to-side, narrower front-back).
#   - A pull-out spray head slides from the spout tip on a short prismatic
#     "hose" joint (0 .. 0.08 m downward travel).
#   - Shallow ribbing (6 ribs) runs along the spray head body.
#   - A thin dark seam ring marks the swivel collar joint.
#
# Layout (world frame, deck at z = 0, +X toward user/sink):
#   - Chrome base disc on the deck; gloss-black cylindrical column (0.04 m dia)
#     rises on the Z axis.
#   - Horizontal cross-cylinder (0.045 m dia, 0.18 m end-to-end along Y)
#     passes through the column at z = 0.085, forming two valve bodies with
#     flat matte-black end caps.
#   - Two slim pin levers (0.10 m, 0.012 m dia) point up from the valve tops.
#   - Chrome collar ring at z = 0.140 with a thin seam line.
#   - Swan-neck gooseneck arcs up and over (apex ~0.35 m), ending in a
#     downward drop leg at x = 0.140.
#   - Pull-out spray head docks at the drop leg tip.
# ---------------------------------------------------------------------------

# Base + column (unchanged from parent)
BASE_DISC_R = 0.042
BASE_DISC_H = 0.008
COLUMN_R = 0.020
COLUMN_TOP = 0.132

# Cross valve cylinder (unchanged)
CROSS_Z = 0.085
CROSS_R = 0.0225
CROSS_TUBE_LEN = 0.170
CAP_LEN = 0.005
CAP_R = 0.0235
CAP_Y = CROSS_TUBE_LEN / 2.0 + CAP_LEN / 2.0

# Pin levers (unchanged)
LEVER_Y = 0.058
BOSS_R = 0.010
BOSS_LEN = 0.016
BOSS_Z = 0.026
PIN_R = 0.006
PIN_LEN = 0.100
PIN_Z0 = 0.032

# Swivel collar
COLLAR_R = 0.0215
COLLAR_LEN = 0.010
SWIVEL_Z = 0.140

# VARIANT 02 -- seam at collar
SEAM_R = COLLAR_R + 0.0008
SEAM_LEN = 0.001
SEAM_Z = SWIVEL_Z - 0.002  # just below the collar top edge

# VARIANT 02 -- lower, wider gooseneck with flattened oval tube
TUBE_RX = 0.012  # semi-axis in arc plane (front-back, narrower)
TUBE_RY = 0.020  # semi-axis perpendicular (side-to-side, wider)
ARC_R = 0.070
RISER_TOP = 0.128
REACH_X = 2.0 * ARC_R  # 0.140 m horizontal reach
DROP_END = 0.060  # spout-local z of drop-leg tip (world 0.200)

# Apex estimate: at the arc top, TUBE_RX points vertically
APEX_WORLD = SWIVEL_Z + RISER_TOP + ARC_R + TUBE_RX  # ~0.350

# Spray-head chrome docking ring
SLEEVE_R = TUBE_RY + 0.003  # 0.023
SLEEVE_LEN = 0.006

# Spray-head outlet
AERATOR_R = 0.014
AERATOR_LEN = 0.003

# VARIANT 02 -- pull-out spray head
SPRAY_R = 0.018
SPRAY_LEN = 0.045
SPRAY_PULL = 0.080
SPRAY_BODY_TOP = 0.005  # body top inserts 5 mm into tube for connectivity
SPRAY_BODY_CENTER_Z = SPRAY_BODY_TOP - SPRAY_LEN / 2.0

# Shallow ribs on the spray head
RIB_COUNT = 6
RIB_WIDTH = 0.004  # tangential
RIB_DEPTH = 0.003  # radial total (half inside body, half proud)
RIB_LEN = 0.035   # along the body axis

SWIVEL_LIMIT = math.radians(110.0)


def _gooseneck_shape() -> cq.Workplane:
    """Swan-neck tube with flattened oval (elliptical) cross-section."""
    path = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0)
        .lineTo(0.0, RISER_TOP)
        .threePointArc((ARC_R, RISER_TOP + ARC_R), (REACH_X, RISER_TOP))
        .lineTo(REACH_X, DROP_END)
    )
    return cq.Workplane("XY").ellipse(TUBE_RX, TUBE_RY).sweep(path)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="high_arc_gooseneck_faucet_v02")

    gloss_black = model.material("veined_gloss_black", rgba=(0.075, 0.075, 0.085, 1.0))
    matte_black = model.material("matte_black", rgba=(0.035, 0.035, 0.038, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    outlet_dark = model.material("outlet_dark", rgba=(0.10, 0.10, 0.10, 1.0))
    rib_mat = model.material("rib_charcoal", rgba=(0.055, 0.055, 0.062, 1.0))

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
    # Chrome collar
    column.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_LEN),
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z - COLLAR_LEN / 2.0)),
        material=chrome,
        name="swivel_collar",
    )
    # VARIANT 02: thin seam ring at the collar
    column.visual(
        Cylinder(radius=SEAM_R, length=SEAM_LEN),
        origin=Origin(xyz=(0.0, 0.0, SEAM_Z)),
        material=matte_black,
        name="collar_seam",
    )

    # --------------------------------------------------------------- gooseneck
    spout = model.part("gooseneck_spout")
    spout.visual(
        mesh_from_cadquery(_gooseneck_shape(), "gooseneck_tube"),
        material=gloss_black,
        name="gooseneck_tube",
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

    # ----------------------------------------------------- spray head (V02)
    spray = model.part("spray_head")

    # Main body cylinder, top inserts into spout tube for connectivity
    spray.visual(
        Cylinder(radius=SPRAY_R, length=SPRAY_LEN),
        origin=Origin(xyz=(0.0, 0.0, SPRAY_BODY_CENTER_Z)),
        material=gloss_black,
        name="spray_body",
    )

    # Shallow ribbing: 6 ribs equally spaced, centered at body surface
    rib_center_z = SPRAY_BODY_CENTER_Z
    for i in range(RIB_COUNT):
        theta = i * (2.0 * math.pi / RIB_COUNT)
        rx = SPRAY_R * math.cos(theta)
        ry = SPRAY_R * math.sin(theta)
        spray.visual(
            Box((RIB_DEPTH, RIB_WIDTH, RIB_LEN)),
            origin=Origin(xyz=(rx, ry, rib_center_z), rpy=(0.0, 0.0, theta)),
            material=rib_mat,
            name=f"spray_rib_{i}",
        )

    # Chrome docking ring at the spout junction
    spray.visual(
        Cylinder(radius=SLEEVE_R, length=SLEEVE_LEN),
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
        material=chrome,
        name="spray_tip",
    )

    # Dark outlet disc at the bottom, overlapping 1 mm into body for connectivity
    outlet_z = (SPRAY_BODY_TOP - SPRAY_LEN) - AERATOR_LEN / 2.0 + 0.001
    spray.visual(
        Cylinder(radius=AERATOR_R, length=AERATOR_LEN),
        origin=Origin(xyz=(0.0, 0.0, outlet_z)),
        material=outlet_dark,
        name="spray_outlet",
    )

    # Prismatic pull-out joint: spray head slides downward from spout tip
    model.articulation(
        "spray_pull",
        ArticulationType.PRISMATIC,
        parent=spout,
        child=spray,
        origin=Origin(xyz=(REACH_X, 0.0, DROP_END)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=10.0, velocity=0.5, lower=0.0, upper=SPRAY_PULL
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
    spray = object_model.get_part("spray_head")
    lever_0 = object_model.get_part("pin_lever_0")
    lever_1 = object_model.get_part("pin_lever_1")

    swivel = object_model.get_articulation("spout_swivel")
    pull = object_model.get_articulation("spray_pull")
    pivot_0 = object_model.get_articulation("lever_pivot_0")
    pivot_1 = object_model.get_articulation("lever_pivot_1")

    # ----- intentional overlap allowances
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
    # Spray head docks into spout tube end
    ctx.allow_overlap(
        spray, spout,
        elem_a="spray_body", elem_b="gooseneck_tube",
        reason="Spray head body inserts into the spout tube end when docked.",
    )
    ctx.allow_overlap(
        spray, spout,
        elem_a="spray_tip", elem_b="gooseneck_tube",
        reason="Chrome docking ring seats at the spout tube end.",
    )

    # ----- grounding and scale
    col_aabb = ctx.part_world_aabb(column)
    ctx.check(
        "tap grounded on deck plane",
        col_aabb is not None and abs(col_aabb[0][2]) <= 0.002,
        details=f"column aabb={col_aabb}",
    )
    disc = ctx.part_element_world_aabb(column, elem="base_disc")
    ctx.check(
        "chrome base disc sits on the deck",
        disc is not None
        and 0.080 <= (disc[1][0] - disc[0][0]) <= 0.090
        and (disc[1][2] - disc[0][2]) <= 0.010,
        details=f"base disc aabb={disc}",
    )

    # ----- VARIANT 02: lower apex, wider reach
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "gooseneck apex is lower than parent (~0.34-0.36 m)",
        spout_aabb is not None and 0.330 <= spout_aabb[1][2] <= 0.370,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "gooseneck arcs forward over the sink (+X reach)",
        spout_aabb is not None and spout_aabb[1][0] >= 0.120,
        details=f"spout aabb={spout_aabb}",
    )

    # ----- VARIANT 02: flattened oval tube wider side-to-side
    tube = ctx.part_element_world_aabb(spout, elem="gooseneck_tube")
    ctx.check(
        "flattened oval tube: Y extent >= 0.036 (wider side-to-side)",
        tube is not None and (tube[1][1] - tube[0][1]) >= 0.036,
        details=f"tube aabb={tube}",
    )

    # ----- cross valve cylinder and end caps
    cross = ctx.part_element_world_aabb(column, elem="cross_tube")
    cap_0 = ctx.part_element_world_aabb(column, elem="valve_end_cap_0")
    cap_1 = ctx.part_element_world_aabb(column, elem="valve_end_cap_1")
    ctx.check(
        "cross-cylinder ~0.045 m diameter at mid-column",
        cross is not None
        and 0.043 <= (cross[1][2] - cross[0][2]) <= 0.047
        and 0.06 <= 0.5 * (cross[0][2] + cross[1][2]) <= 0.11,
        details=f"cross aabb={cross}",
    )
    ctx.check(
        "valve assembly ~0.18 m end-to-end",
        cap_0 is not None
        and cap_1 is not None
        and 0.178 <= (cap_0[1][1] - cap_1[0][1]) <= 0.182,
        details=f"cap_0={cap_0}, cap_1={cap_1}",
    )

    # ----- collar, seam, spout seating
    collar = ctx.part_element_world_aabb(column, elem="swivel_collar")
    seam = ctx.part_element_world_aabb(column, elem="collar_seam")
    ctx.check(
        "chrome collar above cross and below spout",
        collar is not None and cross is not None
        and collar[0][2] >= cross[1][2]
        and spout_aabb is not None
        and collar[1][2] <= spout_aabb[0][2] + 0.005,
        details=f"collar={collar}, cross_top={cross[1][2] if cross else None}",
    )
    # VARIANT 02: seam ring at the collar
    ctx.check(
        "thin seam ring at the swivel collar",
        seam is not None and collar is not None
        and seam[0][2] >= collar[0][2] - 0.001
        and seam[1][2] <= collar[1][2] + 0.001
        and (seam[1][2] - seam[0][2]) <= 0.002,
        details=f"seam={seam}, collar={collar}",
    )
    ctx.expect_contact(
        spout, column,
        elem_a="gooseneck_tube", elem_b="swivel_collar",
        contact_tol=0.002,
        name="gooseneck riser seats on the chrome collar",
    )

    # ----- VARIANT 02: spray head with ribbing
    spray_body = ctx.part_element_world_aabb(spray, elem="spray_body")
    ctx.check(
        "spray head body exists at the spout drop-leg tip",
        spray_body is not None
        and spray_body[0][2] < (SWIVEL_Z + DROP_END + 0.010)
        and spray_body[1][2] > SWIVEL_Z + DROP_END - SPRAY_LEN - 0.010,
        details=f"spray_body={spray_body}",
    )
    rib_names = [f"spray_rib_{i}" for i in range(RIB_COUNT)]
    rib_aabbs = [ctx.part_element_world_aabb(spray, elem=n) for n in rib_names]
    ctx.check(
        f"spray head has {RIB_COUNT} shallow ribs",
        all(r is not None for r in rib_aabbs),
        details=f"found={sum(1 for r in rib_aabbs if r is not None)}/{RIB_COUNT}",
    )
    # Ribs should be wider than the bare body (ribs protrude)
    spray_tip = ctx.part_element_world_aabb(spray, elem="spray_tip")
    spray_outlet = ctx.part_element_world_aabb(spray, elem="spray_outlet")
    ctx.check(
        "spray head has chrome tip ring and dark outlet",
        spray_tip is not None and spray_outlet is not None
        and spray_outlet[0][2] < spray_body[0][2],
        details=f"tip={spray_tip}, outlet={spray_outlet}",
    )

    # ----- VARIANT 02: prismatic pull-out joint
    ctx.check(
        "spray_pull is PRISMATIC along -Z, range 0..0.08 m",
        pull.articulation_type == ArticulationType.PRISMATIC
        and tuple(pull.axis) == (0.0, 0.0, -1.0)
        and pull.motion_limits is not None
        and abs(pull.motion_limits.lower) < 1e-6
        and abs(pull.motion_limits.upper - SPRAY_PULL) < 1e-3,
    )

    # Spray head extends downward when pulled
    rest_spray = ctx.part_world_aabb(spray)
    with ctx.pose({pull: SPRAY_PULL}):
        pulled_spray = ctx.part_world_aabb(spray)
    ctx.check(
        "spray head extends downward when pulled out",
        rest_spray is not None and pulled_spray is not None
        and pulled_spray[0][2] < rest_spray[0][2] - 0.060,
        details=f"rest={rest_spray}, pulled={pulled_spray}",
    )
    # At full pull, spray head still above the deck
    ctx.check(
        "spray head stays above the deck at full extension",
        pulled_spray is not None and pulled_spray[0][2] > 0.010,
        details=f"pulled bottom z={pulled_spray[0][2] if pulled_spray else None}",
    )

    # ----- spout swivel joint
    ctx.check(
        "spout swivel is REVOLUTE ±110° about vertical",
        swivel.articulation_type == ArticulationType.REVOLUTE
        and tuple(swivel.axis) == (0.0, 0.0, 1.0)
        and swivel.motion_limits is not None
        and abs(swivel.motion_limits.lower + SWIVEL_LIMIT) < 1e-6
        and abs(swivel.motion_limits.upper - SWIVEL_LIMIT) < 1e-6,
    )

    rest_tip = ctx.part_element_world_aabb(spray, elem="spray_tip")
    with ctx.pose({swivel: 1.0}):
        sw_tip = ctx.part_element_world_aabb(spray, elem="spray_tip")
    ctx.check(
        "spout swivel carries spray head sideways",
        rest_tip is not None and sw_tip is not None
        and abs(0.5 * (rest_tip[0][1] + rest_tip[1][1])) < 0.01
        and abs(0.5 * (sw_tip[0][1] + sw_tip[1][1])) > 0.05,
        details=f"rest={rest_tip}, swiveled={sw_tip}",
    )

    # ----- pin levers: geometry and seating
    for lever, name in ((lever_0, "pin_lever_0"), (lever_1, "pin_lever_1")):
        pin = ctx.part_element_world_aabb(lever, elem="lever_pin")
        ctx.check(
            f"{name} pin is slim and vertical at rest",
            pin is not None
            and 0.010 <= (pin[1][0] - pin[0][0]) <= 0.014
            and 0.098 <= (pin[1][2] - pin[0][2]) <= 0.102,
            details=f"pin aabb={pin}",
        )
        ctx.expect_overlap(
            lever, column, axes="z",
            elem_a="lever_boss", elem_b="cross_tube",
            min_overlap=0.003,
            name=f"{name} boss seats into valve cylinder",
        )

    # Lever pivot joints
    for pivot, name in ((pivot_0, "lever_pivot_0"), (pivot_1, "lever_pivot_1")):
        ctx.check(
            f"{name} is REVOLUTE -90..0° about valve Y axis",
            pivot.articulation_type == ArticulationType.REVOLUTE
            and tuple(pivot.axis) == (0.0, -1.0, 0.0)
            and pivot.motion_limits is not None
            and abs(pivot.motion_limits.lower + math.pi / 2.0) < 1e-6
            and abs(pivot.motion_limits.upper) < 1e-6
            and pivot.mimic is None,
        )

    # Lever pose: tilt toward user
    rest_0 = ctx.part_world_aabb(lever_0)
    with ctx.pose({pivot_0: -math.pi / 2.0}):
        tilted_0 = ctx.part_world_aabb(lever_0)
    ctx.check(
        "lever 0 tilts toward user at q = -90°",
        rest_0 is not None and tilted_0 is not None
        and tilted_0[1][0] > rest_0[1][0] + 0.08
        and tilted_0[1][2] < CROSS_Z + 0.03,
        details=f"rest={rest_0}, tilted={tilted_0}",
    )

    return ctx.report()


object_model = build_object_model()
