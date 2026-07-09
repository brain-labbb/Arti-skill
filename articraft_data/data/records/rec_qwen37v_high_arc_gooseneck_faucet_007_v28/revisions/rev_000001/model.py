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
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# High-arc gooseneck faucet variant with commercial pre-rinse spring,
# flip-down outlet aerator, ribbed spray head, and hollow outlet bore.
#
# Layout (world frame, deck plane at z = 0):
# - +X is the front of the tap (the direction the gooseneck reaches over the
#   sink), +Z is up.
# - Chrome base disc, gloss-black cylindrical column, horizontal cross valve
#   cylinder with flat end caps, two pin levers, chrome collar ring — all
#   carried over from the parent monobloc mixer.
# - Above the collar a high swan-neck gooseneck arcs up (apex ~0.38 m) and
#   over, with an extended drop leg reaching down to a ribbed spray head.
# - A commercial pre-rinse style helical spring wraps around the drop leg.
# - The spray head is a hollow bore cylinder with shallow ribbing rings.
# - A flip-down aerator disc pivots at the spray head bottom on a horizontal
#   axis, exposing the distinct hollow outlet opening.
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

# Swivel collar (unchanged)
COLLAR_R = 0.0215
COLLAR_LEN = 0.010
SWIVEL_Z = 0.140

# Gooseneck tube (spout-local frame at the collar top, z = 0 → world SWIVEL_Z)
TUBE_R = 0.015
ARC_R = 0.072
RISER_TOP = 0.153
REACH_X = 2.0 * ARC_R  # 0.144 m
DROP_END = 0.040  # extended drop leg (spout-local z)

APEX_WORLD = SWIVEL_Z + RISER_TOP + ARC_R + TUBE_R  # 0.380 m

# Pre-rinse spring around the drop leg
SPRING_COIL_R = 0.0165  # wire center radius (inner wire edge touches tube)
SPRING_WIRE_R = 0.002  # 2 mm wire
SPRING_TURNS = 8
SPRING_Z_TOP = 0.135  # spout-local z of spring top
SPRING_Z_BOT = 0.065  # spout-local z of spring bottom

# Spray head (hollow bore with ribbing)
SPRAY_OUTER_R = 0.022
SPRAY_INNER_R = 0.012  # hollow bore
SPRAY_HEIGHT = 0.040
SPRAY_Z_CENTER = 0.040  # spout-local z center (spans 0.020..0.060)
RIB_TUBE_R = 0.0012  # shallow rib profile
RIB_Z_OFFSETS = (-0.012, -0.004, 0.004, 0.012)  # relative to spray center

# Outlet bore visual (dark sleeve inside the spray head, contacts inner wall)
OUTLET_R = 0.0125  # slightly larger than SPRAY_INNER_R for wall contact
OUTLET_LEN = 0.014
OUTLET_Z = 0.036  # spout-local z, above the aerator boss zone

# Flip-down aerator
AERATOR_DISC_R = 0.014
AERATOR_DISC_H = 0.004
AERATOR_BOSS_R = 0.013  # wider than bore to contact the spray head walls
AERATOR_BOSS_H = 0.006
AERATOR_STEM_R = 0.004
AERATOR_STEM_H = 0.008
AERATOR_PIVOT_Z = 0.020  # spout-local z (spray head bottom)

SWIVEL_LIMIT = math.radians(110.0)
AERATOR_LIMIT = math.radians(65.0)


def _gooseneck_shape() -> cq.Workplane:
    """Swan-neck tube: straight riser, high semicircular arc, extended drop leg."""
    path = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0)
        .lineTo(0.0, RISER_TOP)
        .threePointArc((ARC_R, RISER_TOP + ARC_R), (REACH_X, RISER_TOP))
        .lineTo(REACH_X, DROP_END)
    )
    return cq.Workplane("XY").circle(TUBE_R).sweep(path)


def _spring_shape() -> "MeshGeometry":
    """Helical coil spring around the drop leg."""
    n_pts = SPRING_TURNS * 24
    pts: list[tuple[float, float, float]] = []
    for i in range(n_pts + 1):
        t = i / n_pts
        angle = t * SPRING_TURNS * 2.0 * math.pi
        x = REACH_X + SPRING_COIL_R * math.cos(angle)
        y = SPRING_COIL_R * math.sin(angle)
        z = SPRING_Z_TOP - t * (SPRING_Z_TOP - SPRING_Z_BOT)
        pts.append((x, y, z))
    return tube_from_spline_points(
        pts,
        radius=SPRING_WIRE_R,
        samples_per_segment=4,
        radial_segments=8,
        closed_spline=False,
        cap_ends=True,
    )


def _spray_head_shape() -> cq.Workplane:
    """Hollow bore spray head body (centered on local origin)."""
    shape = (
        cq.Workplane("XY")
        .circle(SPRAY_OUTER_R)
        .circle(SPRAY_INNER_R)
        .extrude(SPRAY_HEIGHT)
    )
    return shape.translate((0.0, 0.0, -SPRAY_HEIGHT / 2.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="high_arc_gooseneck_faucet")

    gloss_black = model.material("veined_gloss_black", rgba=(0.075, 0.075, 0.085, 1.0))
    matte_black = model.material("matte_black", rgba=(0.035, 0.035, 0.038, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    outlet_dark = model.material("outlet_dark", rgba=(0.06, 0.06, 0.06, 1.0))
    spring_steel = model.material("spring_steel", rgba=(0.55, 0.56, 0.58, 1.0))

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
    # Main tube
    spout.visual(
        mesh_from_cadquery(_gooseneck_shape(), "gooseneck_tube"),
        material=gloss_black,
        name="gooseneck_tube",
    )
    # Pre-rinse spring coil around the drop leg
    spout.visual(
        mesh_from_geometry(_spring_shape(), "pre_rinse_spring"),
        material=spring_steel,
        name="pre_rinse_spring",
    )
    # Hollow bore spray head at the drop leg end
    spout.visual(
        mesh_from_cadquery(_spray_head_shape(), "spray_head_body"),
        origin=Origin(xyz=(REACH_X, 0.0, SPRAY_Z_CENTER)),
        material=gloss_black,
        name="spray_head_body",
    )
    # Shallow ribbing rings on the spray head
    for idx, dz in enumerate(RIB_Z_OFFSETS):
        rib_z = SPRAY_Z_CENTER + dz
        spout.visual(
            mesh_from_geometry(
                TorusGeometry(SPRAY_OUTER_R, RIB_TUBE_R), f"spray_rib_{idx}"
            ),
            origin=Origin(xyz=(REACH_X, 0.0, rib_z)),
            material=matte_black,
            name=f"spray_rib_{idx}",
        )
    # Dark outlet bore sleeve inside the spray head (hollow opening emphasis)
    spout.visual(
        Cylinder(radius=OUTLET_R, length=OUTLET_LEN),
        origin=Origin(xyz=(REACH_X, 0.0, OUTLET_Z)),
        material=outlet_dark,
        name="outlet_bore",
    )
    # Spout swivel articulation (column → spout)
    swivel = model.articulation(
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

    # ---------------------------------------------------------- flip aerator
    aerator = model.part("flip_aerator")
    # Hinge boss embeds into the spray head bottom
    aerator.visual(
        Cylinder(radius=AERATOR_BOSS_R, length=AERATOR_BOSS_H),
        origin=Origin(xyz=(0.0, 0.0, AERATOR_BOSS_H / 2.0)),
        material=chrome,
        name="aerator_boss",
    )
    # Connecting stem
    aerator.visual(
        Cylinder(radius=AERATOR_STEM_R, length=AERATOR_STEM_H),
        origin=Origin(xyz=(0.0, 0.0, -AERATOR_STEM_H / 2.0)),
        material=chrome,
        name="aerator_stem",
    )
    # Flip-down disc plate
    aerator.visual(
        Cylinder(radius=AERATOR_DISC_R, length=AERATOR_DISC_H),
        origin=Origin(xyz=(0.0, 0.0, -(AERATOR_STEM_H + AERATOR_DISC_H / 2.0))),
        material=chrome,
        name="aerator_disc",
    )
    # Aerator pivot articulation (spout → aerator)
    # At q=0, aerator hangs straight down (closed/flush). Positive q tilts
    # the disc outward (+X direction) to expose the hollow outlet.
    model.articulation(
        "aerator_pivot",
        ArticulationType.REVOLUTE,
        parent=spout,
        child=aerator,
        origin=Origin(xyz=(REACH_X, 0.0, AERATOR_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0, lower=0.0, upper=AERATOR_LIMIT
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
    aerator_joint = object_model.get_articulation("aerator_pivot")
    pivot_0 = object_model.get_articulation("lever_pivot_0")
    pivot_1 = object_model.get_articulation("lever_pivot_1")

    # --- Intentional seated overlaps ---
    # Lever bosses embed into the valve cylinder
    ctx.allow_overlap(
        lever_0, column,
        elem_a="lever_boss", elem_b="cross_tube",
        reason="Lever boss intentionally seats into the valve cylinder.",
    )
    ctx.allow_overlap(
        lever_1, column,
        elem_a="lever_boss", elem_b="cross_tube",
        reason="Lever boss intentionally seats into the valve cylinder.",
    )
    # Aerator boss embeds into the spray head bottom
    ctx.allow_overlap(
        aerator, spout,
        elem_a="aerator_boss", elem_b="spray_head_body",
        reason="Aerator hinge boss seats into the spray head bottom for pivot mount.",
    )

    # ----- grounding and scale -----
    col_aabb = ctx.part_world_aabb(column)
    ctx.check(
        "tap grounded on the deck plane",
        col_aabb is not None and abs(col_aabb[0][2]) <= 0.002,
        details=f"column aabb={col_aabb}",
    )

    # ----- gooseneck apex and reach -----
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "gooseneck apex near 0.38 m",
        spout_aabb is not None and 0.370 <= spout_aabb[1][2] <= 0.390,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "gooseneck arcs forward over the sink (+X reach)",
        spout_aabb is not None and spout_aabb[1][0] >= 0.130,
        details=f"spout aabb={spout_aabb}",
    )

    # ----- pre-rinse spring around the drop leg -----
    spring_aabb = ctx.part_element_world_aabb(spout, elem="pre_rinse_spring")
    spray_aabb = ctx.part_element_world_aabb(spout, elem="spray_head_body")
    ctx.check(
        "pre-rinse spring wraps around the drop leg above the spray head",
        spring_aabb is not None
        and spray_aabb is not None
        and spring_aabb[0][2] > SWIVEL_Z + 0.04
        and spring_aabb[0][2] >= spray_aabb[1][2] - 0.005
        and 0.030 <= (spring_aabb[1][0] - spring_aabb[0][0]) <= 0.050
        and (spring_aabb[1][2] - spring_aabb[0][2]) >= 0.050,
        details=f"spring={spring_aabb}, spray={spray_aabb}",
    )

    # ----- spray head with ribbing -----
    ctx.check(
        "spray head body is present below the drop leg",
        spray_aabb is not None
        and spray_aabb[0][2] < SWIVEL_Z + DROP_END + 0.005
        and 0.040 <= (spray_aabb[1][0] - spray_aabb[0][0]) <= 0.050,
        details=f"spray={spray_aabb}",
    )
    # Check that at least one rib is present
    rib_0_aabb = ctx.part_element_world_aabb(spout, elem="spray_rib_0")
    rib_3_aabb = ctx.part_element_world_aabb(spout, elem="spray_rib_3")
    ctx.check(
        "shallow ribbing rings on the spray head (at least 2 visible)",
        rib_0_aabb is not None
        and rib_3_aabb is not None
        and spray_aabb is not None
        and rib_0_aabb[0][2] >= spray_aabb[0][2] - 0.001
        and rib_3_aabb[1][2] <= spray_aabb[1][2] + 0.001
        and (rib_0_aabb[1][0] - rib_0_aabb[0][0]) >= 0.042,
        details=f"rib_0={rib_0_aabb}, rib_3={rib_3_aabb}",
    )

    # ----- hollow outlet opening -----
    outlet_aabb = ctx.part_element_world_aabb(spout, elem="outlet_bore")
    ctx.check(
        "distinct hollow outlet opening inside the spray head",
        outlet_aabb is not None
        and spray_aabb is not None
        and outlet_aabb[0][2] >= spray_aabb[0][2] - 0.001
        and outlet_aabb[1][2] <= spray_aabb[1][2] + 0.001
        and 0.022 <= (outlet_aabb[1][0] - outlet_aabb[0][0]) <= 0.028,
        details=f"outlet={outlet_aabb}, spray={spray_aabb}",
    )

    # ----- flip-down aerator joint -----
    ctx.check(
        "aerator pivot is revolute 0..65 deg about the horizontal Y axis",
        aerator_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(aerator_joint.axis) == (0.0, -1.0, 0.0)
        and aerator_joint.motion_limits is not None
        and abs(aerator_joint.motion_limits.lower) < 1e-6
        and abs(aerator_joint.motion_limits.upper - AERATOR_LIMIT) < 1e-6,
    )

    # Aerator pose test: flipping down moves the disc outward
    rest_disc = ctx.part_element_world_aabb(aerator, elem="aerator_disc")
    with ctx.pose({aerator_joint: AERATOR_LIMIT}):
        flipped_disc = ctx.part_element_world_aabb(aerator, elem="aerator_disc")
    ctx.check(
        "aerator flip-down moves the disc forward (+X) at max angle",
        rest_disc is not None
        and flipped_disc is not None
        and 0.5 * (flipped_disc[0][0] + flipped_disc[1][0])
        > 0.5 * (rest_disc[0][0] + rest_disc[1][0]) + 0.004,
        details=f"rest={rest_disc}, flipped={flipped_disc}",
    )

    # ----- spout swivel joint -----
    ctx.check(
        "spout swivel is revolute -110..+110 deg about the vertical column axis",
        swivel.articulation_type == ArticulationType.REVOLUTE
        and tuple(swivel.axis) == (0.0, 0.0, 1.0)
        and swivel.motion_limits is not None
        and abs(swivel.motion_limits.lower + SWIVEL_LIMIT) < 1e-6
        and abs(swivel.motion_limits.upper - SWIVEL_LIMIT) < 1e-6,
    )

    # Swivel pose: spout outlet sweeps sideways
    rest_spray = ctx.part_element_world_aabb(spout, elem="spray_head_body")
    with ctx.pose({swivel: 1.0}):
        sw_spray = ctx.part_element_world_aabb(spout, elem="spray_head_body")
    ctx.check(
        "spout swivel carries the spray head sideways about the vertical axis",
        rest_spray is not None
        and sw_spray is not None
        and abs(0.5 * (rest_spray[0][1] + rest_spray[1][1])) < 1e-6
        and abs(0.5 * (sw_spray[0][1] + sw_spray[1][1])) > 0.06,
        details=f"rest={rest_spray}, swiveled={sw_spray}",
    )

    # ----- pin levers -----
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
            lever, column,
            axes="z",
            elem_a="lever_boss", elem_b="cross_tube",
            min_overlap=0.003,
            name=f"{name} boss seats into the valve cylinder",
        )

    # Lever pose: tilt brings pin toward user (+X)
    rest_0 = ctx.part_world_aabb(lever_0)
    with ctx.pose({pivot_0: -math.pi / 2.0}):
        tilted_0 = ctx.part_world_aabb(lever_0)
    ctx.check(
        "lever 0 tilts from vertical to horizontal toward the user at q=-90 deg",
        rest_0 is not None
        and tilted_0 is not None
        and tilted_0[1][0] > rest_0[1][0] + 0.08
        and tilted_0[1][2] < CROSS_Z + 0.03,
        details=f"rest={rest_0}, tilted={tilted_0}",
    )

    return ctx.report()


object_model = build_object_model()
