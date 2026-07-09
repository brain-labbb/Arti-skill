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
# High-arc gooseneck faucet variant (Variant 30).
#
# Derived from the gloss-black monobloc kitchen mixer tap. Changes from parent:
# - Rear support strut from the base disc rear edge diagonally up behind the
#   column, bracing toward the high-arc region.
# - Pull-out spray head on a short prismatic hose joint at the gooseneck drop
#   leg tip. The spray head slides downward (out of the spout) on -Z axis.
# - Shallow ribbing around the spray head body for grip texture.
#
# Layout (world frame, deck plane at z = 0):
# - +X is front (the direction the gooseneck reaches over the sink), +Z is up.
# - Chrome base disc on deck; gloss-black cylindrical column (0.04 m dia) on Z.
# - Horizontal cross-cylinder (0.045 m dia, 0.18 m end-to-end) at z = 0.085,
#   two valve bodies with flat black end caps.
# - Two slim pin levers (0.012 m dia, 0.10 m long), revolute about valve Y axis.
# - Thin chrome collar ring separates column from swan-neck gooseneck spout,
#   which swivels about the vertical column axis (-110..+110 deg).
# - Rear support strut: angled bar from rear base up behind column toward arc.
# - Spray head at gooseneck tip: prismatic slide on -Z axis, 0..0.06 m travel.
# ---------------------------------------------------------------------------

# Base + column
BASE_DISC_R = 0.042
BASE_DISC_H = 0.008
COLUMN_R = 0.020  # 0.04 m diameter per prompt
COLUMN_TOP = 0.132  # shaft reaches 2 mm into the collar for connectivity

# Cross valve cylinder
CROSS_Z = 0.085
CROSS_R = 0.0225  # 0.045 m diameter per prompt
CROSS_TUBE_LEN = 0.170
CAP_LEN = 0.005  # flat end caps; total end-to-end = 0.170 + 2*0.005 = 0.180 m
CAP_R = 0.0235
CAP_Y = CROSS_TUBE_LEN / 2.0 + CAP_LEN / 2.0  # 0.0875

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

SWIVEL_LIMIT = math.radians(110.0)

# Rear support strut geometry (nearly vertical brace behind the column)
STRUT_BOT_X = -0.038  # rear edge of base disc
STRUT_BOT_Z = 0.008  # just above deck
STRUT_W = 0.012  # width (Y direction)
STRUT_T = 0.006  # thickness (radial direction)

# Path displacement in XZ plane (sweep already encodes the angle; no extra rotation)
_strut_dx = 0.010  # only slight forward lean
_strut_dz = 0.252  # reaches well up toward the arc region
STRUT_LEN = math.sqrt(_strut_dx**2 + _strut_dz**2)
STRUT_TOP_X = STRUT_BOT_X + _strut_dx  # -0.028
STRUT_TOP_Z = STRUT_BOT_Z + _strut_dz  # 0.260

# Spray head geometry (local frame: origin at top, body extends -Z)
SPRAY_BODY_R = 0.014
SPRAY_BODY_LEN = 0.055
SPRAY_COLLAR_R = 0.0155
SPRAY_COLLAR_LEN = 0.007
SPRAY_OUTLET_R = 0.010
SPRAY_OUTLET_LEN = 0.004
SPRAY_RIB_COUNT = 8
SPRAY_RIB_W = 0.003  # tangential width
SPRAY_RIB_H = 0.003  # radial proud height
SPRAY_RIB_LEN = 0.038  # axial length along body
SPRAY_RIB_R = SPRAY_BODY_R + 0.5 * SPRAY_RIB_H  # center radial position

# Prismatic hose slide limits
SPRAY_SLIDE_TRAVEL = 0.060  # 60 mm pull-out travel


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


def _rear_strut_shape() -> cq.Workplane:
    """Rear support strut: a thin rectangular bar angled from base to high arc."""
    # Build in the XZ plane as a swept rectangle along the strut axis
    path = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0)
        .lineTo(_strut_dx, _strut_dz)
    )
    return (
        cq.Workplane("XY")
        .rect(STRUT_T, STRUT_W)
        .sweep(path)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="high_arc_gooseneck_faucet")

    gloss_black = model.material("veined_gloss_black", rgba=(0.075, 0.075, 0.085, 1.0))
    matte_black = model.material("matte_black", rgba=(0.035, 0.035, 0.038, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    outlet_dark = model.material("outlet_dark", rgba=(0.10, 0.10, 0.10, 1.0))

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
    # Rear support strut: nearly vertical brace from base to high arc region.
    # The CadQuery sweep already encodes the path angle; place with no rotation.
    column.visual(
        mesh_from_cadquery(_rear_strut_shape(), "rear_strut"),
        origin=Origin(xyz=(STRUT_BOT_X, 0.0, STRUT_BOT_Z)),
        material=gloss_black,
        name="rear_strut",
    )

    # --------------------------------------------------------------- gooseneck
    spout = model.part("gooseneck_spout")
    spout.visual(
        mesh_from_cadquery(_gooseneck_shape(), "gooseneck_tube"),
        material=gloss_black,
        name="gooseneck_tube",
    )
    # Chrome ferrule ring at the tube end (visual transition to spray head).
    # Sits entirely above the tube end face so it doesn't overlap the pull-out head.
    spout.visual(
        Cylinder(radius=0.017, length=0.010),
        origin=Origin(xyz=(REACH_X, 0.0, DROP_END + 0.005)),
        material=chrome,
        name="tube_ferrule",
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

    # ------------------------------------------------------------- spray head
    spray = model.part("spray_head")
    # Articulation sits 4 mm below the tube end face so the pull-out head
    # does not overlap the gooseneck tube solid at rest.
    SPRAY_JOINT_Z = DROP_END - 0.004  # spout-local z of the spray origin

    # Hose stub: thin cylinder reaching from the spray head up to the ferrule
    # bottom face, providing physical contact with the spout for connectivity.
    spray.visual(
        Cylinder(radius=0.012, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=matte_black,
        name="hose_stub",
    )
    # Chrome collar at the top of the spray head (connection to hose/tube)
    spray.visual(
        Cylinder(radius=SPRAY_COLLAR_R, length=SPRAY_COLLAR_LEN),
        origin=Origin(xyz=(0.0, 0.0, -SPRAY_COLLAR_LEN / 2.0)),
        material=chrome,
        name="spray_collar",
    )
    # Main body cylinder (extends downward below the collar)
    spray.visual(
        Cylinder(radius=SPRAY_BODY_R, length=SPRAY_BODY_LEN),
        origin=Origin(xyz=(0.0, 0.0, -SPRAY_COLLAR_LEN - SPRAY_BODY_LEN / 2.0)),
        material=gloss_black,
        name="spray_body",
    )
    # Outlet at the bottom with dark aerator face
    spray.visual(
        Cylinder(radius=SPRAY_OUTLET_R, length=SPRAY_OUTLET_LEN),
        origin=Origin(xyz=(0.0, 0.0, -SPRAY_COLLAR_LEN - SPRAY_BODY_LEN - SPRAY_OUTLET_LEN / 2.0)),
        material=outlet_dark,
        name="spray_outlet",
    )
    # Shallow ribbing around the spray head body for grip texture
    rib_z_center = -SPRAY_COLLAR_LEN - SPRAY_BODY_LEN * 0.45
    for i in range(SPRAY_RIB_COUNT):
        angle = i * (2.0 * math.pi / SPRAY_RIB_COUNT)
        rib_x = SPRAY_RIB_R * math.cos(angle)
        rib_y = SPRAY_RIB_R * math.sin(angle)
        spray.visual(
            Box((SPRAY_RIB_H, SPRAY_RIB_W, SPRAY_RIB_LEN)),
            origin=Origin(
                xyz=(rib_x, rib_y, rib_z_center),
                rpy=(0.0, 0.0, angle),
            ),
            material=matte_black,
            name=f"spray_rib_{i}",
        )

    # Prismatic hose slide: spray head pulls out downward from the spout tip
    model.articulation(
        "spray_slide",
        ArticulationType.PRISMATIC,
        parent=spout,
        child=spray,
        origin=Origin(xyz=(REACH_X, 0.0, SPRAY_JOINT_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=15.0, velocity=0.3, lower=0.0, upper=SPRAY_SLIDE_TRAVEL
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

    column = object_model.get_part("body_column")
    spout = object_model.get_part("gooseneck_spout")
    spray = object_model.get_part("spray_head")
    lever_0 = object_model.get_part("pin_lever_0")
    lever_1 = object_model.get_part("pin_lever_1")

    swivel = object_model.get_articulation("spout_swivel")
    slide = object_model.get_articulation("spray_slide")
    pivot_0 = object_model.get_articulation("lever_pivot_0")
    pivot_1 = object_model.get_articulation("lever_pivot_1")

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

    # ----- grounding, scale, proportions
    col_aabb = ctx.part_world_aabb(column)
    ctx.check(
        "tap grounded on the deck plane",
        col_aabb is not None and abs(col_aabb[0][2]) <= 0.002,
        details=f"column aabb={col_aabb}",
    )
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "gooseneck apex near 0.38 m",
        spout_aabb is not None and 0.372 <= spout_aabb[1][2] <= 0.388,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "gooseneck arcs forward over the sink (+X reach)",
        spout_aabb is not None and spout_aabb[1][0] >= 0.140,
        details=f"spout aabb={spout_aabb}",
    )

    # ----- rear support strut
    strut = ctx.part_element_world_aabb(column, elem="rear_strut")
    ctx.check(
        "rear support strut extends from base toward high arc region",
        strut is not None
        and strut[0][2] <= 0.020  # bottom near deck
        and strut[1][2] >= 0.200  # top reaches high
        and strut[1][0] < -0.018  # entirely behind the column centerline
        and (strut[1][2] - strut[0][2]) >= 0.18,  # significant vertical span
        details=f"strut aabb={strut}",
    )

    # ----- spray head geometry and ribbing
    spray_body = ctx.part_element_world_aabb(spray, elem="spray_body")
    spray_outlet = ctx.part_element_world_aabb(spray, elem="spray_outlet")
    ctx.check(
        "spray head body is present below the gooseneck drop leg",
        spray_body is not None
        and spray_outlet is not None
        and spray_body[1][2] < spout_aabb[1][2]  # below spout apex
        and spray_outlet[0][2] < spray_body[0][2],  # outlet below body
        details=f"spray_body={spray_body}, spray_outlet={spray_outlet}",
    )
    # Check at least one rib exists
    rib_0 = ctx.part_element_world_aabb(spray, elem="spray_rib_0")
    rib_4 = ctx.part_element_world_aabb(spray, elem="spray_rib_4")
    ctx.check(
        "shallow ribbing present on the spray head (opposing ribs visible)",
        rib_0 is not None
        and rib_4 is not None
        and abs(rib_0[0][0] - rib_4[1][0]) > 0.015  # ribs on opposite sides
        and (rib_0[1][2] - rib_0[0][2]) >= 0.030,  # ribs have axial length
        details=f"rib_0={rib_0}, rib_4={rib_4}",
    )

    # ----- prismatic spray slide joint
    ctx.check(
        "spray_slide is prismatic with 0..0.06 m travel on -Z axis",
        slide.articulation_type == ArticulationType.PRISMATIC
        and slide.axis is not None
        and tuple(slide.axis) == (0.0, 0.0, -1.0)
        and slide.motion_limits is not None
        and abs(slide.motion_limits.lower) < 1e-6
        and abs(slide.motion_limits.upper - SPRAY_SLIDE_TRAVEL) < 1e-3,
    )

    # ----- spray slide pose: head moves downward when extended
    rest_spray = ctx.part_world_aabb(spray)
    rest_spray_center_z = 0.5 * (rest_spray[0][2] + rest_spray[1][2]) if rest_spray else None
    with ctx.pose({slide: SPRAY_SLIDE_TRAVEL}):
        ext_spray = ctx.part_world_aabb(spray)
    ext_spray_center_z = 0.5 * (ext_spray[0][2] + ext_spray[1][2]) if ext_spray else None
    ctx.check(
        "spray head slides downward (outward) at max prismatic extension",
        rest_spray_center_z is not None
        and ext_spray_center_z is not None
        and ext_spray_center_z < rest_spray_center_z - 0.040,
        details=f"rest_z={rest_spray_center_z}, extended_z={ext_spray_center_z}",
    )
    # At rest the spray head stays close to the gooseneck tip (small gap)
    ctx.expect_gap(
        spout,
        spray,
        axis="z",
        positive_elem="tube_ferrule",
        negative_elem="spray_collar",
        max_gap=0.008,
        name="spray head sits close below gooseneck tip at rest",
    )

    # ----- spout swivel (preserved from parent)
    ctx.check(
        "spout swivel is revolute -110..+110 deg about the vertical column axis",
        swivel.articulation_type == ArticulationType.REVOLUTE
        and tuple(swivel.axis) == (0.0, 0.0, 1.0)
        and swivel.motion_limits is not None
        and abs(swivel.motion_limits.lower + SWIVEL_LIMIT) < 1e-6
        and abs(swivel.motion_limits.upper - SWIVEL_LIMIT) < 1e-6,
    )
    # Swivel pose: outlet sweeps sideways (carries spray head too)
    rest_ferrule = ctx.part_element_world_aabb(spout, elem="tube_ferrule")
    with ctx.pose({swivel: 1.0}):
        sw_ferrule = ctx.part_element_world_aabb(spout, elem="tube_ferrule")
    ctx.check(
        "spout swivel carries the outlet sideways about the vertical axis",
        rest_ferrule is not None
        and sw_ferrule is not None
        and abs(0.5 * (rest_ferrule[0][1] + rest_ferrule[1][1])) < 1e-6
        and 0.5 * (sw_ferrule[0][1] + sw_ferrule[1][1]) > 0.08,
        details=f"rest={rest_ferrule}, swiveled={sw_ferrule}",
    )

    # ----- pin levers (preserved from parent)
    cross = ctx.part_element_world_aabb(column, elem="cross_tube")
    for lever, name in ((lever_0, "pin_lever_0"), (lever_1, "pin_lever_1")):
        pin = ctx.part_element_world_aabb(lever, elem="lever_pin")
        ctx.check(
            f"{name} pin is slim (0.012 m dia) and ~0.10 m long, vertical at rest",
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

    # Lever pivot joint checks
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

    # Lever pose check: tilts toward user at -90 deg
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
