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
# Variant 29: High-arc gooseneck faucet with faceted segmented neck,
# single side lever, cold/hot tick marks, and two pedestal mounting collars.
#
# Layout (world frame, deck plane at z = 0):
# - +X is the front (direction the gooseneck reaches over the sink), +Z is up.
# - Chrome base disc on the deck; gloss-black cylindrical column (0.04 m dia)
#   rises on the Z axis.
# - Two small chrome mounting collars ring the pedestal at z~0.030 and z~0.055.
# - A lever housing boss protrudes from the +Y side of the column at z=0.090;
#   a single side lever pivots there (revolute about X, -45..+45 deg).
# - Two thin raised tick marks on the +Y column face indicate cold (lower)
#   and hot (upper) lever positions.
# - A thin chrome collar ring separates the column from the faceted gooseneck
#   spout, which swivels about the vertical column axis (revolute ±110 deg).
# - The gooseneck is built from straight polyline segments (not a smooth arc),
#   producing visible angular bends that read as faceted segmented construction.
# - A chrome tip sleeve with downward outlet terminates the drop leg.
# - Apex height ~0.38 m.
# ---------------------------------------------------------------------------

# Base + column
BASE_DISC_R = 0.042
BASE_DISC_H = 0.008
COLUMN_R = 0.020
COLUMN_TOP = 0.132

# Mounting collars on the pedestal
COLLAR_MOUNT_R = 0.023
COLLAR_MOUNT_LEN = 0.006
COLLAR_MOUNT_Z0 = 0.030
COLLAR_MOUNT_Z1 = 0.055

# Lever housing (boss on the +Y side of the column)
LEVER_Z = 0.090
HOUSING_R = 0.014
HOUSING_LEN = 0.022
HOUSING_Y = COLUMN_R + HOUSING_LEN / 2.0  # center of housing along +Y

# Tick marks (thin raised rectangles on the +Y face of the column)
TICK_W = 0.012  # width along X
TICK_H = 0.003  # height along Z
TICK_D = 0.004  # protrusion along Y (from column surface)
TICK_COLD_Z = LEVER_Z - 0.018  # below lever pivot
TICK_HOT_Z = LEVER_Z + 0.018   # above lever pivot

# Side lever (extends along +Y in lever-local frame at q=0)
LEVER_PIVOT_Y = COLUMN_R + HOUSING_LEN  # outer face of housing
LEVER_BASE_R = 0.010
LEVER_BASE_LEN = 0.012
LEVER_HANDLE_R = 0.007
LEVER_HANDLE_LEN = 0.095
LEVER_TIP_R = 0.009
LEVER_TIP_LEN = 0.008

# Swivel collar + gooseneck
COLLAR_R = 0.0215
COLLAR_LEN = 0.010
SWIVEL_Z = 0.140

TUBE_R = 0.015
ARC_R = 0.072
RISER_TOP = 0.153
REACH_X = 2.0 * ARC_R  # 0.144
DROP_END = 0.124

SLEEVE_R = 0.0165
SLEEVE_LEN = 0.028
AERATOR_R = 0.0118
AERATOR_LEN = 0.003

APEX_WORLD = SWIVEL_Z + RISER_TOP + ARC_R + TUBE_R  # 0.380 m

SWIVEL_LIMIT = math.radians(110.0)
LEVER_LIMIT = math.radians(45.0)


def _faceted_gooseneck() -> cq.Workplane:
    """Faceted gooseneck: straight riser, segmented polyline arc, short drop leg.

    The arc is approximated by 6 straight line segments instead of a smooth
    curve, producing visible angular bends at each segment junction.
    """
    # Compute polyline points along the semicircular arc
    # Arc center: (ARC_R, RISER_TOP) in the XZ workplane
    # Arc goes from angle pi to 0 (left to right over the top)
    n_segments = 6
    arc_pts = []
    for i in range(n_segments + 1):
        angle = math.pi - i * (math.pi / n_segments)
        x = ARC_R + ARC_R * math.cos(angle)
        z = RISER_TOP + ARC_R * math.sin(angle)
        arc_pts.append((x, z))

    # Build the polyline path on the XZ workplane
    wp = cq.Workplane("XZ").moveTo(0.0, 0.0).lineTo(0.0, RISER_TOP)
    for px, pz in arc_pts[1:]:  # skip first point (same as riser top)
        wp = wp.lineTo(px, pz)
    wp = wp.lineTo(REACH_X, DROP_END)

    return cq.Workplane("XY").circle(TUBE_R).sweep(wp)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="faceted_gooseneck_faucet")

    gloss_black = model.material("veined_gloss_black", rgba=(0.075, 0.075, 0.085, 1.0))
    matte_black = model.material("matte_black", rgba=(0.035, 0.035, 0.038, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    outlet_dark = model.material("outlet_dark", rgba=(0.10, 0.10, 0.10, 1.0))
    tick_mat = model.material("tick_indicator", rgba=(0.75, 0.78, 0.80, 1.0))

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

    # Two mounting collars on the pedestal
    column.visual(
        Cylinder(radius=COLLAR_MOUNT_R, length=COLLAR_MOUNT_LEN),
        origin=Origin(xyz=(0.0, 0.0, COLLAR_MOUNT_Z0)),
        material=chrome,
        name="mounting_collar_0",
    )
    column.visual(
        Cylinder(radius=COLLAR_MOUNT_R, length=COLLAR_MOUNT_LEN),
        origin=Origin(xyz=(0.0, 0.0, COLLAR_MOUNT_Z1)),
        material=chrome,
        name="mounting_collar_1",
    )

    # Lever housing boss on the +Y side of the column
    column.visual(
        Cylinder(radius=HOUSING_R, length=HOUSING_LEN),
        origin=Origin(xyz=(0.0, HOUSING_Y, LEVER_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gloss_black,
        name="lever_housing",
    )

    # Cold/hot tick marks as raised geometry on the +Y column face
    tick_y = COLUMN_R + TICK_D / 2.0
    column.visual(
        Box((TICK_W, TICK_D, TICK_H)),
        origin=Origin(xyz=(0.0, tick_y, TICK_COLD_Z)),
        material=tick_mat,
        name="tick_cold",
    )
    column.visual(
        Box((TICK_W, TICK_D, TICK_H)),
        origin=Origin(xyz=(0.0, tick_y, TICK_HOT_Z)),
        material=tick_mat,
        name="tick_hot",
    )

    # Chrome collar ring at the column-spout junction
    column.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_LEN),
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z - COLLAR_LEN / 2.0)),
        material=chrome,
        name="swivel_collar",
    )

    # --------------------------------------------------------------- gooseneck
    spout = model.part("gooseneck_spout")
    spout.visual(
        mesh_from_cadquery(_faceted_gooseneck(), "faceted_gooseneck_tube"),
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

    # ------------------------------------------------------------- side lever
    lever = model.part("side_lever")
    # Base boss that seats into the housing (centered slightly behind pivot
    # so the base embeds into the housing for a mounted appearance)
    lever.visual(
        Cylinder(radius=LEVER_BASE_R, length=LEVER_BASE_LEN),
        origin=Origin(xyz=(0.0, -0.005, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gloss_black,
        name="lever_base",
    )
    # Main handle extending outward along +Y
    handle_y0 = -0.005 + LEVER_BASE_LEN / 2.0  # outer face of the base
    handle_cy = handle_y0 + LEVER_HANDLE_LEN / 2.0
    lever.visual(
        Cylinder(radius=LEVER_HANDLE_R, length=LEVER_HANDLE_LEN),
        origin=Origin(xyz=(0.0, handle_cy, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gloss_black,
        name="lever_handle",
    )
    # Slightly larger grip tip at the end
    tip_cy = handle_y0 + LEVER_HANDLE_LEN + LEVER_TIP_LEN / 2.0
    lever.visual(
        Cylinder(radius=LEVER_TIP_R, length=LEVER_TIP_LEN),
        origin=Origin(xyz=(0.0, tip_cy, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="lever_grip",
    )

    model.articulation(
        "lever_tilt",
        ArticulationType.REVOLUTE,
        parent=column,
        child=lever,
        origin=Origin(xyz=(0.0, LEVER_PIVOT_Y, LEVER_Z)),
        # Axis is horizontal X: positive q tilts the +Y handle end upward (+Z).
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=-LEVER_LIMIT, upper=LEVER_LIMIT
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    column = object_model.get_part("body_column")
    spout = object_model.get_part("gooseneck_spout")
    lever = object_model.get_part("side_lever")

    swivel = object_model.get_articulation("spout_swivel")
    lever_joint = object_model.get_articulation("lever_tilt")

    # Intentional overlap: lever base seats into the housing boss
    ctx.allow_overlap(
        lever,
        column,
        elem_a="lever_base",
        elem_b="lever_housing",
        reason="Lever base intentionally seats into the housing boss for mounting.",
    )

    # ----- grounding and scale
    col_aabb = ctx.part_world_aabb(column)
    ctx.check(
        "faucet grounded on the deck plane",
        col_aabb is not None and abs(col_aabb[0][2]) <= 0.002,
        details=f"column aabb={col_aabb}",
    )

    disc = ctx.part_element_world_aabb(column, elem="base_disc")
    ctx.check(
        "chrome base disc present",
        disc is not None and (disc[1][0] - disc[0][0]) >= 0.080,
        details=f"base disc aabb={disc}",
    )

    # ----- gooseneck apex and reach
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

    # ----- faceted gooseneck tube: verify it exists and is mesh-backed
    tube_visual = spout.get_visual("gooseneck_tube")
    ctx.check(
        "gooseneck tube is a mesh-backed faceted shape",
        tube_visual is not None and hasattr(tube_visual.geometry, "filename"),
    )

    # ----- two mounting collars on the pedestal
    mc0 = ctx.part_element_world_aabb(column, elem="mounting_collar_0")
    mc1 = ctx.part_element_world_aabb(column, elem="mounting_collar_1")
    ctx.check(
        "two mounting collars present on the pedestal",
        mc0 is not None and mc1 is not None,
        details=f"collar_0={mc0}, collar_1={mc1}",
    )
    ctx.check(
        "mounting collars are chrome and wider than the column",
        mc0 is not None
        and mc1 is not None
        and (mc0[1][0] - mc0[0][0]) > 0.040
        and (mc1[1][0] - mc1[0][0]) > 0.040,
        details=f"collar_0_dx={mc0[1][0] - mc0[0][0] if mc0 else None}",
    )
    ctx.check(
        "mounting collars are at two distinct heights on the pedestal",
        mc0 is not None
        and mc1 is not None
        and abs(0.5 * (mc0[0][2] + mc0[1][2]) - 0.5 * (mc1[0][2] + mc1[1][2])) > 0.015,
        details=f"collar_0={mc0}, collar_1={mc1}",
    )

    # ----- cold/hot tick marks as geometry
    tick_c = ctx.part_element_world_aabb(column, elem="tick_cold")
    tick_h = ctx.part_element_world_aabb(column, elem="tick_hot")
    ctx.check(
        "cold and hot tick marks present as geometry",
        tick_c is not None and tick_h is not None,
        details=f"tick_cold={tick_c}, tick_hot={tick_h}",
    )
    ctx.check(
        "tick marks are at two distinct heights near the lever",
        tick_c is not None
        and tick_h is not None
        and tick_c[1][2] < LEVER_Z
        and tick_h[0][2] > LEVER_Z,
        details=f"tick_cold={tick_c}, tick_hot={tick_h}, lever_z={LEVER_Z}",
    )
    ctx.check(
        "tick marks protrude from the column on the +Y side",
        tick_c is not None
        and tick_h is not None
        and tick_c[0][1] > COLUMN_R - 0.001
        and tick_h[0][1] > COLUMN_R - 0.001,
        details=f"tick_cold_y={tick_c[0][1] if tick_c else None}",
    )

    # ----- chrome collar between column and spout
    collar = ctx.part_element_world_aabb(column, elem="swivel_collar")
    ctx.check(
        "chrome swivel collar sits above the mounting collars and below the spout",
        collar is not None
        and mc1 is not None
        and collar[0][2] > mc1[1][2]
        and spout_aabb is not None
        and collar[1][2] <= spout_aabb[0][2] + 1e-6,
        details=f"collar={collar}",
    )

    # ----- spout seats on the collar
    ctx.expect_contact(
        spout,
        column,
        elem_a="gooseneck_tube",
        elem_b="swivel_collar",
        contact_tol=0.002,
        name="gooseneck riser seats on the chrome collar",
    )

    # ----- chrome tip sleeve with downward outlet
    sleeve = ctx.part_element_world_aabb(spout, elem="tip_sleeve")
    aerator = ctx.part_element_world_aabb(spout, elem="outlet_aerator")
    ctx.check(
        "chrome tip sleeve and downward outlet at the spout end",
        sleeve is not None
        and aerator is not None
        and aerator[0][2] < sleeve[0][2],
        details=f"sleeve={sleeve}, aerator={aerator}",
    )

    # ----- single side lever: geometry and mounting
    handle = ctx.part_element_world_aabb(lever, elem="lever_handle")
    ctx.check(
        "side lever handle extends outward from the column",
        handle is not None
        and (handle[1][1] - handle[0][1]) > 0.070
        and handle[0][1] > COLUMN_R,
        details=f"handle aabb={handle}",
    )

    # Lever base seats into housing
    ctx.expect_overlap(
        lever,
        column,
        axes="y",
        elem_a="lever_base",
        elem_b="lever_housing",
        min_overlap=0.005,
        name="lever base seats into the housing boss",
    )

    # ----- joint plan: types, axes, ranges
    ctx.check(
        "spout swivel is revolute ±110 deg about the vertical axis",
        swivel.articulation_type == ArticulationType.REVOLUTE
        and tuple(swivel.axis) == (0.0, 0.0, 1.0)
        and swivel.motion_limits is not None
        and abs(swivel.motion_limits.lower + SWIVEL_LIMIT) < 1e-6
        and abs(swivel.motion_limits.upper - SWIVEL_LIMIT) < 1e-6,
    )
    ctx.check(
        "side lever is revolute ±45 deg about a horizontal axis",
        lever_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(lever_joint.axis) == (1.0, 0.0, 0.0)
        and lever_joint.motion_limits is not None
        and abs(lever_joint.motion_limits.lower + LEVER_LIMIT) < 1e-6
        and abs(lever_joint.motion_limits.upper - LEVER_LIMIT) < 1e-6,
    )

    # ----- lever pose: positive q lifts the handle end upward
    rest_aabb = ctx.part_world_aabb(lever)
    rest_handle_center_z = 0.5 * (handle[0][2] + handle[1][2]) if handle else None
    with ctx.pose({lever_joint: LEVER_LIMIT}):
        tilted_aabb = ctx.part_world_aabb(lever)
        tilted_handle = ctx.part_element_world_aabb(lever, elem="lever_handle")
    tilted_handle_center_z = (
        0.5 * (tilted_handle[0][2] + tilted_handle[1][2]) if tilted_handle else None
    )
    ctx.check(
        "lever positive tilt raises the handle end upward (+Z)",
        rest_handle_center_z is not None
        and tilted_handle_center_z is not None
        and tilted_handle_center_z > rest_handle_center_z + 0.010,
        details=f"rest_z={rest_handle_center_z}, tilted_z={tilted_handle_center_z}",
    )

    with ctx.pose({lever_joint: -LEVER_LIMIT}):
        down_handle = ctx.part_element_world_aabb(lever, elem="lever_handle")
    down_handle_center_z = (
        0.5 * (down_handle[0][2] + down_handle[1][2]) if down_handle else None
    )
    ctx.check(
        "lever negative tilt lowers the handle end downward",
        rest_handle_center_z is not None
        and down_handle_center_z is not None
        and down_handle_center_z < rest_handle_center_z - 0.010,
        details=f"rest_z={rest_handle_center_z}, down_z={down_handle_center_z}",
    )

    # ----- swivel pose: spout outlet sweeps sideways
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

    return ctx.report()


object_model = build_object_model()
