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
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# High-arc gooseneck faucet variant (pre-rinse commercial style).
#
# Variant of the gloss-black monobloc mixer tap forked into a distinct
# high-arc gooseneck sibling with:
# - A slim commercial pre-rinse style spring around the arc
# - A single side lever (revolute joint)
# - A ribbed spray head at the spout tip
#
# Layout (world frame, deck plane at z = 0):
# - +X is the front (direction the gooseneck reaches over the sink), +Z is up.
# - Chrome base disc on deck; gloss-black cylindrical column (0.04 m dia).
# - A thin chrome collar ring separates column from the swivel spout.
# - Swan-neck gooseneck arcs up to ~0.38 m apex, with a pre-rinse spring
#   coiled around the arc portion.
# - Ribbed spray head at the spout tip with downward outlet.
# - Single side lever on the right side of the column, revolute about the
#   horizontal Y axis, range -90 to 0 degrees (tilts toward user).
# ---------------------------------------------------------------------------

# Base + column
BASE_DISC_R = 0.042
BASE_DISC_H = 0.008
COLUMN_R = 0.020  # 0.04 m diameter
COLUMN_TOP = 0.132

# Swivel collar
COLLAR_R = 0.0215
COLLAR_LEN = 0.010
SWIVEL_Z = 0.140

# Gooseneck tube (spout-local frame at collar top)
TUBE_R = 0.015
ARC_R = 0.072
RISER_TOP = 0.153  # centerline apex = 0.153 + 0.072 = 0.225
REACH_X = 2.0 * ARC_R  # 0.144 m horizontal reach
DROP_END = 0.124  # spout-local z of tube tip (world 0.264)

# Spray head (ribbed)
SPRAY_R = 0.019  # slightly wider than tube
SPRAY_LEN = 0.045
SPRAY_RIB_COUNT = 8
SPRAY_RIB_DEPTH = 0.0012
SPRAY_RIB_WIDTH = 0.003

# Spring coil around the arc
SPRING_WIRE_R = 0.0018  # thin wire
SPRING_COIL_R = TUBE_R + SPRING_WIRE_R * 0.5  # coil radius embeds slightly in tube for connectivity
SPRING_TURNS = 10  # number of turns around the arc
# Spring covers the arc portion of the gooseneck

# Side lever (single paddle, pivot along Y axis)
LEVER_MOUNT_Z = 0.090  # height on column
LEVER_ORIGIN_Y = COLUMN_R  # articulation origin on column surface
LEVER_BOSS_R = 0.012
LEVER_BOSS_LEN = 0.014
LEVER_BOSS_DY = 0.008  # boss center offset from lever origin in +Y
LEVER_ARM_W = 0.010
LEVER_ARM_H = 0.006
LEVER_ARM_LEN = 0.080

# Mount boss on column (protrudes from column surface for connectivity)
MOUNT_BOSS_R = 0.015
MOUNT_BOSS_LEN = 0.012
MOUNT_BOSS_Y = COLUMN_R + MOUNT_BOSS_LEN / 2.0 - 0.004  # overlaps column for connectivity

# Apex calculation
APEX_WORLD = SWIVEL_Z + RISER_TOP + ARC_R + TUBE_R  # 0.380 m

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


def _spring_points() -> list[tuple[float, float, float]]:
    """Generate helical spring points that coil around the gooseneck arc portion.

    The spring wraps around the semicircular arc of the gooseneck. The arc
    center is at (ARC_R, RISER_TOP) in the XZ plane, spanning from angle
    pi (left/up) to 0 (right/down). The spring spirals around this path.
    """
    points = []
    # The arc goes from (0, RISER_TOP) through the apex to (REACH_X, RISER_TOP)
    # Parameterize by arc angle theta from pi to 0 (semicircle)
    n_total = SPRING_TURNS * 24  # points per turn * turns
    for i in range(n_total + 1):
        t = i / n_total  # 0..1 along the arc
        theta = math.pi * (1.0 - t)  # arc angle from pi to 0

        # Arc centerline position in XZ
        arc_x = ARC_R + ARC_R * math.cos(theta)
        arc_z = RISER_TOP + ARC_R * math.sin(theta)

        # Helical offset: the spring coils around the tube
        # The helix angle progresses with t
        helix_angle = t * SPRING_TURNS * 2.0 * math.pi

        # Local radial directions perpendicular to the arc tangent
        # The arc tangent at angle theta is perpendicular to the radial direction
        # Radial direction from arc center: (cos(theta), 0, sin(theta))
        # Tangent direction: (sin(theta), 0, -cos(theta)) (pointing along the path)
        # We need two perpendicular directions to the tangent for the helix offset
        # One is the radial: (cos(theta), 0, sin(theta))
        # The other is Y: (0, 1, 0)

        # Helix offset in the radial-Y plane
        hx = SPRING_COIL_R * math.cos(helix_angle)
        hy = SPRING_COIL_R * math.sin(helix_angle)

        # Transform into world: radial component along arc radial, Y stays
        radial_x = math.cos(theta)
        radial_z = math.sin(theta)

        px = arc_x + hx * radial_x
        py = hy  # Y direction is unchanged
        pz = arc_z + hx * radial_z

        points.append((px, py, pz))

    return points


def _ribbed_spray_head() -> cq.Workplane:
    """Create a ribbed spray head: cylinder with shallow circumferential grooves."""
    # Start with a cylinder
    body = cq.Workplane("XY").circle(SPRAY_R).extrude(SPRAY_LEN)

    # Cut shallow grooves around the circumference at regular intervals
    rib_spacing = SPRAY_LEN / (SPRAY_RIB_COUNT + 1)
    for i in range(1, SPRAY_RIB_COUNT + 1):
        z_pos = i * rib_spacing
        # Cut a thin torus-shaped groove
        groove = (
            cq.Workplane("XY")
            .workplane(offset=z_pos)
            .circle(SPRAY_R + 0.001)
            .circle(SPRAY_R - SPRAY_RIB_DEPTH)
            .extrude(SPRAY_RIB_WIDTH)
        )
        body = body.cut(groove)

    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="high_arc_gooseneck_faucet")

    gloss_black = model.material("veined_gloss_black", rgba=(0.075, 0.075, 0.085, 1.0))
    matte_black = model.material("matte_black", rgba=(0.035, 0.035, 0.038, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    spring_steel = model.material("spring_steel", rgba=(0.72, 0.74, 0.76, 1.0))
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
    # Chrome collar ring
    column.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_LEN),
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z - COLLAR_LEN / 2.0)),
        material=chrome,
        name="swivel_collar",
    )
    # Lever mount boss on column side (protrudes from column surface)
    column.visual(
        Cylinder(radius=MOUNT_BOSS_R, length=MOUNT_BOSS_LEN),
        origin=Origin(
            xyz=(0.0, MOUNT_BOSS_Y, LEVER_MOUNT_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=gloss_black,
        name="lever_mount_boss",
    )

    # --------------------------------------------------------------- gooseneck
    spout = model.part("gooseneck_spout")
    spout.visual(
        mesh_from_cadquery(_gooseneck_shape(), "gooseneck_tube"),
        material=gloss_black,
        name="gooseneck_tube",
    )

    # Pre-rinse spring coil around the arc
    spring_pts = _spring_points()
    spring_mesh = tube_from_spline_points(
        spring_pts,
        radius=SPRING_WIRE_R,
        samples_per_segment=4,
        radial_segments=10,
        cap_ends=True,
        spline="catmull_rom",
    )
    spout.visual(
        mesh_from_geometry(spring_mesh, "pre_rinse_spring"),
        material=spring_steel,
        name="pre_rinse_spring",
    )

    # Ribbed spray head at spout tip
    spout.visual(
        mesh_from_cadquery(_ribbed_spray_head(), "spray_head"),
        origin=Origin(xyz=(REACH_X, 0.0, DROP_END - SPRAY_LEN)),
        material=matte_black,
        name="spray_head",
    )

    # Outlet aerator at bottom of spray head
    spout.visual(
        Cylinder(radius=SPRAY_R * 0.6, length=0.003),
        origin=Origin(xyz=(REACH_X, 0.0, DROP_END - SPRAY_LEN - 0.001)),
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
    # Lever boss: pivot housing along Y axis (the pivot direction)
    lever.visual(
        Cylinder(radius=LEVER_BOSS_R, length=LEVER_BOSS_LEN),
        origin=Origin(
            xyz=(0.0, LEVER_BOSS_DY, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=chrome,
        name="lever_boss",
    )
    # Lever arm (flat paddle extending upward from the boss, overlapping for connectivity)
    lever_arm_shape = (
        cq.Workplane("XY")
        .box(LEVER_ARM_W, LEVER_ARM_H, LEVER_ARM_LEN)
        .translate((0.0, 0.0, LEVER_BOSS_R + LEVER_ARM_LEN / 2.0 - 0.003))
    )
    lever.visual(
        mesh_from_cadquery(lever_arm_shape, "lever_arm"),
        material=gloss_black,
        name="lever_arm",
    )

    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=column,
        child=lever,
        origin=Origin(xyz=(0.0, LEVER_ORIGIN_Y, LEVER_MOUNT_Z)),
        # Axis is the horizontal Y axis. With axis -Y, negative q tilts
        # the lever arm from vertical toward +X (the user side).
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
    lever = object_model.get_part("side_lever")

    swivel = object_model.get_articulation("spout_swivel")
    lever_pivot = object_model.get_articulation("lever_pivot")

    # Intentional seated insertion: lever boss embeds into the mount boss
    ctx.allow_overlap(
        lever,
        column,
        elem_a="lever_boss",
        elem_b="lever_mount_boss",
        reason="Lever boss intentionally seats into the column mount boss for pivot housing.",
    )

    # ----- grounding, scale, proportions
    col_aabb = ctx.part_world_aabb(column)
    ctx.check(
        "faucet grounded on the deck plane",
        col_aabb is not None and abs(col_aabb[0][2]) <= 0.002,
        details=f"column aabb={col_aabb}",
    )
    disc = ctx.part_element_world_aabb(column, elem="base_disc")
    ctx.check(
        "single chrome base disc sits on the deck",
        disc is not None
        and 0.080 <= (disc[1][0] - disc[0][0]) <= 0.090
        and (disc[1][2] - disc[0][2]) <= 0.010,
        details=f"base disc aabb={disc}",
    )

    # ----- gooseneck: tall arc, apex near 0.38m
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "gooseneck apex near 0.38 m",
        spout_aabb is not None and 0.370 <= spout_aabb[1][2] <= 0.392,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "gooseneck arcs forward over the sink (+X reach)",
        spout_aabb is not None and spout_aabb[1][0] >= 0.130,
        details=f"spout aabb={spout_aabb}",
    )

    # ----- pre-rinse spring exists around the arc
    spring = ctx.part_element_world_aabb(spout, elem="pre_rinse_spring")
    ctx.check(
        "pre-rinse spring coil wraps around the gooseneck arc",
        spring is not None
        and spring[1][2] - spring[0][2] > 0.04  # spans a significant arc height
        and spring[1][0] - spring[0][0] > 0.04,  # spans horizontal reach
        details=f"spring aabb={spring}",
    )

    # ----- ribbed spray head at spout tip
    spray = ctx.part_element_world_aabb(spout, elem="spray_head")
    ctx.check(
        "ribbed spray head at the spout tip",
        spray is not None
        and 0.035 <= (spray[1][2] - spray[0][2]) <= 0.055  # spray head length
        and spray[0][2] < SWIVEL_Z + DROP_END  # below the tube end
        and 0.034 <= (spray[1][0] - spray[0][0]) <= 0.042,  # ~0.038m dia
        details=f"spray aabb={spray}",
    )

    # ----- chrome collar between column and spout
    collar = ctx.part_element_world_aabb(column, elem="swivel_collar")
    ctx.check(
        "thin chrome collar sits between column and spout",
        collar is not None
        and collar[1][2] - collar[0][2] <= 0.012,
        details=f"collar={collar}",
    )

    # ----- side lever: geometry and articulation
    lever_aabb = ctx.part_world_aabb(lever)
    boss = ctx.part_element_world_aabb(lever, elem="lever_boss")
    arm = ctx.part_element_world_aabb(lever, elem="lever_arm")
    ctx.check(
        "side lever has a boss and an arm extending vertically at rest",
        boss is not None
        and arm is not None
        and arm[1][2] > boss[1][2] + 0.05,  # arm extends above boss
        details=f"boss={boss}, arm={arm}",
    )
    ctx.check(
        "side lever is mounted on the column side (+Y)",
        lever_aabb is not None
        and lever_aabb[0][1] > 0.015,  # clearly offset to +Y side
        details=f"lever aabb={lever_aabb}",
    )

    # Lever boss seats into mount boss
    ctx.expect_overlap(
        lever,
        column,
        axes="y",
        elem_a="lever_boss",
        elem_b="lever_mount_boss",
        min_overlap=0.003,
        name="lever boss seats into column mount boss",
    )

    # ----- joint plan: types, axes, ranges
    ctx.check(
        "spout swivel is revolute -110..+110 deg about the vertical column axis",
        swivel.articulation_type == ArticulationType.REVOLUTE
        and tuple(swivel.axis) == (0.0, 0.0, 1.0)
        and swivel.motion_limits is not None
        and abs(swivel.motion_limits.lower + SWIVEL_LIMIT) < 1e-6
        and abs(swivel.motion_limits.upper - SWIVEL_LIMIT) < 1e-6,
    )
    ctx.check(
        "side lever pivot is revolute -90..0 deg about horizontal Y axis",
        lever_pivot.articulation_type == ArticulationType.REVOLUTE
        and tuple(lever_pivot.axis) == (0.0, -1.0, 0.0)
        and lever_pivot.motion_limits is not None
        and abs(lever_pivot.motion_limits.lower + math.pi / 2.0) < 1e-6
        and abs(lever_pivot.motion_limits.upper) < 1e-6
        and lever_pivot.mimic is None,
    )

    # ----- lever pose: full -90 deg tilt brings the arm toward the user (+X)
    rest_lever = ctx.part_world_aabb(lever)
    with ctx.pose({lever_pivot: -math.pi / 2.0}):
        tilted_lever = ctx.part_world_aabb(lever)
    ctx.check(
        "side lever tilts from vertical toward the user at q=-90 deg",
        rest_lever is not None
        and tilted_lever is not None
        and tilted_lever[1][0] > rest_lever[1][0] + 0.04
        and tilted_lever[1][2] < rest_lever[1][2],
        details=f"rest={rest_lever}, tilted={tilted_lever}",
    )

    # ----- swivel pose: spout outlet sweeps sideways
    rest_spray = ctx.part_element_world_aabb(spout, elem="spray_head")
    with ctx.pose({swivel: 1.0}):
        sw_spray = ctx.part_element_world_aabb(spout, elem="spray_head")
    ctx.check(
        "spout swivel carries the spray head sideways about the vertical axis",
        rest_spray is not None
        and sw_spray is not None
        and abs(0.5 * (rest_spray[0][1] + rest_spray[1][1])) < 0.005
        and abs(0.5 * (sw_spray[0][1] + sw_spray[1][1])) > 0.05,
        details=f"rest={rest_spray}, swiveled={sw_spray}",
    )

    return ctx.report()


object_model = build_object_model()
