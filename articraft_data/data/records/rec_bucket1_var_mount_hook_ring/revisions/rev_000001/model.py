from __future__ import annotations

# Red painted sheet-metal CONICAL fire bucket (apex DOWN).
#
# These classic sand fire buckets are intentionally pointed at the bottom so they
# cannot be set down and stolen/misused: the cone apex points straight down and
# the bucket hangs from its bail handle.
#
# Coordinate convention:
#   - up is +Z. The cone APEX is the lowest point at z=0; the open circular top
#     and its rolled rim are at z=BODY_H. There is NO flat base (correct: it
#     cannot stand upright).
#   - the body is a hollow thin-wall revolved cone shell, weathered red metal.
#   - two riveted pivot lugs sit just outside opposite sides of the rim along +/-Y.
#   - the steel-wire bail handle pivots about the +/-Y diameter line through the
#     two lugs (a REVOLUTE joint). Its joint origin sits exactly on that lug
#     axis so the handle swings with no float, from hanging down the +Y side,
#     up and over the top, to hanging down the -Y side.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)

# --- key dimensions (meters) ---
TOP_R = 0.135  # outer radius at the open top
BODY_H = 0.250  # apex (z=0) up to the rim (z=BODY_H)
WALL = 0.0016  # sheet-metal wall thickness
APEX_R = 0.004  # tiny radius at the very point (a real cone is not razor sharp)

RIM_TUBE = 0.006  # rolled-rim tube (minor) radius
RIM_CENTER_R = TOP_R - RIM_TUBE * 0.4  # torus center radius at the top edge
RIM_Z = BODY_H

# Pivot lugs / handle pivot axis (the rim diameter along Y).
LUG_Y = TOP_R + 0.014
LUG_Z = BODY_H - 0.012
LUG_HALF_T = 0.0035
LUG_PLATE_W = 0.018
RIVET_R = 0.0035

WIRE_R = 0.0028
HANDLE_RISE = 0.150
HANDLE_RIM_CLEAR_Z = (RIM_Z - LUG_Z) + RIM_TUBE + WIRE_R + 0.004

# --- hanging hook ring mount (above rim, on symmetry axis) ---
HOOK_RING_R = 0.014       # ring major radius (torus center to tube center)
HOOK_RING_TUBE = 0.0028   # ring tube radius
HOOK_SHANK_H = 0.022      # shank height
HOOK_SHANK_R = 0.012      # shank radius (fills ring hole for press-fit weld mount)
HOOK_PLATE_R = 0.008      # central mounting plate radius
HOOK_PLATE_H = 0.003      # central plate thickness
HOOK_ARM_W = 0.010        # support arm width
HOOK_ARM_T = 0.002        # support arm thickness
HOOK_INNER_RIM_R = RIM_CENTER_R - RIM_TUBE  # inner edge of rolled rim


def _conical_shell_mesh(top_r: float, height: float, wall: float, apex_r: float, name: str):
    """Hollow thin-wall cone shell, apex at z=0 (pointing down), open top at z=H.

    Profiles are (radius, z). The cone is open at the top (no top cap), with the
    inner wall offset inward by `wall`. There is no flat base; the cone closes to
    a small apex radius at z=0.
    """
    outer = [
        (apex_r, 0.0),
        (top_r * 0.5, height * 0.5),
        (top_r, height),
    ]
    # Inner cavity: slightly above the apex so the point stays solid metal, wall
    # offset inward by `wall` toward the top.
    inner = [
        (0.0, wall * 2.0),
        (max(apex_r - wall, 0.0006), wall * 2.5),
        (top_r * 0.5 - wall, height * 0.5),
        (top_r - wall, height),
    ]
    geom = LatheGeometry.from_shell_profiles(outer, inner, segments=64)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fire_bucket_conical")

    red_metal = model.material("red_metal", rgba=(0.64, 0.10, 0.09, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))

    # --- conical body (root): hollow cone shell + rolled rim + lugs ---
    bucket = model.part("bucket")

    shell_mesh = _conical_shell_mesh(TOP_R, BODY_H, WALL, APEX_R, "bucket_shell")
    bucket.visual(shell_mesh, material=red_metal, name="bucket_shell")

    rim_geom = TorusGeometry(radius=RIM_CENTER_R, tube=RIM_TUBE, radial_segments=24,
                             tubular_segments=64)
    rim_mesh = mesh_from_geometry(rim_geom, "rim")
    bucket.visual(rim_mesh, origin=Origin(xyz=(0.0, 0.0, RIM_Z)), material=red_metal,
                  name="rolled_rim")

    # Two longer pivot lugs bridge from the rim wall out to the handle pivot.
    lug_inner_y = TOP_R - 0.006
    lug_outer_y = LUG_Y + RIVET_R
    lug_reach = lug_outer_y - lug_inner_y
    lug_center_y = (lug_inner_y + lug_outer_y) / 2.0
    for sgn, tag in ((+1.0, "pos"), (-1.0, "neg")):
        bucket.visual(
            Box((2.0 * LUG_HALF_T, lug_reach, LUG_PLATE_W)),
            origin=Origin(xyz=(0.0, sgn * lug_center_y, LUG_Z)),
            material=red_metal,
            name=f"lug_{tag}",
        )
        bucket.visual(
            Cylinder(radius=RIVET_R, length=0.005),
            origin=Origin(xyz=(0.0, sgn * LUG_Y, LUG_Z),
                          rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"rivet_{tag}",
        )

    # --- hanging hook ring mount (riveted above rim, on symmetry axis) ---
    # Four cross support arms bridge from the inner rim edge to the center plate,
    # carrying the shank and suspension ring.
    arm_len = HOOK_INNER_RIM_R - HOOK_PLATE_R
    arm_center_r = (HOOK_INNER_RIM_R + HOOK_PLATE_R) / 2.0
    for i in range(4):
        angle = math.radians(i * 90.0)
        cx = arm_center_r * math.cos(angle)
        cy = arm_center_r * math.sin(angle)
        yaw = angle
        bucket.visual(
            Box((arm_len, HOOK_ARM_W, HOOK_ARM_T)),
            origin=Origin(xyz=(cx, cy, RIM_Z + HOOK_ARM_T / 2.0),
                          rpy=(0.0, 0.0, yaw)),
            material=steel,
            name=f"hook_arm_{i}",
        )

    # Central mounting plate sits on top of the rim plane.
    bucket.visual(
        Cylinder(radius=HOOK_PLATE_R, length=HOOK_PLATE_H),
        origin=Origin(xyz=(0.0, 0.0, RIM_Z + HOOK_PLATE_H / 2.0)),
        material=steel,
        name="hook_plate",
    )

    # Shank stem rises from the plate to carry the ring.
    shank_base_z = RIM_Z + HOOK_PLATE_H
    bucket.visual(
        Cylinder(radius=HOOK_SHANK_R, length=HOOK_SHANK_H),
        origin=Origin(xyz=(0.0, 0.0, shank_base_z + HOOK_SHANK_H / 2.0)),
        material=steel,
        name="hook_shank",
    )

    # Torus ring (suspension eyelet) sits at the top of the shank, oriented
    # horizontally so the opening faces upward for hanging from a hook/post.
    # The ring center is at the shank top so the tube embeds slightly into the
    # shank for a welded-on mount connection.
    ring_z = shank_base_z + HOOK_SHANK_H
    ring_geom = TorusGeometry(
        radius=HOOK_RING_R,
        tube=HOOK_RING_TUBE,
        radial_segments=16,
        tubular_segments=48,
    )
    ring_mesh = mesh_from_geometry(ring_geom, "hook_ring")
    bucket.visual(
        ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, ring_z)),
        material=steel,
        name="hook_ring",
    )

    bucket.inertial = Inertial.from_geometry(
        Cylinder(radius=TOP_R, length=BODY_H),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.0, BODY_H * 0.6)),
    )

    # --- steel-wire bail handle (REVOLUTE about the +/-Y lug diameter) ---
    handle = model.part("handle")
    handle_geom = tube_from_spline_points(
        [
            (0.0, +LUG_Y, 0.0),
            (0.0, +LUG_Y, HANDLE_RIM_CLEAR_Z),
            (0.0, +LUG_Y * 0.96, HANDLE_RISE * 0.55),
            (0.0, +LUG_Y * 0.45, HANDLE_RISE * 0.95),
            (0.0, 0.0, HANDLE_RISE),
            (0.0, -LUG_Y * 0.45, HANDLE_RISE * 0.95),
            (0.0, -LUG_Y * 0.96, HANDLE_RISE * 0.55),
            (0.0, -LUG_Y, HANDLE_RIM_CLEAR_Z),
            (0.0, -LUG_Y, 0.0),
        ],
        radius=WIRE_R,
        samples_per_segment=16,
        radial_segments=16,
        cap_ends=True,
    )
    handle_mesh = mesh_from_geometry(handle_geom, "bail_wire")
    handle.visual(handle_mesh, material=steel, name="bail_wire")
    handle.inertial = Inertial.from_geometry(
        Cylinder(radius=WIRE_R, length=2.0 * LUG_Y), mass=0.08
    )

    model.articulation(
        "bucket_to_handle",
        ArticulationType.REVOLUTE,
        parent=bucket,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, LUG_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=3.0,
            lower=math.radians(-100.0),
            upper=math.radians(100.0),
        ),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def _aabb_center(aabb):
    mn, mx = aabb
    return ((mn[0] + mx[0]) / 2.0, (mn[1] + mx[1]) / 2.0, (mn[2] + mx[2]) / 2.0)


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bucket = object_model.get_part("bucket")
    handle = object_model.get_part("handle")
    joint = object_model.get_articulation("bucket_to_handle")

    # --- intentional overlaps: lugs riveted to rim/wall; rivets through lugs;
    #     bail-wire ends seat into the extended lugs while clearing the rim. ---
    ctx.allow_overlap(bucket, bucket, elem_a="lug_pos", elem_b="rolled_rim",
                      reason="Lug tab is riveted onto the rolled rim/wall.")
    ctx.allow_overlap(bucket, bucket, elem_a="lug_neg", elem_b="rolled_rim",
                      reason="Lug tab is riveted onto the rolled rim/wall.")
    ctx.allow_overlap(bucket, bucket, elem_a="rivet_pos", elem_b="lug_pos",
                      reason="Rivet head is set through the lug tab.")
    ctx.allow_overlap(bucket, bucket, elem_a="rivet_neg", elem_b="lug_neg",
                      reason="Rivet head is set through the lug tab.")
    ctx.allow_overlap(handle, bucket, elem_a="bail_wire", elem_b="lug_pos",
                      reason="Bail-wire end drops into the +Y pivot lug.")
    ctx.allow_overlap(handle, bucket, elem_a="bail_wire", elem_b="lug_neg",
                      reason="Bail-wire end drops into the -Y pivot lug.")

    # --- hook ring mount: arms riveted to plate, shank welded to plate, ring on shank ---
    for i in range(4):
        ctx.allow_overlap(bucket, bucket, elem_a=f"hook_arm_{i}", elem_b="hook_plate",
                          reason=f"Hook arm {i} is riveted to the central mounting plate.")
    ctx.allow_overlap(bucket, bucket, elem_a="hook_shank", elem_b="hook_plate",
                      reason="Hook shank is welded to the mounting plate.")
    ctx.allow_overlap(bucket, bucket, elem_a="hook_ring", elem_b="hook_shank",
                      reason="Hook ring is mounted on top of the shank stem.")

    # --- apex points DOWN at z~0, open top is up; NO flat base ---
    shell_aabb = ctx.part_element_world_aabb(bucket, elem="bucket_shell")
    ctx.check(
        "cone apex is the lowest point at z~0",
        abs(shell_aabb[0][2]) < 0.002,
        details=f"shell_minZ={shell_aabb[0][2]}",
    )
    ctx.check(
        "cone height matches design (~0.25 m)",
        abs((shell_aabb[1][2] - shell_aabb[0][2]) - BODY_H) < 0.01,
        details=f"shell_h={shell_aabb[1][2] - shell_aabb[0][2]}",
    )
    # The bottom is a point, not a flat base.
    ctx.check(
        "bottom converges to a point, not a flat base (apex_r tiny vs top_r)",
        APEX_R < TOP_R * 0.1,
        details=f"apex_r={APEX_R}, top_r={TOP_R}",
    )
    # Confirm the wide end (rim) is at the TOP, the narrow apex at the bottom.
    rim_aabb = ctx.part_element_world_aabb(bucket, elem="rolled_rim")
    ctx.check(
        "wide open mouth (rim) is at the top, narrow point at the bottom",
        _aabb_center(rim_aabb)[2] > _aabb_center(shell_aabb)[2],
        details=f"rim_z={_aabb_center(rim_aabb)[2]}, shell_center_z={_aabb_center(shell_aabb)[2]}",
    )

    # --- body is HOLLOW: thin wall, open interior ---
    ctx.check(
        "cone body modeled hollow (thin wall, open interior)",
        WALL < TOP_R * 0.05,
        details=f"wall={WALL}, top_r={TOP_R}",
    )

    # --- two lugs on opposite sides along Y, symmetric, at the rim ---
    lug_pos_c = _aabb_center(ctx.part_element_world_aabb(bucket, elem="lug_pos"))
    lug_neg_c = _aabb_center(ctx.part_element_world_aabb(bucket, elem="lug_neg"))
    ctx.check(
        "two pivot lugs on opposite rim sides (+/-Y)",
        lug_pos_c[1] > 0.0 > lug_neg_c[1] and abs(lug_pos_c[1] + lug_neg_c[1]) < 0.01,
        details=f"lug_pos_y={lug_pos_c[1]}, lug_neg_y={lug_neg_c[1]}",
    )
    ctx.check(
        "lugs sit near the rim height",
        abs(lug_pos_c[2] - LUG_Z) < 0.02 and abs(lug_neg_c[2] - LUG_Z) < 0.02,
        details=f"lug_pos_z={lug_pos_c[2]}, lug_neg_z={lug_neg_c[2]}, lug_z={LUG_Z}",
    )

    # --- handle is REVOLUTE, axis = the +/-Y diameter line through both lugs ---
    ctx.check(
        "handle joint is revolute",
        str(joint.articulation_type).upper().endswith("REVOLUTE"),
        details=f"type={joint.articulation_type}",
    )
    ax = joint.axis
    ctx.check(
        "handle axis is the horizontal Y diameter line",
        abs(ax[1]) > 0.99 and abs(ax[0]) < 0.02 and abs(ax[2]) < 0.02,
        details=f"axis={ax}",
    )
    jo = joint.origin.xyz
    ctx.check(
        "joint origin sits on the lug pivot axis (no float)",
        abs(jo[0]) < 1e-6 and abs(jo[1]) < 1e-6 and abs(jo[2] - LUG_Z) < 1e-6,
        details=f"origin={jo}, lug_z={LUG_Z}",
    )
    ctx.check(
        "pivot axis passes through both lugs",
        abs(lug_pos_c[2] - jo[2]) < 0.02 and abs(lug_neg_c[2] - jo[2]) < 0.02
        and abs(lug_pos_c[0]) < 0.01 and abs(lug_neg_c[0]) < 0.01,
        details=f"lug_pos={lug_pos_c}, lug_neg={lug_neg_c}, origin_z={jo[2]}",
    )

    lo, hi = joint.motion_limits.lower, joint.motion_limits.upper
    ctx.check(
        "handle swings ~180 deg over the top",
        lo < math.radians(-80.0) and hi > math.radians(80.0),
        details=f"lower={lo}, upper={hi}",
    )

    # --- handle ends meet the lugs (no gap); arch rises over the top ---
    wire_aabb = ctx.part_element_world_aabb(handle, elem="bail_wire")
    ctx.check(
        "bail wire spans across to both lugs (reaches +/-Y rim)",
        wire_aabb[1][1] > LUG_Y - 0.01 and wire_aabb[0][1] < -(LUG_Y - 0.01),
        details=f"wire_y=({wire_aabb[0][1]},{wire_aabb[1][1]}), lug_y={LUG_Y}",
    )
    ctx.expect_contact(handle, bucket, elem_a="bail_wire", elem_b="lug_pos",
                       contact_tol=0.004, name="bail end meets +Y lug")
    ctx.expect_contact(handle, bucket, elem_a="bail_wire", elem_b="lug_neg",
                       contact_tol=0.004, name="bail end meets -Y lug")
    rim_outer_y = RIM_CENTER_R + RIM_TUBE
    ctx.check(
        "bail side connectors stand outside the rolled rim",
        LUG_Y - WIRE_R > rim_outer_y + 0.001,
        details=f"wire_inner_y={LUG_Y - WIRE_R}, rim_outer_y={rim_outer_y}",
    )
    ctx.check(
        "bail starts curving only after it clears the rim height",
        HANDLE_RIM_CLEAR_Z > (RIM_Z + RIM_TUBE - LUG_Z) + WIRE_R,
        details=(
            f"clear_rel_z={HANDLE_RIM_CLEAR_Z}, "
            f"rim_top_rel_z={RIM_Z + RIM_TUBE - LUG_Z}, wire_r={WIRE_R}"
        ),
    )
    ctx.check(
        "bail arch rises above the rim at rest",
        wire_aabb[1][2] > RIM_Z + HANDLE_RISE * 0.5,
        details=f"wire_maxZ={wire_aabb[1][2]}, rim_z={RIM_Z}",
    )

    # --- decisive pose check: handle swings down over the side ---
    apex_rest = _aabb_center(wire_aabb)
    with ctx.pose({joint: math.radians(95.0)}):
        apex_down = _aabb_center(ctx.part_element_world_aabb(handle, elem="bail_wire"))
    ctx.check(
        "handle swings down over the side when rotated",
        apex_down[2] < apex_rest[2] - 0.05,
        details=f"rest center={apex_rest}, swung center={apex_down}",
    )

    # --- colors: body red, wire steel ---
    shell_rgba = bucket.get_visual("bucket_shell").material.rgba
    wire_rgba = handle.get_visual("bail_wire").material.rgba
    ctx.check(
        "cone body is red",
        shell_rgba[0] > 0.5 and shell_rgba[1] < 0.25 and shell_rgba[2] < 0.25,
        details=f"shell_rgba={shell_rgba}",
    )
    ctx.check(
        "bail handle is steel-gray",
        min(wire_rgba[:3]) > 0.55 and max(wire_rgba[:3]) - min(wire_rgba[:3]) < 0.2,
        details=f"wire_rgba={wire_rgba}",
    )

    # --- hook ring mount: closed eyelet above rim on symmetry axis ---
    ring_aabb = ctx.part_element_world_aabb(bucket, elem="hook_ring")
    plate_aabb = ctx.part_element_world_aabb(bucket, elem="hook_plate")
    shank_aabb = ctx.part_element_world_aabb(bucket, elem="hook_shank")

    # Ring exists above the rim
    ctx.check(
        "hook ring exists above the rim",
        ring_aabb[0][2] > RIM_Z,
        details=f"ring_minZ={ring_aabb[0][2]}, rim_z={RIM_Z}",
    )

    # Ring is centered on the symmetry axis (x~0, y~0)
    ring_center = _aabb_center(ring_aabb)
    ctx.check(
        "hook ring is centered on the symmetry axis",
        abs(ring_center[0]) < 0.005 and abs(ring_center[1]) < 0.005,
        details=f"ring_center_xy=({ring_center[0]}, {ring_center[1]})",
    )

    # Ring opening faces upward for hanging (horizontal torus means the ring
    # spans wide in XY but is thin in Z, indicating horizontal orientation)
    ring_dx = ring_aabb[1][0] - ring_aabb[0][0]
    ring_dy = ring_aabb[1][1] - ring_aabb[0][1]
    ring_dz = ring_aabb[1][2] - ring_aabb[0][2]
    ctx.check(
        "hook ring opening faces upward (horizontal torus orientation)",
        ring_dx > ring_dz * 2.0 and ring_dy > ring_dz * 2.0,
        details=f"ring_dx={ring_dx}, ring_dy={ring_dy}, ring_dz={ring_dz}",
    )

    # Shank connects plate to ring (shank rises from plate, ring mounts on shank top)
    ctx.check(
        "hook shank connects plate to ring",
        shank_aabb[0][2] >= plate_aabb[1][2] - 0.001 and ring_center[2] >= shank_aabb[1][2] - 0.005,
        details=f"plate_top={plate_aabb[1][2]}, shank=({shank_aabb[0][2]}, {shank_aabb[1][2]}), ring_center_z={ring_center[2]}",
    )

    # Support arms bridge from rim to center (arms span from inner rim to plate)
    arm_0_aabb = ctx.part_element_world_aabb(bucket, elem="hook_arm_0")
    ctx.check(
        "hook support arms bridge from rim to center plate",
        arm_0_aabb[0][0] > HOOK_PLATE_R - 0.001 and arm_0_aabb[1][0] < HOOK_INNER_RIM_R + 0.001,
        details=f"arm_0_x=({arm_0_aabb[0][0]}, {arm_0_aabb[1][0]}), plate_r={HOOK_PLATE_R}, rim_inner_r={HOOK_INNER_RIM_R}",
    )

    # Bail handle remains a REVOLUTE joint about the horizontal Y axis
    ctx.check(
        "bail handle joint is REVOLUTE about horizontal Y axis",
        str(joint.articulation_type).upper().endswith("REVOLUTE")
        and abs(joint.axis[1]) > 0.99 and abs(joint.axis[0]) < 0.02 and abs(joint.axis[2]) < 0.02,
        details=f"type={joint.articulation_type}, axis={joint.axis}",
    )

    return ctx.report()


object_model = build_object_model()
