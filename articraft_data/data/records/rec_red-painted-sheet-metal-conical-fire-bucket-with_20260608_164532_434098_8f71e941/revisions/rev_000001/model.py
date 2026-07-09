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

    return ctx.report()


object_model = build_object_model()
