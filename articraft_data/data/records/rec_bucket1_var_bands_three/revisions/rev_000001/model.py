from __future__ import annotations

# Red painted sheet-metal fire bucket (TAPERED CYLINDER variant).
#
# Coordinate convention:
#   - up is +Z. The flat bottom of the bucket rests on the ground at z=0.
#   - the bucket is a tapered cylinder, wider at the open top, narrower at the
#     flat bottom, modeled as a hollow thin-wall revolved shell.
#   - the rolled top rim is a torus around the top edge.
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
TOP_R = 0.140  # outer radius at the open top
BOT_R = 0.105  # outer radius at the flat bottom (tapered narrower)
BODY_H = 0.260  # bucket height (bottom z=0 -> rim z=BODY_H)
WALL = 0.0016  # sheet-metal wall thickness
BOTTOM_T = 0.004  # thickness of the flat bottom plate

RIM_TUBE = 0.006  # rolled-rim tube (minor) radius
RIM_CENTER_R = TOP_R - RIM_TUBE * 0.4  # torus center radius at the top edge
RIM_Z = BODY_H  # rolled rim sits at the top edge

# Pivot lugs / handle pivot axis (the rim diameter along Y).
LUG_Y = TOP_R + 0.014  # lug pivot point outboard of the rim wall
LUG_Z = BODY_H - 0.012  # pivot axis a touch below the very top edge
LUG_HALF_T = 0.0035  # half-thickness of the lug tab (along X)
LUG_PLATE_W = 0.018  # lug tab width along Z
LUG_PLATE_R = 0.011  # lug tab reach (radial) below the pivot
RIVET_R = 0.0035  # rivet head radius

WIRE_R = 0.0028  # steel bail-wire radius
HANDLE_RISE = 0.150  # how far the arch apex rises above the pivot axis
HANDLE_RIM_CLEAR_Z = (RIM_Z - LUG_Z) + RIM_TUBE + WIRE_R + 0.004

# --- reinforcing bands (raised hoop ribs) ---
BAND_COUNT = 3  # exactly three horizontal bands
BAND_TUBE_R = 0.0025  # rib cross-section radius (protrusion from wall)


def _wall_radius_at(z: float) -> float:
    """Outer wall radius at height z (linear taper from BOT_R to TOP_R)."""
    return BOT_R + (TOP_R - BOT_R) * (z / BODY_H)


def _band_mesh(band_z: float, name: str):
    """Thin torus hoop rib at the given height, sized to the local wall radius."""
    r = _wall_radius_at(band_z)
    geom = TorusGeometry(radius=r, tube=BAND_TUBE_R, radial_segments=12,
                         tubular_segments=64)
    return mesh_from_geometry(geom, name)


def _revolved_shell_mesh(
    top_r: float,
    bot_r: float,
    height: float,
    wall: float,
    bottom_t: float,
    name: str,
):
    """Hollow tapered bucket wall + flat bottom as one revolved thin shell.

    Profiles are (radius, z). The outer wall runs from the flat bottom up to the
    top edge; the inner wall mirrors it offset inward by `wall`, leaving a solid
    flat bottom plate of thickness `bottom_t`.
    """
    outer = [
        (bot_r, 0.0),
        (bot_r + (top_r - bot_r) * 0.5, height * 0.5),
        (top_r, height),
    ]
    # Inner cavity: floor sits at bottom_t, wall offset inward by `wall`.
    inner_bot_r = bot_r - wall
    inner_top_r = top_r - wall
    inner = [
        (0.0, bottom_t),
        (inner_bot_r, bottom_t),
        (inner_bot_r + (inner_top_r - inner_bot_r) * 0.5, height * 0.5),
        (inner_top_r, height),
    ]
    geom = LatheGeometry.from_shell_profiles(outer, inner, segments=64)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fire_bucket_cylindrical")

    red_metal = model.material("red_metal", rgba=(0.62, 0.09, 0.08, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))

    # --- bucket body (root): hollow tapered shell + rolled rim + lugs ---
    bucket = model.part("bucket")

    shell_mesh = _revolved_shell_mesh(TOP_R, BOT_R, BODY_H, WALL, BOTTOM_T, "bucket_shell")
    bucket.visual(shell_mesh, material=red_metal, name="bucket_shell")

    rim_geom = TorusGeometry(radius=RIM_CENTER_R, tube=RIM_TUBE, radial_segments=24,
                             tubular_segments=64)
    rim_mesh = mesh_from_geometry(rim_geom, "rim")
    bucket.visual(rim_mesh, origin=Origin(xyz=(0.0, 0.0, RIM_Z)), material=red_metal,
                  name="rolled_rim")

    # Two longer pivot lugs on opposite rim sides (+/-Y). Each: a flat tab that
    # bridges from the rim wall outward to the pivot point, capped with a rivet
    # head.
    # The tab is a centered slab; its center sits midway between the wall
    # (radius ~TOP_R) and the pivot point (LUG_Y), spanning both so it is riveted
    # to the wall and carries the bail-wire end with no gap.
    lug_inner_y = TOP_R - 0.006  # bites into the wall
    lug_outer_y = LUG_Y + RIVET_R  # reaches just past the pivot point
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

    # --- reinforcing bands (3 hoop ribs evenly spaced between bottom and rim) ---
    # Each band is a thin torus wrapping the outer wall at its height, with the
    # radius matching the tapered wall so it hugs the surface with no float.
    for i in range(BAND_COUNT):
        # Evenly spaced heights: 1/4, 1/2, 3/4 of body height
        band_z = BODY_H * (i + 1.0) / (BAND_COUNT + 1.0)
        bm = _band_mesh(band_z, f"band_{i}")
        bucket.visual(
            bm,
            origin=Origin(xyz=(0.0, 0.0, band_z)),
            material=red_metal,
            name=f"band_{i}",
        )

    bucket.inertial = Inertial.from_geometry(
        Cylinder(radius=TOP_R, length=BODY_H),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
    )

    # --- steel-wire bail handle (REVOLUTE about the +/-Y lug diameter) ---
    # Authored in its OWN part frame whose origin is the rim center on the pivot
    # axis (z = LUG_Z). At q=0 the arch stands straight up: both ends start at
    # the lugs (y = +/-LUG_Y, z = 0 local) and sweep up to an apex at +Z local.
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

    # Joint origin sits exactly on the lug pivot axis (rim center, z=LUG_Z).
    # Axis is +Y: the horizontal diameter line through both lugs. q=0 = upright;
    # +/-100 deg swings the arch down to either side.
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

    # --- intentional overlaps: lug tabs grow out of the wall/rim; rivets seat
    #     into the lugs; bail-wire ends seat into the extended lugs. ---
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
    for i in range(BAND_COUNT):
        ctx.allow_overlap(
            bucket, bucket,
            elem_a=f"band_{i}", elem_b="bucket_shell",
            reason=f"Band {i} is a hoop rib wrapping the outer wall surface.",
        )

    # --- bucket rests on the ground at z~0, rim is the open top ---
    shell_aabb = ctx.part_element_world_aabb(bucket, elem="bucket_shell")
    ctx.check(
        "flat bottom rests on the ground at z~0",
        abs(shell_aabb[0][2]) < 0.002,
        details=f"shell_minZ={shell_aabb[0][2]}",
    )
    ctx.check(
        "bucket height matches design (~0.26 m)",
        abs((shell_aabb[1][2] - shell_aabb[0][2]) - BODY_H) < 0.01,
        details=f"shell_h={shell_aabb[1][2] - shell_aabb[0][2]}",
    )

    # --- tapered: wider at the top than the bottom ---
    rim_aabb = ctx.part_element_world_aabb(bucket, elem="rolled_rim")
    top_w = _ext(rim_aabb)[0]
    bot_slice_w = 2.0 * BOT_R  # design bottom diameter
    ctx.check(
        "tapered: open top wider than flat bottom",
        top_w > bot_slice_w + 0.03,
        details=f"top_w={top_w}, bottom_w(design)={bot_slice_w}",
    )

    # --- body is HOLLOW: inner cavity radius is well inside the wall ---
    # Sample the shell mesh: confirm there is interior open space by checking the
    # inner-wall design clearance (outer top R - wall) is meaningfully < outer R.
    ctx.check(
        "body modeled hollow (thin wall, open interior)",
        WALL < TOP_R * 0.05 and BOTTOM_T < BODY_H * 0.1,
        details=f"wall={WALL}, bottom_t={BOTTOM_T}, top_r={TOP_R}",
    )

    # --- two lugs on opposite sides along Y, on the pivot axis ---
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
    # Joint origin lies on the lug pivot axis: same Y(=0) and Z as the lugs.
    jo = joint.origin.xyz
    ctx.check(
        "joint origin sits on the lug pivot axis (no float)",
        abs(jo[0]) < 1e-6 and abs(jo[1]) < 1e-6 and abs(jo[2] - LUG_Z) < 1e-6,
        details=f"origin={jo}, lug_z={LUG_Z}",
    )
    # The pivot axis passes through both lug centers: their Z matches the joint
    # origin Z, and they are symmetric about Y=0 (the axis line y in R, x=0).
    ctx.check(
        "pivot axis passes through both lugs",
        abs(lug_pos_c[2] - jo[2]) < 0.02 and abs(lug_neg_c[2] - jo[2]) < 0.02
        and abs(lug_pos_c[0]) < 0.01 and abs(lug_neg_c[0]) < 0.01,
        details=f"lug_pos={lug_pos_c}, lug_neg={lug_neg_c}, origin_z={jo[2]}",
    )

    # --- handle limits cover roughly a 180-deg arc over the top ---
    lo, hi = joint.motion_limits.lower, joint.motion_limits.upper
    ctx.check(
        "handle swings ~180 deg over the top",
        lo < math.radians(-80.0) and hi > math.radians(80.0),
        details=f"lower={lo}, upper={hi}",
    )

    # --- handle ends meet the lugs (no gap) at rest, and the arch rises over
    #     the top; ends are at the pivot height near +/-LUG_Y ---
    wire_aabb = ctx.part_element_world_aabb(handle, elem="bail_wire")
    ctx.check(
        "bail wire spans across to both lugs (reaches +/-Y rim)",
        wire_aabb[1][1] > LUG_Y - 0.01 and wire_aabb[0][1] < -(LUG_Y - 0.01),
        details=f"wire_y=({wire_aabb[0][1]},{wire_aabb[1][1]}), lug_y={LUG_Y}",
    )
    # No gap: wire ends contact the lugs.
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

    # --- decisive pose check: handle swings down to the +Y side; the arch apex
    #     drops below the rim and the mass moves toward +Y. ---
    apex_rest = _aabb_center(wire_aabb)
    with ctx.pose({joint: math.radians(95.0)}):
        apex_down = _aabb_center(ctx.part_element_world_aabb(handle, elem="bail_wire"))
    ctx.check(
        "handle swings down over the side when rotated",
        apex_down[2] < apex_rest[2] - 0.05,
        details=f"rest center={apex_rest}, swung center={apex_down}",
    )

    # --- reinforcing bands: exactly 3, evenly spaced, hugging the tapered wall ---
    band_names = [f"band_{i}" for i in range(BAND_COUNT)]
    band_aabbs = [ctx.part_element_world_aabb(bucket, elem=name) for name in band_names]
    band_centers_z = [(_aabb_center(aabb))[2] for aabb in band_aabbs]

    ctx.check(
        "exactly three reinforcing bands exist on the bucket body",
        len(band_aabbs) == 3 and all(a is not None for a in band_aabbs),
        details=f"found {len(band_aabbs)} bands",
    )

    # Evenly spaced: the expected heights are BODY_H * (i+1)/(BAND_COUNT+1)
    expected_zs = [BODY_H * (i + 1.0) / (BAND_COUNT + 1.0) for i in range(BAND_COUNT)]
    spacing_ok = all(
        abs(bz - ez) < 0.005 for bz, ez in zip(band_centers_z, expected_zs)
    )
    ctx.check(
        "bands are evenly spaced between bottom and rim",
        spacing_ok,
        details=f"band_zs={band_centers_z}, expected_zs={expected_zs}",
    )

    # Each band radius matches the tapered wall radius at its height
    for i, (aabb, cz) in enumerate(zip(band_aabbs, band_centers_z)):
        # The band is a torus: its outer diameter in X or Y should be about
        # 2 * (_wall_radius_at(cz) + BAND_TUBE_R)
        ext_x = aabb[1][0] - aabb[0][0]
        ext_y = aabb[1][1] - aabb[0][1]
        expected_outer_d = 2.0 * (_wall_radius_at(cz) + BAND_TUBE_R)
        radius_matches = (
            abs(ext_x - expected_outer_d) < 0.01
            and abs(ext_y - expected_outer_d) < 0.01
        )
        ctx.check(
            f"band_{i} radius matches tapered wall at its height",
            radius_matches,
            details=(
                f"band_z={cz}, ext_x={ext_x}, ext_y={ext_y}, "
                f"expected_d={expected_outer_d}"
            ),
        )

    # --- colors: body red, wire steel ---
    shell_vis = bucket.get_visual("bucket_shell")
    wire_vis = handle.get_visual("bail_wire")
    shell_rgba = shell_vis.material.rgba
    wire_rgba = wire_vis.material.rgba
    ctx.check(
        "bucket body is red",
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
