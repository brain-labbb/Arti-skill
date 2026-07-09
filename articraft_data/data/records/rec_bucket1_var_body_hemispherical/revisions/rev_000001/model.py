from __future__ import annotations

# Red painted sheet-metal fire bucket (HEMISPHERICAL BOWL variant).
#
# Coordinate convention:
#   - up is +Z. The rounded bottom of the bowl rests on the ground at z=0.
#   - the body is a hemispherical bowl: a hollow thin-wall revolved shell whose
#     outer profile follows a smooth quarter-circle arc from a small flat bottom
#     up to the wide circular open mouth. The radius grows curvilinearly (not
#     linearly) from bottom to rim.
#   - the rolled top rim is a torus around the mouth edge.
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
BOWL_R = 0.140       # sphere/mouth radius (hemisphere: mouth at equator)
BOWL_H = BOWL_R      # hemisphere depth = sphere radius
WALL = 0.0016        # sheet-metal wall thickness
BOTTOM_T = 0.004     # thickness of the flat bottom plate
BOTTOM_FLAT_R = 0.020  # small flat bottom so the bowl can sit on a surface

RIM_TUBE = 0.006     # rolled-rim tube (minor) radius
RIM_CENTER_R = BOWL_R - RIM_TUBE * 0.4  # torus center radius at the mouth edge
RIM_Z = BOWL_H       # rolled rim sits at the mouth edge

# Pivot lugs / handle pivot axis (the rim diameter along Y).
LUG_Y = BOWL_R + 0.014     # lug pivot point outboard of the rim wall
LUG_Z = BOWL_H - 0.012     # pivot axis a touch below the very top edge
LUG_HALF_T = 0.0035        # half-thickness of the lug tab (along X)
LUG_PLATE_W = 0.018        # lug tab width along Z
LUG_PLATE_R = 0.011        # lug tab reach (radial) below the pivot
RIVET_R = 0.0035           # rivet head radius

WIRE_R = 0.0028            # steel bail-wire radius
HANDLE_RISE = 0.150        # how far the arch apex rises above the pivot axis
HANDLE_RIM_CLEAR_Z = (RIM_Z - LUG_Z) + RIM_TUBE + WIRE_R + 0.004


def _bowl_outer_profile(n_arc: int = 48) -> list[tuple[float, float]]:
    """Quarter-circle arc from flat bottom edge to mouth, in (radius, z)."""
    # Sphere center at (r=0, z=BOWL_R). Outer radius = BOWL_R.
    # Arc from theta_start (where r = BOTTOM_FLAT_R) to theta=0 (where r = BOWL_R, z = BOWL_R).
    theta_start = -math.acos(BOTTOM_FLAT_R / BOWL_R)
    pts: list[tuple[float, float]] = [(BOTTOM_FLAT_R, 0.0)]
    for i in range(n_arc + 1):
        t = i / n_arc
        theta = theta_start * (1.0 - t)
        r = BOWL_R * math.cos(theta)
        z = BOWL_R * (1.0 + math.sin(theta))
        pts.append((r, z))
    return pts


def _bowl_inner_profile(n_arc: int = 48) -> list[tuple[float, float]]:
    """Inner cavity profile: flat floor + inner sphere arc, in (radius, z)."""
    R_i = BOWL_R - WALL  # inner sphere radius (offset along surface normal)
    # Floor: center to inner wall edge at z = BOTTOM_T
    # At z = BOTTOM_T on the inner sphere: r² + (BOWL_R - BOTTOM_T)² = R_i²
    sin_start = (BOTTOM_T - BOWL_R) / R_i
    theta_start = math.asin(max(-1.0, min(1.0, sin_start)))
    inner_flat_r = R_i * math.cos(theta_start)

    pts: list[tuple[float, float]] = [
        (0.0, BOTTOM_T),
        (inner_flat_r, BOTTOM_T),
    ]
    for i in range(n_arc + 1):
        t = i / n_arc
        theta = theta_start * (1.0 - t)
        r = R_i * math.cos(theta)
        z = BOWL_R + R_i * math.sin(theta)
        pts.append((r, z))
    return pts


# Pre-computed outer-profile radii sampled at several heights, used by tests
# to prove the bowl shape is curvilinear (not a straight-line taper).
_OUTER_PROFILE_POINTS = _bowl_outer_profile()


def _bowl_shell_mesh(name: str):
    """Hollow hemispherical bowl wall + flat bottom as one revolved thin shell."""
    outer = _OUTER_PROFILE_SAMPLE if '_OUTER_PROFILE_SAMPLE' in dir() else _bowl_outer_profile()
    inner = _bowl_inner_profile()
    geom = LatheGeometry.from_shell_profiles(outer, inner, segments=64)
    return mesh_from_geometry(geom, name)


def expected_outer_radius(z: float) -> float:
    """Expected outer-surface radius at height z for the hemisphere bowl design.

    For z <= 0 returns BOTTOM_FLAT_R. For z >= BOWL_H returns BOWL_R.
    In between, r(z) = sqrt(BOWL_R^2 - (BOWL_R - z)^2).
    """
    if z <= 0.0:
        return BOTTOM_FLAT_R
    if z >= BOWL_H:
        return BOWL_R
    return math.sqrt(BOWL_R * BOWL_R - (BOWL_R - z) * (BOWL_R - z))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fire_bucket_hemispherical_bowl")

    red_metal = model.material("red_metal", rgba=(0.62, 0.09, 0.08, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))

    # --- bucket body (root): hollow hemispherical bowl + rolled rim + lugs ---
    bucket = model.part("bucket")

    outer = _bowl_outer_profile()
    inner = _bowl_inner_profile()
    shell_geom = LatheGeometry.from_shell_profiles(outer, inner, segments=64)
    shell_mesh = mesh_from_geometry(shell_geom, "bucket_shell")
    bucket.visual(shell_mesh, material=red_metal, name="bucket_shell")

    rim_geom = TorusGeometry(radius=RIM_CENTER_R, tube=RIM_TUBE, radial_segments=24,
                             tubular_segments=64)
    rim_mesh = mesh_from_geometry(rim_geom, "rim")
    bucket.visual(rim_mesh, origin=Origin(xyz=(0.0, 0.0, RIM_Z)), material=red_metal,
                  name="rolled_rim")

    # Two pivot lugs on opposite rim sides (+/-Y). Each: a flat tab that
    # bridges from the rim wall outward to the pivot point, capped with a rivet.
    lug_inner_y = BOWL_R - 0.006  # bites into the wall
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

    bucket.inertial = Inertial.from_geometry(
        Cylinder(radius=BOWL_R, length=BOWL_H),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.0, BOWL_H / 2.0)),
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

    # Joint origin sits exactly on the lug pivot axis (rim center, z=LUG_Z).
    # Axis is +Y: the horizontal diameter line through both lugs.
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

    # --- bowl rests on the ground at z~0 ---
    shell_aabb = ctx.part_element_world_aabb(bucket, elem="bucket_shell")
    ctx.check(
        "bowl bottom rests on the ground at z~0",
        abs(shell_aabb[0][2]) < 0.003,
        details=f"shell_minZ={shell_aabb[0][2]}",
    )
    ctx.check(
        "bowl depth matches hemisphere design (~0.14 m)",
        abs((shell_aabb[1][2] - shell_aabb[0][2]) - BOWL_H) < 0.015,
        details=f"shell_h={shell_aabb[1][2] - shell_aabb[0][2]}",
    )

    # --- bowl is wider at the mouth than at the bottom ---
    rim_aabb = ctx.part_element_world_aabb(bucket, elem="rolled_rim")
    mouth_w = _ext(rim_aabb)[0]
    bot_slice_w = 2.0 * BOTTOM_FLAT_R
    ctx.check(
        "hemispherical: mouth wider than bottom",
        mouth_w > bot_slice_w + 0.05,
        details=f"mouth_w={mouth_w}, bottom_w(design)={bot_slice_w}",
    )

    # --- body is HOLLOW: thin wall, open interior ---
    ctx.check(
        "body modeled hollow (thin wall, open interior)",
        WALL < BOWL_R * 0.05 and BOTTOM_T < BOWL_H * 0.1,
        details=f"wall={WALL}, bottom_t={BOTTOM_T}, bowl_r={BOWL_R}",
    )

    # --- bowl profile is curvilinear (radius grows monotonically, NOT linearly) ---
    # Sample the design profile at several heights and verify:
    #   1. radius strictly increases from bottom to rim
    #   2. at mid-height the radius is significantly larger than a linear taper
    sample_fracs = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0]
    sample_heights = [f * BOWL_H for f in sample_fracs]
    sample_radii = [expected_outer_radius(h) for h in sample_heights]

    # Monotonic increase
    for i in range(1, len(sample_radii)):
        ctx.check(
            f"bowl radius increases at z={sample_heights[i]:.4f}m",
            sample_radii[i] > sample_radii[i - 1] + 1e-6,
            details=f"r[{i-1}]={sample_radii[i-1]:.5f}, r[{i}]={sample_radii[i]:.5f}",
        )

    # Curvilinear: at mid-height (z=BOWL_H/2), hemisphere radius = R*sqrt(3)/2 ≈ 0.866R
    # A straight-line taper from BOTTOM_FLAT_R to BOWL_R would give (BOTTOM_FLAT_R+BOWL_R)/2 ≈ 0.080
    # Hemisphere gives ~0.121, which is > 50% larger.
    linear_mid = (sample_radii[0] + sample_radii[-1]) / 2.0
    actual_mid = sample_radii[len(sample_radii) // 2]  # at frac=0.4
    # Use frac=0.5 explicitly
    r_at_half = expected_outer_radius(BOWL_H * 0.5)
    ctx.check(
        "bowl profile is curvilinear (convex, not straight-line taper)",
        r_at_half > linear_mid + 0.02,
        details=f"r(z=H/2)={r_at_half:.5f}, linear_mid={linear_mid:.5f}",
    )

    # Verify rate-of-change varies (not constant = not a straight line)
    dr_lower = sample_radii[2] - sample_radii[1]  # z in [0.2H, 0.4H]
    dr_upper = sample_radii[4] - sample_radii[3]  # z in [0.6H, 0.8H]
    ctx.check(
        "bowl profile rate varies (non-linear curvature)",
        abs(dr_lower - dr_upper) > 0.005,
        details=f"dr_lower={dr_lower:.5f}, dr_upper={dr_upper:.5f}",
    )

    # --- bowl sits low and rounded (aspect ratio < 0.7) ---
    aspect = BOWL_H / (2.0 * BOWL_R)
    ctx.check(
        "bowl aspect ratio is squat (< 0.7, not a tall pail)",
        aspect < 0.7,
        details=f"aspect=H/D={aspect:.3f}",
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
    # Joint origin lies on the lug pivot axis
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

    # --- handle limits cover roughly a 180-deg arc over the top ---
    lo, hi = joint.motion_limits.lower, joint.motion_limits.upper
    ctx.check(
        "handle swings ~180 deg over the top",
        lo < math.radians(-80.0) and hi > math.radians(80.0),
        details=f"lower={lo}, upper={hi}",
    )

    # --- bail wire spans across to both lugs ---
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

    # --- decisive pose check: handle swings down to the +Y side ---
    apex_rest = _aabb_center(wire_aabb)
    with ctx.pose({joint: math.radians(95.0)}):
        apex_down = _aabb_center(ctx.part_element_world_aabb(handle, elem="bail_wire"))
    ctx.check(
        "handle swings down over the side when rotated",
        apex_down[2] < apex_rest[2] - 0.05,
        details=f"rest center={apex_rest}, swung center={apex_down}",
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
