from __future__ import annotations

# Red painted sheet-metal fire bucket with wall-mounting bracket.
#
# Coordinate convention:
#   - up is +Z. The flat bottom of the bucket rests on the ground at z=0.
#   - the bucket is a tapered cylinder, wider at the open top, narrower at the
#     flat bottom, modeled as a hollow thin-wall revolved shell.
#   - the rolled top rim is a torus around the top edge.
#   - two riveted pivot lugs sit just outside opposite sides of the rim along +/-Y.
#   - the steel-wire bail handle pivots about the +/-Y diameter line through the
#     two lugs (a REVOLUTE joint).
#   - a wall-mounting bracket (root) is fixed to a vertical wall at +X. It has a
#     flat steel back-plate with two bolt holes and a curved cradle ring/arm that
#     hugs the bucket body at ~38% height. The bucket is rigidly held in the
#     cradle via a FIXED joint.

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

# --- bucket dimensions (meters) ---
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

# --- bracket dimensions ---
CRADLE_Z_FRAC = 0.38  # cradle height as fraction of bucket height
CRADLE_Z = BODY_H * CRADLE_Z_FRAC
BUCKET_R_AT_CRADLE = BOT_R + (TOP_R - BOT_R) * CRADLE_Z_FRAC
CRADLE_TUBE_R = 0.005  # cradle ring rod radius
# Ring centerline slightly outside bucket wall so the tube inner surface hugs it.
CRADLE_RING_R = BUCKET_R_AT_CRADLE + CRADLE_TUBE_R * 0.4

PLATE_T = 0.005  # back-plate thickness (X)
PLATE_X = 0.20  # back-plate center X
PLATE_W_Y = 0.14  # back-plate width (Y)
PLATE_H_Z = 0.22  # back-plate height (Z)
PLATE_Z_CENTER = BODY_H * 0.45  # back-plate vertical center

BOLT_R = 0.004  # bolt-hole radius
BOLT_SPACING_Z = 0.12  # vertical spacing between bolt holes

# Cradle arc: 340-degree wrap open at +X for the arm connection.
CRADLE_ARC_START_DEG = 10.0
CRADLE_ARC_END_DEG = 350.0
ARM_START_X = CRADLE_RING_R * math.cos(math.radians(CRADLE_ARC_START_DEG))
ARM_LENGTH = PLATE_X - ARM_START_X
ARM_CENTER_X = ARM_START_X + ARM_LENGTH / 2.0
ARM_W_Y = 0.060  # arm width (Y) — wide enough to meet ring endpoints
ARM_T_Z = 0.006  # arm thickness (Z)


def _revolved_shell_mesh(
    top_r: float,
    bot_r: float,
    height: float,
    wall: float,
    bottom_t: float,
    name: str,
):
    """Hollow tapered bucket wall + flat bottom as one revolved thin shell."""
    outer = [
        (bot_r, 0.0),
        (bot_r + (top_r - bot_r) * 0.5, height * 0.5),
        (top_r, height),
    ]
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


def _cradle_ring_mesh(ring_r: float, tube_r: float, z: float,
                      arc_start_deg: float, arc_end_deg: float,
                      n_pts: int = 21) -> object:
    """Partial-arc cradle ring as a tube through spline points."""
    pts = []
    for i in range(n_pts):
        a = math.radians(arc_start_deg +
                         i * (arc_end_deg - arc_start_deg) / (n_pts - 1))
        pts.append((ring_r * math.cos(a), ring_r * math.sin(a), z))
    return mesh_from_geometry(
        tube_from_spline_points(
            pts, radius=tube_r, samples_per_segment=8,
            radial_segments=12, cap_ends=True,
        ),
        "cradle_ring",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fire_bucket_wall_mount")

    red_metal = model.material("red_metal", rgba=(0.62, 0.09, 0.08, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.15, 0.15, 0.18, 1.0))

    # ================================================================
    # BRACKET (root) — flat back-plate, bolt holes, cradle ring + arm
    # ================================================================
    bracket = model.part("bracket")

    # Flat steel back-plate against the wall at +X.
    bracket.visual(
        Box((PLATE_T, PLATE_W_Y, PLATE_H_Z)),
        origin=Origin(xyz=(PLATE_X, 0.0, PLATE_Z_CENTER)),
        material=steel,
        name="back_plate",
    )

    # Two bolt holes through the plate (dark recessed cylinders).
    for i in range(2):
        bolt_z = PLATE_Z_CENTER + (i - 0.5) * BOLT_SPACING_Z
        bracket.visual(
            Cylinder(radius=BOLT_R, length=PLATE_T + 0.004),
            origin=Origin(xyz=(PLATE_X, 0.0, bolt_z),
                          rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_steel,
            name=f"bolt_hole_{i}",
        )

    # Curved cradle ring — partial arc hugging the bucket body.
    bracket.visual(
        _cradle_ring_mesh(CRADLE_RING_R, CRADLE_TUBE_R, CRADLE_Z,
                          CRADLE_ARC_START_DEG, CRADLE_ARC_END_DEG),
        material=steel,
        name="cradle_ring",
    )

    # Cradle arm — flat bar connecting ring gap to back-plate.
    bracket.visual(
        Box((ARM_LENGTH, ARM_W_Y, ARM_T_Z)),
        origin=Origin(xyz=(ARM_CENTER_X, 0.0, CRADLE_Z)),
        material=steel,
        name="cradle_arm",
    )

    bracket.inertial = Inertial.from_geometry(
        Cylinder(radius=0.12, length=PLATE_H_Z),
        mass=1.5,
        origin=Origin(xyz=(PLATE_X / 2.0, 0.0, PLATE_Z_CENTER)),
    )

    # ================================================================
    # BUCKET BODY — hollow tapered shell + rolled rim + lugs
    # ================================================================
    bucket = model.part("bucket")

    shell_mesh = _revolved_shell_mesh(TOP_R, BOT_R, BODY_H, WALL, BOTTOM_T,
                                      "bucket_shell")
    bucket.visual(shell_mesh, material=red_metal, name="bucket_shell")

    rim_geom = TorusGeometry(radius=RIM_CENTER_R, tube=RIM_TUBE,
                             radial_segments=24, tubular_segments=64)
    rim_mesh = mesh_from_geometry(rim_geom, "rim")
    bucket.visual(rim_mesh, origin=Origin(xyz=(0.0, 0.0, RIM_Z)),
                  material=red_metal, name="rolled_rim")

    # Two pivot lugs on opposite rim sides (+/-Y).
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
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
    )

    # ================================================================
    # STEEL-WIRE BAIL HANDLE (REVOLUTE about the +/-Y lug diameter)
    # ================================================================
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

    # ================================================================
    # ARTICULATIONS
    # ================================================================

    # Bracket (root, wall-fixed) → Bucket (rigidly held in cradle).
    model.articulation(
        "bracket_to_bucket",
        ArticulationType.FIXED,
        parent=bracket,
        child=bucket,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # Bucket → Handle: REVOLUTE about the +/-Y lug diameter line.
    # Axis is +Y; q=0 = upright arch; +/-100 deg swings down either side.
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


# ---------------------------------------------------------------------------
# Helpers for tests
# ---------------------------------------------------------------------------

def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def _aabb_center(aabb):
    mn, mx = aabb
    return ((mn[0] + mx[0]) / 2.0, (mn[1] + mx[1]) / 2.0, (mn[2] + mx[2]) / 2.0)


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bracket = object_model.get_part("bracket")
    bucket = object_model.get_part("bucket")
    handle = object_model.get_part("handle")
    handle_joint = object_model.get_articulation("bucket_to_handle")

    # ================================================================
    # Intentional overlap allowances
    # ================================================================

    # Lug tabs riveted onto the rolled rim/wall.
    ctx.allow_overlap(bucket, bucket, elem_a="lug_pos", elem_b="rolled_rim",
                      reason="Lug tab is riveted onto the rolled rim/wall.")
    ctx.allow_overlap(bucket, bucket, elem_a="lug_neg", elem_b="rolled_rim",
                      reason="Lug tab is riveted onto the rolled rim/wall.")
    # Rivet heads set through the lug tabs.
    ctx.allow_overlap(bucket, bucket, elem_a="rivet_pos", elem_b="lug_pos",
                      reason="Rivet head is set through the lug tab.")
    ctx.allow_overlap(bucket, bucket, elem_a="rivet_neg", elem_b="lug_neg",
                      reason="Rivet head is set through the lug tab.")
    # Bail-wire ends drop into the pivot lugs.
    ctx.allow_overlap(handle, bucket, elem_a="bail_wire", elem_b="lug_pos",
                      reason="Bail-wire end drops into the +Y pivot lug.")
    ctx.allow_overlap(handle, bucket, elem_a="bail_wire", elem_b="lug_neg",
                      reason="Bail-wire end drops into the -Y pivot lug.")

    # Bracket: cradle ring hugs the bucket wall (intentional nesting).
    ctx.allow_overlap(bracket, bucket, elem_a="cradle_ring", elem_b="bucket_shell",
                      reason="Cradle ring is shaped to hug the bucket outer wall.")
    # Bracket: cradle arm connects ring to plate, passing near the bucket wall.
    ctx.allow_overlap(bracket, bucket, elem_a="cradle_arm", elem_b="bucket_shell",
                      reason="Cradle arm connects the cradle ring to the back-plate, "
                             "touching the bucket wall at the ring connection point.")
    # Bracket: arm connects to ring and plate (structural overlap at joints).
    ctx.allow_overlap(bracket, bracket, elem_a="cradle_arm", elem_b="cradle_ring",
                      reason="Cradle arm is welded to the cradle ring.")
    ctx.allow_overlap(bracket, bracket, elem_a="cradle_arm", elem_b="back_plate",
                      reason="Cradle arm is bolted to the back-plate.")
    # Bolt holes pass through the back-plate.
    for i in range(2):
        ctx.allow_overlap(bracket, bracket,
                          elem_a=f"bolt_hole_{i}", elem_b="back_plate",
                          reason="Bolt hole passes through the back-plate.")

    # ================================================================
    # BRACKET TESTS
    # ================================================================

    # --- back-plate is a flat vertical plate offset to +X side ---
    plate_aabb = ctx.part_element_world_aabb(bracket, elem="back_plate")
    plate_ext = _ext(plate_aabb)
    ctx.check(
        "back-plate is flat (thin in X)",
        plate_ext[0] < 0.010,
        details=f"plate_dx={plate_ext[0]}",
    )
    ctx.check(
        "back-plate is vertical (tall in Z, wide in Y)",
        plate_ext[2] > 0.10 and plate_ext[1] > 0.08,
        details=f"plate_dy={plate_ext[1]}, plate_dz={plate_ext[2]}",
    )
    ctx.check(
        "back-plate is offset to +X side of bucket",
        plate_aabb[0][0] > BUCKET_R_AT_CRADLE + 0.02,
        details=f"plate_min_x={plate_aabb[0][0]}, bucket_r={BUCKET_R_AT_CRADLE}",
    )

    # --- two bolt holes on the back-plate ---
    for i in range(2):
        bolt_vis = bracket.get_visual(f"bolt_hole_{i}")
        ctx.check(
            f"bolt_hole_{i} exists on bracket",
            bolt_vis is not None,
            details=f"visual bolt_hole_{i} not found",
        )
    bolt_0_aabb = ctx.part_element_world_aabb(bracket, elem="bolt_hole_0")
    bolt_1_aabb = ctx.part_element_world_aabb(bracket, elem="bolt_hole_1")
    ctx.check(
        "bolt holes are vertically separated",
        abs(_aabb_center(bolt_0_aabb)[2] - _aabb_center(bolt_1_aabb)[2]) > 0.06,
        details=f"bolt0_z={_aabb_center(bolt_0_aabb)[2]}, "
                f"bolt1_z={_aabb_center(bolt_1_aabb)[2]}",
    )
    ctx.check(
        "bolt holes lie on the back-plate X face",
        bolt_0_aabb[0][0] > BUCKET_R_AT_CRADLE and bolt_1_aabb[0][0] > BUCKET_R_AT_CRADLE,
        details=f"bolt0_x={bolt_0_aabb[0][0]}, bolt1_x={bolt_1_aabb[0][0]}",
    )

    # --- cradle ring encircles the bucket body ---
    ring_aabb = ctx.part_element_world_aabb(bracket, elem="cradle_ring")
    ctx.check(
        "cradle ring spans both +/-Y sides of bucket (encircles)",
        ring_aabb[0][1] < -BUCKET_R_AT_CRADLE * 0.5
        and ring_aabb[1][1] > BUCKET_R_AT_CRADLE * 0.5,
        details=f"ring_y=({ring_aabb[0][1]},{ring_aabb[1][1]}), "
                f"bucket_r={BUCKET_R_AT_CRADLE}",
    )
    ctx.check(
        "cradle ring reaches around the -X side of bucket",
        ring_aabb[0][0] < -BUCKET_R_AT_CRADLE * 0.5,
        details=f"ring_min_x={ring_aabb[0][0]}",
    )
    ctx.check(
        "cradle ring is at bucket mid-lower height",
        abs(_aabb_center(ring_aabb)[2] - CRADLE_Z) < 0.02,
        details=f"ring_center_z={_aabb_center(ring_aabb)[2]}, cradle_z={CRADLE_Z}",
    )

    # --- cradle ring contacts the bucket wall ---
    ctx.expect_contact(bracket, bucket,
                       elem_a="cradle_ring", elem_b="bucket_shell",
                       contact_tol=0.008,
                       name="cradle ring contacts bucket wall")

    # --- cradle arm bridges ring to plate ---
    arm_aabb = ctx.part_element_world_aabb(bracket, elem="cradle_arm")
    ctx.check(
        "cradle arm spans from ring region to plate region",
        arm_aabb[0][0] < CRADLE_RING_R + 0.02
        and arm_aabb[1][0] > PLATE_X - PLATE_T - 0.01,
        details=f"arm_x=({arm_aabb[0][0]},{arm_aabb[1][0]}), "
                f"ring_r={CRADLE_RING_R}, plate_x={PLATE_X}",
    )

    # ================================================================
    # BUCKET TESTS (preserved from parent)
    # ================================================================

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

    # Tapered: wider at the top than the bottom.
    rim_aabb = ctx.part_element_world_aabb(bucket, elem="rolled_rim")
    top_w = _ext(rim_aabb)[0]
    bot_slice_w = 2.0 * BOT_R
    ctx.check(
        "tapered: open top wider than flat bottom",
        top_w > bot_slice_w + 0.03,
        details=f"top_w={top_w}, bottom_w(design)={bot_slice_w}",
    )

    # Body is hollow.
    ctx.check(
        "body modeled hollow (thin wall, open interior)",
        WALL < TOP_R * 0.05 and BOTTOM_T < BODY_H * 0.1,
        details=f"wall={WALL}, bottom_t={BOTTOM_T}, top_r={TOP_R}",
    )

    # Two lugs on opposite sides along Y.
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

    # ================================================================
    # HANDLE TESTS (preserved from parent)
    # ================================================================

    # Handle joint is REVOLUTE about horizontal Y.
    ctx.check(
        "handle joint is revolute",
        str(handle_joint.articulation_type).upper().endswith("REVOLUTE"),
        details=f"type={handle_joint.articulation_type}",
    )
    ax = handle_joint.axis
    ctx.check(
        "handle axis is the horizontal Y diameter line",
        abs(ax[1]) > 0.99 and abs(ax[0]) < 0.02 and abs(ax[2]) < 0.02,
        details=f"axis={ax}",
    )
    jo = handle_joint.origin.xyz
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

    # Handle swings ~180 deg over the top.
    lo, hi = handle_joint.motion_limits.lower, handle_joint.motion_limits.upper
    ctx.check(
        "handle swings ~180 deg over the top",
        lo < math.radians(-80.0) and hi > math.radians(80.0),
        details=f"lower={lo}, upper={hi}",
    )

    # Bail wire spans to both lugs.
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

    # Decisive pose check: handle swings down to the side.
    apex_rest = _aabb_center(wire_aabb)
    with ctx.pose({handle_joint: math.radians(95.0)}):
        apex_down = _aabb_center(
            ctx.part_element_world_aabb(handle, elem="bail_wire"))
    ctx.check(
        "handle swings down over the side when rotated",
        apex_down[2] < apex_rest[2] - 0.05,
        details=f"rest center={apex_rest}, swung center={apex_down}",
    )

    # ================================================================
    # COLOR TESTS
    # ================================================================
    shell_vis = bucket.get_visual("bucket_shell")
    wire_vis = handle.get_visual("bail_wire")
    plate_vis = bracket.get_visual("back_plate")
    shell_rgba = shell_vis.material.rgba
    wire_rgba = wire_vis.material.rgba
    plate_rgba = plate_vis.material.rgba
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
    ctx.check(
        "bracket back-plate is steel-gray",
        min(plate_rgba[:3]) > 0.55 and max(plate_rgba[:3]) - min(plate_rgba[:3]) < 0.2,
        details=f"plate_rgba={plate_rgba}",
    )

    return ctx.report()


object_model = build_object_model()
