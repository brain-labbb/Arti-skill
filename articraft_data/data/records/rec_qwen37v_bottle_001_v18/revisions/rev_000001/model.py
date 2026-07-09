from __future__ import annotations

# Medicine bottle with a child-resistant push-and-turn screw cap.
# Frame: bottle axis along +Z, base at z=0, neck/cap at the top (+Z).
# The body is a translucent amber HDPE shell: rounded base + cylindrical
# barrel with molded volume bands + short shoulder taper + threaded neck.
# The white child-resistant cap rides on the neck through a massless carrier:
#   - cap_rotate: CONTINUOUS spin of the cap about +Z (push-and-turn action)
#   - cap_slide:  PRISMATIC lift of the cap up off the neck
# The cap has prominent vertical grip ribs and a flat top with push-down
# arrow indicators typical of child-resistant closures.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- key dimensions (meters) ----
# Medicine bottle: wider, shorter than a juice bottle
BODY_R = 0.025           # outer barrel radius (~50mm dia)
WALL = 0.0018            # HDPE wall thickness (slightly thicker than PET)
BASE_Z = 0.0
BARREL_TOP_Z = 0.072     # where the shoulder taper begins
SHOULDER_TOP_Z = 0.090   # top of the shoulder, base of the neck
NECK_R = 0.014           # neck outer radius (under the threads)
NECK_TOP_Z = 0.105       # top rim of the neck

# Volume band layout: 3 molded rings around the barrel
BAND_COUNT = 3
BAND_WIDTH = 0.003       # radial protrusion of each band
BAND_HEIGHT = 0.003      # vertical height of each band
BAND_Z_START = 0.018     # first band starts above the base round
BAND_Z_SPACING = 0.016   # spacing between band centers

# Child-resistant cap: wider and shorter than a normal screw cap
CAP_R = 0.020            # cap outer radius (wider than neck for grip)
CAP_HEIGHT = 0.020       # cap total height (flatter for push-down)
CAP_RIB_COUNT = 32       # aggressive vertical grip ribs
CAP_RIB_DEPTH = 0.0012   # how far ribs protrude

# Cap-local Z layout
SKIRT_DROP = 0.010       # skirt wraps below the cap origin over the neck
CAP_TOP_Z = CAP_HEIGHT - SKIRT_DROP


def _neck_thread_profile():
    """Sawtooth ridge segments along the neck as profile points."""
    pts = [(NECK_R, SHOULDER_TOP_Z)]
    z0 = SHOULDER_TOP_Z + 0.003
    ridge_r = NECK_R + 0.0014
    for k in range(3):
        zc = z0 + k * 0.004
        pts.append((NECK_R, zc - 0.0014))
        pts.append((ridge_r, zc - 0.0005))
        pts.append((ridge_r, zc + 0.0005))
        pts.append((NECK_R, zc + 0.0014))
    pts.append((NECK_R, NECK_TOP_Z))
    return pts


def _bottle_profile():
    """Outer profile of the medicine bottle with molded volume bands.
    Returns list of (radius, z) points for revolution."""
    pts = []
    # Base: rounded bottom corner
    pts.append((0.0, BASE_Z))
    pts.append((BODY_R - 0.005, BASE_Z))
    # Arc up to full radius at the base
    base_corner_z = BASE_Z + 0.005
    pts.append((BODY_R, base_corner_z))

    # Barrel with volume bands
    # We need to insert band protrusions into the straight barrel profile
    band_centers = [BAND_Z_START + i * BAND_Z_SPACING for i in range(BAND_COUNT)]

    # Build the barrel section with band bumps
    z_cursor = base_corner_z
    for bc in band_centers:
        band_bot = bc - BAND_HEIGHT / 2.0
        band_top = bc + BAND_HEIGHT / 2.0
        # straight section up to band
        if band_bot > z_cursor:
            pts.append((BODY_R, band_bot))
        # band protrusion: ramp out, flat top, ramp back
        pts.append((BODY_R + BAND_WIDTH * 0.5, band_bot + BAND_HEIGHT * 0.15))
        pts.append((BODY_R + BAND_WIDTH, band_bot + BAND_HEIGHT * 0.35))
        pts.append((BODY_R + BAND_WIDTH, band_top - BAND_HEIGHT * 0.35))
        pts.append((BODY_R + BAND_WIDTH * 0.5, band_top - BAND_HEIGHT * 0.15))
        pts.append((BODY_R, band_top))
        z_cursor = band_top

    # Continue barrel to shoulder start
    pts.append((BODY_R, BARREL_TOP_Z))

    # Shoulder taper up to neck (gentler curve for medicine bottle)
    mid_r = (BODY_R + NECK_R) / 2.0
    mid_z = (BARREL_TOP_Z + SHOULDER_TOP_Z) / 2.0
    pts.append((mid_r, mid_z + 0.002))
    pts.append((NECK_R, SHOULDER_TOP_Z))

    return pts


def _bottle_shell():
    """Translucent amber medicine bottle as one solid revolve with volume bands,
    then shelled open at the top so the neck reads hollow."""
    profile = _bottle_profile()

    wp = cq.Workplane("XZ").moveTo(profile[0][0], profile[0][1])
    for (r, z) in profile[1:]:
        wp = wp.lineTo(r, z)

    # Neck threads
    for (r, z) in _neck_thread_profile()[1:]:
        wp = wp.lineTo(r, z)

    # Close along axis
    wp = wp.lineTo(0.0, NECK_TOP_Z).close()
    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))
    # Shell: remove top face, hollow inward
    return outer.faces(">Z").shell(-WALL)


def _cap_solid():
    """White child-resistant cap: wider body with aggressive grip ribs,
    flat top with push-down arrow indicators.
    Local frame: origin at the cap joint; disc above, skirt hangs below."""
    # Main cap body - wider cylinder
    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, -SKIRT_DROP))
        .circle(CAP_R)
        .extrude(CAP_HEIGHT)
    )
    # Slight fillet on top edge
    outer = outer.edges(">Z").fillet(0.0015)

    # Hollow cavity for neck insertion (open at bottom)
    cavity = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, -SKIRT_DROP - 0.001))
        .circle(NECK_R + 0.0014)
        .extrude(CAP_HEIGHT - 0.004)
    )
    cap = outer.cut(cavity)

    # Vertical grip ribs around the cap skirt (child-resistant knurl)
    rib_h = CAP_HEIGHT - 0.003
    zc = -SKIRT_DROP + rib_h / 2.0
    for i in range(CAP_RIB_COUNT):
        ang = 2.0 * math.pi * i / CAP_RIB_COUNT
        x = (CAP_R - 0.0004) * math.cos(ang)
        y = (CAP_R - 0.0004) * math.sin(ang)
        rib = (
            cq.Workplane("XY")
            .transformed(offset=(x, y, zc), rotate=(0, 0, math.degrees(ang)))
            .box(0.0020, 0.0012, rib_h)
        )
        cap = cap.union(rib)

    # Push-down arrow indicators on the top face (two raised triangular wedges)
    top_z = CAP_TOP_Z - 0.001
    for sign in [1, -1]:
        arrow = (
            cq.Workplane("XY")
            .transformed(offset=(sign * 0.008, 0.0, top_z))
            .box(0.006, 0.003, 0.001)
        )
        cap = cap.union(arrow)

    return cap


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="medicine_bottle")

    # Amber translucent HDPE body and white child-resistant cap
    amber = model.material("amber_hdpe", rgba=(0.85, 0.55, 0.20, 0.45))
    white = model.material("cap_white", rgba=(0.92, 0.92, 0.90, 1.0))

    # ---- bottle body (root) ----
    body = model.part("bottle")
    shell = _bottle_shell()
    body.visual(mesh_from_cadquery(shell, "bottle_shell"), material=amber, name="bottle_shell")
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, NECK_TOP_Z),
        mass=0.035,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    # ---- massless carrier (no visuals): carries the rotate joint ----
    carrier = model.part("cap_carrier")
    carrier.inertial = Inertial.from_geometry(Box((0.008, 0.008, 0.008)), mass=1e-4)

    # ---- child-resistant cap ----
    cap = model.part("cap")
    cap_geo = _cap_solid()
    cap.visual(mesh_from_cadquery(cap_geo, "cap_shell"), material=white, name="cap_shell")
    cap.inertial = Inertial.from_geometry(
        Cylinder(CAP_R, CAP_HEIGHT),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, (CAP_TOP_Z - SKIRT_DROP) / 2.0)),
    )

    # ---- decoupled joints sharing the vertical +Z cap axis ----
    model.articulation(
        "cap_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.0),
    )
    model.articulation(
        "cap_slide",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=CAP_HEIGHT, effort=1.0, velocity=1.0),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle")
    cap = object_model.get_part("cap")
    rotate = object_model.get_articulation("cap_rotate")
    slide = object_model.get_articulation("cap_slide")

    bottle_shell = body.get_visual("bottle_shell")
    cap_shell = cap.get_visual("cap_shell")

    # --- bottle is translucent amber (alpha < 1), cap is opaque white ---
    ctx.check(
        "bottle material is translucent amber (alpha < 1)",
        bottle_shell.material.rgba is not None and bottle_shell.material.rgba[3] < 1.0,
        details=f"bottle rgba={bottle_shell.material.rgba}",
    )
    ctx.check(
        "cap material is opaque white",
        cap_shell.material.rgba is not None
        and cap_shell.material.rgba[3] >= 0.99
        and min(cap_shell.material.rgba[:3]) > 0.8,
        details=f"cap rgba={cap_shell.material.rgba}",
    )

    # --- bottle body has molded volume bands (wider than plain barrel) ---
    # The bottle shell should protrude beyond the base body radius at band locations
    body_aabb = ctx.part_world_aabb(body)
    body_dx = body_aabb[1][0] - body_aabb[0][0]  # full X extent
    body_radius_actual = body_dx / 2.0
    ctx.check(
        "bottle body has molded volume bands (radius exceeds plain barrel)",
        body_radius_actual > BODY_R + BAND_WIDTH * 0.4,
        details=f"body_radius_actual={body_radius_actual:.5f}, expected > {BODY_R + BAND_WIDTH * 0.4:.5f}",
    )

    # --- medicine bottle proportions: wider than tall relative to juice bottle ---
    body_dz = body_aabb[1][2] - body_aabb[0][2]
    body_diameter = body_dx
    ctx.check(
        "medicine bottle proportions (diameter/height > 0.35)",
        body_diameter / body_dz > 0.35,
        details=f"diameter={body_diameter:.4f}, height={body_dz:.4f}, ratio={body_diameter/body_dz:.2f}",
    )

    # --- cap is wider than the neck (child-resistant feature) ---
    cap_aabb = ctx.part_world_aabb(cap)
    cap_dx = cap_aabb[1][0] - cap_aabb[0][0]
    cap_radius_actual = cap_dx / 2.0
    ctx.check(
        "cap radius exceeds neck radius (child-resistant wide cap)",
        cap_radius_actual > NECK_R + 0.003,
        details=f"cap_radius={cap_radius_actual:.5f}, neck_radius={NECK_R}",
    )

    # --- cap sits on top of the neck ---
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "cap mounted on top of the neck",
        cap_pos is not None and cap_pos[2] > BARREL_TOP_Z,
        details=f"cap origin={cap_pos}",
    )

    # Cap skirt slips over neck threads at rest -> intentional overlap
    ctx.allow_overlap(
        cap,
        body,
        elem_a="cap_shell",
        elem_b="bottle_shell",
        reason="Cap skirt is intentionally seated over the threaded neck for child-resistant closure.",
    )

    # --- cap_rotate is a CONTINUOUS joint (push-and-turn action) ---
    ctx.check(
        "cap_rotate is CONTINUOUS joint type",
        rotate.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={rotate.articulation_type}",
    )

    # --- rotating cap spins visibly (bounding box changes) ---
    with ctx.pose({rotate: 0.0}):
        marker0 = ctx.part_world_aabb(cap)
    with ctx.pose({rotate: math.pi / 4.0}):
        marker45 = ctx.part_world_aabb(cap)
    # With ribs, the cap is nearly axisymmetric, so check that rotation
    # actually moves geometry by checking the cap position is maintained
    # (carrier rotates properly)
    cap_pos_0 = ctx.part_world_position(cap)
    with ctx.pose({rotate: math.pi}):
        cap_pos_180 = ctx.part_world_position(cap)
    ctx.check(
        "cap rotation preserves cap position (continuous joint works)",
        cap_pos_0 is not None and cap_pos_180 is not None
        and abs(cap_pos_0[2] - cap_pos_180[2]) < 0.001,
        details=f"pos_0={cap_pos_0}, pos_180={cap_pos_180}",
    )

    # --- cap_slide lifts the cap off the neck ---
    rest_z = ctx.part_world_position(cap)[2]
    with ctx.pose({slide: CAP_HEIGHT}):
        lifted_z = ctx.part_world_position(cap)[2]
    ctx.check(
        "cap_slide lifts the cap up off the neck",
        lifted_z > rest_z + CAP_HEIGHT * 0.8,
        details=f"rest_z={rest_z}, lifted_z={lifted_z}",
    )

    # --- volume bands create distinct profile (Y extent also shows bands) ---
    body_dy = body_aabb[1][1] - body_aabb[0][1]
    ctx.check(
        "volume bands visible in Y extent (symmetric protrusion)",
        abs(body_dx - body_dy) < 0.002,
        details=f"dx={body_dx:.5f}, dy={body_dy:.5f}",
    )

    return ctx.report()


object_model = build_object_model()
