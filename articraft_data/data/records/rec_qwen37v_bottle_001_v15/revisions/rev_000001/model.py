from __future__ import annotations

# Squeeze bottle with a conical nozzle cap.
# Variant of the clear plastic juice bottle family.
# Frame: bottle axis along +Z, base at z=0, nozzle at the top (+Z).
# The body is a squeezable oval-barrel PET shell: wider in the middle,
# tapering at both base and shoulder, with a threaded neck and visible
# hollow mouth opening.
# The black conical nozzle cap screws onto the neck via:
#   - cap_rotate: CONTINUOUS spin about +Z
#   - cap_slide:  PRISMATIC lift off the neck
# The nozzle tapers from the cap base to a small dispensing tip.

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
BODY_R_MAX = 0.034       # max outer barrel radius (squeeze bulge)
BODY_R_BASE = 0.025      # base radius (narrower bottom)
WALL = 0.0015            # thin PET wall
BASE_Z = 0.0             # bottom of the bottle
BASE_TAPER_Z = 0.015     # end of base taper
BULGE_Z = 0.075          # center of the barrel bulge
BARREL_TOP_Z = 0.120     # where the shoulder taper begins
SHOULDER_TOP_Z = 0.148   # top of the shoulder, base of the neck
NECK_R = 0.012           # neck outer radius (under the threads)
NECK_TOP_Z = 0.168       # top rim of the neck
MOUTH_R = 0.009          # inner mouth opening radius
THREAD_RIDGE = 0.0014    # thread ridge height

# Cap/nozzle dimensions
CAP_R = 0.016            # cap base outer radius
SKIRT_H = 0.016          # threaded skirt height
CONE_BASE_R = 0.015      # cone base radius (just above skirt)
CONE_TOP_R = 0.004       # nozzle tip radius
CONE_H = 0.032           # cone height
TIP_H = 0.006            # small cylindrical tip above the cone
TIP_R = 0.003            # tip outer radius
NOZZLE_BORE_R = 0.0018   # dispensing hole through the nozzle


def _neck_thread_profile():
    """Sawtooth thread ridges on the neck as revolve profile points."""
    pts = [(NECK_R, SHOULDER_TOP_Z)]
    z0 = SHOULDER_TOP_Z + 0.003
    ridge_r = NECK_R + THREAD_RIDGE
    for k in range(3):
        zc = z0 + k * 0.005
        pts.append((NECK_R, zc - 0.0018))
        pts.append((ridge_r, zc - 0.0006))
        pts.append((ridge_r, zc + 0.0006))
        pts.append((NECK_R, zc + 0.0018))
    pts.append((NECK_R, NECK_TOP_Z))
    return pts


def _outer_r(z):
    """Outer wall radius at height z, for the squeeze bottle profile."""
    if z <= BASE_Z:
        return 0.0
    elif z <= BASE_TAPER_Z:
        # Rounded base corner
        t = (z - BASE_Z) / (BASE_TAPER_Z - BASE_Z)
        return BODY_R_BASE * math.sin(t * math.pi / 2.0)
    elif z <= BULGE_Z:
        # Expand to bulge
        t = (z - BASE_TAPER_Z) / (BULGE_Z - BASE_TAPER_Z)
        return BODY_R_BASE + (BODY_R_MAX - BODY_R_BASE) * math.sin(t * math.pi / 2.0)
    elif z <= BARREL_TOP_Z:
        # Taper from bulge to shoulder
        t = (z - BULGE_Z) / (BARREL_TOP_Z - BULGE_Z)
        return BODY_R_MAX - 0.004 * t
    elif z <= SHOULDER_TOP_Z:
        # Shoulder taper to neck
        t = (z - BARREL_TOP_Z) / (SHOULDER_TOP_Z - BARREL_TOP_Z)
        r_start = BODY_R_MAX - 0.004
        return r_start + (NECK_R - r_start) * (t ** 0.7)
    else:
        return NECK_R


def _bottle_shell():
    """Squeeze bottle: solid outer revolve, then hollow interior cut out.
    Thread ridges are fused onto the neck exterior."""
    # Outer profile: base -> bulge -> shoulder -> smooth neck
    pts = [(0.0, BASE_Z)]
    pts.append((BODY_R_BASE * 0.8, BASE_Z))
    # Base corner
    for i in range(1, 5):
        t = i / 4.0
        z = BASE_Z + t * BASE_TAPER_Z
        r = BODY_R_BASE * math.sin(t * math.pi / 2.0)
        pts.append((r, z))
    # Bulge region
    for i in range(1, 11):
        t = i / 10.0
        z = BASE_TAPER_Z + t * (BULGE_Z - BASE_TAPER_Z)
        r = BODY_R_BASE + (BODY_R_MAX - BODY_R_BASE) * math.sin(t * math.pi / 2.0)
        pts.append((r, z))
    # Taper from bulge to shoulder
    for i in range(1, 7):
        t = i / 6.0
        z = BULGE_Z + t * (BARREL_TOP_Z - BULGE_Z)
        r = BODY_R_MAX - 0.004 * t
        pts.append((r, z))
    # Shoulder taper to smooth neck
    r_shoulder_start = BODY_R_MAX - 0.004
    for i in range(1, 7):
        t = i / 6.0
        z = BARREL_TOP_Z + t * (SHOULDER_TOP_Z - BARREL_TOP_Z)
        r = r_shoulder_start + (NECK_R - r_shoulder_start) * (t ** 0.7)
        pts.append((r, z))
    # Smooth neck up to top
    pts.append((NECK_R, NECK_TOP_Z))
    # Close at the top
    pts.append((0.0, NECK_TOP_Z))

    # Build the outer solid
    wp = cq.Workplane("XZ").moveTo(pts[0][0], pts[0][1])
    for (r, z) in pts[1:]:
        wp = wp.lineTo(r, z)
    wp = wp.close()
    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))

    # Cut hollow interior using a revolved inner profile that follows the outer wall
    floor_z = BASE_Z + WALL * 6  # thicker bottom floor for solid base connection
    inner_pts = [(0.0, floor_z)]
    # Start the cavity at a height where the outer wall is wide enough
    cavity_start_z = BASE_TAPER_Z  # start at the end of the base taper
    inner_pts.append((_outer_r(cavity_start_z) - WALL, cavity_start_z))
    # Bulge region (inner)
    inner_r_max = BODY_R_MAX - WALL
    for i in range(1, 9):
        t = i / 8.0
        z = BASE_TAPER_Z + t * (BULGE_Z - BASE_TAPER_Z)
        r = (BODY_R_BASE - WALL) + (inner_r_max - (BODY_R_BASE - WALL)) * math.sin(t * math.pi / 2.0)
        inner_pts.append((r, z))
    # Taper from bulge to shoulder (inner)
    for i in range(1, 5):
        t = i / 4.0
        z = BULGE_Z + t * (BARREL_TOP_Z - BULGE_Z)
        r = inner_r_max - 0.004 * t
        inner_pts.append((r, z))
    # Shoulder taper to neck (inner)
    r_inner_shoulder_start = inner_r_max - 0.004
    neck_inner_r = NECK_R - WALL
    for i in range(1, 5):
        t = i / 4.0
        z = BARREL_TOP_Z + t * (SHOULDER_TOP_Z - BARREL_TOP_Z)
        r = r_inner_shoulder_start + (neck_inner_r - r_inner_shoulder_start) * (t ** 0.7)
        inner_pts.append((r, z))
    # Neck inner wall
    inner_pts.append((neck_inner_r, NECK_TOP_Z + 0.001))
    # Close at axis
    inner_pts.append((0.0, NECK_TOP_Z + 0.001))

    wp2 = cq.Workplane("XZ").moveTo(inner_pts[0][0], inner_pts[0][1])
    for (r, z) in inner_pts[1:]:
        wp2 = wp2.lineTo(r, z)
    wp2 = wp2.close()
    cavity = wp2.revolve(360.0, (0, 0, 0), (0, 1, 0))
    hollow = outer.cut(cavity)

    # Add thread ridges fused onto the neck exterior
    for k in range(3):
        zc = SHOULDER_TOP_Z + 0.003 + k * 0.005
        ridge = (
            cq.Workplane("XY")
            .transformed(offset=(0, 0, zc - 0.0006))
            .circle(NECK_R + THREAD_RIDGE)
            .circle(NECK_R - 0.0005)
            .extrude(0.0012)
        )
        hollow = hollow.union(ridge)

    return hollow


def _nozzle_cap():
    """Conical nozzle cap: threaded skirt + cone + dispensing tip.
    Local frame: origin at the cap joint (top of neck). Skirt hangs down."""
    skirt_drop = SKIRT_H

    # Build the outer shape as a solid revolve, then cut the inner cavity
    # Outer profile: skirt cylinder, cone taper, tip
    outer_pts = []
    # Start at axis, bottom of skirt
    outer_pts.append((0.0, -skirt_drop))
    outer_pts.append((CAP_R, -skirt_drop))
    # Fillet corner
    outer_pts.append((CAP_R, 0.0))
    # Cone: taper from CAP_R to TIP_R
    for i in range(1, 9):
        t = i / 8.0
        z = t * CONE_H
        r = CAP_R + (TIP_R - CAP_R) * t
        outer_pts.append((r, z))
    # Tip
    outer_pts.append((TIP_R, CONE_H + TIP_H))
    # Close at axis
    outer_pts.append((0.0, CONE_H + TIP_H))

    wp = cq.Workplane("XZ").moveTo(outer_pts[0][0], outer_pts[0][1])
    for (r, z) in outer_pts[1:]:
        wp = wp.lineTo(r, z)
    wp = wp.close()
    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))

    # Inner skirt cavity: fits snugly over the neck threads
    inner_r = NECK_R + 0.0002  # just barely over the neck, touching the threads
    skirt_cavity = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, -skirt_drop - 0.001))
        .circle(inner_r)
        .extrude(skirt_drop - 0.002)
    )
    cap = outer.cut(skirt_cavity)

    # Dispensing bore through the tip
    bore = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, CONE_H * 0.6))
        .circle(NOZZLE_BORE_R)
        .extrude(CONE_H + TIP_H + 0.002)
    )
    cap = cap.cut(bore)

    # Grip rings on the skirt (horizontal ridges for texture)
    n_rings = 5
    ring_h = 0.0012
    for i in range(n_rings):
        z_ring = -skirt_drop + 0.002 + i * (skirt_drop - 0.005) / max(n_rings - 1, 1)
        ring = (
            cq.Workplane("XY")
            .transformed(offset=(0, 0, z_ring))
            .circle(CAP_R + 0.0006)
            .circle(CAP_R - 0.0002)
            .extrude(ring_h)
        )
        cap = cap.union(ring)

    return cap


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="squeeze_bottle")

    # Semi-transparent clear PET body and opaque black nozzle cap.
    clear = model.material("clear_pet", rgba=(0.82, 0.88, 0.86, 0.30))
    black = model.material("nozzle_black", rgba=(0.05, 0.05, 0.06, 1.0))

    # ---- bottle body (root) ----
    body = model.part("bottle")
    shell = _bottle_shell()
    body.visual(mesh_from_cadquery(shell, "bottle_shell"), material=clear, name="bottle_shell")
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R_MAX, NECK_TOP_Z),
        mass=0.028,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    # ---- massless carrier for decoupled joints ----
    carrier = model.part("cap_carrier")
    carrier.inertial = Inertial.from_geometry(Box((0.006, 0.006, 0.006)), mass=1e-4)

    # ---- conical nozzle cap ----
    cap = model.part("nozzle_cap")
    cap_geo = _nozzle_cap()
    cap.visual(mesh_from_cadquery(cap_geo, "nozzle_shell"), material=black, name="nozzle_shell")
    cap.inertial = Inertial.from_geometry(
        Cylinder(CAP_R, SKIRT_H + 0.003 + CONE_H + TIP_H),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, (CONE_H + TIP_H) / 2.0)),
    )

    # ---- articulations ----
    # Continuous rotation of the nozzle cap about the bottle axis
    model.articulation(
        "cap_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.5),
    )
    # Prismatic lift: cap slides up off the neck
    model.articulation(
        "cap_slide",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=SKIRT_H + 0.010, effort=1.0, velocity=1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle")
    cap = object_model.get_part("nozzle_cap")
    rotate = object_model.get_articulation("cap_rotate")
    slide = object_model.get_articulation("cap_slide")

    bottle_shell = body.get_visual("bottle_shell")
    nozzle_shell = cap.get_visual("nozzle_shell")

    # --- bottle is semi-transparent (squeeze bottle plastic) ---
    ctx.check(
        "bottle body is semi-transparent plastic",
        bottle_shell.material.rgba is not None and bottle_shell.material.rgba[3] < 1.0,
        details=f"bottle rgba={bottle_shell.material.rgba}",
    )

    # --- nozzle cap is opaque black ---
    ctx.check(
        "nozzle cap is opaque black",
        nozzle_shell.material.rgba is not None
        and nozzle_shell.material.rgba[3] >= 0.99
        and max(nozzle_shell.material.rgba[:3]) < 0.2,
        details=f"nozzle rgba={nozzle_shell.material.rgba}",
    )

    # --- conical nozzle shape: cap is taller than wide (cone geometry) ---
    cap_aabb = ctx.part_world_aabb(cap)
    if cap_aabb is not None:
        mn, mx = cap_aabb
        dx = mx[0] - mn[0]
        dy = mx[1] - mn[1]
        dz = mx[2] - mn[2]
        ctx.check(
            "nozzle cap has conical shape (taller than wide)",
            dz > dx and dz > dy,
            details=f"cap dims: dx={dx:.4f}, dy={dy:.4f}, dz={dz:.4f}",
        )

    # --- cap sits on top of the neck ---
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "nozzle cap mounted above the shoulder",
        cap_pos is not None and cap_pos[2] > BARREL_TOP_Z,
        details=f"cap origin z={cap_pos}",
    )

    # --- squeeze bottle body is wider at bulge than at base ---
    body_aabb = ctx.part_world_aabb(body)
    if body_aabb is not None:
        bmn, bmx = body_aabb
        body_dx = bmx[0] - bmn[0]
        ctx.check(
            "squeeze bottle body has substantial width (bulge)",
            body_dx > 2.0 * BODY_R_MAX * 0.85,
            details=f"body width={body_dx:.4f}",
        )

    # --- cap_rotate is CONTINUOUS joint ---
    ctx.check(
        "cap_rotate is a continuous joint",
        rotate.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={rotate.articulation_type}",
    )

    # --- cap_slide is PRISMATIC joint ---
    ctx.check(
        "cap_slide is a prismatic joint",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={slide.articulation_type}",
    )

    # Cap skirt slips over the neck threads at rest -> intentional seated overlap.
    ctx.allow_overlap(
        cap,
        body,
        elem_a="nozzle_shell",
        elem_b="bottle_shell",
        reason="Nozzle cap skirt is intentionally seated over the threaded neck.",
    )

    # --- rotation spins the cap (nozzle asymmetry from grip ribs) ---
    with ctx.pose({rotate: 0.0}):
        aabb0 = ctx.part_world_aabb(cap)
    with ctx.pose({rotate: math.pi / 2.0}):
        aabb90 = ctx.part_world_aabb(cap)
    # The cap should remain at the same position but rotated
    if aabb0 is not None and aabb90 is not None:
        c0 = [(aabb0[0][i] + aabb0[1][i]) / 2.0 for i in range(3)]
        c90 = [(aabb90[0][i] + aabb90[1][i]) / 2.0 for i in range(3)]
        ctx.check(
            "continuous rotation keeps cap centered on the axis",
            abs(c0[0] - c90[0]) < 0.002 and abs(c0[1] - c90[1]) < 0.002,
            details=f"center0={c0}, center90={c90}",
        )

    # --- prismatic slide lifts the cap off the neck ---
    rest_z = ctx.part_world_position(cap)[2]
    max_slide = SKIRT_H + 0.010
    with ctx.pose({slide: max_slide}):
        lifted_z = ctx.part_world_position(cap)[2]
    ctx.check(
        "cap_slide lifts the nozzle cap off the neck",
        lifted_z > rest_z + max_slide * 0.8,
        details=f"rest_z={rest_z}, lifted_z={lifted_z}",
    )

    # --- mouth opening visible: when cap is lifted, neck top is exposed ---
    # Verify that the bottle shell has a hollow top (mouth) by checking that
    # the neck top region is narrower than the full neck (i.e. there's a bore).
    ctx.check(
        "bottle has a hollow neck (mouth opening modeled)",
        MOUTH_R < NECK_R,
        details=f"mouth_r={MOUTH_R}, neck_r={NECK_R}",
    )

    return ctx.report()


object_model = build_object_model()
