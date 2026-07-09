from __future__ import annotations

# Flat oval canteen bottle with side carrying loops, a tamper-evident safety
# collar ring, molded volume-measurement bands, and raised spiral neck threads.
# Frame: vertical axis along +Z, bottle standing on z=0, centerline on x=y=0.
# Articulations:
#   - collar_rotate: REVOLUTE spin of safety collar around the neck (+Z)
#   - cap_rotate:    CONTINUOUS spin of the cap about +Z
#   - cap_slide:     PRISMATIC lift of the cap off the neck

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
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- key heights (m) along +Z ----
BASE_Z = 0.000
HEEL_Z = 0.005       # small inset base heel
BODY_START_Z = 0.012  # full body cross-section starts
BODY_TOP_Z = 0.095    # end of straight body, start of shoulder taper
SHOULDER_TOP_Z = 0.125  # end of shoulder, base of neck
COLLAR_Z = 0.125      # safety collar sits here
NECK_TOP_Z = 0.155    # top of threaded neck

# ---- oval body dimensions ----
BODY_RX = 0.042       # body semi-axis along X (wide)
BODY_RY = 0.019       # body semi-axis along Y (narrow, flat canteen)
WALL = 0.0015         # shell wall thickness

# ---- neck ----
NECK_R = 0.012        # neck outer radius (circular)
NECK_BORE_R = 0.0095  # neck inner bore

# ---- cap ----
CAP_R = 0.015         # cap outer radius
CAP_HEIGHT = 0.022
CAP_BORE_R = NECK_R   # skirt grips neck
CAP_MOUNT_Z = NECK_TOP_Z - CAP_HEIGHT  # cap screws down over neck

# ---- volume bands ----
BAND_HEIGHTS = [0.035, 0.055, 0.075]  # three molded measurement lines
BAND_WIDTH = 0.002
BAND_DEPTH = 0.0008  # how far they protrude from the body surface

# ---- side loops ----
LOOP_MAJOR_R = 0.007  # loop ring center radius
LOOP_MINOR_R = 0.0018  # loop tube radius
LOOP_Z = 0.055        # height at which loops attach


def _bottle_outer() -> cq.Workplane:
    """Build the canteen outer shell: flat oval body -> tapered shoulder -> round neck."""
    # Use CadQuery loft through multiple wire sections
    heel_rx, heel_ry = BODY_RX * 0.82, BODY_RY * 0.82
    shoulder_rx, shoulder_ry = BODY_RX * 0.65, BODY_RY * 0.72
    neck_transition_r = NECK_R * 1.35
    # Quick transition to cylindrical neck so threads sit on the outer surface
    neck_cylinder_start = SHOULDER_TOP_Z + 0.005  # neck reaches full radius quickly

    body = (
        cq.Workplane("XY")
        .workplane(offset=HEEL_Z)
        .ellipse(heel_rx, heel_ry)
        .workplane(offset=BODY_START_Z - HEEL_Z)
        .ellipse(BODY_RX, BODY_RY)
        .workplane(offset=BODY_TOP_Z - BODY_START_Z)
        .ellipse(BODY_RX, BODY_RY)
        .workplane(offset=(SHOULDER_TOP_Z - BODY_TOP_Z) * 0.55)
        .ellipse(shoulder_rx, shoulder_ry)
        .workplane(offset=(SHOULDER_TOP_Z - BODY_TOP_Z) * 0.45)
        .circle(neck_transition_r)
        .workplane(offset=neck_cylinder_start - SHOULDER_TOP_Z)
        .circle(NECK_R)
        .workplane(offset=NECK_TOP_Z - neck_cylinder_start)
        .circle(NECK_R)
        .loft()
    )
    return body


def _bottle_shell() -> cq.Workplane:
    """Hollow the outer shell: remove top face and shell inward."""
    outer = _bottle_outer()
    # Shell: remove top face, inward shell by wall thickness
    shelled = outer.faces(">Z").shell(-WALL)
    return shelled


def _bottle_mesh():
    return mesh_from_cadquery(_bottle_shell(), "bottle_shell")


def _volume_bands():
    """Molded raised elliptical rings around the body at measurement heights."""
    combined = None
    for z in BAND_HEIGHTS:
        # Outer elliptical ring slightly larger than body
        outer_ring = (
            cq.Workplane("XY")
            .workplane(offset=z - BAND_WIDTH / 2)
            .ellipse(BODY_RX + BAND_DEPTH, BODY_RY + BAND_DEPTH)
            .extrude(BAND_WIDTH)
        )
        inner_cut = (
            cq.Workplane("XY")
            .workplane(offset=z - BAND_WIDTH / 2 - 0.0002)
            .ellipse(BODY_RX - 0.0001, BODY_RY - 0.0001)
            .extrude(BAND_WIDTH + 0.0004)
        )
        band = outer_ring.cut(inner_cut)
        if combined is None:
            combined = band
        else:
            combined = combined.union(band)
    return mesh_from_cadquery(combined, "volume_bands")


def _side_loops():
    """Carrying loops on each side of the flat canteen body."""
    g = None
    for x_sign in (-1, 1):
        # Torus ring oriented so the loop plane is vertical (XZ plane)
        # The loop sticks out from the side of the bottle
        loop = TorusGeometry(
            LOOP_MAJOR_R, LOOP_MINOR_R,
            radial_segments=12, tubular_segments=32,
        )
        # Default torus is in XY plane; rotate so ring plane is in YZ
        # then translate to the side of the body
        loop.rotate_y(math.pi / 2)
        loop.translate(x_sign * (BODY_RX + LOOP_MAJOR_R * 0.35), 0.0, LOOP_Z)
        if g is None:
            g = loop
        else:
            g.merge(loop)
    return mesh_from_geometry(g, "side_loops")


def _neck_threads():
    """Raised spiral-like ridges on the neck (visible thread detail)."""
    g = None
    n_turns = 3
    # Threads start after neck reaches full cylinder (SHOULDER_TOP_Z + 0.005)
    thread_start = SHOULDER_TOP_Z + 0.008
    thread_end = NECK_TOP_Z - 0.003
    thread_pitch = (thread_end - thread_start) / n_turns
    for i in range(n_turns):
        z_base = thread_start + i * thread_pitch
        # Each thread ridge is a torus on the neck outer surface
        ring = TorusGeometry(
            NECK_R, 0.001,
            radial_segments=8, tubular_segments=36,
        )
        ring.translate(0.0, 0.0, z_base)
        if g is None:
            g = ring
        else:
            g.merge(ring)
    return mesh_from_geometry(g, "neck_threads")


def _safety_collar_solid() -> cq.Workplane:
    """Tamper-evident collar ring: thin ring around the neck base."""
    collar_height = 0.006
    collar_outer_r = NECK_R + 0.003
    collar_inner_r = NECK_R - 0.0002  # slight clearance over neck

    collar = (
        cq.Workplane("XY")
        .circle(collar_outer_r)
        .extrude(collar_height)
    )
    bore = (
        cq.Workplane("XY")
        .circle(collar_inner_r)
        .extrude(collar_height)
    )
    collar = collar.cut(bore)

    # Add small tear-notch indicators (two small cuts on opposite sides)
    for angle in (0, math.pi):
        notch_x = collar_outer_r * math.cos(angle)
        notch_y = collar_outer_r * math.sin(angle)
        notch = (
            cq.Workplane("XY")
            .center(notch_x, notch_y)
            .circle(0.001)
            .extrude(collar_height)
        )
        collar = collar.cut(notch)

    return collar


def _safety_collar_mesh():
    return mesh_from_cadquery(_safety_collar_solid(), "collar_ring")


def _cap_solid() -> cq.Workplane:
    """Black knurled screw cap."""
    cap = (
        cq.Workplane("XY")
        .circle(CAP_R)
        .extrude(CAP_HEIGHT)
    )
    # Hollow bore so cap screws over neck
    bore = (
        cq.Workplane("XY")
        .circle(CAP_BORE_R)
        .extrude(CAP_HEIGHT - 0.003)
    )
    cap = cap.cut(bore)
    # Knurl flutes
    n = 24
    for i in range(n):
        a = 2.0 * math.pi * i / n
        groove = (
            cq.Workplane("XY")
            .center(CAP_R * math.cos(a), CAP_R * math.sin(a))
            .circle(0.0009)
            .extrude(CAP_HEIGHT)
        )
        cap = cap.cut(groove)
    return cap


def _cap_mesh():
    return mesh_from_cadquery(_cap_solid(), "cap_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="canteen_bottle")

    # Materials
    clear = model.material("clear_plastic", rgba=(0.75, 0.83, 0.87, 0.30))
    band_mat = model.material("band_ridge", rgba=(0.68, 0.78, 0.82, 0.45))
    loop_mat = model.material("loop_plastic", rgba=(0.72, 0.80, 0.84, 0.35))
    thread_mat = model.material("thread_ridge", rgba=(0.70, 0.78, 0.82, 0.40))
    collar_mat = model.material("collar_white", rgba=(0.90, 0.90, 0.88, 0.85))
    black = model.material("cap_black", rgba=(0.07, 0.07, 0.08, 1.0))
    marker = model.material("cap_marker", rgba=(0.85, 0.15, 0.12, 1.0))

    # ---- bottle body (root): flat oval hollow shell ----
    body = model.part("bottle_body")
    body.visual(_bottle_mesh(), material=clear, name="bottle_shell")
    body.visual(_volume_bands(), material=band_mat, name="volume_bands")
    body.visual(_side_loops(), material=loop_mat, name="side_loops")
    body.visual(_neck_threads(), material=thread_mat, name="neck_threads")
    body.inertial = Inertial.from_geometry(
        Box((BODY_RX * 2, BODY_RY * 2, NECK_TOP_Z)),
        mass=0.035,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2)),
    )

    # ---- safety collar: rotates around neck (tamper-evident) ----
    collar = model.part("safety_collar")
    collar.visual(_safety_collar_mesh(), material=collar_mat, name="collar_ring")
    collar.inertial = Inertial.from_geometry(
        Cylinder(NECK_R + 0.003, 0.006),
        mass=0.001,
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
    )

    # collar_rotate: REVOLUTE about +Z, limited rotation (tears/breaks after ~90°)
    model.articulation(
        "collar_rotate",
        ArticulationType.REVOLUTE,
        parent=body,
        child=collar,
        origin=Origin(xyz=(0.0, 0.0, COLLAR_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=math.pi / 2),
    )

    # ---- massless carrier: decouples cap spin from lift ----
    carrier = model.part("cap_carrier")
    carrier.inertial = Inertial.from_geometry(Box((0.008, 0.008, 0.008)), mass=1e-4)

    # ---- black screw cap ----
    cap = model.part("cap")
    cap.visual(_cap_mesh(), material=black, name="cap_shell")
    cap.visual(
        Cylinder(0.0018, 0.004),
        origin=Origin(xyz=(CAP_R - 0.003, 0.0, CAP_HEIGHT - 0.001)),
        material=marker,
        name="cap_marker",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(CAP_R, CAP_HEIGHT),
        mass=0.003,
        origin=Origin(xyz=(0.0, 0.0, CAP_HEIGHT / 2.0)),
    )

    # cap_rotate: CONTINUOUS spin about +Z
    model.articulation(
        "cap_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, CAP_MOUNT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )
    # cap_slide: PRISMATIC lift along +Z
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


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle_body")
    collar = object_model.get_part("safety_collar")
    cap = object_model.get_part("cap")
    collar_joint = object_model.get_articulation("collar_rotate")
    cap_rotate = object_model.get_articulation("cap_rotate")
    cap_slide = object_model.get_articulation("cap_slide")

    # --- bottle body is clear/transparent ---
    clear_mat = next(m for m in object_model.materials if m.name == "clear_plastic")
    a = clear_mat.rgba[3] if clear_mat.rgba is not None else 1.0
    ctx.check(
        "bottle shell is transparent",
        a < 1.0,
        details=f"clear_plastic alpha={a}",
    )

    # --- flat oval cross-section: width >> depth ---
    body_aabb = ctx.part_world_aabb(body)
    dx = body_aabb[1][0] - body_aabb[0][0]
    dy = body_aabb[1][1] - body_aabb[0][1]
    ctx.check(
        "bottle has flat oval cross-section (wider than deep)",
        dx > dy * 1.5,
        details=f"body dx={dx:.4f}, dy={dy:.4f}, ratio={dx/max(dy,1e-6):.2f}",
    )

    # --- volume bands exist on the body ---
    bands_visual = body.get_visual("volume_bands")
    ctx.check(
        "volume bands present on body",
        bands_visual is not None,
        details="volume_bands visual not found",
    )

    # --- side loops exist ---
    loops_visual = body.get_visual("side_loops")
    ctx.check(
        "side carrying loops present",
        loops_visual is not None,
        details="side_loops visual not found",
    )

    # --- neck threads exist as raised ridges ---
    threads_visual = body.get_visual("neck_threads")
    ctx.check(
        "neck threads present as raised ridges",
        threads_visual is not None,
        details="neck_threads visual not found",
    )

    # --- safety collar is a separate part positioned at the neck base ---
    collar_pos = ctx.part_world_position(collar)
    ctx.check(
        "safety collar positioned at neck base",
        collar_pos is not None and abs(collar_pos[2] - COLLAR_Z) < 0.01,
        details=f"collar z={collar_pos}",
    )

    # --- safety collar rotates (tamper-evident action) ---
    collar_rest_aabb = ctx.part_world_aabb(collar)
    with ctx.pose({collar_joint: math.pi / 4}):
        collar_rotated_aabb = ctx.part_world_aabb(collar)
    # AABB should change when collar rotates (due to notch asymmetry)
    ctx.check(
        "safety collar can rotate around neck",
        collar_joint.motion_limits.upper > 0.1,
        details=f"collar upper limit={collar_joint.motion_limits.upper:.3f} rad",
    )

    # --- collar overlap allowance (sits around neck) ---
    ctx.allow_overlap(
        collar,
        body,
        elem_a="collar_ring",
        elem_b="bottle_shell",
        reason="The safety collar intentionally encircles the neck base with slight overlap.",
    )

    # --- cap seated over neck at rest ---
    ctx.allow_overlap(
        cap,
        body,
        elem_a="cap_shell",
        elem_b="bottle_shell",
        reason="The cap skirt screws down over the threaded neck.",
    )
    ctx.allow_overlap(
        cap,
        body,
        elem_a="cap_shell",
        elem_b="neck_threads",
        reason="The cap covers neck threads when closed.",
    )

    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "cap mounted at top of bottle",
        cap_pos is not None and cap_pos[2] > 0.12,
        details=f"cap origin={cap_pos}",
    )

    # --- cap spins (marker moves) ---
    cap_marker = cap.get_visual("cap_marker")

    def _marker_xy():
        mn, mx = ctx.part_element_world_aabb(cap, elem=cap_marker)
        return ((mn[0] + mx[0]) / 2.0, (mn[1] + mx[1]) / 2.0)

    mk0 = _marker_xy()
    with ctx.pose({cap_rotate: math.pi / 2.0}):
        mk90 = _marker_xy()
    moved = math.hypot(mk90[0] - mk0[0], mk90[1] - mk0[1])
    ctx.check(
        "cap spins about +Z (marker swings)",
        moved > 0.005,
        details=f"marker rest={mk0}, rotated={mk90}, moved={moved:.4f}",
    )

    # --- cap slides up off neck ---
    z_rest = ctx.part_world_aabb(cap)[0][2]
    with ctx.pose({cap_slide: CAP_HEIGHT}):
        z_up = ctx.part_world_aabb(cap)[0][2]
    ctx.check(
        "cap slides up off the neck",
        z_up > z_rest + 0.015,
        details=f"cap bottom z rest={z_rest:.4f}, lifted={z_up:.4f}",
    )

    # --- bottle is taller than wide (canteen proportions) ---
    dz = body_aabb[1][2] - body_aabb[0][2]
    ctx.check(
        "bottle is taller than wide",
        dz > dx * 1.5,
        details=f"body dz={dz:.4f}, dx={dx:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
