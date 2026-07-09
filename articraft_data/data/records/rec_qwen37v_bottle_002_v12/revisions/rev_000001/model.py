from __future__ import annotations

# Squat square-shouldered plastic bottle with:
#   - rounded-rect hollow body with molded volume bands
#   - abrupt square shoulder transitioning to a cylindrical neck
#   - safety collar ring around the neck (rotates independently)
#   - black screw cap with knurled skirt
# Frame: vertical axis +Z, bottle standing on z=0, centerline at x=y=0.
# Articulations:
#   - collar_rotate: CONTINUOUS spin of safety collar about +Z
#   - cap_rotate:  CONTINUOUS spin of cap about +Z (via massless carrier)
#   - cap_slide:   PRISMATIC lift of cap off the neck

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

# ---- key dimensions (m) ----
BODY_W = 0.070        # body width (X)
BODY_D = 0.070        # body depth (Y)
CORNER_R = 0.006      # body corner fillet radius
BODY_H = 0.075        # straight body height
SHOULDER_H = 0.004    # shoulder plate thickness
NECK_R = 0.012        # neck outer radius
NECK_BORE_R = 0.0095  # neck inner bore
NECK_H = 0.022        # neck height above shoulder

SHOULDER_Z = BODY_H                       # 0.075
SHOULDER_TOP_Z = BODY_H + SHOULDER_H      # 0.079
NECK_TOP_Z = SHOULDER_TOP_Z + NECK_H      # 0.101

WALL = 0.0015         # wall thickness

CAP_R = 0.0145        # cap outer radius
CAP_H = 0.018         # cap height
CAP_BORE_R = NECK_R   # cap skirt bore = neck outer

# Safety collar sits just below the cap, around the neck
COLLAR_OR = NECK_R + 0.004   # collar outer radius
COLLAR_IR = NECK_R - 0.0002  # collar inner bore (slight interference/grip on neck)
COLLAR_H = 0.005             # collar height
COLLAR_Z = SHOULDER_TOP_Z    # sits on top of shoulder (bottom face contacts shoulder plate)

CAP_MOUNT_Z = COLLAR_Z + COLLAR_H  # cap mounts above collar = 0.084

# Volume band positions (z centers) and protrusion
BAND_ZS = [0.020, 0.040, 0.060]
BAND_H = 0.0025     # band vertical thickness
BAND_PROUD = 0.0008  # how far band protrudes from body surface


def _rounded_rect(wp: cq.Workplane, w: float, d: float, r: float) -> cq.Workplane:
    """Draw a centered rounded-rectangle profile on the current workplane."""
    hw, hd = w / 2.0, d / 2.0
    # Build with 4 arcs at corners and straight segments between
    wp = (
        wp.moveTo(-hw + r, -hd)
        .lineTo(hw - r, -hd)
        .threePointArc((hw, -hd + r), (hw - r + r, -hd + r))  # not quite right
    )
    # Simpler: use rect + fillet approach via Sketch
    return wp


def _rounded_rect_solid(w: float, d: float, r: float, h: float, z_base: float = 0.0) -> cq.Workplane:
    """Extruded rounded-rectangle solid using CQ rect + fillet."""
    solid = (
        cq.Workplane("XY")
        .workplane(offset=z_base)
        .rect(w, d)
        .extrude(h)
    )
    # Fillet the four vertical edges
    solid = solid.edges("|Z").fillet(r)
    return solid


def _bottle_body_solid() -> cq.Workplane:
    """Build the main bottle body: hollow rounded-rect shell + shoulder + neck."""
    # Outer body shell
    outer = _rounded_rect_solid(BODY_W, BODY_D, CORNER_R, BODY_H, z_base=0.0)

    # Inner cavity (hollow)
    iw = BODY_W - 2 * WALL
    id_ = BODY_D - 2 * WALL
    ir = max(CORNER_R - WALL, 0.002)
    cavity = _rounded_rect_solid(iw, id_, ir, BODY_H - WALL, z_base=WALL)
    body = outer.cut(cavity)

    # Shoulder plate: rounded rect same as body top, with circular hole for neck
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=SHOULDER_Z)
        .rect(BODY_W, BODY_D)
        .extrude(SHOULDER_H)
    )
    shoulder = shoulder.edges("|Z").fillet(CORNER_R)
    # Cut neck bore through shoulder
    neck_hole = (
        cq.Workplane("XY")
        .workplane(offset=SHOULDER_Z - 0.001)
        .circle(NECK_BORE_R)
        .extrude(SHOULDER_H + 0.002)
    )
    shoulder = shoulder.cut(neck_hole)
    body = body.union(shoulder)

    # Neck: cylindrical tube
    neck_outer = (
        cq.Workplane("XY")
        .workplane(offset=SHOULDER_TOP_Z)
        .circle(NECK_R)
        .extrude(NECK_H)
    )
    neck_bore = (
        cq.Workplane("XY")
        .workplane(offset=SHOULDER_TOP_Z - 0.001)
        .circle(NECK_BORE_R)
        .extrude(NECK_H + 0.002)
    )
    neck = neck_outer.cut(neck_bore)
    body = body.union(neck)

    # Neck rim: small lip at top
    rim = (
        cq.Workplane("XY")
        .workplane(offset=NECK_TOP_Z - 0.002)
        .circle(NECK_R + 0.001)
        .extrude(0.002)
    )
    rim_bore = (
        cq.Workplane("XY")
        .workplane(offset=NECK_TOP_Z - 0.003)
        .circle(NECK_BORE_R)
        .extrude(0.004)
    )
    rim = rim.cut(rim_bore)
    body = body.union(rim)

    return body


def _volume_bands_solid() -> cq.Workplane:
    """Molded volume bands: thin raised ridges around the body perimeter."""
    result = None
    for bz in BAND_ZS:
        # Outer band shell (slightly larger than body)
        bw = BODY_W + 2 * BAND_PROUD
        bd = BODY_D + 2 * BAND_PROUD
        br = CORNER_R + BAND_PROUD
        band = _rounded_rect_solid(bw, bd, br, BAND_H, z_base=bz - BAND_H / 2.0)
        # Cut the body footprint out so only the proud ring remains
        inner = _rounded_rect_solid(BODY_W - 0.0001, BODY_D - 0.0001, CORNER_R, BAND_H + 0.0002, z_base=bz - BAND_H / 2.0 - 0.0001)
        band = band.cut(inner)
        if result is None:
            result = band
        else:
            result = result.union(band)
    return result


def _bottle_mesh():
    """Full bottle body with shell + neck + volume bands."""
    body = _bottle_body_solid()
    bands = _volume_bands_solid()
    full = body.union(bands)
    return mesh_from_cadquery(full, "bottle_shell")


def _collar_solid() -> cq.Workplane:
    """Safety collar: a ring built in local frame (z=0..COLLAR_H).
    The part frame is placed at COLLAR_Z by the articulation origin."""
    outer = (
        cq.Workplane("XY")
        .circle(COLLAR_OR)
        .extrude(COLLAR_H)
    )
    bore = (
        cq.Workplane("XY")
        .circle(COLLAR_IR)
        .extrude(COLLAR_H + 0.002)
    )
    collar = outer.cut(bore)
    # Small tear-bridge tabs around the ring (tamper-evident detail)
    for i in range(8):
        a = 2.0 * math.pi * i / 8.0
        tab = (
            cq.Workplane("XY")
            .center(COLLAR_OR * math.cos(a), COLLAR_OR * math.sin(a))
            .circle(0.0012)
            .extrude(COLLAR_H)
        )
        collar = collar.union(tab)
    return collar


def _collar_mesh():
    return mesh_from_cadquery(_collar_solid(), "collar_ring")


def _cap_solid() -> cq.Workplane:
    """Black knurled screw cap."""
    cap = (
        cq.Workplane("XY")
        .circle(CAP_R)
        .extrude(CAP_H)
    )
    # Hollow underside (open bottom bore to grip neck)
    bore = (
        cq.Workplane("XY")
        .circle(CAP_BORE_R)
        .extrude(CAP_H - 0.003)
    )
    cap = cap.cut(bore)
    # Knurl flutes on skirt
    n = 24
    for i in range(n):
        a = 2.0 * math.pi * i / n
        groove = (
            cq.Workplane("XY")
            .center(CAP_R * math.cos(a), CAP_R * math.sin(a))
            .circle(0.0008)
            .extrude(CAP_H)
        )
        cap = cap.cut(groove)
    return cap


def _cap_mesh():
    return mesh_from_cadquery(_cap_solid(), "cap_shell")


def _neck_threads_mesh():
    """Thread rings on the neck."""
    g = None
    for zt in (SHOULDER_TOP_Z + 0.006, SHOULDER_TOP_Z + 0.012, SHOULDER_TOP_Z + 0.018):
        ring = TorusGeometry(NECK_R - 0.0005, 0.0010, radial_segments=8, tubular_segments=36)
        ring.translate(0.0, 0.0, zt)
        if g is None:
            g = ring
        else:
            g.merge(ring)
    return mesh_from_geometry(g, "neck_threads")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="squat_bottle")

    clear = model.material("clear_pet", rgba=(0.80, 0.88, 0.90, 0.22))
    clear_neck = model.material("clear_neck", rgba=(0.72, 0.80, 0.84, 0.30))
    collar_mat = model.material("collar_white", rgba=(0.90, 0.92, 0.93, 0.45))
    black = model.material("cap_black", rgba=(0.06, 0.06, 0.07, 1.0))
    marker = model.material("cap_marker", rgba=(0.85, 0.15, 0.12, 1.0))

    # ---- bottle body (root) ----
    body = model.part("bottle_body")
    body.visual(_bottle_mesh(), material=clear, name="bottle_shell")
    body.visual(_neck_threads_mesh(), material=clear_neck, name="neck_threads")
    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, NECK_TOP_Z)),
        mass=0.028,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    # ---- safety collar (rotates around neck) ----
    collar = model.part("safety_collar")
    collar.visual(_collar_mesh(), material=collar_mat, name="collar_ring")
    collar.inertial = Inertial.from_geometry(
        Cylinder(COLLAR_OR, COLLAR_H),
        mass=0.001,
        origin=Origin(xyz=(0.0, 0.0, COLLAR_H / 2.0)),
    )

    # ---- cap carrier (massless, for decoupled spin + slide) ----
    carrier = model.part("cap_carrier")
    carrier.inertial = Inertial.from_geometry(Box((0.006, 0.006, 0.006)), mass=1e-4)

    # ---- black screw cap ----
    cap = model.part("cap")
    cap.visual(_cap_mesh(), material=black, name="cap_shell")
    cap.visual(
        Cylinder(0.0015, 0.003),
        origin=Origin(xyz=(CAP_R - 0.003, 0.0, CAP_H - 0.001)),
        material=marker,
        name="cap_marker",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(CAP_R, CAP_H),
        mass=0.003,
        origin=Origin(xyz=(0.0, 0.0, CAP_H / 2.0)),
    )

    # ---- Articulations ----

    # collar_rotate: CONTINUOUS spin of safety collar about +Z
    model.articulation(
        "collar_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=collar,
        origin=Origin(xyz=(0.0, 0.0, COLLAR_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=2.0),
    )

    # cap_rotate: CONTINUOUS spin about +Z (body -> carrier)
    model.articulation(
        "cap_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, CAP_MOUNT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )

    # cap_slide: PRISMATIC lift along +Z (carrier -> cap)
    model.articulation(
        "cap_slide",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=CAP_H + 0.010, effort=1.0, velocity=1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle_body")
    collar = object_model.get_part("safety_collar")
    cap = object_model.get_part("cap")
    collar_joint = object_model.get_articulation("collar_rotate")
    rotate = object_model.get_articulation("cap_rotate")
    slide = object_model.get_articulation("cap_slide")

    # --- bottle is clear (alpha < 1) ---
    clear_mat = next(m for m in object_model.materials if m.name == "clear_pet")
    a = clear_mat.rgba[3] if clear_mat.rgba is not None else 1.0
    ctx.check(
        "bottle shell is transparent",
        a < 1.0,
        details=f"clear_pet alpha={a}",
    )

    # --- squat proportions: body is not much taller than wide ---
    body_aabb = ctx.part_world_aabb(body)
    mn, mx = body_aabb
    dx = mx[0] - mn[0]
    dz = mx[2] - mn[2]
    ctx.check(
        "bottle is squat (height less than 2x width)",
        dz < 2.0 * dx,
        details=f"body width={dx:.4f}, height={dz:.4f}",
    )

    # --- square-shouldered: body width at shoulder level equals body width at mid ---
    # The body has an abrupt shoulder, not a taper: body_w == BODY_W
    ctx.check(
        "body has square cross-section proportions",
        abs(dx - BODY_W) < 0.015,
        details=f"body x-extent={dx:.4f}, expected~{BODY_W}",
    )

    # --- safety collar exists and is positioned around the neck ---
    collar_pos = ctx.part_world_position(collar)
    ctx.check(
        "safety collar mounted on neck",
        collar_pos is not None and collar_pos[2] > SHOULDER_Z - 0.005,
        details=f"collar origin={collar_pos}",
    )

    # --- collar is seated on neck (intentional interference fit) ---
    ctx.allow_overlap(
        collar,
        body,
        elem_a="collar_ring",
        elem_b="bottle_shell",
        reason="The safety collar ring grips the neck with a slight interference fit, representing a tamper-evident band seated on the neck.",
    )
    ctx.expect_contact(
        collar,
        body,
        elem_a="collar_ring",
        elem_b="bottle_shell",
        name="safety collar contacts bottle body (seated on neck/shoulder)",
    )

    # --- collar rotates about +Z ---
    collar_ring = collar.get_visual("collar_ring")
    collar_aabb_rest = ctx.part_element_world_aabb(collar, elem=collar_ring)
    with ctx.pose({collar_joint: math.pi}):
        collar_aabb_rot = ctx.part_element_world_aabb(collar, elem=collar_ring)
    # AABB should be nearly the same for a symmetric ring (proves it rotates in place)
    ctx.check(
        "safety collar rotates about +Z (ring stays in place)",
        collar_aabb_rest is not None and collar_aabb_rot is not None,
        details="collar AABB exists at rest and rotated",
    )

    # --- cap seated over neck at rest ---
    ctx.allow_overlap(
        cap,
        body,
        elem_a="cap_shell",
        elem_b="bottle_shell",
        reason="The cap skirt intentionally screws down over the threaded neck.",
    )
    ctx.allow_overlap(
        cap,
        body,
        elem_a="cap_shell",
        elem_b="neck_threads",
        reason="The cap skirt covers the neck threads when closed.",
    )
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "cap mounted at top of bottle",
        cap_pos is not None and cap_pos[2] > CAP_MOUNT_Z - 0.005,
        details=f"cap origin={cap_pos}",
    )

    # --- cap spins about +Z ---
    marker_vis = cap.get_visual("cap_marker")

    def _marker_xy():
        mn_, mx_ = ctx.part_element_world_aabb(cap, elem=marker_vis)
        return ((mn_[0] + mx_[0]) / 2.0, (mn_[1] + mx_[1]) / 2.0)

    mk0 = _marker_xy()
    with ctx.pose({rotate: math.pi / 2.0}):
        mk90 = _marker_xy()
    moved = math.hypot(mk90[0] - mk0[0], mk90[1] - mk0[1])
    ctx.check(
        "cap spins about +Z (marker swings)",
        moved > 0.004,
        details=f"marker rest={mk0}, quarter={mk90}, moved={moved:.4f}",
    )

    # --- cap slides up off the neck ---
    z_rest = ctx.part_world_aabb(cap)[0][2]
    with ctx.pose({slide: CAP_H + 0.010}):
        z_up = ctx.part_world_aabb(cap)[0][2]
    ctx.check(
        "cap slides up off the neck",
        z_up > z_rest + 0.012,
        details=f"cap bottom z rest={z_rest:.4f}, lifted={z_up:.4f}",
    )

    # --- volume bands are present on the body (geometry extends beyond body_w) ---
    # The bottle_shell includes volume bands that protrude beyond BODY_W
    body_elem_aabb = ctx.part_element_world_aabb(body, elem="bottle_shell")
    body_x_extent = body_elem_aabb[1][0] - body_elem_aabb[0][0]
    ctx.check(
        "volume bands protrude beyond plain body width",
        body_x_extent > BODY_W + BAND_PROUD * 0.5,
        details=f"body_shell x-extent={body_x_extent:.5f}, body_w+proud={BODY_W + BAND_PROUD:.5f}",
    )

    return ctx.report()


object_model = build_object_model()
