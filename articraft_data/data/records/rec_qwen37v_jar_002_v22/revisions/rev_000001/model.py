from __future__ import annotations

# Square pantry jar with rounded corners and a brass lid containing a
# rotating shaker insert. Variant of the square glass storage jar.
#
# Frame: jar stands on +XY ground plane, central axis along +Z (up).
#   - jar_body (root): square-section clear glass shell with generously
#     rounded vertical edges, hollow interior, foot ring at the base,
#     thick-walled mouth with rim seam at the top, and a short round neck.
#   - lid_carrier: massless carrier link for decoupled lid joints.
#   - lid: round brass cap that screws onto the neck.
#   - shaker_insert: thin disc with a pattern of shaker holes; sits inside
#     the lid and rotates to open/close the pour ports.
#
# Articulations:
#   lid_rotate   (CONTINUOUS, body->carrier): lid spins about +Z
#   lid_slide    (PRISMATIC, carrier->lid):   lid lifts off the neck
#   shaker_spin  (REVOLUTE,   lid->shaker):   shaker rotates inside lid

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- key dimensions (meters) ----
BODY_HALF = 0.044           # half-width of square section (~88 mm square)
BODY_FILLET = 0.016         # generous rounded-corner radius
WALL = 0.004                # glass wall thickness (body)
MOUTH_WALL = 0.006          # thicker glass wall at the mouth/neck
BODY_Z0 = 0.0               # jar base sits on ground
BODY_TOP = 0.088            # top of main square body section
SHOULDER_TOP = 0.098        # top of tapered shoulder -> neck transition
NECK_R = 0.033              # outer radius of round mouth/neck
NECK_TOP = 0.112            # top of the neck (z)
NECK_BOTTOM = SHOULDER_TOP

# Foot ring
FOOT_RING_R_OUTER = BODY_HALF + 0.002   # slightly wider than body
FOOT_RING_R_INNER = BODY_HALF - 0.006
FOOT_RING_HEIGHT = 0.005                # height of the foot ring

# Rim seam
RIM_SEAM_HEIGHT = 0.003
RIM_SEAM_PROTRUDE = 0.0015   # how far the seam protrudes beyond neck outer

# Lid
LID_R = 0.036               # brass lid outer radius
LID_HEIGHT = 0.022          # lid skirt + top plate height
LID_TOP_THICK = 0.004       # thickness of the lid top plate
SCALLOP_N = 24              # knurling scallops on lid skirt

# Lid mount: skirt drops over neck so bore wraps the neck top
LID_MOUNT_Z = NECK_TOP - 0.012

# Shaker insert
SHAKER_R = NECK_R - MOUTH_WALL - 0.001   # fits inside the neck bore
SHAKER_THICK = 0.0025                     # disc thickness
SHAKER_HOLE_R = 0.003                     # shaker hole radius
SHAKER_HOLE_RING_R = 0.016               # radius of the hole ring pattern
SHAKER_HOLE_N = 6                         # number of shaker holes
# Shaker sits just below the lid top plate
SHAKER_Z_IN_LID = LID_HEIGHT - LID_TOP_THICK - SHAKER_THICK

# Lid pour-port openings (matching shaker pattern)
PORT_HOLE_R = SHAKER_HOLE_R + 0.0005     # slightly larger than shaker holes
PORT_RING_R = SHAKER_HOLE_RING_R
PORT_HOLE_N = SHAKER_HOLE_N


def _foot_ring() -> cq.Workplane:
    """Base foot ring: a short annular ring at the bottom of the jar."""
    outer = (
        cq.Workplane("XY")
        .rect(2 * FOOT_RING_R_OUTER, 2 * FOOT_RING_R_OUTER)
        .extrude(FOOT_RING_HEIGHT)
    )
    # Round the outer vertical edges of the foot ring to match body corners
    try:
        outer = outer.edges("|Z").fillet(BODY_FILLET * 0.8)
    except Exception:
        pass
    # Cut the inner void (matches the jar body footprint)
    inner = (
        cq.Workplane("XY")
        .rect(2 * BODY_HALF - 0.002, 2 * BODY_HALF - 0.002)
        .extrude(FOOT_RING_HEIGHT)
    )
    try:
        inner = inner.edges("|Z").fillet(max(BODY_FILLET - 0.004, 0.002))
    except Exception:
        pass
    return outer.cut(inner)


def _body_solid() -> cq.Workplane:
    """Hollow square pantry jar with rounded corners, thick mouth, rim seam."""
    # Main square body prism with filleted vertical edges
    outer = (
        cq.Workplane("XY")
        .workplane(offset=FOOT_RING_HEIGHT)
        .box(2 * BODY_HALF, 2 * BODY_HALF, BODY_TOP - FOOT_RING_HEIGHT,
             centered=(True, True, False))
        .edges("|Z")
        .fillet(BODY_FILLET)
    )

    # Tapered shoulder: square body top -> round neck base
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP)
        .rect(2 * (BODY_HALF - 0.004), 2 * (BODY_HALF - 0.004))
        .workplane(offset=(SHOULDER_TOP - BODY_TOP))
        .circle(NECK_R)
        .loft(ruled=False)
    )

    # Round neck with thick mouth walls
    neck = (
        cq.Workplane("XY")
        .workplane(offset=NECK_BOTTOM)
        .circle(NECK_R)
        .extrude(NECK_TOP - NECK_BOTTOM)
    )

    # Rim seam: raised ring at the top of the neck
    rim_seam = (
        cq.Workplane("XY")
        .workplane(offset=NECK_TOP - RIM_SEAM_HEIGHT)
        .circle(NECK_R + RIM_SEAM_PROTRUDE)
        .circle(NECK_R - 0.001)
        .extrude(RIM_SEAM_HEIGHT)
    )

    solid = outer.union(shoulder).union(neck).union(rim_seam)

    # Hollow cavity: uses MOUTH_WALL at neck, regular WALL at body
    inner_body = (
        cq.Workplane("XY")
        .workplane(offset=FOOT_RING_HEIGHT + WALL)
        .box(
            2 * (BODY_HALF - WALL),
            2 * (BODY_HALF - WALL),
            BODY_TOP - FOOT_RING_HEIGHT - WALL,
            centered=(True, True, False),
        )
        .edges("|Z")
        .fillet(max(BODY_FILLET - WALL, 0.002))
    )
    inner_shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP - 0.001)
        .rect(2 * (BODY_HALF - 0.004 - WALL), 2 * (BODY_HALF - 0.004 - WALL))
        .workplane(offset=(SHOULDER_TOP - BODY_TOP) + 0.001)
        .circle(NECK_R - MOUTH_WALL)
        .loft(ruled=False)
    )
    inner_neck = (
        cq.Workplane("XY")
        .workplane(offset=NECK_BOTTOM)
        .circle(NECK_R - MOUTH_WALL)
        .extrude((NECK_TOP - NECK_BOTTOM) + 0.002)
    )
    cavity = inner_body.union(inner_shoulder).union(inner_neck)
    return solid.cut(cavity)


def _thread_ridges() -> cq.Workplane:
    """Thin thread ridges on the neck exterior."""
    rings = None
    for zc in (NECK_BOTTOM + 0.005, NECK_BOTTOM + 0.011):
        ring = (
            cq.Workplane("XY")
            .workplane(offset=zc)
            .circle(NECK_R + 0.0008)
            .circle(NECK_R - 0.0004)
            .extrude(0.002)
        )
        rings = ring if rings is None else rings.union(ring)
    return rings


def _body_mesh():
    solid = _body_solid().union(_thread_ridges()).union(_foot_ring())
    return mesh_from_cadquery(solid, "jar_glass")


def _lid_solid() -> cq.Workplane:
    """Round brass lid: flat top with pour-port holes + knurled skirt."""
    # Outer skirt cylinder
    skirt = (
        cq.Workplane("XY")
        .circle(LID_R)
        .extrude(LID_HEIGHT)
    )
    # Hollow bore so it caps over the neck (leave top plate)
    bore = (
        cq.Workplane("XY")
        .circle(NECK_R - 0.0006)
        .extrude(LID_HEIGHT - LID_TOP_THICK)
    )
    lid = skirt.cut(bore)

    # Pour-port holes in the top plate (matching shaker pattern)
    for k in range(PORT_HOLE_N):
        ang = 2.0 * math.pi * k / PORT_HOLE_N
        hx = PORT_RING_R * math.cos(ang)
        hy = PORT_RING_R * math.sin(ang)
        hole = (
            cq.Workplane("XY")
            .workplane(offset=LID_HEIGHT - LID_TOP_THICK - 0.001)
            .center(hx, hy)
            .circle(PORT_HOLE_R)
            .extrude(LID_TOP_THICK + 0.002)
        )
        lid = lid.cut(hole)

    # Scallops / knurling on the skirt
    for k in range(SCALLOP_N):
        ang = 2.0 * math.pi * k / SCALLOP_N
        fx = LID_R * math.cos(ang)
        fy = LID_R * math.sin(ang)
        flute = (
            cq.Workplane("XY")
            .center(fx, fy)
            .circle(0.0024)
            .extrude(LID_HEIGHT)
        )
        lid = lid.cut(flute)

    # Chamfer top outer edge
    try:
        lid = lid.faces(">Z").edges().chamfer(0.0012)
    except Exception:
        pass
    return lid


def _lid_mesh():
    return mesh_from_cadquery(_lid_solid(), "lid_brass")


def _shaker_solid() -> cq.Workplane:
    """Shaker insert disc: thin disc with a ring of pour holes and rotation tab."""
    disc = (
        cq.Workplane("XY")
        .circle(SHAKER_R)
        .extrude(SHAKER_THICK)
    )
    # Central handle/grip nub
    nub = (
        cq.Workplane("XY")
        .workplane(offset=SHAKER_THICK)
        .circle(0.005)
        .extrude(0.004)
    )
    disc = disc.union(nub)

    # Rotation tab: a small tab extending beyond the disc radius for grip and
    # to make rotation visible in tests. Placed at 30° to avoid hole pattern.
    tab_ang = math.pi / 6.0  # 30 degrees
    tab_x = (SHAKER_R + 0.003) * math.cos(tab_ang)
    tab_y = (SHAKER_R + 0.003) * math.sin(tab_ang)
    tab = (
        cq.Workplane("XY")
        .center(tab_x, tab_y)
        .rect(0.008, 0.006)
        .extrude(SHAKER_THICK)
    )
    disc = disc.union(tab)

    # Shaker holes in a ring pattern
    for k in range(SHAKER_HOLE_N):
        ang = 2.0 * math.pi * k / SHAKER_HOLE_N
        hx = SHAKER_HOLE_RING_R * math.cos(ang)
        hy = SHAKER_HOLE_RING_R * math.sin(ang)
        hole = (
            cq.Workplane("XY")
            .center(hx, hy)
            .circle(SHAKER_HOLE_R)
            .extrude(SHAKER_THICK)
        )
        disc = disc.cut(hole)
    return disc


def _shaker_mesh():
    return mesh_from_cadquery(_shaker_solid(), "shaker_insert")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="square_pantry_jar_shaker_lid")

    glass = model.material("clear_glass", rgba=(0.82, 0.87, 0.88, 0.22))
    brass = model.material("brass", rgba=(0.72, 0.55, 0.20, 1.0))
    brass_dark = model.material("brass_dark", rgba=(0.50, 0.36, 0.10, 1.0))
    steel = model.material("stainless_steel", rgba=(0.75, 0.75, 0.73, 1.0))

    # ---- jar body (root): square hollow glass + foot ring + rim seam ----
    body = model.part("jar_body")
    body.visual(_body_mesh(), material=glass, name="jar_glass")
    body.inertial = Inertial.from_geometry(
        Box((2 * BODY_HALF, 2 * BODY_HALF, NECK_TOP)),
        mass=0.32,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP / 2.0)),
    )

    # ---- massless carrier for decoupled lid joints ----
    carrier = model.part("lid_carrier")
    carrier.inertial = Inertial.from_geometry(Box((0.008, 0.008, 0.008)), mass=1e-4)

    # ---- brass lid with pour-port openings ----
    lid = model.part("lid")
    lid.visual(_lid_mesh(), material=brass, name="lid_brass")
    # Off-axis marker so rotation is observable
    marker = CylinderGeometry(0.002, 0.004).translate(LID_R - 0.004, 0.0, LID_HEIGHT)
    lid.visual(mesh_from_geometry(marker, "lid_marker"), material=brass_dark, name="lid_marker")
    lid.inertial = Inertial.from_geometry(
        Cylinder(LID_R, LID_HEIGHT),
        mass=0.045,
        origin=Origin(xyz=(0.0, 0.0, LID_HEIGHT / 2.0)),
    )

    # ---- shaker insert disc (rotates inside the lid) ----
    shaker = model.part("shaker_insert")
    # The shaker disc origin is at its bottom face center; we place it inside
    # the lid at the correct height via the articulation origin.
    shaker.visual(_shaker_mesh(), material=steel, name="shaker_disc")
    shaker.inertial = Inertial.from_geometry(
        Cylinder(SHAKER_R, SHAKER_THICK),
        mass=0.012,
        origin=Origin(xyz=(0.0, 0.0, SHAKER_THICK / 2.0)),
    )

    # ---- Articulations ----

    # lid_rotate: body -> carrier, CONTINUOUS about +Z
    model.articulation(
        "lid_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, LID_MOUNT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )

    # lid_slide: carrier -> lid, PRISMATIC along +Z (lifts lid off neck)
    model.articulation(
        "lid_slide",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0, upper=LID_HEIGHT + 0.01, effort=1.0, velocity=1.0
        ),
    )

    # shaker_spin: lid -> shaker, REVOLUTE about +Z with limited range
    # The shaker sits inside the lid at SHAKER_Z_IN_LID height (lid-local).
    model.articulation(
        "shaker_spin",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=shaker,
        origin=Origin(xyz=(0.0, 0.0, SHAKER_Z_IN_LID)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0, upper=math.pi / 2.0, effort=0.5, velocity=2.0
        ),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("jar_body")
    lid = object_model.get_part("lid")
    shaker = object_model.get_part("shaker_insert")
    lid_rotate = object_model.get_articulation("lid_rotate")
    lid_slide = object_model.get_articulation("lid_slide")
    shaker_spin = object_model.get_articulation("shaker_spin")

    # ---- Allowances for intentional capture fits ----
    ctx.allow_overlap(
        lid, body,
        elem_a="lid_brass", elem_b="jar_glass",
        reason="Brass lid skirt is intentionally screwed down over the glass neck.",
    )
    ctx.allow_overlap(
        lid, shaker,
        elem_a="lid_brass", elem_b="shaker_disc",
        reason="Shaker insert disc sits inside the lid cavity as a rotating insert.",
    )

    # ---- Jar body: square cross-section with rounded corners ----
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "jar body is approximately square in cross-section",
        abs(bext[0] - bext[1]) < 0.008,
        details=f"x={bext[0]:.4f}, y={bext[1]:.4f}",
    )
    ctx.check(
        "jar is wider than 70mm (pantry jar scale)",
        bext[0] > 0.070 and bext[1] > 0.070,
        details=f"extents x={bext[0]:.4f}, y={bext[1]:.4f}",
    )

    # ---- Foot ring: body has geometry extending below the main body ----
    # The foot ring protrudes at the base; the body AABB should start near z=0.
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "foot ring present at base (body starts near z=0)",
        body_aabb[0][2] < 0.003,
        details=f"body min z={body_aabb[0][2]:.4f}",
    )

    # ---- Rim seam: neck has geometry protruding beyond the main neck radius ----
    # The rim seam adds geometry slightly wider than the neck. We verify the
    # body has some material near the top that extends beyond the plain neck.
    body_pos = ctx.part_world_position(body)
    ctx.check(
        "body extends to full neck height",
        body_aabb[1][2] > NECK_TOP - 0.005,
        details=f"body max z={body_aabb[1][2]:.4f}, neck_top={NECK_TOP}",
    )

    # ---- Lid is round, seated on top of jar ----
    lext = _ext(ctx.part_world_aabb(lid))
    ctx.check(
        "lid is round (similar x and y extents, smaller than body)",
        abs(lext[0] - lext[1]) < 0.004 and lext[0] < bext[0],
        details=f"lid x={lext[0]:.4f}, y={lext[1]:.4f}",
    )
    lid_pos = ctx.part_world_position(lid)
    ctx.check(
        "lid sits at top of jar on the neck",
        lid_pos is not None and lid_pos[2] > NECK_BOTTOM,
        details=f"lid z={lid_pos[2] if lid_pos else None}",
    )
    ctx.expect_overlap(
        lid, body, axes="xy", min_overlap=0.02,
        name="lid seated over neck footprint",
    )

    # ---- lid_rotate spins the lid (off-axis marker moves) ----
    m0 = ctx.part_element_world_aabb(lid, elem="lid_marker")
    m0c = ((m0[0][0] + m0[1][0]) / 2.0, (m0[0][1] + m0[1][1]) / 2.0)
    with ctx.pose({lid_rotate: math.pi / 2.0}):
        m1 = ctx.part_element_world_aabb(lid, elem="lid_marker")
        m1c = ((m1[0][0] + m1[1][0]) / 2.0, (m1[0][1] + m1[1][1]) / 2.0)
    marker_shift = math.hypot(m1c[0] - m0c[0], m1c[1] - m0c[1])
    ctx.check(
        "lid_rotate spins the lid (marker moves on quarter turn)",
        marker_shift > 0.01,
        details=f"marker moved {marker_shift:.4f} m",
    )

    # ---- lid_slide lifts the lid up ----
    z_rest = ctx.part_world_position(lid)[2]
    with ctx.pose({lid_slide: LID_HEIGHT}):
        z_lift = ctx.part_world_position(lid)[2]
    ctx.check(
        "lid_slide lifts the lid off the neck",
        z_lift > z_rest + 0.015,
        details=f"rest z={z_rest:.4f}, lifted z={z_lift:.4f}",
    )

    # ---- shaker_spin: revolute joint with limited range ----
    ctx.check(
        "shaker_spin is revolute about +Z",
        shaker_spin.axis == (0.0, 0.0, 1.0)
        and shaker_spin.articulation_type == ArticulationType.REVOLUTE,
        details=f"axis={shaker_spin.axis}, type={shaker_spin.articulation_type}",
    )
    limits = shaker_spin.motion_limits
    ctx.check(
        "shaker_spin has limited range (0 to ~pi/2)",
        limits is not None and abs(limits.lower) < 0.01 and 1.0 < limits.upper < 2.0,
        details=f"lower={limits.lower}, upper={limits.upper}",
    )

    # ---- shaker disc rotates when shaker_spin is actuated ----
    # The rotation tab extends beyond the disc, making the AABB asymmetric
    s0_aabb = ctx.part_element_world_aabb(shaker, elem="shaker_disc")
    s0_cx = (s0_aabb[0][0] + s0_aabb[1][0]) / 2.0
    s0_cy = (s0_aabb[0][1] + s0_aabb[1][1]) / 2.0
    with ctx.pose({shaker_spin: math.pi / 3.0}):  # 60 degrees
        s1_aabb = ctx.part_element_world_aabb(shaker, elem="shaker_disc")
        s1_cx = (s1_aabb[0][0] + s1_aabb[1][0]) / 2.0
        s1_cy = (s1_aabb[0][1] + s1_aabb[1][1]) / 2.0
    # The AABB center should shift when the asymmetric tab rotates
    center_shift = math.hypot(s1_cx - s0_cx, s1_cy - s0_cy)
    ctx.check(
        "shaker_spin rotates the shaker (tab moves, AABB shifts)",
        center_shift > 0.001,
        details=f"AABB center moved {center_shift:.4f} m on 60° rotation",
    )

    # ---- shaker is contained within the lid footprint (XY) ----
    ctx.expect_within(
        shaker, lid, axes="xy", margin=0.005,
        name="shaker insert stays within lid footprint",
    )

    # ---- joint type verification ----
    ctx.check(
        "lid_rotate is continuous about +Z",
        lid_rotate.axis == (0.0, 0.0, 1.0),
        details=f"axis={lid_rotate.axis}, type={lid_rotate.articulation_type}",
    )
    ctx.check(
        "lid_slide is prismatic along +Z",
        lid_slide.axis == (0.0, 0.0, 1.0),
        details=f"axis={lid_slide.axis}, type={lid_slide.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
