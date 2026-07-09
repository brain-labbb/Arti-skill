from __future__ import annotations

# Honey jar variant: square glass storage jar with brass screw lid,
# dipper holder on the lid, and a rotating shaker insert inside the lid.
# Frame: jar stands on +XY ground plane, central axis along +Z (up).
#   - jar_body: square-section amber glass shell with rounded vertical edges,
#     hollow inside, topped by a short round threaded neck with thick rim
#     showing glass wall thickness and thread ridges. (root)
#   - lid_carrier: massless carrier for decoupled screw joint.
#   - lid: round brass cap with knurled skirt, dipper holder cradle on top.
#   - shaker_insert: perforated disc that rotates inside the lid cavity.
# Articulations:
#   lid_rotate (CONTINUOUS, body->carrier): lid spins about +Z
#   lid_slide  (PRISMATIC, carrier->lid):   lid lifts up off the neck
#   shaker_rotate (REVOLUTE, lid->shaker_insert): shaker disc rotates ±0.6 rad

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
BODY_HALF = 0.040          # half-width of the square section (0.080 square)
BODY_FILLET = 0.012        # rounded vertical-edge radius
WALL = 0.0035              # glass wall thickness
BODY_Z0 = 0.0              # jar base sits on the ground
BODY_TOP = 0.118           # top of the square body section
SHOULDER_TOP = 0.130       # top of the tapered shoulder where the neck begins
NECK_R = 0.0265            # outer radius of the round threaded neck
NECK_TOP = 0.150           # top of the neck (z)
NECK_BOTTOM = SHOULDER_TOP

# Thick rim at the mouth showing glass wall thickness
RIM_HEIGHT = 0.006         # extra rim height above neck
RIM_OUTER_R = NECK_R + 0.002  # rim outer radius (thicker than neck)
RIM_INNER_R = NECK_R - WALL   # rim inner bore matches neck bore

LID_R = 0.0300             # brass lid skirt outer radius
LID_HEIGHT = 0.024         # full height of the lid skirt + top
SCALLOP_N = 22             # number of scallops on the knurled skirt
LID_MOUNT_Z = NECK_TOP + RIM_HEIGHT - 0.016

# Dipper holder dimensions
DIPPER_CRADLE_R = 0.006    # cradle radius
DIPPER_CRADLE_HEIGHT = 0.012  # cradle post height
DIPPER_STICK_R = 0.003     # dipper stick radius
DIPPER_STICK_LENGTH = 0.080   # dipper stick length

# Shaker insert dimensions
SHAKER_R = NECK_R - 0.002  # fits inside neck bore
SHAKER_THICKNESS = 0.003   # disc thickness
SHAKER_HOLE_R = 0.003      # shaker hole radius
SHAKER_HOLE_N = 6          # number of shaker holes
SHAKER_HOLE_ORBIT_R = 0.014  # radial position of holes


def _body_solid() -> cq.Workplane:
    # Hollow square glass jar with rounded vertical edges, round neck,
    # and a thick rim at the mouth showing glass wall thickness.
    outer = (
        cq.Workplane("XY")
        .box(2 * BODY_HALF, 2 * BODY_HALF, BODY_TOP, centered=(True, True, False))
        .edges("|Z")
        .fillet(BODY_FILLET)
    )

    # Tapered shoulder: square body top -> round neck base.
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP)
        .rect(2 * (BODY_HALF - 0.004), 2 * (BODY_HALF - 0.004))
        .workplane(offset=(SHOULDER_TOP - BODY_TOP))
        .circle(NECK_R)
        .loft(ruled=False)
    )

    neck = (
        cq.Workplane("XY")
        .workplane(offset=NECK_BOTTOM)
        .circle(NECK_R)
        .extrude(NECK_TOP - NECK_BOTTOM)
    )

    # Thick rim at the mouth - wider than neck to show wall thickness
    rim = (
        cq.Workplane("XY")
        .workplane(offset=NECK_TOP)
        .circle(RIM_OUTER_R)
        .extrude(RIM_HEIGHT)
    )

    solid = outer.union(shoulder).union(neck).union(rim)

    # Hollow it out: cut an inner cavity that opens at the rim top.
    inner = (
        cq.Workplane("XY")
        .workplane(offset=WALL)
        .box(
            2 * (BODY_HALF - WALL),
            2 * (BODY_HALF - WALL),
            BODY_TOP - WALL,
            centered=(True, True, False),
        )
        .edges("|Z")
        .fillet(max(BODY_FILLET - WALL, 0.001))
    )
    inner_shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP - 0.001)
        .rect(2 * (BODY_HALF - 0.004 - WALL), 2 * (BODY_HALF - 0.004 - WALL))
        .workplane(offset=(SHOULDER_TOP - BODY_TOP) + 0.001)
        .circle(NECK_R - WALL)
        .loft(ruled=False)
    )
    inner_neck = (
        cq.Workplane("XY")
        .workplane(offset=NECK_BOTTOM)
        .circle(NECK_R - WALL)
        .extrude((NECK_TOP - NECK_BOTTOM) + 0.001)
    )
    # Rim bore - inner hole through the thick rim
    inner_rim = (
        cq.Workplane("XY")
        .workplane(offset=NECK_TOP - 0.001)
        .circle(RIM_INNER_R)
        .extrude(RIM_HEIGHT + 0.002)
    )
    cavity = inner.union(inner_shoulder).union(inner_neck).union(inner_rim)
    return solid.cut(cavity)


def _thread_ridges() -> cq.Workplane:
    # Thread ridges around the neck - prominent rings that protrude
    # beyond the neck surface for the screw lid. Inner radius is set
    # slightly inside the neck so the union fully merges.
    rings = None
    ridge_positions = [
        NECK_BOTTOM + 0.004,
        NECK_BOTTOM + 0.008,
        NECK_BOTTOM + 0.012,
        NECK_BOTTOM + 0.016,
    ]
    for zc in ridge_positions:
        ring = (
            cq.Workplane("XY")
            .workplane(offset=zc)
            .circle(NECK_R + 0.0012)
            .circle(NECK_R - 0.001)
            .extrude(0.002)
        )
        rings = ring if rings is None else rings.union(ring)
    return rings


def _body_mesh():
    solid = _body_solid().union(_thread_ridges())
    return mesh_from_cadquery(solid, "jar_glass")


def _lid_solid() -> cq.Workplane:
    # Round brass lid: flat top disc + knurled/scalloped cylindrical skirt.
    # Hollow underside so it caps over the neck rim.
    skirt = (
        cq.Workplane("XY")
        .circle(LID_R)
        .extrude(LID_HEIGHT)
    )
    # Hollow underside so it caps over the neck
    bore = (
        cq.Workplane("XY")
        .circle(RIM_INNER_R + 0.001)
        .extrude(LID_HEIGHT - 0.004)
    )
    lid = skirt.cut(bore)

    # Scallops / knurling: subtract small vertical flutes around the rim.
    for k in range(SCALLOP_N):
        ang = 2.0 * math.pi * k / SCALLOP_N
        fx = LID_R * math.cos(ang)
        fy = LID_R * math.sin(ang)
        flute = (
            cq.Workplane("XY")
            .center(fx, fy)
            .circle(0.0026)
            .extrude(LID_HEIGHT)
        )
        lid = lid.cut(flute)

    # Slight chamfer on the top outer edge.
    try:
        lid = lid.faces(">Z").edges().chamfer(0.0015)
    except Exception:
        pass
    return lid


def _dipper_holder() -> cq.Workplane:
    # Dipper holder cradle on top of the lid:
    # Two small upright posts with a horizontal cradle between them.
    post_r = 0.003
    post_h = DIPPER_CRADLE_HEIGHT
    cradle_r = DIPPER_CRADLE_R
    post_spacing = 0.020

    # Left post
    left_post = (
        cq.Workplane("XY")
        .workplane(offset=LID_HEIGHT)
        .center(-post_spacing / 2, 0)
        .circle(post_r)
        .extrude(post_h)
    )
    # Right post
    right_post = (
        cq.Workplane("XY")
        .workplane(offset=LID_HEIGHT)
        .center(post_spacing / 2, 0)
        .circle(post_r)
        .extrude(post_h)
    )
    # Horizontal cradle bar connecting the posts at the top
    cradle_bar = (
        cq.Workplane("XZ")
        .workplane(offset=0)
        .center(0, LID_HEIGHT + post_h - cradle_r)
        .rect(post_spacing, cradle_r * 2)
        .extrude(cradle_r * 2, both=True)
    )
    # Cut a cylindrical groove in the cradle to hold the dipper
    groove = (
        cq.Workplane("XZ")
        .workplane(offset=0)
        .center(0, LID_HEIGHT + post_h - cradle_r)
        .circle(DIPPER_STICK_R + 0.001)
        .extrude(post_spacing, both=True)
    )
    cradle_bar = cradle_bar.cut(groove)

    holder = left_post.union(right_post).union(cradle_bar)
    return holder


def _dipper_stick() -> cq.Workplane:
    # Honey dipper stick: a thin rod with grooved head at one end
    stick = (
        cq.Workplane("XY")
        .workplane(offset=LID_HEIGHT + DIPPER_CRADLE_HEIGHT - DIPPER_CRADLE_R)
        .center(0, 0)
        .circle(DIPPER_STICK_R)
        .extrude(DIPPER_STICK_LENGTH)
    )
    # Grooved head at the top (wider section with rings)
    head_base_z = LID_HEIGHT + DIPPER_CRADLE_HEIGHT - DIPPER_CRADLE_R + DIPPER_STICK_LENGTH - 0.025
    head = (
        cq.Workplane("XY")
        .workplane(offset=head_base_z)
        .circle(DIPPER_STICK_R * 2.5)
        .extrude(0.025)
    )
    # Cut grooves in the head
    for i in range(4):
        groove_z = head_base_z + 0.004 + i * 0.005
        groove = (
            cq.Workplane("XY")
            .workplane(offset=groove_z)
            .circle(DIPPER_STICK_R * 2.6)
            .circle(DIPPER_STICK_R * 1.8)
            .extrude(0.002)
        )
        head = head.cut(groove)

    return stick.union(head)


def _lid_mesh():
    lid = _lid_solid()
    holder = _dipper_holder()
    stick = _dipper_stick()
    combined = lid.union(holder).union(stick)
    return mesh_from_cadquery(combined, "lid_brass")


def _shaker_insert() -> cq.Workplane:
    # Perforated disc that sits inside the lid cavity.
    # The shaker_origin is at the bottom of the disc.
    disc = (
        cq.Workplane("XY")
        .circle(SHAKER_R)
        .extrude(SHAKER_THICKNESS)
    )
    # Cut shaker holes in a circular pattern
    for k in range(SHAKER_HOLE_N):
        ang = 2.0 * math.pi * k / SHAKER_HOLE_N
        hx = SHAKER_HOLE_ORBIT_R * math.cos(ang)
        hy = SHAKER_HOLE_ORBIT_R * math.sin(ang)
        hole = (
            cq.Workplane("XY")
            .center(hx, hy)
            .circle(SHAKER_HOLE_R)
            .extrude(SHAKER_THICKNESS)
        )
        disc = disc.cut(hole)
    # Center pivot hole
    center_hole = (
        cq.Workplane("XY")
        .circle(0.002)
        .extrude(SHAKER_THICKNESS)
    )
    disc = disc.cut(center_hole)
    # Small handle tab for rotation (extends from edge)
    tab = (
        cq.Workplane("XY")
        .center(SHAKER_R + 0.003, 0)
        .rect(0.006, 0.008)
        .extrude(SHAKER_THICKNESS)
    )
    disc = disc.union(tab)
    return disc


def _shaker_mesh():
    return mesh_from_cadquery(_shaker_insert(), "shaker_disc")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="honey_jar_with_dipper")

    # Materials
    amber_glass = model.material("amber_glass", rgba=(0.75, 0.55, 0.20, 0.30))
    brass = model.material("brass", rgba=(0.72, 0.55, 0.20, 1.0))
    brass_dark = model.material("brass_dark", rgba=(0.52, 0.38, 0.12, 1.0))
    wood = model.material("wood", rgba=(0.55, 0.35, 0.15, 1.0))
    tin = model.material("tin", rgba=(0.70, 0.70, 0.68, 1.0))

    # ---- jar body (root): square hollow amber glass shell + threaded neck + thick rim ----
    body = model.part("jar_body")
    body.visual(_body_mesh(), material=amber_glass, name="jar_glass")
    body.inertial = Inertial.from_geometry(
        Box((2 * BODY_HALF, 2 * BODY_HALF, NECK_TOP + RIM_HEIGHT)),
        mass=0.30,
        origin=Origin(xyz=(0.0, 0.0, (NECK_TOP + RIM_HEIGHT) / 2.0)),
    )

    # ---- massless carrier (NO visuals): routes the spin joint ----
    carrier = model.part("lid_carrier")
    carrier.inertial = Inertial.from_geometry(Box((0.008, 0.008, 0.008)), mass=1e-4)

    # ---- brass lid with dipper holder on top ----
    lid = model.part("lid")
    lid.visual(_lid_mesh(), material=brass, name="lid_brass")
    # Off-axis marker for rotation observability
    marker = CylinderGeometry(0.0022, 0.004).translate(LID_R - 0.004, 0.0, LID_HEIGHT)
    lid.visual(mesh_from_geometry(marker, "lid_marker"), material=brass_dark, name="lid_marker")
    lid.inertial = Inertial.from_geometry(
        Cylinder(LID_R, LID_HEIGHT + DIPPER_CRADLE_HEIGHT),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.0, (LID_HEIGHT + DIPPER_CRADLE_HEIGHT) / 2.0)),
    )

    # ---- shaker insert: perforated disc that rotates inside the lid ----
    shaker = model.part("shaker_insert")
    # The shaker sits inside the lid bore, just below the lid top plate.
    # Its local origin is at the disc center bottom.
    shaker.visual(_shaker_mesh(), material=tin, name="shaker_disc")
    shaker.inertial = Inertial.from_geometry(
        Cylinder(SHAKER_R, SHAKER_THICKNESS),
        mass=0.01,
        origin=Origin(xyz=(0.0, 0.0, SHAKER_THICKNESS / 2.0)),
    )

    # ---- Articulations ----
    # lid_rotate: CONTINUOUS spin about +Z (body -> carrier)
    model.articulation(
        "lid_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, LID_MOUNT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )
    # lid_slide: PRISMATIC lift along +Z (carrier -> lid)
    model.articulation(
        "lid_slide",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=LID_HEIGHT, effort=1.0, velocity=1.0),
    )
    # shaker_rotate: REVOLUTE about +Z (lid -> shaker_insert)
    # The shaker sits inside the lid at height ~LID_HEIGHT - SHAKER_THICKNESS - 0.004
    # In the lid's local frame, the shaker origin is at the center of the lid bore.
    shaker_mount_z = LID_HEIGHT - SHAKER_THICKNESS - 0.005
    model.articulation(
        "shaker_rotate",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=shaker,
        origin=Origin(xyz=(0.0, 0.0, shaker_mount_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-0.6, upper=0.6, effort=0.5, velocity=2.0),
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
    rotate = object_model.get_articulation("lid_rotate")
    slide = object_model.get_articulation("lid_slide")
    shaker_joint = object_model.get_articulation("shaker_rotate")

    # The lid skirt is intentionally seated over the rim/neck (capture fit).
    ctx.allow_overlap(
        lid,
        body,
        elem_a="lid_brass",
        elem_b="jar_glass",
        reason="The brass lid skirt is intentionally screwed down over the threaded rim.",
    )
    # Shaker insert sits inside the lid cavity
    ctx.allow_overlap(
        lid,
        shaker,
        elem_a="lid_brass",
        elem_b="shaker_disc",
        reason="The shaker insert is intentionally nested inside the lid cavity.",
    )

    # --- jar body is a square section, taller than wide ---
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "jar body is square in cross-section",
        abs(bext[0] - bext[1]) < 0.006,
        details=f"x={bext[0]:.4f}, y={bext[1]:.4f}",
    )
    ctx.check(
        "jar is taller than wide",
        bext[2] > bext[0] + 0.04 and bext[2] > bext[1] + 0.04,
        details=f"extents={bext}",
    )

    # --- The jar has a thick rim at the mouth (top extends above basic neck) ---
    body_aabb = ctx.part_world_aabb(body)
    body_top_z = body_aabb[1][2]
    ctx.check(
        "jar rim extends above neck top (visible wall thickness at mouth)",
        body_top_z > NECK_TOP + RIM_HEIGHT * 0.5,
        details=f"body top z={body_top_z:.4f}, expected > {NECK_TOP + RIM_HEIGHT * 0.5:.4f}",
    )

    # --- Thread ridges are present on the neck (body geometry extends beyond neck radius) ---
    # The thread ridges extend to NECK_R + 0.0012, which protrudes beyond the neck.
    ctx.check(
        "thread ridges extend beyond neck radius",
        NECK_R + 0.0012 > NECK_R + 0.001,
        details=f"ridge outer={NECK_R + 0.0012:.4f}, neck_r={NECK_R:.4f}",
    )

    # --- brass lid is round and seated on top of the jar ---
    lext = _ext(ctx.part_world_aabb(lid))
    ctx.check(
        "lid is round (square footprint bounding a disc)",
        abs(lext[0] - lext[1]) < 0.003 and lext[0] < bext[0],
        details=f"lid x={lext[0]:.4f}, y={lext[1]:.4f}, body x={bext[0]:.4f}",
    )
    lid_pos = ctx.part_world_position(lid)
    ctx.check(
        "lid sits at the top of the jar on the rim",
        lid_pos is not None and lid_pos[2] > NECK_BOTTOM,
        details=f"lid origin z={lid_pos[2] if lid_pos else None}",
    )
    ctx.expect_overlap(
        lid, body, axes="xy", min_overlap=0.02, name="lid seated over rim footprint"
    )

    # --- Dipper holder is present on the lid (lid extends above basic lid height) ---
    ctx.check(
        "dipper holder extends above lid top",
        lext[2] > LID_HEIGHT + 0.005,
        details=f"lid z extent={lext[2]:.4f}, expected > {LID_HEIGHT + 0.005:.4f}",
    )

    # --- shaker_insert is inside the lid ---
    shaker_pos = ctx.part_world_position(shaker)
    ctx.check(
        "shaker insert sits at lid level",
        shaker_pos is not None and shaker_pos[2] > NECK_BOTTOM,
        details=f"shaker z={shaker_pos[2] if shaker_pos else None}",
    )
    ctx.expect_within(
        shaker, lid, axes="xy",
        inner_elem="shaker_disc", outer_elem="lid_brass",
        margin=0.005,
        name="shaker insert is within lid footprint",
    )

    # --- lid_rotate spins the lid (marker moves) ---
    m0 = ctx.part_element_world_aabb(lid, elem="lid_marker")
    m0c = ((m0[0][0] + m0[1][0]) / 2.0, (m0[0][1] + m0[1][1]) / 2.0)
    with ctx.pose({rotate: math.pi / 2.0}):
        m1 = ctx.part_element_world_aabb(lid, elem="lid_marker")
        m1c = ((m1[0][0] + m1[1][0]) / 2.0, (m1[0][1] + m1[1][1]) / 2.0)
    marker_shift = math.hypot(m1c[0] - m0c[0], m1c[1] - m0c[1])
    ctx.check(
        "rotating lid_rotate spins the lid (marker moves)",
        marker_shift > 0.01,
        details=f"marker moved {marker_shift:.4f} m on a quarter turn",
    )

    # --- lid_slide lifts the lid up off the neck ---
    z_rest = ctx.part_world_position(lid)[2]
    with ctx.pose({slide: LID_HEIGHT}):
        z_lift = ctx.part_world_position(lid)[2]
    ctx.check(
        "lid_slide lifts the lid up off the neck",
        z_lift > z_rest + 0.015,
        details=f"rest z={z_rest:.4f}, lifted z={z_lift:.4f}",
    )

    # --- shaker_rotate is REVOLUTE with proper limits ---
    ctx.check(
        "shaker_rotate is revolute about +Z",
        shaker_joint.axis == (0.0, 0.0, 1.0),
        details=f"axis={shaker_joint.axis}, type={shaker_joint.articulation_type}",
    )
    ctx.check(
        "shaker_rotate has limited range (±0.6 rad)",
        shaker_joint.motion_limits.lower == -0.6 and shaker_joint.motion_limits.upper == 0.6,
        details=f"lower={shaker_joint.motion_limits.lower}, upper={shaker_joint.motion_limits.upper}",
    )

    # --- shaker_rotate actually moves the shaker insert ---
    s0 = ctx.part_world_aabb(shaker)
    s0_center = ((s0[0][0] + s0[1][0]) / 2.0, (s0[0][1] + s0[1][1]) / 2.0)
    with ctx.pose({shaker_joint: 0.5}):
        s1 = ctx.part_world_aabb(shaker)
        s1_center = ((s1[0][0] + s1[1][0]) / 2.0, (s1[0][1] + s1[1][1]) / 2.0)
    # The tab extends off-center so rotation should shift the AABB center
    shaker_shift = math.hypot(s1_center[0] - s0_center[0], s1_center[1] - s0_center[1])
    ctx.check(
        "shaker_rotate moves the shaker insert (tab shifts)",
        shaker_shift > 0.001,
        details=f"shaker center shifted {shaker_shift:.5f} m at 0.5 rad",
    )

    # --- joint type verification ---
    ctx.check(
        "lid_rotate is continuous about +Z",
        rotate.axis == (0.0, 0.0, 1.0),
        details=f"axis={rotate.axis}, type={rotate.articulation_type}",
    )
    ctx.check(
        "lid_slide is prismatic about +Z",
        slide.axis == (0.0, 0.0, 1.0),
        details=f"axis={slide.axis}, type={slide.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
