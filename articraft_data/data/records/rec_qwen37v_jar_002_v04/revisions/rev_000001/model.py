from __future__ import annotations

# Faceted glass storage jar with a metal screw lid.
# Variant of the square glass jar: octagonal faceted body, foot ring,
# thickened mouth rim with visible wall thickness, and rim seam geometry.
# Frame: jar stands on +XY ground plane, central axis along +Z (up).
#   - jar body: octagonal-section clear glass shell, hollow inside,
#     with a foot ring at the base and a tapered shoulder transitioning
#     to a round threaded neck with a thickened mouth rim. (root)
#   - metal screw lid: round knurled cap that screws onto the neck.
#     Modeled via two INDEPENDENT decoupled joints sharing +Z,
#     routed through a massless carrier link:
#       lid_rotate (CONTINUOUS, body->carrier): lid spins about +Z
#       lid_slide  (PRISMATIC, carrier->lid):   lid lifts up off the neck

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
BODY_R = 0.042              # circumscribed radius of the octagonal section
FACETS = 8                  # number of facets (octagonal)
WALL = 0.0035               # glass wall thickness
FOOT_RING_HEIGHT = 0.006    # height of the base foot ring
FOOT_RING_OUTSET = 0.003    # how much the foot ring protrudes beyond body
BODY_Z0 = FOOT_RING_HEIGHT  # body starts above the foot ring
BODY_TOP = BODY_Z0 + 0.110  # top of the faceted body section
SHOULDER_TOP = BODY_TOP + 0.014  # top of the tapered shoulder
NECK_R = 0.027              # outer radius of the round threaded neck
NECK_TOP = SHOULDER_TOP + 0.022  # top of the neck (z)
NECK_BOTTOM = SHOULDER_TOP

# Mouth rim geometry (thickened rim at top of neck)
RIM_HEIGHT = 0.005          # height of the thickened mouth rim
RIM_OUTSET = 0.003          # how much the rim protrudes beyond the neck OD
RIM_TOP = NECK_TOP + RIM_HEIGHT

LID_R = 0.032               # metal lid skirt outer radius
LID_HEIGHT = 0.024          # full height of the lid skirt + top
SCALLOP_N = 24              # number of scallops on the knurled skirt
LID_MOUNT_Z = RIM_TOP - 0.016  # lid seats over the neck


def _octagonal_prism(radius: float, height: float, z_offset: float = 0.0) -> cq.Workplane:
    """Extrude a regular polygon (octagonal by default) as a solid prism."""
    return (
        cq.Workplane("XY")
        .workplane(offset=z_offset)
        .polygon(FACETS, 2 * radius)
        .extrude(height)
    )


def _body_solid() -> cq.Workplane:
    # Faceted octagonal glass jar with foot ring, hollow body, shoulder,
    # round neck, and thickened mouth rim.

    # Foot ring: a short octagonal ring at the base, slightly wider than body.
    foot = _octagonal_prism(BODY_R + FOOT_RING_OUTSET, FOOT_RING_HEIGHT, 0.0)

    # Main octagonal body
    body = _octagonal_prism(BODY_R, BODY_TOP - BODY_Z0, BODY_Z0)

    # Tapered shoulder: octagonal body top -> round neck base
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP)
        .polygon(FACETS, 2 * (BODY_R - 0.003))
        .workplane(offset=SHOULDER_TOP - BODY_TOP)
        .circle(NECK_R)
        .loft(ruled=False)
    )

    # Round neck
    neck = (
        cq.Workplane("XY")
        .workplane(offset=NECK_BOTTOM)
        .circle(NECK_R)
        .extrude(NECK_TOP - NECK_BOTTOM)
    )

    # Thickened mouth rim: a torus-like ring at the top of the neck
    rim = (
        cq.Workplane("XY")
        .workplane(offset=NECK_TOP)
        .circle(NECK_R + RIM_OUTSET)
        .circle(NECK_R - WALL)
        .extrude(RIM_HEIGHT)
    )

    solid = foot.union(body).union(shoulder).union(neck).union(rim)

    # Hollow cavity: subtract inner volume
    inner_body = _octagonal_prism(BODY_R - WALL, BODY_TOP - BODY_Z0 - WALL, BODY_Z0 + WALL)

    inner_shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP - 0.001)
        .polygon(FACETS, 2 * (BODY_R - 0.003 - WALL))
        .workplane(offset=(SHOULDER_TOP - BODY_TOP) + 0.001)
        .circle(NECK_R - WALL)
        .loft(ruled=False)
    )

    inner_neck = (
        cq.Workplane("XY")
        .workplane(offset=NECK_BOTTOM)
        .circle(NECK_R - WALL)
        .extrude((NECK_TOP - NECK_BOTTOM) + RIM_HEIGHT + 0.001)
    )

    cavity = inner_body.union(inner_shoulder).union(inner_neck)
    return solid.cut(cavity)


def _thread_ridges() -> cq.Workplane:
    # Helical-ish thread ridges on the neck (thin rings).
    rings = None
    for zc in (NECK_BOTTOM + 0.005, NECK_BOTTOM + 0.011, NECK_BOTTOM + 0.017):
        if zc + 0.002 > NECK_TOP:
            continue
        ring = (
            cq.Workplane("XY")
            .workplane(offset=zc)
            .circle(NECK_R + 0.0008)
            .circle(NECK_R - 0.0004)
            .extrude(0.0018)
        )
        rings = ring if rings is None else rings.union(ring)
    return rings


def _body_mesh():
    solid = _body_solid()
    threads = _thread_ridges()
    if threads is not None:
        solid = solid.union(threads)
    return mesh_from_cadquery(solid, "jar_glass")


def _foot_ring_mesh():
    """Separate visible foot ring with a slight chamfer at the base edge."""
    # A short octagonal foot ring with a visible chamfer on the bottom outer edge.
    foot_outer = _octagonal_prism(BODY_R + FOOT_RING_OUTSET, FOOT_RING_HEIGHT, 0.0)
    # Hollow out to match the jar's inner floor
    foot_inner = (
        cq.Workplane("XY")
        .polygon(FACETS, 2 * (BODY_R - WALL))
        .extrude(FOOT_RING_HEIGHT - WALL)
    )
    ring = foot_outer.cut(foot_inner)
    # Chamfer the bottom outer edges for a finished look
    try:
        ring = ring.edges("<Z").chamfer(0.001)
    except Exception:
        pass
    return mesh_from_cadquery(ring, "foot_ring")


def _rim_seam_mesh():
    """Visible rim seam: a thin ring at the junction between neck and rim."""
    seam_z = NECK_TOP - 0.001
    seam = (
        cq.Workplane("XY")
        .workplane(offset=seam_z)
        .circle(NECK_R + RIM_OUTSET + 0.001)
        .circle(NECK_R + RIM_OUTSET - 0.001)
        .extrude(0.002)
    )
    return mesh_from_cadquery(seam, "rim_seam")


def _lid_solid() -> cq.Workplane:
    # Round metal lid: flat top disc + knurled/scalloped cylindrical skirt
    skirt = (
        cq.Workplane("XY")
        .circle(LID_R)
        .extrude(LID_HEIGHT)
    )
    # Hollow underside so it caps over the neck
    bore = (
        cq.Workplane("XY")
        .circle(NECK_R - 0.0006)
        .extrude(LID_HEIGHT - 0.004)
    )
    lid = skirt.cut(bore)

    # Scallops / knurling
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

    # Slight chamfer on the top outer edge
    try:
        lid = lid.faces(">Z").edges().chamfer(0.0015)
    except Exception:
        pass
    return lid


def _lid_mesh():
    return mesh_from_cadquery(_lid_solid(), "lid_metal")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="faceted_glass_storage_jar")

    glass = model.material("clear_glass", rgba=(0.78, 0.85, 0.88, 0.22))
    glass_tint = model.material("glass_tint", rgba=(0.72, 0.82, 0.85, 0.30))
    metal = model.material("brushed_metal", rgba=(0.58, 0.58, 0.60, 1.0))
    metal_dark = model.material("metal_dark", rgba=(0.35, 0.35, 0.38, 1.0))

    # ---- jar body (root): faceted hollow glass shell + round threaded neck ----
    body = model.part("jar_body")
    body.visual(_body_mesh(), material=glass, name="jar_glass")
    body.visual(_foot_ring_mesh(), material=glass_tint, name="foot_ring")
    body.visual(_rim_seam_mesh(), material=glass_tint, name="rim_seam")
    body.inertial = Inertial.from_geometry(
        Box((2 * BODY_R, 2 * BODY_R, RIM_TOP)),
        mass=0.32,
        origin=Origin(xyz=(0.0, 0.0, RIM_TOP / 2.0)),
    )

    # ---- massless carrier (NO visuals): routes the spin joint ----
    carrier = model.part("lid_carrier")
    carrier.inertial = Inertial.from_geometry(Box((0.008, 0.008, 0.008)), mass=1e-4)

    # ---- metal screw lid: knurled skirt + off-axis marker ----
    lid = model.part("lid")
    lid.visual(_lid_mesh(), material=metal, name="lid_metal")
    # Off-axis marker so rotation is observable in tests.
    marker = CylinderGeometry(0.0022, 0.004).translate(LID_R - 0.004, 0.0, LID_HEIGHT)
    lid.visual(mesh_from_geometry(marker, "lid_marker"), material=metal_dark, name="lid_marker")
    lid.inertial = Inertial.from_geometry(
        Cylinder(LID_R, LID_HEIGHT),
        mass=0.045,
        origin=Origin(xyz=(0.0, 0.0, LID_HEIGHT / 2.0)),
    )

    # ---- two INDEPENDENT decoupled joints sharing +Z, through the carrier ----
    model.articulation(
        "lid_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, LID_MOUNT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )
    model.articulation(
        "lid_slide",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=LID_HEIGHT, effort=1.0, velocity=1.0),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("jar_body")
    lid = object_model.get_part("lid")
    rotate = object_model.get_articulation("lid_rotate")
    slide = object_model.get_articulation("lid_slide")

    # The lid skirt is intentionally seated over the round neck (capture fit).
    ctx.allow_overlap(
        lid,
        body,
        elem_a="lid_metal",
        elem_b="jar_glass",
        reason="The metal lid skirt is intentionally screwed down over the round neck.",
    )

    # --- jar body is faceted (octagonal) and taller than wide ---
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "jar body has near-equal XY extents (faceted symmetry)",
        abs(bext[0] - bext[1]) < 0.008,
        details=f"x={bext[0]:.4f}, y={bext[1]:.4f}",
    )
    ctx.check(
        "jar is taller than wide",
        bext[2] > bext[0] + 0.03 and bext[2] > bext[1] + 0.03,
        details=f"extents={bext}",
    )

    # --- foot ring exists and is at the base ---
    ctx.check(
        "foot ring visual exists on jar body",
        body.get_visual("foot_ring") is not None,
        details="foot_ring visual not found",
    )
    foot_aabb = ctx.part_element_world_aabb(body, elem="foot_ring")
    if foot_aabb:
        ctx.check(
            "foot ring sits at the base of the jar",
            foot_aabb[0][2] < 0.008,
            details=f"foot ring min z={foot_aabb[0][2]:.4f}",
        )

    # --- rim seam exists at the mouth ---
    ctx.check(
        "rim seam visual exists on jar body",
        body.get_visual("rim_seam") is not None,
        details="rim_seam visual not found",
    )
    rim_aabb = ctx.part_element_world_aabb(body, elem="rim_seam")
    if rim_aabb:
        ctx.check(
            "rim seam is near the top of the jar at the mouth",
            rim_aabb[0][2] > NECK_BOTTOM,
            details=f"rim seam min z={rim_aabb[0][2]:.4f}, neck_bottom={NECK_BOTTOM}",
        )

    # --- glass wall thickness at mouth: rim extends beyond neck radius ---
    if rim_aabb:
        rim_dx = rim_aabb[1][0] - rim_aabb[0][0]
        rim_dy = rim_aabb[1][1] - rim_aabb[0][1]
        ctx.check(
            "mouth rim is wider than the neck (visible wall thickness)",
            rim_dx > 2 * NECK_R + 0.002 and rim_dy > 2 * NECK_R + 0.002,
            details=f"rim dx={rim_dx:.4f}, dy={rim_dy:.4f}, 2*neck_r={2*NECK_R:.4f}",
        )

    # --- metal lid is round and seated on top of the jar ---
    lext = _ext(ctx.part_world_aabb(lid))
    ctx.check(
        "lid is round (near-equal XY footprint)",
        abs(lext[0] - lext[1]) < 0.004,
        details=f"lid x={lext[0]:.4f}, y={lext[1]:.4f}",
    )
    lid_pos = ctx.part_world_position(lid)
    ctx.check(
        "lid sits at the top of the jar on the neck",
        lid_pos is not None and lid_pos[2] > NECK_BOTTOM,
        details=f"lid origin z={lid_pos[2] if lid_pos else None}, neck_bottom={NECK_BOTTOM}",
    )
    ctx.expect_overlap(
        lid, body, axes="xy", min_overlap=0.02, name="lid seated over neck footprint"
    )

    # --- lid_rotate spins the lid: off-axis marker moves around +Z ---
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

    # --- joint types and axes ---
    ctx.check(
        "lid_rotate is continuous about +Z",
        rotate.axis == (0.0, 0.0, 1.0) and rotate.articulation_type == ArticulationType.CONTINUOUS,
        details=f"axis={rotate.axis}, type={rotate.articulation_type}",
    )
    ctx.check(
        "lid_slide is prismatic about +Z",
        slide.axis == (0.0, 0.0, 1.0) and slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"axis={slide.axis}, type={slide.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
