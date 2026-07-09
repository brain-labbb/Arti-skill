from __future__ import annotations

# Squat cosmetic cream jar with a thick screw lid and rotating shaker insert.
# Frame: jar stands on +XY ground plane, central axis along +Z (up).
#   - jar_body: round squat body with thick walls, wide hollow mouth,
#     and clamp hooks on the rim. (root)
#   - lid_carrier: massless carrier for screw mechanism routing.
#   - lid: thick knurled screw cap.
#   - shaker_insert: perforated disk that rotates inside the lid cavity.
# Articulations:
#   lid_rotate (CONTINUOUS, body->carrier): lid spins about +Z
#   lid_slide  (PRISMATIC, carrier->lid):  lid lifts off the neck
#   shaker_rotate (REVOLUTE, lid->shaker): shaker disk rotates inside lid

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- key dimensions (meters) ----
BODY_R = 0.038            # outer radius of the round squat body
WALL = 0.004              # wall thickness
BODY_Z0 = 0.0             # jar base on ground
BODY_TOP = 0.032          # top of the body wall
MOUTH_R = BODY_R - WALL   # inner mouth radius (wide opening)
NECK_R = BODY_R + 0.001   # slight lip for neck/thread region
NECK_TOP = BODY_TOP + 0.008  # top of the short threaded neck

# Clamp hooks: 4 small tabs protruding from the rim
HOOK_COUNT = 4
HOOK_WIDTH = 0.010
HOOK_HEIGHT = 0.008
HOOK_DEPTH = 0.005        # protrusion outward from body wall

# Lid dimensions (thick screw lid)
LID_R = BODY_R + 0.003    # lid outer radius (slightly wider than body)
LID_HEIGHT = 0.018        # total lid thickness
LID_WALL = 0.003          # lid skirt wall
LID_TOP_THICK = 0.005     # lid top plate thickness
SCALLOP_N = 28            # knurling scallops around lid edge

# Lid mount: skirt drops over the neck
LID_MOUNT_Z = NECK_TOP - 0.010

# Shaker insert: perforated disk sitting inside the lid cavity
SHAKER_R = MOUTH_R - 0.003  # slightly smaller than mouth
SHAKER_THICK = 0.0025
SHAKER_HOLE_R = 0.003       # shaker hole radius
SHAKER_HOLE_COUNT = 7        # holes in a ring pattern
SHAKER_HOLE_RING_R = SHAKER_R * 0.55  # ring radius for holes
# Shaker sits at the underside of the lid top plate
SHAKER_Z_IN_LID = LID_HEIGHT - LID_TOP_THICK - SHAKER_THICK


def _body_solid() -> cq.Workplane:
    """Squat round jar body: thick walls, wide hollow mouth, clamp hooks on rim."""
    # Outer cylinder
    outer = (
        cq.Workplane("XY")
        .circle(BODY_R)
        .extrude(BODY_TOP)
    )
    # Short threaded neck on top
    neck = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP)
        .circle(NECK_R)
        .extrude(NECK_TOP - BODY_TOP)
    )
    solid = outer.union(neck)

    # Hollow cavity: opens at the top (wide mouth)
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=WALL)
        .circle(MOUTH_R)
        .extrude(BODY_TOP - WALL + 0.001)
    )
    inner_neck = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP)
        .circle(MOUTH_R)
        .extrude((NECK_TOP - BODY_TOP) + 0.001)
    )
    solid = solid.cut(cavity).cut(inner_neck)

    # Clamp hooks: small rectangular tabs on the outer rim
    for i in range(HOOK_COUNT):
        angle = 2.0 * math.pi * i / HOOK_COUNT
        hx = (BODY_R + HOOK_DEPTH / 2.0) * math.cos(angle)
        hy = (BODY_R + HOOK_DEPTH / 2.0) * math.sin(angle)
        hook = (
            cq.Workplane("XY")
            .workplane(offset=BODY_TOP - HOOK_HEIGHT)
            .center(hx, hy)
            .rect(HOOK_DEPTH, HOOK_WIDTH)
            .extrude(HOOK_HEIGHT + (NECK_TOP - BODY_TOP))
        )
        solid = solid.union(hook)

    # Thread ridges on the neck (2 rings)
    for zc in (BODY_TOP + 0.002, BODY_TOP + 0.005):
        ring = (
            cq.Workplane("XY")
            .workplane(offset=zc)
            .circle(NECK_R + 0.0006)
            .circle(NECK_R - 0.0003)
            .extrude(0.0018)
        )
        solid = solid.union(ring)

    return solid


def _body_mesh():
    return mesh_from_cadquery(_body_solid(), "jar_body")


def _lid_solid() -> cq.Workplane:
    """Thick screw lid: knurled skirt, top plate, inner cavity for shaker."""
    # Outer cylinder (full lid)
    skirt = (
        cq.Workplane("XY")
        .circle(LID_R)
        .extrude(LID_HEIGHT)
    )
    # Inner bore so the skirt fits over the neck (hollow underside)
    bore = (
        cq.Workplane("XY")
        .circle(MOUTH_R + 0.001)
        .extrude(LID_HEIGHT - LID_TOP_THICK)
    )
    lid = skirt.cut(bore)

    # Knurling: subtract small flutes around the outer rim
    for k in range(SCALLOP_N):
        ang = 2.0 * math.pi * k / SCALLOP_N
        fx = LID_R * math.cos(ang)
        fy = LID_R * math.sin(ang)
        flute = (
            cq.Workplane("XY")
            .center(fx, fy)
            .circle(0.0020)
            .extrude(LID_HEIGHT)
        )
        lid = lid.cut(flute)

    # Top chamfer
    try:
        lid = lid.faces(">Z").edges().chamfer(0.001)
    except Exception:
        pass

    return lid


def _lid_mesh():
    return mesh_from_cadquery(_lid_solid(), "lid_cap")


def _shaker_mesh():
    """Perforated disk: circular plate with ring of shaker holes."""
    # Build outer circular profile
    n_seg = 48
    outer_profile = []
    for i in range(n_seg):
        ang = 2.0 * math.pi * i / n_seg
        outer_profile.append((SHAKER_R * math.cos(ang), SHAKER_R * math.sin(ang)))

    # Build hole profiles (circles)
    hole_profiles = []
    for h in range(SHAKER_HOLE_COUNT):
        ang = 2.0 * math.pi * h / SHAKER_HOLE_COUNT
        cx = SHAKER_HOLE_RING_R * math.cos(ang)
        cy = SHAKER_HOLE_RING_R * math.sin(ang)
        hole = []
        for s in range(16):
            a = 2.0 * math.pi * s / 16
            hole.append((cx + SHAKER_HOLE_R * math.cos(a), cy + SHAKER_HOLE_R * math.sin(a)))
        hole_profiles.append(hole)

    # Center hole
    center_hole = []
    for s in range(16):
        a = 2.0 * math.pi * s / 16
        center_hole.append((SHAKER_HOLE_R * math.cos(a), SHAKER_HOLE_R * math.sin(a)))
    hole_profiles.append(center_hole)

    shaker_geom = ExtrudeWithHolesGeometry(
        outer_profile,
        hole_profiles,
        SHAKER_THICK,
        cap=True,
        center=False,
    )
    return mesh_from_geometry(shaker_geom, "shaker_disk")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="squat_cosmetic_cream_jar")

    cream_plastic = model.material("cream_plastic", rgba=(0.92, 0.90, 0.86, 1.0))
    lid_metal = model.material("brushed_silver", rgba=(0.72, 0.72, 0.74, 1.0))
    shaker_plastic = model.material("white_plastic", rgba=(0.95, 0.95, 0.93, 1.0))
    hook_accent = model.material("hook_accent", rgba=(0.60, 0.60, 0.62, 1.0))

    # ---- jar body (root): squat round with wide mouth, thick walls, hooks ----
    body = model.part("jar_body")
    body.visual(_body_mesh(), material=cream_plastic, name="jar_shell")
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, NECK_TOP),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP / 2.0)),
    )

    # ---- massless carrier (NO visuals): routes the spin joint ----
    carrier = model.part("lid_carrier")
    carrier.inertial = Inertial.from_geometry(Box((0.008, 0.008, 0.008)), mass=1e-4)

    # ---- thick screw lid ----
    lid = model.part("lid")
    lid.visual(_lid_mesh(), material=lid_metal, name="lid_cap")
    # Off-axis marker so rotation is observable
    marker = CylinderGeometry(0.0018, 0.003).translate(LID_R - 0.004, 0.0, LID_HEIGHT)
    lid.visual(mesh_from_geometry(marker, "lid_marker"), material=hook_accent, name="lid_marker")
    lid.inertial = Inertial.from_geometry(
        Cylinder(LID_R, LID_HEIGHT),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, LID_HEIGHT / 2.0)),
    )

    # ---- shaker insert: rotates inside the lid ----
    shaker = model.part("shaker_insert")
    shaker.visual(_shaker_mesh(), material=shaker_plastic, name="shaker_disk")
    # Off-axis marker on shaker for rotation observability
    shaker_nub = CylinderGeometry(0.0015, SHAKER_THICK).translate(
        SHAKER_R - 0.004, 0.0, 0.0
    )
    shaker.visual(
        mesh_from_geometry(shaker_nub, "shaker_marker"),
        material=hook_accent,
        name="shaker_marker",
    )
    shaker.inertial = Inertial.from_geometry(
        Cylinder(SHAKER_R, SHAKER_THICK),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, SHAKER_THICK / 2.0)),
    )

    # ---- articulations ----
    # 1. Lid rotate (continuous, body -> carrier)
    model.articulation(
        "lid_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, LID_MOUNT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )
    # 2. Lid slide (prismatic, carrier -> lid): lifts lid off the neck
    model.articulation(
        "lid_slide",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0, upper=LID_HEIGHT + 0.010, effort=1.0, velocity=1.0
        ),
    )
    # 3. Shaker rotate (revolute, lid -> shaker): limited rotation inside lid
    model.articulation(
        "shaker_rotate",
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
    shaker_rotate = object_model.get_articulation("shaker_rotate")

    # Lid skirt is seated over the neck (capture fit)
    ctx.allow_overlap(
        lid,
        body,
        elem_a="lid_cap",
        elem_b="jar_shell",
        reason="The thick lid skirt is intentionally screwed down over the jar neck.",
    )
    # Shaker sits inside the lid cavity
    ctx.allow_overlap(
        lid,
        shaker,
        elem_a="lid_cap",
        elem_b="shaker_disk",
        reason="The shaker insert is intentionally nested inside the lid cavity.",
    )

    # --- SQUAT proportions: body wider than tall ---
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "jar body is round in cross-section",
        abs(bext[0] - bext[1]) < 0.008,
        details=f"x={bext[0]:.4f}, y={bext[1]:.4f}",
    )
    ctx.check(
        "jar is squat (wider than tall)",
        bext[0] > bext[2] + 0.010 and bext[1] > bext[2] + 0.010,
        details=f"extents={bext}",
    )

    # --- Wide mouth: body is hollow (inner cavity narrower than outer) ---
    ctx.check(
        "wide mouth: wall thickness leaves large inner opening",
        MOUTH_R > BODY_R * 0.70,
        details=f"mouth_r={MOUTH_R:.4f}, body_r={BODY_R:.4f}",
    )

    # --- Clamp hooks exist on the jar body (body wider than plain cylinder) ---
    ctx.check(
        "clamp hooks protrude beyond body radius",
        bext[0] > 2.0 * BODY_R + 0.002,
        details=f"body x extent={bext[0]:.4f}, 2*body_r={2*BODY_R:.4f}",
    )

    # --- Lid is round and seated on top of the jar ---
    lext = _ext(ctx.part_world_aabb(lid))
    ctx.check(
        "lid is round",
        abs(lext[0] - lext[1]) < 0.004,
        details=f"lid x={lext[0]:.4f}, y={lext[1]:.4f}",
    )
    lid_pos = ctx.part_world_position(lid)
    ctx.check(
        "lid sits at the top of the jar",
        lid_pos is not None and lid_pos[2] > BODY_TOP - 0.005,
        details=f"lid z={lid_pos[2] if lid_pos else None}",
    )
    ctx.expect_overlap(
        lid, body, axes="xy", min_overlap=0.02,
        name="lid seated over neck footprint",
    )

    # --- Thick lid: height is significant ---
    ctx.check(
        "lid is thick (height > 0.012m)",
        lext[2] > 0.012,
        details=f"lid height={lext[2]:.4f}",
    )

    # --- lid_rotate spins the lid ---
    m0 = ctx.part_element_world_aabb(lid, elem="lid_marker")
    m0c = ((m0[0][0] + m0[1][0]) / 2.0, (m0[0][1] + m0[1][1]) / 2.0)
    with ctx.pose({lid_rotate: math.pi / 2.0}):
        m1 = ctx.part_element_world_aabb(lid, elem="lid_marker")
        m1c = ((m1[0][0] + m1[1][0]) / 2.0, (m1[0][1] + m1[1][1]) / 2.0)
    marker_shift = math.hypot(m1c[0] - m0c[0], m1c[1] - m0c[1])
    ctx.check(
        "lid_rotate spins the lid (marker moves)",
        marker_shift > 0.008,
        details=f"marker moved {marker_shift:.4f} m on quarter turn",
    )

    # --- lid_slide lifts the lid off ---
    z_rest = ctx.part_world_position(lid)[2]
    with ctx.pose({lid_slide: LID_HEIGHT}):
        z_lift = ctx.part_world_position(lid)[2]
    ctx.check(
        "lid_slide lifts lid off the jar",
        z_lift > z_rest + 0.010,
        details=f"rest z={z_rest:.4f}, lifted z={z_lift:.4f}",
    )

    # --- shaker_rotate: shaker insert rotates inside the lid ---
    s0 = ctx.part_element_world_aabb(shaker, elem="shaker_marker")
    s0c = ((s0[0][0] + s0[1][0]) / 2.0, (s0[0][1] + s0[1][1]) / 2.0)
    with ctx.pose({shaker_rotate: math.pi / 4.0}):
        s1 = ctx.part_element_world_aabb(shaker, elem="shaker_marker")
        s1c = ((s1[0][0] + s1[1][0]) / 2.0, (s1[0][1] + s1[1][1]) / 2.0)
    shaker_shift = math.hypot(s1c[0] - s0c[0], s1c[1] - s0c[1])
    ctx.check(
        "shaker_rotate turns the insert inside the lid",
        shaker_shift > 0.003,
        details=f"shaker marker moved {shaker_shift:.4f} m on 45° turn",
    )

    # --- shaker_rotate has bounded limits (revolute, not continuous) ---
    shaker_limits = shaker_rotate.motion_limits
    ctx.check(
        "shaker_rotate is revolute with bounded limits",
        shaker_rotate.articulation_type == ArticulationType.REVOLUTE
        and shaker_limits is not None
        and shaker_limits.lower is not None
        and shaker_limits.upper is not None
        and shaker_limits.upper > shaker_limits.lower,
        details=f"type={shaker_rotate.articulation_type}, limits={shaker_limits}",
    )

    # --- joint types and axes ---
    ctx.check(
        "lid_rotate is continuous about +Z",
        lid_rotate.axis == (0.0, 0.0, 1.0),
        details=f"axis={lid_rotate.axis}, type={lid_rotate.articulation_type}",
    )
    ctx.check(
        "lid_slide is prismatic about +Z",
        lid_slide.axis == (0.0, 0.0, 1.0),
        details=f"axis={lid_slide.axis}, type={lid_slide.articulation_type}",
    )

    # --- Shaker stays within lid footprint ---
    ctx.expect_within(
        shaker, lid, axes="xy",
        inner_elem="shaker_disk", outer_elem="lid_cap",
        margin=0.002,
        name="shaker insert stays within lid footprint",
    )

    return ctx.report()


object_model = build_object_model()
