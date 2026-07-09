from __future__ import annotations

# Squat cosmetic cream jar with a thick screw lid and gasket seal.
# Variant of the square glass storage jar, reworked as a round cosmetic
# cream jar:
#   - jar_body (root): round hollow squat body with wide-mouth threaded neck,
#     plus a visible gasket ring seated in a rim groove.
#   - lid_carrier (massless): routes the continuous spin joint.
#   - lid: thick round knurled screw cap that fits over the neck threads.
# Screw-cap mechanism uses two independent decoupled joints sharing +Z:
#   lid_rotate (CONTINUOUS, body->carrier): lid spins freely about +Z
#   lid_slide  (PRISMATIC, carrier->lid):   lid lifts up off the neck

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
# Squat round cosmetic cream jar proportions.
BODY_R = 0.0330              # outer radius of the round body
BODY_FILLET = 0.004          # bottom-edge rounding
WALL = 0.003                 # wall thickness
BODY_Z0 = 0.0                # jar base sits on the ground
BODY_TOP = 0.032             # top of the round body section (squat)

# Wide mouth: neck is ~85% of body diameter.
NECK_R = 0.0280              # outer radius of the threaded neck
NECK_BOTTOM = BODY_TOP       # neck starts at top of body
NECK_TOP = 0.042             # top of the neck (z)
NECK_HEIGHT = NECK_TOP - NECK_BOTTOM  # 0.010 m

# Gasket groove: sits in a rim groove at the top of the body, just below the neck.
GASKET_R_OUTER = 0.0310      # gasket outer radius (flush with body inner rim)
GASKET_R_INNER = 0.0285      # gasket inner radius (just outside neck)
GASKET_THICK = 0.0025        # gasket height
GASKET_Z = BODY_TOP - GASKET_THICK  # sits at top of body, recessed slightly

# Thick screw lid.
LID_R = 0.0340               # lid outer radius (slightly wider than body for grip)
LID_HEIGHT = 0.022           # total lid height (thick cap)
LID_TOP_THICK = 0.005        # solid top plate thickness
SCALLOP_N = 28               # number of knurling scallops on the skirt
SCALLOP_R = 0.0022           # flute radius for knurling

# Lid mount: skirt drops over the neck; lid local origin at skirt bottom (z=0).
LID_MOUNT_Z = NECK_TOP - 0.012


def _body_solid() -> cq.Workplane:
    """Round squat hollow glass/plastic jar with wide-mouth threaded neck."""
    # Main body: round cylinder with filleted bottom edge.
    body = (
        cq.Workplane("XY")
        .circle(BODY_R)
        .extrude(BODY_TOP)
    )
    try:
        body = body.edges("<Z").fillet(BODY_FILLET)
    except Exception:
        pass

    # Wide-mouth neck cylinder on top.
    neck = (
        cq.Workplane("XY")
        .workplane(offset=NECK_BOTTOM)
        .circle(NECK_R)
        .extrude(NECK_HEIGHT)
    )

    # Shoulder bridge: a solid disc spanning the body-top/neck-base junction
    # so the body wall (r=0.030–0.033) and neck wall (r=0.025–0.028) remain
    # one connected solid after the cavity cut.
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP - 0.001)
        .circle(BODY_R)
        .extrude(0.002)
    )

    solid = body.union(shoulder).union(neck)

    # Hollow out: inner cavity opens at the neck top (wide mouth).
    inner_body = (
        cq.Workplane("XY")
        .workplane(offset=WALL)
        .circle(BODY_R - WALL)
        .extrude(BODY_TOP - WALL)
    )
    inner_neck = (
        cq.Workplane("XY")
        .workplane(offset=NECK_BOTTOM)
        .circle(NECK_R - WALL)
        .extrude(NECK_HEIGHT + 0.001)
    )
    cavity = inner_body.union(inner_neck)
    return solid.cut(cavity)


def _thread_ridges() -> cq.Workplane:
    """Thread ridges on the wide neck so it reads as a screw neck."""
    rings = None
    z_positions = [
        NECK_BOTTOM + 0.003,
        NECK_BOTTOM + 0.007,
    ]
    for zc in z_positions:
        ring = (
            cq.Workplane("XY")
            .workplane(offset=zc)
            .circle(NECK_R + 0.0008)
            .circle(NECK_R - 0.0004)
            .extrude(0.0018)
        )
        rings = ring if rings is None else rings.union(ring)
    return rings


def _gasket_ring() -> cq.Workplane:
    """Gasket ring: a rubber/silicone annular seal sitting in the rim groove."""
    gasket = (
        cq.Workplane("XY")
        .workplane(offset=GASKET_Z)
        .circle(GASKET_R_OUTER)
        .circle(GASKET_R_INNER)
        .extrude(GASKET_THICK)
    )
    return gasket


def _body_mesh():
    solid = _body_solid().union(_thread_ridges())
    return mesh_from_cadquery(solid, "jar_body_shell")


def _gasket_mesh():
    return mesh_from_cadquery(_gasket_ring(), "gasket_ring")


def _lid_solid() -> cq.Workplane:
    """Thick round knurled screw cap for a cosmetic cream jar."""
    # Outer skirt cylinder.
    skirt = (
        cq.Workplane("XY")
        .circle(LID_R)
        .extrude(LID_HEIGHT)
    )
    # Hollow bore so the cap fits over the neck.
    bore = (
        cq.Workplane("XY")
        .circle(NECK_R - 0.0006)
        .extrude(LID_HEIGHT - LID_TOP_THICK)
    )
    lid = skirt.cut(bore)

    # Knurling: subtract small vertical flutes around the outer rim.
    for k in range(SCALLOP_N):
        ang = 2.0 * math.pi * k / SCALLOP_N
        fx = (LID_R - 0.0002) * math.cos(ang)
        fy = (LID_R - 0.0002) * math.sin(ang)
        flute = (
            cq.Workplane("XY")
            .center(fx, fy)
            .circle(SCALLOP_R)
            .extrude(LID_HEIGHT)
        )
        lid = lid.cut(flute)

    # Top edge chamfer for realism.
    try:
        lid = lid.faces(">Z").edges().chamfer(0.0012)
    except Exception:
        pass

    return lid


def _lid_mesh():
    return mesh_from_cadquery(_lid_solid(), "lid_cap")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="squat_cosmetic_cream_jar")

    # Materials
    frosted = model.material("frosted_glass", rgba=(0.88, 0.90, 0.92, 0.35))
    matte_silver = model.material("matte_silver", rgba=(0.78, 0.78, 0.80, 1.0))
    dark_accent = model.material("dark_accent", rgba=(0.35, 0.35, 0.38, 1.0))
    rubber_grey = model.material("rubber_grey", rgba=(0.42, 0.42, 0.40, 1.0))

    # ---- jar body (root): round hollow squat shell + wide threaded neck ----
    body = model.part("jar_body")
    body.visual(_body_mesh(), material=frosted, name="jar_shell")
    body.visual(_gasket_mesh(), material=rubber_grey, name="gasket_ring")
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, BODY_TOP + NECK_HEIGHT),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, (BODY_TOP + NECK_HEIGHT) / 2.0)),
    )

    # ---- massless carrier: routes the spin joint ----
    carrier = model.part("lid_carrier")
    carrier.inertial = Inertial.from_geometry(Box((0.008, 0.008, 0.008)), mass=1e-4)

    # ---- thick screw lid: knurled cap + off-axis rotation marker ----
    lid = model.part("lid")
    lid.visual(_lid_mesh(), material=matte_silver, name="lid_cap")
    # Off-axis marker so rotation is observable in tests.
    marker = CylinderGeometry(0.002, 0.0035).translate(LID_R - 0.005, 0.0, LID_HEIGHT)
    lid.visual(
        mesh_from_geometry(marker, "lid_marker"),
        material=dark_accent,
        name="lid_marker",
    )
    lid.inertial = Inertial.from_geometry(
        Cylinder(LID_R, LID_HEIGHT),
        mass=0.035,
        origin=Origin(xyz=(0.0, 0.0, LID_HEIGHT / 2.0)),
    )

    # ---- articulations: continuous screw + prismatic lift ----
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
        motion_limits=MotionLimits(
            lower=0.0, upper=LID_HEIGHT + 0.005, effort=1.0, velocity=1.0
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
    rotate = object_model.get_articulation("lid_rotate")
    slide = object_model.get_articulation("lid_slide")

    # The lid skirt intentionally seats over the neck (capture fit).
    ctx.allow_overlap(
        lid,
        body,
        elem_a="lid_cap",
        elem_b="jar_shell",
        reason="The thick lid skirt is intentionally screwed down over the wide neck.",
    )

    # --- jar body is round and squat (wider than tall) ---
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "jar body is round in cross-section",
        abs(bext[0] - bext[1]) < 0.006,
        details=f"x={bext[0]:.4f}, y={bext[1]:.4f}",
    )
    ctx.check(
        "jar is squat (wider than tall)",
        bext[0] > bext[2] and bext[1] > bext[2],
        details=f"extents=({bext[0]:.4f}, {bext[1]:.4f}, {bext[2]:.4f})",
    )

    # --- wide mouth: neck is close to body width ---
    mouth_ratio = NECK_R / BODY_R
    ctx.check(
        "wide mouth opening (neck > 75% of body radius)",
        mouth_ratio > 0.75,
        details=f"neck_r={NECK_R}, body_r={BODY_R}, ratio={mouth_ratio:.2f}",
    )

    # --- gasket ring is present on the jar body ---
    gasket_visual = body.get_visual("gasket_ring")
    ctx.check(
        "gasket ring visual exists on jar body",
        gasket_visual is not None,
        details="gasket_ring visual not found on jar_body",
    )

    # --- lid is round and thick, seated on top of the jar ---
    lext = _ext(ctx.part_world_aabb(lid))
    ctx.check(
        "lid is round (equal x/y extents)",
        abs(lext[0] - lext[1]) < 0.004,
        details=f"lid x={lext[0]:.4f}, y={lext[1]:.4f}",
    )
    lid_pos = ctx.part_world_position(lid)
    ctx.check(
        "lid sits at the top of the jar on the neck",
        lid_pos is not None and lid_pos[2] > BODY_TOP - 0.005,
        details=f"lid origin z={lid_pos[2] if lid_pos else None}",
    )
    ctx.expect_overlap(
        lid, body, axes="xy", min_overlap=0.02,
        name="lid seated over neck footprint",
    )

    # --- lid_rotate spins the lid (marker moves on quarter turn) ---
    m0 = ctx.part_element_world_aabb(lid, elem="lid_marker")
    m0c = ((m0[0][0] + m0[1][0]) / 2.0, (m0[0][1] + m0[1][1]) / 2.0)
    with ctx.pose({rotate: math.pi / 2.0}):
        m1 = ctx.part_element_world_aabb(lid, elem="lid_marker")
        m1c = ((m1[0][0] + m1[1][0]) / 2.0, (m1[0][1] + m1[1][1]) / 2.0)
    marker_shift = math.hypot(m1c[0] - m0c[0], m1c[1] - m0c[1])
    ctx.check(
        "rotating lid_rotate spins the lid (marker moves)",
        marker_shift > 0.008,
        details=f"marker moved {marker_shift:.4f} m on a quarter turn",
    )

    # --- lid_slide lifts the lid up off the neck ---
    z_rest = ctx.part_world_position(lid)[2]
    with ctx.pose({slide: LID_HEIGHT}):
        z_lift = ctx.part_world_position(lid)[2]
    ctx.check(
        "lid_slide lifts the lid up off the neck",
        z_lift > z_rest + 0.012,
        details=f"rest z={z_rest:.4f}, lifted z={z_lift:.4f}",
    )

    # --- joint types and axes ---
    ctx.check(
        "lid_rotate is continuous about +Z",
        rotate.axis == (0.0, 0.0, 1.0)
        and rotate.articulation_type == ArticulationType.CONTINUOUS,
        details=f"axis={rotate.axis}, type={rotate.articulation_type}",
    )
    ctx.check(
        "lid_slide is prismatic along +Z",
        slide.axis == (0.0, 0.0, 1.0)
        and slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"axis={slide.axis}, type={slide.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
