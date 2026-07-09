from __future__ import annotations

# Honey jar with a dipper holder on the brass screw lid.
# Frame: jar stands on +XY ground plane, central axis along +Z (up).
#   - jar_body: square-section amber glass shell with rounded vertical edges,
#     hollow inside, topped by a short round threaded neck with a visible
#     thickened glass mouth rim. (root)
#   - lid_carrier: massless carrier link routing the spin joint.
#   - lid: round brass knurled cap with a central dipper-holder hole and raised
#     collar on top.
#   - dipper: wooden honey dipper (grooved ball + handle) seated through the
#     lid hole into the jar.
# Articulations:
#   lid_rotate (CONTINUOUS, body->carrier): continuous screw spin about +Z.
#   lid_slide  (PRISMATIC, carrier->lid):   lid lifts up off the neck.

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
BODY_HALF = 0.042          # half-width of the square section (0.084 m square)
BODY_FILLET = 0.013        # rounded vertical-edge radius
WALL = 0.0038              # glass wall thickness
BODY_Z0 = 0.0              # jar base sits on the ground
BODY_TOP = 0.110           # top of the square body section
SHOULDER_TOP = 0.124       # top of the tapered shoulder where the neck begins
NECK_R = 0.028             # outer radius of the round threaded neck
NECK_TOP = 0.148           # top of the neck (z)
NECK_BOTTOM = SHOULDER_TOP

# Mouth rim: thickened glass lip at the top of the neck
MOUTH_RIM_OUTER = NECK_R + 0.003   # outer radius of the mouth rim
MOUTH_RIM_HEIGHT = 0.005           # height of the raised mouth rim
MOUTH_RIM_BOTTOM = NECK_TOP - MOUTH_RIM_HEIGHT

LID_R = 0.032              # brass lid skirt outer radius
LID_HEIGHT = 0.026         # full height of the lid skirt + top
LID_TOP_THICK = 0.004      # thickness of the lid top plate
SCALLOP_N = 24             # number of scallops on the knurled skirt

# Dipper holder: hole through the lid top + raised collar
DIPPER_HOLE_R = 0.0045     # radius of the dipper handle hole
DIPPER_COLLAR_R = 0.008    # outer radius of the raised collar around the hole
DIPPER_COLLAR_H = 0.006    # height of the collar above the lid top

# Lid mount height: skirt drops over the neck+rim so its bore wraps the neck top
LID_MOUNT_Z = NECK_TOP - 0.018

# Dipper dimensions
DIPPER_HANDLE_R = 0.004    # handle radius
DIPPER_HANDLE_LEN = 0.10   # handle length
DIPPER_BALL_R = 0.012      # grooved ball radius at the bottom
DIPPER_BALL_Z = 0.04       # center of ball above jar base (inside the jar)


def _body_solid() -> cq.Workplane:
    """Hollow square glass jar body with rounded edges, shoulder, neck, and
    a visible thickened glass mouth rim."""
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

    # Thickened mouth rim: a ring at the top of the neck showing glass thickness.
    mouth_rim = (
        cq.Workplane("XY")
        .workplane(offset=MOUTH_RIM_BOTTOM)
        .circle(MOUTH_RIM_OUTER)
        .circle(NECK_R - WALL)
        .extrude(MOUTH_RIM_HEIGHT)
    )

    solid = outer.union(shoulder).union(neck).union(mouth_rim)

    # Hollow it out: inner cavity opens at the neck top through the mouth rim.
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
        .extrude((NECK_TOP - NECK_BOTTOM) + MOUTH_RIM_HEIGHT + 0.001)
    )
    cavity = inner.union(inner_shoulder).union(inner_neck)
    return solid.cut(cavity)


def _thread_ridges() -> cq.Workplane:
    """Helical-ish thread ridges on the neck (thin rings)."""
    rings = None
    for zc in (NECK_BOTTOM + 0.006, NECK_BOTTOM + 0.013):
        ring = (
            cq.Workplane("XY")
            .workplane(offset=zc)
            .circle(NECK_R + 0.0008)
            .circle(NECK_R - 0.0004)
            .extrude(0.0022)
        )
        rings = ring if rings is None else rings.union(ring)
    return rings


def _body_mesh():
    solid = _body_solid().union(_thread_ridges())
    return mesh_from_cadquery(solid, "jar_glass")


def _lid_solid() -> cq.Workplane:
    """Round brass lid with scalloped skirt and a central dipper-holder hole
    with a raised collar on the lid top."""
    skirt = (
        cq.Workplane("XY")
        .circle(LID_R)
        .extrude(LID_HEIGHT)
    )
    # Hollow underside so it caps over the neck (leave a top plate).
    bore = (
        cq.Workplane("XY")
        .circle(NECK_R - 0.0006)
        .extrude(LID_HEIGHT - LID_TOP_THICK)
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
            .circle(0.0024)
            .extrude(LID_HEIGHT)
        )
        lid = lid.cut(flute)

    # Dipper holder hole: cut through the lid top plate center.
    dipper_hole = (
        cq.Workplane("XY")
        .workplane(offset=LID_HEIGHT - LID_TOP_THICK - 0.001)
        .circle(DIPPER_HOLE_R)
        .extrude(LID_TOP_THICK + 0.002)
    )
    lid = lid.cut(dipper_hole)

    # Raised collar around the dipper hole on top of the lid.
    collar = (
        cq.Workplane("XY")
        .workplane(offset=LID_HEIGHT)
        .circle(DIPPER_COLLAR_R)
        .circle(DIPPER_HOLE_R)
        .extrude(DIPPER_COLLAR_H)
    )
    lid = lid.union(collar)

    # Slight chamfer on the top outer edge.
    try:
        lid = lid.faces(">Z").edges().chamfer(0.0012)
    except Exception:
        pass
    return lid


def _lid_mesh():
    return mesh_from_cadquery(_lid_solid(), "lid_brass")


def _dipper_solid() -> cq.Workplane:
    """Wooden honey dipper: a cylindrical handle with a grooved ball at the
    bottom end. The dipper sits through the lid hole into the jar."""
    # Handle: long thin cylinder
    handle = (
        cq.Workplane("XY")
        .circle(DIPPER_HANDLE_R)
        .extrude(DIPPER_HANDLE_LEN)
    )

    # Ball at the bottom (below handle origin): a sphere
    ball = (
        cq.Workplane("XY")
        .workplane(offset=-DIPPER_BALL_R)
        .sphere(DIPPER_BALL_R)
    )

    # Grooves on the ball: subtract thin horizontal rings
    dipper = handle.union(ball)
    for i in range(4):
        groove_z = -DIPPER_BALL_R + (i + 1) * (2 * DIPPER_BALL_R / 5.0)
        groove_depth = 0.0015
        groove = (
            cq.Workplane("XY")
            .workplane(offset=groove_z - 0.001)
            .circle(DIPPER_BALL_R + 0.001)
            .circle(DIPPER_BALL_R - groove_depth)
            .extrude(0.002)
        )
        dipper = dipper.cut(groove)

    # Small rounded knob at the very top of the handle
    top_knob = (
        cq.Workplane("XY")
        .workplane(offset=DIPPER_HANDLE_LEN)
        .sphere(DIPPER_HANDLE_R * 1.3)
    )
    dipper = dipper.union(top_knob)

    return dipper


def _dipper_mesh():
    return mesh_from_cadquery(_dipper_solid(), "dipper_wood")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="honey_jar_with_dipper")

    # Amber/honey-tinted glass
    glass = model.material("amber_glass", rgba=(0.75, 0.52, 0.18, 0.35))
    brass = model.material("brass", rgba=(0.72, 0.55, 0.20, 1.0))
    brass_dark = model.material("brass_dark", rgba=(0.52, 0.38, 0.12, 1.0))
    wood = model.material("wood", rgba=(0.60, 0.40, 0.22, 1.0))

    # ---- jar body (root): square hollow amber glass shell + neck + mouth rim ----
    body = model.part("jar_body")
    body.visual(_body_mesh(), material=glass, name="jar_glass")
    body.inertial = Inertial.from_geometry(
        Box((2 * BODY_HALF, 2 * BODY_HALF, NECK_TOP + MOUTH_RIM_HEIGHT)),
        mass=0.32,
        origin=Origin(xyz=(0.0, 0.0, (NECK_TOP + MOUTH_RIM_HEIGHT) / 2.0)),
    )

    # ---- massless carrier (NO visuals): routes the spin joint ----
    carrier = model.part("lid_carrier")
    carrier.inertial = Inertial.from_geometry(Box((0.008, 0.008, 0.008)), mass=1e-4)

    # ---- brass screw lid: knurled skirt + dipper holder hole + collar ----
    lid = model.part("lid")
    lid.visual(_lid_mesh(), material=brass, name="lid_brass")
    # Off-axis marker for rotation observability
    marker = CylinderGeometry(0.0022, 0.004).translate(LID_R - 0.004, 0.0, LID_HEIGHT)
    lid.visual(mesh_from_geometry(marker, "lid_marker"), material=brass_dark, name="lid_marker")
    lid.inertial = Inertial.from_geometry(
        Cylinder(LID_R, LID_HEIGHT + DIPPER_COLLAR_H),
        mass=0.045,
        origin=Origin(xyz=(0.0, 0.0, (LID_HEIGHT + DIPPER_COLLAR_H) / 2.0)),
    )

    # ---- honey dipper: wooden stick seated through the lid hole ----
    # The dipper part origin is at the lid top center; the handle extends up
    # and the ball hangs down into the jar.
    dipper = model.part("dipper")
    dipper.visual(
        _dipper_mesh(),
        material=wood,
        # Dipper origin at the lid top, ball hangs down inside the jar.
        origin=Origin(xyz=(0.0, 0.0, LID_HEIGHT + DIPPER_COLLAR_H)),
        name="dipper_wood",
    )
    dipper.inertial = Inertial.from_geometry(
        Cylinder(DIPPER_BALL_R, DIPPER_HANDLE_LEN + 2 * DIPPER_BALL_R),
        mass=0.025,
        origin=Origin(xyz=(0.0, 0.0, DIPPER_HANDLE_LEN / 2.0)),
    )

    # ---- articulations ----
    # lid_rotate: continuous screw spin (body -> carrier)
    model.articulation(
        "lid_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, LID_MOUNT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )
    # lid_slide: prismatic lift (carrier -> lid)
    model.articulation(
        "lid_slide",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=LID_HEIGHT + 0.01, effort=1.0, velocity=1.0),
    )
    # dipper is fixed to the lid (it sits in the holder hole)
    model.articulation(
        "dipper_to_lid",
        ArticulationType.FIXED,
        parent=lid,
        child=dipper,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("jar_body")
    lid = object_model.get_part("lid")
    dipper = object_model.get_part("dipper")
    rotate = object_model.get_articulation("lid_rotate")
    slide = object_model.get_articulation("lid_slide")

    # --- intentional overlaps ---
    # Lid skirt seats over the neck (capture fit)
    ctx.allow_overlap(
        lid,
        body,
        elem_a="lid_brass",
        elem_b="jar_glass",
        reason="The brass lid skirt is intentionally screwed down over the round neck and mouth rim.",
    )
    # Dipper handle passes through the lid hole (seated in holder)
    ctx.allow_overlap(
        dipper,
        lid,
        elem_a="dipper_wood",
        elem_b="lid_brass",
        reason="The dipper handle is intentionally seated through the lid dipper-holder hole.",
    )
    # Dipper ball hangs inside the jar body
    ctx.allow_overlap(
        dipper,
        body,
        elem_a="dipper_wood",
        elem_b="jar_glass",
        reason="The dipper ball hangs inside the jar cavity through the neck opening.",
    )

    # --- jar body is a square section ---
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "jar body is square in cross-section",
        abs(bext[0] - bext[1]) < 0.008,
        details=f"x={bext[0]:.4f}, y={bext[1]:.4f}",
    )
    ctx.check(
        "jar is taller than wide",
        bext[2] > bext[0] + 0.03 and bext[2] > bext[1] + 0.03,
        details=f"extents={bext}",
    )

    # --- glass wall thickness at the mouth ---
    # The mouth rim extends beyond the neck outer radius
    ctx.check(
        "mouth rim extends beyond neck (visible glass wall thickness)",
        MOUTH_RIM_OUTER > NECK_R + 0.001,
        details=f"mouth_rim_outer={MOUTH_RIM_OUTER:.4f}, neck_r={NECK_R:.4f}",
    )

    # --- brass lid is round and seated on the neck ---
    lext = _ext(ctx.part_world_aabb(lid))
    ctx.check(
        "lid is round (square footprint bounding a disc)",
        abs(lext[0] - lext[1]) < 0.004 and lext[0] < bext[0],
        details=f"lid x={lext[0]:.4f}, y={lext[1]:.4f}, body x={bext[0]:.4f}",
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

    # --- dipper holder exists on the lid (dipper seated through the hole) ---
    # The dipper visual extends from the lid top upward; check its AABB is above the neck.
    daabb = ctx.part_world_aabb(dipper)
    daabb_top = daabb[1][2]
    daabb_bot = daabb[0][2]
    ctx.check(
        "dipper extends above the lid top (handle sticks up)",
        daabb_top > NECK_TOP + LID_HEIGHT,
        details=f"dipper top z={daabb_top:.4f}, expected above {NECK_TOP + LID_HEIGHT:.4f}",
    )
    ctx.check(
        "dipper ball hangs below the lid into the jar",
        daabb_bot < NECK_TOP,
        details=f"dipper bottom z={daabb_bot:.4f}, neck_top={NECK_TOP:.4f}",
    )
    # Dipper overlaps the lid in XY (seated through the holder hole)
    ctx.expect_overlap(
        dipper, lid, axes="xy", min_overlap=0.003,
        name="dipper seated through lid dipper-holder hole",
    )

    # --- lid_rotate spins the lid (continuous about +Z) ---
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

    # --- joint type and axis checks ---
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
