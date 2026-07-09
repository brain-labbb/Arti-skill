from __future__ import annotations

# Squat square-shouldered pump bottle variant.
# Frame: bottle axis along +Z, base at z=0, pump at top.
# Body is a translucent thin-wall PET shell: square barrel with molded volume
# bands, a square-to-round shoulder loft, short neck, and tether collar.
# The pump head rides on two joints through a massless carrier:
#   - pump_rotate: REVOLUTE slight rotation about +Z at the neck top
#   - pump_slide:  PRISMATIC press-down along -Z (positive q = press)
# A small tether loop (arm + ring) is connected to the neck collar.

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
BODY_W = 0.065           # square body width
BODY_D = 0.065           # square body depth
BODY_H = 0.075           # body height (squat)
CORNER_R = 0.005         # vertical-edge fillet radius
WALL = 0.002             # wall thickness

BAND_H = 0.003           # volume band height
BAND_PROUD = 0.002       # how far bands protrude from the surface
BAND_ZS = (0.018, 0.040, 0.060)  # three volume bands

SHOULDER_H = 0.015       # shoulder taper height
SHOULDER_TOP_Z = BODY_H  # body top = shoulder base
NECK_R = 0.012           # neck outer radius
NECK_H = 0.018           # neck height
NECK_TOP_Z = BODY_H + SHOULDER_H + NECK_H  # 0.108

COLLAR_R = NECK_R + 0.003  # collar outer radius
COLLAR_H = 0.004           # collar height

PUMP_DISC_R = 0.018      # pump disc radius
PUMP_DISC_H = 0.006      # pump disc height
PUMP_BTN_H = 0.004       # button height on top
PUMP_NOZZLE_R = 0.004    # nozzle tube radius
PUMP_NOZZLE_L = 0.022    # nozzle length
PUMP_STEM_R = 0.005      # stem radius
PUMP_STEM_H = 0.012      # stem depth into neck

PUMP_SLIDE_MAX = 0.012   # prismatic travel (press distance)
PUMP_ROTATE_MAX = 0.40   # revolute limit (radians, ~23 deg)

ARM_LEN = 0.010          # tether arm length
TETHER_R = 0.006         # tether ring major radius
TETHER_T = 0.0015        # tether ring tube (wall) half-thickness


def _bottle_shell():
    """Squat square bottle: body + bands + shoulder + neck + collar, hollowed."""
    # Main body box with rounded vertical edges
    body = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H, centered=(True, True, False))
    )
    body = body.edges("|Z").fillet(CORNER_R)

    # Volume bands: thin raised ridges around the body
    for zb in BAND_ZS:
        band = (
            cq.Workplane("XY")
            .transformed(offset=(0, 0, zb))
            .box(
                BODY_W + 2 * BAND_PROUD,
                BODY_D + 2 * BAND_PROUD,
                BAND_H,
                centered=(True, True, False),
            )
        )
        band = band.edges("|Z").fillet(CORNER_R + BAND_PROUD)
        body = body.union(band)

    # Shoulder: loft from body-top rect to neck-base circle
    neck_outer = NECK_R + WALL + 0.001
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_H)
        .rect(BODY_W, BODY_D)
        .workplane(offset=SHOULDER_H)
        .circle(neck_outer)
        .loft()
    )
    body = body.union(shoulder)

    # Neck cylinder
    neck = (
        cq.Workplane("XY")
        .workplane(offset=BODY_H + SHOULDER_H)
        .circle(NECK_R)
        .extrude(NECK_H)
    )
    body = body.union(neck)

    # Collar ring at top of neck
    collar = (
        cq.Workplane("XY")
        .workplane(offset=NECK_TOP_Z - COLLAR_H)
        .circle(COLLAR_R)
        .extrude(COLLAR_H)
    )
    body = body.union(collar)

    # Hollow the interior: body cavity (inset box)
    cavity_w = BODY_W - 2 * WALL
    cavity_d = BODY_D - 2 * WALL
    cavity_h = BODY_H - WALL
    cavity = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, WALL))
        .box(cavity_w, cavity_d, cavity_h, centered=(True, True, False))
    )
    cavity = cavity.edges("|Z").fillet(max(0.001, CORNER_R - WALL))
    body = body.cut(cavity)

    # Neck + shoulder passage (cylindrical cut through shoulder and neck)
    neck_cavity = (
        cq.Workplane("XY")
        .workplane(offset=BODY_H - 0.001)
        .circle(NECK_R - WALL)
        .extrude(SHOULDER_H + NECK_H + COLLAR_H + 0.002)
    )
    body = body.cut(neck_cavity)

    return body


def _pump_head_solid():
    """Pump head: disc + button + stem (nozzle added as separate visual)."""
    # Disc
    disc = cq.Workplane("XY").circle(PUMP_DISC_R).extrude(PUMP_DISC_H)

    # Raised button on top
    button = (
        cq.Workplane("XY")
        .workplane(offset=PUMP_DISC_H)
        .circle(PUMP_DISC_R * 0.55)
        .extrude(PUMP_BTN_H)
    )
    disc = disc.union(button)

    # Stem extending downward into the neck
    stem = (
        cq.Workplane("XY")
        .workplane(offset=-PUMP_STEM_H)
        .circle(PUMP_STEM_R)
        .extrude(PUMP_STEM_H + 0.001)
    )
    disc = disc.union(stem)

    return disc


def _tether_solid():
    """Tether loop: arm extending from collar + annular ring."""
    arm_cz = NECK_TOP_Z - COLLAR_H / 2.0
    # Arm starts slightly inside the collar for connection, extends outward
    arm = (
        cq.Workplane("XY")
        .transformed(offset=(COLLAR_R - 0.002, 0, arm_cz - 0.0015))
        .box(ARM_LEN + 0.002, 0.004, 0.003, centered=(False, True, False))
    )

    # Flat ring (washer) at the end of the arm
    ring_cx = COLLAR_R + ARM_LEN
    ring_cz = arm_cz - TETHER_T
    ring = (
        cq.Workplane("XY")
        .transformed(offset=(ring_cx, 0, ring_cz))
        .circle(TETHER_R + TETHER_T)
        .circle(TETHER_R - TETHER_T)
        .extrude(TETHER_T * 2)
    )
    return arm.union(ring)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pump_bottle")

    # Materials
    clear = model.material("clear_body", rgba=(0.82, 0.88, 0.86, 0.35))
    white = model.material("pump_white", rgba=(0.92, 0.92, 0.94, 1.0))
    gray = model.material("tether_gray", rgba=(0.35, 0.35, 0.38, 1.0))

    # ---- bottle body (root) ----
    body = model.part("bottle")
    shell = _bottle_shell()
    body.visual(
        mesh_from_cadquery(shell, "bottle_shell"),
        material=clear,
        name="bottle_shell",
    )

    # Tether loop on the neck collar
    tether = _tether_solid()
    body.visual(
        mesh_from_cadquery(tether, "tether_loop"),
        material=gray,
        name="tether_loop",
    )

    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, NECK_TOP_Z)),
        mass=0.050,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    # ---- pump carrier (massless link for decoupled joints) ----
    carrier = model.part("pump_carrier")
    carrier.inertial = Inertial.from_geometry(
        Box((0.008, 0.008, 0.008)), mass=1e-4,
    )

    # ---- pump head ----
    pump = model.part("pump_head")
    pump_geo = _pump_head_solid()
    pump.visual(
        mesh_from_cadquery(pump_geo, "pump_body"),
        material=white,
        name="pump_body",
    )
    # Nozzle: horizontal cylinder extending along +Y from the button
    pump.visual(
        Cylinder(PUMP_NOZZLE_R, PUMP_NOZZLE_L),
        origin=Origin(
            xyz=(0.0, PUMP_NOZZLE_L / 2.0, PUMP_DISC_H + 0.002),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=white,
        name="pump_nozzle",
    )
    pump.inertial = Inertial.from_geometry(
        Cylinder(PUMP_DISC_R, PUMP_DISC_H + PUMP_BTN_H + PUMP_STEM_H),
        mass=0.015,
        origin=Origin(xyz=(0.0, 0.0, (PUMP_DISC_H + PUMP_BTN_H - PUMP_STEM_H) / 2.0)),
    )

    # ---- articulations ----
    # pump_rotate: REVOLUTE slight rotation at neck top
    model.articulation(
        "pump_rotate",
        ArticulationType.REVOLUTE,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.0,
            lower=-PUMP_ROTATE_MAX,
            upper=PUMP_ROTATE_MAX,
        ),
    )

    # pump_slide: PRISMATIC press-down (positive q = downward along -Z)
    model.articulation(
        "pump_slide",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=pump,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=0.5,
            lower=0.0,
            upper=PUMP_SLIDE_MAX,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle")
    pump = object_model.get_part("pump_head")
    rotate = object_model.get_articulation("pump_rotate")
    slide = object_model.get_articulation("pump_slide")

    bottle_shell = body.get_visual("bottle_shell")
    tether_loop = body.get_visual("tether_loop")
    pump_body = pump.get_visual("pump_body")
    pump_nozzle = pump.get_visual("pump_nozzle")

    # --- bottle body is translucent ---
    ctx.check(
        "bottle body is translucent (alpha < 1)",
        bottle_shell.material.rgba is not None and bottle_shell.material.rgba[3] < 1.0,
        details=f"rgba={bottle_shell.material.rgba}",
    )

    # --- pump head is opaque white ---
    ctx.check(
        "pump head is opaque",
        pump_body.material.rgba is not None and pump_body.material.rgba[3] >= 0.99,
        details=f"rgba={pump_body.material.rgba}",
    )

    # --- pump_slide is PRISMATIC (non-fixed joint) ---
    ctx.check(
        "pump_slide is a prismatic joint",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={slide.articulation_type}",
    )

    # --- pump_rotate is REVOLUTE (non-fixed joint) ---
    ctx.check(
        "pump_rotate is a revolute joint",
        rotate.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={rotate.articulation_type}",
    )

    # --- pump_slide positive q lowers the pump head ---
    rest_z = ctx.part_world_position(pump)[2]
    with ctx.pose({slide: PUMP_SLIDE_MAX}):
        pressed_z = ctx.part_world_position(pump)[2]
    ctx.check(
        "pump_slide presses the pump head downward",
        pressed_z is not None and rest_z is not None
        and pressed_z < rest_z - PUMP_SLIDE_MAX * 0.8,
        details=f"rest_z={rest_z}, pressed_z={pressed_z}",
    )

    # --- pump_rotate changes nozzle orientation ---
    with ctx.pose({rotate: 0.0}):
        aabb0 = ctx.part_world_aabb(pump)
    with ctx.pose({rotate: PUMP_ROTATE_MAX}):
        aabb1 = ctx.part_world_aabb(pump)
    # Nozzle extends along +Y; rotation should shift the AABB in X
    ctx.check(
        "pump_rotate changes the pump head bounding box",
        aabb0 is not None and aabb1 is not None
        and abs(aabb0[1][0] - aabb1[1][0]) > 0.0005,
        details=f"rest_max_x={aabb0[1][0] if aabb0 else None}, "
                f"rot_max_x={aabb1[1][0] if aabb1 else None}",
    )

    # --- pump head is above the body ---
    pump_pos = ctx.part_world_position(pump)
    ctx.check(
        "pump head is mounted above the neck",
        pump_pos is not None and pump_pos[2] >= BODY_H + SHOULDER_H,
        details=f"pump_z={pump_pos[2] if pump_pos else None}",
    )

    # --- tether loop exists on the bottle ---
    ctx.check(
        "tether loop visual exists on the bottle",
        tether_loop is not None,
    )

    # --- nozzle visual exists ---
    ctx.check(
        "pump nozzle visual exists",
        pump_nozzle is not None,
    )

    # --- bottle has multiple visuals (shell + tether = at least 2) ---
    ctx.check(
        "bottle has shell and tether visuals",
        len(body.visuals) >= 2,
        details=f"visuals={[v.name for v in body.visuals]}",
    )

    # --- pump_rotate has bounded limits (not continuous) ---
    limits = rotate.motion_limits
    ctx.check(
        "pump_rotate has bounded limits (slight rotation)",
        limits is not None and limits.lower is not None and limits.upper is not None
        and abs(limits.lower) < 1.0 and abs(limits.upper) < 1.0,
        details=f"lower={limits.lower if limits else None}, "
                f"upper={limits.upper if limits else None}",
    )

    # --- pump stem inside neck: intentional small overlap ---
    ctx.allow_overlap(
        pump,
        body,
        elem_a="pump_body",
        elem_b="bottle_shell",
        reason="Pump stem is intentionally inserted into the hollow neck cavity.",
    )

    # Verify the stem is within the neck footprint on XY
    ctx.expect_within(
        pump,
        body,
        axes="xy",
        inner_elem="pump_body",
        outer_elem="bottle_shell",
        margin=BODY_W / 2.0,
        name="pump stem stays within the bottle footprint",
    )

    return ctx.report()


object_model = build_object_model()
