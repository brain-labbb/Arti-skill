from __future__ import annotations

# Small amber-glass serum bottle with a lotion-style pump dispenser.
# Fork of the dropper-cap bottle: same round glass body, hollow open-mouth shell,
# neck, and white label band. Closure changed from rubber-bulb dropper to a
# press-down pump dispenser with a dip tube.
#
# Frame: bottle axis along +Z, base at z=0, pump at +Z.
# Construction:
#   - body (root): amber-glass hollow shell (body + shoulder + neck),
#     white label band, stationary pump collar over the neck, stationary dip tube.
#   - pump_head (child): pressable actuator disk with nozzle spout and stem.
# Articulation:
#   - body_to_pump: PRISMATIC, axis=(0,0,-1), q=0 rest (head up on collar),
#     q=PUMP_TRAVEL pressed (head translated straight down).

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- bottle body dimensions (identical to parent) ----
BODY_R = 0.015          # body outer radius (~30 mm diameter)
BODY_Z0 = 0.0           # base
BODY_TOP = 0.060        # top of straight cylinder wall
SHOULDER_TOP = 0.070    # top of rounded shoulder
NECK_R = 0.0080         # neck outer radius
NECK_TOP = 0.0820       # top of neck (bottle mouth)
WALL = 0.0018           # glass wall thickness
BORE_R = NECK_R - WALL  # inner bore radius at neck

# ---- label band (identical to parent) ----
LABEL_Z0 = 0.012
LABEL_Z1 = 0.044
LABEL_R = BODY_R + 0.0006

# ---- pump collar (stationary, mounted on body) ----
COLLAR_R = 0.0105       # outer radius of collar ring
COLLAR_Z0 = 0.066       # collar bottom (overlaps shoulder/neck)
COLLAR_TOP = 0.090      # collar top surface (pump head rests here)

# ---- dip tube (stationary, on body) ----
DIP_R = 0.0020          # dip tube outer radius
DIP_BOTTOM = 0.008      # tube tip near bottle floor
DIP_FLANGE_R = NECK_R + 0.001  # connector flange outer (contacts collar bore)
DIP_FLANGE_H = 0.004    # flange height

# ---- pump head assembly (moving part) ----
HEAD_R = 0.012          # actuator head radius
HEAD_H = 0.006          # actuator head thickness
NOZZLE_R = 0.0025       # nozzle spout radius
NOZZLE_LEN = 0.013      # nozzle extends to the side
STEM_R = 0.003          # actuator stem radius
STEM_LEN = 0.022        # stem length (extends down from head into collar)

PUMP_TRAVEL = 0.008     # press-down travel (8 mm)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _body_glass_mesh():
    """Hollow amber-glass shell: body cylinder, lofted shoulder, neck, with a
    single tapered interior bore so the bottle reads as a real open container."""
    outer = (
        cq.Workplane("XY")
        .circle(BODY_R)
        .extrude(BODY_TOP)
    )
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP)
        .circle(BODY_R)
        .workplane(offset=(SHOULDER_TOP - BODY_TOP))
        .circle(NECK_R + 0.0015)
        .loft(ruled=False)
    )
    neck = (
        cq.Workplane("XY")
        .workplane(offset=SHOULDER_TOP)
        .circle(NECK_R)
        .extrude(NECK_TOP - SHOULDER_TOP)
    )
    solid = outer.union(shoulder).union(neck)
    # Interior cavity: tapered bore staying one wall-thickness inside the outer
    # profile at every height, open at the mouth.
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=WALL)
        .circle(BODY_R - WALL)
        .workplane(offset=(BODY_TOP - WALL))
        .circle(BODY_R - WALL)
        .workplane(offset=(SHOULDER_TOP - BODY_TOP))
        .circle(BORE_R)
        .workplane(offset=(NECK_TOP - SHOULDER_TOP + 0.001))
        .circle(BORE_R)
        .loft(ruled=False)
    )
    solid = solid.cut(cavity)
    return mesh_from_cadquery(solid, "body_glass")


def _collar_mesh():
    """White pump collar: ring that grips the bottle neck, with a solid top cap
    and a small bore for the pump stem to pass through."""
    # Outer ring
    outer = (
        cq.Workplane("XY")
        .workplane(offset=COLLAR_Z0)
        .circle(COLLAR_R)
        .extrude(COLLAR_TOP - COLLAR_Z0)
    )
    # Main bore: hollow out interior to slip over the neck (open at bottom,
    # stops 2 mm short of the top so a solid cap remains).
    bore = (
        cq.Workplane("XY")
        .workplane(offset=COLLAR_Z0 - 0.001)
        .circle(NECK_R + 0.0006)
        .extrude((COLLAR_TOP - 0.002) - (COLLAR_Z0 - 0.001))
    )
    collar = outer.cut(bore)
    # Stem bore: cut a small hole through the top cap for the actuator stem.
    stem_bore = (
        cq.Workplane("XY")
        .workplane(offset=COLLAR_TOP - 0.004)
        .circle(STEM_R + 0.0008)
        .extrude(0.006)
    )
    collar = collar.cut(stem_bore)
    return mesh_from_cadquery(collar, "pump_collar")


def _dip_tube_mesh():
    """Dip tube: thin tube from near the bottle floor up to the collar, with a
    small connector flange at the top that contacts the collar bore wall for
    geometric connectivity."""
    tube = (
        cq.Workplane("XY")
        .workplane(offset=DIP_BOTTOM)
        .circle(DIP_R)
        .extrude(COLLAR_Z0 - DIP_BOTTOM)
    )
    # Connector flange: a short ring at the top that widens to contact the
    # collar bore wall (flange outer > collar main bore inner).
    flange = (
        cq.Workplane("XY")
        .workplane(offset=COLLAR_Z0 - DIP_FLANGE_H)
        .circle(DIP_FLANGE_R)
        .circle(DIP_R - 0.0002)
        .extrude(DIP_FLANGE_H)
    )
    tube = tube.union(flange)
    return mesh_from_cadquery(tube, "dip_tube")


def _pump_actuator_mesh():
    """Pump actuator: flat disk head with a horizontal nozzle spout on one side
    and a stem extending down through the collar bore. All in part-local coords
    with the origin at the head bottom (contact surface with the collar top)."""
    # Head disk: from z=0 to z=HEAD_H
    head = (
        cq.Workplane("XY")
        .circle(HEAD_R)
        .extrude(HEAD_H)
    )
    # Nozzle spout: horizontal cylinder extending in +X from the head side.
    # Built on the YZ workplane offset to the head edge.
    nozzle_z = HEAD_H * 0.55
    nozzle = (
        cq.Workplane("YZ")
        .workplane(offset=HEAD_R - 0.001)
        .center(0, nozzle_z)
        .circle(NOZZLE_R)
        .extrude(NOZZLE_LEN)
    )
    head = head.union(nozzle)
    # Stem: cylinder extending down from the head center.
    # Extends 2 mm into the head volume for mesh connectivity.
    stem = (
        cq.Workplane("XY")
        .workplane(offset=-STEM_LEN)
        .circle(STEM_R)
        .extrude(STEM_LEN + 0.002)
    )
    head = head.union(stem)
    # Small rounded cap on the nozzle tip
    tip = (
        cq.Workplane("XY")
        .workplane(offset=0)
        .transformed(offset=(HEAD_R - 0.001 + NOZZLE_LEN, 0, nozzle_z))
        .sphere(NOZZLE_R)
    )
    head = head.union(tip)
    return mesh_from_cadquery(head, "pump_actuator")


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="serum_bottle_pump")

    amber = model.material("amber_glass", rgba=(0.45, 0.24, 0.06, 0.5))
    white = model.material("white_plastic", rgba=(0.95, 0.95, 0.94, 1.0))
    label_white = model.material("label_white", rgba=(0.97, 0.97, 0.96, 1.0))
    clear = model.material("clear_tube", rgba=(0.82, 0.88, 0.90, 0.35))
    pump_white = model.material("pump_white", rgba=(0.92, 0.92, 0.91, 1.0))

    # ---- body (root) ----
    body = model.part("body")

    # Amber glass hollow shell (identical to parent)
    body.visual(_body_glass_mesh(), material=amber, name="body_glass")

    # White label band wrapping the body middle (identical to parent)
    label = (
        cq.Workplane("XY")
        .workplane(offset=LABEL_Z0)
        .circle(LABEL_R)
        .circle(BODY_R - 0.0002)
        .extrude(LABEL_Z1 - LABEL_Z0)
    )
    body.visual(
        mesh_from_cadquery(label, "label_band"),
        material=label_white,
        name="label_band",
    )

    # Pump collar: stationary ring over the neck with a top cap and stem bore
    body.visual(_collar_mesh(), material=white, name="pump_collar")

    # Dip tube: thin translucent tube from near the bottle floor up to the collar,
    # with a connector flange at the top for geometric connectivity.
    body.visual(_dip_tube_mesh(), material=clear, name="dip_tube")

    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, BODY_TOP),
        mass=0.040,
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP / 2.0)),
    )

    # ---- pump head (moving child) ----
    pump = model.part("pump_head")

    # Actuator: head disk + nozzle + stem as one CadQuery mesh.
    # Part-local origin sits at the head bottom (z=0 in local = collar top in world at rest).
    pump.visual(
        _pump_actuator_mesh(),
        material=pump_white,
        name="pump_actuator",
    )

    pump.inertial = Inertial.from_geometry(
        Cylinder(HEAD_R, HEAD_H + STEM_LEN),
        mass=0.006,
        origin=Origin(xyz=(0.0, 0.0, (HEAD_H - STEM_LEN) / 2.0)),
    )

    # Prismatic articulation: positive q presses the head straight DOWN.
    # Origin at the collar top surface (actual contact surface).
    # axis=(0,0,-1) so positive q translates downward.
    model.articulation(
        "body_to_pump",
        ArticulationType.PRISMATIC,
        parent=body,
        child=pump,
        origin=Origin(xyz=(0.0, 0.0, COLLAR_TOP)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=PUMP_TRAVEL,
            effort=8.0,
            velocity=0.15,
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    pump = object_model.get_part("pump_head")
    press = object_model.get_articulation("body_to_pump")

    # --- body is short, narrow, amber glass (identical to parent) ---
    body_aabb = ctx.part_world_aabb(body)
    bmn, bmx = body_aabb
    body_h = bmx[2] - bmn[2]
    body_dia = max(bmx[0] - bmn[0], bmx[1] - bmn[1])
    ctx.check(
        "bottle body is short — height under ~0.10 m",
        body_h < 0.100,
        details=f"body height={body_h:.4f}",
    )
    ctx.check(
        "bottle body is narrow (~0.03 m dia)",
        0.020 < body_dia < 0.040,
        details=f"body dia={body_dia:.4f}",
    )
    rgba = getattr(body.get_visual("body_glass").material, "rgba", None)
    ctx.check(
        "body glass is amber-tinted and translucent",
        rgba is not None and rgba[0] > rgba[2] and rgba[3] < 0.95,
        details=f"amber_glass rgba={rgba}",
    )

    # --- label band on the body wall ---
    lmn, lmx = ctx.part_element_world_aabb(body, elem="label_band")
    ctx.check(
        "label band is on the body wall, around the middle",
        lmn[2] > 0.005 and lmx[2] < SHOULDER_TOP,
        details=f"label z=[{lmn[2]:.4f},{lmx[2]:.4f}]",
    )

    # --- pump collar sits on the neck region (stationary on body) ---
    cmn, cmx = ctx.part_element_world_aabb(body, elem="pump_collar")
    ctx.check(
        "pump collar sits on the neck region",
        cmn[2] > SHOULDER_TOP - 0.005 and cmx[2] >= NECK_TOP,
        details=f"collar z=[{cmn[2]:.4f},{cmx[2]:.4f}]",
    )

    # --- dip tube extends deep into the bottle and connects to collar region ---
    dmn, dmx = ctx.part_element_world_aabb(body, elem="dip_tube")
    ctx.check(
        "dip tube reaches deep into the bottle",
        dmn[2] < BODY_TOP * 0.5,
        details=f"dip_tube bottom z={dmn[2]:.4f}",
    )
    ctx.check(
        "dip tube top connects to the collar region",
        dmx[2] >= COLLAR_Z0 - 0.001,
        details=f"dip_tube top z={dmx[2]:.4f}, collar bottom={COLLAR_Z0}",
    )

    # --- pump head at rest: the head disk top is at COLLAR_TOP + HEAD_H ---
    rest_pos = ctx.part_world_position(pump)
    hmn_r, hmx_r = ctx.part_element_world_aabb(pump, elem="pump_actuator")
    ctx.check(
        "pump head disk at rest sits on the collar top",
        abs(hmx_r[2] - (COLLAR_TOP + HEAD_H)) < 0.002,
        details=f"head top z(rest)={hmx_r[2]:.4f}, expected={COLLAR_TOP + HEAD_H:.4f}",
    )

    # --- nozzle spout extends beyond the head radius ---
    head_half_x = (hmx_r[0] - hmn_r[0]) / 2.0
    ctx.check(
        "pump head has a nozzle spout extending to one side",
        head_half_x > HEAD_R + 0.003,
        details=f"head half-width x={head_half_x:.4f}, HEAD_R={HEAD_R}",
    )

    # --- pump presses straight DOWN when actuated ---
    with ctx.pose({press: PUMP_TRAVEL}):
        pressed_pos = ctx.part_world_position(pump)

    ctx.check(
        "pump head translates straight down when pressed (no lateral shift)",
        pressed_pos[2] < rest_pos[2] - 0.003
        and abs(pressed_pos[0] - rest_pos[0]) < 1e-4
        and abs(pressed_pos[1] - rest_pos[1]) < 1e-4,
        details=f"rest={rest_pos}, pressed={pressed_pos}",
    )
    ctx.check(
        "pump head moves down by approximately the travel distance",
        abs((rest_pos[2] - pressed_pos[2]) - PUMP_TRAVEL) < 0.001,
        details=f"delta_z={rest_pos[2] - pressed_pos[2]:.4f}, travel={PUMP_TRAVEL}",
    )

    # --- dip tube is stationary (does not move with pump head) ---
    dip_rest = ctx.part_element_world_aabb(body, elem="dip_tube")
    with ctx.pose({press: PUMP_TRAVEL}):
        dip_pressed = ctx.part_element_world_aabb(body, elem="dip_tube")
    ctx.check(
        "dip tube stays stationary when pump is pressed",
        abs(dip_rest[0][2] - dip_pressed[0][2]) < 1e-5
        and abs(dip_rest[1][2] - dip_pressed[1][2]) < 1e-5,
        details=f"rest_z=[{dip_rest[0][2]:.4f},{dip_rest[1][2]:.4f}], "
                f"pressed_z=[{dip_pressed[0][2]:.4f},{dip_pressed[1][2]:.4f}]",
    )

    # --- allow intentional sliding-fit overlap ---
    # The pump actuator stem passes through the collar bore (captured shaft).
    ctx.allow_overlap(
        body, pump,
        elem_a="pump_collar",
        elem_b="pump_actuator",
        reason="The pump stem is intentionally captured inside the collar bore as a sliding fit.",
    )
    # Proof: pump axis stays aligned with the body axis on both lateral axes.
    ctx.expect_origin_distance(
        pump, body,
        axes="xy",
        max_dist=0.001,
        name="pump stem axis stays aligned with body axis (XY)",
    )
    # Proof: the stem remains inserted in the collar at rest and at full press.
    ctx.expect_overlap(
        pump, body,
        axes="z",
        elem_a="pump_actuator",
        elem_b="pump_collar",
        min_overlap=0.005,
        name="stem remains inserted in the collar at rest",
    )
    with ctx.pose({press: PUMP_TRAVEL}):
        ctx.expect_overlap(
            pump, body,
            axes="z",
            elem_a="pump_actuator",
            elem_b="pump_collar",
            min_overlap=0.003,
            name="stem remains inserted in the collar at full press",
        )

    return ctx.report()


object_model = build_object_model()
