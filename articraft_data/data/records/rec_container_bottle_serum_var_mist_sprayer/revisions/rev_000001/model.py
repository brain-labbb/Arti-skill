from __future__ import annotations

# Small amber-glass serum bottle with a fine-mist spray sprayer.
# Frame: bottle axis along +Z, bottle base at z=0, sprayer rising at +Z.
# Construction:
#   - body (root): short amber-glass cylinder body + shoulder + neck (hollow
#     shell), wrapped by a white paper label band around the middle. A white
#     crimped collar is fixed over the neck (with radial crimp ridges), and a
#     thin clear dip tube hangs from inside the collar down into the bottle.
#   - actuator (child): flat white spray button on top of the collar, with a
#     front-facing nozzle and a short pump stem that goes down into the collar.
# Articulation:
#   - body_to_actuator: PRISMATIC, presses straight DOWN into the collar
#     (axis 0,0,-1), travel ~6 mm.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- bottle body dimensions (identical to parent) ----
BODY_R = 0.015          # body outer radius (~0.03 m diameter)
BODY_Z0 = 0.0           # base
BODY_TOP = 0.060        # top of the straight cylinder wall
SHOULDER_TOP = 0.070    # top of the rounded shoulder
NECK_R = 0.0080         # neck outer radius
NECK_TOP = 0.0820       # top of the neck (bottle mouth)
WALL = 0.0018           # glass wall thickness
BORE_R = NECK_R - WALL  # inner bore radius at the neck

# ---- label band (identical to parent) ----
LABEL_Z0 = 0.012
LABEL_Z1 = 0.044
LABEL_R = BODY_R + 0.0006

# ---- crimped collar (fixed to body) ----
COLLAR_R = 0.0105       # collar outer radius (grips over the neck)
COLLAR_Z0 = 0.068       # collar bottom (overlaps shoulder/neck junction)
COLLAR_TOP = 0.090      # collar top surface
COLLAR_BORE_R = 0.005   # internal bore for pump stem

# ---- crimp ridges (radial ribs on the collar) ----
N_CRIMP_RIDGES = 12
CRIMP_RIDGE_R = 0.0012  # ridge half-width
CRIMP_RIDGE_H = 0.006   # ridge height
CRIMP_RIDGE_Z0 = COLLAR_Z0 + 0.002  # start slightly above collar bottom

# ---- dip tube (fixed, hangs into bottle from collar bore) ----
DIP_TUBE_R = 0.0018     # dip tube outer radius
DIP_TUBE_BOTTOM = 0.010 # dip tube tip (near bottle bottom)
DIP_TUBE_TOP = NECK_TOP # connects at neck top inside collar

# ---- actuator (moving) dimensions ----
BUTTON_R = 0.0095       # flat spray button radius
BUTTON_H = 0.005        # button height/thickness
# Button sits on collar top: in world coords bottom=COLLAR_TOP, top=COLLAR_TOP+BUTTON_H
# In child frame (origin at COLLAR_TOP): bottom=0, top=BUTTON_H

# nozzle on the front (+Y face) of the button
NOZZLE_R = 0.0014       # nozzle tube radius
NOZZLE_LEN = 0.009      # nozzle protrusion from button edge
NOZZLE_LOCAL_Z = BUTTON_H * 0.55  # slightly above button center in child frame

# pump stem (goes from button bottom down into collar bore)
STEM_R = 0.003          # pump stem radius
STEM_LOCAL_TOP = 0.0    # at button bottom = child frame origin
STEM_WORLD_BOTTOM = COLLAR_Z0 + 0.004  # penetrates into collar
STEM_LOCAL_BOTTOM = STEM_WORLD_BOTTOM - COLLAR_TOP  # negative in child frame
STEM_LEN = STEM_LOCAL_TOP - STEM_LOCAL_BOTTOM

# prismatic press travel
PRESS_TRAVEL = 0.006    # 6mm button press


def _body_glass_mesh():
    """Hollow amber-glass shell: body, shoulder, neck with internal bore."""
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
    """White crimped collar: ring with bore that fits over the neck."""
    outer = (
        cq.Workplane("XY")
        .workplane(offset=COLLAR_Z0)
        .circle(COLLAR_R)
        .extrude(COLLAR_TOP - COLLAR_Z0)
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=COLLAR_Z0 - 0.001)
        .circle(COLLAR_BORE_R)
        .extrude((COLLAR_TOP + 0.001) - (COLLAR_Z0 - 0.001))
    )
    collar = outer.cut(inner)
    return mesh_from_cadquery(collar, "crimped_collar")


def _crimp_ridge_mesh(i: int):
    """One radial crimp ridge on the collar surface."""
    angle = 2.0 * math.pi * i / N_CRIMP_RIDGES
    cx = (COLLAR_R + CRIMP_RIDGE_R * 0.3) * math.cos(angle)
    cy = (COLLAR_R + CRIMP_RIDGE_R * 0.3) * math.sin(angle)
    ridge = (
        cq.Workplane("XY")
        .workplane(offset=CRIMP_RIDGE_Z0)
        .center(cx, cy)
        .rect(CRIMP_RIDGE_R * 2, CRIMP_RIDGE_R * 1.2)
        .extrude(CRIMP_RIDGE_H)
    )
    return mesh_from_cadquery(ridge, f"crimp_ridge_{i}")


def _dip_tube_mesh():
    """Dip tube with a mounting flange at top that seats in the collar bore."""
    # Main tube from near bottle bottom up to collar area (world coords)
    tube = (
        cq.Workplane("XY")
        .workplane(offset=DIP_TUBE_BOTTOM)
        .circle(DIP_TUBE_R)
        .extrude(DIP_TUBE_TOP - DIP_TUBE_BOTTOM)
    )
    # Mounting flange: a small ring at the top that contacts the collar bore wall
    flange = (
        cq.Workplane("XY")
        .workplane(offset=COLLAR_Z0 + 0.002)
        .circle(COLLAR_BORE_R - 0.0002)  # fits snugly in collar bore
        .circle(DIP_TUBE_R)
        .extrude(0.004)
    )
    return mesh_from_cadquery(tube.union(flange), "dip_tube")


def _button_mesh():
    """Flat spray actuator button in child-frame coords (z=0 to BUTTON_H)."""
    disc = (
        cq.Workplane("XY")
        .circle(BUTTON_R)
        .extrude(BUTTON_H)
    )
    return mesh_from_cadquery(disc, "spray_button")


def _nozzle_mesh():
    """Front-facing nozzle tube in child-frame coords, protruding along +Y."""
    # Single tube with a wider base flange that merges into the button
    # Base flange (wider, short) for robust merge with button body
    base = (
        cq.Workplane("XY")
        .circle(NOZZLE_R * 2.0)
        .extrude(0.003)
    )
    base = base.rotate((0, 0, 0), (1, 0, 0), -90)
    base = base.translate((0, BUTTON_R - 0.003, NOZZLE_LOCAL_Z))

    # Main tube
    tube = (
        cq.Workplane("XY")
        .circle(NOZZLE_R)
        .extrude(NOZZLE_LEN)
    )
    tube = tube.rotate((0, 0, 0), (1, 0, 0), -90)
    tube = tube.translate((0, BUTTON_R - 0.001, NOZZLE_LOCAL_Z))

    # Small flared tip at end (overlapping with tube)
    tip_len = 0.003
    tip_start_y = BUTTON_R - 0.001 + NOZZLE_LEN - tip_len + 0.001
    tip = (
        cq.Workplane("XY")
        .circle(NOZZLE_R * 1.5)
        .extrude(tip_len)
    )
    tip = tip.rotate((0, 0, 0), (1, 0, 0), -90)
    tip = tip.translate((0, tip_start_y, NOZZLE_LOCAL_Z))

    nozzle = base.union(tube).union(tip)
    return mesh_from_cadquery(nozzle, "nozzle")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="serum_bottle_spray")

    amber = model.material("amber_glass", rgba=(0.45, 0.24, 0.06, 0.5))
    white = model.material("white_plastic", rgba=(0.95, 0.95, 0.94, 1.0))
    label_white = model.material("label_white", rgba=(0.97, 0.97, 0.96, 1.0))
    clear = model.material("clear_glass", rgba=(0.85, 0.90, 0.92, 0.35))
    grey = model.material("pump_grey", rgba=(0.55, 0.55, 0.56, 1.0))

    # ---- body (root) ----
    body = model.part("body")
    body.visual(_body_glass_mesh(), material=amber, name="body_glass")

    # white paper label band wrapping the middle of the body
    label = (
        cq.Workplane("XY")
        .workplane(offset=LABEL_Z0)
        .circle(LABEL_R)
        .circle(BODY_R - 0.0002)
        .extrude(LABEL_Z1 - LABEL_Z0)
    )
    body.visual(mesh_from_cadquery(label, "label_band"), material=label_white, name="label_band")

    # crimped collar fixed over the neck
    body.visual(_collar_mesh(), material=white, name="crimped_collar")

    # radial crimp ridges around the collar (repeated sub-parts via loop)
    for i in range(N_CRIMP_RIDGES):
        body.visual(_crimp_ridge_mesh(i), material=white, name=f"crimp_ridge_{i}")

    # dip tube with mounting flange (connects to collar bore)
    body.visual(_dip_tube_mesh(), material=clear, name="dip_tube")

    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, BODY_TOP),
        mass=0.040,
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP / 2.0)),
    )

    # ---- actuator (child, prismatic press-down) ----
    actuator = model.part("actuator")

    # flat spray button (child-frame coords: z=0 to BUTTON_H)
    actuator.visual(_button_mesh(), material=white, name="spray_button")

    # nozzle on the front (+Y) of the button (child-frame coords)
    actuator.visual(_nozzle_mesh(), material=white, name="nozzle")

    # pump stem going from button bottom down into the collar bore
    # In child frame: center at local z = STEM_LOCAL_BOTTOM + STEM_LEN/2
    stem_center_local = STEM_LOCAL_BOTTOM + STEM_LEN / 2.0
    actuator.visual(
        Cylinder(STEM_R, STEM_LEN),
        origin=Origin(xyz=(0.0, 0.0, stem_center_local)),
        material=grey,
        name="pump_stem",
    )

    actuator.inertial = Inertial.from_geometry(
        Cylinder(BUTTON_R, BUTTON_H),
        mass=0.005,
        origin=Origin(xyz=(0.0, 0.0, BUTTON_H / 2.0)),
    )

    # prismatic joint: origin at collar top (actual contact surface),
    # positive q presses the button DOWN
    model.articulation(
        "body_to_actuator",
        ArticulationType.PRISMATIC,
        parent=body,
        child=actuator,
        origin=Origin(xyz=(0.0, 0.0, COLLAR_TOP)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=PRESS_TRAVEL,
            effort=8.0,
            velocity=0.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    actuator = object_model.get_part("actuator")
    press = object_model.get_articulation("body_to_actuator")

    # --- bottle body is short and amber-glass ---
    body_aabb = ctx.part_world_aabb(body)
    bmn, bmx = body_aabb
    body_h = bmx[2] - bmn[2]
    body_dia = max(bmx[0] - bmn[0], bmx[1] - bmn[1])
    ctx.check(
        "bottle body is short — height under ~0.095 m",
        body_h < 0.095,
        details=f"body height={body_h:.4f}",
    )
    ctx.check(
        "bottle body is narrow (~0.03 m dia)",
        0.020 < body_dia < 0.040,
        details=f"body dia={body_dia:.4f}",
    )
    body_glass = body.get_visual("body_glass")
    mat = body_glass.material
    rgba = getattr(mat, "rgba", None)
    ctx.check(
        "body glass is amber-tinted and translucent",
        rgba is not None and rgba[0] > rgba[2] and rgba[3] < 0.95,
        details=f"amber_glass rgba={rgba}",
    )

    # --- white label band wraps the body ---
    label_aabb = ctx.part_element_world_aabb(body, elem="label_band")
    lmn, lmx = label_aabb
    ctx.check(
        "label band is on the body wall, around the middle",
        lmn[2] > 0.005 and lmx[2] < SHOULDER_TOP,
        details=f"label z=[{lmn[2]:.4f},{lmx[2]:.4f}]",
    )

    # --- crimped collar is fixed on the neck (part of body) ---
    collar_aabb = ctx.part_element_world_aabb(body, elem="crimped_collar")
    cmn, cmx = collar_aabb
    ctx.check(
        "crimped collar sits at the neck top above the shoulder",
        cmn[2] > SHOULDER_TOP - 0.005 and cmx[2] > NECK_TOP,
        details=f"collar z=[{cmn[2]:.4f},{cmx[2]:.4f}]",
    )

    # --- dip tube runs from collar area down into the bottle ---
    dip_aabb = ctx.part_element_world_aabb(body, elem="dip_tube")
    dmn, dmx = dip_aabb
    ctx.check(
        "dip tube reaches deep into the bottle",
        dmn[2] < SHOULDER_TOP - 0.020,
        details=f"dip_tube bottom z={dmn[2]:.4f}",
    )
    ctx.check(
        "dip tube top is near the collar/neck region",
        dmx[2] > SHOULDER_TOP,
        details=f"dip_tube top z={dmx[2]:.4f}",
    )

    # --- spray button sits on top of the collar at rest ---
    btn_aabb = ctx.part_element_world_aabb(actuator, elem="spray_button")
    btn_mn, btn_mx = btn_aabb
    ctx.check(
        "spray button sits at the collar top at rest",
        abs(btn_mn[2] - COLLAR_TOP) < 0.001,
        details=f"button bottom z(rest)={btn_mn[2]:.4f}, collar_top={COLLAR_TOP:.4f}",
    )

    # --- nozzle is on the front (+Y) of the button ---
    noz_aabb = ctx.part_element_world_aabb(actuator, elem="nozzle")
    nmn, nmx = noz_aabb
    ctx.check(
        "nozzle protrudes forward (+Y) from the button",
        nmx[1] > BUTTON_R,
        details=f"nozzle max y={nmx[1]:.4f}, button_r={BUTTON_R:.4f}",
    )

    # --- pump stem overlaps the collar bore (intentional insertion) ---
    ctx.allow_overlap(
        actuator,
        body,
        elem_a="pump_stem",
        elem_b="crimped_collar",
        reason="The pump stem is intentionally inserted down into the collar bore.",
    )
    ctx.expect_overlap(
        actuator,
        body,
        axes="z",
        elem_a="pump_stem",
        elem_b="crimped_collar",
        min_overlap=0.002,
        name="pump stem inserted into collar at rest",
    )

    # --- button contacts the collar top surface ---
    ctx.expect_contact(
        actuator,
        body,
        elem_a="spray_button",
        elem_b="crimped_collar",
        contact_tol=0.0005,
        name="button seated on collar top at rest",
    )

    # --- pressing the button moves it DOWN (prismatic) ---
    rest_pos = ctx.part_world_position(actuator)
    with ctx.pose({press: PRESS_TRAVEL}):
        pressed_pos = ctx.part_world_position(actuator)
        btn_pressed = ctx.part_element_world_aabb(actuator, elem="spray_button")
    ctx.check(
        "actuator translates straight down when pressed",
        pressed_pos[2] < rest_pos[2] - 0.003
        and abs(pressed_pos[0] - rest_pos[0]) < 1e-4
        and abs(pressed_pos[1] - rest_pos[1]) < 1e-4,
        details=f"rest={rest_pos}, pressed={pressed_pos}",
    )
    ctx.check(
        "button descends by approximately the press travel when actuated",
        abs((rest_pos[2] - pressed_pos[2]) - PRESS_TRAVEL) < 0.001,
        details=f"travel={rest_pos[2] - pressed_pos[2]:.4f}, expected={PRESS_TRAVEL:.4f}",
    )

    # --- at full press, button is still above collar bottom ---
    ctx.check(
        "pressed button stays above the collar bottom",
        btn_pressed[0][2] > COLLAR_Z0 + 0.001,
        details=f"pressed button bottom z={btn_pressed[0][2]:.4f}, collar_z0={COLLAR_Z0:.4f}",
    )

    # --- crimp ridges exist on the collar ---
    ridge_count = sum(
        1 for v in body.visuals if v.name.startswith("crimp_ridge_")
    )
    ctx.check(
        f"collar has {N_CRIMP_RIDGES} crimp ridges",
        ridge_count == N_CRIMP_RIDGES,
        details=f"found {ridge_count} ridges",
    )

    return ctx.report()


object_model = build_object_model()
