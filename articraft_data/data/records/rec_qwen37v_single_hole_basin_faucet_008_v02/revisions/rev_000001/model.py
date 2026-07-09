from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Single-hole basin faucet: squat body on wide oval pedestal, single-lever
# handle (tilt for flow, cartridge rotation for temperature), curved spout
# with flip-open aerator.  All mirror chrome.
# World: +Z up, deck at z = 0, spout toward +X.
# ---------------------------------------------------------------------------

# Oval pedestal flange (sits on deck).
PED_RX = 0.040  # half-extent front-to-back (X)
PED_RY = 0.028  # half-extent side-to-side (Y)
PED_H = 0.007

# Squat cylindrical body.
BODY_R = 0.024
BODY_H = 0.058
BODY_Z0 = PED_H  # body bottom on pedestal top

# Grip grooves: shallow ring channels cut into the body sidewall.
GROOVE_DEPTH = 0.0015
GROOVE_H = 0.002
GROOVE_COUNT = 5
GROOVE_Z_START = 0.008  # first groove height in body-local frame
GROOVE_SPACING = 0.008

# Cartridge cap seam (thin dark ring at body / cartridge junction).
SEAM_R = BODY_R + 0.001
SEAM_H = 0.0012

# Cartridge housing (rotates for temperature).
CARTRIDGE_R = 0.017
CARTRIDGE_H = 0.013

# Lever handle.
LEVER_BOSS_R = 0.011
LEVER_BOSS_H = 0.007
LEVER_LEN = 0.052
LEVER_W = 0.013
LEVER_H = 0.005

# Spout exit height on body.
SPOUT_EXIT_Z = BODY_Z0 + BODY_H * 0.82

# Spout geometry constants (shared with hinge placement).
_SPOUT_SHANK_X0 = -(BODY_R - 0.004)  # starts inside body wall
_SPOUT_SHANK_X1 = 0.038
_SPOUT_BEND = 0.018
_SPOUT_END_X = _SPOUT_SHANK_X1 + _SPOUT_BEND   # 0.056
_SPOUT_END_Z = -_SPOUT_BEND                     # -0.018
SPOUT_R = 0.011

# Aerator disc.
AERATOR_R = 0.013
AERATOR_H = 0.003
# Hinge point: back edge of aerator disc, at the spout outlet bottom.
_AERATOR_HINGE_X = _SPOUT_END_X - AERATOR_R
_AERATOR_HINGE_Z = _SPOUT_END_Z - 0.005  # just below the flare bottom

# Joint limits.
CARTRIDGE_TURN_LIMIT = math.radians(45.0)
LEVER_TILT_LIMIT = math.radians(30.0)
AERATOR_OPEN_LIMIT = math.radians(75.0)


# ---- CadQuery geometry builders ------------------------------------------

def _build_pedestal() -> cq.Workplane:
    """Wide oval pedestal flange."""
    return cq.Workplane("XY").ellipse(PED_RX, PED_RY).extrude(PED_H)


def _build_body() -> cq.Workplane:
    """Squat cylindrical body with five grip-groove channels cut in."""
    body = cq.Workplane("XY").circle(BODY_R).extrude(BODY_H)
    # Cut shallow annular groove rings into the outer surface.
    for i in range(GROOVE_COUNT):
        z = GROOVE_Z_START + i * GROOVE_SPACING
        ring = (
            cq.Workplane("XY", origin=(0.0, 0.0, z))
            .circle(BODY_R + 0.001)
            .circle(BODY_R - GROOVE_DEPTH)
            .extrude(GROOVE_H)
        )
        body = body.cut(ring)
    return body


def _build_spout() -> cq.Workplane:
    """Curved hollow spout: straight shank → smooth downward bend → flared
    open outlet rim.  Built in spout-local frame (shank along +X)."""
    r_out = SPOUT_R
    path = (
        cq.Workplane("XZ")
        .moveTo(_SPOUT_SHANK_X0, 0.0)
        .lineTo(_SPOUT_SHANK_X1, 0.0)
        .tangentArcPoint((_SPOUT_BEND, -_SPOUT_BEND), relative=True)
    )
    tube = (
        cq.Workplane("YZ", origin=(_SPOUT_SHANK_X0, 0.0, 0.0))
        .circle(r_out)
        .sweep(path)
    )
    # Flared outlet skirt.
    flare = (
        cq.Workplane("XY", origin=(_SPOUT_END_X, 0.0, _SPOUT_END_Z + 0.005))
        .circle(r_out - 0.0002)
        .workplane(offset=-0.009)
        .circle(r_out + 0.004)
        .loft()
    )
    spout = tube.union(flare)
    # Tapered bore opening the outlet mouth.
    bore = (
        cq.Workplane("XY", origin=(_SPOUT_END_X, 0.0, _SPOUT_END_Z - 0.006))
        .circle(r_out + 0.002)
        .workplane(offset=0.016)
        .circle(r_out - 0.003)
        .loft()
    )
    spout = spout.cut(bore)
    # Small hinge lug at the back of the outlet for the aerator pivot.
    lug = (
        cq.Workplane("XZ", origin=(_AERATOR_HINGE_X, 0.0, _AERATOR_HINGE_Z))
        .circle(0.003)
        .extrude(AERATOR_R * 1.4, both=True)
    )
    spout = spout.union(lug)
    return spout


def _build_aerator() -> cq.Workplane:
    """Flat disc aerator with rim detail and a hinge knuckle at the back edge.
    Origin at the hinge pivot; disc centre is offset +X by AERATOR_R."""
    # Main disc.
    disc = (
        cq.Workplane("XY", origin=(AERATOR_R, 0.0, 0.0))
        .circle(AERATOR_R)
        .extrude(AERATOR_H)
    )
    # Thin rim ring around the disc bottom.
    rim = (
        cq.Workplane("XY", origin=(AERATOR_R, 0.0, -0.0005))
        .circle(AERATOR_R + 0.001)
        .circle(AERATOR_R - 0.002)
        .extrude(0.001)
    )
    # Hinge knuckle at the pivot (cylinder along Y, centered on origin).
    knuckle = (
        cq.Workplane("XZ")
        .circle(0.0025)
        .extrude(AERATOR_R * 1.2, both=True)
    )
    # Small bridge from knuckle to disc edge for connectivity.
    bridge = (
        cq.Workplane("XY", origin=(AERATOR_R * 0.5, 0.0, 0.0))
        .box(AERATOR_R, 0.005, AERATOR_H, centered=(True, True, False))
    )
    return disc.union(rim).union(knuckle).union(bridge)


def _build_cartridge() -> cq.Workplane:
    """Cylindrical cartridge housing with a chamfered top edge."""
    housing = cq.Workplane("XY").circle(CARTRIDGE_R).extrude(CARTRIDGE_H)
    try:
        housing = housing.edges(">Z").chamfer(0.001)
    except Exception:
        pass
    return housing


def _build_lever() -> cq.Workplane:
    """Lever handle: cylindrical boss + flat arm extending forward (+X)."""
    boss = cq.Workplane("XY").circle(LEVER_BOSS_R).extrude(LEVER_BOSS_H)
    # Arm extends in +X from boss centre.
    arm_cx = LEVER_BOSS_R * 0.5 + LEVER_LEN / 2.0
    arm = (
        cq.Workplane("XY", origin=(arm_cx, 0.0, 0.001))
        .box(LEVER_LEN, LEVER_W, LEVER_H, centered=(True, True, False))
    )
    lever = boss.union(arm)
    try:
        lever = lever.edges(">Z").fillet(0.0015)
    except Exception:
        pass  # fillet optional; geometry is valid without it
    return lever


# ---- Model assembly -------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    model.material("chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    model.material("chrome_dark", rgba=(0.42, 0.44, 0.47, 1.0))
    model.material("chrome_brushed", rgba=(0.70, 0.72, 0.74, 1.0))

    seam_z = BODY_Z0 + BODY_H  # top of body cylinder

    # ---- body (root): pedestal + body shell + seam ring ----
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_pedestal(), "pedestal", tolerance=0.0003),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="chrome",
        name="oval_pedestal",
    )
    body.visual(
        mesh_from_cadquery(_build_body(), "body_shell", tolerance=0.0003),
        origin=Origin(xyz=(0.0, 0.0, BODY_Z0)),
        material="chrome",
        name="body_shell",
    )
    # Cartridge cap seam ring (dark line at body / cartridge junction).
    body.visual(
        Cylinder(radius=SEAM_R, length=SEAM_H),
        origin=Origin(xyz=(0.0, 0.0, seam_z + SEAM_H / 2.0)),
        material="chrome_dark",
        name="cartridge_seam",
    )

    # ---- spout (fixed to body) ----
    spout = model.part("spout")
    spout.visual(
        mesh_from_cadquery(_build_spout(), "spout_tube", tolerance=0.0003),
        material="chrome",
        name="spout_tube",
    )
    model.articulation(
        "spout_mount",
        ArticulationType.FIXED,
        parent=body,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, SPOUT_EXIT_Z)),
    )

    # ---- cartridge (revolute on body for temperature) ----
    cartridge = model.part("cartridge")
    cartridge.visual(
        mesh_from_cadquery(_build_cartridge(), "cartridge_housing",
                           tolerance=0.0003),
        material="chrome",
        name="cartridge_housing",
    )
    model.articulation(
        "cartridge_turn",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cartridge,
        origin=Origin(xyz=(0.0, 0.0, seam_z + SEAM_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0,
            lower=-CARTRIDGE_TURN_LIMIT,
            upper=CARTRIDGE_TURN_LIMIT,
        ),
    )

    # ---- lever (revolute on cartridge for flow on/off) ----
    lever = model.part("lever")
    lever.visual(
        mesh_from_cadquery(_build_lever(), "lever_handle", tolerance=0.0003),
        material="chrome",
        name="lever_handle",
    )
    # axis = (0, -1, 0): right-hand rule around -Y lifts +X end toward +Z,
    # so positive q raises the front of the lever (flow on).
    model.articulation(
        "lever_tilt",
        ArticulationType.REVOLUTE,
        parent=cartridge,
        child=lever,
        origin=Origin(xyz=(0.0, 0.0, CARTRIDGE_H)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0,
            lower=0.0,
            upper=LEVER_TILT_LIMIT,
        ),
    )

    # ---- aerator (revolute hinge on spout for flip-open) ----
    aerator = model.part("aerator")
    aerator.visual(
        mesh_from_cadquery(_build_aerator(), "aerator_disc", tolerance=0.0003),
        material="chrome_brushed",
        name="aerator_disc",
    )
    # axis = (0, 1, 0): right-hand rule around +Y swings the +X end toward -Z
    # (downward), opening the aerator away from the spout outlet.
    model.articulation(
        "aerator_hinge",
        ArticulationType.REVOLUTE,
        parent=spout,
        child=aerator,
        origin=Origin(xyz=(_AERATOR_HINGE_X, 0.0, _AERATOR_HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0, velocity=2.0,
            lower=0.0,
            upper=AERATOR_OPEN_LIMIT,
        ),
    )

    return model


# ---- Tests ----------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    spout = object_model.get_part("spout")
    cartridge = object_model.get_part("cartridge")
    lever = object_model.get_part("lever")
    aerator = object_model.get_part("aerator")

    cart_turn = object_model.get_articulation("cartridge_turn")
    lev_tilt = object_model.get_articulation("lever_tilt")
    aer_hinge = object_model.get_articulation("aerator_hinge")

    # Intentional seated insertions (scoped per element).
    ctx.allow_overlap(
        spout, body,
        elem_a="spout_tube", elem_b="body_shell",
        reason="Spout shank is intentionally seated inside the body casting.",
    )
    ctx.allow_overlap(
        aerator, spout,
        elem_a="aerator_disc", elem_b="spout_tube",
        reason="Aerator disc seats against the spout outlet rim when closed.",
    )

    # ---- Variant geometry: oval pedestal wider than body ----
    ped_aabb = ctx.part_element_world_aabb(body, elem="oval_pedestal")
    body_aabb = ctx.part_element_world_aabb(body, elem="body_shell")
    ctx.check(
        "oval pedestal wider than body in both X and Y",
        ped_aabb is not None and body_aabb is not None
        and (ped_aabb[1][0] - ped_aabb[0][0])
            > (body_aabb[1][0] - body_aabb[0][0]) + 0.010
        and (ped_aabb[1][1] - ped_aabb[0][1])
            > (body_aabb[1][1] - body_aabb[0][1]) + 0.004,
        details=f"ped={ped_aabb}, body={body_aabb}",
    )

    # ---- Pedestal on deck ----
    ctx.check(
        "pedestal sits flat on the deck",
        ped_aabb is not None and abs(ped_aabb[0][2]) <= 0.0005,
        details=f"ped aabb={ped_aabb}",
    )

    # ---- Squat body: overall height well under parent's 0.13 m ----
    lever_aabb = ctx.part_world_aabb(lever)
    ctx.check(
        "overall faucet height is squat (under 0.12 m)",
        lever_aabb is not None and lever_aabb[1][2] < 0.12,
        details=f"lever aabb={lever_aabb}",
    )

    # ---- Cartridge cap seam visible below the lever ----
    seam_vis = body.get_visual("cartridge_seam")
    ctx.check(
        "cartridge cap seam ring present below lever",
        seam_vis is not None,
    )
    # Seam is between body top and cartridge bottom.
    seam_aabb = ctx.part_element_world_aabb(body, elem="cartridge_seam")
    cart_aabb = ctx.part_world_aabb(cartridge)
    ctx.check(
        "seam ring sits between body top and cartridge bottom",
        seam_aabb is not None and cart_aabb is not None
        and seam_aabb[0][2] >= BODY_Z0 + BODY_H - 0.001
        and seam_aabb[1][2] <= cart_aabb[1][2] + 0.001,
        details=f"seam={seam_aabb}, cartridge={cart_aabb}",
    )

    # ---- Body shell has grip grooves (boolean cuts in the mesh) ----
    shell_vis = body.get_visual("body_shell")
    ctx.check(
        "body shell with grip grooves exists",
        shell_vis is not None,
    )

    # ---- Spout projects forward and curves down ----
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "spout extends forward and droops above the deck",
        spout_aabb is not None
        and spout_aabb[1][0] > 0.050
        and spout_aabb[0][2] < SPOUT_EXIT_Z - 0.005
        and spout_aabb[0][2] > 0.005,
        details=f"spout aabb={spout_aabb}",
    )

    # ---- Joint limits match the prompt ----
    ctl = cart_turn.motion_limits
    ctx.check(
        "cartridge turn limits are ±45°",
        ctl is not None and ctl.lower is not None and ctl.upper is not None
        and abs(ctl.lower + CARTRIDGE_TURN_LIMIT) < 1e-6
        and abs(ctl.upper - CARTRIDGE_TURN_LIMIT) < 1e-6,
        details=f"limits={ctl}",
    )
    ltl = lev_tilt.motion_limits
    ctx.check(
        "lever tilt limits are 0 to 30°",
        ltl is not None and ltl.lower is not None and ltl.upper is not None
        and abs(ltl.lower) < 1e-9
        and abs(ltl.upper - LEVER_TILT_LIMIT) < 1e-6,
        details=f"limits={ltl}",
    )
    ahl = aer_hinge.motion_limits
    ctx.check(
        "aerator hinge limits are 0 to 75°",
        ahl is not None and ahl.lower is not None and ahl.upper is not None
        and abs(ahl.lower) < 1e-9
        and abs(ahl.upper - AERATOR_OPEN_LIMIT) < 1e-6,
        details=f"limits={ahl}",
    )

    # ---- Non-fixed joint type checks ----
    ctx.check(
        "cartridge_turn is revolute",
        cart_turn.articulation_type == ArticulationType.REVOLUTE,
    )
    ctx.check(
        "lever_tilt is revolute",
        lev_tilt.articulation_type == ArticulationType.REVOLUTE,
    )
    ctx.check(
        "aerator_hinge is revolute",
        aer_hinge.articulation_type == ArticulationType.REVOLUTE,
    )

    # ---- Decisive pose: lever tilts up (flow on) ----
    rest_aabb = ctx.part_world_aabb(lever)
    with ctx.pose({lev_tilt: LEVER_TILT_LIMIT}):
        tilted_aabb = ctx.part_world_aabb(lever)
    ctx.check(
        "lever tilt raises the front of the handle (flow on)",
        rest_aabb is not None and tilted_aabb is not None
        and tilted_aabb[1][2] > rest_aabb[1][2] + 0.002,
        details=f"rest={rest_aabb}, tilted={tilted_aabb}",
    )

    # ---- Decisive pose: aerator opens ----
    aer_rest_aabb = ctx.part_world_aabb(aerator)
    with ctx.pose({aer_hinge: AERATOR_OPEN_LIMIT}):
        aer_open_aabb = ctx.part_world_aabb(aerator)
    ctx.check(
        "aerator hinge swings the disc away from the spout outlet",
        aer_rest_aabb is not None and aer_open_aabb is not None
        and aer_open_aabb[0][2] < aer_rest_aabb[0][2] - 0.002,
        details=f"rest={aer_rest_aabb}, open={aer_open_aabb}",
    )

    # ---- Decisive pose: cartridge turn swings lever sideways ----
    cart_rest_aabb = ctx.part_world_aabb(lever)
    with ctx.pose({cart_turn: CARTRIDGE_TURN_LIMIT}):
        cart_turned_aabb = ctx.part_world_aabb(lever)
    ctx.check(
        "cartridge rotation swings the lever sideways (temperature)",
        cart_rest_aabb is not None and cart_turned_aabb is not None
        and (cart_turned_aabb[1][1] - cart_turned_aabb[0][1])
            > (cart_rest_aabb[1][1] - cart_rest_aabb[0][1]) + 0.004,
        details=f"rest={cart_rest_aabb}, turned={cart_turned_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
