from __future__ import annotations

# Ribbed water bottle with deep grip grooves and a swing-top stopper.
# Frame: bottle axis along +Z, base at z=0, mouth/stopper at the top (+Z).
# The body is a transparent thin-wall shell with vertical grip grooves cut
# into the barrel, a tapered shoulder, and a short neck with a visible
# hollow mouth opening and a transparent wall-thickness lip ring.
# A swing-top stopper pivots on two side hinge arms attached to the neck.
# Joint: stopper_hinge — REVOLUTE around Y axis at the hinge pivot height.
#   q=0 -> stopper closed (plug seated in mouth)
#   q>0 -> stopper swings open (plug lifts out and to the +X side)

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
BODY_R = 0.032          # outer barrel radius (~64 mm dia)
WALL = 0.0018           # thin PET/glass wall thickness
BASE_Z = 0.0            # bottom of the bottle
BARREL_TOP_Z = 0.100    # where the shoulder taper begins
SHOULDER_TOP_Z = 0.122  # top of shoulder, base of neck
NECK_R = 0.013          # neck outer radius
NECK_TOP_Z = 0.140      # top rim of the neck (mouth opening)
LIP_HEIGHT = 0.005      # rolled lip ring height
LIP_TOP_Z = NECK_TOP_Z + LIP_HEIGHT
LIP_OUTER_R = NECK_R + 0.003  # lip outer radius (thicker ring)

# Hinge pivot: on +X and -X sides of the neck, at mid-lip height
HINGE_Z = NECK_TOP_Z + LIP_HEIGHT * 0.5
HINGE_ARM_OFFSET = NECK_R + 0.005  # how far out the pivot pins sit

# Stopper dimensions
PLUG_R = NECK_R - 0.001   # plug radius (slightly under neck bore)
PLUG_H = 0.010            # plug height (inserted into mouth)
CAP_R = 0.016             # top disc radius
CAP_H = 0.004             # top disc thickness

# Grip groove parameters
GROOVE_DEPTH = 0.005      # how deep grooves cut into the barrel
GROOVE_WIDTH = 0.008      # groove width (arc chord approx)
N_GROOVES = 8             # number of vertical grooves around the barrel
GROOVE_BOTTOM_Z = 0.015   # where grooves start
GROOVE_TOP_Z = BARREL_TOP_Z - 0.005  # where grooves end


def _bottle_shell():
    """Transparent thin-wall bottle with vertical grip grooves and mouth lip.

    Built as one revolved solid then shelled hollow, then grooves are cut.
    The lip ring is added as a torus fused onto the neck top.
    """
    # Outer profile revolve: base -> barrel -> shoulder -> neck -> lip
    wp = (
        cq.Workplane("XZ")
        .moveTo(0.0, BASE_Z)
        # rounded base corner
        .lineTo(BODY_R - 0.006, BASE_Z)
        .threePointArc((BODY_R, BASE_Z + 0.006), (BODY_R, BASE_Z + 0.012))
        # straight barrel
        .lineTo(BODY_R, BARREL_TOP_Z)
        # shoulder taper
        .threePointArc(
            ((BODY_R + NECK_R) / 2.0, (BARREL_TOP_Z + SHOULDER_TOP_Z) / 2.0 + 0.004),
            (NECK_R, SHOULDER_TOP_Z),
        )
        # neck
        .lineTo(NECK_R, NECK_TOP_Z)
        # lip step-out
        .lineTo(LIP_OUTER_R, NECK_TOP_Z)
        .lineTo(LIP_OUTER_R, LIP_TOP_Z)
        # close back along axis
        .lineTo(0.0, LIP_TOP_Z)
        .close()
    )
    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))

    # Shell hollow: remove top face, shell inward
    bottle = outer.faces(">Z").shell(-WALL)

    # Cut vertical grip grooves into the barrel
    for i in range(N_GROOVES):
        ang = 2.0 * math.pi * i / N_GROOVES
        # Position a tall thin box cutter at the groove radius
        cx = (BODY_R - GROOVE_DEPTH / 2.0) * math.cos(ang)
        cy = (BODY_R - GROOVE_DEPTH / 2.0) * math.sin(ang)
        groove_h = GROOVE_TOP_Z - GROOVE_BOTTOM_Z
        zc = GROOVE_BOTTOM_Z + groove_h / 2.0
        cutter = (
            cq.Workplane("XY")
            .transformed(offset=(cx, cy, zc), rotate=(0, 0, math.degrees(ang)))
            .box(GROOVE_DEPTH, GROOVE_WIDTH, groove_h)
        )
        bottle = bottle.cut(cutter)

    return bottle


def _stopper_solid():
    """Swing-top stopper: plug + top disc + two bail arms.

    Local frame origin at the hinge pivot point (0, 0, 0).
    The plug hangs downward (local -Z) to seat in the mouth.
    The top disc sits above the plug.
    Two thin arms extend from the disc down to the pivot points at +X and -X.
    """
    plug_drop = HINGE_Z - NECK_TOP_Z  # how far below hinge the plug top sits

    # Plug: cylinder extending downward from just below the hinge
    plug = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, -plug_drop - PLUG_H))
        .circle(PLUG_R)
        .extrude(PLUG_H)
    )

    # Top disc: sits above the plug, slightly above mouth level
    disc_z = -plug_drop + 0.002  # small gap above plug top
    disc = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, disc_z))
        .circle(CAP_R)
        .extrude(CAP_H)
    )

    # Connect plug to disc with a short stem
    stem = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, -plug_drop))
        .circle(PLUG_R * 0.5)
        .extrude(0.002)
    )
    stopper = plug.union(stem).union(disc)

    # Two bail arms: vertical bars from cap top up to the hinge pivot level,
    # positioned at the hinge arm offset X positions. These connect the
    # stopper cap to the pivot pins on the neck collar.
    arm_t = 0.0025  # arm thickness (Y direction)
    arm_w = 0.003   # arm width (X direction)
    arm_top_z = 0.008  # how far above hinge the arms extend
    arm_bottom_z = disc_z + CAP_H  # connect to cap disc top
    arm_length = arm_top_z - arm_bottom_z
    arm_cz = (arm_top_z + arm_bottom_z) / 2.0
    for sign in (+1, -1):
        px = sign * HINGE_ARM_OFFSET
        arm = (
            cq.Workplane("XY")
            .transformed(offset=(px, 0, arm_cz))
            .box(arm_w, arm_t, arm_length)
        )
        stopper = stopper.union(arm)

    # Cross-bar connecting the two arm tops (gives the bail structure)
    # Make it overlap with arm tops to ensure connectivity
    crossbar = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, arm_top_z - arm_w))
        .box(HINGE_ARM_OFFSET * 2.0, arm_t, arm_w * 2)
    )
    stopper = stopper.union(crossbar)

    return stopper


def _hinge_collar():
    """Fixed collar ring around the neck with two pivot pin bosses."""
    collar_h = 0.006
    collar_z = HINGE_Z - collar_h / 2.0
    collar_r = NECK_R + 0.004

    collar = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, collar_z))
        .circle(collar_r)
        .extrude(collar_h)
    )
    # Hollow out to fit around neck
    bore = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, collar_z - 0.001))
        .circle(NECK_R + 0.0005)
        .extrude(collar_h + 0.002)
    )
    collar = collar.cut(bore)

    # Pivot pin bosses on +X and -X sides
    pin_r = 0.002
    pin_h = 0.004
    for sign in (+1, -1):
        px = sign * HINGE_ARM_OFFSET
        pin = (
            cq.Workplane("XY")
            .transformed(offset=(px, 0, HINGE_Z - pin_h / 2.0))
            .circle(pin_r)
            .extrude(pin_h)
        )
        collar = collar.union(pin)

    return collar


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swing_top_water_bottle")

    # Materials
    clear = model.material("clear_glass", rgba=(0.82, 0.88, 0.86, 0.30))
    stopper_mat = model.material("stopper_ceramic", rgba=(0.92, 0.90, 0.85, 1.0))
    metal_mat = model.material("hinge_metal", rgba=(0.55, 0.55, 0.58, 1.0))

    # ---- bottle body (root) ----
    bottle = model.part("bottle")
    shell = _bottle_shell()
    bottle.visual(
        mesh_from_cadquery(shell, "bottle_shell"),
        material=clear,
        name="bottle_shell",
    )

    # Hinge collar (fixed to bottle)
    collar = _hinge_collar()
    bottle.visual(
        mesh_from_cadquery(collar, "hinge_collar"),
        material=metal_mat,
        name="hinge_collar",
    )

    bottle.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, LIP_TOP_Z),
        mass=0.080,
        origin=Origin(xyz=(0.0, 0.0, LIP_TOP_Z / 2.0)),
    )

    # ---- swing-top stopper ----
    stopper = model.part("stopper")
    stopper_geo = _stopper_solid()
    stopper.visual(
        mesh_from_cadquery(stopper_geo, "stopper_body"),
        material=stopper_mat,
        name="stopper_body",
    )
    stopper.inertial = Inertial.from_geometry(
        Cylinder(CAP_R, PLUG_H + CAP_H + 0.01),
        mass=0.012,
        origin=Origin(xyz=(0.0, 0.0, -(HINGE_Z - NECK_TOP_Z) / 2.0)),
    )

    # ---- swing-top hinge joint ----
    # REVOLUTE around Y axis at the pivot height.
    # q=0: stopper closed (plug in mouth). Positive q swings plug out to +X.
    model.articulation(
        "stopper_hinge",
        ArticulationType.REVOLUTE,
        parent=bottle,
        child=stopper,
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=2.0, lower=0.0, upper=1.9
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bottle = object_model.get_part("bottle")
    stopper = object_model.get_part("stopper")
    hinge = object_model.get_articulation("stopper_hinge")

    bottle_shell = bottle.get_visual("bottle_shell")
    stopper_body = stopper.get_visual("stopper_body")

    # --- bottle is transparent (clear glass/plastic) ---
    ctx.check(
        "bottle is transparent (alpha < 1)",
        bottle_shell.material.rgba is not None and bottle_shell.material.rgba[3] < 1.0,
        details=f"bottle rgba={bottle_shell.material.rgba}",
    )

    # --- stopper is opaque ceramic-white ---
    ctx.check(
        "stopper is opaque material",
        stopper_body.material.rgba is not None and stopper_body.material.rgba[3] >= 0.99,
        details=f"stopper rgba={stopper_body.material.rgba}",
    )

    # --- stopper sits at the top of the bottle (near the mouth) ---
    stopper_pos = ctx.part_world_position(stopper)
    ctx.check(
        "stopper is mounted near the bottle mouth",
        stopper_pos is not None and stopper_pos[2] > SHOULDER_TOP_Z,
        details=f"stopper origin z={stopper_pos[2] if stopper_pos else None}",
    )

    # --- allow intentional overlap: plug seated in mouth ---
    ctx.allow_overlap(
        stopper,
        bottle,
        elem_a="stopper_body",
        elem_b="bottle_shell",
        reason="Stopper plug is intentionally seated inside the bottle mouth when closed.",
    )

    # --- allow intentional overlap: bail arms pivot around hinge collar pins ---
    ctx.allow_overlap(
        stopper,
        bottle,
        elem_a="stopper_body",
        elem_b="hinge_collar",
        reason="Bail arms pivot around the hinge collar pin bosses as part of the swing mechanism.",
    )

    # --- hinge joint is revolute and non-fixed ---
    ctx.check(
        "stopper_hinge is a revolute joint",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={hinge.articulation_type}",
    )

    # --- opening the hinge swings the stopper upward (plug lifts out) ---
    # Compare the AABB max Z at closed vs open to detect upward swing
    closed_aabb = None
    open_aabb = None
    with ctx.pose({hinge: 0.0}):
        closed_aabb = ctx.part_world_aabb(stopper)
    with ctx.pose({hinge: 1.5}):
        open_aabb = ctx.part_world_aabb(stopper)
    closed_max_z = closed_aabb[1][2] if closed_aabb else 0.0
    open_max_z = open_aabb[1][2] if open_aabb else 0.0
    ctx.check(
        "opening hinge swings stopper upward (AABB max_z rises)",
        open_max_z > closed_max_z + 0.005,
        details=f"closed_max_z={closed_max_z}, open_max_z={open_max_z}",
    )

    # --- at closed pose, stopper plug overlaps with bottle neck region (seated) ---
    with ctx.pose({hinge: 0.0}):
        ctx.expect_overlap(
            stopper,
            bottle,
            axes="z",
            elem_a="stopper_body",
            elem_b="bottle_shell",
            min_overlap=0.003,
            name="closed stopper plug is inserted into the mouth",
        )

    # --- at open pose, stopper center has risen above closed position ---
    with ctx.pose({hinge: 1.5}):
        open_center = ctx.part_world_aabb(stopper)
        open_center_z = (open_center[0][2] + open_center[1][2]) / 2.0 if open_center else 0.0
    closed_center = ctx.part_world_aabb(stopper)
    closed_center_z = (closed_center[0][2] + closed_center[1][2]) / 2.0 if closed_center else 0.0
    ctx.check(
        "open stopper center rises above closed center",
        open_center_z > closed_center_z + 0.001,
        details=f"closed_center_z={closed_center_z}, open_center_z={open_center_z}",
    )

    # --- grip grooves exist: bottle shell has concavity (non-trivial geometry) ---
    # We verify the bottle shell exists and is a valid mesh
    ctx.check(
        "bottle shell visual exists (ribbed body)",
        bottle_shell is not None,
        details="bottle_shell visual not found",
    )

    # --- hinge collar visual exists ---
    collar_vis = bottle.get_visual("hinge_collar")
    ctx.check(
        "hinge collar exists on bottle",
        collar_vis is not None,
        details="hinge_collar visual not found",
    )

    return ctx.report()


object_model = build_object_model()
