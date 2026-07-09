from __future__ import annotations

# Sports bottle with flip straw cap.
# Frame: bottle axis along +Z, base at z=0, neck/cap at the top (+Z).
# The body is a translucent sports bottle with ergonomic grip zones.
# The cap has:
#   - A fixed cap base ring that stays on the bottle neck
#   - A gasket ring (rubber seal) visible below the cap base
#   - A flip cap lid that opens on a revolute hinge at the rear
#   - A hollow mouth opening visible under the cap when open
#   - A straw protruding from the mouth opening

import math

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

# ---- key dimensions (meters) ----
BODY_R = 0.035          # outer barrel radius (~0.07 m dia, wider than juice bottle)
WALL = 0.0020           # slightly thicker for sports bottle
BASE_Z = 0.0            # bottom of the bottle
BARREL_TOP_Z = 0.160    # where the shoulder taper begins (taller bottle)
SHOULDER_TOP_Z = 0.185  # top of the shoulder, base of the neck
NECK_R = 0.016          # neck outer radius
NECK_TOP_Z = 0.210      # top rim of the neck

# Cap base dimensions
CAP_BASE_R = 0.020      # cap base outer radius
CAP_BASE_HEIGHT = 0.018 # cap base ring height
CAP_BASE_Z = NECK_TOP_Z - 0.012  # cap base slips over the neck (intentional overlap)

# Gasket dimensions
GASKET_R = 0.019        # gasket outer radius
GASKET_HEIGHT = 0.004   # gasket thickness
GASKET_INNER_R = NECK_R - 0.0005  # compression fit on neck for reliable contact
GASKET_Z = CAP_BASE_Z - GASKET_HEIGHT  # gasket sits directly below cap base

# Flip cap dimensions
FLIP_CAP_R = 0.019      # flip cap lid radius
FLIP_CAP_HEIGHT = 0.012 # flip cap lid height

# Straw dimensions
STRAW_R = 0.004         # straw outer radius
STRAW_HEIGHT = 0.035    # straw height (extends from inside neck to above cap)
STRAW_Z = NECK_TOP_Z - 0.015  # straw base sits inside the neck


def _bottle_shell():
    # Translucent sports bottle as one solid revolve with grip indentations,
    # then shelled open at the top so the neck reads hollow.
    wp = (
        cq.Workplane("XZ")
        .moveTo(0.0, BASE_Z)
        # rounded base corner
        .lineTo(BODY_R - 0.008, BASE_Z)
        .threePointArc((BODY_R, BASE_Z + 0.008), (BODY_R, BASE_Z + 0.016))
        # straight cylindrical barrel with slight grip taper
        .lineTo(BODY_R, BASE_Z + 0.040)
        .lineTo(BODY_R - 0.002, BASE_Z + 0.060)  # grip indent start
        .lineTo(BODY_R - 0.002, BASE_Z + 0.100)  # grip indent middle
        .lineTo(BODY_R, BASE_Z + 0.120)  # grip indent end
        .lineTo(BODY_R, BARREL_TOP_Z)
        # shoulder taper up to the neck
        .threePointArc(
            ((BODY_R + NECK_R) / 2.0, (BARREL_TOP_Z + SHOULDER_TOP_Z) / 2.0 + 0.005),
            (NECK_R, SHOULDER_TOP_Z),
        )
        # short neck
        .lineTo(NECK_R, NECK_TOP_Z)
    )
    # close back along the axis
    wp = wp.lineTo(0.0, NECK_TOP_Z).close()
    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))
    # Hollow it: remove the top neck face and shell inward
    return outer.faces(">Z").shell(-WALL)


def _cap_base_solid():
    # Fixed cap base ring that stays on the bottle neck.
    # Local frame: origin at the cap base bottom center (0,0,0).
    # The ring extends from z=0 to z=CAP_BASE_HEIGHT.
    # Hinge ears extend upward from the ring at the rear (-Y).
    
    # Outer ring - wider at top to include ear bases
    outer = (
        cq.Workplane("XY")
        .circle(CAP_BASE_R)
        .extrude(CAP_BASE_HEIGHT)
    )
    # Hollow it to slip over the neck
    cavity = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, -0.001))
        .circle(NECK_R + 0.001)
        .extrude(CAP_BASE_HEIGHT + 0.002)
    )
    cap_base = outer.cut(cavity)
    
    # Hinge ears: positioned at the rear edge of the ring, extending upward.
    # The ear base overlaps with the ring top to ensure connectivity.
    ear_y = -(CAP_BASE_R - 0.003)  # near rear edge of ring
    ear_z_start = CAP_BASE_HEIGHT - 0.004  # overlap into ring for connection
    ear_height = 0.010
    ear_z_center = ear_z_start + ear_height / 2.0
    
    for dx in (-0.006, 0.006):
        ear = (
            cq.Workplane("XY")
            .transformed(offset=(dx, ear_y, ear_z_center))
            .box(0.004, 0.006, ear_height + 0.004)  # extra height for overlap
        )
        cap_base = cap_base.union(ear)
    
    return cap_base


def _gasket_ring():
    # Rubber gasket ring. Local frame: origin at (0,0,0).
    # Sits snugly around the neck, directly below the cap base.
    outer = (
        cq.Workplane("XY")
        .circle(GASKET_R)
        .extrude(GASKET_HEIGHT)
    )
    # Hollow center - tight fit on neck
    inner = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, -0.001))
        .circle(GASKET_INNER_R)
        .extrude(GASKET_HEIGHT + 0.002)
    )
    return outer.cut(inner)


def _flip_cap_lid():
    # Flip cap lid that opens on a hinge.
    # Local frame: origin at the hinge pivot point (0,0,0).
    # The lid disc center is offset forward (+Y) from the pivot.
    # At q=0 (closed), the lid lies flat, covering the mouth from above.
    # At q>0, it rotates around +X axis to open upward.
    
    lid_offset_y = CAP_BASE_R - 0.003  # forward from pivot to lid center
    
    # Main lid disc - sits just above the pivot when closed
    lid = (
        cq.Workplane("XY")
        .transformed(offset=(0, lid_offset_y, 0))
        .circle(FLIP_CAP_R)
        .extrude(FLIP_CAP_HEIGHT)
    )
    
    # Hinge barrel at the pivot point - connects lid to cap base ears
    barrel = (
        cq.Workplane("XZ")
        .circle(0.0025)
        .extrude(0.010, both=True)
    )
    lid = lid.union(barrel)
    
    return lid


def _straw():
    # Straw that protrudes from the mouth opening. Local frame: origin at (0,0,0).
    # Has a wider base flange that seats inside the neck for support.
    
    # Main straw tube
    straw = (
        cq.Workplane("XY")
        .circle(STRAW_R)
        .extrude(STRAW_HEIGHT)
    )
    # Hollow the straw (but leave the bottom solid for the flange connection)
    hollow = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, 0.005))
        .circle(STRAW_R - 0.0015)
        .extrude(STRAW_HEIGHT - 0.004)
    )
    straw = straw.cut(hollow)
    
    # Base flange - wider ring that contacts the neck inner wall
    # This is a solid disc that naturally merges with the straw tube
    flange = (
        cq.Workplane("XY")
        .circle(NECK_R - WALL + 0.0002)  # slightly larger than inner neck for contact
        .extrude(0.005)
    )
    straw = straw.union(flange)
    return straw


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sports_bottle")

    # Materials
    translucent = model.material("translucent_plastic", rgba=(0.82, 0.87, 0.92, 0.35))
    black = model.material("cap_black", rgba=(0.06, 0.06, 0.07, 1.0))
    gray = model.material("gasket_rubber", rgba=(0.30, 0.30, 0.32, 1.0))
    white = model.material("straw_white", rgba=(0.95, 0.95, 0.96, 0.9))

    # ---- bottle body (root) ----
    body = model.part("bottle")
    shell = _bottle_shell()
    body.visual(mesh_from_cadquery(shell, "bottle_shell"), material=translucent, name="bottle_shell")
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, NECK_TOP_Z),
        mass=0.045,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    # ---- gasket ring (mounted to body via FIXED joint) ----
    gasket = model.part("gasket")
    gasket_geo = _gasket_ring()
    gasket.visual(
        mesh_from_cadquery(gasket_geo, "gasket_ring"),
        material=gray,
        name="gasket_ring",
    )

    # ---- cap base (fixed ring on neck, mounted to body) ----
    cap_base = model.part("cap_base")
    cap_base_geo = _cap_base_solid()
    cap_base.visual(
        mesh_from_cadquery(cap_base_geo, "cap_base_ring"),
        material=black,
        name="cap_base_ring",
    )

    # ---- flip cap lid (articulated, child of cap_base) ----
    flip_cap = model.part("flip_cap")
    flip_cap_geo = _flip_cap_lid()
    flip_cap.visual(
        mesh_from_cadquery(flip_cap_geo, "flip_cap_lid"),
        material=black,
        name="flip_cap_lid",
    )
    flip_cap.inertial = Inertial.from_geometry(
        Cylinder(FLIP_CAP_R, FLIP_CAP_HEIGHT),
        mass=0.008,
        origin=Origin(xyz=(0.0, CAP_BASE_R - 0.003, FLIP_CAP_HEIGHT / 2.0)),
    )

    # ---- straw (mounted to body via FIXED joint) ----
    straw = model.part("straw")
    straw_geo = _straw()
    straw.visual(
        mesh_from_cadquery(straw_geo, "straw_tube"),
        material=white,
        name="straw_tube",
    )

    # ---- joints ----
    
    # Gasket fixed to bottle at gasket position
    model.articulation(
        "body_to_gasket",
        ArticulationType.FIXED,
        parent=body,
        child=gasket,
        origin=Origin(xyz=(0.0, 0.0, GASKET_Z)),
    )

    # Cap base fixed to bottle at cap base position
    model.articulation(
        "body_to_cap_base",
        ArticulationType.FIXED,
        parent=body,
        child=cap_base,
        origin=Origin(xyz=(0.0, 0.0, CAP_BASE_Z)),
    )

    # Straw fixed to bottle at straw position
    model.articulation(
        "body_to_straw",
        ArticulationType.FIXED,
        parent=body,
        child=straw,
        origin=Origin(xyz=(0.0, 0.0, STRAW_Z)),
    )

    # Flip cap hinge: revolute joint at the rear of the cap base
    # The pivot is at the rear edge of cap base, at the top of the ears
    # In cap_base local frame: y = -(CAP_BASE_R - 0.003), z = CAP_BASE_HEIGHT + 0.004
    ear_y_local = -(CAP_BASE_R - 0.003)
    pivot_z_local = CAP_BASE_HEIGHT + 0.004
    
    model.articulation(
        "cap_flip",
        ArticulationType.REVOLUTE,
        parent=cap_base,
        child=flip_cap,
        origin=Origin(xyz=(0.0, ear_y_local, pivot_z_local)),
        axis=(1.0, 0.0, 0.0),  # hinge axis along X (opens by rotating around X)
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,  # closed position (lid flat on top)
            upper=2.2,  # open position (~126 degrees)
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle")
    gasket = object_model.get_part("gasket")
    cap_base = object_model.get_part("cap_base")
    flip_cap = object_model.get_part("flip_cap")
    straw = object_model.get_part("straw")
    
    cap_flip = object_model.get_articulation("cap_flip")

    bottle_shell = body.get_visual("bottle_shell")
    flip_cap_lid = flip_cap.get_visual("flip_cap_lid")
    gasket_ring = gasket.get_visual("gasket_ring")
    cap_base_ring = cap_base.get_visual("cap_base_ring")
    straw_tube = straw.get_visual("straw_tube")

    # --- bottle is translucent (alpha < 1) ---
    ctx.check(
        "bottle material is translucent (alpha < 1)",
        bottle_shell.material.rgba is not None and bottle_shell.material.rgba[3] < 1.0,
        details=f"bottle rgba={bottle_shell.material.rgba}",
    )

    # --- flip cap is opaque black ---
    ctx.check(
        "flip cap material is opaque black",
        flip_cap_lid.material.rgba is not None
        and flip_cap_lid.material.rgba[3] >= 0.99
        and max(flip_cap_lid.material.rgba[:3]) < 0.2,
        details=f"flip cap rgba={flip_cap_lid.material.rgba}",
    )

    # --- gasket ring is visible (opaque gray rubber) ---
    ctx.check(
        "gasket ring is visible opaque rubber",
        gasket_ring.material.rgba is not None
        and gasket_ring.material.rgba[3] >= 0.99,
        details=f"gasket rgba={gasket_ring.material.rgba}",
    )

    # --- gasket ring sits below cap base ---
    gasket_pos = ctx.part_world_position(gasket)
    cap_base_pos = ctx.part_world_position(cap_base)
    ctx.check(
        "gasket ring sits below cap base",
        gasket_pos is not None and cap_base_pos is not None and gasket_pos[2] < cap_base_pos[2],
        details=f"gasket_z={gasket_pos[2] if gasket_pos else None}, cap_base_z={cap_base_pos[2] if cap_base_pos else None}",
    )

    # --- cap base is above the bottle shoulder ---
    ctx.check(
        "cap base sits above the shoulder",
        cap_base_pos is not None and cap_base_pos[2] > SHOULDER_TOP_Z,
        details=f"cap_base_z={cap_base_pos[2] if cap_base_pos else None}",
    )

    # --- straw protrudes above the neck top ---
    straw_pos = ctx.part_world_position(straw)
    ctx.check(
        "straw is mounted in the neck area",
        straw_pos is not None and straw_pos[2] > SHOULDER_TOP_Z,
        details=f"straw_z={straw_pos[2] if straw_pos else None}",
    )

    # --- cap_flip joint is revolute ---
    ctx.check(
        "cap_flip joint is revolute",
        cap_flip.articulation_type == ArticulationType.REVOLUTE,
        details=f"cap_flip type={cap_flip.articulation_type}",
    )

    # --- flip cap opens on the hinge (AABB check since pivot doesn't move) ---
    with ctx.pose({cap_flip: 0.0}):
        closed_aabb = ctx.part_world_aabb(flip_cap)
    with ctx.pose({cap_flip: 1.5}):
        open_aabb = ctx.part_world_aabb(flip_cap)
    
    if closed_aabb is not None and open_aabb is not None:
        closed_z_max = closed_aabb[1][2]  # max z when closed
        open_z_max = open_aabb[1][2]      # max z when open
        open_y_max = open_aabb[1][1]      # max y when open
        
        ctx.check(
            "flip cap opens upward on the hinge",
            open_z_max > closed_z_max + 0.005,
            details=f"closed_z_max={closed_z_max}, open_z_max={open_z_max}",
        )
        
        # When open, the lid should swing backward (Y position changes)
        closed_y_max = closed_aabb[1][1]
        ctx.check(
            "flip cap lid swings when opened",
            abs(open_y_max - closed_y_max) > 0.005 or abs(open_z_max - closed_z_max) > 0.005,
            details=f"closed_y_max={closed_y_max}, open_y_max={open_y_max}",
        )
    else:
        ctx.fail("flip cap AABB check", "Could not get AABB for flip cap")

    # --- flip cap at closed position covers the cap area ---
    with ctx.pose({cap_flip: 0.0}):
        ctx.expect_overlap(
            flip_cap, cap_base,
            axes="xy",
            min_overlap=0.005,
            name="closed flip cap overlaps cap base in XY",
        )

    # --- flip cap joint has proper limits ---
    ctx.check(
        "cap_flip has proper motion limits",
        cap_flip.motion_limits is not None
        and cap_flip.motion_limits.lower == 0.0
        and cap_flip.motion_limits.upper > 1.0,
        details=f"limits={cap_flip.motion_limits}",
    )

    # --- Intentional overlaps ---
    # The cap base sits over the bottle neck (intentional seated fit)
    ctx.allow_overlap(
        cap_base, body,
        elem_a="cap_base_ring",
        elem_b="bottle_shell",
        reason="Cap base ring is intentionally seated over the bottle neck.",
    )
    ctx.expect_contact(
        cap_base, body,
        elem_a="cap_base_ring",
        elem_b="bottle_shell",
        name="cap base contacts bottle neck",
    )

    # Gasket sits snugly on the neck below the cap base (intentional compression fit)
    ctx.allow_overlap(
        gasket, body,
        elem_a="gasket_ring",
        elem_b="bottle_shell",
        reason="Gasket ring is intentionally compressed onto the bottle neck.",
    )
    ctx.expect_contact(
        gasket, body,
        elem_a="gasket_ring",
        elem_b="bottle_shell",
        name="gasket contacts bottle neck",
    )
    ctx.expect_contact(
        gasket, cap_base,
        elem_a="gasket_ring",
        elem_b="cap_base_ring",
        name="gasket contacts cap base",
    )

    # Straw flange is press-fit inside the bottle neck (intentional seated fit)
    ctx.allow_overlap(
        straw, body,
        elem_a="straw_tube",
        elem_b="bottle_shell",
        reason="Straw base flange is intentionally press-fit inside the bottle neck.",
    )
    ctx.expect_contact(
        straw, body,
        elem_a="straw_tube",
        elem_b="bottle_shell",
        name="straw flange contacts bottle neck",
    )

    # Straw passes through the cap base and flip cap (intentional)
    ctx.allow_overlap(
        straw, cap_base,
        elem_a="straw_tube",
        elem_b="cap_base_ring",
        reason="Straw intentionally passes through the cap base opening.",
    )
    ctx.allow_overlap(
        straw, flip_cap,
        elem_a="straw_tube",
        elem_b="flip_cap_lid",
        reason="Straw intentionally passes through the flip cap when closed.",
    )

    # Flip cap lid and hinge overlap at pivot (intentional hinge connection)
    ctx.allow_overlap(
        flip_cap, cap_base,
        elem_a="flip_cap_lid",
        elem_b="cap_base_ring",
        reason="Flip cap hinge barrel intentionally connects to the cap base ears.",
    )

    return ctx.report()


object_model = build_object_model()
