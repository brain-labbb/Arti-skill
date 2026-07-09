from __future__ import annotations

# Sports bottle with flip straw cap, safety collar ring, and hollow mouth.
# Frame: bottle axis along +Z, base at z=0, neck/cap at the top (+Z).
#
# Structure:
#   bottle (root) -> translucent body with shoulder, threaded neck, hollow
#                    mouth opening, and a transparent wall-thickness lip ring
#   collar       -> safety collar ring around the neck (below cap), rotates
#   cap_base     -> screw-on flip-cap base with hinge ears and straw opening
#   flip_lid     -> flip-up straw cover with nozzle dome, hinged at the rear
#
# Joints:
#   collar_spin  -> CONTINUOUS rotation of the collar around +Z
#   cap_rotate   -> CONTINUOUS rotation of the cap assembly around +Z
#   lid_flip     -> REVOLUTE hinge at the rear of the cap, positive opens up

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
BODY_R = 0.033            # outer barrel radius (~66mm dia sports bottle)
WALL = 0.0018             # translucent wall thickness
BASE_Z = 0.0              # bottom of bottle
BARREL_TOP_Z = 0.110      # where shoulder taper begins
SHOULDER_TOP_Z = 0.134    # top of shoulder / base of neck
NECK_R = 0.015            # neck outer radius (under threads)
NECK_TOP_Z = 0.152        # top rim of the neck (mouth opening)
MOUTH_R = 0.011           # inner mouth opening radius
LIP_HEIGHT = 0.004        # height of the wall-thickness lip ring above neck top
LIP_OUTER_R = NECK_R + 0.001  # outer radius of the lip ring
LIP_INNER_R = MOUTH_R       # inner radius matches the mouth opening

# Collar dimensions
COLLAR_Z = SHOULDER_TOP_Z + 0.001  # sits just above shoulder
COLLAR_HEIGHT = 0.006
COLLAR_OUTER_R = NECK_R + 0.004
COLLAR_INNER_R = NECK_R + 0.0005  # slight clearance over neck

# Cap dimensions
CAP_R = 0.019             # cap outer radius
CAP_HEIGHT = 0.014        # cap base height (the threaded ring portion)
CAP_SKIRT_DROP = 0.010    # how far the skirt drops below cap origin onto neck

# Hinge geometry
HINGE_EAR_WIDTH = 0.006   # width of hinge ear
HINGE_EAR_HEIGHT = 0.010  # height of hinge ear above cap top
HINGE_PIN_R = 0.0015      # hinge pin radius
HINGE_REAR_OFFSET = -(CAP_R - 0.004)  # hinge at rear edge of cap

# Flip lid dimensions
LID_RADIUS = 0.017        # lid dome outer radius
LID_HEIGHT = 0.018        # lid dome height
STRAW_R = 0.003           # straw nozzle radius
STRAW_HEIGHT = 0.012      # straw nozzle height above lid


def _neck_thread_profile():
    """Sawtooth thread ridges on the neck."""
    pts = [(NECK_R, SHOULDER_TOP_Z)]
    z0 = SHOULDER_TOP_Z + 0.003
    ridge_r = NECK_R + 0.0014
    for k in range(3):
        zc = z0 + k * 0.004
        pts.append((NECK_R, zc - 0.0014))
        pts.append((ridge_r, zc - 0.0005))
        pts.append((ridge_r, zc + 0.0005))
        pts.append((NECK_R, zc + 0.0014))
    pts.append((NECK_R, NECK_TOP_Z))
    return pts


def _bottle_shell():
    """Translucent sports bottle with hollow body, shoulder, threaded neck,
    and a visible wall-thickness lip ring at the mouth opening."""
    # Outer profile: base -> barrel -> shoulder -> neck -> lip
    wp = (
        cq.Workplane("XZ")
        .moveTo(0.0, BASE_Z)
        # rounded base corner
        .lineTo(BODY_R - 0.007, BASE_Z)
        .threePointArc((BODY_R, BASE_Z + 0.007), (BODY_R, BASE_Z + 0.014))
        # straight barrel with slight grip waist
        .lineTo(BODY_R, 0.040)
        .threePointArc(
            (BODY_R - 0.002, 0.065),
            (BODY_R, 0.090),
        )
        .lineTo(BODY_R, BARREL_TOP_Z)
        # shoulder taper
        .threePointArc(
            ((BODY_R + NECK_R) / 2.0, (BARREL_TOP_Z + SHOULDER_TOP_Z) / 2.0 + 0.004),
            (NECK_R, SHOULDER_TOP_Z),
        )
    )
    # Threaded neck profile
    for (r, z) in _neck_thread_profile()[1:]:
        wp = wp.lineTo(r, z)
    # Lip ring at mouth: step out to lip outer, go up, step in to mouth opening
    wp = wp.lineTo(LIP_OUTER_R, NECK_TOP_Z)
    wp = wp.lineTo(LIP_OUTER_R, NECK_TOP_Z + LIP_HEIGHT)
    wp = wp.lineTo(LIP_INNER_R, NECK_TOP_Z + LIP_HEIGHT)
    # close along axis
    wp = wp.lineTo(0.0, NECK_TOP_Z + LIP_HEIGHT).close()
    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))
    # Shell it: hollow inside, open at top mouth
    return outer.faces(">Z").shell(-WALL)


def _collar_ring():
    """Safety collar ring in local frame: z=0 at joint origin, ring extends upward."""
    outer = (
        cq.Workplane("XY")
        .circle(COLLAR_OUTER_R)
        .circle(COLLAR_INNER_R)
        .extrude(COLLAR_HEIGHT)
    )
    # Add a small tear tab on one side
    tab = (
        cq.Workplane("XY")
        .transformed(offset=(COLLAR_OUTER_R + 0.002, 0, COLLAR_HEIGHT / 2.0))
        .box(0.006, 0.004, COLLAR_HEIGHT * 0.8)
    )
    tab = tab.edges("|Z").fillet(0.001)
    return outer.union(tab)


def _cap_base():
    """Flip-cap base: threaded ring with hinge ears at rear and straw hole."""
    # Main cap ring (sits on neck)
    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, -CAP_SKIRT_DROP))
        .circle(CAP_R)
        .extrude(CAP_HEIGHT)
    )
    outer = outer.edges(">Z").fillet(0.002)
    # Hollow cavity for threading onto neck
    cavity = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, -CAP_SKIRT_DROP - 0.001))
        .circle(NECK_R + 0.001)
        .extrude(CAP_HEIGHT - 0.003)
    )
    cap = outer.cut(cavity)
    # Top disc (solid except for straw hole area)
    top_disc = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, CAP_HEIGHT - CAP_SKIRT_DROP - 0.003))
        .circle(CAP_R - 0.002)
        .extrude(0.003)
    )
    # Straw hole through the disc
    straw_hole = (
        cq.Workplane("XY")
        .transformed(offset=(0.004, 0, -CAP_SKIRT_DROP - 0.002))
        .circle(STRAW_R + 0.001)
        .extrude(CAP_HEIGHT + 0.004)
    )
    cap = cap.union(top_disc).cut(straw_hole)
    # Hinge ears at rear (two ears with pin hole gap between them)
    ear_z_base = CAP_HEIGHT - CAP_SKIRT_DROP - 0.001
    for dy in (-HINGE_EAR_WIDTH / 2.0 - 0.001, HINGE_EAR_WIDTH / 2.0 + 0.001):
        ear = (
            cq.Workplane("XY")
            .transformed(offset=(HINGE_REAR_OFFSET, dy, ear_z_base))
            .box(0.005, HINGE_EAR_WIDTH * 0.4, HINGE_EAR_HEIGHT)
        )
        ear = ear.edges(">Z").fillet(0.001)
        cap = cap.union(ear)
    return cap


def _flip_lid():
    """Flip-up straw cover with dome shape and protruding straw nozzle.
    Local frame: origin at the hinge pin location. Lid body extends forward (+X)
    and upward from the hinge. When q=0 (closed), the dome sits downward (-Z)
    covering the cap opening; when opened, +q rotates the dome upward."""
    # The dome center is offset forward from the hinge by LID_RADIUS - 0.003
    dome_x = LID_RADIUS - 0.003  # ~0.014m forward of hinge
    # Base plate that covers the cap opening (hangs down when closed)
    base_plate = (
        cq.Workplane("XY")
        .transformed(offset=(dome_x, 0, -0.002))
        .circle(LID_RADIUS)
        .extrude(0.003)
    )
    # Dome body
    dome = (
        cq.Workplane("XY")
        .transformed(offset=(dome_x, 0, -0.002 + 0.003))
        .circle(LID_RADIUS - 0.002)
        .extrude(LID_HEIGHT - 0.006)
    )
    dome = dome.edges(">Z").fillet(LID_RADIUS * 0.4)
    lid = base_plate.union(dome)
    # Straw nozzle protruding from dome
    nozzle = (
        cq.Workplane("XY")
        .transformed(offset=(dome_x + 0.005, 0, -0.002 + LID_HEIGHT - 0.003))
        .circle(STRAW_R)
        .extrude(STRAW_HEIGHT)
    )
    nozzle = nozzle.edges(">Z").fillet(0.001)
    lid = lid.union(nozzle)
    # Hinge tab at origin (connects to the hinge ears on cap_base)
    hinge_tab = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, -0.002))
        .box(0.005, HINGE_EAR_WIDTH * 0.35, 0.005)
    )
    lid = lid.union(hinge_tab)
    return lid


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sports_bottle")

    # Materials
    translucent = model.material("translucent_plastic", rgba=(0.82, 0.88, 0.92, 0.35))
    cap_color = model.material("cap_blue", rgba=(0.12, 0.35, 0.65, 1.0))
    collar_color = model.material("collar_teal", rgba=(0.10, 0.55, 0.60, 0.9))
    lid_color = model.material("lid_blue", rgba=(0.12, 0.35, 0.65, 1.0))

    # ---- bottle body (root) ----
    bottle = model.part("bottle")
    shell = _bottle_shell()
    bottle.visual(
        mesh_from_cadquery(shell, "bottle_shell"),
        material=translucent,
        name="bottle_shell",
    )
    bottle.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, NECK_TOP_Z + LIP_HEIGHT),
        mass=0.040,
        origin=Origin(xyz=(0.0, 0.0, (NECK_TOP_Z + LIP_HEIGHT) / 2.0)),
    )

    # ---- safety collar ring ----
    collar = model.part("collar")
    collar_geo = _collar_ring()
    collar.visual(
        mesh_from_cadquery(collar_geo, "collar_ring"),
        material=collar_color,
        name="collar_ring",
    )
    collar.inertial = Inertial.from_geometry(
        Cylinder(COLLAR_OUTER_R, COLLAR_HEIGHT),
        mass=0.003,
        origin=Origin(xyz=(0.0, 0.0, COLLAR_Z + COLLAR_HEIGHT / 2.0)),
    )

    # ---- cap base (carrier for flip hinge) ----
    cap_base = model.part("cap_base")
    cap_geo = _cap_base()
    cap_base.visual(
        mesh_from_cadquery(cap_geo, "cap_base_shell"),
        material=cap_color,
        name="cap_base_shell",
    )
    cap_base.inertial = Inertial.from_geometry(
        Cylinder(CAP_R, CAP_HEIGHT),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, (CAP_HEIGHT - CAP_SKIRT_DROP) / 2.0)),
    )

    # ---- flip lid ----
    flip_lid = model.part("flip_lid")
    lid_geo = _flip_lid()
    flip_lid.visual(
        mesh_from_cadquery(lid_geo, "flip_lid_shell"),
        material=lid_color,
        name="flip_lid_shell",
    )
    flip_lid.inertial = Inertial.from_geometry(
        Cylinder(LID_RADIUS, LID_HEIGHT),
        mass=0.005,
        origin=Origin(xyz=(0.0, 0.0, LID_HEIGHT / 2.0)),
    )

    # ---- Articulations ----

    # 1. collar_spin: collar rotates around neck axis
    model.articulation(
        "collar_spin",
        ArticulationType.CONTINUOUS,
        parent=bottle,
        child=collar,
        origin=Origin(xyz=(0.0, 0.0, COLLAR_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=2.0),
    )

    # 2. cap_rotate: cap base rotates on neck threads
    model.articulation(
        "cap_rotate",
        ArticulationType.CONTINUOUS,
        parent=bottle,
        child=cap_base,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.5),
    )

    # 3. lid_flip: flip lid hinges open at rear of cap
    # In cap_base local frame: hinge ears are at z = CAP_HEIGHT - CAP_SKIRT_DROP - 0.001
    # Ear center at half ear height above that. Hinge at rear x = HINGE_REAR_OFFSET.
    hinge_local_z = (CAP_HEIGHT - CAP_SKIRT_DROP - 0.001) + HINGE_EAR_HEIGHT / 2.0
    model.articulation(
        "lid_flip",
        ArticulationType.REVOLUTE,
        parent=cap_base,
        child=flip_lid,
        origin=Origin(xyz=(HINGE_REAR_OFFSET, 0.0, hinge_local_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=2.3,
            effort=2.0,
            velocity=3.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bottle = object_model.get_part("bottle")
    collar = object_model.get_part("collar")
    cap_base = object_model.get_part("cap_base")
    flip_lid = object_model.get_part("flip_lid")

    collar_spin = object_model.get_articulation("collar_spin")
    cap_rotate = object_model.get_articulation("cap_rotate")
    lid_flip = object_model.get_articulation("lid_flip")

    bottle_shell = bottle.get_visual("bottle_shell")

    # --- Bottle is translucent ---
    ctx.check(
        "bottle material is translucent (alpha < 1)",
        bottle_shell.material.rgba is not None and bottle_shell.material.rgba[3] < 1.0,
        details=f"bottle rgba={bottle_shell.material.rgba}",
    )

    # --- Mouth opening: the bottle has geometry reaching up to lip height ---
    bottle_aabb = ctx.part_world_aabb(bottle)
    ctx.check(
        "bottle has mouth lip extending above neck",
        bottle_aabb is not None and bottle_aabb[1][2] >= NECK_TOP_Z + LIP_HEIGHT * 0.8,
        details=f"bottle top z={bottle_aabb[1][2] if bottle_aabb else None}",
    )

    # --- Collar sits around the neck ---
    collar_pos = ctx.part_world_position(collar)
    ctx.check(
        "collar positioned at neck region",
        collar_pos is not None and SHOULDER_TOP_Z - 0.005 <= collar_pos[2] <= NECK_TOP_Z,
        details=f"collar origin z={collar_pos[2] if collar_pos else None}",
    )

    # --- Cap base sits on top of the neck ---
    cap_pos = ctx.part_world_position(cap_base)
    ctx.check(
        "cap base mounted on neck",
        cap_pos is not None and cap_pos[2] >= NECK_TOP_Z - 0.005,
        details=f"cap origin z={cap_pos[2] if cap_pos else None}",
    )

    # --- Flip lid is connected to cap base via hinge ---
    lid_pos = ctx.part_world_position(flip_lid)
    ctx.check(
        "flip lid positioned at cap region",
        lid_pos is not None and lid_pos[2] >= NECK_TOP_Z - 0.010,
        details=f"flip_lid origin z={lid_pos[2] if lid_pos else None}",
    )

    # --- Collar spin rotates the collar ---
    with ctx.pose({collar_spin: 0.0}):
        collar_aabb_0 = ctx.part_world_aabb(collar)
    with ctx.pose({collar_spin: math.pi / 2.0}):
        collar_aabb_90 = ctx.part_world_aabb(collar)
    ctx.check(
        "collar_spin articulation moves the collar",
        collar_aabb_0 is not None and collar_aabb_90 is not None,
        details="collar AABB not available at both poses",
    )

    # --- Lid flip opens upward ---
    with ctx.pose({lid_flip: 0.0}):
        lid_closed_z = ctx.part_world_aabb(flip_lid)[1][2]  # top of lid when closed
    with ctx.pose({lid_flip: 2.0}):
        lid_open_z = ctx.part_world_aabb(flip_lid)[1][2]  # top of lid when open
    ctx.check(
        "lid_flip opens the lid upward (higher top Z when open)",
        lid_open_z > lid_closed_z - 0.005,
        details=f"closed top z={lid_closed_z}, open top z={lid_open_z}",
    )

    # The lid should move significantly when flipped open
    with ctx.pose({lid_flip: 0.0}):
        lid_aabb_closed = ctx.part_world_aabb(flip_lid)
    with ctx.pose({lid_flip: 1.5}):
        lid_aabb_open = ctx.part_world_aabb(flip_lid)
    # Compare bottom Z of the lid - when closed the dome hangs below hinge;
    # when open the dome swings up above the hinge.
    closed_bottom = lid_aabb_closed[0][2] if lid_aabb_closed else 0.0
    open_bottom = lid_aabb_open[0][2] if lid_aabb_open else 0.0
    closed_top = lid_aabb_closed[1][2] if lid_aabb_closed else 0.0
    open_top = lid_aabb_open[1][2] if lid_aabb_open else 0.0
    ctx.check(
        "lid_flip significantly changes lid bounding box",
        abs(open_top - closed_top) > 0.003 or abs(open_bottom - closed_bottom) > 0.003,
        details=f"closed=[{closed_bottom:.4f},{closed_top:.4f}], open=[{open_bottom:.4f},{open_top:.4f}]",
    )

    # --- Cap skirt overlaps neck (intentional seated overlap) ---
    ctx.allow_overlap(
        cap_base,
        bottle,
        elem_a="cap_base_shell",
        elem_b="bottle_shell",
        reason="Cap skirt is intentionally seated over the threaded neck for screw-on fit.",
    )

    # --- Collar overlaps neck (intentional - wraps around neck) ---
    ctx.allow_overlap(
        collar,
        bottle,
        elem_a="collar_ring",
        elem_b="bottle_shell",
        reason="Safety collar ring intentionally wraps around the neck outer surface.",
    )

    return ctx.report()


object_model = build_object_model()
