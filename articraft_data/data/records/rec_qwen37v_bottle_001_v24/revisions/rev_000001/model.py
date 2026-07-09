from __future__ import annotations

# Pump-top soap/lotion bottle with a flip cap on a revolute hinge.
# Frame: bottle axis along +Z, base at z=0, pump collar and flip cap at top (+Z).
# The body is a semi-translucent HDPE shell with molded volume bands
# (horizontal ridges around the barrel), a shoulder taper, and a short neck.
# A pump collar threads onto the neck (CONTINUOUS rotation about +Z).
# A flip cap sits on the collar and opens backward on a revolute hinge
# (axis along X at the rear edge of the collar).

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
BODY_R = 0.033          # outer barrel radius (~66 mm dia, lotion bottle)
WALL = 0.0018           # HDPE wall thickness
BASE_Z = 0.0
BARREL_TOP_Z = 0.118    # where the shoulder taper begins
SHOULDER_TOP_Z = 0.146  # top of the shoulder, base of the neck
NECK_R = 0.013          # neck outer radius
NECK_TOP_Z = 0.160      # top rim of the neck

# Volume band parameters
BAND_PROTRUSION = 0.0022
BAND_HALF_H = 0.003
BAND_Z_CENTERS = [0.038, 0.065, 0.092]

# Pump collar
COLLAR_R = 0.017
COLLAR_H = 0.014
COLLAR_BASE_Z = NECK_TOP_Z

# Flip cap
FLIP_CAP_R = 0.014
FLIP_CAP_H = 0.005
HINGE_Y_OFFSET = -(COLLAR_R - 0.003)
HINGE_Z_LOCAL = COLLAR_H


def _outer_profile_pts():
    """Outer profile points for the bottle body (with volume bands)."""
    pts = [(0.0, BASE_Z)]
    # Rounded base corner
    pts.append((BODY_R - 0.005, BASE_Z))
    pts.append((BODY_R, BASE_Z + 0.006))

    # Barrel with volume bands
    z_cursor = BASE_Z + 0.006
    for zc in sorted(BAND_Z_CENTERS):
        z_bot = zc - BAND_HALF_H
        z_top = zc + BAND_HALF_H
        if z_bot > z_cursor + 0.001:
            pts.append((BODY_R, z_bot))
        # Smooth band bump
        pts.append((BODY_R + BAND_PROTRUSION * 0.5, z_bot + BAND_HALF_H * 0.3))
        pts.append((BODY_R + BAND_PROTRUSION, zc))
        pts.append((BODY_R + BAND_PROTRUSION * 0.5, z_top - BAND_HALF_H * 0.3))
        z_cursor = z_top

    pts.append((BODY_R, BARREL_TOP_Z))
    # Shoulder taper
    mid_r = (BODY_R + NECK_R) / 2.0
    mid_z = (BARREL_TOP_Z + SHOULDER_TOP_Z) / 2.0 + 0.003
    pts.append((mid_r, mid_z))
    pts.append((NECK_R, SHOULDER_TOP_Z))
    # Neck
    pts.append((NECK_R, NECK_TOP_Z))
    pts.append((0.0, NECK_TOP_Z))
    return pts


def _inner_profile_pts():
    """Inner cavity profile (offset inward by WALL, simpler shape without bands)."""
    IR = BODY_R - WALL
    INR = NECK_R - WALL
    pts = [(0.0, BASE_Z + WALL)]
    pts.append((IR - 0.004, BASE_Z + WALL))
    pts.append((IR, BASE_Z + 0.006 + WALL))
    # Straight inner barrel (no bands - inner wall is smooth)
    pts.append((IR, BARREL_TOP_Z))
    # Shoulder
    mid_r = (IR + INR) / 2.0
    mid_z = (BARREL_TOP_Z + SHOULDER_TOP_Z) / 2.0 + 0.003
    pts.append((mid_r, mid_z))
    pts.append((INR, SHOULDER_TOP_Z))
    pts.append((INR, NECK_TOP_Z + 0.001))  # extend past top to ensure open
    pts.append((0.0, NECK_TOP_Z + 0.001))
    return pts


def _bottle_shell():
    """Hollow HDPE bottle body: outer revolve minus inner cavity revolve."""
    # Outer solid
    outer_pts = _outer_profile_pts()
    wp = cq.Workplane("XZ").moveTo(outer_pts[0][0], outer_pts[0][1])
    for r, z in outer_pts[1:]:
        wp = wp.lineTo(r, z)
    wp = wp.close()
    outer_solid = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))

    # Inner cavity
    inner_pts = _inner_profile_pts()
    wp2 = cq.Workplane("XZ").moveTo(inner_pts[0][0], inner_pts[0][1])
    for r, z in inner_pts[1:]:
        wp2 = wp2.lineTo(r, z)
    wp2 = wp2.close()
    inner_solid = wp2.revolve(360.0, (0, 0, 0), (0, 1, 0))

    # Subtract inner from outer to get hollow shell
    shell = outer_solid.cut(inner_solid)
    return shell


def _collar_solid():
    """Pump collar ring with dispensing boss and hinge lug.
    Local frame: origin at collar joint (= NECK_TOP_Z in world).
    The skirt extends BELOW the origin to wrap around the neck."""
    SKIRT = 0.010  # how far the skirt hangs below the origin
    total_h = SKIRT + COLLAR_H  # total collar height
    # Outer cylinder from z=-SKIRT to z=COLLAR_H
    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, -SKIRT))
        .circle(COLLAR_R)
        .extrude(total_h)
    )
    outer = outer.edges(">Z").fillet(0.002)
    # Hollow cavity to slip over neck (open at bottom).
    # Cavity radius slightly less than neck radius so the collar contacts
    # the neck outer wall (realistic press-fit / threaded engagement).
    cavity = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, -SKIRT - 0.001))
        .circle(NECK_R - 0.0003)
        .extrude(total_h - 0.003)
    )
    collar = outer.cut(cavity)
    # Dispensing orifice boss on top
    boss = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0.004, COLLAR_H))
        .circle(0.004)
        .extrude(0.003)
    )
    collar = collar.union(boss)
    # Hinge lug at rear top
    lug = (
        cq.Workplane("XY")
        .transformed(offset=(0, HINGE_Y_OFFSET, COLLAR_H - 0.001))
        .box(0.008, 0.006, 0.004)
    )
    collar = collar.union(lug)
    return collar


def _flip_cap_solid():
    """Flip cap disc with thumb tab and hinge knuckle.
    Local frame: origin at hinge pin. Cap extends along +Y when closed."""
    # Main disc centered at (0, FLIP_CAP_R, FLIP_CAP_H*0.5)
    disc = (
        cq.Workplane("XY")
        .transformed(offset=(0, FLIP_CAP_R, 0.0))
        .circle(FLIP_CAP_R)
        .extrude(FLIP_CAP_H)
    )
    disc = disc.edges(">Z").fillet(0.0012)
    # Thumb tab at front edge
    tab = (
        cq.Workplane("XY")
        .transformed(offset=(0, FLIP_CAP_R * 2.0 - 0.003, 0.001))
        .box(0.007, 0.004, FLIP_CAP_H - 0.002)
    )
    cap = disc.union(tab)
    # Hinge knuckle cylinder at origin (along X axis)
    knuckle = (
        cq.Workplane("YZ")
        .circle(0.002)
        .extrude(0.006)
        .translate((-0.003, 0, FLIP_CAP_H * 0.5))
    )
    cap = cap.union(knuckle)
    return cap


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pump_top_bottle")

    # Materials
    hdpe = model.material("hdpe_body", rgba=(0.92, 0.90, 0.86, 0.82))
    collar_mat = model.material("collar_white", rgba=(0.90, 0.88, 0.85, 1.0))
    cap_mat = model.material("flip_cap_blue", rgba=(0.20, 0.40, 0.62, 1.0))

    # ---- bottle body (root) ----
    bottle = model.part("bottle")
    shell = _bottle_shell()
    bottle.visual(mesh_from_cadquery(shell, "bottle_shell"), material=hdpe, name="bottle_shell")
    bottle.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, NECK_TOP_Z),
        mass=0.040,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    # ---- pump collar ----
    collar = model.part("pump_collar")
    collar_geo = _collar_solid()
    collar.visual(
        mesh_from_cadquery(collar_geo, "collar_shell"),
        material=collar_mat,
        name="collar_shell",
    )
    collar.inertial = Inertial.from_geometry(
        Cylinder(COLLAR_R, COLLAR_H + 0.010),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, (COLLAR_H - 0.010) / 2.0)),
    )

    # ---- flip cap ----
    flip = model.part("flip_cap")
    flip_geo = _flip_cap_solid()
    flip.visual(
        mesh_from_cadquery(flip_geo, "flip_cap_shell"),
        material=cap_mat,
        name="flip_cap_shell",
    )
    flip.inertial = Inertial.from_geometry(
        Cylinder(FLIP_CAP_R, FLIP_CAP_H),
        mass=0.004,
        origin=Origin(xyz=(0.0, FLIP_CAP_R, FLIP_CAP_H / 2.0)),
    )

    # ---- articulations ----

    # collar_rotate: CONTINUOUS spin of the collar on the bottle neck
    model.articulation(
        "collar_rotate",
        ArticulationType.CONTINUOUS,
        parent=bottle,
        child=collar,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=1.5),
    )

    # cap_flip: REVOLUTE hinge at rear of collar top
    # Cap extends along +Y from hinge. axis=(1,0,0): positive rotation lifts
    # the +Y edge upward (opening the cap).
    model.articulation(
        "cap_flip",
        ArticulationType.REVOLUTE,
        parent=collar,
        child=flip,
        origin=Origin(xyz=(0.0, HINGE_Y_OFFSET, COLLAR_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=2.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bottle = object_model.get_part("bottle")
    collar = object_model.get_part("pump_collar")
    flip = object_model.get_part("flip_cap")
    collar_rotate = object_model.get_articulation("collar_rotate")
    cap_flip = object_model.get_articulation("cap_flip")

    bottle_shell = bottle.get_visual("bottle_shell")
    flip_cap_shell = flip.get_visual("flip_cap_shell")

    # --- bottle shell exists ---
    ctx.check(
        "bottle shell exists as mesh-backed visual",
        bottle_shell is not None and hasattr(bottle_shell.geometry, "filename"),
    )

    # --- volume bands widen the body beyond plain barrel radius ---
    bottle_aabb = ctx.part_world_aabb(bottle)
    if bottle_aabb is not None:
        mn, mx = bottle_aabb
        body_width_x = mx[0] - mn[0]
        body_width_y = mx[1] - mn[1]
        expected_min = 2.0 * (BODY_R + BAND_PROTRUSION * 0.5)
        ctx.check(
            "molded volume bands widen body beyond plain barrel",
            body_width_x >= expected_min and body_width_y >= expected_min,
            details=f"width_x={body_width_x:.4f} width_y={body_width_y:.4f} expected_min={expected_min:.4f}",
        )

    # --- pump collar on top of neck ---
    collar_pos = ctx.part_world_position(collar)
    ctx.check(
        "pump collar mounted at top of neck",
        collar_pos is not None and collar_pos[2] >= NECK_TOP_Z - 0.001,
        details=f"collar_pos={collar_pos}",
    )

    # Collar seated over neck: intentional overlap
    ctx.allow_overlap(
        collar,
        bottle,
        elem_a="collar_shell",
        elem_b="bottle_shell",
        reason="Pump collar cavity is intentionally seated over the bottle neck.",
    )
    # Proof: collar overlaps bottle footprint in XY and is seated in Z
    ctx.expect_overlap(
        collar,
        bottle,
        axes="xy",
        min_overlap=0.005,
        elem_a="collar_shell",
        elem_b="bottle_shell",
        name="collar centered on bottle neck in XY",
    )
    ctx.expect_overlap(
        collar,
        bottle,
        axes="z",
        min_overlap=0.005,
        elem_a="collar_shell",
        elem_b="bottle_shell",
        name="collar skirt engaged with neck in Z",
    )

    # --- flip cap at top of collar ---
    flip_pos = ctx.part_world_position(flip)
    ctx.check(
        "flip cap mounted above collar",
        flip_pos is not None and flip_pos[2] >= COLLAR_BASE_Z + COLLAR_H - 0.005,
        details=f"flip_pos={flip_pos}",
    )

    # --- cap_flip is REVOLUTE with bounded limits ---
    ctx.check(
        "cap_flip is revolute with bounded motion",
        cap_flip.articulation_type == ArticulationType.REVOLUTE
        and cap_flip.motion_limits is not None
        and cap_flip.motion_limits.lower is not None
        and cap_flip.motion_limits.upper is not None
        and cap_flip.motion_limits.upper > cap_flip.motion_limits.lower,
    )

    # --- flip cap opens upward ---
    closed_max_z = None
    open_max_z = None
    with ctx.pose({cap_flip: 0.0}):
        closed_aabb = ctx.part_world_aabb(flip)
        if closed_aabb:
            closed_max_z = closed_aabb[1][2]
    with ctx.pose({cap_flip: 1.8}):
        open_aabb = ctx.part_world_aabb(flip)
        if open_aabb:
            open_max_z = open_aabb[1][2]

    ctx.check(
        "flip cap opens upward (max Z rises with positive angle)",
        closed_max_z is not None and open_max_z is not None and open_max_z > closed_max_z + 0.005,
        details=f"closed_max_z={closed_max_z}, open_max_z={open_max_z}",
    )

    # --- closed cap overlaps collar in XY ---
    with ctx.pose({cap_flip: 0.0}):
        ctx.expect_overlap(
            flip,
            collar,
            axes="xy",
            min_overlap=0.005,
            name="closed flip cap overlaps collar footprint",
        )

    # --- collar rotation is CONTINUOUS ---
    ctx.check(
        "collar_rotate is continuous",
        collar_rotate.articulation_type == ArticulationType.CONTINUOUS,
    )

    # --- materials ---
    ctx.check(
        "bottle body is semi-translucent (alpha < 1)",
        bottle_shell.material.rgba is not None and bottle_shell.material.rgba[3] < 1.0,
        details=f"bottle rgba={bottle_shell.material.rgba}",
    )
    ctx.check(
        "flip cap is opaque",
        flip_cap_shell.material.rgba is not None and flip_cap_shell.material.rgba[3] >= 0.99,
        details=f"flip cap rgba={flip_cap_shell.material.rgba}",
    )

    return ctx.report()


object_model = build_object_model()
