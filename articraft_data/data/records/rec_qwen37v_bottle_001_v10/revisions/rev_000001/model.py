from __future__ import annotations

# Clear plastic juice bottle with a removable measuring cup cap and flip-top lid.
# Frame: bottle axis along +Z, base at z=0, neck/cap at the top (+Z).
# The body is a transparent thin-wall PET shell with molded volume bands
# around the barrel, a shoulder taper, and a threaded neck.
# A tether loop ring is molded onto the neck base.
# The measuring cup cap sits on the neck (removable via prismatic lift).
# A flip lid opens on a revolute hinge at the back of the cup.

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
BODY_R = 0.030          # outer barrel radius (~60mm dia)
WALL = 0.0016           # thin PET wall thickness
BASE_Z = 0.0            # bottom of the bottle
BARREL_TOP_Z = 0.108    # where the shoulder taper begins
SHOULDER_TOP_Z = 0.132  # top of the shoulder, base of the neck
NECK_R = 0.0145         # neck outer radius (under the threads)
NECK_TOP_Z = 0.150      # top rim of the neck

# Volume bands
BAND_R = BODY_R + 0.0012  # bands protrude slightly from the body
BAND_HEIGHT = 0.002       # height of each band ridge
BAND_Z_POSITIONS = [0.035, 0.055, 0.075, 0.095]  # 4 bands along barrel

# Measuring cup cap dimensions
CUP_R = 0.022           # cup outer radius (wider than neck)
CUP_INNER_R = 0.016     # cup inner radius (slips over neck)
CUP_HEIGHT = 0.035      # cup total height
CUP_WALL = 0.002        # cup wall thickness
CUP_BOTTOM_Z = NECK_TOP_Z  # cup sits on top of neck

# Flip lid dimensions
LID_R = CUP_R - 0.001   # lid slightly smaller than cup inner
LID_THICKNESS = 0.003   # lid disc thickness
HINGE_R = 0.003         # hinge pin radius
HINGE_WIDTH = 0.012     # hinge barrel width

# Tether loop dimensions
TETHER_RING_R = 0.005   # tether loop outer radius
TETHER_RING_TUBE = 0.0015  # tether loop tube radius
TETHER_Z = SHOULDER_TOP_Z + 0.003  # just above shoulder on neck


def _neck_thread_profile():
    """Sawtooth ridge segments along the neck for thread detail."""
    pts = [(NECK_R, SHOULDER_TOP_Z)]
    z0 = SHOULDER_TOP_Z + 0.004
    ridge_r = NECK_R + 0.0016
    for k in range(3):
        zc = z0 + k * 0.0048
        pts.append((NECK_R, zc - 0.0016))
        pts.append((ridge_r, zc - 0.0006))
        pts.append((ridge_r, zc + 0.0006))
        pts.append((NECK_R, zc + 0.0016))
    pts.append((NECK_R, NECK_TOP_Z))
    return pts


def _bottle_shell():
    """Transparent thin-wall bottle with volume bands as one revolved solid."""
    # Build the outer profile including volume bands
    wp = (
        cq.Workplane("XZ")
        .moveTo(0.0, BASE_Z)
        # rounded base corner
        .lineTo(BODY_R - 0.006, BASE_Z)
        .threePointArc((BODY_R, BASE_Z + 0.006), (BODY_R, BASE_Z + 0.012))
    )
    # straight barrel with volume band bumps
    prev_z = BASE_Z + 0.012
    for band_z in BAND_Z_POSITIONS:
        # line up to just below band
        wp = wp.lineTo(BODY_R, band_z - BAND_HEIGHT / 2.0)
        # bump out for band
        wp = wp.lineTo(BAND_R, band_z - BAND_HEIGHT / 2.0)
        wp = wp.lineTo(BAND_R, band_z + BAND_HEIGHT / 2.0)
        wp = wp.lineTo(BODY_R, band_z + BAND_HEIGHT / 2.0)
        prev_z = band_z + BAND_HEIGHT / 2.0
    # continue to barrel top
    wp = wp.lineTo(BODY_R, BARREL_TOP_Z)
    # shoulder taper up to the neck
    wp = wp.threePointArc(
        ((BODY_R + NECK_R) / 2.0, (BARREL_TOP_Z + SHOULDER_TOP_Z) / 2.0 + 0.004),
        (NECK_R, SHOULDER_TOP_Z),
    )
    # ridged neck (threads baked into the outline)
    for (r, z) in _neck_thread_profile()[1:]:
        wp = wp.lineTo(r, z)
    # close back along the axis
    wp = wp.lineTo(0.0, NECK_TOP_Z).close()
    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))
    # Hollow it: remove the top neck face and shell inward
    return outer.faces(">Z").shell(-WALL)


def _tether_loop():
    """Small ring loop around the neck base for cap tether attachment."""
    # Torus-like ring at the neck base
    ring = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, TETHER_Z))
        .circle(TETHER_RING_R)
        .circle(TETHER_RING_R - TETHER_RING_TUBE)
        .extrude(TETHER_RING_TUBE * 2)
    )
    # Add a small tab/lug that connects the ring to the neck surface
    lug = (
        cq.Workplane("XY")
        .transformed(offset=(NECK_R + 0.001, 0, TETHER_Z + TETHER_RING_TUBE))
        .box(0.004, 0.006, TETHER_RING_TUBE * 2)
    )
    return ring.union(lug)


def _measuring_cup():
    """Measuring cup cap - hollow cup with measurement lines and hinge lugs."""
    # Outer cup shell
    outer = (
        cq.Workplane("XY")
        .circle(CUP_R)
        .extrude(CUP_HEIGHT)
    )
    # Fillet top edge
    outer = outer.edges(">Z").fillet(0.002)
    # Hollow interior (open at bottom to slip over neck)
    cavity = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, CUP_WALL))
        .circle(CUP_INNER_R)
        .extrude(CUP_HEIGHT - CUP_WALL)
    )
    cup = outer.cut(cavity)
    # Add external measurement line rings (visible on outside of cup)
    for mz in [0.008, 0.016, 0.024]:
        line = (
            cq.Workplane("XY")
            .transformed(offset=(0, 0, mz))
            .circle(CUP_R + 0.0005)
            .circle(CUP_R - 0.0002)
            .extrude(0.0012)
        )
        cup = cup.union(line)
    # Hinge lugs: two small blocks at the back of the cup top
    # that extend outward to hold the hinge pin.
    # Positioned to overlap with the cup wall for connectivity.
    lug_h = 0.006  # lug height
    lug_d = 0.005  # lug depth (radial protrusion from cup)
    lug_w = 0.005  # lug width (along X)
    # Left lug
    left_lug = (
        cq.Workplane("XY")
        .transformed(offset=(-(HINGE_WIDTH / 2.0 + lug_w / 2.0), -(CUP_R - lug_d / 2.0), CUP_HEIGHT - lug_h / 2.0))
        .box(lug_w, lug_d, lug_h)
    )
    cup = cup.union(left_lug)
    # Right lug
    right_lug = (
        cq.Workplane("XY")
        .transformed(offset=((HINGE_WIDTH / 2.0 + lug_w / 2.0), -(CUP_R - lug_d / 2.0), CUP_HEIGHT - lug_h / 2.0))
        .box(lug_w, lug_d, lug_h)
    )
    cup = cup.union(right_lug)
    return cup


def _flip_lid():
    """Flip lid disc with hinge pin geometry.
    
    Local frame: origin at the hinge point (back edge of the lid).
    The disc extends forward (+Y) from the hinge.
    """
    disc_offset_y = LID_R - 0.002
    # Main lid disc (offset from hinge origin toward +Y)
    lid = (
        cq.Workplane("XY")
        .transformed(offset=(0, disc_offset_y, 0))
        .circle(LID_R)
        .extrude(LID_THICKNESS)
    )
    # Connecting bridge from hinge to disc (ensures connectivity)
    bridge = (
        cq.Workplane("XY")
        .transformed(offset=(0, disc_offset_y / 2.0, 0))
        .box(HINGE_WIDTH - 0.002, disc_offset_y + 0.002, LID_THICKNESS)
    )
    lid = lid.union(bridge)
    # Hinge knuckle at the origin (hinge point) - box shape for reliable mesh
    knuckle = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, LID_THICKNESS / 2.0))
        .box(HINGE_WIDTH - 0.002, 0.004, LID_THICKNESS + 0.002)
    )
    lid = lid.union(knuckle)
    # Small grip tab at the front (+Y side) for opening
    tab_y = 2.0 * disc_offset_y + 0.001
    tab = (
        cq.Workplane("XY")
        .transformed(offset=(0, tab_y, LID_THICKNESS / 2.0))
        .box(0.008, 0.005, LID_THICKNESS)
    )
    lid = lid.union(tab)
    return lid


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="measuring_cup_bottle")

    # Materials
    clear = model.material("clear_pet", rgba=(0.80, 0.86, 0.84, 0.25))
    white = model.material("cap_white", rgba=(0.92, 0.92, 0.90, 1.0))
    gray = model.material("lid_gray", rgba=(0.55, 0.55, 0.58, 1.0))

    # ---- bottle body (root) ----
    body = model.part("bottle")
    shell = _bottle_shell()
    body.visual(mesh_from_cadquery(shell, "bottle_shell"), material=clear, name="bottle_shell")
    # Tether loop on neck
    tether = _tether_loop()
    body.visual(mesh_from_cadquery(tether, "tether_loop"), material=white, name="tether_loop")
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, NECK_TOP_Z),
        mass=0.030,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    # ---- measuring cup cap ----
    cap = model.part("cap")
    cup_geo = _measuring_cup()
    cap.visual(mesh_from_cadquery(cup_geo, "cup_shell"), material=white, name="cup_shell")
    cap.inertial = Inertial.from_geometry(
        Cylinder(CUP_R, CUP_HEIGHT),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, CUP_HEIGHT / 2.0)),
    )

    # ---- flip lid ----
    lid = model.part("flip_lid")
    lid_geo = _flip_lid()
    lid.visual(mesh_from_cadquery(lid_geo, "lid_shell"), material=gray, name="lid_shell")
    lid.inertial = Inertial.from_geometry(
        Cylinder(LID_R, LID_THICKNESS),
        mass=0.003,
        origin=Origin(xyz=(0.0, 0.0, LID_THICKNESS / 2.0)),
    )

    # ---- articulations ----
    # Cap lift: prismatic along +Z to remove the cup from the neck
    model.articulation(
        "cap_lift",
        ArticulationType.PRISMATIC,
        parent=body,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.05, effort=2.0, velocity=0.5),
    )

    # Flip hinge: revolute at back of cup top, opens lid upward
    # Hinge at back lugs, axis along X so positive rotation opens lid upward
    hinge_y = -(CUP_R - 0.0025)  # center of hinge lugs
    hinge_z = CUP_HEIGHT  # top of cup
    model.articulation(
        "flip_hinge",
        ArticulationType.REVOLUTE,
        parent=cap,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=2.6, effort=1.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle")
    cap = object_model.get_part("cap")
    lid = object_model.get_part("flip_lid")
    cap_lift = object_model.get_articulation("cap_lift")
    flip_hinge = object_model.get_articulation("flip_hinge")

    bottle_shell = body.get_visual("bottle_shell")
    cup_shell = cap.get_visual("cup_shell")
    lid_shell = lid.get_visual("lid_shell")
    tether_loop = body.get_visual("tether_loop")

    # --- bottle is clear (alpha < 1) ---
    ctx.check(
        "bottle material is tinted-transparent (alpha < 1)",
        bottle_shell.material.rgba is not None and bottle_shell.material.rgba[3] < 1.0,
        details=f"bottle rgba={bottle_shell.material.rgba}",
    )

    # --- cap is opaque white ---
    ctx.check(
        "cap material is opaque white",
        cup_shell.material.rgba is not None
        and cup_shell.material.rgba[3] >= 0.99
        and min(cup_shell.material.rgba[:3]) > 0.8,
        details=f"cap rgba={cup_shell.material.rgba}",
    )

    # --- tether loop exists on the bottle ---
    ctx.check(
        "tether loop exists on bottle body",
        tether_loop is not None,
        details="tether_loop visual not found",
    )

    # --- tether loop is near the neck ---
    tether_pos = ctx.part_element_world_aabb(body, elem="tether_loop")
    ctx.check(
        "tether loop positioned near the neck",
        tether_pos is not None and tether_pos[0][2] > SHOULDER_TOP_Z - 0.005,
        details=f"tether aabb={tether_pos}",
    )

    # --- cap sits on top of the neck at rest ---
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "measuring cup cap mounted on top of the neck",
        cap_pos is not None and cap_pos[2] >= NECK_TOP_Z - 0.001,
        details=f"cap origin={cap_pos}",
    )

    # --- cap lift joint is prismatic ---
    ctx.check(
        "cap_lift is prismatic (removable cap)",
        cap_lift.articulation_type == ArticulationType.PRISMATIC,
        details=f"cap_lift type={cap_lift.articulation_type}",
    )

    # --- cap lifts off the neck ---
    rest_z = ctx.part_world_position(cap)[2]
    with ctx.pose({cap_lift: 0.04}):
        lifted_z = ctx.part_world_position(cap)[2]
    ctx.check(
        "cap_lift removes the cup from the neck",
        lifted_z > rest_z + 0.03,
        details=f"rest_z={rest_z}, lifted_z={lifted_z}",
    )

    # --- flip hinge is revolute ---
    ctx.check(
        "flip_hinge is revolute",
        flip_hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"flip_hinge type={flip_hinge.articulation_type}",
    )

    # --- flip hinge limits are set ---
    hinge_limits = flip_hinge.motion_limits
    ctx.check(
        "flip_hinge has bounded motion limits",
        hinge_limits is not None
        and hinge_limits.lower is not None
        and hinge_limits.upper is not None
        and hinge_limits.upper > hinge_limits.lower,
        details=f"hinge limits={hinge_limits}",
    )

    # --- flip lid opens upward ---
    with ctx.pose({flip_hinge: 0.0}):
        lid_closed_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({flip_hinge: 1.5}):
        lid_open_aabb = ctx.part_world_aabb(lid)
    closed_max_z = lid_closed_aabb[1][2] if lid_closed_aabb else 0
    open_max_z = lid_open_aabb[1][2] if lid_open_aabb else 0
    ctx.check(
        "flip lid opens upward (max Z increases with hinge angle)",
        open_max_z > closed_max_z + 0.005,
        details=f"closed_max_z={closed_max_z}, open_max_z={open_max_z}",
    )

    # --- flip lid stays attached to cap when open ---
    with ctx.pose({flip_hinge: 2.0}):
        lid_open_aabb = ctx.part_world_aabb(lid)
        cap_open_aabb = ctx.part_world_aabb(cap)
    # Check they still overlap in XY when lid is open
    if lid_open_aabb and cap_open_aabb:
        lid_min_x, lid_max_x = lid_open_aabb[0][0], lid_open_aabb[1][0]
        cap_min_x, cap_max_x = cap_open_aabb[0][0], cap_open_aabb[1][0]
        x_overlap = min(lid_max_x, cap_max_x) - max(lid_min_x, cap_min_x)
    else:
        x_overlap = -1
    ctx.check(
        "flip lid remains connected to cap when open",
        x_overlap > -0.01,
        details=f"xy_overlap_x={x_overlap}",
    )

    return ctx.report()


object_model = build_object_model()
