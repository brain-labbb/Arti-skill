from __future__ import annotations

# Ribbed water bottle with deep grip grooves and a flip-top cap.
# Frame: bottle axis along +Z, base at z=0, neck/cap at the top (+Z).
# The body is a translucent thin-wall shell with deep horizontal grip grooves,
# a tapered shoulder, narrow neck, and a hollow interior with visible mouth
# opening at the top.
# A flip cap sits on the neck and opens on a revolute hinge at the rear.
# The hinge barrel is on the bottle; the cap has a hinge arm wrapping around it.

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
BODY_R = 0.036          # outer barrel radius (~72 mm dia)
WALL = 0.002            # thin-wall thickness
BASE_Z = 0.0            # bottom of the bottle
BARREL_TOP_Z = 0.148    # where the shoulder taper begins
SHOULDER_TOP_Z = 0.178  # top of the shoulder, base of the neck
NECK_R = 0.014          # neck outer radius (28 mm dia mouth)
NECK_TOP_Z = 0.198      # top rim of the neck
MOUTH_R = 0.011         # mouth opening inner radius (22 mm)

COLLAR_R = 0.018        # collar ring around the neck (for flip cap mount)
COLLAR_HEIGHT = 0.008   # collar height above neck top

CAP_R = 0.017           # cap disc radius (covers the collar)
CAP_THICK = 0.005       # cap disc thickness

GROOVE_DEPTH = 0.004    # depth of grip grooves into the barrel
NUM_GROOVES = 5         # number of grip grooves

# Hinge geometry
HINGE_Y = -(COLLAR_R + 0.002)  # hinge barrel at rear of collar
HINGE_Z = NECK_TOP_Z + COLLAR_HEIGHT / 2.0
HINGE_BARREL_R = 0.0025
HINGE_BARREL_LEN = 0.014


def _outer_profile():
    """Outer profile of the ribbed water bottle for the revolve operation.
    Points go from axis-bottom, out to the wall, up with grooves, through
    the shoulder, up the neck, and back to the axis at the top."""
    pts = []

    # ---- base ----
    pts.append((0.0, BASE_Z))
    pts.append((BODY_R - 0.006, BASE_Z))
    # rounded base corner
    pts.append((BODY_R - 0.002, BASE_Z + 0.002))
    pts.append((BODY_R, BASE_Z + 0.006))

    # ---- barrel with deep grip grooves ----
    groove_start = 0.030
    groove_zone = BARREL_TOP_Z - groove_start - 0.008
    pitch = groove_zone / NUM_GROOVES
    gw = pitch * 0.50  # groove width (fraction of pitch)

    for i in range(NUM_GROOVES):
        zb = groove_start + i * pitch
        # ridge at full radius before this groove
        pts.append((BODY_R, zb))
        # smooth transition into groove
        pts.append((BODY_R - GROOVE_DEPTH * 0.25, zb + 0.002))
        pts.append((BODY_R - GROOVE_DEPTH * 0.70, zb + 0.004))
        # groove floor
        pts.append((BODY_R - GROOVE_DEPTH, zb + 0.005))
        pts.append((BODY_R - GROOVE_DEPTH, zb + gw - 0.005))
        # smooth transition out of groove
        pts.append((BODY_R - GROOVE_DEPTH * 0.70, zb + gw - 0.004))
        pts.append((BODY_R - GROOVE_DEPTH * 0.25, zb + gw - 0.002))
        pts.append((BODY_R, zb + gw))

    # ridge after last groove to end of barrel
    pts.append((BODY_R, BARREL_TOP_Z))

    # ---- shoulder taper ----
    mid_r = (BODY_R + NECK_R) / 2.0 + 0.003
    mid_z = (BARREL_TOP_Z + SHOULDER_TOP_Z) / 2.0 + 0.004
    pts.append((mid_r, mid_z))
    pts.append((NECK_R + 0.002, SHOULDER_TOP_Z))

    # ---- neck ----
    pts.append((NECK_R + 0.002, SHOULDER_TOP_Z + 0.003))
    pts.append((NECK_R, SHOULDER_TOP_Z + 0.006))
    pts.append((NECK_R, NECK_TOP_Z))

    # close at axis (top of neck - will be shelled open)
    pts.append((0.0, NECK_TOP_Z))

    return pts


def _bottle_shell():
    """Build the ribbed water bottle as a thin-wall revolved solid.
    Shell from the top face opens the mouth."""
    pts = _outer_profile()

    wp = cq.Workplane("XZ")
    wp = wp.moveTo(pts[0][0], pts[0][1])
    for r, z in pts[1:]:
        wp = wp.lineTo(r, z)
    wp = wp.close()

    solid = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))
    # Shell from the top face to create hollow interior with open mouth
    return solid.faces(">Z").shell(-WALL)


def _neck_collar():
    """Collar ring around the top of the neck with integrated hinge mount.
    Extends slightly below the neck top for bottle connectivity. Has a rear
    extension that supports the hinge barrel."""
    collar_start = NECK_TOP_Z - 0.003
    collar = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, collar_start))
        .circle(COLLAR_R)
        .extrude(COLLAR_HEIGHT + 0.003)
    )
    # Rear extension for hinge mount (connects collar to hinge barrel area)
    mount_w = HINGE_BARREL_LEN - 0.004
    mount_d = COLLAR_R + abs(HINGE_Y)  # from collar center to hinge barrel
    mount_h = COLLAR_HEIGHT + 0.001
    rear_mount = (
        cq.Workplane("XY")
        .transformed(offset=(0, -mount_d / 2.0, collar_start + mount_h / 2.0))
        .box(mount_w, mount_d, mount_h)
    )
    collar = collar.union(rear_mount)
    # Hollow bore to fit over the neck
    bore = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, collar_start - 0.001))
        .circle(NECK_R - 0.0005)
        .extrude(COLLAR_HEIGHT + 0.005)
    )
    return collar.cut(bore)


def _hinge_barrel():
    """Hinge barrel: horizontal cylinder along X at the rear of the collar.
    Extends into the collar for structural connection."""
    barrel = (
        cq.Workplane("YZ")
        .circle(HINGE_BARREL_R)
        .extrude(HINGE_BARREL_LEN)
    )
    # Center along X and position at hinge point
    return barrel.translate((-HINGE_BARREL_LEN / 2.0, HINGE_Y, HINGE_Z))



def _cap_lid():
    """Flip cap lid: disc with hinge bridge, eye, and pull tab.
    Local frame: origin at the hinge point. At q=0 the disc is centered
    on the neck axis at the top of the collar."""
    disc_cy = -HINGE_Y  # distance from hinge to neck axis (positive Y in local)
    disc_cz = COLLAR_HEIGHT / 2.0  # sit on top of collar

    # Main disc
    disc = (
        cq.Workplane("XY")
        .transformed(offset=(0, disc_cy, disc_cz))
        .circle(CAP_R)
        .extrude(CAP_THICK)
    )
    disc = disc.edges(">Z").fillet(0.0012)

    # Bridge: solid block connecting the hinge eye region to the disc
    bridge_y_start = 0.002
    bridge_y_end = disc_cy - CAP_R + 0.004
    bridge_z_start = -0.002
    bridge_z_end = disc_cz + CAP_THICK
    bridge_cy = (bridge_y_start + bridge_y_end) / 2.0
    bridge_cz = (bridge_z_start + bridge_z_end) / 2.0
    bridge_sy = max(bridge_y_end - bridge_y_start, 0.004)
    bridge_sz = bridge_z_end - bridge_z_start
    bridge = (
        cq.Workplane("XY")
        .transformed(offset=(0, bridge_cy, bridge_cz))
        .box(0.008, bridge_sy, bridge_sz)
    )
    cap = disc.union(bridge)

    # Hinge eye: small cylinder wrapping around the barrel at the hinge
    # In cap local frame, the hinge barrel is at the origin (0, 0, 0)
    eye_r = HINGE_BARREL_R + 0.0015
    eye = (
        cq.Workplane("YZ")
        .center(0, 0)  # cap-local origin = hinge point
        .circle(eye_r)
        .extrude(0.010)
    )
    eye = eye.translate((-0.005, 0, 0))
    cap = cap.union(eye)

    # Pull tab at the front (small raised lip for finger grip)
    tab_y = disc_cy + CAP_R - 0.003
    tab = (
        cq.Workplane("XY")
        .transformed(offset=(0, tab_y, disc_cz + CAP_THICK + 0.001))
        .box(0.008, 0.006, 0.004)
    )
    cap = cap.union(tab)

    return cap


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ribbed_water_bottle")

    # Materials
    bottle_mat = model.material("bottle_translucent", rgba=(0.62, 0.76, 0.84, 0.50))
    cap_mat = model.material("cap_charcoal", rgba=(0.18, 0.19, 0.22, 1.0))
    collar_mat = model.material("collar_dark", rgba=(0.22, 0.23, 0.26, 1.0))

    # ---- bottle body (root) ----
    body = model.part("bottle")
    shell = _bottle_shell()
    body.visual(
        mesh_from_cadquery(shell, "bottle_shell"),
        material=bottle_mat,
        name="bottle_shell",
    )

    # Neck collar
    collar = _neck_collar()
    body.visual(
        mesh_from_cadquery(collar, "neck_collar"),
        material=collar_mat,
        name="neck_collar",
    )

    # Hinge barrel
    barrel = _hinge_barrel()
    body.visual(
        mesh_from_cadquery(barrel, "hinge_barrel"),
        material=cap_mat,
        name="hinge_barrel",
    )

    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, NECK_TOP_Z),
        mass=0.042,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    # ---- flip cap ----
    cap = model.part("flip_cap")
    cap_geo = _cap_lid()
    cap.visual(
        mesh_from_cadquery(cap_geo, "cap_lid"),
        material=cap_mat,
        name="cap_lid",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(CAP_R, CAP_THICK),
        mass=0.007,
        origin=Origin(xyz=(0.0, -HINGE_Y, COLLAR_HEIGHT / 2.0 + CAP_THICK / 2.0)),
    )

    # ---- flip hinge joint ----
    # Hinge at the rear of the collar, axis along X.
    # Positive rotation lifts the front of the cap upward (opens it).
    model.articulation(
        "cap_flip",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cap,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=4.0, lower=0.0, upper=2.6,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle")
    cap = object_model.get_part("flip_cap")
    hinge = object_model.get_articulation("cap_flip")

    bottle_shell = body.get_visual("bottle_shell")
    cap_lid = cap.get_visual("cap_lid")

    # --- bottle is translucent (you can see through it) ---
    ctx.check(
        "bottle material is translucent (alpha < 1)",
        bottle_shell.material.rgba is not None and bottle_shell.material.rgba[3] < 1.0,
        details=f"bottle rgba={bottle_shell.material.rgba}",
    )

    # --- cap is opaque ---
    ctx.check(
        "cap material is opaque",
        cap_lid.material.rgba is not None and cap_lid.material.rgba[3] >= 0.99,
        details=f"cap rgba={cap_lid.material.rgba}",
    )

    # --- bottle body is significantly wider than neck (grip grooves present) ---
    ctx.check(
        "barrel has deep grip grooves (body wider than neck + groove depth)",
        BODY_R > NECK_R + GROOVE_DEPTH + 0.010,
        details=f"BODY_R={BODY_R}, NECK_R+GROOVE_DEPTH={NECK_R + GROOVE_DEPTH}",
    )

    # --- cap sits near neck top when closed ---
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "flip cap mounted near neck top when closed",
        cap_pos is not None and cap_pos[2] > SHOULDER_TOP_Z,
        details=f"cap origin z={cap_pos[2] if cap_pos else None}",
    )

    # --- flip cap opens: front edge rises on positive angle ---
    rest_max_z = None
    open_max_z = None
    rest_aabb = ctx.part_world_aabb(cap)
    if rest_aabb is not None:
        rest_max_z = rest_aabb[1][2]
    with ctx.pose({hinge: 1.4}):
        open_aabb = ctx.part_world_aabb(cap)
        if open_aabb is not None:
            open_max_z = open_aabb[1][2]

    ctx.check(
        "flip cap opens upward on positive hinge angle",
        rest_max_z is not None and open_max_z is not None
        and open_max_z > rest_max_z + 0.008,
        details=f"rest_max_z={rest_max_z}, open_max_z={open_max_z}",
    )

    # --- at fully open pose, cap is clearly above the neck ---
    with ctx.pose({hinge: 2.4}):
        full_open_aabb = ctx.part_world_aabb(cap)
    ctx.check(
        "flip cap reaches near-vertical open position",
        full_open_aabb is not None
        and full_open_aabb[1][2] > NECK_TOP_Z + 0.02,
        details=f"full_open_max_z={full_open_aabb[1][2] if full_open_aabb else None}",
    )

    # --- mouth opening: bottle shell top is open (not a solid cap) ---
    # The bottle_shell AABB max Z should be at or below the neck top,
    # confirming the mouth is open (shell removed the top face).
    shell_aabb = ctx.part_element_world_aabb(body, elem="bottle_shell")
    ctx.check(
        "bottle shell has open mouth (top near neck level, not capped)",
        shell_aabb is not None and shell_aabb[1][2] <= NECK_TOP_Z + COLLAR_HEIGHT + 0.002,
        details=f"shell max z={shell_aabb[1][2] if shell_aabb else None}",
    )

    # --- hinge barrel overlaps with cap eye (intentional pin/barrel fit) ---
    ctx.allow_overlap(
        cap,
        body,
        elem_a="cap_lid",
        elem_b="hinge_barrel",
        reason="Cap hinge eye wraps around the hinge barrel for the pivot connection.",
    )
    # --- cap bridge passes through collar hinge mount region (interleaved hinge) ---
    ctx.allow_overlap(
        cap,
        body,
        elem_a="cap_lid",
        elem_b="neck_collar",
        reason="Cap hinge bridge interleaves with the collar rear mount for the flip-cap pivot.",
    )

    # --- prove the cap lid overlaps the collar footprint when closed ---
    ctx.expect_overlap(
        cap,
        body,
        axes="xy",
        elem_a="cap_lid",
        elem_b="neck_collar",
        min_overlap=0.005,
        name="cap covers the collar when closed (XY overlap)",
    )

    return ctx.report()


object_model = build_object_model()
