from __future__ import annotations

# Medicine bottle with child-resistant swing-top stopper.
# Frame: bottle axis along +Z, base at z=0, neck/stopper at the top (+Z).
# Amber translucent hollow bottle body with:
#   - molded volume bands (raised rings) around the barrel
#   - pivot lugs on opposite sides of the neck
#   - tether loop ring attached to the neck
# White swing-top stopper with hinge arms, grip ridges, and push-tab.
# REVOLUTE joint: stopper swings open on the pivot axis through the lugs.

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
BODY_R = 0.025           # outer barrel radius (~50 mm dia)
WALL = 0.002             # PET wall thickness
BASE_Z = 0.0
BARREL_TOP_Z = 0.082     # where shoulder taper begins
SHOULDER_TOP_Z = 0.098   # top of shoulder / base of neck
NECK_R = 0.014           # neck outer radius
NECK_TOP_Z = 0.115       # top rim of neck

# Volume bands
BAND_ZS = [0.025, 0.045, 0.065]
BAND_RAISE = 0.0012
BAND_WIDTH = 0.003

# Stopper
STOPPER_R = 0.013
STOPPER_H = 0.008

# Pivot: axis along Y through both lugs, centered at (0, 0, PIVOT_Z)
PIVOT_Z = NECK_TOP_Z - 0.008   # 0.107
LUG_Y_OFF = 0.018              # lug center Y offset from axis

# Tether loop position
TETHER_Z = SHOULDER_TOP_Z + 0.008  # 0.106


# ---------------------------------------------------------------------------
# Bottle shell (hollow amber body)
# ---------------------------------------------------------------------------

def _bottle_shell():
    """Revolves the outer profile and shells the interior hollow."""
    wp = (
        cq.Workplane("XZ")
        .moveTo(0.0, BASE_Z)
        .lineTo(BODY_R - 0.005, BASE_Z)
        .lineTo(BODY_R - 0.002, BASE_Z + 0.001)
        .lineTo(BODY_R, BASE_Z + 0.005)
        .lineTo(BODY_R, BARREL_TOP_Z)
        .threePointArc(
            ((BODY_R + NECK_R) / 2.0, (BARREL_TOP_Z + SHOULDER_TOP_Z) / 2.0 + 0.003),
            (NECK_R, SHOULDER_TOP_Z),
        )
        .lineTo(NECK_R, NECK_TOP_Z)
        .lineTo(0.0, NECK_TOP_Z)
        .close()
    )
    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))
    return outer.faces(">Z").shell(-WALL)


# ---------------------------------------------------------------------------
# Volume bands (raised rings embedded into the barrel wall)
# ---------------------------------------------------------------------------

def _volume_band(z_height):
    """Annular ring that protrudes outward from the barrel surface."""
    return (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, z_height - BAND_WIDTH / 2))
        .circle(BODY_R + BAND_RAISE)
        .circle(BODY_R - 0.001)
        .extrude(BAND_WIDTH)
    )


# ---------------------------------------------------------------------------
# Pivot lugs (two bosses on ±Y sides of the neck)
# ---------------------------------------------------------------------------

def _pivot_lugs():
    """Two rectangular lugs embedded in the neck wall for the hinge arms."""
    lug1 = (
        cq.Workplane("XY")
        .transformed(offset=(0, LUG_Y_OFF, PIVOT_Z))
        .box(0.008, 0.012, 0.010)
    )
    lug2 = (
        cq.Workplane("XY")
        .transformed(offset=(0, -LUG_Y_OFF, PIVOT_Z))
        .box(0.008, 0.012, 0.010)
    )
    return lug1.union(lug2)


# ---------------------------------------------------------------------------
# Tether loop (D-ring tab + washer ring attached to the neck)
# ---------------------------------------------------------------------------

def _tether_loop():
    """Small cap tether loop on the +X side of the neck."""
    # Tab embedded in the neck wall
    tab = (
        cq.Workplane("XY")
        .transformed(offset=(NECK_R + 0.002, 0, TETHER_Z))
        .box(0.008, 0.006, 0.006)
    )
    # Vertical connector bar from tab down to ring
    connector = (
        cq.Workplane("XY")
        .transformed(offset=(NECK_R + 0.004, 0, TETHER_Z - 0.006))
        .box(0.004, 0.004, 0.010)
    )
    # Washer-shaped ring
    ring_outer = (
        cq.Workplane("XY")
        .transformed(offset=(NECK_R + 0.006, 0, TETHER_Z - 0.013))
        .circle(0.005)
        .extrude(0.006)
    )
    ring_hole = (
        cq.Workplane("XY")
        .transformed(offset=(NECK_R + 0.006, 0, TETHER_Z - 0.014))
        .circle(0.003)
        .extrude(0.008)
    )
    ring = ring_outer.cut(ring_hole)
    return tab.union(connector).union(ring)


# ---------------------------------------------------------------------------
# Swing-top stopper (disc + plug + arms + grip ridges + push tab)
# ---------------------------------------------------------------------------

def _stopper_solid():
    """
    Stopper in local frame (coincident with articulation frame at q=0).
    Articulation frame is at (0, 0, PIVOT_Z) in parent.
    Disc sits on top of the neck when closed (q=0).
    """
    # Disc center in local frame
    disc_dz = NECK_TOP_Z - PIVOT_Z  # 0.008
    disc = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, disc_dz))
        .circle(STOPPER_R)
        .extrude(STOPPER_H)
    )
    disc = disc.edges(">Z").fillet(0.002)

    # Plug that inserts into the neck bore
    plug = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, disc_dz - 0.005))
        .circle(STOPPER_R - 0.002)
        .extrude(0.005)
    )

    # Hinge arms: two flat bars on ±X sides connecting disc to pivot region
    arm_half_w = 0.002
    arm_y = 0.004
    arm_z_center = disc_dz / 2.0 + 0.002  # ~0.006
    arm_height = disc_dz + STOPPER_H + 0.002  # spans below pivot to above disc
    arm1 = (
        cq.Workplane("XY")
        .transformed(offset=(LUG_Y_OFF * 0.72, 0, arm_z_center))
        .box(arm_y, arm_half_w * 2, arm_height)
    )
    arm2 = (
        cq.Workplane("XY")
        .transformed(offset=(-LUG_Y_OFF * 0.72, 0, arm_z_center))
        .box(arm_y, arm_half_w * 2, arm_height)
    )

    stopper = disc.union(plug).union(arm1).union(arm2)

    # Child-resistant grip ridges on top of disc
    for i in range(8):
        ang = 2.0 * math.pi * i / 8
        gx = (STOPPER_R - 0.004) * math.cos(ang)
        gy = (STOPPER_R - 0.004) * math.sin(ang)
        ridge = (
            cq.Workplane("XY")
            .transformed(offset=(gx, gy, disc_dz + STOPPER_H + 0.0005))
            .box(0.003, 0.002, 0.0015)
        )
        stopper = stopper.union(ridge)

    # Push-tab on one side (child-resistant release lever)
    tab = (
        cq.Workplane("XY")
        .transformed(offset=(0, -STOPPER_R - 0.001, disc_dz + STOPPER_H * 0.5))
        .box(0.008, 0.006, 0.005)
    )
    stopper = stopper.union(tab)

    return stopper


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="medicine_bottle")

    amber = model.material("amber_plastic", rgba=(0.72, 0.45, 0.15, 0.55))
    white = model.material("white_plastic", rgba=(0.92, 0.92, 0.90, 1.0))

    # ---- bottle body (root) ----
    body = model.part("bottle")
    shell = _bottle_shell()
    body.visual(
        mesh_from_cadquery(shell, "bottle_shell"),
        material=amber,
        name="bottle_shell",
    )

    # Volume bands
    for idx, bz in enumerate(BAND_ZS):
        band = _volume_band(bz)
        body.visual(
            mesh_from_cadquery(band, f"volume_band_{idx}"),
            material=amber,
            name=f"volume_band_{idx}",
        )

    # Pivot lugs
    lugs = _pivot_lugs()
    body.visual(
        mesh_from_cadquery(lugs, "pivot_lugs"),
        material=amber,
        name="pivot_lugs",
    )

    # Tether loop
    tether = _tether_loop()
    body.visual(
        mesh_from_cadquery(tether, "tether_loop"),
        material=amber,
        name="tether_loop",
    )

    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, NECK_TOP_Z),
        mass=0.040,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    # ---- swing-top stopper ----
    stopper = model.part("stopper")
    stopper_geo = _stopper_solid()
    stopper.visual(
        mesh_from_cadquery(stopper_geo, "stopper_shell"),
        material=white,
        name="stopper_shell",
    )
    stopper.inertial = Inertial.from_geometry(
        Cylinder(STOPPER_R, STOPPER_H + 0.01),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, (NECK_TOP_Z - PIVOT_Z) / 2.0)),
    )

    # ---- swing-top hinge (REVOLUTE) ----
    # Pivot axis along Y through both lugs at (0, 0, PIVOT_Z).
    # Axis (0, -1, 0): positive q swings the stopper upward and open.
    model.articulation(
        "stopper_swing",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stopper,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=2.1, effort=2.0, velocity=2.0),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle")
    stopper = object_model.get_part("stopper")
    hinge = object_model.get_articulation("stopper_swing")

    bottle_shell = body.get_visual("bottle_shell")
    stopper_shell = stopper.get_visual("stopper_shell")

    # --- materials ---
    ctx.check(
        "bottle is amber translucent",
        bottle_shell.material.rgba is not None and bottle_shell.material.rgba[3] < 1.0,
        details=f"bottle rgba={bottle_shell.material.rgba}",
    )
    ctx.check(
        "stopper is opaque white",
        stopper_shell.material.rgba is not None
        and stopper_shell.material.rgba[3] >= 0.99
        and min(stopper_shell.material.rgba[:3]) > 0.8,
        details=f"stopper rgba={stopper_shell.material.rgba}",
    )

    # --- joint is revolute with meaningful range ---
    ctx.check(
        "stopper_swing is revolute",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={hinge.articulation_type}",
    )
    limits = hinge.motion_limits
    ctx.check(
        "hinge has non-trivial angular range",
        limits is not None
        and limits.upper is not None
        and limits.upper > 1.0
        and limits.lower is not None
        and limits.lower >= 0.0,
        details=f"lower={limits.lower if limits else None}, upper={limits.upper if limits else None}",
    )

    # --- stopper opens when hinge rotates ---
    # The part origin sits at the pivot center and never moves; use AABB instead.
    # When the stopper swings open, the disc drops to the side, so the AABB
    # min_z decreases significantly and the Z extent grows.
    with ctx.pose({hinge: 0.0}):
        rest_aabb = ctx.part_world_aabb(stopper)
    with ctx.pose({hinge: 2.0}):
        open_aabb = ctx.part_world_aabb(stopper)
    ctx.check(
        "stopper swings open (AABB min_z drops as disc swings away from neck)",
        rest_aabb is not None
        and open_aabb is not None
        and rest_aabb[0][2] - open_aabb[0][2] > 0.008,
        details=f"rest_min_z={rest_aabb[0][2] if rest_aabb else None}, open_min_z={open_aabb[0][2] if open_aabb else None}",
    )

    # --- named features exist ---
    ctx.check(
        "tether loop visual exists",
        body.get_visual("tether_loop") is not None,
    )
    ctx.check(
        "pivot lugs visual exists",
        body.get_visual("pivot_lugs") is not None,
    )
    for idx in range(len(BAND_ZS)):
        ctx.check(
            f"volume band {idx} exists",
            body.get_visual(f"volume_band_{idx}") is not None,
        )

    # --- stopper seated on neck at rest (plug inside bore, disc on rim) ---
    ctx.allow_overlap(
        stopper,
        body,
        elem_a="stopper_shell",
        elem_b="bottle_shell",
        reason="Stopper plug seats inside the neck bore and hinge arms pass alongside the neck wall to reach pivot lugs.",
    )
    ctx.allow_overlap(
        stopper,
        body,
        elem_a="stopper_shell",
        elem_b="pivot_lugs",
        reason="Hinge arms pivot within the lug sockets on the neck.",
    )

    # Prove the stopper is supported: disc overlaps the neck in XY at rest
    ctx.expect_overlap(
        stopper,
        body,
        axes="xy",
        min_overlap=0.005,
        name="stopper disc overlaps the neck footprint at rest",
    )

    # Prove the stopper X center shifts (the disc swings to one side)
    rest_center_x = (rest_aabb[0][0] + rest_aabb[1][0]) / 2.0
    open_center_x = (open_aabb[0][0] + open_aabb[1][0]) / 2.0
    ctx.check(
        "stopper X center shifts when swung open",
        abs(rest_center_x - open_center_x) > 0.004,
        details=f"rest_center_x={rest_center_x:.5f}, open_center_x={open_center_x:.5f}",
    )

    return ctx.report()


object_model = build_object_model()
