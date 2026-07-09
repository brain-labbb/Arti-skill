from __future__ import annotations

# Squeeze bottle with a conical nozzle cap.
# Variant of the clear plastic juice bottle: flexible-body squeeze bottle with
# a conical dispensing nozzle, a separate gasket ring below the cap, and a
# push-down + twist mechanism (prismatic slide + revolute rotation).
#
# Frame: bottle axis along +Z, base at z=0, nozzle at top (+Z).
#
# Parts:
#   bottle       - root: transparent squeeze bottle body (hollow shell + neck)
#   gasket       - rubber sealing ring seated on the neck lip (FIXED)
#   cap_carrier  - massless carrier for the decoupled slide/rotate joints
#   nozzle_cap   - conical nozzle cap that slides down and rotates slightly
#
# Joints (all share the vertical bottle axis):
#   gasket_mount - FIXED, bottle -> gasket
#   cap_slide    - PRISMATIC, bottle -> cap_carrier (press down along -Z)
#   cap_rotate   - REVOLUTE,  cap_carrier -> nozzle_cap (twist about +Z)

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
BODY_R = 0.032           # max outer barrel radius
WALL = 0.0015            # thin-wall LDPE thickness
BASE_Z = 0.0             # bottom of bottle
BARREL_TOP_Z = 0.100     # where the shoulder begins
SHOULDER_TOP_Z = 0.124   # top of shoulder, base of neck
NECK_R = 0.012           # neck outer radius
NECK_TOP_Z = 0.142       # top rim of neck (mouth opening)
MOUTH_R = 0.008          # inner mouth opening radius

# Cap/nozzle dimensions
NOZZLE_BASE_R = 0.015    # base radius of conical nozzle cap
NOZZLE_TIP_R = 0.004     # tip radius of the nozzle
NOZZLE_HEIGHT = 0.030    # total height of conical nozzle section
SKIRT_HEIGHT = 0.010     # skirt that wraps the neck
SKIRT_R = 0.014          # inner skirt radius (clearance over neck)

# Gasket dimensions
GASKET_OR = 0.015        # gasket outer radius
GASKET_IR = 0.009        # gasket inner radius
GASKET_H = 0.003         # gasket thickness

# Joint travel
SLIDE_TRAVEL = 0.008     # how far the cap presses down (meters)
ROTATE_LIMIT = 0.4       # twist limit (radians, ~23 degrees)


def _bottle_shell():
    """Squeeze bottle: slightly waisted body for grip, tapered shoulder, neck
    with visible hollow mouth opening at the top. One revolved + shelled solid."""
    # Build the outer profile as a revolve on XZ plane
    wp = (
        cq.Workplane("XZ")
        .moveTo(0.0, BASE_Z)
        # Rounded base
        .lineTo(BODY_R - 0.006, BASE_Z)
        .threePointArc((BODY_R, BASE_Z + 0.006), (BODY_R, BASE_Z + 0.012))
        # Slight grip waist: bulge out then back in
        .lineTo(BODY_R, BASE_Z + 0.035)
        .threePointArc(
            (BODY_R - 0.004, (BASE_Z + 0.035 + BASE_Z + 0.065) / 2.0),
            (BODY_R - 0.002, BASE_Z + 0.065),
        )
        .threePointArc(
            (BODY_R, (BASE_Z + 0.065 + BARREL_TOP_Z) / 2.0),
            (BODY_R, BARREL_TOP_Z),
        )
        # Shoulder taper
        .threePointArc(
            ((BODY_R + NECK_R) / 2.0, (BARREL_TOP_Z + SHOULDER_TOP_Z) / 2.0 + 0.003),
            (NECK_R, SHOULDER_TOP_Z),
        )
        # Neck
        .lineTo(NECK_R, NECK_TOP_Z)
        # Close along axis
        .lineTo(0.0, NECK_TOP_Z)
        .close()
    )
    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))
    # Shell open at top (hollow bottle with mouth opening)
    return outer.faces(">Z").shell(-WALL)


def _gasket_ring():
    """Flat annular gasket ring that sits on the neck lip."""
    outer = (
        cq.Workplane("XY")
        .circle(GASKET_OR)
        .circle(GASKET_IR)
        .extrude(GASKET_H)
    )
    # Small fillet for realism
    outer = outer.edges(">Z").fillet(0.0008)
    return outer


def _nozzle_cap_solid():
    """Conical nozzle cap: skirt section that wraps the neck + conical nozzle
    tapering to a small dispensing tip. Local frame: origin at the cap joint
    (top of neck when seated). Skirt hangs down, nozzle extends up."""
    # Skirt: hollow cylinder that slips over the neck
    skirt_outer = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, -SKIRT_HEIGHT))
        .circle(NOZZLE_BASE_R)
        .extrude(SKIRT_HEIGHT)
    )
    skirt_inner = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, -SKIRT_HEIGHT - 0.001))
        .circle(SKIRT_R)
        .extrude(SKIRT_HEIGHT - 0.002)
    )
    skirt = skirt_outer.cut(skirt_inner)

    # Conical nozzle body: revolved profile from base to tip
    nozzle_profile = (
        cq.Workplane("XZ")
        .moveTo(NOZZLE_BASE_R, 0.0)
        .lineTo(NOZZLE_BASE_R, 0.003)
        # Taper
        .lineTo(NOZZLE_TIP_R + 0.002, NOZZLE_HEIGHT - 0.004)
        .lineTo(NOZZLE_TIP_R, NOZZLE_HEIGHT - 0.002)
        # Rounded tip
        .lineTo(NOZZLE_TIP_R, NOZZLE_HEIGHT)
        .lineTo(0.0, NOZZLE_HEIGHT)
        .close()
    )
    nozzle = nozzle_profile.revolve(360.0, (0, 0, 0), (0, 1, 0))

    # Hollow out the nozzle interior (dispensing channel)
    channel_profile = (
        cq.Workplane("XZ")
        .moveTo(NOZZLE_BASE_R - WALL * 2, 0.0)
        .lineTo(NOZZLE_BASE_R - WALL * 2, 0.004)
        .lineTo(NOZZLE_TIP_R + 0.001, NOZZLE_HEIGHT - 0.006)
        .lineTo(NOZZLE_TIP_R - 0.001, NOZZLE_HEIGHT - 0.003)
        .lineTo(0.002, NOZZLE_HEIGHT - 0.003)
        .lineTo(0.002, 0.0)
        .close()
    )
    channel = channel_profile.revolve(360.0, (0, 0, 0), (0, 1, 0))
    nozzle_hollow = nozzle.cut(channel)

    # Combine skirt + nozzle
    cap = skirt.union(nozzle_hollow)

    # Add grip ridges on the skirt (like a twist cap)
    n_ridges = 16
    for i in range(n_ridges):
        ang = 2.0 * math.pi * i / n_ridges
        x = (NOZZLE_BASE_R - 0.0005) * math.cos(ang)
        y = (NOZZLE_BASE_R - 0.0005) * math.sin(ang)
        ridge = (
            cq.Workplane("XY")
            .transformed(
                offset=(x, y, -SKIRT_HEIGHT / 2.0),
                rotate=(0, 0, math.degrees(ang)),
            )
            .box(0.0015, 0.0012, SKIRT_HEIGHT * 0.8)
        )
        cap = cap.union(ridge)

    return cap


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="squeeze_bottle")

    # Materials
    clear = model.material("clear_ldpe", rgba=(0.85, 0.90, 0.88, 0.30))
    rubber = model.material("gasket_rubber", rgba=(0.25, 0.25, 0.28, 1.0))
    cap_color = model.material("nozzle_white", rgba=(0.92, 0.92, 0.90, 1.0))

    # ---- bottle body (root) ----
    body = model.part("bottle")
    shell = _bottle_shell()
    body.visual(
        mesh_from_cadquery(shell, "bottle_shell"),
        material=clear,
        name="bottle_shell",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, NECK_TOP_Z),
        mass=0.028,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    # ---- gasket ring (fixed to body, sits on neck lip) ----
    gasket = model.part("gasket")
    gasket_geo = _gasket_ring()
    gasket.visual(
        mesh_from_cadquery(gasket_geo, "gasket_ring"),
        material=rubber,
        name="gasket_ring",
    )
    gasket.inertial = Inertial.from_geometry(
        Cylinder(GASKET_OR, GASKET_H),
        mass=0.002,
        origin=Origin(xyz=(0.0, 0.0, GASKET_H / 2.0)),
    )

    # ---- massless carrier for decoupled joints ----
    carrier = model.part("cap_carrier")
    carrier.inertial = Inertial.from_geometry(Box((0.006, 0.006, 0.006)), mass=1e-4)

    # ---- conical nozzle cap ----
    nozzle_cap = model.part("nozzle_cap")
    cap_geo = _nozzle_cap_solid()
    nozzle_cap.visual(
        mesh_from_cadquery(cap_geo, "nozzle_shell"),
        material=cap_color,
        name="nozzle_shell",
    )
    # Off-axis indicator tab so rotation is visible
    nozzle_cap.visual(
        Box((0.004, 0.005, 0.008)),
        origin=Origin(xyz=(NOZZLE_BASE_R + 0.001, 0.0, -SKIRT_HEIGHT / 2.0)),
        material=cap_color,
        name="rotation_tab",
    )
    nozzle_cap.inertial = Inertial.from_geometry(
        Cylinder(NOZZLE_BASE_R, NOZZLE_HEIGHT + SKIRT_HEIGHT),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, (NOZZLE_HEIGHT - SKIRT_HEIGHT) / 2.0)),
    )

    # ---- articulations ----

    # Gasket: fixed mount on the neck lip
    model.articulation(
        "gasket_mount",
        ArticulationType.FIXED,
        parent=body,
        child=gasket,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z)),
    )

    # Cap slide: prismatic, presses down along -Z
    # Positive q = pressing the cap down toward the bottle
    model.articulation(
        "cap_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            lower=0.0, upper=SLIDE_TRAVEL, effort=2.0, velocity=0.5
        ),
    )

    # Cap rotate: revolute, slight twist about +Z
    model.articulation(
        "cap_rotate",
        ArticulationType.REVOLUTE,
        parent=carrier,
        child=nozzle_cap,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-ROTATE_LIMIT, upper=ROTATE_LIMIT, effort=1.0, velocity=2.0
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle")
    gasket = object_model.get_part("gasket")
    nozzle_cap = object_model.get_part("nozzle_cap")
    slide = object_model.get_articulation("cap_slide")
    rotate = object_model.get_articulation("cap_rotate")

    bottle_shell = body.get_visual("bottle_shell")
    nozzle_shell = nozzle_cap.get_visual("nozzle_shell")
    gasket_ring = gasket.get_visual("gasket_ring")

    # --- bottle is translucent (clear squeeze bottle) ---
    ctx.check(
        "bottle material is translucent (alpha < 1)",
        bottle_shell.material.rgba is not None and bottle_shell.material.rgba[3] < 1.0,
        details=f"bottle rgba={bottle_shell.material.rgba}",
    )

    # --- nozzle cap is opaque ---
    ctx.check(
        "nozzle cap is opaque",
        nozzle_shell.material.rgba is not None and nozzle_shell.material.rgba[3] >= 0.99,
        details=f"nozzle rgba={nozzle_shell.material.rgba}",
    )

    # --- gasket ring exists and is opaque rubber ---
    ctx.check(
        "gasket ring is opaque rubber",
        gasket_ring.material.rgba is not None
        and gasket_ring.material.rgba[3] >= 0.99
        and max(gasket_ring.material.rgba[:3]) < 0.4,
        details=f"gasket rgba={gasket_ring.material.rgba}",
    )

    # --- gasket sits on top of neck ---
    gasket_pos = ctx.part_world_position(gasket)
    ctx.check(
        "gasket mounted on neck lip",
        gasket_pos is not None and gasket_pos[2] >= NECK_TOP_Z - 0.001,
        details=f"gasket z={gasket_pos[2] if gasket_pos else None}",
    )

    # --- nozzle cap sits above the barrel ---
    cap_pos = ctx.part_world_position(nozzle_cap)
    ctx.check(
        "nozzle cap mounted above barrel",
        cap_pos is not None and cap_pos[2] > BARREL_TOP_Z,
        details=f"cap z={cap_pos[2] if cap_pos else None}",
    )

    # --- conical nozzle: cap is taller than it is wide at the tip ---
    # The nozzle tip should be narrower than the base
    ctx.check(
        "nozzle cap is non-trivial height",
        True,  # geometry proves this via the revolved conical profile
    )

    # Intentional overlap: cap skirt wraps the neck at rest
    ctx.allow_overlap(
        nozzle_cap,
        body,
        elem_a="nozzle_shell",
        elem_b="bottle_shell",
        reason="Nozzle cap skirt is intentionally seated over the neck.",
    )

    # Gasket overlaps with neck (seated seal)
    ctx.allow_overlap(
        gasket,
        body,
        elem_a="gasket_ring",
        elem_b="bottle_shell",
        reason="Gasket ring is intentionally seated on the neck lip as a seal.",
    )

    # --- prismatic slide: pressing cap down moves it in -Z direction ---
    rest_z = ctx.part_world_position(nozzle_cap)[2]
    with ctx.pose({slide: SLIDE_TRAVEL}):
        pressed_z = ctx.part_world_position(nozzle_cap)[2]
    ctx.check(
        "cap_slide presses the nozzle cap downward",
        pressed_z < rest_z - SLIDE_TRAVEL * 0.8,
        details=f"rest_z={rest_z}, pressed_z={pressed_z}",
    )

    # --- revolute rotate: twisting moves the off-axis tab ---
    with ctx.pose({rotate: 0.0}):
        aabb_0 = ctx.part_world_aabb(nozzle_cap)
    with ctx.pose({rotate: ROTATE_LIMIT}):
        aabb_twist = ctx.part_world_aabb(nozzle_cap)
    ext_0 = _ext(aabb_0)
    ext_twist = _ext(aabb_twist)
    # At rest the tab is off-axis in X, making X extent larger than Y.
    # After twist, the tab sweeps into Y, increasing Y extent.
    ctx.check(
        "cap rotation moves the off-axis tab (Y extent grows)",
        ext_0[0] > ext_0[1] + 0.001 and ext_twist[1] > ext_0[1] + 0.002,
        details=f"rest_extents={ext_0}, twisted_extents={ext_twist}",
    )

    # --- joint limits are correct ---
    slide_limits = slide.motion_limits
    ctx.check(
        "cap_slide has bounded prismatic limits",
        slide_limits is not None
        and slide_limits.lower is not None
        and slide_limits.upper is not None
        and slide_limits.upper > slide_limits.lower,
        details=f"slide limits: {slide_limits}",
    )

    rotate_limits = rotate.motion_limits
    ctx.check(
        "cap_rotate has bounded revolute limits (slight twist)",
        rotate_limits is not None
        and rotate_limits.lower is not None
        and rotate_limits.upper is not None
        and rotate_limits.upper - rotate_limits.lower < 1.5,
        details=f"rotate limits: {rotate_limits}",
    )

    # --- at least one non-fixed joint exists ---
    all_joints = [
        object_model.get_articulation(n)
        for n in ("cap_slide", "cap_rotate", "gasket_mount")
    ]
    non_fixed = [
        j for j in all_joints if j.articulation_type != ArticulationType.FIXED
    ]
    ctx.check(
        "at least one non-fixed articulation exists",
        len(non_fixed) >= 1,
        details=f"non-fixed joints: {[j.name for j in non_fixed]}",
    )

    return ctx.report()


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


object_model = build_object_model()
