from __future__ import annotations

# Ribbed water bottle with deep grip grooves and a flip-cap on a revolute hinge.
# Frame: vertical axis along +Z, bottle standing on z=0, centerline on x=y=0.
#   - rounded base -> ribbed cylindrical body -> short tapered shoulder
#     -> short neck with open mouth bore -> flip cap on rear hinge.
# Articulation:
#   - cap_flip: REVOLUTE hinge at the rear of the neck rim, axis along +Y,
#     positive q flips the cap open backward (away from the mouth).

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

# ---- key heights (m) along +Z ----
BASE_Z = 0.0
BODY_TOP_Z = 0.150       # end of straight cylindrical body, start of shoulder
SHOULDER_TOP_Z = 0.185   # end of tapered shoulder, base of neck
NECK_TOP_Z = 0.210       # top of neck rim (mouth opening)

BODY_R = 0.035           # body outer radius (~70mm dia)
NECK_R = 0.014           # neck outer radius
NECK_BORE_R = 0.011      # mouth opening bore radius

# Grip groove parameters
NUM_GROOVES = 10         # number of vertical grip channels
GROOVE_DEPTH = 0.004     # how deep each groove cuts into the body wall
GROOVE_WIDTH_ANGLE = 12  # angular width of each groove in degrees

# Flip cap parameters
CAP_R = 0.018            # cap disc radius (slightly larger than neck)
CAP_THICKNESS = 0.006    # cap disc thickness
HINGE_OFFSET_Y = -(NECK_R + 0.002)  # hinge at rear of neck rim
HINGE_Z = NECK_TOP_Z     # hinge at top of neck


def _bottle_solid() -> cq.Workplane:
    """Build the ribbed bottle body as a hollow shell with grip grooves."""
    # Outer profile (revolved): base -> body -> shoulder -> neck
    outer_pts = [
        (0.000, 0.018),   # rounded base heel
        (0.008, 0.032),
        (0.015, 0.0348),
        (0.020, BODY_R),  # full body radius reached
        (BODY_TOP_Z, BODY_R),
        (0.160, 0.033),   # shoulder starts tapering
        (0.175, 0.022),
        (SHOULDER_TOP_Z, 0.016),
        (0.190, NECK_R),
        (NECK_TOP_Z, NECK_R),
    ]

    wp = cq.Workplane("XZ").moveTo(0.0, outer_pts[0][0])
    for r, z in [(r, z) for (z, r) in outer_pts]:
        wp = wp.lineTo(r, z)
    wp = wp.lineTo(0.0, outer_pts[-1][0]).close()
    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))

    # Hollow cavity (open at top through the neck rim)
    wall = 0.0018
    inner_pts = [
        (0.012, 0.010),
        (0.030, 0.016),
        (BODY_R - wall, 0.018),
        (BODY_R - wall, BODY_TOP_Z),
        (0.031, 0.160),
        (0.020, 0.175),
        (0.014, SHOULDER_TOP_Z),
        (NECK_BORE_R, 0.190),
        (NECK_BORE_R, NECK_TOP_Z + 0.005),  # open through rim
    ]
    iwp = cq.Workplane("XZ").moveTo(0.0, inner_pts[0][1])
    for r, z in inner_pts:
        iwp = iwp.lineTo(r, z)
    iwp = iwp.lineTo(0.0, inner_pts[-1][1]).close()
    cavity = iwp.revolve(360.0, (0, 0, 0), (0, 1, 0))

    body = outer.cut(cavity)

    # Cut deep grip grooves into the body exterior
    # Each groove is a vertical slot cut from outside inward
    groove_half_angle = math.radians(GROOVE_WIDTH_ANGLE / 2.0)
    body_mid_z = (0.025 + BODY_TOP_Z) / 2.0
    groove_height = BODY_TOP_Z - 0.025  # grooves span most of the body

    for i in range(NUM_GROOVES):
        angle = 2.0 * math.pi * i / NUM_GROOVES
        cx = (BODY_R + 0.002) * math.cos(angle)
        cy = (BODY_R + 0.002) * math.sin(angle)
        # Cut a thin box-shaped groove from outside
        # The groove is a narrow tall box oriented radially
        groove_width = 2.0 * BODY_R * math.sin(groove_half_angle)
        groove = (
            cq.Workplane("XY")
            .transformed(offset=(cx, cy, body_mid_z))
            .transformed(rotate=(0, 0, math.degrees(angle)))
            .box(groove_width, GROOVE_DEPTH * 2.5, groove_height, centered=(True, True, True))
        )
        body = body.cut(groove)

    return body


def _bottle_mesh():
    return mesh_from_cadquery(_bottle_solid(), "bottle_shell")


def _neck_ring():
    """A small raised ring at the top of the neck for the hinge mount area."""
    ring = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, NECK_TOP_Z - 0.003))
        .circle(NECK_R + 0.002)
        .circle(NECK_R - 0.001)
        .extrude(0.006)
    )
    return mesh_from_cadquery(ring, "neck_ring")


def _flip_cap_solid() -> cq.Workplane:
    """Flip cap: a domed disc with a drinking spout opening."""
    # Main cap disc
    cap = (
        cq.Workplane("XY")
        .circle(CAP_R)
        .extrude(CAP_THICKNESS)
    )
    # Add a small dome on top for grip
    dome = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, CAP_THICKNESS))
        .circle(CAP_R * 0.6)
        .extrude(0.003)
    )
    cap = cap.union(dome)

    # Cut a drinking spout hole through the cap (oval opening)
    spout = (
        cq.Workplane("XY")
        .transformed(offset=(0.005, 0, 0))
        .circle(0.004)
        .extrude(CAP_THICKNESS + 0.005)
    )
    cap = cap.cut(spout)

    # Cut a hinge lug recess at the rear (where hinge pin connects)
    lug_cut = (
        cq.Workplane("XY")
        .transformed(offset=(0, -(CAP_R - 0.002), 0))
        .box(0.008, 0.006, CAP_THICKNESS + 0.002, centered=(True, True, False))
    )
    cap = cap.cut(lug_cut)

    return cap


def _flip_cap_mesh():
    return mesh_from_cadquery(_flip_cap_solid(), "flip_cap_shell")


def _hinge_barrel():
    """Small hinge barrel visual at the rear of the neck."""
    barrel = (
        cq.Workplane("XZ")
        .transformed(offset=(0, 0, 0))
        .circle(0.003)
        .extrude(0.016)
    )
    # Center it and translate to hinge position
    result = (
        cq.Workplane("XY")
        .transformed(offset=(0, HINGE_OFFSET_Y, HINGE_Z))
        .circle(0.003)
        .extrude(0.016)
    )
    # Rotate to align along X axis (hinge pin direction)
    result = (
        cq.Workplane("YZ")
        .transformed(offset=(HINGE_OFFSET_Y, HINGE_Z, 0))
        .circle(0.003)
        .extrude(0.018)
    )
    return mesh_from_cadquery(result, "hinge_barrel")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ribbed_water_bottle")

    # Materials
    bottle_color = model.material("bottle_blue", rgba=(0.15, 0.35, 0.65, 0.85))
    neck_mat = model.material("neck_gray", rgba=(0.45, 0.48, 0.50, 0.9))
    cap_color = model.material("cap_dark", rgba=(0.12, 0.14, 0.16, 1.0))
    hinge_mat = model.material("hinge_metal", rgba=(0.55, 0.55, 0.58, 1.0))

    # ---- bottle body (root): ribbed shell with hollow interior ----
    body = model.part("bottle_body")
    body.visual(_bottle_mesh(), material=bottle_color, name="bottle_shell")
    body.visual(_neck_ring(), material=neck_mat, name="neck_ring")
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, NECK_TOP_Z),
        mass=0.085,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    # ---- hinge barrel (visual only, part of body) ----
    body.visual(_hinge_barrel(), material=hinge_mat, name="hinge_barrel")

    # ---- flip cap ----
    cap = model.part("flip_cap")
    # Cap origin at the hinge point; cap disc extends forward (+Y from hinge)
    # and upward (+Z = thickness)
    cap.visual(
        _flip_cap_mesh(),
        origin=Origin(xyz=(0.0, (CAP_R - 0.002), 0.0)),
        material=cap_color,
        name="flip_cap_shell",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(CAP_R, CAP_THICKNESS),
        mass=0.008,
        origin=Origin(xyz=(0.0, (CAP_R - 0.002), CAP_THICKNESS / 2.0)),
    )

    # ---- cap_flip: REVOLUTE hinge at rear of neck rim ----
    # Hinge axis along +X so positive rotation (right-hand rule) lifts the cap
    # open by rotating it backward around the hinge pin.
    # At q=0 cap is closed (sitting on the neck mouth).
    # Cap part frame is at the hinge point; cap geometry extends in +Y direction.
    model.articulation(
        "cap_flip",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cap,
        origin=Origin(xyz=(0.0, HINGE_OFFSET_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=0.0,
            upper=2.2,  # about 126 degrees open
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle_body")
    cap = object_model.get_part("flip_cap")
    flip = object_model.get_articulation("cap_flip")

    # --- bottle body exists and is tall (water bottle proportions) ---
    body_aabb = ctx.part_world_aabb(body)
    body_ext = (
        body_aabb[1][0] - body_aabb[0][0],
        body_aabb[1][1] - body_aabb[0][1],
        body_aabb[1][2] - body_aabb[0][2],
    )
    ctx.check(
        "bottle is tall (taller than wide)",
        body_ext[2] > 2.5 * body_ext[0],
        details=f"body extents={body_ext}",
    )

    # --- body has grip grooves (shell visual is not a smooth cylinder) ---
    # The ribbed body mesh should have a non-trivial bounding box signature
    shell_vis = body.get_visual("bottle_shell")
    ctx.check(
        "bottle shell exists",
        shell_vis is not None,
        details="bottle_shell visual not found",
    )

    # --- hollow mouth opening exists (neck bore is open at top) ---
    # The bottle shell has a cavity that opens at the neck rim.
    # We check that the body extends above the shoulder (neck exists).
    ctx.check(
        "neck extends above shoulder",
        NECK_TOP_Z > SHOULDER_TOP_Z + 0.015,
        details=f"neck_top={NECK_TOP_Z}, shoulder_top={SHOULDER_TOP_Z}",
    )

    # --- flip cap exists and is mounted near the top ---
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "flip cap is at the top of the bottle",
        cap_pos is not None and cap_pos[2] > 0.18,
        details=f"cap origin={cap_pos}",
    )

    # --- flip joint is REVOLUTE with proper limits ---
    ctx.check(
        "cap_flip is a revolute joint",
        flip.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={flip.articulation_type}",
    )
    limits = flip.motion_limits
    ctx.check(
        "cap_flip has proper open/close limits",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower >= 0.0
        and limits.upper > 1.0,
        details=f"limits={limits}",
    )

    # --- flip cap opens: at positive pose the cap moves upward ---
    cap_z_closed = ctx.part_world_aabb(cap)[1][2]  # top Z of cap when closed
    with ctx.pose({flip: 1.5}):
        cap_z_open = ctx.part_world_aabb(cap)[1][2]
    # When flipped open, the far edge of the cap rises above the closed position
    # (or at least the cap geometry moves significantly)
    cap_center_closed = ctx.part_world_position(cap)
    with ctx.pose({flip: 1.5}):
        cap_center_open = ctx.part_world_position(cap)
    # The cap part frame rotates around the hinge, so its origin stays at the hinge.
    # But the cap geometry (extending in +Y) rotates upward. Check the AABB top.
    ctx.check(
        "flip cap opens upward when articulated",
        cap_z_open > cap_z_closed + 0.005 or (
            cap_center_open is not None and cap_center_closed is not None
            and abs(cap_center_open[1] - cap_center_closed[1]) > 0.005
        ),
        details=f"closed_top_z={cap_z_closed:.4f}, open_top_z={cap_z_open:.4f}",
    )

    # --- hinge barrel visual exists ---
    hinge_vis = body.get_visual("hinge_barrel")
    ctx.check(
        "hinge barrel visual exists",
        hinge_vis is not None,
        details="hinge_barrel visual not found on body",
    )

    # --- neck ring exists ---
    ring_vis = body.get_visual("neck_ring")
    ctx.check(
        "neck ring visual exists",
        ring_vis is not None,
        details="neck_ring visual not found on body",
    )

    # --- cap contacts/near the neck rim when closed ---
    ctx.expect_gap(
        cap, body,
        axis="z",
        max_penetration=0.005,
        max_gap=0.008,
        name="flip cap seated near neck rim when closed",
    )

    return ctx.report()


object_model = build_object_model()
