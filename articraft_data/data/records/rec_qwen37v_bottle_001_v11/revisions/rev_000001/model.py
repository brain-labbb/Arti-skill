from __future__ import annotations

# Tall cylindrical pump bottle variant (forked from clear juice bottle).
# Frame: bottle axis along +Z, base at z=0, neck/mouth/pump at the top (+Z).
# The body is a tall transparent thin-wall cylinder: flat base + long barrel +
# shoulder taper + narrow neck with a visible mouth lip.
# The pump head assembly rides on the neck through a massless carrier link:
#   - pump_slide:   PRISMATIC press-down along -Z (pump depress motion)
#   - pump_rotate:  REVOLUTE slight twist about +Z (nozzle aim)
# A nozzle spout on the pump head makes the rotation visible.

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
WALL = 0.0018           # thin PET wall thickness
BASE_Z = 0.0            # bottom of the bottle
BARREL_TOP_Z = 0.150    # where the shoulder taper begins
SHOULDER_TOP_Z = 0.168  # top of the shoulder, base of the neck
NECK_R = 0.011          # neck outer radius (narrow)
NECK_TOP_Z = 0.190      # top rim of the neck
LIP_HEIGHT = 0.004      # mouth lip ring height above neck top
LIP_OUTER_R = 0.0135    # outer radius of the mouth lip
LIP_INNER_R = NECK_R - WALL  # inner radius of the lip (hollow mouth)

# Pump collar dimensions
COLLAR_R = 0.016        # pump collar outer radius
COLLAR_H = 0.012        # pump collar height
COLLAR_INNER_R = NECK_R + 0.001  # collar slips over the neck

# Pump head dimensions
HEAD_R = 0.018          # pump head disc radius
HEAD_H = 0.006          # pump head disc height
NOZZLE_LENGTH = 0.025   # nozzle spout length
NOZZLE_R = 0.004        # nozzle tube radius

# Slide limits: pump can be pressed down by this much
SLIDE_RANGE = 0.014


def _bottle_shell():
    """Tall cylindrical bottle with narrow neck, hollow, one revolved solid.
    The mouth lip is a flared ring at the top of the neck, modeled as part of
    the same revolve profile. The shell operation hollows the interior."""
    # Build the outer profile in XZ plane, revolve around Z axis
    pts = []
    # Flat base with slight corner round
    pts.append((0.0, BASE_Z))
    pts.append((BODY_R - 0.005, BASE_Z))
    # Base corner arc approximation (just line segments for simplicity)
    pts.append((BODY_R, BASE_Z + 0.005))
    # Straight tall barrel
    pts.append((BODY_R, BARREL_TOP_Z))
    # Shoulder taper to narrow neck
    mid_r = (BODY_R + NECK_R) / 2.0
    mid_z = (BARREL_TOP_Z + SHOULDER_TOP_Z) / 2.0 + 0.003
    pts.append((mid_r, mid_z))
    pts.append((NECK_R, SHOULDER_TOP_Z))
    # Straight narrow neck
    pts.append((NECK_R, NECK_TOP_Z))
    # Mouth lip: flare out then up then back in to form a visible rim
    pts.append((LIP_OUTER_R, NECK_TOP_Z))
    pts.append((LIP_OUTER_R, NECK_TOP_Z + LIP_HEIGHT))
    pts.append((NECK_R, NECK_TOP_Z + LIP_HEIGHT))
    # Close at axis
    pts.append((0.0, NECK_TOP_Z + LIP_HEIGHT))

    # Build as a revolve from a wire
    wp = cq.Workplane("XZ")
    wp = wp.moveTo(pts[0][0], pts[0][1])
    for (r, z) in pts[1:]:
        wp = wp.lineTo(r, z)
    wp = wp.close()
    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))

    # Shell: remove the top face and hollow inward to show wall thickness
    # The hollow mouth is visible through the transparent walls
    return outer.faces(">Z").shell(-WALL)


def _pump_collar():
    """Pump collar that wraps the neck. Local frame: origin at the pump_slide
    joint (top of neck). The collar hangs down from the origin."""
    # Annular ring: outer cylinder minus inner bore
    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, -COLLAR_H))
        .circle(COLLAR_R)
        .extrude(COLLAR_H)
    )
    bore = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, -COLLAR_H - 0.001))
        .circle(COLLAR_INNER_R)
        .extrude(COLLAR_H + 0.002)
    )
    return outer.cut(bore)


def _pump_head():
    """Pump head disc with nozzle spout. Local frame: origin at the pump_rotate
    joint. The disc sits above and the nozzle extends radially."""
    # Main disc
    disc = (
        cq.Workplane("XY")
        .circle(HEAD_R)
        .extrude(HEAD_H)
    )
    # Fillet top edges
    disc = disc.edges(">Z").fillet(0.0015)

    # Nozzle spout: horizontal tube extending from the disc edge
    nozzle = (
        cq.Workplane("XZ")
        .transformed(offset=(HEAD_R - 0.002, 0, HEAD_H / 2.0))
        .circle(NOZZLE_R)
        .extrude(NOZZLE_LENGTH)
    )
    # Hollow out the nozzle tip (open end)
    nozzle_bore = (
        cq.Workplane("XZ")
        .transformed(offset=(HEAD_R + NOZZLE_LENGTH - 0.004, 0, HEAD_H / 2.0))
        .circle(NOZZLE_R - 0.0015)
        .extrude(0.005)
    )
    head = disc.union(nozzle).cut(nozzle_bore)

    # Small stem underneath connecting to collar bore
    stem = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, -0.008))
        .circle(COLLAR_INNER_R - 0.001)
        .extrude(0.008)
    )
    head = head.union(stem)
    return head


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pump_bottle")

    # Transparent clear body, white pump head, dark grey collar
    clear = model.material("clear_body", rgba=(0.82, 0.88, 0.86, 0.22))
    white = model.material("pump_white", rgba=(0.92, 0.92, 0.90, 1.0))
    grey = model.material("collar_grey", rgba=(0.35, 0.35, 0.38, 1.0))

    # ---- bottle body (root) ----
    body = model.part("bottle")
    shell = _bottle_shell()
    body.visual(mesh_from_cadquery(shell, "bottle_shell"), material=clear, name="bottle_shell")
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, NECK_TOP_Z + LIP_HEIGHT),
        mass=0.040,
        origin=Origin(xyz=(0.0, 0.0, (NECK_TOP_Z + LIP_HEIGHT) / 2.0)),
    )

    # ---- massless carrier: carries the slide joint ----
    carrier = model.part("pump_carrier")
    collar_geo = _pump_collar()
    carrier.visual(
        mesh_from_cadquery(collar_geo, "pump_collar"),
        material=grey,
        name="pump_collar",
    )
    carrier.inertial = Inertial.from_geometry(
        Cylinder(COLLAR_R, COLLAR_H),
        mass=0.004,
        origin=Origin(xyz=(0.0, 0.0, -COLLAR_H / 2.0)),
    )

    # ---- pump head with nozzle ----
    head = model.part("pump_head")
    head_geo = _pump_head()
    head.visual(
        mesh_from_cadquery(head_geo, "pump_head_shell"),
        material=white,
        name="pump_head_shell",
    )
    head.inertial = Inertial.from_geometry(
        Cylinder(HEAD_R, HEAD_H + 0.008),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, HEAD_H / 2.0)),
    )

    # ---- joints ----
    # pump_slide: body -> carrier, prismatic along -Z (press down)
    # At q=0, pump is at rest (up). Positive q presses pump downward.
    model.articulation(
        "pump_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z + LIP_HEIGHT)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=0.5,
            lower=0.0, upper=SLIDE_RANGE,
        ),
    )

    # pump_rotate: carrier -> head, revolute about +Z (nozzle aim)
    model.articulation(
        "pump_rotate",
        ArticulationType.REVOLUTE,
        parent=carrier,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0, velocity=2.0,
            lower=-0.6, upper=0.6,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle")
    carrier = object_model.get_part("pump_carrier")
    head = object_model.get_part("pump_head")
    slide = object_model.get_articulation("pump_slide")
    rotate = object_model.get_articulation("pump_rotate")

    bottle_shell = body.get_visual("bottle_shell")
    head_shell = head.get_visual("pump_head_shell")

    # --- bottle body is transparent ---
    ctx.check(
        "bottle body is transparent (alpha < 1)",
        bottle_shell.material.rgba is not None and bottle_shell.material.rgba[3] < 1.0,
        details=f"bottle rgba={bottle_shell.material.rgba}",
    )

    # --- bottle is tall: height > 0.17m ---
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "bottle is tall (height > 0.17m)",
        body_aabb is not None and (body_aabb[1][2] - body_aabb[0][2]) > 0.17,
        details=f"body aabb={body_aabb}",
    )

    # --- narrow neck: neck radius < body radius ---
    # The bottle shell includes the neck profile; we verify dimensions are correct
    ctx.check(
        "neck is narrower than body (NECK_R < BODY_R)",
        NECK_R < BODY_R * 0.6,
        details=f"NECK_R={NECK_R}, BODY_R={BODY_R}",
    )

    # --- mouth lip exists: bottle top extends above NECK_TOP_Z ---
    ctx.check(
        "mouth lip extends above neck top",
        body_aabb is not None and body_aabb[1][2] > NECK_TOP_Z + LIP_HEIGHT * 0.5,
        details=f"body top z={body_aabb[1][2] if body_aabb else None}",
    )

    # --- pump head is mounted above the bottle body ---
    head_pos = ctx.part_world_position(head)
    ctx.check(
        "pump head is mounted on top of the neck",
        head_pos is not None and head_pos[2] > NECK_TOP_Z,
        details=f"head origin={head_pos}",
    )

    # --- collar wraps the neck: intentional seated overlap ---
    ctx.allow_overlap(
        carrier,
        body,
        elem_a="pump_collar",
        elem_b="bottle_shell",
        reason="Pump collar is intentionally seated over the narrow neck.",
    )

    # --- pump head stem passes through collar bore: captured shaft fit ---
    ctx.allow_overlap(
        head,
        carrier,
        elem_a="pump_head_shell",
        elem_b="pump_collar",
        reason="Pump head stem is intentionally captured inside the collar bore.",
    )
    # Prove the stem is centered: head and carrier origins are close in XY
    ctx.expect_origin_distance(
        head, carrier,
        axes="xy",
        min_dist=0.0,
        max_dist=0.003,
        name="pump head stem stays centered on the collar axis",
    )

    # --- pump head stem dips into the hollow mouth/neck: intentional insertion ---
    ctx.allow_overlap(
        head,
        body,
        elem_a="pump_head_shell",
        elem_b="bottle_shell",
        reason="Pump head stem intentionally extends into the hollow neck mouth opening.",
    )
    # Prove the overlap is small and local: head overlaps the neck Z range
    ctx.expect_overlap(
        head, body,
        axes="z",
        elem_a="pump_head_shell",
        elem_b="bottle_shell",
        min_overlap=0.002,
        name="pump head stem reaches into the hollow neck",
    )

    # --- pump_slide presses the pump downward ---
    rest_z = ctx.part_world_position(head)[2]
    with ctx.pose({slide: SLIDE_RANGE}):
        pressed_z = ctx.part_world_position(head)[2]
    ctx.check(
        "pump_slide presses the head downward",
        pressed_z < rest_z - SLIDE_RANGE * 0.8,
        details=f"rest_z={rest_z}, pressed_z={pressed_z}",
    )

    # --- pump_slide is prismatic with proper limits ---
    ctx.check(
        "pump_slide is prismatic with non-zero range",
        slide.articulation_type == ArticulationType.PRISMATIC
        and slide.motion_limits is not None
        and slide.motion_limits.upper is not None
        and slide.motion_limits.upper > 0.0,
        details=f"type={slide.articulation_type}, limits={slide.motion_limits}",
    )

    # --- pump_rotate rotates the nozzle (asymmetric extents change) ---
    with ctx.pose({rotate: 0.0}):
        aabb0 = ctx.part_world_aabb(head)
    with ctx.pose({rotate: math.pi / 4.0}):
        aabb45 = ctx.part_world_aabb(head)
    ext0 = (aabb0[1][0] - aabb0[0][0], aabb0[1][1] - aabb0[0][1])
    ext45 = (aabb45[1][0] - aabb45[0][0], aabb45[1][1] - aabb45[0][1])
    ctx.check(
        "pump_rotate moves the nozzle (XY extents change on rotation)",
        abs(ext0[0] - ext45[0]) > 0.002 or abs(ext0[1] - ext45[1]) > 0.002,
        details=f"rest xy_ext=({ext0[0]:.4f},{ext0[1]:.4f}), 45deg xy_ext=({ext45[0]:.4f},{ext45[1]:.4f})",
    )

    # --- pump_rotate is revolute with bounded limits ---
    ctx.check(
        "pump_rotate is revolute with bounded limits",
        rotate.articulation_type == ArticulationType.REVOLUTE
        and rotate.motion_limits is not None
        and rotate.motion_limits.lower is not None
        and rotate.motion_limits.upper is not None,
        details=f"type={rotate.articulation_type}, limits={rotate.motion_limits}",
    )

    return ctx.report()


object_model = build_object_model()
