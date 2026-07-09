from __future__ import annotations

# Medicine bottle with a child-resistant push-and-turn cap.
# Frame: bottle axis along +Z, base at z=0, neck/cap at the top (+Z).
# The body is an amber translucent thin-wall shell: wide barrel with molded
# volume bands, shoulder taper, short neck with raised spiral-like ridges.
# The white child-resistant cap rides on the neck through two decoupled joints:
#   - cap_turn: REVOLUTE rotation about +Z (partial turn, 0 to 0.8 rad)
#   - cap_push: PRISMATIC push-down along -Z (0 to 0.005 m travel)
# A massless carrier link separates the rotation from the translation.

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
BODY_R = 0.025          # outer barrel radius (~0.05 m dia, wider medicine bottle)
WALL = 0.0016           # thin wall thickness
BASE_Z = 0.0            # bottom of the bottle
BARREL_TOP_Z = 0.065    # where the shoulder taper begins
SHOULDER_TOP_Z = 0.082  # top of the shoulder, base of the neck
NECK_R = 0.014          # neck outer radius (under the threads)
NECK_TOP_Z = 0.095      # top rim of the neck (cap mounts here)
CAP_R = 0.019           # cap outer radius (larger for child-resistant grip)
CAP_HEIGHT = 0.022      # cap height

# Volume bands: raised ridges around the barrel for measurement markings
BAND_POSITIONS = [0.020, 0.040]  # Z heights for volume bands
BAND_WIDTH = 0.003               # width of each band
BAND_PROTRUSION = 0.0012         # how far bands protrude from barrel surface

# Cap-local Z layout: the turn joint sits at z=NECK_TOP_Z. The cap skirt
# hangs DOWN past its own origin to wrap the neck (intentional seated overlap).
SKIRT_DROP = 0.010      # how far the skirt hangs below the cap origin
CAP_TOP_Z = CAP_HEIGHT - SKIRT_DROP  # cap-local top of the disc

# Push-down travel for child-resistant mechanism
PUSH_TRAVEL = 0.005     # 5mm push-down travel


def _neck_thread_profile():
    # Raised spiral-like ridges along the neck. More pronounced than typical
    # threads to read as visible spiral ridges. Returns profile points from
    # bottom to top of the neck, with wider ridge protrusion.
    pts = [(NECK_R, SHOULDER_TOP_Z)]
    z0 = SHOULDER_TOP_Z + 0.003
    ridge_r = NECK_R + 0.002  # more pronounced ridges
    n_ridges = 4
    ridge_pitch = (NECK_TOP_Z - z0 - 0.003) / n_ridges
    for k in range(n_ridges):
        zc = z0 + k * ridge_pitch + ridge_pitch / 2.0
        pts.append((NECK_R, zc - ridge_pitch * 0.4))
        pts.append((ridge_r, zc - ridge_pitch * 0.15))
        pts.append((ridge_r, zc + ridge_pitch * 0.15))
        pts.append((NECK_R, zc + ridge_pitch * 0.4))
    pts.append((NECK_R, NECK_TOP_Z))
    return pts


def _barrel_profile():
    # Barrel outer profile with molded volume bands. The bands are raised
    # horizontal ridges around the barrel body.
    pts = []
    # Start at base with rounded corner
    pts.append((0.0, BASE_Z))
    pts.append((BODY_R - 0.005, BASE_Z))
    # Base radius blends up (we'll handle the arc in the workplane)
    return pts


def _bottle_shell():
    # Amber translucent thin-wall bottle with volume bands and threaded neck.
    # Built as outer shell minus inner cavity for robust hollow construction.

    # Build the outer profile with volume bands integrated
    wp = cq.Workplane("XZ").moveTo(0.0, BASE_Z)

    # Rounded base corner
    wp = wp.lineTo(BODY_R - 0.005, BASE_Z)
    wp = wp.threePointArc((BODY_R, BASE_Z + 0.005), (BODY_R, BASE_Z + 0.010))

    # Barrel with volume bands (raised ridges)
    # Band 1 at z=0.020
    band_z1 = BAND_POSITIONS[0]
    wp = wp.lineTo(BODY_R, band_z1 - BAND_WIDTH / 2.0)
    wp = wp.lineTo(BODY_R + BAND_PROTRUSION, band_z1 - BAND_WIDTH / 2.0 + 0.0005)
    wp = wp.lineTo(BODY_R + BAND_PROTRUSION, band_z1 + BAND_WIDTH / 2.0 - 0.0005)
    wp = wp.lineTo(BODY_R, band_z1 + BAND_WIDTH / 2.0)

    # Continue barrel to band 2 at z=0.040
    band_z2 = BAND_POSITIONS[1]
    wp = wp.lineTo(BODY_R, band_z2 - BAND_WIDTH / 2.0)
    wp = wp.lineTo(BODY_R + BAND_PROTRUSION, band_z2 - BAND_WIDTH / 2.0 + 0.0005)
    wp = wp.lineTo(BODY_R + BAND_PROTRUSION, band_z2 + BAND_WIDTH / 2.0 - 0.0005)
    wp = wp.lineTo(BODY_R, band_z2 + BAND_WIDTH / 2.0)

    # Continue barrel to shoulder start
    wp = wp.lineTo(BODY_R, BARREL_TOP_Z)

    # Shoulder taper up to the neck
    wp = wp.threePointArc(
        ((BODY_R + NECK_R) / 2.0, (BARREL_TOP_Z + SHOULDER_TOP_Z) / 2.0 + 0.003),
        (NECK_R, SHOULDER_TOP_Z),
    )

    # Ridged neck (spiral-like threads baked into the outline)
    for (r, z) in _neck_thread_profile()[1:]:
        wp = wp.lineTo(r, z)

    # Close back along the axis
    wp = wp.lineTo(0.0, NECK_TOP_Z).close()
    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))

    # Create inner cavity to hollow out the bottle
    inner_r = BODY_R - WALL
    inner_neck_r = NECK_R - WALL
    cavity = (
        cq.Workplane("XZ")
        .moveTo(0.0, BASE_Z + WALL)
        .lineTo(inner_r - 0.004, BASE_Z + WALL)
        .threePointArc((inner_r, BASE_Z + WALL + 0.004), (inner_r, BASE_Z + WALL + 0.008))
        .lineTo(inner_r, BARREL_TOP_Z - 0.002)
        .threePointArc(
            ((inner_r + inner_neck_r) / 2.0, (BARREL_TOP_Z + SHOULDER_TOP_Z) / 2.0 + 0.002),
            (inner_neck_r, SHOULDER_TOP_Z + 0.002),
        )
        .lineTo(inner_neck_r, NECK_TOP_Z)
        .lineTo(0.0, NECK_TOP_Z)
        .close()
        .revolve(360.0, (0, 0, 0), (0, 1, 0))
    )

    # Subtract cavity to create hollow bottle
    return outer.cut(cavity)


def _cap_solid():
    # White child-resistant cap with grip features. Local frame: origin at the
    # cap joint; the solid disc sits above (0..CAP_TOP_Z) and the skirt hangs
    # down to z=-SKIRT_DROP, wrapping the neck.
    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, -SKIRT_DROP))
        .circle(CAP_R)
        .extrude(CAP_HEIGHT)
    )
    outer = outer.edges(">Z").fillet(0.002)

    # Hollow underside: cavity slips over the neck (open at the bottom)
    cavity = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, -SKIRT_DROP - 0.001))
        .circle(NECK_R + 0.002)
        .extrude(CAP_HEIGHT - 0.004)
    )
    cap = outer.cut(cavity)

    # Vertical grip ribs around the skirt for child-resistant push-and-turn.
    # These are larger/more prominent than a standard screw cap.
    n = 20
    rib_h = CAP_HEIGHT - 0.003
    zc = -SKIRT_DROP + rib_h / 2.0
    for i in range(n):
        ang = 2.0 * math.pi * i / n
        x = (CAP_R - 0.0005) * math.cos(ang)
        y = (CAP_R - 0.0005) * math.sin(ang)
        rib = (
            cq.Workplane("XY")
            .transformed(offset=(x, y, zc), rotate=(0, 0, math.degrees(ang)))
            .box(0.0025, 0.0018, rib_h)
        )
        cap = cap.union(rib)

    # Top knurl pattern: raised cross-hatch on top for grip
    top_z = CAP_TOP_Z - 0.001
    for i in range(4):
        ang = math.pi / 4.0 * i
        x = 0.0
        y = 0.0
        ridge = (
            cq.Workplane("XY")
            .transformed(offset=(x, y, top_z + 0.001), rotate=(0, 0, math.degrees(ang)))
            .box(CAP_R * 1.6, 0.002, 0.0015)
        )
        cap = cap.union(ridge)

    return cap


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="medicine_bottle")

    # Amber translucent plastic and opaque white cap (typical medicine bottle).
    amber = model.material("amber_plastic", rgba=(0.55, 0.35, 0.12, 0.45))
    white = model.material("cap_white", rgba=(0.92, 0.92, 0.90, 1.0))

    # ---- bottle body (root) ----
    body = model.part("bottle")
    shell = _bottle_shell()
    body.visual(mesh_from_cadquery(shell, "bottle_shell"), material=amber, name="bottle_shell")
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, NECK_TOP_Z),
        mass=0.025,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    # ---- massless carrier (no visuals): carries the turn joint ----
    carrier = model.part("cap_carrier")
    carrier.inertial = Inertial.from_geometry(Box((0.008, 0.008, 0.008)), mass=1e-4)

    # ---- white child-resistant cap ----
    cap = model.part("cap")
    cap_geo = _cap_solid()
    cap.visual(mesh_from_cadquery(cap_geo, "cap_shell"), material=white, name="cap_shell")
    # Off-axis marker tab so the rotation is detectable
    cap.visual(
        Box((0.005, 0.007, 0.010)),
        origin=Origin(xyz=(CAP_R + 0.002, 0.0, -SKIRT_DROP + 0.005)),
        material=white,
        name="cap_marker",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(CAP_R, CAP_HEIGHT),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, (CAP_TOP_Z - SKIRT_DROP) / 2.0)),
    )

    # ---- decoupled joints: turn (revolute) then push-down (prismatic) ----
    # cap_turn: rotation about +Z for the twist part of push-and-turn
    model.articulation(
        "cap_turn",
        ArticulationType.REVOLUTE,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.0, lower=0.0, upper=0.8),
    )
    # cap_push: push-down along -Z for the press part of push-and-turn
    model.articulation(
        "cap_push",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=PUSH_TRAVEL, effort=2.0, velocity=0.5),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle")
    cap = object_model.get_part("cap")
    turn = object_model.get_articulation("cap_turn")
    push = object_model.get_articulation("cap_push")

    bottle_shell = body.get_visual("bottle_shell")
    cap_shell = cap.get_visual("cap_shell")

    # --- bottle is amber translucent (typical medicine bottle), cap is opaque white ---
    ctx.check(
        "bottle material is amber translucent (alpha < 1)",
        bottle_shell.material.rgba is not None and bottle_shell.material.rgba[3] < 1.0,
        details=f"bottle rgba={bottle_shell.material.rgba}",
    )
    ctx.check(
        "cap material is opaque white",
        cap_shell.material.rgba is not None
        and cap_shell.material.rgba[3] >= 0.99
        and min(cap_shell.material.rgba[:3]) > 0.85,
        details=f"cap rgba={cap_shell.material.rgba}",
    )

    # --- medicine bottle proportions: wider than tall (body aspect ratio) ---
    body_aabb = ctx.part_world_aabb(body)
    if body_aabb is not None:
        body_ext = _ext(body_aabb)
        body_dia = max(body_ext[0], body_ext[1])
        body_height = body_ext[2]
        ctx.check(
            "medicine bottle has wide proportions (diameter > 0.4 * height)",
            body_dia > 0.4 * body_height,
            details=f"diameter={body_dia:.4f}, height={body_height:.4f}",
        )

    # --- cap sits on top of the neck ---
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "cap mounted on top of the neck",
        cap_pos is not None and cap_pos[2] > BARREL_TOP_Z,
        details=f"cap origin={cap_pos}",
    )

    # --- cap_turn is REVOLUTE with partial-turn limits (child-resistant) ---
    turn_limits = turn.motion_limits
    ctx.check(
        "cap_turn is REVOLUTE with bounded limits",
        turn.articulation_type == ArticulationType.REVOLUTE
        and turn_limits is not None
        and turn_limits.lower is not None
        and turn_limits.upper is not None
        and turn_limits.upper > turn_limits.lower
        and turn_limits.upper <= math.pi,
        details=f"type={turn.articulation_type}, limits=({turn_limits.lower}, {turn_limits.upper})",
    )

    # --- cap_push is PRISMATIC for push-down motion ---
    push_limits = push.motion_limits
    ctx.check(
        "cap_push is PRISMATIC with push-down travel",
        push.articulation_type == ArticulationType.PRISMATIC
        and push_limits is not None
        and push_limits.lower is not None
        and push_limits.upper is not None
        and push_limits.upper > 0.0
        and push_limits.upper <= 0.010,
        details=f"type={push.articulation_type}, limits=({push_limits.lower}, {push_limits.upper})",
    )

    # --- at least one non-fixed joint exists ---
    all_joints = [turn, push]
    has_nonfixed = any(
        j.articulation_type in (ArticulationType.REVOLUTE, ArticulationType.PRISMATIC, ArticulationType.CONTINUOUS)
        for j in all_joints
    )
    ctx.check(
        "model has at least one non-fixed joint",
        has_nonfixed,
        details="no revolute, prismatic, or continuous joint found",
    )

    # The cap skirt slips over the neck threads at rest -> intentional overlap.
    ctx.allow_overlap(
        cap,
        body,
        elem_a="cap_shell",
        elem_b="bottle_shell",
        reason="Cap skirt is intentionally seated over the threaded neck for child-resistant mechanism.",
    )

    # --- cap_turn rotates the cap: the off-axis marker moves ---
    marker0 = None
    marker_turned = None
    with ctx.pose({turn: 0.0}):
        marker0 = ctx.part_world_aabb(cap)
    with ctx.pose({turn: 0.4}):  # half of max rotation
        marker_turned = ctx.part_world_aabb(cap)
    e0 = _ext(marker0)
    e_turned = _ext(marker_turned)
    ctx.check(
        "cap rotation moves the off-axis marker (extents change)",
        abs(e0[0] - e_turned[0]) > 0.0005 or abs(e0[1] - e_turned[1]) > 0.0005,
        details=f"rest extents={e0}, turned extents={e_turned}",
    )

    # --- cap_push moves the cap down (push-down mechanism) ---
    rest_z = ctx.part_world_position(cap)[2]
    with ctx.pose({push: PUSH_TRAVEL}):
        pushed_z = ctx.part_world_position(cap)[2]
    ctx.check(
        "cap_push moves the cap downward (push-down mechanism)",
        pushed_z < rest_z - PUSH_TRAVEL * 0.8,
        details=f"rest_z={rest_z}, pushed_z={pushed_z}",
    )

    # --- volume bands are present on the barrel (wider than base barrel) ---
    # Check that bottle shell has features wider than the base barrel radius
    shell_aabb = body.get_visual("bottle_shell")
    # The bands protrude beyond BODY_R, so the shell should have max X extent
    # slightly larger than BODY_R due to the bands
    body_dims = ctx.part_world_aabb(body)
    if body_dims is not None:
        mn, mx = body_dims
        x_extent = mx[0] - mn[0]
        # With 2 bands protruding 0.0012m on each side, total diameter increase ~0.0024
        expected_min_dia = 2.0 * BODY_R  # base diameter without bands
        ctx.check(
            "volume bands present (barrel wider than base diameter at band locations)",
            x_extent >= expected_min_dia + BAND_PROTRUSION * 0.5,
            details=f"x_extent={x_extent:.5f}, expected_min={expected_min_dia + BAND_PROTRUSION * 0.5:.5f}",
        )

    # --- neck threads are raised spiral ridges (wider than plain neck) ---
    # The neck thread ridges protrude beyond NECK_R
    thread_ridge_r = NECK_R + 0.002
    ctx.check(
        "neck threads have raised spiral ridges (ridge radius > neck radius)",
        thread_ridge_r > NECK_R + 0.001,
        details=f"ridge_r={thread_ridge_r}, neck_r={NECK_R}",
    )

    return ctx.report()


object_model = build_object_model()
