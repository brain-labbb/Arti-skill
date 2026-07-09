from __future__ import annotations

# Cosmetic squeeze tube with an angled applicator tip and matching snap cap.
# Fork of the white-screw-cap tube: body and shoulder stay the same round-to-flat
# squeeze-tube form, but the straight threaded nozzle is replaced by a slanted
# dispensing applicator tip (bore exits on a diagonal cut face, like a serum
# or eye-cream wand). The round screw cap is replaced by a matching angled snap
# cap that lifts straight off along the tip axis via a PRISMATIC pull joint.
#
# Frame: tube stands upright along +Z. The flat crimped bottom seam sits at
# z=0; the soft pale-yellow body rises and rounds out, narrows into a white
# shoulder cone, then an angled applicator tip tilts ~30 deg from vertical.
# The snap cap seats over the tip and slides off along the tip axis.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- key heights (meters), measured up +Z from the bottom crimp seam ----
BODY_BOTTOM_Z = 0.0
BODY_TOP_Z = 0.082  # where the soft body meets the white shoulder
SHOULDER_TOP_Z = 0.104  # top of the shoulder cone / base of the applicator tip

BODY_MAX_R = 0.0225  # near-circular radius of the body at the shoulder
CRIMP_HALF_W = 0.0235  # half-width of the flat crimp seam (wide, flat)
CRIMP_HALF_T = 0.0028  # half-thickness of the flat crimp seam (thin)

# ---- applicator tip (angled from vertical toward +X) ----
TIP_ANGLE_DEG = 30
TIP_ANGLE = math.radians(TIP_ANGLE_DEG)
TIP_LENGTH = 0.025  # along the tip axis
TIP_OUTER_R = 0.006
TIP_BORE_R = 0.003
CUT_ANGLE_DEG = 45  # diagonal cut face angle from the cross-section plane
CUT_ANGLE = math.radians(CUT_ANGLE_DEG)

# Tip axis unit vector in world frame (tilted toward +X)
TIP_AXIS_X = math.sin(TIP_ANGLE)
TIP_AXIS_Z = math.cos(TIP_ANGLE)
TIP_AXIS = (TIP_AXIS_X, 0.0, TIP_AXIS_Z)

# ---- snap cap (slides along the tip axis) ----
CAP_LENGTH = 0.030  # cap tube length along tip axis
CAP_OUTER_R = 0.009
CAP_INNER_R = 0.0065
CAP_TOP_THICKNESS = 0.003
CAP_SLIDE_UPPER = CAP_LENGTH + 0.008  # prismatic upper travel limit

# Grip rib parameters (repeated sub-parts on the cap skirt)
N_GRIP_RIBS = 12
GRIP_RIB_HALF_W = 0.0005  # thin radial dimension
GRIP_RIB_HALF_T = 0.0015  # wider tangential dimension
GRIP_RIB_HEIGHT = CAP_LENGTH * 0.60


# ========================================================================
# Shared geometry helpers
# ========================================================================

def _rounded_rect_pts(hw: float, ht: float, fillet: float, n_arc: int = 8):
    """Sample a rounded-rectangle profile (counter-clockwise)."""
    f = min(fillet, hw, ht)
    cx, cy = hw - f, ht - f
    pts = []
    corners = [
        (cx, -cy, -math.pi / 2, 0.0),
        (cx, cy, 0.0, math.pi / 2),
        (-cx, cy, math.pi / 2, math.pi),
        (-cx, -cy, math.pi, 3 * math.pi / 2),
    ]
    for ccx, ccy, a0, a1 in corners:
        for k in range(n_arc + 1):
            a = a0 + (a1 - a0) * k / n_arc
            pts.append((ccx + f * math.cos(a), ccy + f * math.sin(a)))
    return pts


def _tilt_point(x: float, y: float, z: float) -> tuple[float, float, float]:
    """Apply Ry(+TIP_ANGLE) to rotate from the untilted frame into world.
    Used to position cap visuals on the angled tip axis."""
    ca, sa = math.cos(TIP_ANGLE), math.sin(TIP_ANGLE)
    return (ca * x + sa * z, y, -sa * x + ca * z)


def _grip_rib_solid(theta: float) -> cq.Workplane:
    """Shared geometry helper: one small grip rib for the snap cap skirt.
    Built as a thin box in the untilted frame, rotated around Z by theta
    so the thin dimension points radially outward from the cap axis."""
    rib = cq.Workplane("XY").box(
        GRIP_RIB_HALF_W * 2, GRIP_RIB_HALF_T * 2, GRIP_RIB_HEIGHT
    )
    rib = rib.rotate((0, 0, 0), (0, 0, 1), math.degrees(theta))
    return rib


# ========================================================================
# Tube body (root): loft + shoulder (unchanged from parent family)
# ========================================================================

def _body_solid() -> cq.Workplane:
    """Round-to-flat squeeze tube body loft."""
    levels = [
        (BODY_BOTTOM_Z, CRIMP_HALF_W, CRIMP_HALF_T),
        (0.012, CRIMP_HALF_W * 0.99, 0.009),
        (0.030, BODY_MAX_R * 0.99, 0.016),
        (0.055, BODY_MAX_R, BODY_MAX_R * 0.97),
        (BODY_TOP_Z, BODY_MAX_R * 0.985, BODY_MAX_R * 0.985),
    ]
    wp = cq.Workplane("XY")
    prev_z = 0.0
    for i, (z, hw, ht) in enumerate(levels):
        off = z if i == 0 else z - prev_z
        if i == 0:
            wp = wp.workplane(offset=z)
        else:
            wp = wp.workplane(offset=off)
        fr = min(hw, ht) * 0.98
        pts = _rounded_rect_pts(hw, ht, fillet=fr, n_arc=8)
        wp = wp.polyline(pts).close()
        prev_z = z
    return wp.loft(ruled=False)


def _body_mesh():
    """Shelled tube body: hollow from the open shoulder end."""
    outer = _body_solid()
    inner_levels = [
        (0.006, CRIMP_HALF_W * 0.9, CRIMP_HALF_T * 0.4),
        (0.014, CRIMP_HALF_W * 0.9, 0.0072),
        (0.030, BODY_MAX_R * 0.9, 0.0135),
        (0.055, BODY_MAX_R * 0.9, BODY_MAX_R * 0.86),
        (BODY_TOP_Z + 0.01, BODY_MAX_R * 0.9, BODY_MAX_R * 0.86),
    ]
    wp = cq.Workplane("XY")
    prev_z = 0.0
    for i, (z, hw, ht) in enumerate(inner_levels):
        off = z if i == 0 else z - prev_z
        if i == 0:
            wp = wp.workplane(offset=z)
        else:
            wp = wp.workplane(offset=off)
        fr = min(hw, ht) * 0.98
        pts = _rounded_rect_pts(hw, ht, fillet=fr, n_arc=8)
        wp = wp.polyline(pts).close()
        prev_z = z
    inner = wp.loft(ruled=False)
    body = outer.cut(inner)
    return mesh_from_cadquery(body, "tube_body")


def _shoulder_mesh():
    """White shoulder cone with bore, narrowing to the applicator tip base."""
    h = SHOULDER_TOP_Z - BODY_TOP_Z
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP_Z)
        .circle(BODY_MAX_R * 0.985)
        .workplane(offset=h)
        .circle(TIP_OUTER_R * 1.8)
        .loft(ruled=False)
    )
    # Short collar ring at the top for the angled tip to emerge from
    collar = (
        cq.Workplane("XY")
        .workplane(offset=SHOULDER_TOP_Z - 0.005)
        .circle(TIP_OUTER_R * 1.8)
        .extrude(0.005)
    )
    # Central bore through the shoulder into the tube cavity
    bore = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP_Z - 0.002)
        .circle(TIP_BORE_R * 1.1)
        .extrude(h + 0.008)
    )
    return mesh_from_cadquery(shoulder.union(collar).cut(bore), "shoulder")


# ========================================================================
# Applicator tip: angled cylinder with diagonal cut dispensing face
# ========================================================================

def _applicator_tip_mesh():
    """Angled applicator tip with diagonal cut face and open bore.
    Built along local +Z then rotated to TIP_ANGLE and placed at SHOULDER_TOP_Z."""
    # Outer cylinder along local +Z
    tip = cq.Workplane("XY").circle(TIP_OUTER_R).extrude(TIP_LENGTH)

    # Central bore through the tip
    bore = (
        cq.Workplane("XY")
        .workplane(offset=-0.001)
        .circle(TIP_BORE_R)
        .extrude(TIP_LENGTH + 0.002)
    )
    tip = tip.cut(bore)

    # Diagonal cut at the top: remove material above a tilted plane.
    # The cut plane passes through (0, 0, cut_z_pivot) and is tilted at
    # CUT_ANGLE around Y. Material above z = cut_z_pivot - x*tan(CUT_ANGLE)
    # is removed, creating the slanted dispensing face.
    cut_z_pivot = TIP_LENGTH - TIP_OUTER_R * math.tan(CUT_ANGLE)
    cs = TIP_OUTER_R * 10  # cutter size (large enough to cover full cylinder)
    cutter = (
        cq.Workplane("XY")
        .workplane(offset=cut_z_pivot)
        .transformed(rotate=(0, CUT_ANGLE_DEG, 0))
        .rect(cs, cs)
        .extrude(cs)
    )
    tip = tip.cut(cutter)

    # Snap retention groove: a shallow annular channel near the tip base
    groove_z = 0.004
    groove = (
        cq.Workplane("XY")
        .workplane(offset=groove_z)
        .circle(TIP_OUTER_R + 0.001)
        .circle(TIP_OUTER_R - 0.0008)
        .extrude(0.0015)
    )
    tip = tip.cut(groove)

    # Rotate from local +Z to the world tip axis (tilt toward +X)
    tip = tip.rotate((0, 0, 0), (0, 1, 0), TIP_ANGLE_DEG)
    # Translate base to the shoulder top
    tip = tip.translate((0, 0, SHOULDER_TOP_Z))

    return mesh_from_cadquery(tip, "applicator_tip")


# ========================================================================
# Snap cap: hollow shell matching the angled tip
# ========================================================================

def _snap_cap_shell_mesh():
    """Snap cap hollow shell: open at bottom, closed at top.
    Built along local +Z; the visual origin applies the tilt rotation."""
    # Outer cylinder
    cap = cq.Workplane("XY").circle(CAP_OUTER_R).extrude(CAP_LENGTH)

    # Hollow interior (open from bottom, closed near top)
    inner = (
        cq.Workplane("XY")
        .workplane(offset=-0.001)
        .circle(CAP_INNER_R)
        .extrude(CAP_LENGTH - CAP_TOP_THICKNESS + 0.001)
    )
    cap = cap.cut(inner)

    # Small snap ridge inside the cap near the open end (retention feature)
    ridge = (
        cq.Workplane("XY")
        .workplane(offset=0.003)
        .circle(CAP_INNER_R)
        .circle(CAP_INNER_R - 0.0006)
        .extrude(0.0012)
    )
    cap = cap.union(ridge)

    return mesh_from_cadquery(cap, "snap_cap_shell")


# ========================================================================
# Build the articulated model
# ========================================================================

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cosmetic_tube_applicator_tip")

    pale_yellow = model.material("pale_yellow", rgba=(0.93, 0.89, 0.66, 1.0))
    white = model.material("white", rgba=(0.96, 0.96, 0.95, 1.0))
    teal = model.material("teal_cap", rgba=(0.30, 0.58, 0.62, 1.0))
    silver = model.material("silver_accent", rgba=(0.72, 0.76, 0.78, 1.0))

    # ---- tube body (root): body + shoulder + applicator tip ----
    body = model.part("tube_body")
    body.visual(_body_mesh(), material=pale_yellow, name="tube_body")
    body.visual(_shoulder_mesh(), material=white, name="shoulder")
    body.visual(_applicator_tip_mesh(), material=white, name="applicator_tip")
    body.inertial = Inertial.from_geometry(
        Box((0.05, 0.05, 0.14)), mass=0.04, origin=Origin(xyz=(0.0, 0.0, 0.06))
    )

    # ---- snap cap: angled protective cap with grip ribs ----
    cap = model.part("snap_cap")
    # Main shell: tilted to match the tip axis via rpy
    cap.visual(
        _snap_cap_shell_mesh(),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, TIP_ANGLE, 0.0)),
        material=teal,
        name="cap_shell",
    )

    # Grip ribs around the cap skirt (repeated sub-parts via for-i-in-range)
    for i in range(N_GRIP_RIBS):
        theta = 2 * math.pi * i / N_GRIP_RIBS
        # Position on the outer surface of the untilted cap cylinder
        surface_r = CAP_OUTER_R + GRIP_RIB_HALF_W * 0.5
        ux = surface_r * math.cos(theta)
        uy = surface_r * math.sin(theta)
        uz = CAP_LENGTH * 0.38  # lower portion of cap
        # Tilt to match the tip axis
        tx, ty, tz = _tilt_point(ux, uy, uz)
        cap.visual(
            mesh_from_cadquery(_grip_rib_solid(theta), f"grip_rib_{i}"),
            origin=Origin(xyz=(tx, ty, tz), rpy=(0.0, TIP_ANGLE, 0.0)),
            material=silver,
            name=f"grip_rib_{i}",
        )

    cap.inertial = Inertial.from_geometry(
        Box((2 * CAP_OUTER_R, 2 * CAP_OUTER_R, CAP_LENGTH)),
        mass=0.005,
        origin=Origin(xyz=(0.0, 0.0, CAP_LENGTH * 0.5)),
    )

    # ---- PRISMATIC pull joint: cap slides off along the tip axis ----
    model.articulation(
        "cap_pull",
        ArticulationType.PRISMATIC,
        parent=body,
        child=cap,
        # Joint origin at the actual contact surface (tip base / shoulder top)
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_TOP_Z)),
        axis=TIP_AXIS,
        motion_limits=MotionLimits(
            lower=0.0, upper=CAP_SLIDE_UPPER, effort=3.0, velocity=1.5
        ),
    )

    return model


# ========================================================================
# Tests
# ========================================================================

def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("tube_body")
    cap = object_model.get_part("snap_cap")
    cap_pull = object_model.get_articulation("cap_pull")

    # Intentional overlap: cap seats over the applicator tip when closed
    ctx.allow_overlap(
        cap,
        body,
        elem_a="cap_shell",
        elem_b="applicator_tip",
        reason="Snap cap intentionally seats over the applicator tip when closed.",
    )

    # --- Body proportions (unchanged from parent family) ---
    full_ext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "tube body is taller than wide (upright tube proportions)",
        full_ext[2] > full_ext[0] and full_ext[2] > full_ext[1],
        details=f"body extents={full_ext}",
    )
    ctx.check(
        "bottom crimp seam is flat (wide along X, thin along Y)",
        CRIMP_HALF_W > CRIMP_HALF_T * 4.0,
        details=f"crimp half_w={CRIMP_HALF_W}, half_t={CRIMP_HALF_T}",
    )

    # --- Applicator tip is angled (not vertical) ---
    tip_aabb = ctx.part_element_world_aabb(body, elem="applicator_tip")
    tip_ext = _ext(tip_aabb)
    # A vertical cylinder of radius R has X extent = 2R.
    # An angled cylinder has X extent > 2R due to the tilt projection.
    ctx.check(
        "applicator tip is angled from vertical (X extent exceeds diameter)",
        tip_ext[0] > 2.3 * TIP_OUTER_R,
        details=f"tip X extent={tip_ext[0]:.5f}, threshold={2.3*TIP_OUTER_R:.5f}",
    )

    # --- Applicator tip has open bore for dispensing ---
    ctx.check(
        "applicator tip has open bore (annular, not solid)",
        TIP_BORE_R > 0.002 and TIP_OUTER_R - TIP_BORE_R > 0.002,
        details=f"tip outer_r={TIP_OUTER_R}, bore_r={TIP_BORE_R}",
    )

    # --- Diagonal cut face: tip X extent is less than a full tilted cylinder ---
    # The diagonal cut removes material from the +X side of the tilted tip,
    # reducing the X extent compared to a full tilted cylinder.
    full_tilted_x_extent = (
        2 * TIP_OUTER_R * math.cos(TIP_ANGLE) + TIP_LENGTH * math.sin(TIP_ANGLE)
    )
    ctx.check(
        "diagonal cut visible (tip X extent less than full tilted cylinder)",
        tip_ext[0] < full_tilted_x_extent * 0.90,
        details=f"tip X extent={tip_ext[0]:.5f}, full tilted X={full_tilted_x_extent:.5f}",
    )

    # --- Snap cap is mounted at the shoulder top ---
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "snap cap mounted at shoulder top (contact surface)",
        cap_pos is not None and abs(cap_pos[2] - SHOULDER_TOP_Z) < 0.002,
        details=f"cap origin z={cap_pos[2] if cap_pos else None}, "
        f"shoulder_top_z={SHOULDER_TOP_Z}",
    )

    # --- Joint type and axis ---
    ctx.check(
        "cap_pull is PRISMATIC",
        cap_pull.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={cap_pull.articulation_type}",
    )
    axis = tuple(round(a, 4) for a in cap_pull.axis)
    expected_axis = tuple(round(a, 4) for a in TIP_AXIS)
    ctx.check(
        "cap_pull axis matches angled tip axis direction",
        axis == expected_axis,
        details=f"axis={axis}, expected={expected_axis}",
    )

    # --- Mechanism: cap slides off along the tip axis ---
    pos_rest = ctx.part_world_position(cap)
    with ctx.pose({cap_pull: CAP_SLIDE_UPPER}):
        pos_pulled = ctx.part_world_position(cap)
        # When fully pulled, the cap should clear the tip
        ctx.expect_gap(
            cap,
            body,
            axis="z",
            min_gap=0.001,
            positive_elem="cap_shell",
            negative_elem="applicator_tip",
            name="cap clears applicator tip when fully pulled off",
        )

    ctx.check(
        "cap slides off along angled tip axis (Z increases AND X increases)",
        pos_pulled[2] > pos_rest[2] + 0.005
        and pos_pulled[0] > pos_rest[0] + 0.002,
        details=f"rest=({pos_rest[0]:.4f},{pos_rest[2]:.4f}), "
        f"pulled=({pos_pulled[0]:.4f},{pos_pulled[2]:.4f})",
    )

    # --- Grip ribs exist on the cap ---
    rib_aabb = ctx.part_element_world_aabb(cap, elem="grip_rib_0")
    ctx.check(
        "snap cap has grip ribs (grip_rib_0 exists)",
        rib_aabb is not None,
        details="grip_rib_0 visual not found on snap_cap",
    )

    # --- Shoulder bore connects to hollow body ---
    ctx.check(
        "shoulder bore connects through to the hollow body",
        BODY_TOP_Z < SHOULDER_TOP_Z and TIP_BORE_R < BODY_MAX_R * 0.3,
        details=f"body_top={BODY_TOP_Z}, shoulder_top={SHOULDER_TOP_Z}, "
        f"bore_r={TIP_BORE_R}, body_r={BODY_MAX_R}",
    )

    return ctx.report()


object_model = build_object_model()
