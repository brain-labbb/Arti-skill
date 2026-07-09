from __future__ import annotations

# Cosmetic cream squeeze tube with a stand-up flip cap.
# Fork of the white-screw-cap variant: the tall ribbed screw cap is replaced
# with a wide flat-topped octagonal base cap (the tube stands mouth-down on it)
# topped by a small snap flip lid that swings open about a horizontal REVOLUTE
# living-hinge axis.
#
# Frame: tube stands upright along +Z. The flat crimped bottom seam sits at
# z=0; the soft pale-yellow body rises and rounds out, narrows into a white
# shoulder cone, then a short white threaded nozzle. A wide octagonal base cap
# sits fixed on the nozzle, and a flip lid on top of the base cap swings open
# to expose the dispensing orifice.
#
# Construction notes:
#  - The body is a CadQuery loft: flat, wide crimp seam at the bottom that
#    rounds out into a near-circular cross-section higher up (round-to-flat
#    squeeze-tube silhouette), shelled hollow at the open shoulder end.
#  - Shoulder and nozzle are thin-walled annular geometry with a visible bore
#    through the mouth into the tube cavity.
#  - The base cap is an octagonal disc with a central nozzle socket, grip bumps
#    on each facet, a raised dispensing orifice ring, and a hinge barrel at the
#    back edge.
#  - The flip lid is a disc with a central plug and opening tab, mounted on the
#    hinge barrel via a horizontal revolute joint.

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
SHOULDER_TOP_Z = 0.104  # top of the shoulder cone / base of the nozzle
NOZZLE_TOP_Z = 0.122  # top rim of the threaded nozzle

BODY_MAX_R = 0.0225  # near-circular radius of the body at the shoulder
CRIMP_HALF_W = 0.0235  # half-width of the flat crimp seam (wide, flat)
CRIMP_HALF_T = 0.0028  # half-thickness of the flat crimp seam (thin)

NOZZLE_R = 0.0072
NOZZLE_BORE_R = 0.0039

# ---- flip-cap closure dimensions ----
BASE_CAP_RADIUS = 0.025  # octagon circumradius (50mm stance diameter)
BASE_CAP_HEIGHT = 0.010
BASE_CAP_TOP_Z = NOZZLE_TOP_Z + BASE_CAP_HEIGHT  # flat top of the base cap

FLIP_LID_R = 0.011  # flip lid disc radius
FLIP_LID_THICKNESS = 0.003
DISPENSING_ORIFICE_R = 0.005  # dispensing orifice radius on base cap top

# Grip bump parameters (repeated on each octagonal facet)
N_GRIP_BUMPS = 8
GRIP_BUMP_R = 0.0018

# Hinge barrel position
HINGE_X = -(BASE_CAP_RADIUS - 0.005)
HINGE_Z = BASE_CAP_TOP_Z + FLIP_LID_THICKNESS / 2


def _rounded_rect_pts(hw: float, ht: float, fillet: float, n_arc: int = 8):
    f = min(fillet, hw, ht)
    cx = hw - f
    cy = ht - f
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


def _body_solid() -> cq.Workplane:
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
    solid = wp.loft(ruled=False)
    return solid


def _body_mesh():
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
    h = SHOULDER_TOP_Z - BODY_TOP_Z
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP_Z)
        .circle(BODY_MAX_R * 0.985)
        .workplane(offset=h)
        .circle(NOZZLE_R * 1.5)
        .loft(ruled=False)
    )
    collar = (
        cq.Workplane("XY")
        .workplane(offset=SHOULDER_TOP_Z - 0.006)
        .circle(NOZZLE_R * 1.5)
        .extrude(0.006)
    )
    bore = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP_Z - 0.002)
        .circle(NOZZLE_BORE_R * 1.08)
        .extrude(h + 0.010)
    )
    return mesh_from_cadquery(shoulder.union(collar).cut(bore), "shoulder")


def _nozzle_mesh():
    h = NOZZLE_TOP_Z - SHOULDER_TOP_Z
    nozzle = (
        cq.Workplane("XY")
        .workplane(offset=SHOULDER_TOP_Z)
        .circle(NOZZLE_R)
        .circle(NOZZLE_BORE_R)
        .extrude(h)
    )
    n_threads = 4
    for k in range(n_threads):
        zz = SHOULDER_TOP_Z + 0.0025 + k * (h - 0.005) / (n_threads - 1)
        ring = (
            cq.Workplane("XY")
            .workplane(offset=zz)
            .circle(NOZZLE_R + 0.00075)
            .circle(NOZZLE_BORE_R)
            .extrude(0.0012)
        )
        nozzle = nozzle.union(ring)
    return mesh_from_cadquery(nozzle, "nozzle_open_mouth")


def _base_cap_mesh() -> cq.Workplane:
    """Wide flat octagonal stance disc with central nozzle socket.
    The orifice ring and hinge barrel are separate visuals for cleaner mesh."""
    R = BASE_CAP_RADIUS
    h = BASE_CAP_HEIGHT

    # Octagonal disc: extends 1mm below z=0 in local frame to overlap with
    # the nozzle top surface for visual connectivity.
    base = (
        cq.Workplane("XY")
        .workplane(offset=-0.001)
        .polygon(N_GRIP_BUMPS, 2 * R)
        .extrude(h + 0.001)
    )

    # Central socket bore: tight fit around the threaded nozzle for connectivity
    bore = (
        cq.Workplane("XY")
        .workplane(offset=-0.002)
        .circle(NOZZLE_R + 0.0002)
        .extrude(h + 0.003)
    )
    base = base.cut(bore)

    return base


def _base_cap_mesh_wrapped():
    """Wrap the base cap CadQuery build into a managed mesh."""
    return mesh_from_cadquery(_base_cap_mesh(), "base_cap_disc")


def _orifice_ring_mesh():
    """Raised dispensing orifice ring on top of base cap."""
    ring = (
        cq.Workplane("XY")
        .circle(DISPENSING_ORIFICE_R + 0.002)
        .circle(DISPENSING_ORIFICE_R)
        .extrude(0.003)
    )
    return mesh_from_cadquery(ring, "orifice_ring")


def _hinge_barrel_mesh():
    """Hinge barrel at the back edge of the base cap for the flip lid."""
    barrel = (
        cq.Workplane("XY")
        .center(HINGE_X, 0)
        .rect(0.006, 0.014)
        .extrude(FLIP_LID_THICKNESS / 2 + 0.002)
    )
    return mesh_from_cadquery(barrel, "hinge_barrel")


def _grip_bump_mesh():
    """Shared geometry helper for repeated grip bump visuals.
    Small cylinder that embeds through the base cap disc for connectivity."""
    bump = (
        cq.Workplane("XY")
        .workplane(offset=-0.002)
        .circle(GRIP_BUMP_R)
        .extrude(BASE_CAP_HEIGHT + 0.004)
    )
    return mesh_from_cadquery(bump, "grip_bump")


def _flip_lid_mesh():
    """Snap flip lid disc with central plug, hinge tab, and opening tab.
    Built in the part's local frame: origin at the hinge barrel center,
    the disc extends in +X toward the center of the base cap."""
    half_t = FLIP_LID_THICKNESS / 2

    # Main disc: centered at (FLIP_LID_R, 0, 0) in local frame
    lid = (
        cq.Workplane("XY")
        .workplane(offset=-half_t)
        .center(FLIP_LID_R, 0)
        .circle(FLIP_LID_R)
        .extrude(FLIP_LID_THICKNESS)
    )

    # Hinge tab at the origin (extends only in +X from hinge, not behind it)
    hinge_tab = (
        cq.Workplane("XY")
        .workplane(offset=-half_t)
        .center(0.003, 0)
        .rect(0.006, 0.012)
        .extrude(FLIP_LID_THICKNESS)
    )
    lid = lid.union(hinge_tab)

    # Central plug on bottom (snaps into dispensing orifice when closed)
    plug = (
        cq.Workplane("XY")
        .workplane(offset=-half_t - 0.002)
        .center(FLIP_LID_R, 0)
        .circle(DISPENSING_ORIFICE_R * 0.85)
        .extrude(0.002)
    )
    lid = lid.union(plug)

    # Opening tab at the front edge (+X side) for finger grip
    tab = (
        cq.Workplane("XY")
        .workplane(offset=-half_t)
        .center(2 * FLIP_LID_R + 0.003, 0)
        .rect(0.006, 0.009)
        .extrude(FLIP_LID_THICKNESS)
    )
    lid = lid.union(tab)

    return mesh_from_cadquery(lid, "flip_lid_disc")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cosmetic_squeeze_tube_flip_cap")

    pale_yellow = model.material("pale_yellow", rgba=(0.93, 0.89, 0.66, 1.0))
    white = model.material("white", rgba=(0.96, 0.96, 0.95, 1.0))
    cap_white = model.material("cap_white", rgba=(0.94, 0.94, 0.93, 1.0))
    lid_teal = model.material("lid_teal", rgba=(0.55, 0.78, 0.78, 1.0))
    grip_grey = model.material("grip_grey", rgba=(0.82, 0.82, 0.80, 1.0))

    # ---- body (root): pale-yellow squeeze tube + shoulder + nozzle + base cap ----
    body = model.part("tube_body")
    body.visual(_body_mesh(), material=pale_yellow, name="tube_body")
    body.visual(_shoulder_mesh(), material=white, name="shoulder")
    body.visual(_nozzle_mesh(), material=white, name="nozzle")
    # Base cap: wide octagonal stance disc fixed on the nozzle
    body.visual(
        _base_cap_mesh_wrapped(),
        origin=Origin(xyz=(0.0, 0.0, NOZZLE_TOP_Z)),
        material=cap_white,
        name="base_cap_disc",
    )
    # Orifice ring on top of base cap
    body.visual(
        _orifice_ring_mesh(),
        origin=Origin(xyz=(0.0, 0.0, NOZZLE_TOP_Z + BASE_CAP_HEIGHT - 0.001)),
        material=cap_white,
        name="orifice_ring",
    )
    # Hinge barrel at the back edge of base cap
    body.visual(
        _hinge_barrel_mesh(),
        origin=Origin(xyz=(0.0, 0.0, NOZZLE_TOP_Z + BASE_CAP_HEIGHT - 0.002)),
        material=cap_white,
        name="hinge_barrel",
    )
    # Repeated grip bumps on each octagonal facet (separate visuals, loop naming)
    # Positioned well inside the octagon and embedded into the base cap disc
    # for visual connectivity with the base_cap_disc visual.
    facet_dist = BASE_CAP_RADIUS * math.cos(math.pi / N_GRIP_BUMPS)
    for i in range(N_GRIP_BUMPS):
        angle = 2 * math.pi * i / N_GRIP_BUMPS + math.pi / N_GRIP_BUMPS
        bx = (facet_dist - GRIP_BUMP_R * 0.5) * math.cos(angle)
        by = (facet_dist - GRIP_BUMP_R * 0.5) * math.sin(angle)
        bz = NOZZLE_TOP_Z + BASE_CAP_HEIGHT * 0.15
        body.visual(
            _grip_bump_mesh(),
            origin=Origin(xyz=(bx, by, bz)),
            material=grip_grey,
            name=f"grip_bump_{i}",
        )
    body.inertial = Inertial.from_geometry(
        Box((0.050, 0.050, 0.135)), mass=0.045,
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
    )

    # ---- flip lid: snap lid disc that swings open about horizontal hinge ----
    flip_lid = model.part("flip_lid")
    flip_lid.visual(
        _flip_lid_mesh(),
        material=lid_teal,
        name="flip_lid_disc",
    )
    flip_lid.inertial = Inertial.from_geometry(
        Box((2 * FLIP_LID_R + 0.006, 2 * FLIP_LID_R, FLIP_LID_THICKNESS)),
        mass=0.003,
        origin=Origin(xyz=(FLIP_LID_R, 0.0, 0.0)),
    )

    # ---- REVOLUTE joint: flip lid swings open about horizontal Y axis ----
    # Origin at the hinge barrel center on the base cap top face.
    # Axis (0, -1, 0): positive q swings the front edge (+X) upward (+Z).
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flip_lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=2.0, lower=0.0, upper=math.pi * 0.65,
        ),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("tube_body")
    flip_lid = object_model.get_part("flip_lid")
    lid_hinge = object_model.get_articulation("lid_hinge")

    # --- Intentional overlap: flip lid plug seats into dispensing orifice ---
    ctx.allow_overlap(
        flip_lid,
        body,
        elem_a="flip_lid_disc",
        elem_b="base_cap_disc",
        reason="Flip lid plug intentionally seats into the dispensing orifice ring when closed.",
    )

    # --- Body unchanged: tube tapers to a flat, wide crimped bottom seam ---
    full_ext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "tube body is taller than it is wide (upright tube proportions)",
        full_ext[2] > full_ext[0] and full_ext[2] > full_ext[1],
        details=f"body extents={full_ext}",
    )
    ctx.check(
        "bottom crimp seam is flat (wide along X, thin along Y)",
        CRIMP_HALF_W > CRIMP_HALF_T * 4.0,
        details=f"crimp half_w={CRIMP_HALF_W}, half_t={CRIMP_HALF_T}",
    )

    # --- Open nozzle bore unchanged ---
    ctx.check(
        "threaded nozzle is an open annular mouth",
        NOZZLE_BORE_R > 0.0035 and NOZZLE_R - NOZZLE_BORE_R > 0.0025,
        details=f"nozzle outer_r={NOZZLE_R}, bore_r={NOZZLE_BORE_R}",
    )
    ctx.check(
        "shoulder bore aligns with the hollow tube body",
        BODY_TOP_Z < SHOULDER_TOP_Z < NOZZLE_TOP_Z and NOZZLE_BORE_R < BODY_MAX_R * 0.3,
        details=f"body_top={BODY_TOP_Z}, shoulder_top={SHOULDER_TOP_Z}, "
        f"nozzle_top={NOZZLE_TOP_Z}, bore_r={NOZZLE_BORE_R}",
    )

    # --- Base cap is wide, flat, and octagonal ---
    base_cap_aabb = ctx.part_element_world_aabb(body, elem="base_cap_disc")
    base_cap_ext = _ext(base_cap_aabb)
    ctx.check(
        "base cap is wider than the tube body on both XY axes (stable stance)",
        base_cap_ext[0] > 2 * BODY_MAX_R and base_cap_ext[1] > 2 * BODY_MAX_R,
        details=f"base_cap_ext={base_cap_ext}, body_diameter={2*BODY_MAX_R}",
    )
    ctx.check(
        "base cap is flat (much wider than tall)",
        base_cap_ext[0] > base_cap_ext[2] * 3.0 and base_cap_ext[1] > base_cap_ext[2] * 3.0,
        details=f"base_cap_ext={base_cap_ext}",
    )
    ctx.check(
        "base cap sits at the nozzle top of the tube",
        base_cap_aabb[0][2] >= NOZZLE_TOP_Z - 0.002,
        details=f"base_cap min_z={base_cap_aabb[0][2]}, nozzle_top={NOZZLE_TOP_Z}",
    )

    # --- Flip lid mounted on base cap top ---
    lid_pos = ctx.part_world_position(flip_lid)
    ctx.check(
        "flip lid is mounted at the top of the tube on the base cap",
        lid_pos is not None and lid_pos[2] >= BASE_CAP_TOP_Z - 0.005,
        details=f"lid origin={lid_pos}, base_cap_top_z={BASE_CAP_TOP_Z}",
    )

    # --- Mechanism: flip lid swings open about horizontal axis ---
    ctx.check(
        "lid_hinge is revolute about a horizontal axis (Z component near zero)",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE
        and abs(lid_hinge.axis[2]) < 0.01,
        details=f"type={lid_hinge.articulation_type}, axis={lid_hinge.axis}",
    )

    # Pose check: lid opens upward (max Z of AABB increases)
    aabb_closed = ctx.part_world_aabb(flip_lid)
    with ctx.pose({lid_hinge: math.pi * 0.5}):
        aabb_open = ctx.part_world_aabb(flip_lid)
    ctx.check(
        "flip lid swings upward when opened (max Z increases)",
        aabb_open[1][2] > aabb_closed[1][2] + 0.005,
        details=f"closed max_z={aabb_closed[1][2]:.5f}, open max_z={aabb_open[1][2]:.5f}",
    )

    # Pose check: lid front edge moves back when opened (max X decreases)
    ctx.check(
        "flip lid front edge swings back when opened (max X decreases)",
        aabb_open[1][0] < aabb_closed[1][0] - 0.003,
        details=f"closed max_x={aabb_closed[1][0]:.5f}, open max_x={aabb_open[1][0]:.5f}",
    )

    # --- Grip bumps exist with correct naming ---
    grip_count = sum(
        1 for v in body.visuals if v.name and v.name.startswith("grip_bump_")
    )
    ctx.check(
        "grip bumps present on base cap facets",
        grip_count == N_GRIP_BUMPS,
        details=f"found {grip_count} grip bumps, expected {N_GRIP_BUMPS}",
    )

    # --- Dispensing orifice accessible when lid is open ---
    with ctx.pose({lid_hinge: math.pi * 0.5}):
        ctx.expect_gap(
            flip_lid,
            body,
            axis="z",
            min_gap=-0.002,
            positive_elem="flip_lid_disc",
            negative_elem="base_cap_disc",
            name="open lid clears the dispensing orifice area",
        )

    return ctx.report()


object_model = build_object_model()
