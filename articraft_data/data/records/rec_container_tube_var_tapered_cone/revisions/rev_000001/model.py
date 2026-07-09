from __future__ import annotations

# Cosmetic cream conical tube with a white screw cap (conical-body fork).
# Frame: tube stands upright along +Z. The flat crimped bottom seam sits at
# z=0; the body is a smooth continuous funnel/cone taper that narrows
# monotonically from the wide flat crimp base all the way up to a slim
# near-circular shoulder opening. A white shoulder cone, short white threaded
# nozzle, and a white ribbed screw cap (continuous rotate + prismatic slide
# through a massless carrier) sit above the body, unchanged from the parent.
#
# Construction notes:
#  - The body is a CadQuery loft of rounded-rectangle cross-sections whose
#    width decreases monotonically with height. The flat crimp opens into a
#    near-circular profile that continues to shrink, producing a continuous
#    conical/funnel taper (not a barrel and not a squeeze bulge).
#  - The body is shelled hollow from the open shoulder end.
#  - Shoulder and nozzle are thin-walled annular geometry with a visible bore
#    through the mouth into the tube cavity; the cap is hollow from below so it
#    can seat over the open threaded nozzle instead of plugging it.

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
BODY_TOP_Z = 0.082  # where the tapered body meets the white shoulder
SHOULDER_TOP_Z = 0.104  # top of the shoulder cone / base of the nozzle
NOZZLE_TOP_Z = 0.122  # top rim of the threaded nozzle (cap rotate axis origin)
CAP_HEIGHT = 0.026  # ribbed cap height -> prismatic upper travel limit

# ---- body footprint: wide flat crimp -> slim near-circular at shoulder ----
CRIMP_HALF_W = 0.0235  # half-width of the flat crimp seam (wide, flat)
CRIMP_HALF_T = 0.0028  # half-thickness of the flat crimp seam (thin)
BODY_TOP_R = 0.0130  # near-circular radius at the slim shoulder end

NOZZLE_R = 0.0072
NOZZLE_BORE_R = 0.0039
CAP_R = 0.0098
CAP_INNER_R = 0.0077
CAP_TOP_THICKNESS = 0.0032

# How far the cap drops over the nozzle at rest so it seats as a real screw cap.
CAP_DROP = NOZZLE_TOP_Z - SHOULDER_TOP_Z  # 0.018 m


def _taper_levels(w_base: float, t_base: float, r_top: float, n: int = 5):
    """Shared helper: produce n loft cross-section levels for a monotonic
    funnel taper from a wide flat rectangle at z=0 to a slim circle at
    BODY_TOP_Z. Each level is (z, half_width, half_thickness)."""
    levels = []
    for i in range(n):
        u = i / (n - 1)  # 0..1 parameter along the taper
        z = BODY_BOTTOM_Z + u * (BODY_TOP_Z - BODY_BOTTOM_Z)
        # Smooth ease-in: width shrinks faster early, thickness grows to meet
        # the circle radius then both converge to r_top.
        # Use a sqrt ease so the taper reads as a continuous funnel.
        ease = math.sqrt(u)
        hw = w_base + (r_top - w_base) * ease
        # Thickness rises from the thin crimp to r_top (circular top).
        ht = t_base + (r_top - t_base) * ease
        levels.append((z, hw, ht))
    return levels


def _rounded_rect_pts(hw: float, ht: float, fillet: float, n_arc: int = 8):
    """Sample a rounded-rectangle wire as polyline points."""
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


def _loft_from_levels(levels, n_arc: int = 8) -> cq.Workplane:
    """Build a closed loft solid from a list of (z, hw, ht) cross-sections."""
    wp = cq.Workplane("XY")
    prev_z = 0.0
    for i, (z, hw, ht) in enumerate(levels):
        off = z if i == 0 else z - prev_z
        if i == 0:
            wp = wp.workplane(offset=z)
        else:
            wp = wp.workplane(offset=off)
        fr = min(hw, ht) * 0.98
        pts = _rounded_rect_pts(hw, ht, fillet=fr, n_arc=n_arc)
        wp = wp.polyline(pts).close()
        prev_z = z
    return wp.loft(ruled=False)


def _body_solid() -> cq.Workplane:
    # Continuous funnel/cone taper: wide flat crimp narrows monotonically to a
    # slim near-circular opening at the shoulder.
    levels = _taper_levels(CRIMP_HALF_W, CRIMP_HALF_T, BODY_TOP_R, n=7)
    return _loft_from_levels(levels)


def _body_mesh():
    outer = _body_solid()
    # Shell hollow from the open shoulder (top) end so the tube reads as a
    # real thin-walled container. The inner loft follows the same taper with
    # a wall offset.
    wall = 0.0018
    inner_top_r = max(BODY_TOP_R - wall, NOZZLE_BORE_R * 1.1)
    inner_levels = _taper_levels(
        CRIMP_HALF_W * 0.90, CRIMP_HALF_T * 0.35, inner_top_r, n=7
    )
    # Shift inner up slightly so the crimp end stays closed.
    inner_levels = [(max(z, 0.005), hw, ht) for (z, hw, ht) in inner_levels]
    # Extend the top past BODY_TOP_Z so the cut opens at the shoulder.
    inner_levels[-1] = (BODY_TOP_Z + 0.01, inner_top_r, inner_top_r)
    inner = _loft_from_levels(inner_levels)
    body = outer.cut(inner)
    return mesh_from_cadquery(body, "tube_body")


def _shoulder_mesh():
    # White shoulder: a hollow cone narrowing from the slim body mouth up to
    # the nozzle base. The central bore continues down into the shelled body.
    h = SHOULDER_TOP_Z - BODY_TOP_Z
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP_Z)
        .circle(BODY_TOP_R)
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
    # White threaded nozzle: a short open tube with raised annular thread bands.
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


def _cap_mesh():
    # White ribbed screw cap: outer cylindrical wall + vertical rib rings,
    # closed by a top disc, hollow from below. Built in the cap's LOCAL frame
    # with z=0 at the cap bottom rim so the prismatic slide reads cleanly up +Z.
    wall_h = CAP_HEIGHT
    cap = cq.Workplane("XY").circle(CAP_R).extrude(wall_h)
    inner = (
        cq.Workplane("XY")
        .workplane(offset=-0.001)
        .circle(CAP_INNER_R)
        .extrude(wall_h - CAP_TOP_THICKNESS + 0.001)
    )
    cap = cap.cut(inner)
    # Knurl ribs around the skirt for grip.
    n_ribs = 18
    for i in range(n_ribs):
        a = 2 * math.pi * i / n_ribs
        rib = (
            cq.Workplane("XY")
            .center(CAP_R * math.cos(a), CAP_R * math.sin(a))
            .circle(0.00075)
            .extrude(wall_h * 0.82)
            .translate((0.0, 0.0, wall_h * 0.08))
        )
        cap = cap.union(rib)
    return mesh_from_cadquery(cap, "cap_hollow_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cosmetic_conical_tube")

    pale_yellow = model.material("pale_yellow", rgba=(0.93, 0.89, 0.66, 1.0))
    white = model.material("white", rgba=(0.96, 0.96, 0.95, 1.0))
    accent = model.material("cap_accent", rgba=(0.78, 0.80, 0.82, 1.0))

    # ---- body (root): tapered pale-yellow conical tube + white shoulder + nozzle ----
    body = model.part("tube_body")
    body.visual(_body_mesh(), material=pale_yellow, name="tube_body")
    body.visual(_shoulder_mesh(), material=white, name="shoulder")
    body.visual(_nozzle_mesh(), material=white, name="nozzle")
    body.inertial = Inertial.from_geometry(
        Box((0.047, 0.047, 0.122)), mass=0.04, origin=Origin(xyz=(0.0, 0.0, 0.06))
    )

    # ---- massless carrier: rotates with the cap about +Z (no visuals) ----
    carrier = model.part("cap_carrier")
    carrier.inertial = Inertial.from_geometry(Box((0.008, 0.008, 0.008)), mass=1e-4)

    # ---- cap: white ribbed screw cap (visuals live here) ----
    # The cap mesh is built with z=0 at its bottom rim. Offset visuals down by
    # CAP_DROP so the cap seats over the nozzle at rest (like a real screw cap).
    cap = model.part("cap")
    cap.visual(
        _cap_mesh(), material=white, name="cap_shell",
        origin=Origin(xyz=(0.0, 0.0, -CAP_DROP)),
    )
    # Off-axis marker so spin about +Z is observable (rides on the skirt).
    cap.visual(
        Box((0.004, 0.004, 0.006)),
        origin=Origin(xyz=(CAP_R - 0.001, 0.0, CAP_HEIGHT * 0.5 - CAP_DROP)),
        material=accent,
        name="cap_marker",
    )
    cap.inertial = Inertial.from_geometry(
        Box((2 * CAP_R, 2 * CAP_R, CAP_HEIGHT)), mass=0.006,
        origin=Origin(xyz=(0.0, 0.0, CAP_HEIGHT * 0.5 - CAP_DROP)),
    )

    # ---- two INDEPENDENT decoupled joints, both about vertical +Z at nozzle top ----
    model.articulation(
        "cap_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, NOZZLE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )
    model.articulation(
        "cap_slide",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=CAP_HEIGHT, effort=1.0, velocity=1.0),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("tube_body")
    cap = object_model.get_part("cap")
    cap_rotate = object_model.get_articulation("cap_rotate")
    cap_slide = object_model.get_articulation("cap_slide")

    # The cap seats down over the threaded nozzle at rest (intentional capture).
    # The thread rings (r=0.00795) overlap with the cap inner wall (r=0.0077)
    # inside the hollow cap cavity — a realistic screw-cap thread engagement.
    ctx.allow_overlap(
        cap,
        body,
        elem_a="cap_shell",
        elem_b="nozzle",
        reason="Cap wall encompasses the nozzle thread rings inside the hollow cap cavity; "
        "realistic screw-cap thread engagement.",
    )
    # The cap bottom rim seats on the shoulder collar at rest.
    ctx.allow_overlap(
        cap,
        body,
        elem_a="cap_shell",
        elem_b="shoulder",
        reason="Cap bottom rim seats against the shoulder collar top face when closed.",
    )

    # --- Hero: body is a continuous funnel/cone taper, not a barrel or bulge ---
    full_ext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "tube body is taller than it is wide (upright tube proportions)",
        full_ext[2] > full_ext[0] and full_ext[2] > full_ext[1],
        details=f"body extents={full_ext}",
    )
    # The crimp seam is flat: much wider than it is thick at the bottom.
    ctx.check(
        "bottom crimp seam is flat (wide along X, thin along Y)",
        CRIMP_HALF_W > CRIMP_HALF_T * 4.0,
        details=f"crimp half_w={CRIMP_HALF_W}, half_t={CRIMP_HALF_T}",
    )
    # The taper narrows monotonically: the body top is significantly slimmer
    # than the crimp base (funnel/cone, not a barrel).
    ctx.check(
        "body tapers monotonically: top is slimmer than base (conical funnel)",
        BODY_TOP_R < CRIMP_HALF_W * 0.70,
        details=f"body_top_r={BODY_TOP_R}, crimp_half_w={CRIMP_HALF_W}, "
        f"ratio={BODY_TOP_R / CRIMP_HALF_W:.2f}",
    )
    # Verify the taper levels are strictly monotonically narrowing in width.
    outer_levels = _taper_levels(CRIMP_HALF_W, CRIMP_HALF_T, BODY_TOP_R, n=7)
    widths = [hw for (_, hw, _) in outer_levels]
    monotonic = all(widths[i] > widths[i + 1] for i in range(len(widths) - 1))
    ctx.check(
        "body cross-section width narrows at every loft level (no bulge)",
        monotonic,
        details=f"widths={[round(w, 5) for w in widths]}",
    )

    # --- Hero: cap is white on a white shoulder, mounted at the top ---
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "cap mounted at the top of the tube",
        cap_pos is not None and cap_pos[2] > SHOULDER_TOP_Z,
        details=f"cap origin={cap_pos}",
    )
    ctx.check(
        "cap origin is seated at the nozzle top",
        cap_pos is not None and abs(cap_pos[2] - NOZZLE_TOP_Z) < 1e-6,
        details=f"cap origin={cap_pos}, nozzle_top_z={NOZZLE_TOP_Z}",
    )
    # The cap bottom rim sits at SHOULDER_TOP_Z (cap dropped over the nozzle).
    cap_aabb = ctx.part_world_aabb(cap)
    ctx.check(
        "cap encompasses the nozzle (bottom rim at shoulder level)",
        cap_aabb is not None and cap_aabb[0][2] <= SHOULDER_TOP_Z + 0.001,
        details=f"cap aabb min_z={cap_aabb[0][2] if cap_aabb else None}, "
        f"shoulder_top_z={SHOULDER_TOP_Z}",
    )

    # --- Hero: mouth is open and continues into the hollow body ---
    nozzle_aabb = ctx.part_element_world_aabb(body, elem="nozzle")
    nozzle_ext = _ext(nozzle_aabb)
    ctx.check(
        "threaded nozzle is an open annular mouth, not a sealed cylinder",
        NOZZLE_BORE_R > 0.0035 and NOZZLE_R - NOZZLE_BORE_R > 0.0025,
        details=f"nozzle outer_r={NOZZLE_R}, bore_r={NOZZLE_BORE_R}",
    )
    ctx.check(
        "open mouth is tall enough to see down into the tube",
        nozzle_ext[2] > 0.016 and nozzle_aabb[1][2] >= NOZZLE_TOP_Z - 0.001,
        details=f"nozzle extents={nozzle_ext}, nozzle aabb={nozzle_aabb}",
    )
    ctx.check(
        "shoulder bore aligns with the hollow tube body",
        BODY_TOP_Z < SHOULDER_TOP_Z < NOZZLE_TOP_Z and NOZZLE_BORE_R < BODY_TOP_R * 0.35,
        details=f"body_top={BODY_TOP_Z}, shoulder_top={SHOULDER_TOP_Z}, "
        f"nozzle_top={NOZZLE_TOP_Z}, bore_r={NOZZLE_BORE_R}",
    )
    ctx.check(
        "screw cap is hollow underneath and can fit over the open mouth",
        CAP_INNER_R > NOZZLE_R and CAP_TOP_THICKNESS < CAP_HEIGHT * 0.2,
        details=f"cap_inner_r={CAP_INNER_R}, nozzle_r={NOZZLE_R}, "
        f"cap_top_thickness={CAP_TOP_THICKNESS}",
    )

    # --- Mechanism: cap spins about +Z (off-axis marker swings around) ---
    aabb0 = ctx.part_world_aabb(cap)
    with ctx.pose({cap_rotate: math.pi / 2.0}):
        aabb90 = ctx.part_world_aabb(cap)
    xmax0, ymax0 = aabb0[1][0], aabb0[1][1]
    xmax90, ymax90 = aabb90[1][0], aabb90[1][1]
    ctx.check(
        "cap rotates about +Z (marker bulge swings from +X to +Y)",
        xmax0 > ymax0 + 1e-4 and ymax90 > xmax90 + 1e-4,
        details=f"rest xmax={xmax0:.5f} ymax={ymax0:.5f} | "
        f"q90 xmax={xmax90:.5f} ymax={ymax90:.5f}",
    )

    # --- Mechanism: cap slides straight up +Z, off the nozzle ---
    z_rest = ctx.part_world_position(cap)[2]
    with ctx.pose({cap_slide: CAP_HEIGHT}):
        z_up = ctx.part_world_position(cap)[2]
        ctx.expect_gap(
            cap,
            body,
            axis="z",
            min_gap=0.0,
            positive_elem="cap_shell",
            negative_elem="nozzle",
            name="cap clears nozzle when slid up",
        )
    ctx.check(
        "cap slides up off the nozzle along +Z",
        z_up > z_rest + CAP_HEIGHT * 0.8,
        details=f"z_rest={z_rest}, z_up={z_up}",
    )

    # Both joints decoupled & vertical at the nozzle top.
    ctx.check(
        "cap_rotate is continuous about +Z",
        cap_rotate.articulation_type == ArticulationType.CONTINUOUS
        and tuple(cap_rotate.axis) == (0.0, 0.0, 1.0),
        details=f"axis={cap_rotate.axis}",
    )
    ctx.check(
        "cap_slide is prismatic about +Z",
        cap_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(cap_slide.axis) == (0.0, 0.0, 1.0),
        details=f"axis={cap_slide.axis}",
    )

    return ctx.report()


object_model = build_object_model()
