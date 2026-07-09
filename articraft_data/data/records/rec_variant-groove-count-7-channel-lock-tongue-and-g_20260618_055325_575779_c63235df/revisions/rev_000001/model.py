from __future__ import annotations

# Channel-lock tongue-and-groove pliers variant (groove_count = 7).
# Based on the heavy-duty lineman pliers parent; structural change is the
# tongue-and-groove adjustment mechanism replacing the fixed rivet pivot.
#
# Layout: tool lies in the XY plane, Z is the thickness axis.
# Jaws point +X, handles sweep back to -X.
# Lower half (root) has the grooved shank. Jaw carrier slides along the shank
# (PRISMATIC). Upper half pivots from the carrier (REVOLUTE).
# 7 adjustment grooves are evenly spaced on the lower jaw shank.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- shared dimensions (meters) ----
HALF_T = 0.009            # half thickness of each forged plate
JAW_FACE = 0.0003         # closed jaw inner faces sit at y = +/-JAW_FACE
OPEN_LIMIT = math.radians(30.0)
TANG_HALF_W = 0.004       # steel handle tang half-width in plan

# ---- groove / shank dimensions ----
GROOVE_COUNT = 7
GROOVE_SPACING = 0.005    # center-to-center spacing between grooves
GROOVE_X0 = -0.003        # x position of groove_0 in lower-half frame
GROOVE_W = 0.0025         # groove width along X
GROOVE_DY = 0.009         # groove depth across Y (most of shank width)
GROOVE_DZ = 0.002         # groove depth into Z (recess into shank top face)

SHANK_X_MIN = -0.022
SHANK_X_MAX = 0.038
SHANK_HALF_W = 0.007      # shank half-width in Y

# Carrier (tongue bracket on the shank)
CARRIER_L = 0.018         # length along X
CARRIER_W = 0.016         # width along Y
CARRIER_T = 0.006         # thickness along Z
CARRIER_BOSS_R = 0.005    # pivot boss radius on carrier top
CARRIER_BOSS_H = 0.003    # pivot boss height

# Default groove index (middle)
DEFAULT_GROOVE_IDX = 3
DEFAULT_PIVOT_X = GROOVE_X0 + DEFAULT_GROOVE_IDX * GROOVE_SPACING

# Prismatic slide limits relative to the articulation origin at DEFAULT_PIVOT_X
SLIDE_LOWER = -(DEFAULT_GROOVE_IDX * GROOVE_SPACING)
SLIDE_UPPER = (GROOVE_COUNT - 1 - DEFAULT_GROOVE_IDX) * GROOVE_SPACING


def _groove_x(i: int) -> float:
    """X position of groove_i in the lower-half frame."""
    return GROOVE_X0 + i * GROOVE_SPACING


# ---- interpolation helpers ----

def _interp(x: float, pts: list[tuple[float, float]]) -> float:
    pts = sorted(pts)
    if x <= pts[0][0]:
        return pts[0][1]
    if x >= pts[-1][0]:
        return pts[-1][1]
    for (x0, v0), (x1, v1) in zip(pts, pts[1:]):
        if x0 <= x <= x1:
            t = (x - x0) / (x1 - x0)
            return v0 + t * (v1 - v0)
    return pts[-1][1]


# ---- lower half handle geometry (world frame) ----
LOWER_TANG_PTS = [
    (-0.016, -0.004),
    (-0.036, -0.011),
    (-0.076, -0.019),
    (-0.106, -0.024),
    (-0.124, -0.026),
]
LOWER_CENTERLINE = LOWER_TANG_PTS + [(-0.137, -0.0274)]

LOWER_GRIP_SECTIONS = [
    (-0.038, 0.0100, 0.0085),   # tight thumb guard: intersects steel tang
    (-0.046, 0.0080, 0.0110),
    (-0.066, 0.0078, 0.0108),
    (-0.091, 0.0080, 0.0112),
    (-0.111, 0.0086, 0.0118),
    (-0.126, 0.0088, 0.0120),
    (-0.134, 0.0062, 0.0086),
    (-0.137, 0.0028, 0.0040),
]
LOWER_INLAY_XS = [-0.042, -0.061, -0.081, -0.101, -0.118, -0.124]

# ---- upper half handle geometry (local frame, pivot at origin) ----
UPPER_TANG_PTS = [
    (-0.026, -0.004),
    (-0.046, -0.011),
    (-0.086, -0.019),
    (-0.116, -0.024),
    (-0.134, -0.026),
]
UPPER_CENTERLINE = UPPER_TANG_PTS + [(-0.147, -0.0274)]

UPPER_GRIP_SECTIONS = [
    (-0.048, 0.0100, 0.0085),   # tight thumb guard: intersects steel tang
    (-0.056, 0.0080, 0.0110),
    (-0.076, 0.0078, 0.0108),
    (-0.101, 0.0080, 0.0112),
    (-0.121, 0.0086, 0.0118),
    (-0.136, 0.0088, 0.0120),
    (-0.144, 0.0062, 0.0086),
    (-0.147, 0.0028, 0.0040),
]
UPPER_INLAY_XS = [-0.052, -0.071, -0.091, -0.111, -0.128, -0.134]

INLAY_HALF_H = 0.0075
INLAY_Z_CENTER = 0.006


def _lower_yc(x: float) -> float:
    return _interp(x, LOWER_CENTERLINE)


def _upper_yc(x: float) -> float:
    return _interp(x, UPPER_CENTERLINE)


def _lower_grip_w(x: float) -> float:
    return _interp(x, [(sx, w) for sx, w, _h in LOWER_GRIP_SECTIONS])


def _upper_grip_w(x: float) -> float:
    return _interp(x, [(sx, w) for sx, w, _h in UPPER_GRIP_SECTIONS])


def _strip_loop(pts: list[tuple[float, float]], half_w: float) -> list[tuple[float, float]]:
    n = len(pts)
    normals: list[tuple[float, float]] = []
    for i in range(n):
        if i == 0:
            dx, dy = pts[1][0] - pts[0][0], pts[1][1] - pts[0][1]
        elif i == n - 1:
            dx, dy = pts[-1][0] - pts[-2][0], pts[-1][1] - pts[-2][1]
        else:
            dx, dy = pts[i + 1][0] - pts[i - 1][0], pts[i + 1][1] - pts[i - 1][1]
        length = math.hypot(dx, dy)
        normals.append((-dy / length, dx / length))
    left = [(p[0] + half_w * nx, p[1] + half_w * ny) for p, (nx, ny) in zip(pts, normals)]
    right = [(p[0] - half_w * nx, p[1] - half_w * ny) for p, (nx, ny) in zip(pts, normals)]
    return left + right[::-1]


def _ellipse_wire(x: float, yc: float, zc: float, w: float, h: float) -> cq.Wire:
    wp = cq.Workplane("YZ", origin=(x, 0.0, 0.0)).center(yc, zc).ellipse(w, h)
    return wp.val()


# ========================= LOWER HALF GEOMETRY =========================

def _lower_jaw_solid() -> cq.Workplane:
    """Lower jaw: polished steel, squared nose, serrations, pipe grip, wire cutter.
    s=+1: jaw on +Y side.
    """
    s = 1
    profile = [
        (0.020, JAW_FACE),
        (0.075, JAW_FACE),
        (0.075, 0.0105),
        (0.066, 0.0118),
        (0.052, 0.0136),
        (0.038, 0.0152),
        (0.028, 0.0160),
        (0.020, 0.0140),
    ]
    pts = [(x, s * y) for x, y in profile]
    jaw = cq.Workplane("XY").polyline(pts).close().extrude(HALF_T, both=True)

    # Serrations on the inner face
    for i in range(6):
        xi = 0.040 + 0.005 * i
        groove = (
            cq.Workplane("XY")
            .box(0.0014, 0.0024, 0.020)
            .translate((xi, s * JAW_FACE, 0.0))
        )
        jaw = jaw.cut(groove)

    # Pipe-grip recess
    recess = (
        cq.Workplane("XY")
        .circle(0.0045)
        .extrude(0.011, both=True)
        .translate((0.034, s * JAW_FACE, 0.0))
    )
    jaw = jaw.cut(recess)

    # Scallop teeth around recess
    for ang_deg in (40.0, 75.0, 105.0, 140.0):
        a = math.radians(ang_deg)
        sx = 0.034 + 0.0045 * math.cos(a)
        sy = s * (JAW_FACE + 0.0045 * math.sin(a))
        scallop = (
            cq.Workplane("XY")
            .circle(0.0008)
            .extrude(0.011, both=True)
            .translate((sx, sy, 0.0))
        )
        jaw = jaw.cut(scallop)

    # Wire-cutter V-notch near the shank
    notch_pts = [(0.023, s * -0.001), (0.029, s * -0.001), (0.026, s * 0.003)]
    notch = cq.Workplane("XY").polyline(notch_pts).close().extrude(0.011, both=True)
    jaw = jaw.cut(notch)

    return jaw


def _lower_shank_solid() -> cq.Workplane:
    """Lower shank: flat steel bar with 7 groove cuts on the top face."""
    shank_len = SHANK_X_MAX - SHANK_X_MIN
    shank = (
        cq.Workplane("XY")
        .box(shank_len, 2 * SHANK_HALF_W, 2 * HALF_T)
        .translate(((SHANK_X_MIN + SHANK_X_MAX) / 2.0, 0.0, 0.0))
    )

    # Cut groove notches into the top face (+Z)
    for i in range(GROOVE_COUNT):
        gx = _groove_x(i)
        cutter = (
            cq.Workplane("XY")
            .box(GROOVE_W, GROOVE_DY, GROOVE_DZ + 0.001)
            .translate((gx, 0.0, HALF_T - GROOVE_DZ / 2.0 + 0.0005))
        )
        shank = shank.cut(cutter)

    return shank


def _lower_handle_solid() -> cq.Workplane:
    """Steel handle tang sweeping back from the shank into the grip."""
    s = 1
    pts = [(x, s * y) for x, y in LOWER_TANG_PTS]
    loop = _strip_loop(pts, TANG_HALF_W)
    return cq.Workplane("XY").polyline(loop).close().extrude(HALF_T, both=True)


def _lower_grip_solid() -> cq.Solid:
    """Soft-touch rubber grip on the lower handle."""
    s = 1
    wires = [
        _ellipse_wire(x, s * _lower_yc(x), 0.0, w, h)
        for x, w, h in LOWER_GRIP_SECTIONS
    ]
    return cq.Solid.makeLoft(wires, ruled=False)


def _lower_inlay_solid() -> cq.Solid:
    """Glossy red inlay along the outer face of the lower grip."""
    s = 1
    wires = [
        _ellipse_wire(x, s * _lower_yc(x), INLAY_Z_CENTER,
                      _lower_grip_w(x) - 0.0012, INLAY_HALF_H)
        for x in LOWER_INLAY_XS
    ]
    return cq.Solid.makeLoft(wires, ruled=False)


def _groove_plate_solid(i: int) -> cq.Workplane:
    """One groove liner plate at groove_i position on the shank top face.

    The plate extends below the groove cut bottom into the solid shank to
    ensure mesh connectivity with the shank body.
    """
    gx = _groove_x(i)
    plate_h = GROOVE_DZ + 0.002  # taller than the cut depth
    return (
        cq.Workplane("XY")
        .box(GROOVE_W - 0.0002, GROOVE_DY - 0.0004, plate_h)
        .translate((gx, 0.0, HALF_T - GROOVE_DZ / 2.0 - 0.0005))
    )


# ========================= JAW CARRIER GEOMETRY =========================

def _carrier_body_solid() -> cq.Workplane:
    """Jaw carrier: flat bracket sitting on the shank top face."""
    body = (
        cq.Workplane("XY")
        .box(CARRIER_L, CARRIER_W, CARRIER_T)
        .translate((0.0, 0.0, HALF_T + CARRIER_T / 2.0))
    )
    return body


def _carrier_tongue_solid() -> cq.Workplane:
    """Tongue that drops into the groove slot on the shank."""
    tongue = (
        cq.Workplane("XY")
        .box(GROOVE_W - 0.0002, GROOVE_DY - 0.0006, GROOVE_DZ)
        .translate((0.0, 0.0, HALF_T - GROOVE_DZ / 2.0))
    )
    return tongue


# ========================= UPPER HALF GEOMETRY =========================

def _upper_jaw_solid() -> cq.Workplane:
    """Upper jaw in the upper-half local frame (pivot at origin).
    s=-1: jaw on -Y side.
    """
    s = -1
    profile = [
        (0.015, JAW_FACE),
        (0.065, JAW_FACE),
        (0.065, 0.0105),
        (0.056, 0.0118),
        (0.042, 0.0136),
        (0.030, 0.0150),
        (0.020, 0.0155),
        (0.015, 0.0135),
    ]
    pts = [(x, s * y) for x, y in profile]
    jaw = cq.Workplane("XY").polyline(pts).close().extrude(HALF_T, both=True)

    # Serrations
    for i in range(5):
        xi = 0.030 + 0.006 * i
        groove = (
            cq.Workplane("XY")
            .box(0.0014, 0.0024, 0.020)
            .translate((xi, s * JAW_FACE, 0.0))
        )
        jaw = jaw.cut(groove)

    # Pipe-grip recess
    recess = (
        cq.Workplane("XY")
        .circle(0.004)
        .extrude(0.011, both=True)
        .translate((0.025, s * JAW_FACE, 0.0))
    )
    jaw = jaw.cut(recess)

    return jaw


def _upper_tongue_solid() -> cq.Workplane:
    """Upper jaw tongue/shank extending back from the jaw to the handle area.

    Extended to overlap with both the jaw (forward) and the handle (backward)
    to ensure mesh connectivity within the upper_half part.
    """
    s = -1
    pts = [
        (-0.030, s * 0.003),
        (0.020, s * 0.003),
        (0.020, s * 0.014),
        (-0.030, s * 0.014),
    ]
    return cq.Workplane("XY").polyline(pts).close().extrude(HALF_T, both=True)


def _upper_handle_solid() -> cq.Workplane:
    """Steel handle tang on the upper half (local frame)."""
    s = -1
    pts = [(x, s * y) for x, y in UPPER_TANG_PTS]
    loop = _strip_loop(pts, TANG_HALF_W)
    return cq.Workplane("XY").polyline(loop).close().extrude(HALF_T, both=True)


def _upper_grip_solid() -> cq.Solid:
    """Rubber grip on the upper handle (local frame)."""
    s = -1
    wires = [
        _ellipse_wire(x, s * _upper_yc(x), 0.0, w, h)
        for x, w, h in UPPER_GRIP_SECTIONS
    ]
    return cq.Solid.makeLoft(wires, ruled=False)


def _upper_inlay_solid() -> cq.Solid:
    """Red inlay along the outer face of the upper grip (local frame)."""
    s = -1
    wires = [
        _ellipse_wire(x, s * _upper_yc(x), INLAY_Z_CENTER,
                      _upper_grip_w(x) - 0.0012, INLAY_HALF_H)
        for x in UPPER_INLAY_XS
    ]
    return cq.Solid.makeLoft(wires, ruled=False)


def _upper_hub_solid() -> cq.Workplane:
    """Forged hub disc at the upper-half pivot area.

    Connects the tongue (on -Y side) to the handle tang (on +Y side)
    through a circular forging disc centered on the shank centerline.
    """
    hub = (
        cq.Workplane("XY")
        .circle(0.015)
        .extrude(HALF_T, both=True)
        .translate((-0.025, 0.0, 0.0))
    )
    return hub


# ========================= BUILD MODEL =========================

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="channel_lock_pliers")

    # Materials
    steel_polished = model.material("steel_polished", rgba=(0.80, 0.81, 0.84, 1.0))
    steel_forged = model.material("steel_forged", rgba=(0.66, 0.67, 0.70, 1.0))
    steel_brushed = model.material("steel_brushed", rgba=(0.61, 0.62, 0.65, 1.0))
    groove_dark = model.material("groove_dark", rgba=(0.30, 0.31, 0.33, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.09, 1.0))
    grip_red = model.material("grip_red", rgba=(0.80, 0.09, 0.11, 1.0))
    carrier_blue = model.material("carrier_blue", rgba=(0.15, 0.28, 0.52, 1.0))

    # -------- Lower half (root) --------
    lower = model.part("lower_half")

    lower.visual(
        mesh_from_cadquery(_lower_jaw_solid(), "lower_jaw", tolerance=0.0002),
        name="jaw",
        material=steel_polished,
    )
    lower.visual(
        mesh_from_cadquery(_lower_shank_solid(), "lower_shank", tolerance=0.0002),
        name="shank",
        material=steel_forged,
    )
    lower.visual(
        mesh_from_cadquery(_lower_handle_solid(), "lower_handle", tolerance=0.0002),
        name="handle",
        material=steel_forged,
    )
    lower.visual(
        mesh_from_cadquery(_lower_grip_solid(), "lower_grip", tolerance=0.0002),
        name="grip",
        material=rubber_black,
    )
    lower.visual(
        mesh_from_cadquery(_lower_inlay_solid(), "lower_inlay", tolerance=0.0002),
        name="grip_inlay",
        material=grip_red,
    )

    # 7 grooves on the lower shank (inline parent visuals, for-loop naming)
    for i in range(GROOVE_COUNT):
        lower.visual(
            mesh_from_cadquery(_groove_plate_solid(i), f"groove_{i}", tolerance=0.0003),
            name=f"groove_{i}",
            material=groove_dark,
        )

    # -------- Jaw carrier (intermediate sliding part) --------
    carrier = model.part("jaw_carrier")

    carrier.visual(
        mesh_from_cadquery(_carrier_body_solid(), "carrier_body", tolerance=0.0002),
        name="carrier_body",
        material=carrier_blue,
    )
    carrier.visual(
        mesh_from_cadquery(_carrier_tongue_solid(), "carrier_tongue", tolerance=0.0002),
        name="carrier_tongue",
        material=steel_brushed,
    )
    # Pivot boss on top of the carrier
    carrier.visual(
        Cylinder(CARRIER_BOSS_R, CARRIER_BOSS_H),
        origin=Origin(xyz=(0.0, 0.0, HALF_T + CARRIER_T + CARRIER_BOSS_H / 2.0)),
        name="pivot_boss",
        material=steel_brushed,
    )

    # -------- Upper half --------
    upper = model.part("upper_half")

    upper.visual(
        mesh_from_cadquery(_upper_jaw_solid(), "upper_jaw", tolerance=0.0002),
        name="jaw",
        material=steel_polished,
    )
    upper.visual(
        mesh_from_cadquery(_upper_tongue_solid(), "upper_tongue", tolerance=0.0002),
        name="tongue",
        material=steel_forged,
    )
    upper.visual(
        mesh_from_cadquery(_upper_hub_solid(), "upper_hub", tolerance=0.0002),
        name="hub",
        material=steel_forged,
    )
    upper.visual(
        mesh_from_cadquery(_upper_handle_solid(), "upper_handle", tolerance=0.0002),
        name="handle",
        material=steel_forged,
    )
    upper.visual(
        mesh_from_cadquery(_upper_grip_solid(), "upper_grip", tolerance=0.0002),
        name="grip",
        material=rubber_black,
    )
    upper.visual(
        mesh_from_cadquery(_upper_inlay_solid(), "upper_inlay", tolerance=0.0002),
        name="grip_inlay",
        material=grip_red,
    )

    # -------- Articulations --------

    # PRISMATIC: jaw carrier slides along the lower shank (X axis).
    # Origin at the default groove position so q=0 is the middle groove.
    model.articulation(
        "slide",
        ArticulationType.PRISMATIC,
        parent=lower,
        child=carrier,
        origin=Origin(xyz=(DEFAULT_PIVOT_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0, velocity=0.1,
            lower=SLIDE_LOWER, upper=SLIDE_UPPER,
        ),
    )

    # REVOLUTE: upper half pivots from the carrier (Z axis, perpendicular to tool plane).
    # Origin at the top of the carrier boss, so the pivot is at the contact surface.
    pivot_z = HALF_T + CARRIER_T + CARRIER_BOSS_H
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=carrier,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, pivot_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=80.0, velocity=3.0,
            lower=0.0, upper=OPEN_LIMIT,
        ),
    )

    return model


# ========================= TESTS =========================

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower = object_model.get_part("lower_half")
    carrier = object_model.get_part("jaw_carrier")
    upper = object_model.get_part("upper_half")
    slide = object_model.get_articulation("slide")
    pivot = object_model.get_articulation("pivot")

    lower_jaw = lower.get_visual("jaw")
    upper_jaw = upper.get_visual("jaw")
    lower_shank = lower.get_visual("shank")

    # ---- Joint structure checks ----
    slide_limits = slide.motion_limits
    ctx.check(
        "slide is a prismatic joint with groove-range travel",
        slide.articulation_type == ArticulationType.PRISMATIC
        and slide_limits is not None
        and slide_limits.lower is not None
        and slide_limits.upper is not None
        and abs(slide_limits.lower - SLIDE_LOWER) < 1e-6
        and abs(slide_limits.upper - SLIDE_UPPER) < 1e-6,
        details=f"limits={slide_limits}",
    )

    pivot_limits = pivot.motion_limits
    ctx.check(
        "pivot is a revolute jaw-open joint (0..30 degrees)",
        pivot.articulation_type == ArticulationType.REVOLUTE
        and pivot_limits is not None
        and pivot_limits.lower is not None
        and pivot_limits.upper is not None
        and abs(pivot_limits.lower) < 1e-9
        and abs(pivot_limits.upper - OPEN_LIMIT) < 1e-6,
        details=f"limits={pivot_limits}",
    )

    # ---- 7 grooves exist on the lower shank ----
    groove_names = [f"groove_{i}" for i in range(GROOVE_COUNT)]
    for gname in groove_names:
        gv = lower.get_visual(gname)
        ctx.check(
            f"groove {gname} exists on lower shank",
            gv is not None,
            details=f"visual '{gname}' not found on lower_half",
        )

    # Grooves are evenly spaced along X on the shank
    groove_centers = []
    for gname in groove_names:
        gaabb = ctx.part_element_world_aabb(lower, elem=gname)
        if gaabb is not None:
            cx = 0.5 * (gaabb[0][0] + gaabb[1][0])
            groove_centers.append(cx)

    if len(groove_centers) == GROOVE_COUNT:
        spacings = [groove_centers[i + 1] - groove_centers[i]
                    for i in range(GROOVE_COUNT - 1)]
        avg_spacing = sum(spacings) / len(spacings)
        max_dev = max(abs(s - avg_spacing) for s in spacings)
        ctx.check(
            "grooves are evenly spaced along the shank",
            0.004 <= avg_spacing <= 0.006 and max_dev < 0.001,
            details=f"avg_spacing={avg_spacing:.4f} max_dev={max_dev:.4f}",
        )
    else:
        ctx.fail("all groove AABBs resolve", f"got {len(groove_centers)}/{GROOVE_COUNT}")

    # Grooves sit on the shank (within shank footprint in XY)
    ctx.expect_within(
        lower, lower,
        axes="xy",
        inner_elem=groove_names[0],
        outer_elem=lower_shank,
        margin=0.001,
        name="first groove within shank footprint",
    )
    ctx.expect_within(
        lower, lower,
        axes="xy",
        inner_elem=groove_names[-1],
        outer_elem=lower_shank,
        margin=0.001,
        name="last groove within shank footprint",
    )

    # ---- Carrier sits on the shank ----
    ctx.expect_contact(
        carrier, lower,
        elem_a="carrier_body",
        elem_b="shank",
        contact_tol=0.002,
        name="carrier body contacts the shank top face",
    )

    # Carrier tongue overlaps shank in XY (engages the groove)
    ctx.expect_overlap(
        carrier, lower,
        axes="xy",
        elem_a="carrier_tongue",
        elem_b="shank",
        min_overlap=0.002,
        name="carrier tongue overlaps shank in plan view",
    )

    # Allow the carrier tongue to overlap the shank (it sits in the groove cut)
    ctx.allow_overlap(
        carrier, lower,
        elem_a="carrier_tongue",
        elem_b="shank",
        reason="The carrier tongue is intentionally seated inside a groove cut "
               "in the shank top face, representing the tongue-and-groove engagement.",
    )
    ctx.expect_gap(
        carrier, lower,
        axis="z",
        positive_elem="carrier_body",
        negative_elem="shank",
        min_gap=-0.001,
        max_gap=0.002,
        name="carrier body sits on or slightly into the shank top",
    )

    # ---- Closed rest pose: jaw faces nearly touching ----
    ctx.expect_gap(
        lower, upper,
        axis="y",
        positive_elem=lower_jaw,
        negative_elem=upper_jaw,
        min_gap=0.0,
        max_gap=0.003,
        name="jaws closed and nearly touching at rest",
    )

    # ---- Open pose: jaws separate, handles spread ----
    closed_upper_jaw = ctx.part_element_world_aabb(upper, elem="jaw")
    closed_upper_grip = ctx.part_element_world_aabb(upper, elem="grip")

    with ctx.pose({pivot: OPEN_LIMIT}):
        ctx.expect_gap(
            lower, upper,
            axis="y",
            positive_elem=lower_jaw,
            negative_elem=upper_jaw,
            min_gap=0.004,
            name="jaws open apart at the 30 degree pose",
        )
        open_upper_jaw = ctx.part_element_world_aabb(upper, elem="jaw")
        open_upper_grip = ctx.part_element_world_aabb(upper, elem="grip")

        ctx.check(
            "upper jaw swings away from the lower jaw when opened",
            closed_upper_jaw is not None
            and open_upper_jaw is not None
            and open_upper_jaw[0][1] < closed_upper_jaw[0][1] - 0.012,
            details=f"closed={closed_upper_jaw}, open={open_upper_jaw}",
        )
        ctx.check(
            "upper handle spreads outward when jaws open",
            closed_upper_grip is not None
            and open_upper_grip is not None
            and open_upper_grip[1][1] > closed_upper_grip[1][1] + 0.02,
            details=f"closed={closed_upper_grip}, open={open_upper_grip}",
        )

    # ---- Prismatic slide: carrier actually moves along X ----
    rest_carrier = ctx.part_element_world_aabb(carrier, elem="carrier_body")
    with ctx.pose({slide: SLIDE_UPPER}):
        extended_carrier = ctx.part_element_world_aabb(carrier, elem="carrier_body")
        ctx.check(
            "carrier slides forward along the shank at max extension",
            rest_carrier is not None
            and extended_carrier is not None
            and extended_carrier[0][0] > rest_carrier[0][0] + 0.010,
            details=f"rest={rest_carrier}, extended={extended_carrier}",
        )

    with ctx.pose({slide: SLIDE_LOWER}):
        retracted_carrier = ctx.part_element_world_aabb(carrier, elem="carrier_body")
        ctx.check(
            "carrier slides backward along the shank at min position",
            rest_carrier is not None
            and retracted_carrier is not None
            and retracted_carrier[1][0] < rest_carrier[1][0] - 0.010,
            details=f"rest={rest_carrier}, retracted={retracted_carrier}",
        )

    # ---- Overall proportions ----
    la = ctx.part_world_aabb(lower)
    ua = ctx.part_world_aabb(upper)
    if la is not None and ua is not None:
        length = max(la[1][0], ua[1][0]) - min(la[0][0], ua[0][0])
        ctx.check(
            "overall tool length about 0.20-0.25 m",
            0.19 <= length <= 0.28,
            details=f"length={length:.4f}",
        )

    # ---- Grip inlays visible ----
    li = ctx.part_element_world_aabb(lower, elem="grip_inlay")
    lg = ctx.part_element_world_aabb(lower, elem="grip")
    if li is not None and lg is not None:
        ctx.check(
            "lower red inlay proud on the grip top face",
            li[1][2] >= lg[1][2] - 0.0005,
            details=f"inlay_top={li[1][2]:.4f} grip_top={lg[1][2]:.4f}",
        )

    ui = ctx.part_element_world_aabb(upper, elem="grip_inlay")
    ug = ctx.part_element_world_aabb(upper, elem="grip")
    if ui is not None and ug is not None:
        ctx.check(
            "upper red inlay proud on the grip top face",
            ui[1][2] >= ug[1][2] - 0.0005,
            details=f"inlay_top={ui[1][2]:.4f} grip_top={ug[1][2]:.4f}",
        )

    # Pivot boss visible on carrier top
    boss_aabb = ctx.part_element_world_aabb(carrier, elem="pivot_boss")
    body_aabb = ctx.part_element_world_aabb(carrier, elem="carrier_body")
    if boss_aabb is not None and body_aabb is not None:
        ctx.check(
            "pivot boss sits above the carrier body",
            boss_aabb[0][2] >= body_aabb[1][2] - 0.001,
            details=f"boss_bottom={boss_aabb[0][2]:.4f} body_top={body_aabb[1][2]:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
