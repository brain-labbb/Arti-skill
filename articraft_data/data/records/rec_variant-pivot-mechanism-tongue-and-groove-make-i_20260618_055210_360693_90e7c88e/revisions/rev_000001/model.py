from __future__ import annotations

# Channel-lock tongue-and-groove pliers variant.
# The lower jaw shank carries 5 stepped grooves; the upper jaw pivot
# indexes along them via a PRISMATIC joint to set jaw width.
# A REVOLUTE joint at the tongue provides jaw open/close.
#
# Layout: the tool lies in the XY plane with Z as the thickness axis.
# The lower half (root) has its jaw on +Y; the upper half jaw is on -Y.
# The groove track runs along -X on the lower shank behind the jaw hub.
# The two forged halves are half-lapped at the engagement zone so one
# half visibly passes over the other.

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
HALF_T = 0.009          # half thickness of each forged plate (full plate 0.018)
LAP_R = 0.016           # half-lap joint disc radius at the pivot
HUB_R = 0.015           # forged hub radius around the engagement zone
EPS = 0.0001            # lap clearance so the stacked halves do not penetrate
JAW_FACE = 0.0003       # closed jaw inner faces sit at y = +/-JAW_FACE
NOSE_X = 0.075          # blunt squared nose tip
OPEN_LIMIT = math.radians(30.0)

# Groove track (channel-lock adjustment)
GROOVE_COUNT = 5
GROOVE_W = 0.0020       # groove channel width along X
GROOVE_L = 0.012        # groove channel length along Y (spans shank width)
GROOVE_D = 0.0025       # groove channel depth along Z
GROOVE_SPACING = 0.004  # center-to-center groove spacing along X
GROOVE_X_START = -0.010 # first groove center X
GROOVE_TRACK_Y = -0.006 # groove track centerline Y (on the lower shank)
PRISMATIC_TRAVEL = GROOVE_SPACING * (GROOVE_COUNT - 1)  # 0.016 m

# Pivot carrier / tongue
TONGUE_W = 0.0030       # tongue width along X (fits between grooves)
TONGUE_L = 0.010        # tongue length along Y
TONGUE_H = 0.005        # tongue height along Z
BUTTON_R = 0.006        # push-button radius
BUTTON_H = 0.005        # push-button height above the upper surface

TANG_HALF_W = 0.004     # steel handle tang half width in plan

# Steel handle tang centerline in the tool plane (for the half whose jaw is +Y).
TANG_PTS = [
    (-0.010, -0.004),
    (-0.030, -0.011),
    (-0.070, -0.019),
    (-0.100, -0.024),
    (-0.118, -0.026),
]
# Extended centerline used by the over-molded grip (reaches the bulbous end).
CENTERLINE = TANG_PTS + [(-0.131, -0.0274)]

# Grip loft stations: (x, half_width_y, half_height_z)
GRIP_SECTIONS = [
    (-0.032, 0.0100, 0.0130),  # flared thumb guard near the pivot
    (-0.040, 0.0080, 0.0110),
    (-0.060, 0.0078, 0.0108),
    (-0.085, 0.0080, 0.0112),
    (-0.105, 0.0086, 0.0118),
    (-0.120, 0.0088, 0.0120),  # bulbous end swell
    (-0.128, 0.0062, 0.0086),
    (-0.131, 0.0028, 0.0040),
]

# Red inlay stations along the outer/top face (stops before the black end bulb).
INLAY_XS = [-0.036, -0.055, -0.075, -0.095, -0.112, -0.118]
INLAY_HALF_H = 0.0075
INLAY_Z_CENTER = 0.006


def _interp(x: float, pts: list[tuple[float, float]]) -> float:
    """Piecewise-linear interpolation over (x, value) points (any x order)."""
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


def _yc(x: float) -> float:
    """Handle centerline y at station x (jaw-on-+Y half)."""
    return _interp(x, CENTERLINE)


def _grip_w(x: float) -> float:
    return _interp(x, [(sx, w) for sx, w, _h in GRIP_SECTIONS])


def _strip_loop(pts: list[tuple[float, float]], half_w: float) -> list[tuple[float, float]]:
    """Closed plan-view loop offsetting a polyline by +/-half_w along 2D normals."""
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


def _lap_cut(s: int) -> cq.Workplane:
    """Material removed at the pivot so this half keeps only its lap layer.

    s=+1 keeps the lower lap (z <= -EPS); s=-1 keeps the upper lap (z >= +EPS).
    Uses a single large rectangular cut covering the entire engagement zone
    (hub + groove track + shank-to-hub overlap region).
    """
    # Cover from well past the leftmost groove to past the hub edge,
    # and wide enough in Y to include the full shank/hub overlap.
    cut_x_min = -0.030
    cut_x_max = 0.018
    cut_y_min = -0.025
    cut_y_max = 0.020
    cut_w = cut_x_max - cut_x_min
    cut_h = cut_y_max - cut_y_min
    cut_cx = 0.5 * (cut_x_min + cut_x_max)
    cut_cy = 0.5 * (cut_y_min + cut_y_max)
    cut_d = 0.014  # enough to clear full plate thickness + margin

    rect = cq.Workplane("XY").box(cut_w, cut_h, cut_d)
    if s > 0:
        # Remove everything above z=-EPS (keep lower lap only)
        return rect.translate((cut_cx, cut_cy, -EPS + cut_d / 2.0))
    else:
        # Remove everything below z=+EPS (keep upper lap only)
        return rect.translate((cut_cx, cut_cy, EPS - cut_d / 2.0))


def _jaw_solid(s: int) -> cq.Workplane:
    """Polished jaw: squared nose, serrations, pipe-grip recess, wire cutter."""
    profile = [
        (0.010, JAW_FACE),
        (NOSE_X, JAW_FACE),
        (NOSE_X, 0.0105),   # blunt squared nose tip
        (0.066, 0.0118),
        (0.052, 0.0136),
        (0.038, 0.0152),
        (0.026, 0.0162),
        (0.014, 0.0166),
        (0.009, 0.0150),
    ]
    pts = [(x, s * y) for x, y in profile]
    jaw = cq.Workplane("XY").polyline(pts).close().extrude(HALF_T, both=True)

    # Fine horizontal gripping serrations across the inner nose face.
    for i in range(7):
        xi = 0.046 + 0.004 * i
        groove = (
            cq.Workplane("XY")
            .box(0.0014, 0.0024, 0.020)
            .translate((xi, s * JAW_FACE, 0.0))
        )
        jaw = jaw.cut(groove)

    # Rounded pipe-grip recess behind the nose.
    recess = (
        cq.Workplane("XY")
        .circle(0.0045)
        .extrude(0.011, both=True)
        .translate((0.030, s * JAW_FACE, 0.0))
    )
    jaw = jaw.cut(recess)

    # Small scallop teeth around the recess rim (serrated pipe grip).
    for ang_deg in (40.0, 75.0, 105.0, 140.0):
        a = math.radians(ang_deg)
        sx = 0.030 + 0.0045 * math.cos(a)
        sy = s * (JAW_FACE + 0.0045 * math.sin(a))
        scallop = (
            cq.Workplane("XY")
            .circle(0.0008)
            .extrude(0.011, both=True)
            .translate((sx, sy, 0.0))
        )
        jaw = jaw.cut(scallop)

    # Short wire-cutter V-notch between the pipe grip and the pivot.
    notch_pts = [(0.015, s * -0.001), (0.0215, s * -0.001), (0.0185, s * 0.0035)]
    notch = cq.Workplane("XY").polyline(notch_pts).close().extrude(0.011, both=True)
    jaw = jaw.cut(notch)

    return jaw.cut(_lap_cut(s))


def _hub_solid(s: int) -> cq.Workplane:
    """Forged circular hub around the engagement zone, thinned to its lap layer."""
    hub = cq.Workplane("XY").circle(HUB_R).extrude(HALF_T, both=True)
    return hub.cut(_lap_cut(s))


def _shank_solid(s: int) -> cq.Workplane:
    """Steel handle tang sweeping back from the hub into the grip."""
    pts = [(x, s * y) for x, y in TANG_PTS]
    loop = _strip_loop(pts, TANG_HALF_W)
    shank = cq.Workplane("XY").polyline(loop).close().extrude(HALF_T, both=True)
    return shank.cut(_lap_cut(s))


def _ellipse_wire(x: float, yc: float, zc: float, w: float, h: float) -> cq.Wire:
    wp = cq.Workplane("YZ", origin=(x, 0.0, 0.0)).center(yc, zc).ellipse(w, h)
    return wp.val()


def _grip_solid(s: int) -> cq.Solid:
    """Soft-touch rubber grip: flared guard, curved shaft, bulbous end."""
    wires = [
        _ellipse_wire(x, s * _yc(x), 0.0, w, h)
        for x, w, h in GRIP_SECTIONS
    ]
    return cq.Solid.makeLoft(wires, ruled=False)


def _inlay_solid(s: int) -> cq.Solid:
    """Glossy red inlay loft running along the outer/top face of the grip."""
    wires = [
        _ellipse_wire(x, s * _yc(x), INLAY_Z_CENTER, _grip_w(x) - 0.0012, INLAY_HALF_H)
        for x in INLAY_XS
    ]
    return cq.Solid.makeLoft(wires, ruled=False)


# ---- tongue-and-groove geometry helpers ----

def _groove_step_solid() -> cq.Workplane:
    """One groove step: a visible rectangular channel on the lower shank face.

    Shared geometry helper used by the groove track for-loop.
    """
    return cq.Workplane("XY").box(GROOVE_W, GROOVE_L, GROOVE_D)


def _tongue_solid() -> cq.Workplane:
    """Tongue protrusion on the pivot carrier that indexes into the groove."""
    return cq.Workplane("XY").box(TONGUE_W, TONGUE_L, TONGUE_H)


def _button_solid() -> cq.Workplane:
    """Visible push-button on the pivot carrier outer face."""
    button = (
        cq.Workplane("XY")
        .circle(BUTTON_R)
        .extrude(BUTTON_H)
    )
    # Add a small chamfer ring around the button base for visual detail
    ring = (
        cq.Workplane("XY")
        .circle(BUTTON_R + 0.001)
        .circle(BUTTON_R)
        .extrude(0.001)
    )
    return button.union(ring)


def _carrier_link_solid() -> cq.Workplane:
    """Small connecting link between button and tongue (passes through upper lap)."""
    return (
        cq.Workplane("XY")
        .circle(0.003)
        .extrude(HALF_T + 0.002)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="channel_lock_pliers")

    steel_polished = model.material("steel_polished", rgba=(0.80, 0.81, 0.84, 1.0))
    steel_forged = model.material("steel_forged", rgba=(0.66, 0.67, 0.70, 1.0))
    steel_brushed = model.material("steel_brushed", rgba=(0.61, 0.62, 0.65, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.35, 0.36, 0.38, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.09, 1.0))
    grip_red = model.material("grip_red", rgba=(0.80, 0.09, 0.11, 1.0))
    button_orange = model.material("button_orange", rgba=(0.90, 0.45, 0.10, 1.0))

    # ---- lower half (root, s=+1, jaw on +Y) ----
    lower = model.part("lower_half")

    lower.visual(
        mesh_from_cadquery(_jaw_solid(1), "lower_jaw", tolerance=0.0002),
        name="jaw",
        material=steel_polished,
    )
    lower.visual(
        mesh_from_cadquery(_hub_solid(1), "lower_hub", tolerance=0.0002),
        name="hub",
        material=steel_forged,
    )
    lower.visual(
        mesh_from_cadquery(_shank_solid(1), "lower_shank", tolerance=0.0002),
        name="shank",
        material=steel_forged,
    )
    lower.visual(
        mesh_from_cadquery(_grip_solid(1), "lower_grip", tolerance=0.0002),
        name="grip",
        material=rubber_black,
    )
    lower.visual(
        mesh_from_cadquery(_inlay_solid(1), "lower_inlay", tolerance=0.0002),
        name="grip_inlay",
        material=grip_red,
    )

    # Groove track: 5 stepped grooves along the lower shank
    for i in range(GROOVE_COUNT):
        x_pos = GROOVE_X_START + i * GROOVE_SPACING
        lower.visual(
            mesh_from_cadquery(_groove_step_solid(), f"groove_{i}", tolerance=0.0002),
            origin=Origin(xyz=(x_pos, GROOVE_TRACK_Y, -EPS - GROOVE_D / 2.0)),
            name=f"groove_{i}",
            material=steel_dark,
        )

    # ---- pivot carrier (tongue + button, slides in groove track) ----
    carrier = model.part("pivot_carrier")

    carrier.visual(
        mesh_from_cadquery(_tongue_solid(), "carrier_tongue", tolerance=0.0002),
        origin=Origin(xyz=(0.0, 0.0, -TONGUE_H / 2.0)),
        name="tongue",
        material=steel_forged,
    )
    carrier.visual(
        mesh_from_cadquery(_carrier_link_solid(), "carrier_link", tolerance=0.0002),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="link",
        material=steel_brushed,
    )
    carrier.visual(
        mesh_from_cadquery(_button_solid(), "carrier_button", tolerance=0.0002),
        origin=Origin(xyz=(0.0, 0.0, HALF_T + 0.002)),
        name="button",
        material=button_orange,
    )

    # ---- upper half (s=-1, jaw on -Y) ----
    upper = model.part("upper_half")

    upper.visual(
        mesh_from_cadquery(_jaw_solid(-1), "upper_jaw", tolerance=0.0002),
        name="jaw",
        material=steel_polished,
    )
    upper.visual(
        mesh_from_cadquery(_hub_solid(-1), "upper_hub", tolerance=0.0002),
        name="hub",
        material=steel_forged,
    )
    upper.visual(
        mesh_from_cadquery(_shank_solid(-1), "upper_shank", tolerance=0.0002),
        name="shank",
        material=steel_forged,
    )
    upper.visual(
        mesh_from_cadquery(_grip_solid(-1), "upper_grip", tolerance=0.0002),
        name="grip",
        material=rubber_black,
    )
    upper.visual(
        mesh_from_cadquery(_inlay_solid(-1), "upper_inlay", tolerance=0.0002),
        name="grip_inlay",
        material=grip_red,
    )

    # ---- articulations ----

    # PRISMATIC: groove selection — slides the pivot carrier along the track
    # Origin at groove_0 position on the lower half shank.
    # Axis along -X so positive q slides toward groove_4 (wider jaw setting).
    model.articulation(
        "groove_select",
        ArticulationType.PRISMATIC,
        parent=lower,
        child=carrier,
        origin=Origin(xyz=(GROOVE_X_START, GROOVE_TRACK_Y, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0, velocity=0.10,
            lower=0.0, upper=PRISMATIC_TRAVEL,
        ),
    )

    # REVOLUTE: jaw open/close — rotates the upper half about the tongue
    # Axis along -Z so positive q swings the upper jaw toward -Y (opening).
    model.articulation(
        "jaw_pivot",
        ArticulationType.REVOLUTE,
        parent=carrier,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=80.0, velocity=3.0,
            lower=0.0, upper=OPEN_LIMIT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower = object_model.get_part("lower_half")
    carrier = object_model.get_part("pivot_carrier")
    upper = object_model.get_part("upper_half")
    groove_select = object_model.get_articulation("groove_select")
    jaw_pivot = object_model.get_articulation("jaw_pivot")

    jaw_lower = lower.get_visual("jaw")
    jaw_upper = upper.get_visual("jaw")
    hub_lower = lower.get_visual("hub")
    hub_upper = upper.get_visual("hub")

    # ---- mechanism checks ----

    # Groove select is a prismatic joint with correct travel
    gs_limits = groove_select.motion_limits
    ctx.check(
        "groove_select is prismatic with 0..16mm travel",
        groove_select.articulation_type == ArticulationType.PRISMATIC
        and gs_limits is not None
        and gs_limits.lower is not None
        and gs_limits.upper is not None
        and abs(gs_limits.lower) < 1e-9
        and abs(gs_limits.upper - PRISMATIC_TRAVEL) < 1e-6,
        details=f"limits={gs_limits}",
    )

    # Jaw pivot is a revolute joint with 0..30 degree range
    jp_limits = jaw_pivot.motion_limits
    ctx.check(
        "jaw_pivot is a 0..30 degree revolute joint",
        jaw_pivot.articulation_type == ArticulationType.REVOLUTE
        and jp_limits is not None
        and jp_limits.lower is not None
        and jp_limits.upper is not None
        and abs(jp_limits.lower) < 1e-9
        and abs(jp_limits.upper - OPEN_LIMIT) < 1e-6,
        details=f"limits={jp_limits}",
    )

    # ---- groove track visibility ----

    # All 5 grooves exist as named visuals on the lower half
    groove_names = [f"groove_{i}" for i in range(GROOVE_COUNT)]
    for gname in groove_names:
        ctx.check(
            f"groove visual {gname} exists on lower_half",
            lower.get_visual(gname) is not None,
            details=f"missing visual {gname}",
        )

    # Grooves are evenly spaced along -X
    groove_aabbs = []
    for gname in groove_names:
        aabb = ctx.part_element_world_aabb(lower, elem=gname)
        groove_aabbs.append(aabb)
    if all(a is not None for a in groove_aabbs):
        centers_x = [0.5 * (a[0][0] + a[1][0]) for a in groove_aabbs]
        spacings = [centers_x[i + 1] - centers_x[i] for i in range(len(centers_x) - 1)]
        avg_spacing = sum(spacings) / len(spacings) if spacings else 0.0
        ctx.check(
            "grooves evenly spaced along X at ~4mm",
            all(abs(s - avg_spacing) < 0.001 for s in spacings)
            and 0.003 <= abs(avg_spacing) <= 0.005,
            details=f"spacings={[f'{s:.4f}' for s in spacings]}",
        )
    else:
        ctx.fail("groove AABBs resolve", "one or more groove AABBs missing")

    # ---- rest pose checks (groove_select at q=0, jaw_pivot at q=0) ----

    # Jaws are present and roughly aligned
    ctx.expect_overlap(
        lower,
        upper,
        axes="x",
        elem_a=jaw_lower,
        elem_b=jaw_upper,
        min_overlap=0.04,
        name="jaws overlap in X at rest (gripping zone)",
    )

    # The halves interleave at the engagement zone (half-lap stacking)
    ctx.expect_gap(
        upper,
        lower,
        axis="z",
        positive_elem=hub_upper,
        negative_elem=hub_lower,
        min_gap=0.0,
        max_gap=0.001,
        name="upper hub lap stacks above lower hub lap",
    )
    ctx.expect_contact(
        lower,
        upper,
        elem_a=hub_lower,
        elem_b=hub_upper,
        contact_tol=0.0005,
        name="hub laps seat against each other at the engagement zone",
    )

    # Pivot carrier tongue is in the groove track
    ctx.expect_overlap(
        carrier,
        lower,
        axes="xy",
        elem_a="tongue",
        elem_b="groove_0",
        min_overlap=0.001,
        name="tongue overlaps groove_0 at rest position",
    )

    # Half-lap interleaving: the lower hub and upper jaw are intentionally
    # interleaved at the engagement zone. Small mesh/boolean artifacts at the
    # lap boundary can register as overlap in the lower-lap zone.
    ctx.allow_overlap(
        lower,
        upper,
        elem_a="hub",
        elem_b="jaw",
        reason="The lower hub and upper jaw interleave at the half-lap "
        "engagement zone; small mesh artifacts at the lap boundary may "
        "register as overlap in the lower-lap zone.",
    )
    ctx.expect_gap(
        upper,
        lower,
        axis="z",
        positive_elem=hub_upper,
        negative_elem=hub_lower,
        min_gap=0.0,
        max_gap=0.001,
        name="hub laps correctly stack with upper above lower",
    )

    # The carrier link passes through the upper hub (intentional capture)
    ctx.allow_overlap(
        carrier,
        upper,
        elem_a="link",
        elem_b=hub_upper,
        reason="The carrier link intentionally passes through the upper hub "
        "lap, connecting the button to the tongue through the half-lap joint.",
    )
    ctx.expect_within(
        carrier,
        upper,
        axes="xy",
        inner_elem="link",
        outer_elem=hub_upper,
        margin=0.002,
        name="carrier link stays within upper hub footprint",
    )

    # The tongue sits in the lower half groove track (intentional nesting)
    ctx.allow_overlap(
        carrier,
        lower,
        elem_a="tongue",
        elem_b="shank",
        reason="The tongue intentionally indexes into the groove channel "
        "on the lower shank, representing the tongue-and-groove engagement.",
    )
    ctx.expect_contact(
        carrier,
        lower,
        elem_a="tongue",
        elem_b="shank",
        contact_tol=0.002,
        name="tongue seats against the lower shank groove surface",
    )

    # Button is visible above the upper surface
    btn_aabb = ctx.part_element_world_aabb(carrier, elem="button")
    upper_aabb = ctx.part_world_aabb(upper)
    if btn_aabb is not None and upper_aabb is not None:
        ctx.check(
            "push button proud above upper half surface",
            btn_aabb[1][2] >= upper_aabb[1][2] - 0.001,
            details=f"button_top={btn_aabb[1][2]:.4f} upper_top={upper_aabb[1][2]:.4f}",
        )

    # ---- prismatic motion check ----

    # At max travel, carrier has moved along -X
    rest_carrier_pos = ctx.part_world_position(carrier)
    with ctx.pose({groove_select: PRISMATIC_TRAVEL}):
        extended_carrier_pos = ctx.part_world_position(carrier)
        ctx.check(
            "carrier slides toward +X at max prismatic travel",
            rest_carrier_pos is not None
            and extended_carrier_pos is not None
            and extended_carrier_pos[0] > rest_carrier_pos[0] + 0.010,
            details=f"rest={rest_carrier_pos}, extended={extended_carrier_pos}",
        )

        # At extended position, tongue overlaps groove_4
        ctx.expect_overlap(
            carrier,
            lower,
            axes="xy",
            elem_a="tongue",
            elem_b="groove_4",
            min_overlap=0.001,
            name="tongue overlaps groove_4 at max travel",
        )

    # ---- revolute jaw motion check ----

    # Decisive open pose: jaws separate when revolute opens
    closed_jaw_upper_aabb = ctx.part_element_world_aabb(upper, elem="jaw")
    with ctx.pose({jaw_pivot: OPEN_LIMIT}):
        open_jaw_upper_aabb = ctx.part_element_world_aabb(upper, elem="jaw")
        ctx.check(
            "upper jaw swings away at the 30 degree open pose",
            closed_jaw_upper_aabb is not None
            and open_jaw_upper_aabb is not None
            and open_jaw_upper_aabb[0][1] < closed_jaw_upper_aabb[0][1] - 0.010,
            details=f"closed_min_y={closed_jaw_upper_aabb}, open_min_y={open_jaw_upper_aabb}",
        )

    # ---- overall envelope ----
    a_lower = ctx.part_world_aabb(lower)
    a_upper = ctx.part_world_aabb(upper)
    g_lower = ctx.part_element_world_aabb(lower, elem="grip")
    g_upper = ctx.part_element_world_aabb(upper, elem="grip")
    ok_env = (a_lower is not None and a_upper is not None
              and g_lower is not None and g_upper is not None)
    if ok_env:
        length = max(a_lower[1][0], a_upper[1][0]) - min(a_lower[0][0], a_upper[0][0])
        ctx.check(
            "overall length about 0.20 m",
            0.18 <= length <= 0.22,
            details=f"length={length:.4f}",
        )
    else:
        ctx.fail("envelope AABBs resolve", "missing part/grip AABB")

    # ---- red inlay checks ----
    i_lower = ctx.part_element_world_aabb(lower, elem="grip_inlay")
    if i_lower is not None and g_lower is not None:
        ctx.check(
            "red inlay proud on the lower grip top face",
            i_lower[1][2] >= g_lower[1][2] - 0.0005,
            details=f"inlay_top={i_lower[1][2]:.4f} grip_top={g_lower[1][2]:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
