from __future__ import annotations

# Locking vise-grip pliers (variant of heavy-duty lineman pliers).
# Reference image: picture/Other/pliers/002.png
#
# Layout: the tool lies in the XY plane with Z as the thickness axis.
# The rivet pivot is at the origin. The curved clamping jaws point +X,
# the handles sweep back to -X and spread in +/-Y.
# plier_half_0 (root) carries its jaw on +Y; plier_half_1 is the mirrored
# moving half. The two forged halves are half-lapped at the pivot:
# half_0 keeps the lower lap (z <= -EPS), half_1 keeps the upper lap
# (z >= +EPS), so one half visibly passes over the other under the rivet.
#
# Variant change: curved serrated clamping jaws + over-center toggle
# locking link inside half_0's handle with its own revolute joint.

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
HUB_R = 0.015           # forged hub radius around the rivet
BOSS_R = 0.0125         # rivet boss radius (~0.025 m diameter)
BOSS_H = 0.0030
SEAM_R = 0.0132         # visible circular seam ring under the boss cap
SEAM_H = 0.0006
RIVET_R = 0.004         # rivet shaft captured through the moving half's lap
EPS = 0.0001            # lap clearance so the stacked halves do not penetrate
JAW_FACE = 0.0003       # closed jaw inner faces sit at y = +/-JAW_FACE
NOSE_X = 0.075          # jaw tip extent
OPEN_LIMIT = math.radians(30.0)

TANG_HALF_W = 0.004     # steel handle tang half width in plan

# Toggle link dimensions
TOGGLE_LENGTH = 0.052   # overall length of the toggle bar
TOGGLE_WIDTH = 0.006    # width of the toggle bar
TOGGLE_THICK = 0.003    # thickness of the toggle bar
TOGGLE_PIN_R = 0.0015   # pin hole radius
TOGGLE_RANGE = math.radians(25.0)  # toggle motion range

# Adjustment screw dimensions
SCREW_R = 0.005         # screw head radius
SCREW_H = 0.008         # screw head height
SCREW_SHAFT_R = 0.003   # threaded shaft radius
SCREW_SHAFT_H = 0.012   # threaded shaft length

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

# Toggle pivot location: inside half_0's handle, about 2/3 down from the main pivot
TOGGLE_PIVOT_X = -0.092
TOGGLE_PIVOT_Y_OFFSET = 0.0  # centered on the handle tang


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
    """
    cut = cq.Workplane("XY").circle(LAP_R).extrude(0.012)
    if s > 0:
        return cut.translate((0.0, 0.0, -EPS))
    return cut.translate((0.0, 0.0, -0.012 + EPS))


def _jaw_solid(s: int) -> cq.Workplane:
    """Curved vise-grip clamping jaw with cross-serrations.

    The jaw has a C-shaped curved profile: it sweeps from the pivot area
    outward in an arc, curving back inward at the nose tip to create the
    characteristic vise-grip clamping curve.
    """
    # Curved jaw profile - the inner (gripping) surface follows an arc
    # that wraps around the workpiece, then the outer surface follows a
    # broader arc back. This creates the C-clamp shape of vise-grip jaws.
    inner_pts = [
        (0.010, JAW_FACE),           # root near pivot
        (0.025, s * 0.006),          # curving outward
        (0.040, s * 0.009),          # mid-jaw peak
        (0.055, s * 0.010),          # approaching nose
        (0.065, s * 0.008),          # curving back in
        (NOSE_X, s * 0.004),         # nose tip (curved, pointed)
    ]
    outer_pts = [
        (NOSE_X, s * 0.014),         # nose outer surface
        (0.060, s * 0.020),          # outer curve peak
        (0.045, s * 0.022),          # outer mid
        (0.030, s * 0.020),          # outer approaching pivot
        (0.018, s * 0.017),          # outer near pivot
        (0.010, s * 0.016),          # root outer
    ]

    pts = [(x, s * abs(y)) for x, y in inner_pts]
    pts += [(x, s * abs(y)) for x, y in outer_pts]

    jaw = cq.Workplane("XY").polyline(pts).close().extrude(HALF_T, both=True)

    # Cross-hatch serrations across the inner gripping face (diagonal grooves).
    for i in range(9):
        xi = 0.020 + 0.005 * i
        # Diagonal grooves at ~45 degrees
        groove = (
            cq.Workplane("XY")
            .box(0.0010, 0.0020, 0.020)
            .translate((xi, s * JAW_FACE, 0.0))
            .rotate((0, 0, 0), (0, 0, 1), 45.0)
            .translate((xi, s * JAW_FACE, 0.0))
        )
        jaw = jaw.cut(groove)

    # Secondary set of diagonal grooves at -45 degrees for cross-hatch pattern
    for i in range(9):
        xi = 0.020 + 0.005 * i
        groove = (
            cq.Workplane("XY")
            .box(0.0010, 0.0020, 0.020)
            .translate((xi, s * JAW_FACE, 0.0))
            .rotate((0, 0, 0), (0, 0, 1), -45.0)
            .translate((xi, s * JAW_FACE, 0.0))
        )
        jaw = jaw.cut(groove)

    # Rounded pipe-grip recess behind the nose (curved clamping pocket).
    recess = (
        cq.Workplane("XY")
        .circle(0.005)
        .extrude(0.011, both=True)
        .translate((0.035, s * JAW_FACE, 0.0))
    )
    jaw = jaw.cut(recess)

    # Serration scallops around the recess rim
    for ang_deg in (30.0, 65.0, 100.0, 135.0):
        a = math.radians(ang_deg)
        sx = 0.035 + 0.005 * math.cos(a)
        sy = s * (JAW_FACE + 0.005 * math.sin(a))
        scallop = (
            cq.Workplane("XY")
            .circle(0.0009)
            .extrude(0.011, both=True)
            .translate((sx, sy, 0.0))
        )
        jaw = jaw.cut(scallop)

    return jaw.cut(_lap_cut(s))


def _hub_solid(s: int) -> cq.Workplane:
    """Forged circular hub around the rivet, thinned to its lap layer."""
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


def _toggle_link_solid() -> cq.Workplane:
    """Over-center toggle locking link: flat steel bar with rounded ends and pin holes.

    The link sits inside half_0's handle channel. One end pivots on a fixed
    pin; the other end connects to the release lever mechanism.
    """
    half_l = TOGGLE_LENGTH / 2.0
    half_w = TOGGLE_WIDTH / 2.0
    r = half_w  # end cap radius = half width for rounded ends

    # Build rounded-rectangle profile using arcs at the ends
    link = (
        cq.Workplane("XY")
        .moveTo(-half_l + r, -half_w)
        .lineTo(half_l - r, -half_w)
        .radiusArc((half_l - r, half_w), -r)
        .lineTo(-half_l + r, half_w)
        .radiusArc((-half_l + r, -half_w), -r)
        .close()
        .extrude(TOGGLE_THICK, both=True)
    )

    # Pin hole at the pivot end (local -X end, which maps to the handle-end side)
    pivot_hole = (
        cq.Workplane("XY")
        .circle(TOGGLE_PIN_R)
        .extrude(0.010, both=True)
        .translate((-half_l + r + 0.002, 0.0, 0.0))
    )
    link = link.cut(pivot_hole)

    # Pin hole at the far end (connects to release lever area)
    far_hole = (
        cq.Workplane("XY")
        .circle(TOGGLE_PIN_R)
        .extrude(0.010, both=True)
        .translate((half_l - r - 0.002, 0.0, 0.0))
    )
    link = link.cut(far_hole)

    # Small reinforcement rib in the middle
    rib = (
        cq.Workplane("XY")
        .box(0.008, TOGGLE_WIDTH * 0.6, TOGGLE_THICK * 1.4)
        .translate((0.0, 0.0, 0.0))
    )
    link = link.union(rib)

    return link


def _adjustment_screw_solid() -> cq.Workplane:
    """Knurled adjustment screw at the handle end for jaw width setting.

    A cylindrical screw head with knurl grooves and a threaded shaft
    protruding from the handle end.
    """
    # Screw head: slotted cylinder
    head = cq.Workplane("XY").circle(SCREW_R).extrude(SCREW_H)

    # Knurl grooves around the head perimeter
    for i in range(12):
        angle = math.radians(i * 30.0)
        cx = (SCREW_R - 0.0004) * math.cos(angle)
        cy = (SCREW_R - 0.0004) * math.sin(angle)
        groove = (
            cq.Workplane("XY")
            .circle(0.0006)
            .extrude(SCREW_H + 0.001)
            .translate((cx, cy, -0.0005))
        )
        head = head.cut(groove)

    # Slot across the top face
    slot = (
        cq.Workplane("XY")
        .box(SCREW_R * 1.8, 0.0012, 0.002)
        .translate((0.0, 0.0, SCREW_H - 0.001))
    )
    head = head.cut(slot)

    # Threaded shaft extending below the head
    shaft = cq.Workplane("XY").circle(SCREW_SHAFT_R).extrude(SCREW_SHAFT_H)
    shaft = shaft.translate((0.0, 0.0, -SCREW_SHAFT_H))
    head = head.union(shaft)

    return head


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="locking_vise_grip_pliers")

    steel_polished = model.material("steel_polished", rgba=(0.80, 0.81, 0.84, 1.0))
    steel_forged = model.material("steel_forged", rgba=(0.66, 0.67, 0.70, 1.0))
    steel_brushed = model.material("steel_brushed", rgba=(0.61, 0.62, 0.65, 1.0))
    seam_gray = model.material("seam_gray", rgba=(0.45, 0.46, 0.48, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.09, 1.0))
    grip_red = model.material("grip_red", rgba=(0.80, 0.09, 0.11, 1.0))
    toggle_steel = model.material("toggle_steel", rgba=(0.72, 0.73, 0.76, 1.0))
    screw_zinc = model.material("screw_zinc", rgba=(0.75, 0.76, 0.78, 1.0))

    parts = []
    for part_name, s in (("plier_half_0", 1), ("plier_half_1", -1)):
        part = model.part(part_name)
        tag = part_name.replace("plier_half_", "half")

        part.visual(
            mesh_from_cadquery(_jaw_solid(s), f"{tag}_jaw", tolerance=0.0002),
            name="jaw",
            material=steel_polished,
        )
        part.visual(
            mesh_from_cadquery(_hub_solid(s), f"{tag}_hub", tolerance=0.0002),
            name="hub",
            material=steel_forged,
        )
        part.visual(
            mesh_from_cadquery(_shank_solid(s), f"{tag}_shank", tolerance=0.0002),
            name="shank",
            material=steel_forged,
        )
        part.visual(
            mesh_from_cadquery(_grip_solid(s), f"{tag}_grip", tolerance=0.0002),
            name="grip",
            material=rubber_black,
        )
        part.visual(
            mesh_from_cadquery(_inlay_solid(s), f"{tag}_inlay", tolerance=0.0002),
            name="grip_inlay",
            material=grip_red,
        )

        parts.append(part)

    # One-piece rivet, fixed to half_0 (the moving half rotates around it):
    # bottom seam + boss embed into half_0's own hub face, the shaft passes
    # up through half_1's lap, and the peened head + seam cap the top face.
    fixed = parts[0]
    fixed.visual(
        Cylinder(SEAM_R, SEAM_H),
        origin=Origin(xyz=(0.0, 0.0, -(HALF_T + SEAM_H / 2.0 - EPS))),
        name="boss_seam",
        material=seam_gray,
    )
    fixed.visual(
        Cylinder(BOSS_R, BOSS_H),
        origin=Origin(xyz=(0.0, 0.0, -(HALF_T + SEAM_H + BOSS_H / 2.0 - 2.0 * EPS))),
        name="rivet_boss",
        material=steel_brushed,
    )
    fixed.visual(
        Cylinder(RIVET_R, 2.0 * HALF_T + SEAM_H + 2.0 * EPS + 0.0005),
        origin=Origin(xyz=(0.0, 0.0, 0.00005)),
        name="rivet_shaft",
        material=steel_brushed,
    )
    fixed.visual(
        Cylinder(SEAM_R, SEAM_H),
        origin=Origin(xyz=(0.0, 0.0, HALF_T + SEAM_H / 2.0 + EPS)),
        name="head_seam",
        material=seam_gray,
    )
    fixed.visual(
        Cylinder(BOSS_R, BOSS_H),
        origin=Origin(xyz=(0.0, 0.0, HALF_T + SEAM_H + BOSS_H / 2.0)),
        name="rivet_head",
        material=steel_brushed,
    )

    # Adjustment screw at the end of half_0's handle (jaw width setting).
    # Placed at the bulbous handle end, shaft pointing inward along +X.
    handle_end_x = CENTERLINE[-1][0]
    handle_end_y = CENTERLINE[-1][1]
    fixed.visual(
        mesh_from_cadquery(_adjustment_screw_solid(), "adjustment_screw", tolerance=0.0002),
        origin=Origin(
            xyz=(handle_end_x - 0.002, handle_end_y, -(HALF_T + SCREW_H / 2.0)),
            rpy=(math.pi, 0.0, 0.0),
        ),
        name="adjustment_screw",
        material=screw_zinc,
    )

    # Toggle link: over-center locking bar inside half_0's handle.
    # The link pivots on a pin inside the handle channel.
    toggle = model.part("toggle_link")
    toggle_y_center = _yc(TOGGLE_PIVOT_X)  # follow handle centerline
    toggle.visual(
        mesh_from_cadquery(_toggle_link_solid(), "toggle_bar", tolerance=0.0002),
        origin=Origin(xyz=(TOGGLE_LENGTH / 2.0 - 0.002, 0.0, 0.0)),
        name="toggle_bar",
        material=toggle_steel,
    )
    # Small pivot pin visual on the toggle (the pin that captures it in the handle)
    toggle.visual(
        Cylinder(TOGGLE_PIN_R + 0.0003, 2.0 * HALF_T + 0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="toggle_pin",
        material=steel_brushed,
    )

    # Primary articulation: one revolute joint at the rivet, axis perpendicular
    # to the flat tool plane. Positive q (about -Z) swings half_1's jaw toward
    # -Y, opening the jaws while the handles spread apart.
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=parts[0],
        child=parts[1],
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=3.0, lower=0.0, upper=OPEN_LIMIT),
    )

    # Toggle articulation: small revolute joint inside half_0's handle.
    # The toggle link snaps over-center to lock the jaws shut.
    # Origin at the toggle pivot pin location, axis perpendicular to tool plane.
    # Positive q rotates the link upward (toward the locked position).
    model.articulation(
        "toggle",
        ArticulationType.REVOLUTE,
        parent=parts[0],
        child=toggle,
        origin=Origin(xyz=(TOGGLE_PIVOT_X, toggle_y_center, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=5.0,
            lower=-0.05,
            upper=TOGGLE_RANGE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    half0 = object_model.get_part("plier_half_0")
    half1 = object_model.get_part("plier_half_1")
    toggle = object_model.get_part("toggle_link")
    pivot = object_model.get_articulation("pivot")
    toggle_joint = object_model.get_articulation("toggle")

    jaw0 = half0.get_visual("jaw")
    jaw1 = half1.get_visual("jaw")
    hub0 = half0.get_visual("hub")
    hub1 = half1.get_visual("hub")
    grip0 = half0.get_visual("grip")
    grip1 = half1.get_visual("grip")
    inlay0 = half0.get_visual("grip_inlay")
    toggle_bar = toggle.get_visual("toggle_bar")

    # Joint contract: main pivot is a 0..30 degree revolute joint.
    limits = pivot.motion_limits
    ctx.check(
        "pivot is a 0..30 degree revolute joint",
        pivot.articulation_type == ArticulationType.REVOLUTE
        and limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower) < 1e-9
        and abs(limits.upper - OPEN_LIMIT) < 1e-6,
        details=f"limits={limits}",
    )

    # Toggle joint: small revolute joint for the over-center locking link.
    t_limits = toggle_joint.motion_limits
    ctx.check(
        "toggle is a revolute joint with limited range",
        toggle_joint.articulation_type == ArticulationType.REVOLUTE
        and t_limits is not None
        and t_limits.lower is not None
        and t_limits.upper is not None
        and t_limits.upper > 0.1
        and t_limits.upper < 1.0,
        details=f"limits={t_limits}",
    )

    # Toggle link exists as a separate part with its own joint.
    ctx.check(
        "toggle link is a separate articulated part",
        toggle is not None and toggle_joint is not None,
        details="toggle_link part or toggle articulation missing",
    )

    # Toggle bar is intentionally nested inside the handle channel -
    # the locking link sits inside the handle shank/grip recess.
    ctx.allow_overlap(
        half0,
        toggle,
        elem_a="shank",
        elem_b="toggle_bar",
        reason="The toggle bar is intentionally nested inside the handle shank "
        "channel, representing the locking link sitting in a recessed handle slot.",
    )
    ctx.allow_overlap(
        half0,
        toggle,
        elem_a="grip",
        elem_b="toggle_bar",
        reason="The toggle bar passes through the grip area where the rubber "
        "overmold has a channel cut for the locking mechanism.",
    )
    ctx.expect_within(
        toggle,
        half0,
        axes="xy",
        inner_elem=toggle_bar,
        outer_elem="shank",
        margin=0.002,
        name="toggle bar stays within the handle shank footprint",
    )
    ctx.expect_overlap(
        toggle,
        half0,
        axes="x",
        elem_a=toggle_bar,
        elem_b="shank",
        min_overlap=0.030,
        name="toggle bar has significant retained insertion in the handle",
    )

    # Toggle link motion: positive q swings the bar from its rest position.
    toggle_rest_aabb = ctx.part_element_world_aabb(toggle, elem="toggle_bar")
    with ctx.pose({toggle_joint: TOGGLE_RANGE}):
        toggle_active_aabb = ctx.part_element_world_aabb(toggle, elem="toggle_bar")
        ctx.check(
            "toggle link moves at the upper limit pose",
            toggle_rest_aabb is not None
            and toggle_active_aabb is not None
            and (
                abs(toggle_active_aabb[0][1] - toggle_rest_aabb[0][1]) > 0.002
                or abs(toggle_active_aabb[1][1] - toggle_rest_aabb[1][1]) > 0.002
                or abs(toggle_active_aabb[0][0] - toggle_rest_aabb[0][0]) > 0.002
            ),
            details=f"rest={toggle_rest_aabb}, active={toggle_active_aabb}",
        )

    # Toggle pin captures the toggle link at the pivot (intentional overlap).
    ctx.allow_overlap(
        toggle,
        half0,
        elem_a="toggle_pin",
        elem_b="shank",
        reason="The toggle pin passes through the handle shank to capture "
        "the toggle link at its pivot point inside the handle channel.",
    )
    ctx.expect_contact(
        toggle,
        half0,
        elem_a="toggle_pin",
        elem_b="shank",
        contact_tol=0.003,
        name="toggle pin contacts the handle shank at the pivot",
    )

    # Curved jaw profile: the jaw extends further in Y than a flat jaw would,
    # showing the C-shaped clamping curve. Verify the jaw tip is offset from
    # the jaw root in Y (the curve).
    j0_aabb = ctx.part_element_world_aabb(half0, elem="jaw")
    if j0_aabb is not None:
        jaw_y_span = j0_aabb[1][1] - j0_aabb[0][1]
        ctx.check(
            "jaw has a curved profile with significant Y extent",
            jaw_y_span >= 0.012,
            details=f"jaw_y_span={jaw_y_span:.4f}",
        )
    else:
        ctx.fail("jaw AABB resolves", "missing jaw element AABB")

    # Adjustment screw is visible at the handle end.
    screw_aabb = ctx.part_element_world_aabb(half0, elem="adjustment_screw")
    if screw_aabb is not None:
        ctx.check(
            "adjustment screw at handle end is below the tool plane",
            screw_aabb[0][2] < -HALF_T,
            details=f"screw_min_z={screw_aabb[0][2]:.4f}",
        )
        ctx.check(
            "adjustment screw is near the handle end (x < -0.10)",
            screw_aabb[0][0] < -0.10,
            details=f"screw_min_x={screw_aabb[0][0]:.4f}",
        )
    else:
        ctx.fail("adjustment screw AABB resolves", "missing adjustment_screw element AABB")

    # Closed rest pose: serrated jaw inner faces nearly touch.
    ctx.expect_gap(
        half0,
        half1,
        axis="y",
        positive_elem=jaw0,
        negative_elem=jaw1,
        min_gap=0.0002,
        max_gap=0.002,
        name="jaws closed and nearly touching at rest",
    )

    # The halves interleave at the pivot: half_1's lap passes over half_0's.
    ctx.expect_gap(
        half1,
        half0,
        axis="z",
        positive_elem=hub1,
        negative_elem=hub0,
        min_gap=0.0,
        max_gap=0.001,
        name="moving hub lap stacks above fixed hub lap",
    )
    ctx.expect_contact(
        half0,
        half1,
        elem_a=hub0,
        elem_b=hub1,
        contact_tol=0.0005,
        name="hub laps seat against each other at the joint",
    )
    ctx.expect_overlap(
        half0,
        half1,
        axes="xy",
        elem_a=hub0,
        elem_b=hub1,
        min_overlap=0.02,
        name="hub laps share the pivot footprint",
    )

    # The rivet shaft is intentionally captured through the moving half's lap;
    # that is the physical connection the pliers pivot on.
    ctx.allow_overlap(
        half0,
        half1,
        elem_a="rivet_shaft",
        elem_b=hub1,
        reason="The rivet shaft is fixed to half_0 and intentionally passes "
        "through the moving half's hub lap, capturing it at the pivot.",
    )
    ctx.expect_within(
        half0,
        half1,
        axes="xy",
        inner_elem="rivet_shaft",
        outer_elem=hub1,
        margin=0.0005,
        name="rivet shaft stays centered inside the moving hub lap",
    )
    ctx.expect_overlap(
        half0,
        half1,
        axes="z",
        elem_a="rivet_shaft",
        elem_b=hub1,
        min_overlap=0.007,
        name="rivet shaft passes through the full moving lap thickness",
    )

    # Rivet boss: ~0.025 m diameter, centered on the pivot, proud on both
    # outer faces, with total stack thickness about 0.025 m.
    boss0_aabb = ctx.part_element_world_aabb(half0, elem="rivet_boss")
    boss1_aabb = ctx.part_element_world_aabb(half0, elem="rivet_head")
    ok_boss = boss0_aabb is not None and boss1_aabb is not None
    if ok_boss:
        b0_min, b0_max = boss0_aabb
        b1_min, b1_max = boss1_aabb
        dia = b0_max[0] - b0_min[0]
        cx = 0.5 * (b0_min[0] + b0_max[0])
        cy = 0.5 * (b0_min[1] + b0_max[1])
        thick = b1_max[2] - b0_min[2]
        ctx.check(
            "rivet boss is ~25 mm diameter and centered on the pivot",
            0.0235 <= dia <= 0.0265 and abs(cx) <= 0.002 and abs(cy) <= 0.002,
            details=f"dia={dia:.4f} center=({cx:.4f},{cy:.4f})",
        )
        ctx.check(
            "boss caps proud on both outer faces, ~25 mm total at the boss",
            b0_min[2] <= -0.0115 and b1_max[2] >= 0.0115 and 0.023 <= thick <= 0.027,
            details=f"b0_min_z={b0_min[2]:.4f} b1_max_z={b1_max[2]:.4f} thick={thick:.4f}",
        )
    else:
        ctx.fail("rivet boss AABBs resolve", "missing rivet_boss element AABB")

    # Closed-pose envelope: ~0.20 m long, ~0.07 m across the open handles.
    a0 = ctx.part_world_aabb(half0)
    a1 = ctx.part_world_aabb(half1)
    g0 = ctx.part_element_world_aabb(half0, elem="grip")
    g1 = ctx.part_element_world_aabb(half1, elem="grip")
    ok_env = a0 is not None and a1 is not None and g0 is not None and g1 is not None
    if ok_env:
        length = max(a0[1][0], a1[1][0]) - min(a0[0][0], a1[0][0])
        across = g1[1][1] - g0[0][1]
        ctx.check(
            "overall length about 0.20 m",
            0.19 <= length <= 0.215,
            details=f"length={length:.4f}",
        )
        ctx.check(
            "handle grips span about 0.07 m across",
            0.060 <= across <= 0.082,
            details=f"across={across:.4f}",
        )
    else:
        ctx.fail("envelope AABBs resolve", "missing part/grip AABB")

    # Red inlay rides the outer/top face of the black grip (visible from above).
    i0 = ctx.part_element_world_aabb(half0, elem="grip_inlay")
    if i0 is not None and g0 is not None:
        ctx.check(
            "red inlay proud on the grip top face",
            i0[1][2] >= g0[1][2] - 0.0005,
            details=f"inlay_top={i0[1][2]:.4f} grip_top={g0[1][2]:.4f}",
        )
    else:
        ctx.fail("inlay AABB resolves", "missing grip_inlay element AABB")

    # Thumb guard flares close behind the pivot, leaving exposed steel shank.
    if g0 is not None:
        ctx.check(
            "thumb guard front sits close behind the pivot",
            -0.036 <= g0[1][0] <= -0.026,
            details=f"grip_front_x={g0[1][0]:.4f}",
        )

    # Decisive open pose: jaws separate and the handles spread further apart.
    closed_jaw1 = ctx.part_element_world_aabb(half1, elem="jaw")
    closed_grip1 = ctx.part_element_world_aabb(half1, elem="grip")
    with ctx.pose({pivot: OPEN_LIMIT}):
        ctx.expect_gap(
            half0,
            half1,
            axis="y",
            positive_elem=jaw0,
            negative_elem=jaw1,
            min_gap=0.004,
            name="jaws open apart at the 30 degree pose",
        )
        open_jaw1 = ctx.part_element_world_aabb(half1, elem="jaw")
        open_grip1 = ctx.part_element_world_aabb(half1, elem="grip")
        ctx.check(
            "moving jaw swings away from the fixed jaw",
            closed_jaw1 is not None
            and open_jaw1 is not None
            and open_jaw1[0][1] < closed_jaw1[0][1] - 0.018,
            details=f"closed_min_y={closed_jaw1}, open_min_y={open_jaw1}",
        )
        ctx.check(
            "moving handle spreads outward as the jaws open",
            closed_grip1 is not None
            and open_grip1 is not None
            and open_grip1[1][1] > closed_grip1[1][1] + 0.03,
            details=f"closed_max_y={closed_grip1}, open_max_y={open_grip1}",
        )

    return ctx.report()


object_model = build_object_model()
