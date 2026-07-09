from __future__ import annotations

# Compact diagonal cutting pliers (diagonal cutters / dikes).
# Variant of heavy-duty lineman pliers: shorter overall, compact diagonal
# cutting jaws with visible bevel wedges, shorter handles, a locking release
# lever pivoting inside one handle, and circular pivot rivet caps on both sides.
#
# Layout: the tool lies in the XY plane with Z as the thickness axis.
# The rivet pivot is at the origin. The cutting jaws point +X,
# the handles sweep back to -X and spread in +/-Y.
# plier_half_0 (root/fixed) carries its jaw on +Y; plier_half_1 is the
# moving half. The two forged halves are half-lapped at the pivot.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- shared dimensions (meters) ----
HALF_T = 0.008           # half thickness of each forged plate
LAP_R = 0.013            # half-lap joint disc radius at the pivot
HUB_R = 0.012            # forged hub radius around the rivet
BOSS_R = 0.010           # rivet cap radius (~0.020 m diameter)
BOSS_H = 0.0025
SEAM_R = 0.0105          # visible circular seam ring under the cap
SEAM_H = 0.0005
RIVET_R = 0.003          # rivet shaft radius
EPS = 0.0001             # lap clearance
JAW_FACE = 0.0003        # closed jaw inner faces sit at y = +/-JAW_FACE
NOSE_X = 0.028           # compact diagonal cutter tip (short jaws)
OPEN_LIMIT = math.radians(25.0)

# Handle tang: shorter than lineman pliers (~0.11m from pivot to end)
TANG_HALF_W = 0.0035     # steel handle tang half width in plan

# Steel handle tang centerline in the tool plane (jaw-on-+Y half).
TANG_PTS = [
    (-0.008, -0.004),
    (-0.030, -0.011),
    (-0.060, -0.018),
    (-0.090, -0.023),
    (-0.108, -0.025),
]
CENTERLINE = TANG_PTS + [(-0.120, -0.0258)]

# Grip loft stations: (x, half_width_y, half_height_z)
# Shorter grip than lineman pliers
GRIP_SECTIONS = [
    (-0.030, 0.0085, 0.0110),  # flared guard near the pivot
    (-0.040, 0.0072, 0.0100),
    (-0.060, 0.0070, 0.0098),
    (-0.080, 0.0072, 0.0100),
    (-0.100, 0.0076, 0.0105),
    (-0.112, 0.0058, 0.0080),
    (-0.120, 0.0025, 0.0038),
]

# Red inlay stations along the outer/top face
INLAY_XS = [-0.034, -0.052, -0.070, -0.088, -0.105]
INLAY_HALF_H = 0.0065
INLAY_Z_CENTER = 0.005

# Release lever dimensions
LEVER_LENGTH = 0.022
LEVER_WIDTH = 0.006
LEVER_THICK = 0.003
LEVER_PIVOT_X = -0.055   # lever pivot location along the handle (half_0)
LEVER_RANGE = math.radians(15.0)


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
    """Material removed at the pivot so this half keeps only its lap layer."""
    cut = cq.Workplane("XY").circle(LAP_R).extrude(0.010)
    if s > 0:
        return cut.translate((0.0, 0.0, -EPS))
    return cut.translate((0.0, 0.0, -0.010 + EPS))


def _diagonal_jaw_solid(s: int) -> cq.Workplane:
    """Compact diagonal cutting jaw with visible bevel wedge geometry.

    The jaw is short and angled. The cutting edge runs diagonally across
    the inner face. A visible bevel wedge is modeled as material removed
    from the inner face to create the sharp angled cutting edge.
    """
    # Main jaw body: compact profile from hub to short diagonal tip
    profile = [
        (0.008, JAW_FACE),
        (NOSE_X, JAW_FACE),
        (NOSE_X, 0.0080),          # compact squared tip
        (0.024, 0.0100),
        (0.018, 0.0115),
        (0.012, 0.0125),
        (0.008, 0.0120),
    ]
    pts = [(x, s * y) for x, y in profile]
    jaw = cq.Workplane("XY").polyline(pts).close().extrude(HALF_T, both=True)

    # Diagonal cutting edge: a bevel wedge cut into the inner face.
    # The bevel runs at an angle from the tip back toward the pivot,
    # creating the characteristic diagonal cutter geometry.
    # This is a triangular prism removed from the inner face.
    bevel_depth = 0.0025
    bevel_length = 0.016  # how far back the bevel runs

    # Upper bevel wedge (cut from top of inner face)
    bevel_pts_top = [
        (0.010, s * JAW_FACE),
        (0.010 + bevel_length, s * JAW_FACE),
        (0.010 + bevel_length, s * (JAW_FACE + bevel_depth)),
        (0.010, s * (JAW_FACE + 0.0003)),  # nearly flush at start
    ]
    bevel_top = (
        cq.Workplane("XY")
        .polyline(bevel_pts_top)
        .close()
        .extrude(HALF_T * 0.7, both=True)
    )
    jaw = jaw.cut(bevel_top)

    # Lower bevel wedge (cut from bottom of inner face, offset slightly)
    bevel_pts_bot = [
        (0.012, s * JAW_FACE),
        (0.012 + bevel_length - 0.004, s * JAW_FACE),
        (0.012 + bevel_length - 0.004, s * (JAW_FACE + bevel_depth * 0.8)),
        (0.012, s * (JAW_FACE + 0.0005)),
    ]
    bevel_bot = (
        cq.Workplane("XY")
        .polyline(bevel_pts_bot)
        .close()
        .extrude(HALF_T * 0.7, both=True)
    )
    jaw = jaw.cut(bevel_bot)

    # Small V-notch cutter relief near the pivot for wire cutting
    notch_pts = [
        (0.010, s * -0.001),
        (0.014, s * -0.001),
        (0.012, s * 0.002),
    ]
    notch = cq.Workplane("XY").polyline(notch_pts).close().extrude(0.010, both=True)
    jaw = jaw.cut(notch)

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
        _ellipse_wire(x, s * _yc(x), INLAY_Z_CENTER, _grip_w(x) - 0.0010, INLAY_HALF_H)
        for x in INLAY_XS
    ]
    return cq.Solid.makeLoft(wires, ruled=False)


def _lever_slot_solid() -> cq.Workplane:
    """Rectangular slot cut into the grip to house the release lever."""
    # A recess in the handle where the lever sits
    slot = (
        cq.Workplane("XY")
        .box(LEVER_LENGTH + 0.002, LEVER_WIDTH + 0.002, LEVER_THICK + 0.001)
    )
    return slot.translate((LEVER_PIVOT_X + LEVER_LENGTH / 2.0, _yc(LEVER_PIVOT_X + LEVER_LENGTH / 2.0), 0.0))


def _lever_body_solid() -> cq.Workplane:
    """The locking release lever: a flat paddle with a pivot hole and thumb tab."""
    # Main lever body - flat rectangular paddle
    lever = (
        cq.Workplane("XY")
        .box(LEVER_LENGTH, LEVER_WIDTH, LEVER_THICK)
    )
    # Rounded thumb tab at the free end
    tab = (
        cq.Workplane("XY")
        .circle(LEVER_WIDTH * 0.6)
        .extrude(LEVER_THICK)
        .translate((LEVER_LENGTH / 2.0, 0.0, -LEVER_THICK / 2.0))
    )
    lever = lever.union(tab)
    # Pivot bore at the root end
    bore = (
        cq.Workplane("XY")
        .circle(0.0012)
        .extrude(LEVER_THICK + 0.002)
        .translate((-LEVER_LENGTH / 2.0, 0.0, -LEVER_THICK / 2.0 - 0.001))
    )
    lever = lever.cut(bore)
    return lever


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="diagonal_cutting_pliers")

    steel_polished = model.material("steel_polished", rgba=(0.80, 0.81, 0.84, 1.0))
    steel_forged = model.material("steel_forged", rgba=(0.66, 0.67, 0.70, 1.0))
    steel_brushed = model.material("steel_brushed", rgba=(0.61, 0.62, 0.65, 1.0))
    seam_gray = model.material("seam_gray", rgba=(0.45, 0.46, 0.48, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.09, 1.0))
    grip_red = model.material("grip_red", rgba=(0.80, 0.09, 0.11, 1.0))
    lever_gray = model.material("lever_gray", rgba=(0.35, 0.35, 0.38, 1.0))

    parts = []
    for part_name, s in (("plier_half_0", 1), ("plier_half_1", -1)):
        part = model.part(part_name)
        tag = part_name.replace("plier_half_", "half")

        part.visual(
            mesh_from_cadquery(_diagonal_jaw_solid(s), f"{tag}_jaw", tolerance=0.0002),
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

    # --- Pivot rivet: caps on BOTH sides ---
    fixed = parts[0]

    # Bottom side: seam ring + boss cap
    fixed.visual(
        Cylinder(SEAM_R, SEAM_H),
        origin=Origin(xyz=(0.0, 0.0, -(HALF_T + SEAM_H / 2.0 - EPS))),
        name="boss_seam_bottom",
        material=seam_gray,
    )
    fixed.visual(
        Cylinder(BOSS_R, BOSS_H),
        origin=Origin(xyz=(0.0, 0.0, -(HALF_T + SEAM_H + BOSS_H / 2.0 - 2.0 * EPS))),
        name="rivet_cap_bottom",
        material=steel_brushed,
    )

    # Rivet shaft through both halves
    fixed.visual(
        Cylinder(RIVET_R, 2.0 * HALF_T + SEAM_H + 2.0 * EPS + 0.0005),
        origin=Origin(xyz=(0.0, 0.0, 0.00005)),
        name="rivet_shaft",
        material=steel_brushed,
    )

    # Top side: seam ring + boss cap
    fixed.visual(
        Cylinder(SEAM_R, SEAM_H),
        origin=Origin(xyz=(0.0, 0.0, HALF_T + SEAM_H / 2.0 + EPS)),
        name="boss_seam_top",
        material=seam_gray,
    )
    fixed.visual(
        Cylinder(BOSS_R, BOSS_H),
        origin=Origin(xyz=(0.0, 0.0, HALF_T + SEAM_H + BOSS_H / 2.0)),
        name="rivet_cap_top",
        material=steel_brushed,
    )

    # --- Locking release lever: pivots inside half_0's handle ---
    lever = model.part("release_lever")

    # Lever body positioned at the lever pivot location in the handle
    lever_center_y = _yc(LEVER_PIVOT_X + LEVER_LENGTH / 2.0)
    lever_body = _lever_body_solid().translate(
        (LEVER_PIVOT_X + LEVER_LENGTH / 2.0, lever_center_y, 0.0)
    )
    lever.visual(
        mesh_from_cadquery(lever_body, "lever_body", tolerance=0.0002),
        name="lever_body",
        material=lever_gray,
    )

    # Primary articulation: revolute at the rivet pivot
    # Positive q (about -Z) swings half_1's jaw toward -Y, opening jaws
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=parts[0],
        child=parts[1],
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=3.0, lower=0.0, upper=OPEN_LIMIT),
    )

    # Release lever articulation: revolute at the lever pivot pin
    # The lever rotates about Z (perpendicular to tool plane), pivoting
    # inside the handle recess. Positive q (about +Z) swings the free end
    # outward (release direction).
    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=parts[0],
        child=lever,
        origin=Origin(xyz=(LEVER_PIVOT_X, lever_center_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=0.0, upper=LEVER_RANGE),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    half0 = object_model.get_part("plier_half_0")
    half1 = object_model.get_part("plier_half_1")
    lever = object_model.get_part("release_lever")
    pivot = object_model.get_articulation("pivot")
    lever_pivot = object_model.get_articulation("lever_pivot")

    jaw0 = half0.get_visual("jaw")
    jaw1 = half1.get_visual("jaw")
    hub0 = half0.get_visual("hub")
    hub1 = half1.get_visual("hub")
    grip0 = half0.get_visual("grip")
    grip1 = half1.get_visual("grip")

    # --- Joint contract: pivot is revolute 0..25 degrees ---
    limits = pivot.motion_limits
    ctx.check(
        "pivot is a 0..25 degree revolute joint",
        pivot.articulation_type == ArticulationType.REVOLUTE
        and limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower) < 1e-9
        and abs(limits.upper - OPEN_LIMIT) < 1e-6,
        details=f"limits={limits}",
    )

    # --- Lever joint contract: revolute 0..15 degrees ---
    lever_limits = lever_pivot.motion_limits
    ctx.check(
        "lever_pivot is a 0..15 degree revolute joint",
        lever_pivot.articulation_type == ArticulationType.REVOLUTE
        and lever_limits is not None
        and lever_limits.lower is not None
        and lever_limits.upper is not None
        and abs(lever_limits.lower) < 1e-9
        and abs(lever_limits.upper - LEVER_RANGE) < 1e-6,
        details=f"lever_limits={lever_limits}",
    )

    # --- Compact diagonal jaws: shorter than lineman pliers ---
    jaw0_aabb = ctx.part_element_world_aabb(half0, elem="jaw")
    if jaw0_aabb is not None:
        jaw_length = jaw0_aabb[1][0] - jaw0_aabb[0][0]
        ctx.check(
            "compact diagonal jaw is short (under 0.030 m)",
            jaw_length < 0.030,
            details=f"jaw_length={jaw_length:.4f}",
        )
    else:
        ctx.fail("jaw AABB resolves", "missing jaw element AABB")

    # --- Closed rest pose: cutter faces nearly touch ---
    ctx.expect_gap(
        half0,
        half1,
        axis="y",
        positive_elem=jaw0,
        negative_elem=jaw1,
        min_gap=0.0002,
        max_gap=0.0015,
        name="cutter jaws closed and nearly touching at rest",
    )

    # --- Halves interleave at the pivot ---
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
        min_overlap=0.016,
        name="hub laps share the pivot footprint",
    )

    # --- Rivet shaft captured through the moving half ---
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
        min_overlap=0.006,
        name="rivet shaft passes through the full moving lap thickness",
    )

    # --- Rivet caps on BOTH sides ---
    cap_bottom = ctx.part_element_world_aabb(half0, elem="rivet_cap_bottom")
    cap_top = ctx.part_element_world_aabb(half0, elem="rivet_cap_top")
    ok_caps = cap_bottom is not None and cap_top is not None
    if ok_caps:
        # Both caps should be on opposite sides of the tool (one below, one above)
        ctx.check(
            "bottom rivet cap is below the tool plane",
            cap_bottom[0][2] < -HALF_T,
            details=f"cap_bottom_min_z={cap_bottom[0][2]:.4f}",
        )
        ctx.check(
            "top rivet cap is above the tool plane",
            cap_top[1][2] > HALF_T,
            details=f"cap_top_max_z={cap_top[1][2]:.4f}",
        )
        # Both caps should be centered on the pivot
        cx_bot = 0.5 * (cap_bottom[0][0] + cap_bottom[1][0])
        cx_top = 0.5 * (cap_top[0][0] + cap_top[1][0])
        cy_bot = 0.5 * (cap_bottom[0][1] + cap_bottom[1][1])
        cy_top = 0.5 * (cap_top[0][1] + cap_top[1][1])
        ctx.check(
            "both rivet caps centered on the pivot",
            abs(cx_bot) <= 0.002 and abs(cy_bot) <= 0.002
            and abs(cx_top) <= 0.002 and abs(cy_top) <= 0.002,
            details=f"bot=({cx_bot:.4f},{cy_bot:.4f}) top=({cx_top:.4f},{cy_top:.4f})",
        )
    else:
        ctx.fail("rivet cap AABBs resolve", "missing rivet_cap_bottom or rivet_cap_top AABB")

    # --- Release lever exists and is mounted near the handle ---
    lever_aabb = ctx.part_world_aabb(lever)
    if lever_aabb is not None:
        ctx.check(
            "release lever is positioned in the handle region",
            lever_aabb[0][0] < -0.030 and lever_aabb[1][0] < -0.020,
            details=f"lever_x_range=[{lever_aabb[0][0]:.4f}, {lever_aabb[1][0]:.4f}]",
        )
    else:
        ctx.fail("lever AABB resolves", "missing release_lever part AABB")

    # --- Lever is supported by half_0 (connected via pivot articulation) ---
    lever_body_aabb = ctx.part_element_world_aabb(lever, elem="lever_body")
    grip0_aabb = ctx.part_element_world_aabb(half0, elem="grip")
    if lever_body_aabb is not None and grip0_aabb is not None:
        # Lever should overlap with the grip region in XY
        ctx.expect_overlap(
            lever,
            half0,
            axes="xy",
            elem_a="lever_body",
            elem_b="grip",
            min_overlap=0.005,
            name="release lever overlaps with handle grip in XY",
        )

    # --- Lever pivots outward at max angle ---
    closed_lever = ctx.part_element_world_aabb(lever, elem="lever_body")
    with ctx.pose({lever_pivot: LEVER_RANGE}):
        open_lever = ctx.part_element_world_aabb(lever, elem="lever_body")
        ctx.check(
            "release lever swings outward at max angle",
            closed_lever is not None
            and open_lever is not None
            and (
                abs(open_lever[0][1] - closed_lever[0][1]) > 0.001
                or abs(open_lever[1][1] - closed_lever[1][1]) > 0.001
            ),
            details=f"closed_y={closed_lever[0][1]:.4f}..{closed_lever[1][1]:.4f} "
            f"open_y={open_lever[0][1]:.4f}..{open_lever[1][1]:.4f}",
        )

    # --- Compact overall size: shorter than parent lineman pliers ---
    a0 = ctx.part_world_aabb(half0)
    a1 = ctx.part_world_aabb(half1)
    g0 = ctx.part_element_world_aabb(half0, elem="grip")
    g1 = ctx.part_element_world_aabb(half1, elem="grip")
    ok_env = a0 is not None and a1 is not None and g0 is not None and g1 is not None
    if ok_env:
        length = max(a0[1][0], a1[1][0]) - min(a0[0][0], a1[0][0])
        across = g1[1][1] - g0[0][1]
        ctx.check(
            "compact overall length about 0.14-0.17 m",
            0.13 <= length <= 0.18,
            details=f"length={length:.4f}",
        )
        ctx.check(
            "handle grips span about 0.05-0.07 m across",
            0.045 <= across <= 0.075,
            details=f"across={across:.4f}",
        )
    else:
        ctx.fail("envelope AABBs resolve", "missing part/grip AABB")

    # --- Open pose: jaws separate and handles spread ---
    closed_jaw1 = ctx.part_element_world_aabb(half1, elem="jaw")
    closed_grip1 = ctx.part_element_world_aabb(half1, elem="grip")
    with ctx.pose({pivot: OPEN_LIMIT}):
        ctx.expect_gap(
            half0,
            half1,
            axis="y",
            positive_elem=jaw0,
            negative_elem=jaw1,
            min_gap=0.003,
            name="cutter jaws open apart at the 25 degree pose",
        )
        open_jaw1 = ctx.part_element_world_aabb(half1, elem="jaw")
        open_grip1 = ctx.part_element_world_aabb(half1, elem="grip")
        ctx.check(
            "moving jaw swings away from the fixed jaw",
            closed_jaw1 is not None
            and open_jaw1 is not None
            and open_jaw1[0][1] < closed_jaw1[0][1] - 0.010,
            details=f"closed_min_y={closed_jaw1}, open_min_y={open_jaw1}",
        )
        ctx.check(
            "moving handle spreads outward as the jaws open",
            closed_grip1 is not None
            and open_grip1 is not None
            and open_grip1[1][1] > closed_grip1[1][1] + 0.015,
            details=f"closed_max_y={closed_grip1}, open_max_y={open_grip1}",
        )

    return ctx.report()


object_model = build_object_model()
