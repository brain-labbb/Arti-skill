from __future__ import annotations

# Round-nose jewelry pliers with tapered conical jaws.
# Variant of heavy-duty combination (lineman) pliers.
#
# Layout: the tool lies in the XY plane with Z as the thickness axis.
# The rivet pivot is at the origin. The conical jaws point +X,
# the handles sweep back to -X and spread in +/-Y.
# plier_half_0 (root) carries its jaw on +Y; plier_half_1 is the mirrored
# moving half. The two forged halves are half-lapped at the pivot.

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
HALF_T = 0.006           # half thickness of each forged plate (full plate 0.012)
LAP_R = 0.012            # half-lap joint disc radius at the pivot
HUB_R = 0.011            # forged hub radius around the rivet
BOSS_R = 0.009           # rivet boss radius (~0.018 m diameter)
BOSS_H = 0.0025
SEAM_R = 0.0095          # visible circular seam ring under the boss cap
SEAM_H = 0.0005
RIVET_R = 0.003          # rivet shaft captured through the moving half's lap
EPS = 0.0001             # lap clearance
JAW_FACE = 0.0003        # closed jaw inner faces sit at y = +/-JAW_FACE
JAW_BASE_X = 0.010       # jaw starts just past the pivot
JAW_TIP_X = 0.065        # conical jaw tip (round nose)
JAW_BASE_R = 0.006       # jaw radius at the base (where it meets the hub)
JAW_TIP_R = 0.001        # jaw radius at the tip (tapered point)
OPEN_LIMIT = math.radians(30.0)

TANG_HALF_W = 0.003      # steel handle tang half width in plan

# Steel handle tang centerline in the tool plane (for the half whose jaw is +Y).
TANG_PTS = [
    (-0.008, -0.004),
    (-0.025, -0.010),
    (-0.055, -0.016),
    (-0.085, -0.020),
    (-0.105, -0.022),
]
CENTERLINE = TANG_PTS + [(-0.115, -0.023)]

# Grip loft stations: (x, half_width_y, half_height_z)
GRIP_SECTIONS = [
    (-0.026, 0.0070, 0.0090),  # flared guard near the pivot
    (-0.035, 0.0058, 0.0080),
    (-0.055, 0.0055, 0.0078),
    (-0.075, 0.0056, 0.0080),
    (-0.095, 0.0058, 0.0082),
    (-0.108, 0.0060, 0.0084),  # slight end swell
    (-0.113, 0.0042, 0.0060),
    (-0.115, 0.0020, 0.0030),
]


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


def _conical_jaw_solid(s: int) -> cq.Workplane:
    """Tapered conical round-nose jaw with serrated inner teeth.

    Each jaw half is a loft of circles tangent to the parting plane at y=s*JAW_FACE.
    The flat inner face is formed by cutting below the parting plane.
    Serrated teeth are cut as grooves on the inner face.
    """
    n_sections = 14
    wires = []
    for i in range(n_sections + 1):
        t = i / n_sections
        x = JAW_BASE_X + t * (JAW_TIP_X - JAW_BASE_X)
        # Taper radius: smooth decrease from base to tip
        r = JAW_BASE_R * (1.0 - t) + JAW_TIP_R * t
        # Center each circle so its inner tangent sits at y = s * JAW_FACE
        cy = s * (JAW_FACE + r)
        wp = cq.Workplane("YZ", origin=(x, 0.0, 0.0)).center(cy, 0.0).circle(r)
        wires.append(wp.val())

    jaw = cq.Solid.makeLoft(wires, ruled=False)
    jaw_wp = cq.Workplane("XY").add(jaw)

    # Cut everything below the parting plane to create a flat inner face.
    # For s=+1 jaw, cut y < JAW_FACE; for s=-1, cut y > -JAW_FACE.
    cut_depth = JAW_BASE_R * 3.0  # deep enough to remove all material past the flat
    cut_width = JAW_TIP_X - JAW_BASE_X + 0.01
    cut_height = HALF_T * 3.0
    if s > 0:
        # Remove material below y = JAW_FACE
        inner_cut = (
            cq.Workplane("XY")
            .box(cut_width, cut_depth, cut_height)
            .translate(((JAW_BASE_X + JAW_TIP_X) / 2.0,
                        JAW_FACE - cut_depth / 2.0, 0.0))
        )
    else:
        # Remove material above y = -JAW_FACE
        inner_cut = (
            cq.Workplane("XY")
            .box(cut_width, cut_depth, cut_height)
            .translate(((JAW_BASE_X + JAW_TIP_X) / 2.0,
                        -JAW_FACE + cut_depth / 2.0, 0.0))
        )
    jaw_wp = jaw_wp.cut(inner_cut)

    # Serrated teeth on the inner jaw face - transverse grooves
    n_teeth = 16
    for i in range(n_teeth):
        t = (i + 0.5) / n_teeth
        xi = JAW_BASE_X + 0.003 + t * (JAW_TIP_X - JAW_BASE_X - 0.008)
        r_at_x = JAW_BASE_R * (1.0 - t) + JAW_TIP_R * t
        # Groove width scales with local jaw radius
        gw = min(r_at_x * 1.4, 0.008)
        groove = (
            cq.Workplane("XY")
            .box(0.0006, gw, 0.0008)
            .translate((xi, s * (JAW_FACE + 0.0001), 0.0))
        )
        jaw_wp = jaw_wp.cut(groove)

    # Longitudinal serration grooves (along the jaw length)
    n_long = 4
    for i in range(n_long):
        zi = -0.003 + i * 0.002
        long_groove = (
            cq.Workplane("XY")
            .box(JAW_TIP_X - JAW_BASE_X - 0.006, 0.0005, 0.0006)
            .translate(((JAW_BASE_X + JAW_TIP_X) / 2.0, s * (JAW_FACE + 0.0001), zi))
        )
        jaw_wp = jaw_wp.cut(long_groove)

    return jaw_wp.cut(_lap_cut(s))


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
    """Soft-touch rubber grip: curved shaft, rounded end."""
    wires = [
        _ellipse_wire(x, s * _yc(x), 0.0, w, h)
        for x, w, h in GRIP_SECTIONS
    ]
    return cq.Solid.makeLoft(wires, ruled=False)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="round_nose_pliers")

    steel_polished = model.material("steel_polished", rgba=(0.82, 0.83, 0.86, 1.0))
    steel_forged = model.material("steel_forged", rgba=(0.62, 0.63, 0.66, 1.0))
    steel_brushed = model.material("steel_brushed", rgba=(0.58, 0.59, 0.62, 1.0))
    seam_gray = model.material("seam_gray", rgba=(0.42, 0.43, 0.45, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.06, 0.06, 0.07, 1.0))

    parts = []
    for part_name, s in (("plier_half_0", 1), ("plier_half_1", -1)):
        part = model.part(part_name)
        tag = part_name.replace("plier_half_", "half")

        part.visual(
            mesh_from_cadquery(_conical_jaw_solid(s), f"{tag}_jaw", tolerance=0.0002),
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

        parts.append(part)

    # Rivet assembly fixed to half_0
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
        Cylinder(RIVET_R, 2.0 * HALF_T + SEAM_H + 2.0 * EPS + 0.0004),
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

    # Primary articulation: revolute pivot at the rivet.
    # Positive q (about -Z) swings half_1's jaw toward -Y, opening the jaws.
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=parts[0],
        child=parts[1],
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=3.0, lower=0.0, upper=OPEN_LIMIT),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    half0 = object_model.get_part("plier_half_0")
    half1 = object_model.get_part("plier_half_1")
    pivot = object_model.get_articulation("pivot")

    jaw0 = half0.get_visual("jaw")
    jaw1 = half1.get_visual("jaw")
    hub0 = half0.get_visual("hub")
    hub1 = half1.get_visual("hub")
    grip0 = half0.get_visual("grip")
    grip1 = half1.get_visual("grip")

    # --- Joint contract: pivot is a 0..30 degree revolute ---
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

    # --- Conical jaw geometry: jaws taper from base to tip ---
    jaw0_aabb = ctx.part_element_world_aabb(half0, elem="jaw")
    if jaw0_aabb is not None:
        j_min, j_max = jaw0_aabb
        jaw_length = j_max[0] - j_min[0]
        jaw_width = j_max[1] - j_min[1]
        jaw_height = j_max[2] - j_min[2]
        # Jaw should be longer than it is wide (tapered cone)
        ctx.check(
            "jaw is elongated along the tool axis (tapered conical shape)",
            jaw_length > 0.04 and jaw_length > jaw_width * 3.0,
            details=f"length={jaw_length:.4f} width={jaw_width:.4f}",
        )
        # Jaw width at base should be substantial (>3mm)
        ctx.check(
            "jaw base has substantial width (round nose profile)",
            jaw_width > 0.003,
            details=f"jaw_width={jaw_width:.4f}",
        )
    else:
        ctx.fail("jaw AABB resolves", "missing jaw element AABB")

    # --- Serrated teeth geometry on inner jaws ---
    # At closed pose, the inner jaw faces are very close together
    ctx.expect_gap(
        half0,
        half1,
        axis="y",
        positive_elem=jaw0,
        negative_elem=jaw1,
        min_gap=0.0001,
        max_gap=0.002,
        name="serrated inner jaw faces nearly touch at rest",
    )

    # --- Closed rest pose: jaws close, handles spread ---
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
        min_overlap=0.014,
        name="hub laps share the pivot footprint",
    )

    # --- Rivet shaft captured through moving half ---
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
        min_overlap=0.005,
        name="rivet shaft passes through the full moving lap thickness",
    )

    # --- Rivet boss centered on the pivot ---
    boss0_aabb = ctx.part_element_world_aabb(half0, elem="rivet_boss")
    boss1_aabb = ctx.part_element_world_aabb(half0, elem="rivet_head")
    ok_boss = boss0_aabb is not None and boss1_aabb is not None
    if ok_boss:
        b0_min, b0_max = boss0_aabb
        b1_min, b1_max = boss1_aabb
        dia = b0_max[0] - b0_min[0]
        cx = 0.5 * (b0_min[0] + b0_max[0])
        cy = 0.5 * (b0_min[1] + b0_max[1])
        ctx.check(
            "rivet boss is ~18 mm diameter and centered on the pivot",
            0.016 <= dia <= 0.020 and abs(cx) <= 0.002 and abs(cy) <= 0.002,
            details=f"dia={dia:.4f} center=({cx:.4f},{cy:.4f})",
        )
    else:
        ctx.fail("rivet boss AABBs resolve", "missing rivet_boss element AABB")

    # --- Overall proportions: ~0.15-0.18 m long jewelry pliers ---
    a0 = ctx.part_world_aabb(half0)
    a1 = ctx.part_world_aabb(half1)
    ok_env = a0 is not None and a1 is not None
    if ok_env:
        length = max(a0[1][0], a1[1][0]) - min(a0[0][0], a1[0][0])
        ctx.check(
            "overall length about 0.15-0.18 m (jewelry pliers)",
            0.14 <= length <= 0.20,
            details=f"length={length:.4f}",
        )
    else:
        ctx.fail("envelope AABBs resolve", "missing part AABB")

    # --- Decisive open pose: jaws separate and handles spread ---
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
            name="conical jaws open apart at the 30 degree pose",
        )
        open_jaw1 = ctx.part_element_world_aabb(half1, elem="jaw")
        open_grip1 = ctx.part_element_world_aabb(half1, elem="grip")
        ctx.check(
            "moving jaw swings away from the fixed jaw",
            closed_jaw1 is not None
            and open_jaw1 is not None
            and open_jaw1[0][1] < closed_jaw1[0][1] - 0.012,
            details=f"closed_min_y={closed_jaw1}, open_min_y={open_jaw1}",
        )
        ctx.check(
            "moving handle spreads outward as the jaws open",
            closed_grip1 is not None
            and open_grip1 is not None
            and open_grip1[1][1] > closed_grip1[1][1] + 0.02,
            details=f"closed_max_y={closed_grip1}, open_max_y={open_grip1}",
        )

    return ctx.report()


object_model = build_object_model()
