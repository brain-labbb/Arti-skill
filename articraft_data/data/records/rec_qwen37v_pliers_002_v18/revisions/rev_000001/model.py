from __future__ import annotations

# End-cutting nippers (carpenter's pincers).
# Variant of heavy-duty combination pliers with jaws PERPENDICULAR to handles.
#
# Layout: the tool lies in the XY plane with Z as the thickness axis.
# The rivet pivot is at the origin. The handles sweep back to -X and
# spread in +/-Y. The jaw blades extend upward (+Z) from the pivot,
# perpendicular to the handle axis. The two forged halves are half-lapped
# at the pivot: half_0 keeps the lower lap (z <= -EPS), half_1 keeps the
# upper lap (z >= +EPS), so one half visibly passes over the other under
# the rivet. Circular rivet caps are visible on both outer faces.

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
JAW_FACE = 0.0005       # closed jaw inner faces sit at y = +/-JAW_FACE
OPEN_LIMIT = math.radians(30.0)

# Perpendicular jaw blade dimensions
JAW_HEIGHT = 0.040      # jaw blade extends this far in +Z from pivot
JAW_THICK = 0.005       # jaw blade thickness in Y direction

TANG_HALF_W = 0.004     # steel handle tang half width in plan

# Steel handle tang centerline in the tool plane (for the half whose jaw is +Y).
# Extended further back for ~0.20 m total length (handles + jaw forward extent).
TANG_PTS = [
    (-0.010, -0.004),
    (-0.030, -0.011),
    (-0.070, -0.019),
    (-0.110, -0.025),
    (-0.140, -0.028),
    (-0.155, -0.030),
]
# Extended centerline used by the over-molded grip (reaches the bulbous end).
CENTERLINE = TANG_PTS + [(-0.168, -0.0314)]

# Grip loft stations: (x, half_width_y, half_height_z)
GRIP_SECTIONS = [
    (-0.032, 0.0100, 0.0130),  # flared thumb guard near the pivot
    (-0.040, 0.0080, 0.0110),
    (-0.060, 0.0078, 0.0108),
    (-0.095, 0.0080, 0.0112),
    (-0.125, 0.0086, 0.0118),
    (-0.148, 0.0088, 0.0120),  # bulbous end swell
    (-0.160, 0.0062, 0.0086),
    (-0.168, 0.0028, 0.0040),
]

# Red inlay stations along the outer/top face (stops before the black end bulb).
INLAY_XS = [-0.036, -0.060, -0.085, -0.110, -0.135, -0.148]
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
    """
    cut = cq.Workplane("XY").circle(LAP_R).extrude(0.012)
    if s > 0:
        return cut.translate((0.0, 0.0, -EPS))
    return cut.translate((0.0, 0.0, -0.012 + EPS))


def _jaw_solid(s: int) -> cq.Workplane:
    """Perpendicular end-cutting jaw blade extending in +Z from the pivot.

    The jaw is a curved pincer blade whose cutting edge is at the top (+Z),
    perpendicular to the handle axis (-X). Each half sits at y = s*JAW_FACE
    so the inner cutting faces nearly meet when closed.

    The jaw base extends down into the hub layer so it connects to the hub disc
    without needing the half-lap cut (the jaw is perpendicular to the pivot plane).
    """
    # Each half's jaw base starts at its hub layer Z
    if s > 0:
        z_base = -HALF_T  # lower lap: base at hub bottom
    else:
        z_base = EPS      # upper lap: base at hub top of lower boundary

    # Jaw profile in XZ plane: a D-shaped pincer blade
    # Starts at the hub base, rises up, curves forward and back down
    profile_xz = [
        (0.004, z_base),       # base at hub, near pivot
        (0.002, z_base + 0.012),
        (0.003, z_base + 0.025),
        (0.006, z_base + 0.035),
        (0.012, z_base + 0.040),  # blade tip (highest point)
        (0.020, z_base + 0.039),
        (0.027, z_base + 0.034),
        (0.032, z_base + 0.025),
        (0.034, z_base + 0.015),
        (0.030, z_base + 0.005),
        (0.022, z_base + 0.001),
        (0.016, z_base),       # base outer
    ]

    # Build the jaw as a loft from XZ profiles at two Y stations
    y_inner = s * JAW_FACE
    y_outer = s * (JAW_FACE + JAW_THICK)

    def make_profile_wire(y_pos: float) -> cq.Wire:
        pts_3d = [(x, y_pos, z) for x, z in profile_xz]
        pts_3d.append(pts_3d[0])  # close the loop
        edges = []
        for i in range(len(pts_3d) - 1):
            e = cq.Edge.makeLine(
                cq.Vector(*pts_3d[i]),
                cq.Vector(*pts_3d[i + 1]),
            )
            edges.append(e)
        return cq.Wire.assembleEdges(edges)

    wire_inner = make_profile_wire(y_inner)
    wire_outer = make_profile_wire(y_outer)

    jaw = cq.Solid.makeLoft([wire_inner, wire_outer], ruled=True)
    return cq.Workplane("XY").add(jaw)


def _hub_solid(s: int) -> cq.Workplane:
    """Forged circular hub around the rivet, thinned to its lap layer.

    The hub is the pivot disc that interleaves with the other half.
    """
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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="end_cutting_nippers")

    steel_polished = model.material("steel_polished", rgba=(0.80, 0.81, 0.84, 1.0))
    steel_forged = model.material("steel_forged", rgba=(0.66, 0.67, 0.70, 1.0))
    steel_brushed = model.material("steel_brushed", rgba=(0.61, 0.62, 0.65, 1.0))
    seam_gray = model.material("seam_gray", rgba=(0.45, 0.46, 0.48, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.09, 1.0))
    grip_red = model.material("grip_red", rgba=(0.80, 0.09, 0.11, 1.0))

    parts = []
    for part_name, s in (("nipper_half_0", 1), ("nipper_half_1", -1)):
        part = model.part(part_name)
        tag = part_name.replace("nipper_half_", "half")

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

    # Circular pivot rivet cap on BOTH sides:
    # Bottom seam + boss on one outer face, top seam + boss on the other.
    # The rivet shaft passes through both halves at the pivot.
    fixed = parts[0]

    # --- Bottom side (rivet cap side 1) ---
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

    # --- Rivet shaft through both halves ---
    fixed.visual(
        Cylinder(RIVET_R, 2.0 * HALF_T + SEAM_H + 2.0 * EPS + 0.0005),
        origin=Origin(xyz=(0.0, 0.0, 0.00005)),
        name="rivet_shaft",
        material=steel_brushed,
    )

    # --- Top side (rivet cap side 2) ---
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

    # Primary articulation: one revolute joint at the rivet, axis perpendicular
    # to the flat tool plane (Z axis). Positive q (about -Z) swings half_1's jaw
    # toward -Y, opening the perpendicular jaws while the handles spread apart.
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=parts[0],
        child=parts[1],
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=3.0, lower=0.0, upper=OPEN_LIMIT),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    half0 = object_model.get_part("nipper_half_0")
    half1 = object_model.get_part("nipper_half_1")
    pivot = object_model.get_articulation("pivot")

    jaw0 = half0.get_visual("jaw")
    jaw1 = half1.get_visual("jaw")
    hub0 = half0.get_visual("hub")
    hub1 = half1.get_visual("hub")
    grip0 = half0.get_visual("grip")
    grip1 = half1.get_visual("grip")

    # ---- Joint contract: a single non-fixed revolute pivot ----
    limits = pivot.motion_limits
    ctx.check(
        "pivot is a non-fixed revolute joint (0..30 degrees)",
        pivot.articulation_type == ArticulationType.REVOLUTE
        and limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower) < 1e-9
        and abs(limits.upper - OPEN_LIMIT) < 1e-6
        and limits.upper > 0.01,
        details=f"limits={limits}",
    )

    # ---- Perpendicular jaw geometry: jaws extend in +Z, not inline with handles ----
    jaw0_aabb = ctx.part_element_world_aabb(half0, elem="jaw")
    jaw1_aabb = ctx.part_element_world_aabb(half1, elem="jaw")
    if jaw0_aabb is not None and jaw1_aabb is not None:
        jaw0_min, jaw0_max = jaw0_aabb
        jaw1_min, jaw1_max = jaw1_aabb
        jaw_height_0 = jaw0_max[2] - jaw0_min[2]
        jaw_x_extent_0 = jaw0_max[0] - jaw0_min[0]
        ctx.check(
            "jaws extend perpendicular to handles (Z extent > X extent)",
            jaw_height_0 > 0.025 and jaw_height_0 > jaw_x_extent_0,
            details=f"jaw0_z_extent={jaw_height_0:.4f} jaw0_x_extent={jaw_x_extent_0:.4f}",
        )
        ctx.check(
            "both jaw blades reach at least 25 mm above the pivot",
            jaw0_max[2] >= 0.025 and jaw1_max[2] >= 0.025,
            details=f"jaw0_max_z={jaw0_max[2]:.4f} jaw1_max_z={jaw1_max[2]:.4f}",
        )
    else:
        ctx.fail("jaw AABBs resolve", "missing jaw element AABB")

    # ---- Closed rest pose: perpendicular jaw inner faces nearly touch ----
    ctx.expect_gap(
        half0,
        half1,
        axis="y",
        positive_elem=jaw0,
        negative_elem=jaw1,
        min_gap=-0.002,
        max_gap=0.003,
        name="perpendicular jaws closed and nearly touching at rest",
    )

    # ---- Halves interleave at the pivot via half-lap joint ----
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

    # ---- Rivet shaft captured through moving half ----
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

    # ---- Circular pivot rivet cap on BOTH sides ----
    boss_aabb = ctx.part_element_world_aabb(half0, elem="rivet_boss")
    head_aabb = ctx.part_element_world_aabb(half0, elem="rivet_head")
    ok_caps = boss_aabb is not None and head_aabb is not None
    if ok_caps:
        b_min, b_max = boss_aabb
        h_min, h_max = head_aabb
        boss_dia = b_max[0] - b_min[0]
        head_dia = h_max[0] - h_min[0]
        boss_cx = 0.5 * (b_min[0] + b_max[0])
        boss_cy = 0.5 * (b_min[1] + b_max[1])
        head_cx = 0.5 * (h_min[0] + h_max[0])
        head_cy = 0.5 * (h_min[1] + h_max[1])
        ctx.check(
            "rivet boss cap on bottom side is ~25 mm diameter and centered",
            0.0235 <= boss_dia <= 0.0265 and abs(boss_cx) <= 0.002 and abs(boss_cy) <= 0.002,
            details=f"boss_dia={boss_dia:.4f} center=({boss_cx:.4f},{boss_cy:.4f})",
        )
        ctx.check(
            "rivet head cap on top side is ~25 mm diameter and centered",
            0.0235 <= head_dia <= 0.0265 and abs(head_cx) <= 0.002 and abs(head_cy) <= 0.002,
            details=f"head_dia={head_dia:.4f} center=({head_cx:.4f},{head_cy:.4f})",
        )
        ctx.check(
            "rivet caps proud on both outer faces",
            b_min[2] <= -0.0115 and h_max[2] >= 0.0115,
            details=f"boss_min_z={b_min[2]:.4f} head_max_z={h_max[2]:.4f}",
        )
    else:
        ctx.fail("rivet cap AABBs resolve", "missing rivet_boss or rivet_head element AABB")

    # ---- Overall envelope: ~0.20 m total, ~0.07 m across handles ----
    a0 = ctx.part_world_aabb(half0)
    a1 = ctx.part_world_aabb(half1)
    g0 = ctx.part_element_world_aabb(half0, elem="grip")
    g1 = ctx.part_element_world_aabb(half1, elem="grip")
    ok_env = a0 is not None and a1 is not None and g0 is not None and g1 is not None
    if ok_env:
        length = max(a0[1][0], a1[1][0]) - min(a0[0][0], a1[0][0])
        across = g1[1][1] - g0[0][1]
        ctx.check(
            "overall length about 0.20 m (handles + jaw forward extent)",
            0.185 <= length <= 0.22,
            details=f"length={length:.4f}",
        )
        ctx.check(
            "handle grips span about 0.07 m across",
            0.060 <= across <= 0.085,
            details=f"across={across:.4f}",
        )
    else:
        ctx.fail("envelope AABBs resolve", "missing part/grip AABB")

    # ---- Red inlay on grip outer face ----
    i0 = ctx.part_element_world_aabb(half0, elem="grip_inlay")
    if i0 is not None and g0 is not None:
        ctx.check(
            "red inlay proud on the grip top face",
            i0[1][2] >= g0[1][2] - 0.0005,
            details=f"inlay_top={i0[1][2]:.4f} grip_top={g0[1][2]:.4f}",
        )
    else:
        ctx.fail("inlay AABB resolves", "missing grip_inlay element AABB")

    # ---- Decisive open pose: jaws separate and handles spread ----
    closed_jaw1 = ctx.part_element_world_aabb(half1, elem="jaw")
    closed_grip1 = ctx.part_element_world_aabb(half1, elem="grip")
    with ctx.pose({pivot: OPEN_LIMIT}):
        ctx.expect_gap(
            half0,
            half1,
            axis="y",
            positive_elem=jaw0,
            negative_elem=jaw1,
            min_gap=0.0005,
            name="perpendicular jaws separate at the 30 degree pose",
        )
        open_jaw1 = ctx.part_element_world_aabb(half1, elem="jaw")
        open_grip1 = ctx.part_element_world_aabb(half1, elem="grip")
        ctx.check(
            "moving jaw swings away from the fixed jaw (Y position shifts)",
            closed_jaw1 is not None
            and open_jaw1 is not None
            and open_jaw1[0][1] < closed_jaw1[0][1] - 0.004,
            details=f"closed_min_y={closed_jaw1[0][1]:.4f}, open_min_y={open_jaw1[0][1]:.4f}",
        )
        ctx.check(
            "moving handle spreads outward as the jaws open",
            closed_grip1 is not None
            and open_grip1 is not None
            and open_grip1[1][1] > closed_grip1[1][1] + 0.02,
            details=f"closed_max_y={closed_grip1[1][1]:.4f}, open_max_y={open_grip1[1][1]:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
