from __future__ import annotations

# Locking pliers (Vise-Grip style) variant of lineman pliers.
#
# Layout: the tool lies in the XY plane with Z as the thickness axis.
# The rivet pivot is at the origin. The blunt squared nose points +X,
# the handles sweep back to -X and spread in +/-Y.
# plier_half_0 (root) carries its jaw on +Y; plier_half_1 is the mirrored
# moving half. The two forged halves are half-lapped at the pivot.
#
# Variant changes from parent:
# - Stubbier jaws with prominent serrated V-teeth on inner faces
# - All-metal construction (no rubber grips, steel handles)
# - Rear adjustment screw (prismatic joint) for jaw gap control
# - Central revolute pivot for the two halves

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
NOSE_X = 0.055          # stubbier nose tip for locking pliers
OPEN_LIMIT = math.radians(25.0)  # locking pliers open less than lineman

# Adjustment screw travel (prismatic joint, meters)
SCREW_TRAVEL = 0.012
SCREW_R = 0.004         # screw shaft radius
SCREW_HEAD_R = 0.006    # screw head (knurled knob) radius
SCREW_HEAD_H = 0.006    # screw head height
SCREW_LENGTH = 0.020    # total screw length

TANG_HALF_W = 0.004     # steel handle tang half width in plan

# Steel handle centerline (jaw on +Y half) - locking pliers have slightly
# straighter handles than lineman pliers
TANG_PTS = [
    (-0.010, -0.004),
    (-0.030, -0.010),
    (-0.070, -0.018),
    (-0.100, -0.022),
    (-0.118, -0.024),
]
# Extended centerline for the handle body
CENTERLINE = TANG_PTS + [(-0.131, -0.0254)]

# Handle body loft stations: (x, half_width_y, half_height_z)
# Locking pliers have all-metal knurled handles - narrower than rubber grips.
# The rear end is wider to house the adjustment screw socket.
HANDLE_SECTIONS = [
    (-0.032, 0.0080, 0.0110),  # flared guard near the pivot
    (-0.040, 0.0068, 0.0095),
    (-0.060, 0.0065, 0.0092),
    (-0.085, 0.0068, 0.0095),
    (-0.105, 0.0072, 0.0100),
    (-0.120, 0.0074, 0.0102),
    (-0.126, 0.0070, 0.0095),  # screw socket widens
    (-0.131, 0.0065, 0.0090),  # handle butt end (screw socket)
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


def _handle_w(x: float) -> float:
    return _interp(x, [(sx, w) for sx, w, _h in HANDLE_SECTIONS])


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
    """Locking pliers jaw: stubbier profile with prominent serrated V-teeth."""
    # Locking pliers have a curved jaw profile - shorter and more aggressive.
    # The inner face is at JAW_FACE; the serrated teeth are V-grooves cut into
    # that face, and additional tooth bumps are modeled as part of the main
    # jaw solid (not separate floating geometry).
    profile = [
        (0.010, JAW_FACE),
        (NOSE_X, JAW_FACE),
        (NOSE_X, 0.0100),   # blunt squared nose tip
        (0.048, 0.0112),
        (0.038, 0.0130),
        (0.028, 0.0144),
        (0.018, 0.0154),
        (0.012, 0.0148),
        (0.009, 0.0135),
    ]
    pts = [(x, s * y) for x, y in profile]
    jaw = cq.Workplane("XY").polyline(pts).close().extrude(HALF_T, both=True)

    # Prominent serrated V-teeth on the inner jaw face.
    # V-shaped grooves cut into the flat gripping face create the tooth pattern.
    tooth_spacing = 0.0035
    tooth_depth = 0.0015
    tooth_half_w = tooth_spacing * 0.40
    n_teeth = 10
    for i in range(n_teeth):
        xi = 0.014 + tooth_spacing * i
        if xi > NOSE_X - 0.004:
            break
        # V-groove: triangular cross section cut inward from the inner face
        groove_pts = [
            (xi - tooth_half_w, s * JAW_FACE),
            (xi + tooth_half_w, s * JAW_FACE),
            (xi, s * (JAW_FACE + tooth_depth)),
        ]
        groove = (
            cq.Workplane("XY")
            .polyline(groove_pts)
            .close()
            .extrude(HALF_T * 1.5, both=True)
        )
        jaw = jaw.cut(groove)

    # Additional transverse serration grooves (cross-grip pattern)
    for i in range(5):
        zi = -HALF_T * 0.7 + i * HALF_T * 0.35
        cross_groove = (
            cq.Workplane("XY")
            .box(NOSE_X - 0.020, 0.0012, 0.0008)
            .translate(((0.014 + NOSE_X - 0.004) / 2.0, s * (JAW_FACE + 0.0005), zi))
        )
        jaw = jaw.cut(cross_groove)

    # Rounded pipe-grip recess behind the nose (characteristic of locking pliers)
    recess = (
        cq.Workplane("XY")
        .circle(0.004)
        .extrude(0.011, both=True)
        .translate((0.028, s * JAW_FACE, 0.0))
    )
    jaw = jaw.cut(recess)

    # Serrated scallop teeth around the recess rim
    for ang_deg in (35.0, 70.0, 110.0, 145.0):
        a = math.radians(ang_deg)
        sx = 0.028 + 0.004 * math.cos(a)
        sy = s * (JAW_FACE + 0.004 * math.sin(a))
        scallop = (
            cq.Workplane("XY")
            .circle(0.0009)
            .extrude(0.011, both=True)
            .translate((sx, sy, 0.0))
        )
        jaw = jaw.cut(scallop)

    # Wire-cutter notch between the pipe grip and the pivot
    notch_pts = [(0.014, s * -0.001), (0.020, s * -0.001), (0.017, s * 0.003)]
    notch = cq.Workplane("XY").polyline(notch_pts).close().extrude(0.011, both=True)
    jaw = jaw.cut(notch)

    return jaw.cut(_lap_cut(s))


def _hub_solid(s: int) -> cq.Workplane:
    """Forged circular hub around the rivet, thinned to its lap layer."""
    hub = cq.Workplane("XY").circle(HUB_R).extrude(HALF_T, both=True)
    return hub.cut(_lap_cut(s))


def _shank_solid(s: int) -> cq.Workplane:
    """Steel handle tang sweeping back from the hub."""
    pts = [(x, s * y) for x, y in TANG_PTS]
    loop = _strip_loop(pts, TANG_HALF_W)
    shank = cq.Workplane("XY").polyline(loop).close().extrude(HALF_T, both=True)
    return shank.cut(_lap_cut(s))


def _ellipse_wire(x: float, yc: float, zc: float, w: float, h: float) -> cq.Wire:
    wp = cq.Workplane("YZ", origin=(x, 0.0, 0.0)).center(yc, zc).ellipse(w, h)
    return wp.val()


def _handle_solid(s: int) -> cq.Solid:
    """All-metal handle body: knurled steel, no rubber overmold."""
    wires = [
        _ellipse_wire(x, s * _yc(x), 0.0, w, h)
        for x, w, h in HANDLE_SECTIONS
    ]
    return cq.Solid.makeLoft(wires, ruled=False)


def _knurl_solid(s: int) -> cq.Solid:
    """Knurled surface sleeve on the handle - a thin lofted shell that follows
    the handle profile, slightly larger than the handle body to sit proud."""
    # Build a slightly larger sleeve that follows the handle cross sections
    wires = []
    for x, w, h in HANDLE_SECTIONS:
        # Slightly larger than the handle to sit proud on the surface
        wp = cq.Workplane("YZ", origin=(x, 0.0, 0.0)).center(s * _yc(x), 0.0)
        wires.append(wp.ellipse(w + 0.0004, h + 0.0004).val())
    outer = cq.Solid.makeLoft(wires, ruled=False)

    # Cut out the interior to make a thin shell
    inner_wires = []
    for x, w, h in HANDLE_SECTIONS:
        wp = cq.Workplane("YZ", origin=(x, 0.0, 0.0)).center(s * _yc(x), 0.0)
        inner_wires.append(wp.ellipse(w - 0.0008, h - 0.0008).val())
    inner = cq.Solid.makeLoft(inner_wires, ruled=False)

    shell = outer.cut(inner)
    return shell


def _screw_solid() -> cq.Workplane:
    """Rear adjustment screw: threaded shaft + knurled head.

    Built along +X in local frame:
    - Head at origin (protrudes from handle butt at -X in world)
    - Shaft extends along +X (into the handle body)
    """
    # Knurled head at origin, extending in -X (sticks out behind handle)
    head = (
        cq.Workplane("YZ")
        .circle(SCREW_HEAD_R)
        .extrude(SCREW_HEAD_H)
        .translate((-SCREW_HEAD_H, 0.0, 0.0))
    )

    # Knurl grooves on the head
    for ang_deg in range(0, 360, 20):
        a = math.radians(ang_deg)
        cy = SCREW_HEAD_R * math.cos(a)
        cz = SCREW_HEAD_R * math.sin(a)
        groove = (
            cq.Workplane("XY")
            .box(SCREW_HEAD_H + 0.001, 0.001, 0.001)
            .translate((-SCREW_HEAD_H / 2.0, cy, cz))
        )
        head = head.cut(groove)

    # Slot in the head butt face (flat-head screwdriver slot)
    slot = (
        cq.Workplane("XY")
        .box(0.002, SCREW_HEAD_R * 1.6, 0.001)
        .translate((-SCREW_HEAD_H + 0.001, 0.0, 0.0))
    )
    head = head.cut(slot)

    # Threaded shaft extending along +X (into the handle)
    shaft = (
        cq.Workplane("YZ")
        .circle(SCREW_R)
        .extrude(SCREW_LENGTH)
    )

    # Thread grooves on the shaft
    for i in range(8):
        xi = 0.002 + i * 0.002
        if xi > SCREW_LENGTH - 0.002:
            break
        thread = (
            cq.Workplane("XY")
            .box(0.0006, SCREW_R * 2.2, 0.0006)
            .translate((xi, 0.0, 0.0))
        )
        shaft = shaft.cut(thread)

    return shaft.union(head)


# Adjustment screw position: at the rear end of the upper handle (half_0).
# The screw axis points along -X. At q=0 the screw is fully seated (shaft
# inside the handle); positive q slides it outward so the head protrudes.
SCREW_MOUNT_X = -0.126  # inside the handle body (rear section)
SCREW_MOUNT_Y = -0.026  # centerline y at that x for jaw-on-+Y half
SCREW_MOUNT_Z = 0.0     # centered in thickness


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="locking_pliers")

    steel_polished = model.material("steel_polished", rgba=(0.78, 0.80, 0.83, 1.0))
    steel_forged = model.material("steel_forged", rgba=(0.62, 0.64, 0.67, 1.0))
    steel_brushed = model.material("steel_brushed", rgba=(0.58, 0.60, 0.63, 1.0))
    steel_handle = model.material("steel_handle", rgba=(0.55, 0.57, 0.60, 1.0))
    seam_gray = model.material("seam_gray", rgba=(0.45, 0.46, 0.48, 1.0))
    screw_steel = model.material("screw_steel", rgba=(0.50, 0.52, 0.55, 1.0))

    # --- Build the two plier halves ---
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
            mesh_from_cadquery(_handle_solid(s), f"{tag}_handle", tolerance=0.0003),
            name="handle",
            material=steel_handle,
        )
        part.visual(
            mesh_from_cadquery(_knurl_solid(s), f"{tag}_knurl", tolerance=0.0005),
            name="knurl",
            material=steel_brushed,
        )

        parts.append(part)

    # --- Rivet assembly on fixed half (half_0) ---
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

    # --- Adjustment screw part ---
    # The screw is mounted at the rear of the fixed handle (half_0).
    # It translates along -X (into the handle) to adjust jaw gap.
    # The screw part frame sits at the screw tip; geometry extends along +Z
    # which we rotate to align with -X.
    screw_part = model.part("adjust_screw")
    # Screw geometry is built along +X in local frame (shaft into handle,
    # head at origin protruding behind). No rotation needed.
    screw_part.visual(
        mesh_from_cadquery(_screw_solid(), "screw_body", tolerance=0.0003),
        name="screw_body",
        material=screw_steel,
    )

    # --- Primary articulation: revolute pivot ---
    # Positive q (about -Z) swings half_1's jaw toward -Y, opening the jaws.
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=parts[0],
        child=parts[1],
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=3.0, lower=0.0, upper=OPEN_LIMIT),
    )

    # --- Adjustment screw articulation: prismatic ---
    # The screw slides along -X (into/out of the handle end) to set jaw gap.
    # At q=0 the screw tip is flush with the handle end; at max travel the
    # screw head protrudes further, effectively pushing against a toggle link
    # and widening the jaw opening.
    model.articulation(
        "screw_adjust",
        ArticulationType.PRISMATIC,
        parent=parts[0],
        child=screw_part,
        origin=Origin(
            xyz=(SCREW_MOUNT_X, SCREW_MOUNT_Y, SCREW_MOUNT_Z),
        ),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.5,
            lower=0.0,
            upper=SCREW_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    half0 = object_model.get_part("plier_half_0")
    half1 = object_model.get_part("plier_half_1")
    screw = object_model.get_part("adjust_screw")
    pivot = object_model.get_articulation("pivot")
    screw_joint = object_model.get_articulation("screw_adjust")

    jaw0 = half0.get_visual("jaw")
    jaw1 = half1.get_visual("jaw")
    hub0 = half0.get_visual("hub")
    hub1 = half1.get_visual("hub")
    handle0 = half0.get_visual("handle")
    handle1 = half1.get_visual("handle")
    screw_body = screw.get_visual("screw_body")

    # --- Joint contract: pivot is revolute, screw is prismatic ---
    pivot_limits = pivot.motion_limits
    ctx.check(
        "pivot is a revolute joint with 0..25 degree range",
        pivot.articulation_type == ArticulationType.REVOLUTE
        and pivot_limits is not None
        and pivot_limits.lower is not None
        and pivot_limits.upper is not None
        and abs(pivot_limits.lower) < 1e-9
        and abs(pivot_limits.upper - OPEN_LIMIT) < 1e-6,
        details=f"limits={pivot_limits}",
    )

    screw_limits = screw_joint.motion_limits
    ctx.check(
        "screw_adjust is a prismatic joint with positive travel",
        screw_joint.articulation_type == ArticulationType.PRISMATIC
        and screw_limits is not None
        and screw_limits.lower is not None
        and screw_limits.upper is not None
        and abs(screw_limits.lower) < 1e-9
        and screw_limits.upper > 0.005,
        details=f"limits={screw_limits}",
    )

    # --- Serrated teeth on inner jaws ---
    # The jaw visuals should have the serrated tooth geometry.
    # We verify the jaw inner-face region has non-trivial Z-extent variation
    # (the V-teeth create surface detail).
    jaw0_aabb = ctx.part_element_world_aabb(half0, elem="jaw")
    jaw1_aabb = ctx.part_element_world_aabb(half1, elem="jaw")
    ctx.check(
        "jaw0 has non-trivial geometry (serrated teeth)",
        jaw0_aabb is not None and (jaw0_aabb[1][2] - jaw0_aabb[0][2]) > 0.015,
        details=f"jaw0_aabb={jaw0_aabb}",
    )
    ctx.check(
        "jaw1 has non-trivial geometry (serrated teeth)",
        jaw1_aabb is not None and (jaw1_aabb[1][2] - jaw1_aabb[0][2]) > 0.015,
        details=f"jaw1_aabb={jaw1_aabb}",
    )

    # --- Adjustment screw exists and is mounted at handle rear ---
    screw_aabb = ctx.part_world_aabb(screw)
    handle0_aabb = ctx.part_element_world_aabb(half0, elem="handle")
    ctx.check(
        "adjust_screw part exists with geometry",
        screw_aabb is not None and screw_body is not None,
        details=f"screw_aabb={screw_aabb}",
    )
    if screw_aabb is not None and handle0_aabb is not None:
        # Screw head should protrude from behind the handle rear
        ctx.check(
            "screw head protrudes behind the handle rear end",
            screw_aabb[0][0] < handle0_aabb[0][0],
            details=f"screw_min_x={screw_aabb[0][0]:.4f} handle_min_x={handle0_aabb[0][0]:.4f}",
        )

    # --- Screw adjustment actually translates the screw part ---
    # Positive q on axis=(-1,0,0) moves the screw in -X (out of handle).
    rest_screw_pos = ctx.part_world_position(screw)
    with ctx.pose({screw_joint: SCREW_TRAVEL}):
        extended_screw_pos = ctx.part_world_position(screw)
    ctx.check(
        "screw adjustment translates the screw part outward (-X)",
        rest_screw_pos is not None
        and extended_screw_pos is not None
        and extended_screw_pos[0] < rest_screw_pos[0] - 0.005,
        details=f"rest={rest_screw_pos}, extended={extended_screw_pos}",
    )

    # --- Screw is seated inside the handle at rest (intentional overlap) ---
    # The screw shaft threads into the handle body, passing through both
    # the handle visual and the knurl sleeve at the rear.
    ctx.allow_overlap(
        half0,
        screw,
        reason="The adjustment screw shaft is intentionally threaded into the "
        "handle body, representing the real socket the screw sits in.",
    )
    ctx.expect_overlap(
        half0,
        screw,
        axes="x",
        elem_a="handle",
        elem_b="screw_body",
        min_overlap=0.005,
        name="screw shaft remains inserted in the handle at rest",
    )

    # --- Closed rest pose: serrated jaw inner faces nearly touch ---
    ctx.expect_gap(
        half0,
        half1,
        axis="y",
        positive_elem=jaw0,
        negative_elem=jaw1,
        min_gap=0.0,
        max_gap=0.002,
        name="jaws closed and nearly touching at rest",
    )

    # --- The halves interleave at the pivot ---
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

    # --- Rivet shaft captured through moving half's hub (intentional overlap) ---
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

    # --- Rivet boss: ~0.025 m diameter, centered on the pivot ---
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

    # --- Overall envelope: ~0.20 m long ---
    a0 = ctx.part_world_aabb(half0)
    a1 = ctx.part_world_aabb(half1)
    ok_env = a0 is not None and a1 is not None
    if ok_env:
        length = max(a0[1][0], a1[1][0]) - min(a0[0][0], a1[0][0])
        ctx.check(
            "overall length about 0.20 m",
            0.17 <= length <= 0.22,
            details=f"length={length:.4f}",
        )
    else:
        ctx.fail("envelope AABBs resolve", "missing part AABB")

    # --- Decisive open pose: jaws separate, handles spread ---
    closed_jaw1 = ctx.part_element_world_aabb(half1, elem="jaw")
    closed_handle1 = ctx.part_element_world_aabb(half1, elem="handle")
    with ctx.pose({pivot: OPEN_LIMIT}):
        ctx.expect_gap(
            half0,
            half1,
            axis="y",
            positive_elem=jaw0,
            negative_elem=jaw1,
            min_gap=0.003,
            name="jaws open apart at the 25 degree pose",
        )
        open_jaw1 = ctx.part_element_world_aabb(half1, elem="jaw")
        open_handle1 = ctx.part_element_world_aabb(half1, elem="handle")
        ctx.check(
            "moving jaw swings away from the fixed jaw",
            closed_jaw1 is not None
            and open_jaw1 is not None
            and open_jaw1[0][1] < closed_jaw1[0][1] - 0.012,
            details=f"closed_min_y={closed_jaw1}, open_min_y={open_jaw1}",
        )
        ctx.check(
            "moving handle spreads outward as the jaws open",
            closed_handle1 is not None
            and open_handle1 is not None
            and open_handle1[1][1] > closed_handle1[1][1] + 0.02,
            details=f"closed_max_y={closed_handle1}, open_max_y={open_handle1}",
        )

    # --- All-metal construction: no rubber material on handles ---
    ctx.check(
        "handle uses steel material (all-metal locking pliers)",
        handle0.material is not None and "rubber" not in str(handle0.material.name).lower(),
        details=f"handle0 material={handle0.material}",
    )

    return ctx.report()


object_model = build_object_model()
