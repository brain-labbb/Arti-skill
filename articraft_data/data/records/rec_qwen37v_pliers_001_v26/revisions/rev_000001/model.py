from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Bent-nose pliers (electronics pliers with angled jaws).
#
# Two mirrored half-tools cross at a central pivot with circular rivet caps
# on both sides. Each half is: a bent-nose jaw (straight section then ~45°
# bend near the tip) with visible cutter bevel wedges, a slim steel neck,
# and a curved over-molded handle. A small latch folds over the handles on
# its own revolute joint.
#
# Geometry is authored per half in a "closed-design" local frame.
# The rest pose (q=0) splays each half by HALF_OPEN; positive q closes jaws.
# ---------------------------------------------------------------------------

HALF_OPEN = math.radians(12.5)
CLOSE_TRAVEL = 2.0 * HALF_OPEN  # ~25 degrees closing

PLATE_T = 0.0035

# Steel layer stack
ZL0, ZL1 = 0.0035, 0.0035 + PLATE_T
ZU0, ZU1 = ZL1, ZL1 + PLATE_T

BLADE_Z0, BLADE_Z1 = ZL0, ZU1
BLADE_X_MIN = 0.0085

BOSS_R = 0.0075
HOLE_R = 0.0028
SHANK_R = 0.0024
HEAD_R = 0.0045  # slightly larger for visible caps

# Pivot cap geometry (circular discs on both sides)
CAP_R = 0.0060
CAP_T = 0.0012

EDGE_LAND = 0.0003

# Handle over-mold
GRIP_H = 0.0105
GRIP_LZ0 = 0.0
GRIP_LZ1 = GRIP_LZ0 + GRIP_H
GRIP_UZ0 = GRIP_LZ0 + (ZU0 - ZL0)
GRIP_UZ1 = GRIP_UZ0 + GRIP_H
INLAY_T = 0.0007
INLAY_EMBED = 0.0002

# Bent-nose jaw: straight section then 45° angled tip
# Lower half jaw at +y side; the bend occurs around x=0.025
BEND_ANGLE = math.radians(45)
JAW_PTS = [
    (0.0000, 0.0080),
    (0.0060, 0.0082),
    (0.0140, 0.0076),
    (0.0220, 0.0060),
    # bend region - jaw angles toward center
    (0.0280, 0.0040),
    (0.0330, 0.0020),
    (0.0360, 0.0008),
    (0.0355, EDGE_LAND),
    (0.0280, 0.0018),
    (0.0220, 0.0032),
    (0.0140, 0.0046),
    (0.0090, EDGE_LAND),
    (0.0020, 0.0042),
]

# Slim steel neck/tang
TANG_PTS = [
    (0.0020, -0.0032),
    (-0.0100, -0.0044),
    (-0.0200, -0.0052),
    (-0.0300, -0.0062),
    (-0.0300, -0.0112),
    (-0.0200, -0.0096),
    (-0.0100, -0.0082),
    (0.0008, -0.0072),
]

# Curved over-molded handle
GRIP_PTS = [
    (-0.0240, -0.0038),
    (-0.0400, -0.0048),
    (-0.0580, -0.0060),
    (-0.0740, -0.0072),
    (-0.0890, -0.0080),
    (-0.0955, -0.0093),
    (-0.0915, -0.0128),
    (-0.0780, -0.0148),
    (-0.0620, -0.0152),
    (-0.0460, -0.0138),
    (-0.0300, -0.0112),
    (-0.0235, -0.0095),
]

# Inlay strip
INLAY_PTS = [
    (-0.0305, -0.0068),
    (-0.0480, -0.0082),
    (-0.0660, -0.0095),
    (-0.0790, -0.0102),
    (-0.0860, -0.0106),
    (-0.0810, -0.0116),
    (-0.0670, -0.0122),
    (-0.0510, -0.0114),
    (-0.0370, -0.0100),
    (-0.0315, -0.0090),
]

# Latch dimensions
LATCH_LENGTH = 0.032
LATCH_WIDTH = 0.008
LATCH_THICK = 0.0020
# Pivot on top of lower handle, near its end.
# Lower grip AABB extends roughly x: -0.092 to -0.019, y: -0.036 to -0.009, z: 0 to 0.0105.
# The latch pivot sits on the grip top surface near the far end of the handle.
LATCH_PIVOT_X = -0.078
LATCH_PIVOT_Y = -0.022
LATCH_PIVOT_Z = GRIP_LZ1 - LATCH_THICK * 0.3  # slightly embedded into grip top for seating


def _mirror(pts: list[tuple[float, float]], s: float) -> list[tuple[float, float]]:
    return [(x, s * y) for (x, y) in pts]


def _poly_prism(pts: list[tuple[float, float]], z0: float, z1: float) -> cq.Workplane:
    return (
        cq.Workplane("XY", origin=(0.0, 0.0, z0))
        .polyline(pts)
        .close()
        .extrude(z1 - z0)
    )


def _spline_wire(pts: list[tuple[float, float]], z: float, inset: float = 0.0) -> cq.Wire:
    edge = cq.Edge.makeSpline([cq.Vector(x, y, z) for (x, y) in pts], periodic=True)
    wire = cq.Wire.assembleEdges([edge])
    if inset:
        wire = wire.offset2D(-inset)[0]
    return wire


def _soft_prism(
    pts: list[tuple[float, float]],
    z0: float,
    z1: float,
    cap: float = 0.0020,
    inset: float = 0.0018,
) -> cq.Workplane:
    mid = cq.Solid.extrudeLinear(
        cq.Face.makeFromWires(_spline_wire(pts, z0 + cap)),
        cq.Vector(0.0, 0.0, (z1 - z0) - 2.0 * cap),
    )
    top = cq.Solid.makeLoft([_spline_wire(pts, z1 - cap), _spline_wire(pts, z1, inset=inset)])
    bot = cq.Solid.makeLoft([_spline_wire(pts, z0, inset=inset), _spline_wire(pts, z0 + cap)])
    return cq.Workplane(obj=mid.fuse(top).fuse(bot))


def _spline_prism(pts: list[tuple[float, float]], z0: float, z1: float) -> cq.Workplane:
    face = cq.Face.makeFromWires(_spline_wire(pts, z0))
    return cq.Workplane(obj=cq.Solid.extrudeLinear(face, cq.Vector(0.0, 0.0, z1 - z0)))


def _bevel_wedge(s: float) -> cq.Workplane:
    """Cutter bevel wedge: visible angled geometry on the jaw inner face."""
    tri = [
        (s * EDGE_LAND, BLADE_Z0 + 0.0004),
        (s * 0.0048, BLADE_Z1 - 0.0004),
        (s * EDGE_LAND, BLADE_Z1 - 0.0004),
    ]
    return (
        cq.Workplane("YZ", origin=(BLADE_X_MIN, 0.0, 0.0))
        .polyline(tri)
        .close()
        .extrude(0.0310)
    )


def _cutter_bevel_visible(s: float) -> cq.Workplane:
    """Visible cutter bevel wedge geometry - a triangular prism along the jaw."""
    # Wedge cross-section: thin at cutting edge, thick at jaw body
    wedge_pts = [
        (s * EDGE_LAND, BLADE_Z0 + 0.0005),
        (s * 0.0035, BLADE_Z0 + 0.0025),
        (s * 0.0035, BLADE_Z0 + 0.0005),
    ]
    return (
        cq.Workplane("YZ", origin=(0.0100, 0.0, 0.0))
        .polyline(wedge_pts)
        .close()
        .extrude(0.0240)
    )


def _blade_clip_box() -> cq.Workplane:
    return cq.Workplane("XY", origin=(BLADE_X_MIN + 0.0250, 0.0, 0.0070)).box(
        0.0500, 0.0600, 0.0200
    )


def _half_blade(s: float, own_z0: float, own_z1: float) -> cq.Workplane:
    prof = _mirror(JAW_PTS, s)
    full = _poly_prism(prof, BLADE_Z0, BLADE_Z1).intersect(_blade_clip_box())
    rear = _poly_prism(prof, own_z0, own_z1)
    return full.union(rear).cut(_bevel_wedge(s))


def _half_boss(own_z0: float, own_z1: float, with_hole: bool) -> cq.Workplane:
    boss = cq.Workplane("XY", origin=(0.0, 0.0, own_z0)).circle(BOSS_R).extrude(
        own_z1 - own_z0
    )
    if with_hole:
        hole = cq.Workplane("XY", origin=(0.0, 0.0, own_z0 - 0.001)).circle(HOLE_R).extrude(
            (own_z1 - own_z0) + 0.002
        )
        boss = boss.cut(hole)
    return boss


def _pivot_cap(z_base: float) -> cq.Workplane:
    """Circular pivot rivet cap disc."""
    cap = cq.Workplane("XY", origin=(0.0, 0.0, z_base)).circle(CAP_R).extrude(CAP_T)
    try:
        cap = cap.edges(">Z").fillet(0.0004)
    except Exception:
        pass
    return cap


def _rivet() -> cq.Workplane:
    lower_head = cq.Workplane("XY", origin=(0.0, 0.0, 0.0021)).circle(HEAD_R).extrude(0.0019)
    shank = cq.Workplane("XY", origin=(0.0, 0.0, 0.0021)).circle(SHANK_R).extrude(0.0100)
    upper_head = cq.Workplane("XY", origin=(0.0, 0.0, ZU1 + 0.0001)).circle(HEAD_R).extrude(
        0.0014
    )
    rivet = lower_head.union(shank).union(upper_head)
    try:
        rivet = rivet.edges(">Z").fillet(0.0009)
    except Exception:
        pass
    return rivet


def _latch_body() -> cq.Workplane:
    """Latch: a thin rounded strip that folds over the handles."""
    # Rounded rectangle strip
    hw = LATCH_WIDTH / 2.0
    hl = LATCH_LENGTH / 2.0
    r = 0.002
    pts = [
        (-hl + r, -hw),
        (hl - r, -hw),
        (hl, -hw + r),
        (hl, hw - r),
        (hl - r, hw),
        (-hl + r, hw),
        (-hl, hw - r),
        (-hl, -hw + r),
    ]
    body = (
        cq.Workplane("XY", origin=(0.0, 0.0, -LATCH_THICK / 2.0))
        .polyline(pts)
        .close()
        .extrude(LATCH_THICK)
    )
    # Add a small pivot hole indicator (bore at one end)
    hole = (
        cq.Workplane("XY", origin=(-hl + 0.003, 0.0, -LATCH_THICK - 0.001))
        .circle(0.0015)
        .extrude(LATCH_THICK + 0.002)
    )
    body = body.cut(hole)
    # Add a small hook/catch nub at the free end
    nub = (
        cq.Workplane("XY", origin=(hl - 0.004, -0.002, -LATCH_THICK / 2.0))
        .box(0.004, 0.004, LATCH_THICK + 0.001)
    )
    body = body.union(nub)
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bent_nose_pliers")

    steel = model.material("brushed_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    polished = model.material("polished_steel", rgba=(0.86, 0.87, 0.90, 1.0))
    orange = model.material("orange_grip", rgba=(0.97, 0.52, 0.07, 1.0))
    peach = model.material("peach_inlay", rgba=(1.0, 0.80, 0.64, 0.78))
    dark_steel = model.material("dark_steel", rgba=(0.45, 0.46, 0.48, 1.0))

    # ----- lower half (base link)
    lower = model.part("lower_half")
    lower_pose = Origin(rpy=(0.0, 0.0, HALF_OPEN))

    lower.visual(
        mesh_from_cadquery(_half_blade(+1.0, ZL0, ZL1), "lower_blade"),
        origin=lower_pose,
        material=steel,
        name="jaw",
    )
    lower.visual(
        mesh_from_cadquery(_half_boss(ZL0, ZL1, with_hole=False), "lower_boss"),
        origin=lower_pose,
        material=steel,
        name="pivot_boss",
    )
    lower.visual(
        mesh_from_cadquery(_poly_prism(_mirror(TANG_PTS, +1.0), ZL0, ZL1), "lower_tang"),
        origin=lower_pose,
        material=steel,
        name="neck_tang",
    )
    lower.visual(
        mesh_from_cadquery(_soft_prism(_mirror(GRIP_PTS, +1.0), GRIP_LZ0, GRIP_LZ1), "lower_grip"),
        origin=lower_pose,
        material=orange,
        name="grip",
    )
    lower.visual(
        mesh_from_cadquery(
            _spline_prism(
                _mirror(INLAY_PTS, +1.0), GRIP_LZ1 - INLAY_EMBED, GRIP_LZ1 - INLAY_EMBED + INLAY_T
            ),
            "lower_inlay",
        ),
        origin=lower_pose,
        material=peach,
        name="grip_inlay",
    )
    lower.visual(
        mesh_from_cadquery(_rivet(), "rivet"),
        origin=lower_pose,
        material=polished,
        name="rivet",
    )
    # Pivot cap on lower side (visible circular disc)
    lower.visual(
        mesh_from_cadquery(_pivot_cap(ZL0 - CAP_T), "lower_cap"),
        origin=lower_pose,
        material=polished,
        name="pivot_cap_lower",
    )
    # Visible cutter bevel wedge on lower jaw
    lower.visual(
        mesh_from_cadquery(_cutter_bevel_visible(+1.0), "lower_bevel"),
        origin=lower_pose,
        material=dark_steel,
        name="cutter_bevel",
    )

    # ----- upper half (moving link)
    upper = model.part("upper_half")

    upper.visual(
        mesh_from_cadquery(_half_blade(-1.0, ZU0, ZU1), "upper_blade"),
        material=steel,
        name="jaw",
    )
    upper.visual(
        mesh_from_cadquery(_half_boss(ZU0, ZU1, with_hole=True), "upper_boss"),
        material=steel,
        name="pivot_boss",
    )
    upper.visual(
        mesh_from_cadquery(_poly_prism(_mirror(TANG_PTS, -1.0), ZU0, ZU1), "upper_tang"),
        material=steel,
        name="neck_tang",
    )
    upper.visual(
        mesh_from_cadquery(_soft_prism(_mirror(GRIP_PTS, -1.0), GRIP_UZ0, GRIP_UZ1), "upper_grip"),
        material=orange,
        name="grip",
    )
    upper.visual(
        mesh_from_cadquery(
            _spline_prism(
                _mirror(INLAY_PTS, -1.0), GRIP_UZ1 - INLAY_EMBED, GRIP_UZ1 - INLAY_EMBED + INLAY_T
            ),
            "upper_inlay",
        ),
        material=peach,
        name="grip_inlay",
    )
    # Pivot cap on upper side (visible circular disc)
    upper.visual(
        mesh_from_cadquery(_pivot_cap(ZU1), "upper_cap"),
        material=polished,
        name="pivot_cap_upper",
    )
    # Visible cutter bevel wedge on upper jaw
    upper.visual(
        mesh_from_cadquery(_cutter_bevel_visible(-1.0), "upper_bevel"),
        material=dark_steel,
        name="cutter_bevel",
    )

    # ----- latch part: folds over the handles
    latch = model.part("latch")
    # Latch stowed position: extends along +X from pivot (toward tool center).
    # Visual origin is relative to the latch part frame (set by articulation).
    # Shift so the bore end (x = -hl + 0.003) sits at the part frame origin.
    latch_x_offset = LATCH_LENGTH / 2.0 - 0.003  # ~0.013
    latch.visual(
        mesh_from_cadquery(_latch_body(), "latch_body"),
        origin=Origin(xyz=(latch_x_offset, 0.0, 0.0)),
        material=dark_steel,
        name="latch_body",
    )

    # ----- main pivot articulation: lower -> upper
    model.articulation(
        "rivet_pivot",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, -HALF_OPEN)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=4.0, lower=0.0, upper=CLOSE_TRAVEL),
    )

    # ----- latch articulation: folds over handles
    # Latch pivots on the lower half about the Z axis (perpendicular to tool plane).
    # At q=0 the latch is stowed along the lower handle (extending toward tool center).
    # Positive q swings the free end toward +Y (bridging across toward the upper handle).
    latch_stow_angle = 0.0
    latch_locked_angle = math.radians(85)  # folds ~85° to bridge across handles
    model.articulation(
        "latch_hinge",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=latch,
        origin=Origin(xyz=(LATCH_PIVOT_X, LATCH_PIVOT_Y, LATCH_PIVOT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=latch_stow_angle, upper=latch_locked_angle
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower = object_model.get_part("lower_half")
    upper = object_model.get_part("upper_half")
    latch = object_model.get_part("latch")
    pivot = object_model.get_articulation("rivet_pivot")
    latch_joint = object_model.get_articulation("latch_hinge")

    # --- pivot stack: bosses coaxial, upper sits above lower
    ctx.expect_overlap(
        lower,
        upper,
        axes="xy",
        elem_a="pivot_boss",
        elem_b="pivot_boss",
        min_overlap=0.012,
        name="pivot bosses stack coaxially",
    )
    ctx.expect_gap(
        upper,
        lower,
        axis="z",
        positive_elem="pivot_boss",
        negative_elem="pivot_boss",
        min_gap=-0.00002,
        max_gap=0.0006,
        name="upper boss sits slightly above lower boss",
    )

    # --- circular pivot caps present on both sides
    lcap = ctx.part_element_world_aabb(lower, elem="pivot_cap_lower")
    ucap = ctx.part_element_world_aabb(upper, elem="pivot_cap_upper")
    ctx.check(
        "lower pivot cap exists below pivot",
        lcap is not None and lcap[0][2] < ZL0 - 0.0002,
        details=f"lower_cap={lcap}",
    )
    ctx.check(
        "upper pivot cap exists above pivot",
        ucap is not None and ucap[1][2] > ZU1 + 0.0002,
        details=f"upper_cap={ucap}",
    )
    # Caps should be circular and centered on pivot
    if lcap is not None and ucap is not None:
        cap_dx_l = lcap[1][0] - lcap[0][0]
        cap_dy_l = lcap[1][1] - lcap[0][1]
        ctx.check(
            "lower cap roughly circular (similar X/Y extents)",
            abs(cap_dx_l - cap_dy_l) < 0.003,
            details=f"dx={cap_dx_l:.4f}, dy={cap_dy_l:.4f}",
        )

    # --- cutter bevels present as visible wedge geometry on both jaws
    lbevel = ctx.part_element_world_aabb(lower, elem="cutter_bevel")
    ubevel = ctx.part_element_world_aabb(upper, elem="cutter_bevel")
    ctx.check(
        "lower jaw has visible cutter bevel wedge",
        lbevel is not None and (lbevel[1][2] - lbevel[0][2]) > 0.001,
        details=f"lower_bevel={lbevel}",
    )
    ctx.check(
        "upper jaw has visible cutter bevel wedge",
        ubevel is not None and (ubevel[1][2] - ubevel[0][2]) > 0.001,
        details=f"upper_bevel={ubevel}",
    )
    # Bevels should overlap with jaw in XY (they are on the jaw)
    if lbevel is not None:
        ljaw = ctx.part_element_world_aabb(lower, elem="jaw")
        ctx.check(
            "lower bevel within jaw region",
            ljaw is not None
            and lbevel[0][0] >= ljaw[0][0] - 0.002
            and lbevel[1][0] <= ljaw[1][0] + 0.002,
            details=f"bevel={lbevel}, jaw={ljaw}",
        )

    # --- latch part exists and contacts lower handle
    latch_body = ctx.part_element_world_aabb(latch, elem="latch_body")
    ctx.check(
        "latch body exists",
        latch_body is not None,
        details=f"latch_body={latch_body}",
    )
    # Latch should overlap with the lower grip (seated on handle)
    if latch_body is not None:
        lgrip_aabb = ctx.part_element_world_aabb(lower, elem="grip")
        ctx.check(
            "latch positioned on lower handle",
            lgrip_aabb is not None
            and latch_body[0][0] >= lgrip_aabb[0][0] - 0.002
            and latch_body[1][0] <= lgrip_aabb[1][0] + 0.002
            and latch_body[0][1] >= lgrip_aabb[0][1] - 0.002
            and latch_body[1][1] <= lgrip_aabb[1][1] + 0.002,
            details=f"latch={latch_body}, grip={lgrip_aabb}",
        )

    # Allow small seating overlap between latch and lower grip
    ctx.allow_overlap(
        lower,
        latch,
        elem_a="grip",
        elem_b="latch_body",
        reason="Latch is seated on top of the lower handle grip surface with slight embed for visual contact.",
    )
    ctx.expect_contact(
        lower,
        latch,
        elem_a="grip",
        elem_b="latch_body",
        contact_tol=0.002,
        name="latch contacts lower grip when stowed",
    )

    # --- latch hinge joint is non-fixed with proper limits
    latch_limits = latch_joint.motion_limits
    ctx.check(
        "latch hinge has non-trivial travel range",
        latch_limits is not None
        and latch_limits.upper is not None
        and latch_limits.upper > 0.5,
        details=f"latch_limits={latch_limits}",
    )

    # --- latch articulation moves the latch body
    latch_rest_aabb = ctx.part_element_world_aabb(latch, elem="latch_body")
    with ctx.pose({latch_joint: latch_joint.motion_limits.upper}):
        latch_folded_aabb = ctx.part_element_world_aabb(latch, elem="latch_body")
    ctx.check(
        "latch moves when hinge is actuated",
        latch_rest_aabb is not None
        and latch_folded_aabb is not None
        and (
            abs(latch_folded_aabb[0][0] - latch_rest_aabb[0][0]) > 0.003
            or abs(latch_folded_aabb[0][1] - latch_rest_aabb[0][1]) > 0.003
            or abs(latch_folded_aabb[1][0] - latch_rest_aabb[1][0]) > 0.003
            or abs(latch_folded_aabb[1][1] - latch_rest_aabb[1][1]) > 0.003
        ),
        details=f"rest={latch_rest_aabb}, folded={latch_folded_aabb}",
    )

    # --- rest pose: jaws open, handles splayed
    ctx.expect_gap(
        lower,
        upper,
        axis="y",
        positive_elem="jaw",
        negative_elem="jaw",
        min_gap=0.002,
        name="jaw tips are open at rest",
    )
    ctx.expect_gap(
        upper,
        lower,
        axis="y",
        positive_elem="grip",
        negative_elem="grip",
        min_gap=0.012,
        name="handles splay apart at rest",
    )

    # --- rivet passes through boss stack
    ctx.expect_within(
        lower,
        upper,
        axes="xy",
        inner_elem="rivet",
        outer_elem="pivot_boss",
        margin=0.0002,
        name="rivet centered through the boss bore",
    )

    # --- bent-nose jaws: tips should be closer together in Y than jaw roots
    # (showing the angled bend)
    ljaw = ctx.part_element_world_aabb(lower, elem="jaw")
    ujaw = ctx.part_element_world_aabb(upper, elem="jaw")
    if ljaw is not None and ujaw is not None:
        # The jaws should span a reasonable length
        jaw_length = max(ljaw[1][0], ujaw[1][0]) - min(ljaw[0][0], ujaw[0][0])
        ctx.check(
            "bent-nose jaws have reasonable length",
            jaw_length > 0.025,
            details=f"jaw_span={jaw_length:.4f}",
        )

    # --- main pivot articulation works
    limits = pivot.motion_limits
    ctx.check(
        "pivot travel is roughly 0..25 degrees",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower) < 1e-9
        and 0.38 <= limits.upper <= 0.48,
        details=f"limits={limits}",
    )

    open_blade = ctx.part_element_world_aabb(upper, elem="jaw")
    open_grip = ctx.part_element_world_aabb(upper, elem="grip")
    with ctx.pose({pivot: CLOSE_TRAVEL}):
        ctx.expect_contact(
            lower,
            upper,
            elem_a="jaw",
            elem_b="jaw",
            contact_tol=0.003,
            name="jaw tips approach when fully closed",
        )
        closed_blade = ctx.part_element_world_aabb(upper, elem="jaw")
        closed_grip = ctx.part_element_world_aabb(upper, elem="grip")

    ctx.check(
        "closing swings upper jaw toward lower jaw",
        open_blade is not None
        and closed_blade is not None
        and closed_blade[1][1] > open_blade[1][1] + 0.003,
        details=f"open={open_blade}, closed={closed_blade}",
    )
    ctx.check(
        "handles scissor opposite to the jaws",
        open_grip is not None
        and closed_grip is not None
        and closed_grip[0][1] < open_grip[0][1] - 0.003,
        details=f"open={open_grip}, closed={closed_grip}",
    )

    # --- overall proportions
    la = ctx.part_world_aabb(lower)
    ua = ctx.part_world_aabb(upper)
    if la is not None and ua is not None:
        length = max(la[1][0], ua[1][0]) - min(la[0][0], ua[0][0])
        width = max(la[1][1], ua[1][1]) - min(la[0][1], ua[0][1])
        height = max(la[1][2], ua[1][2]) - min(la[0][2], ua[0][2])
        ctx.check(
            "overall length ~0.13 m",
            0.10 <= length <= 0.15,
            details=f"length={length:.4f}",
        )
        ctx.check(
            "splayed handle width ~0.06 m",
            0.04 <= width <= 0.08,
            details=f"width={width:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
