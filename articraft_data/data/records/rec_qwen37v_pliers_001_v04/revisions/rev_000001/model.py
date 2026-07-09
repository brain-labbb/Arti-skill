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
# Slip-joint pliers (6-inch, ~0.15 m overall).
#
# Two forged steel halves cross at a rivet pivot. Each half is one rigid
# link: flat gripping jaw -> neck/tang -> handle with slip-on grip sleeve.
#
# The lower half is the base link; the upper half rotates about the pivot
# axis (Z, perpendicular to the flat plane of the tool).
#
# Slip-joint feature: the lower half has an elongated slot near the pivot
# area that allows the pivot pin to sit in one of two positions (close or
# wide jaw opening). Circular rivet caps sit on both sides of the pivot.
#
# Grip sleeves are separate color-distinct rubber sleeves that slide onto
# the handle portions.
# ---------------------------------------------------------------------------

HALF_OPEN = math.radians(15.0)  # each half yawed this much at rest (open)
CLOSE_TRAVEL = 2.0 * HALF_OPEN  # ~30 degrees of closing travel

PLATE_T = 0.004  # forged steel plate thickness per half

# Steel layer stack (lower plate below upper plate at the pivot boss).
ZL0, ZL1 = 0.003, 0.003 + PLATE_T  # lower half: 0.003 .. 0.007
ZU0, ZU1 = ZL1, ZL1 + PLATE_T       # upper half: 0.007 .. 0.011

# Full-height jaw region spans both plate layers.
JAW_Z0, JAW_Z1 = ZL0, ZU1

BOSS_R = 0.008
RIVET_R = 0.003
CAP_R = 0.006   # pivot cap washer radius
CAP_T = 0.0012  # pivot cap thickness

# Slip-joint slot parameters (on lower half only).
SLOT_R = 0.0032  # radius of each circular position in the slot
SLOT_SPAN = 0.006  # center-to-center distance between two pivot positions

# Grip sleeve parameters.
GRIP_SLEEVE_T = 0.003  # sleeve wall thickness (adds to handle width)
GRIP_SLEEVE_LEN = 0.060  # sleeve length along handle


def _mirror_y(pts: list[tuple[float, float]], s: float) -> list[tuple[float, float]]:
    return [(x, s * y) for (x, y) in pts]


def _poly_prism(pts: list[tuple[float, float]], z0: float, z1: float) -> cq.Workplane:
    return (
        cq.Workplane("XY", origin=(0.0, 0.0, z0))
        .polyline(pts)
        .close()
        .extrude(z1 - z0)
    )


def _circle_prism(r: float, z0: float, z1: float) -> cq.Workplane:
    return cq.Workplane("XY", origin=(0.0, 0.0, z0)).circle(r).extrude(z1 - z0)


# ---- Flat gripping jaw outline (lower half, jaw body at +y side) ----
# Wider and flatter than a cutting jaw; rectangular cross-section with
# rounded nose and a slight taper toward the tip.
JAW_PTS = [
    (0.0000, 0.010),
    (0.008, 0.010),
    (0.020, 0.009),
    (0.032, 0.007),
    (0.040, 0.005),
    (0.044, 0.003),
    (0.042, 0.001),
    (0.008, 0.001),
    (0.000, 0.005),
]

# Serrated jaw face region (small raised pads on the inner gripping face).
JAW_FACE_PTS = [
    (0.010, 0.0015),
    (0.038, 0.0015),
    (0.038, 0.0045),
    (0.010, 0.0045),
]

# Neck/tang from the boss back to the handle.
TANG_PTS = [
    (0.002, -0.004),
    (-0.008, -0.005),
    (-0.018, -0.006),
    (-0.028, -0.006),
    (-0.028, -0.012),
    (-0.018, -0.011),
    (-0.008, -0.009),
    (0.002, -0.008),
]

# Handle body outline (steel core under the grip sleeve).
HANDLE_PTS = [
    (-0.022, -0.004),
    (-0.040, -0.005),
    (-0.060, -0.006),
    (-0.080, -0.0065),
    (-0.095, -0.007),
    (-0.100, -0.008),
    (-0.098, -0.013),
    (-0.085, -0.0145),
    (-0.068, -0.015),
    (-0.050, -0.014),
    (-0.035, -0.012),
    (-0.022, -0.010),
]

# Grip sleeve outline (slightly larger than handle, wraps around it).
SLEEVE_PTS = [
    (-0.026, -0.003),
    (-0.042, -0.004),
    (-0.062, -0.005),
    (-0.078, -0.0055),
    (-0.090, -0.006),
    (-0.094, -0.007),
    (-0.092, -0.014),
    (-0.082, -0.016),
    (-0.066, -0.0165),
    (-0.050, -0.0155),
    (-0.036, -0.0135),
    (-0.026, -0.011),
]

# Grip texture ridges (small raised strips along the sleeve).
RIDGE_PTS = [
    (-0.040, -0.0042),
    (-0.075, -0.0054),
    (-0.075, -0.0062),
    (-0.040, -0.0050),
]


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
    cap: float = 0.0015,
    inset: float = 0.0012,
) -> cq.Workplane:
    """Extruded spline outline with loft-capped soft bevels."""
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


def _half_jaw(s: float, own_z0: float, own_z1: float) -> cq.Workplane:
    """Flat gripping jaw for one half."""
    prof = _mirror_y(JAW_PTS, s)
    body = _poly_prism(prof, JAW_Z0, JAW_Z1)
    # Add serrated face pads on the inner gripping surface.
    face_prof = _mirror_y(JAW_FACE_PTS, s)
    # Serration pads are thin raised strips.
    pad_h = 0.0005
    if s > 0:
        pad = _poly_prism(face_prof, JAW_Z0, JAW_Z0 + pad_h)
    else:
        pad = _poly_prism(face_prof, JAW_Z1 - pad_h, JAW_Z1)
    return body.union(pad)


def _half_boss(own_z0: float, own_z1: float, with_slot: bool = False) -> cq.Workplane:
    """Pivot boss area. Lower half gets elongated slip-joint body with slot;
    upper half gets round boss with hole."""

    if with_slot:
        # Elongated (pill-shaped) boss to contain both pivot positions.
        # The boss extends from x=BOSS_R to x=-(SLOT_SPAN + BOSS_R) with
        # rounded ends.
        pill_half_len = SLOT_SPAN / 2.0 + BOSS_R
        # Build pill as union of two circles and a connecting rectangle.
        c1 = cq.Workplane("XY", origin=(0.0, 0.0, own_z0)).circle(BOSS_R).extrude(own_z1 - own_z0)
        c2 = cq.Workplane("XY", origin=(-SLOT_SPAN, 0.0, own_z0)).circle(BOSS_R).extrude(own_z1 - own_z0)
        rect = (
            cq.Workplane("XY", origin=(-SLOT_SPAN / 2.0, 0.0, own_z0))
            .rect(SLOT_SPAN, 2.0 * BOSS_R)
            .extrude(own_z1 - own_z0)
        )
        boss = c1.union(c2).union(rect)

        # Cut the slip-joint slot through the boss.
        slot_h = (own_z1 - own_z0) + 0.002
        z_mid = own_z0 - 0.001
        hole1 = _circle_prism(SLOT_R, z_mid, z_mid + slot_h)
        hole2 = cq.Workplane("XY", origin=(-SLOT_SPAN, 0.0, z_mid)).circle(SLOT_R).extrude(slot_h)
        channel = (
            cq.Workplane("XY", origin=(-SLOT_SPAN / 2.0, 0.0, z_mid))
            .rect(SLOT_SPAN, 2.0 * SLOT_R * 0.8)
            .extrude(slot_h)
        )
        slot = hole1.union(hole2).union(channel)
        boss = boss.cut(slot)
    else:
        boss = _circle_prism(BOSS_R, own_z0, own_z1)
        # Round hole for pivot pin.
        hole = _circle_prism(RIVET_R, own_z0 - 0.001, own_z1 + 0.001)
        boss = boss.cut(hole)

    return boss


def _pivot_assembly() -> cq.Workplane:
    """Combined rivet pin + lower cap + upper cap as one connected mesh.
    The pin connects the two caps; the whole assembly floats in the slot."""
    # Rivet shank spans from below lower cap to above upper cap.
    shank = _circle_prism(RIVET_R * 0.9, ZL0 - CAP_T - 0.0002, ZU1 + CAP_T + 0.0006)
    # Lower cap disc.
    lower_cap = (
        cq.Workplane("XY", origin=(0.0, 0.0, ZL0 - CAP_T - 0.0002))
        .circle(CAP_R)
        .extrude(CAP_T)
    )
    # Upper cap disc.
    upper_cap = (
        cq.Workplane("XY", origin=(0.0, 0.0, ZU1 + 0.0006))
        .circle(CAP_R)
        .extrude(CAP_T)
    )
    # Small central raised boss on each cap.
    lower_boss = (
        cq.Workplane("XY", origin=(0.0, 0.0, ZL0 - CAP_T - 0.0002))
        .circle(RIVET_R * 1.2)
        .extrude(CAP_T + 0.0005)
    )
    upper_boss = (
        cq.Workplane("XY", origin=(0.0, 0.0, ZU1 + 0.0006))
        .circle(RIVET_R * 1.2)
        .extrude(CAP_T + 0.0005)
    )
    return shank.union(lower_cap).union(upper_cap).union(lower_boss).union(upper_boss)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slip_joint_pliers")

    # Materials.
    steel = model.material("forged_steel", rgba=(0.55, 0.56, 0.58, 1.0))
    chrome = model.material("chrome_cap", rgba=(0.82, 0.83, 0.85, 1.0))
    blue_grip = model.material("blue_rubber", rgba=(0.12, 0.35, 0.72, 1.0))
    red_accent = model.material("red_accent", rgba=(0.85, 0.15, 0.12, 1.0))

    # ----- Lower half (base link): jaw at +y, handle at -y, lower steel layer.
    lower = model.part("lower_half")
    lower_pose = Origin(rpy=(0.0, 0.0, HALF_OPEN))

    lower.visual(
        mesh_from_cadquery(_half_jaw(+1.0, ZL0, ZL1), "lower_jaw"),
        origin=lower_pose,
        material=steel,
        name="jaw",
    )
    lower.visual(
        mesh_from_cadquery(_half_boss(ZL0, ZL1, with_slot=True), "lower_boss"),
        origin=lower_pose,
        material=steel,
        name="pivot_boss",
    )
    lower.visual(
        mesh_from_cadquery(_poly_prism(_mirror_y(TANG_PTS, +1.0), ZL0, ZL1), "lower_tang"),
        origin=lower_pose,
        material=steel,
        name="neck",
    )
    lower.visual(
        mesh_from_cadquery(_poly_prism(_mirror_y(HANDLE_PTS, +1.0), ZL0, ZL1), "lower_handle"),
        origin=lower_pose,
        material=steel,
        name="handle_core",
    )
    # Grip sleeve (blue rubber, slightly larger than handle).
    grip_z0 = ZL0 - 0.001
    grip_z1 = ZL1 + 0.001
    lower.visual(
        mesh_from_cadquery(_soft_prism(_mirror_y(SLEEVE_PTS, +1.0), grip_z0, grip_z1), "lower_sleeve"),
        origin=lower_pose,
        material=blue_grip,
        name="grip_sleeve",
    )
    # Red accent stripe on the grip sleeve.
    stripe_pts = [
        (-0.040, -0.0044),
        (-0.075, -0.0056),
        (-0.075, -0.0064),
        (-0.040, -0.0052),
    ]
    stripe_z = ZL1 + 0.001
    lower.visual(
        mesh_from_cadquery(
            _poly_prism(_mirror_y(stripe_pts, +1.0), stripe_z, stripe_z + 0.0006),
            "lower_stripe",
        ),
        origin=lower_pose,
        material=red_accent,
        name="grip_stripe",
    )

    # Pivot assembly: rivet pin + caps as one connected mesh on the base half.
    lower.visual(
        mesh_from_cadquery(_pivot_assembly(), "pivot_assembly"),
        origin=lower_pose,
        material=chrome,
        name="pivot_assembly",
    )

    # ----- Upper half (moving link): mirrored, upper steel layer, round hole.
    upper = model.part("upper_half")

    upper.visual(
        mesh_from_cadquery(_half_jaw(-1.0, ZU0, ZU1), "upper_jaw"),
        material=steel,
        name="jaw",
    )
    upper.visual(
        mesh_from_cadquery(_half_boss(ZU0, ZU1, with_slot=False), "upper_boss"),
        material=steel,
        name="pivot_boss",
    )
    upper.visual(
        mesh_from_cadquery(_poly_prism(_mirror_y(TANG_PTS, -1.0), ZU0, ZU1), "upper_tang"),
        material=steel,
        name="neck",
    )
    upper.visual(
        mesh_from_cadquery(_poly_prism(_mirror_y(HANDLE_PTS, -1.0), ZU0, ZU1), "upper_handle"),
        material=steel,
        name="handle_core",
    )
    # Grip sleeve.
    grip_uz0 = ZU0 - 0.001
    grip_uz1 = ZU1 + 0.001
    upper.visual(
        mesh_from_cadquery(_soft_prism(_mirror_y(SLEEVE_PTS, -1.0), grip_uz0, grip_uz1), "upper_sleeve"),
        material=blue_grip,
        name="grip_sleeve",
    )
    # Red accent stripe.
    upper.visual(
        mesh_from_cadquery(
            _poly_prism(_mirror_y(stripe_pts, -1.0), grip_uz1, grip_uz1 + 0.0006),
            "upper_stripe",
        ),
        material=red_accent,
        name="grip_stripe",
    )

    # ----- Revolute pivot at the rivet, axis perpendicular to the tool plane.
    # q=0 is the splayed rest pose; positive q closes the jaws while
    # the handles scissor toward each other.
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, -HALF_OPEN)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=4.0, lower=0.0, upper=CLOSE_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower = object_model.get_part("lower_half")
    upper = object_model.get_part("upper_half")
    pivot = object_model.get_articulation("pivot")

    # --- Pivot stack: bosses coaxial, upper sits above lower.
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
        min_gap=-0.00005,
        max_gap=0.0008,
        name="upper boss sits on lower boss",
    )

    # --- Pivot caps present on both sides of the tool (combined pivot assembly).
    pivot_asm_aabb = ctx.part_element_world_aabb(lower, elem="pivot_assembly")
    boss_aabb_l = ctx.part_element_world_aabb(lower, elem="pivot_boss")
    boss_aabb_u = ctx.part_element_world_aabb(upper, elem="pivot_boss")

    ctx.check(
        "pivot assembly spans below lower boss and above upper boss",
        pivot_asm_aabb is not None
        and boss_aabb_l is not None
        and boss_aabb_u is not None
        and pivot_asm_aabb[0][2] < boss_aabb_l[0][2] - 0.0005
        and pivot_asm_aabb[1][2] > boss_aabb_u[1][2] + 0.0005,
        details=f"pivot_asm={pivot_asm_aabb}, lower_boss={boss_aabb_l}, upper_boss={boss_aabb_u}",
    )

    # --- Pivot assembly has visible caps (wider than the rivet shank).
    if pivot_asm_aabb is not None:
        asm_dx = pivot_asm_aabb[1][0] - pivot_asm_aabb[0][0]
        asm_dy = pivot_asm_aabb[1][1] - pivot_asm_aabb[0][1]
        ctx.check(
            "pivot assembly has cap discs (wider than shank)",
            asm_dx > 2.0 * CAP_R * 0.8,
            details=f"asm_dx={asm_dx:.4f}, expected>{2.0 * CAP_R * 0.8:.4f}",
        )
        ctx.check(
            "pivot caps are roughly circular in XY",
            abs(asm_dx - asm_dy) < 0.004,
            details=f"dx={asm_dx:.4f}, dy={asm_dy:.4f}",
        )

    # --- Slip-joint slot visible on the lower half pivot boss.
    # The slot makes the lower boss smaller in X than a full circle.
    lower_boss_aabb = ctx.part_element_world_aabb(lower, elem="pivot_boss")
    if lower_boss_aabb is not None:
        boss_dx = lower_boss_aabb[1][0] - lower_boss_aabb[0][0]
        boss_dy = lower_boss_aabb[1][1] - lower_boss_aabb[0][1]
        ctx.check(
            "lower boss has slip-joint slot (elongated in X)",
            boss_dx > boss_dy * 1.1,
            details=f"boss_dx={boss_dx:.4f}, boss_dy={boss_dy:.4f}",
        )

    # --- Grip sleeves are color-separated from the steel handle.
    for part_obj in (lower, upper):
        sleeve = ctx.part_element_world_aabb(part_obj, elem="grip_sleeve")
        core = ctx.part_element_world_aabb(part_obj, elem="handle_core")
        ctx.check(
            f"{part_obj.name} grip sleeve wraps around handle core",
            sleeve is not None
            and core is not None
            and sleeve[0][2] <= core[0][2] + 0.0005
            and sleeve[1][2] >= core[1][2] - 0.0005,
            details=f"sleeve={sleeve}, core={core}",
        )

    # --- Rest pose: jaws open, handles splayed apart.
    ctx.expect_gap(
        lower,
        upper,
        axis="y",
        positive_elem="jaw",
        negative_elem="jaw",
        min_gap=0.002,
        name="jaws open at rest",
    )
    ctx.expect_gap(
        upper,
        lower,
        axis="y",
        positive_elem="grip_sleeve",
        negative_elem="grip_sleeve",
        min_gap=0.008,
        name="handles splay apart at rest",
    )

    # --- Overall proportions: ~0.15 m long, ~0.05-0.07 m wide splayed.
    la = ctx.part_world_aabb(lower)
    ua = ctx.part_world_aabb(upper)
    if la is not None and ua is not None:
        length = max(la[1][0], ua[1][0]) - min(la[0][0], ua[0][0])
        width = max(la[1][1], ua[1][1]) - min(la[0][1], ua[0][1])
        height = max(la[1][2], ua[1][2]) - min(la[0][2], ua[0][2])
        ctx.check(
            "overall length ~0.15 m",
            0.12 <= length <= 0.18,
            details=f"length={length:.4f}",
        )
        ctx.check(
            "splayed handle width ~0.06 m",
            0.04 <= width <= 0.09,
            details=f"width={width:.4f}",
        )
        ctx.check(
            "flat tool thickness ~0.012 m",
            0.009 <= height <= 0.020,
            details=f"height={height:.4f}",
        )

    # --- Articulation: positive q closes jaws, handles scissor in opposition.
    limits = pivot.motion_limits
    ctx.check(
        "pivot has non-trivial travel range",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower) < 1e-9
        and 0.40 <= limits.upper <= 0.65,
        details=f"limits={limits}",
    )

    open_jaw = ctx.part_element_world_aabb(upper, elem="jaw")
    open_grip = ctx.part_element_world_aabb(upper, elem="grip_sleeve")
    with ctx.pose({pivot: CLOSE_TRAVEL}):
        ctx.expect_contact(
            lower,
            upper,
            elem_a="jaw",
            elem_b="jaw",
            contact_tol=0.002,
            name="jaw faces approach when fully closed",
        )
        closed_jaw = ctx.part_element_world_aabb(upper, elem="jaw")
        closed_grip = ctx.part_element_world_aabb(upper, elem="grip_sleeve")

    ctx.check(
        "closing swings upper jaw toward lower jaw",
        open_jaw is not None
        and closed_jaw is not None
        and closed_jaw[1][1] > open_jaw[1][1] + 0.003,
        details=f"open={open_jaw}, closed={closed_jaw}",
    )
    ctx.check(
        "handles scissor opposite to the jaws",
        open_grip is not None
        and closed_grip is not None
        and closed_grip[0][1] < open_grip[0][1] - 0.003,
        details=f"open={open_grip}, closed={closed_grip}",
    )

    return ctx.report()


object_model = build_object_model()
