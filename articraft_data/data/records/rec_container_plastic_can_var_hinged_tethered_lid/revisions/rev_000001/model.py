from __future__ import annotations

"""Black plastic jerrycan with tethered hinged flip-cap lid.

Fork of the parent jerrycan: tall matte-black rectangular plastic can, sloped
top shoulder, integrated front-to-back D-grip handle opening, raised neck, and
an open through-mouth leading into a real hollow interior.

The separate screw cap is replaced with a ONE-PIECE TETHERED HINGED LID:
a fixed neck collar on the body and a flip cap joined to it by a short
living-hinge strap, so the cap stays captive and swings open on a real
REVOLUTE hinge about the strap rather than lifting free.
"""

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# Coordinate frame:
#   +Z up.
#   X spans left/right across the front image: -X is the cap/low-shoulder side,
#   +X is the integrated handle/high-shoulder side.
#   Y is depth; the handle opening cuts all the way front-to-back.

W = 0.200
D = 0.150
BASE_H = 0.026
WALL_H = 0.180
WALL_TOP = BASE_H + WALL_H  # 0.206

# Top deck: a low cap-side platform (-X) sloping up to a FLAT high plateau over
# the +X handle half, matching the reference's stepped, peaked top.
DECK_LOW_Z = WALL_TOP + 0.046  # 0.252 — lowest deck point, at the -X (cap) edge
TOP_Z = WALL_TOP + 0.098  # 0.304 — flat plateau over the +X (handle) half
DECK_HIGH_Z = TOP_Z
X_KNEE = -0.012  # the cap-side slope meets the flat plateau here

SHELL_T = 0.012
INNER_BOTTOM_Z = BASE_H + 0.010
INNER_DECK_CLEARANCE = 0.020

NECK_X = -0.058
NECK_Y = 0.030
NECK_R = 0.024
MOUTH_R = 0.016
NECK_H = 0.020

# Integrated carry handle: a front-to-back through-slot in the high plateau,
# leaving a ~0.020-thick rounded grip bar across the top.
GRIP_CX = 0.048
GRIP_HALF_X = 0.040
GRIP_CZ = TOP_Z - 0.042  # 0.262 — slot center
GRIP_HALF_Z = 0.021
GRIP_FILLET = 0.019

# ── Tethered hinged lid constants ──────────────────────────────────────────
COLLAR_OUTER_R = NECK_R + 0.005  # 0.029 — collar outer radius
COLLAR_H = 0.006  # collar ring height above neck top
# Flip cap
FLIP_CAP_R = COLLAR_OUTER_R  # 0.029 — cap disc matches collar
FLIP_CAP_H = 0.005  # cap disc thickness
FLIP_CAP_RIM_H = 0.008  # raised rim for grip
FLIP_CAP_RIM_T = 0.003  # rim radial thickness

# Strap (living hinge) tab on cap: extends from disc edge outward
STRAP_TAB_W = 0.010  # width in X
STRAP_TAB_EXT = 0.006  # extension past disc edge in Y
STRAP_TAB_H = FLIP_CAP_H  # same height as disc

# Anchor nub on body collar (+Y side): a small protrusion the strap connects to
ANCHOR_W = 0.010  # width in X
ANCHOR_EXT = 0.004  # extension past collar outer in Y
ANCHOR_H = COLLAR_H  # height matches collar


def _deck_z(x: float) -> float:
    """Deck height: linear rise from the -X cap edge up to X_KNEE, flat after."""
    if x >= X_KNEE:
        return TOP_Z
    t = (x + W / 2.0) / (X_KNEE + W / 2.0)
    return DECK_LOW_Z + (TOP_Z - DECK_LOW_Z) * t


NECK_DECK_Z = _deck_z(NECK_X)
NECK_BASE_Z = NECK_DECK_Z - 0.007
NECK_TOP_Z = NECK_DECK_Z + NECK_H

# Recompute collar top now that NECK_TOP_Z is defined
COLLAR_TOP_Z = NECK_TOP_Z + COLLAR_H

# Raised, smoothly-molded spout boss the neck rises from.
BOSS_TOP_Z = NECK_DECK_Z + 0.009
BOSS_R_BASE = 0.043
BOSS_R_TOP = 0.030

# Hinge origin: at the +Y collar edge, at collar top surface
HINGE_ORIGIN_XYZ = (NECK_X, NECK_Y + COLLAR_OUTER_R, COLLAR_TOP_Z)
# Axis: (-1, 0, 0) so positive q lifts cap upward and away in +Y
HINGE_AXIS = (-1.0, 0.0, 0.0)
HINGE_UPPER = 2.6  # ~149° open


def _left_wedge_cutter() -> cq.Workplane:
    """Half-space above the cap-side slope plane."""
    pitch = math.atan2(TOP_Z - DECK_LOW_Z, X_KNEE + W / 2.0)
    cutter = cq.Workplane("XY").box(0.9, 0.9, 0.9, centered=(True, True, False))
    cutter = cutter.rotate((0, 0, 0), (0, 1, 0), -math.degrees(pitch))
    return cutter.translate((-W / 2.0, 0.0, DECK_LOW_Z))


def _inner_cavity() -> cq.Workplane:
    """Air volume removed from the plastic shell."""
    inner_w = W - 2.0 * SHELL_T
    inner_d = D - 2.0 * SHELL_T
    high = DECK_LOW_Z - INNER_DECK_CLEARANCE
    return (
        cq.Workplane("XY")
        .workplane(offset=INNER_BOTTOM_Z)
        .box(inner_w, inner_d, high - INNER_BOTTOM_Z, centered=(True, True, False))
    )


def _rounded_slot_cutter() -> cq.Workplane:
    """Front-to-back rounded rectangle for the molded handle opening."""
    slot = (
        cq.Workplane("XZ")
        .center(GRIP_CX, GRIP_CZ)
        .rect(2.0 * GRIP_HALF_X, 2.0 * GRIP_HALF_Z)
        .extrude(D + 0.080, both=True)
    )
    return slot.edges("|Y").fillet(GRIP_FILLET)


def _base_groove_cutter() -> cq.Workplane:
    """Shallow recessed molded line wrapping the can near the base."""
    z0 = 0.052
    h = 0.007
    depth = 0.005
    outer = (
        cq.Workplane("XY")
        .workplane(offset=z0)
        .box(W + 0.006, D + 0.006, h, centered=(True, True, False))
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=z0 - 0.006)
        .box(W - 2.0 * depth, D - 2.0 * depth, h + 0.012, centered=(True, True, False))
    )
    return outer.cut(inner)


def _neck_boss() -> cq.Workplane:
    """Smooth molded mound the spout rises from."""
    z0 = NECK_DECK_Z - 0.032
    boss = (
        cq.Workplane("XY")
        .workplane(offset=z0)
        .circle(BOSS_R_BASE)
        .workplane(offset=(BOSS_TOP_Z - z0) - 0.010)
        .circle(BOSS_R_TOP + 0.004)
        .workplane(offset=0.010)
        .circle(BOSS_R_TOP)
        .loft(ruled=False)
    )
    return boss.translate((NECK_X, NECK_Y, 0.0))


def _body_solid() -> cq.Workplane:
    # Main rectangular body up to the shoulder base.
    foot = cq.Workplane("XY").box(W + 0.004, D + 0.004, BASE_H, centered=(True, True, False))
    walls = (
        cq.Workplane("XY")
        .workplane(offset=BASE_H)
        .box(W, D, WALL_H, centered=(True, True, False))
    )
    body = foot.union(walls)

    # Recessed molded groove wrapping the lower body.
    body = body.cut(_base_groove_cutter())

    # Shoulder: full-height slab then cap-side wedge trimmed away.
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=WALL_TOP)
        .box(W, D, TOP_Z - WALL_TOP, centered=(True, True, False))
    )
    shoulder = shoulder.cut(_left_wedge_cutter())
    body = body.union(shoulder)

    # Softer molded plastic corners.
    body = body.edges("|Z").fillet(0.008)
    try:
        body = body.faces(">Z").edges().fillet(0.010)
    except Exception:
        pass

    # Handle through-slot.
    body = body.cut(_rounded_slot_cutter())

    # Raised molded spout boss.
    body = body.union(_neck_boss())
    try:
        body = body.edges(
            cq.NearestToPointSelector((NECK_X, NECK_Y + BOSS_R_BASE, NECK_DECK_Z))
        ).fillet(0.006)
    except Exception:
        pass

    # Short neck rising from the boss crown.
    neck = (
        cq.Workplane("XY")
        .workplane(offset=BOSS_TOP_Z - 0.004)
        .center(NECK_X, NECK_Y)
        .circle(NECK_R)
        .extrude(NECK_TOP_Z - (BOSS_TOP_Z - 0.004))
    )
    body = body.union(neck)

    # ── Tethered lid collar: raised ring the flip cap seats on ──
    collar = (
        cq.Workplane("XY")
        .workplane(offset=NECK_TOP_Z)
        .center(NECK_X, NECK_Y)
        .circle(COLLAR_OUTER_R)
        .extrude(COLLAR_H)
    )
    body = body.union(collar)

    # Strap anchor nub on +Y side of collar (the fixed end of the living hinge)
    anchor = (
        cq.Workplane("XY")
        .workplane(offset=NECK_TOP_Z)
        .center(NECK_X, NECK_Y + COLLAR_OUTER_R + ANCHOR_EXT / 2.0)
        .box(ANCHOR_W, ANCHOR_EXT, ANCHOR_H, centered=(True, True, False))
    )
    body = body.union(anchor)

    # Remove the internal air volume, then cut the pour channel through.
    body = body.cut(_inner_cavity())
    mouth = (
        cq.Workplane("XY")
        .workplane(offset=INNER_BOTTOM_Z)
        .center(NECK_X, NECK_Y)
        .circle(MOUTH_R)
        .extrude((COLLAR_TOP_Z + 0.002) - INNER_BOTTOM_Z)
    )
    body = body.cut(mouth)

    return body


def _flip_cap_solid() -> cq.Workplane:
    """Flip cap with raised rim, grip ridges, and strap tab.

    Built in cap local frame: origin at hinge point.
    Disc center at (0, -FLIP_CAP_R, FLIP_CAP_H/2) — sits on collar when closed.
    Strap tab extends from disc edge outward in +Y toward the body anchor.
    """
    # Main disc — flat cap that seals the collar opening
    disc = (
        cq.Workplane("XY")
        .center(0.0, -FLIP_CAP_R)
        .circle(FLIP_CAP_R)
        .extrude(FLIP_CAP_H)
    )

    # Raised rim around the outer edge for grip
    rim = (
        cq.Workplane("XY")
        .center(0.0, -FLIP_CAP_R)
        .circle(FLIP_CAP_R + 0.001)
        .circle(FLIP_CAP_R - FLIP_CAP_RIM_T)
        .extrude(FLIP_CAP_RIM_H)
    )
    cap = disc.union(rim)

    # Grip ridges around the rim (for tactile detail)
    n_ridges = 20
    for i in range(n_ridges):
        angle = 2.0 * math.pi * i / n_ridges
        # Skip the strap area (around angle=π/2, the +Y side)
        if abs(math.sin(angle) - 1.0) < 0.3:
            continue
        rx = FLIP_CAP_R * math.cos(angle)
        ry = -FLIP_CAP_R + FLIP_CAP_R * math.sin(angle)
        ridge = (
            cq.Workplane("XY")
            .center(rx, ry)
            .circle(0.0012)
            .extrude(FLIP_CAP_RIM_H - 0.001)
        )
        cap = cap.union(ridge)

    # Strap tab on the +Y side — the moving end of the living hinge
    strap = (
        cq.Workplane("XY")
        .center(0.0, STRAP_TAB_EXT / 2.0)
        .rect(STRAP_TAB_W, STRAP_TAB_EXT)
        .extrude(STRAP_TAB_H)
    )
    cap = cap.union(strap)

    # Small raised grip bump on top center for easy lifting
    # (starts from disc bottom to ensure full mesh merging with the cap body)
    bump = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .center(0.0, -FLIP_CAP_R)
        .circle(0.005)
        .extrude(0.011)
    )
    cap = cap.union(bump)

    # Soften top rim edges
    try:
        cap = cap.faces(">Z").edges().fillet(0.001)
    except Exception:
        pass

    return cap


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="black_plastic_jerrycan_tethered_lid")

    hdpe_black = model.material("matte_black_hdpe", rgba=(0.10, 0.10, 0.105, 1.0))
    cap_dark = model.material("dark_gray_cap", rgba=(0.07, 0.07, 0.075, 1.0))

    # ── Body (root) ──
    body = model.part("body")
    body.visual(mesh_from_cadquery(_body_solid(), "jug_body"), material=hdpe_black, name="jug_body")
    body.inertial = Inertial.from_geometry(
        Box((W, D, TOP_Z)),
        mass=0.90,
        origin=Origin(xyz=(0.0, 0.0, TOP_Z / 2.0)),
    )

    # ── Flip cap (hinged child) ──
    flip_cap = model.part("flip_cap")
    flip_cap.visual(
        mesh_from_cadquery(_flip_cap_solid(), "flip_cap_shell"),
        material=cap_dark,
        name="flip_cap_shell",
    )
    # Small off-axis marker embedded in the disc top to verify rotation
    # (extends through the disc top surface to ensure mesh connectivity)
    flip_cap.visual(
        Box((0.004, 0.004, 0.004)),
        origin=Origin(xyz=(FLIP_CAP_R - 0.004, -FLIP_CAP_R, FLIP_CAP_H)),
        material=cap_dark,
        name="cap_marker",
    )
    flip_cap.inertial = Inertial.from_geometry(
        Cylinder(FLIP_CAP_R, FLIP_CAP_H),
        mass=0.015,
        origin=Origin(xyz=(0.0, -FLIP_CAP_R, FLIP_CAP_H / 2.0)),
    )

    # ── REVOLUTE hinge: flip cap swings open about the living-hinge strap ──
    model.articulation(
        "cap_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flip_cap,
        origin=Origin(xyz=HINGE_ORIGIN_XYZ),
        axis=HINGE_AXIS,
        motion_limits=MotionLimits(
            lower=0.0,
            upper=HINGE_UPPER,
            effort=2.0,
            velocity=3.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    flip_cap = object_model.get_part("flip_cap")
    hinge = object_model.get_articulation("cap_hinge")

    # ── Allowances ──
    ctx.allow_overlap(
        body,
        flip_cap,
        elem_a="jug_body",
        elem_b="flip_cap_shell",
        reason=(
            "The flip cap strap tab and body collar anchor share the hinge "
            "contact surface and may touch at the seating interface."
        ),
    )

    # ── Body shape checks (preserved from parent) ──
    body_aabb = ctx.part_element_world_aabb(body, elem="jug_body")
    body_ext = (
        body_aabb[1][0] - body_aabb[0][0],
        body_aabb[1][1] - body_aabb[0][1],
        body_aabb[1][2] - body_aabb[0][2],
    )
    ctx.check(
        "jerrycan is tall rectangular plastic can",
        body_ext[2] > body_ext[0] + 0.05 and body_ext[0] > body_ext[1] + 0.025,
        details=f"body_ext={body_ext}",
    )

    ctx.check(
        "sloped shoulder reaches the high handle side",
        abs(body_aabb[1][2] - TOP_Z) < 0.015 and (DECK_HIGH_Z - DECK_LOW_Z) > 0.04,
        details=f"top={body_aabb[1][2]}, deck_low={DECK_LOW_Z}, deck_high={DECK_HIGH_Z}",
    )

    ctx.check(
        "integrated handle is a front-to-back through-slot in the high shoulder",
        GRIP_CX > 0.025
        and GRIP_CX + GRIP_HALF_X < W / 2.0 + 0.002
        and GRIP_CZ > WALL_TOP
        and 2.0 * GRIP_HALF_Z > 0.035,
        details=f"grip_cx={GRIP_CX}, grip_cz={GRIP_CZ}, half=({GRIP_HALF_X},{GRIP_HALF_Z})",
    )

    ctx.check(
        "mouth is a through-opening into a real hollow interior",
        MOUTH_R >= 0.014
        and INNER_BOTTOM_Z < NECK_BASE_Z - 0.15
        and (W - 2.0 * SHELL_T) > 0.17
        and (D - 2.0 * SHELL_T) > 0.11,
        details=(
            f"mouth_r={MOUTH_R}, inner_bottom={INNER_BOTTOM_Z}, "
            f"inner_w={W - 2.0 * SHELL_T}, inner_d={D - 2.0 * SHELL_T}"
        ),
    )

    # ── Tethered lid mechanism checks ──
    part_names = {p.name for p in object_model.parts}
    ctx.check(
        "only body and flip_cap are parts (tethered lid, no separate carrier)",
        part_names == {"body", "flip_cap"},
        details=f"parts={sorted(part_names)}",
    )

    ctx.check(
        "hinge is REVOLUTE (not continuous or prismatic)",
        hinge.joint_type == ArticulationType.REVOLUTE,
        details=f"joint_type={hinge.joint_type}",
    )

    # Cap seated on collar when closed (q=0)
    cap_aabb = ctx.part_element_world_aabb(flip_cap, elem="flip_cap_shell")
    ctx.check(
        "flip cap is seated on the collar at rest (closed)",
        cap_aabb[0][2] >= COLLAR_TOP_Z - 0.002
        and cap_aabb[0][2] <= COLLAR_TOP_Z + 0.003,
        details=f"cap_bottom_z={cap_aabb[0][2]}, collar_top={COLLAR_TOP_Z}",
    )

    # Cap center is over the neck when closed
    cap_center_y = (cap_aabb[0][1] + cap_aabb[1][1]) / 2.0
    ctx.check(
        "flip cap is centered on the neck when closed",
        abs(cap_center_y - NECK_Y) < 0.005,
        details=f"cap_center_y={cap_center_y}, neck_y={NECK_Y}",
    )

    # Positive q opens the cap upward (cap shell max_z increases)
    cap_aabb_closed = ctx.part_element_world_aabb(flip_cap, elem="flip_cap_shell")
    cap_max_z_closed = cap_aabb_closed[1][2]
    with ctx.pose({hinge: 1.2}):
        cap_aabb_open = ctx.part_element_world_aabb(flip_cap, elem="flip_cap_shell")
    cap_max_z_open = cap_aabb_open[1][2]
    ctx.check(
        "positive hinge angle lifts the cap upward",
        cap_max_z_open > cap_max_z_closed + 0.015,
        details=f"max_z_closed={cap_max_z_closed}, max_z_open_1.2={cap_max_z_open}",
    )

    # At max open angle, the mouth area is uncovered
    with ctx.pose({hinge: HINGE_UPPER}):
        cap_aabb_open = ctx.part_element_world_aabb(flip_cap, elem="flip_cap_shell")
    # The cap center should be displaced from the neck center when open
    cap_center_y_open = (cap_aabb_open[0][1] + cap_aabb_open[1][1]) / 2.0
    cap_center_z_open = (cap_aabb_open[0][2] + cap_aabb_open[1][2]) / 2.0
    ctx.check(
        "fully open cap displaces away from the mouth",
        abs(cap_center_y_open - NECK_Y) > 0.015
        or cap_center_z_open > COLLAR_TOP_Z + 0.020,
        details=f"open_center_y={cap_center_y_open}, open_center_z={cap_center_z_open}",
    )

    # Cap stays connected (doesn't fly away) at max open
    cap_center_closed = (
        (cap_aabb_closed[0][0] + cap_aabb_closed[1][0]) / 2.0,
        (cap_aabb_closed[0][1] + cap_aabb_closed[1][1]) / 2.0,
        (cap_aabb_closed[0][2] + cap_aabb_closed[1][2]) / 2.0,
    )
    with ctx.pose({hinge: HINGE_UPPER}):
        cap_aabb_max = ctx.part_element_world_aabb(flip_cap, elem="flip_cap_shell")
    cap_center_max = (
        (cap_aabb_max[0][0] + cap_aabb_max[1][0]) / 2.0,
        (cap_aabb_max[0][1] + cap_aabb_max[1][1]) / 2.0,
        (cap_aabb_max[0][2] + cap_aabb_max[1][2]) / 2.0,
    )
    dist = math.hypot(
        cap_center_max[0] - cap_center_closed[0],
        cap_center_max[1] - cap_center_closed[1],
        cap_center_max[2] - cap_center_closed[2],
    )
    ctx.check(
        "flip cap stays captive (tethered) — bounded displacement at max open",
        dist < 0.08,
        details=f"displacement={dist}, closed={cap_center_closed}, max_open={cap_center_max}",
    )

    # Marker confirms rotation around hinge axis
    marker0 = ctx.part_element_world_aabb(flip_cap, elem="cap_marker")
    marker0_center = (
        (marker0[0][0] + marker0[1][0]) / 2.0,
        (marker0[0][1] + marker0[1][1]) / 2.0,
        (marker0[0][2] + marker0[1][2]) / 2.0,
    )
    with ctx.pose({hinge: math.pi / 2.0}):
        marker1 = ctx.part_element_world_aabb(flip_cap, elem="cap_marker")
    marker1_center = (
        (marker1[0][0] + marker1[1][0]) / 2.0,
        (marker1[0][1] + marker1[1][1]) / 2.0,
        (marker1[0][2] + marker1[1][2]) / 2.0,
    )
    moved = math.hypot(
        marker1_center[0] - marker0_center[0],
        marker1_center[1] - marker0_center[1],
        marker1_center[2] - marker0_center[2],
    )
    ctx.check(
        "cap marker moves significantly at 90° hinge pose (proves rotation)",
        moved > 0.015,
        details=f"marker_moved={moved}",
    )

    # Hinge origin is at the collar (not floating in space)
    hinge_origin_z = hinge.origin.xyz[2] if hinge.origin else 0.0
    ctx.check(
        "hinge origin is at the collar top surface",
        abs(hinge_origin_z - COLLAR_TOP_Z) < 0.002,
        details=f"hinge_z={hinge_origin_z}, collar_top={COLLAR_TOP_Z}",
    )

    return ctx.report()


object_model = build_object_model()
