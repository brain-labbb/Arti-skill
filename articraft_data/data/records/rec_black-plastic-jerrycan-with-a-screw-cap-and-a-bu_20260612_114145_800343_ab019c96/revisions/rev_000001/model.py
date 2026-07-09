from __future__ import annotations

"""Black plastic jerrycan rebuilt from the reference image.

The model keeps the original record identity but replaces the previous geometry
with a fresh construction: a tall matte-black rectangular plastic can, sloped
top shoulder, integrated front-to-back D-grip handle opening, raised screw-neck,
and an open through-mouth leading into a real hollow interior.
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
# the +X handle half, matching the reference's stepped, peaked top rather than a
# single steep wedge.
DECK_LOW_Z = WALL_TOP + 0.046  # 0.252 — lowest deck point, at the -X (cap) edge
TOP_Z = WALL_TOP + 0.098  # 0.304 — flat plateau over the +X (handle) half
DECK_HIGH_Z = TOP_Z  # retained name; the plateau is the high deck
X_KNEE = -0.012  # the cap-side slope meets the flat plateau here

SHELL_T = 0.012
INNER_BOTTOM_Z = BASE_H + 0.010
INNER_DECK_CLEARANCE = 0.020

NECK_X = -0.058
NECK_Y = 0.030
NECK_R = 0.024
MOUTH_R = 0.016
NECK_H = 0.020

CAP_R = 0.028
CAP_H = 0.030
CAP_CLOSED_BOTTOM_Z = -0.003
CAP_OPEN_LIFT = 0.040

# Integrated carry handle: a front-to-back through-slot in the high plateau,
# leaving a ~0.020-thick rounded grip bar across the top.
GRIP_CX = 0.048
GRIP_HALF_X = 0.040
GRIP_CZ = TOP_Z - 0.042  # 0.262 — slot center
GRIP_HALF_Z = 0.021
GRIP_FILLET = 0.019


def _deck_z(x: float) -> float:
    """Deck height: linear rise from the -X cap edge up to X_KNEE, flat after."""
    if x >= X_KNEE:
        return TOP_Z
    t = (x + W / 2.0) / (X_KNEE + W / 2.0)
    return DECK_LOW_Z + (TOP_Z - DECK_LOW_Z) * t


NECK_DECK_Z = _deck_z(NECK_X)
NECK_BASE_Z = NECK_DECK_Z - 0.007
NECK_TOP_Z = NECK_DECK_Z + NECK_H

# Raised, smoothly-molded spout boss the neck rises from.
BOSS_TOP_Z = NECK_DECK_Z + 0.009
BOSS_R_BASE = 0.043
BOSS_R_TOP = 0.030


def _left_wedge_cutter() -> cq.Workplane:
    """Half-space above the cap-side slope plane; trims the -X shoulder wedge
    while leaving the +X plateau (where the plane rises above TOP_Z) untouched."""
    pitch = math.atan2(TOP_Z - DECK_LOW_Z, X_KNEE + W / 2.0)
    cutter = cq.Workplane("XY").box(0.9, 0.9, 0.9, centered=(True, True, False))
    cutter = cutter.rotate((0, 0, 0), (0, 1, 0), -math.degrees(pitch))
    # Bottom face of the rotated box lies on the slope through the -X top corner.
    return cutter.translate((-W / 2.0, 0.0, DECK_LOW_Z))


def _inner_cavity() -> cq.Workplane:
    """Air volume removed from the plastic shell (flat-topped, below the deck)."""
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
    """Smooth molded mound the spout rises from, blended into the shoulder via a
    soft 3-section loft (wide embedded skirt -> raised collar)."""
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
    # Main rectangular body up to the shoulder base. A small foot rim sits just
    # proud of the wall for a stable, molded-base look.
    foot = cq.Workplane("XY").box(W + 0.004, D + 0.004, BASE_H, centered=(True, True, False))
    walls = (
        cq.Workplane("XY")
        .workplane(offset=BASE_H)
        .box(W, D, WALL_H, centered=(True, True, False))
    )
    body = foot.union(walls)

    # Recessed molded groove wrapping the lower body (visible in the reference).
    body = body.cut(_base_groove_cutter())

    # Shoulder: a full-height slab to the plateau top, then the -X cap-side
    # wedge is trimmed away so a flat high plateau remains over the handle half.
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=WALL_TOP)
        .box(W, D, TOP_Z - WALL_TOP, centered=(True, True, False))
    )
    shoulder = shoulder.cut(_left_wedge_cutter())
    body = body.union(shoulder)

    # Softer molded plastic corners before cutting functional openings.
    body = body.edges("|Z").fillet(0.008)
    # Round the plateau's top perimeter for the molded-plastic look.
    try:
        body = body.faces(">Z").edges().fillet(0.010)
    except Exception:
        pass

    # Reference handle: a front-to-back through-slot in the high plateau, leaving
    # a rounded grip bar across the top.
    body = body.cut(_rounded_slot_cutter())

    # Raised molded spout boss the shoulder sweeps up into.
    body = body.union(_neck_boss())
    # Soften where the boss meets the deck for the molded-plastic blend.
    try:
        body = body.edges(
            cq.NearestToPointSelector((NECK_X, NECK_Y + BOSS_R_BASE, NECK_DECK_Z))
        ).fillet(0.006)
    except Exception:
        pass

    # Short screw neck rising from the boss crown.
    neck = (
        cq.Workplane("XY")
        .workplane(offset=BOSS_TOP_Z - 0.004)
        .center(NECK_X, NECK_Y)
        .circle(NECK_R)
        .extrude(NECK_TOP_Z - (BOSS_TOP_Z - 0.004))
    )
    body = body.union(neck)

    # Collar lip just beneath the cap (the cap overhangs this ring).
    rim = (
        cq.Workplane("XY")
        .workplane(offset=NECK_TOP_Z - 0.004)
        .center(NECK_X, NECK_Y)
        .circle(NECK_R + 0.003)
        .extrude(0.004)
    )
    body = body.union(rim)

    # Remove the internal air volume, then cut the pour channel through to it.
    body = body.cut(_inner_cavity())
    mouth = (
        cq.Workplane("XY")
        .workplane(offset=INNER_BOTTOM_Z)
        .center(NECK_X, NECK_Y)
        .circle(MOUTH_R)
        .extrude((NECK_TOP_Z + 0.006) - INNER_BOTTOM_Z)
    )
    body = body.cut(mouth)

    return body


def _cap_mesh():
    """Finely-knurled screw cap with a smooth chamfered top rim, seated on the
    collar in the reference-like rest pose."""
    cap = CylinderGeometry(CAP_R, CAP_H, radial_segments=64)
    cap.translate(0.0, 0.0, CAP_CLOSED_BOTTOM_Z + CAP_H / 2.0)

    # Fine vertical knurling over the lower ~3/4, leaving a smooth top rim band.
    rib_h = CAP_H * 0.74
    rib_cz = CAP_CLOSED_BOTTOM_Z + CAP_H * 0.04 + rib_h / 2.0
    n_ribs = 36
    for i in range(n_ribs):
        angle = 2.0 * math.pi * i / n_ribs
        rib = CylinderGeometry(0.0012, rib_h, radial_segments=6)
        rib.translate(
            CAP_R * math.cos(angle),
            CAP_R * math.sin(angle),
            rib_cz,
        )
        cap.merge(rib)

    # Slightly recessed flat top (rim sits a touch proud, like a molded cap).
    top_disc = CylinderGeometry(CAP_R * 0.86, 0.003, radial_segments=48)
    top_disc.translate(0.0, 0.0, CAP_CLOSED_BOTTOM_Z + CAP_H - 0.0015)
    cap.merge(top_disc)
    return mesh_from_geometry(cap, "cap_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="black_plastic_jerrycan")

    hdpe_black = model.material("matte_black_hdpe", rgba=(0.10, 0.10, 0.105, 1.0))
    cap_black = model.material("ribbed_black_cap", rgba=(0.055, 0.055, 0.060, 1.0))

    body = model.part("body")
    body.visual(mesh_from_cadquery(_body_solid(), "jug_body"), material=hdpe_black, name="jug_body")
    body.inertial = Inertial.from_geometry(
        Box((W, D, TOP_Z)),
        mass=0.90,
        origin=Origin(xyz=(0.0, 0.0, TOP_Z / 2.0)),
    )

    carrier = model.part("cap_carrier")
    carrier.inertial = Inertial.from_geometry(Box((0.01, 0.01, 0.01)), mass=1e-4)

    cap = model.part("cap")
    cap.visual(_cap_mesh(), material=cap_black, name="cap_shell")
    # Small off-axis marker (same color as the cap) used only to verify rotation.
    cap.visual(
        Box((0.004, 0.004, 0.004)),
        origin=Origin(xyz=(CAP_R - 0.003, 0.0, CAP_CLOSED_BOTTOM_Z + CAP_H / 2.0)),
        material=cap_black,
        name="cap_marker",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(CAP_R, CAP_H),
        mass=0.025,
        origin=Origin(xyz=(0.0, 0.0, CAP_CLOSED_BOTTOM_Z + CAP_H / 2.0)),
    )

    model.articulation(
        "cap_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(NECK_X, NECK_Y, NECK_TOP_Z)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )
    model.articulation(
        "cap_slide",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(lower=0.0, upper=CAP_OPEN_LIFT, effort=1.0, velocity=1.0),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    carrier = object_model.get_part("cap_carrier")
    cap = object_model.get_part("cap")
    rotate = object_model.get_articulation("cap_rotate")
    slide = object_model.get_articulation("cap_slide")

    ctx.allow_overlap(
        body,
        cap,
        elem_a="jug_body",
        elem_b="cap_shell",
        reason="The cap aligns with the threaded neck and may overlap only while closing.",
    )
    ctx.allow_isolated_part(
        cap,
        reason="The screw cap can separate from the neck when the slide joint is opened.",
    )

    body_ext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "jerrycan is tall rectangular plastic can",
        body_ext[2] > body_ext[0] + 0.05 and body_ext[0] > body_ext[1] + 0.025,
        details=f"body_ext={body_ext}",
    )

    body_aabb = ctx.part_element_world_aabb(body, elem="jug_body")
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

    cap_aabb = ctx.part_element_world_aabb(cap, elem="cap_shell")
    ctx.check(
        "default cap pose is seated on the threaded neck like the reference",
        cap_aabb[0][2] <= NECK_TOP_Z + 0.002 and cap_aabb[1][2] > NECK_TOP_Z + 0.015,
        details=f"cap_bottom={cap_aabb[0][2]}, neck_top={NECK_TOP_Z}",
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

    part_names = {p.name for p in object_model.parts}
    ctx.check(
        "only body, cap carrier, and screw cap are articulated parts",
        part_names == {"body", "cap_carrier", "cap"},
        details=f"parts={sorted(part_names)}",
    )

    marker0 = ctx.part_element_world_aabb(cap, elem="cap_marker")
    marker0_xy = ((marker0[0][0] + marker0[1][0]) / 2.0, (marker0[0][1] + marker0[1][1]) / 2.0)
    with ctx.pose({rotate: math.pi / 2.0}):
        marker1 = ctx.part_element_world_aabb(cap, elem="cap_marker")
        marker1_xy = (
            (marker1[0][0] + marker1[1][0]) / 2.0,
            (marker1[0][1] + marker1[1][1]) / 2.0,
        )
    moved = math.hypot(marker1_xy[0] - marker0_xy[0], marker1_xy[1] - marker0_xy[1])
    ctx.check(
        "cap rotates around the neck axis",
        moved > 0.010,
        details=f"marker_moved={moved}, before={marker0_xy}, after={marker1_xy}",
    )

    z0 = ctx.part_world_position(cap)[2]
    with ctx.pose({slide: CAP_OPEN_LIFT}):
        z1 = ctx.part_world_position(cap)[2]
    ctx.check(
        "cap can lift along the screw axis to reveal the through-mouth",
        z1 > z0 + 0.030,
        details=f"z0={z0}, z1={z1}",
    )

    carrier_pos = ctx.part_world_position(carrier)
    ctx.check(
        "cap carrier remains at the threaded neck",
        carrier_pos is not None and abs(carrier_pos[2] - NECK_TOP_Z) < 1e-6,
        details=f"carrier_pos={carrier_pos}",
    )

    return ctx.report()


object_model = build_object_model()
