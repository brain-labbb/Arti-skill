from __future__ import annotations

# Architect / balanced-arm desk lamp (Anglepoise / Luxo style).
#
# Real object: a weighted round disc base carries a two-link articulated arm.
# Each link is a parallelogram of twin parallel rods braced by a tension spring.
# A conical metal shade with a bulb socket / finial on top hangs from the end of
# the upper arm. The three real pivots (shoulder at the base, elbow between the
# two arms, and the head tilt) are all revolute hinges turning about a horizontal
# Y axis so the lamp folds in the vertical X-Z plane.
#
# Kinematics: every link is authored FLAT in its own local frame, with its
# proximal pivot at the local origin and the rods running along +X. The rest
# (photo-like) pose is baked entirely into each joint's origin.rpy pitch, so the
# chain stacks naturally and q=0 already reproduces a standing, reaching lamp.
# Rotation about +Y by angle t sends local +X toward (cos t, 0, -sin t); a
# NEGATIVE pitch therefore lifts a +X arm upward (+Z).

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
# Real-world dimensions (meters)
# ---------------------------------------------------------------------------

BASE_RADIUS = 0.105
BASE_HEIGHT = 0.024  # weighted disc thickness
NECK_RADIUS = 0.022
NECK_HEIGHT = 0.030  # short raised collar that carries the shoulder pivot
SHOULDER_Z = BASE_HEIGHT + NECK_HEIGHT  # world z of the shoulder pivot

LOWER_LEN = 0.230  # rod span between shoulder and elbow pivots
UPPER_LEN = 0.230  # rod span between elbow and wrist pivots

ROD_RADIUS = 0.0055  # slender steel rod
ROD_GAUGE = 0.032  # transverse centre-to-centre spacing of the twin rods

BRACKET_HALF = 0.020  # half-length (along X) of a knuckle bracket plate
BRACKET_T = 0.007  # bracket plate thickness (Z)
PIVOT_PIN_R = 0.0065

SHADE_TOP_R = 0.030
SHADE_BOT_R = 0.082
SHADE_HEIGHT = 0.092
SHADE_WALL = 0.0020

# Rest-pose joint pitches (radians, about +Y). Negative lifts a +X member up.
LOWER_PITCH = -math.radians(60.0)  # lower arm rises steeply from the base
UPPER_PITCH = math.radians(50.0)  # upper arm bends forward off the elbow
HEAD_PITCH = -math.radians(40.0)  # shade hangs with its mouth down-and-forward


# ---------------------------------------------------------------------------
# CadQuery shape builders. Each is authored in its LOCAL frame with the proximal
# pivot at the origin. Everything in one link is unioned into a SINGLE solid so
# the exported mesh has no disconnected islands.
# ---------------------------------------------------------------------------


def _base_shape() -> cq.Workplane:
    """Weighted round disc base with a raised collar carrying the shoulder."""
    disc = (
        cq.Workplane("XY")
        .circle(BASE_RADIUS)
        .extrude(BASE_HEIGHT)
        .edges(">Z")
        .chamfer(0.006)
    )
    collar = (
        cq.Workplane("XY")
        .workplane(offset=BASE_HEIGHT - 0.002)
        .circle(NECK_RADIUS * 1.6)
        .workplane(offset=NECK_HEIGHT + 0.002)
        .circle(NECK_RADIUS)
        .loft(combine=True)
    )
    # Small clevis ear on top of the collar that the lower arm pivots against.
    ear = (
        cq.Workplane("XY")
        .box(0.018, ROD_GAUGE + 0.024, 0.016)
        .translate((0.0, 0.0, SHOULDER_Z - 0.002))
    )
    return disc.union(collar).union(ear)


def _arm_link(length: float, *, spring: bool) -> cq.Workplane:
    """A parallelogram arm link authored flat along +X with its proximal pivot
    at the local origin. Two parallel rods are tied together by full-width
    knuckle bracket plates at each pivot (which guarantee one connected solid)
    plus a transverse pivot pin. An optional tension spring is slung between the
    pivots, anchored into both bracket plates so it is never a floating island.
    """
    half_gauge = ROD_GAUGE / 2.0
    plate_w_y = ROD_GAUGE + 2.0 * ROD_RADIUS + 0.008
    pin_len = plate_w_y + 0.010

    # Twin rods (YZ workplane offset to +/- Y, extruded along +X from x=0).
    body: cq.Workplane | None = None
    for y in (-half_gauge, half_gauge):
        rod = cq.Workplane("YZ").center(y, 0.0).circle(ROD_RADIUS).extrude(length)
        body = rod if body is None else body.union(rod)
    assert body is not None

    # Knuckle bracket plates at both pivots; each spans the full rod gauge so the
    # two rods are physically fused.
    for x in (0.0, length):
        plate = (
            cq.Workplane("XY")
            .box(2.0 * BRACKET_HALF, plate_w_y, BRACKET_T)
            .translate((x, 0.0, 0.0))
            .edges("|Z")
            .fillet(0.004)
        )
        body = body.union(plate)
        pin = (
            cq.Workplane("XY")
            .transformed(rotate=(90.0, 0.0, 0.0))
            .circle(PIVOT_PIN_R)
            .extrude(pin_len, both=True)
            .translate((x, 0.0, 0.0))
        )
        body = body.union(pin)

    if spring:
        # Coil-look tension spring slung below the rod axis, its ends embedded in
        # the two bracket plates so it reads as braced and stays connected.
        z_off = -0.014
        spring_x0 = 0.010
        spring_x1 = length - 0.010
        spring_len = spring_x1 - spring_x0
        # Spring body (a slim rod proxy) running along +X just below the rods.
        coil = (
            cq.Workplane("YZ")
            .workplane(offset=spring_x0)
            .center(0.0, z_off)
            .circle(0.0040)
            .extrude(spring_len)
        )
        body = body.union(coil)
        # Hook tabs that tie each spring end up into its bracket plate.
        for x in (spring_x0, spring_x1):
            tab = (
                cq.Workplane("XY")
                .box(0.006, 0.006, abs(z_off) + BRACKET_T)
                .translate((x, 0.0, z_off / 2.0))
            )
            body = body.union(tab)

    return body


def _shade_shape() -> cq.Workplane:
    """Conical lamp shade (narrow socket top, wide flared mouth) with a bulb
    socket and finial on top. Authored with the wrist clamp at the local origin;
    the cone opens downward (-Z) so it hangs like a pendant head."""
    # Outer + inner profile revolved about Z to make a thin-walled cone shell.
    # Profile in (radius, z): narrow top near z=0, wide mouth at z = -H.
    outer = [
        (SHADE_TOP_R, 0.0),
        (SHADE_TOP_R, -0.012),
        (SHADE_BOT_R, -SHADE_HEIGHT),
    ]
    inner = [
        (SHADE_BOT_R - SHADE_WALL, -SHADE_HEIGHT),
        (SHADE_TOP_R - SHADE_WALL, -0.012),
        (SHADE_TOP_R - SHADE_WALL, 0.0),
    ]
    pts = outer + inner
    shade = (
        cq.Workplane("XZ")
        .polyline([(r, z) for (r, z) in pts])
        .close()
        .revolve(360.0, (0, 0, 0), (0, 1, 0))
    )

    # Solid top cap closing the narrow opening; fuses everything above to the
    # cone rim so the head exports as one connected solid (no island).
    top_cap = cq.Workplane("XY").workplane(offset=-0.002).circle(SHADE_TOP_R).extrude(0.005)

    # Bulb socket rising from the cap up to the wrist clamp stub.
    socket = cq.Workplane("XY").circle(0.013).extrude(0.026)
    finial = cq.Workplane("XY").workplane(offset=0.030).sphere(0.0080)

    # Clamp stub bridging the wrist pivot to the socket, plus the transverse pin.
    stub = (
        cq.Workplane("XY")
        .box(2.0 * BRACKET_HALF, ROD_GAUGE + 0.010, BRACKET_T)
        .translate((0.0, 0.0, 0.024))
    )
    wrist_pin = (
        cq.Workplane("XY")
        .transformed(rotate=(90.0, 0.0, 0.0))
        .circle(PIVOT_PIN_R)
        .extrude(ROD_GAUGE + 0.022, both=True)
        .translate((0.0, 0.0, 0.024))
    )

    body = shade.union(top_cap).union(socket).union(finial).union(stub).union(wrist_pin)
    # Lift so the wrist clamp stub sits at the local origin (the wrist pivot).
    body = body.translate((0.0, 0.0, -0.024))
    return body


# ---------------------------------------------------------------------------
# Model assembly
# ---------------------------------------------------------------------------


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="architect_desk_lamp")

    model.material("lamp_blue", rgba=(0.31, 0.39, 0.49, 1.0))
    model.material("lamp_blue_dark", rgba=(0.20, 0.26, 0.34, 1.0))
    model.material("steel", rgba=(0.74, 0.76, 0.80, 1.0))
    model.material("brass", rgba=(0.80, 0.67, 0.36, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_base_shape(), "base.obj"), material="lamp_blue", name="base_disc")

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        mesh_from_cadquery(_arm_link(LOWER_LEN, spring=True), "lower_arm.obj"),
        material="lamp_blue_dark",
        name="lower_arm_body",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        mesh_from_cadquery(_arm_link(UPPER_LEN, spring=True), "upper_arm.obj"),
        material="lamp_blue_dark",
        name="upper_arm_body",
    )

    head = model.part("head")
    head.visual(mesh_from_cadquery(_shade_shape(), "shade.obj"), material="lamp_blue", name="shade_body")

    # Shoulder: base collar -> lower arm. Rest pitch lifts the lower arm up.
    model.articulation(
        "base_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z), rpy=(0.0, LOWER_PITCH, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.0, lower=-1.2, upper=1.2),
    )

    # Elbow: lower arm far end -> upper arm. Joint sits at the lower-arm distal
    # pivot (local x = LOWER_LEN). Rest pitch bends the upper arm forward.
    model.articulation(
        "lower_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(LOWER_LEN, 0.0, 0.0), rpy=(0.0, UPPER_PITCH, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.0, lower=-2.4, upper=1.2),
    )

    # Wrist / head tilt: upper arm far end -> head. Joint at the upper-arm distal
    # pivot (local x = UPPER_LEN). Rest pitch swings the shade down to hang.
    model.articulation(
        "upper_arm_to_head",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=head,
        origin=Origin(xyz=(UPPER_LEN, 0.0, 0.0), rpy=(0.0, HEAD_PITCH, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=7.0, velocity=2.0, lower=-1.8, upper=1.8),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    head = object_model.get_part("head")

    shoulder = object_model.get_articulation("base_to_lower_arm")
    elbow = object_model.get_articulation("lower_to_upper_arm")
    wrist = object_model.get_articulation("upper_arm_to_head")

    # --- Joint topology / type / axis checks -----------------------------
    for joint, parent, child in (
        (shoulder, "base", "lower_arm"),
        (elbow, "lower_arm", "upper_arm"),
        (wrist, "upper_arm", "head"),
    ):
        ctx.check(
            f"{joint.name}_is_revolute",
            joint.articulation_type == ArticulationType.REVOLUTE,
            details=f"type={joint.articulation_type}",
        )
        ctx.check(
            f"{joint.name}_axis_is_y",
            tuple(round(a, 6) for a in joint.axis) == (0.0, 1.0, 0.0),
            details=f"axis={joint.axis}",
        )
        ctx.check(
            f"{joint.name}_parent_child",
            joint.parent == parent and joint.child == child,
            details=f"parent={joint.parent}, child={joint.child}",
        )

    # --- Hero geometry present & placed ----------------------------------
    base_aabb = ctx.part_world_aabb(base)
    if base_aabb is not None:
        (bx0, by0, bz0), (bx1, by1, bz1) = base_aabb
        ctx.check(
            "base_is_wide_disc",
            (bx1 - bx0) > 0.18 and (by1 - by0) > 0.18,
            details=f"base footprint={(bx1 - bx0, by1 - by0)}",
        )
        ctx.check(
            "base_sits_on_table",
            abs(bz0) < 0.005,
            details=f"base bottom z={bz0}",
        )

    # Shade reads as a wide downward-opening cone: its mouth (max y-extent) is
    # much wider than its narrow socket top.
    shade_vis = head.get_visual("shade_body")
    shade_aabb = ctx.part_element_world_aabb(head, elem=shade_vis)
    if shade_aabb is not None:
        (sx0, sy0, sz0), (sx1, sy1, sz1) = shade_aabb
        ctx.check(
            "shade_reads_as_wide_cone",
            (sy1 - sy0) > 1.4 * (2.0 * SHADE_TOP_R),
            details=f"shade y-extent={(sy1 - sy0)}",
        )

    # Head is elevated and reaching out from the base column, not collapsed.
    head_pos = ctx.part_world_position(head)
    if head_pos is not None:
        ctx.check(
            "head_is_elevated",
            head_pos[2] > 0.18,
            details=f"head z={head_pos[2]}",
        )
        ctx.check(
            "head_reaches_out",
            abs(head_pos[0]) > 0.10,
            details=f"head x={head_pos[0]}",
        )

    # --- Pivot interlocks -------------------------------------------------
    # Each revolute hinge is a real knuckle/clevis pivot: the child's bracket
    # plate, transverse pin and proximal end nest into the parent's mating
    # bracket/ear so the joint is captured. These small embeds are intentional
    # mechanism, not stray collisions, so they are scoped allowances paired with
    # an exact contact proof that the members actually meet at the pivot.
    ctx.allow_overlap(
        base,
        lower_arm,
        elem_a="base_disc",
        elem_b="lower_arm_body",
        reason="Lower-arm shoulder knuckle/pin is captured against the base collar ear at the shoulder pivot.",
    )
    ctx.allow_overlap(
        lower_arm,
        upper_arm,
        elem_a="lower_arm_body",
        elem_b="upper_arm_body",
        reason="Upper-arm proximal knuckle/pin nests into the lower-arm distal bracket at the elbow pivot.",
    )
    ctx.allow_overlap(
        head,
        upper_arm,
        elem_a="shade_body",
        elem_b="upper_arm_body",
        reason="Head clamp stub/pin is captured by the upper-arm distal bracket at the wrist/head-tilt pivot.",
    )

    # --- Connectivity: links chain end-to-end ----------------------------
    ctx.expect_contact(lower_arm, base, name="lower_arm_touches_base", contact_tol=0.02)
    ctx.expect_contact(upper_arm, lower_arm, name="upper_arm_touches_lower_arm", contact_tol=0.02)
    ctx.expect_contact(head, upper_arm, name="head_touches_upper_arm", contact_tol=0.02)

    # --- Mechanism: actuating each joint moves its child as expected ------
    rest_head = ctx.part_world_position(head)
    rest_z = rest_head[2] if rest_head is not None else 0.0
    rest_x = rest_head[0] if rest_head is not None else 0.0

    # Shoulder: positive q (right-hand rule about +Y) pitches the lower arm's
    # +X members toward -Z, lowering the whole arm toward the desk. Confirm a
    # clear vertical response of the head, proving the shoulder actuates.
    with ctx.pose({shoulder: 0.6}):
        dropped = ctx.part_world_position(head)
        ctx.check(
            "shoulder_lowers_arm",
            dropped is not None and dropped[2] < rest_z - 0.05,
            details=f"rest_z={rest_z}, dropped={dropped}",
        )

    # Elbow: changing the elbow angle folds the arm and shifts the head in X.
    with ctx.pose({elbow: -0.6}):
        folded = ctx.part_world_position(head)
        ctx.check(
            "elbow_folds_arm",
            folded is not None and abs(folded[0] - rest_x) > 0.04,
            details=f"rest_x={rest_x}, folded={folded}",
        )

    # Wrist: tilting the head pivot rotates the whole shade about the wrist.
    # A Y-axis tilt swings the shade's top/back edge through Z and its forward
    # reach through X, so both the shade's max-z and max-x respond clearly.
    if shade_aabb is not None:
        rest_top = shade_aabb[1][2]
        rest_reach = shade_aabb[1][0]
        with ctx.pose({wrist: 0.7}):
            tilted = ctx.part_element_world_aabb(head, elem=shade_vis)
            ctx.check(
                "wrist_tilts_shade",
                tilted is not None
                and (
                    abs(tilted[1][2] - rest_top) > 0.02
                    or abs(tilted[1][0] - rest_reach) > 0.02
                ),
                details=(
                    f"rest_top={rest_top}, rest_reach={rest_reach}, "
                    f"tilted_top={tilted[1][2] if tilted else None}, "
                    f"tilted_reach={tilted[1][0] if tilted else None}"
                ),
            )

    return ctx.report()


object_model = build_object_model()
