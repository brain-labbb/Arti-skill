from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoxGeometry,
    CylinderGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ----------------------------------------------------------------------------
# Playground seesaw variant: single beam on a central A-frame support.
#
# Layout (world frame, Z up, base centered on the origin):
# - Sky-blue A-frame: two splayed tube legs meeting at a top apex with a
#   crossbar for lateral stability, about 0.70 m tall.
# - Visible axle bracket plates flanking the apex where the beam pivots.
# - One yellow rocking beam (~2.6 m) running east-west (along X).
# - Each end carries a molded seat with raised lips and a T-handlebar.
# - A locking pin rotates near the central bracket (revolute joint).
# - Articulation: beam pivots on a horizontal axis (Y) at the apex, +/- 18 deg.
# - Locking pin rotates about Y from 0 (locked, horizontal) to ~1.4 rad (unlocked).
# ----------------------------------------------------------------------------

TUBE_R = 0.020          # ~40 mm diameter main tubing
BRACE_R = 0.016
SUPPORT_R = 0.018
HANDLE_R = 0.016
PIN_R = 0.008           # locking pin radius
PIN_LEN = 0.10          # locking pin length

TILT = math.radians(18.0)  # beam rocking range

A_FRAME_HEIGHT = 0.70
A_FRAME_HALF_SPAN = 0.34   # ground half-spread of the A-frame legs
A_FRAME_DEPTH = 0.22       # front-to-back depth of the A-frame base
CROSSBAR_Z = 0.22          # height of the lower crossbar

BEAM_LEN = 2.60
MAIN_Z = 0.06              # main top tube height above the pivot axis
SLEEVE_R = 0.032
SLEEVE_LEN = 0.12
SEAT_X = 1.15              # seat center along beam from pivot
SEAT_Z = 0.01              # seat mounting height above pivot axis
HANDLE_X = 0.80            # T-handlebar post, inboard of the seat
HANDLE_TOP_Z = 0.30        # crossbar height above pivot axis

BRACKET_W = 0.08           # bracket plate width (along beam)
BRACKET_H = 0.07           # bracket plate height
BRACKET_T = 0.006          # bracket plate thickness

SKY_BLUE = Material("sky_blue_paint", rgba=(0.33, 0.62, 0.84, 1.0))
WORN_YELLOW = Material("worn_yellow_paint", rgba=(0.87, 0.74, 0.12, 1.0))
RUST_BROWN = Material("rust_brown_steel", rgba=(0.42, 0.21, 0.13, 1.0))
DARK_GREEN = Material("dark_green_plastic", rgba=(0.15, 0.35, 0.18, 1.0))
ZINC_GRAY = Material("zinc_gray", rgba=(0.55, 0.55, 0.52, 1.0))


def _tube_between(
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    radius: float,
    *,
    radial_segments: int = 16,
) -> MeshGeometry:
    """Straight capped tube between two 3D points."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    geom = CylinderGeometry(radius, length, radial_segments=radial_segments)
    ux, uy, uz = dx / length, dy / length, dz / length
    ax, ay, az = -uy, ux, 0.0
    s = math.sqrt(ax * ax + ay * ay + az * az)
    if s > 1e-9:
        geom.rotate((ax / s, ay / s, az / s), math.atan2(s, uz))
    elif uz < 0.0:
        geom.rotate_x(math.pi)
    geom.translate(
        (p0[0] + p1[0]) / 2.0,
        (p0[1] + p1[1]) / 2.0,
        (p0[2] + p1[2]) / 2.0,
    )
    return geom


def _build_aframe() -> list[tuple[MeshGeometry, str]]:
    """Build the A-frame support from tube segments.

    Returns a list of (mesh, name) pairs.
    Two splayed legs meeting at the apex, plus a lower crossbar.
    """
    apex = (0.0, 0.0, A_FRAME_HEIGHT)
    # Four feet: front-left, front-right, back-left, back-right
    # But for an A-frame, we have two legs in the YZ plane
    # Leg 1: from (0, -A_FRAME_HALF_SPAN, 0) up to apex
    # Leg 2: from (0, +A_FRAME_HALF_SPAN, 0) up to apex
    # Crossbar ties the two legs at CROSSBAR_Z

    foot_neg = (0.0, -A_FRAME_HALF_SPAN, 0.022)
    foot_pos = (0.0, A_FRAME_HALF_SPAN, 0.022)

    meshes: list[tuple[MeshGeometry, str]] = []

    # Leg 1 (negative Y foot to apex)
    leg1 = tube_from_spline_points(
        [foot_neg, (0.0, -A_FRAME_HALF_SPAN * 0.3, A_FRAME_HEIGHT * 0.7), apex],
        radius=TUBE_R,
        samples_per_segment=10,
        radial_segments=16,
        cap_ends=True,
    )
    meshes.append((leg1, "leg_neg"))

    # Leg 2 (positive Y foot to apex)
    leg2 = tube_from_spline_points(
        [foot_pos, (0.0, A_FRAME_HALF_SPAN * 0.3, A_FRAME_HEIGHT * 0.7), apex],
        radius=TUBE_R,
        samples_per_segment=10,
        radial_segments=16,
        cap_ends=True,
    )
    meshes.append((leg2, "leg_pos"))

    # Lower crossbar between the two legs
    # Find the Y positions on each leg at CROSSBAR_Z
    frac = CROSSBAR_Z / A_FRAME_HEIGHT
    y_neg = -A_FRAME_HALF_SPAN * (1.0 - frac * 0.7)
    y_pos = A_FRAME_HALF_SPAN * (1.0 - frac * 0.7)
    crossbar = _tube_between(
        (0.0, y_neg, CROSSBAR_Z),
        (0.0, y_pos, CROSSBAR_Z),
        SUPPORT_R,
    )
    meshes.append((crossbar, "crossbar"))

    # Small foot plates at ground level - taller to contact the leg bottoms
    for sy, foot_name in [(-1.0, "foot_neg"), (1.0, "foot_pos")]:
        # Foot extends from z=0 to z=0.025 to contact leg at z=0.022
        foot = CylinderGeometry(0.035, 0.025, radial_segments=16).translate(
            0.0, sy * A_FRAME_HALF_SPAN, 0.0125
        )
        meshes.append((foot, foot_name))

    return meshes


def _build_brackets() -> list[tuple[MeshGeometry, str]]:
    """Build two axle bracket plates flanking the A-frame apex.

    Each bracket is a flat plate with a visible profile near the apex.
    Returns list of (mesh, name) pairs.
    """
    bracket_z = A_FRAME_HEIGHT
    offset_y = 0.025  # distance from center to each bracket plate (contacts leg surface)

    results = []
    for sy, name in [(-1.0, "bracket_neg"), (1.0, "bracket_pos")]:
        # Flat plate: BRACKET_W along X, BRACKET_T along Y, BRACKET_H along Z
        plate = BoxGeometry((BRACKET_W, BRACKET_T, BRACKET_H)).translate(
            0.0, sy * offset_y, bracket_z
        )
        results.append((plate, name))

    return results


def _build_molded_seat() -> MeshGeometry:
    """Build a molded seat with raised lips from box primitives.

    The seat is centered at origin (XY), oriented with X along beam, Y lateral.
    Raised lips on sides and back make it read as a molded plastic seat.
    Total height including lips: ~0.053 m.
    """
    seat_w = 0.28   # along X (beam direction)
    seat_d = 0.30   # along Y (lateral)
    seat_t = 0.018  # base plate thickness
    lip_h = 0.035   # lip height above seat surface
    lip_t = 0.012   # lip wall thickness
    back_h = lip_h * 1.3  # back lip taller

    # Base plate centered at z=0
    base = BoxGeometry((seat_w, seat_d, seat_t))

    # Side lips (raised walls along Y edges)
    left_lip = BoxGeometry((seat_w, lip_t, lip_h)).translate(
        0.0, seat_d / 2.0 - lip_t / 2.0, seat_t / 2.0 + lip_h / 2.0
    )
    right_lip = BoxGeometry((seat_w, lip_t, lip_h)).translate(
        0.0, -(seat_d / 2.0 - lip_t / 2.0), seat_t / 2.0 + lip_h / 2.0
    )

    # Back lip (taller wall along -X edge)
    back_lip = BoxGeometry((lip_t, seat_d - 2.0 * lip_t, back_h)).translate(
        -(seat_w / 2.0 - lip_t / 2.0), 0.0, seat_t / 2.0 + back_h / 2.0
    )

    seat = base.merge(left_lip).merge(right_lip).merge(back_lip)
    return seat


def _build_beam() -> tuple[MeshGeometry, MeshGeometry, MeshGeometry, MeshGeometry]:
    """Build one rocking beam in its local frame (X along beam, pivot at origin).

    Returns (truss_tube, axle_sleeve, handlebar_pos_x, handlebar_neg_x).
    """
    # Main top tube
    truss = (
        CylinderGeometry(TUBE_R, BEAM_LEN, radial_segments=18)
        .rotate_y(math.pi / 2.0)
        .translate(0.0, 0.0, MAIN_Z)
    )
    for sx in (1.0, -1.0):
        # Diagonal brace from axle sleeve up to main tube
        truss.merge(
            _tube_between(
                (sx * 0.04, 0.0, 0.005),
                (sx * 0.55, 0.0, MAIN_Z),
                BRACE_R,
            )
        )
        # Short bent seat support
        truss.merge(
            tube_from_spline_points(
                [
                    (sx * 0.96, 0.0, MAIN_Z),
                    (sx * 1.06, 0.0, 0.040),
                    (sx * 1.12, 0.0, 0.018),
                    (sx * 1.20, 0.0, 0.010),
                ],
                radius=SUPPORT_R,
                samples_per_segment=10,
                radial_segments=14,
                cap_ends=True,
            )
        )

    # Axle sleeve
    sleeve = (
        CylinderGeometry(SLEEVE_R, SLEEVE_LEN, radial_segments=20)
        .rotate_x(math.pi / 2.0)
    )
    # Weld post connecting sleeve to main tube
    weld_post = CylinderGeometry(0.014, MAIN_Z - 0.024, radial_segments=14).translate(
        0.0, 0.0, (MAIN_Z + 0.024) / 2.0
    )
    sleeve.merge(weld_post)

    handlebars: list[MeshGeometry] = []
    for sx in (1.0, -1.0):
        post = CylinderGeometry(HANDLE_R, 0.26, radial_segments=14).translate(
            sx * HANDLE_X, 0.0, MAIN_Z + 0.12
        )
        bar = (
            CylinderGeometry(HANDLE_R, 0.28, radial_segments=14)
            .rotate_x(math.pi / 2.0)
            .translate(sx * HANDLE_X, 0.0, HANDLE_TOP_Z)
        )
        handlebars.append(post.merge(bar))

    return truss, sleeve, handlebars[0], handlebars[1]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="aframe_playground_seesaw")

    # --- A-frame base --------------------------------------------------------
    base = model.part("aframe_base")
    for mesh, name in _build_aframe():
        base.visual(
            mesh_from_geometry(mesh, f"aframe_{name}"),
            material=SKY_BLUE,
            name=name,
        )

    # Axle brackets (two plates flanking the apex)
    for mesh, name in _build_brackets():
        base.visual(
            mesh_from_geometry(mesh, f"aframe_{name}"),
            material=ZINC_GRAY,
            name=name,
        )

    # --- Rocking beam --------------------------------------------------------
    truss, sleeve, hb0, hb1 = _build_beam()
    beam = model.part("beam")
    beam.visual(
        mesh_from_geometry(truss, "beam_truss"),
        material=WORN_YELLOW,
        name="truss_tube",
    )
    beam.visual(
        mesh_from_geometry(sleeve, "beam_sleeve"),
        material=WORN_YELLOW,
        name="axle_sleeve",
    )
    beam.visual(
        mesh_from_geometry(hb0, "beam_handlebar_0"),
        material=WORN_YELLOW,
        name="handlebar_0",
    )
    beam.visual(
        mesh_from_geometry(hb1, "beam_handlebar_1"),
        material=WORN_YELLOW,
        name="handlebar_1",
    )

    # Molded seats with raised lips - placed via visual origin
    seat_mesh = _build_molded_seat()
    for sx, seat_name in [(1.0, "seat_0"), (-1.0, "seat_1")]:
        beam.visual(
            mesh_from_geometry(seat_mesh.clone(), f"beam_{seat_name}"),
            origin=Origin(xyz=(sx * SEAT_X, 0.0, SEAT_Z)),
            material=DARK_GREEN,
            name=seat_name,
        )

    # --- Locking pin ---------------------------------------------------------
    pin = model.part("locking_pin")
    # Pin is a small cylinder with a head, oriented along Y near the bracket edge.
    # Built in pin-local frame: shaft extends along +Y from the rotation axis.
    pin_shaft = (
        CylinderGeometry(PIN_R, PIN_LEN, radial_segments=12)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, PIN_LEN / 2.0, 0.0)
    )
    # Pin head (wider disk at origin, touching the bracket)
    pin_head = (
        CylinderGeometry(PIN_R * 3.0, 0.010, radial_segments=14)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, -0.005, 0.0)
    )
    pin_shaft.merge(pin_head)
    pin.visual(
        mesh_from_geometry(pin_shaft, "pin_body"),
        material=ZINC_GRAY,
        name="pin_body",
    )

    # --- Articulations -------------------------------------------------------
    # Beam pivot: revolute, axis along Y (perpendicular to beam length)
    model.articulation(
        "beam_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, A_FRAME_HEIGHT)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=150.0, velocity=2.5, lower=-TILT, upper=TILT),
    )

    # Locking pin: revolute, axis along Z, rotates from horizontal (locked) to upright
    # Pin placed at the outer edge of bracket_pos, contacting the bracket face
    model.articulation(
        "pin_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=pin,
        origin=Origin(xyz=(0.04, 0.028, A_FRAME_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.5, lower=0.0, upper=1.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("aframe_base")
    beam = object_model.get_part("beam")
    pin = object_model.get_part("locking_pin")
    beam_pivot = object_model.get_articulation("beam_pivot")
    pin_joint = object_model.get_articulation("pin_joint")

    # --- A-frame geometry checks ---
    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "A-frame base is about 0.7 m tall",
        base_aabb is not None and 0.65 <= base_aabb[1][2] <= 0.80,
        details=f"base aabb={base_aabb}",
    )
    ctx.check(
        "A-frame feet rest on the ground plane",
        base_aabb is not None and -0.01 <= base_aabb[0][2] <= 0.02,
        details=f"base aabb={base_aabb}",
    )

    # Visible axle brackets exist on the base
    bracket_neg_aabb = ctx.part_element_world_aabb(base, elem="bracket_neg")
    bracket_pos_aabb = ctx.part_element_world_aabb(base, elem="bracket_pos")
    ctx.check(
        "A-frame has visible axle bracket plates",
        bracket_neg_aabb is not None and bracket_pos_aabb is not None,
        details=f"bracket_neg={bracket_neg_aabb}, bracket_pos={bracket_pos_aabb}",
    )
    # Brackets should be near the apex height
    if bracket_neg_aabb and bracket_pos_aabb:
        bracket_center_z = (
            (bracket_neg_aabb[0][2] + bracket_neg_aabb[1][2]) / 2.0
            + (bracket_pos_aabb[0][2] + bracket_pos_aabb[1][2]) / 2.0
        ) / 2.0
        ctx.check(
            "bracket plates are positioned near the A-frame apex",
            abs(bracket_center_z - A_FRAME_HEIGHT) < 0.06,
            details=f"bracket center z={bracket_center_z:.3f}, apex={A_FRAME_HEIGHT}",
        )

    # --- Captured-axle fit: sleeve wraps the apex leg tubes ---
    ctx.allow_overlap(
        beam,
        base,
        elem_a="axle_sleeve",
        elem_b="leg_neg",
        reason="Beam axle sleeve intentionally wraps the negative leg tube at the apex, forming the pivot axle.",
    )
    ctx.allow_overlap(
        beam,
        base,
        elem_a="axle_sleeve",
        elem_b="leg_pos",
        reason="Beam axle sleeve intentionally wraps the positive leg tube at the apex, forming the pivot axle.",
    )
    ctx.allow_overlap(
        beam,
        base,
        elem_a="axle_sleeve",
        elem_b="bracket_neg",
        reason="Beam axle sleeve passes through the bracket_neg axle region at the apex.",
    )
    ctx.allow_overlap(
        beam,
        base,
        elem_a="axle_sleeve",
        elem_b="bracket_pos",
        reason="Beam axle sleeve passes through the bracket_pos axle region at the apex.",
    )
    ctx.expect_contact(
        beam,
        base,
        elem_a="axle_sleeve",
        elem_b="leg_neg",
        name="beam sleeve rides on the negative leg axle",
    )
    ctx.expect_contact(
        beam,
        base,
        elem_a="axle_sleeve",
        elem_b="leg_pos",
        name="beam sleeve rides on the positive leg axle",
    )

    # --- Molded seats with raised lips ---
    for end in (0, 1):
        seat_aabb = ctx.part_element_world_aabb(beam, elem=f"seat_{end}")
        ctx.check(
            f"molded seat {end} exists on the beam",
            seat_aabb is not None,
            details=f"seat_{end} aabb={seat_aabb}",
        )
        if seat_aabb:
            seat_height = seat_aabb[1][2] - seat_aabb[0][2]
            # Molded seats with lips should be taller than a flat 12mm plate
            ctx.check(
                f"seat {end} has raised lips (height > 30mm)",
                seat_height > 0.030,
                details=f"seat height={seat_height:.4f}m",
            )
            # Seats should be near the beam ends
            seat_cx = (seat_aabb[0][0] + seat_aabb[1][0]) / 2.0
            ctx.check(
                f"seat {end} is positioned near the beam end",
                abs(seat_cx) > 0.95,
                details=f"seat center x={seat_cx:.3f}",
            )

    # --- Handlebars ---
    for end in (0, 1):
        handle_aabb = ctx.part_element_world_aabb(beam, elem=f"handlebar_{end}")
        seat_aabb = ctx.part_element_world_aabb(beam, elem=f"seat_{end}")
        ctx.check(
            f"handlebar {end} exists and stands upright inboard of its seat",
            handle_aabb is not None
            and seat_aabb is not None
            and handle_aabb[1][2] > seat_aabb[1][2] + 0.10,
            details=f"handle={handle_aabb}, seat={seat_aabb}",
        )

    # --- Locking pin ---
    # Pin overlaps are intentional: mounted on bracket_pos, shaft passes near sleeve
    ctx.allow_overlap(
        pin,
        base,
        elem_a="pin_body",
        elem_b="bracket_pos",
        reason="Locking pin is mounted directly on the bracket_pos plate surface.",
    )
    ctx.allow_overlap(
        pin,
        beam,
        elem_a="pin_body",
        elem_b="axle_sleeve",
        reason="Locking pin shaft passes near the axle sleeve to lock/unlock the beam.",
    )

    pin_aabb = ctx.part_world_aabb(pin)
    ctx.check(
        "locking pin exists near the central bracket",
        pin_aabb is not None and pin_aabb[0][2] > A_FRAME_HEIGHT - 0.10,
        details=f"pin aabb={pin_aabb}",
    )
    ctx.expect_contact(
        pin,
        base,
        elem_a="pin_body",
        elem_b="bracket_pos",
        name="locking pin is mounted on the bracket plate",
    )

    # Pin joint has non-trivial range
    pin_lim = pin_joint.motion_limits
    ctx.check(
        "locking pin has a non-fixed revolute joint with meaningful range",
        pin_lim is not None and pin_lim.upper - pin_lim.lower > 0.5,
        details=f"pin limits=({pin_lim.lower if pin_lim else None}, {pin_lim.upper if pin_lim else None})",
    )

    # Pin articulation: check that the shaft tip moves (AABB changes in Y as it rotates around Z)
    rest_pin_aabb = ctx.part_element_world_aabb(pin, elem="pin_body")
    with ctx.pose({pin_joint: 1.4}):
        rotated_pin_aabb = ctx.part_element_world_aabb(pin, elem="pin_body")
        ctx.check(
            "locking pin rotates when articulated (shaft tip moves laterally)",
            rest_pin_aabb is not None
            and rotated_pin_aabb is not None
            and (abs(rotated_pin_aabb[1][1] - rest_pin_aabb[1][1]) > 0.03
                 or abs(rotated_pin_aabb[0][0] - rest_pin_aabb[0][0]) > 0.03),
            details=f"rest={rest_pin_aabb}, rotated={rotated_pin_aabb}",
        )

    # --- Beam pivot articulation ---
    beam_lim = beam_pivot.motion_limits
    ctx.check(
        "beam pivot rocks +/- 18 degrees",
        beam_lim is not None
        and abs(beam_lim.lower + TILT) < 1e-6
        and abs(beam_lim.upper - TILT) < 1e-6,
        details=f"limits=({beam_lim.lower if beam_lim else None}, {beam_lim.upper if beam_lim else None})",
    )

    # Decisive pose: beam seesaws
    rest_seat0 = ctx.part_element_world_aabb(beam, elem="seat_0")
    rest_seat1 = ctx.part_element_world_aabb(beam, elem="seat_1")
    with ctx.pose({beam_pivot: TILT}):
        tilt_seat0 = ctx.part_element_world_aabb(beam, elem="seat_0")
        tilt_seat1 = ctx.part_element_world_aabb(beam, elem="seat_1")
        beam_aabb = ctx.part_world_aabb(beam)
        ctx.check(
            "beam seesaws: one end drops, the opposite end rises",
            rest_seat0 is not None
            and tilt_seat0 is not None
            and rest_seat1 is not None
            and tilt_seat1 is not None
            and tilt_seat0[0][2] < rest_seat0[0][2] - 0.25
            and tilt_seat1[0][2] > rest_seat1[0][2] + 0.25,
            details=f"seat0 {rest_seat0} -> {tilt_seat0}, seat1 {rest_seat1} -> {tilt_seat1}",
        )
        ctx.check(
            "fully tilted beam stays clear of the ground",
            beam_aabb is not None and beam_aabb[0][2] > 0.02,
            details=f"beam aabb={beam_aabb}",
        )
        ctx.expect_contact(
            beam,
            base,
            elem_a="axle_sleeve",
            elem_b="leg_neg",
            name="tilted beam sleeve stays on its axle",
        )

    return ctx.report()


object_model = build_object_model()
