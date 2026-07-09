from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CylinderGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# ----------------------------------------------------------------------------
# Compact backyard seesaw with triangular A-frame supports and rubber ground
# pads under each support leg.
#
# Layout (world frame, Z up, base centered on the origin):
# - Base: two A-frame triangular supports made from steel tubing (~38 mm dia),
#   spaced along the beam axis (X). Each A-frame has two angled legs meeting
#   at an apex ~0.45 m high. A horizontal crossbar connects the two apexes.
#   Rubber ground pads sit under each of the four feet.
# - Beam: a single yellow tube (~1.8 m long) pivots on the crossbar at its
#   midpoint. Each end carries a rust-brown seat plate and a yellow T-handlebar
#   just inboard of the seat.
# - Articulation: one central revolute joint, horizontal axis perpendicular to
#   the beam (Y axis), +/- 18 degrees rocking range.
# ----------------------------------------------------------------------------

TUBE_R = 0.019       # ~38 mm main tubing
BRACE_R = 0.016      # smaller brace tubing
HANDLE_R = 0.015     # handlebar tubing
SUPPORT_R = 0.017    # support leg tubing

BEAM_LEN = 1.80      # compact backyard beam
BEAM_Z_OFFSET = 0.02 # beam center above the pivot axis

APEX_Z = 0.45        # apex height of A-frame supports
LEG_SPREAD_Y = 0.28  # half-spread of legs in Y (perpendicular to beam)
FRAME_SPACING_X = 0.12  # half-spacing of the two A-frames along the beam

CROSSBAR_R = 0.020   # crossbar tube radius

SEAT_X = 0.80        # seat center from beam midpoint
SEAT_SIZE = (0.24, 0.28, 0.012)
SEAT_Z_OFFSET = -0.02  # seat below beam centerline

HANDLE_X = 0.58      # handlebar post from beam midpoint
HANDLE_POST_H = 0.26 # handlebar post height
HANDLE_BAR_W = 0.28  # handlebar crossbar half-width

PAD_SIZE = (0.10, 0.10, 0.012)  # rubber ground pad
PAD_COLOR = Material("rubber_pad", rgba=(0.12, 0.12, 0.12, 1.0))

STEEL_BLUE = Material("steel_blue_paint", rgba=(0.25, 0.41, 0.58, 1.0))
WORN_YELLOW = Material("worn_yellow_paint", rgba=(0.87, 0.74, 0.12, 1.0))
RUST_BROWN = Material("rust_brown_seat", rgba=(0.45, 0.22, 0.13, 1.0))

TILT = math.radians(18.0)  # rocking range


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


def _aframe_leg_mesh(sx: float, sy: float) -> MeshGeometry:
    """One angled leg of an A-frame: from ground foot to apex."""
    foot = (sx * FRAME_SPACING_X, sy * LEG_SPREAD_Y, 0.006)
    apex = (sx * FRAME_SPACING_X, 0.0, APEX_Z)
    return _tube_between(foot, apex, SUPPORT_R)


def _crossbar_mesh() -> MeshGeometry:
    """Horizontal crossbar connecting the two A-frame apexes along X."""
    p0 = (-FRAME_SPACING_X, 0.0, APEX_Z)
    p1 = (FRAME_SPACING_X, 0.0, APEX_Z)
    return _tube_between(p0, p1, CROSSBAR_R)


def _horizontal_brace_mesh() -> MeshGeometry:
    """Horizontal tie bar between the two A-frames at mid-height for rigidity."""
    mid_z = APEX_Z * 0.45
    # Connect corresponding legs at mid-height on each side
    result = None
    for sy in (1.0, -1.0):
        # Compute mid-height point on each leg
        frac = mid_z / APEX_Z
        for sx in (-1.0, 1.0):
            foot = (sx * FRAME_SPACING_X, sy * LEG_SPREAD_Y, 0.006)
            apex = (sx * FRAME_SPACING_X, 0.0, APEX_Z)
            mid_x = foot[0] + frac * (apex[0] - foot[0])
            mid_y = foot[1] + frac * (apex[1] - foot[1])
            if sx < 0:
                p0 = (mid_x, mid_y, mid_z)
            else:
                p1 = (mid_x, mid_y, mid_z)
        tube = _tube_between(p0, p1, BRACE_R)
        if result is None:
            result = tube
        else:
            result.merge(tube)
    return result


def _beam_truss_mesh() -> MeshGeometry:
    """Main beam tube along X with short diagonal braces near center for visual detail."""
    # Main tube
    main = CylinderGeometry(TUBE_R, BEAM_LEN, radial_segments=18)
    main.rotate_y(math.pi / 2.0)
    main.translate(0.0, 0.0, BEAM_Z_OFFSET)

    # Short diagonal braces from the pivot bracket area up to the main tube
    for sx in (1.0, -1.0):
        brace = _tube_between(
            (sx * 0.06, 0.0, 0.025),
            (sx * 0.30, 0.0, BEAM_Z_OFFSET),
            BRACE_R,
        )
        main.merge(brace)

    # Pivot bracket: short U-clamp plates wrapping over the crossbar
    bracket_h = 0.05
    bracket_w = 0.04
    for sy in (-1.0, 1.0):
        plate = _tube_between(
            (-0.06, sy * bracket_w, 0.0),
            (0.06, sy * bracket_w, 0.0),
            BRACE_R,
        )
        main.merge(plate)
    # Top tie connecting the two bracket sides
    tie = _tube_between(
        (0.0, -bracket_w, 0.0),
        (0.0, bracket_w, 0.0),
        BRACE_R,
    )
    main.merge(tie)

    # Short seat support tubes dropping from beam to seat level
    for sx in (1.0, -1.0):
        support = _tube_between(
            (sx * (SEAT_X - 0.04), 0.0, BEAM_Z_OFFSET - TUBE_R),
            (sx * SEAT_X, 0.0, SEAT_Z_OFFSET + SEAT_SIZE[2] / 2.0),
            SUPPORT_R,
        )
        main.merge(support)

    return main


def _handlebar_mesh(sx: float) -> MeshGeometry:
    """T-shaped handlebar: vertical post + horizontal crossbar."""
    post_base_z = BEAM_Z_OFFSET + TUBE_R
    post = CylinderGeometry(HANDLE_R, HANDLE_POST_H, radial_segments=14)
    post.translate(sx * HANDLE_X, 0.0, post_base_z + HANDLE_POST_H / 2.0)

    bar_top_z = post_base_z + HANDLE_POST_H
    bar = CylinderGeometry(HANDLE_R, HANDLE_BAR_W, radial_segments=14)
    bar.rotate_x(math.pi / 2.0)
    bar.translate(sx * HANDLE_X, 0.0, bar_top_z)

    return post.merge(bar)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="backyard_seesaw")

    # --- Static base: two A-frame supports + crossbar + rubber pads ----------
    base = model.part("base")

    # Four A-frame legs (two per A-frame)
    leg_idx = 0
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            leg = _aframe_leg_mesh(sx, sy)
            base.visual(
                mesh_from_geometry(leg, f"aframe_leg_{leg_idx}"),
                material=STEEL_BLUE,
                name=f"aframe_leg_{leg_idx}",
            )
            leg_idx += 1

    # Crossbar connecting apexes
    base.visual(
        mesh_from_geometry(_crossbar_mesh(), "crossbar"),
        material=STEEL_BLUE,
        name="crossbar",
    )

    # Horizontal tie braces for structural rigidity
    base.visual(
        mesh_from_geometry(_horizontal_brace_mesh(), "horizontal_brace"),
        material=STEEL_BLUE,
        name="horizontal_brace",
    )

    # Rubber ground pads under each of the four feet
    pad_idx = 0
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            foot_x = sx * FRAME_SPACING_X
            foot_y = sy * LEG_SPREAD_Y
            base.visual(
                Box(PAD_SIZE),
                origin=Origin(xyz=(foot_x, foot_y, PAD_SIZE[2] / 2.0)),
                material=PAD_COLOR,
                name=f"rubber_pad_{pad_idx}",
            )
            pad_idx += 1

    # --- Rocking beam with seats and handlebars ------------------------------
    beam = model.part("beam")

    beam.visual(
        mesh_from_geometry(_beam_truss_mesh(), "beam_truss"),
        material=WORN_YELLOW,
        name="beam_truss",
    )

    # Seats at each end
    for idx, sx in enumerate((1.0, -1.0)):
        beam.visual(
            Box(SEAT_SIZE),
            origin=Origin(xyz=(sx * SEAT_X, 0.0, SEAT_Z_OFFSET)),
            material=RUST_BROWN,
            name=f"seat_plate_{idx}",
        )

    # T-handlebars inboard of seats
    for idx, sx in enumerate((1.0, -1.0)):
        beam.visual(
            mesh_from_geometry(_handlebar_mesh(sx), f"handlebar_{idx}"),
            material=WORN_YELLOW,
            name=f"handlebar_{idx}",
        )

    # --- Central revolute pivot on the crossbar ------------------------------
    model.articulation(
        "beam_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, APEX_Z + CROSSBAR_R)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=2.5,
            lower=-TILT,
            upper=TILT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    beam = object_model.get_part("beam")
    pivot = object_model.get_articulation("beam_pivot")

    # Pivot bracket intentionally wraps the crossbar (the physical pivot axle)
    ctx.allow_overlap(
        beam,
        base,
        elem_a="beam_truss",
        elem_b="crossbar",
        reason="Beam pivot bracket intentionally wraps the crossbar which serves as the pivot axle.",
    )
    ctx.expect_contact(
        beam,
        base,
        elem_a="beam_truss",
        elem_b="crossbar",
        name="beam pivot bracket rides on the crossbar axle",
    )

    # --- Triangular support structure exists ----------------------------------
    # Four A-frame legs
    for i in range(4):
        leg_aabb = ctx.part_element_world_aabb(base, elem=f"aframe_leg_{i}")
        ctx.check(
            f"aframe_leg_{i} exists as part of the triangular support",
            leg_aabb is not None and leg_aabb[1][2] > 0.35,
            details=f"aframe_leg_{i} aabb={leg_aabb}",
        )

    # Crossbar ties the apexes together
    crossbar_aabb = ctx.part_element_world_aabb(base, elem="crossbar")
    ctx.check(
        "crossbar connects the two triangular supports at the apex",
        crossbar_aabb is not None and crossbar_aabb[0][2] > 0.40,
        details=f"crossbar aabb={crossbar_aabb}",
    )

    # --- Rubber ground pads under support legs ------------------------------
    for i in range(4):
        pad_aabb = ctx.part_element_world_aabb(base, elem=f"rubber_pad_{i}")
        ctx.check(
            f"rubber_pad_{i} is present under a support leg",
            pad_aabb is not None and pad_aabb[0][2] < 0.02 and pad_aabb[1][2] < 0.03,
            details=f"rubber_pad_{i} aabb={pad_aabb}",
        )

    # Pads should be at the outer spread (near the leg feet)
    pad0 = ctx.part_element_world_aabb(base, elem="rubber_pad_0")
    pad2 = ctx.part_element_world_aabb(base, elem="rubber_pad_2")
    if pad0 is not None and pad2 is not None:
        pad0_cy = (pad0[0][1] + pad0[1][1]) / 2.0
        pad2_cy = (pad2[0][1] + pad2[1][1]) / 2.0
        ctx.check(
            "rubber pads sit at the leg spread positions",
            abs(pad0_cy) > 0.15 and abs(pad2_cy) > 0.15,
            details=f"pad0 cy={pad0_cy:.3f}, pad2 cy={pad2_cy:.3f}",
        )

    # --- Beam has seats at opposite ends ------------------------------------
    for idx in (0, 1):
        seat_aabb = ctx.part_element_world_aabb(beam, elem=f"seat_plate_{idx}")
        ctx.check(
            f"seat_plate_{idx} exists at beam end",
            seat_aabb is not None,
            details=f"seat_plate_{idx} aabb={seat_aabb}",
        )

    # Seats are far apart (opposite ends of the beam)
    s0 = ctx.part_element_world_aabb(beam, elem="seat_plate_0")
    s1 = ctx.part_element_world_aabb(beam, elem="seat_plate_1")
    if s0 is not None and s1 is not None:
        s0_cx = (s0[0][0] + s1[1][0]) / 2.0  # intentional use of both
        ctx.check(
            "seats are at opposite ends of the beam",
            abs((s0[0][0] + s0[1][0]) / 2.0 - (s1[0][0] + s1[1][0]) / 2.0) > 1.2,
            details=f"seat0 center x={(s0[0][0]+s0[1][0])/2:.3f}, seat1 center x={(s1[0][0]+s1[1][0])/2:.3f}",
        )

    # --- Handlebars exist inboard of seats ----------------------------------
    for idx in (0, 1):
        hb_aabb = ctx.part_element_world_aabb(beam, elem=f"handlebar_{idx}")
        ctx.check(
            f"handlebar_{idx} exists on the beam",
            hb_aabb is not None,
            details=f"handlebar_{idx} aabb={hb_aabb}",
        )

    # --- Revolute joint with correct limits ---------------------------------
    lim = pivot.motion_limits
    ctx.check(
        "beam_pivot is a revolute joint rocking +/- 18 degrees",
        lim is not None
        and abs(lim.lower + TILT) < 1e-6
        and abs(lim.upper - TILT) < 1e-6,
        details=f"limits=({lim.lower if lim else None}, {lim.upper if lim else None})",
    )

    # Beam pivot axis is horizontal Y (perpendicular to beam length along X)
    ctx.check(
        "beam_pivot axis is horizontal perpendicular to beam",
        abs(pivot.axis[1]) > 0.99,
        details=f"axis={pivot.axis}",
    )

    # --- Seesaw motion: one end goes up, other goes down --------------------
    rest_s0 = ctx.part_element_world_aabb(beam, elem="seat_plate_0")
    rest_s1 = ctx.part_element_world_aabb(beam, elem="seat_plate_1")

    with ctx.pose({pivot: TILT}):
        tilt_s0 = ctx.part_element_world_aabb(beam, elem="seat_plate_0")
        tilt_s1 = ctx.part_element_world_aabb(beam, elem="seat_plate_1")
        beam_aabb = ctx.part_world_aabb(beam)
        ctx.check(
            "seesaw rocks: one seat drops while the opposite rises",
            rest_s0 is not None
            and tilt_s0 is not None
            and rest_s1 is not None
            and tilt_s1 is not None
            and tilt_s0[0][2] < rest_s0[0][2] - 0.20
            and tilt_s1[0][2] > rest_s1[0][2] + 0.20,
            details=f"seat0 {rest_s0[0][2]:.3f}->{tilt_s0[0][2]:.3f}, seat1 {rest_s1[0][2]:.3f}->{tilt_s1[0][2]:.3f}",
        )
        ctx.check(
            "tilted beam stays clear of the ground",
            beam_aabb is not None and beam_aabb[0][2] > 0.01,
            details=f"beam aabb min z={beam_aabb[0][2]:.3f}",
        )

    # --- Compact backyard scale ---------------------------------------------
    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "support structure height is compact (~0.4-0.55 m)",
        base_aabb is not None and 0.40 <= base_aabb[1][2] <= 0.58,
        details=f"base top z={base_aabb[1][2]:.3f}",
    )

    beam_aabb_rest = ctx.part_world_aabb(beam)
    ctx.check(
        "beam is compact length (~1.5-2.0 m)",
        beam_aabb_rest is not None
        and 1.5 <= (beam_aabb_rest[1][0] - beam_aabb_rest[0][0]) <= 2.1,
        details=f"beam length={beam_aabb_rest[1][0]-beam_aabb_rest[0][0]:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
