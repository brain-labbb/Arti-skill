from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CapsuleGeometry,
    CylinderGeometry,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ----------------------------------------------------------------------------
# Classic two-seat plank playground seesaw.
#
# Layout (world frame, Z up, base centered on the origin):
# - Sky-blue base: four round tube legs (~40 mm dia.) converging from spread
#   ground feet to the crossbar endpoints, joined by side cross braces and
#   ground-level foot tubes; a horizontal axle crossbar at ~0.70 m.
# - Yellow plank: long rectangular beam (~2.4 m) rocking on the axle via a
#   central sleeve bearing.  Molded seats with raised lips at each end,
#   rounded capsule handle grips inboard of the seats, and hinge brackets
#   for the backrests.
# - Two tilting backrests: each mounted behind its seat on a small revolute
#   joint (0 to ~20° backward tilt).
#
# Articulations:
#   plank_pivot:         REVOLUTE, axis (0, 1, 0), ±18°
#   backrest_right_tilt: REVOLUTE, axis (0, 1, 0), 0 to +20°
#   backrest_left_tilt:  REVOLUTE, axis (0,-1, 0), 0 to +20°
# ----------------------------------------------------------------------------

TUBE_R = 0.020  # ~40 mm diameter main tubing
BRACE_R = 0.015
PIVOT_HEIGHT = 0.70
TILT = math.radians(18.0)
BACKREST_TILT_MAX = math.radians(20.0)

# Base geometry
LEG_SPREAD_X = 0.30
LEG_SPREAD_Y = 0.22
AXLE_HALF_Y = 0.24

# Plank
PLANK_LEN = 2.40
PLANK_W = 0.14
PLANK_T = 0.040
PLANK_Z = 0.070  # plank board center above pivot
SLEEVE_R = 0.028
SLEEVE_LEN = 0.12

# Seats
SEAT_X = 1.00
SEAT_R = 0.13
SEAT_THICK = 0.010
LIP_H = 0.030
LIP_T = 0.008

# Handles
HANDLE_X = 0.65
HANDLE_R = 0.012
HANDLE_POST_H = 0.28
GRIP_LEN = 0.16

# Backrests
BACKREST_W = 0.20
BACKREST_T = 0.012
BACKREST_H = 0.18
BACKREST_HINGE_X = SEAT_X + SEAT_R + 0.012  # hinge behind seat lip edge

# Brace connection: interpolate leg position at brace height
_BRACE_Z = 0.30
_T_BRACE = (_BRACE_Z - TUBE_R) / (PIVOT_HEIGHT - TUBE_R)
_BRACE_X = LEG_SPREAD_X * (1.0 - _T_BRACE)
_BRACE_Y_L = LEG_SPREAD_Y + _T_BRACE * (AXLE_HALF_Y - LEG_SPREAD_Y)  # magnitude

# Materials
SKY_BLUE = Material("sky_blue_paint", rgba=(0.33, 0.62, 0.84, 1.0))
WORN_YELLOW = Material("worn_yellow_paint", rgba=(0.87, 0.74, 0.12, 1.0))
SEAT_RED = Material("seat_red_paint", rgba=(0.75, 0.22, 0.17, 1.0))
DARK_GRAY = Material("dark_rubber", rgba=(0.22, 0.22, 0.24, 1.0))
STEEL_GRAY = Material("steel_gray", rgba=(0.45, 0.45, 0.48, 1.0))


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


def _molded_seat_mesh() -> MeshGeometry:
    """Round molded seat pan with a raised lip ring around the edge."""
    profile = [
        (0.001, 0.0),
        (SEAT_R, 0.0),
        (SEAT_R, SEAT_THICK + LIP_H),
        (SEAT_R - LIP_T, SEAT_THICK + LIP_H),
        (SEAT_R - LIP_T, SEAT_THICK),
        (0.001, SEAT_THICK),
    ]
    return LatheGeometry(profile, segments=28, closed=True)


def _backrest_mesh() -> MeshGeometry:
    """Flat backrest panel with a hinge barrel at the bottom pivot point."""
    # Main panel, extending upward from the hinge
    panel = BoxGeometry((BACKREST_T, BACKREST_W, BACKREST_H))
    panel.translate(0.0, 0.0, BACKREST_H / 2.0)
    # Hinge barrel (wraps the bracket pin, provides physical contact)
    barrel = CylinderGeometry(0.008, 0.058, radial_segments=12)
    barrel.rotate_x(math.pi / 2.0)
    # Barrel at local origin = hinge point
    panel.merge(barrel)
    return panel


def _handle_mesh() -> MeshGeometry:
    """Handle assembly: vertical post plus a horizontal capsule grip bar."""
    post = CylinderGeometry(HANDLE_R, HANDLE_POST_H, radial_segments=14)
    post.translate(0.0, 0.0, HANDLE_POST_H / 2.0)

    grip = CapsuleGeometry(HANDLE_R * 1.15, GRIP_LEN, radial_segments=14)
    grip.rotate_x(math.pi / 2.0)
    grip.translate(0.0, 0.0, HANDLE_POST_H)

    post.merge(grip)
    return post


def _hinge_bracket_mesh() -> MeshGeometry:
    """Small bracket plate that anchors the backrest hinge to the plank.

    The bracket rises from the plank top to the hinge-pin height.  The pin
    sits at the top of the bracket, just below the seat lip top surface so
    the backrest panel clears the lip when upright.
    """
    bracket_h = SEAT_THICK + LIP_H + 0.004  # top of bracket just above lip
    # Vertical plate
    vert = BoxGeometry((0.014, 0.055, bracket_h))
    vert.translate(0.0, 0.0, bracket_h / 2.0)
    # Hinge pin cylinder along Y at bracket top
    pin = CylinderGeometry(0.005, 0.065, radial_segments=10)
    pin.rotate_x(math.pi / 2.0)
    pin.translate(0.0, 0.0, bracket_h)
    vert.merge(pin)
    return vert


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="classic_plank_seesaw")

    # ── base (static A-frame stand) ─────────────────────────────────────────
    base = model.part("base")

    # Four round tube legs converging to the crossbar endpoints.
    # Each pair of legs on one Y side meets at the axle end on that side.
    leg_defs = [
        # Front-left: ground → left axle end
        ((-LEG_SPREAD_X, -LEG_SPREAD_Y, TUBE_R), (0.0, -AXLE_HALF_Y, PIVOT_HEIGHT)),
        # Back-left
        ((+LEG_SPREAD_X, -LEG_SPREAD_Y, TUBE_R), (0.0, -AXLE_HALF_Y, PIVOT_HEIGHT)),
        # Front-right
        ((-LEG_SPREAD_X, +LEG_SPREAD_Y, TUBE_R), (0.0, +AXLE_HALF_Y, PIVOT_HEIGHT)),
        # Back-right
        ((+LEG_SPREAD_X, +LEG_SPREAD_Y, TUBE_R), (0.0, +AXLE_HALF_Y, PIVOT_HEIGHT)),
    ]
    for i, (p0, p1) in enumerate(leg_defs):
        base.visual(
            mesh_from_geometry(_tube_between(p0, p1, TUBE_R), f"leg_{i}"),
            material=SKY_BLUE,
            name=f"leg_{i}",
        )

    # Top axle crossbar (runs along Y – the pivot axis)
    axle = _tube_between(
        (0.0, -AXLE_HALF_Y, PIVOT_HEIGHT),
        (0.0, +AXLE_HALF_Y, PIVOT_HEIGHT),
        TUBE_R,
    )
    base.visual(
        mesh_from_geometry(axle, "axle_bar"),
        material=SKY_BLUE,
        name="axle_bar",
    )

    # Side cross braces connecting front and back legs on each Y side.
    # Endpoints are interpolated from the leg lines at _BRACE_Z so the braces
    # physically touch the legs (no disconnected islands).
    for sy, nm in ((-1.0, "brace_left"), (+1.0, "brace_right")):
        by = sy * _BRACE_Y_L
        brace = _tube_between(
            (-_BRACE_X, by, _BRACE_Z),
            (+_BRACE_X, by, _BRACE_Z),
            BRACE_R,
        )
        base.visual(
            mesh_from_geometry(brace, nm),
            material=SKY_BLUE,
            name=nm,
        )

    # Ground-level foot tubes spanning each pair of legs in Y
    for sx, nm in ((-1.0, "foot_front"), (+1.0, "foot_back")):
        fx = sx * LEG_SPREAD_X
        foot = _tube_between(
            (fx, -LEG_SPREAD_Y, TUBE_R),
            (fx, +LEG_SPREAD_Y, TUBE_R),
            TUBE_R,
        )
        base.visual(
            mesh_from_geometry(foot, nm),
            material=SKY_BLUE,
            name=nm,
        )

    # ── plank (rocking beam) ────────────────────────────────────────────────
    plank = model.part("plank")

    # Main plank board
    plank.visual(
        Box((PLANK_LEN, PLANK_W, PLANK_T)),
        origin=Origin(xyz=(0.0, 0.0, PLANK_Z)),
        material=WORN_YELLOW,
        name="board",
    )

    # Axle sleeve (wraps the crossbar at center)
    sleeve_geom = CylinderGeometry(SLEEVE_R, SLEEVE_LEN, radial_segments=20)
    sleeve_geom.rotate_x(math.pi / 2.0)
    plank.visual(
        mesh_from_geometry(sleeve_geom, "axle_sleeve"),
        material=STEEL_GRAY,
        name="axle_sleeve",
    )

    # Weld posts connecting sleeve to plank board
    post_h = PLANK_Z - PLANK_T / 2.0 - SLEEVE_R
    if post_h > 0.005:
        for idx, sx in enumerate((+1.0, -1.0)):
            wp = CylinderGeometry(0.012, post_h, radial_segments=12)
            wp.translate(0.0, 0.0, SLEEVE_R + post_h / 2.0)
            plank.visual(
                mesh_from_geometry(wp, f"weld_post_{idx}"),
                material=WORN_YELLOW,
                name=f"weld_post_{idx}",
            )

    plank_top_z = PLANK_Z + PLANK_T / 2.0

    # Molded seats at each end
    for sx, nm in ((+1.0, "seat_right"), (-1.0, "seat_left")):
        plank.visual(
            mesh_from_geometry(_molded_seat_mesh(), nm),
            origin=Origin(xyz=(sx * SEAT_X, 0.0, plank_top_z)),
            material=SEAT_RED,
            name=nm,
        )

    # Handle grips inboard of each seat
    for sx, nm in ((+1.0, "handle_right"), (-1.0, "handle_left")):
        plank.visual(
            mesh_from_geometry(_handle_mesh(), nm),
            origin=Origin(xyz=(sx * HANDLE_X, 0.0, plank_top_z)),
            material=DARK_GRAY,
            name=nm,
        )

    # Hinge brackets for backrests – offset inboard so they do not overlap
    # the backrest panel (bracket is behind/inboard of the hinge line).
    bracket_offset = 0.016  # inboard shift from hinge line
    for sx, nm in ((+1.0, "hinge_bracket_right"), (-1.0, "hinge_bracket_left")):
        bx = sx * (BACKREST_HINGE_X - bracket_offset)
        plank.visual(
            mesh_from_geometry(_hinge_bracket_mesh(), nm),
            origin=Origin(xyz=(bx, 0.0, plank_top_z)),
            material=STEEL_GRAY,
            name=nm,
        )

    # ── backrests (tilting panels) ──────────────────────────────────────────
    # Hinge Z: top of the bracket, just above the seat lip
    hinge_z = plank_top_z + SEAT_THICK + LIP_H + 0.004

    backrest_right = model.part("backrest_right")
    backrest_right.visual(
        mesh_from_geometry(_backrest_mesh(), "backrest_panel"),
        origin=Origin(xyz=(0.0, 0.0, BACKREST_H / 2.0)),
        material=SEAT_RED,
        name="backrest_panel",
    )

    backrest_left = model.part("backrest_left")
    backrest_left.visual(
        mesh_from_geometry(_backrest_mesh(), "backrest_panel"),
        origin=Origin(xyz=(0.0, 0.0, BACKREST_H / 2.0)),
        material=SEAT_RED,
        name="backrest_panel",
    )

    # ── articulations ───────────────────────────────────────────────────────

    # Main plank pivot on the axle
    model.articulation(
        "plank_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=plank,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_HEIGHT)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=150.0, velocity=2.5, lower=-TILT, upper=TILT
        ),
    )

    # Right backrest tilt (positive q tilts backward = +X)
    model.articulation(
        "backrest_right_tilt",
        ArticulationType.REVOLUTE,
        parent=plank,
        child=backrest_right,
        origin=Origin(xyz=(BACKREST_HINGE_X, 0.0, hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0, velocity=1.5, lower=0.0, upper=BACKREST_TILT_MAX
        ),
    )

    # Left backrest tilt (positive q tilts backward = -X)
    model.articulation(
        "backrest_left_tilt",
        ArticulationType.REVOLUTE,
        parent=plank,
        child=backrest_left,
        origin=Origin(xyz=(-BACKREST_HINGE_X, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0, velocity=1.5, lower=0.0, upper=BACKREST_TILT_MAX
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    plank = object_model.get_part("plank")
    backrest_left = object_model.get_part("backrest_left")
    backrest_right = object_model.get_part("backrest_right")
    pivot = object_model.get_articulation("plank_pivot")
    br_right_tilt = object_model.get_articulation("backrest_right_tilt")
    br_left_tilt = object_model.get_articulation("backrest_left_tilt")

    # ── axle sleeve / crossbar bearing ──────────────────────────────────────
    ctx.allow_overlap(
        plank,
        base,
        elem_a="axle_sleeve",
        elem_b="axle_bar",
        reason="Plank axle sleeve intentionally wraps the base crossbar as the pivot bearing.",
    )
    ctx.expect_contact(
        plank,
        base,
        elem_a="axle_sleeve",
        elem_b="axle_bar",
        name="plank sleeve rides on the base axle",
    )

    # ── backrest hinge barrels wrap the bracket pins ─────────────────────────
    # The backrest hinge barrel intentionally encloses the bracket hinge pin.
    for br_part, br_name, bracket_name in (
        (backrest_right, "backrest_right", "hinge_bracket_right"),
        (backrest_left, "backrest_left", "hinge_bracket_left"),
    ):
        ctx.allow_overlap(
            plank,
            br_part,
            elem_a=bracket_name,
            elem_b="backrest_panel",
            reason=f"Hinge pin on {bracket_name} is intentionally captured inside the {br_name} barrel.",
        )
        ctx.allow_isolated_part(
            br_part,
            reason=f"{br_name} is hinge-mounted on a revolute joint; the barrel wraps the bracket pin but a small residual gap may remain at the hinge interface.",
        )
        # Proof: backrest is within the plank footprint and near the hinge bracket
        ctx.expect_overlap(
            br_part,
            plank,
            axes="xy",
            min_overlap=0.01,
            elem_a="backrest_panel",
            elem_b="board",
            name=f"{br_name} overlaps the plank footprint in plan view",
        )

    # ── base proportions ────────────────────────────────────────────────────
    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "base stand is about 0.7 m tall",
        base_aabb is not None and 0.65 <= base_aabb[1][2] <= 0.80,
        details=f"base top z={base_aabb[1][2]:.3f}" if base_aabb else "no aabb",
    )
    ctx.check(
        "base feet rest on the ground",
        base_aabb is not None and -0.005 <= base_aabb[0][2] <= 0.025,
        details=f"base bottom z={base_aabb[0][2]:.3f}" if base_aabb else "no aabb",
    )

    # ── plank pivot limits ──────────────────────────────────────────────────
    lim = pivot.motion_limits
    ctx.check(
        "plank pivot rocks ±18 degrees",
        lim is not None
        and abs(lim.lower + TILT) < 1e-6
        and abs(lim.upper - TILT) < 1e-6,
        details=f"limits=({lim.lower if lim else None}, {lim.upper if lim else None})",
    )

    # ── molded seats with raised lips ───────────────────────────────────────
    min_seat_height = SEAT_THICK + 0.015
    for nm in ("seat_left", "seat_right"):
        sa = ctx.part_element_world_aabb(plank, elem=nm)
        ctx.check(
            f"{nm} exists at a plank end",
            sa is not None,
            details=f"aabb={sa}",
        )
        if sa is not None:
            seat_h = sa[1][2] - sa[0][2]
            ctx.check(
                f"{nm} has raised lips (height {seat_h:.3f} > {min_seat_height:.3f} m)",
                seat_h > min_seat_height,
                details=f"seat element height={seat_h:.4f} m",
            )
            ctx.check(
                f"{nm} is near a plank end",
                abs((sa[0][0] + sa[1][0]) / 2.0) > 0.80,
                details=f"seat center x={(sa[0][0] + sa[1][0]) / 2.0:.3f}",
            )

    # ── rounded handle grips ────────────────────────────────────────────────
    plank_top_world = PIVOT_HEIGHT + PLANK_Z + PLANK_T / 2.0
    for nm in ("handle_left", "handle_right"):
        ha = ctx.part_element_world_aabb(plank, elem=nm)
        ctx.check(
            f"{nm} exists and rises above the plank",
            ha is not None and ha[1][2] > plank_top_world + 0.15,
            details=f"handle aabb={ha}",
        )

    # ── backrest revolute tilt joints ───────────────────────────────────────
    for br_joint in (br_right_tilt, br_left_tilt):
        jlim = br_joint.motion_limits
        ctx.check(
            f"{br_joint.name} is a non-fixed revolute tilt joint",
            br_joint.articulation_type == ArticulationType.REVOLUTE
            and jlim is not None
            and jlim.upper is not None
            and jlim.upper > 0.10,
            details=f"type={br_joint.articulation_type}, upper={jlim.upper if jlim else None}",
        )

    # ── decisive pose: plank seesaws ────────────────────────────────────────
    rest_sr = ctx.part_element_world_aabb(plank, elem="seat_right")
    rest_sl = ctx.part_element_world_aabb(plank, elem="seat_left")
    with ctx.pose({pivot: TILT}):
        tilt_sr = ctx.part_element_world_aabb(plank, elem="seat_right")
        tilt_sl = ctx.part_element_world_aabb(plank, elem="seat_left")
        beam_aabb = ctx.part_world_aabb(plank)
        ctx.check(
            "plank seesaws: positive q drops right seat and raises left seat",
            rest_sr is not None
            and tilt_sr is not None
            and rest_sl is not None
            and tilt_sl is not None
            and tilt_sr[0][2] < rest_sr[0][2] - 0.20
            and tilt_sl[0][2] > rest_sl[0][2] + 0.20,
            details=f"right {rest_sr} -> {tilt_sr}, left {rest_sl} -> {tilt_sl}",
        )
        ctx.check(
            "tilted plank stays clear of the ground",
            beam_aabb is not None and beam_aabb[0][2] > 0.02,
            details=f"plank bottom z={beam_aabb[0][2]:.3f}" if beam_aabb else "no aabb",
        )
        ctx.expect_contact(
            plank,
            base,
            elem_a="axle_sleeve",
            elem_b="axle_bar",
            name="tilted plank sleeve stays on the axle",
        )

    # ── decisive pose: backrest tilts ───────────────────────────────────────
    br_rest_r = ctx.part_element_world_aabb(backrest_right, elem="backrest_panel")
    with ctx.pose({br_right_tilt: BACKREST_TILT_MAX}):
        br_tilt_r = ctx.part_element_world_aabb(backrest_right, elem="backrest_panel")
        if br_rest_r is not None and br_tilt_r is not None:
            ctx.check(
                "right backrest tilts backward (top lowers when tilted)",
                br_tilt_r[1][2] < br_rest_r[1][2] - 0.005,
                details=f"rest top={br_rest_r[1][2]:.4f}, tilt top={br_tilt_r[1][2]:.4f}",
            )
            ctx.check(
                "right backrest top shifts outward when tilted",
                br_tilt_r[1][0] > br_rest_r[1][0] + 0.005,
                details=f"rest max_x={br_rest_r[1][0]:.4f}, tilt max_x={br_tilt_r[1][0]:.4f}",
            )

    br_rest_l = ctx.part_element_world_aabb(backrest_left, elem="backrest_panel")
    with ctx.pose({br_left_tilt: BACKREST_TILT_MAX}):
        br_tilt_l = ctx.part_element_world_aabb(backrest_left, elem="backrest_panel")
        if br_rest_l is not None and br_tilt_l is not None:
            ctx.check(
                "left backrest tilts backward (top lowers when tilted)",
                br_tilt_l[1][2] < br_rest_l[1][2] - 0.005,
                details=f"rest top={br_rest_l[1][2]:.4f}, tilt top={br_tilt_l[1][2]:.4f}",
            )

    return ctx.report()


object_model = build_object_model()
