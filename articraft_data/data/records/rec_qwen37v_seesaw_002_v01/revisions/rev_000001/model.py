from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoxGeometry,
    CylinderGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ----------------------------------------------------------------------------
# Classic two-seat plank playground seesaw.
#
# World frame: Z up, plank runs along X, pivot axis along Y.
#
# Support stand: four splayed round tube legs converging toward two top
#   attachment points (y = ±LEG_TOP_Y) at PIVOT_Z. A horizontal axle tube
#   (the bracket) spans between those points. Flat foot pads at each leg base.
#   Two cross braces tie the front/back legs together at mid-height.
#   Visible axle caps (silver discs) sit on the outside of each bracket end.
#
# Plank: a long rectangular board (~2.4 m) with a central axle sleeve that
#   wraps the bracket axle. At each end: a molded seat (pan + raised lips on
#   three sides) and a T-shaped handlebar just inboard of the seat.
#
# Articulation: plank_pivot, REVOLUTE, horizontal axis along Y (perpendicular
#   to the plank), +/- 18 degrees. Positive q tilts +X end down, -X end up.
# ----------------------------------------------------------------------------

PLANK_LEN = 2.40
PLANK_W = 0.18
PLANK_T = 0.035

PIVOT_Z = 0.55

LEG_R = 0.022
BRACKET_R = 0.018
HANDLE_R = 0.012
SLEEVE_R = 0.024
SLEEVE_LEN = 0.14

LEG_GX = 0.10        # fore-aft ground splay of legs
LEG_GY = 0.38        # lateral ground splay of legs
LEG_TOP_Y = 0.14     # lateral offset of leg tops (= bracket half-length)
FOOT_R = 0.040
FOOT_T = 0.006

CAP_R = 0.030
CAP_T = 0.010

SEAT_X = 1.00        # seat center distance from plank midpoint
SEAT_PAN = (0.26, 0.28, 0.008)
LIP_H = 0.035
LIP_T = 0.008

HANDLE_X = 0.66
HANDLE_POST_H = 0.28
HANDLE_BAR_W = 0.24

TILT = math.radians(18.0)

# Materials
GREEN_PAINT = Material("green_paint", rgba=(0.24, 0.56, 0.28, 1.0))
RED_PLANK = Material("red_plank_paint", rgba=(0.78, 0.22, 0.18, 1.0))
YELLOW_SEAT = Material("yellow_seat", rgba=(0.92, 0.78, 0.15, 1.0))
DARK_METAL = Material("dark_gray_metal", rgba=(0.25, 0.25, 0.27, 1.0))
SILVER_CAP = Material("silver_cap", rgba=(0.72, 0.74, 0.76, 1.0))


# ---- geometry helpers -------------------------------------------------------

def _tube_between(
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    radius: float,
    *,
    radial_segments: int = 16,
) -> CylinderGeometry:
    """Capped cylinder between two 3-D points."""
    dx, dy, dz = p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]
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


def _leg_top(sy: float) -> tuple[float, float, float]:
    """Top endpoint of a leg group (sy = ±1 for left/right)."""
    return (0.0, sy * LEG_TOP_Y, PIVOT_Z)


def _leg_ground(sx: float, sy: float) -> tuple[float, float, float]:
    """Ground endpoint of a leg (extends to z=0 so the tube merges with the foot pad)."""
    return (sx * LEG_GX, sy * LEG_GY, 0.0)


def _leg_point(ground: tuple, top: tuple, t: float) -> tuple[float, float, float]:
    """Linearly interpolate along a leg at parameter t ∈ [0, 1]."""
    return (
        ground[0] + (top[0] - ground[0]) * t,
        ground[1] + (top[1] - ground[1]) * t,
        ground[2] + (top[2] - ground[2]) * t,
    )


def _foot_mesh(x: float, y: float):
    """Flat circular foot pad at ground level."""
    disc = CylinderGeometry(FOOT_R, FOOT_T, radial_segments=20)
    disc.translate(x, y, FOOT_T / 2.0)
    return disc


def _axle_cap_mesh(y_pos: float):
    """Disc-shaped axle cap at a bracket end, oriented along Y."""
    cap = CylinderGeometry(CAP_R, CAP_T, radial_segments=20)
    cap.rotate_x(math.pi / 2.0)
    cap.translate(0.0, y_pos, PIVOT_Z)
    return cap


def _seat_mesh(outer_x_sign: float):
    """Molded seat pan with raised lips on two sides and the outer (back) edge."""
    pan = BoxGeometry(SEAT_PAN)
    half_w = SEAT_PAN[1] / 2.0  # 0.14
    half_l = SEAT_PAN[0] / 2.0  # 0.13
    lip_z_base = SEAT_PAN[2] / 2.0  # top of pan

    # Side lips (±Y): run along X direction
    for sy in (-1.0, 1.0):
        lip = BoxGeometry((SEAT_PAN[0] - 2 * LIP_T, LIP_T, LIP_H))
        lip.translate(0.0, sy * (half_w - LIP_T / 2.0), lip_z_base + LIP_H / 2.0)
        pan.merge(lip)

    # Back lip (outer X side): runs along Y direction
    back_lip = BoxGeometry((LIP_T, SEAT_PAN[1] - 2 * LIP_T, LIP_H))
    back_lip.translate(
        outer_x_sign * (half_l - LIP_T / 2.0),
        0.0,
        lip_z_base + LIP_H / 2.0,
    )
    pan.merge(back_lip)
    return pan


def _handlebar_mesh():
    """T-shaped handlebar: vertical post + horizontal crossbar."""
    post = CylinderGeometry(HANDLE_R, HANDLE_POST_H, radial_segments=14)
    post.translate(0.0, 0.0, HANDLE_POST_H / 2.0)
    bar = CylinderGeometry(HANDLE_R, HANDLE_BAR_W, radial_segments=14)
    bar.rotate_x(math.pi / 2.0)
    bar.translate(0.0, 0.0, HANDLE_POST_H)
    post.merge(bar)
    return post


# ---- assembly ---------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_seat_plank_seesaw")

    # --- support stand (static root) ----------------------------------------
    stand = model.part("support_stand")

    leg_defs = [
        (-1.0, -1.0),  # front-left
        (-1.0, +1.0),  # front-right
        (+1.0, -1.0),  # back-left
        (+1.0, +1.0),  # back-right
    ]
    for i, (sx, sy) in enumerate(leg_defs):
        g = _leg_ground(sx, sy)
        t = _leg_top(sy)
        leg = _tube_between(g, t, LEG_R)
        # Merge foot pad into the leg mesh so each leg+foot is one connected piece
        foot = _foot_mesh(g[0], g[1])
        leg.merge(foot)
        stand.visual(
            mesh_from_geometry(leg, f"leg_{i}"),
            material=GREEN_PAINT,
            name=f"leg_{i}",
        )

    # Cross braces between front and back legs on each side
    brace_t = 0.42
    for si, sy in enumerate((-1.0, 1.0)):
        g_fl = _leg_ground(-1.0, sy)
        t_l = _leg_top(sy)
        g_bl = _leg_ground(+1.0, sy)
        t_r = _leg_top(sy)  # same top for both legs on same side
        p_front = _leg_point(g_fl, t_l, brace_t)
        p_back = _leg_point(g_bl, t_r, brace_t)
        brace = _tube_between(p_front, p_back, 0.014)
        stand.visual(
            mesh_from_geometry(brace, f"cross_brace_{si}"),
            material=GREEN_PAINT,
            name=f"cross_brace_{si}",
        )

    # Top bracket / axle tube spanning between the two leg-top groups
    bracket = CylinderGeometry(BRACKET_R, 2.0 * LEG_TOP_Y, radial_segments=18)
    bracket.rotate_x(math.pi / 2.0)
    bracket.translate(0.0, 0.0, PIVOT_Z)
    stand.visual(
        mesh_from_geometry(bracket, "top_bracket"),
        material=GREEN_PAINT,
        name="top_bracket",
    )

    # Axle caps: visible silver discs on the outside of each bracket end
    for ci, yp in enumerate([-LEG_TOP_Y - CAP_T / 2.0, LEG_TOP_Y + CAP_T / 2.0]):
        cap = _axle_cap_mesh(yp)
        stand.visual(
            mesh_from_geometry(cap, f"axle_cap_{ci}"),
            material=SILVER_CAP,
            name=f"axle_cap_{ci}",
        )

    # --- plank (rocking child) ----------------------------------------------
    plank = model.part("plank")

    # Main plank board centered at the part origin
    board = BoxGeometry((PLANK_LEN, PLANK_W, PLANK_T))
    plank.visual(
        mesh_from_geometry(board, "plank_board"),
        material=RED_PLANK,
        name="plank_board",
    )

    # Central axle sleeve (wraps the bracket axle)
    sleeve = CylinderGeometry(SLEEVE_R, SLEEVE_LEN, radial_segments=18)
    sleeve.rotate_x(math.pi / 2.0)
    plank.visual(
        mesh_from_geometry(sleeve, "axle_sleeve"),
        material=DARK_METAL,
        name="axle_sleeve",
    )

    # Molded seats at each end, with back lip facing outward
    for i, (sx, sign) in enumerate([(SEAT_X, 1.0), (-SEAT_X, -1.0)]):
        seat = _seat_mesh(sign)
        seat.translate(sx, 0.0, PLANK_T / 2.0 + 0.001)
        plank.visual(
            mesh_from_geometry(seat, f"seat_{i}"),
            material=YELLOW_SEAT,
            name=f"seat_{i}",
        )

    # T-handlebars just inboard of the seats
    for i, hx in enumerate([HANDLE_X, -HANDLE_X]):
        hb = _handlebar_mesh()
        hb.translate(hx, 0.0, PLANK_T / 2.0)
        plank.visual(
            mesh_from_geometry(hb, f"handlebar_{i}"),
            material=DARK_METAL,
            name=f"handlebar_{i}",
        )

    # --- articulation -------------------------------------------------------
    model.articulation(
        "plank_pivot",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=plank,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=100.0, velocity=2.0, lower=-TILT, upper=TILT
        ),
    )

    return model


# ---- tests ------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("support_stand")
    plank = object_model.get_part("plank")
    pivot = object_model.get_articulation("plank_pivot")

    # --- intentional overlaps -----------------------------------------------
    # Axle sleeve wraps the bracket axle (captured-pin fit)
    ctx.allow_overlap(
        plank, stand,
        elem_a="axle_sleeve", elem_b="top_bracket",
        reason="Plank axle sleeve intentionally wraps the stand bracket axle at the pivot.",
    )
    # Plank board has the bracket axle passing through its center
    ctx.allow_overlap(
        plank, stand,
        elem_a="plank_board", elem_b="top_bracket",
        reason="Bracket axle passes through the center of the plank board where the sleeve sits.",
    )

    ctx.expect_contact(
        plank, stand,
        elem_a="axle_sleeve", elem_b="top_bracket",
        name="plank sleeve rides on the bracket axle",
    )

    # --- support stand proportions ------------------------------------------
    stand_aabb = ctx.part_world_aabb(stand)
    ctx.check(
        "support stand is about 0.55 m tall",
        stand_aabb is not None and 0.50 <= stand_aabb[1][2] <= 0.65,
        details=f"stand aabb={stand_aabb}",
    )
    ctx.check(
        "stand feet rest on the ground",
        stand_aabb is not None and -0.01 <= stand_aabb[0][2] <= 0.02,
        details=f"stand aabb={stand_aabb}",
    )

    # --- axle caps visible at bracket ends ----------------------------------
    for ci in range(2):
        cap_aabb = ctx.part_element_world_aabb(stand, elem=f"axle_cap_{ci}")
        ctx.check(
            f"axle_cap_{ci} is present at the bracket end",
            cap_aabb is not None,
            details=f"cap aabb={cap_aabb}",
        )
        if cap_aabb is not None:
            # Caps should be at pivot height and outside the plank width
            cap_cz = (cap_aabb[0][2] + cap_aabb[1][2]) / 2.0
            ctx.check(
                f"axle_cap_{ci} sits at pivot height",
                abs(cap_cz - PIVOT_Z) < 0.02,
                details=f"cap center z={cap_cz:.4f}",
            )

    # --- molded seats with raised lips --------------------------------------
    for i in range(2):
        seat_aabb = ctx.part_element_world_aabb(plank, elem=f"seat_{i}")
        board_aabb = ctx.part_element_world_aabb(plank, elem="plank_board")
        ctx.check(
            f"seat_{i} exists near plank end",
            seat_aabb is not None
            and abs((seat_aabb[0][0] + seat_aabb[1][0]) / 2.0) > 0.80,
            details=f"seat aabb={seat_aabb}",
        )
        ctx.check(
            f"seat_{i} has raised lips protruding above the plank surface",
            seat_aabb is not None and board_aabb is not None
            and seat_aabb[1][2] > board_aabb[1][2] + 0.020,
            details=(
                f"seat top z={seat_aabb[1][2] if seat_aabb else None}, "
                f"board top z={board_aabb[1][2] if board_aabb else None}"
            ),
        )

    # --- handlebars exist just inboard of seats -----------------------------
    for i in range(2):
        hb_aabb = ctx.part_element_world_aabb(plank, elem=f"handlebar_{i}")
        seat_aabb = ctx.part_element_world_aabb(plank, elem=f"seat_{i}")
        ctx.check(
            f"handlebar_{i} is present and stands taller than the seat",
            hb_aabb is not None and seat_aabb is not None
            and hb_aabb[1][2] > seat_aabb[1][2] + 0.10,
            details=f"handle aabb={hb_aabb}, seat aabb={seat_aabb}",
        )

    # --- pivot limits -------------------------------------------------------
    lim = pivot.motion_limits
    ctx.check(
        "plank pivot rocks +/- 18 degrees",
        lim is not None
        and lim.lower is not None and lim.upper is not None
        and abs(lim.lower + TILT) < 1e-6
        and abs(lim.upper - TILT) < 1e-6,
        details=f"limits=({lim.lower if lim else None}, {lim.upper if lim else None})",
    )

    # --- decisive pose: plank seesaws correctly -----------------------------
    rest_s0 = ctx.part_element_world_aabb(plank, elem="seat_0")
    rest_s1 = ctx.part_element_world_aabb(plank, elem="seat_1")

    with ctx.pose({pivot: TILT}):
        tilt_s0 = ctx.part_element_world_aabb(plank, elem="seat_0")
        tilt_s1 = ctx.part_element_world_aabb(plank, elem="seat_1")
        plank_aabb = ctx.part_world_aabb(plank)

        ctx.check(
            "plank seesaws: +X end drops, -X end rises at positive q",
            rest_s0 is not None and tilt_s0 is not None
            and rest_s1 is not None and tilt_s1 is not None
            and tilt_s0[1][2] < rest_s0[1][2] - 0.15
            and tilt_s1[1][2] > rest_s1[1][2] + 0.15,
            details=f"seat0 {rest_s0} -> {tilt_s0}, seat1 {rest_s1} -> {tilt_s1}",
        )
        ctx.check(
            "tilted plank stays above ground",
            plank_aabb is not None and plank_aabb[0][2] > 0.01,
            details=f"plank aabb={plank_aabb}",
        )
        ctx.expect_contact(
            plank, stand,
            elem_a="axle_sleeve", elem_b="top_bracket",
            name="tilted plank sleeve stays on axle",
        )

    return ctx.report()


object_model = build_object_model()
