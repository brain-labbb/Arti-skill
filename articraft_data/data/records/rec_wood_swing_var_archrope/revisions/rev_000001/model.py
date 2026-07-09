from __future__ import annotations

# Arched-beam garden swing bench variant.
#
# Frame convention:
#   X = fore/aft depth (bench swings in X)
#   Y = left/right width (hanger bar runs along Y)
#   Z = up
#
# Root part = fixed support frame: two vertical cylindrical end posts on
# cross-feet, diagonal rod braces, an arched overhead beam swept along a
# smooth spline from post top to post top, two steel straps dropping from
# the arch to a horizontal hanger bar.
# Child part = one rigid slatted bench pendulum (seat, reclined backrest,
# armrest ends, four thin steel hanger rods) on a single revolute joint
# whose axis is horizontal (Y) at the hanger bar.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)

# --- Frame dimensions (meters) ------------------------------------------------
POST_Y = 0.95           # end-post center planes at y = +/- POST_Y
POST_HEIGHT = 1.70      # post top height
POST_R = 0.045          # post radius (cylindrical pine)

FOOT_HALF_X = 0.62      # foot half-length in X (fore/aft stability)
FOOT_W = 0.09           # foot width in Y
FOOT_H = 0.05           # foot thickness

ARCH_RISE = 0.15        # arch rise above post tops
ARCH_APEX_Z = POST_HEIGHT + ARCH_RISE   # 1.85 m at center
BEAM_W = 0.09           # arch beam cross-section width (X)
BEAM_H = 0.10           # arch beam cross-section height (Z when level)

PIVOT_Z = 1.47          # swing pivot height (hanger bar center)
SWING_LIMIT = 0.6       # rad fore/aft
ROD_Y = 0.70            # hanger rod Y planes

# Diagonal brace parameters
BRACE_FOOT_X = 0.50     # brace foot anchor in X
BRACE_TOP_Z = 1.05      # brace top connection on post

# --- Bench dimensions (identical to parent, in bench pivot frame) -------------
SEAT_TOP = -0.93        # seat slat top surface (world ~0.54)
SEAT_HALF_X = 0.275     # seat outer half depth
SEAT_W = 1.50           # bench width along Y
BACK_RECLINE = math.radians(10.0)
BACK_LEN = 0.46         # backrest stile length


# ---------------------------------------------------------------------------
# Shared geometry helpers
# ---------------------------------------------------------------------------

def _arch_centerline():
    """Return the arch beam centerline as spline points in the YZ plane."""
    mid_y = POST_Y * 0.5
    mid_z = POST_HEIGHT + ARCH_RISE * 0.75  # parabolic quarter-point
    return [
        (0.0, -POST_Y, POST_HEIGHT),
        (0.0, -mid_y, mid_z),
        (0.0, 0.0, ARCH_APEX_Z),
        (0.0, mid_y, mid_z),
        (0.0, POST_Y, POST_HEIGHT),
    ]


def _build_arch_beam():
    """Build the arched overhead beam mesh via swept rectangular profile."""
    path = _arch_centerline()
    profile = rounded_rect_profile(BEAM_W, BEAM_H, radius=0.012)
    return sweep_profile_along_spline(
        path,
        profile=profile,
        samples_per_segment=18,
        cap_profile=True,
        up_hint=(0.0, 0.0, 1.0),
    )


def _arch_z_at(y: float) -> float:
    """Approximate arch centerline height at a given Y (parabolic fit)."""
    return POST_HEIGHT + ARCH_RISE * (1.0 - (y / POST_Y) ** 2)


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="arched_garden_swing_bench")
    model.material("pine", rgba=(0.84, 0.72, 0.52, 1.0))
    model.material("pine_dark", rgba=(0.73, 0.60, 0.42, 1.0))
    model.material("steel", rgba=(0.46, 0.47, 0.50, 1.0))

    frame = model.part("swing_frame")

    # ---- Two vertical cylindrical end posts ---------------------------------
    for i in range(2):
        y = POST_Y if i == 0 else -POST_Y
        frame.visual(
            Cylinder(radius=POST_R, length=POST_HEIGHT),
            origin=Origin(xyz=(0.0, y, POST_HEIGHT / 2.0)),
            material="pine",
            name=f"post_{i}",
        )

    # ---- Base feet (one per post, extending fore/aft for stability) ---------
    for i in range(2):
        y = POST_Y if i == 0 else -POST_Y
        frame.visual(
            Box((FOOT_HALF_X * 2.0, FOOT_W, FOOT_H)),
            origin=Origin(xyz=(0.0, y, FOOT_H / 2.0)),
            material="pine_dark",
            name=f"foot_{i}",
        )

    # ---- Diagonal rod braces (two per post, fore and aft) -------------------
    brace_dx = BRACE_FOOT_X
    brace_dz = BRACE_TOP_Z - FOOT_H
    brace_len = math.hypot(brace_dx, brace_dz)
    brace_tilt = math.atan2(brace_dx, brace_dz)
    brace_cx = BRACE_FOOT_X / 2.0
    brace_cz = (FOOT_H + BRACE_TOP_Z) / 2.0

    for i in range(2):
        y = POST_Y if i == 0 else -POST_Y
        for j, sgn in enumerate((-1.0, 1.0)):
            frame.visual(
                Cylinder(radius=0.016, length=brace_len),
                origin=Origin(
                    xyz=(sgn * brace_cx, y, brace_cz),
                    rpy=(0.0, sgn * brace_tilt, 0.0),
                ),
                material="steel",
                name=f"brace_{i}_{j}",
            )

    # ---- Arched overhead beam -----------------------------------------------
    arch_mesh = _build_arch_beam()
    frame.visual(
        mesh_from_geometry(arch_mesh, "arch_beam"),
        name="arch_beam",
        material="pine_dark",
    )

    # ---- Hanger straps (drop from arch underside to hanger bar) -------------
    arch_z_rod = _arch_z_at(ROD_Y)
    strap_top = arch_z_rod - BEAM_H / 2.0      # arch underside at y=ROD_Y
    strap_bot = PIVOT_Z - 0.016                 # extend below bar center
    strap_len = strap_top - strap_bot
    strap_cz = (strap_top + strap_bot) / 2.0

    for i in range(2):
        y = ROD_Y if i == 0 else -ROD_Y
        frame.visual(
            Box((0.035, 0.03, strap_len)),
            origin=Origin(xyz=(0.0, y, strap_cz)),
            material="steel",
            name=f"hanger_strap_{i}",
        )

    # ---- Hanger bar (horizontal pivot axle, parallel to Y) ------------------
    frame.visual(
        Cylinder(radius=0.014, length=1.50),
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="steel",
        name="hanger_bar",
    )

    # ============================================================ bench swing
    bench = model.part("bench_swing")

    # Seat frame rails (front and rear, along Y).
    rail_z = -0.978
    for tag, rx in (
        ("front", SEAT_HALF_X - 0.0275),
        ("rear", -(SEAT_HALF_X - 0.0275)),
    ):
        bench.visual(
            Box((0.055, SEAT_W, 0.055)),
            origin=Origin(xyz=(rx, 0.0, rail_z)),
            material="pine_dark",
            name=f"seat_{tag}_rail",
        )
    for s, ry in enumerate(
        (-(SEAT_W / 2.0 - 0.0275), SEAT_W / 2.0 - 0.0275)
    ):
        bench.visual(
            Box((2.0 * SEAT_HALF_X, 0.055, 0.055)),
            origin=Origin(xyz=(0.0, ry, rail_z)),
            material="pine_dark",
            name=f"seat_end_rail_{s}",
        )

    # Flat seat slats (along Y), spaced with gaps in X.
    n_seat = 6
    for i in range(n_seat):
        sx = -0.225 + i * (0.45 / (n_seat - 1))
        bench.visual(
            Box((0.071, 1.46, 0.028)),
            origin=Origin(xyz=(sx, 0.0, SEAT_TOP - 0.014)),
            material="pine",
            name=f"seat_slat_{i}",
        )

    # Reclined backrest: two tilted stiles + horizontal slats on their face.
    bx, bz = -0.2475, -1.0
    dirx, dirz = -math.sin(BACK_RECLINE), math.cos(BACK_RECLINE)
    for s, sy in enumerate((-0.71, 0.71)):
        bench.visual(
            Box((0.05, 0.05, BACK_LEN)),
            origin=Origin(
                xyz=(
                    bx + dirx * BACK_LEN / 2.0,
                    sy,
                    bz + dirz * BACK_LEN / 2.0,
                ),
                rpy=(0.0, -BACK_RECLINE, 0.0),
            ),
            material="pine_dark",
            name=f"back_stile_{s}",
        )
    fx, fz = math.cos(BACK_RECLINE), math.sin(BACK_RECLINE)
    for i, t in enumerate((0.13, 0.225, 0.32, 0.415)):
        bench.visual(
            Box((0.025, 1.42, 0.07)),
            origin=Origin(
                xyz=(
                    bx + dirx * t + 0.0315 * fx,
                    0.0,
                    bz + dirz * t + 0.0315 * fz,
                ),
                rpy=(0.0, -BACK_RECLINE, 0.0),
            ),
            material="pine",
            name=f"back_slat_{i}",
        )

    # Armrest ends: front arm post + armrest board back to the stile.
    for s, ay in enumerate((-0.71, 0.71)):
        bench.visual(
            Box((0.05, 0.05, 0.26)),
            origin=Origin(xyz=(0.225, ay, -0.83)),
            material="pine_dark",
            name=f"arm_post_{s}",
        )
        bench.visual(
            Box((0.58, 0.06, 0.035)),
            origin=Origin(xyz=(-0.04, ay, -0.6875)),
            material="pine",
            name=f"armrest_{s}",
        )

    # Four thin steel hanger rods: a fore/aft V pair at each bench end.
    rod_specs = []
    for s, ry in enumerate((-ROD_Y, ROD_Y)):
        rod_specs.append((f"hanger_rod_front_{s}", ry, 0.21))
        rod_specs.append((f"hanger_rod_rear_{s}", ry, -0.29))
    for rname, ry, bx_rod in rod_specs:
        top = (0.0, 0.0)
        bot = (bx_rod, -0.695)
        dx, dz = bot[0] - top[0], bot[1] - top[1]
        length = math.hypot(dx, dz) + 0.03
        bench.visual(
            Cylinder(radius=0.011, length=length),
            origin=Origin(
                xyz=(
                    (top[0] + bot[0]) / 2.0,
                    ry,
                    (top[1] + bot[1]) / 2.0,
                ),
                rpy=(0.0, math.atan2(dx, dz), 0.0),
            ),
            material="steel",
            name=rname,
        )

    # ----------------------------------------------------------- articulation
    model.articulation(
        "bench_pivot",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=bench,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=150.0,
            velocity=2.5,
            lower=-SWING_LIMIT,
            upper=SWING_LIMIT,
        ),
    )

    return model


object_model = build_object_model()


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("swing_frame")
    bench = object_model.get_part("bench_swing")
    pivot = object_model.get_articulation("bench_pivot")

    def count(part, prefix: str) -> int:
        return sum(1 for v in part.visuals if v.name and v.name.startswith(prefix))

    def has(part, name: str) -> bool:
        return part.get_visual(name) is not None

    # --- Intentional overlaps ------------------------------------------------

    # Arch beam sits on top of the posts (small embed at the junction).
    for i in range(2):
        ctx.allow_overlap(
            frame, frame,
            elem_a="arch_beam",
            elem_b=f"post_{i}",
            reason="Arch beam is seated onto the post top (structural bearing).",
        )

    # Hanger straps embed into the arch beam underside and wrap the bar.
    for i in range(2):
        ctx.allow_overlap(
            frame, frame,
            elem_a="arch_beam",
            elem_b=f"hanger_strap_{i}",
            reason="Strap top is seated into the arch beam underside.",
        )
        ctx.allow_overlap(
            frame, frame,
            elem_a="hanger_bar",
            elem_b=f"hanger_strap_{i}",
            reason="Strap wraps around the hanger bar (captured shaft).",
        )

    # Hanger rod tops seat onto the hanger bar and pass inside the straps.
    # Rod s=0 is at ry=-ROD_Y, strap 1 is at y=-ROD_Y (and vice versa).
    for s in (0, 1):
        strap_idx = 1 - s  # rod s=0 → strap 1, rod s=1 → strap 0
        for rod in (f"hanger_rod_front_{s}", f"hanger_rod_rear_{s}"):
            ctx.allow_overlap(
                frame, bench,
                elem_a="hanger_bar",
                elem_b=rod,
                reason="Rod top is seated onto the pivot hanger bar (captured shaft).",
            )
            ctx.allow_overlap(
                frame, bench,
                elem_a=f"hanger_strap_{strap_idx}",
                elem_b=rod,
                reason="Rod top end passes inside the strap that carries the bar.",
            )

    ctx.expect_contact(
        frame, bench,
        elem_a="hanger_bar",
        elem_b="hanger_rod_front_0",
        name="hanger rods are seated on the pivot bar",
    )

    # --- Frame structure: arched beam on two vertical posts ------------------

    ctx.check("two vertical end posts", count(frame, "post_") == 2)
    ctx.check("arch beam present", has(frame, "arch_beam"))

    # Posts are vertical cylinders (Y-span ≈ 2*radius, not splayed).
    for i in range(2):
        post_bb = ctx.part_element_world_aabb(frame, elem=f"post_{i}")
        if post_bb is not None:
            y_span = post_bb[1][1] - post_bb[0][1]
            ctx.check(
                f"post_{i} is a vertical cylinder",
                y_span < 0.12,
                details=f"y_span={y_span:.3f}",
            )

    # Arch beam rises above the post tops (proves it is curved, not flat).
    arch_bb = ctx.part_element_world_aabb(frame, elem="arch_beam")
    if arch_bb is not None:
        ctx.check(
            "arch beam is curved (apex above post tops)",
            arch_bb[1][2] > POST_HEIGHT + 0.08,
            details=f"arch_top={arch_bb[1][2]:.3f}",
        )
        ctx.check(
            "arch spans between the posts",
            arch_bb[0][1] < -POST_Y + 0.05 and arch_bb[1][1] > POST_Y - 0.05,
            details=f"y_min={arch_bb[0][1]:.3f} y_max={arch_bb[1][1]:.3f}",
        )

    # No roof, canopy, or gable elements.
    n_roof = sum(
        1 for v in frame.visuals
        if v.name and any(k in v.name for k in ("roof", "gable", "ridge", "purlin", "wall_"))
    )
    ctx.check(
        "no roof, canopy, or gable",
        n_roof == 0,
        details=f"roof_elements={n_roof}",
    )

    # Base feet and braces provide structural support.
    ctx.check("two base feet", count(frame, "foot_") == 2)
    ctx.check("four diagonal braces", count(frame, "brace_") == 4)
    ctx.check("no wall slats or lattice", count(frame, "wall_") == 0)

    # Frame rests on the ground and has correct overall scale.
    fb = ctx.part_world_aabb(frame)
    if fb is not None:
        (xmn, ymn, zmn), (xmx, ymx, zmx) = fb
        ctx.check(
            "frame rests on the ground",
            abs(zmn) < 0.01,
            details=f"zmin={zmn:.3f}",
        )
        ctx.check(
            "frame is ~2.0 m wide",
            1.85 < (ymx - ymn) < 2.10,
            details=f"w={ymx - ymn:.2f}",
        )
        ctx.check(
            "frame is ~1.9 m tall",
            1.80 < zmx < 2.00,
            details=f"h={zmx:.2f}",
        )
        ctx.check(
            "frame depth from feet",
            1.10 < (xmx - xmn) < 1.50,
            details=f"d={xmx - xmn:.2f}",
        )

    # --- Bench swing (identical to parent) -----------------------------------

    n_seat = count(bench, "seat_slat_")
    ctx.check("seat is slatted (6 slats)", n_seat == 6, details=f"{n_seat}")
    n_back = count(bench, "back_slat_")
    ctx.check("backrest is slatted (4 slats)", n_back == 4, details=f"{n_back}")
    ctx.check("two armrest ends", count(bench, "armrest_") == 2)
    ctx.check("four thin hanger rods", count(bench, "hanger_rod_") == 4)

    bb = ctx.part_world_aabb(bench)
    if bb is not None:
        (bxmn, bymn, bzmn), (bxmx, bymx, bzmx) = bb
        ctx.check(
            "bench is ~1.5 m wide",
            1.45 < (bymx - bymn) < 1.55,
            details=f"w={bymx - bymn:.2f}",
        )
        ctx.check(
            "bench clears the posts sideways",
            bymx < POST_Y - 0.02 and bymn > -(POST_Y - 0.02),
            details=f"ymax={bymx:.3f}",
        )
        ctx.check(
            "bench hangs above the floor",
            bzmn > 0.30,
            details=f"zmin={bzmn:.2f}",
        )

    seat = ctx.part_element_world_aabb(bench, elem="seat_slat_0")
    if seat is not None:
        ctx.check(
            "seat at sitting height (~0.55 m)",
            0.48 < seat[1][2] < 0.62,
            details=f"seat_top={seat[1][2]:.2f}",
        )

    # Reclined backrest leans backwards.
    stile = ctx.part_element_world_aabb(bench, elem="back_stile_0")
    if stile is not None and seat is not None:
        ctx.check(
            "backrest is reclined behind the seat",
            stile[0][0] < -0.30 and stile[1][2] > seat[1][2] + 0.30,
            details=f"stile_xmin={stile[0][0]:.2f} stile_top={stile[1][2]:.2f}",
        )

    # --- Pendulum swing motion -----------------------------------------------

    def _cx(aabb):
        return None if aabb is None else (aabb[0][0] + aabb[1][0]) / 2.0

    with ctx.pose({pivot: -SWING_LIMIT}):
        fwd_bb = ctx.part_world_aabb(bench)
        if fwd_bb is not None:
            ctx.check(
                "bench clears the floor swung forward",
                fwd_bb[0][2] > 0.05,
                details=f"zmin={fwd_bb[0][2]:.3f}",
            )

    with ctx.pose({pivot: SWING_LIMIT}):
        bwd_bb = ctx.part_world_aabb(bench)
        if bwd_bb is not None:
            ctx.check(
                "bench clears the floor swung backward",
                bwd_bb[0][2] > 0.05,
                details=f"zmin={bwd_bb[0][2]:.3f}",
            )

    fwd, bwd = _cx(fwd_bb), _cx(bwd_bb)
    if fwd is not None and bwd is not None:
        ctx.check(
            "bench swings fore/aft about the Y axis",
            (fwd - bwd) > 0.4,
            details=f"fwd_cx={fwd:.3f} bwd_cx={bwd:.3f}",
        )

    return ctx.report()
