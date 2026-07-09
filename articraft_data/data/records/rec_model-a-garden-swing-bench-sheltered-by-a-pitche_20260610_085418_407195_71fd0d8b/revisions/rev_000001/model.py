from __future__ import annotations

# Garden swing bench sheltered by a pitched roof.
#
# Frame convention:
#   X = fore/aft depth (the bench swings in X, roof slopes drop in +/-X)
#   Y = left/right width (ridge beam runs along Y between the two end walls)
#   Z = up
#
# Root part = fixed shelter: two A-frame end walls filled with vertical slats
# plus a small slatted triangular gable each, joined by a ridge beam and roof
# purlins, carrying a sage-green corrugated roof (ribs run DOWN the slope).
# Child part = one rigid slatted bench pendulum (seat, reclined backrest,
# armrest ends, four thin steel hanger rods) on a single revolute joint whose
# axis is horizontal and parallel to the ridge, at the upper anchor bar.

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
)

# --- Shelter frame dimensions (meters) ----------------------------------------
WALL_Y = 0.86          # end-wall center planes at y = +/-WALL_Y
POST_BASE_X = 0.60     # wall post foot position in X
POST_TOP_X = 0.465     # wall post position at the top rail (slight A splay)
POST_TOP_Z = 1.30      # wall post top / gable base height
POST_W = 0.07          # post cross-section

ROOF_ZC = 1.83         # roof sheet center plane height at the apex
ROOF_T = 0.022         # roof sheet thickness
ROOF_PITCH = math.radians(45.0)
EAVE_X = 0.70          # horizontal half-run of each roof slope (overhang incl.)
# Vertical offset from sheet center plane to its underside.
_UO = (ROOF_T / 2.0) / math.cos(ROOF_PITCH)
ROOF_UNDER_Z0 = ROOF_ZC - _UO  # roof underside plane: z = ROOF_UNDER_Z0 - |x|

PIVOT_Z = 1.47         # swing pivot height (hanger bar under the ridge)
SWING_LIMIT = 0.6      # rad, fore/aft

# --- Bench dimensions (in the bench pivot frame: origin at the pivot) ---------
SEAT_TOP = -0.93       # seat slat top (world ~0.54)
SEAT_HALF_X = 0.275    # seat outer half depth
SEAT_W = 1.50          # bench width along Y
ROD_Y = 0.70           # hanger rod plane
BACK_RECLINE = math.radians(10.0)
BACK_LEN = 0.46        # backrest stile length


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sheltered_garden_swing_bench")
    model.material("pine", rgba=(0.84, 0.72, 0.52, 1.0))
    model.material("pine_dark", rgba=(0.73, 0.60, 0.42, 1.0))
    model.material("roof_sage", rgba=(0.47, 0.57, 0.44, 1.0))
    model.material("roof_sage_dark", rgba=(0.40, 0.50, 0.38, 1.0))
    model.material("steel", rgba=(0.46, 0.47, 0.50, 1.0))

    frame = model.part("shelter_frame")

    # ------------------------------------------------ A-frame slatted end walls
    post_tilt = math.atan2(POST_BASE_X - POST_TOP_X, POST_TOP_Z)  # from vertical
    post_len = 1.32
    # Lift so the lowest tilted-box corner rests exactly on the ground (z=0).
    post_cz = (post_len / 2.0) * math.cos(post_tilt) + (POST_W / 2.0) * math.sin(post_tilt)
    post_line_slope = (POST_BASE_X - POST_TOP_X) / POST_TOP_Z  # dx per dz (inward)
    post_cx = POST_BASE_X - post_line_slope * post_cz

    for w, wy in enumerate((-WALL_Y, WALL_Y)):
        # Two slightly splayed posts per wall (A-frame legs).
        for i, sgn in enumerate((-1.0, 1.0)):
            pitch = math.atan2(-sgn * (POST_BASE_X - POST_TOP_X), POST_TOP_Z)
            frame.visual(
                Box((POST_W, POST_W, post_len)),
                origin=Origin(xyz=(sgn * post_cx, wy, post_cz), rpy=(0.0, pitch, 0.0)),
                material="pine",
                name=f"wall_{w}_post_{i}",
            )
        # Bottom rail near the ground and top rail at the gable base.
        frame.visual(
            Box((1.24, 0.06, 0.07)),
            origin=Origin(xyz=(0.0, wy, 0.08)),
            material="pine_dark",
            name=f"wall_{w}_base_rail",
        )
        frame.visual(
            Box((1.00, 0.06, 0.07)),
            origin=Origin(xyz=(0.0, wy, 1.28)),
            material="pine_dark",
            name=f"wall_{w}_top_rail",
        )
        # Vertical wall slats: open lattice between the rails.
        n_slat = 7
        for k in range(n_slat):
            sx = -0.40 + k * (0.80 / (n_slat - 1))
            z0, z1 = 0.055, 1.30
            frame.visual(
                Box((0.06, 0.045, z1 - z0)),
                origin=Origin(xyz=(sx, wy, (z0 + z1) / 2.0)),
                material="pine",
                name=f"wall_{w}_slat_{k}",
            )
        # Small triangular slatted gable: vertical slats whose flat tops step
        # down following the roof slope (top trimmed to the slope at the
        # slat's outer edge, embedded ~8 mm into the sheet underside).
        for k in range(7):
            gx = -0.315 + k * 0.105
            top = min(ROOF_UNDER_Z0 - (abs(gx) + 0.03) + 0.008, 1.755)
            z0 = 1.255
            frame.visual(
                Box((0.06, 0.045, top - z0)),
                origin=Origin(xyz=(gx, wy, (z0 + top) / 2.0)),
                material="pine",
                name=f"wall_{w}_gable_slat_{k}",
            )

    # ------------------------------------------------------ ridge beam + hanger
    frame.visual(
        Box((0.10, 1.80, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 1.75)),
        material="pine_dark",
        name="ridge_beam",
    )
    # Two steel straps drop from the ridge beam to carry the hanger bar.
    for s, sy in enumerate((-ROD_Y, ROD_Y)):
        frame.visual(
            Box((0.035, 0.03, 0.27)),
            origin=Origin(xyz=(0.0, sy, 1.585)),
            material="steel",
            name=f"hanger_strap_{s}",
        )
    # Horizontal hanger bar (the swing pivot axle), parallel to the ridge.
    frame.visual(
        Cylinder(radius=0.014, length=1.46),
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="steel",
        name="hanger_bar",
    )

    # ------------------------------------------------------- corrugated roof
    panel_y = 2.00
    slope_len = EAVE_X / math.cos(ROOF_PITCH)  # ~0.99
    for side, tag in ((1.0, "front"), (-1.0, "rear")):
        p = side * ROOF_PITCH
        mid = EAVE_X / 2.0
        frame.visual(
            Box((slope_len + 0.012, panel_y, ROOF_T)),
            origin=Origin(xyz=(side * mid, 0.0, ROOF_ZC - mid), rpy=(0.0, p, 0.0)),
            material="roof_sage",
            name=f"roof_sheet_{tag}",
        )
        # Corrugation ribs RUN DOWN THE SLOPE (long axis ridge->eave),
        # repeated across the width with gaps, sitting proud of the sheet.
        nx, nz = math.sin(p), math.cos(p)  # outward sheet normal
        n_rib = 24
        for j in range(n_rib):
            ry = -panel_y / 2.0 + (j + 0.5) * (panel_y / n_rib)
            frame.visual(
                Box((slope_len - 0.02, 0.034, 0.024)),
                origin=Origin(
                    xyz=(side * mid + 0.018 * nx, ry, ROOF_ZC - mid + 0.018 * nz),
                    rpy=(0.0, p, 0.0),
                ),
                material="roof_sage_dark",
                name=f"roof_rib_{tag}_{j}",
            )
        # Two roof purlins per slope under the sheet, spanning wall to wall.
        for k, xp in enumerate((0.28, 0.55)):
            off = 0.028  # below sheet center plane along inward normal
            frame.visual(
                Box((0.07, 1.80, 0.05)),
                origin=Origin(
                    xyz=(side * (xp - off * nx * side) - 0.0, 0.0, ROOF_ZC - xp - off * nz),
                    rpy=(0.0, p, 0.0),
                ),
                material="pine_dark",
                name=f"roof_purlin_{tag}_{k}",
            )
    # Ridge cap over the apex joint of the two slopes.
    frame.visual(
        Box((0.16, panel_y, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 1.86)),
        material="roof_sage_dark",
        name="ridge_cap",
    )

    # ------------------------------------------------------------- bench swing
    # Bench frame origin = the pivot on the hanger bar; everything below is one
    # rigid pendulum body.
    bench = model.part("bench_swing")

    # Seat frame rails.
    rail_z = -0.978
    for tag, rx in (("front", SEAT_HALF_X - 0.0275), ("rear", -(SEAT_HALF_X - 0.0275))):
        bench.visual(
            Box((0.055, SEAT_W, 0.055)),
            origin=Origin(xyz=(rx, 0.0, rail_z)),
            material="pine_dark",
            name=f"seat_{tag}_rail",
        )
    for s, ry in enumerate((-(SEAT_W / 2.0 - 0.0275), SEAT_W / 2.0 - 0.0275)):
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
    bx, bz = -0.2475, -1.0  # stile base on the rear seat rail
    dirx, dirz = -math.sin(BACK_RECLINE), math.cos(BACK_RECLINE)
    for s, sy in enumerate((-0.71, 0.71)):
        bench.visual(
            Box((0.05, 0.05, BACK_LEN)),
            origin=Origin(
                xyz=(bx + dirx * BACK_LEN / 2.0, sy, bz + dirz * BACK_LEN / 2.0),
                rpy=(0.0, -BACK_RECLINE, 0.0),
            ),
            material="pine_dark",
            name=f"back_stile_{s}",
        )
    # Back slats mounted on the front face of the stiles.
    fx, fz = math.cos(BACK_RECLINE), math.sin(BACK_RECLINE)  # stile front normal
    for i, t in enumerate((0.13, 0.225, 0.32, 0.415)):
        bench.visual(
            Box((0.025, 1.42, 0.07)),
            origin=Origin(
                xyz=(bx + dirx * t + 0.0315 * fx, 0.0, bz + dirz * t + 0.0315 * fz),
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

    # Four thin steel hanger rods: a fore/aft V pair at each bench end, from
    # the pivot bar down to the armrest at each end.
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
                xyz=((top[0] + bot[0]) / 2.0, ry, (top[1] + bot[1]) / 2.0),
                rpy=(0.0, math.atan2(dx, dz), 0.0),
            ),
            material="steel",
            name=rname,
        )

    # ------------------------------------------------------------ articulation
    model.articulation(
        "bench_pivot",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=bench,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=150.0, velocity=2.5, lower=-SWING_LIMIT, upper=SWING_LIMIT
        ),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("shelter_frame")
    bench = object_model.get_part("bench_swing")
    pivot = object_model.get_articulation("bench_pivot")

    # Hanger rod tops seat into the pivot hanger bar (and pass inside the
    # straps that carry it) -- intentional captured-shaft embeds.
    for s in (0, 1):
        for rod in (f"hanger_rod_front_{s}", f"hanger_rod_rear_{s}"):
            ctx.allow_overlap(
                frame,
                bench,
                elem_a="hanger_bar",
                elem_b=rod,
                reason="Rod top is seated onto the pivot hanger bar (captured shaft).",
            )
            ctx.allow_overlap(
                frame,
                bench,
                elem_a=f"hanger_strap_{s}",
                elem_b=rod,
                reason="Rod top end passes inside the strap that carries the bar.",
            )
    ctx.expect_contact(
        frame,
        bench,
        elem_a="hanger_bar",
        elem_b="hanger_rod_front_0",
        name="hanger rods are seated on the pivot bar",
    )

    def count(part, prefix: str) -> int:
        return sum(1 for v in part.visuals if v.name and v.name.startswith(prefix))

    # Hero structure: two slatted A-frame end walls with slatted gables.
    ctx.check("four wall posts (two A-frame walls)", count(frame, "wall_") >= 4)
    n_wall_slat = count(frame, "wall_0_slat_") + count(frame, "wall_1_slat_")
    ctx.check("end walls are slatted lattices", n_wall_slat == 14, details=f"{n_wall_slat}")
    n_gable = count(frame, "wall_0_gable_slat_") + count(frame, "wall_1_gable_slat_")
    ctx.check("both gables are slatted", n_gable == 14, details=f"{n_gable}")
    ctx.check("ridge beam present", frame.get_visual("ridge_beam") is not None)
    ctx.check("ridge cap present", frame.get_visual("ridge_cap") is not None)
    n_purlin = count(frame, "roof_purlin_")
    ctx.check("roof purlins join the walls", n_purlin == 4, details=f"{n_purlin}")
    n_rib = count(frame, "roof_rib_")
    ctx.check("roof is corrugated (many ribs)", n_rib >= 40, details=f"{n_rib}")

    # Corrugation ridges run DOWN the slope: each rib is long in X, thin in Y.
    rib = ctx.part_element_world_aabb(frame, elem="roof_rib_front_0")
    if rib is not None:
        rx = rib[1][0] - rib[0][0]
        ry = rib[1][1] - rib[0][1]
        ctx.check(
            "corrugation ribs run down the slope",
            rx > 0.5 and ry < 0.1,
            details=f"rib_x={rx:.2f} rib_y={ry:.2f}",
        )

    # Roof overhangs the frame on all sides.
    sheet = ctx.part_element_world_aabb(frame, elem="roof_sheet_front")
    post = ctx.part_element_world_aabb(frame, elem="wall_1_post_0")
    if sheet is not None and post is not None:
        ctx.check(
            "roof overhangs the walls sideways",
            sheet[1][1] > post[1][1] + 0.08 and sheet[0][1] < -(post[1][1] + 0.08),
            details=f"sheet_ymax={sheet[1][1]:.2f} post_ymax={post[1][1]:.2f}",
        )
        ctx.check(
            "roof eave overhangs the wall feet",
            sheet[1][0] > 0.66,
            details=f"sheet_xmax={sheet[1][0]:.2f}",
        )

    # Overall scale ~2.0 m wide x 1.9 m tall x 1.4 m deep, resting on ground.
    fb = ctx.part_world_aabb(frame)
    if fb is not None:
        (xmn, ymn, zmn), (xmx, ymx, zmx) = fb
        ctx.check(
            "shelter is ~2.0 m wide, ~1.9 m tall, ~1.4 m deep",
            1.95 < (ymx - ymn) < 2.05 and 1.80 < zmx < 1.95 and 1.32 < (xmx - xmn) < 1.46,
            details=f"w={ymx - ymn:.2f} h={zmx:.2f} d={xmx - xmn:.2f}",
        )
        ctx.check("frame rests on the ground", abs(zmn) < 0.01, details=f"zmin={zmn:.3f}")

    # Bench: ~1.5 m wide slatted pendulum hanging clear of walls and floor.
    n_seat = count(bench, "seat_slat_")
    ctx.check("seat is slatted", n_seat == 6, details=f"{n_seat}")
    n_back = count(bench, "back_slat_")
    ctx.check("backrest is slatted", n_back == 4, details=f"{n_back}")
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
            "bench clears the slatted walls sideways",
            bymx < 0.82 and bymn > -0.82,
            details=f"ymax={bymx:.3f}",
        )
        ctx.check(
            "bench hangs above the floor, below the roof",
            bzmn > 0.30 and bzmx < 1.60,
            details=f"zmin={bzmn:.2f} zmax={bzmx:.2f}",
        )
    seat = ctx.part_element_world_aabb(bench, elem="seat_slat_0")
    if seat is not None:
        ctx.check(
            "seat at sitting height (~0.55 m)",
            0.48 < seat[1][2] < 0.62,
            details=f"seat_top={seat[1][2]:.2f}",
        )

    # The reclined backrest leans backwards (top further back than base).
    stile = ctx.part_element_world_aabb(bench, elem="back_stile_0")
    if stile is not None and seat is not None:
        ctx.check(
            "backrest is reclined behind the seat",
            stile[0][0] < -0.30 and stile[1][2] > seat[1][2] + 0.30,
            details=f"stile_xmin={stile[0][0]:.2f} stile_top={stile[1][2]:.2f}",
        )

    # Pendulum motion: the single revolute about Y displaces the bench
    # fore/aft, and the bench stays off the floor and under the roof slope
    # (plane z = ROOF_UNDER_Z0 - |x|) at both extremes. Positive joint values
    # swing the bench toward the backrest (-X) side.
    def _cx(aabb):
        return None if aabb is None else (aabb[0][0] + aabb[1][0]) / 2.0

    with ctx.pose({pivot: -SWING_LIMIT}):
        fwd_bb = ctx.part_world_aabb(bench)
        arm = ctx.part_element_world_aabb(bench, elem="armrest_0")
        if fwd_bb is not None:
            ctx.check(
                "bench clears the floor swung forward",
                fwd_bb[0][2] > 0.05,
                details=f"zmin={fwd_bb[0][2]:.3f}",
            )
        if arm is not None:
            worst = arm[1][2] + max(abs(arm[0][0]), abs(arm[1][0]))
            ctx.check(
                "armrest stays under the roof slope swung forward",
                worst <= ROOF_UNDER_Z0 + 0.001,
                details=f"plane_sum={worst:.3f} limit={ROOF_UNDER_Z0:.3f}",
            )
    with ctx.pose({pivot: SWING_LIMIT}):
        bwd_bb = ctx.part_world_aabb(bench)
        slat = ctx.part_element_world_aabb(bench, elem="back_slat_3")
        stile = ctx.part_element_world_aabb(bench, elem="back_stile_0")
        if bwd_bb is not None:
            ctx.check(
                "bench clears the floor swung backward",
                bwd_bb[0][2] > 0.05,
                details=f"zmin={bwd_bb[0][2]:.3f}",
            )
        if stile is not None:
            ctx.check(
                "backrest top stays below the roof swung backward",
                stile[1][2] < 1.24,
                details=f"stile_zmax={stile[1][2]:.3f}",
            )
        if slat is not None:
            worst = slat[1][2] + max(abs(slat[0][0]), abs(slat[1][0]))
            ctx.check(
                "top back slat stays under the roof slope swung backward",
                worst <= ROOF_UNDER_Z0 + 0.001,
                details=f"plane_sum={worst:.3f} limit={ROOF_UNDER_Z0:.3f}",
            )
    fwd, bwd = _cx(fwd_bb), _cx(bwd_bb)
    if fwd is not None and bwd is not None:
        ctx.check(
            "bench swings fore/aft about the ridge-parallel axis",
            (fwd - bwd) > 0.4,
            details=f"fwd_cx={fwd:.3f} bwd_cx={bwd:.3f}",
        )

    return ctx.report()
