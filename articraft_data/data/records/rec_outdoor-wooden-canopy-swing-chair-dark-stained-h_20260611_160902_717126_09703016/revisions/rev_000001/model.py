from __future__ import annotations

# Outdoor wooden canopy swing chair (dark-stained hardwood).
#
# Frame convention:
#   X = fore/aft depth (bench swings in X; sitter faces +X)
#   Y = left/right width (top beam and canopy ridge run along Y)
#   Z = up
#
# Root part = fixed stand: two A-shaped side supports (square-section legs)
# with a side cross rail each, joined by a top beam; a small flat tray shelf
# on the outside of each A-frame at armrest height; above the beam a lightly
# pitched green canopy (ridge bar, eave rails, rafters, two slope panels)
# with a thin scalloped fabric skirt strip hanging around all four edges.
# Four wooden swing arms hang from collinear revolute pivots on the top beam
# (one driver + three mimic followers, multiplier 1.0), and the slatted
# two-seat bench (seat, vertical-slat backrest, armrests) is fixed to the
# arms and swings as a pendulum between the A-frames.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# --- Stand dimensions (meters) -------------------------------------------------
LEG_W = 0.065          # square leg section
FOOT_X = 0.55          # leg foot center x at ground
LEG_TOP_X = 0.05       # leg axis x at the top reference height
LEG_TOP_Z = 1.66       # top reference height for the leg axis line
LEG_LEN = 1.74         # leg length along its axis
LEG_Y = 0.78           # A-frame center planes at y = +/-LEG_Y

BEAM_Z = 1.62          # top beam center height
BEAM_W = 0.07          # beam section
BEAM_LEN = 1.78

RAIL_Z = 0.57          # side cross rail height (tray support)
TRAY_Y = 0.90          # tray shelf center plane (outside the A-frame)

# --- Canopy ---------------------------------------------------------------------
RIDGE_Z = 1.815        # ridge bar center height
EAVE_X = 0.60          # eave rail x position
EAVE_Z = 1.74          # eave rail center height
CANOPY_PITCH = math.atan2(0.075, EAVE_X)  # gentle pitch (~7 deg)
PANEL_W = 1.76         # canopy width along Y
SKIRT_H = 0.13         # scalloped skirt height
SKIRT_TOP_Z = 1.775
SKIRT_T = 0.012

# --- Swing ----------------------------------------------------------------------
PIVOT_Z = 1.60         # swing pivot height (bolts through the top beam)
ARM_Y = 0.585          # swing arm center planes at y = +/-ARM_Y
ARM_ATTACH_Z = -0.97   # arm/bench attach height relative to the pivot
ARM_FRONT_X = 0.24     # front arm lower-end x (bench frame)
ARM_REAR_X = -0.28     # rear arm lower-end x (bench frame)
SWING_LIMIT = 0.4      # rad

# --- Bench (in the pendulum frame: origin under the pivot line) ------------------
SEAT_TOP = -1.10       # seat slat top (world ~0.50)
SEAT_W = 1.10          # bench width along Y
BACK_RECLINE = math.radians(12.0)

DRIVE_JOINT = "swing_pivot_front_0"


def _scalloped_skirt(length: float, name: str, peak: float = 0.0):
    """Thin vertical fabric strip with a scalloped (wavy) lower edge.

    Built in the local XY plane: x = along the strip, y = up (bottom of the
    scallop lobes at y=0, straight top edge at y=SKIRT_H, optional triangular
    peak above for the pitched gable ends), extruded SKIRT_T thick and
    centered on local z=0.
    """
    n = max(2, int(round(length / 0.11)))
    r = length / (2.0 * n)
    solid = (
        cq.Workplane("XY")
        .center(0.0, (SKIRT_H + r) / 2.0)
        .rect(length, SKIRT_H - r)
        .extrude(SKIRT_T)
    )
    for k in range(n):
        cx = -length / 2.0 + (2 * k + 1) * r
        solid = solid.union(cq.Workplane("XY").center(cx, r).circle(r).extrude(SKIRT_T))
    if peak > 0.0:
        solid = solid.union(
            cq.Workplane("XY")
            .polyline(
                [
                    (-length / 2.0, SKIRT_H),
                    (0.0, SKIRT_H + peak),
                    (length / 2.0, SKIRT_H),
                ]
            )
            .close()
            .extrude(SKIRT_T)
        )
    solid = solid.translate((0.0, 0.0, -SKIRT_T / 2.0))
    return mesh_from_cadquery(solid, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wooden_canopy_swing_chair")
    model.material("wood_dark", rgba=(0.28, 0.165, 0.09, 1.0))
    model.material("wood", rgba=(0.37, 0.225, 0.12, 1.0))
    model.material("canvas_green", rgba=(0.10, 0.36, 0.18, 1.0))
    model.material("canvas_green_dark", rgba=(0.075, 0.27, 0.135, 1.0))
    model.material("steel", rgba=(0.45, 0.46, 0.48, 1.0))

    frame = model.part("a_frame_stand")

    # ------------------------------------------------------ A-frame legs
    tilt = math.atan2(FOOT_X - LEG_TOP_X, LEG_TOP_Z)
    slope = (FOOT_X - LEG_TOP_X) / LEG_TOP_Z  # dx per dz toward the center
    # Lift so the lowest tilted-box corner rests exactly on the ground.
    leg_cz = (LEG_LEN / 2.0) * math.cos(tilt) + (LEG_W / 2.0) * math.sin(tilt)
    leg_cx = FOOT_X - slope * leg_cz
    for s, ly in enumerate((-LEG_Y, LEG_Y)):
        for tag, sgn in (("front", 1.0), ("rear", -1.0)):
            frame.visual(
                Box((LEG_W, LEG_W, LEG_LEN)),
                origin=Origin(xyz=(sgn * leg_cx, ly, leg_cz), rpy=(0.0, -sgn * tilt, 0.0)),
                material="wood_dark",
                name=f"leg_{tag}_{s}",
            )
        # Side cross rail between the two legs of this A (tray support).
        frame.visual(
            Box((0.83, 0.05, 0.06)),
            origin=Origin(xyz=(0.0, ly, RAIL_Z)),
            material="wood_dark",
            name=f"side_rail_{s}",
        )

    # Small flat tray shelves on the OUTSIDE of each A-frame at armrest height.
    for s, sgn in enumerate((-1.0, 1.0)):
        for b, bx in enumerate((-0.20, 0.20)):
            frame.visual(
                Box((0.05, 0.30, 0.03)),
                origin=Origin(xyz=(bx, sgn * (TRAY_Y - 0.005), 0.585)),
                material="wood_dark",
                name=f"tray_bearer_{s}_{b}",
            )
        frame.visual(
            Box((0.58, 0.30, 0.025)),
            origin=Origin(xyz=(0.0, sgn * TRAY_Y, 0.6125)),
            material="wood",
            name=f"tray_shelf_{s}",
        )

    # ------------------------------------------------------ top beam
    frame.visual(
        Box((BEAM_W, BEAM_LEN, BEAM_W)),
        origin=Origin(xyz=(0.0, 0.0, BEAM_Z)),
        material="wood_dark",
        name="top_beam",
    )

    # ------------------------------------------------------ canopy frame
    # Struts sit outboard of the swing-arm planes (arms at |y| <= ~0.60).
    for s, sy in enumerate((-0.70, 0.70)):
        frame.visual(
            Box((0.05, 0.05, 0.16)),
            origin=Origin(xyz=(0.0, sy, 1.735)),
            material="wood_dark",
            name=f"ridge_strut_{s}",
        )
    frame.visual(
        Box((0.05, 1.72, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, RIDGE_Z)),
        material="wood_dark",
        name="ridge_bar",
    )
    for tag, sgn in (("front", 1.0), ("rear", -1.0)):
        frame.visual(
            Box((0.04, 1.72, 0.04)),
            origin=Origin(xyz=(sgn * EAVE_X, 0.0, EAVE_Z)),
            material="wood_dark",
            name=f"eave_rail_{tag}",
        )
        for s, ry in enumerate((-0.86, 0.86)):
            frame.visual(
                Box((0.645, 0.04, 0.045)),
                origin=Origin(
                    xyz=(sgn * 0.30, ry, 1.7775),
                    rpy=(0.0, sgn * CANOPY_PITCH, 0.0),
                ),
                material="wood_dark",
                name=f"rafter_{tag}_{s}",
            )

    # ------------------------------------------------------ canopy panels + skirt
    for tag, sgn in (("front", 1.0), ("rear", -1.0)):
        frame.visual(
            Box((0.68, PANEL_W, 0.012)),
            origin=Origin(
                xyz=(sgn * 0.325, 0.0, 1.797),
                rpy=(0.0, sgn * CANOPY_PITCH, 0.0),
            ),
            material="canvas_green",
            name=f"canopy_panel_{tag}",
        )
    frame.visual(
        Box((0.10, PANEL_W, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 1.852)),
        material="canvas_green_dark",
        name="canopy_ridge_cap",
    )
    # Scalloped skirt strips around the canopy edge. Eave strips: local x ->
    # world Y, local y -> world Z, thickness -> world X (rpy = (pi/2, 0, pi/2)).
    for tag, sgn in (("front", 1.0), ("rear", -1.0)):
        frame.visual(
            _scalloped_skirt(PANEL_W, f"skirt_{tag}_mesh"),
            origin=Origin(
                xyz=(sgn * 0.66, 0.0, SKIRT_TOP_Z - SKIRT_H),
                rpy=(math.pi / 2.0, 0.0, math.pi / 2.0),
            ),
            material="canvas_green",
            name=f"canopy_skirt_{tag}",
        )
    # Gable-end strips with a peaked top following the pitch: local x ->
    # world X, local y -> world Z, thickness -> world Y (rpy = (pi/2, 0, 0)).
    for s, sgn in enumerate((-1.0, 1.0)):
        frame.visual(
            _scalloped_skirt(1.34, f"skirt_side_{s}_mesh", peak=0.065),
            origin=Origin(
                xyz=(0.0, sgn * 0.883, SKIRT_TOP_Z - SKIRT_H),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material="canvas_green",
            name=f"canopy_skirt_side_{s}",
        )

    # ------------------------------------------------------ four wooden swing arms
    # Each arm hangs from its own revolute pivot on the top beam; the four
    # pivots are collinear on the beam axis. Front/rear arms at each side
    # share a pivot bolt and fan out (slight V) to the bench armrest.
    def _arm_part(name: str, lower_x: float, with_bolt: bool):
        part = model.part(name)
        theta = math.atan2(abs(lower_x), -ARM_ATTACH_Z)
        sgn = 1.0 if lower_x > 0.0 else -1.0
        reach = math.hypot(lower_x, ARM_ATTACH_Z)
        t0, t1 = -0.06, reach + 0.045  # extend above the pivot and past the attach
        length = t1 - t0
        tc = (t0 + t1) / 2.0
        dx, dz = sgn * math.sin(theta), -math.cos(theta)
        part.visual(
            Box((0.055, 0.035, length)),
            origin=Origin(
                xyz=(dx * tc, 0.0, dz * tc),
                rpy=(0.0, -sgn * theta, 0.0),
            ),
            material="wood",
            name="bar",
        )
        if with_bolt:
            part.visual(
                Cylinder(radius=0.011, length=0.09),
                origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material="steel",
                name="pivot_bolt",
            )
        return part

    arm_front_0 = _arm_part("swing_arm_front_0", ARM_FRONT_X, with_bolt=True)
    arm_front_1 = _arm_part("swing_arm_front_1", ARM_FRONT_X, with_bolt=True)
    arm_rear_0 = _arm_part("swing_arm_rear_0", ARM_REAR_X, with_bolt=False)
    arm_rear_1 = _arm_part("swing_arm_rear_1", ARM_REAR_X, with_bolt=False)

    # ------------------------------------------------------ slatted bench
    bench = model.part("bench")

    # Seat frame: front/rear rails + end rails.
    for tag, rx in (("front", 0.225), ("rear", -0.225)):
        bench.visual(
            Box((0.05, SEAT_W, 0.05)),
            origin=Origin(xyz=(rx, 0.0, -1.15)),
            material="wood_dark",
            name=f"seat_{tag}_rail",
        )
    for s, ry in enumerate((-0.5225, 0.5225)):
        bench.visual(
            Box((0.50, 0.055, 0.05)),
            origin=Origin(xyz=(0.0, ry, -1.15)),
            material="wood_dark",
            name=f"seat_end_rail_{s}",
        )
    # Six flat seat slats (long axis Y) with gaps in X.
    for i in range(6):
        sx = -0.2125 + i * 0.085
        bench.visual(
            Box((0.075, 1.06, 0.025)),
            origin=Origin(xyz=(sx, 0.0, SEAT_TOP - 0.0125)),
            material="wood",
            name=f"seat_slat_{i}",
        )

    # Reclined backrest with VERTICAL slats between two horizontal rails.
    bx0, bz0 = -0.255, -1.10  # back plane base point on the rear rail
    ux, uz = -math.sin(BACK_RECLINE), math.cos(BACK_RECLINE)
    for tag, t, sec in (
        ("lower", 0.06, (0.045, 1.06, 0.05)),
        ("upper", 0.52, (0.045, 1.06, 0.055)),
    ):
        bench.visual(
            Box(sec),
            origin=Origin(
                xyz=(bx0 + ux * t, 0.0, bz0 + uz * t),
                rpy=(0.0, -BACK_RECLINE, 0.0),
            ),
            material="wood_dark",
            name=f"back_{tag}_rail",
        )
    for i in range(9):
        sy = -0.50 + i * 0.125
        t = 0.285
        bench.visual(
            Box((0.022, 0.062, 0.43)),
            origin=Origin(
                xyz=(bx0 + ux * t, sy, bz0 + uz * t),
                rpy=(0.0, -BACK_RECLINE, 0.0),
            ),
            material="wood",
            name=f"back_slat_{i}",
        )

    # Armrests: board from the backrest to a front post on the seat rail.
    for s, ay in enumerate((-0.55, 0.55)):
        bench.visual(
            Box((0.045, 0.045, 0.17)),
            origin=Origin(xyz=(0.225, ay * 0.99, -1.07)),
            material="wood_dark",
            name=f"arm_post_{s}",
        )
        bench.visual(
            Box((0.57, 0.08, 0.03)),
            origin=Origin(xyz=(-0.02, ay, -0.985)),
            material="wood",
            name=f"armrest_{s}",
        )

    # ------------------------------------------------------ articulations
    arm_specs = [
        (DRIVE_JOINT, arm_front_0, -ARM_Y, None),
        ("swing_pivot_front_1", arm_front_1, ARM_Y, Mimic(joint=DRIVE_JOINT, multiplier=1.0)),
        ("swing_pivot_rear_0", arm_rear_0, -ARM_Y, Mimic(joint=DRIVE_JOINT, multiplier=1.0)),
        ("swing_pivot_rear_1", arm_rear_1, ARM_Y, Mimic(joint=DRIVE_JOINT, multiplier=1.0)),
    ]
    for jname, part, jy, mimic in arm_specs:
        model.articulation(
            jname,
            ArticulationType.REVOLUTE,
            parent=frame,
            child=part,
            origin=Origin(xyz=(0.0, jy, PIVOT_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=120.0, velocity=2.5, lower=-SWING_LIMIT, upper=SWING_LIMIT
            ),
            mimic=mimic,
        )

    # The bench is bolted rigidly to the swing arms (carried by the driver arm
    # in the kinematic tree; the mimic followers move identically).
    model.articulation(
        "bench_mount",
        ArticulationType.FIXED,
        parent=arm_front_0,
        child=bench,
        origin=Origin(xyz=(0.0, ARM_Y, 0.0)),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("a_frame_stand")
    bench = object_model.get_part("bench")
    arms = [
        object_model.get_part("swing_arm_front_0"),
        object_model.get_part("swing_arm_front_1"),
        object_model.get_part("swing_arm_rear_0"),
        object_model.get_part("swing_arm_rear_1"),
    ]
    driver = object_model.get_articulation(DRIVE_JOINT)
    followers = [
        object_model.get_articulation("swing_pivot_front_1"),
        object_model.get_articulation("swing_pivot_rear_0"),
        object_model.get_articulation("swing_pivot_rear_1"),
    ]

    # --- intentional overlaps ---------------------------------------------
    for arm in arms:
        ctx.allow_overlap(
            frame,
            arm,
            elem_a="top_beam",
            elem_b="bar",
            reason="Arm top is captured on a pivot bolt passing through the beam.",
        )
    for arm in (arms[0], arms[1]):
        ctx.allow_overlap(
            frame,
            arm,
            elem_a="top_beam",
            elem_b="pivot_bolt",
            reason="Pivot bolt passes through the top beam.",
        )
    for fr, rr in ((arms[0], arms[2]), (arms[1], arms[3])):
        ctx.allow_overlap(
            fr,
            rr,
            elem_a="bar",
            elem_b="bar",
            reason="Front and rear hanger share the pivot boss; bars stack on one bolt.",
        )
        ctx.allow_overlap(
            fr,
            rr,
            elem_a="pivot_bolt",
            elem_b="bar",
            reason="Shared pivot bolt passes through both hanger bars.",
        )
    for arm, s in ((arms[0], 0), (arms[1], 1), (arms[2], 0), (arms[3], 1)):
        ctx.allow_overlap(
            bench,
            arm,
            elem_a=f"armrest_{s}",
            elem_b="bar",
            reason="Arm lower end is bolted onto the bench armrest side.",
        )

    # Arms are seated on the beam and bolted to the armrests.
    ctx.expect_contact(
        frame,
        arms[0],
        elem_a="top_beam",
        elem_b="bar",
        name="swing arm hangs from the top beam",
    )
    ctx.expect_contact(
        bench,
        arms[0],
        elem_a="armrest_0",
        elem_b="bar",
        name="swing arm is bolted to the bench armrest",
    )

    def count(part, prefix: str) -> int:
        return sum(1 for v in part.visuals if v.name and v.name.startswith(prefix))

    # --- four swing arms with a mimic chain --------------------------------
    n_arm_parts = sum(1 for p in object_model.parts if p.name.startswith("swing_arm_"))
    ctx.check("four wooden swing arms", n_arm_parts == 4, details=f"{n_arm_parts}")
    ctx.check("driver pivot is not a mimic", driver.mimic is None)
    for f in followers:
        ok = (
            f.mimic is not None
            and f.mimic.joint == DRIVE_JOINT
            and abs(f.mimic.multiplier - 1.0) < 1e-9
        )
        ctx.check(f"{f.name} mimics the driver with multiplier 1.0", ok)
    ctx.check(
        "swing limits are about -0.4..0.4 rad",
        abs(driver.motion_limits.lower + SWING_LIMIT) < 1e-9
        and abs(driver.motion_limits.upper - SWING_LIMIT) < 1e-9,
    )

    # --- frame: A-frames, beam, trays on both sides -------------------------
    ctx.check("four A-frame legs", count(frame, "leg_") == 4)
    ctx.check("side cross rails on both A-frames", count(frame, "side_rail_") == 2)
    ctx.check("top beam present", frame.get_visual("top_beam") is not None)
    fb = ctx.part_world_aabb(frame)
    if fb is not None:
        ctx.check("stand rests on the ground", abs(fb[0][2]) < 0.01, details=f"zmin={fb[0][2]:.4f}")
    for s, sgn in ((0, -1.0), (1, 1.0)):
        tray = ctx.part_element_world_aabb(frame, elem=f"tray_shelf_{s}")
        leg = ctx.part_element_world_aabb(frame, elem=f"leg_front_{s}")
        if tray is not None and leg is not None:
            cy = (tray[0][1] + tray[1][1]) / 2.0
            leg_cy = (leg[0][1] + leg[1][1]) / 2.0
            ctx.check(
                f"tray shelf {s} sits outboard of its A-frame at armrest height",
                sgn * cy > sgn * leg_cy + 0.08 and 0.55 < tray[1][2] < 0.70,
                details=f"tray_cy={cy:.3f} leg_cy={leg_cy:.3f} top={tray[1][2]:.3f}",
            )

    # --- canopy above the bench, scalloped skirt below the canopy edge ------
    panel = ctx.part_element_world_aabb(frame, elem="canopy_panel_front")
    bb = ctx.part_world_aabb(bench)
    if panel is not None and bb is not None:
        ctx.check(
            "green canopy is above the bench",
            panel[0][2] > bb[1][2] + 0.5,
            details=f"panel_zmin={panel[0][2]:.2f} bench_zmax={bb[1][2]:.2f}",
        )
    skirt_names = (
        "canopy_skirt_front",
        "canopy_skirt_rear",
        "canopy_skirt_side_0",
        "canopy_skirt_side_1",
    )
    for sk in skirt_names:
        skirt = ctx.part_element_world_aabb(frame, elem=sk)
        ok = skirt is not None
        details = "missing"
        if skirt is not None and panel is not None:
            h = skirt[1][2] - skirt[0][2]
            # Hangs below the canopy eave edge while still overlapping it.
            ok = skirt[0][2] < panel[0][2] - 0.05 and skirt[1][2] > panel[0][2] and 0.10 < h < 0.25
            details = f"zmin={skirt[0][2]:.3f} zmax={skirt[1][2]:.3f} h={h:.3f}"
        ctx.check(f"scalloped skirt {sk} hangs below the canopy edge", ok, details=details)

    # --- bench: slat counts, vertical back slats, seat height ----------------
    n_seat = count(bench, "seat_slat_")
    ctx.check("seat has six slats", n_seat == 6, details=f"{n_seat}")
    n_back = count(bench, "back_slat_")
    ctx.check("backrest has nine vertical slats", n_back == 9, details=f"{n_back}")
    slat = ctx.part_element_world_aabb(bench, elem="back_slat_4")
    if slat is not None:
        dy = slat[1][1] - slat[0][1]
        dz = slat[1][2] - slat[0][2]
        ctx.check(
            "back slats are vertical (taller than wide)",
            dz > 3.0 * dy,
            details=f"dz={dz:.3f} dy={dy:.3f}",
        )
    ctx.check("two armrests", count(bench, "armrest_") == 2)
    seat = ctx.part_element_world_aabb(bench, elem="seat_slat_0")
    if seat is not None:
        ctx.check(
            "seat at sitting height (~0.50 m)",
            0.44 < seat[1][2] < 0.57,
            details=f"seat_top={seat[1][2]:.3f}",
        )
    if bb is not None:
        ctx.check(
            "two-seat bench width (~1.1 m)",
            1.02 < (bb[1][1] - bb[0][1]) < 1.30,
            details=f"w={bb[1][1] - bb[0][1]:.2f}",
        )

    # --- swing clearance at the limits --------------------------------------
    # Legs sit at |y| >= ~0.75; the bench/arm assembly must stay narrower and
    # clear of the ground and the beam at both extremes.
    leg_inner = LEG_Y - 0.0325

    def _swing_aabb():
        boxes = [ctx.part_world_aabb(bench)] + [ctx.part_world_aabb(a) for a in arms]
        boxes = [b for b in boxes if b is not None]
        if not boxes:
            return None
        mn = [min(b[0][i] for b in boxes) for i in range(3)]
        mx = [max(b[1][i] for b in boxes) for i in range(3)]
        return (tuple(mn), tuple(mx))

    centers = {}
    for q in (-SWING_LIMIT, 0.0, SWING_LIMIT):
        with ctx.pose({driver: q}):
            sw = _swing_aabb()
            if sw is None:
                continue
            ctx.check(
                f"swing assembly clears the ground at q={q:+.1f}",
                sw[0][2] > 0.05,
                details=f"zmin={sw[0][2]:.3f}",
            )
            ctx.check(
                f"swing assembly clears the A-frame legs sideways at q={q:+.1f}",
                sw[1][1] < leg_inner - 0.10 and sw[0][1] > -(leg_inner - 0.10),
                details=f"ymax={sw[1][1]:.3f} limit={leg_inner - 0.10:.3f}",
            )
            bb_q = ctx.part_world_aabb(bench)
            if bb_q is not None:
                centers[q] = (bb_q[0][0] + bb_q[1][0]) / 2.0
                ctx.check(
                    f"bench stays below the top beam at q={q:+.1f}",
                    bb_q[1][2] < BEAM_Z - BEAM_W / 2.0 - 0.05,
                    details=f"bench_zmax={bb_q[1][2]:.3f}",
                )
    if -SWING_LIMIT in centers and SWING_LIMIT in centers:
        travel = centers[-SWING_LIMIT] - centers[SWING_LIMIT]
        ctx.check(
            "bench swings fore/aft on the pivots",
            travel > 0.5,
            details=f"travel={travel:.3f}",
        )

    return ctx.report()
