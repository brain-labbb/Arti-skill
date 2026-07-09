from __future__ import annotations

# Metal A-frame swing bench in plain CAD-render style (monochrome light gray).
# Reference: picture/Bench/Wood Swing/004.png
#
# Frame convention:
#   X = fore/aft (the bench faces +X and swings in X)
#   Y = left/right (crossbar runs along Y; "left" = +Y for a seated user)
#   Z = up
#
# Structure:
#   - Two tubular-steel A-frame end supports (angled legs, apex sleeve,
#     horizontal cross brace, two diagonal braces, flat foot plates).
#   - A top crossbar spanning the apexes, carrying four clevis pivot lugs.
#   - A slatted metal bench (flat slatted seat, curved rolltop slatted back,
#     armrest ends with round cup holders) hanging on four rigid tubular link
#     arms with pivot bolts.
#
# Articulation: the four link arms swing on revolute pivots at the crossbar.
# `swing_drive` (front-left arm) is the driver; the other three arms mimic it
# with multiplier 1.0. The bench is fixed to the driver arm, so the whole
# pendulum swings rigidly, exactly like the real four-link hanger.

import math

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
)

# --- Frame dimensions (meters) -------------------------------------------------
APEX_Z = 1.80  # apex sleeve / crossbar axis height
FRAME_Y = 0.80  # A-frame center planes at y = +/-FRAME_Y
FOOT_X = 0.75  # leg foot position in X
LEG_R = 0.018  # leg tube radius (36 mm dia)
BRACE_R = 0.014  # brace tube radius (28 mm dia)
BAR_R = 0.020  # crossbar tube radius (40 mm dia)
PLATE_T = 0.012  # foot plate thickness

PIVOT_Z = 1.755  # link-arm pivot bolt height (below the crossbar axis)
ARM_Y = 0.71  # link-arm planes at y = +/-ARM_Y
ARM_DX = 0.28  # fore/aft offset of arm lower ends (V-shaped hanger pair)
ARM_DROP = 0.845  # vertical drop from pivot to the armrest anchor bolt
ARM_R = 0.0125  # link-arm tube radius (25 mm dia)
SWING_LIMIT = 0.45  # rad

# --- Bench dimensions (bench frame origin at world (0, 0, BENCH_Z) at q=0) -----
BENCH_Z = 0.62
SEAT_W = 1.40  # bench width along Y
SEAT_HALF_X = 0.26  # seat rail half depth
N_SEAT_SLATS = 16
N_BACK_SLATS = 8

# Curved rolltop back: circular arc in the XZ plane (bench frame).
BACK_R = 0.70  # arc radius
BACK_SX, BACK_SZ = -0.27, 0.03  # arc start point (at the rear seat rail)
BACK_PHI0 = 0.12  # tangent lean-back angle at the start (rad)
BACK_DPHI = 0.1043  # arc step per slat row
_BCX = BACK_SX - BACK_R * math.cos(BACK_PHI0)
_BCZ = BACK_SZ - BACK_R * math.sin(BACK_PHI0)


def _arc_point(phi: float) -> tuple[float, float]:
    """Point on the back arc (x, z) in the bench frame."""
    return (_BCX + BACK_R * math.cos(phi), _BCZ + BACK_R * math.sin(phi))


def _build_a_frame(part, ys: float) -> None:
    """Two angled legs + apex sleeve + braces + foot plates at plane y=ys."""
    for tag, xf in (("front", FOOT_X), ("rear", -FOOT_X)):
        # Leg tube from just above its foot plate up into the apex sleeve
        # (stopping short of the crossbar that runs inside the sleeve).
        z0, z1 = 0.013, 1.776
        scale = math.hypot(FOOT_X, APEX_Z) / APEX_Z
        length = (z1 - z0) * scale
        zc = (z0 + z1) / 2.0
        xc = xf * (1.0 - zc / APEX_Z)
        part.visual(
            Cylinder(radius=LEG_R, length=length),
            origin=Origin(xyz=(xc, ys, zc), rpy=(0.0, math.atan2(-xf, APEX_Z), 0.0)),
            material="steel_frame",
            name=f"leg_{tag}",
        )
        # Flat rectangular foot plate under the leg end.
        part.visual(
            Box((0.15, 0.10, PLATE_T)),
            origin=Origin(xyz=(xf + 0.01 * math.copysign(1.0, xf), ys, PLATE_T / 2.0)),
            material="steel_dark",
            name=f"foot_plate_{tag}",
        )
    # Apex sleeve: short fat collar around the crossbar where the legs meet.
    part.visual(
        Cylinder(radius=0.032, length=0.11),
        origin=Origin(xyz=(0.0, ys, APEX_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="steel_dark",
        name="apex_sleeve",
    )
    # Horizontal cross brace between the legs at mid height.
    part.visual(
        Cylinder(radius=BRACE_R, length=1.07),
        origin=Origin(xyz=(0.0, ys, 0.55), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="steel_frame",
        name="cross_brace",
    )
    # Two diagonal braces from the cross-brace center up to the legs.
    for tag, sgn in (("front", 1.0), ("rear", -1.0)):
        xe = sgn * FOOT_X * (1.0 - 1.15 / APEX_Z)
        dx, dz = xe - 0.0, 1.15 - 0.55
        length = math.hypot(dx, dz) + 0.03
        part.visual(
            Cylinder(radius=BRACE_R, length=length),
            origin=Origin(
                xyz=(dx / 2.0, ys, 0.55 + dz / 2.0),
                rpy=(0.0, math.atan2(dx, dz), 0.0),
            ),
            material="steel_frame",
            name=f"diag_brace_{tag}",
        )


def _build_link_arm(part, dx: float, *, with_pivot_bolt: bool) -> None:
    """Rigid tubular hanger arm in its pivot frame (origin = pivot bolt).

    The front and rear arm of each side form a V-pair sharing one clevis
    pivot; only the front arm carries the physical pivot bolt.
    """
    drop = -ARM_DROP
    length = math.hypot(dx, drop)
    ux, uz = dx / length, drop / length
    # Tube from the pivot down to just past the anchor bolt.
    tube_len = length + 0.012
    part.visual(
        Cylinder(radius=ARM_R, length=tube_len),
        origin=Origin(
            xyz=(ux * tube_len / 2.0, 0.0, uz * tube_len / 2.0),
            rpy=(0.0, math.atan2(dx, drop), 0.0),
        ),
        material="steel_frame",
        name="arm_tube",
    )
    if with_pivot_bolt:
        # Pivot bolt through the clevis lug at the crossbar.
        part.visual(
            Cylinder(radius=0.0145, length=0.05),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="steel_dark",
            name="pivot_bolt",
        )
        for tag, sy in (("left", 0.028), ("right", -0.028)):
            part.visual(
                Cylinder(radius=0.019, length=0.008),
                origin=Origin(xyz=(0.0, sy, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material="steel_dark",
                name=f"pivot_bolt_head_{tag}",
            )
    # Anchor bolt into the bench armrest at the lower end.
    part.visual(
        Cylinder(radius=0.012, length=0.05),
        origin=Origin(xyz=(dx, 0.0, drop), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="steel_dark",
        name="anchor_bolt",
    )
    for tag, sy in (("left", 0.027), ("right", -0.027)):
        part.visual(
            Cylinder(radius=0.016, length=0.008),
            origin=Origin(xyz=(dx, sy, drop), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="steel_dark",
            name=f"anchor_bolt_head_{tag}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="metal_a_frame_swing_bench")
    # Monochrome CAD-render palette: everything plain light gray metal.
    model.material("steel_light", rgba=(0.86, 0.87, 0.89, 1.0))
    model.material("steel_frame", rgba=(0.79, 0.80, 0.83, 1.0))
    model.material("steel_dark", rgba=(0.62, 0.64, 0.67, 1.0))

    # ----------------------------------------------------------- A-frame ends
    a_frame_left = model.part("a_frame_left")
    _build_a_frame(a_frame_left, FRAME_Y)
    a_frame_right = model.part("a_frame_right")
    _build_a_frame(a_frame_right, -FRAME_Y)

    # ------------------------------------------------- crossbar + pivot lugs
    crossbar = model.part("crossbar")
    crossbar.visual(
        Cylinder(radius=BAR_R, length=1.78),
        origin=Origin(xyz=(0.0, 0.0, APEX_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="steel_frame",
        name="bar",
    )
    for tag, sy in (("left", 0.886), ("right", -0.886)):
        crossbar.visual(
            Cylinder(radius=0.026, length=0.014),
            origin=Origin(xyz=(0.0, sy, APEX_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="steel_dark",
            name=f"end_cap_{tag}",
        )
    # Two clevis lugs (two plates each) hanging under the bar at the pivots.
    # Each lug carries the V-pair of link arms for its side of the bench.
    for tag, ly in (("left", ARM_Y), ("right", -ARM_Y)):
        for side, off in (("inner", -0.018), ("outer", 0.018)):
            crossbar.visual(
                Box((0.05, 0.008, 0.075)),
                origin=Origin(xyz=(0.0, ly + off, 1.7775)),
                material="steel_dark",
                name=f"lug_{tag}_{side}",
            )

    # --------------------------------------------------------- four link arms
    arm_specs = (
        ("link_arm_front_left", ARM_DX, True),
        ("link_arm_front_right", ARM_DX, True),
        ("link_arm_rear_left", -ARM_DX, False),
        ("link_arm_rear_right", -ARM_DX, False),
    )
    arms: dict[str, object] = {}
    for name, dx, with_bolt in arm_specs:
        arm = model.part(name)
        _build_link_arm(arm, dx, with_pivot_bolt=with_bolt)
        arms[name] = arm

    # ------------------------------------------------------------- bench seat
    bench_seat = model.part("bench_seat")
    rail_half = SEAT_W / 2.0
    for tag, rx in (("front", SEAT_HALF_X), ("rear", -SEAT_HALF_X)):
        bench_seat.visual(
            Box((0.05, SEAT_W, 0.05)),
            origin=Origin(xyz=(rx, 0.0, 0.0)),
            material="steel_light",
            name=f"rail_{tag}",
        )
    for tag, ry in (("left", 0.675), ("right", -0.675)):
        bench_seat.visual(
            Box((2.0 * SEAT_HALF_X + 0.05, 0.05, 0.05)),
            origin=Origin(xyz=(0.0, ry, 0.0)),
            material="steel_light",
            name=f"rail_{tag}",
        )
    # Slatted seat: slats run fore/aft, repeated across the width with gaps.
    step = (SEAT_W - 0.14) / (N_SEAT_SLATS - 1)
    for k in range(N_SEAT_SLATS):
        sy = -(rail_half - 0.07) + k * step
        bench_seat.visual(
            Box((0.50, 0.054, 0.022)),
            origin=Origin(xyz=(0.01, sy, 0.035)),
            material="steel_light",
            name=f"seat_slat_{k}",
        )
    # Armrest ends with round cup holders (as in the CAD render).
    for tag, sgn in (("left", 1.0), ("right", -1.0)):
        bench_seat.visual(
            Box((0.045, 0.045, 0.27)),
            origin=Origin(xyz=(SEAT_HALF_X, sgn * 0.66, 0.155)),
            material="steel_light",
            name=f"arm_post_front_{tag}",
        )
        bench_seat.visual(
            Box((0.04, 0.04, 0.27)),
            origin=Origin(xyz=(-0.20, sgn * 0.66, 0.155)),
            material="steel_light",
            name=f"arm_post_rear_{tag}",
        )
        bench_seat.visual(
            Box((0.60, 0.06, 0.025)),
            origin=Origin(xyz=(0.0, sgn * 0.665, 0.29)),
            material="steel_light",
            name=f"armrest_{tag}",
        )
        bench_seat.visual(
            Cylinder(radius=0.04, length=0.032),
            origin=Origin(xyz=(0.20, sgn * 0.64, 0.316)),
            material="steel_dark",
            name=f"cup_holder_{tag}",
        )

    # ---------------------------------------------- curved slatted bench back
    bench_back = model.part("bench_back")
    # Horizontal slats placed along a circular arc (rolltop curve).
    for k in range(N_BACK_SLATS):
        phi = 0.18 + k * BACK_DPHI
        px, pz = _arc_point(phi)
        bench_back.visual(
            Box((0.022, 1.36, 0.06)),
            origin=Origin(xyz=(px, 0.0, pz), rpy=(0.0, -phi, 0.0)),
            material="steel_light",
            name=f"back_slat_{k}",
        )
    # Three curved support ribs behind the slats (segmented along the arc),
    # the lowest segment bolting onto the rear seat rail.
    for r, ry in (("center", 0.0), ("left", 0.60), ("right", -0.60)):
        for j in range(9):
            phi = 0.0757 + j * BACK_DPHI
            px, pz = _arc_point(phi)
            nx, nz = math.cos(phi), math.sin(phi)  # outward (front) normal
            bench_back.visual(
                Box((0.02, 0.05, 0.082)),
                origin=Origin(
                    xyz=(px - 0.018 * nx, ry, pz - 0.018 * nz),
                    rpy=(0.0, -phi, 0.0),
                ),
                material="steel_frame",
                name=f"back_rib_{r}_{j}",
            )

    # ------------------------------------------------------------ kinematics
    model.articulation(
        "frame_left_to_right",
        ArticulationType.FIXED,
        parent=a_frame_left,
        child=a_frame_right,
    )
    model.articulation(
        "frame_to_crossbar",
        ArticulationType.FIXED,
        parent=a_frame_left,
        child=crossbar,
    )
    limits = MotionLimits(effort=200.0, velocity=2.0, lower=-SWING_LIMIT, upper=SWING_LIMIT)
    # Driver pivot: front-left link arm. Positive q swings the bench rearward.
    model.articulation(
        "swing_drive",
        ArticulationType.REVOLUTE,
        parent=crossbar,
        child=arms["link_arm_front_left"],
        origin=Origin(xyz=(0.0, ARM_Y, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=limits,
    )
    for jname, aname, ay in (
        ("swing_follow_front_right", "link_arm_front_right", -ARM_Y),
        ("swing_follow_rear_left", "link_arm_rear_left", ARM_Y),
        ("swing_follow_rear_right", "link_arm_rear_right", -ARM_Y),
    ):
        model.articulation(
            jname,
            ArticulationType.REVOLUTE,
            parent=crossbar,
            child=arms[aname],
            origin=Origin(xyz=(0.0, ay, PIVOT_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=limits,
            mimic=Mimic(joint="swing_drive", multiplier=1.0),
        )
    # Bench rides rigidly on the driver arm (all four pivots share one axis).
    model.articulation(
        "arm_to_bench",
        ArticulationType.FIXED,
        parent=arms["link_arm_front_left"],
        child=bench_seat,
        origin=Origin(xyz=(0.0, -ARM_Y, BENCH_Z - PIVOT_Z)),
    )
    model.articulation(
        "seat_to_back",
        ArticulationType.FIXED,
        parent=bench_seat,
        child=bench_back,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    af_l = object_model.get_part("a_frame_left")
    af_r = object_model.get_part("a_frame_right")
    crossbar = object_model.get_part("crossbar")
    seat = object_model.get_part("bench_seat")
    back = object_model.get_part("bench_back")
    arm_names = (
        "link_arm_front_left",
        "link_arm_front_right",
        "link_arm_rear_left",
        "link_arm_rear_right",
    )
    arms = {n: object_model.get_part(n) for n in arm_names}
    drive = object_model.get_articulation("swing_drive")

    # ---------------------------------------------------------- allowances
    # Crossbar is captured inside both apex sleeves (sleeve wraps the bar).
    for af in (af_l, af_r):
        ctx.allow_overlap(
            crossbar,
            af,
            elem_a="bar",
            elem_b="apex_sleeve",
            reason="Crossbar tube is captured inside the A-frame apex sleeve.",
        )
    # Pivot bolts pass through the clevis lug plates; the front/rear arms of
    # each side form a V-pair joined at the shared pivot yoke; anchor bolts
    # are threaded into the armrests.
    for side in ("left", "right"):
        front = arms[f"link_arm_front_{side}"]
        rear = arms[f"link_arm_rear_{side}"]
        for plate in ("inner", "outer"):
            ctx.allow_overlap(
                crossbar,
                front,
                elem_a=f"lug_{side}_{plate}",
                elem_b="pivot_bolt",
                reason="Pivot bolt passes through the clevis lug plate.",
            )
        ctx.allow_overlap(
            front,
            rear,
            elem_a="arm_tube",
            elem_b="arm_tube",
            reason="V-pair hanger tubes are joined at the shared pivot yoke.",
        )
        ctx.allow_overlap(
            front,
            rear,
            elem_a="pivot_bolt",
            elem_b="arm_tube",
            reason="The shared pivot bolt passes through both hanger tube ends.",
        )
    for aname, arm in arms.items():
        armrest = "armrest_left" if aname.endswith("left") else "armrest_right"
        for elem in ("anchor_bolt", "anchor_bolt_head_left", "anchor_bolt_head_right"):
            ctx.allow_overlap(
                arm,
                seat,
                elem_a=elem,
                elem_b=armrest,
                reason="Anchor bolt is threaded into the bench armrest end.",
            )
    # The lowest back-rib segments bolt onto the rear seat rail.
    for r in ("center", "left", "right"):
        ctx.allow_overlap(
            back,
            seat,
            elem_a=f"back_rib_{r}_0",
            elem_b="rail_rear",
            reason="Curved back rib base is bolted to the rear seat rail.",
        )

    # -------------------------------------------- mimic chain: four link arms
    def count(part, prefix: str) -> int:
        return sum(1 for v in part.visuals if v.name and v.name.startswith(prefix))

    ctx.check("four link arms exist", len(arms) == 4)
    for n in arm_names:
        has_bolt = count(arms[n], "pivot_bolt") >= 1 if n.startswith("link_arm_front") else True
        ctx.check(
            f"{n} has a tube and an anchor bolt",
            count(arms[n], "arm_tube") == 1 and count(arms[n], "anchor_bolt") >= 1 and has_bolt,
        )
    lim = drive.motion_limits
    ctx.check(
        "driver swing limits ~ -0.45..0.45 rad",
        lim is not None
        and abs(lim.lower + SWING_LIMIT) < 1e-9
        and abs(lim.upper - SWING_LIMIT) < 1e-9,
    )
    followers = (
        "swing_follow_front_right",
        "swing_follow_rear_left",
        "swing_follow_rear_right",
    )
    for jn in followers:
        j = object_model.get_articulation(jn)
        ctx.check(
            f"{jn} mimics swing_drive with multiplier 1.0",
            j.mimic is not None
            and j.mimic.joint == "swing_drive"
            and abs(j.mimic.multiplier - 1.0) < 1e-9
            and abs(j.mimic.offset) < 1e-9,
        )

    # -------------------------------------------------- frame stands on plates
    n_plates = 0
    for af, side in ((af_l, "left"), (af_r, "right")):
        for tag in ("front", "rear"):
            bb = ctx.part_element_world_aabb(af, elem=f"foot_plate_{tag}")
            if bb is not None:
                n_plates += 1
                ctx.check(
                    f"{side} {tag} foot plate rests flat on the ground",
                    abs(bb[0][2]) < 0.0005,
                    details=f"zmin={bb[0][2]:.4f}",
                )
    ctx.check("four flat foot plates", n_plates == 4)
    fb = ctx.part_world_aabb(af_l)
    if fb is not None:
        ctx.check(
            "A-frame is ~1.8 m tall with ~1.6 m leg spread",
            1.78 < fb[1][2] < 1.90 and 1.50 < (fb[1][0] - fb[0][0]) < 1.75,
            details=f"h={fb[1][2]:.2f} spread={fb[1][0] - fb[0][0]:.2f}",
        )

    # ------------------------------------------------------------ slat counts
    n_seat = count(seat, "seat_slat_")
    ctx.check("seat is slatted (16 fore/aft slats)", n_seat == N_SEAT_SLATS, details=f"{n_seat}")
    n_back = count(back, "back_slat_")
    ctx.check("back is slatted (8 curved rows)", n_back == N_BACK_SLATS, details=f"{n_back}")
    # The back actually curves: top row is well above and behind the bottom row.
    b0 = ctx.part_element_world_aabb(back, elem="back_slat_0")
    b7 = ctx.part_element_world_aabb(back, elem=f"back_slat_{N_BACK_SLATS - 1}")
    if b0 is not None and b7 is not None:
        c0 = ((b0[0][0] + b0[1][0]) / 2.0, (b0[0][2] + b0[1][2]) / 2.0)
        c7 = ((b7[0][0] + b7[1][0]) / 2.0, (b7[0][2] + b7[1][2]) / 2.0)
        ctx.check(
            "back curves up and rolls rearward",
            (c7[1] - c0[1]) > 0.35 and (c0[0] - c7[0]) > 0.15,
            details=f"dz={c7[1] - c0[1]:.2f} dx_back={c0[0] - c7[0]:.2f}",
        )
    ctx.check("two cup holder armrest ends", count(seat, "cup_holder_") == 2)

    # ------------------------------------------- bench is mounted, not floating
    ctx.expect_contact(
        crossbar,
        arms["link_arm_front_left"],
        elem_a="lug_left_inner",
        elem_b="pivot_bolt",
        name="pivot bolt is seated in the crossbar lug",
    )
    ctx.expect_contact(
        arms["link_arm_front_left"],
        arms["link_arm_rear_left"],
        elem_a="arm_tube",
        elem_b="arm_tube",
        name="V-pair hanger tubes meet at the shared pivot yoke",
    )
    ctx.expect_contact(
        arms["link_arm_front_left"],
        seat,
        elem_a="anchor_bolt",
        elem_b="armrest_left",
        name="link arm is bolted to the armrest",
    )
    ctx.expect_contact(
        crossbar,
        af_l,
        elem_a="bar",
        elem_b="apex_sleeve",
        name="crossbar is seated in the apex sleeve",
    )
    ctx.expect_contact(
        back,
        seat,
        elem_a="back_rib_center_0",
        elem_b="rail_rear",
        name="curved back is mounted on the rear seat rail",
    )

    # --------------------------- swing clearance through the whole motion range
    bar_bb = ctx.part_world_aabb(crossbar)
    bar_zmin = bar_bb[0][2] if bar_bb is not None else APEX_Z - BAR_R
    centers = {}
    for q in (-SWING_LIMIT, 0.0, SWING_LIMIT):
        with ctx.pose({drive: q}):
            zmin, ymax = math.inf, 0.0
            for part in (seat, back, *arms.values()):
                bb = ctx.part_world_aabb(part)
                if bb is None:
                    continue
                zmin = min(zmin, bb[0][2])
                ymax = max(ymax, abs(bb[0][1]), abs(bb[1][1]))
            sb = ctx.part_world_aabb(seat)
            kb = ctx.part_world_aabb(back)
            centers[q] = (sb[0][0] + sb[1][0]) / 2.0 if sb is not None else 0.0
            ctx.check(
                f"bench clears the ground at q={q:+.2f}",
                zmin > 0.05,
                details=f"zmin={zmin:.3f}",
            )
            if sb is not None and kb is not None:
                bench_zmax = max(sb[1][2], kb[1][2])
                ctx.check(
                    f"bench hangs below the crossbar at q={q:+.2f}",
                    bench_zmax < bar_zmin - 0.05,
                    details=f"bench_zmax={bench_zmax:.3f} bar_zmin={bar_zmin:.3f}",
                )
            ctx.check(
                f"bench clears the A-frame legs sideways at q={q:+.2f}",
                ymax < FRAME_Y - LEG_R - 0.015,
                details=f"|y|max={ymax:.3f} leg_plane_inner={FRAME_Y - LEG_R:.3f}",
            )
    ctx.check(
        "driving the pivot actually swings the bench fore/aft",
        (centers[-SWING_LIMIT] - centers[SWING_LIMIT]) > 0.6,
        details=f"dx={centers[-SWING_LIMIT] - centers[SWING_LIMIT]:.3f}",
    )

    # Seat at sitting height at rest.
    sb = ctx.part_element_world_aabb(seat, elem="seat_slat_0")
    if sb is not None:
        ctx.check(
            "seat surface at sitting height (~0.66 m)",
            0.58 < sb[1][2] < 0.72,
            details=f"seat_top={sb[1][2]:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
