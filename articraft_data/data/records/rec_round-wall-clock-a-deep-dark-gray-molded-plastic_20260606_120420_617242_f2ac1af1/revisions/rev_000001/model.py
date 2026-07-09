from __future__ import annotations

# Round wall clock with three rotating hands.
#
# Frame convention:
#   The dial faces +Z; the hands rotate about the central +Z axis (the dial
#   normal). Diameter ~0.30 m, case depth ~0.06 m.
#
# Root part = fixed case: dark-gray molded bezel ring + back plate, white dial
# with printed numerals and tick marks, center hub, and a translucent domed
# glass cover (all fixed).
# Three child parts = hour, minute, and second hands. Each is a REVOLUTE joint
# about the central +Z axis with continuous full-circle travel, offset in depth
# so the hands never intersect.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# --- Dimensions (meters) -----------------------------------------------------
R_OUT = 0.155        # outer bezel radius -> ~0.31 m diameter
R_DIAL = 0.135       # white dial radius
CASE_DEPTH = 0.050   # bezel rim depth
BACK_T = 0.010
DIAL_Z = 0.014       # dial front surface height
DIAL_T = 0.006

# Hand depths (above the dial, below the glass), offset so they don't intersect.
HUB_Z = DIAL_Z + DIAL_T          # 0.020
HOUR_Z = HUB_Z + 0.004           # 0.024
MIN_Z = HUB_Z + 0.009            # 0.029
SEC_Z = HUB_Z + 0.014            # 0.034
HANDS_TOP = SEC_Z + 0.002        # highest hand surface

# Domed clear-glass cover: a spherical cap sitting just above the hands.
DOME_BASE_R = R_DIAL + 0.004      # base radius of the cap
DOME_RISE = 0.026                 # how far the dome bulges up
DOME_BOTTOM = HANDS_TOP + 0.004   # cap base plane (above the hands)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="round_wall_clock")
    model.material("case_gray", rgba=(0.22, 0.23, 0.25, 1.0))
    model.material("dial_white", rgba=(0.96, 0.96, 0.94, 1.0))
    model.material("numeral", rgba=(0.30, 0.33, 0.36, 1.0))
    model.material("hand_gray", rgba=(0.28, 0.30, 0.33, 1.0))
    model.material("hand_red", rgba=(0.82, 0.13, 0.12, 1.0))
    model.material("glass", rgba=(0.80, 0.85, 0.90, 0.28))

    case = model.part("clock_case")

    # Back plate (closes the rear of the case).
    case.visual(
        Cylinder(radius=R_OUT, length=BACK_T),
        origin=Origin(xyz=(0.0, 0.0, BACK_T / 2.0)),
        material="case_gray",
        name="back_plate",
    )

    # Molded bezel ring (deep dark-gray rim). Built from a ring of segments so
    # the front stays open for the dial and glass; modeled as an annular wall
    # using many tapered wedge boxes would be heavy, so use a thick-walled
    # cylinder approximated by an outer rim built from short arc blocks.
    n_rim = 48
    rim_h = CASE_DEPTH - BACK_T
    rim_mid_r = (R_OUT + R_DIAL) / 2.0 + 0.004
    rim_w = (R_OUT - R_DIAL) + 0.008  # radial wall thickness
    seg_len = 2.0 * math.pi * rim_mid_r / n_rim * 1.25
    for i in range(n_rim):
        ang = 2.0 * math.pi * i / n_rim
        cx = rim_mid_r * math.cos(ang)
        cy = rim_mid_r * math.sin(ang)
        case.visual(
            Box((rim_w, seg_len, rim_h)),
            origin=Origin(xyz=(cx, cy, BACK_T + rim_h / 2.0), rpy=(0.0, 0.0, ang)),
            material="case_gray",
            name=f"bezel_seg_{i}",
        )

    # White dial face, inset.
    case.visual(
        Cylinder(radius=R_DIAL, length=DIAL_T),
        origin=Origin(xyz=(0.0, 0.0, DIAL_Z + DIAL_T / 2.0)),
        material="dial_white",
        name="dial_face",
    )

    # Printed numerals 1..12 as small raised dark blocks near the rim, plus a
    # short tick mark at each hour position. 12 at top (+Y), going clockwise.
    num_r = R_DIAL - 0.020
    tick_r = R_DIAL - 0.006
    for h in range(1, 13):
        # 12 at top, clock runs clockwise: angle measured from +Y going to +X.
        theta = math.pi / 2.0 - (h / 12.0) * 2.0 * math.pi
        nx = num_r * math.cos(theta)
        ny = num_r * math.sin(theta)
        # numeral block; wider for two-digit hours (10,11,12).
        nw = 0.026 if h >= 10 else 0.016
        case.visual(
            Box((nw, 0.020, 0.004)),
            origin=Origin(xyz=(nx, ny, DIAL_Z + DIAL_T + 0.002)),
            material="numeral",
            name=f"numeral_{h}",
        )
        # hour tick mark
        tx = tick_r * math.cos(theta)
        ty = tick_r * math.sin(theta)
        case.visual(
            Box((0.004, 0.012, 0.003), ),
            origin=Origin(xyz=(tx, ty, DIAL_Z + DIAL_T + 0.0015), rpy=(0.0, 0.0, theta)),
            material="numeral",
            name=f"tick_{h}",
        )

    # Center hub (the pivot boss the hands mount on). Rises from the dial up to
    # just below the highest hand so all three hands can nest over it.
    hub_len = SEC_Z - (DIAL_Z + DIAL_T) + 0.004
    case.visual(
        Cylinder(radius=0.010, length=hub_len),
        origin=Origin(xyz=(0.0, 0.0, (DIAL_Z + DIAL_T) + hub_len / 2.0)),
        material="hand_gray",
        name="center_hub",
    )

    # Domed clear-glass cover: a true spherical cap (revolved) authored in
    # CadQuery, sitting just above the hands and bulging past the bezel front.
    # Sphere radius from cap base radius and rise.
    rs = (DOME_BASE_R**2 + DOME_RISE**2) / (2.0 * DOME_RISE)
    # Full sphere centered so its top apex is at z = DOME_RISE and the cap base
    # (the z=0 plane) has radius DOME_BASE_R; then keep only z >= 0 (the cap).
    sphere_center_z = DOME_RISE - rs
    glass_t = 0.003  # glass shell wall thickness
    outer = cq.Workplane("XY").transformed(offset=(0, 0, sphere_center_z)).sphere(rs)
    inner = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, sphere_center_z))
        .sphere(rs - glass_t)
    )
    shell = outer.cut(inner)  # hollow glass shell
    # keep the cap above the base plane z=0
    cap = shell.intersect(
        cq.Workplane("XY").transformed(offset=(0, 0, DOME_RISE)).box(
            2.0 * DOME_BASE_R, 2.0 * DOME_BASE_R, 2.0 * DOME_RISE
        )
    )
    case.visual(
        mesh_from_cadquery(cap, "glass_dome"),
        origin=Origin(xyz=(0.0, 0.0, DOME_BOTTOM)),
        material="glass",
        name="glass_dome",
    )

    # ----------------------------------------------------------------- hands
    # Each hand part frame is at the dial center; the visible bar extends along
    # +Y (12 o'clock) from the center, so positive rotation sweeps clockwise via
    # the chosen axis sign. Hands are offset in Z so they never intersect.
    def _hand(name: str, length: float, width: float, z: float, material: str,
              tail: float) -> None:
        part = model.part(name)
        # main pointer bar (extends +Y from center)
        part.visual(
            Box((width, length, 0.004)),
            origin=Origin(xyz=(0.0, length / 2.0 - tail, z)),
            material=material,
            name=f"{name}_bar",
        )
        # small counterweight tail (extends -Y) so the hand reads balanced
        part.visual(
            Box((width, tail, 0.004)),
            origin=Origin(xyz=(0.0, -tail / 2.0, z)),
            material=material,
            name=f"{name}_tail",
        )
        # boss that sits over the center hub
        part.visual(
            Cylinder(radius=0.009, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=material,
            name=f"{name}_boss",
        )

    _hand("hour_hand", length=0.075, width=0.012, z=HOUR_Z, material="hand_gray", tail=0.018)
    _hand("minute_hand", length=0.115, width=0.009, z=MIN_Z, material="hand_gray", tail=0.020)
    _hand("second_hand", length=0.122, width=0.004, z=SEC_Z, material="hand_red", tail=0.026)

    # ------------------------------------------------------------ articulation
    # All hands pivot about the central +Z axis at the dial center, continuous
    # full-circle travel.
    for name in ("hour_hand", "minute_hand", "second_hand"):
        model.articulation(
            f"case_to_{name}",
            ArticulationType.REVOLUTE,
            parent=case,
            child=name,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=1.0, velocity=10.0, lower=-2.0 * math.pi, upper=2.0 * math.pi
            ),
        )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    case = object_model.get_part("clock_case")
    hour = object_model.get_part("hour_hand")
    minute = object_model.get_part("minute_hand")
    second = object_model.get_part("second_hand")
    j_hour = object_model.get_articulation("case_to_hour_hand")
    j_min = object_model.get_articulation("case_to_minute_hand")
    j_sec = object_model.get_articulation("case_to_second_hand")

    # The hand bosses sit over the center hub; allow that local nesting.
    for hand_name in ("hour_hand", "minute_hand", "second_hand"):
        ctx.allow_overlap(
            case,
            hand_name,
            elem_a="center_hub",
            elem_b=f"{hand_name}_boss",
            reason="Hand boss nests over the center hub pivot.",
        )

    # Hero geometry counts.
    n_num = sum(1 for v in case.visuals if v.name and v.name.startswith("numeral_"))
    ctx.check("dial shows 12 numerals", n_num == 12, details=f"numerals={n_num}")

    n_tick = sum(1 for v in case.visuals if v.name and v.name.startswith("tick_"))
    ctx.check("dial has hour tick marks", n_tick == 12, details=f"ticks={n_tick}")

    n_bezel = sum(1 for v in case.visuals if v.name and v.name.startswith("bezel_seg_"))
    ctx.check("molded bezel ring present", n_bezel >= 24, details=f"bezel_segs={n_bezel}")

    ctx.check("white dial present", case.get_visual("dial_face") is not None)
    ctx.check("domed glass cover present", case.get_visual("glass_dome") is not None)
    ctx.check("center hub present", case.get_visual("center_hub") is not None)

    # Real-world scale: ~0.30 m diameter, ~0.06 m deep.
    cb = ctx.part_world_aabb(case)
    if cb is not None:
        (xmn, ymn, zmn), (xmx, ymx, zmx) = cb
        ctx.check(
            "clock is ~0.30 m diameter and ~0.06 m deep",
            0.28 < (xmx - xmn) < 0.34 and 0.04 < (zmx - zmn) < 0.09,
            details=f"dia={xmx - xmn:.3f} depth={zmx - zmn:.3f}",
        )

    # Hand lengths: hour short, minute and second long.
    hb = ctx.part_element_world_aabb(hour, elem="hour_hand_bar")
    mb = ctx.part_element_world_aabb(minute, elem="minute_hand_bar")
    sb = ctx.part_element_world_aabb(second, elem="second_hand_bar")
    if hb is not None and mb is not None and sb is not None:
        hour_reach = hb[1][1]
        min_reach = mb[1][1]
        sec_reach = sb[1][1]
        ctx.check(
            "hour hand shorter than minute hand",
            hour_reach < min_reach - 0.02,
            details=f"hour={hour_reach:.3f} minute={min_reach:.3f}",
        )
        ctx.check(
            "second hand is long",
            sec_reach > 0.09,
            details=f"second={sec_reach:.3f}",
        )

    # Hands are stacked in depth so they don't intersect.
    ctx.check(
        "hands are offset in depth",
        HOUR_Z < MIN_Z < SEC_Z,
        details=f"hour_z={HOUR_Z:.3f} min_z={MIN_Z:.3f} sec_z={SEC_Z:.3f}",
    )

    # Each hand rotates about center: a 90-degree pose swings the tip from +Y
    # toward +X (clockwise) without the part origin moving.
    def _tip(part):
        a = ctx.part_world_aabb(part)
        return None if a is None else a

    rest_tip = ctx.part_world_aabb(minute)
    with ctx.pose({j_min: math.pi / 2.0}):
        turned_tip = ctx.part_world_aabb(minute)
    if rest_tip is not None and turned_tip is not None:
        # at rest the bar reaches far in +Y; after a quarter turn it reaches in X.
        rest_y = rest_tip[1][1]
        turned_x = max(abs(turned_tip[0][0]), abs(turned_tip[1][0]))
        ctx.check(
            "minute hand rotates about the center",
            turned_x > 0.08 and rest_y > 0.08,
            details=f"rest_y={rest_y:.3f} turned_x={turned_x:.3f}",
        )

    # Sanity: all three hand joints share the central axis and origin.
    for j in (j_hour, j_min, j_sec):
        ctx.check(
            f"{j.name} pivots at center about Z",
            tuple(j.axis) == (0.0, 0.0, 1.0)
            and abs(j.origin.xyz[0]) < 1e-9
            and abs(j.origin.xyz[1]) < 1e-9,
            details=f"axis={j.axis} origin={j.origin.xyz}",
        )

    return ctx.report()
