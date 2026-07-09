from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)

# ---------------------------------------------------------------------------
# Commercial playground swing set, glossy red steel A-frame.
#   X = width (3.0 m top rail), Y = depth (1.6 m at the feet), Z = up (2.4 m).
# Root part: red A-frame (legs, braces, bolt-hole base plates, top rail) plus
# four galvanized clevis hangers with pins. Two rigid pendulum swing parts
# (rod pair + sagged rubber belt seat) each hang on a revolute joint whose
# axis runs along the rail through the hanger pins.
# Suspension variant: rigid straight steel rods replace link chains.
# ---------------------------------------------------------------------------

RED = Material(name="glossy_red_steel", rgba=(0.72, 0.045, 0.06, 1.0))
GALV = Material(name="galvanized_steel", rgba=(0.74, 0.76, 0.79, 1.0))
RUBBER = Material(name="black_rubber", rgba=(0.06, 0.06, 0.065, 1.0))

RAIL_Z = 2.33  # top rail centerline height
RAIL_SIZE = (3.0, 0.08, 0.08)  # rail top ends at 2.37 -> ~2.4 m overall
APEX_X = 1.38  # A-frame end planes
FOOT_Y = 0.76  # leg foot centers -> ~1.6 m deep at the feet
LEG_TUBE = 0.06  # square tube
PLATE_T = 0.014  # base plate thickness
APEX_Z = 2.31  # leg top end center (embedded in rail)
FOOT_Z = 0.020  # leg bottom end center (embedded in plate)

PIVOT_Z = 2.228  # hanger pin axis = swing revolute axis height
SWING_X = 0.70  # swing centers at +/-0.70
ROD_DX = 0.22  # rod half-spacing on each swing
HANGER_XS = (
    -SWING_X - ROD_DX,
    -SWING_X + ROD_DX,
    SWING_X - ROD_DX,
    SWING_X + ROD_DX,
)

PIN_R = 0.004

# Suspension rod: straight steel tube from pin down to seat clamp
ROD_RADIUS = 0.010  # 20 mm diameter rod

# Clamp and seat (same vertical layout as the chain parent)
CLAMP_Z = -1.505  # clamp center in swing local frame (z=0 at pivot)
CLAMP_HALF_H = 0.020
SEAT_END_Z = CLAMP_Z - 0.010
SEAT_SAG = 0.113  # belt bottom dips below the clamped ends

# Rod spans from pin centerline (z=0) down to clamp top face
ROD_BOTTOM_Z = CLAMP_Z + CLAMP_HALF_H  # -1.485
ROD_LENGTH = abs(ROD_BOTTOM_Z)  # 1.485 m
ROD_CENTER_Z = ROD_BOTTOM_Z / 2.0  # -0.7425


def _base_plate_geometry():
    """Flat rectangular base plate with four bolt holes."""
    outer = rounded_rect_profile(0.20, 0.16, 0.02)
    holes = []
    for hx in (-0.075, 0.075):
        for hy in (-0.058, 0.058):
            holes.append(
                [
                    (hx + 0.007 * math.cos(t), hy + 0.007 * math.sin(t))
                    for t in [2.0 * math.pi * i / 16 for i in range(16)]
                ]
            )
    return ExtrudeWithHolesGeometry(outer, holes, PLATE_T, cap=True, center=True)


def _seat_belt_geometry():
    """Flexible belt seat sagged into a shallow U, swept along X."""
    path = [
        (-ROD_DX, 0.0, SEAT_END_Z),
        (-0.185, 0.0, SEAT_END_Z - 0.063),
        (-0.10, 0.0, SEAT_END_Z - 0.102),
        (0.0, 0.0, SEAT_END_Z - SEAT_SAG),
        (0.10, 0.0, SEAT_END_Z - 0.102),
        (0.185, 0.0, SEAT_END_Z - 0.063),
        (ROD_DX, 0.0, SEAT_END_Z),
    ]
    return sweep_profile_along_spline(
        path,
        profile=rounded_rect_profile(0.14, 0.012, 0.004),
        samples_per_segment=8,
        cap_profile=True,
    )


def _add_a_frame_end(frame, end_index: int, x_end: float, plate_mesh) -> None:
    leg_len = math.hypot(FOOT_Y, APEX_Z - FOOT_Z)
    tilt = math.atan2(FOOT_Y, APEX_Z - FOOT_Z)
    for leg_index, sy in enumerate((-1.0, 1.0)):
        frame.visual(
            Box((LEG_TUBE, LEG_TUBE, leg_len)),
            origin=Origin(
                xyz=(x_end, sy * FOOT_Y / 2.0, (APEX_Z + FOOT_Z) / 2.0),
                rpy=(sy * tilt, 0.0, 0.0),
            ),
            material=RED,
            name=f"end_{end_index}_leg_{leg_index}",
        )
        frame.visual(
            plate_mesh,
            origin=Origin(xyz=(x_end, sy * FOOT_Y, PLATE_T / 2.0)),
            material=RED,
            name=f"base_plate_{end_index}_{leg_index}",
        )
    # Two horizontal cross braces partway down, spanning between the legs.
    for label, brace_z in (("upper", 1.35), ("lower", 0.60)):
        y_leg = FOOT_Y * (APEX_Z - brace_z) / (APEX_Z - FOOT_Z)
        frame.visual(
            Box((0.04, 2.0 * y_leg + 0.06, 0.04)),
            origin=Origin(xyz=(x_end, 0.0, brace_z)),
            material=RED,
            name=f"end_{end_index}_{label}_brace",
        )


def _add_hanger(frame, index: int, x_h: float) -> None:
    """Galvanized clevis hanger bolted under the rail, with a transverse pin."""
    frame.visual(
        Box((0.05, 0.09, 0.012)),
        origin=Origin(xyz=(x_h, 0.0, 2.287)),
        material=GALV,
        name=f"hanger_mount_{index}",
    )
    frame.visual(
        Box((0.034, 0.022, 0.018)),
        origin=Origin(xyz=(x_h, 0.0, 2.274)),
        material=GALV,
        name=f"hanger_body_{index}",
    )
    for side, sx in enumerate((-1.0, 1.0)):
        frame.visual(
            Box((0.006, 0.022, 0.046)),
            origin=Origin(xyz=(x_h + sx * 0.0145, 0.0, 2.245)),
            material=GALV,
            name=f"hanger_cheek_{index}_{side}",
        )
    frame.visual(
        Cylinder(radius=PIN_R, length=0.040),
        origin=Origin(xyz=(x_h, 0.0, PIVOT_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=GALV,
        name=f"hanger_pin_{index}",
    )


def _add_swing(model, frame, index: int, x_center: float, belt_mesh) -> None:
    """One rigid pendulum: two straight suspension rods + clamps + sagged belt."""
    swing = model.part(f"swing_{index}")
    for rod_i, sx in enumerate((-1.0, 1.0)):
        rx = sx * ROD_DX
        # Straight steel rod from pin level down to clamp top
        swing.visual(
            Cylinder(radius=ROD_RADIUS, length=ROD_LENGTH),
            origin=Origin(xyz=(rx, 0.0, ROD_CENTER_Z)),
            material=GALV,
            name=f"rod_{rod_i}",
        )
        # Red clamp plate where rod meets seat
        swing.visual(
            Box((0.05, 0.09, 0.04)),
            origin=Origin(xyz=(rx, 0.0, CLAMP_Z)),
            material=RED,
            name=f"clamp_{rod_i}",
        )
    swing.visual(belt_mesh, origin=Origin(), material=RUBBER, name="seat_belt")

    model.articulation(
        f"swing_{index}_pivot",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=swing,
        origin=Origin(xyz=(x_center, 0.0, PIVOT_Z)),
        # Axis horizontal along the rail; positive q swings the seat to +Y.
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=200.0, velocity=4.0, lower=-1.0, upper=1.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_playground_swing_set")

    frame = model.part("frame")
    frame.visual(
        Box(RAIL_SIZE),
        origin=Origin(xyz=(0.0, 0.0, RAIL_Z)),
        material=RED,
        name="top_rail",
    )

    plate_mesh = mesh_from_geometry(_base_plate_geometry(), "base_plate")
    for end_index, x_end in enumerate((-APEX_X, APEX_X)):
        _add_a_frame_end(frame, end_index, x_end, plate_mesh)

    for index, x_h in enumerate(HANGER_XS):
        _add_hanger(frame, index, x_h)

    belt_mesh = mesh_from_geometry(_seat_belt_geometry(), "seat_belt")
    for i, x_center in enumerate((-SWING_X, SWING_X)):
        _add_swing(model, frame, i, x_center, belt_mesh)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    swing_0 = object_model.get_part("swing_0")
    swing_1 = object_model.get_part("swing_1")
    pivot_0 = object_model.get_articulation("swing_0_pivot")
    pivot_1 = object_model.get_articulation("swing_1_pivot")

    # --- Captured-pin bearing: each rod top embeds onto its clevis pin.
    # swing_0: rod_0 → hanger_pin_0, rod_1 → hanger_pin_1
    # swing_1: rod_0 → hanger_pin_2, rod_1 → hanger_pin_3
    hanger_pin_for_swing = {0: 0, 1: 2}  # first pin index per swing
    for swing, swing_idx in ((swing_0, 0), (swing_1, 1)):
        pin_base = hanger_pin_for_swing[swing_idx]
        for rod_i in range(2):
            ctx.allow_overlap(
                frame,
                swing,
                elem_a=f"hanger_pin_{pin_base + rod_i}",
                elem_b=f"rod_{rod_i}",
                reason=(
                    "Rod top reaches the clevis pin; the pin beds into the rod "
                    "cross-section as a captured-pin pivot bearing."
                ),
            )

    # --- Overall envelope: ~3.0 m wide, ~2.4 m tall, ~1.6 m deep at the feet.
    frame_aabb = ctx.part_world_aabb(frame)
    ctx.check(
        "frame is ~3.0 m wide along the rail",
        frame_aabb is not None and 2.9 <= frame_aabb[1][0] - frame_aabb[0][0] <= 3.1,
        details=f"frame aabb={frame_aabb}",
    )
    ctx.check(
        "frame is ~2.4 m tall",
        frame_aabb is not None and 2.30 <= frame_aabb[1][2] <= 2.45,
        details=f"frame aabb={frame_aabb}",
    )
    ctx.check(
        "A-frame feet spread ~1.6-1.7 m deep",
        frame_aabb is not None and 1.5 <= frame_aabb[1][1] - frame_aabb[0][1] <= 1.8,
        details=f"frame aabb={frame_aabb}",
    )
    ctx.check(
        "base plates rest on the ground plane",
        frame_aabb is not None and abs(frame_aabb[0][2]) <= 0.002,
        details=f"frame aabb={frame_aabb}",
    )

    # --- Top rail spans between the A-frame end planes.
    rail_aabb = ctx.part_element_world_aabb(frame, elem="top_rail")
    ctx.check(
        "top rail sits at the apex height and spans both ends",
        rail_aabb is not None
        and rail_aabb[0][0] <= -APEX_X
        and rail_aabb[1][0] >= APEX_X
        and rail_aabb[0][2] >= 2.25,
        details=f"rail aabb={rail_aabb}",
    )

    # --- Rod suspension: each rod is a straight narrow member ~1.5 m long.
    for swing, label in ((swing_0, "swing_0"), (swing_1, "swing_1")):
        for rod_i in range(2):
            rod_aabb = ctx.part_element_world_aabb(swing, elem=f"rod_{rod_i}")
            ctx.check(
                f"{label} rod_{rod_i} is a straight member ~1.4-1.55 m long",
                rod_aabb is not None and 1.40 <= rod_aabb[1][2] - rod_aabb[0][2] <= 1.55,
                details=f"rod aabb={rod_aabb}",
            )
            ctx.check(
                f"{label} rod_{rod_i} is narrow (steel rod, not a bar or chain)",
                rod_aabb is not None
                and rod_aabb[1][0] - rod_aabb[0][0] <= 0.035
                and rod_aabb[1][1] - rod_aabb[0][1] <= 0.035,
                details=f"rod aabb={rod_aabb}",
            )

    # --- Rods reach the pin and carry the seat (contact at top, overlap at clamp).
    for swing, swing_idx in ((swing_0, 0), (swing_1, 1)):
        pin_base = hanger_pin_for_swing[swing_idx]
        for rod_i in range(2):
            ctx.expect_contact(
                swing,
                frame,
                elem_a=f"rod_{rod_i}",
                elem_b=f"hanger_pin_{pin_base + rod_i}",
                name=f"swing_{swing_idx} rod_{rod_i} reaches its hanger pin",
            )

    # --- Belt seat: sagged U, hung at child height, clamped at both rods.
    for swing, label in ((swing_0, "swing_0"), (swing_1, "swing_1")):
        belt_aabb = ctx.part_element_world_aabb(swing, elem="seat_belt")
        clamp0 = ctx.part_element_world_aabb(swing, elem="clamp_0")
        clamp1 = ctx.part_element_world_aabb(swing, elem="clamp_1")
        ctx.check(
            f"{label} belt seat hangs at child seat height",
            belt_aabb is not None and 0.45 <= belt_aabb[0][2] <= 0.75,
            details=f"belt aabb={belt_aabb}",
        )
        ctx.check(
            f"{label} belt sags into a U below its clamped ends",
            belt_aabb is not None
            and clamp0 is not None
            and belt_aabb[0][2] < clamp0[0][2] - 0.05,
            details=f"belt={belt_aabb}, clamp={clamp0}",
        )
        ctx.check(
            f"{label} belt is a wide flexible strap (~0.14 m deep)",
            belt_aabb is not None and 0.10 <= belt_aabb[1][1] - belt_aabb[0][1] <= 0.18,
            details=f"belt aabb={belt_aabb}",
        )
        for clamp_name, clamp_aabb_i in (("clamp_0", clamp0), ("clamp_1", clamp1)):
            ctx.check(
                f"{label} {clamp_name} grips the belt end",
                belt_aabb is not None
                and clamp_aabb_i is not None
                and belt_aabb[0][0] <= clamp_aabb_i[1][0]
                and belt_aabb[1][0] >= clamp_aabb_i[0][0]
                and belt_aabb[1][2] >= clamp_aabb_i[0][2],
                details=f"belt={belt_aabb}, clamp={clamp_aabb_i}",
            )

    # --- Articulation: each swing is an independent fore-aft pendulum.
    belt0_rest = ctx.part_element_world_aabb(swing_0, elem="seat_belt")
    belt1_rest = ctx.part_element_world_aabb(swing_1, elem="seat_belt")
    with ctx.pose({pivot_0: 1.0}):
        belt0_fwd = ctx.part_element_world_aabb(swing_0, elem="seat_belt")
        belt1_still = ctx.part_element_world_aabb(swing_1, elem="seat_belt")
        ctx.check(
            "swing_0 at +1.0 rad swings its seat forward (+Y) and up",
            belt0_rest is not None
            and belt0_fwd is not None
            and (belt0_fwd[0][1] + belt0_fwd[1][1]) / 2.0 > 0.8
            and belt0_fwd[0][2] > belt0_rest[0][2] + 0.3,
            details=f"rest={belt0_rest}, fwd={belt0_fwd}",
        )
        ctx.check(
            "swing_1 stays at rest while swing_0 swings (independent)",
            belt1_rest is not None
            and belt1_still is not None
            and abs((belt1_still[0][1] + belt1_still[1][1]) / 2.0) < 0.02
            and abs(belt1_still[0][2] - belt1_rest[0][2]) < 0.005,
            details=f"rest={belt1_rest}, posed={belt1_still}",
        )
        # Rod stays pinned at its hanger during full swing.
        ctx.expect_contact(
            swing_0,
            frame,
            elem_a="rod_0",
            elem_b="hanger_pin_0",
            name="swing_0 rod_0 stays seated on its pin at +1.0 rad",
        )
    with ctx.pose({pivot_1: -1.0}):
        belt1_back = ctx.part_element_world_aabb(swing_1, elem="seat_belt")
        ctx.check(
            "swing_1 at -1.0 rad swings its seat backward (-Y)",
            belt1_back is not None and (belt1_back[0][1] + belt1_back[1][1]) / 2.0 < -0.8,
            details=f"posed={belt1_back}",
        )
        ctx.expect_contact(
            swing_1,
            frame,
            elem_a="rod_1",
            elem_b="hanger_pin_3",
            name="swing_1 rod_1 stays seated on its pin at -1.0 rad",
        )

    return ctx.report()


object_model = build_object_model()
