from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Animal-shaped toddler seesaw (horse theme)
#
# World frame: beam runs along X, pivot axis along Y, Z up.
# - A-frame base: two arched tube legs + rubber ground pads under each foot
# - Horse-shaped beam: rounded torso body, neck+head at +X, tail at -X,
#   four decorative legs embedded in body underside, seats, handles,
#   textured footrests on body sides
# - Rubber bumpers under each end on prismatic compression joints
# - Central revolute pivot, +/- 20 degrees from level
# ---------------------------------------------------------------------------

PIVOT_Z = 0.52            # pivot height

ARCH_FOOT_X = 0.44
ARCH_FOOT_Y = 0.26
ARCH_APEX_Y = 0.035
ARCH_FOOT_Z = 0.018
TUBE_R = 0.022

AXLE_R = 0.013
AXLE_LEN = 0.17

# Beam-local frame: origin at axle center
BODY_LEN = 1.20
BODY_W = 0.22
BODY_H = 0.16
BODY_BOT = 0.04           # body bottom above axle
BODY_CTR = BODY_BOT + BODY_H / 2.0  # 0.12
BODY_TOP = BODY_BOT + BODY_H        # 0.20

SEAT_X = 0.42
HANDLE_X = 0.28
BUMPER_X = 0.55           # under the body where there is mounting surface
FOOTREST_X = 0.42
TILT = math.radians(20.0)
BUMPER_TRAVEL = 0.025

# Ground pad
PAD_W = 0.14
PAD_L = 0.14
PAD_T = 0.012


def _arch_points(side: float) -> list[tuple[float, float, float]]:
    pts: list[tuple[float, float, float]] = []
    rise = PIVOT_Z - ARCH_FOOT_Z
    for i in range(11):
        t = -1.0 + 0.2 * i
        s = 1.0 - t * t
        x = ARCH_FOOT_X * t
        z = ARCH_FOOT_Z + rise * s
        y = side * ARCH_FOOT_Y + (-side * ARCH_APEX_Y - side * ARCH_FOOT_Y) * s
        pts.append((x, y, z))
    return pts


def _handle_points(x: float) -> list[tuple[float, float, float]]:
    half_w = 0.030
    leg_bot = BODY_TOP - 0.008
    arc_z = BODY_TOP + 0.20
    pts: list[tuple[float, float, float]] = [
        (x, -half_w, leg_bot),
        (x, -half_w, arc_z - 0.06),
    ]
    for k in range(7):
        a = math.pi * k / 6.0
        pts.append((x, -half_w * math.cos(a), arc_z + half_w * math.sin(a)))
    pts.append((x, half_w, arc_z - 0.06))
    pts.append((x, half_w, leg_bot))
    return pts


def _horse_body_cq() -> object:
    """CadQuery horse body in beam-local frame (origin at axle center).
    
    The body is a single unified solid that reads as a stylized horse.
    """
    # Main torso
    torso = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, BODY_CTR))
        .box(BODY_LEN, BODY_W, BODY_H)
        .edges("|Z").fillet(0.050)
    )

    # Neck rising from +X end
    neck = (
        cq.Workplane("XY")
        .transformed(offset=(0.54, 0.0, BODY_TOP + 0.05))
        .box(0.14, 0.14, 0.18)
        .edges("|Z").fillet(0.035)
    )

    # Head extending forward from neck top
    head = (
        cq.Workplane("XY")
        .transformed(offset=(0.70, 0.0, BODY_TOP + 0.16))
        .box(0.30, 0.12, 0.11)
        .edges("|Z").fillet(0.030)
    )

    # Snout
    snout = (
        cq.Workplane("XY")
        .transformed(offset=(0.88, 0.0, BODY_TOP + 0.13))
        .box(0.10, 0.10, 0.08)
        .edges("|Z").fillet(0.020)
    )

    # Ears (lowered to overlap with head top for CadQuery fusion)
    ear_l = (
        cq.Workplane("XY")
        .transformed(offset=(0.62, 0.05, BODY_TOP + 0.21))
        .box(0.05, 0.028, 0.08)
        .edges("|Z").fillet(0.008)
    )
    ear_r = (
        cq.Workplane("XY")
        .transformed(offset=(0.62, -0.05, BODY_TOP + 0.21))
        .box(0.05, 0.028, 0.08)
        .edges("|Z").fillet(0.008)
    )

    # Tail
    tail = (
        cq.Workplane("XY")
        .transformed(offset=(-0.66, 0.0, BODY_CTR + 0.03))
        .box(0.18, 0.05, 0.08)
        .edges("|Z").fillet(0.015)
    )
    tail_tip = (
        cq.Workplane("XY")
        .transformed(offset=(-0.80, 0.0, BODY_CTR - 0.02))
        .box(0.12, 0.04, 0.06)
        .edges("|Z").fillet(0.012)
    )

    result = torso.union(neck).union(head).union(snout)
    result = result.union(ear_l).union(ear_r)
    result = result.union(tail).union(tail_tip)

    # Four stubby decorative legs - small bumps on body underside
    # (they do NOT extend below BODY_BOT so bumpers remain the lowest feature)
    for lx, ly in [(0.36, 0.08), (0.36, -0.08), (-0.36, 0.08), (-0.36, -0.08)]:
        leg = (
            cq.Workplane("XY")
            .transformed(offset=(lx, ly, BODY_BOT + 0.015))
            .box(0.07, 0.07, 0.06)
            .edges("|Z").fillet(0.018)
        )
        result = result.union(leg)

    return result


def _footrest_geometry(side_x: float, side_y: float, index: int):
    """Textured footrest: vertical plate on body side."""
    plate_w = 0.10   # along X
    plate_h = 0.07   # along Z (height)
    plate_t = 0.008  # thickness along Y
    # Profile in XY: width x thickness, extrude along Z by height
    profile = [
        (-plate_w / 2, -plate_t / 2),
        (plate_w / 2, -plate_t / 2),
        (plate_w / 2, plate_t / 2),
        (-plate_w / 2, plate_t / 2),
    ]
    geom = ExtrudeGeometry(profile, plate_h, cap=True, center=True)
    # Now geometry is: X wide, Y thin, Z tall
    # Rotate 90° around X to swap Y→Z and Z→-Y:
    # After rotation: X wide, Y tall, Z thin
    # Actually we want X wide, Z tall, Y thin.
    # rotate_x(-90°): Y→Z, Z→-Y → result: X wide, Z from old Y (thin), Y from old -Z (tall)
    # That's wrong. Let me use rotate_x(90°): Y→-Z, Z→Y
    # Result: X wide, Y from old Z (tall), Z from old -Y (thin)
    # That gives: X wide, Y tall, Z thin → not what I want.
    # 
    # I want: X wide (0.10), Y thin (0.008), Z tall (0.07)
    # Current after extrude: X=0.10, Y=0.008, Z=0.07
    # That's already correct! No rotation needed.
    
    # Embed plate 3mm into the body surface
    y_pos = side_y * (BODY_W / 2.0 - 0.003 + plate_t / 2.0)
    z_pos = BODY_BOT + BODY_H * 0.25  # quarter height on body side
    geom.translate(side_x, y_pos, z_pos)
    return mesh_from_geometry(geom, f"footrest_plate_{index}")


def _bumper_geometry(index: int):
    """Rubber bumper: rounded disc with mounting stem."""
    r = 0.050
    t = 0.028
    n = 24
    profile: list[tuple[float, float]] = []
    for k in range(n + 1):
        a = 2.0 * math.pi * k / n
        profile.append((r * math.cos(a), r * math.sin(a)))
    geom = ExtrudeGeometry(profile, t, cap=True, center=True)
    # Disc in XY, thickness along Z
    # Pad center at z = -0.018 so top at -0.004, bottom at -0.032
    # This clears the body bottom (z=0) by 0.004m gap
    geom.translate(0.0, 0.0, -0.018)
    return mesh_from_geometry(geom, f"bumper_pad_{index}")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="animal_toddler_seesaw")

    # Materials
    galvanized = model.material("galvanized_tube", rgba=(0.58, 0.60, 0.58, 1.0))
    body_green = model.material("horse_body_paint", rgba=(0.25, 0.55, 0.30, 1.0))
    seat_red = model.material("seat_plastic", rgba=(0.75, 0.18, 0.12, 1.0))
    handle_yellow = model.material("handle_paint", rgba=(0.85, 0.75, 0.15, 1.0))
    rubber = model.material("black_rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    rubber_grey = model.material("rubber_pad_grey", rgba=(0.22, 0.22, 0.22, 1.0))
    footrest_teal = model.material("footrest_grip", rgba=(0.15, 0.50, 0.50, 1.0))
    rust = model.material("axle_steel", rgba=(0.45, 0.30, 0.18, 1.0))

    # --------------------------------------------------------------- base ---
    base = model.part("base")
    for i, side in enumerate((1.0, -1.0)):
        base.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _arch_points(side),
                    radius=TUBE_R,
                    samples_per_segment=8,
                    radial_segments=18,
                    cap_ends=True,
                ),
                f"arch_{i}",
            ),
            material=galvanized,
            name=f"arch_{i}",
        )

    # Pivot axle
    base.visual(
        Cylinder(radius=AXLE_R, length=AXLE_LEN),
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rust,
        name="pivot_axle",
    )
    for i, side in enumerate((1.0, -1.0)):
        base.visual(
            Cylinder(radius=0.020, length=0.012),
            origin=Origin(
                xyz=(0.0, side * (AXLE_LEN / 2.0 - 0.005), PIVOT_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=rust,
            name=f"axle_nut_{i}",
        )

    # Rubber ground pads under each arch foot
    pad_idx = 0
    for side_x in (1.0, -1.0):
        for side_y in (1.0, -1.0):
            base.visual(
                Box((PAD_L, PAD_W, PAD_T)),
                origin=Origin(
                    xyz=(side_x * ARCH_FOOT_X, side_y * ARCH_FOOT_Y, PAD_T / 2.0)
                ),
                material=rubber_grey,
                name=f"ground_pad_{pad_idx}",
            )
            pad_idx += 1

    # --------------------------------------------------------------- beam ---
    beam = model.part("beam")

    # Pivot sleeve - short ring around axle (kept short to avoid arch tube overlap)
    beam.visual(
        Cylinder(radius=AXLE_R + 0.004, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rust,
        name="pivot_sleeve",
    )
    # Gusset connecting sleeve to body bottom (narrow in Y, starts above axle)
    gusset_bot = AXLE_R + 0.001  # just above axle top surface
    gusset_top = BODY_BOT + 0.005  # embeds into body bottom
    gusset_h = gusset_top - gusset_bot
    gusset_ctr = (gusset_bot + gusset_top) / 2.0
    beam.visual(
        Box((0.08, 0.016, gusset_h)),
        origin=Origin(xyz=(0.0, 0.0, gusset_ctr)),
        material=body_green,
        name="pivot_gusset",
    )

    # Horse body (CadQuery mesh) - this is the main beam structure
    beam.visual(
        mesh_from_cadquery(_horse_body_cq(), "horse_body"),
        material=body_green,
        name="horse_body",
    )

    # Seats on the horse's back
    for i, side in enumerate((1.0, -1.0)):
        beam.visual(
            Box((0.20, 0.20, 0.016)),
            origin=Origin(xyz=(side * SEAT_X, 0.0, BODY_TOP + 0.008)),
            material=seat_red,
            name=f"seat_{i}",
        )
        # Seat back rim
        beam.visual(
            Box((0.20, 0.016, 0.032)),
            origin=Origin(xyz=(side * SEAT_X, -0.092, BODY_TOP + 0.016 + 0.008)),
            material=seat_red,
            name=f"seat_rim_{i}",
        )

    # Grab handles
    for i, side in enumerate((1.0, -1.0)):
        beam.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _handle_points(side * HANDLE_X),
                    radius=0.008,
                    samples_per_segment=8,
                    radial_segments=16,
                    cap_ends=True,
                ),
                f"handle_{i}",
            ),
            material=handle_yellow,
            name=f"handle_{i}",
        )

    # Textured footrests on body sides near seats
    for i, side in enumerate((1.0, -1.0)):
        for j, sy in enumerate((1.0, -1.0)):
            idx = i * 2 + j
            beam.visual(
                _footrest_geometry(side * FOOTREST_X, sy, idx),
                material=footrest_teal,
                name=f"footrest_{idx}",
            )
        # Grip ridges on each footrest side (embedded into body surface)
        footrest_z = BODY_BOT + BODY_H * 0.25  # match footrest plate z
        for sy in (1.0, -1.0):
            y_base = sy * (BODY_W / 2.0 - 0.002 + 0.003)
            for bx in (-0.030, -0.010, 0.010, 0.030):
                beam.visual(
                    Box((0.006, 0.006, 0.050)),
                    origin=Origin(
                        xyz=(side * FOOTREST_X + bx, y_base, footrest_z)
                    ),
                    material=footrest_teal,
                    name=f"footrest_ridge_{i}_{sy}_{bx}",
                )

    # -------------------------------------------------------- bumpers ---
    bumpers = []
    for i, side in enumerate((1.0, -1.0)):
        bumper = model.part(f"bumper_{i}")
        bumper.visual(
            _bumper_geometry(i),
            material=rubber,
            name=f"bumper_pad_{i}",
        )
        # Mounting stem - extends from above body bottom down into pad
        # Stem: from z=-0.008 to z=0.028 (length=0.036), overlaps pad top at z=-0.004
        stem_h = 0.036
        stem_ctr = 0.010
        bumper.visual(
            Cylinder(radius=0.016, length=stem_h),
            origin=Origin(xyz=(0.0, 0.0, stem_ctr)),
            material=rubber,
            name=f"bumper_stem_{i}",
        )
        bumpers.append(bumper)

    # ------------------------------------------------------- joints ---
    model.articulation(
        "beam_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0, velocity=2.0, lower=-TILT, upper=TILT
        ),
    )

    for i, side in enumerate((1.0, -1.0)):
        # Bumper joint origin at body bottom surface
        model.articulation(
            f"bumper_{i}_compress",
            ArticulationType.PRISMATIC,
            parent=beam,
            child=bumpers[i],
            origin=Origin(xyz=(side * BUMPER_X, 0.0, BODY_BOT)),
            axis=(0.0, 0.0, 1.0),  # positive q compresses upward (into body)
            motion_limits=MotionLimits(
                effort=200.0,
                velocity=0.5,
                lower=0.0,
                upper=BUMPER_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    beam = object_model.get_part("beam")
    bumper_0 = object_model.get_part("bumper_0")
    bumper_1 = object_model.get_part("bumper_1")
    pivot = object_model.get_articulation("beam_pivot")
    bump0_joint = object_model.get_articulation("bumper_0_compress")
    bump1_joint = object_model.get_articulation("bumper_1_compress")

    # --- Pivot sleeve captures axle ---
    ctx.allow_overlap(
        beam,
        base,
        elem_a="pivot_sleeve",
        elem_b="pivot_axle",
        reason="Pivot sleeve is a bushing nested around the axle bolt.",
    )
    ctx.expect_contact(
        beam,
        base,
        elem_a="pivot_sleeve",
        elem_b="pivot_axle",
        name="pivot sleeve seated on axle",
    )

    # --- Bumper stems overlap the horse body bottom (mounting contact) ---
    ctx.allow_overlap(
        beam,
        bumper_0,
        elem_a="horse_body",
        elem_b="bumper_stem_0",
        reason="Bumper stem is seated into the body bottom as a mounting insert.",
    )
    ctx.allow_overlap(
        beam,
        bumper_1,
        elem_a="horse_body",
        elem_b="bumper_stem_1",
        reason="Bumper stem is seated into the body bottom as a mounting insert.",
    )
    ctx.expect_contact(
        beam,
        bumper_0,
        elem_a="horse_body",
        elem_b="bumper_stem_0",
        name="bumper_0 stem contacts horse body",
    )
    ctx.expect_contact(
        beam,
        bumper_1,
        elem_a="horse_body",
        elem_b="bumper_stem_1",
        name="bumper_1 stem contacts horse body",
    )

    # --- Ground pads present ---
    for i in range(4):
        pad_box = ctx.part_element_world_aabb(base, elem=f"ground_pad_{i}")
        ctx.check(
            f"ground_pad_{i} near ground",
            pad_box is not None and pad_box[0][2] >= -0.002 and pad_box[1][2] < 0.03,
            details=f"ground_pad_{i} aabb={pad_box}",
        )

    # --- Horse body spans beam length ---
    horse_box = ctx.part_element_world_aabb(beam, elem="horse_body")
    ctx.check(
        "horse body spans the beam",
        horse_box is not None and (horse_box[1][0] - horse_box[0][0]) > 1.2,
        details=f"horse_body aabb={horse_box}",
    )

    # --- Seats on top of body ---
    for i in range(2):
        seat = ctx.part_element_world_aabb(beam, elem=f"seat_{i}")
        ctx.check(
            f"seat_{i} on top of body",
            seat is not None
            and horse_box is not None
            and seat[0][2] > horse_box[0][2] + BODY_H * 0.4,
            details=f"seat aabb={seat}",
        )

    # --- Footrests present ---
    for i in range(4):
        fr = ctx.part_element_world_aabb(beam, elem=f"footrest_{i}")
        ctx.check(
            f"footrest_{i} exists on body side",
            fr is not None and fr[0][2] > -0.2 and fr[1][2] < PIVOT_Z + 0.4,
            details=f"footrest_{i} aabb={fr}",
        )

    # --- Pivot joint: horizontal Y axis, +/- 20 degrees ---
    ax = pivot.axis
    ctx.check(
        "pivot axis is horizontal Y",
        abs(ax[0]) < 1e-9 and abs(ax[1] - 1.0) < 1e-9 and abs(ax[2]) < 1e-9,
        details=f"axis={ax}",
    )
    lim = pivot.motion_limits
    ctx.check(
        "rocking limits +/- 20 degrees",
        lim is not None
        and lim.lower is not None
        and lim.upper is not None
        and abs(lim.lower + TILT) < 1e-6
        and abs(lim.upper - TILT) < 1e-6,
        details=f"limits=({lim.lower}, {lim.upper})",
    )

    # --- Bumper prismatic joints ---
    for i, jnt in enumerate((bump0_joint, bump1_joint)):
        jtype = jnt.articulation_type
        ctx.check(
            f"bumper_{i}_compress is prismatic",
            str(jtype) == "ArticulationType.PRISMATIC" or jtype == ArticulationType.PRISMATIC,
            details=f"type={jtype}",
        )
        jlim = jnt.motion_limits
        ctx.check(
            f"bumper_{i}_compress has travel {BUMPER_TRAVEL}m",
            jlim is not None
            and jlim.lower is not None
            and jlim.upper is not None
            and abs(jlim.lower) < 1e-6
            and abs(jlim.upper - BUMPER_TRAVEL) < 1e-6,
            details=f"limits=({jlim.lower}, {jlim.upper})",
        )

    # --- Bumper pads below horse body ---
    for i in range(2):
        bp = ctx.part_element_world_aabb(
            object_model.get_part(f"bumper_{i}"), elem=f"bumper_pad_{i}"
        )
        ctx.check(
            f"bumper_pad_{i} below horse body",
            bp is not None
            and horse_box is not None
            and bp[1][2] < horse_box[1][2]
            and bp[0][2] < horse_box[0][2],
            details=f"bumper_pad aabb={bp}, horse aabb={horse_box}",
        )

    # --- Rocking pose ---
    rest_b0 = ctx.part_element_world_aabb(bumper_0, elem="bumper_pad_0")
    with ctx.pose({pivot: TILT}):
        down_b0 = ctx.part_element_world_aabb(bumper_0, elem="bumper_pad_0")
        up_b1 = ctx.part_element_world_aabb(bumper_1, elem="bumper_pad_1")
        ctx.check(
            "positive rock lowers +X end",
            rest_b0 is not None
            and down_b0 is not None
            and down_b0[0][2] < rest_b0[0][2] - 0.15,
            details=f"rest={rest_b0}, tilted={down_b0}",
        )
        ctx.check(
            "positive rock raises -X end",
            up_b1 is not None and up_b1[0][2] > 0.5,
            details=f"raised bumper aabb={up_b1}",
        )

    # --- Bumper compression pose ---
    rest_bp0 = ctx.part_element_world_aabb(bumper_0, elem="bumper_pad_0")
    with ctx.pose({bump0_joint: BUMPER_TRAVEL}):
        comp_bp0 = ctx.part_element_world_aabb(bumper_0, elem="bumper_pad_0")
        ctx.check(
            "bumper_0 compression moves pad upward",
            rest_bp0 is not None
            and comp_bp0 is not None
            and comp_bp0[0][2] > rest_bp0[0][2] + 0.010,
            details=f"rest={rest_bp0}, compressed={comp_bp0}",
        )

    # --- Base on ground ---
    base_box = ctx.part_world_aabb(base)
    ctx.check(
        "base on ground",
        base_box is not None and -0.01 <= base_box[0][2] <= 0.02,
        details=f"base aabb={base_box}",
    )

    return ctx.report()


object_model = build_object_model()
