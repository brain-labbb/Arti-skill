from __future__ import annotations

# Tall ergonomic office chair (IKEA Markus style), all black.
#
# Kinematic tree:
#   base (root: 6-spoke star base, caster stems/yokes, gas-lift outer tube)
#     -> caster_wheel_0..5   CONTINUOUS about each horizontal axle
#     -> lift_piston         PRISMATIC along Z (gas lift, 0..0.12 m)
#        -> seat             CONTINUOUS swivel about Z (seat pan, armrest loops, mech)
#           -> backrest      REVOLUTE about Y (recline, 0..-0.25 rad)
#           -> tilt_lever    REVOLUTE paddle lever on the mechanism side

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)

# ---------------------------------------------------------------- dimensions
CASTER_COUNT = 6
CASTER_RADIUS_POS = 0.295  # radial distance of each caster axle from the column
WHEEL_RADIUS = 0.030
WHEEL_AXLE_Z = WHEEL_RADIUS  # axle height so wheels touch the floor at z=0
SPOKE_TIP_R = 0.30

LIFT_TUBE_TOP = 0.360  # top of the gas-lift outer tube (prismatic joint plane)
LIFT_TRAVEL = 0.12
PISTON_TOP = 0.062  # piston top above the joint plane -> swivel joint height
SEAT_FRAME_Z = LIFT_TUBE_TOP + PISTON_TOP  # 0.422 m at q_lift = 0

RECLINE_PIVOT = (-0.21, 0.0, -0.02)  # in the seat frame
RECLINE_RANGE = 0.25


def _spoke_mesh(angle: float, index: int):
    """One tapered star-base spoke pointing along +X, then yawed into place."""
    profile = [
        (0.020, -0.034),
        (SPOKE_TIP_R, -0.019),
        (SPOKE_TIP_R, 0.019),
        (0.020, 0.034),
    ]
    geom = ExtrudeGeometry(profile, 0.034, cap=True, center=True)
    geom.rotate_y(0.07)  # outer end drops toward the caster
    geom.translate(0.0, 0.0, 0.115)
    geom.rotate_z(angle)
    return mesh_from_geometry(geom, f"base_spoke_{index}")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="markus_style_office_chair")

    plastic_black = model.material("plastic_black", rgba=(0.10, 0.10, 0.11, 1.0))
    mesh_black = model.material("mesh_black", rgba=(0.05, 0.05, 0.055, 1.0))
    fabric_black = model.material("fabric_black", rgba=(0.085, 0.085, 0.09, 1.0))
    chrome = model.material("chrome", rgba=(0.72, 0.73, 0.75, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.28, 0.29, 0.31, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.06, 0.06, 0.065, 1.0))

    # ------------------------------------------------------------------ base
    base = model.part("base")
    base.visual(
        Cylinder(radius=0.048, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        material=plastic_black,
        name="star_hub",
    )

    caster_angles = [2.0 * math.pi * i / CASTER_COUNT + math.pi / 6.0 for i in range(CASTER_COUNT)]
    for i, ang in enumerate(caster_angles):
        cx = CASTER_RADIUS_POS * math.cos(ang)
        cy = CASTER_RADIUS_POS * math.sin(ang)
        base.visual(
            _spoke_mesh(ang, i),
            material=plastic_black,
            name=f"spoke_{i}",
        )
        # Vertical caster stem dropping from the spoke tip.
        base.visual(
            Cylinder(radius=0.009, length=0.042),
            origin=Origin(xyz=(cx, cy, 0.061)),
            material=plastic_black,
            name=f"caster_stem_{i}",
        )
        # Central yoke hub between the twin wheels; the axle passes through it.
        base.visual(
            Box((0.032, 0.014, 0.026)),
            origin=Origin(xyz=(cx, cy, 0.042), rpy=(0.0, 0.0, ang)),
            material=plastic_black,
            name=f"caster_yoke_{i}",
        )

    # Gas-lift outer tube (dark column) with a thin chrome collar at its top.
    base.visual(
        Cylinder(radius=0.027, length=0.205),
        origin=Origin(xyz=(0.0, 0.0, 0.2575)),
        material=plastic_black,
        name="lift_tube",
    )
    base.visual(
        Cylinder(radius=0.0285, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.351)),
        material=chrome,
        name="lift_collar",
    )

    # --------------------------------------------------------- caster wheels
    # Twin-wheel casters: two black discs on a shared horizontal axle with
    # chrome hub caps. Continuous joint about the local +Y axle.
    for i, ang in enumerate(caster_angles):
        wheel = model.part(f"caster_wheel_{i}")
        for side, sy in (("0", -0.0165), ("1", 0.0165)):
            wheel.visual(
                Cylinder(radius=WHEEL_RADIUS, length=0.013),
                origin=Origin(xyz=(0.0, sy, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=rubber_black,
                name=f"wheel_{side}",
            )
        wheel.visual(
            Cylinder(radius=0.005, length=0.046),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel_dark,
            name="axle",
        )
        for side, sy in (("0", -0.0245), ("1", 0.0245)):
            wheel.visual(
                Cylinder(radius=0.019, length=0.005),
                origin=Origin(xyz=(0.0, sy, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=chrome,
                name=f"hub_cap_{side}",
            )
        model.articulation(
            f"base_to_caster_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=wheel,
            origin=Origin(
                xyz=(
                    CASTER_RADIUS_POS * math.cos(ang),
                    CASTER_RADIUS_POS * math.sin(ang),
                    WHEEL_AXLE_Z,
                ),
                rpy=(0.0, 0.0, ang),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=20.0, velocity=20.0),
        )

    # ------------------------------------------------------------ gas piston
    lift_piston = model.part("lift_piston")
    lift_piston.visual(
        # Extends 0.19 m down inside the outer tube so it stays inserted at
        # full extension (retained insertion >= 0.07 m at q = 0.12).
        Cylinder(radius=0.0165, length=0.252),
        origin=Origin(xyz=(0.0, 0.0, -0.064)),
        material=steel_dark,
        name="piston_rod",
    )
    model.articulation(
        "base_to_lift_piston",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lift_piston,
        origin=Origin(xyz=(0.0, 0.0, LIFT_TUBE_TOP)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.10, lower=0.0, upper=LIFT_TRAVEL),
    )

    # ------------------------------------------------------------------ seat
    seat = model.part("seat")
    seat.visual(
        Box((0.26, 0.20, 0.075)),
        origin=Origin(xyz=(0.01, 0.0, -0.020)),
        material=plastic_black,
        name="mech_housing",
    )
    seat.visual(
        Box((0.090, 0.12, 0.050)),
        origin=Origin(xyz=(-0.150, 0.0, -0.020)),
        material=plastic_black,
        name="mech_rear_bracket",
    )

    pan_geom = ExtrudeGeometry(rounded_rect_profile(0.50, 0.46, 0.10), 0.07, center=True)
    pan_geom.translate(0.01, 0.0, 0.043)
    seat.visual(
        mesh_from_geometry(pan_geom, "seat_pan"),
        material=plastic_black,
        name="seat_pan",
    )
    cushion_geom = ExtrudeGeometry(rounded_rect_profile(0.44, 0.42, 0.09), 0.03, center=True)
    cushion_geom.translate(0.02, 0.0, 0.088)
    seat.visual(
        mesh_from_geometry(cushion_geom, "seat_cushion"),
        material=fabric_black,
        name="seat_cushion",
    )

    # Closed-loop armrests: flattened black plastic rings on angled stems
    # bolted under the seat sides.
    for label, sign in (("left", 1.0), ("right", -1.0)):
        stem_geom = tube_from_spline_points(
            [
                (0.05, sign * 0.205, 0.015),
                (0.03, sign * 0.245, 0.070),
                (0.01, sign * 0.252, 0.130),
            ],
            radius=0.014,
            samples_per_segment=14,
            radial_segments=14,
            cap_ends=True,
        )
        seat.visual(
            mesh_from_geometry(stem_geom, f"armrest_stem_{label}"),
            material=plastic_black,
            name=f"{label}_armrest_stem",
        )
        ring_geom = TorusGeometry(radius=0.085, tube=0.016, radial_segments=14, tubular_segments=40)
        ring_geom.rotate_x(math.pi / 2.0)  # stand the loop in the XZ plane
        ring_geom.scale(1.0, 0.7, 1.0)  # flattened ring profile
        ring_geom.translate(0.01, sign * 0.255, 0.215)
        seat.visual(
            mesh_from_geometry(ring_geom, f"armrest_loop_{label}"),
            material=plastic_black,
            name=f"{label}_armrest_loop",
        )

    model.articulation(
        "lift_piston_to_seat",
        ArticulationType.CONTINUOUS,
        parent=lift_piston,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, PISTON_TOP)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=3.0),
    )

    # ------------------------------------------------------------- tilt lever
    lever = model.part("tilt_lever")
    lever.visual(
        Cylinder(radius=0.006, length=0.070),
        origin=Origin(xyz=(0.0, -0.020, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=plastic_black,
        name="lever_shaft",
    )
    lever.visual(
        Box((0.055, 0.035, 0.010)),
        origin=Origin(xyz=(0.0, -0.065, 0.0)),
        material=plastic_black,
        name="lever_paddle",
    )
    model.articulation(
        "seat_to_tilt_lever",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=lever,
        origin=Origin(xyz=(0.06, -0.10, -0.03)),
        # Negated X axis so positive q flips the paddle tip (at -Y) upward.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=4.0, lower=0.0, upper=0.25),
    )

    # -------------------------------------------------------------- backrest
    backrest = model.part("backrest")
    backrest.visual(
        Cylinder(radius=0.022, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=plastic_black,
        name="pivot_barrel",
    )
    for label, sign in (("left", 1.0), ("right", -1.0)):
        backrest.visual(
            Box((0.090, 0.030, 0.040)),
            origin=Origin(xyz=(-0.045, sign * 0.045, 0.0)),
            material=plastic_black,
            name=f"{label}_pivot_arm",
        )
    backrest.visual(
        Box((0.050, 0.100, 0.180)),
        origin=Origin(xyz=(-0.060, 0.0, 0.070)),
        material=plastic_black,
        name="spine",
    )
    backrest.visual(
        Box((0.050, 0.420, 0.050)),
        origin=Origin(xyz=(-0.020, 0.0, 0.135)),
        material=plastic_black,
        name="bottom_rail",
    )

    # Curved side rails following the lumbar S-curve of the frame.
    for label, sign in (("left", 1.0), ("right", -1.0)):
        rail_geom = tube_from_spline_points(
            [
                (-0.020, sign * 0.210, 0.130),
                (0.005, sign * 0.210, 0.320),
                (-0.030, sign * 0.205, 0.560),
                (-0.090, sign * 0.195, 0.840),
            ],
            radius=0.017,
            samples_per_segment=14,
            radial_segments=14,
            cap_ends=True,
        )
        backrest.visual(
            mesh_from_geometry(rail_geom, f"side_rail_{label}"),
            material=plastic_black,
            name=f"{label}_side_rail",
        )

    backrest.visual(
        Box((0.045, 0.400, 0.050)),
        origin=Origin(xyz=(-0.085, 0.0, 0.845), rpy=(0.0, -0.21, 0.0)),
        material=plastic_black,
        name="top_rail",
    )

    # Dark mesh center panel in three curved segments inside the frame.
    panel_specs = (
        ("mesh_panel_lower", (-0.016, 0.0, 0.245), 0.13, 0.24),
        ("mesh_panel_mid", (-0.0205, 0.0, 0.440), -0.145, 0.27),
        ("mesh_panel_upper", (-0.064, 0.0, 0.700), -0.21, 0.30),
    )
    for name, center, pitch, height in panel_specs:
        backrest.visual(
            Box((0.016, 0.400, height)),
            origin=Origin(xyz=center, rpy=(0.0, pitch, 0.0)),
            material=mesh_black,
            name=name,
        )

    # Padded pillow-like headrest capping the top of the frame.
    headrest_geom = CapsuleGeometry(radius=0.075, length=0.26, radial_segments=20)
    headrest_geom.rotate_x(math.pi / 2.0)  # length along Y (chair width)
    headrest_geom.scale(0.60, 1.0, 0.93)  # flatten into a pillow form
    headrest_geom.rotate_y(-0.21)
    headrest_geom.translate(-0.105, 0.0, 0.900)
    backrest.visual(
        mesh_from_geometry(headrest_geom, "headrest_pillow"),
        material=fabric_black,
        name="headrest_pillow",
    )

    model.articulation(
        "seat_to_backrest",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=backrest,
        origin=Origin(xyz=RECLINE_PIVOT),
        # Recline: negative q tips the backrest top toward -X (backward).
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=200.0, velocity=1.0, lower=-RECLINE_RANGE, upper=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lift_piston = object_model.get_part("lift_piston")
    seat = object_model.get_part("seat")
    backrest = object_model.get_part("backrest")
    lever = object_model.get_part("tilt_lever")
    lift_joint = object_model.get_articulation("base_to_lift_piston")
    swivel_joint = object_model.get_articulation("lift_piston_to_seat")
    recline_joint = object_model.get_articulation("seat_to_backrest")
    lever_joint = object_model.get_articulation("seat_to_tilt_lever")

    # ----------------------------------------------------- intentional fits
    ctx.allow_overlap(
        base,
        lift_piston,
        elem_a="lift_tube",
        elem_b="piston_rod",
        reason="Gas-lift piston rod intentionally slides inside the solid outer tube proxy.",
    )
    ctx.allow_overlap(
        base,
        lift_piston,
        elem_a="lift_collar",
        elem_b="piston_rod",
        reason="Piston rod passes through the chrome collar ring at the tube mouth.",
    )
    ctx.allow_overlap(
        lift_piston,
        seat,
        elem_a="piston_rod",
        elem_b="mech_housing",
        reason="Piston top is seated in the tilt-mechanism socket under the seat.",
    )
    ctx.allow_overlap(
        seat,
        backrest,
        elem_a="mech_rear_bracket",
        elem_b="pivot_barrel",
        reason="Backrest pivot barrel is captured in the mechanism rear bracket clevis.",
    )
    ctx.allow_overlap(
        seat,
        lever,
        elem_a="mech_housing",
        elem_b="lever_shaft",
        reason="Lever shaft passes into its bore in the mechanism housing.",
    )
    for i in range(CASTER_COUNT):
        ctx.allow_overlap(
            base,
            object_model.get_part(f"caster_wheel_{i}"),
            elem_a=f"caster_yoke_{i}",
            elem_b="axle",
            reason="Twin-wheel axle is captured through the caster yoke hub.",
        )

    # ------------------------------------------------------- hero geometry
    cushion_aabb = ctx.part_element_world_aabb(seat, elem="seat_cushion")
    ctx.check(
        "seat top sits at office-chair height (0.45-0.55 m)",
        cushion_aabb is not None and 0.45 <= cushion_aabb[1][2] <= 0.55,
        details=f"cushion aabb={cushion_aabb}",
    )
    pan_aabb = ctx.part_element_world_aabb(seat, elem="seat_pan")
    ctx.check(
        "seat pan is about 0.5 x 0.45 m",
        pan_aabb is not None
        and 0.44 <= (pan_aabb[1][0] - pan_aabb[0][0]) <= 0.56
        and 0.40 <= (pan_aabb[1][1] - pan_aabb[0][1]) <= 0.52,
        details=f"pan aabb={pan_aabb}",
    )

    head_aabb = ctx.part_element_world_aabb(backrest, elem="headrest_pillow")
    ctx.check(
        "headrest pillow caps the backrest at 1.25-1.42 m",
        head_aabb is not None and 1.25 <= head_aabb[1][2] <= 1.42,
        details=f"headrest aabb={head_aabb}",
    )
    back_aabb = ctx.part_world_aabb(backrest)
    ctx.check(
        "slim backrest is about 0.45 m wide",
        back_aabb is not None and 0.40 <= (back_aabb[1][1] - back_aabb[0][1]) <= 0.50,
        details=f"backrest aabb={back_aabb}",
    )
    panel_aabb = ctx.part_element_world_aabb(backrest, elem="mesh_panel_mid")
    rail_aabb = ctx.part_element_world_aabb(backrest, elem="left_side_rail")
    ctx.check(
        "mesh center panel is framed inside the side rails",
        panel_aabb is not None
        and rail_aabb is not None
        and panel_aabb[1][1] <= rail_aabb[1][1] + 0.001,
        details=f"panel={panel_aabb}, rail={rail_aabb}",
    )

    for label, low, high in (("left", 0.22, 0.30), ("right", -0.30, -0.22)):
        loop_aabb = ctx.part_element_world_aabb(seat, elem=f"{label}_armrest_loop")
        cy = None if loop_aabb is None else 0.5 * (loop_aabb[0][1] + loop_aabb[1][1])
        ctx.check(
            f"{label} closed-loop armrest sits at the seat side, top near 0.74 m",
            loop_aabb is not None
            and low <= cy <= high
            and 0.66 <= loop_aabb[1][2] <= 0.82,
            details=f"{label} loop aabb={loop_aabb}",
        )

    collar_aabb = ctx.part_element_world_aabb(base, elem="lift_collar")
    ctx.check(
        "thin chrome collar rings the top of the gas-lift column",
        collar_aabb is not None
        and (collar_aabb[1][2] - collar_aabb[0][2]) <= 0.025
        and 0.30 <= collar_aabb[1][2] <= 0.40,
        details=f"collar aabb={collar_aabb}",
    )
    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "6-spoke star base spans a stable footprint",
        base_aabb is not None
        and (base_aabb[1][0] - base_aabb[0][0]) >= 0.54
        and (base_aabb[1][1] - base_aabb[0][1]) >= 0.54,
        details=f"base aabb={base_aabb}",
    )

    # ------------------------------------------------- casters on the floor
    for i in range(CASTER_COUNT):
        wheel = object_model.get_part(f"caster_wheel_{i}")
        wheel_aabb = ctx.part_world_aabb(wheel)
        ctx.check(
            f"caster wheel {i} rests on the floor",
            wheel_aabb is not None and abs(wheel_aabb[0][2]) <= 0.003,
            details=f"wheel {i} aabb={wheel_aabb}",
        )
    spin_joint = object_model.get_articulation("base_to_caster_wheel_0")
    ctx.check(
        "caster wheels spin about a horizontal axle",
        spin_joint.articulation_type == ArticulationType.CONTINUOUS
        and abs(spin_joint.axis[2]) < 1e-9,
        details=f"axis={spin_joint.axis}",
    )
    wheel0 = object_model.get_part("caster_wheel_0")
    wheel0_rest_pos = ctx.part_world_position(wheel0)
    ctx.check(
        "caster axle height matches the wheel radius (rolling contact)",
        wheel0_rest_pos is not None and abs(wheel0_rest_pos[2] - WHEEL_RADIUS) <= 1e-6,
        details=f"wheel 0 origin={wheel0_rest_pos}",
    )
    with ctx.pose({spin_joint: 1.0}):
        spun_pos = ctx.part_world_position(wheel0)
        ctx.check(
            "caster wheel spins in place about its own axle",
            wheel0_rest_pos is not None
            and spun_pos is not None
            and all(abs(a - b) <= 1e-9 for a, b in zip(spun_pos, wheel0_rest_pos)),
            details=f"rest={wheel0_rest_pos}, spun={spun_pos}",
        )

    # -------------------------------------------------- 6-caster star base
    ctx.check(
        "star base has exactly six caster wheel parts",
        all(
            object_model.get_part(f"caster_wheel_{i}") is not None
            for i in range(CASTER_COUNT)
        ) and CASTER_COUNT == 6,
        details=f"CASTER_COUNT={CASTER_COUNT}",
    )
    caster_joint_names = [f"base_to_caster_wheel_{i}" for i in range(CASTER_COUNT)]
    caster_joints = [object_model.get_articulation(n) for n in caster_joint_names]
    ctx.check(
        "all six caster wheels have independent CONTINUOUS swivel joints",
        all(
            j is not None and j.articulation_type == ArticulationType.CONTINUOUS
            for j in caster_joints
        ),
        details=f"joints={[j.articulation_type if j else None for j in caster_joints]}",
    )

    # ----------------------------------------------------- gas lift travel
    ctx.expect_within(
        lift_piston,
        base,
        axes="xy",
        inner_elem="piston_rod",
        outer_elem="lift_tube",
        margin=0.001,
        name="piston rod stays centered in the lift tube",
    )
    ctx.expect_overlap(
        lift_piston,
        base,
        axes="z",
        elem_a="piston_rod",
        elem_b="lift_tube",
        min_overlap=0.15,
        name="piston rod is inserted in the tube at rest",
    )
    seat_rest = ctx.part_world_position(seat)
    with ctx.pose({lift_joint: LIFT_TRAVEL}):
        seat_up = ctx.part_world_position(seat)
        ctx.check(
            "gas lift raises the seat by the full 0.12 m travel",
            seat_rest is not None
            and seat_up is not None
            and abs((seat_up[2] - seat_rest[2]) - LIFT_TRAVEL) < 1e-6,
            details=f"rest={seat_rest}, up={seat_up}",
        )
        ctx.expect_overlap(
            lift_piston,
            base,
            axes="z",
            elem_a="piston_rod",
            elem_b="lift_tube",
            min_overlap=0.04,
            name="piston keeps retained insertion at full extension",
        )

    # ------------------------------------------------------------- swivel
    with ctx.pose({swivel_joint: math.pi / 2.0}):
        loop_aabb = ctx.part_element_world_aabb(seat, elem="right_armrest_loop")
        cx = None if loop_aabb is None else 0.5 * (loop_aabb[0][0] + loop_aabb[1][0])
        ctx.check(
            "seat assembly swivels about the gas-lift Z axis",
            cx is not None and cx > 0.15,
            details=f"right loop center x after 90deg swivel: {cx}",
        )
        head_swivel = ctx.part_element_world_aabb(backrest, elem="headrest_pillow")
        hy = None if head_swivel is None else 0.5 * (head_swivel[0][1] + head_swivel[1][1])
        ctx.check(
            "backrest swivels together with the seat",
            # +90deg about Z maps the rear headrest (x ~ -0.32) to y ~ -0.32.
            hy is not None and hy < -0.20,
            details=f"headrest center y after 90deg swivel: {hy}",
        )

    # ------------------------------------------------------------- recline
    head_rest_cx = 0.5 * (head_aabb[0][0] + head_aabb[1][0]) if head_aabb else None
    with ctx.pose({recline_joint: -RECLINE_RANGE}):
        head_back = ctx.part_element_world_aabb(backrest, elem="headrest_pillow")
        hbx = None if head_back is None else 0.5 * (head_back[0][0] + head_back[1][0])
        ctx.check(
            "backrest reclines backward about the horizontal mechanism axis",
            head_rest_cx is not None and hbx is not None and hbx < head_rest_cx - 0.15,
            details=f"headrest center x rest={head_rest_cx}, reclined={hbx}",
        )
        ctx.expect_overlap(
            backrest,
            seat,
            axes="xz",
            elem_a="pivot_barrel",
            elem_b="mech_rear_bracket",
            min_overlap=0.002,
            name="pivot barrel stays captured in the bracket while reclined",
        )
    ctx.expect_overlap(
        backrest,
        seat,
        axes="xz",
        elem_a="pivot_barrel",
        elem_b="mech_rear_bracket",
        min_overlap=0.002,
        name="pivot barrel is captured in the mechanism rear bracket",
    )

    # ---------------------------------------------------------- tilt lever
    paddle_rest = ctx.part_element_world_aabb(lever, elem="lever_paddle")
    with ctx.pose({lever_joint: 0.25}):
        paddle_up = ctx.part_element_world_aabb(lever, elem="lever_paddle")
        ctx.check(
            "mechanism paddle lever flips upward when actuated",
            paddle_rest is not None
            and paddle_up is not None
            and paddle_up[1][2] > paddle_rest[1][2] + 0.010,
            details=f"paddle top rest={paddle_rest}, up={paddle_up}",
        )

    return ctx.report()


object_model = build_object_model()
