from __future__ import annotations

# Tall ergonomic office chair (IKEA Markus style), all black.
# Variant: flip-up armrests.
#
# Kinematic tree:
#   base (root: 5-spoke star base, caster stems/yokes, gas-lift outer tube)
#     -> caster_wheel_0..4   CONTINUOUS about each horizontal axle
#     -> lift_piston         PRISMATIC along Z (gas lift, 0..0.12 m)
#        -> seat             CONTINUOUS swivel about Z (seat pan, mech)
#           -> armrest_0     REVOLUTE flip-up hinge at rear, axis -Y
#           -> armrest_1     REVOLUTE flip-up hinge at rear, axis -Y
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
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)

# ---------------------------------------------------------------- dimensions
CASTER_COUNT = 5
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

# Flip-up armrest dimensions and placement.
ARMREST_COUNT = 2
ARMREST_HINGE_X = -0.15      # hinge at the rear of the seat, near backrest
ARMREST_HINGE_Y = 0.22        # offset from seat center at each side
ARMREST_HINGE_Z = 0.13        # above seat cushion top to clear the cushion
ARMREST_FLIP_UPPER = 1.4      # ~80 degrees upward flip


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


def _armrest_arm_geom(sign: float):
    """Curved support arm tube from the hinge origin to the pad mount."""
    return tube_from_spline_points(
        [
            (0.0, 0.0, 0.02),
            (0.05, sign * 0.015, 0.05),
            (0.10, sign * 0.025, 0.08),
            (0.15, sign * 0.030, 0.09),
        ],
        radius=0.012,
        samples_per_segment=14,
        radial_segments=14,
        cap_ends=True,
    )


def _armrest_pad_geom(sign: float):
    """Flattened padded armrest top, elongated along the forward axis."""
    geom = ExtrudeGeometry(rounded_rect_profile(0.16, 0.065, 0.015), 0.025, center=True)
    geom.translate(0.10, sign * 0.030, 0.10)
    return geom


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

    caster_angles = [2.0 * math.pi * i / CASTER_COUNT + math.pi / 5.0 for i in range(CASTER_COUNT)]
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

    # Armrest hinge mount brackets on the seat sides. Each bracket extends
    # from the seat pan edge up to the armrest hinge, providing the visible
    # and physical mount for the flip-up armrest pivot.
    armrest_signs = (1.0, -1.0)  # 0 = left (+Y), 1 = right (-Y)
    for i, sign in enumerate(armrest_signs):
        seat.visual(
            Box((0.050, 0.028, 0.060)),
            origin=Origin(
                xyz=(ARMREST_HINGE_X, sign * ARMREST_HINGE_Y, ARMREST_HINGE_Z - 0.025),
            ),
            material=plastic_black,
            name=f"armrest_mount_{i}",
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

    # ------------------------------------------------------- flip-up armrests
    # Each armrest pivots upward on its own REVOLUTE hinge at the rear of the
    # armrest where it meets the backrest. Built in a for-loop with shared
    # geometry helpers and mirrored placement.
    armrest_signs = (1.0, -1.0)  # 0 = left (+Y), 1 = right (-Y)
    for i, sign in enumerate(armrest_signs):
        armrest = model.part(f"armrest_{i}")

        # Hinge barrel — visible pivot axis at the armrest origin.
        armrest.visual(
            Cylinder(radius=0.015, length=0.050),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel_dark,
            name="hinge_barrel",
        )

        # Curved support arm from hinge to pad mount.
        arm_geom = _armrest_arm_geom(sign)
        armrest.visual(
            mesh_from_geometry(arm_geom, "support_arm"),
            material=plastic_black,
            name="support_arm",
        )

        # Flattened padded armrest top.
        pad_geom = _armrest_pad_geom(sign)
        armrest.visual(
            mesh_from_geometry(pad_geom, "armrest_pad"),
            material=fabric_black,
            name="armrest_pad",
        )

        # REVOLUTE hinge: axis = (0, -1, 0) so positive q lifts the
        # forward-extending arm upward (+X rotates toward +Z).
        model.articulation(
            f"seat_to_armrest_{i}",
            ArticulationType.REVOLUTE,
            parent=seat,
            child=armrest,
            origin=Origin(
                xyz=(ARMREST_HINGE_X, sign * ARMREST_HINGE_Y, ARMREST_HINGE_Z),
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=2.0,
                lower=0.0,
                upper=ARMREST_FLIP_UPPER,
            ),
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
    for i in range(ARMREST_COUNT):
        ctx.allow_overlap(
            seat,
            object_model.get_part(f"armrest_{i}"),
            elem_a=f"armrest_mount_{i}",
            elem_b="hinge_barrel",
            reason="Armrest hinge barrel is captured in the seat-side mount bracket.",
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
        "5-spoke star base spans a stable footprint",
        base_aabb is not None
        and (base_aabb[1][0] - base_aabb[0][0]) >= 0.55
        and (base_aabb[1][1] - base_aabb[0][1]) >= 0.55,
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

    # ------------------------------------------------- flip-up armrests
    armrest_joints = []
    for i in range(ARMREST_COUNT):
        armrest_i = object_model.get_part(f"armrest_{i}")
        joint_i = object_model.get_articulation(f"seat_to_armrest_{i}")
        armrest_joints.append(joint_i)

        # Joint type check.
        ctx.check(
            f"armrest_{i} has a REVOLUTE flip-up hinge",
            joint_i.articulation_type == ArticulationType.REVOLUTE,
            details=f"type={joint_i.articulation_type}",
        )

        # Rest-pose pad position: at the seat side, at armrest height.
        pad_aabb = ctx.part_element_world_aabb(armrest_i, elem="armrest_pad")
        sign = 1.0 if i == 0 else -1.0
        cy = None if pad_aabb is None else 0.5 * (pad_aabb[0][1] + pad_aabb[1][1])
        ctx.check(
            f"armrest_{i} pad sits at the correct seat side and height",
            pad_aabb is not None
            and sign * cy > 0.18
            and 0.55 <= pad_aabb[1][2] <= 0.75,
            details=f"armrest_{i} pad aabb={pad_aabb}",
        )

    # Flip-up pose test: positive q raises the armrest pad upward.
    armrest_0 = object_model.get_part("armrest_0")
    armrest_0_joint = armrest_joints[0]
    pad_rest_aabb = ctx.part_element_world_aabb(armrest_0, elem="armrest_pad")
    with ctx.pose({armrest_0_joint: ARMREST_FLIP_UPPER}):
        pad_up_aabb = ctx.part_element_world_aabb(armrest_0, elem="armrest_pad")
        ctx.check(
            "armrest_0 flips upward when hinge is actuated",
            pad_rest_aabb is not None
            and pad_up_aabb is not None
            and pad_up_aabb[1][2] > pad_rest_aabb[1][2] + 0.05,
            details=f"pad rest top={pad_rest_aabb[1][2]}, flipped top={pad_up_aabb[1][2]}",
        )

    # Independent articulation: flipping one armrest does not move the other.
    armrest_1 = object_model.get_part("armrest_1")
    pad1_rest_aabb = ctx.part_element_world_aabb(armrest_1, elem="armrest_pad")
    with ctx.pose({armrest_0_joint: ARMREST_FLIP_UPPER}):
        pad1_during_aabb = ctx.part_element_world_aabb(armrest_1, elem="armrest_pad")
        ctx.check(
            "armrest_1 stays in place when armrest_0 flips up",
            pad1_rest_aabb is not None
            and pad1_during_aabb is not None
            and abs(pad1_during_aabb[1][2] - pad1_rest_aabb[1][2]) < 0.001,
            details=f"armrest_1 rest top={pad1_rest_aabb[1][2]}, during={pad1_during_aabb[1][2]}",
        )

    # Hinge axis is horizontal (no Z component).
    ctx.check(
        "armrest hinge axis is horizontal (lateral)",
        abs(armrest_0_joint.axis[2]) < 1e-9,
        details=f"axis={armrest_0_joint.axis}",
    )

    # Each armrest hinge barrel is seated in its mount bracket.
    for i in range(ARMREST_COUNT):
        armrest_i = object_model.get_part(f"armrest_{i}")
        ctx.expect_contact(
            armrest_i,
            seat,
            elem_a="hinge_barrel",
            elem_b=f"armrest_mount_{i}",
            contact_tol=0.005,
            name=f"armrest_{i} hinge barrel contacts seat mount",
        )

    # ------------------------------------------------------------- swivel
    with ctx.pose({swivel_joint: math.pi / 2.0}):
        pad_swivel_aabb = ctx.part_element_world_aabb(armrest_1, elem="armrest_pad")
        cx = None if pad_swivel_aabb is None else 0.5 * (pad_swivel_aabb[0][0] + pad_swivel_aabb[1][0])
        ctx.check(
            "seat assembly swivels about the gas-lift Z axis",
            cx is not None and cx > 0.10,
            details=f"armrest_1 pad center x after 90deg swivel: {cx}",
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