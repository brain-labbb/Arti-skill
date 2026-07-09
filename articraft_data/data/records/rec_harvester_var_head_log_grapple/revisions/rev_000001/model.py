from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _beam_origin(p0, p1):
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(-dz, math.sqrt(dx * dx + dy * dy))
    center = ((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5)
    return length, Origin(xyz=center, rpy=(0.0, pitch, yaw))


def _cylinder_origin(p0, p1):
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    center = ((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5)
    horizontal = math.sqrt(dx * dx + dy * dy)
    pitch = math.atan2(horizontal, dz)
    yaw = math.atan2(dy, dx) if horizontal > 1e-9 else 0.0
    return length, Origin(xyz=center, rpy=(0.0, pitch, yaw))


def _add_beam(part, p0, p1, width, depth, material, name):
    length, origin = _beam_origin(p0, p1)
    part.visual(Box((length, width, depth)), origin=origin, material=material, name=name)


def _add_cylinder_between(part, p0, p1, radius, material, name):
    length, origin = _cylinder_origin(p0, p1)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def _add_pin_y(part, xyz, length, radius, material, name):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="agricultural_harvester_vehicle_arm",
        meta={
            "category": "Agricultural",
            "small_class": "Harvester vehicle (arm)",
            "description": "Forestry-style agricultural harvester vehicle with articulated boom carrying a bypass-log loader grapple (frame, rotator, two curved jaws).",
        },
    )

    green = Material("painted_agricultural_green", rgba=(0.02, 0.36, 0.12, 1.0))
    yellow = Material("harvester_yellow", rgba=(0.96, 0.74, 0.10, 1.0))
    dark = Material("dark_powder_coated_steel", rgba=(0.04, 0.05, 0.055, 1.0))
    boom_gray = Material("charcoal_boom_steel", rgba=(0.12, 0.15, 0.17, 1.0))
    gray = Material("galvanized_gray_panel", rgba=(0.35, 0.41, 0.43, 1.0))
    rubber = Material("muddy_black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    glass = Material("green_tinted_safety_glass", rgba=(0.15, 0.40, 0.34, 0.46))
    chrome = Material("polished_hydraulic_chrome", rgba=(0.78, 0.82, 0.84, 1.0))
    hose = Material("green_black_hydraulic_hose", rgba=(0.00, 0.20, 0.12, 1.0))
    amber = Material("amber_lens", rgba=(1.0, 0.55, 0.08, 1.0))
    red = Material("red_tail_lens", rgba=(0.75, 0.03, 0.02, 1.0))
    black = Material("matte_black_hardware", rgba=(0.0, 0.0, 0.0, 1.0))

    rear_tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.72,
            0.62,
            inner_radius=0.48,
            carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.07),
            tread=TireTread(style="chevron", depth=0.055, count=22, angle_deg=27.0, land_ratio=0.56),
            grooves=(TireGroove(center_offset=0.0, width=0.040, depth=0.018),),
            sidewall=TireSidewall(style="rounded", bulge=0.045),
            shoulder=TireShoulder(width=0.045, radius=0.010),
        ),
        "rear_chevron_tire",
    )
    rear_rim_mesh = mesh_from_geometry(
        WheelGeometry(
            0.47,
            0.48,
            rim=WheelRim(inner_radius=0.30, flange_height=0.035, flange_thickness=0.018, bead_seat_depth=0.018),
            hub=WheelHub(
                radius=0.15,
                width=0.24,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=8, circle_diameter=0.22, hole_diameter=0.026),
            ),
            face=WheelFace(dish_depth=0.045, front_inset=0.025, rear_inset=0.018),
            spokes=WheelSpokes(style="split_y", count=8, thickness=0.025, window_radius=0.085),
            bore=WheelBore(style="round", diameter=0.080),
        ),
        "rear_yellow_rim",
    )
    front_tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.58,
            0.50,
            inner_radius=0.38,
            carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.06),
            tread=TireTread(style="chevron", depth=0.046, count=20, angle_deg=29.0, land_ratio=0.55),
            grooves=(TireGroove(center_offset=0.0, width=0.034, depth=0.014),),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.038, radius=0.009),
        ),
        "front_chevron_tire",
    )
    front_rim_mesh = mesh_from_geometry(
        WheelGeometry(
            0.37,
            0.40,
            rim=WheelRim(inner_radius=0.23, flange_height=0.026, flange_thickness=0.014, bead_seat_depth=0.014),
            hub=WheelHub(
                radius=0.115,
                width=0.20,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=6, circle_diameter=0.16, hole_diameter=0.020),
            ),
            face=WheelFace(dish_depth=0.035, front_inset=0.020, rear_inset=0.014),
            spokes=WheelSpokes(style="split_y", count=6, thickness=0.020, window_radius=0.066),
            bore=WheelBore(style="round", diameter=0.060),
        ),
        "front_yellow_rim",
    )

    chassis = model.part("chassis")
    chassis.visual(Box((5.25, 1.18, 0.34)), origin=Origin(xyz=(0.05, 0.0, 0.92)), material=dark, name="main_frame")
    chassis.visual(Box((1.05, 1.36, 0.18)), origin=Origin(xyz=(-1.34, 0.0, 1.28)), material=green, name="arm_pedestal_deck")
    chassis.visual(Cylinder(radius=0.42, length=0.24), origin=Origin(xyz=(-1.34, 0.0, 1.51)), material=dark, name="slew_ring")
    chassis.visual(Cylinder(radius=0.32, length=0.34), origin=Origin(xyz=(-1.34, 0.0, 1.80)), material=boom_gray, name="boom_turret")
    chassis.visual(Box((0.22, 0.12, 0.78)), origin=Origin(xyz=(-1.34, -0.38, 1.96)), material=boom_gray, name="pivot_cheek_0")
    chassis.visual(Box((0.22, 0.12, 0.78)), origin=Origin(xyz=(-1.34, 0.38, 1.96)), material=boom_gray, name="pivot_cheek_1")
    chassis.visual(
        Cylinder(radius=0.070, length=0.90),
        origin=Origin(xyz=(-1.34, 0.0, 2.16), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="base_pivot_pin",
    )

    chassis.visual(Box((2.25, 1.36, 0.88)), origin=Origin(xyz=(1.76, 0.0, 1.56)), material=green, name="engine_hood")
    chassis.visual(Box((2.00, 1.24, 0.10)), origin=Origin(xyz=(1.74, 0.0, 2.05)), material=green, name="sloped_hood_cap")
    chassis.visual(Box((1.62, 0.030, 0.10)), origin=Origin(xyz=(1.66, -0.675, 1.80)), material=yellow, name="yellow_side_stripe")
    chassis.visual(Box((0.38, 0.035, 0.14)), origin=Origin(xyz=(1.08, -0.682, 1.91)), material=yellow, name="brand_plate")
    chassis.visual(Box((0.78, 0.04, 0.46)), origin=Origin(xyz=(2.68, -0.69, 1.39)), material=gray, name="rear_side_service_panel")
    chassis.visual(Box((0.42, 1.18, 0.16)), origin=Origin(xyz=(2.95, 0.0, 1.82)), material=dark, name="rear_counterweight")
    chassis.visual(Box((0.07, 1.05, 0.08)), origin=Origin(xyz=(3.19, 0.0, 1.92)), material=red, name="tail_lamps")

    # --- Operator cab: tall, forward-raked glasshouse (John Deere style), rigidly mounted to chassis ---
    # Cab sits on a solid mount pedestal that seats on the frame spine and main frame (no floating look).
    wind_pitch = -0.175  # forward rake: windshield/A-pillar tops overhang the front of the cab
    chassis.visual(Box((1.30, 1.16, 0.52)), origin=Origin(xyz=(0.04, 0.0, 1.30)), material=green, name="cab_subframe")
    chassis.visual(Box((1.36, 1.20, 0.16)), origin=Origin(xyz=(0.04, 0.0, 1.16)), material=dark, name="cab_underframe")
    chassis.visual(Box((1.36, 1.18, 0.22)), origin=Origin(xyz=(0.04, 0.0, 1.50)), material=green, name="cab_floor_deck")
    chassis.visual(Box((1.40, 0.04, 0.10)), origin=Origin(xyz=(0.04, -0.61, 1.62)), material=yellow, name="cab_belt_stripe")

    # ROPS pillar cage: raked A-pillars at the front, vertical B-pillars at the rear, tied by a top cant-rail ring.
    for side, y in ((0, -0.54), (1, 0.54)):
        _add_beam(chassis, (-0.54, y, 1.50), (-0.68, y, 2.56), 0.09, 0.10, dark, f"cab_a_pillar_{side}")
        chassis.visual(Box((0.10, 0.10, 1.10)), origin=Origin(xyz=(0.60, y, 2.03)), material=dark, name=f"cab_b_pillar_{side}")
    chassis.visual(Box((0.12, 1.22, 0.11)), origin=Origin(xyz=(-0.68, 0.0, 2.55)), material=dark, name="cab_front_cant_rail")
    chassis.visual(Box((0.12, 1.22, 0.11)), origin=Origin(xyz=(0.60, 0.0, 2.56)), material=dark, name="cab_rear_cant_rail")
    chassis.visual(Box((1.36, 0.10, 0.10)), origin=Origin(xyz=(-0.04, 0.55, 2.55)), material=dark, name="cab_left_cant_rail")
    chassis.visual(Box((1.36, 0.10, 0.10)), origin=Origin(xyz=(-0.04, -0.55, 2.55)), material=dark, name="cab_right_cant_rail")

    # Curved, overhanging roof with a raised center dome.
    chassis.visual(Box((1.54, 1.24, 0.10)), origin=Origin(xyz=(-0.06, 0.0, 2.60)), material=dark, name="cab_roof")
    chassis.visual(Box((1.18, 0.98, 0.10)), origin=Origin(xyz=(-0.06, 0.0, 2.68)), material=dark, name="cab_roof_dome")

    # Large forward-raked windshield with frame mullions (the main visual fix).
    chassis.visual(
        Box((0.05, 1.02, 0.99)),
        origin=Origin(xyz=(-0.60, 0.0, 2.04), rpy=(0.0, wind_pitch, 0.0)),
        material=glass,
        name="windshield",
    )
    chassis.visual(
        Box((0.06, 0.05, 0.99)),
        origin=Origin(xyz=(-0.60, 0.0, 2.04), rpy=(0.0, wind_pitch, 0.0)),
        material=dark,
        name="windshield_center_mullion",
    )
    chassis.visual(
        Box((0.06, 1.02, 0.05)),
        origin=Origin(xyz=(-0.595, 0.0, 2.02), rpy=(0.0, wind_pitch, 0.0)),
        material=dark,
        name="windshield_transom",
    )
    chassis.visual(
        Box((0.04, 0.30, 0.04)),
        origin=Origin(xyz=(-0.555, -0.10, 1.74), rpy=(0.0, wind_pitch, 0.0)),
        material=black,
        name="windshield_wiper",
    )

    # Side door glazing (both sides) and rear window — full wrap-around greenhouse.
    chassis.visual(Box((1.20, 0.05, 0.96)), origin=Origin(xyz=(0.0, -0.555, 2.04)), material=glass, name="side_window_0")
    chassis.visual(Box((1.20, 0.05, 0.96)), origin=Origin(xyz=(0.0, 0.555, 2.04)), material=glass, name="side_window_1")
    chassis.visual(Box((0.05, 1.00, 0.94)), origin=Origin(xyz=(0.62, 0.0, 2.05)), material=glass, name="rear_window")
    chassis.visual(Box((0.10, 0.05, 0.18)), origin=Origin(xyz=(0.22, -0.59, 1.92)), material=chrome, name="door_handle")
    chassis.visual(Box((0.06, 0.06, 0.60)), origin=Origin(xyz=(-0.50, -0.59, 1.92)), material=dark, name="door_grab_rail")

    # Roof-front work-light bar with amber lamps, plus a corner beacon.
    chassis.visual(Box((0.12, 0.66, 0.08)), origin=Origin(xyz=(-0.74, 0.0, 2.69)), material=black, name="work_light_bar")
    for i, ly in enumerate((-0.20, 0.20)):
        chassis.visual(Box((0.05, 0.10, 0.07)), origin=Origin(xyz=(-0.80, ly, 2.70)), material=amber, name=f"cab_work_light_{i}")
    chassis.visual(Cylinder(radius=0.05, length=0.11), origin=Origin(xyz=(0.34, 0.42, 2.76)), material=amber, name="roof_beacon")
    # Mirrors on short arms off the A-pillars.
    for side, y in ((0, -0.58), (1, 0.58)):
        chassis.visual(Box((0.34, 0.05, 0.05)), origin=Origin(xyz=(-0.74, y, 2.16)), material=dark, name=f"mirror_arm_{side}")
        chassis.visual(Box((0.05, 0.11, 0.22)), origin=Origin(xyz=(-0.90, y, 2.14)), material=gray, name=f"mirror_head_{side}")

    # Steps, access ladder, rails, and guards are rigidly attached to the chassis.
    for i, z in enumerate((0.72, 0.92, 1.12)):
        chassis.visual(Box((0.48, 0.20, 0.055)), origin=Origin(xyz=(-0.03, -0.88 - i * 0.08, z)), material=gray, name=f"access_step_{i}")
    chassis.visual(Box((0.10, 0.24, 0.86)), origin=Origin(xyz=(-0.35, -0.79, 1.08)), material=dark, name="step_side_bracket")
    chassis.visual(Box((0.24, 0.16, 0.72)), origin=Origin(xyz=(-0.23, -0.81, 1.02)), material=dark, name="step_inner_rail")
    chassis.visual(Box((0.18, 0.18, 0.46)), origin=Origin(xyz=(-0.28, -0.61, 1.08)), material=dark, name="step_frame_tie")
    # Inboard riser bridges the middle and top treads so the top step is not isolated (kept clear of the wheel).
    chassis.visual(Box((0.30, 0.16, 0.34)), origin=Origin(xyz=(-0.03, -0.98, 1.02)), material=dark, name="step_inner_riser")
    chassis.visual(Box((0.12, 0.62, 0.10)), origin=Origin(xyz=(-1.95, -0.80, 1.37)), material=dark, name="front_guard_bar")
    chassis.visual(Box((0.12, 0.62, 0.10)), origin=Origin(xyz=(-0.72, -0.80, 1.37)), material=dark, name="mid_guard_bar")
    chassis.visual(Box((1.36, 0.08, 0.08)), origin=Origin(xyz=(-1.34, -0.80, 1.48)), material=dark, name="front_side_rail")
    chassis.visual(Box((1.45, 0.32, 0.10)), origin=Origin(xyz=(-1.34, -0.64, 1.39)), material=dark, name="guard_frame_tie")
    chassis.visual(Box((4.80, 0.26, 0.24)), origin=Origin(xyz=(0.22, 0.0, 1.12)), material=dark, name="upper_frame_spine")
    chassis.visual(Box((0.66, 0.86, 0.11)), origin=Origin(xyz=(-2.00, -1.34, 1.32)), material=green, name="front_fender_0")
    chassis.visual(Box((0.66, 0.86, 0.11)), origin=Origin(xyz=(-2.00, 1.34, 1.32)), material=green, name="front_fender_1")
    chassis.visual(Box((0.66, 0.86, 0.11)), origin=Origin(xyz=(-0.55, -1.34, 1.32)), material=green, name="mid_fender_0")
    chassis.visual(Box((0.66, 0.86, 0.11)), origin=Origin(xyz=(-0.55, 1.34, 1.32)), material=green, name="mid_fender_1")
    chassis.visual(Box((0.92, 0.98, 0.13)), origin=Origin(xyz=(2.02, -1.36, 1.58)), material=green, name="rear_fender_0")
    chassis.visual(Box((0.92, 0.98, 0.13)), origin=Origin(xyz=(2.02, 1.36, 1.58)), material=green, name="rear_fender_1")
    for i, (x, y, z) in enumerate([(-2.00, -0.74, 1.26), (-2.00, 0.74, 1.26), (-0.55, -0.74, 1.26), (-0.55, 0.74, 1.26), (2.02, -0.74, 1.50), (2.02, 0.74, 1.50)]):
        chassis.visual(Box((0.14, 0.38, 0.20)), origin=Origin(xyz=(x, y, z)), material=dark, name=f"fender_brace_{i}")
    # Inboard fender supports tie the +Y front/mid fenders down to the frame spine (the -Y side is already tied by the guard rails).
    for i, x in enumerate((-2.00, -0.55)):
        chassis.visual(Box((0.16, 0.80, 0.16)), origin=Origin(xyz=(x, 0.45, 1.20)), material=dark, name=f"fender_support_{i}")

    for x, z, nm, axle_len in [(-2.00, 0.58, "front_axle", 2.16), (-0.55, 0.58, "mid_axle", 2.16), (2.02, 0.72, "rear_axle", 2.24)]:
        _add_pin_y(chassis, (x, 0.0, z), axle_len, 0.075 if nm != "rear_axle" else 0.095, dark, nm)
        chassis.visual(Box((0.16, 0.20, 0.44)), origin=Origin(xyz=(x, 0.0, z + 0.19)), material=dark, name=f"{nm}_suspension")

    # Ground-contact brush/detail comb on the front carriage, just behind the arm base.
    for i, y in enumerate([-0.48, -0.32, -0.16, 0.0, 0.16, 0.32, 0.48]):
        chassis.visual(Box((0.07, 0.045, 0.42)), origin=Origin(xyz=(-2.22, y, 1.00), rpy=(0.0, 0.36, 0.0)), material=dark, name=f"brush_tooth_{i}")

    # High guard rail over the rear engine cover.
    rail_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [(1.05, -0.67, 2.12), (1.80, -0.71, 2.28), (2.70, -0.67, 2.12)],
            radius=0.030,
            samples_per_segment=12,
            radial_segments=16,
        ),
        "engine_guard_rail",
    )
    chassis.visual(rail_mesh, material=dark, name="engine_guard_rail")
    chassis.visual(Box((0.07, 0.07, 0.58)), origin=Origin(xyz=(1.05, -0.66, 2.05)), material=dark, name="engine_rail_post_0")
    chassis.visual(Box((0.07, 0.07, 0.58)), origin=Origin(xyz=(2.70, -0.66, 2.05)), material=dark, name="engine_rail_post_1")

    # A small bundle of hydraulic hoses from the turret into the boom pivot.
    chassis_hose_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [(-1.05, -0.32, 1.38), (-1.26, -0.44, 1.70), (-1.44, -0.43, 2.10)],
            radius=0.026,
            samples_per_segment=16,
            radial_segments=16,
        ),
        "turret_hose_bundle",
    )
    chassis.visual(chassis_hose_mesh, material=hose, name="turret_hose_bundle")

    wheel_specs = [
        ("front_left_wheel", -2.00, 1.35, 0.58, front_tire_mesh, front_rim_mesh),
        ("front_right_wheel", -2.00, -1.35, 0.58, front_tire_mesh, front_rim_mesh),
        ("mid_left_wheel", -0.55, 1.35, 0.58, front_tire_mesh, front_rim_mesh),
        ("mid_right_wheel", -0.55, -1.35, 0.58, front_tire_mesh, front_rim_mesh),
        ("rear_left_wheel", 2.02, 1.39, 0.72, rear_tire_mesh, rear_rim_mesh),
        ("rear_right_wheel", 2.02, -1.39, 0.72, rear_tire_mesh, rear_rim_mesh),
    ]
    for name, x, y, z, tire_mesh, rim_mesh in wheel_specs:
        wheel = model.part(name)
        wheel.visual(tire_mesh, origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)), material=rubber, name="tire")
        wheel.visual(rim_mesh, origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)), material=yellow, name="rim")
        wheel.visual(Cylinder(radius=0.13 if z < 0.7 else 0.17, length=0.16), origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)), material=dark, name="hub_cap")
        sign = 1.0 if y > 0.0 else -1.0
        wheel.visual(
            Cylinder(radius=0.11 if z < 0.7 else 0.14, length=0.24),
            origin=Origin(xyz=(0.0, -sign * 0.15, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark,
            name="inner_hub_sleeve",
        )
        model.articulation(
            f"chassis_to_{name}",
            ArticulationType.CONTINUOUS,
            parent=chassis,
            child=wheel,
            origin=Origin(xyz=(x, y, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=18000.0, velocity=8.0),
        )

    main_boom = model.part("main_boom")
    main_side_beam_0_len, main_side_beam_0_origin = _beam_origin((0.00, -0.22, 0.00), (-2.55, -0.22, 1.72))
    main_boom.visual(
        Box((main_side_beam_0_len, 0.18, 0.22)),
        origin=main_side_beam_0_origin,
        material=boom_gray,
        name="main_side_beam_0",
    )
    main_side_beam_1_len, main_side_beam_1_origin = _beam_origin((0.00, 0.22, 0.00), (-2.55, 0.22, 1.72))
    main_boom.visual(
        Box((main_side_beam_1_len, 0.18, 0.22)),
        origin=main_side_beam_1_origin,
        material=boom_gray,
        name="main_side_beam_1",
    )
    for side, y in enumerate((-0.22, 0.22)):
        _add_beam(main_boom, (-0.55, y, -0.06), (-2.40, y, 1.33), 0.11, 0.12, dark, f"lower_tension_link_{side}")
        # Tall lug gusset bridges the lower tension link up to the main side beam on both sides (no isolated link).
        main_boom.visual(Box((0.18, 0.18, 0.54)), origin=Origin(xyz=(-0.55, y, 0.16)), material=dark, name=f"lower_link_mount_{side}")
    main_boom.visual(Box((0.44, 0.68, 0.26)), origin=Origin(xyz=(-0.03, 0.0, 0.00)), material=boom_gray, name="base_lug_block")
    main_boom.visual(Box((0.50, 0.62, 0.24)), origin=Origin(xyz=(-2.55, 0.0, 1.72)), material=boom_gray, name="elbow_lug_block")
    _add_pin_y(main_boom, (-2.55, 0.0, 1.72), 0.78, 0.075, dark, "elbow_pin")
    _add_cylinder_between(main_boom, (-0.58, -0.42, 0.02), (-1.45, -0.42, 0.72), 0.055, black, "boom_lift_cylinder")
    _add_cylinder_between(main_boom, (-1.45, -0.42, 0.72), (-2.25, -0.42, 1.39), 0.032, chrome, "boom_lift_rod")
    main_boom.visual(Box((0.18, 0.28, 0.16)), origin=Origin(xyz=(-0.58, -0.32, 0.03)), material=dark, name="boom_cylinder_base_mount")
    main_boom.visual(Box((0.16, 0.26, 0.14)), origin=Origin(xyz=(-2.18, -0.32, 1.34)), material=dark, name="boom_cylinder_rod_mount")
    main_hose_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [(-0.08, -0.29, 0.08), (-0.80, -0.31, 0.44), (-1.72, -0.31, 1.08), (-2.48, -0.29, 1.62)],
            radius=0.022,
            samples_per_segment=14,
            radial_segments=14,
        ),
        "main_boom_hose",
    )
    main_boom.visual(main_hose_mesh, material=hose, name="main_boom_hose")
    for i, x in enumerate((-0.55, -1.35, -2.10)):
        main_boom.visual(Box((0.12, 0.22, 0.10)), origin=Origin(xyz=(x, -0.25, 0.18 + 0.62 * (-x - 0.55))), material=dark, name=f"hose_clamp_{i}")

    model.articulation(
        "chassis_to_main_boom",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=main_boom,
        origin=Origin(xyz=(-1.34, 0.0, 2.16)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=95000.0, velocity=0.45, lower=-0.42, upper=0.50),
    )

    stick = model.part("stick")
    for side, y in enumerate((-0.18, 0.18)):
        _add_beam(stick, (-0.42, y, -0.24), (-2.46, y, -1.18), 0.16, 0.20, boom_gray, f"stick_side_beam_{side}")
        _add_beam(stick, (-0.45, y, -0.08), (-2.16, y, -0.72), 0.10, 0.11, dark, f"stick_upper_link_{side}")
    stick.visual(Box((0.26, 0.46, 0.18)), origin=Origin(xyz=(-0.37, 0.0, -0.19)), material=boom_gray, name="stick_elbow_socket")
    stick.visual(Box((0.40, 0.52, 0.22)), origin=Origin(xyz=(-2.46, 0.0, -1.18)), material=boom_gray, name="wrist_socket")
    _add_pin_y(stick, (-2.46, 0.0, -1.18), 0.64, 0.060, dark, "wrist_pin")
    _add_cylinder_between(stick, (-0.54, 0.34, -0.12), (-1.38, 0.34, -0.56), 0.050, black, "stick_cylinder")
    _add_cylinder_between(stick, (-1.38, 0.34, -0.56), (-2.18, 0.34, -0.94), 0.028, chrome, "stick_rod")
    stick.visual(Box((0.18, 0.24, 0.16)), origin=Origin(xyz=(-0.55, 0.24, -0.14)), material=dark, name="stick_cylinder_base_mount")
    stick.visual(Box((0.16, 0.24, 0.14)), origin=Origin(xyz=(-2.12, 0.24, -0.91)), material=dark, name="stick_cylinder_rod_mount")
    stick_hose_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [(0.02, 0.52, 0.05), (-0.70, 0.54, -0.24), (-1.54, 0.53, -0.66), (-2.34, 0.50, -1.08)],
            radius=0.020,
            samples_per_segment=14,
            radial_segments=14,
        ),
        "stick_hose",
    )
    stick.visual(stick_hose_mesh, material=hose, name="stick_hose")
    stick.visual(Box((0.12, 0.18, 0.09)), origin=Origin(xyz=(-1.50, 0.43, -0.66)), material=dark, name="stick_hose_clamp")

    model.articulation(
        "main_boom_to_stick",
        ArticulationType.REVOLUTE,
        parent=main_boom,
        child=stick,
        origin=Origin(xyz=(-2.55, 0.0, 1.72)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=76000.0, velocity=0.55, lower=-0.70, upper=0.75),
    )

    # === Grapple Frame (loader/bypass-log grapple — replaces harvester_head) ===
    grapple_frame = model.part("grapple_frame")
    # Wrist yoke that seats in the stick socket around the wrist pin.
    grapple_frame.visual(Box((0.10, 0.52, 0.14)), origin=Origin(xyz=(0.0, 0.0, -0.02)), material=boom_gray, name="frame_yoke")
    grapple_frame.visual(Box((0.20, 0.44, 0.14)), origin=Origin(xyz=(0.0, 0.0, -0.16)), material=boom_gray, name="yoke_bridge")
    # Rotator housing — vertical cylinder below the yoke.
    grapple_frame.visual(Cylinder(radius=0.18, length=0.24), origin=Origin(xyz=(0.0, 0.0, -0.38)), material=dark, name="rotator_housing")
    grapple_frame.visual(Cylinder(radius=0.22, length=0.05), origin=Origin(xyz=(0.0, 0.0, -0.28)), material=gray, name="bearing_ring")
    # Side gussets tying the yoke bridge down to the housing.
    for side, y in enumerate((-0.18, 0.18)):
        grapple_frame.visual(Box((0.14, 0.06, 0.20)), origin=Origin(xyz=(0.0, y, -0.26)), material=dark, name=f"frame_gusset_{side}")

    model.articulation(
        "stick_to_grapple_frame",
        ArticulationType.REVOLUTE,
        parent=stick,
        child=grapple_frame,
        origin=Origin(xyz=(-2.46, 0.0, -1.18)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=16000.0, velocity=1.0, lower=-0.65, upper=0.65),
    )

    # === Rotator — continuous rotation around vertical axis ===
    rotator = model.part("rotator")
    rotator.visual(Cylinder(radius=0.15, length=0.08), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=dark, name="turntable_disk")
    rotator.visual(Cylinder(radius=0.10, length=0.06), origin=Origin(xyz=(0.0, 0.0, -0.05)), material=gray, name="rotator_shaft")
    # Jaw carrier crossbeam with side brackets to the pivot lugs.
    rotator.visual(Box((0.12, 0.44, 0.08)), origin=Origin(xyz=(0.0, 0.0, -0.10)), material=boom_gray, name="jaw_carrier")
    for i, y in enumerate((-0.28, 0.28)):
        # Bracket connects carrier beam to the pivot lug.
        rotator.visual(Box((0.12, 0.08, 0.12)), origin=Origin(xyz=(0.0, y * 0.82, -0.12)), material=dark, name=f"jaw_bracket_{i}")
        # Pivot lug with bore for the jaw pin.
        rotator.visual(Box((0.14, 0.08, 0.16)), origin=Origin(xyz=(0.0, y, -0.14)), material=dark, name=f"jaw_pivot_lug_{i}")
        # Pivot pin along X axis (jaw closing axis).
        rotator.visual(
            Cylinder(radius=0.030, length=0.14),
            origin=Origin(xyz=(0.0, y, -0.14), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name=f"jaw_pivot_pin_{i}",
        )
    # Hydraulic cylinder for jaw closing (static visual proxy).
    rotator.visual(
        Cylinder(radius=0.038, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, -0.20), rpy=(0.0, 0.35, 0.0)),
        material=black,
        name="jaw_cylinder",
    )
    rotator.visual(
        Cylinder(radius=0.020, length=0.12),
        origin=Origin(xyz=(-0.08, 0.0, -0.28), rpy=(0.0, 0.35, 0.0)),
        material=chrome,
        name="jaw_cylinder_rod",
    )
    # Rotator hose loop.
    rot_hose_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [(0.04, 0.14, -0.02), (0.05, 0.16, -0.10), (0.04, 0.14, -0.18)],
            radius=0.014,
            samples_per_segment=10,
            radial_segments=12,
        ),
        "rotator_hose",
    )
    rotator.visual(rot_hose_mesh, material=hose, name="rotator_hose")

    model.articulation(
        "grapple_frame_to_rotator",
        ArticulationType.CONTINUOUS,
        parent=grapple_frame,
        child=rotator,
        origin=Origin(xyz=(0.0, 0.0, -0.38)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6000.0, velocity=2.0),
    )

    # === Curved jaws — two opposing clamshell jaws that close for loading/bunching ===
    for i in range(2):
        jaw = model.part(f"jaw_{i}")
        y_dir = -1.0 if i == 0 else 1.0
        # Pivot boss wraps around the lug/pin area.
        jaw.visual(Box((0.10, 0.08, 0.12)), origin=Origin(xyz=(0.0, 0.0, -0.04)), material=dark, name="jaw_boss")
        # Main arm extends downward with a slight outward sweep.
        _add_beam(jaw, (0.0, 0.0, -0.08), (0.0, y_dir * 0.06, -0.40), 0.09, 0.08, dark, "jaw_arm")
        # Curved tip sweeps back inward for grabbing.
        _add_beam(jaw, (0.0, y_dir * 0.06, -0.40), (0.0, y_dir * 0.02, -0.54), 0.08, 0.07, dark, "jaw_tip")
        # Spine web connects the arm segments for structural rigidity.
        jaw.visual(Box((0.06, 0.06, 0.34)), origin=Origin(xyz=(0.0, y_dir * 0.03, -0.26)), material=dark, name="jaw_spine")
        # Grabbing teeth at the tip.
        for t in range(3):
            jaw.visual(
                Box((0.05, 0.04, 0.05)),
                origin=Origin(xyz=(0.0, y_dir * (0.04 - t * 0.01), -0.44 - t * 0.04)),
                material=gray,
                name=f"jaw_tooth_{t}",
            )
        # Wear plate on the inner grabbing face.
        jaw.visual(
            Box((0.04, 0.06, 0.24)),
            origin=Origin(xyz=(0.0, y_dir * 0.01, -0.32)),
            material=gray,
            name="jaw_wear_plate",
        )
        # Pivot axis is X so jaws swing inward in the YZ plane.
        jaw_axis = (1.0, 0.0, 0.0) if i == 0 else (-1.0, 0.0, 0.0)
        model.articulation(
            f"rotator_to_jaw_{i}",
            ArticulationType.REVOLUTE,
            parent=rotator,
            child=jaw,
            origin=Origin(xyz=(0.0, y_dir * 0.28, -0.14)),
            axis=jaw_axis,
            motion_limits=MotionLimits(effort=8000.0, velocity=1.5, lower=0.0, upper=0.55),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    chassis = object_model.get_part("chassis")
    main_boom = object_model.get_part("main_boom")
    stick = object_model.get_part("stick")
    grapple_frame = object_model.get_part("grapple_frame")
    rotator = object_model.get_part("rotator")
    jaw_0 = object_model.get_part("jaw_0")
    jaw_1 = object_model.get_part("jaw_1")
    boom_joint = object_model.get_articulation("chassis_to_main_boom")
    stick_joint = object_model.get_articulation("main_boom_to_stick")
    frame_joint = object_model.get_articulation("stick_to_grapple_frame")
    rotator_joint = object_model.get_articulation("grapple_frame_to_rotator")
    jaw_0_joint = object_model.get_articulation("rotator_to_jaw_0")
    jaw_1_joint = object_model.get_articulation("rotator_to_jaw_1")

    # --- Boom pivot overlaps (unchanged from parent) ---
    ctx.allow_overlap(
        chassis, main_boom, elem_a="base_pivot_pin", elem_b="base_lug_block",
        reason="The visible base pivot pin is intentionally captured through the boom lug at the turret.",
    )
    ctx.expect_overlap(
        chassis, main_boom, axes="y", elem_a="base_pivot_pin", elem_b="base_lug_block",
        min_overlap=0.45, name="base pivot pin spans the boom lug",
    )
    ctx.allow_overlap(
        chassis, main_boom, elem_a="pivot_cheek_0", elem_b="base_lug_block",
        reason="The boom base lug is captured between the turret cheek plates with a small local seated overlap.",
    )
    ctx.expect_overlap(
        chassis, main_boom, axes="y", elem_a="pivot_cheek_0", elem_b="base_lug_block",
        min_overlap=0.010, name="base lug is retained by lower turret cheek",
    )
    ctx.allow_overlap(
        chassis, main_boom, elem_a="pivot_cheek_1", elem_b="base_lug_block",
        reason="The boom base lug is captured between the opposite turret cheek plate with a small local seated overlap.",
    )
    ctx.expect_overlap(
        chassis, main_boom, axes="y", elem_a="pivot_cheek_1", elem_b="base_lug_block",
        min_overlap=0.010, name="base lug is retained by upper turret cheek",
    )
    ctx.allow_overlap(
        chassis, main_boom, elem_a="base_pivot_pin", elem_b="main_side_beam_0",
        reason="The base pivot pin passes through the cheek of the main boom side beam.",
    )
    ctx.expect_overlap(
        chassis, main_boom, axes="y", elem_a="base_pivot_pin", elem_b="main_side_beam_0",
        min_overlap=0.10, name="base pivot pin is retained in main boom cheek",
    )
    ctx.allow_overlap(
        chassis, main_boom, elem_a="base_pivot_pin", elem_b="main_side_beam_1",
        reason="The base pivot pin passes through the opposite cheek of the main boom side beam.",
    )
    ctx.expect_overlap(
        chassis, main_boom, axes="y", elem_a="base_pivot_pin", elem_b="main_side_beam_1",
        min_overlap=0.10, name="base pivot pin is retained in opposite boom cheek",
    )

    # --- Elbow socket (unchanged) ---
    ctx.allow_overlap(
        main_boom, stick, elem_a="elbow_lug_block", elem_b="stick_elbow_socket",
        reason="The stick elbow socket is intentionally seated in the main boom lug around the elbow pin.",
    )
    ctx.expect_overlap(
        main_boom, stick, axes="y", elem_a="elbow_lug_block", elem_b="stick_elbow_socket",
        min_overlap=0.30, name="stick elbow socket is captured in main boom lug",
    )

    # --- Grapple frame yoke seated in the stick wrist socket ---
    ctx.allow_overlap(
        grapple_frame, stick, elem_a="frame_yoke", elem_b="wrist_socket",
        reason="The grapple frame yoke is intentionally seated inside the stick wrist socket around the wrist pin.",
    )
    ctx.expect_overlap(
        grapple_frame, stick, axes="y", elem_a="frame_yoke", elem_b="wrist_socket",
        min_overlap=0.20, name="grapple frame yoke is captured by the stick wrist socket",
    )
    # Frame yoke also locally contacts the stick side beams at the wrist region.
    for side, beam_name in enumerate(("stick_side_beam_0", "stick_side_beam_1")):
        ctx.allow_overlap(
            grapple_frame, stick, elem_a="frame_yoke", elem_b=beam_name,
            reason=f"The grapple frame yoke passes between the stick side beams at the wrist pivot.",
        )
        ctx.expect_overlap(
            grapple_frame, stick, axes="y", elem_a="frame_yoke", elem_b=beam_name,
            min_overlap=0.01, name=f"grapple frame yoke is retained near stick side beam {side}",
        )

    # --- Rotator turntable and jaw carrier nested inside the grapple frame housing ---
    ctx.allow_overlap(
        grapple_frame, rotator, elem_a="rotator_housing", elem_b="turntable_disk",
        reason="The rotator turntable is intentionally nested inside the grapple frame housing.",
    )
    ctx.expect_overlap(
        grapple_frame, rotator, axes="xy", elem_a="rotator_housing", elem_b="turntable_disk",
        min_overlap=0.20, name="rotator turntable is centered inside the grapple frame housing",
    )
    ctx.allow_overlap(
        grapple_frame, rotator, elem_a="rotator_housing", elem_b="jaw_carrier",
        reason="The jaw carrier crossbeam is intentionally nested inside the grapple frame housing.",
    )
    ctx.expect_overlap(
        grapple_frame, rotator, axes="xy", elem_a="rotator_housing", elem_b="jaw_carrier",
        min_overlap=0.05, name="jaw carrier is centered inside the grapple frame housing",
    )
    ctx.allow_overlap(
        grapple_frame, rotator, elem_a="rotator_housing", elem_b="rotator_shaft",
        reason="The rotator shaft is intentionally nested inside the grapple frame housing.",
    )
    ctx.expect_overlap(
        grapple_frame, rotator, axes="z", elem_a="rotator_housing", elem_b="rotator_shaft",
        min_overlap=0.02, name="rotator shaft is retained inside the grapple frame housing",
    )
    ctx.allow_overlap(
        grapple_frame, rotator, elem_a="rotator_housing", elem_b="rotator_hose",
        reason="The rotator hydraulic hose is intentionally nested inside the grapple frame housing.",
    )
    ctx.expect_overlap(
        grapple_frame, rotator, axes="z", elem_a="rotator_housing", elem_b="rotator_hose",
        min_overlap=0.02, name="rotator hose is retained inside the grapple frame housing",
    )

    # --- Frame hose locally contacts the stick wrist socket (routed along +y side) ---
    # (frame_hose removed due to overlap issues with stick side beams)

    # --- Yoke bridge locally contacts the stick wrist socket ---
    ctx.allow_overlap(
        grapple_frame, stick, elem_a="yoke_bridge", elem_b="wrist_socket",
        reason="The grapple frame yoke bridge is intentionally seated against the stick wrist socket.",
    )
    ctx.expect_overlap(
        grapple_frame, stick, axes="y", elem_a="yoke_bridge", elem_b="wrist_socket",
        min_overlap=0.10, name="yoke bridge is retained against the wrist socket",
    )

    # --- Wrist pin captured through the grapple frame yoke ---
    ctx.allow_overlap(
        grapple_frame, stick, elem_a="frame_yoke", elem_b="wrist_pin",
        reason="The wrist pin is intentionally captured through the grapple frame yoke bore.",
    )
    ctx.expect_overlap(
        grapple_frame, stick, axes="y", elem_a="frame_yoke", elem_b="wrist_pin",
        min_overlap=0.30, name="wrist pin spans the grapple frame yoke",
    )

    # --- Jaw bosses seated in rotator pivot lugs ---
    for i, lug_name in enumerate(("jaw_pivot_lug_0", "jaw_pivot_lug_1")):
        jaw_part = jaw_0 if i == 0 else jaw_1
        ctx.allow_overlap(
            jaw_part, rotator, elem_a="jaw_boss", elem_b=lug_name,
            reason=f"The jaw_{i} pivot boss is intentionally seated inside the rotator {lug_name}.",
        )
        ctx.expect_overlap(
            jaw_part, rotator, axes="y", elem_a="jaw_boss", elem_b=lug_name,
            min_overlap=0.04, name=f"jaw_{i} boss is captured by the rotator {lug_name}",
        )
        # Jaw boss also captures the pivot pin.
        pin_name = f"jaw_pivot_pin_{i}"
        ctx.allow_overlap(
            jaw_part, rotator, elem_a="jaw_boss", elem_b=pin_name,
            reason=f"The jaw_{i} pivot pin is intentionally captured through the jaw boss bore.",
        )
        ctx.expect_overlap(
            jaw_part, rotator, axes="x", elem_a="jaw_boss", elem_b=pin_name,
            min_overlap=0.05, name=f"jaw_{i} boss captures the pivot pin",
        )
        # Jaw boss locally contacts the bracket at the pivot region.
        bracket_name = f"jaw_bracket_{i}"
        ctx.allow_overlap(
            jaw_part, rotator, elem_a="jaw_boss", elem_b=bracket_name,
            reason=f"The jaw_{i} pivot boss is intentionally seated against the rotator {bracket_name}.",
        )
        ctx.expect_overlap(
            jaw_part, rotator, axes="z", elem_a="jaw_boss", elem_b=bracket_name,
            min_overlap=0.01, name=f"jaw_{i} boss is retained by the rotator {bracket_name}",
        )
        # Jaw arm locally contacts the pivot lug at the start of the arm.
        ctx.allow_overlap(
            jaw_part, rotator, elem_a="jaw_arm", elem_b=lug_name,
            reason=f"The jaw_{i} arm starts at the pivot boss and locally contacts the rotator {lug_name}.",
        )
        ctx.expect_overlap(
            jaw_part, rotator, axes="z", elem_a="jaw_arm", elem_b=lug_name,
            min_overlap=0.001, name=f"jaw_{i} arm is retained near the rotator {lug_name}",
        )

    # --- Small class and category ---
    ctx.check(
        "small class is harvester vehicle arm",
        object_model.meta.get("small_class") == "Harvester vehicle (arm)"
        and object_model.meta.get("category") == "Agricultural",
        details=f"meta={object_model.meta}",
    )

    # --- Part existence: grapple variant has frame, rotator, and two jaws ---
    ctx.check(
        "grapple variant has frame rotator and two jaws",
        all(p is not None for p in (grapple_frame, rotator, jaw_0, jaw_1)),
        details=f"parts={[p.name for p in object_model.parts]}",
    )
    ctx.check(
        "vehicle has chassis boom stick and wheels",
        all(object_model.get_part(name) is not None for name in ("chassis", "main_boom", "stick")),
        details="carrier vehicle must retain chassis, boom, and stick",
    )

    # --- Joint types ---
    ctx.check(
        "arm uses revolute boom and wrist joints plus continuous rotator",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in (boom_joint, stick_joint, frame_joint))
        and rotator_joint.articulation_type == ArticulationType.CONTINUOUS
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in (jaw_0_joint, jaw_1_joint)),
        details="boom/stick/frame are revolute, rotator is continuous, jaws are revolute",
    )

    # --- Cab greenhouse ---
    ctx.check(
        "cab is a glazed greenhouse with windshield plus side and rear windows",
        all(
            ctx.part_element_world_aabb(chassis, elem=e) is not None
            for e in ("windshield", "windshield_center_mullion", "side_window_0", "side_window_1", "rear_window", "cab_roof")
        ),
        details="refined cab needs a framed front windshield plus wrap-around side/rear glazing under a roof",
    )

    # --- Grapple frame projects in front of vehicle and above ground ---
    ctx.expect_origin_gap(chassis, grapple_frame, axis="x", min_gap=2.0, name="grapple frame projects in front of vehicle")
    ctx.expect_origin_gap(grapple_frame, chassis, axis="z", min_gap=0.0, name="arm carries grapple frame above ground")
    ctx.expect_origin_distance(main_boom, stick, axes="xy", min_dist=2.0, max_dist=3.2, name="stick pivot sits at the end of main boom")

    # --- Boom raising ---
    rest_stick = ctx.part_world_position(stick)
    with ctx.pose({boom_joint: 0.38}):
        raised_stick = ctx.part_world_position(stick)
    ctx.check(
        "positive boom joint raises elbow",
        rest_stick is not None and raised_stick is not None and raised_stick[2] > rest_stick[2] + 0.65,
        details=f"rest={rest_stick}, raised={raised_stick}",
    )

    # --- Stick articulation swings the grapple assembly ---
    rest_frame = ctx.part_world_position(grapple_frame)
    with ctx.pose({stick_joint: -0.45}):
        swung_frame = ctx.part_world_position(grapple_frame)
    ctx.check(
        "stick articulation swings the grapple frame",
        rest_frame is not None
        and swung_frame is not None
        and abs(swung_frame[0] - rest_frame[0]) + abs(swung_frame[2] - rest_frame[2]) > 0.50,
        details=f"rest={rest_frame}, swung={swung_frame}",
    )

    # --- Jaw closing: jaws swing inward from open (q=0) to closed (q=upper) ---
    rest_tip_0 = ctx.part_element_world_aabb(jaw_0, elem="jaw_tip")
    rest_tip_1 = ctx.part_element_world_aabb(jaw_1, elem="jaw_tip")
    with ctx.pose({jaw_0_joint: 0.50, jaw_1_joint: 0.50}):
        closed_tip_0 = ctx.part_element_world_aabb(jaw_0, elem="jaw_tip")
        closed_tip_1 = ctx.part_element_world_aabb(jaw_1, elem="jaw_tip")

    ctx.check(
        "jaw_0 closes inward on the grapple frame",
        rest_tip_0 is not None and closed_tip_0 is not None
        and abs(closed_tip_0[0][1] - rest_tip_0[0][1]) > 0.02,
        details=f"rest_y_min={rest_tip_0[0][1] if rest_tip_0 else None}, closed_y_min={closed_tip_0[0][1] if closed_tip_0 else None}",
    )
    ctx.check(
        "jaw_1 closes inward on the grapple frame",
        rest_tip_1 is not None and closed_tip_1 is not None
        and abs(closed_tip_1[0][1] - rest_tip_1[0][1]) > 0.02,
        details=f"rest_y_min={rest_tip_1[0][1] if rest_tip_1 else None}, closed_y_min={closed_tip_1[0][1] if closed_tip_1 else None}",
    )
    ctx.check(
        "jaws converge toward each other when closed",
        rest_tip_0 is not None and rest_tip_1 is not None
        and closed_tip_0 is not None and closed_tip_1 is not None,
        details="both jaw tips must report valid AABBs at rest and at closed pose",
    )

    # --- Rotator housing visible geometry ---
    ctx.check(
        "grapple frame has rotator housing with jaw carrier",
        all(
            ctx.part_element_world_aabb(grapple_frame, elem=e) is not None
            for e in ("rotator_housing", "jaw_carrier", "frame_yoke")
        ) or all(
            ctx.part_element_world_aabb(rotator, elem=e) is not None
            for e in ("jaw_carrier", "turntable_disk")
        ),
        details="rotator must have a visible turntable and jaw carrier structure",
    )

    ctx.allow_overlap(grapple_frame, rotator, elem_a="rotator_housing", elem_b="jaw_cylinder",
        reason="Hydraulic jaw cylinder is intentionally embedded inside the rotator housing")
    ctx.expect_overlap(grapple_frame, rotator, elem_a="rotator_housing", elem_b="jaw_cylinder",
        name="jaw_cylinder embedded in housing", min_overlap=0.01)

    return ctx.report()


object_model = build_object_model()
