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
            "description": "Forestry-style agricultural harvester vehicle with articulated harvesting arm and processor head.",
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

    head = model.part("harvester_head")
    head.visual(Cylinder(radius=0.14, length=0.36), origin=Origin(xyz=(-0.16, 0.0, -0.34)), material=dark, name="wrist_rotator")
    head.visual(Box((0.72, 0.58, 0.38)), origin=Origin(xyz=(-0.42, 0.0, -0.50)), material=yellow, name="processor_housing")
    head.visual(Box((0.66, 0.10, 0.30)), origin=Origin(xyz=(-0.42, -0.34, -0.51)), material=dark, name="side_cover")
    head.visual(Box((0.66, 0.10, 0.30)), origin=Origin(xyz=(-0.42, 0.34, -0.51)), material=dark, name="opposite_cover")
    _add_pin_y(head, (-0.40, 0.0, -0.75), 0.76, 0.105, rubber, "lower_feed_roller")
    _add_pin_y(head, (-0.33, 0.0, -0.30), 0.70, 0.090, rubber, "upper_feed_roller")
    head.visual(Box((0.72, 0.08, 0.06)), origin=Origin(xyz=(-0.80, 0.0, -0.92), rpy=(0.0, -0.18, 0.0)), material=gray, name="saw_bar")
    head.visual(Box((0.10, 0.64, 0.10)), origin=Origin(xyz=(-0.12, 0.0, -0.14)), material=boom_gray, name="head_yoke")
    head.visual(Box((0.24, 0.50, 0.18)), origin=Origin(xyz=(-0.25, 0.0, -0.25)), material=boom_gray, name="yoke_housing_bridge")
    head.visual(Box((0.12, 0.10, 0.55)), origin=Origin(xyz=(-0.18, -0.38, -0.50), rpy=(0.0, -0.28, 0.0)), material=dark, name="fixed_delimbing_knife_0")
    head.visual(Box((0.12, 0.10, 0.55)), origin=Origin(xyz=(-0.18, 0.38, -0.50), rpy=(0.0, -0.28, 0.0)), material=dark, name="fixed_delimbing_knife_1")
    head.visual(Box((0.10, 0.28, 0.18)), origin=Origin(xyz=(-0.82, 0.0, -0.25)), material=dark, name="grapple_hinge_plate")
    head_hose_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [(-0.04, 0.44, -0.04), (-0.18, 0.48, -0.26), (-0.44, 0.47, -0.60)],
            radius=0.018,
            samples_per_segment=14,
            radial_segments=14,
        ),
        "head_hose_loop",
    )
    head.visual(head_hose_mesh, material=hose, name="head_hose_loop")
    head.visual(Box((0.12, 0.20, 0.10)), origin=Origin(xyz=(-0.24, 0.38, -0.42)), material=dark, name="head_hose_clamp")

    model.articulation(
        "stick_to_head",
        ArticulationType.REVOLUTE,
        parent=stick,
        child=head,
        origin=Origin(xyz=(-2.46, 0.0, -1.18)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=16000.0, velocity=1.0, lower=-0.65, upper=0.65),
    )

    grapple = model.part("grapple_arm")
    grapple.visual(Box((0.18, 0.68, 0.12)), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=dark, name="grapple_cross_tube")
    for side, y in enumerate((-0.27, 0.27)):
        _add_beam(grapple, (0.0, y, 0.0), (-0.34, y, -0.42), 0.08, 0.12, boom_gray, f"grapple_link_{side}")
        _add_beam(grapple, (-0.34, y, -0.42), (-0.54, y, -0.73), 0.07, 0.10, dark, f"curved_tine_{side}")
    model.articulation(
        "head_to_grapple_arm",
        ArticulationType.REVOLUTE,
        parent=head,
        child=grapple,
        origin=Origin(xyz=(-0.86, 0.0, -0.22)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12000.0, velocity=1.2, lower=-0.25, upper=0.70),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    chassis = object_model.get_part("chassis")
    main_boom = object_model.get_part("main_boom")
    stick = object_model.get_part("stick")
    head = object_model.get_part("harvester_head")
    grapple = object_model.get_part("grapple_arm")
    boom_joint = object_model.get_articulation("chassis_to_main_boom")
    stick_joint = object_model.get_articulation("main_boom_to_stick")
    head_joint = object_model.get_articulation("stick_to_head")
    grapple_joint = object_model.get_articulation("head_to_grapple_arm")

    ctx.allow_overlap(
        chassis,
        main_boom,
        elem_a="base_pivot_pin",
        elem_b="base_lug_block",
        reason="The visible base pivot pin is intentionally captured through the boom lug at the turret.",
    )
    ctx.expect_overlap(
        chassis,
        main_boom,
        axes="y",
        elem_a="base_pivot_pin",
        elem_b="base_lug_block",
        min_overlap=0.45,
        name="base pivot pin spans the boom lug",
    )
    ctx.allow_overlap(
        chassis,
        main_boom,
        elem_a="pivot_cheek_0",
        elem_b="base_lug_block",
        reason="The boom base lug is captured between the turret cheek plates with a small local seated overlap.",
    )
    ctx.expect_overlap(
        chassis,
        main_boom,
        axes="y",
        elem_a="pivot_cheek_0",
        elem_b="base_lug_block",
        min_overlap=0.010,
        name="base lug is retained by lower turret cheek",
    )
    ctx.allow_overlap(
        chassis,
        main_boom,
        elem_a="pivot_cheek_1",
        elem_b="base_lug_block",
        reason="The boom base lug is captured between the opposite turret cheek plate with a small local seated overlap.",
    )
    ctx.expect_overlap(
        chassis,
        main_boom,
        axes="y",
        elem_a="pivot_cheek_1",
        elem_b="base_lug_block",
        min_overlap=0.010,
        name="base lug is retained by upper turret cheek",
    )
    ctx.allow_overlap(
        chassis,
        main_boom,
        elem_a="base_pivot_pin",
        elem_b="main_side_beam_0",
        reason="The base pivot pin passes through the cheek of the main boom side beam.",
    )
    ctx.expect_overlap(
        chassis,
        main_boom,
        axes="y",
        elem_a="base_pivot_pin",
        elem_b="main_side_beam_0",
        min_overlap=0.10,
        name="base pivot pin is retained in main boom cheek",
    )
    ctx.allow_overlap(
        chassis,
        main_boom,
        elem_a="base_pivot_pin",
        elem_b="main_side_beam_1",
        reason="The base pivot pin passes through the opposite cheek of the main boom side beam.",
    )
    ctx.expect_overlap(
        chassis,
        main_boom,
        axes="y",
        elem_a="base_pivot_pin",
        elem_b="main_side_beam_1",
        min_overlap=0.10,
        name="base pivot pin is retained in opposite boom cheek",
    )
    ctx.allow_overlap(
        head,
        stick,
        elem_a="head_yoke",
        elem_b="wrist_socket",
        reason="The wrist yoke is intentionally seated inside the stick socket around the wrist pivot.",
    )
    ctx.expect_overlap(
        head,
        stick,
        axes="y",
        elem_a="head_yoke",
        elem_b="wrist_socket",
        min_overlap=0.30,
        name="wrist yoke is captured by the stick socket",
    )
    ctx.allow_overlap(
        main_boom,
        stick,
        elem_a="elbow_lug_block",
        elem_b="stick_elbow_socket",
        reason="The stick elbow socket is intentionally seated in the main boom lug around the elbow pin.",
    )
    ctx.expect_overlap(
        main_boom,
        stick,
        axes="y",
        elem_a="elbow_lug_block",
        elem_b="stick_elbow_socket",
        min_overlap=0.30,
        name="stick elbow socket is captured in main boom lug",
    )
    ctx.allow_overlap(
        head,
        grapple,
        elem_a="grapple_hinge_plate",
        elem_b="grapple_cross_tube",
        reason="The grapple hinge tube is locally captured by the head hinge plate.",
    )
    ctx.expect_overlap(
        head,
        grapple,
        axes="y",
        elem_a="grapple_hinge_plate",
        elem_b="grapple_cross_tube",
        min_overlap=0.18,
        name="grapple hinge tube spans the hinge plate",
    )

    ctx.check(
        "small class is harvester vehicle arm",
        object_model.meta.get("small_class") == "Harvester vehicle (arm)"
        and object_model.meta.get("category") == "Agricultural",
        details=f"meta={object_model.meta}",
    )
    ctx.check(
        "visible vehicle has chassis wheels boom stick head and grapple",
        all(object_model.get_part(name) is not None for name in ("chassis", "main_boom", "stick", "harvester_head", "grapple_arm"))
        and len(object_model.parts) >= 11,
        details=f"parts={[p.name for p in object_model.parts]}",
    )
    ctx.check(
        "arm uses multiple revolute boom joints",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in (boom_joint, stick_joint, head_joint, grapple_joint)),
        details="boom, stick, wrist, and grapple joints should all be revolute mechanisms",
    )
    ctx.check(
        "cab is a glazed greenhouse with windshield plus side and rear windows",
        all(
            ctx.part_element_world_aabb(chassis, elem=e) is not None
            for e in ("windshield", "windshield_center_mullion", "side_window_0", "side_window_1", "rear_window", "cab_roof")
        ),
        details="refined cab needs a framed front windshield plus wrap-around side/rear glazing under a roof",
    )

    ctx.expect_origin_gap(chassis, head, axis="x", min_gap=2.5, name="harvesting head projects in front of vehicle")
    ctx.expect_origin_gap(head, chassis, axis="z", min_gap=0.35, name="arm carries head above ground and under boom")
    ctx.expect_origin_distance(main_boom, stick, axes="xy", min_dist=2.0, max_dist=3.2, name="stick pivot sits at the end of main boom")

    rest_stick = ctx.part_world_position(stick)
    rest_head = ctx.part_world_position(head)
    with ctx.pose({boom_joint: 0.38}):
        raised_stick = ctx.part_world_position(stick)
    ctx.check(
        "positive boom joint raises elbow",
        rest_stick is not None and raised_stick is not None and raised_stick[2] > rest_stick[2] + 0.65,
        details=f"rest={rest_stick}, raised={raised_stick}",
    )

    with ctx.pose({stick_joint: -0.45}):
        curled_head = ctx.part_world_position(head)
    ctx.check(
        "stick articulation swings the harvester head",
        rest_head is not None
        and curled_head is not None
        and abs(curled_head[0] - rest_head[0]) + abs(curled_head[2] - rest_head[2]) > 0.50,
        details=f"rest={rest_head}, curled={curled_head}",
    )

    rest_grapple = ctx.part_element_world_aabb(grapple, elem="curved_tine_0")
    with ctx.pose({grapple_joint: 0.55}):
        closed_grapple = ctx.part_element_world_aabb(grapple, elem="curved_tine_0")
    ctx.check(
        "grapple arm has an actuated clamp motion",
        rest_grapple is not None
        and closed_grapple is not None
        and abs(closed_grapple[0][0] - rest_grapple[0][0]) + abs(closed_grapple[0][2] - rest_grapple[0][2]) > 0.10,
        details=f"rest={rest_grapple}, moved={closed_grapple}",
    )

    return ctx.report()


object_model = build_object_model()
