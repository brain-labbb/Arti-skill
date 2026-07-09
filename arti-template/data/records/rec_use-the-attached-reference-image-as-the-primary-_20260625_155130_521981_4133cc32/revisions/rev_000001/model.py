from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    VentGrilleFrame,
    VentGrilleGeometry,
    VentGrilleMounts,
    VentGrilleSleeve,
    VentGrilleSlats,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="agricultural_tractor",
        meta={
            "category": "Agricultural",
            "small_class": "Tractor",
            "description": "Detailed old blue farm tractor with cab, hood, grille, fenders, hitch, rear cargo trailer bed, rotating tires, and steerable front axle.",
        },
    )

    painted_blue = Material("weathered_blue_painted_steel", rgba=(0.23, 0.42, 0.50, 1.0))
    dark_blue = Material("dark_chipped_blue_steel", rgba=(0.08, 0.16, 0.20, 1.0))
    black = Material("aged_black_rubber", rgba=(0.018, 0.017, 0.015, 1.0))
    dark_metal = Material("oily_dark_steel", rgba=(0.06, 0.065, 0.06, 1.0))
    worn_metal = Material("worn_galvanized_steel", rgba=(0.46, 0.50, 0.50, 1.0))
    glass = Material("slightly_smoky_cab_glass", rgba=(0.65, 0.83, 0.90, 0.36))
    amber = Material("amber_lens", rgba=(1.0, 0.58, 0.12, 1.0))
    light = Material("ribbed_clear_lens", rgba=(0.86, 0.88, 0.78, 1.0))
    soil_wear = Material("soil_polished_worn_metal", rgba=(0.28, 0.27, 0.24, 1.0))
    weathered_wood = Material("weathered_trailer_wood", rgba=(0.28, 0.22, 0.17, 1.0))

    chassis = model.part("chassis")

    # Load-carrying frame and engine/body shell.  X is forward, Y is across the axle, Z is up.
    chassis.visual(Box((2.35, 0.62, 0.22)), origin=Origin(xyz=(0.15, 0.0, 0.74)), material=dark_metal, name="ladder_frame")
    chassis.visual(Box((0.92, 1.02, 0.18)), origin=Origin(xyz=(-0.35, 0.0, 0.91)), material=painted_blue, name="operator_platform")
    chassis.visual(Box((1.25, 0.62, 0.18)), origin=Origin(xyz=(0.68, 0.0, 0.86)), material=dark_blue, name="engine_belly")
    chassis.visual(Box((1.10, 0.74, 0.48)), origin=Origin(xyz=(0.72, 0.0, 1.14)), material=painted_blue, name="hood")
    chassis.visual(Box((0.18, 0.82, 0.64)), origin=Origin(xyz=(1.36, 0.0, 1.09)), material=painted_blue, name="nose_surround")
    chassis.visual(Box((0.36, 0.86, 0.18)), origin=Origin(xyz=(1.22, 0.0, 0.64)), material=dark_blue, name="front_bolster")
    chassis.visual(Box((0.22, 0.74, 0.05)), origin=Origin(xyz=(1.47, 0.0, 0.62)), material=worn_metal, name="front_bumper")

    grille = VentGrilleGeometry(
        (0.52, 0.56),
        frame=0.035,
        face_thickness=0.010,
        duct_depth=0.034,
        slat_pitch=0.030,
        slat_width=0.010,
        slat_angle_deg=0.0,
        corner_radius=0.012,
        slats=VentGrilleSlats(profile="boxed", direction="down", divider_count=6, divider_width=0.006),
        frame_profile=VentGrilleFrame(style="beveled", depth=0.006),
        mounts=VentGrilleMounts(style="holes", inset=0.028, hole_diameter=0.012),
        sleeve=VentGrilleSleeve(style="short", depth=0.020, wall=0.004),
    )
    chassis.visual(
        mesh_from_geometry(grille, "radiator_grille"),
        origin=Origin(xyz=(1.455, 0.0, 1.10), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="radiator_grille",
    )

    # Raised seams, access panels, fastener strips, and worn lower panels.
    chassis.visual(Box((0.90, 0.018, 0.018)), origin=Origin(xyz=(0.74, 0.370, 1.37)), material=dark_metal, name="hood_side_seam_0")
    chassis.visual(Box((0.90, 0.018, 0.018)), origin=Origin(xyz=(0.74, -0.370, 1.37)), material=dark_metal, name="hood_side_seam_1")
    chassis.visual(Box((0.72, 0.035, 0.014)), origin=Origin(xyz=(0.72, 0.0, 1.385)), material=dark_metal, name="hood_center_seam")
    chassis.visual(Box((0.06, 0.69, 0.025)), origin=Origin(xyz=(0.20, 0.0, 1.05)), material=dark_metal, name="hood_rear_panel_gap")
    chassis.visual(Box((0.12, 0.74, 0.045)), origin=Origin(xyz=(1.28, 0.0, 0.80)), material=soil_wear, name="front_wear_strip")

    # Rear axle housing and final-drive bosses stop near the wheel inner faces.
    chassis.visual(Cylinder(radius=0.095, length=1.30), origin=Origin(xyz=(-0.65, 0.0, 0.65), rpy=(-pi / 2.0, 0.0, 0.0)), material=dark_metal, name="rear_axle_housing")
    for side, y in enumerate((-0.61, 0.61)):
        chassis.visual(Cylinder(radius=0.18, length=0.10), origin=Origin(xyz=(-0.65, y, 0.65), rpy=(-pi / 2.0, 0.0, 0.0)), material=dark_blue, name=f"final_drive_{side}")
    chassis.visual(Cylinder(radius=0.070, length=0.28), origin=Origin(xyz=(-0.65, -0.77, 0.65), rpy=(-pi / 2.0, 0.0, 0.0)), material=dark_metal, name="rear_axle_stub_0")
    chassis.visual(Cylinder(radius=0.070, length=0.28), origin=Origin(xyz=(-0.65, 0.77, 0.65), rpy=(-pi / 2.0, 0.0, 0.0)), material=dark_metal, name="rear_axle_stub_1")

    # Rear fenders, with inner apron brackets tying them to the operator deck.
    chassis.visual(Box((0.82, 0.50, 0.12)), origin=Origin(xyz=(-0.68, -0.78, 1.39)), material=painted_blue, name="rear_fender_top_0")
    chassis.visual(Box((0.78, 0.08, 0.56)), origin=Origin(xyz=(-0.66, -0.54, 1.11)), material=dark_blue, name="rear_fender_apron_0")
    chassis.visual(Box((0.10, 0.42, 0.05)), origin=Origin(xyz=(-0.33, -0.78, 1.30)), material=dark_metal, name="fender_step_brace_0")
    chassis.visual(Cylinder(radius=0.018, length=0.30), origin=Origin(xyz=(-0.92, -0.62, 1.33), rpy=(0.0, pi / 2.0, 0.0)), material=dark_metal, name="fender_handrail_0")
    chassis.visual(Box((0.82, 0.50, 0.12)), origin=Origin(xyz=(-0.68, 0.78, 1.39)), material=painted_blue, name="rear_fender_top_1")
    chassis.visual(Box((0.78, 0.08, 0.56)), origin=Origin(xyz=(-0.66, 0.54, 1.11)), material=dark_blue, name="rear_fender_apron_1")
    chassis.visual(Box((0.10, 0.42, 0.05)), origin=Origin(xyz=(-0.33, 0.78, 1.30)), material=dark_metal, name="fender_step_brace_1")
    chassis.visual(Cylinder(radius=0.018, length=0.30), origin=Origin(xyz=(-0.92, 0.62, 1.33), rpy=(0.0, pi / 2.0, 0.0)), material=dark_metal, name="fender_handrail_1")

    # Cab frame: separate posts, roof lip, transparent panels, mirrors, and hardware all physically touch.
    chassis.visual(Box((1.10, 1.18, 0.13)), origin=Origin(xyz=(-0.35, 0.0, 2.32)), material=dark_blue, name="cab_roof")
    chassis.visual(Box((1.18, 1.25, 0.04)), origin=Origin(xyz=(-0.35, 0.0, 2.405)), material=worn_metal, name="cab_roof_rain_cap")
    for ix, x in enumerate((-0.82, 0.12)):
        for iy, y in enumerate((-0.50, 0.50)):
            chassis.visual(Box((0.075, 0.075, 1.40)), origin=Origin(xyz=(x, y, 1.62)), material=dark_blue, name=f"cab_post_{ix}_{iy}")
    chassis.visual(Box((0.06, 0.96, 0.92)), origin=Origin(xyz=(0.115, 0.0, 1.68)), material=glass, name="front_windshield")
    chassis.visual(Box((0.06, 0.96, 0.84)), origin=Origin(xyz=(-0.82, 0.0, 1.66)), material=glass, name="rear_cab_glass")
    chassis.visual(Box((0.94, 0.045, 0.94)), origin=Origin(xyz=(-0.35, 0.515, 1.68)), material=glass, name="side_window_0")
    chassis.visual(Box((0.94, 0.045, 0.94)), origin=Origin(xyz=(-0.35, -0.515, 1.68)), material=glass, name="side_window_1")
    chassis.visual(Box((0.38, 0.06, 0.55)), origin=Origin(xyz=(-0.55, 0.535, 1.38)), material=painted_blue, name="cab_door_panel_0")
    chassis.visual(Box((0.38, 0.06, 0.55)), origin=Origin(xyz=(-0.55, -0.535, 1.38)), material=painted_blue, name="cab_door_panel_1")
    for side, y in enumerate((-0.66, 0.66)):
        chassis.visual(Cylinder(radius=0.025, length=0.34), origin=Origin(xyz=(0.05, y, 1.93), rpy=(-pi / 2.0, 0.0, 0.0)), material=dark_metal, name=f"mirror_stem_{side}")
        chassis.visual(Box((0.035, 0.045, 0.18)), origin=Origin(xyz=(0.05, y * 1.10, 1.93)), material=dark_metal, name=f"side_mirror_{side}")

    # Seat, steering column, steering wheel, and controls.
    chassis.visual(Cylinder(radius=0.055, length=0.33), origin=Origin(xyz=(-0.43, 0.0, 1.10)), material=dark_metal, name="seat_pedestal")
    chassis.visual(Box((0.42, 0.46, 0.12)), origin=Origin(xyz=(-0.44, 0.0, 1.30)), material=black, name="seat_cushion")
    chassis.visual(Box((0.11, 0.46, 0.48)), origin=Origin(xyz=(-0.65, 0.0, 1.52)), material=black, name="seat_back")
    chassis.visual(Cylinder(radius=0.030, length=0.45), origin=Origin(xyz=(0.00, 0.0, 1.35), rpy=(0.0, -0.38, 0.0)), material=dark_metal, name="steering_column")
    chassis.visual(Cylinder(radius=0.18, length=0.035), origin=Origin(xyz=(-0.08, 0.0, 1.56), rpy=(0.0, -0.38, 0.0)), material=black, name="steering_wheel")
    chassis.visual(Box((0.10, 0.42, 0.08)), origin=Origin(xyz=(0.10, 0.0, 1.32)), material=dark_metal, name="instrument_panel")
    for i, y in enumerate((-0.14, 0.0, 0.14)):
        chassis.visual(Cylinder(radius=0.026, length=0.018), origin=Origin(xyz=(0.135, y, 1.35), rpy=(0.0, pi / 2.0, 0.0)), material=amber if i == 1 else light, name=f"dash_gauge_{i}")

    # Exhaust stack, lights, bumper brackets, and front work hardware.
    chassis.visual(Cylinder(radius=0.055, length=0.72), origin=Origin(xyz=(0.47, -0.38, 1.62)), material=dark_metal, name="exhaust_stack")
    chassis.visual(Cylinder(radius=0.085, length=0.26), origin=Origin(xyz=(0.47, -0.38, 1.47)), material=worn_metal, name="exhaust_muffler")
    chassis.visual(Box((0.16, 0.10, 0.035)), origin=Origin(xyz=(0.52, -0.38, 1.99)), material=dark_metal, name="exhaust_rain_cap")
    for side, y in enumerate((-0.49, 0.49)):
        chassis.visual(Cylinder(radius=0.105, length=0.055), origin=Origin(xyz=(1.49, y, 1.16), rpy=(0.0, pi / 2.0, 0.0)), material=light, name=f"front_headlamp_{side}")
        chassis.visual(Cylinder(radius=0.018, length=0.18), origin=Origin(xyz=(1.40, y * 0.92, 1.125), rpy=(0.0, pi / 2.0, 0.0)), material=dark_metal, name=f"headlamp_bracket_{side}")
        chassis.visual(Box((0.20, 0.13, 0.05)), origin=Origin(xyz=(1.38, y * 0.93, 1.16)), material=dark_metal, name=f"headlamp_mount_bar_{side}")
        chassis.visual(Box((0.03, 0.06, 0.11)), origin=Origin(xyz=(0.30, y * 0.76, 1.15)), material=amber, name=f"side_marker_{side}")

    # Rear drawbar hitch, three-point-look brackets, and a proportionally large cargo trailer bed.
    chassis.visual(Box((0.66, 0.12, 0.09)), origin=Origin(xyz=(-1.30, 0.0, 0.64)), material=soil_wear, name="drawbar")
    chassis.visual(Box((0.14, 0.24, 0.025)), origin=Origin(xyz=(-1.64, 0.0, 0.695)), material=dark_metal, name="drawbar_clevis_top_plate")
    chassis.visual(Box((0.14, 0.24, 0.025)), origin=Origin(xyz=(-1.64, 0.0, 0.585)), material=dark_metal, name="drawbar_clevis_bottom_plate")
    chassis.visual(Cylinder(radius=0.033, length=0.15), origin=Origin(xyz=(-1.64, 0.0, 0.64)), material=worn_metal, name="hitch_pin_lug")
    chassis.visual(Box((0.36, 0.06, 0.055)), origin=Origin(xyz=(-1.21, 0.29, 0.72), rpy=(0.0, 0.18, 0.0)), material=dark_metal, name="hitch_lift_arm_0")
    chassis.visual(Box((0.36, 0.06, 0.055)), origin=Origin(xyz=(-1.21, -0.29, 0.72), rpy=(0.0, 0.18, 0.0)), material=dark_metal, name="hitch_lift_arm_1")

    # Two-wheeled cargo trailer. Local coordinates are centered on the hitch pin
    # so the whole trailer assembly yaws around the connection point.
    trailer_frame = model.part("trailer_frame")
    trailer_frame.visual(Box((0.62, 0.075, 0.070)), origin=Origin(xyz=(-0.22, 0.0, 0.0)), material=dark_metal, name="trailer_tongue")
    trailer_frame.visual(Box((0.20, 0.20, 0.07)), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=dark_metal, name="trailer_hitch_coupler")
    trailer_frame.visual(Cylinder(radius=0.030, length=0.24), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=worn_metal, name="trailer_coupler_pin_bushing")
    trailer_frame.visual(Box((0.82, 0.055, 0.060)), origin=Origin(xyz=(-0.44, -0.25, 0.055), rpy=(0.0, 0.06, 0.55)), material=dark_metal, name="trailer_a_frame_0")
    trailer_frame.visual(Box((0.82, 0.055, 0.060)), origin=Origin(xyz=(-0.44, 0.25, 0.055), rpy=(0.0, 0.06, -0.55)), material=dark_metal, name="trailer_a_frame_1")
    trailer_frame.visual(Box((0.18, 0.58, 0.11)), origin=Origin(xyz=(-0.26, 0.0, 0.08)), material=dark_metal, name="trailer_front_hitch_bolster")
    trailer_frame.visual(Box((0.08, 0.96, 0.18)), origin=Origin(xyz=(-0.16, 0.0, 0.12)), material=dark_metal, name="trailer_front_mount_plate")
    trailer_frame.visual(Box((1.70, 1.10, 0.10)), origin=Origin(xyz=(-1.08, 0.0, 0.17)), material=weathered_wood, name="rear_cargo_bed_floor")
    trailer_frame.visual(Box((1.74, 0.08, 0.52)), origin=Origin(xyz=(-1.08, -0.59, 0.48)), material=weathered_wood, name="rear_cargo_bed_side_0")
    trailer_frame.visual(Box((1.74, 0.08, 0.52)), origin=Origin(xyz=(-1.08, 0.59, 0.48)), material=weathered_wood, name="rear_cargo_bed_side_1")
    trailer_frame.visual(Box((0.08, 1.10, 0.50)), origin=Origin(xyz=(-0.18, 0.0, 0.47)), material=weathered_wood, name="rear_cargo_bed_front_panel")
    trailer_frame.visual(Box((0.08, 1.10, 0.50)), origin=Origin(xyz=(-1.98, 0.0, 0.47)), material=weathered_wood, name="rear_cargo_bed_tailgate")
    for rail_i, z in enumerate((1.02, 1.22)):
        trailer_frame.visual(Box((1.66, 0.035, 0.035)), origin=Origin(xyz=(-1.08, -0.66, z - 0.66)), material=dark_metal, name=f"trailer_side_rail_0_{rail_i}")
        trailer_frame.visual(Box((1.66, 0.035, 0.035)), origin=Origin(xyz=(-1.08, 0.66, z - 0.66)), material=dark_metal, name=f"trailer_side_rail_1_{rail_i}")
    for side, y in enumerate((-0.40, 0.40)):
        trailer_frame.visual(Box((1.82, 0.07, 0.10)), origin=Origin(xyz=(-1.08, y, 0.07)), material=dark_metal, name=f"trailer_underframe_rail_{side}")
        trailer_frame.visual(Box((0.62, 0.08, 0.055)), origin=Origin(xyz=(-1.28, y, -0.12)), material=dark_metal, name=f"trailer_leaf_spring_{side}")
        trailer_frame.visual(Box((0.07, 0.06, 0.34)), origin=Origin(xyz=(-0.98, y, -0.03)), material=dark_metal, name=f"trailer_spring_hanger_front_{side}")
        trailer_frame.visual(Box((0.07, 0.06, 0.34)), origin=Origin(xyz=(-1.58, y, -0.03)), material=dark_metal, name=f"trailer_spring_hanger_rear_{side}")
        trailer_frame.visual(Box((0.16, 0.11, 0.09)), origin=Origin(xyz=(-1.28, y, -0.20)), material=dark_metal, name=f"trailer_axle_saddle_{side}")
    for x_i, x in enumerate((-1.98, -2.92, -3.40)):
        trailer_frame.visual(Box((0.08, 1.04, 0.10)), origin=Origin(xyz=(x + 1.64, 0.0, 0.07)), material=dark_metal, name=f"trailer_crossmember_{x_i}")
    trailer_frame.visual(Cylinder(radius=0.045, length=1.38), origin=Origin(xyz=(-1.28, 0.0, -0.26), rpy=(-pi / 2.0, 0.0, 0.0)), material=dark_metal, name="trailer_axle")

    steering_axle = model.part("steering_axle")
    steering_axle.visual(Box((0.20, 1.48, 0.13)), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=dark_blue, name="front_axle_beam")
    steering_axle.visual(Cylinder(radius=0.095, length=0.16), origin=Origin(xyz=(0.0, 0.0, 0.09)), material=dark_metal, name="steering_pivot_pin")
    steering_axle.visual(Cylinder(radius=0.030, length=1.34), origin=Origin(xyz=(0.10, 0.0, 0.055), rpy=(-pi / 2.0, 0.0, 0.0)), material=dark_metal, name="tie_rod")
    steering_axle.visual(Box((0.36, 0.42, 0.10)), origin=Origin(xyz=(0.02, -0.74, 0.53)), material=painted_blue, name="front_fender_0")
    steering_axle.visual(Box((0.045, 0.050, 0.55)), origin=Origin(xyz=(0.00, -0.55, 0.28)), material=dark_metal, name="front_fender_strut_0")
    steering_axle.visual(Cylinder(radius=0.060, length=0.13), origin=Origin(xyz=(0.0, -0.74, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)), material=dark_metal, name="kingpin_knuckle_0")
    steering_axle.visual(Box((0.36, 0.42, 0.10)), origin=Origin(xyz=(0.02, 0.74, 0.53)), material=painted_blue, name="front_fender_1")
    steering_axle.visual(Box((0.045, 0.050, 0.55)), origin=Origin(xyz=(0.00, 0.55, 0.28)), material=dark_metal, name="front_fender_strut_1")
    steering_axle.visual(Cylinder(radius=0.060, length=0.13), origin=Origin(xyz=(0.0, 0.74, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)), material=dark_metal, name="kingpin_knuckle_1")

    steering_joint = model.articulation(
        "chassis_to_steering_axle",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=steering_axle,
        origin=Origin(xyz=(1.12, 0.0, 0.40)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.7, lower=-0.45, upper=0.45),
    )
    trailer_yaw_joint = model.articulation(
        "chassis_to_trailer_frame",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=trailer_frame,
        origin=Origin(xyz=(-1.64, 0.0, 0.66)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=450.0, velocity=0.45, lower=-0.24, upper=0.24),
    )

    def add_wheel(part_name: str, radius: float, width: float, inner_radius: float, tire_style: str) -> object:
        wheel = model.part(part_name)
        tire = TireGeometry(
            radius,
            width,
            inner_radius=inner_radius,
            carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.07),
            tread=TireTread(style=tire_style, depth=0.032 if tire_style == "chevron" else 0.018, count=22 if tire_style == "chevron" else 26, angle_deg=28.0, land_ratio=0.55),
            grooves=(TireGroove(center_offset=0.0, width=0.030 if tire_style == "chevron" else 0.018, depth=0.010),),
            sidewall=TireSidewall(style="rounded", bulge=0.06),
            shoulder=TireShoulder(width=0.025 if radius > 0.5 else 0.016, radius=0.006),
        )
        rim = WheelGeometry(
            inner_radius + 0.025,
            width * 0.78,
            rim=WheelRim(inner_radius=inner_radius * 0.64, flange_height=0.020, flange_thickness=0.010, bead_seat_depth=0.010),
            hub=WheelHub(
                radius=inner_radius * 0.34,
                width=width * 0.60,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=6, circle_diameter=inner_radius * 0.42, hole_diameter=0.018 if radius > 0.5 else 0.012),
            ),
            face=WheelFace(dish_depth=0.026 if radius > 0.5 else 0.016, front_inset=0.012, rear_inset=0.008),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.020 if radius > 0.5 else 0.012, window_radius=0.050 if radius > 0.5 else 0.032),
            bore=WheelBore(style="round", diameter=0.070 if radius > 0.5 else 0.045),
        )
        wheel.visual(mesh_from_geometry(tire, f"{part_name}_tire_mesh"), material=black, name="tire")
        wheel.visual(mesh_from_geometry(rim, f"{part_name}_rim_mesh"), material=painted_blue, name="rim")
        return wheel

    rear_wheel_0 = add_wheel("rear_wheel_0", 0.62, 0.36, 0.36, "chevron")
    rear_wheel_1 = add_wheel("rear_wheel_1", 0.62, 0.36, 0.36, "chevron")
    front_wheel_0 = add_wheel("front_wheel_0", 0.40, 0.26, 0.24, "ribbed")
    front_wheel_1 = add_wheel("front_wheel_1", 0.40, 0.26, 0.24, "ribbed")
    trailer_wheel_0 = add_wheel("trailer_wheel_0", 0.40, 0.26, 0.24, "ribbed")
    trailer_wheel_1 = add_wheel("trailer_wheel_1", 0.40, 0.26, 0.24, "ribbed")

    rear_wheel_mark_0 = model.part("rear_wheel_mark_0")
    rear_wheel_mark_0.visual(Box((0.26, 0.09, 0.16)), origin=Origin(xyz=(0.0, 0.0, 0.53)), material=soil_wear, name="mud_patch")
    model.articulation(
        "rear_wheel_to_mark_0",
        ArticulationType.FIXED,
        parent=rear_wheel_0,
        child=rear_wheel_mark_0,
    )
    rear_wheel_mark_1 = model.part("rear_wheel_mark_1")
    rear_wheel_mark_1.visual(Box((0.26, 0.09, 0.16)), origin=Origin(xyz=(0.0, 0.0, 0.53)), material=soil_wear, name="mud_patch")
    model.articulation(
        "rear_wheel_to_mark_1",
        ArticulationType.FIXED,
        parent=rear_wheel_1,
        child=rear_wheel_mark_1,
    )

    rear_spin_0 = model.articulation(
        "rear_wheel_spin_0",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=rear_wheel_0,
        origin=Origin(xyz=(-0.65, -0.84, 0.65), rpy=(0.0, 0.0, pi / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=12.0),
    )
    model.articulation(
        "rear_wheel_spin_1",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=rear_wheel_1,
        origin=Origin(xyz=(-0.65, 0.84, 0.65), rpy=(0.0, 0.0, pi / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=12.0),
        mimic=Mimic(joint=rear_spin_0.name, multiplier=1.0),
    )
    model.articulation(
        "front_wheel_spin_0",
        ArticulationType.CONTINUOUS,
        parent=steering_axle,
        child=front_wheel_0,
        origin=Origin(xyz=(0.0, -0.78, 0.0), rpy=(0.0, 0.0, pi / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=500.0, velocity=14.0),
        mimic=Mimic(joint=rear_spin_0.name, multiplier=1.55),
    )
    model.articulation(
        "front_wheel_spin_1",
        ArticulationType.CONTINUOUS,
        parent=steering_axle,
        child=front_wheel_1,
        origin=Origin(xyz=(0.0, 0.78, 0.0), rpy=(0.0, 0.0, pi / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=500.0, velocity=14.0),
        mimic=Mimic(joint=rear_spin_0.name, multiplier=1.55),
    )
    model.articulation(
        "trailer_wheel_spin_0",
        ArticulationType.CONTINUOUS,
        parent=trailer_frame,
        child=trailer_wheel_0,
        origin=Origin(xyz=(-1.28, -0.76, -0.26), rpy=(0.0, 0.0, pi / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=450.0, velocity=14.0),
        mimic=Mimic(joint=rear_spin_0.name, multiplier=1.55),
    )
    model.articulation(
        "trailer_wheel_spin_1",
        ArticulationType.CONTINUOUS,
        parent=trailer_frame,
        child=trailer_wheel_1,
        origin=Origin(xyz=(-1.28, 0.76, -0.26), rpy=(0.0, 0.0, pi / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=450.0, velocity=14.0),
        mimic=Mimic(joint=rear_spin_0.name, multiplier=1.55),
    )

    # Retain the explicitly named object class for downstream consumers.
    model.meta["small_class"] = "Tractor"
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    steering_axle = object_model.get_part("steering_axle")
    trailer_frame = object_model.get_part("trailer_frame")
    rear_wheel = object_model.get_part("rear_wheel_0")
    front_wheel = object_model.get_part("front_wheel_0")
    trailer_wheel = object_model.get_part("trailer_wheel_0")
    rear_mark = object_model.get_part("rear_wheel_mark_0")
    steering_joint = object_model.get_articulation("chassis_to_steering_axle")
    trailer_yaw_joint = object_model.get_articulation("chassis_to_trailer_frame")
    rear_spin = object_model.get_articulation("rear_wheel_spin_0")

    ctx.check(
        "asset is explicitly a Tractor",
        object_model.name == "agricultural_tractor" and object_model.meta.get("small_class") == "Tractor",
        details=f"name={object_model.name}, meta={object_model.meta}",
    )
    for visual_name in (
        "hood",
        "cab_roof",
        "radiator_grille",
        "rear_axle_housing",
        "drawbar",
        "exhaust_stack",
        "steering_column",
        "front_headlamp_0",
        "drawbar_clevis_top_plate",
    ):
        ctx.check(f"tractor has {visual_name}", chassis.get_visual(visual_name) is not None)
    for visual_name in (
        "trailer_hitch_coupler",
        "trailer_a_frame_0",
        "trailer_front_hitch_bolster",
        "trailer_front_mount_plate",
        "rear_cargo_bed_floor",
        "rear_cargo_bed_tailgate",
        "trailer_axle",
        "trailer_underframe_rail_0",
        "trailer_leaf_spring_0",
        "trailer_axle_saddle_0",
    ):
        ctx.check(f"trailer frame has {visual_name}", trailer_frame.get_visual(visual_name) is not None)
    ctx.check("trailer wheel reuses wheel tire geometry", trailer_wheel.get_visual("tire") is not None and trailer_wheel.get_visual("rim") is not None)

    ctx.check(
        "primary mechanisms include steering, trailer yaw, and wheel rotation",
        steering_joint.articulation_type == ArticulationType.REVOLUTE
        and trailer_yaw_joint.articulation_type == ArticulationType.REVOLUTE
        and rear_spin.articulation_type == ArticulationType.CONTINUOUS,
    )
    ctx.check(
        "trailer yaw joint is limited to a small z-axis swing",
        trailer_yaw_joint.axis == (0.0, 0.0, 1.0)
        and trailer_yaw_joint.motion_limits is not None
        and trailer_yaw_joint.motion_limits.lower == -0.24
        and trailer_yaw_joint.motion_limits.upper == 0.24,
        details=f"axis={trailer_yaw_joint.axis}, limits={trailer_yaw_joint.motion_limits}",
    )
    ctx.allow_overlap(
        chassis,
        steering_axle,
        elem_a="front_bolster",
        elem_b="steering_pivot_pin",
        reason="The steerable axle's vertical pivot pin is intentionally captured inside the tractor front bolster.",
    )
    ctx.allow_overlap(
        steering_axle,
        front_wheel,
        elem_a="kingpin_knuckle_0",
        elem_b="rim",
        reason="The front wheel hub/rim is intentionally modeled around the knuckle spindle so the wheel is visibly mounted.",
    )
    ctx.allow_overlap(
        steering_axle,
        "front_wheel_1",
        elem_a="kingpin_knuckle_1",
        elem_b="rim",
        reason="The front wheel hub/rim is intentionally modeled around the knuckle spindle so the wheel is visibly mounted.",
    )
    ctx.allow_overlap(
        steering_axle,
        front_wheel,
        elem_a="front_axle_beam",
        elem_b="rim",
        reason="The front axle beam end is intentionally represented as a robust spindle inserted into the wheel hub.",
    )
    ctx.allow_overlap(
        steering_axle,
        "front_wheel_1",
        elem_a="front_axle_beam",
        elem_b="rim",
        reason="The front axle beam end is intentionally represented as a robust spindle inserted into the wheel hub.",
    )
    ctx.allow_overlap(
        chassis,
        rear_wheel,
        elem_a="rear_axle_stub_0",
        elem_b="rim",
        reason="The rear drive axle stub is intentionally seated inside the rotating wheel hub.",
    )
    ctx.allow_overlap(
        chassis,
        "rear_wheel_1",
        elem_a="rear_axle_stub_1",
        elem_b="rim",
        reason="The rear drive axle stub is intentionally seated inside the rotating wheel hub.",
    )
    ctx.allow_overlap(
        rear_mark,
        rear_wheel,
        elem_a="mud_patch",
        elem_b="tire",
        reason="A small soil/wear patch is intentionally embedded into the tire surface so wheel rotation is visibly readable.",
    )
    ctx.allow_overlap(
        "rear_wheel_mark_1",
        "rear_wheel_1",
        elem_a="mud_patch",
        elem_b="tire",
        reason="A small soil/wear patch is intentionally embedded into the tire surface so wheel rotation is visibly readable.",
    )
    ctx.allow_overlap(
        trailer_frame,
        trailer_wheel,
        elem_a="trailer_axle",
        elem_b="rim",
        reason="The trailer axle intentionally passes through the front-wheel-style trailer wheel hub.",
    )
    ctx.allow_overlap(
        trailer_frame,
        "trailer_wheel_1",
        elem_a="trailer_axle",
        elem_b="rim",
        reason="The trailer axle intentionally passes through the front-wheel-style trailer wheel hub.",
    )
    ctx.expect_overlap(
        chassis,
        steering_axle,
        axes="xy",
        min_overlap=0.08,
        elem_a="front_bolster",
        elem_b="steering_pivot_pin",
        name="steering pivot pin is captured in the front bolster",
    )
    ctx.expect_overlap(
        steering_axle,
        front_wheel,
        axes="xy",
        min_overlap=0.08,
        elem_a="kingpin_knuckle_0",
        elem_b="rim",
        name="front wheel hub remains on knuckle spindle",
    )
    ctx.expect_overlap(
        steering_axle,
        front_wheel,
        axes="y",
        min_overlap=0.05,
        elem_a="front_axle_beam",
        elem_b="rim",
        name="front axle beam enters the hub as a visible spindle",
    )
    ctx.expect_overlap(
        chassis,
        rear_wheel,
        axes="xy",
        min_overlap=0.06,
        elem_a="rear_axle_stub_0",
        elem_b="rim",
        name="rear wheel hub remains on rear axle stub",
    )
    ctx.expect_overlap(
        rear_mark,
        rear_wheel,
        axes="xz",
        min_overlap=0.05,
        elem_a="mud_patch",
        elem_b="tire",
        name="soil patch is seated in the rear tire surface",
    )
    ctx.expect_gap(
        chassis,
        rear_wheel,
        axis="z",
        min_gap=0.015,
        max_gap=0.18,
        positive_elem="rear_fender_top_0",
        negative_elem="tire",
        name="rear fender clears the large rear tire",
    )
    ctx.expect_gap(
        steering_axle,
        front_wheel,
        axis="z",
        min_gap=0.030,
        max_gap=0.18,
        positive_elem="front_fender_0",
        negative_elem="tire",
        name="front fender clears steerable front tire",
    )
    ctx.expect_overlap(
        chassis,
        rear_wheel,
        axes="xy",
        min_overlap=0.20,
        elem_a="rear_fender_top_0",
        elem_b="tire",
        name="rear fender is actually over the rear tire",
    )
    ctx.expect_overlap(
        chassis,
        front_wheel,
        axes="x",
        min_overlap=0.10,
        elem_a="front_bolster",
        elem_b="tire",
        name="front wheel pair sits under tractor nose/bolster",
    )
    ctx.expect_overlap(
        trailer_frame,
        trailer_wheel,
        axes="xy",
        min_overlap=0.06,
        elem_a="trailer_axle",
        elem_b="rim",
        name="trailer axle seats inside front-wheel-style trailer hub",
    )

    rest_marker_aabb = ctx.part_element_world_aabb(rear_mark, elem="mud_patch")
    with ctx.pose({rear_spin: 1.0}):
        rotated_marker_aabb = ctx.part_element_world_aabb(rear_mark, elem="mud_patch")
    ctx.check(
        "rear wheel rotation visibly carries valve marker around tire",
        rest_marker_aabb is not None
        and rotated_marker_aabb is not None
        and abs(((rest_marker_aabb[0][2] + rest_marker_aabb[1][2]) * 0.5) - ((rotated_marker_aabb[0][2] + rotated_marker_aabb[1][2]) * 0.5)) > 0.08,
        details=f"rest={rest_marker_aabb}, rotated={rotated_marker_aabb}",
    )

    rest_front = ctx.part_world_position(front_wheel)
    with ctx.pose({steering_joint: 0.35}):
        steered_front = ctx.part_world_position(front_wheel)
    ctx.check(
        "front axle steering yaws the front wheel centers",
        rest_front is not None
        and steered_front is not None
        and abs(steered_front[0] - rest_front[0]) > 0.04,
        details=f"rest={rest_front}, steered={steered_front}",
    )

    rest_trailer_frame = ctx.part_world_position(trailer_frame)
    rest_trailer_wheel_0 = ctx.part_world_position(trailer_wheel)
    rest_trailer_wheel_1 = ctx.part_world_position("trailer_wheel_1")
    rest_tailgate = ctx.part_element_world_aabb(trailer_frame, elem="rear_cargo_bed_tailgate")
    with ctx.pose({trailer_yaw_joint: 0.20}):
        yawed_trailer_frame = ctx.part_world_position(trailer_frame)
        yawed_trailer_wheel_0 = ctx.part_world_position(trailer_wheel)
        yawed_trailer_wheel_1 = ctx.part_world_position("trailer_wheel_1")
        yawed_tailgate = ctx.part_element_world_aabb(trailer_frame, elem="rear_cargo_bed_tailgate")
    rest_tailgate_y = ((rest_tailgate[0][1] + rest_tailgate[1][1]) * 0.5) if rest_tailgate is not None else None
    yawed_tailgate_y = ((yawed_tailgate[0][1] + yawed_tailgate[1][1]) * 0.5) if yawed_tailgate is not None else None
    ctx.check(
        "trailer bed and trailer wheels yaw together around the hitch pin",
        rest_trailer_frame is not None
        and yawed_trailer_frame is not None
        and abs(yawed_trailer_frame[0] - rest_trailer_frame[0]) < 0.002
        and abs(yawed_trailer_frame[1] - rest_trailer_frame[1]) < 0.002
        and rest_trailer_wheel_0 is not None
        and yawed_trailer_wheel_0 is not None
        and abs(yawed_trailer_wheel_0[1] - rest_trailer_wheel_0[1]) > 0.12
        and rest_trailer_wheel_1 is not None
        and yawed_trailer_wheel_1 is not None
        and abs(yawed_trailer_wheel_1[1] - rest_trailer_wheel_1[1]) > 0.12
        and rest_tailgate_y is not None
        and yawed_tailgate_y is not None
        and abs(yawed_tailgate_y - rest_tailgate_y) > 0.30,
        details=f"frame={rest_trailer_frame}->{yawed_trailer_frame}, wheel0={rest_trailer_wheel_0}->{yawed_trailer_wheel_0}, wheel1={rest_trailer_wheel_1}->{yawed_trailer_wheel_1}, tailgate_y={rest_tailgate_y}->{yawed_tailgate_y}",
    )

    wheel_names = (
        "rear_wheel_0",
        "rear_wheel_1",
        "front_wheel_0",
        "front_wheel_1",
        "trailer_wheel_0",
        "trailer_wheel_1",
    )
    tire_aabbs = {
        name: ctx.part_element_world_aabb(object_model.get_part(name), elem="tire") for name in wheel_names
    }
    tire_bottoms = {
        name: aabb[0][2] for name, aabb in tire_aabbs.items() if aabb is not None
    }
    ctx.check(
        "all six tires sit on the same ground plane",
        len(tire_bottoms) == 6 and max(abs(bottom) for bottom in tire_bottoms.values()) < 0.002,
        details=f"bottom_z={tire_bottoms}",
    )

    front_tire_aabb = tire_aabbs["front_wheel_0"]
    trailer_tire_aabb = tire_aabbs["trailer_wheel_0"]
    ctx.check(
        "trailer wheel matches front wheel diameter and ground contact height",
        front_tire_aabb is not None
        and trailer_tire_aabb is not None
        and abs((front_tire_aabb[1][2] - front_tire_aabb[0][2]) - (trailer_tire_aabb[1][2] - trailer_tire_aabb[0][2])) < 0.01
        and abs(front_tire_aabb[0][2] - trailer_tire_aabb[0][2]) < 0.01,
        details=f"front={front_tire_aabb}, trailer={trailer_tire_aabb}",
    )

    bed_floor_aabb = ctx.part_element_world_aabb(trailer_frame, elem="rear_cargo_bed_floor")
    support_aabbs = {
        name: ctx.part_element_world_aabb(trailer_frame, elem=name)
        for name in (
            "trailer_underframe_rail_0",
            "trailer_underframe_rail_1",
            "trailer_crossmember_0",
            "trailer_crossmember_1",
            "trailer_crossmember_2",
        )
    }
    support_tops = {
        name: aabb[1][2] for name, aabb in support_aabbs.items() if aabb is not None
    }
    floor_bottom = bed_floor_aabb[0][2] if bed_floor_aabb is not None else None
    ctx.check(
        "cargo bed floor rests on the trailer underframe instead of floating",
        floor_bottom is not None
        and len(support_tops) == 5
        and max(abs(floor_bottom - top) for top in support_tops.values()) < 0.003,
        details=f"floor_bottom={floor_bottom}, support_tops={support_tops}",
    )

    return ctx.report()


object_model = build_object_model()
