from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TireTread,
    TorusGeometry,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _arc_fender_geometry(
    *,
    inner_radius: float,
    thickness: float,
    width: float,
    start_deg: float,
    end_deg: float,
    segments: int = 36,
) -> MeshGeometry:
    """Curved sheet-metal fender over a wheel, local arc in XZ and width in Y."""

    geom = MeshGeometry()
    angles = [start_deg * pi / 180.0 + (end_deg - start_deg) * pi / 180.0 * i / segments for i in range(segments + 1)]
    # vertex order per angle: inner left, inner right, outer left, outer right
    for theta in angles:
        for radius in (inner_radius, inner_radius + thickness):
            for y in (-width / 2.0, width / 2.0):
                geom.add_vertex(radius * cos(theta), y, radius * sin(theta))

    def vid(i: int, radial: int, side: int) -> int:
        return i * 4 + radial * 2 + side

    for i in range(segments):
        # inner curved face
        geom.add_face(vid(i, 0, 0), vid(i + 1, 0, 0), vid(i + 1, 0, 1))
        geom.add_face(vid(i, 0, 0), vid(i + 1, 0, 1), vid(i, 0, 1))
        # outer curved face
        geom.add_face(vid(i, 1, 0), vid(i, 1, 1), vid(i + 1, 1, 1))
        geom.add_face(vid(i, 1, 0), vid(i + 1, 1, 1), vid(i + 1, 1, 0))
        # side flanges
        geom.add_face(vid(i, 0, 0), vid(i, 1, 0), vid(i + 1, 1, 0))
        geom.add_face(vid(i, 0, 0), vid(i + 1, 1, 0), vid(i + 1, 0, 0))
        geom.add_face(vid(i, 0, 1), vid(i + 1, 0, 1), vid(i + 1, 1, 1))
        geom.add_face(vid(i, 0, 1), vid(i + 1, 1, 1), vid(i, 1, 1))

    # end caps
    for i in (0, segments):
        geom.add_face(vid(i, 0, 0), vid(i, 0, 1), vid(i, 1, 1))
        geom.add_face(vid(i, 0, 0), vid(i, 1, 1), vid(i, 1, 0))
    return geom


def _add_cylinder_x(part, radius: float, length: float, xyz, material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _add_cylinder_y(part, radius: float, length: float, xyz, material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_cylinder_z(part, radius: float, length: float, xyz, material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _wheel_meshes(
    prefix: str,
    *,
    radius: float,
    width: float,
    rim_radius: float,
    tread_style: str,
    tread_depth: float,
    tread_count: int,
):
    tire = TireGeometry(
        radius,
        width,
        inner_radius=rim_radius * 1.02,
        carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.08),
        tread=TireTread(style=tread_style, depth=tread_depth, count=tread_count, angle_deg=28.0, land_ratio=0.56),
        sidewall=TireSidewall(style="rounded", bulge=0.055),
        shoulder=TireShoulder(width=0.030 if radius > 0.45 else 0.012, radius=0.010),
    )
    wheel = WheelGeometry(
        rim_radius,
        width * 0.72,
        rim=WheelRim(
            inner_radius=rim_radius * 0.70,
            flange_height=0.030 if radius > 0.45 else 0.015,
            flange_thickness=0.012,
            bead_seat_depth=0.010,
        ),
        hub=WheelHub(
            radius=rim_radius * 0.33,
            width=width * 0.54,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=6 if radius > 0.45 else 5, circle_diameter=rim_radius * 0.42, hole_diameter=0.018),
        ),
        face=WheelFace(dish_depth=0.018, front_inset=0.008, rear_inset=0.006),
        spokes=WheelSpokes(style="split_y" if radius > 0.45 else "straight", count=6 if radius > 0.45 else 5, thickness=0.014, window_radius=0.040),
        bore=WheelBore(style="round", diameter=rim_radius * 0.18),
    )
    return mesh_from_geometry(tire, f"{prefix}_tire"), mesh_from_geometry(wheel, f"{prefix}_rim")


def _add_wheel_visuals(part, prefix: str, *, radius: float, width: float, rim_radius: float, tire_mat: Material, rim_mat: Material, large: bool) -> None:
    tire_mesh, rim_mesh = _wheel_meshes(
        prefix,
        radius=radius,
        width=width,
        rim_radius=rim_radius,
        tread_style="chevron" if large else "ribbed",
        tread_depth=0.038 if large else 0.012,
        tread_count=22 if large else 18,
    )
    wheel_origin = Origin(rpy=(0.0, 0.0, pi / 2.0))
    part.visual(tire_mesh, origin=wheel_origin, material=tire_mat, name="tire")
    part.visual(rim_mesh, origin=wheel_origin, material=rim_mat, name="yellow_rim")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="agricultural_tractor_002",
        meta={
            "category": "Agricultural",
            "small_class": "Tractor",
            "reference": "picture/Agricultural/Tractor/002.png",
        },
    )

    green = model.material("john_deere_green", rgba=(0.02, 0.48, 0.13, 1.0))
    yellow = model.material("deere_yellow", rgba=(0.93, 0.86, 0.18, 1.0))
    cream = model.material("aged_cream_stripe", rgba=(0.93, 0.86, 0.52, 1.0))
    dark_green = model.material("dark_green_shadow", rgba=(0.01, 0.22, 0.08, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    black = model.material("black_hardware", rgba=(0.02, 0.022, 0.024, 1.0))
    steel = model.material("worn_steel", rgba=(0.45, 0.46, 0.42, 1.0))
    soot = model.material("sooty_exhaust", rgba=(0.20, 0.22, 0.21, 1.0))
    chrome = model.material("chrome_bezel", rgba=(0.78, 0.78, 0.72, 1.0))
    rust = model.material("light_rust_wear", rgba=(0.55, 0.28, 0.08, 1.0))
    glass = model.material("lamp_glass", rgba=(0.95, 0.95, 0.82, 1.0))

    chassis = model.part("chassis")

    # Ladder frame, driveline, engine block, and long hood.
    # Narrowed ladder frames to clear the closely spaced narrow-front wheels.
    chassis.visual(Box((2.45, 0.04, 0.16)), origin=Origin(xyz=(0.03, -0.20, 0.48)), material=dark_green, name="ladder_frame")
    chassis.visual(Box((2.45, 0.04, 0.16)), origin=Origin(xyz=(0.03, 0.20, 0.48)), material=dark_green, name="ladder_frame_1")
    chassis.visual(Box((0.78, 0.58, 0.42)), origin=Origin(xyz=(-0.66, 0.0, 0.70)), material=green, name="transmission_case")
    chassis.visual(Box((0.48, 0.46, 0.34)), origin=Origin(xyz=(-0.12, 0.0, 0.74)), material=green, name="engine_block")
    chassis.visual(Box((1.72, 0.58, 0.34)), origin=Origin(xyz=(0.56, 0.0, 1.02)), material=green, name="long_hood")
    chassis.visual(Box((1.60, 0.42, 0.105)), origin=Origin(xyz=(0.50, 0.0, 1.205)), material=green, name="hood_raised_spine")
    chassis.visual(Box((1.58, 0.050, 0.080)), origin=Origin(xyz=(0.48, -0.294, 1.08)), material=cream, name="side_stripe_0")
    chassis.visual(Box((1.58, 0.050, 0.080)), origin=Origin(xyz=(0.48, 0.294, 1.08)), material=cream, name="side_stripe_1")
    chassis.visual(Box((0.64, 0.050, 0.045)), origin=Origin(xyz=(0.63, -0.296, 0.94)), material=dark_green, name="side_panel_0")
    chassis.visual(Box((0.64, 0.050, 0.045)), origin=Origin(xyz=(0.63, 0.296, 0.94)), material=dark_green, name="side_panel_1")
    # Raised block lettering on the cream side stripe, standing proud but embedded
    # into the stripe so the letters are real supported geometry rather than a texture.
    for idx, x in enumerate((-0.06, 0.045, 0.15, 0.255, 0.41, 0.515, 0.62, 0.725, 0.83)):
        chassis.visual(Box((0.020, 0.020, 0.050)), origin=Origin(xyz=(x, -0.322, 1.083)), material=dark_green, name=f"john_deere_letter_{idx}")
    chassis.visual(Box((0.09, 0.020, 0.018)), origin=Origin(xyz=(0.31, -0.322, 1.083)), material=dark_green, name="john_deere_word_space")

    # Front grille face with individual vertical slats and side model badge.
    chassis.visual(Box((0.090, 0.60, 0.50)), origin=Origin(xyz=(1.43, 0.0, 0.98)), material=cream, name="front_grille_panel")
    for idx, y in enumerate((-0.235, -0.185, -0.135, -0.085, -0.035, 0.015, 0.065, 0.115, 0.165, 0.215)):
        chassis.visual(Box((0.030, 0.020, 0.370)), origin=Origin(xyz=(1.485, y, 0.98)), material=dark_green, name=f"grille_slat_{idx}")
    chassis.visual(Box((0.055, 0.050, 0.090)), origin=Origin(xyz=(1.485, -0.298, 0.78)), material=cream, name="number_badge_520")
    for idx, z in enumerate((0.805, 0.780, 0.755)):
        chassis.visual(Box((0.018, 0.020, 0.010)), origin=Origin(xyz=(1.516, -0.323, z)), material=dark_green, name=f"badge_520_mark_{idx}")

    # Exposed side engine pipes and controls, connected into the engine block.
    _add_cylinder_x(chassis, 0.032, 1.00, (0.38, -0.285, 0.78), green, "upper_side_pipe")
    _add_cylinder_x(chassis, 0.026, 0.88, (0.42, -0.285, 0.66), green, "lower_side_pipe")
    chassis.visual(Box((0.11, 0.14, 0.20)), origin=Origin(xyz=(0.05, -0.275, 0.70)), material=dark_green, name="carburetor_box")
    _add_cylinder_y(chassis, 0.035, 0.22, (0.08, -0.380, 0.86), steel, "air_intake_can")
    _add_cylinder_x(chassis, 0.018, 0.58, (-0.02, -0.275, 0.58), steel, "linkage_rod")
    chassis.visual(Box((0.08, 0.090, 0.08)), origin=Origin(xyz=(-0.02, -0.255, 0.78)), material=green, name="manifold_mount_0")
    chassis.visual(Box((0.08, 0.090, 0.08)), origin=Origin(xyz=(0.47, -0.255, 0.76)), material=green, name="manifold_mount_1")
    chassis.visual(Box((0.55, 0.35, 0.055)), origin=Origin(xyz=(-0.52, -0.18, 0.43)), material=steel, name="perforated_step")

    # Axle housings and front pedestal.
    chassis.visual(
        Cylinder(radius=0.105, length=1.20),
        origin=Origin(xyz=(-1.02, 0.0, 0.58), rpy=(pi / 2.0, 0.0, 0.0)),
        material=green,
        name="rear_axle_housing",
    )
    # Narrow row-crop pivot bolster casting. Slimmed in Y to clear the closely
    # spaced front wheels while still capturing the pivot pin.
    chassis.visual(Box((0.18, 0.04, 0.20)), origin=Origin(xyz=(1.28, 0.0, 0.42)), material=green, name="front_bolster")
    chassis.visual(Box((0.12, 0.04, 0.52)), origin=Origin(xyz=(1.30, 0.0, 0.63)), material=green, name="front_pedestal")

    # Open station: seat, platform, steering column, and a separately rotating wheel.
    chassis.visual(Box((0.62, 0.70, 0.090)), origin=Origin(xyz=(-0.82, 0.0, 0.935)), material=green, name="operator_platform")
    chassis.visual(Box((0.56, 0.52, 0.10)), origin=Origin(xyz=(-0.91, 0.0, 1.015)), material=yellow, name="seat_cushion")
    chassis.visual(Box((0.10, 0.56, 0.36)), origin=Origin(xyz=(-1.18, 0.0, 1.20)), material=yellow, name="seat_back")
    chassis.visual(Box((0.24, 0.54, 0.34)), origin=Origin(xyz=(-0.33, 0.0, 1.03)), material=green, name="dash_cowl")
    chassis.visual(
        Cylinder(radius=0.024, length=0.42),
        origin=Origin(xyz=(-0.31, 0.0, 1.27), rpy=(0.0, -0.55, 0.0)),
        material=green,
        name="steering_column",
    )
    chassis.visual(Box((0.17, 0.16, 0.12)), origin=Origin(xyz=(-0.33, 0.0, 1.11)), material=green, name="steering_column_base")

    steering_wheel = model.part("steering_wheel")
    steering_wheel.visual(
        mesh_from_geometry(TorusGeometry(radius=0.195, tube=0.010, radial_segments=16, tubular_segments=72), "steering_wheel_ring"),
        origin=Origin(),
        material=black,
        name="steering_wheel_ring",
    )
    # Full-length spokes that physically bridge the hub to the rim ring so the
    # ring is connected geometry rather than a floating island.
    for idx, yaw in enumerate((0.0, 2.09, 4.18)):
        steering_wheel.visual(
            Box((0.25, 0.012, 0.012)),
            origin=Origin(xyz=(0.075 * cos(yaw), 0.075 * sin(yaw), 0.0), rpy=(0.0, 0.0, yaw)),
            material=black,
            name=f"steering_spoke_{idx}",
        )
    _add_cylinder_z(steering_wheel, 0.038, 0.050, (0.0, 0.0, 0.0), black, "steering_hub")
    _add_cylinder_z(steering_wheel, 0.020, 0.020, (0.0, 0.0, 0.025), steel, "hub_retaining_nut")

    # Rolled fenders with bracket blocks tied to the platform.
    fender_mesh = mesh_from_geometry(
        _arc_fender_geometry(inner_radius=0.640, thickness=0.055, width=0.42, start_deg=18.0, end_deg=162.0),
        "rear_curved_fender",
    )
    chassis.visual(fender_mesh, origin=Origin(xyz=(-1.02, -0.72, 0.58)), material=green, name="rear_fender_0")
    chassis.visual(fender_mesh, origin=Origin(xyz=(-1.02, 0.72, 0.58)), material=green, name="rear_fender_1")
    chassis.visual(Box((0.18, 0.24, 0.42)), origin=Origin(xyz=(-0.88, -0.40, 0.92)), material=green, name="fender_bracket_0")
    chassis.visual(Box((0.18, 0.24, 0.42)), origin=Origin(xyz=(-0.88, 0.40, 0.92)), material=green, name="fender_bracket_1")
    chassis.visual(Box((0.13, 0.08, 0.36)), origin=Origin(xyz=(-1.02, -0.49, 1.06)), material=green, name="fender_stay_0")
    chassis.visual(Box((0.13, 0.08, 0.36)), origin=Origin(xyz=(-1.02, 0.49, 1.06)), material=green, name="fender_stay_1")
    chassis.visual(Box((0.09, 0.05, 0.055)), origin=Origin(xyz=(-1.02, -0.50, 1.20)), material=black, name="fender_bolt_0")
    chassis.visual(Box((0.09, 0.05, 0.055)), origin=Origin(xyz=(-1.02, 0.50, 1.20)), material=black, name="fender_bolt_1")

    # Exhaust stack, headlamps, drawbar mount, and seams.
    chassis.visual(Cylinder(radius=0.040, length=0.78), origin=Origin(xyz=(0.92, -0.18, 1.55)), material=soot, name="vertical_exhaust")
    chassis.visual(Cylinder(radius=0.052, length=0.25), origin=Origin(xyz=(0.92, -0.18, 1.92)), material=steel, name="exhaust_muffler")
    chassis.visual(Cylinder(radius=0.050, length=0.060), origin=Origin(xyz=(0.92, -0.18, 1.205)), material=chrome, name="exhaust_base_collar")
    chassis.visual(Cylinder(radius=0.045, length=0.18), origin=Origin(xyz=(0.92, -0.18, 2.115)), material=steel, name="exhaust_top_step")
    chassis.visual(Box((0.74, 0.020, 0.022)), origin=Origin(xyz=(0.28, -0.280, 1.185)), material=cream, name="hood_seam_0")
    chassis.visual(Box((0.74, 0.020, 0.022)), origin=Origin(xyz=(0.28, 0.280, 1.185)), material=cream, name="hood_seam_1")
    chassis.visual(Box((0.32, 0.012, 0.025)), origin=Origin(xyz=(0.84, -0.323, 1.105)), material=rust, name="paint_wear_stripe")
    _add_cylinder_x(chassis, 0.060, 0.040, (0.02, -0.36, 0.94), glass, "headlight_0")
    _add_cylinder_x(chassis, 0.060, 0.040, (0.02, 0.36, 0.94), glass, "headlight_1")
    _add_cylinder_x(chassis, 0.070, 0.025, (0.02, -0.345, 0.94), chrome, "headlight_bezel_0")
    _add_cylinder_x(chassis, 0.070, 0.025, (0.02, 0.345, 0.94), chrome, "headlight_bezel_1")
    chassis.visual(Box((0.08, 0.075, 0.12)), origin=Origin(xyz=(0.04, -0.295, 0.90)), material=green, name="lamp_bracket_0")
    chassis.visual(Box((0.08, 0.075, 0.12)), origin=Origin(xyz=(0.04, 0.295, 0.90)), material=green, name="lamp_bracket_1")
    chassis.visual(Box((0.50, 0.46, 0.13)), origin=Origin(xyz=(-1.25, 0.0, 0.47)), material=dark_green, name="rear_crossmember")
    chassis.visual(Box((0.18, 0.40, 0.14)), origin=Origin(xyz=(-1.45, 0.0, 0.45)), material=dark_green, name="rear_hitch_mount")

    # Front steering axle adopted from the sibling Tractor (001.png) logic: a single
    # solid beam pivots as ONE piece about a central vertical pin captured in the
    # front bolster, and both front wheels spin directly off the beam ends. There
    # are no separate steering knuckles. Re-skinned in this tractor's green/steel
    # palette so the visual style is preserved.
    front_axle = model.part("front_axle")
    # Narrow row-crop tricycle front: beam shortened and spindles pulled inward
    # to y~=+/-0.10 so the two front wheels sit close together at the centerline.
    front_axle.visual(Box((0.16, 0.26, 0.11)), origin=Origin(xyz=(0.0, 0.0, -0.10)), material=green, name="front_axle_beam")
    # Shortened pivot pin to clear the closely spaced front tires
    front_axle.visual(Cylinder(radius=0.060, length=0.12), origin=Origin(xyz=(0.0, 0.0, -0.04)), material=steel, name="center_pivot_pin")
    # Shortened tie rod for narrow front track
    _add_cylinder_y(front_axle, 0.018, 0.14, (0.07, 0.0, -0.165), steel, "tie_rod")
    _add_cylinder_y(front_axle, 0.035, 0.16, (0.0, -0.10, -0.10), steel, "spindle_0")
    _add_cylinder_y(front_axle, 0.035, 0.16, (0.0, 0.10, -0.10), steel, "spindle_1")
    front_axle.visual(Box((0.10, 0.10, 0.18)), origin=Origin(xyz=(0.0, -0.10, -0.06)), material=dark_green, name="knuckle_boss_0")
    front_axle.visual(Box((0.10, 0.10, 0.18)), origin=Origin(xyz=(0.0, 0.10, -0.06)), material=dark_green, name="knuckle_boss_1")

    # Four wheels as rotating parts.
    rear_wheel_0 = model.part("rear_wheel_0")
    _add_wheel_visuals(rear_wheel_0, "rear_wheel_0", radius=0.58, width=0.34, rim_radius=0.33, tire_mat=rubber, rim_mat=yellow, large=True)
    rear_wheel_1 = model.part("rear_wheel_1")
    _add_wheel_visuals(rear_wheel_1, "rear_wheel_1", radius=0.58, width=0.34, rim_radius=0.33, tire_mat=rubber, rim_mat=yellow, large=True)
    front_wheel_0 = model.part("front_wheel_0")
    _add_wheel_visuals(front_wheel_0, "front_wheel_0", radius=0.30, width=0.16, rim_radius=0.20, tire_mat=rubber, rim_mat=yellow, large=False)
    front_wheel_1 = model.part("front_wheel_1")
    _add_wheel_visuals(front_wheel_1, "front_wheel_1", radius=0.30, width=0.16, rim_radius=0.20, tire_mat=rubber, rim_mat=yellow, large=False)

    # Rear hitch lift/drawbar linkage.
    hitch = model.part("hitch")
    hitch.visual(Box((0.70, 0.10, 0.055)), origin=Origin(xyz=(-0.35, 0.0, 0.0)), material=steel, name="drawbar")
    hitch.visual(Box((0.44, 0.055, 0.055)), origin=Origin(xyz=(-0.23, -0.18, 0.09), rpy=(0.0, 0.22, 0.0)), material=steel, name="lift_arm_0")
    hitch.visual(Box((0.44, 0.055, 0.055)), origin=Origin(xyz=(-0.23, 0.18, 0.09), rpy=(0.0, 0.22, 0.0)), material=steel, name="lift_arm_1")
    hitch.visual(
        Cylinder(radius=0.030, length=0.46),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hitch_pivot_pin",
    )
    hitch.visual(Box((0.10, 0.30, 0.08)), origin=Origin(xyz=(-0.70, 0.0, 0.0)), material=black, name="hitch_clevis")

    model.articulation(
        "chassis_to_front_axle",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=front_axle,
        origin=Origin(xyz=(1.28, 0.0, 0.40)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=-0.45, upper=0.45),
        mimic=Mimic(joint="steering_wheel_turn", multiplier=0.36),
    )
    model.articulation(
        "steering_wheel_turn",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=steering_wheel,
        origin=Origin(xyz=(-0.42, 0.0, 1.45), rpy=(0.0, -0.55, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=4.0, lower=-1.25, upper=1.25),
    )
    model.articulation(
        "rear_wheel_0_spin",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=rear_wheel_0,
        origin=Origin(xyz=(-1.02, -0.70, 0.58)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=300.0, velocity=12.0),
    )
    model.articulation(
        "rear_wheel_1_spin",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=rear_wheel_1,
        origin=Origin(xyz=(-1.02, 0.70, 0.58)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=300.0, velocity=12.0),
    )
    model.articulation(
        "front_wheel_0_spin",
        ArticulationType.CONTINUOUS,
        parent=front_axle,
        child=front_wheel_0,
        origin=Origin(xyz=(0.0, -0.10, -0.10)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=18.0),
    )
    model.articulation(
        "front_wheel_1_spin",
        ArticulationType.CONTINUOUS,
        parent=front_axle,
        child=front_wheel_1,
        origin=Origin(xyz=(0.0, 0.10, -0.10)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=18.0),
    )
    model.articulation(
        "chassis_to_hitch",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=hitch,
        origin=Origin(xyz=(-1.54, 0.0, 0.42)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.7, lower=-0.30, upper=0.42),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    ctx.check(
        "small class is Tractor",
        object_model.meta.get("category") == "Agricultural" and object_model.meta.get("small_class") == "Tractor",
        details=f"meta={object_model.meta}",
    )

    required_parts = {
        "chassis",
        "front_axle",
        "steering_wheel",
        "rear_wheel_0",
        "rear_wheel_1",
        "front_wheel_0",
        "front_wheel_1",
        "hitch",
    }
    ctx.check("tractor subassemblies are modeled", required_parts.issubset({part.name for part in object_model.parts}))

    non_fixed = [joint for joint in object_model.articulations if str(joint.articulation_type).split(".")[-1] != "FIXED"]
    ctx.check("visible mechanisms include at least five non-fixed joints", len(non_fixed) >= 5, details=f"joints={[joint.name for joint in non_fixed]}")

    for visual_name in (
        "long_hood",
        "hood_raised_spine",
        "front_grille_panel",
        "number_badge_520",
        "john_deere_letter_0",
        "vertical_exhaust",
        "exhaust_top_step",
        "headlight_0",
        "headlight_bezel_0",
        "rear_hitch_mount",
        "rear_fender_0",
        "rear_fender_1",
    ):
        ctx.check(f"chassis includes {visual_name}", object_model.get_part("chassis").get_visual(visual_name) is not None)
    ctx.check("steering wheel is a movable part", object_model.get_part("steering_wheel").get_visual("steering_wheel_ring") is not None)

    rear_aabb = ctx.part_world_aabb("rear_wheel_0")
    front_aabb = ctx.part_world_aabb("front_wheel_0")
    rear_diameter = rear_aabb[1][2] - rear_aabb[0][2] if rear_aabb and front_aabb else 0.0
    front_diameter = front_aabb[1][2] - front_aabb[0][2] if rear_aabb and front_aabb else 0.0
    ctx.check(
        "rear tires are visibly larger than front wheels",
        rear_diameter > front_diameter * 1.55,
        details=f"rear_diameter={rear_diameter}, front_diameter={front_diameter}",
    )

    steering = object_model.get_articulation("steering_wheel_turn")
    front_axle_joint = object_model.get_articulation("chassis_to_front_axle")
    wheel_spin = object_model.get_articulation("rear_wheel_0_spin")
    hitch_joint = object_model.get_articulation("chassis_to_hitch")

    rest_front_0 = ctx.part_world_position("front_wheel_0")
    rest_front_1 = ctx.part_world_position("front_wheel_1")
    rest_spoke_aabb = ctx.part_element_world_aabb("steering_wheel", elem="steering_spoke_0")
    with ctx.pose({steering: 1.0}):
        turned_front_0 = ctx.part_world_position("front_wheel_0")
        turned_front_1 = ctx.part_world_position("front_wheel_1")
        turned_spoke_aabb = ctx.part_element_world_aabb("steering_wheel", elem="steering_spoke_0")
    ctx.check(
        "steering wheel yaws the whole front axle as one piece and swings both front wheels",
        front_axle_joint.articulation_type == ArticulationType.REVOLUTE
        and front_axle_joint.mimic is not None
        and front_axle_joint.mimic.joint == "steering_wheel_turn"
        and rest_front_0 is not None
        and turned_front_0 is not None
        and abs(turned_front_0[0] - rest_front_0[0]) > 0.02
        and rest_front_1 is not None
        and turned_front_1 is not None
        and abs(turned_front_1[0] - rest_front_1[0]) > 0.02
        # the two wheels swing in opposite fore/aft directions as the beam yaws
        and (turned_front_0[0] - rest_front_0[0]) * (turned_front_1[0] - rest_front_1[0]) < 0.0,
        details=f"front0={rest_front_0}->{turned_front_0}, front1={rest_front_1}->{turned_front_1}",
    )
    ctx.check(
        "steering wheel itself visibly rotates",
        rest_spoke_aabb is not None
        and turned_spoke_aabb is not None
        and max(abs(turned_spoke_aabb[i][j] - rest_spoke_aabb[i][j]) for i in range(2) for j in range(3)) > 0.03,
        details=f"rest_spoke={rest_spoke_aabb}, turned_spoke={turned_spoke_aabb}",
    )

    rest_hitch_aabb = ctx.part_world_aabb("hitch")
    with ctx.pose({hitch_joint: 0.38, wheel_spin: 1.2}):
        raised_hitch_aabb = ctx.part_world_aabb("hitch")
        posed_rear_aabb = ctx.part_world_aabb("rear_wheel_0")
    ctx.check(
        "hitch linkage raises about its rear pivot",
        rest_hitch_aabb is not None and raised_hitch_aabb is not None and raised_hitch_aabb[1][2] > rest_hitch_aabb[1][2] + 0.12,
        details=f"rest={rest_hitch_aabb}, raised={raised_hitch_aabb}",
    )
    for wheel_name, joint_name in (
        ("rear_wheel_0", "rear_wheel_0_spin"),
        ("rear_wheel_1", "rear_wheel_1_spin"),
        ("front_wheel_0", "front_wheel_0_spin"),
        ("front_wheel_1", "front_wheel_1_spin"),
    ):
        spin_joint = object_model.get_articulation(joint_name)
        rest_pos = ctx.part_world_position(wheel_name)
        with ctx.pose({spin_joint: 1.2}):
            spun_pos = ctx.part_world_position(wheel_name)
        ctx.check(
            f"{wheel_name} spin keeps hub center mounted",
            rest_pos is not None and spun_pos is not None and max(abs(spun_pos[i] - rest_pos[i]) for i in range(3)) < 0.001,
            details=f"rest={rest_pos}, spun={spun_pos}, posed_rear_aabb={posed_rear_aabb}",
        )

    # The single front axle beam pivots about a central vertical pin that is
    # intentionally captured inside the front bolster / pedestal casting.
    for elem in ("front_bolster", "front_pedestal"):
        ctx.allow_overlap(
            "chassis",
            "front_axle",
            elem_a=elem,
            elem_b="center_pivot_pin",
            reason="The front axle's central vertical pivot pin is intentionally captured inside the front casting.",
        )
    ctx.allow_overlap(
        "chassis",
        "front_axle",
        elem_a="front_bolster",
        elem_b="front_axle_beam",
        reason="The center of the front axle beam rises into the pivot bolster casting around the pivot pin.",
    )
    ctx.allow_overlap(
        "chassis",
        "steering_wheel",
        elem_a="steering_column",
        elem_b="steering_hub",
        reason="The steering wheel hub is intentionally mounted on the top end of the steering column.",
    )
    for spoke_idx in range(3):
        ctx.allow_overlap(
            "chassis",
            "steering_wheel",
            elem_a="steering_column",
            elem_b=f"steering_spoke_{spoke_idx}",
            reason="The steering column tip meets the wheel at the hub where the spokes radiate, so a spoke shares the mount region.",
        )
    ctx.allow_overlap(
        "chassis",
        "hitch",
        elem_a="rear_hitch_mount",
        elem_b="hitch_pivot_pin",
        reason="The drawbar pivot pin is intentionally seated through the rear hitch bracket.",
    )
    for wheel_name in ("rear_wheel_0", "rear_wheel_1"):
        ctx.allow_overlap(
            "chassis",
            wheel_name,
            elem_a="rear_axle_housing",
            elem_b="yellow_rim",
            reason="The axle stub is intentionally seated into the wheel hub/rim.",
        )
    for spindle_name, wheel_name, boss_name in (
        ("spindle_0", "front_wheel_0", "knuckle_boss_0"),
        ("spindle_1", "front_wheel_1", "knuckle_boss_1"),
    ):
        ctx.allow_overlap(
            "front_axle",
            wheel_name,
            elem_a=spindle_name,
            elem_b="yellow_rim",
            reason="The front axle beam spindle is intentionally captured in the front wheel hub so the wheel is visibly mounted.",
        )
        ctx.allow_overlap(
            "front_axle",
            wheel_name,
            elem_a=boss_name,
            elem_b="yellow_rim",
            reason="The knuckle boss wraps around the spindle at the wheel hub junction, seated inside the rim opening.",
        )
        ctx.allow_overlap(
            "front_axle",
            wheel_name,
            elem_a="front_axle_beam",
            elem_b="yellow_rim",
            reason="The shortened front axle beam spans between the closely spaced spindles and passes through the wheel hub region.",
        )
        ctx.allow_overlap(
            "front_axle",
            wheel_name,
            elem_a="tie_rod",
            elem_b="yellow_rim",
            reason="The tie rod steering linkage passes near the wheel hub as it connects the two closely spaced front wheel knuckles.",
        )
        ctx.allow_overlap(
            "front_axle",
            wheel_name,
            elem_a="center_pivot_pin",
            elem_b="yellow_rim",
            reason="The central pivot pin sits between the two closely spaced narrow-front wheels where their rim openings nearly meet at the centerline.",
        )
    ctx.expect_overlap("front_axle", "chassis", axes="xy", min_overlap=0.04, elem_a="center_pivot_pin", elem_b="front_bolster", name="front pivot pin is captured in bolster")
    ctx.expect_overlap("front_axle", "chassis", axes="xy", min_overlap=0.04, elem_a="center_pivot_pin", elem_b="front_pedestal", name="front pivot pin is captured in pedestal")
    ctx.expect_overlap("hitch", "chassis", axes="y", min_overlap=0.20, elem_a="hitch_pivot_pin", elem_b="rear_hitch_mount", name="hitch pin spans rear bracket")
    ctx.expect_overlap("rear_wheel_0", "chassis", axes="xz", min_overlap=0.06, elem_a="yellow_rim", elem_b="rear_axle_housing", name="rear rim aligns with axle housing")
    ctx.expect_overlap("rear_wheel_1", "chassis", axes="xz", min_overlap=0.06, elem_a="yellow_rim", elem_b="rear_axle_housing", name="opposite rear rim aligns with axle housing")
    ctx.expect_overlap("front_wheel_0", "front_axle", axes="xz", min_overlap=0.03, elem_a="yellow_rim", elem_b="spindle_0", name="front wheel hub is on the axle spindle")
    ctx.expect_overlap("front_wheel_1", "front_axle", axes="xz", min_overlap=0.03, elem_a="yellow_rim", elem_b="spindle_1", name="opposite front wheel hub is on the axle spindle")

    # Narrow row-crop tricycle front: the shortened beam and closely spaced
    # spindles place both front wheels near the centerline.
    front_wheel_0_center = ctx.part_world_position("front_wheel_0")
    front_wheel_1_center = ctx.part_world_position("front_wheel_1")
    ctx.check(
        "narrow row-crop front: both front wheels sit close together near centerline",
        front_wheel_0_center is not None
        and front_wheel_1_center is not None
        and abs(front_wheel_0_center[1]) < 0.20
        and abs(front_wheel_1_center[1]) < 0.20
        and abs(front_wheel_0_center[1] - front_wheel_1_center[1]) < 0.30,
        details=f"front_wheel_0_y={front_wheel_0_center[1] if front_wheel_0_center else None}, "
        f"front_wheel_1_y={front_wheel_1_center[1] if front_wheel_1_center else None}",
    )
    front_beam_aabb = ctx.part_element_world_aabb("front_axle", elem="front_axle_beam")
    ctx.check(
        "front_axle_beam is shortened for narrow tricycle front",
        front_beam_aabb is not None and (front_beam_aabb[1][1] - front_beam_aabb[0][1]) < 0.40,
        details=f"beam_y_extent={front_beam_aabb[1][1] - front_beam_aabb[0][1] if front_beam_aabb else None}",
    )
    fender_aabb = ctx.part_element_world_aabb("chassis", elem="rear_fender_0")
    fender_aabb_1 = ctx.part_element_world_aabb("chassis", elem="rear_fender_1")
    ctx.check(
        "rear fenders arch over the tires",
        fender_aabb is not None
        and fender_aabb_1 is not None
        and rear_aabb is not None
        and fender_aabb[0][2] > rear_aabb[0][2] + 0.70
        and fender_aabb_1[0][2] > rear_aabb[0][2] + 0.70,
        details=f"fender_0={fender_aabb}, fender_1={fender_aabb_1}, tire={rear_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
