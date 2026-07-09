from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireSidewall,
    TireShoulder,
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


TAU = 2.0 * math.pi


def circle_profile(radius: float, segments: int = 72) -> list[tuple[float, float]]:
    return [
        (radius * math.cos(TAU * i / segments), radius * math.sin(TAU * i / segments))
        for i in range(segments)
    ]


def slot_profile(width: float, height: float, segments: int = 12) -> list[tuple[float, float]]:
    """Rounded slot profile centered on the origin, long dimension along Y."""
    r = width * 0.5
    straight = max(0.0, height - width)
    pts: list[tuple[float, float]] = []
    # top semicircle, then bottom semicircle
    for i in range(segments + 1):
        a = math.pi - math.pi * i / segments
        pts.append((r * math.cos(a), straight * 0.5 + r * math.sin(a)))
    for i in range(segments + 1):
        a = -math.pi * i / segments
        pts.append((r * math.cos(a), -straight * 0.5 + r * math.sin(a)))
    return pts


def offset_profile(profile: list[tuple[float, float]], dy: float, dz: float) -> list[tuple[float, float]]:
    return [(y + dy, z + dz) for (y, z) in profile]


def map_profile_extrusion_to_yz(geom):
    """Map Extrude* local (profile_x, profile_y, thickness_z) to (x, y, z)."""
    mapped = geom.copy()
    mapped.vertices = [(z, x, y) for (x, y, z) in mapped.vertices]
    return mapped


def annular_yz(radius: float, hole_radius: float, thickness_x: float, *, segments: int = 96):
    geom = ExtrudeWithHolesGeometry(
        circle_profile(radius, segments),
        [circle_profile(hole_radius, segments)],
        thickness_x,
        center=True,
    )
    return map_profile_extrusion_to_yz(geom)


def plate_yz(outer, holes, thickness_x: float):
    geom = ExtrudeWithHolesGeometry(outer, holes, thickness_x, center=True)
    return map_profile_extrusion_to_yz(geom)


def torus_around_x(radius: float, tube: float, *, radial_segments: int = 16, tubular_segments: int = 72):
    geom = TorusGeometry(radius, tube, radial_segments=radial_segments, tubular_segments=tubular_segments)
    return map_profile_extrusion_to_yz(geom)


def add_x_cylinder(part, name: str, radius: float, length: float, xyz, material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="electrical_cable_reel",
        meta={
            "domain": "Electrical_Wiring",
            "small_class": "Cable reel",
            "description": "Portable trolley-style electrical cable reel on a two-wheeled cart base with push-handle, rotating flanged drum, wound rubber cable, crank handle, outlet block, labels, fasteners, and strain relief.",
        },
    )

    cream = model.material("painted_cream_metal", rgba=(0.78, 0.74, 0.62, 1.0))
    cream_shadow = model.material("recessed_cream_shadow", rgba=(0.62, 0.59, 0.50, 1.0))
    black = model.material("molded_black_plastic", rgba=(0.015, 0.014, 0.013, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.005, 0.005, 0.006, 1.0))
    dark = model.material("socket_dark_recess", rgba=(0.0, 0.0, 0.0, 1.0))
    metal = model.material("galvanized_steel", rgba=(0.68, 0.70, 0.68, 1.0))
    brass = model.material("brass_terminal", rgba=(0.95, 0.72, 0.28, 1.0))
    label = model.material("printed_warning_label", rgba=(0.95, 0.92, 0.72, 1.0))
    red = model.material("red_logo_print", rgba=(0.85, 0.08, 0.05, 1.0))

    axle_z = 0.36

    frame = model.part("frame")

    # Cart platform: base rails, uprights, cross-axle, two wheels, and trolley push-handle.
    # The cart replaces the static skid U-base while keeping the same side-plate frame above.
    cart_platform_z = 0.080  # base-rail center height (raised to clear wheels below)
    wheel_radius = 0.058     # 116 mm wheel diameter — a common trolley caster size
    wheel_center_z = 0.000   # cross-axle at ground level (wheels rest on floor)
    wheel_y = 0.310          # wheel track half-width (outside the base rails)
    wheel_x = -0.020         # axle slightly forward of cart center for stable tilt

    for idx, y in enumerate((-0.235, 0.235)):
        frame.visual(
            Box((0.86, 0.035, 0.045)),
            origin=Origin(xyz=(0.0, y, cart_platform_z)),
            material=cream,
            name=f"base_rail_{idx}",
        )
    # Uprights connect base rails to the side-plate frame above.
    upright_base_z = cart_platform_z + 0.0225  # top of base rail
    upright_top_z = axle_z - 0.078              # bottom of side plate (axle_z = 0.36)
    upright_height = upright_top_z - upright_base_z
    upright_center_z = 0.5 * (upright_base_z + upright_top_z)
    for idx, (x, y) in enumerate(((-0.385, -0.205), (-0.385, 0.205), (0.385, -0.205), (0.385, 0.205))):
        frame.visual(
            Box((0.050, 0.040, upright_height)),
            origin=Origin(xyz=(x, y, upright_center_z)),
            material=cream,
            name=f"rail_upright_{idx}",
        )

    # Cross-axle tube connecting the two wheels under the cart platform.
    frame.visual(
        Cylinder(radius=0.014, length=2.0 * wheel_y + 0.060),
        origin=Origin(xyz=(wheel_x, 0.0, wheel_center_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="cross_axle_bar",
    )

    # Wheel fork brackets that hang from the base rails down to the cross-axle.
    # Each fork spans from the rail Y position out to the wheel Y position.
    rail_bottom_z = cart_platform_z - 0.0225
    fork_height = rail_bottom_z - wheel_center_z + 0.010  # overlap with rail bottom and axle
    fork_center_z = 0.5 * (rail_bottom_z + wheel_center_z)
    rail_y = 0.235  # base rail Y position
    for idx, y_sign in enumerate((-1.0, 1.0)):
        y_rail = y_sign * rail_y
        y_wheel = y_sign * wheel_y
        # Fork Y span from rail to wheel
        fork_y_center = 0.5 * (y_rail + y_wheel)
        fork_y_width = abs(y_wheel - y_rail) + 0.020  # extend past both rail and wheel centers
        # Vertical fork leg from rail bottom down to axle height.
        frame.visual(
            Box((0.040, fork_y_width, fork_height)),
            origin=Origin(xyz=(wheel_x - 0.015, fork_y_center, fork_center_z)),
            material=cream,
            name=f"wheel_fork_{idx}",
        )
        # Axle bearing boss on each fork.
        add_x_cylinder(
            frame, f"wheel_boss_{idx}", 0.022, 0.026,
            (wheel_x, y_wheel, wheel_center_z), metal,
        )

    # Two trolley wheels with rubber tires on the cross-axle.
    def build_trolley_wheel(name_prefix: str):
        wheel_geom = WheelGeometry(
            wheel_radius * 0.78,
            0.032,
            rim=WheelRim(
                inner_radius=wheel_radius * 0.52,
                flange_height=0.006,
                flange_thickness=0.003,
                bead_seat_depth=0.003,
            ),
            hub=WheelHub(
                radius=0.020,
                width=0.028,
                cap_style="flat",
            ),
            face=WheelFace(dish_depth=0.002),
            spokes=WheelSpokes(style="straight", count=4, thickness=0.004),
            bore=WheelBore(style="round", diameter=0.014),
            center=True,
        )
        tire_geom = TireGeometry(
            wheel_radius,
            0.036,
            inner_radius=wheel_radius * 0.74,
            sidewall=TireSidewall(style="square", bulge=0.01),
        )
        return wheel_geom, tire_geom

    for idx, y_sign in enumerate((-1.0, 1.0)):
        y_wheel = y_sign * wheel_y
        wheel_g, tire_g = build_trolley_wheel(f"cart_wheel_{idx}")
        # WheelGeometry spins about local X; place so the wheel sits in YZ plane at the fork.
        frame.visual(
            mesh_from_geometry(wheel_g, f"cart_wheel_rim_{idx}"),
            origin=Origin(xyz=(wheel_x, y_wheel, wheel_center_z)),
            material=metal,
            name=f"cart_wheel_rim_{idx}",
        )
        frame.visual(
            mesh_from_geometry(tire_g, f"cart_wheel_tire_{idx}"),
            origin=Origin(xyz=(wheel_x, y_wheel, wheel_center_z)),
            material=rubber,
            name=f"cart_wheel_tire_{idx}",
        )
        # Axle nut on outside of each wheel.
        add_x_cylinder(
            frame, f"cart_wheel_nut_{idx}", 0.012, 0.008,
            (wheel_x, y_wheel + y_sign * 0.024, wheel_center_z), metal,
        )

    # Trolley push-handle: two vertical posts from the rear of the cart, joined by a cross-grip.
    handle_base_x = 0.420
    handle_top_x = 0.470
    handle_top_z = 0.620
    handle_y_span = 0.205
    for idx, y_sign in enumerate((-1.0, 1.0)):
        y_post = y_sign * handle_y_span
        post_path = [
            (handle_base_x, y_post, cart_platform_z + 0.022),
            (handle_base_x + 0.020, y_post, cart_platform_z + 0.180),
            (handle_top_x, y_post, handle_top_z - 0.040),
            (handle_top_x, y_post, handle_top_z),
        ]
        frame.visual(
            mesh_from_geometry(
                tube_from_spline_points(post_path, radius=0.012, samples_per_segment=8, radial_segments=12),
                f"trolley_post_{idx}_mesh",
            ),
            origin=Origin(),
            material=cream,
            name=f"trolley_post_{idx}",
        )
    # Horizontal cross-grip at the top of the push-handle.
    frame.visual(
        Cylinder(radius=0.014, length=2.0 * handle_y_span + 0.020),
        origin=Origin(xyz=(handle_top_x, 0.0, handle_top_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="trolley_grip_bar",
    )
    # Small end caps on the grip bar.
    for idx, y_sign in enumerate((-1.0, 1.0)):
        add_x_cylinder(
            frame, f"trolley_grip_cap_{idx}", 0.016, 0.006,
            (handle_top_x, y_sign * (handle_y_span + 0.012), handle_top_z), black,
        )

    side_outer = [
        (-0.205, -0.315),
        (0.185, -0.315),
        (0.165, -0.150),
        (0.095, 0.145),
        (0.020, 0.285),
        (-0.115, 0.235),
        (-0.190, 0.055),
    ]
    side_holes = [
        circle_profile(0.066, 72),
        offset_profile(slot_profile(0.058, 0.118, 16), -0.105, -0.235),  # large lower lightening/hand cutout
        offset_profile(slot_profile(0.018, 0.072, 10), 0.132, -0.205),
        offset_profile(slot_profile(0.018, 0.064, 10), -0.155, -0.095),
    ]
    side_plate_mesh = plate_yz(side_outer, side_holes, 0.034)
    for suffix, x in (("front", -0.385), ("rear", 0.385)):
        frame.visual(
            mesh_from_geometry(side_plate_mesh, f"{suffix}_side_plate_mesh"),
            origin=Origin(xyz=(x, 0.0, axle_z)),
            material=cream,
            name=f"{suffix}_side_plate",
        )
        add_x_cylinder(frame, f"{suffix}_bearing_race", 0.078, 0.018, (x - 0.019 if x < 0 else x + 0.019, 0.0, axle_z), metal)

    # Static visible axle stubs and shaft hardware at the two bearing supports.
    add_x_cylinder(frame, "axle_shaft", 0.032, 0.090, (-0.435, 0.0, axle_z), metal)
    add_x_cylinder(frame, "rear_axle_stub", 0.032, 0.090, (0.435, 0.0, axle_z), metal)
    for name, x in (("front_axle_nut", -0.475), ("rear_axle_nut", 0.475)):
        add_x_cylinder(frame, name, 0.045, 0.030, (x, 0.0, axle_z), metal)
    for i, (y, z_rel) in enumerate(((0.118, 0.070), (-0.118, 0.070), (0.130, -0.105), (-0.140, -0.185), (0.055, 0.205))):
        add_x_cylinder(frame, f"frame_screw_{i}", 0.010, 0.006, (-0.404, y, axle_z + z_rel), metal)

    # Small printed rating plate and brand mark on the front side bracket.
    frame.visual(
        Box((0.004, 0.080, 0.052)),
        origin=Origin(xyz=(-0.404, -0.046, axle_z - 0.138)),
        material=label,
        name="rating_label",
    )
    frame.visual(
        Box((0.005, 0.030, 0.010)),
        origin=Origin(xyz=(-0.408, -0.073, axle_z - 0.113)),
        material=red,
        name="red_logo",
    )

    reel = model.part("reel")

    # Drum, side cheeks, raised lips, hub, and visible bolted pattern.
    reel.visual(
        mesh_from_geometry(annular_yz(0.150, 0.052, 0.530, segments=96), "perforated_drum_core_mesh"),
        origin=Origin(),
        material=black,
        name="drum_core",
    )
    for suffix, x in (("front", -0.270), ("rear", 0.270)):
        cheek_x = -0.269 if x < 0 else 0.269
        reel.visual(
            mesh_from_geometry(annular_yz(0.232, 0.055, 0.038, segments=112), f"{suffix}_spool_cheek_mesh"),
            origin=Origin(xyz=(cheek_x, 0.0, 0.0)),
            material=cream,
            name=f"{suffix}_spool_cheek",
        )
        reel.visual(
            mesh_from_geometry(torus_around_x(0.220, 0.012, tubular_segments=96), f"{suffix}_rolled_lip_mesh"),
            origin=Origin(xyz=(cheek_x - 0.002 if x < 0 else cheek_x + 0.002, 0.0, 0.0)),
            material=cream,
            name=f"{suffix}_rolled_lip",
        )
        reel.visual(
            mesh_from_geometry(annular_yz(0.088, 0.044, 0.052, segments=80), f"{suffix}_hub_collar_mesh"),
            origin=Origin(xyz=(cheek_x - 0.030 if x < 0 else cheek_x + 0.030, 0.0, 0.0)),
            material=cream,
            name=f"{suffix}_hub_collar",
        )

    # Concentric pressed detail on front cheek.
    for r, nm in ((0.145, "front_recess_ring"), (0.185, "front_outer_recess")):
        reel.visual(
            mesh_from_geometry(torus_around_x(r, 0.0035, tubular_segments=96), f"{nm}_mesh"),
            origin=Origin(xyz=(-0.292, 0.0, 0.0)),
            material=cream_shadow,
            name=nm,
        )

    for i in range(8):
        a = TAU * i / 8.0 + math.radians(12.0)
        y = 0.112 * math.cos(a)
        z = 0.112 * math.sin(a)
        add_x_cylinder(reel, f"flange_bolt_{i}", 0.007, 0.006, (-0.291, y, z), metal)
    for i in range(6):
        a = TAU * i / 6.0
        y = 0.175 * math.cos(a)
        z = 0.175 * math.sin(a)
        add_x_cylinder(reel, f"vent_hole_dark_{i}", 0.006, 0.004, (-0.287, y, z), dark)

    # Wound rubber cable: a continuous helix plus a subtle under-drum.
    helix_points = []
    turns = 25
    samples = turns * 10 + 1
    for i in range(samples):
        t = i / (samples - 1)
        x = -0.228 + 0.456 * t
        a = TAU * turns * t
        helix_points.append((x, 0.159 * math.cos(a), 0.159 * math.sin(a)))
    cable_helix = tube_from_spline_points(
        helix_points,
        radius=0.0085,
        samples_per_segment=2,
        radial_segments=14,
        cap_ends=True,
    )
    reel.visual(
        mesh_from_geometry(cable_helix, "wound_cable_helix_mesh"),
        origin=Origin(),
        material=rubber,
        name="wound_cable_helix",
    )
    # Short loose cable tail with molded strain relief emerging from the winding pack.
    tail_path = [
        (-0.232, 0.156, 0.012),
        (-0.278, 0.178, 0.030),
        (-0.306, 0.196, 0.075),
        (-0.318, 0.214, 0.125),
    ]
    reel.visual(
        mesh_from_geometry(tube_from_spline_points(tail_path, radius=0.010, samples_per_segment=10, radial_segments=16), "cable_tail_mesh"),
        origin=Origin(),
        material=rubber,
        name="cable_tail",
    )
    reel.visual(
        Box((0.035, 0.050, 0.030)),
        origin=Origin(xyz=(-0.315, 0.198, 0.095), rpy=(0.0, 0.0, 0.18)),
        material=black,
        name="strain_relief",
    )
    reel.visual(
        Box((0.006, 0.060, 0.036)),
        origin=Origin(xyz=(-0.335, 0.198, 0.095), rpy=(0.0, 0.0, 0.18)),
        material=metal,
        name="strain_relief_band",
    )

    # Outlet/socket block and electrical terminal details on front flange.
    reel.visual(
        Box((0.040, 0.094, 0.066)),
        origin=Origin(xyz=(-0.306, -0.108, 0.080)),
        material=black,
        name="outlet_block",
    )
    for i, dz in enumerate((-0.018, 0.018)):
        add_x_cylinder(reel, f"socket_face_{i}", 0.014, 0.006, (-0.328, -0.108, 0.080 + dz), dark)
        add_x_cylinder(reel, f"brass_terminal_{i}", 0.004, 0.005, (-0.332, -0.108, 0.080 + dz), brass)
    reel.visual(
        Box((0.004, 0.070, 0.022)),
        origin=Origin(xyz=(-0.325, -0.108, 0.033)),
        material=label,
        name="warning_label",
    )

    # Crank arm fixed to the rotating reel, outside the front support plate.
    crank_points = [
        (-0.420, 0.000, 0.000),
        (-0.430, -0.045, -0.055),
        (-0.462, -0.100, -0.132),
        (-0.500, -0.130, -0.170),
    ]
    reel.visual(
        mesh_from_geometry(tube_from_spline_points(crank_points, radius=0.0075, samples_per_segment=14, radial_segments=14), "crank_arm_mesh"),
        origin=Origin(),
        material=metal,
        name="crank_arm",
    )
    add_x_cylinder(reel, "front_hub_neck", 0.026, 0.164, (-0.375, 0.0, 0.0), metal)
    add_x_cylinder(reel, "crank_root_boss", 0.034, 0.030, (-0.436, 0.0, 0.0), metal)
    add_x_cylinder(reel, "crank_pin", 0.008, 0.110, (-0.555, -0.130, -0.170), metal)
    add_x_cylinder(reel, "crank_washer", 0.020, 0.012, (-0.506, -0.130, -0.170), metal)

    grip = model.part("crank_grip")
    add_x_cylinder(grip, "rubber_sleeve", 0.018, 0.086, (-0.056, 0.0, 0.0), rubber)
    grip.visual(
        Box((0.075, 0.006, 0.006)),
        origin=Origin(xyz=(-0.056, 0.0, 0.018)),
        material=cream_shadow,
        name="grip_rib",
    )
    add_x_cylinder(grip, "end_cap", 0.019, 0.006, (-0.102, 0.0, 0.0), black)

    model.articulation(
        "frame_to_reel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=reel,
        origin=Origin(xyz=(0.0, 0.0, axle_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=4.0, lower=-TAU, upper=TAU),
    )
    model.articulation(
        "reel_to_crank_grip",
        ArticulationType.CONTINUOUS,
        parent=reel,
        child=grip,
        origin=Origin(xyz=(-0.500, -0.130, -0.170)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    reel = object_model.get_part("reel")
    grip = object_model.get_part("crank_grip")
    reel_joint = object_model.get_articulation("frame_to_reel")
    grip_joint = object_model.get_articulation("reel_to_crank_grip")

    ctx.allow_overlap(
        frame,
        reel,
        elem_a="axle_shaft",
        elem_b="crank_arm",
        reason="The crank arm root is intentionally keyed onto the projecting axle/shaft at the reel center.",
    )
    ctx.allow_overlap(
        frame,
        reel,
        elem_a="axle_shaft",
        elem_b="crank_root_boss",
        reason="The crank root boss is a visible hub clamped around the rotating axle end.",
    )
    ctx.allow_overlap(
        frame,
        reel,
        elem_a="axle_shaft",
        elem_b="front_hub_neck",
        reason="The visible front rotating hub neck is concentric around the fixed axle stub.",
    )
    ctx.allow_overlap(
        frame,
        reel,
        elem_a="front_bearing_race",
        elem_b="front_hub_neck",
        reason="The front hub neck intentionally passes through the bearing race carried by the side plate.",
    )
    ctx.allow_overlap(
        frame,
        reel,
        elem_a="front_side_plate",
        elem_b="front_hub_neck",
        reason="The front rotating hub neck intentionally passes through the bored bearing opening in the side plate.",
    )
    ctx.allow_overlap(
        grip,
        reel,
        elem_a="rubber_sleeve",
        elem_b="crank_pin",
        reason="The free-spinning rubber crank sleeve is intentionally modeled around its metal handle pin.",
    )
    ctx.allow_overlap(
        grip,
        reel,
        elem_a="end_cap",
        elem_b="crank_pin",
        reason="The molded end cap sits on the end of the handle pin to retain the crank grip.",
    )

    ctx.check(
        "small class is Cable reel",
        object_model.meta.get("small_class") == "Cable reel" and "cable_reel" in object_model.name,
        details=f"name={object_model.name}, meta={object_model.meta}",
    )

    for part_obj, names, label in (
        (
            reel,
            [
                "front_spool_cheek",
                "rear_spool_cheek",
                "drum_core",
                "wound_cable_helix",
                "cable_tail",
                "outlet_block",
                "strain_relief",
                "warning_label",
            ],
            "reel visible subassemblies",
        ),
        (
            frame,
            [
                "front_side_plate",
                "rear_side_plate",
                "base_rail_0",
                "base_rail_1",
                "axle_shaft",
                "rating_label",
                "cart_wheel_rim_0",
                "cart_wheel_tire_0",
                "cart_wheel_rim_1",
                "cart_wheel_tire_1",
                "cross_axle_bar",
                "trolley_post_0",
                "trolley_post_1",
                "trolley_grip_bar",
            ],
            "frame visible subassemblies",
        ),
        (grip, ["rubber_sleeve", "grip_rib"], "crank grip geometry"),
    ):
        missing = []
        for visual_name in names:
            try:
                part_obj.get_visual(visual_name)
            except Exception:
                missing.append(visual_name)
        ctx.check(label, not missing, details=f"missing visuals: {missing}")

    # Cart-variant specific: the wheeled trolley base replaces the static skid.
    ctx.expect_gap(
        frame,
        frame,
        axis="z",
        positive_elem="base_rail_0",
        negative_elem="cart_wheel_tire_0",
        min_gap=-0.020,
        max_gap=0.030,
        name="cart wheel sits at or below the base rail (wheeled trolley base)",
    )
    ctx.expect_overlap(
        frame,
        frame,
        axes="z",
        elem_a="cross_axle_bar",
        elem_b="cart_wheel_rim_0",
        min_overlap=0.020,
        name="cross-axle bar spans the cart wheels",
    )
    ctx.expect_gap(
        frame,
        reel,
        axis="z",
        positive_elem="trolley_grip_bar",
        negative_elem="drum_core",
        min_gap=0.020,
        name="trolley push-handle grip extends above the reel drum for pushing",
    )
    ctx.expect_overlap(
        frame,
        frame,
        axes="x",
        elem_a="trolley_post_0",
        elem_b="base_rail_0",
        min_overlap=0.010,
        name="trolley post is mounted on the base rail",
    )

    ctx.check(
        "reel rotates on central x axle",
        reel_joint.articulation_type != ArticulationType.FIXED
        and tuple(round(v, 3) for v in reel_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={reel_joint.articulation_type}, axis={reel_joint.axis}",
    )
    ctx.check(
        "crank grip spins on its handle pin",
        grip_joint.articulation_type != ArticulationType.FIXED
        and tuple(round(v, 3) for v in grip_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={grip_joint.articulation_type}, axis={grip_joint.axis}",
    )

    ctx.expect_within(
        reel,
        frame,
        axes="x",
        inner_elem="wound_cable_helix",
        outer_elem="base_rail_0",
        margin=0.04,
        name="wound cable stays between the support-frame ends",
    )
    ctx.expect_gap(
        reel,
        frame,
        axis="x",
        positive_elem="front_spool_cheek",
        negative_elem="front_side_plate",
        min_gap=0.050,
        max_gap=0.120,
        name="front spool cheek clears the front side plate",
    )
    ctx.expect_gap(
        frame,
        reel,
        axis="x",
        positive_elem="rear_side_plate",
        negative_elem="rear_spool_cheek",
        min_gap=0.050,
        max_gap=0.120,
        name="rear spool cheek clears the rear side plate",
    )
    ctx.expect_overlap(
        reel,
        frame,
        axes="yz",
        elem_a="front_hub_collar",
        elem_b="front_bearing_race",
        min_overlap=0.040,
        name="front hub collar aligns with front bearing race",
    )
    ctx.expect_overlap(
        reel,
        frame,
        axes="yz",
        elem_a="rear_hub_collar",
        elem_b="rear_bearing_race",
        min_overlap=0.040,
        name="rear hub collar aligns with rear bearing race",
    )
    ctx.expect_overlap(
        frame,
        reel,
        axes="x",
        elem_a="axle_shaft",
        elem_b="crank_arm",
        min_overlap=0.025,
        name="crank arm root is keyed to axle",
    )
    ctx.expect_overlap(
        frame,
        reel,
        axes="x",
        elem_a="axle_shaft",
        elem_b="crank_root_boss",
        min_overlap=0.025,
        name="crank root boss surrounds axle",
    )
    ctx.expect_overlap(
        frame,
        reel,
        axes="x",
        elem_a="front_bearing_race",
        elem_b="front_hub_neck",
        min_overlap=0.006,
        name="front bearing race captures hub neck",
    )
    ctx.expect_overlap(
        frame,
        reel,
        axes="x",
        elem_a="front_side_plate",
        elem_b="front_hub_neck",
        min_overlap=0.020,
        name="front hub neck passes through side plate bearing opening",
    )
    ctx.expect_within(
        reel,
        frame,
        axes="yz",
        inner_elem="drum_core",
        outer_elem="axle_shaft",
        margin=0.13,
        name="drum is centered around the axle",
    )
    ctx.expect_overlap(
        reel,
        grip,
        axes="x",
        elem_a="crank_pin",
        elem_b="rubber_sleeve",
        min_overlap=0.070,
        name="crank grip sleeve remains on its pin",
    )
    ctx.expect_overlap(
        reel,
        grip,
        axes="x",
        elem_a="crank_pin",
        elem_b="end_cap",
        min_overlap=0.004,
        name="crank end cap retains the handle pin",
    )
    ctx.expect_within(
        reel,
        grip,
        axes="yz",
        inner_elem="crank_pin",
        outer_elem="rubber_sleeve",
        margin=0.012,
        name="crank pin is centered inside rubber sleeve",
    )

    ctx.expect_overlap(
        reel,
        frame,
        axes="yz",
        elem_a="front_hub_collar",
        elem_b="front_bearing_race",
        min_overlap=0.040,
        name="front hub collar is carried by front bearing race",
    )
    ctx.expect_contact(
        grip,
        reel,
        elem_a="rubber_sleeve",
        elem_b="crank_washer",
        contact_tol=0.004,
        name="rubber crank grip is seated on crank washer",
    )

    def aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    base_socket = aabb_center(ctx.part_element_world_aabb(reel, elem="outlet_block"))
    with ctx.pose({reel_joint: 0.85}):
        turned_socket = aabb_center(ctx.part_element_world_aabb(reel, elem="outlet_block"))
    ctx.check(
        "reel pose visibly carries outlet block around axle",
        base_socket is not None
        and turned_socket is not None
        and abs(base_socket[1] - turned_socket[1]) + abs(base_socket[2] - turned_socket[2]) > 0.035,
        details=f"rest={base_socket}, turned={turned_socket}",
    )

    base_rib = aabb_center(ctx.part_element_world_aabb(grip, elem="grip_rib"))
    with ctx.pose({grip_joint: math.pi / 2.0}):
        spun_rib = aabb_center(ctx.part_element_world_aabb(grip, elem="grip_rib"))
    ctx.check(
        "crank grip rib moves when handle spins",
        base_rib is not None
        and spun_rib is not None
        and abs(base_rib[1] - spun_rib[1]) + abs(base_rib[2] - spun_rib[2]) > 0.010,
        details=f"rest={base_rib}, spun={spun_rib}",
    )

    return ctx.report()


object_model = build_object_model()
