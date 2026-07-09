from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    MeshGeometry,
    Mimic,
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
)


N_PLANET_PINIONS = 4
PLANET_ORBIT_RADIUS = 0.040

ROT_Z_TO_X = (0.0, math.pi / 2.0, 0.0)
ROT_Z_TO_Y = (math.pi / 2.0, 0.0, 0.0)


def _add_quad(mesh: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    mesh.add_face(a, b, c)
    mesh.add_face(a, c, d)


def _annular_cylinder_mesh(
    *,
    inner_radius: float,
    outer_radius: float,
    width: float,
    segments: int = 64,
) -> MeshGeometry:
    """Hollow cylinder centered on local X, useful for bearings and shaft hubs."""
    mesh = MeshGeometry()
    outer_loops: list[list[int]] = []
    inner_loops: list[list[int]] = []
    for x in (-width / 2.0, width / 2.0):
        outer: list[int] = []
        inner: list[int] = []
        for i in range(segments):
            theta = 2.0 * math.pi * i / segments
            ct, st = math.cos(theta), math.sin(theta)
            outer.append(mesh.add_vertex(x, outer_radius * ct, outer_radius * st))
            inner.append(mesh.add_vertex(x, inner_radius * ct, inner_radius * st))
        outer_loops.append(outer)
        inner_loops.append(inner)

    for i in range(segments):
        j = (i + 1) % segments
        _add_quad(mesh, outer_loops[0][i], outer_loops[0][j], outer_loops[1][j], outer_loops[1][i])
        _add_quad(mesh, inner_loops[0][j], inner_loops[0][i], inner_loops[1][i], inner_loops[1][j])
        _add_quad(mesh, outer_loops[0][j], outer_loops[0][i], inner_loops[0][i], inner_loops[0][j])
        _add_quad(mesh, outer_loops[1][i], outer_loops[1][j], inner_loops[1][j], inner_loops[1][i])
    return mesh


def _toothed_bevel_mesh(
    *,
    inner_radius: float,
    root_radius: float,
    tooth_height: float,
    width: float,
    teeth: int,
    cone_delta: float,
    samples_per_tooth: int = 8,
) -> MeshGeometry:
    """Annular, visibly toothed bevel gear centered on local X."""
    segments = teeth * samples_per_tooth
    mesh = MeshGeometry()
    outer_loops: list[list[int]] = []
    inner_loops: list[list[int]] = []

    for side, x in enumerate((-width / 2.0, width / 2.0)):
        cone = -cone_delta if side == 0 else cone_delta
        outer: list[int] = []
        inner: list[int] = []
        for i in range(segments):
            theta = 2.0 * math.pi * i / segments
            tooth_phase = (i % samples_per_tooth) / samples_per_tooth
            # Broad triangular addendum with a root land between teeth.
            centered = abs(tooth_phase - 0.5) * 2.0
            tooth = max(0.0, 1.0 - centered / 0.72)
            # A small secondary ripple makes the tooth flanks less toy-like.
            tooth *= 0.92 + 0.08 * math.cos(4.0 * math.pi * tooth_phase)
            outer_radius = root_radius + cone + tooth_height * tooth
            ct, st = math.cos(theta), math.sin(theta)
            outer.append(mesh.add_vertex(x, outer_radius * ct, outer_radius * st))
            inner.append(mesh.add_vertex(x, inner_radius * ct, inner_radius * st))
        outer_loops.append(outer)
        inner_loops.append(inner)

    for i in range(segments):
        j = (i + 1) % segments
        _add_quad(mesh, outer_loops[0][i], outer_loops[0][j], outer_loops[1][j], outer_loops[1][i])
        _add_quad(mesh, inner_loops[0][j], inner_loops[0][i], inner_loops[1][i], inner_loops[1][j])
        _add_quad(mesh, outer_loops[0][j], outer_loops[0][i], inner_loops[0][i], inner_loops[0][j])
        _add_quad(mesh, outer_loops[1][i], outer_loops[1][j], inner_loops[1][j], inner_loops[1][i])
    return mesh


def _make_tire_mesh(name: str):
    return mesh_from_geometry(
        TireGeometry(
            0.135,
            0.082,
            inner_radius=0.092,
            carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.08),
            tread=TireTread(style="chevron", depth=0.008, count=26, angle_deg=28.0, land_ratio=0.56),
            grooves=(
                TireGroove(center_offset=-0.020, width=0.004, depth=0.003),
                TireGroove(center_offset=0.020, width=0.004, depth=0.003),
            ),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.008, radius=0.004),
        ),
        name,
    )


def _make_rim_mesh(name: str):
    return mesh_from_geometry(
        WheelGeometry(
            0.096,
            0.076,
            rim=WheelRim(
                inner_radius=0.060,
                flange_height=0.008,
                flange_thickness=0.004,
                bead_seat_depth=0.003,
            ),
            hub=WheelHub(
                radius=0.030,
                width=0.052,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.043, hole_diameter=0.005),
            ),
            face=WheelFace(dish_depth=0.010, front_inset=0.003, rear_inset=0.003),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.004, window_radius=0.014),
            bore=WheelBore(style="round", diameter=0.015),
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="exposed_differential_drive_wheel_assembly")

    rubber = model.material("satin_black_rubber", rgba=(0.008, 0.008, 0.007, 1.0))
    tire_edge = model.material("dark_tread_shadow", rgba=(0.0, 0.0, 0.0, 1.0))
    machined = model.material("brushed_aluminum", rgba=(0.72, 0.72, 0.68, 1.0))
    dark_metal = model.material("gunmetal_carrier", rgba=(0.12, 0.13, 0.14, 1.0))
    blue = model.material("blue_motor_housing", rgba=(0.02, 0.12, 0.54, 1.0))
    green = model.material("green_half_shaft", rgba=(0.0, 0.78, 0.22, 1.0))
    red = model.material("red_half_shaft", rgba=(0.78, 0.06, 0.05, 1.0))
    orange = model.material("anodized_orange_gear", rgba=(1.0, 0.36, 0.05, 1.0))
    brass = model.material("brass_side_gear", rgba=(0.88, 0.62, 0.20, 1.0))
    yellow = model.material("yellow_pinion_gear", rgba=(1.0, 0.74, 0.12, 1.0))
    bolt_black = model.material("black_oxide_bolts", rgba=(0.015, 0.015, 0.014, 1.0))

    carrier = model.part("carrier")

    # Compact structural carrier: a rear bridge plate, two bearing pedestals,
    # yoke plates, and motor support rails.  It is intentionally open at the
    # front so the differential gears remain exposed.
    carrier.visual(Box((0.300, 0.045, 0.030)), origin=Origin(xyz=(0.0, 0.135, 0.045)), material=dark_metal, name="lower_bridge")
    carrier.visual(Box((0.280, 0.030, 0.160)), origin=Origin(xyz=(0.0, 0.155, 0.130)), material=dark_metal, name="rear_plate")
    carrier.visual(Box((0.200, 0.045, 0.026)), origin=Origin(xyz=(0.0, 0.135, 0.272)), material=dark_metal, name="top_bridge")
    for sx in (-1.0, 1.0):
        x = sx * 0.155
        carrier.visual(Box((0.018, 0.052, 0.190)), origin=Origin(xyz=(x, 0.102, 0.135)), material=dark_metal, name=f"cheek_plate_{0 if sx < 0 else 1}")
        carrier.visual(Box((0.034, 0.064, 0.020)), origin=Origin(xyz=(x, 0.060, 0.130)), material=dark_metal, name=f"bearing_lug_{0 if sx < 0 else 1}")
        carrier.visual(Box((0.034, 0.018, 0.075)), origin=Origin(xyz=(x, 0.023, 0.074)), material=dark_metal, name=f"bearing_pedestal_{0 if sx < 0 else 1}")
        carrier.visual(
            mesh_from_geometry(_annular_cylinder_mesh(inner_radius=0.017, outer_radius=0.036, width=0.030), f"bearing_cap_{0 if sx < 0 else 1}"),
            origin=Origin(xyz=(x, 0.0, 0.130)),
            material=machined,
            name=f"bearing_cap_{0 if sx < 0 else 1}",
        )
        for z in (0.087, 0.173):
            carrier.visual(
                Cylinder(radius=0.006, length=0.005),
                origin=Origin(xyz=(x, 0.077, z), rpy=ROT_Z_TO_Y),
                material=bolt_black,
                name=f"cap_bolt_{0 if sx < 0 else 1}_{0 if z < 0.13 else 1}",
            )

    # Motor/input housing and its rigid mount.
    carrier.visual(Cylinder(radius=0.050, length=0.120), origin=Origin(xyz=(0.0, -0.255, 0.235), rpy=ROT_Z_TO_Y), material=blue, name="motor_can")
    carrier.visual(Cylinder(radius=0.054, length=0.014), origin=Origin(xyz=(0.0, -0.188, 0.235), rpy=ROT_Z_TO_Y), material=blue, name="motor_front_cap")
    carrier.visual(Cylinder(radius=0.040, length=0.018), origin=Origin(xyz=(0.0, -0.320, 0.235), rpy=ROT_Z_TO_Y), material=blue, name="motor_rear_cap")
    for i, z in enumerate((0.195, 0.215, 0.255, 0.275)):
        carrier.visual(Box((0.092, 0.012, 0.006)), origin=Origin(xyz=(0.0, -0.255, z)), material=blue, name=f"motor_cooling_fin_{i}")
    carrier.visual(Box((0.160, 0.016, 0.030)), origin=Origin(xyz=(0.0, -0.187, 0.235)), material=dark_metal, name="motor_saddle")
    for sx in (-1.0, 1.0):
        carrier.visual(Box((0.018, 0.345, 0.018)), origin=Origin(xyz=(sx * 0.075, -0.020, 0.246)), material=dark_metal, name=f"motor_support_rail_{0 if sx < 0 else 1}")
        carrier.visual(Cylinder(radius=0.005, length=0.006), origin=Origin(xyz=(sx * 0.075, -0.178, 0.246), rpy=ROT_Z_TO_Y), material=bolt_black, name=f"motor_mount_bolt_{0 if sx < 0 else 1}")
        carrier.visual(Box((0.020, 0.030, 0.072)), origin=Origin(xyz=(sx * 0.090, 0.145, 0.235)), material=dark_metal, name=f"top_bridge_post_{0 if sx < 0 else 1}")

    # Left wheel with green half shaft.
    left_wheel = model.part("left_wheel")
    left_wheel.visual(_make_tire_mesh("left_tire"), material=rubber, name="treaded_tire")
    left_wheel.visual(_make_rim_mesh("left_rim"), material=machined, name="split_spoke_rim")
    left_wheel.visual(Cylinder(radius=0.013, length=0.384), origin=Origin(xyz=(0.192, 0.0, 0.0), rpy=ROT_Z_TO_X), material=green, name="green_half_shaft")
    left_wheel.visual(Cylinder(radius=0.022, length=0.020), origin=Origin(xyz=(0.350, 0.0, 0.0), rpy=ROT_Z_TO_X), material=machined, name="shaft_collar")

    # Right wheel with red half shaft.
    right_wheel = model.part("right_wheel")
    right_wheel.visual(_make_tire_mesh("right_tire"), material=rubber, name="treaded_tire")
    right_wheel.visual(_make_rim_mesh("right_rim"), material=machined, name="split_spoke_rim")
    right_wheel.visual(Cylinder(radius=0.013, length=0.405), origin=Origin(xyz=(-0.2025, 0.0, 0.0), rpy=ROT_Z_TO_X), material=red, name="red_half_shaft")
    right_wheel.visual(Cylinder(radius=0.022, length=0.020), origin=Origin(xyz=(-0.180, 0.0, 0.0), rpy=ROT_Z_TO_X), material=machined, name="shaft_collar")

    # ──────────────────────────────────────────────────────────────────────
    # Exposed internal gears
    # ──────────────────────────────────────────────────────────────────────

    # Orange ring gear / carrier assembly.
    orange_gear = model.part("orange_gear")
    orange_gear.visual(
        mesh_from_geometry(
            _toothed_bevel_mesh(inner_radius=0.024, root_radius=0.086, tooth_height=0.016, width=0.036, teeth=34, cone_delta=0.010),
            "orange_bevel_teeth",
        ),
        material=orange,
        name="orange_bevel_teeth",
    )
    orange_gear.visual(
        mesh_from_geometry(_annular_cylinder_mesh(inner_radius=0.011, outer_radius=0.034, width=0.056), "orange_hub"),
        material=orange,
        name="orange_hub",
    )

    # Spider cross carried by the orange ring gear: central hub, four radial
    # arms, and four pin bosses that carry the planet pinions.
    orange_gear.visual(
        mesh_from_geometry(
            _annular_cylinder_mesh(inner_radius=0.012, outer_radius=0.017, width=0.024),
            "spider_hub",
        ),
        material=orange,
        name="spider_hub",
    )
    R_arm_mid = (0.017 + PLANET_ORBIT_RADIUS) / 2.0
    arm_len = PLANET_ORBIT_RADIUS - 0.017
    for i in range(N_PLANET_PINIONS):
        theta = i * 2.0 * math.pi / N_PLANET_PINIONS
        ct, st = math.cos(theta), math.sin(theta)
        # Arm box: long dimension along the radial direction.
        dy = arm_len * abs(ct) + 0.010 * abs(st)
        dz = 0.010 * abs(ct) + arm_len * abs(st)
        orange_gear.visual(
            Box((0.013, dy, dz)),
            origin=Origin(xyz=(0.0, R_arm_mid * ct, R_arm_mid * st)),
            material=orange,
            name=f"spider_arm_{i}",
        )
        # Pin boss at the planet orbit radius.
        pin_rpy = ROT_Z_TO_Y if i % 2 == 0 else (0.0, 0.0, 0.0)
        orange_gear.visual(
            Cylinder(radius=0.005, length=0.022),
            origin=Origin(xyz=(0.0, PLANET_ORBIT_RADIUS * ct, PLANET_ORBIT_RADIUS * st), rpy=pin_rpy),
            material=machined,
            name=f"spider_pin_{i}",
        )

    # Left side gear (drives the green half shaft / left wheel).
    left_side_gear = model.part("left_side_gear")
    left_side_gear.visual(
        mesh_from_geometry(
            _toothed_bevel_mesh(inner_radius=0.014, root_radius=0.035, tooth_height=0.008, width=0.020, teeth=16, cone_delta=0.004),
            "left_side_teeth",
        ),
        material=brass,
        name="left_side_teeth",
    )
    left_side_gear.visual(
        mesh_from_geometry(_annular_cylinder_mesh(inner_radius=0.012, outer_radius=0.022, width=0.030), "left_side_hub"),
        material=brass,
        name="left_side_hub",
    )

    # Right side gear (drives the red half shaft / right wheel).
    right_side_gear = model.part("right_side_gear")
    right_side_gear.visual(
        mesh_from_geometry(
            _toothed_bevel_mesh(inner_radius=0.014, root_radius=0.035, tooth_height=0.008, width=0.020, teeth=16, cone_delta=0.004),
            "right_side_teeth",
        ),
        material=brass,
        name="right_side_teeth",
    )
    right_side_gear.visual(
        mesh_from_geometry(_annular_cylinder_mesh(inner_radius=0.012, outer_radius=0.022, width=0.030), "right_side_hub"),
        material=brass,
        name="right_side_hub",
    )

    # Input pinion (drives the orange ring gear from the motor).
    input_pinion = model.part("input_pinion")
    input_pinion.visual(
        mesh_from_geometry(
            _toothed_bevel_mesh(inner_radius=0.006, root_radius=0.039, tooth_height=0.008, width=0.038, teeth=14, cone_delta=0.003),
            "input_pinion_teeth",
        ),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=yellow,
        name="input_pinion_teeth",
    )
    input_pinion.visual(Cylinder(radius=0.009, length=0.190), origin=Origin(xyz=(0.0, -0.058, 0.0), rpy=ROT_Z_TO_Y), material=yellow, name="input_shaft")
    input_pinion.visual(Cylinder(radius=0.017, length=0.018), origin=Origin(xyz=(0.0, -0.010, 0.0), rpy=ROT_Z_TO_Y), material=brass, name="pinion_collar")

    # Planet pinions: N=4 bevel gears on the spider cross, emitted with a loop.
    # Each planet meshes with both side gears and is carried by the orange
    # ring gear / carrier assembly.
    planet_visual_rpy = [
        (0.0, 0.0, math.pi / 2.0),     # i=0: radial axis along +Y
        (0.0, -math.pi / 2.0, 0.0),    # i=1: radial axis along +Z
        (0.0, 0.0, -math.pi / 2.0),    # i=2: radial axis along -Y
        (0.0, math.pi / 2.0, 0.0),     # i=3: radial axis along -Z
    ]
    planet_joint_axis = [
        (0.0, 1.0, 0.0),   # i=0
        (0.0, 0.0, 1.0),   # i=1
        (0.0, 1.0, 0.0),   # i=2
        (0.0, 0.0, 1.0),   # i=3
    ]

    for i in range(N_PLANET_PINIONS):
        theta = i * 2.0 * math.pi / N_PLANET_PINIONS
        ct, st = math.cos(theta), math.sin(theta)

        planet = model.part(f"planet_pinion_{i}")
        planet.visual(
            mesh_from_geometry(
                _toothed_bevel_mesh(
                    inner_radius=0.005,
                    root_radius=0.016,
                    tooth_height=0.005,
                    width=0.016,
                    teeth=10,
                    cone_delta=0.002,
                ),
                f"planet_teeth_mesh_{i}",
            ),
            origin=Origin(rpy=planet_visual_rpy[i]),
            material=brass,
            name="planet_teeth",
        )
        planet.visual(
            mesh_from_geometry(
                _annular_cylinder_mesh(inner_radius=0.005, outer_radius=0.009, width=0.020),
                f"planet_hub_mesh_{i}",
            ),
            origin=Origin(rpy=planet_visual_rpy[i]),
            material=brass,
            name="planet_hub",
        )

    # ──────────────────────────────────────────────────────────────────────
    # Articulations
    # ──────────────────────────────────────────────────────────────────────
    spin = MotionLimits(effort=12.0, velocity=20.0)

    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=carrier,
        child=left_wheel,
        origin=Origin(xyz=(-0.360, 0.0, 0.130)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=spin,
        mimic=Mimic(joint="left_side_gear_spin", multiplier=1.0),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=carrier,
        child=right_wheel,
        origin=Origin(xyz=(0.360, 0.0, 0.130)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=spin,
        mimic=Mimic(joint="right_side_gear_spin", multiplier=1.0),
    )
    model.articulation(
        "orange_gear_spin",
        ArticulationType.CONTINUOUS,
        parent=carrier,
        child=orange_gear,
        origin=Origin(xyz=(0.024, 0.0, 0.130)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=spin,
        mimic=Mimic(joint="input_pinion_spin", multiplier=-0.42),
    )
    model.articulation(
        "left_side_gear_spin",
        ArticulationType.CONTINUOUS,
        parent=carrier,
        child=left_side_gear,
        origin=Origin(xyz=(0.010, 0.0, 0.130)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=spin,
        mimic=Mimic(joint="orange_gear_spin", multiplier=1.0),
    )
    model.articulation(
        "right_side_gear_spin",
        ArticulationType.CONTINUOUS,
        parent=carrier,
        child=right_side_gear,
        origin=Origin(xyz=(0.038, 0.0, 0.130)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=spin,
        mimic=Mimic(joint="orange_gear_spin", multiplier=1.0),
    )

    # Planet pinion spins: each planet rotates on its spider-cross pin,
    # mimicking the orange gear spin to represent differential action.
    for i in range(N_PLANET_PINIONS):
        theta = i * 2.0 * math.pi / N_PLANET_PINIONS
        ct, st = math.cos(theta), math.sin(theta)
        model.articulation(
            f"planet_pinion_spin_{i}",
            ArticulationType.CONTINUOUS,
            parent=orange_gear,
            child=model.get_part(f"planet_pinion_{i}"),
            origin=Origin(xyz=(0.0, PLANET_ORBIT_RADIUS * ct, PLANET_ORBIT_RADIUS * st)),
            axis=planet_joint_axis[i],
            motion_limits=MotionLimits(effort=6.0, velocity=30.0),
            mimic=Mimic(joint="orange_gear_spin", multiplier=-1.0),
        )

    model.articulation(
        "input_pinion_spin",
        ArticulationType.CONTINUOUS,
        parent=carrier,
        child=input_pinion,
        origin=Origin(xyz=(0.0, -0.115, 0.222)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=25.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    # ── Intentional-overlap allowances ──────────────────────────────────

    # Motor shaft seated in the housing.
    ctx.allow_overlap(
        "carrier", "input_pinion",
        reason="The rotating input shaft and its collar pass through the motor housing, front cap, saddle, and cooling fins.",
    )

    # Ring-gear carrier hub encloses the side gears and planet pinions.
    ctx.allow_overlap(
        "orange_gear", "left_side_gear",
        reason="The left side gear sits inside the ring gear carrier hub bore.",
    )
    ctx.allow_overlap(
        "orange_gear", "right_side_gear",
        reason="The right side gear nests inside the ring gear carrier hub.",
    )
    for i in range(N_PLANET_PINIONS):
        ctx.allow_overlap(
            "orange_gear", f"planet_pinion_{i}",
            reason="The planet pinion is carried inside the ring gear carrier by the spider cross pin.",
        )

    # Side gear teeth intermesh with planet pinion teeth.
    for i in range(N_PLANET_PINIONS):
        ctx.allow_overlap(
            "left_side_gear", f"planet_pinion_{i}",
            reason="The left side gear teeth intermesh with the planet pinion bevel teeth.",
        )
        ctx.allow_overlap(
            "right_side_gear", f"planet_pinion_{i}",
            reason="The right side gear teeth intermesh with the planet pinion bevel teeth.",
        )

    # Half shafts pass through the side gear bores and the carrier hub.
    ctx.allow_overlap(
        "left_wheel", "left_side_gear",
        reason="The green half shaft and its collar pass through the left side gear splined bore.",
    )
    ctx.allow_overlap(
        "right_wheel", "right_side_gear",
        reason="The red half shaft passes through the right side gear splined bore.",
    )
    ctx.allow_overlap(
        "left_side_gear", "right_wheel",
        reason="The red half shaft passes through the left side gear bore to reach the right side gear.",
    )
    ctx.allow_overlap(
        "left_wheel", "orange_gear",
        reason="The green half shaft extends into the orange ring gear carrier hub bore.",
    )
    ctx.allow_overlap(
        "orange_gear", "right_wheel",
        reason="The red half shaft passes through the orange bevel/ring gear hub bore.",
    )
    ctx.allow_overlap(
        "left_wheel", "right_wheel",
        reason="The two half shafts and their collars meet inside the differential carrier where they connect to the side gears.",
    )

    # ── Continuous-joint existence and type checks ──────────────────────

    rotating_joint_names = (
        "left_wheel_spin",
        "right_wheel_spin",
        "orange_gear_spin",
        "left_side_gear_spin",
        "right_side_gear_spin",
        "input_pinion_spin",
    ) + tuple(f"planet_pinion_spin_{i}" for i in range(N_PLANET_PINIONS))

    for joint_name in rotating_joint_names:
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} is continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"{joint_name} type is {joint.articulation_type!r}",
        )

    # ── Fixed-center rotation checks ────────────────────────────────────

    input_drive_joint = object_model.get_articulation("input_pinion_spin")

    # Parts whose world center should be fixed when the input drives the
    # orange gear (they spin on their own axes, parented to the carrier).
    carrier_child_rotating_parts = (
        "orange_gear",
        "left_side_gear",
        "right_side_gear",
        "input_pinion",
    )
    for part_name in carrier_child_rotating_parts:
        part = object_model.get_part(part_name)
        rest = ctx.part_world_position(part)
        with ctx.pose({input_drive_joint: 1.37}):
            spun = ctx.part_world_position(part)
        fixed = (
            rest is not None
            and spun is not None
            and all(abs(a - b) < 1e-7 for a, b in zip(rest, spun))
        )
        ctx.check(
            f"{part_name} rotation center is fixed",
            fixed,
            details=f"rest={rest}, spun={spun}",
        )

    # Left wheel spins on its own axis (driven through the mimic chain from
    # input_pinion_spin → orange_gear_spin → left_side_gear_spin → left_wheel_spin).
    left_wheel_part = object_model.get_part("left_wheel")
    rest_lw = ctx.part_world_position(left_wheel_part)
    with ctx.pose({input_drive_joint: 1.37}):
        spun_lw = ctx.part_world_position(left_wheel_part)
    ctx.check(
        "left_wheel rotation center is fixed",
        rest_lw is not None and spun_lw is not None
        and all(abs(a - b) < 1e-7 for a, b in zip(rest_lw, spun_lw)),
        details=f"rest={rest_lw}, spun={spun_lw}",
    )

    # Planet pinions orbit with the orange gear but their own spin joint
    # keeps them centered on their spider-cross pin.  Verify the planet
    # spin joint is parented to the orange gear (spider cross carrier).
    for i in range(N_PLANET_PINIONS):
        joint = object_model.get_articulation(f"planet_pinion_spin_{i}")
        ctx.check(
            f"planet_pinion_{i} is carried by the orange gear spider cross",
            joint.parent == "orange_gear",
            details=f"planet_pinion_spin_{i} parent is {joint.parent!r}",
        )

    # ── Mimic-chain checks ──────────────────────────────────────────────

    orange_joint = object_model.get_articulation("orange_gear_spin")
    right_joint = object_model.get_articulation("right_wheel_spin")
    left_drive_joint = object_model.get_articulation("left_wheel_spin")
    left_side_joint = object_model.get_articulation("left_side_gear_spin")
    right_side_joint = object_model.get_articulation("right_side_gear_spin")

    ctx.check(
        "yellow input pinion drives orange gear in the opposite direction",
        orange_joint.mimic is not None
        and orange_joint.mimic.joint == "input_pinion_spin"
        and orange_joint.mimic.multiplier < 0.0,
        details=f"orange mimic={orange_joint.mimic!r}",
    )
    ctx.check(
        "left side gear follows the orange ring gear",
        left_side_joint.mimic is not None
        and left_side_joint.mimic.joint == "orange_gear_spin",
        details=f"left side mimic={left_side_joint.mimic!r}",
    )
    ctx.check(
        "right side gear follows the orange ring gear",
        right_side_joint.mimic is not None
        and right_side_joint.mimic.joint == "orange_gear_spin",
        details=f"right side mimic={right_side_joint.mimic!r}",
    )
    ctx.check(
        "left wheel follows the left side gear",
        left_drive_joint.mimic is not None
        and left_drive_joint.mimic.joint == "left_side_gear_spin",
        details=f"left wheel mimic={left_drive_joint.mimic!r}",
    )
    ctx.check(
        "right wheel follows the right side gear",
        right_joint.mimic is not None
        and right_joint.mimic.joint == "right_side_gear_spin",
        details=f"right wheel mimic={right_joint.mimic!r}",
    )

    # Each planet pinion mimics the orange gear spin.
    for i in range(N_PLANET_PINIONS):
        pjoint = object_model.get_articulation(f"planet_pinion_spin_{i}")
        ctx.check(
            f"planet_pinion_spin_{i} mimics orange_gear_spin",
            pjoint.mimic is not None
            and pjoint.mimic.joint == "orange_gear_spin",
            details=f"planet_pinion_spin_{i} mimic={pjoint.mimic!r}",
        )

    # ── Structural assertion for the spider cross (TARGET axis) ─────────

    orange_part = object_model.get_part("orange_gear")
    spider_arm_names = [f"spider_arm_{i}" for i in range(N_PLANET_PINIONS)]
    found_arms = [v for v in orange_part.visuals if v.name in spider_arm_names]
    ctx.check(
        f"spider cross carries exactly {N_PLANET_PINIONS} radial arms on the orange ring gear",
        len(found_arms) == N_PLANET_PINIONS,
        details=f"found {len(found_arms)} spider arms: {[v.name for v in found_arms]}",
    )

    planet_part_names = [f"planet_pinion_{i}" for i in range(N_PLANET_PINIONS)]
    found_planets = [p for p in object_model.parts if p.name in planet_part_names]
    ctx.check(
        f"exactly {N_PLANET_PINIONS} planet pinions on the spider cross",
        len(found_planets) == N_PLANET_PINIONS,
        details=f"found {len(found_planets)} planet pinion parts",
    )

    # ── Overlap / retention assertions ──────────────────────────────────

    ctx.expect_overlap("input_pinion", "orange_gear", axes="z", min_overlap=0.030, name="input pinion teeth share the orange gear mesh height")
    ctx.expect_overlap("left_side_gear", "orange_gear", axes="yz", min_overlap=0.050, name="left side gear sits inside orange ring envelope")
    ctx.expect_overlap("right_side_gear", "orange_gear", axes="yz", min_overlap=0.050, name="right side gear sits inside orange ring envelope")
    ctx.expect_overlap("left_side_gear", "left_wheel", axes="x", elem_a="left_side_hub", elem_b="green_half_shaft", min_overlap=0.020, name="left side gear is retained on green shaft")
    ctx.expect_overlap("right_side_gear", "right_wheel", axes="x", elem_a="right_side_hub", elem_b="red_half_shaft", min_overlap=0.020, name="right side gear is retained on red shaft")
    ctx.expect_overlap("orange_gear", "right_wheel", axes="x", elem_a="orange_hub", elem_b="red_half_shaft", min_overlap=0.020, name="orange gear is retained on red shaft")
    ctx.expect_overlap("input_pinion", "orange_gear", axes="y", elem_a="input_pinion_teeth", elem_b="orange_bevel_teeth", min_overlap=0.001, name="smaller yellow pinion reaches forward into the orange gear mesh")
    ctx.expect_overlap("carrier", "input_pinion", axes="y", elem_a="motor_can", elem_b="input_shaft", min_overlap=0.020, name="input shaft remains seated in motor housing")
    ctx.expect_overlap("carrier", "input_pinion", axes="y", elem_a="motor_front_cap", elem_b="input_shaft", min_overlap=0.010, name="input shaft passes through motor front cap")
    ctx.expect_overlap("left_wheel", "right_wheel", axes="x", min_overlap=0.010, name="half shafts meet inside the differential carrier")

    # Each planet pinion overlaps the orange ring gear envelope (retained on cross).
    for i in range(N_PLANET_PINIONS):
        ctx.expect_overlap(
            f"planet_pinion_{i}", "orange_gear",
            axes="yz", min_overlap=0.010,
            name=f"planet_pinion_{i} is retained inside the orange ring gear envelope",
        )

    return ctx.report()


object_model = build_object_model()
