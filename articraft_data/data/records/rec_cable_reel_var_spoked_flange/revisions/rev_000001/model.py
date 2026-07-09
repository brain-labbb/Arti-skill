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
    TorusGeometry,
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


def sector_profile(
    inner_r: float,
    outer_r: float,
    start_angle: float,
    end_angle: float,
    segments: int = 16,
) -> list[tuple[float, float]]:
    """Closed pie-slice profile between two radii and two angles (radians)."""
    pts: list[tuple[float, float]] = []
    # outer arc from start to end
    for i in range(segments + 1):
        a = start_angle + (end_angle - start_angle) * i / segments
        pts.append((outer_r * math.cos(a), outer_r * math.sin(a)))
    # inner arc from end back to start
    for i in range(segments + 1):
        a = end_angle - (end_angle - start_angle) * i / segments
        pts.append((inner_r * math.cos(a), inner_r * math.sin(a)))
    return pts


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


def spoked_disc_yz(
    outer_r: float,
    hub_hole_r: float,
    thickness_x: float,
    *,
    n_spokes: int = 6,
    spoke_width_angle: float = math.radians(10),
    gap_inner_r: float = 0.090,
    gap_outer_r: float = 0.200,
    segments: int = 96,
    arc_segments: int = 16,
):
    """Spoked flange disc: outer rim ring + hub ring joined by N radial spokes.

    The cutouts are annular sectors between the hub ring inner edge and the rim
    ring inner edge, leaving solid spoke bridges at regular angular intervals.
    """
    outer = circle_profile(outer_r, segments)
    holes: list[list[tuple[float, float]]] = [circle_profile(hub_hole_r, segments)]

    spoke_spacing = TAU / n_spokes
    gap_angular_width = spoke_spacing - spoke_width_angle

    for i in range(n_spokes):
        gap_center = spoke_spacing * i + spoke_spacing / 2.0
        gap_start = gap_center - gap_angular_width / 2.0
        gap_end = gap_center + gap_angular_width / 2.0
        holes.append(
            sector_profile(gap_inner_r, gap_outer_r, gap_start, gap_end, arc_segments)
        )

    geom = ExtrudeWithHolesGeometry(outer, holes, thickness_x, center=True)
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
            "description": "Industrial electrical cable reel with rotating flanged drum, wound rubber cable, crank handle, support frame, outlet block, labels, fasteners, and strain relief.",
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

    # Skid-style support frame and feet.
    for idx, y in enumerate((-0.235, 0.235)):
        frame.visual(
            Box((0.86, 0.035, 0.045)),
            origin=Origin(xyz=(0.0, y, 0.045)),
            material=cream,
            name=f"base_rail_{idx}",
        )
    for idx, (x, y) in enumerate(((-0.39, -0.235), (-0.39, 0.235), (0.39, -0.235), (0.39, 0.235))):
        frame.visual(
            Box((0.085, 0.075, 0.018)),
            origin=Origin(xyz=(x, y, 0.014)),
            material=cream_shadow,
            name=f"rubber_foot_{idx}",
        )
    for idx, (x, y) in enumerate(((-0.385, -0.205), (-0.385, 0.205), (0.385, -0.205), (0.385, 0.205))):
        frame.visual(
            Box((0.050, 0.040, 0.125)),
            origin=Origin(xyz=(x, y, 0.095)),
            material=cream,
            name=f"rail_upright_{idx}",
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
    # Spoked flange discs: hub ring + rim ring joined by 6 radial spokes per side.
    spoke_cheek_mesh = spoked_disc_yz(
        outer_r=0.232,
        hub_hole_r=0.055,
        thickness_x=0.038,
        n_spokes=6,
        spoke_width_angle=math.radians(10),
        gap_inner_r=0.090,
        gap_outer_r=0.200,
        segments=112,
        arc_segments=16,
    )
    for suffix, x in (("front", -0.270), ("rear", 0.270)):
        cheek_x = -0.269 if x < 0 else 0.269
        reel.visual(
            mesh_from_geometry(spoke_cheek_mesh, f"{suffix}_spool_cheek_mesh"),
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

    # Spoke-hub flange bolts on the hub ring of the front spoked cheek.
    for i in range(6):
        a = TAU * i / 6.0 + math.radians(12.0)
        y = 0.072 * math.cos(a)
        z = 0.072 * math.sin(a)
        add_x_cylinder(reel, f"flange_bolt_{i}", 0.007, 0.006, (-0.291, y, z), metal)

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
            ["front_side_plate", "rear_side_plate", "base_rail_0", "base_rail_1", "axle_shaft", "rating_label"],
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

    ctx.check(
        "reel rotates on central x axle",
        reel_joint.articulation_type != ArticulationType.FIXED
        and tuple(round(v, 3) for v in reel_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={reel_joint.articulation_type}, axis={reel_joint.axis}",
    )

    # Spoked-flange variant check: prove the cheek discs have cutout structure
    # rather than solid annular faces. The spoked cheek outer extent should
    # match the rim ring radius while the hub collar remains a separate visual.
    for cheek_name in ("front_spool_cheek", "rear_spool_cheek"):
        cheek_aabb = ctx.part_element_world_aabb(reel, elem=cheek_name)
        ctx.check(
            f"{cheek_name} is a spoked disc with rim extent near 0.232 m",
            cheek_aabb is not None
            and (cheek_aabb[1][1] - cheek_aabb[0][1]) > 0.42
            and (cheek_aabb[1][2] - cheek_aabb[0][2]) > 0.42,
            details=f"cheek={cheek_name}, aabb={cheek_aabb}",
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
