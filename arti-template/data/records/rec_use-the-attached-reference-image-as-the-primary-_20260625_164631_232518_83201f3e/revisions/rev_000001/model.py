from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _annular_cylinder_x(
    outer_radius: float,
    inner_radius: float,
    length: float,
    *,
    segments: int = 72,
) -> MeshGeometry:
    """Hollow cylinder centered on the local X axis."""
    geom = MeshGeometry()
    x0 = -length / 2.0
    x1 = length / 2.0
    outer0: list[int] = []
    outer1: list[int] = []
    inner0: list[int] = []
    inner1: list[int] = []

    for i in range(segments):
        a = 2.0 * math.pi * i / segments
        ca = math.cos(a)
        sa = math.sin(a)
        outer0.append(geom.add_vertex(x0, outer_radius * ca, outer_radius * sa))
        outer1.append(geom.add_vertex(x1, outer_radius * ca, outer_radius * sa))
        inner0.append(geom.add_vertex(x0, inner_radius * ca, inner_radius * sa))
        inner1.append(geom.add_vertex(x1, inner_radius * ca, inner_radius * sa))

    for i in range(segments):
        j = (i + 1) % segments
        # Outer wall.
        geom.add_face(outer0[i], outer0[j], outer1[j])
        geom.add_face(outer0[i], outer1[j], outer1[i])
        # Inner bore wall, reversed normals.
        geom.add_face(inner0[j], inner0[i], inner1[i])
        geom.add_face(inner0[j], inner1[i], inner1[j])
        # End annuli.
        geom.add_face(outer0[j], outer0[i], inner0[i])
        geom.add_face(outer0[j], inner0[i], inner0[j])
        geom.add_face(outer1[i], outer1[j], inner1[j])
        geom.add_face(outer1[i], inner1[j], inner1[i])

    return geom


def _side_plate(side_x: float) -> cq.Workplane:
    """Photo-like orange side cheek, authored in YZ and rotated into X."""
    thickness = 0.020
    outer = [
        (-0.310, 0.030),
        (0.310, 0.030),
        (0.292, 0.115),
        (0.248, 0.420),
        (0.162, 0.510),
        (-0.162, 0.510),
        (-0.248, 0.420),
        (-0.292, 0.115),
    ]

    plate = cq.Workplane("XY").polyline(outer).close().extrude(thickness).translate(
        (0.0, 0.0, -thickness / 2.0)
    )

    def cut_poly(points: list[tuple[float, float]]) -> None:
        nonlocal plate
        cutter = (
            cq.Workplane("XY")
            .polyline(points)
            .close()
            .extrude(thickness * 4.0)
            .translate((0.0, 0.0, -thickness * 2.0))
        )
        plate = plate.cut(cutter)

    # Large photo-like lightening windows.  These leave a real molded perimeter,
    # diagonal spokes into the hub area, and a broad lower foot rail.
    cut_poly([(-0.242, 0.122), (-0.184, 0.386), (-0.046, 0.314)])
    cut_poly([(0.046, 0.314), (0.184, 0.386), (0.242, 0.122)])
    cut_poly([(-0.190, 0.078), (-0.036, 0.248), (-0.004, 0.082)])
    cut_poly([(0.004, 0.082), (0.036, 0.248), (0.190, 0.078)])

    top_slot = (
        cq.Workplane("XY")
        .center(0.000, 0.462)
        .slot2D(0.235, 0.050)
        .extrude(thickness * 4.0)
        .translate((0.0, 0.0, -thickness * 2.0))
    )
    axle_hole = (
        cq.Workplane("XY")
        .center(0.000, 0.310)
        .circle(0.034)
        .extrude(thickness * 4.0)
        .translate((0.0, 0.0, -thickness * 2.0))
    )
    plate = plate.cut(top_slot).cut(axle_hole)

    # Map local (x, y, z) -> world (z, x, y), then move to either side.
    return plate.rotate((0, 0, 0), (1, 1, 1), 120).translate((side_x, 0.0, 0.0))


def _top_handle_shell(side_x: float) -> cq.Workplane:
    thickness = 0.022
    shell = (
        cq.Workplane("XY")
        .center(0.0, 0.472)
        .slot2D(0.275, 0.078)
        .extrude(thickness)
        .translate((0.0, 0.0, -thickness / 2.0))
    )
    cutter = (
        cq.Workplane("XY")
        .center(0.0, 0.472)
        .slot2D(0.205, 0.038)
        .extrude(thickness * 4.0)
        .translate((0.0, 0.0, -thickness * 2.0))
    )
    return shell.cut(cutter).rotate((0, 0, 0), (1, 1, 1), 120).translate((side_x, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="electrical_cable_reel",
        meta={"small_class": "Cable reel", "category": "Electrical_Wiring"},
    )

    orange = model.material("safety_orange_plastic", rgba=(1.0, 0.27, 0.02, 1.0))
    black = model.material("black_molded_plastic", rgba=(0.01, 0.01, 0.012, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.005, 0.006, 0.008, 1.0))
    grey = model.material("galvanized_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    dark_metal = model.material("dark_screw_heads", rgba=(0.02, 0.022, 0.024, 1.0))
    brass = model.material("brass_terminals", rgba=(0.80, 0.55, 0.18, 1.0))
    blue = model.material("blue_rubber_cable", rgba=(0.02, 0.08, 0.15, 1.0))
    label_white = model.material("white_label_ink", rgba=(0.92, 0.96, 1.0, 1.0))
    label_blue = model.material("blue_rating_label", rgba=(0.08, 0.30, 0.62, 1.0))
    warning_yellow = model.material("yellow_warning_label", rgba=(1.0, 0.82, 0.05, 1.0))

    frame = model.part("frame")

    # Molded side frames matching the photo: one continuous orange cheek per side
    # with triangular lightening openings and an integrated top carry slot.
    for side_i, side_x in enumerate((-0.275, 0.275)):
        frame.visual(
            mesh_from_cadquery(_side_plate(side_x), f"reference_side_frame_{side_i}"),
            material=orange,
            name=f"reference_side_frame_{side_i}",
        )
        for boss_i, (y, z, radius) in enumerate(
            [
                (-0.250, 0.090, 0.017),
                (-0.276, 0.430, 0.018),
                (0.276, 0.430, 0.018),
                (0.250, 0.090, 0.017),
                (0.000, 0.310, 0.026),
            ]
        ):
            frame.visual(
                Cylinder(radius=radius, length=0.034),
                origin=Origin(xyz=(side_x, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=orange if boss_i < 4 else black,
                name=f"side_boss_{side_i}_{boss_i}",
            )
            frame.visual(
                Cylinder(radius=0.0065 if boss_i < 4 else 0.010, length=0.040),
                origin=Origin(xyz=(side_x + (0.003 if side_x > 0 else -0.003), y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=grey,
                name=f"side_rivet_{side_i}_{boss_i}",
            )

    # Galvanized tie rods and fixed central axle.
    for idx, (y, z, r) in enumerate(
        [
            (-0.276, 0.430, 0.011),
            (0.276, 0.430, 0.011),
            (-0.250, 0.090, 0.009),
            (0.250, 0.090, 0.009),
        ]
    ):
        frame.visual(
            Cylinder(radius=r, length=0.590),
            origin=Origin(xyz=(0.0, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=grey,
            name=f"tie_rod_{idx}",
        )
        for cap_i, x in enumerate((-0.300, 0.300)):
            frame.visual(
                Cylinder(radius=r * 1.45, length=0.020),
                origin=Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=grey,
                name=f"tie_rod_{idx}_end_cap_{cap_i}",
            )
    frame.visual(
        Cylinder(radius=0.017, length=0.640),
        origin=Origin(xyz=(0.0, 0.0, 0.310), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grey,
        name="fixed_axle",
    )
    for i, x in enumerate((-0.275, 0.275)):
        frame.visual(
            Cylinder(radius=0.022, length=0.036),
            origin=Origin(xyz=(x, 0.0, 0.310), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=grey,
            name=f"axle_bearing_{i}",
        )

    # Front base rail, feet, and label face.
    frame.visual(
        Box((0.575, 0.036, 0.070)),
        origin=Origin(xyz=(0.0, -0.315, 0.105)),
        material=black,
        name="front_base_rail",
    )
    frame.visual(
        Box((0.575, 0.030, 0.046)),
        origin=Origin(xyz=(0.0, 0.260, 0.070)),
        material=orange,
        name="rear_foot_rail",
    )
    for x in (-0.225, 0.225):
        frame.visual(
            Box((0.110, 0.075, 0.020)),
            origin=Origin(xyz=(x, -0.305, 0.020)),
            material=orange,
            name=f"front_foot_{0 if x < 0 else 1}",
        )

    frame.visual(
        Box((0.115, 0.003, 0.032)),
        origin=Origin(xyz=(-0.090, -0.334, 0.113)),
        material=label_white,
        name="brand_label",
    )
    frame.visual(
        Box((0.155, 0.003, 0.018)),
        origin=Origin(xyz=(0.125, -0.334, 0.130)),
        material=label_blue,
        name="rating_label",
    )
    frame.visual(
        Box((0.055, 0.003, 0.018)),
        origin=Origin(xyz=(0.250, -0.334, 0.082)),
        material=warning_yellow,
        name="warning_label",
    )

    # Raised logo-like strokes on the front label without relying on font assets.
    for i, (x, z, w) in enumerate(
        [
            (-0.132, 0.108, 0.010),
            (-0.115, 0.120, 0.020),
            (-0.090, 0.108, 0.012),
            (-0.066, 0.121, 0.026),
        ]
    ):
        frame.visual(
            Box((w, 0.002, 0.006)),
            origin=Origin(xyz=(x, -0.336, z)),
            material=label_blue,
            name=f"brand_stroke_{i}",
        )

    # Electrical outlet block and cable gland/strain relief on the front rail.
    frame.visual(
        Box((0.165, 0.050, 0.070)),
        origin=Origin(xyz=(0.135, -0.306, 0.185)),
        material=black,
        name="outlet_block",
    )
    frame.visual(
        Box((0.130, 0.006, 0.052)),
        origin=Origin(xyz=(0.135, -0.334, 0.185)),
        material=label_white,
        name="outlet_faceplate",
    )
    for i, x in enumerate((0.100, 0.170)):
        frame.visual(
            Cylinder(radius=0.014, length=0.007),
            origin=Origin(xyz=(x, -0.339, 0.192), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_rubber,
            name=f"socket_recess_{i}",
        )
        frame.visual(
            Cylinder(radius=0.0045, length=0.005),
            origin=Origin(xyz=(x, -0.3395, 0.168), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brass,
            name=f"terminal_screw_{i}",
        )

    frame.visual(
        Cylinder(radius=0.018, length=0.044),
        origin=Origin(xyz=(0.215, -0.332, 0.132), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grey,
        name="strain_relief_collar",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.050),
        origin=Origin(xyz=(0.215, -0.360, 0.132), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_rubber,
        name="rubber_gland",
    )

    # Flexible output lead and plug.  It is a continuous tube starting at the gland.
    cable_geom = tube_from_spline_points(
        [
            (0.215, -0.383, 0.132),
            (0.260, -0.470, 0.085),
            (0.170, -0.575, 0.045),
            (0.000, -0.600, 0.050),
            (-0.120, -0.545, 0.062),
        ],
        radius=0.008,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )
    frame.visual(mesh_from_geometry(cable_geom, "output_lead"), material=blue, name="output_lead")
    frame.visual(
        Cylinder(radius=0.014, length=0.055),
        origin=Origin(xyz=(-0.145, -0.532, 0.064), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_rubber,
        name="plug_boot",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.035),
        origin=Origin(xyz=(-0.190, -0.532, 0.064), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grey,
        name="metal_plug_shell",
    )

    # Screw heads on side cheeks and base rail.
    screw_points = [
        (-0.250, 0.105),
        (-0.218, 0.420),
        (0.220, 0.405),
        (0.225, 0.085),
        (0.000, 0.310),
    ]
    screw_idx = 0
    for sx, x_face in [(-1.0, -0.286), (1.0, 0.286)]:
        for y, z in screw_points:
            frame.visual(
                Cylinder(radius=0.006, length=0.006),
                origin=Origin(xyz=(x_face, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=dark_metal,
                name=f"side_screw_{screw_idx}",
            )
            screw_idx += 1
    for i, x in enumerate((-0.250, -0.010, 0.260)):
        frame.visual(
            Cylinder(radius=0.0055, length=0.004),
            origin=Origin(xyz=(x, -0.3345, 0.075), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"base_screw_{i}",
        )

    reel = model.part("reel")
    cheek_mesh = mesh_from_geometry(_annular_cylinder_x(0.248, 0.052, 0.026), "black_spool_cheek")
    drum_mesh = mesh_from_geometry(_annular_cylinder_x(0.170, 0.055, 0.344), "hollow_drum_core")
    hub_mesh = mesh_from_geometry(_annular_cylinder_x(0.068, 0.017, 0.430), "visible_hub_sleeve")

    reel.visual(cheek_mesh, origin=Origin(xyz=(-0.187, 0.0, 0.0)), material=black, name="spool_cheek_0")
    reel.visual(cheek_mesh, origin=Origin(xyz=(0.187, 0.0, 0.0)), material=black, name="spool_cheek_1")
    reel.visual(drum_mesh, material=black, name="hollow_drum_core")
    reel.visual(hub_mesh, material=black, name="visible_hub")

    # Visible wound cable: individual rubber loops rather than a single smooth cylinder.
    loop_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.178, tube=0.0085, radial_segments=14, tubular_segments=76),
        "wound_cable_loop",
    )
    for i in range(31):
        x = -0.150 + i * 0.010
        reel.visual(
            loop_mesh,
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=orange,
            name=f"wound_loop_{i:02d}",
        )

    # Ribbed hub cap and small warning mark on the rotating face.
    reel.visual(
        mesh_from_geometry(_annular_cylinder_x(0.050, 0.026, 0.018), "hub_cap_ring"),
        origin=Origin(xyz=(-0.205, 0.0, 0.0)),
        material=black,
        name="hub_cap",
    )
    reel.visual(
        Box((0.004, 0.020, 0.006)),
        origin=Origin(xyz=(-0.215, 0.025, 0.020)),
        material=warning_yellow,
        name="hub_marker",
    )
    for i in range(18):
        a = 2 * math.pi * i / 18
        reel.visual(
            Box((0.006, 0.010, 0.125)),
            origin=Origin(
                xyz=(-0.209, 0.125 * math.cos(a), 0.125 * math.sin(a)),
                rpy=(a - math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_metal,
            name=f"hub_rib_{i:02d}",
        )

    # Offset winding crank in the gap between reel cheek and frame side.
    reel.visual(
        Box((0.013, 0.095, 0.018)),
        origin=Origin(xyz=(0.207, -0.130, 0.0)),
        material=orange,
        name="crank_arm",
    )
    reel.visual(
        Cylinder(radius=0.011, length=0.042),
        origin=Origin(xyz=(0.218, -0.175, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grey,
        name="crank_peg",
    )
    reel.visual(
        Cylinder(radius=0.018, length=0.042),
        origin=Origin(xyz=(0.239, -0.175, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_rubber,
        name="crank_knob",
    )

    # Captive cable end emerging from the rotating drum before winding into the layers.
    reel_tail = tube_from_spline_points(
        [
            (-0.145, -0.108, -0.145),
            (-0.070, -0.142, -0.173),
            (0.055, -0.132, -0.170),
            (0.145, -0.095, -0.145),
        ],
        radius=0.0068,
        samples_per_segment=10,
        radial_segments=14,
        cap_ends=True,
    )
    reel.visual(mesh_from_geometry(reel_tail, "rotating_cable_tail"), material=orange, name="rotating_cable_tail")

    model.articulation(
        "frame_to_reel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=reel,
        origin=Origin(xyz=(0.0, 0.0, 0.310)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=6.0),
        meta={"mechanism": "spool rotates freely on the fixed axle"},
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    lo, hi = aabb
    return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    reel = object_model.get_part("reel")
    joint = object_model.get_articulation("frame_to_reel")

    ctx.allow_overlap(
        frame,
        reel,
        elem_a="fixed_axle",
        elem_b="visible_hub",
        reason=(
            "The static steel axle is intentionally captured inside the rotating "
            "hub sleeve; the mesh sleeve is a visual bearing proxy around the shaft."
        ),
    )

    ctx.check(
        "small class is cable reel",
        object_model.name == "electrical_cable_reel"
        and object_model.meta.get("small_class") == "Cable reel",
        details=f"name={object_model.name}, meta={object_model.meta}",
    )
    ctx.check(
        "primary rotating reel joint exists",
        joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={joint.articulation_type}, axis={joint.axis}",
    )
    ctx.check(
        "two black spool cheeks and many wound cable loops",
        all(reel.get_visual(n) is not None for n in ("spool_cheek_0", "spool_cheek_1"))
        and sum(1 for v in reel.visuals if v.name.startswith("wound_loop_")) >= 28,
        details="expected paired flanges and individual wound cable loops",
    )
    ctx.check(
        "photo-like open orange side frames present",
        all(
            frame.get_visual(n) is not None
            for n in (
                "reference_side_frame_0",
                "reference_side_frame_1",
                "side_boss_0_1",
                "side_boss_1_1",
                "tie_rod_0",
                "tie_rod_1",
                "front_base_rail",
            )
        ),
        details="reference has continuous molded cheek frames, triangular cutouts, supported silver rods, and a black front rail",
    )
    ctx.check(
        "electrical outlet details present",
        all(
            frame.get_visual(n) is not None
            for n in (
                "outlet_block",
                "outlet_faceplate",
                "socket_recess_0",
                "socket_recess_1",
                "terminal_screw_0",
                "terminal_screw_1",
                "strain_relief_collar",
                "output_lead",
                "metal_plug_shell",
            )
        ),
        details="outlet block, sockets, terminals, strain relief, lead, and plug are required",
    )
    ctx.check(
        "labels and fasteners are modeled geometry",
        all(
            frame.get_visual(n) is not None
            for n in ("brand_label", "rating_label", "warning_label", "side_screw_0", "base_screw_0")
        ),
        details="labels and screw heads should be separate raised/attached visuals",
    )

    ctx.expect_within(
        reel,
        frame,
        axes="x",
        inner_elem="visible_hub",
        outer_elem="fixed_axle",
        margin=0.03,
        name="hub sleeve is centered on the fixed axle",
    )
    ctx.expect_overlap(
        reel,
        frame,
        axes="x",
        elem_a="visible_hub",
        elem_b="fixed_axle",
        min_overlap=0.38,
        name="fixed axle passes through the rotating hub",
    )
    ctx.expect_gap(
        reel,
        frame,
        axis="z",
        positive_elem="hollow_drum_core",
        negative_elem="rear_foot_rail",
        min_gap=0.03,
        name="rotating drum clears the lower frame rail",
    )

    with ctx.pose({joint: 0.0}):
        a0 = ctx.part_element_world_aabb(reel, elem="crank_knob")
        c0 = _aabb_center(a0)
    with ctx.pose({joint: 1.25}):
        a1 = ctx.part_element_world_aabb(reel, elem="crank_knob")
        c1 = _aabb_center(a1)
    ctx.check(
        "crank knob visibly rotates with the reel",
        c0 is not None
        and c1 is not None
        and abs(c1[1] - c0[1]) > 0.04
        and abs(c1[2] - c0[2]) > 0.08,
        details=f"closed_center={c0}, rotated_center={c1}",
    )

    return ctx.report()


object_model = build_object_model()
