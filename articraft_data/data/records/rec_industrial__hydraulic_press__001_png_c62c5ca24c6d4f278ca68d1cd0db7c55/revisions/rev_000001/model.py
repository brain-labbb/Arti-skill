from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _annular_z_sleeve(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    outer = cq.Workplane("XY").circle(outer_radius).extrude(length / 2.0, both=True)
    bore = cq.Workplane("XY").circle(inner_radius).extrude(length, both=True)
    return outer.cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_h_frame_hydraulic_press")

    gray = model.material("painted_warm_gray", rgba=(0.55, 0.57, 0.56, 1.0))
    dark_gray = model.material("dark_recess_gray", rgba=(0.20, 0.21, 0.20, 1.0))
    edge_gray = model.material("slightly_dark_seams", rgba=(0.38, 0.40, 0.39, 1.0))
    steel = model.material("brushed_steel", rgba=(0.78, 0.78, 0.74, 1.0))
    polished = model.material("polished_ram_rod", rgba=(0.86, 0.86, 0.82, 1.0))
    black = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    yellow = model.material("safety_yellow", rgba=(1.0, 0.72, 0.06, 1.0))
    red = model.material("red_pushbutton", rgba=(0.85, 0.06, 0.04, 1.0))
    green = model.material("green_pushbutton", rgba=(0.02, 0.55, 0.18, 1.0))
    amber = model.material("amber_indicator", rgba=(1.0, 0.55, 0.03, 1.0))
    white = model.material("white_label", rgba=(0.92, 0.92, 0.86, 1.0))

    frame = model.part("frame")

    def frame_box(name: str, size, xyz, material=gray, rpy=(0.0, 0.0, 0.0)) -> None:
        frame.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    def frame_cyl(name: str, radius: float, length: float, xyz, material, rpy=(0.0, 0.0, 0.0)) -> None:
        frame.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    # H-frame structure: two columns, massive top crosshead, low sill and real base feet.
    frame_box("left_column", (0.28, 0.50, 1.62), (-0.65, 0.0, 0.91))
    frame_box("right_column", (0.28, 0.50, 1.62), (0.65, 0.0, 0.91))
    frame_box("top_crosshead", (1.65, 0.60, 0.36), (0.0, 0.0, 1.90))
    frame_box("rear_top_cap", (1.38, 0.54, 0.08), (0.0, 0.03, 2.12))
    frame_box("lower_sill", (1.28, 0.56, 0.24), (0.0, 0.0, 0.26))

    for x, side in [(-0.65, "left"), (0.65, "right")]:
        frame_box(f"{side}_foot_pad", (0.48, 0.84, 0.12), (x, 0.0, 0.06))
        frame_box(f"{side}_front_flare", (0.16, 0.32, 0.45), (x, -0.30, 0.24), rpy=(math.radians(-10.0), 0.0, 0.0))
        frame_box(f"{side}_rear_flare", (0.16, 0.32, 0.45), (x, 0.30, 0.24), rpy=(math.radians(10.0), 0.0, 0.0))
        frame_box(f"{side}_inner_wear_plate", (0.035, 0.38, 1.28), (x - math.copysign(0.158, x), -0.002, 0.98), material=edge_gray)
    frame.visual(Box((0.23, 0.50, 0.06)), origin=Origin(xyz=(-0.435, 0.0, 0.545)), material=edge_gray, name="left_bed_ledge")
    frame.visual(Box((0.23, 0.50, 0.06)), origin=Origin(xyz=(0.435, 0.0, 0.545)), material=edge_gray, name="right_bed_ledge")

    # Bolted seams and warning labels make the gray fabricated sheet-metal frame read as industrial.
    frame_box("front_crosshead_seam", (1.46, 0.014, 0.025), (0.0, -0.306, 1.745), material=edge_gray)
    frame_box("top_logo_plate", (0.22, 0.018, 0.12), (0.0, -0.295, 1.985), material=yellow)
    frame_box("logo_dark_slot", (0.12, 0.020, 0.030), (0.0, -0.307, 1.985), material=dark_gray)
    for i, x in enumerate([-0.55, -0.28, 0.28, 0.55]):
        frame_cyl(f"crosshead_bolt_{i}", 0.024, 0.018, (x, -0.313, 1.78), dark_gray, rpy=(math.pi / 2, 0.0, 0.0))
    for i, x in enumerate([-0.78, -0.52, 0.52, 0.78]):
        frame_cyl(f"foot_bolt_{i}", 0.026, 0.016, (x, -0.28, 0.126), dark_gray)
    for i, x in enumerate([-0.65, 0.65]):
        frame_box(f"warning_label_{i}", (0.055, 0.008, 0.070), (x, -0.252, 1.05), material=yellow)
        frame_box(f"warning_mark_{i}", (0.020, 0.010, 0.035), (x, -0.258, 1.05), material=black)

    # Fixed hydraulic cylinder under the crosshead.
    frame_cyl("cylinder_flange", 0.275, 0.060, (0.0, 0.0, 1.720), steel)
    frame.visual(
        mesh_from_cadquery(_annular_z_sleeve(0.205, 0.100, 0.300), "fixed_cylinder_hollow_shell"),
        origin=Origin(xyz=(0.0, 0.0, 1.570)),
        material=steel,
        name="fixed_cylinder",
    )
    frame.visual(
        mesh_from_cadquery(_annular_z_sleeve(0.150, 0.098, 0.040), "hollow_gland_ring"),
        origin=Origin(xyz=(0.0, 0.0, 1.400)),
        material=dark_gray,
        name="gland_ring",
    )
    frame.visual(
        Cylinder(radius=0.030, length=0.070),
        origin=Origin(xyz=(0.880, 0.22, 1.660), rpy=(0.0, math.pi / 2, 0.0)),
        material=steel,
        name="upper_hose_port",
    )
    frame.visual(Box((0.070, 0.080, 0.032)), origin=Origin(xyz=(0.825, 0.22, 1.616)), material=steel, name="upper_port_bracket")
    frame.visual(
        Cylinder(radius=0.022, length=0.060),
        origin=Origin(xyz=(0.800, 0.10, 1.240), rpy=(0.0, math.pi / 2, 0.0)),
        material=steel,
        name="lower_hose_port",
    )

    # Adjustable bed pins on the front of the columns.
    for i, z in enumerate([0.53, 0.62, 0.71]):
        frame_cyl(f"left_bed_pin_{i}", 0.020, 0.032, (-0.505, -0.263, z), dark_gray, rpy=(math.pi / 2, 0.0, 0.0))
        frame_cyl(f"right_bed_pin_{i}", 0.020, 0.032, (0.505, -0.263, z), dark_gray, rpy=(math.pi / 2, 0.0, 0.0))

    bed = model.part("bed")
    bed.visual(Box((0.88, 0.48, 0.13)), origin=Origin(), material=edge_gray, name="bed_block")
    bed.visual(Box((0.88, 0.035, 0.075)), origin=Origin(xyz=(0.0, -0.247, 0.042)), material=yellow, name="hazard_yellow")
    for i, x in enumerate([-0.37, -0.25, -0.13, -0.01, 0.11, 0.23, 0.35]):
        bed.visual(
            Box((0.045, 0.042, 0.105)),
            origin=Origin(xyz=(x, -0.263, 0.043), rpy=(0.0, math.radians(23.0), 0.0)),
            material=black,
            name=f"hazard_black_{i}",
        )
    bed.visual(Box((0.76, 0.38, 0.018)), origin=Origin(xyz=(0.0, 0.0, 0.060)), material=steel, name="replaceable_bed_plate")
    bed.visual(Box((0.76, 0.018, 0.025)), origin=Origin(xyz=(0.0, -0.18, 0.074)), material=dark_gray, name="front_bed_seam")
    model.articulation("frame_to_bed", ArticulationType.FIXED, parent=frame, child=bed, origin=Origin(xyz=(0.0, 0.0, 0.640)))

    ram = model.part("ram")
    ram.visual(Cylinder(radius=0.096, length=0.620), origin=Origin(xyz=(0.0, 0.0, -0.230)), material=polished, name="ram_rod")
    ram.visual(Cylinder(radius=0.135, length=0.090), origin=Origin(xyz=(0.0, 0.0, -0.425)), material=steel, name="ram_collared_head")
    ram.visual(Cylinder(radius=0.155, length=0.070), origin=Origin(xyz=(0.0, 0.0, -0.505)), material=dark_gray, name="press_platen")
    ram.visual(Box((0.23, 0.23, 0.035)), origin=Origin(xyz=(0.0, 0.0, -0.555)), material=dark_gray, name="square_platen_face")
    model.articulation(
        "frame_to_ram",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=ram,
        origin=Origin(xyz=(0.0, 0.0, 1.520)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=180000.0, velocity=0.08, lower=0.0, upper=0.16),
    )

    pedestal = model.part("control_pedestal")
    pedestal.visual(Box((0.38, 0.34, 0.62)), origin=Origin(xyz=(0.0, 0.0, 0.31)), material=gray, name="cabinet_body")
    pedestal.visual(Box((0.38, 0.34, 0.18)), origin=Origin(xyz=(0.0, 0.0, 0.71)), material=gray, name="control_head")
    pedestal.visual(Box((0.34, 0.026, 0.135)), origin=Origin(xyz=(0.0, -0.185, 0.76), rpy=(math.radians(-8.0), 0.0, 0.0)), material=edge_gray, name="sloped_panel")
    pedestal.visual(Box((0.33, 0.012, 0.32)), origin=Origin(xyz=(0.0, -0.176, 0.44)), material=edge_gray, name="front_door_seam")
    pedestal.visual(Box((0.28, 0.010, 0.055)), origin=Origin(xyz=(0.0, -0.183, 0.45)), material=yellow, name="pedestal_warning_label")
    pedestal.visual(Box((0.050, 0.080, 0.110)), origin=Origin(xyz=(-0.170, 0.030, 0.845)), material=gray, name="upper_hose_mount")
    for i, x in enumerate([-0.13, 0.13]):
        pedestal.visual(Box((0.055, 0.40, 0.080)), origin=Origin(xyz=(x, 0.0, 0.04)), material=gray, name=f"pedestal_foot_{i}")

    pedestal.visual(Cylinder(radius=0.045, length=0.018), origin=Origin(xyz=(-0.125, -0.199, 0.805), rpy=(math.pi / 2, 0.0, 0.0)), material=dark_gray, name="gauge_bezel")
    pedestal.visual(Cylinder(radius=0.035, length=0.020), origin=Origin(xyz=(-0.125, -0.211, 0.805), rpy=(math.pi / 2, 0.0, 0.0)), material=white, name="gauge_face")
    pedestal.visual(Box((0.050, 0.006, 0.006)), origin=Origin(xyz=(-0.112, -0.224, 0.812), rpy=(0.0, 0.0, math.radians(-25.0))), material=black, name="gauge_needle")
    for i, (x, mat) in enumerate([(-0.025, green), (0.035, red), (0.095, amber)]):
        pedestal.visual(Sphere(radius=0.016), origin=Origin(xyz=(x, -0.197, 0.812)), material=mat, name=f"indicator_light_{i}")
    pedestal.visual(Cylinder(radius=0.025, length=0.040), origin=Origin(xyz=(-0.190, 0.030, 0.875), rpy=(0.0, math.pi / 2, 0.0)), material=steel, name="upper_hose_fitting")
    pedestal.visual(Cylinder(radius=0.020, length=0.035), origin=Origin(xyz=(-0.190, -0.020, 0.575), rpy=(0.0, math.pi / 2, 0.0)), material=steel, name="lower_hose_fitting")

    upper_hose = tube_from_spline_points(
        [
            (-0.480, 0.270, 1.660),
            (-0.260, 0.440, 1.600),
            (-0.145, 0.380, 1.180),
            (-0.190, 0.080, 0.900),
            (-0.190, 0.030, 0.875),
        ],
        radius=0.017,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
    )
    lower_hose = tube_from_spline_points(
        [
            (-0.560, 0.145, 1.240),
            (-0.390, 0.280, 1.120),
            (-0.285, 0.230, 0.790),
            (-0.190, -0.020, 0.575),
        ],
        radius=0.012,
        samples_per_segment=16,
        radial_segments=16,
        cap_ends=True,
    )
    pedestal.visual(mesh_from_geometry(upper_hose, "upper_hose"), origin=Origin(), material=black, name="upper_hose")
    pedestal.visual(mesh_from_geometry(lower_hose, "lower_hose"), origin=Origin(), material=black, name="lower_hose")
    model.articulation(
        "frame_to_pedestal",
        ArticulationType.FIXED,
        parent=frame,
        child=pedestal,
        origin=Origin(xyz=(1.36, -0.05, 0.0)),
    )

    button_specs = [
        ("green_button", (-0.030, -0.205, 0.755), green),
        ("red_button", (0.035, -0.205, 0.755), red),
        ("amber_button", (0.100, -0.205, 0.755), amber),
    ]
    for name, xyz, mat in button_specs:
        button = model.part(name)
        button.visual(
            Cylinder(radius=0.021, length=0.018),
            origin=Origin(xyz=(0.0, -0.009, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
            material=mat,
            name="button_cap",
        )
        button.visual(
            Cylinder(radius=0.014, length=0.010),
            origin=Origin(xyz=(0.0, 0.002, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
            material=dark_gray,
            name="button_stem",
        )
        model.articulation(
            f"pedestal_to_{name}",
            ArticulationType.PRISMATIC,
            parent=pedestal,
            child=button,
            origin=Origin(xyz=xyz),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=0.06, lower=0.0, upper=0.008),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    bed = object_model.get_part("bed")
    ram = object_model.get_part("ram")
    pedestal = object_model.get_part("control_pedestal")
    ram_joint = object_model.get_articulation("frame_to_ram")

    ctx.allow_overlap(
        frame,
        ram,
        elem_a="gland_ring",
        elem_b="ram_rod",
        reason="The polished ram rod is intentionally captured through the cylinder gland ring.",
    )
    ctx.allow_overlap(
        frame,
        ram,
        elem_a="fixed_cylinder",
        elem_b="ram_rod",
        reason="The hydraulic rod intentionally remains inserted inside the upper cylinder body.",
    )
    ctx.allow_overlap(
        pedestal,
        frame,
        elem_a="upper_hose",
        elem_b="upper_hose_port",
        reason="The flexible hydraulic hose is intentionally seated over the press-side hose port.",
    )
    ctx.allow_overlap(
        pedestal,
        frame,
        elem_a="lower_hose",
        elem_b="lower_hose_port",
        reason="The return hose is intentionally seated over the lower press-side port.",
    )
    for button_name in ("green_button", "red_button", "amber_button"):
        ctx.allow_overlap(
            button_name,
            pedestal,
            elem_a="button_stem",
            elem_b="sloped_panel",
            reason="The push-button stem is intentionally inserted through the control-panel face.",
        )

    frame_visuals = {v.name for v in frame.visuals}
    bed_visuals = {v.name for v in bed.visuals}
    pedestal_visuals = {v.name for v in pedestal.visuals}

    ctx.check(
        "H-frame has two columns and a massive crosshead",
        {"left_column", "right_column", "top_crosshead", "lower_sill"}.issubset(frame_visuals),
        details=f"frame visuals={sorted(frame_visuals)}",
    )
    ctx.check(
        "flared base feet are modeled",
        {"left_foot_pad", "right_foot_pad", "left_front_flare", "right_front_flare"}.issubset(frame_visuals),
        details=f"frame visuals={sorted(frame_visuals)}",
    )
    ctx.check(
        "fixed hydraulic cylinder is present above ram",
        {"fixed_cylinder", "cylinder_flange", "gland_ring"}.issubset(frame_visuals),
        details=f"frame visuals={sorted(frame_visuals)}",
    )
    ctx.check(
        "side warning labels are present",
        {"warning_label_0", "warning_label_1"}.issubset(frame_visuals),
        details=f"frame visuals={sorted(frame_visuals)}",
    )
    ctx.check(
        "press bed has hazard-striped platen face",
        "bed_block" in bed_visuals
        and "hazard_yellow" in bed_visuals
        and sum(1 for name in bed_visuals if name.startswith("hazard_black_")) >= 6,
        details=f"bed visuals={sorted(bed_visuals)}",
    )
    ctx.check(
        "control pedestal includes cabinet, gauge, lights, and hoses",
        {"cabinet_body", "control_head", "gauge_face", "gauge_needle", "upper_hose", "lower_hose"}.issubset(pedestal_visuals),
        details=f"pedestal visuals={sorted(pedestal_visuals)}",
    )
    ctx.check(
        "ram joint is non-fixed vertical prismatic",
        ram_joint.articulation_type == ArticulationType.PRISMATIC
        and abs(ram_joint.axis[2]) > 0.9
        and ram_joint.axis[2] < 0.0
        and ram_joint.motion_limits is not None
        and ram_joint.motion_limits.upper is not None
        and 0.12 <= ram_joint.motion_limits.upper <= 0.17,
        details=f"type={ram_joint.articulation_type}, axis={ram_joint.axis}, limits={ram_joint.motion_limits}",
    )

    ctx.expect_overlap(ram, bed, axes="xy", min_overlap=0.18, name="ram/platen is centered over bed")
    ctx.expect_gap(ram, bed, axis="z", min_gap=0.18, max_gap=0.28, name="raised ram clears the press bed")
    ctx.expect_gap(
        bed,
        frame,
        axis="z",
        positive_elem="bed_block",
        negative_elem="left_bed_ledge",
        max_gap=0.002,
        max_penetration=0.002,
        name="bed sits on frame ledge",
    )
    ctx.expect_origin_gap(pedestal, frame, axis="x", min_gap=1.20, max_gap=1.50, name="pedestal stands to the right of press")
    ctx.expect_within(
        ram,
        frame,
        axes="xy",
        inner_elem="ram_rod",
        outer_elem="gland_ring",
        margin=0.001,
        name="ram rod is centered in gland ring",
    )
    ctx.expect_overlap(
        ram,
        frame,
        axes="z",
        elem_a="ram_rod",
        elem_b="gland_ring",
        min_overlap=0.02,
        name="ram rod remains captured through gland",
    )
    ctx.expect_within(
        ram,
        frame,
        axes="xy",
        inner_elem="ram_rod",
        outer_elem="fixed_cylinder",
        margin=0.001,
        name="ram rod is centered inside cylinder body",
    )
    ctx.expect_overlap(
        ram,
        frame,
        axes="z",
        elem_a="ram_rod",
        elem_b="fixed_cylinder",
        min_overlap=0.08,
        name="ram rod retains insertion in cylinder body",
    )
    ctx.expect_overlap(
        pedestal,
        frame,
        axes="xyz",
        elem_a="upper_hose",
        elem_b="upper_hose_port",
        min_overlap=0.005,
        name="upper hose is seated on press port",
    )
    ctx.expect_overlap(
        pedestal,
        frame,
        axes="xyz",
        elem_a="lower_hose",
        elem_b="lower_hose_port",
        min_overlap=0.004,
        name="lower hose is seated on press port",
    )
    for button_name in ("green_button", "red_button", "amber_button"):
        ctx.expect_overlap(
            button_name,
            pedestal,
            axes="xz",
            elem_a="button_stem",
            elem_b="sloped_panel",
            min_overlap=0.020,
            name=f"{button_name} stem is retained in panel",
        )

    rest_pos = ctx.part_world_position(ram)
    with ctx.pose({ram_joint: 0.16}):
        lowered_pos = ctx.part_world_position(ram)
        ctx.expect_gap(ram, bed, axis="z", min_gap=0.05, max_gap=0.10, name="lowered ram approaches bed without passing through")
    ctx.check(
        "positive ram travel descends vertically",
        rest_pos is not None and lowered_pos is not None and lowered_pos[2] < rest_pos[2] - 0.13,
        details=f"rest={rest_pos}, lowered={lowered_pos}",
    )

    upper_hose_aabb = ctx.part_element_world_aabb(pedestal, elem="upper_hose")
    ctx.check(
        "upper hydraulic hose spans from press to pedestal",
        upper_hose_aabb is not None
        and upper_hose_aabb[0][0] < 0.95
        and upper_hose_aabb[1][0] > 1.15
        and upper_hose_aabb[1][2] > 1.40,
        details=f"upper_hose_aabb={upper_hose_aabb}",
    )

    button_joints = [
        object_model.get_articulation("pedestal_to_green_button"),
        object_model.get_articulation("pedestal_to_red_button"),
        object_model.get_articulation("pedestal_to_amber_button"),
    ]
    ctx.check(
        "separate push buttons are articulated controls",
        all(j.articulation_type == ArticulationType.PRISMATIC and j.motion_limits and j.motion_limits.upper for j in button_joints),
        details=f"button joints={[j.name for j in button_joints]}",
    )

    return ctx.report()


object_model = build_object_model()
