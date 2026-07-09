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


# ── Wall-bracket variant: geometry constants ────────────────────────────
WALL_FRONT_Y = 0.310          # front face of the wall plate (user side)
WALL_PLATE_THICKNESS = 0.012  # plate thickness in Y
AXLE_Z = 0.310                # axle centerline height


def _a_bracket(
    side_x: float,
    wall_front_y: float = WALL_FRONT_Y,
    thickness: float = 0.010,
) -> cq.Workplane:
    """Trapezoidal A-bracket plate for a wall-mounted reel.

    The bracket is thin in X, with a trapezoidal profile in YZ:
    wide at the wall attachment, narrow at the forward arm tip that
    carries the axle bearing.
    """
    pts = [
        (wall_front_y, AXLE_Z + 0.150),   # upper wall attachment
        (0.000, AXLE_Z + 0.030),          # upper arm tip
        (0.000, AXLE_Z - 0.030),          # lower arm tip
        (wall_front_y, AXLE_Z - 0.170),   # lower wall attachment
    ]
    bracket = (
        cq.Workplane("YZ")
        .polyline(pts)
        .close()
        .extrude(thickness)
        .translate((side_x - thickness / 2.0, 0.0, 0.0))
    )
    return bracket


def _bracket_stiffener(
    side_x: float,
    wall_front_y: float = WALL_FRONT_Y,
    thickness: float = 0.008,
) -> cq.Workplane:
    """Small horizontal stiffener rib welded to the bracket arm."""
    arm_length = wall_front_y - 0.020
    stiffener = (
        cq.Workplane("XZ")
        .center(side_x, AXLE_Z)
        .rect(thickness, 0.058)
        .extrude(arm_length)
        .translate((0.0, 0.020, 0.0))
    )
    return stiffener


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

    # ── FRAME: wall-bracket mount (variant axis) ─────────────────────────
    frame = model.part("frame")

    # Flat galvanized wall mounting plate.
    wall_plate_width = 0.580
    wall_plate_height = 0.460
    wall_plate_center_z = 0.300
    wall_plate_center_y = WALL_FRONT_Y + WALL_PLATE_THICKNESS / 2.0

    frame.visual(
        Box((wall_plate_width, WALL_PLATE_THICKNESS, wall_plate_height)),
        origin=Origin(xyz=(0.0, wall_plate_center_y, wall_plate_center_z)),
        material=grey,
        name="wall_mounting_plate",
    )

    # Six wall-anchor bolt heads on the front face of the plate.
    bolt_x_positions = (-0.240, 0.0, 0.240)
    bolt_z_positions = (
        wall_plate_center_z + wall_plate_height / 2.0 - 0.040,
        wall_plate_center_z - wall_plate_height / 2.0 + 0.040,
    )
    bolt_idx = 0
    for x in bolt_x_positions:
        for z in bolt_z_positions:
            frame.visual(
                Cylinder(radius=0.008, length=0.008),
                origin=Origin(
                    xyz=(x, WALL_FRONT_Y - 0.001, z),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=dark_metal,
                name=f"wall_bolt_{bolt_idx}",
            )
            bolt_idx += 1

    # Two trapezoidal A-brackets extending from the plate forward.
    bracket_side_xs = (-0.265, 0.265)
    for side_i, side_x in enumerate(bracket_side_xs):
        bracket = _a_bracket(side_x, WALL_FRONT_Y)
        frame.visual(
            mesh_from_cadquery(bracket, f"a_bracket_{side_i}"),
            material=grey,
            name=f"a_bracket_{side_i}",
        )

        # Horizontal stiffener rib on each bracket arm.
        stiffener = _bracket_stiffener(side_x, WALL_FRONT_Y)
        frame.visual(
            mesh_from_cadquery(stiffener, f"bracket_stiffener_{side_i}"),
            material=grey,
            name=f"bracket_stiffener_{side_i}",
        )

        # Bearing saddle block at the arm tip cradling the axle bearing.
        frame.visual(
            Box((0.034, 0.044, 0.065)),
            origin=Origin(xyz=(side_x, 0.022, AXLE_Z)),
            material=grey,
            name=f"bearing_saddle_{side_i}",
        )

        # Bracket-to-plate bolts (upper and lower wall attachment points).
        for bolt_i, z in enumerate((AXLE_Z + 0.130, AXLE_Z - 0.150)):
            frame.visual(
                Cylinder(radius=0.007, length=0.008),
                origin=Origin(
                    xyz=(side_x, WALL_FRONT_Y - 0.001, z),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=dark_metal,
                name=f"bracket_bolt_{side_i}_{bolt_i}",
            )

    # Fixed steel axle passing through both bearings.
    frame.visual(
        Cylinder(radius=0.017, length=0.640),
        origin=Origin(xyz=(0.0, 0.0, AXLE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grey,
        name="fixed_axle",
    )
    for i, x in enumerate((-0.275, 0.275)):
        frame.visual(
            Cylinder(radius=0.022, length=0.036),
            origin=Origin(xyz=(x, 0.0, AXLE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=grey,
            name=f"axle_bearing_{i}",
        )

    # ── Outlet block mounted on the wall plate below the reel ────────────
    outlet_center_y = WALL_FRONT_Y - 0.025  # back face flush with plate front
    outlet_z = 0.088

    frame.visual(
        Box((0.165, 0.050, 0.070)),
        origin=Origin(xyz=(0.135, outlet_center_y, outlet_z)),
        material=black,
        name="outlet_block",
    )
    frame.visual(
        Box((0.130, 0.006, 0.052)),
        origin=Origin(xyz=(0.135, outlet_center_y - 0.028, outlet_z)),
        material=label_white,
        name="outlet_faceplate",
    )
    for i, x in enumerate((0.100, 0.170)):
        frame.visual(
            Cylinder(radius=0.014, length=0.007),
            origin=Origin(
                xyz=(x, outlet_center_y - 0.033, outlet_z + 0.015),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_rubber,
            name=f"socket_recess_{i}",
        )
        frame.visual(
            Cylinder(radius=0.0045, length=0.005),
            origin=Origin(
                xyz=(x, outlet_center_y - 0.031, outlet_z - 0.010),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=brass,
            name=f"terminal_screw_{i}",
        )

    frame.visual(
        Cylinder(radius=0.018, length=0.044),
        origin=Origin(
            xyz=(0.215, outlet_center_y, outlet_z - 0.050),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=grey,
        name="strain_relief_collar",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.050),
        origin=Origin(
            xyz=(0.215, outlet_center_y - 0.030, outlet_z - 0.050),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_rubber,
        name="rubber_gland",
    )

    # Flexible output lead hanging down from the strain relief.
    cable_geom = tube_from_spline_points(
        [
            (0.215, outlet_center_y - 0.055, outlet_z - 0.050),
            (0.240, outlet_center_y - 0.110, outlet_z - 0.078),
            (0.210, outlet_center_y - 0.170, outlet_z - 0.088),
            (0.140, outlet_center_y - 0.195, outlet_z - 0.085),
            (0.040, outlet_center_y - 0.190, outlet_z - 0.076),
        ],
        radius=0.008,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )
    frame.visual(
        mesh_from_geometry(cable_geom, "output_lead"),
        material=blue,
        name="output_lead",
    )
    frame.visual(
        Cylinder(radius=0.014, length=0.055),
        origin=Origin(
            xyz=(0.015, outlet_center_y - 0.188, outlet_z - 0.074),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_rubber,
        name="plug_boot",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.035),
        origin=Origin(
            xyz=(-0.025, outlet_center_y - 0.188, outlet_z - 0.074),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=grey,
        name="metal_plug_shell",
    )

    # Labels on the wall plate face, below the reel sweep.
    label_y = WALL_FRONT_Y - 0.001
    frame.visual(
        Box((0.115, 0.003, 0.032)),
        origin=Origin(xyz=(-0.090, label_y, 0.180)),
        material=label_white,
        name="brand_label",
    )
    frame.visual(
        Box((0.155, 0.003, 0.018)),
        origin=Origin(xyz=(-0.090, label_y, 0.140)),
        material=label_blue,
        name="rating_label",
    )
    frame.visual(
        Box((0.055, 0.003, 0.018)),
        origin=Origin(xyz=(-0.210, label_y, 0.180)),
        material=warning_yellow,
        name="warning_label",
    )

    # Raised logo strokes on the brand label area.
    for i, (x, z, w) in enumerate(
        [
            (-0.132, 0.175, 0.010),
            (-0.115, 0.187, 0.020),
            (-0.090, 0.175, 0.012),
            (-0.066, 0.188, 0.026),
        ]
    ):
        frame.visual(
            Box((w, 0.002, 0.006)),
            origin=Origin(xyz=(x, label_y - 0.002, z)),
            material=label_blue,
            name=f"brand_stroke_{i}",
        )

    # Small grounding lug on the wall plate.
    frame.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(
            xyz=(-0.230, WALL_FRONT_Y - 0.001, 0.120),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=brass,
        name="grounding_lug",
    )

    # ── REEL: identical to parent baseline ───────────────────────────────
    reel = model.part("reel")
    cheek_mesh = mesh_from_geometry(
        _annular_cylinder_x(0.248, 0.052, 0.026), "black_spool_cheek"
    )
    drum_mesh = mesh_from_geometry(
        _annular_cylinder_x(0.170, 0.055, 0.344), "hollow_drum_core"
    )
    hub_mesh = mesh_from_geometry(
        _annular_cylinder_x(0.068, 0.017, 0.430), "visible_hub_sleeve"
    )

    reel.visual(
        cheek_mesh,
        origin=Origin(xyz=(-0.187, 0.0, 0.0)),
        material=black,
        name="spool_cheek_0",
    )
    reel.visual(
        cheek_mesh,
        origin=Origin(xyz=(0.187, 0.0, 0.0)),
        material=black,
        name="spool_cheek_1",
    )
    reel.visual(drum_mesh, material=black, name="hollow_drum_core")
    reel.visual(hub_mesh, material=black, name="visible_hub")

    # Visible wound cable: individual rubber loops.
    loop_mesh = mesh_from_geometry(
        TorusGeometry(
            radius=0.178, tube=0.0085, radial_segments=14, tubular_segments=76
        ),
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
        mesh_from_geometry(
            _annular_cylinder_x(0.050, 0.026, 0.018), "hub_cap_ring"
        ),
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

    # Offset winding crank in the gap between reel cheek and bracket.
    reel.visual(
        Box((0.013, 0.095, 0.018)),
        origin=Origin(xyz=(0.207, -0.130, 0.0)),
        material=orange,
        name="crank_arm",
    )
    reel.visual(
        Cylinder(radius=0.011, length=0.042),
        origin=Origin(
            xyz=(0.218, -0.175, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)
        ),
        material=grey,
        name="crank_peg",
    )
    reel.visual(
        Cylinder(radius=0.018, length=0.042),
        origin=Origin(
            xyz=(0.239, -0.175, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)
        ),
        material=dark_rubber,
        name="crank_knob",
    )

    # Captive cable end emerging from the rotating drum.
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
    reel.visual(
        mesh_from_geometry(reel_tail, "rotating_cable_tail"),
        material=orange,
        name="rotating_cable_tail",
    )

    # ── Articulation: reel rotates on the fixed axle (identical) ─────────
    model.articulation(
        "frame_to_reel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=reel,
        origin=Origin(xyz=(0.0, 0.0, AXLE_Z)),
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

    # Axle captured inside the hub sleeve (same as parent).
    ctx.allow_overlap(
        frame,
        reel,
        elem_a="fixed_axle",
        elem_b="visible_hub",
        reason=(
            "The static steel axle is intentionally captured inside the "
            "rotating hub sleeve; the mesh sleeve is a visual bearing proxy "
            "around the shaft."
        ),
    )

    ctx.check(
        "small class is cable reel",
        object_model.name == "electrical_cable_reel"
        and object_model.meta.get("small_class") == "Cable reel",
        details=f"name={object_model.name}, meta={object_model.meta}",
    )

    # ── Variant axis: wall bracket mount ─────────────────────────────────
    ctx.check(
        "wall bracket mount with flat plate and two A-brackets",
        all(
            frame.get_visual(n) is not None
            for n in (
                "wall_mounting_plate",
                "a_bracket_0",
                "a_bracket_1",
                "bearing_saddle_0",
                "bearing_saddle_1",
                "bracket_stiffener_0",
                "bracket_stiffener_1",
                "wall_bolt_0",
                "bracket_bolt_0_0",
                "bracket_bolt_1_0",
            )
        ),
        details=(
            "wall-mounted variant requires a flat wall mounting plate, two "
            "trapezoidal A-brackets with stiffener ribs, bearing saddles at "
            "each arm tip, and visible wall-anchor and bracket-to-plate bolts"
        ),
    )
    frame_visual_names = {v.name for v in frame.visuals}
    ctx.check(
        "no floor rails or feet on wall-mount variant",
        all(
            n not in frame_visual_names
            for n in (
                "front_base_rail",
                "rear_foot_rail",
                "front_foot_0",
                "front_foot_1",
            )
        ),
        details="wall-mounted reel must not have floor skid rails or feet",
    )

    # ── Primary joint (identical to parent) ──────────────────────────────
    ctx.check(
        "primary rotating reel joint exists",
        joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={joint.articulation_type}, axis={joint.axis}",
    )
    ctx.check(
        "two black spool cheeks and many wound cable loops",
        all(
            reel.get_visual(n) is not None
            for n in ("spool_cheek_0", "spool_cheek_1")
        )
        and sum(1 for v in reel.visuals if v.name.startswith("wound_loop_")) >= 28,
        details="expected paired flanges and individual wound cable loops",
    )

    # ── Electrical details (outlet, strain relief, lead, plug) ──────────
    ctx.check(
        "electrical outlet details present on wall plate",
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
                "grounding_lug",
            )
        ),
        details="outlet block, sockets, terminals, strain relief, lead, plug, and grounding lug required",
    )

    ctx.check(
        "labels and fasteners are modeled geometry",
        all(
            frame.get_visual(n) is not None
            for n in (
                "brand_label",
                "rating_label",
                "warning_label",
                "wall_bolt_0",
                "bracket_bolt_0_0",
            )
        ),
        details="labels and bolt heads should be separate attached visuals",
    )

    # ── Exact geometry assertions ────────────────────────────────────────
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

    # Reel clears the wall plate behind it.
    ctx.expect_gap(
        frame,
        reel,
        axis="y",
        positive_elem="wall_mounting_plate",
        negative_elem="spool_cheek_1",
        min_gap=0.03,
        name="reel cheek clears the wall mounting plate",
    )

    # Outlet block is below the reel drum.
    ctx.expect_gap(
        reel,
        frame,
        axis="z",
        positive_elem="hollow_drum_core",
        negative_elem="outlet_block",
        min_gap=0.01,
        name="rotating drum clears the outlet block below",
    )

    # ── Crank rotation proof ─────────────────────────────────────────────
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
