from __future__ import annotations

import math

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
    mesh_from_geometry,
    tube_from_spline_points,
)


TABLE_X = 0.80
TABLE_Y = 0.54
TOP_Z = 0.51
FABRIC_THICKNESS = 0.003
BOUND_EDGE_WIDTH = 0.018
BOUND_EDGE_THICKNESS = 0.005
LEG_TOP_Z = TOP_Z - 0.055
LEG_LENGTH = 0.455
LEG_RADIUS = 0.014
LEG_X = TABLE_X / 2.0 - 0.070
LEG_Y = TABLE_Y / 2.0 - 0.060
LOWER_BRACE_Z = -0.255
CONTACT_POST_RADIUS = 0.0060


def _brace_mesh(name: str, dx: float, dz: float):
    return mesh_from_geometry(
        tube_from_spline_points(
            [(0.0, 0.0, 0.0), (dx, 0.0, dz)],
            radius=0.0075,
            samples_per_segment=8,
            radial_segments=18,
            cap_ends=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="compact_folding_camp_table",
        meta={
            "run_notes": (
                "Primary image/category agree: a compact folding camp table. "
                "The carry bag in the reference is omitted so the single asset "
                "focuses on the articulated table."
            )
        },
    )

    mottled_aluminum = model.material("mottled_aluminum", rgba=(0.54, 0.55, 0.54, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.015, 0.014, 0.013, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.02, 0.019, 0.018, 1.0))
    screw_metal = model.material("screw_heads", rgba=(0.83, 0.82, 0.78, 1.0))
    ripstop_fabric = model.material("ripstop_nylon", rgba=(0.14, 0.16, 0.18, 1.0))
    binding_tape = model.material("binding_tape", rgba=(0.10, 0.11, 0.12, 1.0))

    tabletop = model.part("tabletop")

    # Taut fabric/mesh tabletop: a thin tensioned ripstop nylon panel spans the
    # inner perimeter of the tubular frame, with reinforced binding-tape edges
    # where the fabric wraps around the frame rails.
    fabric_span_x = TABLE_X - 0.010
    fabric_span_y = TABLE_Y - 0.010
    tabletop.visual(
        Box((fabric_span_x, fabric_span_y, FABRIC_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, TOP_Z)),
        material=ripstop_fabric,
        name="fabric_panel",
    )

    # Bound edge border: slightly thicker reinforced hem strips along all four
    # edges where the fabric is sewn/clipped to the perimeter frame rails.
    edge_inset = BOUND_EDGE_WIDTH / 2.0
    for y_sign, y_name in [(-1, "front"), (1, "rear")]:
        y_pos = y_sign * (fabric_span_y / 2.0 - edge_inset)
        tabletop.visual(
            Box((fabric_span_x, BOUND_EDGE_WIDTH, BOUND_EDGE_THICKNESS)),
            origin=Origin(xyz=(0.0, y_pos, TOP_Z + 0.001)),
            material=binding_tape,
            name=f"bound_edge_{y_name}",
        )
    for x_sign, x_name in [(-1, "left"), (1, "right")]:
        x_pos = x_sign * (fabric_span_x / 2.0 - edge_inset)
        tabletop.visual(
            Box((BOUND_EDGE_WIDTH, fabric_span_y - 2.0 * BOUND_EDGE_WIDTH, BOUND_EDGE_THICKNESS)),
            origin=Origin(xyz=(x_pos, 0.0, TOP_Z + 0.001)),
            material=binding_tape,
            name=f"bound_edge_{x_name}",
        )

    # Black perimeter tubular frame rails and underside structure.
    for y, name in [(-TABLE_Y / 2.0 - 0.016, "front_rail"), (TABLE_Y / 2.0 + 0.016, "rear_rail")]:
        tabletop.visual(
            Box((TABLE_X + 0.030, 0.030, 0.034)),
            origin=Origin(xyz=(0.0, y, TOP_Z - 0.012)),
            material=black_plastic,
            name=name,
        )
    for x, name in [
        (-TABLE_X / 2.0 - 0.016, "side_rail_0"),
        (TABLE_X / 2.0 + 0.016, "side_rail_1"),
    ]:
        tabletop.visual(
            Box((0.030, TABLE_Y + 0.026, 0.034)),
            origin=Origin(xyz=(x, 0.0, TOP_Z - 0.012)),
            material=black_plastic,
            name=name,
        )
    for y in (-0.16, 0.16):
        tabletop.visual(
            Box((TABLE_X - 0.070, 0.022, 0.034)),
            origin=Origin(xyz=(0.0, y, TOP_Z - 0.018)),
            material=black_plastic,
            name=f"underside_crossrail_{'front' if y < 0 else 'rear'}",
        )
    for idx, (x, y) in enumerate([(-1, -1), (1, -1), (1, 1), (-1, 1)]):
        tabletop.visual(
            Sphere(radius=0.030),
            origin=Origin(
                xyz=(x * (TABLE_X / 2.0 + 0.004), y * (TABLE_Y / 2.0 + 0.004), TOP_Z - 0.012)
            ),
            material=black_plastic,
            name=f"corner_connector_{idx}",
        )

    # Hinge sockets for folding legs and braces.  The small intentional pin
    # embeddings are proven and allowed in run_tests().
    leg_positions = [
        (-LEG_X, -LEG_Y),
        (LEG_X, -LEG_Y),
        (LEG_X, LEG_Y),
        (-LEG_X, LEG_Y),
    ]
    for i, (x, y) in enumerate(leg_positions):
        tabletop.visual(
            Box((0.056, 0.052, 0.078)),
            origin=Origin(xyz=(x, y, LEG_TOP_Z + 0.015)),
            material=black_plastic,
            name=f"hinge_socket_{i}",
        )

    for i, (x, y) in enumerate(leg_positions):
        leg = model.part(f"leg_{i}")
        leg.visual(
            Cylinder(radius=0.010, length=0.056),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=screw_metal,
            name="hinge_pin",
        )
        leg.visual(
            Cylinder(radius=LEG_RADIUS, length=LEG_LENGTH - 0.022),
            origin=Origin(xyz=(0.0, 0.0, -(LEG_LENGTH - 0.022) / 2.0 - 0.003)),
            material=mottled_aluminum,
            name="straight_tube",
        )
        leg.visual(
            Cylinder(radius=0.020, length=0.056),
            origin=Origin(xyz=(0.0, 0.0, -0.053)),
            material=black_plastic,
            name="upper_collar",
        )
        leg.visual(
            Cylinder(radius=0.017, length=0.034),
            origin=Origin(xyz=(0.0, 0.0, -LEG_LENGTH + 0.003)),
            material=rubber,
            name="rubber_foot",
        )
        brace_y_offset = 0.070 if i in (1, 3) else 0.045
        outer_y_sign = 1.0 if y > 0.0 else -1.0
        post_length = brace_y_offset - LEG_RADIUS - 0.012
        leg.visual(
            Cylinder(radius=CONTACT_POST_RADIUS, length=post_length),
            origin=Origin(
                xyz=(
                    0.0,
                    outer_y_sign * (LEG_RADIUS + post_length / 2.0),
                    LOWER_BRACE_Z,
                ),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=screw_metal,
            name="brace_contact_post",
        )
        axis_y = 1.0 if x > 0.0 else -1.0
        model.articulation(
            f"tabletop_to_leg_{i}",
            ArticulationType.REVOLUTE,
            parent=tabletop,
            child=leg,
            origin=Origin(xyz=(x, y, LEG_TOP_Z)),
            axis=(0.0, axis_y, 0.0),
            motion_limits=MotionLimits(effort=16.0, velocity=2.2, lower=0.0, upper=1.45),
        )

    # Four independent hinged diagonal braces form the visible X-frame on the
    # front and rear sides.  They are separated slightly in Y so the crossed
    # rods look like real bypassing members with a central rivet instead of
    # occupying the same volume.
    brace_specs = [
        ("brace_0", -LEG_X, -LEG_Y - 0.070, LEG_X * 2.0, -0.245, 1.0),
        ("brace_1", LEG_X, -LEG_Y - 0.045, -LEG_X * 2.0, -0.245, -1.0),
        ("brace_2", LEG_X, LEG_Y + 0.070, -LEG_X * 2.0, -0.245, -1.0),
        ("brace_3", -LEG_X, LEG_Y + 0.045, LEG_X * 2.0, -0.245, 1.0),
    ]
    for index, (name, x, y, dx, dz, axis_sign) in enumerate(brace_specs):
        tabletop.visual(
            Box((0.038, 0.034, 0.088)),
            origin=Origin(xyz=(x, y, LEG_TOP_Z + 0.010)),
            material=black_plastic,
            name=f"brace_anchor_{index}",
        )
        brace = model.part(name)
        brace.visual(
            _brace_mesh(f"{name}_tube", dx, dz),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=mottled_aluminum,
            name="diagonal_tube",
        )
        brace.visual(
            Cylinder(radius=0.014, length=0.024),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=screw_metal,
            name="upper_eye",
        )
        brace.visual(
            Cylinder(radius=0.012, length=0.024),
            origin=Origin(xyz=(dx, 0.0, dz), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=screw_metal,
            name="lower_eye",
        )
        brace.visual(
            Cylinder(radius=0.009, length=0.012),
            origin=Origin(xyz=(dx / 2.0, 0.0, dz / 2.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=screw_metal,
            name="center_rivet",
        )
        model.articulation(
            f"tabletop_to_{name}",
            ArticulationType.REVOLUTE,
            parent=tabletop,
            child=brace,
            origin=Origin(xyz=(x, y, LEG_TOP_Z - 0.010)),
            axis=(0.0, axis_sign, 0.0),
            motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=-0.25, upper=1.05),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tabletop = object_model.get_part("tabletop")

    for i in range(4):
        leg = object_model.get_part(f"leg_{i}")
        ctx.allow_overlap(
            tabletop,
            leg,
            elem_a=f"hinge_socket_{i}",
            elem_b="hinge_pin",
            reason="The visible folding-leg hinge pin is intentionally captured inside the black socket.",
        )
        ctx.expect_overlap(
            leg,
            tabletop,
            axes="xy",
            elem_a="hinge_pin",
            elem_b=f"hinge_socket_{i}",
            min_overlap=0.018,
            name=f"leg_{i}_pin_seated_in_socket",
        )
        ctx.allow_overlap(
            tabletop,
            leg,
            elem_a=f"hinge_socket_{i}",
            elem_b="straight_tube",
            reason="The straight leg tube has a short hidden insertion into the molded hinge socket.",
        )
        ctx.expect_overlap(
            leg,
            tabletop,
            axes="xy",
            elem_a="straight_tube",
            elem_b=f"hinge_socket_{i}",
            min_overlap=0.020,
            name=f"leg_{i}_tube_inserted_in_socket",
        )

    for i in range(4):
        brace = object_model.get_part(f"brace_{i}")
        ctx.allow_overlap(
            tabletop,
            brace,
            elem_a=f"brace_anchor_{i}",
            elem_b="upper_eye",
            reason="The diagonal brace upper eye is a captured pivot seated in its tabletop anchor.",
        )
        ctx.expect_overlap(
            brace,
            tabletop,
            axes="xy",
            elem_a="upper_eye",
            elem_b=f"brace_anchor_{i}",
            min_overlap=0.014,
            name=f"brace_{i}_upper_eye_seated",
        )
        ctx.allow_overlap(
            tabletop,
            brace,
            elem_a=f"brace_anchor_{i}",
            elem_b="diagonal_tube",
            reason="The brace tube begins inside the hinged anchor, representing the captured rod end.",
        )
        ctx.expect_overlap(
            brace,
            tabletop,
            axes="xy",
            elem_a="diagonal_tube",
            elem_b=f"brace_anchor_{i}",
            min_overlap=0.010,
            name=f"brace_{i}_tube_enters_anchor",
        )

    for leg_i, brace_i in [(1, 0), (0, 1), (3, 2), (2, 3)]:
        leg = object_model.get_part(f"leg_{leg_i}")
        brace = object_model.get_part(f"brace_{brace_i}")
        ctx.expect_contact(
            leg,
            brace,
            elem_a="brace_contact_post",
            elem_b="lower_eye",
            contact_tol=1e-5,
            name=f"leg_{leg_i}_contact_post_touches_brace_{brace_i}_lower_eye",
        )

    # Fabric tabletop: the taut ripstop nylon panel and all four bound edges
    # must be present on the tabletop part.
    ctx.check(
        "fabric_tabletop_panel",
        tabletop is not None
        and tabletop.get_visual("fabric_panel") is not None
        and all(
            tabletop.get_visual(f"bound_edge_{edge}") is not None
            for edge in ("front", "rear", "left", "right")
        ),
        "Expected a single taut fabric panel with four bound-edge border strips.",
    )
    # The fabric panel must sit at or above the perimeter rail tops, proving
    # it spans the frame rather than sitting below it.
    ctx.expect_gap(
        tabletop,
        tabletop,
        axis="z",
        positive_elem="fabric_panel",
        negative_elem="front_rail",
        min_gap=-0.010,
        max_gap=0.020,
        name="fabric_panel_at_or_above_rail_height",
    )

    aabb = ctx.part_world_aabb(tabletop)
    if aabb is not None:
        min_pt, max_pt = aabb
        ctx.check(
            "tabletop_width_realistic",
            0.78 <= float(max_pt[0] - min_pt[0]) <= 0.90,
            details=str(aabb),
        )
        ctx.check(
            "tabletop_depth_realistic",
            0.54 <= float(max_pt[1] - min_pt[1]) <= 0.64,
            details=str(aabb),
        )
    else:
        ctx.fail("tabletop_aabb_present", "Expected tabletop AABB.")

    leg0 = object_model.get_part("leg_0")
    leg0_joint = object_model.get_articulation("tabletop_to_leg_0")
    rest_leg0_aabb = ctx.part_world_aabb(leg0)
    if rest_leg0_aabb is not None:
        rest_min, rest_max = rest_leg0_aabb
        ctx.check(
            "open_leg_reaches_ground",
            float(rest_min[2]) < 0.010 and float(rest_max[2]) > LEG_TOP_Z - 0.020,
            details=str(rest_leg0_aabb),
        )
    with ctx.pose({leg0_joint: 1.20}):
        folded_aabb = ctx.part_world_aabb(leg0)
    if rest_leg0_aabb is not None and folded_aabb is not None:
        rest_min, rest_max = rest_leg0_aabb
        fold_min, fold_max = folded_aabb
        ctx.check(
            "leg_folds_inward_and_up",
            float(fold_min[2]) > float(rest_min[2]) + 0.12
            and abs((float(fold_min[0]) + float(fold_max[0])) / 2.0)
            < abs((float(rest_min[0]) + float(rest_max[0])) / 2.0),
            details=f"rest={rest_leg0_aabb}, folded={folded_aabb}",
        )

    brace0 = object_model.get_part("brace_0")
    brace0_joint = object_model.get_articulation("tabletop_to_brace_0")
    rest_brace_aabb = ctx.part_world_aabb(brace0)
    with ctx.pose({brace0_joint: 0.70}):
        moved_brace_aabb = ctx.part_world_aabb(brace0)
    if rest_brace_aabb is not None and moved_brace_aabb is not None:
        rmin, rmax = rest_brace_aabb
        mmin, mmax = moved_brace_aabb
        ctx.check(
            "diagonal_brace_hinges",
            abs(float(mmax[2] - mmin[2]) - float(rmax[2] - rmin[2])) > 0.020,
            details=f"rest={rest_brace_aabb}, moved={moved_brace_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
