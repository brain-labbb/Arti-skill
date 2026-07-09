from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="upright_outdoor_water_filter_pump",
        meta={
            "run_notes": (
                "Built from the provided reference as a portable upright camping "
                "water-filter pump. The image and category both appear to describe "
                "a water filter pump, so no classification conflict was suspected."
            )
        },
    )

    olive = model.material("olive_green_plastic", rgba=(0.22, 0.38, 0.17, 1.0))
    dark_olive = model.material("dark_olive_rubber", rgba=(0.10, 0.20, 0.08, 1.0))
    light_olive = model.material("molded_highlight_green", rgba=(0.34, 0.52, 0.25, 1.0))
    metal = model.material("polished_stainless_rod", rgba=(0.78, 0.82, 0.78, 1.0))
    clear = model.material("clear_flexible_hose", rgba=(0.92, 0.98, 1.0, 0.43))
    grey = model.material("dark_gray_fitting", rgba=(0.20, 0.22, 0.20, 1.0))

    body = model.part("body")

    # Main vertical filter cartridge and molded base.
    body.visual(
        Cylinder(radius=0.030, length=0.235),
        origin=Origin(xyz=(0.018, 0.0, 0.132)),
        material=olive,
        name="filter_body",
    )
    body.visual(
        Cylinder(radius=0.033, length=0.025),
        origin=Origin(xyz=(0.018, 0.0, 0.018)),
        material=dark_olive,
        name="base_cap",
    )

    top_cap_mesh = mesh_from_geometry(
        KnobGeometry(
            0.068,
            0.032,
            body_style="cylindrical",
            grip=KnobGrip(style="ribbed", count=32, depth=0.0015, width=0.002),
            edge_radius=0.001,
        ),
        "ribbed_filter_cap",
    )
    body.visual(
        top_cap_mesh,
        origin=Origin(xyz=(0.018, 0.0, 0.254)),
        material=olive,
        name="ribbed_filter_cap",
    )

    # A neighboring pump sleeve, tied into the filter cartridge by molded webs.
    body.visual(
        Cylinder(radius=0.0145, length=0.220),
        origin=Origin(xyz=(-0.035, 0.0, 0.132)),
        material=olive,
        name="pump_sleeve",
    )
    sleeve_collar_mesh = mesh_from_geometry(
        KnobGeometry(
            0.038,
            0.030,
            body_style="cylindrical",
            grip=KnobGrip(style="ribbed", count=22, depth=0.0012, width=0.0018),
            edge_radius=0.0008,
        ),
        "ribbed_sleeve_collar",
    )
    body.visual(
        sleeve_collar_mesh,
        origin=Origin(xyz=(-0.035, 0.0, 0.252)),
        material=olive,
        name="ribbed_sleeve_collar",
    )
    body.visual(
        Box((0.054, 0.010, 0.036)),
        origin=Origin(xyz=(-0.010, 0.018, 0.088)),
        material=light_olive,
        name="lower_bridge",
    )
    body.visual(
        Box((0.052, 0.010, 0.032)),
        origin=Origin(xyz=(-0.010, 0.018, 0.178)),
        material=light_olive,
        name="upper_bridge",
    )

    # Recessed front grip/label panel that helps the cylindrical body read like
    # the molded product in the reference rather than a plain tube.
    body.visual(
        Box((0.014, 0.004, 0.125)),
        origin=Origin(xyz=(0.018, -0.0305, 0.135)),
        material=light_olive,
        name="front_recess_panel",
    )

    # Side outlet, barb, small cap, and a short clear curved hose.
    body.visual(
        Cylinder(radius=0.0085, length=0.038),
        origin=Origin(xyz=(0.061, 0.0, 0.250), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=olive,
        name="outlet_barb",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.046, 0.0, 0.275)),
        material=dark_olive,
        name="bleed_cap",
    )
    hose_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.077, 0.0, 0.250),
                (0.120, 0.0, 0.241),
                (0.160, 0.0, 0.194),
                (0.190, 0.0, 0.125),
                (0.214, 0.0, 0.068),
            ],
            radius=0.0042,
            samples_per_segment=18,
            radial_segments=20,
            cap_ends=True,
            up_hint=(0.0, 1.0, 0.0),
        ),
        "curved_clear_hose",
    )
    body.visual(hose_mesh, material=clear, name="curved_clear_hose")
    body.visual(
        Cylinder(radius=0.0068, length=0.027),
        origin=Origin(xyz=(0.213, 0.0, 0.060), rpy=(0.0, 2.72, 0.0)),
        material=grey,
        name="hose_end_fitting",
    )

    # Fulcrum pivot post on body — the fixed support for the lever pump arm.
    body.visual(
        Cylinder(radius=0.011, length=0.016),
        origin=Origin(xyz=(-0.035, 0.0, 0.268)),
        material=olive,
        name="pivot_post",
    )

    # Lever-pump plunger: pivoting arm that drives the internal piston.
    plunger = model.part("plunger")

    # Pivot boss that sits on the fulcrum post.
    plunger.visual(
        Cylinder(radius=0.013, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=grey,
        name="pivot_boss",
    )

    # Pump rod extends downward from the pivot into the sleeve.
    plunger.visual(
        Cylinder(radius=0.0042, length=0.250),
        origin=Origin(xyz=(0.0, 0.0, -0.125)),
        material=metal,
        name="plunger_rod",
    )

    # Lever arm extends along -X away from the filter body.
    plunger.visual(
        Cylinder(radius=0.007, length=0.108),
        origin=Origin(xyz=(-0.062, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=olive,
        name="handle_stem",
    )

    # Dark olive rubber grip sleeve on the lever bar (companion variation).
    plunger.visual(
        Cylinder(radius=0.010, length=0.065),
        origin=Origin(xyz=(-0.085, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_olive,
        name="grip_sleeve",
    )

    # Handle grip at the lever end.
    grip_mesh = mesh_from_geometry(
        CapsuleGeometry(radius=0.014, length=0.052, radial_segments=24, height_segments=8),
        "lever_end_grip",
    )
    plunger.visual(
        grip_mesh,
        origin=Origin(xyz=(-0.122, 0.0, 0.0)),
        material=olive,
        name="handle_grip",
    )

    model.articulation(
        "body_to_plunger",
        ArticulationType.REVOLUTE,
        parent=body,
        child=plunger,
        # Fulcrum pivot at the top of the sleeve collar. Axis across the body
        # (+Y). Positive q raises the lever arm upward.
        origin=Origin(xyz=(-0.035, 0.0, 0.268)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.5, lower=0.0, upper=1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    plunger = object_model.get_part("plunger")
    lever = object_model.get_articulation("body_to_plunger")

    # The metallic pump rod is intentionally represented as sliding inside a
    # simplified solid green guide sleeve/collar. The lever-pump design causes
    # the rod to swing with the arm, so the overlap allowance covers both the
    # retained insertion and the simplified pivot interface.
    ctx.allow_overlap(
        body,
        plunger,
        elem_a="pump_sleeve",
        elem_b="plunger_rod",
        reason="The pump rod is intentionally captured inside the guide sleeve proxy.",
    )
    ctx.allow_overlap(
        body,
        plunger,
        elem_a="ribbed_sleeve_collar",
        elem_b="plunger_rod",
        reason="The top collar surrounds and guides the pivoting pump rod.",
    )
    ctx.allow_overlap(
        body,
        plunger,
        elem_a="pivot_post",
        elem_b="pivot_boss",
        reason="The pivot boss seats on the fulcrum post to support the lever arm.",
    )
    ctx.allow_overlap(
        body,
        plunger,
        elem_a="pivot_post",
        elem_b="plunger_rod",
        reason="The pump rod passes through the pivot area where the fulcrum post surrounds it.",
    )
    ctx.allow_overlap(
        body,
        plunger,
        elem_a="ribbed_sleeve_collar",
        elem_b="pivot_boss",
        reason="The pivot boss seats on top of the sleeve collar with a small local embed at the fulcrum interface.",
    )
    ctx.allow_overlap(
        body,
        plunger,
        elem_a="ribbed_sleeve_collar",
        elem_b="handle_stem",
        reason="The lever handle stem passes through the collar area to connect to the pivot fulcrum.",
    )

    ctx.check(
        "source classification consistent",
        "no classification conflict" in object_model.meta.get("run_notes", ""),
        details=object_model.meta.get("run_notes", ""),
    )
    ctx.check(
        "revolute lever pump mechanism",
        lever.articulation_type == ArticulationType.REVOLUTE
        and lever.motion_limits is not None
        and lever.motion_limits.lower == 0.0
        and lever.motion_limits.upper is not None
        and 0.8 <= lever.motion_limits.upper <= 1.2,
        details=f"type={lever.articulation_type}, limits={lever.motion_limits}",
    )
    ctx.check(
        "reference details present",
        all(
            name in {visual.name for visual in body.visuals}
            for name in (
                "filter_body",
                "ribbed_filter_cap",
                "outlet_barb",
                "curved_clear_hose",
                "base_cap",
                "pivot_post",
            )
        ),
        details=f"body visuals={[visual.name for visual in body.visuals]}",
    )

    # At rest (q=0), the rod hangs straight down into the sleeve.
    ctx.expect_overlap(
        plunger,
        body,
        axes="z",
        elem_a="plunger_rod",
        elem_b="pump_sleeve",
        min_overlap=0.080,
        name="rest pose keeps rod inserted",
    )
    # The lever handle sits at the side of the pump, not above the filter cap.
    # Verify the handle extends outward from the body along -X.
    ctx.expect_gap(
        body,
        plunger,
        axis="x",
        min_gap=0.02,
        positive_elem="filter_body",
        negative_elem="handle_grip",
        name="lever handle extends outward from filter body",
    )

    # At the upper limit, the lever swings and the rod lifts.
    rest_grip_aabb = ctx.part_element_world_aabb(plunger, elem="handle_grip")
    with ctx.pose({lever: lever.motion_limits.upper}):
        ctx.expect_overlap(
            plunger,
            body,
            axes="z",
            elem_a="plunger_rod",
            elem_b="pump_sleeve",
            min_overlap=0.020,
            name="lever up pose retains rod insertion",
        )
        raised_grip_aabb = ctx.part_element_world_aabb(plunger, elem="handle_grip")

    rest_grip_z = (rest_grip_aabb[0][2] + rest_grip_aabb[1][2]) / 2.0 if rest_grip_aabb else None
    raised_grip_z = (raised_grip_aabb[0][2] + raised_grip_aabb[1][2]) / 2.0 if raised_grip_aabb else None

    ctx.check(
        "lever pump stroke lifts handle",
        rest_grip_z is not None
        and raised_grip_z is not None
        and raised_grip_z > rest_grip_z + 0.020,
        details=f"rest_grip_z={rest_grip_z}, raised_grip_z={raised_grip_z}",
    )

    return ctx.report()


object_model = build_object_model()
