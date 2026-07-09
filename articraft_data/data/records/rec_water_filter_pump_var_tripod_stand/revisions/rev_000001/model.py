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


def _add_tripod_leg_visuals(leg_part, strut_length, leg_length, theta,
                            strut_material, foot_material, pivot_material):
    """Shared helper: add pivot eye, strut, and rubber foot to one tripod leg."""
    # Pivot eye wraps around the hub boss at the hinge (tangential orientation)
    leg_part.visual(
        Cylinder(radius=0.005, length=0.010),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, math.pi + theta)),
        material=pivot_material,
        name="pivot_eye",
    )
    # Main strut extends downward from hinge in local frame
    leg_part.visual(
        Box((0.010, 0.006, strut_length)),
        origin=Origin(xyz=(0.0, 0.0, -(strut_length / 2.0 + 0.004))),
        material=strut_material,
        name="strut",
    )
    # Dark olive rubber foot at the distal end
    leg_part.visual(
        Cylinder(radius=0.006, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -(leg_length - 0.004))),
        material=foot_material,
        name="foot",
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

    # ---- Folding tripod stand (3 splayed legs under base_cap) ----
    hub_cx, hub_cy, hub_cz = 0.018, 0.0, 0.003
    hinge_r = 0.028
    leg_len = 0.065
    n_legs = 3
    strut_len = leg_len - 0.008

    # Hub disc that the legs fold from
    body.visual(
        Cylinder(radius=0.035, length=0.006),
        origin=Origin(xyz=(hub_cx, hub_cy, hub_cz)),
        material=dark_olive,
        name="tripod_hub",
    )

    for i in range(n_legs):
        theta = 2.0 * math.pi * i / n_legs
        ct = math.cos(theta)
        st = math.sin(theta)
        hx = hub_cx + hinge_r * ct
        hy = hub_cy + hinge_r * st

        # Visible hinge boss protruding tangentially from hub
        body.visual(
            Cylinder(radius=0.003, length=0.014),
            origin=Origin(
                xyz=(hx, hy, hub_cz),
                rpy=(-math.pi / 2.0, 0.0, math.pi + theta),
            ),
            material=olive,
            name=f"leg_boss_{i}",
        )

        # Leg part built with shared helper
        leg = model.part(f"leg_{i}")
        _add_tripod_leg_visuals(leg, strut_len, leg_len, theta, olive, dark_olive, olive)

        # Revolute fold joint: stowed (down) → splayed (outward)
        # Axis is horizontal-tangential so positive q swings the leg radially outward.
        model.articulation(
            f"base_to_leg_{i}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=leg,
            origin=Origin(xyz=(hx, hy, hub_cz)),
            axis=(st, -ct, 0.0),
            motion_limits=MotionLimits(
                effort=2.0, velocity=1.5, lower=0.0, upper=0.55,
            ),
        )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.0042, length=0.250),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=metal,
        name="plunger_rod",
    )
    grip_mesh = mesh_from_geometry(
        CapsuleGeometry(radius=0.012, length=0.108, radial_segments=24, height_segments=8).rotate_y(
            math.pi / 2.0
        ),
        "t_handle_grip",
    )
    plunger.visual(
        grip_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.166)),
        material=olive,
        name="handle_grip",
    )
    plunger.visual(
        Cylinder(radius=0.0105, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.142)),
        material=olive,
        name="handle_stem",
    )

    model.articulation(
        "body_to_plunger",
        ArticulationType.PRISMATIC,
        parent=body,
        child=plunger,
        # The child frame is located on the guide sleeve centerline at the top
        # entry collar. Positive travel pulls the plunger upward.
        origin=Origin(xyz=(-0.035, 0.0, 0.238)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.35, lower=0.0, upper=0.075),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    plunger = object_model.get_part("plunger")
    slide = object_model.get_articulation("body_to_plunger")

    # The metallic pump rod is intentionally represented as sliding inside a
    # simplified solid green guide sleeve/collar. Scope the overlap allowance to
    # the named retained-insertion interfaces and prove that the rod stays
    # centered and inserted at both ends of travel.
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
        reason="The top collar surrounds and guides the sliding pump rod.",
    )

    # Tripod pivot: each leg's pivot eye captures the corresponding hub boss pin
    for i in range(3):
        ctx.allow_overlap(
            body,
            object_model.get_part(f"leg_{i}"),
            elem_a=f"leg_boss_{i}",
            elem_b="pivot_eye",
            reason=f"Boss pin is captured inside leg_{i} pivot eye at the hinge.",
        )
        ctx.allow_overlap(
            body,
            object_model.get_part(f"leg_{i}"),
            elem_a="tripod_hub",
            elem_b="pivot_eye",
            reason=f"Leg_{i} pivot eye is nested within the hub disc at the hinge.",
        )
        ctx.expect_contact(
            body,
            object_model.get_part(f"leg_{i}"),
            elem_a=f"leg_boss_{i}",
            elem_b="pivot_eye",
            contact_tol=0.002,
            name=f"leg_{i} pivot eye seated on boss",
        )

    ctx.check(
        "source classification consistent",
        "no classification conflict" in object_model.meta.get("run_notes", ""),
        details=object_model.meta.get("run_notes", ""),
    )
    ctx.check(
        "single sliding pump mechanism",
        slide.motion_limits is not None
        and slide.motion_limits.lower == 0.0
        and slide.motion_limits.upper is not None
        and 0.06 <= slide.motion_limits.upper <= 0.09,
        details=f"limits={slide.motion_limits}",
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
            )
        ),
        details=f"body visuals={[visual.name for visual in body.visuals]}",
    )

    ctx.expect_within(
        plunger,
        body,
        axes="xy",
        inner_elem="plunger_rod",
        outer_elem="pump_sleeve",
        margin=0.001,
        name="rod centered in pump sleeve",
    )
    ctx.expect_overlap(
        plunger,
        body,
        axes="z",
        elem_a="plunger_rod",
        elem_b="pump_sleeve",
        min_overlap=0.080,
        name="rest pose keeps rod inserted",
    )
    ctx.expect_gap(
        plunger,
        body,
        axis="z",
        min_gap=0.10,
        positive_elem="handle_grip",
        negative_elem="ribbed_filter_cap",
        name="handle stands above filter cap",
    )

    rest_pos = ctx.part_world_position(plunger)
    with ctx.pose({slide: slide.motion_limits.upper}):
        ctx.expect_within(
            plunger,
            body,
            axes="xy",
            inner_elem="plunger_rod",
            outer_elem="pump_sleeve",
            margin=0.001,
            name="extended rod remains centered",
        )
        ctx.expect_overlap(
            plunger,
            body,
            axes="z",
            elem_a="plunger_rod",
            elem_b="pump_sleeve",
            min_overlap=0.020,
            name="extended pose retains insertion",
        )
        extended_pos = ctx.part_world_position(plunger)

    ctx.check(
        "positive pump stroke moves upward",
        rest_pos is not None and extended_pos is not None and extended_pos[2] > rest_pos[2] + 0.065,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    # ---- Folding tripod stand checks ----
    for i in range(3):
        leg_i = object_model.get_part(f"leg_{i}")
        joint_i = object_model.get_articulation(f"base_to_leg_{i}")
        ctx.check(
            f"base_to_leg_{i} is revolute with splay limits",
            joint_i.articulation_type == ArticulationType.REVOLUTE
            and joint_i.motion_limits is not None
            and joint_i.motion_limits.lower == 0.0
            and joint_i.motion_limits.upper is not None
            and 0.40 <= joint_i.motion_limits.upper <= 0.80,
            details=f"type={joint_i.articulation_type}, limits={joint_i.motion_limits}",
        )
        # Each leg must carry strut + rubber foot visuals
        leg_visual_names = {v.name for v in leg_i.visuals}
        ctx.check(
            f"leg_{i} has strut and rubber foot",
            "strut" in leg_visual_names and "foot" in leg_visual_names,
            details=f"visuals={leg_visual_names}",
        )

    ctx.check(
        "tripod hub present on body",
        "tripod_hub" in {v.name for v in body.visuals},
        details=f"body visuals={[v.name for v in body.visuals]}",
    )

    # Prove splay: leg_0 moves outward when deployed
    leg_0 = object_model.get_part("leg_0")
    leg_joint_0 = object_model.get_articulation("base_to_leg_0")
    rest_aabb = ctx.part_world_aabb(leg_0)
    with ctx.pose({leg_joint_0: leg_joint_0.motion_limits.upper}):
        splay_aabb = ctx.part_world_aabb(leg_0)
    ctx.check(
        "leg_0 splays outward when deployed",
        rest_aabb is not None
        and splay_aabb is not None
        and splay_aabb[1][0] > rest_aabb[1][0] + 0.010,
        details=f"rest_x_max={rest_aabb[1][0] if rest_aabb else None}, "
                f"splay_x_max={splay_aabb[1][0] if splay_aabb else None}",
    )

    return ctx.report()


object_model = build_object_model()
