from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


POLE_X = (-0.085, 0.085)
POLE_Y = 0.0

# Parameters for each intermediate telescoping stage (top to bottom).
# Stage 0 sits just below upper_assemblies; stage N-1 sits just above lower_stage.
# Tube radii decrease so each stage nests inside the previous sleeve.
STAGE_DATA = [
    {
        "prefix": "mid",
        "tube_r": 0.0080,
        "tube_len": 0.340,
        "tube_z": -0.030,
        "sleeve_r": 0.0090,
        "sleeve_len": 0.130,
        "sleeve_z": -0.175,
        "band_r": 0.0100,
        "band_z": -0.130,
        "collar_r": 0.0130,
        "collar_h": 0.034,
        "collar_z": -0.200,
        "pin_r": 0.0040,
        "pin_y": 0.016,
        "clamp_size": (0.022, 0.009, 0.018),
        "clamp_z": -0.212,
        "joint_z": -0.200,
        "slide_upper": 0.120,
    },
    {
        "prefix": "mid2",
        "tube_r": 0.0064,
        "tube_len": 0.300,
        "tube_z": -0.025,
        "sleeve_r": 0.0074,
        "sleeve_len": 0.110,
        "sleeve_z": -0.155,
        "band_r": 0.0082,
        "band_z": -0.120,
        "collar_r": 0.0110,
        "collar_h": 0.030,
        "collar_z": -0.180,
        "pin_r": 0.0036,
        "pin_y": 0.013,
        "clamp_size": (0.020, 0.008, 0.017),
        "clamp_z": -0.192,
        "joint_z": -0.180,
        "slide_upper": 0.100,
    },
]

LOWER_SLIDE_UPPER = 0.100


def _cyl_z(part, radius, length, xyz, material, name):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _cyl_x(part, radius, length, xyz, material, name):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _visual_ymax(ctx: TestContext, part, elem: str):
    box = ctx.part_element_world_aabb(part, elem=elem)
    if box is None:
        return None
    return box[1][1]


def _make_cork_handle_mesh(name: str):
    """Lathed, lightly ergonomic cork grip with top and lower flares."""
    profile = [
        (0.0130, -0.085),
        (0.0180, -0.075),
        (0.0160, -0.050),
        (0.0135, -0.020),
        (0.0165, 0.015),
        (0.0185, 0.050),
        (0.0155, 0.080),
        (0.0125, 0.088),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=48, closed=True), name)


def _make_foam_grip_mesh(name: str):
    """Ribbed lower black grip sleeve under the cork handle."""
    profile = [
        (0.0110, -0.070),
        (0.0165, -0.064),
        (0.0165, -0.048),
        (0.0125, -0.043),
        (0.0125, -0.030),
        (0.0160, -0.025),
        (0.0160, -0.010),
        (0.0125, -0.005),
        (0.0125, 0.010),
        (0.0156, 0.016),
        (0.0156, 0.030),
        (0.0122, 0.036),
        (0.0122, 0.050),
        (0.0150, 0.056),
        (0.0150, 0.070),
        (0.0110, 0.074),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=40, closed=True), name)


def _make_tip_mesh(name: str):
    return mesh_from_geometry(ConeGeometry(0.0075, 0.050, radial_segments=32, closed=True), name)


def _make_basket_mesh(name: str):
    return mesh_from_geometry(TorusGeometry(0.030, 0.0026, radial_segments=16, tubular_segments=48), name)


def _add_upper_pole(root, i, x, mats, handle_mesh, foam_mesh):
    cork, black, white, metal, dark = mats
    root.visual(
        handle_mesh,
        origin=Origin(xyz=(x, POLE_Y, 0.585)),
        material=cork,
        name=f"pole_{i}_cork_handle",
    )
    root.visual(
        foam_mesh,
        origin=Origin(xyz=(x, POLE_Y, 0.425)),
        material=black,
        name=f"pole_{i}_foam_grip",
    )
    _cyl_z(root, 0.0175, 0.030, (x, POLE_Y, 0.688), metal, f"pole_{i}_top_cap")
    _cyl_z(root, 0.0100, 0.310, (x, POLE_Y, 0.325), white, f"pole_{i}_upper_sleeve")
    _cyl_z(root, 0.0115, 0.016, (x, POLE_Y, 0.475), black, f"pole_{i}_upper_black_band")
    _cyl_z(root, 0.0115, 0.014, (x, POLE_Y, 0.205), black, f"pole_{i}_lower_black_band")
    _cyl_z(root, 0.0032, 0.308, (x, POLE_Y, 0.536), black, f"pole_{i}_hidden_core")
    root.visual(
        Box((0.005, 0.0024, 0.180)),
        origin=Origin(xyz=(x, POLE_Y - 0.0105, 0.335)),
        material=dark,
        name=f"pole_{i}_carbon_label",
    )
    _cyl_z(root, 0.0145, 0.038, (x, POLE_Y, 0.180), black, f"pole_{i}_upper_clamp_collar")
    _cyl_x(root, 0.0045, 0.028, (x, POLE_Y + 0.016, 0.180), metal, f"pole_{i}_upper_clamp_pin")
    root.visual(
        Box((0.024, 0.010, 0.020)),
        origin=Origin(xyz=(x, POLE_Y + 0.011, 0.166)),
        material=black,
        name=f"pole_{i}_upper_clamp_body",
    )
    root.visual(
        Box((0.018, 0.012, 0.088)),
        origin=Origin(xyz=(x - 0.038, POLE_Y - 0.049, 0.575), rpy=(0.0, 0.0, -0.35)),
        material=black,
        name=f"pole_{i}_strap_tag",
    )
    root.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (x - 0.006, -0.004, 0.670),
                    (x - 0.030, -0.036, 0.640),
                    (x - 0.037, -0.055, 0.585),
                    (x - 0.022, -0.040, 0.535),
                    (x - 0.004, -0.014, 0.610),
                    (x + 0.006, -0.004, 0.670),
                ],
                radius=0.0028,
                samples_per_segment=14,
                radial_segments=14,
                cap_ends=True,
            ),
            f"pole_{i}_wrist_loop_mesh",
        ),
        material=black,
        name=f"pole_{i}_wrist_loop",
    )


def _add_intermediate_stage(part, stage_data, mats):
    """Build tube, sleeve, band, clamp collar, pin, and clamp body for one
    intermediate telescoping stage. Shared helper used in a loop."""
    _, black, white, metal, dark = mats
    d = stage_data
    _cyl_z(part, d["tube_r"], d["tube_len"], (0.0, 0.0, d["tube_z"]), dark, "tube")
    _cyl_z(part, d["sleeve_r"], d["sleeve_len"], (0.0, 0.0, d["sleeve_z"]), white, "sleeve")
    _cyl_z(part, d["band_r"], 0.014, (0.0, 0.0, d["band_z"]), black, "sleeve_band")
    _cyl_z(part, d["collar_r"], d["collar_h"], (0.0, 0.0, d["collar_z"]), black, "clamp_collar")
    _cyl_x(part, d["pin_r"], 0.026, (0.0, d["pin_y"], d["collar_z"]), metal, "clamp_pin")
    part.visual(
        Box(d["clamp_size"]),
        origin=Origin(xyz=(0.0, 0.011, d["clamp_z"])),
        material=black,
        name="clamp_body",
    )


def _add_lower_stage(part, mats, tip_mesh, basket_mesh):
    _, black, _, metal, dark = mats
    _cyl_z(part, 0.0050, 0.480, (0.0, 0.0, -0.110), dark, "lower_tube")
    _cyl_z(part, 0.0062, 0.030, (0.0, 0.0, -0.325), metal, "ferrule")
    part.visual(
        basket_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.270)),
        material=black,
        name="basket_ring",
    )
    part.visual(
        Box((0.066, 0.004, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.270)),
        material=black,
        name="basket_spoke_x",
    )
    part.visual(
        Box((0.004, 0.066, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.270)),
        material=black,
        name="basket_spoke_y",
    )
    part.visual(
        tip_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.362), rpy=(math.pi, 0.0, 0.0)),
        material=metal,
        name="carbide_tip",
    )


def _add_lever_visuals(part, material, pin_material):
    _cyl_x(part, 0.0060, 0.017, (0.0, 0.0, 0.0), pin_material, "lever_knuckle")
    part.visual(
        Box((0.014, 0.008, 0.058)),
        origin=Origin(xyz=(0.0, 0.008, -0.032)),
        material=material,
        name="lever_blade",
    )
    part.visual(
        Box((0.018, 0.007, 0.012)),
        origin=Origin(xyz=(0.0, 0.011, -0.060)),
        material=material,
        name="lever_lip",
    )


def _make_pair_tether_mesh(name: str):
    geom = tube_from_spline_points(
        [
            (POLE_X[0] + 0.006, -0.006, 0.672),
            (-0.030, -0.018, 0.690),
            (0.030, -0.018, 0.690),
            (POLE_X[1] - 0.006, -0.006, 0.672),
        ],
        radius=0.0019,
        samples_per_segment=12,
        radial_segments=12,
        cap_ends=True,
    )
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="collapsible_trekking_pole_pair",
        meta={
            "run_notes": (
                "4-section collapsible trekking pole pair with cork handles, "
                "wrist straps, segmented shafts, flip clamps, baskets, and tips. "
                "Each pole has upper + mid + mid2 + lower telescoping stages "
                "(4 nested tubes of decreasing diameter chained via prismatic joints)."
            )
        },
    )

    cork = model.material("cork_like_tan", rgba=(0.72, 0.45, 0.22, 1.0))
    black = model.material("matte_black_rubber", rgba=(0.005, 0.006, 0.006, 1.0))
    white = model.material("white_aluminum", rgba=(0.93, 0.92, 0.88, 1.0))
    metal = model.material("brushed_silver", rgba=(0.70, 0.72, 0.73, 1.0))
    dark = model.material("dark_carbon_fiber", rgba=(0.015, 0.018, 0.017, 1.0))
    mats = (cork, black, white, metal, dark)

    # --- Root: upper assemblies (handles, upper sleeves, upper clamps) ---
    upper = model.part("upper_assemblies")
    handle_mesh = _make_cork_handle_mesh("shared_cork_handle")
    foam_mesh = _make_foam_grip_mesh("shared_foam_grip")
    for i, x in enumerate(POLE_X):
        _add_upper_pole(upper, i, x, mats, handle_mesh, foam_mesh)

    upper.visual(
        _make_pair_tether_mesh("pair_tether_mesh"),
        material=black,
        name="pair_tether",
    )

    tip_mesh = _make_tip_mesh("shared_lower_tip")
    basket_mesh = _make_basket_mesh("shared_trekking_basket")

    # --- Per-pole telescoping chain: upper -> mid -> mid2 -> lower ---
    for i, x in enumerate(POLE_X):
        parent_part = upper
        parent_prefix = "upper"

        # Build intermediate stages via shared helper in a loop
        stage_parts = []  # [(part, prefix, stage_data), ...]
        for stage_idx, sd in enumerate(STAGE_DATA):
            prefix = sd["prefix"]
            stage_part = model.part(f"{prefix}_stage_{i}")
            _add_intermediate_stage(stage_part, sd, mats)

            # Prismatic joint: parent -> this stage (slides along -Z)
            if stage_idx == 0:
                joint_origin = Origin(xyz=(x, POLE_Y, 0.180))
            else:
                joint_origin = Origin(xyz=(0.0, 0.0, prev_local_joint_z))

            model.articulation(
                f"{parent_prefix}_to_{prefix}_{i}",
                ArticulationType.PRISMATIC,
                parent=parent_part,
                child=stage_part,
                origin=joint_origin,
                axis=(0.0, 0.0, -1.0),
                motion_limits=MotionLimits(
                    effort=70.0, velocity=0.25,
                    lower=0.0, upper=sd["slide_upper"],
                ),
            )

            stage_parts.append((stage_part, prefix, sd))
            parent_part = stage_part
            parent_prefix = prefix
            prev_local_joint_z = sd["joint_z"]

        # Terminal lower stage
        lower = model.part(f"lower_stage_{i}")
        _add_lower_stage(lower, mats, tip_mesh, basket_mesh)

        model.articulation(
            f"{parent_prefix}_to_lower_{i}",
            ArticulationType.PRISMATIC,
            parent=parent_part,
            child=lower,
            origin=Origin(xyz=(0.0, 0.0, prev_local_joint_z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=60.0, velocity=0.22,
                lower=0.0, upper=LOWER_SLIDE_UPPER,
            ),
        )

        # --- Flip-lock levers ---

        # Upper clamp lever (on upper_assemblies at the upper clamp collar)
        upper_lever = model.part(f"upper_lever_{i}")
        _add_lever_visuals(upper_lever, black, metal)
        model.articulation(
            f"upper_clamp_hinge_{i}",
            ArticulationType.REVOLUTE,
            parent=upper,
            child=upper_lever,
            origin=Origin(xyz=(x, POLE_Y + 0.016, 0.180)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=1.25),
        )

        # Per-stage clamp levers (one per intermediate stage)
        for stage_part, prefix, sd in stage_parts:
            lever = model.part(f"{prefix}_lever_{i}")
            _add_lever_visuals(lever, black, metal)
            model.articulation(
                f"{prefix}_clamp_hinge_{i}",
                ArticulationType.REVOLUTE,
                parent=stage_part,
                child=lever,
                origin=Origin(xyz=(0.0, sd["pin_y"], sd["collar_z"])),
                axis=(1.0, 0.0, 0.0),
                motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=1.25),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    upper = object_model.get_part("upper_assemblies")

    for i in range(2):
        # Collect parts and joints for this pole's telescoping chain
        stages = []  # [(part, prefix, stage_data), ...]
        for sd in STAGE_DATA:
            prefix = sd["prefix"]
            stages.append(
                (object_model.get_part(f"{prefix}_stage_{i}"), prefix, sd)
            )

        lower = object_model.get_part(f"lower_stage_{i}")
        last_stage, last_prefix, last_sd = stages[-1]

        # Prismatic joints in chain order
        prismatic_joints = []
        parent_prefix = "upper"
        for sd in STAGE_DATA:
            prefix = sd["prefix"]
            prismatic_joints.append(
                object_model.get_articulation(f"{parent_prefix}_to_{prefix}_{i}")
            )
            parent_prefix = prefix
        prismatic_joints.append(
            object_model.get_articulation(f"{parent_prefix}_to_lower_{i}")
        )

        # Lever parts and hinge joints
        upper_lever = object_model.get_part(f"upper_lever_{i}")
        upper_hinge = object_model.get_articulation(f"upper_clamp_hinge_{i}")

        stage_levers = []
        stage_hinges = []
        for sd in STAGE_DATA:
            prefix = sd["prefix"]
            stage_levers.append(object_model.get_part(f"{prefix}_lever_{i}"))
            stage_hinges.append(
                object_model.get_articulation(f"{prefix}_clamp_hinge_{i}")
            )

        # ============================================================
        # Overlap allowances
        # ============================================================

        # Upper sleeve overlaps with first intermediate stage (mid) tube
        mid_part, mid_prefix, mid_sd = stages[0]
        ctx.allow_overlap(
            upper, mid_part,
            elem_a=f"pole_{i}_upper_sleeve", elem_b="tube",
            reason="The mid tube is intentionally retained inside the upper telescoping sleeve.",
        )
        ctx.allow_overlap(
            upper, mid_part,
            elem_a=f"pole_{i}_upper_clamp_collar", elem_b="tube",
            reason="The upper flip-lock collar clamps around the sliding mid tube.",
        )
        ctx.allow_overlap(
            upper, mid_part,
            elem_a=f"pole_{i}_lower_black_band", elem_b="tube",
            reason="The decorative lower band surrounds the mid tube at the sleeve mouth.",
        )

        # Adjacent intermediate stage overlaps (sleeve/band/collar contain next tube)
        for s_idx in range(len(stages) - 1):
            p_stage, p_prefix, p_sd = stages[s_idx]
            c_stage, c_prefix, c_sd = stages[s_idx + 1]

            ctx.allow_overlap(
                p_stage, c_stage,
                elem_a="sleeve", elem_b="tube",
                reason=f"The {c_prefix} tube is intentionally inserted into the {p_prefix} sleeve.",
            )
            ctx.allow_overlap(
                p_stage, c_stage,
                elem_a="clamp_collar", elem_b="tube",
                reason=f"The {p_prefix} clamp collar clamps around the {c_prefix} tube.",
            )
            ctx.allow_overlap(
                p_stage, c_stage,
                elem_a="sleeve_band", elem_b="tube",
                reason=f"The {p_prefix} sleeve band encircles the {c_prefix} tube.",
            )
            ctx.allow_overlap(
                p_stage, c_stage,
                elem_a="tube", elem_b="tube",
                reason=f"The {c_prefix} tube telescopes inside the {p_prefix} tube proxy.",
            )

        # Last intermediate stage to lower
        ctx.allow_overlap(
            last_stage, lower,
            elem_a="sleeve", elem_b="lower_tube",
            reason="The lower tube is intentionally inserted into the last intermediate sleeve.",
        )
        ctx.allow_overlap(
            last_stage, lower,
            elem_a="clamp_collar", elem_b="lower_tube",
            reason="The last stage clamp collar clamps around the lower tube.",
        )
        ctx.allow_overlap(
            last_stage, lower,
            elem_a="sleeve_band", elem_b="lower_tube",
            reason="The last stage sleeve band encircles the lower tube.",
        )
        ctx.allow_overlap(
            last_stage, lower,
            elem_a="tube", elem_b="lower_tube",
            reason="The lower tube telescopes inside the last intermediate tube proxy.",
        )

        # Lever knuckle/pin overlaps
        ctx.allow_overlap(
            upper, upper_lever,
            elem_a=f"pole_{i}_upper_clamp_pin", elem_b="lever_knuckle",
            reason="The upper lever knuckle rotates around the clamp pin.",
        )
        for s_idx in range(len(stages)):
            stage_part, prefix, sd = stages[s_idx]
            lever = stage_levers[s_idx]
            ctx.allow_overlap(
                stage_part, lever,
                elem_a="clamp_pin", elem_b="lever_knuckle",
                reason=f"The {prefix} lever knuckle rotates around the clamp pin.",
            )

        # ============================================================
        # Exact assertions: rest-pose containment and retention
        # ============================================================

        # Mid tube stays coaxial inside upper sleeve
        ctx.expect_within(
            mid_part, upper, axes="xy",
            inner_elem="tube", outer_elem=f"pole_{i}_upper_sleeve",
            margin=0.004,
            name=f"mid stage {i} stays coaxial in upper sleeve",
        )
        ctx.expect_overlap(
            mid_part, upper, axes="z",
            elem_a="tube", elem_b=f"pole_{i}_upper_sleeve",
            min_overlap=0.045,
            name=f"mid stage {i} retained in collapsed upper sleeve",
        )
        ctx.expect_overlap(
            mid_part, upper, axes="z",
            elem_a="tube", elem_b=f"pole_{i}_lower_black_band",
            min_overlap=0.005,
            name=f"mid stage {i} passes through upper band",
        )

        # Stage-to-stage containment and retention
        for s_idx in range(len(stages) - 1):
            p_stage, p_prefix, _ = stages[s_idx]
            c_stage, c_prefix, _ = stages[s_idx + 1]

            ctx.expect_within(
                c_stage, p_stage, axes="xy",
                inner_elem="tube", outer_elem="sleeve",
                margin=0.004,
                name=f"{c_prefix} stage {i} stays coaxial in {p_prefix} sleeve",
            )
            ctx.expect_overlap(
                c_stage, p_stage, axes="z",
                elem_a="tube", elem_b="sleeve",
                min_overlap=0.040,
                name=f"{c_prefix} stage {i} retained in {p_prefix} sleeve",
            )
            ctx.expect_overlap(
                c_stage, p_stage, axes="z",
                elem_a="tube", elem_b="sleeve_band",
                min_overlap=0.005,
                name=f"{c_prefix} stage {i} passes through {p_prefix} band",
            )

        # Last stage to lower containment and retention
        ctx.expect_within(
            lower, last_stage, axes="xy",
            inner_elem="lower_tube", outer_elem="sleeve",
            margin=0.004,
            name=f"lower stage {i} stays coaxial in {last_prefix} sleeve",
        )
        ctx.expect_overlap(
            lower, last_stage, axes="z",
            elem_a="lower_tube", elem_b="sleeve",
            min_overlap=0.040,
            name=f"lower stage {i} retained in {last_prefix} sleeve",
        )
        ctx.expect_overlap(
            lower, last_stage, axes="z",
            elem_a="lower_tube", elem_b="sleeve_band",
            min_overlap=0.005,
            name=f"lower stage {i} passes through {last_prefix} band",
        )

        # ============================================================
        # Articulated-pose checks: telescoping and lever flip
        # ============================================================

        rest_positions = [ctx.part_world_position(s[0]) for s in stages]
        rest_lower = ctx.part_world_position(lower)
        rest_upper_lever_y = _visual_ymax(ctx, upper_lever, "lever_blade")
        rest_stage_lever_y = [
            _visual_ymax(ctx, lev, "lever_blade") for lev in stage_levers
        ]

        # Build extension pose: all prismatic at max, all hinges open
        ext_pose = {}
        for j, sd in zip(prismatic_joints[:-1], STAGE_DATA):
            ext_pose[j] = sd["slide_upper"]
        ext_pose[prismatic_joints[-1]] = LOWER_SLIDE_UPPER
        ext_pose[upper_hinge] = 0.95
        for h in stage_hinges:
            ext_pose[h] = 0.95

        with ctx.pose(ext_pose):
            # Retention at extension
            ctx.expect_overlap(
                mid_part, upper, axes="z",
                elem_a="tube", elem_b=f"pole_{i}_upper_sleeve",
                min_overlap=0.010,
                name=f"mid stage {i} retained when extended",
            )
            for s_idx in range(len(stages) - 1):
                p_stage, p_prefix, _ = stages[s_idx]
                c_stage, c_prefix, _ = stages[s_idx + 1]
                ctx.expect_overlap(
                    c_stage, p_stage, axes="z",
                    elem_a="tube", elem_b="sleeve",
                    min_overlap=0.010,
                    name=f"{c_prefix} stage {i} retained when extended",
                )
            ctx.expect_overlap(
                lower, last_stage, axes="z",
                elem_a="lower_tube", elem_b="sleeve",
                min_overlap=0.010,
                name=f"lower stage {i} retained when extended",
            )

            ext_positions = [ctx.part_world_position(s[0]) for s in stages]
            ext_lower = ctx.part_world_position(lower)
            open_upper_lever_y = _visual_ymax(ctx, upper_lever, "lever_blade")
            open_stage_lever_y = [
                _visual_ymax(ctx, lev, "lever_blade") for lev in stage_levers
            ]

        # Each intermediate stage telescopes downward
        for s_idx in range(len(stages)):
            prefix = stages[s_idx][1]
            ctx.check(
                f"{prefix} stage {i} telescopes downward",
                rest_positions[s_idx] is not None
                and ext_positions[s_idx] is not None
                and ext_positions[s_idx][2] < rest_positions[s_idx][2] - 0.05,
                details=f"rest={rest_positions[s_idx]}, extended={ext_positions[s_idx]}",
            )

        ctx.check(
            f"lower stage {i} telescopes downward",
            rest_lower is not None
            and ext_lower is not None
            and ext_lower[2] < rest_lower[2] - 0.05,
            details=f"rest={rest_lower}, extended={ext_lower}",
        )

        # Upper lever flips outward
        ctx.check(
            f"upper clamp lever {i} flips outward",
            rest_upper_lever_y is not None
            and open_upper_lever_y is not None
            and open_upper_lever_y > rest_upper_lever_y + 0.006,
            details=f"rest_ymax={rest_upper_lever_y}, open_ymax={open_upper_lever_y}",
        )

        # Per-stage levers flip outward
        for s_idx in range(len(stages)):
            prefix = stages[s_idx][1]
            ctx.check(
                f"{prefix} clamp lever {i} flips outward",
                rest_stage_lever_y[s_idx] is not None
                and open_stage_lever_y[s_idx] is not None
                and open_stage_lever_y[s_idx] > rest_stage_lever_y[s_idx] + 0.006,
                details=f"rest_ymax={rest_stage_lever_y[s_idx]}, open_ymax={open_stage_lever_y[s_idx]}",
            )

        # ============================================================
        # TARGET assertion: 4th telescoping section (mid2_stage) exists
        # ============================================================
        mid2_part = object_model.get_part(f"mid2_stage_{i}")
        mid2_slide = object_model.get_articulation(f"mid_to_mid2_{i}")
        ctx.check(
            f"mid2_stage_{i} is a valid 4th telescoping section",
            mid2_part is not None and mid2_slide is not None,
            details="4-section pole requires mid2_stage part and mid_to_mid2 prismatic joint",
        )

    return ctx.report()


object_model = build_object_model()
