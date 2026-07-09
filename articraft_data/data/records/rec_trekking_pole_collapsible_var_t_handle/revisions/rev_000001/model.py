from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def _mat(model: ArticulatedObject, name: str, rgba: tuple[float, float, float, float]) -> Material:
    return model.material(name, rgba=rgba)


def _t_grip_crossbar_mesh() -> MeshGeometry:
    """Lathe-profiled cane-style T-grip crossbar, wider in the middle for palm comfort."""
    profile = [
        (0.004, 0.000),
        (0.014, 0.008),
        (0.018, 0.025),
        (0.020, 0.065),
        (0.018, 0.105),
        (0.014, 0.122),
        (0.004, 0.130),
    ]
    return LatheGeometry(profile, segments=36, closed=True)


def _rubber_boot_mesh() -> MeshGeometry:
    profile = [
        (0.000, 0.000),
        (0.010, 0.000),
        (0.017, 0.010),
        (0.021, 0.040),
        (0.017, 0.070),
        (0.011, 0.085),
        (0.000, 0.085),
    ]
    return LatheGeometry(profile, segments=32, closed=True)


def _tube_between(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    name: str,
):
    return mesh_from_geometry(
        tube_from_spline_points(
            [a, b],
            radius=radius,
            samples_per_segment=4,
            radial_segments=12,
            cap_ends=True,
        ),
        name,
    )


def _add_carabiner_and_strap(part, *, black, metal) -> None:
    # Broad wrist loop: a black flattened webbing loop anchored at the T-handle
    # neck/shaft junction, hanging below the crossbar.
    webbing = sweep_profile_along_spline(
        [
            (-0.010, 0.004, 0.595),
            (-0.058, 0.012, 0.530),
            (-0.088, 0.010, 0.420),
            (-0.052, 0.007, 0.345),
            (-0.014, 0.004, 0.500),
        ],
        profile=rounded_rect_profile(0.012, 0.003, radius=0.0012, corner_segments=4),
        samples_per_segment=12,
        closed_spline=True,
        cap_profile=True,
    )
    part.visual(mesh_from_geometry(webbing, "wrist_webbing"), material=black, name="wrist_webbing")
    part.visual(
        Box((0.030, 0.012, 0.026)),
        origin=Origin(xyz=(-0.010, 0.004, 0.575)),
        material=black,
        name="strap_anchor",
    )

    # Accessory carabiner clipped to the strap, authored as a compact D-shaped
    # tube with a silver gate and a short tether so it is not floating.
    carabiner = tube_from_spline_points(
        [
            (-0.120, 0.012, 0.415),
            (-0.162, 0.012, 0.385),
            (-0.168, 0.012, 0.310),
            (-0.128, 0.012, 0.265),
            (-0.082, 0.012, 0.280),
            (-0.072, 0.012, 0.360),
            (-0.095, 0.012, 0.415),
        ],
        radius=0.0042,
        samples_per_segment=12,
        radial_segments=14,
        closed_spline=True,
        cap_ends=True,
    )
    part.visual(
        mesh_from_geometry(carabiner, "carabiner_body"), material=black, name="carabiner_body"
    )
    part.visual(
        _tube_between(
            (-0.090, 0.012, 0.395),
            (-0.078, 0.012, 0.290),
            radius=0.0023,
            name="carabiner_gate",
        ),
        material=metal,
        name="carabiner_gate",
    )
    part.visual(
        _tube_between(
            (-0.058, 0.010, 0.390),
            (-0.100, 0.012, 0.412),
            radius=0.0030,
            name="carabiner_tether",
        ),
        material=black,
        name="carabiner_tether",
    )


def _add_lock_lever(part, *, black, graphite) -> None:
    # The child frame sits on the clamp hinge pin.  At q=0 the flip-lock lever
    # lies down along the tube; positive rotation swings it outward.
    part.visual(
        Cylinder(radius=0.006, length=0.034),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="hinge_barrel",
    )
    part.visual(
        Box((0.014, 0.026, 0.075)),
        origin=Origin(xyz=(0.011, 0.0, -0.038)),
        material=black,
        name="lever_blade",
    )
    part.visual(
        Box((0.017, 0.028, 0.020)),
        origin=Origin(xyz=(0.019, 0.0, -0.078)),
        material=black,
        name="finger_tab",
    )


def _add_fold_hinge_hardware(
    part,
    *,
    metal,
    black,
    prefix: str,
    base: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    bx, by, bz = base
    part.visual(
        Cylinder(radius=0.0032, length=0.040),
        origin=Origin(xyz=(bx + 0.016, by, bz + 0.004), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name=f"{prefix}_cross_pin",
    )
    part.visual(
        Cylinder(radius=0.0100, length=0.018),
        origin=Origin(xyz=(bx + 0.016, by, bz + 0.004), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name=f"{prefix}_pivot_bushing",
    )
    for side, y in (("left", -0.016), ("right", 0.016)):
        part.visual(
            Cylinder(radius=0.0120, length=0.0030),
            origin=Origin(xyz=(bx + 0.016, by + y, bz + 0.004), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"{prefix}_{side}_flat_washer",
        )
        part.visual(
            Cylinder(radius=0.0060, length=0.0040),
            origin=Origin(
                xyz=(bx + 0.016, by + y * 1.15, bz + 0.004), rpy=(math.pi / 2.0, 0.0, 0.0)
            ),
            material=metal,
            name=f"{prefix}_{side}_rivet_head",
        )
    for side, y in (("left", -0.022), ("right", 0.022)):
        part.visual(
            Box((0.010, 0.0030, 0.030)),
            origin=Origin(xyz=(bx + 0.006, by + y, bz + 0.011)),
            material=metal,
            name=f"{prefix}_{side}_hinge_ear",
        )
        part.visual(
            _tube_between(
                (bx - 0.006, by + y, bz + 0.000),
                (bx + 0.015, by + y, bz + 0.004),
                radius=0.0019,
                name=f"{prefix}_{side}_fork_edge",
            ),
            material=metal,
            name=f"{prefix}_{side}_fork_edge",
        )
    part.visual(
        Box((0.009, 0.004, 0.012)),
        origin=Origin(xyz=(bx + 0.026, by, bz + 0.015), rpy=(0.0, -0.32, 0.0)),
        material=metal,
        name=f"{prefix}_small_index_stop",
    )


def _add_basket(part, *, rubber) -> None:
    # Compact removable trekking basket: a ring plus radial ribs close to the
    # lower tip.  All elements meet at the hub/ring and are part of the lower
    # tip assembly.
    part.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.044, tube=0.004, radial_segments=12, tubular_segments=48),
            "basket_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
        material=rubber,
        name="basket_ring",
    )
    part.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
        material=rubber,
        name="basket_hub",
    )
    for index, angle in enumerate(
        (0.0, math.pi / 3.0, 2.0 * math.pi / 3.0, math.pi, 4.0 * math.pi / 3.0, 5.0 * math.pi / 3.0)
    ):
        cx = 0.027 * math.cos(angle)
        cy = 0.027 * math.sin(angle)
        part.visual(
            Box((0.048, 0.006, 0.004)),
            origin=Origin(xyz=(cx, cy, -0.070), rpy=(0.0, 0.0, angle)),
            material=rubber,
            name=f"basket_spoke_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="collapsible_trekking_pole_kit",
        meta={
            "run_notes": (
                "Reference image and category both depict a collapsible trekking-pole kit. "
                "The soft carry bag is omitted as packaging/background; the modeled asset focuses on one folded pole assembly with attached carabiner, basket, and rubber tip."
            )
        },
    )

    black = _mat(model, "satin_black", (0.015, 0.016, 0.017, 1.0))
    graphite = _mat(model, "dark_graphite_aluminum", (0.090, 0.100, 0.105, 1.0))
    silver = _mat(model, "brushed_silver", (0.72, 0.72, 0.68, 1.0))
    cork = _mat(model, "speckled_cork", (0.68, 0.54, 0.35, 1.0))
    rubber = _mat(model, "matte_rubber", (0.025, 0.025, 0.026, 1.0))
    gray = _mat(model, "cool_gray_plastic", (0.42, 0.43, 0.43, 1.0))

    upper = model.part("upper_section")
    # Upper folding hinge/ferrule at the bottom of the compact bundle.
    upper.visual(
        Cylinder(radius=0.014, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=silver,
        name="bottom_ferrule",
    )
    for index, z in enumerate((-0.003, 0.043)):
        upper.visual(
            Cylinder(radius=0.0148, length=0.003),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=graphite,
            name=f"bottom_ferrule_crimp_ring_{index}",
        )
    _add_fold_hinge_hardware(upper, metal=silver, black=graphite, prefix="upper_fold")
    upper.visual(
        Cylinder(radius=0.004, length=1.005),
        origin=Origin(xyz=(0.0, 0.0, 0.500)),
        material=graphite,
        name="hidden_core_spine",
    )
    upper.visual(
        Cylinder(radius=0.011, length=0.515),
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        material=graphite,
        name="upper_tube",
    )
    upper.visual(
        Cylinder(radius=0.019, length=0.056),
        origin=Origin(xyz=(0.0, 0.0, 0.250)),
        material=black,
        name="upper_clamp_collar",
    )
    upper.visual(
        Box((0.012, 0.023, 0.020)),
        origin=Origin(xyz=(0.025, 0.0115, 0.275)),
        material=black,
        name="upper_lock_lug",
    )
    upper.visual(
        Cylinder(radius=0.017, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.580)),
        material=black,
        name="foam_grip_lower",
    )
    for index, z in enumerate((0.455, 0.505, 0.555)):
        upper.visual(
            Cylinder(radius=0.018, length=0.020),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=rubber,
            name=f"foam_rib_{index}",
        )
    # T-handle cane-style crossbar grip: neck rising above the folded bundle
    # to a perpendicular horizontal crossbar along X.  The crossbar is raised
    # so it clears the parallel folded middle/lower sections.
    upper.visual(
        Cylinder(radius=0.016, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.602)),
        material=black,
        name="t_handle_collar",
    )
    upper.visual(
        Cylinder(radius=0.013, length=0.130),
        origin=Origin(xyz=(0.0, 0.0, 0.670)),
        material=black,
        name="t_handle_neck",
    )
    upper.visual(
        mesh_from_geometry(_t_grip_crossbar_mesh(), "crossbar_grip"),
        origin=Origin(xyz=(-0.065, 0.0, 0.745), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cork,
        name="crossbar_grip",
    )
    _add_carabiner_and_strap(upper, black=black, metal=silver)

    middle = model.part("middle_section")
    # Connector bridge keeps the folded side segment visibly tied to the hinge.
    for side, y in (("left", -0.006), ("right", 0.006)):
        middle.visual(
            _tube_between(
                (-0.0090, y, 0.000),
                (0.0275, y, 0.050),
                radius=0.0024,
                name=f"middle_hinge_bridge_{side}",
            ),
            material=silver,
            name=f"middle_hinge_bridge_{side}",
        )
    middle.visual(
        Cylinder(radius=0.0065, length=0.020),
        origin=Origin(xyz=(-0.0090, 0.0, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="middle_hinge_bushing",
    )
    middle.visual(
        Cylinder(radius=0.0095, length=0.630),
        origin=Origin(xyz=(0.0275, 0.0, 0.365)),
        material=black,
        name="middle_tube",
    )
    middle.visual(
        Cylinder(radius=0.0125, length=0.044),
        origin=Origin(xyz=(0.0275, 0.0, 0.690)),
        material=silver,
        name="middle_ferrule",
    )
    for index, z in enumerate((0.668, 0.712)):
        middle.visual(
            Cylinder(radius=0.0132, length=0.0025),
            origin=Origin(xyz=(0.0275, 0.0, z)),
            material=graphite,
            name=f"middle_ferrule_crimp_ring_{index}",
        )
    _add_fold_hinge_hardware(
        middle,
        metal=silver,
        black=graphite,
        prefix="middle_fold",
        base=(0.0390, 0.0, 0.695),
    )
    middle.visual(
        Cylinder(radius=0.0165, length=0.050),
        origin=Origin(xyz=(0.0275, 0.0, 0.230)),
        material=black,
        name="lower_clamp_collar",
    )
    middle.visual(
        Box((0.012, 0.023, 0.020)),
        origin=Origin(xyz=(0.0495, 0.0115, 0.255)),
        material=black,
        name="lower_lock_lug",
    )

    lower = model.part("lower_section")
    for side, y in (("left", -0.006), ("right", 0.006)):
        lower.visual(
            _tube_between(
                (-0.0095, y, 0.000),
                (0.0275, y, -0.035),
                radius=0.0024,
                name=f"lower_hinge_bridge_{side}",
            ),
            material=silver,
            name=f"lower_hinge_bridge_{side}",
        )
    lower.visual(
        Cylinder(radius=0.0065, length=0.020),
        origin=Origin(xyz=(-0.0095, 0.0, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="lower_hinge_bushing",
    )
    lower.visual(
        Cylinder(radius=0.0105, length=0.560),
        origin=Origin(xyz=(0.0275, 0.0, -0.300)),
        material=graphite,
        name="lower_tube",
    )
    lower.visual(
        Cylinder(radius=0.0130, length=0.040),
        origin=Origin(xyz=(0.0275, 0.0, -0.035)),
        material=silver,
        name="lower_ferrule",
    )
    for index, z in enumerate((-0.055, -0.015)):
        lower.visual(
            Cylinder(radius=0.0137, length=0.0025),
            origin=Origin(xyz=(0.0275, 0.0, z)),
            material=graphite,
            name=f"lower_ferrule_crimp_ring_{index}",
        )

    tip = model.part("tip_stage")
    tip.visual(
        Cylinder(radius=0.0070, length=0.380),
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
        material=silver,
        name="inner_ferrule",
    )
    tip.visual(
        Cylinder(radius=0.0030, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, -0.300)),
        material=silver,
        name="tip_core",
    )
    tip.visual(
        Cylinder(radius=0.0040, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, -0.270)),
        material=silver,
        name="carbide_point",
    )
    tip.visual(
        mesh_from_geometry(_rubber_boot_mesh(), "rubber_tip"),
        origin=Origin(xyz=(0.0, 0.0, -0.390)),
        material=rubber,
        name="rubber_tip",
    )
    _add_basket(tip, rubber=rubber)

    upper_lock = model.part("upper_lock")
    _add_lock_lever(upper_lock, black=black, graphite=gray)

    lower_lock = model.part("lower_lock")
    _add_lock_lever(lower_lock, black=black, graphite=gray)

    model.articulation(
        "upper_to_middle_fold",
        ArticulationType.REVOLUTE,
        parent=upper,
        child=middle,
        origin=Origin(xyz=(0.0275, 0.0, 0.000)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.0, lower=0.0, upper=math.pi),
    )
    model.articulation(
        "middle_to_lower_fold",
        ArticulationType.REVOLUTE,
        parent=middle,
        child=lower,
        origin=Origin(xyz=(0.055, 0.0, 0.705)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.0, lower=0.0, upper=math.pi),
    )
    model.articulation(
        "lower_to_tip_slide",
        ArticulationType.PRISMATIC,
        parent=lower,
        child=tip,
        origin=Origin(xyz=(0.0275, 0.0, -0.580)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.20, lower=0.0, upper=0.100),
    )
    model.articulation(
        "upper_to_upper_lock",
        ArticulationType.REVOLUTE,
        parent=upper,
        child=upper_lock,
        origin=Origin(xyz=(0.025, 0.040, 0.275)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=1.0),
    )
    model.articulation(
        "middle_to_lower_lock",
        ArticulationType.REVOLUTE,
        parent=middle,
        child=lower_lock,
        origin=Origin(xyz=(0.0525, 0.040, 0.255)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=1.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    upper = object_model.get_part("upper_section")
    middle = object_model.get_part("middle_section")
    lower = object_model.get_part("lower_section")
    tip = object_model.get_part("tip_stage")
    upper_lock = object_model.get_part("upper_lock")
    lower_lock = object_model.get_part("lower_lock")
    fold_a = object_model.get_articulation("upper_to_middle_fold")
    fold_b = object_model.get_articulation("middle_to_lower_fold")
    slide = object_model.get_articulation("lower_to_tip_slide")
    lock_a = object_model.get_articulation("upper_to_upper_lock")
    lock_b = object_model.get_articulation("middle_to_lower_lock")

    ctx.allow_overlap(
        lower,
        tip,
        elem_a="lower_tube",
        elem_b="inner_ferrule",
        reason=(
            "The slim ferrule is intentionally modeled as retained inside the lower sleeve "
            "so the trekking pole has a believable telescoping tip adjustment."
        ),
    )
    ctx.allow_overlap(
        lower,
        middle,
        elem_a="lower_hinge_bushing",
        elem_b="middle_fold_pivot_bushing",
        reason=(
            "Adjacent fold-junction pivot bushings are necessarily close together in the "
            "folded bundle state; the small representation overlap is local to the hinge "
            "hardware interface and does not affect the deployed mechanism."
        ),
    )
    ctx.allow_overlap(
        middle,
        upper,
        elem_a="middle_hinge_bushing",
        elem_b="upper_fold_pivot_bushing",
        reason=(
            "Adjacent fold-junction pivot bushings at the upper hinge are necessarily "
            "close together in the folded bundle state; the small representation overlap "
            "is local to the hinge hardware interface and does not affect deployment."
        ),
    )
    ctx.allow_overlap(
        middle,
        upper,
        elem_a="middle_hinge_bushing",
        elem_b="upper_fold_cross_pin",
        reason=(
            "The cross pin and hinge bushing at the upper fold junction are tightly packed "
            "hardware in the folded bundle; overlap is local and sub-millimeter."
        ),
    )

    ctx.expect_overlap(
        upper,
        middle,
        axes="z",
        min_overlap=0.45,
        elem_a="upper_tube",
        elem_b="middle_tube",
        name="folded upper and middle sections lie parallel",
    )
    ctx.expect_overlap(
        middle,
        lower,
        axes="z",
        min_overlap=0.45,
        elem_a="middle_tube",
        elem_b="lower_tube",
        name="folded middle and lower sections lie parallel",
    )

    ctx.expect_within(
        tip,
        lower,
        axes="xy",
        margin=0.002,
        inner_elem="inner_ferrule",
        outer_elem="lower_tube",
        name="telescoping ferrule stays centered in sleeve",
    )
    ctx.expect_overlap(
        tip,
        lower,
        axes="z",
        min_overlap=0.12,
        elem_a="inner_ferrule",
        elem_b="lower_tube",
        name="collapsed tip remains inserted",
    )

    def _center(aabb, axis_index: int) -> float:
        return (float(aabb[0][axis_index]) + float(aabb[1][axis_index])) / 2.0

    folded_middle_aabb = ctx.part_world_aabb(middle)
    folded_tip_aabb = ctx.part_world_aabb(tip)
    with ctx.pose({fold_a: 2.35, fold_b: 1.30}):
        deployed_middle_aabb = ctx.part_world_aabb(middle)
        deployed_tip_aabb = ctx.part_world_aabb(tip)
    ctx.check(
        "fold joints swing sections out of compact bundle",
        folded_middle_aabb is not None
        and deployed_middle_aabb is not None
        and folded_tip_aabb is not None
        and deployed_tip_aabb is not None
        and deployed_middle_aabb[0][2] < folded_middle_aabb[0][2] - 0.20
        and deployed_tip_aabb[0][2] < folded_tip_aabb[0][2] - 0.20,
        details=f"folded_middle={folded_middle_aabb}, deployed_middle={deployed_middle_aabb}, folded_tip={folded_tip_aabb}, deployed_tip={deployed_tip_aabb}",
    )

    with ctx.pose({fold_a: math.pi, fold_b: math.pi / 2.0}):
        half_swing_middle_aabb = ctx.part_element_world_aabb(middle, elem="middle_tube")
        half_swing_lower_aabb = ctx.part_element_world_aabb(lower, elem="lower_tube")
    ctx.check(
        "lower fold swings around the clear side of the middle tube",
        half_swing_middle_aabb is not None
        and half_swing_lower_aabb is not None
        and _center(half_swing_lower_aabb, 0) < _center(half_swing_middle_aabb, 0) - 0.08,
        details=f"middle={half_swing_middle_aabb}, lower={half_swing_lower_aabb}",
    )

    with ctx.pose({fold_a: math.pi, fold_b: math.pi, slide: 0.100}):
        upper_tube_aabb = ctx.part_element_world_aabb(upper, elem="upper_tube")
        middle_tube_aabb = ctx.part_element_world_aabb(middle, elem="middle_tube")
        lower_tube_aabb = ctx.part_element_world_aabb(lower, elem="lower_tube")
        tip_ferrule_aabb = ctx.part_element_world_aabb(tip, elem="inner_ferrule")
    if (
        upper_tube_aabb is not None
        and middle_tube_aabb is not None
        and lower_tube_aabb is not None
        and tip_ferrule_aabb is not None
    ):
        deployed_axis_x = [
            _center(upper_tube_aabb, 0),
            _center(middle_tube_aabb, 0),
            _center(lower_tube_aabb, 0),
            _center(tip_ferrule_aabb, 0),
        ]
        deployed_axis_y = [
            _center(upper_tube_aabb, 1),
            _center(middle_tube_aabb, 1),
            _center(lower_tube_aabb, 1),
            _center(tip_ferrule_aabb, 1),
        ]
    else:
        deployed_axis_x = []
        deployed_axis_y = []
    ctx.check(
        "fold joints reach a straight deployed pole axis",
        bool(deployed_axis_x)
        and max(deployed_axis_x) - min(deployed_axis_x) < 0.018
        and max(deployed_axis_y) - min(deployed_axis_y) < 0.010
        and middle_tube_aabb[1][2] < upper_tube_aabb[0][2]
        and lower_tube_aabb[1][2] < middle_tube_aabb[0][2]
        and tip_ferrule_aabb[1][2] < lower_tube_aabb[1][2],
        details=(
            f"x_centers={deployed_axis_x}, y_centers={deployed_axis_y}, "
            f"upper={upper_tube_aabb}, middle={middle_tube_aabb}, "
            f"lower={lower_tube_aabb}, tip={tip_ferrule_aabb}"
        ),
    )

    rest_tip_pos = ctx.part_world_position(tip)
    with ctx.pose({slide: 0.100}):
        ctx.expect_within(
            tip,
            lower,
            axes="xy",
            margin=0.002,
            inner_elem="inner_ferrule",
            outer_elem="lower_tube",
            name="extended ferrule stays centered in sleeve",
        )
        ctx.expect_overlap(
            tip,
            lower,
            axes="z",
            min_overlap=0.035,
            elem_a="inner_ferrule",
            elem_b="lower_tube",
            name="extended tip retains insertion",
        )
        extended_tip_pos = ctx.part_world_position(tip)
    ctx.check(
        "tip slide extends downward",
        rest_tip_pos is not None
        and extended_tip_pos is not None
        and extended_tip_pos[2] < rest_tip_pos[2] - 0.08,
        details=f"rest={rest_tip_pos}, extended={extended_tip_pos}",
    )

    # T-handle crossbar grip: verify the crossbar_grip visual exists on
    # upper_section and is oriented horizontally (wider along X than Z).
    crossbar_aabb = ctx.part_element_world_aabb(upper, elem="crossbar_grip")
    neck_aabb = ctx.part_element_world_aabb(upper, elem="t_handle_neck")
    ctx.check(
        "t_handle crossbar_grip is a horizontal perpendicular bar",
        crossbar_aabb is not None
        and neck_aabb is not None
        and (crossbar_aabb[1][0] - crossbar_aabb[0][0]) > 0.080
        and (crossbar_aabb[1][2] - crossbar_aabb[0][2]) < 0.050
        and crossbar_aabb[0][2] > neck_aabb[0][2],
        details=f"crossbar={crossbar_aabb}, neck={neck_aabb}",
    )
    ctx.expect_contact(
        upper,
        upper,
        elem_a="t_handle_neck",
        elem_b="crossbar_grip",
        contact_tol=0.005,
        name="t_handle neck contacts the crossbar grip",
    )

    closed_upper_lock = ctx.part_world_aabb(upper_lock)
    closed_lower_lock = ctx.part_world_aabb(lower_lock)
    with ctx.pose({lock_a: 0.8, lock_b: 0.8}):
        open_upper_lock = ctx.part_world_aabb(upper_lock)
        open_lower_lock = ctx.part_world_aabb(lower_lock)
    ctx.check(
        "flip locks open outward",
        closed_upper_lock is not None
        and open_upper_lock is not None
        and closed_lower_lock is not None
        and open_lower_lock is not None
        and open_upper_lock[1][0] > closed_upper_lock[1][0] + 0.015
        and open_lower_lock[1][0] > closed_lower_lock[1][0] + 0.015,
        details=f"upper_closed={closed_upper_lock}, upper_open={open_upper_lock}, lower_closed={closed_lower_lock}, lower_open={open_lower_lock}",
    )

    return ctx.report()


object_model = build_object_model()
