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

# ---------------------------------------------------------------------------
# Fold-chain design parameters (N=4 Z-fold shock-corded collapsible pole)
# ---------------------------------------------------------------------------
NUM_FOLD_SECTIONS = 4
DX = 0.014  # X offset of tube centre from section frame origin
JOINT_DX = 2 * DX  # X offset of next fold joint from section frame origin
TUBE_LENGTH = 0.220
FERRULE_LENGTH = 0.036

# Goes-up sections (even indices: 0, 2)
UP_TUBE_START = 0.050
UP_TUBE_END = UP_TUBE_START + TUBE_LENGTH  # 0.270
UP_JOINT_Z = UP_TUBE_END + FERRULE_LENGTH + 0.014  # 0.320

# Goes-down sections (odd indices: 1, 3)
DN_TUBE_END = -0.020
DN_TUBE_START = DN_TUBE_END - TUBE_LENGTH  # -0.240
DN_JOINT_Z = DN_TUBE_START - 0.020  # -0.260

TUBE_RADII = (0.0100, 0.0095, 0.0090, 0.0085)
FERRULE_RADII = (0.0130, 0.0125, 0.0120, 0.0115)


def _mat(model: ArticulatedObject, name: str, rgba: tuple[float, float, float, float]) -> Material:
    return model.material(name, rgba=rgba)


def _ergonomic_handle_mesh() -> MeshGeometry:
    """Lathed cork grip with a flared palm swell and a hook-like top cap seat."""
    profile = [
        (0.020, 0.000),
        (0.023, 0.020),
        (0.027, 0.090),
        (0.031, 0.180),
        (0.029, 0.270),
        (0.023, 0.335),
        (0.019, 0.365),
    ]
    return LatheGeometry(profile, segments=40, closed=True)


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
    webbing = sweep_profile_along_spline(
        [
            (-0.010, 0.004, 0.700),
            (-0.065, 0.012, 0.630),
            (-0.105, 0.010, 0.490),
            (-0.062, 0.007, 0.385),
            (-0.014, 0.004, 0.610),
        ],
        profile=rounded_rect_profile(0.012, 0.003, radius=0.0012, corner_segments=4),
        samples_per_segment=12,
        closed_spline=True,
        cap_profile=True,
    )
    part.visual(mesh_from_geometry(webbing, "wrist_webbing"), material=black, name="wrist_webbing")
    part.visual(
        Box((0.030, 0.012, 0.026)),
        origin=Origin(xyz=(-0.010, 0.004, 0.675)),
        material=black,
        name="strap_anchor",
    )

    carabiner = tube_from_spline_points(
        [
            (-0.150, 0.012, 0.475),
            (-0.198, 0.012, 0.440),
            (-0.205, 0.012, 0.360),
            (-0.158, 0.012, 0.310),
            (-0.103, 0.012, 0.330),
            (-0.093, 0.012, 0.420),
            (-0.120, 0.012, 0.475),
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
            (-0.118, 0.012, 0.452),
            (-0.102, 0.012, 0.340),
            radius=0.0023,
            name="carabiner_gate",
        ),
        material=metal,
        name="carabiner_gate",
    )
    part.visual(
        _tube_between(
            (-0.077, 0.010, 0.445),
            (-0.127, 0.012, 0.472),
            radius=0.0030,
            name="carabiner_tether",
        ),
        material=black,
        name="carabiner_tether",
    )


def _add_lock_lever(part, *, black, graphite) -> None:
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


def _build_fold_section(
    model: ArticulatedObject,
    section_index: int,
    *,
    silver: Material,
    graphite: Material,
    black: Material,
    add_hinge: bool = True,
    add_clamp: bool = False,
) -> None:
    """Create one fold tube section part with bridges, tube, ferrule, and optional hardware."""
    name = f"fold_section_{section_index}"
    section = model.part(name)
    prefix = f"s{section_index}"
    goes_up = section_index % 2 == 0
    tube_r = TUBE_RADII[section_index]
    ferrule_r = FERRULE_RADII[section_index]

    # --- Connector bridges and hinge bushing at section origin (hinge side) ---
    # Bridge endpoints extend well into the tube body to ensure geometry connectivity.
    bridge_z_end = (UP_TUBE_START + 0.025) if goes_up else (DN_TUBE_END - 0.025)
    for side, y in (("left", -0.006), ("right", 0.006)):
        section.visual(
            _tube_between(
                (-DX * 0.65, y, 0.000),
                (DX, y, bridge_z_end),
                radius=0.0024,
                name=f"{prefix}_hinge_bridge_{side}",
            ),
            material=silver,
            name=f"{prefix}_hinge_bridge_{side}",
        )
    section.visual(
        Cylinder(radius=0.0065, length=0.020),
        origin=Origin(xyz=(-DX * 0.65, 0.0, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name=f"{prefix}_hinge_bushing",
    )

    if goes_up:
        # Tube extends upward (+Z) from hinge
        tube_cz = UP_TUBE_START + TUBE_LENGTH / 2.0
        section.visual(
            Cylinder(radius=tube_r, length=TUBE_LENGTH),
            origin=Origin(xyz=(DX, 0.0, tube_cz)),
            material=graphite,
            name=f"{prefix}_tube",
        )
        section.visual(
            Cylinder(radius=ferrule_r, length=FERRULE_LENGTH),
            origin=Origin(xyz=(DX, 0.0, UP_TUBE_END + FERRULE_LENGTH / 2.0)),
            material=silver,
            name=f"{prefix}_ferrule",
        )
        for idx, z in enumerate((UP_TUBE_END + 0.002, UP_TUBE_END + FERRULE_LENGTH - 0.002)):
            section.visual(
                Cylinder(radius=ferrule_r + 0.0007, length=0.0025),
                origin=Origin(xyz=(DX, 0.0, z)),
                material=graphite,
                name=f"{prefix}_crimp_ring_{idx}",
            )
        if add_hinge:
            # Base overlaps with ferrule body so geometry connectivity is maintained
            hinge_bz = UP_TUBE_END + FERRULE_LENGTH * 0.25
            _add_fold_hinge_hardware(
                section, metal=silver, black=graphite,
                prefix=f"{prefix}_fold",
                base=(JOINT_DX - 0.016, 0.0, hinge_bz),
            )
        if add_clamp:
            cz = UP_TUBE_START + TUBE_LENGTH * 0.55
            section.visual(
                Cylinder(radius=tube_r + 0.006, length=0.040),
                origin=Origin(xyz=(DX, 0.0, cz)),
                material=black,
                name=f"{prefix}_clamp_collar",
            )
            section.visual(
                Box((0.012, 0.023, 0.020)),
                origin=Origin(xyz=(DX + 0.016, 0.0115, cz + 0.022)),
                material=black,
                name=f"{prefix}_lock_lug",
            )
    else:
        # Tube extends downward (-Z) from hinge
        tube_cz = DN_TUBE_END - TUBE_LENGTH / 2.0
        section.visual(
            Cylinder(radius=tube_r, length=TUBE_LENGTH),
            origin=Origin(xyz=(DX, 0.0, tube_cz)),
            material=graphite,
            name=f"{prefix}_tube",
        )
        section.visual(
            Cylinder(radius=ferrule_r, length=FERRULE_LENGTH),
            origin=Origin(xyz=(DX, 0.0, DN_TUBE_END - FERRULE_LENGTH / 2.0 + 0.008)),
            material=silver,
            name=f"{prefix}_ferrule",
        )
        for idx, z in enumerate((DN_TUBE_END - 0.002, DN_TUBE_END - FERRULE_LENGTH + 0.002)):
            section.visual(
                Cylinder(radius=ferrule_r + 0.0007, length=0.0025),
                origin=Origin(xyz=(DX, 0.0, z)),
                material=graphite,
                name=f"{prefix}_crimp_ring_{idx}",
            )
        if add_hinge:
            # Base overlaps with tube body so geometry connectivity is maintained
            hinge_bz = DN_TUBE_START + 0.020
            _add_fold_hinge_hardware(
                section, metal=silver, black=graphite,
                prefix=f"{prefix}_fold",
                base=(JOINT_DX - 0.016, 0.0, hinge_bz),
            )
        if add_clamp:
            cz = DN_TUBE_END - TUBE_LENGTH * 0.40
            section.visual(
                Cylinder(radius=tube_r + 0.006, length=0.040),
                origin=Origin(xyz=(DX, 0.0, cz)),
                material=black,
                name=f"{prefix}_clamp_collar",
            )
            section.visual(
                Box((0.012, 0.023, 0.020)),
                origin=Origin(xyz=(DX + 0.016, 0.0115, cz - 0.022)),
                material=black,
                name=f"{prefix}_lock_lug",
            )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="collapsible_trekking_pole_kit",
        meta={
            "run_notes": (
                "4-segment Z-fold shock-corded collapsible trekking pole (N=4 fold sections). "
                "Reference image and category both depict a collapsible trekking-pole kit. "
                "The soft carry bag is omitted as packaging/background; the modeled asset "
                "focuses on one folded pole assembly with attached carabiner, basket, and rubber tip."
            )
        },
    )

    black = _mat(model, "satin_black", (0.015, 0.016, 0.017, 1.0))
    graphite = _mat(model, "dark_graphite_aluminum", (0.090, 0.100, 0.105, 1.0))
    silver = _mat(model, "brushed_silver", (0.72, 0.72, 0.68, 1.0))
    cork = _mat(model, "speckled_cork", (0.68, 0.54, 0.35, 1.0))
    rubber = _mat(model, "matte_rubber", (0.025, 0.025, 0.026, 1.0))
    gray = _mat(model, "cool_gray_plastic", (0.42, 0.43, 0.43, 1.0))

    # -----------------------------------------------------------------------
    # Upper section (root): handle, grip, strap, carabiner, top fold hinge
    # -----------------------------------------------------------------------
    upper = model.part("upper_section")
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
        Cylinder(radius=0.004, length=0.750),
        origin=Origin(xyz=(0.0, 0.0, 0.370)),
        material=graphite,
        name="hidden_core_spine",
    )
    upper.visual(
        Cylinder(radius=0.011, length=0.300),
        origin=Origin(xyz=(0.0, 0.0, 0.195)),
        material=graphite,
        name="upper_tube",
    )
    upper.visual(
        Cylinder(radius=0.017, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=black,
        name="upper_clamp_collar",
    )
    upper.visual(
        Box((0.012, 0.023, 0.020)),
        origin=Origin(xyz=(-0.022, 0.0115, 0.175)),
        material=black,
        name="upper_lock_lug",
    )
    upper.visual(
        Cylinder(radius=0.017, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.380)),
        material=black,
        name="foam_grip_lower",
    )
    for index, z in enumerate((0.310, 0.345, 0.380)):
        upper.visual(
            Cylinder(radius=0.018, length=0.020),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=rubber,
            name=f"foam_rib_{index}",
        )
    upper.visual(
        mesh_from_geometry(_ergonomic_handle_mesh(), "cork_handle"),
        origin=Origin(xyz=(0.0, 0.0, 0.400)),
        material=cork,
        name="cork_handle",
    )
    upper.visual(
        Cylinder(radius=0.025, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.770)),
        material=gray,
        name="top_cap_shell",
    )
    upper.visual(
        Box((0.050, 0.030, 0.030)),
        origin=Origin(xyz=(0.012, 0.0, 0.808), rpy=(0.0, -0.38, 0.0)),
        material=black,
        name="palm_hook",
    )
    _add_carabiner_and_strap(upper, black=black, metal=silver)

    # -----------------------------------------------------------------------
    # Fold sections (N=4) created in a loop with indexed names
    # -----------------------------------------------------------------------
    for i in range(NUM_FOLD_SECTIONS):
        is_last = i == NUM_FOLD_SECTIONS - 1
        _build_fold_section(
            model,
            i,
            silver=silver,
            graphite=graphite,
            black=black,
            add_hinge=not is_last,
            add_clamp=(i == 1),  # clamp on fold_section_1 (mid-chain)
        )

    # -----------------------------------------------------------------------
    # Tip stage: telescoping tip with basket and rubber boot
    # -----------------------------------------------------------------------
    tip = model.part("tip_stage")
    tip.visual(
        Cylinder(radius=0.0060, length=0.300),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=silver,
        name="inner_ferrule",
    )
    tip.visual(
        Cylinder(radius=0.0030, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, -0.180)),
        material=silver,
        name="tip_core",
    )
    tip.visual(
        Cylinder(radius=0.0040, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.200)),
        material=silver,
        name="carbide_point",
    )
    tip.visual(
        mesh_from_geometry(_rubber_boot_mesh(), "rubber_tip"),
        origin=Origin(xyz=(0.0, 0.0, -0.290)),
        material=rubber,
        name="rubber_tip",
    )
    _add_basket(tip, rubber=rubber)

    # -----------------------------------------------------------------------
    # Lock lever parts
    # -----------------------------------------------------------------------
    upper_lock = model.part("upper_lock")
    _add_lock_lever(upper_lock, black=black, graphite=gray)

    middle_lock = model.part("middle_lock")
    _add_lock_lever(middle_lock, black=black, graphite=gray)

    # -----------------------------------------------------------------------
    # Articulations: fold chain + tip slide + lock levers
    # -----------------------------------------------------------------------
    # Fold joints alternate axis sign for Z-fold pattern
    fold_axis_signs = [1.0, -1.0, 1.0, -1.0]
    fold_sections = [model.get_part(f"fold_section_{i}") for i in range(NUM_FOLD_SECTIONS)]

    # Joint 0: upper → fold_section_0
    model.articulation(
        "fold_joint_0",
        ArticulationType.REVOLUTE,
        parent=upper,
        child=fold_sections[0],
        origin=Origin(xyz=(DX, 0.0, 0.0)),
        axis=(0.0, fold_axis_signs[0], 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.0, lower=0.0, upper=math.pi),
    )

    # Joints 1..N-1: fold_section_{i-1} → fold_section_{i}
    for i in range(1, NUM_FOLD_SECTIONS):
        parent_section = fold_sections[i - 1]
        child_section = fold_sections[i]
        prev_goes_up = (i - 1) % 2 == 0
        joint_z = UP_JOINT_Z if prev_goes_up else DN_JOINT_Z
        model.articulation(
            f"fold_joint_{i}",
            ArticulationType.REVOLUTE,
            parent=parent_section,
            child=child_section,
            origin=Origin(xyz=(JOINT_DX, 0.0, joint_z)),
            axis=(0.0, fold_axis_signs[i], 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=1.0, lower=0.0, upper=math.pi),
        )

    # Tip slide: fold_section_3 → tip_stage
    last_section = fold_sections[-1]
    model.articulation(
        "tip_slide",
        ArticulationType.PRISMATIC,
        parent=last_section,
        child=tip,
        origin=Origin(xyz=(DX, 0.0, DN_TUBE_START)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.20, lower=0.0, upper=0.080),
    )

    # Upper lock lever (moved to -X side to avoid fold section tube)
    model.articulation(
        "upper_to_upper_lock",
        ArticulationType.REVOLUTE,
        parent=upper,
        child=upper_lock,
        origin=Origin(xyz=(-0.022, 0.040, 0.175)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=1.0),
    )

    # Middle lock lever on fold_section_1
    s1 = fold_sections[1]
    model.articulation(
        "section_1_to_middle_lock",
        ArticulationType.REVOLUTE,
        parent=s1,
        child=middle_lock,
        origin=Origin(xyz=(DX + 0.016, 0.040, DN_TUBE_END - TUBE_LENGTH * 0.40 - 0.022)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    upper = object_model.get_part("upper_section")
    tip = object_model.get_part("tip_stage")
    upper_lock = object_model.get_part("upper_lock")
    middle_lock = object_model.get_part("middle_lock")

    fold_sections = [object_model.get_part(f"fold_section_{i}") for i in range(NUM_FOLD_SECTIONS)]
    fold_joints = [object_model.get_articulation(f"fold_joint_{i}") for i in range(NUM_FOLD_SECTIONS)]
    slide = object_model.get_articulation("tip_slide")
    lock_a = object_model.get_articulation("upper_to_upper_lock")
    lock_b = object_model.get_articulation("section_1_to_middle_lock")

    last_section = fold_sections[-1]

    # --- Verify 4 fold sections exist (structural delta assertion) ---
    ctx.check(
        "four fold sections present in Z-fold chain",
        all(
            object_model.get_part(f"fold_section_{i}") is not None
            for i in range(NUM_FOLD_SECTIONS)
        ),
        details=f"expected {NUM_FOLD_SECTIONS} fold sections",
    )
    ctx.check(
        "four fold revolute joints present",
        all(
            object_model.get_articulation(f"fold_joint_{i}") is not None
            for i in range(NUM_FOLD_SECTIONS)
        ),
        details=f"expected {NUM_FOLD_SECTIONS} fold joints",
    )

    # --- Overlap allowance: tip ferrule retained inside last fold section tube ---
    ctx.allow_overlap(
        last_section,
        tip,
        elem_a=f"s{NUM_FOLD_SECTIONS - 1}_tube",
        elem_b="inner_ferrule",
        reason=(
            "The slim ferrule is intentionally modeled as retained inside the last fold "
            "section sleeve so the trekking pole has a believable telescoping tip adjustment."
        ),
    )
    ctx.allow_overlap(
        last_section,
        tip,
        elem_a=f"s{NUM_FOLD_SECTIONS - 1}_ferrule",
        elem_b="inner_ferrule",
        reason=(
            "The tip inner ferrule passes through the section end ferrule as part of "
            "the telescoping retention mechanism."
        ),
    )

    # --- Overlap allowances: hinge pivot bushings embedded in adjacent hinge bushings ---
    # Each fold joint has a pivot bushing on the parent end and a hinge bushing on the
    # child end that are intentionally co-located to form the physical hinge mechanism.
    ctx.allow_overlap(
        upper,
        fold_sections[0],
        elem_a="bottom_ferrule",
        elem_b="s0_hinge_bushing",
        reason=(
            "The hinge bushing on fold_section_0 is intentionally embedded in the upper "
            "bottom ferrule to form the first fold pivot."
        ),
    )
    for i in range(NUM_FOLD_SECTIONS - 1):
        parent_sec = fold_sections[i]
        child_sec = fold_sections[i + 1]
        # Allow pivot bushing ↔ hinge bushing overlap at the hinge mechanism
        ctx.allow_overlap(
            parent_sec,
            child_sec,
            elem_a=f"s{i}_fold_pivot_bushing",
            elem_b=f"s{i+1}_hinge_bushing",
            reason=(
                f"The pivot bushing on fold_section_{i} and hinge bushing on "
                f"fold_section_{i+1} are co-located to form the fold hinge mechanism."
            ),
        )
        # Allow pivot bushing ↔ ferrule overlap (hinge hardware close to ferrule)
        ctx.allow_overlap(
            parent_sec,
            child_sec,
            elem_a=f"s{i}_fold_pivot_bushing",
            elem_b=f"s{i+1}_ferrule",
            reason=(
                f"The pivot bushing on fold_section_{i} is intentionally close to the "
                f"ferrule on fold_section_{i+1} at the hinge point when folded."
            ),
        )
        # Allow pivot bushing ↔ tube overlap (hinge hardware close to adjacent tube)
        ctx.allow_overlap(
            parent_sec,
            child_sec,
            elem_a=f"s{i}_fold_pivot_bushing",
            elem_b=f"s{i+1}_tube",
            reason=(
                f"The pivot bushing on fold_section_{i} sits close to the tube on "
                f"fold_section_{i+1} at the hinge connection when folded parallel."
            ),
        )

    # Allow hinge bushing ↔ core spine overlap at the upper-to-section-0 pivot
    ctx.allow_overlap(
        fold_sections[0],
        upper,
        elem_a="s0_hinge_bushing",
        elem_b="hidden_core_spine",
        reason=(
            "The hinge bushing on fold_section_0 sits close to the internal shock-cord "
            "core spine at the upper pivot connection."
        ),
    )
    # Allow hinge bushing ↔ upper pivot bushing overlap at the upper-to-section-0 pivot
    ctx.allow_overlap(
        fold_sections[0],
        upper,
        elem_a="s0_hinge_bushing",
        elem_b="upper_fold_pivot_bushing",
        reason=(
            "The hinge bushing on fold_section_0 and the pivot bushing on the upper "
            "section are co-located to form the first fold hinge mechanism."
        ),
    )

    # --- Folded: adjacent sections lie parallel (Z overlap) ---
    tube_names = [f"s{i}_tube" for i in range(NUM_FOLD_SECTIONS)]
    # upper_tube overlaps with fold_section_0 tube
    ctx.expect_overlap(
        upper,
        fold_sections[0],
        axes="z",
        min_overlap=0.12,
        elem_a="upper_tube",
        elem_b=tube_names[0],
        name="folded upper and fold_section_0 lie parallel",
    )
    for i in range(NUM_FOLD_SECTIONS - 1):
        ctx.expect_overlap(
            fold_sections[i],
            fold_sections[i + 1],
            axes="z",
            min_overlap=0.12,
            elem_a=tube_names[i],
            elem_b=tube_names[i + 1],
            name=f"folded fold_section_{i} and fold_section_{i+1} lie parallel",
        )

    # --- Tip containment and insertion ---
    ctx.expect_within(
        tip,
        last_section,
        axes="xy",
        margin=0.003,
        inner_elem="inner_ferrule",
        outer_elem=tube_names[-1],
        name="telescoping ferrule stays centered in sleeve",
    )
    ctx.expect_overlap(
        tip,
        last_section,
        axes="z",
        min_overlap=0.08,
        elem_a="inner_ferrule",
        elem_b=tube_names[-1],
        name="collapsed tip remains inserted",
    )

    # --- Deployed pose: fold joints swing sections out and align ---
    def _center(aabb_val, axis_index: int) -> float:
        return (float(aabb_val[0][axis_index]) + float(aabb_val[1][axis_index])) / 2.0

    # Check that fold joints swing sections out of the compact bundle
    folded_s0_aabb = ctx.part_world_aabb(fold_sections[0])
    folded_tip_aabb = ctx.part_world_aabb(tip)
    deployed_pose = {fold_joints[i]: math.pi for i in range(NUM_FOLD_SECTIONS)}
    with ctx.pose(deployed_pose):
        deployed_s0_aabb = ctx.part_world_aabb(fold_sections[0])
        deployed_tip_aabb = ctx.part_world_aabb(tip)
    ctx.check(
        "fold joints swing sections out of compact bundle",
        folded_s0_aabb is not None
        and deployed_s0_aabb is not None
        and folded_tip_aabb is not None
        and deployed_tip_aabb is not None
        and deployed_s0_aabb[0][2] < folded_s0_aabb[0][2] - 0.10
        and deployed_tip_aabb[0][2] < folded_tip_aabb[0][2] - 0.10,
        details=f"folded_s0={folded_s0_aabb}, deployed_s0={deployed_s0_aabb}, folded_tip={folded_tip_aabb}, deployed_tip={deployed_tip_aabb}",
    )

    # Deployed alignment: all tubes on a straight axis
    full_deployed_pose = {fold_joints[i]: math.pi for i in range(NUM_FOLD_SECTIONS)}
    full_deployed_pose[slide] = 0.060
    with ctx.pose(full_deployed_pose):
        upper_tube_aabb = ctx.part_element_world_aabb(upper, elem="upper_tube")
        section_tube_aabbs = [
            ctx.part_element_world_aabb(fold_sections[i], elem=tube_names[i])
            for i in range(NUM_FOLD_SECTIONS)
        ]
        tip_ferrule_aabb = ctx.part_element_world_aabb(tip, elem="inner_ferrule")

    all_aabbs = [upper_tube_aabb] + section_tube_aabbs + [tip_ferrule_aabb]
    if all(a is not None for a in all_aabbs):
        deployed_axis_x = [_center(a, 0) for a in all_aabbs]
        deployed_axis_y = [_center(a, 1) for a in all_aabbs]
    else:
        deployed_axis_x = []
        deployed_axis_y = []

    # Check Z stacking: each fold section below the previous (excluding tip,
    # which telescopes inside the last section and intentionally overlaps in Z)
    pole_sections = [upper_tube_aabb] + section_tube_aabbs
    z_stack_ok = True
    if all(a is not None for a in pole_sections):
        for k in range(len(pole_sections) - 1):
            if pole_sections[k + 1][1][2] >= pole_sections[k][0][2]:
                z_stack_ok = False
                break
    # Tip should extend below the last section's center (telescoping is OK)
    tip_below_last = True
    if tip_ferrule_aabb is not None and section_tube_aabbs[-1] is not None:
        last_center_z = (section_tube_aabbs[-1][0][2] + section_tube_aabbs[-1][1][2]) / 2.0
        tip_center_z = (tip_ferrule_aabb[0][2] + tip_ferrule_aabb[1][2]) / 2.0
        tip_below_last = tip_center_z < last_center_z

    ctx.check(
        "fold joints reach a straight deployed pole axis",
        bool(deployed_axis_x)
        and max(deployed_axis_x) - min(deployed_axis_x) < 0.020
        and max(deployed_axis_y) - min(deployed_axis_y) < 0.012
        and z_stack_ok
        and tip_below_last,
        details=(
            f"x_centers={[round(x, 4) for x in deployed_axis_x]}, "
            f"y_centers={[round(y, 4) for y in deployed_axis_y]}, "
            f"upper={upper_tube_aabb}, sections={section_tube_aabbs}, tip={tip_ferrule_aabb}"
        ),
    )

    # --- Tip slide extends downward ---
    rest_tip_pos = ctx.part_world_position(tip)
    with ctx.pose({slide: 0.080}):
        ctx.expect_within(
            tip,
            last_section,
            axes="xy",
            margin=0.003,
            inner_elem="inner_ferrule",
            outer_elem=tube_names[-1],
            name="extended ferrule stays centered in sleeve",
        )
        ctx.expect_overlap(
            tip,
            last_section,
            axes="z",
            min_overlap=0.025,
            elem_a="inner_ferrule",
            elem_b=tube_names[-1],
            name="extended tip retains insertion",
        )
        extended_tip_pos = ctx.part_world_position(tip)
    ctx.check(
        "tip slide extends downward",
        rest_tip_pos is not None
        and extended_tip_pos is not None
        and extended_tip_pos[2] < rest_tip_pos[2] - 0.06,
        details=f"rest={rest_tip_pos}, extended={extended_tip_pos}",
    )

    # --- Flip locks open outward ---
    closed_upper_lock = ctx.part_world_aabb(upper_lock)
    closed_middle_lock = ctx.part_world_aabb(middle_lock)
    with ctx.pose({lock_a: 0.8, lock_b: 0.8}):
        open_upper_lock = ctx.part_world_aabb(upper_lock)
        open_middle_lock = ctx.part_world_aabb(middle_lock)
    ctx.check(
        "flip locks open outward",
        closed_upper_lock is not None
        and open_upper_lock is not None
        and closed_middle_lock is not None
        and open_middle_lock is not None
        and open_upper_lock[0][0] < closed_upper_lock[0][0] - 0.012
        and open_middle_lock[1][0] > closed_middle_lock[1][0] + 0.012,
        details=(
            f"upper_closed={closed_upper_lock}, upper_open={open_upper_lock}, "
            f"middle_closed={closed_middle_lock}, middle_open={open_middle_lock}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
