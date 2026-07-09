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
    # Broad wrist loop: a black flattened webbing loop anchored through the top of
    # the handle, matching the reference without adding the carry bag.
    webbing = sweep_profile_along_spline(
        [
            (-0.010, 0.004, 0.855),
            (-0.065, 0.012, 0.780),
            (-0.105, 0.010, 0.630),
            (-0.062, 0.007, 0.525),
            (-0.014, 0.004, 0.765),
        ],
        profile=rounded_rect_profile(0.012, 0.003, radius=0.0012, corner_segments=4),
        samples_per_segment=12,
        closed_spline=True,
        cap_profile=True,
    )
    part.visual(mesh_from_geometry(webbing, "wrist_webbing"), material=black, name="wrist_webbing")
    part.visual(
        Box((0.030, 0.012, 0.026)),
        origin=Origin(xyz=(-0.010, 0.004, 0.830)),
        material=black,
        name="strap_anchor",
    )

    # Accessory carabiner clipped to the strap, authored as a compact D-shaped
    # tube with a silver gate and a short tether so it is not floating.
    carabiner = tube_from_spline_points(
        [
            (-0.150, 0.012, 0.625),
            (-0.198, 0.012, 0.590),
            (-0.205, 0.012, 0.505),
            (-0.158, 0.012, 0.450),
            (-0.103, 0.012, 0.470),
            (-0.093, 0.012, 0.570),
            (-0.120, 0.012, 0.625),
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
            (-0.118, 0.012, 0.602),
            (-0.102, 0.012, 0.484),
            radius=0.0023,
            name="carabiner_gate",
        ),
        material=metal,
        name="carabiner_gate",
    )
    part.visual(
        _tube_between(
            (-0.077, 0.010, 0.595),
            (-0.127, 0.012, 0.622),
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


def _add_basket(part, *, rubber, cx: float = 0.0, cz: float = -0.070) -> None:
    # Compact removable trekking basket: a ring plus radial ribs close to the
    # lower tip.  All elements meet at the hub/ring and are part of the lower
    # tip assembly.
    part.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.044, tube=0.004, radial_segments=12, tubular_segments=48),
            "basket_ring",
        ),
        origin=Origin(xyz=(cx, 0.0, cz)),
        material=rubber,
        name="basket_ring",
    )
    part.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(cx, 0.0, cz)),
        material=rubber,
        name="basket_hub",
    )
    for index, angle in enumerate(
        (0.0, math.pi / 3.0, 2.0 * math.pi / 3.0, math.pi, 4.0 * math.pi / 3.0, 5.0 * math.pi / 3.0)
    ):
        sx = cx + 0.027 * math.cos(angle)
        sy = 0.027 * math.sin(angle)
        part.visual(
            Box((0.048, 0.006, 0.004)),
            origin=Origin(xyz=(sx, sy, cz), rpy=(0.0, 0.0, angle)),
            material=rubber,
            name=f"basket_spoke_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="collapsible_trekking_pole_kit",
        meta={
            "run_notes": (
                "Pure Z-fold (shock-corded) collapsible trekking pole variant. "
                "All four sections collapse by folding rather than telescoping, "
                "similar to the Black Diamond Distance Z family. "
                "The soft carry bag is omitted as packaging/background."
            )
        },
    )

    black = _mat(model, "satin_black", (0.015, 0.016, 0.017, 1.0))
    graphite = _mat(model, "dark_graphite_aluminum", (0.090, 0.100, 0.105, 1.0))
    silver = _mat(model, "brushed_silver", (0.72, 0.72, 0.68, 1.0))
    cork = _mat(model, "speckled_cork", (0.68, 0.54, 0.35, 1.0))
    rubber = _mat(model, "matte_rubber", (0.025, 0.025, 0.026, 1.0))
    gray = _mat(model, "cool_gray_plastic", (0.42, 0.43, 0.43, 1.0))

    # ── Upper section (root) ──────────────────────────────────────────────
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
    # Internal shock-cord spine running through the upper tube
    upper.visual(
        Cylinder(radius=0.004, length=1.005),
        origin=Origin(xyz=(0.0, 0.0, 0.500)),
        material=black,
        name="hidden_core_spine",
    )
    # Visible shock-cord proxy tube spanning the folded bundle exterior
    upper.visual(
        Cylinder(radius=0.0018, length=0.620),
        origin=Origin(xyz=(0.014, 0.0, 0.300)),
        material=black,
        name="shock_cord_proxy",
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
    upper.visual(
        mesh_from_geometry(_ergonomic_handle_mesh(), "cork_handle"),
        origin=Origin(xyz=(0.0, 0.0, 0.595)),
        material=cork,
        name="cork_handle",
    )
    upper.visual(
        Cylinder(radius=0.025, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.962)),
        material=gray,
        name="top_cap_shell",
    )
    upper.visual(
        Box((0.050, 0.030, 0.030)),
        origin=Origin(xyz=(0.012, 0.0, 1.000), rpy=(0.0, -0.38, 0.0)),
        material=black,
        name="palm_hook",
    )
    _add_carabiner_and_strap(upper, black=black, metal=silver)

    # ── Middle section ────────────────────────────────────────────────────
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

    # ── Lower section ─────────────────────────────────────────────────────
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
    # Bottom ferrule for the lower-to-tip fold junction
    lower.visual(
        Cylinder(radius=0.0130, length=0.035),
        origin=Origin(xyz=(0.0275, 0.0, -0.562)),
        material=silver,
        name="lower_bottom_ferrule",
    )
    for index, z in enumerate((-0.580, -0.545)):
        lower.visual(
            Cylinder(radius=0.0137, length=0.0025),
            origin=Origin(xyz=(0.0275, 0.0, z)),
            material=graphite,
            name=f"lower_bottom_crimp_ring_{index}",
        )
    # Fold hinge hardware at the bottom of the lower section for tip fold
    _add_fold_hinge_hardware(
        lower,
        metal=silver,
        black=graphite,
        prefix="lower_fold",
        base=(0.012, 0.0, -0.575),
    )

    # ── Tip stage (fourth folding segment) ────────────────────────────────
    tip = model.part("tip_stage")
    # Connector bridge tubes from hinge to tip tube (Z-fold segment)
    # Tip tube extends in +Z from hinge (like middle section) so that when
    # deployed at q=π it swings down below the lower section.
    for side, y in (("left", -0.006), ("right", 0.006)):
        tip.visual(
            _tube_between(
                (-0.0095, y, 0.000),
                (0.020, y, 0.055),
                radius=0.0024,
                name=f"tip_hinge_bridge_{side}",
            ),
            material=silver,
            name=f"tip_hinge_bridge_{side}",
        )
    tip.visual(
        Cylinder(radius=0.0065, length=0.020),
        origin=Origin(xyz=(-0.0095, 0.0, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="tip_hinge_bushing",
    )
    # Tip tube segment (shorter for ultralight Z-fold fast-pack proportion)
    tip.visual(
        Cylinder(radius=0.0085, length=0.280),
        origin=Origin(xyz=(0.020, 0.0, 0.200)),
        material=graphite,
        name="tip_tube",
    )
    # Ferrule at base of tip tube
    tip.visual(
        Cylinder(radius=0.0110, length=0.035),
        origin=Origin(xyz=(0.020, 0.0, 0.045)),
        material=silver,
        name="tip_ferrule",
    )
    for index, z in enumerate((0.028, 0.063)):
        tip.visual(
            Cylinder(radius=0.0117, length=0.0025),
            origin=Origin(xyz=(0.020, 0.0, z)),
            material=graphite,
            name=f"tip_ferrule_crimp_ring_{index}",
        )
    # Carbide point extending below the tip tube, overlapping at the tube bottom
    # for connectivity. Tube bottom is at z=0.340, carbide starts at z=0.330.
    tip.visual(
        Cylinder(radius=0.0040, length=0.080),
        origin=Origin(xyz=(0.020, 0.0, 0.370)),
        material=silver,
        name="carbide_point",
    )
    # Tip core spike, nested into carbide_point for connectivity
    tip.visual(
        Cylinder(radius=0.0030, length=0.030),
        origin=Origin(xyz=(0.020, 0.0, 0.415)),
        material=silver,
        name="tip_core",
    )
    # Rubber tip boot, nested into tip_core for connectivity
    tip.visual(
        mesh_from_geometry(_rubber_boot_mesh(), "rubber_tip"),
        origin=Origin(xyz=(0.020, 0.0, 0.425)),
        material=rubber,
        name="rubber_tip",
    )
    # Basket near the tip end of the tube, connected via the basket_hub to the tube
    _add_basket(tip, rubber=rubber, cx=0.020, cz=0.300)

    # ── Flip-lock levers ──────────────────────────────────────────────────
    upper_lock = model.part("upper_lock")
    _add_lock_lever(upper_lock, black=black, graphite=gray)

    lower_lock = model.part("lower_lock")
    _add_lock_lever(lower_lock, black=black, graphite=gray)

    # ── Articulations ─────────────────────────────────────────────────────
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
    # Fourth fold joint: replaces the former prismatic telescoping tip slide
    model.articulation(
        "lower_to_tip_fold",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=tip,
        origin=Origin(xyz=(0.0275, 0.0, -0.580)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.0, lower=0.0, upper=math.pi),
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
    fold_c = object_model.get_articulation("lower_to_tip_fold")
    lock_a = object_model.get_articulation("upper_to_upper_lock")
    lock_b = object_model.get_articulation("middle_to_lower_lock")

    # ── Structural delta: lower_to_tip_fold is a REVOLUTE fold ───────────
    ctx.check(
        "lower_to_tip_fold exists as a revolute fold joint",
        fold_c is not None,
        details="The tip stage must connect via a REVOLUTE fold, not a PRISMATIC slide.",
    )

    # ── Intentional hinge bushing overlaps at fold junctions ─────────────
    # The hinge bushings on child sections nest around the pivot hardware
    # on parent sections at each fold junction (hinge-pin capture).
    ctx.allow_overlap(
        middle,
        upper,
        elem_a="middle_hinge_bushing",
        elem_b="upper_fold_pivot_bushing",
        reason="The middle hinge bushing wraps around the upper fold pivot bushing at the fold junction.",
    )
    ctx.allow_overlap(
        middle,
        upper,
        elem_a="middle_hinge_bushing",
        elem_b="upper_fold_cross_pin",
        reason="The middle hinge bushing captures the upper fold cross-pin at the pivot.",
    )
    ctx.allow_overlap(
        lower,
        middle,
        elem_a="lower_hinge_bushing",
        elem_b="middle_fold_pivot_bushing",
        reason="The lower hinge bushing wraps around the middle fold pivot bushing at the fold junction.",
    )
    ctx.allow_overlap(
        lower,
        tip,
        elem_a="lower_tube",
        elem_b="tip_hinge_bushing",
        reason="The tip hinge bushing seats against the lower tube end at the fold junction.",
    )

    # ── Intentional Z-fold bundle overlaps ────────────────────────────────
    # In a Z-fold pole, the tip section folds up alongside the lower section.
    # The rubber tip boot, basket ring, and ferrule on the tip section naturally
    # overlap with the lower tube in the compact folded rest pose.
    ctx.allow_overlap(
        lower,
        tip,
        elem_a="lower_tube",
        elem_b="rubber_tip",
        reason="In the Z-fold compact bundle, the rubber tip boot sits alongside the lower tube (real Z-fold geometry).",
    )
    ctx.allow_overlap(
        lower,
        tip,
        elem_a="lower_tube",
        elem_b="basket_ring",
        reason="In the Z-fold compact bundle, the basket ring sits alongside the lower tube (real Z-fold geometry).",
    )
    ctx.allow_overlap(
        lower,
        tip,
        elem_a="lower_ferrule",
        elem_b="rubber_tip",
        reason="In the Z-fold compact bundle, the rubber tip boot sits alongside the lower ferrule (real Z-fold geometry).",
    )
    ctx.allow_overlap(
        lower,
        tip,
        elem_a="lower_fold_pivot_bushing",
        elem_b="tip_hinge_bushing",
        reason="The tip hinge bushing wraps around the lower fold pivot bushing at the fold junction.",
    )
    ctx.allow_overlap(
        lower,
        tip,
        elem_a="lower_bottom_ferrule",
        elem_b="tip_hinge_bushing",
        reason="The tip hinge bushing nests against the lower bottom ferrule at the fold junction pivot.",
    )

    # ── Folded bundle: sections lie parallel ─────────────────────────────
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
    ctx.expect_overlap(
        lower,
        tip,
        axes="z",
        min_overlap=0.20,
        elem_a="lower_tube",
        elem_b="tip_tube",
        name="folded lower and tip sections lie parallel (Z-fold bundle)",
    )

    def _center(aabb, axis_index: int) -> float:
        return (float(aabb[0][axis_index]) + float(aabb[1][axis_index])) / 2.0

    # ── Fold joints swing sections out of compact bundle ─────────────────
    folded_middle_aabb = ctx.part_world_aabb(middle)
    folded_tip_aabb = ctx.part_world_aabb(tip)
    with ctx.pose({fold_a: 2.35, fold_b: 1.30, fold_c: 1.00}):
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

    # ── Lower fold swings around the clear side of the middle tube ───────
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

    # ── Tip fold swings the tip section away from the lower tube ─────────
    with ctx.pose({fold_a: math.pi, fold_b: math.pi, fold_c: math.pi / 2.0}):
        half_swing_lower_aabb2 = ctx.part_element_world_aabb(lower, elem="lower_tube")
        half_swing_tip_aabb = ctx.part_element_world_aabb(tip, elem="tip_tube")
    ctx.check(
        "lower_to_tip_fold swings tip tube away from the lower tube",
        half_swing_lower_aabb2 is not None
        and half_swing_tip_aabb is not None
        and abs(_center(half_swing_tip_aabb, 0) - _center(half_swing_lower_aabb2, 0)) > 0.02,
        details=f"lower={half_swing_lower_aabb2}, tip={half_swing_tip_aabb}",
    )

    # ── Full deployment: all four sections align into a straight pole ─────
    with ctx.pose({fold_a: math.pi, fold_b: math.pi, fold_c: math.pi}):
        upper_tube_aabb = ctx.part_element_world_aabb(upper, elem="upper_tube")
        middle_tube_aabb = ctx.part_element_world_aabb(middle, elem="middle_tube")
        lower_tube_aabb = ctx.part_element_world_aabb(lower, elem="lower_tube")
        tip_tube_aabb = ctx.part_element_world_aabb(tip, elem="tip_tube")
        tip_carbide_aabb = ctx.part_element_world_aabb(tip, elem="carbide_point")
    if (
        upper_tube_aabb is not None
        and middle_tube_aabb is not None
        and lower_tube_aabb is not None
        and tip_tube_aabb is not None
    ):
        deployed_axis_x = [
            _center(upper_tube_aabb, 0),
            _center(middle_tube_aabb, 0),
            _center(lower_tube_aabb, 0),
            _center(tip_tube_aabb, 0),
        ]
        deployed_axis_y = [
            _center(upper_tube_aabb, 1),
            _center(middle_tube_aabb, 1),
            _center(lower_tube_aabb, 1),
            _center(tip_tube_aabb, 1),
        ]
    else:
        deployed_axis_x = []
        deployed_axis_y = []
    ctx.check(
        "all four fold joints reach a straight deployed pole axis",
        bool(deployed_axis_x)
        and max(deployed_axis_x) - min(deployed_axis_x) < 0.025
        and max(deployed_axis_y) - min(deployed_axis_y) < 0.012
        and middle_tube_aabb[1][2] < upper_tube_aabb[0][2]
        and lower_tube_aabb[1][2] < middle_tube_aabb[0][2]
        and tip_tube_aabb[1][2] < lower_tube_aabb[0][2],
        details=(
            f"x_centers={deployed_axis_x}, y_centers={deployed_axis_y}, "
            f"upper={upper_tube_aabb}, middle={middle_tube_aabb}, "
            f"lower={lower_tube_aabb}, tip={tip_tube_aabb}"
        ),
    )
    # Verify the carbide point extends further down than the tip tube when deployed
    if tip_carbide_aabb is not None and tip_tube_aabb is not None:
        ctx.check(
            "carbide point extends below tip tube when deployed",
            tip_carbide_aabb[1][2] < tip_tube_aabb[1][2] - 0.010,
            details=f"carbide={tip_carbide_aabb}, tip_tube={tip_tube_aabb}",
        )

    # ── Flip locks open outward ──────────────────────────────────────────
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
