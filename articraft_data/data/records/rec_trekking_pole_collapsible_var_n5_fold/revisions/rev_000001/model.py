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
# 5-segment Z-fold geometry constants
# ---------------------------------------------------------------------------
NUM_SEGMENTS = 5          # total fold tube segments (0=upper, 1-3=middle, 4=lower)
SEG_LENGTH = 0.180        # tube length per fold segment (m)
FOLD_D = 0.014            # X offset per fold joint (m)
HINGE_H = 0.008           # extra space beyond tube end for hinge (m)

# Direction pattern: segments alternate UP / DOWN to form Z-fold
# Seg 0 (upper): UP   Seg 1: UP   Seg 2: DOWN   Seg 3: UP   Seg 4 (lower): DOWN
SEG_DIRECTIONS = ("up", "up", "down", "up", "down")
TUBE_RADII = (0.011, 0.010, 0.0095, 0.009, 0.0095)

# Joint origins in each parent segment's local frame
_JOINT_XYZ = [
    (FOLD_D, 0.0, 0.0),                       # fold_01 in seg 0
    (2 * FOLD_D, 0.0, SEG_LENGTH + HINGE_H),  # fold_12 in seg 1 (top)
    (2 * FOLD_D, 0.0, -(SEG_LENGTH + HINGE_H)),  # fold_23 in seg 2 (bottom)
    (2 * FOLD_D, 0.0, SEG_LENGTH + HINGE_H),  # fold_34 in seg 3 (top)
]
_JOINT_AXIS = [
    (0.0, 1.0, 0.0),   # fold_01
    (0.0, -1.0, 0.0),  # fold_12
    (0.0, 1.0, 0.0),   # fold_23
    (0.0, -1.0, 0.0),  # fold_34
]


# ---------------------------------------------------------------------------
# Material helper
# ---------------------------------------------------------------------------
def _mat(model: ArticulatedObject, name: str, rgba) -> Material:
    return model.material(name, rgba=rgba)


# ---------------------------------------------------------------------------
# Mesh generators (unchanged from parent)
# ---------------------------------------------------------------------------
def _ergonomic_handle_mesh() -> MeshGeometry:
    """Lathed cork grip with a flared palm swell and a hook-like top cap seat."""
    profile = [
        (0.020, 0.000), (0.023, 0.020), (0.027, 0.090),
        (0.031, 0.180), (0.029, 0.270), (0.023, 0.335), (0.019, 0.365),
    ]
    return LatheGeometry(profile, segments=40, closed=True)


def _rubber_boot_mesh() -> MeshGeometry:
    profile = [
        (0.000, 0.000), (0.010, 0.000), (0.017, 0.010),
        (0.021, 0.040), (0.017, 0.070), (0.011, 0.085), (0.000, 0.085),
    ]
    return LatheGeometry(profile, segments=32, closed=True)


def _tube_between(a, b, *, radius: float, name: str):
    return mesh_from_geometry(
        tube_from_spline_points(
            [a, b], radius=radius, samples_per_segment=4,
            radial_segments=12, cap_ends=True,
        ),
        name,
    )


# ---------------------------------------------------------------------------
# Sub-assembly helpers
# ---------------------------------------------------------------------------
def _add_carabiner_and_strap(part, *, black, metal, z_off: float = -0.275) -> None:
    """Wrist strap loop + accessory carabiner, Z-shifted for shortened upper."""
    webbing = sweep_profile_along_spline(
        [
            (-0.010, 0.004, 0.855 + z_off),
            (-0.065, 0.012, 0.780 + z_off),
            (-0.105, 0.010, 0.630 + z_off),
            (-0.062, 0.007, 0.525 + z_off),
            (-0.014, 0.004, 0.765 + z_off),
        ],
        profile=rounded_rect_profile(0.012, 0.003, radius=0.0012, corner_segments=4),
        samples_per_segment=12, closed_spline=True, cap_profile=True,
    )
    part.visual(mesh_from_geometry(webbing, "wrist_webbing"), material=black, name="wrist_webbing")
    part.visual(
        Box((0.030, 0.012, 0.026)),
        origin=Origin(xyz=(-0.010, 0.004, 0.830 + z_off)),
        material=black, name="strap_anchor",
    )
    carabiner = tube_from_spline_points(
        [
            (-0.150, 0.012, 0.625 + z_off), (-0.198, 0.012, 0.590 + z_off),
            (-0.205, 0.012, 0.505 + z_off), (-0.158, 0.012, 0.450 + z_off),
            (-0.103, 0.012, 0.470 + z_off), (-0.093, 0.012, 0.570 + z_off),
            (-0.120, 0.012, 0.625 + z_off),
        ],
        radius=0.0042, samples_per_segment=12, radial_segments=14,
        closed_spline=True, cap_ends=True,
    )
    part.visual(
        mesh_from_geometry(carabiner, "carabiner_body"), material=black, name="carabiner_body",
    )
    part.visual(
        _tube_between(
            (-0.118, 0.012, 0.602 + z_off), (-0.102, 0.012, 0.484 + z_off),
            radius=0.0023, name="carabiner_gate",
        ),
        material=metal, name="carabiner_gate",
    )
    part.visual(
        _tube_between(
            (-0.077, 0.010, 0.595 + z_off), (-0.127, 0.012, 0.622 + z_off),
            radius=0.0030, name="carabiner_tether",
        ),
        material=black, name="carabiner_tether",
    )


def _add_lock_lever(part, *, black, graphite) -> None:
    """Flip-lock lever: hinge barrel + blade + finger tab."""
    part.visual(
        Cylinder(radius=0.006, length=0.034),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite, name="hinge_barrel",
    )
    part.visual(
        Box((0.014, 0.026, 0.075)),
        origin=Origin(xyz=(0.011, 0.0, -0.038)),
        material=black, name="lever_blade",
    )
    part.visual(
        Box((0.017, 0.028, 0.020)),
        origin=Origin(xyz=(0.019, 0.0, -0.078)),
        material=black, name="finger_tab",
    )


def _add_fold_hinge_hardware(
    part, *, metal, black, prefix: str,
    base=(0.0, 0.0, 0.0), flip_z: bool = False,
) -> None:
    """Hinge hardware: cross pin, bushing, washers, ears, forks, index stop.
    flip_z=True mirrors Z offsets for bottom hinges on DOWN segments."""
    bx, by, bz = base
    s = -1 if flip_z else 1
    part.visual(
        Cylinder(radius=0.0032, length=0.040),
        origin=Origin(xyz=(bx + 0.016, by, bz + s * 0.004), rpy=(math.pi / 2, 0, 0)),
        material=metal, name=f"{prefix}_cross_pin",
    )
    part.visual(
        Cylinder(radius=0.0100, length=0.018),
        origin=Origin(xyz=(bx + 0.016, by, bz + s * 0.004), rpy=(math.pi / 2, 0, 0)),
        material=black, name=f"{prefix}_pivot_bushing",
    )
    for side, y in (("left", -0.016), ("right", 0.016)):
        part.visual(
            Cylinder(radius=0.0120, length=0.0030),
            origin=Origin(xyz=(bx + 0.016, by + y, bz + s * 0.004), rpy=(math.pi / 2, 0, 0)),
            material=metal, name=f"{prefix}_{side}_flat_washer",
        )
        part.visual(
            Cylinder(radius=0.0060, length=0.0040),
            origin=Origin(xyz=(bx + 0.016, by + y * 1.15, bz + s * 0.004), rpy=(math.pi / 2, 0, 0)),
            material=metal, name=f"{prefix}_{side}_rivet_head",
        )
    for side, y in (("left", -0.022), ("right", 0.022)):
        part.visual(
            Box((0.010, 0.0030, 0.030)),
            origin=Origin(xyz=(bx + 0.006, by + y, bz + s * 0.011)),
            material=metal, name=f"{prefix}_{side}_hinge_ear",
        )
        part.visual(
            _tube_between(
                (bx - 0.006, by + y, bz),
                (bx + 0.015, by + y, bz + s * 0.004),
                radius=0.0019, name=f"{prefix}_{side}_fork_edge",
            ),
            material=metal, name=f"{prefix}_{side}_fork_edge",
        )
    part.visual(
        Box((0.009, 0.004, 0.012)),
        origin=Origin(xyz=(bx + 0.026, by, bz + s * 0.015), rpy=(0.0, -s * 0.32, 0.0)),
        material=metal, name=f"{prefix}_small_index_stop",
    )


def _add_basket(part, *, rubber) -> None:
    """Compact removable trekking basket near the tip."""
    part.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.044, tube=0.004, radial_segments=12, tubular_segments=48),
            "basket_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
        material=rubber, name="basket_ring",
    )
    part.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
        material=rubber, name="basket_hub",
    )
    for index, angle in enumerate(
        (0.0, math.pi / 3, 2 * math.pi / 3, math.pi, 4 * math.pi / 3, 5 * math.pi / 3)
    ):
        part.visual(
            Box((0.048, 0.006, 0.004)),
            origin=Origin(xyz=(0.027 * math.cos(angle), 0.027 * math.sin(angle), -0.070),
                          rpy=(0.0, 0.0, angle)),
            material=rubber, name=f"basket_spoke_{index}",
        )


# ---------------------------------------------------------------------------
# Shared fold-segment builder (loop helper for segments 1..3)
# ---------------------------------------------------------------------------
def _build_fold_segment(
    model, index: int, *, tube_length: float, tube_radius: float,
    direction: str, has_child_joint: bool,
    d: float, hinge_h: float, mats: dict,
):
    """Create one fold tube segment part with bridges, tube, ferrule, hinge."""
    seg = model.part(f"fold_segment_{index}")
    sign = 1 if direction == "up" else -1
    tz = sign * (tube_length / 2.0 + hinge_h / 2.0)

    # Main tube
    seg.visual(
        Cylinder(radius=tube_radius, length=tube_length),
        origin=Origin(xyz=(d, 0.0, tz)),
        material=mats["graphite"], name=f"seg{index}_tube",
    )

    # Ferrule at far end
    ferrule_z = sign * (tube_length + hinge_h * 0.5)
    seg.visual(
        Cylinder(radius=tube_radius + 0.003, length=0.018),
        origin=Origin(xyz=(d, 0.0, ferrule_z)),
        material=mats["silver"], name=f"seg{index}_ferrule",
    )
    for ci, off in enumerate((-0.006, 0.006)):
        seg.visual(
            Cylinder(radius=tube_radius + 0.0037, length=0.0025),
            origin=Origin(xyz=(d, 0.0, ferrule_z + sign * off)),
            material=mats["graphite"], name=f"seg{index}_crimp_{ci}",
        )

    # Connector bridges at origin end (linking to parent hinge)
    bridge_end_z = sign * hinge_h
    for side, y in (("left", -0.006), ("right", 0.006)):
        seg.visual(
            _tube_between(
                (-d, y, 0.0), (d, y, bridge_end_z),
                radius=0.0024, name=f"seg{index}_bridge_{side}",
            ),
            material=mats["silver"], name=f"seg{index}_bridge_{side}",
        )

    # Hinge bushing at origin
    seg.visual(
        Cylinder(radius=0.0065, length=0.020),
        origin=Origin(xyz=(-d, 0.0, 0.0), rpy=(math.pi / 2, 0, 0)),
        material=mats["graphite"], name=f"seg{index}_bushing",
    )

    # Child joint hinge hardware at far end
    if has_child_joint:
        hinge_base_z = sign * (tube_length + hinge_h)
        _add_fold_hinge_hardware(
            seg, metal=mats["silver"], black=mats["graphite"],
            prefix=f"seg{index}_fold",
            base=(2 * d - 0.016, 0.0, hinge_base_z - sign * 0.004),
            flip_z=(direction == "down"),
        )

    return seg


# ---------------------------------------------------------------------------
# Object model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="collapsible_trekking_pole_kit_5seg",
        meta={
            "run_notes": (
                "5-segment Z-fold collapsible trekking pole for ultra-compact packed length "
                "(trail-running / fastpacking form). Carry bag omitted; asset focuses on one "
                "folded pole assembly with attached carabiner, basket, and rubber tip."
            )
        },
    )

    # Materials
    black = _mat(model, "satin_black", (0.015, 0.016, 0.017, 1.0))
    graphite = _mat(model, "dark_graphite_aluminum", (0.090, 0.100, 0.105, 1.0))
    silver = _mat(model, "brushed_silver", (0.72, 0.72, 0.68, 1.0))
    cork = _mat(model, "speckled_cork", (0.68, 0.54, 0.35, 1.0))
    rubber = _mat(model, "matte_rubber", (0.025, 0.025, 0.026, 1.0))
    gray = _mat(model, "cool_gray_plastic", (0.42, 0.43, 0.43, 1.0))
    mats = {"black": black, "graphite": graphite, "silver": silver,
            "cork": cork, "rubber": rubber, "gray": gray}

    d = FOLD_D
    L = SEG_LENGTH
    h = HINGE_H

    # ================================================================
    # SEGMENT 0 — upper_section (root, carries handle / strap / carabiner)
    # ================================================================
    upper = model.part("upper_section")

    # Bottom ferrule + crimp rings
    upper.visual(
        Cylinder(radius=0.014, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.020)), material=silver, name="bottom_ferrule",
    )
    for ci, z in enumerate((-0.003, 0.043)):
        upper.visual(
            Cylinder(radius=0.0148, length=0.003),
            origin=Origin(xyz=(0.0, 0.0, z)), material=graphite,
            name=f"bottom_ferrule_crimp_ring_{ci}",
        )

    # Fold hinge hardware for joint fold_01
    _add_fold_hinge_hardware(upper, metal=silver, black=graphite, prefix="upper_fold")

    # Internal shock-cord spine
    upper.visual(
        Cylinder(radius=0.004, length=0.730),
        origin=Origin(xyz=(0.0, 0.0, 0.365)), material=graphite, name="hidden_core_spine",
    )

    # Upper tube (shortened for 5-segment pack)
    upper.visual(
        Cylinder(radius=0.011, length=L),
        origin=Origin(xyz=(0.0, 0.0, L / 2.0 + h / 2.0)),
        material=graphite, name="upper_tube",
    )

    # Upper clamp collar + lock lug
    upper.visual(
        Cylinder(radius=0.019, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.060)), material=black, name="upper_clamp_collar",
    )
    upper.visual(
        Box((0.012, 0.023, 0.020)),
        origin=Origin(xyz=(0.025, 0.0115, 0.075)), material=black, name="upper_lock_lug",
    )

    # Foam grip zone
    upper.visual(
        Cylinder(radius=0.017, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.220)), material=black, name="foam_grip_lower",
    )
    for ri, z in enumerate((0.220, 0.255, 0.290)):
        upper.visual(
            Cylinder(radius=0.018, length=0.020),
            origin=Origin(xyz=(0.0, 0.0, z)), material=rubber, name=f"foam_rib_{ri}",
        )

    # Cork handle (ergonomic lathe mesh)
    upper.visual(
        mesh_from_geometry(_ergonomic_handle_mesh(), "cork_handle"),
        origin=Origin(xyz=(0.0, 0.0, 0.320)), material=cork, name="cork_handle",
    )

    # Top cap + palm hook
    upper.visual(
        Cylinder(radius=0.025, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.687)), material=gray, name="top_cap_shell",
    )
    upper.visual(
        Box((0.050, 0.030, 0.030)),
        origin=Origin(xyz=(0.012, 0.0, 0.725), rpy=(0.0, -0.38, 0.0)),
        material=black, name="palm_hook",
    )

    # Wrist strap + carabiner accessory
    _add_carabiner_and_strap(upper, black=black, metal=silver, z_off=-0.275)

    # ================================================================
    # SEGMENTS 1–3 — middle fold segments via loop
    # ================================================================
    for i in range(1, NUM_SEGMENTS - 1):
        _build_fold_segment(
            model, i,
            tube_length=L, tube_radius=TUBE_RADII[i],
            direction=SEG_DIRECTIONS[i], has_child_joint=True,
            d=d, hinge_h=h, mats=mats,
        )

    # ================================================================
    # SEGMENT 4 — lower_section (connects to telescoping tip)
    # ================================================================
    lower = model.part("lower_section")
    sign4 = -1  # DOWN
    tz4 = sign4 * (L / 2.0 + h / 2.0)

    # Tube
    lower.visual(
        Cylinder(radius=TUBE_RADII[4], length=L),
        origin=Origin(xyz=(d, 0.0, tz4)), material=graphite, name="lower_tube",
    )

    # Ferrule + crimp rings at bottom
    ferrule_z4 = sign4 * (L + h * 0.5)
    lower.visual(
        Cylinder(radius=TUBE_RADII[4] + 0.003, length=0.018),
        origin=Origin(xyz=(d, 0.0, ferrule_z4)), material=silver, name="lower_ferrule",
    )
    for ci, off in enumerate((-0.006, 0.006)):
        lower.visual(
            Cylinder(radius=TUBE_RADII[4] + 0.0037, length=0.0025),
            origin=Origin(xyz=(d, 0.0, ferrule_z4 + sign4 * off)),
            material=graphite, name=f"lower_ferrule_crimp_{ci}",
        )

    # Connector bridges at origin
    bridge_end_z4 = sign4 * h
    for side, y in (("left", -0.006), ("right", 0.006)):
        lower.visual(
            _tube_between(
                (-d, y, 0.0), (d, y, bridge_end_z4),
                radius=0.0024, name=f"lower_bridge_{side}",
            ),
            material=silver, name=f"lower_bridge_{side}",
        )

    # Hinge bushing at origin
    lower.visual(
        Cylinder(radius=0.0065, length=0.020),
        origin=Origin(xyz=(-d, 0.0, 0.0), rpy=(math.pi / 2, 0, 0)),
        material=graphite, name="lower_bushing",
    )

    # Clamp collar + lock lug for tip adjustment
    lower.visual(
        Cylinder(radius=0.0165, length=0.050),
        origin=Origin(xyz=(d, 0.0, -0.100)), material=black, name="lower_clamp_collar",
    )
    lower.visual(
        Box((0.012, 0.023, 0.020)),
        origin=Origin(xyz=(d + 0.022, 0.0115, -0.085)),
        material=black, name="lower_lock_lug",
    )

    # ================================================================
    # TIP STAGE (telescoping, unchanged geometry)
    # ================================================================
    tip = model.part("tip_stage")
    tip.visual(
        Cylinder(radius=0.0070, length=0.380),
        origin=Origin(xyz=(0.0, 0.0, -0.050)), material=silver, name="inner_ferrule",
    )
    tip.visual(
        Cylinder(radius=0.0030, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, -0.300)), material=silver, name="tip_core",
    )
    tip.visual(
        Cylinder(radius=0.0040, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, -0.270)), material=silver, name="carbide_point",
    )
    tip.visual(
        mesh_from_geometry(_rubber_boot_mesh(), "rubber_tip"),
        origin=Origin(xyz=(0.0, 0.0, -0.390)), material=rubber, name="rubber_tip",
    )
    _add_basket(tip, rubber=rubber)

    # ================================================================
    # LOCK LEVERS
    # ================================================================
    upper_lock = model.part("upper_lock")
    _add_lock_lever(upper_lock, black=black, graphite=gray)

    lower_lock = model.part("lower_lock")
    _add_lock_lever(lower_lock, black=black, graphite=gray)

    # ================================================================
    # ARTICULATIONS
    # ================================================================

    # Collect segment parts in order for joint creation
    seg_parts = [upper]
    for i in range(1, NUM_SEGMENTS - 1):
        seg_parts.append(model.get_part(f"fold_segment_{i}"))
    seg_parts.append(lower)

    # 4 fold revolute joints (loop)
    for j in range(NUM_SEGMENTS - 1):
        ox, oy, oz = _JOINT_XYZ[j]
        ax, ay, az = _JOINT_AXIS[j]
        model.articulation(
            f"fold_{j}{j + 1}",
            ArticulationType.REVOLUTE,
            parent=seg_parts[j],
            child=seg_parts[j + 1],
            origin=Origin(xyz=(ox, oy, oz)),
            axis=(ax, ay, az),
            motion_limits=MotionLimits(effort=8.0, velocity=1.0, lower=0.0, upper=math.pi),
        )

    # Telescoping tip slide (shorter travel for compact 5-seg pole)
    model.articulation(
        "lower_to_tip_slide",
        ArticulationType.PRISMATIC,
        parent=lower, child=tip,
        origin=Origin(xyz=(d, 0.0, -(L + 0.020))),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.20, lower=0.0, upper=0.080),
    )

    # Flip-lock levers
    model.articulation(
        "upper_to_upper_lock",
        ArticulationType.REVOLUTE,
        parent=upper, child=upper_lock,
        origin=Origin(xyz=(0.025, 0.040, 0.075)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=1.0),
    )
    model.articulation(
        "lower_to_lower_lock",
        ArticulationType.REVOLUTE,
        parent=lower, child=lower_lock,
        origin=Origin(xyz=(d + 0.022, 0.040, -0.085)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=1.0),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def _tube_elem(seg_index: int) -> str:
    """Map segment index to its tube visual name."""
    if seg_index == 0:
        return "upper_tube"
    if seg_index == NUM_SEGMENTS - 1:
        return "lower_tube"
    return f"seg{seg_index}_tube"


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    upper = object_model.get_part("upper_section")
    lower = object_model.get_part("lower_section")
    tip = object_model.get_part("tip_stage")
    upper_lock = object_model.get_part("upper_lock")
    lower_lock = object_model.get_part("lower_lock")

    # Gather fold segments and joints
    fold_segs = [upper]
    for i in range(1, NUM_SEGMENTS - 1):
        fold_segs.append(object_model.get_part(f"fold_segment_{i}"))
    fold_segs.append(lower)

    fold_joints = []
    for j in range(NUM_SEGMENTS - 1):
        fold_joints.append(object_model.get_articulation(f"fold_{j}{j + 1}"))

    slide = object_model.get_articulation("lower_to_tip_slide")
    lock_a = object_model.get_articulation("upper_to_upper_lock")
    lock_b = object_model.get_articulation("lower_to_lower_lock")

    # ---- Structural: 5 fold segments + 4 fold joints exist ----
    ctx.check(
        "5 fold tube segments exist (upper + 3 middle + lower)",
        all(s is not None for s in fold_segs),
    )
    ctx.check(
        "4 revolute fold joints exist in Z-fold chain",
        all(j is not None for j in fold_joints),
    )

    # Specific 5th-segment assertion (structural delta)
    fold_seg_3 = object_model.get_part("fold_segment_3")
    ctx.check(
        "fold_segment_3 exists (5th tube section for compact Z-fold)",
        fold_seg_3 is not None,
    )
    fold_34 = object_model.get_articulation("fold_34")
    ctx.check(
        "fold_34 joint connects segment 3 to lower section",
        fold_34 is not None,
    )

    # ---- Overlap allowances: hinge-point embedding + telescoping tip ----
    # At each Z-fold hinge, the child's bushing embeds into the parent's ferrule
    # (and vice-versa) to represent the real pivot bearing.  These are small,
    # local, mechanically meaningful embeddings.
    _hinge_pairs = [
        (upper, fold_segs[1], "bottom_ferrule", "seg1_bushing"),
        (fold_segs[1], fold_segs[2], "seg1_ferrule", "seg2_bushing"),
        (fold_segs[2], fold_segs[3], "seg2_ferrule", "seg3_bushing"),
        (fold_segs[3], lower, "seg3_ferrule", "lower_bushing"),
    ]
    for pa, pb, ea, eb in _hinge_pairs:
        ctx.allow_overlap(
            pa, pb, elem_a=ea, elem_b=eb,
            reason=(
                f"The {eb} is intentionally embedded in the {ea} at the Z-fold hinge "
                "pivot to represent the real bearing/bushing interface."
            ),
        )

    # Hinge pivot bushings sit at the junction between adjacent segments' tubes
    _pivot_tube_pairs = [
        (upper, fold_segs[1], "upper_fold_pivot_bushing", "seg1_tube"),
        (fold_segs[1], fold_segs[2], "seg1_fold_pivot_bushing", "seg2_tube"),
        (fold_segs[2], fold_segs[3], "seg2_fold_pivot_bushing", "seg3_tube"),
        (fold_segs[3], lower, "seg3_fold_pivot_bushing", "lower_tube"),
    ]
    for pa, pb, ea, eb in _pivot_tube_pairs:
        ctx.allow_overlap(
            pa, pb, elem_a=ea, elem_b=eb,
            reason=(
                f"The {ea} at the Z-fold hinge wraps around the adjacent {eb} "
                "to represent the pivot bearing; small local overlap is intentional."
            ),
        )

    # Upper lock lug protrudes slightly into the adjacent folded segment's
    # tube envelope; this is realistic for a compact Z-fold pole.
    ctx.allow_overlap(
        fold_segs[1], upper,
        elem_a="seg1_tube", elem_b="upper_lock_lug",
        reason=(
            "The upper_lock_lug on the clamp collar protrudes slightly into the "
            "adjacent folded segment's tube envelope; compact Z-fold geometry."
        ),
    )

    # Telescoping tip: ferrule retained inside lower sleeve
    ctx.allow_overlap(
        lower, tip,
        elem_a="lower_tube", elem_b="inner_ferrule",
        reason=(
            "The slim ferrule is intentionally modeled as retained inside the lower sleeve "
            "so the trekking pole has a believable telescoping tip adjustment."
        ),
    )
    ctx.allow_overlap(
        lower, tip,
        elem_a="lower_clamp_collar", elem_b="inner_ferrule",
        reason=(
            "The clamp collar wraps around the lower tube where the tip ferrule slides "
            "through; small local overlap at the clamping interface is intentional."
        ),
    )
    ctx.allow_overlap(
        lower, tip,
        elem_a="lower_ferrule", elem_b="inner_ferrule",
        reason=(
            "The lower_ferrule caps the end of the lower tube where the inner_ferrule "
            "slides through; the two concentric cylinders intentionally overlap at the "
            "telescoping interface."
        ),
    )

    # ---- Folded bundle: adjacent tubes overlap in Z (parallel) ----
    for j in range(NUM_SEGMENTS - 1):
        ctx.expect_overlap(
            fold_segs[j], fold_segs[j + 1],
            axes="z", min_overlap=0.08,
            elem_a=_tube_elem(j), elem_b=_tube_elem(j + 1),
            name=f"folded segments {j} and {j+1} lie parallel in bundle",
        )

    # ---- Tip containment and insertion ----
    ctx.expect_within(
        tip, lower, axes="xy", margin=0.003,
        inner_elem="inner_ferrule", outer_elem="lower_tube",
        name="telescoping ferrule stays centered in sleeve",
    )
    ctx.expect_overlap(
        tip, lower, axes="z", min_overlap=0.04,
        elem_a="inner_ferrule", elem_b="lower_tube",
        name="collapsed tip remains inserted",
    )

    # ---- Deployed pose: straight axis + sequential Z ordering ----
    def _center(bb, idx):
        return (float(bb[0][idx]) + float(bb[1][idx])) / 2.0

    deployed_pose = {fold_joints[j]: math.pi for j in range(NUM_SEGMENTS - 1)}
    deployed_pose[slide] = 0.080

    with ctx.pose(deployed_pose):
        tube_aabbs = []
        for k, seg in enumerate(fold_segs):
            tube_aabbs.append(ctx.part_element_world_aabb(seg, elem=_tube_elem(k)))
        tip_aabb = ctx.part_element_world_aabb(tip, elem="inner_ferrule")

    all_aabbs = tube_aabbs + [tip_aabb]
    if all(a is not None for a in all_aabbs):
        x_centers = [_center(a, 0) for a in all_aabbs]
        y_centers = [_center(a, 1) for a in all_aabbs]

        ctx.check(
            "5-segment deployed pole axis is straight (X alignment)",
            max(x_centers) - min(x_centers) < 0.022,
            details=f"x_centers={x_centers}",
        )
        ctx.check(
            "5-segment deployed pole axis is straight (Y alignment)",
            max(y_centers) - min(y_centers) < 0.014,
            details=f"y_centers={y_centers}",
        )

        # Z ordering: 5 tube segments must be sequential (tip telescopes into
        # lower so its max Z may overlap the lower tube — check tubes only)
        z_ordered = all(
            tube_aabbs[k][0][2] > tube_aabbs[k + 1][1][2]
            for k in range(len(tube_aabbs) - 1)
        )
        ctx.check(
            "deployed tube segments sequentially ordered along Z (5 sections)",
            z_ordered,
            details=f"tube_aabbs={tube_aabbs}",
        )
        # Tip extends below the lower section
        ctx.check(
            "tip ferrule extends below lower tube when deployed",
            tip_aabb[0][2] < tube_aabbs[-1][0][2],
            details=f"tip_min_z={tip_aabb[0][2]}, lower_min_z={tube_aabbs[-1][0][2]}",
        )
    else:
        ctx.fail("deployed pole geometry", "could not compute deployed AABBs")

    # ---- Fold joints swing sections out of compact bundle ----
    folded_mid = ctx.part_world_aabb(fold_segs[2])
    with ctx.pose({fold_joints[0]: 2.35, fold_joints[1]: 1.30}):
        deployed_mid = ctx.part_world_aabb(fold_segs[2])
    ctx.check(
        "fold joints swing middle sections out of compact bundle",
        folded_mid is not None and deployed_mid is not None
        and deployed_mid[0][2] < folded_mid[0][2] - 0.08,
        details=f"folded={folded_mid}, deployed={deployed_mid}",
    )

    # ---- Tip slide extends downward ----
    rest_tip_pos = ctx.part_world_position(tip)
    with ctx.pose({slide: 0.080}):
        ctx.expect_within(
            tip, lower, axes="xy", margin=0.003,
            inner_elem="inner_ferrule", outer_elem="lower_tube",
            name="extended ferrule stays centered in sleeve",
        )
        ctx.expect_overlap(
            tip, lower, axes="z", min_overlap=0.030,
            elem_a="inner_ferrule", elem_b="lower_tube",
            name="extended tip retains insertion",
        )
        ext_tip_pos = ctx.part_world_position(tip)
    ctx.check(
        "tip slide extends downward",
        rest_tip_pos is not None and ext_tip_pos is not None
        and ext_tip_pos[2] < rest_tip_pos[2] - 0.06,
        details=f"rest={rest_tip_pos}, extended={ext_tip_pos}",
    )

    # ---- Flip-lock levers open outward ----
    closed_ul = ctx.part_world_aabb(upper_lock)
    closed_ll = ctx.part_world_aabb(lower_lock)
    with ctx.pose({lock_a: 0.8, lock_b: 0.8}):
        open_ul = ctx.part_world_aabb(upper_lock)
        open_ll = ctx.part_world_aabb(lower_lock)
    ctx.check(
        "flip locks open outward",
        closed_ul is not None and open_ul is not None
        and closed_ll is not None and open_ll is not None
        and open_ul[1][0] > closed_ul[1][0] + 0.015
        and open_ll[1][0] > closed_ll[1][0] + 0.015,
        details=f"ul_closed={closed_ul}, ul_open={open_ul}, ll_closed={closed_ll}, ll_open={open_ll}",
    )

    return ctx.report()


object_model = build_object_model()
