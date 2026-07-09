from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_side_loft,
)


# ── Layout constants ──────────────────────────────────────────────
SEGMENT_COUNT = 4
SEG_LENGTH = 0.200           # tube length per segment (m)
TUBE_OUTER_R = 0.0110        # outer tube radius
TUBE_INNER_R = 0.0090        # inner bore radius
FERRULE_HEIGHT = 0.056       # ferrule extends below first tube section
SEG_GAP = 0.0                # segments meet exactly at fold joints
FOLD_UPPER = math.pi         # 180° fold-back per hinge


def _hollow_tube_mesh(outer_radius, inner_radius, z_min, z_max, *, name):
    """Revolved thin-wall tube with real central clearance."""
    shell = LatheGeometry.from_shell_profiles(
        [(outer_radius, z_min), (outer_radius, z_max)],
        [(inner_radius, z_min), (inner_radius, z_max)],
        segments=48,
        start_cap="flat",
        end_cap="flat",
        lip_samples=4,
    )
    return mesh_from_geometry(shell, name)


def _ferrule_mesh():
    """Compact flared rubber ferrule — identical to parent baseline."""
    ferrule = LatheGeometry(
        [
            (0.000, 0.000),
            (0.022, 0.000),
            (0.026, 0.006),
            (0.024, 0.014),
            (0.018, 0.026),
            (0.016, 0.050),
            (0.010, 0.056),
            (0.000, 0.056),
        ],
        segments=56,
        closed=True,
    )
    return mesh_from_geometry(ferrule, "rubber_ferrule")


def _handle_grip_mesh():
    """Ergonomic T-handle grip — identical to parent baseline."""
    grip = superellipse_side_loft(
        [
            (-0.083, 0.000, 0.034, 0.042),
            (-0.052, -0.001, 0.038, 0.048),
            (-0.020, -0.004, 0.043, 0.052),
            (0.000, -0.005, 0.045, 0.054),
            (0.020, -0.004, 0.043, 0.052),
            (0.052, -0.001, 0.038, 0.048),
            (0.083, 0.000, 0.034, 0.042),
        ],
        exponents=3.0,
        segments=56,
        cap=True,
        closed=True,
    )
    grip.rotate_z(-math.pi / 2.0)
    return mesh_from_geometry(grip, "ergonomic_grip")


def _seg_tube_z(i):
    """Local (z_min, z_max) of the tube for segment *i*."""
    if i == 0:
        # Small overlap with ferrule top (0.056) guarantees connectivity.
        return (FERRULE_HEIGHT - 0.006, FERRULE_HEIGHT + SEG_LENGTH)
    return (0.0, SEG_LENGTH)


def _seg_joint_z(i):
    """Local Z of the fold joint above segment *i* (tube top + gap)."""
    return _seg_tube_z(i)[1] + SEG_GAP


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_point_folding_cane")

    chrome = model.material("brushed_chrome", rgba=(0.78, 0.80, 0.82, 1.0))
    dark_chrome = model.material("shadow_chrome", rgba=(0.42, 0.44, 0.46, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.012, 0.012, 0.014, 1.0))
    molded_black = model.material("molded_black", rgba=(0.020, 0.020, 0.024, 1.0))

    segments = []

    # ── Shaft segments (shared helper, loop, name_{i}) ────────────
    for i in range(SEGMENT_COUNT):
        seg = model.part(f"shaft_seg_{i}")
        z_min, z_max = _seg_tube_z(i)

        # Main tube section
        seg.visual(
            _hollow_tube_mesh(
                TUBE_OUTER_R, TUBE_INNER_R, z_min, z_max,
                name=f"tube_{i}",
            ),
            material=chrome,
            name=f"tube_{i}",
        )

        # Pivot-band collar at tube top (visible fold hinge marker)
        if i < SEGMENT_COUNT - 1:
            band_h = 0.008
            seg.visual(
                Cylinder(radius=TUBE_OUTER_R + 0.002, length=band_h),
                origin=Origin(xyz=(0.0, 0.0, z_max - band_h / 2.0)),
                material=dark_chrome,
                name=f"pivot_band_{i}",
            )

        # Rubber ferrule on bottom segment only (identical to parent)
        if i == 0:
            seg.visual(
                _ferrule_mesh(),
                material=black_rubber,
                name="ferrule",
            )

        seg.inertial = Inertial.from_geometry(
            Box((0.030, 0.030, z_max - z_min)),
            mass=0.10,
            origin=Origin(xyz=(0.0, 0.0, (z_min + z_max) / 2.0)),
        )
        segments.append(seg)

    # ── Fold joints (revolute, axis X) ────────────────────────────
    for i in range(SEGMENT_COUNT - 1):
        model.articulation(
            f"fold_joint_{i}",
            ArticulationType.REVOLUTE,
            parent=segments[i],
            child=segments[i + 1],
            origin=Origin(xyz=(0.0, 0.0, _seg_joint_z(i))),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=2.0,
                lower=0.0,
                upper=FOLD_UPPER,
            ),
        )

    # ── Handle (identical to parent baseline) ─────────────────────
    handle = model.part("handle")
    handle.visual(
        _handle_grip_mesh(),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=molded_black,
        name="grip",
    )
    handle.visual(
        Cylinder(radius=0.0160, length=0.054),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=molded_black,
        name="neck",
    )
    handle.visual(
        Cylinder(radius=0.0200, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=molded_black,
        name="socket_flare",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.180, 0.060, 0.070)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
    )

    # Fixed joint: top segment → handle (mounted on tube top, no gap)
    top_tube_z = _seg_tube_z(SEGMENT_COUNT - 1)[1]
    model.articulation(
        "seg3_to_handle",
        ArticulationType.FIXED,
        parent=segments[-1],
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, top_tube_z)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    seg0 = object_model.get_part("shaft_seg_0")
    seg1 = object_model.get_part("shaft_seg_1")
    seg3 = object_model.get_part("shaft_seg_3")
    handle = object_model.get_part("handle")
    fold0 = object_model.get_articulation("fold_joint_0")
    fold1 = object_model.get_articulation("fold_joint_1")
    fold2 = object_model.get_articulation("fold_joint_2")

    # ── Handle seats on top shaft segment ─────────────────────────
    ctx.expect_contact(
        handle, seg3,
        elem_a="socket_flare",
        elem_b="tube_3",
        contact_tol=0.002,
        name="handle socket seats on shaft_seg_3",
    )

    # ── Ferrule is on the bottom segment ──────────────────────────
    ctx.expect_overlap(
        seg0, seg0,
        axes="z",
        elem_a="ferrule",
        elem_b="tube_0",
        min_overlap=0.003,
        name="ferrule overlaps tube_0 for connectivity",
    )

    # ── Rest pose: straight cane ──────────────────────────────────
    rest_seg1_aabb = ctx.part_world_aabb(seg1)
    rest_handle_aabb = ctx.part_world_aabb(handle)
    rest_seg0_aabb = ctx.part_world_aabb(seg0)

    # ── fold_joint_0 pivots shaft_seg_1 off the vertical ──────────
    with ctx.pose({fold0: math.pi / 2.0}):
        folded_seg1_aabb = ctx.part_world_aabb(seg1)

    # When seg_1 folds 90° around X at its base joint, its Z extent
    # (vertical) must shrink dramatically while Y extent grows.
    rest_z = (rest_seg1_aabb[1][2] - rest_seg1_aabb[0][2]) if rest_seg1_aabb else None
    fold_z = (folded_seg1_aabb[1][2] - folded_seg1_aabb[0][2]) if folded_seg1_aabb else None
    rest_y = (rest_seg1_aabb[1][1] - rest_seg1_aabb[0][1]) if rest_seg1_aabb else None
    fold_y = (folded_seg1_aabb[1][1] - folded_seg1_aabb[0][1]) if folded_seg1_aabb else None

    ctx.check(
        "fold_joint_0 pivots shaft_seg_1 off vertical",
        rest_z is not None
        and fold_z is not None
        and rest_y is not None
        and fold_y is not None
        and fold_z < rest_z - 0.05
        and fold_y > rest_y + 0.05,
        details=f"rest_z={rest_z:.4f}, fold_z={fold_z:.4f}, rest_y={rest_y:.4f}, fold_y={fold_y:.4f}",
    )

    # ── Fully folded pose collapses cane height ───────────────────
    with ctx.pose({fold0: math.pi, fold1: math.pi, fold2: math.pi}):
        folded_handle_aabb = ctx.part_world_aabb(handle)

    rest_top = rest_handle_aabb[1][2] if rest_handle_aabb else None
    folded_top = folded_handle_aabb[1][2] if folded_handle_aabb else None

    ctx.check(
        "folding all joints collapses cane height",
        rest_top is not None
        and folded_top is not None
        and folded_top < rest_top - 0.05,
        details=f"straight_top={rest_top}, folded_top={folded_top}",
    )

    # ── Cane height at walking scale ──────────────────────────────
    cane_bottom = rest_seg0_aabb[0][2] if rest_seg0_aabb else None
    cane_top = rest_handle_aabb[1][2] if rest_handle_aabb else None
    ctx.check(
        "cane height is walking scale",
        cane_bottom is not None
        and cane_top is not None
        and 0.82 <= (cane_top - cane_bottom) <= 1.05,
        details=f"bottom={cane_bottom}, top={cane_top}",
    )

    return ctx.report()


object_model = build_object_model()
