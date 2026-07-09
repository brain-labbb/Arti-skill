from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _mat(model: ArticulatedObject, name: str, rgba: tuple[float, float, float, float]) -> str:
    model.material(name, rgba=rgba)
    return name


def _rounded_square(sx: float, sy: float, sz: float, r: float) -> cq.Workplane:
    """Rounded-square solid used for the base tray."""
    return cq.Workplane("XY").box(sx, sy, sz).edges("|Z").fillet(r)


def _hollow_base() -> cq.Workplane:
    """Glossy black tray with a recessed cavity for the powder pan."""
    body = _rounded_square(0.144, 0.134, 0.030, 0.022).translate((0.0, 0.0, 0.015))
    cavity = _rounded_square(0.120, 0.110, 0.020, 0.013).translate((0.0, 0.0, 0.023))
    return body.cut(cavity)


def _rounded_panel(w: float, d: float, h: float, r: float) -> cq.Workplane:
    """Rounded rectangular panel used for clamshell leaf shells and insets."""
    return cq.Workplane("XY").box(w, d, h).edges("|Z").fillet(r)


# ---------------------------------------------------------------------------
# Shared dimensional constants
# ---------------------------------------------------------------------------

COMPACT_W = 0.144
COMPACT_D = 0.134
HALF_D = COMPACT_D / 2.0          # 0.067 – each leaf covers half the depth
BASE_H = 0.030
LEAF_H = 0.012
TOP_H = 0.008
HINGE_Z = BASE_H + 0.002          # 0.032 – hinge axis just above the tray rim
REAR_Y = COMPACT_D / 2.0          # +0.067
FRONT_Y = -COMPACT_D / 2.0        # -0.067
HINGE_XS = (-0.032, 0.032)        # symmetric hinge barrel/pin X positions
JOINT_LIMITS = MotionLimits(effort=2.2, velocity=1.4, lower=0.0, upper=1.55)


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="square_luxury_cushion_compact")

    # ── Materials ──
    black = _mat(model, "glossy_black", (0.012, 0.012, 0.012, 1.0))
    white = _mat(model, "glossy_white", (0.93, 0.92, 0.88, 1.0))
    chrome = _mat(model, "chrome_trim", (0.66, 0.67, 0.66, 1.0))
    logo_ink = _mat(model, "logo_black", (0.02, 0.02, 0.02, 1.0))
    mirror_mat = _mat(model, "mirror", (0.72, 0.78, 0.80, 0.55))
    powder = _mat(model, "beige_powder", (0.80, 0.64, 0.48, 1.0))
    cream = _mat(model, "cream_accent", (0.90, 0.85, 0.76, 1.0))

    # ── Base tray (root) ──
    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_hollow_base(), "compact_base"),
        material=black, name="black_tray",
    )
    # Beige powder cushion in the recess
    base.visual(
        Cylinder(radius=0.046, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=powder, name="powder_pan",
    )
    base.visual(
        Cylinder(radius=0.038, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=powder, name="powder_dome",
    )
    # Rear hinge barrels (pair, for rear leaf)
    for i in range(2):
        base.visual(
            Cylinder(radius=0.005, length=0.030),
            origin=Origin(xyz=(HINGE_XS[i], REAR_Y, HINGE_Z),
                          rpy=(0.0, math.pi / 2.0, 0.0)),
            material=chrome, name=f"rear_hinge_barrel_{i}",
        )
    # Front hinge barrels (pair, for front leaf)
    for i in range(2):
        base.visual(
            Cylinder(radius=0.005, length=0.030),
            origin=Origin(xyz=(HINGE_XS[i], FRONT_Y, HINGE_Z),
                          rpy=(0.0, math.pi / 2.0, 0.0)),
            material=chrome, name=f"front_hinge_barrel_{i}",
        )

    # ── Rear leaf  (hinges from rear edge, opens upward/backward) ──
    # Part frame sits on the rear hinge line; body extends in -Y toward center.
    leaf_rear = model.part("leaf_rear")
    leaf_rear.visual(
        mesh_from_cadquery(
            _rounded_panel(COMPACT_W, HALF_D, LEAF_H, 0.018), "leaf_rear_shell"),
        origin=Origin(xyz=(0.0, -HALF_D / 2.0, 0.004)),
        material=black, name="rear_black_panel",
    )
    leaf_rear.visual(
        mesh_from_cadquery(
            _rounded_panel(0.128, 0.054, TOP_H, 0.014), "leaf_rear_inset"),
        origin=Origin(xyz=(0.0, -HALF_D / 2.0 + 0.003, 0.012)),
        material=white, name="rear_white_top",
    )
    # Chrome parting-edge strip at the center meeting line
    leaf_rear.visual(
        Box((COMPACT_W - 0.024, 0.003, LEAF_H)),
        origin=Origin(xyz=(0.0, -(HALF_D - 0.0015), 0.004)),
        material=chrome, name="rear_parting_edge",
    )
    # Hinge pins captured by rear barrels
    for i in range(2):
        leaf_rear.visual(
            Cylinder(radius=0.0035, length=0.022),
            origin=Origin(xyz=(HINGE_XS[i], 0.0, 0.0),
                          rpy=(0.0, math.pi / 2.0, 0.0)),
            material=chrome, name=f"rear_hinge_pin_{i}",
        )
    # Center catch tab at the meeting edge
    leaf_rear.visual(
        Box((0.020, 0.006, 0.008)),
        origin=Origin(xyz=(0.0, -(HALF_D - 0.003), 0.002)),
        material=chrome, name="rear_center_catch",
    )
    # Interlocking double-ring emblem on rear leaf exterior
    for i in range(2):
        dx = (-0.011, 0.011)[i]
        ring = TorusGeometry(0.017, 0.0028, radial_segments=16, tubular_segments=28)
        leaf_rear.visual(
            mesh_from_geometry(ring, f"logo_ring_{i}"),
            origin=Origin(xyz=(dx, -HALF_D / 2.0, 0.017)),
            material=logo_ink, name=f"logo_ring_{i}",
        )
    # Inner mirror on the rear leaf underside
    leaf_rear.visual(
        Cylinder(radius=0.044, length=0.0025),
        origin=Origin(xyz=(0.0, -HALF_D / 2.0, -0.003)),
        material=mirror_mat, name="inner_mirror",
    )

    # ── Front leaf  (hinges from front edge, opens upward/forward) ──
    # Part frame sits on the front hinge line; body extends in +Y toward center.
    leaf_front = model.part("leaf_front")
    leaf_front.visual(
        mesh_from_cadquery(
            _rounded_panel(COMPACT_W, HALF_D, LEAF_H, 0.018), "leaf_front_shell"),
        origin=Origin(xyz=(0.0, HALF_D / 2.0, 0.004)),
        material=black, name="front_black_panel",
    )
    leaf_front.visual(
        mesh_from_cadquery(
            _rounded_panel(0.128, 0.054, TOP_H, 0.014), "leaf_front_inset"),
        origin=Origin(xyz=(0.0, HALF_D / 2.0 - 0.003, 0.012)),
        material=cream, name="front_cream_top",
    )
    # Chrome parting-edge strip at the center meeting line
    leaf_front.visual(
        Box((COMPACT_W - 0.024, 0.003, LEAF_H)),
        origin=Origin(xyz=(0.0, HALF_D - 0.0015, 0.004)),
        material=chrome, name="front_parting_edge",
    )
    # Hinge pins captured by front barrels
    for i in range(2):
        leaf_front.visual(
            Cylinder(radius=0.0035, length=0.022),
            origin=Origin(xyz=(HINGE_XS[i], 0.0, 0.0),
                          rpy=(0.0, math.pi / 2.0, 0.0)),
            material=chrome, name=f"front_hinge_pin_{i}",
        )
    # Center catch tab at the meeting edge
    leaf_front.visual(
        Box((0.020, 0.006, 0.008)),
        origin=Origin(xyz=(0.0, HALF_D - 0.003, 0.002)),
        material=chrome, name="front_center_catch",
    )

    # ── Articulations (uniform joint policy: same limits, symmetric axes) ──
    # Rear leaf: axis = -X so positive q lifts the free (center) edge upward.
    model.articulation(
        "base_to_leaf_rear",
        ArticulationType.REVOLUTE,
        parent=base, child=leaf_rear,
        origin=Origin(xyz=(0.0, REAR_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=JOINT_LIMITS,
    )
    # Front leaf: axis = +X so positive q lifts the free (center) edge upward.
    model.articulation(
        "base_to_leaf_front",
        ArticulationType.REVOLUTE,
        parent=base, child=leaf_front,
        origin=Origin(xyz=(0.0, FRONT_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=JOINT_LIMITS,
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    leaf_rear = object_model.get_part("leaf_rear")
    leaf_front = object_model.get_part("leaf_front")
    hinge_rear = object_model.get_articulation("base_to_leaf_rear")
    hinge_front = object_model.get_articulation("base_to_leaf_front")

    # ── Hinge-pin / barrel nesting allowances ──
    for i in range(2):
        ctx.allow_overlap(
            leaf_rear, base,
            elem_a=f"rear_hinge_pin_{i}", elem_b=f"rear_hinge_barrel_{i}",
            reason="Rear leaf hinge pin is captured inside the rear hinge barrel.",
        )
        ctx.allow_overlap(
            base, leaf_rear,
            elem_a=f"rear_hinge_barrel_{i}", elem_b="rear_black_panel",
            reason="Rear hinge barrel is locally nested under the rear leaf shell edge.",
        )
        ctx.allow_overlap(
            leaf_front, base,
            elem_a=f"front_hinge_pin_{i}", elem_b=f"front_hinge_barrel_{i}",
            reason="Front leaf hinge pin is captured inside the front hinge barrel.",
        )
        ctx.allow_overlap(
            base, leaf_front,
            elem_a=f"front_hinge_barrel_{i}", elem_b="front_black_panel",
            reason="Front hinge barrel is locally nested under the front leaf shell edge.",
        )

    # ── Closed-pose: each leaf covers its half of the base ──
    ctx.expect_overlap(
        leaf_rear, base, axes="xy", min_overlap=0.040,
        name="rear leaf covers rear half of base footprint",
    )
    ctx.expect_overlap(
        leaf_front, base, axes="xy", min_overlap=0.040,
        name="front leaf covers front half of base footprint",
    )

    # Both leaves share the same X footprint when closed
    ctx.expect_overlap(
        leaf_rear, leaf_front, axes="x", min_overlap=0.100,
        name="both leaves share the same X footprint when closed",
    )

    # ── Rear leaf opens upward ──
    closed_rear = ctx.part_element_world_aabb(leaf_rear, elem="rear_white_top")
    with ctx.pose({hinge_rear: 1.15}):
        opened_rear = ctx.part_element_world_aabb(leaf_rear, elem="rear_white_top")
    ctx.check(
        "rear leaf opens upward",
        closed_rear is not None and opened_rear is not None
        and opened_rear[1][2] > closed_rear[1][2] + 0.030,
        details=f"closed_max_z={closed_rear[1][2] if closed_rear else None}, "
                f"opened_max_z={opened_rear[1][2] if opened_rear else None}",
    )

    # ── Front leaf opens upward ──
    closed_front = ctx.part_element_world_aabb(leaf_front, elem="front_cream_top")
    with ctx.pose({hinge_front: 1.15}):
        opened_front = ctx.part_element_world_aabb(leaf_front, elem="front_cream_top")
    ctx.check(
        "front leaf opens upward",
        closed_front is not None and opened_front is not None
        and opened_front[1][2] > closed_front[1][2] + 0.030,
        details=f"closed_max_z={closed_front[1][2] if closed_front else None}, "
                f"opened_max_z={opened_front[1][2] if opened_front else None}",
    )

    # ── Both leaves open simultaneously (independent clamshell action) ──
    with ctx.pose({hinge_rear: 1.0, hinge_front: 1.0}):
        both_rear = ctx.part_element_world_aabb(leaf_rear, elem="rear_white_top")
        both_front = ctx.part_element_world_aabb(leaf_front, elem="front_cream_top")
    ctx.check(
        "both leaves open independently",
        both_rear is not None and both_front is not None
        and both_rear[1][2] > 0.060 and both_front[1][2] > 0.060,
        details=f"rear_max_z={both_rear[1][2] if both_rear else None}, "
                f"front_max_z={both_front[1][2] if both_front else None}",
    )

    # ── Inner mirror on rear leaf underside ──
    ctx.expect_overlap(
        leaf_rear, leaf_rear, axes="xy",
        elem_a="inner_mirror", elem_b="rear_black_panel",
        min_overlap=0.060,
        name="inner mirror is mounted on rear leaf underside",
    )

    # ── Powder pan remains seated in base ──
    ctx.expect_overlap(
        base, base, axes="xy",
        elem_a="powder_pan", elem_b="black_tray",
        min_overlap=0.080,
        name="powder pan remains seated in base tray",
    )

    # ── Two-leaf structure: exactly two independently hinged leaves ──
    arts = object_model.articulations
    revolute_count = sum(
        1 for a in arts if a.articulation_type == ArticulationType.REVOLUTE
    )
    ctx.check(
        "compact has exactly two revolute leaf joints",
        revolute_count == 2,
        details=f"found {revolute_count} revolute joints",
    )

    # ── Structural richness ──
    ctx.check(
        "compact has rich visual structure",
        len(base.visuals) >= 7
        and len(leaf_rear.visuals) >= 8
        and len(leaf_front.visuals) >= 5,
        details=f"base={len(base.visuals)}, leaf_rear={len(leaf_rear.visuals)}, "
                f"leaf_front={len(leaf_front.visuals)}",
    )

    return ctx.report()


object_model = build_object_model()
