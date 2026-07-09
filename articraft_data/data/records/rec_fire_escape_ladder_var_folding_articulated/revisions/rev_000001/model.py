from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


# ── Materials ───────────────────────────────────────────────────────────────
STEEL = Material("galvanized_steel", color=(0.78, 0.80, 0.78, 1.0))
SHADOW_STEEL = Material("shadowed_steel", color=(0.56, 0.58, 0.56, 1.0))
SAFETY_YELLOW = Material("safety_yellow", color=(1.0, 0.85, 0.02, 1.0))
RUBBER = Material("black_rubber", color=(0.03, 0.03, 0.03, 1.0))


# ── Geometry constants ─────────────────────────────────────────────────────
RAIL_X = 0.25          # half-spacing between rails (m)
RAIL_R = 0.020         # rail radius
RUNG_R = 0.014         # rung radius
RUNG_LEN = 0.54        # rung length (spans between rails with slight overhang)
SEG_LEN = 2.0          # each segment length
N_RUNGS = 6            # rungs per segment
RUNG_SP = 0.30         # rung spacing
RUNG_OFF = 0.20        # offset from segment end to first rung
PIN_R = 0.015          # hinge pin radius
PIN_LEN = 0.56         # hinge pin length (protrudes past rails)
SLV_R = 0.025          # hinge sleeve radius
SLV_LEN = 0.54         # hinge sleeve length (spans between rails, touching both)
SEG0_Z = 4.0           # bottom-z of segment 0 rails (world)


# ── Cylinder helpers (preserved from parent) ───────────────────────────────
def _cyl_x(part, name: str, *, center, length: float, radius: float,
           material=STEEL) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material, name=name,
    )


def _cyl_y(part, name: str, *, center, length: float, radius: float,
           material=STEEL) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material, name=name,
    )


def _cyl_z(part, name: str, *, center, length: float, radius: float,
           material=STEEL) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center),
        material=material, name=name,
    )


# ── Build ──────────────────────────────────────────────────────────────────
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="folding_fire_escape_ladder",
        meta={
            "run_notes": (
                "Multi-section folding fire escape ladder with three rigid rail "
                "segments connected by revolute fold hinges. Top segment is "
                "anchored to the wall via standoffs and anchor plates. Middle "
                "and bottom segments fold flat for stowage and unfold downward "
                "for deployment. Each fold hinge is realised as a visible pin "
                "captured by a coaxial sleeve. Cage omitted per folding-ladder "
                "convention. Replaces the parent prismatic slide with two "
                "REVOLUTE fold hinges."
            ),
        },
    )

    # ── Segment 0 · top, anchored to wall ──────────────────────────────
    seg0 = model.part("ladder_seg_0")

    # Side rails (z = SEG0_Z .. SEG0_Z+SEG_LEN)
    for x, s in [(-RAIL_X, "0"), (RAIL_X, "1")]:
        _cyl_z(seg0, f"fixed_side_rail_{s}",
               center=(x, 0.0, SEG0_Z + SEG_LEN / 2.0),
               length=SEG_LEN, radius=RAIL_R)

    # Rungs via indexed for-loop
    for i in range(N_RUNGS):
        _cyl_x(seg0, f"fixed_rung_{i}",
               center=(0.0, 0.0, SEG0_Z + RUNG_OFF + i * RUNG_SP),
               length=RUNG_LEN, radius=RUNG_R)

    # Wall standoffs and anchor plates (two levels)
    for lv, z in enumerate((SEG0_Z + 0.60, SEG0_Z + 1.60)):
        for x, s in [(-RAIL_X, "0"), (RAIL_X, "1")]:
            _cyl_y(seg0, f"wall_standoff_{s}_{lv}",
                   center=(x, -0.19, z),
                   length=0.38, radius=0.016, material=SHADOW_STEEL)
            seg0.visual(
                Box((0.13, 0.035, 0.11)),
                origin=Origin(xyz=(x, -0.39, z)),
                material=SHADOW_STEEL,
                name=f"wall_anchor_plate_{s}_{lv}",
            )

    # Hinge pin 0 (at bottom of seg-0 rails, spans between rails)
    _cyl_x(seg0, "fold_pin_0",
           center=(0.0, 0.0, SEG0_Z),
           length=PIN_LEN, radius=PIN_R, material=SAFETY_YELLOW)

    # ── Segment 1 · middle, folds from seg-0 ──────────────────────────
    seg1 = model.part("ladder_seg_1")
    # Local frame: origin at hinge, rails extend downward in -Z.
    # At q=0 (deployed) rails span world z = SEG0_Z-SEG_LEN .. SEG0_Z.

    for x, s in [(-RAIL_X, "0"), (RAIL_X, "1")]:
        _cyl_z(seg1, f"seg1_side_rail_{s}",
               center=(x, 0.0, -SEG_LEN / 2.0),
               length=SEG_LEN, radius=RAIL_R)

    for i in range(N_RUNGS):
        _cyl_x(seg1, f"seg1_rung_{i}",
               center=(0.0, 0.0, -(RUNG_OFF + i * RUNG_SP)),
               length=RUNG_LEN, radius=RUNG_R)

    # Sleeve capturing fold_pin_0 (top of seg-1)
    _cyl_x(seg1, "fold_sleeve_0",
           center=(0.0, 0.0, 0.0),
           length=SLV_LEN, radius=SLV_R, material=SHADOW_STEEL)

    # Hinge pin 1 (bottom of seg-1, for seg-2)
    _cyl_x(seg1, "fold_pin_1",
           center=(0.0, 0.0, -SEG_LEN),
           length=PIN_LEN, radius=PIN_R, material=SAFETY_YELLOW)

    # ── Segment 2 · bottom, folds from seg-1 ──────────────────────────
    seg2 = model.part("ladder_seg_2")
    # Local frame: origin at hinge, rails extend downward in -Z.

    for x, s in [(-RAIL_X, "0"), (RAIL_X, "1")]:
        _cyl_z(seg2, f"seg2_side_rail_{s}",
               center=(x, 0.0, -SEG_LEN / 2.0),
               length=SEG_LEN, radius=RAIL_R)

    for i in range(N_RUNGS):
        _cyl_x(seg2, f"seg2_rung_{i}",
               center=(0.0, 0.0, -(RUNG_OFF + i * RUNG_SP)),
               length=RUNG_LEN, radius=RUNG_R)

    # Sleeve capturing fold_pin_1 (top of seg-2)
    _cyl_x(seg2, "fold_sleeve_1",
           center=(0.0, 0.0, 0.0),
           length=SLV_LEN, radius=SLV_R, material=SHADOW_STEEL)

    # Rubber feet at bottom
    for x, s in [(-RAIL_X, "0"), (RAIL_X, "1")]:
        seg2.visual(
            Box((0.06, 0.05, 0.04)),
            origin=Origin(xyz=(x, 0.0, -SEG_LEN)),
            material=RUBBER,
            name=f"rubber_foot_{s}",
        )

    # ── Articulations ──────────────────────────────────────────────────
    # fold_hinge_0: seg-0 → seg-1
    #   q=0 → deployed (seg-1 hangs straight down)
    #   q=π → stowed   (seg-1 folds up alongside seg-0)
    model.articulation(
        "fold_hinge_0",
        ArticulationType.REVOLUTE,
        parent=seg0, child=seg1,
        origin=Origin(xyz=(0.0, 0.0, SEG0_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=50.0, velocity=1.0, lower=0.0, upper=math.pi),
    )

    # fold_hinge_1: seg-1 → seg-2
    model.articulation(
        "fold_hinge_1",
        ArticulationType.REVOLUTE,
        parent=seg1, child=seg2,
        origin=Origin(xyz=(0.0, 0.0, -SEG_LEN)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=50.0, velocity=1.0, lower=0.0, upper=math.pi),
    )

    return model


# ── Tests ──────────────────────────────────────────────────────────────────
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    seg0 = object_model.get_part("ladder_seg_0")
    seg1 = object_model.get_part("ladder_seg_1")
    seg2 = object_model.get_part("ladder_seg_2")
    hinge0 = object_model.get_articulation("fold_hinge_0")
    hinge1 = object_model.get_articulation("fold_hinge_1")

    # ── Overlap allowances for hinge pin/sleeve embeddings ─────────
    ctx.allow_overlap(
        seg0, seg1, elem_a="fold_pin_0", elem_b="fold_sleeve_0",
        reason="Hinge sleeve captures pin; local coaxial overlap "
               "represents the pinned revolute joint.")
    ctx.allow_overlap(
        seg1, seg2, elem_a="fold_pin_1", elem_b="fold_sleeve_1",
        reason="Hinge sleeve captures pin; local coaxial overlap "
               "represents the pinned revolute joint.")

    # Pins pass through child rail ends at the hinge junction
    ctx.allow_overlap(
        seg0, seg1, elem_a="fold_pin_0", elem_b="seg1_side_rail_0",
        reason="Pin passes through the child rail end to form the hinge pivot.")
    ctx.allow_overlap(
        seg0, seg1, elem_a="fold_pin_0", elem_b="seg1_side_rail_1",
        reason="Pin passes through the child rail end to form the hinge pivot.")
    ctx.allow_overlap(
        seg1, seg2, elem_a="fold_pin_1", elem_b="seg2_side_rail_0",
        reason="Pin passes through the child rail end to form the hinge pivot.")
    ctx.allow_overlap(
        seg1, seg2, elem_a="fold_pin_1", elem_b="seg2_side_rail_1",
        reason="Pin passes through the child rail end to form the hinge pivot.")

    # Sleeves contact parent rail ends at the hinge junction
    ctx.allow_overlap(
        seg0, seg1, elem_a="fixed_side_rail_0", elem_b="fold_sleeve_0",
        reason="Sleeve wraps around the hinge pin and contacts the parent "
               "rail end; local overlap represents the hinge barrel junction.")
    ctx.allow_overlap(
        seg0, seg1, elem_a="fixed_side_rail_1", elem_b="fold_sleeve_0",
        reason="Sleeve wraps around the hinge pin and contacts the parent "
               "rail end; local overlap represents the hinge barrel junction.")
    ctx.allow_overlap(
        seg1, seg2, elem_a="seg1_side_rail_0", elem_b="fold_sleeve_1",
        reason="Sleeve wraps around the hinge pin and contacts the parent "
               "rail end; local overlap represents the hinge barrel junction.")
    ctx.allow_overlap(
        seg1, seg2, elem_a="seg1_side_rail_1", elem_b="fold_sleeve_1",
        reason="Sleeve wraps around the hinge pin and contacts the parent "
               "rail end; local overlap represents the hinge barrel junction.")

    # ── Prove hinge-0 pin/sleeve capture ───────────────────────────
    ctx.expect_within(
        seg0, seg1, axes="yz",
        inner_elem="fold_pin_0", outer_elem="fold_sleeve_0",
        margin=0.015,
        name="fold_pin_0 centered in fold_sleeve_0 (YZ)",
    )
    ctx.expect_overlap(
        seg0, seg1, axes="x",
        elem_a="fold_pin_0", elem_b="fold_sleeve_0",
        min_overlap=0.40,
        name="fold_pin_0 axially engaged in fold_sleeve_0",
    )

    # ── Prove hinge-1 pin/sleeve capture ───────────────────────────
    ctx.expect_within(
        seg1, seg2, axes="yz",
        inner_elem="fold_pin_1", outer_elem="fold_sleeve_1",
        margin=0.015,
        name="fold_pin_1 centered in fold_sleeve_1 (YZ)",
    )
    ctx.expect_overlap(
        seg1, seg2, axes="x",
        elem_a="fold_pin_1", elem_b="fold_sleeve_1",
        min_overlap=0.40,
        name="fold_pin_1 axially engaged in fold_sleeve_1",
    )

    # ── Pose checks: fold hinges produce real angular displacement ─
    rest_rail1 = ctx.part_element_world_aabb(seg1, elem="seg1_side_rail_0")
    with ctx.pose({hinge0: 1.5}):
        fold_rail1 = ctx.part_element_world_aabb(seg1, elem="seg1_side_rail_0")
    ctx.check(
        "fold_hinge_0 swings seg_1 through visible arc",
        rest_rail1 is not None and fold_rail1 is not None
        and abs(fold_rail1[0][2] - rest_rail1[0][2]) > 0.5,
        details=f"rest_min_z={rest_rail1[0][2] if rest_rail1 else None}, "
                f"fold_min_z={fold_rail1[0][2] if fold_rail1 else None}",
    )

    rest_rail2 = ctx.part_element_world_aabb(seg2, elem="seg2_side_rail_0")
    with ctx.pose({hinge1: 1.5}):
        fold_rail2 = ctx.part_element_world_aabb(seg2, elem="seg2_side_rail_0")
    ctx.check(
        "fold_hinge_1 swings seg_2 through visible arc",
        rest_rail2 is not None and fold_rail2 is not None
        and abs(fold_rail2[0][2] - rest_rail2[0][2]) > 0.5,
        details=f"rest_min_z={rest_rail2[0][2] if rest_rail2 else None}, "
                f"fold_min_z={fold_rail2[0][2] if fold_rail2 else None}",
    )

    # ── Rung loop existence on each segment ────────────────────────
    for seg_name, prefix, n in [
        ("ladder_seg_0", "fixed_rung", N_RUNGS),
        ("ladder_seg_1", "seg1_rung", N_RUNGS),
        ("ladder_seg_2", "seg2_rung", N_RUNGS),
    ]:
        seg = object_model.get_part(seg_name)
        rung_vis = [v for v in seg.visuals if v.name.startswith(prefix + "_")]
        ctx.check(
            f"{seg_name} emits {n} loop-indexed rungs ({prefix}_*)",
            len(rung_vis) == n,
            details=f"found {len(rung_vis)}: {[v.name for v in rung_vis]}",
        )

    return ctx.report()


object_model = build_object_model()
