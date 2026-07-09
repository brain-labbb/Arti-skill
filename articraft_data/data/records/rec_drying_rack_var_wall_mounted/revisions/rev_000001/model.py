from __future__ import annotations

import math
from typing import Iterable

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CylinderGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


ALUMINUM = Material("brushed_aluminum", rgba=(0.72, 0.74, 0.74, 1.0))
DARK_METAL = Material("dark_steel_hardware", rgba=(0.12, 0.12, 0.13, 1.0))
BLACK_PLASTIC = Material("black_plastic_caps", rgba=(0.02, 0.02, 0.02, 1.0))
WALL_MOUNT = Material("zinc_mounting_plates", rgba=(0.55, 0.56, 0.58, 1.0))


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _as_vec(point: Iterable[float]) -> tuple[float, float, float]:
    x, y, z = point
    return (float(x), float(y), float(z))


def _cylinder_between(
    start: Iterable[float],
    end: Iterable[float],
    radius: float,
    *,
    radial_segments: int = 20,
) -> MeshGeometry:
    """Closed circular tube between two local points."""
    p0 = _as_vec(start)
    p1 = _as_vec(end)
    vx = p1[0] - p0[0]
    vy = p1[1] - p0[1]
    vz = p1[2] - p0[2]
    length = math.sqrt(vx * vx + vy * vy + vz * vz)
    if length <= 1.0e-8:
        raise ValueError("tube endpoints must be distinct")

    geom = CylinderGeometry(radius, length, radial_segments=radial_segments, closed=True)
    direction = (vx / length, vy / length, vz / length)
    dot = max(-1.0, min(1.0, direction[2]))
    angle = math.acos(dot)
    axis = (-direction[1], direction[0], 0.0)
    axis_len = math.sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
    if axis_len > 1.0e-8:
        geom.rotate((axis[0] / axis_len, axis[1] / axis_len, axis[2] / axis_len), angle)
    elif direction[2] < 0.0:
        geom.rotate((1.0, 0.0, 0.0), math.pi)
    geom.translate(
        (p0[0] + p1[0]) * 0.5,
        (p0[1] + p1[1]) * 0.5,
        (p0[2] + p1[2]) * 0.5,
    )
    return geom


def _tube_visual(
    part, name: str, start, end, radius: float,
    material: Material = ALUMINUM, *, radial_segments: int = 20,
) -> None:
    part.visual(
        mesh_from_geometry(
            _cylinder_between(start, end, radius, radial_segments=radial_segments),
            name,
        ),
        material=material,
        name=name,
    )


def _merged_tubes(
    segments: Iterable[tuple[tuple[float, float, float], tuple[float, float, float], float]],
    *,
    radial_segments: int = 20,
) -> MeshGeometry:
    merged = MeshGeometry()
    for start, end, radius in segments:
        merged.merge(_cylinder_between(start, end, radius, radial_segments=radial_segments))
    return merged


def _straight_frame_visual(part, name: str, segments, material=ALUMINUM) -> None:
    part.visual(
        mesh_from_geometry(_merged_tubes(segments, radial_segments=18), name),
        material=material,
        name=name,
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="wall_mounted_drying_rack",
        meta={
            "category_note": (
                "Wall-mounted fold-down laundry drying rack. "
                "Category: Household_Laundry / Clothes drying rack."
            ),
        },
    )

    # ──────────────────────────────────────────────────────────────────────
    # main_frame — fixed rectangular wall bracket
    # ──────────────────────────────────────────────────────────────────────
    bracket = model.part("main_frame")

    hw = 0.40           # half-width  (total width 0.80 m)
    by = 0.012          # tube-centre offset from wall surface (y = 0)
    zb = 1.10           # bottom-rail height  (hinge line)
    zt = 1.60           # top-rail height

    # Vertical side posts
    _tube_visual(bracket, "bracket_left_post",
                 (-hw, by, zb), (-hw, by, zt), 0.012)
    _tube_visual(bracket, "bracket_right_post",
                 (hw, by, zb), (hw, by, zt), 0.012)

    # Horizontal rails (top and bottom-hinge)
    _tube_visual(bracket, "bracket_top_rail",
                 (-hw, by, zt), (hw, by, zt), 0.011)
    _tube_visual(bracket, "bracket_bottom_rail",
                 (-hw, by, zb), (hw, by, zb), 0.012)

    # Short diagonal gussets in the upper corners only (clear the wing sweep)
    _tube_visual(bracket, "gusset_left",
                 (-hw, by, zt - 0.12), (-hw + 0.12, by, zt),
                 0.006, DARK_METAL, radial_segments=14)
    _tube_visual(bracket, "gusset_right",
                 (hw, by, zt - 0.12), (hw - 0.12, by, zt),
                 0.006, DARK_METAL, radial_segments=14)

    # Wall-mounting plates at four corners
    for i, (x, z) in enumerate(((-hw, zb), (-hw, zt), (hw, zb), (hw, zt))):
        bracket.visual(
            Box((0.060, 0.006, 0.080)),
            origin=Origin(xyz=(x, 0.003, z)),
            material=WALL_MOUNT,
            name=f"mount_plate_{i}",
        )

    # Hinge knuckles (bracket-side hardware on the bottom rail)
    hinge_xs = (-0.28, 0.0, 0.28)
    for i, x in enumerate(hinge_xs):
        bracket.visual(
            Box((0.038, 0.030, 0.032)),
            origin=Origin(xyz=(x, by, zb)),
            material=DARK_METAL,
            name=f"hinge_knuckle_{i}",
        )

    # Plastic caps on top of the posts
    for i, x in enumerate((-hw, hw)):
        bracket.visual(
            Box((0.026, 0.026, 0.020)),
            origin=Origin(xyz=(x, by, zt + 0.010)),
            material=BLACK_PLASTIC,
            name=f"post_cap_{i}",
        )

    # ──────────────────────────────────────────────────────────────────────
    # front_wing — swing-down drying frame
    # ──────────────────────────────────────────────────────────────────────
    wing = model.part("front_wing")

    ww = 0.38           # half-width  (slightly narrower than bracket)
    wd = 0.48           # frame depth when deployed
    yo = 0.028          # y-offset so the frame clears the bracket when folded

    # Build the wing in its own local frame.
    # At q = 0 the wing extends in +Z (folded upward against the wall).
    # The hinge tube sits at local z = 0, along X.
    segments = [
        # Hinge tube (coaxial with bracket_bottom_rail)
        ((-ww, 0.0, 0.0), (ww, 0.0, 0.0), 0.011),
        # Top rail
        ((-ww, yo, wd), (ww, yo, wd), 0.010),
        # Side rails
        ((-ww, 0.0, 0.0), (-ww, yo, wd), 0.010),
        ((ww, 0.0, 0.0),  (ww, yo, wd),  0.010),
    ]

    # Six parallel horizontal drying bars (flush with the side rails)
    n_bars = 6
    for i in range(n_bars):
        t = (i + 1) / (n_bars + 1)
        segments.append((
            (-ww, yo * t, wd * t),
            (ww, yo * t, wd * t),
            0.007,
        ))

    _straight_frame_visual(wing, "wing_tube_frame", segments, ALUMINUM)

    # Hinge sleeves (wing-side barrel tubes wrapping around knuckles)
    for i, x in enumerate(hinge_xs):
        _tube_visual(
            wing, f"hinge_sleeve_{i}",
            (x - 0.020, 0.0, 0.0), (x + 0.020, 0.0, 0.0),
            0.016, DARK_METAL, radial_segments=16,
        )

    # End caps on the top-rail corners
    for i, x in enumerate((-ww, ww)):
        wing.visual(
            Box((0.024, 0.024, 0.024)),
            origin=Origin(xyz=(x, yo, wd)),
            material=BLACK_PLASTIC,
            name=f"wing_cap_{i}",
        )

    # ──────────────────────────────────────────────────────────────────────
    # Articulation: REVOLUTE hinge at the bracket bottom rail
    # ──────────────────────────────────────────────────────────────────────
    # The wing's local +Z points upward at q = 0 (folded against wall).
    # axis = (-1, 0, 0): right-hand rule around -X takes +Z toward +Y,
    # so positive q deploys the wing outward from the wall.
    model.articulation(
        "frame_to_front_wing",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=wing,
        origin=Origin(xyz=(0.0, by, zb)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=1.5, lower=0.0, upper=1.50,
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bracket = object_model.get_part("main_frame")
    wing    = object_model.get_part("front_wing")
    hinge   = object_model.get_articulation("frame_to_front_wing")

    # ── Intentional hinge overlaps ──────────────────────────────────────
    ctx.allow_overlap(
        "front_wing", "main_frame",
        elem_a="wing_tube_frame",
        elem_b="bracket_bottom_rail",
        reason=(
            "The wing hinge tube is intentionally coaxial with the "
            "bracket bottom rail to form the hinge pivot."
        ),
    )
    for i in range(3):
        ctx.allow_overlap(
            "front_wing", "main_frame",
            elem_a=f"hinge_sleeve_{i}",
            elem_b=f"hinge_knuckle_{i}",
            reason=(
                f"Hinge sleeve {i} wraps around hinge knuckle {i} "
                "as the barrel pivot."
            ),
        )
        ctx.allow_overlap(
            "front_wing", "main_frame",
            elem_a=f"hinge_sleeve_{i}",
            elem_b="bracket_bottom_rail",
            reason=(
                f"Hinge sleeve {i} is coaxial with the bracket bottom rail "
                "to form the hinge barrel pivot."
            ),
        )
        ctx.allow_overlap(
            "front_wing", "main_frame",
            elem_a="wing_tube_frame",
            elem_b=f"hinge_knuckle_{i}",
            reason=(
                f"The wing hinge tube passes through hinge knuckle {i} "
                "to form the barrel pivot axis."
            ),
        )

    # Proof: the hinge elements are in contact (seated, not separated)
    ctx.expect_contact(
        wing, bracket,
        elem_a="wing_tube_frame",
        elem_b="bracket_bottom_rail",
        name="wing hinge tube contacts bracket bottom rail at rest",
    )

    # ── Structural identity ─────────────────────────────────────────────
    mount_plate_count = sum(
        1 for v in bracket.visuals if v.name and v.name.startswith("mount_plate_")
    )
    ctx.check(
        "main_frame is a wall bracket with four mounting plates",
        mount_plate_count == 4,
        details=f"Found {mount_plate_count} mounting plates (expected 4).",
    )

    ctx.check(
        "model has two parts (bracket + wing) and one REVOLUTE joint",
        len(object_model.parts) == 2 and len(object_model.articulations) == 1,
        details=(
            f"parts={len(object_model.parts)}, "
            f"joints={len(object_model.articulations)}"
        ),
    )

    ctx.check(
        "front_wing carries the wing_tube_frame bar array",
        any(v.name == "wing_tube_frame" for v in wing.visuals),
        details="Missing wing_tube_frame visual on front_wing.",
    )

    # ── Bracket has a real vertical span between top and bottom rails ──
    ctx.expect_gap(
        bracket, bracket,
        axis="z",
        min_gap=0.40,
        positive_elem="bracket_top_rail",
        negative_elem="bracket_bottom_rail",
        name="bracket spans a significant vertical height on the wall",
    )

    # ── Pose: positive hinge angle deploys wing outward from wall (+Y) ──
    rest_aabb = ctx.part_world_aabb(wing)
    with ctx.pose({hinge: 1.0}):
        deployed_aabb = ctx.part_world_aabb(wing)
    ctx.check(
        "positive hinge angle deploys wing outward from wall (+Y)",
        rest_aabb is not None
        and deployed_aabb is not None
        and deployed_aabb[1][1] > rest_aabb[1][1] + 0.10,
        details=(
            f"rest_max_y={rest_aabb and rest_aabb[1][1]:.3f}, "
            f"deployed_max_y={deployed_aabb and deployed_aabb[1][1]:.3f}"
        ),
    )

    # ── Pose: fully deployed wing is near horizontal ────────────────────
    with ctx.pose({hinge: 1.50}):
        full_deploy_aabb = ctx.part_world_aabb(wing)
    ctx.check(
        "fully deployed wing has small Z extent (near horizontal)",
        full_deploy_aabb is not None
        and (full_deploy_aabb[1][2] - full_deploy_aabb[0][2]) < 0.15,
        details=(
            f"deployed_z_extent="
            f"{full_deploy_aabb and full_deploy_aabb[1][2] - full_deploy_aabb[0][2]:.3f}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
