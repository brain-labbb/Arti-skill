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
DARK_METAL = Material("dark_hardware", rgba=(0.08, 0.085, 0.085, 1.0))
BLACK_PLASTIC = Material("black_plastic_caps", rgba=(0.01, 0.01, 0.01, 1.0))
PALE_PLASTIC = Material("pale_brackets", rgba=(0.78, 0.80, 0.80, 1.0))
CORD_MATERIAL = Material("nylon_suspension_cord", rgba=(0.85, 0.83, 0.78, 1.0))


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
    axis_len = math.sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2])
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
    part,
    name: str,
    start: Iterable[float],
    end: Iterable[float],
    radius: float,
    material: Material = ALUMINUM,
    *,
    radial_segments: int = 20,
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
# Geometry constants
# ---------------------------------------------------------------------------
CEILING_Z = 2.35
RAIL_HALF_X = 0.65
RAIL_SPACING = 0.08

DECK_HALF_X = 0.55
DECK_HALF_Y = 0.22
BAR_COUNT = 6
CORD_HEIGHT = 0.72

# Joint: at q=0 the bar deck hangs at its lowest (loading) position.
# Positive q raises the deck toward the ceiling.
LIFT_ORIGIN_Z = 1.55
LIFT_UPPER = 0.55


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="ceiling_pulley_clothes_airer",
        meta={
            "category_note": (
                "Ceiling-suspended pulley airer variant of the laundry drying rack family. "
                "Mounting topology changed from floor-standing X-frame legs to overhead "
                "ceiling suspension with a prismatic vertical lift joint."
            )
        },
    )

    # ==================================================================
    # main_frame: fixed ceiling mounting rail with end brackets
    # ==================================================================
    rail = model.part("main_frame")

    # Two parallel ceiling rails along X
    _tube_visual(
        rail, "ceiling_rail_front",
        (-RAIL_HALF_X, RAIL_SPACING, CEILING_Z),
        (RAIL_HALF_X, RAIL_SPACING, CEILING_Z),
        0.012, ALUMINUM,
    )
    _tube_visual(
        rail, "ceiling_rail_rear",
        (-RAIL_HALF_X, -RAIL_SPACING, CEILING_Z),
        (RAIL_HALF_X, -RAIL_SPACING, CEILING_Z),
        0.012, ALUMINUM,
    )

    # Cross braces connecting the two rails (including pulley-mount positions)
    for i, x in enumerate((-0.55, -0.20, 0.20, 0.55)):
        _tube_visual(
            rail, f"rail_cross_{i}",
            (x, -RAIL_SPACING, CEILING_Z),
            (x, RAIL_SPACING, CEILING_Z),
            0.010, ALUMINUM,
        )

    # End mounting brackets (ceiling plates)
    for i, x_end in enumerate((-RAIL_HALF_X - 0.02, RAIL_HALF_X + 0.02)):
        rail.visual(
            Box((0.055, 0.18, 0.032)),
            origin=Origin(xyz=(x_end, 0.0, CEILING_Z + 0.016)),
            material=PALE_PLASTIC,
            name=f"ceiling_bracket_{i}",
        )
        # Fastener bolts on each bracket
        for dy in (-0.050, 0.050):
            rail.visual(
                Box((0.014, 0.014, 0.008)),
                origin=Origin(xyz=(x_end, dy, CEILING_Z + 0.036)),
                material=DARK_METAL,
                name=f"bracket_bolt_{i}_{dy:+.3f}",
            )

    # Pulley guide tubes (short vertical tubes where suspension cords pass)
    for i, x in enumerate((-0.55, 0.55)):
        _tube_visual(
            rail, f"pulley_guide_{i}",
            (x, 0.0, CEILING_Z - 0.065),
            (x, 0.0, CEILING_Z),
            0.009, DARK_METAL, radial_segments=14,
        )
        # Small pulley wheel housing
        rail.visual(
            Box((0.035, 0.040, 0.030)),
            origin=Origin(xyz=(x, 0.0, CEILING_Z - 0.065 - 0.015)),
            material=DARK_METAL,
            name=f"pulley_housing_{i}",
        )

    # ==================================================================
    # front_wing: hanging bar deck with suspension cords
    # ==================================================================
    deck = model.part("front_wing")

    # Bar deck ladder frame (single merged mesh named wing_tube_frame)
    segments: list[tuple[tuple[float, float, float], tuple[float, float, float], float]] = []

    # Side rails (along Y)
    segments.append(((-DECK_HALF_X, -DECK_HALF_Y, 0.0), (-DECK_HALF_X, DECK_HALF_Y, 0.0), 0.010))
    segments.append(((DECK_HALF_X, -DECK_HALF_Y, 0.0), (DECK_HALF_X, DECK_HALF_Y, 0.0), 0.010))
    # End rails (along X)
    segments.append(((-DECK_HALF_X, -DECK_HALF_Y, 0.0), (DECK_HALF_X, -DECK_HALF_Y, 0.0), 0.009))
    segments.append(((-DECK_HALF_X, DECK_HALF_Y, 0.0), (DECK_HALF_X, DECK_HALF_Y, 0.0), 0.009))

    # Parallel drying bars spanning X, evenly spaced in Y
    for i in range(BAR_COUNT):
        t = (i + 0.5) / BAR_COUNT
        y = -DECK_HALF_Y + 2.0 * DECK_HALF_Y * t
        segments.append((
            (-DECK_HALF_X, y, 0.0),
            (DECK_HALF_X, y, 0.0),
            0.008,
        ))

    _straight_frame_visual(deck, "wing_tube_frame", segments, ALUMINUM)

    # Suspension cords: two vertical cords from bar deck side rails up to ceiling
    # Placed at side rail midpoints (x=±0.55, y=0) to connect to the side rails.
    for i, x in enumerate((-DECK_HALF_X, DECK_HALF_X)):
        _tube_visual(
            deck, f"suspension_cord_{i}",
            (x, 0.0, -0.005),
            (x, 0.0, CORD_HEIGHT),
            0.004, CORD_MATERIAL, radial_segments=10,
        )
        # Cord clip at top
        deck.visual(
            Box((0.022, 0.022, 0.016)),
            origin=Origin(xyz=(x, 0.0, CORD_HEIGHT - 0.002)),
            material=DARK_METAL,
            name=f"cord_clip_{i}",
        )
        # Cord anchor ring at deck connection
        deck.visual(
            Box((0.020, 0.020, 0.014)),
            origin=Origin(xyz=(x, 0.0, -0.005)),
            material=DARK_METAL,
            name=f"cord_anchor_{i}",
        )

    # Corner end caps on the bar deck
    for i, (cx, cy) in enumerate([
        (-DECK_HALF_X, -DECK_HALF_Y),
        (-DECK_HALF_X, DECK_HALF_Y),
        (DECK_HALF_X, -DECK_HALF_Y),
        (DECK_HALF_X, DECK_HALF_Y),
    ]):
        deck.visual(
            Box((0.026, 0.026, 0.022)),
            origin=Origin(xyz=(cx, cy, 0.0)),
            material=BLACK_PLASTIC,
            name=f"deck_corner_cap_{i}",
        )

    # ==================================================================
    # Prismatic joint: vertical lift
    # ==================================================================
    model.articulation(
        "frame_to_front_wing",
        ArticulationType.PRISMATIC,
        parent=rail,
        child=deck,
        origin=Origin(xyz=(0.0, 0.0, LIFT_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0, velocity=0.15,
            lower=0.0, upper=LIFT_UPPER,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    rail = object_model.get_part("main_frame")
    deck = object_model.get_part("front_wing")
    lift_joint = object_model.get_articulation("frame_to_front_wing")

    # --- Allowances for suspension cord and clip seating into pulley housings ---
    for i in range(2):
        ctx.allow_overlap(
            "front_wing",
            "main_frame",
            elem_a=f"cord_clip_{i}",
            elem_b=f"pulley_housing_{i}",
            reason=(
                "The cord clip seats into the pulley housing at rest position, "
                "representing the captured end of the suspension cord retracted "
                "into the ceiling pulley mechanism."
            ),
        )
        ctx.allow_overlap(
            "front_wing",
            "main_frame",
            elem_a=f"suspension_cord_{i}",
            elem_b=f"pulley_housing_{i}",
            reason=(
                "The upper end of the suspension cord feeds into the pulley housing, "
                "representing the cord retraction path through the ceiling-mounted "
                "pulley guide."
            ),
        )

    # --- Proof: cord clips and cords are seated in pulley housings at rest ---
    for i in range(2):
        ctx.expect_contact(
            deck,
            rail,
            elem_a=f"cord_clip_{i}",
            elem_b=f"pulley_housing_{i}",
            name=f"cord_clip_{i} seats in pulley_housing_{i} at rest",
        )
        ctx.expect_contact(
            deck,
            rail,
            elem_a=f"suspension_cord_{i}",
            elem_b=f"pulley_housing_{i}",
            name=f"suspension_cord_{i} feeds into pulley_housing_{i} at rest",
        )

    # --- Structural identity ---
    ctx.check(
        "airer has ceiling rail and one hanging bar deck",
        len(object_model.parts) == 2 and len(object_model.articulations) == 1,
        details=f"parts={len(object_model.parts)}, joints={len(object_model.articulations)}",
    )

    # --- Joint type: frame_to_front_wing must be PRISMATIC ---
    ctx.check(
        "frame_to_front_wing is a prismatic vertical lift joint",
        lift_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={lift_joint.articulation_type}",
    )

    # --- Bar deck hangs below the ceiling rail at rest ---
    ctx.expect_gap(
        rail,
        deck,
        axis="z",
        min_gap=0.35,
        positive_elem="ceiling_rail_front",
        negative_elem="wing_tube_frame",
        name="bar deck hangs well below the ceiling rail at rest (lowered position)",
    )

    # --- Bar deck spans most of the ceiling rail width ---
    ctx.expect_overlap(
        deck,
        rail,
        axes="x",
        min_overlap=0.9,
        elem_a="wing_tube_frame",
        elem_b="ceiling_rail_front",
        name="wing_tube_frame bar deck spans most of the ceiling rail width",
    )

    # --- Prismatic motion raises the bar deck ---
    rest_aabb = ctx.part_world_aabb(deck)
    with ctx.pose({lift_joint: LIFT_UPPER}):
        raised_aabb = ctx.part_world_aabb(deck)

    ctx.check(
        "positive prismatic q raises the bar deck toward the ceiling rail",
        rest_aabb is not None
        and raised_aabb is not None
        and raised_aabb[0][2] > rest_aabb[0][2] + 0.40,
        details=(
            f"rest_min_z={rest_aabb[0][2] if rest_aabb else None}, "
            f"raised_min_z={raised_aabb[0][2] if raised_aabb else None}"
        ),
    )

    # --- At raised position, bar deck is closer to the ceiling rail ---
    with ctx.pose({lift_joint: LIFT_UPPER}):
        ctx.expect_gap(
            rail,
            deck,
            axis="z",
            max_gap=0.30,
            positive_elem="ceiling_rail_front",
            negative_elem="wing_tube_frame",
            name="raised bar deck sits close to the ceiling rail",
        )

    return ctx.report()


object_model = build_object_model()
