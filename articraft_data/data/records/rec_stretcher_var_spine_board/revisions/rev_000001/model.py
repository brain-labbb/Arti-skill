from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
    tube_from_spline_points,
)


# ---------------------------------------------------------------------------
# Preserved helper functions from parent (KEEP: _tube, _rounded_pad_mesh,
# tube_from_spline_points)
# ---------------------------------------------------------------------------

def _origin_for_cylinder_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    extend: float = 0.0,
) -> tuple[Origin, float]:
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 1e-9:
        raise ValueError("tube endpoints must be separated")
    ux, uy, uz = dx / length, dy / length, dz / length
    sx -= ux * extend
    sy -= uy * extend
    sz -= uz * extend
    ex += ux * extend
    ey += uy * extend
    ez += uz * extend
    length += 2.0 * extend
    mx, my, mz = (sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5
    yaw = math.atan2(uy, ux)
    pitch = math.atan2(math.sqrt(ux * ux + uy * uy), uz)
    return Origin(xyz=(mx, my, mz), rpy=(0.0, pitch, yaw)), length


def _tube(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    material: Material,
    name: str,
    *,
    extend: float = 0.002,
) -> None:
    origin, length = _origin_for_cylinder_between(start, end, extend=extend)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=origin,
        material=material,
        name=name,
    )


def _rounded_pad_mesh(width: float, length: float, thickness: float, radius: float, name: str):
    profile = rounded_rect_profile(length, width, radius, corner_segments=8)
    return mesh_from_geometry(ExtrudeGeometry.centered(profile, thickness), name)


# ---------------------------------------------------------------------------
# Board geometry helpers
# ---------------------------------------------------------------------------

def _translate_profile(profile, dx: float, dy: float):
    """Translate a 2D profile by (dx, dy)."""
    return [(x + dx, y + dy) for x, y in profile]


def _make_spine_board_mesh():
    """Build a flat rigid spine board with oval hand-hole cutouts along both edges."""
    board_length = 1.83
    board_width = 0.41
    board_thickness = 0.05
    corner_radius = 0.025

    outer = rounded_rect_profile(board_length, board_width, corner_radius, corner_segments=8)

    # 5 pairs of oval hand holes along both long edges (10 total).
    hole_profiles = []
    hole_x_positions = [-0.58, -0.30, -0.02, 0.26, 0.54]
    edge_y_offset = 0.155  # distance from board center to hole center

    for hx in hole_x_positions:
        for sy in (+edge_y_offset, -edge_y_offset):
            hole = superellipse_profile(0.10, 0.030, exponent=2.0, segments=24)
            hole = _translate_profile(hole, hx, sy)
            hole_profiles.append(hole)

    board_geom = ExtrudeWithHolesGeometry(
        outer, hole_profiles, board_thickness, center=True,
    )
    return mesh_from_geometry(board_geom, "spine_board")


# ---------------------------------------------------------------------------
# Head immobilizer block geometry helper
# ---------------------------------------------------------------------------

def _add_immobilizer_block(part, *, side_sign: float, orange, padded_black, metal) -> None:
    """Add head immobilizer ear-flap block geometry to a child part.

    The child frame origin is at the hinge pivot.  At q=0 the block stands
    upright (deployed / locking position).  Positive q folds it outward.

    side_sign: +1 for the +Y block, -1 for the -Y block.
    """
    inward_y = -0.028 * side_sign  # block extends inward from hinge

    # Rigid base plate (same HDPE as the board)
    part.visual(
        Box((0.14, 0.055, 0.10)),
        origin=Origin(xyz=(0.0, inward_y, 0.050)),
        material=orange,
        name="block_base",
    )
    # Foam pad on the inner face (rotated to face the Y direction).
    # The pad is seated against the block base with a small intentional
    # overlap (~2.5 mm) representing compressed foam on the rigid plate.
    pad_mesh = _rounded_pad_mesh(0.08, 0.12, 0.018, 0.012, f"block_pad")
    part.visual(
        pad_mesh,
        origin=Origin(
            xyz=(0.0, inward_y - 0.034 * side_sign, 0.050),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=padded_black,
        name="block_pad",
    )
    # Small hinge tab at the pivot
    part.visual(
        Box((0.040, 0.014, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=metal,
        name="hinge_tab",
    )


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="rigid_spine_backboard",
        meta={
            "run_notes": (
                "Rigid spinal backboard (long spine board) for trauma "
                "immobilization and transfer.  High-visibility orange HDPE "
                "colorway with hand-hole perimeter, hinged head-immobilizer "
                "ear-flap blocks, and spider strap set.  No wheels or legs."
            ),
        },
    )

    # --- Materials ---
    orange_hdpe = model.material("orange_hdpe", rgba=(1.0, 0.45, 0.05, 1.0))
    black = model.material("matte_black", rgba=(0.02, 0.02, 0.02, 1.0))
    padded_black = model.material("soft_black_foam", rgba=(0.04, 0.04, 0.04, 1.0))
    strap_nylon = model.material("dark_nylon_strap", rgba=(0.08, 0.08, 0.09, 1.0))
    buckle_red = model.material("buckle_red", rgba=(0.80, 0.10, 0.05, 1.0))
    metal = model.material("brushed_metal", rgba=(0.65, 0.66, 0.62, 1.0))

    # ===================================================================
    # deck_frame: flat rigid spine board (root part)
    # ===================================================================
    deck = model.part("deck_frame")

    # Main board with hand-hole cutouts
    deck.visual(
        _make_spine_board_mesh(),
        material=orange_hdpe,
        name="board",
    )

    # Head bed pad — thin foam layer at the head end
    deck.visual(
        _rounded_pad_mesh(0.28, 0.30, 0.006, 0.04, "head_bed_pad"),
        origin=Origin(xyz=(-0.72, 0.0, 0.028)),
        material=padded_black,
        name="head_bed_pad",
    )

    # Spider strap set — 3 straps across the body
    strap_x_positions = [-0.32, 0.12, 0.56]
    for i, sx in enumerate(strap_x_positions):
        # Main strap body
        deck.visual(
            Box((0.038, 0.38, 0.003)),
            origin=Origin(xyz=(sx, 0.0, 0.027)),
            material=strap_nylon,
            name=f"strap_{i}",
        )
        # Quick-release buckle
        deck.visual(
            Box((0.048, 0.038, 0.010)),
            origin=Origin(xyz=(sx, 0.175, 0.031)),
            material=buckle_red,
            name=f"buckle_{i}",
        )
        # Keeper / retainer on opposite side
        deck.visual(
            Box((0.042, 0.032, 0.008)),
            origin=Origin(xyz=(sx, -0.178, 0.029)),
            material=metal,
            name=f"keeper_{i}",
        )

    # Strap anchor loops at board edges (using preserved _tube helper)
    for i, sx in enumerate(strap_x_positions):
        _tube(
            deck,
            (sx - 0.012, 0.198, 0.028),
            (sx + 0.012, 0.198, 0.028),
            0.003, metal, f"strap_loop_{i}_0", extend=0.0,
        )
        _tube(
            deck,
            (sx - 0.012, -0.198, 0.028),
            (sx + 0.012, -0.198, 0.028),
            0.003, metal, f"strap_loop_{i}_1", extend=0.0,
        )

    # Carry handles at head and foot ends (using preserved tube_from_spline_points)
    head_handle_geom = tube_from_spline_points(
        [
            (-0.89, -0.14, 0.025),
            (-0.93, -0.14, 0.045),
            (-0.93, 0.14, 0.045),
            (-0.89, 0.14, 0.025),
        ],
        radius=0.008,
        samples_per_segment=8,
        radial_segments=14,
        cap_ends=True,
    )
    deck.visual(
        mesh_from_geometry(head_handle_geom, "head_handle"),
        material=black,
        name="head_handle",
    )

    foot_handle_geom = tube_from_spline_points(
        [
            (0.89, -0.14, 0.025),
            (0.93, -0.14, 0.045),
            (0.93, 0.14, 0.045),
            (0.89, 0.14, 0.025),
        ],
        radius=0.008,
        samples_per_segment=8,
        radial_segments=14,
        cap_ends=True,
    )
    deck.visual(
        mesh_from_geometry(foot_handle_geom, "foot_handle"),
        material=black,
        name="foot_handle",
    )

    # ===================================================================
    # side_rail_0 / side_rail_1: head immobilizer ear-flap blocks
    # ===================================================================
    rail_0 = model.part("side_rail_0")
    rail_1 = model.part("side_rail_1")
    _add_immobilizer_block(rail_0, side_sign=+1.0, orange=orange_hdpe, padded_black=padded_black, metal=metal)
    _add_immobilizer_block(rail_1, side_sign=-1.0, orange=orange_hdpe, padded_black=padded_black, metal=metal)

    # ===================================================================
    # Articulations (repurposed from parent deck_to_side_rail_0 / _1)
    # ===================================================================
    head_x = -0.73
    edge_y = 0.195

    # At q=0: block stands upright (deployed, locking the head).
    # Positive q: block folds outward / flat (storage position).
    model.articulation(
        "deck_to_side_rail_0",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=rail_0,
        origin=Origin(xyz=(head_x, edge_y, 0.025)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=1.45),
    )
    model.articulation(
        "deck_to_side_rail_1",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=rail_1,
        origin=Origin(xyz=(head_x, -edge_y, 0.025)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=1.45),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck_frame")
    rail_0 = object_model.get_part("side_rail_0")
    rail_1 = object_model.get_part("side_rail_1")

    joint_0 = object_model.get_articulation("deck_to_side_rail_0")
    joint_1 = object_model.get_articulation("deck_to_side_rail_1")

    # --- Board features ---
    ctx.check(
        "spine board main board visual exists",
        deck.get_visual("board") is not None,
    )
    ctx.check(
        "head bed pad exists on the board",
        deck.get_visual("head_bed_pad") is not None,
    )

    # --- Spider strap set ---
    for i in range(3):
        ctx.check(
            f"strap_{i} exists on the board",
            deck.get_visual(f"strap_{i}") is not None,
        )

    # --- Head immobilizer blocks fold outward for storage ---
    # At q=0 (deployed), blocks stand upright near the board center line.
    # At positive q (stowed), blocks fold outward past the board edge.
    rail_0_deployed = ctx.part_world_aabb(rail_0)
    rail_1_deployed = ctx.part_world_aabb(rail_1)

    with ctx.pose({joint_0: 1.30, joint_1: 1.30}):
        rail_0_stowed = ctx.part_world_aabb(rail_0)
        rail_1_stowed = ctx.part_world_aabb(rail_1)

    ctx.check(
        "head immobilizer block 0 (side_rail_0) folds outward when stowed",
        rail_0_deployed is not None
        and rail_0_stowed is not None
        and rail_0_stowed[1][1] > rail_0_deployed[1][1] + 0.03,
        details=f"deployed_max_y={rail_0_deployed[1] if rail_0_deployed else None}, "
                f"stowed_max_y={rail_0_stowed[1] if rail_0_stowed else None}",
    )
    ctx.check(
        "head immobilizer block 1 (side_rail_1) folds outward when stowed",
        rail_1_deployed is not None
        and rail_1_stowed is not None
        and rail_1_stowed[0][1] < rail_1_deployed[0][1] - 0.03,
        details=f"deployed_min_y={rail_1_deployed[0] if rail_1_deployed else None}, "
                f"stowed_min_y={rail_1_stowed[0] if rail_1_stowed else None}",
    )

    # --- Hinge joints are revolute (not fixed) ---
    ctx.check(
        "deck_to_side_rail_0 is a revolute hinge",
        joint_0.articulation_type == ArticulationType.REVOLUTE,
    )
    ctx.check(
        "deck_to_side_rail_1 is a revolute hinge",
        joint_1.articulation_type == ArticulationType.REVOLUTE,
    )

    # --- No wheeled undercarriage or legs remain ---
    removed_parts = (
        "head_leg", "foot_leg",
        "caster_0", "caster_1", "caster_2", "caster_3",
        "backrest",
    )
    for pname in removed_parts:
        found = True
        try:
            object_model.get_part(pname)
        except Exception:
            found = False
        ctx.check(
            f"removed part '{pname}' does not exist on spine board",
            not found,
            details=f"{pname} was found but should have been removed",
        )

    return ctx.report()


object_model = build_object_model()
