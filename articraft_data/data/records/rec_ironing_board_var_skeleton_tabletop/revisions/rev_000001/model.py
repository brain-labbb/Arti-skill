from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


# ── Compact tabletop board dimensions ──────────────────────────────────────
BOARD_LENGTH = 0.80
BOARD_WIDTH = 0.32
BOARD_NOSE_X = -0.40
BOARD_REAR_X = 0.40
REAR_CAP_START = 0.35

# ── Short leg frame ───────────────────────────────────────────────────────
LEG_HEIGHT = 0.12          # tabletop stance: ~0.17 m board-top to foot
HINGE_Z = -0.048           # hinge line below board underside


# ───────────────────────────────────────────────────────────────────────────
# Board planform (same tapered family, compact scale)
# ───────────────────────────────────────────────────────────────────────────

def _half_width_at_x(x: float, *, scale: float = 1.0) -> float:
    """Plan-view ironing-board half width: sharp nose, broad rounded rear."""
    max_half = (BOARD_WIDTH * scale) * 0.5
    if x <= BOARD_NOSE_X:
        return 0.0
    if x >= REAR_CAP_START:
        rx = max(0.001, BOARD_REAR_X - REAR_CAP_START)
        u = min(1.0, max(0.0, (x - REAR_CAP_START) / rx))
        return max_half * math.sqrt(max(0.0, 1.0 - u * u))
    t = (x - BOARD_NOSE_X) / (REAR_CAP_START - BOARD_NOSE_X)
    return max_half * (math.sin(t * math.pi * 0.5) ** 0.56) * (0.985 + 0.015 * t)


def _board_outline(scale_xy: float = 1.0, samples: int = 42) -> list[tuple[float, float]]:
    """Closed 2D loop for the pointed, tapered board top."""
    nose_x = BOARD_NOSE_X * scale_xy
    rear_start = REAR_CAP_START * scale_xy
    rear_x = BOARD_REAR_X * scale_xy
    max_half = BOARD_WIDTH * scale_xy * 0.5

    upper: list[tuple[float, float]] = [(nose_x, 0.0)]
    for i in range(1, samples + 1):
        x = nose_x + (rear_start - nose_x) * i / samples
        y = _half_width_at_x(x / scale_xy, scale=scale_xy)
        upper.append((x, y))

    cap: list[tuple[float, float]] = []
    rx = rear_x - rear_start
    for i in range(1, 18):
        ang = math.pi * 0.5 - math.pi * i / 18.0
        cap.append((rear_start + rx * math.cos(ang), max_half * math.sin(ang)))

    lower = [(x, -y) for (x, y) in reversed(upper[1:])]
    return upper + cap + lower


def _extruded_profile_mesh(
    profile: list[tuple[float, float]], z_bottom: float, z_top: float
) -> MeshGeometry:
    """Simple closed extrusion for the soft cover and metal board plate."""
    geom = MeshGeometry()
    n = len(profile)
    for x, y in profile:
        geom.add_vertex(x, y, z_bottom)
    for x, y in profile:
        geom.add_vertex(x, y, z_top)
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(i, j, n + j)
        geom.add_face(i, n + j, n + i)
    bottom_center = geom.add_vertex(0.02, 0.0, z_bottom)
    top_center = geom.add_vertex(0.02, 0.0, z_top)
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(bottom_center, j, i)
        geom.add_face(top_center, n + i, n + j)
    return geom


# ───────────────────────────────────────────────────────────────────────────
# Tube helpers
# ───────────────────────────────────────────────────────────────────────────

def _tube(points: list[tuple[float, float, float]], radius: float, name_hint: str) -> MeshGeometry:
    return tube_from_spline_points(
        points, radius=radius, samples_per_segment=6, radial_segments=18, cap_ends=True,
    )


def _tube_assembly(segments: list[list[tuple[float, float, float]]], radius: float) -> MeshGeometry:
    geom = MeshGeometry()
    for idx, segment in enumerate(segments):
        geom.merge(_tube(segment, radius, f"tube_{idx}"))
    return geom


# ───────────────────────────────────────────────────────────────────────────
# Short-leg skeleton (tabletop fold-flat frame)
# ───────────────────────────────────────────────────────────────────────────

def _leg_pair_mesh(direction: int) -> MeshGeometry:
    """Connected two-side tubular leg assembly in its hinge-local frame.

    direction=-1: rear hinge pair (legs angle very slightly toward nose).
    direction=+1: front hinge pair (legs angle very slightly toward rear).

    At q=0 the legs hang nearly straight down for a low tabletop stance.
    At q≈π/2 the legs swing flat against the board underside (stow topology).
    """
    foot_x = direction * 0.01          # minimal splay for fold-flat clearance
    foot_z = -LEG_HEIGHT               # short stance
    top_y = 0.085                      # half-span at hinge
    foot_y = 0.115                     # half-span at feet (wider base)

    segments = [
        # Two main sloped leg tubes
        [(0.0, -top_y, 0.0), (foot_x * 0.5, -top_y * 1.08, foot_z * 0.50), (foot_x, -foot_y, foot_z)],
        [(0.0, top_y, 0.0), (foot_x * 0.5, top_y * 1.08, foot_z * 0.50), (foot_x, foot_y, foot_z)],
        # Cross bar at hinge (top)
        [(0.0, -top_y, 0.0), (0.0, top_y, 0.0)],
        # T-foot cross bar (bottom)
        [(foot_x, -foot_y - 0.035, foot_z), (foot_x, foot_y + 0.035, foot_z)],
        # Mid-height brace
        [(foot_x * 0.55, -foot_y * 0.78, foot_z * 0.55), (foot_x * 0.55, foot_y * 0.78, foot_z * 0.55)],
    ]
    return _tube_assembly(segments, 0.012)


def _foot_caps_mesh(direction: int) -> MeshGeometry:
    """Short rubber sleeves at each end of the T-foot cross tube."""
    foot_x = direction * 0.01
    foot_z = -LEG_HEIGHT
    foot_y = 0.115
    return _tube_assembly(
        [
            [(foot_x, -foot_y - 0.030, foot_z), (foot_x, -foot_y + 0.005, foot_z)],
            [(foot_x, foot_y - 0.005, foot_z), (foot_x, foot_y + 0.030, foot_z)],
        ],
        0.016,
    )


def _underframe_mesh() -> MeshGeometry:
    """Fixed underside rails and diagonal braces carried by the board."""
    segments = [
        [(-0.26, -0.090, -0.024), (0.28, -0.090, -0.024)],
        [(-0.26, 0.090, -0.024), (0.28, 0.090, -0.024)],
        [(-0.20, -0.090, -0.025), (0.22, 0.090, -0.028)],
        [(-0.20, 0.090, -0.025), (0.22, -0.090, -0.028)],
        [(0.20, -0.108, -0.030), (0.20, 0.108, -0.030)],
        [(-0.16, -0.100, -0.030), (-0.16, 0.100, -0.030)],
    ]
    return _tube_assembly(segments, 0.006)


def _wire_rest_mesh() -> MeshGeometry:
    """Rear wire holder loop, embedded into the board tail."""
    segments = [
        [(0.32, -0.090, -0.010), (0.39, -0.100, 0.028), (0.43, 0.000, 0.036), (0.39, 0.100, 0.028), (0.32, 0.090, -0.010)],
        [(0.34, -0.065, -0.008), (0.40, -0.065, 0.020), (0.40, 0.065, 0.020), (0.34, 0.065, -0.008)],
    ]
    return _tube_assembly(segments, 0.004)


# ───────────────────────────────────────────────────────────────────────────
# Object model
# ───────────────────────────────────────────────────────────────────────────

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tabletop_ironing_board")

    blue_fabric = model.material("navy_fabric", rgba=(0.03, 0.09, 0.24, 1.0))
    white_print = model.material("white_dot_print", rgba=(0.92, 0.96, 0.96, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.62, 0.68, 0.68, 1.0))
    shadow = model.material("dark_hole_shadow", rgba=(0.02, 0.025, 0.030, 1.0))
    rubber = model.material("grey_plastic_feet", rgba=(0.33, 0.38, 0.40, 1.0))

    # ── Board ──────────────────────────────────────────────────────────
    board = model.part("board")

    fabric_mesh = mesh_from_geometry(
        _extruded_profile_mesh(_board_outline(scale_xy=1.018), -0.012, 0.014),
        "fitted_fabric_cover",
    )
    board.visual(fabric_mesh, material=blue_fabric, name="fitted_cover")

    plate_mesh = mesh_from_geometry(
        _extruded_profile_mesh(_board_outline(scale_xy=0.968), -0.032, -0.010),
        "tapered_metal_board",
    )
    board.visual(plate_mesh, material=galvanized, name="perforated_plate")

    # Navy fabric cover with pale perforation-like dots
    dot_index = 0
    for row, x in enumerate([BOARD_NOSE_X + 0.08 + i * 0.060 for i in range(11)]):
        hw = _half_width_at_x(x, scale=0.91)
        if hw < 0.025:
            continue
        cols = max(1, int((2.0 * hw - 0.040) / 0.042))
        for c in range(cols):
            if cols == 1:
                y = 0.0
            else:
                y = -hw + 0.025 + c * ((2.0 * hw - 0.050) / max(1, cols - 1))
            if (row + c) % 2:
                y += 0.007
            if abs(y) < hw - 0.018:
                board.visual(
                    Cylinder(radius=0.007, length=0.0014),
                    origin=Origin(xyz=(x, y, 0.0145)),
                    material=white_print,
                    name=f"cover_dot_{dot_index}",
                )
                dot_index += 1

    # Dark circles on the exposed underside indicate perforated metal deck
    for i, x in enumerate([-0.16, -0.08, 0.00, 0.08, 0.16, 0.24]):
        for j, y in enumerate((-0.055, 0.055)):
            board.visual(
                Cylinder(radius=0.008, length=0.0012),
                origin=Origin(xyz=(x, y, -0.0326)),
                material=shadow,
                name=f"underside_hole_{i}_{j}",
            )

    board.visual(
        mesh_from_geometry(_underframe_mesh(), "underside_braces"),
        material=galvanized, name="underside_braces",
    )
    board.visual(
        mesh_from_geometry(_wire_rest_mesh(), "rear_wire_rest"),
        material=galvanized, name="rear_wire_rest",
    )

    # Clevis hinge brackets: just outside the leg tubes at each hinge line.
    # The bracket inner face sits ~0.002 m inside the tube outer face to
    # represent the captured pivot pin (small intentional hinge embedding).
    # Brackets extend upward to the underframe rail level for structural
    # continuity with the board underside.
    for x, tag, span in ((0.28, "rear", 0.100), (-0.22, "front", 0.100)):
        for side in (-1, 1):
            board.visual(
                Box((0.038, 0.010, 0.040)),
                origin=Origin(xyz=(x, side * span, -0.044)),
                material=galvanized,
                name=f"{tag}_hinge_bracket_{0 if side < 0 else 1}",
            )
            if tag == "rear":
                board.visual(
                    Box((0.042, 0.032, 0.008)),
                    origin=Origin(xyz=(x, side * 0.100, -0.024)),
                    material=galvanized,
                    name=f"rear_hinge_mount_{0 if side < 0 else 1}",
                )

    # ── Rear leg pair ──────────────────────────────────────────────────
    rear_leg = model.part("rear_leg_pair")
    rear_leg.visual(
        mesh_from_geometry(_leg_pair_mesh(-1), "rear_leg_tubes"),
        material=galvanized, name="leg_tubes",
    )
    rear_leg.visual(
        mesh_from_geometry(_foot_caps_mesh(-1), "rear_foot_caps"),
        material=rubber, name="foot_caps",
    )
    for i, y in enumerate((-0.085, 0.085)):
        rear_leg.visual(
            Cylinder(radius=0.014, length=0.005),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=galvanized,
            name=f"pivot_washer_{i}",
        )

    # ── Front leg pair ─────────────────────────────────────────────────
    front_leg = model.part("front_leg_pair")
    front_leg.visual(
        mesh_from_geometry(_leg_pair_mesh(1), "front_leg_tubes"),
        material=galvanized, name="leg_tubes",
    )
    front_leg.visual(
        mesh_from_geometry(_foot_caps_mesh(1), "front_foot_caps"),
        material=rubber, name="foot_caps",
    )
    for i, y in enumerate((-0.085, 0.085)):
        front_leg.visual(
            Cylinder(radius=0.011, length=0.005),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=shadow,
            name=f"pivot_bolt_{i}",
        )

    # ── Articulations: fold-flat stow topology ─────────────────────────
    # Positive q swings legs from deployed (hanging down) to stowed (flat
    # against the board underside).  With the leg foot at local (-0.01, ±y,
    # -0.12) and axis=(0,1,0), q≈π/2 rotates the foot to roughly (-0.12, ±y,
    # ~0), i.e. horizontal and below the board — fold-flat.
    model.articulation(
        "board_to_rear_leg",
        ArticulationType.REVOLUTE,
        parent=board,
        child=rear_leg,
        origin=Origin(xyz=(0.28, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.5, lower=0.0, upper=1.62),
    )
    model.articulation(
        "board_to_front_leg",
        ArticulationType.REVOLUTE,
        parent=board,
        child=front_leg,
        origin=Origin(xyz=(-0.22, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.5, lower=0.0, upper=1.62),
    )

    return model


# ───────────────────────────────────────────────────────────────────────────
# Tests
# ───────────────────────────────────────────────────────────────────────────

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    board = object_model.get_part("board")
    rear_leg = object_model.get_part("rear_leg_pair")
    front_leg = object_model.get_part("front_leg_pair")
    rear_hinge = object_model.get_articulation("board_to_rear_leg")
    front_hinge = object_model.get_articulation("board_to_front_leg")

    # ── Hinge-pin embedding allowances ─────────────────────────────────
    # The clevis brackets slightly capture the leg tube at the pivot axis.
    for bracket_name in ("rear_hinge_bracket_0", "rear_hinge_bracket_1"):
        ctx.allow_overlap(
            board, rear_leg,
            elem_a=bracket_name, elem_b="leg_tubes",
            reason="Clevis hinge bracket captures the leg tube at the pivot axis for the fold-flat hinge connection.",
        )
    for bracket_name in ("front_hinge_bracket_0", "front_hinge_bracket_1"):
        ctx.allow_overlap(
            board, front_leg,
            elem_a=bracket_name, elem_b="leg_tubes",
            reason="Clevis hinge bracket captures the leg tube at the pivot axis for the fold-flat hinge connection.",
        )

    # Proof: brackets retain leg tubes at each hinge
    for bracket_name in ("rear_hinge_bracket_0", "rear_hinge_bracket_1"):
        ctx.expect_overlap(
            board, rear_leg, axes="xy",
            elem_a=bracket_name, elem_b="leg_tubes",
            min_overlap=0.005,
            name=f"{bracket_name} retains leg tube at pivot",
        )
    for bracket_name in ("front_hinge_bracket_0", "front_hinge_bracket_1"):
        ctx.expect_overlap(
            board, front_leg, axes="xy",
            elem_a=bracket_name, elem_b="leg_tubes",
            min_overlap=0.005,
            name=f"{bracket_name} retains leg tube at pivot",
        )

    # ── Compact tabletop board proportions ─────────────────────────────
    board_aabb = ctx.part_world_aabb(board)
    ctx.check(
        "board has compact tabletop proportions",
        board_aabb is not None
        and 0.70 < (board_aabb[1][0] - board_aabb[0][0]) < 0.95
        and 0.26 < (board_aabb[1][1] - board_aabb[0][1]) < 0.40,
        details=f"board_aabb={board_aabb}",
    )

    # ── Short-leg stance: the primary structural-axis claim ────────────
    rear_rest = ctx.part_world_aabb(rear_leg)
    front_rest = ctx.part_world_aabb(front_leg)
    board_bottom = board_aabb[0][2] if board_aabb else -0.05
    ctx.check(
        "rear_leg_pair and front_leg_pair give short tabletop stance (~0.12-0.18 m)",
        rear_rest is not None
        and front_rest is not None
        and rear_rest[0][2] > -0.22
        and front_rest[0][2] > -0.22
        and rear_rest[0][2] < -0.06
        and front_rest[0][2] < -0.06,
        details=f"rear={rear_rest}, front={front_rest}",
    )

    # Both leg pairs occupy the same Z band (stable tabletop base)
    ctx.expect_overlap(
        rear_leg, front_leg, axes="z", min_overlap=0.05,
        name="two short leg pairs share the same stance height",
    )

    # ── Fold-flat stow topology: the joint-graph claim ─────────────────
    with ctx.pose({rear_hinge: 1.55, front_hinge: 1.55}):
        rear_folded = ctx.part_world_aabb(rear_leg)
        front_folded = ctx.part_world_aabb(front_leg)

    ctx.check(
        "board_to_rear_leg and board_to_front_leg fold legs coplanar against board underside",
        rear_rest is not None
        and front_rest is not None
        and rear_folded is not None
        and front_folded is not None
        and rear_folded[0][2] > rear_rest[0][2] + 0.06
        and front_folded[0][2] > front_rest[0][2] + 0.06
        and rear_folded[0][2] > board_bottom - 0.025
        and front_folded[0][2] > board_bottom - 0.025
        and rear_folded[1][2] < 0.020
        and front_folded[1][2] < 0.020,
        details=(
            f"rear_rest={rear_rest}, rear_folded={rear_folded}, "
            f"front_rest={front_rest}, front_folded={front_folded}, "
            f"board_bottom={board_bottom}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
