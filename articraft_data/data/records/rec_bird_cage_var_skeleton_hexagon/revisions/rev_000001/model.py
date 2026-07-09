from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)

WIRE_R = 0.0032
CAGE_R = 0.215
BASE_R = 0.248
STRAIGHT_Z0 = 0.075
STRAIGHT_Z1 = 0.468
DOME_Z = 0.720
DOOR_W = 0.130
DOOR_H = 0.205
DOOR_Z = 0.285

# --- Hexagonal cross-section geometry ---
# Regular hexagon with corners at 0°, 60°, 120°, 180°, 240°, 300°
# gives a flat face at -Y between corners 4 (240°) and 5 (300°).
HEX_FACE_DIST = CAGE_R * math.cos(math.pi / 6.0)  # inscribed radius ≈ 0.1862
DOOR_Y = -HEX_FACE_DIST - 0.011

DOOR_BOTTOM_Z = DOOR_Z - DOOR_H / 2.0
DOOR_TOP_Z = DOOR_Z + DOOR_H / 2.0
DOOR_OPENING_HALF_W = DOOR_W / 2.0 + 0.014

# Corner positions: flat face forward
# Corner 0: (+R, 0)            right
# Corner 1: (+R/2, +√3/2·R)   upper-right
# Corner 2: (−R/2, +√3/2·R)   upper-left
# Corner 3: (−R, 0)            left
# Corner 4: (−R/2, −√3/2·R)   front-left
# Corner 5: (+R/2, −√3/2·R)   front-right
HEX_CORNERS = [
    (CAGE_R * math.cos(i * math.tau / 6.0), CAGE_R * math.sin(i * math.tau / 6.0))
    for i in range(6)
]
FRONT_FACE_IDX = 4  # face from corner 4 → corner 5 faces −Y
FRONT_FACE_Y = -HEX_FACE_DIST


def _merge(parts: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for p in parts:
        merged.merge(p)
    return merged


def _rod_z(
    radius: float, length: float, center: tuple[float, float, float], segments: int = 16
) -> MeshGeometry:
    return CylinderGeometry(radius=radius, height=length, radial_segments=segments).translate(
        *center
    )


def _rod_x(
    radius: float, length: float, center: tuple[float, float, float], segments: int = 16
) -> MeshGeometry:
    return (
        CylinderGeometry(radius=radius, height=length, radial_segments=segments)
        .rotate_y(math.pi / 2.0)
        .translate(*center)
    )


def _rod_y(
    radius: float, length: float, center: tuple[float, float, float], segments: int = 16
) -> MeshGeometry:
    return (
        CylinderGeometry(radius=radius, height=length, radial_segments=segments)
        .rotate_x(math.pi / 2.0)
        .translate(*center)
    )


def _tube_xy(
    x0: float, y0: float, x1: float, y1: float, z: float, tube_r: float
) -> MeshGeometry:
    """Straight horizontal tube between two XY points at height z."""
    return tube_from_spline_points(
        [(x0, y0, z), (x1, y1, z)],
        radius=tube_r,
        samples_per_segment=2,
        radial_segments=12,
        cap_ends=True,
    )


def _hex_ring_rails(z: float, tube_r: float) -> list[MeshGeometry]:
    """Build the 6 edge rails of a hexagonal ring frame at height z."""
    rails: list[MeshGeometry] = []
    door_in_z = DOOR_BOTTOM_Z < z < DOOR_TOP_Z

    for face_i in range(6):
        x0, y0 = HEX_CORNERS[face_i]
        x1, y1 = HEX_CORNERS[(face_i + 1) % 6]

        if face_i == FRONT_FACE_IDX and door_in_z:
            # Split the front-face rail around the door aperture
            left_clip = -DOOR_OPENING_HALF_W
            right_clip = DOOR_OPENING_HALF_W
            if x0 < left_clip:
                rails.append(_tube_xy(x0, y0, left_clip, y0, z, tube_r))
            if x1 > right_clip:
                rails.append(_tube_xy(right_clip, y1, x1, y1, z, tube_r))
        else:
            rails.append(_tube_xy(x0, y0, x1, y1, z, tube_r))

    return rails


def _birdcage_wire_mesh() -> MeshGeometry:
    bars: list[MeshGeometry] = []

    # ── Structural hexagonal ring frames at each level ────────────────
    for z, tube_r in (
        (STRAIGHT_Z0, 0.0048),
        (0.155, 0.0034),
        (0.325, 0.0032),
        (STRAIGHT_Z1, 0.0044),
    ):
        bars.extend(_hex_ring_rails(z, tube_r))

    # ── Corner posts (heavier verticals at each hex corner) ───────────
    for i in range(6):
        cx, cy = HEX_CORNERS[i]
        bars.append(
            _rod_z(
                0.005,
                STRAIGHT_Z1 - STRAIGHT_Z0,
                (cx, cy, (STRAIGHT_Z0 + STRAIGHT_Z1) / 2.0),
                14,
            )
        )
        # Roof meridian from corner – prominent structural rib
        bars.append(
            tube_from_spline_points(
                [
                    (cx, cy, STRAIGHT_Z1),
                    (cx * 0.75, cy * 0.75, 0.545),
                    (cx * 0.42, cy * 0.42, 0.645),
                    (cx * 0.12, cy * 0.12, DOME_Z),
                ],
                radius=WIRE_R,
                samples_per_segment=10,
                radial_segments=12,
                cap_ends=True,
            )
        )

    # ── Face bars (evenly distributed on each flat face) ──────────────
    bars_per_face = 5
    for face_i in range(6):
        x0, y0 = HEX_CORNERS[face_i]
        x1, y1 = HEX_CORNERS[(face_i + 1) % 6]
        is_front = face_i == FRONT_FACE_IDX

        for j in range(1, bars_per_face + 1):
            t = j / (bars_per_face + 1)
            bx = x0 + t * (x1 - x0)
            by = y0 + t * (y1 - y0)

            in_door_opening = is_front and abs(bx) <= DOOR_OPENING_HALF_W
            if in_door_opening:
                lower_len = DOOR_BOTTOM_Z - STRAIGHT_Z0
                upper_len = STRAIGHT_Z1 - DOOR_TOP_Z
                if lower_len > 0.005:
                    bars.append(
                        _rod_z(WIRE_R, lower_len, (bx, by, STRAIGHT_Z0 + lower_len / 2.0), 12)
                    )
                if upper_len > 0.005:
                    bars.append(
                        _rod_z(WIRE_R, upper_len, (bx, by, DOOR_TOP_Z + upper_len / 2.0), 12)
                    )
            else:
                bars.append(
                    _rod_z(
                        WIRE_R,
                        STRAIGHT_Z1 - STRAIGHT_Z0,
                        (bx, by, (STRAIGHT_Z0 + STRAIGHT_Z1) / 2.0),
                        12,
                    )
                )

            # Roof meridian from each face bar
            bars.append(
                tube_from_spline_points(
                    [
                        (bx, by, STRAIGHT_Z1),
                        (bx * 0.72, by * 0.72, 0.548),
                        (bx * 0.38, by * 0.38, 0.650),
                        (bx * 0.10, by * 0.10, DOME_Z),
                    ],
                    radius=WIRE_R * 0.85,
                    samples_per_segment=10,
                    radial_segments=12,
                    cap_ends=True,
                )
            )

    # ── Crown ring and crossing spokes ────────────────────────────────
    bars.append(
        TorusGeometry(
            radius=0.036, tube=0.0048, radial_segments=12, tubular_segments=48
        ).translate(0.0, 0.0, DOME_Z)
    )
    bars.append(_rod_x(0.0032, 0.078, (0.0, 0.0, DOME_Z), 12))
    bars.append(_rod_y(0.0032, 0.078, (0.0, 0.0, DOME_Z), 12))
    bars.append(_rod_z(0.0050, 0.035, (0.0, 0.0, DOME_Z + 0.012), 14))

    # ── Door opening frame (heavier wire around aperture) ─────────────
    opening_frame_h = DOOR_H + 0.022
    bars.extend(
        [
            _rod_z(0.0052, opening_frame_h, (-DOOR_OPENING_HALF_W, FRONT_FACE_Y, DOOR_Z), 16),
            _rod_z(0.0052, opening_frame_h, (DOOR_OPENING_HALF_W, FRONT_FACE_Y, DOOR_Z), 16),
            _rod_x(0.0052, DOOR_OPENING_HALF_W * 2.0, (0.0, FRONT_FACE_Y, DOOR_BOTTOM_Z), 16),
            _rod_x(0.0052, DOOR_OPENING_HALF_W * 2.0, (0.0, FRONT_FACE_Y, DOOR_TOP_Z), 16),
        ]
    )

    # ── Door surround: hinge-side / keeper bars, hinge pin, keeper ────
    left_x = -DOOR_W / 2.0
    right_x = DOOR_W / 2.0
    hinge_bar_y = FRONT_FACE_Y
    keeper_x = right_x + 0.032
    keeper_bar_y = FRONT_FACE_Y

    bars.extend(
        [
            # Hinge-side and keeper-side cage bars (full straight-section height)
            _rod_z(
                WIRE_R,
                STRAIGHT_Z1 - STRAIGHT_Z0,
                (left_x, hinge_bar_y, (STRAIGHT_Z0 + STRAIGHT_Z1) / 2.0),
                12,
            ),
            _rod_z(
                WIRE_R,
                STRAIGHT_Z1 - STRAIGHT_Z0,
                (keeper_x, keeper_bar_y, (STRAIGHT_Z0 + STRAIGHT_Z1) / 2.0),
                12,
            ),
            # Door frame (slightly proud of the cage face)
            _rod_z(0.0042, DOOR_H + 0.028, (left_x, DOOR_Y + 0.002, DOOR_Z), 14),
            _rod_z(0.0042, DOOR_H + 0.028, (right_x, DOOR_Y + 0.002, DOOR_Z), 14),
            _rod_x(0.0042, DOOR_W + 0.028, (0.0, DOOR_Y + 0.002, DOOR_BOTTOM_Z), 14),
            _rod_x(0.0042, DOOR_W + 0.028, (0.0, DOOR_Y + 0.002, DOOR_TOP_Z), 14),
            # Hinge pin
            _rod_z(0.0022, DOOR_H + 0.055, (left_x, DOOR_Y, DOOR_Z), 12),
            # Latch keeper arms
            _rod_x(0.0025, 0.030, (right_x + 0.018, DOOR_Y - 0.001, DOOR_Z + 0.006), 12),
            _rod_z(0.0025, 0.035, (right_x + 0.032, DOOR_Y - 0.001, DOOR_Z + 0.006), 12),
        ]
    )

    # Hinge straps connecting the pin back to the cage frame
    strap_len = abs(DOOR_Y - hinge_bar_y)
    if strap_len > 0.002:
        for z in (DOOR_Z - 0.075, DOOR_Z + 0.075):
            bars.append(
                _rod_y(
                    0.0024,
                    strap_len,
                    (left_x, (DOOR_Y + hinge_bar_y) / 2.0, z),
                    10,
                )
            )
    keeper_strap_len = abs((DOOR_Y - 0.001) - keeper_bar_y)
    if keeper_strap_len > 0.002:
        bars.append(
            _rod_y(
                0.0022,
                keeper_strap_len,
                (keeper_x, ((DOOR_Y - 0.001) + keeper_bar_y) / 2.0, DOOR_Z + 0.006),
                10,
            )
        )

    return _merge(bars)


def _top_hook_mesh() -> MeshGeometry:
    # A single continuous wire rises from the dome apex into a small hanging hook.
    return tube_from_spline_points(
        [
            (0.0, 0.0, DOME_Z - 0.006),
            (0.0, 0.0, DOME_Z + 0.045),
            (0.0, 0.0, DOME_Z + 0.095),
            (0.012, 0.0, DOME_Z + 0.126),
            (0.036, 0.0, DOME_Z + 0.121),
            (0.040, 0.0, DOME_Z + 0.095),
            (0.020, 0.0, DOME_Z + 0.087),
        ],
        radius=0.0042,
        samples_per_segment=12,
        radial_segments=16,
        cap_ends=True,
    )


def _base_tray_mesh() -> MeshGeometry:
    tray = LatheGeometry.from_shell_profiles(
        [
            (0.030, -0.020),
            (0.175, -0.018),
            (0.232, -0.004),
            (BASE_R, 0.020),
            (BASE_R, 0.055),
            (0.236, 0.074),
        ],
        [
            (0.015, -0.006),
            (0.160, -0.004),
            (0.218, 0.012),
            (0.224, 0.048),
            (0.208, 0.060),
        ],
        segments=72,
        start_cap="flat",
        end_cap="round",
        lip_samples=8,
    )
    return tray


def _perch_mesh() -> MeshGeometry:
    return _rod_x(0.0085, 0.438, (0.0, 0.0, 0.248), 18)


def _door_mesh() -> MeshGeometry:
    g = MeshGeometry()
    # Local door frame: hinge line is local x=0, door extends along +X.
    g.merge(_rod_z(0.004, DOOR_H, (0.0, 0.0, 0.0), 14))
    g.merge(_rod_z(0.004, DOOR_H, (DOOR_W, 0.0, 0.0), 14))
    g.merge(_rod_x(0.004, DOOR_W, (DOOR_W / 2.0, 0.0, -DOOR_H / 2.0), 14))
    g.merge(_rod_x(0.004, DOOR_W, (DOOR_W / 2.0, 0.0, DOOR_H / 2.0), 14))
    g.merge(_rod_x(0.0027, DOOR_W, (DOOR_W / 2.0, 0.0, 0.000), 12))
    for frac in (0.20, 0.40, 0.60, 0.80):
        g.merge(_rod_z(0.0024, DOOR_H - 0.010, (DOOR_W * frac, 0.0, 0.0), 10))
    # Hinge knuckles wrap around the fixed pin.
    for zc in (-0.066, 0.066):
        g.merge(_rod_z(0.0068, 0.064, (0.0, 0.0, zc), 16))
    return g


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hexagonal_bird_cage")

    aged_brass = model.material("aged_brass", rgba=(0.73, 0.48, 0.22, 1.0))
    dark_bronze = model.material("dark_bronze", rgba=(0.38, 0.24, 0.12, 1.0))
    warm_wood = model.material("warm_wood", rgba=(0.48, 0.29, 0.13, 1.0))

    base = model.part("base_tray")
    base.visual(
        mesh_from_geometry(_base_tray_mesh(), "base_tray"), material=dark_bronze, name="tray_shell"
    )

    frame = model.part("wire_frame")
    frame.visual(
        mesh_from_geometry(_birdcage_wire_mesh(), "wire_frame"),
        material=aged_brass,
        name="cage_bars",
    )
    frame.visual(
        mesh_from_geometry(_top_hook_mesh(), "top_hook"), material=aged_brass, name="top_hook"
    )

    perch = model.part("perch")
    perch.visual(mesh_from_geometry(_perch_mesh(), "perch"), material=warm_wood, name="perch_rod")

    door = model.part("access_door")
    door.visual(
        mesh_from_geometry(_door_mesh(), "door_wire_panel"),
        material=aged_brass,
        name="door_wire_panel",
    )

    latch = model.part("latch")
    latch.visual(
        Cylinder(radius=0.0085, length=0.006),
        origin=Origin(xyz=(0.0, -0.003, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aged_brass,
        name="pivot_washer",
    )
    latch.visual(
        Box((0.055, 0.006, 0.010)),
        origin=Origin(xyz=(0.0275, -0.006, 0.0)),
        material=aged_brass,
        name="swing_bar",
    )

    model.articulation(
        "frame_to_door",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=door,
        origin=Origin(xyz=(-DOOR_W / 2.0, DOOR_Y, DOOR_Z)),
        # Negative Z makes the free edge swing outward toward the viewer/front (-Y).
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.5, lower=0.0, upper=1.35),
    )
    model.articulation(
        "tray_to_frame",
        ArticulationType.FIXED,
        parent=base,
        child=frame,
        origin=Origin(),
    )
    model.articulation(
        "frame_to_perch",
        ArticulationType.FIXED,
        parent=frame,
        child=perch,
        origin=Origin(),
    )
    model.articulation(
        "door_to_latch",
        ArticulationType.REVOLUTE,
        parent=door,
        child=latch,
        origin=Origin(xyz=(DOOR_W - 0.018, 0.0, 0.010)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.15, velocity=3.0, lower=0.0, upper=1.5708),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_tray")
    frame = object_model.get_part("wire_frame")
    perch = object_model.get_part("perch")
    door = object_model.get_part("access_door")
    latch = object_model.get_part("latch")
    door_joint = object_model.get_articulation("frame_to_door")
    latch_joint = object_model.get_articulation("door_to_latch")

    ctx.allow_overlap(
        frame,
        door,
        elem_a="cage_bars",
        elem_b="door_wire_panel",
        reason="The access-door hinge knuckles are intentionally captured around the fixed hinge pin.",
    )
    ctx.allow_overlap(
        frame,
        perch,
        elem_a="cage_bars",
        elem_b="perch_rod",
        reason="The wooden perch dowel ends are intentionally seated through the opposing wire bars.",
    )
    ctx.allow_overlap(
        base,
        frame,
        elem_a="tray_shell",
        elem_b="cage_bars",
        reason="The lower wire hoop is intentionally seated into the rolled rim of the base tray.",
    )
    ctx.expect_overlap(
        door,
        frame,
        axes="z",
        elem_a="door_wire_panel",
        elem_b="cage_bars",
        min_overlap=0.16,
        name="hinge pin and door knuckles share vertical engagement",
    )

    ctx.check(
        "has hexagonal cage mechanisms",
        door_joint is not None and latch_joint is not None,
        details=f"parts={len(object_model.parts)}, articulations={len(object_model.articulations)}",
    )
    ctx.expect_overlap(
        frame, door, axes="xz", min_overlap=0.09, name="door is placed on front cage opening"
    )
    ctx.expect_overlap(
        frame, latch, axes="z", min_overlap=0.01, name="latch sits at door keeper height"
    )
    ctx.expect_overlap(
        base, frame, axes="xy", min_overlap=0.38, name="wire cage is seated over base tray"
    )
    ctx.expect_gap(
        frame,
        base,
        axis="z",
        max_penetration=0.007,
        positive_elem="cage_bars",
        negative_elem="tray_shell",
        name="lower wire hoop is shallowly seated in tray rim",
    )
    ctx.expect_within(
        perch, frame, axes="xy", margin=0.015, name="perch spans inside the cage bars"
    )
    ctx.expect_overlap(
        perch,
        frame,
        axes="x",
        elem_a="perch_rod",
        elem_b="cage_bars",
        min_overlap=0.40,
        name="perch remains captured across cage width",
    )

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: 1.0}):
        open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "access door swings outward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < closed_aabb[0][1] - 0.045,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    closed_latch = ctx.part_world_aabb(latch)
    with ctx.pose({latch_joint: 1.2}):
        turned_latch = ctx.part_world_aabb(latch)
    ctx.check(
        "latch rotates clear of keeper",
        closed_latch is not None
        and turned_latch is not None
        and turned_latch[1][2] > closed_latch[1][2] + 0.025,
        details=f"closed={closed_latch}, turned={turned_latch}",
    )

    # ── Hexagonal cross-section verification ──────────────────────────
    frame_aabb = ctx.part_element_world_aabb(frame, elem="cage_bars")
    ctx.check(
        "hexagonal cross-section: front face inset from circular radius",
        frame_aabb is not None and frame_aabb[0][1] > -CAGE_R + 0.012,
        details=(
            f"cage_bars y_min={frame_aabb[0][1] if frame_aabb else None}, "
            f"expected > {-CAGE_R + 0.012:.4f} (hex face closer than circle)"
        ),
    )
    ctx.check(
        "hexagonal corners reach full cage radius at sides",
        frame_aabb is not None
        and frame_aabb[1][0] > CAGE_R - 0.010
        and frame_aabb[0][0] < -CAGE_R + 0.010,
        details=(
            f"cage_bars x=[{frame_aabb[0][0]:.4f}, {frame_aabb[1][0]:.4f}]"
            if frame_aabb
            else "no aabb"
        ),
    )

    return ctx.report()


object_model = build_object_model()
