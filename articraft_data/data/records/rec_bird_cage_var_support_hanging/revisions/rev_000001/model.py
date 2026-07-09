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
DOOR_Y = -CAGE_R - 0.011
DOOR_BOTTOM_Z = DOOR_Z - DOOR_H / 2.0
DOOR_TOP_Z = DOOR_Z + DOOR_H / 2.0
DOOR_OPENING_HALF_W = DOOR_W / 2.0 + 0.014


def _merge(parts: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for part in parts:
        merged.merge(part)
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


def _front_opening_ring(radius: float, tube: float, z: float) -> MeshGeometry:
    half_angle = math.asin(min(DOOR_OPENING_HALF_W / radius, 0.95))
    front_angle = -math.pi / 2.0
    start = front_angle + half_angle
    end = front_angle + math.tau - half_angle
    points = [
        (radius * math.cos(a), radius * math.sin(a), z)
        for a in [start + (end - start) * i / 72 for i in range(73)]
    ]
    return tube_from_spline_points(
        points,
        radius=tube,
        samples_per_segment=2,
        radial_segments=12,
        cap_ends=True,
    )


def _front_opening_chord(tube: float, z: float) -> MeshGeometry:
    points = []
    for i in range(17):
        x = -DOOR_OPENING_HALF_W + 2.0 * DOOR_OPENING_HALF_W * i / 16
        y = -math.sqrt(max(CAGE_R * CAGE_R - x * x, 0.0))
        points.append((x, y, z))
    return tube_from_spline_points(
        points,
        radius=tube,
        samples_per_segment=2,
        radial_segments=12,
        cap_ends=True,
    )


def _birdcage_wire_mesh() -> MeshGeometry:
    bars: list[MeshGeometry] = []

    # Structural hoops that preserve the circular cage silhouette and bar spacing.
    for z, tube in (
        (STRAIGHT_Z0, 0.0048),
        (0.155, 0.0034),
        (0.325, 0.0032),
        (STRAIGHT_Z1, 0.0044),
    ):
        if DOOR_BOTTOM_Z < z < DOOR_TOP_Z:
            bars.append(_front_opening_ring(CAGE_R, tube, z))
        else:
            bars.append(
                TorusGeometry(
                    radius=CAGE_R, tube=tube, radial_segments=14, tubular_segments=96
                ).translate(0.0, 0.0, z)
            )

    bar_count = 36
    for i in range(bar_count):
        a = math.tau * i / bar_count
        x, y = CAGE_R * math.cos(a), CAGE_R * math.sin(a)
        in_door_opening = y < 0.0 and abs(x) <= DOOR_OPENING_HALF_W
        if in_door_opening:
            lower_len = DOOR_BOTTOM_Z - STRAIGHT_Z0
            upper_len = STRAIGHT_Z1 - DOOR_TOP_Z
            bars.append(_rod_z(WIRE_R, lower_len, (x, y, STRAIGHT_Z0 + lower_len / 2.0), 12))
            bars.append(_rod_z(WIRE_R, upper_len, (x, y, DOOR_TOP_Z + upper_len / 2.0), 12))
        else:
            bars.append(
                _rod_z(
                    WIRE_R, STRAIGHT_Z1 - STRAIGHT_Z0, (x, y, (STRAIGHT_Z0 + STRAIGHT_Z1) / 2.0), 12
                )
            )

        # Arched roof meridians meet the top cap, giving the domed reference silhouette.
        roof_points = [
            (CAGE_R * math.cos(a), CAGE_R * math.sin(a), STRAIGHT_Z1),
            (0.198 * math.cos(a), 0.198 * math.sin(a), 0.545),
            (0.132 * math.cos(a), 0.132 * math.sin(a), 0.655),
            (0.036 * math.cos(a), 0.036 * math.sin(a), DOME_Z),
        ]
        bars.append(
            tube_from_spline_points(
                roof_points,
                radius=WIRE_R,
                samples_per_segment=10,
                radial_segments=12,
                cap_ends=True,
            )
        )

    # Crown ring and crossing spokes tie the roof meridians to the hanging hook.
    bars.append(
        TorusGeometry(radius=0.036, tube=0.0048, radial_segments=12, tubular_segments=48).translate(
            0.0, 0.0, DOME_Z
        )
    )
    bars.append(_rod_x(0.0032, 0.078, (0.0, 0.0, DOME_Z), 12))
    bars.append(_rod_y(0.0032, 0.078, (0.0, 0.0, DOME_Z), 12))
    bars.append(_rod_z(0.0050, 0.035, (0.0, 0.0, DOME_Z + 0.012), 14))

    # Fixed opening frame receives the cut cage wires so the door aperture reads as built-in.
    opening_left_x = -DOOR_OPENING_HALF_W
    opening_right_x = DOOR_OPENING_HALF_W
    opening_left_y = -math.sqrt(max(CAGE_R * CAGE_R - opening_left_x * opening_left_x, 0.0))
    opening_right_y = -math.sqrt(max(CAGE_R * CAGE_R - opening_right_x * opening_right_x, 0.0))
    opening_frame_h = DOOR_H + 0.022
    bars.extend(
        [
            _rod_z(0.0052, opening_frame_h, (opening_left_x, opening_left_y, DOOR_Z), 16),
            _rod_z(0.0052, opening_frame_h, (opening_right_x, opening_right_y, DOOR_Z), 16),
            _front_opening_chord(0.0052, DOOR_BOTTOM_Z),
            _front_opening_chord(0.0052, DOOR_TOP_Z),
        ]
    )

    # Door surround, hinge pin, and latch keeper on the outside of the front face.
    left_x = -DOOR_W / 2.0
    right_x = DOOR_W / 2.0
    bottom_z = DOOR_BOTTOM_Z
    top_z = DOOR_TOP_Z
    hinge_bar_y = -math.sqrt(max(CAGE_R * CAGE_R - left_x * left_x, 0.0))
    keeper_x = right_x + 0.032
    keeper_bar_y = -math.sqrt(max(CAGE_R * CAGE_R - keeper_x * keeper_x, 0.0))
    bars.extend(
        [
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
            _rod_z(0.0042, DOOR_H + 0.028, (left_x, DOOR_Y + 0.002, DOOR_Z), 14),
            _rod_z(0.0042, DOOR_H + 0.028, (right_x, DOOR_Y + 0.002, DOOR_Z), 14),
            _rod_x(0.0042, DOOR_W + 0.028, (0.0, DOOR_Y + 0.002, bottom_z), 14),
            _rod_x(0.0042, DOOR_W + 0.028, (0.0, DOOR_Y + 0.002, top_z), 14),
            _rod_z(0.0022, DOOR_H + 0.055, (left_x, DOOR_Y, DOOR_Z), 12),
            _rod_x(0.0025, 0.030, (right_x + 0.018, DOOR_Y - 0.001, DOOR_Z + 0.006), 12),
            _rod_z(0.0025, 0.035, (right_x + 0.032, DOOR_Y - 0.001, DOOR_Z + 0.006), 12),
        ]
    )

    # Two little hinge straps visibly attach the pin back to the cage frame.
    for z in (DOOR_Z - 0.075, DOOR_Z + 0.075):
        bars.append(
            _rod_y(0.0024, abs(DOOR_Y - hinge_bar_y), (left_x, (DOOR_Y + hinge_bar_y) / 2.0, z), 10)
        )
    bars.append(
        _rod_y(
            0.0022,
            abs((DOOR_Y - 0.001) - keeper_bar_y),
            (keeper_x, ((DOOR_Y - 0.001) + keeper_bar_y) / 2.0, DOOR_Z + 0.006),
            10,
        )
    )

    return _merge(bars)


def _top_hook_mesh() -> MeshGeometry:
    # A strong arched carrying bail rises from two crown ring attachment
    # points and continues into a hanging loop at the apex — the sole support
    # interface for the hanging cage (replacing floor feet with top suspension).
    # The entire bail is one continuous tube: left foot → arch apex → loop →
    # arch apex → right foot.  This guarantees a single connected mesh.
    # Feet anchor at the crown ring (radius=0.036, z=DOME_Z) so the bail
    # reads as physically welded to the cage frame.
    foot_r = 0.036
    bail_top_z = DOME_Z + 0.130
    loop_r = 0.018
    loop_cz = bail_top_z + loop_r

    # Build the full path: arch up from left foot, loop at top, arch down to
    # right foot.
    n_loop = 24
    path: list[tuple[float, float, float]] = [
        (-foot_r, 0.0, DOME_Z),
        (-0.034, 0.0, DOME_Z + 0.040),
        (-0.025, 0.0, DOME_Z + 0.085),
        (-0.012, 0.0, DOME_Z + 0.115),
        (0.0, 0.0, bail_top_z),
    ]
    # Loop around from apex back to apex (in the XZ plane).
    for i in range(1, n_loop):
        a = math.tau * i / n_loop
        path.append(
            (loop_r * math.sin(a), 0.0, loop_cz - loop_r * math.cos(a))
        )
    path.append((0.0, 0.0, bail_top_z))
    # Continue down the right side of the arch.
    path.extend([
        (0.012, 0.0, DOME_Z + 0.115),
        (0.025, 0.0, DOME_Z + 0.085),
        (0.034, 0.0, DOME_Z + 0.040),
        (foot_r, 0.0, DOME_Z),
    ])

    return tube_from_spline_points(
        path,
        radius=0.0048,
        samples_per_segment=6,
        radial_segments=14,
        cap_ends=True,
    )


def _base_tray_mesh() -> MeshGeometry:
    # Shallow suspended catch-pan fixed under the frame.  No decorative feet —
    # the cage hangs from the top bail so the tray is a thin dish only.
    # The rim meets the lowest wire hoop at STRAIGHT_Z0.
    rim_z = STRAIGHT_Z0
    tray_depth = 0.020
    tray = LatheGeometry.from_shell_profiles(
        [
            (0.020, rim_z - tray_depth),
            (0.160, rim_z - tray_depth + 0.001),
            (0.220, rim_z - 0.004),
            (BASE_R, rim_z - 0.001),
            (BASE_R, rim_z + 0.001),
            (0.236, rim_z + 0.002),
        ],
        [
            (0.012, rim_z - tray_depth + 0.004),
            (0.150, rim_z - tray_depth + 0.005),
            (0.210, rim_z - 0.002),
            (0.222, rim_z),
            (0.210, rim_z + 0.001),
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
    model = ArticulatedObject(name="domed_bird_cage")

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
        "frame_to_tray",
        ArticulationType.FIXED,
        parent=frame,
        child=base,
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
        reason="The lower wire hoop is intentionally seated into the rolled rim of the catch pan.",
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
        "has domed cage mechanisms",
        door_joint is not None and latch_joint is not None,
        details=f"parts={len(object_model.parts)}, articulations={len(object_model.articulations)}",
    )
    ctx.expect_overlap(
        frame, door, axes="xz", min_overlap=0.09, name="door is placed on front cage opening"
    )
    ctx.expect_overlap(
        frame, latch, axes="z", min_overlap=0.01, name="latch sits at door keeper height"
    )

    # Hanging cage structural checks: bail is the sole top support,
    # tray hangs below the frame as a shallow catch pan.
    ctx.expect_overlap(
        base, frame, axes="xy", min_overlap=0.40, name="catch pan is centered under cage frame"
    )
    ctx.expect_gap(
        frame,
        base,
        axis="z",
        max_penetration=0.007,
        positive_elem="cage_bars",
        negative_elem="tray_shell",
        name="lower wire hoop is shallowly seated in catch-pan rim",
    )

    # The bail (top_hook visual) must be the highest geometry in the model,
    # confirming the cage hangs from the top rather than sitting on feet.
    frame_aabb = ctx.part_world_aabb(frame)
    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "bail is sole top support above cage body",
        frame_aabb is not None
        and base_aabb is not None
        and frame_aabb[1][2] > 0.80,
        details=f"frame_top_z={frame_aabb[1][2] if frame_aabb else None}, "
        f"base_bottom_z={base_aabb[0][2] if base_aabb else None}",
    )
    ctx.check(
        "catch pan sits below cage body with no floor feet",
        frame_aabb is not None
        and base_aabb is not None
        and base_aabb[0][2] < frame_aabb[0][2] + 0.010,
        details=f"tray_bottom={base_aabb[0][2] if base_aabb else None}, "
        f"frame_bottom={frame_aabb[0][2] if frame_aabb else None}",
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

    return ctx.report()


object_model = build_object_model()
