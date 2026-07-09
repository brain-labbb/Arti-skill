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


BOARD_LENGTH = 1.45
BOARD_WIDTH = 0.38
BOARD_NOSE_X = -0.72
BOARD_REAR_X = 0.72


def _half_width_at_x(x: float, *, scale: float = 1.0) -> float:
    """Plan-view ironing-board half width: sharp nose, broad rounded rear."""
    rear_cap_start = 0.64
    max_half = (BOARD_WIDTH * scale) * 0.5
    if x <= BOARD_NOSE_X:
        return 0.0
    if x >= rear_cap_start:
        # Elliptical rounded tail cap, widest at the cap shoulder.
        rx = max(0.001, BOARD_REAR_X - rear_cap_start)
        u = min(1.0, max(0.0, (x - rear_cap_start) / rx))
        return max_half * math.sqrt(max(0.0, 1.0 - u * u))
    t = (x - BOARD_NOSE_X) / (rear_cap_start - BOARD_NOSE_X)
    # Faster widening near the nose, then almost parallel sides through the body.
    return max_half * (math.sin(t * math.pi * 0.5) ** 0.56) * (0.985 + 0.015 * t)


def _board_outline(scale_xy: float = 1.0, samples: int = 42) -> list[tuple[float, float]]:
    """Closed 2D loop for the pointed, tapered board top."""
    nose_x = BOARD_NOSE_X * scale_xy
    rear_start = 0.64 * scale_xy
    rear_x = BOARD_REAR_X * scale_xy
    max_half = BOARD_WIDTH * scale_xy * 0.5

    upper: list[tuple[float, float]] = [(nose_x, 0.0)]
    for i in range(1, samples + 1):
        x = nose_x + (rear_start - nose_x) * i / samples
        # Evaluate with unscaled x so the same shape law is preserved.
        y = _half_width_at_x(x / scale_xy, scale=scale_xy)
        upper.append((x, y))

    # Rounded rear cap from upper shoulder to lower shoulder.
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
    """Simple closed extrusion used for the soft cover and metal board plate."""
    geom = MeshGeometry()
    n = len(profile)
    for x, y in profile:
        geom.add_vertex(x, y, z_bottom)
    for x, y in profile:
        geom.add_vertex(x, y, z_top)

    # Side walls.
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


def _tube(points: list[tuple[float, float, float]], radius: float, name_hint: str) -> MeshGeometry:
    return tube_from_spline_points(
        points,
        radius=radius,
        samples_per_segment=6,
        radial_segments=18,
        cap_ends=True,
    )


def _tube_assembly(segments: list[list[tuple[float, float, float]]], radius: float) -> MeshGeometry:
    geom = MeshGeometry()
    for idx, segment in enumerate(segments):
        geom.merge(_tube(segment, radius, f"tube_{idx}"))
    return geom


def _leg_pair_mesh(direction: int) -> MeshGeometry:
    """Connected two-side tubular leg assembly in its hinge-local frame."""
    # direction=-1: rear hinge leg pair slopes to the front/nose.
    # direction=+1: front hinge leg pair slopes to the rear/blunt end.
    foot_x = direction * 0.96
    foot_z = -0.77
    top_y = 0.165 if direction < 0 else 0.085
    foot_y = 0.255 if direction < 0 else 0.195
    mid_y = top_y + (foot_y - top_y) * 0.60

    segments = [
        [(0.0, -top_y, 0.0), (foot_x * 0.45, -top_y * 1.08, foot_z * 0.46), (foot_x, -foot_y, foot_z)],
        [(0.0, top_y, 0.0), (foot_x * 0.45, top_y * 1.08, foot_z * 0.46), (foot_x, foot_y, foot_z)],
        [(0.0, -top_y, 0.0), (0.0, top_y, 0.0)],
        [(foot_x, -foot_y - 0.055, foot_z), (foot_x, foot_y + 0.055, foot_z)],
        [(foot_x * 0.60, -mid_y, foot_z * 0.60), (foot_x * 0.60, mid_y, foot_z * 0.60)],
    ]
    return _tube_assembly(segments, 0.018)


def _foot_caps_mesh(direction: int) -> MeshGeometry:
    foot_x = direction * 0.96
    foot_z = -0.77
    foot_y = 0.300 if direction < 0 else 0.240
    # Short rubber sleeves at each end of the floor cross tube.
    return _tube_assembly(
        [
            [(foot_x, -foot_y - 0.045, foot_z), (foot_x, -foot_y + 0.006, foot_z)],
            [(foot_x, foot_y - 0.006, foot_z), (foot_x, foot_y + 0.045, foot_z)],
        ],
        0.022,
    )


def _underframe_mesh() -> MeshGeometry:
    """Fixed underside rails and diagonal braces carried by the board."""
    segments = [
        [(-0.50, -0.118, -0.026), (0.52, -0.118, -0.026)],
        [(-0.50, 0.118, -0.026), (0.52, 0.118, -0.026)],
        [(-0.43, -0.118, -0.027), (0.42, 0.118, -0.030)],
        [(-0.43, 0.118, -0.027), (0.42, -0.118, -0.030)],
        [(0.40, -0.150, -0.033), (0.40, 0.150, -0.033)],
        [(-0.32, -0.135, -0.033), (-0.32, 0.135, -0.033)],
    ]
    return _tube_assembly(segments, 0.0075)


def _wire_rest_mesh() -> MeshGeometry:
    """Rear wire holder loop, embedded into the board tail."""
    segments = [
        [(0.62, -0.145, -0.010), (0.78, -0.160, 0.040), (0.86, 0.000, 0.054), (0.78, 0.160, 0.040), (0.62, 0.145, -0.010)],
        [(0.66, -0.105, -0.008), (0.79, -0.105, 0.030), (0.79, 0.105, 0.030), (0.66, 0.105, -0.008)],
    ]
    return _tube_assembly(segments, 0.0045)


def _notch_rail_mesh() -> MeshGeometry:
    """A small saw-tooth height-adjustment rail under the board."""
    path = [(-0.22, 0.0, -0.032), (-0.22, 0.0, -0.078)]
    # Six V notches formed as one continuous bent rod, with risers up to the underside.
    for i in range(6):
        x = -0.16 + i * 0.07
        if path[-1] != (x, 0.0, -0.078):
            path.append((x, 0.0, -0.078))
        path.extend([(x + 0.020, 0.0, -0.105), (x + 0.040, 0.0, -0.078)])
    path.extend([(0.24, 0.0, -0.078), (0.24, 0.0, -0.032)])
    return _tube(path, 0.0055, "height_notch_rail")


SHELF_RUNG_COUNT = 10
SHELF_X0 = -0.42
SHELF_X1 = 0.48
SHELF_Y0 = -0.06
SHELF_Y1 = 0.06
SHELF_Z = 0.0
SHELF_DROP = 0.109


def _shelf_frame_mesh() -> MeshGeometry:
    """Rectangular perimeter wire frame for the under-board linen shelf."""
    segments = [
        [(SHELF_X0, SHELF_Y0, SHELF_Z), (SHELF_X1, SHELF_Y0, SHELF_Z)],
        [(SHELF_X0, SHELF_Y1, SHELF_Z), (SHELF_X1, SHELF_Y1, SHELF_Z)],
        [(SHELF_X0, SHELF_Y0, SHELF_Z), (SHELF_X0, SHELF_Y1, SHELF_Z)],
        [(SHELF_X1, SHELF_Y0, SHELF_Z), (SHELF_X1, SHELF_Y1, SHELF_Z)],
        # Two mid-span cross braces for rigidity.
        [(0.03, SHELF_Y0, SHELF_Z), (0.03, SHELF_Y1, SHELF_Z)],
        [(-0.20, SHELF_Y0, SHELF_Z), (-0.20, SHELF_Y1, SHELF_Z)],
    ]
    return _tube_assembly(segments, 0.0045)


def _shelf_rung_mesh(y: float) -> MeshGeometry:
    """Single longitudinal shelf rung tube spanning the frame length."""
    return _tube(
        [(SHELF_X0 + 0.005, y, SHELF_Z), (SHELF_X1 - 0.005, y, SHELF_Z)],
        0.003,
        "shelf_rung",
    )


def _shelf_drop_supports_mesh() -> MeshGeometry:
    """Four vertical tubes connecting the shelf frame up to the board underframe."""
    inset = 0.015
    segments = [
        [(SHELF_X0 + inset, SHELF_Y0 + inset, SHELF_Z), (SHELF_X0 + inset, SHELF_Y0 + inset, SHELF_Z + SHELF_DROP)],
        [(SHELF_X0 + inset, SHELF_Y1 - inset, SHELF_Z), (SHELF_X0 + inset, SHELF_Y1 - inset, SHELF_Z + SHELF_DROP)],
        [(SHELF_X1 - inset, SHELF_Y0 + inset, SHELF_Z), (SHELF_X1 - inset, SHELF_Y0 + inset, SHELF_Z + SHELF_DROP)],
        [(SHELF_X1 - inset, SHELF_Y1 - inset, SHELF_Z), (SHELF_X1 - inset, SHELF_Y1 - inset, SHELF_Z + SHELF_DROP)],
    ]
    return _tube_assembly(segments, 0.0045)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_ironing_board")

    blue_fabric = model.material("navy_fabric", rgba=(0.03, 0.09, 0.24, 1.0))
    white_print = model.material("white_dot_print", rgba=(0.92, 0.96, 0.96, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.62, 0.68, 0.68, 1.0))
    shadow = model.material("dark_hole_shadow", rgba=(0.02, 0.025, 0.030, 1.0))
    rubber = model.material("grey_plastic_feet", rgba=(0.33, 0.38, 0.40, 1.0))
    latch_black = model.material("black_release", rgba=(0.025, 0.025, 0.025, 1.0))

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

    # The reference reads as a navy fabric cover with many pale perforation-like dots.
    dot_index = 0
    for row, x in enumerate([BOARD_NOSE_X + 0.13 + i * 0.105 for i in range(13)]):
        hw = _half_width_at_x(x, scale=0.91)
        if hw < 0.030:
            continue
        cols = max(1, int((2.0 * hw - 0.045) / 0.055))
        for c in range(cols):
            if cols == 1:
                y = 0.0
            else:
                y = -hw + 0.030 + c * ((2.0 * hw - 0.060) / (cols - 1))
            if (row + c) % 2:
                y += 0.010
            if abs(y) < hw - 0.022:
                board.visual(
                    Cylinder(radius=0.010, length=0.0014),
                    origin=Origin(xyz=(x, y, 0.0145)),
                    material=white_print,
                    name=f"cover_dot_{dot_index}",
                )
                dot_index += 1

    # A few dark circles on the exposed underside indicate the perforated metal deck.
    for i, x in enumerate([-0.32, -0.20, -0.08, 0.04, 0.16, 0.28, 0.40]):
        for y in (-0.075, 0.075):
            board.visual(
                Cylinder(radius=0.011, length=0.0012),
                origin=Origin(xyz=(x, y, -0.0326)),
                material=shadow,
                name=f"underside_hole_{i}_{0 if y < 0 else 1}",
            )

    board.visual(mesh_from_geometry(_underframe_mesh(), "underside_braces"), material=galvanized, name="underside_braces")
    board.visual(mesh_from_geometry(_wire_rest_mesh(), "rear_wire_rest"), material=galvanized, name="rear_wire_rest")
    board.visual(mesh_from_geometry(_notch_rail_mesh(), "height_notch_rail"), material=galvanized, name="height_notch_rail")

    # Clevis-like hinge brackets: located outside the moving top bars so the joint is visible but not interpenetrating.
    for x, tag, span in ((0.42, "rear", 0.205), (-0.32, "front", 0.140)):
        for side in (-1, 1):
            board.visual(
                Box((0.055, 0.014, 0.050)),
                origin=Origin(xyz=(x, side * span, -0.052)),
                material=galvanized,
                name=f"{tag}_hinge_bracket_{0 if side < 0 else 1}",
            )
            if tag == "rear":
                board.visual(
                    Box((0.060, 0.050, 0.010)),
                    origin=Origin(xyz=(x, side * 0.186, -0.026)),
                    material=galvanized,
                    name=f"rear_hinge_mount_{0 if side < 0 else 1}",
                )

    rear_leg = model.part("rear_leg_pair")
    rear_leg.visual(mesh_from_geometry(_leg_pair_mesh(-1), "rear_leg_tubes"), material=galvanized, name="leg_tubes")
    rear_leg.visual(mesh_from_geometry(_foot_caps_mesh(-1), "rear_foot_caps"), material=rubber, name="foot_caps")
    # Pivot washers on the outer side of the X-frame.
    for y in (-0.168, 0.168):
        rear_leg.visual(
            Cylinder(radius=0.020, length=0.006),
            origin=Origin(xyz=(-0.41, y, -0.335), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=galvanized,
            name=f"pivot_washer_{0 if y < 0 else 1}",
        )

    front_leg = model.part("front_leg_pair")
    front_leg.visual(mesh_from_geometry(_leg_pair_mesh(1), "front_leg_tubes"), material=galvanized, name="leg_tubes")
    front_leg.visual(mesh_from_geometry(_foot_caps_mesh(1), "front_foot_caps"), material=rubber, name="foot_caps")
    for y in (-0.088, 0.088):
        front_leg.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(xyz=(0.39, y, -0.335), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=shadow,
            name=f"pivot_bolt_{0 if y < 0 else 1}",
        )

    height_latch = model.part("height_latch")
    height_latch.visual(
        Box((0.060, 0.055, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=galvanized,
        name="slider_block",
    )
    height_latch.visual(
        Box((0.090, 0.014, 0.010)),
        origin=Origin(xyz=(0.030, -0.032, -0.041), rpy=(0.0, 0.0, -0.20)),
        material=latch_black,
        name="release_tab",
    )
    height_latch.visual(
        Cylinder(radius=0.006, length=0.060),
        origin=Origin(xyz=(-0.020, 0.0, -0.045), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="catch_pin",
    )

    linen_shelf = model.part("linen_shelf")
    linen_shelf.visual(
        mesh_from_geometry(_shelf_frame_mesh(), "shelf_frame"),
        material=galvanized,
        name="shelf_frame",
    )
    linen_shelf.visual(
        mesh_from_geometry(_shelf_drop_supports_mesh(), "shelf_drop_supports"),
        material=galvanized,
        name="drop_supports",
    )
    shelf_inner_y0 = SHELF_Y0 + 0.015
    shelf_inner_y1 = SHELF_Y1 - 0.015
    for i in range(SHELF_RUNG_COUNT):
        if SHELF_RUNG_COUNT == 1:
            rung_y = 0.0
        else:
            rung_y = shelf_inner_y0 + (shelf_inner_y1 - shelf_inner_y0) * i / (SHELF_RUNG_COUNT - 1)
        linen_shelf.visual(
            mesh_from_geometry(_shelf_rung_mesh(rung_y), f"shelf_rung_{i}"),
            material=galvanized,
            name=f"rung_{i}",
        )

    model.articulation(
        "board_to_linen_shelf",
        ArticulationType.FIXED,
        parent=board,
        child=linen_shelf,
        origin=Origin(xyz=(0.05, 0.0, -0.14)),
    )

    model.articulation(
        "board_to_rear_leg",
        ArticulationType.REVOLUTE,
        parent=board,
        child=rear_leg,
        origin=Origin(xyz=(0.42, 0.0, -0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.5, lower=0.0, upper=1.18),
    )
    model.articulation(
        "board_to_front_leg",
        ArticulationType.REVOLUTE,
        parent=board,
        child=front_leg,
        origin=Origin(xyz=(-0.32, 0.0, -0.055)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.5, lower=0.0, upper=1.18),
    )
    model.articulation(
        "board_to_height_latch",
        ArticulationType.PRISMATIC,
        parent=board,
        child=height_latch,
        origin=Origin(xyz=(-0.025, 0.0, -0.082)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.12, lower=0.0, upper=0.11),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    board = object_model.get_part("board")
    rear_leg = object_model.get_part("rear_leg_pair")
    front_leg = object_model.get_part("front_leg_pair")
    latch = object_model.get_part("height_latch")
    shelf = object_model.get_part("linen_shelf")
    rear_hinge = object_model.get_articulation("board_to_rear_leg")
    front_hinge = object_model.get_articulation("board_to_front_leg")
    latch_slide = object_model.get_articulation("board_to_height_latch")

    ctx.allow_overlap(
        board,
        latch,
        elem_a="height_notch_rail",
        elem_b="slider_block",
        reason="The height catch block is intentionally seated into the notched rail as a captured sliding latch.",
    )

    ctx.allow_overlap(
        board,
        shelf,
        elem_a="perforated_plate",
        elem_b="drop_supports",
        reason="The shelf drop supports are intentionally welded into the board underside plate as a fixed structural mount.",
    )

    board_aabb = ctx.part_world_aabb(board)
    ctx.check(
        "board has full ironing-board proportions",
        board_aabb is not None
        and (board_aabb[1][0] - board_aabb[0][0]) > 1.35
        and 0.34 < (board_aabb[1][1] - board_aabb[0][1]) < 0.48,
        details=f"board_aabb={board_aabb}",
    )

    rear_rest = ctx.part_world_aabb(rear_leg)
    front_rest = ctx.part_world_aabb(front_leg)
    ctx.check(
        "crossed legs reach well below the top",
        rear_rest is not None
        and front_rest is not None
        and rear_rest[0][2] < -0.78
        and front_rest[0][2] < -0.78,
        details=f"rear={rear_rest}, front={front_rest}",
    )
    ctx.expect_overlap(rear_leg, front_leg, axes="xz", min_overlap=0.20, name="two leg pairs form a visible X")
    ctx.expect_overlap(latch, board, axes="xy", min_overlap=0.05, name="height latch sits under notched rail")
    ctx.expect_overlap(
        latch,
        board,
        axes="xy",
        elem_a="slider_block",
        elem_b="height_notch_rail",
        min_overlap=0.010,
        name="slider block is retained in the notched rail",
    )
    ctx.expect_gap(
        board,
        latch,
        axis="z",
        positive_elem="height_notch_rail",
        negative_elem="slider_block",
        max_penetration=0.008,
        name="slider block has only shallow seated engagement",
    )

    with ctx.pose({rear_hinge: 1.12, front_hinge: 1.12}):
        rear_folded = ctx.part_world_aabb(rear_leg)
        front_folded = ctx.part_world_aabb(front_leg)
    ctx.check(
        "leg hinges fold feet upward toward the board",
        rear_rest is not None
        and front_rest is not None
        and rear_folded is not None
        and front_folded is not None
        and rear_folded[0][2] > rear_rest[0][2] + 0.40
        and front_folded[0][2] > front_rest[0][2] + 0.40,
        details=f"rear_rest={rear_rest}, rear_folded={rear_folded}, front_rest={front_rest}, front_folded={front_folded}",
    )

    latch_rest = ctx.part_world_position(latch)
    with ctx.pose({latch_slide: 0.10}):
        latch_shifted = ctx.part_world_position(latch)
    ctx.check(
        "height adjustment catch slides along the notched rail",
        latch_rest is not None and latch_shifted is not None and latch_shifted[0] > latch_rest[0] + 0.08,
        details=f"rest={latch_rest}, shifted={latch_shifted}",
    )

    # --- Linen shelf checks ---
    shelf_joint = object_model.get_articulation("board_to_linen_shelf")

    # The shelf must be positioned below the board top surface.
    shelf_aabb = ctx.part_world_aabb(shelf)
    ctx.check(
        "linen_shelf hangs below the board",
        shelf_aabb is not None and shelf_aabb[0][2] < -0.10,
        details=f"shelf_aabb={shelf_aabb}",
    )

    # The shelf must overlap the board footprint in XY (it is carried under it).
    ctx.expect_overlap(
        shelf,
        board,
        axes="xy",
        min_overlap=0.10,
        name="linen_shelf sits within board footprint",
    )

    # Prove the drop supports reach the board underside (structural mount contact).
    ctx.expect_contact(
        board,
        shelf,
        elem_a="perforated_plate",
        elem_b="drop_supports",
        contact_tol=0.005,
        name="shelf drop supports reach the board underside plate",
    )

    # Verify all 10 rungs are present as named visuals.
    rung_names = [f"rung_{i}" for i in range(10)]
    missing_rungs = [n for n in rung_names if shelf.get_visual(n) is None]
    ctx.check(
        "linen_shelf carries 10 evenly spaced rungs",
        len(missing_rungs) == 0,
        details=f"missing_rungs={missing_rungs}",
    )

    # First and last rungs should span the shelf width (pass visual objects, not names).
    rung_first = shelf.get_visual("rung_0")
    rung_last = shelf.get_visual("rung_9")
    if rung_first is not None and rung_last is not None:
        ctx.expect_gap(
            shelf,
            shelf,
            axis="y",
            positive_elem=rung_last,
            negative_elem=rung_first,
            min_gap=0.06,
            max_gap=0.14,
            name="outermost shelf rungs span the rack width",
        )

    return ctx.report()


object_model = build_object_model()
