from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    tube_from_spline_points,
)


AXLE_PIVOT = (0.0, -0.60, 0.25)


def _origin(x: float, y: float, z: float, rpy=(0.0, 0.0, 0.0)) -> Origin:
    return Origin(xyz=(x, y, z), rpy=rpy)


def _barrow_origin(x: float, y: float, z: float, rpy=(0.0, 0.0, 0.0)) -> Origin:
    return Origin(
        xyz=(x - AXLE_PIVOT[0], y - AXLE_PIVOT[1], z - AXLE_PIVOT[2]),
        rpy=rpy,
    )


def _barrow_point(point):
    return tuple(point[i] - AXLE_PIVOT[i] for i in range(3))


def _barrow_mesh(geometry: MeshGeometry) -> MeshGeometry:
    return geometry.translate(-AXLE_PIVOT[0], -AXLE_PIVOT[1], -AXLE_PIVOT[2])


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _add_box_to_mesh(
    geom: MeshGeometry,
    x0: float, x1: float,
    y0: float, y1: float,
    z0: float, z1: float,
) -> None:
    """Add an axis-aligned box as 12 triangles to an existing mesh."""
    v = [
        geom.add_vertex(x0, y0, z0),  # 0: ---
        geom.add_vertex(x1, y0, z0),  # 1: +--
        geom.add_vertex(x1, y1, z0),  # 2: ++-
        geom.add_vertex(x0, y1, z0),  # 3: -+-
        geom.add_vertex(x0, y0, z1),  # 4: --+
        geom.add_vertex(x1, y0, z1),  # 5: +-+
        geom.add_vertex(x1, y1, z1),  # 6: +++
        geom.add_vertex(x0, y1, z1),  # 7: -++
    ]
    # Bottom (-Z)
    geom.add_face(v[0], v[2], v[1])
    geom.add_face(v[0], v[3], v[2])
    # Top (+Z)
    geom.add_face(v[4], v[5], v[6])
    geom.add_face(v[4], v[6], v[7])
    # Front (-Y)
    geom.add_face(v[0], v[1], v[5])
    geom.add_face(v[0], v[5], v[4])
    # Back (+Y)
    geom.add_face(v[2], v[3], v[7])
    geom.add_face(v[2], v[7], v[6])
    # Left (-X)
    geom.add_face(v[0], v[4], v[7])
    geom.add_face(v[0], v[7], v[3])
    # Right (+X)
    geom.add_face(v[1], v[2], v[6])
    geom.add_face(v[1], v[6], v[5])


def _flat_deck_geometry() -> MeshGeometry:
    """Flat load platform deck with shallow kick-rails for a brick barrow."""
    geom = MeshGeometry()

    # Plate dimensions (in barrow-space world coords)
    pw = DECK_WIDTH
    pl = DECK_LENGTH
    pt = 0.007      # plate thickness
    z_bot = 0.380
    z_top = z_bot + pt

    # Kick-rail dimensions
    rh = 0.030      # rail height above plate top
    rw = 0.014      # rail wall thickness
    z_rail_top = z_top + rh

    hw = pw * 0.5
    hl = pl * 0.5
    yc = DECK_CENTER_Y  # Y center offset

    # Main plate
    _add_box_to_mesh(geom, -hw, hw, yc - hl, yc + hl, z_bot, z_top)

    # Front kick-rail (-Y edge)
    _add_box_to_mesh(geom, -hw, hw, yc - hl, yc - hl + rw, z_top, z_rail_top)
    # Rear kick-rail (+Y edge)
    _add_box_to_mesh(geom, -hw, hw, yc + hl - rw, yc + hl, z_top, z_rail_top)
    # Left kick-rail (-X edge, between front and rear rails)
    _add_box_to_mesh(geom, -hw, -hw + rw, yc - hl + rw, yc + hl - rw, z_top, z_rail_top)
    # Right kick-rail (+X edge, between front and rear rails)
    _add_box_to_mesh(geom, hw - rw, hw, yc - hl + rw, yc + hl - rw, z_top, z_rail_top)

    return geom


def _triangular_plate_geometry(
    x_center: float,
    thickness: float,
    yz_points: list[tuple[float, float]],
) -> MeshGeometry:
    """Small fork/bracket plate, a prism whose broad face lies in the YZ plane."""
    geom = MeshGeometry()
    x0 = x_center - thickness * 0.5
    x1 = x_center + thickness * 0.5
    front = [geom.add_vertex(x0, y, z) for y, z in yz_points]
    back = [geom.add_vertex(x1, y, z) for y, z in yz_points]
    n = len(yz_points)

    # Broad faces.
    for i in range(1, n - 1):
        geom.add_face(front[0], front[i], front[i + 1])
        geom.add_face(back[0], back[i + 1], back[i])
    # Edge band.
    for i in range(n):
        j = (i + 1) % n
        _add_quad(geom, front[i], front[j], back[j], back[i])
    return geom


def _tube_mesh(points: list[tuple[float, float, float]], radius: float, name: str):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=16,
            radial_segments=18,
            cap_ends=True,
        ),
        name,
    )


def _barrow_tube_mesh(points: list[tuple[float, float, float]], radius: float, name: str):
    return _tube_mesh([_barrow_point(point) for point in points], radius, name)


# ──────────────────────────────────────────────────────────────────────────────
# Build
# ──────────────────────────────────────────────────────────────────────────────

# Deck layout constants (used by build and tests)
DECK_PLATE_Z_TOP = 0.387  # top surface of deck plate in barrow space
DECK_WIDTH = 0.70
DECK_LENGTH = 0.86
DECK_CENTER_Y = 0.17      # center of deck in barrow-space Y (clears tire rear)
TREAD_STRIP_COUNT = 8


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="agricultural_single_wheelbarrow",
        meta={
            "category": "Agricultural",
            "small_class": "Single-Wheelbarrow",
            "reference": "picture/Agricultural/Single-Wheelbarrow/001.png",
            "variant": "flat_deck_brick_barrow",
        },
    )

    # ── Materials (yellow/black builder's barrow) ──
    yellow = model.material("yellow_painted_steel", rgba=(0.92, 0.78, 0.06, 1.0))
    yellow_dark = model.material("dark_yellow_deck_edge", rgba=(0.78, 0.65, 0.04, 1.0))
    black_frame = model.material("black_painted_frame", rgba=(0.06, 0.06, 0.07, 1.0))
    black_trim = model.material("black_trim_rubber", rgba=(0.04, 0.04, 0.05, 1.0))
    rubber = model.material("black_rubber", rgba=(0.02, 0.02, 0.02, 1.0))
    grip_rubber = model.material("grey_textured_grip", rgba=(0.20, 0.20, 0.22, 1.0))
    hardware = model.material("dark_bolt_hardware", rgba=(0.03, 0.03, 0.03, 1.0))
    tread_mat = model.material("anti_slip_tread", rgba=(0.12, 0.12, 0.13, 1.0))

    axle_pivot = model.part("wheel_axle_pivot")
    barrow = model.part("barrow")

    # ── Flat load platform deck (replaces deep tray) ──
    barrow.visual(
        mesh_from_geometry(_barrow_mesh(_flat_deck_geometry()), "flat_deck_plate"),
        material=yellow,
        name="flat_deck",
    )

    # Anti-slip tread strips across the deck width (diamond-plate style ridges)
    tread_z = DECK_PLATE_Z_TOP + 0.0015  # just above plate surface
    tread_dx = DECK_WIDTH - 2 * 0.014 - 0.04  # inset from kick-rails
    tread_y_start = DECK_CENTER_Y - DECK_LENGTH * 0.5 + 0.06
    tread_y_end = DECK_CENTER_Y + DECK_LENGTH * 0.5 - 0.06
    for i in range(TREAD_STRIP_COUNT):
        frac = (i + 1) / (TREAD_STRIP_COUNT + 1)
        ty = tread_y_start + frac * (tread_y_end - tread_y_start)
        barrow.visual(
            Box((tread_dx, 0.012, 0.003)),
            origin=_barrow_origin(0.0, ty, tread_z),
            material=tread_mat,
            name=f"tread_strip_{i}",
        )

    # ── Tube chassis (unchanged) ──
    left_rail_points = [
        (-0.33, -0.86, 0.22),
        (-0.33, -0.60, 0.25),
        (-0.32, -0.20, 0.30),
        (-0.31, 0.28, 0.35),
        (-0.31, 0.58, 0.56),
        (-0.31, 1.08, 0.88),
        (-0.31, 1.42, 0.95),
    ]
    right_rail_points = [(abs(x), y, z) for x, y, z in left_rail_points]
    barrow.visual(
        _barrow_tube_mesh(left_rail_points, 0.018, "left_handle_rail"),
        material=black_frame, name="handle_rail_0",
    )
    barrow.visual(
        _barrow_tube_mesh(right_rail_points, 0.018, "right_handle_rail"),
        material=black_frame, name="handle_rail_1",
    )

    front_loop_points = [
        (-0.33, -0.86, 0.22),
        (-0.18, -0.92, 0.205),
        (0.0, -0.94, 0.205),
        (0.18, -0.92, 0.205),
        (0.33, -0.86, 0.22),
    ]
    barrow.visual(
        _barrow_tube_mesh(front_loop_points, 0.018, "front_guard_loop"),
        material=black_frame, name="front_guard",
    )

    for y, z, nm in [(-0.22, 0.300, "front_cross_tube"), (0.28, 0.355, "rear_cross_tube")]:
        barrow.visual(
            Cylinder(radius=0.016, length=0.68),
            origin=_barrow_origin(0.0, y, z, (0.0, math.pi / 2.0, 0.0)),
            material=black_frame,
            name=nm,
        )

    # ── Support legs (unchanged geometry) ──
    for x, nm in [(-0.31, "support_leg_0"), (0.31, "support_leg_1")]:
        leg_points = [
            (x, 0.06, 0.33),
            (x, 0.22, 0.18),
            (x, 0.40, 0.045),
            (x, 0.60, 0.045),
            (x, 0.72, 0.36),
        ]
        barrow.visual(_barrow_tube_mesh(leg_points, 0.016, nm), material=black_frame, name=nm)
        barrow.visual(
            Box((0.070, 0.055, 0.012)),
            origin=_barrow_origin(x, 0.50, 0.035),
            material=black_frame,
            name=f"foot_pad_{0 if x < 0 else 1}",
        )

    # ── Tray/deck mounting pads (adjusted Z to bridge frame to flat deck) ──
    for x in (-0.18, 0.18):
        for y in (-0.12, 0.24):
            suffix = f"{0 if x < 0 else 1}_{0 if y < 0 else 1}"
            # Front mounts need more height to reach deck; rear mounts are shorter.
            mount_z_center = 0.350 if y < 0 else 0.365
            mount_height = 0.064 if y < 0 else 0.034
            barrow.visual(
                Box((0.085, 0.060, mount_height)),
                origin=_barrow_origin(x, y, mount_z_center),
                material=black_frame,
                name=f"tray_mount_{suffix}",
            )

    # ── Rubber handle grips with ring texture ──
    for x, idx in [(-0.31, 0), (0.31, 1)]:
        barrow.visual(
            Cylinder(radius=0.035, length=0.18),
            origin=_barrow_origin(x, 1.505, 0.965, (-math.pi / 2.0, 0.0, 0.0)),
            material=grip_rubber,
            name=f"grip_{idx}",
        )
        for ring_i, y in enumerate((1.445, 1.485, 1.525, 1.565)):
            barrow.visual(
                Cylinder(radius=0.037, length=0.008),
                origin=_barrow_origin(x, y, 0.965, (-math.pi / 2.0, 0.0, 0.0)),
                material=hardware,
                name=f"grip_ring_{idx}_{ring_i}",
            )

    # ── Axle assembly and fork plates (unchanged) ──
    barrow.visual(
        Cylinder(radius=0.022, length=0.74),
        origin=_barrow_origin(0.0, -0.60, 0.25, (0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="axle",
    )
    plate_yz = [(-0.735, 0.185), (-0.505, 0.205), (-0.475, 0.318), (-0.630, 0.355), (-0.750, 0.292)]
    for x, idx in [(-0.090, 0), (0.090, 1)]:
        barrow.visual(
            mesh_from_geometry(
                _barrow_mesh(_triangular_plate_geometry(x, 0.018, plate_yz)),
                f"axle_bracket_{idx}",
            ),
            material=black_frame,
            name=f"axle_bracket_{idx}",
        )
        for y, z, rad, length, bolt_idx in [
            (-0.600, 0.250, 0.026, 0.014, 0),
            (-0.680, 0.278, 0.015, 0.010, 1),
            (-0.520, 0.290, 0.015, 0.010, 2),
        ]:
            barrow.visual(
                Cylinder(radius=rad, length=length),
                origin=_barrow_origin(
                    x + (0.014 if x > 0 else -0.014), y, z,
                    (0.0, math.pi / 2.0, 0.0),
                ),
                material=hardware,
                name=f"bracket_bolt_{idx}_{bolt_idx}",
            )

    # ── Front struts (connect flat deck front to axle bracket area) ──
    for x, idx in [(-0.27, 0), (0.27, 1)]:
        strut_points = [(x, -0.26, 0.40), (x, -0.44, 0.34), (x, -0.65, 0.28)]
        barrow.visual(
            _barrow_tube_mesh(strut_points, 0.014, f"front_strut_{idx}"),
            material=black_frame, name=f"front_strut_{idx}",
        )

    # ── Wheel child part (unchanged) ──
    wheel = model.part("wheel")
    tire = TireGeometry(
        0.235,
        0.112,
        inner_radius=0.158,
        carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.08),
        tread=TireTread(style="block", depth=0.010, count=24, land_ratio=0.55),
        grooves=(
            TireGroove(center_offset=0.0, width=0.010, depth=0.004),
            TireGroove(center_offset=-0.030, width=0.006, depth=0.0025),
            TireGroove(center_offset=0.030, width=0.006, depth=0.0025),
        ),
        sidewall=TireSidewall(style="rounded", bulge=0.06),
        shoulder=TireShoulder(width=0.012, radius=0.004),
    )
    wheel.visual(mesh_from_geometry(tire, "rubber_tire"), material=rubber, name="rubber_tire")

    rim = WheelGeometry(
        0.154,
        0.082,
        rim=WheelRim(inner_radius=0.080, flange_height=0.012, flange_thickness=0.006, bead_seat_depth=0.005),
        hub=WheelHub(
            radius=0.048,
            width=0.074,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=5, circle_diameter=0.062, hole_diameter=0.006),
        ),
        face=WheelFace(dish_depth=0.010, front_inset=0.004, rear_inset=0.004),
        spokes=WheelSpokes(style="split_y", count=5, thickness=0.006, window_radius=0.020),
        bore=WheelBore(style="round", diameter=0.044),
    )
    wheel.visual(mesh_from_geometry(rim, "metal_rim"), material=yellow_dark, name="metal_rim")

    # Valve stem (makes wheel rotation visibly pose-dependent)
    wheel.visual(
        Cylinder(radius=0.004, length=0.016),
        origin=Origin(xyz=(0.063, 0.055, 0.145), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="valve_stem",
    )

    # ── Articulations (unchanged graph) ──
    model.articulation(
        "axle_pivot_to_barrow",
        ArticulationType.REVOLUTE,
        parent=axle_pivot,
        child=barrow,
        origin=_origin(*AXLE_PIVOT),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=34.0, velocity=1.0, lower=0.0, upper=1.05),
    )

    model.articulation(
        "axle_pivot_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=axle_pivot,
        child=wheel,
        origin=_origin(*AXLE_PIVOT),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=25.0),
    )

    return model


# ──────────────────────────────────────────────────────────────────────────────
# Tests
# ──────────────────────────────────────────────────────────────────────────────

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrow = object_model.get_part("barrow")
    wheel = object_model.get_part("wheel")
    body_tip = object_model.get_articulation("axle_pivot_to_barrow")
    wheel_spin = object_model.get_articulation("axle_pivot_to_wheel")

    # Category and structure
    ctx.check(
        "small class is Single-Wheelbarrow",
        object_model.meta.get("small_class") == "Single-Wheelbarrow",
        details=f"meta={object_model.meta}",
    )
    ctx.check(
        "agricultural single wheelbarrow has axle pivot, tipping body, and moving wheel",
        len(object_model.parts) == 3
        and body_tip.parent == "wheel_axle_pivot"
        and body_tip.child == "barrow"
        and wheel_spin.parent == "wheel_axle_pivot"
        and wheel_spin.child == "wheel",
        details=f"parts={[p.name for p in object_model.parts]}, articulations={[a.name for a in object_model.articulations]}",
    )
    ctx.check(
        "front wheel joint spins about axle",
        wheel_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(wheel_spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={wheel_spin.articulation_type}, axis={wheel_spin.axis}",
    )
    ctx.check(
        "barrow body tips forward around the same wheel axle",
        body_tip.articulation_type == ArticulationType.REVOLUTE
        and tuple(body_tip.axis) == (1.0, 0.0, 0.0)
        and body_tip.motion_limits.lower == 0.0
        and body_tip.motion_limits.upper >= 1.0,
        details=f"type={body_tip.articulation_type}, axis={body_tip.axis}, limits={body_tip.motion_limits}",
    )

    # Axle/wheel overlap allowance
    ctx.allow_overlap(
        barrow,
        wheel,
        elem_a="axle",
        elem_b="metal_rim",
        reason=(
            "The fixed steel axle is intentionally captured inside the wheel hub/bushing; "
            "this local hidden fit supports the rotating wheel."
        ),
    )

    # ── Flat deck variant checks (TARGET-specific) ──
    deck_aabb = ctx.part_element_world_aabb(barrow, elem="flat_deck")
    ctx.check(
        "flat_deck load platform has brick-barrow proportions (wide, low, no deep walls)",
        deck_aabb is not None
        and deck_aabb[1][0] - deck_aabb[0][0] > 0.65   # width > 0.65m
        and deck_aabb[1][1] - deck_aabb[0][1] > 0.80   # length > 0.80m
        and deck_aabb[1][2] - deck_aabb[0][2] < 0.060  # total height < 60mm (plate + kick-rails only)
        and deck_aabb[1][2] - deck_aabb[0][2] > 0.025,  # but more than just plate thickness
        details=f"flat_deck_aabb={deck_aabb}",
    )

    # Anti-slip tread strips present on deck
    tread_count = 0
    for i in range(TREAD_STRIP_COUNT):
        if ctx.part_element_world_aabb(barrow, elem=f"tread_strip_{i}") is not None:
            tread_count += 1
    ctx.check(
        f"anti-slip tread strips present on flat deck (found {tread_count}/{TREAD_STRIP_COUNT})",
        tread_count == TREAD_STRIP_COUNT,
        details=f"expected {TREAD_STRIP_COUNT} tread_strip_* visuals on barrow part",
    )

    # Variant marker in meta
    ctx.check(
        "variant metadata identifies flat_deck_brick_barrow",
        object_model.meta.get("variant") == "flat_deck_brick_barrow",
        details=f"meta={object_model.meta}",
    )

    # Key visuals present
    for visual_name in (
        "handle_rail_0",
        "handle_rail_1",
        "grip_0",
        "grip_1",
        "front_guard",
        "support_leg_0",
        "support_leg_1",
        "axle",
        "axle_bracket_0",
        "axle_bracket_1",
        "front_strut_0",
        "front_strut_1",
        "rubber_tire",
        "metal_rim",
    ):
        part = wheel if visual_name in {"rubber_tire", "metal_rim"} else barrow
        ctx.check(
            f"visible {visual_name} present",
            ctx.part_element_world_aabb(part, elem=visual_name) is not None,
            details=visual_name,
        )

    # Wheel centering on axle
    ctx.expect_within(
        wheel,
        barrow,
        axes="x",
        inner_elem="rubber_tire",
        outer_elem="axle",
        margin=0.0,
        name="single wheel is centered between axle ends",
    )
    ctx.expect_overlap(
        wheel,
        barrow,
        axes="x",
        elem_a="metal_rim",
        elem_b="axle",
        min_overlap=0.07,
        name="rim hub surrounds the axle along its width",
    )

    # Wheel rotation moves valve stem
    valve0 = ctx.part_element_world_aabb(wheel, elem="valve_stem")
    with ctx.pose({wheel_spin: 1.25}):
        valve1 = ctx.part_element_world_aabb(wheel, elem="valve_stem")

    def _center_yz(aabb):
        return ((aabb[0][1] + aabb[1][1]) * 0.5, (aabb[0][2] + aabb[1][2]) * 0.5)

    moved = False
    if valve0 is not None and valve1 is not None:
        y0, z0 = _center_yz(valve0)
        y1, z1 = _center_yz(valve1)
        moved = abs(y1 - y0) > 0.035 and abs(z1 - z0) > 0.035
    ctx.check("wheel rotation moves valve stem around axle", moved, details=f"rest={valve0}, posed={valve1}")

    # Body tip swings the whole barrow forward
    def _center_xyz(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    grip0 = ctx.part_element_world_aabb(barrow, elem="grip_0")
    with ctx.pose({body_tip: 0.65}):
        grip1 = ctx.part_element_world_aabb(barrow, elem="grip_0")
    tipped = False
    if grip0 is not None and grip1 is not None:
        rest_center = _center_xyz(grip0)
        tip_center = _center_xyz(grip1)
        tipped = tip_center[2] > rest_center[2] + 0.75 and tip_center[1] < rest_center[1] - 0.45
    ctx.check(
        "body tip joint swings the whole barrow forward around the wheel",
        tipped, details=f"rest={grip0}, tipped={grip1}",
    )

    return ctx.report()


object_model = build_object_model()
