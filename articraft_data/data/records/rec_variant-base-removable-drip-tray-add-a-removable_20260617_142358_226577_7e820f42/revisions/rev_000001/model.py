from __future__ import annotations

"""Triple-faucet countertop beverage dispensing tower with removable drip tray.

Variant (base = removable_drip_tray): the drip tray slides out toward the
front on a prismatic joint. The tower body (base frame + column + faucets)
is the fixed root. Three chrome faucets fan out near the top of the column
at -40/0/+40 degrees; each carries a glossy black tapered tap handle on a
chrome lever collar. Each handle is an independent revolute joint about a
horizontal axis through its faucet body: q=0 is upright/closed, q=+0.70 rad
(~40 deg) pulls the handle forward over the spout to dispense.
"""

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    CapsuleGeometry,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)

# ---------------------------------------------------------------- constants
NUM_FAUCETS = 3

TRAY_X = 0.45  # tray width (left-right)
TRAY_Y = 0.32  # tray depth (front-back); front faces -Y
TRAY_H = 0.030

BASE_T = 0.005  # base plate thickness
BASE_X = TRAY_X + 0.014  # base frame width
BASE_Y = TRAY_Y + 0.014  # base frame depth
RAIL_W = 0.006  # guide rail width
RAIL_H = TRAY_H + 0.004  # guide rail height above base plate

TRAY_REST_Z = BASE_T  # tray sits on the base plate

COLUMN_Y = 0.065  # column axis toward the rear
COLUMN_BASE_Z = TRAY_REST_Z + TRAY_H - 0.0005  # flare seated 0.5 mm into plate
COLUMN_R = 0.0425
COLUMN_SHAFT_H = 0.402

FAN_ANGLE = math.radians(40.0)
FAUCET_REACH = 0.100
PIVOT_RISE = 0.027
HANDLE_LEN = 0.141
TAP_TRAVEL = 0.70  # rad, ~40 degrees forward

TRAY_SLIDE = 0.20  # prismatic travel distance (m)

# Faucet configs: (yaw_angle, height_on_column relative to column base)
FAUCET_CONFIGS = (
    (-FAN_ANGLE, 0.334),  # i=0, left
    (0.0, 0.350),         # i=1, center
    (FAN_ANGLE, 0.334),   # i=2, right
)


def _fan_dir(yaw: float) -> tuple[float, float]:
    """Horizontal unit vector of a faucet: yaw=0 points to the front (-Y)."""
    return (math.sin(yaw), -math.cos(yaw))


# -------------------------------------------------------- geometry helpers
def _build_base_frame() -> object:
    """CadQuery base frame: flat plate + side guide rails + back stop."""
    # Base plate with rounded corners
    base = (
        cq.Workplane("XY")
        .box(BASE_X, BASE_Y, BASE_T, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.020)
    )

    # Left guide rail
    left_rail = (
        cq.Workplane("XY")
        .box(RAIL_W, BASE_Y - 0.010, RAIL_H, centered=(True, True, False))
        .translate((-TRAY_X / 2.0 - RAIL_W / 2.0, 0.0, BASE_T))
    )

    # Right guide rail
    right_rail = (
        cq.Workplane("XY")
        .box(RAIL_W, BASE_Y - 0.010, RAIL_H, centered=(True, True, False))
        .translate((TRAY_X / 2.0 + RAIL_W / 2.0, 0.0, BASE_T))
    )

    # Back stop rail
    back_stop = (
        cq.Workplane("XY")
        .box(BASE_X, RAIL_W, RAIL_H, centered=(True, True, False))
        .translate((0.0, BASE_Y / 2.0 - RAIL_W / 2.0, BASE_T))
    )

    return base.union(left_rail).union(right_rail).union(back_stop)


def _build_tray_rim() -> ExtrudeWithHolesGeometry:
    return ExtrudeWithHolesGeometry(
        rounded_rect_profile(TRAY_X, TRAY_Y, 0.030),
        [rounded_rect_profile(TRAY_X - 0.024, TRAY_Y - 0.024, 0.022)],
        TRAY_H,
        center=False,
    )


def _build_tray_floor() -> ExtrudeGeometry:
    return ExtrudeGeometry.from_z0(
        rounded_rect_profile(TRAY_X - 0.020, TRAY_Y - 0.020, 0.024), 0.006
    )


def _build_tray_plate() -> PerforatedPanelGeometry:
    return PerforatedPanelGeometry(
        (TRAY_X - 0.020, TRAY_Y - 0.020),
        0.004,
        hole_diameter=0.006,
        pitch=(0.015, 0.015),
        frame=0.014,
        corner_radius=0.020,
        stagger=True,
    )


def _build_flare_collar() -> LatheGeometry:
    return LatheGeometry(
        [
            (0.0, 0.0),
            (0.075, 0.0),
            (0.068, 0.006),
            (0.055, 0.020),
            (0.047, 0.040),
            (COLUMN_R, 0.060),
            (0.0, 0.060),
        ],
        segments=48,
    )


def _build_faucet_body(dx: float, dy: float, yaw: float, z_f: float) -> CapsuleGeometry:
    return (
        CapsuleGeometry(0.0145, 0.035)
        .rotate_x(math.pi / 2.0)
        .rotate_z(yaw)
        .translate(dx * FAUCET_REACH, COLUMN_Y + dy * FAUCET_REACH, z_f)
    )


def _build_spout_tube(dx: float, dy: float, z_f: float):
    spout_pts = [
        (dx * s, COLUMN_Y + dy * s, z_f + dz)
        for s, dz in ((0.105, 0.0), (0.126, -0.003), (0.137, -0.016), (0.139, -0.033))
    ]
    return tube_from_spline_points(
        spout_pts, radius=0.0085, samples_per_segment=10, radial_segments=18
    )


def _build_nozzle(dx: float, dy: float, z_f: float) -> LatheGeometry:
    return LatheGeometry(
        [(0.0, 0.0), (0.0060, 0.0), (0.0092, 0.016), (0.0, 0.016)], segments=24
    ).translate(dx * 0.139, COLUMN_Y + dy * 0.139, z_f - 0.049)


def _build_handle_collar() -> LatheGeometry:
    return LatheGeometry(
        [(0.0, 0.0), (0.0065, 0.0), (0.0105, 0.020), (0.0, 0.020)], segments=24
    ).translate(0.0, 0.0, 0.001)


def _build_handle_grip() -> LatheGeometry:
    return LatheGeometry(
        [
            (0.0, 0.018),
            (0.0090, 0.0205),
            (0.0078, 0.045),
            (0.0108, 0.118),
            (0.0085, 0.136),
            (0.0, HANDLE_LEN),
        ],
        segments=28,
    )


# ---------------------------------------------------------------- build
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="triple_faucet_tower_removable_tray")

    steel = Material("brushed_steel", rgba=(0.58, 0.60, 0.63, 1.0))
    well_steel = Material("tray_well_steel", rgba=(0.36, 0.38, 0.41, 1.0))
    chrome = Material("chrome", rgba=(0.80, 0.82, 0.86, 1.0))
    gloss_black = Material("gloss_black", rgba=(0.05, 0.05, 0.06, 1.0))
    dark_steel = Material("frame_steel", rgba=(0.40, 0.42, 0.45, 1.0))

    # ---------------------------------------------------------- tower body (root)
    tower = model.part("tower_body")

    # Base frame with guide rails (CadQuery)
    tower.visual(
        mesh_from_cadquery(_build_base_frame(), "base_frame"),
        material=dark_steel,
        name="base_frame",
    )

    # Column shaft: extends from base plate top to column top
    shaft_bottom_z = BASE_T
    shaft_top_z = COLUMN_BASE_Z + COLUMN_SHAFT_H
    shaft_length = shaft_top_z - shaft_bottom_z
    tower.visual(
        Cylinder(COLUMN_R, shaft_length),
        origin=Origin(xyz=(0.0, COLUMN_Y, (shaft_bottom_z + shaft_top_z) / 2.0)),
        material=steel,
        name="column_shaft",
    )

    # Column cap
    tower.visual(
        Cylinder(COLUMN_R + 0.0015, 0.006),
        origin=Origin(xyz=(0.0, COLUMN_Y, COLUMN_BASE_Z + COLUMN_SHAFT_H + 0.003)),
        material=steel,
        name="column_cap",
    )

    # Flare collar at column base
    tower.visual(
        mesh_from_geometry(_build_flare_collar(), "flare_collar_mesh"),
        origin=Origin(xyz=(0.0, COLUMN_Y, COLUMN_BASE_Z)),
        material=steel,
        name="flare_collar",
    )

    # Faucets (visuals on tower body, indexed)
    for i in range(NUM_FAUCETS):
        yaw, z_local = FAUCET_CONFIGS[i]
        dx, dy = _fan_dir(yaw)
        z_f = COLUMN_BASE_Z + z_local  # world z of faucet centerline

        # Shank: horizontal chrome cylinder from column wall outward
        tower.visual(
            Cylinder(0.011, 0.060),
            origin=Origin(
                xyz=(dx * 0.0675, COLUMN_Y + dy * 0.0675, z_f),
                rpy=(math.pi / 2.0, 0.0, yaw),
            ),
            material=chrome,
            name=f"faucet_shank_{i}",
        )

        # Faucet body: short horizontal chrome capsule
        tower.visual(
            mesh_from_geometry(_build_faucet_body(dx, dy, yaw, z_f), f"faucet_body_{i}_mesh"),
            material=chrome,
            name=f"faucet_body_{i}",
        )

        # Downward-curved spout tube
        tower.visual(
            mesh_from_geometry(_build_spout_tube(dx, dy, z_f), f"faucet_spout_{i}_mesh"),
            material=chrome,
            name=f"faucet_spout_{i}",
        )

        # Tapered nozzle
        tower.visual(
            mesh_from_geometry(_build_nozzle(dx, dy, z_f), f"faucet_nozzle_{i}_mesh"),
            material=chrome,
            name=f"faucet_nozzle_{i}",
        )

        # Bonnet: fixed chrome boss on top of the body
        tower.visual(
            Cylinder(0.012, 0.017),
            origin=Origin(xyz=(dx * FAUCET_REACH, COLUMN_Y + dy * FAUCET_REACH, z_f + 0.0185)),
            material=chrome,
            name=f"faucet_bonnet_{i}",
        )

    # ---------------------------------------------------------- drip tray (prismatic)
    tray = model.part("drip_tray")

    tray.visual(
        mesh_from_geometry(_build_tray_rim(), "tray_rim_mesh"),
        material=steel,
        name="tray_rim",
    )
    tray.visual(
        mesh_from_geometry(_build_tray_floor(), "tray_floor_mesh"),
        material=well_steel,
        name="tray_floor",
    )
    tray.visual(
        mesh_from_geometry(_build_tray_plate(), "tray_plate_mesh"),
        origin=Origin(xyz=(0.0, 0.0, TRAY_H - 0.002)),
        material=steel,
        name="perforated_plate",
    )

    # Prismatic joint: tray slides toward front (-Y)
    model.articulation(
        "tray_slide",
        ArticulationType.PRISMATIC,
        parent=tower,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, TRAY_REST_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0, velocity=0.4, lower=0.0, upper=TRAY_SLIDE
        ),
    )

    # ---------------------------------------------------------- tap handles (revolute)
    for i in range(NUM_FAUCETS):
        yaw, z_local = FAUCET_CONFIGS[i]
        dx, dy = _fan_dir(yaw)
        z_f = COLUMN_BASE_Z + z_local

        handle = model.part(f"tap_handle_{i}")
        handle.visual(
            Cylinder(0.005, 0.018),
            origin=Origin(xyz=(0.0, 0.0, -0.003)),
            material=chrome,
            name="pivot_stem",
        )
        handle.visual(
            mesh_from_geometry(_build_handle_collar(), f"handle_collar_{i}_mesh"),
            material=chrome,
            name="lever_collar",
        )
        handle.visual(
            mesh_from_geometry(_build_handle_grip(), f"handle_grip_{i}_mesh"),
            material=gloss_black,
            name="tap_grip",
        )

        # Revolute pivot through the faucet bonnet
        model.articulation(
            f"tap_pivot_{i}",
            ArticulationType.REVOLUTE,
            parent=tower,
            child=handle,
            origin=Origin(
                xyz=(dx * FAUCET_REACH, COLUMN_Y + dy * FAUCET_REACH, z_f + PIVOT_RISE),
                rpy=(0.0, 0.0, yaw),
            ),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0, velocity=4.0, lower=0.0, upper=TAP_TRAVEL
            ),
        )

    return model


# ---------------------------------------------------------------- tests
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tower = object_model.get_part("tower_body")
    tray = object_model.get_part("drip_tray")
    slide = object_model.get_articulation("tray_slide")
    handles = {i: object_model.get_part(f"tap_handle_{i}") for i in range(NUM_FAUCETS)}
    pivots = {i: object_model.get_articulation(f"tap_pivot_{i}") for i in range(NUM_FAUCETS)}
    yaws = {i: FAUCET_CONFIGS[i][0] for i in range(NUM_FAUCETS)}

    # -------- intentional, scoped overlaps
    # Tray elements seat on the base frame plate between guide rails.
    # The base frame U-channel convex hull encompasses the tray seating area.
    for tray_elem in ("tray_rim", "tray_floor", "perforated_plate"):
        ctx.allow_overlap(
            tray,
            tower,
            elem_a=tray_elem,
            elem_b="base_frame",
            reason=f"Removable drip tray {tray_elem} seats inside the base frame U-channel between guide rails.",
        )
    ctx.expect_contact(
        tray,
        tower,
        elem_a="tray_rim",
        elem_b="base_frame",
        name="tray rim contacts base frame at seating face",
    )
    # Column shaft and flare collar pass through the removable tray inner well
    ctx.allow_overlap(
        tower,
        tray,
        elem_a="column_shaft",
        elem_b="tray_rim",
        reason="Column shaft passes through the tray inner well cutout; the removable tray slides around the column.",
    )
    ctx.expect_within(
        tower,
        tray,
        axes="xy",
        inner_elem="column_shaft",
        outer_elem="tray_rim",
        margin=0.01,
        name="column shaft stays within tray well footprint",
    )
    ctx.allow_overlap(
        tower,
        tray,
        elem_a="column_shaft",
        elem_b="tray_floor",
        reason="Column shaft passes through the removable tray inner well; the tray slides around the column.",
    )
    ctx.allow_overlap(
        tower,
        tray,
        elem_a="column_shaft",
        elem_b="perforated_plate",
        reason="Column shaft passes through the tray perforated plate well area.",
    )
    ctx.allow_overlap(
        tower,
        tray,
        elem_a="flare_collar",
        elem_b="perforated_plate",
        reason="Flared column collar is seated 0.5 mm into the perforated drip-tray plate.",
    )
    ctx.allow_overlap(
        tower,
        tray,
        elem_a="flare_collar",
        elem_b="tray_floor",
        reason="Flare collar base overlaps tray floor in the inner well region.",
    )
    for i in range(NUM_FAUCETS):
        ctx.allow_overlap(
            handles[i],
            tower,
            elem_a="pivot_stem",
            elem_b=f"faucet_bonnet_{i}",
            reason="Captured handle pivot stem is intentionally nested inside the faucet bonnet.",
        )

    # -------- prismatic tray slide joint
    ctx.check(
        "tray slide is prismatic",
        str(slide.articulation_type).lower().endswith("prismatic"),
        details=f"type={slide.articulation_type}",
    )
    slide_limits = slide.motion_limits
    ctx.check(
        "tray slide travels 0 to ~0.20 m forward",
        slide_limits is not None
        and slide_limits.lower is not None
        and slide_limits.upper is not None
        and abs(slide_limits.lower) <= 1e-9
        and 0.15 <= slide_limits.upper <= 0.25,
        details=f"limits={slide_limits}",
    )

    # -------- tray at rest is seated in the base frame
    tray_rest_aabb = ctx.part_world_aabb(tray)
    ctx.check("tray AABB resolves at rest", tray_rest_aabb is not None)
    if tray_rest_aabb is not None:
        (x0, y0, z0), (x1, y1, z1) = tray_rest_aabb
        ctx.check(
            "tray at rest is ~0.45 x 0.32 m and near z=0",
            0.44 <= (x1 - x0) <= 0.46
            and 0.31 <= (y1 - y0) <= 0.33
            and z0 <= 0.010
            and 0.030 <= z1 <= 0.042,
            details=f"tray aabb={tray_rest_aabb}",
        )

    # -------- tray slides out toward -Y (front)
    rest_center_y = None
    extended_center_y = None
    if tray_rest_aabb is not None:
        rest_center_y = 0.5 * (tray_rest_aabb[0][1] + tray_rest_aabb[1][1])
    with ctx.pose({slide: TRAY_SLIDE}):
        extended_aabb = ctx.part_world_aabb(tray)
        if extended_aabb is not None:
            extended_center_y = 0.5 * (extended_aabb[0][1] + extended_aabb[1][1])

    if rest_center_y is not None and extended_center_y is not None:
        y_shift = rest_center_y - extended_center_y  # positive means moved toward -Y
        ctx.check(
            "tray slides forward (toward -Y) when extended",
            y_shift > 0.15,
            details=f"rest_y={rest_center_y:.4f}, extended_y={extended_center_y:.4f}, shift={y_shift:.4f}",
        )

    # -------- physical scale: base frame grounded at z=0
    base_aabb = ctx.part_world_aabb(tower)
    ctx.check(
        "tower body grounded at z~0",
        base_aabb is not None and abs(base_aabb[0][2]) <= 0.003,
        details=f"tower aabb={base_aabb}",
    )

    # -------- column top height
    col_top = base_aabb[1][2] if base_aabb is not None else 0
    ctx.check(
        "tower top is ~0.43-0.50 m above floor",
        0.42 <= col_top <= 0.52,
        details=f"tower top z={col_top}",
    )

    # -------- tray stays within base frame footprint at rest
    ctx.expect_within(
        tray, tower, axes="xy", margin=0.005, name="tray within base frame at rest"
    )

    # -------- tray contacts base plate (seated on the frame)
    ctx.expect_contact(
        tray,
        tower,
        elem_a="tray_rim",
        elem_b="base_frame",
        name="tray rim seats on base frame",
    )
    # Verify tray bottom is at the expected height
    if tray_rest_aabb is not None:
        ctx.check(
            "tray bottom seats at base plate top",
            abs(tray_rest_aabb[0][2] - BASE_T) <= 0.002,
            details=f"tray bottom z={tray_rest_aabb[0][2]:.4f}, expected={BASE_T}",
        )

    # -------- handles: joint plan and pose checks
    for i in range(NUM_FAUCETS):
        joint = pivots[i]
        ctx.check(
            f"tap pivot {i} is revolute",
            str(joint.articulation_type).lower().endswith("revolute"),
            details=f"type={joint.articulation_type}",
        )
        limits = joint.motion_limits
        ctx.check(
            f"tap pivot {i} travels 0 to ~40 deg",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and abs(limits.lower) <= 1e-9
            and 0.60 <= limits.upper <= 0.80,
            details=f"limits={limits}",
        )

    # -------- pulling each tap tips its handle forward over its own spout
    for i in range(NUM_FAUCETS):
        dx, dy = _fan_dir(yaws[i])
        rest = ctx.part_world_aabb(handles[i])
        with ctx.pose({pivots[i]: TAP_TRAVEL}):
            pulled = ctx.part_world_aabb(handles[i])
        if rest is None or pulled is None:
            ctx.fail(f"handle {i} pose AABBs resolve", "missing AABB")
            continue
        rest_c = (0.5 * (rest[0][0] + rest[1][0]), 0.5 * (rest[0][1] + rest[1][1]))
        pull_c = (0.5 * (pulled[0][0] + pulled[1][0]), 0.5 * (pulled[0][1] + pulled[1][1]))
        forward = (pull_c[0] - rest_c[0]) * dx + (pull_c[1] - rest_c[1]) * dy
        drop = rest[1][2] - pulled[1][2]
        ctx.check(
            f"pulled handle {i} swings forward and its tip drops",
            forward > 0.025 and drop > 0.015,
            details=f"forward={forward:.4f}, top drop={drop:.4f}",
        )

    # -------- handles contact their bonnets
    for i in range(NUM_FAUCETS):
        ctx.expect_contact(
            handles[i],
            tower,
            name=f"handle {i} stem captured in faucet bonnet",
        )

    # -------- faucets occupy distinct fan stations
    handle_aabbs = {i: ctx.part_world_aabb(handles[i]) for i in range(NUM_FAUCETS)}
    if all(a is not None for a in handle_aabbs.values()):
        cx = {i: 0.5 * (handle_aabbs[i][0][0] + handle_aabbs[i][1][0]) for i in range(NUM_FAUCETS)}
        ctx.check(
            "handles occupy distinct fan stations (left/center/right)",
            cx[0] < -0.04 and abs(cx[1]) < 0.02 and cx[2] > 0.04,
            details=f"handle x-centers={cx}",
        )

    return ctx.report()


object_model = build_object_model()
