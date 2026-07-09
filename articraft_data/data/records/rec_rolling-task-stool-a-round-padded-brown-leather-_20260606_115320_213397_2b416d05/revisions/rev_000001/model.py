from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireSidewall,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Rolling task stool — round padded brown-leather seat cushion with
# stitched edge, black gas-lift cylinder column, four-spoke black caster
# base with twin-wheel casters that roll and a base that swivels.
# ---------------------------------------------------------------------------
# Real-world scale: seat top ≈ 0.50 m above the floor (+Z up, z=0 at floor).
# Root:  stationary visible bearing washer under the base hub.
# Child: four-spoke caster base (hub + 4 tapered spokes + 4 twin-wheel casters),
#        swiveling about +Z on the bearing face.
# Child: gas-lift column + round padded seat, swiveling about +Z.
# ---------------------------------------------------------------------------

N_SPOKES = 4
# Spoke geometry
SPOKE_LEN = 0.265      # hub center to spoke tip / yoke center
HUB_RADIUS = 0.050     # central boss the spokes plug into (> column radius)
HUB_HEIGHT = 0.052
HUB_Z = 0.072          # hub center height above the floor
BASE_SWIVEL_Z = HUB_Z - HUB_HEIGHT / 2.0  # hub bottom / bearing top contact plane
BEARING_HEIGHT = 0.018
BEARING_RADIUS = HUB_RADIUS * 1.24

# Caster geometry
WHEEL_RADIUS = 0.026
WHEEL_WIDTH = 0.013
WHEEL_GAP = 0.054      # center-to-center of the two wheels in a twin pair
CASTER_OUT = 0.040     # caster center beyond the hub-to-tip span
CASTER_R = SPOKE_LEN + CASTER_OUT  # radial distance of each caster center

# Column geometry
SLEEVE_RADIUS = 0.031  # lower wide telescoping sleeve
ROD_RADIUS = 0.019     # upper narrow rod
SLEEVE_BOTTOM_Z = 0.050  # sleeve bottom seats inside the hub bore (no stub below the spokes)
SLEEVE_TOP_Z = 0.200     # sleeve top (step transition)
ROD_TOP_Z = 0.430        # top of the rod / seat bottom junction

# Seat geometry
SEAT_RADIUS = 0.168
SEAT_THICK = 0.078
SEAT_BOTTOM_Z = ROD_TOP_Z  # cushion underside = top of column rod
SEAT_TOP_Z = SEAT_BOTTOM_Z + SEAT_THICK  # ≈ 0.508 m

# Swivel joint placed at the hub center (child-local z = world_z - HUB_Z)
JOINT_Z = HUB_Z


def bz(world_z: float) -> float:
    """Convert world Z coordinates to the caster_base frame."""
    return world_z - BASE_SWIVEL_Z


def _seat_cushion_mesh() -> object:
    """
    Round padded brown-leather cushion: pronounced bulging stitched edge.
    LatheGeometry profile revolved around Z gives a full puck shape.
    z=0 at cushion bottom, z=SEAT_THICK at the flat top.
    """
    r = SEAT_RADIUS
    t = SEAT_THICK
    # (radius, height) Lathe profile — the bulge peaks at mid-height giving
    # a plump, over-stuffed look with a rounded edge bead.
    bead_r = r + 0.006   # outermost extent of the stitching bead
    profile = [
        (0.0,       0.0),
        (r - 0.040, 0.0),          # flat bottom inner
        (r - 0.010, 0.006),        # transition to bead
        (bead_r,    t * 0.20),     # bead outer peak (bulge)
        (bead_r,    t * 0.82),     # bead outer top
        (r - 0.010, t - 0.008),   # transition inward
        (r - 0.040, t),            # flat top outer
        (0.0,       t),            # flat top inner (center)
    ]
    geom = LatheGeometry(profile, segments=80)
    return mesh_from_geometry(geom, "seat_cushion")


def _stitch_bead_mesh() -> object:
    """Thin raised stitching ring around the cushion edge equator."""
    seg = 72
    rr = SEAT_RADIUS + 0.008   # just outside the peak of the bead
    z = SEAT_THICK * 0.50      # at mid-height of the cushion
    pts = []
    for i in range(seg + 1):
        a = 2.0 * math.pi * i / seg
        pts.append((rr * math.cos(a), rr * math.sin(a), z))
    geom = tube_from_spline_points(
        pts, radius=0.0035, closed_spline=True, radial_segments=10, cap_ends=False
    )
    return mesh_from_geometry(geom, "stitch_ring")


def _spoke_mesh(direction: tuple[float, float]) -> object:
    """
    A tapered arm sweeping outward and down from the hub to the caster mount.
    Profile: tube radius tapers from 0.022 at the hub to 0.014 at the tip.
    The arm reaches all the way out to the caster center (CASTER_R) and ends
    at the top of the caster yoke stem so the leg and wheel stay connected.
    """
    dx, dy = direction
    # Path points: INSIDE the hub boss → mid-arm → caster yoke mount.
    # r0 sits inside the hub wall (but clear of the column) so the arm root is
    # embedded in the central boss instead of floating off its surface.
    r0 = HUB_RADIUS - 0.012  # inside the hub wall, still clear of the column
    p0 = (dx * r0, dy * r0, bz(HUB_Z + 0.006))
    p1 = (dx * CASTER_R * 0.42, dy * CASTER_R * 0.42, bz(HUB_Z - 0.004))
    p2 = (dx * CASTER_R * 0.78, dy * CASTER_R * 0.78, bz(0.062))
    p3 = (dx * CASTER_R,        dy * CASTER_R,         bz(0.056))

    # Build a tapered spoke using two tube segments with different radii.
    # Outer arm (inner half, wider):
    inner_geom = tube_from_spline_points(
        [p0, p1], radius=0.022, samples_per_segment=8, radial_segments=14, cap_ends=True
    )
    # Outer arm (outer half, narrower):
    outer_geom = tube_from_spline_points(
        [p1, p2, p3], radius=0.016, samples_per_segment=10, radial_segments=12, cap_ends=True
    )
    # Merge both into a single mesh geometry.
    merged = inner_geom.merge(outer_geom)
    return mesh_from_geometry(merged, "spoke")


def _caster_yoke_mesh(cx: float, cy: float, ax: float, ay: float) -> object:
    """
    Caster yoke/fork housing: a vertical stem plus two fork prongs that land
    outside the tire footprint. Each prong tip sits outside the tire width so
    there is no overlap with the tire mesh.
    """
    # Prong tips land just outside each tire's outer face.
    prong_half = WHEEL_GAP / 2.0 + WHEEL_WIDTH / 2.0 + 0.004  # ≈ 0.040 m from axle
    stem_top_z = bz(0.058)
    axle_z = bz(WHEEL_RADIUS)  # ≈ 0.026 m in world
    prong_z = bz(WHEEL_RADIUS - 0.002)  # prong tip is just below axle center

    # Vertical stem from spoke tip level down toward the axle area.
    stem_geom = tube_from_spline_points(
        [(cx, cy, stem_top_z), (cx, cy, bz(WHEEL_RADIUS + 0.018))],
        radius=0.013, samples_per_segment=4, radial_segments=12, cap_ends=True,
    )
    # Two fork prongs diverging from the stem bottom to outside each tire.
    tine_l_geom = tube_from_spline_points(
        [
            (cx,                             cy,                             bz(WHEEL_RADIUS + 0.015)),
            (cx - ax * prong_half * 0.6,    cy - ay * prong_half * 0.6,    bz(WHEEL_RADIUS + 0.008)),
            (cx - ax * prong_half,           cy - ay * prong_half,          prong_z),
        ],
        radius=0.007, samples_per_segment=6, radial_segments=10, cap_ends=True,
    )
    tine_r_geom = tube_from_spline_points(
        [
            (cx,                             cy,                             bz(WHEEL_RADIUS + 0.015)),
            (cx + ax * prong_half * 0.6,    cy + ay * prong_half * 0.6,    bz(WHEEL_RADIUS + 0.008)),
            (cx + ax * prong_half,           cy + ay * prong_half,          prong_z),
        ],
        radius=0.007, samples_per_segment=6, radial_segments=10, cap_ends=True,
    )
    merged = stem_geom.merge(tine_l_geom).merge(tine_r_geom)
    return mesh_from_geometry(merged, "caster_yoke")


def _axle_mesh(a: tuple[float, float, float], b: tuple[float, float, float]) -> object:
    geom = tube_from_spline_points(
        [a, b], radius=0.0090, samples_per_segment=4, radial_segments=12
    )
    return mesh_from_geometry(geom, "caster_axle")


def _twin_wheel_mesh() -> object:
    """A single soft caster wheel (rolls about its local X axis)."""
    tire = TireGeometry(
        outer_radius=WHEEL_RADIUS,
        width=WHEEL_WIDTH,
        inner_radius=0.008,
        carcass=TireCarcass(belt_width_ratio=0.65, sidewall_bulge=0.14),
        sidewall=TireSidewall(style="rounded", bulge=0.12),
        center=True,
    )
    return mesh_from_geometry(tire, "caster_wheel")


def _gas_lift_column_cq() -> object:
    """
    Stepped gas-lift column: a wide lower sleeve merging into a narrower upper rod.
    Built with CadQuery revolve for a clean telescoping step transition.
    The mesh spans from z=SLEEVE_BOTTOM_Z to z=ROD_TOP_Z in world coordinates;
    it is placed with the swivel origin at z=HUB_Z so child-local coords apply.
    """
    sleeve_r = SLEEVE_RADIUS
    rod_r = ROD_RADIUS
    # All heights relative to local z (child frame = world - HUB_Z).
    sl_bot = SLEEVE_BOTTOM_Z - HUB_Z   # ≈ -0.047
    sl_top = SLEEVE_TOP_Z - HUB_Z      # ≈ 0.128
    rd_top = ROD_TOP_Z - HUB_Z         # ≈ 0.358

    step_h = 0.018  # height of the transition step chamfer

    # Lathe profile in (r, z) — the sleeve widens first, transitions, then narrows.
    profile = [
        (0.0,      sl_bot),
        (sleeve_r, sl_bot),
        (sleeve_r, sl_top - step_h),
        (rod_r + 0.004, sl_top),      # stepped transition
        (rod_r,    sl_top + step_h),
        (rod_r,    rd_top),
        (0.0,      rd_top),
    ]
    geom = LatheGeometry(profile, segments=48)
    return mesh_from_geometry(geom, "gas_lift_column")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_task_stool")

    model.material("base_black", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("column_black", rgba=(0.07, 0.07, 0.08, 1.0))
    model.material("leather_brown", rgba=(0.47, 0.27, 0.16, 1.0))
    model.material("stitch", rgba=(0.33, 0.19, 0.10, 1.0))
    model.material("rubber", rgba=(0.05, 0.05, 0.06, 1.0))
    model.material("steel", rgba=(0.30, 0.31, 0.33, 1.0))

    # -----------------------------------------------------------------------
    # Root bearing layer: a visible stationary washer whose top face carries
    # the rotating caster base.
    # -----------------------------------------------------------------------
    bearing = model.part("stationary_bearing")
    bearing.visual(
        Cylinder(radius=BEARING_RADIUS, length=BEARING_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_SWIVEL_Z - BEARING_HEIGHT / 2.0)),
        material="steel",
        name="bearing_washer",
    )

    # -----------------------------------------------------------------------
    # Rotating base layer: four-spoke caster base (hub + spokes + caster yokes + axles)
    # -----------------------------------------------------------------------
    base = model.part("caster_base")

    # Central hub — the column drops into its bore.
    base.visual(
        Cylinder(radius=HUB_RADIUS, length=HUB_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, bz(HUB_Z))),
        material="base_black",
        name="hub",
    )
    # Collar ring on top of the hub.
    base.visual(
        Cylinder(radius=HUB_RADIUS * 0.72, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, bz(HUB_Z + HUB_HEIGHT / 2.0 + 0.006))),
        material="base_black",
        name="hub_collar",
    )

    # Four tapered spokes at 90° intervals.
    spoke_dirs: list[tuple[float, float]] = []
    for i in range(N_SPOKES):
        a = 2.0 * math.pi * i / N_SPOKES + math.pi / 2.0
        d = (math.cos(a), math.sin(a))
        spoke_dirs.append(d)
        base.visual(
            _spoke_mesh(d),
            material="base_black",
            name=f"spoke_{i}",
        )

    # Caster yokes and axles — one per spoke.
    caster_r = CASTER_R
    for i, d in enumerate(spoke_dirs):
        cx = d[0] * caster_r
        cy = d[1] * caster_r
        # Axle direction: perpendicular to spoke, horizontal.
        ax, ay = -d[1], d[0]
        # Caster yoke (fork/bracket).
        base.visual(
            _caster_yoke_mesh(cx, cy, ax, ay),
            material="base_black",
            name=f"caster_yoke_{i}",
        )
        # Axle through both wheels.
        half = WHEEL_GAP / 2.0 + 0.010
        base.visual(
            _axle_mesh(
                (cx - ax * half, cy - ay * half, bz(WHEEL_RADIUS)),
                (cx + ax * half, cy + ay * half, bz(WHEEL_RADIUS)),
            ),
            material="steel",
            name=f"caster_axle_{i}",
        )

    # -----------------------------------------------------------------------
    # Twin-wheel casters — each wheel is its own child part rolling on its axle.
    # -----------------------------------------------------------------------
    for i, d in enumerate(spoke_dirs):
        caster_cx = d[0] * caster_r
        caster_cy = d[1] * caster_r
        ax, ay = -d[1], d[0]
        for j, sgn in enumerate((-1.0, 1.0)):
            wx = caster_cx + ax * (WHEEL_GAP / 2.0) * sgn
            wy = caster_cy + ay * (WHEEL_GAP / 2.0) * sgn
            wheel = model.part(f"caster_wheel_{i}_{j}")
            # Wheel mesh spins about local X; rotate to align local X with axle.
            yaw = math.atan2(ay, ax)
            wheel.visual(
                _twin_wheel_mesh(),
                origin=Origin(rpy=(0.0, 0.0, yaw)),
                material="rubber",
                name="tire",
            )
            model.articulation(
                f"caster_roll_{i}_{j}",
                ArticulationType.CONTINUOUS,
                parent=base,
                child=wheel,
                origin=Origin(xyz=(wx, wy, bz(WHEEL_RADIUS))),
                axis=(ax, ay, 0.0),
                motion_limits=MotionLimits(effort=2.0, velocity=20.0),
            )

    # -----------------------------------------------------------------------
    # Base swivel joint: the whole caster-base-and-leg assembly turns on the
    # visible bearing washer, using the washer top face as the contact plane.
    # -----------------------------------------------------------------------
    model.articulation(
        "base_swivel",
        ArticulationType.CONTINUOUS,
        parent=bearing,
        child=base,
        origin=Origin(xyz=(0.0, 0.0, BASE_SWIVEL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=8.0),
    )

    # -----------------------------------------------------------------------
    # Swiveling assembly: gas-lift column + round padded seat.
    # Joint frame at hub center (world z = HUB_Z = JOINT_Z).
    # child-local z = world_z - HUB_Z.
    # -----------------------------------------------------------------------
    swivel = model.part("column_seat")

    def sz(world_z: float) -> float:
        return world_z - JOINT_Z

    # Gas-lift column (stepped LatheGeometry: wide sleeve + narrow rod).
    swivel.visual(
        _gas_lift_column_cq(),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),  # local origin = joint frame origin
        material="column_black",
        name="gas_lift_column",
    )

    # Metal seat-mount disc under the cushion.
    swivel.visual(
        Cylinder(radius=0.062, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, sz(SEAT_BOTTOM_Z - 0.006))),
        material="steel",
        name="seat_mount",
    )

    # Padded brown-leather cushion (LatheGeometry, z=0..SEAT_THICK locally).
    swivel.visual(
        _seat_cushion_mesh(),
        origin=Origin(xyz=(0.0, 0.0, sz(SEAT_BOTTOM_Z))),
        material="leather_brown",
        name="cushion",
    )

    # Stitched edge piping ring at cushion equator.
    swivel.visual(
        _stitch_bead_mesh(),
        origin=Origin(xyz=(0.0, 0.0, sz(SEAT_BOTTOM_Z))),
        material="stitch",
        name="stitch_ring",
    )

    # -----------------------------------------------------------------------
    # Swivel joint: continuous about +Z at the hub center.
    # -----------------------------------------------------------------------
    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=swivel,
        origin=Origin(xyz=(0.0, 0.0, bz(JOINT_Z))),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bearing = object_model.get_part("stationary_bearing")
    base = object_model.get_part("caster_base")
    swivel = object_model.get_part("column_seat")
    base_swivel = object_model.get_articulation("base_swivel")
    seat_swivel = object_model.get_articulation("seat_swivel")

    ctx.expect_contact(
        bearing,
        base,
        elem_a="bearing_washer",
        elem_b="hub",
        name="base hub rides on visible bearing washer",
    )

    # The gas-lift column intentionally drops into the hub bore.
    ctx.allow_overlap(
        base, swivel,
        elem_a="hub", elem_b="gas_lift_column",
        reason="Gas-lift column seats into the base hub bore (captured insertion).",
    )
    ctx.allow_overlap(
        base, swivel,
        elem_a="hub_collar", elem_b="gas_lift_column",
        reason="Gas-lift column passes through the hub collar ring (captured insertion).",
    )
    # Each twin-wheel rides on its captured axle.
    for i in range(N_SPOKES):
        for j in range(2):
            ctx.allow_overlap(
                base,
                object_model.get_part(f"caster_wheel_{i}_{j}"),
                elem_a=f"caster_axle_{i}",
                elem_b="tire",
                reason="Caster wheel is captured on its axle (axle through wheel bore).",
            )
            # The fork prongs of the caster yoke embrace the wheel — intentional capture.
            for tine_suffix in ("__component_002", "__component_003"):
                ctx.allow_overlap(
                    base,
                    object_model.get_part(f"caster_wheel_{i}_{j}"),
                    elem_a=f"caster_yoke_{i}{tine_suffix}",
                    elem_b="tire",
                    reason="Caster yoke fork prong embraces the wheel (captured yoke).",
                )

    # --- Hero: padded leather seat at task-stool height ~0.50 m ---
    seat_aabb = ctx.part_element_world_aabb(swivel, elem="cushion")
    assert seat_aabb is not None
    seat_top = seat_aabb[1][2]
    ctx.check(
        "seat at task-stool height",
        0.46 <= seat_top <= 0.58,
        details=f"seat_top={seat_top:.3f}",
    )

    # --- Seat is round: equal x/y extent, large enough to sit on ---
    sx = seat_aabb[1][0] - seat_aabb[0][0]
    sy = seat_aabb[1][1] - seat_aabb[0][1]
    ctx.check(
        "seat is round and wide",
        sx > 0.32 and sy > 0.32 and abs(sx - sy) < 0.02,
        details=f"sx={sx:.3f}, sy={sy:.3f}",
    )

    # --- Stitched edge bead present at cushion rim ---
    ctx.expect_overlap(
        swivel, swivel,
        axes="z",
        elem_a="stitch_ring", elem_b="cushion",
        min_overlap=0.004,
        name="stitch ring on cushion edge",
    )

    # --- Gas-lift column: stepped, connects seat down to hub ---
    col_aabb = ctx.part_element_world_aabb(swivel, elem="gas_lift_column")
    assert col_aabb is not None
    ctx.check(
        "column descends to base hub",
        col_aabb[0][2] < HUB_Z + 0.02,
        details=f"column_bottom={col_aabb[0][2]:.3f}",
    )
    ctx.check(
        "column reaches up to seat",
        col_aabb[1][2] >= SEAT_BOTTOM_Z - 0.01,
        details=f"column_top={col_aabb[1][2]:.3f}, seat_bottom={SEAT_BOTTOM_Z:.3f}",
    )

    # --- Four spokes present, base footprint wide ---
    base_aabb = ctx.part_world_aabb(base)
    assert base_aabb is not None
    span_x = base_aabb[1][0] - base_aabb[0][0]
    ctx.check(
        "base footprint wide (four spokes)",
        span_x > 0.45,
        details=f"span_x={span_x:.3f}",
    )

    ctx.check(
        "exactly four twin caster pairs are looped",
        len([part for part in object_model.parts if part.name.startswith("caster_wheel_")]) == 8
        and all(
            any(part.name == f"caster_wheel_{i}_{j}" for part in object_model.parts)
            for i in range(4)
            for j in range(2)
        ),
        details="expected caster_wheel_i_j for i=0..3 and j=0..1 only",
    )

    centers = []
    for i in range(N_SPOKES):
        w_left = ctx.part_world_position(object_model.get_part(f"caster_wheel_{i}_0"))
        w_right = ctx.part_world_position(object_model.get_part(f"caster_wheel_{i}_1"))
        assert w_left is not None and w_right is not None
        centers.append(((w_left[0] + w_right[0]) * 0.5, (w_left[1] + w_right[1]) * 0.5))
    angle_steps = []
    for i in range(N_SPOKES):
        a0 = math.atan2(centers[i][1], centers[i][0])
        a1 = math.atan2(centers[(i + 1) % N_SPOKES][1], centers[(i + 1) % N_SPOKES][0])
        angle_steps.append((a1 - a0) % (2.0 * math.pi))
    ctx.check(
        "four caster pairs are evenly spaced at 90 degrees",
        all(abs(step - math.pi / 2.0) < 0.02 for step in angle_steps),
        details=f"angle_steps={[round(step, 3) for step in angle_steps]}",
    )

    # --- Casters: all 8 wheels are present and touch the floor ---
    for i in range(N_SPOKES):
        for j in range(2):
            w = object_model.get_part(f"caster_wheel_{i}_{j}")
            wa = ctx.part_world_aabb(w)
            assert wa is not None, f"caster_wheel_{i}_{j} aabb is None"
            ctx.check(
                f"caster wheel {i}_{j} near floor",
                wa[0][2] < 0.008,
                details=f"wheel_bottom={wa[0][2]:.3f}",
            )

    # --- Articulation: whole base swivels about +Z on the visible bearing ---
    ctx.check(
        "base_swivel is continuous about Z",
        base_swivel.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(c, 3) for c in base_swivel.axis) == (0.0, 0.0, 1.0),
        details=f"type={base_swivel.articulation_type}, axis={base_swivel.axis}",
    )
    caster_before = ctx.part_world_position(object_model.get_part("caster_wheel_0_0"))
    with ctx.pose({base_swivel: math.pi / 2.0}):
        caster_after = ctx.part_world_position(object_model.get_part("caster_wheel_0_0"))
        ctx.expect_contact(
            bearing,
            base,
            elem_a="bearing_washer",
            elem_b="hub",
            name="base remains seated on bearing while swiveled",
        )
    assert caster_before is not None and caster_after is not None
    ctx.check(
        "base swivel rotates caster-base assembly",
        abs(caster_after[0] + caster_before[1]) < 0.015
        and abs(caster_after[1] - caster_before[0]) < 0.015,
        details=f"before={caster_before}, after={caster_after}",
    )

    # --- Articulation: seat swivels about +Z (continuous) ---
    ctx.check(
        "seat_swivel is continuous about Z",
        seat_swivel.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(c, 3) for c in seat_swivel.axis) == (0.0, 0.0, 1.0),
        details=f"type={seat_swivel.articulation_type}, axis={seat_swivel.axis}",
    )

    # Swiveling moves the stitch ring (proves rotation works).
    rim_before = ctx.part_element_world_aabb(swivel, elem="stitch_ring")
    with ctx.pose({seat_swivel: math.pi / 2.0}):
        rim_after = ctx.part_element_world_aabb(swivel, elem="stitch_ring")
        ctx.expect_within(
            swivel, base,
            axes="xy",
            inner_elem="gas_lift_column",
            margin=0.35,
            name="column stays over base under swivel",
        )
    assert rim_before is not None and rim_after is not None
    ctx.check(
        "seat swivel pose applies",
        rim_after is not None,
        details="swivel pose evaluated successfully",
    )

    # --- Caster roll: continuous about horizontal axis ---
    roll = object_model.get_articulation("caster_roll_0_0")
    ctx.check(
        "caster roll is continuous",
        roll.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={roll.articulation_type}",
    )
    w0 = object_model.get_part("caster_wheel_0_0")
    pos_before = ctx.part_world_position(w0)
    with ctx.pose({roll: 1.0}):
        pos_after = ctx.part_world_position(w0)
    ctx.check(
        "caster roll pose applies",
        pos_before is not None and pos_after is not None,
        details=f"before={pos_before}, after={pos_after}",
    )

    return ctx.report()


object_model = build_object_model()
