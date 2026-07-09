from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    LatheGeometry,
    LoftSection,
    MotionLimits,
    Origin,
    SectionLoftSpec,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    section_loft,
    sweep_profile_along_spline,
    rounded_rect_profile,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Swivel bar stool — tan leather seat with curved wrap-around low backrest,
# slim brushed-gold pedestal, round footrest ring on 4 diagonal struts,
# and a three-leg tripod base radiating from the central column.
# ---------------------------------------------------------------------------
# Real-world scale: seat top ≈ 0.70 m. +Z is up; base sits on floor at z=0.
# Root (fixed):  gold tripod base + gold pedestal + gold footrest ring + struts.
# Child (swivels about Z): padded tan leather seat + tub/barrel backrest.
# ---------------------------------------------------------------------------

# Tripod base
TRIPOD_LEG_COUNT = 3
TRIPOD_OUTER_R = 0.19       # how far each leg extends from center
LEG_PROFILE_W = 0.028       # leg cross-section width (lateral)
LEG_PROFILE_H = 0.016       # leg cross-section height (vertical)
LEG_PROFILE_R = 0.006       # corner radius of the rounded-rect profile
FOOT_PAD_RADIUS = 0.014     # small disc glide at each leg tip
FOOT_PAD_THICK = 0.005

PEDESTAL_RADIUS = 0.028
PEDESTAL_BOTTOM_Z = 0.040   # column ends at the leg junction, above the floor
PEDESTAL_TOP_Z = 0.62

FOOTREST_Z = 0.23
FOOTREST_RING_RADIUS = 0.145
FOOTREST_TUBE_R = 0.009

# Seat cushion: square-ish, padded
SEAT_HALF_W = 0.205   # half-width (X)
SEAT_HALF_D = 0.200   # half-depth (Y)
SEAT_THICK = 0.080
SEAT_BOTTOM_Z = PEDESTAL_TOP_Z + 0.015  # seat cushion bottom just above joint
SEAT_TOP_Z = SEAT_BOTTOM_Z + SEAT_THICK  # ≈ 0.715 m

# Backrest: tub/barrel form wrapping around back and sides
BACK_WRAP_R = 0.215   # centerline radius of the barrel arc
BACK_THICK = 0.058    # radial thickness of the upholstered band
BACK_HEIGHT = 0.235   # height of the backrest band
BACK_INNER_R = BACK_WRAP_R - BACK_THICK / 2.0
BACK_OUTER_R = BACK_WRAP_R + BACK_THICK / 2.0


def _tripod_leg_mesh(angle: float) -> object:
    """
    One tripod leg radiating from the central column.
    Built as a sweep of a rounded-rect profile along a curved path that
    descends from the pedestal surface to the floor.
    """
    cos_a, sin_a = math.cos(angle), math.sin(angle)

    inner_r = PEDESTAL_RADIUS + 0.003  # start just outside the pedestal
    mid_r = (inner_r + TRIPOD_OUTER_R) * 0.50

    # Path: starts at pedestal surface near column top-of-leg zone,
    # descends smoothly to near-floor at the outer tip.
    pts = [
        (cos_a * inner_r, sin_a * inner_r, 0.058),
        (cos_a * (inner_r + 0.030), sin_a * (inner_r + 0.030), 0.048),
        (cos_a * mid_r, sin_a * mid_r, 0.030),
        (cos_a * TRIPOD_OUTER_R, sin_a * TRIPOD_OUTER_R, LEG_PROFILE_H * 0.5 + 0.001),
    ]

    profile = rounded_rect_profile(LEG_PROFILE_W, LEG_PROFILE_H, radius=LEG_PROFILE_R)

    geom = sweep_profile_along_spline(
        pts,
        profile=profile,
        samples_per_segment=6,
        cap_profile=True,
        up_hint=(0.0, 0.0, 1.0),
    )
    return mesh_from_geometry(geom, "tripod_leg")


def _tripod_hub_mesh() -> object:
    """
    Small cylindrical hub collar where the three legs meet the pedestal.
    Sits around the lower pedestal, just above the floor.
    """
    hub_r = PEDESTAL_RADIUS + 0.010
    hub_h = 0.035
    # Lathe a simple rounded profile
    profile = [
        (PEDESTAL_RADIUS - 0.001, 0.0),
        (hub_r, 0.003),
        (hub_r, hub_h - 0.003),
        (PEDESTAL_RADIUS + 0.004, hub_h),
        (PEDESTAL_RADIUS - 0.001, hub_h),
    ]
    geom = LatheGeometry(profile, segments=48)
    return mesh_from_geometry(geom, "tripod_hub")


def _footrest_ring_mesh() -> object:
    """A gold torus-ring at footrest height."""
    seg = 80
    pts = []
    for i in range(seg + 1):
        a = 2.0 * math.pi * i / seg
        pts.append(
            (FOOTREST_RING_RADIUS * math.cos(a), FOOTREST_RING_RADIUS * math.sin(a), 0.0)
        )
    geom = tube_from_spline_points(
        pts,
        radius=FOOTREST_TUBE_R,
        closed_spline=True,
        radial_segments=16,
        cap_ends=False,
    )
    return mesh_from_geometry(geom, "footrest_ring")


def _footrest_strut_mesh(angle: float) -> object:
    """Diagonal strut from the pedestal to the footrest ring."""
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    inner_r = PEDESTAL_RADIUS - 0.002
    outer_r = FOOTREST_RING_RADIUS - 0.004
    p0 = (cos_a * inner_r, sin_a * inner_r, FOOTREST_Z - 0.005)
    p1 = (cos_a * outer_r, sin_a * outer_r, FOOTREST_Z + 0.005)
    geom = tube_from_spline_points(
        [p0, p1], radius=0.007, samples_per_segment=4, radial_segments=10, cap_ends=True
    )
    return mesh_from_geometry(geom, "footrest_strut")


def _seat_cushion_mesh() -> object:
    """
    Padded tan leather seat cushion: rounded-square slab, dome top.
    CadQuery box with vertical-edge and top-face fillets.
    Mesh spans z=0 to SEAT_THICK locally; origin placed at SEAT_BOTTOM_Z.
    """
    w = SEAT_HALF_W * 2.0
    d = SEAT_HALF_D * 2.0
    t = SEAT_THICK
    result = (
        cq.Workplane("XY")
        .box(w, d, t, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.045)
        .edges(">Z")
        .fillet(t * 0.25)
    )
    return mesh_from_cadquery(result, "seat_cushion")


def _backrest_mesh() -> object:
    """
    Curved wrap-around tub/barrel low backrest in tan leather.
    Wraps ~220° around the back and sides of the seat (opens toward +Y front).
    Built as a sweep of a rounded-rect cross-section (radial_thick × height)
    along the arc path, using sweep_profile_along_spline.

    The mesh is built in local space with bottom at z=0, top at z=BACK_HEIGHT.
    The origin placement raises it to sit at the seat top.
    """
    half_deg = 110.0
    half_rad = math.radians(half_deg)
    n = 36

    # Arc path: sweeps from +110° to -110° (opening toward +Y front).
    # Center the arc on the back side of the stool (-Y direction).
    pts = []
    for i in range(n + 1):
        frac = i / n
        # Start at +110°, sweep CCW to -110° (opening at front, +Y).
        a = half_rad - 2.0 * half_rad * frac
        # Arc centered slightly behind center so it hugs the seat rear.
        cx, cy = 0.0, -BACK_WRAP_R * 0.05
        x = cx + BACK_WRAP_R * math.sin(a)
        y = cy - BACK_WRAP_R * math.cos(a)
        # The path is at mid-height of the backrest (z=0 locally).
        pts.append((x, y, BACK_HEIGHT / 2.0))

    # Cross-section profile: rounded rectangle (radial_thick × height).
    profile = rounded_rect_profile(BACK_THICK, BACK_HEIGHT, radius=0.018)

    geom = sweep_profile_along_spline(
        pts,
        profile=profile,
        samples_per_segment=4,
        cap_profile=True,
        up_hint=(0.0, 0.0, 1.0),
    )
    return mesh_from_geometry(geom, "backrest")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swivel_bar_stool")

    model.material("gold", rgba=(0.85, 0.70, 0.30, 1.0))
    model.material("gold_dark", rgba=(0.68, 0.54, 0.22, 1.0))
    model.material("leather_tan", rgba=(0.73, 0.51, 0.36, 1.0))
    model.material("steel", rgba=(0.30, 0.31, 0.33, 1.0))

    # -----------------------------------------------------------------------
    # Root part: fixed tripod base + pedestal + footrest ring + 4 struts
    # -----------------------------------------------------------------------
    frame = model.part("base_pedestal")

    # Three tripod legs radiating at 120° from the central column.
    for i in range(TRIPOD_LEG_COUNT):
        angle = 2.0 * math.pi * i / TRIPOD_LEG_COUNT
        frame.visual(
            _tripod_leg_mesh(angle),
            material="gold",
            name=f"tripod_leg_{i}",
        )
        # Small disc foot-pad glide at each leg tip.
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        frame.visual(
            Cylinder(radius=FOOT_PAD_RADIUS, length=FOOT_PAD_THICK),
            origin=Origin(xyz=(
                cos_a * TRIPOD_OUTER_R,
                sin_a * TRIPOD_OUTER_R,
                FOOT_PAD_THICK / 2.0,
            )),
            material="gold_dark",
            name=f"foot_pad_{i}",
        )

    # Hub collar where the legs meet the pedestal.
    frame.visual(
        _tripod_hub_mesh(),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material="gold",
        name="tripod_hub",
    )

    # Slim cylindrical pedestal — rises from the leg junction to the seat.
    ped_len = PEDESTAL_TOP_Z - PEDESTAL_BOTTOM_Z
    ped_mid = (PEDESTAL_TOP_Z + PEDESTAL_BOTTOM_Z) / 2.0
    frame.visual(
        Cylinder(radius=PEDESTAL_RADIUS, length=ped_len),
        origin=Origin(xyz=(0.0, 0.0, ped_mid)),
        material="gold",
        name="pedestal",
    )

    # Footrest ring — gold torus at footrest height.
    frame.visual(
        _footrest_ring_mesh(),
        origin=Origin(xyz=(0.0, 0.0, FOOTREST_Z)),
        material="gold",
        name="footrest_ring",
    )

    # Four diagonal struts (at 45°, 135°, 225°, 315°) connecting pedestal to ring.
    for i in range(4):
        angle = math.pi / 4.0 + math.pi / 2.0 * i
        frame.visual(
            _footrest_strut_mesh(angle),
            material="gold_dark",
            name=f"footrest_strut_{i}",
        )

    # -----------------------------------------------------------------------
    # Child part: swiveling seat cushion + wrap-around barrel backrest.
    # All visuals are in child-local coords; joint frame at world z=PEDESTAL_TOP_Z.
    # child_local_z = world_z - PEDESTAL_TOP_Z
    # -----------------------------------------------------------------------
    seat = model.part("seat_backrest")

    def sz(world_z: float) -> float:
        return world_z - PEDESTAL_TOP_Z

    # Gold swivel collar caps the pedestal top (slightly above joint frame).
    seat.visual(
        Cylinder(radius=0.048, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, sz(PEDESTAL_TOP_Z + 0.003))),
        material="gold",
        name="swivel_collar",
    )

    # Thin steel seat-mount plate under the cushion.
    seat.visual(
        Cylinder(radius=0.105, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, sz(SEAT_BOTTOM_Z - 0.006))),
        material="steel",
        name="seat_plate",
    )

    # Padded tan leather seat cushion (CadQuery rounded box, z=0..SEAT_THICK locally).
    seat.visual(
        _seat_cushion_mesh(),
        origin=Origin(xyz=(0.0, 0.0, sz(SEAT_BOTTOM_Z))),
        material="leather_tan",
        name="cushion",
    )

    # Curved wrap-around low backrest.
    # The sweep profile spans z=0..BACK_HEIGHT locally. Place so band bottom
    # starts slightly below the seat top for a natural overlap/connection.
    back_local_z = sz(SEAT_TOP_Z - BACK_HEIGHT / 2.0)  # center of band at seat top
    seat.visual(
        _backrest_mesh(),
        origin=Origin(xyz=(0.0, 0.0, back_local_z)),
        material="leather_tan",
        name="backrest",
    )

    # -----------------------------------------------------------------------
    # Articulation: continuous swivel about +Z at the pedestal top.
    # -----------------------------------------------------------------------
    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, PEDESTAL_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("base_pedestal")
    seat = object_model.get_part("seat_backrest")
    swivel = object_model.get_articulation("seat_swivel")

    # Allow intentional overlaps at the structural coupling.
    ctx.allow_overlap(
        frame, seat,
        elem_a="pedestal", elem_b="swivel_collar",
        reason="Swivel collar captures over the pedestal top.",
    )
    ctx.allow_overlap(
        frame, seat,
        elem_a="pedestal", elem_b="seat_plate",
        reason="Seat mount plate seats onto the pedestal top.",
    )

    # --- Seat sits at bar-stool height (~0.70 m) ---
    cushion_aabb = ctx.part_element_world_aabb(seat, elem="cushion")
    assert cushion_aabb is not None
    seat_top = cushion_aabb[1][2]
    ctx.check(
        "seat at bar height",
        0.66 <= seat_top <= 0.80,
        details=f"seat_top={seat_top:.3f}",
    )

    # --- Cushion is roughly square and padded ---
    sx = cushion_aabb[1][0] - cushion_aabb[0][0]
    sy = cushion_aabb[1][1] - cushion_aabb[0][1]
    ctx.check(
        "seat cushion is square-ish and wide",
        sx > 0.36 and sy > 0.36 and abs(sx - sy) < 0.04,
        details=f"sx={sx:.3f}, sy={sy:.3f}",
    )

    # --- Wrap-around backrest: tall, wide, rises above the seat ---
    back_aabb = ctx.part_element_world_aabb(seat, elem="backrest")
    assert back_aabb is not None
    back_top = back_aabb[1][2]
    back_w = back_aabb[1][0] - back_aabb[0][0]
    ctx.check(
        "backrest rises above seat (low back tub form)",
        back_top > seat_top + 0.06 and back_top < seat_top + 0.32,
        details=f"back_top={back_top:.3f}, seat_top={seat_top:.3f}",
    )
    ctx.check(
        "backrest is wide (wrap-around)",
        back_w > 0.28,
        details=f"back_width={back_w:.3f}",
    )
    ctx.check(
        "backrest bottom near seat level",
        back_aabb[0][2] <= seat_top + 0.02,
        details=f"back_bottom={back_aabb[0][2]:.3f}, seat_top={seat_top:.3f}",
    )

    # --- Pedestal is slim ---
    ped_aabb = ctx.part_element_world_aabb(frame, elem="pedestal")
    assert ped_aabb is not None
    ped_w = ped_aabb[1][0] - ped_aabb[0][0]
    ctx.check(
        "pedestal is slim",
        ped_w < 0.07,
        details=f"pedestal_width={ped_w:.3f}",
    )

    # --- Footrest ring: at mid-height, large diameter ---
    ring_aabb = ctx.part_element_world_aabb(frame, elem="footrest_ring")
    assert ring_aabb is not None
    ring_w = ring_aabb[1][0] - ring_aabb[0][0]
    ring_z = (ring_aabb[0][2] + ring_aabb[1][2]) / 2.0
    ctx.check(
        "footrest ring is wide and at mid-height",
        ring_w > 0.26 and 0.12 < ring_z < 0.38,
        details=f"ring_width={ring_w:.3f}, ring_z={ring_z:.3f}",
    )

    # --- Four struts connect pedestal to ring ---
    for i in range(4):
        strut_aabb = ctx.part_element_world_aabb(frame, elem=f"footrest_strut_{i}")
        ctx.check(
            f"footrest strut {i} present",
            strut_aabb is not None,
            details=f"strut_{i} aabb={strut_aabb}",
        )

    # --- Tripod base: three legs radiating at 120° from the column ---
    leg_aabbs = []
    for i in range(TRIPOD_LEG_COUNT):
        leg_aabb = ctx.part_element_world_aabb(frame, elem=f"tripod_leg_{i}")
        ctx.check(
            f"tripod_leg_{i} present",
            leg_aabb is not None,
            details=f"tripod_leg_{i} aabb={leg_aabb}",
        )
        if leg_aabb is not None:
            leg_aabbs.append(leg_aabb)

    # Legs must extend near the floor (z_min close to 0).
    if leg_aabbs:
        floor_contacts = [aabb[0][2] < 0.020 for aabb in leg_aabbs]
        ctx.check(
            "all tripod legs reach near the floor",
            all(floor_contacts),
            details=f"leg_z_mins={[f'{aabb[0][2]:.4f}' for aabb in leg_aabbs]}",
        )

    # Legs must extend outward from the column (wider than pedestal).
    if leg_aabbs:
        spans = [aabb[1][0] - aabb[0][0] for aabb in leg_aabbs] + \
                [aabb[1][1] - aabb[0][1] for aabb in leg_aabbs]
        ctx.check(
            "tripod legs extend well outward from column",
            max(spans) > 0.14,
            details=f"max_leg_span={max(spans):.3f}",
        )

    # Check that legs are roughly at 120° intervals (not bunched on one side).
    if len(leg_aabbs) == 3:
        centers = [
            ((aabb[0][0] + aabb[1][0]) / 2.0, (aabb[0][1] + aabb[1][1]) / 2.0)
            for aabb in leg_aabbs
        ]
        angles = [math.atan2(cy, cx) for cx, cy in centers]
        # Sort and check gaps are roughly 120° ± 30°
        angles_sorted = sorted(angles)
        gaps = []
        for j in range(3):
            gap = (angles_sorted[(j + 1) % 3] - angles_sorted[j]) % (2.0 * math.pi)
            gaps.append(gap)
        ctx.check(
            "tripod legs are at ~120° intervals",
            all(abs(g - 2.0 * math.pi / 3.0) < 0.6 for g in gaps),
            details=f"gaps_rad={[f'{g:.2f}' for g in gaps]}",
        )

    # Foot pads at each leg tip.
    for i in range(TRIPOD_LEG_COUNT):
        pad_aabb = ctx.part_element_world_aabb(frame, elem=f"foot_pad_{i}")
        ctx.check(
            f"foot_pad_{i} present at floor",
            pad_aabb is not None and pad_aabb[0][2] < 0.005,
            details=f"pad_{i} aabb={pad_aabb}",
        )

    # Hub collar at leg junction.
    hub_aabb = ctx.part_element_world_aabb(frame, elem="tripod_hub")
    ctx.check(
        "tripod hub collar present",
        hub_aabb is not None and hub_aabb[0][2] < 0.06,
        details=f"hub aabb={hub_aabb}",
    )

    # --- Swivel joint: continuous about Z ---
    ctx.check(
        "seat_swivel is continuous about Z",
        swivel.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(c, 3) for c in swivel.axis) == (0.0, 0.0, 1.0),
        details=f"type={swivel.articulation_type}, axis={swivel.axis}",
    )

    # Rotating the swivel must move the backrest but not the ring.
    back_before = ctx.part_element_world_aabb(seat, elem="backrest")
    with ctx.pose({swivel: math.pi / 2.0}):
        back_after = ctx.part_element_world_aabb(seat, elem="backrest")
        ring_after = ctx.part_element_world_aabb(frame, elem="footrest_ring")
    assert back_before is not None and back_after is not None and ring_after is not None
    moved = (
        abs(back_after[0][1] - back_before[0][1]) > 0.03
        or abs(back_after[0][0] - back_before[0][0]) > 0.03
    )
    ctx.check(
        "swiveling moves the backrest (seat rotates)",
        moved,
        details=f"before_min={back_before[0]}, after_min={back_after[0]}",
    )
    ctx.check(
        "footrest ring stays fixed under swivel",
        abs(ring_after[0][0] - ring_aabb[0][0]) < 1e-5,
        details="ring is on the fixed root frame",
    )

    return ctx.report()


object_model = build_object_model()
