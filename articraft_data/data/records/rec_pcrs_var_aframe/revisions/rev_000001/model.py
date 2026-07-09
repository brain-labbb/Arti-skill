from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mesh,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Modern circular ring swing hanging from an A-frame steel support.
#
# World frame: Z up, ground at z = 0, X spans the ~3.0 m frame width,
# Y is the swing (fore-aft) direction.
#
# A-frame layout (meters):
# - apex_x:     +-1.50, two A-frame pairs
# - apex_z:     2.60 (leg convergence point / ridge beam centre)
# - leg spread: +-0.75 in Y (feet splay fore-aft from each apex)
# - legs:       0.04 radius round tube, ~2.70 m long
# - ridge beam: 3.0 x 0.12 x 0.10 box at z 2.55 .. 2.65
# - braces:     two horizontal tubes per A at z = 0.80 and 1.70
# - swing pivot: (0, 0, 2.55), revolute about +X, -45 .. +45 deg
# - chain/spin/ring: identical to the pergola parent
# ---------------------------------------------------------------------------

# A-frame dimensions
APEX_X = 1.50
APEX_Z = 2.60
LEG_SPREAD = 0.75
LEG_RADIUS = 0.04
RIDGE_W = 0.12
RIDGE_H = 0.10
RIDGE_Z0 = APEX_Z - RIDGE_H / 2.0  # 2.55
RIDGE_Z1 = APEX_Z + RIDGE_H / 2.0  # 2.65

LEG_LENGTH = math.sqrt(LEG_SPREAD ** 2 + APEX_Z ** 2)
LEG_ANGLE = math.atan2(LEG_SPREAD, APEX_Z)

BRACE_HEIGHTS = (0.80, 1.70)
BRACE_RADIUS = 0.025

# Chain layout (chain-local frame, origin at the swing pivot on the beam underside)
STEM_LEN = 0.09
EYE_C = -0.078
LINK_R = 0.028
LINK_TUBE = 0.0065
LINK_STRETCH = 1.45
LINK_CENTERS = (-0.144, -0.234, -0.324, -0.415)
HOOK_EYE_C = -0.481
HOOK_ARC_C = -0.536
HOOK_ARC_R = 0.024
SPIN_Z = -0.576

# Ring layout (ring-local frame, origin at the spin joint / eyelet center)
EYELET_R = 0.030
EYELET_TUBE = 0.007
CLEVIS_C = -0.051
HOOP_R = 0.725
HOOP_TUBE = 0.025
HOOP_Y = 0.0375
RING_C = -0.815
PLANK_RC = 0.693
PLANK_COUNT = 22
PLANK_THETA0 = math.radians(197.0)
PLANK_THETA1 = math.radians(343.0)


def _oval_link_mesh(plane: str, mesh_name: str) -> Mesh:
    """Oval chain link, long axis along Z, ring plane 'yz' or 'xz'."""
    geom = TorusGeometry(LINK_R, LINK_TUBE, radial_segments=12, tubular_segments=36)
    geom.scale(LINK_STRETCH, 1.0, 1.0)
    geom.rotate_y(math.pi / 2.0)
    if plane == "xz":
        geom.rotate_z(math.pi / 2.0)
    return mesh_from_geometry(geom, mesh_name)


def _leg_origin(sx: float, sy_sign: float) -> Origin:
    """Midpoint and tilt for an A-frame leg from foot to apex."""
    mid_y = sy_sign * LEG_SPREAD / 2.0
    mid_z = APEX_Z / 2.0
    tilt = sy_sign * LEG_ANGLE
    return Origin(xyz=(sx, mid_y, mid_z), rpy=(tilt, 0.0, 0.0))


def _brace_origin_and_length(sx: float, brace_z: float) -> tuple[Origin, float]:
    """Horizontal cross-brace at a given height between the two legs."""
    half_span = LEG_SPREAD * (1.0 - brace_z / APEX_Z)
    length = 2.0 * half_span
    origin = Origin(xyz=(sx, 0.0, brace_z), rpy=(math.pi / 2.0, 0.0, 0.0))
    return origin, length


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="circular_ring_swing_aframe")

    galvanized = model.material("galvanized_steel", rgba=(0.58, 0.60, 0.62, 1.0))
    light_steel = model.material("light_steel", rgba=(0.76, 0.78, 0.80, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.30, 0.32, 0.35, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.84, 0.86, 0.89, 1.0))
    wood = model.material("warm_wood", rgba=(0.60, 0.40, 0.22, 1.0))

    # ------------------------------------------------------------------ frame
    frame = model.part("aframe_support")

    # Ridge beam joining the two A-frame apexes.
    frame.visual(
        Box((2.0 * APEX_X, RIDGE_W, RIDGE_H)),
        origin=Origin(xyz=(0.0, 0.0, APEX_Z)),
        material=light_steel,
        name="ridge_beam",
    )

    # Each side: two splayed legs, foot plates, cross braces, apex plate.
    for si, sx in enumerate((-APEX_X, APEX_X)):
        for li, sy_sign in enumerate((1.0, -1.0)):
            # Angled leg (cylinder from foot at y=sy_sign*SPREAD, z=0 to apex).
            frame.visual(
                Cylinder(radius=LEG_RADIUS, length=LEG_LENGTH),
                origin=_leg_origin(sx, sy_sign),
                material=galvanized,
                name=f"leg_{si}_{li}",
            )
            # Ground foot plate.
            foot_y = sy_sign * LEG_SPREAD
            frame.visual(
                Cylinder(radius=0.06, length=0.020),
                origin=Origin(xyz=(sx, foot_y, 0.010)),
                material=dark_steel,
                name=f"foot_plate_{si}_{li}",
            )

        for bi, bz in enumerate(BRACE_HEIGHTS):
            brace_origin, brace_len = _brace_origin_and_length(sx, bz)
            frame.visual(
                Cylinder(radius=BRACE_RADIUS, length=brace_len),
                origin=brace_origin,
                material=galvanized,
                name=f"brace_{si}_{bi}",
            )

        # Apex gusset plate tying the legs into the ridge beam.
        frame.visual(
            Box((0.014, 0.30, 0.36)),
            origin=Origin(xyz=(sx, 0.0, APEX_Z - 0.10)),
            material=light_steel,
            name=f"apex_plate_{si}",
        )

    # ------------------------------------------------------------------ chain
    chain = model.part("hanger_chain")

    # Eye-bolt stem socketed up into the ridge beam.
    chain.visual(
        Cylinder(radius=0.012, length=STEM_LEN),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=chrome,
        name="anchor_stem",
    )

    eye_geom = TorusGeometry(0.016, 0.005, radial_segments=12, tubular_segments=32)
    eye_geom.rotate_x(math.pi / 2.0)
    chain.visual(
        mesh_from_geometry(eye_geom, "anchor_eye"),
        origin=Origin(xyz=(0.0, 0.0, EYE_C)),
        material=chrome,
        name="anchor_eye",
    )

    link_yz = _oval_link_mesh("yz", "chain_link_yz")
    link_xz = _oval_link_mesh("xz", "chain_link_xz")
    for li, lc in enumerate(LINK_CENTERS):
        chain.visual(
            link_yz if li % 2 == 0 else link_xz,
            origin=Origin(xyz=(0.0, 0.0, lc)),
            material=chrome,
            name=f"chain_link_{li}",
        )

    hook_eye_geom = TorusGeometry(0.016, 0.0055, radial_segments=12, tubular_segments=32)
    hook_eye_geom.rotate_y(math.pi / 2.0)
    chain.visual(
        mesh_from_geometry(hook_eye_geom, "hook_eye"),
        origin=Origin(xyz=(0.0, 0.0, HOOK_EYE_C)),
        material=chrome,
        name="hook_eye",
    )

    hook_pts = [(0.0, 0.0, -0.4985)]
    for theta_deg in (90.0, 130.0, 170.0, 210.0, 250.0, 290.0, 330.0):
        th = math.radians(theta_deg)
        hook_pts.append(
            (HOOK_ARC_R * math.cos(th), 0.0, HOOK_ARC_C + HOOK_ARC_R * math.sin(th))
        )
    hook_geom = tube_from_spline_points(
        hook_pts,
        radius=0.0085,
        samples_per_segment=10,
        radial_segments=14,
        cap_ends=True,
    )
    chain.visual(
        mesh_from_geometry(hook_geom, "hook"),
        material=chrome,
        name="hook",
    )

    # ------------------------------------------------------------------- ring
    ring = model.part("ring_seat")

    eyelet_geom = TorusGeometry(EYELET_R, EYELET_TUBE, radial_segments=14, tubular_segments=40)
    eyelet_geom.rotate_y(math.pi / 2.0)
    ring.visual(
        mesh_from_geometry(eyelet_geom, "ring_eyelet"),
        material=chrome,
        name="ring_eyelet",
    )

    ring.visual(
        Box((0.05, 0.115, 0.042)),
        origin=Origin(xyz=(0.0, 0.0, CLEVIS_C)),
        material=chrome,
        name="hanger_clevis",
    )

    hoop_geom = TorusGeometry(HOOP_R, HOOP_TUBE, radial_segments=18, tubular_segments=140)
    hoop_geom.rotate_x(math.pi / 2.0)
    hoop_mesh = mesh_from_geometry(hoop_geom, "ring_hoop")
    for hi, hy in enumerate((-HOOP_Y, HOOP_Y)):
        ring.visual(
            hoop_mesh,
            origin=Origin(xyz=(0.0, hy, RING_C)),
            material=chrome,
            name=f"hoop_{hi}",
        )

    for si in range(PLANK_COUNT):
        theta = PLANK_THETA0 + (PLANK_THETA1 - PLANK_THETA0) * si / (PLANK_COUNT - 1)
        cx = PLANK_RC * math.cos(theta)
        cz = RING_C + PLANK_RC * math.sin(theta)
        ring.visual(
            Box((0.070, 0.115, 0.018)),
            origin=Origin(xyz=(cx, 0.0, cz), rpy=(0.0, math.pi / 2.0 - theta, 0.0)),
            material=wood,
            name=f"seat_plank_{si}",
        )

    # ----------------------------------------------------------- articulations
    model.articulation(
        "canopy_swing",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=chain,
        origin=Origin(xyz=(0.0, 0.0, RIDGE_Z0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=600.0,
            velocity=3.0,
            lower=-math.radians(45.0),
            upper=math.radians(45.0),
        ),
    )

    model.articulation(
        "ring_spin",
        ArticulationType.CONTINUOUS,
        parent=chain,
        child=ring,
        origin=Origin(xyz=(0.0, 0.0, SPIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("aframe_support")
    chain = object_model.get_part("hanger_chain")
    ring = object_model.get_part("ring_seat")
    swing = object_model.get_articulation("canopy_swing")
    spin = object_model.get_articulation("ring_spin")

    # Intentional local embeds that make the mechanism read as real hardware.
    ctx.allow_overlap(
        frame,
        chain,
        elem_a="ridge_beam",
        elem_b="anchor_stem",
        reason="The chain anchor stem is an eye-bolt shank intentionally socketed into the ridge beam.",
    )
    ctx.allow_overlap(
        chain,
        ring,
        elem_a="hook",
        elem_b="ring_eyelet",
        reason="The ring eyelet is captured on the hook bowl with a small seating embed, like a real hook-and-eye interlock.",
    )

    # --- chain anchored on the ridge beam centreline -------------------------
    ctx.expect_overlap(
        chain,
        frame,
        axes="z",
        elem_a="anchor_stem",
        elem_b="ridge_beam",
        min_overlap=0.02,
        name="chain anchor stem is socketed into the ridge beam",
    )
    ctx.expect_within(
        chain,
        frame,
        axes="xy",
        inner_elem="anchor_stem",
        outer_elem="ridge_beam",
        margin=0.001,
        name="chain anchor hangs on the ridge beam centreline",
    )

    # --- hook-and-eyelet suspension ------------------------------------------
    ctx.expect_contact(
        chain,
        ring,
        elem_a="hook",
        elem_b="ring_eyelet",
        contact_tol=0.0005,
        name="ring eyelet is seated on the hook",
    )
    ctx.expect_overlap(
        chain,
        ring,
        axes="z",
        elem_a="hook",
        elem_b="ring_eyelet",
        min_overlap=0.015,
        name="hook retains the ring eyelet vertically",
    )

    # --- A-frame support hero features ----------------------------------------
    frame_aabb = ctx.part_world_aabb(frame)
    ctx.check(
        "A-frame is about 3.0 m wide and 2.6 m tall",
        frame_aabb is not None
        and 2.95 <= frame_aabb[1][0] - frame_aabb[0][0] <= 3.20
        and 2.55 <= frame_aabb[1][2] <= 2.72,
        details=f"frame aabb={frame_aabb}",
    )

    leg_00 = ctx.part_element_world_aabb(frame, elem="leg_0_0")
    leg_01 = ctx.part_element_world_aabb(frame, elem="leg_0_1")
    leg_10 = ctx.part_element_world_aabb(frame, elem="leg_1_0")
    leg_11 = ctx.part_element_world_aabb(frame, elem="leg_1_1")
    ctx.check(
        "four angled legs present (two per A-frame side)",
        all(v is not None for v in (leg_00, leg_01, leg_10, leg_11)),
        details=f"legs={[leg_00, leg_01, leg_10, leg_11]}",
    )

    # Legs reach the ground.
    ctx.check(
        "all legs reach down to the ground plane",
        leg_00 is not None and leg_01 is not None
        and leg_10 is not None and leg_11 is not None
        and leg_00[0][2] < 0.05 and leg_01[0][2] < 0.05
        and leg_10[0][2] < 0.05 and leg_11[0][2] < 0.05,
        details=f"leg bottoms: {[v[0][2] for v in (leg_00, leg_01, leg_10, leg_11)]}",
    )

    # Legs splay outward: feet are wider in Y than the apex.
    ctx.check(
        "A-frame legs splay outward (feet spread in Y)",
        leg_00 is not None and leg_01 is not None
        and leg_00[1][1] - leg_01[0][1] > 1.0,
        details=f"leg_00 max_y={leg_00[1][1]}, leg_01 min_y={leg_01[0][1]}",
    )

    # Ridge beam spans between the two A-frame apexes.
    beam = ctx.part_element_world_aabb(frame, elem="ridge_beam")
    ctx.check(
        "ridge beam spans between the two A-frame apexes",
        beam is not None
        and beam[1][0] - beam[0][0] > 2.8
        and beam[0][2] > 2.50
        and beam[1][2] < 2.70,
        details=f"ridge_beam={beam}",
    )

    # Cross braces connect legs on each A-frame side.
    brace_count = sum(
        1 for v in frame.visuals if (v.name or "").startswith("brace_")
    )
    ctx.check(
        "each A-frame side has cross braces",
        brace_count >= 4,
        details=f"brace_count={brace_count}",
    )

    # Apex plates tie the legs to the ridge beam.
    apex_0 = ctx.part_element_world_aabb(frame, elem="apex_plate_0")
    apex_1 = ctx.part_element_world_aabb(frame, elem="apex_plate_1")
    ctx.check(
        "apex gusset plates sit at each A-frame apex",
        apex_0 is not None and apex_1 is not None
        and apex_0[0][2] > 2.2 and apex_1[0][2] > 2.2,
        details=f"apex_0={apex_0}, apex_1={apex_1}",
    )

    # --- twin-hoop ring with wooden bench arc ---------------------------------
    hoop_0 = ctx.part_element_world_aabb(ring, elem="hoop_0")
    hoop_1 = ctx.part_element_world_aabb(ring, elem="hoop_1")
    ctx.check(
        "twin hoops are separated by a narrow gap",
        hoop_0 is not None
        and hoop_1 is not None
        and 0.015 <= hoop_1[0][1] - hoop_0[1][1] <= 0.04,
        details=f"hoop_0={hoop_0}, hoop_1={hoop_1}",
    )
    ctx.check(
        "ring hoop is about 1.5 m in outer diameter",
        hoop_0 is not None
        and 1.45 <= hoop_0[1][0] - hoop_0[0][0] <= 1.55
        and 1.45 <= hoop_0[1][2] - hoop_0[0][2] <= 1.55,
        details=f"hoop_0={hoop_0}",
    )

    ring_aabb = ctx.part_world_aabb(ring)
    ctx.check(
        "ring hangs clear of the ground and below the ridge beam",
        ring_aabb is not None and ring_aabb[0][2] > 0.30 and ring_aabb[1][2] < RIDGE_Z0,
        details=f"ring aabb={ring_aabb}",
    )

    ring_center_z = RIDGE_Z0 + SPIN_Z + RING_C
    bottom_plank = ctx.part_element_world_aabb(ring, elem="seat_plank_11")
    ctx.check(
        "wooden seat planks line the lower inner arc of the ring",
        bottom_plank is not None and bottom_plank[1][2] < ring_center_z - 0.45,
        details=f"seat_plank_11={bottom_plank}, ring_center_z={ring_center_z}",
    )
    ctx.check(
        "seat planks span across both hoops",
        bottom_plank is not None
        and hoop_0 is not None
        and hoop_1 is not None
        and bottom_plank[0][1] < hoop_0[0][1] + 0.02
        and bottom_plank[1][1] > hoop_1[1][1] - 0.02,
        details=f"seat_plank_11={bottom_plank}",
    )

    # --- swing articulation (fore-aft about the beam axis) --------------------
    rest_pos = ctx.part_world_position(ring)
    with ctx.pose({swing: math.radians(45.0)}):
        fwd_pos = ctx.part_world_position(ring)
    with ctx.pose({swing: -math.radians(45.0)}):
        back_pos = ctx.part_world_position(ring)
    ctx.check(
        "positive swing carries the ring forward and upward",
        rest_pos is not None
        and fwd_pos is not None
        and fwd_pos[1] > rest_pos[1] + 0.30
        and fwd_pos[2] > rest_pos[2] + 0.10,
        details=f"rest={rest_pos}, fwd={fwd_pos}",
    )
    ctx.check(
        "negative swing carries the ring backward",
        rest_pos is not None and back_pos is not None and back_pos[1] < rest_pos[1] - 0.30,
        details=f"rest={rest_pos}, back={back_pos}",
    )

    # --- continuous spin about the vertical chain axis -------------------------
    with ctx.pose({spin: math.pi / 2.0}):
        spun_hoop = ctx.part_element_world_aabb(ring, elem="hoop_0")
    ctx.check(
        "quarter spin turns the ring plane about the vertical chain axis",
        spun_hoop is not None
        and spun_hoop[1][1] - spun_hoop[0][1] > 1.2
        and spun_hoop[1][0] - spun_hoop[0][0] < 0.3,
        details=f"spun hoop_0={spun_hoop}",
    )

    return ctx.report()


object_model = build_object_model()
