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
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Modern circular ring swing hanging from a single steel arch.
#
# World frame: Z up, ground at z = 0, X spans the ~3.0 m arch width,
# Y is the swing (fore-aft) direction.
#
# Layout summary (meters):
# - arch beam:   swept rounded-square tube, parabolic path, crown at z~2.54
# - foot plates: 0.30 x 0.30 x 0.025, at x = +-1.45
# - crown bracket: anchor lug at arch crown underside
# - swing pivot: (0, 0, ARCH_UNDERSIDE), revolute about +X, -45 .. +45 deg
# - chain:       eye-bolt stem + anchor eye + 4 oval links + hook eye + J-hook
# - spin pivot:  hook bowl, chain-local (0, 0, -0.576), continuous about +Z
# - ring:        twin chrome hoops OD 1.5 (R 0.725, tube 0.025) at y = +-0.0375,
#                wooden seat planks lining the lower inner arc
# ---------------------------------------------------------------------------

# Arch dimensions
ARCH_FOOT_X = 1.45
ARCH_CROWN_Z = 2.54  # center of beam at crown
ARCH_BEAM = 0.12  # square cross-section size
ARCH_CORNER_R = 0.015  # cross-section corner radius
FOOT_SIZE = 0.30
FOOT_THICK = 0.025
ARCH_UNDERSIDE = ARCH_CROWN_Z - ARCH_BEAM / 2.0  # ≈ 2.48
ARCH_TOP = ARCH_CROWN_Z + ARCH_BEAM / 2.0  # ≈ 2.60

# Chain layout (chain-local frame, origin at the swing pivot on the arch underside)
STEM_LEN = 0.09
EYE_C = -0.078
LINK_R = 0.028
LINK_TUBE = 0.0065
LINK_STRETCH = 1.45
LINK_CENTERS = (-0.144, -0.234, -0.324, -0.415)  # alternating yz / xz planes
HOOK_EYE_C = -0.481
HOOK_ARC_C = -0.536
HOOK_ARC_R = 0.024
SPIN_Z = -0.576  # hook bowl seat / spin joint, chain-local

# Ring layout (ring-local frame, origin at the spin joint / eyelet center)
EYELET_R = 0.030
EYELET_TUBE = 0.007
CLEVIS_C = -0.051
HOOP_R = 0.725
HOOP_TUBE = 0.025
HOOP_Y = 0.0375
RING_C = -0.815  # hoop circle center, ring-local
PLANK_RC = 0.693  # plank center radius (outer face embeds 2 mm into hoop bore)
PLANK_COUNT = 22
PLANK_THETA0 = math.radians(197.0)
PLANK_THETA1 = math.radians(343.0)


def _arch_path_points(n: int = 13) -> list[tuple[float, float, float]]:
    """Parabolic arch path from left foot to right foot, crown at ARCH_CROWN_Z."""
    pts: list[tuple[float, float, float]] = []
    foot_top = FOOT_THICK  # beam starts at foot plate top for contact
    for i in range(n):
        t = i / (n - 1)
        x = -ARCH_FOOT_X + t * 2.0 * ARCH_FOOT_X
        # Parabola: z = crown * (1 - (x/half)^2) + foot_top * (x/half)^2
        ratio = (x / ARCH_FOOT_X) ** 2
        z = ARCH_CROWN_Z * (1.0 - ratio) + foot_top * ratio
        pts.append((x, 0.0, z))
    return pts


def _oval_link_mesh(plane: str, mesh_name: str) -> Mesh:
    """Oval chain link, long axis along Z, ring plane 'yz' or 'xz'."""
    geom = TorusGeometry(LINK_R, LINK_TUBE, radial_segments=12, tubular_segments=36)
    geom.scale(LINK_STRETCH, 1.0, 1.0)
    geom.rotate_y(math.pi / 2.0)  # long axis -> Z, ring plane -> YZ
    if plane == "xz":
        geom.rotate_z(math.pi / 2.0)
    return mesh_from_geometry(geom, mesh_name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="circular_ring_swing_arch")

    galvanized = model.material("galvanized_steel", rgba=(0.58, 0.60, 0.62, 1.0))
    light_steel = model.material("light_steel", rgba=(0.76, 0.78, 0.80, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.30, 0.32, 0.35, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.84, 0.86, 0.89, 1.0))
    wood = model.material("warm_wood", rgba=(0.60, 0.40, 0.22, 1.0))

    # ---------------------------------------------------------- arch frame
    frame = model.part("arch_frame")

    # Single swept arch beam: rounded-square profile along parabolic path
    arch_profile = rounded_rect_profile(ARCH_BEAM, ARCH_BEAM, ARCH_CORNER_R)
    arch_geom = sweep_profile_along_spline(
        _arch_path_points(n=13),
        profile=arch_profile,
        samples_per_segment=16,
        cap_profile=True,
        up_hint=(0.0, 1.0, 0.0),
    )
    frame.visual(
        mesh_from_geometry(arch_geom, "arch_beam"),
        material=galvanized,
        name="arch_beam",
    )

    # Foot plates at each base
    for i, fx in enumerate((-ARCH_FOOT_X, ARCH_FOOT_X)):
        frame.visual(
            Box((FOOT_SIZE, FOOT_SIZE, FOOT_THICK)),
            origin=Origin(xyz=(fx, 0.0, FOOT_THICK / 2.0)),
            material=dark_steel,
            name=f"foot_plate_{i}",
        )

    # Crown anchor bracket — a small welded lug plate at the arch underside
    frame.visual(
        Box((0.10, 0.10, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, ARCH_UNDERSIDE - 0.02)),
        material=light_steel,
        name="crown_bracket",
    )

    # ---------------------------------------------------------- hanger chain
    chain = model.part("hanger_chain")

    # Eye-bolt stem socketed up into the crown bracket
    chain.visual(
        Cylinder(radius=0.012, length=STEM_LEN),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=chrome,
        name="anchor_stem",
    )

    eye_geom = TorusGeometry(0.016, 0.005, radial_segments=12, tubular_segments=32)
    eye_geom.rotate_x(math.pi / 2.0)  # ring plane XZ
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
    hook_eye_geom.rotate_y(math.pi / 2.0)  # ring plane YZ
    chain.visual(
        mesh_from_geometry(hook_eye_geom, "hook_eye"),
        origin=Origin(xyz=(0.0, 0.0, HOOK_EYE_C)),
        material=chrome,
        name="hook_eye",
    )

    # J-hook: straight shank out of the hook eye, then a bowl arc in the XZ plane.
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

    # ---------------------------------------------------------- ring seat
    ring = model.part("ring_seat")

    eyelet_geom = TorusGeometry(EYELET_R, EYELET_TUBE, radial_segments=14, tubular_segments=40)
    eyelet_geom.rotate_y(math.pi / 2.0)  # ring plane YZ, hole axis X
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
    hoop_geom.rotate_x(math.pi / 2.0)  # ring plane XZ, axis Y
    hoop_mesh = mesh_from_geometry(hoop_geom, "ring_hoop")
    for hi, hy in enumerate((-HOOP_Y, HOOP_Y)):
        ring.visual(
            hoop_mesh,
            origin=Origin(xyz=(0.0, hy, RING_C)),
            material=chrome,
            name=f"hoop_{hi}",
        )

    # Wooden seat planks lining the lower inner arc, spanning both hoops.
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
        origin=Origin(xyz=(0.0, 0.0, ARCH_UNDERSIDE)),
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

    frame = object_model.get_part("arch_frame")
    chain = object_model.get_part("hanger_chain")
    ring = object_model.get_part("ring_seat")
    swing = object_model.get_articulation("canopy_swing")
    spin = object_model.get_articulation("ring_spin")

    # Intentional local embeds that make the mechanism read as real hardware.
    ctx.allow_overlap(
        frame,
        chain,
        elem_a="arch_beam",
        elem_b="anchor_stem",
        reason="The chain anchor stem is an eye-bolt shank intentionally socketed up through the arch crown beam.",
    )
    ctx.allow_overlap(
        frame,
        chain,
        elem_a="crown_bracket",
        elem_b="anchor_stem",
        reason="The chain anchor stem passes through the crown bracket lug welded to the arch underside.",
    )
    ctx.allow_overlap(
        chain,
        ring,
        elem_a="hook",
        elem_b="ring_eyelet",
        reason="The ring eyelet is captured on the hook bowl with a small seating embed, like a real hook-and-eye interlock.",
    )

    # --- arch frame geometry claims -------------------------------------------
    frame_aabb = ctx.part_world_aabb(frame)
    ctx.check(
        "arch frame spans about 3.0 m wide and 2.6 m tall",
        frame_aabb is not None
        and 2.7 <= frame_aabb[1][0] - frame_aabb[0][0] <= 3.2
        and 2.45 <= frame_aabb[1][2] <= 2.70,
        details=f"frame aabb={frame_aabb}",
    )

    foot_0 = ctx.part_element_world_aabb(frame, elem="foot_plate_0")
    foot_1 = ctx.part_element_world_aabb(frame, elem="foot_plate_1")
    ctx.check(
        "two foot plates sit at the ground on either side",
        foot_0 is not None
        and foot_1 is not None
        and foot_0[0][2] < 0.03
        and foot_1[0][2] < 0.03
        and foot_1[0][0] - foot_0[1][0] > 2.0,
        details=f"foot_0={foot_0}, foot_1={foot_1}",
    )

    arch_beam = ctx.part_element_world_aabb(frame, elem="arch_beam")
    ctx.check(
        "arch beam is a single continuous curved member",
        arch_beam is not None
        and arch_beam[1][0] - arch_beam[0][0] > 2.5
        and arch_beam[1][2] > 2.3,
        details=f"arch_beam={arch_beam}",
    )

    crown = ctx.part_element_world_aabb(frame, elem="crown_bracket")
    ctx.check(
        "crown bracket sits at the arch apex underside",
        crown is not None
        and abs(crown[0][0] + crown[1][0]) < 0.15
        and crown[1][2] > 2.35,
        details=f"crown={crown}",
    )

    # --- chain anchored at arch crown -----------------------------------------
    ctx.expect_overlap(
        chain,
        frame,
        axes="z",
        elem_a="anchor_stem",
        elem_b="crown_bracket",
        min_overlap=0.02,
        name="chain anchor stem is socketed into the crown bracket",
    )
    ctx.expect_within(
        chain,
        frame,
        axes="xy",
        inner_elem="anchor_stem",
        outer_elem="crown_bracket",
        margin=0.001,
        name="chain anchor hangs at the arch crown centerline",
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
        "ring hangs clear of the ground and below the arch crown",
        ring_aabb is not None and ring_aabb[0][2] > 0.30 and ring_aabb[1][2] < ARCH_UNDERSIDE,
        details=f"ring aabb={ring_aabb}",
    )

    ring_center_z = ARCH_UNDERSIDE + SPIN_Z + RING_C
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

    # --- swing articulation (fore-aft about the arch crown beam axis) ---------
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
