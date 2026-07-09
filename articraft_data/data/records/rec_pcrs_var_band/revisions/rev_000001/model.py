from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Modern circular ring swing hanging from a steel pergola frame.
#
# World frame: Z up, ground at z = 0, X spans the 3.0 m frame width,
# Y is the swing (fore-aft) direction.
#
# Layout summary (meters):
# - posts:      0.15 sq tube, x = +-1.425, z 0 .. 2.555 (embed into top rail)
# - top rail:   3.00 x 0.28 x 0.05, z 2.55 .. 2.60
# - louver fins: 34 fins, 0.04 thick x 0.25 deep x 0.13 tall, z 2.425 .. 2.555
# - swing pivot: (0, 0, 2.55), revolute about +X, -45 .. +45 deg
# - chain:      eye-bolt stem + anchor eye + 4 oval links + hook eye + J-hook
# - spin pivot: hook bowl, chain-local (0, 0, -0.576), continuous about +Z
# - ring:       single wide flat steel band OD 1.5 (R 0.75, 5 mm thick, 110 mm wide),
#               wooden seat planks lining the lower inner arc
# ---------------------------------------------------------------------------

# Frame dimensions
POST_X = 1.425
POST_SIZE = 0.15
POST_TOP = 2.555
RAIL_Z0, RAIL_Z1 = 2.55, 2.60
FIN_Z0, FIN_Z1 = 2.425, 2.555
FIN_COUNT = 34
FIN_SPAN = 2.94  # fin centers from -1.47 to +1.47

# Chain layout (chain-local frame, origin at the swing pivot on the rail underside)
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
BAND_OUTER_R = 0.750  # outer radius of the flat steel band (1.50 m OD)
BAND_THICKNESS = 0.005  # radial thickness of the flat band (5 mm steel ribbon)
BAND_WIDTH = 0.110  # axial width of the band (the ribbon height when upright)
RING_C = -0.815  # band circle center, ring-local (along local Z)
PLANK_RC = BAND_OUTER_R - BAND_THICKNESS - 0.009 + 0.002  # plank center radius (outer face embeds 2 mm into band bore)
PLANK_COUNT = 22
PLANK_THETA0 = math.radians(197.0)
PLANK_THETA1 = math.radians(343.0)


def _oval_link_mesh(plane: str, mesh_name: str) -> Mesh:
    """Oval chain link, long axis along Z, ring plane 'yz' or 'xz'."""
    geom = TorusGeometry(LINK_R, LINK_TUBE, radial_segments=12, tubular_segments=36)
    geom.scale(LINK_STRETCH, 1.0, 1.0)
    geom.rotate_y(math.pi / 2.0)  # long axis -> Z, ring plane -> YZ
    if plane == "xz":
        geom.rotate_z(math.pi / 2.0)
    return mesh_from_geometry(geom, mesh_name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="circular_ring_swing_pergola")

    galvanized = model.material("galvanized_steel", rgba=(0.58, 0.60, 0.62, 1.0))
    light_steel = model.material("light_steel", rgba=(0.76, 0.78, 0.80, 1.0))
    fin_grey = model.material("fin_grey", rgba=(0.22, 0.23, 0.25, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.84, 0.86, 0.89, 1.0))
    wood = model.material("warm_wood", rgba=(0.60, 0.40, 0.22, 1.0))
    rivet_steel = model.material("rivet_steel", rgba=(0.45, 0.47, 0.50, 1.0))

    # ------------------------------------------------------------------ frame
    frame = model.part("pergola_frame")

    for pi, px in enumerate((-POST_X, POST_X)):
        frame.visual(
            Box((POST_SIZE, POST_SIZE, POST_TOP)),
            origin=Origin(xyz=(px, 0.0, POST_TOP / 2.0)),
            material=galvanized,
            name=f"post_{pi}",
        )

    frame.visual(
        Box((3.0, 0.28, RAIL_Z1 - RAIL_Z0)),
        origin=Origin(xyz=(0.0, 0.0, (RAIL_Z0 + RAIL_Z1) / 2.0)),
        material=light_steel,
        name="top_rail",
    )

    fin_pitch = FIN_SPAN / (FIN_COUNT - 1)
    for i in range(FIN_COUNT):
        fx = -FIN_SPAN / 2.0 + i * fin_pitch
        frame.visual(
            Box((0.04, 0.25, FIN_Z1 - FIN_Z0)),
            origin=Origin(xyz=(fx, 0.0, (FIN_Z0 + FIN_Z1) / 2.0)),
            material=fin_grey,
            name=f"fin_{i}",
        )

    # Riveted gusset plates on the front/back faces of each post top.
    for pi, px in enumerate((-POST_X, POST_X)):
        for fi, fy in enumerate((-0.080, 0.080)):
            frame.visual(
                Box((0.15, 0.012, 0.42)),
                origin=Origin(xyz=(px, fy, 2.36)),
                material=light_steel,
                name=f"gusset_{pi}_{fi}",
            )
            ry = math.copysign(0.088, fy)
            for ri, (dx, dz) in enumerate(
                ((-0.042, -0.14), (0.042, -0.14), (-0.042, 0.14), (0.042, 0.14))
            ):
                frame.visual(
                    Cylinder(radius=0.011, length=0.012),
                    origin=Origin(xyz=(px + dx, ry, 2.36 + dz), rpy=(math.pi / 2.0, 0.0, 0.0)),
                    material=rivet_steel,
                    name=f"rivet_{pi}_{fi}_{ri}",
                )

    # ------------------------------------------------------------------ chain
    chain = model.part("hanger_chain")

    # Eye-bolt stem socketed up into the top rail (z -0.06 .. +0.03 local).
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

    # ------------------------------------------------------------------- ring
    ring = model.part("ring_seat")

    eyelet_geom = TorusGeometry(EYELET_R, EYELET_TUBE, radial_segments=14, tubular_segments=40)
    eyelet_geom.rotate_y(math.pi / 2.0)  # ring plane YZ, hole axis X (threads the hook)
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

    # --- single wide flat steel band (a broad flat ribbon rolled into a circle)
    # Built on the XY plane (ring in XY, hole axis Z), then rotated to XZ (axis Y).
    band_inner_r = BAND_OUTER_R - BAND_THICKNESS
    band_cq = (
        cq.Workplane("XY")
        .circle(BAND_OUTER_R)
        .circle(band_inner_r)
        .extrude(BAND_WIDTH / 2.0, both=True)
    )
    # Rotate from XY plane (axis Z) to XZ plane (axis Y) for the ring-local frame.
    band_cq = band_cq.rotateAboutCenter((1, 0, 0), 90)
    band_mesh = mesh_from_cadquery(band_cq, "flat_band")

    brushed_steel = model.material("brushed_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    ring.visual(
        band_mesh,
        origin=Origin(xyz=(0.0, 0.0, RING_C)),
        material=brushed_steel,
        name="flat_band",
    )

    # Wooden seat planks lining the lower inner arc of the flat band.
    for si in range(PLANK_COUNT):
        theta = PLANK_THETA0 + (PLANK_THETA1 - PLANK_THETA0) * si / (PLANK_COUNT - 1)
        cx = PLANK_RC * math.cos(theta)
        cz = RING_C + PLANK_RC * math.sin(theta)
        ring.visual(
            Box((0.065, 0.095, 0.018)),
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
        origin=Origin(xyz=(0.0, 0.0, RAIL_Z0)),
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

    frame = object_model.get_part("pergola_frame")
    chain = object_model.get_part("hanger_chain")
    ring = object_model.get_part("ring_seat")
    swing = object_model.get_articulation("canopy_swing")
    spin = object_model.get_articulation("ring_spin")

    # Intentional local embeds that make the mechanism read as real hardware.
    ctx.allow_overlap(
        frame,
        chain,
        elem_a="top_rail",
        elem_b="anchor_stem",
        reason="The chain anchor stem is an eye-bolt shank intentionally socketed into the canopy top rail.",
    )
    ctx.allow_overlap(
        chain,
        ring,
        elem_a="hook",
        elem_b="ring_eyelet",
        reason="The ring eyelet is captured on the hook bowl with a small seating embed, like a real hook-and-eye interlock.",
    )

    # --- chain anchored on the canopy centerline -----------------------------
    ctx.expect_overlap(
        chain,
        frame,
        axes="z",
        elem_a="anchor_stem",
        elem_b="top_rail",
        min_overlap=0.02,
        name="chain anchor stem is socketed into the canopy top rail",
    )
    ctx.expect_within(
        chain,
        frame,
        axes="xy",
        inner_elem="anchor_stem",
        outer_elem="top_rail",
        margin=0.001,
        name="chain anchor hangs on the canopy centerline",
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

    # --- pergola frame hero features ------------------------------------------
    frame_aabb = ctx.part_world_aabb(frame)
    ctx.check(
        "frame is about 3.0 m wide and 2.6 m tall",
        frame_aabb is not None
        and 2.95 <= frame_aabb[1][0] - frame_aabb[0][0] <= 3.05
        and 2.55 <= frame_aabb[1][2] <= 2.65,
        details=f"frame aabb={frame_aabb}",
    )

    post_0 = ctx.part_element_world_aabb(frame, elem="post_0")
    post_1 = ctx.part_element_world_aabb(frame, elem="post_1")
    ctx.check(
        "two square posts stand about 2.7 m apart",
        post_0 is not None
        and post_1 is not None
        and 2.5 <= post_1[0][0] - post_0[1][0] <= 2.8,
        details=f"post_0={post_0}, post_1={post_1}",
    )

    fin_count = sum(1 for v in frame.visuals if (v.name or "").startswith("fin_"))
    ctx.check(
        "canopy has a dense row of louver fins",
        fin_count >= 30,
        details=f"fin_count={fin_count}",
    )
    ctx.expect_overlap(
        frame,
        frame,
        axes="z",
        elem_a="fin_0",
        elem_b="top_rail",
        min_overlap=0.003,
        name="louver fins hang from the steel top rail",
    )

    fin_0 = ctx.part_element_world_aabb(frame, elem="fin_0")
    rail = ctx.part_element_world_aabb(frame, elem="top_rail")
    ctx.check(
        "top rail caps the fins from above",
        fin_0 is not None and rail is not None and rail[1][2] > fin_0[1][2],
        details=f"fin_0={fin_0}, rail={rail}",
    )

    gusset = ctx.part_element_world_aabb(frame, elem="gusset_0_1")
    ctx.check(
        "gusset plates sit at the post top corners",
        gusset is not None and gusset[0][2] > 2.0 and gusset[0][0] < -POST_X + 0.16,
        details=f"gusset={gusset}",
    )

    # --- single wide flat steel band with wooden bench arc --------------------
    band_aabb = ctx.part_element_world_aabb(ring, elem="flat_band")
    ctx.check(
        "flat band is about 1.5 m in outer diameter",
        band_aabb is not None
        and 1.45 <= band_aabb[1][0] - band_aabb[0][0] <= 1.55
        and 1.45 <= band_aabb[1][2] - band_aabb[0][2] <= 1.55,
        details=f"flat_band={band_aabb}",
    )
    ctx.check(
        "flat band is a wide ribbon (axial width ~0.10 m, thin radial depth)",
        band_aabb is not None
        and 0.08 <= band_aabb[1][1] - band_aabb[0][1] <= 0.14,
        details=f"flat_band={band_aabb}",
    )

    ring_aabb = ctx.part_world_aabb(ring)
    ctx.check(
        "ring hangs clear of the ground and below the canopy fins",
        ring_aabb is not None and ring_aabb[0][2] > 0.30 and ring_aabb[1][2] < FIN_Z0,
        details=f"ring aabb={ring_aabb}",
    )

    ring_center_z = RAIL_Z0 + SPIN_Z + RING_C
    bottom_plank = ctx.part_element_world_aabb(ring, elem="seat_plank_11")
    ctx.check(
        "wooden seat planks line the lower inner arc of the ring",
        bottom_plank is not None and bottom_plank[1][2] < ring_center_z - 0.45,
        details=f"seat_plank_11={bottom_plank}, ring_center_z={ring_center_z}",
    )
    ctx.check(
        "seat planks fit within the band width",
        bottom_plank is not None
        and band_aabb is not None
        and bottom_plank[0][1] >= band_aabb[0][1] - 0.01
        and bottom_plank[1][1] <= band_aabb[1][1] + 0.01,
        details=f"seat_plank_11={bottom_plank}, band={band_aabb}",
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
        spun_band = ctx.part_element_world_aabb(ring, elem="flat_band")
    ctx.check(
        "quarter spin turns the ring plane about the vertical chain axis",
        spun_band is not None
        and spun_band[1][1] - spun_band[0][1] > 1.2
        and spun_band[1][0] - spun_band[0][0] < 0.3,
        details=f"spun flat_band={spun_band}",
    )

    return ctx.report()


object_model = build_object_model()
