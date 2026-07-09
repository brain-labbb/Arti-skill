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
# Modern circular ring swing with 4-point bridle suspension, hanging from
# a steel pergola frame, about 3.0 m wide and 2.6 m tall overall.
#
# World frame: Z up, ground at z = 0, X spans the 3.0 m frame width,
# Y is the swing (fore-aft) direction.
#
# Layout summary (meters):
# - posts:         0.15 sq tube, x = +-1.425, z 0 .. 2.555
# - top rail:      3.00 x 0.28 x 0.05, z 2.55 .. 2.60
# - louver fins:   34 fins, 0.04 thick x 0.25 deep x 0.13 tall
# - swing pivot:   (0, 0, 2.55), revolute about +X, -45 .. +45 deg
# - swivel_hook:   anchor stem + plate + barrel + convergence eye
# - spin pivot:    convergence eye, swivel-local (0, 0, -0.20), continuous +Z
# - ring:          twin chrome hoops OD 1.5, 4-point bridle chains,
#                  wooden seat planks lining the lower inner arc
# ---------------------------------------------------------------------------

# Frame dimensions (identical to parent)
POST_X = 1.425
POST_SIZE = 0.15
POST_TOP = 2.555
RAIL_Z0, RAIL_Z1 = 2.55, 2.60
FIN_Z0, FIN_Z1 = 2.425, 2.555
FIN_COUNT = 34
FIN_SPAN = 2.94

# Swivel-hook layout (swivel-local frame, origin at canopy swing pivot)
STEM_LEN = 0.09
PLATE_R = 0.025
PLATE_H = 0.010
BARREL_R = 0.018
BARREL_LEN = 0.08
ROD_R = 0.009
SPIN_Z = -0.20  # spin joint z in swivel-local coords

# Ring layout (ring-local frame, origin at spin joint / bridle convergence)
HOOP_R = 0.725
HOOP_TUBE = 0.025
HOOP_Y = 0.0375
RING_C = -1.18  # hoop circle center, ring-local (pushed down for long bridle)

# Bridle layout: 4 equally-spaced anchor angles on the top rim arc
BRIDLE_ANGLES_DEG = (60.0, 80.0, 100.0, 120.0)
BRIDLE_CHAIN_R = 0.006
BRIDLE_BAR_W = 0.025   # anchor bracket width (X)
BRIDLE_BAR_D = 0.020   # anchor bracket depth (Z)
BRIDLE_BAR_H = 2.0 * HOOP_Y + 2.0 * HOOP_TUBE  # spans between hoop outer faces

# Seat plank layout (identical to parent)
PLANK_RC = 0.693
PLANK_COUNT = 22
PLANK_THETA0 = math.radians(197.0)
PLANK_THETA1 = math.radians(343.0)


def _bridle_chain_mesh(ax: float, az: float, mesh_name: str) -> Mesh:
    """Tube mesh from the bridle convergence (0,0,0) to a ring-top anchor."""
    sag = 0.010
    pts = [
        (0.0, 0.0, 0.0),
        (ax * 0.30, 0.0, az * 0.30 - sag * 0.4),
        (ax * 0.60, 0.0, az * 0.60 - sag),
        (ax, 0.0, az),
    ]
    geom = tube_from_spline_points(
        pts,
        radius=BRIDLE_CHAIN_R,
        samples_per_segment=8,
        radial_segments=12,
        cap_ends=True,
    )
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

    # ----------------------------------------------------------- swivel hook
    swivel = model.part("swivel_hook")

    # Eye-bolt stem socketed up into the top rail.
    swivel.visual(
        Cylinder(radius=0.012, length=STEM_LEN),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=chrome,
        name="anchor_stem",
    )

    # Anchor plate at rail underside.
    swivel.visual(
        Cylinder(radius=PLATE_R, length=PLATE_H),
        origin=Origin(xyz=(0.0, 0.0, -0.065)),
        material=chrome,
        name="anchor_plate",
    )

    # Swivel barrel (the rotating body).
    barrel_cz = -0.065 - PLATE_H / 2.0 - BARREL_LEN / 2.0  # -0.11
    swivel.visual(
        Cylinder(radius=BARREL_R, length=BARREL_LEN),
        origin=Origin(xyz=(0.0, 0.0, barrel_cz)),
        material=chrome,
        name="swivel_barrel",
    )

    # Drop rod from barrel bottom to convergence eye.
    barrel_bot = barrel_cz - BARREL_LEN / 2.0  # -0.15
    rod_len = abs(SPIN_Z) - abs(barrel_bot)    # 0.05
    rod_cz = (barrel_bot + SPIN_Z) / 2.0       # -0.175
    swivel.visual(
        Cylinder(radius=ROD_R, length=rod_len),
        origin=Origin(xyz=(0.0, 0.0, rod_cz)),
        material=chrome,
        name="drop_rod",
    )

    # Convergence eye at the spin-joint / bridle meeting point.
    conv_geom = TorusGeometry(0.015, 0.006, radial_segments=14, tubular_segments=36)
    conv_geom.rotate_x(math.pi / 2.0)  # ring plane XZ, hole axis Y
    swivel.visual(
        mesh_from_geometry(conv_geom, "convergence_eye"),
        origin=Origin(xyz=(0.0, 0.0, SPIN_Z)),
        material=chrome,
        name="convergence_eye",
    )

    # --------------------------------------------------------------- ring
    ring = model.part("ring_seat")

    # 4-point bridle: anchor brackets + chains, emitted via for-loop.
    for i in range(len(BRIDLE_ANGLES_DEG)):
        theta = math.radians(BRIDLE_ANGLES_DEG[i])
        ax = HOOP_R * math.cos(theta)
        az = RING_C + HOOP_R * math.sin(theta)

        # Anchor bracket spanning between the twin hoops at the attachment point.
        ring.visual(
            Box((BRIDLE_BAR_W, BRIDLE_BAR_H, BRIDLE_BAR_D)),
            origin=Origin(xyz=(ax, 0.0, az)),
            material=chrome,
            name=f"bridle_anchor_{i}",
        )

        # Bridle chain tube from convergence (0,0,0) down to anchor.
        ring.visual(
            _bridle_chain_mesh(ax, az, f"bridle_chain_{i}"),
            material=chrome,
            name=f"bridle_chain_{i}",
        )

    # Twin chrome hoops (identical to parent).
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

    # Wooden seat planks lining the lower inner arc (identical to parent).
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
        child=swivel,
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
        parent=swivel,
        child=ring,
        origin=Origin(xyz=(0.0, 0.0, SPIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("pergola_frame")
    swivel = object_model.get_part("swivel_hook")
    ring = object_model.get_part("ring_seat")
    swing = object_model.get_articulation("canopy_swing")
    spin = object_model.get_articulation("ring_spin")

    # --- intentional overlaps ------------------------------------------------
    ctx.allow_overlap(
        frame, swivel,
        elem_a="top_rail", elem_b="anchor_stem",
        reason="The swivel anchor stem is an eye-bolt shank socketed into the canopy top rail.",
    )
    ctx.allow_overlap(
        swivel, ring,
        elem_a="drop_rod", elem_b="bridle_chain_0",
        reason="Bridle chain 0 converges into the drop rod / convergence eye region.",
    )
    ctx.allow_overlap(
        swivel, ring,
        elem_a="drop_rod", elem_b="bridle_chain_1",
        reason="Bridle chain 1 converges into the drop rod / convergence eye region.",
    )
    ctx.allow_overlap(
        swivel, ring,
        elem_a="drop_rod", elem_b="bridle_chain_2",
        reason="Bridle chain 2 converges into the drop rod / convergence eye region.",
    )
    ctx.allow_overlap(
        swivel, ring,
        elem_a="drop_rod", elem_b="bridle_chain_3",
        reason="Bridle chain 3 converges into the drop rod / convergence eye region.",
    )
    ctx.allow_overlap(
        swivel, ring,
        elem_a="convergence_eye", elem_b="bridle_chain_0",
        reason="Bridle chain 0 converges into the convergence eye torus at the bridle meeting point.",
    )
    ctx.allow_overlap(
        swivel, ring,
        elem_a="convergence_eye", elem_b="bridle_chain_1",
        reason="Bridle chain 1 converges into the convergence eye torus at the bridle meeting point.",
    )
    ctx.allow_overlap(
        swivel, ring,
        elem_a="convergence_eye", elem_b="bridle_chain_2",
        reason="Bridle chain 2 converges into the convergence eye torus at the bridle meeting point.",
    )
    ctx.allow_overlap(
        swivel, ring,
        elem_a="convergence_eye", elem_b="bridle_chain_3",
        reason="Bridle chain 3 converges into the convergence eye torus at the bridle meeting point.",
    )

    # --- swivel anchored on the canopy centerline ----------------------------
    ctx.expect_overlap(
        swivel, frame,
        axes="z",
        elem_a="anchor_stem", elem_b="top_rail",
        min_overlap=0.02,
        name="swivel anchor stem is socketed into the canopy top rail",
    )
    ctx.expect_within(
        swivel, frame,
        axes="xy",
        inner_elem="anchor_stem", outer_elem="top_rail",
        margin=0.001,
        name="swivel anchor hangs on the canopy centerline",
    )

    # --- 4-point bridle suspension -------------------------------------------
    bridle_count = sum(1 for v in ring.visuals if (v.name or "").startswith("bridle_chain_"))
    ctx.check(
        "four bridle chains rise from the ring top rim",
        bridle_count == 4,
        details=f"bridle_chain count={bridle_count}",
    )

    anchor_count = sum(1 for v in ring.visuals if (v.name or "").startswith("bridle_anchor_"))
    ctx.check(
        "four bridle anchor brackets on the ring top rim",
        anchor_count == 4,
        details=f"bridle_anchor count={anchor_count}",
    )

    # Each bridle chain converges at the swivel eye.
    for i in range(4):
        ctx.expect_contact(
            swivel, ring,
            elem_a="convergence_eye", elem_b=f"bridle_chain_{i}",
            contact_tol=0.020,
            name=f"bridle chain {i} reaches the convergence eye",
        )

    # Bridle anchors sit on the twin hoops (upper arc).
    for i in range(4):
        ctx.expect_contact(
            ring, ring,
            elem_a=f"bridle_anchor_{i}", elem_b="hoop_0",
            contact_tol=0.002,
            name=f"bridle anchor {i} contacts hoop 0",
        )

    # --- pergola frame hero features -----------------------------------------
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
        frame, frame,
        axes="z",
        elem_a="fin_0", elem_b="top_rail",
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

    # --- twin-hoop ring with wooden bench arc --------------------------------
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
        "seat planks span across both hoops",
        bottom_plank is not None
        and hoop_0 is not None
        and hoop_1 is not None
        and bottom_plank[0][1] < hoop_0[0][1] + 0.02
        and bottom_plank[1][1] > hoop_1[1][1] - 0.02,
        details=f"seat_plank_11={bottom_plank}",
    )

    # --- swing articulation (fore-aft about the beam axis) -------------------
    # Use hoop center (not part origin) since the ring origin is at the
    # convergence point near the swing pivot.
    rest_hoop = ctx.part_element_world_aabb(ring, elem="hoop_0")
    rest_y = (rest_hoop[0][1] + rest_hoop[1][1]) / 2.0
    rest_z = (rest_hoop[0][2] + rest_hoop[1][2]) / 2.0

    with ctx.pose({swing: math.radians(45.0)}):
        fwd_hoop = ctx.part_element_world_aabb(ring, elem="hoop_0")
    fwd_y = (fwd_hoop[0][1] + fwd_hoop[1][1]) / 2.0
    fwd_z = (fwd_hoop[0][2] + fwd_hoop[1][2]) / 2.0

    with ctx.pose({swing: -math.radians(45.0)}):
        back_hoop = ctx.part_element_world_aabb(ring, elem="hoop_0")
    back_y = (back_hoop[0][1] + back_hoop[1][1]) / 2.0

    ctx.check(
        "positive swing carries the ring forward and upward",
        fwd_y > rest_y + 0.30 and fwd_z > rest_z + 0.10,
        details=f"rest=({rest_y:.3f},{rest_z:.3f}), fwd=({fwd_y:.3f},{fwd_z:.3f})",
    )
    ctx.check(
        "negative swing carries the ring backward",
        back_y < rest_y - 0.30,
        details=f"rest_y={rest_y:.3f}, back_y={back_y:.3f}",
    )

    # --- continuous spin about the vertical axis -----------------------------
    with ctx.pose({spin: math.pi / 2.0}):
        spun_hoop = ctx.part_element_world_aabb(ring, elem="hoop_0")
    ctx.check(
        "quarter spin turns the ring plane about the vertical axis",
        spun_hoop is not None
        and spun_hoop[1][1] - spun_hoop[0][1] > 1.2
        and spun_hoop[1][0] - spun_hoop[0][0] < 0.3,
        details=f"spun hoop_0={spun_hoop}",
    )

    return ctx.report()


object_model = build_object_model()
