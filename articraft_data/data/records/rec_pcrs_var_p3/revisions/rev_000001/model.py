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
# Modern circular ring swing hanging from a steel pergola frame.
# 3-point bridle variant: three chains rise from equally-spaced anchor eyes
# on the ring top rim to a single overhead swivel/hook on the canopy.
#
# World frame: Z up, ground at z = 0, X spans the 3.0 m frame width,
# Y is the swing (fore-aft) direction.
#
# Layout summary (meters):
# - posts:      0.15 sq tube, x = +-1.425, z 0 .. 2.555 (embed into top rail)
# - top rail:   3.00 x 0.28 x 0.05, z 2.55 .. 2.60
# - louver fins: 34 fins, 0.04 thick x 0.25 deep x 0.13 tall, z 2.425 .. 2.555
# - swing pivot: (0, 0, 2.55), revolute about +X, -45 .. +45 deg
# - swivel:     stem into rail + shaft + barrel + cap at z_local = -SWIVEL_DROP
# - spin pivot: swivel cap bottom, z_local = -SWIVEL_DROP, continuous about +Z
# - bridle:     3 chains from ring top-rim anchors converging at spin pivot
# - ring:       twin chrome hoops OD 1.5 (R 0.725, tube 0.025) at y = +-0.0375,
#               wooden seat planks lining the lower inner arc
# ---------------------------------------------------------------------------

# Frame dimensions
POST_X = 1.425
POST_SIZE = 0.15
POST_TOP = 2.555
RAIL_Z0, RAIL_Z1 = 2.55, 2.60
FIN_Z0, FIN_Z1 = 2.425, 2.555
FIN_COUNT = 34
FIN_SPAN = 2.94  # fin centres from -1.47 to +1.47

# Swivel / bridle layout
SWIVEL_DROP = 0.20          # spin-joint z below rail underside
STEM_LEN = 0.09             # eye-bolt stem socketed into top rail
SHAFT_R = 0.008             # connecting shaft radius
BARREL_R = 0.022            # swivel barrel radius
BARREL_H = 0.055            # swivel barrel height
CAP_R = 0.032               # swivel cap plate radius
CAP_H = 0.012               # swivel cap plate thickness

# Bridle chain geometry
BRIDLE_N = 3
BRIDLE_ANGLES_DEG = (30.0, 90.0, 150.0)   # equally-spaced on top rim
CHAIN_STOCK_R = 0.0055      # chain rod radius
CONVERGENCE_Z = 0.008       # chain convergence above ring origin, into cap

# Ring layout (ring-local frame, origin at the spin joint / convergence point)
HOOP_R = 0.725
HOOP_TUBE = 0.025
HOOP_Y = 0.0375
RING_C = -1.27              # hoop circle centre, ring-local z
PLANK_RC = 0.693            # plank centre radius (outer face embeds 2 mm into hoop bore)
PLANK_COUNT = 22
PLANK_THETA0 = math.radians(197.0)
PLANK_THETA1 = math.radians(343.0)


def _bridle_anchor(theta_rad: float) -> tuple[float, float, float]:
    """Ring-local position of a bridle anchor eye at angle theta on the hoop."""
    return (
        HOOP_R * math.cos(theta_rad),
        0.0,
        RING_C + HOOP_R * math.sin(theta_rad),
    )


def _chain_tube(anchor: tuple[float, float, float], mesh_name: str) -> Mesh:
    """Straight chain tube from a ring anchor to the convergence point."""
    n_pts = 5
    pts = [
        (
            anchor[0] * (1.0 - t),
            anchor[1] * (1.0 - t),
            anchor[2] + (CONVERGENCE_Z - anchor[2]) * t,
        )
        for t in (i / (n_pts - 1) for i in range(n_pts))
    ]
    return mesh_from_geometry(
        tube_from_spline_points(
            pts,
            radius=CHAIN_STOCK_R,
            samples_per_segment=8,
            radial_segments=12,
            cap_ends=True,
        ),
        mesh_name,
    )


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

    # --------------------------------------------------------- bridle swivel
    swivel = model.part("bridle_swivel")

    # Eye-bolt stem socketed up into the top rail (z -0.06 .. +0.03 local).
    swivel.visual(
        Cylinder(radius=0.012, length=STEM_LEN),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=chrome,
        name="anchor_stem",
    )

    # Connecting shaft from stem bottom down to barrel top.
    barrel_z_center = -SWIVEL_DROP + BARREL_H / 2.0 + CAP_H
    barrel_top = barrel_z_center + BARREL_H / 2.0
    stem_bottom = -0.015 - STEM_LEN / 2.0
    shaft_len = stem_bottom - barrel_top
    shaft_center = (stem_bottom + barrel_top) / 2.0
    swivel.visual(
        Cylinder(radius=SHAFT_R, length=shaft_len),
        origin=Origin(xyz=(0.0, 0.0, shaft_center)),
        material=chrome,
        name="swivel_shaft",
    )

    # Swivel barrel between shaft and cap.
    swivel.visual(
        Cylinder(radius=BARREL_R, length=BARREL_H),
        origin=Origin(xyz=(0.0, 0.0, barrel_z_center)),
        material=chrome,
        name="swivel_barrel",
    )

    # Cap plate at the bottom of the swivel (convergence flange).
    cap_z_center = -SWIVEL_DROP + CAP_H / 2.0
    swivel.visual(
        Cylinder(radius=CAP_R, length=CAP_H),
        origin=Origin(xyz=(0.0, 0.0, cap_z_center)),
        material=chrome,
        name="swivel_cap",
    )

    # Three small bolt heads on the cap top surface mark where each
    # bridle chain through-bolts into the convergence flange.
    bolt_r = 0.008
    bolt_h = 0.005
    for i in range(BRIDLE_N):
        theta = math.radians(BRIDLE_ANGLES_DEG[i])
        bx = 0.018 * math.cos(theta)
        bz = -SWIVEL_DROP + CAP_H + bolt_h / 2.0
        swivel.visual(
            Cylinder(radius=bolt_r, length=bolt_h),
            origin=Origin(xyz=(bx, 0.0, bz)),
            material=rivet_steel,
            name=f"chain_bolt_{i}",
        )

    # --------------------------------------------------------------- ring
    ring = model.part("ring_seat")

    # Three anchor eyes on the ring top rim (bridle attachment points).
    anchor_eye_r = 0.015
    anchor_eye_tube = 0.005
    for i in range(BRIDLE_N):
        theta = math.radians(BRIDLE_ANGLES_DEG[i])
        ax, ay, az = _bridle_anchor(theta)
        ae_geom = TorusGeometry(anchor_eye_r, anchor_eye_tube,
                                radial_segments=12, tubular_segments=32)
        ae_geom.rotate_x(math.pi / 2.0)  # ring plane XZ, hole axis Y
        ring.visual(
            mesh_from_geometry(ae_geom, f"anchor_eye_{i}"),
            origin=Origin(xyz=(ax, ay, az)),
            material=chrome,
            name=f"anchor_eye_{i}",
        )

    # Three bridle chains: each runs from its anchor eye up to the
    # convergence point (ring-local z = CONVERGENCE_Z, inside the swivel cap).
    for i in range(BRIDLE_N):
        theta = math.radians(BRIDLE_ANGLES_DEG[i])
        anchor = _bridle_anchor(theta)
        ring.visual(
            _chain_tube(anchor, f"bridle_chain_{i}"),
            material=chrome,
            name=f"bridle_chain_{i}",
        )

    # Twin chrome hoops (open ring, unchanged from parent).
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
        origin=Origin(xyz=(0.0, 0.0, -SWIVEL_DROP)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("pergola_frame")
    swivel = object_model.get_part("bridle_swivel")
    ring = object_model.get_part("ring_seat")
    swing = object_model.get_articulation("canopy_swing")
    spin = object_model.get_articulation("ring_spin")

    # Intentional local embeds that make the mechanism read as real hardware.
    ctx.allow_overlap(
        frame,
        swivel,
        elem_a="top_rail",
        elem_b="anchor_stem",
        reason="The swivel anchor stem is an eye-bolt shank intentionally socketed into the canopy top rail.",
    )

    # Each bridle chain terminates inside the swivel cap plate, representing
    # a through-bolted convergence fitting.
    for i in range(BRIDLE_N):
        ctx.allow_overlap(
            swivel,
            ring,
            elem_a="swivel_cap",
            elem_b=f"bridle_chain_{i}",
            reason=f"Bridle chain {i} terminates inside the swivel cap plate, representing a through-bolted convergence fitting.",
        )

    # --- swivel anchored on the canopy centreline --------------------------
    ctx.expect_overlap(
        swivel,
        frame,
        axes="z",
        elem_a="anchor_stem",
        elem_b="top_rail",
        min_overlap=0.02,
        name="swivel anchor stem is socketed into the canopy top rail",
    )
    ctx.expect_within(
        swivel,
        frame,
        axes="xy",
        inner_elem="anchor_stem",
        outer_elem="top_rail",
        margin=0.001,
        name="swivel anchor hangs on the canopy centreline",
    )

    # --- bridle: 3 chains converge from ring top rim to swivel -------------
    chain_count = sum(
        1 for v in ring.visuals if (v.name or "").startswith("bridle_chain_")
    )
    ctx.check(
        "three bridle chains rise from the ring",
        chain_count == BRIDLE_N,
        details=f"bridle_chain count={chain_count}",
    )

    anchor_count = sum(
        1 for v in ring.visuals if (v.name or "").startswith("anchor_eye_")
    )
    ctx.check(
        "three anchor eyes on the ring top rim",
        anchor_count == BRIDLE_N,
        details=f"anchor_eye count={anchor_count}",
    )

    # All anchor eyes sit above the ring centre (top rim).
    ring_center_z = RAIL_Z0 - SWIVEL_DROP + RING_C
    for i in range(BRIDLE_N):
        ae = ctx.part_element_world_aabb(ring, elem=f"anchor_eye_{i}")
        ctx.check(
            f"anchor_eye_{i} is on the top rim (above ring centre)",
            ae is not None and (ae[0][2] + ae[1][2]) / 2.0 > ring_center_z + 0.10,
            details=f"anchor_eye_{i}={ae}, ring_center_z={ring_center_z}",
        )

    # Each bridle chain converges at the swivel cap (overlap in z).
    for i in range(BRIDLE_N):
        ctx.expect_overlap(
            ring,
            swivel,
            axes="z",
            elem_a=f"bridle_chain_{i}",
            elem_b="swivel_cap",
            min_overlap=0.003,
            name=f"bridle_chain_{i} converges at the swivel cap",
        )

    # The 3 chains spread across the ring width (distinct x positions).
    chain_xs = []
    for i in range(BRIDLE_N):
        ca = ctx.part_element_world_aabb(ring, elem=f"bridle_chain_{i}")
        if ca is not None:
            chain_xs.append((ca[0][0] + ca[1][0]) / 2.0)
    ctx.check(
        "bridle chains spread across the ring width",
        len(chain_xs) == BRIDLE_N and max(chain_xs) - min(chain_xs) > 0.40,
        details=f"chain centres x={chain_xs}",
    )

    # --- pergola frame hero features ---------------------------------------
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

    # --- twin-hoop ring with wooden bench arc ------------------------------
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

    # Ring hangs clear of the ground; hoop top is below the canopy fins.
    hoop_0_top = hoop_0[1][2] if hoop_0 else None
    ctx.check(
        "ring hoop hangs below the canopy fins",
        hoop_0_top is not None and hoop_0_top < FIN_Z0,
        details=f"hoop_0 top={hoop_0_top}, fin bottom={FIN_Z0}",
    )
    ring_aabb = ctx.part_world_aabb(ring)
    ctx.check(
        "ring hangs clear of the ground",
        ring_aabb is not None and ring_aabb[0][2] > 0.15,
        details=f"ring aabb={ring_aabb}",
    )

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

    # --- swing articulation (fore-aft about the beam axis) -----------------
    # Use hoop element to measure swing displacement (part origin is at the
    # convergence point, close to the pivot; hoop center has the real lever arm).
    rest_hoop = ctx.part_element_world_aabb(ring, elem="hoop_0")
    rest_cy = (rest_hoop[0][1] + rest_hoop[1][1]) / 2.0 if rest_hoop else None
    rest_cz = (rest_hoop[0][2] + rest_hoop[1][2]) / 2.0 if rest_hoop else None

    with ctx.pose({swing: math.radians(45.0)}):
        fwd_hoop = ctx.part_element_world_aabb(ring, elem="hoop_0")
    fwd_cy = (fwd_hoop[0][1] + fwd_hoop[1][1]) / 2.0 if fwd_hoop else None
    fwd_cz = (fwd_hoop[0][2] + fwd_hoop[1][2]) / 2.0 if fwd_hoop else None

    ctx.check(
        "positive swing carries the hoop forward and upward",
        rest_cy is not None and fwd_cy is not None
        and fwd_cy > rest_cy + 0.30
        and fwd_cz is not None and rest_cz is not None
        and fwd_cz > rest_cz + 0.10,
        details=f"rest=({rest_cy}, {rest_cz}), fwd=({fwd_cy}, {fwd_cz})",
    )

    with ctx.pose({swing: -math.radians(45.0)}):
        back_hoop = ctx.part_element_world_aabb(ring, elem="hoop_0")
    back_cy = (back_hoop[0][1] + back_hoop[1][1]) / 2.0 if back_hoop else None
    ctx.check(
        "negative swing carries the hoop backward",
        rest_cy is not None and back_cy is not None and back_cy < rest_cy - 0.30,
        details=f"rest_cy={rest_cy}, back_cy={back_cy}",
    )

    # --- continuous spin about the vertical chain axis ----------------------
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
