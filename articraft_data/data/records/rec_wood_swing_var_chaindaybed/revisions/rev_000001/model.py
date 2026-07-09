from __future__ import annotations

import math

# Wooden pergola garden swing daybed with chain suspension.
#
# Frame convention:
#   X = front/back (the swing travels in X)
#   Y = left/right (width, parallel to the top suspension beam)
#   Z = up
#
# Root part = fixed pergola: four corner posts, a top beam frame, slatted
# pergola roof rafters, and slatted side screens between the posts.
# Four chain parts (chain_link_0..3), each a loop of identical oval links
# built from a shared TorusGeometry helper, hang from the central suspension
# beam.  The daybed (seat frame + slats + cushions) is fixed to chain_link_0.
# Chains 1-3 mimic chain_link_0 so the whole seat swings as one pendulum.

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)

# --- Overall dimensions (meters) ---------------------------------------------
POST = 0.10          # square post cross-section
HALF_X = 1.00        # half footprint front/back -> 2.0 m deep
HALF_Y = 1.10        # half footprint left/right -> 2.2 m wide
POST_TOP = 2.05      # top of the corner posts (z)
BEAM_H = 0.14        # top beam height
BEAM_W = 0.08        # top beam thickness
BEAM_Z = POST_TOP + BEAM_H / 2.0  # beam centerline z

# Suspension / pendulum
HANG_BEAM_X = 0.0          # central suspension beam carries the swing
PIVOT_Z = POST_TOP          # pivot at the suspension beam underside

# Chain link geometry (shared helper parameters)
LINK_LEN = 0.060      # outer length along long axis (m)
LINK_WID = 0.030      # outer width along short axis (m)
WIRE_R = 0.005        # wire cross-section radius (m)
LINK_PITCH = 0.048    # target center-to-center distance between links (m)


# --------------------------------------------------------------------------- #
# Chain link helper                                                             #
# --------------------------------------------------------------------------- #

def _chain_link_meshes():
    """Build two chain-link mesh variants (alternating 90° orientations).

    Both are oval rings built from a shared TorusGeometry helper, scaled to
    produce an oblong link.  Variant A has the ring plane in XZ (long axis
    along Z); variant B has the ring plane in YZ (long axis along Z).
    Adjacent chain links alternate A/B so they interlock like a real chain.
    """
    R = LINK_WID / 2.0 - WIRE_R   # torus major radius
    sx = LINK_LEN / LINK_WID       # oval stretch factor

    # Orientation A: ring in XZ plane, long axis along Z
    ga = TorusGeometry(radius=R, tube=WIRE_R, radial_segments=8, tubular_segments=24)
    ga.scale(sx, 1.0, 1.0)
    ga.rotate_x(math.pi / 2.0)
    ga.rotate_y(math.pi / 2.0)
    mesh_a = mesh_from_geometry(ga, "chain_link_a")

    # Orientation B: ring in YZ plane, long axis along Z
    gb = TorusGeometry(radius=R, tube=WIRE_R, radial_segments=8, tubular_segments=24)
    gb.scale(sx, 1.0, 1.0)
    gb.rotate_y(math.pi / 2.0)
    mesh_b = mesh_from_geometry(gb, "chain_link_b")

    return mesh_a, mesh_b


# --------------------------------------------------------------------------- #
# Model construction                                                           #
# --------------------------------------------------------------------------- #

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pergola_garden_swing_daybed")

    # Materials
    model.material("timber", rgba=(0.74, 0.57, 0.36, 1.0))
    model.material("timber_dark", rgba=(0.62, 0.45, 0.27, 1.0))
    model.material("chain_steel", rgba=(0.50, 0.52, 0.55, 1.0))
    model.material("cushion", rgba=(0.86, 0.80, 0.68, 1.0))

    # ================================================================ pergola
    pergola = model.part("pergola_frame")

    # Four corner posts.
    post_xy = [
        (-HALF_X, -HALF_Y),
        (-HALF_X, HALF_Y),
        (HALF_X, -HALF_Y),
        (HALF_X, HALF_Y),
    ]
    for i, (px, py) in enumerate(post_xy):
        pergola.visual(
            Box((POST, POST, POST_TOP)),
            origin=Origin(xyz=(px, py, POST_TOP / 2.0)),
            material="timber",
            name=f"post_{i}",
        )

    # Top beam frame: two long beams along X (left/right) + two cross beams
    # along Y (front/back) sitting on top of the posts.
    beam_len_x = 2.0 * HALF_X + POST
    beam_len_y = 2.0 * HALF_Y + POST
    for i, py in enumerate((-HALF_Y, HALF_Y)):
        pergola.visual(
            Box((beam_len_x, BEAM_W, BEAM_H)),
            origin=Origin(xyz=(0.0, py, BEAM_Z)),
            material="timber_dark",
            name=f"side_beam_{i}",
        )
    for i, px in enumerate((-HALF_X, HALF_X)):
        pergola.visual(
            Box((BEAM_W, beam_len_y, BEAM_H)),
            origin=Origin(xyz=(px, 0.0, BEAM_Z)),
            material="timber_dark",
            name=f"cross_beam_{i}",
        )

    # Central suspension beam (along Y) under the rafters; the chains hang
    # from this beam so the swing pivots about its centerline.
    pergola.visual(
        Box((BEAM_W + 0.04, beam_len_y, BEAM_H + 0.02)),
        origin=Origin(xyz=(HANG_BEAM_X, 0.0, BEAM_Z)),
        material="timber_dark",
        name="suspension_beam",
    )

    # Corner knee braces.
    brace_run = 0.34
    brace_sec = 0.06
    brace_len = math.hypot(brace_run, brace_run)
    brace_z = POST_TOP - brace_run / 2.0 - 0.02
    for i, (px, py) in enumerate(post_xy):
        sx = -1.0 if px > 0 else 1.0
        pitch = math.atan2(sx * brace_run, brace_run)
        pergola.visual(
            Box((brace_sec, brace_sec, brace_len)),
            origin=Origin(
                xyz=(px + sx * brace_run / 2.0, py, brace_z),
                rpy=(0.0, pitch, 0.0),
            ),
            material="timber_dark",
            name=f"knee_brace_{i}",
        )

    # Slatted pergola roof rafters running across the top (along X).
    raft_z = BEAM_Z + BEAM_H / 2.0 + 0.035
    n_raft = 13
    raft_w = 0.05
    raft_h = 0.07
    for i in range(n_raft):
        ry = -HALF_Y + (i + 0.5) * (2.0 * HALF_Y) / n_raft
        pergola.visual(
            Box((beam_len_x + 0.20, raft_w, raft_h)),
            origin=Origin(xyz=(0.0, ry, raft_z)),
            material="timber",
            name=f"rafter_{i}",
        )

    # Slatted side screens between the posts on the left and right faces.
    n_slat = 9
    slat_h = 0.06
    slat_t = 0.025
    screen_top = POST_TOP - 0.10
    screen_bot = 0.55
    span = screen_top - screen_bot
    slat_len_y = 2.0 * HALF_Y - POST
    for side_i, px in enumerate((-HALF_X, HALF_X)):
        for j in range(n_slat):
            sz = screen_bot + (j + 0.5) * span / n_slat
            pergola.visual(
                Box((slat_t, slat_len_y, slat_h)),
                origin=Origin(xyz=(px, 0.0, sz)),
                material="timber",
                name=f"screen_slat_{side_i}_{j}",
            )

    # ====================================================== chain suspension
    mesh_a, mesh_b = _chain_link_meshes()

    # Seat corner offsets (in chain / daybed frame, relative to pivot)
    seat_cx = 0.0
    rod_x_front = seat_cx + 0.50
    rod_x_rear = seat_cx - 0.50
    rod_y = HALF_Y - POST - 0.10
    seat_top_z = -0.90

    # Each chain hangs from the beam to one seat corner.
    # Rear chains are offset inward on the beam to avoid top-link overlap
    # with the front chains (both would otherwise share the same beam Y).
    CHAIN_Y_INSET = 0.04
    # (bottom_x, beam_y, seat_y, bottom_z) — all in chain part frame coords
    # except beam_y which is the joint origin Y in the pergola frame.
    chain_specs = [
        (rod_x_front,  rod_y,                   rod_y,  seat_top_z),   # 0: front-right
        (rod_x_front, -rod_y,                  -rod_y,  seat_top_z),   # 1: front-left
        (rod_x_rear,   rod_y - CHAIN_Y_INSET,   rod_y,  seat_top_z),   # 2: rear-right
        (rod_x_rear,  -rod_y + CHAIN_Y_INSET,  -rod_y,  seat_top_z),   # 3: rear-left
    ]

    chains = []
    for i in range(4):
        chain = model.part(f"chain_link_{i}")
        chains.append(chain)

        bottom_x, beam_y, seat_y, bottom_z = chain_specs[i]
        # In the chain part frame (origin at beam attachment point):
        bx = bottom_x   # HANG_BEAM_X = 0
        by = seat_y - beam_y
        bz = bottom_z   # relative to PIVOT_Z
        chain_len = math.sqrt(bx * bx + by * by + bz * bz)
        n_links = max(3, round(chain_len / LINK_PITCH))
        # Pitch angle to align link long axis with chain direction (XZ projection)
        pitch_angle = math.atan2(-bx, -bz)

        for j in range(n_links):
            t = (j + 0.5) / n_links
            cx = t * bx
            cy = t * by
            cz = t * bz
            link_mesh = mesh_a if j % 2 == 0 else mesh_b
            chain.visual(
                link_mesh,
                origin=Origin(xyz=(cx, cy, cz), rpy=(0.0, pitch_angle, 0.0)),
                material="chain_steel",
                name=f"link_{j}",
            )

    # =============================================================== daybed
    # The daybed part frame matches the original pivot frame so all seat
    # geometry coordinates are unchanged from the parent.
    daybed = model.part("swing_daybed")

    frame_t = 0.06
    seat_w_y = 2.0 * rod_y + 0.12
    seat_d_x = (rod_x_front - rod_x_rear) + 0.14
    frame_z = seat_top_z + frame_t / 2.0

    # Front & rear rails (along Y)
    for i, rx in enumerate((rod_x_front, rod_x_rear)):
        daybed.visual(
            Box((frame_t, seat_w_y, frame_t)),
            origin=Origin(xyz=(rx, 0.0, frame_z)),
            material="timber_dark",
            name=f"seat_rail_x_{i}",
        )
    # Side rails (along X)
    for i, ry in enumerate((rod_y + 0.06, -(rod_y + 0.06))):
        daybed.visual(
            Box((seat_d_x, frame_t, frame_t)),
            origin=Origin(xyz=(seat_cx, ry, frame_z)),
            material="timber_dark",
            name=f"seat_rail_y_{i}",
        )

    # Seat base slats spanning across (along Y), spaced in X.
    n_seat_slat = 9
    sslat_t = 0.022
    sslat_h = 0.05
    slat_span = rod_x_front - rod_x_rear
    slat_z = seat_top_z + sslat_h / 2.0
    for i in range(n_seat_slat):
        sx = rod_x_rear + (i + 0.5) * slat_span / n_seat_slat
        daybed.visual(
            Box((sslat_t, seat_w_y - 0.06, sslat_h)),
            origin=Origin(xyz=(sx, 0.0, slat_z)),
            material="timber",
            name=f"seat_slat_{i}",
        )

    # Backrest frame (rear, vertical-ish) and slats.
    back_h = 0.55
    back_z0 = seat_top_z + frame_t
    back_x = rod_x_rear + 0.02
    n_back_slat = 6
    bslat_t = 0.022
    bslat_h = 0.06
    for i in range(n_back_slat):
        bz = back_z0 + (i + 0.5) * back_h / n_back_slat
        daybed.visual(
            Box((bslat_t, seat_w_y - 0.06, bslat_h)),
            origin=Origin(xyz=(back_x, 0.0, bz)),
            material="timber",
            name=f"back_slat_{i}",
        )

    # Seat cushion (beige), thick base + inset crown.
    cush_w_y = seat_w_y - 0.08
    cush_base_t = 0.10
    cush_crown_t = 0.07
    cush_x = seat_cx + 0.05
    cush_d = slat_span - 0.05
    seat_cush_z0 = seat_top_z + sslat_h
    daybed.visual(
        Box((cush_d, cush_w_y, cush_base_t)),
        origin=Origin(xyz=(cush_x, 0.0, seat_cush_z0 + cush_base_t / 2.0)),
        material="cushion",
        name="seat_cushion",
    )
    daybed.visual(
        Box((cush_d - 0.06, cush_w_y - 0.06, cush_crown_t)),
        origin=Origin(
            xyz=(cush_x, 0.0, seat_cush_z0 + cush_base_t + cush_crown_t / 2.0 - 0.01),
        ),
        material="cushion",
        name="seat_cushion_crown",
    )

    # Back pillows.
    bcush_t = 0.13
    bcush_h = back_h - 0.04
    n_back_cush = 3
    bcush_face_x = back_x + bslat_t / 2.0 + bcush_t / 2.0
    pillow_w = (seat_w_y - 0.12) / n_back_cush
    for i in range(n_back_cush):
        cy = -(seat_w_y - 0.12) / 2.0 + (i + 0.5) * pillow_w
        daybed.visual(
            Box((bcush_t, pillow_w - 0.03, bcush_h)),
            origin=Origin(xyz=(bcush_face_x, cy, back_z0 + bcush_h / 2.0 + 0.03)),
            material="cushion",
            name=f"back_cushion_{i}",
        )

    # End bolster cushions.
    bol_d = cush_d * 0.55
    bol_w = 0.14
    bol_h = 0.16
    for i, by in enumerate((cush_w_y / 2.0 - bol_w / 2.0, -(cush_w_y / 2.0 - bol_w / 2.0))):
        daybed.visual(
            Box((bol_d, bol_w, bol_h)),
            origin=Origin(
                xyz=(cush_x, by, seat_cush_z0 + cush_base_t + bol_h / 2.0),
            ),
            material="cushion",
            name=f"bolster_{i}",
        )

    # ========================================================= articulations

    # Primary chain pivot: chain_link_0 revolute on the suspension beam.
    _, beam_y_0, _, _ = chain_specs[0]
    model.articulation(
        "beam_to_chain_0",
        ArticulationType.REVOLUTE,
        parent=pergola,
        child=chains[0],
        origin=Origin(xyz=(HANG_BEAM_X, beam_y_0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=2.0, lower=-0.35, upper=0.35),
    )

    # Other three chains mimic the primary pivot.
    for i in range(1, 4):
        _, beam_y_i, _, _ = chain_specs[i]
        model.articulation(
            f"beam_to_chain_{i}",
            ArticulationType.REVOLUTE,
            parent=pergola,
            child=chains[i],
            origin=Origin(xyz=(HANG_BEAM_X, beam_y_i, PIVOT_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=80.0, velocity=2.0, lower=-0.35, upper=0.35),
            mimic=Mimic(joint="beam_to_chain_0", multiplier=1.0, offset=0.0),
        )

    # Daybed fixed to chain_link_0.  The offset maps chain_0's frame (at the
    # beam, y=+rod_y) back to the original daybed pivot frame (y=0) so all
    # seat geometry stays unchanged.
    model.articulation(
        "chain_to_daybed",
        ArticulationType.FIXED,
        parent=chains[0],
        child=daybed,
        origin=Origin(xyz=(0.0, -rod_y, 0.0)),
    )

    return model


object_model = build_object_model()


# --------------------------------------------------------------------------- #
# Tests                                                                        #
# --------------------------------------------------------------------------- #

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pergola = object_model.get_part("pergola_frame")
    daybed = object_model.get_part("swing_daybed")
    swing = object_model.get_articulation("beam_to_chain_0")

    chains = [object_model.get_part(f"chain_link_{i}") for i in range(4)]

    # ---- Chain structure checks ----

    # Each chain has multiple interlocking links.
    for i in range(4):
        n_links = sum(
            1 for v in chains[i].visuals if v.name and v.name.startswith("link_")
        )
        ctx.check(
            f"chain_link_{i} has multiple links",
            n_links >= 8,
            details=f"links={n_links}",
        )

    # All four chain parts exist with revolute (non-fixed) joints.
    for i in range(4):
        art_name = f"beam_to_chain_{i}"
        art = object_model.get_articulation(art_name)
        ctx.check(
            f"{art_name} is revolute (non-fixed pivot)",
            art.articulation_type == ArticulationType.REVOLUTE,
            details=f"type={art.articulation_type}",
        )

    # Chains 1-3 mimic chain 0.
    for i in range(1, 4):
        art = object_model.get_articulation(f"beam_to_chain_{i}")
        ctx.check(
            f"beam_to_chain_{i} mimics beam_to_chain_0",
            art.mimic is not None and art.mimic.joint == "beam_to_chain_0",
        )

    # ---- Overlap allowances ----
    # Top links of each chain hook over the suspension beam as a pivot mount.
    for i in range(4):
        ctx.allow_overlap(
            pergola,
            chains[i],
            elem_a="suspension_beam",
            elem_b="link_0",
            reason="Top chain link hooks over the suspension beam as a pivot mount.",
        )
    # Bottom links of each chain connect to the daybed seat frame. The last
    # few links pass through the cushion/rail zone as they reach the frame.
    for i in range(4):
        ctx.allow_overlap(
            chains[i],
            daybed,
            reason="Chain bottom links connect to the daybed seat frame, passing near the cushion edge.",
        )

    # Prove the top link contacts the beam (pivot mount).
    ctx.expect_contact(
        pergola,
        chains[0],
        elem_a="suspension_beam",
        elem_b="link_0",
        contact_tol=0.015,
        name="chain 0 top link contacts suspension beam",
    )

    # ---- Hero geometry checks (unchanged from parent) ----
    n_rafters = sum(1 for v in pergola.visuals if v.name and v.name.startswith("rafter_"))
    ctx.check("pergola has multiple roof rafters", n_rafters >= 10, details=f"rafters={n_rafters}")

    n_screen = sum(1 for v in pergola.visuals if v.name and v.name.startswith("screen_slat_"))
    ctx.check("side screens are slatted", n_screen >= 12, details=f"screen_slats={n_screen}")

    n_posts = sum(1 for v in pergola.visuals if v.name and v.name.startswith("post_"))
    ctx.check("four corner posts", n_posts == 4, details=f"posts={n_posts}")

    n_brace = sum(1 for v in pergola.visuals if v.name and v.name.startswith("knee_brace_"))
    ctx.check("pergola has corner knee braces", n_brace == 4, details=f"braces={n_brace}")

    n_seat_slat = sum(1 for v in daybed.visuals if v.name and v.name.startswith("seat_slat_"))
    ctx.check("seat is slatted", n_seat_slat >= 6, details=f"seat_slats={n_seat_slat}")

    n_back_slat = sum(1 for v in daybed.visuals if v.name and v.name.startswith("back_slat_"))
    ctx.check("backrest is slatted", n_back_slat >= 5, details=f"back_slats={n_back_slat}")

    # Cushions
    n_back_cush = sum(1 for v in daybed.visuals if v.name and v.name.startswith("back_cushion_"))
    n_bolster = sum(1 for v in daybed.visuals if v.name and v.name.startswith("bolster_"))
    ctx.check(
        "plump beige seat cushion present",
        daybed.get_visual("seat_cushion") is not None
        and daybed.get_visual("seat_cushion_crown") is not None,
    )
    ctx.check("multiple back pillows", n_back_cush >= 3, details=f"back_cushions={n_back_cush}")
    ctx.check("end bolster cushions present", n_bolster == 2, details=f"bolsters={n_bolster}")

    # ---- Scale and ground checks ----
    pb = ctx.part_world_aabb(pergola)
    if pb is not None:
        (xmn, ymn, zmn), (xmx, ymx, zmx) = pb
        ctx.check(
            "pergola is full-size (>~2 m wide and >~2 m tall)",
            (ymx - ymn) > 2.0 and (zmx - zmn) > 2.0,
            details=f"width={ymx - ymn:.2f} height={zmx - zmn:.2f}",
        )
        ctx.check(
            "pergola rests on the ground (zmin ~ 0)",
            abs(zmn) < 0.02,
            details=f"zmin={zmn:.3f}",
        )

    # ---- Daybed hangs below roof and above floor ----
    with ctx.pose({swing: 0.0}):
        db = ctx.part_world_aabb(daybed)
    if db is not None:
        (_, _, dzmn), (_, _, dzmx) = db
        ctx.check(
            "daybed hangs below pergola roof",
            dzmx < BEAM_Z and dzmn > 0.0,
            details=f"daybed_top={dzmx:.2f} beam={BEAM_Z:.2f}",
        )

    # Daybed clears the floor across the full swing range.
    floor_clear = True
    for q in (-0.35, 0.35):
        with ctx.pose({swing: q}):
            qb = ctx.part_world_aabb(daybed)
        if qb is not None and qb[0][2] <= 0.0:
            floor_clear = False
    ctx.check("daybed clears the floor through its full swing", floor_clear)

    # ---- Pendulum motion: seat displaces fore/aft ----
    def _cx(aabb):
        return None if aabb is None else (aabb[0][0] + aabb[1][0]) / 2.0

    with ctx.pose({swing: 0.30}):
        fwd = _cx(ctx.part_world_aabb(daybed))
    with ctx.pose({swing: -0.30}):
        bwd = _cx(ctx.part_world_aabb(daybed))
    if fwd is not None and bwd is not None:
        ctx.check(
            "swing rotates (seat displaces fore/aft)",
            abs(fwd - bwd) > 0.10,
            details=f"fwd_cx={fwd:.3f} bwd_cx={bwd:.3f}",
        )

    # ---- No rigid rods remain (replaced by chains) ----
    n_rods = sum(1 for v in daybed.visuals if v.name and v.name.startswith("suspension_rod_"))
    ctx.check("rigid suspension rods removed (replaced by chains)", n_rods == 0, details=f"rods={n_rods}")

    return ctx.report()
