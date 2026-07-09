from __future__ import annotations

import math

# Wooden pergola garden swing daybed.
#
# Frame convention:
#   X = front/back (the swing travels in X)
#   Y = left/right (width, parallel to the top suspension beam)
#   Z = up
#
# Root part = fixed pergola: four corner posts, a top beam frame, slatted
# pergola roof rafters, and slatted side screens between the posts.
# Child part = the cushioned daybed (seat frame + base slats + cushions +
# four suspension rods) hung from the front top beam, swinging as a pendulum
# about the Y axis (parallel to the beam).

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
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
PIVOT_Z = POST_TOP        # pivot at the suspension beam underside
ROD_LEN = 0.95            # suspension rod length


def _wood(model: ArticulatedObject) -> None:
    model.material("timber", rgba=(0.74, 0.57, 0.36, 1.0))
    model.material("timber_dark", rgba=(0.62, 0.45, 0.27, 1.0))
    model.material("steel_rod", rgba=(0.55, 0.56, 0.58, 1.0))
    model.material("cushion", rgba=(0.86, 0.80, 0.68, 1.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pergola_garden_swing_daybed")
    _wood(model)

    # ---------------------------------------------------------------- pergola
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

    # Central suspension beam (along Y) under the rafters; the daybed rods hang
    # from this beam so the swing pivots about its centerline.
    pergola.visual(
        Box((BEAM_W + 0.04, beam_len_y, BEAM_H + 0.02)),
        origin=Origin(xyz=(HANG_BEAM_X, 0.0, BEAM_Z)),
        material="timber_dark",
        name="suspension_beam",
    )

    # Corner knee braces: short diagonal struts from each post up to the top
    # beam, bracing the frame as in the reference timber pergola.
    brace_run = 0.34
    brace_sec = 0.06
    brace_len = math.hypot(brace_run, brace_run)
    brace_z = POST_TOP - brace_run / 2.0 - 0.02
    for i, (px, py) in enumerate(post_xy):
        sx = -1.0 if px > 0 else 1.0  # point the brace toward frame center
        pitch = math.atan2(sx * brace_run, brace_run)  # 45 deg toward center
        pergola.visual(
            Box((brace_sec, brace_sec, brace_len)),
            origin=Origin(
                xyz=(px + sx * brace_run / 2.0, py, brace_z),
                rpy=(0.0, pitch, 0.0),
            ),
            material="timber_dark",
            name=f"knee_brace_{i}",
        )

    # Slatted pergola roof rafters running across the top (along X), resting on
    # the side beams, spaced along Y.
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
    slat_len_y = 2.0 * HALF_Y - POST  # between the two posts on a side
    for side_i, px in enumerate((-HALF_X, HALF_X)):
        for j in range(n_slat):
            sz = screen_bot + (j + 0.5) * span / n_slat
            pergola.visual(
                Box((slat_t, slat_len_y, slat_h)),
                origin=Origin(xyz=(px, 0.0, sz)),
                material="timber",
                name=f"screen_slat_{side_i}_{j}",
            )

    # ----------------------------------------------------------------- daybed
    # The daybed part frame is the pivot frame: origin at the front top beam,
    # centered in Y. Geometry hangs below (rods) and extends in +Z->down only
    # via negative offsets; the seat sits forward of the pivot.
    daybed = model.part("swing_daybed")

    # Suspension rods: four rods, all hung from the single pivot beam line
    # (x=0 in the daybed/pivot frame), tilting fore/aft down to the seat
    # corners so the whole seat hangs centered under the pivot and swings as a
    # rigid pendulum. Each rod is drawn between an exact top and bottom point.
    rod_r = 0.012
    seat_cx = 0.0          # seat centered under the pivot beam line
    rod_x_front = seat_cx + 0.50
    rod_x_rear = seat_cx - 0.50
    rod_y = HALF_Y - POST - 0.10
    seat_top_z = -0.90     # seat frame top relative to pivot

    rod_endpoints = [
        ((0.0, rod_y, 0.0), (rod_x_front, rod_y, seat_top_z)),
        ((0.0, -rod_y, 0.0), (rod_x_front, -rod_y, seat_top_z)),
        ((0.0, rod_y, 0.0), (rod_x_rear, rod_y, seat_top_z)),
        ((0.0, -rod_y, 0.0), (rod_x_rear, -rod_y, seat_top_z)),
    ]
    for i, (top, bot) in enumerate(rod_endpoints):
        dx = bot[0] - top[0]
        dz = bot[2] - top[2]
        length = math.hypot(dx, dz)
        cx = (top[0] + bot[0]) / 2.0
        cz = (top[2] + bot[2]) / 2.0
        # Cylinder local +Z must point from top->bottom. Pitch about Y so that
        # +Z maps to (dx, 0, dz). atan2 gives the rotation from +Z toward +X.
        pitch = math.atan2(dx, dz)
        daybed.visual(
            Cylinder(radius=rod_r, length=length + 0.02),
            origin=Origin(xyz=(cx, top[1], cz), rpy=(0.0, pitch, 0.0)),
            material="steel_rod",
            name=f"suspension_rod_{i}",
        )
    frame_t = 0.06
    seat_w_y = 2.0 * rod_y + 0.12
    seat_d_x = (rod_x_front - rod_x_rear) + 0.14
    frame_z = seat_top_z + frame_t / 2.0
    # front & rear rails (along Y)
    for i, rx in enumerate((rod_x_front, rod_x_rear)):
        daybed.visual(
            Box((frame_t, seat_w_y, frame_t)),
            origin=Origin(xyz=(rx, 0.0, frame_z)),
            material="timber_dark",
            name=f"seat_rail_x_{i}",
        )
    # side rails (along X)
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

    # Seat cushion (beige), thick and plump. Built as a thick base plus a
    # slightly inset rounded crown layer so it reads as a padded daybed mattress
    # rather than a flat slab.
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

    # Back: several plump throw pillows leaning against the backrest, as in the
    # reference (a row of separate cushions rather than one flat pad).
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

    # End bolster cushions: a rounded pillow at each end of the daybed seat,
    # tucked against the side rails (visible armrest bolsters in the reference).
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

    # ------------------------------------------------------------ articulation
    # Pendulum: pivot at the front top beam, axis along Y (the beam), origin in
    # the pergola frame. Positive q swings the seat forward/back.
    model.articulation(
        "pergola_to_swing",
        ArticulationType.REVOLUTE,
        parent=pergola,
        child=daybed,
        origin=Origin(xyz=(HANG_BEAM_X, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=2.0, lower=-0.35, upper=0.35),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pergola = object_model.get_part("pergola_frame")
    daybed = object_model.get_part("swing_daybed")
    swing = object_model.get_articulation("pergola_to_swing")

    # The rod tops intentionally seat into the suspension beam so the daybed
    # hangs from it. Allow that local embedding and prove the contact.
    for i in range(4):
        ctx.allow_overlap(
            pergola,
            daybed,
            elem_a="suspension_beam",
            elem_b=f"suspension_rod_{i}",
            reason="Suspension rod tops are seated into the suspension beam to hang the daybed.",
        )
    ctx.expect_contact(
        pergola,
        daybed,
        elem_a="suspension_beam",
        elem_b="suspension_rod_0",
        name="rod is seated into the suspension beam",
    )

    # Hero geometry: posts, rafters, screen slats, seat slats, cushions, rods.
    n_rafters = sum(1 for v in pergola.visuals if v.name and v.name.startswith("rafter_"))
    ctx.check("pergola has multiple roof rafters", n_rafters >= 10, details=f"rafters={n_rafters}")

    n_screen = sum(1 for v in pergola.visuals if v.name and v.name.startswith("screen_slat_"))
    ctx.check("side screens are slatted", n_screen >= 12, details=f"screen_slats={n_screen}")

    n_posts = sum(1 for v in pergola.visuals if v.name and v.name.startswith("post_"))
    ctx.check("four corner posts", n_posts == 4, details=f"posts={n_posts}")

    n_brace = sum(1 for v in pergola.visuals if v.name and v.name.startswith("knee_brace_"))
    ctx.check("pergola has corner knee braces", n_brace == 4, details=f"braces={n_brace}")

    n_rods = sum(1 for v in daybed.visuals if v.name and v.name.startswith("suspension_rod_"))
    ctx.check("daybed hung by suspension rods", n_rods == 4, details=f"rods={n_rods}")

    n_seat_slat = sum(1 for v in daybed.visuals if v.name and v.name.startswith("seat_slat_"))
    ctx.check("seat is slatted", n_seat_slat >= 6, details=f"seat_slats={n_seat_slat}")

    n_back_slat = sum(1 for v in daybed.visuals if v.name and v.name.startswith("back_slat_"))
    ctx.check("backrest is slatted", n_back_slat >= 5, details=f"back_slats={n_back_slat}")

    # Cushions: plump seat (base + crown), a row of back pillows, end bolsters.
    n_back_cush = sum(1 for v in daybed.visuals if v.name and v.name.startswith("back_cushion_"))
    n_bolster = sum(1 for v in daybed.visuals if v.name and v.name.startswith("bolster_"))
    ctx.check(
        "plump beige seat cushion present",
        daybed.get_visual("seat_cushion") is not None
        and daybed.get_visual("seat_cushion_crown") is not None,
    )
    ctx.check("multiple back pillows", n_back_cush >= 3, details=f"back_cushions={n_back_cush}")
    ctx.check("end bolster cushions present", n_bolster == 2, details=f"bolsters={n_bolster}")

    # Pergola is large outdoor furniture (real-world scale).
    pb = ctx.part_world_aabb(pergola)
    if pb is not None:
        (xmn, ymn, zmn), (xmx, ymx, zmx) = pb
        ctx.check(
            "pergola is full-size (>~2 m wide and >~2 m tall)",
            (ymx - ymn) > 2.0 and (zmx - zmn) > 2.0,
            details=f"width={ymx - ymn:.2f} height={zmx - zmn:.2f}",
        )

    # Pergola rests on the ground at z=0.
    if pb is not None:
        ctx.check(
            "pergola rests on the ground (zmin ~ 0)",
            abs(zmn) < 0.02,
            details=f"zmin={zmn:.3f}",
        )

    # Daybed hangs below the top beam (seat well below pivot) and above the
    # floor at rest.
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

    # Pendulum motion: forward swing displaces the seat body fore/aft. Measure
    # the daybed AABB center (the part origin sits on the pivot and won't move).
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

    return ctx.report()
