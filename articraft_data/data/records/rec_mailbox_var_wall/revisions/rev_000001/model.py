from __future__ import annotations

# Vintage blue cast-iron U.S. Mail letter collection box – WALL-MOUNT variant.
#
# A residential wall-mount letter box: the hollow cast-iron cabinet body is
# fixed flush against a flat rectangular wall-mount back plate (with bolt
# bosses at the corners), so the box hangs on a vertical wall or fence
# instead of standing on a post or the ground.
#
# The body is a HOLLOW rectangular cabinet in faded blue with rust/copper
# weathering. A rounded tubular cap sits across the top. The front carries,
# top to bottom: a recessed upper panel below the cap, an embossed "LETTERS"
# header band, a pull-down hopper deposit flap (the rusty middle region),
# and a lower panel with a small schedule-card holder.
#
# The cabinet is built as thin cast-iron walls (front with the deposit
# opening, back, two sides, floor, top lid) enclosing an EMPTY interior.
# When the LETTERS flap is pulled down the opening reveals the real dark
# hollow chute behind it.
#
# Coordinate convention:
#   - up is +Z; the box hangs on the wall with its bottom at z = MOUNT_Z.
#   - the FRONT (LETTERS / deposit flap) faces +X; the back (wall side) is -X.
#   - the back plate back face lies in a vertical plane at x = PLATE_BACK_X.
#   - centerline is y = 0.
#   - the rounded top cap is a half-cylinder whose axis runs along Y.
#
# Root structure: the flat wall-mount back plate is the root. The hollow box
# body is fixed flush to its front face. The rounded top cap is fixed on
# the body. The pull-down LETTERS flap hangs from a BOTTOM hinge on the
# front face and tips outward/down to open the chute (REVOLUTE).
#
# Articulations:
#   - plate_to_body : FIXED (body mounted flush to plate front)
#   - body_to_cap   : FIXED (cap seated on body top)
#   - body_to_flap  : REVOLUTE pull-down hopper deposit flap (bottom hinge)

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# ---- key dimensions (meters) -------------------------------------------------
BODY_W = 0.40              # box width  (along Y)
BODY_D = 0.34              # box depth  (along X, front-back)
BODY_H = 0.78              # vertical body height (front wall)
WALL_T = 0.014             # cast-iron wall thickness

CAP_R = 0.165             # rounded top half-cylinder radius (across X)
CAP_LEN = 0.40            # cap length along Y

# Wall-mount elevation: body bottom sits at this height above the ground.
MOUNT_Z = 1.00

# Back plate dimensions
PLATE_T = 0.012            # plate thickness
PLATE_W = BODY_W + 0.06   # plate wider than body for flanges
PLATE_H = BODY_H + 0.10   # plate taller than body for bolt access
PLATE_BACK_X = -(BODY_D / 2.0) - PLATE_T  # back face x (against the wall)

FRONT_X = BODY_D / 2.0
BACK_X = -BODY_D / 2.0

# front zones (z heights from body bottom, absolute = + MOUNT_Z)
LOWER_TOP = 0.30          # top of the lower (card-holder) panel
FLAP_BOT = LOWER_TOP + 0.02
FLAP_H = 0.22             # pull-down deposit flap height
FLAP_TOP = FLAP_BOT + FLAP_H
HEADER_BOT = FLAP_TOP + 0.005  # LETTERS header band
HEADER_H = 0.075

FLAP_W = 0.30
FLAP_T = 0.020
JAMB_W = (BODY_W - FLAP_W) / 2.0  # front jamb width on each side of the opening

# Bolt boss dimensions (on the back face of the plate)
BOSS_R = 0.012
BOSS_H = 0.008


def _half_cyl_top() -> MeshGeometry:
    # Rounded tubular top: the UPPER half of a cylinder whose axis runs along Y.
    # Flat bottom on z=0 (local), domed top; sits on the body top so only the
    # rounded shell shows.
    n = 36
    hl = CAP_LEN / 2.0
    g = MeshGeometry()
    top_ids: list[int] = []
    bot_ids: list[int] = []
    arc = []
    for i in range(n + 1):
        a = math.pi * i / n
        x = CAP_R * math.cos(a)
        z = CAP_R * math.sin(a)
        arc.append((x, z))
    for (x, z) in arc:
        top_ids.append(g.add_vertex(x, hl, z))
    for (x, z) in arc:
        bot_ids.append(g.add_vertex(x, -hl, z))
    for i in range(n):
        a0, a1 = top_ids[i], top_ids[i + 1]
        b0, b1 = bot_ids[i], bot_ids[i + 1]
        g.add_face(a0, b0, b1)
        g.add_face(a0, b1, a1)
    tc = g.add_vertex(0.0, hl, 0.0)
    bc = g.add_vertex(0.0, -hl, 0.0)
    for i in range(n):
        g.add_face(tc, top_ids[i + 1], top_ids[i])
        g.add_face(bc, bot_ids[i], bot_ids[i + 1])
    g.add_face(top_ids[0], bot_ids[0], bot_ids[n])
    g.add_face(top_ids[0], bot_ids[n], top_ids[n])
    return g


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mount_mailbox")

    blue = model.material("blue", rgba=(0.34, 0.42, 0.52, 1.0))
    blue_dk = model.material("blue_dk", rgba=(0.26, 0.33, 0.42, 1.0))
    rust = model.material("rust", rgba=(0.55, 0.40, 0.30, 1.0))
    rust_dk = model.material("rust_dk", rgba=(0.45, 0.31, 0.22, 1.0))
    cap_blue = model.material("cap_blue", rgba=(0.31, 0.39, 0.49, 1.0))
    interior = model.material("interior", rgba=(0.08, 0.09, 0.11, 1.0))
    card = model.material("card", rgba=(0.82, 0.78, 0.66, 1.0))
    brass = model.material("brass", rgba=(0.62, 0.55, 0.32, 1.0))
    letters_mat = model.material("letters", rgba=(0.20, 0.24, 0.30, 1.0))
    plate_mat = model.material("plate_mat", rgba=(0.30, 0.35, 0.42, 1.0))
    bolt_mat = model.material("bolt_mat", rgba=(0.40, 0.38, 0.34, 1.0))

    # ---------------------------------------------------------- BACK PLATE (root)
    # Flat rectangular wall-mount plate. Back face at PLATE_BACK_X (against wall),
    # front face at BACK_X (flush with body back wall outer face).
    plate = model.part("back_plate")
    plate_cx = PLATE_BACK_X + PLATE_T / 2.0
    plate_cz = MOUNT_Z + BODY_H / 2.0

    plate.visual(
        Box((PLATE_T, PLATE_W, PLATE_H)),
        origin=Origin(xyz=(plate_cx, 0.0, plate_cz)),
        material=plate_mat,
        name="plate_panel",
    )

    # Bolt bosses at 4 corners on the back face (protrude toward -X into the wall).
    boss_yy = PLATE_W / 2.0 - 0.035
    boss_zz_lo = MOUNT_Z + 0.04
    boss_zz_hi = MOUNT_Z + BODY_H - 0.04
    boss_cx = PLATE_BACK_X - BOSS_H / 2.0
    for i, (by, bz) in enumerate(
        ((-boss_yy, boss_zz_lo), (boss_yy, boss_zz_lo),
         (-boss_yy, boss_zz_hi), (boss_yy, boss_zz_hi))
    ):
        plate.visual(
            Cylinder(radius=BOSS_R, length=BOSS_H),
            origin=Origin(
                xyz=(boss_cx, by, bz),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=bolt_mat,
            name=f"bolt_boss_{i}",
        )

    # ----------------------------------------------------------- BOX BODY
    # Hollow upright shell fixed flush to the plate front face. Thin cast-iron
    # walls enclose an empty interior; the front face leaves a deposit opening
    # for the pull-down flap.
    body = model.part("body")
    inner_w = BODY_W - 2 * WALL_T
    MZ = MOUNT_Z  # shorthand for elevation offset

    # back wall (-X)
    body.visual(
        Box((WALL_T, BODY_W, BODY_H)),
        origin=Origin(xyz=(BACK_X + WALL_T / 2.0, 0.0, BODY_H / 2.0 + MZ)),
        material=blue,
        name="back_wall",
    )
    # dark interior liner on the inner (front) face of the back wall
    body.visual(
        Box((0.004, inner_w, BODY_H - 2 * WALL_T)),
        origin=Origin(xyz=(BACK_X + WALL_T + 0.002, 0.0, BODY_H / 2.0 + MZ)),
        material=interior,
        name="interior_back_liner",
    )
    # side walls (+/-Y)
    for i, sgn in enumerate((1.0, -1.0)):
        body.visual(
            Box((BODY_D, WALL_T, BODY_H)),
            origin=Origin(xyz=(0.0, sgn * (BODY_W / 2.0 - WALL_T / 2.0), BODY_H / 2.0 + MZ)),
            material=blue,
            name=f"side_wall_{i}",
        )
    # floor + top lid
    body.visual(
        Box((BODY_D, BODY_W, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, WALL_T / 2.0 + MZ)),
        material=blue_dk,
        name="floor",
    )
    body.visual(
        Box((BODY_D, BODY_W, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, BODY_H - WALL_T / 2.0 + MZ)),
        material=blue_dk,
        name="top_lid",
    )

    # ---- front face panels (leave the deposit opening over the flap region) ----
    # lower front wall (below the flap)
    body.visual(
        Box((WALL_T, BODY_W, FLAP_BOT)),
        origin=Origin(xyz=(FRONT_X - WALL_T / 2.0, 0.0, FLAP_BOT / 2.0 + MZ)),
        material=blue,
        name="front_lower",
    )
    # upper front wall (above the flap)
    upper_h = BODY_H - FLAP_TOP
    body.visual(
        Box((WALL_T, BODY_W, upper_h)),
        origin=Origin(xyz=(FRONT_X - WALL_T / 2.0, 0.0, FLAP_TOP + upper_h / 2.0 + MZ)),
        material=blue,
        name="front_upper",
    )
    # front jambs framing the deposit opening
    for i, sgn in enumerate((1.0, -1.0)):
        body.visual(
            Box((WALL_T, JAMB_W, FLAP_H)),
            origin=Origin(
                xyz=(
                    FRONT_X - WALL_T / 2.0,
                    sgn * (BODY_W / 2.0 - JAMB_W / 2.0),
                    (FLAP_BOT + FLAP_TOP) / 2.0 + MZ,
                ),
            ),
            material=blue,
            name=f"front_jamb_{i}",
        )

    # front lower panel with raised frame + schedule-card holder
    body.visual(
        Box((0.012, FLAP_W + 0.03, LOWER_TOP - 0.06)),
        origin=Origin(xyz=(FRONT_X + 0.004, 0.0, (LOWER_TOP - 0.06) / 2.0 + 0.05 + MZ)),
        material=blue_dk,
        name="lower_panel_frame",
    )
    body.visual(
        Box((0.006, 0.13, 0.10)),
        origin=Origin(xyz=(FRONT_X + 0.010, 0.0, 0.18 + MZ)),
        material=card,
        name="schedule_card",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(FRONT_X + 0.012, 0.0, 0.085 + MZ), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="lower_knob",
    )

    # LETTERS header band
    band_back = FRONT_X - 0.004
    band_front = FRONT_X + 0.010
    band_t = band_front - band_back
    body.visual(
        Box((band_t, FLAP_W + 0.03, HEADER_H)),
        origin=Origin(
            xyz=((band_back + band_front) / 2.0, 0.0, HEADER_BOT + HEADER_H / 2.0 + MZ)
        ),
        material=rust,
        name="letters_band",
    )
    body.visual(
        Box((0.006, FLAP_W - 0.02, 0.028)),
        origin=Origin(xyz=(band_front + 0.002, 0.0, HEADER_BOT + HEADER_H / 2.0 + MZ)),
        material=letters_mat,
        name="letters_text",
    )

    # side "U.S. MAIL" raised plate on +Y side
    body.visual(
        Box((0.10, 0.006, 0.16)),
        origin=Origin(xyz=(0.04, BODY_W / 2.0 + 0.003, 0.50 + MZ)),
        material=blue_dk,
        name="side_plate",
    )

    # --------------------------------------------------------- ROUNDED TOP CAP
    cap = model.part("cap")
    cap_mesh = _half_cyl_top()
    cap_seat_z = BODY_H + MZ
    cap.visual(
        mesh_from_geometry(cap_mesh, "cap_shell"),
        origin=Origin(xyz=(0.0, 0.0, cap_seat_z)),
        material=cap_blue,
        name="cap_shell",
    )
    cap.visual(
        Box((0.05, CAP_LEN * 0.9, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, cap_seat_z + CAP_R - 0.005)),
        material=blue_dk,
        name="cap_ridge",
    )

    # ------------------------------------------------------------- DEPOSIT FLAP
    flap = model.part("flap")
    flap.visual(
        Box((FLAP_T, FLAP_W, FLAP_H)),
        origin=Origin(xyz=(0.0, 0.0, FLAP_H / 2.0)),
        material=rust_dk,
        name="flap_panel",
    )
    flap.visual(
        Box((0.006, FLAP_W + 0.02, 0.018)),
        origin=Origin(xyz=(FLAP_T / 2.0 + 0.003, 0.0, FLAP_H - 0.012)),
        material=rust,
        name="flap_top_rim",
    )
    flap.visual(
        Cylinder(radius=0.011, length=FLAP_W * 0.5),
        origin=Origin(
            xyz=(FLAP_T / 2.0 + 0.012, 0.0, FLAP_H - 0.03),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=brass,
        name="flap_handle",
    )

    # ----------------------------------------------------------- ARTICULATIONS
    # Back plate → body: FIXED, body flush on plate front face.
    model.articulation(
        "plate_to_body",
        ArticulationType.FIXED,
        parent=plate,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # Body → cap: FIXED, cap seated on body top.
    model.articulation(
        "body_to_cap",
        ArticulationType.FIXED,
        parent=body,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # Bottom-hinge pull-down flap. Hinge line along Y at the bottom of the flap,
    # just outside the front face. Panel extends +Z from the hinge; axis +Y
    # makes positive q tip the free (top) edge outward/down.
    model.articulation(
        "body_to_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flap,
        origin=Origin(xyz=(FRONT_X + FLAP_T / 2.0, 0.0, FLAP_BOT + MZ)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=0.0, upper=1.3),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    plate = object_model.get_part("back_plate")
    body = object_model.get_part("body")
    cap = object_model.get_part("cap")
    flap = object_model.get_part("flap")
    flap_joint = object_model.get_articulation("body_to_flap")

    # Dark interior liner is bonded to the inner face of the back wall.
    ctx.allow_overlap(
        body, body,
        elem_a="interior_back_liner", elem_b="back_wall",
        reason="Dark interior liner is bonded to the inner face of the back wall.",
    )

    # --- back plate is the root and its back face is in a vertical plane ------
    plate_aabb = ctx.part_world_aabb(plate)
    panel_aabb = ctx.part_element_world_aabb(plate, elem="plate_panel")
    ctx.check(
        "back plate panel back face is in a vertical plane (wall side)",
        panel_aabb is not None and abs(panel_aabb[0][0] - PLATE_BACK_X) < 0.005,
        details=f"panel_min_x={None if panel_aabb is None else panel_aabb[0][0]}, "
        f"expected={PLATE_BACK_X}",
    )

    # --- body is elevated above ground (wall-mounted) -------------------------
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "body is elevated above ground (wall-mounted)",
        body_aabb is not None and body_aabb[0][2] > 0.80,
        details=f"body_min_z={None if body_aabb is None else body_aabb[0][2]}",
    )

    # --- body back wall is flush against the plate front face -----------------
    if plate_aabb is not None and body_aabb is not None:
        ctx.expect_gap(
            body, plate, axis="x",
            positive_elem="back_wall", negative_elem="plate_panel",
            min_gap=-0.002, max_gap=0.002,
            name="body back wall is flush against plate front face",
        )

    # --- plate overlaps body footprint in YZ ----------------------------------
    ctx.expect_overlap(
        plate, body, axes="yz", min_overlap=0.30,
        name="plate covers behind the body footprint",
    )

    # --- flap joint is bottom-hinge revolute about horizontal Y ---------------
    ctx.check(
        "flap joint is revolute",
        str(flap_joint.articulation_type).upper().endswith("REVOLUTE"),
        details=f"type={flap_joint.articulation_type}",
    )
    ax = flap_joint.axis
    ctx.check(
        "flap hinge axis is horizontal Y",
        abs(abs(ax[1]) - 1.0) < 1e-6 and abs(ax[0]) < 1e-6 and abs(ax[2]) < 1e-6,
        details=f"axis={ax}",
    )

    # --- realistic proportions ------------------------------------------------
    if body_aabb is not None:
        bh = body_aabb[1][2] - body_aabb[0][2]
        bw = body_aabb[1][1] - body_aabb[0][1]
        ctx.check(
            "body height is realistic (~0.78 m)",
            0.74 < bh < 0.92,
            details=f"height={bh}",
        )
        ctx.check(
            "body width is realistic (~0.4 m)",
            0.34 < bw < 0.50,
            details=f"width={bw}",
        )

    # --- HOLLOW INTERIOR: the chute behind the deposit flap is genuinely deep --
    back_aabb = ctx.part_element_world_aabb(body, elem="interior_back_liner")
    if back_aabb is not None:
        depth = FRONT_X - back_aabb[1][0]
        ctx.check(
            "interior chute is deep (open, not a solid/shallow back)",
            depth > 0.25,
            details=f"front_to_back_depth={depth}",
        )

    # --- rounded cap sits on top of the body ----------------------------------
    cap_aabb = ctx.part_world_aabb(cap)
    if cap_aabb is not None and body_aabb is not None:
        ctx.check(
            "rounded cap is at the top of the box",
            cap_aabb[1][2] >= body_aabb[1][2] - 0.001,
            details=f"cap_top={cap_aabb[1][2]}, body_top={body_aabb[1][2]}",
        )
    ctx.expect_overlap(
        cap, body, axes="xy", min_overlap=0.15,
        name="cap seats over the body footprint",
    )

    # --- closed flap covers the front opening ---------------------------------
    ctx.expect_overlap(
        flap, body, axes="yz", min_overlap=0.10,
        name="closed flap covers the deposit opening",
    )
    ctx.expect_gap(
        flap, body, axis="x",
        positive_elem="flap_panel", negative_elem="front_lower",
        min_gap=-0.002, max_gap=0.022,
        name="closed flap is roughly flush with the front",
    )

    # --- LETTERS header is above the flap -------------------------------------
    band_aabb = ctx.part_element_world_aabb(body, elem="letters_band")
    flap_aabb = ctx.part_world_aabb(flap)
    if band_aabb is not None and flap_aabb is not None:
        ctx.check(
            "LETTERS band sits above the deposit flap",
            band_aabb[0][2] >= flap_aabb[1][2] - 0.02,
            details=f"band_bot={band_aabb[0][2]}, flap_top={flap_aabb[1][2]}",
        )

    # --- opened flap swings outward (+X) and the free edge drops --------------
    rest_flap_aabb = ctx.part_world_aabb(flap)
    with ctx.pose({flap_joint: 1.2}):
        open_flap_aabb = ctx.part_world_aabb(flap)
    ctx.check(
        "flap opens outward in +X",
        rest_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[1][0] > rest_flap_aabb[1][0] + 0.04,
        details=f"rest_maxX={None if rest_flap_aabb is None else rest_flap_aabb[1][0]}, "
        f"open_maxX={None if open_flap_aabb is None else open_flap_aabb[1][0]}",
    )
    ctx.check(
        "flap free edge drops when open",
        rest_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[1][2] < rest_flap_aabb[1][2] - 0.04,
        details=f"rest_topZ={None if rest_flap_aabb is None else rest_flap_aabb[1][2]}, "
        f"open_topZ={None if open_flap_aabb is None else open_flap_aabb[1][2]}",
    )

    # --- bolt bosses are on the back face of the plate ------------------------
    for i in range(4):
        boss_aabb = ctx.part_element_world_aabb(plate, elem=f"bolt_boss_{i}")
        if boss_aabb is not None and panel_aabb is not None:
            ctx.check(
                f"bolt_boss_{i} protrudes behind the plate back face",
                boss_aabb[0][0] < panel_aabb[0][0] + 0.001,
                details=f"boss_min_x={boss_aabb[0][0]}, panel_min_x={panel_aabb[0][0]}",
            )

    return ctx.report()
