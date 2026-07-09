from __future__ import annotations

# Side-access 20-foot intermodal shipping container (fork variant).
# Container long axis along X, width along Y, height along Z.
# Floor at z=0, body spans z in [0, H].
#
# Difference from parent: cargo opening moved from the +X end to the +Y long
# side wall. Both end walls (-X and +X) are now solid corrugated panels. The
# +Y side has a full-length opening framed by header/sill rails and two corner
# posts, with a pair of corrugated doors hinged on vertical edges at each end
# of the opening. Each door swings outward (+Y) about a vertical REVOLUTE axis.
# -Y side wall remains a full corrugated panel.
#
# Articulations:
#   body_to_door_0: REVOLUTE, vertical hinge at -X edge of side opening, 0..120°
#   body_to_door_1: REVOLUTE, vertical hinge at +X edge of side opening, 0..120°
#   door_i_to_handle_i_j: REVOLUTE about rod axis (Z), 0..110°

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# ── container outer dimensions (m, realistic ISO 20ft) ──
L = 6.06          # length along X
W = 2.44          # width along Y
H = 2.59          # height along Z

WALL_T = 0.035    # wall skin thickness
CORNER = 0.16     # corner casting cube size
DOOR_T = 0.06     # door panel depth
FLOOR_H = 0.10    # floor plate height

CORR_PITCH = 0.30
CORR_DEPTH = 0.04
CORR_WIDTH = 0.12

# Body spans the full container length (no end doorway)
BODY_X0 = -L / 2.0
BODY_X1 =  L / 2.0

# ── side doorway on +Y wall ──
DOORWAY_MARGIN = CORNER + 0.14          # space from each end to opening edge
OPENING_X0 = BODY_X0 + DOORWAY_MARGIN   # -X edge of opening
OPENING_X1 = BODY_X1 - DOORWAY_MARGIN   # +X edge of opening
OPENING_LEN = OPENING_X1 - OPENING_X0

HEADER_H = 0.14
SILL_H   = 0.14
POST_W_X = DOORWAY_MARGIN - CORNER      # post fills gap: corner casting → opening
POST_D_Y = 0.14

DOOR_GAP = 0.02
DOOR_W = (OPENING_LEN - DOOR_GAP) / 2.0
DOOR_H = H - 0.40

# Rod Y in door-local frame (proud of corrugation ribs on +Y outward face)
ROD_Y_LOCAL = -DOOR_T / 2.0 + DOOR_T * 0.225 + CORR_DEPTH - 0.012


# ── geometry helpers ──

def _corrugated_wall(length_x: float, height_z: float, base_y: float, depth_sign: float):
    """Vertical-corrugation wall in XZ plane at y = base_y."""
    geo = BoxGeometry((length_x, WALL_T, height_z))
    geo.translate(0.0, base_y, 0.0)
    n = max(1, int(length_x / CORR_PITCH))
    x0 = -length_x / 2.0 + CORR_PITCH / 2.0
    for i in range(n):
        x = x0 + i * CORR_PITCH
        rib = BoxGeometry((CORR_WIDTH, CORR_DEPTH, height_z * 0.96))
        rib.translate(x, base_y + depth_sign * (WALL_T / 2.0 + CORR_DEPTH / 2.0), 0.0)
        geo.merge(rib)
    return geo


def _corrugated_end_wall(width_y: float, height_z: float, base_x: float, depth_sign: float):
    """End wall in YZ plane at x = base_x, vertical ribs along Y."""
    geo = BoxGeometry((WALL_T, width_y, height_z))
    geo.translate(base_x, 0.0, 0.0)
    n = max(1, int(width_y / CORR_PITCH))
    y0 = -width_y / 2.0 + CORR_PITCH / 2.0
    for i in range(n):
        y = y0 + i * CORR_PITCH
        rib = BoxGeometry((CORR_DEPTH, CORR_WIDTH, height_z * 0.96))
        rib.translate(base_x + depth_sign * (WALL_T / 2.0 + CORR_DEPTH / 2.0), y, 0.0)
        geo.merge(rib)
    return geo


def _corrugated_side_door_panel(width_x: float, height_z: float):
    """Side-door panel in XZ plane, thin along Y, ribs face +Y (outward).
    Centered at local origin.
    """
    geo = BoxGeometry((width_x, DOOR_T * 0.45, height_z))
    fr_t = 0.07
    fr_d = DOOR_T
    # top / bottom frame rails (along X)
    for sz in (-1.0, 1.0):
        m = BoxGeometry((width_x, fr_d, fr_t))
        m.translate(0.0, 0.0, sz * (height_z / 2.0 - fr_t / 2.0))
        geo.merge(m)
    # left / right frame stiles (along Z)
    for sx in (-1.0, 1.0):
        m = BoxGeometry((fr_t, fr_d, height_z))
        m.translate(sx * (width_x / 2.0 - fr_t / 2.0), 0.0, 0.0)
        geo.merge(m)
    # vertical corrugation ribs facing +Y
    inner_w = width_x - 2.0 * fr_t
    n = max(1, int(inner_w / CORR_PITCH))
    x0 = -inner_w / 2.0 + CORR_PITCH / 2.0
    for i in range(n):
        x = x0 + i * CORR_PITCH
        rib = BoxGeometry((CORR_WIDTH, CORR_DEPTH, height_z - 2.0 * fr_t - 0.02))
        rib.translate(x, DOOR_T * 0.225 + CORR_DEPTH / 2.0, 0.0)
        geo.merge(rib)
    return geo


def _corner_casting():
    return BoxGeometry((CORNER, CORNER, CORNER))


def _build_handle_geometry():
    """Cam-handle assembly (hub + lever + grip) protruding along +Y."""
    hub = CylinderGeometry(0.022, 0.06, radial_segments=16).rotate_x(math.pi / 2.0)
    hub.translate(0.0, 0.04, 0.0)
    lever = BoxGeometry((0.045, 0.14, 0.035))
    lever.translate(0.0, 0.13, 0.0)
    grip = CylinderGeometry(0.025, 0.10, radial_segments=12).rotate_x(math.pi / 2.0)
    grip.translate(0.0, 0.20, 0.0)
    return hub, lever, grip


# ── build ──

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_access_shipping_container")

    white = model.material("white_steel", rgba=(0.90, 0.91, 0.92, 1.0))
    grey  = model.material("grey_steel",  rgba=(0.62, 0.64, 0.66, 1.0))
    dark  = model.material("dark_hardware", rgba=(0.18, 0.19, 0.21, 1.0))

    # ===============================================================
    # BODY (root)
    # ===============================================================
    body = model.part("body")
    side_h = H - FLOOR_H
    wz = H / 2.0  # wall vertical center

    # ── floor ──
    floor = BoxGeometry((L, W, FLOOR_H))
    floor.translate(0.0, 0.0, FLOOR_H / 2.0)
    body.visual(mesh_from_geometry(floor, "floor"), material=grey, name="floor")

    # ── roof ──
    roof = BoxGeometry((L, W, 0.05))
    roof.translate(0.0, 0.0, H - 0.025)
    body.visual(mesh_from_geometry(roof, "roof"), material=white, name="roof")

    # ── -Y side wall (full corrugated, solid) ──
    wn = _corrugated_wall(L, side_h, -(W / 2.0 - WALL_T / 2.0), -1.0)
    wn.translate(0.0, 0.0, wz)
    body.visual(mesh_from_geometry(wn, "side_wall_n"), material=white, name="side_wall_n")

    # ── -X end wall (corrugated, ribs face -X outward) ──
    en = _corrugated_end_wall(W - 0.10, side_h, BODY_X0 + WALL_T / 2.0, -1.0)
    en.translate(0.0, 0.0, wz)
    body.visual(mesh_from_geometry(en, "end_wall_n"), material=white, name="end_wall_n")

    # ── +X end wall (corrugated, ribs face +X outward) — solid, no doorway ──
    ep = _corrugated_end_wall(W - 0.10, side_h, BODY_X1 - WALL_T / 2.0, +1.0)
    ep.translate(0.0, 0.0, wz)
    body.visual(mesh_from_geometry(ep, "end_wall_p"), material=white, name="end_wall_p")

    # ── +Y side doorway frame: header, sill, two posts ──
    header = BoxGeometry((OPENING_LEN, 0.12, HEADER_H))
    header.translate(0.0, W / 2.0 - 0.06, H - HEADER_H / 2.0)
    body.visual(mesh_from_geometry(header, "door_header"), material=grey, name="door_header")

    sill = BoxGeometry((OPENING_LEN, 0.12, SILL_H))
    sill.translate(0.0, W / 2.0 - 0.06, SILL_H / 2.0)
    body.visual(mesh_from_geometry(sill, "door_sill"), material=grey, name="door_sill")

    post_x_positions = [
        OPENING_X0 - POST_W_X / 2.0,   # -X post
        OPENING_X1 + POST_W_X / 2.0,   # +X post
    ]
    for i in range(2):
        post = BoxGeometry((POST_W_X, POST_D_Y, H))
        post.translate(post_x_positions[i], W / 2.0 - POST_D_Y / 2.0, H / 2.0)
        body.visual(mesh_from_geometry(post, f"post_{i}"), material=grey, name=f"post_{i}")

    # ── 8 corner castings ──
    ci = 0
    for sx in (BODY_X0 + CORNER / 2.0, BODY_X1 - CORNER / 2.0):
        for sy in (-1.0, 1.0):
            for sz in (CORNER / 2.0, H - CORNER / 2.0):
                cc = _corner_casting()
                cc.translate(sx, sy * (W / 2.0 - CORNER / 2.0), sz)
                body.visual(mesh_from_geometry(cc, f"corner_{ci}"), material=dark, name=f"corner_{ci}")
                ci += 1

    body.inertial = Inertial.from_geometry(
        Box((L, W, H)), mass=2300.0, origin=Origin(xyz=(0.0, 0.0, H / 2.0))
    )

    # ===============================================================
    # DOORS on +Y side wall
    # ===============================================================
    # door_0: hinged at -X opening edge, free edge toward +X (container center)
    # door_1: hinged at +X opening edge, free edge toward -X (container center)
    door_specs = [
        # (index, hinge_x,   free_dir, axis_z_sign)
        (0,      OPENING_X0, +1.0,     +1.0),
        (1,      OPENING_X1, -1.0,     -1.0),
    ]

    for di, hinge_x, free_dir, axis_z in door_specs:
        dname = f"door_{di}"
        door = model.part(dname)

        # Panel: hinge edge at local x = 0, free edge at free_dir * DOOR_W
        panel = _corrugated_side_door_panel(DOOR_W, DOOR_H)
        panel.translate(free_dir * (DOOR_W / 2.0), -DOOR_T / 2.0, 0.0)
        door.visual(mesh_from_geometry(panel, "door_panel"), material=white, name="door_panel")

        door.inertial = Inertial.from_geometry(
            Box((DOOR_W, DOOR_T, DOOR_H)),
            mass=80.0,
            origin=Origin(xyz=(free_dir * DOOR_W / 2.0, -DOOR_T / 2.0, 0.0)),
        )

        # Hinge at +Y wall surface, at the opening edge
        model.articulation(
            f"body_to_{dname}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=door,
            origin=Origin(xyz=(hinge_x, W / 2.0, H / 2.0)),
            axis=(0.0, 0.0, axis_z),
            motion_limits=MotionLimits(
                effort=200.0, velocity=2.0,
                lower=0.0, upper=math.radians(120.0),
            ),
        )

        # ── locking rods + cam handles ──
        rod_xs = [
            free_dir * DOOR_W * 0.30,
            free_dir * DOOR_W * 0.78,
        ]
        for ri in range(2):
            rx = rod_xs[ri]

            # vertical locking rod
            rod = CylinderGeometry(0.018, DOOR_H - 0.10, radial_segments=16)
            rod.translate(rx, ROD_Y_LOCAL, 0.0)
            door.visual(mesh_from_geometry(rod, f"rod_{ri}"), material=dark, name=f"rod_{ri}")

            # keeper brackets (top + bottom)
            for ki, sz in enumerate((1.0, -1.0)):
                kp = BoxGeometry((0.05, 0.06, 0.05))
                kp.translate(rx, ROD_Y_LOCAL - 0.02, sz * (DOOR_H / 2.0 - 0.18))
                door.visual(
                    mesh_from_geometry(kp, f"keeper_{ri}_{ki}"),
                    material=grey, name=f"keeper_{ri}_{ki}",
                )

            # cam handle (child part)
            hname = f"handle_{di}_{ri}"
            handle = model.part(hname)

            hub, lever, grip = _build_handle_geometry()
            handle.visual(mesh_from_geometry(hub, "hub"), material=dark, name="hub")
            handle.visual(mesh_from_geometry(lever, "lever"), material=dark, name="lever")
            handle.visual(mesh_from_geometry(grip, "grip"), material=grey, name="grip")

            handle.inertial = Inertial.from_geometry(
                Box((0.06, 0.22, 0.06)), mass=1.2, origin=Origin(xyz=(0.0, 0.11, 0.0))
            )

            model.articulation(
                f"{dname}_to_{hname}",
                ArticulationType.REVOLUTE,
                parent=door,
                child=handle,
                origin=Origin(xyz=(rx, ROD_Y_LOCAL, 0.05)),
                axis=(0.0, 0.0, 1.0),
                motion_limits=MotionLimits(
                    effort=20.0, velocity=3.0,
                    lower=0.0, upper=math.radians(110.0),
                ),
            )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body   = object_model.get_part("body")
    door_0 = object_model.get_part("door_0")
    door_1 = object_model.get_part("door_1")
    hinge_0 = object_model.get_articulation("body_to_door_0")
    hinge_1 = object_model.get_articulation("body_to_door_1")

    # ── container proportions (long box) ──
    body_ext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "container is a long box (X longest)",
        body_ext[0] > body_ext[1] + 1.0 and body_ext[0] > body_ext[2] + 1.0,
        details=f"body extents={body_ext}",
    )

    # ── doors are on the +Y side wall, not at an end ──
    for d, nm in ((door_0, "door_0"), (door_1, "door_1")):
        pos = ctx.part_world_position(d)
        ctx.check(
            f"{nm} is on the +Y side wall",
            pos is not None and pos[1] > W / 2.0 - 0.3,
            details=f"{nm} origin={pos}",
        )
        ctx.check(
            f"{nm} is along the side (not at an end wall)",
            pos is not None and abs(pos[0]) < L / 2.0 - 0.2,
            details=f"{nm} x={pos[0] if pos else None}",
        )

    # ── hinge seating (intentional local overlap with posts) ──
    ctx.allow_overlap(
        door_0, body, elem_a="door_panel", elem_b="post_0",
        reason="Door 0 hinge edge frame seats against the -X doorway post that carries its hinge.",
    )
    ctx.allow_overlap(
        door_1, body, elem_a="door_panel", elem_b="post_1",
        reason="Door 1 hinge edge frame seats against the +X doorway post that carries its hinge.",
    )
    ctx.expect_contact(door_0, body, name="door_0 seated against doorway frame")
    ctx.expect_contact(door_1, body, name="door_1 seated against doorway frame")

    # ── handle / rod overlaps ──
    for di in range(2):
        dname = f"door_{di}"
        d = object_model.get_part(dname)
        for ri in range(2):
            hname = f"handle_{di}_{ri}"
            h = object_model.get_part(hname)
            ctx.allow_overlap(
                d, h, elem_a=f"rod_{ri}", elem_b="hub",
                reason=f"Handle hub seated around locking rod on {dname}.",
            )
            ctx.allow_overlap(
                d, h, elem_a=f"rod_{ri}", elem_b="lever",
                reason=f"Handle lever passes over rod on {dname}.",
            )
            ctx.allow_overlap(
                d, h, elem_a="door_panel", elem_b="hub",
                reason=f"Handle hub sits against corrugated door face on {dname}.",
            )

    # ── doors swing open: free edge moves outward (+Y) ──
    for d, hinge, nm in ((door_0, hinge_0, "door_0"), (door_1, hinge_1, "door_1")):
        y_max_rest = ctx.part_world_aabb(d)[1][1]
        with ctx.pose({hinge: math.radians(90.0)}):
            y_max_open = ctx.part_world_aabb(d)[1][1]
        ctx.check(
            f"{nm} swings outward (+Y) when opened",
            y_max_open > y_max_rest + 0.5,
            details=f"{nm} y_max rest={y_max_rest:.3f} open={y_max_open:.3f}",
        )

    # ── cam handles rotate about rod axes ──
    for di in range(2):
        dname = f"door_{di}"
        for ri in range(2):
            hname = f"handle_{di}_{ri}"
            handle = object_model.get_part(hname)
            hjoint = object_model.get_articulation(f"{dname}_to_{hname}")
            ctx.expect_contact(
                handle, object_model.get_part(dname),
                name=f"{hname} mounted on {dname}",
            )
            ext0 = _ext(ctx.part_world_aabb(handle))
            with ctx.pose({hjoint: math.radians(90.0)}):
                ext90 = _ext(ctx.part_world_aabb(handle))
            ctx.check(
                f"{hname} rotates about rod axis (lever sweeps)",
                abs(ext90[0] - ext0[0]) > 0.05 or abs(ext90[1] - ext0[1]) > 0.05,
                details=f"{hname} rest={ext0} turned={ext90}",
            )

    return ctx.report()


object_model = build_object_model()
