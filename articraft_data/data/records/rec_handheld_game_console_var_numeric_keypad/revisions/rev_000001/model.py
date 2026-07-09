from __future__ import annotations

# Red handheld BRICK GAME console (classic Tetris "Brick Game").
# Frame: long axis = X (length ~0.16), width = Y (~0.075), thickness = Z (~0.018).
#   Front control face points up at +Z; the LCD sits toward +X (top), controls
#   toward -X (bottom). Buttons press DOWN into the slab along -Z.
# Articulations:
#   - D-pad: REVOLUTE rocker tilting a small amount about the Y axis.
#   - 12 numeric keypad keys (4 rows x 3 cols): each PRISMATIC press straight
#     down (~2 mm travel).
#   - power slide switch: PRISMATIC slide along X (small travel).

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- overall slab dimensions ----
LEN = 0.160  # X
WID = 0.075  # Y
THK = 0.018  # Z (thickness at the thicker, lower end)

TOP_Z = THK / 2.0  # front control face sits near here (with slight wedge)

# Button press depth.
BTN_TRAVEL = 0.002


def _slab_solid() -> cq.Workplane:
    # Rounded red slab with a slight wedge: the top (LCD, +X) end is a touch
    # thinner than the controls (-X) end, giving the classic angled feel.
    solid = (
        cq.Workplane("XY")
        .rect(LEN, WID)
        .extrude(THK)
        .translate((0.0, 0.0, -THK / 2.0))
        .edges("|Z")
        .fillet(0.010)
    )
    drop = 0.004
    pts = [
        (0.0, -THK / 2.0),
        (LEN / 2.0, -THK / 2.0),
        (LEN / 2.0, -THK / 2.0 + drop),
    ]
    wedge = (
        cq.Workplane("XZ")
        .polyline(pts)
        .close()
        .extrude(WID * 1.5)
        .translate((0.0, WID * 0.75, 0.0))
    )
    solid = solid.cut(wedge)
    return solid


def _recess_box() -> cq.Workplane:
    rx, ry = 0.052, 0.052
    return (
        cq.Workplane("XY")
        .center(0.045, 0.0)
        .rect(rx, ry)
        .extrude(THK)
        .translate((0.0, 0.0, TOP_Z - 0.004))
    )


def _body_mesh():
    solid = _slab_solid().cut(_recess_box())
    return mesh_from_cadquery(solid, "shell")


def _grid_mesh():
    geo = None
    cell = 0.0030
    nx, ny = 9, 9
    x0 = 0.045 - (nx - 1) * cell / 2.0
    y0 = -(ny - 1) * cell / 2.0
    rail = 0.0004
    for i in range(nx):
        bar = BoxGeometry((rail, ny * cell, 0.0008)).translate(x0 + i * cell, 0.0, 0.0)
        geo = bar if geo is None else (geo.merge(bar) or geo)
    for j in range(ny):
        bar = BoxGeometry((nx * cell, rail, 0.0008)).translate(0.045, y0 + j * cell, 0.0)
        geo = bar if geo is None else (geo.merge(bar) or geo)
    return mesh_from_geometry(geo, "pixel_grid")


def _key_cap_solid() -> cq.Workplane:
    """Small rounded-rectangle key cap for the numeric keypad (phone-style)."""
    kw, kd, kh = 0.008, 0.007, 0.003
    cap = (
        cq.Workplane("XY")
        .box(kw, kd, kh)
        .edges("|Z")
        .fillet(0.001)
    )
    return cap


# ---- Keypad grid parameters (shared between build and tests) ----
KEY_ROWS = 4
KEY_COLS = 3
KEY_COUNT = KEY_ROWS * KEY_COLS
KEY_PITCH_X = 0.011   # row spacing along X (long axis)
KEY_PITCH_Y = 0.011   # column spacing along Y (width axis)
KEY_GRID_CX = -0.022  # grid center X on the control face
KEY_GRID_CY = -0.018  # grid center Y (right-hand side of face)
KEY_H = 0.003          # key cap height


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="brick_game_console")

    red = model.material("shell_red", rgba=(0.78, 0.12, 0.12, 1.0))
    bezel_gray = model.material("bezel_gray", rgba=(0.55, 0.56, 0.58, 1.0))
    screen = model.material("screen_dark", rgba=(0.30, 0.36, 0.30, 1.0))
    grid_green = model.material("grid_green", rgba=(0.18, 0.24, 0.18, 1.0))
    yellow = model.material("button_yellow", rgba=(0.95, 0.78, 0.10, 1.0))
    black = model.material("dpad_black", rgba=(0.10, 0.10, 0.11, 1.0))
    key_cream = model.material("key_cream", rgba=(0.88, 0.85, 0.76, 1.0))

    # ---------------- body (root) ----------------
    body = model.part("body")
    body.visual(_body_mesh(), material=red, name="shell")

    # Gray bezel ring seated in the recess, with a dark screen face inside it.
    bezel_top = TOP_Z - 0.0005
    body.visual(
        Box((0.052, 0.052, 0.004)),
        origin=Origin(xyz=(0.045, 0.0, bezel_top - 0.002)),
        material=bezel_gray,
        name="bezel",
    )
    screen_top = bezel_top - 0.0015
    body.visual(
        Box((0.040, 0.040, 0.0025)),
        origin=Origin(xyz=(0.045, 0.0, screen_top - 0.00125)),
        material=screen,
        name="screen_face",
    )
    # Pixel grid sitting just on top of the screen face.
    body.visual(
        _grid_mesh(),
        origin=Origin(xyz=(0.0, 0.0, screen_top + 0.0002)),
        material=grid_green,
        name="pixel_grid",
    )

    body.inertial = Inertial.from_geometry(
        Box((LEN, WID, THK)), mass=0.18, origin=Origin(xyz=(0.0, 0.0, 0.0))
    )

    # ---------------- D-pad rocker (revolute) ----------------
    # Black plus-shaped cross on the lower-left of the face. Mounted on a small
    # pivot boss so it can rock about Y. Origin at the cross center.
    dpad_cx, dpad_cy = -0.040, 0.020
    dpad_z = TOP_Z + 0.001

    dpad = model.part("dpad")
    arm = 0.0095
    cross_h = 0.005
    cross = BoxGeometry((2 * arm, 0.011, cross_h))
    cross = cross.merge(BoxGeometry((0.011, 2 * arm, cross_h))) or cross
    dpad.visual(
        mesh_from_geometry(cross, "dpad_cross"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=black,
        name="dpad_cross",
    )
    dpad.inertial = Inertial.from_geometry(Box((2 * arm, 2 * arm, cross_h)), mass=0.004)
    dpad_joint = model.articulation(
        "dpad_rocker",
        ArticulationType.REVOLUTE,
        parent=body,
        child=dpad,
        origin=Origin(xyz=(dpad_cx, dpad_cy, dpad_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0, velocity=2.0, lower=-0.20, upper=0.20
        ),
    )

    # ---------------- numeric keypad (prismatic) ----------------
    # 3-column x 4-row grid of small rounded-rectangle keys, phone-style.
    # Each key presses straight down (-Z) into the shell.
    face_z = TOP_Z
    key_cap_mesh = mesh_from_cadquery(_key_cap_solid(), "key_cap")

    key_parts = []
    for i in range(KEY_COUNT):
        row = i // KEY_COLS
        col = i % KEY_COLS
        # Row 0 is topmost (+X, closest to screen); higher row index = further -X.
        kx = KEY_GRID_CX + ((KEY_ROWS - 1) / 2.0 - row) * KEY_PITCH_X
        ky = KEY_GRID_CY + (col - (KEY_COLS - 1) / 2.0) * KEY_PITCH_Y

        key = model.part(f"key_{i}")
        key.visual(
            key_cap_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=key_cream,
            name=f"key_{i}_cap",
        )
        key.inertial = Inertial.from_geometry(
            Box((0.008, 0.007, KEY_H)), mass=0.001
        )
        joint = model.articulation(
            f"key_{i}_press",
            ArticulationType.PRISMATIC,
            parent=body,
            child=key,
            origin=Origin(xyz=(kx, ky, face_z + KEY_H / 2.0 - 0.0006)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=1.0, velocity=0.05, lower=0.0, upper=BTN_TRAVEL
            ),
        )
        key_parts.append((key, joint, f"key_{i}"))

    # ---------------- power slide switch (prismatic) ----------------
    slide = model.part("power_switch")
    slide.visual(
        Box((0.012, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=black,
        name="switch_nub",
    )
    slide.inertial = Inertial.from_geometry(Box((0.012, 0.006, 0.004)), mass=0.002)
    slide_joint = model.articulation(
        "power_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=slide,
        origin=Origin(xyz=(0.070, -0.026, TOP_Z + 0.0005)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0, velocity=0.05, lower=-0.006, upper=0.006
        ),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")

    # ---- slab proportions: longer than wide, and a flat slab ----
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "slab is longer than it is wide",
        bext[0] > bext[1] + 0.04,
        details=f"body extents={bext}",
    )
    ctx.check(
        "slab is flat (thin in Z)",
        bext[2] < bext[1] * 0.5,
        details=f"body extents={bext}",
    )

    # ---- screen recessed in its bezel ----
    bz = ctx.part_element_world_aabb(body, elem="bezel")
    sz = ctx.part_element_world_aabb(body, elem="screen_face")
    ctx.check(
        "screen face is recessed below the bezel top",
        sz[1][2] < bz[1][2] + 1e-6,
        details=f"screen top={sz[1][2]}, bezel top={bz[1][2]}",
    )
    ctx.expect_within(
        body, body, axes="xy", inner_elem="screen_face", outer_elem="bezel",
        margin=0.0, name="screen face sits inside the bezel footprint",
    )

    # ---- D-pad rocker articulates (tilts about Y) ----
    dpad = object_model.get_part("dpad")
    dpad_joint = object_model.get_articulation("dpad_rocker")
    ctx.expect_contact(dpad, body, name="dpad seated on the front face")
    aabb0 = ctx.part_world_aabb(dpad)
    with ctx.pose({dpad_joint: 0.18}):
        aabb1 = ctx.part_world_aabb(dpad)
    ctx.check(
        "d-pad rocker tilts (z extent changes when posed)",
        abs(aabb1[1][2] - aabb0[1][2]) > 0.0008,
        details=f"rest top z={aabb0[1][2]}, tilted top z={aabb1[1][2]}",
    )

    # ---- numeric keypad: 12 keys in a 4x3 grid, all pressable ----
    # Every key rests on the front face and is above the slab midplane.
    for i in range(KEY_COUNT):
        key = object_model.get_part(f"key_{i}")
        ctx.expect_contact(key, body, name=f"key_{i} rests on the front face")
        kp = ctx.part_world_position(key)
        ctx.check(
            f"key_{i} mounted on the front face (+Z side)",
            kp is not None and kp[2] > 0.0,
            details=f"key_{i} pos={kp}",
        )

    # Verify regular grid: row centers have uniform X spacing.
    row_centers_x = []
    for row in range(KEY_ROWS):
        xs = []
        for col in range(KEY_COLS):
            idx = row * KEY_COLS + col
            pos = ctx.part_world_position(object_model.get_part(f"key_{idx}"))
            xs.append(pos[0])
        row_centers_x.append(sum(xs) / len(xs))

    spacings_x = [
        row_centers_x[r] - row_centers_x[r + 1]
        for r in range(len(row_centers_x) - 1)
    ]
    ctx.check(
        "keypad rows have uniform X spacing",
        all(abs(s - spacings_x[0]) < 0.0005 for s in spacings_x),
        details=f"row spacings={spacings_x}",
    )

    # Verify regular grid: column centers have uniform Y spacing.
    col_centers_y = []
    for col in range(KEY_COLS):
        ys = []
        for row in range(KEY_ROWS):
            idx = row * KEY_COLS + col
            pos = ctx.part_world_position(object_model.get_part(f"key_{idx}"))
            ys.append(pos[1])
        col_centers_y.append(sum(ys) / len(ys))

    spacings_y = [
        col_centers_y[c + 1] - col_centers_y[c]
        for c in range(len(col_centers_y) - 1)
    ]
    ctx.check(
        "keypad columns have uniform Y spacing",
        all(abs(s - spacings_y[0]) < 0.0005 for s in spacings_y),
        details=f"column spacings={spacings_y}",
    )

    # Confirm a sample key presses straight down (-Z).
    sample_key = object_model.get_part("key_0")
    sample_joint = object_model.get_articulation("key_0_press")
    rest_z = ctx.part_world_position(sample_key)[2]
    with ctx.pose({sample_joint: BTN_TRAVEL}):
        pressed_z = ctx.part_world_position(sample_key)[2]
    ctx.check(
        "key_0 presses straight down",
        pressed_z < rest_z - 0.0015,
        details=f"rest_z={rest_z}, pressed_z={pressed_z}",
    )

    # Confirm a second key also presses down (uniform joint policy).
    sample_key2 = object_model.get_part("key_11")
    sample_joint2 = object_model.get_articulation("key_11_press")
    rest_z2 = ctx.part_world_position(sample_key2)[2]
    with ctx.pose({sample_joint2: BTN_TRAVEL}):
        pressed_z2 = ctx.part_world_position(sample_key2)[2]
    ctx.check(
        "key_11 presses straight down",
        pressed_z2 < rest_z2 - 0.0015,
        details=f"rest_z={rest_z2}, pressed_z={pressed_z2}",
    )

    # Confirm the keypad is on the -Y side, D-pad on the +Y side.
    dpad_pos = ctx.part_world_position(dpad)
    key_mid = object_model.get_part("key_5")
    key_pos = ctx.part_world_position(key_mid)
    ctx.check(
        "keypad is on the opposite side of the D-pad along Y",
        key_pos[1] < dpad_pos[1] - 0.010,
        details=f"dpad_y={dpad_pos[1]}, keypad_y={key_pos[1]}",
    )

    # Confirm exactly KEY_COUNT key parts exist (no hand-named one-offs).
    key_names = [f"key_{i}" for i in range(KEY_COUNT)]
    for nm in key_names:
        ctx.check(
            f"{nm} exists in the model",
            object_model.get_part(nm) is not None,
            details=f"part {nm} not found",
        )

    # ---- power slide switch slides along X ----
    slide = object_model.get_part("power_switch")
    slide_j = object_model.get_articulation("power_slide")
    s_rest = ctx.part_world_position(slide)[0]
    with ctx.pose({slide_j: 0.006}):
        s_slid = ctx.part_world_position(slide)[0]
    ctx.check(
        "power switch slides along the slab",
        s_slid > s_rest + 0.004,
        details=f"rest_x={s_rest}, slid_x={s_slid}",
    )
    ctx.expect_contact(slide, body, name="power switch seated on shell")

    return ctx.report()


object_model = build_object_model()
