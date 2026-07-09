from __future__ import annotations

# Red handheld BRICK GAME console (classic Tetris "Brick Game").
# Frame: long axis = X (length ~0.16), width = Y (~0.075), thickness = Z (~0.018).
#   Front control face points up at +Z; the LCD sits toward +X (top), controls
#   toward -X (bottom). Buttons press DOWN into the slab along -Z.
# Articulations:
#   - D-pad: REVOLUTE rocker tilting a small amount about the Y axis.
#   - 6 round yellow buttons: each PRISMATIC press straight down (~2 mm).
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
    # Build a rounded-rect footprint extruded in Z, with rounded long edges,
    # then shave the back face at the top end to create the wedge.
    solid = (
        cq.Workplane("XY")
        .rect(LEN, WID)
        .extrude(THK)
        .translate((0.0, 0.0, -THK / 2.0))
        .edges("|Z")
        .fillet(0.010)
    )
    # Wedge: shave a thin tapered slice off the BACK (-Z) of the upper (+X) end
    # so the screen end is slightly thinner -> classic angled slab. The cutter
    # is a wedge prism placed below the back face so it only removes material
    # near the upper end and never fragments the slab.
    drop = 0.004  # how much thinner the top end becomes
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
    # Cavity cut into the front face to seat the gray screen bezel.
    rx, ry = 0.052, 0.052
    return (
        cq.Workplane("XY")
        .center(0.045, 0.0)
        .rect(rx, ry)
        .extrude(THK)  # cut downward from above the top face
        .translate((0.0, 0.0, TOP_Z - 0.004))
    )


def _body_mesh():
    solid = _slab_solid().cut(_recess_box())
    return mesh_from_cadquery(solid, "shell")


def _grid_mesh():
    # Faint pixel grid: thin ridges on the dark screen face.
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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="brick_game_console")

    red = model.material("shell_red", rgba=(0.78, 0.12, 0.12, 1.0))
    bezel_gray = model.material("bezel_gray", rgba=(0.55, 0.56, 0.58, 1.0))
    screen = model.material("screen_dark", rgba=(0.30, 0.36, 0.30, 1.0))
    grid_green = model.material("grid_green", rgba=(0.18, 0.24, 0.18, 1.0))
    yellow = model.material("button_yellow", rgba=(0.95, 0.78, 0.10, 1.0))
    black = model.material("dpad_black", rgba=(0.10, 0.10, 0.11, 1.0))

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
    dpad_z = TOP_Z + 0.001  # sits proud of the lower face (which is at full THK/2)

    dpad = model.part("dpad")
    arm = 0.0095
    cross_h = 0.005
    # plus shape from two crossed bars
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

    # ---------------- yellow round buttons (prismatic) ----------------
    # Right cluster: a 2x2 of larger action buttons + two smaller buttons below.
    btn_r_big = 0.0075
    btn_r_small = 0.0050
    btn_h = 0.0045
    face_z = TOP_Z  # buttons protrude above the front face

    # 2x2 action buttons (centers), on the lower-right region.
    grid_cx, grid_cy = -0.030, -0.020
    dx, dy = 0.020, 0.020
    big_specs = [
        ("btn_0", grid_cx - dx / 2.0, grid_cy + dy / 2.0, btn_r_big),
        ("btn_1", grid_cx + dx / 2.0, grid_cy + dy / 2.0, btn_r_big),
        ("btn_2", grid_cx - dx / 2.0, grid_cy - dy / 2.0, btn_r_big),
        ("btn_3", grid_cx + dx / 2.0, grid_cy - dy / 2.0, btn_r_big),
    ]
    # two small start/select buttons further down the slab
    small_specs = [
        ("btn_4", -0.060, -0.012, btn_r_small),
        ("btn_5", -0.060, -0.032, btn_r_small),
    ]
    all_specs = big_specs + small_specs

    button_parts = []
    for name, bx, by, r in all_specs:
        b = model.part(name)
        cap = CylinderGeometry(r, btn_h, radial_segments=28)
        b.visual(
            mesh_from_geometry(cap, f"{name}_cap"),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=yellow,
            name=f"{name}_cap",
        )
        b.inertial = Inertial.from_geometry(
            Cylinder(radius=r, length=btn_h), mass=0.0015
        )
        joint = model.articulation(
            f"{name}_press",
            ArticulationType.PRISMATIC,
            parent=body,
            child=b,
            # button cap center sits just above the front face; well lets it press in.
            origin=Origin(xyz=(bx, by, face_z + btn_h / 2.0 - 0.0006)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=1.0, velocity=0.05, lower=0.0, upper=BTN_TRAVEL
            ),
        )
        button_parts.append((b, joint, name))

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
    bezel = body.get_visual("bezel")
    scr = body.get_visual("screen_face")
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
    # cross sits proud of the face
    ctx.expect_contact(dpad, body, name="dpad seated on the front face")
    aabb0 = ctx.part_world_aabb(dpad)
    with ctx.pose({dpad_joint: 0.18}):
        aabb1 = ctx.part_world_aabb(dpad)
    # tilting about Y raises one X end and lowers the other -> top Z changes.
    ctx.check(
        "d-pad rocker tilts (z extent changes when posed)",
        abs(aabb1[1][2] - aabb0[1][2]) > 0.0008,
        details=f"rest top z={aabb0[1][2]}, tilted top z={aabb1[1][2]}",
    )

    # ---- yellow buttons: sit on the front face and press straight down ----
    button_names = ["btn_0", "btn_1", "btn_2", "btn_3", "btn_4", "btn_5"]
    for nm in button_names:
        b = object_model.get_part(nm)
        j = object_model.get_articulation(f"{nm}_press")
        ctx.expect_contact(b, body, name=f"{nm} rests on the front face")
        # button above the lower face center (front side, +Z)
        bp = ctx.part_world_position(b)
        ctx.check(
            f"{nm} mounted on the front face",
            bp is not None and bp[2] > 0.0,
            details=f"{nm} pos={bp}",
        )

    # Sample one button and confirm a downward press drops its z.
    sample = object_model.get_part("btn_0")
    sample_j = object_model.get_articulation("btn_0_press")
    rest_z = ctx.part_world_position(sample)[2]
    with ctx.pose({sample_j: BTN_TRAVEL}):
        pressed_z = ctx.part_world_position(sample)[2]
    ctx.check(
        "btn_0 presses straight down",
        pressed_z < rest_z - 0.0015,
        details=f"rest_z={rest_z}, pressed_z={pressed_z}",
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
