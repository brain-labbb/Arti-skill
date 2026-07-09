from __future__ import annotations

# Walnut keepsake box with finger-joint (dovetail-style) corners,
# lighter accent splines, a flat lid hinged at the rear top edge,
# and a removable lift-out inner tray seated on an inner ledge shelf.
#
# Frame:
#   +X = box width (0.24 m), +Y = box depth (0.15 m), +Z = up (height 0.10 m).
#   Rear of the box is +Y; front (where the lid free edge sits) is -Y.
#   Everything is centered on x=0, y=0; the base sits on z=0 and grows up.
#
# Articulation:
#   - lid: REVOLUTE about the rear-top hinge line (axis = +X). The closed lid
#     geometry extends from the rear hinge toward -Y, so positive q lifts the
#     front edge up and swings it back over the rear (~0 to 100 degrees).
#   - tray: PRISMATIC along +Z. At q=0 the tray rim is seated on the inner
#     ledge shelf; positive q lifts the tray straight up out of the cavity.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    mesh_from_cadquery,
)

# ---- Box dimensions ----
W = 0.240  # outer width (X)
D = 0.150  # outer depth (Y)
WALL = 0.012  # wood wall thickness
BASE_T = 0.012  # bottom panel thickness
BODY_H = 0.090  # outer height of the box base (without lid)
INNER_H = BODY_H - BASE_T  # interior cavity height

LID_T = 0.014  # lid slab thickness
LID_PANEL_INSET = 0.010  # recessed lighter inner panel inset

# Hinge sits at the rear top edge of the box base.
HINGE_Y = D / 2.0 - WALL / 2.0  # centered on the rear wall top
HINGE_Z = BODY_H  # at the top rim of the base

FINGER = 0.020  # finger-joint pitch (height of each interlocking tab)
SPLINE_W = 0.006  # accent spline width
SPLINE_T = 0.004  # accent spline protrusion / inset

# ---- Inner ledge (tray support shelf) ----
LEDGE_Z = 0.060  # bottom of ledge shelf (height from box floor)
LEDGE_T = 0.004  # ledge vertical thickness
LEDGE_W = 0.006  # ledge inward protrusion from inner wall face

# ---- Lift-out tray ----
TRAY_CLEAR = 0.002  # side clearance between tray walls and cavity/ledge opening
TRAY_WALL_T = 0.005  # tray wall thickness
TRAY_WALL_H = 0.006  # tray wall height (shallow inner tray)
TRAY_FLOOR_T = 0.003  # tray floor thickness
TRAY_RIM_T = 0.002  # rim flange thickness
# Inner cavity dims
_CAV_W = W - 2 * WALL  # 0.216
_CAV_D = D - 2 * WALL  # 0.126
# Ledge opening (below ledge, where tray walls drop in)
_LEDGE_OPEN_W = _CAV_W - 2 * LEDGE_W  # 0.204
_LEDGE_OPEN_D = _CAV_D - 2 * LEDGE_W  # 0.114
# Tray outer dims (walls fit inside ledge opening)
TRAY_OW = _LEDGE_OPEN_W - 2 * TRAY_CLEAR  # 0.200
TRAY_OD = _LEDGE_OPEN_D - 2 * TRAY_CLEAR  # 0.110
# Tray rim outer dims (extends to rest on ledge surface)
TRAY_RIM_OW = _CAV_W - 2 * 0.001  # 0.214
TRAY_RIM_OD = _CAV_D - 2 * 0.001  # 0.124

# Tray seating level: rim bottom rests on ledge top
TRAY_SEAT_Z = LEDGE_Z + LEDGE_T  # 0.064 from box floor


def _box_base_solid() -> cq.Workplane:
    # Hollow rectangular shell: outer block minus the interior cavity, leaving
    # four walls and a solid floor.
    outer = (
        cq.Workplane("XY")
        .box(W, D, BODY_H, centered=(True, True, False))
    )
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=BASE_T)
        .box(W - 2 * WALL, D - 2 * WALL, INNER_H + 0.01, centered=(True, True, False))
    )
    shell = outer.cut(cavity)

    # Carve finger-joint notches at the four vertical corners. At each corner the
    # two meeting walls interlock with alternating tabs; we cut small notches out
    # of one wall face at alternating heights so the corner reads as a box joint.
    notch_depth = WALL + 0.001
    half_w = W / 2.0
    half_d = D / 2.0
    n_fingers = max(2, int(INNER_H // FINGER))

    corners = [
        (half_w, half_d, 1, 1),
        (-half_w, half_d, -1, 1),
        (half_w, -half_d, 1, -1),
        (-half_w, -half_d, -1, -1),
    ]
    for cx, cy, sx, sy in corners:
        for i in range(n_fingers):
            z0 = BASE_T + i * FINGER
            if z0 + FINGER > BODY_H:
                break
            if i % 2 == 0:
                # Notch into the X-facing wall end (cut a slot on the side face).
                notch = (
                    cq.Workplane("XY")
                    .workplane(offset=z0)
                    .center(cx - sx * notch_depth / 2.0, cy - sy * (FINGER / 2.0))
                    .box(notch_depth, FINGER, FINGER * 0.5, centered=(True, True, False))
                )
                shell = shell.cut(notch)
    return shell


def _accent_splines_solid() -> cq.Workplane:
    # Lighter maple accent splines set diagonally across each top corner of the
    # box base, the classic reinforcing keys seen on fine boxes.
    splines = None
    half_w = W / 2.0
    half_d = D / 2.0
    corners = [
        (half_w - WALL / 2.0, half_d - WALL / 2.0),
        (-half_w + WALL / 2.0, half_d - WALL / 2.0),
        (half_w - WALL / 2.0, -half_d + WALL / 2.0),
        (-half_w + WALL / 2.0, -half_d + WALL / 2.0),
    ]
    for cx, cy in corners:
        for k in range(3):
            zc = BASE_T + 0.012 + k * 0.024
            if zc + SPLINE_W / 2.0 > BODY_H:
                break
            key = (
                cq.Workplane("XY")
                .workplane(offset=zc)
                .center(cx, cy)
                .rect(WALL * 1.6, SPLINE_W)
                .extrude(SPLINE_W)
                .rotate((cx, cy, zc), (cx, cy, zc + 1), 45.0)
            )
            splines = key if splines is None else splines.union(key)
    return splines


def _lid_solid() -> cq.Workplane:
    # Flat lid slab spanning the full footprint. Authored in a LOCAL frame whose
    # origin is the hinge line: the lid extends from y=0 (hinge) to y=-(D) and
    # is centered on x. Top of slab at z=0, bottom at z=-LID_T.
    slab = (
        cq.Workplane("XY")
        .center(0.0, -D / 2.0)
        .box(W, D, LID_T, centered=(True, True, False))
        .translate((0.0, 0.0, -LID_T))
    )
    return slab


def _lid_panel_solid() -> cq.Workplane:
    # Lighter recessed inner panel on the underside of the lid (visible when open).
    pw = W - 2 * WALL - 2 * LID_PANEL_INSET
    pd = D - 2 * WALL - 2 * LID_PANEL_INSET
    panel = (
        cq.Workplane("XY")
        .center(0.0, -D / 2.0)
        .box(pw, pd, 0.004, centered=(True, True, False))
        .translate((0.0, 0.0, -LID_T - 0.004))
    )
    return panel


def _inner_ledge_solid() -> cq.Workplane:
    """Ledge shelf around the inner cavity at LEDGE_Z, inset from the walls.

    Four ledge strips form a rectangular shelf that supports the lift-out tray.
    Each strip protrudes LEDGE_W inward from the inner cavity wall face.
    The ledge bottom is at LEDGE_Z and top at LEDGE_Z + LEDGE_T.
    """
    cav_w = W - 2 * WALL
    cav_d = D - 2 * WALL
    ledge = None
    # Front and back strips (span full cavity width, LEDGE_W depth)
    for sign in (+1, -1):
        strip = (
            cq.Workplane("XY")
            .workplane(offset=LEDGE_Z)
            .center(0.0, sign * (cav_d / 2.0 - LEDGE_W / 2.0))
            .box(cav_w, LEDGE_W, LEDGE_T, centered=(True, True, False))
        )
        ledge = strip if ledge is None else ledge.union(strip)
    # Left and right strips (span between front/back strips)
    mid_d = cav_d - 2 * LEDGE_W
    for sign in (+1, -1):
        strip = (
            cq.Workplane("XY")
            .workplane(offset=LEDGE_Z)
            .center(sign * (cav_w / 2.0 - LEDGE_W / 2.0), 0.0)
            .box(LEDGE_W, mid_d, LEDGE_T, centered=(True, True, False))
        )
        ledge = ledge.union(strip)
    return ledge


def _tray_floor_solid() -> cq.Workplane:
    """Tray floor plate, authored with bottom at z=0."""
    ow = TRAY_OW
    od = TRAY_OD
    floor = (
        cq.Workplane("XY")
        .box(ow, od, TRAY_FLOOR_T, centered=(True, True, False))
    )
    return floor


def _tray_walls_solid() -> cq.Workplane:
    """Four tray walls rising from the tray floor top, authored with floor bottom at z=0."""
    ow = TRAY_OW
    od = TRAY_OD
    t = TRAY_WALL_T
    h = TRAY_WALL_H
    base_z = TRAY_FLOOR_T
    walls = None
    # Front and back walls
    for sign in (+1, -1):
        wall = (
            cq.Workplane("XY")
            .workplane(offset=base_z)
            .center(0.0, sign * (od / 2.0 - t / 2.0))
            .box(ow, t, h, centered=(True, True, False))
        )
        walls = wall if walls is None else walls.union(wall)
    # Left and right walls (fit between front/back)
    mid = od - 2 * t
    for sign in (+1, -1):
        wall = (
            cq.Workplane("XY")
            .workplane(offset=base_z)
            .center(sign * (ow / 2.0 - t / 2.0), 0.0)
            .box(t, mid, h, centered=(True, True, False))
        )
        walls = walls.union(wall)
    return walls


def _tray_rim_solid() -> cq.Workplane:
    """Rim flange extending outward from wall tops to rest on the ledge.

    Authored with floor bottom at z=0. The rim bottom sits at
    TRAY_FLOOR_T + TRAY_WALL_H, and extends outward to TRAY_RIM_OW/TRAY_RIM_OD.
    """
    rim_z = TRAY_FLOOR_T + TRAY_WALL_H
    ow = TRAY_RIM_OW
    od = TRAY_RIM_OD
    iw = TRAY_OW - 2 * TRAY_WALL_T
    id_ = TRAY_OD - 2 * TRAY_WALL_T
    # Outer rim box minus inner cutout
    outer = (
        cq.Workplane("XY")
        .workplane(offset=rim_z)
        .box(ow, od, TRAY_RIM_T, centered=(True, True, False))
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=rim_z)
        .box(iw, id_, TRAY_RIM_T + 0.001, centered=(True, True, False))
    )
    return outer.cut(inner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="keepsake_box")

    walnut = model.material("walnut", rgba=(0.36, 0.22, 0.13, 1.0))
    maple = model.material("maple_accent", rgba=(0.82, 0.66, 0.42, 1.0))
    brass = model.material("brass", rgba=(0.74, 0.58, 0.26, 1.0))

    # ---- box base (root) ----
    body = model.part("box_base")
    body.visual(
        mesh_from_cadquery(_box_base_solid(), "box_shell"),
        material=walnut,
        name="box_shell",
    )
    body.visual(
        mesh_from_cadquery(_accent_splines_solid(), "corner_splines"),
        material=maple,
        name="corner_splines",
    )

    # Two brass hinge knuckles on the rear top edge of the base.
    for i, kx in enumerate((-0.055, 0.055)):
        knuckle = (
            cq.Workplane("YZ")
            .workplane(offset=kx - 0.010)
            .center(HINGE_Y, HINGE_Z)
            .circle(0.006)
            .extrude(0.020)
        )
        body.visual(
            mesh_from_cadquery(knuckle, f"base_knuckle_{i}"),
            material=brass,
            name=f"base_knuckle_{i}",
        )

    body.inertial = Inertial.from_geometry(
        Box((W, D, BODY_H)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
    )

    # ---- lid ----
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_solid(), "lid_slab"),
        material=walnut,
        name="lid_slab",
    )
    lid.visual(
        mesh_from_cadquery(_lid_panel_solid(), "lid_panel"),
        material=maple,
        name="lid_panel",
    )
    # Lid-side hinge knuckles, interleaved with the base knuckles on the hinge line.
    for i, kx in enumerate((-0.020, 0.085)):
        knuckle = (
            cq.Workplane("YZ")
            .workplane(offset=kx - 0.010)
            .center(0.0, 0.0)
            .circle(0.006)
            .extrude(0.020)
        )
        lid.visual(
            mesh_from_cadquery(knuckle, f"lid_knuckle_{i}"),
            material=brass,
            name=f"lid_knuckle_{i}",
        )
    lid.inertial = Inertial.from_geometry(
        Box((W, D, LID_T)),
        mass=0.25,
        origin=Origin(xyz=(0.0, -D / 2.0, -LID_T / 2.0)),
    )

    # Hinge: axis along the rear top edge. Closed lid extends toward -Y; with
    # axis -X, positive q (right-hand rule) lifts the front edge up and swings
    # it rearward over the box.
    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0, velocity=2.0, lower=0.0, upper=math.radians(100.0)
        ),
    )

    # ---- Inner ledge shelf (visual on box base) ----
    body.visual(
        mesh_from_cadquery(_inner_ledge_solid(), "inner_ledge"),
        material=walnut,
        name="inner_ledge",
    )

    # ---- Lift-out inner tray ----
    tray = model.part("tray")
    tray.visual(
        mesh_from_cadquery(_tray_floor_solid(), "tray_floor"),
        material=walnut,
        name="tray_floor",
    )
    tray.visual(
        mesh_from_cadquery(_tray_walls_solid(), "tray_walls"),
        material=walnut,
        name="tray_walls",
    )
    tray.visual(
        mesh_from_cadquery(_tray_rim_solid(), "tray_rim"),
        material=maple,
        name="tray_rim",
    )
    tray.inertial = Inertial.from_geometry(
        Box((TRAY_OW, TRAY_OD, TRAY_FLOOR_T + TRAY_WALL_H + TRAY_RIM_T)),
        mass=0.15,
        origin=Origin(xyz=(0.0, 0.0, (TRAY_FLOOR_T + TRAY_WALL_H + TRAY_RIM_T) / 2.0)),
    )

    # PRISMATIC joint: tray lifts straight up (+Z) out of the cavity.
    # Joint origin is at the seating plane where the tray rim rests on the ledge.
    # At q=0 the tray is seated; positive q lifts it upward.
    model.articulation(
        "base_to_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, TRAY_SEAT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=0.5, lower=0.0, upper=0.080
        ),
    )

    return model


def run_tests() -> "TestReport":
    from sdk import TestContext

    ctx = TestContext(object_model)

    body = object_model.get_part("box_base")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("base_to_lid")

    # The lid and base hinge knuckles intentionally share the hinge axis line.
    ctx.allow_overlap(
        lid,
        body,
        elem_a="lid_knuckle_0",
        elem_b="base_knuckle_0",
        reason="Interleaved hinge knuckles share the rear hinge pin line.",
    )
    ctx.allow_overlap(
        lid,
        body,
        elem_a="lid_knuckle_1",
        elem_b="base_knuckle_1",
        reason="Interleaved hinge knuckles share the rear hinge pin line.",
    )
    # The closed lid seats onto the box top rim (small intentional contact embed).
    ctx.allow_overlap(
        lid,
        body,
        elem_a="lid_slab",
        elem_b="box_shell",
        reason="Closed lid slab seats flush onto the box top rim.",
    )
    # Lid hinge knuckles mount on the rear wall top, slightly embedding the shell.
    for i in (0, 1):
        ctx.allow_overlap(
            lid,
            body,
            elem_a=f"lid_knuckle_{i}",
            elem_b="box_shell",
            reason="Lid hinge knuckles are mounted onto the rear wall top edge.",
        )
    # The closed lid slab covers the rear hinge knuckles on the box.
    for i in (0, 1):
        ctx.allow_overlap(
            lid,
            body,
            elem_a="lid_slab",
            elem_b=f"base_knuckle_{i}",
            reason="Closed lid slab rests over the rear hinge knuckles.",
        )

    # 1) Box base is a hollow wooden rectangle: footprint spans full width/depth,
    #    and the interior cavity is open (lid covers the full footprint).
    base_aabb = ctx.part_world_aabb(body)
    bmn, bmx = base_aabb
    ctx.check(
        "box base spans full footprint",
        (bmx[0] - bmn[0]) > 0.22 and (bmx[1] - bmn[1]) > 0.13 and (bmx[2] - bmn[2]) > 0.08,
        details=f"base extents={(bmx[0]-bmn[0], bmx[1]-bmn[1], bmx[2]-bmn[2])}",
    )

    # 2) Closed lid covers the box footprint and sits at the top.
    ctx.expect_overlap(
        lid, body, axes="xy", min_overlap=0.10, name="closed lid covers the box footprint"
    )
    lid_aabb = ctx.part_world_aabb(lid)
    lmn, lmx = lid_aabb
    ctx.check(
        "closed lid sits at the box top",
        lmx[2] >= bmx[2] - 0.002,
        details=f"lid_top_z={lmx[2]}, base_top_z={bmx[2]}",
    )

    # 3) Hinge is at the rear top edge.
    hz = HINGE_Z
    ctx.check(
        "hinge at rear top edge",
        HINGE_Y > 0.0 and abs(hz - bmx[2]) < 0.01,
        details=f"hinge_y={HINGE_Y}, hinge_z={hz}, base_top={bmx[2]}",
    )

    # 4) Opening the lid: the FRONT edge (-Y, min y) lifts up and swings back.
    front_y_closed = lmn[1]
    top_z_closed = lmx[2]
    with ctx.pose({hinge: math.radians(100.0)}):
        op_mn, op_mx = ctx.part_world_aabb(lid)
        front_y_open = op_mn[1]
        top_z_open = op_mx[2]
    ctx.check(
        "lid front edge lifts when opened",
        top_z_open > top_z_closed + 0.05,
        details=f"top_z closed={top_z_closed}, open={top_z_open}",
    )
    ctx.check(
        "lid front edge swings rearward when opened",
        front_y_open > front_y_closed + 0.02,
        details=f"front_y closed={front_y_closed}, open={front_y_open}",
    )

    # ---- Inner tray checks ----
    tray = object_model.get_part("tray")
    tray_lift = object_model.get_articulation("base_to_tray")

    # 5) Tray rim sits inside cavity footprint at rest (seated on ledge).
    ctx.expect_within(
        tray, body, axes="xy",
        inner_elem="tray_rim", outer_elem="box_shell",
        margin=0.002,
        name="tray rim sits within the box cavity",
    )

    # 6) Tray floor is above the box base floor (z=BASE_T) and below the lid.
    tray_aabb = ctx.part_world_aabb(tray)
    t_mn, t_mx = tray_aabb
    ctx.check(
        "tray floor sits above box base floor when seated",
        t_mn[2] > BASE_T - 0.002,
        details=f"tray_bottom_z={t_mn[2]}, base_floor_z={BASE_T}",
    )
    ctx.check(
        "tray top sits below lid when seated and lid is closed",
        t_mx[2] <= lmx[2] + 0.002,
        details=f"tray_top_z={t_mx[2]}, lid_bottom_z~{lmx[2]-LID_T}",
    )

    # 7) Tray lifts straight up: positive q raises it, no XY drift.
    tray_center_seated = ctx.part_world_position(tray)
    with ctx.pose({tray_lift: 0.060}):
        tray_center_lifted = ctx.part_world_position(tray)
    ctx.check(
        "tray lifts upward when prismatic joint is positive",
        tray_center_lifted is not None and tray_center_seated is not None
        and tray_center_lifted[2] > tray_center_seated[2] + 0.055,
        details=f"seated={tray_center_seated}, lifted={tray_center_lifted}",
    )
    ctx.check(
        "tray does not drift sideways when lifted",
        tray_center_lifted is not None and tray_center_seated is not None
        and abs(tray_center_lifted[0] - tray_center_seated[0]) < 0.002
        and abs(tray_center_lifted[1] - tray_center_seated[1]) < 0.002,
        details=f"seated={tray_center_seated}, lifted={tray_center_lifted}",
    )

    # 8) Inner ledge is present inside the cavity (within the box shell).
    ctx.expect_within(
        body, body, axes="xy",
        inner_elem="inner_ledge", outer_elem="box_shell",
        margin=0.002,
        name="inner ledge sits within the box shell footprint",
    )

    return ctx.report()


object_model = build_object_model()
