"""Collapsible welded-wire pet travel crate.

Open welded-wire cage lattice body with rigid corner posts, top and
bottom perimeter rails, grid wire bars on all four walls and roof,
thin floor pan tray, hinged wire-grate front door, and wire top
carry handle.

Layout: +X = front (door), +Y = left, +Z = up. Ground at z = 0.
"""

from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ──────────────────────────────────────────────────────────── dimensions
CAGE_L = 0.58   # length  (X, front-to-back)
CAGE_W = 0.40   # width   (Y, side-to-side)
CAGE_H = 0.38   # height  (Z, ground to top rail center)

POST_R = 0.005   # corner post radius
RAIL_R = 0.004   # perimeter rail radius
WIRE_R = 0.0025  # wall / roof grid bar radius

FLOOR_T = 0.005   # floor pan base thickness
FLOOR_LIP = 0.010  # floor pan lip height
FLOOR_WALL = 0.004  # lip wall thickness

BAR_SPACING = 0.055  # cage bar centre-to-centre pitch

HL = CAGE_L / 2.0
HW = CAGE_W / 2.0

# Z references
BOT_RAIL_Z = FLOOR_T + RAIL_R          # 0.009
TOP_RAIL_Z = CAGE_H - RAIL_R            # 0.376
POST_Z_BOT = 0.0
POST_Z_TOP = CAGE_H

BAR_Z_BOT = BOT_RAIL_Z + RAIL_R         # 0.013
BAR_Z_TOP = TOP_RAIL_Z - RAIL_R         # 0.372

# wire door panel (nearly full front face)
DOOR_H = BAR_Z_TOP - BAR_Z_BOT - 0.008  # 0.351
DOOR_W = CAGE_W - 2 * POST_R - 0.016    # 0.374
BORDER_R = 0.0045
DOOR_BAR_R = 0.003
N_VBARS = 7
N_HBARS = 5

HINGE_X = HL - POST_R                   # 0.285
HINGE_Y = HW - POST_R - 0.005           # 0.190
HINGE_Z = BAR_Z_BOT                     # 0.013

# carry handle
HANDLE_SPAN = 0.14
HANDLE_RISE = 0.055
HANDLE_WIRE_R = 0.004

# ──────────────────────────────────────────────────────────── materials
CHROME = Material(name="chrome_wire", rgba=(0.72, 0.73, 0.74, 1.0))
BLACK_WIRE = Material(name="black_coated_wire", rgba=(0.10, 0.10, 0.11, 1.0))
DARK_METAL = Material(name="dark_metal", rgba=(0.22, 0.22, 0.23, 1.0))
PAN_MAT = Material(name="black_pan_plastic", rgba=(0.13, 0.13, 0.14, 1.0))


# ──────────────────────────────────────────────────────────── helpers
def _bar(body, direction, center, length, radius, material, name):
    """Emit one axis-aligned wire-bar cylinder on *body*."""
    if direction == "z":
        rpy = (0.0, 0.0, 0.0)
    elif direction == "x":
        rpy = (0.0, math.pi / 2.0, 0.0)
    else:  # "y"
        rpy = (math.pi / 2.0, 0.0, 0.0)
    body.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=rpy),
        material=material,
        name=name,
    )


def _floor_pan_solid() -> cq.Workplane:
    """Thin plastic floor-pan tray with a raised perimeter lip."""
    pan_l = CAGE_L - 0.004
    pan_w = CAGE_W - 0.004
    base = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, FLOOR_T / 2.0))
        .box(pan_l, pan_w, FLOOR_T)
    )
    lip = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, FLOOR_T))
        .rect(pan_l, pan_w)
        .rect(pan_l - 2 * FLOOR_WALL, pan_w - 2 * FLOOR_WALL)
        .extrude(FLOOR_LIP)
    )
    return base.union(lip)


def _linspace(start, stop, n):
    """Return *n* evenly-spaced values from *start* to *stop* inclusive."""
    if n <= 1:
        return [start]
    step = (stop - start) / (n - 1)
    return [start + k * step for k in range(n)]


# ──────────────────────────────────────────────────────────── model
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wire_pet_travel_crate")

    body = model.part("kennel_body")

    # ── floor pan ──────────────────────────────────────────────────────
    body.visual(
        mesh_from_cadquery(_floor_pan_solid(), "floor_pan"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=PAN_MAT,
        name="floor_pan",
    )

    # ── corner posts (4) ───────────────────────────────────────────────
    post_len = POST_Z_TOP - POST_Z_BOT
    post_z_mid = (POST_Z_BOT + POST_Z_TOP) / 2.0
    corners = ((-1, -1), (-1, 1), (1, -1), (1, 1))
    for i, (sx, sy) in enumerate(corners):
        body.visual(
            Cylinder(radius=POST_R, length=post_len),
            origin=Origin(xyz=(sx * (HL - POST_R), sy * (HW - POST_R), post_z_mid)),
            material=CHROME,
            name=f"corner_post_{i}",
        )

    # ── perimeter rails (8: 4 bottom + 4 top) ─────────────────────────
    rail_x_len = CAGE_L - 2 * POST_R   # side rails (along X)
    rail_y_len = CAGE_W - 2 * POST_R   # end rails  (along Y)
    cx = HL - POST_R                    # corner X
    cy = HW - POST_R                    # corner Y

    rail_specs = [
        # (name, direction, center)
        ("bottom_rail_left",  "x", (0.0,  -cy, BOT_RAIL_Z)),
        ("bottom_rail_right", "x", (0.0,   cy, BOT_RAIL_Z)),
        ("bottom_rail_back",  "y", (-cx, 0.0,  BOT_RAIL_Z)),
        ("bottom_rail_front", "y", ( cx, 0.0,  BOT_RAIL_Z)),
        ("top_rail_left",     "x", (0.0,  -cy, TOP_RAIL_Z)),
        ("top_rail_right",    "x", (0.0,   cy, TOP_RAIL_Z)),
        ("top_rail_back",     "y", (-cx, 0.0,  TOP_RAIL_Z)),
        ("top_rail_front",    "y", ( cx, 0.0,  TOP_RAIL_Z)),
    ]
    for rname, rdir, rcenter in rail_specs:
        rlen = rail_x_len if rdir == "x" else rail_y_len
        _bar(body, rdir, rcenter, rlen, RAIL_R, CHROME, rname)

    # ── cage wall & roof bars (loop-emitted) ──────────────────────────
    v_bar_len = BAR_Z_TOP - BAR_Z_BOT
    v_bar_z_mid = (BAR_Z_BOT + BAR_Z_TOP) / 2.0

    # bar inset from posts
    x_inset = POST_R + BAR_SPACING
    y_inset = POST_R + BAR_SPACING
    x_span = CAGE_L - 2 * (HL - x_inset)  # usable X range for bars
    y_span = CAGE_W - 2 * (HW - y_inset)  # usable Y range for bars

    x_bar_start = -(HL - x_inset)
    x_bar_end   =  (HL - x_inset)
    y_bar_start = -(HW - y_inset)
    y_bar_end   =  (HW - y_inset)

    n_side_v = max(2, int((x_bar_end - x_bar_start) / BAR_SPACING) + 1)
    n_end_v  = max(2, int((y_bar_end - y_bar_start) / BAR_SPACING) + 1)

    x_positions = _linspace(x_bar_start, x_bar_end, n_side_v)
    y_positions = _linspace(y_bar_start, y_bar_end, n_end_v)

    # horizontal bar Z positions (between bottom and top rails)
    h_z_start = BAR_Z_BOT + BAR_SPACING
    h_z_end   = BAR_Z_TOP - BAR_SPACING * 0.4
    n_wall_h  = max(2, int((h_z_end - h_z_start) / BAR_SPACING) + 1)
    h_z_positions = _linspace(h_z_start, h_z_end, n_wall_h)

    vi = 0  # vertical bar counter
    hi = 0  # horizontal bar counter

    # --- left side wall (Y = -cy) ---
    wall_y_l = -cy
    for x in x_positions:
        _bar(body, "z", (x, wall_y_l, v_bar_z_mid), v_bar_len, WIRE_R, BLACK_WIRE, f"cage_bar_v_{vi}"); vi += 1
    for z in h_z_positions:
        _bar(body, "x", (0.0, wall_y_l, z), rail_x_len, WIRE_R, BLACK_WIRE, f"cage_bar_h_{hi}"); hi += 1

    # --- right side wall (Y = +cy) ---
    wall_y_r = cy
    for x in x_positions:
        _bar(body, "z", (x, wall_y_r, v_bar_z_mid), v_bar_len, WIRE_R, BLACK_WIRE, f"cage_bar_v_{vi}"); vi += 1
    for z in h_z_positions:
        _bar(body, "x", (0.0, wall_y_r, z), rail_x_len, WIRE_R, BLACK_WIRE, f"cage_bar_h_{hi}"); hi += 1

    # --- back wall (X = -cx) ---
    wall_x_b = -cx
    for y in y_positions:
        _bar(body, "z", (wall_x_b, y, v_bar_z_mid), v_bar_len, WIRE_R, BLACK_WIRE, f"cage_bar_v_{vi}"); vi += 1
    for z in h_z_positions:
        _bar(body, "y", (wall_x_b, 0.0, z), rail_y_len, WIRE_R, BLACK_WIRE, f"cage_bar_h_{hi}"); hi += 1

    # --- front wall: bars only above the door opening ---
    #     Door top in world Z ≈ HINGE_Z + DOOR_H ≈ 0.364
    #     Available space above door to top rail is very small (< BAR_SPACING)
    #     so the door panel itself IS the front face. No extra front bars.

    # --- roof bars ---
    roof_z = TOP_RAIL_Z + RAIL_R   # 0.380
    for y in y_positions:
        _bar(body, "x", (0.0, y, roof_z), rail_x_len, WIRE_R, BLACK_WIRE, f"cage_bar_h_{hi}"); hi += 1
    for x in x_positions:
        _bar(body, "y", (x, 0.0, roof_z), rail_y_len, WIRE_R, BLACK_WIRE, f"cage_bar_h_{hi}"); hi += 1

    # ── wire carry handle ─────────────────────────────────────────────
    # Cross-bar connects risers to roof lattice
    _bar(body, "x", (0.0, 0.0, roof_z), HANDLE_SPAN, HANDLE_WIRE_R, CHROME, "handle_cross_bar")

    # Two vertical risers
    for i, sx in enumerate((-1.0, 1.0)):
        _bar(body, "z",
             (sx * HANDLE_SPAN / 2.0, 0.0, roof_z + HANDLE_RISE / 2.0),
             HANDLE_RISE, HANDLE_WIRE_R, CHROME, f"handle_riser_{i}")

    # Grip bar
    _bar(body, "x",
         (0.0, 0.0, roof_z + HANDLE_RISE),
         HANDLE_SPAN, HANDLE_WIRE_R, BLACK_WIRE, "handle_grip_bar")

    # ─────────────────────────────────────────────────── wire door
    door = model.part("wire_door")

    yb0, yb1 = -0.006, -(DOOR_W - 0.006)
    zb0, zb1 = 0.006, DOOR_H - 0.006
    y_mid = (yb0 + yb1) / 2.0
    z_mid = (zb0 + zb1) / 2.0

    # heavy border wires
    for i, yb in enumerate((yb0, yb1)):
        door.visual(
            Cylinder(radius=BORDER_R, length=(zb1 - zb0) + 2 * BORDER_R),
            origin=Origin(xyz=(0.0, yb, z_mid)),
            material=BLACK_WIRE,
            name=f"door_border_vertical_{i}",
        )
    for i, zb in enumerate((zb0, zb1)):
        door.visual(
            Cylinder(radius=BORDER_R, length=(yb0 - yb1) + 2 * BORDER_R),
            origin=Origin(xyz=(0.0, y_mid, zb), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=BLACK_WIRE,
            name=f"door_border_horizontal_{i}",
        )

    # inner wire grid
    for i in range(N_VBARS):
        yv = yb0 + (i + 1) * (yb1 - yb0) / (N_VBARS + 1)
        door.visual(
            Cylinder(radius=DOOR_BAR_R, length=(zb1 - zb0)),
            origin=Origin(xyz=(0.0, yv, z_mid)),
            material=BLACK_WIRE,
            name=f"door_wire_vertical_{i}",
        )
    for i in range(N_HBARS):
        zh = zb0 + (i + 1) * (zb1 - zb0) / (N_HBARS + 1)
        door.visual(
            Cylinder(radius=DOOR_BAR_R, length=(yb0 - yb1)),
            origin=Origin(xyz=(0.0, y_mid, zh), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=BLACK_WIRE,
            name=f"door_wire_horizontal_{i}",
        )

    # hinge knuckles (wrap around the front corner post)
    for i, zk_frac in enumerate((0.15, 0.85)):
        zk = DOOR_H * zk_frac
        door.visual(
            Box((0.012, 0.016, 0.022)),
            origin=Origin(xyz=(-0.005, 0.002, zk)),
            material=DARK_METAL,
            name=f"door_hinge_knuckle_{i}",
        )

    # squeeze-latch grip on the free edge (set inward to clear corner post)
    door.visual(
        Box((0.014, 0.018, 0.05)),
        origin=Origin(xyz=(0.010, yb1 + 0.012, z_mid)),
        material=DARK_METAL,
        name="door_latch_grip",
    )

    # ─────────────────────────────────────────────────── articulation
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        # no wall tilt for a vertical-wire cage (unlike the sloped shell)
        origin=Origin(xyz=(HINGE_X, HINGE_Y, HINGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=0.0, upper=1.6),
    )

    return model


# ──────────────────────────────────────────────────────────── tests
def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("kennel_body")
    door = object_model.get_part("wire_door")
    hinge = object_model.get_articulation("body_to_door")

    # hinge knuckles intentionally wrap around the front-right corner post
    for i in range(2):
        ctx.allow_overlap(
            body, door,
            elem_a="corner_post_3",
            elem_b=f"door_hinge_knuckle_{i}",
            reason="door hinge knuckles wrap around the front corner post for pivoting",
        )

    # ── structural-variant proof: cage lattice bars exist ──
    body_visual_names = [v.name for v in body.visuals]
    v_bars = [n for n in body_visual_names if n.startswith("cage_bar_v_")]
    h_bars = [n for n in body_visual_names if n.startswith("cage_bar_h_")]

    ctx.check(
        "cage_has_vertical_wire_bars",
        len(v_bars) >= 20,
        f"found {len(v_bars)} vertical cage bars (expected ≥20)",
    )
    ctx.check(
        "cage_has_horizontal_wire_bars",
        len(h_bars) >= 15,
        f"found {len(h_bars)} horizontal cage bars (expected ≥15)",
    )
    ctx.check(
        "no_molded_shell_in_wire_crate",
        "kennel_upper_shell" not in body_visual_names and "lower_tub" not in body_visual_names,
        "molded shell parts must not exist in the wire cage variant",
    )
    ctx.check(
        "body_to_door_is_revolute",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        f"hinge type={hinge.articulation_type}",
    )

    # ── door closed: grate covers the front opening ──
    with ctx.pose({hinge: 0.0}):
        ctx.expect_overlap(door, body, axes="yz", min_overlap=0.10)
        aabb = ctx.part_world_aabb(door)
        ctx.check(
            "closed_door_at_front_face",
            aabb is not None and aabb[1][0] > HL - 0.05,
            f"door aabb={aabb}",
        )

    # ── door open: free edge swings outward past the cage ──
    with ctx.pose({hinge: 1.5}):
        aabb = ctx.part_world_aabb(door)
        ctx.check(
            "open_door_swings_outward",
            aabb is not None and aabb[1][0] > HL + 0.05,
            f"door aabb={aabb}",
        )

    return ctx.report()


object_model = build_object_model()
