from __future__ import annotations

"""Vintage industrial steel locker cabinet – variant 04.

Two full-height hinged doors (left and right) with a central sliding
tambour-style panel. Shelf boards are visible through the centre opening when
the tambour is slid open. Door gap seams frame all moving fronts.

Overall envelope ~1.6 m wide x 0.5 m deep x ~1.8 m tall, brushed/tarnished raw
steel. A hollow thin-wall (~0.02 m) carcass sits on four short splayed legs and
carries a thin riveted top cap strip. The front is divided into three bays:
left door (hinged on left edge, opens outward left), right door (hinged on
right edge, opens outward right), and a centre bay covered by a tambour panel
that slides right on a prismatic joint to reveal interior shelves.
"""

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Global dimensions (meters). Cabinet is centred on X, front face at +Y.
# ---------------------------------------------------------------------------
CAB_W = 1.60
CAB_D = 0.50
CAB_TOP = 1.80
LEG_H = 0.15
WALL_T = 0.02

FRONT_Y = CAB_D / 2.0  # +0.25
BACK_Y = -CAB_D / 2.0

BOTTOM_RAIL_TOP = LEG_H + 0.06  # 0.21
TOP_RAIL_BOT = CAB_TOP - 0.06  # 1.74

STILE_W = 0.03

# Front bay layout: left door | centre opening | right door
INNER_X = CAB_W / 2.0 - WALL_T  # 0.78, inner face of side walls
LEFT_EDGE = -INNER_X  # -0.78
RIGHT_EDGE = INNER_X  # +0.78
DIVIDER_L = -0.25  # inner edge of left door bay
DIVIDER_R = 0.25  # inner edge of right door bay

LEFT_DOOR_W = DIVIDER_L - LEFT_EDGE  # 0.53
RIGHT_DOOR_W = RIGHT_EDGE - DIVIDER_R  # 0.53
CENTER_W = DIVIDER_R - DIVIDER_L  # 0.50

DOOR_T = WALL_T
DOOR_Z0 = BOTTOM_RAIL_TOP + 0.002  # 0.212
DOOR_Z1 = TOP_RAIL_BOT - 0.002  # 1.738
DOOR_H = DOOR_Z1 - DOOR_Z0  # 1.526
DOOR_ZC = 0.5 * (DOOR_Z0 + DOOR_Z1)  # 0.975
HINGE_INSET = 0.0005

SLOT_LEN = 0.36
SLOT_W = 0.030
SLOT_ZC = -0.40

BARREL_R = 0.0075
KNUCKLE_R = 0.0095
BARREL_LEN = DOOR_H - 0.03

CAP_T = 0.022
CAP_OVERHANG = 0.02

DOOR_OPEN = math.radians(110.0)
KNOB_TURN = math.radians(90.0)

# Tambour (recessed behind the stiles and doors, narrower than the opening
# so it doesn't overlap the door leaves in X).
TAMBOUR_W = CENTER_W - 0.010  # 0.490 – 5mm gap each side to the stiles
TAMBOUR_T = 0.012
TAMBOUR_H = DOOR_H
TAMBOUR_SLIDE = CENTER_W + 0.02  # travel to fully clear centre opening
# Tambour sits behind the stile front face: front of tambour at
# FRONT_Y - WALL_T - 0.001 so there's no Y overlap with the stiles or doors.
TAMBOUR_Y_FRONT = FRONT_Y - WALL_T - 0.001
TAMBOUR_Y_CENTER = TAMBOUR_Y_FRONT - TAMBOUR_T / 2.0

# Seam dimensions
SEAM_W = 0.003
SEAM_DEPTH = 0.005


def _door_solid(sign: float, door_w: float, mesh_name: str):
    """Door leaf: flat panel with a rounded-end through slot near the bottom.
    sign=+1: panel extends along +X from hinge (left-hinged).
    sign=-1: panel extends along -X from hinge (right-hinged)."""
    xc = sign * door_w / 2.0
    panel = (
        cq.Workplane("XY")
        .box(door_w, DOOR_T, DOOR_H)
        .translate((xc, -DOOR_T / 2.0, 0.0))
    )
    cutter = (
        cq.Workplane("XZ")
        .slot2D(SLOT_LEN, SLOT_W, 90)
        .extrude(0.05, both=True)
        .translate((xc, 0.0, SLOT_ZC))
    )
    leaf = panel.cut(cutter)
    return mesh_from_cadquery(leaf, mesh_name)


def _tambour_solid(mesh_name: str):
    """Tambour panel: flat slab with horizontal slat grooves across the face."""
    base = cq.Workplane("XY").box(TAMBOUR_W, TAMBOUR_T, TAMBOUR_H)
    slat_spacing = 0.04
    n_slats = int(TAMBOUR_H / slat_spacing)
    for i in range(n_slats):
        z = -TAMBOUR_H / 2.0 + (i + 0.5) * slat_spacing
        groove = (
            cq.Workplane("XY")
            .box(TAMBOUR_W + 0.01, 0.003, 0.002)
            .translate((0.0, TAMBOUR_T / 2.0 - 0.0005, z))
        )
        base = base.cut(groove)
    return mesh_from_cadquery(base, mesh_name)


def _hinge_barrel_solid(mesh_name: str):
    """Piano-hinge knuckle column along the door hinge edge (local Z axis)."""
    barrel = cq.Workplane("XY").circle(BARREL_R).extrude(BARREL_LEN / 2.0, both=True)
    ring_h = 0.055
    for zc in (-0.60, -0.30, 0.0, 0.30, 0.60):
        ring = (
            cq.Workplane("XY")
            .circle(KNUCKLE_R)
            .extrude(ring_h / 2.0, both=True)
            .translate((0.0, 0.0, zc))
        )
        barrel = barrel.union(ring)
    return mesh_from_cadquery(barrel, mesh_name)


def _leg_solid(mesh_name: str):
    """Splayed tapered leg."""
    leg = (
        cq.Workplane("XY")
        .center(0.03, 0.03)
        .rect(0.035, 0.035)
        .workplane(offset=LEG_H + 0.01)
        .center(-0.03, -0.03)
        .rect(0.06, 0.06)
        .loft()
    )
    return mesh_from_cadquery(leg, mesh_name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_steel_locker_cabinet_v04")

    steel_body = model.material("steel_body", rgba=(0.60, 0.61, 0.63, 1.0))
    steel_door = model.material("steel_door", rgba=(0.55, 0.56, 0.58, 1.0))
    steel_trim = model.material("steel_trim", rgba=(0.46, 0.47, 0.49, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.07, 0.07, 0.08, 1.0))
    steel_leg = model.material("steel_leg", rgba=(0.38, 0.39, 0.41, 1.0))
    steel_knob = model.material("steel_knob", rgba=(0.18, 0.18, 0.20, 1.0))
    steel_rivet = model.material("steel_rivet", rgba=(0.52, 0.53, 0.55, 1.0))
    steel_tambour = model.material("steel_tambour", rgba=(0.50, 0.51, 0.53, 1.0))
    steel_shelf = model.material("steel_shelf", rgba=(0.58, 0.59, 0.61, 1.0))
    seam_mat = model.material("seam_mat", rgba=(0.10, 0.10, 0.11, 1.0))

    # ------------------------------------------------------------------
    # Cabinet body: hollow carcass + legs + front frame + shelves + cap
    # ------------------------------------------------------------------
    body = model.part("cabinet_body")
    carcass_h = CAB_TOP - LEG_H  # 1.65
    carcass_zc = LEG_H + carcass_h / 2.0

    # Side walls
    for sx, vname in ((-1.0, "side_wall_0"), (1.0, "side_wall_1")):
        body.visual(
            Box((WALL_T, CAB_D, carcass_h)),
            origin=Origin(xyz=(sx * (CAB_W / 2.0 - WALL_T / 2.0), 0.0, carcass_zc)),
            material=steel_body,
            name=vname,
        )
    # Back wall
    body.visual(
        Box((CAB_W - WALL_T, WALL_T, carcass_h - 0.02)),
        origin=Origin(xyz=(0.0, BACK_Y + WALL_T / 2.0, carcass_zc)),
        material=steel_body,
        name="back_wall",
    )
    # Bottom and top panels
    body.visual(
        Box((CAB_W, CAB_D, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, LEG_H + WALL_T / 2.0)),
        material=steel_body,
        name="bottom_panel",
    )
    body.visual(
        Box((CAB_W, CAB_D, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, CAB_TOP - WALL_T / 2.0)),
        material=steel_body,
        name="top_panel",
    )

    # Front frame: bottom rail, top rail
    body.visual(
        Box((CAB_W - 2.0 * WALL_T, WALL_T, BOTTOM_RAIL_TOP - LEG_H + 0.01)),
        origin=Origin(
            xyz=(0.0, FRONT_Y - WALL_T / 2.0, (LEG_H + BOTTOM_RAIL_TOP) / 2.0)
        ),
        material=steel_body,
        name="front_bottom_rail",
    )
    body.visual(
        Box((CAB_W - 2.0 * WALL_T, WALL_T, CAB_TOP - TOP_RAIL_BOT + 0.01)),
        origin=Origin(xyz=(0.0, FRONT_Y - WALL_T / 2.0, (TOP_RAIL_BOT + CAB_TOP) / 2.0)),
        material=steel_body,
        name="front_top_rail",
    )

    # Vertical stiles at bay dividers (between doors and centre opening)
    stile_h = TOP_RAIL_BOT - BOTTOM_RAIL_TOP + 0.01
    for i, xc in enumerate((DIVIDER_L + STILE_W / 2.0, DIVIDER_R - STILE_W / 2.0)):
        body.visual(
            Box((STILE_W, WALL_T, stile_h)),
            origin=Origin(xyz=(xc, FRONT_Y - WALL_T / 2.0, DOOR_ZC)),
            material=steel_trim,
            name=f"front_stile_{i}",
        )

    # Tambour track rails (thin strips at top/bottom of centre opening,
    # recessed behind the stiles to match the tambour panel depth).
    track_y = TAMBOUR_Y_CENTER
    body.visual(
        Box((CENTER_W + 0.04, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, track_y, DOOR_Z0 - 0.006)),
        material=steel_trim,
        name="tambour_track_bottom",
    )
    body.visual(
        Box((CENTER_W + 0.04, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, track_y, DOOR_Z1 + 0.006)),
        material=steel_trim,
        name="tambour_track_top",
    )

    # Top cap
    body.visual(
        Box((CAB_W + 2 * CAP_OVERHANG, CAB_D + 2 * CAP_OVERHANG, CAP_T)),
        origin=Origin(xyz=(0.0, 0.0, CAB_TOP - 0.001 + CAP_T / 2.0)),
        material=steel_trim,
        name="top_cap",
    )

    # Rivet dots along the top rail
    n_riv = 13
    for i in range(n_riv):
        rx = -0.72 + i * (1.44 / (n_riv - 1))
        body.visual(
            Sphere(radius=0.0045),
            origin=Origin(xyz=(rx, FRONT_Y + 0.002, 1.77)),
            material=steel_rivet,
            name=f"rivet_{i}",
        )

    # Legs
    leg_mesh = _leg_solid("splayed_leg")
    leg_corners = [
        (0.72, 0.19, 0.0),
        (-0.72, 0.19, math.pi / 2.0),
        (-0.72, -0.19, math.pi),
        (0.72, -0.19, 3.0 * math.pi / 2.0),
    ]
    for i, (lx, ly, yaw) in enumerate(leg_corners):
        body.visual(
            leg_mesh,
            origin=Origin(xyz=(lx, ly, 0.0), rpy=(0.0, 0.0, yaw)),
            material=steel_leg,
            name=f"leg_{i}",
        )

    # Interior shelf boards (visible through centre opening when tambour open).
    # Width overlaps the inner face of the side walls by ~5mm each side so
    # the shelves read as supported and are geometrically connected.
    shelf_z_positions = [0.55, 0.97, 1.39]
    shelf_w = CAB_W - 2.0 * WALL_T + 0.01  # slight embed into side walls
    shelf_d = CAB_D - 2.0 * WALL_T - 0.04
    for i, sz in enumerate(shelf_z_positions):
        body.visual(
            Box((shelf_w, shelf_d, 0.015)),
            origin=Origin(xyz=(0.0, -0.01, sz)),
            material=steel_shelf,
            name=f"shelf_{i}",
        )

    # ------------------------------------------------------------------
    # Door gap seams – thin dark strips framing each door and the centre
    # opening, slightly proud of the front face to read as shadow gaps.
    # ------------------------------------------------------------------
    seam_y = FRONT_Y + 0.001

    # Left door seams (top, bottom, outer, inner)
    ld_cx = 0.5 * (LEFT_EDGE + DIVIDER_L)
    body.visual(
        Box((LEFT_DOOR_W + SEAM_W, SEAM_DEPTH, SEAM_W)),
        origin=Origin(xyz=(ld_cx, seam_y, DOOR_Z1 + SEAM_W / 2.0)),
        material=seam_mat, name="seam_left_top",
    )
    body.visual(
        Box((LEFT_DOOR_W + SEAM_W, SEAM_DEPTH, SEAM_W)),
        origin=Origin(xyz=(ld_cx, seam_y, DOOR_Z0 - SEAM_W / 2.0)),
        material=seam_mat, name="seam_left_bottom",
    )
    body.visual(
        Box((SEAM_W, SEAM_DEPTH, DOOR_H + 2 * SEAM_W)),
        origin=Origin(xyz=(LEFT_EDGE - SEAM_W / 2.0, seam_y, DOOR_ZC)),
        material=seam_mat, name="seam_left_outer",
    )
    body.visual(
        Box((SEAM_W, SEAM_DEPTH, DOOR_H + 2 * SEAM_W)),
        origin=Origin(xyz=(DIVIDER_L + SEAM_W / 2.0, seam_y, DOOR_ZC)),
        material=seam_mat, name="seam_left_inner",
    )

    # Right door seams
    rd_cx = 0.5 * (DIVIDER_R + RIGHT_EDGE)
    body.visual(
        Box((RIGHT_DOOR_W + SEAM_W, SEAM_DEPTH, SEAM_W)),
        origin=Origin(xyz=(rd_cx, seam_y, DOOR_Z1 + SEAM_W / 2.0)),
        material=seam_mat, name="seam_right_top",
    )
    body.visual(
        Box((RIGHT_DOOR_W + SEAM_W, SEAM_DEPTH, SEAM_W)),
        origin=Origin(xyz=(rd_cx, seam_y, DOOR_Z0 - SEAM_W / 2.0)),
        material=seam_mat, name="seam_right_bottom",
    )
    body.visual(
        Box((SEAM_W, SEAM_DEPTH, DOOR_H + 2 * SEAM_W)),
        origin=Origin(xyz=(RIGHT_EDGE + SEAM_W / 2.0, seam_y, DOOR_ZC)),
        material=seam_mat, name="seam_right_outer",
    )
    body.visual(
        Box((SEAM_W, SEAM_DEPTH, DOOR_H + 2 * SEAM_W)),
        origin=Origin(xyz=(DIVIDER_R - SEAM_W / 2.0, seam_y, DOOR_ZC)),
        material=seam_mat, name="seam_right_inner",
    )

    # Centre opening seams (top and bottom)
    body.visual(
        Box((CENTER_W + SEAM_W, SEAM_DEPTH, SEAM_W)),
        origin=Origin(xyz=(0.0, seam_y, DOOR_Z1 + SEAM_W / 2.0)),
        material=seam_mat, name="seam_center_top",
    )
    body.visual(
        Box((CENTER_W + SEAM_W, SEAM_DEPTH, SEAM_W)),
        origin=Origin(xyz=(0.0, seam_y, DOOR_Z0 - SEAM_W / 2.0)),
        material=seam_mat, name="seam_center_bottom",
    )

    # ------------------------------------------------------------------
    # Left door (hinged on left edge, opens outward to the left)
    # ------------------------------------------------------------------
    door_left = model.part("door_left")
    left_hinge_x = LEFT_EDGE + HINGE_INSET

    door_left.visual(
        _door_solid(+1.0, LEFT_DOOR_W, "door_leaf_left"),
        material=steel_door,
        name="leaf",
    )
    door_left.visual(
        Box((SLOT_W + 0.016, 0.005, SLOT_LEN + 0.036)),
        origin=Origin(xyz=(LEFT_DOOR_W / 2.0, -DOOR_T - 0.001, SLOT_ZC)),
        material=steel_dark,
        name="vent_backing",
    )
    for j, dz in enumerate((0.60, 0.62, 0.64)):
        door_left.visual(
            Box((0.20, 0.004, 0.006)),
            origin=Origin(xyz=(LEFT_DOOR_W / 2.0, -0.0012, dz)),
            material=steel_dark,
            name=f"vent_line_{j}",
        )
    door_left.visual(
        _hinge_barrel_solid("hinge_barrel_left"),
        origin=Origin(xyz=(0.0, 0.004, 0.0)),
        material=steel_trim,
        name="hinge_barrel",
    )

    model.articulation(
        "door_left_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door_left,
        origin=Origin(xyz=(left_hinge_x, FRONT_Y, DOOR_ZC)),
        axis=(0.0, 0.0, 1.0),  # +Z: left-hinged leaf swings outward (-X, +Y)
        motion_limits=MotionLimits(
            effort=40.0, velocity=2.0, lower=0.0, upper=DOOR_OPEN
        ),
    )

    # Latch knob on left door
    knob_left = model.part("latch_knob_left")
    knob_left.visual(
        Cylinder(radius=0.018, length=0.005),
        origin=Origin(xyz=(0.0, 0.0025, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_knob, name="backplate",
    )
    knob_left.visual(
        Cylinder(radius=0.0065, length=0.014),
        origin=Origin(xyz=(0.0, 0.011, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_knob, name="boss",
    )
    knob_left.visual(
        Box((0.010, 0.008, 0.034)),
        origin=Origin(xyz=(0.0, 0.020, 0.0)),
        material=steel_knob, name="handle_bar",
    )
    knob_left.visual(
        Sphere(radius=0.006),
        origin=Origin(xyz=(0.0, 0.020, -0.019)),
        material=steel_knob, name="handle_tip",
    )
    model.articulation(
        "latch_left",
        ArticulationType.REVOLUTE,
        parent=door_left,
        child=knob_left,
        origin=Origin(xyz=(LEFT_DOOR_W - 0.10, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0, velocity=4.0, lower=0.0, upper=KNOB_TURN
        ),
    )

    # ------------------------------------------------------------------
    # Right door (hinged on right edge, opens outward to the right)
    # ------------------------------------------------------------------
    door_right = model.part("door_right")
    right_hinge_x = RIGHT_EDGE - HINGE_INSET

    door_right.visual(
        _door_solid(-1.0, RIGHT_DOOR_W, "door_leaf_right"),
        material=steel_door,
        name="leaf",
    )
    door_right.visual(
        Box((SLOT_W + 0.016, 0.005, SLOT_LEN + 0.036)),
        origin=Origin(xyz=(-RIGHT_DOOR_W / 2.0, -DOOR_T - 0.001, SLOT_ZC)),
        material=steel_dark,
        name="vent_backing",
    )
    for j, dz in enumerate((0.60, 0.62, 0.64)):
        door_right.visual(
            Box((0.20, 0.004, 0.006)),
            origin=Origin(xyz=(-RIGHT_DOOR_W / 2.0, -0.0012, dz)),
            material=steel_dark,
            name=f"vent_line_{j}",
        )
    door_right.visual(
        _hinge_barrel_solid("hinge_barrel_right"),
        origin=Origin(xyz=(0.0, 0.004, 0.0)),
        material=steel_trim,
        name="hinge_barrel",
    )

    model.articulation(
        "door_right_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door_right,
        origin=Origin(xyz=(right_hinge_x, FRONT_Y, DOOR_ZC)),
        axis=(0.0, 0.0, -1.0),  # -Z: right-hinged leaf swings outward (+X, +Y)
        motion_limits=MotionLimits(
            effort=40.0, velocity=2.0, lower=0.0, upper=DOOR_OPEN
        ),
    )

    # Latch knob on right door
    knob_right = model.part("latch_knob_right")
    knob_right.visual(
        Cylinder(radius=0.018, length=0.005),
        origin=Origin(xyz=(0.0, 0.0025, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_knob, name="backplate",
    )
    knob_right.visual(
        Cylinder(radius=0.0065, length=0.014),
        origin=Origin(xyz=(0.0, 0.011, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_knob, name="boss",
    )
    knob_right.visual(
        Box((0.010, 0.008, 0.034)),
        origin=Origin(xyz=(0.0, 0.020, 0.0)),
        material=steel_knob, name="handle_bar",
    )
    knob_right.visual(
        Sphere(radius=0.006),
        origin=Origin(xyz=(0.0, 0.020, -0.019)),
        material=steel_knob, name="handle_tip",
    )
    model.articulation(
        "latch_right",
        ArticulationType.REVOLUTE,
        parent=door_right,
        child=knob_right,
        origin=Origin(xyz=(-RIGHT_DOOR_W + 0.10, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0, velocity=4.0, lower=0.0, upper=KNOB_TURN
        ),
    )

    # ------------------------------------------------------------------
    # Tambour sliding panel (prismatic, slides right to reveal shelves)
    # ------------------------------------------------------------------
    tambour = model.part("tambour_panel")
    tambour.visual(
        _tambour_solid("tambour_slats"),
        material=steel_tambour,
        name="tambour_body",
    )
    # Pull handle on the left edge – a thin tab positioned inside the stile
    # boundary in X so it can protrude forward for gripping without
    # overlapping the stile geometry.
    handle_depth = 0.015  # stands proud of the tambour body
    handle_x = -0.19  # well inside the left stile edge (stile inner at -0.22)
    tambour.visual(
        Box((0.008, handle_depth, 0.050)),
        origin=Origin(
            xyz=(handle_x, TAMBOUR_T / 2.0 + handle_depth / 2.0, 0.0)
        ),
        material=steel_knob,
        name="pull_handle",
    )

    model.articulation(
        "tambour_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tambour,
        origin=Origin(xyz=(0.0, TAMBOUR_Y_CENTER, DOOR_ZC)),
        axis=(1.0, 0.0, 0.0),  # slides +X (right) to open
        motion_limits=MotionLimits(
            effort=20.0, velocity=0.5, lower=0.0, upper=TAMBOUR_SLIDE
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("cabinet_body")
    door_left = object_model.get_part("door_left")
    door_right = object_model.get_part("door_right")
    tambour = object_model.get_part("tambour_panel")
    hinge_l = object_model.get_articulation("door_left_hinge")
    hinge_r = object_model.get_articulation("door_right_hinge")
    slide = object_model.get_articulation("tambour_slide")
    knob_l = object_model.get_part("latch_knob_left")
    knob_r = object_model.get_part("latch_knob_right")
    latch_l = object_model.get_articulation("latch_left")
    latch_r = object_model.get_articulation("latch_right")

    # --- Intentional overlaps: hinge barrels lap frame edges ---
    ctx.allow_overlap(
        door_left, body,
        elem_a="hinge_barrel", elem_b="side_wall_0",
        reason="Piano-hinge knuckle column intentionally laps the left frame edge.",
    )
    ctx.allow_overlap(
        door_right, body,
        elem_a="hinge_barrel", elem_b="side_wall_1",
        reason="Piano-hinge knuckle column intentionally laps the right frame edge.",
    )
    # Tambour panel is recessed behind the stiles (no overlap).
    # The pull handle tab intentionally extends past the stile plane for
    # user access – this is a local protrusion, not an overlap.

    # --- Overall envelope ---
    aabb = ctx.part_world_aabb(body)
    ok = aabb is not None
    ctx.check("cabinet body has bounds", ok, details=str(aabb))
    if aabb is not None:
        (x0, y0, z0), (x1, y1, z1) = aabb
        ctx.check(
            "overall width ~1.6 m", 1.58 <= (x1 - x0) <= 1.70,
            details=f"width={x1 - x0:.3f}",
        )
        ctx.check(
            "overall depth ~0.5 m", 0.48 <= (y1 - y0) <= 0.58,
            details=f"depth={y1 - y0:.3f}",
        )
        ctx.check(
            "overall height ~1.8 m", 1.78 <= z1 <= 1.86,
            details=f"top={z1:.3f}",
        )
        ctx.check("legs rest on the floor", abs(z0) <= 1e-6, details=f"zmin={z0:.5f}")

    # --- Left door: revolute, vertical axis, correct range ---
    ctx.check(
        "left door hinge is revolute",
        hinge_l.articulation_type == ArticulationType.REVOLUTE,
    )
    ax_l = hinge_l.axis
    ctx.check(
        "left door hinge axis is vertical",
        abs(ax_l[0]) < 1e-9 and abs(ax_l[1]) < 1e-9 and abs(abs(ax_l[2]) - 1.0) < 1e-9,
        details=str(ax_l),
    )
    lim_l = hinge_l.motion_limits
    ctx.check(
        "left door opens 0..~110 deg",
        lim_l is not None and lim_l.lower == 0.0
        and abs(lim_l.upper - math.radians(110.0)) < 1e-6,
    )

    # --- Right door: revolute, vertical axis, correct range ---
    ctx.check(
        "right door hinge is revolute",
        hinge_r.articulation_type == ArticulationType.REVOLUTE,
    )
    ax_r = hinge_r.axis
    ctx.check(
        "right door hinge axis is vertical",
        abs(ax_r[0]) < 1e-9 and abs(ax_r[1]) < 1e-9 and abs(abs(ax_r[2]) - 1.0) < 1e-9,
        details=str(ax_r),
    )
    lim_r = hinge_r.motion_limits
    ctx.check(
        "right door opens 0..~110 deg",
        lim_r is not None and lim_r.lower == 0.0
        and abs(lim_r.upper - math.radians(110.0)) < 1e-6,
    )

    # Hinge positions: left at cabinet left edge, right at cabinet right edge
    ctx.check(
        "left hinge at left edge, right hinge at right edge",
        hinge_l.origin.xyz[0] < -0.70 and hinge_r.origin.xyz[0] > 0.70,
        details=f"L={hinge_l.origin.xyz[0]:.3f}, R={hinge_r.origin.xyz[0]:.3f}",
    )

    # Doors closed: leaves flush with front face
    for door, label in ((door_left, "left"), (door_right, "right")):
        daabb = ctx.part_element_world_aabb(door, elem="leaf")
        ctx.check(
            f"{label} door closed leaf is flush with the front face",
            daabb is not None
            and abs(daabb[1][1] - FRONT_Y) < 1e-4
            and abs(daabb[0][1] - (FRONT_Y - DOOR_T)) < 1e-4,
            details=str(daabb),
        )

    # Open pose: doors swing outward past the front face
    closed_l = ctx.part_world_aabb(door_left)
    closed_r = ctx.part_world_aabb(door_right)
    with ctx.pose({hinge_l: DOOR_OPEN, hinge_r: DOOR_OPEN}):
        open_l = ctx.part_world_aabb(door_left)
        open_r = ctx.part_world_aabb(door_right)
    ctx.check(
        "open leaves swing outward past the front face",
        open_l is not None and open_r is not None
        and open_l[1][1] > FRONT_Y + 0.20
        and open_r[1][1] > FRONT_Y + 0.20,
        details=f"open_l={open_l}, open_r={open_r}",
    )
    ctx.check(
        "doors open away from centre",
        closed_l is not None and closed_r is not None
        and open_l[0][0] < closed_l[0][0] - 0.05
        and open_r[1][0] > closed_r[1][0] + 0.05,
    )

    # --- Tambour slide: prismatic, correct axis and limits ---
    ctx.check(
        "tambour slide is prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
    )
    ctx.check(
        "tambour slide axis is +X (horizontal)",
        slide.axis == (1.0, 0.0, 0.0),
        details=str(slide.axis),
    )
    tlim = slide.motion_limits
    ctx.check(
        "tambour slide has positive travel",
        tlim is not None and tlim.lower == 0.0 and tlim.upper > 0.30,
        details=f"upper={tlim.upper:.3f}" if tlim else "no limits",
    )

    # Tambour at rest covers the centre opening
    t_rest = ctx.part_world_aabb(tambour)
    ctx.check(
        "tambour at rest covers centre opening",
        t_rest is not None
        and t_rest[0][0] < DIVIDER_L + 0.02
        and t_rest[1][0] > DIVIDER_R - 0.02,
        details=str(t_rest),
    )

    # Tambour slides right: at max travel the panel is shifted right
    with ctx.pose({slide: TAMBOUR_SLIDE}):
        t_open = ctx.part_world_aabb(tambour)
    ctx.check(
        "tambour slides right when opened",
        t_open is not None and t_rest is not None
        and t_open[0][0] > t_rest[0][0] + 0.30,
        details=f"rest={t_rest}, open={t_open}",
    )

    # --- Shelf boards exist and are inside the cabinet ---
    # Shelves embed ~5mm into each side wall for geometric connectivity,
    # so the margin allows the shelf to extend slightly past the inner pocket.
    for i in range(3):
        shelf_name = f"shelf_{i}"
        saabb = ctx.part_element_world_aabb(body, elem=shelf_name)
        ctx.check(
            f"{shelf_name} exists inside the cabinet",
            saabb is not None
            and saabb[0][0] > LEFT_EDGE - 0.01
            and saabb[1][0] < RIGHT_EDGE + 0.01
            and saabb[0][2] > LEG_H
            and saabb[1][2] < CAB_TOP,
            details=str(saabb),
        )

    # --- Door gap seams present around doors ---
    for seam_name in (
        "seam_left_top", "seam_left_bottom", "seam_left_outer", "seam_left_inner",
        "seam_right_top", "seam_right_bottom", "seam_right_outer", "seam_right_inner",
        "seam_center_top", "seam_center_bottom",
    ):
        saabb = ctx.part_element_world_aabb(body, elem=seam_name)
        ctx.check(
            f"gap seam {seam_name} present at front face",
            saabb is not None and abs(saabb[1][1] - FRONT_Y) < 0.015,
            details=str(saabb),
        )

    # --- Latch knobs ---
    for knob, latch, door, label in (
        (knob_l, latch_l, door_left, "left"),
        (knob_r, latch_r, door_right, "right"),
    ):
        ctx.check(
            f"latch_{label} is a quarter-turn revolute",
            latch.articulation_type == ArticulationType.REVOLUTE
            and latch.axis == (0.0, 1.0, 0.0)
            and latch.motion_limits is not None
            and abs(latch.motion_limits.upper - math.pi / 2.0) < 1e-6,
        )
        ctx.expect_contact(
            knob, door,
            elem_a="backplate", elem_b="leaf",
            contact_tol=1e-6,
            name=f"latch_knob_{label} backplate seats on the leaf face",
        )

    return ctx.report()


object_model = build_object_model()
