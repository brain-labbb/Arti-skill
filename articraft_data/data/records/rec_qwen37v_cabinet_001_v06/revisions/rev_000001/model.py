from __future__ import annotations

"""Vintage industrial steel cabinet variant: open side cubbies with visible
shelves flanking a closed center cabinet with one mirrored door.

Overall envelope ~1.6 m wide x 0.5 m deep x ~1.8 m tall, brushed/tarnished raw
steel. A hollow thin-wall (~0.02 m) carcass sits on four short splayed legs and
carries a thin riveted top cap strip. The front is divided into three bays:
left open cubby, center closed cabinet (one hinged door with mirror panel),
right open cubby. Vertical dividers separate the bays. Each open cubby exposes
two horizontal shelf boards. The single center door hinges on its left edge
(piano-hinge knuckle column), swinging outward 0..~110 deg, and carries a
quarter-turn latch knob at mid-height, a mirror panel on the front face, a dark
recessed ventilation slot near the bottom, and stamped vent lines near the top.
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
CAB_W = 1.60  # overall carcass width  (X)
CAB_D = 0.50  # overall carcass depth  (Y)
CAB_TOP = 1.80  # carcass top height   (Z)
LEG_H = 0.15  # splayed leg height; carcass starts here
WALL_T = 0.02  # thin steel wall thickness

FRONT_Y = CAB_D / 2.0  # +0.25, front face plane
BACK_Y = -CAB_D / 2.0

BOTTOM_RAIL_TOP = LEG_H + 0.06  # 0.21
TOP_RAIL_BOT = CAB_TOP - 0.06  # 1.74

# Bay dividers: left cubby | center cabinet | right cubby
DIVIDER_X = (-0.30, 0.30)  # two vertical divider positions
INNER_LEFT = -(CAB_W / 2.0 - WALL_T)  # -0.78
INNER_RIGHT = CAB_W / 2.0 - WALL_T  # +0.78

# Cubby widths
LEFT_CUBBY_W = DIVIDER_X[0] - INNER_LEFT  # 0.48
RIGHT_CUBBY_W = INNER_RIGHT - DIVIDER_X[1]  # 0.48
CENTER_W = DIVIDER_X[1] - DIVIDER_X[0]  # 0.60

DOOR_W = CENTER_W - 0.016  # 0.584 leaf width with clearance
DOOR_T = WALL_T
DOOR_Z0 = BOTTOM_RAIL_TOP + 0.002  # 0.212
DOOR_Z1 = TOP_RAIL_BOT - 0.002  # 1.738
DOOR_H = DOOR_Z1 - DOOR_Z0  # 1.526
DOOR_ZC = 0.5 * (DOOR_Z0 + DOOR_Z1)  # 0.975
HINGE_X = DIVIDER_X[0] + 0.001  # hinge at left edge of center opening

SLOT_LEN = 0.36  # dark rounded-end vent slot near the bottom
SLOT_W = 0.030
SLOT_ZC = -0.40  # in door-local z (door centre = 0)

# Mirror panel on door front
MIRROR_W = DOOR_W - 0.08
MIRROR_H = DOOR_H - 0.20

BARREL_R = 0.0075
KNUCKLE_R = 0.0095
BARREL_LEN = DOOR_H - 0.03

CAP_T = 0.022  # riveted top cap strip
CAP_OVERHANG = 0.02

SHELF_T = 0.015
SHELF_DEPTH = CAB_D - 2.0 * WALL_T - 0.02  # fits inside carcass

DOOR_OPEN = math.radians(110.0)
KNOB_TURN = math.radians(90.0)


def _door_solid(mesh_name: str):
    """Door leaf as one CadQuery solid: flat panel with a rounded-end
    through slot near the bottom. Panel extends along +X from the hinge line
    (left-hinged door)."""
    xc = DOOR_W / 2.0
    panel = (
        cq.Workplane("XY")
        .box(DOOR_W, DOOR_T, DOOR_H)
        .translate((xc, -DOOR_T / 2.0, 0.0))
    )
    # Vertical slot, rounded ends, cut through the leaf thickness.
    cutter = (
        cq.Workplane("XZ")
        .slot2D(SLOT_LEN, SLOT_W, 90)
        .extrude(0.05, both=True)
        .translate((xc, 0.0, SLOT_ZC))
    )
    leaf = panel.cut(cutter)
    return mesh_from_cadquery(leaf, mesh_name)


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
    """Splayed tapered leg: small foot on the floor, offset outward toward
    local (+x, +y); wide top section embedded into the carcass bottom."""
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
    model = ArticulatedObject(name="vintage_steel_cabinet_cubbies")

    steel_body = model.material("steel_body", rgba=(0.60, 0.61, 0.63, 1.0))
    steel_door = model.material("steel_door", rgba=(0.55, 0.56, 0.58, 1.0))
    steel_trim = model.material("steel_trim", rgba=(0.46, 0.47, 0.49, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.07, 0.07, 0.08, 1.0))
    steel_leg = model.material("steel_leg", rgba=(0.38, 0.39, 0.41, 1.0))
    steel_knob = model.material("steel_knob", rgba=(0.18, 0.18, 0.20, 1.0))
    steel_rivet = model.material("steel_rivet", rgba=(0.52, 0.53, 0.55, 1.0))
    mirror_mat = model.material("mirror", rgba=(0.82, 0.84, 0.86, 1.0))
    shelf_mat = model.material("shelf_wood", rgba=(0.55, 0.45, 0.32, 1.0))

    # ------------------------------------------------------------------
    # Cabinet body: hollow carcass + legs + front frame + top cap + rivets
    # + dividers + cubby shelves
    # ------------------------------------------------------------------
    body = model.part("cabinet_body")
    carcass_h = CAB_TOP - LEG_H  # 1.65
    carcass_zc = LEG_H + carcass_h / 2.0

    # Side walls (full depth, full carcass height).
    for sx, vname in ((-1.0, "side_wall_0"), (1.0, "side_wall_1")):
        body.visual(
            Box((WALL_T, CAB_D, carcass_h)),
            origin=Origin(xyz=(sx * (CAB_W / 2.0 - WALL_T / 2.0), 0.0, carcass_zc)),
            material=steel_body,
            name=vname,
        )
    # Back wall.
    body.visual(
        Box((CAB_W - WALL_T, WALL_T, carcass_h - 0.02)),
        origin=Origin(xyz=(0.0, BACK_Y + WALL_T / 2.0, carcass_zc)),
        material=steel_body,
        name="back_wall",
    )
    # Bottom and top panels.
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

    # Vertical dividers separating the three bays.
    for i, dx in enumerate(DIVIDER_X):
        body.visual(
            Box((WALL_T, CAB_D - 2.0 * WALL_T, carcass_h - 0.02)),
            origin=Origin(xyz=(dx, 0.0, carcass_zc)),
            material=steel_body,
            name=f"divider_{i}",
        )

    # Front frame: bottom rail and top rail (full width, no intermediate stiles
    # since the cubbies are open and dividers serve as bay boundaries).
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

    # Thin riveted top cap strip with a slight overhang.
    body.visual(
        Box((CAB_W + 2 * CAP_OVERHANG, CAB_D + 2 * CAP_OVERHANG, CAP_T)),
        origin=Origin(xyz=(0.0, 0.0, CAB_TOP - 0.001 + CAP_T / 2.0)),
        material=steel_trim,
        name="top_cap",
    )
    # Raised rivet dots along the top rail.
    n_riv = 13
    for i in range(n_riv):
        rx = -0.72 + i * (1.44 / (n_riv - 1))
        body.visual(
            Sphere(radius=0.0045),
            origin=Origin(xyz=(rx, FRONT_Y + 0.002, 1.77)),
            material=steel_rivet,
            name=f"rivet_{i}",
        )

    # Splayed legs: one lofted solid reused at the four corners with yaw.
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

    # ------------------------------------------------------------------
    # Cubby shelves: 2 shelves per open cubby, visible through the open front.
    # ------------------------------------------------------------------
    shelf_z_positions = [0.65, 1.15]  # two shelf heights within the cubby
    # Left cubby shelves
    left_cubby_xc = 0.5 * (INNER_LEFT + DIVIDER_X[0])  # center of left cubby
    for j, sz in enumerate(shelf_z_positions):
        body.visual(
            Box((LEFT_CUBBY_W - 0.01, SHELF_DEPTH, SHELF_T)),
            origin=Origin(xyz=(left_cubby_xc, 0.0, sz)),
            material=shelf_mat,
            name=f"left_shelf_{j}",
        )
    # Right cubby shelves
    right_cubby_xc = 0.5 * (DIVIDER_X[1] + INNER_RIGHT)
    for j, sz in enumerate(shelf_z_positions):
        body.visual(
            Box((RIGHT_CUBBY_W - 0.01, SHELF_DEPTH, SHELF_T)),
            origin=Origin(xyz=(right_cubby_xc, 0.0, sz)),
            material=shelf_mat,
            name=f"right_shelf_{j}",
        )
    # Center cabinet interior shelf (overlaps divider inner faces for support).
    center_xc = 0.0
    body.visual(
        Box((CENTER_W + 0.01, SHELF_DEPTH, SHELF_T)),
        origin=Origin(xyz=(center_xc, 0.0, 0.95)),
        material=shelf_mat,
        name="center_shelf",
    )

    # ------------------------------------------------------------------
    # Center door: single hinged door with mirror panel, vent slot, vent lines
    # ------------------------------------------------------------------
    door = model.part("center_door")
    door_xc = DOOR_W / 2.0  # door-local panel centre (extends +X from hinge)

    door.visual(
        _door_solid("door_leaf"),
        material=steel_door,
        name="leaf",
    )
    # Dark backing plate behind the through slot -> recessed dark slot.
    door.visual(
        Box((SLOT_W + 0.016, 0.005, SLOT_LEN + 0.036)),
        origin=Origin(xyz=(door_xc, -DOOR_T - 0.001, SLOT_ZC)),
        material=steel_dark,
        name="vent_backing",
    )
    # Stamped vent lines near the top (slightly proud thin dark strips).
    for j, dz in enumerate((0.60, 0.62, 0.64)):
        door.visual(
            Box((0.20, 0.004, 0.006)),
            origin=Origin(xyz=(door_xc, -0.0012, dz)),
            material=steel_dark,
            name=f"vent_line_{j}",
        )
    # Mirror panel on the front face of the door.
    door.visual(
        Box((MIRROR_W, 0.004, MIRROR_H)),
        origin=Origin(xyz=(door_xc, 0.002, 0.05)),
        material=mirror_mat,
        name="mirror_panel",
    )
    # Piano-hinge knuckle column on the hinge edge.
    door.visual(
        _hinge_barrel_solid("hinge_barrel"),
        origin=Origin(xyz=(0.0, 0.004, 0.0)),
        material=steel_trim,
        name="hinge_barrel",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(HINGE_X, FRONT_Y, DOOR_ZC)),
        # +Z axis: positive q swings the free edge (+X side) outward (+Y).
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0, velocity=2.0, lower=0.0, upper=DOOR_OPEN
        ),
    )

    # Quarter-turn latch knob at mid-height near the free edge.
    knob = model.part("latch_knob")
    knob.visual(
        Cylinder(radius=0.018, length=0.005),
        origin=Origin(xyz=(0.0, 0.0025, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_knob,
        name="backplate",
    )
    knob.visual(
        Cylinder(radius=0.0065, length=0.014),
        origin=Origin(xyz=(0.0, 0.011, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_knob,
        name="boss",
    )
    knob.visual(
        Box((0.010, 0.008, 0.034)),
        origin=Origin(xyz=(0.0, 0.020, 0.0)),
        material=steel_knob,
        name="handle_bar",
    )
    knob.visual(
        Sphere(radius=0.006),
        origin=Origin(xyz=(0.0, 0.020, -0.019)),
        material=steel_knob,
        name="handle_tip",
    )
    model.articulation(
        "latch",
        ArticulationType.REVOLUTE,
        parent=door,
        child=knob,
        origin=Origin(xyz=(DOOR_W - 0.10, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0, velocity=4.0, lower=0.0, upper=KNOB_TURN
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("cabinet_body")
    door = object_model.get_part("center_door")
    hinge = object_model.get_articulation("door_hinge")
    knob = object_model.get_part("latch_knob")
    latch = object_model.get_articulation("latch")

    # Intentional local lap: hinge knuckle column embeds into the left divider
    # edge (captured piano-hinge knuckles).
    ctx.allow_overlap(
        door,
        body,
        elem_a="hinge_barrel",
        elem_b="divider_0",
        reason="Piano-hinge knuckle column intentionally laps the divider edge it pivots on.",
    )

    # --- Overall envelope, true scale, grounded on the floor ----------------
    aabb = ctx.part_world_aabb(body)
    ok = aabb is not None
    ctx.check("cabinet body has bounds", ok, details=str(aabb))
    if aabb is not None:
        (x0, y0, z0), (x1, y1, z1) = aabb
        ctx.check(
            "overall width ~1.6 m (cap overhang included)",
            1.58 <= (x1 - x0) <= 1.70,
            details=f"width={x1 - x0:.3f}",
        )
        ctx.check(
            "overall depth ~0.5 m",
            0.48 <= (y1 - y0) <= 0.58,
            details=f"depth={y1 - y0:.3f}",
        )
        ctx.check(
            "overall height ~1.8 m",
            1.78 <= z1 <= 1.86,
            details=f"top={z1:.3f}",
        )
        ctx.check("legs rest on the floor", abs(z0) <= 1e-6, details=f"zmin={z0:.5f}")

    # --- Door hinge: type, axis, range, closed seating ----------------------
    ctx.check(
        "door hinge is revolute",
        hinge.articulation_type == ArticulationType.REVOLUTE,
    )
    ax = hinge.axis
    ctx.check(
        "door hinge axis is vertical",
        abs(ax[0]) < 1e-9 and abs(ax[1]) < 1e-9 and abs(abs(ax[2]) - 1.0) < 1e-9,
        details=str(ax),
    )
    lim = hinge.motion_limits
    ctx.check(
        "door opens 0..~110 deg",
        lim is not None
        and lim.lower == 0.0
        and abs(lim.upper - math.radians(110.0)) < 1e-6,
    )

    # Closed door sits flush in the front frame plane.
    daabb = ctx.part_element_world_aabb(door, elem="leaf")
    ctx.check(
        "closed door is flush with the front face",
        daabb is not None
        and abs(daabb[1][1] - FRONT_Y) < 1e-4
        and abs(daabb[0][1] - (FRONT_Y - DOOR_T)) < 1e-4,
        details=str(daabb),
    )

    # Door hinge is at the left side of the center bay.
    ctx.check(
        "door hinge is on the left side of center bay",
        hinge.origin.xyz[0] < -0.20,
        details=f"hinge_x={hinge.origin.xyz[0]:.4f}",
    )

    # Opening pose: door swings outward (+Y).
    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({hinge: DOOR_OPEN}):
        open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "open door swings outward past the front face",
        open_aabb is not None and open_aabb[1][1] > FRONT_Y + 0.20,
        details=f"open={open_aabb}",
    )
    # Left-hinged door: free edge swings left (-X) and outward (+Y) when opening.
    ctx.check(
        "open door free edge swings left (left-hinged outward swing)",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][0] < closed_aabb[0][0] - 0.05,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    # --- Mirror panel on door front -----------------------------------------
    mirror_aabb = ctx.part_element_world_aabb(door, elem="mirror_panel")
    ctx.check(
        "mirror panel is on the front face of the door",
        mirror_aabb is not None and mirror_aabb[1][1] > FRONT_Y - 0.005,
        details=str(mirror_aabb),
    )
    ctx.check(
        "mirror panel is substantial in size",
        mirror_aabb is not None
        and (mirror_aabb[1][0] - mirror_aabb[0][0]) > 0.30
        and (mirror_aabb[1][2] - mirror_aabb[0][2]) > 0.80,
        details=str(mirror_aabb),
    )

    # --- Cubby shelves visible through open front ---------------------------
    for side, prefix in (("left", "left_shelf"), ("right", "right_shelf")):
        for j in range(2):
            shelf_elem = f"{prefix}_{j}"
            saabb = ctx.part_element_world_aabb(body, elem=shelf_elem)
            ctx.check(
                f"{side} cubby shelf {j} exists at visible height",
                saabb is not None
                and saabb[0][2] > BOTTOM_RAIL_TOP
                and saabb[1][2] < TOP_RAIL_BOT,
                details=str(saabb),
            )
            # Shelf is accessible from front: its Y extent reaches near FRONT_Y
            ctx.check(
                f"{side} cubby shelf {j} is visible from front opening",
                saabb is not None and saabb[1][1] > FRONT_Y - 0.06,
                details=str(saabb),
            )

    # --- Open cubbies: no blocking door or panel in front of cubby openings --
    # Left cubby opening: x from INNER_LEFT to DIVIDER_X[0], at front face
    # Right cubby opening: x from DIVIDER_X[1] to INNER_RIGHT, at front face
    # The body should not have a front panel blocking these openings.
    # We verify the bottom rail doesn't block the full opening height.
    rail_aabb = ctx.part_element_world_aabb(body, elem="front_bottom_rail")
    ctx.check(
        "bottom rail only covers the lower strip, not the full opening",
        rail_aabb is not None and rail_aabb[1][2] < 0.30,
        details=str(rail_aabb),
    )

    # --- Latch knob ---------------------------------------------------------
    ctx.check(
        "latch is a quarter-turn revolute about the door normal",
        latch.articulation_type == ArticulationType.REVOLUTE
        and latch.axis == (0.0, 1.0, 0.0)
        and latch.motion_limits is not None
        and abs(latch.motion_limits.upper - math.pi / 2.0) < 1e-6,
    )
    ctx.expect_contact(
        knob,
        door,
        elem_a="backplate",
        elem_b="leaf",
        contact_tol=1e-6,
        name="latch knob backplate seats on the leaf face",
    )
    ctx.check(
        "latch knob sits at door mid-height",
        (
            lambda a: a is not None and abs(0.5 * (a[0][2] + a[1][2]) - DOOR_ZC) < 0.03
        )(ctx.part_world_aabb(knob)),
    )

    # Off-axis handle tip proves the knob really rotates about the door normal.
    tip_rest = ctx.part_element_world_aabb(knob, elem="handle_tip")
    with ctx.pose({latch: KNOB_TURN}):
        tip_turn = ctx.part_element_world_aabb(knob, elem="handle_tip")
    ctx.check(
        "turning latch sweeps the handle tip sideways and upward",
        tip_rest is not None
        and tip_turn is not None
        and abs(tip_turn[0][0] - tip_rest[0][0]) > 0.012
        and tip_turn[0][2] > tip_rest[0][2] + 0.012,
        details=f"rest={tip_rest}, turned={tip_turn}",
    )

    # Riveted top cap detail present along the top rail.
    rivet_aabb = ctx.part_element_world_aabb(body, elem="rivet_0")
    ctx.check(
        "rivet dots stand proud of the top rail face",
        rivet_aabb is not None
        and rivet_aabb[1][1] > FRONT_Y + 0.003
        and rivet_aabb[0][2] > TOP_RAIL_BOT,
        details=str(rivet_aabb),
    )

    return ctx.report()


object_model = build_object_model()
