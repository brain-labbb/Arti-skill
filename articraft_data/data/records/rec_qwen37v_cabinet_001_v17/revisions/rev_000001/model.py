from __future__ import annotations

"""Locker-style steel cabinet with one full-width hinged door, visible barrel
hinges, vent slots, and block feet at the base.

Overall envelope ~1.6 m wide x 0.5 m deep x ~1.8 m tall, brushed/tarnished raw
steel. A hollow thin-wall (~0.02 m) carcass sits on four block feet and carries
a thin riveted top cap strip. The front opening is a single full-width door
hinged on the left side with three exposed barrel hinges. The door carries a
vertical ventilation slot near the bottom (dark recessed slot with rounded
ends), stamped vent lines near the top, and a quarter-turn latch knob at
mid-height on the free (right) edge.
"""

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BarrelHingeGeometry,
    Box,
    Cylinder,
    HingeHolePattern,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Global dimensions (meters). Cabinet is centred on X, front face at +Y.
# ---------------------------------------------------------------------------
CAB_W = 1.60  # overall carcass width  (X)
CAB_D = 0.50  # overall carcass depth  (Y)
CAB_TOP = 1.80  # carcass top height   (Z)
FOOT_H = 0.10  # block foot height; carcass starts here
WALL_T = 0.02  # thin steel wall thickness

FRONT_Y = CAB_D / 2.0  # +0.25, front face plane
BACK_Y = -CAB_D / 2.0

BOTTOM_RAIL_TOP = FOOT_H + 0.06  # 0.16
TOP_RAIL_BOT = CAB_TOP - 0.06  # 1.74

# Single full-width door
DOOR_W = CAB_W - 2.0 * WALL_T - 0.004  # ~1.556 m with clearance
DOOR_T = WALL_T
DOOR_Z0 = BOTTOM_RAIL_TOP + 0.002  # 0.162
DOOR_Z1 = TOP_RAIL_BOT - 0.002  # 1.738
DOOR_H = DOOR_Z1 - DOOR_Z0  # 1.576
DOOR_ZC = 0.5 * (DOOR_Z0 + DOOR_Z1)  # 0.950

# Hinge line on left side
HINGE_X = -CAB_W / 2.0 + WALL_T  # inner face of left wall

SLOT_LEN = 0.40  # dark rounded-end vent slot near the bottom
SLOT_W = 0.035
SLOT_ZC = -0.45  # in door-local z (door centre = 0)

CAP_T = 0.022  # riveted top cap strip
CAP_OVERHANG = 0.02

DOOR_OPEN = math.radians(110.0)
KNOB_TURN = math.radians(90.0)

# Block feet
FOOT_W = 0.08
FOOT_D = 0.08


def _door_solid(mesh_name: str):
    """Door leaf as one CadQuery solid: flat panel extending along +X from
    the hinge line, with a rounded-end through slot near the bottom."""
    xc = DOOR_W / 2.0  # panel centre offset from hinge edge
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


def _foot_solid(mesh_name: str):
    """Block foot: simple rectangular steel block with slightly chamfered top."""
    foot = (
        cq.Workplane("XY")
        .box(FOOT_W, FOOT_D, FOOT_H)
        .translate((0.0, 0.0, FOOT_H / 2.0))
    )
    return mesh_from_cadquery(foot, mesh_name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="locker_steel_cabinet")

    steel_body = model.material("steel_body", rgba=(0.60, 0.61, 0.63, 1.0))
    steel_door = model.material("steel_door", rgba=(0.55, 0.56, 0.58, 1.0))
    steel_trim = model.material("steel_trim", rgba=(0.46, 0.47, 0.49, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.07, 0.07, 0.08, 1.0))
    steel_foot = model.material("steel_foot", rgba=(0.38, 0.39, 0.41, 1.0))
    steel_knob = model.material("steel_knob", rgba=(0.18, 0.18, 0.20, 1.0))
    steel_rivet = model.material("steel_rivet", rgba=(0.52, 0.53, 0.55, 1.0))
    steel_hinge = model.material("steel_hinge", rgba=(0.42, 0.43, 0.45, 1.0))

    # ------------------------------------------------------------------
    # Cabinet body: hollow carcass + feet + front frame + top cap + rivets
    # ------------------------------------------------------------------
    body = model.part("cabinet_body")
    carcass_h = CAB_TOP - FOOT_H  # 1.70
    carcass_zc = FOOT_H + carcass_h / 2.0

    # Side walls (full depth, full carcass height).
    for sx, vname in ((-1.0, "side_wall_left"), (1.0, "side_wall_right")):
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
        origin=Origin(xyz=(0.0, 0.0, FOOT_H + WALL_T / 2.0)),
        material=steel_body,
        name="bottom_panel",
    )
    body.visual(
        Box((CAB_W, CAB_D, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, CAB_TOP - WALL_T / 2.0)),
        material=steel_body,
        name="top_panel",
    )
    # Interior shelf (thin, embedded into side walls).
    body.visual(
        Box((CAB_W - 2.0 * WALL_T + 0.01, 0.43, 0.015)),
        origin=Origin(xyz=(0.0, -0.02, 0.95)),
        material=steel_body,
        name="interior_shelf",
    )
    # Front frame: bottom rail and top rail only (single door opening).
    body.visual(
        Box((CAB_W - 2.0 * WALL_T, WALL_T, BOTTOM_RAIL_TOP - FOOT_H + 0.01)),
        origin=Origin(
            xyz=(0.0, FRONT_Y - WALL_T / 2.0, (FOOT_H + BOTTOM_RAIL_TOP) / 2.0)
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

    # Block feet at the four corners.
    foot_mesh = _foot_solid("block_foot")
    foot_corners = [
        (CAB_W / 2.0 - FOOT_W / 2.0 - 0.02, CAB_D / 2.0 - FOOT_D / 2.0 - 0.02),
        (-CAB_W / 2.0 + FOOT_W / 2.0 + 0.02, CAB_D / 2.0 - FOOT_D / 2.0 - 0.02),
        (-CAB_W / 2.0 + FOOT_W / 2.0 + 0.02, -CAB_D / 2.0 + FOOT_D / 2.0 + 0.02),
        (CAB_W / 2.0 - FOOT_W / 2.0 - 0.02, -CAB_D / 2.0 + FOOT_D / 2.0 + 0.02),
    ]
    for i, (fx, fy) in enumerate(foot_corners):
        body.visual(
            foot_mesh,
            origin=Origin(xyz=(fx, fy, 0.0)),
            material=steel_foot,
            name=f"foot_{i}",
        )

    # ------------------------------------------------------------------
    # Single full-width door hinged on the left side
    # ------------------------------------------------------------------
    door = model.part("door")

    door.visual(
        _door_solid("door_leaf"),
        material=steel_door,
        name="leaf",
    )
    # Dark backing plate behind the through slot -> recessed dark slot.
    door.visual(
        Box((SLOT_W + 0.016, 0.005, SLOT_LEN + 0.036)),
        origin=Origin(xyz=(DOOR_W / 2.0, -DOOR_T - 0.001, SLOT_ZC)),
        material=steel_dark,
        name="vent_backing",
    )
    # Stamped vent lines near the top (slightly proud thin dark strips).
    for j, dz in enumerate((0.58, 0.60, 0.62, 0.64)):
        door.visual(
            Box((0.22, 0.004, 0.006)),
            origin=Origin(xyz=(DOOR_W / 2.0, -0.0012, dz)),
            material=steel_dark,
            name=f"vent_line_{j}",
        )

    # Three visible barrel hinges on the left side (top, middle, bottom).
    hinge_length = 0.090
    hinge_positions_z = [
        DOOR_Z0 + 0.15,  # bottom hinge
        DOOR_ZC,  # middle hinge
        DOOR_Z1 - 0.15,  # top hinge
    ]
    for i, hz in enumerate(hinge_positions_z):
        hinge_geom = BarrelHingeGeometry(
            hinge_length,
            leaf_width_a=0.028,
            leaf_width_b=0.024,
            leaf_thickness=0.003,
            pin_diameter=0.005,
            knuckle_outer_diameter=0.014,
            knuckle_count=5,
            clearance=0.0005,
            open_angle_deg=110.0,
            holes_a=HingeHolePattern(
                style="round", count=3, diameter=0.004, edge_margin=0.008
            ),
            holes_b=HingeHolePattern(
                style="round", count=2, diameter=0.004, edge_margin=0.010
            ),
        )
        hinge_mesh = mesh_from_geometry(hinge_geom, f"barrel_hinge_{i}")
        # Place hinge at the left edge of the door, centered at hz.
        # The hinge leaf_a (frame side) is fixed to the body, leaf_b (door side)
        # moves with the door. We attach the whole hinge visual to the door for
        # simplicity (it rotates with the door when opened).
        door.visual(
            hinge_mesh,
            origin=Origin(xyz=(0.0, 0.0, hz - DOOR_ZC)),
            material=steel_hinge,
            name=f"hinge_barrel_{i}",
        )

    # Articulation: door hinge on left side, vertical axis
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(HINGE_X, FRONT_Y, DOOR_ZC)),
        # +Z axis: positive rotation swings the free edge (+X) outward (+Y)
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0, velocity=2.0, lower=0.0, upper=DOOR_OPEN
        ),
    )

    # ------------------------------------------------------------------
    # Quarter-turn latch knob at mid-height near the free (right) edge
    # ------------------------------------------------------------------
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
    # Off-axis teardrop tip at the lower end of the bar.
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
        # On the door front face, near the free (right) edge at mid-height.
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
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("door_hinge")
    knob = object_model.get_part("latch_knob")
    latch = object_model.get_articulation("latch")

    # --- Intentional overlap allowances ---
    # The barrel hinge leaves intentionally embed slightly into the frame and
    # door surfaces where they mount (captured hardware).
    ctx.allow_overlap(
        door,
        body,
        elem_a="hinge_barrel_0",
        elem_b="side_wall_left",
        reason="Barrel hinge frame leaf intentionally laps the fixed left wall where it mounts.",
    )
    ctx.allow_overlap(
        door,
        body,
        elem_a="hinge_barrel_1",
        elem_b="side_wall_left",
        reason="Barrel hinge frame leaf intentionally laps the fixed left wall where it mounts.",
    )
    ctx.allow_overlap(
        door,
        body,
        elem_a="hinge_barrel_2",
        elem_b="side_wall_left",
        reason="Barrel hinge frame leaf intentionally laps the fixed left wall where it mounts.",
    )

    # --- Overall envelope, true scale, grounded on the floor ---
    aabb = ctx.part_world_aabb(body)
    ok = aabb is not None
    ctx.check("cabinet body has bounds", ok, details=str(aabb))
    if aabb is not None:
        (x0, y0, z0), (x1, y1, z1) = aabb
        ctx.check(
            "overall width ~1.6 m",
            1.55 <= (x1 - x0) <= 1.70,
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
        ctx.check("feet rest on the floor", abs(z0) <= 1e-6, details=f"zmin={z0:.5f}")

    # --- Block feet present at the base ---
    for i in range(4):
        foot_aabb = ctx.part_element_world_aabb(body, elem=f"foot_{i}")
        ctx.check(
            f"foot_{i} is present at the base",
            foot_aabb is not None and foot_aabb[0][2] < 0.001,
            details=str(foot_aabb),
        )

    # --- Door: single full-width door, hinge type/axis/range ---
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

    # Hinge is on the left side
    ctx.check(
        "door hinge is on the left side",
        hinge.origin.xyz[0] < -0.70,
        details=f"hinge_x={hinge.origin.xyz[0]:.3f}",
    )

    # Door covers most of the cabinet width
    door_aabb = ctx.part_element_world_aabb(door, elem="leaf")
    ctx.check(
        "door spans most of the cabinet width",
        door_aabb is not None and (door_aabb[1][0] - door_aabb[0][0]) > 1.40,
        details=str(door_aabb),
    )

    # Closed leaf sits flush in the front frame plane
    ctx.check(
        "closed door leaf is flush with the front face",
        door_aabb is not None
        and abs(door_aabb[1][1] - FRONT_Y) < 1e-4
        and abs(door_aabb[0][1] - (FRONT_Y - DOOR_T)) < 1e-4,
        details=str(door_aabb),
    )

    # --- Visible barrel hinges on the door side ---
    for i in range(3):
        hinge_aabb = ctx.part_element_world_aabb(door, elem=f"hinge_barrel_{i}")
        ctx.check(
            f"hinge_barrel_{i} is present on the door side",
            hinge_aabb is not None and hinge_aabb[0][0] < -0.60,
            details=str(hinge_aabb),
        )

    # --- Vent slots on the door ---
    vb = ctx.part_element_world_aabb(door, elem="vent_backing")
    ctx.check(
        "vent slot sits in the lower half of the door",
        vb is not None and vb[1][2] < DOOR_ZC and vb[0][2] > DOOR_Z0,
        details=str(vb),
    )

    # --- Opening pose: door swings outward ---
    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({hinge: DOOR_OPEN}):
        open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "open door swings outward past the front face",
        open_aabb is not None and open_aabb[1][1] > FRONT_Y + 0.25,
        details=f"open={open_aabb}",
    )
    # At 110 deg the free edge wraps past perpendicular and swings to the left
    # of the hinge, so check that the door extends far outward in +Y instead.
    ctx.check(
        "open door extends far outward from the front face",
        open_aabb is not None and open_aabb[1][1] - FRONT_Y > 1.0,
        details=f"outward_extent={open_aabb[1][1] - FRONT_Y:.3f}" if open_aabb else "",
    )
    # Also check at a moderate angle that the free edge moves rightward.
    with ctx.pose({hinge: math.radians(45.0)}):
        partial_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "at 45 deg the door free edge is to the right of the hinge",
        partial_aabb is not None and partial_aabb[1][0] > HINGE_X + 0.5,
        details=str(partial_aabb),
    )

    # --- Latch knob ---
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
        name="latch knob backplate seats on the door face",
    )
    ctx.check(
        "latch knob sits near the free (right) edge",
        latch.origin.xyz[0] > 0.50,
        details=f"knob_x={latch.origin.xyz[0]:.3f}",
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
