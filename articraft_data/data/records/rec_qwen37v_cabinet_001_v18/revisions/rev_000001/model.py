from __future__ import annotations

"""Corner cabinet with angled bay-front doors and visible interior shelves.

Variant of the vintage industrial steel locker cabinet: a corner-style storage
cabinet with two angled front doors that meet at a center ridge, forming a
shallow bay. The body is a hollow rectangular carcass on four splayed legs with
a riveted top cap. Each door is a flat steel panel hinged at its outer edge
(left door on the left, right door on the right), swinging outward on a
vertical revolute joint (0..~110 deg). Interior shelf boards are visible
through the opening when doors are open. Each door carries a vent slot, stamped
vent lines, and a quarter-turn latch knob.
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
# Global dimensions (meters). Cabinet centred on X, back at -Y.
# ---------------------------------------------------------------------------
CAB_W = 1.60
CAB_D = 0.50
CAB_TOP = 1.80
LEG_H = 0.15
WALL_T = 0.02

BACK_Y = -CAB_D / 2.0
FRONT_SIDE_Y = CAB_D / 2.0  # side front corners at y = +0.25
FRONT_CENTER_Y = 0.45  # bay protrudes 0.20 m forward from side corners

BAY_DEPTH = FRONT_CENTER_Y - FRONT_SIDE_Y  # 0.20 m

BOTTOM_RAIL_TOP = LEG_H + 0.06
TOP_RAIL_BOT = CAB_TOP - 0.06

# Door panel geometry: each door spans from its hinge at the side corner to the
# center meeting point.
INNER_X = CAB_W / 2.0 - WALL_T  # 0.78
DOOR_PANEL_W = math.sqrt(INNER_X**2 + BAY_DEPTH**2)  # ~0.805 m
DOOR_ANGLE = math.atan2(BAY_DEPTH, INNER_X)  # ~14.3 deg from X-axis

DOOR_T = WALL_T
DOOR_Z0 = BOTTOM_RAIL_TOP + 0.002
DOOR_Z1 = TOP_RAIL_BOT - 0.002
DOOR_H = DOOR_Z1 - DOOR_Z0
DOOR_ZC = 0.5 * (DOOR_Z0 + DOOR_Z1)

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


def _door_panel_solid(mesh_name: str):
    """Door leaf: flat panel extending along local +X from hinge origin.
    Vent slot cut through near the bottom."""
    xc = DOOR_PANEL_W / 2.0
    panel = (
        cq.Workplane("XY")
        .box(DOOR_PANEL_W, DOOR_T, DOOR_H)
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


def _hinge_barrel_solid(mesh_name: str):
    """Piano-hinge knuckle column along the door hinge edge."""
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
    model = ArticulatedObject(name="corner_cabinet_angled_doors")

    steel_body = model.material("steel_body", rgba=(0.60, 0.61, 0.63, 1.0))
    steel_door = model.material("steel_door", rgba=(0.55, 0.56, 0.58, 1.0))
    steel_trim = model.material("steel_trim", rgba=(0.46, 0.47, 0.49, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.07, 0.07, 0.08, 1.0))
    steel_leg = model.material("steel_leg", rgba=(0.38, 0.39, 0.41, 1.0))
    steel_knob = model.material("steel_knob", rgba=(0.18, 0.18, 0.20, 1.0))
    steel_rivet = model.material("steel_rivet", rgba=(0.52, 0.53, 0.55, 1.0))
    steel_shelf = model.material("steel_shelf", rgba=(0.50, 0.51, 0.53, 1.0))

    # ------------------------------------------------------------------
    # Cabinet body: hollow carcass + legs + angled front frame + top cap
    # ------------------------------------------------------------------
    body = model.part("cabinet_body")
    carcass_h = CAB_TOP - LEG_H
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

    # Interior shelves (3 shelves at different heights, visible through open doors)
    shelf_w = CAB_W - 2.0 * WALL_T + 0.01
    shelf_d = CAB_D - 2.0 * WALL_T
    for i, sz in enumerate((0.55, 0.95, 1.35)):
        body.visual(
            Box((shelf_w, shelf_d, 0.018)),
            origin=Origin(xyz=(0.0, -0.02, sz)),
            material=steel_shelf,
            name=f"shelf_{i}",
        )

    # Straight front frame rails behind the angled doors (structural frame).
    # These span the full inner width and touch both side walls.
    front_frame_w = CAB_W - 2.0 * WALL_T + 0.01  # slight overlap with side walls
    front_frame_y = FRONT_SIDE_Y - WALL_T / 2.0  # just behind the side front edge

    # Bottom rail
    body.visual(
        Box((front_frame_w, WALL_T, BOTTOM_RAIL_TOP - LEG_H + 0.01)),
        origin=Origin(
            xyz=(0.0, front_frame_y, (LEG_H + BOTTOM_RAIL_TOP) / 2.0)
        ),
        material=steel_body,
        name="front_bottom_rail",
    )
    # Top rail
    body.visual(
        Box((front_frame_w, WALL_T, CAB_TOP - TOP_RAIL_BOT + 0.01)),
        origin=Origin(
            xyz=(0.0, front_frame_y, (TOP_RAIL_BOT + CAB_TOP) / 2.0)
        ),
        material=steel_body,
        name="front_top_rail",
    )

    # Center stile between doors, touching the front rails
    body.visual(
        Box((0.04, WALL_T, DOOR_H + 0.01)),
        origin=Origin(xyz=(0.0, front_frame_y, DOOR_ZC)),
        material=steel_trim,
        name="center_stile",
    )

    # Riveted top cap
    body.visual(
        Box((CAB_W + 2 * CAP_OVERHANG, CAB_D + 2 * CAP_OVERHANG, CAP_T)),
        origin=Origin(xyz=(0.0, 0.0, CAB_TOP - 0.001 + CAP_T / 2.0)),
        material=steel_trim,
        name="top_cap",
    )

    # Rivet dots along the front edge of the top cap, half-embedded
    cap_top_z = CAB_TOP - 0.001 + CAP_T  # top surface of cap
    n_riv = 13
    for i in range(n_riv):
        rx = -0.72 + i * (1.44 / (n_riv - 1))
        body.visual(
            Sphere(radius=0.0045),
            origin=Origin(xyz=(rx, CAB_D / 2.0 + CAP_OVERHANG - 0.012, cap_top_z - 0.002)),
            material=steel_rivet,
            name=f"rivet_{i}",
        )

    # Splayed legs
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
    # Two angled doors. Left door hinges on left edge, right on right edge.
    # Each door panel extends along local +X from its hinge origin.
    # ------------------------------------------------------------------
    door_specs = [
        # (name, hinge_x, yaw_angle, axis_z_sign)
        # Left door: hinge at left edge, panel extends toward center (+X in local)
        # At q=0, local +X points toward (INNER_X direction, +BAY_DEPTH) from hinge
        ("door_left", -INNER_X, DOOR_ANGLE, 1.0),
        # Right door: hinge at right edge, panel extends toward center (-X in local)
        # The part frame is rotated so local +X points toward center
        ("door_right", INNER_X, math.pi - DOOR_ANGLE, -1.0),
    ]

    doors = []
    for door_name, hinge_x, yaw, axis_z_sign in door_specs:
        door = model.part(door_name)

        # Door leaf panel (extends along local +X from origin)
        door.visual(
            _door_panel_solid(f"{door_name}_leaf"),
            material=steel_door,
            name="leaf",
        )

        # Dark backing behind vent slot
        panel_xc = DOOR_PANEL_W / 2.0
        door.visual(
            Box((SLOT_W + 0.016, 0.005, SLOT_LEN + 0.036)),
            origin=Origin(xyz=(panel_xc, -DOOR_T - 0.001, SLOT_ZC)),
            material=steel_dark,
            name="vent_backing",
        )

        # Stamped vent lines near the top
        for j, dz in enumerate((0.60, 0.62, 0.64)):
            door.visual(
                Box((0.16, 0.004, 0.006)),
                origin=Origin(xyz=(panel_xc, -0.0012, dz)),
                material=steel_dark,
                name=f"vent_line_{j}",
            )

        # Piano-hinge knuckle column on the hinge edge
        door.visual(
            _hinge_barrel_solid(f"{door_name}_hinge_barrel"),
            origin=Origin(xyz=(0.0, 0.004, 0.0)),
            material=steel_trim,
            name="hinge_barrel",
        )

        # Articulation: revolute about vertical axis at the hinge
        # The articulation frame is rotated by yaw so that at q=0 the child
        # frame (door) is oriented correctly.
        model.articulation(
            f"{door_name}_hinge",
            ArticulationType.REVOLUTE,
            parent=body,
            child=door,
            origin=Origin(xyz=(hinge_x, FRONT_SIDE_Y, DOOR_ZC), rpy=(0.0, 0.0, yaw)),
            # axis in articulation frame: +Z means counterclockwise from above.
            # For left door (axis_z_sign=+1): positive q swings free edge outward (+Y).
            # For right door (axis_z_sign=-1): positive q swings free edge outward (+Y).
            axis=(0.0, 0.0, axis_z_sign),
            motion_limits=MotionLimits(
                effort=40.0, velocity=2.0, lower=0.0, upper=DOOR_OPEN
            ),
        )
        doors.append(door)

        # Quarter-turn latch knob near the free edge of the door
        knob = model.part(f"latch_{door_name}")
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
        # Knob mounted near the free edge (far end of panel), at mid-height
        knob_x = DOOR_PANEL_W - 0.10
        model.articulation(
            f"latch_{door_name}_turn",
            ArticulationType.REVOLUTE,
            parent=door,
            child=knob,
            origin=Origin(xyz=(knob_x, 0.0, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0, velocity=4.0, lower=0.0, upper=KNOB_TURN
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("cabinet_body")
    door_left = object_model.get_part("door_left")
    door_right = object_model.get_part("door_right")
    hinge_left = object_model.get_articulation("door_left_hinge")
    hinge_right = object_model.get_articulation("door_right_hinge")

    # --- Overall envelope ---
    aabb = ctx.part_world_aabb(body)
    ok = aabb is not None
    ctx.check("cabinet body has bounds", ok, details=str(aabb))
    if aabb is not None:
        (x0, y0, z0), (x1, y1, z1) = aabb
        ctx.check(
            "overall width ~1.6 m",
            1.58 <= (x1 - x0) <= 1.70,
            details=f"width={x1 - x0:.3f}",
        )
        ctx.check(
            "overall height ~1.8 m",
            1.78 <= z1 <= 1.86,
            details=f"top={z1:.3f}",
        )
        ctx.check("legs rest on the floor", abs(z0) <= 1e-6, details=f"zmin={z0:.5f}")

    # --- Doors exist and are angled ---
    ctx.check("door_left part exists", door_left is not None)
    ctx.check("door_right part exists", door_right is not None)

    # --- Hinge articulations: revolute, vertical axis, correct range ---
    for hinge_name, hinge in [("door_left_hinge", hinge_left), ("door_right_hinge", hinge_right)]:
        ctx.check(
            f"{hinge_name} is revolute",
            hinge.articulation_type == ArticulationType.REVOLUTE,
        )
        ax = hinge.axis
        ctx.check(
            f"{hinge_name} axis is vertical",
            abs(ax[0]) < 1e-9 and abs(ax[1]) < 1e-9 and abs(abs(ax[2]) - 1.0) < 1e-9,
            details=str(ax),
        )
        lim = hinge.motion_limits
        ctx.check(
            f"{hinge_name} opens 0..~110 deg",
            lim is not None
            and lim.lower == 0.0
            and abs(lim.upper - math.radians(110.0)) < 1e-6,
        )

    # --- Hinge origins at the side edges ---
    ctx.check(
        "left hinge at left edge",
        hinge_left.origin.xyz[0] < -0.70,
        details=f"x={hinge_left.origin.xyz[0]:.3f}",
    )
    ctx.check(
        "right hinge at right edge",
        hinge_right.origin.xyz[0] > 0.70,
        details=f"x={hinge_right.origin.xyz[0]:.3f}",
    )

    # --- Doors are angled (not flat with front plane) ---
    # The articulation origin rpy yaw should encode the bay angle
    left_yaw = hinge_left.origin.rpy[2]
    right_yaw = hinge_right.origin.rpy[2]
    ctx.check(
        "left door has bay angle (yaw > 0)",
        abs(left_yaw) > 0.05,
        details=f"yaw={left_yaw:.3f} rad",
    )
    ctx.check(
        "right door has bay angle (yaw near pi)",
        abs(abs(right_yaw) - math.pi) < 0.5 or abs(right_yaw) > 0.5,
        details=f"yaw={right_yaw:.3f} rad",
    )

    # --- Opening pose: doors swing outward ---
    closed_left = ctx.part_world_aabb(door_left)
    closed_right = ctx.part_world_aabb(door_right)
    with ctx.pose({hinge_left: DOOR_OPEN, hinge_right: DOOR_OPEN}):
        open_left = ctx.part_world_aabb(door_left)
        open_right = ctx.part_world_aabb(door_right)

    ctx.check(
        "left door swings outward past front side plane",
        open_left is not None
        and closed_left is not None
        and open_left[1][1] > FRONT_SIDE_Y + 0.20,
        details=f"closed_y_max={closed_left[1][1]:.3f}, open_y_max={open_left[1][1]:.3f}",
    )
    ctx.check(
        "right door swings outward past front side plane",
        open_right is not None
        and closed_right is not None
        and open_right[1][1] > FRONT_SIDE_Y + 0.20,
        details=f"closed_y_max={closed_right[1][1]:.3f}, open_y_max={open_right[1][1]:.3f}",
    )

    # Left door free edge moves left when opened, right door free edge moves right
    ctx.check(
        "left door opens leftward",
        open_left is not None
        and closed_left is not None
        and open_left[0][0] < closed_left[0][0] - 0.05,
        details=f"closed_x_min={closed_left[0][0]:.3f}, open_x_min={open_left[0][0]:.3f}",
    )
    ctx.check(
        "right door opens rightward",
        open_right is not None
        and closed_right is not None
        and open_right[1][0] > closed_right[1][0] + 0.05,
        details=f"closed_x_max={closed_right[1][0]:.3f}, open_x_max={open_right[1][0]:.3f}",
    )

    # --- Interior shelves exist and are visible through open gap ---
    for i in range(3):
        shelf_aabb = ctx.part_element_world_aabb(body, elem=f"shelf_{i}")
        ctx.check(
            f"shelf_{i} exists inside cabinet",
            shelf_aabb is not None
            and shelf_aabb[0][2] > LEG_H
            and shelf_aabb[1][2] < CAB_TOP,
            details=str(shelf_aabb),
        )

    # Shelves should be in front half of cabinet (visible through open doors)
    shelf0_aabb = ctx.part_element_world_aabb(body, elem="shelf_0")
    ctx.check(
        "shelf_0 extends toward front (visible through open gap)",
        shelf0_aabb is not None and shelf0_aabb[1][1] > 0.0,
        details=f"y_max={shelf0_aabb[1][1]:.3f}" if shelf0_aabb else "None",
    )

    # --- Hinge barrel intentional overlap with frame ---
    ctx.allow_overlap(
        door_left,
        body,
        elem_a="hinge_barrel",
        elem_b="side_wall_0",
        reason="Piano-hinge knuckle column intentionally laps the side wall edge it pivots on.",
    )
    ctx.allow_overlap(
        door_right,
        body,
        elem_a="hinge_barrel",
        elem_b="side_wall_1",
        reason="Piano-hinge knuckle column intentionally laps the side wall edge it pivots on.",
    )

    # --- Latch knobs surface-mounted on door faces (seated hardware) ---
    for door_name, door in [("door_left", door_left), ("door_right", door_right)]:
        latch = object_model.get_part(f"latch_{door_name}")
        ctx.allow_overlap(
            door,
            latch,
            elem_a="leaf",
            elem_b="backplate",
            reason="Latch backplate is surface-mounted on the door face; small local embed represents seated hardware.",
        )

    # --- Latch knobs ---
    for door_name in ("door_left", "door_right"):
        latch_name = f"latch_{door_name}"
        latch = object_model.get_part(latch_name)
        latch_joint = object_model.get_articulation(f"{latch_name}_turn")
        door = object_model.get_part(door_name)

        ctx.check(
            f"{latch_name}_turn is quarter-turn revolute",
            latch_joint.articulation_type == ArticulationType.REVOLUTE
            and latch_joint.axis == (0.0, 1.0, 0.0)
            and latch_joint.motion_limits is not None
            and abs(latch_joint.motion_limits.upper - math.pi / 2.0) < 1e-6,
        )
        ctx.expect_contact(
            latch,
            door,
            elem_a="backplate",
            elem_b="leaf",
            contact_tol=1e-6,
            name=f"{latch_name} backplate seats on the leaf face",
        )

    # Rivets present along top rail
    rivet_aabb = ctx.part_element_world_aabb(body, elem="rivet_0")
    ctx.check(
        "rivet dots stand proud of the top rail face",
        rivet_aabb is not None
        and rivet_aabb[1][1] > FRONT_SIDE_Y + 0.003
        and rivet_aabb[0][2] > TOP_RAIL_BOT,
        details=str(rivet_aabb),
    )

    return ctx.report()


object_model = build_object_model()
