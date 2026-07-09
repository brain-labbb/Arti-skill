from __future__ import annotations

"""Vintage industrial steel locker cabinet — variant with open side cubbies,
closed central cabinet with two hinged doors, a rotating center-seam latch,
visible cubby shelf boards, and recessed panel borders on the doors.

Overall envelope ~1.6 m wide x 0.5 m deep x ~1.8 m tall, brushed/tarnished raw
steel. A hollow thin-wall (~0.02 m) carcass sits on four short splayed legs and
carries a thin riveted top cap strip. The front is divided into three bays:
the left and right bays are open cubbies with visible shelf boards, and the
centre bay has two full-height hinged doors (left door hinges on its left edge,
right door hinges on its right edge, opening away from centre). A small
rotating latch at the centre seam locks both doors. Each door carries a dark
recessed ventilation slot with rounded ends near the bottom, stamped vent
lines near the top, and raised rectangular border strips forming a recessed
panel effect.
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

# Centre bay pocket edges (between stiles at x=-0.3975 and x=+0.3975)
CENTRE_LEFT_EDGE = -0.3825
CENTRE_RIGHT_EDGE = 0.3825

DOOR_W = 0.364
DOOR_T = WALL_T
DOOR_Z0 = BOTTOM_RAIL_TOP + 0.002  # 0.212
DOOR_Z1 = TOP_RAIL_BOT - 0.002  # 1.738
DOOR_H = DOOR_Z1 - DOOR_Z0  # 1.526
DOOR_ZC = 0.5 * (DOOR_Z0 + DOOR_Z1)  # 0.975
HINGE_INSET = 0.0005

SLOT_LEN = 0.34
SLOT_W = 0.030
SLOT_ZC = -0.40  # door-local z

BARREL_R = 0.0075
KNUCKLE_R = 0.0095
BARREL_LEN = DOOR_H - 0.03

CAP_T = 0.022
CAP_OVERHANG = 0.02

DOOR_OPEN = math.radians(110.0)
LATCH_TURN = math.radians(90.0)

# Cubby dimensions
CUBBY_LEFT_X0 = -CAB_W / 2.0 + WALL_T  # -0.78
CUBBY_LEFT_X1 = -0.3975 - STILE_W / 2.0  # ~-0.4125
CUBBY_RIGHT_X0 = 0.3975 + STILE_W / 2.0
CUBBY_RIGHT_X1 = CAB_W / 2.0 - WALL_T

CUBBY_SHELF_W = CUBBY_LEFT_X1 - CUBBY_LEFT_X0  # ~0.3675
CUBBY_SHELF_D = CAB_D - 2.0 * WALL_T - 0.01  # ~0.47
CUBBY_SHELF_T = 0.015

# Recessed panel border
BORDER_W = 0.032
BORDER_PROUD = 0.003


def _door_solid(sign: float, mesh_name: str):
    """Door leaf: flat panel with a rounded-end through slot near the bottom."""
    xc = sign * DOOR_W / 2.0
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
    model = ArticulatedObject(name="vintage_steel_locker_cabinet_v26")

    steel_body = model.material("steel_body", rgba=(0.60, 0.61, 0.63, 1.0))
    steel_door = model.material("steel_door", rgba=(0.55, 0.56, 0.58, 1.0))
    steel_trim = model.material("steel_trim", rgba=(0.46, 0.47, 0.49, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.07, 0.07, 0.08, 1.0))
    steel_leg = model.material("steel_leg", rgba=(0.38, 0.39, 0.41, 1.0))
    steel_rivet = model.material("steel_rivet", rgba=(0.52, 0.53, 0.55, 1.0))
    steel_shelf = model.material("steel_shelf", rgba=(0.50, 0.51, 0.53, 1.0))
    steel_latch = model.material("steel_latch", rgba=(0.22, 0.22, 0.24, 1.0))
    steel_border = model.material("steel_border", rgba=(0.48, 0.49, 0.51, 1.0))

    # ------------------------------------------------------------------
    # Cabinet body: hollow carcass + legs + front frame + top cap + rivets
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
    # Centre interior shelf (behind the doors)
    body.visual(
        Box((CENTRE_RIGHT_EDGE - CENTRE_LEFT_EDGE - 0.01, CAB_D - 3 * WALL_T, 0.015)),
        origin=Origin(xyz=(0.0, -WALL_T, 0.95)),
        material=steel_shelf,
        name="centre_shelf",
    )

    # Front frame: bottom rail, top rail, three stiles
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
    stile_h = TOP_RAIL_BOT - BOTTOM_RAIL_TOP + 0.01
    for i, xc in enumerate((-0.3975, 0.0, 0.3975)):
        body.visual(
            Box((STILE_W, WALL_T, stile_h)),
            origin=Origin(xyz=(xc, FRONT_Y - WALL_T / 2.0, DOOR_ZC)),
            material=steel_trim,
            name=f"front_stile_{i}",
        )

    # Riveted top cap strip
    body.visual(
        Box((CAB_W + 2 * CAP_OVERHANG, CAB_D + 2 * CAP_OVERHANG, CAP_T)),
        origin=Origin(xyz=(0.0, 0.0, CAB_TOP - 0.001 + CAP_T / 2.0)),
        material=steel_trim,
        name="top_cap",
    )
    n_riv = 13
    for i in range(n_riv):
        rx = -0.72 + i * (1.44 / (n_riv - 1))
        body.visual(
            Sphere(radius=0.0045),
            origin=Origin(xyz=(rx, FRONT_Y + 0.002, 1.77)),
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
    # Open side cubbies: shelf boards visible through the open front
    # ------------------------------------------------------------------
    shelf_heights = [0.55, 0.95, 1.35]
    for side, (x0, x1) in enumerate(
        ((CUBBY_LEFT_X0, CUBBY_LEFT_X1), (CUBBY_RIGHT_X0, CUBBY_RIGHT_X1))
    ):
        shelf_xc = 0.5 * (x0 + x1)
        for j, sz in enumerate(shelf_heights):
            body.visual(
                Box((CUBBY_SHELF_W, CUBBY_SHELF_D, CUBBY_SHELF_T)),
                origin=Origin(xyz=(shelf_xc, -WALL_T, sz)),
                material=steel_shelf,
                name=f"cubby_shelf_{side}_{j}",
            )

    # Cubby back panels (close off the back of the cubbies from the centre)
    for side, (x0, x1) in enumerate(
        ((CUBBY_LEFT_X0, CUBBY_LEFT_X1), (CUBBY_RIGHT_X0, CUBBY_RIGHT_X1))
    ):
        cubby_w = x1 - x0
        xc = 0.5 * (x0 + x1)
        # Thin divider wall between cubby and centre section
        body.visual(
            Box((WALL_T, CAB_D - 2 * WALL_T, DOOR_H)),
            origin=Origin(
                xyz=(
                    x1 if side == 0 else x0,
                    -WALL_T,
                    DOOR_ZC,
                )
            ),
            material=steel_body,
            name=f"cubby_divider_{side}",
        )

    # ------------------------------------------------------------------
    # Two centre doors (door_1 = centre-left, door_2 = centre-right)
    # ------------------------------------------------------------------
    door_specs = [
        # (door_index, hinge world x, sign, label)
        (1, CENTRE_LEFT_EDGE + HINGE_INSET, +1.0, "door_1"),  # centre-left, left-hinged
        (2, CENTRE_RIGHT_EDGE - HINGE_INSET, -1.0, "door_2"),  # centre-right, right-hinged
    ]
    doors = []
    for door_idx, hinge_x, sign, door_name in door_specs:
        door = model.part(door_name)
        xc = sign * DOOR_W / 2.0

        # Door leaf with vent slot
        door.visual(
            _door_solid(sign, f"door_leaf_{door_idx}"),
            material=steel_door,
            name="leaf",
        )

        # Dark backing plate behind the vent slot
        door.visual(
            Box((SLOT_W + 0.016, 0.005, SLOT_LEN + 0.036)),
            origin=Origin(xyz=(xc, -DOOR_T - 0.001, SLOT_ZC)),
            material=steel_dark,
            name="vent_backing",
        )

        # Stamped vent lines near the top
        for j, dz in enumerate((0.60, 0.62, 0.64)):
            door.visual(
                Box((0.16, 0.004, 0.006)),
                origin=Origin(xyz=(xc, -0.0012, dz)),
                material=steel_dark,
                name=f"vent_line_{j}",
            )

        # Piano-hinge knuckle column
        door.visual(
            _hinge_barrel_solid(f"hinge_barrel_{door_idx}"),
            origin=Origin(xyz=(0.0, 0.004, 0.0)),
            material=steel_trim,
            name="hinge_barrel",
        )

        # Recessed panel border: 4 raised strips forming a frame on the door face
        # Left/right vertical borders
        inner_w = DOOR_W - 2.0 * BORDER_W
        inner_h = DOOR_H - 2.0 * BORDER_W
        for bx in (BORDER_W / 2.0, DOOR_W - BORDER_W / 2.0):
            door.visual(
                Box((BORDER_W, BORDER_PROUD, DOOR_H)),
                origin=Origin(xyz=(sign * bx, BORDER_PROUD / 2.0, 0.0)),
                material=steel_border,
                name=f"border_v_{int(bx*1000)}",
            )
        # Top/bottom horizontal borders
        for bz in (DOOR_H / 2.0 - BORDER_W / 2.0, -DOOR_H / 2.0 + BORDER_W / 2.0):
            door.visual(
                Box((inner_w, BORDER_PROUD, BORDER_W)),
                origin=Origin(xyz=(xc, BORDER_PROUD / 2.0, bz)),
                material=steel_border,
                name=f"border_h_{int(abs(bz)*100)}",
            )

        # Door hinge articulation
        model.articulation(
            f"door_{door_idx}_hinge",
            ArticulationType.REVOLUTE,
            parent=body,
            child=door,
            origin=Origin(xyz=(hinge_x, FRONT_Y, DOOR_ZC)),
            axis=(0.0, 0.0, sign),
            motion_limits=MotionLimits(
                effort=40.0, velocity=2.0, lower=0.0, upper=DOOR_OPEN
            ),
        )
        doors.append(door)

    # ------------------------------------------------------------------
    # Centre rotating latch at the seam (mounted on body, catches both doors)
    # ------------------------------------------------------------------
    latch = model.part("center_latch")
    # Base plate (circular, mounted on the centre stile)
    latch.visual(
        Cylinder(radius=0.022, length=0.006),
        origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_latch,
        name="latch_base",
    )
    # Rotating boss
    latch.visual(
        Cylinder(radius=0.009, length=0.010),
        origin=Origin(xyz=(0.0, 0.011, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_latch,
        name="latch_boss",
    )
    # Latch arm: horizontal bar that bridges the centre seam when locked
    latch.visual(
        Box((0.10, 0.008, 0.018)),
        origin=Origin(xyz=(0.0, 0.019, 0.0)),
        material=steel_latch,
        name="latch_arm",
    )
    # Small knob at the end of the arm
    latch.visual(
        Sphere(radius=0.008),
        origin=Origin(xyz=(0.05, 0.019, 0.0)),
        material=steel_latch,
        name="latch_knob",
    )

    model.articulation(
        "center_latch_joint",
        ArticulationType.REVOLUTE,
        parent=body,
        child=latch,
        origin=Origin(xyz=(0.0, FRONT_Y, DOOR_ZC)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0, velocity=4.0, lower=0.0, upper=LATCH_TURN
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("cabinet_body")
    door_1 = object_model.get_part("door_1")
    door_2 = object_model.get_part("door_2")
    hinge_1 = object_model.get_articulation("door_1_hinge")
    hinge_2 = object_model.get_articulation("door_2_hinge")
    latch = object_model.get_part("center_latch")
    latch_joint = object_model.get_articulation("center_latch_joint")

    # Intentional local laps: piano-hinge knuckle columns embed into frame edges
    ctx.allow_overlap(
        door_1,
        body,
        elem_a="hinge_barrel",
        elem_b="front_stile_0",
        reason="Piano-hinge knuckle column laps the fixed stile it pivots on.",
    )
    ctx.allow_overlap(
        door_2,
        body,
        elem_a="hinge_barrel",
        elem_b="front_stile_2",
        reason="Piano-hinge knuckle column laps the fixed stile it pivots on.",
    )

    # --- Overall envelope ---------------------------------------------------
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

    # --- Doors: hinge type, axis, range, closed seating ---------------------
    for door, hinge, name in ((door_1, hinge_1, "door_1"), (door_2, hinge_2, "door_2")):
        ctx.check(
            f"{name} hinge is revolute",
            hinge.articulation_type == ArticulationType.REVOLUTE,
        )
        ax = hinge.axis
        ctx.check(
            f"{name} hinge axis is vertical",
            abs(ax[0]) < 1e-9 and abs(ax[1]) < 1e-9 and abs(abs(ax[2]) - 1.0) < 1e-9,
            details=str(ax),
        )
        lim = hinge.motion_limits
        ctx.check(
            f"{name} opens 0..~110 deg",
            lim is not None
            and lim.lower == 0.0
            and abs(lim.upper - math.radians(110.0)) < 1e-6,
        )
        # Closed leaf sits flush with front face
        daabb = ctx.part_element_world_aabb(door, elem="leaf")
        ctx.check(
            f"{name} closed leaf flush with front face",
            daabb is not None
            and abs(daabb[1][1] - FRONT_Y) < 1e-4
            and abs(daabb[0][1] - (FRONT_Y - DOOR_T)) < 1e-4,
            details=str(daabb),
        )
        ctx.expect_within(
            door,
            body,
            axes="x",
            margin=0.012,
            name=f"{name} stays inside cabinet width when closed",
        )

    # Hinge sides: door_1 hinges on left edge, door_2 on right edge
    ctx.check(
        "door_1 hinges on left, door_2 hinges on right",
        hinge_1.origin.xyz[0] < -0.37 and hinge_2.origin.xyz[0] > 0.37,
        details=f"h1={hinge_1.origin.xyz[0]:.3f}, h2={hinge_2.origin.xyz[0]:.3f}",
    )

    # Opening pose: leaves swing outward
    closed1 = ctx.part_world_aabb(door_1)
    closed2 = ctx.part_world_aabb(door_2)
    with ctx.pose({hinge_1: DOOR_OPEN, hinge_2: DOOR_OPEN}):
        open1 = ctx.part_world_aabb(door_1)
        open2 = ctx.part_world_aabb(door_2)
    ctx.check(
        "doors swing outward past the front face",
        open1 is not None
        and open2 is not None
        and open1[1][1] > FRONT_Y + 0.25
        and open2[1][1] > FRONT_Y + 0.25,
    )
    ctx.check(
        "doors open away from centre",
        closed1 is not None
        and closed2 is not None
        and open1[0][0] < closed1[0][0] - 0.05
        and open2[1][0] > closed2[1][0] + 0.05,
    )

    # --- Recessed panel borders on doors ------------------------------------
    for door, name in ((door_1, "door_1"), (door_2, "door_2")):
        # Check that border strips exist and sit proud of the leaf face
        border_aabb = ctx.part_element_world_aabb(door, elem="border_v_16")
        leaf_aabb = ctx.part_element_world_aabb(door, elem="leaf")
        ctx.check(
            f"{name} has recessed panel border strips",
            border_aabb is not None and leaf_aabb is not None,
            details=f"border={border_aabb}, leaf={leaf_aabb}",
        )
        if border_aabb is not None and leaf_aabb is not None:
            ctx.check(
                f"{name} border stands proud of leaf face",
                border_aabb[1][1] > leaf_aabb[1][1] - 1e-6,
                details=f"border_max_y={border_aabb[1][1]:.5f}, leaf_max_y={leaf_aabb[1][1]:.5f}",
            )

    # --- Open side cubbies with visible shelf boards ------------------------
    for side in (0, 1):
        for j in range(3):
            shelf_name = f"cubby_shelf_{side}_{j}"
            saabb = ctx.part_element_world_aabb(body, elem=shelf_name)
            ctx.check(
                f"{shelf_name} exists in the cubby",
                saabb is not None,
                details=str(saabb),
            )
            if saabb is not None:
                # Shelves should be within the door-opening height range
                ctx.check(
                    f"{shelf_name} sits within the opening height",
                    saabb[0][2] > DOOR_Z0 - 0.01 and saabb[1][2] < DOOR_Z1 + 0.01,
                    details=str(saabb),
                )
                # Shelves should be visible from the front (y extends to near FRONT_Y)
                ctx.check(
                    f"{shelf_name} is visible from the front",
                    saabb[1][1] > FRONT_Y - 0.06,
                    details=f"shelf_front={saabb[1][1]:.3f}",
                )

    # --- Centre rotating latch ----------------------------------------------
    ctx.check(
        "center latch joint is revolute",
        latch_joint.articulation_type == ArticulationType.REVOLUTE,
    )
    ctx.check(
        "center latch axis is along door normal (Y)",
        latch_joint.axis == (0.0, 1.0, 0.0),
        details=str(latch_joint.axis),
    )
    lim = latch_joint.motion_limits
    ctx.check(
        "center latch rotates 0..90 deg",
        lim is not None
        and lim.lower == 0.0
        and abs(lim.upper - math.pi / 2.0) < 1e-6,
    )
    # Latch sits at the centre seam
    latch_pos = latch_joint.origin.xyz
    ctx.check(
        "center latch is at the centre seam",
        abs(latch_pos[0]) < 0.02,
        details=f"latch_x={latch_pos[0]:.4f}",
    )
    ctx.check(
        "center latch is at door mid-height",
        abs(latch_pos[2] - DOOR_ZC) < 0.03,
        details=f"latch_z={latch_pos[2]:.3f}",
    )

    # Latch arm sweeps when rotated
    arm_rest = ctx.part_element_world_aabb(latch, elem="latch_arm")
    with ctx.pose({latch_joint: LATCH_TURN}):
        arm_turned = ctx.part_element_world_aabb(latch, elem="latch_arm")
    ctx.check(
        "rotating the latch sweeps the arm",
        arm_rest is not None
        and arm_turned is not None
        and abs(arm_turned[0][0] - arm_rest[0][0]) > 0.02,
        details=f"rest={arm_rest}, turned={arm_turned}",
    )

    # Latch base seats on the body front face
    ctx.expect_contact(
        latch,
        body,
        elem_a="latch_base",
        elem_b="front_stile_1",
        contact_tol=0.005,
        name="latch base seats on the centre stile",
    )

    # Riveted top cap detail
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
