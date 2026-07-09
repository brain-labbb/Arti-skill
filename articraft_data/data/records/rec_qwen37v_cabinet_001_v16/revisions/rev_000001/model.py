from __future__ import annotations

"""Vintage industrial steel locker cabinet — variant with open side cubbies,
central hinged doors, and a prismatic tambour front.

Overall envelope ~1.6 m wide x 0.5 m deep x ~1.8 m tall, brushed/tarnished raw
steel. A hollow thin-wall (~0.02 m) carcass sits on four short splayed legs and
carries a thin riveted top cap strip. The front is divided into three zones:

- Left open cubby (0.28 m wide) with three visible shelf boards.
- Central closed cabinet (~1.00 m wide): upper half has two hinged doors
  (revolute about vertical Z axes, 0..~110 deg outward), lower half has a
  tambour panel that slides sideways on a prismatic joint along +X.
- Right open cubby (0.28 m wide) with three visible shelf boards.

Small gap seams surround all moving fronts (doors and tambour). Each door
carries a latch knob, dark recessed vent slot, and stamped vent lines.
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

# Section layout along X
CUBBY_W = 0.28
DIVIDER_W = 0.02
CENTER_W = CAB_W - 2.0 * CUBBY_W - 2.0 * DIVIDER_W  # 1.00

# Section X boundaries
CUBBY_L_X0 = -CAB_W / 2.0 + WALL_T  # -0.78
CUBBY_L_X1 = CUBBY_L_X0 + CUBBY_W   # -0.50
DIV_L_X0 = CUBBY_L_X1                # -0.50
DIV_L_X1 = DIV_L_X0 + DIVIDER_W      # -0.48
CENTER_X0 = DIV_L_X1                 # -0.48
CENTER_X1 = CENTER_X0 + CENTER_W     # +0.52
DIV_R_X0 = CENTER_X1                 # +0.52
DIV_R_X1 = DIV_R_X0 + DIVIDER_W      # +0.54
CUBBY_R_X0 = DIV_R_X1               # +0.54
CUBBY_R_X1 = CUBBY_R_X0 + CUBBY_W   # +0.82

# Z boundaries for front frame rails
BOTTOM_RAIL_TOP = LEG_H + 0.06  # 0.21
TOP_RAIL_BOT = CAB_TOP - 0.06   # 1.74
MID_RAIL_Z = 0.90
MID_RAIL_T = 0.04
MID_RAIL_BOT = MID_RAIL_Z - MID_RAIL_T / 2.0  # 0.88
MID_RAIL_TOP = MID_RAIL_Z + MID_RAIL_T / 2.0  # 0.92

# Tambour section
TAMB_Z0 = BOTTOM_RAIL_TOP + 0.003  # 0.213 (gap seam at bottom)
TAMB_Z1 = MID_RAIL_BOT - 0.003    # 0.877 (gap seam at top)
TAMB_H = TAMB_Z1 - TAMB_Z0       # ~0.664
TAMB_ZC = 0.5 * (TAMB_Z0 + TAMB_Z1)
TAMB_OPEN_W = CENTER_W            # 1.00 opening width
TAMB_W = TAMB_OPEN_W - 0.008      # 0.992 panel width (4mm gap each side)
TAMB_T = WALL_T                   # 0.02 thick
TAMB_TRAVEL = 0.50                # prismatic travel along +X

# Door section
DOOR_Z0 = MID_RAIL_TOP + 0.003    # 0.923 (gap seam)
DOOR_Z1 = TOP_RAIL_BOT - 0.003    # 1.737 (gap seam)
DOOR_H = DOOR_Z1 - DOOR_Z0       # ~0.814
DOOR_ZC = 0.5 * (DOOR_Z0 + DOOR_Z1)
DOOR_OPEN_W = CENTER_W / 2.0 - 0.016  # each door ~0.484 wide (gap seams around stile)
DOOR_T = WALL_T

DOOR_OPEN = math.radians(110.0)
KNOB_TURN = math.radians(90.0)

# Hinge positions
HINGE_LEFT_X = CENTER_X0 + 0.001   # left door hinge near left edge of opening
HINGE_RIGHT_X = CENTER_X1 - 0.001  # right door hinge near right edge

# Cubby shelves
SHELF_T = 0.015
N_CUBBY_SHELVES = 3

CAP_T = 0.022
CAP_OVERHANG = 0.02

SLOT_LEN = 0.22  # shorter slot for narrower doors
SLOT_W = 0.025
SLOT_ZC = -0.25  # door-local z

BARREL_R = 0.007
KNUCKLE_R = 0.009
BARREL_LEN = DOOR_H - 0.03


def _door_solid(sign: float, mesh_name: str):
    """Door leaf: flat panel with rounded-end through slot near the bottom.
    sign=+1 -> panel extends along +X (left-hinged);
    sign=-1 -> along -X (right-hinged)."""
    xc = sign * DOOR_OPEN_W / 2.0
    panel = (
        cq.Workplane("XY")
        .box(DOOR_OPEN_W, DOOR_T, DOOR_H)
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
    ring_h = 0.05
    for zc in (-0.30, 0.0, 0.30):
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


def _tambour_solid(mesh_name: str):
    """Tambour sliding panel: flat steel sheet with horizontal slat grooves."""
    panel = (
        cq.Workplane("XY")
        .box(TAMB_W, TAMB_T, TAMB_H)
    )
    # Horizontal slat groove lines
    n_grooves = 8
    groove_spacing = TAMB_H / (n_grooves + 1)
    for i in range(n_grooves):
        gz = -TAMB_H / 2.0 + groove_spacing * (i + 1)
        groove = (
            cq.Workplane("XY")
            .box(TAMB_W - 0.02, 0.003, 0.004)
            .translate((0.0, TAMB_T / 2.0 + 0.001, gz))
        )
        panel = panel.union(groove)
    return mesh_from_cadquery(panel, mesh_name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_steel_locker_cabinet")

    steel_body = model.material("steel_body", rgba=(0.60, 0.61, 0.63, 1.0))
    steel_door = model.material("steel_door", rgba=(0.55, 0.56, 0.58, 1.0))
    steel_trim = model.material("steel_trim", rgba=(0.46, 0.47, 0.49, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.07, 0.07, 0.08, 1.0))
    steel_leg = model.material("steel_leg", rgba=(0.38, 0.39, 0.41, 1.0))
    steel_knob = model.material("steel_knob", rgba=(0.18, 0.18, 0.20, 1.0))
    steel_rivet = model.material("steel_rivet", rgba=(0.52, 0.53, 0.55, 1.0))
    steel_tambour = model.material("steel_tambour", rgba=(0.52, 0.53, 0.56, 1.0))
    steel_shelf = model.material("steel_shelf", rgba=(0.58, 0.59, 0.61, 1.0))

    # ------------------------------------------------------------------
    # Cabinet body: hollow carcass + dividers + legs + frame + cap + rivets
    # + cubby shelves
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

    # Vertical dividers between cubbies and central cabinet
    for dx, vname in ((0.5 * (DIV_L_X0 + DIV_L_X1), "divider_left"),
                       (0.5 * (DIV_R_X0 + DIV_R_X1), "divider_right")):
        body.visual(
            Box((DIVIDER_W, CAB_D - 2.0 * WALL_T, carcass_h)),
            origin=Origin(xyz=(dx, 0.0, carcass_zc)),
            material=steel_body,
            name=vname,
        )

    # Front frame: bottom rail, top rail, middle rail (across central section)
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
    # Middle rail across central section only
    body.visual(
        Box((CENTER_W + 0.01, WALL_T, MID_RAIL_T)),
        origin=Origin(xyz=(0.5 * (CENTER_X0 + CENTER_X1), FRONT_Y - WALL_T / 2.0, MID_RAIL_Z)),
        material=steel_trim,
        name="front_mid_rail",
    )
    # Center stile between upper doors
    body.visual(
        Box((0.02, WALL_T, DOOR_H + 0.01)),
        origin=Origin(xyz=(0.5 * (CENTER_X0 + CENTER_X1), FRONT_Y - WALL_T / 2.0, DOOR_ZC)),
        material=steel_trim,
        name="center_stile",
    )

    # Cubby shelves (visible through the open front)
    cubby_inner_h = carcass_h - 2.0 * WALL_T  # between bottom and top panels
    for cubby_idx, (cx0, cx1) in enumerate([(CUBBY_L_X0, CUBBY_L_X1), (CUBBY_R_X0, CUBBY_R_X1)]):
        cubby_cx = 0.5 * (cx0 + cx1)
        cubby_w = cx1 - cx0
        shelf_depth = CAB_D - 3.0 * WALL_T  # inset from back wall
        for si in range(N_CUBBY_SHELVES):
            frac = (si + 1) / (N_CUBBY_SHELVES + 1)
            sz = LEG_H + WALL_T + frac * cubby_inner_h
            body.visual(
                Box((cubby_w - 0.005, shelf_depth, SHELF_T)),
                origin=Origin(xyz=(cubby_cx, -WALL_T / 2.0, sz)),
                material=steel_shelf,
                name=f"cubby{cubby_idx}_shelf_{si}",
            )

    # Tambour guide rails (thin tracks at top and bottom of tambour opening)
    for rz, rname in ((TAMB_Z0 - 0.002, "tambour_rail_bottom"),
                       (TAMB_Z1 + 0.002, "tambour_rail_top")):
        body.visual(
            Box((CENTER_W + 0.04, 0.012, 0.006)),
            origin=Origin(xyz=(0.5 * (CENTER_X0 + CENTER_X1), FRONT_Y - 0.006, rz)),
            material=steel_trim,
            name=rname,
        )

    # Thin riveted top cap strip
    body.visual(
        Box((CAB_W + 2 * CAP_OVERHANG, CAB_D + 2 * CAP_OVERHANG, CAP_T)),
        origin=Origin(xyz=(0.0, 0.0, CAB_TOP - 0.001 + CAP_T / 2.0)),
        material=steel_trim,
        name="top_cap",
    )
    # Raised rivet dots along the top rail
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

    # Interior shelf in central cabinet (visible when tambour is open)
    body.visual(
        Box((CENTER_W - 0.01, CAB_D - 3.0 * WALL_T, SHELF_T)),
        origin=Origin(xyz=(0.5 * (CENTER_X0 + CENTER_X1), -WALL_T, TAMB_ZC)),
        material=steel_shelf,
        name="center_shelf",
    )

    # ------------------------------------------------------------------
    # Two upper doors (revolute, hinge on outer edges, open outward)
    # ------------------------------------------------------------------
    # door_0: left door, hinges on left edge, panel extends +X
    # door_1: right door, hinges on right edge, panel extends -X
    door_specs = [
        (HINGE_LEFT_X, +1.0),   # left door
        (HINGE_RIGHT_X, -1.0),  # right door
    ]
    doors = []
    for i, (hinge_x, sign) in enumerate(door_specs):
        door = model.part(f"door_{i}")
        xc = sign * DOOR_OPEN_W / 2.0

        door.visual(
            _door_solid(sign, f"door_leaf_{i}"),
            material=steel_door,
            name="leaf",
        )
        # Dark backing plate behind the vent slot
        door.visual(
            Box((SLOT_W + 0.014, 0.005, SLOT_LEN + 0.03)),
            origin=Origin(xyz=(xc, -DOOR_T - 0.001, SLOT_ZC)),
            material=steel_dark,
            name="vent_backing",
        )
        # Stamped vent lines near top
        for j, dz in enumerate((0.30, 0.32, 0.34)):
            door.visual(
                Box((0.14, 0.004, 0.005)),
                origin=Origin(xyz=(xc, -0.001, dz)),
                material=steel_dark,
                name=f"vent_line_{j}",
            )
        # Piano-hinge knuckle column
        door.visual(
            _hinge_barrel_solid(f"hinge_barrel_{i}"),
            origin=Origin(xyz=(0.0, 0.004, 0.0)),
            material=steel_trim,
            name="hinge_barrel",
        )

        model.articulation(
            f"door_{i}_hinge",
            ArticulationType.REVOLUTE,
            parent=body,
            child=door,
            origin=Origin(xyz=(hinge_x, FRONT_Y, DOOR_ZC)),
            # Left door (+X extends right): +Z axis opens outward (left swing)
            # Right door (-X extends left): -Z axis opens outward (right swing)
            axis=(0.0, 0.0, sign),
            motion_limits=MotionLimits(
                effort=40.0, velocity=2.0, lower=0.0, upper=DOOR_OPEN
            ),
        )
        doors.append(door)

        # Latch knob
        knob = model.part(f"latch_knob_{i}")
        knob.visual(
            Cylinder(radius=0.016, length=0.005),
            origin=Origin(xyz=(0.0, 0.0025, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel_knob,
            name="backplate",
        )
        knob.visual(
            Cylinder(radius=0.006, length=0.012),
            origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel_knob,
            name="boss",
        )
        knob.visual(
            Box((0.009, 0.007, 0.030)),
            origin=Origin(xyz=(0.0, 0.018, 0.0)),
            material=steel_knob,
            name="handle_bar",
        )
        knob.visual(
            Sphere(radius=0.005),
            origin=Origin(xyz=(0.0, 0.018, -0.017)),
            material=steel_knob,
            name="handle_tip",
        )
        model.articulation(
            f"latch_{i}",
            ArticulationType.REVOLUTE,
            parent=door,
            child=knob,
            origin=Origin(xyz=(sign * (DOOR_OPEN_W - 0.08), 0.0, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0, velocity=4.0, lower=0.0, upper=KNOB_TURN
            ),
        )

    # ------------------------------------------------------------------
    # Tambour front (prismatic, slides along +X)
    # ------------------------------------------------------------------
    tambour = model.part("tambour")
    tambour.visual(
        _tambour_solid("tambour_panel"),
        material=steel_tambour,
        name="panel",
    )
    # Small handle/pull on the tambour face
    tambour.visual(
        Box((0.06, 0.008, 0.018)),
        origin=Origin(xyz=(-TAMB_W / 2.0 + 0.06, TAMB_T / 2.0 + 0.004, 0.0)),
        material=steel_knob,
        name="pull_handle",
    )

    # Tambour part frame at the closed (centered) position.
    # The prismatic joint origin is at the center of the opening;
    # the child frame coincides with it at q=0.
    tamb_center_x = 0.5 * (CENTER_X0 + CENTER_X1)
    model.articulation(
        "tambour_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tambour,
        origin=Origin(xyz=(tamb_center_x, FRONT_Y - TAMB_T / 2.0, TAMB_ZC)),
        axis=(1.0, 0.0, 0.0),  # slides along +X
        motion_limits=MotionLimits(
            effort=30.0, velocity=1.5, lower=0.0, upper=TAMB_TRAVEL
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("cabinet_body")
    doors = [object_model.get_part(f"door_{i}") for i in range(2)]
    hinges = [object_model.get_articulation(f"door_{i}_hinge") for i in range(2)]
    knobs = [object_model.get_part(f"latch_knob_{i}") for i in range(2)]
    latches = [object_model.get_articulation(f"latch_{i}") for i in range(2)]
    tambour = object_model.get_part("tambour")
    tamb_slide = object_model.get_articulation("tambour_slide")

    # Intentional local laps: hinge knuckle columns embed into frame edges
    frame_elems = ["divider_left", "divider_right"]
    for door, elem in zip(doors, frame_elems):
        ctx.allow_overlap(
            door,
            body,
            elem_a="hinge_barrel",
            elem_b=elem,
            reason="Piano-hinge knuckle column intentionally laps the divider edge it pivots on.",
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

    # --- Side cubbies: open front with visible shelves ----------------------
    for ci, (cx0, cx1) in enumerate([(CUBBY_L_X0, CUBBY_L_X1), (CUBBY_R_X0, CUBBY_R_X1)]):
        for si in range(N_CUBBY_SHELVES):
            shelf_name = f"cubby{ci}_shelf_{si}"
            saabb = ctx.part_element_world_aabb(body, elem=shelf_name)
            ctx.check(
                f"{shelf_name} exists within cubby bounds",
                saabb is not None
                and saabb[0][0] >= cx0 - 0.01
                and saabb[1][0] <= cx1 + 0.01
                and saabb[0][2] > LEG_H
                and saabb[1][2] < CAB_TOP,
                details=str(saabb),
            )

    # --- Doors: geometry, hinge type/axis/range, closed seating -------------
    for i, (door, hinge) in enumerate(zip(doors, hinges)):
        ctx.check(
            f"door_{i} hinge is revolute",
            hinge.articulation_type == ArticulationType.REVOLUTE,
        )
        ax = hinge.axis
        ctx.check(
            f"door_{i} hinge axis is vertical",
            abs(ax[0]) < 1e-9 and abs(ax[1]) < 1e-9 and abs(abs(ax[2]) - 1.0) < 1e-9,
            details=str(ax),
        )
        lim = hinge.motion_limits
        ctx.check(
            f"door_{i} opens 0..~110 deg",
            lim is not None
            and lim.lower == 0.0
            and abs(lim.upper - math.radians(110.0)) < 1e-6,
        )
        # Closed leaf sits flush in the front frame plane
        daabb = ctx.part_element_world_aabb(door, elem="leaf")
        ctx.check(
            f"door_{i} closed leaf is flush with the front face",
            daabb is not None
            and abs(daabb[1][1] - FRONT_Y) < 1e-3
            and abs(daabb[0][1] - (FRONT_Y - DOOR_T)) < 1e-3,
            details=str(daabb),
        )
        # Gap seam: door stays within cabinet width when closed
        ctx.expect_within(
            door,
            body,
            axes="x",
            margin=0.005,
            name=f"door_{i} gap seam keeps it inside cabinet width",
        )

    # Left door hinges on left edge, right door on right edge
    ctx.check(
        "doors hinge on outer edges of central section",
        hinges[0].origin.xyz[0] < CENTER_X0 + 0.02
        and hinges[1].origin.xyz[0] > CENTER_X1 - 0.02,
        details=str([h.origin.xyz[0] for h in hinges]),
    )

    # Opening pose: leaves swing outward
    closed0 = ctx.part_world_aabb(doors[0])
    closed1 = ctx.part_world_aabb(doors[1])
    with ctx.pose({hinges[0]: DOOR_OPEN, hinges[1]: DOOR_OPEN}):
        open0 = ctx.part_world_aabb(doors[0])
        open1 = ctx.part_world_aabb(doors[1])
    ctx.check(
        "open doors swing outward past the front face",
        open0 is not None
        and open1 is not None
        and open0[1][1] > FRONT_Y + 0.15
        and open1[1][1] > FRONT_Y + 0.15,
        details=f"open0={open0}, open1={open1}",
    )
    ctx.check(
        "doors open away from cabinet center",
        closed0 is not None
        and closed1 is not None
        and open0[0][0] < closed0[0][0] - 0.05
        and open1[1][0] > closed1[1][0] + 0.05,
        details=f"closed0={closed0}, open0={open0}",
    )

    # --- Tambour: prismatic joint, sideways slide --------------------------
    ctx.check(
        "tambour_slide is prismatic",
        tamb_slide.articulation_type == ArticulationType.PRISMATIC,
    )
    ctx.check(
        "tambour_slide axis is along X",
        abs(tamb_slide.axis[0] - 1.0) < 1e-9
        and abs(tamb_slide.axis[1]) < 1e-9
        and abs(tamb_slide.axis[2]) < 1e-9,
        details=str(tamb_slide.axis),
    )
    tlim = tamb_slide.motion_limits
    ctx.check(
        "tambour_slide range 0..~0.50 m",
        tlim is not None and tlim.lower == 0.0 and abs(tlim.upper - TAMB_TRAVEL) < 1e-6,
    )

    # Tambour at rest covers the opening
    taabb_rest = ctx.part_world_aabb(tambour)
    ctx.check(
        "tambour closed covers central opening",
        taabb_rest is not None
        and taabb_rest[0][0] < CENTER_X0 + 0.02
        and taabb_rest[1][0] > CENTER_X1 - 0.02,
        details=str(taabb_rest),
    )

    # Tambour at max slide moves rightward
    with ctx.pose({tamb_slide: TAMB_TRAVEL}):
        taabb_open = ctx.part_world_aabb(tambour)
    ctx.check(
        "tambour slides rightward revealing left side of opening",
        taabb_open is not None
        and taabb_open[0][0] > taabb_rest[0][0] + 0.40,
        details=f"rest={taabb_rest}, open={taabb_open}",
    )

    # Tambour gap seam: panel fits within the opening height
    ctx.expect_gap(
        body,
        tambour,
        axis="z",
        min_gap=-0.005,
        max_gap=0.010,
        positive_elem="front_mid_rail",
        name="tambour top gap seam below mid rail",
    )

    # --- Latch knobs --------------------------------------------------------
    for i, (knob, latch, door) in enumerate(zip(knobs, latches, doors)):
        ctx.check(
            f"latch_{i} is a quarter-turn revolute about the door normal",
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
            name=f"latch_knob_{i} backplate seats on the leaf face",
        )

    # Off-axis handle tip proves the knob rotates
    tip_rest = ctx.part_element_world_aabb(knobs[0], elem="handle_tip")
    with ctx.pose({latches[0]: KNOB_TURN}):
        tip_turn = ctx.part_element_world_aabb(knobs[0], elem="handle_tip")
    ctx.check(
        "turning latch_0 sweeps the handle tip",
        tip_rest is not None
        and tip_turn is not None
        and abs(tip_turn[0][0] - tip_rest[0][0]) > 0.010
        and tip_turn[0][2] > tip_rest[0][2] + 0.010,
        details=f"rest={tip_rest}, turned={tip_turn}",
    )

    # Riveted top cap
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
