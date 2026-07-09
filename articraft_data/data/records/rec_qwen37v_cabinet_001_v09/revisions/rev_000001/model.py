from __future__ import annotations

"""Vintage industrial steel locker cabinet – variant 09.

Four full-height hinged doors with visible piano-hinge barrels, a lift-up top
lid over shallow storage, and a small rotating T-latch that locks the two
centre doors at the centre seam.

Reference image: picture/Other/Cabinet/001.png

Overall envelope ~1.6 m wide x 0.5 m deep x ~1.9 m tall (including tray/lid),
brushed/tarnished raw steel. A hollow thin-wall (~0.02 m) carcass sits on four
short splayed legs and carries a thin riveted top cap strip. On top of the cap
a shallow tray rim creates a recessed storage compartment covered by a
rear-hinged lift-up lid. The front is split into four flat doors separated by
narrow stiles. The two left doors hinge on their left edges, the two right
doors hinge on their right edges (visible knuckle barrels on the hinge edges),
so the pairs swing open away from the cabinet centre. Each door is an
independent revolute joint about a vertical axis, 0..~110 deg outward. Each
door also carries a quarter-turn latch knob, a dark recessed ventilation slot,
and stamped vent lines near the top.
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
BACK_Y = -CAB_D / 2.0  # -0.25

BOTTOM_RAIL_TOP = LEG_H + 0.06  # 0.21
TOP_RAIL_BOT = CAB_TOP - 0.06  # 1.74

STILE_W = 0.03
POCKET_EDGES = [-0.78, -0.4125, -0.3825, -0.015, 0.015, 0.3825, 0.4125, 0.78]

DOOR_W = 0.364
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

# Lid and shallow storage tray
TRAY_DEPTH = 0.050
TRAY_Z0 = CAB_TOP + CAP_T - 0.001  # top of cap ≈ 1.821
LID_T = 0.015
LID_W = CAB_W - 2 * WALL_T - 0.012  # fits inside tray rims
LID_D = CAB_D - 2 * WALL_T - 0.012
LID_OPEN = math.radians(80.0)

# Centre latch
LATCH_TURN = math.radians(90.0)

DOOR_OPEN = math.radians(110.0)
KNOB_TURN = math.radians(90.0)


# ---------------------------------------------------------------------------
# Shared CadQuery meshes
# ---------------------------------------------------------------------------

def _door_solid(sign: float, mesh_name: str):
    """Door leaf: flat panel with a rounded-end through slot near the bottom."""
    xc = sign * DOOR_W / 2.0
    panel = (
        cq.Workplane("XY")
        .box(DOOR_W, DOOR_T, DOOR_H)
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


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_steel_locker_cabinet")

    steel_body = model.material("steel_body", rgba=(0.60, 0.61, 0.63, 1.0))
    steel_door_a = model.material("steel_door_a", rgba=(0.55, 0.56, 0.58, 1.0))
    steel_door_b = model.material("steel_door_b", rgba=(0.50, 0.51, 0.54, 1.0))
    steel_trim = model.material("steel_trim", rgba=(0.46, 0.47, 0.49, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.07, 0.07, 0.08, 1.0))
    steel_leg = model.material("steel_leg", rgba=(0.38, 0.39, 0.41, 1.0))
    steel_knob = model.material("steel_knob", rgba=(0.18, 0.18, 0.20, 1.0))
    steel_rivet = model.material("steel_rivet", rgba=(0.52, 0.53, 0.55, 1.0))
    steel_lid = model.material("steel_lid", rgba=(0.57, 0.58, 0.60, 1.0))
    steel_latch = model.material("steel_latch", rgba=(0.22, 0.22, 0.24, 1.0))

    # ------------------------------------------------------------------
    # Cabinet body: hollow carcass + legs + frame + cap + tray + lid hinge
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
    # Interior shelf
    body.visual(
        Box((CAB_W - 2.0 * WALL_T + 0.01, 0.43, 0.015)),
        origin=Origin(xyz=(0.0, -0.02, 0.95)),
        material=steel_body,
        name="interior_shelf",
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
        origin=Origin(
            xyz=(0.0, FRONT_Y - WALL_T / 2.0, (TOP_RAIL_BOT + CAB_TOP) / 2.0)
        ),
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

    # Thin riveted top cap strip with a slight overhang
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

    # --- Shallow storage tray rim (on top of the cap) -----------------
    tray_zc = TRAY_Z0 + TRAY_DEPTH / 2.0
    inner_w = CAB_W - 2 * WALL_T
    inner_d = CAB_D - 2 * WALL_T
    # Front tray wall
    body.visual(
        Box((inner_w, WALL_T, TRAY_DEPTH)),
        origin=Origin(xyz=(0.0, FRONT_Y - WALL_T / 2.0, tray_zc)),
        material=steel_trim,
        name="tray_wall_front",
    )
    # Back tray wall
    body.visual(
        Box((inner_w, WALL_T, TRAY_DEPTH)),
        origin=Origin(xyz=(0.0, BACK_Y + WALL_T / 2.0, tray_zc)),
        material=steel_trim,
        name="tray_wall_back",
    )
    # Left tray wall
    body.visual(
        Box((WALL_T, inner_d - 2 * WALL_T, TRAY_DEPTH)),
        origin=Origin(xyz=(-(CAB_W / 2.0 - WALL_T / 2.0), 0.0, tray_zc)),
        material=steel_trim,
        name="tray_wall_left",
    )
    # Right tray wall
    body.visual(
        Box((WALL_T, inner_d - 2 * WALL_T, TRAY_DEPTH)),
        origin=Origin(xyz=(CAB_W / 2.0 - WALL_T / 2.0, 0.0, tray_zc)),
        material=steel_trim,
        name="tray_wall_right",
    )
    # Tray floor
    body.visual(
        Box((inner_w - 0.005, inner_d - 0.005, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, TRAY_Z0 + 0.002)),
        material=steel_body,
        name="tray_floor",
    )

    # --- Lid hinge barrels (stationary, on the body back edge) ---------
    lid_hinge_z = TRAY_Z0 + TRAY_DEPTH  # top of tray walls
    for i, bx in enumerate((-0.50, 0.0, 0.50)):
        body.visual(
            Cylinder(radius=0.010, length=0.050),
            origin=Origin(
                xyz=(bx, BACK_Y + WALL_T + 0.015, lid_hinge_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=steel_trim,
            name=f"lid_hinge_barrel_{i}",
        )

    # --- Legs ----------------------------------------------------------
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
    # Top lid – rear-hinged lift-up panel over the shallow tray
    # ------------------------------------------------------------------
    lid = model.part("top_lid")
    # Panel extends from hinge (y=0) toward front (+Y), thickness in +Z
    lid.visual(
        Box((LID_W, LID_D, LID_T)),
        origin=Origin(xyz=(0.0, LID_D / 2.0, LID_T / 2.0)),
        material=steel_lid,
        name="lid_panel",
    )
    # Small grip lip on the front edge
    lid.visual(
        Box((0.14, 0.018, 0.022)),
        origin=Origin(xyz=(0.0, LID_D + 0.004, LID_T / 2.0)),
        material=steel_trim,
        name="lid_handle",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        # Hinge at the back edge of the tray top
        origin=Origin(xyz=(0.0, BACK_Y + WALL_T, lid_hinge_z)),
        axis=(1.0, 0.0, 0.0),  # +q lifts front edge upward
        motion_limits=MotionLimits(
            effort=15.0, velocity=2.0, lower=0.0, upper=LID_OPEN
        ),
    )

    # ------------------------------------------------------------------
    # Centre-seam rotating T-latch
    # ------------------------------------------------------------------
    center_latch = model.part("center_latch")
    # Mounting plate (proud of the front face)
    center_latch.visual(
        Box((0.044, 0.006, 0.064)),
        origin=Origin(xyz=(0.0, 0.003, 0.0)),
        material=steel_latch,
        name="latch_plate",
    )
    # Shaft boss
    center_latch.visual(
        Cylinder(radius=0.009, length=0.020),
        origin=Origin(xyz=(0.0, 0.016, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_latch,
        name="latch_shaft",
    )
    # Cross bar – spans both centre doors when locked (q=0)
    center_latch.visual(
        Box((0.100, 0.010, 0.016)),
        origin=Origin(xyz=(0.0, 0.030, 0.0)),
        material=steel_latch,
        name="latch_bar",
    )
    # Rounded tips on each end of the bar
    for sx, sfx in ((-1.0, "l"), (1.0, "r")):
        center_latch.visual(
            Sphere(radius=0.009),
            origin=Origin(xyz=(sx * 0.055, 0.030, 0.0)),
            material=steel_knob,
            name=f"latch_tip_{sfx}",
        )

    model.articulation(
        "center_latch_turn",
        ArticulationType.REVOLUTE,
        parent=body,
        child=center_latch,
        origin=Origin(xyz=(0.0, FRONT_Y, DOOR_ZC)),
        axis=(0.0, 1.0, 0.0),  # rotate about door normal
        motion_limits=MotionLimits(
            effort=5.0, velocity=4.0, lower=0.0, upper=LATCH_TURN
        ),
    )

    # ------------------------------------------------------------------
    # Four doors
    # ------------------------------------------------------------------
    door_specs = [
        (POCKET_EDGES[0] + HINGE_INSET, +1.0, steel_door_a),  # far left
        (POCKET_EDGES[2] + HINGE_INSET, +1.0, steel_door_b),  # centre-left
        (POCKET_EDGES[5] - HINGE_INSET, -1.0, steel_door_b),  # centre-right
        (POCKET_EDGES[7] - HINGE_INSET, -1.0, steel_door_a),  # far right
    ]
    doors = []
    for i, (hinge_x, sign, leaf_mat) in enumerate(door_specs):
        door = model.part(f"door_{i}")
        xc = sign * DOOR_W / 2.0

        door.visual(
            _door_solid(sign, f"door_leaf_{i}"),
            material=leaf_mat,
            name="leaf",
        )
        # Dark backing plate behind the through slot
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
        # Visible hinge barrel (knuckle column on the hinge edge)
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
            axis=(0.0, 0.0, sign),
            motion_limits=MotionLimits(
                effort=40.0, velocity=2.0, lower=0.0, upper=DOOR_OPEN
            ),
        )
        doors.append(door)

        # Quarter-turn latch knob at mid-height near the free edge
        knob = model.part(f"latch_knob_{i}")
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
            f"latch_{i}",
            ArticulationType.REVOLUTE,
            parent=door,
            child=knob,
            origin=Origin(xyz=(sign * (DOOR_W - 0.10), 0.0, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0, velocity=4.0, lower=0.0, upper=KNOB_TURN
            ),
        )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("cabinet_body")
    doors = [object_model.get_part(f"door_{i}") for i in range(4)]
    hinges = [object_model.get_articulation(f"door_{i}_hinge") for i in range(4)]
    knobs = [object_model.get_part(f"latch_knob_{i}") for i in range(4)]
    latches = [object_model.get_articulation(f"latch_{i}") for i in range(4)]
    lid = object_model.get_part("top_lid")
    lid_hinge = object_model.get_articulation("lid_hinge")
    center_latch = object_model.get_part("center_latch")
    center_latch_turn = object_model.get_articulation("center_latch_turn")

    # --- Intentional overlaps: door hinge barrels lap the frame edges ---
    frame_elems = ["side_wall_0", "front_stile_0", "front_stile_2", "side_wall_1"]
    for door, elem in zip(doors, frame_elems):
        ctx.allow_overlap(
            door,
            body,
            elem_a="hinge_barrel",
            elem_b=elem,
            reason="Piano-hinge knuckle column intentionally laps the fixed frame edge it pivots on.",
        )

    # Lid hinge barrels on the body embed slightly into the lid panel edge
    # at the pivot – this is the captured hinge hardware.
    for barrel_name in ("lid_hinge_barrel_0", "lid_hinge_barrel_1", "lid_hinge_barrel_2"):
        ctx.allow_overlap(
            body,
            lid,
            elem_a=barrel_name,
            elem_b="lid_panel",
            reason="Lid hinge barrel is the captured pivot hardware embedded in the lid panel edge.",
        )
    # Proof: lid hinge origin is near the lid panel rear edge
    ctx.expect_contact(
        body,
        lid,
        elem_a="lid_hinge_barrel_1",
        elem_b="lid_panel",
        contact_tol=0.020,
        name="lid hinge barrel contacts the lid panel at the pivot edge",
    )

    # --- Overall envelope -----------------------------------------------
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
            "overall height ~1.8 m+ (with tray)",
            1.78 <= z1 <= 2.00,
            details=f"top={z1:.3f}",
        )
        ctx.check(
            "legs rest on the floor", abs(z0) <= 1e-6, details=f"zmin={z0:.5f}"
        )

    # --- Doors: geometry, hinge type/axis/range, closed seating ---------
    for i, (door, hinge) in enumerate(zip(doors, hinges)):
        ctx.check(
            f"door_{i} hinge is revolute",
            hinge.articulation_type == ArticulationType.REVOLUTE,
        )
        ax = hinge.axis
        ctx.check(
            f"door_{i} hinge axis is vertical",
            abs(ax[0]) < 1e-9
            and abs(ax[1]) < 1e-9
            and abs(abs(ax[2]) - 1.0) < 1e-9,
            details=str(ax),
        )
        lim = hinge.motion_limits
        ctx.check(
            f"door_{i} opens 0..~110 deg",
            lim is not None
            and lim.lower == 0.0
            and abs(lim.upper - math.radians(110.0)) < 1e-6,
        )
        daabb = ctx.part_element_world_aabb(door, elem="leaf")
        ctx.check(
            f"door_{i} closed leaf is flush with the front face",
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
            name=f"door_{i} stays inside the cabinet width when closed",
        )
        vb = ctx.part_element_world_aabb(door, elem="vent_backing")
        ctx.check(
            f"door_{i} vent slot sits in the lower half of the leaf",
            vb is not None and vb[1][2] < DOOR_ZC and vb[0][2] > DOOR_Z0,
            details=str(vb),
        )
        # Visible hinge barrel on each door
        hb = ctx.part_element_world_aabb(door, elem="hinge_barrel")
        ctx.check(
            f"door_{i} has a visible hinge barrel spanning most of the door height",
            hb is not None and (hb[1][2] - hb[0][2]) > 0.5 * DOOR_H,
            details=str(hb),
        )

    # Hinge sides: outer doors at cabinet edges, centre pair beside them
    ctx.check(
        "left pair hinges on left edges, right pair on right edges",
        hinges[0].origin.xyz[0] < -0.77
        and -0.39 < hinges[1].origin.xyz[0] < -0.37
        and 0.37 < hinges[2].origin.xyz[0] < 0.39
        and hinges[3].origin.xyz[0] > 0.77,
        details=str([h.origin.xyz[0] for h in hinges]),
    )

    # Opening pose: leaves swing outward, pairs away from centre
    closed0 = ctx.part_world_aabb(doors[0])
    closed3 = ctx.part_world_aabb(doors[3])
    with ctx.pose({hinges[0]: DOOR_OPEN, hinges[3]: DOOR_OPEN}):
        open0 = ctx.part_world_aabb(doors[0])
        open3 = ctx.part_world_aabb(doors[3])
    ctx.check(
        "open leaves swing outward past the front face",
        open0 is not None
        and open3 is not None
        and open0[1][1] > FRONT_Y + 0.25
        and open3[1][1] > FRONT_Y + 0.25,
        details=f"open0={open0}, open3={open3}",
    )
    ctx.check(
        "outer pair opens away from the cabinet centre",
        closed0 is not None
        and closed3 is not None
        and open0[0][0] < closed0[0][0] - 0.05
        and open3[1][0] > closed3[1][0] + 0.05,
        details=f"closed0={closed0}, open0={open0}",
    )

    # --- Per-door latch knobs -------------------------------------------
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
        ctx.check(
            f"latch_knob_{i} sits at door mid-height",
            (
                lambda a: a is not None
                and abs(0.5 * (a[0][2] + a[1][2]) - DOOR_ZC) < 0.03
            )(ctx.part_world_aabb(knob)),
        )

    tip_rest = ctx.part_element_world_aabb(knobs[0], elem="handle_tip")
    with ctx.pose({latches[0]: KNOB_TURN}):
        tip_turn = ctx.part_element_world_aabb(knobs[0], elem="handle_tip")
    ctx.check(
        "turning latch_0 sweeps the handle tip sideways and upward",
        tip_rest is not None
        and tip_turn is not None
        and abs(tip_turn[0][0] - tip_rest[0][0]) > 0.012
        and tip_turn[0][2] > tip_rest[0][2] + 0.012,
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

    # --- Top lid --------------------------------------------------------
    ctx.check(
        "lid_hinge is revolute",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE,
    )
    ctx.check(
        "lid_hinge axis is horizontal along X (spans cabinet width)",
        abs(lid_hinge.axis[0] - 1.0) < 1e-9
        and abs(lid_hinge.axis[1]) < 1e-9
        and abs(lid_hinge.axis[2]) < 1e-9,
        details=str(lid_hinge.axis),
    )
    llim = lid_hinge.motion_limits
    ctx.check(
        "lid_hinge opens 0..~80 deg",
        llim is not None
        and llim.lower == 0.0
        and abs(llim.upper - LID_OPEN) < 1e-6,
    )

    # Closed lid seats on top of the cabinet tray
    lid_closed = ctx.part_element_world_aabb(lid, elem="lid_panel")
    ctx.check(
        "closed lid panel sits above the cabinet top",
        lid_closed is not None and lid_closed[0][2] > CAB_TOP - 0.01,
        details=str(lid_closed),
    )

    # Open lid: front edge rises
    with ctx.pose({lid_hinge: LID_OPEN}):
        lid_open = ctx.part_element_world_aabb(lid, elem="lid_panel")
    ctx.check(
        "open lid lifts the front edge well above the closed height",
        lid_open is not None
        and lid_closed is not None
        and lid_open[1][2] > lid_closed[1][2] + 0.10,
        details=f"closed_max_z={lid_closed[1][2] if lid_closed else None}, "
        f"open_max_z={lid_open[1][2] if lid_open else None}",
    )

    # --- Centre-seam rotating latch ------------------------------------
    ctx.check(
        "center_latch_turn is revolute about door normal (Y)",
        center_latch_turn.articulation_type == ArticulationType.REVOLUTE
        and center_latch_turn.axis == (0.0, 1.0, 0.0),
    )
    clim = center_latch_turn.motion_limits
    ctx.check(
        "center latch rotates 0..90 deg",
        clim is not None
        and clim.lower == 0.0
        and abs(clim.upper - LATCH_TURN) < 1e-6,
    )

    # Latch sits at the centre seam (x ≈ 0)
    cl_aabb = ctx.part_world_aabb(center_latch)
    ctx.check(
        "center latch is positioned at x ≈ 0 (centre seam)",
        cl_aabb is not None
        and abs(0.5 * (cl_aabb[0][0] + cl_aabb[1][0])) < 0.06,
        details=str(cl_aabb),
    )

    # Latch at mid-height of the doors
    ctx.check(
        "center latch is at door mid-height",
        cl_aabb is not None
        and abs(0.5 * (cl_aabb[0][2] + cl_aabb[1][2]) - DOOR_ZC) < 0.05,
        details=str(cl_aabb),
    )

    # Latch bar rotates from horizontal (X-wide) to vertical (Z-tall)
    bar_rest = ctx.part_element_world_aabb(center_latch, elem="latch_bar")
    with ctx.pose({center_latch_turn: LATCH_TURN}):
        bar_turned = ctx.part_element_world_aabb(center_latch, elem="latch_bar")
    ctx.check(
        "turning center latch rotates bar from horizontal to vertical",
        bar_rest is not None
        and bar_turned is not None
        and (bar_rest[1][0] - bar_rest[0][0]) > 0.06
        and (bar_turned[1][2] - bar_turned[0][2]) > 0.06,
        details=f"rest={bar_rest}, turned={bar_turned}",
    )

    # Latch plate is proud of the front face (not embedded)
    plate_aabb = ctx.part_element_world_aabb(center_latch, elem="latch_plate")
    ctx.check(
        "center latch plate sits proud of the cabinet front face",
        plate_aabb is not None and plate_aabb[1][1] > FRONT_Y,
        details=str(plate_aabb),
    )

    return ctx.report()


object_model = build_object_model()
