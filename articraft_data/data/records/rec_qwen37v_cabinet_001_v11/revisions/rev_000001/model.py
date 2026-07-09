from __future__ import annotations

"""Tall two-door industrial steel cabinet with raised plinth base and hinged top lid.

Variant of the vintage industrial steel locker cabinet family.

Overall envelope ~0.80 m wide x 0.50 m deep x ~1.80 m tall, brushed/tarnished
raw steel. A hollow thin-wall (~0.02 m) carcass sits on a raised plinth base
with caster blocks at the four corners. The front has two full-height hinged
doors with exposed barrel hinges, separated by a centre stile. The left door
hinges on its left edge and the right door on its right edge, so the pair
opens away from the cabinet centre. Each door is an independent revolute joint
about a vertical axis, 0..~110 deg outward, and carries a latch knob at
mid-height, a dark recessed ventilation slot near the bottom, and stamped vent
lines near the top. A flat steel lid sits on top and hinges upward on a rear
revolute joint (0..~85 deg).
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
CAB_W = 0.80  # overall carcass width  (X)
CAB_D = 0.50  # overall carcass depth  (Y)
CAB_TOP = 1.80  # carcass top height   (Z)
WALL_T = 0.02  # thin steel wall thickness

CASTER_H = 0.040  # caster block height
PLINTH_H = 0.100  # plinth base height
CARCASS_BOTTOM = CASTER_H + PLINTH_H  # 0.14

FRONT_Y = CAB_D / 2.0  # +0.25
BACK_Y = -CAB_D / 2.0  # -0.25

BOTTOM_RAIL_TOP = CARCASS_BOTTOM + 0.06  # 0.20
TOP_RAIL_BOT = CAB_TOP - 0.06  # 1.74

STILE_W = 0.03

# Door pockets between inner faces of side walls
POCKET_LEFT = -(CAB_W / 2.0 - WALL_T)  # -0.38
POCKET_RIGHT = CAB_W / 2.0 - WALL_T  # +0.38
POCKET_W = POCKET_RIGHT - POCKET_LEFT  # 0.76

DOOR_W = (POCKET_W - STILE_W) / 2.0 - 0.003  # ~0.362
DOOR_T = WALL_T
DOOR_Z0 = BOTTOM_RAIL_TOP + 0.002  # 0.202
DOOR_Z1 = TOP_RAIL_BOT - 0.002  # 1.738
DOOR_H = DOOR_Z1 - DOOR_Z0  # ~1.536
DOOR_ZC = 0.5 * (DOOR_Z0 + DOOR_Z1)  # ~0.970

HINGE_INSET = 0.0005

SLOT_LEN = 0.28
SLOT_W = 0.028
SLOT_ZC = -0.40

CAP_T = 0.022
CAP_OVERHANG = 0.015

LID_T = 0.020

DOOR_OPEN = math.radians(110.0)
LID_OPEN = math.radians(85.0)
KNOB_TURN = math.radians(90.0)

CASTER_SIZE = 0.050  # caster block footprint
PLINTH_INSET = 0.015  # plinth inset from cabinet walls


def _door_solid(sign: float, mesh_name: str):
    """Door leaf: flat panel with a rounded-end through slot near the bottom.
    sign=+1 -> panel extends along +X from hinge (left-hinged).
    sign=-1 -> panel extends along -X from hinge (right-hinged)."""
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


def _lid_solid(mesh_name: str):
    """Top lid panel: flat steel slab that fits within the cap overhang.
    Built in local frame: hinge edge at Y=0, panel extends along +Y."""
    lid_w = CAB_W - 0.004  # slight clearance inside cap
    lid_d = CAB_D - 0.004
    lid = (
        cq.Workplane("XY")
        .box(lid_w, lid_d, LID_T)
        .translate((0.0, lid_d / 2.0, LID_T / 2.0))
    )
    return mesh_from_cadquery(lid, mesh_name)


def _caster_solid(mesh_name: str):
    """Caster block: small mounting plate with a wheel cylinder below."""
    plate = (
        cq.Workplane("XY")
        .box(CASTER_SIZE, CASTER_SIZE, 0.015)
        .translate((0.0, 0.0, CASTER_H - 0.015 / 2.0))
    )
    wheel = (
        cq.Workplane("XZ")
        .circle(0.018)
        .extrude(0.022, both=True)
        .translate((0.0, 0.0, 0.018))
    )
    fork_l = (
        cq.Workplane("XY")
        .box(0.004, 0.022, 0.024)
        .translate((-0.014, 0.0, 0.024))
    )
    fork_r = (
        cq.Workplane("XY")
        .box(0.004, 0.022, 0.024)
        .translate((0.014, 0.0, 0.024))
    )
    caster = plate.union(wheel).union(fork_l).union(fork_r)
    return mesh_from_cadquery(caster, mesh_name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tall_two_door_steel_cabinet")

    steel_body = model.material("steel_body", rgba=(0.60, 0.61, 0.63, 1.0))
    steel_door_a = model.material("steel_door_a", rgba=(0.55, 0.56, 0.58, 1.0))
    steel_door_b = model.material("steel_door_b", rgba=(0.50, 0.51, 0.54, 1.0))
    steel_trim = model.material("steel_trim", rgba=(0.46, 0.47, 0.49, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.07, 0.07, 0.08, 1.0))
    steel_plinth = model.material("steel_plinth", rgba=(0.42, 0.43, 0.45, 1.0))
    steel_knob = model.material("steel_knob", rgba=(0.18, 0.18, 0.20, 1.0))
    steel_rivet = model.material("steel_rivet", rgba=(0.52, 0.53, 0.55, 1.0))
    steel_caster = model.material("steel_caster", rgba=(0.30, 0.31, 0.33, 1.0))
    steel_lid = model.material("steel_lid", rgba=(0.57, 0.58, 0.60, 1.0))
    steel_hinge = model.material("steel_hinge", rgba=(0.40, 0.41, 0.43, 1.0))

    # ------------------------------------------------------------------
    # Cabinet body: hollow carcass + plinth + casters + front frame + cap
    # ------------------------------------------------------------------
    body = model.part("cabinet_body")
    carcass_h = CAB_TOP - CARCASS_BOTTOM  # 1.66
    carcass_zc = CARCASS_BOTTOM + carcass_h / 2.0

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
    # Bottom panel
    body.visual(
        Box((CAB_W, CAB_D, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, CARCASS_BOTTOM + WALL_T / 2.0)),
        material=steel_body,
        name="bottom_panel",
    )
    # Top panel (under the lid)
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

    # Front frame: bottom rail, top rail, centre stile
    body.visual(
        Box((CAB_W - 2.0 * WALL_T, WALL_T, BOTTOM_RAIL_TOP - CARCASS_BOTTOM + 0.01)),
        origin=Origin(
            xyz=(0.0, FRONT_Y - WALL_T / 2.0, (CARCASS_BOTTOM + BOTTOM_RAIL_TOP) / 2.0)
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
    body.visual(
        Box((STILE_W, WALL_T, stile_h)),
        origin=Origin(xyz=(0.0, FRONT_Y - WALL_T / 2.0, DOOR_ZC)),
        material=steel_trim,
        name="centre_stile",
    )

    # Raised plinth base
    plinth_w = CAB_W - 2.0 * PLINTH_INSET
    plinth_d = CAB_D - 2.0 * PLINTH_INSET
    body.visual(
        Box((plinth_w, plinth_d, PLINTH_H)),
        origin=Origin(xyz=(0.0, 0.0, CASTER_H + PLINTH_H / 2.0)),
        material=steel_plinth,
        name="plinth_base",
    )
    # Plinth top lip (slight outward flare)
    body.visual(
        Box((CAB_W + 0.006, CAB_D + 0.006, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, CASTER_H + PLINTH_H - 0.006)),
        material=steel_plinth,
        name="plinth_lip",
    )

    # Caster blocks at four corners
    caster_mesh = _caster_solid("caster_block")
    caster_corners = [
        (CAB_W / 2.0 - CASTER_SIZE / 2.0 - 0.02, CAB_D / 2.0 - CASTER_SIZE / 2.0 - 0.02),
        (-CAB_W / 2.0 + CASTER_SIZE / 2.0 + 0.02, CAB_D / 2.0 - CASTER_SIZE / 2.0 - 0.02),
        (-CAB_W / 2.0 + CASTER_SIZE / 2.0 + 0.02, -CAB_D / 2.0 + CASTER_SIZE / 2.0 + 0.02),
        (CAB_W / 2.0 - CASTER_SIZE / 2.0 - 0.02, -CAB_D / 2.0 + CASTER_SIZE / 2.0 + 0.02),
    ]
    for i, (cx, cy) in enumerate(caster_corners):
        body.visual(
            caster_mesh,
            origin=Origin(xyz=(cx, cy, 0.0)),
            material=steel_caster,
            name=f"caster_{i}",
        )

    # Thin riveted top cap strip with slight overhang (sits below the lid)
    body.visual(
        Box((CAB_W + 2 * CAP_OVERHANG, CAB_D + 2 * CAP_OVERHANG, CAP_T)),
        origin=Origin(xyz=(0.0, 0.0, CAB_TOP - CAP_T / 2.0)),
        material=steel_trim,
        name="top_cap",
    )
    # Raised rivet dots along the front top rail
    n_riv = 9
    for i in range(n_riv):
        rx = -0.32 + i * (0.64 / (n_riv - 1))
        body.visual(
            Sphere(radius=0.0045),
            origin=Origin(xyz=(rx, FRONT_Y + 0.002, 1.77)),
            material=steel_rivet,
            name=f"rivet_{i}",
        )

    # Lid hinge barrels at rear top edge (fixed to body, contact the lid)
    for i, lx in enumerate((-0.25, 0.0, 0.25)):
        body.visual(
            Cylinder(radius=0.008, length=0.050),
            origin=Origin(xyz=(lx, BACK_Y + 0.005, CAB_TOP + LID_T / 2.0)),
            material=steel_hinge,
            name=f"lid_hinge_knuckle_{i}",
        )

    # ------------------------------------------------------------------
    # Two doors with barrel hinges
    # ------------------------------------------------------------------
    barrel_hinge = BarrelHingeGeometry(
        0.068,
        leaf_width_a=0.022,
        leaf_width_b=0.020,
        leaf_thickness=0.003,
        pin_diameter=0.004,
        knuckle_count=5,
        holes_a=HingeHolePattern(
            style="round", count=3, diameter=0.003, edge_margin=0.008
        ),
        holes_b=HingeHolePattern(
            style="round", count=3, diameter=0.003, edge_margin=0.008
        ),
    )
    barrel_mesh = mesh_from_geometry(barrel_hinge, "door_barrel_hinge")

    door_specs = [
        # (hinge world x, sign, leaf material)
        (POCKET_LEFT + HINGE_INSET, +1.0, steel_door_a),  # left door
        (POCKET_RIGHT - HINGE_INSET, -1.0, steel_door_b),  # right door
    ]
    doors = []
    for i, (hinge_x, sign, leaf_mat) in enumerate(door_specs):
        door = model.part(f"door_{i}")
        xc = sign * DOOR_W / 2.0  # door-local panel centre

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
                Box((0.14, 0.004, 0.006)),
                origin=Origin(xyz=(xc, -0.0012, dz)),
                material=steel_dark,
                name=f"vent_line_{j}",
            )

        # Visible barrel hinges (upper and lower) on the door side
        hinge_z_offsets = [
            DOOR_Z0 + 0.12 - DOOR_ZC,  # lower hinge
            DOOR_Z1 - 0.12 - DOOR_ZC,  # upper hinge
        ]
        for hi, hz in enumerate(hinge_z_offsets):
            door.visual(
                barrel_mesh,
                origin=Origin(xyz=(0.0, -0.001, hz)),
                material=steel_hinge,
                name=f"barrel_hinge_{hi}",
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

    # ------------------------------------------------------------------
    # Top lid hinged at the rear edge
    # ------------------------------------------------------------------
    lid = model.part("lid")
    lid.visual(
        _lid_solid("lid_panel"),
        material=steel_lid,
        name="panel",
    )
    # Small handle on the lid front edge (seated on the panel top)
    lid.visual(
        Box((0.08, 0.012, 0.015)),
        origin=Origin(xyz=(0.0, CAB_D - 0.025, LID_T + 0.0065)),
        material=steel_knob,
        name="lid_handle",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        # Hinge at rear top edge of carcass
        origin=Origin(xyz=(0.0, BACK_Y, CAB_TOP)),
        # +X axis: positive q rotates +Y toward +Z -> lid front edge lifts up
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=1.5, lower=0.0, upper=LID_OPEN
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
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("lid_hinge")

    # Intentional local laps: barrel hinge leaves embed into the frame edge
    for door, elem in zip(doors, ["side_wall_0", "side_wall_1"]):
        for bh in ("barrel_hinge_0", "barrel_hinge_1"):
            ctx.allow_overlap(
                door,
                body,
                elem_a=bh,
                elem_b=elem,
                reason="Barrel hinge leaf intentionally laps the fixed frame edge it pivots on.",
            )

    # Lid hinge knuckles are embedded in the lid panel rear edge (pivot capture)
    for ki in range(3):
        ctx.allow_overlap(
            body,
            lid,
            elem_a=f"lid_hinge_knuckle_{ki}",
            elem_b="panel",
            reason="Lid hinge knuckle is intentionally captured at the rear pivot edge of the lid.",
        )

    # --- Overall envelope and grounded on floor ------------------------
    aabb = ctx.part_world_aabb(body)
    ok = aabb is not None
    ctx.check("cabinet body has bounds", ok, details=str(aabb))
    if aabb is not None:
        (x0, y0, z0), (x1, y1, z1) = aabb
        ctx.check(
            "overall width ~0.8 m",
            0.78 <= (x1 - x0) <= 0.90,
            details=f"width={x1 - x0:.3f}",
        )
        ctx.check(
            "overall depth ~0.5 m",
            0.48 <= (y1 - y0) <= 0.58,
            details=f"depth={y1 - y0:.3f}",
        )
        ctx.check(
            "overall height ~1.8 m",
            1.78 <= z1 <= 1.88,
            details=f"top={z1:.3f}",
        )
        ctx.check(
            "casters rest on the floor", abs(z0) <= 0.002, details=f"zmin={z0:.5f}"
        )

    # --- Plinth base exists between casters and carcass ----------------
    plinth_aabb = ctx.part_element_world_aabb(body, elem="plinth_base")
    ctx.check(
        "plinth base sits above casters and below carcass",
        plinth_aabb is not None
        and plinth_aabb[0][2] > CASTER_H - 0.005
        and plinth_aabb[1][2] < CARCASS_BOTTOM + 0.005,
        details=str(plinth_aabb),
    )

    # --- Caster blocks present at the base -----------------------------
    for ci in range(4):
        ca = ctx.part_element_world_aabb(body, elem=f"caster_{ci}")
        ctx.check(
            f"caster_{ci} at floor level",
            ca is not None and ca[0][2] < 0.005,
            details=str(ca),
        )

    # --- Doors: hinge type, axis, range, seating ----------------------
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
        # Closed leaf sits flush with the front face
        daabb = ctx.part_element_world_aabb(door, elem="leaf")
        ctx.check(
            f"door_{i} closed leaf is flush with front face",
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
            name=f"door_{i} stays inside cabinet width when closed",
        )

    # Left door hinges on left edge, right door on right edge
    ctx.check(
        "left door hinges on left, right on right",
        hinges[0].origin.xyz[0] < -0.36
        and hinges[1].origin.xyz[0] > 0.36,
        details=str([h.origin.xyz[0] for h in hinges]),
    )

    # Opening pose: both leaves swing outward
    with ctx.pose({hinges[0]: DOOR_OPEN, hinges[1]: DOOR_OPEN}):
        open0 = ctx.part_world_aabb(doors[0])
        open1 = ctx.part_world_aabb(doors[1])
    ctx.check(
        "open leaves swing outward past the front face",
        open0 is not None
        and open1 is not None
        and open0[1][1] > FRONT_Y + 0.20
        and open1[1][1] > FRONT_Y + 0.20,
        details=f"open0={open0}, open1={open1}",
    )

    # --- Visible barrel hinges on doors --------------------------------
    for i, door in enumerate(doors):
        for bh_name in ("barrel_hinge_0", "barrel_hinge_1"):
            bha = ctx.part_element_world_aabb(door, elem=bh_name)
            ctx.check(
                f"door_{i} {bh_name} visible on hinge edge",
                bha is not None and (bha[1][2] - bha[0][2]) > 0.04,
                details=str(bha),
            )
            # Proof: barrel hinge contacts the frame edge at the pivot
            frame_elem = "side_wall_0" if i == 0 else "side_wall_1"
            ctx.expect_contact(
                door,
                body,
                elem_a=bh_name,
                elem_b=frame_elem,
                contact_tol=0.005,
                name=f"door_{i} {bh_name} contacts frame edge at pivot",
            )

    # --- Lid: hinge type, axis, upward opening -------------------------
    ctx.check(
        "lid hinge is revolute",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE,
    )
    lid_ax = lid_hinge.axis
    ctx.check(
        "lid hinge axis is horizontal along X",
        abs(abs(lid_ax[0]) - 1.0) < 1e-9
        and abs(lid_ax[1]) < 1e-9
        and abs(lid_ax[2]) < 1e-9,
        details=str(lid_ax),
    )
    lid_lim = lid_hinge.motion_limits
    ctx.check(
        "lid hinge range 0..~85 deg",
        lid_lim is not None
        and lid_lim.lower == 0.0
        and abs(lid_lim.upper - math.radians(85.0)) < 1e-6,
    )

    # Lid hinge origin is at the rear of the cabinet
    ctx.check(
        "lid hinge is at the rear edge",
        lid_hinge.origin.xyz[1] < BACK_Y + 0.01,
        details=str(lid_hinge.origin.xyz),
    )

    # Closed lid sits on top of the carcass
    lid_closed = ctx.part_element_world_aabb(lid, elem="panel")
    ctx.check(
        "closed lid sits at carcass top level",
        lid_closed is not None and abs(lid_closed[0][2] - CAB_TOP) < 0.01,
        details=str(lid_closed),
    )

    # Opening pose: lid front edge lifts upward
    lid_closed_center = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: LID_OPEN}):
        lid_open_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "open lid lifts upward",
        lid_closed_center is not None
        and lid_open_aabb is not None
        and lid_open_aabb[1][2] > lid_closed_center[1][2] + 0.15,
        details=f"closed={lid_closed_center}, open={lid_open_aabb}",
    )

    # Proof: lid hinge knuckles stay near the rear pivot edge
    for ki in range(3):
        ctx.expect_overlap(
            body,
            lid,
            axes="y",
            elem_a=f"lid_hinge_knuckle_{ki}",
            elem_b="panel",
            min_overlap=0.001,
            name=f"lid_hinge_knuckle_{ki} overlaps lid panel at rear pivot",
        )

    # --- Latch knobs ---------------------------------------------------
    for i, (knob, latch, door) in enumerate(zip(knobs, latches, doors)):
        ctx.check(
            f"latch_{i} is a quarter-turn revolute about door normal",
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
            name=f"latch_knob_{i} backplate seats on leaf face",
        )

    # Rivet dots present along the top rail
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
