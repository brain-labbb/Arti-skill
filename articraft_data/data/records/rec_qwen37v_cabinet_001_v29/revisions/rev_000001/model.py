from __future__ import annotations

"""Vintage industrial steel locker cabinet – variant 29.

Four full-height hinged doors with visible barrel hinges, a lift-up top lid
over shallow storage, and caster block feet at the base.

Overall envelope ~1.6 m wide x 0.5 m deep x ~1.82 m tall (lid closed),
brushed/tarnished raw steel. A hollow thin-wall (~0.02 m) carcass sits on
four caster block feet and carries a shallow storage tray at the top,
covered by a hinged lid. The front is split into four flat doors separated
by narrow stiles. The two left doors hinge on their left edges, the two
right doors hinge on their right edges (visible barrel hinge knuckles on
the door side), so the pairs swing open away from the cabinet centre.
Each door is an independent revolute joint about a vertical axis,
0..~110 deg outward. The lid is a separate revolute joint at the rear
top edge, 0..~75 deg upward.
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
CAB_TOP = 1.80  # carcass top height   (Z) – top of side/back walls
LEG_H = 0.14  # caster block foot height; carcass starts here
WALL_T = 0.02  # thin steel wall thickness

FRONT_Y = CAB_D / 2.0  # +0.25, front face plane
BACK_Y = -CAB_D / 2.0

# Shallow storage tray at the top of the carcass.
RIM_H = 0.08  # depth of the shallow storage well
STORAGE_FLOOR = CAB_TOP - RIM_H  # 1.72 – top surface of storage floor

BOTTOM_RAIL_TOP = LEG_H + 0.06  # 0.20
TOP_RAIL_BOT = STORAGE_FLOOR - 0.02  # 1.70 – top rail extends to storage floor

STILE_W = 0.03
# Door pockets between the inner faces of the side walls (x = -0.78 .. 0.78)
POCKET_EDGES = [-0.78, -0.4125, -0.3825, -0.015, 0.015, 0.3825, 0.4125, 0.78]

DOOR_W = 0.364  # leaf width (pocket is 0.3675 wide -> swing clearance)
DOOR_T = WALL_T
DOOR_Z0 = BOTTOM_RAIL_TOP + 0.002  # 0.202
DOOR_Z1 = TOP_RAIL_BOT - 0.002  # 1.698
DOOR_H = DOOR_Z1 - DOOR_Z0  # ~1.496
DOOR_ZC = 0.5 * (DOOR_Z0 + DOOR_Z1)
HINGE_INSET = 0.0005  # hinge line inset from the pocket edge

SLOT_LEN = 0.36  # dark rounded-end vent slot near the bottom
SLOT_W = 0.030
SLOT_ZC = -0.40  # in door-local z (door centre = 0)

BARREL_R = 0.010
KNUCKLE_R = 0.013
BARREL_LEN = DOOR_H - 0.03

CAP_T = 0.018  # rim strip at the top of the storage well
CAP_OVERHANG = 0.015

LID_T = 0.022  # lid panel thickness
LID_INSET = 0.005  # lid fits inside the rim with a small gap
LID_OPEN = math.radians(75.0)  # lid opening angle

DOOR_OPEN = math.radians(110.0)
KNOB_TURN = math.radians(90.0)

# Caster block foot dimensions.
FOOT_W = 0.075  # foot block width (X and Y)
FOOT_H = LEG_H - 0.012  # main block height (minus pad)
FOOT_PAD_T = 0.012  # rubber pad thickness at bottom
FOOT_FLANGE_T = 0.010  # mounting flange at top
FOOT_FLANGE_W = 0.095  # flange width


def _door_solid(sign: float, mesh_name: str):
    """Door leaf as one CadQuery solid: flat panel with a rounded-end
    through slot near the bottom. ``sign``=+1 -> panel extends along +X from
    the hinge line; ``sign``=-1 -> along -X (right-hinged)."""
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
    """Door-side barrel hinge knuckle column along local Z axis."""
    barrel = cq.Workplane("XY").circle(BARREL_R).extrude(BARREL_LEN / 2.0, both=True)
    ring_h = 0.060
    for zc in (-0.58, -0.29, 0.0, 0.29, 0.58):
        ring = (
            cq.Workplane("XY")
            .circle(KNUCKLE_R)
            .extrude(ring_h / 2.0, both=True)
            .translate((0.0, 0.0, zc))
        )
        barrel = barrel.union(ring)
    return mesh_from_cadquery(barrel, mesh_name)


def _caster_foot_solid(mesh_name: str):
    """Caster block foot: rectangular block with mounting flange and floor pad."""
    block = (
        cq.Workplane("XY")
        .box(FOOT_W, FOOT_W, FOOT_H)
        .translate((0.0, 0.0, FOOT_PAD_T + FOOT_H / 2.0))
    )
    # Mounting flange at top.
    flange = (
        cq.Workplane("XY")
        .box(FOOT_FLANGE_W, FOOT_FLANGE_W, FOOT_FLANGE_T)
        .translate((0.0, 0.0, FOOT_PAD_T + FOOT_H + FOOT_FLANGE_T / 2.0))
    )
    block = block.union(flange)
    # Floor pad (slightly wider, dark rubber).
    pad = (
        cq.Workplane("XY")
        .box(FOOT_W + 0.010, FOOT_W + 0.010, FOOT_PAD_T)
        .translate((0.0, 0.0, FOOT_PAD_T / 2.0))
    )
    foot = block.union(pad)
    return mesh_from_cadquery(foot, mesh_name)


def _lid_solid(mesh_name: str):
    """Lid panel: flat steel plate with a small front lip for grip."""
    lid_w = CAB_W - 2.0 * WALL_T - 2.0 * LID_INSET
    lid_d = CAB_D - 2.0 * WALL_T - 2.0 * LID_INSET
    panel = (
        cq.Workplane("XY")
        .box(lid_w, lid_d, LID_T)
        .translate((0.0, lid_d / 2.0, LID_T / 2.0))
    )
    # Small front lip/handle ridge.
    lip = (
        cq.Workplane("XY")
        .box(0.12, 0.012, 0.008)
        .translate((0.0, lid_d - 0.006, LID_T + 0.004))
    )
    lid = panel.union(lip)
    return mesh_from_cadquery(lid, mesh_name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_steel_locker_cabinet_v29")

    steel_body = model.material("steel_body", rgba=(0.60, 0.61, 0.63, 1.0))
    steel_door_a = model.material("steel_door_a", rgba=(0.55, 0.56, 0.58, 1.0))
    steel_door_b = model.material("steel_door_b", rgba=(0.50, 0.51, 0.54, 1.0))
    steel_trim = model.material("steel_trim", rgba=(0.46, 0.47, 0.49, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.07, 0.07, 0.08, 1.0))
    steel_leg = model.material("steel_leg", rgba=(0.38, 0.39, 0.41, 1.0))
    steel_knob = model.material("steel_knob", rgba=(0.18, 0.18, 0.20, 1.0))
    steel_rivet = model.material("steel_rivet", rgba=(0.52, 0.53, 0.55, 1.0))
    steel_lid = model.material("steel_lid", rgba=(0.57, 0.58, 0.60, 1.0))
    rubber_pad = model.material("rubber_pad", rgba=(0.12, 0.12, 0.13, 1.0))

    # ------------------------------------------------------------------
    # Cabinet body: hollow carcass + caster feet + front frame + rim
    # ------------------------------------------------------------------
    body = model.part("cabinet_body")
    carcass_h = CAB_TOP - LEG_H  # 1.66
    carcass_zc = LEG_H + carcass_h / 2.0

    # Side walls (full depth, full carcass height).
    for sx, vname in ((-1.0, "side_wall_0"), (1.0, "side_wall_1")):
        body.visual(
            Box((WALL_T, CAB_D, carcass_h)),
            origin=Origin(xyz=(sx * (CAB_W / 2.0 - WALL_T / 2.0), 0.0, carcass_zc)),
            material=steel_body,
            name=vname,
        )
    # Back wall (overlaps side walls slightly so the shell is one body).
    body.visual(
        Box((CAB_W - WALL_T, WALL_T, carcass_h - 0.02)),
        origin=Origin(xyz=(0.0, BACK_Y + WALL_T / 2.0, carcass_zc)),
        material=steel_body,
        name="back_wall",
    )
    # Bottom panel.
    body.visual(
        Box((CAB_W, CAB_D, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, LEG_H + WALL_T / 2.0)),
        material=steel_body,
        name="bottom_panel",
    )
    # Storage floor panel (top of the main compartment, floor of shallow tray).
    body.visual(
        Box((CAB_W - 2.0 * WALL_T + 0.01, CAB_D - 2.0 * WALL_T + 0.01, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, STORAGE_FLOOR - WALL_T / 2.0)),
        material=steel_body,
        name="storage_floor",
    )
    # Interior shelf (thin, embedded into side and back walls).
    body.visual(
        Box((CAB_W - 2.0 * WALL_T + 0.01, 0.43, 0.015)),
        origin=Origin(xyz=(0.0, -0.02, 0.95)),
        material=steel_body,
        name="interior_shelf",
    )

    # Front frame: bottom rail, top rail (extends up to storage floor), stiles.
    body.visual(
        Box((CAB_W - 2.0 * WALL_T, WALL_T, BOTTOM_RAIL_TOP - LEG_H + 0.01)),
        origin=Origin(
            xyz=(0.0, FRONT_Y - WALL_T / 2.0, (LEG_H + BOTTOM_RAIL_TOP) / 2.0)
        ),
        material=steel_body,
        name="front_bottom_rail",
    )
    front_top_rail_h = CAB_TOP - TOP_RAIL_BOT + 0.01  # covers from ~1.69 to 1.81
    body.visual(
        Box((CAB_W - 2.0 * WALL_T, WALL_T, front_top_rail_h)),
        origin=Origin(
            xyz=(0.0, FRONT_Y - WALL_T / 2.0, (TOP_RAIL_BOT + CAB_TOP) / 2.0 + 0.005)
        ),
        material=steel_body,
        name="front_top_rail",
    )
    stile_h = TOP_RAIL_BOT - BOTTOM_RAIL_TOP + 0.01
    for i, xc in enumerate((-0.3975, 0.0, 0.3975)):
        body.visual(
            Box((STILE_W, WALL_T, stile_h)),
            origin=Origin(xyz=(xc, FRONT_Y - WALL_T / 2.0, (DOOR_Z0 + DOOR_Z1) / 2.0)),
            material=steel_trim,
            name=f"front_stile_{i}",
        )

    # Shallow storage rim: front wall (the top rail covers this), plus
    # the side walls and back wall already extend to CAB_TOP. Add a thin
    # rim cap strip around the top edge.
    body.visual(
        Box((CAB_W + 2 * CAP_OVERHANG, CAB_D + 2 * CAP_OVERHANG, CAP_T)),
        origin=Origin(xyz=(0.0, 0.0, CAB_TOP + CAP_T / 2.0)),
        material=steel_trim,
        name="rim_cap",
    )

    # Raised rivet dots along the front rim.
    n_riv = 13
    for i in range(n_riv):
        rx = -0.72 + i * (1.44 / (n_riv - 1))
        body.visual(
            Sphere(radius=0.0045),
            origin=Origin(xyz=(rx, FRONT_Y + 0.002, CAB_TOP + CAP_T + 0.002)),
            material=steel_rivet,
            name=f"rivet_{i}",
        )

    # Caster block feet at the four corners.
    foot_mesh = _caster_foot_solid("caster_foot")
    foot_corners = [
        (CAB_W / 2.0 - 0.08, FRONT_Y - 0.08),
        (-CAB_W / 2.0 + 0.08, FRONT_Y - 0.08),
        (-CAB_W / 2.0 + 0.08, BACK_Y + 0.08),
        (CAB_W / 2.0 - 0.08, BACK_Y + 0.08),
    ]
    for i, (fx, fy) in enumerate(foot_corners):
        body.visual(
            foot_mesh,
            origin=Origin(xyz=(fx, fy, 0.0)),
            material=steel_leg,
            name=f"caster_foot_{i}",
        )
        # Dark rubber pad visual (separate material on the pad region).
        body.visual(
            Box((FOOT_W + 0.010, FOOT_W + 0.010, FOOT_PAD_T)),
            origin=Origin(xyz=(fx, fy, FOOT_PAD_T / 2.0)),
            material=rubber_pad,
            name=f"foot_pad_{i}",
        )

    # ------------------------------------------------------------------
    # Four doors with visible barrel hinges.
    # ------------------------------------------------------------------
    door_specs = [
        # (hinge world x, sign, leaf material)
        (POCKET_EDGES[0] + HINGE_INSET, +1.0, steel_door_a),  # far left
        (POCKET_EDGES[2] + HINGE_INSET, +1.0, steel_door_b),  # centre-left
        (POCKET_EDGES[5] - HINGE_INSET, -1.0, steel_door_b),  # centre-right
        (POCKET_EDGES[7] - HINGE_INSET, -1.0, steel_door_a),  # far right
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
        # Dark backing plate behind the through slot -> recessed dark slot.
        door.visual(
            Box((SLOT_W + 0.016, 0.005, SLOT_LEN + 0.036)),
            origin=Origin(xyz=(xc, -DOOR_T - 0.001, SLOT_ZC)),
            material=steel_dark,
            name="vent_backing",
        )
        # Stamped vent lines near the top (slightly proud thin dark strips).
        for j, dz in enumerate((0.58, 0.60, 0.62)):
            door.visual(
                Box((0.16, 0.004, 0.006)),
                origin=Origin(xyz=(xc, -0.0012, dz)),
                material=steel_dark,
                name=f"vent_line_{j}",
            )
        # Door-side barrel hinge knuckle column (visible hinge barrel).
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

        # Quarter-turn latch knob at mid-height near the free edge.
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
    # Lift-up top lid over the shallow storage tray.
    # ------------------------------------------------------------------
    lid = model.part("top_lid")
    lid.visual(
        _lid_solid("lid_panel"),
        material=steel_lid,
        name="lid_panel",
    )
    # Lid hinge at the back top edge. Part frame at the hinge line;
    # the lid panel extends along +Y (toward the front) from there.
    # Axis (1, 0, 0): positive rotation lifts +Y side upward (+Z).
    lid_hinge_y = BACK_Y + WALL_T + LID_INSET  # just inside the back wall
    lid_hinge_z = CAB_TOP + CAP_T  # sits on top of the rim cap

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, lid_hinge_y, lid_hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0, velocity=2.0, lower=0.0, upper=LID_OPEN
        ),
    )

    # Lid hinge barrel (visible barrel at the back edge of the lid).
    lid_barrel_len = CAB_W - 0.20
    lid.visual(
        Cylinder(radius=0.012, length=lid_barrel_len),
        origin=Origin(
            xyz=(0.0, 0.0, LID_T / 2.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel_trim,
        name="lid_hinge_barrel",
    )

    # Fixed lid hinge barrel on the body (interleaves with the lid barrel).
    body.visual(
        Cylinder(radius=0.010, length=lid_barrel_len + 0.04),
        origin=Origin(
            xyz=(0.0, lid_hinge_y, lid_hinge_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel_trim,
        name="lid_hinge_body_barrel",
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("cabinet_body")
    doors = [object_model.get_part(f"door_{i}") for i in range(4)]
    hinges = [object_model.get_articulation(f"door_{i}_hinge") for i in range(4)]
    knobs = [object_model.get_part(f"latch_knob_{i}") for i in range(4)]
    latches = [object_model.get_articulation(f"latch_{i}") for i in range(4)]
    lid = object_model.get_part("top_lid")
    lid_hinge = object_model.get_articulation("lid_hinge")

    # Intentional local laps: each door's hinge knuckle column laps the
    # fixed frame edge it pivots on (captured barrel hinge knuckles).
    frame_elems = ["side_wall_0", "front_stile_0", "front_stile_2", "side_wall_1"]
    for door, elem in zip(doors, frame_elems):
        ctx.allow_overlap(
            door,
            body,
            elem_a="hinge_barrel",
            elem_b=elem,
            reason="Door-side barrel hinge knuckle intentionally laps the fixed frame edge it pivots on.",
        )

    # Lid hinge barrel on the body intentionally contacts the lid barrel
    # and the lid panel edge at the pivot line.
    ctx.allow_overlap(
        lid,
        body,
        elem_a="lid_hinge_barrel",
        elem_b="lid_hinge_body_barrel",
        reason="Lid barrel and body barrel interleave at the hinge pivot line.",
    )
    ctx.allow_overlap(
        lid,
        body,
        elem_a="lid_panel",
        elem_b="lid_hinge_body_barrel",
        reason="Lid panel edge embeds slightly into the fixed body barrel at the hinge pivot.",
    )

    # --- Overall envelope, true scale, grounded on the floor ----------------
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
        ctx.check("feet rest on the floor", abs(z0) <= 1e-6, details=f"zmin={z0:.5f}")

    # --- Caster block feet --------------------------------------------------
    for i in range(4):
        foot_aabb = ctx.part_element_world_aabb(body, elem=f"caster_foot_{i}")
        ctx.check(
            f"caster_foot_{i} exists and touches the floor",
            foot_aabb is not None and foot_aabb[0][2] < 0.001,
            details=str(foot_aabb),
        )
        pad_aabb = ctx.part_element_world_aabb(body, elem=f"foot_pad_{i}")
        ctx.check(
            f"foot_pad_{i} is at the base of the foot",
            pad_aabb is not None and pad_aabb[0][2] < 0.001 and pad_aabb[1][2] < 0.020,
            details=str(pad_aabb),
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
        # Closed leaf sits flush in the front frame plane.
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

    # --- Visible hinge barrels on doors ------------------------------------
    for i, door in enumerate(doors):
        barrel_aabb = ctx.part_element_world_aabb(door, elem="hinge_barrel")
        ctx.check(
            f"door_{i} has a visible hinge barrel",
            barrel_aabb is not None
            and (barrel_aabb[1][0] - barrel_aabb[0][0]) > 0.015
            and (barrel_aabb[1][2] - barrel_aabb[0][2]) > 0.80,
            details=f"barrel extents={barrel_aabb}",
        )
    # Hinge sides: outer doors hinge at the cabinet's outer edges.
    ctx.check(
        "left pair hinges on left edges, right pair on right edges",
        hinges[0].origin.xyz[0] < -0.77
        and -0.39 < hinges[1].origin.xyz[0] < -0.37
        and 0.37 < hinges[2].origin.xyz[0] < 0.39
        and hinges[3].origin.xyz[0] > 0.77,
        details=str([h.origin.xyz[0] for h in hinges]),
    )

    # Opening pose: leaves swing outward, pairs away from centre.
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

    # --- Latch knobs ---------------------------------------------------------
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

    # --- Lift-up top lid ----------------------------------------------------
    ctx.check(
        "lid hinge is revolute",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE,
    )
    lid_ax = lid_hinge.axis
    ctx.check(
        "lid hinge axis is horizontal (along X)",
        abs(abs(lid_ax[0]) - 1.0) < 1e-9
        and abs(lid_ax[1]) < 1e-9
        and abs(lid_ax[2]) < 1e-9,
        details=str(lid_ax),
    )
    lid_lim = lid_hinge.motion_limits
    ctx.check(
        "lid opens 0..~75 deg",
        lid_lim is not None
        and lid_lim.lower == 0.0
        and abs(lid_lim.upper - math.radians(75.0)) < 1e-6,
    )

    # Closed lid sits on top of the cabinet, near the rim cap height.
    lid_closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    ctx.check(
        "closed lid sits at the top of the cabinet",
        lid_closed_aabb is not None
        and lid_closed_aabb[0][2] > CAB_TOP - 0.01
        and lid_closed_aabb[0][2] < CAB_TOP + 0.04,
        details=str(lid_closed_aabb),
    )

    # Open lid: front edge lifts upward.
    lid_closed_center = None
    if lid_closed_aabb is not None:
        lid_closed_center = [
            0.5 * (lid_closed_aabb[0][i] + lid_closed_aabb[1][i]) for i in range(3)
        ]
    with ctx.pose({lid_hinge: LID_OPEN}):
        lid_open_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    ctx.check(
        "open lid lifts the front edge well above the cabinet top",
        lid_open_aabb is not None and lid_open_aabb[1][2] > CAB_TOP + 0.20,
        details=f"open_top={lid_open_aabb[1][2] if lid_open_aabb else None}",
    )

    # Lid hinge barrel is visible.
    lid_barrel_aabb = ctx.part_element_world_aabb(lid, elem="lid_hinge_barrel")
    ctx.check(
        "lid has a visible hinge barrel",
        lid_barrel_aabb is not None
        and (lid_barrel_aabb[1][0] - lid_barrel_aabb[0][0]) > 1.0,
        details=str(lid_barrel_aabb),
    )

    # Riveted rim detail present.
    rivet_aabb = ctx.part_element_world_aabb(body, elem="rivet_0")
    ctx.check(
        "rivet dots stand proud of the rim face",
        rivet_aabb is not None
        and rivet_aabb[1][1] > FRONT_Y + 0.003
        and rivet_aabb[0][2] > CAB_TOP,
        details=str(rivet_aabb),
    )

    return ctx.report()


object_model = build_object_model()
