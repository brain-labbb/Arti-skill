from __future__ import annotations

"""Vintage industrial steel locker cabinet variant 19.

A storage cabinet with:
- Two hinged doors on the left (visible barrel hinges, separate pull handles)
- A tambour sliding front on the right (prismatic joint, slides sideways)
- A lift-up top lid over shallow storage (revolute hinge at back edge)

Overall envelope ~1.6 m wide x 0.5 m deep x ~1.8 m tall, brushed/tarnished raw
steel with a mottled gray finish. Hollow thin-wall carcass on four splayed legs.
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
# Global dimensions (meters). Cabinet centred on X, front face at +Y.
# ---------------------------------------------------------------------------
CAB_W = 1.60
CAB_D = 0.50
CAB_TOP = 1.80
LEG_H = 0.15
WALL_T = 0.02

FRONT_Y = CAB_D / 2.0  # +0.25
BACK_Y = -CAB_D / 2.0  # -0.25

# Horizontal divider creates shallow top compartment for the lid
DIVIDER_Z = 1.52

BOTTOM_RAIL_TOP = LEG_H + 0.06  # 0.21

STILE_W = 0.03

# Inner wall boundaries
INNER_LEFT = -(CAB_W / 2.0 - WALL_T)   # -0.78
INNER_RIGHT = CAB_W / 2.0 - WALL_T     # +0.78

# Two doors on the left half
# Centre stile between the two left doors at x ~ -0.39
LEFT_DOOR_CENTRE_STILE_X = -0.39
DOOR_W = 0.35
DOOR_T = WALL_T
DOOR_Z0 = BOTTOM_RAIL_TOP + 0.002  # 0.212
DOOR_Z1 = DIVIDER_Z - 0.025         # 1.495
DOOR_H = DOOR_Z1 - DOOR_Z0          # ~1.283
DOOR_ZC = 0.5 * (DOOR_Z0 + DOOR_Z1)

# Tambour on the right half
TAMBOUR_W = 0.73
TAMBOUR_H = DOOR_H
TAMBOUR_T = 0.014
# Centre of the right pocket: from centre_stile right edge (0.015) to inner right wall (0.78)
TAMBOUR_X_CENTER = 0.3975
TAMBOUR_SLIDE = TAMBOUR_W + 0.04

# Lid
LID_W = CAB_W - 2 * WALL_T
LID_D = CAB_D - 2 * WALL_T
LID_T = WALL_T

# Vent slot on doors
SLOT_LEN = 0.26
SLOT_W = 0.025
SLOT_ZC = -0.35

# Pull handle
HANDLE_W = 0.10
HANDLE_H = 0.018
HANDLE_D = 0.025

# Joint limits
DOOR_OPEN = math.radians(110.0)
LID_OPEN = math.radians(85.0)


def _door_solid(sign: float, mesh_name: str):
    """Door leaf: flat panel with rounded-end through slot near the bottom."""
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
    """Tambour front panel with horizontal slat grooves."""
    panel = (
        cq.Workplane("XY")
        .box(TAMBOUR_W, TAMBOUR_T, TAMBOUR_H)
    )
    slat_spacing = 0.045
    n_slats = int(TAMBOUR_H / slat_spacing)
    for i in range(n_slats):
        z_off = -TAMBOUR_H / 2.0 + slat_spacing * (i + 0.5)
        groove = (
            cq.Workplane("XY")
            .box(TAMBOUR_W - 0.02, 0.003, 0.004)
            .translate((0.0, TAMBOUR_T / 2.0 + 0.001, z_off))
        )
        panel = panel.union(groove)
    return mesh_from_cadquery(panel, mesh_name)


def _pull_handle_mesh(mesh_name: str):
    """D-shaped pull handle: bar with two mounting posts."""
    bar = (
        cq.Workplane("XY")
        .box(HANDLE_W, HANDLE_H, HANDLE_D)
        .translate((0.0, 0.0, 0.0))
    )
    for dx in (-HANDLE_W / 2.0 + 0.012, HANDLE_W / 2.0 - 0.012):
        post = (
            cq.Workplane("XY")
            .circle(0.005)
            .extrude(HANDLE_D)
            .translate((dx, 0.0, -HANDLE_D))
        )
        bar = bar.union(post)
    return mesh_from_cadquery(bar, mesh_name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_steel_cabinet_v19")

    steel_body = model.material("steel_body", rgba=(0.60, 0.61, 0.63, 1.0))
    steel_door = model.material("steel_door", rgba=(0.55, 0.56, 0.58, 1.0))
    steel_trim = model.material("steel_trim", rgba=(0.46, 0.47, 0.49, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.07, 0.07, 0.08, 1.0))
    steel_leg = model.material("steel_leg", rgba=(0.38, 0.39, 0.41, 1.0))
    steel_rivet = model.material("steel_rivet", rgba=(0.52, 0.53, 0.55, 1.0))
    steel_tambour = model.material("steel_tambour", rgba=(0.52, 0.53, 0.56, 1.0))
    steel_handle = model.material("steel_handle", rgba=(0.30, 0.31, 0.33, 1.0))
    steel_lid = model.material("steel_lid", rgba=(0.58, 0.59, 0.61, 1.0))
    steel_hinge = model.material("steel_hinge", rgba=(0.42, 0.43, 0.45, 1.0))

    # ==================================================================
    # Cabinet body
    # ==================================================================
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
    # Bottom panel
    body.visual(
        Box((CAB_W, CAB_D, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, LEG_H + WALL_T / 2.0)),
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
    # Horizontal divider
    body.visual(
        Box((CAB_W - 2 * WALL_T + 0.01, CAB_D - 2 * WALL_T + 0.01, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, DIVIDER_Z)),
        material=steel_body,
        name="divider_panel",
    )
    # Front bottom rail
    body.visual(
        Box((CAB_W - 2 * WALL_T, WALL_T, BOTTOM_RAIL_TOP - LEG_H + 0.01)),
        origin=Origin(
            xyz=(0.0, FRONT_Y - WALL_T / 2.0, (LEG_H + BOTTOM_RAIL_TOP) / 2.0)
        ),
        material=steel_body,
        name="front_bottom_rail",
    )
    # Front top rail (above door pocket, below divider)
    rail_top_z = DIVIDER_Z
    rail_bot_z = DOOR_Z1 + 0.005  # small gap above doors
    rail_h = rail_top_z - rail_bot_z
    body.visual(
        Box((CAB_W - 2 * WALL_T, WALL_T, rail_h)),
        origin=Origin(xyz=(0.0, FRONT_Y - WALL_T / 2.0, (rail_bot_z + rail_top_z) / 2.0)),
        material=steel_body,
        name="front_top_rail",
    )
    # Centre stile (between left doors and right tambour)
    body.visual(
        Box((STILE_W, WALL_T, DOOR_H + 0.02)),
        origin=Origin(xyz=(0.0, FRONT_Y - WALL_T / 2.0, DOOR_ZC)),
        material=steel_trim,
        name="centre_stile",
    )
    # Centre stile between the two left doors
    body.visual(
        Box((STILE_W, WALL_T, DOOR_H + 0.02)),
        origin=Origin(xyz=(LEFT_DOOR_CENTRE_STILE_X, FRONT_Y - WALL_T / 2.0, DOOR_ZC)),
        material=steel_trim,
        name="door_centre_stile",
    )
    # Tambour guide rails
    body.visual(
        Box((TAMBOUR_W + 0.04, 0.018, 0.014)),
        origin=Origin(xyz=(TAMBOUR_X_CENTER, FRONT_Y - 0.005, DOOR_Z1 + 0.012)),
        material=steel_trim,
        name="tambour_top_rail",
    )
    body.visual(
        Box((TAMBOUR_W + 0.04, 0.018, 0.014)),
        origin=Origin(xyz=(TAMBOUR_X_CENTER, FRONT_Y - 0.005, DOOR_Z0 - 0.012)),
        material=steel_trim,
        name="tambour_bottom_rail",
    )

    # Top cap strip
    cap_overhang = 0.02
    cap_t = 0.022
    body.visual(
        Box((CAB_W + 2 * cap_overhang, CAB_D + 2 * cap_overhang, cap_t)),
        origin=Origin(xyz=(0.0, 0.0, CAB_TOP - 0.001 + cap_t / 2.0)),
        material=steel_trim,
        name="top_cap",
    )
    # Rivet dots
    n_riv = 13
    for i in range(n_riv):
        rx = -0.72 + i * (1.44 / (n_riv - 1))
        body.visual(
            Sphere(radius=0.0045),
            origin=Origin(xyz=(rx, FRONT_Y + 0.002, CAB_TOP + 0.005)),
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

    # ==================================================================
    # Visible barrel hinges
    # ==================================================================
    barrel_hinge_geom = BarrelHingeGeometry(
        0.120,
        leaf_width_a=0.022,
        leaf_width_b=0.018,
        leaf_thickness=0.003,
        pin_diameter=0.004,
        knuckle_outer_diameter=0.012,
        knuckle_count=5,
        holes_a=HingeHolePattern(style="round", count=3, diameter=0.003, edge_margin=0.008),
        holes_b=HingeHolePattern(style="round", count=2, diameter=0.003, edge_margin=0.010),
    )
    barrel_hinge_mesh = mesh_from_geometry(barrel_hinge_geom, "barrel_hinge")

    # ==================================================================
    # Two doors on the left half
    # ==================================================================
    door_hinge_positions = [
        INNER_LEFT + 0.002,  # door 0: far left hinge
        LEFT_DOOR_CENTRE_STILE_X + STILE_W / 2.0 + 0.002,  # door 1: centre stile hinge
    ]

    doors = []
    handle_mesh = _pull_handle_mesh("pull_handle")

    for i in range(2):
        hinge_x = door_hinge_positions[i]
        sign = +1.0

        door = model.part(f"door_{i}")

        # Door leaf
        door.visual(
            _door_solid(sign, f"door_leaf_{i}"),
            material=steel_door,
            name="leaf",
        )
        # Dark backing behind vent slot
        xc = sign * DOOR_W / 2.0
        door.visual(
            Box((SLOT_W + 0.014, 0.005, SLOT_LEN + 0.03)),
            origin=Origin(xyz=(xc, -DOOR_T - 0.001, SLOT_ZC)),
            material=steel_dark,
            name="vent_backing",
        )
        # Stamped vent lines near top
        for j, dz in enumerate((0.52, 0.54, 0.56)):
            door.visual(
                Box((0.14, 0.004, 0.006)),
                origin=Origin(xyz=(xc, -0.0012, dz)),
                material=steel_dark,
                name=f"vent_line_{j}",
            )
        # Visible barrel hinges at upper and lower positions
        # Place them on the hinge edge, embedded slightly into the door face
        for k, z_offset in enumerate((DOOR_H * 0.35, -DOOR_H * 0.35)):
            door.visual(
                barrel_hinge_mesh,
                # Hinge leaf_a sits against the door surface (y ~ -0.002 to embed)
                origin=Origin(xyz=(0.0, -0.002, z_offset)),
                material=steel_hinge,
                name=f"barrel_hinge_{k}",
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

        # Separate pull handle - FIXED mounted on door face near free edge
        handle_part = model.part(f"pull_handle_{i}")
        handle_part.visual(
            handle_mesh,
            material=steel_handle,
            name="handle_body",
        )
        model.articulation(
            f"handle_{i}_mount",
            ArticulationType.FIXED,
            parent=door,
            child=handle_part,
            # Mount on front face, near free edge, at mid-height
            origin=Origin(xyz=(sign * (DOOR_W - 0.06), 0.001, 0.0)),
        )

    # ==================================================================
    # Tambour front (right half) - slides sideways
    # ==================================================================
    tambour = model.part("tambour_front")
    tambour.visual(
        _tambour_solid("tambour_panel"),
        material=steel_tambour,
        name="panel",
    )
    # Pull tab on the tambour front face
    tambour.visual(
        Box((0.04, 0.012, 0.025)),
        origin=Origin(xyz=(-TAMBOUR_W / 2.0 + 0.04, TAMBOUR_T / 2.0 + 0.005, 0.0)),
        material=steel_handle,
        name="tambour_pull",
    )

    model.articulation(
        "tambour_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tambour,
        origin=Origin(xyz=(TAMBOUR_X_CENTER, FRONT_Y - TAMBOUR_T / 2.0, DOOR_ZC)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=0.5, lower=0.0, upper=TAMBOUR_SLIDE
        ),
    )

    # ==================================================================
    # Lift-up top lid over shallow storage
    # ==================================================================
    lid = model.part("top_lid")
    # Lid panel: hinge at back edge (local y=0), extends forward to y=LID_D
    lid.visual(
        Box((LID_W, LID_D, LID_T)),
        origin=Origin(xyz=(0.0, LID_D / 2.0, LID_T / 2.0)),
        material=steel_lid,
        name="lid_panel",
    )
    # Lid handle near front edge
    lid.visual(
        Box((0.06, 0.012, 0.015)),
        origin=Origin(xyz=(0.0, LID_D - 0.04, LID_T + 0.006)),
        material=steel_handle,
        name="lid_handle",
    )

    # Lid hinge at back edge of the top compartment
    lid_hinge_y = BACK_Y + WALL_T  # -0.23
    lid_hinge_z = DIVIDER_Z + WALL_T / 2.0  # 1.53

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, lid_hinge_y, lid_hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0, velocity=1.5, lower=0.0, upper=LID_OPEN
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("cabinet_body")
    doors = [object_model.get_part(f"door_{i}") for i in range(2)]
    hinges = [object_model.get_articulation(f"door_{i}_hinge") for i in range(2)]
    handles = [object_model.get_part(f"pull_handle_{i}") for i in range(2)]
    tambour = object_model.get_part("tambour_front")
    tambour_joint = object_model.get_articulation("tambour_slide")
    lid = object_model.get_part("top_lid")
    lid_joint = object_model.get_articulation("lid_hinge")

    # --- Intentional overlaps -----------------------------------------------
    # Barrel hinges embed slightly into the door leaf surface
    for door in doors:
        for k in range(2):
            ctx.allow_overlap(
                door,
                door,
                elem_a=f"barrel_hinge_{k}",
                elem_b="leaf",
                reason="Barrel hinge leaf is intentionally embedded slightly into the door surface for realistic mounting.",
            )
    # Door 0 barrel hinges extend past the door edge into the side wall frame
    # (captured hinge barrel, like a real surface-mounted barrel hinge)
    for k in range(2):
        ctx.allow_overlap(
            body,
            doors[0],
            elem_a="side_wall_0",
            elem_b=f"barrel_hinge_{k}",
            reason="Barrel hinge knuckle extends slightly past the door edge into the frame, as in a real surface-mounted hinge.",
        )
    # Door 1 barrel hinges extend past the door edge into the door_centre_stile
    for k in range(2):
        ctx.allow_overlap(
            body,
            doors[1],
            elem_a="door_centre_stile",
            elem_b=f"barrel_hinge_{k}",
            reason="Barrel hinge knuckle extends slightly past the door edge into the centre stile, as in a real surface-mounted hinge.",
        )
    # Pull handles mount flush on door face with slight contact overlap
    for i, handle in enumerate(handles):
        ctx.allow_overlap(
            doors[i],
            handle,
            elem_a="leaf",
            elem_b="handle_body",
            reason="Pull handle is intentionally mounted flush against the door face.",
        )
    # Tambour in guide rails
    ctx.allow_overlap(
        tambour,
        body,
        elem_a="panel",
        elem_b="tambour_top_rail",
        reason="Tambour panel slides within the guide rail channel.",
    )
    ctx.allow_overlap(
        tambour,
        body,
        elem_a="panel",
        elem_b="tambour_bottom_rail",
        reason="Tambour panel slides within the guide rail channel.",
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
            1.78 <= z1 <= 1.88,
            details=f"top={z1:.3f}",
        )
        ctx.check("legs rest on the floor", abs(z0) <= 1e-6, details=f"zmin={z0:.5f}")

    # --- Doors: revolute, vertical axis, range ------------------------------
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

    # --- Visible barrel hinges on doors -------------------------------------
    for i, door in enumerate(doors):
        bh = ctx.part_element_world_aabb(door, elem="barrel_hinge_0")
        ctx.check(
            f"door_{i} has visible barrel hinge",
            bh is not None and (bh[1][2] - bh[0][2]) > 0.04,
            details=str(bh),
        )

    # --- Separate pull handles mounted on doors -----------------------------
    for i, (handle, door) in enumerate(zip(handles, doors)):
        haabb = ctx.part_world_aabb(handle)
        ctx.check(
            f"pull_handle_{i} is on the front face of door_{i}",
            haabb is not None and haabb[1][1] > FRONT_Y - 0.01,
            details=str(haabb),
        )

    # --- Doors open outward -------------------------------------------------
    closed0 = ctx.part_world_aabb(doors[0])
    with ctx.pose({hinges[0]: DOOR_OPEN}):
        open0 = ctx.part_world_aabb(doors[0])
    ctx.check(
        "door_0 swings outward past the front face",
        open0 is not None and open0[1][1] > FRONT_Y + 0.20,
        details=f"open0={open0}",
    )

    # --- Tambour: prismatic joint slides sideways ---------------------------
    ctx.check(
        "tambour joint is prismatic",
        tambour_joint.articulation_type == ArticulationType.PRISMATIC,
    )
    tax = tambour_joint.axis
    ctx.check(
        "tambour slides along X axis",
        abs(tax[0] - 1.0) < 1e-9 and abs(tax[1]) < 1e-9 and abs(tax[2]) < 1e-9,
        details=str(tax),
    )
    tlim = tambour_joint.motion_limits
    ctx.check(
        "tambour slide range > 0.3 m",
        tlim is not None and tlim.lower == 0.0 and tlim.upper > 0.3,
    )

    tambour_rest = ctx.part_world_position(tambour)
    with ctx.pose({tambour_joint: TAMBOUR_SLIDE}):
        tambour_slid = ctx.part_world_position(tambour)
    ctx.check(
        "tambour moves in +X when slid open",
        tambour_rest is not None
        and tambour_slid is not None
        and tambour_slid[0] > tambour_rest[0] + 0.1,
        details=f"rest={tambour_rest}, slid={tambour_slid}",
    )

    # --- Lid: revolute hinge opens upward -----------------------------------
    ctx.check(
        "lid joint is revolute",
        lid_joint.articulation_type == ArticulationType.REVOLUTE,
    )
    lax = lid_joint.axis
    ctx.check(
        "lid hinge axis is along X",
        abs(lax[0] - 1.0) < 1e-9 and abs(lax[1]) < 1e-9 and abs(lax[2]) < 1e-9,
        details=str(lax),
    )
    llim = lid_joint.motion_limits
    ctx.check(
        "lid opens 0..~85 deg",
        llim is not None
        and llim.lower == 0.0
        and abs(llim.upper - LID_OPEN) < 1e-6,
    )

    # Lid at rest sits on top of the divider
    lid_rest_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    ctx.check(
        "lid panel sits above divider at rest",
        lid_rest_aabb is not None and lid_rest_aabb[0][2] > DIVIDER_Z - 0.02,
        details=str(lid_rest_aabb),
    )

    # Lid opens upward
    with ctx.pose({lid_joint: LID_OPEN}):
        lid_open_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    ctx.check(
        "lid rises significantly when opened",
        lid_open_aabb is not None and lid_open_aabb[1][2] > DIVIDER_Z + 0.15,
        details=str(lid_open_aabb),
    )

    # Lid hinge is at the back edge
    ctx.check(
        "lid hinge is at the back edge",
        lid_joint.origin.xyz[1] < BACK_Y + 0.05,
        details=f"hinge_y={lid_joint.origin.xyz[1]:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
