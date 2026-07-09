from __future__ import annotations

"""Vintage industrial steel cabinet with sliding tambour front and drawers.

Variant of the vintage steel locker. This version replaces the four hinged doors
with a sliding tambour-style front panel covering the upper storage section and
two prismatic drawers in the lower section. Interior shelf boards are visible
through the opening when the tambour is raised.

Overall envelope ~1.6 m wide x 0.5 m deep x ~1.8 m tall, brushed/tarnished raw
steel finish. A hollow thin-wall (~0.02 m) carcass sits on four short splayed
legs and carries a thin riveted top cap strip.
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

# Vertical zones
BOTTOM_RAIL_TOP = LEG_H + 0.06  # 0.21
DRAWER_ZONE_TOP = 0.65
DIVIDER_T = 0.02
DIVIDER_TOP = DRAWER_ZONE_TOP + DIVIDER_T  # 0.67
OPENING_TOP = CAB_TOP - 0.08  # 1.72
TOP_RAIL_BOT = OPENING_TOP

CAP_T = 0.022
CAP_OVERHANG = 0.02

# Drawer dimensions
DRAWER_W = CAB_W - 2 * WALL_T - 0.02  # 1.54
DRAWER_H = 0.19
DRAWER_D = 0.40
DRAWER_FRONT_T = 0.015
DRAWER_WALL_T = 0.010
DRAWER_BOTTOM_T = 0.008
DRAWER_GAP = 0.015

DRAWER_Z = [
    BOTTOM_RAIL_TOP + 0.005 + DRAWER_H / 2.0,  # 0.31
    BOTTOM_RAIL_TOP + 0.005 + DRAWER_H + DRAWER_GAP + DRAWER_H / 2.0,  # 0.515
]
DRAWER_MAX_EXTENSION = 0.35

# Tambour dimensions
TAMBOUR_W = CAB_W - 2 * WALL_T - 0.03  # 1.53
TAMBOUR_H = OPENING_TOP - DIVIDER_TOP  # 1.05
TAMBOUR_T = 0.012
TAMBOUR_TRAVEL = TAMBOUR_H - 0.05  # 1.00

# Shelf positions inside upper section
SHELF_Z = [0.92, 1.22, 1.52]
SHELF_W = CAB_W - 2 * WALL_T - 0.01
SHELF_D = CAB_D - 2 * WALL_T - 0.02
SHELF_T = 0.015


def _leg_solid(mesh_name: str):
    """Splayed tapered leg: small foot on the floor, wide top embedded into
    the carcass bottom."""
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
    """Tambour panel: flat steel panel with horizontal slat ridges built as
    one CadQuery union solid."""
    panel = (
        cq.Workplane("XY")
        .box(TAMBOUR_W, TAMBOUR_T, TAMBOUR_H)
    )
    # Horizontal ridges on the front face to suggest tambour slats
    n_ridges = 14
    spacing = TAMBOUR_H / (n_ridges + 1)
    for j in range(1, n_ridges + 1):
        gz = -TAMBOUR_H / 2.0 + j * spacing
        ridge = (
            cq.Workplane("XY")
            .box(TAMBOUR_W - 0.02, 0.004, 0.005)
            .translate((0.0, TAMBOUR_T / 2.0 + 0.002, gz))
        )
        panel = panel.union(ridge)
    return mesh_from_cadquery(panel, mesh_name)


def _drawer_box_solid(mesh_name: str):
    """Open-top drawer box: front, bottom, sides, back as one CadQuery solid."""
    inner_w = DRAWER_W - 2 * DRAWER_WALL_T
    inner_d = DRAWER_D - DRAWER_FRONT_T - DRAWER_WALL_T
    box_h = DRAWER_H - DRAWER_BOTTOM_T

    # Front face (full width, full height)
    front = (
        cq.Workplane("XY")
        .box(DRAWER_W, DRAWER_FRONT_T, DRAWER_H)
        .translate((0.0, -DRAWER_FRONT_T / 2.0, 0.0))
    )
    # Bottom
    bottom = (
        cq.Workplane("XY")
        .box(inner_w, DRAWER_D - DRAWER_FRONT_T, DRAWER_BOTTOM_T)
        .translate((0.0, -DRAWER_FRONT_T - (DRAWER_D - DRAWER_FRONT_T) / 2.0,
                     -DRAWER_H / 2.0 + DRAWER_BOTTOM_T / 2.0))
    )
    # Left side
    left = (
        cq.Workplane("XY")
        .box(DRAWER_WALL_T, DRAWER_D - DRAWER_FRONT_T, box_h)
        .translate((-(DRAWER_W / 2.0 - DRAWER_WALL_T / 2.0),
                     -DRAWER_FRONT_T - (DRAWER_D - DRAWER_FRONT_T) / 2.0,
                     -DRAWER_H / 2.0 + DRAWER_BOTTOM_T + box_h / 2.0))
    )
    # Right side
    right = (
        cq.Workplane("XY")
        .box(DRAWER_WALL_T, DRAWER_D - DRAWER_FRONT_T, box_h)
        .translate(((DRAWER_W / 2.0 - DRAWER_WALL_T / 2.0),
                     -DRAWER_FRONT_T - (DRAWER_D - DRAWER_FRONT_T) / 2.0,
                     -DRAWER_H / 2.0 + DRAWER_BOTTOM_T + box_h / 2.0))
    )
    # Back
    back = (
        cq.Workplane("XY")
        .box(inner_w, DRAWER_WALL_T, box_h)
        .translate((0.0,
                     -DRAWER_FRONT_T - (DRAWER_D - DRAWER_FRONT_T) + DRAWER_WALL_T / 2.0,
                     -DRAWER_H / 2.0 + DRAWER_BOTTOM_T + box_h / 2.0))
    )
    body = front.union(bottom).union(left).union(right).union(back)
    return mesh_from_cadquery(body, mesh_name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_tambour_cabinet")

    # Materials – brushed/tarnished raw steel palette
    steel_body = model.material("steel_body", rgba=(0.60, 0.61, 0.63, 1.0))
    steel_trim = model.material("steel_trim", rgba=(0.46, 0.47, 0.49, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.07, 0.07, 0.08, 1.0))
    steel_leg = model.material("steel_leg", rgba=(0.38, 0.39, 0.41, 1.0))
    steel_rivet = model.material("steel_rivet", rgba=(0.52, 0.53, 0.55, 1.0))
    steel_drawer = model.material("steel_drawer", rgba=(0.55, 0.56, 0.58, 1.0))
    steel_tambour = model.material("steel_tambour", rgba=(0.50, 0.51, 0.53, 1.0))
    steel_shelf = model.material("steel_shelf", rgba=(0.58, 0.59, 0.61, 1.0))
    steel_handle = model.material("steel_handle", rgba=(0.22, 0.22, 0.24, 1.0))

    # =================================================================
    # CABINET BODY (root)
    # =================================================================
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

    # Bottom panel
    body.visual(
        Box((CAB_W, CAB_D, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, LEG_H + WALL_T / 2.0)),
        material=steel_body,
        name="bottom_panel",
    )

    # Top panel
    body.visual(
        Box((CAB_W, CAB_D, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, CAB_TOP - WALL_T / 2.0)),
        material=steel_body,
        name="top_panel",
    )

    # Horizontal divider between drawers and upper section
    body.visual(
        Box((CAB_W - 2 * WALL_T + 0.01, CAB_D - WALL_T, DIVIDER_T)),
        origin=Origin(xyz=(0.0, WALL_T / 2.0, DRAWER_ZONE_TOP + DIVIDER_T / 2.0)),
        material=steel_body,
        name="divider_panel",
    )

    # Front frame: bottom rail, mid rail, top rail
    body.visual(
        Box((CAB_W - 2 * WALL_T, WALL_T, BOTTOM_RAIL_TOP - LEG_H + 0.01)),
        origin=Origin(
            xyz=(0.0, FRONT_Y - WALL_T / 2.0, (LEG_H + BOTTOM_RAIL_TOP) / 2.0)
        ),
        material=steel_body,
        name="front_bottom_rail",
    )
    body.visual(
        Box((CAB_W - 2 * WALL_T, WALL_T, CAB_TOP - TOP_RAIL_BOT + 0.01)),
        origin=Origin(
            xyz=(0.0, FRONT_Y - WALL_T / 2.0, (TOP_RAIL_BOT + CAB_TOP) / 2.0)
        ),
        material=steel_body,
        name="front_top_rail",
    )
    # Mid rail between drawers and upper opening
    body.visual(
        Box((CAB_W - 2 * WALL_T, WALL_T, DIVIDER_T + 0.01)),
        origin=Origin(
            xyz=(0.0, FRONT_Y - WALL_T / 2.0, DRAWER_ZONE_TOP + DIVIDER_T / 2.0)
        ),
        material=steel_body,
        name="front_mid_rail",
    )

    # Tambour track (thin vertical channel strips on inner side walls)
    track_h = TAMBOUR_H + 0.12
    for sx, vname in ((-1.0, "tambour_track_0"), (1.0, "tambour_track_1")):
        body.visual(
            Box((0.018, 0.018, track_h)),
            origin=Origin(xyz=(
                sx * (CAB_W / 2.0 - WALL_T - 0.009),
                FRONT_Y - WALL_T - 0.009,
                DIVIDER_TOP + TAMBOUR_H / 2.0,
            )),
            material=steel_trim,
            name=vname,
        )

    # Top cap strip with rivets
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

    # Interior shelf boards (visible through the tambour opening)
    for i, sz in enumerate(SHELF_Z):
        body.visual(
            Box((SHELF_W, SHELF_D, SHELF_T)),
            origin=Origin(xyz=(0.0, -0.01, sz)),
            material=steel_shelf,
            name=f"shelf_{i}",
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

    # =================================================================
    # TAMBOUR PANEL (prismatic, slides upward to reveal shelves)
    # =================================================================
    tambour = model.part("tambour_panel")

    # Panel solid with horizontal slat ridges.
    # Child origin at panel bottom-center; panel extends upward along +Z.
    tambour.visual(
        _tambour_solid("tambour_face"),
        origin=Origin(xyz=(0.0, 0.0, TAMBOUR_H / 2.0)),
        material=steel_tambour,
        name="panel",
    )
    # Bottom-edge pull handle (above the divider line)
    tambour.visual(
        Box((0.20, 0.020, 0.012)),
        origin=Origin(xyz=(0.0, TAMBOUR_T / 2.0 + 0.010, 0.020)),
        material=steel_handle,
        name="pull_handle",
    )

    # Joint: tambour slides up along Z from the bottom of the opening
    model.articulation(
        "tambour_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tambour,
        origin=Origin(xyz=(0.0, FRONT_Y - WALL_T / 2.0, DIVIDER_TOP)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0, velocity=0.30, lower=0.0, upper=TAMBOUR_TRAVEL,
        ),
    )

    # =================================================================
    # DRAWERS (prismatic, slide forward along +Y)
    # =================================================================
    drawer_mesh = _drawer_box_solid("drawer_box")

    drawers = []
    for i in range(2):
        drawer = model.part(f"drawer_{i}")
        zc = DRAWER_Z[i]

        # Drawer box – child origin at front-face center (closed position)
        drawer.visual(
            drawer_mesh,
            material=steel_drawer,
            name="box",
        )
        # Handle bar on front face
        drawer.visual(
            Box((0.12, 0.015, 0.020)),
            origin=Origin(xyz=(0.0, 0.008, 0.0)),
            material=steel_handle,
            name="handle",
        )
        # Handle end caps
        for sx in (-1.0, 1.0):
            drawer.visual(
                Cylinder(radius=0.012, length=0.020),
                origin=Origin(xyz=(
                    sx * 0.060,
                    0.008,
                    0.0,
                ), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=steel_handle,
                name=f"handle_cap{'_r' if sx > 0 else '_l'}",
            )

        # Prismatic joint: slides forward along +Y
        model.articulation(
            f"drawer_{i}_slide",
            ArticulationType.PRISMATIC,
            parent=body,
            child=drawer,
            origin=Origin(xyz=(0.0, FRONT_Y, zc)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=40.0, velocity=0.25, lower=0.0, upper=DRAWER_MAX_EXTENSION,
            ),
        )
        drawers.append(drawer)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("cabinet_body")
    tambour = object_model.get_part("tambour_panel")
    tambour_joint = object_model.get_articulation("tambour_slide")
    drawers = [object_model.get_part(f"drawer_{i}") for i in range(2)]
    drawer_joints = [object_model.get_articulation(f"drawer_{i}_slide") for i in range(2)]

    # --- Intentional overlaps -------------------------------------------
    # Tambour panel slides in tracks; small local overlap with track rails
    # is the intended captured-slide representation.
    for track_name in ("tambour_track_0", "tambour_track_1"):
        ctx.allow_overlap(
            tambour,
            body,
            elem_a="panel",
            elem_b=track_name,
            reason="Tambour panel is intentionally represented as sliding inside the track channels.",
        )

    # Tambour panel top edge slightly embeds behind the top rail when closed
    # (the rail captures the panel upper edge like a real tambour header).
    ctx.allow_overlap(
        tambour,
        body,
        elem_a="panel",
        elem_b="front_top_rail",
        reason="Tambour panel upper edge is captured behind the front top rail header.",
    )

    # Tambour panel bottom edge tucks behind the mid rail divider when closed
    # (a real tambour's bottom edge seats into the divider rail groove).
    ctx.allow_overlap(
        tambour,
        body,
        elem_a="panel",
        elem_b="front_mid_rail",
        reason="Tambour panel bottom edge seats into the divider rail groove when closed.",
    )

    # --- Overall envelope -----------------------------------------------
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

    # --- Tambour mechanism ----------------------------------------------
    ctx.check(
        "tambour_slide is prismatic",
        tambour_joint.articulation_type == ArticulationType.PRISMATIC,
    )
    ax = tambour_joint.axis
    ctx.check(
        "tambour axis is vertical (Z)",
        abs(ax[0]) < 1e-9 and abs(ax[1]) < 1e-9 and abs(abs(ax[2]) - 1.0) < 1e-9,
        details=str(ax),
    )
    lim = tambour_joint.motion_limits
    ctx.check(
        "tambour travel 0..~1.0 m",
        lim is not None and lim.lower == 0.0 and 0.80 <= lim.upper <= 1.10,
        details=f"upper={lim.upper}",
    )

    # At rest (q=0), tambour covers the upper opening
    tambour_rest = ctx.part_world_aabb(tambour)
    ctx.check(
        "tambour at rest covers the upper opening",
        tambour_rest is not None
        and tambour_rest[0][2] < DIVIDER_TOP + 0.02
        and tambour_rest[1][2] > OPENING_TOP - 0.02,
        details=str(tambour_rest),
    )

    # Opening pose: tambour slides upward, bottom edge rises
    with ctx.pose({tambour_joint: TAMBOUR_TRAVEL}):
        tambour_open = ctx.part_world_aabb(tambour)
    ctx.check(
        "tambour slides upward when opened",
        tambour_open is not None
        and tambour_open[0][2] > tambour_rest[0][2] + 0.50,
        details=f"rest_bottom={tambour_rest[0][2]:.3f}, open_bottom={tambour_open[0][2]:.3f}",
    )

    # Proof checks for intentional overlaps:
    # Panel bottom edge (at rest) sits near the divider line in Z
    ctx.check(
        "tambour panel bottom sits near the divider line when closed",
        tambour_rest is not None
        and abs(tambour_rest[0][2] - DIVIDER_TOP) < 0.015,
        details=f"panel_bottom_z={tambour_rest[0][2]:.4f}, divider_top={DIVIDER_TOP:.4f}",
    )
    # Panel top edge stays near the opening top when closed
    ctx.check(
        "tambour panel top seats near the header when closed",
        tambour_rest is not None
        and abs(tambour_rest[1][2] - OPENING_TOP) < 0.025,
        details=f"panel_top_z={tambour_rest[1][2]:.4f}, opening_top={OPENING_TOP:.4f}",
    )

    # --- Shelves visible through the opening ----------------------------
    body_aabb = ctx.part_world_aabb(body)
    for i, sz in enumerate(SHELF_Z):
        shelf_aabb = ctx.part_element_world_aabb(body, elem=f"shelf_{i}")
        ctx.check(
            f"shelf_{i} exists in the upper section",
            shelf_aabb is not None
            and shelf_aabb[0][2] > DIVIDER_TOP
            and shelf_aabb[1][2] < OPENING_TOP,
            details=str(shelf_aabb),
        )
        # Shelf fits inside the cabinet width
        if shelf_aabb is not None and body_aabb is not None:
            ctx.check(
                f"shelf_{i} fits inside the cabinet width",
                shelf_aabb[0][0] > body_aabb[0][0] - 0.01
                and shelf_aabb[1][0] < body_aabb[1][0] + 0.01,
                details=f"shelf_x=[{shelf_aabb[0][0]:.3f},{shelf_aabb[1][0]:.3f}] body_x=[{body_aabb[0][0]:.3f},{body_aabb[1][0]:.3f}]",
            )

    # --- Drawer mechanisms -----------------------------------------------
    for i, (drawer, joint) in enumerate(zip(drawers, drawer_joints)):
        ctx.check(
            f"drawer_{i}_slide is prismatic",
            joint.articulation_type == ArticulationType.PRISMATIC,
        )
        dax = joint.axis
        ctx.check(
            f"drawer_{i} axis is forward (+Y)",
            abs(dax[0]) < 1e-9 and abs(abs(dax[1]) - 1.0) < 1e-9 and abs(dax[2]) < 1e-9,
            details=str(dax),
        )
        dlim = joint.motion_limits
        ctx.check(
            f"drawer_{i} travel 0..~0.35 m",
            dlim is not None and dlim.lower == 0.0 and 0.25 <= dlim.upper <= 0.40,
            details=f"upper={dlim.upper}",
        )

        # Closed: front face flush with cabinet front
        front_rest = ctx.part_element_world_aabb(drawer, elem="handle")
        ctx.check(
            f"drawer_{i} at rest has handle near the front face",
            front_rest is not None and abs(front_rest[1][1] - FRONT_Y) < 0.03,
            details=str(front_rest),
        )

        # Opening pose: drawer extends forward
        closed_pos = ctx.part_world_position(drawer)
        with ctx.pose({joint: DRAWER_MAX_EXTENSION}):
            extended_pos = ctx.part_world_position(drawer)
        ctx.check(
            f"drawer_{i} extends forward when opened",
            closed_pos is not None
            and extended_pos is not None
            and extended_pos[1] > closed_pos[1] + 0.20,
            details=f"closed_y={closed_pos[1]:.3f}, extended_y={extended_pos[1]:.3f}",
        )

    # --- Drawer handles exist -------------------------------------------
    for i, drawer in enumerate(drawers):
        handle_aabb = ctx.part_element_world_aabb(drawer, elem="handle")
        ctx.check(
            f"drawer_{i} has a handle",
            handle_aabb is not None and (handle_aabb[1][1] - handle_aabb[0][1]) > 0.005,
            details=str(handle_aabb),
        )

    # --- Riveted top cap detail present ---------------------------------
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
