from __future__ import annotations

"""Vintage industrial steel cabinet variant: glass-framed upper doors, solid
lower doors, hinged top lid, visible hinge barrels, and separate pull handles.

Overall ~1.6 m wide × 0.5 m deep × ~1.82 m tall, brushed/tarnished raw steel.
The front is split by a horizontal mid rail into four glass-framed upper doors
and two solid lower doors. Each door carries visible cylindrical hinge barrels
along its hinge edge and a bar-style pull handle near the free edge. A top lid
hinges upward on a rear revolute joint. The two lower doors hinge on their
outer edges; the four upper doors mirror the same pattern (outer pair on outer
edges, inner pair on inner stiles). All door hinges are independent revolute
joints about vertical axes, 0..~110 deg outward.
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
# Global dimensions (meters). Cabinet centred on X, front face at +Y.
# ---------------------------------------------------------------------------
CAB_W = 1.60
CAB_D = 0.50
CAB_TOP = 1.80
LEG_H = 0.15
WALL_T = 0.02

FRONT_Y = CAB_D / 2.0  # +0.25
BACK_Y = -CAB_D / 2.0  # -0.25

# Vertical zone boundaries
BOTTOM_RAIL_TOP = LEG_H + 0.06  # 0.21
MID_RAIL_BOT = 0.97
MID_RAIL_TOP = 1.00
TOP_RAIL_BOT = CAB_TOP - 0.06  # 1.74

# Lower door zone
LOWER_Z0 = BOTTOM_RAIL_TOP + 0.002  # 0.212
LOWER_Z1 = MID_RAIL_BOT - 0.002  # 0.968
LOWER_H = LOWER_Z1 - LOWER_Z0  # ~0.756
LOWER_ZC = 0.5 * (LOWER_Z0 + LOWER_Z1)

# Upper door zone
UPPER_Z0 = MID_RAIL_TOP + 0.002  # 1.002
UPPER_Z1 = TOP_RAIL_BOT - 0.002  # 1.738
UPPER_H = UPPER_Z1 - UPPER_Z0  # ~0.736
UPPER_ZC = 0.5 * (UPPER_Z0 + UPPER_Z1)

STILE_W = 0.03
DOOR_T = WALL_T

# Lower doors: 2 doors, hinged on outer edges
LOWER_W = 0.762
LOWER_SPECS = [
    # (hinge_x, sign)  sign=+1 → panel extends +X (left-hinged)
    (-0.78, +1.0),
    (0.78, -1.0),
]

# Upper doors: 4 doors, hinged on outer and inner stile edges
UPPER_W = 0.364
UPPER_SPECS = [
    (-0.78, +1.0),  # far left
    (-0.3825, +1.0),  # centre-left
    (0.3825, -1.0),  # centre-right
    (0.78, -1.0),  # far right
]

HINGE_INSET = 0.0005

# Hinge barrel dimensions
BARREL_R = 0.009
KNUCKLE_R = 0.011

# Lid
LID_T = 0.015
LID_HINGE_Z = CAB_TOP  # lid sits directly on top panel
LID_OPEN = math.radians(85.0)

# Door open angle
DOOR_OPEN = math.radians(110.0)

# Glass frame
FRAME_RAIL = 0.035
GLASS_T = 0.004

# Pull handle
HANDLE_STANDOFF = 0.024
HANDLE_BAR_R = 0.006
HANDLE_POST_R = 0.005
HANDLE_POST_LEN = HANDLE_STANDOFF - 0.004

# Vent slot on lower doors
SLOT_LEN = 0.28
SLOT_W = 0.026
SLOT_ZC_LOCAL = -0.22  # in door-local z (below centre)

CAP_OVERHANG = 0.015


def _hinge_barrel_solid(barrel_len: float, mesh_name: str):
    """Cylindrical hinge barrel with knuckle rings."""
    barrel = (
        cq.Workplane("XY")
        .circle(BARREL_R)
        .extrude(barrel_len / 2.0, both=True)
    )
    ring_h = 0.048
    n_rings = max(3, int(barrel_len / 0.22))
    span = barrel_len * 0.82
    for i in range(n_rings):
        z = -span / 2.0 + i * (span / max(1, n_rings - 1))
        ring = (
            cq.Workplane("XY")
            .circle(KNUCKLE_R)
            .extrude(ring_h / 2.0, both=True)
            .translate((0.0, 0.0, z))
        )
        barrel = barrel.union(ring)
    return mesh_from_cadquery(barrel, mesh_name)


def _glass_frame_solid(w: float, h: float, t: float, rail_w: float, mesh_name: str):
    """Steel door frame with rectangular through-opening for glass."""
    frame = cq.Workplane("XY").box(w, t, h)
    iw = w - 2.0 * rail_w
    ih = h - 2.0 * rail_w
    cutter = cq.Workplane("XY").box(iw, t + 0.02, ih)
    result = frame.cut(cutter)
    return mesh_from_cadquery(result, mesh_name)


def _lower_door_solid(w: float, h: float, t: float, sign: float, mesh_name: str):
    """Solid door panel with a rounded-end vent slot near the bottom."""
    xc = sign * w / 2.0
    panel = (
        cq.Workplane("XY")
        .box(w, t, h)
        .translate((xc, -t / 2.0, 0.0))
    )
    slot_cutter = (
        cq.Workplane("XZ")
        .slot2D(SLOT_LEN, SLOT_W, 90)
        .extrude(0.05, both=True)
        .translate((xc, 0.0, SLOT_ZC_LOCAL))
    )
    leaf = panel.cut(slot_cutter)
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


def _add_pull_handle(door, sign: float, door_w: float, mat):
    """Add bar-style pull handle visuals near the free edge of a door.
    The backplate embeds into the door panel for visual connectivity."""
    hx = sign * (door_w - 0.07)
    # Backplate — embeds through the door panel thickness
    plate_t = DOOR_T + 0.004
    door.visual(
        Box((0.024, plate_t, 0.11)),
        origin=Origin(xyz=(hx, -DOOR_T / 2.0 + 0.002, 0.0)),
        material=mat,
        name="handle_plate",
    )
    # Two standoff posts
    post_y_start = 0.002
    for dz in (-0.038, 0.038):
        door.visual(
            Cylinder(radius=HANDLE_POST_R, length=HANDLE_POST_LEN),
            origin=Origin(
                xyz=(hx, post_y_start + HANDLE_POST_LEN / 2.0, dz),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=mat,
            name="handle_post",
        )
    # Grip bar (vertical)
    bar_len = 0.085
    door.visual(
        Cylinder(radius=HANDLE_BAR_R, length=bar_len),
        origin=Origin(xyz=(hx, HANDLE_STANDOFF, 0.0)),
        material=mat,
        name="handle_bar",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_steel_cabinet_variant")

    # Materials
    steel_body = model.material("steel_body", rgba=(0.60, 0.61, 0.63, 1.0))
    steel_upper = model.material("steel_upper", rgba=(0.55, 0.56, 0.58, 1.0))
    steel_lower = model.material("steel_lower", rgba=(0.50, 0.51, 0.54, 1.0))
    steel_trim = model.material("steel_trim", rgba=(0.46, 0.47, 0.49, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.07, 0.07, 0.08, 1.0))
    steel_leg = model.material("steel_leg", rgba=(0.38, 0.39, 0.41, 1.0))
    steel_handle = model.material("steel_handle", rgba=(0.28, 0.29, 0.31, 1.0))
    steel_rivet = model.material("steel_rivet", rgba=(0.52, 0.53, 0.55, 1.0))
    glass_mat = model.material("glass", rgba=(0.74, 0.80, 0.88, 0.55))

    # ==================================================================
    # CABINET BODY (root): carcass, legs, front frame, top panel, rivets
    # ==================================================================
    body = model.part("cabinet_body")
    carcass_h = CAB_TOP - LEG_H  # 1.65
    carcass_zc = LEG_H + carcass_h / 2.0

    # Side walls
    for sx, vn in ((-1.0, "side_wall_0"), (1.0, "side_wall_1")):
        body.visual(
            Box((WALL_T, CAB_D, carcass_h)),
            origin=Origin(xyz=(sx * (CAB_W / 2.0 - WALL_T / 2.0), 0.0, carcass_zc)),
            material=steel_body,
            name=vn,
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

    # Interior shelf
    body.visual(
        Box((CAB_W - 2.0 * WALL_T + 0.01, 0.43, 0.015)),
        origin=Origin(xyz=(0.0, -0.02, 0.60)),
        material=steel_body,
        name="interior_shelf",
    )

    # --- Front frame rails ---
    # Bottom rail
    body.visual(
        Box((CAB_W - 2.0 * WALL_T, WALL_T, BOTTOM_RAIL_TOP - LEG_H + 0.01)),
        origin=Origin(
            xyz=(0.0, FRONT_Y - WALL_T / 2.0, (LEG_H + BOTTOM_RAIL_TOP) / 2.0)
        ),
        material=steel_body,
        name="front_bottom_rail",
    )
    # Mid rail (separates lower and upper zones)
    mid_rail_h = MID_RAIL_TOP - MID_RAIL_BOT + 0.01
    body.visual(
        Box((CAB_W - 2.0 * WALL_T, WALL_T, mid_rail_h)),
        origin=Origin(
            xyz=(0.0, FRONT_Y - WALL_T / 2.0, (MID_RAIL_BOT + MID_RAIL_TOP) / 2.0)
        ),
        material=steel_body,
        name="front_mid_rail",
    )
    # Top rail
    body.visual(
        Box((CAB_W - 2.0 * WALL_T, WALL_T, CAB_TOP - TOP_RAIL_BOT + 0.01)),
        origin=Origin(
            xyz=(0.0, FRONT_Y - WALL_T / 2.0, (TOP_RAIL_BOT + CAB_TOP) / 2.0)
        ),
        material=steel_body,
        name="front_top_rail",
    )

    # --- Front frame stiles ---
    # Lower zone: 1 centre stile
    lower_stile_h = MID_RAIL_BOT - BOTTOM_RAIL_TOP + 0.01
    body.visual(
        Box((STILE_W, WALL_T, lower_stile_h)),
        origin=Origin(
            xyz=(0.0, FRONT_Y - WALL_T / 2.0, (BOTTOM_RAIL_TOP + MID_RAIL_BOT) / 2.0)
        ),
        material=steel_trim,
        name="lower_stile_0",
    )
    # Upper zone: 3 stiles
    upper_stile_h = TOP_RAIL_BOT - MID_RAIL_TOP + 0.01
    upper_stile_zc = (MID_RAIL_TOP + TOP_RAIL_BOT) / 2.0
    for i, xc in enumerate((-0.3975, 0.0, 0.3975)):
        body.visual(
            Box((STILE_W, WALL_T, upper_stile_h)),
            origin=Origin(xyz=(xc, FRONT_Y - WALL_T / 2.0, upper_stile_zc)),
            material=steel_trim,
            name=f"upper_stile_{i}",
        )

    # Raised rivet dots along the top rail
    for i in range(13):
        rx = -0.72 + i * (1.44 / 12.0)
        body.visual(
            Sphere(radius=0.0045),
            origin=Origin(xyz=(rx, FRONT_Y + 0.002, 1.77)),
            material=steel_rivet,
            name=f"rivet_{i}",
        )

    # Splayed legs
    leg_mesh = _leg_solid("splayed_leg")
    corners = [
        (0.72, 0.19, 0.0),
        (-0.72, 0.19, math.pi / 2.0),
        (-0.72, -0.19, math.pi),
        (0.72, -0.19, 3.0 * math.pi / 2.0),
    ]
    for i, (lx, ly, yaw) in enumerate(corners):
        body.visual(
            leg_mesh,
            origin=Origin(xyz=(lx, ly, 0.0), rpy=(0.0, 0.0, yaw)),
            material=steel_leg,
            name=f"leg_{i}",
        )

    # ==================================================================
    # LOWER DOORS (2 solid steel panels with vent slots + handles)
    # ==================================================================
    lower_barrel_len = LOWER_H - 0.04
    lower_barrel_mesh = _hinge_barrel_solid(lower_barrel_len, "lower_hinge_barrel")
    lower_doors = []
    lower_hinges = []

    for i, (hinge_x, sign) in enumerate(LOWER_SPECS):
        door = model.part(f"lower_door_{i}")
        xc = sign * LOWER_W / 2.0

        # Solid door leaf with vent slot
        door.visual(
            _lower_door_solid(LOWER_W, LOWER_H, DOOR_T, sign, f"lower_leaf_{i}"),
            material=steel_lower,
            name="leaf",
        )

        # Dark backing behind vent slot
        door.visual(
            Box((SLOT_W + 0.014, 0.005, SLOT_LEN + 0.028)),
            origin=Origin(xyz=(xc, -DOOR_T - 0.001, SLOT_ZC_LOCAL)),
            material=steel_dark,
            name="vent_backing",
        )

        # Visible hinge barrel on door hinge edge — embeds into panel
        door.visual(
            lower_barrel_mesh,
            origin=Origin(xyz=(0.0, -0.002, 0.0)),
            material=steel_trim,
            name="hinge_barrel",
        )
        # Hinge leaf plate connecting barrel to door panel
        door.visual(
            Box((0.022, DOOR_T + 0.004, lower_barrel_len * 0.55)),
            origin=Origin(xyz=(sign * 0.011, -DOOR_T / 2.0, 0.0)),
            material=steel_trim,
            name="hinge_leaf",
        )

        # Pull handle near free edge
        _add_pull_handle(door, sign, LOWER_W, steel_handle)

        # Articulation: revolute about vertical Z axis
        hinge_x_world = hinge_x + sign * HINGE_INSET
        art = model.articulation(
            f"lower_door_{i}_hinge",
            ArticulationType.REVOLUTE,
            parent=body,
            child=door,
            origin=Origin(xyz=(hinge_x_world, FRONT_Y, LOWER_ZC)),
            axis=(0.0, 0.0, sign),
            motion_limits=MotionLimits(
                effort=40.0, velocity=2.0, lower=0.0, upper=DOOR_OPEN
            ),
        )
        lower_doors.append(door)
        lower_hinges.append(art)

    # ==================================================================
    # UPPER DOORS (4 glass-framed doors with handles)
    # ==================================================================
    upper_barrel_len = UPPER_H - 0.04
    upper_barrel_mesh = _hinge_barrel_solid(upper_barrel_len, "upper_hinge_barrel")
    upper_frame_mesh = _glass_frame_solid(
        UPPER_W, UPPER_H, DOOR_T, FRAME_RAIL, "upper_frame"
    )
    glass_iw = UPPER_W - 2.0 * FRAME_RAIL
    glass_ih = UPPER_H - 2.0 * FRAME_RAIL

    upper_doors = []
    upper_hinges = []

    for i, (hinge_x, sign) in enumerate(UPPER_SPECS):
        door = model.part(f"upper_door_{i}")
        xc = sign * UPPER_W / 2.0

        # Steel frame with rectangular opening
        door.visual(
            upper_frame_mesh,
            origin=Origin(xyz=(xc, -DOOR_T / 2.0, 0.0)),
            material=steel_upper,
            name="frame",
        )

        # Glass pane — slightly oversized to embed 3 mm into the frame rails
        glass_embed_w = glass_iw + 0.006
        glass_embed_h = glass_ih + 0.006
        door.visual(
            Box((glass_embed_w, GLASS_T, glass_embed_h)),
            origin=Origin(xyz=(xc, -DOOR_T / 2.0, 0.0)),
            material=glass_mat,
            name="glass_pane",
        )

        # Visible hinge barrel — shifted to embed into the frame hinge rail
        door.visual(
            upper_barrel_mesh,
            origin=Origin(xyz=(0.0, -0.002, 0.0)),
            material=steel_trim,
            name="hinge_barrel",
        )
        # Hinge leaf plate connecting barrel to frame (ensures connectivity)
        door.visual(
            Box((0.022, DOOR_T + 0.004, upper_barrel_len * 0.55)),
            origin=Origin(xyz=(sign * 0.011, -DOOR_T / 2.0, 0.0)),
            material=steel_trim,
            name="hinge_leaf",
        )

        # Pull handle — backplate embeds into the frame free-edge rail
        hx = sign * (UPPER_W - 0.045)
        door.visual(
            Box((0.032, DOOR_T + 0.004, 0.11)),
            origin=Origin(xyz=(hx, -DOOR_T / 2.0, 0.0)),
            material=steel_handle,
            name="handle_plate",
        )
        for dz in (-0.038, 0.038):
            door.visual(
                Cylinder(radius=HANDLE_POST_R, length=HANDLE_POST_LEN),
                origin=Origin(
                    xyz=(hx, HANDLE_POST_LEN / 2.0 + 0.002, dz),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=steel_handle,
                name="handle_post",
            )
        door.visual(
            Cylinder(radius=HANDLE_BAR_R, length=0.085),
            origin=Origin(xyz=(hx, HANDLE_STANDOFF, 0.0)),
            material=steel_handle,
            name="handle_bar",
        )

        # Articulation
        hinge_x_world = hinge_x + sign * HINGE_INSET
        art = model.articulation(
            f"upper_door_{i}_hinge",
            ArticulationType.REVOLUTE,
            parent=body,
            child=door,
            origin=Origin(xyz=(hinge_x_world, FRONT_Y, UPPER_ZC)),
            axis=(0.0, 0.0, sign),
            motion_limits=MotionLimits(
                effort=40.0, velocity=2.0, lower=0.0, upper=DOOR_OPEN
            ),
        )
        upper_doors.append(door)
        upper_hinges.append(art)

    # ==================================================================
    # TOP LID (hinged at rear, opens upward)
    # ==================================================================
    lid = model.part("top_lid")
    lid_w = CAB_W + CAP_OVERHANG
    lid_d = CAB_D + CAP_OVERHANG

    # Lid panel extends forward (+Y in local) from the hinge
    lid.visual(
        Box((lid_w, lid_d, LID_T)),
        origin=Origin(xyz=(0.0, lid_d / 2.0, LID_T / 2.0)),
        material=steel_trim,
        name="lid_panel",
    )
    # Small lip/handle at front edge of lid
    lid.visual(
        Box((0.12, 0.015, 0.008)),
        origin=Origin(xyz=(0.0, lid_d - 0.008, LID_T + 0.004)),
        material=steel_handle,
        name="lid_pull",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        # Hinge at the rear top edge of the carcass
        origin=Origin(xyz=(0.0, BACK_Y, LID_HINGE_Z)),
        # +X axis: right-hand rule rotates +Y toward +Z → lifts front edge up
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0, velocity=2.0, lower=0.0, upper=LID_OPEN
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("cabinet_body")
    lower_doors = [object_model.get_part(f"lower_door_{i}") for i in range(2)]
    lower_hinges = [
        object_model.get_articulation(f"lower_door_{i}_hinge") for i in range(2)
    ]
    upper_doors = [object_model.get_part(f"upper_door_{i}") for i in range(4)]
    upper_hinges = [
        object_model.get_articulation(f"upper_door_{i}_hinge") for i in range(4)
    ]
    lid = object_model.get_part("top_lid")
    lid_hinge = object_model.get_articulation("lid_hinge")

    # --- Intentional overlaps: hinge barrels embed into frame edges --------
    lower_frame_elems = ["side_wall_0", "side_wall_1"]
    for door, elem in zip(lower_doors, lower_frame_elems):
        ctx.allow_overlap(
            door,
            body,
            elem_a="hinge_barrel",
            elem_b=elem,
            reason="Hinge barrel knuckles intentionally lap the frame edge they pivot on.",
        )

    upper_frame_elems = [
        "side_wall_0",
        "upper_stile_0",
        "upper_stile_2",
        "side_wall_1",
    ]
    for door, elem in zip(upper_doors, upper_frame_elems):
        ctx.allow_overlap(
            door,
            body,
            elem_a="hinge_barrel",
            elem_b=elem,
            reason="Hinge barrel knuckles intentionally lap the frame edge they pivot on.",
        )

    # --- Overall envelope --------------------------------------------------
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
        ctx.check("legs rest on the floor", abs(z0) <= 1e-6, details=f"zmin={z0:.5f}")

    # --- Glass-framed upper doors exist ------------------------------------
    for i, door in enumerate(upper_doors):
        glass_aabb = ctx.part_element_world_aabb(door, elem="glass_pane")
        ctx.check(
            f"upper_door_{i} has glass pane",
            glass_aabb is not None and glass_aabb[1][2] - glass_aabb[0][2] > 0.40,
            details=str(glass_aabb),
        )
        frame_aabb = ctx.part_element_world_aabb(door, elem="frame")
        ctx.check(
            f"upper_door_{i} has steel frame",
            frame_aabb is not None,
            details=str(frame_aabb),
        )

    # --- Solid lower doors -------------------------------------------------
    for i, door in enumerate(lower_doors):
        leaf_aabb = ctx.part_element_world_aabb(door, elem="leaf")
        ctx.check(
            f"lower_door_{i} is solid panel",
            leaf_aabb is not None
            and leaf_aabb[1][0] - leaf_aabb[0][0] > 0.50,
            details=str(leaf_aabb),
        )
        vb = ctx.part_element_world_aabb(door, elem="vent_backing")
        ctx.check(
            f"lower_door_{i} has vent slot",
            vb is not None,
            details=str(vb),
        )

    # --- Pull handles on all doors ----------------------------------------
    all_doors = lower_doors + upper_doors
    for i, door in enumerate(all_doors):
        bar_aabb = ctx.part_element_world_aabb(door, elem="handle_bar")
        ctx.check(
            f"door {i} has pull handle bar",
            bar_aabb is not None,
            details=str(bar_aabb),
        )

    # --- Visible hinge barrels on all doors --------------------------------
    for i, door in enumerate(all_doors):
        hb = ctx.part_element_world_aabb(door, elem="hinge_barrel")
        ctx.check(
            f"door {i} has visible hinge barrel",
            hb is not None and hb[1][2] - hb[0][2] > 0.30,
            details=str(hb),
        )

    # --- Door hinges: revolute, vertical axis, correct range ---------------
    all_hinges = lower_hinges + upper_hinges
    for i, hinge in enumerate(all_hinges):
        ctx.check(
            f"hinge_{i} is revolute",
            hinge.articulation_type == ArticulationType.REVOLUTE,
        )
        ax = hinge.axis
        ctx.check(
            f"hinge_{i} axis is vertical",
            abs(ax[0]) < 1e-9 and abs(ax[1]) < 1e-9 and abs(abs(ax[2]) - 1.0) < 1e-9,
            details=str(ax),
        )
        lim = hinge.motion_limits
        ctx.check(
            f"hinge_{i} range 0..~110 deg",
            lim is not None
            and lim.lower == 0.0
            and abs(lim.upper - DOOR_OPEN) < 1e-6,
        )

    # --- Top lid: exists, revolute, rear hinge, opens upward ---------------
    ctx.check(
        "lid_hinge is revolute",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE,
    )
    lid_ax = lid_hinge.axis
    ctx.check(
        "lid_hinge axis is horizontal (X)",
        abs(lid_ax[0] - 1.0) < 1e-9 and abs(lid_ax[1]) < 1e-9 and abs(lid_ax[2]) < 1e-9,
        details=str(lid_ax),
    )
    lid_lim = lid_hinge.motion_limits
    ctx.check(
        "lid opens 0..~85 deg",
        lid_lim is not None
        and lid_lim.lower == 0.0
        and abs(lid_lim.upper - LID_OPEN) < 1e-6,
    )
    # Hinge is at the rear of the cabinet
    ctx.check(
        "lid hinge is at the rear",
        lid_hinge.origin.xyz[1] < -0.20,
        details=str(lid_hinge.origin.xyz),
    )

    # Lid opening pose: front edge moves upward
    lid_closed = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: LID_OPEN}):
        lid_open = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward",
        lid_closed is not None
        and lid_open is not None
        and lid_open[1][2] > lid_closed[1][2] + 0.20,
        details=f"closed={lid_closed}, open={lid_open}",
    )

    # --- Doors open outward (pose check) -----------------------------------
    closed_l0 = ctx.part_world_aabb(lower_doors[0])
    with ctx.pose({lower_hinges[0]: DOOR_OPEN}):
        open_l0 = ctx.part_world_aabb(lower_doors[0])
    ctx.check(
        "lower_door_0 swings outward when opened",
        closed_l0 is not None
        and open_l0 is not None
        and open_l0[1][1] > FRONT_Y + 0.20,
        details=f"closed={closed_l0}, open={open_l0}",
    )

    closed_u0 = ctx.part_world_aabb(upper_doors[0])
    with ctx.pose({upper_hinges[0]: DOOR_OPEN}):
        open_u0 = ctx.part_world_aabb(upper_doors[0])
    ctx.check(
        "upper_door_0 swings outward when opened",
        closed_u0 is not None
        and open_u0 is not None
        and open_u0[1][1] > FRONT_Y + 0.20,
        details=f"closed={closed_u0}, open={open_u0}",
    )

    # --- Mid rail separates upper and lower zones --------------------------
    mid_aabb = ctx.part_element_world_aabb(body, elem="front_mid_rail")
    ctx.check(
        "mid rail sits between lower and upper door zones",
        mid_aabb is not None
        and mid_aabb[0][2] > LOWER_ZC
        and mid_aabb[1][2] < UPPER_ZC,
        details=str(mid_aabb),
    )

    return ctx.report()


object_model = build_object_model()
