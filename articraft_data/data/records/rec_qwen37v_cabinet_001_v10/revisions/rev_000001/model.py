from __future__ import annotations

"""Narrow bathroom wall cabinet with mirrored door, sliding drawer, and interior shelves.

Variant 10 of the vintage industrial steel locker family. This is a narrow
wall-mounted bathroom cabinet (~0.45 m wide x 0.65 m tall x 0.15 m deep) in
brushed steel. The upper compartment has a single mirrored door hinged on the
left edge (revolute, vertical axis, opens outward 0..110 deg). The lower
compartment has one drawer on a prismatic slide (pulls out toward the viewer,
0..0.10 m). Two interior shelves are visible through the open door. Thin dark
gap-seam frame strips surround the door and drawer openings.
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
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Global dimensions (meters). Cabinet centred on X, front face at +Y.
# ---------------------------------------------------------------------------
CAB_W = 0.45       # carcass width  (X)
CAB_D = 0.15       # carcass depth  (Y)
CAB_H = 0.65       # carcass height (Z)
WALL_T = 0.015     # thin steel wall
CAB_Z0 = 1.10      # bottom of cabinet (wall-mounted)

FRONT_Y = CAB_D / 2.0     # +0.075
BACK_Y = -CAB_D / 2.0     # -0.075

# Drawer zone (lower compartment)
DRAWER_ZONE_H = 0.13
DRAWER_Z0 = CAB_Z0 + WALL_T          # 1.115
DRAWER_Z1 = DRAWER_Z0 + DRAWER_ZONE_H  # 1.245
DRAWER_ZC = 0.5 * (DRAWER_Z0 + DRAWER_Z1)

# Horizontal divider panel between drawer and door compartments
DIVIDER_T = WALL_T
DIVIDER_ZC = DRAWER_Z1 + DIVIDER_T / 2.0  # 1.2525

# Door zone (upper compartment)
DOOR_Z0 = DRAWER_Z1 + DIVIDER_T      # 1.260
DOOR_Z1 = CAB_Z0 + CAB_H - WALL_T    # 1.735
DOOR_H = DOOR_Z1 - DOOR_Z0           # 0.475
DOOR_ZC = 0.5 * (DOOR_Z0 + DOOR_Z1)  # 1.4975

# Interior opening width (between inner side-wall faces)
OPEN_W = CAB_W - 2.0 * WALL_T        # 0.420
OPEN_LEFT = -OPEN_W / 2.0             # -0.210
OPEN_RIGHT = OPEN_W / 2.0             # +0.210

# Gap around moving fronts (door gap seams)
GAP = 0.003

# Door leaf
DOOR_W = OPEN_W - 2.0 * GAP           # 0.414
DOOR_T = WALL_T                        # 0.015
DOOR_PANEL_H = DOOR_H - 2.0 * GAP     # 0.469

# Drawer leaf / box
DRAWER_W = OPEN_W - 2.0 * GAP          # 0.414
DRAWER_PANEL_H = DRAWER_ZONE_H - 2.0 * GAP  # 0.124
DRAWER_FRONT_T = WALL_T                 # 0.015
DRAWER_SIDE_T = 0.010
DRAWER_DEPTH = CAB_D - 2.0 * WALL_T - 0.005  # 0.115
DRAWER_BOTTOM_T = 0.008

# Interior shelves
SHELF_T = 0.010
SHELF_W = OPEN_W - 0.004
SHELF_D = CAB_D - 2.0 * WALL_T - 0.004

# Door articulation
DOOR_OPEN = math.radians(110.0)

# Drawer articulation
DRAWER_TRAVEL = 0.10

# Frame seam dimensions
SEAM_W = 0.006   # seam strip width
SEAM_T = 0.003   # seam strip thickness (proud of front face)


def _mirror_door_mesh(mesh_name: str):
    """Door leaf: flat steel panel with a recessed mirror inset on the outer face."""
    # Outer panel
    panel = (
        cq.Workplane("XY")
        .box(DOOR_W, DOOR_T, DOOR_PANEL_H)
        .translate((DOOR_W / 2.0, -DOOR_T / 2.0, 0.0))
    )
    return mesh_from_cadquery(panel, mesh_name)


def _drawer_box_meshes():
    """Return (front_panel_mesh, side_mesh, bottom_mesh) for the drawer box."""
    front = (
        cq.Workplane("XY")
        .box(DRAWER_W, DRAWER_FRONT_T, DRAWER_PANEL_H)
        .translate((0.0, -DRAWER_FRONT_T / 2.0, 0.0))
    )
    # Side walls (one mesh, two boxes unioned)
    side_h = DRAWER_PANEL_H - DRAWER_BOTTOM_T - 0.005
    side_zc = -DRAWER_PANEL_H / 2.0 + DRAWER_BOTTOM_T + side_h / 2.0 + 0.0025
    side_y = -DRAWER_FRONT_T - DRAWER_DEPTH / 2.0
    left_side = (
        cq.Workplane("XY")
        .box(DRAWER_SIDE_T, DRAWER_DEPTH, side_h)
        .translate((-DRAWER_W / 2.0 + DRAWER_SIDE_T / 2.0, side_y, side_zc))
    )
    right_side = (
        cq.Workplane("XY")
        .box(DRAWER_SIDE_T, DRAWER_DEPTH, side_h)
        .translate((DRAWER_W / 2.0 - DRAWER_SIDE_T / 2.0, side_y, side_zc))
    )
    sides = left_side.union(right_side)
    # Back wall
    back = (
        cq.Workplane("XY")
        .box(DRAWER_W - 2.0 * DRAWER_SIDE_T, DRAWER_SIDE_T, side_h)
        .translate((0.0, -DRAWER_FRONT_T - DRAWER_DEPTH + DRAWER_SIDE_T / 2.0, side_zc))
    )
    sides = sides.union(back)
    # Bottom
    bottom_y = -DRAWER_FRONT_T - DRAWER_DEPTH / 2.0
    bottom = (
        cq.Workplane("XY")
        .box(DRAWER_W - 2.0 * DRAWER_SIDE_T, DRAWER_DEPTH, DRAWER_BOTTOM_T)
        .translate((0.0, bottom_y, -DRAWER_PANEL_H / 2.0 + DRAWER_BOTTOM_T / 2.0))
    )
    return (
        mesh_from_cadquery(front, "drawer_front"),
        mesh_from_cadquery(sides, "drawer_sides"),
        mesh_from_cadquery(bottom, "drawer_bottom"),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bathroom_wall_cabinet")

    # Materials
    steel_body = model.material("steel_body", rgba=(0.58, 0.59, 0.61, 1.0))
    steel_frame = model.material("steel_frame", rgba=(0.48, 0.49, 0.51, 1.0))
    steel_shelf = model.material("steel_shelf", rgba=(0.54, 0.55, 0.57, 1.0))
    steel_drawer = model.material("steel_drawer", rgba=(0.52, 0.53, 0.55, 1.0))
    mirror_mat = model.material("mirror_face", rgba=(0.85, 0.87, 0.90, 1.0))
    seam_mat = model.material("seam_dark", rgba=(0.12, 0.12, 0.14, 1.0))
    handle_mat = model.material("handle_steel", rgba=(0.35, 0.36, 0.38, 1.0))

    # ==================================================================
    # Cabinet body (root part): hollow shell + divider + shelves + seams
    # ==================================================================
    body = model.part("cabinet_body")

    # Side walls (full depth, full height)
    for sx, vname in ((-1.0, "side_wall_0"), (1.0, "side_wall_1")):
        body.visual(
            Box((WALL_T, CAB_D, CAB_H)),
            origin=Origin(xyz=(sx * (CAB_W / 2.0 - WALL_T / 2.0), 0.0, CAB_Z0 + CAB_H / 2.0)),
            material=steel_body,
            name=vname,
        )
    # Back wall
    body.visual(
        Box((CAB_W - WALL_T, WALL_T, CAB_H - 0.02)),
        origin=Origin(xyz=(0.0, BACK_Y + WALL_T / 2.0, CAB_Z0 + CAB_H / 2.0)),
        material=steel_body,
        name="back_wall",
    )
    # Bottom panel
    body.visual(
        Box((CAB_W, CAB_D, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, CAB_Z0 + WALL_T / 2.0)),
        material=steel_body,
        name="bottom_panel",
    )
    # Top panel
    body.visual(
        Box((CAB_W, CAB_D, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, CAB_Z0 + CAB_H - WALL_T / 2.0)),
        material=steel_body,
        name="top_panel",
    )
    # Horizontal divider between drawer and door compartments
    body.visual(
        Box((CAB_W - WALL_T, CAB_D - WALL_T, DIVIDER_T)),
        origin=Origin(xyz=(0.0, -WALL_T / 2.0, DIVIDER_ZC)),
        material=steel_body,
        name="divider_panel",
    )

    # Interior shelves (2 shelves in the upper compartment)
    shelf_z_positions = [
        DOOR_Z0 + DOOR_H / 3.0,
        DOOR_Z0 + 2.0 * DOOR_H / 3.0,
    ]
    for i, sz in enumerate(shelf_z_positions):
        body.visual(
            Box((SHELF_W, SHELF_D, SHELF_T)),
            origin=Origin(xyz=(0.0, -WALL_T / 2.0, sz)),
            material=steel_shelf,
            name=f"shelf_{i}",
        )

    # --- Door gap seam strips around the door opening ---
    # These are thin dark strips on the front face framing the door opening.
    seam_front_y = FRONT_Y + SEAM_T / 2.0  # slightly proud of front face

    # Top seam strip (above door opening)
    body.visual(
        Box((OPEN_W + 2.0 * SEAM_W, SEAM_T, SEAM_W)),
        origin=Origin(xyz=(0.0, seam_front_y, DOOR_Z1 + GAP + SEAM_W / 2.0)),
        material=seam_mat,
        name="door_seam_top",
    )
    # Bottom seam strip (below door opening)
    body.visual(
        Box((OPEN_W + 2.0 * SEAM_W, SEAM_T, SEAM_W)),
        origin=Origin(xyz=(0.0, seam_front_y, DOOR_Z0 - GAP - SEAM_W / 2.0)),
        material=seam_mat,
        name="door_seam_bottom",
    )
    # Left seam strip
    body.visual(
        Box((SEAM_W, SEAM_T, DOOR_PANEL_H + 2.0 * GAP + 2.0 * SEAM_W)),
        origin=Origin(xyz=(OPEN_LEFT - GAP - SEAM_W / 2.0, seam_front_y, DOOR_ZC)),
        material=seam_mat,
        name="door_seam_left",
    )
    # Right seam strip
    body.visual(
        Box((SEAM_W, SEAM_T, DOOR_PANEL_H + 2.0 * GAP + 2.0 * SEAM_W)),
        origin=Origin(xyz=(OPEN_RIGHT + GAP + SEAM_W / 2.0, seam_front_y, DOOR_ZC)),
        material=seam_mat,
        name="door_seam_right",
    )

    # --- Drawer gap seam strips around the drawer opening ---
    # Top seam (above drawer opening)
    body.visual(
        Box((OPEN_W + 2.0 * SEAM_W, SEAM_T, SEAM_W)),
        origin=Origin(xyz=(0.0, seam_front_y, DRAWER_Z1 + GAP + SEAM_W / 2.0 - DIVIDER_T)),
        material=seam_mat,
        name="drawer_seam_top",
    )
    # Bottom seam (below drawer opening)
    body.visual(
        Box((OPEN_W + 2.0 * SEAM_W, SEAM_T, SEAM_W)),
        origin=Origin(xyz=(0.0, seam_front_y, DRAWER_Z0 - GAP - SEAM_W / 2.0 + WALL_T)),
        material=seam_mat,
        name="drawer_seam_bottom",
    )
    # Left seam
    body.visual(
        Box((SEAM_W, SEAM_T, DRAWER_PANEL_H + 2.0 * GAP + 2.0 * SEAM_W)),
        origin=Origin(xyz=(OPEN_LEFT - GAP - SEAM_W / 2.0, seam_front_y, DRAWER_ZC)),
        material=seam_mat,
        name="drawer_seam_left",
    )
    # Right seam
    body.visual(
        Box((SEAM_W, SEAM_T, DRAWER_PANEL_H + 2.0 * GAP + 2.0 * SEAM_W)),
        origin=Origin(xyz=(OPEN_RIGHT + GAP + SEAM_W / 2.0, seam_front_y, DRAWER_ZC)),
        material=seam_mat,
        name="drawer_seam_right",
    )

    # ==================================================================
    # Mirror door (revolute joint, hinged on left edge)
    # ==================================================================
    door = model.part("mirror_door")

    # Door leaf (steel panel)
    door.visual(
        _mirror_door_mesh("door_leaf"),
        material=steel_frame,
        name="leaf",
    )
    # Mirror inset on the outer face (slightly proud, lighter material)
    mirror_inset = 0.020
    mirror_w = DOOR_W - 2.0 * mirror_inset
    mirror_h = DOOR_PANEL_H - 2.0 * mirror_inset
    door.visual(
        Box((mirror_w, 0.003, mirror_h)),
        origin=Origin(xyz=(DOOR_W / 2.0, 0.001, 0.0)),
        material=mirror_mat,
        name="mirror",
    )
    # Small handle knob near the right (free) edge
    door.visual(
        Cylinder(radius=0.008, length=0.012),
        origin=Origin(
            xyz=(DOOR_W - 0.030, 0.006, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=handle_mat,
        name="door_handle",
    )

    # Door hinge articulation: left edge, vertical axis, positive q opens outward
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(OPEN_LEFT, FRONT_Y, DOOR_ZC)),
        # Panel extends +X from hinge; rotation around +Z swings free edge toward +Y (outward).
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=2.0, lower=0.0, upper=DOOR_OPEN,
        ),
    )

    # ==================================================================
    # Drawer (prismatic joint, slides out along +Y)
    # ==================================================================
    drawer = model.part("drawer")

    front_mesh, sides_mesh, bottom_mesh = _drawer_box_meshes()
    drawer.visual(front_mesh, material=steel_drawer, name="front_panel")
    drawer.visual(sides_mesh, material=steel_body, name="side_walls")
    drawer.visual(bottom_mesh, material=steel_body, name="bottom_panel")

    # Small handle on the drawer front (centered horizontal bar)
    drawer.visual(
        Box((0.060, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, 0.004, 0.0)),
        material=handle_mat,
        name="drawer_handle",
    )

    # Drawer slide articulation: prismatic along +Y (outward)
    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.0, FRONT_Y, DRAWER_ZC)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0, velocity=0.25, lower=0.0, upper=DRAWER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("cabinet_body")
    door = object_model.get_part("mirror_door")
    drawer = object_model.get_part("drawer")
    door_hinge = object_model.get_articulation("door_hinge")
    drawer_slide = object_model.get_articulation("drawer_slide")

    # --- Overall envelope -----------------------------------------------
    aabb = ctx.part_world_aabb(body)
    ok = aabb is not None
    ctx.check("cabinet body has bounds", ok, details=str(aabb))
    if aabb is not None:
        (x0, y0, z0), (x1, y1, z1) = aabb
        ctx.check(
            "cabinet width ~0.45 m",
            0.43 <= (x1 - x0) <= 0.48,
            details=f"width={x1 - x0:.3f}",
        )
        ctx.check(
            "cabinet depth ~0.15 m",
            0.13 <= (y1 - y0) <= 0.20,
            details=f"depth={y1 - y0:.3f}",
        )
        ctx.check(
            "cabinet height ~0.65 m",
            0.63 <= (z1 - z0) <= 0.68,
            details=f"height={z1 - z0:.3f}",
        )
        ctx.check(
            "wall-mounted bottom at ~1.10 m",
            1.08 <= z0 <= 1.12,
            details=f"z0={z0:.3f}",
        )

    # --- Door hinge: revolute, vertical axis, opens outward -------------
    ctx.check(
        "door hinge is revolute",
        door_hinge.articulation_type == ArticulationType.REVOLUTE,
    )
    ax = door_hinge.axis
    ctx.check(
        "door hinge axis is vertical (Z)",
        abs(ax[0]) < 1e-9 and abs(ax[1]) < 1e-9 and abs(abs(ax[2]) - 1.0) < 1e-9,
        details=str(ax),
    )
    lim = door_hinge.motion_limits
    ctx.check(
        "door hinge range 0..~110 deg",
        lim is not None
        and lim.lower == 0.0
        and abs(lim.upper - math.radians(110.0)) < 1e-6,
    )

    # Closed door sits flush at the front face
    door_aabb_closed = ctx.part_element_world_aabb(door, elem="leaf")
    ctx.check(
        "closed door leaf is at the front face",
        door_aabb_closed is not None
        and abs(door_aabb_closed[1][1] - FRONT_Y) < 0.005,
        details=str(door_aabb_closed),
    )

    # Open pose: door swings outward past front face
    with ctx.pose({door_hinge: DOOR_OPEN}):
        door_aabb_open = ctx.part_world_aabb(door)
    ctx.check(
        "open door swings outward past front face",
        door_aabb_open is not None
        and door_aabb_open[1][1] > FRONT_Y + 0.10,
        details=str(door_aabb_open),
    )

    # --- Drawer slide: prismatic, +Y axis, extends outward -------------
    ctx.check(
        "drawer slide is prismatic",
        drawer_slide.articulation_type == ArticulationType.PRISMATIC,
    )
    dax = drawer_slide.axis
    ctx.check(
        "drawer slide axis is +Y (outward)",
        abs(dax[0]) < 1e-9 and abs(dax[1] - 1.0) < 1e-9 and abs(dax[2]) < 1e-9,
        details=str(dax),
    )
    dlim = drawer_slide.motion_limits
    ctx.check(
        "drawer slide range 0..0.10 m",
        dlim is not None
        and dlim.lower == 0.0
        and abs(dlim.upper - DRAWER_TRAVEL) < 1e-6,
    )

    # Closed drawer front at front face
    drawer_closed = ctx.part_element_world_aabb(drawer, elem="front_panel")
    ctx.check(
        "closed drawer front is at the front face",
        drawer_closed is not None
        and abs(drawer_closed[1][1] - FRONT_Y) < 0.005,
        details=str(drawer_closed),
    )

    # Extended drawer moves forward
    with ctx.pose({drawer_slide: DRAWER_TRAVEL}):
        drawer_open = ctx.part_element_world_aabb(drawer, elem="front_panel")
    ctx.check(
        "extended drawer front moves outward",
        drawer_open is not None
        and drawer_closed is not None
        and drawer_open[1][1] > drawer_closed[1][1] + 0.05,
        details=f"closed={drawer_closed}, open={drawer_open}",
    )

    # --- Interior shelves present ---------------------------------------
    for i in range(2):
        shelf_aabb = ctx.part_element_world_aabb(body, elem=f"shelf_{i}")
        ctx.check(
            f"shelf_{i} exists inside upper compartment",
            shelf_aabb is not None
            and shelf_aabb[0][2] > DOOR_Z0
            and shelf_aabb[1][2] < DOOR_Z1,
            details=str(shelf_aabb),
        )

    # --- Door gap seam strips present around openings -------------------
    for seam_name in ("door_seam_top", "door_seam_bottom", "door_seam_left", "door_seam_right"):
        saabb = ctx.part_element_world_aabb(body, elem=seam_name)
        ctx.check(
            f"gap seam strip {seam_name} present at front face",
            saabb is not None and saabb[1][1] > FRONT_Y,
            details=str(saabb),
        )
    for seam_name in ("drawer_seam_top", "drawer_seam_bottom", "drawer_seam_left", "drawer_seam_right"):
        saabb = ctx.part_element_world_aabb(body, elem=seam_name)
        ctx.check(
            f"gap seam strip {seam_name} present at front face",
            saabb is not None and saabb[1][1] > FRONT_Y,
            details=str(saabb),
        )

    # --- Mirror surface on the door ------------------------------------
    mirror_aabb = ctx.part_element_world_aabb(door, elem="mirror")
    ctx.check(
        "mirror inset present on door outer face",
        mirror_aabb is not None
        and mirror_aabb[1][1] > FRONT_Y - 0.002,
        details=str(mirror_aabb),
    )

    # --- Drawer stays within cabinet width when closed ------------------
    ctx.expect_within(
        drawer,
        body,
        axes="x",
        margin=0.010,
        name="drawer stays inside the cabinet width when closed",
    )

    return ctx.report()


object_model = build_object_model()
