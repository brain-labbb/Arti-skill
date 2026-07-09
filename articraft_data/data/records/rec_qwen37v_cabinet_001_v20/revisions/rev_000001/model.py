from __future__ import annotations

"""Narrow bathroom wall cabinet with one mirrored door.

A compact wall-mounted steel cabinet (~0.40 m wide, 0.55 m tall, 0.14 m deep)
in brushed stainless steel. The hollow thin-wall carcass mounts flush to a
wall (back face at -Y). One full-front mirrored door swings open on side
barrel hinges at its left edge, revealing two interior shelf boards. The door
face has a recessed panel border (an inset rectangular frame) surrounding the
mirror. A small magnetic latch at the right edge retains the door when closed.
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
# Dimensions (meters). Cabinet centered on X, back against wall at -Y.
# ---------------------------------------------------------------------------
CAB_W = 0.40   # overall width  (X)
CAB_D = 0.14   # overall depth  (Y)
CAB_H = 0.55   # overall height (Z)
WALL_T = 0.012 # thin steel wall

# Cabinet floats on wall: bottom at 1.20 m from floor
MOUNT_Z = 1.20

FRONT_Y = CAB_D / 2.0    # +0.07
BACK_Y = -CAB_D / 2.0    # -0.07

# Door fills the front opening with small clearance
DOOR_CLEARANCE = 0.002
DOOR_W = CAB_W - 2 * WALL_T - 2 * DOOR_CLEARANCE  # ~0.372
DOOR_H = CAB_H - 2 * WALL_T - 2 * DOOR_CLEARANCE  # ~0.522
DOOR_T = 0.010  # door panel thickness

# Recessed panel border on door face
BORDER_INSET = 0.025   # how far the border frame sits from door edges
BORDER_WIDTH = 0.012   # width of the frame rail
BORDER_DEPTH = 0.003   # how deep the recess is

# Mirror is slightly smaller than the border inner opening
MIRROR_INSET = 0.002

# Interior shelves
SHELF_T = 0.008
SHELF_COUNT = 2

# Hinge
HINGE_BARREL_R = 0.005
HINGE_KNUCKLE_R = 0.007
HINGE_LEN = 0.06

DOOR_OPEN_ANGLE = math.radians(110.0)


def _door_panel_solid(mesh_name: str):
    """Door leaf as one CadQuery solid: flat panel with a recessed
    rectangular border frame cut into the front face. The door panel
    extends along +X from the hinge line (local origin at hinge edge)."""
    xc = DOOR_W / 2.0
    panel = (
        cq.Workplane("XY")
        .box(DOOR_W, DOOR_T, DOOR_H)
        .translate((xc, 0.0, 0.0))
    )
    # Recessed border: cut a rectangular groove on the front face (+Y side)
    # Outer border rect (inset from edges)
    outer_w = DOOR_W - 2 * BORDER_INSET
    outer_h = DOOR_H - 2 * BORDER_INSET
    # Inner opening of the border frame
    inner_w = outer_w - 2 * BORDER_WIDTH
    inner_h = outer_h - 2 * BORDER_WIDTH
    # Cut the recess as a rectangular pocket on the front face
    recess_cutter = (
        cq.Workplane("XY")
        .box(outer_w, BORDER_DEPTH, outer_h)
        .translate((xc, DOOR_T / 2.0 - BORDER_DEPTH / 2.0 + 0.0001, 0.0))
    )
    # Add back the inner raised area (mirror sits here, proud of the recess)
    inner_raise = (
        cq.Workplane("XY")
        .box(inner_w, BORDER_DEPTH, inner_h)
        .translate((xc, DOOR_T / 2.0 - BORDER_DEPTH / 2.0 + 0.0001, 0.0))
    )
    result = panel.cut(recess_cutter).union(inner_raise)
    return mesh_from_cadquery(result, mesh_name)


def _hinge_barrel_solid(mesh_name: str):
    """Small barrel hinge knuckle column along local Z axis."""
    barrel = (
        cq.Workplane("XY")
        .circle(HINGE_BARREL_R)
        .extrude(HINGE_LEN / 2.0, both=True)
    )
    # Two knuckle rings
    for dz in (-0.015, 0.015):
        ring = (
            cq.Workplane("XY")
            .circle(HINGE_KNUCKLE_R)
            .extrude(0.012 / 2.0, both=True)
            .translate((0.0, 0.0, dz))
        )
        barrel = barrel.union(ring)
    return mesh_from_cadquery(barrel, mesh_name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bathroom_wall_cabinet")

    # Materials
    steel_body = model.material("steel_body", rgba=(0.72, 0.73, 0.75, 1.0))
    steel_door = model.material("steel_door", rgba=(0.68, 0.69, 0.71, 1.0))
    steel_trim = model.material("steel_trim", rgba=(0.58, 0.59, 0.61, 1.0))
    steel_shelf = model.material("steel_shelf", rgba=(0.76, 0.77, 0.78, 1.0))
    mirror_mat = model.material("mirror", rgba=(0.88, 0.92, 0.95, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.12, 0.12, 0.14, 1.0))
    steel_knob = model.material("steel_knob", rgba=(0.30, 0.31, 0.33, 1.0))

    # ------------------------------------------------------------------
    # Cabinet body: hollow thin-wall carcass, wall-mounted
    # ------------------------------------------------------------------
    body = model.part("cabinet_body")
    body_zc = MOUNT_Z + CAB_H / 2.0

    # Side walls
    for sx, vname in ((-1.0, "side_wall_left"), (1.0, "side_wall_right")):
        body.visual(
            Box((WALL_T, CAB_D, CAB_H)),
            origin=Origin(xyz=(sx * (CAB_W / 2.0 - WALL_T / 2.0), 0.0, body_zc)),
            material=steel_body,
            name=vname,
        )
    # Back wall (against wall)
    body.visual(
        Box((CAB_W - 2 * WALL_T, WALL_T, CAB_H - 2 * WALL_T)),
        origin=Origin(xyz=(0.0, BACK_Y + WALL_T / 2.0, body_zc)),
        material=steel_body,
        name="back_wall",
    )
    # Top and bottom panels
    body.visual(
        Box((CAB_W, CAB_D, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, MOUNT_Z + WALL_T / 2.0)),
        material=steel_body,
        name="bottom_panel",
    )
    body.visual(
        Box((CAB_W, CAB_D, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, MOUNT_Z + CAB_H - WALL_T / 2.0)),
        material=steel_body,
        name="top_panel",
    )
    # Front frame rails (thin strips behind the door, at the inner face)
    # Positioned behind the closed door leaf to avoid overlap.
    rail_depth = WALL_T
    rail_z_top = MOUNT_Z + CAB_H - WALL_T - rail_depth / 2.0
    rail_z_bot = MOUNT_Z + WALL_T + rail_depth / 2.0
    rail_y_center = FRONT_Y - DOOR_T - rail_depth / 2.0 - 0.001
    body.visual(
        Box((CAB_W - 2 * WALL_T, rail_depth, rail_depth)),
        origin=Origin(xyz=(0.0, rail_y_center, rail_z_top)),
        material=steel_trim,
        name="front_top_rail",
    )
    body.visual(
        Box((CAB_W - 2 * WALL_T, rail_depth, rail_depth)),
        origin=Origin(xyz=(0.0, rail_y_center, rail_z_bot)),
        material=steel_trim,
        name="front_bottom_rail",
    )

    # Interior shelves (fixed to body, visible through open door)
    # Shelf width slightly exceeds inner opening to overlap side walls (connectivity).
    shelf_w = CAB_W - 2 * WALL_T + 0.002
    shelf_d = CAB_D - WALL_T + 0.002  # overlaps back wall
    for i in range(SHELF_COUNT):
        shelf_z = MOUNT_Z + WALL_T + (i + 1) * (CAB_H - 2 * WALL_T) / (SHELF_COUNT + 1)
        body.visual(
            Box((shelf_w, shelf_d, SHELF_T)),
            origin=Origin(xyz=(0.0, 0.0, shelf_z)),
            material=steel_shelf,
            name=f"shelf_{i}",
        )

    # Wall-mounting plate on the back (visible from rear)
    body.visual(
        Box((0.20, 0.005, 0.10)),
        origin=Origin(xyz=(0.0, BACK_Y - 0.002, body_zc + 0.10)),
        material=steel_trim,
        name="mounting_plate",
    )

    # ------------------------------------------------------------------
    # Mirrored door - hinged at left edge
    # ------------------------------------------------------------------
    door = model.part("door")
    # Door part origin sits at hinge line (left edge of opening, front face)
    # Panel extends along +X from the hinge
    door_zc = MOUNT_Z + CAB_H / 2.0  # door vertical center in world

    # Door leaf with recessed panel border (CadQuery solid)
    door.visual(
        _door_panel_solid("door_panel"),
        material=steel_door,
        name="leaf",
    )

    # Mirror panel (slightly inset in the recessed border opening)
    mirror_w = DOOR_W - 2 * BORDER_INSET - 2 * BORDER_WIDTH - 2 * MIRROR_INSET
    mirror_h = DOOR_H - 2 * BORDER_INSET - 2 * BORDER_WIDTH - 2 * MIRROR_INSET
    mirror_xc = DOOR_W / 2.0
    door.visual(
        Box((mirror_w, 0.003, mirror_h)),
        origin=Origin(xyz=(mirror_xc, DOOR_T / 2.0 + 0.001, 0.0)),
        material=mirror_mat,
        name="mirror",
    )

    # Hinge barrel on left edge of door
    door.visual(
        _hinge_barrel_solid("hinge_barrel"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=steel_trim,
        name="hinge_barrel",
    )

    # Small pull knob on right edge (for opening)
    door.visual(
        Cylinder(radius=0.008, length=0.012),
        origin=Origin(xyz=(DOOR_W - 0.02, DOOR_T / 2.0 + 0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_knob,
        name="pull_knob",
    )

    # Articulation: door hinges at left edge of cabinet opening
    hinge_x = -(CAB_W / 2.0 - WALL_T) + 0.001
    hinge_y = FRONT_Y
    hinge_z = door_zc

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, hinge_z)),
        # +Z axis, positive rotation swings door outward (+Y) and to the left
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=2.0, lower=0.0, upper=DOOR_OPEN_ANGLE
        ),
    )

    # ------------------------------------------------------------------
    # Magnetic latch catch on body (right side of opening)
    # ------------------------------------------------------------------
    latch = model.part("latch_catch")
    latch.visual(
        Box((0.015, 0.008, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=steel_dark,
        name="catch_plate",
    )
    latch.visual(
        Cylinder(radius=0.003, length=0.006),
        origin=Origin(xyz=(0.0, 0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_knob,
        name="magnet_boss",
    )
    # Mount latch on the body front-right inner edge
    # Latch sits against the inner face of the right side wall near front
    # Position so catch plate overlaps slightly with side wall for connectivity
    latch_x = CAB_W / 2.0 - WALL_T - 0.0065
    latch_y = FRONT_Y - DOOR_T - 0.004
    latch_z = door_zc

    model.articulation(
        "latch_mount",
        ArticulationType.FIXED,
        parent=body,
        child=latch,
        origin=Origin(xyz=(latch_x, latch_y, latch_z)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("cabinet_body")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("door_hinge")
    latch = object_model.get_part("latch_catch")

    # Intentional overlap: hinge barrel embeds into the body side wall edge
    ctx.allow_overlap(
        door,
        body,
        elem_a="hinge_barrel",
        elem_b="side_wall_left",
        reason="Barrel hinge knuckle intentionally laps the fixed frame edge it pivots on.",
    )

    # --- Cabinet body proportions and wall-mount position ---
    aabb = ctx.part_world_aabb(body)
    ok = aabb is not None
    ctx.check("cabinet body has bounds", ok, details=str(aabb))
    if aabb is not None:
        (x0, y0, z0), (x1, y1, z1) = aabb
        width = x1 - x0
        height = z1 - z0
        depth = y1 - y0
        ctx.check(
            "narrow cabinet width ~0.40 m",
            0.38 <= width <= 0.44,
            details=f"width={width:.3f}",
        )
        ctx.check(
            "shallow cabinet depth ~0.14 m",
            0.12 <= depth <= 0.18,
            details=f"depth={depth:.3f}",
        )
        ctx.check(
            "cabinet height ~0.55 m",
            0.52 <= height <= 0.60,
            details=f"height={height:.3f}",
        )
        ctx.check(
            "cabinet is wall-mounted (bottom above floor)",
            z0 > 1.0,
            details=f"z_min={z0:.3f}",
        )

    # --- Door hinge is revolute about vertical axis ---
    ctx.check(
        "door hinge is revolute",
        hinge.articulation_type == ArticulationType.REVOLUTE,
    )
    ax = hinge.axis
    ctx.check(
        "door hinge axis is vertical (Z)",
        abs(ax[0]) < 1e-9 and abs(ax[1]) < 1e-9 and abs(abs(ax[2]) - 1.0) < 1e-9,
        details=str(ax),
    )
    lim = hinge.motion_limits
    ctx.check(
        "door opens 0 to ~110 degrees",
        lim is not None and lim.lower == 0.0 and lim.upper > math.radians(90.0),
        details=f"lower={lim.lower}, upper={lim.upper}",
    )

    # --- Hinge is on the left side (negative X) ---
    ctx.check(
        "door hinge is on the left edge",
        hinge.origin.xyz[0] < -0.10,
        details=f"hinge_x={hinge.origin.xyz[0]:.3f}",
    )

    # --- Door closed: flush with front face ---
    door_aabb = ctx.part_element_world_aabb(door, elem="leaf")
    ctx.check(
        "door closed leaf near front face",
        door_aabb is not None and door_aabb[1][1] > FRONT_Y - 0.01,
        details=str(door_aabb),
    )

    # --- Door opens outward ---
    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({hinge: DOOR_OPEN_ANGLE}):
        open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door swings outward past front face",
        open_aabb is not None and open_aabb[1][1] > FRONT_Y + 0.10,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )
    # The door's right edge should swing leftward when opening (toward -X)
    ctx.check(
        "door free edge swings away from cabinet when opened",
        open_aabb is not None and closed_aabb is not None
        and open_aabb[0][0] < closed_aabb[0][0] - 0.05,
        details=f"closed_min_x={closed_aabb[0][0]:.3f}, open_min_x={open_aabb[0][0]:.3f}",
    )

    # --- Mirror panel present on door ---
    mirror_aabb = ctx.part_element_world_aabb(door, elem="mirror")
    ctx.check(
        "mirror panel is on the door front face",
        mirror_aabb is not None and mirror_aabb[1][1] > door_aabb[1][1] - 0.005,
        details=str(mirror_aabb),
    )
    ctx.check(
        "mirror covers most of the door area",
        mirror_aabb is not None
        and (mirror_aabb[1][0] - mirror_aabb[0][0]) > 0.25
        and (mirror_aabb[1][2] - mirror_aabb[0][2]) > 0.35,
        details=f"mirror_w={mirror_aabb[1][0] - mirror_aabb[0][0]:.3f}, mirror_h={mirror_aabb[1][2] - mirror_aabb[0][2]:.3f}",
    )

    # --- Shelves visible inside cabinet ---
    for i in range(SHELF_COUNT):
        shelf_aabb = ctx.part_element_world_aabb(body, elem=f"shelf_{i}")
        ctx.check(
            f"shelf_{i} exists inside cabinet",
            shelf_aabb is not None
            and shelf_aabb[0][0] > -CAB_W / 2.0
            and shelf_aabb[1][0] < CAB_W / 2.0
            and shelf_aabb[0][2] > MOUNT_Z
            and shelf_aabb[1][2] < MOUNT_Z + CAB_H,
            details=str(shelf_aabb),
        )

    # --- Shelves are within the cabinet interior ---
    for i in range(SHELF_COUNT):
        ctx.expect_within(
            body,
            body,
            axes="x",
            inner_elem=f"shelf_{i}",
            outer_elem="bottom_panel",
            margin=0.01,
            name=f"shelf_{i} is within cabinet width",
        )
    # Verify shelves are at different heights (distinct shelf boards)
    shelf_positions = []
    for i in range(SHELF_COUNT):
        shelf_aabb = ctx.part_element_world_aabb(body, elem=f"shelf_{i}")
        if shelf_aabb is not None:
            shelf_positions.append(0.5 * (shelf_aabb[0][2] + shelf_aabb[1][2]))
    if len(shelf_positions) >= 2:
        ctx.check(
            "shelves are at different heights",
            abs(shelf_positions[1] - shelf_positions[0]) > 0.05,
            details=f"shelf_z_centers={shelf_positions}",
        )

    # --- Door stays within cabinet width when closed ---
    ctx.expect_within(
        door,
        body,
        axes="x",
        margin=0.005,
        name="door fits within cabinet width when closed",
    )

    # --- Latch catch is fixed and positioned on the right side ---
    latch_joint = object_model.get_articulation("latch_mount")
    ctx.check(
        "latch is a fixed joint",
        latch_joint.articulation_type == ArticulationType.FIXED,
    )
    latch_aabb = ctx.part_world_aabb(latch)
    ctx.check(
        "latch catch is on the right side of the opening",
        latch_aabb is not None and latch_aabb[0][0] > 0.10,
        details=str(latch_aabb),
    )

    return ctx.report()


object_model = build_object_model()
