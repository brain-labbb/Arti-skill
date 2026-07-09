from __future__ import annotations

"""Heavy industrial blast door set into a concrete wall.

Vault-style variant with a spoked rotary handwheel latch:
- board-formed concrete wall with a dark steel door frame,
- matte black steel door leaf hung on three barrel hinges along its left edge,
- small rectangular viewing window with a raised black bezel,
- round blue emergency button, keyed lock cylinder with silver escutcheon,
- three-spoke stainless handwheel on a round rose, rotating in the leaf
  plane about the leaf-normal axis to retract the latch.

Articulations:
- ``door_hinge`` (REVOLUTE): the leaf swings open toward the viewer (-Y).
- ``handle_spindle`` (REVOLUTE): the handwheel rotates in the leaf plane
  (about the leaf-normal axis) to retract the latch.
"""

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------- dimensions
# World frame: X = along the wall, Y = wall thickness (viewer at -Y), Z = up.
WALL_W = 3.40
WALL_T = 0.24
WALL_H = 2.60

OPEN_W = 0.96  # clear opening width  (x in [-0.48, 0.48])
OPEN_H = 2.04  # clear opening height (z in [0, 2.04])

FRAME_OUT_W = 1.12  # steel frame outer width
FRAME_OUT_H = 2.12  # steel frame outer height
FRAME_PROUD = 0.02  # frame stands proud of the concrete front face (y=0)
FRAME_DEPTH = 0.12  # total frame depth (y in [-0.02, 0.10])

LEAF_W = 1.04
LEAF_H = 2.10
LEAF_T = 0.06

# Hinge axis (door pivot), just outside the frame's left edge, proud of it.
HINGE_X = -0.54
HINGE_Y = -0.056  # door mid-thickness plane; leaf back face clears frame front
HINGE_Z_LIST = [0.35, 1.05, 1.75]

# Door-local frame: origin on the hinge axis, leaf extends +X, front face at
# local y = -LEAF_T/2 (toward the viewer), z up from the floor.
LEAF_X0 = 0.02  # leaf left edge (local x)
LEAF_FRONT = -LEAF_T / 2.0

WINDOW_CX = 0.42  # window centre (local x)
WINDOW_CZ = 1.55
WINDOW_W = 0.32
WINDOW_H = 0.20

BUTTON_X = 0.70
BUTTON_Z = 1.58
LOCK_X = 0.80
LOCK_Z = 1.30
HANDLE_X = 0.80
HANDLE_Z = 1.05

# ------------------------------------------------------------------ materials
CONCRETE = Material(name="board_formed_concrete", rgba=(0.62, 0.61, 0.58, 1.0))
FRAME_STEEL = Material(name="frame_steel", rgba=(0.15, 0.16, 0.17, 1.0))
LEAF_STEEL = Material(name="leaf_steel", rgba=(0.10, 0.10, 0.11, 1.0))
BEZEL_BLACK = Material(name="bezel_black", rgba=(0.06, 0.06, 0.07, 1.0))
GLASS = Material(name="window_glass", rgba=(0.62, 0.72, 0.76, 0.45))
STAINLESS = Material(name="stainless", rgba=(0.74, 0.75, 0.77, 1.0))
BLUE_BUTTON = Material(name="blue_button", rgba=(0.10, 0.38, 0.85, 1.0))
HINGE_STEEL = Material(name="hinge_steel", rgba=(0.20, 0.21, 0.22, 1.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_blast_door")

    # -------------------------------------------------------- wall + frame
    wall = model.part("wall_frame")

    concrete = cq.Workplane("XY").box(
        WALL_W, WALL_T, WALL_H, centered=(True, False, False)
    )
    opening_cut = (
        cq.Workplane("XY")
        .box(OPEN_W, WALL_T + 0.02, OPEN_H + 0.01, centered=(True, False, False))
        .translate((0.0, -0.01, -0.005))
    )
    concrete = concrete.cut(opening_cut)
    wall.visual(
        mesh_from_cadquery(concrete, "concrete_wall"),
        material=CONCRETE,
        name="concrete_wall",
    )

    frame = (
        cq.Workplane("XY")
        .box(FRAME_OUT_W, FRAME_DEPTH, FRAME_OUT_H, centered=(True, False, False))
        .translate((0.0, -FRAME_PROUD, 0.0))
    )
    frame_cut = (
        cq.Workplane("XY")
        .box(OPEN_W, FRAME_DEPTH + 0.04, OPEN_H + 0.01, centered=(True, False, False))
        .translate((0.0, -FRAME_PROUD - 0.02, -0.005))
    )
    frame = frame.cut(frame_cut)
    wall.visual(
        mesh_from_cadquery(frame, "steel_frame"),
        material=FRAME_STEEL,
        name="steel_frame",
    )

    # Frame-side hinge tabs: short steel lugs cantilevered off the jamb face
    # that wrap the hinge knuckle barrels (placed above each door strap so
    # they never collide with the swinging straps).
    for i, hz in enumerate(HINGE_Z_LIST):
        wall.visual(
            Box((0.062, 0.057, 0.030)),
            origin=Origin(xyz=(-0.554, -0.0335, hz + 0.075)),
            material=FRAME_STEEL,
            name=f"hinge_tab_{i}",
        )

    # --------------------------------------------------------- door leaf
    door = model.part("door_leaf")

    leaf = (
        cq.Workplane("XY")
        .box(LEAF_W, LEAF_T, LEAF_H, centered=(True, True, False))
        .translate((LEAF_X0 + LEAF_W / 2.0, 0.0, 0.01))
    )
    window_cut = (
        cq.Workplane("XY")
        .box(WINDOW_W + 0.02, LEAF_T + 0.02, WINDOW_H + 0.02)
        .translate((WINDOW_CX, 0.0, WINDOW_CZ))
    )
    leaf = leaf.cut(window_cut)
    door.visual(
        mesh_from_cadquery(leaf, "door_leaf_plate"),
        material=LEAF_STEEL,
        name="leaf_plate",
    )

    # Raised black window bezel, passing through the leaf around the cutout.
    window_bezel = BezelGeometry(
        (WINDOW_W, WINDOW_H),
        (WINDOW_W + 0.10, WINDOW_H + 0.10),
        LEAF_T + 0.024,
        opening_shape="rect",
        outer_shape="rect",
    )
    door.visual(
        mesh_from_geometry(window_bezel, "window_bezel"),
        origin=Origin(xyz=(WINDOW_CX, 0.0, WINDOW_CZ), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=BEZEL_BLACK,
        name="window_bezel",
    )
    # Glass pane retained inside the bezel opening (slightly oversized so it
    # seats into the bezel walls).
    door.visual(
        Box((WINDOW_W + 0.03, 0.008, WINDOW_H + 0.03)),
        origin=Origin(xyz=(WINDOW_CX, 0.0, WINDOW_CZ)),
        material=GLASS,
        name="window_glass",
    )

    # Round blue emergency button, proud of the leaf front face.
    door.visual(
        Cylinder(radius=0.021, length=0.012),
        origin=Origin(
            xyz=(BUTTON_X, LEAF_FRONT - 0.004, BUTTON_Z), rpy=(math.pi / 2.0, 0.0, 0.0)
        ),
        material=BLUE_BUTTON,
        name="blue_button",
    )

    # Keyed lock cylinder: silver escutcheon disc plus key cylinder.
    door.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(
            xyz=(LOCK_X, LEAF_FRONT - 0.003, LOCK_Z), rpy=(math.pi / 2.0, 0.0, 0.0)
        ),
        material=STAINLESS,
        name="lock_escutcheon",
    )
    door.visual(
        Cylinder(radius=0.008, length=0.010),
        origin=Origin(
            xyz=(LOCK_X, LEAF_FRONT - 0.010, LOCK_Z), rpy=(math.pi / 2.0, 0.0, 0.0)
        ),
        material=FRAME_STEEL,
        name="lock_cylinder",
    )

    # Handle rose plate (the lever itself is a separate articulated part).
    door.visual(
        Cylinder(radius=0.032, length=0.012),
        origin=Origin(
            xyz=(HANDLE_X, LEAF_FRONT - 0.004, HANDLE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)
        ),
        material=STAINLESS,
        name="handle_rose",
    )

    # Three barrel hinges on the pivot axis: axisymmetric knuckle barrels on
    # the axis plus weld straps tying them to the leaf's left edge.
    for i, hz in enumerate(HINGE_Z_LIST):
        door.visual(
            Cylinder(radius=0.024, length=0.18),
            origin=Origin(xyz=(0.0, 0.0, hz)),
            material=HINGE_STEEL,
            name=f"hinge_barrel_{i}",
        )
        door.visual(
            Box((0.13, 0.014, 0.10)),
            origin=Origin(xyz=(0.055, LEAF_FRONT - 0.004, hz)),
            material=HINGE_STEEL,
            name=f"hinge_strap_{i}",
        )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=wall,
        child=door,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, 0.0)),
        # Axis -Z so positive q swings the free edge toward the viewer (-Y).
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=200.0, velocity=1.0, lower=0.0, upper=2.0),
    )

    # ------------------------------------------------------- handwheel latch
    lever = model.part("lever_handle")

    # Wheel geometry constants
    WHEEL_R = 0.080         # rim centerline radius (160 mm diameter)
    WHEEL_TUBE_R = 0.009    # rim tube cross-section radius
    HUB_R = 0.016           # spindle / hub radius
    SPOKE_R = 0.007         # spoke cross-section radius
    WHEEL_Y = -0.012        # wheel plane in lever-local Y (proud of rose)

    # Spindle hub: cylinder along Y, passing through the rose into the leaf.
    SPINDLE_LEN = 0.047
    SPINDLE_CY = WHEEL_Y + SPINDLE_LEN / 2.0
    spindle_hub = (
        cq.Workplane("XY")
        .circle(HUB_R)
        .extrude(SPINDLE_LEN)
        .translate((0, 0, -SPINDLE_LEN / 2.0))
        .rotate((0, 0, 0), (1, 0, 0), 90)
        .translate((0, SPINDLE_CY, 0))
    )
    lever.visual(
        mesh_from_cadquery(spindle_hub, "spindle_hub"),
        material=STAINLESS,
        name="spindle_hub",
    )

    # Wheel rim: annular cylinder (ring) in the XZ plane at y = WHEEL_Y.
    RIM_WIDTH = 2.0 * WHEEL_TUBE_R  # thickness along Y
    rim_outer_r = WHEEL_R + WHEEL_TUBE_R
    rim_inner_r = WHEEL_R - WHEEL_TUBE_R
    rim_solid = (
        cq.Workplane("XY")
        .circle(rim_outer_r)
        .circle(rim_inner_r)
        .extrude(RIM_WIDTH)
        .translate((0, 0, -RIM_WIDTH / 2.0))
        .rotate((0, 0, 0), (1, 0, 0), 90)
        .translate((0, WHEEL_Y, 0))
    )
    lever.visual(
        mesh_from_cadquery(rim_solid, "wheel_rim"),
        material=STAINLESS,
        name="wheel_rim",
    )

    # Three spokes at 120° intervals in the XZ plane, hub to rim.
    SPOKE_INNER = HUB_R - 0.002   # 2 mm embed into hub for connectivity
    SPOKE_OUTER = WHEEL_R - WHEEL_TUBE_R + 0.002  # 2 mm embed into rim
    SPOKE_LEN = SPOKE_OUTER - SPOKE_INNER
    SPOKE_MID_R = (SPOKE_INNER + SPOKE_OUTER) / 2.0

    for i in range(3):
        angle_deg = i * 120
        # Build spoke along +X, then rotate around Y for placement.
        spoke = (
            cq.Workplane("XY")
            .circle(SPOKE_R)
            .extrude(SPOKE_LEN)
            .translate((0, 0, -SPOKE_LEN / 2.0))
            .rotate((0, 0, 0), (0, 1, 0), 90)
            .translate((SPOKE_MID_R, 0, 0))
        )
        if angle_deg != 0:
            spoke = spoke.rotate((0, 0, 0), (0, 1, 0), angle_deg)
        spoke = spoke.translate((0, WHEEL_Y, 0))
        lever.visual(
            mesh_from_cadquery(spoke, f"wheel_spoke_{i}"),
            material=STAINLESS,
            name=f"wheel_spoke_{i}",
        )

    model.articulation(
        "handle_spindle",
        ArticulationType.REVOLUTE,
        parent=door,
        child=lever,
        origin=Origin(xyz=(HANDLE_X, LEAF_FRONT - 0.010, HANDLE_Z)),
        # Axis -Y (outward leaf normal): positive q rotates the handwheel
        # in the leaf XZ plane — vault-style rotary latch action.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=2.0, lower=0.0, upper=math.pi),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wall = object_model.get_part("wall_frame")
    door = object_model.get_part("door_leaf")
    lever = object_model.get_part("lever_handle")
    hinge = object_model.get_articulation("door_hinge")
    spindle = object_model.get_articulation("handle_spindle")

    # The handwheel spindle hub passes through the rose into the leaf bore.
    ctx.allow_overlap(
        door,
        lever,
        elem_a="handle_rose",
        elem_b="spindle_hub",
        reason="handwheel spindle hub passes through the rose plate bore",
    )
    ctx.allow_overlap(
        door,
        lever,
        elem_a="leaf_plate",
        elem_b="spindle_hub",
        reason="handwheel spindle hub extends into the leaf latch bore",
    )
    # Frame-side hinge tabs wrap the knuckle barrels on the pivot axis.
    for i in range(3):
        ctx.allow_overlap(
            wall,
            door,
            elem_a=f"hinge_tab_{i}",
            elem_b=f"hinge_barrel_{i}",
            reason="frame hinge lug wraps the knuckle barrel on the pivot axis",
        )

    # ---- closed pose: leaf covers the framed opening, just clear of frame
    with ctx.pose({hinge: 0.0, spindle: 0.0}):
        ctx.expect_overlap(door, wall, axes="xz", min_overlap=0.5)
        # Hinge engagement: each frame lug really wraps its knuckle barrel.
        for i in range(3):
            ctx.expect_overlap(
                wall,
                door,
                elem_a=f"hinge_tab_{i}",
                elem_b=f"hinge_barrel_{i}",
                axes="xy",
                min_overlap=0.003,
            )

        leaf_aabb = ctx.part_element_world_aabb(door, elem="leaf_plate")
        glass_aabb = ctx.part_element_world_aabb(door, elem="window_glass")
        bezel_aabb = ctx.part_element_world_aabb(door, elem="window_bezel")
        button_aabb = ctx.part_element_world_aabb(door, elem="blue_button")
        barrel_aabb = ctx.part_element_world_aabb(door, elem="hinge_barrel_1")

        ctx.check(
            "window_glass_inside_bezel",
            glass_aabb is not None
            and bezel_aabb is not None
            and glass_aabb[0][0] > bezel_aabb[0][0]
            and glass_aabb[1][0] < bezel_aabb[1][0]
            and glass_aabb[0][2] > bezel_aabb[0][2]
            and glass_aabb[1][2] < bezel_aabb[1][2],
            "window glass must sit inside the raised bezel outline",
        )
        ctx.check(
            "blue_button_proud_of_leaf_front",
            button_aabb is not None
            and leaf_aabb is not None
            and button_aabb[0][1] < leaf_aabb[0][1] + 1e-6,
            "blue button must stand proud of the leaf front face",
        )
        ctx.check(
            "leaf_back_clears_frame_front",
            leaf_aabb is not None
            and leaf_aabb[1][1] <= -FRAME_PROUD + 1e-4
            and leaf_aabb[1][1] >= -0.05,
            "closed leaf back face must sit just in front of the frame face",
        )
        ctx.check(
            "hinge_barrels_on_pivot_axis",
            barrel_aabb is not None
            and abs((barrel_aabb[0][0] + barrel_aabb[1][0]) / 2.0 - HINGE_X) < 0.005
            and abs((barrel_aabb[0][1] + barrel_aabb[1][1]) / 2.0 - HINGE_Y) < 0.005,
            "hinge barrels must be centred on the door pivot axis",
        )

        # Spindle hub sits within the rose footprint on XZ.
        ctx.expect_within(
            lever,
            door,
            axes="xz",
            inner_elem="spindle_hub",
            outer_elem="handle_rose",
            margin=0.001,
            name="spindle centered within rose",
        )

        # Wheel rim spans at least 120 mm in both X and Z (round wheel).
        rim_aabb = ctx.part_element_world_aabb(lever, elem="wheel_rim")
        spoke0_aabb = ctx.part_element_world_aabb(lever, elem="wheel_spoke_0")
        ctx.check(
            "wheel_rim_has_round_extent",
            rim_aabb is not None
            and (rim_aabb[1][0] - rim_aabb[0][0]) > 0.12
            and (rim_aabb[1][2] - rim_aabb[0][2]) > 0.12,
            "wheel rim must span at least 120 mm in both X and Z",
        )

        closed_aabb = ctx.part_world_aabb(door)

    # ---- open pose: the leaf really swings out toward the viewer (-Y)
    with ctx.pose({hinge: 1.3}):
        open_aabb = ctx.part_world_aabb(door)
        ctx.check(
            "door_swings_open_toward_viewer",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[0][1] < closed_aabb[0][1] - 0.5,
            "door leaf must swing well out of the wall plane toward -Y",
        )

    # ---- handwheel rotation: spoke 0 shifts in Z when the wheel turns 90°
    with ctx.pose({spindle: math.pi / 2.0}):
        spoke0_rotated = ctx.part_element_world_aabb(lever, elem="wheel_spoke_0")
        ctx.check(
            "handwheel_spoke_rotates_in_leaf_plane",
            spoke0_aabb is not None
            and spoke0_rotated is not None
            and abs(
                (spoke0_aabb[0][2] + spoke0_aabb[1][2]) / 2.0
                - (spoke0_rotated[0][2] + spoke0_rotated[1][2]) / 2.0
            )
            > 0.03,
            "spoke 0 Z-center must shift at least 30 mm when the handwheel "
            "rotates 90 degrees about the leaf-normal axis",
        )

    return ctx.report()
