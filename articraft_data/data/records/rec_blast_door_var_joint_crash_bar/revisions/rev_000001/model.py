from __future__ import annotations

"""Heavy industrial blast door set into a concrete wall.

Matches picture/Industrial/Blast door/001.png:
- board-formed concrete wall with a dark steel door frame,
- matte black steel door leaf hung on three barrel hinges along its left edge,
- small rectangular viewing window with a raised black bezel,
- round blue emergency button, keyed lock cylinder with silver escutcheon,
- horizontal stainless push crash-bar spanning most of the leaf width,
  mounted on two end brackets that slide into door-face guides.

Articulations:
- ``door_hinge`` (REVOLUTE): the leaf swings open toward the viewer (-Y).
- ``push_bar`` (PRISMATIC): the crash bar translates inward (+Y, toward the
  door face) to retract the panic latch.
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

# Push crash-bar dimensions (door-local frame).
BAR_LENGTH = 0.60  # tube spans most of the leaf width
BAR_RADIUS = 0.020  # 20 mm stainless tube
BAR_STANDOFF = 0.055  # bar centre stands 55 mm proud of the leaf front face
BAR_CX = 0.54  # bar centre X in door-local (≈ centre of leaf)
BAR_CZ = HANDLE_Z  # same height as the rose
BRACKET_DX = 0.25  # bracket X offset from bar centre
BRACKET_W = 0.045  # bracket stem width (X)
BRACKET_D = 0.030  # bracket stem depth (Y, toward door face)
BRACKET_H = 0.055  # bracket stem height (Z)
SOCKET_W = 0.050  # door-face socket post width (slightly wider than bracket)
SOCKET_D = 0.026  # socket post protrusion from door face (Y, toward bar)
SOCKET_H = 0.060  # socket post height (slightly taller than bracket)
PUSH_TRAVEL = 0.025  # max inward travel of the bar (metres)

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

    # Handle rose plate (mounting disc behind the crash-bar mechanism).
    door.visual(
        Cylinder(radius=0.032, length=0.012),
        origin=Origin(
            xyz=(HANDLE_X, LEAF_FRONT - 0.004, HANDLE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)
        ),
        material=STAINLESS,
        name="handle_rose",
    )

    # Door-face socket posts: two steel posts protruding from the leaf front
    # face that capture the crash-bar bracket stems in a sliding fit.
    for i, sign in enumerate((-1, 1)):
        gx = BAR_CX + sign * BRACKET_DX
        door.visual(
            Box((SOCKET_W, SOCKET_D, SOCKET_H)),
            origin=Origin(xyz=(gx, LEAF_FRONT - SOCKET_D / 2.0, BAR_CZ)),
            material=FRAME_STEEL,
            name=f"bar_socket_{i}",
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

    # ------------------------------------------------------- crash bar
    push = model.part("push_bar")

    # Horizontal push-bar tube spanning most of the leaf width.
    push.visual(
        Cylinder(radius=BAR_RADIUS, length=BAR_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=STAINLESS,
        name="bar_tube",
    )

    # Two end brackets that tie the bar to the door-face guides.
    for i, sign in enumerate((-1, 1)):
        push.visual(
            Box((BRACKET_W, BRACKET_D, BRACKET_H)),
            origin=Origin(xyz=(sign * BRACKET_DX, BRACKET_D / 2.0, 0.0)),
            material=STAINLESS,
            name=f"bracket_{i}",
        )

    model.articulation(
        "push_bar",
        ArticulationType.PRISMATIC,
        parent=door,
        child=push,
        # Origin at bar centre, standing proud of the leaf front face.
        origin=Origin(xyz=(BAR_CX, LEAF_FRONT - BAR_STANDOFF, BAR_CZ)),
        # Axis +Y (door-local): positive q pushes the bar inward toward the
        # door face, retracting the panic latch.
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0, velocity=0.5, lower=0.0, upper=PUSH_TRAVEL
        ),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wall = object_model.get_part("wall_frame")
    door = object_model.get_part("door_leaf")
    push = object_model.get_part("push_bar")
    hinge = object_model.get_articulation("door_hinge")
    push_joint = object_model.get_articulation("push_bar")

    # Frame-side hinge tabs wrap the knuckle barrels on the pivot axis.
    for i in range(3):
        ctx.allow_overlap(
            wall,
            door,
            elem_a=f"hinge_tab_{i}",
            elem_b=f"hinge_barrel_{i}",
            reason="frame hinge lug wraps the knuckle barrel on the pivot axis",
        )

    # The bracket stems on the push_bar slide into the socket posts on the
    # door face, creating an intentional captured overlap that represents the
    # panic-bar sliding fit at every pose in the travel range.
    for i in range(2):
        ctx.allow_overlap(
            door,
            push,
            elem_a=f"bar_socket_{i}",
            elem_b=f"bracket_{i}",
            reason="crash-bar bracket stem slides inside the door-mounted socket post",
        )

    # ---- closed pose: leaf covers the framed opening, bar at rest
    with ctx.pose({hinge: 0.0, push_joint: 0.0}):
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

        # Bar tube at rest is well proud of the leaf front face.
        ctx.expect_gap(
            door,
            push,
            axis="y",
            positive_elem="leaf_plate",
            negative_elem="bar_tube",
            min_gap=0.020,
            name="push_bar_tube_clears_leaf_front_at_rest",
        )
        # Bar spans most of the leaf width in X.
        ctx.expect_overlap(
            push,
            door,
            axes="x",
            elem_a="bar_tube",
            elem_b="leaf_plate",
            min_overlap=0.50,
            name="push_bar_tube_spans_most_of_leaf_width",
        )
        # At rest the bracket stems engage the socket posts (captured fit).
        for i in range(2):
            ctx.expect_overlap(
                door,
                push,
                elem_a=f"bar_socket_{i}",
                elem_b=f"bracket_{i}",
                axes="y",
                min_overlap=0.0005,
                name=f"bracket_{i}_captured_in_socket_at_rest",
            )

        closed_aabb = ctx.part_world_aabb(door)
        bar_rest_aabb = ctx.part_element_world_aabb(push, elem="bar_tube")

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

    # ---- push-bar pose: pressing the bar retracts it toward the door face
    with ctx.pose({push_joint: PUSH_TRAVEL}):
        bar_pushed_aabb = ctx.part_element_world_aabb(push, elem="bar_tube")
        ctx.check(
            "push_bar_translates_inward",
            bar_rest_aabb is not None
            and bar_pushed_aabb is not None
            and bar_pushed_aabb[0][1] > bar_rest_aabb[0][1] + 0.010,
            "push bar must translate toward the door face (+Y) when pressed",
        )
        # At max push the brackets still engage the door-face sockets.
        for i in range(2):
            ctx.expect_overlap(
                door,
                push,
                elem_a=f"bar_socket_{i}",
                elem_b=f"bracket_{i}",
                axes="xz",
                min_overlap=0.010,
                name=f"bracket_{i}_engages_socket_at_max_push",
            )

    return ctx.report()
