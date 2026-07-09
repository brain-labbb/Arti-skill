from __future__ import annotations

"""Heavy industrial sliding blast door set into a concrete wall.

Variant of the side-hinged blast door converted to a horizontally sliding
configuration:
- board-formed concrete wall with a dark steel door frame,
- matte black steel door leaf sliding sideways on a head rail,
- two trolley hanger brackets welded to the leaf top edge, riding in a
  slotted C-channel rail above the opening,
- small rectangular viewing window with a raised black bezel,
- round blue emergency button, keyed lock cylinder with silver escutcheon,
- stainless lever handle on a round rose.

Articulations:
- ``door_slide`` (PRISMATIC): the leaf rolls sideways (+X) to clear the opening.
- ``handle_spindle`` (REVOLUTE): the lever rotates down to retract the latch.
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
    Sphere,
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

# Door Y position: leaf centre plane sits just in front of the frame face.
DOOR_Y = -(FRAME_PROUD + LEAF_T / 2.0)  # -0.05

LEAF_FRONT = -LEAF_T / 2.0  # front face in door-local Y

# Feature positions relative to the centred leaf (door-local origin at the
# slide joint, which sits at the floor on the opening centreline).
WINDOW_CX = -0.12   # window slightly left of centre on the leaf
WINDOW_CZ = 1.55
WINDOW_W = 0.32
WINDOW_H = 0.20

BUTTON_X = 0.16
BUTTON_Z = 1.58
LOCK_X = 0.26
LOCK_Z = 1.30
HANDLE_X = 0.26
HANDLE_Z = 1.05

# Head rail dimensions and position.
RAIL_L = 2.20
RAIL_W = 0.065
RAIL_H = 0.080
RAIL_WALL = 0.010
RAIL_SLOT_W = 0.015
RAIL_CX = 0.55   # centre X of rail (biased right for travel)
RAIL_CY = DOOR_Y  # rail and door share the same Y plane
RAIL_CZ = 2.18    # rail centre height

# Hanger positions on the leaf (loop-placed pair).
HANGER_X_LIST = [-0.30, 0.30]

# Slide travel: positive q moves the leaf +X to clear the opening.
SLIDE_TRAVEL = 1.20

# ------------------------------------------------------------------ materials
CONCRETE = Material(name="board_formed_concrete", rgba=(0.62, 0.61, 0.58, 1.0))
FRAME_STEEL = Material(name="frame_steel", rgba=(0.15, 0.16, 0.17, 1.0))
LEAF_STEEL = Material(name="leaf_steel", rgba=(0.10, 0.10, 0.11, 1.0))
BEZEL_BLACK = Material(name="bezel_black", rgba=(0.06, 0.06, 0.07, 1.0))
GLASS = Material(name="window_glass", rgba=(0.62, 0.72, 0.76, 0.45))
STAINLESS = Material(name="stainless", rgba=(0.74, 0.75, 0.77, 1.0))
BLUE_BUTTON = Material(name="blue_button", rgba=(0.10, 0.38, 0.85, 1.0))


def _build_head_rail() -> cq.Workplane:
    """Slotted rectangular tube track for the trolley hangers."""
    outer = cq.Workplane("XY").box(RAIL_L, RAIL_W, RAIL_H, centered=(True, True, True))
    hollow_w = RAIL_W - 2.0 * RAIL_WALL
    hollow_h = RAIL_H - 2.0 * RAIL_WALL
    hollow = cq.Workplane("XY").box(
        RAIL_L + 0.01, hollow_w, hollow_h, centered=(True, True, True)
    )
    # Slot cut through the bottom wall so hanger stems can enter the channel.
    slot_h = RAIL_WALL + 0.015
    slot = (
        cq.Workplane("XY")
        .box(RAIL_L + 0.01, RAIL_SLOT_W, slot_h, centered=(True, True, False))
        .translate((0.0, 0.0, -RAIL_H / 2.0))
    )
    rail = outer.cut(hollow).cut(slot)
    return rail.translate((RAIL_CX, RAIL_CY, RAIL_CZ))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_sliding_blast_door")

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

    # Head rail: slotted rectangular tube above the opening.
    wall.visual(
        mesh_from_cadquery(_build_head_rail(), "head_rail"),
        material=FRAME_STEEL,
        name="head_rail",
    )

    # Rail mounting brackets: gusset plates tying the rail to the wall face.
    for i, bx in enumerate([-0.15, 1.25]):
        wall.visual(
            Box((0.06, 0.045, 0.06)),
            origin=Origin(xyz=(bx, -0.0225, 2.15)),
            material=FRAME_STEEL,
            name=f"rail_bracket_{i}",
        )

    # --------------------------------------------------------- door leaf
    door = model.part("door_leaf")

    # Leaf plate centred on the opening (X = 0) at rest.
    leaf = (
        cq.Workplane("XY")
        .box(LEAF_W, LEAF_T, LEAF_H, centered=(True, True, False))
        .translate((0.0, 0.0, 0.01))
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
    # Glass pane retained inside the bezel opening.
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

    # Trolley hanger brackets: loop-placed pair welded to the leaf top edge,
    # each with a stem plate entering the rail slot and a trolley block inside
    # the rail channel.
    for i, hx in enumerate(HANGER_X_LIST):
        door.visual(
            Box((0.05, 0.012, 0.14)),
            origin=Origin(xyz=(hx, 0.0, 2.11)),
            material=FRAME_STEEL,
            name=f"hanger_stem_{i}",
        )
        door.visual(
            Box((0.08, 0.034, 0.03)),
            origin=Origin(xyz=(hx, 0.0, 2.167)),
            material=FRAME_STEEL,
            name=f"trolley_{i}",
        )

    # PRISMATIC slide joint: door rolls +X along the head rail.
    model.articulation(
        "door_slide",
        ArticulationType.PRISMATIC,
        parent=wall,
        child=door,
        origin=Origin(xyz=(0.0, DOOR_Y, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=300.0, velocity=0.25, lower=0.0, upper=SLIDE_TRAVEL
        ),
    )

    # ------------------------------------------------------- lever handle
    lever = model.part("lever_handle")
    # Spindle hub, embedding back through the rose into the leaf.
    lever.visual(
        Cylinder(radius=0.014, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=STAINLESS,
        name="spindle_hub",
    )
    # Lever arm pointing toward the closing edge (-X), in front of the rose.
    lever.visual(
        Box((0.16, 0.022, 0.030)),
        origin=Origin(xyz=(-0.085, -0.020, 0.0)),
        material=STAINLESS,
        name="lever_arm",
    )
    lever.visual(
        Sphere(radius=0.016),
        origin=Origin(xyz=(-0.165, -0.020, 0.0)),
        material=STAINLESS,
        name="lever_tip",
    )

    model.articulation(
        "handle_spindle",
        ArticulationType.REVOLUTE,
        parent=door,
        child=lever,
        origin=Origin(xyz=(HANDLE_X, LEAF_FRONT - 0.010, HANDLE_Z)),
        # Axis -Y so positive q presses the lever tip down.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=3.0, lower=0.0, upper=0.9),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wall = object_model.get_part("wall_frame")
    door = object_model.get_part("door_leaf")
    lever = object_model.get_part("lever_handle")
    slide = object_model.get_articulation("door_slide")
    spindle = object_model.get_articulation("handle_spindle")

    # Handle spindle hub passes through the rose into the leaf bore.
    ctx.allow_overlap(
        door,
        lever,
        reason="handle spindle hub engages through the rose into the leaf bore",
    )

    # Hanger stems pass through the rail slot; trolley blocks ride inside the
    # rail channel.  Scope the allowance to the named elements.
    for i in range(2):
        ctx.allow_overlap(
            wall,
            door,
            elem_a="head_rail",
            elem_b=f"hanger_stem_{i}",
            reason=f"hanger stem {i} enters the rail slot to suspend the sliding leaf",
        )
        ctx.allow_overlap(
            wall,
            door,
            elem_a="head_rail",
            elem_b=f"trolley_{i}",
            reason=f"trolley block {i} rides inside the rail channel",
        )

    # ---- closed pose: leaf covers the framed opening
    with ctx.pose({slide: 0.0, spindle: 0.0}):
        ctx.expect_overlap(door, wall, axes="xz", min_overlap=0.5)

        # Trolley blocks remain engaged inside the rail channel.
        for i in range(2):
            ctx.expect_overlap(
                door,
                wall,
                elem_a=f"trolley_{i}",
                elem_b="head_rail",
                axes="xz",
                min_overlap=0.005,
                name=f"trolley_{i} engaged in head rail",
            )

        leaf_aabb = ctx.part_element_world_aabb(door, elem="leaf_plate")
        glass_aabb = ctx.part_element_world_aabb(door, elem="window_glass")
        bezel_aabb = ctx.part_element_world_aabb(door, elem="window_bezel")
        button_aabb = ctx.part_element_world_aabb(door, elem="blue_button")

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
            and leaf_aabb[1][1] >= -0.06,
            "closed leaf back face must sit just in front of the frame face",
        )

        closed_pos = ctx.part_world_position(door)
        lever_closed = ctx.part_world_aabb(lever)

    # ---- open pose: the leaf slides sideways to clear the opening
    with ctx.pose({slide: SLIDE_TRAVEL}):
        open_pos = ctx.part_world_position(door)
        ctx.check(
            "door_slides_sideways_to_open",
            closed_pos is not None
            and open_pos is not None
            and open_pos[0] > closed_pos[0] + SLIDE_TRAVEL - 0.01,
            "door leaf must translate along +X by the full slide travel",
        )
        # At full travel the leaf should clear the framed opening in X.
        open_leaf_aabb = ctx.part_world_aabb(door)
        ctx.check(
            "leaf_clears_opening_when_open",
            open_leaf_aabb is not None
            and open_leaf_aabb[0][0] > OPEN_W / 2.0,
            "open leaf must sit entirely to the right of the framed opening",
        )

    # ---- handle pose: pressing the lever drops the lever tip
    with ctx.pose({spindle: 0.7}):
        lever_down = ctx.part_world_aabb(lever)
        ctx.check(
            "lever_presses_down",
            lever_closed is not None
            and lever_down is not None
            and lever_down[0][2] < lever_closed[0][2] - 0.03,
            "lever tip must drop when the handle spindle rotates",
        )

    # ---- variant-specific: door_slide joint is PRISMATIC and the hangers
    #      connect the leaf to the rail (the defining skeleton change).
    ctx.check(
        "door_slide_joint_is_prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
        "primary door joint must be PRISMATIC for the sliding skeleton",
    )

    return ctx.report()
