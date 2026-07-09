from __future__ import annotations

"""Double-leaf biparting industrial blast door set into a concrete wall.

Two mirrored matte-black steel door leaves meet at a centre astragal, each
hung on three barrel hinges from its own jamb in a widened dark-steel frame
set into board-formed concrete.  Both leaves carry a viewing window with a
raised black bezel, round blue emergency button, keyed lock cylinder with
silver escutcheon, and a stainless handle rose.  The right (active) leaf
also has a stainless lever handle that rotates to retract the latch.

Articulations:
- ``door_hinge_left`` (REVOLUTE): left leaf swings open toward the viewer (-Y).
- ``door_hinge_right`` (REVOLUTE): right leaf swings open toward the viewer (-Y).
- ``handle_spindle`` (REVOLUTE): lever rotates down to retract the latch.
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

OPEN_W = 1.92          # clear opening width  (doubled for biparting pair)
OPEN_H = 2.04          # clear opening height

FRAME_OUT_W = 2.08     # steel frame outer width
FRAME_OUT_H = 2.12
FRAME_PROUD = 0.02     # frame stands proud of the concrete front face (y=0)
FRAME_DEPTH = 0.12

LEAF_W = 0.98          # each leaf width
LEAF_H = 2.10
LEAF_T = 0.06

HINGE_X_LEFT = -1.00   # left-jamb pivot axis (world x)
HINGE_X_RIGHT = 1.00   # right-jamb pivot axis (world x)
HINGE_Y = -0.056       # door mid-thickness plane
HINGE_Z_LIST = [0.35, 1.05, 1.75]

LEAF_X0 = 0.02         # gap from hinge axis to leaf start edge
LEAF_FRONT = -LEAF_T / 2.0

# Feature distances from hinge axis (mirrored for each leaf)
WINDOW_CX = 0.40
WINDOW_CZ = 1.55
WINDOW_W = 0.30
WINDOW_H = 0.20

BUTTON_X = 0.65
BUTTON_Z = 1.58
LOCK_X = 0.76
LOCK_Z = 1.30
HANDLE_X = 0.76
HANDLE_Z = 1.05

# Astragal dimensions (left leaf, bridges the centre gap)
ASTRAGAL_W = 0.030
ASTRAGAL_T = 0.008
ASTRAGAL_EXTRA = 0.012  # how far past the free edge the astragal extends

# ------------------------------------------------------------------ materials
CONCRETE = Material(name="board_formed_concrete", rgba=(0.62, 0.61, 0.58, 1.0))
FRAME_STEEL = Material(name="frame_steel", rgba=(0.15, 0.16, 0.17, 1.0))
LEAF_STEEL = Material(name="leaf_steel", rgba=(0.10, 0.10, 0.11, 1.0))
BEZEL_BLACK = Material(name="bezel_black", rgba=(0.06, 0.06, 0.07, 1.0))
GLASS = Material(name="window_glass", rgba=(0.62, 0.72, 0.76, 0.45))
STAINLESS = Material(name="stainless", rgba=(0.74, 0.75, 0.77, 1.0))
BLUE_BUTTON = Material(name="blue_button", rgba=(0.10, 0.38, 0.85, 1.0))
HINGE_STEEL = Material(name="hinge_steel", rgba=(0.20, 0.21, 0.22, 1.0))


# ----------------------------------------------------------------- leaf helper
def _build_leaf(model, wall, leaf_sign: int):
    """Build one door leaf.

    ``leaf_sign = +1``: **left leaf** — part frame on the left jamb, leaf
    extends +X toward the centre.
    ``leaf_sign = -1``: **right leaf** — part frame on the right jamb, leaf
    extends −X toward the centre.
    """
    is_left = leaf_sign > 0
    leaf_name = "door_leaf_left" if is_left else "door_leaf_right"
    hinge_name = "door_hinge_left" if is_left else "door_hinge_right"
    hinge_x = HINGE_X_LEFT if is_left else HINGE_X_RIGHT

    door = model.part(leaf_name)

    # ---- leaf plate with window cutout
    leaf_cx = leaf_sign * (LEAF_X0 + LEAF_W / 2.0)
    leaf_solid = (
        cq.Workplane("XY")
        .box(LEAF_W, LEAF_T, LEAF_H, centered=(True, True, False))
        .translate((leaf_cx, 0.0, 0.01))
    )
    window_cx_local = leaf_sign * WINDOW_CX
    window_cut = (
        cq.Workplane("XY")
        .box(WINDOW_W + 0.02, LEAF_T + 0.02, WINDOW_H + 0.02)
        .translate((window_cx_local, 0.0, WINDOW_CZ))
    )
    leaf_solid = leaf_solid.cut(window_cut)
    door.visual(
        mesh_from_cadquery(leaf_solid, f"{leaf_name}_plate"),
        material=LEAF_STEEL,
        name="leaf_plate",
    )

    # ---- raised black window bezel
    bezel = BezelGeometry(
        (WINDOW_W, WINDOW_H),
        (WINDOW_W + 0.10, WINDOW_H + 0.10),
        LEAF_T + 0.024,
        opening_shape="rect",
        outer_shape="rect",
    )
    door.visual(
        mesh_from_geometry(bezel, f"{leaf_name}_bezel"),
        origin=Origin(
            xyz=(window_cx_local, 0.0, WINDOW_CZ),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=BEZEL_BLACK,
        name="window_bezel",
    )
    # ---- glass pane
    door.visual(
        Box((WINDOW_W + 0.03, 0.008, WINDOW_H + 0.03)),
        origin=Origin(xyz=(window_cx_local, 0.0, WINDOW_CZ)),
        material=GLASS,
        name="window_glass",
    )

    # ---- round blue emergency button
    button_x = leaf_sign * BUTTON_X
    door.visual(
        Cylinder(radius=0.021, length=0.012),
        origin=Origin(
            xyz=(button_x, LEAF_FRONT - 0.004, BUTTON_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=BLUE_BUTTON,
        name="blue_button",
    )

    # ---- keyed lock
    lock_x = leaf_sign * LOCK_X
    door.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(
            xyz=(lock_x, LEAF_FRONT - 0.003, LOCK_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=STAINLESS,
        name="lock_escutcheon",
    )
    door.visual(
        Cylinder(radius=0.008, length=0.010),
        origin=Origin(
            xyz=(lock_x, LEAF_FRONT - 0.010, LOCK_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=FRAME_STEEL,
        name="lock_cylinder",
    )

    # ---- handle rose (lever itself is a separate part on the right leaf only)
    handle_x = leaf_sign * HANDLE_X
    door.visual(
        Cylinder(radius=0.032, length=0.012),
        origin=Origin(
            xyz=(handle_x, LEAF_FRONT - 0.004, HANDLE_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=STAINLESS,
        name="handle_rose",
    )

    # ---- barrel hinges (loop-generated)
    for i, hz in enumerate(HINGE_Z_LIST):
        door.visual(
            Cylinder(radius=0.024, length=0.18),
            origin=Origin(xyz=(0.0, 0.0, hz)),
            material=HINGE_STEEL,
            name=f"hinge_barrel_{i}",
        )
        door.visual(
            Box((0.13, 0.014, 0.10)),
            origin=Origin(xyz=(leaf_sign * 0.055, LEAF_FRONT - 0.004, hz)),
            material=HINGE_STEEL,
            name=f"hinge_strap_{i}",
        )

    # ---- astragal (left leaf only — bridges the centre meeting gap)
    if is_left:
        astragal_x = LEAF_X0 + LEAF_W + ASTRAGAL_EXTRA / 2.0
        astragal_zc = 0.01 + LEAF_H / 2.0
        # Seat the astragal 1 mm into the leaf front face so the mesh is
        # connected to the leaf plate rather than floating proud.
        astragal_y = LEAF_FRONT - ASTRAGAL_T / 2.0 + 0.001
        door.visual(
            Box((ASTRAGAL_W, ASTRAGAL_T, LEAF_H - 0.06)),
            origin=Origin(
                xyz=(astragal_x, astragal_y, astragal_zc),
            ),
            material=LEAF_STEEL,
            name="astragal",
        )

    # ---- swing articulation
    # Left leaf (+X from hinge): axis −Z → positive q swings free edge toward −Y.
    # Right leaf (−X from hinge): axis +Z → positive q swings free edge toward −Y.
    hinge_axis = (0.0, 0.0, -float(leaf_sign))
    model.articulation(
        hinge_name,
        ArticulationType.REVOLUTE,
        parent=wall,
        child=door,
        origin=Origin(xyz=(hinge_x, HINGE_Y, 0.0)),
        axis=hinge_axis,
        motion_limits=MotionLimits(
            effort=200.0, velocity=1.0, lower=0.0, upper=2.0
        ),
    )
    return door


# ============================================================= object model
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_blast_door_double")

    # ---------------------------------------------------------- wall + frame
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

    # Frame-side hinge tabs (loop for both jambs)
    for i, hz in enumerate(HINGE_Z_LIST):
        # Left jamb
        wall.visual(
            Box((0.062, 0.057, 0.030)),
            origin=Origin(xyz=(HINGE_X_LEFT - 0.014, -0.0335, hz + 0.075)),
            material=FRAME_STEEL,
            name=f"hinge_tab_{i}",
        )
        # Right jamb
        wall.visual(
            Box((0.062, 0.057, 0.030)),
            origin=Origin(xyz=(HINGE_X_RIGHT + 0.014, -0.0335, hz + 0.075)),
            material=FRAME_STEEL,
            name=f"hinge_tab_{i + 3}",
        )

    # -------------------------------------------------------- both door leaves
    door_left = _build_leaf(model, wall, leaf_sign=+1)
    door_right = _build_leaf(model, wall, leaf_sign=-1)

    # ---------------------------------------------------------- lever handle
    # Mounted on the right (active) leaf only.
    lever = model.part("lever_handle")
    lever.visual(
        Cylinder(radius=0.014, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=STAINLESS,
        name="spindle_hub",
    )
    # Lever arm points toward hinge side (+X in right-leaf local frame).
    lever.visual(
        Box((0.16, 0.022, 0.030)),
        origin=Origin(xyz=(0.085, -0.020, 0.0)),
        material=STAINLESS,
        name="lever_arm",
    )
    lever.visual(
        Sphere(radius=0.016),
        origin=Origin(xyz=(0.165, -0.020, 0.0)),
        material=STAINLESS,
        name="lever_tip",
    )

    # Handle spindle: on the right leaf, arm at +X, axis +Y → positive q drops tip.
    handle_x_local = -HANDLE_X  # right leaf uses negative local x
    model.articulation(
        "handle_spindle",
        ArticulationType.REVOLUTE,
        parent=door_right,
        child=lever,
        origin=Origin(xyz=(handle_x_local, LEAF_FRONT - 0.010, HANDLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0, velocity=3.0, lower=0.0, upper=0.9
        ),
    )

    return model


object_model = build_object_model()


# =================================================================== tests
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wall = object_model.get_part("wall_frame")
    door_left = object_model.get_part("door_leaf_left")
    door_right = object_model.get_part("door_leaf_right")
    lever = object_model.get_part("lever_handle")

    hinge_left = object_model.get_articulation("door_hinge_left")
    hinge_right = object_model.get_articulation("door_hinge_right")
    spindle = object_model.get_articulation("handle_spindle")

    # --- intentional overlap allowances ---

    # Handle spindle hub passes through the rose into the leaf bore.
    ctx.allow_overlap(
        door_right,
        lever,
        reason="handle spindle hub engages through the rose into the leaf bore",
    )

    # Frame hinge lugs wrap the knuckle barrels on both pivot axes.
    for i in range(3):
        ctx.allow_overlap(
            wall, door_left,
            elem_a=f"hinge_tab_{i}",
            elem_b=f"hinge_barrel_{i}",
            reason="left-jamb frame hinge lug wraps the knuckle barrel on the pivot axis",
        )
        ctx.allow_overlap(
            wall, door_right,
            elem_a=f"hinge_tab_{i + 3}",
            elem_b=f"hinge_barrel_{i}",
            reason="right-jamb frame hinge lug wraps the knuckle barrel on the pivot axis",
        )

    # Left-leaf astragal sits proud of the right-leaf front face at the centre
    # meeting edge (thin compression-seal overlap).
    ctx.allow_overlap(
        door_left, door_right,
        elem_a="astragal",
        elem_b="leaf_plate",
        reason="left leaf astragal overlaps right leaf meeting edge to seal the biparting gap",
    )

    # --- closed-pose checks ---
    with ctx.pose({hinge_left: 0.0, hinge_right: 0.0, spindle: 0.0}):
        # Both leaves cover the framed opening.
        ctx.expect_overlap(
            door_left, wall, axes="xz", min_overlap=0.5,
            name="door_leaf_left_covers_opening",
        )
        ctx.expect_overlap(
            door_right, wall, axes="xz", min_overlap=0.5,
            name="door_leaf_right_covers_opening",
        )

        # Hinge engagement on both jambs.
        for i in range(3):
            ctx.expect_overlap(
                wall, door_left,
                elem_a=f"hinge_tab_{i}", elem_b=f"hinge_barrel_{i}",
                axes="xy", min_overlap=0.003,
                name=f"left_hinge_tab_{i}_wraps_barrel",
            )
            ctx.expect_overlap(
                wall, door_right,
                elem_a=f"hinge_tab_{i + 3}", elem_b=f"hinge_barrel_{i}",
                axes="xy", min_overlap=0.003,
                name=f"right_hinge_tab_{i}_wraps_barrel",
            )

        # Astragal bridges the centre meeting edge.
        ctx.expect_overlap(
            door_left, door_right,
            elem_a="astragal", elem_b="leaf_plate",
            axes="x", min_overlap=0.005,
            name="astragal_bridges_centre_astragal_gap",
        )

        # Window glass containment inside bezel (both leaves).
        for leaf, leaf_label in [(door_left, "left"), (door_right, "right")]:
            glass_aabb = ctx.part_element_world_aabb(leaf, elem="window_glass")
            bezel_aabb = ctx.part_element_world_aabb(leaf, elem="window_bezel")
            ctx.check(
                f"window_glass_inside_bezel_{leaf_label}",
                glass_aabb is not None
                and bezel_aabb is not None
                and glass_aabb[0][0] > bezel_aabb[0][0]
                and glass_aabb[1][0] < bezel_aabb[1][0]
                and glass_aabb[0][2] > bezel_aabb[0][2]
                and glass_aabb[1][2] < bezel_aabb[1][2],
                "window glass must sit inside the raised bezel outline",
            )

        closed_left = ctx.part_world_aabb(door_left)
        closed_right = ctx.part_world_aabb(door_right)
        lever_closed = ctx.part_world_aabb(lever)

    # --- door_hinge_left: left leaf swings open toward viewer (-Y) ---
    with ctx.pose({hinge_left: 1.3, hinge_right: 0.0}):
        open_left = ctx.part_world_aabb(door_left)
        ctx.check(
            "door_hinge_left_swings_open",
            closed_left is not None
            and open_left is not None
            and open_left[0][1] < closed_left[0][1] - 0.5,
            "door_leaf_left must swing well out of the wall plane toward -Y",
        )

    # --- door_hinge_right: right leaf swings open toward viewer (-Y) ---
    with ctx.pose({hinge_left: 0.0, hinge_right: 1.3}):
        open_right = ctx.part_world_aabb(door_right)
        ctx.check(
            "door_hinge_right_swings_open",
            closed_right is not None
            and open_right is not None
            and open_right[0][1] < closed_right[0][1] - 0.5,
            "door_leaf_right must swing well out of the wall plane toward -Y",
        )

    # --- handle_spindle: pressing the lever drops the lever tip ---
    with ctx.pose({hinge_right: 0.0, spindle: 0.7}):
        lever_down = ctx.part_world_aabb(lever)
        ctx.check(
            "lever_presses_down",
            lever_closed is not None
            and lever_down is not None
            and lever_down[0][2] < lever_closed[0][2] - 0.03,
            "lever tip must drop when the handle spindle rotates",
        )

    return ctx.report()
