from __future__ import annotations

# Top-hung barn door sliding glass door.
# Replaces the perimeter frame + head/sill channel track of the parent with
# an exposed horizontal rail mounted above the opening, two roller hangers on
# the leaf top, and a single small floor guide pin at the bottom.
# The leaf glazing and one prismatic horizontal joint remain identical.
#
# Conventions:
# - World up is +Z. Width along X, height along Z, depth along Y.
# - The door stands on the ground: all geometry authored from z=0 up.
# - Rail and leaf are coplanar (both parallel to the wall surface at Y=0).

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# ----------------------------------------------------------------------------
# Real-world dimensions (meters)
# ----------------------------------------------------------------------------
UNIT_W = 2.80          # opening width
UNIT_H = 2.00          # opening height

# Parent leaf dimensions (preserved from the two-panel sliding door parent).
FRAME_FACE = 0.042     # parent frame face width (used for leaf sizing)
INNER_W = UNIT_W - 2 * FRAME_FACE
INNER_H = UNIT_H - 2 * FRAME_FACE
BAY_W = INNER_W / 2.0

LEAF_FRAME_FACE = 0.035
LEAF_FRAME_DEPTH = 0.040
GLASS_T = 0.008

leaf_w = BAY_W                     # leaf fills one bay width
leaf_h = INNER_H - 0.032          # same as parent

# Barn door rail
RAIL_W = 3.40          # exposed rail width (extends past opening both sides)
RAIL_H = 0.060         # rail vertical face height
RAIL_D = 0.040         # rail depth (front-to-back, Y direction)
RAIL_BOTTOM_Z = UNIT_H + 0.050   # rail bottom 50 mm above the opening

# Roller hanger hardware
HANGER_W = 0.050       # hanger bracket width
HANGER_T = 0.008       # hanger bracket thickness (Y)
HANGER_H = 0.080       # visible gap between leaf top and rail bottom

ROLLER_R = 0.018       # roller wheel radius
ROLLER_W = 0.025       # roller wheel width (along Y axle)

# Wall mounting brackets
BRACKET_W = 0.060
BRACKET_PLATE_T = 0.006
BRACKET_PLATE_H = 0.100

# Floor guide
GUIDE_R = 0.010
GUIDE_H = 0.050
GUIDE_BASE_SIZE = 0.040
GUIDE_BASE_T = 0.004

# Y layout: rail and leaf coplanar (both parallel to the wall surface).
Y_RAIL = 0.0
Y_LEAF = 0.0

# Leaf Z layout: hangs from the rail via hangers.
LEAF_TOP_Z = RAIL_BOTTOM_Z - HANGER_H
LEAF_CENTER_Z = LEAF_TOP_Z - leaf_h / 2.0

# Prismatic joint parameters (identical to parent).
leaf_closed_cx = BAY_W / 2.0 + FRAME_FACE / 4.0
travel = BAY_W - FRAME_FACE

# Hanger X positions (symmetric on the leaf, in local leaf coords).
HANGER_SPACING = leaf_w * 0.6
HANGER_XS = [-HANGER_SPACING / 2.0, HANGER_SPACING / 2.0]

# Colors / materials
BLACK_STEEL = (0.10, 0.10, 0.11)
DARK_ALU = (0.13, 0.13, 0.14)
GLASS_COLOR = (0.82, 0.85, 0.87, 0.16)
HANDLE_METAL = (0.16, 0.16, 0.17)
BRUSHED_STEEL = (0.38, 0.38, 0.40)
FLOOR_BRASS = (0.55, 0.45, 0.20)


def _build_roller_mesh() -> "Mesh":
    """Build a curved roller wheel mesh (cylinder along Y axis)."""
    geom = CylinderGeometry(ROLLER_R, ROLLER_W, radial_segments=32, closed=True)
    # CylinderGeometry is along Z; rotate 90° around X to align with Y.
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, "roller_wheel")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="barn_door_sliding_glass")

    model.material("black_steel", rgba=(*BLACK_STEEL, 1.0))
    model.material("dark_alu", rgba=(*DARK_ALU, 1.0))
    model.material("glass", rgba=GLASS_COLOR)
    model.material("handle_metal", rgba=(*HANDLE_METAL, 1.0))
    model.material("brushed_steel", rgba=(*BRUSHED_STEEL, 1.0))
    model.material("floor_brass", rgba=(*FLOOR_BRASS, 1.0))

    # Build the shared roller mesh once (used by both hangers).
    roller_mesh = _build_roller_mesh()

    # ------------------------------------------------------------------
    # ROOT: barn rail + wall brackets + floor guide (all static)
    # ------------------------------------------------------------------
    rail_part = model.part("barn_rail")

    # Exposed horizontal rail bar.
    rail_part.visual(
        Box((RAIL_W, RAIL_D, RAIL_H)),
        origin=Origin(xyz=(0.0, Y_RAIL, RAIL_BOTTOM_Z + RAIL_H / 2.0)),
        name="rail_bar",
        material="black_steel",
    )

    # Wall mounting brackets (2, symmetric via loop).
    # Full-height mounting strips from floor to rail top — realistic for
    # barn door hardware where vertical mounting rails carry the load.
    bracket_xs = [-RAIL_W / 2.0 + 0.20, RAIL_W / 2.0 - 0.20]
    plate_full_h = RAIL_BOTTOM_Z + RAIL_H   # floor to rail top
    for i in range(2):
        bx = bracket_xs[i]
        # Full-height wall mounting plate.
        rail_part.visual(
            Box((BRACKET_W, BRACKET_PLATE_T, plate_full_h)),
            origin=Origin(xyz=(
                bx,
                -(RAIL_D / 2.0 + BRACKET_PLATE_T / 2.0),
                plate_full_h / 2.0,
            )),
            name=f"bracket_plate_{i}",
            material="black_steel",
        )
        # Horizontal support shelf (under the rail, flush with rail bottom).
        rail_part.visual(
            Box((BRACKET_W, RAIL_D, BRACKET_PLATE_T)),
            origin=Origin(xyz=(
                bx,
                Y_RAIL,
                RAIL_BOTTOM_Z - BRACKET_PLATE_T / 2.0,
            )),
            name=f"bracket_arm_{i}",
            material="black_steel",
        )

    # Floor guide at the base of the left mounting strip.
    # The guide base slightly embeds into the bracket plate front face,
    # representing the seated mounting connection.
    GUIDE_X = bracket_xs[0]
    GUIDE_Y = -0.001   # slight shift toward wall so base embeds into plate face
    rail_part.visual(
        Box((GUIDE_BASE_SIZE, GUIDE_BASE_SIZE, GUIDE_BASE_T)),
        origin=Origin(xyz=(GUIDE_X, GUIDE_Y, GUIDE_BASE_T / 2.0)),
        name="floor_guide_base",
        material="black_steel",
    )
    rail_part.visual(
        Cylinder(radius=GUIDE_R, length=GUIDE_H),
        origin=Origin(xyz=(GUIDE_X, GUIDE_Y, GUIDE_BASE_T + GUIDE_H / 2.0)),
        name="floor_guide_pin",
        material="floor_brass",
    )

    # ------------------------------------------------------------------
    # MOVING LEAF: identical glazing + slim frame + handle (from parent)
    # ------------------------------------------------------------------
    leaf = model.part("sliding_leaf")

    fa = LEAF_FRAME_FACE
    fd = LEAF_FRAME_DEPTH
    stile_h = leaf_h - 2 * fa

    # Local leaf frame: origin at the joint (rail bottom face level).
    # Leaf top at Z_local = -HANGER_H, leaf bottom at Z_local = -(HANGER_H + leaf_h).
    mid_cz = -(HANGER_H + leaf_h / 2.0)
    top_rail_cz = -(HANGER_H + fa / 2.0)
    bot_rail_cz = -(HANGER_H + leaf_h - fa / 2.0)

    # Leaf frame stiles (2, loop): butt between top and bottom rails.
    for i in range(2):
        sx = (-leaf_w / 2.0 + fa / 2.0) if i == 0 else (leaf_w / 2.0 - fa / 2.0)
        leaf.visual(
            Box((fa, fd, stile_h)),
            origin=Origin(xyz=(sx, 0.0, mid_cz)),
            name=f"leaf_stile_{i}",
            material="dark_alu",
        )

    # Top rail.
    leaf.visual(
        Box((leaf_w - 2 * fa, fd, fa)),
        origin=Origin(xyz=(0.0, 0.0, top_rail_cz)),
        name="leaf_top_rail",
        material="dark_alu",
    )
    # Bottom rail.
    leaf.visual(
        Box((leaf_w - 2 * fa, fd, fa)),
        origin=Origin(xyz=(0.0, 0.0, bot_rail_cz)),
        name="leaf_bottom_rail",
        material="dark_alu",
    )
    # Glass infill.
    leaf.visual(
        Box((leaf_w - 2 * fa, GLASS_T, leaf_h - 2 * fa)),
        origin=Origin(xyz=(0.0, 0.0, mid_cz)),
        name="leaf_pane",
        material="glass",
    )

    # Recessed lever handle on the leading (left) stile, room side (+Y).
    handle_z_world = 1.0   # ~1.0 m above ground
    handle_z_local = handle_z_world - RAIL_BOTTOM_Z
    handle_x = -leaf_w / 2.0 + fa / 2.0
    HANDLE_LEN = 0.28
    leaf.visual(
        Box((0.012, 0.016, HANDLE_LEN)),
        origin=Origin(xyz=(handle_x, fd / 2.0 + 0.008, handle_z_local)),
        name="handle_lever",
        material="handle_metal",
    )

    # ------------------------------------------------------------------
    # ROLLER HANGERS (2, loop) — on the leaf, riding the rail
    # ------------------------------------------------------------------
    for i in range(2):
        hx = HANGER_XS[i]
        # Hanger bracket plate: from leaf top up to the rail bottom.
        leaf.visual(
            Box((HANGER_W, HANGER_T, HANGER_H)),
            origin=Origin(xyz=(hx, 0.0, -(HANGER_H / 2.0))),
            name=f"hanger_{i}",
            material="brushed_steel",
        )
        # Roller wheel (curved mesh) sitting on the rail bottom face.
        leaf.visual(
            roller_mesh,
            origin=Origin(xyz=(hx, 0.0, -ROLLER_R)),
            name=f"roller_{i}",
            material="brushed_steel",
        )

    # ------------------------------------------------------------------
    # ARTICULATION: single prismatic joint (identical to parent)
    # ------------------------------------------------------------------
    model.articulation(
        "leaf_slide",
        ArticulationType.PRISMATIC,
        parent=rail_part,
        child=leaf,
        # Joint origin on the rail bottom face (actual contact surface).
        origin=Origin(xyz=(leaf_closed_cx, Y_LEAF, RAIL_BOTTOM_Z)),
        # Positive q slides the leaf left (-X) to open.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0, velocity=0.8, lower=0.0, upper=travel,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    rail_part = object_model.get_part("barn_rail")
    leaf = object_model.get_part("sliding_leaf")
    slide = object_model.get_articulation("leaf_slide")

    # --- Barn door mechanism hardware is present ---
    ctx.check(
        "exposed rail bar exists",
        rail_part.get_visual("rail_bar") is not None,
        details="rail_bar visual missing from barn_rail",
    )
    for i in range(2):
        ctx.check(
            f"roller hanger {i} exists",
            leaf.get_visual(f"hanger_{i}") is not None,
            details=f"hanger_{i} visual missing from sliding_leaf",
        )
        ctx.check(
            f"roller wheel {i} exists",
            leaf.get_visual(f"roller_{i}") is not None,
            details=f"roller_{i} visual missing from sliding_leaf",
        )
    ctx.check(
        "floor guide pin exists",
        rail_part.get_visual("floor_guide_pin") is not None,
        details="floor_guide_pin visual missing from barn_rail",
    )

    # --- Confirm perimeter frame is removed (mechanism change) ---
    rail_visual_names = {v.name for v in rail_part.visuals}
    ctx.check(
        "no perimeter frame jambs",
        "jamb_left" not in rail_visual_names
        and "jamb_right" not in rail_visual_names,
        details="perimeter frame jambs must not exist in barn door variant",
    )
    ctx.check(
        "no head/sill track channels",
        "head_track" not in rail_visual_names
        and "sill_track" not in rail_visual_names,
        details="enclosed head/sill channels must not exist in barn door variant",
    )

    # --- Leaf glazing and handle are preserved ---
    ctx.check(
        "leaf glass pane exists",
        leaf.get_visual("leaf_pane") is not None,
        details="leaf_pane visual missing",
    )
    ctx.check(
        "handle lever exists",
        leaf.get_visual("handle_lever") is not None,
        details="handle_lever visual missing",
    )

    # --- Glass is transparent ---
    glass_mat = next(
        (m for m in object_model.materials if m.name == "glass"), None
    )
    alpha = glass_mat.rgba[3] if glass_mat is not None and glass_mat.rgba else 1.0
    ctx.check(
        "glass material is transparent",
        glass_mat is not None and alpha < 0.5,
        details=f"glass alpha={alpha}",
    )

    # --- Rail is above the opening (z > UNIT_H) ---
    rail_aabb = ctx.part_element_world_aabb(rail_part, elem="rail_bar")
    ctx.check(
        "rail sits above the door opening",
        rail_aabb is not None and rail_aabb[0][2] > UNIT_H - 0.01,
        details=f"rail z-min={rail_aabb[0][2] if rail_aabb else None}",
    )

    # --- Roller hangers contact the rail bottom ---
    for i in range(2):
        ctx.expect_contact(
            leaf, rail_part,
            elem_a=f"roller_{i}",
            elem_b="rail_bar",
            contact_tol=0.002,
            name=f"roller_{i} contacts the rail bottom face",
        )

    # --- Leaf slides correctly (positive q moves leaf left to open) ---
    closed_pos = ctx.part_world_position(leaf)
    upper = slide.motion_limits.upper
    with ctx.pose({slide: upper}):
        open_pos = ctx.part_world_position(leaf)
    ctx.check(
        "positive q slides the leaf toward -X (open)",
        closed_pos is not None
        and open_pos is not None
        and open_pos[0] < closed_pos[0] - 0.30,
        details=f"closed_x={closed_pos[0] if closed_pos else None}, "
                f"open_x={open_pos[0] if open_pos else None}",
    )

    # --- Hangers stay within rail width at rest and at full travel ---
    for i in range(2):
        ctx.expect_within(
            leaf, rail_part,
            axes="x",
            inner_elem=f"hanger_{i}",
            outer_elem="rail_bar",
            margin=0.01,
            name=f"hanger_{i} stays within rail width at rest",
        )
    with ctx.pose({slide: upper}):
        for i in range(2):
            ctx.expect_within(
                leaf, rail_part,
                axes="x",
                inner_elem=f"hanger_{i}",
                outer_elem="rail_bar",
                margin=0.01,
                name=f"hanger_{i} stays within rail width at open limit",
            )

    # --- Leaf stays above the floor ---
    leaf_aabb = ctx.part_world_aabb(leaf)
    ctx.check(
        "leaf stays above the floor",
        leaf_aabb is not None and leaf_aabb[0][2] >= -0.01,
        details=f"leaf z-min={leaf_aabb[0][2] if leaf_aabb else None}",
    )

    # --- Floor guide stays below the leaf bottom ---
    ctx.expect_gap(
        leaf, rail_part,
        axis="z",
        positive_elem="leaf_bottom_rail",
        negative_elem="floor_guide_pin",
        min_gap=0.005,
        name="floor guide stays below the leaf bottom",
    )

    return ctx.report()


object_model = build_object_model()
