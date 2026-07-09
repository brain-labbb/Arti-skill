from __future__ import annotations

# Modern bedside nightstand — hinged cabinet door variant.
#
# Same carcass shell as the parent (matte medium-gray lacquered panels with
# three-sided gallery lip, inset plinth base) but replaces the two prismatic
# drawers with a single front cabinet door hinged along the left vertical
# side edge.  The door swings outward (revolute about vertical Z) to reveal
# an interior shelf.  A matching polished-chrome bar handle is mounted near
# the free edge.
#
# World layout: front faces +X (back at x=0, front at x=D), width along Y,
# height along +Z.  Plinth rests on the floor at z=0.

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

# ---------- key dimensions (meters) ----------
W = 0.650            # overall width (Y)
D = 0.450            # overall depth (X)
H = 0.450            # overall height (Z)
WALL = 0.018         # carcass panel thickness
LIP = 0.060          # gallery lip rise above the recessed top shelf
PLINTH_H = 0.045     # inset plinth height (floating look)
PLINTH_INSET = 0.045

SHELF_THK = 0.016
SHELF_TOP_Z = H - LIP                        # recessed top surface (0.390)
SHELF_BOT_Z = SHELF_TOP_Z - SHELF_THK        # underside of top shelf (0.374)

BODY_BOT_Z = PLINTH_H                        # carcass starts above plinth
FACE_THK = 0.016                             # door panel thickness
INNER_W = W - 2.0 * WALL                     # cavity width between side panels
INNER_D = D - WALL                           # cavity depth (back inner face to front)

# Door opening zone: from carcass bottom to top-shelf underside.
FACE_BOT_MARGIN = 0.004
FACE_TOP_MARGIN = 0.004
DOOR_H = SHELF_BOT_Z - BODY_BOT_Z - FACE_BOT_MARGIN - FACE_TOP_MARGIN
DOOR_W = INNER_W - 0.004                     # 4 mm clearance on free edge
DOOR_CZ = BODY_BOT_Z + FACE_BOT_MARGIN + DOOR_H / 2.0

# Hinge: left vertical side edge (positive-Y inner face of side panel)
HINGE_Y = INNER_W / 2.0
DOOR_OPEN_ANGLE = 1.50                       # ~86°

# Interior shelf
INT_SHELF_THK = 0.016
INT_SHELF_Z = BODY_BOT_Z + (SHELF_BOT_Z - BODY_BOT_Z) * 0.45


def _build_door(model: ArticulatedObject, name: str, pale, chrome):
    """Single cabinet door panel + chrome bar handle.

    Door-local frame: origin at the hinge line; door extends toward -Y
    (free edge) and toward -X (inward from the outer face).  Outer face
    is at local x = 0.
    """
    door = model.part(name)

    # Flat door panel (pale satin finish).
    door.visual(
        Box((FACE_THK, DOOR_W, DOOR_H)),
        origin=Origin(xyz=(-FACE_THK / 2.0, -DOOR_W / 2.0, 0.0)),
        material=pale,
        name="door_panel",
    )

    # --- polished-chrome bar handle on two posts, near the free edge ---
    handle_y = -(DOOR_W - 0.120)             # offset from free edge
    handle_z = DOOR_H * 0.30                 # near the top
    bar_len = 0.160
    post_dy = 0.065
    for i in range(2):
        s = 1 - 2 * i                        # +1, -1
        door.visual(
            Box((0.016, 0.012, 0.012)),
            origin=Origin(xyz=(0.004, handle_y + s * post_dy, handle_z)),
            material=chrome,
            name=f"handle_post_{i}",
        )
    door.visual(
        Box((0.014, bar_len, 0.016)),
        origin=Origin(xyz=(0.019, handle_y, handle_z)),
        material=chrome,
        name="handle_bar",
    )

    door.inertial = Inertial.from_geometry(
        Box((FACE_THK, DOOR_W, DOOR_H)), mass=3.0
    )
    return door


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hinged_door_bedside_nightstand")

    gray = model.material("carcass_gray", rgba=(0.50, 0.52, 0.55, 1.0))
    gray_dark = model.material("plinth_gray", rgba=(0.27, 0.28, 0.30, 1.0))
    pale = model.material("front_pale_satin", rgba=(0.85, 0.87, 0.88, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.80, 0.82, 0.85, 1.0))
    shelf_mat = model.material("interior_shelf", rgba=(0.72, 0.74, 0.76, 1.0))

    # ===================================================================
    # ROOT: carcass (hollow shell with gallery lip and inset plinth)
    # ===================================================================
    carcass = model.part("carcass")

    # Inset plinth base (floating look).
    carcass.visual(
        Box((D - 2 * PLINTH_INSET, W - 2 * PLINTH_INSET, PLINTH_H)),
        origin=Origin(xyz=(D / 2.0, 0.0, PLINTH_H / 2.0)),
        material=gray_dark,
        name="plinth",
    )

    panel_h = H - BODY_BOT_Z
    # Two thick side panels (full depth, rising LIP above the top shelf).
    for i in range(2):
        s = 1 - 2 * i                        # +1, -1
        carcass.visual(
            Box((D, WALL, panel_h)),
            origin=Origin(
                xyz=(D / 2.0, s * (W / 2.0 - WALL / 2.0),
                     BODY_BOT_Z + panel_h / 2.0)
            ),
            material=gray,
            name=f"side_panel_{i}",
        )

    # Back panel (full height, between the side panels).
    carcass.visual(
        Box((WALL, INNER_W, panel_h)),
        origin=Origin(xyz=(WALL / 2.0, 0.0, BODY_BOT_Z + panel_h / 2.0)),
        material=gray,
        name="back_panel",
    )

    # Recessed top shelf — front edge flush with the cabinet face (open lip).
    carcass.visual(
        Box((D, INNER_W, SHELF_THK)),
        origin=Origin(xyz=(D / 2.0, 0.0, SHELF_BOT_Z + SHELF_THK / 2.0)),
        material=gray,
        name="top_shelf",
    )

    # Carcass bottom panel (recessed behind the door).
    carcass.visual(
        Box((D - FACE_THK, INNER_W, 0.016)),
        origin=Origin(xyz=((D - FACE_THK) / 2.0, 0.0, BODY_BOT_Z + 0.008)),
        material=gray,
        name="bottom_panel",
    )

    # Interior shelf — visible when the door opens.  Contacts the back
    # panel and both side-panel inner faces for solid support; stops short
    # of the door inner face so there is no overlap at rest.
    shelf_back_x = WALL                       # flush with back panel inner face
    shelf_front_x = D - FACE_THK - 0.014     # 14 mm behind door inner face
    shelf_depth = shelf_front_x - shelf_back_x
    shelf_cx = (shelf_front_x + shelf_back_x) / 2.0
    carcass.visual(
        Box((shelf_depth, INNER_W, INT_SHELF_THK)),
        origin=Origin(xyz=(shelf_cx, 0.0, INT_SHELF_Z)),
        material=shelf_mat,
        name="interior_shelf",
    )

    carcass.inertial = Inertial.from_geometry(Box((D, W, H)), mass=22.0)

    # ===================================================================
    # DOOR (single REVOLUTE, hinge along left vertical side edge)
    # ===================================================================
    for i in range(1):                       # uniform loop policy (n=1 door)
        door = _build_door(model, "door", pale, chrome)
        model.articulation(
            "carcass_to_door",
            ArticulationType.REVOLUTE,
            parent=carcass,
            child=door,
            origin=Origin(xyz=(D, HINGE_Y, DOOR_CZ)),
            axis=(0.0, 0.0, 1.0),            # +Z: positive q opens outward
            motion_limits=MotionLimits(
                effort=10.0, velocity=1.0,
                lower=0.0, upper=DOOR_OPEN_ANGLE,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    door = object_model.get_part("door")
    j_door = object_model.get_articulation("carcass_to_door")

    # --- Grounding and true overall scale (~0.65 × 0.45 × 0.45 m) ---
    cb = ctx.part_world_aabb(carcass)
    assert cb is not None
    ctx.check("plinth_on_floor", abs(cb[0][2]) < 0.003,
              details=f"carcass min z={cb[0][2]:.4f}")
    width_y = cb[1][1] - cb[0][1]
    depth_x = cb[1][0] - cb[0][0]
    height_z = cb[1][2] - cb[0][2]
    ctx.check("width_065", abs(width_y - 0.65) < 0.015, details=f"w={width_y:.3f}")
    ctx.check("depth_045", abs(depth_x - 0.45) < 0.015, details=f"d={depth_x:.3f}")
    ctx.check("height_045", abs(height_z - 0.45) < 0.015, details=f"h={height_z:.3f}")

    # --- Inset plinth (floating look) ---
    plinth = ctx.part_element_world_aabb(carcass, elem="plinth")
    side = ctx.part_element_world_aabb(carcass, elem="side_panel_0")
    assert plinth is not None and side is not None
    ctx.check(
        "plinth_inset",
        plinth[0][0] > cb[0][0] + 0.02 and plinth[1][0] < cb[1][0] - 0.02
        and plinth[0][1] > cb[0][1] + 0.02 and plinth[1][1] < cb[1][1] - 0.02,
        details=f"plinth x=({plinth[0][0]:.3f},{plinth[1][0]:.3f})",
    )

    # --- Gallery lip: side/back panels rise ~0.06 m above top shelf ---
    shelf = ctx.part_element_world_aabb(carcass, elem="top_shelf")
    back = ctx.part_element_world_aabb(carcass, elem="back_panel")
    assert shelf is not None and back is not None
    lip_rise = side[1][2] - shelf[1][2]
    ctx.check("gallery_lip_rise", abs(lip_rise - 0.06) < 0.005,
              details=f"lip rise={lip_rise:.3f}")
    ctx.check("back_lip_matches_sides", abs(back[1][2] - side[1][2]) < 0.002,
              details=f"back top={back[1][2]:.3f}, side top={side[1][2]:.3f}")
    # Front edge open: shelf front face flush with cabinet front.
    ctx.check("shelf_front_flush_open", abs(shelf[1][0] - cb[1][0]) < 0.003,
              details=f"shelf front x={shelf[1][0]:.3f}, carcass front={cb[1][0]:.3f}")

    # --- Door joint: revolute about vertical Z axis ---
    ctx.check("door_joint_revolute",
              j_door.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("door_joint_axis_z",
              abs(j_door.axis[2]) > 0.99 and abs(j_door.axis[0]) < 0.01
              and abs(j_door.axis[1]) < 0.01,
              details=f"axis={j_door.axis}")
    ctx.check("door_joint_range",
              abs(j_door.motion_limits.lower) < 1e-9
              and j_door.motion_limits.upper > 1.0,
              details=f"range=({j_door.motion_limits.lower:.2f}, "
                       f"{j_door.motion_limits.upper:.2f})")

    # --- Closed pose: door flush with cabinet front ---
    front_x = cb[1][0]
    door_panel = ctx.part_element_world_aabb(door, elem="door_panel")
    assert door_panel is not None
    ctx.check("door_flush_closed", abs(door_panel[1][0] - front_x) < 0.003,
              details=f"door front x={door_panel[1][0]:.4f}, "
                       f"carcass front={front_x:.4f}")

    # Door panel covers the full opening height.
    opening_h = SHELF_BOT_Z - BODY_BOT_Z
    ctx.check("door_covers_opening",
              abs((door_panel[1][2] - door_panel[0][2]) - opening_h) < 0.015,
              details=f"door h={door_panel[1][2] - door_panel[0][2]:.3f}, "
                       f"opening h={opening_h:.3f}")

    # Chrome bar handle stands proud of the door face.
    bar = ctx.part_element_world_aabb(door, elem="handle_bar")
    assert bar is not None
    ctx.check("handle_bar_proud", bar[0][0] > door_panel[1][0] + 0.008,
              details=f"bar min x={bar[0][0]:.4f}")

    # --- Interior shelf exists within the carcass ---
    int_shelf = ctx.part_element_world_aabb(carcass, elem="interior_shelf")
    assert int_shelf is not None
    ctx.check("interior_shelf_within_carcass",
              int_shelf[0][0] > cb[0][0] + 0.01
              and int_shelf[1][0] < cb[1][0] - 0.01
              and int_shelf[0][1] > cb[0][1] + 0.01
              and int_shelf[1][1] < cb[1][1] - 0.01,
              details=f"shelf x=({int_shelf[0][0]:.3f},{int_shelf[1][0]:.3f})")
    # Shelf is at roughly mid-height of the interior.
    shelf_z_center = (int_shelf[0][2] + int_shelf[1][2]) / 2.0
    mid_z = (BODY_BOT_Z + SHELF_BOT_Z) / 2.0
    ctx.check("interior_shelf_mid_height",
              abs(shelf_z_center - mid_z) < 0.06,
              details=f"shelf z={shelf_z_center:.3f}, mid={mid_z:.3f}")
    # Shelf front edge is behind the door inner face (no overlap at rest).
    ctx.check("shelf_behind_door",
              int_shelf[1][0] < door_panel[0][0] + 0.001,
              details=f"shelf front x={int_shelf[1][0]:.4f}, "
                       f"door back x={door_panel[0][0]:.4f}")

    # --- Open pose: door swings outward (+X) ---
    with ctx.pose({j_door: 1.0}):
        open_panel = ctx.part_element_world_aabb(door, elem="door_panel")
    assert open_panel is not None
    ctx.check("door_swings_outward",
              open_panel[1][0] > front_x + 0.10,
              details=f"open door max x={open_panel[1][0]:.3f}, "
                       f"front={front_x:.3f}")

    # At open pose the free edge should be clearly past the carcass side.
    with ctx.pose({j_door: 1.0}):
        open_bar = ctx.part_element_world_aabb(door, elem="handle_bar")
    assert open_bar is not None
    ctx.check("handle_visible_when_open",
              open_bar[0][0] > front_x + 0.05,
              details=f"open bar min x={open_bar[0][0]:.3f}")

    return ctx.report()


object_model = build_object_model()
