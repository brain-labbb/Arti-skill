from __future__ import annotations

# Modern bedside nightstand with drop-down flap door
# (matte medium-gray lacquered carcass, pale satin door front,
# polished-chrome bar handle).
#
# Variant of the two-drawer parent: replaces the two independent prismatic
# drawers with a single drop-down flap door hinged along its BOTTOM edge
# (revolute about the width Y axis), tilting forward and down to open.
#
# World layout: front faces +X (back at x=0, front at x=D), width along Y,
# height along +Z. The plinth rests on the floor at z=0 and is inset from all
# faces so the body appears to float. The carcass is a hollow shell: two thick
# side panels and a back panel rise ~0.06 m above the recessed top shelf,
# forming a three-sided gallery lip (front edge open/flush).
#
# Articulation: single REVOLUTE flap door, hinged at the bottom front edge
# of the carcass opening, axis along -Y so positive q tilts the top of the
# door forward and downward, range 0 (closed/vertical) to ~1.50 rad (~86°).

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
WALL = 0.018         # thick carcass panel thickness
LIP = 0.060          # gallery lip rise above the recessed top shelf
PLINTH_H = 0.045     # inset plinth height (floating look)
PLINTH_INSET = 0.045

SHELF_THK = 0.016
SHELF_TOP_Z = H - LIP            # recessed top surface height (0.390)
SHELF_BOT_Z = SHELF_TOP_Z - SHELF_THK  # 0.374

BODY_BOT_Z = PLINTH_H            # carcass panels start on top of the plinth
FACE_THK = 0.016                 # door slab thickness
INNER_W = W - 2.0 * WALL         # cavity width between side panels (0.614)

# Flap door zone: full front opening from carcass bottom to shelf underside.
FACE_TOP_MARGIN = 0.004
ZONE_H = SHELF_BOT_Z - BODY_BOT_Z       # 0.329
DOOR_H = ZONE_H - FACE_TOP_MARGIN       # 0.325
DOOR_W = INNER_W - 0.004                # 0.610 (2 mm clearance each side)

# Hinge line at the bottom front edge of the carcass opening.
HINGE_X = D
HINGE_Z = BODY_BOT_Z

# Handle dimensions (matching parent chrome bar style).
HANDLE_BAR_LEN = 0.180
HANDLE_POST_DX = 0.075


def _add_chrome_bar_handle(part, chrome, handle_z):
    """Add chrome bar handle on two posts at the given Z in part-local frame.
    Handle extends in +X from the outer face (local x=0)."""
    for i in range(2):
        s = 1 if i == 0 else -1
        part.visual(
            Box((0.016, 0.012, 0.012)),
            origin=Origin(xyz=(0.004, s * HANDLE_POST_DX, handle_z)),
            material=chrome,
            name=f"handle_post_{i}",
        )
    part.visual(
        Box((0.014, HANDLE_BAR_LEN, 0.016)),
        origin=Origin(xyz=(0.019, 0.0, handle_z)),
        material=chrome,
        name="handle_bar",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flap_door_bedside_nightstand")

    gray = model.material("carcass_gray", rgba=(0.50, 0.52, 0.55, 1.0))
    gray_dark = model.material("plinth_gray", rgba=(0.27, 0.28, 0.30, 1.0))
    pale = model.material("front_pale_satin", rgba=(0.85, 0.87, 0.88, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.80, 0.82, 0.85, 1.0))
    interior = model.material("interior_gray", rgba=(0.72, 0.74, 0.76, 1.0))

    # ===================================================================
    # ROOT: carcass (hollow shell with gallery lip and inset plinth)
    # ===================================================================
    carcass = model.part("carcass")

    # --- Inset plinth base (floating look) ---
    carcass.visual(
        Box((D - 2 * PLINTH_INSET, W - 2 * PLINTH_INSET, PLINTH_H)),
        origin=Origin(xyz=(D / 2.0, 0.0, PLINTH_H / 2.0)),
        material=gray_dark,
        name="plinth",
    )

    panel_h = H - BODY_BOT_Z

    # --- Two thick side panels (full depth, rising LIP above top shelf) ---
    for i in range(2):
        tag = 1 if i == 0 else -1
        carcass.visual(
            Box((D, WALL, panel_h)),
            origin=Origin(xyz=(D / 2.0, tag * (W / 2.0 - WALL / 2.0),
                               BODY_BOT_Z + panel_h / 2.0)),
            material=gray,
            name=f"side_panel_{i}",
        )

    # --- Back panel (full height, between side panels) ---
    carcass.visual(
        Box((WALL, INNER_W, panel_h)),
        origin=Origin(xyz=(WALL / 2.0, 0.0, BODY_BOT_Z + panel_h / 2.0)),
        material=gray,
        name="back_panel",
    )

    # --- Recessed top shelf (front edge flush, open front lip) ---
    carcass.visual(
        Box((D, INNER_W, SHELF_THK)),
        origin=Origin(xyz=(D / 2.0, 0.0, SHELF_BOT_Z + SHELF_THK / 2.0)),
        material=gray,
        name="top_shelf",
    )

    # --- Carcass bottom panel (floor of the cabinet interior) ---
    carcass.visual(
        Box((D - FACE_THK, INNER_W, 0.016)),
        origin=Origin(xyz=((D - FACE_THK) / 2.0, 0.0, BODY_BOT_Z + 0.008)),
        material=gray,
        name="bottom_panel",
    )

    # --- Interior shelf (visible when flap door is open) ---
    int_shelf_d = D - WALL - FACE_THK - 0.006  # stops short of door inner face
    int_shelf_z = BODY_BOT_Z + ZONE_H * 0.48
    carcass.visual(
        Box((int_shelf_d, INNER_W - 0.004, SHELF_THK)),
        origin=Origin(xyz=(WALL + int_shelf_d / 2.0, 0.0, int_shelf_z)),
        material=interior,
        name="interior_shelf",
    )

    carcass.inertial = Inertial.from_geometry(Box((D, W, H)), mass=22.0)

    # ===================================================================
    # FLAP DOOR (REVOLUTE, hinged at bottom front edge)
    # ===================================================================
    flap_door = model.part("flap_door")

    # Part frame origin at the hinge line (bottom center of door opening).
    # Door slab extends upward in +Z; outer face at local x=0 (flush with
    # carcass front when closed). Thickness extends toward -X (interior).
    flap_door.visual(
        Box((FACE_THK, DOOR_W, DOOR_H)),
        origin=Origin(xyz=(-FACE_THK / 2.0, 0.0, DOOR_H / 2.0)),
        material=pale,
        name="door_slab",
    )

    # Chrome bar handle near the top edge of the door.
    handle_z = DOOR_H - 0.040
    _add_chrome_bar_handle(flap_door, chrome, handle_z)

    flap_door.inertial = Inertial.from_geometry(
        Box((FACE_THK, DOOR_W, DOOR_H)), mass=3.0
    )

    # Revolute articulation: hinge at the bottom front edge.
    # axis=(0, +1, 0): right-hand rule about +Y curls +Z toward +X,
    # so positive q tilts the door top forward (+X) and downward.
    model.articulation(
        "carcass_to_flap_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=flap_door,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.5,
                                   lower=0.0, upper=1.50),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    flap_door = object_model.get_part("flap_door")
    j_flap = object_model.get_articulation("carcass_to_flap_door")

    # --- Grounding and true overall scale (~0.65 x 0.45 x 0.45 m) ---
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

    # --- Inset plinth (floating look): recessed from every carcass face ---
    plinth = ctx.part_element_world_aabb(carcass, elem="plinth")
    assert plinth is not None
    ctx.check(
        "plinth_inset",
        plinth[0][0] > cb[0][0] + 0.02 and plinth[1][0] < cb[1][0] - 0.02
        and plinth[0][1] > cb[0][1] + 0.02 and plinth[1][1] < cb[1][1] - 0.02,
        details=f"plinth x=({plinth[0][0]:.3f},{plinth[1][0]:.3f})",
    )

    # --- Gallery lip: side/back panels rise ~0.06 m above top shelf ---
    shelf = ctx.part_element_world_aabb(carcass, elem="top_shelf")
    back = ctx.part_element_world_aabb(carcass, elem="back_panel")
    side = ctx.part_element_world_aabb(carcass, elem="side_panel_0")
    assert shelf is not None and back is not None and side is not None
    lip_rise = side[1][2] - shelf[1][2]
    ctx.check("gallery_lip_rise", abs(lip_rise - 0.06) < 0.005,
              details=f"lip rise={lip_rise:.3f}")
    ctx.check("back_lip_matches_sides", abs(back[1][2] - side[1][2]) < 0.002,
              details=f"back top={back[1][2]:.3f}, side top={side[1][2]:.3f}")
    ctx.check("shelf_front_flush_open", abs(shelf[1][0] - cb[1][0]) < 0.003,
              details=f"shelf front x={shelf[1][0]:.3f}, carcass front={cb[1][0]:.3f}")

    # --- Flap door joint: revolute about Y axis, 0..1.50 rad ---
    ctx.check("flap_joint_revolute",
              j_flap.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("flap_joint_axis_y",
              abs(j_flap.axis[1]) > 0.99
              and abs(j_flap.axis[0]) < 0.01
              and abs(j_flap.axis[2]) < 0.01)
    ctx.check("flap_joint_lower_zero",
              abs(j_flap.motion_limits.lower) < 1e-9)
    ctx.check("flap_joint_upper",
              abs(j_flap.motion_limits.upper - 1.50) < 1e-6,
              details=f"upper={j_flap.motion_limits.upper}")

    # --- Closed pose: door slab flush with cabinet front ---
    front_x = cb[1][0]
    door_slab = ctx.part_element_world_aabb(flap_door, elem="door_slab")
    bar = ctx.part_element_world_aabb(flap_door, elem="handle_bar")
    assert door_slab is not None and bar is not None

    ctx.check("door_flush_closed", abs(door_slab[1][0] - front_x) < 0.002,
              details=f"door front x={door_slab[1][0]:.4f}, carcass={front_x:.4f}")

    # Chrome bar handle stands proud of the door face on its posts.
    ctx.check("handle_bar_proud_closed", bar[0][0] > door_slab[1][0] + 0.008,
              details=f"bar min x={bar[0][0]:.4f}")

    # Door spans the full front opening height (bottom near carcass bottom,
    # top near the shelf underside).
    ctx.check("door_covers_opening_height",
              door_slab[0][2] < BODY_BOT_Z + 0.005
              and door_slab[1][2] > SHELF_BOT_Z - 0.010,
              details=f"door z=({door_slab[0][2]:.3f},{door_slab[1][2]:.3f})")

    # Door fits within carcass width.
    ctx.expect_within(flap_door, carcass, axes="y", margin=0.001,
                      name="flap_door_within_width")

    # --- Interior shelf visible inside the carcass ---
    int_shelf = ctx.part_element_world_aabb(carcass, elem="interior_shelf")
    assert int_shelf is not None
    ctx.check("interior_shelf_inside",
              int_shelf[0][0] > cb[0][0] + 0.01 and int_shelf[1][0] < front_x - 0.01,
              details=f"shelf x=({int_shelf[0][0]:.3f},{int_shelf[1][0]:.3f})")

    # --- Open pose: door tilts forward and down ---
    closed_top_z = door_slab[1][2]
    with ctx.pose({j_flap: j_flap.motion_limits.upper}):
        open_slab = ctx.part_element_world_aabb(flap_door, elem="door_slab")
        assert open_slab is not None
        open_front_x = open_slab[1][0]
        open_top_z = open_slab[1][2]

    # Door top edge should have moved well forward of the carcass front.
    ctx.check("flap_opens_forward",
              open_front_x > front_x + 0.05,
              details=f"open front x={open_front_x:.3f}, carcass front={front_x:.3f}")

    # Door top edge should have dropped substantially (tilts downward).
    ctx.check("flap_opens_downward",
              open_top_z < closed_top_z - 0.05,
              details=f"open top z={open_top_z:.3f}, closed top z={closed_top_z:.3f}")

    return ctx.report()


object_model = build_object_model()
