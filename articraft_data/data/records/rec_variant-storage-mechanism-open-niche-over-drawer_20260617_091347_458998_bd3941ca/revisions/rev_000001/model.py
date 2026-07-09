from __future__ import annotations

# Modern bedside nightstand variant: open display niche over one drawer.
# (matte medium-gray lacquered carcass, pale satin drawer front, polished-chrome
# bar handle).
#
# World layout: front faces +X (back at x=0, front at x=D), width along Y,
# height along +Z. The plinth rests on the floor at z=0 and is inset from all
# faces so the body appears to float. The carcass is a hollow shell: two thick
# side panels and a back panel rise ~0.06 m above the recessed top shelf,
# forming a three-sided gallery lip (front edge open/flush).
#
# Upper zone: fixed open display niche (no door or drawer), open at the front.
# Lower zone: one prismatic drawer sliding forward along +X, range 0..0.36 m,
# fully flush with the cabinet face at q=0. A horizontal divider shelf
# separates the niche from the drawer zone.

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
FACE_THK = 0.016                 # drawer front slab thickness
INNER_W = W - 2.0 * WALL         # cavity width between side panels

# ---------- zone split: niche over drawer ----------
NICHE_H = 0.155                  # open niche interior height
DIVIDER_THK = SHELF_THK          # horizontal divider shelf thickness

# Niche: open from NICHE_BOT_Z to SHELF_BOT_Z (front is open).
NICHE_BOT_Z = SHELF_BOT_Z - NICHE_H       # 0.219
DIVIDER_TOP_Z = NICHE_BOT_Z               # top of divider shelf
DIVIDER_BOT_Z = NICHE_BOT_Z - DIVIDER_THK # 0.203

# Drawer zone: from BODY_BOT_Z up to DIVIDER_BOT_Z.
DRAWER_ZONE_H = DIVIDER_BOT_Z - BODY_BOT_Z  # 0.158
FACE_TOP_MARGIN = 0.004
FACE_H = DRAWER_ZONE_H - FACE_TOP_MARGIN   # drawer front slab height
CZ_DRAWER = BODY_BOT_Z + FACE_H / 2.0      # drawer front center Z

# Drawer tray (hollow open-top box).
BOX_D = 0.400
BOX_W = 0.600
BOX_H = 0.112
TRAY_T = 0.012
FACE_W = INNER_W - 0.004
TRAVEL = 0.360


def _build_drawer(model: ArticulatedObject, name: str, pale, chrome, tray_mat):
    """Hollow open-top tray + slab front + chrome bar handle, in drawer-local
    frame: slab front outer surface at local x=0, tray extends toward -X."""
    drawer = model.part(name)

    # Flat slab front (pale satin), spanning the carcass opening.
    drawer.visual(
        Box((FACE_THK, FACE_W, FACE_H)),
        origin=Origin(xyz=(-FACE_THK / 2.0, 0.0, 0.0)),
        material=pale,
        name="front_slab",
    )

    # --- hollow open-top tray ---
    box_back_x = -(FACE_THK + BOX_D)
    box_cx = -(FACE_THK + BOX_D / 2.0)
    box_bot = -BOX_H / 2.0
    # Tray bottom panel (full footprint).
    drawer.visual(
        Box((BOX_D, BOX_W, TRAY_T)),
        origin=Origin(xyz=(box_cx, 0.0, box_bot + TRAY_T / 2.0)),
        material=tray_mat,
        name="tray_bottom",
    )
    wall_h = BOX_H - TRAY_T + 0.002          # walls sit on the bottom, 2 mm embed
    wall_cz = box_bot + TRAY_T - 0.002 + wall_h / 2.0
    # Tray back wall.
    drawer.visual(
        Box((TRAY_T, BOX_W, wall_h)),
        origin=Origin(xyz=(box_back_x + TRAY_T / 2.0, 0.0, wall_cz)),
        material=tray_mat,
        name="tray_back_wall",
    )
    # Tray side walls (embed 2 mm into the slab back for solid attachment).
    side_len = BOX_D + 0.002
    for s, tag in ((1, "0"), (-1, "1")):
        drawer.visual(
            Box((side_len, TRAY_T, wall_h)),
            origin=Origin(
                xyz=(-FACE_THK + 0.002 - side_len / 2.0,
                     s * (BOX_W / 2.0 - TRAY_T / 2.0), wall_cz)
            ),
            material=tray_mat,
            name=f"tray_side_wall_{tag}",
        )
    # Tray front wall behind the slab (closes the tray; slab is wider/taller).
    drawer.visual(
        Box((TRAY_T, BOX_W, wall_h)),
        origin=Origin(xyz=(-FACE_THK - TRAY_T / 2.0 + 0.002, 0.0, wall_cz)),
        material=tray_mat,
        name="tray_front_wall",
    )

    # --- polished-chrome bar handle on two posts, near the top edge ---
    handle_z = FACE_H * 0.30
    bar_len = 0.180
    post_dx = 0.075
    for s, tag in ((1, "0"), (-1, "1")):
        drawer.visual(
            Box((0.016, 0.012, 0.012)),
            origin=Origin(xyz=(0.004, s * post_dx, handle_z)),
            material=chrome,
            name=f"handle_post_{tag}",
        )
    drawer.visual(
        Box((0.014, bar_len, 0.016)),
        origin=Origin(xyz=(0.019, 0.0, handle_z)),
        material=chrome,
        name="handle_bar",
    )

    drawer.inertial = Inertial.from_geometry(Box((BOX_D, BOX_W, BOX_H)), mass=4.0)
    return drawer


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="niche_over_drawer_nightstand")

    gray = model.material("carcass_gray", rgba=(0.50, 0.52, 0.55, 1.0))
    gray_dark = model.material("plinth_gray", rgba=(0.27, 0.28, 0.30, 1.0))
    pale = model.material("front_pale_satin", rgba=(0.85, 0.87, 0.88, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.80, 0.82, 0.85, 1.0))
    tray_mat = model.material("tray_gray", rgba=(0.72, 0.74, 0.76, 1.0))

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
    for s, tag in ((1, "0"), (-1, "1")):
        carcass.visual(
            Box((D, WALL, panel_h)),
            origin=Origin(xyz=(D / 2.0, s * (W / 2.0 - WALL / 2.0),
                               BODY_BOT_Z + panel_h / 2.0)),
            material=gray,
            name=f"side_panel_{tag}",
        )
    # Back panel (full height, between the side panels).
    carcass.visual(
        Box((WALL, INNER_W, panel_h)),
        origin=Origin(xyz=(WALL / 2.0, 0.0, BODY_BOT_Z + panel_h / 2.0)),
        material=gray,
        name="back_panel",
    )
    # Recessed top shelf, front edge flush with the cabinet face (open front lip).
    carcass.visual(
        Box((D, INNER_W, SHELF_THK)),
        origin=Origin(xyz=(D / 2.0, 0.0, SHELF_BOT_Z + SHELF_THK / 2.0)),
        material=gray,
        name="top_shelf",
    )

    # Horizontal divider shelf between the open niche and the drawer zone.
    carcass.visual(
        Box((D - FACE_THK, INNER_W, DIVIDER_THK)),
        origin=Origin(xyz=((D - FACE_THK) / 2.0, 0.0,
                           DIVIDER_BOT_Z + DIVIDER_THK / 2.0)),
        material=gray,
        name="divider_shelf",
    )

    # Carcass bottom panel (recessed behind the drawer front).
    carcass.visual(
        Box((D - FACE_THK, INNER_W, 0.016)),
        origin=Origin(xyz=((D - FACE_THK) / 2.0, 0.0, BODY_BOT_Z + 0.008)),
        material=gray,
        name="bottom_panel",
    )

    # Side-mounted runner rails supporting the drawer tray (just below the
    # tray bottom, fixed to the side panels).
    rail_h = 0.010
    rail_len = 0.400
    rail_top = CZ_DRAWER - BOX_H / 2.0  # rails carry the tray bottom (contact)
    for s, stag in (("0", 1), ("1", -1)):
        carcass.visual(
            Box((rail_len, 0.014, rail_h)),
            origin=Origin(xyz=(0.030 + rail_len / 2.0,
                               stag * (INNER_W / 2.0 - 0.007),
                               rail_top - rail_h / 2.0)),
            material=gray,
            name=f"runner_rail_{s}",
        )

    carcass.inertial = Inertial.from_geometry(Box((D, W, H)), mass=22.0)

    # ===================================================================
    # DRAWER (PRISMATIC slide along +X, 0..0.36 m)
    # ===================================================================
    drawer = _build_drawer(model, "drawer", pale, chrome, tray_mat)

    model.articulation(
        "carcass_to_drawer",
        ArticulationType.PRISMATIC,
        parent=carcass,
        child=drawer,
        origin=Origin(xyz=(D, 0.0, CZ_DRAWER)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.5,
                                   lower=0.0, upper=TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    drawer = object_model.get_part("drawer")
    j_drawer = object_model.get_articulation("carcass_to_drawer")

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
    side = ctx.part_element_world_aabb(carcass, elem="side_panel_0")
    assert plinth is not None and side is not None
    ctx.check(
        "plinth_inset",
        plinth[0][0] > cb[0][0] + 0.02 and plinth[1][0] < cb[1][0] - 0.02
        and plinth[0][1] > cb[0][1] + 0.02 and plinth[1][1] < cb[1][1] - 0.02,
        details=f"plinth x=({plinth[0][0]:.3f},{plinth[1][0]:.3f})",
    )

    # --- Gallery lip: side/back panels rise ~0.06 m above the top shelf ---
    shelf = ctx.part_element_world_aabb(carcass, elem="top_shelf")
    back = ctx.part_element_world_aabb(carcass, elem="back_panel")
    assert shelf is not None and back is not None
    lip_rise = side[1][2] - shelf[1][2]
    ctx.check("gallery_lip_rise", abs(lip_rise - 0.06) < 0.005,
              details=f"lip rise={lip_rise:.3f}")
    ctx.check("back_lip_matches_sides", abs(back[1][2] - side[1][2]) < 0.002,
              details=f"back top={back[1][2]:.3f}, side top={side[1][2]:.3f}")
    # Front edge open: the shelf front face is flush with the cabinet front.
    ctx.check("shelf_front_flush_open", abs(shelf[1][0] - cb[1][0]) < 0.003,
              details=f"shelf front x={shelf[1][0]:.3f}, carcass front={cb[1][0]:.3f}")

    # --- Open niche: upper zone is a fixed open display compartment ---
    divider = ctx.part_element_world_aabb(carcass, elem="divider_shelf")
    assert divider is not None
    # Divider shelf sits between niche and drawer zone.
    niche_height = shelf[0][2] - divider[1][2]
    ctx.check("niche_open_height",
              0.10 < niche_height < 0.20,
              details=f"niche interior height={niche_height:.3f}")
    # Niche front is open: no drawer or door part occupies the niche opening.
    # The niche opening spans the inner width between side panels.
    niche_width = divider[1][1] - divider[0][1]
    ctx.check("niche_spans_inner_width",
              abs(niche_width - INNER_W) < 0.005,
              details=f"niche width={niche_width:.3f}, inner_w={INNER_W:.3f}")

    # --- Drawer joint: prismatic, forward +X, 0..0.36 m ---
    ctx.check("drawer_joint_prismatic",
              j_drawer.articulation_type == ArticulationType.PRISMATIC)
    ctx.check("drawer_joint_axis_x",
              abs(j_drawer.axis[0]) > 0.99 and abs(j_drawer.axis[2]) < 0.01)
    ctx.check("drawer_joint_range",
              abs(j_drawer.motion_limits.lower) < 1e-9
              and abs(j_drawer.motion_limits.upper - 0.36) < 1e-6,
              details=f"range=({j_drawer.motion_limits.lower}, {j_drawer.motion_limits.upper})")

    # --- Drawer sits below the divider shelf ---
    face = ctx.part_element_world_aabb(drawer, elem="front_slab")
    assert face is not None
    ctx.check("drawer_below_divider",
              face[1][2] < divider[0][2] + 0.005,
              details=f"drawer top z={face[1][2]:.3f}, divider bot z={divider[0][2]:.3f}")

    # --- Closed pose: front flush with cabinet face, tray nested inside ---
    front_x = cb[1][0]
    tray = ctx.part_element_world_aabb(drawer, elem="tray_bottom")
    bar = ctx.part_element_world_aabb(drawer, elem="handle_bar")
    assert tray is not None and bar is not None
    ctx.check("front_flush_closed", abs(face[1][0] - front_x) < 0.002,
              details=f"face front x={face[1][0]:.4f}, carcass front={front_x:.4f}")
    ctx.check("tray_nested_closed",
              tray[1][0] < front_x and tray[0][0] > cb[0][0] + 0.01,
              details=f"tray x=({tray[0][0]:.3f},{tray[1][0]:.3f})")
    # Chrome bar stands proud of the slab face on its posts.
    ctx.check("handle_bar_proud", bar[0][0] > face[1][0] + 0.008,
              details=f"bar min x={bar[0][0]:.4f}")
    # Hollow open-top tray: side walls rise above the tray bottom.
    wall0 = ctx.part_element_world_aabb(drawer, elem="tray_side_wall_0")
    wall1 = ctx.part_element_world_aabb(drawer, elem="tray_side_wall_1")
    assert wall0 is not None and wall1 is not None
    ctx.check("tray_open_top",
              wall0[1][2] > tray[1][2] + 0.05,
              details=f"wall top={wall0[1][2]:.3f}, bottom top={tray[1][2]:.3f}")
    ctx.expect_within(drawer, carcass, axes="y", margin=0.001,
                      name="drawer_within_width")

    # --- Open pose: drawer slides forward 0.36 m and retains insertion ---
    rest = ctx.part_world_position(drawer)
    with ctx.pose({j_drawer: j_drawer.motion_limits.upper}):
        out = ctx.part_world_position(drawer)
        tray_back = ctx.part_element_world_aabb(drawer, elem="tray_back_wall")
        assert tray_back is not None
        rear_x = tray_back[0][0]
    assert rest is not None and out is not None
    ctx.check("drawer_slides_forward",
              abs((out[0] - rest[0]) - 0.36) < 1e-6,
              details=f"dx={out[0] - rest[0]:.4f}")
    ctx.check("drawer_retains_insertion", rear_x < front_x - 0.02,
              details=f"open rear x={rear_x:.3f}, front={front_x:.3f}")

    # --- Niche remains unchanged when drawer opens ---
    with ctx.pose({j_drawer: 0.30}):
        div_open = ctx.part_element_world_aabb(carcass, elem="divider_shelf")
        assert div_open is not None
        ctx.check("niche_fixed_when_drawer_opens",
                  abs(div_open[1][2] - divider[1][2]) < 0.001,
                  details=f"divider z stable at {div_open[1][2]:.4f}")

    return ctx.report()


object_model = build_object_model()
