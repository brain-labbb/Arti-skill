from __future__ import annotations

# Modern single-drawer bedside nightstand (matte medium-gray lacquered carcass,
# pale satin drawer front, polished-chrome bar handle).
#
# Variant of the two-drawer parent: one tall drawer instead of two stacked.
# Carcass shell, inset plinth base, chrome bar handle, and runner rails are
# identical to the parent; only the number of looped drawers changes.
#
# World layout: front faces +X (back at x=0, front at x=D), width along Y,
# height along +Z. The plinth rests on the floor at z=0 and is inset from all
# faces so the body appears to float. The carcass is a hollow shell: two thick
# side panels and a back panel rise ~0.06 m above the recessed top shelf,
# forming a three-sided gallery lip (front edge open/flush).
#
# Articulation: one PRISMATIC drawer sliding forward along +X, range 0 to
# 0.36 m, fully flush with the cabinet face at q=0. The drawer is a hollow
# open-top tray with a flat slab front and a chrome bar handle on two posts.
# Drawers are emitted via a for i in range(n) loop with drawer_{i} naming.

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
SHELF_BOT_Z = SHELF_TOP_Z - SHELF_THK

BODY_BOT_Z = PLINTH_H            # carcass panels start on top of the plinth
FACE_THK = 0.016                 # drawer front slab thickness
INNER_W = W - 2.0 * WALL         # cavity width between side panels

# ---------- drawer variant config ----------
DRAWER_COUNT = 1                 # single tall-drawer variant
TRAVEL = 0.360

# Drawer front zone: from the carcass bottom edge up to the top shelf underside.
FACE_TOP_MARGIN = 0.004
ZONE_H = SHELF_BOT_Z - BODY_BOT_Z  # available height for drawer fronts
FACE_GAP = 0.010                    # thin reveal between stacked fronts (n>1)

# Even vertical placement for n drawers within the zone.
FACE_H = (ZONE_H - FACE_TOP_MARGIN - FACE_GAP * max(DRAWER_COUNT - 1, 0)) / DRAWER_COUNT

# Drawer tray (hollow open-top box) dimensions scale with the front height.
BOX_D = 0.400
BOX_W = 0.600
BOX_H = min(FACE_H - 0.040, 0.280)  # tall tray but leave wall headroom
TRAY_T = 0.012
FACE_W = INNER_W - 0.004

# Center-Z positions for each drawer front (bottom to top).
DRAWER_CZ = [
    BODY_BOT_Z + FACE_H * (i + 0.5) + FACE_GAP * i
    for i in range(DRAWER_COUNT)
]


def _build_drawer(model: ArticulatedObject, name: str, face_h: float,
                  pale, chrome, tray_mat):
    """Hollow open-top tray + slab front + chrome bar handle, in drawer-local
    frame: slab front outer surface at local x=0, tray extends toward -X."""
    drawer = model.part(name)

    # Flat slab front (pale satin), spanning the carcass opening.
    drawer.visual(
        Box((FACE_THK, FACE_W, face_h)),
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
    handle_z = face_h * 0.30
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
    model = ArticulatedObject(name="single_drawer_bedside_nightstand")

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
    for s, tag in (("0", 1), ("1", -1)):
        carcass.visual(
            Box((D, WALL, panel_h)),
            origin=Origin(xyz=(D / 2.0, tag * (W / 2.0 - WALL / 2.0),
                               BODY_BOT_Z + panel_h / 2.0)),
            material=gray,
            name=f"side_panel_{s}",
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
    # Carcass bottom panel (recessed behind the drawer front).
    carcass.visual(
        Box((D - FACE_THK, INNER_W, 0.016)),
        origin=Origin(xyz=((D - FACE_THK) / 2.0, 0.0, BODY_BOT_Z + 0.008)),
        material=gray,
        name="bottom_panel",
    )

    # ===================================================================
    # DRAWERS (loop: DRAWER_COUNT independent PRISMATIC slides along +X)
    # ===================================================================
    drawers = []
    for i in range(DRAWER_COUNT):
        name = f"drawer_{i}"
        cz = DRAWER_CZ[i]

        drawer = _build_drawer(model, name, FACE_H, pale, chrome, tray_mat)
        drawers.append((drawer, cz, name))

        # Side-mounted runner rails supporting this drawer tray (just below the
        # tray bottom, fixed to the side panels).
        rail_h = 0.010
        rail_len = 0.400
        rail_top = cz - BOX_H / 2.0  # rails carry the tray bottoms (contact)
        for s, stag in (("0", 1), ("1", -1)):
            carcass.visual(
                Box((rail_len, 0.014, rail_h)),
                origin=Origin(xyz=(0.030 + rail_len / 2.0,
                                   stag * (INNER_W / 2.0 - 0.007),
                                   rail_top - rail_h / 2.0)),
                material=gray,
                name=f"runner_rail_{i}_{s}",
            )

    carcass.inertial = Inertial.from_geometry(Box((D, W, H)), mass=22.0)

    # Articulate each drawer with a uniform prismatic joint policy.
    for drawer, cz, name in drawers:
        model.articulation(
            f"carcass_to_{name}",
            ArticulationType.PRISMATIC,
            parent=carcass,
            child=drawer,
            origin=Origin(xyz=(D, 0.0, cz)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=40.0, velocity=0.5,
                                       lower=0.0, upper=TRAVEL),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")

    # Collect drawers and joints from the loop.
    drawer_parts = []
    drawer_joints = []
    for i in range(DRAWER_COUNT):
        drawer_parts.append(object_model.get_part(f"drawer_{i}"))
        drawer_joints.append(object_model.get_articulation(f"carcass_to_drawer_{i}"))

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

    # --- Single-drawer variant: exactly one tall drawer ---
    ctx.check("drawer_count_is_one", DRAWER_COUNT == 1,
              details=f"drawer_count={DRAWER_COUNT}")

    # --- Tall drawer front fills the zone (no horizontal reveal gap) ---
    d0 = drawer_parts[0]
    face0 = ctx.part_element_world_aabb(d0, elem="front_slab")
    assert face0 is not None
    front_height = face0[1][2] - face0[0][2]
    ctx.check("tall_drawer_front",
              front_height > 0.20,
              details=f"front height={front_height:.3f}")

    # --- Drawer joints: prismatic, forward +X, 0..0.36 m ---
    for j in drawer_joints:
        ctx.check(f"{j.name}_prismatic",
                  j.articulation_type == ArticulationType.PRISMATIC)
        ctx.check(f"{j.name}_axis_x", abs(j.axis[0]) > 0.99 and abs(j.axis[2]) < 0.01)
        ctx.check(f"{j.name}_range",
                  abs(j.motion_limits.lower) < 1e-9
                  and abs(j.motion_limits.upper - 0.36) < 1e-6,
                  details=f"range=({j.motion_limits.lower}, {j.motion_limits.upper})")

    # --- Closed pose: fronts flush with cabinet face, trays nested inside ---
    front_x = cb[1][0]
    for i, (d, j) in enumerate(zip(drawer_parts, drawer_joints)):
        name = f"drawer_{i}"
        face = ctx.part_element_world_aabb(d, elem="front_slab")
        tray = ctx.part_element_world_aabb(d, elem="tray_bottom")
        bar = ctx.part_element_world_aabb(d, elem="handle_bar")
        assert face is not None and tray is not None and bar is not None
        ctx.check(f"{name}_front_flush_closed", abs(face[1][0] - front_x) < 0.002,
                  details=f"face front x={face[1][0]:.4f}, carcass front={front_x:.4f}")
        ctx.check(f"{name}_tray_nested_closed",
                  tray[1][0] < front_x and tray[0][0] > cb[0][0] + 0.01,
                  details=f"tray x=({tray[0][0]:.3f},{tray[1][0]:.3f})")
        # Chrome bar stands proud of the slab face on its posts.
        ctx.check(f"{name}_handle_bar_proud", bar[0][0] > face[1][0] + 0.008,
                  details=f"bar min x={bar[0][0]:.4f}")
        # Hollow open-top tray: side walls rise above the tray bottom.
        wall0 = ctx.part_element_world_aabb(d, elem="tray_side_wall_0")
        wall1 = ctx.part_element_world_aabb(d, elem="tray_side_wall_1")
        assert wall0 is not None and wall1 is not None
        ctx.check(f"{name}_tray_open_top",
                  wall0[1][2] > tray[1][2] + 0.05,
                  details=f"wall top={wall0[1][2]:.3f}, bottom top={tray[1][2]:.3f}")
        ctx.expect_within(d, carcass, axes="y", margin=0.001,
                          name=f"{name}_within_width")

    # --- Open pose: drawer slides forward 0.36 m and retains insertion ---
    for i, (d, j) in enumerate(zip(drawer_parts, drawer_joints)):
        name = f"drawer_{i}"
        rest = ctx.part_world_position(d)
        with ctx.pose({j: j.motion_limits.upper}):
            out = ctx.part_world_position(d)
            tray = ctx.part_element_world_aabb(d, elem="tray_back_wall")
            assert tray is not None
            rear_x = tray[0][0]
        assert rest is not None and out is not None
        ctx.check(f"{name}_slides_forward",
                  abs((out[0] - rest[0]) - 0.36) < 1e-6,
                  details=f"dx={out[0] - rest[0]:.4f}")
        ctx.check(f"{name}_retains_insertion", rear_x < front_x - 0.02,
                  details=f"open rear x={rear_x:.3f}, front={front_x:.3f}")

    # --- Runner rails present for each drawer ---
    for i in range(DRAWER_COUNT):
        for s in ("0", "1"):
            rail_name = f"runner_rail_{i}_{s}"
            rail = ctx.part_element_world_aabb(carcass, elem=rail_name)
            ctx.check(f"{rail_name}_exists", rail is not None,
                      details=f"runner rail {rail_name} not found")

    return ctx.report()


object_model = build_object_model()
