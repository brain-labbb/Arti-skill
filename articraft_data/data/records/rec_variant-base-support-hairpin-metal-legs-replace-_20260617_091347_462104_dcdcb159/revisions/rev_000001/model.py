from __future__ import annotations

# Modern two-drawer bedside nightstand — hairpin metal legs variant.
#
# World layout: front faces +X (back at x=0, front at x=D), width along Y,
# height along +Z. Four thin bent-steel hairpin legs at the corners support
# the carcass, replacing the parent's inset plinth. The carcass is a hollow
# shell: two thick side panels and a back panel rise ~0.06 m above the
# recessed top shelf, forming a three-sided gallery lip (front edge open/flush).
#
# Articulation: two independent PRISMATIC drawers sliding forward along +X,
# range 0 to 0.36 m, fully flush with the cabinet face at q=0. Each drawer is
# a hollow open-top tray with a flat slab front and a chrome bar handle on two
# posts.
#
# Variant change: base_support = hairpin_metal_legs (replaces plinth).
# All carcass internals (drawers, gallery shell, handles, rails) are identical
# to the parent; only the base support layer changes.

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------- key dimensions (meters) ----------
W = 0.650            # overall width (Y)
D = 0.450            # overall depth (X)
WALL = 0.018         # thick carcass panel thickness
LIP = 0.060          # gallery lip rise above the recessed top shelf

# Hairpin leg dimensions (variant base support)
LEG_H = 0.120        # leg height (floor to carcass bottom)
ROD_R = 0.005        # rod radius (~10 mm diameter steel rod)
SPLAY = 0.052        # distance between the two prong mount points
LEG_INSET_X = 0.036  # leg center inset from front/back carcass edges
LEG_INSET_Y = 0.040  # leg center inset from side carcass edges

SHELF_THK = 0.016
BODY_BOT_Z = LEG_H   # carcass panels start on top of the legs
# Keep carcass body height identical to parent (parent: H - PLINTH_H = 0.405).
CARCASS_BODY_H = 0.405
H = LEG_H + CARCASS_BODY_H   # overall height including legs
SHELF_TOP_Z = H - LIP
SHELF_BOT_Z = SHELF_TOP_Z - SHELF_THK

FACE_THK = 0.016                 # drawer front slab thickness
INNER_W = W - 2.0 * WALL         # cavity width between side panels

# Drawer front zone: from the carcass bottom edge up to the top shelf underside.
FACE_GAP = 0.010                 # thin horizontal reveal between the two fronts
FACE_TOP_MARGIN = 0.004
ZONE_H = SHELF_BOT_Z - BODY_BOT_Z
FACE_H = (ZONE_H - FACE_GAP - FACE_TOP_MARGIN) / 2.0
CZ_BOTTOM = BODY_BOT_Z + FACE_H / 2.0
CZ_TOP = BODY_BOT_Z + FACE_H + FACE_GAP + FACE_H / 2.0

# Drawer tray (hollow open-top box).
BOX_D = 0.400
BOX_W = 0.600
BOX_H = 0.112
TRAY_T = 0.012
FACE_W = INNER_W - 0.004
TRAVEL = 0.360


def _build_hairpin_leg_mesh(cx: float, cy: float):
    """Build one U-shaped hairpin leg rod at corner position (cx, cy).

    The rod bends in the XZ plane: two prongs splay apart along X,
    meeting at a smooth floor-level bend. Returns a MeshGeometry.
    """
    hs = SPLAY / 2.0
    # Spline control points: prong1 mount → curve → floor bend → curve → prong2 mount
    points = [
        (cx - hs,             cy, LEG_H),
        (cx - hs * 0.84,      cy, LEG_H * 0.70),
        (cx - hs * 0.52,      cy, LEG_H * 0.36),
        (cx - hs * 0.18,      cy, LEG_H * 0.08),
        (cx,                  cy, ROD_R),
        (cx + hs * 0.18,      cy, LEG_H * 0.08),
        (cx + hs * 0.52,      cy, LEG_H * 0.36),
        (cx + hs * 0.84,      cy, LEG_H * 0.70),
        (cx + hs,             cy, LEG_H),
    ]
    return tube_from_spline_points(
        points,
        radius=ROD_R,
        samples_per_segment=18,
        radial_segments=12,
        cap_ends=True,
    )


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
    model = ArticulatedObject(name="two_drawer_bedside_nightstand")

    gray = model.material("carcass_gray", rgba=(0.50, 0.52, 0.55, 1.0))
    pale = model.material("front_pale_satin", rgba=(0.85, 0.87, 0.88, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.80, 0.82, 0.85, 1.0))
    tray_mat = model.material("tray_gray", rgba=(0.72, 0.74, 0.76, 1.0))
    steel = model.material("brushed_steel", rgba=(0.30, 0.31, 0.34, 1.0))

    # ===================================================================
    # ROOT: carcass (hollow shell with gallery lip, on hairpin legs)
    # ===================================================================
    carcass = model.part("carcass")

    # --- Hairpin legs (4 corners, inline non-moving decorations) ---
    corners_xy = [
        (LEG_INSET_X,       -(W / 2.0 - LEG_INSET_Y)),   # back, -Y side
        (LEG_INSET_X,       +(W / 2.0 - LEG_INSET_Y)),   # back, +Y side
        (D - LEG_INSET_X,   -(W / 2.0 - LEG_INSET_Y)),   # front, -Y side
        (D - LEG_INSET_X,   +(W / 2.0 - LEG_INSET_Y)),   # front, +Y side
    ]
    # Shared geometry helper + loop with name_{i} naming
    for i in range(4):
        cx, cy = corners_xy[i]
        leg_mesh = _build_hairpin_leg_mesh(cx, cy)
        carcass.visual(
            mesh_from_geometry(leg_mesh, f"hairpin_leg_{i}"),
            material=steel,
            name=f"hairpin_leg_{i}",
        )
        # Small mounting plate (flat steel bracket between leg and carcass bottom)
        carcass.visual(
            Box((SPLAY + 0.014, 0.024, 0.004)),
            origin=Origin(xyz=(cx, cy, LEG_H - 0.002)),
            material=steel,
            name=f"mount_plate_{i}",
        )

    # --- Carcass panels (identical construction to parent) ---
    panel_h = H - BODY_BOT_Z   # = CARCASS_BODY_H = 0.405
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
    # Carcass bottom panel (recessed behind the bottom drawer front).
    carcass.visual(
        Box((D - FACE_THK, INNER_W, 0.016)),
        origin=Origin(xyz=((D - FACE_THK) / 2.0, 0.0, BODY_BOT_Z + 0.008)),
        material=gray,
        name="bottom_panel",
    )

    # Side-mounted runner rails supporting each drawer tray (just below the
    # tray bottoms, fixed to the side panels).
    rail_h = 0.010
    rail_len = 0.400
    for cz, dtag in ((CZ_BOTTOM, "bottom"), (CZ_TOP, "top")):
        rail_top = cz - BOX_H / 2.0  # rails carry the tray bottoms (contact)
        for s, stag in (("0", 1), ("1", -1)):
            carcass.visual(
                Box((rail_len, 0.014, rail_h)),
                origin=Origin(xyz=(0.030 + rail_len / 2.0,
                                   stag * (INNER_W / 2.0 - 0.007),
                                   rail_top - rail_h / 2.0)),
                material=gray,
                name=f"{dtag}_runner_rail_{s}",
            )

    carcass.inertial = Inertial.from_geometry(Box((D, W, H)), mass=22.0)

    # ===================================================================
    # DRAWERS (independent PRISMATIC slides along +X)
    # ===================================================================
    top_drawer = _build_drawer(model, "top_drawer", pale, chrome, tray_mat)
    bottom_drawer = _build_drawer(model, "bottom_drawer", pale, chrome, tray_mat)

    for drawer, cz, tag in ((top_drawer, CZ_TOP, "top"),
                            (bottom_drawer, CZ_BOTTOM, "bottom")):
        model.articulation(
            f"carcass_to_{tag}_drawer",
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
    top_drawer = object_model.get_part("top_drawer")
    bottom_drawer = object_model.get_part("bottom_drawer")
    j_top = object_model.get_articulation("carcass_to_top_drawer")
    j_bottom = object_model.get_articulation("carcass_to_bottom_drawer")

    # --- Grounding and overall scale ---
    cb = ctx.part_world_aabb(carcass)
    assert cb is not None
    ctx.check("legs_on_floor", cb[0][2] < 0.010,
              details=f"carcass min z={cb[0][2]:.4f}")
    width_y = cb[1][1] - cb[0][1]
    depth_x = cb[1][0] - cb[0][0]
    height_z = cb[1][2] - cb[0][2]
    ctx.check("width_065", abs(width_y - 0.65) < 0.015, details=f"w={width_y:.3f}")
    ctx.check("depth_045", abs(depth_x - 0.45) < 0.015, details=f"d={depth_x:.3f}")
    # Total height: legs (0.12) + carcass body (0.405) ≈ 0.525 m
    ctx.check("height_with_legs", 0.48 < height_z < 0.58,
              details=f"h={height_z:.3f}")

    # --- Hairpin legs: 4 bent-steel rods at corners, touching the floor ---
    corners_expected_x = [LEG_INSET_X, LEG_INSET_X,
                          D - LEG_INSET_X, D - LEG_INSET_X]
    for i in range(4):
        leg = ctx.part_element_world_aabb(carcass, elem=f"hairpin_leg_{i}")
        assert leg is not None, f"hairpin_leg_{i} not found"
        # Each leg reaches the floor
        ctx.check(f"leg_{i}_touches_floor", leg[0][2] < 0.012,
                  details=f"leg {i} min z={leg[0][2]:.4f}")
        # Each leg is approximately LEG_H tall
        leg_h_measured = leg[1][2] - leg[0][2]
        ctx.check(f"leg_{i}_height",
                  abs(leg_h_measured - LEG_H) < 0.025,
                  details=f"leg {i} height={leg_h_measured:.3f}")
        # Each leg is near its expected corner X position
        leg_cx = (leg[0][0] + leg[1][0]) / 2.0
        ctx.check(f"leg_{i}_corner_x",
                  abs(leg_cx - corners_expected_x[i]) < 0.04,
                  details=f"leg {i} cx={leg_cx:.3f}, expected ~{corners_expected_x[i]:.3f}")
        # Mounting plate exists for each leg
        plate = ctx.part_element_world_aabb(carcass, elem=f"mount_plate_{i}")
        ctx.check(f"leg_{i}_has_mount_plate", plate is not None)

    # Legs are thin rods, not box placeholders: check that leg Y extent is
    # narrow (rod diameter ~2*ROD_R = 0.010 m, not the full carcass width).
    for i in range(4):
        leg = ctx.part_element_world_aabb(carcass, elem=f"hairpin_leg_{i}")
        assert leg is not None
        leg_dy = leg[1][1] - leg[0][1]
        ctx.check(f"leg_{i}_thin_rod",
                  leg_dy < 0.030,
                  details=f"leg {i} Y extent={leg_dy:.4f} (should be rod-thin)")

    # --- Gallery lip: side/back panels rise ~0.06 m above the top shelf ---
    shelf = ctx.part_element_world_aabb(carcass, elem="top_shelf")
    side = ctx.part_element_world_aabb(carcass, elem="side_panel_0")
    back = ctx.part_element_world_aabb(carcass, elem="back_panel")
    assert shelf is not None and side is not None and back is not None
    lip_rise = side[1][2] - shelf[1][2]
    ctx.check("gallery_lip_rise", abs(lip_rise - 0.06) < 0.005,
              details=f"lip rise={lip_rise:.3f}")
    ctx.check("back_lip_matches_sides", abs(back[1][2] - side[1][2]) < 0.002,
              details=f"back top={back[1][2]:.3f}, side top={side[1][2]:.3f}")
    # Front edge open: the shelf front face is flush with the cabinet front.
    ctx.check("shelf_front_flush_open", abs(shelf[1][0] - cb[1][0]) < 0.003,
              details=f"shelf front x={shelf[1][0]:.3f}, carcass front={cb[1][0]:.3f}")

    # --- Drawer joints: independent prismatic, forward +X, 0..0.36 m ---
    for j in (j_top, j_bottom):
        ctx.check(f"{j.name}_prismatic",
                  j.articulation_type == ArticulationType.PRISMATIC)
        ctx.check(f"{j.name}_axis_x", abs(j.axis[0]) > 0.99 and abs(j.axis[2]) < 0.01)
        ctx.check(f"{j.name}_range",
                  abs(j.motion_limits.lower) < 1e-9
                  and abs(j.motion_limits.upper - 0.36) < 1e-6,
                  details=f"range=({j.motion_limits.lower}, {j.motion_limits.upper})")

    # --- Stacked fronts: top drawer above bottom drawer, thin reveal between ---
    top_face = ctx.part_element_world_aabb(top_drawer, elem="front_slab")
    bot_face = ctx.part_element_world_aabb(bottom_drawer, elem="front_slab")
    assert top_face is not None and bot_face is not None
    ctx.check("equal_height_fronts",
              abs((top_face[1][2] - top_face[0][2])
                  - (bot_face[1][2] - bot_face[0][2])) < 0.002)
    reveal = top_face[0][2] - bot_face[1][2]
    ctx.check("thin_reveal_between_fronts", 0.004 < reveal < 0.02,
              details=f"reveal={reveal:.4f}")

    # --- Closed pose: fronts flush with cabinet face, trays nested inside ---
    front_x = cb[1][0]
    for name, d in (("top", top_drawer), ("bottom", bottom_drawer)):
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
        interior_w = min(wall0[0][1], wall1[0][1]) * 0.0 + (
            max(wall0[0][1], wall1[0][1]) - min(wall0[1][1], wall1[1][1])
        )
        ctx.check(f"{name}_tray_hollow_interior", abs(interior_w) > 0.5,
                  details=f"interior width={interior_w:.3f}")
        ctx.expect_within(d, carcass, axes="y", margin=0.001,
                          name=f"{name}_drawer_within_width")

    # --- Open pose: each drawer slides forward 0.36 m and retains insertion ---
    for name, d, j in (("top", top_drawer, j_top), ("bottom", bottom_drawer, j_bottom)):
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

    # --- Independence: opening the top drawer leaves the bottom drawer shut ---
    with ctx.pose({j_top: 0.30}):
        bf = ctx.part_element_world_aabb(bottom_drawer, elem="front_slab")
        assert bf is not None
        ctx.check("drawers_independent", abs(bf[1][0] - front_x) < 0.002,
                  details=f"bottom face x={bf[1][0]:.4f} with top open")

    return ctx.report()


object_model = build_object_model()
