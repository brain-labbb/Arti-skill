from __future__ import annotations

# Modern two-drawer bedside nightstand — recessed finger-pull variant.
# Matte medium-gray lacquered carcass with pale satin drawer fronts.
# Each drawer front carries an integrated finger-pull groove recessed
# into the top edge (no protruding hardware).
#
# World layout: front faces +X (back at x=0, front at x=D), width along Y,
# height along +Z. The plinth rests on the floor at z=0 and is inset from
# all faces so the body appears to float. The carcass is a hollow shell:
# two thick side panels and a back panel rise ~0.06 m above the recessed
# top shelf, forming a three-sided gallery lip (front edge open/flush).
#
# Articulation: two independent PRISMATIC drawers sliding forward along +X,
# range 0 to 0.36 m, fully flush with the cabinet face at q=0. Each drawer
# is a hollow open-top tray with a CadQuery slab front carrying an
# integrated finger-pull groove at the top edge.

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq

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

# Finger-pull groove dimensions.
GROOVE_W = 0.140          # horizontal span of the pull slot
GROOVE_H = 0.014          # vertical height of the slot opening
GROOVE_DEPTH = 0.010      # depth of the pocket into the slab
GROOVE_LIP = 0.001        # thin lip above groove on the front face


def _finger_pull_slab(name: str):
    """CadQuery front slab with integrated finger-pull groove at the top edge.

    The slab is centered at the local origin (CadQuery frame):
      x in [-FACE_THK/2, +FACE_THK/2]
      y in [-FACE_W/2,   +FACE_W/2]
      z in [-FACE_H/2,   +FACE_H/2]
    The groove is a rounded stadium-shaped pocket cut into the +X (front)
    face, positioned near the top edge with a thin lip above.
    """
    slab = cq.Workplane("XY").box(FACE_THK, FACE_W, FACE_H)
    groove_center_z = FACE_H / 2.0 - GROOVE_LIP - GROOVE_H / 2.0
    slab = (
        slab.faces(">X").workplane()
        .center(0.0, groove_center_z)
        .slot2D(GROOVE_W, GROOVE_H)
        .cutBlind(-GROOVE_DEPTH)
    )
    return mesh_from_cadquery(slab, f"{name}_front_slab")


def _build_drawer(model: ArticulatedObject, name: str, pale, groove_mat, tray_mat):
    """Hollow open-top tray + CadQuery slab front with finger-pull groove.
    Drawer-local frame: slab front outer surface at local x=0, tray extends -X."""
    drawer = model.part(name)

    # Front slab with finger-pull groove (CadQuery mesh).
    drawer.visual(
        _finger_pull_slab(name),
        origin=Origin(xyz=(-FACE_THK / 2.0, 0.0, 0.0)),
        material=pale,
        name="front_slab",
    )

    # Groove accent strip — a thin darker plate seated against the groove
    # back wall inside the recess (seated-trim pattern, small embed into
    # the slab for visual connectivity).
    accent_thk = 0.002
    accent_w = GROOVE_W - GROOVE_H - 0.006
    accent_h = GROOVE_H - 0.004
    accent_z = FACE_H / 2.0 - GROOVE_LIP - GROOVE_H / 2.0
    accent_x = -GROOVE_DEPTH + accent_thk / 2.0 - 0.0005
    drawer.visual(
        Box((accent_thk, accent_w, accent_h)),
        origin=Origin(xyz=(accent_x, 0.0, accent_z)),
        material=groove_mat,
        name="finger_pull_groove",
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

    drawer.inertial = Inertial.from_geometry(Box((BOX_D, BOX_W, BOX_H)), mass=4.0)
    return drawer


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_drawer_bedside_nightstand")

    gray = model.material("carcass_gray", rgba=(0.50, 0.52, 0.55, 1.0))
    gray_dark = model.material("plinth_gray", rgba=(0.27, 0.28, 0.30, 1.0))
    pale = model.material("front_pale_satin", rgba=(0.85, 0.87, 0.88, 1.0))
    groove_mat = model.material("groove_shadow", rgba=(0.35, 0.36, 0.38, 1.0))
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
    # DRAWERS (independent PRISMATIC slides along +X, for-loop pattern)
    # ===================================================================
    drawer_cz = [CZ_TOP, CZ_BOTTOM]
    drawers = []
    for i in range(2):
        name = f"drawer_{i}"
        d = _build_drawer(model, name, pale, groove_mat, tray_mat)
        drawers.append(d)
        model.articulation(
            f"carcass_to_{name}",
            ArticulationType.PRISMATIC,
            parent=carcass,
            child=d,
            origin=Origin(xyz=(D, 0.0, drawer_cz[i])),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=40.0, velocity=0.5,
                                       lower=0.0, upper=TRAVEL),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    drawers = [object_model.get_part(f"drawer_{i}") for i in range(2)]
    joints = [object_model.get_articulation(f"carcass_to_drawer_{i}") for i in range(2)]

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

    # --- Drawer joints: independent prismatic, forward +X, 0..0.36 m ---
    for i, j in enumerate(joints):
        ctx.check(f"drawer_{i}_prismatic",
                  j.articulation_type == ArticulationType.PRISMATIC)
        ctx.check(f"drawer_{i}_axis_x", abs(j.axis[0]) > 0.99 and abs(j.axis[2]) < 0.01)
        ctx.check(f"drawer_{i}_range",
                  abs(j.motion_limits.lower) < 1e-9
                  and abs(j.motion_limits.upper - 0.36) < 1e-6,
                  details=f"range=({j.motion_limits.lower}, {j.motion_limits.upper})")

    # --- Stacked fronts: top drawer above bottom drawer, thin reveal between ---
    front_x = cb[1][0]
    faces = []
    for i, d in enumerate(drawers):
        face = ctx.part_element_world_aabb(d, elem="front_slab")
        assert face is not None
        faces.append(face)
    ctx.check("equal_height_fronts",
              abs((faces[0][1][2] - faces[0][0][2])
                  - (faces[1][1][2] - faces[1][0][2])) < 0.002)
    reveal = faces[0][0][2] - faces[1][1][2]
    ctx.check("thin_reveal_between_fronts", 0.004 < reveal < 0.02,
              details=f"reveal={reveal:.4f}")

    # --- Finger-pull groove: present, recessed, at top of front slab ---
    for i, d in enumerate(drawers):
        face = ctx.part_element_world_aabb(d, elem="front_slab")
        groove = ctx.part_element_world_aabb(d, elem="finger_pull_groove")
        assert face is not None and groove is not None
        # Groove accent is recessed behind the slab front face.
        ctx.check(f"drawer_{i}_groove_recessed",
                  groove[1][0] < face[1][0] - 0.004,
                  details=f"groove max x={groove[1][0]:.4f}, face max x={face[1][0]:.4f}")
        # Groove is near the top of the front slab.
        ctx.check(f"drawer_{i}_groove_at_top",
                  face[1][2] - groove[1][2] < 0.020,
                  details=f"face top z={face[1][2]:.4f}, groove top z={groove[1][2]:.4f}")
        # No protruding handle hardware exists on this drawer.
        vis_names = [v.name for v in d.visuals]
        ctx.check(f"drawer_{i}_no_handle_bar",
                  "handle_bar" not in vis_names,
                  details=f"visuals={vis_names}")
        ctx.check(f"drawer_{i}_no_handle_posts",
                  "handle_post_0" not in vis_names and "handle_post_1" not in vis_names,
                  details=f"visuals={vis_names}")

    # --- Closed pose: fronts flush with cabinet face, trays nested inside ---
    for i, d in enumerate(drawers):
        face = ctx.part_element_world_aabb(d, elem="front_slab")
        tray = ctx.part_element_world_aabb(d, elem="tray_bottom")
        assert face is not None and tray is not None
        ctx.check(f"drawer_{i}_front_flush_closed",
                  abs(face[1][0] - front_x) < 0.002,
                  details=f"face front x={face[1][0]:.4f}, carcass front={front_x:.4f}")
        ctx.check(f"drawer_{i}_tray_nested_closed",
                  tray[1][0] < front_x and tray[0][0] > cb[0][0] + 0.01,
                  details=f"tray x=({tray[0][0]:.3f},{tray[1][0]:.3f})")
        # Hollow open-top tray: side walls rise above the tray bottom.
        wall0 = ctx.part_element_world_aabb(d, elem="tray_side_wall_0")
        assert wall0 is not None
        ctx.check(f"drawer_{i}_tray_open_top",
                  wall0[1][2] > tray[1][2] + 0.05,
                  details=f"wall top={wall0[1][2]:.3f}, bottom top={tray[1][2]:.3f}")
        ctx.expect_within(d, carcass, axes="y", margin=0.001,
                          name=f"drawer_{i}_within_width")

    # --- Open pose: each drawer slides forward 0.36 m and retains insertion ---
    for i, (d, j) in enumerate(zip(drawers, joints)):
        rest = ctx.part_world_position(d)
        with ctx.pose({j: j.motion_limits.upper}):
            out = ctx.part_world_position(d)
            tray = ctx.part_element_world_aabb(d, elem="tray_back_wall")
            assert tray is not None
            rear_x = tray[0][0]
        assert rest is not None and out is not None
        ctx.check(f"drawer_{i}_slides_forward",
                  abs((out[0] - rest[0]) - 0.36) < 1e-6,
                  details=f"dx={out[0] - rest[0]:.4f}")
        ctx.check(f"drawer_{i}_retains_insertion", rear_x < front_x - 0.02,
                  details=f"open rear x={rear_x:.3f}, front={front_x:.3f}")

    # --- Independence: opening the top drawer leaves the bottom drawer shut ---
    with ctx.pose({joints[0]: 0.30}):
        bf = ctx.part_element_world_aabb(drawers[1], elem="front_slab")
        assert bf is not None
        ctx.check("drawers_independent", abs(bf[1][0] - front_x) < 0.002,
                  details=f"drawer_1 face x={bf[1][0]:.4f} with drawer_0 open")

    return ctx.report()


object_model = build_object_model()
