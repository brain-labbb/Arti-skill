from __future__ import annotations

# Tall two-door cabinet (~0.90 m W x 1.80 m H x 0.50 m D) with raised plinth
# base, visible hinge barrels, and bar-style pull handles.
#
# World layout: front faces +X (back at x=0, front face at x=BD),
# width along Y (centered), height along +Z, grounded at z=0 on a raised
# plinth base ~0.10 m tall. Matte black wood carcass and door fronts; a thin
# smooth silver-gray top slab overhangs the body on all sides.
#
# Two overlay doors swing on revolute hinge joints at the left and right
# front edges. Visible hinge barrel cylinders sit at each hinge line.
# Each door carries a bar-style pull handle near the free edge.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)

# ---------- key dimensions (meters) ----------
W_TOTAL = 0.90           # overall width including top overhang (Y)
D_TOTAL = 0.50           # overall depth including top overhang (X)
H_TOTAL = 1.80           # overall height (Z)
OVERHANG = 0.020         # top slab overhang on all sides
TOP_THK = 0.022          # silver top slab thickness

BW = W_TOTAL - 2 * OVERHANG      # body width 0.86
BD = D_TOTAL - 2 * OVERHANG      # body depth 0.46
PLINTH_H = 0.10                  # raised plinth base height
PLINTH_INSET = 0.025             # plinth inset from body edges

BODY_BOT = PLINTH_H              # body bottom z (0.10)
BODY_TOP = H_TOTAL - TOP_THK    # body top z (1.778)
BH = BODY_TOP - BODY_BOT        # body height (1.678)

WALL = 0.018                     # carcass panel thickness
INNER_W = BW - 2 * WALL         # inner opening width

# Door dimensions (overlay style)
FACE_PROUD = 0.003               # door stands proud of carcass front
DOOR_GAP = 0.004                 # vertical gap top/bottom
DOOR_CENTER_GAP = 0.005         # gap between the two doors at center
DOOR_W = (INNER_W - DOOR_CENTER_GAP) / 2.0
DOOR_H = BH - 2 * DOOR_GAP
DOOR_THK = 0.020                 # door panel thickness

# Hinge barrel geometry
HINGE_BARREL_R = 0.008
HINGE_BARREL_H = 0.055
# Hinge Z positions (world coordinates)
HINGE_ZS = [
    BODY_BOT + WALL + DOOR_GAP + 0.12,
    BODY_BOT + BH / 2.0,
    BODY_TOP - WALL - DOOR_GAP - 0.12,
]
# Hinge Z offsets in door local frame (relative to door center z=0)
HINGE_LOCAL_ZS = [hz - (BODY_BOT + BH / 2.0) for hz in HINGE_ZS]

# Handle dimensions (vertical bar-style pull handle)
HANDLE_BAR_R = 0.005
HANDLE_BAR_LEN = 0.140
HANDLE_STEM_R = 0.004
HANDLE_STEM_LEN = 0.022
HANDLE_STEM_SPACING = 0.100


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tall_two_door_cabinet")

    black = model.material("matte_black_wood", rgba=(0.075, 0.075, 0.08, 1.0))
    black_deep = model.material("black_wood_deep", rgba=(0.055, 0.055, 0.06, 1.0))
    silver_top = model.material("silver_gray_top", rgba=(0.72, 0.73, 0.75, 1.0))
    silver = model.material("polished_silver", rgba=(0.90, 0.91, 0.93, 1.0))
    brass_hinge = model.material("hinge_brass", rgba=(0.72, 0.58, 0.28, 1.0))

    # ===================================================================
    # ROOT: carcass (hollow shell + plinth + silver top)
    # ===================================================================
    carcass = model.part("carcass")

    # Side panels.
    for tag, s in (("0", 1), ("1", -1)):
        carcass.visual(
            Box((BD, WALL, BH)),
            origin=Origin(xyz=(BD / 2.0, s * (BW / 2.0 - WALL / 2.0),
                               BODY_BOT + BH / 2.0)),
            material=black,
            name=f"side_panel_{tag}",
        )

    # Back panel.
    carcass.visual(
        Box((WALL, INNER_W, BH)),
        origin=Origin(xyz=(WALL / 2.0, 0.0, BODY_BOT + BH / 2.0)),
        material=black,
        name="back_panel",
    )

    # Bottom board.
    carcass.visual(
        Box((BD, INNER_W, WALL)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_BOT + WALL / 2.0)),
        material=black,
        name="bottom_board",
    )

    # Top stretcher board.
    carcass.visual(
        Box((BD, INNER_W, WALL)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_TOP - WALL / 2.0)),
        material=black,
        name="top_stretcher",
    )

    # Front top rail.
    carcass.visual(
        Box((WALL, INNER_W, DOOR_GAP)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0,
                           BODY_TOP - WALL - DOOR_GAP / 2.0)),
        material=black,
        name="front_top_rail",
    )

    # Front bottom rail.
    carcass.visual(
        Box((WALL, INNER_W, DOOR_GAP)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0,
                           BODY_BOT + WALL + DOOR_GAP / 2.0)),
        material=black,
        name="front_bottom_rail",
    )

    # Center stile behind the door center gap.
    carcass.visual(
        Box((WALL, DOOR_CENTER_GAP, BH - 2 * WALL)),
        origin=Origin(xyz=(BD - WALL / 2.0, 0.0, BODY_BOT + BH / 2.0)),
        material=black_deep,
        name="center_stile",
    )

    # Interior shelf.
    shelf_z = BODY_BOT + BH * 0.45
    carcass.visual(
        Box((BD - 2 * WALL, INNER_W - 0.004, 0.016)),
        origin=Origin(xyz=(BD / 2.0, 0.0, shelf_z)),
        material=black_deep,
        name="interior_shelf",
    )

    # Raised plinth base.
    plinth_w = BW - 2 * PLINTH_INSET
    plinth_d = BD - PLINTH_INSET
    carcass.visual(
        Box((plinth_d, plinth_w, PLINTH_H)),
        origin=Origin(xyz=(PLINTH_INSET / 2.0 + plinth_d / 2.0, 0.0,
                           PLINTH_H / 2.0)),
        material=black_deep,
        name="plinth_base",
    )

    # Silver-gray top slab with overhang.
    carcass.visual(
        Box((D_TOTAL, W_TOTAL, TOP_THK)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_TOP + TOP_THK / 2.0)),
        material=silver_top,
        name="top_slab",
    )

    carcass.inertial = Inertial.from_geometry(
        Box((BD, BW, H_TOTAL)), mass=55.0)

    # ===================================================================
    # DOORS: two overlay doors on revolute hinge joints
    # ===================================================================

    def _build_door(model: ArticulatedObject, name: str, side: float):
        """Build a door in its local frame. Origin is at the hinge pin center.

        side=-1: left door, panel extends along +Y from origin
        side=+1: right door, panel extends along -Y from origin

        The door panel is overlay: sits in front of the carcass (local +X).
        """
        door = model.part(name)

        # Door panel: overlay position, in front of the hinge pin.
        panel_cx = FACE_PROUD + DOOR_THK / 2.0
        panel_cy = side * (-DOOR_W / 2.0)
        door.visual(
            Box((DOOR_THK, DOOR_W, DOOR_H)),
            origin=Origin(xyz=(panel_cx, panel_cy, 0.0)),
            material=black,
            name="door_panel",
        )

        # Door hinge edge strip (darker accent at the hinge side).
        door.visual(
            Box((DOOR_THK + 0.002, 0.006, DOOR_H - 0.020)),
            origin=Origin(xyz=(panel_cx, 0.0, 0.0)),
            material=black_deep,
            name="door_hinge_edge",
        )

        # --- Hinge barrel cylinders (visible brass barrels at hinge line) ---
        # Positioned at local origin in X/Y (the hinge pin axis),
        # at three heights along the door.
        for i, lcz in enumerate(HINGE_LOCAL_ZS):
            door.visual(
                Cylinder(radius=HINGE_BARREL_R, length=HINGE_BARREL_H),
                origin=Origin(xyz=(0.0, 0.0, lcz)),
                material=brass_hinge,
                name=f"hinge_barrel_{i}",
            )

        # --- Bar-style pull handle near the free edge ---
        # Handle is vertical (along Z), mounted on two horizontal stems
        # protruding from the door front face along +X.
        handle_free_y = side * (-DOOR_W + 0.055)
        handle_front_x = FACE_PROUD + DOOR_THK  # front face of door

        # Two stems protruding from front face.
        for stem_tag, dz in (("0", -HANDLE_STEM_SPACING / 2.0),
                              ("1", HANDLE_STEM_SPACING / 2.0)):
            door.visual(
                Cylinder(radius=HANDLE_STEM_R, length=HANDLE_STEM_LEN),
                origin=Origin(xyz=(handle_front_x + HANDLE_STEM_LEN / 2.0,
                                   handle_free_y, dz),
                              rpy=(0.0, math.pi / 2.0, 0.0)),
                material=silver,
                name=f"handle_stem_{stem_tag}",
            )

        # Vertical bar connecting the two stems.
        door.visual(
            Cylinder(radius=HANDLE_BAR_R,
                     length=HANDLE_STEM_SPACING + 2 * HANDLE_BAR_R),
            origin=Origin(xyz=(handle_front_x + HANDLE_STEM_LEN,
                               handle_free_y, 0.0)),
            material=silver,
            name="handle_bar",
        )

        door.inertial = Inertial.from_geometry(
            Box((DOOR_THK, DOOR_W, DOOR_H)), mass=5.0)
        return door

    left_door = _build_door(model, "left_door", side=-1)
    right_door = _build_door(model, "right_door", side=1)

    # ===================================================================
    # ARTICULATIONS
    # ===================================================================

    # Left door: hinge at left front edge. Door extends +Y from hinge.
    # axis=(0,0,-1): positive q rotates +Y toward +X (door opens outward).
    model.articulation(
        "carcass_to_left_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=left_door,
        origin=Origin(xyz=(BD, -BW / 2.0, BODY_BOT + BH / 2.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.5,
                                    lower=0.0, upper=1.5),
    )

    # Right door: hinge at right front edge. Door extends -Y from hinge.
    # axis=(0,0,1): positive q rotates -Y toward +X (door opens outward).
    model.articulation(
        "carcass_to_right_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=right_door,
        origin=Origin(xyz=(BD, BW / 2.0, BODY_BOT + BH / 2.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.5,
                                    lower=0.0, upper=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    left_hinge = object_model.get_articulation("carcass_to_left_door")
    right_hinge = object_model.get_articulation("carcass_to_right_door")

    # --- Overall scale and grounding ---
    cb = ctx.part_world_aabb(carcass)
    assert cb is not None
    ctx.check("plinth_on_floor", abs(cb[0][2]) < 0.003,
              details=f"min z={cb[0][2]:.4f}")
    width_y = cb[1][1] - cb[0][1]
    depth_x = cb[1][0] - cb[0][0]
    height_z = cb[1][2] - cb[0][2]
    ctx.check("width_090", abs(width_y - 0.90) < 0.03,
              details=f"w={width_y:.3f}")
    ctx.check("depth_050", abs(depth_x - 0.50) < 0.05,
              details=f"d={depth_x:.3f}")
    ctx.check("height_180", abs(height_z - 1.80) < 0.02,
              details=f"h={height_z:.3f}")

    # --- Plinth base raises the cabinet body ---
    plinth = ctx.part_element_world_aabb(carcass, elem="plinth_base")
    assert plinth is not None
    ctx.check("plinth_raises_body",
              abs(plinth[1][2] - PLINTH_H) < 0.005,
              details=f"plinth top z={plinth[1][2]:.3f}")
    ctx.check("plinth_inset",
              plinth[0][1] > -(BW / 2.0) + 0.010
              and plinth[1][1] < (BW / 2.0) - 0.010,
              details=f"plinth y=({plinth[0][1]:.3f},{plinth[1][1]:.3f})")

    # --- Silver top slab overhangs body ---
    top = ctx.part_element_world_aabb(carcass, elem="top_slab")
    side = ctx.part_element_world_aabb(carcass, elem="side_panel_0")
    assert top is not None and side is not None
    ctx.check("top_overhang",
              top[1][1] > side[1][1] + 0.015
              and top[0][1] < -side[1][1] - 0.015,
              details=f"top y=({top[0][1]:.3f},{top[1][1]:.3f})")
    ctx.check("top_thin_slab",
              abs((top[1][2] - top[0][2]) - TOP_THK) < 0.002)

    # --- Two doors with proper size ---
    left_panel = ctx.part_element_world_aabb(left_door, elem="door_panel")
    right_panel = ctx.part_element_world_aabb(right_door, elem="door_panel")
    assert left_panel is not None and right_panel is not None
    door_w = left_panel[1][1] - left_panel[0][1]
    door_h = left_panel[1][2] - left_panel[0][2]
    ctx.check("door_width", abs(door_w - DOOR_W) < 0.005,
              details=f"w={door_w:.3f}")
    ctx.check("door_height_tall", door_h > 1.50,
              details=f"h={door_h:.3f}")

    # --- Both hinges are revolute with vertical axis and correct limits ---
    for jname, j in [("left_hinge", left_hinge), ("right_hinge", right_hinge)]:
        ctx.check(f"{jname}_revolute",
                  j.articulation_type == ArticulationType.REVOLUTE)
        ctx.check(f"{jname}_vertical_axis",
                  abs(j.axis[2]) > 0.99 and abs(j.axis[0]) < 0.01
                  and abs(j.axis[1]) < 0.01,
                  details=f"axis={j.axis}")
        ctx.check(f"{jname}_range",
                  abs(j.motion_limits.lower) < 1e-9
                  and abs(j.motion_limits.upper - 1.5) < 0.01,
                  details=f"range=({j.motion_limits.lower},{j.motion_limits.upper})")

    # --- Visible hinge barrels ---
    for door_name in ("left_door", "right_door"):
        for i in range(3):
            barrel = ctx.part_element_world_aabb(
                object_model.get_part(door_name), elem=f"hinge_barrel_{i}")
            assert barrel is not None
            ctx.check(f"{door_name}_hinge_barrel_{i}_visible",
                      (barrel[1][2] - barrel[0][2]) > HINGE_BARREL_H - 0.005,
                      details=f"barrel h={barrel[1][2] - barrel[0][2]:.3f}")

    # Hinge barrels overlap with carcass side panels at the hinge edge.
    # This is intentional: the barrel captures the door at the hinge line.
    side_elem_for_door = {"left_door": "side_panel_1", "right_door": "side_panel_0"}
    for door_name in ("left_door", "right_door"):
        se = side_elem_for_door[door_name]
        for i in range(3):
            ctx.allow_overlap(
                "carcass", door_name,
                elem_a=se, elem_b=f"hinge_barrel_{i}",
                reason=f"Hinge barrel is intentionally embedded at the {door_name} hinge edge, "
                       f"partially nested in the carcass side panel for a realistic barrel hinge mount.",
            )
    # Proof: hinge barrels maintain contact with side panels at hinge line.
    for door_name in ("left_door", "right_door"):
        se = side_elem_for_door[door_name]
        door_part = object_model.get_part(door_name)
        ctx.expect_contact(
            door_part, carcass,
            elem_a="hinge_barrel_1", elem_b=se,
            contact_tol=0.012,
            name=f"{door_name}_hinge_barrel_contact",
        )

    # --- Pull handles on each door ---
    for door_name in ("left_door", "right_door"):
        door_part = object_model.get_part(door_name)
        bar = ctx.part_element_world_aabb(door_part, elem="handle_bar")
        stem = ctx.part_element_world_aabb(door_part, elem="handle_stem_0")
        assert bar is not None and stem is not None
        ctx.check(f"{door_name}_handle_bar_exists",
                  (bar[1][2] - bar[0][2]) > HANDLE_STEM_SPACING - 0.01,
                  details=f"bar span z={bar[1][2] - bar[0][2]:.3f}")
        ctx.check(f"{door_name}_handle_proud_of_door",
                  stem[1][0] > BD + FACE_PROUD + DOOR_THK + 0.005,
                  details=f"stem max x={stem[1][0]:.4f}")

    # --- Closed pose: doors at front, overlay position ---
    carcass_front = BD
    for door_name, door_part in [("left_door", left_door),
                                  ("right_door", right_door)]:
        panel = ctx.part_element_world_aabb(door_part, elem="door_panel")
        assert panel is not None
        ctx.check(f"{door_name}_overlay_front",
                  panel[0][0] > carcass_front,
                  details=f"door back x={panel[0][0]:.4f}, carcass front={carcass_front}")

    # --- Open pose: doors swing outward (max x increases) ---
    for door_name, door_part, hinge in [
        ("left_door", left_door, left_hinge),
        ("right_door", right_door, right_hinge),
    ]:
        rest_panel = ctx.part_element_world_aabb(door_part, elem="door_panel")
        assert rest_panel is not None
        rest_max_x = rest_panel[1][0]
        with ctx.pose({hinge: 1.0}):
            open_panel = ctx.part_element_world_aabb(door_part, elem="door_panel")
            assert open_panel is not None
            open_max_x = open_panel[1][0]
            open_center_y = (open_panel[0][1] + open_panel[1][1]) / 2.0
        ctx.check(f"{door_name}_opens_outward",
                  open_max_x > rest_max_x + 0.05,
                  details=f"rest max x={rest_max_x:.3f}, open max x={open_max_x:.3f}")

    # --- Independence: opening one door leaves the other closed ---
    with ctx.pose({left_hinge: 1.2}):
        right_closed = ctx.part_element_world_aabb(right_door, elem="door_panel")
        assert right_closed is not None
        ctx.check("doors_independent",
                  abs(right_closed[0][0] - (BD + FACE_PROUD)) < 0.005,
                  details=f"right door back x={right_closed[0][0]:.4f}")

    return ctx.report()


object_model = build_object_model()
