from __future__ import annotations

# Corner cabinet variant: rectangular carcass with angled front doors forming
# a V-shaped (concave) front, a top lid hinged upward at the rear edge, interior
# shelf boards visible through the open gap, and recessed panel borders on the
# doors (shaker-style frame + inset center panel). Matte black wood carcass and
# doors; silver-gray top/lid. Four square legs ~0.15 m tall.
#
# World frame: +X = front (doors face +X), +Y = right, +Z = up, grounded at z=0.

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
W_TOTAL = 0.70           # overall width including top overhang (Y)
D_TOTAL = 0.50           # overall depth including top overhang (X)
H_TOTAL = 0.85           # overall height (Z)
OVERHANG = 0.020         # top slab overhang on all sides
TOP_THK = 0.022          # lid/top slab thickness

BW = W_TOTAL - 2 * OVERHANG      # body width  0.66
BD = D_TOTAL - 2 * OVERHANG      # body depth  0.46 (back x=0, front x=BD)
LEG_H = 0.150
BODY_BOT = LEG_H                  # 0.150
BODY_TOP = H_TOTAL - TOP_THK     # 0.828
BH = BODY_TOP - BODY_BOT         # 0.678

WALL = 0.018                      # carcass panel thickness
INNER_W = BW - 2 * WALL          # 0.624
INNER_D = BD - WALL              # 0.442 (from back panel inner face to front)
# Shelves must not extend into the angled door zone. The door inner edge is at
# roughly FRONT_X - DOOR_W*sin(angle) ≈ 0.33. Keep shelves behind that.
SHELF_MAX_X = 0.30
SHELF_DEPTH = SHELF_MAX_X - WALL  # shelf depth from back panel inner face

# Door zone: opening between bottom board top and top frame board bottom.
ZONE_BOT = BODY_BOT + WALL       # 0.168
ZONE_TOP = BODY_TOP - WALL       # 0.810
DOOR_H = ZONE_TOP - ZONE_BOT    # 0.642

# Door geometry: angled at DOOR_ANGLE from the front plane (V-shape).
DOOR_ANGLE = math.radians(22)    # 22 degrees from front plane
HINGE_Y = BW / 2.0 - WALL       # 0.312 -- hinge at inner face of side panel
# Door width along the door face: chosen so inner edges nearly meet at center.
# Door width along the door face: leave a small gap at center between doors.
CENTER_GAP = 0.010  # 10mm gap from center to each door's inner edge
DOOR_W = (HINGE_Y - CENTER_GAP) / math.cos(DOOR_ANGLE)  # ~0.328
DOOR_THK = 0.018

# Front plane and joint positions.
FRONT_X = BD                      # 0.46

# Recessed panel borders on doors.
FRAME_W = 0.040                   # frame border width
PANEL_RECESS = 0.005              # center panel set back from frame face

# Knob: small polished silver ball on short stem.
KNOB_R = 0.012
STEM_R = 0.005
STEM_L = 0.012

# Shelf boards.
SHELF_THK = 0.016
N_SHELVES = 2

# Lid dimensions (covers the top of the body with overhangs).
LID_DEPTH = BD + OVERHANG        # lid extends from rear to front + overhang
LID_WIDTH = BW + 2 * OVERHANG   # lid width = body width + overhangs = 0.70


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="black_corner_cabinet")

    black = model.material("matte_black_wood", rgba=(0.075, 0.075, 0.08, 1.0))
    black_deep = model.material("black_wood_deep", rgba=(0.055, 0.055, 0.06, 1.0))
    silver_top = model.material("silver_gray_top", rgba=(0.72, 0.73, 0.75, 1.0))
    silver = model.material("polished_silver", rgba=(0.90, 0.91, 0.93, 1.0))
    shelf_mat = model.material("shelf_black", rgba=(0.10, 0.10, 0.11, 1.0))

    # ===================================================================
    # ROOT: carcass (hollow shell + legs + shelves)
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

    # Top frame board (just below the lid).
    carcass.visual(
        Box((BD, INNER_W, WALL)),
        origin=Origin(xyz=(BD / 2.0, 0.0, BODY_TOP - WALL / 2.0)),
        material=black,
        name="top_frame_board",
    )

    # Interior shelf boards (fixed, part of carcass).
    # Shelves are shorter in depth than the carcass to clear the angled doors.
    shelf_zone = ZONE_TOP - ZONE_BOT
    for i in range(N_SHELVES):
        sz = ZONE_BOT + shelf_zone * (i + 1) / (N_SHELVES + 1)
        carcass.visual(
            Box((SHELF_DEPTH, INNER_W - 0.01, SHELF_THK)),
            origin=Origin(xyz=(WALL + SHELF_DEPTH / 2.0, 0.0, sz)),
            material=shelf_mat,
            name=f"shelf_{i}",
        )

    # Four square legs.
    leg_sq = 0.045
    for tag, lx, ly in (("front_right", BD - leg_sq / 2.0, BW / 2.0 - leg_sq / 2.0),
                        ("front_left", BD - leg_sq / 2.0, -(BW / 2.0 - leg_sq / 2.0)),
                        ("rear_right", leg_sq / 2.0 + WALL, BW / 2.0 - leg_sq / 2.0),
                        ("rear_left", leg_sq / 2.0 + WALL, -(BW / 2.0 - leg_sq / 2.0))):
        carcass.visual(
            Box((leg_sq, leg_sq, LEG_H + 0.004)),
            origin=Origin(xyz=(lx, ly, (LEG_H + 0.004) / 2.0)),
            material=black,
            name=f"leg_{tag}",
        )



    carcass.inertial = Inertial.from_geometry(
        Box((BD, BW, BH)), mass=35.0)

    # ===================================================================
    # DOORS: two angled front doors with recessed panels, revolute joints
    # ===================================================================
    angle = DOOR_ANGLE

    for door_tag, side in (("0", 1), ("1", -1)):
        door_name = f"door_{door_tag}"
        door = model.part(door_name)

        # Rotation for door visuals: rotates the box so width goes from hinge
        # toward center.
        if side > 0:
            rz = math.pi - angle   # right door
        else:
            rz = angle              # left door

        # Door center in local (joint) frame.
        cx = -(DOOR_W / 2.0) * math.sin(angle)
        cy = -side * (DOOR_W / 2.0) * math.cos(angle)

        # --- Center panel (recessed inset, thinner than frame) ---
        door.visual(
            Box((DOOR_THK - PANEL_RECESS, DOOR_W - 2 * FRAME_W, DOOR_H - 2 * FRAME_W)),
            origin=Origin(xyz=(cx, cy, 0.0), rpy=(0.0, 0.0, rz)),
            material=black_deep,
            name="center_panel",
        )

        # --- Frame strips (recessed panel border) ---
        # Top frame strip.
        door.visual(
            Box((DOOR_THK, DOOR_W, FRAME_W)),
            origin=Origin(xyz=(cx, cy, DOOR_H / 2.0 - FRAME_W / 2.0),
                          rpy=(0.0, 0.0, rz)),
            material=black,
            name="frame_top",
        )

        # Bottom frame strip.
        door.visual(
            Box((DOOR_THK, DOOR_W, FRAME_W)),
            origin=Origin(xyz=(cx, cy, -(DOOR_H / 2.0 - FRAME_W / 2.0)),
                          rpy=(0.0, 0.0, rz)),
            material=black,
            name="frame_bottom",
        )

        # Hinge-side frame strip (vertical strip at hinge edge).
        strip_hinge_cx = -(FRAME_W / 2.0) * math.sin(angle)
        strip_hinge_cy = -side * (FRAME_W / 2.0) * math.cos(angle)
        door.visual(
            Box((DOOR_THK, FRAME_W, DOOR_H - 2 * FRAME_W)),
            origin=Origin(xyz=(strip_hinge_cx, strip_hinge_cy, 0.0),
                          rpy=(0.0, 0.0, rz)),
            material=black,
            name="frame_hinge",
        )

        # Free-edge frame strip (inner edge of door, near center of cabinet).
        strip_free_cx = -(DOOR_W - FRAME_W / 2.0) * math.sin(angle)
        strip_free_cy = -side * (DOOR_W - FRAME_W / 2.0) * math.cos(angle)
        door.visual(
            Box((DOOR_THK, FRAME_W, DOOR_H - 2 * FRAME_W)),
            origin=Origin(xyz=(strip_free_cx, strip_free_cy, 0.0),
                          rpy=(0.0, 0.0, rz)),
            material=black,
            name="frame_free_edge",
        )

        # --- Knob on the free edge frame strip ---
        # Position the knob at the center of the frame_free_edge strip.
        knob_cx = strip_free_cx
        knob_cy = strip_free_cy
        # Knob protrudes outward from the door face.
        # Outward direction: perpendicular to door face, toward +X.
        out_x = math.cos(angle)
        out_y = -side * math.sin(angle)
        # Embed stem 4mm into the door frame strip for connectivity.
        embed = 0.004
        stem_cx = knob_cx + out_x * (DOOR_THK / 2.0 - embed + STEM_L / 2.0)
        stem_cy = knob_cy + out_y * (DOOR_THK / 2.0 - embed + STEM_L / 2.0)

        door.visual(
            Cylinder(radius=STEM_R, length=STEM_L + embed),
            origin=Origin(xyz=(stem_cx, stem_cy, 0.0),
                          rpy=(0.0, math.pi / 2.0, 0.0)),
            material=silver,
            name="knob_stem",
        )
        ball_cx = knob_cx + out_x * (DOOR_THK / 2.0 + STEM_L + KNOB_R - embed)
        ball_cy = knob_cy + out_y * (DOOR_THK / 2.0 + STEM_L + KNOB_R - embed)
        door.visual(
            Sphere(radius=KNOB_R),
            origin=Origin(xyz=(ball_cx, ball_cy, 0.0)),
            material=silver,
            name="knob_ball",
        )

        door.inertial = Inertial.from_geometry(
            Box((DOOR_THK, DOOR_W, DOOR_H)), mass=3.0)

        # --- Revolute articulation ---
        hinge_y = side * HINGE_Y
        hinge_z = ZONE_BOT + DOOR_H / 2.0
        if side > 0:
            axis = (0.0, 0.0, 1.0)   # right door: +Z
        else:
            axis = (0.0, 0.0, -1.0)  # left door: -Z

        model.articulation(
            f"carcass_to_{door_name}",
            ArticulationType.REVOLUTE,
            parent=carcass,
            child=door,
            origin=Origin(xyz=(FRONT_X, hinge_y, hinge_z)),
            axis=axis,
            motion_limits=MotionLimits(
                effort=20.0, velocity=1.5,
                lower=0.0, upper=1.5,
            ),
        )

    # ===================================================================
    # LID: top panel hinged at rear edge, swings upward
    # ===================================================================
    lid = model.part("lid")

    # Lid panel: the silver-gray top slab.
    # In local frame (at hinge, q=0), the lid extends forward (+X) from hinge.
    lid.visual(
        Box((LID_DEPTH, LID_WIDTH, TOP_THK)),
        origin=Origin(xyz=(LID_DEPTH / 2.0, 0.0, TOP_THK / 2.0)),
        material=silver_top,
        name="lid_panel",
    )

    # Small lid handle on the front edge.
    lid.visual(
        Box((0.04, 0.08, 0.012)),
        origin=Origin(xyz=(LID_DEPTH - 0.03, 0.0, TOP_THK + 0.006)),
        material=silver,
        name="lid_handle",
    )

    lid.inertial = Inertial.from_geometry(
        Box((LID_DEPTH, LID_WIDTH, TOP_THK)), mass=2.5)

    # Lid articulation: revolute at rear top edge.
    lid_hinge_x = WALL   # just past the back panel
    lid_hinge_z = BODY_TOP
    model.articulation(
        "carcass_to_lid",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=lid,
        origin=Origin(xyz=(lid_hinge_x, 0.0, lid_hinge_z)),
        # axis = (0, -1, 0): positive rotation goes from +X toward +Z.
        # The lid extends along +X from hinge, so positive q lifts the front edge up.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0, velocity=1.0,
            lower=0.0, upper=1.3,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    door_0 = object_model.get_part("door_0")
    door_1 = object_model.get_part("door_1")
    lid = object_model.get_part("lid")

    hinge_0 = object_model.get_articulation("carcass_to_door_0")
    hinge_1 = object_model.get_articulation("carcass_to_door_1")
    lid_joint = object_model.get_articulation("carcass_to_lid")

    # --- Grounding and overall scale ---
    cb = ctx.part_world_aabb(carcass)
    assert cb is not None
    ctx.check("legs_on_floor", abs(cb[0][2]) < 0.003,
              details=f"min z={cb[0][2]:.4f}")

    # Total assembly height includes the lid at closed position.
    lid_aabb_closed = ctx.part_element_world_aabb(lid, elem="lid_panel")
    assert lid_aabb_closed is not None
    total_h = lid_aabb_closed[1][2] - cb[0][2]
    ctx.check("height_085", abs(total_h - H_TOTAL) < 0.01,
              details=f"total h={total_h:.3f}")

    # Total assembly width from lid (widest part with overhangs).
    ctx.check("width_070", abs(LID_WIDTH - W_TOTAL) < 0.005,
              details=f"lid width={LID_WIDTH:.3f}")

    # --- Angled front doors (corner cabinet) ---
    for name, j in [("door_0", hinge_0), ("door_1", hinge_1)]:
        ctx.check(f"{name}_revolute",
                  j.articulation_type == ArticulationType.REVOLUTE)
        ctx.check(f"{name}_vertical_axis",
                  abs(j.axis[2]) > 0.99 and abs(j.axis[0]) < 0.01 and abs(j.axis[1]) < 0.01,
                  details=f"axis={j.axis}")
        ctx.check(f"{name}_range",
                  abs(j.motion_limits.lower) < 1e-9 and j.motion_limits.upper > 1.0,
                  details=f"range=({j.motion_limits.lower},{j.motion_limits.upper})")

    # --- Doors are angled (not flush with front plane) ---
    door_0_free = ctx.part_element_world_aabb(door_0, elem="frame_free_edge")
    door_1_free = ctx.part_element_world_aabb(door_1, elem="frame_free_edge")
    assert door_0_free is not None and door_1_free is not None
    ctx.check("door_0_inner_edge_recessed",
              door_0_free[1][0] < FRONT_X - 0.05,
              details=f"door_0 free edge max x={door_0_free[1][0]:.4f}")
    ctx.check("door_1_inner_edge_recessed",
              door_1_free[1][0] < FRONT_X - 0.05,
              details=f"door_1 free edge max x={door_1_free[1][0]:.4f}")

    # --- Recessed panel borders on doors ---
    # Verify the frame structure: top/bottom frame strips extend above/below
    # the center panel, and the hinge-side strip exists alongside the center panel.
    for d_name, d in [("door_0", door_0), ("door_1", door_1)]:
        frame_top = ctx.part_element_world_aabb(d, elem="frame_top")
        frame_bot = ctx.part_element_world_aabb(d, elem="frame_bottom")
        center = ctx.part_element_world_aabb(d, elem="center_panel")
        frame_hinge = ctx.part_element_world_aabb(d, elem="frame_hinge")
        frame_free = ctx.part_element_world_aabb(d, elem="frame_free_edge")
        assert (frame_top is not None and frame_bot is not None
                and center is not None and frame_hinge is not None
                and frame_free is not None)
        # Frame strips are at the top and bottom edges of the door.
        ctx.check(f"{d_name}_frame_top_above_center",
                  frame_top[1][2] > center[1][2] + 0.01,
                  details=f"frame_top max z={frame_top[1][2]:.4f}, center max z={center[1][2]:.4f}")
        ctx.check(f"{d_name}_frame_bottom_below_center",
                  frame_bot[0][2] < center[0][2] - 0.01,
                  details=f"frame_bot min z={frame_bot[0][2]:.4f}, center min z={center[0][2]:.4f}")
        # Frame strips and center panel are vertically overlapping (frame surrounds center).
        ctx.check(f"{d_name}_frame_surrounds_center_z",
                  frame_top[0][2] < center[1][2] and frame_bot[1][2] > center[0][2],
                  details="frame must overlap center vertically")
        # The hinge-side strip and free-edge strip span between top/bottom frames.
        ctx.check(f"{d_name}_vertical_strips_span",
                  frame_hinge[1][2] > frame_hinge[0][2] + 0.30
                  and frame_free[1][2] > frame_free[0][2] + 0.30,
                  details=f"hinge span={frame_hinge[1][2]-frame_hinge[0][2]:.3f}")

    # --- Top lid hinges upward on a rear revolute joint ---
    ctx.check("lid_revolute",
              lid_joint.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("lid_axis_horizontal",
              abs(lid_joint.axis[1]) > 0.99
              and abs(lid_joint.axis[0]) < 0.01
              and abs(lid_joint.axis[2]) < 0.01,
              details=f"axis={lid_joint.axis}")
    lid_origin_x = lid_joint.origin.xyz[0] if lid_joint.origin else 0.0
    ctx.check("lid_hinge_at_rear",
              lid_origin_x < BD / 2.0,
              details=f"hinge x={lid_origin_x:.4f}")

    # Lid closed pose: sits flat on top of carcass.
    ctx.check("lid_closed_on_top",
              abs(lid_aabb_closed[0][2] - BODY_TOP) < 0.005,
              details=f"lid bottom z={lid_aabb_closed[0][2]:.4f}, body top={BODY_TOP:.4f}")

    # Lid open pose: front edge rises.
    lid_handle_closed = ctx.part_element_world_aabb(lid, elem="lid_handle")
    assert lid_handle_closed is not None
    closed_handle_z = lid_handle_closed[1][2]
    with ctx.pose({lid_joint: lid_joint.motion_limits.upper}):
        lid_handle_open = ctx.part_element_world_aabb(lid, elem="lid_handle")
        assert lid_handle_open is not None
        open_handle_z = lid_handle_open[1][2]
    ctx.check("lid_opens_upward",
              open_handle_z > closed_handle_z + 0.10,
              details=f"closed z={closed_handle_z:.4f}, open z={open_handle_z:.4f}")

    # --- Shelf boards visible through the open gap ---
    for i in range(N_SHELVES):
        shelf = ctx.part_element_world_aabb(carcass, elem=f"shelf_{i}")
        assert shelf is not None
        ctx.check(f"shelf_{i}_inside_body",
                  shelf[1][2] > ZONE_BOT + 0.01 and shelf[0][2] < ZONE_TOP - 0.01,
                  details=f"shelf z=({shelf[0][2]:.4f},{shelf[1][2]:.4f})")
        # Shelves span most of the interior width.
        shelf_w = shelf[1][1] - shelf[0][1]
        ctx.check(f"shelf_{i}_spans_interior",
                  shelf_w > INNER_W * 0.8,
                  details=f"shelf w={shelf_w:.3f}")

    # --- Doors open outward (positive q swings free edge toward +X) ---
    for d_name, d, j in [("door_0", door_0, hinge_0), ("door_1", door_1, hinge_1)]:
        closed_free = ctx.part_element_world_aabb(d, elem="frame_free_edge")
        assert closed_free is not None
        closed_free_x = (closed_free[0][0] + closed_free[1][0]) / 2.0
        with ctx.pose({j: j.motion_limits.upper}):
            open_free = ctx.part_element_world_aabb(d, elem="frame_free_edge")
            assert open_free is not None
            open_free_x = (open_free[0][0] + open_free[1][0]) / 2.0
        ctx.check(f"{d_name}_opens_outward",
                  open_free_x > closed_free_x + 0.05,
                  details=f"closed x={closed_free_x:.4f}, open x={open_free_x:.4f}")

    # --- At least one non-fixed joint ---
    all_joints = [hinge_0, hinge_1, lid_joint]
    non_fixed = [j for j in all_joints
                 if j.articulation_type in (ArticulationType.REVOLUTE,
                                            ArticulationType.PRISMATIC,
                                            ArticulationType.CONTINUOUS)]
    ctx.check("non_fixed_joints_exist",
              len(non_fixed) >= 3,
              details=f"non_fixed count={len(non_fixed)}")

    return ctx.report()


object_model = build_object_model()
