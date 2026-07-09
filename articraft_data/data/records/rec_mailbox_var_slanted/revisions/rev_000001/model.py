from __future__ import annotations

# Slanted-top boxy cabinet residential mailbox on a scrollwork post.
#
# A rectangular sheet-metal cabinet with a peaked/slanted lid that falls from
# a higher back edge down toward a lower front edge. The interior is genuinely
# hollow: back wall, two trapezoidal side walls, floor, sloped lid, and a
# front face with only a header strip above and a sill strip below the deposit
# opening. The open cavity reads as dark and deep behind the opening.
#
# Two articulations (same as parent):
#   1. body_to_door : REVOLUTE front door, bottom hinge (flips down to open)
#   2. body_to_flag : REVOLUTE side signal flag, horizontal pin (raises/lowers)
#
# A U.S.-flag decal panel rides on the +Y side wall.
#
# Coordinate convention:
#   - up is +Z; the post legs stand on the floor at z = 0.
#   - the cabinet axis runs along X; the open end / DOOR faces +X (front).
#   - the closed back end is at -X; centerline is y = 0.
#   - the signal flag is on the +Y side.
#
# Root structure: the POST is the root resting on the floor, and the box is
# fixed on top of the post. The door and the flag are children of the box.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# ---- key dimensions (meters) -------------------------------------------------
BOX_LEN = 0.46             # cabinet length along X (front to back)
BOX_W = 0.20               # cabinet width along Y
BACK_H = 0.22              # back wall height above floor
FRONT_H = 0.16             # front wall height above floor (shorter → slant)
WALL_T = 0.006             # sheet-metal thickness
HEADER_H = 0.025           # front header strip height
SILL_H = 0.015             # front sill strip height
INNER_W = BOX_W - 2.0 * WALL_T   # inner width between side walls

BOX_FLOOR_Z = 0.55         # mailbox floor height above ground (on the post)
FRONT_X = BOX_LEN / 2.0    # front opening plane
BACK_X = -BOX_LEN / 2.0

# door
DOOR_W = INNER_W - 0.004   # width with side clearance
DOOR_H = FRONT_H - HEADER_H - SILL_H - 0.004  # height with clearance
DOOR_T = 0.008             # door panel thickness

# scrollwork post
POST_LEG = 0.022
POST_SPACING = 0.16
POST_TOP_Z = BOX_FLOOR_Z
SCROLL_TUBE = 0.012

# signal flag
FLAG_W = 0.085
FLAG_H = 0.075
FLAG_ARM = 0.10


def _side_wall_plate(y_outer: float, thickness: float,
                     x_back: float, x_front: float,
                     h_back: float, h_front: float,
                     z_base: float) -> MeshGeometry:
    """Thin trapezoidal plate for one side wall of the slanted cabinet."""
    y_inner = y_outer - thickness if y_outer > 0 else y_outer + thickness
    g = MeshGeometry()
    # outer face (4 corners)
    ob_b = g.add_vertex(x_back,  y_outer, z_base)
    ob_t = g.add_vertex(x_back,  y_outer, z_base + h_back)
    of_t = g.add_vertex(x_front, y_outer, z_base + h_front)
    of_b = g.add_vertex(x_front, y_outer, z_base)
    # inner face (4 corners)
    ib_b = g.add_vertex(x_back,  y_inner, z_base)
    ib_t = g.add_vertex(x_back,  y_inner, z_base + h_back)
    if_t = g.add_vertex(x_front, y_inner, z_base + h_front)
    if_b = g.add_vertex(x_front, y_inner, z_base)

    if y_outer > 0:
        # right wall: outer normal +Y
        g.add_face(ob_b, of_t, of_b)
        g.add_face(ob_b, ob_t, of_t)
        g.add_face(ib_b, if_b, if_t)
        g.add_face(ib_b, if_t, ib_t)
    else:
        # left wall: outer normal -Y
        g.add_face(ob_b, of_b, of_t)
        g.add_face(ob_b, of_t, ob_t)
        g.add_face(ib_b, ib_t, if_t)
        g.add_face(ib_b, if_t, if_b)
    # back edge
    g.add_face(ob_b, ob_t, ib_t)
    g.add_face(ob_b, ib_t, ib_b)
    # front edge
    g.add_face(of_b, if_t, of_t)
    g.add_face(of_b, if_b, if_t)
    # top edge (sloped)
    g.add_face(ob_t, of_t, if_t)
    g.add_face(ob_t, if_t, ib_t)
    # bottom edge
    g.add_face(ob_b, ib_b, if_b)
    g.add_face(ob_b, if_b, of_b)
    return g


def _sloped_lid_plate(x_back: float, x_front: float,
                      y_half: float,
                      h_back: float, h_front: float,
                      z_base: float, thickness: float) -> MeshGeometry:
    """Thin sloped rectangular plate for the cabinet lid."""
    dx = x_front - x_back
    dz = h_front - h_back
    slope_len = math.sqrt(dx * dx + dz * dz)
    # outward normal (pointing up from the slope surface)
    nx = -dz / slope_len
    nz = dx / slope_len
    off_x = thickness * nx
    off_z = thickness * nz

    g = MeshGeometry()
    # bottom face (rests on the wall tops)
    bb_l = g.add_vertex(x_back,  -y_half, z_base + h_back)
    bb_r = g.add_vertex(x_back,   y_half, z_base + h_back)
    bf_l = g.add_vertex(x_front, -y_half, z_base + h_front)
    bf_r = g.add_vertex(x_front,  y_half, z_base + h_front)
    # top face (offset outward by thickness)
    tb_l = g.add_vertex(x_back  + off_x, -y_half, z_base + h_back  + off_z)
    tb_r = g.add_vertex(x_back  + off_x,  y_half, z_base + h_back  + off_z)
    tf_l = g.add_vertex(x_front + off_x, -y_half, z_base + h_front + off_z)
    tf_r = g.add_vertex(x_front + off_x,  y_half, z_base + h_front + off_z)

    # top face (normal up)
    g.add_face(tb_l, tf_l, tf_r)
    g.add_face(tb_l, tf_r, tb_r)
    # bottom face
    g.add_face(bb_l, bb_r, bf_r)
    g.add_face(bb_l, bf_r, bf_l)
    # back edge
    g.add_face(bb_l, tb_l, tb_r)
    g.add_face(bb_l, tb_r, bb_r)
    # front edge
    g.add_face(bf_l, bf_r, tf_r)
    g.add_face(bf_l, tf_r, tf_l)
    # left edge (y = -y_half)
    g.add_face(bb_l, bf_l, tf_l)
    g.add_face(bb_l, tf_l, tb_l)
    # right edge (y = +y_half)
    g.add_face(bb_r, tb_r, tf_r)
    g.add_face(bb_r, tf_r, bf_r)
    return g


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slanted_cabinet_mailbox")

    black = model.material("black", rgba=(0.10, 0.10, 0.11, 1.0))
    black_satin = model.material("black_satin", rgba=(0.16, 0.16, 0.17, 1.0))
    iron = model.material("iron", rgba=(0.13, 0.13, 0.14, 1.0))
    interior = model.material("interior", rgba=(0.05, 0.05, 0.06, 1.0))
    red = model.material("flag_red", rgba=(0.74, 0.10, 0.13, 1.0))
    flag_blue = model.material("flag_blue", rgba=(0.16, 0.20, 0.45, 1.0))
    flag_white = model.material("flag_white", rgba=(0.88, 0.88, 0.90, 1.0))
    flag_red2 = model.material("decal_red", rgba=(0.72, 0.12, 0.16, 1.0))

    # --------------------------------------------------------- POST (root)
    post = model.part("post")
    leg_h = BOX_FLOOR_Z - WALL_T
    for i, xx in enumerate((-POST_SPACING / 2.0, POST_SPACING / 2.0)):
        post.visual(
            Box((POST_LEG, POST_LEG, leg_h)),
            origin=Origin(xyz=(xx, 0.0, leg_h / 2.0)),
            material=iron,
            name=f"post_leg_{i}",
        )
        post.visual(
            Box((POST_LEG + 0.02, POST_LEG + 0.02, 0.012)),
            origin=Origin(xyz=(xx, 0.0, 0.006)),
            material=black_satin,
            name=f"post_foot_{i}",
        )
    crossbar_top = BOX_FLOOR_Z - WALL_T
    post.visual(
        Box((POST_SPACING + POST_LEG, 0.05, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, crossbar_top - 0.009)),
        material=iron,
        name="post_crossbar",
    )
    n_scroll = 3
    z0 = 0.12
    z1 = POST_TOP_Z - 0.10
    for k in range(n_scroll):
        zc = z0 + (z1 - z0) * k / (n_scroll - 1)
        for s, sgn in enumerate((-1.0, 1.0)):
            post.visual(
                Cylinder(radius=0.030, length=SCROLL_TUBE * 2.0),
                origin=Origin(
                    xyz=(sgn * 0.040, 0.0, zc),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=iron,
                name=f"scroll_{k}_{s}",
            )

    # --------------------------------------------------------- BODY (cabinet)
    # Hollow slanted-top cabinet: floor, back wall, two trapezoidal side walls,
    # sloped lid, front header strip, front sill strip. All sheet-metal thin.
    body = model.part("body")

    # Floor plate (full width, interior-dark so the cavity bottom reads hollow)
    body.visual(
        Box((BOX_LEN, BOX_W, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, BOX_FLOOR_Z - WALL_T / 2.0)),
        material=interior,
        name="floor",
    )

    # Back wall (fits between side walls to avoid corner overlap)
    body.visual(
        Box((WALL_T, INNER_W, BACK_H)),
        origin=Origin(xyz=(BACK_X + WALL_T / 2.0, 0.0,
                           BOX_FLOOR_Z + BACK_H / 2.0)),
        material=interior,
        name="back_wall",
    )

    # Side walls (trapezoidal — taller at back, shorter at front)
    for i, (y_out, wall_name) in enumerate([
        (BOX_W / 2.0, "side_wall_right"),
        (-BOX_W / 2.0, "side_wall_left"),
    ]):
        wall = _side_wall_plate(
            y_out, WALL_T, BACK_X, FRONT_X, BACK_H, FRONT_H, BOX_FLOOR_Z,
        )
        body.visual(
            mesh_from_geometry(wall, wall_name),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=black,
            name=wall_name,
        )

    # Sloped lid (rests on top of the walls, slopes from back to front)
    lid = _sloped_lid_plate(
        BACK_X, FRONT_X, BOX_W / 2.0, BACK_H, FRONT_H, BOX_FLOOR_Z, WALL_T,
    )
    body.visual(
        mesh_from_geometry(lid, "lid"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=black,
        name="lid",
    )

    # Front header (strip above the deposit opening)
    body.visual(
        Box((WALL_T, INNER_W, HEADER_H)),
        origin=Origin(xyz=(FRONT_X - WALL_T / 2.0, 0.0,
                           BOX_FLOOR_Z + FRONT_H - HEADER_H / 2.0)),
        material=black,
        name="front_header",
    )

    # Front sill (strip below the deposit opening)
    body.visual(
        Box((WALL_T, INNER_W, SILL_H)),
        origin=Origin(xyz=(FRONT_X - WALL_T / 2.0, 0.0,
                           BOX_FLOOR_Z + SILL_H / 2.0)),
        material=black,
        name="front_sill",
    )

    # U.S. flag decal on the +Y side wall: thin stripe plates proud of the
    # flat wall surface. Each stripe is a thin box just outside the wall.
    decal_x = -0.05
    decal_w = 0.26
    stripe_h = 0.018
    stripe_thickness = 0.002
    n_stripe = 4
    for i in range(n_stripe):
        z_local = FRONT_H * (0.70 - i * 0.14)
        body.visual(
            Box((decal_w, stripe_thickness, stripe_h)),
            origin=Origin(xyz=(decal_x,
                               BOX_W / 2.0 + stripe_thickness / 2.0,
                               BOX_FLOOR_Z + z_local)),
            material=(flag_white if i % 2 else flag_red2),
            name=f"decal_stripe_{i}",
        )
    # blue canton over the front portion of the stripe field
    z_can = FRONT_H * 0.56
    body.visual(
        Box((decal_w * 0.42, stripe_thickness, FRONT_H * 0.30)),
        origin=Origin(xyz=(decal_x - decal_w * 0.27,
                           BOX_W / 2.0 + stripe_thickness / 2.0,
                           BOX_FLOOR_Z + z_can)),
        material=flag_blue,
        name="decal_canton",
    )

    # --------------------------------------------------------- FRONT DOOR
    # Rectangular door covering the deposit opening; bottom-hinged.
    door = model.part("door")
    door.visual(
        Box((DOOR_T, DOOR_W, DOOR_H)),
        origin=Origin(xyz=(0.0, 0.0, DOOR_H / 2.0)),
        material=black,
        name="door_panel",
    )
    # small grab tab / latch knob at the top of the door
    door.visual(
        Cylinder(radius=0.010, length=0.016),
        origin=Origin(
            xyz=(0.006, 0.0, DOOR_H - 0.015),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=black_satin,
        name="door_knob",
    )

    # --------------------------------------------------------- SIGNAL FLAG
    # Red signal flag on the +Y side: an L of vertical arm + rectangular flag
    # panel, pivoting about a horizontal pin. At q=0 the flag is RAISED.
    flag = model.part("flag")
    arm_y = 0.012
    # pivot boss collar (seats against the side wall)
    flag.visual(
        Cylinder(radius=0.011, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=red,
        name="flag_boss",
    )
    # arm rising from the pivot (outboard)
    flag.visual(
        Box((0.012, 0.010, FLAG_ARM)),
        origin=Origin(xyz=(0.0, arm_y, FLAG_ARM / 2.0)),
        material=red,
        name="flag_arm",
    )
    # flag panel at the top of the arm
    flag.visual(
        Box((FLAG_W, 0.006, FLAG_H)),
        origin=Origin(
            xyz=(FLAG_W / 2.0 - 0.004, arm_y, FLAG_ARM - FLAG_H / 2.0)
        ),
        material=red,
        name="flag_panel",
    )

    # ----------------------------------------------------------- ARTICULATIONS
    model.articulation(
        "post_to_body",
        ArticulationType.FIXED,
        parent=post,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # Front door: bottom hinge along Y at the sill top, front edge.
    # Door plate extends up +Z from the hinge; axis +Y tips the top edge out.
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(FRONT_X + 0.004, 0.0, BOX_FLOOR_Z + SILL_H)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0,
                                   lower=0.0, upper=1.65),
    )

    # Signal flag: horizontal pin pointing out the +Y side wall, near front.
    # Flag arm rises +Z at q=0 (raised). Positive q rotates flag DOWN.
    flag_pivot_zl = FRONT_H * 0.35
    flag_pivot_y = BOX_W / 2.0 + WALL_T / 2.0
    model.articulation(
        "body_to_flag",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flag,
        origin=Origin(xyz=(FRONT_X - 0.10, flag_pivot_y,
                           BOX_FLOOR_Z + flag_pivot_zl)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0,
                                   lower=0.0, upper=1.9),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    post = object_model.get_part("post")
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    flag = object_model.get_part("flag")
    door_joint = object_model.get_articulation("body_to_door")
    flag_joint = object_model.get_articulation("body_to_flag")

    # --- overlap allowances for sheet-metal construction ---

    # The lid bottom surface is coplanar with the wall tops at the back edge,
    # creating a tiny local overlap (<1 mm) where the lid passes through the
    # back-wall top plane along the slope.
    ctx.allow_overlap(
        body, body,
        elem_a="lid", elem_b="back_wall",
        reason="Lid bottom seats on the back-wall top edge along the slope; "
               "tiny local embed at the back corner (<1 mm).",
    )
    ctx.allow_overlap(
        body, body,
        elem_a="lid", elem_b="side_wall_right",
        reason="Lid bottom seats on the right side-wall top edge along the slope.",
    )
    ctx.allow_overlap(
        body, body,
        elem_a="lid", elem_b="side_wall_left",
        reason="Lid bottom seats on the left side-wall top edge along the slope.",
    )

    # Signal-flag pivot boss is pinned through the right side wall.
    ctx.allow_overlap(
        flag, body,
        elem_a="flag_boss", elem_b="side_wall_right",
        reason="Flag pivot boss is pinned through the side wall at the mount.",
    )

    # --- both joints are revolute about horizontal (Y) axes ---
    for jn, j in (("door", door_joint), ("flag", flag_joint)):
        ctx.check(
            f"{jn} joint is revolute",
            str(j.articulation_type).upper().endswith("REVOLUTE"),
            details=f"type={j.articulation_type}",
        )
        ax = j.axis
        ctx.check(
            f"{jn} hinge axis is horizontal Y",
            abs(abs(ax[1]) - 1.0) < 1e-6 and abs(ax[0]) < 1e-6 and abs(ax[2]) < 1e-6,
            details=f"axis={ax}",
        )

    # --- post stands on the floor at z ~ 0 ---
    post_aabb = ctx.part_world_aabb(post)
    ctx.check(
        "post feet rest on floor (z~0)",
        post_aabb is not None and abs(post_aabb[0][2]) < 0.01,
        details=f"post_min_z={None if post_aabb is None else post_aabb[0][2]}",
    )

    # --- mailbox is raised on the post ---
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "mailbox is raised on the post",
        body_aabb is not None and body_aabb[0][2] > 0.45,
        details=f"body_min_z={None if body_aabb is None else body_aabb[0][2]}",
    )

    # --- slanted cabinet proportions ---
    if body_aabb is not None:
        bx = body_aabb[1][0] - body_aabb[0][0]
        by = body_aabb[1][1] - body_aabb[0][1]
        ctx.check(
            "cabinet length is realistic (~0.46 m)",
            0.40 < bx < 0.55,
            details=f"length_x={bx}",
        )
        ctx.check(
            "cabinet width is realistic (~0.20 m)",
            0.16 < by < 0.26,
            details=f"width_y={by}",
        )

    # --- slanted top: back of lid is higher than front header ---
    lid_aabb = ctx.part_element_world_aabb(body, elem="lid")
    header_aabb = ctx.part_element_world_aabb(body, elem="front_header")
    if lid_aabb is not None and header_aabb is not None:
        ctx.check(
            "lid back edge is higher than front header (slanted top)",
            lid_aabb[1][2] > header_aabb[1][2] + 0.02,
            details=f"lid_top_z={lid_aabb[1][2]}, header_top_z={header_aabb[1][2]}",
        )

    # --- hollow interior: deep cavity, not a solid fill ---
    back_aabb = ctx.part_element_world_aabb(body, elem="back_wall")
    if back_aabb is not None:
        depth = FRONT_X - back_aabb[1][0]
        ctx.check(
            "cabinet cavity is deep (open, not solid fill)",
            depth > 0.30,
            details=f"front_to_back_depth={depth}",
        )

    # --- cabinet has side walls (trapezoidal, not arched) ---
    sw_r_aabb = ctx.part_element_world_aabb(body, elem="side_wall_right")
    sw_l_aabb = ctx.part_element_world_aabb(body, elem="side_wall_left")
    ctx.check(
        "both side walls present",
        sw_r_aabb is not None and sw_l_aabb is not None,
        details=f"right={sw_r_aabb}, left={sw_l_aabb}",
    )

    # --- closed door covers the front opening ---
    ctx.expect_overlap(
        door, body, axes="yz", min_overlap=0.08,
        name="closed door covers the front opening",
    )

    # --- closed door seats near the front header (small Z clearance) ---
    ctx.expect_gap(
        door, body, axis="x",
        positive_elem="door_panel", negative_elem="front_header",
        min_gap=-0.010, max_gap=0.015,
        name="closed door seats near the front header",
    )

    # --- door flips DOWN and OUTWARD when opened ---
    rest_door = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: 1.55}):
        open_door = ctx.part_world_aabb(door)
    ctx.check(
        "door swings outward (+X) when open",
        rest_door is not None and open_door is not None
        and open_door[1][0] > rest_door[1][0] + 0.05,
        details=f"rest_maxX={None if rest_door is None else rest_door[1][0]}, "
                f"open_maxX={None if open_door is None else open_door[1][0]}",
    )
    ctx.check(
        "door free (top) edge drops when open",
        rest_door is not None and open_door is not None
        and open_door[1][2] < rest_door[1][2] - 0.05,
        details=f"rest_topZ={None if rest_door is None else rest_door[1][2]}, "
                f"open_topZ={None if open_door is None else open_door[1][2]}",
    )

    # --- flag is on the +Y side and raised at rest ---
    flag_aabb = ctx.part_world_aabb(flag)
    ctx.check(
        "signal flag is on the +Y side",
        flag_aabb is not None and flag_aabb[0][1] > 0.0,
        details=f"flag_min_y={None if flag_aabb is None else flag_aabb[0][1]}",
    )
    if flag_aabb is not None and body_aabb is not None:
        ctx.check(
            "flag is raised above the box at rest",
            flag_aabb[1][2] > BOX_FLOOR_Z + 0.10,
            details=f"flag_top_z={flag_aabb[1][2]}",
        )

    # --- flag lowers (panel drops) when rotated ---
    rest_flag = ctx.part_world_aabb(flag)
    with ctx.pose({flag_joint: 1.7}):
        low_flag = ctx.part_world_aabb(flag)
    ctx.check(
        "signal flag lowers when rotated",
        rest_flag is not None and low_flag is not None
        and low_flag[1][2] < rest_flag[1][2] - 0.05,
        details=f"rest_topZ={None if rest_flag is None else rest_flag[1][2]}, "
                f"low_topZ={None if low_flag is None else low_flag[1][2]}",
    )

    return ctx.report()
