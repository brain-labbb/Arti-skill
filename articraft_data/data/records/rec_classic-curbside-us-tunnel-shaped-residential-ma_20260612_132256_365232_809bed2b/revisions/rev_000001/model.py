from __future__ import annotations

# Classic curbside U.S. tunnel-shaped residential mailbox on a scrollwork post.
#
# The familiar arched ("tunnel" / half-cylinder) sheet-metal mailbox, mounted up
# on a decorative scroll-iron post. It has TWO articulations:
#   1. the front DOOR (the flat tombstone-shaped door on the open end) flips down
#      on a BOTTOM hinge to open the mailbox (REVOLUTE);
#   2. the red signal FLAG on the side rotates up/down about a horizontal pin
#      (REVOLUTE) to signal outgoing mail.
# A U.S.-flag decal panel rides on the arched body side.
#
# The arched body is a genuinely HOLLOW tunnel: a thin sheet-metal wall built
# from an OUTER arched skin and a concentric INNER arched skin (offset inward by
# the wall thickness), closed by a floor and a dark back wall. There is NO solid
# fill inside, so opening the front door reveals the real empty arched cavity
# running back to the dark back wall, not a solid block.
#
# Coordinate convention:
#   - up is +Z; the post legs stand on the floor at z = 0.
#   - the tunnel axis runs along X; the open end / DOOR faces +X (front).
#   - the closed back end is at -X; centerline is y = 0.
#   - the arched cross-section lies in the YZ plane (dome up in +Z).
#   - the signal flag is on the +Y side.
#
# Root structure: the POST is the root resting on the floor, and the box is
# fixed on top of the post. The door and the flag are children of the box.
#
# Articulations:
#   - body_to_door : REVOLUTE front door, bottom hinge (flips down to open)
#   - body_to_flag : REVOLUTE side signal flag, horizontal pin (raises/lowers)

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
BOX_R = 0.105             # arched cross-section radius (half-width / dome height)
BOX_LEN = 0.46            # tunnel length along X
WALL_T = 0.006

BOX_FLOOR_Z = 0.55        # mailbox floor height above the ground (on the post)
FRONT_X = BOX_LEN / 2.0   # front opening plane (the door end)
BACK_X = -BOX_LEN / 2.0

# scrollwork post
POST_LEG = 0.022          # square leg cross-section
POST_SPACING = 0.16       # spacing between the two legs (along X)
POST_TOP_Z = BOX_FLOOR_Z  # legs reach up to the mailbox floor
SCROLL_TUBE = 0.012       # scroll iron tube radius

# signal flag
FLAG_W = 0.085            # flag panel width (along its arm)
FLAG_H = 0.075            # flag panel height
FLAG_ARM = 0.10           # arm length from pivot up to the flag


def _arch_skin(r: float, reverse: bool = False) -> MeshGeometry:
    # A single arched half-cylinder surface (no thickness), axis along X, open at
    # both X ends and open along the flat bottom. Cross-section is a half-circle
    # in YZ (z >= 0). `reverse` flips the winding so the surface faces inward
    # (used for the inner skin, visible from inside the cavity).
    n = 28
    g = MeshGeometry()
    arc = []
    for i in range(n + 1):
        a = math.pi * i / n  # 0..pi
        arc.append((r * math.cos(a), r * math.sin(a)))
    front_ids = [g.add_vertex(FRONT_X, y, z) for (y, z) in arc]
    back_ids = [g.add_vertex(BACK_X, y, z) for (y, z) in arc]
    for i in range(n):
        a0, a1 = front_ids[i], front_ids[i + 1]
        b0, b1 = back_ids[i], back_ids[i + 1]
        if not reverse:
            g.add_face(a0, b1, b0)
            g.add_face(a0, a1, b1)
        else:
            g.add_face(a0, b0, b1)
            g.add_face(a0, b1, a1)
    return g


def _arch_mouth_ring(r_out: float, r_in: float, x: float) -> MeshGeometry:
    # A flat annular ring (front-facing, +X normal) spanning the wall thickness
    # at the mouth, so the open front shows a finished sheet-metal edge rather
    # than a paper-thin lip. A two-sided ring (both windings) so it reads from
    # either side.
    n = 26
    g = MeshGeometry()
    out = []
    inn = []
    for i in range(n + 1):
        a = math.pi * i / n
        out.append(g.add_vertex(x, r_out * math.cos(a), r_out * math.sin(a)))
        inn.append(g.add_vertex(x, r_in * math.cos(a), r_in * math.sin(a)))
    for i in range(n):
        g.add_face(out[i], inn[i], inn[i + 1])
        g.add_face(out[i], inn[i + 1], out[i + 1])
        # back side too
        g.add_face(out[i], inn[i + 1], inn[i])
        g.add_face(out[i], out[i + 1], inn[i + 1])
    return g


def _door_plate(r: float, t: float) -> MeshGeometry:
    # Flat D-shaped door plate of thickness t along local X. Flat base on local
    # z=0, arch up in +Z; spans Y in [-r, r]. (Matches the arched front opening.)
    n = 26
    loop = []
    loop.append((-r, 0.0))
    loop.append((r, 0.0))
    for i in range(1, n):
        a = math.pi * i / n
        loop.append((r * math.cos(a), r * math.sin(a)))
    g = MeshGeometry()
    front = [g.add_vertex(t / 2.0, y, z) for (y, z) in loop]
    back = [g.add_vertex(-t / 2.0, y, z) for (y, z) in loop]
    m = len(loop)
    # caps (triangle fans around vertex 0)
    for i in range(1, m - 1):
        g.add_face(front[0], front[i], front[i + 1])
        g.add_face(back[0], back[i + 1], back[i])
    # rim
    for i in range(m):
        j = (i + 1) % m
        g.add_face(front[i], back[i], back[j])
        g.add_face(front[i], back[j], front[j])
    return g


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="classic_us_mailbox")

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
    # two square legs reaching from the ground up to just under the mailbox floor
    leg_h = BOX_FLOOR_Z - WALL_T
    for i, xx in enumerate((-POST_SPACING / 2.0, POST_SPACING / 2.0)):
        post.visual(
            Box((POST_LEG, POST_LEG, leg_h)),
            origin=Origin(xyz=(xx, 0.0, leg_h / 2.0)),
            material=iron,
            name=f"post_leg_{i}",
        )
        # foot pad on the ground
        post.visual(
            Box((POST_LEG + 0.02, POST_LEG + 0.02, 0.012)),
            origin=Origin(xyz=(xx, 0.0, 0.006)),
            material=black_satin,
            name=f"post_foot_{i}",
        )
    # top cross member that the mailbox floor bolts to (its top just meets the
    # underside of the mailbox floor)
    crossbar_top = BOX_FLOOR_Z - WALL_T
    post.visual(
        Box((POST_SPACING + POST_LEG, 0.05, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, crossbar_top - 0.009)),
        material=iron,
        name="post_crossbar",
    )
    # decorative scroll ironwork: a stack of heart/scroll rings between the legs
    n_scroll = 3
    z0 = 0.12
    z1 = POST_TOP_Z - 0.10
    for k in range(n_scroll):
        zc = z0 + (z1 - z0) * k / (n_scroll - 1)
        # two opposed C-scrolls forming a heart between the legs (tori, axis Y)
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

    # --------------------------------------------------------- MAILBOX BODY
    # Hollow arched tunnel: outer skin (visible from outside) + inner skin
    # (visible from inside the cavity) + floor + dark back wall. No solid fill.
    body = model.part("body")
    outer = _arch_skin(BOX_R, reverse=False)
    body.visual(
        mesh_from_geometry(outer, "arched_shell"),
        origin=Origin(xyz=(0.0, 0.0, BOX_FLOOR_Z)),
        material=black,
        name="arched_shell",
    )
    inner = _arch_skin(BOX_R - WALL_T, reverse=True)
    body.visual(
        mesh_from_geometry(inner, "arched_inner"),
        origin=Origin(xyz=(0.0, 0.0, BOX_FLOOR_Z)),
        material=interior,
        name="arched_inner",
    )
    # finished sheet-metal mouth ring around the front opening (annulus, not a
    # solid disk -- the cavity stays open through it).
    mouth = _arch_mouth_ring(BOX_R, BOX_R - WALL_T, FRONT_X - 0.002)
    body.visual(
        mesh_from_geometry(mouth, "front_rim"),
        origin=Origin(xyz=(0.0, 0.0, BOX_FLOOR_Z)),
        material=black,
        name="front_rim",
    )
    # flat floor plate of the mailbox (interior dark so the cavity bottom reads
    # as a hollow chute floor)
    body.visual(
        Box((BOX_LEN, 2.0 * (BOX_R - WALL_T), WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, BOX_FLOOR_Z - WALL_T / 2.0)),
        material=interior,
        name="box_floor",
    )
    # closed back wall (D-shaped) at -X, dark interior face deep in the cavity
    back_plate = _door_plate(BOX_R - 0.001, WALL_T)
    body.visual(
        mesh_from_geometry(back_plate, "back_wall"),
        origin=Origin(xyz=(BACK_X + WALL_T / 2.0, 0.0, BOX_FLOOR_Z)),
        material=interior,
        name="back_wall",
    )

    # U.S. flag decal on the +Y arched side: thin stripe plates that hug the
    # curved shell. Each stripe sits at the shell radius for its own z height so
    # its inner edge contacts the shell (no floating decal). Stripes are stacked
    # close together over the upper arch.
    decal_x = -0.05
    decal_w = 0.26   # along X
    stripe_h = 0.018
    n_stripe = 4
    for i in range(n_stripe):
        # center each stripe at a z on the upper arch (well above mid-height)
        z_local = BOX_R * (0.58 - i * 0.135)  # local z above floor
        z_off = BOX_FLOOR_Z + z_local
        # shell surface y at this z (positive +Y side)
        y_surf = math.sqrt(max(BOX_R * BOX_R - z_local * z_local, 1e-6))
        body.visual(
            Box((decal_w, 0.012, stripe_h)),
            origin=Origin(xyz=(decal_x, y_surf - 0.005, z_off)),
            material=(flag_white if i % 2 else flag_red2),
            name=f"decal_stripe_{i}",
        )
    # blue canton over the front portion of the stripe field
    z_can = BOX_R * 0.52
    y_can = math.sqrt(max(BOX_R * BOX_R - z_can * z_can, 1e-6))
    body.visual(
        Box((decal_w * 0.42, 0.012, BOX_R * 0.34)),
        origin=Origin(
            xyz=(decal_x - decal_w * 0.27, y_can - 0.005, BOX_FLOOR_Z + z_can)
        ),
        material=flag_blue,
        name="decal_canton",
    )

    # --------------------------------------------------------- FRONT DOOR
    # D-shaped door covering the front opening; bottom-hinged so it flips down.
    door = model.part("door")
    door_plate = _door_plate(BOX_R - 0.002, 0.008)
    # At q=0 the door is vertical in the YZ plane; its hinge frame is at the
    # bottom front edge, so the plate's flat base sits on the hinge line and it
    # arches up in +Z.
    door.visual(
        mesh_from_geometry(door_plate, "door_panel"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=black,
        name="door_panel",
    )
    # small grab tab / latch knob at the top of the door
    door.visual(
        Cylinder(radius=0.010, length=0.016),
        origin=Origin(
            xyz=(0.006, 0.0, BOX_R - 0.025),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=black_satin,
        name="door_knob",
    )

    # --------------------------------------------------------- SIGNAL FLAG
    # Red signal flag on the +Y side: an L of a vertical arm + a rectangular flag
    # panel, pivoting about a horizontal pin that points out the side (+Y). At
    # q=0 the flag is RAISED (arm vertical). Lowering rotates it down/forward.
    flag = model.part("flag")
    arm_y = 0.012   # outboard offset of the arm/panel from the pivot
    # pivot boss collar (short cylinder along the pin, seats against the shell)
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
    # flag panel at the top of the arm (outboard)
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

    # Front door: bottom hinge along Y at the bottom front edge of the opening.
    # Door plate arches up +Z from the hinge; axis +Y tips the top edge out/down.
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(FRONT_X + 0.004, 0.0, BOX_FLOOR_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.65),
    )

    # Signal flag: horizontal pin pointing out the +Y side, near the front. The
    # flag arm rises +Z at q=0 (raised). Positive q rotates the flag DOWN about
    # the side pin.
    flag_pivot_zl = 0.04  # local z above floor
    flag_pivot_y = math.sqrt(BOX_R * BOX_R - flag_pivot_zl * flag_pivot_zl) + 0.006
    model.articulation(
        "body_to_flag",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flag,
        origin=Origin(xyz=(FRONT_X - 0.10, flag_pivot_y, BOX_FLOOR_Z + flag_pivot_zl)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=0.0, upper=1.9),
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

    # The inner skin / mouth ring / floor / back wall are concentric pieces of
    # the one hollow shell and intentionally meet at shared edges.
    ctx.allow_overlap(
        body, body,
        elem_a="arched_inner", elem_b="arched_shell",
        reason="Inner arched skin nests just inside the outer arched skin (the wall).",
    )
    ctx.allow_overlap(
        body, body,
        elem_a="front_rim", elem_b="arched_shell",
        reason="Mouth ring closes the wall thickness at the front edge of the shell.",
    )
    ctx.allow_overlap(
        body, body,
        elem_a="back_wall", elem_b="arched_inner",
        reason="Back wall seals the rear of the arched cavity against the inner skin.",
    )
    # The signal-flag pivot boss intentionally seats a few mm against the arched
    # side shell, like a real bracket pinned through the side wall.
    ctx.allow_overlap(
        flag, body,
        elem_a="flag_boss", elem_b="arched_shell",
        reason="Flag pivot boss is pinned through the side shell at the mount.",
    )
    ctx.allow_overlap(
        flag, body,
        elem_a="flag_boss", elem_b="arched_inner",
        reason="Flag pivot boss passes through the inner skin where it is pinned.",
    )

    # --- both joints are revolute about horizontal (Y) axes -------------------
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

    # --- post stands on the floor at z ~ 0 ------------------------------------
    post_aabb = ctx.part_world_aabb(post)
    ctx.check(
        "post feet rest on floor (z~0)",
        post_aabb is not None and abs(post_aabb[0][2]) < 0.01,
        details=f"post_min_z={None if post_aabb is None else post_aabb[0][2]}",
    )

    # --- mailbox is raised on the post ----------------------------------------
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "mailbox is raised on the post",
        body_aabb is not None and body_aabb[0][2] > 0.45,
        details=f"body_min_z={None if body_aabb is None else body_aabb[0][2]}",
    )

    # --- arched body proportions (tunnel: wider/longer than tall dome) ---------
    if body_aabb is not None:
        bx = body_aabb[1][0] - body_aabb[0][0]
        by = body_aabb[1][1] - body_aabb[0][1]
        ctx.check(
            "tunnel length is realistic (~0.46 m)",
            0.40 < bx < 0.55,
            details=f"length_x={bx}",
        )
        ctx.check(
            "tunnel width is realistic (~0.21 m)",
            0.18 < by < 0.26,
            details=f"width_y={by}",
        )

    # --- HOLLOW INTERIOR: the cavity is genuinely deep and open. The dark back
    # wall the open door reveals sits far behind the front mouth, and the inner
    # skin exists (proving a hollow tube, not a solid block).
    back_aabb = ctx.part_element_world_aabb(body, elem="back_wall")
    if back_aabb is not None:
        depth = FRONT_X - back_aabb[1][0]
        ctx.check(
            "arched cavity is deep (open, not a solid fill)",
            depth > 0.30,
            details=f"front_to_back_depth={depth}",
        )
    inner_aabb = ctx.part_element_world_aabb(body, elem="arched_inner")
    ctx.check(
        "inner arched skin present (hollow tube wall)",
        inner_aabb is not None,
        details=f"inner_aabb={inner_aabb}",
    )

    # --- closed door covers the front opening ---------------------------------
    ctx.expect_overlap(
        door, body, axes="yz", min_overlap=0.10,
        name="closed door covers the front opening",
    )
    # --- closed door seats at the front rim (roughly flush, small seam) -------
    ctx.expect_gap(
        door, body, axis="x",
        positive_elem="door_panel", negative_elem="front_rim",
        min_gap=-0.008, max_gap=0.014,
        name="closed door seats on the front rim",
    )

    # --- door flips DOWN and OUTWARD when opened ------------------------------
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

    # --- flag is on the +Y side and raised at rest ----------------------------
    flag_aabb = ctx.part_world_aabb(flag)
    ctx.check(
        "signal flag is on the +Y side",
        flag_aabb is not None and flag_aabb[0][1] > 0.0,
        details=f"flag_min_y={None if flag_aabb is None else flag_aabb[0][1]}",
    )
    # raised flag panel sits above its pivot
    if flag_aabb is not None and body_aabb is not None:
        ctx.check(
            "flag is raised above the box at rest",
            flag_aabb[1][2] > BOX_FLOOR_Z + 0.10,
            details=f"flag_top_z={flag_aabb[1][2]}",
        )

    # --- flag lowers (panel drops) when rotated -------------------------------
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
