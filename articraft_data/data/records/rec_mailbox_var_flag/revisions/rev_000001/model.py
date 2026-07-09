from __future__ import annotations

# Curbside blue street collection box on a single steel post.
#
# A tall slanted-top sheet-metal cabinet (the kind used for newspaper / charity
# collection on a sidewalk) raised up on one square steel post that is anchored
# at the ground. The cabinet front carries a pull-down hopper deposit door that
# swings open about a BOTTOM hinge so a passer-by can drop items into the chute.
#
# The cabinet is a genuinely HOLLOW sheet-metal shell: back wall, two side walls,
# floor, a roof, and a front face with only a sill below and a header above the
# door opening. Nothing fills the middle, so when the deposit door swings down
# the open mouth reveals the real dark interior cavity all the way back to the
# back wall, not a solid block.
#
# Coordinate convention:
#   - up is +Z; the post base sits on the floor at z = 0.
#   - the cabinet FRONT (the deposit door) faces +X; the back wall is -X.
#   - the cabinet is centered on the post in Y (centerline y = 0).
#   - the peaked / slanted top falls from the back (-X, high) toward the
#     front (+X, low), as in the reference photo.
#
# Root structure: the steel post is the root. The cabinet body is fixed on top
# of the post. The hopper deposit door hangs from a bottom hinge on the lower
# front edge of the cabinet (REVOLUTE), opening outward and down.
#
# Articulations:
#   - body_to_door : REVOLUTE pull-down hopper deposit door (bottom hinge)

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

# ---- key dimensions (meters) -------------------------------------------------
POST_SIDE = 0.105          # square steel post cross-section
POST_HEIGHT = 1.46         # post height from floor up to cabinet underside
POST_TOP_Z = POST_HEIGHT   # z of the post top (cabinet underside sits here)

BOX_W = 0.42               # cabinet width  (along Y)
BOX_D = 0.34               # cabinet depth  (along X, front-back)
BOX_H = 0.50               # cabinet front-wall height (vertical body)
TOP_RISE = 0.085           # extra height the peak adds at the high (back) edge

BOX_BOT_Z = POST_TOP_Z     # cabinet body bottom
BOX_CTR_Z = BOX_BOT_Z + BOX_H / 2.0
BOX_TOP_Z = BOX_BOT_Z + BOX_H

WALL_T = 0.010             # sheet-metal wall thickness

# front deposit door (pull-down hopper) on the front (+X) face
DOOR_W = 0.34              # door width  (along Y)
DOOR_H = 0.26              # door height (along the front face)
DOOR_T = 0.018            # door panel thickness
DOOR_BOT_OFF = 0.07        # door bottom sits this far above the cabinet bottom
HANDLE_R = 0.012

FRONT_X = BOX_D / 2.0      # front face plane (x)
BACK_X = -BOX_D / 2.0      # back wall plane (x)

# semaphore flag pivot on the right side wall (-Y)
FLAG_PIVOT_X = 0.05
FLAG_PIVOT_Y = -BOX_W / 2.0
FLAG_PIVOT_Z = BOX_BOT_Z + BOX_H * 0.65


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="curbside_collection_box")

    steel = model.material("steel", rgba=(0.42, 0.45, 0.50, 1.0))
    box_blue = model.material("box_blue", rgba=(0.40, 0.46, 0.55, 1.0))
    box_blue_dk = model.material("box_blue_dk", rgba=(0.30, 0.35, 0.43, 1.0))
    door_blue = model.material("door_blue", rgba=(0.46, 0.52, 0.60, 1.0))
    interior = model.material("interior", rgba=(0.07, 0.08, 0.10, 1.0))
    chrome = model.material("chrome", rgba=(0.72, 0.74, 0.78, 1.0))
    base_plate = model.material("base_plate", rgba=(0.33, 0.35, 0.39, 1.0))
    flag_red = model.material("flag_red", rgba=(0.82, 0.12, 0.10, 1.0))

    # -------------------------------------------------------------- POST (root)
    post = model.part("post")
    # main square steel column
    post.visual(
        Box((POST_SIDE, POST_SIDE, POST_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, POST_HEIGHT / 2.0)),
        material=steel,
        name="post_column",
    )
    # ground base / mounting plate (flat plate bolted to the sidewalk)
    post.visual(
        Box((0.22, 0.22, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=base_plate,
        name="base_plate",
    )
    # a short shoulder collar where the post meets the cabinet underside
    post.visual(
        Box((0.16, 0.16, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, POST_TOP_Z - 0.02)),
        material=steel,
        name="post_collar",
    )

    # ------------------------------------------------------------- CABINET BODY
    # A hollow shell. Each wall is a thin sheet-metal panel; the interior between
    # them is left completely empty so the open door reveals a real cavity.
    body = model.part("body")

    # back wall (-X) -- full height, dark interior face deep behind the opening
    body.visual(
        Box((WALL_T, BOX_W, BOX_H)),
        origin=Origin(xyz=(BACK_X + WALL_T / 2.0, 0.0, BOX_CTR_Z)),
        material=box_blue_dk,
        name="back_wall",
    )
    # thin dark interior liner on the front face of the back wall, so the depth
    # seen through the open door reads as a dark hollow chute (not painted blue).
    body.visual(
        Box((0.004, BOX_W - 2 * WALL_T, BOX_H - 2 * WALL_T)),
        origin=Origin(xyz=(BACK_X + WALL_T + 0.002, 0.0, BOX_CTR_Z)),
        material=interior,
        name="interior_back_liner",
    )
    # left side wall (+Y)
    body.visual(
        Box((BOX_D, WALL_T, BOX_H)),
        origin=Origin(xyz=(0.0, BOX_W / 2.0 - WALL_T / 2.0, BOX_CTR_Z)),
        material=box_blue,
        name="side_wall_0",
    )
    # right side wall (-Y)
    body.visual(
        Box((BOX_D, WALL_T, BOX_H)),
        origin=Origin(xyz=(0.0, -BOX_W / 2.0 + WALL_T / 2.0, BOX_CTR_Z)),
        material=box_blue,
        name="side_wall_1",
    )
    # bottom floor
    body.visual(
        Box((BOX_D, BOX_W, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, BOX_BOT_Z + WALL_T / 2.0)),
        material=box_blue_dk,
        name="floor",
    )

    # front wall (+X) -- a header strip above the door and a sill strip below it,
    # leaving an opening for the deposit door. (Top header is taller.) The middle
    # is OPEN: the only thing behind it is the empty interior cavity.
    door_open_bot = BOX_BOT_Z + DOOR_BOT_OFF
    door_open_top = door_open_bot + DOOR_H
    sill_h = DOOR_BOT_OFF
    header_h = BOX_H - DOOR_BOT_OFF - DOOR_H
    body.visual(
        Box((WALL_T, BOX_W, sill_h)),
        origin=Origin(xyz=(FRONT_X - WALL_T / 2.0, 0.0, BOX_BOT_Z + sill_h / 2.0)),
        material=box_blue,
        name="front_sill",
    )
    body.visual(
        Box((WALL_T, BOX_W, header_h)),
        origin=Origin(
            xyz=(FRONT_X - WALL_T / 2.0, 0.0, door_open_top + header_h / 2.0)
        ),
        material=box_blue,
        name="front_header",
    )
    # narrow front jambs on each side of the door opening (so the opening reads
    # as a framed mouth rather than the full cabinet width).
    jamb_w = 0.03
    for sgn, jn in ((1.0, "0"), (-1.0, "1")):
        body.visual(
            Box((WALL_T, jamb_w, DOOR_H)),
            origin=Origin(
                xyz=(
                    FRONT_X - WALL_T / 2.0,
                    sgn * (BOX_W / 2.0 - jamb_w / 2.0),
                    (door_open_bot + door_open_top) / 2.0,
                ),
            ),
            material=box_blue,
            name=f"front_jamb_{jn}",
        )

    # peaked / slanted top: a wedge that is high at the back (-X) and low at the
    # front (+X). A single sloped lid box closes the top of the hollow shell.
    slope_len = math.hypot(BOX_D, TOP_RISE)
    slope_pitch = math.atan2(TOP_RISE, BOX_D)  # tilt down toward +X
    body.visual(
        Box((slope_len, BOX_W, WALL_T)),
        origin=Origin(
            xyz=(0.0, 0.0, BOX_TOP_Z + TOP_RISE / 2.0),
            rpy=(0.0, slope_pitch, 0.0),
        ),
        material=box_blue,
        name="slanted_top",
    )
    # close the triangular gable on each side (back is higher than front)
    gable_back = model.material("gable_back", rgba=(0.34, 0.40, 0.49, 1.0))
    for sign, yname in ((1.0, "0"), (-1.0, "1")):
        body.visual(
            Box((WALL_T, WALL_T, TOP_RISE)),
            origin=Origin(
                xyz=(
                    -BOX_D / 2.0 + WALL_T / 2.0,
                    sign * (BOX_W / 2.0 - WALL_T / 2.0),
                    BOX_TOP_Z + TOP_RISE / 2.0,
                ),
            ),
            material=gable_back,
            name=f"gable_post_{yname}",
        )

    # ---------------------------------------------------------- DEPOSIT DOOR
    # Pull-down hopper door: hinged along its BOTTOM edge on the front face, so it
    # tips outward/down. Modeled in its CLOSED pose (vertical, flush in the front
    # opening). The hinge frame is placed at the bottom edge of the door opening.
    door = model.part("door")
    # door panel: at q=0 it is vertical; its frame origin is the bottom hinge
    # line, so the panel extends upward (+Z) from the hinge.
    door.visual(
        Box((DOOR_T, DOOR_W, DOOR_H)),
        origin=Origin(xyz=(0.0, 0.0, DOOR_H / 2.0)),
        material=door_blue,
        name="door_panel",
    )
    # recessed pull lip / handle bar near the top of the door
    door.visual(
        Cylinder(radius=HANDLE_R, length=DOOR_W * 0.55),
        origin=Origin(
            xyz=(DOOR_T / 2.0 + HANDLE_R * 0.6, 0.0, DOOR_H - 0.045),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=chrome,
        name="door_handle",
    )
    # small raised label plate on the door
    door.visual(
        Box((0.004, DOOR_W * 0.6, DOOR_H * 0.4)),
        origin=Origin(xyz=(DOOR_T / 2.0 + 0.002, 0.0, DOOR_H * 0.42)),
        material=box_blue_dk,
        name="door_label",
    )

    # --------------------------------------------------------- SEMAPHORE FLAG
    # L-shaped signal flag on the RIGHT side (-Y) of the cabinet. A short pivot
    # boss protrudes from the side wall; a vertical arm extends upward from the
    # boss; a rectangular red flag panel sits at the arm tip, extending in +X.
    # At rest (q=0) the arm is vertical (flag raised). Positive q lowers the arm
    # forward (toward +X), the standard outgoing-mail signal.
    FLAG_BOSS_R = 0.011
    FLAG_BOSS_LEN = 0.013
    FLAG_ARM_L = 0.13
    FLAG_ARM_W = 0.013
    FLAG_ARM_T = 0.004
    FLAG_PNL_DX = 0.055
    FLAG_PNL_DZ = 0.038
    FLAG_PNL_DY = 0.003

    flag = model.part("flag")

    # pivot boss: short cylinder protruding outward from the side wall (-Y)
    flag.visual(
        Cylinder(radius=FLAG_BOSS_R, length=FLAG_BOSS_LEN),
        origin=Origin(
            xyz=(0.0, -FLAG_BOSS_LEN / 2.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=chrome,
        name="pivot_boss",
    )
    # vertical arm: thin flat bar extending upward from the boss end
    flag.visual(
        Box((FLAG_ARM_T, FLAG_ARM_W, FLAG_ARM_L)),
        origin=Origin(xyz=(0.0, -FLAG_BOSS_LEN, FLAG_ARM_L / 2.0)),
        material=flag_red,
        name="flag_arm",
    )
    # rectangular flag panel at the arm tip, extending in +X
    flag.visual(
        Box((FLAG_PNL_DX, FLAG_PNL_DY, FLAG_PNL_DZ)),
        origin=Origin(
            xyz=(FLAG_PNL_DX / 2.0, -FLAG_BOSS_LEN, FLAG_ARM_L - FLAG_PNL_DZ / 2.0)
        ),
        material=flag_red,
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

    # Bottom-hinge pull-down door. Hinge line lies along Y at the bottom of the
    # door opening, just outside the front face. Door geometry extends +Z from
    # the hinge; axis +Y makes positive q tip the free (top) edge outward/down.
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(FRONT_X + DOOR_T / 2.0, 0.0, door_open_bot)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=1.4),
    )

    # Side semaphore flag. Hinge pin is horizontal along Y at the boss on the
    # right side wall. At q=0 the arm points +Z (raised). Axis +Y rotates the
    # arm from +Z toward +X (forward/down) with positive q, lowering the flag.
    model.articulation(
        "body_to_flag",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flag,
        origin=Origin(xyz=(FLAG_PIVOT_X, FLAG_PIVOT_Y, FLAG_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=4.0, lower=0.0, upper=math.pi / 2.0
        ),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    post = object_model.get_part("post")
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    door_joint = object_model.get_articulation("body_to_door")

    # The thin dark interior liner sits just in front of the back wall inside the
    # shell; that contained contact is intentional, not a collision.
    ctx.allow_overlap(
        body, body,
        elem_a="interior_back_liner", elem_b="back_wall",
        reason="Dark interior liner is bonded to the inner face of the back wall.",
    )

    # --- joint is a bottom-hinge revolute about a horizontal (Y) axis ----------
    ctx.check(
        "door joint is revolute",
        str(door_joint.articulation_type).upper().endswith("REVOLUTE"),
        details=f"type={door_joint.articulation_type}",
    )
    ax = door_joint.axis
    ctx.check(
        "door hinge axis is horizontal Y",
        abs(abs(ax[1]) - 1.0) < 1e-6 and abs(ax[0]) < 1e-6 and abs(ax[2]) < 1e-6,
        details=f"axis={ax}",
    )

    # --- base sits on the floor at z ~ 0 --------------------------------------
    post_aabb = ctx.part_world_aabb(post)
    ctx.check(
        "post base rests on floor (z~0)",
        post_aabb is not None and abs(post_aabb[0][2]) < 0.01,
        details=f"post_min_z={None if post_aabb is None else post_aabb[0][2]}",
    )

    # --- cabinet is raised up on the post (mounted high, not at the ground) ----
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "cabinet body is raised on the post",
        body_aabb is not None and body_aabb[0][2] > 1.0,
        details=f"body_min_z={None if body_aabb is None else body_aabb[0][2]}",
    )

    # --- cabinet sits ON TOP of the post: post top reaches the cabinet floor ---
    ctx.expect_contact(
        post, body, elem_a="post_collar", elem_b="floor", contact_tol=0.005,
        name="post supports the cabinet floor",
    )

    # --- HOLLOW INTERIOR: the cavity behind the door opening is genuinely deep.
    # The back liner (the only thing the open mouth reveals) must sit far behind
    # the front opening, proving an open chute rather than a shallow false back.
    back_aabb = ctx.part_element_world_aabb(body, elem="interior_back_liner")
    if back_aabb is not None:
        depth = FRONT_X - back_aabb[1][0]
        ctx.check(
            "interior cavity is deep (open, not a solid/shallow back)",
            depth > 0.25,
            details=f"front_to_back_depth={depth}",
        )

    # --- door is in front of the cabinet front opening at rest -----------------
    ctx.expect_overlap(
        door, body, axes="yz", min_overlap=0.10,
        name="closed door covers the front opening",
    )

    # --- closed door is flush against the front face (small seam, no big gap) ---
    ctx.expect_gap(
        door, body, axis="x",
        positive_elem="door_panel", negative_elem="front_header",
        min_gap=-0.005, max_gap=0.012,
        name="closed door is flush with the front face",
    )

    # --- key dimensions roughly match the real object --------------------------
    if body_aabb is not None:
        body_h = body_aabb[1][2] - body_aabb[0][2]
        ctx.check(
            "cabinet height is realistic",
            0.45 < body_h < 0.70,
            details=f"body_height={body_h}",
        )
    # whole-object top should clear ~1.9 m
    ctx.check(
        "overall stands ~1.5-2.1 m tall",
        body_aabb is not None and 1.7 < body_aabb[1][2] < 2.3,
        details=f"top_z={None if body_aabb is None else body_aabb[1][2]}",
    )

    # --- opened pose: the free edge swings outward (+X) and down --------------
    rest_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: 1.3}):
        open_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door opens outward in +X",
        rest_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][0] > (rest_door_aabb[1][0] + 0.05),
        details=f"rest_maxX={None if rest_door_aabb is None else rest_door_aabb[1][0]}, "
        f"open_maxX={None if open_door_aabb is None else open_door_aabb[1][0]}",
    )
    ctx.check(
        "door free edge drops when open",
        rest_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][2] < rest_door_aabb[1][2] - 0.05,
        details=f"rest_topZ={None if rest_door_aabb is None else rest_door_aabb[1][2]}, "
        f"open_topZ={None if open_door_aabb is None else open_door_aabb[1][2]}",
    )

    # --- SEMAPHORE FLAG -------------------------------------------------------
    flag = object_model.get_part("flag")
    flag_joint = object_model.get_articulation("body_to_flag")

    ctx.check(
        "flag joint is revolute",
        str(flag_joint.articulation_type).upper().endswith("REVOLUTE"),
        details=f"type={flag_joint.articulation_type}",
    )
    fax = flag_joint.axis
    ctx.check(
        "flag pivot axis is horizontal Y",
        abs(abs(fax[1]) - 1.0) < 1e-6 and abs(fax[0]) < 1e-6 and abs(fax[2]) < 1e-6,
        details=f"axis={fax}",
    )

    # At rest the arm is vertical (raised); the flag panel top should be high
    # above the pivot z.
    rest_flag_aabb = ctx.part_world_aabb(flag)
    ctx.check(
        "flag is raised at rest (arm vertical)",
        rest_flag_aabb is not None
        and rest_flag_aabb[1][2] > FLAG_PIVOT_Z + 0.10,
        details=f"flag_top_z={None if rest_flag_aabb is None else rest_flag_aabb[1][2]}",
    )

    # Positive q lowers the flag: the top of the flag drops.
    with ctx.pose({flag_joint: math.pi / 2.0}):
        lowered_flag_aabb = ctx.part_world_aabb(flag)
    ctx.check(
        "positive q lowers the flag (top drops)",
        rest_flag_aabb is not None
        and lowered_flag_aabb is not None
        and lowered_flag_aabb[1][2] < rest_flag_aabb[1][2] - 0.05,
        details=(
            f"rest_top={None if rest_flag_aabb is None else rest_flag_aabb[1][2]}, "
            f"lowered_top={None if lowered_flag_aabb is None else lowered_flag_aabb[1][2]}"
        ),
    )

    # Flag is on the right side (-Y) of the body
    ctx.check(
        "flag is mounted on the right side of the body",
        rest_flag_aabb is not None
        and rest_flag_aabb[0][1] < -0.10,
        details=f"flag_min_y={None if rest_flag_aabb is None else rest_flag_aabb[0][1]}",
    )

    # Boss is in contact with the side wall (seated mount)
    ctx.expect_contact(
        flag, body,
        elem_a="pivot_boss", elem_b="side_wall_1",
        contact_tol=0.005,
        name="flag boss is seated against the right side wall",
    )

    return ctx.report()
