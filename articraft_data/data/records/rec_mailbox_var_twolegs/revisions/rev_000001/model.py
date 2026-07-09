from __future__ import annotations

# Curbside blue street collection box on a twin-leg ground stand.
#
# A tall slanted-top sheet-metal cabinet (the kind used for newspaper / charity
# collection on a sidewalk) raised up on a pair of vertical square-tube steel
# legs — one on each side — standing on the ground at z = 0 with small foot
# pads, tied together by a horizontal cross member just under the cabinet floor.
#
# The cabinet is a genuinely HOLLOW sheet-metal shell: back wall, two side walls,
# floor, a roof, and a front face with only a sill below and a header above the
# door opening. Nothing fills the middle, so when the deposit door swings down
# the open mouth reveals the real dark interior cavity all the way back to the
# back wall, not a solid block.
#
# Coordinate convention:
#   - up is +Z; the leg foot pads sit on the floor at z = 0.
#   - the cabinet FRONT (the deposit door) faces +X; the back wall is -X.
#   - the cabinet is centered between the legs in Y (centerline y = 0).
#   - the peaked / slanted top falls from the back (-X, high) toward the
#     front (+X, low), as in the reference photo.
#
# Root structure: the twin-leg frame is the root. The cabinet body is fixed on
# top of the frame. The hopper deposit door hangs from a bottom hinge on the
# lower front edge of the cabinet (REVOLUTE), opening outward and down.
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
LEG_SIDE = 0.045             # square-tube leg cross-section
LEG_HEIGHT = 1.46            # leg height from floor up to cabinet underside
LEG_TOP_Z = LEG_HEIGHT       # z of the leg tops (cabinet underside sits here)
LEG_SPREAD = 0.32            # center-to-center distance between the two legs (Y)

FOOT_PAD_SIDE = 0.08         # foot pad width/depth
FOOT_PAD_H = 0.012           # foot pad thickness

CROSS_W = 0.040              # cross member width (Y extent = distance between legs)
CROSS_D = 0.040              # cross member depth (X extent)
CROSS_H = 0.035              # cross member height (Z extent)

BOX_W = 0.42                 # cabinet width  (along Y)
BOX_D = 0.34                 # cabinet depth  (along X, front-back)
BOX_H = 0.50                 # cabinet front-wall height (vertical body)
TOP_RISE = 0.085             # extra height the peak adds at the high (back) edge

BOX_BOT_Z = LEG_TOP_Z        # cabinet body bottom
BOX_CTR_Z = BOX_BOT_Z + BOX_H / 2.0
BOX_TOP_Z = BOX_BOT_Z + BOX_H

WALL_T = 0.010               # sheet-metal wall thickness

# front deposit door (pull-down hopper) on the front (+X) face
DOOR_W = 0.34                # door width  (along Y)
DOOR_H = 0.26                # door height (along the front face)
DOOR_T = 0.018               # door panel thickness
DOOR_BOT_OFF = 0.07          # door bottom sits this far above the cabinet bottom
HANDLE_R = 0.012

FRONT_X = BOX_D / 2.0        # front face plane (x)
BACK_X = -BOX_D / 2.0        # back wall plane (x)


def _add_leg_assembly(frame, index: int, y_center: float, *, steel, base_mat) -> None:
    """Emit one leg + foot pad onto the frame part.

    Shared geometry helper called in a for-loop so both legs share the same
    construction and naming convention (leg_0/leg_1, foot_pad_0/foot_pad_1).
    """
    # vertical square-tube leg
    frame.visual(
        Box((LEG_SIDE, LEG_SIDE, LEG_HEIGHT)),
        origin=Origin(xyz=(0.0, y_center, LEG_HEIGHT / 2.0)),
        material=steel,
        name=f"leg_{index}",
    )
    # flat foot pad at the ground
    frame.visual(
        Box((FOOT_PAD_SIDE, FOOT_PAD_SIDE, FOOT_PAD_H)),
        origin=Origin(xyz=(0.0, y_center, FOOT_PAD_H / 2.0)),
        material=base_mat,
        name=f"foot_pad_{index}",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="curbside_collection_box")

    steel = model.material("steel", rgba=(0.42, 0.45, 0.50, 1.0))
    box_blue = model.material("box_blue", rgba=(0.40, 0.46, 0.55, 1.0))
    box_blue_dk = model.material("box_blue_dk", rgba=(0.30, 0.35, 0.43, 1.0))
    door_blue = model.material("door_blue", rgba=(0.46, 0.52, 0.60, 1.0))
    interior = model.material("interior", rgba=(0.07, 0.08, 0.10, 1.0))
    chrome = model.material("chrome", rgba=(0.72, 0.74, 0.78, 1.0))
    base_mat = model.material("base_pad", rgba=(0.33, 0.35, 0.39, 1.0))

    # --------------------------------------------------------- FRAME (root)
    # Twin-leg ground stand: two vertical square-tube legs connected by a
    # horizontal cross member just under the cabinet floor.
    frame = model.part("frame")

    # emit the two legs (and their foot pads) via the shared helper
    for i, y_sign in enumerate((1.0, -1.0)):
        _add_leg_assembly(
            frame, i, y_sign * LEG_SPREAD / 2.0,
            steel=steel, base_mat=base_mat,
        )

    # horizontal cross member tying the two legs together just under the cabinet
    # floor — runs in Y between the leg inner faces
    cross_span = LEG_SPREAD - LEG_SIDE  # clear span between leg inner faces
    frame.visual(
        Box((CROSS_D, cross_span, CROSS_H)),
        origin=Origin(xyz=(0.0, 0.0, LEG_TOP_Z - CROSS_H / 2.0)),
        material=steel,
        name="cross_member",
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

    # ----------------------------------------------------------- ARTICULATIONS
    model.articulation(
        "frame_to_body",
        ArticulationType.FIXED,
        parent=frame,
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

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
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

    # --- frame foot pads sit on the floor at z ~ 0 ----------------------------
    frame_aabb = ctx.part_world_aabb(frame)
    ctx.check(
        "frame foot pads rest on floor (z~0)",
        frame_aabb is not None and abs(frame_aabb[0][2]) < 0.02,
        details=f"frame_min_z={None if frame_aabb is None else frame_aabb[0][2]}",
    )

    # --- cabinet is raised up on the twin-leg frame ----------------------------
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "cabinet body is raised on the leg frame",
        body_aabb is not None and body_aabb[0][2] > 1.0,
        details=f"body_min_z={None if body_aabb is None else body_aabb[0][2]}",
    )

    # --- frame legs are on each side of the cabinet center (Y spread) ----------
    leg0_aabb = ctx.part_element_world_aabb(frame, elem="leg_0")
    leg1_aabb = ctx.part_element_world_aabb(frame, elem="leg_1")
    if leg0_aabb is not None and leg1_aabb is not None:
        leg0_cy = (leg0_aabb[0][1] + leg0_aabb[1][1]) / 2.0
        leg1_cy = (leg1_aabb[0][1] + leg1_aabb[1][1]) / 2.0
        spread = abs(leg0_cy - leg1_cy)
        ctx.check(
            "two legs are spread apart on each side",
            spread > 0.15,
            details=f"leg_spread={spread}",
        )
        ctx.check(
            "legs are symmetric about Y=0",
            abs(leg0_cy + leg1_cy) < 0.02,
            details=f"sum_of_centers={leg0_cy + leg1_cy}",
        )

    # --- cabinet sits ON TOP of the frame cross member -------------------------
    ctx.expect_contact(
        frame, body, elem_a="cross_member", elem_b="floor", contact_tol=0.005,
        name="cross member supports the cabinet floor",
    )

    # --- HOLLOW INTERIOR: the cavity behind the door opening is genuinely deep.
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

    return ctx.report()
