from __future__ import annotations

# Curbside blue street collection box on a ground pedestal base.
#
# A slanted-top sheet-metal cabinet (the kind used for newspaper / charity
# collection on a sidewalk) standing directly on a broad stepped pedestal foot
# that rests flat on the ground at z = 0. The pedestal is a stepped plinth /
# skirt ring that flares out under the cabinet for stability, so the whole unit
# reads as a free-standing sidewalk collection box.
#
# The cabinet front carries a pull-down hopper deposit door that swings open
# about a BOTTOM hinge so a passer-by can drop items into the chute.
#
# The cabinet is a genuinely HOLLOW sheet-metal shell: back wall, two side walls,
# floor, a roof, and a front face with only a sill below and a header above the
# door opening. Nothing fills the middle, so when the deposit door swings down
# the open mouth reveals the real dark interior cavity all the way back to the
# back wall, not a solid block.
#
# Coordinate convention:
#   - up is +Z; the pedestal base sits on the floor at z = 0.
#   - the cabinet FRONT (the deposit door) faces +X; the back wall is -X.
#   - the cabinet is centered on the pedestal in Y (centerline y = 0).
#   - the peaked / slanted top falls from the back (-X, high) toward the
#     front (+X, low), as in the reference photo.
#
# Root structure: the pedestal is the root. The cabinet body is fixed on top
# of the pedestal. The hopper deposit door hangs from a bottom hinge on the lower
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
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

# ---- key dimensions (meters) -------------------------------------------------
# Pedestal base: stepped plinth that flares out wider than the cabinet
PED_BASE_W = 0.52           # base flange width  (along X, front-back)
PED_BASE_D = 0.44           # base flange depth  (along Y, left-right)
PED_BASE_H = 0.035          # base flange height
PED_BASE_R = 0.020          # base flange corner radius

PED_STEP_W = 0.48           # step width
PED_STEP_D = 0.40           # step depth
PED_STEP_H = 0.035          # step height
PED_STEP_R = 0.015          # step corner radius

PED_SKIRT_W = 0.44          # skirt width (matches or slightly exceeds cabinet)
PED_SKIRT_D = 0.36          # skirt depth
PED_SKIRT_H = 0.48          # skirt height (the tall column section)
PED_SKIRT_R = 0.012         # skirt corner radius

PED_TOTAL_H = PED_BASE_H + PED_STEP_H + PED_SKIRT_H  # 0.55 m

BOX_W = 0.42               # cabinet width  (along Y)
BOX_D = 0.34               # cabinet depth  (along X, front-back)
BOX_H = 0.50               # cabinet front-wall height (vertical body)
TOP_RISE = 0.085           # extra height the peak adds at the high (back) edge

BOX_BOT_Z = PED_TOTAL_H    # cabinet body bottom (sits on pedestal top)
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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="curbside_collection_box")

    steel = model.material("steel", rgba=(0.42, 0.45, 0.50, 1.0))
    box_blue = model.material("box_blue", rgba=(0.40, 0.46, 0.55, 1.0))
    box_blue_dk = model.material("box_blue_dk", rgba=(0.30, 0.35, 0.43, 1.0))
    door_blue = model.material("door_blue", rgba=(0.46, 0.52, 0.60, 1.0))
    interior = model.material("interior", rgba=(0.07, 0.08, 0.10, 1.0))
    chrome = model.material("chrome", rgba=(0.72, 0.74, 0.78, 1.0))
    base_plate = model.material("base_plate", rgba=(0.33, 0.35, 0.39, 1.0))

    # --------------------------------------------------------- PEDESTAL (root)
    # A stepped ground pedestal: broad base flange, intermediate step, and a
    # tall skirt column that rises up to support the cabinet. Each section uses
    # rounded-corner extrusions to read as manufactured sheet-metal plinth.
    pedestal = model.part("pedestal")

    _pedestal_sections = [
        # (w, d, h, r, z_base, material, name)
        (PED_BASE_W, PED_BASE_D, PED_BASE_H, PED_BASE_R, 0.0, base_plate, "pedestal_base"),
        (PED_STEP_W, PED_STEP_D, PED_STEP_H, PED_STEP_R, PED_BASE_H, steel, "pedestal_step"),
        (PED_SKIRT_W, PED_SKIRT_D, PED_SKIRT_H, PED_SKIRT_R, PED_BASE_H + PED_STEP_H, steel, "pedestal_skirt"),
    ]
    for i, (w, d, h, r, z_base, mat, nm) in enumerate(_pedestal_sections):
        profile = rounded_rect_profile(w, d, r, corner_segments=8)
        mesh = mesh_from_geometry(
            ExtrudeGeometry.from_z0(profile, h),
            nm,
        )
        pedestal.visual(
            mesh,
            origin=Origin(xyz=(0.0, 0.0, z_base)),
            material=mat,
            name=nm,
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
        "pedestal_to_body",
        ArticulationType.FIXED,
        parent=pedestal,
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

    pedestal = object_model.get_part("pedestal")
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

    # --- pedestal base sits on the floor at z ~ 0 ------------------------------
    ped_aabb = ctx.part_world_aabb(pedestal)
    ctx.check(
        "pedestal base rests on floor (z~0)",
        ped_aabb is not None and abs(ped_aabb[0][2]) < 0.01,
        details=f"pedestal_min_z={None if ped_aabb is None else ped_aabb[0][2]}",
    )

    # --- cabinet sits ON TOP of the pedestal (ground-level unit, not raised) ---
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "cabinet body sits on the pedestal",
        body_aabb is not None and 0.40 < body_aabb[0][2] < 0.70,
        details=f"body_min_z={None if body_aabb is None else body_aabb[0][2]}",
    )

    # --- cabinet floor contacts pedestal skirt top ----------------------------
    ctx.expect_contact(
        pedestal, body, elem_a="pedestal_skirt", elem_b="floor", contact_tol=0.005,
        name="pedestal supports the cabinet floor",
    )

    # --- pedestal is wider than the cabinet for stability ---------------------
    if ped_aabb is not None and body_aabb is not None:
        ped_w = ped_aabb[1][1] - ped_aabb[0][1]
        body_w = body_aabb[1][1] - body_aabb[0][1]
        ctx.check(
            "pedestal base is wider than cabinet",
            ped_w > body_w,
            details=f"pedestal_width={ped_w}, body_width={body_w}",
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
    # whole-object top should clear ~1.0-1.3 m (ground pedestal unit)
    ctx.check(
        "overall stands ~1.0-1.3 m tall",
        body_aabb is not None and 1.0 < body_aabb[1][2] < 1.4,
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
