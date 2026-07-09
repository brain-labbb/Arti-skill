from __future__ import annotations

# Vintage blue cast-iron U.S. Mail letter collection box.
#
# An upright pedestal letter box (the old cast-iron "LETTERS" street box) that
# stands directly on the ground. The body is a HOLLOW rectangular cabinet in
# faded blue with rust/copper weathering. A rounded tubular cap sits across the
# top. The front carries, top to bottom: a recessed upper panel below the cap,
# an embossed "LETTERS" header band, a side-hinged swing access door, and a
# lower panel with a small schedule-card holder.
#
# The cabinet is built as thin cast-iron walls (front with the door opening,
# back, two sides, floor, top lid) enclosing an EMPTY interior. When the door
# swings open sideways the opening reveals the real dark hollow chute behind it,
# all the way back to the back wall -- not a solid block.
#
# Coordinate convention:
#   - up is +Z; the box rests on the floor at z = 0.
#   - the FRONT (LETTERS / access door) faces +X; the back is -X.
#   - centerline is y = 0; left/right are symmetric.
#   - the rounded top cap is a half-cylinder whose axis runs along Y.
#
# Root structure: the box body is the root (rests on the floor). The rounded top
# cap is fixed on it. The front access door is hinged along the LEFT vertical
# edge of the front opening and swings open sideways like a cupboard door
# (REVOLUTE about a vertical Z axis).
#
# Articulations:
#   - body_to_cap  : FIXED rounded top cap
#   - body_to_door : REVOLUTE side-hinged swing door (vertical hinge axis)

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
BODY_W = 0.40              # box width  (along Y)
BODY_D = 0.34              # box depth  (along X, front-back)
BODY_H = 0.78              # vertical body height (front wall)
WALL_T = 0.014             # cast-iron wall thickness

CAP_R = 0.165             # rounded top half-cylinder radius (across X)
CAP_LEN = 0.40            # cap length along Y
CAP_Z = BODY_H           # cap base sits on the body top

FRONT_X = BODY_D / 2.0
BACK_X = -BODY_D / 2.0

# front zones (z heights from body bottom)
LOWER_TOP = 0.30          # top of the lower (card-holder) panel
DOOR_BOT = LOWER_TOP + 0.02
DOOR_H = 0.22             # access door height
DOOR_TOP = DOOR_BOT + DOOR_H
HEADER_BOT = DOOR_TOP + 0.005  # LETTERS header band
HEADER_H = 0.075

DOOR_W = 0.30
DOOR_T = 0.020
JAMB_W = (BODY_W - DOOR_W) / 2.0  # front jamb width on each side of opening

# hinge position: left vertical edge of the opening
HINGE_Y = -DOOR_W / 2.0
DOOR_MID_Z = (DOOR_BOT + DOOR_TOP) / 2.0


def _half_cyl_top() -> MeshGeometry:
    # Rounded tubular top: the UPPER half of a cylinder whose axis runs along Y.
    # Flat bottom on z=0 (local), domed top; sits on the body top so only the
    # rounded shell shows. Built directly so it does not bury inside the body.
    n = 36
    hl = CAP_LEN / 2.0
    g = MeshGeometry()
    top_ids: list[int] = []  # +Y end rim
    bot_ids: list[int] = []  # -Y end rim
    # arc from angle 0 (one side) to pi (other side), z = R*sin, x = R*cos
    arc = []
    for i in range(n + 1):
        a = math.pi * i / n
        x = CAP_R * math.cos(a)
        z = CAP_R * math.sin(a)
        arc.append((x, z))
    for (x, z) in arc:
        top_ids.append(g.add_vertex(x, hl, z))
    for (x, z) in arc:
        bot_ids.append(g.add_vertex(x, -hl, z))
    # side shell
    for i in range(n):
        a0, a1 = top_ids[i], top_ids[i + 1]
        b0, b1 = bot_ids[i], bot_ids[i + 1]
        g.add_face(a0, b0, b1)
        g.add_face(a0, b1, a1)
    # end caps (semicircle fans) + flat bottom strip closing each end
    tc = g.add_vertex(0.0, hl, 0.0)
    bc = g.add_vertex(0.0, -hl, 0.0)
    for i in range(n):
        g.add_face(tc, top_ids[i + 1], top_ids[i])
        g.add_face(bc, bot_ids[i], bot_ids[i + 1])
    # flat underside (two big triangles between the two end base edges)
    g.add_face(top_ids[0], bot_ids[0], bot_ids[n])
    g.add_face(top_ids[0], bot_ids[n], top_ids[n])
    return g


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_letters_box")

    blue = model.material("blue", rgba=(0.34, 0.42, 0.52, 1.0))
    blue_dk = model.material("blue_dk", rgba=(0.26, 0.33, 0.42, 1.0))
    rust = model.material("rust", rgba=(0.55, 0.40, 0.30, 1.0))
    rust_dk = model.material("rust_dk", rgba=(0.45, 0.31, 0.22, 1.0))
    cap_blue = model.material("cap_blue", rgba=(0.31, 0.39, 0.49, 1.0))
    interior = model.material("interior", rgba=(0.08, 0.09, 0.11, 1.0))
    card = model.material("card", rgba=(0.82, 0.78, 0.66, 1.0))
    brass = model.material("brass", rgba=(0.62, 0.55, 0.32, 1.0))
    letters_mat = model.material("letters", rgba=(0.20, 0.24, 0.30, 1.0))

    # ----------------------------------------------------------- BOX BODY (root)
    # A hollow upright shell. Thin cast-iron walls enclose an empty interior; the
    # front face leaves an access opening for the side-hinged swing door.
    body = model.part("body")
    inner_w = BODY_W - 2 * WALL_T

    # back wall (-X)
    body.visual(
        Box((WALL_T, BODY_W, BODY_H)),
        origin=Origin(xyz=(BACK_X + WALL_T / 2.0, 0.0, BODY_H / 2.0)),
        material=blue,
        name="back_wall",
    )
    # dark interior liner on the inner (front) face of the back wall -- this is
    # the dark hollow depth seen through the open door.
    body.visual(
        Box((0.004, inner_w, BODY_H - 2 * WALL_T)),
        origin=Origin(xyz=(BACK_X + WALL_T + 0.002, 0.0, BODY_H / 2.0)),
        material=interior,
        name="interior_back_liner",
    )
    # side walls (+/-Y)
    for i, sgn in enumerate((1.0, -1.0)):
        body.visual(
            Box((BODY_D, WALL_T, BODY_H)),
            origin=Origin(xyz=(0.0, sgn * (BODY_W / 2.0 - WALL_T / 2.0), BODY_H / 2.0)),
            material=blue,
            name=f"side_wall_{i}",
        )
    # floor + top lid (top lid is hidden under the rounded cap)
    body.visual(
        Box((BODY_D, BODY_W, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, WALL_T / 2.0)),
        material=blue_dk,
        name="floor",
    )
    body.visual(
        Box((BODY_D, BODY_W, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, BODY_H - WALL_T / 2.0)),
        material=blue_dk,
        name="top_lid",
    )
    # base skirt / foot ring at the floor
    body.visual(
        Box((BODY_D + 0.03, BODY_W + 0.03, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=blue_dk,
        name="base_skirt",
    )
    # back reinforcing ribs (cast-iron look) on the back face
    for i, yy in enumerate((-0.11, 0.0, 0.11)):
        body.visual(
            Box((0.012, 0.03, BODY_H * 0.7)),
            origin=Origin(xyz=(BACK_X - 0.004, yy, BODY_H * 0.45)),
            material=blue_dk,
            name=f"back_rib_{i}",
        )

    # ---- front face panels (leave the access opening over the door region) ----
    # lower front wall (below the door): carries the schedule card + knob.
    body.visual(
        Box((WALL_T, BODY_W, DOOR_BOT)),
        origin=Origin(xyz=(FRONT_X - WALL_T / 2.0, 0.0, DOOR_BOT / 2.0)),
        material=blue,
        name="front_lower",
    )
    # upper front wall (above the door, behind the LETTERS band, under the cap).
    upper_h = BODY_H - DOOR_TOP
    body.visual(
        Box((WALL_T, BODY_W, upper_h)),
        origin=Origin(xyz=(FRONT_X - WALL_T / 2.0, 0.0, DOOR_TOP + upper_h / 2.0)),
        material=blue,
        name="front_upper",
    )
    # front jambs framing the access opening on each side of the door.
    for i, sgn in enumerate((1.0, -1.0)):
        body.visual(
            Box((WALL_T, JAMB_W, DOOR_H)),
            origin=Origin(
                xyz=(
                    FRONT_X - WALL_T / 2.0,
                    sgn * (BODY_W / 2.0 - JAMB_W / 2.0),
                    DOOR_MID_Z,
                ),
            ),
            material=blue,
            name=f"front_jamb_{i}",
        )

    # hinge barrels on the left jamb (vertical pin stubs for the door hinge)
    barrel_r = 0.007
    barrel_len = 0.028
    for i, dz_frac in enumerate((0.30, -0.30)):
        barrel_z = DOOR_MID_Z + dz_frac * DOOR_H
        body.visual(
            Cylinder(radius=barrel_r, length=barrel_len),
            origin=Origin(
                xyz=(FRONT_X + 0.002, HINGE_Y, barrel_z),
                rpy=(0.0, 0.0, 0.0),  # cylinder along +Z by default
            ),
            material=rust_dk,
            name=f"hinge_barrel_{i}",
        )

    # strike plate on the right jamb (where the door latches when closed)
    # Placed on the jamb face, slightly inside the door opening edge so the
    # closed door panel does not overlap it.
    strike_y = BODY_W / 2.0 - JAMB_W / 2.0  # center of right jamb in Y
    body.visual(
        Box((0.008, 0.022, 0.06)),
        origin=Origin(xyz=(FRONT_X + 0.002, strike_y, DOOR_MID_Z)),
        material=brass,
        name="strike_plate",
    )

    # front lower panel with raised frame + schedule-card holder
    body.visual(
        Box((0.012, DOOR_W + 0.03, LOWER_TOP - 0.06)),
        origin=Origin(xyz=(FRONT_X + 0.004, 0.0, (LOWER_TOP - 0.06) / 2.0 + 0.05)),
        material=blue_dk,
        name="lower_panel_frame",
    )
    body.visual(
        Box((0.006, 0.13, 0.10)),
        origin=Origin(xyz=(FRONT_X + 0.010, 0.0, 0.18)),
        material=card,
        name="schedule_card",
    )
    # small pickup-time knob below the card
    body.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(FRONT_X + 0.012, 0.0, 0.085), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="lower_knob",
    )

    # LETTERS header band (raised, embossed look) above the door, on the upper
    # front wall.
    band_back = FRONT_X - 0.004
    band_front = FRONT_X + 0.010
    band_t = band_front - band_back
    body.visual(
        Box((band_t, DOOR_W + 0.03, HEADER_H)),
        origin=Origin(
            xyz=((band_back + band_front) / 2.0, 0.0, HEADER_BOT + HEADER_H / 2.0)
        ),
        material=rust,
        name="letters_band",
    )
    # raised "LETTERS" lettering bar across the band
    body.visual(
        Box((0.006, DOOR_W - 0.02, 0.028)),
        origin=Origin(xyz=(band_front + 0.002, 0.0, HEADER_BOT + HEADER_H / 2.0)),
        material=letters_mat,
        name="letters_text",
    )

    # side "U.S. MAIL" raised plate on +Y side
    body.visual(
        Box((0.10, 0.006, 0.16)),
        origin=Origin(xyz=(0.04, BODY_W / 2.0 + 0.003, 0.50)),
        material=blue_dk,
        name="side_plate",
    )

    # --------------------------------------------------------- ROUNDED TOP CAP
    cap = model.part("cap")
    cap_mesh = _half_cyl_top()
    # Seat the half-cylinder so its flat underside rests flush on the body top
    # lid (coincident faces), held by the fixed joint.
    cap_seat_z = CAP_Z
    cap.visual(
        mesh_from_geometry(cap_mesh, "cap_shell"),
        origin=Origin(xyz=(0.0, 0.0, cap_seat_z)),
        material=cap_blue,
        name="cap_shell",
    )
    # raised finial collar ridge along the crown
    cap.visual(
        Box((0.05, CAP_LEN * 0.9, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, cap_seat_z + CAP_R - 0.005)),
        material=blue_dk,
        name="cap_ridge",
    )

    # ----------------------------------------------------------- ACCESS DOOR
    # Side-hinged swing door: hinged on the LEFT vertical edge of the front
    # opening. The part frame sits at the hinge line; the panel extends along
    # +Y from the hinge to the free (right/latch) edge.
    door = model.part("door")
    # Main door panel: extends from hinge (y=0) to free edge (y=DOOR_W),
    # centered in thickness (x) and height (z) around the part frame.
    door.visual(
        Box((DOOR_T, DOOR_W, DOOR_H)),
        origin=Origin(xyz=(0.0, DOOR_W / 2.0, 0.0)),
        material=rust_dk,
        name="door_panel",
    )
    # raised frame rim around the door outer face
    door.visual(
        Box((0.006, DOOR_W - 0.02, DOOR_H - 0.02)),
        origin=Origin(xyz=(DOOR_T / 2.0 + 0.003, DOOR_W / 2.0, 0.0)),
        material=rust,
        name="door_frame_rim",
    )
    # pull handle on the free (right/latch) edge of the door
    door.visual(
        Cylinder(radius=0.011, length=0.06),
        origin=Origin(
            xyz=(DOOR_T / 2.0 + 0.014, DOOR_W - 0.03, 0.0),
            rpy=(0.0, 0.0, 0.0),  # vertical handle along Z
        ),
        material=brass,
        name="door_handle",
    )
    # hinge straps on the hinge edge (wrap around the barrel pins)
    for i, dz_frac in enumerate((0.30, -0.30)):
        strap_z = dz_frac * DOOR_H
        door.visual(
            Box((DOOR_T + 0.008, 0.035, 0.022)),
            origin=Origin(xyz=(-0.004, 0.018, strap_z)),
            material=rust,
            name=f"door_hinge_strap_{i}",
        )

    # ----------------------------------------------------------- ARTICULATIONS
    model.articulation(
        "body_to_cap",
        ArticulationType.FIXED,
        parent=body,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # Side-hinge swing door. Hinge line along Z at the left jamb of the front
    # opening. Panel extends +Y from the hinge; axis -Z makes positive q swing
    # the free edge outward (+X direction) like a cupboard door opening.
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(FRONT_X + DOOR_T / 2.0, HINGE_Y, DOOR_MID_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=0.0, upper=1.4),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    cap = object_model.get_part("cap")
    door = object_model.get_part("door")
    door_joint = object_model.get_articulation("body_to_door")

    # Dark interior liner is bonded to the inner face of the back wall.
    ctx.allow_overlap(
        body, body,
        elem_a="interior_back_liner", elem_b="back_wall",
        reason="Dark interior liner is bonded to the inner face of the back wall.",
    )

    # Hinge barrel pins penetrate the door hinge edge -- this is the real
    # captured-pin interface between the body hinge barrels and the door.
    ctx.allow_overlap(
        body, door,
        elem_a="hinge_barrel_0", elem_b="door_panel",
        reason="Hinge barrel pin is captured inside the door hinge edge.",
    )
    ctx.allow_overlap(
        body, door,
        elem_a="hinge_barrel_1", elem_b="door_panel",
        reason="Hinge barrel pin is captured inside the door hinge edge.",
    )
    # Hinge straps wrap around the barrel pins — intentional hinge capture.
    ctx.allow_overlap(
        body, door,
        elem_a="hinge_barrel_0", elem_b="door_hinge_strap_0",
        reason="Hinge strap wraps around the barrel pin as part of the hinge mechanism.",
    )
    ctx.allow_overlap(
        body, door,
        elem_a="hinge_barrel_1", elem_b="door_hinge_strap_1",
        reason="Hinge strap wraps around the barrel pin as part of the hinge mechanism.",
    )
    # Proof: the captured hinge pins stay in contact with the door panel.
    ctx.expect_contact(
        body, door,
        elem_a="hinge_barrel_0", elem_b="door_panel",
        contact_tol=0.010,
        name="hinge barrel 0 contacts the door hinge edge",
    )
    ctx.expect_contact(
        body, door,
        elem_a="hinge_barrel_1", elem_b="door_panel",
        contact_tol=0.010,
        name="hinge barrel 1 contacts the door hinge edge",
    )

    # --- door joint is revolute about a vertical (Z) axis -----------------------
    ctx.check(
        "door joint is revolute",
        str(door_joint.articulation_type).upper().endswith("REVOLUTE"),
        details=f"type={door_joint.articulation_type}",
    )
    ax = door_joint.axis
    ctx.check(
        "door hinge axis is vertical Z",
        abs(abs(ax[2]) - 1.0) < 1e-6 and abs(ax[0]) < 1e-6 and abs(ax[1]) < 1e-6,
        details=f"axis={ax}",
    )

    # --- box rests on the floor at z ~ 0 --------------------------------------
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "box base rests on floor (z~0)",
        body_aabb is not None and abs(body_aabb[0][2]) < 0.01,
        details=f"body_min_z={None if body_aabb is None else body_aabb[0][2]}",
    )

    # --- realistic upright proportions ----------------------------------------
    if body_aabb is not None:
        bh = body_aabb[1][2] - body_aabb[0][2]
        bw = body_aabb[1][1] - body_aabb[0][1]
        ctx.check(
            "body height is realistic (~0.78 m body)",
            0.74 < bh < 0.92,
            details=f"height={bh}",
        )
        ctx.check(
            "body width is realistic (~0.4 m)",
            0.34 < bw < 0.50,
            details=f"width={bw}",
        )

    # --- HOLLOW INTERIOR: the chute behind the door is genuinely deep. ---------
    back_aabb = ctx.part_element_world_aabb(body, elem="interior_back_liner")
    if back_aabb is not None:
        depth = FRONT_X - back_aabb[1][0]
        ctx.check(
            "interior chute is deep (open, not a solid/shallow back)",
            depth > 0.25,
            details=f"front_to_back_depth={depth}",
        )

    # --- rounded cap sits on top of the body ----------------------------------
    cap_aabb = ctx.part_world_aabb(cap)
    if cap_aabb is not None and body_aabb is not None:
        ctx.check(
            "rounded cap is at the top of the box",
            cap_aabb[1][2] >= body_aabb[1][2] - 0.001,
            details=f"cap_top={cap_aabb[1][2]}, body_top={body_aabb[1][2]}",
        )
    ctx.expect_overlap(
        cap, body, axes="xy", min_overlap=0.15,
        name="cap seats over the body footprint",
    )

    # --- closed door covers the front opening and is roughly flush ------------
    ctx.expect_overlap(
        door, body, axes="yz", min_overlap=0.10,
        name="closed door covers the access opening",
    )
    ctx.expect_gap(
        door, body, axis="x",
        positive_elem="door_panel", negative_elem="front_lower",
        min_gap=-0.002, max_gap=0.022,
        name="closed door is roughly flush with the front",
    )

    # --- LETTERS header is above the door -------------------------------------
    band_aabb = ctx.part_element_world_aabb(body, elem="letters_band")
    door_aabb = ctx.part_world_aabb(door)
    if band_aabb is not None and door_aabb is not None:
        ctx.check(
            "LETTERS band sits above the access door",
            band_aabb[0][2] >= door_aabb[1][2] - 0.02,
            details=f"band_bot={band_aabb[0][2]}, door_top={door_aabb[1][2]}",
        )

    # --- opened door swings outward (+X) like a cupboard door -----------------
    rest_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: 1.2}):
        open_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door opens outward in +X",
        rest_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][0] > rest_door_aabb[1][0] + 0.04,
        details=f"rest_maxX={None if rest_door_aabb is None else rest_door_aabb[1][0]}, "
        f"open_maxX={None if open_door_aabb is None else open_door_aabb[1][0]}",
    )
    # The free edge (right side at rest) swings leftward toward the hinge side
    # when the door opens — this proves the door pivots about a vertical hinge.
    ctx.check(
        "door free edge swings sideways when open",
        rest_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][1] < rest_door_aabb[1][1] - 0.04,
        details=f"rest_maxY={None if rest_door_aabb is None else rest_door_aabb[1][1]}, "
        f"open_maxY={None if open_door_aabb is None else open_door_aabb[1][1]}",
    )

    # --- hinge origin is at the visible left jamb of the front opening --------
    hinge_origin = door_joint.origin
    ctx.check(
        "door hinge origin is at the left jamb edge",
        hinge_origin is not None and hinge_origin.xyz[1] < -0.05,
        details=f"hinge_y={None if hinge_origin is None else hinge_origin.xyz[1]}",
    )
    ctx.check(
        "door hinge origin is at the front face height",
        hinge_origin is not None
        and DOOR_BOT - 0.01 < hinge_origin.xyz[2] < DOOR_TOP + 0.01,
        details=f"hinge_z={None if hinge_origin is None else hinge_origin.xyz[2]}",
    )

    return ctx.report()
