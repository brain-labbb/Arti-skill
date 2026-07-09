from __future__ import annotations

"""Flip-top matchbox with a hinged lid.

Layout (world frame, Z up, box resting on the ground plane z=0):
- The cream cardboard body is the root part: an open-top tray (thin floor +
  four thin upright walls), 0.055 m long (X), 0.037 m wide (Y), 0.014 m tall
  (Z). Reddish-brown stippled striker strips cover each long side. A thin
  cylindrical cardboard hinge knuckle (fold spine) runs along the top of the
  rear (-Y) long wall.
- A flat rectangular lid panel covers the top and attaches to the rear wall
  top edge by a revolute hinge whose axis runs along the box long axis (X).
  A dark-brown printed rectangular border decorates the lid top face. The
  lid is posed about 50° open at rest so the matches inside are visible.
- Ten wooden matchsticks (0.045 m long, 0.002 m square) lie in the body
  parallel to X with dark reddish-brown rounded heads toward one end.
- Two loose matches lie splayed on the ground in front (+Y) of the box.
"""

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# ---------------------------------------------------------------- constants
BODY_L = 0.055  # X, long axis
BODY_W = 0.037  # Y
BODY_H = 0.014  # Z
T = 0.0008  # cardboard wall thickness
T_LID = 0.0008  # lid panel thickness

STICK_L = 0.045
STICK_S = 0.002  # square cross-section
HEAD_LX = 0.0045  # ellipsoid head full length
HEAD_R = 0.0014  # ellipsoid head half width/height

LID_REST_ANGLE = math.radians(50)  # rest pose: lid ~50° open

KNUCKLE_R = 0.0012  # hinge knuckle radius
KNUCKLE_L = BODY_L * 0.70  # knuckle length along X

CREAM = Material(name="body_cardboard", rgba=(0.89, 0.84, 0.71, 1.0))
CREAM_LID = Material(name="lid_cardboard", rgba=(0.92, 0.88, 0.76, 1.0))
STRIKER = Material(name="striker_stipple", rgba=(0.66, 0.36, 0.28, 1.0))
PRINT_BROWN = Material(name="print_brown", rgba=(0.24, 0.14, 0.09, 1.0))
WOOD = Material(name="match_wood", rgba=(0.85, 0.72, 0.50, 1.0))
HEAD_BROWN = Material(name="head_brown", rgba=(0.38, 0.13, 0.08, 1.0))
KNUCKLE_MAT = Material(name="knuckle_cardboard", rgba=(0.82, 0.76, 0.62, 1.0))


def _add_match_visuals(part, head_mesh) -> None:
    """One matchstick: square wooden stick + rounded head capping the +X tip."""
    part.visual(
        Box((STICK_L, STICK_S, STICK_S)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=WOOD,
        name="stick",
    )
    # Head coats the wood tip; bulges slightly upward so it clears the floor.
    part.visual(
        head_mesh,
        origin=Origin(xyz=(STICK_L / 2.0 - 0.0002, 0.0, 0.0006)),
        material=HEAD_BROWN,
        name="head",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="safety_matchbox")

    # Shared ellipsoid mesh for every match head (long axis along X).
    head_geom = SphereGeometry(1.0, width_segments=24, height_segments=16)
    head_geom.scale(HEAD_LX / 2.0, HEAD_R, HEAD_R)
    head_mesh = mesh_from_geometry(head_geom, "match_head")

    # ------------------------------------------------------------- body
    # Open-top cardboard tray: thin floor + four thin upright walls.
    body = model.part("body")

    # Floor
    body.visual(
        Box((BODY_L, BODY_W, T)),
        origin=Origin(xyz=(0.0, 0.0, T / 2.0)),
        material=CREAM,
        name="floor",
    )

    # Long side walls (full length, full height)
    wall_zc = BODY_H / 2.0
    for i, sy in enumerate((-1.0, 1.0)):
        body.visual(
            Box((BODY_L, T, BODY_H)),
            origin=Origin(xyz=(0.0, sy * (BODY_W / 2.0 - T / 2.0), wall_zc)),
            material=CREAM,
            name=f"side_wall_{i}",
        )
        # Reddish-brown stippled striker strip on the outside of each long side,
        # embedded 0.0002 m into the wall so it reads as glued on.
        body.visual(
            Box((0.050, 0.0006, 0.010)),
            origin=Origin(
                xyz=(0.0, sy * (BODY_W / 2.0 + 0.0001), BODY_H / 2.0)
            ),
            material=STRIKER,
            name=f"striker_{i}",
        )

    # Short end walls (fit between side walls to avoid corner overlap)
    end_wall_w = BODY_W - 2.0 * T
    for i, sx in enumerate((-1.0, 1.0)):
        body.visual(
            Box((T, end_wall_w, BODY_H)),
            origin=Origin(xyz=(sx * (BODY_L / 2.0 - T / 2.0), 0.0, wall_zc)),
            material=CREAM,
            name=f"end_wall_{i}",
        )

    # Hinge knuckle: thin cylindrical cardboard fold spine along the top of
    # the rear (-Y) long wall, aligned with the X axis.
    body.visual(
        Cylinder(radius=KNUCKLE_R, length=KNUCKLE_L),
        origin=Origin(
            xyz=(0.0, -(BODY_W / 2.0), BODY_H),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=KNUCKLE_MAT,
        name="hinge_knuckle",
    )

    # --------------------------------------------------------------- lid
    # Flat rectangular lid panel; hinge edge is at the part origin (y=0),
    # panel extends toward local +Y (front of box when closed).
    lid = model.part("lid")
    lid.visual(
        Box((BODY_L, BODY_W, T_LID)),
        origin=Origin(xyz=(0.0, BODY_W / 2.0, T_LID / 2.0)),
        material=CREAM_LID,
        name="lid_panel",
    )

    # Dark-brown printed rectangular border on the lid top face.
    # In lid frame the top surface is at z = T_LID; border strips sit slightly
    # proud (center at z = T_LID + 0.0002). The lid panel is centered at
    # y = BODY_W/2 (hinge edge at y=0), so the border frame must be centered
    # there too -- not on the hinge axis.
    border_z = T_LID + 0.0002
    border_w = 0.0012
    inset = 0.0035
    long_len = BODY_L - 2.0 * inset
    short_len = BODY_W - 2.0 * inset
    panel_yc = BODY_W / 2.0  # lid panel center in lid frame
    for i, sy in enumerate((-1.0, 1.0)):
        lid.visual(
            Box((long_len, border_w, 0.0004)),
            origin=Origin(
                xyz=(0.0, panel_yc + sy * (BODY_W / 2.0 - inset - border_w / 2.0), border_z)
            ),
            material=PRINT_BROWN,
            name=f"border_long_{i}",
        )
    for i, sx in enumerate((-1.0, 1.0)):
        lid.visual(
            Box((border_w, short_len, 0.0004)),
            origin=Origin(
                xyz=(sx * (BODY_L / 2.0 - inset - border_w / 2.0), panel_yc, border_z)
            ),
            material=PRINT_BROWN,
            name=f"border_short_{i}",
        )

    # --------------------------------------------- body-to-lid hinge
    # Revolute hinge at the rear (-Y) wall top outer edge.
    # The articulation origin rpy offsets the child frame so q=0 displays
    # the lid ~50° open. axis=(1,0,0): positive q opens the lid further.
    #   q = -LID_REST_ANGLE  →  lid fully closed (horizontal)
    #   q = 0                →  lid ~50° open (rest pose)
    #   q = upper            →  lid ~120° from horizontal (fully open)
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(
            xyz=(0.0, -(BODY_W / 2.0), BODY_H),
            rpy=(LID_REST_ANGLE, 0.0, 0.0),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=-LID_REST_ANGLE,
            upper=math.radians(70),
        ),
    )

    # ----------------------------------------- matches inside the body
    # Ten sticks lying parallel to X on the body floor, heads toward +X.
    floor_top = T
    stick_zc = floor_top + STICK_S / 2.0
    stick_xc = 0.0015  # small offset for head clearance to end wall
    pitch = 0.0031
    for i in range(10):
        yc = -pitch * 4.5 + pitch * i
        match = model.part(f"match_{i}")
        _add_match_visuals(match, head_mesh)
        model.articulation(
            f"body_to_match_{i}",
            ArticulationType.FIXED,
            parent=body,
            child=match,
            origin=Origin(xyz=(stick_xc, yc, stick_zc)),
        )

    # ------------------------------------- loose matches on the ground
    # Two matches lying flat in front (+Y) of the box, slightly splayed.
    ground_zc = STICK_S / 2.0
    placements = (
        ((0.000, 0.030), math.pi + 0.10),
        ((0.010, 0.042), math.pi + 0.35),
    )
    for i, ((cx, cy), yaw) in enumerate(placements):
        gm = model.part(f"ground_match_{i}")
        _add_match_visuals(gm, head_mesh)
        model.articulation(
            f"body_to_ground_match_{i}",
            ArticulationType.FIXED,
            parent=body,
            child=gm,
            origin=Origin(xyz=(cx, cy, ground_zc), rpy=(0.0, 0.0, yaw)),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")
    matches = [object_model.get_part(f"match_{i}") for i in range(10)]
    ground_matches = [object_model.get_part(f"ground_match_{i}") for i in range(2)]

    # The two loose matches intentionally lie alone on the ground in front of
    # the box, as requested by the prompt.
    for gm in ground_matches:
        ctx.allow_isolated_part(
            gm,
            reason="Loose match placed on the ground in front of the box per the prompt.",
        )

    # The lid panel hinge edge wraps around the cylindrical knuckle at the
    # rear wall top edge; small local overlap is intentional.
    ctx.allow_overlap(
        body,
        lid,
        elem_a="hinge_knuckle",
        elem_b="lid_panel",
        reason=(
            "The lid panel edge wraps around the cylindrical cardboard knuckle "
            "at the hinge line, producing small local interpenetration that "
            "represents the fold spine of a cardboard flip-top box."
        ),
    )

    # ---- body hero geometry ---------------------------------------------
    aabb = ctx.part_world_aabb(body)
    ok_dims = False
    if aabb is not None:
        (x0, y0, z0), (x1, y1, z1) = aabb
        ok_dims = (
            abs((x1 - x0) - BODY_L) < 0.003
            and abs((y1 - y0) - BODY_W) < 0.004
            and abs(z0) < 0.0005
        )
    ctx.check(
        "body matches the prompt dimensions and rests on the ground",
        ok_dims,
        details=f"body aabb={aabb}",
    )

    # Striker strips sit proud on the outside of both long side walls.
    for i, sy in enumerate((-1.0, 1.0)):
        sb = ctx.part_element_world_aabb(body, elem=f"striker_{i}")
        ok = sb is not None and (
            (sy < 0 and sb[0][1] < -(BODY_W / 2.0 + 0.0001))
            or (sy > 0 and sb[1][1] > (BODY_W / 2.0 + 0.0001))
        )
        ctx.check(
            f"striker strip {i} stands proud of its long side wall",
            ok,
            details=f"striker_{i} aabb={sb}",
        )

    # ---- body is an open-top hollow shell, not a solid block ------------
    floor_b = ctx.part_element_world_aabb(body, elem="floor")
    wall_b = ctx.part_element_world_aabb(body, elem="side_wall_0")
    ctx.check(
        "body is a thin-walled open-top shell, not a solid block",
        floor_b is not None
        and wall_b is not None
        and (floor_b[1][2] - floor_b[0][2]) < 0.0015
        and (wall_b[1][1] - wall_b[0][1]) < 0.0015
        and wall_b[1][2] > floor_b[1][2] + 0.008,
        details=f"floor={floor_b} wall={wall_b}",
    )

    # Hinge knuckle is a visible cylinder on the body rear wall top edge.
    knuckle_b = ctx.part_element_world_aabb(body, elem="hinge_knuckle")
    ctx.check(
        "hinge knuckle is visible at the rear wall top edge",
        knuckle_b is not None
        and knuckle_b[1][2] > BODY_H - 0.001
        and knuckle_b[0][1] < -(BODY_W / 2.0 - 0.003),
        details=f"hinge_knuckle aabb={knuckle_b}",
    )

    # ---- lid with dark-brown border -------------------------------------
    lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid panel exists and has reasonable size",
        lid_aabb is not None
        and (lid_aabb[1][0] - lid_aabb[0][0]) > BODY_L * 0.8
        and (lid_aabb[1][1] - lid_aabb[0][1]) > BODY_W * 0.5,
        details=f"lid aabb={lid_aabb}",
    )

    # The printed border frame must sit ON the lid panel, not float off it.
    # Every border strip's world AABB must lie inside the lid panel footprint
    # (small tolerance for the proud print thickness and panel edge inset).
    panel_b = ctx.part_element_world_aabb(lid, elem="lid_panel")
    for nm in ("border_long_0", "border_long_1", "border_short_0", "border_short_1"):
        strip_b = ctx.part_element_world_aabb(lid, elem=nm)
        on_panel = (
            panel_b is not None
            and strip_b is not None
            and strip_b[0][0] > panel_b[0][0] - 0.001
            and strip_b[1][0] < panel_b[1][0] + 0.001
            and strip_b[0][1] > panel_b[0][1] - 0.001
            and strip_b[1][1] < panel_b[1][1] + 0.001
        )
        ctx.check(
            f"{nm} lies on the lid panel (not floating off it)",
            on_panel,
            details=f"{nm}={strip_b} panel={panel_b}",
        )

    # ---- revolute hinge mechanism ---------------------------------------
    ctx.check(
        "lid hinges on a revolute joint along the long axis",
        str(hinge.articulation_type).upper().endswith("REVOLUTE")
        and abs(hinge.axis[0]) > 0.9
        and abs(hinge.axis[1]) < 0.1
        and abs(hinge.axis[2]) < 0.1,
        details=f"type={hinge.articulation_type} axis={hinge.axis}",
    )

    # Hinge origin sits at the rear wall top edge.
    hinge_origin = hinge.origin
    ctx.check(
        "hinge origin is at the rear wall top edge",
        hinge_origin.xyz[1] < -(BODY_W / 2.0 - 0.005)
        and abs(hinge_origin.xyz[2] - BODY_H) < 0.003,
        details=f"hinge origin={hinge_origin.xyz}",
    )

    # Rest pose (q=0): lid is ~50° open. The lid panel top edge rises well
    # above the body wall height, proving the tilt.
    rest_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "rest pose shows the lid tilted open (~50°)",
        rest_aabb is not None and rest_aabb[1][2] > BODY_H + 0.015,
        details=f"lid aabb at rest={rest_aabb}",
    )

    # Closed pose: q = lower limit → lid horizontal, covering body top.
    hinge_lower = hinge.motion_limits.lower
    hinge_upper = hinge.motion_limits.upper
    with ctx.pose({hinge: hinge_lower}):
        closed_aabb = ctx.part_world_aabb(lid)
        ctx.check(
            "closed lid covers the body opening",
            closed_aabb is not None
            and abs(closed_aabb[0][2] - BODY_H) < 0.003
            and closed_aabb[1][2] < BODY_H + 0.003,
            details=f"closed lid aabb={closed_aabb}",
        )
        # Lid XY footprint at closed overlaps the body XY footprint.
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.025,
            name="closed lid overlaps body footprint in XY",
        )

    # Fully open pose: lid swings past vertical; the front edge tilts behind
    # the hinge line. Check that the lid AABB changes meaningfully.
    with ctx.pose({hinge: hinge_upper}):
        open_aabb = ctx.part_world_aabb(lid)
        ctx.check(
            "fully open lid swings past vertical (position changes from rest)",
            open_aabb is not None
            and rest_aabb is not None
            and (
                abs(open_aabb[0][1] - rest_aabb[0][1]) > 0.005
                or abs(open_aabb[1][2] - rest_aabb[1][2]) > 0.002
            ),
            details=f"open lid aabb={open_aabb}, rest lid aabb={rest_aabb}",
        )

    # ---- matches inside the body ----------------------------------------
    ctx.check("ten matchsticks fill the body", len(matches) == 10)
    for i, m in enumerate(matches):
        ctx.expect_within(
            m,
            body,
            axes="xy",
            margin=0.0,
            name=f"match_{i} lies inside the body cavity",
        )
    # Sticks rest on the body floor (spot-check both ends of the row).
    for i in (0, 9):
        ctx.expect_contact(
            matches[i],
            body,
            elem_a="stick",
            elem_b="floor",
            contact_tol=1e-5,
            name=f"match_{i} stick rests on the body floor",
        )

    # Heads point toward the +X end of the sticks and clear the floor.
    head_b = ctx.part_element_world_aabb(matches[0], elem="head")
    stick_b = ctx.part_element_world_aabb(matches[0], elem="stick")
    ctx.check(
        "match heads sit at the +X end of the sticks",
        head_b is not None
        and stick_b is not None
        and (head_b[0][0] + head_b[1][0]) / 2.0
        > (stick_b[0][0] + stick_b[1][0]) / 2.0 + 0.015,
        details=f"head={head_b} stick={stick_b}",
    )
    ctx.expect_gap(
        matches[0],
        body,
        axis="z",
        positive_elem="head",
        negative_elem="floor",
        min_gap=0.0,
        max_gap=0.001,
        name="match head bulb clears the body floor",
    )

    # ---- loose matches on the ground ------------------------------------
    for i, gm in enumerate(ground_matches):
        gb = ctx.part_world_aabb(gm)
        ctx.check(
            f"ground_match_{i} lies flat on the ground",
            gb is not None and abs(gb[0][2]) < 0.0005 and gb[1][2] < 0.006,
            details=f"aabb={gb}",
        )
        ctx.expect_gap(
            gm,
            body,
            axis="y",
            min_gap=0.003,
            name=f"ground_match_{i} lies clear in front (+Y) of the box",
        )
    # The two loose matches are splayed, not overlapping each other.
    ctx.expect_origin_distance(
        ground_matches[0],
        ground_matches[1],
        axes="xy",
        min_dist=0.008,
        name="loose matches are separated on the ground",
    )

    return ctx.report()


object_model = build_object_model()
