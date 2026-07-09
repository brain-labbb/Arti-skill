from __future__ import annotations

"""Classic safety matchbox with a sliding inner tray of matches.

Reference image: picture/Others/Matchbox/001.png

Layout (world frame, Z up, box resting on the ground plane z=0):
- The cream cardboard outer sleeve is the root part, centered at x=y=0,
  0.055 m long (X), 0.037 m wide (Y), 0.014 m tall (Z). It is a true
  hollow shell open at both X ends: top panel, bottom panel and two thin
  side walls, with a dark-brown printed border on the top face and a
  reddish-brown stippled striker strip on each long side.
- The inner tray is an open-top hollow cardboard tray (thin floor + four
  thin walls) that slides lengthwise (+X) through the sleeve on a
  prismatic joint. The displayed rest pose (q = 0) is about half open as
  in the photo; the joint limits are lower = -0.020 m (fully closed,
  flush) and upper = +0.020 m (0.040 m total extension, still captured).
- Sixteen wooden matchsticks (0.045 m long, 0.002 m square) lie in the tray
  parallel to X with dark reddish-brown rounded heads toward the open
  (+X) end. Two loose matches lie splayed on the ground in front (+Y) of
  the box.
"""

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# ---------------------------------------------------------------- constants
SLEEVE_L = 0.055  # X, long axis
SLEEVE_W = 0.037  # Y
SLEEVE_H = 0.014  # Z
T = 0.0008  # cardboard wall thickness

TRAY_L = SLEEVE_L
TRAY_W = 0.0346  # 0.0004 m sliding clearance to each sleeve inner wall
TRAY_H = 0.0114  # floor bottom rides on the sleeve bottom panel
TRAY_Z0 = T  # tray floor bottom sits on top of the sleeve bottom panel

STICK_L = 0.045
STICK_S = 0.002  # square cross-section
HEAD_LX = 0.0045  # ellipsoid head full length (along stick)
HEAD_R = 0.0014  # ellipsoid head half width/height

SLIDE_HALF = 0.020  # rest pose is half open; +/- this is full travel

CREAM = Material(name="sleeve_cardboard", rgba=(0.89, 0.84, 0.71, 1.0))
CREAM_TRAY = Material(name="tray_cardboard", rgba=(0.92, 0.88, 0.76, 1.0))
STRIKER = Material(name="striker_stipple", rgba=(0.66, 0.36, 0.28, 1.0))
PRINT_BROWN = Material(name="print_brown", rgba=(0.24, 0.14, 0.09, 1.0))
WOOD = Material(name="match_wood", rgba=(0.85, 0.72, 0.50, 1.0))
HEAD_BROWN = Material(name="head_brown", rgba=(0.38, 0.13, 0.08, 1.0))


def _add_match_visuals(part, head_mesh) -> None:
    """One matchstick: square wooden stick + rounded head capping the +X tip."""
    part.visual(
        Box((STICK_L, STICK_S, STICK_S)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=WOOD,
        name="stick",
    )
    # Head overlaps the stick tip (the head coats the wood tip in reality)
    # and bulges slightly upward so it clears the tray floor / ground.
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

    # ------------------------------------------------------------- sleeve
    sleeve = model.part("sleeve")
    sleeve.visual(
        Box((SLEEVE_L, SLEEVE_W, T)),
        origin=Origin(xyz=(0.0, 0.0, T / 2.0)),
        material=CREAM,
        name="bottom_panel",
    )
    sleeve.visual(
        Box((SLEEVE_L, SLEEVE_W, T)),
        origin=Origin(xyz=(0.0, 0.0, SLEEVE_H - T / 2.0)),
        material=CREAM,
        name="top_panel",
    )
    for i, sy in enumerate((-1.0, 1.0)):
        # Thin full-height side wall; sleeve stays open at both X ends.
        sleeve.visual(
            Box((SLEEVE_L, T, SLEEVE_H)),
            origin=Origin(xyz=(0.0, sy * (SLEEVE_W / 2.0 - T / 2.0), SLEEVE_H / 2.0)),
            material=CREAM,
            name=f"side_wall_{i}",
        )
        # Reddish-brown stippled striker strip on the outside of each long side,
        # embedded 0.0002 m into the wall so it reads as glued on.
        sleeve.visual(
            Box((0.050, 0.0006, 0.010)),
            origin=Origin(xyz=(0.0, sy * (SLEEVE_W / 2.0 + 0.0001), SLEEVE_H / 2.0)),
            material=STRIKER,
            name=f"striker_{i}",
        )

    # Thin dark-brown printed border on the top face (raised 0.0002 m print
    # proxy strips, embedded 0.0002 m so they stay connected to the panel).
    border_z = SLEEVE_H  # strip center -> spans panel_top-0.0002 .. +0.0002
    border_w = 0.0012
    inset = 0.0035
    long_len = SLEEVE_L - 2.0 * inset
    short_len = SLEEVE_W - 2.0 * inset
    for i, sy in enumerate((-1.0, 1.0)):
        sleeve.visual(
            Box((long_len, border_w, 0.0004)),
            origin=Origin(xyz=(0.0, sy * (SLEEVE_W / 2.0 - inset - border_w / 2.0), border_z)),
            material=PRINT_BROWN,
            name=f"border_long_{i}",
        )
    for i, sx in enumerate((-1.0, 1.0)):
        sleeve.visual(
            Box((border_w, short_len, 0.0004)),
            origin=Origin(xyz=(sx * (SLEEVE_L / 2.0 - inset - border_w / 2.0), 0.0, border_z)),
            material=PRINT_BROWN,
            name=f"border_short_{i}",
        )

    # --------------------------------------------------------------- tray
    tray = model.part("tray")
    tray.visual(
        Box((TRAY_L, TRAY_W, T)),
        origin=Origin(xyz=(0.0, 0.0, TRAY_Z0 + T / 2.0)),
        material=CREAM_TRAY,
        name="tray_floor",
    )
    wall_zc = TRAY_Z0 + TRAY_H / 2.0
    for i, sy in enumerate((-1.0, 1.0)):
        tray.visual(
            Box((TRAY_L, T, TRAY_H)),
            origin=Origin(xyz=(0.0, sy * (TRAY_W / 2.0 - T / 2.0), wall_zc)),
            material=CREAM_TRAY,
            name=f"tray_side_wall_{i}",
        )
    for i, sx in enumerate((-1.0, 1.0)):
        tray.visual(
            Box((T, TRAY_W, TRAY_H)),
            origin=Origin(xyz=(sx * (TRAY_L / 2.0 - T / 2.0), 0.0, wall_zc)),
            material=CREAM_TRAY,
            name=f"tray_end_wall_{i}",
        )

    # Prismatic slide along +X. The joint origin is offset +0.020 m so the
    # displayed rest pose (q = 0) is half open like the photo:
    #   q = -0.020 -> tray fully closed, flush with the sleeve
    #   q = +0.020 -> tray extended 0.040 m, still captured by the sleeve
    model.articulation(
        "sleeve_to_tray",
        ArticulationType.PRISMATIC,
        parent=sleeve,
        child=tray,
        origin=Origin(xyz=(SLIDE_HALF, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=0.15, lower=-SLIDE_HALF, upper=SLIDE_HALF
        ),
    )

    # ------------------------------------------------- matches in the tray
    # Sixteen sticks lying parallel to X on the tray floor, heads toward the
    # open (+X) end. Pitch 0.002 m packs them tightly side by side across
    # the tray interior (tray inner width ~0.033 m, 16 * 0.002 = 0.032 m).
    floor_top = TRAY_Z0 + T
    stick_zc = floor_top + STICK_S / 2.0
    stick_xc = 0.0015  # leaves head clearance to the +X tray end wall
    n_matches = 16
    pitch = 0.002
    for i in range(n_matches):
        yc = -pitch * (n_matches - 1) / 2.0 + pitch * i
        match = model.part(f"match_{i}")
        _add_match_visuals(match, head_mesh)
        model.articulation(
            f"tray_to_match_{i}",
            ArticulationType.FIXED,
            parent=tray,
            child=match,
            origin=Origin(xyz=(stick_xc, yc, stick_zc)),
        )

    # ------------------------------------------- loose matches on the ground
    # Two matches lying flat in front (+Y) of the box, slightly splayed, heads
    # pointing toward -X like the photo (their local +X head end is yawed
    # around by ~pi).
    ground_zc = STICK_S / 2.0
    placements = (
        ((0.000, 0.030), math.pi + 0.10),
        ((0.010, 0.042), math.pi + 0.35),
    )
    for i, ((cx, cy), yaw) in enumerate(placements):
        gm = model.part(f"ground_match_{i}")
        _add_match_visuals(gm, head_mesh)
        model.articulation(
            f"sleeve_to_ground_match_{i}",
            ArticulationType.FIXED,
            parent=sleeve,
            child=gm,
            origin=Origin(xyz=(cx, cy, ground_zc), rpy=(0.0, 0.0, yaw)),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    sleeve = object_model.get_part("sleeve")
    tray = object_model.get_part("tray")
    slide = object_model.get_articulation("sleeve_to_tray")
    matches = [object_model.get_part(f"match_{i}") for i in range(16)]
    ground_matches = [object_model.get_part(f"ground_match_{i}") for i in range(2)]

    # The two loose matches intentionally lie alone on the ground in front of
    # the box, as requested by the prompt.
    for gm in ground_matches:
        ctx.allow_isolated_part(
            gm, reason="Loose match placed on the ground in front of the box per the prompt."
        )

    # ---- sleeve hero geometry -------------------------------------------
    aabb = ctx.part_world_aabb(sleeve)
    ok_dims = False
    if aabb is not None:
        (x0, y0, z0), (x1, y1, z1) = aabb
        ok_dims = (
            abs((x1 - x0) - SLEEVE_L) < 0.003
            and abs((y1 - y0) - SLEEVE_W) < 0.004
            and abs((z1 - z0) - SLEEVE_H) < 0.002
            and abs(z0) < 0.0005
        )
    ctx.check(
        "sleeve matches the prompt dimensions and rests on the ground",
        ok_dims,
        details=f"sleeve aabb={aabb}",
    )

    # Striker strips sit proud on the outside of both long side walls.
    for i, sy in enumerate((-1.0, 1.0)):
        sb = ctx.part_element_world_aabb(sleeve, elem=f"striker_{i}")
        ok = sb is not None and (
            (sy < 0 and sb[0][1] < -(SLEEVE_W / 2.0 + 0.0001))
            or (sy > 0 and sb[1][1] > (SLEEVE_W / 2.0 + 0.0001))
        )
        ctx.check(
            f"striker strip {i} stands proud of its long side wall",
            ok,
            details=f"striker_{i} aabb={sb}",
        )

    # Printed border strips sit on the top face, slightly proud of the panel.
    bb = ctx.part_element_world_aabb(sleeve, elem="border_long_0")
    ctx.check(
        "dark-brown border print lies on the sleeve top face",
        bb is not None and bb[1][2] > SLEEVE_H and bb[0][2] < SLEEVE_H,
        details=f"border_long_0 aabb={bb}",
    )

    # Sleeve is a hollow shell: its interior between the panels and walls is
    # open, so the tray's projected cross-section fits fully inside it.
    ctx.expect_within(
        tray,
        sleeve,
        axes="yz",
        margin=0.0,
        name="tray cross-section stays captured inside the hollow sleeve",
    )

    # ---- prismatic slide ------------------------------------------------
    ctx.check(
        "tray slides on a prismatic joint along the long axis",
        str(slide.articulation_type).upper().endswith("PRISMATIC")
        and abs(slide.axis[0]) == 1.0
        and slide.axis[1] == 0.0
        and slide.axis[2] == 0.0,
        details=f"type={slide.articulation_type} axis={slide.axis}",
    )

    # Rest pose (q=0) is about half open as in the photo: the tray protrudes
    # roughly 0.02 m beyond the sleeve's +X opening but stays well inserted.
    rest_aabb = ctx.part_world_aabb(tray)
    ctx.check(
        "rest pose shows the tray about half open",
        rest_aabb is not None
        and 0.012 < (rest_aabb[1][0] - SLEEVE_L / 2.0) < 0.028,
        details=f"tray aabb at rest={rest_aabb}",
    )
    ctx.expect_overlap(
        tray,
        sleeve,
        axes="x",
        min_overlap=0.030,
        name="half-open tray retains insertion in the sleeve",
    )

    # Fully closed: tray flush inside the sleeve footprint.
    with ctx.pose({slide: -SLIDE_HALF}):
        ctx.expect_within(
            tray,
            sleeve,
            axes="x",
            margin=0.0006,
            name="closed tray sits flush inside the sleeve",
        )
        closed_pos = ctx.part_world_position(tray)

    # Fully extended: about 0.04 m of travel, still captured by the sleeve.
    with ctx.pose({slide: SLIDE_HALF}):
        ctx.expect_overlap(
            tray,
            sleeve,
            axes="x",
            min_overlap=0.012,
            name="fully extended tray is still captured by the sleeve",
        )
        ctx.expect_within(
            tray,
            sleeve,
            axes="yz",
            margin=0.0,
            name="extended tray still rides inside the sleeve cross-section",
        )
        open_pos = ctx.part_world_position(tray)

    ctx.check(
        "tray travel from flush to extended is about 0.04 m along +X",
        closed_pos is not None
        and open_pos is not None
        and abs((open_pos[0] - closed_pos[0]) - 2.0 * SLIDE_HALF) < 0.002,
        details=f"closed={closed_pos} open={open_pos}",
    )

    # ---- tray shell is hollow -------------------------------------------
    floor_b = ctx.part_element_world_aabb(tray, elem="tray_floor")
    wall_b = ctx.part_element_world_aabb(tray, elem="tray_side_wall_0")
    ctx.check(
        "tray is a thin-walled open-top shell, not a solid block",
        floor_b is not None
        and wall_b is not None
        and (floor_b[1][2] - floor_b[0][2]) < 0.0015
        and (wall_b[1][1] - wall_b[0][1]) < 0.0015
        and wall_b[1][2] > floor_b[1][2] + 0.008,
        details=f"floor={floor_b} wall={wall_b}",
    )
    # Tray floor rides directly on the sleeve bottom panel.
    ctx.expect_contact(
        tray,
        sleeve,
        elem_a="tray_floor",
        elem_b="bottom_panel",
        contact_tol=1e-5,
        name="tray floor rides on the sleeve bottom panel",
    )

    # ---- matches in the tray --------------------------------------------
    ctx.check("sixteen matchsticks fill the tray", len(matches) == 16)
    for i, m in enumerate(matches):
        ctx.expect_within(
            m,
            tray,
            axes="xy",
            margin=0.0,
            name=f"match_{i} lies inside the tray cavity",
        )
    # Sticks rest on the tray floor (spot-check both ends of the row).
    for i in (0, 15):
        ctx.expect_contact(
            matches[i],
            tray,
            elem_a="stick",
            elem_b="tray_floor",
            contact_tol=1e-5,
            name=f"match_{i} stick rests on the tray floor",
        )

    # Heads point toward the open (+X) end of the tray and clear the floor.
    head_b = ctx.part_element_world_aabb(matches[0], elem="head")
    stick_b = ctx.part_element_world_aabb(matches[0], elem="stick")
    ctx.check(
        "match heads sit at the +X (open) end of the sticks",
        head_b is not None
        and stick_b is not None
        and (head_b[0][0] + head_b[1][0]) / 2.0
        > (stick_b[0][0] + stick_b[1][0]) / 2.0 + 0.015,
        details=f"head={head_b} stick={stick_b}",
    )
    ctx.expect_gap(
        matches[0],
        tray,
        axis="z",
        positive_elem="head",
        negative_elem="tray_floor",
        min_gap=0.0,
        max_gap=0.001,
        name="match head bulb clears the tray floor",
    )

    # ---- loose matches on the ground -------------------------------------
    for i, gm in enumerate(ground_matches):
        gb = ctx.part_world_aabb(gm)
        ctx.check(
            f"ground_match_{i} lies flat on the ground",
            gb is not None and abs(gb[0][2]) < 0.0005 and gb[1][2] < 0.006,
            details=f"aabb={gb}",
        )
        ctx.expect_gap(
            gm,
            sleeve,
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
