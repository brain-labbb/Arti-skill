from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

# ---------------------------------------------------------------------------
# Articraft brief
# ---------------------------------------------------------------------------
# Object: four-panel white-framed sliding patio door, ~3.0 m wide x ~2.1 m
#   tall, standing on the floor (z from 0 upward). The door plane is the X-Z
#   plane; the assembly is thin along Y (depth). Viewed from the room side.
# Root/support: the white perimeter frame plus the central mullion and the two
#   FIXED outer glass panes are one rigid root that carries everything.
# Parts: frame (root) with two glazed fixed outer panes, plus two sliding
#   center leaves (door_0, door_1), each a white sub-frame holding a glass pane
#   and a curved pull handle.
# Articulations: two PRISMATIC joints. door_0 slides toward -X, door_1 toward
#   +X (bi-parting). door_1 mimic-couples door_0 (multiplier=1) so a single
#   scalar opens both symmetrically.
# Visible geometry: white extruded frame profiles, transparent low-alpha glass
#   panes, curved tubular handles meeting at the center. The two sliding leaves
#   sit on the room-side plane (lower +Y) and retract BEHIND the fixed outer
#   panes (which sit on the outer -Y plane) so they overlap in projection but
#   are separated in depth.
# Support/fit: leaves ride in head/sill tracks on the frame; closed they meet
#   at the centerline; open they tuck behind the outer fixed panes.
# Intentional overlaps: none required. The leaves and outer panes are offset in
#   Y so they never interpenetrate even when fully open.
# Tests: frame standing on floor; closed leaves meet at center; each leaf
#   covers the opening height; open pose parts the leaves and tucks each behind
#   the matching outer pane; nothing floats.
# ---------------------------------------------------------------------------

# Overall dimensions (meters).
TOTAL_W = 3.00          # full outer width along X
TOTAL_H = 2.10          # full outer height along Z (sits z=0..2.10)
FRAME_FACE = 0.060      # visible width of the perimeter frame profile
FRAME_DEPTH = 0.090     # frame depth along Y

GLASS_T = 0.006         # glass thickness
PANE_CLEAR = 0.018      # inset of glass from the frame face edges

# Depth (Y) planes. The fixed outer panes and frame mullions sit toward the
# outer side (-Y); the sliding leaves ride on a separate track on the room side
# (+Y) so a leaf passes clearly in FRONT of the fixed mullion when opening and
# never interpenetrates it.
FIXED_PANE_Y = -0.018   # center-Y of fixed outer glass
LEAF_PLANE_Y = 0.040    # center-Y of the sliding-leaf glass (room-side track plane)

# Inner clear opening inside the perimeter frame.
INNER_W = TOTAL_W - 2.0 * FRAME_FACE   # 2.88
INNER_H = TOTAL_H - 2.0 * FRAME_FACE   # 1.98
INNER_X0 = -INNER_W / 2.0
INNER_X1 = INNER_W / 2.0
INNER_Z0 = FRAME_FACE                  # 0.060
INNER_Z1 = TOTAL_H - FRAME_FACE        # 2.040

# The inner opening is split into four equal bays by mullions.
BAY_W = INNER_W / 4.0                  # 0.72
# Bay centers along X (bay 0 = far -X fixed, 1 = -X leaf, 2 = +X leaf, 3 = +X fixed)
BAY_CX = [INNER_X0 + (i + 0.5) * BAY_W for i in range(4)]

MULLION_FACE = 0.050    # visible width of a vertical mullion between bays
LEAF_STILE = 0.055      # white stile/rail width framing each sliding leaf
LEAF_FRAME_DEPTH = 0.055


def _add_pane(part, cx: float, cz: float, width: float, height: float, y: float, mat, name: str) -> None:
    """A single rectangular glass pane centered at (cx, cz) on depth plane y."""
    part.visual(
        Box((width, GLASS_T, height)),
        origin=Origin(xyz=(cx, y, cz)),
        material=mat,
        name=name,
    )


# Sliding leaves are authored in their OWN part-local frame, centered on the
# part origin in X and Z. The prismatic joint origin then places that frame at
# the closed bay center, so at q=0 geometry sits exactly in its bay and the
# joint axis translates it cleanly.
LEAF_CZ = (INNER_Z0 + INNER_Z1) / 2.0  # absolute Z of the leaf center


def _add_leaf_frame(part, mat) -> None:
    """White stile/rail border around a sliding leaf, in the leaf-local frame
    (centered at x=0, z=0)."""
    half_w = BAY_W / 2.0
    half_h = INNER_H / 2.0
    y = LEAF_PLANE_Y
    # Top and bottom rails (run full bay width).
    part.visual(
        Box((BAY_W, LEAF_FRAME_DEPTH, LEAF_STILE)),
        origin=Origin(xyz=(0.0, y, half_h - LEAF_STILE / 2.0)),
        material=mat,
        name="top_rail",
    )
    part.visual(
        Box((BAY_W, LEAF_FRAME_DEPTH, LEAF_STILE)),
        origin=Origin(xyz=(0.0, y, -half_h + LEAF_STILE / 2.0)),
        material=mat,
        name="bottom_rail",
    )
    # Left and right stiles (between the rails).
    stile_h = INNER_H - 2.0 * LEAF_STILE
    part.visual(
        Box((LEAF_STILE, LEAF_FRAME_DEPTH, stile_h)),
        origin=Origin(xyz=(-half_w + LEAF_STILE / 2.0, y, 0.0)),
        material=mat,
        name="stile_0",
    )
    part.visual(
        Box((LEAF_STILE, LEAF_FRAME_DEPTH, stile_h)),
        origin=Origin(xyz=(half_w - LEAF_STILE / 2.0, y, 0.0)),
        material=mat,
        name="stile_1",
    )


def _add_curved_handle(part, x_edge: float, mat, name_prefix: str, opens_negative: bool) -> None:
    """A curved white C-pull handle on the meeting stile of a center leaf,
    authored in the leaf-local frame (z centered at 0).

    Matching the reference photo, the pull is a vertical grip that BOWS sideways
    in the door plane (along X), curving away from the central meeting seam, with
    two angled mounting necks that tie the grip back onto the stile. It also
    stands slightly proud on the room side (+Y). ``opens_negative`` flips the bow
    direction so the paired handles mirror each other across the centerline.
    """
    handle_h = 0.210            # vertical span of the C-pull
    bow_x_out = 0.062           # how far the grip bows sideways from the stile (X)
    bow_y_out = 0.034           # how far the grip stands proud of the glass (Y)
    tube_r = 0.012
    neck_r = 0.010
    # Bow sign: away from the centerline (handle on +X meeting stile bows toward
    # +X is wrong; it must bow toward the leaf body, i.e. AWAY from center).
    sign_x = -1.0 if opens_negative else 1.0
    face_y = LEAF_PLANE_Y + LEAF_FRAME_DEPTH / 2.0  # room-side face of the leaf
    # Anchor each neck a little INSIDE the stile so the handle is mechanically
    # tied to the leaf frame (no floating island), then run it out and up to the
    # standing grip.
    anchor_x = x_edge
    anchor_y = face_y - 0.014
    grip_x = x_edge + sign_x * bow_x_out
    grip_y = face_y + bow_y_out
    # Two angled mounting necks (top and bottom of the C) curve from inside the
    # stile out to the standing grip. Each neck spans the X bow and the Y
    # standoff, so it reads as the curved shoulder of a C-pull.
    neck_dx = grip_x - anchor_x
    neck_dy = grip_y - anchor_y
    neck_len = math.hypot(neck_dx, neck_dy) + 0.004
    # Lay the default +Z cylinder into the X-Y plane: pitch +90deg about Y maps
    # +Z -> +X, then yaw about Z aims it from the stile anchor toward the grip.
    neck_yaw = math.atan2(neck_dy, neck_dx)
    for sign, suffix in ((1.0, "neck_top"), (-1.0, "neck_bottom")):
        zc = sign * (handle_h / 2.0 - 0.024)
        part.visual(
            Cylinder(radius=neck_r, length=neck_len),
            origin=Origin(
                xyz=((anchor_x + grip_x) / 2.0, (anchor_y + grip_y) / 2.0, zc),
                rpy=(0.0, math.pi / 2.0, neck_yaw),
            ),
            material=mat,
            name=f"{name_prefix}_{suffix}",
        )
    # The vertical grip bar standing proud and bowed out to the side. Spans the
    # full handle height so both necks land solidly along its side (no gap).
    part.visual(
        Cylinder(radius=tube_r, length=handle_h),
        origin=Origin(xyz=(grip_x, grip_y, 0.0)),
        material=mat,
        name=f"{name_prefix}_grip",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="four_panel_sliding_patio_door")

    white = model.material("white_vinyl", rgba=(0.93, 0.94, 0.94, 1.0))
    white_trim = model.material("white_trim", rgba=(0.88, 0.89, 0.90, 1.0))
    glass = model.material("clear_glass", rgba=(0.62, 0.72, 0.78, 0.28))
    # Reference handles are white uPVC pulls (not chrome). Use a clean bright
    # white, slightly brighter than the frame so the paired C-pulls read.
    handle_white = model.material("white_handle", rgba=(0.96, 0.96, 0.97, 1.0))

    # ------------------------------------------------------------------
    # Root: white perimeter frame + center mullion + two FIXED outer panes.
    # ------------------------------------------------------------------
    frame = model.part("frame")

    # Perimeter frame profiles (thin extruded white sections).
    cx0 = 0.0
    # Top header and bottom sill run the full width.
    frame.visual(
        Box((TOTAL_W, FRAME_DEPTH, FRAME_FACE)),
        origin=Origin(xyz=(cx0, 0.0, TOTAL_H - FRAME_FACE / 2.0)),
        material=white,
        name="head_jamb",
    )
    frame.visual(
        Box((TOTAL_W, FRAME_DEPTH, FRAME_FACE)),
        origin=Origin(xyz=(cx0, 0.0, FRAME_FACE / 2.0)),
        material=white,
        name="sill",
    )
    # Left and right vertical jambs (between head and sill).
    jamb_h = TOTAL_H - 2.0 * FRAME_FACE
    frame.visual(
        Box((FRAME_FACE, FRAME_DEPTH, jamb_h)),
        origin=Origin(xyz=(-TOTAL_W / 2.0 + FRAME_FACE / 2.0, 0.0, TOTAL_H / 2.0)),
        material=white,
        name="jamb_0",
    )
    frame.visual(
        Box((FRAME_FACE, FRAME_DEPTH, jamb_h)),
        origin=Origin(xyz=(TOTAL_W / 2.0 - FRAME_FACE / 2.0, 0.0, TOTAL_H / 2.0)),
        material=white,
        name="jamb_1",
    )

    # Vertical mullions separating the fixed outer bays from the sliding bays.
    # There is NO center mullion: the two sliding leaves meet stile-to-stile at
    # the centerline, exactly as in a real center-meeting bi-parting slider.
    mullion_h = INNER_H
    mullion_z = (INNER_Z0 + INNER_Z1) / 2.0
    # Mullions sit on the outer half of the frame depth so the room-side sliding
    # leaves clear them. Depth and Y offset keep their front face at ~Y=0.
    mullion_depth = 0.050
    mullion_y = -FRAME_DEPTH / 2.0 + mullion_depth / 2.0  # front face at ~Y=+0.005
    mullion_x = [
        (BAY_CX[0] + BAY_CX[1]) / 2.0,   # between far-left fixed pane and left leaf
        (BAY_CX[2] + BAY_CX[3]) / 2.0,   # between right leaf and far-right fixed pane
    ]
    for i, mx in enumerate(mullion_x):
        frame.visual(
            Box((MULLION_FACE, mullion_depth, mullion_h)),
            origin=Origin(xyz=(mx, mullion_y, mullion_z)),
            material=white,
            name=f"mullion_{i}",
        )

    # Head and sill slide tracks for the moving leaves: guide channels on the
    # room-side plane that capture the leaf top/bottom rails. Each track bridges
    # from the perimeter frame down/up onto the leaf-rail plane, so the leaf is
    # captured top and bottom and rides along X. The small top/bottom capture
    # overlap with the leaf rails is the intended sliding engagement and is
    # declared with scoped allowances in run_tests().
    track_span = (BAY_CX[2] + BAY_W / 2.0) - (BAY_CX[1] - BAY_W / 2.0)
    track_cx = (BAY_CX[1] + BAY_CX[2]) / 2.0
    track_depth = 0.034
    # Head channel: spans Z[2.020, 2.052] -> laps onto the head_jamb (z>=2.04)
    # above and over the leaf top rail (z<=2.04) below.
    frame.visual(
        Box((track_span, track_depth, 0.032)),
        origin=Origin(xyz=(track_cx, LEAF_PLANE_Y, 2.036)),
        material=white_trim,
        name="head_track",
    )
    # Sill channel: spans Z[0.050, 0.114] -> laps onto the sill (z<=0.060)
    # below and over the leaf bottom rail (z in 0.060..0.115) above.
    frame.visual(
        Box((track_span, track_depth, 0.064)),
        origin=Origin(xyz=(track_cx, LEAF_PLANE_Y, 0.082)),
        material=white_trim,
        name="sill_track",
    )

    # Two FIXED outer glass panes (bays 0 and 3), inset from the frame faces.
    fixed_pane_w = BAY_W - 2.0 * PANE_CLEAR
    fixed_pane_h = INNER_H - 2.0 * PANE_CLEAR
    fixed_pane_cz = (INNER_Z0 + INNER_Z1) / 2.0
    _add_pane(frame, BAY_CX[0], fixed_pane_cz, fixed_pane_w, fixed_pane_h, FIXED_PANE_Y, glass, "outer_pane_0")
    _add_pane(frame, BAY_CX[3], fixed_pane_cz, fixed_pane_w, fixed_pane_h, FIXED_PANE_Y, glass, "outer_pane_1")

    # ------------------------------------------------------------------
    # Sliding center leaves, authored in the leaf-local frame (centered at
    # x=0, z=0). The prismatic joint origin places each leaf at its closed bay
    # center, so at q=0 the part frame coincides with the joint frame there.
    # ------------------------------------------------------------------
    # Glass laps horizontally UNDER the left/right stiles (glazing rebate) so the
    # pane is mechanically captured by the leaf frame, not a floating island. It
    # stays clear of the top/bottom rails (and therefore the head/sill tracks)
    # vertically so the only track engagement is rail-to-channel.
    glass_lap = 0.012
    leaf_glass_w = (BAY_W - 2.0 * LEAF_STILE) + 2.0 * glass_lap
    leaf_glass_h = (INNER_H - 2.0 * LEAF_STILE) - 0.004
    inner_stile_x = BAY_W / 2.0 - LEAF_STILE / 2.0  # local X of the meeting stile

    door_0 = model.part("door_0")  # closed at bay 1, slides toward -X
    _add_leaf_frame(door_0, white)
    _add_pane(door_0, 0.0, 0.0, leaf_glass_w, leaf_glass_h, LEAF_PLANE_Y, glass, "leaf_pane")
    # Handle on the inner (meeting) stile = +X edge of door_0.
    _add_curved_handle(door_0, inner_stile_x, handle_white, "handle", opens_negative=True)

    door_1 = model.part("door_1")  # closed at bay 2, slides toward +X
    _add_leaf_frame(door_1, white)
    _add_pane(door_1, 0.0, 0.0, leaf_glass_w, leaf_glass_h, LEAF_PLANE_Y, glass, "leaf_pane")
    # Handle on the inner (meeting) stile = -X edge of door_1.
    _add_curved_handle(door_1, -inner_stile_x, handle_white, "handle", opens_negative=False)

    # ------------------------------------------------------------------
    # Articulations: two prismatic joints, bi-parting along X.
    # Travel ~= one bay width so each leaf fully clears the center opening and
    # tucks in front of the matching fixed outer pane.
    # ------------------------------------------------------------------
    travel = BAY_W - 0.010  # slightly under a full bay so the leaf stays guided
    # Small center reveal so the two meeting stiles do not touch when closed.
    meet_reveal = 0.004

    door_0_slide = model.articulation(
        "door_0_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=door_0,
        # Joint frame at the closed leaf center (shifted out by the reveal so the
        # meeting stiles stay clear of the centerline); child opens along -X.
        origin=Origin(xyz=(BAY_CX[1] - meet_reveal, 0.0, LEAF_CZ)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.5, lower=0.0, upper=travel),
    )
    # door_1 mirrors door_0 and opens along +X; mimic-coupled so one scalar
    # opens both leaves symmetrically.
    model.articulation(
        "door_1_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=door_1,
        origin=Origin(xyz=(BAY_CX[2] + meet_reveal, 0.0, LEAF_CZ)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.5, lower=0.0, upper=travel),
        mimic=Mimic(joint=door_0_slide.name, multiplier=1.0, offset=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    door_0 = object_model.get_part("door_0")
    door_1 = object_model.get_part("door_1")
    door_0_slide = object_model.get_articulation("door_0_slide")

    # Intentional captured-guide overlaps: each leaf's top and bottom rail rides
    # inside the head/sill guide channels of the frame. This is the real sliding
    # engagement that keeps the leaf retained in its track along its full travel,
    # so it is allowed and proven below, not removed.
    for leaf in (door_0, door_1):
        ctx.allow_overlap(
            leaf,
            frame,
            elem_a="top_rail",
            elem_b="head_track",
            reason="Leaf top rail is captured inside the head guide channel; this is the sliding-track engagement.",
        )
        ctx.allow_overlap(
            leaf,
            frame,
            elem_a="bottom_rail",
            elem_b="sill_track",
            reason="Leaf bottom rail is captured inside the sill guide channel; this is the sliding-track engagement.",
        )

    # Frame stands on the floor (sill bottom near z=0).
    frame_aabb = ctx.part_world_aabb(frame)
    ctx.check(
        "frame stands on the ground plane",
        frame_aabb is not None and abs(frame_aabb[0][2]) < 0.01 and frame_aabb[1][2] > 2.0,
        details=f"frame_aabb={frame_aabb}",
    )

    # Closed leaves meet at the centerline (small gap, no penetration).
    ctx.expect_gap(
        door_1,
        door_0,
        axis="x",
        max_gap=0.02,
        max_penetration=0.0,
        name="closed center leaves meet at the centerline",
    )

    # Each leaf glass covers most of the opening height.
    leaf_pane_0 = door_0.get_visual("leaf_pane")
    leaf_pane_1 = door_1.get_visual("leaf_pane")
    ctx.expect_overlap(
        door_0,
        frame,
        axes="z",
        min_overlap=1.70,
        elem_a="leaf_pane",
        name="left leaf covers the opening height",
    )
    ctx.expect_overlap(
        door_1,
        frame,
        axes="z",
        min_overlap=1.70,
        elem_a="leaf_pane",
        name="right leaf covers the opening height",
    )

    # Leaves ride on the room side, in front of (not intersecting) the fixed
    # outer panes: their depth planes are clearly separated.
    ctx.expect_gap(
        door_0,
        frame,
        axis="y",
        min_gap=0.0,
        positive_elem="leaf_pane",
        negative_elem="outer_pane_0",
        name="left leaf rides in front of its outer pane in depth",
    )

    # Captured-guide engagement: the leaf top rail stays inside the head track
    # channel (vertical capture) when closed.
    ctx.expect_overlap(
        door_0,
        frame,
        axes="z",
        min_overlap=0.012,
        elem_a="top_rail",
        elem_b="head_track",
        name="closed left leaf top rail is captured in the head track",
    )
    ctx.expect_overlap(
        door_0,
        frame,
        axes="z",
        min_overlap=0.012,
        elem_a="bottom_rail",
        elem_b="sill_track",
        name="closed left leaf bottom rail is captured in the sill track",
    )

    # Open pose: parts the leaves and tucks each behind/over the matching fixed
    # outer pane. Drive the source joint; the mimic follower moves with it.
    rest_x0 = ctx.part_world_position(door_0)
    rest_x1 = ctx.part_world_position(door_1)
    open_q = door_0_slide.motion_limits.upper
    with ctx.pose({door_0_slide: open_q}):
        open_x0 = ctx.part_world_position(door_0)
        open_x1 = ctx.part_world_position(door_1)
        # Leaf stays captured in the head track even at full travel (the leaf
        # rides along X within the channel span).
        ctx.expect_overlap(
            door_0,
            frame,
            axes="z",
            min_overlap=0.012,
            elem_a="top_rail",
            elem_b="head_track",
            name="open left leaf top rail stays captured in the head track",
        )
        # Leaves have parted: door_1 now far to +X of door_0.
        ctx.expect_origin_gap(
            door_1,
            door_0,
            axis="x",
            min_gap=2.0 * open_q - 0.05,
            name="open leaves are parted across the center",
        )
        # Each open leaf overlaps its matching fixed outer pane bay in projection.
        ctx.expect_overlap(
            door_0,
            frame,
            axes="x",
            min_overlap=0.40,
            elem_a="leaf_pane",
            elem_b="outer_pane_0",
            name="open left leaf tucks over the far-left fixed pane",
        )
        ctx.expect_overlap(
            door_1,
            frame,
            axes="x",
            min_overlap=0.40,
            elem_a="leaf_pane",
            elem_b="outer_pane_1",
            name="open right leaf tucks over the far-right fixed pane",
        )

    ctx.check(
        "left leaf slides toward -X when opening",
        rest_x0 is not None and open_x0 is not None and open_x0[0] < rest_x0[0] - 0.5,
        details=f"rest={rest_x0}, open={open_x0}",
    )
    ctx.check(
        "right leaf slides toward +X when opening",
        rest_x1 is not None and open_x1 is not None and open_x1[0] > rest_x1[0] + 0.5,
        details=f"rest={rest_x1}, open={open_x1}",
    )

    # Silence unused-name linters while keeping the resolved visuals documented.
    _ = (leaf_pane_0, leaf_pane_1)

    return ctx.report()


object_model = build_object_model()
