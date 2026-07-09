from __future__ import annotations

# Portable rechargeable LED flood / work light.
# A yellow tubular H-frame stand carries a black rectangular LED flood head on a
# panning single-rod support. The support rotates about a vertical post on the
# bench stand, and the head tilts up/down on a compact central hinge. The head has a glass LED
# panel face, a black bezel, and a yellow U-shaped carry handle with a black
# rubber grip.

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
    tube_from_spline_points,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Real-world dimensions (meters)
# ---------------------------------------------------------------------------
HEAD_W = 0.220  # face width (left-right, Y)
HEAD_H = 0.175  # face height (vertical when face is upright, X in head frame)
HEAD_D = 0.055  # housing depth (front-to-back, Z in head frame)

GLASS_INSET = 0.018  # bezel border around the glass on each side
FRAME_T = 0.010  # bezel frame wall thickness

PIVOT_R = 0.012  # side pivot boss radius
UPRIGHT_R = 0.010  # yellow stand upright tube radius
TUBE_R = 0.012  # yellow base tube radius
FOOT_R = 0.020  # black rubber foot radius

# Base H-frame footprint
BASE_LEN = 0.260  # length of each side rail (front-back run, X)
BASE_SPAN = 0.210  # spacing between the two side rails (Y)
UPRIGHT_H = 0.150  # height of the vertical uprights
RAIL_LIFT = 0.018  # raised middle of each bent base rail

# Head pivot height above the base plane. The pan/tilt variant needs a little
# more air than the fixed head so the battery box can swing without clipping
# the H-frame.
PIVOT_Z = UPRIGHT_H + 0.025
PAN_Z = 0.055  # top bearing height of the fixed vertical post
TILT_Z_IN_YOKE = PIVOT_Z - PAN_Z


def _tube(points, radius, name, *, segments=18):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=14,
            radial_segments=segments,
            cap_ends=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="led_work_light")

    # Materials
    safety_yellow = model.material("safety_yellow", rgba=(0.96, 0.78, 0.06, 1.0))
    housing_black = model.material("housing_black", rgba=(0.10, 0.10, 0.11, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.06, 0.06, 0.07, 1.0))
    glass_white = model.material("glass_white", rgba=(0.93, 0.94, 0.90, 1.0))
    led_dot = model.material("led_dot", rgba=(0.80, 0.84, 0.70, 1.0))
    steel = model.material("steel", rgba=(0.55, 0.57, 0.60, 1.0))

    # =====================================================================
    # STAND FRAME (root) -- yellow tubular H-base + fixed vertical pan post
    # =====================================================================
    stand = model.part("stand_frame")

    half_span = BASE_SPAN / 2.0
    half_len = BASE_LEN / 2.0

    # Two side rails running front-to-back (along X) at +/- Y.
    # Build as bent tubes that rise slightly to a flat run, like the real
    # stamped-tube legs. Endpoints sit just inside the foot caps.
    rail_lift = RAIL_LIFT
    foot_inset = 0.026
    for sign, tag in ((+1.0, "pos_y"), (-1.0, "neg_y")):
        y = sign * half_span
        rail_pts = [
            (-half_len + foot_inset, y, 0.006),
            (-half_len + 0.060, y, rail_lift),
            (half_len - 0.060, y, rail_lift),
            (half_len - foot_inset, y, 0.006),
        ]
        stand.visual(
            _tube(rail_pts, TUBE_R, f"side_rail_{tag}"),
            material=safety_yellow,
            name=f"side_rail_{tag}",
        )

    # Center cross member tying the two rails together (the bar of the "H").
    cross_pts = [
        (0.0, -half_span, rail_lift),
        (0.0, 0.0, rail_lift),
        (0.0, half_span, rail_lift),
    ]
    stand.visual(
        _tube(cross_pts, TUBE_R, "base_cross_member"),
        material=safety_yellow,
        name="base_cross_member",
    )

    # Four black rubber end-cap feet, one on each rail end.
    for sx, sy, tag in (
        (+1.0, +1.0, "fr"),
        (+1.0, -1.0, "br"),
        (-1.0, +1.0, "fl"),
        (-1.0, -1.0, "bl"),
    ):
        fx = sx * (half_len - foot_inset)
        fy = sy * half_span
        stand.visual(
            Cylinder(radius=FOOT_R, length=0.052),
            origin=Origin(xyz=(fx, fy, 0.006), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber_black,
            name=f"foot_{tag}",
        )

    # Fixed vertical post rising from the base cross member. The U-yoke sits on
    # this real top face and pans around it, rather than being an invisible pad.
    stand.visual(
        Cylinder(radius=0.014, length=PAN_Z - rail_lift),
        origin=Origin(xyz=(0.0, 0.0, (PAN_Z + rail_lift) / 2.0)),
        material=safety_yellow,
        name="vertical_post",
    )
    stand.visual(
        Cylinder(radius=0.028, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, PAN_Z - 0.006)),
        material=housing_black,
        name="pan_bearing_top",
    )

    # =====================================================================
    # PANNING SUPPORT (child of stand) -- turntable + real two-sided U-yoke
    # Local origin is the bearing contact plane on top of the fixed post.
    # =====================================================================
    yoke = model.part("u_yoke")

    yoke.visual(
        Cylinder(radius=0.032, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=housing_black,
        name="yoke_turntable",
    )

    fork_half_span = HEAD_W / 2.0 + 0.024
    yoke.visual(
        Cylinder(radius=0.020, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=safety_yellow,
        name="support_socket",
    )
    # A short neck and crossbar put the side arms outside the lamp housing.
    # This reads as a real fork bracket, and leaves the center open so the
    # battery pack and black back shell do not run through the support.
    yoke.visual(
        Cylinder(radius=UPRIGHT_R, length=TILT_Z_IN_YOKE - 0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.020 + (TILT_Z_IN_YOKE - 0.020) / 2.0)),
        material=safety_yellow,
        name="center_post",
    )
    yoke.visual(
        Cylinder(radius=0.0075, length=2.0 * fork_half_span),
        origin=Origin(xyz=(0.0, 0.0, TILT_Z_IN_YOKE - 0.030), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=safety_yellow,
        name="lower_fork_crossbar",
    )
    for sign, tag in ((+1.0, "pos_y"), (-1.0, "neg_y")):
        y = sign * fork_half_span
        yoke.visual(
            _tube(
                [
                    (0.0, sign * 0.018, TILT_Z_IN_YOKE - 0.030),
                    (0.0, sign * (fork_half_span - 0.020), TILT_Z_IN_YOKE - 0.030),
                    (0.0, y, TILT_Z_IN_YOKE - 0.006),
                    (0.0, y, TILT_Z_IN_YOKE + 0.018),
                ],
                0.008,
                f"fork_arm_{tag}",
                segments=18,
            ),
            material=safety_yellow,
            name=f"fork_arm_{tag}",
        )
        yoke.visual(
            Box((0.046, 0.014, 0.040)),
            origin=Origin(xyz=(0.0, y, TILT_Z_IN_YOKE)),
            material=safety_yellow,
            name=f"pivot_cheek_{tag}",
        )
        yoke.visual(
            Cylinder(radius=0.015, length=0.010),
            origin=Origin(
                xyz=(0.0, sign * (fork_half_span + 0.010), TILT_Z_IN_YOKE),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=housing_black,
            name=f"pivot_knob_{tag}",
        )
    yoke.visual(
        Cylinder(radius=0.006, length=2.0 * fork_half_span + 0.028),
        origin=Origin(xyz=(0.0, 0.0, TILT_Z_IN_YOKE), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_axle",
    )

    # =====================================================================
    # LIGHT HEAD (child) -- black flood housing, glass LED face, yellow carry
    # handle. Head frame origin sits at the pivot center.
    #   +X = "up" along the face (toward the top edge / handle)
    #   +Y = left-right across the face
    #   +Z = front (out of the glass, the lit direction)
    # =====================================================================
    head = model.part("light_head")

    # Housing shell built as a thin box "tub": back wall + 4 side walls so the
    # front is an open recess that the glass panel seats into (hollow, not solid).
    back_z = -HEAD_D / 2.0
    # Back wall
    head.visual(
        Box((HEAD_H, HEAD_W, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, back_z + 0.005)),
        material=housing_black,
        name="housing_back",
    )
    # Side walls (top / bottom along X, left / right along Y)
    wall_z = 0.0
    wall_d = HEAD_D - 0.006
    head.visual(
        Box((FRAME_T, HEAD_W, wall_d)),
        origin=Origin(xyz=(HEAD_H / 2.0 - FRAME_T / 2.0, 0.0, wall_z)),
        material=housing_black,
        name="housing_wall_top",
    )
    head.visual(
        Box((FRAME_T, HEAD_W, wall_d)),
        origin=Origin(xyz=(-HEAD_H / 2.0 + FRAME_T / 2.0, 0.0, wall_z)),
        material=housing_black,
        name="housing_wall_bottom",
    )
    head.visual(
        Box((HEAD_H, FRAME_T, wall_d)),
        origin=Origin(xyz=(0.0, HEAD_W / 2.0 - FRAME_T / 2.0, wall_z)),
        material=housing_black,
        name="housing_wall_left",
    )
    head.visual(
        Box((HEAD_H, FRAME_T, wall_d)),
        origin=Origin(xyz=(0.0, -HEAD_W / 2.0 + FRAME_T / 2.0, wall_z)),
        material=housing_black,
        name="housing_wall_right",
    )

    # Front bezel frame (black trim ring) around the glass opening.
    front_z = HEAD_D / 2.0
    bezel_t = 0.006
    glass_w = HEAD_W - 2.0 * GLASS_INSET
    glass_h = HEAD_H - 2.0 * GLASS_INSET
    # Top/bottom bezel bars
    head.visual(
        Box((GLASS_INSET, HEAD_W, bezel_t)),
        origin=Origin(xyz=(HEAD_H / 2.0 - GLASS_INSET / 2.0, 0.0, front_z - bezel_t / 2.0)),
        material=housing_black,
        name="bezel_top",
    )
    head.visual(
        Box((GLASS_INSET, HEAD_W, bezel_t)),
        origin=Origin(xyz=(-HEAD_H / 2.0 + GLASS_INSET / 2.0, 0.0, front_z - bezel_t / 2.0)),
        material=housing_black,
        name="bezel_bottom",
    )
    # Left/right bezel bars
    head.visual(
        Box((glass_h, GLASS_INSET, bezel_t)),
        origin=Origin(xyz=(0.0, HEAD_W / 2.0 - GLASS_INSET / 2.0, front_z - bezel_t / 2.0)),
        material=housing_black,
        name="bezel_left",
    )
    head.visual(
        Box((glass_h, GLASS_INSET, bezel_t)),
        origin=Origin(xyz=(0.0, -HEAD_W / 2.0 + GLASS_INSET / 2.0, front_z - bezel_t / 2.0)),
        material=housing_black,
        name="bezel_right",
    )

    # Glass / diffuser LED panel seated in the recess behind the bezel front.
    # Sized to tuck just under the bezel lip so it reads as seated and stays
    # geometrically connected to the bezel frame (no floating island).
    glass_z = front_z - bezel_t - 0.004
    glass_panel_h = glass_h + 2.0 * 0.004
    glass_panel_w = glass_w + 2.0 * 0.004
    head.visual(
        Box((glass_panel_h, glass_panel_w, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, glass_z)),
        material=glass_white,
        name="led_glass_panel",
    )

    # LED dot array sitting just proud of the glass, in rows/columns.
    led_rows, led_cols = 5, 8
    led_size = 0.009
    led_z = glass_z + 0.004
    margin = 0.012
    span_x = glass_h - 2.0 * margin
    span_y = glass_w - 2.0 * margin
    for r in range(led_rows):
        px = -span_x / 2.0 + span_x * r / (led_rows - 1)
        for c in range(led_cols):
            py = -span_y / 2.0 + span_y * c / (led_cols - 1)
            head.visual(
                Box((led_size, led_size, 0.0025)),
                origin=Origin(xyz=(px, py, led_z)),
                material=led_dot,
                name=f"led_{r}_{c}",
            )

    # Side pivot bosses: short cylinders on each side of the housing, captured
    # by the U-yoke cheeks. The middle stays open, preventing the support from
    # cutting through the rear shell or battery pack during tilt.
    boss_len = 0.032
    boss_y = HEAD_W / 2.0 + 0.012
    for sign, tag in ((+1.0, "pos_y"), (-1.0, "neg_y")):
        head.visual(
            Cylinder(radius=PIVOT_R, length=boss_len),
            origin=Origin(xyz=(0.0, sign * boss_y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"pivot_boss_{tag}",
        )

    # U-shaped yellow carry handle rising from the top of the head, with a black
    # rubber grip across the top. Rooted into the housing top wall.
    htop = HEAD_H / 2.0
    h_rise = 0.105
    h_arc = 0.060
    handle_pts = [
        (htop - 0.004, HEAD_W / 2.0 - 0.018, -0.004),
        (htop + h_rise * 0.55, HEAD_W / 2.0 - 0.030, h_arc * 0.55),
        (htop + h_rise, 0.0, h_arc),
        (htop + h_rise * 0.55, -HEAD_W / 2.0 + 0.030, h_arc * 0.55),
        (htop - 0.004, -HEAD_W / 2.0 + 0.018, -0.004),
    ]
    head.visual(
        _tube(handle_pts, 0.010, "carry_handle", segments=20),
        material=safety_yellow,
        name="carry_handle",
    )
    # Black rubber grip sleeve over the top span of the handle.
    grip_pts = [
        (htop + h_rise - 0.002, 0.045, h_arc + 0.001),
        (htop + h_rise + 0.002, 0.0, h_arc + 0.002),
        (htop + h_rise - 0.002, -0.045, h_arc + 0.001),
    ]
    head.visual(
        _tube(grip_pts, 0.015, "handle_grip", segments=20),
        material=rubber_black,
        name="handle_grip",
    )

    # =====================================================================
    # ARTICULATION STACK
    # 1) Stand-to-support pans about the fixed vertical post.
    # 2) Head tilts between the two side cheeks of the yoke.
    # =====================================================================
    model.articulation(
        "stand_to_yoke",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, PAN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "yoke_to_head",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, TILT_Z_IN_YOKE)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.55, upper=0.70),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand_frame")
    yoke = object_model.get_part("u_yoke")
    head = object_model.get_part("light_head")
    pan = object_model.get_articulation("stand_to_yoke")
    tilt = object_model.get_articulation("yoke_to_head")

    # ---- Joint type / axis claims --------------------------------------
    ctx.check(
        "pan joint is revolute",
        pan.articulation_type == ArticulationType.REVOLUTE,
        details=f"got {pan.articulation_type}",
    )
    pan_ax = tuple(round(a, 6) for a in pan.axis)
    ctx.check(
        "pan axis is vertical (Z)",
        pan_ax == (0.0, 0.0, 1.0),
        details=f"axis={pan_ax}",
    )
    ctx.check(
        "tilt joint is revolute",
        tilt.articulation_type == ArticulationType.REVOLUTE,
        details=f"got {tilt.articulation_type}",
    )
    ax = tuple(round(a, 6) for a in tilt.axis)
    ctx.check(
        "tilt axis is left-right (Y)",
        abs(ax[1]) == 1.0 and ax[0] == 0.0 and ax[2] == 0.0,
        details=f"axis={ax}",
    )

    # ---- Hero geometry present -----------------------------------------
    glass = head.get_visual("led_glass_panel")
    handle = head.get_visual("carry_handle")
    grip = head.get_visual("handle_grip")
    head_visual_names = {visual.name for visual in head.visuals}
    ctx.check("glass LED panel present", glass is not None)
    ctx.check("carry handle present", handle is not None)
    ctx.check("handle grip present", grip is not None)
    ctx.check("rear yellow battery block removed", "battery_pack" not in head_visual_names)
    ctx.check("rear battery port panel removed", "battery_port_panel" not in head_visual_names)

    # ---- Placement: handle apex rises above the glass face -------------
    # At rest (q=0) the head frame is aligned with world, so the head-local
    # "up" axis (+X) maps to world +X. The handle arc apex must clear the
    # glass panel along that axis.
    handle_top = ctx.part_element_world_aabb(head, elem="carry_handle")[1][0]
    glass_top = ctx.part_element_world_aabb(head, elem="led_glass_panel")[1][0]
    ctx.check(
        "handle arc rises above the LED face",
        handle_top > glass_top + 0.05,
        details=f"handle_top_x={handle_top:.4f}, glass_top_x={glass_top:.4f}",
    )
    # Handle roots into the housing top wall (mounted, not floating).
    ctx.expect_contact(
        head,
        head,
        elem_a="carry_handle",
        elem_b="housing_wall_top",
        name="handle rooted into housing top",
    )

    # ---- Stand carries the panning yoke; yoke carries the head ----------
    for tag in ("fr", "br", "fl", "bl"):
        foot = stand.get_visual(f"foot_{tag}")
        ctx.check(f"foot_{tag} present", foot is not None)

    ctx.check("fixed vertical post present", stand.get_visual("vertical_post") is not None)
    ctx.check("panning support turntable present", yoke.get_visual("yoke_turntable") is not None)
    ctx.check("support socket present", yoke.get_visual("support_socket") is not None)
    ctx.check("center post present", yoke.get_visual("center_post") is not None)
    ctx.check("lower fork crossbar present", yoke.get_visual("lower_fork_crossbar") is not None)
    ctx.check("pivot axle present", yoke.get_visual("pivot_axle") is not None)
    for tag in ("pos_y", "neg_y"):
        ctx.check(f"fork arm {tag} present", yoke.get_visual(f"fork_arm_{tag}") is not None)
        ctx.check(f"pivot cheek {tag} present", yoke.get_visual(f"pivot_cheek_{tag}") is not None)
        ctx.check(f"pivot knob {tag} present", yoke.get_visual(f"pivot_knob_{tag}") is not None)
        ctx.check(f"head side pivot {tag} present", head.get_visual(f"pivot_boss_{tag}") is not None)
    ctx.expect_contact(
        yoke,
        stand,
        elem_a="yoke_turntable",
        elem_b="pan_bearing_top",
        name="yoke sits on vertical post bearing",
    )

    # The fork support rises from the pan bearing and holds only the side pivot
    # bosses, leaving the center of the lamp back clear.
    ctx.expect_contact(
        yoke,
        yoke,
        elem_a="support_socket",
        elem_b="yoke_turntable",
        name="single support socket is seated on turntable",
    )
    ctx.expect_contact(
        yoke,
        yoke,
        elem_a="center_post",
        elem_b="support_socket",
        name="center post rises from support socket",
    )
    for tag in ("pos_y", "neg_y"):
        ctx.allow_overlap(
            head,
            yoke,
            elem_a=f"pivot_boss_{tag}",
            elem_b=f"pivot_cheek_{tag}",
            reason="Side pivot boss passes through the yoke cheek as the tilt bearing.",
        )
        ctx.expect_overlap(
            head,
            yoke,
            axes="y",
            elem_a=f"pivot_boss_{tag}",
            elem_b=f"pivot_cheek_{tag}",
            min_overlap=0.006,
            name=f"side pivot boss {tag} is captured by yoke cheek",
        )
        ctx.expect_overlap(
            head,
            yoke,
            axes="z",
            elem_a=f"pivot_boss_{tag}",
            elem_b=f"pivot_cheek_{tag}",
            min_overlap=0.018,
            name=f"side pivot boss {tag} aligns vertically with cheek",
        )

    pos_cheek = ctx.part_element_world_aabb(yoke, elem="pivot_cheek_pos_y")
    neg_cheek = ctx.part_element_world_aabb(yoke, elem="pivot_cheek_neg_y")
    pos_wall = ctx.part_element_world_aabb(head, elem="housing_wall_left")
    neg_wall = ctx.part_element_world_aabb(head, elem="housing_wall_right")
    ctx.check(
        "positive yoke cheek clears lamp housing side",
        pos_cheek[0][1] > pos_wall[1][1] + 0.008,
        details=f"cheek_inner_y={pos_cheek[0][1]:.4f}, housing_outer_y={pos_wall[1][1]:.4f}",
    )
    ctx.check(
        "negative yoke cheek clears lamp housing side",
        neg_cheek[1][1] < neg_wall[0][1] - 0.008,
        details=f"cheek_inner_y={neg_cheek[1][1]:.4f}, housing_outer_y={neg_wall[0][1]:.4f}",
    )
    yoke_visual_names = {visual.name for visual in yoke.visuals}
    ctx.check("old single support rod removed", "support_rod" not in yoke_visual_names)
    ctx.check("old tilt hinge block removed", "tilt_hinge_block" not in yoke_visual_names)
    ctx.check("old central pivot removed", "center_pivot_boss" not in head_visual_names)
    ctx.check("old central hinge lug removed", "center_hinge_lug" not in head_visual_names)

    # ---- Pan actually rotates the yoke around the vertical post ----------
    rest_socket = ctx.part_element_world_aabb(yoke, elem="fork_arm_pos_y")
    with ctx.pose({pan: 0.65}):
        panned_socket = ctx.part_element_world_aabb(yoke, elem="support_socket")
        panned_fork = ctx.part_element_world_aabb(yoke, elem="fork_arm_pos_y")
    rest_socket_center_y = (rest_socket[0][1] + rest_socket[1][1]) / 2.0
    panned_socket_center_y = (panned_fork[0][1] + panned_fork[1][1]) / 2.0
    ctx.check(
        "positive pan swings the fork around the post",
        abs(panned_socket_center_y - rest_socket_center_y) > 0.012,
        details=f"rest_y={rest_socket_center_y:.4f}, panned_y={panned_socket_center_y:.4f}",
    )

    # ---- Tilt actually moves the face upward ---------------------------
    rest = ctx.part_element_world_aabb(head, elem="led_glass_panel")
    with ctx.pose({tilt: 0.6}):
        up = ctx.part_element_world_aabb(head, elem="led_glass_panel")
    rest_top = rest[1][2]
    up_top = up[1][2]
    ctx.check(
        "positive tilt raises the LED face",
        up_top > rest_top + 0.01,
        details=f"rest_top_z={rest_top:.4f}, tilted_top_z={up_top:.4f}",
    )
    with ctx.pose({tilt: tilt.motion_limits.lower}):
        low_housing = ctx.part_element_world_aabb(head, elem="housing_back")[0][2]
    ctx.check(
        "lowest tilt keeps the rear housing clear of the base frame",
        low_housing > RAIL_LIFT + 0.010,
        details=f"housing_min_z={low_housing:.4f}, base_rail_z={RAIL_LIFT:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
