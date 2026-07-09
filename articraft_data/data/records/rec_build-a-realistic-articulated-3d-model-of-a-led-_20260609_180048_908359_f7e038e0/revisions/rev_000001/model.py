from __future__ import annotations

# Portable rechargeable LED flood / work light.
# A yellow tubular H-frame stand carries a black rectangular LED flood head that
# tilts up and down on a side pivot. The head has a glass LED panel face, a black
# bezel, a yellow battery pack on the back, and a yellow U-shaped carry handle
# with a black rubber grip. The primary mechanism is the head tilt (REVOLUTE).

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

# Head pivot height above the base plane
PIVOT_Z = UPRIGHT_H


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
    # STAND FRAME (root) -- yellow tubular H-base + 2 uprights + black feet
    # =====================================================================
    stand = model.part("stand_frame")

    half_span = BASE_SPAN / 2.0
    half_len = BASE_LEN / 2.0

    # Two side rails running front-to-back (along X) at +/- Y.
    # Build as bent tubes that rise slightly to a flat run, like the real
    # stamped-tube legs. Endpoints sit just inside the foot caps.
    rail_lift = 0.018
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

    # Two vertical uprights rising to the pivot. They flare outward as they rise
    # so the vertical run sits well outboard of the housing side walls, then
    # cradle the head's side pivot bosses from outside the housing.
    upright_top_y = HEAD_W / 2.0 + 0.018
    for sign, tag in ((+1.0, "pos_y"), (-1.0, "neg_y")):
        y0 = sign * half_span
        y1 = sign * upright_top_y
        up_pts = [
            (0.0, y0, rail_lift),
            (0.0, sign * (half_span + 0.012), UPRIGHT_H * 0.30),
            (0.0, y1, UPRIGHT_H * 0.62),
            (0.0, y1, UPRIGHT_H),
        ]
        stand.visual(
            _tube(up_pts, UPRIGHT_R, f"upright_{tag}"),
            material=safety_yellow,
            name=f"upright_{tag}",
        )
        # Pivot knob / bolt head on the outside of each upright top.
        stand.visual(
            Cylinder(radius=0.013, length=0.010),
            origin=Origin(
                xyz=(0.0, sign * (upright_top_y + 0.006), PIVOT_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=housing_black,
            name=f"pivot_knob_{tag}",
        )

    # =====================================================================
    # LIGHT HEAD (child) -- black flood housing, glass LED face, battery box,
    # yellow carry handle. Head frame origin sits at the pivot center.
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

    # Side pivot bosses: short cylinders on each side of the housing, on the
    # Y axis, captured by the stand upright tops.
    boss_len = 0.046
    boss_y = HEAD_W / 2.0 + 0.004
    for sign, tag in ((+1.0, "pos_y"), (-1.0, "neg_y")):
        head.visual(
            Cylinder(radius=PIVOT_R, length=boss_len),
            origin=Origin(xyz=(0.0, sign * boss_y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"pivot_boss_{tag}",
        )

    # Yellow battery pack box mounted on the back of the housing (lower rear),
    # like the angled yellow pack visible behind the head in the image.
    # Kept narrower than the upright spacing so it clears the stand uprights.
    batt_w = 0.150
    batt_depth = 0.050
    # Front of the box embeds slightly into the back wall so it reads bolted on.
    batt_z = back_z - batt_depth / 2.0 + 0.006
    head.visual(
        Box((0.090, batt_w, batt_depth)),
        origin=Origin(xyz=(-0.030, 0.0, batt_z)),
        material=safety_yellow,
        name="battery_pack",
    )
    # Black control/port strip on the back of the battery pack.
    head.visual(
        Box((0.030, 0.060, 0.010)),
        origin=Origin(xyz=(-0.030, 0.0, batt_z - batt_depth / 2.0 - 0.004)),
        material=housing_black,
        name="battery_port_panel",
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
    # ARTICULATION -- head tilts up/down about the Y pivot axis.
    # The pivot frame is at the stand upright tops (z = PIVOT_Z), centered.
    # Head frame origin coincides with the pivot center at q = 0.
    # Positive q (right-hand rule about +Y) rotates the front (+Z) of the face
    # toward +X ... so use -Y to make the face tilt UPWARD (face toward +Z/up).
    # Here at rest the face points +Z (forward); positive tilt aims it upward.
    # =====================================================================
    model.articulation(
        "stand_to_head",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.70, upper=0.70),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand_frame")
    head = object_model.get_part("light_head")
    tilt = object_model.get_articulation("stand_to_head")

    # ---- Joint type / axis claims --------------------------------------
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
    battery = head.get_visual("battery_pack")
    ctx.check("glass LED panel present", glass is not None)
    ctx.check("carry handle present", handle is not None)
    ctx.check("handle grip present", grip is not None)
    ctx.check("battery pack present", battery is not None)

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

    # ---- Stand carries the head: four feet + uprights ------------------
    for tag in ("fr", "br", "fl", "bl"):
        foot = stand.get_visual(f"foot_{tag}")
        ctx.check(f"foot_{tag} present", foot is not None)

    # Head must overlap the stand footprint and the pivot bosses must be
    # captured by the uprights -> intentional nested fit.
    ctx.allow_overlap(
        head,
        stand,
        elem_a="pivot_boss_pos_y",
        elem_b="upright_pos_y",
        reason="Pivot boss is captured inside the upright top to form the tilt bearing.",
    )
    ctx.allow_overlap(
        head,
        stand,
        elem_a="pivot_boss_neg_y",
        elem_b="upright_neg_y",
        reason="Pivot boss is captured inside the upright top to form the tilt bearing.",
    )
    ctx.allow_overlap(
        head,
        stand,
        elem_a="pivot_boss_pos_y",
        elem_b="pivot_knob_pos_y",
        reason="Pivot bolt head seats into the pivot boss face.",
    )
    ctx.allow_overlap(
        head,
        stand,
        elem_a="pivot_boss_neg_y",
        elem_b="pivot_knob_neg_y",
        reason="Pivot bolt head seats into the pivot boss face.",
    )

    # Pivot boss reaches the upright top (the supporting bearing contact).
    ctx.expect_contact(
        head,
        stand,
        elem_a="pivot_boss_pos_y",
        elem_b="upright_pos_y",
        name="pivot boss bears on upright",
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

    return ctx.report()


object_model = build_object_model()
