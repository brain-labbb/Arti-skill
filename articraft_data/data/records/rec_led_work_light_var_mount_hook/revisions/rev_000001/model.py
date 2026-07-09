from __future__ import annotations

# Portable rechargeable LED flood / work light.
# A compact handheld yellow base carries the same black rectangular LED flood
# head on a side pivot. The base has a rear bench stand foot and a fold-out
# hanging hook. The primary user-facing mechanisms are the head tilt and the
# hanging hook hinge (both REVOLUTE).

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
    rounded_rect_profile,
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

# Compact handheld base / yoke dimensions
BASE_BODY_X = 0.120
BASE_BODY_Y = 0.270
BASE_BODY_Z = 0.060
BASE_TOP_Z = BASE_BODY_Z + 0.010
YOKE_H = 0.130  # raises head pivot from 0.150m to 0.200m (4/3) for tilt clearance

# Head pivot height above the base plane
PIVOT_Z = BASE_TOP_Z + YOKE_H


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


def _rounded_box(size_x, size_y, size_z, radius, name):
    """Rounded-rectangle extrusion used for compact molded plastic features."""
    return mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(size_x, size_y, radius, corner_segments=8),
            size_z,
            center=True,
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
    # HANDHELD BASE (root) -- compact molded battery/handle pod, rear bench
    # stand foot, hinge lugs for the fold-out hook, and yellow yoke cheeks
    # carrying the flood head pivot.
    # =====================================================================
    base = model.part("handheld_base")

    base.visual(
        _rounded_box(BASE_BODY_X, BASE_BODY_Y, BASE_BODY_Z, 0.022, "base_shell"),
        origin=Origin(xyz=(0.0, 0.0, BASE_BODY_Z / 2.0)),
        material=safety_yellow,
        name="base_shell",
    )
    # Raised saddle on the top face, visibly molded into the main base.
    base.visual(
        _rounded_box(0.072, 0.150, 0.018, 0.012, "top_saddle"),
        origin=Origin(xyz=(0.0, 0.0, BASE_BODY_Z + 0.004)),
        material=safety_yellow,
        name="top_saddle",
    )
    # Black inset control/charge panel and a round power button on the top.
    base.visual(
        Box((0.038, 0.085, 0.004)),
        origin=Origin(xyz=(-0.036, 0.0, BASE_BODY_Z + 0.015)),
        material=housing_black,
        name="control_panel",
    )
    base.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(-0.036, 0.0, BASE_BODY_Z + 0.020)),
        material=rubber_black,
        name="power_button",
    )

    # Broad rear rubber foot props the unit on a workbench without the old floor
    # stand. It is molded onto the back/underside and not a separate mechanism.
    base.visual(
        _rounded_box(0.032, 0.145, 0.024, 0.010, "rear_stand_foot"),
        origin=Origin(xyz=(-BASE_BODY_X / 2.0 - 0.006, 0.0, 0.018)),
        material=rubber_black,
        name="rear_stand_foot",
    )
    # Small front rubber pad for bench use, connected to the underside.
    base.visual(
        _rounded_box(0.024, 0.120, 0.014, 0.007, "front_rubber_pad"),
        origin=Origin(xyz=(BASE_BODY_X / 2.0 - 0.015, 0.0, 0.007)),
        material=rubber_black,
        name="front_rubber_pad",
    )

    # Hook hinge lugs are real visible parent faces for the fold-out hook pivot.
    hook_hinge_x = -BASE_BODY_X / 2.0 - 0.008
    hook_hinge_z = BASE_BODY_Z - 0.008
    for i, sign in enumerate((-1.0, 1.0)):
        y = sign * 0.048
        base.visual(
            Box((0.024, 0.018, 0.032)),
            origin=Origin(xyz=(hook_hinge_x, y, hook_hinge_z)),
            material=safety_yellow,
            name=f"hook_lug_{i}",
        )
        base.visual(
            Cylinder(radius=0.006, length=0.020),
            origin=Origin(xyz=(hook_hinge_x, sign * 0.063, hook_hinge_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"hook_pin_cap_{i}",
        )

    # Two short yoke cheeks rise directly from the molded top saddle to the
    # same side pivot bosses on the flood head. Kept compact for handheld use.
    yoke_y = HEAD_W / 2.0 + 0.018
    for i, sign in enumerate((-1.0, 1.0)):
        y = sign * yoke_y
        yoke_pts = [
            (0.0, sign * 0.105, BASE_BODY_Z + 0.004),
            (0.0, sign * 0.118, BASE_BODY_Z + 0.044),
            (0.0, y, PIVOT_Z),
        ]
        base.visual(
            _tube(yoke_pts, UPRIGHT_R, f"yoke_arm_{i}", segments=20),
            material=safety_yellow,
            name=f"yoke_arm_{i}",
        )
        base.visual(
            Box((0.026, 0.034, 0.036)),
            origin=Origin(xyz=(0.0, sign * 0.105, BASE_BODY_Z + 0.018)),
            material=safety_yellow,
            name=f"yoke_socket_{i}",
        )
        base.visual(
            Cylinder(radius=0.014, length=0.012),
            origin=Origin(xyz=(0.0, sign * (yoke_y + 0.006), PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=housing_black,
            name=f"tilt_knob_{i}",
        )

    # Fold-out hanging hook (child). Frame origin is the hinge pin on the rear
    # top face; q=0 tucks the hook down along the back, positive q swings it up.
    hook = model.part("hanging_hook")
    hook.visual(
        Cylinder(radius=0.006, length=0.108),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hook_hinge_barrel",
    )
    hook_pts = [
        (0.000, -0.030, 0.000),
        (-0.035, -0.036, -0.010),
        (-0.095, 0.000, -0.076),
        (-0.035, 0.036, -0.010),
        (0.000, 0.030, 0.000),
    ]
    hook.visual(
        _tube(hook_pts, 0.0055, "folding_hook", segments=18),
        material=steel,
        name="folding_hook",
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
    # ARTICULATIONS -- the flood head keeps the original side-pivot tilt, and
    # the compact base adds a rear fold-out hanging hook on a real hinge pin.
    # =====================================================================
    model.articulation(
        "base_to_hook",
        ArticulationType.REVOLUTE,
        parent=base,
        child=hook,
        origin=Origin(xyz=(hook_hinge_x, 0.0, hook_hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.5, lower=0.0, upper=1.65),
    )

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.70, upper=0.70),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("handheld_base")
    hook = object_model.get_part("hanging_hook")
    head = object_model.get_part("light_head")
    tilt = object_model.get_articulation("base_to_head")
    hook_hinge = object_model.get_articulation("base_to_hook")

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
    ctx.check(
        "hook hinge is revolute",
        hook_hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"got {hook_hinge.articulation_type}",
    )
    hook_ax = tuple(round(a, 6) for a in hook_hinge.axis)
    ctx.check(
        "hook hinge axis is left-right",
        abs(hook_ax[1]) == 1.0 and hook_ax[0] == 0.0 and hook_ax[2] == 0.0,
        details=f"axis={hook_ax}",
    )

    # ---- Hero geometry present -----------------------------------------
    glass = head.get_visual("led_glass_panel")
    handle = head.get_visual("carry_handle")
    grip = head.get_visual("handle_grip")
    battery = head.get_visual("battery_pack")
    base_shell = base.get_visual("base_shell")
    rear_foot = base.get_visual("rear_stand_foot")
    hook_curve = hook.get_visual("folding_hook")
    ctx.check("glass LED panel present", glass is not None)
    ctx.check("carry handle present", handle is not None)
    ctx.check("handle grip present", grip is not None)
    ctx.check("battery pack present", battery is not None)
    ctx.check("compact molded base present", base_shell is not None)
    ctx.check("rear stand foot present", rear_foot is not None)
    ctx.check("fold-out hanging hook present", hook_curve is not None)

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

    # ---- Compact handheld base replaces the floor stand ----------------
    base_aabb = ctx.part_world_aabb(base)
    if base_aabb is not None:
        mins, maxs = base_aabb
        base_size = tuple(float(maxs[i] - mins[i]) for i in range(3))
        ctx.check(
            "base is compact handheld footprint",
            base_size[0] < 0.23 and base_size[1] < 0.32,
            details=f"base_size={base_size!r}",
        )
    else:
        ctx.fail("base aabb available", "Expected compact base AABB.")

    # Head pivot bosses and hook hinge barrel are captured by visible parent
    # yoke/lug geometry -> intentional local bearing overlaps only.
    ctx.allow_overlap(
        head,
        base,
        elem_a="pivot_boss_pos_y",
        elem_b="yoke_arm_1",
        reason="Pivot boss is captured inside the compact yoke cheek to form the tilt bearing.",
    )
    ctx.allow_overlap(
        head,
        base,
        elem_a="pivot_boss_neg_y",
        elem_b="yoke_arm_0",
        reason="Pivot boss is captured inside the compact yoke cheek to form the tilt bearing.",
    )
    ctx.allow_overlap(
        head,
        base,
        elem_a="pivot_boss_pos_y",
        elem_b="tilt_knob_1",
        reason="Pivot bolt head seats into the pivot boss face.",
    )
    ctx.allow_overlap(
        head,
        base,
        elem_a="pivot_boss_neg_y",
        elem_b="tilt_knob_0",
        reason="Pivot bolt head seats into the pivot boss face.",
    )
    ctx.allow_overlap(
        hook,
        base,
        elem_a="hook_hinge_barrel",
        elem_b="hook_lug_0",
        reason="The fold-out hook hinge barrel is intentionally captured in the molded rear lug.",
    )
    ctx.allow_overlap(
        hook,
        base,
        elem_a="hook_hinge_barrel",
        elem_b="hook_lug_1",
        reason="The fold-out hook hinge barrel is intentionally captured in the molded rear lug.",
    )

    # Pivot boss reaches the compact yoke cheek (the supporting bearing contact).
    ctx.expect_contact(
        head,
        base,
        elem_a="pivot_boss_pos_y",
        elem_b="yoke_arm_1",
        name="pivot boss bears on compact yoke",
    )
    ctx.expect_contact(
        hook,
        base,
        elem_a="hook_hinge_barrel",
        elem_b="hook_lug_0",
        name="hook hinge barrel is seated in rear lug",
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

    # ---- Fold-out hook actually swings on the rear hinge ----------------
    hook_rest = ctx.part_element_world_aabb(hook, elem="folding_hook")
    with ctx.pose({hook_hinge: 1.2}):
        hook_out = ctx.part_element_world_aabb(hook, elem="folding_hook")
    rest_top = hook_rest[1][2]
    out_top = hook_out[1][2]
    ctx.check(
        "hook folds upward from rear hinge",
        out_top > rest_top + 0.04,
        details=f"rest_top_z={rest_top:.4f}, folded_top_z={out_top:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
