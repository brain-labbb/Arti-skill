from __future__ import annotations

# Portable rechargeable LED flood / work light.
# A yellow tabletop jobsite tripod carries a black rectangular LED flood head
# that tilts up and down on a mast-top pivot. The head has a glass LED panel
# face, a black bezel, a yellow battery pack on the back, and a yellow U-shaped
# carry handle with a black rubber grip. The primary mechanism is the head tilt
# (REVOLUTE).

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

# Tabletop jobsite tripod footprint
MAST_X = HEAD_H / 2.0 + 0.035  # mast centerline sits just beyond the head shell
MAST_H = 0.175
HUB_Z = 0.035
TRIPOD_RADIUS = 0.165
TRIPOD_LEG_R = 0.010
TRIPOD_FOOT_R = 0.017

# Head pivot height above the base plane, at the mast-top saddle.
PIVOT_Z = MAST_H


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
    # STAND FRAME (root) -- yellow tabletop tripod + central mast.
    # =====================================================================
    stand = model.part("stand_frame")

    # Low three-way hub at the center of the footprint.
    stand.visual(
        Cylinder(radius=0.028, length=0.028),
        origin=Origin(xyz=(MAST_X, 0.0, HUB_Z)),
        material=safety_yellow,
        name="hub_collar",
    )

    # Three identical splayed tubular legs, emitted with regular radial
    # placement.  The feet stay within a workbench/tabletop footprint.
    for i in range(3):
        angle = math.radians(90.0 + i * 120.0)
        ca = math.cos(angle)
        sa = math.sin(angle)
        leg_pts = [
            (MAST_X + ca * 0.018, sa * 0.018, HUB_Z - 0.004),
            (MAST_X + ca * 0.070, sa * 0.070, 0.024),
            (MAST_X + ca * (TRIPOD_RADIUS - 0.030), sa * (TRIPOD_RADIUS - 0.030), 0.014),
            (MAST_X + ca * TRIPOD_RADIUS, sa * TRIPOD_RADIUS, 0.012),
        ]
        stand.visual(
            _tube(leg_pts, TRIPOD_LEG_R, f"leg_{i}"),
            material=safety_yellow,
            name=f"leg_{i}",
        )
        stand.visual(
            Cylinder(radius=TRIPOD_FOOT_R, length=0.046),
            origin=Origin(
                xyz=(MAST_X + ca * TRIPOD_RADIUS, sa * TRIPOD_RADIUS, 0.012),
                rpy=(0.0, math.pi / 2.0, angle),
            ),
            material=rubber_black,
            name=f"foot_{i}",
        )

    # Central vertical mast rising from the hub.  It terminates at the real
    # tilt axis and carries a visible through-axle for the unchanged head.
    stand.visual(
        Cylinder(radius=UPRIGHT_R, length=PIVOT_Z - HUB_Z),
        origin=Origin(xyz=(MAST_X, 0.0, (PIVOT_Z + HUB_Z) / 2.0)),
        material=safety_yellow,
        name="mast",
    )
    stand.visual(
        Cylinder(radius=0.017, length=0.026),
        origin=Origin(xyz=(MAST_X, 0.0, PIVOT_Z - 0.006)),
        material=safety_yellow,
        name="mast_top_collar",
    )
    yoke_y = HEAD_W / 2.0 + 0.026
    for i, sign in enumerate((-1.0, 1.0)):
        yoke_pts = [
            (MAST_X, 0.0, PIVOT_Z - 0.012),
            (MAST_X * 0.92, sign * 0.055, PIVOT_Z - 0.045),
            (0.030, sign * yoke_y, PIVOT_Z - 0.035),
            (0.0, sign * yoke_y, PIVOT_Z),
        ]
        stand.visual(
            _tube(yoke_pts, 0.007, f"pivot_yoke_{i}", segments=18),
            material=safety_yellow,
            name=f"pivot_yoke_{i}",
        )
    stand.visual(
        Cylinder(radius=0.008, length=HEAD_W + 0.060),
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_axle",
    )
    for i, sign in enumerate((-1.0, 1.0)):
        stand.visual(
            Cylinder(radius=0.013, length=0.010),
            origin=Origin(
                xyz=(0.0, sign * (HEAD_W / 2.0 + 0.035), PIVOT_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=housing_black,
            name=f"pivot_knob_{i}",
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

    # ---- Changed base: tripod hub, three splayed legs, central mast -----
    for elem in ("hub_collar", "mast", "mast_top_collar", "pivot_axle", "pivot_yoke_0", "pivot_yoke_1"):
        ctx.check(f"{elem} present", stand.get_visual(elem) is not None)

    foot_centers = []
    for i in range(3):
        ctx.check(f"leg_{i} present", stand.get_visual(f"leg_{i}") is not None)
        ctx.check(f"foot_{i} present", stand.get_visual(f"foot_{i}") is not None)
        ctx.expect_contact(
            stand,
            stand,
            elem_a=f"leg_{i}",
            elem_b="hub_collar",
            name=f"leg_{i} roots into hub",
        )
        ctx.expect_contact(
            stand,
            stand,
            elem_a=f"foot_{i}",
            elem_b=f"leg_{i}",
            name=f"foot_{i} caps leg",
        )
        aabb = ctx.part_element_world_aabb(stand, elem=f"foot_{i}")
        foot_centers.append(
            (
                (aabb[0][0] + aabb[1][0]) / 2.0,
                (aabb[0][1] + aabb[1][1]) / 2.0,
            )
        )

    radii = [math.hypot(x - MAST_X, y) for x, y in foot_centers]
    ctx.check(
        "three tripod feet are evenly splayed",
        len(radii) == 3 and min(radii) > 0.140 and max(radii) - min(radii) < 0.020,
        details=f"foot_centers={foot_centers}, radii={radii}",
    )
    ctx.expect_contact(
        stand,
        stand,
        elem_a="mast",
        elem_b="hub_collar",
        name="mast rises from hub",
    )
    ctx.expect_contact(
        stand,
        stand,
        elem_a="mast",
        elem_b="pivot_yoke_0",
        name="mast top carries yoke",
    )
    ctx.expect_contact(
        stand,
        stand,
        elem_a="pivot_yoke_0",
        elem_b="pivot_axle",
        name="yoke carries tilt axle",
    )

    # Head pivot bosses ride on the visible mast-top through-axle.  This is a
    # local captured bearing overlap, paired with projected-retention checks.
    for boss in ("pivot_boss_neg_y", "pivot_boss_pos_y"):
        ctx.allow_overlap(
            head,
            stand,
            elem_a=boss,
            elem_b="pivot_axle",
            reason="The head pivot boss is intentionally sleeved over the mast-top axle.",
        )
        ctx.expect_overlap(
            head,
            stand,
            axes="y",
            elem_a=boss,
            elem_b="pivot_axle",
            min_overlap=0.020,
            name=f"{boss} retained on axle",
        )

    for boss, yoke in (("pivot_boss_neg_y", "pivot_yoke_0"), ("pivot_boss_pos_y", "pivot_yoke_1")):
        ctx.allow_overlap(
            head,
            stand,
            elem_a=boss,
            elem_b=yoke,
            reason="The mast-top yoke locally cups the pivot boss as the visible tilt bracket.",
        )
        ctx.expect_overlap(
            head,
            stand,
            axes="y",
            elem_a=boss,
            elem_b=yoke,
            min_overlap=0.006,
            name=f"{boss} cupped by yoke",
        )

    # The axle passes through the unchanged side walls at the molded boss area.
    for wall in ("housing_wall_left", "housing_wall_right"):
        ctx.allow_overlap(
            head,
            stand,
            elem_a=wall,
            elem_b="pivot_axle",
            reason="The through-axle passes through the molded side wall at the pivot boss.",
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
