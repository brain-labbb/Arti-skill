from __future__ import annotations

# Portable rechargeable LED flood / work light.
# A yellow triangulated folding A-frame stand carries a black rectangular LED
# flood head that tilts up and down on a side pivot. The head has a glass LED
# panel face, a black bezel, a yellow battery pack on the back, and a yellow
# U-shaped carry handle with a black rubber grip. The primary mechanism is the
# head tilt (REVOLUTE); the two A-frame legs also fold on visible apex pins.

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

# Folding A-frame footprint
BASE_LEN = 0.285  # front-to-rear spread between the two folding feet (X)
BASE_SPAN = 0.230  # width of each transverse workbench foot (Y)
APEX_Z = 0.060  # height of the leg hinge/apex bracket
UPRIGHT_H = 0.200  # retained head pivot height, raised 1/3 for tilt clearance

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
    # STAND FRAME (root) -- triangulated folding A-frame apex/yoke
    # =====================================================================
    stand = model.part("stand_frame")

    half_span = BASE_SPAN / 2.0
    half_len = BASE_LEN / 2.0

    # The old flat H-base is replaced by a compact apex bracket.  A low
    # transverse hinge spine ties the two side yoke plates together below the
    # battery pack, while the side yokes rise outside the housing to carry the
    # existing tilt pivot.
    upright_top_y = HEAD_W / 2.0 + 0.018
    pin_outer_y = upright_top_y + 0.030
    cross_pts = [(0.0, -pin_outer_y, APEX_Z), (0.0, 0.0, APEX_Z), (0.0, pin_outer_y, APEX_Z)]
    stand.visual(
        _tube(cross_pts, TUBE_R, "apex_crossbar"),
        material=safety_yellow,
        name="apex_crossbar",
    )

    # Front and rear hinge pins for the two folding legs.  The pins are real
    # visible parent hardware; the leg barrels below are jointed on these axes.
    hinge_x = 0.030
    for i, sx in enumerate((+1.0, -1.0)):
        stand.visual(
            _tube(
                [
                    (0.0, -pin_outer_y, APEX_Z),
                    (sx * hinge_x, -pin_outer_y, APEX_Z),
                    (sx * hinge_x, pin_outer_y, APEX_Z),
                    (0.0, pin_outer_y, APEX_Z),
                ],
                0.008,
                f"leg_pin_{i}",
                segments=16,
            ),
            material=steel,
            name=f"leg_pin_{i}",
        )

    # Short triangulated side yokes: these are the apex bracket cheeks that
    # carry the existing head tilt pivot, not ground-reaching uprights.
    y = upright_top_y
    stand.visual(
        _tube([(0.0, y, APEX_Z), (0.008, y, (APEX_Z + PIVOT_Z) * 0.55), (0.0, y, PIVOT_Z)], UPRIGHT_R, "pivot_yoke_pos_y"),
        material=safety_yellow,
        name="pivot_yoke_pos_y",
    )
    gusset_h = PIVOT_Z - APEX_Z - 0.020
    stand.visual(
        Box((0.010, 0.006, gusset_h)),
        origin=Origin(xyz=(0.004, y, APEX_Z + gusset_h / 2.0)),
        material=safety_yellow,
        name="yoke_gusset_pos_y",
    )
    stand.visual(
        Cylinder(radius=0.013, length=0.010),
        origin=Origin(xyz=(0.0, upright_top_y + 0.006, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=housing_black,
        name="pivot_knob_pos_y",
    )

    y = -upright_top_y
    stand.visual(
        _tube([(0.0, y, APEX_Z), (0.008, y, (APEX_Z + PIVOT_Z) * 0.55), (0.0, y, PIVOT_Z)], UPRIGHT_R, "pivot_yoke_neg_y"),
        material=safety_yellow,
        name="pivot_yoke_neg_y",
    )
    stand.visual(
        Box((0.010, 0.006, gusset_h)),
        origin=Origin(xyz=(0.004, y, APEX_Z + gusset_h / 2.0)),
        material=safety_yellow,
        name="yoke_gusset_neg_y",
    )
    stand.visual(
        Cylinder(radius=0.013, length=0.010),
        origin=Origin(xyz=(0.0, -upright_top_y - 0.006, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=housing_black,
        name="pivot_knob_neg_y",
    )

    # =====================================================================
    # FOLDING LEGS (jointed children) -- two splayed hinged A-frame legs
    # =====================================================================
    # Each repeated leg is emitted by the same geometry policy: a yellow hinge
    # barrel at the apex pin, two diagonal side struts, a transverse foot tube,
    # and black rubber sleeves at the workbench contact ends.
    leg_parts = []
    for i, sx in enumerate((+1.0, -1.0)):
        leg = model.part(f"folding_leg_{i}")
        leg_parts.append((leg, sx))

        leg.visual(
            Cylinder(radius=0.014, length=2.0 * upright_top_y + 0.012),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=safety_yellow,
            name="hinge_barrel",
        )

        local_foot_x = sx * (half_len - 0.014) - sx * hinge_x
        local_foot_z = TUBE_R - APEX_Z
        strut_root_z = -0.019
        for j, sy in enumerate((+1.0, -1.0)):
            strut_pts = [
                (0.0, sy * (upright_top_y - 0.020), strut_root_z),
                (0.50 * local_foot_x, sy * (half_span - 0.030), strut_root_z + 0.50 * (local_foot_z - strut_root_z)),
                (local_foot_x, sy * (half_span - 0.018), local_foot_z),
            ]
            leg.visual(
                _tube(strut_pts, TUBE_R * 0.78, f"leg_{i}_strut_{j}"),
                material=safety_yellow,
                name=f"side_strut_{j}",
            )

        foot_pts = [
            (local_foot_x, -half_span + 0.012, local_foot_z),
            (local_foot_x, 0.0, local_foot_z),
            (local_foot_x, half_span - 0.012, local_foot_z),
        ]
        leg.visual(
            _tube(foot_pts, TUBE_R, f"leg_{i}_foot_bar"),
            material=safety_yellow,
            name="foot_bar",
        )
        for j, sy in enumerate((+1.0, -1.0)):
            leg.visual(
                Cylinder(radius=FOOT_R, length=0.042),
                origin=Origin(
                    xyz=(local_foot_x, sy * (half_span - 0.008), local_foot_z),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=rubber_black,
                name=f"rubber_foot_{j}",
            )

    for i, (leg, sx) in enumerate(leg_parts):
        model.articulation(
            f"stand_to_leg_{i}",
            ArticulationType.REVOLUTE,
            parent=stand,
            child=leg,
            origin=Origin(xyz=(sx * hinge_x, 0.0, APEX_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=-0.35, upper=0.55),
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
    head.visual(
        Cylinder(radius=PIVOT_R, length=boss_len),
        origin=Origin(xyz=(0.0, boss_y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_boss_pos_y",
    )
    head.visual(
        Cylinder(radius=PIVOT_R, length=boss_len),
        origin=Origin(xyz=(0.0, -boss_y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_boss_neg_y",
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
    legs = [object_model.get_part(f"folding_leg_{i}") for i in range(2)]
    tilt = object_model.get_articulation("stand_to_head")
    leg_joints = [object_model.get_articulation(f"stand_to_leg_{i}") for i in range(2)]

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

    # ---- Changed stand: triangulated folding A-frame --------------------
    ctx.check("apex crossbar present", stand.get_visual("apex_crossbar") is not None)
    ctx.check("side yoke present", stand.get_visual("pivot_yoke_pos_y") is not None and stand.get_visual("pivot_yoke_neg_y") is not None)

    for i, leg in enumerate(legs):
        ctx.check(f"folding_leg_{i} present", leg is not None)
        ctx.check(f"folding_leg_{i} hinge barrel present", leg.get_visual("hinge_barrel") is not None)
        ctx.check(f"folding_leg_{i} foot bar present", leg.get_visual("foot_bar") is not None)
        ctx.check(
            f"folding_leg_{i} joint is revolute",
            leg_joints[i].articulation_type == ArticulationType.REVOLUTE,
            details=f"got {leg_joints[i].articulation_type}",
        )
        leg_axis = tuple(round(a, 6) for a in leg_joints[i].axis)
        ctx.check(
            f"folding_leg_{i} folds about a transverse pin",
            abs(leg_axis[1]) == 1.0 and leg_axis[0] == 0.0 and leg_axis[2] == 0.0,
            details=f"axis={leg_axis}",
        )

        # Captured hinge pin/barrel is an intentional local overlap at the
        # actual visible parent pin, proving that the joint is mounted on the
        # bracket face rather than on an invisible anchor.
        ctx.allow_overlap(
            stand,
            leg,
            elem_a=f"leg_pin_{i}",
            elem_b="hinge_barrel",
            reason="The folding leg barrel is intentionally captured around the visible apex hinge pin.",
        )
        ctx.expect_overlap(
            stand,
            leg,
            axes="y",
            elem_a=f"leg_pin_{i}",
            elem_b="hinge_barrel",
            min_overlap=0.10,
            name=f"folding_leg_{i} hinge barrel spans the apex pin",
        )

    # Head must overlap the stand footprint and the pivot bosses must be
    # captured by the uprights -> intentional nested fit.
    ctx.allow_overlap(
        head,
        stand,
        elem_a="pivot_boss_pos_y",
        elem_b="pivot_yoke_pos_y",
        reason="Pivot boss is captured inside the A-frame yoke top to form the tilt bearing.",
    )
    ctx.allow_overlap(
        head,
        stand,
        elem_a="pivot_boss_neg_y",
        elem_b="pivot_yoke_neg_y",
        reason="Pivot boss is captured inside the A-frame yoke top to form the tilt bearing.",
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
        elem_b="pivot_yoke_pos_y",
        name="pivot boss bears on A-frame yoke",
    )

    # The two leg foot bars must splay in opposite X directions from the apex,
    # forming a workbench A-frame footprint rather than the former H-frame.
    foot0 = ctx.part_element_world_aabb(legs[0], elem="foot_bar")
    foot1 = ctx.part_element_world_aabb(legs[1], elem="foot_bar")
    ctx.check(
        "two folding legs splay to opposite sides of the apex",
        foot0[0][0] > 0.08 and foot1[1][0] < -0.08,
        details=f"front_min_x={foot0[0][0]:.4f}, rear_max_x={foot1[1][0]:.4f}",
    )
    ctx.check(
        "A-frame feet sit on the workbench plane",
        foot0[0][2] <= 0.004 and foot1[0][2] <= 0.004,
        details=f"foot0_min_z={foot0[0][2]:.4f}, foot1_min_z={foot1[0][2]:.4f}",
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
