from __future__ import annotations

# Portable rechargeable LED flood / work light.
# A yellow tubular H-frame stand carries a black round COB LED flood head that
# tilts up and down on a side pivot. The head has a circular glass COB face, a
# lathe-formed black housing with a ring bezel, a yellow battery pack on the
# back, and a yellow U-shaped carry handle with a black rubber grip. The primary
# mechanism is the head tilt (REVOLUTE).

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
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
HEAD_W = 0.220  # round head outer diameter (left-right, Y)
HEAD_H = 0.220  # round head outer diameter (vertical/up, X in head frame)
HEAD_D = 0.055  # housing depth (front-to-back, Z in head frame)

HEAD_R = HEAD_W / 2.0
GLASS_INSET = 0.020  # annular bezel border around the glass
GLASS_R = HEAD_R - GLASS_INSET
FRAME_T = 0.010  # bezel frame wall thickness
LED_RING_RADII = (0.000, 0.022, 0.045, 0.066)
LED_RING_COUNTS = (1, 8, 14, 20)

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


def _lathe(profile, name, *, segments=96):
    return mesh_from_geometry(LatheGeometry(profile, segments=segments, closed=True), name)


def _led_disc():
    return Cylinder(radius=0.0033, length=0.0024)


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

    # Round COB housing: a lathe-formed shallow cup with a closed back, sloped
    # cylindrical wall, and open front.  The local Z axis is the optical axis.
    back_z = -HEAD_D / 2.0
    front_z = HEAD_D / 2.0
    housing_profile = [
        (0.000, back_z),
        (0.070, back_z),
        (0.096, back_z + 0.004),
        (HEAD_R - 0.004, -0.010),
        (HEAD_R, 0.004),
        (HEAD_R, front_z - 0.004),
        (HEAD_R - 0.004, front_z),
        (GLASS_R + 0.006, front_z),
        (GLASS_R + 0.006, front_z - 0.010),
        (0.074, back_z + 0.010),
        (0.000, back_z + 0.010),
    ]
    head.visual(
        _lathe(housing_profile, "round_housing_shell"),
        material=housing_black,
        name="round_housing_shell",
    )

    # Slightly proud front trim ring; its inner lip overlaps the diffuser edge
    # like a real captured glass gasket. This replaces the old rectangular bezel.
    bezel_profile = [
        (GLASS_R - 0.004, front_z - 0.004),
        (HEAD_R - 0.006, front_z - 0.004),
        (HEAD_R, front_z - 0.001),
        (HEAD_R, front_z + 0.004),
        (HEAD_R - 0.006, front_z + 0.007),
        (GLASS_R - 0.004, front_z + 0.007),
        (GLASS_R - 0.008, front_z + 0.003),
        (GLASS_R - 0.008, front_z - 0.001),
    ]
    head.visual(
        _lathe(bezel_profile, "ring_bezel"),
        material=housing_black,
        name="ring_bezel",
    )

    # One round glass / diffuser panel seated under the annular ring bezel.
    glass_z = front_z - 0.002
    glass_panel_radius = GLASS_R - 0.001
    head.visual(
        Cylinder(radius=glass_panel_radius, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, glass_z)),
        material=glass_white,
        name="round_glass_panel",
    )

    # Thin circular COB carrier visible through the diffuser.
    head.visual(
        Cylinder(radius=GLASS_R - 0.020, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, glass_z + 0.0035)),
        material=steel,
        name="cob_carrier_disc",
    )

    # COB emitters placed in concentric rings.  They are inline visuals on the
    # head (not jointed decoration parts), named led_i with regular placement.
    led_z = glass_z + 0.0060
    led_index = 0
    for ring_i, count in enumerate(LED_RING_COUNTS):
        radius = LED_RING_RADII[ring_i]
        for i in range(count):
            angle = 0.0 if count == 1 else (2.0 * math.pi * i / count)
            px = radius * math.cos(angle)
            py = radius * math.sin(angle)
            head.visual(
                _led_disc(),
                origin=Origin(xyz=(px, py, led_z)),
                material=led_dot,
                name=f"led_{led_index}",
            )
            led_index += 1

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
    handle_root_x = 0.074
    handle_root_y = 0.082
    handle_pts = [
        (handle_root_x, handle_root_y, -0.004),
        (htop + h_rise * 0.55, handle_root_y * 0.85, h_arc * 0.55),
        (htop + h_rise, 0.0, h_arc),
        (htop + h_rise * 0.55, -handle_root_y * 0.85, h_arc * 0.55),
        (handle_root_x, -handle_root_y, -0.004),
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
    glass = head.get_visual("round_glass_panel")
    housing = head.get_visual("round_housing_shell")
    bezel = head.get_visual("ring_bezel")
    carrier = head.get_visual("cob_carrier_disc")
    handle = head.get_visual("carry_handle")
    grip = head.get_visual("handle_grip")
    battery = head.get_visual("battery_pack")
    ctx.check("round glass COB panel present", glass is not None)
    ctx.check("circular lathe housing present", housing is not None)
    ctx.check("annular ring bezel present", bezel is not None)
    ctx.check("COB carrier disc present", carrier is not None)
    ctx.check("carry handle present", handle is not None)
    ctx.check("handle grip present", grip is not None)
    ctx.check("battery pack present", battery is not None)
    ctx.check(
        "concentric COB LEDs present",
        all(head.get_visual(f"led_{i}") is not None for i in range(sum(LED_RING_COUNTS))),
        details="expected led_0 through led_42 from concentric ring loop",
    )

    # The revised head is round: the face spans equally in head-local/world X
    # and Y at rest, unlike the parent rectangular flood head.
    glass_aabb = ctx.part_element_world_aabb(head, elem="round_glass_panel")
    glass_dx = glass_aabb[1][0] - glass_aabb[0][0]
    glass_dy = glass_aabb[1][1] - glass_aabb[0][1]
    ctx.check(
        "glass panel is circular",
        abs(glass_dx - glass_dy) < 0.002 and 0.16 < glass_dx < 0.19,
        details=f"dx={glass_dx:.4f}, dy={glass_dy:.4f}",
    )
    housing_aabb = ctx.part_element_world_aabb(head, elem="round_housing_shell")
    housing_dx = housing_aabb[1][0] - housing_aabb[0][0]
    housing_dy = housing_aabb[1][1] - housing_aabb[0][1]
    ctx.check(
        "housing is a round lathe disc",
        abs(housing_dx - housing_dy) < 0.003 and housing_dx > glass_dx + 0.030,
        details=f"housing_dx={housing_dx:.4f}, housing_dy={housing_dy:.4f}, glass_dx={glass_dx:.4f}",
    )

    # ---- Placement: handle apex rises above the glass face -------------
    # At rest (q=0) the head frame is aligned with world, so the head-local
    # "up" axis (+X) maps to world +X. The handle arc apex must clear the
    # glass panel along that axis.
    handle_top = ctx.part_element_world_aabb(head, elem="carry_handle")[1][0]
    glass_top = ctx.part_element_world_aabb(head, elem="round_glass_panel")[1][0]
    ctx.check(
        "handle arc rises above the LED face",
        handle_top > glass_top + 0.05,
        details=f"handle_top_x={handle_top:.4f}, glass_top_x={glass_top:.4f}",
    )
    # Handle roots into the curved housing shell (mounted, not floating).
    ctx.expect_contact(
        head,
        head,
        elem_a="carry_handle",
        elem_b="round_housing_shell",
        name="handle rooted into round housing",
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
    rest = ctx.part_element_world_aabb(head, elem="round_glass_panel")
    with ctx.pose({tilt: 0.6}):
        up = ctx.part_element_world_aabb(head, elem="round_glass_panel")
    rest_top = rest[1][2]
    up_top = up[1][2]
    ctx.check(
        "positive tilt raises the LED face",
        up_top > rest_top + 0.01,
        details=f"rest_top_z={rest_top:.4f}, tilted_top_z={up_top:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
