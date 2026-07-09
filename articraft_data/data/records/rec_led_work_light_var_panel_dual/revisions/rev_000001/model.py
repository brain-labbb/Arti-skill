from __future__ import annotations

# Portable rechargeable LED flood / work light.
# A yellow tubular H-frame stand carries a twin black LED flood-head assembly
# that tilts up and down on a shared crossbar. Each side-by-side housing has its
# own glass LED panel, black bezel, fasteners, and LED sub-grid; both housings
# share the same yellow back battery pack and U-shaped carry handle. The primary
# mechanism remains the head tilt (REVOLUTE).

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
HEAD_W = 0.220  # total twin-head array width (left-right, Y)
HEAD_H = 0.175  # face height (vertical when face is upright, X in head frame)
HEAD_D = 0.055  # housing depth (front-to-back, Z in head frame)

PANEL_COUNT = 2
HOUSING_GAP = 0.018
FLOOD_W = (HEAD_W - HOUSING_GAP) / PANEL_COUNT

GLASS_INSET = 0.018  # bezel border around the glass on each side
FRAME_T = 0.010  # bezel frame wall thickness
LED_ROWS = 5
LED_COLS = 4

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


def _emit_flood_housing(
    head,
    index: int,
    center_y: float,
    *,
    housing_black,
    glass_white,
    led_dot,
    steel,
):
    """Emit one flood housing and its LED sub-grid into the shared tilt part."""
    back_z = -HEAD_D / 2.0
    front_z = HEAD_D / 2.0
    wall_z = 0.0
    wall_d = HEAD_D - 0.006
    bezel_t = 0.006
    glass_w = FLOOD_W - 2.0 * GLASS_INSET
    glass_h = HEAD_H - 2.0 * GLASS_INSET
    glass_z = front_z - bezel_t - 0.004
    glass_panel_h = glass_h + 2.0 * 0.004
    glass_panel_w = glass_w + 2.0 * 0.004

    # Hollow black tub: back wall and four side walls, leaving an open front
    # recess for the glass/diffuser.
    head.visual(
        Box((HEAD_H, FLOOD_W, 0.010)),
        origin=Origin(xyz=(0.0, center_y, back_z + 0.005)),
        material=housing_black,
        name=f"housing_back_{index}",
    )
    head.visual(
        Box((FRAME_T, FLOOD_W, wall_d)),
        origin=Origin(xyz=(HEAD_H / 2.0 - FRAME_T / 2.0, center_y, wall_z)),
        material=housing_black,
        name=f"housing_wall_top_{index}",
    )
    head.visual(
        Box((FRAME_T, FLOOD_W, wall_d)),
        origin=Origin(xyz=(-HEAD_H / 2.0 + FRAME_T / 2.0, center_y, wall_z)),
        material=housing_black,
        name=f"housing_wall_bottom_{index}",
    )
    head.visual(
        Box((HEAD_H, FRAME_T, wall_d)),
        origin=Origin(xyz=(0.0, center_y + FLOOD_W / 2.0 - FRAME_T / 2.0, wall_z)),
        material=housing_black,
        name=f"housing_wall_outer_{index}",
    )
    head.visual(
        Box((HEAD_H, FRAME_T, wall_d)),
        origin=Origin(xyz=(0.0, center_y - FLOOD_W / 2.0 + FRAME_T / 2.0, wall_z)),
        material=housing_black,
        name=f"housing_wall_inner_{index}",
    )

    # Raised front bezel/trim ring.
    head.visual(
        Box((GLASS_INSET, FLOOD_W, bezel_t)),
        origin=Origin(xyz=(HEAD_H / 2.0 - GLASS_INSET / 2.0, center_y, front_z - bezel_t / 2.0)),
        material=housing_black,
        name=f"bezel_top_{index}",
    )
    head.visual(
        Box((GLASS_INSET, FLOOD_W, bezel_t)),
        origin=Origin(xyz=(-HEAD_H / 2.0 + GLASS_INSET / 2.0, center_y, front_z - bezel_t / 2.0)),
        material=housing_black,
        name=f"bezel_bottom_{index}",
    )
    head.visual(
        Box((glass_h, GLASS_INSET, bezel_t)),
        origin=Origin(xyz=(0.0, center_y + FLOOD_W / 2.0 - GLASS_INSET / 2.0, front_z - bezel_t / 2.0)),
        material=housing_black,
        name=f"bezel_outer_{index}",
    )
    head.visual(
        Box((glass_h, GLASS_INSET, bezel_t)),
        origin=Origin(xyz=(0.0, center_y - FLOOD_W / 2.0 + GLASS_INSET / 2.0, front_z - bezel_t / 2.0)),
        material=housing_black,
        name=f"bezel_inner_{index}",
    )

    # Glass/diffuser and individual LED chip sub-grid.
    head.visual(
        Box((glass_panel_h, glass_panel_w, 0.010)),
        origin=Origin(xyz=(0.0, center_y, glass_z)),
        material=glass_white,
        name=f"led_glass_panel_{index}",
    )
    led_size = 0.0075
    led_z = glass_z + 0.004
    margin = 0.010
    span_x = glass_h - 2.0 * margin
    span_y = glass_w - 2.0 * margin
    for r in range(LED_ROWS):
        px = -span_x / 2.0 + span_x * r / (LED_ROWS - 1)
        for c in range(LED_COLS):
            py = center_y - span_y / 2.0 + span_y * c / (LED_COLS - 1)
            head.visual(
                Box((led_size, led_size, 0.0025)),
                origin=Origin(xyz=(px, py, led_z)),
                material=led_dot,
                name=f"led_{index}_{r}_{c}",
            )

    # Small silver screw heads at the four bezel corners.
    screw_x = HEAD_H / 2.0 - 0.010
    screw_y = FLOOD_W / 2.0 - 0.010
    for corner, sx, sy in (
        (0, +1.0, +1.0),
        (1, +1.0, -1.0),
        (2, -1.0, +1.0),
        (3, -1.0, -1.0),
    ):
        head.visual(
            Cylinder(radius=0.0032, length=0.0018),
            origin=Origin(xyz=(sx * screw_x, center_y + sy * screw_y, front_z + 0.0009)),
            material=steel,
            name=f"bezel_screw_{index}_{corner}",
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
    # LIGHT HEAD (child) -- two black flood housings on one shared crossbar,
    # plus glass LED faces, battery box, and yellow carry handle.
    # Head frame origin sits at the pivot/crossbar center.
    #   +X = "up" along the face (toward the top edge / handle)
    #   +Y = left-right across the face
    #   +Z = front (out of the glass, the lit direction)
    # =====================================================================
    head = model.part("light_head")

    back_z = -HEAD_D / 2.0
    flood_centers = [
        (i - (PANEL_COUNT - 1) / 2.0) * (FLOOD_W + HOUSING_GAP)
        for i in range(PANEL_COUNT)
    ]

    # One visible crossbar/axle spans behind both housings and carries the
    # tilt bosses at its ends. It passes through the gap between the heads so
    # the twin assembly reads as a single tilting unit rather than two joints.
    head.visual(
        Cylinder(radius=0.008, length=HEAD_W - 0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="shared_crossbar",
    )

    # Side-by-side flood housings, emitted by a regular loop with the same
    # helper and uniform geometry policy. Each gets its own LED sub-grid.
    for i, cy in enumerate(flood_centers):
        _emit_flood_housing(
            head,
            i,
            cy,
            housing_black=housing_black,
            glass_white=glass_white,
            led_dot=led_dot,
            steel=steel,
        )

    # Side pivot bosses: short cylinders on each side of the housing, on the
    # Y axis, captured by the stand upright tops and coaxial with the crossbar.
    boss_len = 0.046
    boss_y = HEAD_W / 2.0 + 0.004
    for sign, tag in ((+1.0, "pos_y"), (-1.0, "neg_y")):
        head.visual(
            Cylinder(radius=PIVOT_R, length=boss_len),
            origin=Origin(xyz=(0.0, sign * boss_y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"pivot_boss_{tag}",
        )

    # Yellow battery pack box mounted on the back, common to the twin housings,
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
    # rubber grip across the top. Rooted into the two outer top walls.
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
    glass_0 = head.get_visual("led_glass_panel_0")
    glass_1 = head.get_visual("led_glass_panel_1")
    crossbar = head.get_visual("shared_crossbar")
    handle = head.get_visual("carry_handle")
    grip = head.get_visual("handle_grip")
    battery = head.get_visual("battery_pack")
    ctx.check("two glass LED panels present", glass_0 is not None and glass_1 is not None)
    ctx.check("shared tilt crossbar present", crossbar is not None)
    ctx.check("carry handle present", handle is not None)
    ctx.check("handle grip present", grip is not None)
    ctx.check("battery pack present", battery is not None)

    ctx.check(
        "one shared tilt joint for both flood housings",
        len(object_model.articulations) == 1 and getattr(tilt.child, "name", tilt.child) == "light_head",
        details=f"joint_count={len(object_model.articulations)} child={getattr(tilt.child, 'name', tilt.child)}",
    )

    # Each repeated housing owns its glass, bezel screws, and LED sub-grid.
    for i in range(PANEL_COUNT):
        ctx.check(f"housing_{i} back present", head.get_visual(f"housing_back_{i}") is not None)
        ctx.check(f"housing_{i} glass present", head.get_visual(f"led_glass_panel_{i}") is not None)
        ctx.check(f"housing_{i} corner screws present", all(head.get_visual(f"bezel_screw_{i}_{c}") is not None for c in range(4)))
        ctx.check(
            f"housing_{i} LED sub-grid present",
            all(head.get_visual(f"led_{i}_{r}_{c}") is not None for r in range(LED_ROWS) for c in range(LED_COLS)),
        )

    glass0_aabb = ctx.part_element_world_aabb(head, elem="led_glass_panel_0")
    glass1_aabb = ctx.part_element_world_aabb(head, elem="led_glass_panel_1")
    glass0_y = (glass0_aabb[0][1] + glass0_aabb[1][1]) / 2.0
    glass1_y = (glass1_aabb[0][1] + glass1_aabb[1][1]) / 2.0
    ctx.check(
        "flood housings are side-by-side",
        abs(glass1_y - glass0_y) > FLOOD_W * 0.8,
        details=f"glass0_y={glass0_y:.4f}, glass1_y={glass1_y:.4f}",
    )

    # ---- Placement: handle apex rises above the glass face -------------
    # At rest (q=0) the head frame is aligned with world, so the head-local
    # "up" axis (+X) maps to world +X. The handle arc apex must clear the
    # glass panel along that axis.
    handle_top = ctx.part_element_world_aabb(head, elem="carry_handle")[1][0]
    glass_top = max(
        ctx.part_element_world_aabb(head, elem="led_glass_panel_0")[1][0],
        ctx.part_element_world_aabb(head, elem="led_glass_panel_1")[1][0],
    )
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
        elem_b="housing_wall_top_0",
        name="handle rooted into housing top",
    )
    ctx.expect_overlap(
        head,
        head,
        axes="y",
        elem_a="shared_crossbar",
        elem_b="led_glass_panel_0",
        min_overlap=0.02,
        name="crossbar spans first flood housing",
    )
    ctx.expect_overlap(
        head,
        head,
        axes="y",
        elem_a="shared_crossbar",
        elem_b="led_glass_panel_1",
        min_overlap=0.02,
        name="crossbar spans second flood housing",
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
    rest = ctx.part_element_world_aabb(head, elem="led_glass_panel_0")
    with ctx.pose({tilt: 0.6}):
        up = ctx.part_element_world_aabb(head, elem="led_glass_panel_0")
    rest_top = rest[1][2]
    up_top = up[1][2]
    ctx.check(
        "positive tilt raises the LED face",
        up_top > rest_top + 0.01,
        details=f"rest_top_z={rest_top:.4f}, tilted_top_z={up_top:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
