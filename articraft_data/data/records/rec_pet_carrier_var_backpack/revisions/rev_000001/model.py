"""Wearable pet backpack carrier (purple fabric, grey mesh panels, backpack harness).

Variant of the soft-sided airline pet carrier converted to a backpack with a
semi-rigid back panel, two padded shoulder straps, sternum clip, and top grab
handle.  Retains the domed mesh-vented top, zip-open top hatch that swings up,
front mesh door that folds down open, large side mesh windows, and light-blue
interior pet pad.

Long axis = X (front door at +X). Ground at z = 0.
Back panel and harness on the -X (rear / wearer-facing) side.
"""

from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
    section_loft,
    tube_from_spline_points,
)

# ---------------------------------------------------------------- dimensions
BODY_L = 0.46  # x
BODY_W = 0.27  # y
WALL_T = 0.012
TUB_TOP = 0.26  # z of tub wall top
FLOOR_T = 0.014

DOME_Z0 = 0.258  # dome bottom (embeds 2 mm into wall tops)
DOME_Z1 = 0.340  # dome flat top
DOME_TOP_HX = 0.16
DOME_TOP_HY = 0.10

# top hatch opening in the dome (world coords)
HATCH_X0, HATCH_X1 = -0.13, 0.07
HATCH_HY = 0.070

# front door opening (in +X end wall)
DOOR_OPEN_HY = 0.105
DOOR_SILL_TOP = 0.036
DOOR_OPEN_TOP = 0.258

# back-panel / harness dimensions
PANEL_T = 0.010          # panel thickness (x)
PANEL_HY = 0.110         # panel half-width (y)
PANEL_HZ = 0.120         # panel half-height (z)
PANEL_CZ = 0.140         # panel center z (world)
PANEL_X = -(BODY_L / 2 + PANEL_T / 2)  # x center of panel in world (-0.235)

# colors
PURPLE = (0.50, 0.36, 0.72, 1.0)
PURPLE_DARK = (0.40, 0.28, 0.60, 1.0)
MESH_GREY = (0.74, 0.74, 0.76, 1.0)
MESH_DARK = (0.42, 0.42, 0.45, 1.0)
TRIM_BLACK = (0.09, 0.09, 0.10, 1.0)
PIPING_WHITE = (0.93, 0.93, 0.94, 1.0)
PAD_BLUE = (0.47, 0.60, 0.70, 1.0)
HARNESS_DARK = (0.20, 0.20, 0.22, 1.0)   # semi-rigid back panel
STRAP_PAD_C = (0.32, 0.32, 0.35, 1.0)    # padded strap fabric
CLIP_SILVER = (0.65, 0.65, 0.68, 1.0)    # sternum clip hardware


def _rect_loop(hx: float, hy: float, z: float, c: float, cx: float = 0.0):
    """Closed chamfered-rectangle loop in a z-plane (octagon)."""
    return [
        (cx - hx + c, -hy, z),
        (cx + hx - c, -hy, z),
        (cx + hx, -hy + c, z),
        (cx + hx, hy - c, z),
        (cx + hx - c, hy, z),
        (cx - hx + c, hy, z),
        (cx - hx, hy - c, z),
        (cx - hx, -hy + c, z),
    ]


def _build_dome_mesh():
    """Solid domed top lofted over the tub, with the hatch opening cut out."""
    dome = section_loft(
        [
            _rect_loop(BODY_L / 2, BODY_W / 2, DOME_Z0, 0.028),
            _rect_loop(BODY_L / 2 - 0.008, BODY_W / 2 - 0.006, 0.300, 0.040),
            _rect_loop(DOME_TOP_HX, DOME_TOP_HY, DOME_Z1, 0.045),
        ]
    )
    hole_cx = (HATCH_X0 + HATCH_X1) / 2.0
    hole_hx = (HATCH_X1 - HATCH_X0) / 2.0
    cutter = section_loft(
        [
            _rect_loop(hole_hx, HATCH_HY, DOME_Z0 - 0.01, 0.012, cx=hole_cx),
            _rect_loop(hole_hx, HATCH_HY, DOME_Z1 + 0.01, 0.012, cx=hole_cx),
        ]
    )
    return boolean_difference(dome, cutter)


def _strap_spline_local(panel_hz: float, sign: float):
    """Spline points for one shoulder strap in panel-local frame.

    The panel-local frame has its origin at the panel center.  The strap
    starts at the top of the panel, arcs outward (-X in local = further
    behind the panel in world) to trace an over-the-shoulder curve, then
    returns to the panel bottom.
    """
    y0 = sign * 0.080
    rear_x = -0.020  # 20 mm behind panel center (15 mm behind rear face)
    return [
        (rear_x, y0, panel_hz - 0.010),
        (rear_x - 0.015, sign * 0.095, panel_hz + 0.020),
        (rear_x - 0.035, sign * 0.108, panel_hz + 0.012),
        (rear_x - 0.045, sign * 0.110, 0.030),
        (rear_x - 0.040, sign * 0.105, -0.020),
        (rear_x - 0.020, sign * 0.092, -(panel_hz - 0.035)),
        (rear_x, y0, -(panel_hz - 0.010)),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wearable_pet_backpack_carrier")

    model.material("purple_fabric", rgba=PURPLE)
    model.material("purple_dark", rgba=PURPLE_DARK)
    model.material("mesh_grey", rgba=MESH_GREY)
    model.material("mesh_dark", rgba=MESH_DARK)
    model.material("trim_black", rgba=TRIM_BLACK)
    model.material("piping_white", rgba=PIPING_WHITE)
    model.material("pad_blue", rgba=PAD_BLUE)
    model.material("harness_dark", rgba=HARNESS_DARK)
    model.material("strap_pad", rgba=STRAP_PAD_C)
    model.material("clip_silver", rgba=CLIP_SILVER)

    # ------------------------------------------------------------ body (root)
    body = model.part("carrier_body")

    # floor
    body.visual(
        Box((BODY_L, BODY_W, FLOOR_T)),
        origin=Origin(xyz=(0.0, 0.0, FLOOR_T / 2)),
        material="purple_dark",
        name="floor_panel",
    )
    # long side walls (fabric)
    wall_h = TUB_TOP - 0.008
    for sign, side in ((1.0, "left"), (-1.0, "right")):
        body.visual(
            Box((BODY_L, WALL_T, wall_h)),
            origin=Origin(xyz=(0.0, sign * (BODY_W / 2 - WALL_T / 2), 0.008 + wall_h / 2)),
            material="purple_fabric",
            name=f"{side}_side_wall",
        )
    # rear end wall
    body.visual(
        Box((WALL_T, BODY_W - 2 * WALL_T, wall_h)),
        origin=Origin(xyz=(-(BODY_L / 2 - WALL_T / 2), 0.0, 0.008 + wall_h / 2)),
        material="purple_fabric",
        name="rear_end_wall",
    )
    # front end: sill + two jambs framing the door opening
    front_x = BODY_L / 2 - WALL_T / 2
    body.visual(
        Box((WALL_T, BODY_W - 2 * WALL_T, DOOR_SILL_TOP - 0.008)),
        origin=Origin(xyz=(front_x, 0.0, (DOOR_SILL_TOP + 0.008) / 2)),
        material="purple_fabric",
        name="front_door_sill",
    )
    jamb_w = BODY_W / 2 - WALL_T - DOOR_OPEN_HY + 0.012
    jamb_h = DOOR_OPEN_TOP - DOOR_SILL_TOP
    for sign, side in ((1.0, "left"), (-1.0, "right")):
        body.visual(
            Box((WALL_T, jamb_w, jamb_h)),
            origin=Origin(
                xyz=(
                    front_x,
                    sign * (BODY_W / 2 - WALL_T - jamb_w / 2 + 0.006),
                    DOOR_SILL_TOP + jamb_h / 2,
                )
            ),
            material="purple_fabric",
            name=f"front_{side}_jamb",
        )

    # domed top with hatch opening
    body.visual(
        mesh_from_geometry(_build_dome_mesh(), "dome_shell"),
        origin=Origin(),
        material="purple_fabric",
        name="dome_top_shell",
    )

    # grey mesh vent strips on both sloped long faces of the dome
    slope_ang = math.atan2(BODY_W / 2 - DOME_TOP_HY, DOME_Z1 - DOME_Z0)
    for sign, side in ((1.0, "left"), (-1.0, "right")):
        y_mid = (BODY_W / 2 + DOME_TOP_HY) / 2
        z_mid = (DOME_Z0 + DOME_Z1) / 2
        body.visual(
            Box((0.27, 0.004, 0.078)),
            origin=Origin(
                xyz=(0.0, sign * (y_mid - 0.004), z_mid),
                rpy=(sign * slope_ang, 0.0, 0.0),
            ),
            material="piping_white",
            name=f"{side}_dome_vent_piping",
        )
        body.visual(
            Box((0.255, 0.004, 0.064)),
            origin=Origin(
                xyz=(0.0, sign * (y_mid - 0.0025), z_mid),
                rpy=(sign * slope_ang, 0.0, 0.0),
            ),
            material="mesh_grey",
            name=f"{side}_dome_vent_mesh",
        )

    # large side mesh windows with black trim frames (both long faces)
    win_cx, win_hx, win_cz, win_hz = 0.03, 0.085, 0.150, 0.080
    for sign, side in ((1.0, "left"), (-1.0, "right")):
        y_face = sign * (BODY_W / 2)
        body.visual(
            Box((2 * win_hx, 0.004, 2 * win_hz)),
            origin=Origin(xyz=(win_cx, y_face - sign * 0.001, win_cz)),
            material="mesh_grey",
            name=f"{side}_window_mesh",
        )
        strips = [
            ((2 * win_hx + 0.036, 0.006, 0.018), (win_cx, win_cz + win_hz + 0.009)),
            ((2 * win_hx + 0.036, 0.006, 0.018), (win_cx, win_cz - win_hz - 0.009)),
            ((0.018, 0.006, 2 * win_hz), (win_cx + win_hx + 0.009, win_cz)),
            ((0.018, 0.006, 2 * win_hz), (win_cx - win_hx - 0.009, win_cz)),
        ]
        for i, (size, (sx, sz)) in enumerate(strips):
            body.visual(
                Box(size),
                origin=Origin(xyz=(sx, y_face, sz)),
                material="trim_black",
                name=f"{side}_window_trim_{i}",
            )
        body.visual(
            Box((0.055, 0.004, 0.016)),
            origin=Origin(xyz=(win_cx, y_face + sign * 0.0005, win_cz - win_hz - 0.032)),
            material="piping_white",
            name=f"{side}_logo_patch",
        )

    # light-blue pet pad on the floor
    body.visual(
        Box((0.38, 0.21, 0.044)),
        origin=Origin(xyz=(-0.01, 0.0, 0.006 + 0.022)),
        material="pad_blue",
        name="pet_pad",
    )

    # ------------------------------------------------- top hatch (swings up)
    hatch = model.part("top_hatch")
    lid_hx = (HATCH_X1 - HATCH_X0) / 2 + 0.0075
    lid_hy = HATCH_HY + 0.0075
    hatch.visual(
        Box((2 * lid_hx - 0.016, 2 * lid_hy - 0.016, 0.004)),
        origin=Origin(xyz=(lid_hx, 0.0, 0.0025)),
        material="mesh_grey",
        name="hatch_mesh_panel",
    )
    lid_strips = [
        ((2 * lid_hx, 0.016, 0.006), (lid_hx, lid_hy - 0.008)),
        ((2 * lid_hx, 0.016, 0.006), (lid_hx, -(lid_hy - 0.008))),
        ((0.016, 2 * lid_hy, 0.006), (0.008, 0.0)),
        ((0.016, 2 * lid_hy, 0.006), (2 * lid_hx - 0.008, 0.0)),
    ]
    for i, (size, (sx, sy)) in enumerate(lid_strips):
        hatch.visual(
            Box(size),
            origin=Origin(xyz=(sx, sy, 0.003)),
            material="trim_black",
            name=f"hatch_trim_{i}",
        )
    hatch.visual(
        Box((0.018, 0.010, 0.005)),
        origin=Origin(xyz=(2 * lid_hx - 0.002, 0.03, 0.0065)),
        material="trim_black",
        name="hatch_zip_pull",
    )

    model.articulation(
        "body_to_top_hatch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=hatch,
        origin=Origin(xyz=(HATCH_X0 - 0.0075, 0.0, DOME_Z1 - 0.0005)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=0.0, upper=2.4),
    )

    # ---------------------------------------------- front door (folds down)
    door = model.part("front_door")
    door_hw = DOOR_OPEN_HY - 0.005
    door_h = 0.214
    door.visual(
        Box((0.005, 2 * door_hw - 0.016, door_h - 0.016)),
        origin=Origin(xyz=(0.0, 0.0, door_h / 2)),
        material="mesh_dark",
        name="door_mesh_panel",
    )
    door_strips = [
        ((0.007, 2 * door_hw, 0.016), (0.0, door_h - 0.008)),
        ((0.007, 2 * door_hw, 0.016), (0.0, 0.008)),
        ((0.007, 0.016, door_h), (door_hw - 0.008, door_h / 2)),
        ((0.007, 0.016, door_h), (-(door_hw - 0.008), door_h / 2)),
    ]
    for i, (size, (sy, sz)) in enumerate(door_strips):
        door.visual(
            Box(size),
            origin=Origin(xyz=(0.001, sy, sz)),
            material="trim_black",
            name=f"door_trim_{i}",
        )
    door.visual(
        Box((0.005, 0.060, 0.020)),
        origin=Origin(xyz=(0.0015, 0.0, door_h * 0.62)),
        material="piping_white",
        name="door_brand_patch",
    )

    model.articulation(
        "body_to_front_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(BODY_L / 2, 0.0, DOOR_SILL_TOP + 0.004)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=0.0, upper=1.6),
    )

    # ----------------------------------------- back panel + harness (FIXED)
    # The back_panel part frame is placed at the panel center via the FIXED
    # articulation origin.  All visuals below use panel-local coordinates.
    panel = model.part("back_panel")

    # semi-rigid back panel (local origin = panel center)
    panel.visual(
        Box((PANEL_T, 2 * PANEL_HY, 2 * PANEL_HZ)),
        origin=Origin(),
        material="harness_dark",
        name="panel_shell",
    )
    # raised foam padding strips on the wearer-facing side (-X face)
    pad_x = -(PANEL_T / 2 + 0.003)
    for z_off in (-0.060, 0.0, 0.060):
        panel.visual(
            Box((0.006, 2 * PANEL_HY - 0.030, 0.038)),
            origin=Origin(xyz=(pad_x, 0.0, z_off)),
            material="strap_pad",
            name="panel_pad_strip",
        )

    # top grab handle (small tube arch above the panel, local coords)
    grab_pts = [
        (0.0, -0.035, PANEL_HZ + 0.002),
        (-0.010, -0.020, PANEL_HZ + 0.028),
        (-0.010, 0.0, PANEL_HZ + 0.032),
        (-0.010, 0.020, PANEL_HZ + 0.028),
        (0.0, 0.035, PANEL_HZ + 0.002),
    ]
    panel.visual(
        mesh_from_geometry(
            tube_from_spline_points(grab_pts, radius=0.005), "grab_handle"
        ),
        origin=Origin(),
        material="harness_dark",
        name="grab_handle",
    )

    # two contoured padded shoulder straps (loop-emitted via shared helper)
    for i, sign in enumerate((-1.0, 1.0)):
        pts = _strap_spline_local(PANEL_HZ, sign)
        panel.visual(
            mesh_from_geometry(
                tube_from_spline_points(pts, radius=0.020), f"shoulder_strap_{i}"
            ),
            origin=Origin(),
            material="strap_pad",
            name=f"shoulder_strap_{i}",
        )

    # sternum clip bridging the two straps at chest height (local coords)
    panel.visual(
        Box((0.018, 0.180, 0.016)),
        origin=Origin(xyz=(-0.052, 0.0, 0.020)),
        material="clip_silver",
        name="sternum_clip",
    )
    # small buckle detail at clip center
    panel.visual(
        Box((0.022, 0.030, 0.020)),
        origin=Origin(xyz=(-0.052, 0.0, 0.020)),
        material="trim_black",
        name="sternum_buckle",
    )

    # FIXED articulation: back_panel bonded to carrier_body rear wall
    # Origin places the panel frame at the rear wall outer face, panel center height.
    model.articulation(
        "body_to_back_panel",
        ArticulationType.FIXED,
        parent=body,
        child=panel,
        origin=Origin(xyz=(PANEL_X, 0.0, PANEL_CZ)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("carrier_body")
    hatch = object_model.get_part("top_hatch")
    door = object_model.get_part("front_door")
    panel = object_model.get_part("back_panel")
    hatch_joint = object_model.get_articulation("body_to_top_hatch")
    door_joint = object_model.get_articulation("body_to_front_door")

    # closed hatch lid seats on the dome rim (0.5 mm zip-seam embed)
    ctx.allow_overlap(
        hatch,
        body,
        reason="closed hatch lid seats on the dome rim around the opening (0.5 mm zip-seam embed)",
    )

    # ---- back_panel harness: structural presence and geometry claims ----
    panel_shell_aabb = ctx.part_element_world_aabb(panel, elem="panel_shell")
    ctx.check(
        "back_panel_bonded_to_rear_wall",
        panel_shell_aabb is not None
        and panel_shell_aabb[1][0] <= -(BODY_L / 2) + 0.002
        and panel_shell_aabb[0][0] < -(BODY_L / 2) - PANEL_T + 0.002,
        details=f"panel_shell aabb={panel_shell_aabb}",
    )

    # shoulder straps arc behind the panel (wearer-side sweep)
    for i in range(2):
        strap_aabb = ctx.part_element_world_aabb(panel, elem=f"shoulder_strap_{i}")
        ctx.check(
            f"shoulder_strap_{i}_arcs_behind_panel",
            strap_aabb is not None
            and strap_aabb[0][0] < PANEL_X - 0.025,
            details=f"strap_{i} aabb={strap_aabb}",
        )

    # sternum clip spans across center between the two straps
    clip_aabb = ctx.part_element_world_aabb(panel, elem="sternum_clip")
    ctx.check(
        "sternum_clip_bridges_straps",
        clip_aabb is not None
        and clip_aabb[0][1] < -0.05
        and clip_aabb[1][1] > 0.05,
        details=f"clip aabb={clip_aabb}",
    )

    # grab handle sits above the panel top edge
    handle_aabb = ctx.part_element_world_aabb(panel, elem="grab_handle")
    ctx.check(
        "grab_handle_above_panel_top",
        handle_aabb is not None
        and handle_aabb[1][2] > PANEL_CZ + PANEL_HZ + 0.010,
        details=f"handle aabb={handle_aabb}",
    )

    # ---- closed pose: panels seat over their openings ----
    with ctx.pose({hatch_joint: 0.0, door_joint: 0.0}):
        ctx.expect_overlap(hatch, body, axes="xy", min_overlap=0.10)
        ctx.expect_contact(hatch, body)
        lid_aabb = ctx.part_world_aabb(hatch)
        ctx.check(
            "hatch_closed_covers_opening",
            lid_aabb is not None
            and lid_aabb[0][0] < HATCH_X0
            and lid_aabb[1][0] > HATCH_X1
            and lid_aabb[0][1] < -HATCH_HY
            and lid_aabb[1][1] > HATCH_HY,
            details=f"lid aabb={lid_aabb}",
        )
        ctx.check(
            "hatch_closed_rests_on_dome_top",
            lid_aabb is not None and DOME_Z1 - 0.003 < lid_aabb[0][2] < DOME_Z1 + 0.006,
            details=f"lid aabb={lid_aabb}",
        )
        d_aabb = ctx.part_world_aabb(door)
        ctx.check(
            "door_closed_flush_in_opening",
            d_aabb is not None
            and d_aabb[1][0] < BODY_L / 2 + 0.008
            and d_aabb[0][1] > -DOOR_OPEN_HY
            and d_aabb[1][1] < DOOR_OPEN_HY
            and d_aabb[0][2] > DOOR_SILL_TOP - 0.002
            and d_aabb[1][2] < DOOR_OPEN_TOP,
            details=f"door aabb={d_aabb}",
        )

    # ---- open top hatch: free edge swings up above the dome ----
    with ctx.pose({hatch_joint: 2.0}):
        lid_aabb = ctx.part_world_aabb(hatch)
        ctx.check(
            "hatch_open_rises_above_dome",
            lid_aabb is not None and lid_aabb[1][2] > DOME_Z1 + 0.10,
            details=f"open lid aabb={lid_aabb}",
        )

    # ---- open front door: panel folds outward past the front wall ----
    with ctx.pose({door_joint: 1.5}):
        d_aabb = ctx.part_world_aabb(door)
        ctx.check(
            "door_open_folds_outward",
            d_aabb is not None and d_aabb[1][0] > BODY_L / 2 + 0.15,
            details=f"open door aabb={d_aabb}",
        )
        ctx.check(
            "door_open_drops_low",
            d_aabb is not None and d_aabb[1][2] < DOOR_OPEN_TOP - 0.10,
            details=f"open door aabb={d_aabb}",
        )

    # ---- interior pad sits inside, above the floor ----
    pad_aabb = ctx.part_element_world_aabb(body, elem="pet_pad")
    ctx.check(
        "pet_pad_inside_on_floor",
        pad_aabb is not None
        and pad_aabb[0][2] < FLOOR_T
        and pad_aabb[1][2] > DOOR_SILL_TOP
        and pad_aabb[1][0] < BODY_L / 2 - WALL_T,
        details=f"pad aabb={pad_aabb}",
    )

    # articulation origins must sit on real geometry
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)

    return ctx.report()


object_model = build_object_model()
