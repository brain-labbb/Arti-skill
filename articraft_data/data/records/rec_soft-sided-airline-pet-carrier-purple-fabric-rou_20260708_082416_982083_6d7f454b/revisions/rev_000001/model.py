"""Soft-sided airline pet carrier (purple fabric, grey mesh panels).

Reference: purple duffel-style carrier with a domed mesh-vented top, a
zip-open top hatch that swings up, a front mesh door that folds down open,
a large side mesh window, grey webbing straps, and a light-blue pet pad.

Long axis = X (front door at +X). Ground at z = 0.
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

PURPLE = (0.50, 0.36, 0.72, 1.0)
PURPLE_DARK = (0.40, 0.28, 0.60, 1.0)
MESH_GREY = (0.74, 0.74, 0.76, 1.0)
MESH_DARK = (0.42, 0.42, 0.45, 1.0)
TRIM_BLACK = (0.09, 0.09, 0.10, 1.0)
PIPING_WHITE = (0.93, 0.93, 0.94, 1.0)
STRAP_GREY = (0.62, 0.63, 0.65, 1.0)
PAD_BLUE = (0.47, 0.60, 0.70, 1.0)


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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="soft_sided_pet_carrier")

    model.material("purple_fabric", rgba=PURPLE)
    model.material("purple_dark", rgba=PURPLE_DARK)
    model.material("mesh_grey", rgba=MESH_GREY)
    model.material("mesh_dark", rgba=MESH_DARK)
    model.material("trim_black", rgba=TRIM_BLACK)
    model.material("piping_white", rgba=PIPING_WHITE)
    model.material("strap_grey", rgba=STRAP_GREY)
    model.material("pad_blue", rgba=PAD_BLUE)

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

    # grey mesh vent strips on both sloped long faces of the dome (with white piping)
    slope_ang = math.atan2(BODY_W / 2 - DOME_TOP_HY, DOME_Z1 - DOME_Z0)  # about X
    for sign, side in ((1.0, "left"), (-1.0, "right")):
        y_mid = (BODY_W / 2 + DOME_TOP_HY) / 2  # mid-slope
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
        # trim frame strips (top/bottom/front/rear)
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
        # small white logo patch under the window
        body.visual(
            Box((0.055, 0.004, 0.016)),
            origin=Origin(xyz=(win_cx, y_face + sign * 0.0005, win_cz - win_hz - 0.032)),
            material="piping_white",
            name=f"{side}_logo_patch",
        )

    # grey webbing straps down both long faces, flanking the window
    for sign, side in ((1.0, "left"), (-1.0, "right")):
        y_face = sign * (BODY_W / 2)
        for j, sx in enumerate((-0.115, 0.165)):
            body.visual(
                Box((0.034, 0.005, 0.246)),
                origin=Origin(xyz=(sx, y_face, 0.010 + 0.123)),
                material="strap_grey",
                name=f"{side}_strap_{j}",
            )
        # black side-release buckle on the rear strap
        body.visual(
            Box((0.040, 0.011, 0.046)),
            origin=Origin(xyz=(-0.115, y_face + sign * 0.003, 0.205)),
            material="trim_black",
            name=f"{side}_strap_buckle",
        )

    # webbing carry handle arched over the dome, ahead of the hatch
    handle_pts = [
        (0.11, -0.078, 0.330),
        (0.11, -0.050, 0.376),
        (0.11, 0.0, 0.396),
        (0.11, 0.050, 0.376),
        (0.11, 0.078, 0.330),
    ]
    body.visual(
        mesh_from_geometry(
            tube_from_spline_points(handle_pts, radius=0.0065), "carry_handle"
        ),
        origin=Origin(),
        material="strap_grey",
        name="carry_handle_loop",
    )

    # light-blue pet pad on the floor (visible through the open door)
    body.visual(
        Box((0.38, 0.21, 0.044)),
        origin=Origin(xyz=(-0.01, 0.0, 0.006 + 0.022)),
        material="pad_blue",
        name="pet_pad",
    )

    # ------------------------------------------------- top hatch (swings up)
    hatch = model.part("top_hatch")
    lid_hx = (HATCH_X1 - HATCH_X0) / 2 + 0.0075  # 0.1075 -> lid 0.215 long
    lid_hy = HATCH_HY + 0.0075
    # grey mesh field of the lid (extends +X from the rear hinge line)
    hatch.visual(
        Box((2 * lid_hx - 0.016, 2 * lid_hy - 0.016, 0.004)),
        origin=Origin(xyz=(lid_hx, 0.0, 0.0025)),
        material="mesh_grey",
        name="hatch_mesh_panel",
    )
    # black zip-trim border strips
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
    # black zipper pull tab at the free (front) edge
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
        # hinge along the rear edge of the hatch opening, on the dome top face
        origin=Origin(xyz=(HATCH_X0 - 0.0075, 0.0, DOME_Z1 - 0.0005)),
        # lid extends +X from hinge; -Y makes positive q swing the free edge up
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=0.0, upper=2.4),
    )

    # ---------------------------------------------- front door (folds down)
    door = model.part("front_door")
    door_hw = DOOR_OPEN_HY - 0.005  # half width 0.100
    door_h = 0.214  # extends +Z from the bottom hinge in child frame
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
    # white brand patch on the door mesh
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
        # hinge along the bottom edge of the front opening, at the outer face
        origin=Origin(xyz=(BODY_L / 2, 0.0, DOOR_SILL_TOP + 0.004)),
        # door extends +Z; +Y makes positive q fold the panel outward/down (+X)
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=0.0, upper=1.6),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("carrier_body")
    hatch = object_model.get_part("top_hatch")
    door = object_model.get_part("front_door")
    hatch_joint = object_model.get_articulation("body_to_top_hatch")
    door_joint = object_model.get_articulation("body_to_front_door")

    # the closed hatch lid intentionally seats 0.5 mm into the dome rim so the
    # zip-trim border reads as resting on the fabric top
    ctx.allow_overlap(
        hatch,
        body,
        reason="closed hatch lid seats on the dome rim around the opening (0.5 mm zip-seam embed)",
    )

    # ---- closed pose: panels seat over their openings without penetration
    with ctx.pose({hatch_joint: 0.0, door_joint: 0.0}):
        # hatch lid lies flat on the dome top, covering the opening footprint
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
        # closed door sits inside the front opening, roughly flush with the wall
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

    # ---- open top hatch: free edge swings up above the dome
    with ctx.pose({hatch_joint: 2.0}):
        lid_aabb = ctx.part_world_aabb(hatch)
        ctx.check(
            "hatch_open_rises_above_dome",
            lid_aabb is not None and lid_aabb[1][2] > DOME_Z1 + 0.10,
            details=f"open lid aabb={lid_aabb}",
        )

    # ---- open front door: panel folds outward past the front wall and drops
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

    # ---- interior pad really sits inside, above the floor
    pad_aabb = ctx.part_element_world_aabb(body, elem="pet_pad")
    ctx.check(
        "pet_pad_inside_on_floor",
        pad_aabb is not None
        and pad_aabb[0][2] < FLOOR_T
        and pad_aabb[1][2] > DOOR_SILL_TOP
        and pad_aabb[1][0] < BODY_L / 2 - WALL_T,
        details=f"pad aabb={pad_aabb}",
    )

    # hinges must sit on real geometry
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)

    return ctx.report()


object_model = build_object_model()
