from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Part,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Variant 30: widespread two-handle faucet sibling.
#
# Three-piece deck-mounted layout (Z up, spout sweeps forward along +Y):
#   - dark deck plate (root) with three chrome pieces mounted on top (z = 0)
#   - center spout column at x = 0: tapered square-pyramid base, stepped cap,
#     flat-topped waterfall spout, oval finial diverter on top
#   - valve columns at x = +/-0.15: smaller tapered pyramids with cap and a
#     SHORT vertical axle carrying a four-spoke cross handle
#   - narrow dark seams at all three deck bases
#   - handles asymmetrically angled but balanced around the spout
#
# Articulation: each cross handle revolute about its short vertical axle
# (-pi..pi); the oval finial is a revolute diverter (-pi/2..pi/2).
# ---------------------------------------------------------------------------

HANDLE_SPREAD_X = 0.150  # valve column centers at +/-0.150 -> 0.30 m spread

# Center column
C_PYR_BASE = 0.070
C_PYR_TOP = 0.046
C_PYR_H = 0.080
CAP1_SIZE = 0.056
CAP1_H = 0.010
CAP2_SIZE = 0.046
CAP2_H = 0.008
CAP_TOP_Z = C_PYR_H + CAP1_H + CAP2_H  # 0.098

# Valve columns
V_PYR_BASE = 0.060
V_PYR_TOP = 0.034
V_PYR_H = 0.070
V_CAP_SIZE = 0.040
V_CAP_H = 0.008
# Short vertical axle: visible stub above the cap
V_STEM_R = 0.0065
V_STEM_TOP_Z = 0.086  # short axle: only 8 mm above cap top
HANDLE_JOINT_Z = 0.084  # hub captures 2 mm of axle top

# Cross handle
HUB_R = 0.0085
HUB_H = 0.024
SPOKE_R = 0.0042
SPOKE_LEN = 0.040
SPOKE_Z = 0.010
BALL_R = 0.0065
BALL_C = 0.0385  # ball centers -> tip-to-tip = 2*(0.0385+0.0065) = 0.090

# Asymmetric rest angles (radians) — balanced around the spout
LEFT_HANDLE_REST_ANGLE = 0.40   # ~23° counterclockwise
RIGHT_HANDLE_REST_ANGLE = -0.65  # ~-37° clockwise (different magnitude = asymmetric)

# Spout
SPOUT_WIDTH = 0.050

# Finial diverter
FINIAL_RX = 0.018
FINIAL_RY = 0.012
FINIAL_RZ = 0.008
FINIAL_STEM_R = 0.0045
FINIAL_CENTER_Z = 0.014

# Seam dimensions
SEAM_WIDTH = 0.002   # 2 mm wide seam frame
SEAM_HEIGHT = 0.001  # 1 mm tall


def _pyramid_frustum(base: float, top: float, height: float) -> cq.Workplane:
    """Tapered square-pyramid column, base on z=0, narrowing upward."""
    return (
        cq.Workplane("XY")
        .rect(base, base)
        .workplane(offset=height)
        .rect(top, top)
        .loft(combine=True)
    )


def _base_seam(base_size: float) -> cq.Workplane:
    """Thin rectangular frame representing a narrow seam at the deck base.

    Sits at z=0, wraps just outside the base perimeter.
    """
    outer = base_size + 2 * SEAM_WIDTH
    inner = base_size
    return (
        cq.Workplane("XY")
        .rect(outer, outer)
        .extrude(SEAM_HEIGHT)
        .faces(">Z")
        .workplane()
        .rect(inner, inner)
        .cutBlind(-SEAM_HEIGHT)
    )


def _waterfall_spout() -> cq.Workplane:
    """Wide flat-topped spout sweeping forward (+Y) into a waterfall arc."""
    profile = (
        cq.Workplane("YZ")
        .moveTo(0.010, 0.062)
        .lineTo(0.010, 0.078)
        .spline(
            [(0.060, 0.077), (0.110, 0.070), (0.150, 0.054), (0.174, 0.030)],
            includeCurrent=True,
        )
        .lineTo(0.160, 0.024)
        .spline(
            [(0.140, 0.040), (0.105, 0.054), (0.060, 0.061), (0.010, 0.062)],
            includeCurrent=True,
        )
        .close()
        .extrude(SPOUT_WIDTH)
    )
    return profile.translate((-SPOUT_WIDTH / 2.0, 0.0, 0.0))


def _oval_finial() -> cq.Shape:
    """Small oval (elliptical-plan) finial button via nonuniform sphere scale."""
    unit = cq.Workplane("XY").sphere(1.0).val()
    mat = cq.Matrix(
        [
            [FINIAL_RX, 0.0, 0.0, 0.0],
            [0.0, FINIAL_RY, 0.0, 0.0],
            [0.0, 0.0, FINIAL_RZ, 0.0],
        ]
    )
    return unit.transformGeometry(mat)


def _add_cross_handle(part: Part, chrome: str, rest_angle: float = 0.0) -> None:
    """Four-spoke cross handle with ball ends, rotated by rest_angle about Z.

    Local frame origin is the handle joint frame: hub bottom at z=0.
    Spoke i=0 points at angle rest_angle from +X in the XY plane.
    """
    part.visual(
        Cylinder(radius=HUB_R, length=HUB_H),
        origin=Origin(xyz=(0.0, 0.0, HUB_H / 2.0)),
        material=chrome,
        name="hub",
    )
    part.visual(
        Sphere(radius=HUB_R),
        origin=Origin(xyz=(0.0, 0.0, HUB_H)),
        material=chrome,
        name="hub_dome",
    )
    # Four spokes at 90° intervals starting from rest_angle.
    # rpy=(0, pi/2, theta) orients a Z-aligned cylinder along direction
    # (cos theta, sin theta, 0) via R = Rz(theta) * Ry(pi/2).
    spoke_thetas = [
        rest_angle,
        math.pi + rest_angle,
        math.pi / 2.0 + rest_angle,
        -math.pi / 2.0 + rest_angle,
    ]
    for i, theta in enumerate(spoke_thetas):
        ct = math.cos(theta)
        st = math.sin(theta)
        cx = (SPOKE_LEN / 2.0) * ct
        cy = (SPOKE_LEN / 2.0) * st
        part.visual(
            Cylinder(radius=SPOKE_R, length=SPOKE_LEN),
            origin=Origin(xyz=(cx, cy, SPOKE_Z), rpy=(0.0, math.pi / 2.0, theta)),
            material=chrome,
            name=f"spoke_{i}",
        )
    for i, theta in enumerate(spoke_thetas):
        ct = math.cos(theta)
        st = math.sin(theta)
        bx = BALL_C * ct
        by = BALL_C * st
        part.visual(
            Sphere(radius=BALL_R),
            origin=Origin(xyz=(bx, by, SPOKE_Z)),
            material=chrome,
            name=f"ball_{i}",
        )


def _add_valve_column(part: Part, chrome: str, seam_mat: str) -> None:
    """Tapered pyramid valve base with square cap, short axle, and base seam."""
    # Narrow seam at deck base
    part.visual(
        mesh_from_cadquery(_base_seam(V_PYR_BASE), f"{part.name}_seam"),
        material=seam_mat,
        name="base_seam",
    )
    part.visual(
        mesh_from_cadquery(
            _pyramid_frustum(V_PYR_BASE, V_PYR_TOP, V_PYR_H),
            f"{part.name}_pyramid",
        ),
        material=chrome,
        name="valve_pyramid",
    )
    part.visual(
        Box((V_CAP_SIZE, V_CAP_SIZE, V_CAP_H)),
        origin=Origin(xyz=(0.0, 0.0, V_PYR_H + V_CAP_H / 2.0)),
        material=chrome,
        name="valve_cap",
    )
    # Short vertical axle above cap
    cap_top = V_PYR_H + V_CAP_H
    axle_len = V_STEM_TOP_Z - cap_top
    part.visual(
        Cylinder(radius=V_STEM_R, length=axle_len),
        origin=Origin(xyz=(0.0, 0.0, cap_top + axle_len / 2.0)),
        material=chrome,
        name="valve_stem",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_two_handle_faucet_v30")

    chrome = model.material("chrome", rgba=(0.88, 0.89, 0.92, 1.0))
    deck_mat = model.material("deck_charcoal", rgba=(0.09, 0.09, 0.10, 1.0))
    seam_mat = model.material("seam_dark", rgba=(0.04, 0.04, 0.05, 1.0))

    # --- Dark deck plate (root) ---
    deck = model.part("deck")
    deck.visual(
        Box((0.42, 0.20, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),  # top face at z = 0
        material=deck_mat,
        name="deck_plate",
    )

    # --- Center spout column ---
    spout_body = model.part("spout_body")
    # Narrow seam at center base
    spout_body.visual(
        mesh_from_cadquery(_base_seam(C_PYR_BASE), "center_seam"),
        material=seam_mat,
        name="base_seam",
    )
    spout_body.visual(
        mesh_from_cadquery(
            _pyramid_frustum(C_PYR_BASE, C_PYR_TOP, C_PYR_H), "center_pyramid"
        ),
        material=chrome.name,
        name="spout_pyramid",
    )
    spout_body.visual(
        Box((CAP1_SIZE, CAP1_SIZE, CAP1_H)),
        origin=Origin(xyz=(0.0, 0.0, C_PYR_H + CAP1_H / 2.0)),
        material=chrome.name,
        name="cap_step_lower",
    )
    spout_body.visual(
        Box((CAP2_SIZE, CAP2_SIZE, CAP2_H)),
        origin=Origin(xyz=(0.0, 0.0, C_PYR_H + CAP1_H + CAP2_H / 2.0)),
        material=chrome.name,
        name="cap_step_upper",
    )
    spout_body.visual(
        mesh_from_cadquery(_waterfall_spout(), "waterfall_spout"),
        material=chrome.name,
        name="spout",
    )
    model.articulation(
        "deck_to_spout_body",
        ArticulationType.FIXED,
        parent=deck,
        child=spout_body,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # --- Oval finial diverter button on the cap ---
    finial = model.part("diverter_finial")
    finial.visual(
        Cylinder(radius=FINIAL_STEM_R, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),  # embeds 2 mm into the cap
        material=chrome.name,
        name="finial_stem",
    )
    finial.visual(
        mesh_from_cadquery(_oval_finial(), "finial_oval"),
        origin=Origin(xyz=(0.0, 0.0, FINIAL_CENTER_Z)),
        material=chrome.name,
        name="finial_oval",
    )
    model.articulation(
        "diverter_spin",
        ArticulationType.REVOLUTE,
        parent=spout_body,
        child=finial,
        origin=Origin(xyz=(0.0, 0.0, CAP_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=2.0, lower=-math.pi / 2.0, upper=math.pi / 2.0
        ),
    )

    # --- Valve columns and cross handles (left = -X, right = +X) ---
    handle_rest_angles = {
        "left": LEFT_HANDLE_REST_ANGLE,
        "right": RIGHT_HANDLE_REST_ANGLE,
    }
    for side, sx in (("left", -1.0), ("right", 1.0)):
        valve = model.part(f"{side}_valve")
        _add_valve_column(valve, chrome.name, seam_mat)
        model.articulation(
            f"deck_to_{side}_valve",
            ArticulationType.FIXED,
            parent=deck,
            child=valve,
            origin=Origin(xyz=(sx * HANDLE_SPREAD_X, 0.0, 0.0)),
        )

        handle = model.part(f"{side}_handle")
        _add_cross_handle(handle, chrome.name, rest_angle=handle_rest_angles[side])
        model.articulation(
            f"{side}_handle_spin",
            ArticulationType.REVOLUTE,
            parent=valve,
            child=handle,
            origin=Origin(xyz=(0.0, 0.0, HANDLE_JOINT_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=5.0, velocity=3.0, lower=-math.pi, upper=math.pi
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    spout_body = object_model.get_part("spout_body")
    finial = object_model.get_part("diverter_finial")
    left_valve = object_model.get_part("left_valve")
    right_valve = object_model.get_part("right_valve")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")
    j_left = object_model.get_articulation("left_handle_spin")
    j_right = object_model.get_articulation("right_handle_spin")
    j_div = object_model.get_articulation("diverter_spin")

    # Intentional captured fits: handle hubs over short axles, finial stem
    # seated into the stepped cap.
    ctx.allow_overlap(
        left_handle,
        left_valve,
        elem_a="hub",
        elem_b="valve_stem",
        reason="Cross-handle hub intentionally captures the short vertical axle.",
    )
    ctx.allow_overlap(
        right_handle,
        right_valve,
        elem_a="hub",
        elem_b="valve_stem",
        reason="Cross-handle hub intentionally captures the short vertical axle.",
    )
    ctx.allow_overlap(
        finial,
        spout_body,
        elem_a="finial_stem",
        elem_b="cap_step_upper",
        reason="Finial stem is intentionally seated 2 mm into the stepped cap.",
    )

    # --- All three chrome pieces seated on the dark deck, not floating ---
    for piece in (spout_body, left_valve, right_valve):
        ctx.expect_gap(
            piece,
            deck,
            axis="z",
            max_gap=0.002,
            max_penetration=0.0005,
            name=f"{piece.name} base seated on deck top",
        )
        ctx.expect_within(
            piece,
            deck,
            axes="x",
            margin=0.001,
            name=f"{piece.name} stands within the deck plate",
        )

    # --- Narrow seams present at all three deck bases ---
    for piece_name in ("spout_body", "left_valve", "right_valve"):
        piece = object_model.get_part(piece_name)
        seam_aabb = ctx.part_element_world_aabb(piece, elem="base_seam")
        ctx.check(
            f"{piece_name} has a narrow seam at the deck base",
            seam_aabb is not None
            and (seam_aabb[1][2] - seam_aabb[0][2]) < 0.003,
            details=f"{piece_name} seam aabb={seam_aabb}",
        )

    # --- Three-piece spread of about 0.30 m, spout centered ---
    ctx.expect_origin_distance(
        left_handle,
        right_handle,
        axes="x",
        min_dist=0.29,
        max_dist=0.31,
        name="handle spread is about 0.30 m",
    )
    ctx.expect_origin_gap(
        right_valve,
        spout_body,
        axis="x",
        min_gap=0.14,
        max_gap=0.16,
        name="right valve flanks the spout column",
    )
    ctx.expect_origin_gap(
        spout_body,
        left_valve,
        axis="x",
        min_gap=0.14,
        max_gap=0.16,
        name="left valve flanks the spout column",
    )

    # --- Waterfall spout: forward reach ~0.18 m, tip arcs down ---
    spout_aabb = ctx.part_element_world_aabb(spout_body, elem="spout")
    ctx.check(
        "spout reaches about 0.18 m forward",
        spout_aabb is not None and 0.16 <= spout_aabb[1][1] <= 0.20,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "spout tip arcs down well below the cap but stays above the deck",
        spout_aabb is not None and 0.01 <= spout_aabb[0][2] <= 0.045,
        details=f"spout aabb={spout_aabb}",
    )
    pyr_aabb = ctx.part_element_world_aabb(spout_body, elem="spout_pyramid")
    ctx.check(
        "center pyramid base is about 0.07 m square at the deck",
        pyr_aabb is not None
        and 0.066 <= (pyr_aabb[1][0] - pyr_aabb[0][0]) <= 0.074
        and 0.066 <= (pyr_aabb[1][1] - pyr_aabb[0][1]) <= 0.074,
        details=f"pyramid aabb={pyr_aabb}",
    )
    cap_aabb = ctx.part_element_world_aabb(spout_body, elem="cap_step_upper")
    ctx.check(
        "stepped cap tops the center pyramid",
        cap_aabb is not None and abs(cap_aabb[1][2] - CAP_TOP_Z) < 0.002,
        details=f"cap aabb={cap_aabb}",
    )

    # --- Short vertical axles: visible axle above cap is under 12 mm ---
    cap_top_z = V_PYR_H + V_CAP_H
    visible_axle = V_STEM_TOP_Z - cap_top_z
    ctx.check(
        "short vertical axle height is under 12 mm above cap",
        0.004 < visible_axle < 0.012,
        details=f"visible axle = {visible_axle:.4f} m",
    )

    # --- Cross handles: ~0.09 m tip-to-tip, seated over the short axles ---
    # Tip-to-tip measured via opposite ball center distance (AABB projection
    # depends on the asymmetric rest angle, so direct span is unreliable).
    for handle, valve in ((left_handle, left_valve), (right_handle, right_valve)):
        b0 = ctx.part_element_world_aabb(handle, elem="ball_0")
        b1 = ctx.part_element_world_aabb(handle, elem="ball_1")
        if b0 is not None and b1 is not None:
            c0 = [
                (b0[0][0] + b0[1][0]) / 2.0,
                (b0[0][1] + b0[1][1]) / 2.0,
                (b0[0][2] + b0[1][2]) / 2.0,
            ]
            c1 = [
                (b1[0][0] + b1[1][0]) / 2.0,
                (b1[0][1] + b1[1][1]) / 2.0,
                (b1[0][2] + b1[1][2]) / 2.0,
            ]
            span = math.sqrt(sum((c0[i] - c1[i]) ** 2 for i in range(3))) + 2.0 * BALL_R
        else:
            span = 0.0
        ctx.check(
            f"{handle.name} cross is about 0.09 m tip-to-tip",
            0.086 <= span <= 0.094,
            details=f"{handle.name} tip-to-tip span={span:.4f}",
        )
        ctx.expect_gap(
            handle,
            valve,
            axis="z",
            max_gap=0.0005,
            max_penetration=0.004,
            name=f"{handle.name} hub seats over the short axle",
        )
        ctx.expect_within(
            handle,
            valve,
            axes="xy",
            inner_elem="hub",
            outer_elem="valve_pyramid",
            margin=0.001,
            name=f"{handle.name} hub centered on its valve column",
        )

    # --- Asymmetric handle angles at rest: left and right differ ---
    left_ball0_rest = ctx.part_element_world_aabb(left_handle, elem="ball_0")
    right_ball0_rest = ctx.part_element_world_aabb(right_handle, elem="ball_0")
    ctx.check(
        "handles are asymmetrically angled at rest (ball_0 positions differ in Y)",
        left_ball0_rest is not None
        and right_ball0_rest is not None,
        details="both ball_0 AABBs must exist",
    )
    if left_ball0_rest is not None and right_ball0_rest is not None:
        # Left ball_0 center Y relative to left valve center
        left_valve_pos = ctx.part_world_position(left_valve)
        right_valve_pos = ctx.part_world_position(right_valve)
        left_y_offset = (
            (left_ball0_rest[0][1] + left_ball0_rest[1][1]) / 2.0
            - left_valve_pos[1]
        )
        right_y_offset = (
            (right_ball0_rest[0][1] + right_ball0_rest[1][1]) / 2.0
            - right_valve_pos[1]
        )
        ctx.check(
            "left and right handle rest angles are asymmetric (different Y offsets)",
            abs(abs(left_y_offset) - abs(right_y_offset)) > 0.005,
            details=f"left_y_offset={left_y_offset:.4f}, right_y_offset={right_y_offset:.4f}",
        )

    # --- Finial button seated on the cap top ---
    ctx.expect_gap(
        finial,
        spout_body,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.003,
        name="finial stem seats into the cap top",
    )
    ctx.expect_within(
        finial,
        spout_body,
        axes="xy",
        inner_elem="finial_oval",
        outer_elem="cap_step_upper",
        margin=0.001,
        name="oval finial centered on the cap",
    )

    # --- Joint limits match the prompt ---
    for joint, lo, hi in ((j_left, -math.pi, math.pi), (j_right, -math.pi, math.pi)):
        lim = joint.motion_limits
        ctx.check(
            f"{joint.name} range is -180..+180 deg",
            lim is not None
            and lim.lower is not None
            and lim.upper is not None
            and abs(lim.lower - lo) < 0.01
            and abs(lim.upper - hi) < 0.01,
        )
    div_lim = j_div.motion_limits
    ctx.check(
        "diverter range is -90..+90 deg",
        div_lim is not None
        and div_lim.lower is not None
        and div_lim.upper is not None
        and abs(div_lim.lower + math.pi / 2.0) < 0.01
        and abs(div_lim.upper - math.pi / 2.0) < 0.01,
    )

    # --- Non-fixed joints exist and are revolute ---
    for joint_name in ("left_handle_spin", "right_handle_spin", "diverter_spin"):
        j = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} is a non-fixed revolute joint",
            j.articulation_type == ArticulationType.REVOLUTE,
            details=f"{joint_name} type={j.articulation_type}",
        )

    # --- Decisive pose checks: handles spin, diverter rotates ---
    def _ball_center(handle: Part) -> tuple[float, float] | None:
        aabb = ctx.part_element_world_aabb(handle, elem="ball_0")
        if aabb is None:
            return None
        return (
            (aabb[0][0] + aabb[1][0]) / 2.0,
            (aabb[0][1] + aabb[1][1]) / 2.0,
        )

    rest_left = _ball_center(left_handle)
    with ctx.pose({j_left: math.pi / 4.0}):
        posed_left = _ball_center(left_handle)
    ctx.check(
        "left handle spins about its short vertical axle",
        rest_left is not None
        and posed_left is not None
        and math.hypot(posed_left[0] - rest_left[0], posed_left[1] - rest_left[1])
        > 0.02,
        details=f"rest={rest_left}, posed={posed_left}",
    )

    rest_right = _ball_center(right_handle)
    with ctx.pose({j_right: -math.pi / 4.0}):
        posed_right = _ball_center(right_handle)
    ctx.check(
        "right handle spins independently about its short axle",
        rest_right is not None
        and posed_right is not None
        and math.hypot(posed_right[0] - rest_right[0], posed_right[1] - rest_right[1])
        > 0.02,
        details=f"rest={rest_right}, posed={posed_right}",
    )

    # Oval finial: long axis along X at rest; rotated 90 deg it swaps to Y.
    oval_rest = ctx.part_element_world_aabb(finial, elem="finial_oval")
    ctx.check(
        "finial oval long axis along X at rest",
        oval_rest is not None
        and (oval_rest[1][0] - oval_rest[0][0])
        > (oval_rest[1][1] - oval_rest[0][1]) + 0.008,
        details=f"oval rest aabb={oval_rest}",
    )
    with ctx.pose({j_div: math.pi / 2.0}):
        oval_posed = ctx.part_element_world_aabb(finial, elem="finial_oval")
    ctx.check(
        "diverter finial rotates 90 deg about the vertical axis",
        oval_posed is not None
        and (oval_posed[1][1] - oval_posed[0][1])
        > (oval_posed[1][0] - oval_posed[0][0]) + 0.008,
        details=f"oval posed aabb={oval_posed}",
    )

    return ctx.report()


object_model = build_object_model()
