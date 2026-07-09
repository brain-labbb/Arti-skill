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
# Art-Deco widespread two-handle faucet, mirror chrome on dark deck.
#
# Layout (meters, Z up, spout extends forward along +Y):
#   - dark deck plate (root) with three chrome pieces mounted on top (z = 0)
#   - center spout column at x = 0: tapered square-pyramid base (0.07 sq at
#     deck -> 0.046 sq at z = 0.08), stepped square cap, short rectangular
#     waterfall channel spout reaching ~0.09 m forward
#   - valve columns at x = +/-0.15: smaller tapered pyramids (0.06 sq, 0.07
#     tall) with square cap and slim stem carrying a four-spoke cross handle,
#     0.09 tip-to-tip with ball ends
#   - narrow dark seam frames at all three deck bases
# Articulation: each cross handle revolute about its vertical stem axis
# (-pi..pi).
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
V_STEM_R = 0.0065
V_STEM_TOP_Z = 0.096
HANDLE_JOINT_Z = 0.093  # hub captures the stem top by 3 mm

# Cross handle
HUB_R = 0.0085
HUB_H = 0.034
SPOKE_R = 0.0042
SPOKE_LEN = 0.040
SPOKE_Z = 0.012
BALL_R = 0.0065
BALL_C = 0.0385  # ball centers -> tip-to-tip = 2*(0.0385+0.0065) = 0.090

# Spout channel
SP_W = 0.048       # outer width (X)
SP_L = 0.090       # body length (Y, forward reach)
SP_H = 0.015       # total height
SP_CD = 0.008      # channel depth from top
SP_WT = 0.004      # wall thickness

# Deck seam
SEAM_W = 0.002     # seam line width
SEAM_H = 0.0015    # seam height


def _pyramid_frustum(base: float, top: float, height: float) -> cq.Workplane:
    """Tapered square-pyramid column, base on z=0, narrowing upward."""
    return (
        cq.Workplane("XY")
        .rect(base, base)
        .workplane(offset=height)
        .rect(top, top)
        .loft(combine=True)
    )


def _waterfall_channel() -> cq.Workplane:
    """Short rectangular waterfall channel spout.

    U-channel open at top and front, closed at back and sides.
    Built with back face at y=0, bottom at z=0, extending along +Y.
    """
    w = SP_W
    l = SP_L
    h = SP_H
    cd = SP_CD
    wt = SP_WT
    ft = h - cd        # floor thickness = 0.007
    cw = w - 2 * wt    # channel inner width = 0.040

    # Outer body centered at origin
    body = cq.Workplane("XY").box(w, l, h)

    # Groove: cuts channel from top, open at front, closed at back
    # Y: from back-wall inner face to past front face (clean cut)
    # Z: from floor top to past body top (clean cut)
    gy = l - wt + 0.001
    gz = cd + 0.001
    groove = (
        cq.Workplane("XY")
        .box(cw, gy, gz)
        .translate((0.0, (wt + 0.001) / 2.0, (ft + 0.001) / 2.0))
    )

    channel = body.cut(groove)

    # Translate: back face to y=0, bottom to z=0
    return channel.translate((0.0, l / 2.0, h / 2.0))


def _base_seam(base_size: float) -> cq.Workplane:
    """Thin rectangular frame showing the deck mounting seam.

    Slightly wraps around the column base (inner rect 0.5 mm smaller
    than the base) to ensure visual connectivity with the column.
    """
    outer = base_size + 2 * SEAM_W
    inner = base_size - 0.001
    return (
        cq.Workplane("XY")
        .rect(outer, outer)
        .rect(inner, inner)
        .extrude(SEAM_H)
    )


def _add_cross_handle(part: Part, chrome: str) -> None:
    """Four-spoke cross handle with ball ends, rotating about local +Z."""
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
    spoke_dirs = [
        ((SPOKE_LEN / 2.0, 0.0, SPOKE_Z), (0.0, math.pi / 2.0, 0.0)),
        ((-SPOKE_LEN / 2.0, 0.0, SPOKE_Z), (0.0, math.pi / 2.0, 0.0)),
        ((0.0, SPOKE_LEN / 2.0, SPOKE_Z), (math.pi / 2.0, 0.0, 0.0)),
        ((0.0, -SPOKE_LEN / 2.0, SPOKE_Z), (math.pi / 2.0, 0.0, 0.0)),
    ]
    for i, (xyz, rpy) in enumerate(spoke_dirs):
        part.visual(
            Cylinder(radius=SPOKE_R, length=SPOKE_LEN),
            origin=Origin(xyz=xyz, rpy=rpy),
            material=chrome,
            name=f"spoke_{i}",
        )
    ball_centers = [
        (BALL_C, 0.0, SPOKE_Z),
        (-BALL_C, 0.0, SPOKE_Z),
        (0.0, BALL_C, SPOKE_Z),
        (0.0, -BALL_C, SPOKE_Z),
    ]
    for i, xyz in enumerate(ball_centers):
        part.visual(
            Sphere(radius=BALL_R),
            origin=Origin(xyz=xyz),
            material=chrome,
            name=f"ball_{i}",
        )


def _add_valve_column(part: Part, chrome: str) -> None:
    """Tapered pyramid valve base with square cap and slim bonnet stem."""
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
    stem_z0 = V_PYR_H + V_CAP_H / 2.0
    part.visual(
        Cylinder(radius=V_STEM_R, length=V_STEM_TOP_Z - stem_z0),
        origin=Origin(xyz=(0.0, 0.0, (stem_z0 + V_STEM_TOP_Z) / 2.0)),
        material=chrome,
        name="valve_stem",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_two_handle_faucet")

    chrome = model.material("chrome", rgba=(0.88, 0.89, 0.92, 1.0))
    deck_mat = model.material("deck_charcoal", rgba=(0.09, 0.09, 0.10, 1.0))
    seam_mat = model.material("seam_dark", rgba=(0.12, 0.12, 0.14, 1.0))

    # --- Dark deck plate (root) ---
    deck = model.part("deck")
    deck.visual(
        Box((0.42, 0.20, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=deck_mat,
        name="deck_plate",
    )

    # --- Center spout column ---
    spout_body = model.part("spout_body")
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
    # Short rectangular waterfall channel spout
    spout_body.visual(
        mesh_from_cadquery(_waterfall_channel(), "waterfall_channel"),
        origin=Origin(xyz=(0.0, -0.010, CAP_TOP_Z - 0.001)),
        material=chrome.name,
        name="spout",
    )
    # Deck seam at center column base
    spout_body.visual(
        mesh_from_cadquery(_base_seam(C_PYR_BASE), "center_seam"),
        material=seam_mat.name,
        name="deck_seam",
    )
    model.articulation(
        "deck_to_spout_body",
        ArticulationType.FIXED,
        parent=deck,
        child=spout_body,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # --- Valve columns and cross handles (left = -X, right = +X) ---
    for side, sx in (("left", -1.0), ("right", 1.0)):
        valve = model.part(f"{side}_valve")
        _add_valve_column(valve, chrome.name)
        # Deck seam at valve base
        valve.visual(
            mesh_from_cadquery(_base_seam(V_PYR_BASE), f"{side}_seam"),
            material=seam_mat.name,
            name="deck_seam",
        )
        model.articulation(
            f"deck_to_{side}_valve",
            ArticulationType.FIXED,
            parent=deck,
            child=valve,
            origin=Origin(xyz=(sx * HANDLE_SPREAD_X, 0.0, 0.0)),
        )

        handle = model.part(f"{side}_handle")
        _add_cross_handle(handle, chrome.name)
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
    left_valve = object_model.get_part("left_valve")
    right_valve = object_model.get_part("right_valve")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")
    j_left = object_model.get_articulation("left_handle_spin")
    j_right = object_model.get_articulation("right_handle_spin")

    # Intentional captured fits: handle hubs over valve stems
    ctx.allow_overlap(
        left_handle,
        left_valve,
        elem_a="hub",
        elem_b="valve_stem",
        reason="Cross-handle hub intentionally captures the valve bonnet stem.",
    )
    ctx.allow_overlap(
        right_handle,
        right_valve,
        elem_a="hub",
        elem_b="valve_stem",
        reason="Cross-handle hub intentionally captures the valve bonnet stem.",
    )

    # --- All three chrome pieces seated on the dark deck ---
    for piece in (spout_body, left_valve, right_valve):
        ctx.expect_gap(
            piece,
            deck,
            axis="z",
            max_gap=0.002,
            max_penetration=0.002,
            name=f"{piece.name} base seated on deck top",
        )
        ctx.expect_within(
            piece,
            deck,
            axes="x",
            margin=0.001,
            name=f"{piece.name} stands within the deck plate",
        )

    # --- Three-piece spread of about 0.30 m ---
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

    # --- Spout is a short rectangular waterfall channel ---
    spout_aabb = ctx.part_element_world_aabb(spout_body, elem="spout")
    ctx.check(
        "spout forward reach < 0.12 m (short channel, not long arc)",
        spout_aabb is not None and spout_aabb[1][1] < 0.12,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "spout forward reach > 0.06 m",
        spout_aabb is not None and spout_aabb[1][1] > 0.06,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "spout channel width about 0.048 m",
        spout_aabb is not None
        and 0.044 <= (spout_aabb[1][0] - spout_aabb[0][0]) <= 0.052,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "spout channel height about 0.015 m (low profile)",
        spout_aabb is not None
        and 0.012 <= (spout_aabb[1][2] - spout_aabb[0][2]) <= 0.018,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "spout channel bottom near cap top level (not arcing down)",
        spout_aabb is not None and 0.090 <= spout_aabb[0][2] <= 0.105,
        details=f"spout aabb={spout_aabb}",
    )

    # --- Narrow seams at all three deck bases ---
    for piece_name in ("spout_body", "left_valve", "right_valve"):
        piece = object_model.get_part(piece_name)
        seam_aabb = ctx.part_element_world_aabb(piece, elem="deck_seam")
        ctx.check(
            f"{piece_name} has visible deck seam",
            seam_aabb is not None,
            details=f"seam aabb={seam_aabb}",
        )
        if seam_aabb is not None:
            ctx.check(
                f"{piece_name} seam is thin (< 3 mm tall)",
                (seam_aabb[1][2] - seam_aabb[0][2]) < 0.003,
                details=f"seam height={seam_aabb[1][2] - seam_aabb[0][2]:.4f}",
            )

    # --- Center pyramid base about 0.07 m square ---
    pyr_aabb = ctx.part_element_world_aabb(spout_body, elem="spout_pyramid")
    ctx.check(
        "center pyramid base is about 0.07 m square at the deck",
        pyr_aabb is not None
        and 0.066 <= (pyr_aabb[1][0] - pyr_aabb[0][0]) <= 0.074
        and 0.066 <= (pyr_aabb[1][1] - pyr_aabb[0][1]) <= 0.074,
        details=f"pyramid aabb={pyr_aabb}",
    )

    # --- Cross handles: 0.09 m tip-to-tip, seated over the stems ---
    for handle, valve in ((left_handle, left_valve), (right_handle, right_valve)):
        h_aabb = ctx.part_world_aabb(handle)
        ctx.check(
            f"{handle.name} cross is about 0.09 m tip-to-tip",
            h_aabb is not None and 0.086 <= (h_aabb[1][0] - h_aabb[0][0]) <= 0.094,
            details=f"{handle.name} aabb={h_aabb}",
        )
        ctx.expect_gap(
            handle,
            valve,
            axis="z",
            max_gap=0.0005,
            max_penetration=0.004,
            name=f"{handle.name} hub seats over the valve stem",
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

    # --- Joint limits: both handles -180..+180 deg ---
    for joint in (j_left, j_right):
        lim = joint.motion_limits
        ctx.check(
            f"{joint.name} range is -180..+180 deg",
            lim is not None
            and lim.lower is not None
            and lim.upper is not None
            and abs(lim.lower + math.pi) < 0.01
            and abs(lim.upper - math.pi) < 0.01,
        )

    # --- Decisive pose: handles spin about vertical stems ---
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
        "left handle spins about its vertical stem axis",
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
        "right handle spins independently about its stem axis",
        rest_right is not None
        and posed_right is not None
        and math.hypot(posed_right[0] - rest_right[0], posed_right[1] - rest_right[1])
        > 0.02,
        details=f"rest={rest_right}, posed={posed_right}",
    )

    return ctx.report()


object_model = build_object_model()
