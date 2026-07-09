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
# Widespread two-handle faucet, polished chrome on dark deck.
#
# Layout (meters, Z up, spout reaches forward along +Y):
#   - dark deck plate (root) with three chrome pieces mounted on top (z = 0)
#   - center column at x = 0: round tapered cylindrical base (0.07 dia at
#     deck, narrowing to 0.050 dia, 0.08 tall) with a stepped cap ring; a
#     separate spout arm swivels continuously about the vertical axis
#   - valve columns at x = +/-0.15: round tapered cylindrical bases (0.055
#     dia, 0.07 tall), flat cap disk, visible stem collar, slim stem, then
#     cross handle with four spokes and ball ends
#   - hot (red) cap disk on left valve, cold (blue) cap disk on right valve
#
# Articulation:
#   - left_handle_spin: REVOLUTE about vertical, -pi..pi
#   - right_handle_spin: REVOLUTE about vertical, -pi..pi
#   - spout_swivel: CONTINUOUS about vertical axis (full 360 rotation)
# ---------------------------------------------------------------------------

HANDLE_SPREAD_X = 0.150  # valve column centers at +/-0.150 -> 0.30 m spread

# Center column
C_BASE_R_BOT = 0.035   # radius at deck
C_BASE_R_TOP = 0.025   # radius at top of taper
C_BASE_H = 0.080
CAP_RING_R = 0.028
CAP_RING_H = 0.008
CAP_TOP_Z = C_BASE_H + CAP_RING_H  # 0.088

# Spout arm
SPOUT_PIVOT_R = 0.018  # pivot collar radius
SPOUT_PIVOT_H = 0.028  # taller collar acts as neck riser
SPOUT_NECK_R = 0.014   # connecting neck radius
SPOUT_NECK_H = 0.020   # neck from collar top to spout body base
SPOUT_WIDTH = 0.044

# Valve columns
V_BASE_R_BOT = 0.0275  # radius at deck
V_BASE_R_TOP = 0.020   # radius at top
V_BASE_H = 0.070
V_CAP_R = 0.022
V_CAP_H = 0.006
V_COLLAR_R = 0.014
V_COLLAR_H = 0.008
V_STEM_R = 0.006
V_STEM_H = 0.022       # stem length above collar
# Handle joint sits 3mm below stem top so the hub captures the stem
HANDLE_JOINT_Z = V_BASE_H + V_CAP_H + V_COLLAR_H + V_STEM_H - 0.003  # ~0.103

# Hot/cold indicator disks
INDICATOR_R = 0.010
INDICATOR_H = 0.002

# Cross handle
HUB_R = 0.0085
HUB_H = 0.030
SPOKE_R = 0.004
SPOKE_LEN = 0.040
SPOKE_Z = 0.010
BALL_R = 0.006
BALL_C = 0.038  # ball centers -> tip-to-tip ~0.088


def _tapered_cylinder(r_bot: float, r_top: float, height: float) -> cq.Workplane:
    """Tapered cylindrical column, base on z=0, narrowing upward."""
    return (
        cq.Workplane("XY")
        .circle(r_bot)
        .workplane(offset=height)
        .circle(r_top)
        .loft(combine=True)
    )


def _waterfall_spout() -> cq.Workplane:
    """Wide flat-topped spout sweeping forward (+Y) into a waterfall arc.

    Side profile in the YZ plane, extruded across X for flat slab sides.
    Local z=0 is the spout body base (sits on the neck top). The profile
    starts with a flat bottom edge at z=0 centered on the neck, so the
    extruded mesh contacts the neck cylinder for connectivity.
    """
    profile = (
        cq.Workplane("YZ")
        .moveTo(-0.012, 0.000)
        .lineTo(0.012, 0.000)
        .lineTo(0.012, 0.018)
        .spline(
            [(0.058, 0.017), (0.105, 0.010), (0.148, -0.006), (0.170, -0.034)],
            includeCurrent=True,
        )
        .lineTo(0.156, -0.040)
        .spline(
            [(0.138, -0.024), (0.105, -0.008), (0.058, 0.001), (0.012, 0.003)],
            includeCurrent=True,
        )
        .lineTo(-0.012, 0.003)
        .lineTo(-0.012, 0.000)
        .close()
        .extrude(SPOUT_WIDTH)
    )
    return profile.translate((-SPOUT_WIDTH / 2.0, 0.0, 0.0))


def _add_cross_handle(part: Part, chrome: str) -> None:
    """Four-spoke cross handle with ball ends, rotating about local +Z.

    Local frame origin is the handle joint frame: hub bottom at z=0.
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
    """Round tapered valve base with cap, stem collar, and slim bonnet stem."""
    # Tapered cylindrical base
    part.visual(
        mesh_from_cadquery(
            _tapered_cylinder(V_BASE_R_BOT, V_BASE_R_TOP, V_BASE_H),
            f"{part.name}_base",
        ),
        material=chrome,
        name="valve_base",
    )
    # Flat cap disk
    part.visual(
        Cylinder(radius=V_CAP_R, length=V_CAP_H),
        origin=Origin(xyz=(0.0, 0.0, V_BASE_H + V_CAP_H / 2.0)),
        material=chrome,
        name="valve_cap",
    )
    # Visible stem collar
    collar_z0 = V_BASE_H + V_CAP_H
    part.visual(
        Cylinder(radius=V_COLLAR_R, length=V_COLLAR_H),
        origin=Origin(xyz=(0.0, 0.0, collar_z0 + V_COLLAR_H / 2.0)),
        material=chrome,
        name="stem_collar",
    )
    # Slim stem above collar
    stem_z0 = collar_z0 + V_COLLAR_H
    part.visual(
        Cylinder(radius=V_STEM_R, length=V_STEM_H),
        origin=Origin(xyz=(0.0, 0.0, stem_z0 + V_STEM_H / 2.0)),
        material=chrome,
        name="valve_stem",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_two_handle_faucet")

    chrome = model.material("chrome", rgba=(0.88, 0.89, 0.92, 1.0))
    deck_mat = model.material("deck_charcoal", rgba=(0.09, 0.09, 0.10, 1.0))
    hot_mat = model.material("hot_red", rgba=(0.85, 0.12, 0.10, 1.0))
    cold_mat = model.material("cold_blue", rgba=(0.10, 0.25, 0.82, 1.0))

    # --- Dark deck plate (root) ---
    deck = model.part("deck")
    deck.visual(
        Box((0.42, 0.20, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=deck_mat,
        name="deck_plate",
    )

    # --- Center column base (fixed to deck) ---
    spout_column = model.part("spout_column")
    spout_column.visual(
        mesh_from_cadquery(
            _tapered_cylinder(C_BASE_R_BOT, C_BASE_R_TOP, C_BASE_H),
            "center_base",
        ),
        material=chrome.name,
        name="column_base",
    )
    spout_column.visual(
        Cylinder(radius=CAP_RING_R, length=CAP_RING_H),
        origin=Origin(xyz=(0.0, 0.0, C_BASE_H + CAP_RING_H / 2.0)),
        material=chrome.name,
        name="cap_ring",
    )
    model.articulation(
        "deck_to_spout_column",
        ArticulationType.FIXED,
        parent=deck,
        child=spout_column,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # --- Spout arm (swivels continuously about vertical axis) ---
    spout_arm = model.part("spout_arm")
    # Pivot collar that sits on the cap ring
    spout_arm.visual(
        Cylinder(radius=SPOUT_PIVOT_R, length=SPOUT_PIVOT_H),
        origin=Origin(xyz=(0.0, 0.0, SPOUT_PIVOT_H / 2.0)),
        material=chrome.name,
        name="pivot_collar",
    )
    # Neck riser connecting collar to spout body
    neck_z0 = SPOUT_PIVOT_H
    spout_arm.visual(
        Cylinder(radius=SPOUT_NECK_R, length=SPOUT_NECK_H),
        origin=Origin(xyz=(0.0, 0.0, neck_z0 + SPOUT_NECK_H / 2.0)),
        material=chrome.name,
        name="spout_neck",
    )
    # The waterfall spout proper, rooted at the top of the neck
    spout_base_z = neck_z0 + SPOUT_NECK_H
    spout_arm.visual(
        mesh_from_cadquery(_waterfall_spout(), "waterfall_spout"),
        origin=Origin(xyz=(0.0, 0.0, spout_base_z)),
        material=chrome.name,
        name="spout",
    )
    model.articulation(
        "spout_swivel",
        ArticulationType.CONTINUOUS,
        parent=spout_column,
        child=spout_arm,
        origin=Origin(xyz=(0.0, 0.0, CAP_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0),
    )

    # --- Valve columns and cross handles (left = -X, right = +X) ---
    for side, sx in (("left", -1.0), ("right", 1.0)):
        valve = model.part(f"{side}_valve")
        _add_valve_column(valve, chrome.name)

        # Hot/cold indicator disk on the cap (offset outside the collar)
        indicator_mat = hot_mat.name if side == "left" else cold_mat.name
        valve.visual(
            Cylinder(radius=INDICATOR_R, length=INDICATOR_H),
            origin=Origin(xyz=(0.0, 0.018, V_BASE_H + V_CAP_H + INDICATOR_H / 2.0)),
            material=indicator_mat,
            name=f"{side}_indicator",
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
    spout_column = object_model.get_part("spout_column")
    spout_arm = object_model.get_part("spout_arm")
    left_valve = object_model.get_part("left_valve")
    right_valve = object_model.get_part("right_valve")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")
    j_left = object_model.get_articulation("left_handle_spin")
    j_right = object_model.get_articulation("right_handle_spin")
    j_swivel = object_model.get_articulation("spout_swivel")

    # Intentional captured fits: handle hubs over valve stems, pivot collar
    # seated onto the cap ring.
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
    ctx.allow_overlap(
        spout_arm,
        spout_column,
        elem_a="pivot_collar",
        elem_b="cap_ring",
        reason="Spout pivot collar seats onto the cap ring with slight nesting.",
    )

    # --- All three chrome pieces seated on the dark deck, not floating ---
    for piece in (spout_column, left_valve, right_valve):
        ctx.expect_gap(
            piece,
            deck,
            axis="z",
            max_gap=0.001,
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
        spout_column,
        axis="x",
        min_gap=0.12,
        max_gap=0.16,
        name="right valve flanks the spout column",
    )
    ctx.expect_origin_gap(
        spout_column,
        left_valve,
        axis="x",
        min_gap=0.12,
        max_gap=0.16,
        name="left valve flanks the spout column",
    )

    # --- Round bases: verify cylindrical footprint ---
    for valve in (left_valve, right_valve):
        base_aabb = ctx.part_element_world_aabb(valve, elem="valve_base")
        ctx.check(
            f"{valve.name} has a round base about 0.055 m diameter",
            base_aabb is not None
            and 0.050 <= (base_aabb[1][0] - base_aabb[0][0]) <= 0.060
            and 0.050 <= (base_aabb[1][1] - base_aabb[0][1]) <= 0.060,
            details=f"base aabb={base_aabb}",
        )

    col_base_aabb = ctx.part_element_world_aabb(spout_column, elem="column_base")
    ctx.check(
        "center column has a round base about 0.07 m diameter",
        col_base_aabb is not None
        and 0.065 <= (col_base_aabb[1][0] - col_base_aabb[0][0]) <= 0.075
        and 0.065 <= (col_base_aabb[1][1] - col_base_aabb[0][1]) <= 0.075,
        details=f"column base aabb={col_base_aabb}",
    )

    # --- Stem collars present under each handle ---
    for valve in (left_valve, right_valve):
        collar_aabb = ctx.part_element_world_aabb(valve, elem="stem_collar")
        ctx.check(
            f"{valve.name} has a visible stem collar",
            collar_aabb is not None
            and (collar_aabb[1][2] - collar_aabb[0][2]) > 0.004,
            details=f"collar aabb={collar_aabb}",
        )

    # --- Hot/cold indicator disks present ---
    hot_aabb = ctx.part_element_world_aabb(left_valve, elem="left_indicator")
    cold_aabb = ctx.part_element_world_aabb(right_valve, elem="right_indicator")
    ctx.check(
        "hot indicator disk present on left valve",
        hot_aabb is not None and (hot_aabb[1][2] - hot_aabb[0][2]) > 0.001,
        details=f"hot indicator aabb={hot_aabb}",
    )
    ctx.check(
        "cold indicator disk present on right valve",
        cold_aabb is not None and (cold_aabb[1][2] - cold_aabb[0][2]) > 0.001,
        details=f"cold indicator aabb={cold_aabb}",
    )

    # --- Spout arm: forward reach, tip arcs down from peak ---
    spout_aabb = ctx.part_element_world_aabb(spout_arm, elem="spout")
    ctx.check(
        "spout reaches forward from the column",
        spout_aabb is not None and spout_aabb[1][1] > 0.14,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "spout tip arcs down well below the spout peak and stays above the deck",
        spout_aabb is not None
        and 0.05 <= spout_aabb[0][2] <= 0.12
        and spout_aabb[1][2] - spout_aabb[0][2] > 0.03,
        details=f"spout aabb={spout_aabb}",
    )

    # --- Spout swivel is CONTINUOUS about vertical ---
    ctx.check(
        "spout_swivel is a continuous joint",
        j_swivel.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={j_swivel.articulation_type}",
    )
    ctx.check(
        "spout_swivel axis is vertical",
        j_swivel.axis is not None
        and abs(j_swivel.axis[2]) > 0.99,
        details=f"axis={j_swivel.axis}",
    )

    # --- Cross handles: ~0.09 m tip-to-tip, seated over the stems ---
    for handle, valve in ((left_handle, left_valve), (right_handle, right_valve)):
        h_aabb = ctx.part_world_aabb(handle)
        ctx.check(
            f"{handle.name} cross is about 0.09 m tip-to-tip",
            h_aabb is not None and 0.082 <= (h_aabb[1][0] - h_aabb[0][0]) <= 0.096,
            details=f"{handle.name} aabb={h_aabb}",
        )
        ctx.expect_gap(
            handle,
            valve,
            axis="z",
            max_gap=0.001,
            max_penetration=0.005,
            name=f"{handle.name} hub seats over the valve stem",
        )

    # --- Handle joints are revolute with correct limits ---
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

    # --- Decisive pose: left handle spins ---
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

    # --- Decisive pose: spout swivels ---
    spout_rest_center = ctx.part_element_world_aabb(spout_arm, elem="spout")
    with ctx.pose({j_swivel: math.pi / 2.0}):
        spout_posed_center = ctx.part_element_world_aabb(spout_arm, elem="spout")
    ctx.check(
        "spout arm swivels 90 deg about the vertical axis",
        spout_rest_center is not None
        and spout_posed_center is not None
        and abs(spout_posed_center[1][1] - spout_rest_center[1][1]) > 0.05,
        details=f"rest={spout_rest_center}, posed={spout_posed_center}",
    )

    return ctx.report()


object_model = build_object_model()
