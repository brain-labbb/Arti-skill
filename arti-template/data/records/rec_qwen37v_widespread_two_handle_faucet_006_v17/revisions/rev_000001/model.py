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
# Widespread two-handle faucet with high swan-neck spout and hinged aerator.
# Variant 17 of Art-Deco roman tub faucet.
#
# Layout (meters, Z up, spout sweeps forward along +Y):
#   - dark deck plate (root) with three chrome pieces mounted on top (z = 0)
#   - center spout column at x = 0: tapered square-pyramid base, stepped cap,
#     high curved swan-neck tube arcing forward, hinged aerator at the tip
#   - valve columns at x = +/-0.15: smaller tapered pyramids with visible
#     stem collars and slim stems carrying four-spoke cross handles
#   - underside hex nuts on the deck below each base
# Articulation: each cross handle revolute about vertical stem axis (-pi..pi);
#   aerator pivots downward on horizontal hinge (0..0.6 rad).
# ---------------------------------------------------------------------------

HANDLE_SPREAD_X = 0.150

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
HANDLE_JOINT_Z = 0.093

# Stem collar (visible ring under each handle)
COLLAR_R = 0.013
COLLAR_H = 0.007

# Cross handle
HUB_R = 0.0085
HUB_H = 0.034
SPOKE_R = 0.0042
SPOKE_LEN = 0.040
SPOKE_Z = 0.012
BALL_R = 0.0065
BALL_C = 0.0385

# Swan neck tube
TUBE_R = 0.012

# Aerator
AERATOR_R = 0.014
AERATOR_H = 0.018

# Underside mounting nut
NUT_AF = 0.024
NUT_H = 0.012


def _pyramid_frustum(base: float, top: float, height: float) -> cq.Workplane:
    """Tapered square-pyramid column, base on z=0, narrowing upward."""
    return (
        cq.Workplane("XY")
        .rect(base, base)
        .workplane(offset=height)
        .rect(top, top)
        .loft(combine=True)
    )


def _swan_neck_tube() -> cq.Workplane:
    """High arcing swan-neck tube via sweep along a spline path in YZ plane.

    Multiple collinear points along the vertical rise prevent spline
    overshoot so the tube stays centered above the cap.
    """
    path = cq.Workplane("YZ").spline(
        [
            (0.0, 0.0),
            (0.0, 0.03),
            (0.0, 0.06),
            (0.0, 0.09),
            (0.0, 0.12),
            (0.02, 0.145),
            (0.06, 0.158),
            (0.10, 0.150),
            (0.14, 0.12),
            (0.155, 0.08),
        ]
    )
    profile = cq.Workplane("XY").circle(TUBE_R)
    tube = profile.sweep(path)
    return tube.translate((0, 0, CAP_TOP_Z))


def _hex_prism(across_flats: float, height: float) -> cq.Workplane:
    """Hexagonal prism for mounting nuts."""
    r = across_flats / (2.0 * math.cos(math.radians(30)))
    pts = []
    for i in range(6):
        a = math.radians(60 * i + 30)
        pts.append((r * math.cos(a), r * math.sin(a)))
    wp = cq.Workplane("XY").moveTo(pts[0][0], pts[0][1])
    for pt in pts[1:]:
        wp = wp.lineTo(pt[0], pt[1])
    return wp.close().extrude(height)


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
    for i, xyz in enumerate(
        [
            (BALL_C, 0.0, SPOKE_Z),
            (-BALL_C, 0.0, SPOKE_Z),
            (0.0, BALL_C, SPOKE_Z),
            (0.0, -BALL_C, SPOKE_Z),
        ]
    ):
        part.visual(
            Sphere(radius=BALL_R),
            origin=Origin(xyz=xyz),
            material=chrome,
            name=f"ball_{i}",
        )


def _add_valve_column(part: Part, chrome: str) -> None:
    """Tapered pyramid valve base with square cap, stem collar, and bonnet stem."""
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
    # Visible stem collar above the cap
    collar_z0 = V_PYR_H + V_CAP_H  # 0.078
    part.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_H),
        origin=Origin(xyz=(0.0, 0.0, collar_z0 + COLLAR_H / 2.0)),
        material=chrome,
        name="stem_collar",
    )
    # Bonnet stem through collar up into handle hub
    stem_z0 = V_PYR_H + V_CAP_H / 2.0  # 0.074, rooted inside cap
    part.visual(
        Cylinder(radius=V_STEM_R, length=V_STEM_TOP_Z - stem_z0),
        origin=Origin(xyz=(0.0, 0.0, (stem_z0 + V_STEM_TOP_Z) / 2.0)),
        material=chrome,
        name="valve_stem",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_swan_neck_faucet")

    chrome = model.material("chrome", rgba=(0.88, 0.89, 0.92, 1.0))
    deck_mat = model.material("deck_charcoal", rgba=(0.09, 0.09, 0.10, 1.0))

    # --- Dark deck plate (root) with underside mounting nuts ---
    deck = model.part("deck")
    deck.visual(
        Box((0.42, 0.20, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=deck_mat,
        name="deck_plate",
    )
    # Underside hex nuts embedded 1 mm into deck bottom for connectivity
    nut_z0 = -0.022 - NUT_H + 0.001  # nut top overlaps deck bottom by 1 mm
    for nut_name, nx in (
        ("center_nut", 0.0),
        ("left_nut", -HANDLE_SPREAD_X),
        ("right_nut", HANDLE_SPREAD_X),
    ):
        deck.visual(
            mesh_from_cadquery(_hex_prism(NUT_AF, NUT_H), nut_name),
            origin=Origin(xyz=(nx, 0.0, nut_z0)),
            material=chrome.name,
            name=nut_name,
        )

    # --- Center spout column with swan neck ---
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
    # Swan-neck tube
    spout_body.visual(
        mesh_from_cadquery(_swan_neck_tube(), "swan_neck"),
        material=chrome.name,
        name="spout_neck",
    )

    model.articulation(
        "deck_to_spout_body",
        ArticulationType.FIXED,
        parent=deck,
        child=spout_body,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # --- Aerator at swan-neck tip (hinged pivot) ---
    tip_y = 0.155
    tip_z = CAP_TOP_Z + 0.08  # ≈ 0.178

    aerator = model.part("aerator")
    aerator.visual(
        Cylinder(radius=AERATOR_R, length=AERATOR_H),
        origin=Origin(xyz=(0.0, 0.0, -AERATOR_H / 2.0)),
        material=chrome.name,
        name="aerator_body",
    )
    aerator.visual(
        Cylinder(radius=AERATOR_R + 0.002, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, -AERATOR_H + 0.001)),
        material=deck_mat.name,
        name="aerator_screen",
    )

    model.articulation(
        "spout_to_aerator",
        ArticulationType.REVOLUTE,
        parent=spout_body,
        child=aerator,
        origin=Origin(xyz=(0.0, tip_y, tip_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=2.0, lower=0.0, upper=0.6
        ),
    )

    # --- Valve columns and cross handles ---
    for side, sx in (("left", -1.0), ("right", 1.0)):
        valve = model.part(f"{side}_valve")
        _add_valve_column(valve, chrome.name)
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
    aerator = object_model.get_part("aerator")
    left_valve = object_model.get_part("left_valve")
    right_valve = object_model.get_part("right_valve")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")

    j_left = object_model.get_articulation("left_handle_spin")
    j_right = object_model.get_articulation("right_handle_spin")
    j_aer = object_model.get_articulation("spout_to_aerator")

    # ---- Intentional captured fits ----
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
        spout_body,
        aerator,
        elem_a="spout_neck",
        elem_b="aerator_body",
        reason="Aerator body connects to the swan-neck tube at the hinge point.",
    )

    # ---- Chrome pieces seated on deck (scoped to pyramid base vs deck plate) ----
    for piece, pyr in (
        (spout_body, "spout_pyramid"),
        (left_valve, "valve_pyramid"),
        (right_valve, "valve_pyramid"),
    ):
        ctx.expect_gap(
            piece,
            deck,
            axis="z",
            positive_elem=pyr,
            negative_elem="deck_plate",
            max_gap=0.001,
            max_penetration=0.0005,
            name=f"{piece.name} base seated on deck top",
        )

    # ---- Three-piece spread ~0.30 m ----
    ctx.expect_origin_distance(
        left_handle,
        right_handle,
        axes="x",
        min_dist=0.29,
        max_dist=0.31,
        name="handle spread is about 0.30 m",
    )

    # ---- Swan-neck spout geometry ----
    neck_aabb = ctx.part_element_world_aabb(spout_body, elem="spout_neck")
    ctx.check(
        "swan neck rises above 0.20 m height",
        neck_aabb is not None and neck_aabb[1][2] > 0.20,
        details=f"neck aabb={neck_aabb}",
    )
    ctx.check(
        "swan neck curves forward past 0.10 m from center",
        neck_aabb is not None and neck_aabb[1][1] > 0.10,
        details=f"neck aabb={neck_aabb}",
    )

    # ---- Aerator at spout tip ----
    aer_aabb = ctx.part_world_aabb(aerator)
    ctx.check(
        "aerator hangs below the swan neck tip",
        aer_aabb is not None and aer_aabb[0][2] < 0.17 and aer_aabb[1][2] > 0.10,
        details=f"aerator aabb={aer_aabb}",
    )
    ctx.expect_overlap(
        spout_body,
        aerator,
        axes="xy",
        min_overlap=0.005,
        elem_a="spout_neck",
        elem_b="aerator_body",
        name="aerator connected to spout neck at hinge",
    )

    # ---- Aerator hinge pivot (non-fixed joint) ----
    aer_lim = j_aer.motion_limits
    ctx.check(
        "aerator hinge has valid revolute limits",
        aer_lim is not None
        and aer_lim.lower is not None
        and aer_lim.upper is not None
        and abs(aer_lim.lower) < 0.01
        and 0.4 < aer_lim.upper < 0.8,
    )

    # Pose check: aerator y_max increases when tilted forward
    aer_rest = ctx.part_element_world_aabb(aerator, elem="aerator_body")
    with ctx.pose({j_aer: 0.5}):
        aer_posed = ctx.part_element_world_aabb(aerator, elem="aerator_body")
    ctx.check(
        "aerator pivots forward when hinge is actuated",
        aer_rest is not None
        and aer_posed is not None
        and (aer_posed[1][1] - aer_rest[1][1]) > 0.003,
        details=f"rest_y_max={aer_rest[1][1] if aer_rest else None}, "
        f"posed_y_max={aer_posed[1][1] if aer_posed else None}",
    )

    # ---- Stem collars present on both valves ----
    for valve in (left_valve, right_valve):
        collar_aabb = ctx.part_element_world_aabb(valve, elem="stem_collar")
        ctx.check(
            f"{valve.name} has visible stem collar",
            collar_aabb is not None,
            details=f"collar aabb={collar_aabb}",
        )

    # ---- Underside mounting nuts below deck ----
    for nut_name in ("center_nut", "left_nut", "right_nut"):
        nut_aabb = ctx.part_element_world_aabb(deck, elem=nut_name)
        ctx.check(
            f"{nut_name} visible below deck",
            nut_aabb is not None and nut_aabb[0][2] < -0.020,
            details=f"nut aabb={nut_aabb}",
        )

    # ---- Cross handles ~0.09 m tip-to-tip ----
    for handle in (left_handle, right_handle):
        h_aabb = ctx.part_world_aabb(handle)
        ctx.check(
            f"{handle.name} cross is about 0.09 m tip-to-tip",
            h_aabb is not None and 0.086 <= (h_aabb[1][0] - h_aabb[0][0]) <= 0.094,
            details=f"{handle.name} aabb={h_aabb}",
        )

    # ---- Handle seating over stems ----
    for handle, valve in ((left_handle, left_valve), (right_handle, right_valve)):
        ctx.expect_gap(
            handle,
            valve,
            axis="z",
            max_gap=0.001,
            max_penetration=0.004,
            name=f"{handle.name} hub seats over the valve stem",
        )

    # ---- Handle joint limits ----
    for joint, lo, hi in (
        (j_left, -math.pi, math.pi),
        (j_right, -math.pi, math.pi),
    ):
        lim = joint.motion_limits
        ctx.check(
            f"{joint.name} range is -180..+180 deg",
            lim is not None
            and lim.lower is not None
            and lim.upper is not None
            and abs(lim.lower - lo) < 0.01
            and abs(lim.upper - hi) < 0.01,
        )

    # ---- Decisive pose: left handle spins ----
    def _ball_center(handle: Part):
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
        and math.hypot(
            posed_left[0] - rest_left[0], posed_left[1] - rest_left[1]
        )
        > 0.02,
        details=f"rest={rest_left}, posed={posed_left}",
    )

    return ctx.report()


object_model = build_object_model()
