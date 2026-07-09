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
# Widespread two-handle deck-mounted faucet with bridge bar and pivoting
# aerator.  Art-Deco style, mirror chrome on dark deck.
#
# Layout (meters, Z up, spout sweeps forward along +Y):
#   - dark deck plate (root)
#   - bridge bar: slim chrome bar spanning the three posts at deck level
#   - center spout column at x = 0: tapered square-pyramid base, stepped cap,
#     flat-topped waterfall spout, oval finial diverter, pivoting aerator
#   - valve columns at x = +/-0.15: smaller tapered pyramids with stem and
#     four-spoke cross handle
#   - narrow seams at all three deck bases
#
# Articulation:
#   - left/right cross handles: revolute about vertical stem (-pi..pi)
#   - diverter finial: revolute about vertical axis (-pi/2..pi/2)
#   - aerator: revolute about horizontal X axis at spout tip (0..0.8 rad)
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

# Cross handle
HUB_R = 0.0085
HUB_H = 0.034
SPOKE_R = 0.0042
SPOKE_LEN = 0.040
SPOKE_Z = 0.012
BALL_R = 0.0065
BALL_C = 0.0385

# Spout
SPOUT_WIDTH = 0.050

# Finial diverter
FINIAL_RX = 0.018
FINIAL_RY = 0.012
FINIAL_RZ = 0.008
FINIAL_STEM_R = 0.0045
FINIAL_CENTER_Z = 0.014

# Bridge bar – slim chrome bar linking the three posts at deck level
BRIDGE_H = 0.006
BRIDGE_DEPTH = 0.012
BRIDGE_SPAN = 0.300  # total X span matching handle spread

# Deck-base seams
SEAM_THICK = 0.0015
SEAM_OVERHANG = 0.003

# Aerator at spout tip
AERATOR_R = 0.011
AERATOR_H = 0.015
SPOUT_TIP_Y = 0.174
SPOUT_TIP_Z = 0.027


def _pyramid_frustum(base: float, top: float, height: float) -> cq.Workplane:
    """Tapered square-pyramid column, base on z=0, narrowing upward."""
    return (
        cq.Workplane("XY")
        .rect(base, base)
        .workplane(offset=height)
        .rect(top, top)
        .loft(combine=True)
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


def _add_valve_column(part: Part, chrome: str, seam_mat: str) -> None:
    """Tapered pyramid valve base with square cap, slim stem, and base seam."""
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
    # Narrow seam at deck base
    seam_size = V_PYR_BASE + 2.0 * SEAM_OVERHANG
    part.visual(
        Box((seam_size, seam_size, SEAM_THICK)),
        origin=Origin(xyz=(0.0, 0.0, SEAM_THICK / 2.0)),
        material=seam_mat,
        name="base_seam",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_two_handle_faucet")

    chrome = model.material("chrome", rgba=(0.88, 0.89, 0.92, 1.0))
    deck_mat = model.material("deck_charcoal", rgba=(0.09, 0.09, 0.10, 1.0))
    seam_mat = model.material("seam_dark", rgba=(0.04, 0.04, 0.05, 1.0))

    # ── Dark deck plate (root) ──────────────────────────────────────────
    deck = model.part("deck")
    deck.visual(
        Box((0.42, 0.20, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=deck_mat,
        name="deck_plate",
    )

    # ── Bridge bar ──────────────────────────────────────────────────────
    bridge = model.part("bridge_bar")
    bridge.visual(
        Box((BRIDGE_SPAN, BRIDGE_DEPTH, BRIDGE_H)),
        origin=Origin(xyz=(0.0, 0.0, BRIDGE_H / 2.0)),
        material=chrome.name,
        name="bridge_span",
    )
    model.articulation(
        "deck_to_bridge",
        ArticulationType.FIXED,
        parent=deck,
        child=bridge,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # ── Center spout column ─────────────────────────────────────────────
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
    spout_body.visual(
        mesh_from_cadquery(_waterfall_spout(), "waterfall_spout"),
        material=chrome.name,
        name="spout",
    )
    # Narrow seam at center deck base
    c_seam_size = C_PYR_BASE + 2.0 * SEAM_OVERHANG
    spout_body.visual(
        Box((c_seam_size, c_seam_size, SEAM_THICK)),
        origin=Origin(xyz=(0.0, 0.0, SEAM_THICK / 2.0)),
        material=seam_mat,
        name="base_seam",
    )
    model.articulation(
        "deck_to_spout_body",
        ArticulationType.FIXED,
        parent=deck,
        child=spout_body,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # ── Oval finial diverter button ─────────────────────────────────────
    finial = model.part("diverter_finial")
    finial.visual(
        Cylinder(radius=FINIAL_STEM_R, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
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

    # ── Pivoting aerator at spout tip ───────────────────────────────────
    aerator = model.part("aerator")
    # Aerator body hangs below the hinge point
    aerator.visual(
        Cylinder(radius=AERATOR_R, length=AERATOR_H),
        origin=Origin(xyz=(0.0, 0.0, -AERATOR_H / 2.0)),
        material=chrome.name,
        name="aerator_body",
    )
    # Small hinge barrel along X
    aerator.visual(
        Cylinder(radius=0.004, length=SPOUT_WIDTH * 0.5),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome.name,
        name="hinge_barrel",
    )
    model.articulation(
        "aerator_pivot",
        ArticulationType.REVOLUTE,
        parent=spout_body,
        child=aerator,
        origin=Origin(xyz=(0.0, SPOUT_TIP_Y, SPOUT_TIP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0, velocity=1.0, lower=0.0, upper=0.80
        ),
    )

    # ── Valve columns and cross handles ─────────────────────────────────
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
    bridge = object_model.get_part("bridge_bar")
    spout_body = object_model.get_part("spout_body")
    finial = object_model.get_part("diverter_finial")
    aerator = object_model.get_part("aerator")
    left_valve = object_model.get_part("left_valve")
    right_valve = object_model.get_part("right_valve")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")
    j_left = object_model.get_articulation("left_handle_spin")
    j_right = object_model.get_articulation("right_handle_spin")
    j_div = object_model.get_articulation("diverter_spin")
    j_aer = object_model.get_articulation("aerator_pivot")

    # ── Intentional captured fits ───────────────────────────────────────
    ctx.allow_overlap(
        left_handle, left_valve,
        elem_a="hub", elem_b="valve_stem",
        reason="Cross-handle hub intentionally captures the valve bonnet stem.",
    )
    ctx.allow_overlap(
        right_handle, right_valve,
        elem_a="hub", elem_b="valve_stem",
        reason="Cross-handle hub intentionally captures the valve bonnet stem.",
    )
    ctx.allow_overlap(
        finial, spout_body,
        elem_a="finial_stem", elem_b="cap_step_upper",
        reason="Finial stem is intentionally seated 2 mm into the stepped cap.",
    )

    # Bridge bar passes through the column bases to read as connecting them
    for col, col_pyr in (
        (spout_body, "spout_pyramid"),
        (left_valve, "valve_pyramid"),
        (right_valve, "valve_pyramid"),
    ):
        ctx.allow_overlap(
            bridge, col,
            elem_a="bridge_span", elem_b=col_pyr,
            reason=f"Bridge bar intentionally embedded in {col.name} base to read as connecting the posts.",
        )

    # Aerator hinge barrel is embedded in the spout tip
    ctx.allow_overlap(
        aerator, spout_body,
        elem_a="hinge_barrel", elem_b="spout",
        reason="Aerator hinge barrel is intentionally seated into the spout tip to represent the pivot mechanism.",
    )

    # ── Three chrome posts seated on deck ───────────────────────────────
    for piece in (spout_body, left_valve, right_valve):
        ctx.expect_gap(
            piece, deck,
            axis="z", max_gap=0.001, max_penetration=0.0005,
            name=f"{piece.name} base seated on deck top",
        )

    # ── Bridge bar span and seating ─────────────────────────────────────
    bridge_aabb = ctx.part_world_aabb(bridge)
    ctx.check(
        "bridge bar spans about 0.30 m between posts",
        bridge_aabb is not None and 0.29 <= (bridge_aabb[1][0] - bridge_aabb[0][0]) <= 0.31,
        details=f"bridge aabb={bridge_aabb}",
    )
    ctx.expect_gap(
        bridge, deck,
        axis="z", max_gap=0.001, max_penetration=0.0005,
        name="bridge bar seated on deck surface",
    )

    # ── Base seams present on all three posts ───────────────────────────
    for col in (spout_body, left_valve, right_valve):
        seam_aabb = ctx.part_element_world_aabb(col, elem="base_seam")
        ctx.check(
            f"{col.name} has a narrow deck-base seam",
            seam_aabb is not None
            and abs(seam_aabb[0][2]) < 0.002
            and (seam_aabb[1][2] - seam_aabb[0][2]) < 0.004,
            details=f"{col.name} seam aabb={seam_aabb}",
        )

    # ── Three-piece spread ~0.30 m ──────────────────────────────────────
    ctx.expect_origin_distance(
        left_handle, right_handle,
        axes="x", min_dist=0.29, max_dist=0.31,
        name="handle spread is about 0.30 m",
    )
    ctx.expect_origin_gap(
        right_valve, spout_body,
        axis="x", min_gap=0.14, max_gap=0.16,
        name="right valve flanks the spout column",
    )
    ctx.expect_origin_gap(
        spout_body, left_valve,
        axis="x", min_gap=0.14, max_gap=0.16,
        name="left valve flanks the spout column",
    )

    # ── Waterfall spout geometry ────────────────────────────────────────
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

    # ── Aerator at spout tip ────────────────────────────────────────────
    ctx.expect_gap(
        aerator, deck,
        axis="z", min_gap=0.005,
        name="aerator hangs above the deck at rest",
    )
    aer_aabb = ctx.part_element_world_aabb(aerator, elem="aerator_body")
    ctx.check(
        "aerator body hangs below the spout tip",
        aer_aabb is not None and aer_aabb[0][2] > 0.005 and aer_aabb[0][2] < 0.030,
        details=f"aerator aabb={aer_aabb}",
    )

    # Aerator hinge limits: 0 to ~0.80 rad
    aer_lim = j_aer.motion_limits
    ctx.check(
        "aerator pivot range is 0 to ~0.80 rad",
        aer_lim is not None
        and aer_lim.lower is not None
        and aer_lim.upper is not None
        and abs(aer_lim.lower) < 0.01
        and abs(aer_lim.upper - 0.80) < 0.05,
        details=f"aerator limits={aer_lim}",
    )

    # Decisive pose: aerator body swings forward (Y increases) when pivoted
    aer_rest_aabb = ctx.part_element_world_aabb(aerator, elem="aerator_body")
    with ctx.pose({j_aer: 0.50}):
        aer_posed_aabb = ctx.part_element_world_aabb(aerator, elem="aerator_body")
    ctx.check(
        "aerator pivots forward/downward when opened",
        aer_rest_aabb is not None
        and aer_posed_aabb is not None
        and (aer_posed_aabb[0][1] + aer_posed_aabb[1][1]) / 2.0
        > (aer_rest_aabb[0][1] + aer_rest_aabb[1][1]) / 2.0 + 0.002,
        details=f"rest_y={(aer_rest_aabb[0][1] + aer_rest_aabb[1][1]) / 2.0:.4f}, "
                f"posed_y={(aer_posed_aabb[0][1] + aer_posed_aabb[1][1]) / 2.0:.4f}",
    )

    # ── Cross handles ───────────────────────────────────────────────────
    for handle, valve in ((left_handle, left_valve), (right_handle, right_valve)):
        h_aabb = ctx.part_world_aabb(handle)
        ctx.check(
            f"{handle.name} cross is about 0.09 m tip-to-tip",
            h_aabb is not None and 0.086 <= (h_aabb[1][0] - h_aabb[0][0]) <= 0.094,
            details=f"{handle.name} aabb={h_aabb}",
        )
        ctx.expect_gap(
            handle, valve,
            axis="z", max_gap=0.0005, max_penetration=0.004,
            name=f"{handle.name} hub seats over the valve stem",
        )

    # ── Finial button ───────────────────────────────────────────────────
    ctx.expect_gap(
        finial, spout_body,
        axis="z", max_gap=0.0005, max_penetration=0.003,
        name="finial stem seats into the cap top",
    )

    # ── Joint limits match the prompt ───────────────────────────────────
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

    # ── Decisive handle-spin pose checks ────────────────────────────────
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
        and math.hypot(posed_left[0] - rest_left[0], posed_left[1] - rest_left[1]) > 0.02,
        details=f"rest={rest_left}, posed={posed_left}",
    )

    rest_right = _ball_center(right_handle)
    with ctx.pose({j_right: -math.pi / 4.0}):
        posed_right = _ball_center(right_handle)
    ctx.check(
        "right handle spins independently about its stem axis",
        rest_right is not None
        and posed_right is not None
        and math.hypot(posed_right[0] - rest_right[0], posed_right[1] - rest_right[1]) > 0.02,
        details=f"rest={rest_right}, posed={posed_right}",
    )

    # ── Diverter finial pose check ──────────────────────────────────────
    oval_rest = ctx.part_element_world_aabb(finial, elem="finial_oval")
    ctx.check(
        "finial oval long axis along X at rest",
        oval_rest is not None
        and (oval_rest[1][0] - oval_rest[0][0]) > (oval_rest[1][1] - oval_rest[0][1]) + 0.008,
        details=f"oval rest aabb={oval_rest}",
    )
    with ctx.pose({j_div: math.pi / 2.0}):
        oval_posed = ctx.part_element_world_aabb(finial, elem="finial_oval")
    ctx.check(
        "diverter finial rotates 90 deg about the vertical axis",
        oval_posed is not None
        and (oval_posed[1][1] - oval_posed[0][1]) > (oval_posed[1][0] - oval_posed[0][0]) + 0.008,
        details=f"oval posed aabb={oval_posed}",
    )

    return ctx.report()


object_model = build_object_model()
