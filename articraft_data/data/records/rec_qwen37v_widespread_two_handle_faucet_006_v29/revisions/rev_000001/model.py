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
# Art-Deco three-piece deck-mounted roman tub faucet, mirror chrome.
#
# Layout (meters, Z up, spout sweeps forward along +Y):
#   - dark deck plate (root) with three chrome pieces mounted on top (z = 0)
#   - center spout column at x = 0: tapered square-pyramid base (0.07 sq at
#     deck -> 0.046 sq at z = 0.08), stepped square cap, flat-topped waterfall
#     spout reaching ~0.18 forward, oval finial diverter on top
#   - valve columns at x = +/-0.15: smaller tapered pyramids (0.06 sq, 0.07
#     tall) with square cap and slim stem carrying a four-spoke cross handle,
#     0.09 tip-to-tip with ball ends
# Articulation: each cross handle revolute about its vertical stem axis
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

# Spout
SPOUT_WIDTH = 0.050

# Finial diverter
FINIAL_RX = 0.018
FINIAL_RY = 0.012
FINIAL_RZ = 0.008
FINIAL_STEM_R = 0.0045
FINIAL_CENTER_Z = 0.014

# Escutcheon plates (raised oval under each post)
ESC_RX = 0.045       # half-length along X
ESC_RY = 0.032       # half-width along Y
ESC_H = 0.005        # plate thickness, sits proud of deck
ESC_CENTER_Z = ESC_H / 2.0  # bottom on deck z=0

# Stem collars (visible ring under each handle, on top of valve cap)
COLLAR_OR = 0.011    # outer radius
COLLAR_IR = 0.007    # inner radius (slightly larger than stem)
COLLAR_H = 0.005     # collar height
COLLAR_BASE_Z = V_PYR_H + V_CAP_H  # top of valve cap

# Underside mounting nuts (hex nuts below deck)
NUT_AF = 0.014       # across-flats dimension
NUT_H = 0.006        # nut height
NUT_CENTER_Z = -0.011 - NUT_H / 2.0  # below deck top surface
SHANK_R = 0.005      # threaded shank radius through deck
SHANK_TOP_Z = 0.0    # connects to escutcheon base
SHANK_BOT_Z = NUT_CENTER_Z + NUT_H / 2.0  # top of nut = deck bottom


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
    """Wide flat-topped spout sweeping forward (+Y) into a waterfall arc.

    Side profile drawn in the YZ plane, extruded across X for flat
    Art-Deco slab sides. The root (y ~ 0.010) is buried inside the
    pyramid column so the spout reads as emerging from the body.
    """
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


def _oval_escutcheon() -> cq.Workplane:
    """Raised oval escutcheon plate, base on z=0, extruded upward."""
    return (
        cq.Workplane("XY")
        .ellipse(ESC_RX, ESC_RY)
        .extrude(ESC_H)
    )


def _stem_collar() -> cq.Workplane:
    """Annular collar ring sitting on top of valve cap, base on z=0."""
    return (
        cq.Workplane("XY")
        .circle(COLLAR_OR)
        .circle(COLLAR_IR)
        .extrude(COLLAR_H)
    )


def _hex_nut() -> cq.Workplane:
    """Hex nut prism, base on z=0. Across-flats = NUT_AF."""
    # circumscribed diameter from across-flats
    circ_dia = NUT_AF / math.cos(math.pi / 6.0)
    return (
        cq.Workplane("XY")
        .polygon(6, circ_dia)
        .extrude(NUT_H)
    )


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
    # Radial spokes: SDK Cylinder long axis is local Z -> rotate onto X / Y.
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


def _add_valve_column(part: Part, chrome: str, base_z: float = 0.0) -> None:
    """Tapered pyramid valve base with square cap, stem collar, and stem.

    ``base_z`` shifts the column upward (e.g. to sit on an escutcheon plate).
    """
    part.visual(
        mesh_from_cadquery(
            _pyramid_frustum(V_PYR_BASE, V_PYR_TOP, V_PYR_H),
            f"{part.name}_pyramid",
        ),
        origin=Origin(xyz=(0.0, 0.0, base_z)),
        material=chrome,
        name="valve_pyramid",
    )
    part.visual(
        Box((V_CAP_SIZE, V_CAP_SIZE, V_CAP_H)),
        origin=Origin(xyz=(0.0, 0.0, base_z + V_PYR_H + V_CAP_H / 2.0)),
        material=chrome,
        name="valve_cap",
    )
    # Visible stem collar ring on top of the valve cap
    part.visual(
        mesh_from_cadquery(_stem_collar(), f"{part.name}_collar"),
        origin=Origin(xyz=(0.0, 0.0, base_z + COLLAR_BASE_Z)),
        material=chrome,
        name="stem_collar",
    )
    stem_z0 = base_z + V_PYR_H + V_CAP_H / 2.0  # rooted inside the cap
    stem_top = base_z + V_STEM_TOP_Z
    part.visual(
        Cylinder(radius=V_STEM_R, length=stem_top - stem_z0),
        origin=Origin(xyz=(0.0, 0.0, (stem_z0 + stem_top) / 2.0)),
        material=chrome,
        name="valve_stem",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_two_handle_faucet")

    chrome = model.material("chrome", rgba=(0.88, 0.89, 0.92, 1.0))
    deck_mat = model.material("deck_charcoal", rgba=(0.09, 0.09, 0.10, 1.0))

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
    # Raised oval escutcheon plate under center post
    spout_body.visual(
        mesh_from_cadquery(_oval_escutcheon(), "center_escutcheon"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=chrome.name,
        name="escutcheon",
    )
    spout_body.visual(
        mesh_from_cadquery(
            _pyramid_frustum(C_PYR_BASE, C_PYR_TOP, C_PYR_H), "center_pyramid"
        ),
        origin=Origin(xyz=(0.0, 0.0, ESC_H)),
        material=chrome.name,
        name="spout_pyramid",
    )
    spout_body.visual(
        Box((CAP1_SIZE, CAP1_SIZE, CAP1_H)),
        origin=Origin(xyz=(0.0, 0.0, ESC_H + C_PYR_H + CAP1_H / 2.0)),
        material=chrome.name,
        name="cap_step_lower",
    )
    spout_body.visual(
        Box((CAP2_SIZE, CAP2_SIZE, CAP2_H)),
        origin=Origin(xyz=(0.0, 0.0, ESC_H + C_PYR_H + CAP1_H + CAP2_H / 2.0)),
        material=chrome.name,
        name="cap_step_upper",
    )
    spout_body.visual(
        mesh_from_cadquery(_waterfall_spout(), "waterfall_spout"),
        origin=Origin(xyz=(0.0, 0.0, ESC_H)),
        material=chrome.name,
        name="spout",
    )
    # Underside hex mounting nut
    spout_body.visual(
        mesh_from_cadquery(_hex_nut(), "center_nut"),
        origin=Origin(xyz=(0.0, 0.0, NUT_CENTER_Z - NUT_H / 2.0)),
        material=chrome.name,
        name="mounting_nut",
    )
    # Threaded shank connecting nut through deck to escutcheon
    _shank_len = SHANK_TOP_Z - SHANK_BOT_Z
    spout_body.visual(
        Cylinder(radius=SHANK_R, length=_shank_len),
        origin=Origin(xyz=(0.0, 0.0, (SHANK_BOT_Z + SHANK_TOP_Z) / 2.0)),
        material=chrome.name,
        name="mounting_shank",
    )
    model.articulation(
        "deck_to_spout_body",
        ArticulationType.FIXED,
        parent=deck,
        child=spout_body,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # --- Oval finial diverter button on the cap ---
    SPOUT_CAP_TOP = ESC_H + CAP_TOP_Z  # cap top in spout_body frame
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
        origin=Origin(xyz=(0.0, 0.0, SPOUT_CAP_TOP)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=2.0, lower=-math.pi / 2.0, upper=math.pi / 2.0
        ),
    )

    # --- Valve columns and cross handles (left = -X, right = +X) ---
    VALVE_JOINT_Z = ESC_H + HANDLE_JOINT_Z  # handle joint in valve frame
    for side, sx in (("left", -1.0), ("right", 1.0)):
        valve = model.part(f"{side}_valve")
        # Raised oval escutcheon plate under valve post
        valve.visual(
            mesh_from_cadquery(_oval_escutcheon(), f"{side}_escutcheon"),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=chrome.name,
            name="escutcheon",
        )
        # Underside hex mounting nut
        valve.visual(
            mesh_from_cadquery(_hex_nut(), f"{side}_nut"),
            origin=Origin(xyz=(0.0, 0.0, NUT_CENTER_Z - NUT_H / 2.0)),
            material=chrome.name,
            name="mounting_nut",
        )
        # Threaded shank connecting nut through deck to escutcheon
        _shank_len = SHANK_TOP_Z - SHANK_BOT_Z
        valve.visual(
            Cylinder(radius=SHANK_R, length=_shank_len),
            origin=Origin(xyz=(0.0, 0.0, (SHANK_BOT_Z + SHANK_TOP_Z) / 2.0)),
            material=chrome.name,
            name="mounting_shank",
        )
        # Valve column sits on top of escutcheon
        _add_valve_column(valve, chrome.name, base_z=ESC_H)
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
            origin=Origin(xyz=(0.0, 0.0, VALVE_JOINT_Z)),
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

    SPOUT_CAP_TOP = ESC_H + CAP_TOP_Z

    # Intentional captured fits: handle hubs over valve stems, finial stem
    # seated into the stepped cap.
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
        finial,
        spout_body,
        elem_a="finial_stem",
        elem_b="cap_step_upper",
        reason="Finial stem is intentionally seated 2 mm into the stepped cap.",
    )
    # Mounting hardware (nut + shank) intentionally passes through the deck.
    for column in (spout_body, left_valve, right_valve):
        ctx.allow_overlap(
            column,
            deck,
            elem_a="mounting_nut",
            elem_b="deck_plate",
            reason="Mounting nut is below the deck surface, threaded through the deck hole.",
        )
        ctx.allow_overlap(
            column,
            deck,
            elem_a="mounting_shank",
            elem_b="deck_plate",
            reason="Threaded shank passes through the deck hole connecting nut to column above.",
        )

    # --- Escutcheon plates sit on deck surface, not floating ---
    for piece in (spout_body, left_valve, right_valve):
        ctx.expect_gap(
            piece,
            deck,
            axis="z",
            positive_elem="escutcheon",
            max_gap=0.001,
            max_penetration=0.0005,
            name=f"{piece.name} escutcheon seated on deck top",
        )
        ctx.expect_within(
            piece,
            deck,
            axes="x",
            inner_elem="escutcheon",
            margin=0.002,
            name=f"{piece.name} escutcheon within deck footprint",
        )

    # --- Variant: escutcheon plates are oval and raised ---
    for piece in (spout_body, left_valve, right_valve):
        esc_aabb = ctx.part_element_world_aabb(piece, elem="escutcheon")
        ctx.check(
            f"{piece.name} has a raised oval escutcheon plate",
            esc_aabb is not None
            and (esc_aabb[1][2] - esc_aabb[0][2]) >= ESC_H - 0.001
            and (esc_aabb[1][0] - esc_aabb[0][0]) > (esc_aabb[1][1] - esc_aabb[0][1]),
            details=f"escutcheon aabb={esc_aabb}",
        )

    # --- Variant: stem collars visible under each handle ---
    for valve in (left_valve, right_valve):
        collar_aabb = ctx.part_element_world_aabb(valve, elem="stem_collar")
        ctx.check(
            f"{valve.name} has a visible stem collar above the cap",
            collar_aabb is not None
            and collar_aabb[0][2] >= ESC_H + V_PYR_H + V_CAP_H - 0.001,
            details=f"collar aabb={collar_aabb}",
        )

    # --- Variant: underside mounting nuts below the bases ---
    for piece in (spout_body, left_valve, right_valve):
        nut_aabb = ctx.part_element_world_aabb(piece, elem="mounting_nut")
        ctx.check(
            f"{piece.name} has a hex mounting nut below the deck",
            nut_aabb is not None and nut_aabb[1][2] < 0.0,
            details=f"nut aabb={nut_aabb}",
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

    # --- Waterfall spout: forward reach ~0.18 m, tip arcs down, overhangs ---
    spout_aabb = ctx.part_element_world_aabb(spout_body, elem="spout")
    ctx.check(
        "spout reaches about 0.18 m forward",
        spout_aabb is not None and 0.16 <= spout_aabb[1][1] <= 0.20,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "spout tip arcs down well below the cap but stays above the deck",
        spout_aabb is not None and 0.01 <= spout_aabb[0][2] <= 0.050,
        details=f"spout aabb={spout_aabb}",
    )
    pyr_aabb = ctx.part_element_world_aabb(spout_body, elem="spout_pyramid")
    ctx.check(
        "center pyramid base is about 0.07 m square",
        pyr_aabb is not None
        and 0.066 <= (pyr_aabb[1][0] - pyr_aabb[0][0]) <= 0.074
        and 0.066 <= (pyr_aabb[1][1] - pyr_aabb[0][1]) <= 0.074,
        details=f"pyramid aabb={pyr_aabb}",
    )
    cap_aabb = ctx.part_element_world_aabb(spout_body, elem="cap_step_upper")
    ctx.check(
        "stepped cap tops the center pyramid",
        cap_aabb is not None and abs(cap_aabb[1][2] - SPOUT_CAP_TOP) < 0.002,
        details=f"cap aabb={cap_aabb}",
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
            name=f"{handle.name} hub seats over the valve stem collar",
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

    # --- Decisive pose checks: both handles rotate independently ---
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

    # Oval finial: long axis along X at rest; rotated 90 deg it swaps to Y.
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
