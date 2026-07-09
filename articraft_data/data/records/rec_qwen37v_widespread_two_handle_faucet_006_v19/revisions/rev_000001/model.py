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
# Variant 19: Widespread two-handle faucet (Art-Deco, three-piece layout)
#
# Structural changes from parent:
#   - Raised oval escutcheon plate under all three posts
#   - Cross handles rotate around short visible vertical axles
#   - Visible stem collars under each handle
#   - Separate hot and cold cap disks as geometry (red/blue)
# ---------------------------------------------------------------------------

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

# Escutcheon plate (oval, raised above deck)
ESC_RX = 0.175  # half-length along X (spans full spread with margin)
ESC_RY = 0.050  # half-width along Y
ESC_H = 0.005   # plate thickness

# Stem collar (visible ring under each handle)
COLLAR_R = 0.012
COLLAR_H = 0.006

# Visible short axle above stem top (handle rotates around this)
AXLE_R = 0.007
AXLE_H = 0.008

# Hot/cold cap indicator disks (fit inside the stem collar)
DISK_R = 0.010
DISK_H = 0.003


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


def _oval_plate(rx: float, ry: float, height: float) -> cq.Workplane:
    """Oval (elliptical plan) raised plate, base at z=0, extruded upward."""
    return (
        cq.Workplane("XY")
        .ellipse(rx, ry)
        .extrude(height)
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


def _add_valve_column(
    part: Part, chrome: str, *, cap_disk_mat: str | None = None
) -> None:
    """Tapered pyramid valve base with square cap, stem, collar, axle, and disk."""
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
    stem_z0 = V_PYR_H + V_CAP_H / 2.0  # rooted inside the cap
    part.visual(
        Cylinder(radius=V_STEM_R, length=V_STEM_TOP_Z - stem_z0),
        origin=Origin(xyz=(0.0, 0.0, (stem_z0 + V_STEM_TOP_Z) / 2.0)),
        material=chrome,
        name="valve_stem",
    )
    # Visible stem collar: decorative ring at the top of the cap
    collar_z = V_PYR_H + V_CAP_H
    part.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_H),
        origin=Origin(xyz=(0.0, 0.0, collar_z + COLLAR_H / 2.0)),
        material=chrome,
        name="stem_collar",
    )
    # Short visible axle above stem top (handle rotates around this)
    part.visual(
        Cylinder(radius=AXLE_R, length=AXLE_H),
        origin=Origin(xyz=(0.0, 0.0, V_STEM_TOP_Z + AXLE_H / 2.0)),
        material=chrome,
        name="handle_axle",
    )
    # Hot/cold indicator disk sitting on top of the collar (visible from above)
    if cap_disk_mat is not None:
        disk_z = V_PYR_H + V_CAP_H + COLLAR_H  # sits on collar top
        part.visual(
            Cylinder(radius=DISK_R, length=DISK_H),
            origin=Origin(xyz=(0.0, 0.0, disk_z + DISK_H / 2.0)),
            material=cap_disk_mat,
            name="cap_disk",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="art_deco_roman_tub_faucet")

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
    # Raised oval escutcheon plate under all three posts
    deck.visual(
        mesh_from_cadquery(_oval_plate(ESC_RX, ESC_RY, ESC_H), "escutcheon_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=chrome.name,
        name="escutcheon",
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
        origin=Origin(xyz=(0.0, 0.0, ESC_H)),
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

    # --- Hot/cold indicator materials ---
    hot_red = model.material("hot_red", rgba=(0.85, 0.15, 0.12, 1.0))
    cold_blue = model.material("cold_blue", rgba=(0.15, 0.30, 0.85, 1.0))

    # --- Valve columns and cross handles (left = -X, right = +X) ---
    for side, sx in (("left", -1.0), ("right", 1.0)):
        valve = model.part(f"{side}_valve")
        disk_mat = hot_red.name if side == "left" else cold_blue.name
        _add_valve_column(valve, chrome.name, cap_disk_mat=disk_mat)
        model.articulation(
            f"deck_to_{side}_valve",
            ArticulationType.FIXED,
            parent=deck,
            child=valve,
            origin=Origin(xyz=(sx * HANDLE_SPREAD_X, 0.0, ESC_H)),
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
    finial = object_model.get_part("diverter_finial")
    left_valve = object_model.get_part("left_valve")
    right_valve = object_model.get_part("right_valve")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")
    j_left = object_model.get_articulation("left_handle_spin")
    j_right = object_model.get_articulation("right_handle_spin")
    j_div = object_model.get_articulation("diverter_spin")

    # --- Variant 19: Escutcheon plate is a visible oval on the deck ---
    esc_aabb = ctx.part_element_world_aabb(deck, elem="escutcheon")
    ctx.check(
        "escutcheon plate exists and is oval",
        esc_aabb is not None
        and (esc_aabb[1][0] - esc_aabb[0][0]) > 2.5 * (esc_aabb[1][1] - esc_aabb[0][1]),
        details=f"escutcheon aabb={esc_aabb}",
    )
    ctx.check(
        "escutcheon plate spans all three posts along X",
        esc_aabb is not None
        and esc_aabb[0][0] < -HANDLE_SPREAD_X + 0.01
        and esc_aabb[1][0] > HANDLE_SPREAD_X - 0.01,
        details=f"escutcheon aabb={esc_aabb}",
    )

    # --- Variant 19: Stem collars visible under each handle ---
    for valve in (left_valve, right_valve):
        collar_aabb = ctx.part_element_world_aabb(valve, elem="stem_collar")
        ctx.check(
            f"{valve.name} has a visible stem collar",
            collar_aabb is not None
            and 0.010 <= (collar_aabb[1][1] - collar_aabb[0][1]) <= 0.026,
            details=f"collar aabb={collar_aabb}",
        )

    # --- Variant 19: Hot/cold cap disks as geometry (not text) ---
    hot_disk_aabb = ctx.part_element_world_aabb(left_valve, elem="cap_disk")
    cold_disk_aabb = ctx.part_element_world_aabb(right_valve, elem="cap_disk")
    ctx.check(
        "left valve has a hot cap disk",
        hot_disk_aabb is not None,
        details=f"hot disk aabb={hot_disk_aabb}",
    )
    ctx.check(
        "right valve has a cold cap disk",
        cold_disk_aabb is not None,
        details=f"cold disk aabb={cold_disk_aabb}",
    )
    # Verify disks are distinct materials (different colors)
    hot_disk_visual = left_valve.get_visual("cap_disk")
    cold_disk_visual = right_valve.get_visual("cap_disk")

    def _mat_name(m) -> str:
        return m.name if hasattr(m, "name") else str(m)

    ctx.check(
        "hot and cold disks have distinct materials",
        hot_disk_visual is not None
        and cold_disk_visual is not None
        and hot_disk_visual.material is not None
        and cold_disk_visual.material is not None
        and _mat_name(hot_disk_visual.material) != _mat_name(cold_disk_visual.material),
    )

    # Intentional captured fits: handle hubs over valve stems and axles,
    # finial stem seated into the stepped cap.
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
        left_handle,
        left_valve,
        elem_a="hub",
        elem_b="handle_axle",
        reason="Cross-handle hub rotates around the short vertical axle.",
    )
    ctx.allow_overlap(
        right_handle,
        right_valve,
        elem_a="hub",
        elem_b="handle_axle",
        reason="Cross-handle hub rotates around the short vertical axle.",
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
        cap_aabb is not None and abs(cap_aabb[1][2] - (CAP_TOP_Z + ESC_H)) < 0.002,
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
            max_penetration=0.012,
            name=f"{handle.name} hub seats over the valve stem and axle",
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

    # --- Decisive pose checks (cross is 90-deg symmetric, so pose 45 deg) ---
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
