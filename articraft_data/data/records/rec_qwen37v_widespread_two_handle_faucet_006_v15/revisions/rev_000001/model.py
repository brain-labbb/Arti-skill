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
# Art-Deco widespread two-handle wall-mount spout faucet, mirror chrome.
#
# Layout (meters, Z up, spout sweeps forward along +Y):
#   - dark deck plate (root) with two chrome valve columns at x = +/-0.15
#   - vertical chrome wall plate behind the deck (y ~ -0.10)
#   - wall-mounted spout body: stepped escutcheon, short neck, flat-topped
#     waterfall spout arm reaching ~0.18 m forward, diverter platform + finial
#   - each valve column: tapered square pyramid base (0.06 sq, 0.07 tall),
#     square cap, visible stem collar, slim stem carrying a four-spoke cross
#     handle (0.09 m tip-to-tip with ball ends)
# Articulation: each cross handle revolute about its vertical stem axis
# (-pi..pi); the oval finial is a revolute diverter (-pi/2..pi/2).
# ---------------------------------------------------------------------------

HANDLE_SPREAD_X = 0.150  # valve column centers at +/-0.150 -> 0.30 m spread

# Wall plate
WALL_Y = -0.100  # wall plate center Y (behind the deck)
WALL_PLATE_W = 0.140
WALL_PLATE_H = 0.120
WALL_PLATE_T = 0.008

# Spout body (wall-mounted)
SPOUT_ORIGIN_Z = 0.065  # spout body origin height above deck
ESC_W = 0.088  # escutcheon outer width
ESC_H = 0.100  # escutcheon outer height
ESC_T_OUTER = 0.005  # outer step thickness
ESC_T_INNER = 0.003  # inner step thickness
NECK_W = 0.044  # neck width
NECK_L = 0.028  # neck length (along Y)
NECK_H = 0.030  # neck height (along Z)
PLATFORM_W = 0.040
PLATFORM_L = 0.028
PLATFORM_H = 0.006
SPOUT_WIDTH = 0.050

# Valve columns
V_PYR_BASE = 0.060
V_PYR_TOP = 0.034
V_PYR_H = 0.070
V_CAP_SIZE = 0.040
V_CAP_H = 0.008
V_STEM_R = 0.0065
V_STEM_TOP_Z = 0.096
COLLAR_R = 0.010  # stem collar outer radius
COLLAR_H = 0.006  # stem collar height
HANDLE_JOINT_Z = 0.093  # hub captures the stem top by 3 mm

# Cross handle
HUB_R = 0.0085
HUB_H = 0.034
SPOKE_R = 0.0042
SPOKE_LEN = 0.040
SPOKE_Z = 0.012
BALL_R = 0.0065
BALL_C = 0.0385  # ball centers -> tip-to-tip = 2*(0.0385+0.0065) = 0.090

# Finial diverter
FINIAL_RX = 0.018
FINIAL_RY = 0.012
FINIAL_RZ = 0.008
FINIAL_STEM_R = 0.0045
FINIAL_CENTER_Z = 0.014


def _pyramid_frustum(base: float, top: float, height: float) -> cq.Workplane:
    """Tapered square-pyramid column, base on z=0, narrowing upward."""
    return (
        cq.Workplane("XY")
        .rect(base, base)
        .workplane(offset=height)
        .rect(top, top)
        .loft(combine=True)
    )


def _wall_waterfall_spout() -> cq.Workplane:
    """Waterfall spout arm for wall mounting.

    Side profile drawn in the YZ plane, extruded across X for flat
    Art-Deco slab sides. The root (y ~ 0.035) emerges from the neck.
    Local frame: origin at the spout body origin (wall face center).
    +Y forward, +Z up relative to spout body origin.
    """
    profile = (
        cq.Workplane("YZ")
        .moveTo(0.035, 0.008)
        .lineTo(0.035, 0.022)
        .spline(
            [(0.065, 0.020), (0.105, 0.012), (0.145, -0.006), (0.175, -0.030)],
            includeCurrent=True,
        )
        .lineTo(0.162, -0.036)
        .spline(
            [(0.138, -0.014), (0.100, 0.002), (0.065, 0.006), (0.035, 0.008)],
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
    stem_z0 = V_PYR_H + V_CAP_H / 2.0  # rooted inside the cap
    part.visual(
        Cylinder(radius=V_STEM_R, length=V_STEM_TOP_Z - stem_z0),
        origin=Origin(xyz=(0.0, 0.0, (stem_z0 + V_STEM_TOP_Z) / 2.0)),
        material=chrome,
        name="valve_stem",
    )
    # Visible stem collar: decorative ring at top of valve cap, under the handle
    collar_z = V_PYR_H + V_CAP_H + COLLAR_H / 2.0
    part.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_H),
        origin=Origin(xyz=(0.0, 0.0, collar_z)),
        material=chrome,
        name="stem_collar",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_wall_mount_faucet")

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

    # --- Vertical chrome wall plate behind the deck ---
    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        Box((WALL_PLATE_W, WALL_PLATE_T, WALL_PLATE_H)),
        origin=Origin(xyz=(0.0, 0.0, WALL_PLATE_H / 2.0)),
        material=chrome.name,
        name="wall_plate_panel",
    )
    model.articulation(
        "deck_to_wall_plate",
        ArticulationType.FIXED,
        parent=deck,
        child=wall_plate,
        origin=Origin(xyz=(0.0, WALL_Y, 0.0)),
    )

    # --- Wall-mounted spout body ---
    spout_body = model.part("spout_body")

    # Escutcheon: Art-Deco stepped rectangular wall plate
    spout_body.visual(
        Box((ESC_W, ESC_T_OUTER, ESC_H)),
        origin=Origin(xyz=(0.0, ESC_T_OUTER / 2.0, 0.0)),
        material=chrome.name,
        name="escutcheon_outer",
    )
    spout_body.visual(
        Box((ESC_W - 0.012, ESC_T_INNER, ESC_H - 0.012)),
        origin=Origin(xyz=(0.0, ESC_T_OUTER + ESC_T_INNER / 2.0, 0.0)),
        material=chrome.name,
        name="escutcheon_inner",
    )

    # Neck: short horizontal transition from escutcheon to spout arm
    neck_y = ESC_T_OUTER + ESC_T_INNER + NECK_L / 2.0
    spout_body.visual(
        Box((NECK_W, NECK_L, NECK_H)),
        origin=Origin(xyz=(0.0, neck_y, 0.0)),
        material=chrome.name,
        name="spout_neck",
    )

    # Diverter platform: small raised pad on top of neck for the finial
    platform_z = NECK_H / 2.0 + PLATFORM_H / 2.0
    spout_body.visual(
        Box((PLATFORM_W, PLATFORM_L, PLATFORM_H)),
        origin=Origin(xyz=(0.0, neck_y, platform_z)),
        material=chrome.name,
        name="diverter_platform",
    )

    # Waterfall spout arm
    spout_body.visual(
        mesh_from_cadquery(_wall_waterfall_spout(), "wall_spout_arm"),
        material=chrome.name,
        name="spout",
    )

    # Mount spout body on wall plate front face
    wall_front_y = WALL_PLATE_T / 2.0
    model.articulation(
        "wall_to_spout",
        ArticulationType.FIXED,
        parent=wall_plate,
        child=spout_body,
        origin=Origin(xyz=(0.0, wall_front_y, SPOUT_ORIGIN_Z)),
    )

    # --- Oval finial diverter on the spout body platform ---
    finial = model.part("diverter_finial")
    finial.visual(
        Cylinder(radius=FINIAL_STEM_R, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),  # embeds 2 mm into the platform
        material=chrome.name,
        name="finial_stem",
    )
    finial.visual(
        mesh_from_cadquery(_oval_finial(), "finial_oval"),
        origin=Origin(xyz=(0.0, 0.0, FINIAL_CENTER_Z)),
        material=chrome.name,
        name="finial_oval",
    )
    platform_top_z = platform_z + PLATFORM_H / 2.0
    model.articulation(
        "diverter_spin",
        ArticulationType.REVOLUTE,
        parent=spout_body,
        child=finial,
        origin=Origin(xyz=(0.0, neck_y, platform_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=2.0, lower=-math.pi / 2.0, upper=math.pi / 2.0
        ),
    )

    # --- Valve columns and cross handles (left = -X, right = +X) ---
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
    wall_plate = object_model.get_part("wall_plate")
    spout_body = object_model.get_part("spout_body")
    finial = object_model.get_part("diverter_finial")
    left_valve = object_model.get_part("left_valve")
    right_valve = object_model.get_part("right_valve")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")
    j_left = object_model.get_articulation("left_handle_spin")
    j_right = object_model.get_articulation("right_handle_spin")
    j_div = object_model.get_articulation("diverter_spin")

    # Intentional captured fits
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
        elem_b="diverter_platform",
        reason="Finial stem is intentionally seated 2 mm into the diverter platform.",
    )

    # --- Wall plate is vertical and behind the deck ---
    wp_aabb = ctx.part_world_aabb(wall_plate)
    ctx.check(
        "wall_plate is vertical (height >> thickness)",
        wp_aabb is not None
        and (wp_aabb[1][2] - wp_aabb[0][2]) > (wp_aabb[1][1] - wp_aabb[0][1]) * 5,
        details=f"wall_plate aabb={wp_aabb}",
    )
    ctx.check(
        "wall_plate is behind the deck center (negative Y)",
        wp_aabb is not None and (wp_aabb[0][1] + wp_aabb[1][1]) / 2.0 < -0.05,
        details=f"wall_plate aabb={wp_aabb}",
    )

    # --- Spout body is wall-mounted (elevated above the deck) ---
    ctx.expect_gap(
        spout_body,
        deck,
        axis="z",
        min_gap=0.005,
        name="spout_body clears the deck surface (wall-mounted)",
    )
    spout_body_aabb = ctx.part_world_aabb(spout_body)
    ctx.check(
        "spout_body lowest point is above the deck",
        spout_body_aabb is not None and spout_body_aabb[0][2] > 0.005,
        details=f"spout_body aabb={spout_body_aabb}",
    )

    # --- Stem collars present on both valve columns ---
    for valve in (left_valve, right_valve):
        collar_aabb = ctx.part_element_world_aabb(valve, elem="stem_collar")
        ctx.check(
            f"{valve.name} has a visible stem_collar",
            collar_aabb is not None,
            details=f"stem_collar aabb={collar_aabb}",
        )
        cap_aabb = ctx.part_element_world_aabb(valve, elem="valve_cap")
        if collar_aabb is not None and cap_aabb is not None:
            ctx.check(
                f"{valve.name} stem_collar sits at or above the valve cap top",
                collar_aabb[0][2] >= cap_aabb[1][2] - 0.002,
                details=f"collar bottom={collar_aabb[0][2]}, cap top={cap_aabb[1][2]}",
            )

    # --- Valve columns seated on deck ---
    for valve in (left_valve, right_valve):
        ctx.expect_gap(
            valve,
            deck,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0005,
            name=f"{valve.name} base seated on deck top",
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

    # --- Spout reach and arc ---
    spout_elem_aabb = ctx.part_element_world_aabb(spout_body, elem="spout")
    ctx.check(
        "spout reaches forward from the wall (>0.04 m past deck center)",
        spout_elem_aabb is not None and spout_elem_aabb[1][1] > 0.04,
        details=f"spout aabb={spout_elem_aabb}",
    )
    ctx.check(
        "spout tip arcs down but stays above the deck",
        spout_elem_aabb is not None and 0.01 <= spout_elem_aabb[0][2] <= 0.055,
        details=f"spout aabb={spout_elem_aabb}",
    )

    # --- Cross handles: 0.09 m tip-to-tip, seated over stems ---
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

    # --- Finial button seated on diverter platform ---
    ctx.expect_gap(
        finial,
        spout_body,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.003,
        positive_elem="finial_stem",
        negative_elem="diverter_platform",
        name="finial stem seats into the diverter platform",
    )
    ctx.expect_within(
        finial,
        spout_body,
        axes="xy",
        inner_elem="finial_oval",
        outer_elem="diverter_platform",
        margin=0.002,
        name="oval finial centered on the diverter platform",
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
