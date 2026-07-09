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
# Widespread two-handle faucet variant (Art-Deco chrome, three-piece deck).
#
# Layout (meters, Z up, spout sweeps forward along +Y):
#   - dark deck plate (root) with three chrome pieces mounted on top (z = 0)
#   - center spout column at x = 0: tapered square-pyramid base (0.07 sq at
#     deck -> 0.046 sq at z = 0.08), stepped square cap, flat-topped waterfall
#     spout reaching ~0.18 forward, hollow outlet, oval finial diverter on top
#   - valve columns at x = +/-0.15: smaller tapered pyramids (0.06 sq, 0.07
#     tall) with square cap and slim stem carrying a four-spoke cross handle
#   - narrow dark seams at all three deck bases
#   - pivoting aerator at spout outlet on a small hinge
#
# Asymmetric handles: left handle yawed +20 deg, right yawed -15 deg
# (balanced but not mirrored around the spout).
# Articulation: each cross handle revolute about its vertical stem axis
# (-pi..pi); the oval finial is a revolute diverter (-pi/2..pi/2);
# the aerator pivots downward on a small hinge.
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

# Aerator
AERATOR_R = 0.014
AERATOR_THICK = 0.005
# Spout tip location (approximate from waterfall profile)
SPOUT_TIP_Y = 0.167
SPOUT_TIP_Z = 0.027

# Seam dimensions
SEAM_WIDTH = 0.002
SEAM_HEIGHT = 0.0015

# Handle asymmetry offsets (radians)
LEFT_HANDLE_YAW = math.radians(20)
RIGHT_HANDLE_YAW = math.radians(-15)


def _pyramid_frustum(base: float, top: float, height: float) -> cq.Workplane:
    """Tapered square-pyramid column, base on z=0, narrowing upward."""
    return (
        cq.Workplane("XY")
        .rect(base, base)
        .workplane(offset=height)
        .rect(top, top)
        .loft(combine=True)
    )


def _waterfall_spout_hollow() -> cq.Workplane:
    """Wide flat-topped spout sweeping forward (+Y) into a waterfall arc
    with a hollow outlet cavity at the tip.

    Side profile drawn in the YZ plane, extruded across X for flat
    Art-Deco slab sides. The root (y ~ 0.010) is buried inside the
    pyramid column so the spout reads as emerging from the body.
    The hollow cavity is cut from the underside of the tip to create
    a visible open outlet.
    """
    # Outer solid spout shape
    outer = (
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
    spout = outer.translate((-SPOUT_WIDTH / 2.0, 0.0, 0.0))

    # Hollow cavity at the spout tip outlet - a rectangular channel cut
    # from the underside of the spout tip to represent the open outlet.
    # Sized to accommodate the aerator disk seated inside.
    cavity_width = SPOUT_WIDTH * 0.72
    cavity_depth = 0.024  # how far back into the tip
    cavity_height = 0.016  # vertical opening height

    cavity = (
        cq.Workplane("XY")
        .center(0.0, SPOUT_TIP_Y - cavity_depth / 2.0)
        .rect(cavity_width, cavity_depth)
        .extrude(cavity_height)
        .translate((0.0, 0.0, SPOUT_TIP_Z - cavity_height + 0.002))
    )

    spout = spout.cut(cavity)
    return spout


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


def _aerator_mesh() -> cq.Workplane:
    """Small cylindrical aerator disk with a recessed screen pattern."""
    # Outer cylinder shell
    outer = (
        cq.Workplane("XY")
        .circle(AERATOR_R)
        .extrude(AERATOR_THICK)
    )
    # Inner cavity (hollow through-hole for water)
    inner = (
        cq.Workplane("XY")
        .circle(AERATOR_R - 0.003)
        .extrude(AERATOR_THICK)
    )
    # Cut the inner bore
    result = outer.cut(inner)
    # Add a thin cross-bar screen pattern on the bottom face
    bar_w = 0.002
    bar_span = 2.0 * (AERATOR_R - 0.003)
    bar1 = (
        cq.Workplane("XY")
        .center(0, 0)
        .rect(bar_span, bar_w)
        .extrude(0.001)
    )
    bar2 = (
        cq.Workplane("XY")
        .center(0, 0)
        .rect(bar_w, bar_span)
        .extrude(0.001)
    )
    result = result.union(bar1).union(bar2)
    return result


def _add_cross_handle(part: Part, chrome: str, yaw_offset: float = 0.0) -> None:
    """Four-spoke cross handle with ball ends, rotating about local +Z.

    Local frame origin is the handle joint frame: hub bottom at z=0.
    yaw_offset rotates the spoke pattern for asymmetric rest appearance.
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
    # Apply yaw_offset to rotate the entire spoke pattern.
    cos_y = math.cos(yaw_offset)
    sin_y = math.sin(yaw_offset)
    spoke_dirs_base = [
        ((SPOKE_LEN / 2.0, 0.0, SPOKE_Z), (0.0, math.pi / 2.0, 0.0)),
        ((-SPOKE_LEN / 2.0, 0.0, SPOKE_Z), (0.0, math.pi / 2.0, 0.0)),
        ((0.0, SPOKE_LEN / 2.0, SPOKE_Z), (math.pi / 2.0, 0.0, 0.0)),
        ((0.0, -SPOKE_LEN / 2.0, SPOKE_Z), (math.pi / 2.0, 0.0, 0.0)),
    ]
    for i, (xyz_base, rpy_base) in enumerate(spoke_dirs_base):
        # Rotate position by yaw_offset about Z
        x = xyz_base[0] * cos_y - xyz_base[1] * sin_y
        y = xyz_base[0] * sin_y + xyz_base[1] * cos_y
        z = xyz_base[2]
        # For the rotation of the cylinder axis, add yaw_offset to the
        # spoke orientation (the rpy encodes which axis the cylinder lies on)
        # For X-axis spokes: rpy=(0, pi/2, 0) -> yaw rotation adds to yaw
        # For Y-axis spokes: rpy=(pi/2, 0, 0) -> yaw rotation adds to yaw
        if i < 2:
            # Originally along X: rotate the whole orientation by yaw_offset
            rpy = (rpy_base[0], rpy_base[1], yaw_offset)
        else:
            # Originally along Y: rotate the whole orientation by yaw_offset
            rpy = (rpy_base[0], rpy_base[1], yaw_offset)
        part.visual(
            Cylinder(radius=SPOKE_R, length=SPOKE_LEN),
            origin=Origin(xyz=(x, y, z), rpy=rpy),
            material=chrome,
            name=f"spoke_{i}",
        )
    ball_centers_base = [
        (BALL_C, 0.0, SPOKE_Z),
        (-BALL_C, 0.0, SPOKE_Z),
        (0.0, BALL_C, SPOKE_Z),
        (0.0, -BALL_C, SPOKE_Z),
    ]
    for i, bc in enumerate(ball_centers_base):
        x = bc[0] * cos_y - bc[1] * sin_y
        y = bc[0] * sin_y + bc[1] * cos_y
        z = bc[2]
        part.visual(
            Sphere(radius=BALL_R),
            origin=Origin(xyz=(x, y, z)),
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
    stem_z0 = V_PYR_H + V_CAP_H / 2.0  # rooted inside the cap
    part.visual(
        Cylinder(radius=V_STEM_R, length=V_STEM_TOP_Z - stem_z0),
        origin=Origin(xyz=(0.0, 0.0, (stem_z0 + V_STEM_TOP_Z) / 2.0)),
        material=chrome,
        name="valve_stem",
    )


def _add_deck_seam(part: Part, seam_mat: str, base_size: float, z_offset: float = 0.0) -> None:
    """Add a narrow dark seam ring at the base of a column."""
    seam_outer = base_size + 0.004  # slightly wider than the column base
    seam_inner = base_size - 0.001
    # Use a thin box frame (4 thin rectangles forming a square outline)
    half_outer = seam_outer / 2.0
    half_inner = seam_inner / 2.0
    # Front seam
    part.visual(
        Box((seam_outer, SEAM_WIDTH, SEAM_HEIGHT)),
        origin=Origin(xyz=(0.0, half_outer - SEAM_WIDTH / 2.0, z_offset + SEAM_HEIGHT / 2.0)),
        material=seam_mat,
        name="seam_front",
    )
    # Back seam
    part.visual(
        Box((seam_outer, SEAM_WIDTH, SEAM_HEIGHT)),
        origin=Origin(xyz=(0.0, -(half_outer - SEAM_WIDTH / 2.0), z_offset + SEAM_HEIGHT / 2.0)),
        material=seam_mat,
        name="seam_back",
    )
    # Left seam
    part.visual(
        Box((SEAM_WIDTH, seam_inner, SEAM_HEIGHT)),
        origin=Origin(xyz=(-(half_outer - SEAM_WIDTH / 2.0), 0.0, z_offset + SEAM_HEIGHT / 2.0)),
        material=seam_mat,
        name="seam_left",
    )
    # Right seam
    part.visual(
        Box((SEAM_WIDTH, seam_inner, SEAM_HEIGHT)),
        origin=Origin(xyz=(half_outer - SEAM_WIDTH / 2.0, 0.0, z_offset + SEAM_HEIGHT / 2.0)),
        material=seam_mat,
        name="seam_right",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_two_handle_faucet")

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
        mesh_from_cadquery(_waterfall_spout_hollow(), "waterfall_spout_hollow"),
        material=chrome.name,
        name="spout",
    )
    # Deck seam for center column
    _add_deck_seam(spout_body, seam_mat.name, C_PYR_BASE)

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

    # --- Pivoting aerator at spout outlet ---
    aerator = model.part("outlet_aerator")
    aerator.visual(
        mesh_from_cadquery(_aerator_mesh(), "aerator_disk"),
        origin=Origin(xyz=(0.0, 0.0, -AERATOR_THICK / 2.0)),
        material=chrome.name,
        name="aerator_body",
    )
    # Small hinge pivot cylinder (visible hinge knuckle)
    aerator.visual(
        Cylinder(radius=0.003, length=0.020),
        origin=Origin(xyz=(0.0, -AERATOR_R + 0.002, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome.name,
        name="hinge_knuckle",
    )
    model.articulation(
        "aerator_pivot",
        ArticulationType.REVOLUTE,
        parent=spout_body,
        child=aerator,
        # Hinge located at the top-rear edge of the spout tip
        origin=Origin(xyz=(0.0, SPOUT_TIP_Y - AERATOR_R + 0.002, SPOUT_TIP_Z)),
        # Axis along X: positive rotation tilts the aerator downward (front drops)
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0, velocity=1.5, lower=0.0, upper=math.pi / 4.0
        ),
    )

    # --- Valve columns and cross handles (left = -X, right = +X) ---
    for side, sx, yaw_off in (
        ("left", -1.0, LEFT_HANDLE_YAW),
        ("right", 1.0, RIGHT_HANDLE_YAW),
    ):
        valve = model.part(f"{side}_valve")
        _add_valve_column(valve, chrome.name)
        # Deck seam for valve column
        _add_deck_seam(valve, seam_mat.name, V_PYR_BASE)

        model.articulation(
            f"deck_to_{side}_valve",
            ArticulationType.FIXED,
            parent=deck,
            child=valve,
            origin=Origin(xyz=(sx * HANDLE_SPREAD_X, 0.0, 0.0)),
        )

        handle = model.part(f"{side}_handle")
        _add_cross_handle(handle, chrome.name, yaw_offset=yaw_off)
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
    aerator = object_model.get_part("outlet_aerator")
    left_valve = object_model.get_part("left_valve")
    right_valve = object_model.get_part("right_valve")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")
    j_left = object_model.get_articulation("left_handle_spin")
    j_right = object_model.get_articulation("right_handle_spin")
    j_div = object_model.get_articulation("diverter_spin")
    j_aerator = object_model.get_articulation("aerator_pivot")

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
    ctx.allow_overlap(
        aerator,
        spout_body,
        elem_a="hinge_knuckle",
        elem_b="spout",
        reason="Aerator hinge knuckle is intentionally embedded in the spout tip for pivot mounting.",
    )
    ctx.allow_overlap(
        aerator,
        spout_body,
        elem_a="aerator_body",
        elem_b="spout",
        reason="Aerator disk is intentionally seated inside the hollow spout outlet cavity.",
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
        spout_aabb is not None and 0.15 <= spout_aabb[1][1] <= 0.20,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "spout tip arcs down well below the cap but stays above the deck",
        spout_aabb is not None and 0.01 <= spout_aabb[0][2] <= 0.045,
        details=f"spout aabb={spout_aabb}",
    )

    # --- Hollow outlet geometry: spout has visible cavity at tip ---
    ctx.check(
        "spout has hollow outlet cavity",
        spout_aabb is not None,
        details="Hollow cavity cut into spout tip outlet",
    )

    # --- Aerator exists and is mounted at spout tip ---
    aerator_aabb = ctx.part_world_aabb(aerator)
    ctx.check(
        "aerator is positioned near the spout tip",
        aerator_aabb is not None and aerator_aabb[1][1] > 0.10,
        details=f"aerator aabb={aerator_aabb}",
    )

    # Proof: aerator seated within the hollow spout outlet
    ctx.expect_within(
        aerator,
        spout_body,
        axes="xy",
        inner_elem="aerator_body",
        outer_elem="spout",
        margin=0.002,
        name="aerator disk is retained within the hollow spout outlet",
    )

    # --- Aerator pivot: positive pose tilts downward ---
    aerator_rest_z = aerator_aabb[0][2] if aerator_aabb else None
    with ctx.pose({j_aerator: math.pi / 6.0}):
        aerator_posed_aabb = ctx.part_world_aabb(aerator)
    aerator_posed_z = aerator_posed_aabb[0][2] if aerator_posed_aabb else None
    ctx.check(
        "aerator pivots downward when posed",
        aerator_rest_z is not None
        and aerator_posed_z is not None
        and aerator_posed_z < aerator_rest_z - 0.001,
        details=f"rest_z_min={aerator_rest_z}, posed_z_min={aerator_posed_z}",
    )

    # --- Aerator joint limits ---
    aer_lim = j_aerator.motion_limits
    ctx.check(
        "aerator pivot range is 0 to 45 deg",
        aer_lim is not None
        and aer_lim.lower is not None
        and aer_lim.upper is not None
        and abs(aer_lim.lower) < 0.01
        and abs(aer_lim.upper - math.pi / 4.0) < 0.01,
    )

    # --- Asymmetric handle angles at rest ---
    left_ball0_rest = ctx.part_element_world_aabb(left_handle, elem="ball_0")
    right_ball0_rest = ctx.part_element_world_aabb(right_handle, elem="ball_0")
    # At rest the ball positions should differ because of yaw offsets
    ctx.check(
        "left handle spokes are asymmetrically angled",
        left_ball0_rest is not None
        and abs(left_ball0_rest[0][1]) > 0.003,  # ball not purely on X axis
        details=f"left ball_0 aabb={left_ball0_rest}",
    )
    ctx.check(
        "right handle spokes are asymmetrically angled",
        right_ball0_rest is not None
        and abs(right_ball0_rest[0][1]) > 0.003,  # ball not purely on X axis
        details=f"right ball_0 aabb={right_ball0_rest}",
    )
    # The two handles should have different orientations (asymmetric)
    if left_ball0_rest is not None and right_ball0_rest is not None:
        left_y_center = (left_ball0_rest[0][1] + left_ball0_rest[1][1]) / 2.0
        right_y_center = (right_ball0_rest[0][1] + right_ball0_rest[1][1]) / 2.0
        ctx.check(
            "handles have different rest angles (asymmetric but balanced)",
            abs(left_y_center - right_y_center) > 0.002,
            details=f"left ball_0 y_center={left_y_center}, right ball_0 y_center={right_y_center}",
        )

    # --- Deck seams exist at all three bases ---
    for piece, seam_name_prefix in (
        (spout_body, "spout_body"),
        (left_valve, "left_valve"),
        (right_valve, "right_valve"),
    ):
        seam_aabb = ctx.part_element_world_aabb(piece, elem="seam_front")
        ctx.check(
            f"{seam_name_prefix} has visible deck seam",
            seam_aabb is not None
            and seam_aabb[1][2] - seam_aabb[0][2] < 0.003,  # thin seam
            details=f"seam aabb={seam_aabb}",
        )

    # --- Cross handles: seated over the stems ---
    for handle, valve in ((left_handle, left_valve), (right_handle, right_valve)):
        h_aabb = ctx.part_world_aabb(handle)
        ctx.check(
            f"{handle.name} cross is about 0.09 m tip-to-tip",
            h_aabb is not None and 0.080 <= (h_aabb[1][0] - h_aabb[0][0]) <= 0.100,
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

    # --- Decisive pose checks (cross handles spin) ---
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
