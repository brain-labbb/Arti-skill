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
# Widespread two-handle faucet variant 03, mirror chrome on dark deck.
#
# Layout (meters, Z up, spout sweeps forward along +Y):
#   - dark deck plate (root) with three chrome pieces mounted on top (z = 0)
#   - center spout column at x = 0: round tapered cylindrical base,
#     stepped round cap, flat-topped waterfall spout reaching ~0.18 forward,
#     oval finial diverter on top, pivoting aerator at spout outlet
#   - valve columns at x = +/-0.15: round tapered cylindrical bases with
#     round cap, visible stem collar, slim stem, and four-spoke cross handle
# Articulation: each cross handle revolute about its vertical stem axis
# (-pi..pi); the oval finial is a revolute diverter (-pi/2..pi/2);
# the outlet aerator pivots downward on a revolute hinge (0..0.8 rad).
# ---------------------------------------------------------------------------

HANDLE_SPREAD_X = 0.150  # valve column centers at +/-0.150 -> 0.30 m spread

# Center column (round base)
C_BASE_R = 0.035  # base radius at deck
C_TOP_R = 0.023   # top radius (tapered)
C_BASE_H = 0.080  # base height
CAP1_R = 0.028    # lower cap step radius
CAP1_H = 0.010
CAP2_R = 0.023    # upper cap step radius
CAP2_H = 0.008
CAP_TOP_Z = C_BASE_H + CAP1_H + CAP2_H  # 0.098

# Valve columns (round bases)
V_BASE_R = 0.030  # base radius at deck
V_TOP_R = 0.017   # top radius (tapered)
V_BASE_H = 0.070  # base height
V_CAP_R = 0.020   # round cap radius
V_CAP_H = 0.008
V_COLLAR_R = 0.014  # stem collar outer radius
V_COLLAR_H = 0.006  # stem collar height
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
AERATOR_R = 0.012
AERATOR_H = 0.014
AERATOR_PIVOT_Y = 0.168  # approximate Y position at spout tip
AERATOR_PIVOT_Z = 0.028  # approximate Z at spout tip underside


def _tapered_cylinder(base_r: float, top_r: float, height: float) -> cq.Workplane:
    """Tapered round cylinder column, base on z=0, narrowing upward."""
    return (
        cq.Workplane("XY")
        .circle(base_r)
        .workplane(offset=height)
        .circle(top_r)
        .loft(combine=True)
    )


def _waterfall_spout() -> cq.Workplane:
    """Wide flat-topped spout sweeping forward (+Y) into a waterfall arc.

    Side profile drawn in the YZ plane, extruded across X for flat
    Art-Deco slab sides. The root (y ~ 0.010) is buried inside the
    column so the spout reads as emerging from the body.
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


def _aerator_body() -> cq.Workplane:
    """Cylindrical aerator body with integrated hinge pin barrel.

    The vertical cylinder is the aerator outlet body. A small horizontal
    barrel at the top represents the hinge pivot, fused into one solid
    so the part reads as one connected mesh.
    """
    body = (
        cq.Workplane("XY")
        .circle(AERATOR_R)
        .extrude(AERATOR_H)
    )
    # Hinge barrel: horizontal cylinder at the top of the body,
    # offset to the front (+Y) edge where it meets the spout.
    pin = (
        cq.Workplane("XZ")
        .transformed(offset=(0.0, AERATOR_H, 0.0))
        .circle(0.004)
        .extrude(0.024)
        .translate((0.0, -0.012, 0.0))
    )
    return body.union(pin)


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
    """Round tapered valve base with round cap, stem collar, and slim stem."""
    # Round tapered base
    part.visual(
        mesh_from_cadquery(
            _tapered_cylinder(V_BASE_R, V_TOP_R, V_BASE_H),
            f"{part.name}_round_base",
        ),
        material=chrome,
        name="valve_base",
    )
    # Round cap
    part.visual(
        Cylinder(radius=V_CAP_R, length=V_CAP_H),
        origin=Origin(xyz=(0.0, 0.0, V_BASE_H + V_CAP_H / 2.0)),
        material=chrome,
        name="valve_cap",
    )
    # Visible stem collar (ring under the handle)
    collar_z0 = V_BASE_H + V_CAP_H
    part.visual(
        Cylinder(radius=V_COLLAR_R, length=V_COLLAR_H),
        origin=Origin(xyz=(0.0, 0.0, collar_z0 + V_COLLAR_H / 2.0)),
        material=chrome,
        name="stem_collar",
    )
    # Slim bonnet stem
    stem_z0 = collar_z0 + V_COLLAR_H
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

    # --- Dark deck plate (root) ---
    deck = model.part("deck")
    deck.visual(
        Box((0.42, 0.20, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),  # top face at z = 0
        material=deck_mat,
        name="deck_plate",
    )

    # --- Center spout column (round base) ---
    spout_body = model.part("spout_body")
    spout_body.visual(
        mesh_from_cadquery(
            _tapered_cylinder(C_BASE_R, C_TOP_R, C_BASE_H), "center_round_base"
        ),
        material=chrome.name,
        name="spout_base",
    )
    spout_body.visual(
        Cylinder(radius=CAP1_R, length=CAP1_H),
        origin=Origin(xyz=(0.0, 0.0, C_BASE_H + CAP1_H / 2.0)),
        material=chrome.name,
        name="cap_step_lower",
    )
    spout_body.visual(
        Cylinder(radius=CAP2_R, length=CAP2_H),
        origin=Origin(xyz=(0.0, 0.0, C_BASE_H + CAP1_H + CAP2_H / 2.0)),
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

    # --- Outlet aerator on a pivoting hinge at spout tip ---
    aerator = model.part("spout_aerator")
    # The aerator mesh combines the outlet body and hinge barrel into one solid.
    # Origin places the hinge barrel axis (top of mesh) at the pivot point.
    aerator.visual(
        mesh_from_cadquery(_aerator_body(), "aerator_assembly"),
        origin=Origin(xyz=(0.0, 0.0, -AERATOR_H)),
        material=chrome.name,
        name="aerator_mesh",
    )
    # Aerator pivots downward (tilts around X axis so positive q tilts outlet
    # downward away from the spout). Origin at the hinge barrel axis.
    model.articulation(
        "aerator_tilt",
        ArticulationType.REVOLUTE,
        parent=spout_body,
        child=aerator,
        origin=Origin(xyz=(0.0, AERATOR_PIVOT_Y, AERATOR_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=1.5, lower=0.0, upper=0.80
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
    spout_body = object_model.get_part("spout_body")
    finial = object_model.get_part("diverter_finial")
    aerator = object_model.get_part("spout_aerator")
    left_valve = object_model.get_part("left_valve")
    right_valve = object_model.get_part("right_valve")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")
    j_left = object_model.get_articulation("left_handle_spin")
    j_right = object_model.get_articulation("right_handle_spin")
    j_div = object_model.get_articulation("diverter_spin")
    j_aerator = object_model.get_articulation("aerator_tilt")

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
        elem_a="aerator_mesh",
        elem_b="spout",
        reason="Aerator hinge barrel is intentionally embedded at the spout tip outlet to represent the pivot mount.",
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

    # --- Round bases: verify valve and center bases are circular (equal XY) ---
    for valve in (left_valve, right_valve):
        base_aabb = ctx.part_element_world_aabb(valve, elem="valve_base")
        ctx.check(
            f"{valve.name} base is round (equal X and Y extents)",
            base_aabb is not None
            and abs(
                (base_aabb[1][0] - base_aabb[0][0])
                - (base_aabb[1][1] - base_aabb[0][1])
            )
            < 0.003,
            details=f"base aabb={base_aabb}",
        )
        ctx.check(
            f"{valve.name} round base diameter is about 0.06 m",
            base_aabb is not None
            and 0.056 <= (base_aabb[1][0] - base_aabb[0][0]) <= 0.064,
            details=f"base aabb={base_aabb}",
        )

    center_base_aabb = ctx.part_element_world_aabb(spout_body, elem="spout_base")
    ctx.check(
        "center spout base is round (equal X and Y extents)",
        center_base_aabb is not None
        and abs(
            (center_base_aabb[1][0] - center_base_aabb[0][0])
            - (center_base_aabb[1][1] - center_base_aabb[0][1])
        )
        < 0.003,
        details=f"center base aabb={center_base_aabb}",
    )
    ctx.check(
        "center round base diameter is about 0.07 m",
        center_base_aabb is not None
        and 0.066 <= (center_base_aabb[1][0] - center_base_aabb[0][0]) <= 0.074,
        details=f"center base aabb={center_base_aabb}",
    )

    # --- Visible stem collars under each handle ---
    for valve in (left_valve, right_valve):
        collar_aabb = ctx.part_element_world_aabb(valve, elem="stem_collar")
        ctx.check(
            f"{valve.name} has a visible stem collar",
            collar_aabb is not None
            and (collar_aabb[1][0] - collar_aabb[0][0]) > 0.020,
            details=f"collar aabb={collar_aabb}",
        )
        # Collar sits above the cap and below the handle joint
        ctx.check(
            f"{valve.name} stem collar is above the valve cap",
            collar_aabb is not None
            and collar_aabb[0][2] > V_BASE_H + V_CAP_H * 0.5,
            details=f"collar z_min={collar_aabb[0][2] if collar_aabb else None}",
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

    # --- Aerator mounted at spout tip (proof check for allowance) ---
    aerator_rest_aabb = ctx.part_world_aabb(aerator)
    ctx.expect_overlap(
        aerator,
        spout_body,
        axes="y",
        min_overlap=0.001,
        name="aerator overlaps spout tip on Y (hinge mount)",
    )
    ctx.expect_overlap(
        aerator,
        spout_body,
        axes="z",
        min_overlap=0.001,
        name="aerator shares Z range with spout tip (mounted at outlet)",
    )
    # Aerator hangs above the deck and below the spout pivot
    ctx.check(
        "aerator hangs above the deck surface",
        aerator_rest_aabb is not None and aerator_rest_aabb[0][2] > 0.005,
        details=f"aerator rest aabb={aerator_rest_aabb}",
    )
    ctx.check(
        "aerator hangs below the spout tip pivot at rest",
        aerator_rest_aabb is not None and aerator_rest_aabb[0][2] < AERATOR_PIVOT_Z,
        details=f"aerator rest aabb={aerator_rest_aabb}",
    )

    # --- Aerator hinge: revolute joint with valid limits ---
    aerator_lim = j_aerator.motion_limits
    ctx.check(
        "aerator tilt joint is revolute",
        j_aerator.articulation_type == ArticulationType.REVOLUTE,
    )
    ctx.check(
        "aerator tilt range is 0..0.80 rad (about 46 deg)",
        aerator_lim is not None
        and aerator_lim.lower is not None
        and aerator_lim.upper is not None
        and abs(aerator_lim.lower) < 0.01
        and abs(aerator_lim.upper - 0.80) < 0.01,
    )

    # Aerator at tilt: the bottom of the aerator body moves downward and forward.
    # Measure the aerator_mesh element bottom Z at rest vs tilted pose.
    rest_elem_aabb = ctx.part_element_world_aabb(aerator, elem="aerator_mesh")
    rest_z_min = rest_elem_aabb[0][2] if rest_elem_aabb else None
    with ctx.pose({j_aerator: 0.6}):
        tilted_elem_aabb = ctx.part_element_world_aabb(aerator, elem="aerator_mesh")
    tilted_z_min = tilted_elem_aabb[0][2] if tilted_elem_aabb else None
    ctx.check(
        "aerator tilts downward when joint is posed",
        rest_z_min is not None
        and tilted_z_min is not None
        and tilted_z_min < rest_z_min - 0.002,
        details=f"rest_z_min={rest_z_min}, tilted_z_min={tilted_z_min}",
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

    # --- Finial button seated on the cap top ---
    ctx.expect_gap(
        finial,
        spout_body,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.003,
        name="finial stem seats into the cap top",
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

    # --- Decisive pose checks: cross handle spins ---
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
