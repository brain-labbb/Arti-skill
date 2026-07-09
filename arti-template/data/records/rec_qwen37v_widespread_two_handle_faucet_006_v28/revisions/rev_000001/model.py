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
# Variant 28: widespread two-handle faucet with short rectangular waterfall
# channel spout, pivoting outlet aerator on a small hinge, narrow deck-base
# seams on all three columns, and decorative ring ridges on the handle
# pedestals.  Mirror chrome on a dark deck, Art-Deco styling.
#
# Layout (meters, Z up, spout extends along +Y):
#   - dark deck plate (root), top face at z = 0
#   - center column at x = 0: tapered square-pyramid base (0.07 sq, 0.08 tall),
#     stepped cap, short U-channel waterfall spout (~0.10 m reach), oval finial
#     diverter on cap, pivoting aerator on channel outlet
#   - valve columns at x = ±0.15: tapered pyramids (0.06 sq, 0.07 tall) with
#     two decorative ring ridges, square cap, slim stem, four-spoke cross handle
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

# Short rectangular waterfall channel
SPOUT_WIDTH = 0.044
SPOUT_HEIGHT = 0.018
SPOUT_WALL = 0.004
SPOUT_LENGTH = 0.115
SPOUT_Y_OFFSET = -0.015
SPOUT_OUTLET_Y = SPOUT_Y_OFFSET + SPOUT_LENGTH  # 0.100

# Aerator
AERATOR_WIDTH = 0.034
AERATOR_LENGTH = 0.018
AERATOR_THICK = 0.003
AERATOR_HINGE_Y = SPOUT_OUTLET_Y  # 0.100
AERATOR_HINGE_Z = CAP_TOP_Z - SPOUT_HEIGHT  # 0.080

# Finial diverter
FINIAL_RX = 0.018
FINIAL_RY = 0.012
FINIAL_RZ = 0.008
FINIAL_STEM_R = 0.0045
FINIAL_CENTER_Z = 0.014

# Ring ridge parameters
RING_PROTRUSION = 0.005
RING_HEIGHT = 0.003
RING_HEIGHTS = (0.020, 0.045)

# Seam parameters
SEAM_WIDTH = 0.0015
SEAM_HEIGHT = 0.001


def _pyramid_frustum(base: float, top: float, height: float) -> cq.Workplane:
    """Tapered square-pyramid column, base on z=0, narrowing upward."""
    return (
        cq.Workplane("XY")
        .rect(base, base)
        .workplane(offset=height)
        .rect(top, top)
        .loft(combine=True)
    )


def _valve_column_with_rings() -> cq.Workplane:
    """Valve pyramid with two decorative square ring ridges fused in."""
    pyramid = _pyramid_frustum(V_PYR_BASE, V_PYR_TOP, V_PYR_H)
    result = pyramid
    for h in RING_HEIGHTS:
        side_at_h = V_PYR_BASE + (V_PYR_TOP - V_PYR_BASE) * h / V_PYR_H
        outer_side = side_at_h + 2.0 * RING_PROTRUSION
        # Solid square ridge that protrudes from the pyramid surface;
        # the pyramid fills the interior so only the rim is visible.
        ring = (
            cq.Workplane("XY")
            .workplane(offset=h)
            .rect(outer_side, outer_side)
            .extrude(RING_HEIGHT)
        )
        result = result.union(ring)
    return result


def _waterfall_channel() -> cq.Workplane:
    """Short rectangular U-channel spout extending along +Y.

    Built from three boxes: top plate + two side walls.
    Top plate at z = 0, walls extend down to z = -SPOUT_HEIGHT.
    Channel extends from y = 0 to y = SPOUT_LENGTH.
    """
    w = SPOUT_WIDTH
    h = SPOUT_HEIGHT
    t = SPOUT_WALL
    L = SPOUT_LENGTH

    top = cq.Workplane("XY").box(w, L, t).translate((0.0, L / 2.0, -t / 2.0))
    left_wall = (
        cq.Workplane("XY")
        .box(t, L, h)
        .translate((-w / 2.0 + t / 2.0, L / 2.0, -h / 2.0))
    )
    right_wall = (
        cq.Workplane("XY")
        .box(t, L, h)
        .translate((w / 2.0 - t / 2.0, L / 2.0, -h / 2.0))
    )
    return top.union(left_wall).union(right_wall)


def _base_seam(base_side: float) -> cq.Workplane:
    """Thin dark seam frame at the column base perimeter."""
    outer = base_side + 2.0 * SEAM_WIDTH
    return (
        cq.Workplane("XY")
        .rect(outer, outer)
        .rect(base_side, base_side)
        .extrude(SEAM_HEIGHT)
    )


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


# ---- build ----------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_waterfall_faucet")

    chrome = model.material("chrome", rgba=(0.88, 0.89, 0.92, 1.0))
    deck_mat = model.material("deck_charcoal", rgba=(0.09, 0.09, 0.10, 1.0))
    seam_mat = model.material("seam_dark", rgba=(0.04, 0.04, 0.05, 1.0))

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
        origin=Origin(xyz=(0.0, SPOUT_Y_OFFSET, CAP_TOP_Z)),
        material=chrome.name,
        name="spout_channel",
    )
    # Hinge barrel at channel outlet (parent-side hinge hardware)
    # Spans the channel interior and embeds into the side walls for connectivity.
    _barrel_len = SPOUT_WIDTH - 2.0 * SPOUT_WALL + 0.004
    spout_body.visual(
        Cylinder(radius=0.003, length=_barrel_len),
        origin=Origin(
            xyz=(0.0, AERATOR_HINGE_Y, AERATOR_HINGE_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=chrome.name,
        name="hinge_barrel",
    )
    # Narrow seam at the deck base
    spout_body.visual(
        mesh_from_cadquery(_base_seam(C_PYR_BASE), "center_seam"),
        material=seam_mat.name,
        name="base_seam",
    )
    model.articulation(
        "deck_to_spout_body",
        ArticulationType.FIXED,
        parent=deck,
        child=spout_body,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # --- Aerator (pivots downward on small hinge) ---
    aerator = model.part("aerator")
    # Flat rectangular plate extending forward from hinge
    aerator.visual(
        Box((AERATOR_WIDTH, AERATOR_LENGTH, AERATOR_THICK)),
        origin=Origin(xyz=(0.0, AERATOR_LENGTH / 2.0, -AERATOR_THICK / 2.0)),
        material=chrome.name,
        name="aerator_plate",
    )
    # Hinge pin (child-side, coaxial with barrel)
    aerator.visual(
        Cylinder(radius=0.002, length=0.010),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=chrome.name,
        name="hinge_pin",
    )
    model.articulation(
        "aerator_hinge",
        ArticulationType.REVOLUTE,
        parent=spout_body,
        child=aerator,
        origin=Origin(xyz=(0.0, AERATOR_HINGE_Y, AERATOR_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0, velocity=1.5, lower=0.0, upper=0.70
        ),
    )

    # --- Oval finial diverter button on the cap ---
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

    # --- Valve columns and cross handles (left = -X, right = +X) ---
    for side, sx in (("left", -1.0), ("right", 1.0)):
        valve = model.part(f"{side}_valve")
        # Combined pyramid + decorative ring ridges
        valve.visual(
            mesh_from_cadquery(
                _valve_column_with_rings(), f"{side}_valve_column"
            ),
            material=chrome.name,
            name="valve_column",
        )
        valve.visual(
            Box((V_CAP_SIZE, V_CAP_SIZE, V_CAP_H)),
            origin=Origin(xyz=(0.0, 0.0, V_PYR_H + V_CAP_H / 2.0)),
            material=chrome.name,
            name="valve_cap",
        )
        stem_z0 = V_PYR_H + V_CAP_H / 2.0
        valve.visual(
            Cylinder(radius=V_STEM_R, length=V_STEM_TOP_Z - stem_z0),
            origin=Origin(xyz=(0.0, 0.0, (stem_z0 + V_STEM_TOP_Z) / 2.0)),
            material=chrome.name,
            name="valve_stem",
        )
        # Narrow seam at the deck base
        valve.visual(
            mesh_from_cadquery(_base_seam(V_PYR_BASE), f"{side}_seam"),
            material=seam_mat.name,
            name="base_seam",
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


# ---- tests ----------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    spout_body = object_model.get_part("spout_body")
    aerator = object_model.get_part("aerator")
    finial = object_model.get_part("diverter_finial")
    left_valve = object_model.get_part("left_valve")
    right_valve = object_model.get_part("right_valve")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")
    j_left = object_model.get_articulation("left_handle_spin")
    j_right = object_model.get_articulation("right_handle_spin")
    j_div = object_model.get_articulation("diverter_spin")
    j_aerator = object_model.get_articulation("aerator_hinge")

    # --- Intentional captured fits ---
    ctx.allow_overlap(
        left_handle, left_valve,
        elem_a="hub", elem_b="valve_stem",
        reason="Cross-handle hub captures the valve bonnet stem.",
    )
    ctx.allow_overlap(
        right_handle, right_valve,
        elem_a="hub", elem_b="valve_stem",
        reason="Cross-handle hub captures the valve bonnet stem.",
    )
    ctx.allow_overlap(
        finial, spout_body,
        elem_a="finial_stem", elem_b="cap_step_upper",
        reason="Finial stem seated 2 mm into the stepped cap.",
    )
    ctx.allow_overlap(
        aerator, spout_body,
        elem_a="hinge_pin", elem_b="hinge_barrel",
        reason="Aerator hinge pin nested inside hinge barrel at channel outlet.",
    )
    ctx.allow_overlap(
        aerator, spout_body,
        elem_a="aerator_plate", elem_b="hinge_barrel",
        reason="Aerator plate wraps the hinge barrel at the pivot connection.",
    )

    # --- Three chrome pieces seated on the dark deck ---
    for piece in (spout_body, left_valve, right_valve):
        ctx.expect_gap(
            piece, deck, axis="z",
            max_gap=0.002, max_penetration=0.001,
            name=f"{piece.name} base seated on deck",
        )
        ctx.expect_within(
            piece, deck, axes="x", margin=0.002,
            name=f"{piece.name} within deck plate footprint",
        )

    # --- Three-piece spread about 0.30 m ---
    ctx.expect_origin_distance(
        left_handle, right_handle, axes="x",
        min_dist=0.29, max_dist=0.31,
        name="handle spread is about 0.30 m",
    )

    # --- Short rectangular waterfall channel ---
    channel_aabb = ctx.part_element_world_aabb(spout_body, elem="spout_channel")
    ctx.check(
        "spout channel is short (forward reach < 0.13 m from center)",
        channel_aabb is not None and channel_aabb[1][1] < 0.13,
        details=f"channel aabb={channel_aabb}",
    )
    ctx.check(
        "spout channel extends forward beyond the center column",
        channel_aabb is not None and channel_aabb[1][1] > 0.06,
        details=f"channel aabb={channel_aabb}",
    )
    ctx.check(
        "channel width is about 0.044 m",
        channel_aabb is not None
        and 0.040 <= (channel_aabb[1][0] - channel_aabb[0][0]) <= 0.048,
        details=f"channel aabb={channel_aabb}",
    )
    ctx.check(
        "channel has visible depth (U-channel walls)",
        channel_aabb is not None
        and 0.014 <= (channel_aabb[1][2] - channel_aabb[0][2]) <= 0.022,
        details=f"channel aabb={channel_aabb}",
    )

    # --- Aerator hinge mechanism ---
    aerator_rest = ctx.part_element_world_aabb(aerator, elem="aerator_plate")
    ctx.check(
        "aerator is positioned at the channel outlet area",
        aerator_rest is not None and aerator_rest[1][1] > 0.07,
        details=f"aerator rest aabb={aerator_rest}",
    )

    # Aerator hinge limits: 0 to ~0.7 rad downward
    aero_lim = j_aerator.motion_limits
    ctx.check(
        "aerator hinge range is 0 to ~0.7 rad downward",
        aero_lim is not None
        and aero_lim.lower is not None
        and aero_lim.upper is not None
        and abs(aero_lim.lower) < 0.01
        and 0.50 <= aero_lim.upper <= 0.90,
    )

    # Decisive pose: aerator tilts downward when hinge opens
    with ctx.pose({j_aerator: 0.50}):
        aerator_posed = ctx.part_element_world_aabb(aerator, elem="aerator_plate")
    ctx.check(
        "aerator pivots downward when hinge opens",
        aerator_rest is not None and aerator_posed is not None
        and aerator_posed[0][2] < aerator_rest[0][2] - 0.003,
        details=(
            f"rest_min_z={aerator_rest[0][2] if aerator_rest else None}, "
            f"posed_min_z={aerator_posed[0][2] if aerator_posed else None}"
        ),
    )

    # --- Decorative ring ridges on valve pedestals ---
    for valve in (left_valve, right_valve):
        col_aabb = ctx.part_element_world_aabb(valve, elem="valve_column")
        ctx.check(
            f"{valve.name} column wider than base pyramid (ring ridges present)",
            col_aabb is not None
            and (col_aabb[1][0] - col_aabb[0][0]) > V_PYR_BASE + 0.002,
            details=f"column aabb={col_aabb}",
        )

    # --- Narrow seams at all three deck bases ---
    for piece in (spout_body, left_valve, right_valve):
        seam_aabb = ctx.part_element_world_aabb(piece, elem="base_seam")
        ctx.check(
            f"{piece.name} has a visible base seam near the deck",
            seam_aabb is not None
            and seam_aabb[0][2] < 0.003
            and seam_aabb[1][2] < 0.005,
            details=f"seam aabb={seam_aabb}",
        )

    # --- Cross handles ---
    for handle, valve in ((left_handle, left_valve), (right_handle, right_valve)):
        h_aabb = ctx.part_world_aabb(handle)
        ctx.check(
            f"{handle.name} cross is about 0.09 m tip-to-tip",
            h_aabb is not None and 0.086 <= (h_aabb[1][0] - h_aabb[0][0]) <= 0.094,
            details=f"{handle.name} aabb={h_aabb}",
        )
        ctx.expect_gap(
            handle, valve, axis="z",
            max_gap=0.0005, max_penetration=0.004,
            name=f"{handle.name} hub seats over valve stem",
        )

    # --- Handle joint limits ---
    for joint in (j_left, j_right):
        lim = joint.motion_limits
        ctx.check(
            f"{joint.name} range is -180..+180 deg",
            lim is not None
            and abs(lim.lower + math.pi) < 0.01
            and abs(lim.upper - math.pi) < 0.01,
        )

    # Handle spin pose check
    def _ball_center(handle: Part) -> tuple[float, float] | None:
        aabb = ctx.part_element_world_aabb(handle, elem="ball_0")
        if aabb is None:
            return None
        return ((aabb[0][0] + aabb[1][0]) / 2.0, (aabb[0][1] + aabb[1][1]) / 2.0)

    rest_left = _ball_center(left_handle)
    with ctx.pose({j_left: math.pi / 4.0}):
        posed_left = _ball_center(left_handle)
    ctx.check(
        "left handle spins about vertical stem axis",
        rest_left is not None and posed_left is not None
        and math.hypot(
            posed_left[0] - rest_left[0], posed_left[1] - rest_left[1]
        ) > 0.02,
        details=f"rest={rest_left}, posed={posed_left}",
    )

    # --- Finial diverter ---
    div_lim = j_div.motion_limits
    ctx.check(
        "diverter range is -90..+90 deg",
        div_lim is not None
        and abs(div_lim.lower + math.pi / 2.0) < 0.01
        and abs(div_lim.upper - math.pi / 2.0) < 0.01,
    )

    return ctx.report()


object_model = build_object_model()
