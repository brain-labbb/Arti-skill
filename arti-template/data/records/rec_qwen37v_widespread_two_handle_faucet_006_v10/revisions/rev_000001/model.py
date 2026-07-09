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
# Art-Deco widespread two-handle faucet (variant 10), mirror chrome.
#
# Layout (meters, Z up, spout sweeps forward along +Y):
#   - dark deck plate (root) with three chrome pieces mounted on top (z = 0)
#   - center spout column at x = 0: tapered square-pyramid base (0.07 sq at
#     deck -> 0.046 sq at z = 0.08), stepped square cap, flat-topped waterfall
#     spout reaching ~0.18 forward; entire spout body swivels on a continuous
#     vertical joint; oval finial diverter on top
#   - valve columns at x = +/-0.15: smaller tapered pyramids (0.06 sq, 0.07
#     tall) with decorative raised square-section ring ridges, square cap,
#     slim stem carrying a four-spoke cross handle (0.09 tip-to-tip, ball ends)
#   - narrow dark seams at all three deck bases
#   - left handle spokes angled ~22.5° off cardinal, right at 0° (asymmetric
#     but balanced)
# Articulation: spout continuous about vertical; each cross handle revolute
# (-pi..pi); oval finial revolute diverter (-pi/2..pi/2).
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

# Decorative rings on valve pedestals
RING_THICKNESS = 0.003  # how far the ring stands proud
RING_HEIGHT = 0.004  # vertical band height
RING_POSITIONS_FRAC = (0.30, 0.60)  # fractional heights on the pyramid


def _pyramid_frustum(base: float, top: float, height: float) -> cq.Workplane:
    """Tapered square-pyramid column, base on z=0, narrowing upward."""
    return (
        cq.Workplane("XY")
        .rect(base, base)
        .workplane(offset=height)
        .rect(top, top)
        .loft(combine=True)
    )


def _pyramid_width_at(base: float, top: float, height: float, z: float) -> float:
    """Linear interpolation of square-pyramid width at height z."""
    frac = max(0.0, min(1.0, z / height))
    return base + (top - base) * frac


def _square_ring(outer: float, inner: float, h: float) -> cq.Workplane:
    """Hollow square-section ring (washer), base on z=0."""
    solid = cq.Workplane("XY").box(outer, outer, h)
    hole = cq.Workplane("XY").box(inner, inner, h + 0.002)
    return solid.cut(hole)


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


def _add_cross_handle(part: Part, chrome: str, angle_offset: float = 0.0) -> None:
    """Four-spoke cross handle with ball ends, rotating about local +Z.

    Local frame origin is the handle joint frame: hub bottom at z=0.
    angle_offset rotates the spoke layout about Z (radians).
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
    ca = math.cos(angle_offset)
    sa = math.sin(angle_offset)
    spoke_dirs = [
        ((SPOKE_LEN / 2.0, 0.0, SPOKE_Z), (0.0, math.pi / 2.0, 0.0)),
        ((-SPOKE_LEN / 2.0, 0.0, SPOKE_Z), (0.0, math.pi / 2.0, 0.0)),
        ((0.0, SPOKE_LEN / 2.0, SPOKE_Z), (math.pi / 2.0, 0.0, 0.0)),
        ((0.0, -SPOKE_LEN / 2.0, SPOKE_Z), (math.pi / 2.0, 0.0, 0.0)),
    ]
    for i, (xyz_unrot, rpy) in enumerate(spoke_dirs):
        # Rotate position about Z by angle_offset
        x = xyz_unrot[0] * ca - xyz_unrot[1] * sa
        y = xyz_unrot[0] * sa + xyz_unrot[1] * ca
        z = xyz_unrot[2]
        # Add the offset to the yaw component of rpy
        new_rpy = (rpy[0], rpy[1], rpy[2] + angle_offset)
        part.visual(
            Cylinder(radius=SPOKE_R, length=SPOKE_LEN),
            origin=Origin(xyz=(x, y, z), rpy=new_rpy),
            material=chrome,
            name=f"spoke_{i}",
        )
    ball_centers_unrot = [
        (BALL_C, 0.0, SPOKE_Z),
        (-BALL_C, 0.0, SPOKE_Z),
        (0.0, BALL_C, SPOKE_Z),
        (0.0, -BALL_C, SPOKE_Z),
    ]
    for i, (bx, by, bz) in enumerate(ball_centers_unrot):
        rx = bx * ca - by * sa
        ry = bx * sa + by * ca
        part.visual(
            Sphere(radius=BALL_R),
            origin=Origin(xyz=(rx, ry, bz)),
            material=chrome,
            name=f"ball_{i}",
        )


def _add_valve_column(part: Part, chrome: str, seam_mat: str) -> None:
    """Tapered pyramid valve base with decorative ring ridges, seam, cap, stem."""
    # Base seam (thin dark square ring at deck level)
    seam_outer = V_PYR_BASE + 0.004  # slightly wider than base
    seam_inner = V_PYR_BASE - 0.001
    part.visual(
        mesh_from_cadquery(
            _square_ring(seam_outer, seam_inner, 0.002),
            f"{part.name}_seam",
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
        material=seam_mat,
        name="base_seam",
    )
    # Pyramid body
    part.visual(
        mesh_from_cadquery(
            _pyramid_frustum(V_PYR_BASE, V_PYR_TOP, V_PYR_H),
            f"{part.name}_pyramid",
        ),
        material=chrome,
        name="valve_pyramid",
    )
    # Decorative ring ridges on pedestal
    for i, frac in enumerate(RING_POSITIONS_FRAC):
        z_pos = frac * V_PYR_H
        w = _pyramid_width_at(V_PYR_BASE, V_PYR_TOP, V_PYR_H, z_pos)
        ring_outer = w + 2 * RING_THICKNESS
        ring_inner = w - 0.001  # sits just inside surface, proud by RING_THICKNESS
        part.visual(
            mesh_from_cadquery(
                _square_ring(ring_outer, ring_inner, RING_HEIGHT),
                f"{part.name}_ring_{i}",
            ),
            origin=Origin(xyz=(0.0, 0.0, z_pos)),
            material=chrome,
            name=f"deco_ring_{i}",
        )
    # Cap
    part.visual(
        Box((V_CAP_SIZE, V_CAP_SIZE, V_CAP_H)),
        origin=Origin(xyz=(0.0, 0.0, V_PYR_H + V_CAP_H / 2.0)),
        material=chrome,
        name="valve_cap",
    )
    # Stem
    stem_z0 = V_PYR_H + V_CAP_H / 2.0
    part.visual(
        Cylinder(radius=V_STEM_R, length=V_STEM_TOP_Z - stem_z0),
        origin=Origin(xyz=(0.0, 0.0, (stem_z0 + V_STEM_TOP_Z) / 2.0)),
        material=chrome,
        name="valve_stem",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="art_deco_widespread_faucet")

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

    # --- Center spout column (swivels continuously) ---
    spout_body = model.part("spout_body")
    # Base seam for center column
    c_seam_outer = C_PYR_BASE + 0.004
    c_seam_inner = C_PYR_BASE - 0.001
    spout_body.visual(
        mesh_from_cadquery(
            _square_ring(c_seam_outer, c_seam_inner, 0.002),
            "center_seam",
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
        material=seam_mat.name,
        name="base_seam",
    )
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
    # Continuous vertical swivel joint for the spout
    model.articulation(
        "deck_to_spout_body",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=spout_body,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0),
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

    # --- Valve columns and cross handles (left = -X, right = +X) ---
    # Asymmetric spoke angles: left at 22.5°, right at 0° (balanced but different)
    handle_angles = {"left": math.pi / 8.0, "right": 0.0}

    for side, sx in (("left", -1.0), ("right", 1.0)):
        valve = model.part(f"{side}_valve")
        _add_valve_column(valve, chrome.name, seam_mat.name)
        model.articulation(
            f"deck_to_{side}_valve",
            ArticulationType.FIXED,
            parent=deck,
            child=valve,
            origin=Origin(xyz=(sx * HANDLE_SPREAD_X, 0.0, 0.0)),
        )

        handle = model.part(f"{side}_handle")
        _add_cross_handle(handle, chrome.name, angle_offset=handle_angles[side])
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
    j_spout = object_model.get_articulation("deck_to_spout_body")
    j_left = object_model.get_articulation("left_handle_spin")
    j_right = object_model.get_articulation("right_handle_spin")
    j_div = object_model.get_articulation("diverter_spin")

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

    # --- Spout body is a CONTINUOUS joint about vertical ---
    ctx.check(
        "spout joint is continuous type",
        j_spout.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={j_spout.articulation_type}",
    )
    ctx.check(
        "spout joint axis is vertical",
        j_spout.axis is not None
        and abs(j_spout.axis[0]) < 0.01
        and abs(j_spout.axis[1]) < 0.01
        and abs(abs(j_spout.axis[2]) - 1.0) < 0.01,
        details=f"axis={j_spout.axis}",
    )

    # --- Spout swivel pose check: rotating 45° moves the spout tip laterally ---
    spout_rest_aabb = ctx.part_element_world_aabb(spout_body, elem="spout")
    with ctx.pose({j_spout: math.pi / 4.0}):
        spout_posed_aabb = ctx.part_element_world_aabb(spout_body, elem="spout")
    ctx.check(
        "spout swivels about vertical axis (45° pose shifts tip X)",
        spout_rest_aabb is not None
        and spout_posed_aabb is not None
        and abs(
            (spout_posed_aabb[0][0] + spout_posed_aabb[1][0]) / 2.0
            - (spout_rest_aabb[0][0] + spout_rest_aabb[1][0]) / 2.0
        )
        > 0.02,
        details=f"rest={spout_rest_aabb}, posed={spout_posed_aabb}",
    )

    # --- All three chrome pieces seated on the dark deck, not floating ---
    for piece in (spout_body, left_valve, right_valve):
        ctx.expect_gap(
            piece,
            deck,
            axis="z",
            max_gap=0.001,
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

    # --- Narrow seams at all three deck bases ---
    for piece in (spout_body, left_valve, right_valve):
        seam_aabb = ctx.part_element_world_aabb(piece, elem="base_seam")
        ctx.check(
            f"{piece.name} has a visible base seam",
            seam_aabb is not None and (seam_aabb[1][0] - seam_aabb[0][0]) > 0.05,
            details=f"seam aabb={seam_aabb}",
        )

    # --- Decorative ring ridges on handle pedestals ---
    for valve in (left_valve, right_valve):
        for ring_name in ("deco_ring_0", "deco_ring_1"):
            ring_aabb = ctx.part_element_world_aabb(valve, elem=ring_name)
            ctx.check(
                f"{valve.name} has decorative {ring_name}",
                ring_aabb is not None
                and (ring_aabb[1][0] - ring_aabb[0][0]) > 0.03
                and (ring_aabb[1][2] - ring_aabb[0][2]) < 0.010,
                details=f"{ring_name} aabb={ring_aabb}",
            )

    # --- Asymmetric handle spoke angles: left rotated, right at cardinal ---
    # At rest, left handle ball_0 should not be on the +X axis (rotated 22.5°),
    # right handle ball_0 should be near +X axis (0° offset).
    left_ball0 = ctx.part_element_world_aabb(left_handle, elem="ball_0")
    right_ball0 = ctx.part_element_world_aabb(right_handle, elem="ball_0")
    if left_ball0 is not None:
        left_ball_cx = (left_ball0[0][0] + left_ball0[1][0]) / 2.0
        left_ball_cy = (left_ball0[0][1] + left_ball0[1][1]) / 2.0
        left_handle_pos = ctx.part_world_position(left_handle)
        if left_handle_pos is not None:
            dx_l = left_ball_cx - left_handle_pos[0]
            dy_l = left_ball_cy - left_handle_pos[1]
            # At 22.5° offset, ball_0 should have significant Y component
            ctx.check(
                "left handle spokes angled asymmetrically (ball_0 off cardinal X)",
                abs(dy_l) > 0.010,
                details=f"ball_0 offset from handle center: dx={dx_l:.4f}, dy={dy_l:.4f}",
            )
    if right_ball0 is not None:
        right_ball_cx = (right_ball0[0][0] + right_ball0[1][0]) / 2.0
        right_ball_cy = (right_ball0[0][1] + right_ball0[1][1]) / 2.0
        right_handle_pos = ctx.part_world_position(right_handle)
        if right_handle_pos is not None:
            dx_r = right_ball_cx - right_handle_pos[0]
            dy_r = right_ball_cy - right_handle_pos[1]
            # At 0° offset, ball_0 should be near pure +X (dy ≈ 0)
            ctx.check(
                "right handle spokes at cardinal orientation (ball_0 near +X)",
                abs(dy_r) < 0.005,
                details=f"ball_0 offset from handle center: dx={dx_r:.4f}, dy={dy_r:.4f}",
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
        if h_aabb is not None:
            dx = h_aabb[1][0] - h_aabb[0][0]
            dy = h_aabb[1][1] - h_aabb[0][1]
            max_extent = max(dx, dy)
        else:
            max_extent = 0.0
        ctx.check(
            f"{handle.name} cross is about 0.09 m tip-to-tip",
            h_aabb is not None and 0.080 <= max_extent <= 0.098,
            details=f"{handle.name} max_extent={max_extent:.4f}, aabb={h_aabb}",
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

    # --- Decisive pose checks for handle rotation ---
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
