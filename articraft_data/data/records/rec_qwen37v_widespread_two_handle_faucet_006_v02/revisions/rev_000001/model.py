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
# Widespread two-handle faucet with low bridge arch spout, polished chrome.
#
# Layout (meters, Z up, arch sweeps forward along +Y):
#   - dark deck plate (root) at z = 0
#   - center bridge arch spout at x = 0: small tapered square base (0.050 m
#     at deck, 0.025 m tall) with a seam ring, a curved bridge arch body
#     reaching ~0.12 m forward with peak ~0.068 m, and a hollow cylindrical
#     outlet hanging from the arch underside
#   - left/right valve columns at x = ±0.150: tapered pyramid bases (0.060 m,
#     0.070 m tall) with seam ring, square cap, slim stem, four-spoke cross
#     handle (0.090 m tip-to-tip with ball ends)
#
# Articulation: each cross handle is a REVOLUTE joint about the vertical
# stem axis, range -π..+π.
# ---------------------------------------------------------------------------

HANDLE_SPREAD_X = 0.150  # 0.30 m total spread

# Center bridge arch
C_BASE_SIZE = 0.050
C_BASE_TOP = 0.036
C_BASE_H = 0.025
ARCH_WIDTH = 0.042
ARCH_PEAK_Z = 0.068
ARCH_REACH_Y = 0.120

# Hollow outlet
OUTLET_R_OUTER = 0.011
OUTLET_R_INNER = 0.007
OUTLET_H = 0.015
OUTLET_Y = 0.088
OUTLET_Z = 0.022

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

# Seams
SEAM_BORDER = 0.003
SEAM_H = 0.002


def _pyramid_frustum(base: float, top: float, height: float) -> cq.Workplane:
    """Tapered square-pyramid column, base on z=0, narrowing upward."""
    return (
        cq.Workplane("XY")
        .rect(base, base)
        .workplane(offset=height)
        .rect(top, top)
        .loft(combine=True)
    )


def _bridge_arch_body() -> cq.Workplane:
    """Low bridge arch spout body. Profile in YZ plane, extruded along X.

    The arch is a thick curved band: outer surface forms a gentle arch
    peaking at ~0.068 m, inner surface (underside) follows ~0.018 m lower,
    creating a visible arch opening. The rear is embedded slightly into
    the center base for structural connection.
    """
    profile = (
        cq.Workplane("YZ")
        .moveTo(-0.005, C_BASE_H - 0.005)
        # Outer curve (top of arch)
        .spline(
            [
                (0.020, 0.052),
                (0.050, ARCH_PEAK_Z),
                (0.080, 0.062),
                (0.105, 0.044),
                (ARCH_REACH_Y, 0.030),
            ],
            includeCurrent=True,
        )
        # Front edge
        .lineTo(ARCH_REACH_Y, 0.018)
        # Inner curve (underside of arch)
        .spline(
            [
                (0.105, 0.030),
                (0.080, 0.044),
                (0.050, 0.050),
                (0.020, 0.040),
                (-0.005, 0.015),
            ],
            includeCurrent=True,
        )
        .close()
        .extrude(ARCH_WIDTH)
    )
    return profile.translate((-ARCH_WIDTH / 2.0, 0.0, 0.0))


def _outlet_tube() -> cq.Workplane:
    """Hollow cylindrical outlet tube (annular cross-section)."""
    return (
        cq.Workplane("XY")
        .circle(OUTLET_R_OUTER)
        .circle(OUTLET_R_INNER)
        .extrude(OUTLET_H)
    )


def _seam_frame(outer_size: float) -> cq.Workplane:
    """Thin rectangular frame seam ring at deck level."""
    inner_size = outer_size - 2 * SEAM_BORDER
    return (
        cq.Workplane("XY")
        .rect(outer_size, outer_size)
        .rect(inner_size, inner_size)
        .extrude(SEAM_H)
    )


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
    """Tapered pyramid valve base with seam ring, square cap, slim stem."""
    part.visual(
        mesh_from_cadquery(
            _seam_frame(V_PYR_BASE + 2 * SEAM_BORDER), f"{part.name}_seam"
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=seam_mat,
        name="base_seam",
    )
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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_bridge_arch_faucet")

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

    # --- Center bridge arch spout ---
    spout_body = model.part("spout_body")
    # Seam ring at deck base
    spout_body.visual(
        mesh_from_cadquery(_seam_frame(C_BASE_SIZE + 2 * SEAM_BORDER), "center_seam"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=seam_mat,
        name="base_seam",
    )
    # Tapered square base
    spout_body.visual(
        mesh_from_cadquery(
            _pyramid_frustum(C_BASE_SIZE, C_BASE_TOP, C_BASE_H),
            "center_base_pyramid",
        ),
        material=chrome.name,
        name="spout_base",
    )
    # Bridge arch body
    spout_body.visual(
        mesh_from_cadquery(_bridge_arch_body(), "bridge_arch"),
        material=chrome.name,
        name="arch_body",
    )
    # Hollow outlet tube under arch
    spout_body.visual(
        mesh_from_cadquery(_outlet_tube(), "outlet_tube"),
        origin=Origin(xyz=(0.0, OUTLET_Y, OUTLET_Z)),
        material=chrome.name,
        name="outlet",
    )
    model.articulation(
        "deck_to_spout_body",
        ArticulationType.FIXED,
        parent=deck,
        child=spout_body,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # --- Valve columns and cross handles ---
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
    spout_body = object_model.get_part("spout_body")
    left_valve = object_model.get_part("left_valve")
    right_valve = object_model.get_part("right_valve")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")
    j_left = object_model.get_articulation("left_handle_spin")
    j_right = object_model.get_articulation("right_handle_spin")

    # Intentional captured fits: handle hubs over valve stems
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

    # --- All three pieces seated on deck ---
    for piece in (spout_body, left_valve, right_valve):
        ctx.expect_gap(
            piece,
            deck,
            axis="z",
            max_gap=0.004,
            max_penetration=0.001,
            name=f"{piece.name} base seated on deck",
        )

    # --- Three-piece widespread spread ~0.30 m ---
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
        min_gap=0.12,
        max_gap=0.16,
        name="right valve flanks the center spout",
    )
    ctx.expect_origin_gap(
        spout_body,
        left_valve,
        axis="x",
        min_gap=0.12,
        max_gap=0.16,
        name="left valve flanks the center spout",
    )

    # --- Bridge arch geometry: low peak, forward reach, visible opening ---
    arch_aabb = ctx.part_element_world_aabb(spout_body, elem="arch_body")
    ctx.check(
        "bridge arch peak is low (below 0.080 m)",
        arch_aabb is not None and arch_aabb[1][2] < 0.080,
        details=f"arch aabb={arch_aabb}",
    )
    ctx.check(
        "bridge arch reaches forward at least 0.08 m",
        arch_aabb is not None and arch_aabb[1][1] >= 0.08,
        details=f"arch aabb={arch_aabb}",
    )
    ctx.check(
        "bridge arch has visible opening (underside well above deck)",
        arch_aabb is not None and arch_aabb[0][2] > 0.008,
        details=f"arch aabb={arch_aabb}",
    )

    # --- Hollow central outlet geometry ---
    outlet_aabb = ctx.part_element_world_aabb(spout_body, elem="outlet")
    ctx.check(
        "hollow outlet hangs from arch underside (top below arch peak)",
        outlet_aabb is not None
        and arch_aabb is not None
        and outlet_aabb[1][2] < arch_aabb[1][2] - 0.010
        and outlet_aabb[0][2] > 0.005,
        details=f"outlet aabb={outlet_aabb}, arch aabb={arch_aabb}",
    )
    ctx.check(
        "outlet is forward-positioned under the arch reach",
        outlet_aabb is not None and outlet_aabb[0][1] > 0.05,
        details=f"outlet aabb={outlet_aabb}",
    )

    # --- Seams at all three deck bases ---
    for piece_name in ("spout_body", "left_valve", "right_valve"):
        piece = object_model.get_part(piece_name)
        seam_aabb = ctx.part_element_world_aabb(piece, elem="base_seam")
        ctx.check(
            f"{piece_name} has seam frame at deck base",
            seam_aabb is not None and seam_aabb[1][2] <= 0.004,
            details=f"seam aabb={seam_aabb}",
        )
        # Seam is wider than the base
        base_aabb = ctx.part_element_world_aabb(
            piece,
            elem="spout_base" if piece_name == "spout_body" else "valve_pyramid",
        )
        ctx.check(
            f"{piece_name} seam extends beyond base footprint",
            seam_aabb is not None
            and base_aabb is not None
            and (seam_aabb[1][0] - seam_aabb[0][0])
            > (base_aabb[1][0] - base_aabb[0][0]) - 0.001,
            details=f"seam={seam_aabb}, base={base_aabb}",
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
            name=f"{handle.name} hub seats over valve stem",
        )
        ctx.expect_within(
            handle,
            valve,
            axes="xy",
            inner_elem="hub",
            outer_elem="valve_pyramid",
            margin=0.001,
            name=f"{handle.name} hub centered on valve column",
        )

    # --- Joint limits: both handles revolute -π..+π ---
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

    # --- Decisive pose: handles rotate about short vertical axles ---
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
        "left handle rotates about its vertical axle",
        rest_left is not None
        and posed_left is not None
        and math.hypot(
            posed_left[0] - rest_left[0], posed_left[1] - rest_left[1]
        )
        > 0.02,
        details=f"rest={rest_left}, posed={posed_left}",
    )

    rest_right = _ball_center(right_handle)
    with ctx.pose({j_right: -math.pi / 4.0}):
        posed_right = _ball_center(right_handle)
    ctx.check(
        "right handle rotates independently about its vertical axle",
        rest_right is not None
        and posed_right is not None
        and math.hypot(
            posed_right[0] - rest_right[0], posed_right[1] - rest_right[1]
        )
        > 0.02,
        details=f"rest={rest_right}, posed={posed_right}",
    )

    return ctx.report()


object_model = build_object_model()
