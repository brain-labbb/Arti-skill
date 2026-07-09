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
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Widespread two-handle faucet (Art-Deco style), mirror chrome on dark deck.
# Variant 07: swan-neck spout with continuous swivel, stem collars, hot/cold
# cap disks.
#
# Layout (meters, Z up, spout curves forward along +Y):
#   - dark deck plate (root) with three chrome pieces mounted on top (z = 0)
#   - center column at x = 0: tapered square-pyramid base, stepped cap with
#     swivel collar; a high curved swan-neck spout on a continuous vertical
#     joint rises ~0.28 m and sweeps forward ~0.15 m before arcing down
#   - valve columns at x = +/-0.15: smaller tapered pyramids with square cap,
#     visible stem collar, and hot/cold indicator cap disk
#   - each valve carries a slim stem and four-spoke cross handle,
#     0.09 m tip-to-tip with ball ends
#
# Articulation:
#   - each cross handle: REVOLUTE about its stem vertical axis (-pi..pi)
#   - central swan-neck spout: CONTINUOUS about vertical axis (swivel)
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

# Swivel collar ring on the center column (visible pivot base)
SWIVEL_R = 0.025
SWIVEL_H = 0.012
SWIVEL_TOP_Z = CAP_TOP_Z + SWIVEL_H  # 0.110

# Valve columns
V_PYR_BASE = 0.060
V_PYR_TOP = 0.034
V_PYR_H = 0.070
V_CAP_SIZE = 0.040
V_CAP_H = 0.008
V_STEM_R = 0.0065
V_STEM_TOP_Z = 0.096
HANDLE_JOINT_Z = 0.093  # hub captures the stem top by 3 mm

# Stem collar
COLLAR_R = 0.014
COLLAR_H = 0.010
COLLAR_BASE_Z = V_PYR_H + V_CAP_H  # top of cap

# Hot / cold cap disks
CAP_DISK_R = 0.012
CAP_DISK_H = 0.003
CAP_DISK_Z = V_PYR_H + V_CAP_H / 2.0  # centered on cap top face

# Cross handle
HUB_R = 0.0085
HUB_H = 0.034
SPOKE_R = 0.0042
SPOKE_LEN = 0.040
SPOKE_Z = 0.012
BALL_R = 0.0065
BALL_C = 0.0385  # ball centers -> tip-to-tip = 2*(0.0385+0.0065) = 0.090

# Swan-neck spout
SPOUT_TUBE_R = 0.012
SPOUT_OUTLET_R = 0.016  # flared tip


def _pyramid_frustum(base: float, top: float, height: float) -> cq.Workplane:
    """Tapered square-pyramid column, base on z=0, narrowing upward."""
    return (
        cq.Workplane("XY")
        .rect(base, base)
        .workplane(offset=height)
        .rect(top, top)
        .loft(combine=True)
    )


def _swan_neck_points() -> list[tuple[float, float, float]]:
    """Control points for the high swan-neck arc.

    Starts at the swivel collar center (z ~ 0), rises vertically, then sweeps
    forward (+Y) and arcs down to the outlet. Points are in the spout_neck
    local frame (origin at the swivel joint).
    """
    return [
        (0.0, 0.0, 0.0),          # root inside swivel collar
        (0.0, 0.0, 0.040),        # vertical rise
        (0.0, 0.0, 0.100),        # continued rise
        (0.0, 0.010, 0.160),      # start curving forward
        (0.0, 0.040, 0.190),      # apex region
        (0.0, 0.080, 0.185),      # forward sweep
        (0.0, 0.120, 0.160),      # descending arc
        (0.0, 0.150, 0.120),      # outlet position
    ]


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


def _add_valve_column(part: Part, chrome: str, side: str) -> None:
    """Tapered pyramid valve base, square cap, stem collar, cap disk."""
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
    # Stem collar: visible ring between cap and handle
    part.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_H),
        origin=Origin(xyz=(0.0, 0.0, COLLAR_BASE_Z + COLLAR_H / 2.0)),
        material=chrome,
        name="stem_collar",
    )
    stem_z0 = V_PYR_H + V_CAP_H / 2.0  # rooted inside the cap
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
    hot_mat = model.material("hot_indicator", rgba=(0.75, 0.15, 0.12, 1.0))
    cold_mat = model.material("cold_indicator", rgba=(0.12, 0.25, 0.70, 1.0))

    # --- Dark deck plate (root) ---
    deck = model.part("deck")
    deck.visual(
        Box((0.42, 0.20, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),  # top face at z = 0
        material=deck_mat,
        name="deck_plate",
    )

    # --- Center spout column (fixed base) ---
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
    # Swivel collar visible on the spout body top
    spout_body.visual(
        Cylinder(radius=SWIVEL_R, length=SWIVEL_H),
        origin=Origin(xyz=(0.0, 0.0, CAP_TOP_Z + SWIVEL_H / 2.0)),
        material=chrome.name,
        name="swivel_collar",
    )
    model.articulation(
        "deck_to_spout_body",
        ArticulationType.FIXED,
        parent=deck,
        child=spout_body,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # --- Swan-neck spout on continuous swivel ---
    spout_neck = model.part("spout_neck")
    # Small pivot ring at the base of the swan neck
    spout_neck.visual(
        Cylinder(radius=SWIVEL_R - 0.002, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),  # embeds slightly into collar
        material=chrome.name,
        name="swivel_ring",
    )
    # Swan-neck tube
    neck_geom = tube_from_spline_points(
        _swan_neck_points(),
        radius=SPOUT_TUBE_R,
        samples_per_segment=16,
        radial_segments=20,
        cap_ends=False,
    )
    spout_neck.visual(
        mesh_from_geometry(neck_geom, "swan_neck"),
        material=chrome.name,
        name="swan_neck",
    )
    # Flared outlet tip
    spout_neck.visual(
        Cylinder(radius=SPOUT_OUTLET_R, length=0.008),
        origin=Origin(xyz=(0.0, 0.150, 0.116)),
        material=chrome.name,
        name="outlet_flare",
    )

    model.articulation(
        "spout_swivel",
        ArticulationType.CONTINUOUS,
        parent=spout_body,
        child=spout_neck,
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0),
    )

    # --- Valve columns and cross handles (left = -X, right = +X) ---
    for side, sx in (("left", -1.0), ("right", 1.0)):
        valve = model.part(f"{side}_valve")
        _add_valve_column(valve, chrome.name, side)

        # Hot / cold cap disk indicator on top of valve cap
        disk_mat = hot_mat.name if side == "left" else cold_mat.name
        valve.visual(
            Cylinder(radius=CAP_DISK_R, length=CAP_DISK_H),
            origin=Origin(xyz=(0.0, 0.0, V_PYR_H + V_CAP_H + CAP_DISK_H / 2.0)),
            material=disk_mat,
            name=f"{side}_cap_disk",
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


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    spout_body = object_model.get_part("spout_body")
    spout_neck = object_model.get_part("spout_neck")
    left_valve = object_model.get_part("left_valve")
    right_valve = object_model.get_part("right_valve")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")
    j_left = object_model.get_articulation("left_handle_spin")
    j_right = object_model.get_articulation("right_handle_spin")
    j_swivel = object_model.get_articulation("spout_swivel")

    # Intentional captured fits: handle hubs over valve stems,
    # swivel ring seated into the swivel collar.
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
        spout_neck,
        spout_body,
        elem_a="swivel_ring",
        elem_b="swivel_collar",
        reason="Swivel ring is intentionally seated inside the swivel collar.",
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

    # --- Swan-neck spout: rises high, curves forward ---
    neck_aabb = ctx.part_element_world_aabb(spout_neck, elem="swan_neck")
    ctx.check(
        "swan neck rises above 0.20 m from deck",
        neck_aabb is not None and neck_aabb[1][2] > 0.20,
        details=f"swan_neck aabb={neck_aabb}",
    )
    ctx.check(
        "swan neck sweeps forward at least 0.10 m",
        neck_aabb is not None and neck_aabb[1][1] > 0.10,
        details=f"swan_neck aabb={neck_aabb}",
    )
    ctx.check(
        "swan neck outlet arcs down below the apex",
        neck_aabb is not None and neck_aabb[0][2] < neck_aabb[1][2] - 0.05,
        details=f"swan_neck aabb={neck_aabb}",
    )

    # --- Spout swivel is CONTINUOUS (no lower/upper limits) ---
    swivel_type = j_swivel.articulation_type
    ctx.check(
        "spout swivel joint is CONTINUOUS type",
        swivel_type == ArticulationType.CONTINUOUS,
        details=f"type={swivel_type}",
    )
    swivel_lim = j_swivel.motion_limits
    ctx.check(
        "spout swivel has no lower/upper bounds",
        swivel_lim is not None
        and swivel_lim.lower is None
        and swivel_lim.upper is None,
    )

    # --- Swivel pose: spout outlet moves in XY when rotated ---
    outlet_rest = ctx.part_element_world_aabb(spout_neck, elem="outlet_flare")
    with ctx.pose({j_swivel: math.pi / 2.0}):
        outlet_posed = ctx.part_element_world_aabb(spout_neck, elem="outlet_flare")
    ctx.check(
        "spout swivel rotates the outlet about 90 degrees",
        outlet_rest is not None
        and outlet_posed is not None
        and (
            abs(outlet_posed[1][0] - outlet_rest[1][0]) > 0.08
            or abs(outlet_posed[0][0] - outlet_rest[0][0]) > 0.08
        ),
        details=f"rest={outlet_rest}, posed={outlet_posed}",
    )

    # --- Stem collars present on each valve ---
    for valve in (left_valve, right_valve):
        collar_aabb = ctx.part_element_world_aabb(valve, elem="stem_collar")
        ctx.check(
            f"{valve.name} has a visible stem collar",
            collar_aabb is not None
            and (collar_aabb[1][0] - collar_aabb[0][0]) > 0.020,
            details=f"{valve.name} collar aabb={collar_aabb}",
        )

    # --- Hot and cold cap disks present as geometry ---
    left_disk = ctx.part_element_world_aabb(left_valve, elem="left_cap_disk")
    right_disk = ctx.part_element_world_aabb(right_valve, elem="right_cap_disk")
    ctx.check(
        "left valve has a hot cap disk",
        left_disk is not None
        and (left_disk[1][0] - left_disk[0][0]) > 0.018,
        details=f"left_disk aabb={left_disk}",
    )
    ctx.check(
        "right valve has a cold cap disk",
        right_disk is not None
        and (right_disk[1][0] - right_disk[0][0]) > 0.018,
        details=f"right_disk aabb={right_disk}",
    )
    # Disks should be on top of the valve caps
    for valve, disk_name in ((left_valve, "left_cap_disk"), (right_valve, "right_cap_disk")):
        disk_aabb = ctx.part_element_world_aabb(valve, elem=disk_name)
        cap_aabb = ctx.part_element_world_aabb(valve, elem="valve_cap")
        ctx.check(
            f"{disk_name} sits above the valve cap",
            disk_aabb is not None
            and cap_aabb is not None
            and disk_aabb[0][2] >= cap_aabb[1][2] - 0.002,
            details=f"disk={disk_aabb}, cap={cap_aabb}",
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
        ctx.expect_within(
            handle,
            valve,
            axes="xy",
            inner_elem="hub",
            outer_elem="valve_pyramid",
            margin=0.001,
            name=f"{handle.name} hub centered on its valve column",
        )

    # --- Joint limits for handles ---
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

    # --- Decisive pose checks for cross handles ---
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

    return ctx.report()


object_model = build_object_model()
