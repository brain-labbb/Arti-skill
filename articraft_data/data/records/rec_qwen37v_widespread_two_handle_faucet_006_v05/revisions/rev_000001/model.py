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
# Art-Deco widespread two-handle wall-mounted faucet, mirror chrome.
#
# Layout (meters, Z up, spout sweeps forward along +Y):
#   - wall panel (root) at y = -0.10, vertical
#   - dark deck plate fixed to wall, top face at z = 0
#   - center spout body wall-mounted at z = 0.18: stepped Art-Deco
#     escutcheon against wall + flat-topped waterfall spout reaching ~0.17 m
#   - valve columns at x = +/-0.15 on deck: tapered pyramids (0.06 sq,
#     0.07 tall) with cap, stem collar, slim stem, mounting pipe + nut
#   - four-spoke cross handles on each valve, 0.09 m tip-to-tip
# Articulation: each cross handle revolute about its vertical stem axis
# (-pi..pi).
# ---------------------------------------------------------------------------

HANDLE_SPREAD_X = 0.150  # valve column centers at +/-0.150 -> 0.30 m spread

# Wall
WALL_W = 0.42
WALL_D = 0.020
WALL_H = 0.28
WALL_INNER_Y = -0.10  # wall face the faucet mounts to
WALL_CENTER_Y = WALL_INNER_Y - WALL_D / 2.0  # -0.11
WALL_BOTTOM_Z = -0.02
WALL_CENTER_Z = WALL_BOTTOM_Z + WALL_H / 2.0  # 0.12

# Deck
DECK_W = 0.42
DECK_D = 0.20
DECK_H = 0.022

# Spout mount
SPOUT_MOUNT_Z = 0.18  # height of spout center above deck

# Escutcheon (Art-Deco stepped plate against wall)
ESC_OUTER_W = 0.080
ESC_OUTER_D = 0.008
ESC_OUTER_H = 0.100
ESC_INNER_W = 0.060
ESC_INNER_D = 0.006
ESC_INNER_H = 0.075

# Spout
SPOUT_WIDTH = 0.050

# Valve columns
V_PYR_BASE = 0.060
V_PYR_TOP = 0.034
V_PYR_H = 0.070
V_CAP_SIZE = 0.040
V_CAP_H = 0.008
V_STEM_R = 0.0065
V_STEM_TOP_Z = 0.096
HANDLE_JOINT_Z = 0.093  # hub captures the stem top by 3 mm

# Stem collar (visible ring under each handle)
COLLAR_R = 0.011
COLLAR_H = 0.006
COLLAR_Z = V_PYR_H + V_CAP_H + COLLAR_H / 2.0  # 0.081

# Mounting pipe and underside nut
PIPE_R = 0.007
PIPE_H = 0.032
NUT_R = 0.010
NUT_H = 0.007
NUT_Z = -PIPE_H + NUT_H / 2.0  # -0.0285

# Cross handle
HUB_R = 0.0085
HUB_H = 0.034
SPOKE_R = 0.0042
SPOKE_LEN = 0.040
SPOKE_Z = 0.012
BALL_R = 0.0065
BALL_C = 0.0385  # ball centers -> tip-to-tip = 2*(0.0385+0.0065) = 0.090


def _pyramid_frustum(base: float, top: float, height: float) -> cq.Workplane:
    """Tapered square-pyramid column, base on z=0, narrowing upward."""
    return (
        cq.Workplane("XY")
        .rect(base, base)
        .workplane(offset=height)
        .rect(top, top)
        .loft(combine=True)
    )


def _waterfall_spout_wall() -> cq.Workplane:
    """Wide flat-topped spout sweeping forward (+Y) from a wall escutcheon.

    Side profile drawn in the YZ plane, extruded across X for flat
    Art-Deco slab sides. The root (y ~ 0.012) is embedded inside the
    escutcheon so the spout reads as emerging from the body.
    """
    profile = (
        cq.Workplane("YZ")
        .moveTo(0.012, -0.008)
        .lineTo(0.012, 0.010)
        .spline(
            [(0.050, 0.009), (0.100, 0.004), (0.140, -0.010), (0.170, -0.040)],
            includeCurrent=True,
        )
        .lineTo(0.158, -0.046)
        .spline(
            [(0.130, -0.024), (0.095, -0.008), (0.050, -0.005), (0.012, -0.008)],
            includeCurrent=True,
        )
        .close()
        .extrude(SPOUT_WIDTH)
    )
    return profile.translate((-SPOUT_WIDTH / 2.0, 0.0, 0.0))


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


def _add_valve_column(part: Part, chrome: str, dark_metal: str) -> None:
    """Tapered pyramid valve base with cap, stem, collar, pipe, and nut."""
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
    # Visible stem collar ring under the handle
    part.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_H),
        origin=Origin(xyz=(0.0, 0.0, COLLAR_Z)),
        material=chrome,
        name="stem_collar",
    )
    # Mounting pipe through deck
    part.visual(
        Cylinder(radius=PIPE_R, length=PIPE_H),
        origin=Origin(xyz=(0.0, 0.0, -PIPE_H / 2.0)),
        material=chrome,
        name="mounting_pipe",
    )
    # Underside nut below deck
    part.visual(
        Cylinder(radius=NUT_R, length=NUT_H),
        origin=Origin(xyz=(0.0, 0.0, NUT_Z)),
        material=dark_metal,
        name="mounting_nut",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mount_widespread_faucet")

    chrome = model.material("chrome", rgba=(0.88, 0.89, 0.92, 1.0))
    deck_mat = model.material("deck_charcoal", rgba=(0.09, 0.09, 0.10, 1.0))
    wall_mat = model.material("wall_tile", rgba=(0.72, 0.70, 0.68, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.30, 0.30, 0.33, 1.0))

    # --- Wall panel (root) ---
    wall = model.part("wall")
    wall.visual(
        Box((WALL_W, WALL_D, WALL_H)),
        origin=Origin(xyz=(0.0, WALL_CENTER_Y, WALL_CENTER_Z)),
        material=wall_mat,
        name="wall_panel",
    )

    # --- Dark deck plate (fixed child of wall) ---
    deck = model.part("deck")
    deck.visual(
        Box((DECK_W, DECK_D, DECK_H)),
        origin=Origin(xyz=(0.0, 0.0, -DECK_H / 2.0)),
        material=deck_mat,
        name="deck_plate",
    )
    model.articulation(
        "wall_to_deck",
        ArticulationType.FIXED,
        parent=wall,
        child=deck,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # --- Spout body (wall-mounted) ---
    spout_body = model.part("spout_body")
    # Art-Deco stepped escutcheon
    spout_body.visual(
        Box((ESC_OUTER_W, ESC_OUTER_D, ESC_OUTER_H)),
        origin=Origin(xyz=(0.0, ESC_OUTER_D / 2.0, 0.0)),
        material=chrome.name,
        name="escutcheon_outer",
    )
    spout_body.visual(
        Box((ESC_INNER_W, ESC_INNER_D, ESC_INNER_H)),
        origin=Origin(xyz=(0.0, ESC_OUTER_D + ESC_INNER_D / 2.0, 0.0)),
        material=chrome.name,
        name="escutcheon_inner",
    )
    # Waterfall spout arm
    spout_body.visual(
        mesh_from_cadquery(_waterfall_spout_wall(), "waterfall_spout_wall"),
        material=chrome.name,
        name="spout",
    )
    model.articulation(
        "wall_to_spout",
        ArticulationType.FIXED,
        parent=wall,
        child=spout_body,
        origin=Origin(xyz=(0.0, WALL_INNER_Y, SPOUT_MOUNT_Z)),
    )

    # --- Valve columns and cross handles (left = -X, right = +X) ---
    for side, sx in (("left", -1.0), ("right", 1.0)):
        valve = model.part(f"{side}_valve")
        _add_valve_column(valve, chrome.name, dark_metal.name)
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

    wall = object_model.get_part("wall")
    deck = object_model.get_part("deck")
    spout_body = object_model.get_part("spout_body")
    left_valve = object_model.get_part("left_valve")
    right_valve = object_model.get_part("right_valve")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")
    j_left = object_model.get_articulation("left_handle_spin")
    j_right = object_model.get_articulation("right_handle_spin")

    # Intentional captured fits: handle hubs over valve stems, mounting
    # pipes through deck plate.
    for handle, valve in (
        (left_handle, left_valve),
        (right_handle, right_valve),
    ):
        ctx.allow_overlap(
            handle,
            valve,
            elem_a="hub",
            elem_b="valve_stem",
            reason="Cross-handle hub intentionally captures the valve bonnet stem.",
        )
        ctx.allow_overlap(
            valve,
            deck,
            elem_a="mounting_pipe",
            elem_b="deck_plate",
            reason="Mounting pipe passes through the deck plate to secure the valve.",
        )

    # --- Wall-mount: spout body is well above the deck ---
    ctx.expect_gap(
        spout_body,
        deck,
        axis="z",
        min_gap=0.04,
        name="spout body is wall-mounted above the deck",
    )

    # --- Spout body on wall face ---
    ctx.expect_gap(
        spout_body,
        wall,
        axis="y",
        max_gap=0.002,
        max_penetration=0.005,
        name="spout escutcheon seats against the wall face",
    )

    # --- Deck contacts wall ---
    ctx.expect_gap(
        deck,
        wall,
        axis="y",
        max_gap=0.002,
        max_penetration=0.002,
        name="deck rear edge contacts the wall",
    )

    # --- Valve columns seated on deck ---
    for valve in (left_valve, right_valve):
        ctx.expect_gap(
            valve,
            deck,
            axis="z",
            positive_elem="valve_pyramid",
            negative_elem="deck_plate",
            max_gap=0.001,
            max_penetration=0.0005,
            name=f"{valve.name} base seated on deck top",
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

    # --- Stem collars visible above the valve caps ---
    for valve in (left_valve, right_valve):
        collar_aabb = ctx.part_element_world_aabb(valve, elem="stem_collar")
        cap_aabb = ctx.part_element_world_aabb(valve, elem="valve_cap")
        ctx.check(
            f"{valve.name} stem collar sits above the valve cap",
            collar_aabb is not None
            and cap_aabb is not None
            and collar_aabb[0][2] > cap_aabb[1][2] - 0.002,
            details=f"collar={collar_aabb}, cap={cap_aabb}",
        )

    # --- Stem collar wider than the stem ---
    for valve in (left_valve, right_valve):
        collar_aabb = ctx.part_element_world_aabb(valve, elem="stem_collar")
        stem_aabb = ctx.part_element_world_aabb(valve, elem="valve_stem")
        ctx.check(
            f"{valve.name} stem collar is wider than the stem",
            collar_aabb is not None
            and stem_aabb is not None
            and (collar_aabb[1][0] - collar_aabb[0][0])
            > (stem_aabb[1][0] - stem_aabb[0][0]) + 0.004,
            details=f"collar={collar_aabb}, stem={stem_aabb}",
        )

    # --- Underside nuts below the deck surface ---
    for valve in (left_valve, right_valve):
        nut_aabb = ctx.part_element_world_aabb(valve, elem="mounting_nut")
        ctx.check(
            f"{valve.name} mounting nut is below the deck surface",
            nut_aabb is not None and nut_aabb[1][2] < -0.018,
            details=f"nut aabb={nut_aabb}",
        )

    # --- Waterfall spout: forward reach from wall, tip arcs down ---
    spout_aabb = ctx.part_element_world_aabb(spout_body, elem="spout")
    ctx.check(
        "spout reaches forward past the wall face",
        spout_aabb is not None and spout_aabb[1][1] > WALL_INNER_Y + 0.14,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "spout tip arcs down below the mount height",
        spout_aabb is not None and spout_aabb[0][2] < SPOUT_MOUNT_Z - 0.02,
        details=f"spout aabb={spout_aabb}",
    )

    # --- Escutcheon present on wall ---
    esc_aabb = ctx.part_element_world_aabb(spout_body, elem="escutcheon_outer")
    ctx.check(
        "escutcheon outer step is about 0.08 m wide",
        esc_aabb is not None
        and 0.076 <= (esc_aabb[1][0] - esc_aabb[0][0]) <= 0.084,
        details=f"escutcheon aabb={esc_aabb}",
    )

    # --- Cross handles: 0.09 m tip-to-tip, seated over the stems ---
    for handle, valve in (
        (left_handle, left_valve),
        (right_handle, right_valve),
    ):
        h_aabb = ctx.part_world_aabb(handle)
        ctx.check(
            f"{handle.name} cross is about 0.09 m tip-to-tip",
            h_aabb is not None
            and 0.086 <= (h_aabb[1][0] - h_aabb[0][0]) <= 0.094,
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

    # --- Joint limits match the prompt ---
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
        "right handle spins independently about its stem axis",
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
