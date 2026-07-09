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
# Widespread two-handle bathroom faucet, mirror chrome on dark deck.
#
# Layout (meters, Z up, spout sweeps forward along +Y):
#   - dark deck plate (root) with three chrome pieces mounted on top (z = 0)
#   - center spout column at x = 0: taller tapered square-pyramid base
#     (0.07 sq at deck -> narrower at z = 0.11), stepped cap, flat-topped
#     waterfall spout reaching ~0.18 forward
#   - valve columns at x = +/-0.185: smaller tapered pyramids (0.06 sq,
#     0.07 tall) with square cap, visible stem collar, and slim stem
#   - lever handles on each valve: cylindrical hub + lever arm extending
#     outward (+Y), tilting forward/back about X axis
#   - underside hex nuts below each base, visible beneath the deck
#
# Articulation: each lever handle is a revolute joint about its horizontal
# X axis (forward/back tilt), range -0.5..+0.5 rad.
# ---------------------------------------------------------------------------

HANDLE_SPREAD_X = 0.185  # valve centers at +/-0.185 -> 0.37 m spread

# Center column (taller than original roman tub variant)
C_PYR_BASE = 0.070
C_PYR_TOP = 0.042
C_PYR_H = 0.110
CAP1_SIZE = 0.056
CAP1_H = 0.010
CAP2_SIZE = 0.044
CAP2_H = 0.008
CAP_TOP_Z = C_PYR_H + CAP1_H + CAP2_H  # 0.128

# Valve columns
V_PYR_BASE = 0.060
V_PYR_TOP = 0.034
V_PYR_H = 0.070
V_CAP_SIZE = 0.040
V_CAP_H = 0.008
# Stem collar: visible ring between cap and handle
COLLAR_R = 0.012
COLLAR_H = 0.006
COLLAR_BASE_Z = V_PYR_H + V_CAP_H  # sits on top of cap
# Stem through the collar
V_STEM_R = 0.006
V_STEM_TOP_Z = COLLAR_BASE_Z + COLLAR_H + 0.008  # protrudes above collar
HANDLE_JOINT_Z = COLLAR_BASE_Z + COLLAR_H  # handle hub sits on collar top

# Lever handle
HUB_R = 0.010
HUB_H = 0.018
LEVER_WIDTH = 0.012  # cross-section width
LEVER_HEIGHT = 0.008  # cross-section height (thin slab)
LEVER_LENGTH = 0.050  # extends outward from hub center
LEVER_TIP_R = 0.006  # rounded end cap

# Underside mounting nuts
NUT_HEX_R = 0.010  # hex inscribed radius (flat-to-flat / 2)
NUT_H = 0.008
NUT_Z = -0.022 - NUT_H / 2.0  # below deck top surface

# Spout
SPOUT_WIDTH = 0.050


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
    The spout is positioned higher to match the taller center column.
    """
    base_z = C_PYR_H - 0.020  # emerge from upper portion of taller column
    profile = (
        cq.Workplane("YZ")
        .moveTo(0.010, base_z + 0.014)
        .lineTo(0.010, base_z + 0.030)
        .spline(
            [(0.060, base_z + 0.029), (0.110, base_z + 0.022), (0.150, base_z + 0.006), (0.174, base_z - 0.018)],
            includeCurrent=True,
        )
        .lineTo(0.160, base_z - 0.024)
        .spline(
            [(0.140, base_z - 0.008), (0.105, base_z + 0.006), (0.060, base_z + 0.013), (0.010, base_z + 0.014)],
            includeCurrent=True,
        )
        .close()
        .extrude(SPOUT_WIDTH)
    )
    return profile.translate((-SPOUT_WIDTH / 2.0, 0.0, 0.0))


def _hex_nut(radius: float, height: float) -> cq.Workplane:
    """Small hexagonal nut, centered at origin with flat faces."""
    return (
        cq.Workplane("XY")
        .polygon(6, radius * 2.0)
        .extrude(height)
        .translate((0.0, 0.0, -height / 2.0))
    )


def _add_lever_handle(part: Part, chrome: str) -> None:
    """Lever handle: cylindrical hub + extending arm with rounded tip.

    Local frame origin is the handle joint frame: hub bottom at z=0.
    The lever extends along +Y; rotation about X tilts it forward/back.
    """
    # Hub cylinder
    part.visual(
        Cylinder(radius=HUB_R, length=HUB_H),
        origin=Origin(xyz=(0.0, 0.0, HUB_H / 2.0)),
        material=chrome,
        name="hub",
    )
    # Dome cap on hub top
    part.visual(
        Sphere(radius=HUB_R),
        origin=Origin(xyz=(0.0, 0.0, HUB_H)),
        material=chrome,
        name="hub_dome",
    )
    # Lever arm: box extending along +Y from hub center height
    lever_z = HUB_H / 2.0  # mid-height of hub
    part.visual(
        Box((LEVER_WIDTH, LEVER_LENGTH, LEVER_HEIGHT)),
        origin=Origin(xyz=(0.0, LEVER_LENGTH / 2.0, lever_z)),
        material=chrome,
        name="lever_arm",
    )
    # Rounded tip at end of lever
    part.visual(
        Sphere(radius=LEVER_TIP_R),
        origin=Origin(xyz=(0.0, LEVER_LENGTH, lever_z)),
        material=chrome,
        name="lever_tip",
    )


def _add_valve_column(part: Part, chrome: str) -> None:
    """Tapered pyramid valve base with square cap, visible stem collar, and stem."""
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
    # Visible stem collar: cylindrical ring on top of the cap
    part.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_H),
        origin=Origin(xyz=(0.0, 0.0, COLLAR_BASE_Z + COLLAR_H / 2.0)),
        material=chrome,
        name="stem_collar",
    )
    # Slim stem protruding above collar
    stem_z0 = COLLAR_BASE_Z + COLLAR_H
    part.visual(
        Cylinder(radius=V_STEM_R, length=V_STEM_TOP_Z - stem_z0),
        origin=Origin(xyz=(0.0, 0.0, (stem_z0 + V_STEM_TOP_Z) / 2.0)),
        material=chrome,
        name="valve_stem",
    )
    # Mounting bolt connecting base through deck to underside nut
    bolt_length = abs(NUT_Z) + NUT_H / 2.0  # from z=0 down to nut center
    part.visual(
        Cylinder(radius=0.004, length=bolt_length),
        origin=Origin(xyz=(0.0, 0.0, -bolt_length / 2.0)),
        material=chrome,
        name="mounting_bolt",
    )
    # Underside hex nut (below deck)
    part.visual(
        mesh_from_cadquery(_hex_nut(NUT_HEX_R, NUT_H), f"{part.name}_nut"),
        origin=Origin(xyz=(0.0, 0.0, NUT_Z)),
        material=chrome,
        name="underside_nut",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_two_handle_faucet")

    chrome = model.material("chrome", rgba=(0.88, 0.89, 0.92, 1.0))
    deck_mat = model.material("deck_charcoal", rgba=(0.09, 0.09, 0.10, 1.0))

    # --- Dark deck plate (root) ---
    deck = model.part("deck")
    deck.visual(
        Box((0.48, 0.20, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),  # top face at z = 0
        material=deck_mat,
        name="deck_plate",
    )

    # --- Center spout column (taller than original) ---
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
    # Center mounting bolt and underside nut
    bolt_length = abs(NUT_Z) + NUT_H / 2.0
    spout_body.visual(
        Cylinder(radius=0.004, length=bolt_length),
        origin=Origin(xyz=(0.0, 0.0, -bolt_length / 2.0)),
        material=chrome.name,
        name="mounting_bolt",
    )
    spout_body.visual(
        mesh_from_cadquery(_hex_nut(NUT_HEX_R, NUT_H), "center_nut"),
        origin=Origin(xyz=(0.0, 0.0, NUT_Z)),
        material=chrome.name,
        name="underside_nut",
    )
    model.articulation(
        "deck_to_spout_body",
        ArticulationType.FIXED,
        parent=deck,
        child=spout_body,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # --- Valve columns and lever handles (left = -X, right = +X) ---
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
        _add_lever_handle(handle, chrome.name)
        model.articulation(
            f"{side}_handle_tilt",
            ArticulationType.REVOLUTE,
            parent=valve,
            child=handle,
            origin=Origin(xyz=(0.0, 0.0, HANDLE_JOINT_Z)),
            axis=(1.0, 0.0, 0.0),  # horizontal X axis: forward/back tilt
            motion_limits=MotionLimits(
                effort=5.0, velocity=3.0, lower=-0.50, upper=0.50
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
    j_left = object_model.get_articulation("left_handle_tilt")
    j_right = object_model.get_articulation("right_handle_tilt")

    # Intentional captured fits: handle hubs capture valve stems
    ctx.allow_overlap(
        left_handle,
        left_valve,
        elem_a="hub",
        elem_b="valve_stem",
        reason="Lever-handle hub intentionally captures the valve stem top.",
    )
    ctx.allow_overlap(
        right_handle,
        right_valve,
        elem_a="hub",
        elem_b="valve_stem",
        reason="Lever-handle hub intentionally captures the valve stem top.",
    )
    ctx.allow_overlap(
        left_handle,
        left_valve,
        elem_a="hub",
        elem_b="stem_collar",
        reason="Hub seats onto the stem collar with small local embed.",
    )
    ctx.allow_overlap(
        right_handle,
        right_valve,
        elem_a="hub",
        elem_b="stem_collar",
        reason="Hub seats onto the stem collar with small local embed.",
    )

    # Mounting bolts pass through deck holes (real faucet mounting hardware)
    for piece in (spout_body, left_valve, right_valve):
        ctx.allow_overlap(
            piece,
            deck,
            elem_a="mounting_bolt",
            elem_b="deck_plate",
            reason="Mounting bolt passes through a deck hole as real faucet mounting hardware.",
        )
        base_elem = "spout_pyramid" if piece.name == "spout_body" else "valve_pyramid"
        ctx.expect_contact(
            piece,
            deck,
            elem_a=base_elem,
            elem_b="deck_plate",
            name=f"{piece.name} pyramid base contacts the deck surface",
        )

    # --- All three chrome pieces seated on the dark deck ---
    # Check the pyramid base elements specifically (parts include underside nuts)
    for piece, base_elem in (
        (spout_body, "spout_pyramid"),
        (left_valve, "valve_pyramid"),
        (right_valve, "valve_pyramid"),
    ):
        ctx.expect_gap(
            piece,
            deck,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0005,
            positive_elem=base_elem,
            name=f"{piece.name} base seated on deck top",
        )
        ctx.expect_within(
            piece,
            deck,
            axes="x",
            margin=0.001,
            inner_elem=base_elem,
            name=f"{piece.name} stands within the deck plate footprint",
        )

    # --- Wider spread than the original 0.30m roman tub ---
    ctx.expect_origin_distance(
        left_handle,
        right_handle,
        axes="x",
        min_dist=0.35,
        max_dist=0.39,
        name="handle spread is wider (~0.37 m)",
    )
    ctx.expect_origin_gap(
        right_valve,
        spout_body,
        axis="x",
        min_gap=0.17,
        max_gap=0.20,
        name="right valve is spread farther from spout",
    )
    ctx.expect_origin_gap(
        spout_body,
        left_valve,
        axis="x",
        min_gap=0.17,
        max_gap=0.20,
        name="left valve is spread farther from spout",
    )

    # --- Taller central spout column ---
    pyr_aabb = ctx.part_element_world_aabb(spout_body, elem="spout_pyramid")
    ctx.check(
        "center pyramid is taller than original (>= 0.10 m)",
        pyr_aabb is not None and (pyr_aabb[1][2] - pyr_aabb[0][2]) >= 0.10,
        details=f"pyramid aabb={pyr_aabb}",
    )
    ctx.check(
        "center pyramid base is about 0.07 m square at the deck",
        pyr_aabb is not None
        and 0.066 <= (pyr_aabb[1][0] - pyr_aabb[0][0]) <= 0.074
        and 0.066 <= (pyr_aabb[1][1] - pyr_aabb[0][1]) <= 0.074,
        details=f"pyramid aabb={pyr_aabb}",
    )

    # --- Waterfall spout reach ---
    spout_aabb = ctx.part_element_world_aabb(spout_body, elem="spout")
    ctx.check(
        "spout reaches about 0.18 m forward",
        spout_aabb is not None and 0.16 <= spout_aabb[1][1] <= 0.20,
        details=f"spout aabb={spout_aabb}",
    )

    # --- Stem collars visible on each valve column ---
    for valve in (left_valve, right_valve):
        collar_aabb = ctx.part_element_world_aabb(valve, elem="stem_collar")
        ctx.check(
            f"{valve.name} has a visible stem collar",
            collar_aabb is not None
            and (collar_aabb[1][2] - collar_aabb[0][2]) > 0.003,
            details=f"collar aabb={collar_aabb}",
        )

    # --- Underside nuts below the deck ---
    for piece in (spout_body, left_valve, right_valve):
        nut_aabb = ctx.part_element_world_aabb(piece, elem="underside_nut")
        ctx.check(
            f"{piece.name} has an underside nut below deck level",
            nut_aabb is not None and nut_aabb[1][2] < -0.005,
            details=f"nut aabb={nut_aabb}",
        )

    # --- Lever handles: arm extends outward, seats on valve ---
    for handle, valve in ((left_handle, left_valve), (right_handle, right_valve)):
        h_aabb = ctx.part_world_aabb(handle)
        ctx.check(
            f"{handle.name} lever extends outward (Y span > 0.04 m)",
            h_aabb is not None and (h_aabb[1][1] - h_aabb[0][1]) > 0.04,
            details=f"{handle.name} aabb={h_aabb}",
        )
        ctx.expect_gap(
            handle,
            valve,
            axis="z",
            elem_a="hub",
            elem_b="stem_collar",
            max_gap=0.002,
            max_penetration=0.003,
            name=f"{handle.name} hub seats on valve collar",
        )

    # --- Joint limits: lever tilt about horizontal X, -0.5..+0.5 rad ---
    for joint in (j_left, j_right):
        lim = joint.motion_limits
        ctx.check(
            f"{joint.name} is a revolute tilt with limits near ±0.5 rad",
            lim is not None
            and lim.lower is not None
            and lim.upper is not None
            and abs(lim.lower + 0.50) < 0.02
            and abs(lim.upper - 0.50) < 0.02,
        )
        # Verify axis is horizontal (X)
        ctx.check(
            f"{joint.name} axis is horizontal (X direction)",
            hasattr(joint, "axis")
            and joint.axis is not None
            and abs(joint.axis[0]) > 0.9
            and abs(joint.axis[1]) < 0.1
            and abs(joint.axis[2]) < 0.1,
        )

    # --- Decisive pose checks: lever tilts forward/back ---
    for handle, joint in ((left_handle, j_left), (right_handle, j_right)):
        rest_tip = ctx.part_element_world_aabb(handle, elem="lever_tip")
        rest_y = (rest_tip[0][1] + rest_tip[1][1]) / 2.0 if rest_tip else None

        with ctx.pose({joint: 0.40}):
            posed_tip = ctx.part_element_world_aabb(handle, elem="lever_tip")
        posed_y = (posed_tip[0][1] + posed_tip[1][1]) / 2.0 if posed_tip else None
        posed_z = (posed_tip[0][2] + posed_tip[1][2]) / 2.0 if posed_tip else None
        rest_z_center = None
        if rest_tip:
            rest_z_center = (rest_tip[0][2] + rest_tip[1][2]) / 2.0

        ctx.check(
            f"{joint.name} positive pose tilts lever tip upward",
            rest_z_center is not None
            and posed_z is not None
            and posed_z > rest_z_center + 0.005,
            details=f"rest_z={rest_z_center}, posed_z={posed_z}",
        )

    return ctx.report()


object_model = build_object_model()
