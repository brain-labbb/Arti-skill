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
# Widespread two-handle faucet, polished chrome on dark deck.
# Three-piece layout: left cross handle, central spout, right cross handle.
# Total spread about 0.30 m.
#
# Variant 23 changes from parent Art-Deco faucet:
# - Round tapered cylinder bases (replacing square pyramids) for valves
# - Diverter knob behind the spout on a PRISMATIC up-down joint
# - Visible stem collars under each cross handle
# - Small hexagonal underside mounting nuts below all three bases
# ---------------------------------------------------------------------------

HANDLE_SPREAD_X = 0.150  # valve centers at +/-0.150 -> 0.30 m spread

# Round valve bases
V_BASE_R_BOT = 0.030   # radius at deck level
V_BASE_R_TOP = 0.022   # radius at top (tapered)
V_BASE_H = 0.070       # base height
V_CAP_R = 0.024        # cap radius
V_CAP_H = 0.008        # cap height

# Stem collar (visible ring between cap and handle hub)
COLLAR_R = 0.013
COLLAR_H = 0.008

# Valve stem
V_STEM_R = 0.0065
V_STEM_TOP_Z = 0.098

# Cross handle
HUB_R = 0.0085
HUB_H = 0.034
SPOKE_R = 0.0042
SPOKE_LEN = 0.040
SPOKE_Z = 0.012
BALL_R = 0.0065
BALL_C = 0.0385
HANDLE_JOINT_Z = 0.094  # hub captures stem top by ~4 mm

# Center spout body (round tapered base)
C_BASE_R_BOT = 0.036
C_BASE_R_TOP = 0.026
C_BASE_H = 0.080
C_CAP_R = 0.028
C_CAP_H = 0.010

# Spout
SPOUT_WIDTH = 0.050

# Diverter (behind spout, prismatic up-down)
DIV_Y = -0.033
DIV_GUIDE_R = 0.006
DIV_GUIDE_H = 0.045
DIV_GUIDE_Z_BOT = 0.050
DIV_JOINT_Z = DIV_GUIDE_Z_BOT + DIV_GUIDE_H  # guide mouth: 0.095
DIV_STEM_R = 0.004
DIV_STEM_H = 0.028
DIV_GRIP_R = 0.009
DIV_GRIP_H = 0.012

# Underside mounting nut (hexagonal)
NUT_AF = 0.020          # across-flats
NUT_H = 0.006
NUT_Z_TOP = -0.022      # flush with deck bottom


def _cone_frustum(r_bot: float, r_top: float, height: float) -> cq.Workplane:
    """Tapered round cone frustum, base on z=0, narrowing upward."""
    return (
        cq.Workplane("XY")
        .circle(r_bot)
        .workplane(offset=height)
        .circle(r_top)
        .loft(combine=True)
    )


def _hex_prism(af: float, h: float) -> cq.Workplane:
    """Hexagonal prism centered at origin, across-flats = af, extruded +Z."""
    d = af / math.cos(math.pi / 6.0)
    return cq.Workplane("XY").polygon(6, d).extrude(h)


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


def _add_valve_column(part: Part, chrome: str) -> None:
    """Round tapered base, cap, stem collar, stem, and underside hex nut."""
    # Round tapered base
    part.visual(
        mesh_from_cadquery(
            _cone_frustum(V_BASE_R_BOT, V_BASE_R_TOP, V_BASE_H),
            f"{part.name}_base",
        ),
        material=chrome,
        name="valve_base",
    )
    # Cap disk
    part.visual(
        Cylinder(radius=V_CAP_R, length=V_CAP_H),
        origin=Origin(xyz=(0.0, 0.0, V_BASE_H + V_CAP_H / 2.0)),
        material=chrome,
        name="valve_cap",
    )
    # Stem collar (visible decorative ring above cap, below handle)
    collar_z = V_BASE_H + V_CAP_H + COLLAR_H / 2.0
    part.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_H),
        origin=Origin(xyz=(0.0, 0.0, collar_z)),
        material=chrome,
        name="stem_collar",
    )
    # Bonnet stem
    stem_z0 = V_BASE_H + V_CAP_H / 2.0
    part.visual(
        Cylinder(radius=V_STEM_R, length=V_STEM_TOP_Z - stem_z0),
        origin=Origin(xyz=(0.0, 0.0, (stem_z0 + V_STEM_TOP_Z) / 2.0)),
        material=chrome,
        name="valve_stem",
    )
    # Mounting stud (threaded rod through deck connecting base to nut)
    stud_z_bot = NUT_Z_TOP       # -0.022 (deck bottom)
    stud_z_top = 0.010           # inside the base
    part.visual(
        Cylinder(radius=0.004, length=stud_z_top - stud_z_bot),
        origin=Origin(xyz=(0.0, 0.0, (stud_z_bot + stud_z_top) / 2.0)),
        material=chrome,
        name="mounting_stud",
    )
    # Underside hex mounting nut (below deck)
    part.visual(
        mesh_from_cadquery(
            _hex_prism(NUT_AF, NUT_H),
            f"{part.name}_nut",
        ),
        origin=Origin(xyz=(0.0, 0.0, NUT_Z_TOP - NUT_H)),
        material=chrome,
        name="mounting_nut",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_two_handle_faucet")

    chrome = model.material("chrome", rgba=(0.88, 0.89, 0.92, 1.0))
    deck_mat = model.material("deck_charcoal", rgba=(0.09, 0.09, 0.10, 1.0))

    # --- Dark deck plate (root) ---
    deck = model.part("deck")
    deck.visual(
        Box((0.42, 0.20, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=deck_mat,
        name="deck_plate",
    )

    # --- Center spout body (round tapered base + waterfall spout) ---
    spout_body = model.part("spout_body")
    spout_body.visual(
        mesh_from_cadquery(
            _cone_frustum(C_BASE_R_BOT, C_BASE_R_TOP, C_BASE_H),
            "center_base",
        ),
        material=chrome.name,
        name="spout_base",
    )
    spout_body.visual(
        Cylinder(radius=C_CAP_R, length=C_CAP_H),
        origin=Origin(xyz=(0.0, 0.0, C_BASE_H + C_CAP_H / 2.0)),
        material=chrome.name,
        name="spout_cap",
    )
    spout_body.visual(
        mesh_from_cadquery(_waterfall_spout(), "waterfall_spout"),
        material=chrome.name,
        name="spout",
    )
    # Diverter guide sleeve (small tube on the back of the base)
    guide_zc = DIV_GUIDE_Z_BOT + DIV_GUIDE_H / 2.0
    spout_body.visual(
        Cylinder(radius=DIV_GUIDE_R, length=DIV_GUIDE_H),
        origin=Origin(xyz=(0.0, DIV_Y, guide_zc)),
        material=chrome.name,
        name="diverter_guide",
    )
    # Center mounting stud (through deck, connecting base to nut)
    stud_z_bot = NUT_Z_TOP
    stud_z_top = 0.010
    spout_body.visual(
        Cylinder(radius=0.004, length=stud_z_top - stud_z_bot),
        origin=Origin(xyz=(0.0, 0.0, (stud_z_bot + stud_z_top) / 2.0)),
        material=chrome.name,
        name="mounting_stud",
    )
    # Center underside hex mounting nut
    spout_body.visual(
        mesh_from_cadquery(_hex_prism(NUT_AF, NUT_H), "center_nut"),
        origin=Origin(xyz=(0.0, 0.0, NUT_Z_TOP - NUT_H)),
        material=chrome.name,
        name="mounting_nut",
    )
    model.articulation(
        "deck_to_spout_body",
        ArticulationType.FIXED,
        parent=deck,
        child=spout_body,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # --- Diverter knob (PRISMATIC, behind spout, slides up-down) ---
    diverter = model.part("diverter_knob")
    # Stem slides inside the guide sleeve
    diverter.visual(
        Cylinder(radius=DIV_STEM_R, length=DIV_STEM_H),
        origin=Origin(xyz=(0.0, 0.0, -DIV_STEM_H / 2.0)),
        material=chrome.name,
        name="diverter_stem",
    )
    # Grip knob on top of stem
    diverter.visual(
        Cylinder(radius=DIV_GRIP_R, length=DIV_GRIP_H),
        origin=Origin(xyz=(0.0, 0.0, DIV_GRIP_H / 2.0)),
        material=chrome.name,
        name="diverter_grip",
    )
    # Dome cap on grip top
    diverter.visual(
        Sphere(radius=DIV_GRIP_R * 0.7),
        origin=Origin(xyz=(0.0, 0.0, DIV_GRIP_H)),
        material=chrome.name,
        name="diverter_dome",
    )
    model.articulation(
        "diverter_slide",
        ArticulationType.PRISMATIC,
        parent=spout_body,
        child=diverter,
        origin=Origin(xyz=(0.0, DIV_Y, DIV_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=1.5, lower=0.0, upper=0.025
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
    diverter = object_model.get_part("diverter_knob")
    left_valve = object_model.get_part("left_valve")
    right_valve = object_model.get_part("right_valve")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")
    j_left = object_model.get_articulation("left_handle_spin")
    j_right = object_model.get_articulation("right_handle_spin")
    j_div = object_model.get_articulation("diverter_slide")

    _base_elems = {
        "spout_body": "spout_base",
        "left_valve": "valve_base",
        "right_valve": "valve_base",
    }

    # ---- Intentional overlap allowances ----
    # Handle hubs capture valve stems
    ctx.allow_overlap(
        left_handle, left_valve,
        elem_a="hub", elem_b="valve_stem",
        reason="Cross-handle hub intentionally captures the valve bonnet stem.",
    )
    ctx.allow_overlap(
        right_handle, right_valve,
        elem_a="hub", elem_b="valve_stem",
        reason="Cross-handle hub intentionally captures the valve bonnet stem.",
    )
    # Diverter stem slides inside the guide sleeve
    ctx.allow_overlap(
        diverter, spout_body,
        elem_a="diverter_stem", elem_b="diverter_guide",
        reason="Diverter stem intentionally slides inside the guide sleeve.",
    )
    # Mounting studs pass through deck (faucet mounting hardware)
    for piece in (spout_body, left_valve, right_valve):
        ctx.allow_overlap(
            piece, deck,
            elem_a="mounting_stud", elem_b="deck_plate",
            reason="Mounting stud passes through the deck as deck-mount faucet hardware.",
        )
        ctx.expect_contact(
            piece, deck,
            elem_a=_base_elems[piece.name], elem_b="deck_plate",
            name=f"{piece.name} base contacts the deck surface",
        )

    # ---- All three pieces seated on deck (scoped to base elements) ----
    for piece in (spout_body, left_valve, right_valve):
        elem = _base_elems[piece.name]
        ctx.expect_gap(
            piece, deck, axis="z",
            positive_elem=elem,
            max_gap=0.001, max_penetration=0.0005,
            name=f"{piece.name} base seated on deck top",
        )
        ctx.expect_within(
            piece, deck, axes="x", margin=0.001,
            name=f"{piece.name} stands within the deck plate",
        )

    # ---- Three-piece spread ~0.30 m ----
    ctx.expect_origin_distance(
        left_handle, right_handle,
        axes="x", min_dist=0.29, max_dist=0.31,
        name="handle spread is about 0.30 m",
    )

    # ---- Round bases (width ≈ depth for circular cross-section) ----
    for valve in (left_valve, right_valve):
        base_aabb = ctx.part_element_world_aabb(valve, elem="valve_base")
        ctx.check(
            f"{valve.name} has a round base (width ≈ depth)",
            base_aabb is not None
            and abs(
                (base_aabb[1][0] - base_aabb[0][0])
                - (base_aabb[1][1] - base_aabb[0][1])
            ) < 0.004,
            details=f"base aabb={base_aabb}",
        )

    # ---- Stem collars visible on each valve column ----
    for valve in (left_valve, right_valve):
        collar_aabb = ctx.part_element_world_aabb(valve, elem="stem_collar")
        cap_aabb = ctx.part_element_world_aabb(valve, elem="valve_cap")
        ctx.check(
            f"{valve.name} has a stem collar above the cap",
            collar_aabb is not None and cap_aabb is not None
            and collar_aabb[0][2] >= cap_aabb[0][2] - 0.001,
            details=f"collar={collar_aabb}, cap={cap_aabb}",
        )

    # ---- Underside hex nuts below all three bases ----
    for valve in (left_valve, right_valve):
        nut_aabb = ctx.part_element_world_aabb(valve, elem="mounting_nut")
        ctx.check(
            f"{valve.name} mounting nut below deck level",
            nut_aabb is not None and nut_aabb[1][2] < -0.020,
            details=f"nut aabb={nut_aabb}",
        )
    center_nut_aabb = ctx.part_element_world_aabb(spout_body, elem="mounting_nut")
    ctx.check(
        "spout body mounting nut below deck level",
        center_nut_aabb is not None and center_nut_aabb[1][2] < -0.020,
        details=f"center nut aabb={center_nut_aabb}",
    )

    # ---- Cross handles: ~0.09 m tip-to-tip, seated over stems ----
    for handle, valve in ((left_handle, left_valve), (right_handle, right_valve)):
        h_aabb = ctx.part_world_aabb(handle)
        ctx.check(
            f"{handle.name} cross is about 0.09 m tip-to-tip",
            h_aabb is not None and 0.086 <= (h_aabb[1][0] - h_aabb[0][0]) <= 0.094,
            details=f"{handle.name} aabb={h_aabb}",
        )
        ctx.expect_gap(
            handle, valve, axis="z",
            max_gap=0.002, max_penetration=0.005,
            name=f"{handle.name} hub seats over the valve stem",
        )

    # ---- Spout forward reach ~0.18 m ----
    spout_aabb = ctx.part_element_world_aabb(spout_body, elem="spout")
    ctx.check(
        "spout reaches about 0.18 m forward",
        spout_aabb is not None and 0.16 <= spout_aabb[1][1] <= 0.20,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "spout tip arcs down but stays above deck",
        spout_aabb is not None and 0.01 <= spout_aabb[0][2] <= 0.045,
        details=f"spout aabb={spout_aabb}",
    )

    # ---- Diverter prismatic joint ----
    div_lim = j_div.motion_limits
    ctx.check(
        "diverter slide is prismatic with 0..0.025 m range",
        j_div.articulation_type == ArticulationType.PRISMATIC
        and div_lim is not None
        and div_lim.lower is not None
        and div_lim.upper is not None
        and abs(div_lim.lower) < 0.001
        and abs(div_lim.upper - 0.025) < 0.001,
    )

    # Diverter knob behind spout (negative Y from spout center)
    div_grip_rest = ctx.part_element_world_aabb(diverter, elem="diverter_grip")
    spout_aabb_full = ctx.part_world_aabb(spout_body)
    ctx.check(
        "diverter grip is behind spout center (negative Y)",
        div_grip_rest is not None and spout_aabb_full is not None
        and (div_grip_rest[0][1] + div_grip_rest[1][1]) / 2.0
        < (spout_aabb_full[0][1] + spout_aabb_full[1][1]) / 2.0 - 0.01,
        details=f"grip={div_grip_rest}, spout={spout_aabb_full}",
    )

    # Pose check: diverter slides upward when extended
    rest_z = None
    extended_z = None
    if div_grip_rest is not None:
        rest_z = (div_grip_rest[0][2] + div_grip_rest[1][2]) / 2.0
    with ctx.pose({j_div: 0.025}):
        div_grip_ext = ctx.part_element_world_aabb(diverter, elem="diverter_grip")
        if div_grip_ext is not None:
            extended_z = (div_grip_ext[0][2] + div_grip_ext[1][2]) / 2.0
    ctx.check(
        "diverter knob slides upward when extended",
        rest_z is not None and extended_z is not None
        and extended_z > rest_z + 0.020,
        details=f"rest_z={rest_z}, extended_z={extended_z}",
    )

    # ---- Handle revolute joints ----
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

    # Decisive pose: cross handles spin about vertical axis
    def _ball_center(handle: Part):
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
        rest_left is not None and posed_left is not None
        and math.hypot(
            posed_left[0] - rest_left[0], posed_left[1] - rest_left[1]
        ) > 0.02,
        details=f"rest={rest_left}, posed={posed_left}",
    )

    rest_right = _ball_center(right_handle)
    with ctx.pose({j_right: -math.pi / 4.0}):
        posed_right = _ball_center(right_handle)
    ctx.check(
        "right handle spins independently about its stem axis",
        rest_right is not None and posed_right is not None
        and math.hypot(
            posed_right[0] - rest_right[0], posed_right[1] - rest_right[1]
        ) > 0.02,
        details=f"rest={rest_right}, posed={posed_right}",
    )

    return ctx.report()


object_model = build_object_model()
