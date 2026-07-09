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
# Widespread two-handle faucet variant (Art-Deco chrome, three-piece layout).
#
# Layout (meters, Z up, spout sweeps forward along +Y):
#   - dark deck plate (root) with bridge bar linking three chrome posts
#   - center spout column at x = 0: tapered square-pyramid base, stepped cap,
#     flat-topped waterfall spout with hollow outlet, reach ~0.18 m
#   - diverter knob behind spout (at -Y), prismatic vertical slide
#   - valve columns at x = +/-0.15: tapered pyramids with cross handles
#   - narrow seam rings at all three deck bases
#   - bridge bar: chrome horizontal member visually linking all three posts
#
# Articulation:
#   - left/right cross handles: revolute about vertical stem axis (-pi..pi)
#   - diverter knob: prismatic along Z (0..0.025 m)
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

# Spout
SPOUT_WIDTH = 0.050

# Bridge bar (two segments between posts, penetrating pyramid walls slightly)
BRIDGE_Z_BOTTOM = 0.0  # bottom face on deck top for contact
BRIDGE_WIDTH = 0.016  # front-to-back depth
BRIDGE_HEIGHT = 0.012  # vertical thickness
# Pyramid base half-widths: valve 0.030, center 0.035
# Penetrate 2mm into each pyramid wall for physical connectivity
BRIDGE_LEFT_X0 = -0.122   # 2mm inside left valve right edge (-0.120)
BRIDGE_LEFT_X1 = -0.033   # 2mm inside center left edge (-0.035)
BRIDGE_RIGHT_X0 = 0.033   # 2mm inside center right edge (+0.035)
BRIDGE_RIGHT_X1 = 0.122   # 2mm inside right valve left edge (+0.120)

# Seam rings
SEAM_THICKNESS = 0.002
SEAM_OVERHANG = 0.003  # extends beyond base by this amount

# Diverter knob (behind spout at -Y, contacts pyramid wall)
DIV_KNOB_R = 0.008
DIV_KNOB_H = 0.018
DIV_KNOB_Y = -0.030  # knob front face contacts/overlaps pyramid back wall
DIV_SLIDE_RANGE = 0.025  # prismatic travel


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
    """Wide flat-topped spout with hollow outlet at the tip.

    The outer shell sweeps forward (+Y) into a waterfall arc.
    A cavity is cut from the underside of the tip to create a hollow outlet.
    """
    # Outer shell profile in YZ plane, extruded across X
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

    # Hollow outlet cavity: cut a rectangular pocket from the underside of the tip
    # The cavity sits near the spout tip (y ~ 0.155..0.175) on the underside
    cavity_width = SPOUT_WIDTH * 0.6
    cavity = (
        cq.Workplane("XY")
        .center(0.0, 0.162)
        .rect(cavity_width, 0.020)
        .extrude(0.030)  # cut upward from below
        .translate((0.0, 0.0, 0.010))
    )
    spout = spout.cut(cavity)
    return spout


def _bridge_bar() -> cq.Workplane:
    """Continuous chrome bridge bar linking the three posts.

    A single horizontal bar spanning the full width, with rectangular pockets
    cut at each pyramid location. Pockets are slightly narrower in X and Y
    than the pyramid bases, leaving thin connecting strips that keep the bar
    as one solid while the pocket walls overlap the pyramid faces.
    """
    total_half = HANDLE_SPREAD_X + V_PYR_BASE / 2.0 + 0.005
    z_bot = BRIDGE_Z_BOTTOM

    # Full bar
    bar = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, z_bot + BRIDGE_HEIGHT / 2.0))
        .box(total_half * 2.0, BRIDGE_WIDTH, BRIDGE_HEIGHT)
    )

    # Cut pockets slightly smaller than pyramid bases (1mm overlap each side in X)
    # Leave 2mm strips at front/back (Y) so the bar stays connected
    pocket_h = BRIDGE_HEIGHT + 0.002  # through-cut in Z
    pocket_y = BRIDGE_WIDTH - 0.004   # 2mm strips remain at front and back

    # Center pocket (pyramid base 0.070, pocket 0.068)
    center_pocket_w = C_PYR_BASE - 0.002
    center_cut = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, z_bot + pocket_h / 2.0))
        .box(center_pocket_w, pocket_y, pocket_h)
    )
    bar = bar.cut(center_cut)

    # Left valve pocket (pyramid base 0.060, pocket 0.058)
    valve_pocket_w = V_PYR_BASE - 0.002
    left_cut = (
        cq.Workplane("XY")
        .transformed(offset=(-HANDLE_SPREAD_X, 0.0, z_bot + pocket_h / 2.0))
        .box(valve_pocket_w, pocket_y, pocket_h)
    )
    bar = bar.cut(left_cut)

    # Right valve pocket
    right_cut = (
        cq.Workplane("XY")
        .transformed(offset=(HANDLE_SPREAD_X, 0.0, z_bot + pocket_h / 2.0))
        .box(valve_pocket_w, pocket_y, pocket_h)
    )
    bar = bar.cut(right_cut)

    return bar


def _seam_ring(base_size: float) -> cq.Workplane:
    """Thin dark seam ring at the base of a post, slightly larger than the base."""
    outer = base_size + SEAM_OVERHANG * 2
    inner = base_size - 0.001
    # Build outer solid frame
    outer_solid = (
        cq.Workplane("XY")
        .rect(outer, outer)
        .extrude(SEAM_THICKNESS)
    )
    # Build inner cutout
    inner_solid = (
        cq.Workplane("XY")
        .rect(inner, inner)
        .extrude(SEAM_THICKNESS)
    )
    # Boolean subtract
    return outer_solid.cut(inner_solid)


def _diverter_knob() -> cq.Workplane:
    """Small cylindrical diverter knob with a grip ridge."""
    body = (
        cq.Workplane("XY")
        .circle(DIV_KNOB_R)
        .extrude(DIV_KNOB_H)
    )
    # Add a small grip ridge (wider ring near the top)
    ridge = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, DIV_KNOB_H * 0.75))
        .circle(DIV_KNOB_R + 0.002)
        .extrude(0.004)
    )
    return body.union(ridge)


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


def _add_valve_column(part: Part, chrome: str, seam_dark: str) -> None:
    """Tapered pyramid valve base with square cap, slim bonnet stem, and seam."""
    # Seam ring at base
    part.visual(
        mesh_from_cadquery(_seam_ring(V_PYR_BASE), f"{part.name}_seam"),
        origin=Origin(xyz=(0.0, 0.0, -SEAM_THICKNESS / 2.0)),
        material=seam_dark,
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
    model = ArticulatedObject(name="widespread_two_handle_faucet")

    chrome = model.material("chrome", rgba=(0.88, 0.89, 0.92, 1.0))
    deck_mat = model.material("deck_charcoal", rgba=(0.09, 0.09, 0.10, 1.0))
    seam_dark = model.material("seam_dark", rgba=(0.04, 0.04, 0.05, 1.0))

    # --- Dark deck plate (root) ---
    deck = model.part("deck")
    deck.visual(
        Box((0.42, 0.20, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=deck_mat,
        name="deck_plate",
    )

    # --- Bridge bar linking all three posts ---
    bridge = model.part("bridge_bar")
    bridge.visual(
        mesh_from_cadquery(_bridge_bar(), "bridge_bar_mesh"),
        material=chrome.name,
        name="bridge_bar",
    )
    model.articulation(
        "deck_to_bridge",
        ArticulationType.FIXED,
        parent=deck,
        child=bridge,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # --- Center spout column ---
    spout_body = model.part("spout_body")
    # Seam at center base
    spout_body.visual(
        mesh_from_cadquery(_seam_ring(C_PYR_BASE), "center_seam"),
        origin=Origin(xyz=(0.0, 0.0, -SEAM_THICKNESS / 2.0)),
        material=seam_dark.name,
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
        mesh_from_cadquery(_waterfall_spout_hollow(), "waterfall_spout_hollow"),
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

    # --- Diverter knob behind the spout, prismatic vertical slide ---
    diverter = model.part("diverter_knob")
    diverter.visual(
        mesh_from_cadquery(_diverter_knob(), "diverter_knob_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=chrome.name,
        name="knob_body",
    )
    # Small stem that penetrates into the spout body (guide rail proxy)
    diverter.visual(
        Cylinder(radius=0.004, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=chrome.name,
        name="knob_stem",
    )
    model.articulation(
        "diverter_slide",
        ArticulationType.PRISMATIC,
        parent=spout_body,
        child=diverter,
        origin=Origin(xyz=(0.0, DIV_KNOB_Y, CAP_TOP_Z * 0.6)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=0.5, lower=0.0, upper=DIV_SLIDE_RANGE
        ),
    )

    # --- Valve columns and cross handles ---
    for side, sx in (("left", -1.0), ("right", 1.0)):
        valve = model.part(f"{side}_valve")
        _add_valve_column(valve, chrome.name, seam_dark.name)
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
    bridge = object_model.get_part("bridge_bar")
    spout_body = object_model.get_part("spout_body")
    diverter = object_model.get_part("diverter_knob")
    left_valve = object_model.get_part("left_valve")
    right_valve = object_model.get_part("right_valve")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")
    j_left = object_model.get_articulation("left_handle_spin")
    j_right = object_model.get_articulation("right_handle_spin")
    j_div = object_model.get_articulation("diverter_slide")

    # --- Intentional overlaps: handle hubs capture valve stems ---
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
    # Bridge bar segments penetrate pyramid walls for structural connectivity
    ctx.allow_overlap(
        bridge,
        left_valve,
        elem_a="bridge_bar",
        elem_b="valve_pyramid",
        reason="Bridge bar penetrates left valve pyramid wall for visual and structural linking.",
    )
    ctx.allow_overlap(
        bridge,
        right_valve,
        elem_a="bridge_bar",
        elem_b="valve_pyramid",
        reason="Bridge bar penetrates right valve pyramid wall for visual and structural linking.",
    )
    ctx.allow_overlap(
        bridge,
        spout_body,
        elem_a="bridge_bar",
        elem_b="spout_pyramid",
        reason="Bridge bar penetrates center spout pyramid wall for visual and structural linking.",
    )
    # Diverter knob stem and body nested in spout body wall
    ctx.allow_overlap(
        diverter,
        spout_body,
        elem_a="knob_stem",
        elem_b="spout_pyramid",
        reason="Diverter knob stem is a prismatic guide inserted into the spout body.",
    )
    ctx.allow_overlap(
        diverter,
        spout_body,
        elem_a="knob_body",
        elem_b="spout_pyramid",
        reason="Diverter knob body is seated against the spout pyramid back wall.",
    )

    # --- Bridge bar spans between the three posts ---
    bridge_aabb = ctx.part_world_aabb(bridge)
    ctx.check(
        "bridge bar spans most of the handle spread width",
        bridge_aabb is not None
        and (bridge_aabb[1][0] - bridge_aabb[0][0]) > 0.26,
        details=f"bridge aabb={bridge_aabb}",
    )
    # Bridge bar overlaps each pyramid base (connectivity via notched fit)
    ctx.expect_overlap(
        bridge,
        left_valve,
        axes="x",
        elem_a="bridge_bar",
        elem_b="valve_pyramid",
        min_overlap=0.001,
        name="bridge bar overlaps left valve pyramid base",
    )
    ctx.expect_overlap(
        bridge,
        right_valve,
        axes="x",
        elem_a="bridge_bar",
        elem_b="valve_pyramid",
        min_overlap=0.001,
        name="bridge bar overlaps right valve pyramid base",
    )
    ctx.expect_overlap(
        bridge,
        spout_body,
        axes="x",
        elem_a="bridge_bar",
        elem_b="spout_pyramid",
        min_overlap=0.001,
        name="bridge bar overlaps center spout pyramid base",
    )
    # Bridge bar is at deck level
    ctx.expect_gap(
        bridge,
        deck,
        axis="z",
        max_gap=0.001,
        max_penetration=0.002,
        name="bridge bar sits on deck surface",
    )

    # --- Seam rings present at all three bases ---
    for post, seam_name in [
        (spout_body, "base_seam"),
        (left_valve, "base_seam"),
        (right_valve, "base_seam"),
    ]:
        seam_aabb = ctx.part_element_world_aabb(post, elem=seam_name)
        ctx.check(
            f"{post.name} has a seam ring at its base",
            seam_aabb is not None,
            details=f"seam aabb={seam_aabb}",
        )

    # --- All chrome pieces seated on deck ---
    for piece in (spout_body, left_valve, right_valve):
        ctx.expect_gap(
            piece,
            deck,
            axis="z",
            max_gap=0.003,
            max_penetration=0.001,
            name=f"{piece.name} base seated on deck top",
        )

    # --- Three-piece spread ~0.30 m ---
    ctx.expect_origin_distance(
        left_handle,
        right_handle,
        axes="x",
        min_dist=0.29,
        max_dist=0.31,
        name="handle spread is about 0.30 m",
    )

    # --- Diverter knob: prismatic joint with correct limits ---
    div_lim = j_div.motion_limits
    ctx.check(
        "diverter_slide is prismatic with 0..0.025 m range",
        div_lim is not None
        and j_div.articulation_type == ArticulationType.PRISMATIC
        and abs(div_lim.lower) < 0.001
        and abs(div_lim.upper - DIV_SLIDE_RANGE) < 0.001,
        details=f"limits={div_lim}",
    )
    # Diverter knob is behind the spout (at negative Y)
    div_aabb = ctx.part_world_aabb(diverter)
    spout_aabb = ctx.part_world_aabb(spout_body)
    ctx.check(
        "diverter knob is positioned behind spout (negative Y from spout center)",
        div_aabb is not None and spout_aabb is not None
        and (div_aabb[0][1] + div_aabb[1][1]) / 2.0 < (spout_aabb[0][1] + spout_aabb[1][1]) / 2.0,
        details=f"diverter_y={(div_aabb[0][1]+div_aabb[1][1])/2.0 if div_aabb else None}, spout_y={(spout_aabb[0][1]+spout_aabb[1][1])/2.0 if spout_aabb else None}",
    )

    # --- Decisive pose: diverter knob slides upward ---
    div_rest_pos = ctx.part_world_position(diverter)
    with ctx.pose({j_div: DIV_SLIDE_RANGE}):
        div_extended_pos = ctx.part_world_position(diverter)
    ctx.check(
        "diverter knob slides upward when actuated",
        div_rest_pos is not None
        and div_extended_pos is not None
        and div_extended_pos[2] > div_rest_pos[2] + 0.010,
        details=f"rest={div_rest_pos}, extended={div_extended_pos}",
    )

    # --- Spout reaches forward with hollow outlet ---
    spout_elem_aabb = ctx.part_element_world_aabb(spout_body, elem="spout")
    ctx.check(
        "spout reaches about 0.18 m forward",
        spout_elem_aabb is not None and 0.14 <= spout_elem_aabb[1][1] <= 0.20,
        details=f"spout aabb={spout_elem_aabb}",
    )

    # --- Cross handles: seated and spin ---
    for handle, valve, joint in [
        (left_handle, left_valve, j_left),
        (right_handle, right_valve, j_right),
    ]:
        ctx.expect_gap(
            handle,
            valve,
            axis="z",
            max_gap=0.0005,
            max_penetration=0.004,
            name=f"{handle.name} hub seats over the valve stem",
        )
        lim = joint.motion_limits
        ctx.check(
            f"{joint.name} range is -180..+180 deg",
            lim is not None
            and abs(lim.lower + math.pi) < 0.01
            and abs(lim.upper - math.pi) < 0.01,
        )

    # --- Decisive pose: handles spin ---
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
        and math.hypot(posed_left[0] - rest_left[0], posed_left[1] - rest_left[1]) > 0.02,
        details=f"rest={rest_left}, posed={posed_left}",
    )

    rest_right = _ball_center(right_handle)
    with ctx.pose({j_right: -math.pi / 4.0}):
        posed_right = _ball_center(right_handle)
    ctx.check(
        "right handle spins independently about its stem axis",
        rest_right is not None
        and posed_right is not None
        and math.hypot(posed_right[0] - rest_right[0], posed_right[1] - rest_right[1]) > 0.02,
        details=f"rest={rest_right}, posed={posed_right}",
    )

    return ctx.report()


object_model = build_object_model()
