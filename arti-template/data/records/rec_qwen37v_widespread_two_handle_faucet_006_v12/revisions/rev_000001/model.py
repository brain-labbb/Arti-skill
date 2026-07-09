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
# Variant 12: widespread two-handle faucet with low bridge arch spout,
# prismatic diverter knob, and narrow deck-base seams.
#
# Layout (meters, Z up):
#   - dark deck plate (root) with three chrome pieces mounted on top (z = 0)
#   - center spout column at x = 0: tapered square-pyramid base (0.07 sq at
#     deck -> 0.046 sq at z = 0.08), stepped square cap; a low bridge arch
#     spans outward along X between the two valve columns
#   - diverter knob behind the spout (-Y) on a prismatic Z-axis slide
#   - valve columns at x = +/-0.15: smaller tapered pyramids (0.06 sq, 0.07
#     tall) with square cap and slim stem carrying a four-spoke cross handle
#   - narrow dark seam rings at all three deck bases
# Articulation: cross handles revolute about vertical stem axis (-pi..pi);
# diverter knob prismatic along Z (0..0.015 m travel).
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

# Bridge arch spout
ARCH_SPAN = 0.098  # half-span outward from center along X (shortened to clear handle balls)
ARCH_WIDTH = 0.040  # width of the arch slab in Y
ARCH_THICK = 0.012  # slab thickness
ARCH_PEAK_Z = 0.045  # peak height above the cap top (unused, kept for reference)

# Diverter knob (prismatic)
DIV_KNOB_R = 0.009
DIV_KNOB_H = 0.018
DIV_STEM_R = 0.004
DIV_STEM_H = 0.020
DIV_BEHIND_Y = -0.022  # behind the spout center (-Y), within cap extent
DIV_REST_Z = CAP_TOP_Z + 0.001  # rests at 0.099, knob body just above the cap top
DIV_TRAVEL = 0.015  # upward travel

# Guide bracket on the back of the spout column
BRACKET_W = 0.020  # X width
BRACKET_D = 0.018  # Y depth (from column back face)
BRACKET_H = 0.030  # Z height
BRACKET_Z = C_PYR_H + 0.005  # bracket bottom at 0.085

# Deck base seams
SEAM_WIDTH = 0.002  # seam thickness
SEAM_INSET = 0.003  # how much larger than base the seam extends


def _pyramid_frustum(base: float, top: float, height: float) -> cq.Workplane:
    """Tapered square-pyramid column, base on z=0, narrowing upward."""
    return (
        cq.Workplane("XY")
        .rect(base, base)
        .workplane(offset=height)
        .rect(top, top)
        .loft(combine=True)
    )


def _bridge_arch() -> cq.Workplane:
    """Low bridge arch spanning outward along X from the center column.

    A gentle parabolic arch in the XZ plane, extruded along Y for a flat
    Art-Deco slab profile. The arch root is embedded inside the center
    pyramid column so it reads as emerging from the body.
    """
    # Arch bottom at center (x=0) is embedded inside the pyramid top / cap
    arch_base_z = C_PYR_H  # 0.080, at the pyramid top / cap_lower bottom
    arch_rise = 0.018  # arch rises 18mm from root to tips (gentle upward sweep)

    n_pts = 11
    top_pts = []
    bot_pts = []
    for i in range(n_pts):
        t = i / (n_pts - 1)
        x = -ARCH_SPAN + 2.0 * ARCH_SPAN * t
        ratio = (x / ARCH_SPAN) ** 2
        # Bottom curve: rises from center outward (bridge arch shape)
        z_bot = arch_base_z + arch_rise * ratio
        z_top = z_bot + ARCH_THICK
        bot_pts.append((x, z_bot))
        top_pts.append((x, z_top))

    # Build closed profile in XZ plane
    # CadQuery XZ workplane extrudes along -Y, so we translate to center
    wp = cq.Workplane("XZ")
    wp = wp.moveTo(top_pts[0][0], top_pts[0][1])
    for pt in top_pts[1:]:
        wp = wp.lineTo(pt[0], pt[1])
    wp = wp.lineTo(bot_pts[-1][0], bot_pts[-1][1])
    for pt in reversed(bot_pts[:-1]):
        wp = wp.lineTo(pt[0], pt[1])
    wp = wp.close()

    arch = wp.extrude(ARCH_WIDTH)
    # XZ extrudes along -Y; shift so arch is centered on Y=0
    arch = arch.translate((0.0, ARCH_WIDTH / 2.0, 0.0))
    return arch


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

    # --- Center spout column with low bridge arch ---
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
    # Low bridge arch spout
    spout_body.visual(
        mesh_from_cadquery(_bridge_arch(), "bridge_arch"),
        material=chrome.name,
        name="spout_arch",
    )
    # Guide bracket on the back of the center column for the diverter
    bracket_y_center = -(C_PYR_TOP / 2.0 + BRACKET_D / 2.0)
    spout_body.visual(
        Box((BRACKET_W, BRACKET_D, BRACKET_H)),
        origin=Origin(xyz=(0.0, bracket_y_center, BRACKET_Z + BRACKET_H / 2.0)),
        material=chrome.name,
        name="diverter_bracket",
    )
    # Narrow seam at center base
    seam_size = C_PYR_BASE + 2 * SEAM_INSET
    spout_body.visual(
        Box((seam_size, seam_size, SEAM_WIDTH)),
        origin=Origin(xyz=(0.0, 0.0, SEAM_WIDTH / 2.0)),
        material=seam_mat,
        name="center_seam",
    )
    model.articulation(
        "deck_to_spout_body",
        ArticulationType.FIXED,
        parent=deck,
        child=spout_body,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # --- Diverter knob behind the spout, prismatic up-down ---
    diverter = model.part("diverter_knob")
    # Stem embedded in the spout body slot
    diverter.visual(
        Cylinder(radius=DIV_STEM_R, length=DIV_STEM_H),
        origin=Origin(xyz=(0.0, 0.0, -DIV_STEM_H / 2.0)),
        material=chrome.name,
        name="diverter_stem",
    )
    # Knob body above
    diverter.visual(
        Cylinder(radius=DIV_KNOB_R, length=DIV_KNOB_H),
        origin=Origin(xyz=(0.0, 0.0, DIV_KNOB_H / 2.0)),
        material=chrome.name,
        name="diverter_knob_body",
    )
    # Small dome on top of knob
    diverter.visual(
        Sphere(radius=DIV_KNOB_R * 0.7),
        origin=Origin(xyz=(0.0, 0.0, DIV_KNOB_H)),
        material=chrome.name,
        name="diverter_dome",
    )
    model.articulation(
        "diverter_slide",
        ArticulationType.PRISMATIC,
        parent=spout_body,
        child=diverter,
        origin=Origin(xyz=(0.0, DIV_BEHIND_Y, DIV_REST_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=0.5, lower=0.0, upper=DIV_TRAVEL
        ),
    )

    # --- Valve columns and cross handles (left = -X, right = +X) ---
    for side, sx in (("left", -1.0), ("right", 1.0)):
        valve = model.part(f"{side}_valve")
        _add_valve_column(valve, chrome.name)
        # Narrow seam at valve base
        v_seam_size = V_PYR_BASE + 2 * SEAM_INSET
        valve.visual(
            Box((v_seam_size, v_seam_size, SEAM_WIDTH)),
            origin=Origin(xyz=(0.0, 0.0, SEAM_WIDTH / 2.0)),
            material=seam_mat,
            name=f"{side}_seam",
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
    diverter = object_model.get_part("diverter_knob")
    left_valve = object_model.get_part("left_valve")
    right_valve = object_model.get_part("right_valve")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")
    j_left = object_model.get_articulation("left_handle_spin")
    j_right = object_model.get_articulation("right_handle_spin")
    j_div = object_model.get_articulation("diverter_slide")

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
    # Diverter stem seated in spout body bracket guide slot
    ctx.allow_overlap(
        diverter,
        spout_body,
        elem_a="diverter_stem",
        elem_b="diverter_bracket",
        reason="Diverter stem is intentionally embedded in the bracket guide slot.",
    )
    # Diverter stem passes through the center column body
    ctx.allow_overlap(
        diverter,
        spout_body,
        elem_a="diverter_stem",
        elem_b="cap_step_lower",
        reason="Diverter stem passes through the center column cap as a guide.",
    )
    # Diverter knob body slides through the bracket
    ctx.allow_overlap(
        diverter,
        spout_body,
        elem_a="diverter_knob_body",
        elem_b="diverter_bracket",
        reason="Diverter knob body is intentionally represented as sliding through the bracket guide.",
    )

    # --- Proof checks for diverter allowances ---
    ctx.expect_overlap(
        diverter,
        spout_body,
        axes="xy",
        elem_a="diverter_stem",
        elem_b="diverter_bracket",
        min_overlap=0.002,
        name="diverter stem has XY insertion in the bracket guide",
    )
    ctx.expect_overlap(
        diverter,
        spout_body,
        axes="z",
        elem_a="diverter_knob_body",
        elem_b="diverter_bracket",
        min_overlap=0.008,
        name="diverter knob body retains Z insertion in the bracket at rest",
    )

    # --- All three chrome pieces seated on the dark deck, not floating ---
    for piece in (spout_body, left_valve, right_valve):
        ctx.expect_gap(
            piece,
            deck,
            axis="z",
            max_gap=0.003,
            max_penetration=0.001,
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

    # --- Bridge arch: low profile, spans between handles ---
    arch_aabb = ctx.part_element_world_aabb(spout_body, elem="spout_arch")
    ctx.check(
        "bridge arch spans outward along X between the valve columns",
        arch_aabb is not None and (arch_aabb[1][0] - arch_aabb[0][0]) > 0.18,
        details=f"arch aabb={arch_aabb}",
    )
    ctx.check(
        "bridge arch is low profile (peak well below 0.20 m)",
        arch_aabb is not None and arch_aabb[1][2] < 0.20,
        details=f"arch aabb={arch_aabb}",
    )
    ctx.check(
        "bridge arch sits above the cap",
        arch_aabb is not None and arch_aabb[0][2] > C_PYR_H - 0.01,
        details=f"arch aabb={arch_aabb}",
    )

    # --- Narrow seams at all three deck bases ---
    for piece, seam_name in (
        (spout_body, "center_seam"),
        (left_valve, "left_seam"),
        (right_valve, "right_seam"),
    ):
        seam_aabb = ctx.part_element_world_aabb(piece, elem=seam_name)
        ctx.check(
            f"{seam_name} visible at base of {piece.name}",
            seam_aabb is not None and abs(seam_aabb[1][2] - SEAM_WIDTH) < 0.002,
            details=f"{seam_name} aabb={seam_aabb}",
        )

    # --- Diverter knob: prismatic joint, behind spout, slides up-down ---
    ctx.check(
        "diverter_slide is a prismatic joint",
        j_div.articulation_type == ArticulationType.PRISMATIC,
    )
    div_lim = j_div.motion_limits
    ctx.check(
        "diverter prismatic range is 0 to 0.015 m travel",
        div_lim is not None
        and div_lim.lower is not None
        and div_lim.upper is not None
        and abs(div_lim.lower) < 0.001
        and abs(div_lim.upper - DIV_TRAVEL) < 0.001,
    )
    # Diverter is behind the spout (-Y)
    div_pos = ctx.part_world_position(diverter)
    spout_pos = ctx.part_world_position(spout_body)
    ctx.check(
        "diverter knob is behind the spout (negative Y)",
        div_pos is not None and spout_pos is not None and div_pos[1] < spout_pos[1] - 0.02,
        details=f"diverter={div_pos}, spout={spout_pos}",
    )

    # --- Decisive pose: diverter slides upward ---
    rest_div_z = ctx.part_world_position(diverter)
    with ctx.pose({j_div: DIV_TRAVEL}):
        extended_div_z = ctx.part_world_position(diverter)
        ctx.expect_overlap(
            diverter,
            spout_body,
            axes="xy",
            elem_a="diverter_knob_body",
            elem_b="diverter_bracket",
            min_overlap=0.006,
            name="diverter knob body retains XY overlap with bracket at max travel",
        )
    ctx.check(
        "diverter knob slides upward when prismatic joint is actuated",
        rest_div_z is not None
        and extended_div_z is not None
        and extended_div_z[2] > rest_div_z[2] + 0.010,
        details=f"rest={rest_div_z}, extended={extended_div_z}",
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
            positive_elem="hub",
            negative_elem="valve_stem",
            max_gap=0.0005,
            max_penetration=0.004,
            name=f"{handle.name} hub seats over the valve stem",
        )
        ctx.expect_within(
            handle,
            valve,
            axes="xy",
            inner_elem="hub",
            margin=0.005,
            name=f"{handle.name} hub centered on its valve column",
        )

    # --- Joint limits: handles are revolute -pi..pi ---
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

    # --- Decisive pose checks for handles ---
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
