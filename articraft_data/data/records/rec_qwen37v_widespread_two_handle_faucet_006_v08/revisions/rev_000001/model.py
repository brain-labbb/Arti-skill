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
#
# Layout (meters, Z up, spout channel extends along +Y):
#   - dark deck plate (root) with three chrome pieces mounted on top (z = 0)
#   - center spout column at x = 0: tapered square-pyramid base (0.07 sq at
#     deck -> 0.046 sq at z = 0.08), stepped square cap, short rectangular
#     waterfall channel reaching ~0.12 m forward with hollow outlet
#   - valve columns at x = +/-0.15: smaller tapered pyramids (0.06 sq, 0.07
#     tall) with square cap and slim stem carrying a lever handle
#   - narrow dark seam rings at all three deck bases
# Articulation: each lever handle is REVOLUTE about the X-axis (forward-back
# tilt), limits -0.5 to +0.5 rad (~±29°). Spout body is fixed.
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

# Lever handle
HUB_R = 0.009
HUB_H = 0.018
LEVER_WIDTH = 0.012
LEVER_HEIGHT = 0.008
LEVER_LENGTH = 0.055
GRIP_R = 0.007

# Rectangular waterfall channel
CHANNEL_WIDTH = 0.048
CHANNEL_HEIGHT = 0.022
CHANNEL_LENGTH = 0.120
CHANNEL_WALL = 0.004
CHANNEL_ROOT_Y = 0.012  # root embedded in column

# Deck seam
SEAM_THICKNESS = 0.0015
SEAM_WIDTH = 0.003


def _pyramid_frustum(base: float, top: float, height: float) -> cq.Workplane:
    """Tapered square-pyramid column, base on z=0, narrowing upward."""
    return (
        cq.Workplane("XY")
        .rect(base, base)
        .workplane(offset=height)
        .rect(top, top)
        .loft(combine=True)
    )


def _rectangular_waterfall_channel() -> cq.Workplane:
    """Short rectangular waterfall channel with hollow outlet.

    A rectangular tube extending forward (+Y) from the column. The far end
    is open, revealing the hollow interior (the waterfall outlet). The near
    end is closed (root embedded in the column body).
    """
    outer_w = CHANNEL_WIDTH
    outer_h = CHANNEL_HEIGHT
    length = CHANNEL_LENGTH
    wall = CHANNEL_WALL

    # Outer shell box, centered at origin, extending from y=0 to y=length
    outer = (
        cq.Workplane("XY")
        .box(outer_w, length, outer_h, centered=(True, False, True))
    )

    # Hollow interior: slightly smaller, open at the far +Y end
    inner_w = outer_w - 2 * wall
    inner_h = outer_h - wall  # floor stays solid, top open effect via outlet
    # Interior extends from near end (y = wall) to far end (y = length + 0.001)
    # to create open outlet
    inner_length = length - wall + 0.002
    inner = (
        cq.Workplane("XY")
        .box(inner_w, inner_length, inner_h, centered=(True, False, True))
        .translate((0, wall + (inner_length - length) / 2.0, wall / 2.0))
    )

    channel = outer.cut(inner)
    return channel


def _base_seam(base_size: float) -> cq.Workplane:
    """Narrow dark seam ring at the base perimeter.

    A thin frame outline at z=0 showing where the fixture meets the deck.
    Modeled as a thin flat ring (outer square minus inner square).
    """
    outer = base_size + 2 * SEAM_WIDTH
    inner = base_size - 0.001  # slight inset
    seam = (
        cq.Workplane("XY")
        .rect(outer, outer)
        .extrude(SEAM_THICKNESS)
        .faces(">Z")
        .workplane()
        .rect(inner, inner)
        .cutThruAll()
    )
    return seam


def _add_lever_handle(part: Part, chrome: str) -> None:
    """Lever handle with forward-extending bar, rotating about local X-axis.

    Local frame origin is the handle joint frame: hub bottom at z=0.
    The lever extends along +Y at rest (forward direction).
    """
    # Hub cylinder
    part.visual(
        Cylinder(radius=HUB_R, length=HUB_H),
        origin=Origin(xyz=(0.0, 0.0, HUB_H / 2.0)),
        material=chrome,
        name="hub",
    )
    # Hub dome cap
    part.visual(
        Sphere(radius=HUB_R),
        origin=Origin(xyz=(0.0, 0.0, HUB_H)),
        material=chrome,
        name="hub_dome",
    )
    # Lever arm - rectangular bar extending along +Y
    lever_y_center = LEVER_LENGTH / 2.0 + HUB_R * 0.3
    lever_z = HUB_H * 0.6
    part.visual(
        Box((LEVER_WIDTH, LEVER_LENGTH, LEVER_HEIGHT)),
        origin=Origin(xyz=(0.0, lever_y_center, lever_z)),
        material=chrome,
        name="lever_arm",
    )
    # Grip sphere at the lever tip
    grip_y = HUB_R * 0.3 + LEVER_LENGTH
    part.visual(
        Sphere(radius=GRIP_R),
        origin=Origin(xyz=(0.0, grip_y, lever_z)),
        material=chrome,
        name="lever_grip",
    )


def _add_valve_column(part: Part, chrome: str, seam_mat: str) -> None:
    """Tapered pyramid valve base with square cap, slim bonnet stem, and seam."""
    # Base seam
    part.visual(
        mesh_from_cadquery(_base_seam(V_PYR_BASE), f"{part.name}_seam"),
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

    # --- Center spout column ---
    spout_body = model.part("spout_body")
    # Base seam for center column
    spout_body.visual(
        mesh_from_cadquery(_base_seam(C_PYR_BASE), "center_seam"),
        material=seam_mat,
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
    # Rectangular waterfall channel spout
    # Position: emerges from near the top of the column, extends forward (+Y)
    channel_z = C_PYR_H + CAP1_H  # at cap level
    spout_body.visual(
        mesh_from_cadquery(
            _rectangular_waterfall_channel(), "waterfall_channel"
        ),
        origin=Origin(xyz=(0.0, CHANNEL_ROOT_Y, channel_z + CHANNEL_HEIGHT / 2.0)),
        material=chrome.name,
        name="spout_channel",
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
        _add_valve_column(valve, chrome.name, seam_mat)
        model.articulation(
            f"deck_to_{side}_valve",
            ArticulationType.FIXED,
            parent=deck,
            child=valve,
            origin=Origin(xyz=(sx * HANDLE_SPREAD_X, 0.0, 0.0)),
        )

        handle = model.part(f"{side}_handle")
        _add_lever_handle(handle, chrome.name)
        # Lever rotates forward-back about X-axis
        # Positive q: lever end tips from +Y toward +Z (up/back)
        # Negative q: lever end tips from +Y toward -Z (down/forward)
        model.articulation(
            f"{side}_handle_tilt",
            ArticulationType.REVOLUTE,
            parent=valve,
            child=handle,
            origin=Origin(xyz=(0.0, 0.0, HANDLE_JOINT_Z)),
            axis=(1.0, 0.0, 0.0),
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

    # Intentional captured fits: handle hubs over valve stems.
    ctx.allow_overlap(
        left_handle,
        left_valve,
        elem_a="hub",
        elem_b="valve_stem",
        reason="Lever handle hub intentionally captures the valve bonnet stem.",
    )
    ctx.allow_overlap(
        right_handle,
        right_valve,
        elem_a="hub",
        elem_b="valve_stem",
        reason="Lever handle hub intentionally captures the valve bonnet stem.",
    )

    # --- All three chrome pieces seated on the dark deck, not floating ---
    for piece in (spout_body, left_valve, right_valve):
        ctx.expect_gap(
            piece,
            deck,
            axis="z",
            max_gap=0.003,
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

    # --- Rectangular waterfall channel spout ---
    channel_aabb = ctx.part_element_world_aabb(spout_body, elem="spout_channel")
    ctx.check(
        "spout channel reaches forward about 0.12 m",
        channel_aabb is not None and 0.10 <= channel_aabb[1][1] <= 0.16,
        details=f"channel aabb={channel_aabb}",
    )
    ctx.check(
        "spout channel is rectangular (wider than tall)",
        channel_aabb is not None
        and (channel_aabb[1][0] - channel_aabb[0][0]) > (channel_aabb[1][2] - channel_aabb[0][2]),
        details=f"channel aabb={channel_aabb}",
    )

    # --- Hollow outlet: the channel interior is cut out ---
    # The channel visual should have less volume than a solid box of same AABB
    # We check that the channel bounding box height is close to CHANNEL_HEIGHT
    # but the channel is a shell (has hollow interior via CadQuery cut)
    ctx.check(
        "waterfall channel sits at cap level",
        channel_aabb is not None and channel_aabb[0][2] > C_PYR_H * 0.9,
        details=f"channel aabb={channel_aabb}",
    )

    # --- Narrow seams present at all three deck bases ---
    for piece, elem_name in (
        (spout_body, "base_seam"),
        (left_valve, "base_seam"),
        (right_valve, "base_seam"),
    ):
        seam_aabb = ctx.part_element_world_aabb(piece, elem=elem_name)
        ctx.check(
            f"{piece.name} has a visible base seam",
            seam_aabb is not None
            and (seam_aabb[1][2] - seam_aabb[0][2]) < 0.004,
            details=f"{piece.name} seam aabb={seam_aabb}",
        )

    # --- Lever handles: extend forward, seated over stems ---
    for handle, valve in ((left_handle, left_valve), (right_handle, right_valve)):
        h_aabb = ctx.part_world_aabb(handle)
        ctx.check(
            f"{handle.name} lever extends forward (Y extent > X extent)",
            h_aabb is not None
            and (h_aabb[1][1] - h_aabb[0][1]) > (h_aabb[1][0] - h_aabb[0][0]) + 0.01,
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
            margin=0.005,
            name=f"{handle.name} hub centered on its valve column",
        )

    # --- Joint limits: lever handles forward-back ±0.5 rad ---
    for joint in (j_left, j_right):
        lim = joint.motion_limits
        ctx.check(
            f"{joint.name} range is about -0.5 to +0.5 rad",
            lim is not None
            and lim.lower is not None
            and lim.upper is not None
            and abs(lim.lower + 0.50) < 0.01
            and abs(lim.upper - 0.50) < 0.01,
        )
    # Joint axis is X (forward-back tilt)
    for joint in (j_left, j_right):
        ctx.check(
            f"{joint.name} axis is horizontal (X) for forward-back tilt",
            joint.axis is not None
            and abs(joint.axis[0]) > 0.9
            and abs(joint.axis[1]) < 0.1
            and abs(joint.axis[2]) < 0.1,
        )

    # --- Decisive pose checks: lever tilts forward-back ---
    # At positive q, lever grip should move upward (positive Z change)
    grip_rest_z_left = None
    grip_posed_z_left = None
    grip_rest_aabb = ctx.part_element_world_aabb(left_handle, elem="lever_grip")
    if grip_rest_aabb:
        grip_rest_z_left = (grip_rest_aabb[0][2] + grip_rest_aabb[1][2]) / 2.0
    with ctx.pose({j_left: 0.40}):
        grip_posed_aabb = ctx.part_element_world_aabb(left_handle, elem="lever_grip")
        if grip_posed_aabb:
            grip_posed_z_left = (grip_posed_aabb[0][2] + grip_posed_aabb[1][2]) / 2.0
    ctx.check(
        "left lever handle tilts (grip Z changes with positive pose)",
        grip_rest_z_left is not None
        and grip_posed_z_left is not None
        and abs(grip_posed_z_left - grip_rest_z_left) > 0.005,
        details=f"rest_z={grip_rest_z_left}, posed_z={grip_posed_z_left}",
    )

    grip_rest_z_right = None
    grip_posed_z_right = None
    grip_rest_aabb = ctx.part_element_world_aabb(right_handle, elem="lever_grip")
    if grip_rest_aabb:
        grip_rest_z_right = (grip_rest_aabb[0][2] + grip_rest_aabb[1][2]) / 2.0
    with ctx.pose({j_right: -0.40}):
        grip_posed_aabb = ctx.part_element_world_aabb(right_handle, elem="lever_grip")
        if grip_posed_aabb:
            grip_posed_z_right = (grip_posed_aabb[0][2] + grip_posed_aabb[1][2]) / 2.0
    ctx.check(
        "right lever handle tilts independently (grip Z changes with negative pose)",
        grip_rest_z_right is not None
        and grip_posed_z_right is not None
        and abs(grip_posed_z_right - grip_rest_z_right) > 0.005,
        details=f"rest_z={grip_rest_z_right}, posed_z={grip_posed_z_right}",
    )

    # --- Center pyramid base is about 0.07 m square ---
    pyr_aabb = ctx.part_element_world_aabb(spout_body, elem="spout_pyramid")
    ctx.check(
        "center pyramid base is about 0.07 m square at the deck",
        pyr_aabb is not None
        and 0.066 <= (pyr_aabb[1][0] - pyr_aabb[0][0]) <= 0.074
        and 0.066 <= (pyr_aabb[1][1] - pyr_aabb[0][1]) <= 0.074,
        details=f"pyramid aabb={pyr_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
