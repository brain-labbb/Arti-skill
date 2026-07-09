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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Widespread two-handle bathroom faucet in polished gold brass.
#
# Frame conventions:
#   - The deck (countertop) is the horizontal XY plane; top surface at z = DECK_T.
#   - Everything mounts upward (+Z) from the deck.
#   - The spout projects forward along -Y and arcs upward then downward.
#   - Handles spread along the X axis at x = +/- HANDLE_SPREAD_X.
#   - Both handles rotate around vertical stems (axis +Z).
# ---------------------------------------------------------------------------

# Layout
HANDLE_SPREAD_X = 0.14  # wider spread (original was 0.10)

# Deck plate (mounting substrate)
DECK_W = 0.40
DECK_D = 0.22
DECK_T = 0.012
DECK_TOP_Z = DECK_T

# Spout (taller, deck-mounted arc)
SPOUT_TUBE_R = 0.015  # outer radius (0.03 m diameter)
SPOUT_BORE_R = 0.0105  # inner bore radius (visible at outlet)
SPOUT_RISE = 0.12  # vertical rise before curving forward
SPOUT_PEAK_Z = 0.25  # peak height above deck (taller than original 0.20)

# Spout escutcheon flange (stepped, ~0.07 m diameter)
FLANGE_R1, FLANGE_T1 = 0.035, 0.008
FLANGE_R2, FLANGE_T2 = 0.026, 0.008

# Valve assemblies (vertical orientation, projecting upward from deck)
VALVE_ESC_R1, VALVE_ESC_T1 = 0.033, 0.008
VALVE_ESC_R2, VALVE_ESC_T2 = 0.026, 0.008
VALVE_BODY_R = 0.0145
VALVE_BODY_H = 0.040
VALVE_TOP_Z = VALVE_ESC_T1 + VALVE_ESC_T2 + VALVE_BODY_H

# Stem collar (visible ring between valve body and handle)
COLLAR_R = 0.018
COLLAR_H = 0.010
COLLAR_TOP_Z = VALVE_TOP_Z + COLLAR_H

# Cross handle (horizontal plane, rotating around vertical Z axis)
HANDLE_ROD_R = 0.0045
HANDLE_ROD_LEN = 0.100  # tip-to-tip ~0.10 m
HUB_R = 0.013
HUB_H = 0.026
KNURL_R = 0.0145
STEM_R = 0.007
STEM_LEN = 0.022  # extends through collar into valve body

# Hot / cold cap disks
CAP_R = 0.008
CAP_T = 0.003

# Computed by build_object_model() for hollow-bore verification.
SPOUT_SOLID_VOLUME: float = 0.0
SPOUT_UNBORED_VOLUME: float = 0.0


def _build_spout_solid() -> cq.Workplane:
    """Spout in its local frame: origin at deck top (z=0), tube rises in +Z,
    arcs forward along -Y, curves down to an open hollow outlet."""
    # Path in the YZ plane (local x -> world Y, local y -> world Z).
    # Spout projects along -Y (forward toward user).
    # Single smooth arc from vertical rise to downward outlet.
    path = (
        cq.Workplane("YZ")
        .moveTo(0.0, 0.0)
        .lineTo(0.0, SPOUT_RISE)
        .threePointArc((-0.05, 0.24), (-0.14, 0.15))
    )
    # Bore path slightly extended at both ends for clean boolean cuts.
    bore_path = (
        cq.Workplane("YZ")
        .moveTo(0.0, -0.005)
        .lineTo(0.0, SPOUT_RISE)
        .threePointArc((-0.05, 0.24), (-0.14, 0.14))
    )

    # Profile perpendicular to path start (path starts along +Z, so XY plane).
    tube = cq.Workplane("XY").circle(SPOUT_TUBE_R).sweep(path)
    bore = (
        cq.Workplane("XY")
        .workplane(offset=-0.005)
        .circle(SPOUT_BORE_R)
        .sweep(bore_path)
    )

    # Stepped escutcheon flange at the base (on deck surface).
    flange_outer = cq.Workplane("XY").circle(FLANGE_R1).extrude(FLANGE_T1)
    flange_step = (
        cq.Workplane("XY")
        .workplane(offset=FLANGE_T1)
        .circle(FLANGE_R2)
        .extrude(FLANGE_T2)
    )

    unbored = tube.union(flange_outer).union(flange_step)
    solid = unbored.cut(bore)

    global SPOUT_SOLID_VOLUME, SPOUT_UNBORED_VOLUME
    SPOUT_SOLID_VOLUME = solid.val().Volume()
    SPOUT_UNBORED_VOLUME = unbored.val().Volume()
    return solid


def _build_hub_solid() -> cq.Workplane:
    """Cross-handle central hub in the handle frame: axis +Z (vertical),
    base at z=0, with a knurled (faceted) middle band."""
    hub = cq.Workplane("XY").circle(HUB_R).extrude(HUB_H)
    knurl = (
        cq.Workplane("XY")
        .workplane(offset=0.007)
        .polygon(16, 2.0 * KNURL_R)
        .extrude(0.012)
    )
    # Small dome on top of hub (below the cap disk).
    dome = cq.Workplane("XY").workplane(offset=HUB_H - 0.004).sphere(0.010)
    return hub.union(knurl).union(dome)


def _add_valve_visuals(valve, gold) -> None:
    """Stepped escutcheon + cylindrical valve body + stem collar, all
    projecting upward (+Z) from the deck surface (valve frame z=0)."""
    # Escutcheon base ring.
    valve.visual(
        Cylinder(radius=VALVE_ESC_R1, length=VALVE_ESC_T1),
        origin=Origin(xyz=(0.0, 0.0, VALVE_ESC_T1 / 2.0)),
        material=gold,
        name="escutcheon_base",
    )
    # Escutcheon step.
    valve.visual(
        Cylinder(radius=VALVE_ESC_R2, length=VALVE_ESC_T2),
        origin=Origin(xyz=(0.0, 0.0, VALVE_ESC_T1 + VALVE_ESC_T2 / 2.0)),
        material=gold,
        name="escutcheon_step",
    )
    # Valve body cylinder.
    body_base = VALVE_ESC_T1 + VALVE_ESC_T2
    valve.visual(
        Cylinder(radius=VALVE_BODY_R, length=VALVE_BODY_H),
        origin=Origin(xyz=(0.0, 0.0, body_base + VALVE_BODY_H / 2.0)),
        material=gold,
        name="valve_body",
    )
    # Stem collar (visible ring between valve body and handle).
    valve.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_H),
        origin=Origin(xyz=(0.0, 0.0, VALVE_TOP_Z + COLLAR_H / 2.0)),
        material=gold,
        name="stem_collar",
    )


def _add_handle_visuals(handle, hub_mesh, gold, cap_material) -> None:
    """Cross handle in the handle frame (joint frame at top of stem collar,
    axis +Z vertical): stem, knurled hub, horizontal cross spokes with
    sphere tips, and a hot/cold cap disk on top."""
    # Stem extends downward into valve body bore (seats through collar).
    handle.visual(
        Cylinder(radius=STEM_R, length=STEM_LEN),
        origin=Origin(xyz=(0.0, 0.0, -STEM_LEN / 2.0)),
        material=gold,
        name="stem",
    )
    # Hub with knurled band.
    handle.visual(hub_mesh, material=gold, name="hub")
    # Horizontal spoke rod along X (cylinder default axis is Z; rotate to X).
    handle.visual(
        Cylinder(radius=HANDLE_ROD_R, length=HANDLE_ROD_LEN),
        origin=Origin(
            xyz=(0.0, 0.0, HUB_H / 2.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=gold,
        name="x_spokes",
    )
    # Horizontal spoke rod along Y (rotate Z-axis cylinder to Y).
    handle.visual(
        Cylinder(radius=HANDLE_ROD_R, length=HANDLE_ROD_LEN),
        origin=Origin(
            xyz=(0.0, 0.0, HUB_H / 2.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=gold,
        name="y_spokes",
    )
    # Rounded spoke tips (4 tips in horizontal plane).
    half = HANDLE_ROD_LEN / 2.0
    for name, (dx, dy) in (
        ("tip_x_pos", (half, 0.0)),
        ("tip_x_neg", (-half, 0.0)),
        ("tip_y_pos", (0.0, half)),
        ("tip_y_neg", (0.0, -half)),
    ):
        handle.visual(
            Sphere(radius=HANDLE_ROD_R),
            origin=Origin(xyz=(dx, dy, HUB_H / 2.0)),
            material=gold,
            name=name,
        )
    # Hot or cold cap disk on top of hub.
    handle.visual(
        Cylinder(radius=CAP_R, length=CAP_T),
        origin=Origin(xyz=(0.0, 0.0, HUB_H + CAP_T / 2.0)),
        material=cap_material,
        name="cap_disk",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_gold_faucet")

    gold = model.material("polished_gold_brass", rgba=(0.85, 0.66, 0.20, 1.0))
    deck_white = model.material("deck_surface", rgba=(0.92, 0.92, 0.88, 1.0))
    hot_red = model.material("hot_indicator", rgba=(0.80, 0.12, 0.12, 1.0))
    cold_blue = model.material("cold_indicator", rgba=(0.12, 0.20, 0.78, 1.0))

    # --- deck plate (root, mounting substrate) ---
    deck = model.part("deck_plate")
    deck.visual(
        Box((DECK_W, DECK_D, DECK_T)),
        origin=Origin(xyz=(0.0, 0.0, DECK_T / 2.0)),
        material=deck_white,
        name="deck",
    )

    # --- central spout (fixed) ---
    spout = model.part("spout")
    spout.visual(
        mesh_from_cadquery(_build_spout_solid(), "spout"),
        material=gold,
        name="tube",
    )
    model.articulation(
        "deck_to_spout",
        ArticulationType.FIXED,
        parent=deck,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, DECK_TOP_Z)),
    )

    # --- valve assemblies (fixed) and cross handles (revolute, vertical) ---
    hub_mesh = mesh_from_cadquery(_build_hub_solid(), "handle_hub")
    for side, sx in (("left", -1.0), ("right", 1.0)):
        cap_mat = hot_red if side == "left" else cold_blue

        valve = model.part(f"{side}_valve")
        _add_valve_visuals(valve, gold)
        model.articulation(
            f"deck_to_{side}_valve",
            ArticulationType.FIXED,
            parent=deck,
            child=valve,
            origin=Origin(xyz=(sx * HANDLE_SPREAD_X, 0.0, DECK_TOP_Z)),
        )

        handle = model.part(f"{side}_handle")
        _add_handle_visuals(handle, hub_mesh, gold, cap_mat)
        model.articulation(
            f"{side}_handle_spindle",
            ArticulationType.REVOLUTE,
            parent=valve,
            child=handle,
            # Joint frame at top of stem collar; axis is vertical (+Z).
            origin=Origin(xyz=(0.0, 0.0, COLLAR_TOP_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=5.0, velocity=3.0, lower=-math.pi, upper=math.pi
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck_plate")
    spout = object_model.get_part("spout")
    left_valve = object_model.get_part("left_valve")
    right_valve = object_model.get_part("right_valve")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")
    left_joint = object_model.get_articulation("left_handle_spindle")
    right_joint = object_model.get_articulation("right_handle_spindle")

    # --- joint plan: two independent revolute handles, vertical axis, ±π ---
    for joint in (left_joint, right_joint):
        ctx.check(
            f"{joint.name}_revolute",
            str(joint.joint_type).lower().endswith("revolute"),
            f"type={joint.joint_type}",
        )
        ax = joint.axis
        ctx.check(
            f"{joint.name}_vertical_axis",
            abs(ax[0]) < 1e-9 and abs(ax[1]) < 1e-9 and abs(ax[2] - 1.0) < 1e-9,
            f"axis={ax}",
        )
        lim = joint.motion_limits
        ctx.check(
            f"{joint.name}_full_turn_range",
            lim is not None
            and abs(lim.lower + math.pi) < 1e-6
            and abs(lim.upper - math.pi) < 1e-6,
            f"limits=({lim.lower}, {lim.upper})",
        )

    # --- intentional embedding: handle stems seat into valve body bores ---
    ctx.allow_overlap(
        left_handle,
        left_valve,
        elem_a=left_handle.get_visual("stem"),
        elem_b=left_valve.get_visual("valve_body"),
        reason="valve stem is seated inside the valve body bore and turns with the handle",
    )
    ctx.allow_overlap(
        right_handle,
        right_valve,
        elem_a=right_handle.get_visual("stem"),
        elem_b=right_valve.get_visual("valve_body"),
        reason="valve stem is seated inside the valve body bore and turns with the handle",
    )
    # Stem also passes through the collar (intentional embedding).
    ctx.allow_overlap(
        left_handle,
        left_valve,
        elem_a=left_handle.get_visual("stem"),
        elem_b=left_valve.get_visual("stem_collar"),
        reason="stem passes through the collar ring on its way into the valve body",
    )
    ctx.allow_overlap(
        right_handle,
        right_valve,
        elem_a=right_handle.get_visual("stem"),
        elem_b=right_valve.get_visual("stem_collar"),
        reason="stem passes through the collar ring on its way into the valve body",
    )

    # --- spout geometry: hollow bore, taller arc ---
    ctx.check(
        "spout_tube_is_hollow",
        0.0 < SPOUT_SOLID_VOLUME < 0.97 * SPOUT_UNBORED_VOLUME,
        f"solid={SPOUT_SOLID_VOLUME:.3e} m^3 vs unbored={SPOUT_UNBORED_VOLUME:.3e} m^3",
    )
    spout_aabb = ctx.part_world_aabb(spout)
    assert spout_aabb is not None
    (sx0, sy0, sz0), (sx1, sy1, sz1) = spout_aabb
    ctx.check(
        "spout_peak_taller_than_0p22",
        sz1 >= DECK_TOP_Z + 0.22,
        f"spout zmax={sz1:.3f}, deck_top={DECK_TOP_Z}",
    )
    ctx.check(
        "spout_projects_forward_at_least_0p12",
        sy0 < -0.12,
        f"spout forward extent sy0={sy0:.3f}",
    )
    ctx.check(
        "spout_height_range_shows_tall_arc",
        (sz1 - sz0) > 0.20,
        f"spout z range=({sz0:.3f}, {sz1:.3f}), span={sz1 - sz0:.3f}",
    )

    # --- valve placement: flanking spout at wider spread ---
    lv = ctx.part_world_position(left_valve)
    rv = ctx.part_world_position(right_valve)
    assert lv is not None and rv is not None
    ctx.check(
        "valves_wider_spread_symmetrical",
        abs(lv[0] + HANDLE_SPREAD_X) < 1e-4
        and abs(rv[0] - HANDLE_SPREAD_X) < 1e-4,
        f"left_x={lv[0]:.4f}, right_x={rv[0]:.4f}",
    )
    ctx.check(
        "valves_spread_farther_than_original",
        abs(rv[0] - lv[0]) > 0.24,
        f"valve spacing={rv[0] - lv[0]:.3f} (should exceed 0.24 m)",
    )

    # --- stem collars exist on both valves ---
    for valve_name in ("left_valve", "right_valve"):
        v = object_model.get_part(valve_name)
        collar = v.get_visual("stem_collar")
        ctx.check(
            f"{valve_name}_has_stem_collar",
            collar is not None,
            f"stem_collar visual not found on {valve_name}",
        )

    # --- hot and cold cap disks exist as geometry ---
    left_cap = left_handle.get_visual("cap_disk")
    right_cap = right_handle.get_visual("cap_disk")
    ctx.check(
        "left_handle_has_hot_cap_disk",
        left_cap is not None,
        "cap_disk visual not found on left_handle",
    )
    ctx.check(
        "right_handle_has_cold_cap_disk",
        right_cap is not None,
        "cap_disk visual not found on right_handle",
    )

    # --- cross handle size: about 0.10 m tip to tip (horizontal plane) ---
    lh_aabb = ctx.part_world_aabb(left_handle)
    assert lh_aabb is not None
    (hx0, hy0, hz0), (hx1, hy1, hz1) = lh_aabb
    ctx.check(
        "cross_handle_about_0p10_tip_to_tip",
        0.095 <= (hx1 - hx0) <= 0.115 and 0.095 <= (hy1 - hy0) <= 0.115,
        f"handle extents x={hx1 - hx0:.3f}, y={hy1 - hy0:.3f}",
    )

    # --- handle mounted on valve (not floating) ---
    ctx.expect_overlap(left_handle, left_valve, axes="xy", min_overlap=0.005)
    ctx.expect_overlap(right_handle, right_valve, axes="xy", min_overlap=0.005)

    # --- overall width about 0.38 m across handle tips ---
    rh_aabb = ctx.part_world_aabb(right_handle)
    assert rh_aabb is not None
    total_w = rh_aabb[1][0] - lh_aabb[0][0]
    ctx.check(
        "overall_widespread_width",
        0.34 <= total_w <= 0.42,
        f"handle-tip to handle-tip width={total_w:.3f}",
    )

    # --- rotation proof: spokes rotate in horizontal plane around Z ---
    with ctx.pose({left_joint: math.pi / 4.0}):
        rot_aabb = ctx.part_world_aabb(left_handle)
        assert rot_aabb is not None
        rot_x = rot_aabb[1][0] - rot_aabb[0][0]
        ctx.check(
            "left_handle_spokes_rotate_off_axis",
            rot_x < 0.095,
            f"x extent at q=45deg is {rot_x:.3f} (cross at 45 deg shrinks from ~0.105)",
        )
        cen = ctx.part_world_position(left_handle)
        assert cen is not None
        ctx.check(
            "left_handle_stays_on_valve_axis_while_rotating",
            abs(cen[0] + HANDLE_SPREAD_X) < 0.005,
            f"handle origin x={cen[0]:.4f}",
        )

    with ctx.pose({right_joint: -math.pi / 2.0}):
        ctx.expect_overlap(right_handle, right_valve, axes="xy", min_overlap=0.005)
        ctx.expect_gap(right_handle, spout, axis="x", min_gap=0.01)

    # --- deck plate grounded ---
    deck_aabb = ctx.part_world_aabb(deck)
    assert deck_aabb is not None
    ctx.check(
        "deck_plate_grounded",
        abs(deck_aabb[0][2]) < 1e-6,
        f"deck zmin={deck_aabb[0][2]:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
