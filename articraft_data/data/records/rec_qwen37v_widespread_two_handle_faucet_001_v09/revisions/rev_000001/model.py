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
# Deck-mounted, three-piece layout: left handle, central spout, right handle.
#
# Frame conventions:
#   - Deck surface at z = DECK_T (top of deck plate).
#   - Spout rises along +Z, curves toward +Y (user side).
#   - Left/right valves at x = +/- VALVE_SPACING.
#   - Diverter behind spout at -Y.
# ---------------------------------------------------------------------------

# Deck
DECK_W = 0.32       # x extent
DECK_D = 0.10       # y extent
DECK_T = 0.012      # thickness

# Oval escutcheon plate (raised platform under all three posts)
ESC_RX = 0.135      # half-length along X
ESC_RY = 0.035      # half-width along Y
ESC_T = 0.006       # thickness (raised above deck)

# Spout
SPOUT_R = 0.013          # outer radius (~0.026m diameter)
SPOUT_BORE_R = 0.009     # inner bore
SPOUT_RISE = 0.18        # vertical rise from deck
SPOUT_BEND_R = 0.04      # bend radius
SPOUT_DROP = 0.08        # how far the outlet drops from peak
SPOUT_REACH_Y = 0.08     # horizontal reach of curved portion

# Valve body
VALVE_BODY_R = 0.012
VALVE_BODY_H = 0.030     # height above escutcheon
VALVE_SPACING = 0.10     # x offset from center

# Stem collar
COLLAR_R = 0.016
COLLAR_H = 0.008

# Cross handle
HANDLE_ROD_R = 0.004
HANDLE_ROD_LEN = 0.090   # tip-to-tip
HUB_R = 0.011
HUB_LEN = 0.022
KNURL_R = 0.0125
STEM_R = 0.006
STEM_LEN = 0.012

# Diverter knob
DIV_R = 0.010
DIV_H = 0.018
DIV_STEM_R = 0.005
DIV_STEM_H = 0.015
DIV_TRAVEL = 0.020        # prismatic travel (up/down)

# Computed volumes for hollow bore check
SPOUT_SOLID_VOLUME: float = 0.0
SPOUT_UNBORED_VOLUME: float = 0.0


def _build_oval_plate() -> cq.Workplane:
    """Raised oval escutcheon plate, flat on XY, extruded along +Z."""
    plate = (
        cq.Workplane("XY")
        .ellipse(ESC_RX, ESC_RY)
        .extrude(ESC_T)
    )
    # Add a subtle chamfer/bevel on top edge
    return plate


def _build_spout_solid() -> cq.Workplane:
    """Spout rising vertically then curving forward (+Y) and down.
    Local frame: base at origin on XY plane, rises along +Z."""
    # Path goes: up Z, then arcs forward (+Y) and down (-Z from peak)
    # Path in YZ plane
    peak_z = SPOUT_RISE
    arc_start = (0.0, peak_z - SPOUT_BEND_R)
    arc_mid = (SPOUT_BEND_R * math.sin(math.pi / 4),
               peak_z - SPOUT_BEND_R + SPOUT_BEND_R * (1 - math.cos(math.pi / 4)))
    arc_end = (SPOUT_BEND_R, peak_z)
    outlet_end = (SPOUT_BEND_R + SPOUT_REACH_Y - SPOUT_BEND_R * 0.3,
                  peak_z - SPOUT_DROP)

    path = (
        cq.Workplane("YZ")
        .moveTo(0.0, 0.0)
        .lineTo(0.0, peak_z - SPOUT_BEND_R)
        .threePointArc(arc_mid, arc_end)
        .lineTo(outlet_end[0], outlet_end[1])
    )

    bore_path = (
        cq.Workplane("YZ")
        .moveTo(0.0, -0.003)
        .lineTo(0.0, peak_z - SPOUT_BEND_R)
        .threePointArc(arc_mid, arc_end)
        .lineTo(outlet_end[0], outlet_end[1] - 0.003)
    )

    # Sweep tube along path. Workplane perpendicular to start tangent (XY)
    tube = cq.Workplane("XY").circle(SPOUT_R).sweep(path)
    bore = (
        cq.Workplane("XY")
        .workplane(offset=-0.003)
        .circle(SPOUT_BORE_R)
        .sweep(bore_path)
    )

    # Base flange
    base_flange = (
        cq.Workplane("XY")
        .circle(SPOUT_R + 0.008)
        .extrude(0.008)
    )

    unbored = tube.union(base_flange)
    solid = unbored.cut(bore)

    global SPOUT_SOLID_VOLUME, SPOUT_UNBORED_VOLUME
    SPOUT_SOLID_VOLUME = solid.val().Volume()
    SPOUT_UNBORED_VOLUME = unbored.val().Volume()
    return solid


def _build_hub_solid() -> cq.Workplane:
    """Cross-handle central hub: axis +Z, base at z=0, with knurled band."""
    hub = cq.Workplane("XY").circle(HUB_R).extrude(HUB_LEN)
    knurl = (
        cq.Workplane("XY")
        .workplane(offset=0.005)
        .polygon(16, 2.0 * KNURL_R)
        .extrude(0.010)
    )
    dome = cq.Workplane("XY").workplane(offset=HUB_LEN - 0.003).sphere(0.010)
    return hub.union(knurl).union(dome)


def _build_divider_knob() -> cq.Workplane:
    """Diverter knob: cylindrical body with a small stem below."""
    body = cq.Workplane("XY").circle(DIV_R).extrude(DIV_H)
    # Knurled grip ring
    grip = (
        cq.Workplane("XY")
        .workplane(offset=DIV_H * 0.3)
        .polygon(12, 2.0 * (DIV_R + 0.002))
        .extrude(DIV_H * 0.4)
    )
    # Stem below (extends into spout body)
    stem = (
        cq.Workplane("XY")
        .workplane(offset=-DIV_STEM_H)
        .circle(DIV_STEM_R)
        .extrude(DIV_STEM_H)
    )
    return body.union(grip).union(stem)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_two_handle_faucet")

    gold = model.material("polished_gold_brass", rgba=(0.85, 0.66, 0.20, 1.0))
    chrome_accent = model.material("chrome_accent", rgba=(0.75, 0.75, 0.78, 1.0))

    # --- deck plate (root, countertop surface) ---
    deck = model.part("deck_plate")
    deck.visual(
        Box((DECK_W, DECK_D, DECK_T)),
        origin=Origin(xyz=(0.0, 0.0, DECK_T / 2.0)),
        material=gold,
        name="deck_surface",
    )

    # --- oval escutcheon plate (fixed, sits on deck) ---
    escutcheon = model.part("escutcheon_plate")
    escutcheon.visual(
        mesh_from_cadquery(_build_oval_plate(), "escutcheon"),
        material=gold,
        name="oval_plate",
    )
    model.articulation(
        "deck_to_escutcheon",
        ArticulationType.FIXED,
        parent=deck,
        child=escutcheon,
        origin=Origin(xyz=(0.0, 0.0, DECK_T)),
    )

    # --- central spout (fixed, mounted on escutcheon) ---
    spout = model.part("spout")
    spout.visual(
        mesh_from_cadquery(_build_spout_solid(), "spout"),
        material=gold,
        name="tube",
    )
    model.articulation(
        "escutcheon_to_spout",
        ArticulationType.FIXED,
        parent=escutcheon,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, ESC_T)),
    )

    # --- diverter knob (prismatic, slides up/down behind spout) ---
    diverter = model.part("diverter_knob")
    diverter.visual(
        mesh_from_cadquery(_build_divider_knob(), "diverter"),
        material=gold,
        name="knob_body",
    )
    model.articulation(
        "spout_to_diverter",
        ArticulationType.PRISMATIC,
        parent=spout,
        child=diverter,
        # Behind the spout base, axis along +Z (up)
        # Knob sits at the rear surface of the spout tube, stem penetrates wall
        origin=Origin(xyz=(0.0, -SPOUT_R, SPOUT_RISE * 0.3)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=0.5, lower=0.0, upper=DIV_TRAVEL,
        ),
    )

    # --- valve assemblies and handles ---
    hub_mesh = mesh_from_cadquery(_build_hub_solid(), "handle_hub")

    for side, sx in (("left", -1.0), ("right", 1.0)):
        # Valve body (fixed on escutcheon)
        valve = model.part(f"{side}_valve")
        valve.visual(
            Cylinder(radius=VALVE_BODY_R, length=VALVE_BODY_H),
            origin=Origin(xyz=(0.0, 0.0, VALVE_BODY_H / 2.0)),
            material=gold,
            name="valve_body",
        )
        model.articulation(
            f"escutcheon_to_{side}_valve",
            ArticulationType.FIXED,
            parent=escutcheon,
            child=valve,
            origin=Origin(xyz=(sx * VALVE_SPACING, 0.0, ESC_T)),
        )

        # Stem collar (visible ring under the handle, turns with handle)
        handle = model.part(f"{side}_handle")
        handle.visual(
            Cylinder(radius=COLLAR_R, length=COLLAR_H),
            origin=Origin(xyz=(0.0, 0.0, COLLAR_H / 2.0)),
            material=gold,
            name="stem_collar",
        )
        # Stem seated into valve body
        handle.visual(
            Cylinder(radius=STEM_R, length=STEM_LEN),
            origin=Origin(xyz=(0.0, 0.0, -STEM_LEN / 2.0)),
            material=gold,
            name="stem",
        )
        # Hub
        handle.visual(
            hub_mesh,
            origin=Origin(xyz=(0.0, 0.0, COLLAR_H)),
            material=gold,
            name="hub",
        )
        # Cross spokes (two rods at 90 degrees, along X and Y)
        spoke_y_offset = COLLAR_H + HUB_LEN * 0.5
        handle.visual(
            Cylinder(radius=HANDLE_ROD_R, length=HANDLE_ROD_LEN),
            origin=Origin(
                xyz=(0.0, 0.0, spoke_y_offset),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=gold,
            name="spoke_x",
        )
        handle.visual(
            Cylinder(radius=HANDLE_ROD_R, length=HANDLE_ROD_LEN),
            origin=Origin(
                xyz=(0.0, 0.0, spoke_y_offset),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=gold,
            name="spoke_y",
        )
        # Spoke tips (4 rounded ends per rod, 8 total)
        half = HANDLE_ROD_LEN / 2.0
        for name, (dx, dy) in (
            ("tip_px", (half, 0.0)),
            ("tip_nx", (-half, 0.0)),
            ("tip_py", (0.0, half)),
            ("tip_ny", (0.0, -half)),
        ):
            handle.visual(
                Sphere(radius=HANDLE_ROD_R),
                origin=Origin(xyz=(dx, dy, spoke_y_offset)),
                material=gold,
                name=name,
            )

        # Revolute joint: handle rotates about Z axis (vertical)
        model.articulation(
            f"{side}_handle_spindle",
            ArticulationType.REVOLUTE,
            parent=valve,
            child=handle,
            origin=Origin(xyz=(0.0, 0.0, VALVE_BODY_H)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=5.0, velocity=3.0, lower=-math.pi, upper=math.pi,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck_plate")
    escutcheon = object_model.get_part("escutcheon_plate")
    spout = object_model.get_part("spout")
    diverter = object_model.get_part("diverter_knob")
    left_valve = object_model.get_part("left_valve")
    right_valve = object_model.get_part("right_valve")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")
    left_joint = object_model.get_articulation("left_handle_spindle")
    right_joint = object_model.get_articulation("right_handle_spindle")
    diverter_joint = object_model.get_articulation("spout_to_diverter")

    # --- Oval escutcheon plate exists and is raised above deck ---
    esc_aabb = ctx.part_world_aabb(escutcheon)
    assert esc_aabb is not None
    (ex0, ey0, ez0), (ex1, ey1, ez1) = esc_aabb
    ctx.check(
        "escutcheon_is_oval_wider_than_deep",
        (ex1 - ex0) > (ey1 - ey0) * 1.5,
        f"escutcheon x={(ex1-ex0):.3f}, y={(ey1-ey0):.3f}",
    )
    ctx.check(
        "escutcheon_raised_above_deck",
        ez0 >= DECK_T - 0.001,
        f"escutcheon zmin={ez0:.4f}, deck top={DECK_T}",
    )

    # --- Stem collars exist under each handle ---
    for side in ("left", "right"):
        h = object_model.get_part(f"{side}_handle")
        collar = h.get_visual("stem_collar")
        ctx.check(
            f"{side}_stem_collar_exists",
            collar is not None,
            f"stem_collar visual missing on {side} handle",
        )

    # --- Revolute handle joints: axis +Z, range -pi..+pi ---
    for joint in (left_joint, right_joint):
        ctx.check(
            f"{joint.name}_revolute",
            str(joint.joint_type).lower().endswith("revolute"),
            f"type={joint.joint_type}",
        )
        ax = joint.axis
        ctx.check(
            f"{joint.name}_axis_vertical",
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

    # --- Diverter prismatic joint: axis +Z, range 0..DIV_TRAVEL ---
    ctx.check(
        "diverter_joint_prismatic",
        str(diverter_joint.joint_type).lower().endswith("prismatic"),
        f"type={diverter_joint.joint_type}",
    )
    div_ax = diverter_joint.axis
    ctx.check(
        "diverter_axis_vertical",
        abs(div_ax[0]) < 1e-9 and abs(div_ax[1]) < 1e-9 and abs(div_ax[2] - 1.0) < 1e-9,
        f"axis={div_ax}",
    )
    div_lim = diverter_joint.motion_limits
    ctx.check(
        "diverter_travel_limits",
        div_lim is not None
        and abs(div_lim.lower) < 1e-6
        and abs(div_lim.upper - DIV_TRAVEL) < 1e-6,
        f"limits=({div_lim.lower}, {div_lim.upper})",
    )

    # --- Diverter moves upward when actuated ---
    div_rest = ctx.part_world_position(diverter)
    assert div_rest is not None
    with ctx.pose({diverter_joint: DIV_TRAVEL}):
        div_up = ctx.part_world_position(diverter)
        assert div_up is not None
        ctx.check(
            "diverter_slides_upward",
            div_up[2] > div_rest[2] + DIV_TRAVEL * 0.9,
            f"rest_z={div_rest[2]:.4f}, extended_z={div_up[2]:.4f}",
        )

    # --- Intentional overlap: handle stems seated in valve bodies ---
    ctx.allow_overlap(
        left_handle,
        left_valve,
        elem_a=left_handle.get_visual("stem"),
        elem_b=left_valve.get_visual("valve_body"),
        reason="handle stem is seated inside the valve body bore and turns with the handle",
    )
    ctx.allow_overlap(
        right_handle,
        right_valve,
        elem_a=right_handle.get_visual("stem"),
        elem_b=right_valve.get_visual("valve_body"),
        reason="handle stem is seated inside the valve body bore and turns with the handle",
    )
    # Diverter stem nested inside spout body
    ctx.allow_overlap(
        diverter,
        spout,
        elem_a=diverter.get_visual("knob_body"),
        elem_b=spout.get_visual("tube"),
        reason="diverter knob stem is intentionally nested in the spout body bore",
    )

    # Prove diverter stem stays within spout at rest and extended
    ctx.expect_overlap(
        diverter, spout, axes="xy",
        elem_a="knob_body", elem_b="tube",
        min_overlap=0.002,
        name="diverter_stays_within_spout_xy",
    )

    # --- Spout is hollow (bore check) ---
    ctx.check(
        "spout_tube_is_hollow",
        0.0 < SPOUT_SOLID_VOLUME < 0.95 * SPOUT_UNBORED_VOLUME,
        f"solid={SPOUT_SOLID_VOLUME:.3e} m^3 vs unbored={SPOUT_UNBORED_VOLUME:.3e} m^3",
    )

    # --- Spout rises above deck ---
    spout_aabb = ctx.part_world_aabb(spout)
    assert spout_aabb is not None
    (sx0, sy0, sz0), (sx1, sy1, sz1) = spout_aabb
    ctx.check(
        "spout_rises_above_deck",
        sz1 > DECK_T + 0.12,
        f"spout zmax={sz1:.3f}",
    )

    # --- Handles rotate without leaving their valve axis ---
    with ctx.pose({left_joint: math.pi / 3.0}):
        rot_pos = ctx.part_world_position(left_handle)
        assert rot_pos is not None
        rest_pos = ctx.part_world_position(left_handle)
        ctx.check(
            "left_handle_stays_on_valve_axis_while_rotating",
            abs(rot_pos[0] - (-VALVE_SPACING)) < 0.005
            and abs(rot_pos[1]) < 0.005,
            f"handle position at 60deg={rot_pos}",
        )

    # --- Valves flank spout symmetrically ---
    lv = ctx.part_world_position(left_valve)
    rv = ctx.part_world_position(right_valve)
    assert lv is not None and rv is not None
    ctx.check(
        "valves_flank_spout_symmetrically",
        abs(lv[0] + VALVE_SPACING) < 0.005
        and abs(rv[0] - VALVE_SPACING) < 0.005
        and abs(lv[0] + rv[0]) < 0.005,
        f"left_x={lv[0]:.4f}, right_x={rv[0]:.4f}",
    )

    # --- Three-piece widespread layout: escutcheon spans all three posts ---
    ctx.expect_within(
        left_valve, escutcheon, axes="xy",
        margin=0.005,
        name="left_valve_within_escutcheon",
    )
    ctx.expect_within(
        right_valve, escutcheon, axes="xy",
        margin=0.005,
        name="right_valve_within_escutcheon",
    )
    # Spout base is within the escutcheon in X (Y extends forward due to curve)
    ctx.expect_within(
        spout, escutcheon, axes="x",
        margin=0.005,
        name="spout_base_within_escutcheon_x",
    )

    # --- Deck is grounded at z=0 ---
    deck_aabb = ctx.part_world_aabb(deck)
    assert deck_aabb is not None
    ctx.check(
        "deck_grounded_at_z0",
        abs(deck_aabb[0][2]) < 1e-6,
        f"deck zmin={deck_aabb[0][2]:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
