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
# Deck-mounted on a horizontal surface.  Three units sit on raised oval
# escutcheon plates: left handle, central arched spout, right handle.
# Cross handles rotate about short vertical axles.  Hot (red) and cold
# (blue) indicator caps sit on top of each handle hub.
#
# Frame conventions:
#   - Z is up; the deck top surface is at z = DECK_T.
#   - +Y points toward the user / basin (spout projects +Y).
#   - Viewer-left is -X (left handle at x = -VALVE_X).
# ---------------------------------------------------------------------------

# Layout
VALVE_X = 0.10  # handle/valve centres at x = +/- 0.10

# Deck panel (countertop mounting substrate)
DECK_W, DECK_D, DECK_T = 0.36, 0.14, 0.012

# Oval escutcheon plates (shared for all three posts)
ESC_RX, ESC_RY, ESC_T = 0.033, 0.023, 0.008

# Z-height stations (relative to each part origin at deck bottom z=0)
DECK_TOP = DECK_T                          # 0.012
ESC_BOT = DECK_TOP                         # 0.012
ESC_TOP = ESC_BOT + ESC_T                  # 0.020
VB_BOT = ESC_TOP                           # 0.020
VB_H = 0.040
VB_TOP = VB_BOT + VB_H                     # 0.060
SC_BOT = VB_TOP                            # 0.060
SC_H = 0.008
SC_TOP = SC_BOT + SC_H                     # 0.068

# Valve body
VB_R = 0.014

# Stem collar (visible ring under the handle)
SC_R = 0.017

# Cross handle (rotates about vertical Z)
HUB_R = 0.013
HUB_H = 0.020
KNURL_R = 0.0145
ARM_LEN = 0.048        # half-length from centre (tip-to-tip ~ 0.10 m with tips)
ARM_R = 0.004
ARM_Z = HUB_H * 0.5    # arm plane at mid-hub in handle frame
STEM_R = 0.007
STEM_LEN = 0.012        # stem extends -Z into collar / valve body

# Hot / cold indicator caps
CAP_R = 0.007
CAP_T = 0.003

# Spout tube
SPOUT_TUBE_R = 0.015
SPOUT_BORE_R = 0.011

# Computed for hollow-bore verification
SPOUT_SOLID_VOLUME: float = 0.0
SPOUT_UNBORED_VOLUME: float = 0.0


# ---- CadQuery geometry builders -------------------------------------------

def _build_oval_plate() -> cq.Workplane:
    """Raised oval escutcheon plate, XY footprint, extruded along +Z."""
    return cq.Workplane("XY").ellipse(ESC_RX, ESC_RY).extrude(ESC_T)


def _build_hub_solid() -> cq.Workplane:
    """Handle hub: axis +Z, base at z=0, knurled faceted middle band and
    a small chamfer ring at the top edge."""
    hub = cq.Workplane("XY").circle(HUB_R).extrude(HUB_H)
    knurl = (
        cq.Workplane("XY")
        .workplane(offset=0.005)
        .polygon(16, 2.0 * KNURL_R)
        .extrude(0.010)
    )
    # Top rim ring for visual detail (no dome so cap sits flat on hub top).
    rim = (
        cq.Workplane("XY")
        .workplane(offset=HUB_H - 0.003)
        .circle(HUB_R + 0.002)
        .extrude(0.003)
    )
    return hub.union(knurl).union(rim)


def _build_spout_solid() -> cq.Workplane:
    """Spout in local frame: origin at base, rises along +Z, arches forward
    (+Y), then drops to an open outlet.  Hollow bore visible at the outlet."""
    # Sweep path in the YZ workplane (first coord → Y, second → Z).
    path = (
        cq.Workplane("YZ")
        .moveTo(0.0, 0.0)
        .lineTo(0.0, 0.10)
        .threePointArc((0.04, 0.18), (0.09, 0.10))
        .lineTo(0.09, 0.06)
    )
    bore_path = (
        cq.Workplane("YZ")
        .moveTo(0.0, -0.004)
        .lineTo(0.0, 0.10)
        .threePointArc((0.04, 0.18), (0.09, 0.10))
        .lineTo(0.09, 0.06 - 0.004)
    )

    # Profile on XY (normal +Z), perpendicular to the path start tangent (+Z).
    tube = cq.Workplane("XY").circle(SPOUT_TUBE_R).sweep(path)
    bore = (
        cq.Workplane("XY")
        .workplane(offset=-0.004)
        .circle(SPOUT_BORE_R)
        .sweep(bore_path)
    )

    # Decorative collar where the spout meets the escutcheon.
    base_collar = (
        cq.Workplane("XY").circle(SPOUT_TUBE_R + 0.004).extrude(0.012)
    )

    unbored = tube.union(base_collar)
    solid = unbored.cut(bore)

    global SPOUT_SOLID_VOLUME, SPOUT_UNBORED_VOLUME
    SPOUT_SOLID_VOLUME = solid.val().Volume()
    SPOUT_UNBORED_VOLUME = unbored.val().Volume()
    return solid


# ---- Object model ---------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_two_handle_faucet")

    gold = model.material("polished_gold_brass", rgba=(0.85, 0.66, 0.20, 1.0))
    deck_mat = model.material("counter_deck", rgba=(0.88, 0.86, 0.82, 1.0))
    hot_red = model.material("hot_indicator", rgba=(0.80, 0.15, 0.15, 1.0))
    cold_blue = model.material("cold_indicator", rgba=(0.15, 0.25, 0.75, 1.0))

    # --- shared meshes ---
    plate_mesh = mesh_from_cadquery(_build_oval_plate(), "oval_escutcheon")
    hub_mesh = mesh_from_cadquery(_build_hub_solid(), "handle_hub")
    spout_mesh = mesh_from_cadquery(_build_spout_solid(), "spout_tube")

    # --- deck panel (root) ---
    deck = model.part("deck_panel")
    deck.visual(
        Box((DECK_W, DECK_D, DECK_T)),
        origin=Origin(xyz=(0.0, 0.0, DECK_T / 2.0)),
        material=deck_mat,
        name="deck",
    )

    # --- central spout (fixed) ---
    spout = model.part("spout")
    spout.visual(
        plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, ESC_BOT)),
        material=gold,
        name="escutcheon",
    )
    spout.visual(
        spout_mesh,
        origin=Origin(xyz=(0.0, 0.0, ESC_TOP)),
        material=gold,
        name="tube",
    )
    model.articulation(
        "deck_to_spout",
        ArticulationType.FIXED,
        parent=deck,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # --- valve assemblies and cross handles ---
    for side, sx in (("left", -1.0), ("right", 1.0)):
        valve = model.part(f"{side}_valve")
        # Raised oval escutcheon plate
        valve.visual(
            plate_mesh,
            origin=Origin(xyz=(0.0, 0.0, ESC_BOT)),
            material=gold,
            name="escutcheon",
        )
        # Cylindrical valve body
        valve.visual(
            Cylinder(radius=VB_R, length=VB_H),
            origin=Origin(xyz=(0.0, 0.0, VB_BOT + VB_H / 2.0)),
            material=gold,
            name="valve_body",
        )
        # Visible stem collar
        valve.visual(
            Cylinder(radius=SC_R, length=SC_H),
            origin=Origin(xyz=(0.0, 0.0, SC_BOT + SC_H / 2.0)),
            material=gold,
            name="stem_collar",
        )
        model.articulation(
            f"deck_to_{side}_valve",
            ArticulationType.FIXED,
            parent=deck,
            child=valve,
            origin=Origin(xyz=(sx * VALVE_X, 0.0, 0.0)),
        )

        # --- cross handle (revolute about vertical Z) ---
        handle = model.part(f"{side}_handle")
        # Stem (seats down into the collar / valve bore)
        handle.visual(
            Cylinder(radius=STEM_R, length=STEM_LEN),
            origin=Origin(xyz=(0.0, 0.0, -STEM_LEN / 2.0)),
            material=gold,
            name="stem",
        )
        # Hub with knurl detail (CadQuery mesh)
        handle.visual(hub_mesh, material=gold, name="hub")
        # Arm pair along X (Cylinder default axis Z → rotate to X)
        handle.visual(
            Cylinder(radius=ARM_R, length=2.0 * ARM_LEN),
            origin=Origin(xyz=(0.0, 0.0, ARM_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=gold,
            name="arm_x",
        )
        # Arm pair along Y (Cylinder default axis Z → rotate to Y)
        handle.visual(
            Cylinder(radius=ARM_R, length=2.0 * ARM_LEN),
            origin=Origin(xyz=(0.0, 0.0, ARM_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=gold,
            name="arm_y",
        )
        # Rounded spoke tips
        for tip_name, (dx, dy) in (
            ("tip_px", (ARM_LEN, 0.0)),
            ("tip_nx", (-ARM_LEN, 0.0)),
            ("tip_py", (0.0, ARM_LEN)),
            ("tip_ny", (0.0, -ARM_LEN)),
        ):
            handle.visual(
                Sphere(radius=ARM_R),
                origin=Origin(xyz=(dx, dy, ARM_Z)),
                material=gold,
                name=tip_name,
            )
        # Hot / cold indicator cap disk
        cap_mat = hot_red if side == "left" else cold_blue
        handle.visual(
            Cylinder(radius=CAP_R, length=CAP_T),
            origin=Origin(xyz=(0.0, 0.0, HUB_H + CAP_T / 2.0)),
            material=cap_mat,
            name="indicator_cap",
        )

        model.articulation(
            f"{side}_handle_spindle",
            ArticulationType.REVOLUTE,
            parent=valve,
            child=handle,
            # Joint frame at the top of the stem collar; axis vertical.
            origin=Origin(xyz=(0.0, 0.0, SC_TOP)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=5.0,
                velocity=3.0,
                lower=-math.pi,
                upper=math.pi,
            ),
        )

    return model


# ---- Tests ----------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck_panel")
    spout = object_model.get_part("spout")
    left_valve = object_model.get_part("left_valve")
    right_valve = object_model.get_part("right_valve")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")
    left_joint = object_model.get_articulation("left_handle_spindle")
    right_joint = object_model.get_articulation("right_handle_spindle")

    # --- joints: two independent revolute, vertical axis, full-turn range ---
    for jnt in (left_joint, right_joint):
        ctx.check(
            f"{jnt.name}_is_revolute",
            str(jnt.joint_type).lower().endswith("revolute"),
            f"type={jnt.joint_type}",
        )
        ax = jnt.axis
        ctx.check(
            f"{jnt.name}_vertical_axis",
            abs(ax[0]) < 1e-9 and abs(ax[1]) < 1e-9 and abs(ax[2] - 1.0) < 1e-9,
            f"axis={ax}",
        )
        lim = jnt.motion_limits
        ctx.check(
            f"{jnt.name}_full_turn_range",
            lim is not None
            and abs(lim.lower + math.pi) < 1e-6
            and abs(lim.upper - math.pi) < 1e-6,
            f"limits=({lim.lower}, {lim.upper})",
        )

    # --- oval escutcheon plates: rx > ry (oval, not round) ---
    for valve_part, side_name in ((left_valve, "left"), (right_valve, "right")):
        esc_aabb = ctx.part_element_world_aabb(valve_part, elem="escutcheon")
        assert esc_aabb is not None
        (ex0, ey0, _), (ex1, ey1, _) = esc_aabb
        dx = ex1 - ex0
        dy = ey1 - ey0
        ctx.check(
            f"{side_name}_escutcheon_is_oval",
            dx > dy * 1.15,
            f"dx={dx:.4f}, dy={dy:.4f} (rx/ry ratio too small)",
        )
    # Spout escutcheon also oval
    sp_esc = ctx.part_element_world_aabb(spout, elem="escutcheon")
    assert sp_esc is not None
    ctx.check(
        "spout_escutcheon_is_oval",
        (sp_esc[1][0] - sp_esc[0][0]) > (sp_esc[1][1] - sp_esc[0][1]) * 1.15,
    )

    # --- stem collars exist as named visuals on each valve ---
    for valve_part, side_name in ((left_valve, "left"), (right_valve, "right")):
        collar = valve_part.get_visual("stem_collar")
        ctx.check(
            f"{side_name}_stem_collar_exists",
            collar is not None,
            "stem_collar visual missing",
        )

    # --- hot / cold indicator caps exist as separate geometry ---
    lh_cap = left_handle.get_visual("indicator_cap")
    rh_cap = right_handle.get_visual("indicator_cap")
    ctx.check("left_handle_has_hot_cap", lh_cap is not None)
    ctx.check("right_handle_has_cold_cap", rh_cap is not None)
    # Caps have different materials (hot=red, cold=blue)
    if lh_cap is not None and rh_cap is not None:
        ctx.check(
            "caps_have_distinct_materials",
            lh_cap.material != rh_cap.material,
            f"left_mat={lh_cap.material}, right_mat={rh_cap.material}",
        )

    # --- spout is hollow ---
    ctx.check(
        "spout_tube_is_hollow",
        0.0 < SPOUT_SOLID_VOLUME < 0.97 * SPOUT_UNBORED_VOLUME,
        f"solid={SPOUT_SOLID_VOLUME:.3e} vs unbored={SPOUT_UNBORED_VOLUME:.3e}",
    )

    # --- spout rises above deck and arches forward ---
    sp_aabb = ctx.part_world_aabb(spout)
    assert sp_aabb is not None
    (_, _, sz0), (_, sy1, sz1) = sp_aabb
    ctx.check(
        "spout_rises_above_deck",
        sz1 > DECK_TOP + 0.12,
        f"spout peak z={sz1:.3f}",
    )
    ctx.check(
        "spout_arches_forward",
        sy1 > 0.06,
        f"spout max y={sy1:.3f}",
    )

    # --- valve placement: flanking the spout at x = +/- VALVE_X ---
    lv = ctx.part_world_position(left_valve)
    rv = ctx.part_world_position(right_valve)
    assert lv is not None and rv is not None
    ctx.check(
        "valves_flank_spout_symmetrically",
        abs(lv[0] + VALVE_X) < 1e-6 and abs(rv[0] - VALVE_X) < 1e-6,
        f"left_x={lv[0]}, right_x={rv[0]}",
    )

    # --- handle size: about 0.10 m tip-to-tip ---
    lh_aabb = ctx.part_world_aabb(left_handle)
    assert lh_aabb is not None
    (hx0, hy0, hz0), (hx1, hy1, hz1) = lh_aabb
    ctx.check(
        "cross_handle_about_0p10_tip_to_tip",
        0.090 <= (hx1 - hx0) <= 0.115 and 0.090 <= (hy1 - hy0) <= 0.115,
        f"handle x={hx1 - hx0:.3f}, y={hy1 - hy0:.3f}",
    )

    # --- handle stem seats into valve body and passes through stem collar ---
    for hdl, vlv, side_name in (
        (left_handle, left_valve, "left"),
        (right_handle, right_valve, "right"),
    ):
        ctx.allow_overlap(
            hdl,
            vlv,
            elem_a=hdl.get_visual("stem"),
            elem_b=vlv.get_visual("valve_body"),
            reason="handle stem is seated inside the valve body bore and turns with the handle",
        )
        ctx.allow_overlap(
            hdl,
            vlv,
            elem_a=hdl.get_visual("stem"),
            elem_b=vlv.get_visual("stem_collar"),
            reason="handle stem passes through the visible stem collar and turns with the handle",
        )
    # Prove the stem is retained inside the valve body and collar
    for hdl, vlv, side_name in (
        (left_handle, left_valve, "left"),
        (right_handle, right_valve, "right"),
    ):
        ctx.expect_within(
            hdl,
            vlv,
            axes="xy",
            inner_elem="stem",
            outer_elem="valve_body",
            margin=0.005,
            name=f"{side_name}_stem_centered_in_valve_body",
        )
        ctx.expect_within(
            hdl,
            vlv,
            axes="xy",
            inner_elem="stem",
            outer_elem="stem_collar",
            margin=0.005,
            name=f"{side_name}_stem_centered_in_collar",
        )

    # --- handle rotation proof: arms sweep in horizontal plane ---
    rest_aabb = ctx.part_world_aabb(left_handle)
    assert rest_aabb is not None
    rest_dx = rest_aabb[1][0] - rest_aabb[0][0]
    rest_dz = rest_aabb[1][2] - rest_aabb[0][2]

    with ctx.pose({left_joint: math.pi / 4.0}):
        rot_aabb = ctx.part_world_aabb(left_handle)
        assert rot_aabb is not None
        rot_dx = rot_aabb[1][0] - rot_aabb[0][0]
        rot_dz = rot_aabb[1][2] - rot_aabb[0][2]
        ctx.check(
            "left_handle_rotation_shrinks_x_bbox",
            rot_dx < rest_dx * 0.85,
            f"rest_dx={rest_dx:.4f}, rot_dx={rot_dx:.4f}",
        )
        ctx.check(
            "left_handle_z_extent_stable_during_rotation",
            abs(rot_dz - rest_dz) < 0.003,
            f"rest_dz={rest_dz:.4f}, rot_dz={rot_dz:.4f} (arms stay horizontal)",
        )
        # Handle stays on its valve axis
        cen = ctx.part_world_position(left_handle)
        assert cen is not None
        ctx.check(
            "left_handle_stays_on_valve_axis",
            abs(cen[0] + VALVE_X) < 1e-6,
            f"handle x={cen[0]}",
        )

    with ctx.pose({right_joint: -math.pi / 2.0}):
        # Quarter turn: cross maps onto itself, handle stays on valve
        ctx.expect_overlap(right_handle, right_valve, axes="xy", min_overlap=0.005)
        # Handle clear of spout
        ctx.expect_gap(right_handle, spout, axis="x", min_gap=0.01)

    # --- deck panel grounded at z=0 ---
    deck_aabb = ctx.part_world_aabb(deck)
    assert deck_aabb is not None
    ctx.check(
        "deck_panel_grounded",
        abs(deck_aabb[0][2]) < 1e-6,
        f"deck zmin={deck_aabb[0][2]:.4f}",
    )

    # --- overall width about 0.30 m across handle tips ---
    rh_aabb = ctx.part_world_aabb(right_handle)
    assert rh_aabb is not None
    total_w = rh_aabb[1][0] - lh_aabb[0][0]
    ctx.check(
        "overall_width_about_0p30",
        0.26 <= total_w <= 0.33,
        f"tip-to-tip width={total_w:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
