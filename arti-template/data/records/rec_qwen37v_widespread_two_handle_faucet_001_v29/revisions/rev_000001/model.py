from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Widespread two-handle deck-mounted bathroom faucet in polished gold brass.
#
# Frame conventions:
#   - The deck plate sits flat on a horizontal surface at z = 0.
#   - The spout rises vertically (+Z) from the centre, curving over toward +Y.
#   - Two flanking valve stems are at x = +/- HANDLE_SPACING.
#   - Handles rotate about the vertical axis (Z).
#   - Underside nuts hang below the deck (z < 0).
# ---------------------------------------------------------------------------

# Layout
HANDLE_SPACING = 0.10  # handle centres at x = +/- 0.10 m

# Deck plate (oval escutcheon)
DECK_RX = 0.160  # x semi-axis
DECK_RY = 0.035  # y semi-axis
DECK_BASE_H = 0.008
DECK_STEP_H = 0.004
DECK_TOP = DECK_BASE_H + DECK_STEP_H  # 0.012 m

# Spout
SPOUT_TUBE_R = 0.012
SPOUT_BORE_R = 0.008
SPOUT_RISE = 0.14
SPOUT_REACH_Y = 0.10
SPOUT_PEAK_Z = 0.18
SPOUT_OUTLET_Z = 0.06
SPOUT_FLANGE_R1 = 0.022
SPOUT_FLANGE_H1 = 0.006
SPOUT_FLANGE_R2 = 0.017
SPOUT_FLANGE_H2 = 0.006

# Stems and collars
STEM_R = 0.006
STEM_H = 0.025
COLLAR_R = 0.013
COLLAR_H = 0.006

# Cross handle
HANDLE_ROD_R = 0.004
HANDLE_ROD_LEN = 0.080
HUB_R = 0.011
HUB_H = 0.016
KNURL_R = 0.0125
SPOKE_Z = HUB_H * 0.35  # spoke plane height in handle-local frame

# Underside hex nuts
NUT_FLAT = 0.014  # flat-to-flat diameter
NUT_H = 0.005
NUT_BORE_R = 0.005

# Computed by build for hollow-bore verification in run_tests().
SPOUT_SOLID_VOLUME: float = 0.0
SPOUT_UNBORED_VOLUME: float = 0.0


# ---- CadQuery mesh builders ------------------------------------------------


def _build_deck_plate_solid() -> cq.Workplane:
    """Raised oval escutcheon plate with stepped profile and post bosses."""
    base = cq.Workplane("XY").ellipse(DECK_RX, DECK_RY).extrude(DECK_BASE_H)
    step = (
        cq.Workplane("XY")
        .workplane(offset=DECK_BASE_H)
        .ellipse(DECK_RX - 0.005, DECK_RY - 0.004)
        .extrude(DECK_STEP_H)
    )
    plate = base.union(step)

    # Raised bosses around each post hole on top of the deck plate.
    boss_r = 0.018
    boss_h = 0.003
    for x_pos in (-HANDLE_SPACING, 0.0, HANDLE_SPACING):
        boss = (
            cq.Workplane("XY")
            .workplane(offset=DECK_TOP)
            .center(x_pos, 0.0)
            .circle(boss_r)
            .extrude(boss_h)
        )
        plate = plate.union(boss)
    return plate


def _build_spout_solid() -> cq.Workplane:
    """Spout tube rising from deck, curving over and down to a hollow outlet."""
    # Arc mid/end points in YZ workplane coords (Y, Z).
    mid_y = SPOUT_REACH_Y * 0.45
    mid_z = SPOUT_RISE + (SPOUT_PEAK_Z - SPOUT_RISE) * 0.85
    end_y = SPOUT_REACH_Y
    end_z = SPOUT_PEAK_Z - 0.02

    path = (
        cq.Workplane("YZ")
        .moveTo(0.0, 0.0)
        .lineTo(0.0, SPOUT_RISE)
        .threePointArc((mid_y, mid_z), (end_y, end_z))
        .lineTo(end_y, SPOUT_OUTLET_Z)
    )
    bore_path = (
        cq.Workplane("YZ")
        .moveTo(0.0, -0.004)
        .lineTo(0.0, SPOUT_RISE)
        .threePointArc((mid_y, mid_z), (end_y, end_z))
        .lineTo(end_y, SPOUT_OUTLET_Z - 0.003)
    )

    # Cross-section at path start (XY plane, normal +Z).
    tube = cq.Workplane("XY").circle(SPOUT_TUBE_R).sweep(path)
    bore = (
        cq.Workplane("XY")
        .workplane(offset=-0.004)
        .circle(SPOUT_BORE_R)
        .sweep(bore_path)
    )

    # Stepped base flange where the spout meets the deck.
    flange1 = cq.Workplane("XY").circle(SPOUT_FLANGE_R1).extrude(SPOUT_FLANGE_H1)
    flange2 = (
        cq.Workplane("XY")
        .workplane(offset=SPOUT_FLANGE_H1)
        .circle(SPOUT_FLANGE_R2)
        .extrude(SPOUT_FLANGE_H2)
    )

    unbored = tube.union(flange1).union(flange2)
    solid = unbored.cut(bore)

    global SPOUT_SOLID_VOLUME, SPOUT_UNBORED_VOLUME
    SPOUT_SOLID_VOLUME = solid.val().Volume()
    SPOUT_UNBORED_VOLUME = unbored.val().Volume()
    return solid


def _build_hub_solid() -> cq.Workplane:
    """Cross-handle central hub, axis +Z, bottom face at z = 0."""
    hub = cq.Workplane("XY").circle(HUB_R).extrude(HUB_H)
    knurl = (
        cq.Workplane("XY")
        .workplane(offset=0.004)
        .polygon(16, 2.0 * KNURL_R)
        .extrude(0.008)
    )
    dome = cq.Workplane("XY").workplane(offset=HUB_H).sphere(0.009)
    return hub.union(knurl).union(dome)


def _build_nut_solid() -> cq.Workplane:
    """Hex nut with centre bore, bottom face at z = 0."""
    nut_body = cq.Workplane("XY").polygon(6, NUT_FLAT).extrude(NUT_H)
    bore = (
        cq.Workplane("XY")
        .workplane(offset=-0.001)
        .circle(NUT_BORE_R)
        .extrude(NUT_H + 0.002)
    )
    return nut_body.cut(bore)


# ---- Model -----------------------------------------------------------------


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_two_handle_faucet")

    gold = model.material("polished_gold_brass", rgba=(0.85, 0.66, 0.20, 1.0))

    # ---- deck plate (root, oval escutcheon) ----
    deck = model.part("deck_plate")
    deck.visual(
        mesh_from_cadquery(_build_deck_plate_solid(), "deck_plate"),
        material=gold,
        name="plate",
    )

    # ---- central spout (fixed to deck) ----
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
        origin=Origin(xyz=(0.0, 0.0, DECK_TOP)),
    )

    # ---- shared meshes ----
    hub_mesh = mesh_from_cadquery(_build_hub_solid(), "handle_hub")
    nut_mesh = mesh_from_cadquery(_build_nut_solid(), "hex_nut")

    # ---- spout underside nut ----
    spout_nut = model.part("spout_nut")
    spout_nut.visual(nut_mesh, material=gold, name="hex_nut")
    model.articulation(
        "deck_to_spout_nut",
        ArticulationType.FIXED,
        parent=deck,
        child=spout_nut,
        origin=Origin(xyz=(0.0, 0.0, -NUT_H)),
    )

    # ---- valve stems (fixed) + cross handles (revolute) + nuts ----
    for side, sx in (("left", -1.0), ("right", 1.0)):
        # -- stem assembly (fixed to deck) --
        stem_part = model.part(f"{side}_stem")
        # Vertical shaft (stops below the collar so insert seats only in collar)
        shaft_len = STEM_H - COLLAR_H
        stem_part.visual(
            Cylinder(radius=STEM_R, length=shaft_len),
            origin=Origin(xyz=(0.0, 0.0, shaft_len / 2.0)),
            material=gold,
            name="shaft",
        )
        # Visible collar ring at top of stem
        stem_part.visual(
            Cylinder(radius=COLLAR_R, length=COLLAR_H),
            origin=Origin(xyz=(0.0, 0.0, STEM_H - COLLAR_H / 2.0)),
            material=gold,
            name="collar",
        )
        model.articulation(
            f"deck_to_{side}_stem",
            ArticulationType.FIXED,
            parent=deck,
            child=stem_part,
            origin=Origin(xyz=(sx * HANDLE_SPACING, 0.0, DECK_TOP)),
        )

        # -- cross handle (revolute, vertical axis) --
        handle = model.part(f"{side}_handle")
        # Stem insert (extends down into collar bore only)
        insert_len = COLLAR_H  # seats fully within the collar
        handle.visual(
            Cylinder(radius=STEM_R * 0.85, length=insert_len),
            origin=Origin(xyz=(0.0, 0.0, -insert_len / 2.0)),
            material=gold,
            name="stem_insert",
        )
        handle.visual(hub_mesh, material=gold, name="hub")
        # Spoke along X (cylinder rotated from Z to X)
        handle.visual(
            Cylinder(radius=HANDLE_ROD_R, length=HANDLE_ROD_LEN),
            origin=Origin(xyz=(0.0, 0.0, SPOKE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=gold,
            name="spoke_x",
        )
        # Spoke along Y (cylinder rotated from Z to Y)
        handle.visual(
            Cylinder(radius=HANDLE_ROD_R, length=HANDLE_ROD_LEN),
            origin=Origin(xyz=(0.0, 0.0, SPOKE_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=gold,
            name="spoke_y",
        )
        # Rounded spoke tips
        half = HANDLE_ROD_LEN / 2.0
        for tip_name, (dx, dy) in (
            ("tip_x_pos", (half, 0.0)),
            ("tip_x_neg", (-half, 0.0)),
            ("tip_y_pos", (0.0, half)),
            ("tip_y_neg", (0.0, -half)),
        ):
            handle.visual(
                Sphere(radius=HANDLE_ROD_R * 1.15),
                origin=Origin(xyz=(dx, dy, SPOKE_Z)),
                material=gold,
                name=tip_name,
            )

        # Revolute joint at top of stem, axis vertical (+Z)
        model.articulation(
            f"{side}_handle_spindle",
            ArticulationType.REVOLUTE,
            parent=stem_part,
            child=handle,
            origin=Origin(xyz=(0.0, 0.0, STEM_H)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=5.0, velocity=3.0, lower=-math.pi, upper=math.pi
            ),
        )

        # -- underside nut --
        nut_part = model.part(f"{side}_nut")
        nut_part.visual(nut_mesh, material=gold, name="hex_nut")
        model.articulation(
            f"deck_to_{side}_nut",
            ArticulationType.FIXED,
            parent=deck,
            child=nut_part,
            origin=Origin(xyz=(sx * HANDLE_SPACING, 0.0, -NUT_H)),
        )

    return model


# ---- Tests -----------------------------------------------------------------


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck_plate")
    spout = object_model.get_part("spout")
    left_stem = object_model.get_part("left_stem")
    right_stem = object_model.get_part("right_stem")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")
    left_nut = object_model.get_part("left_nut")
    right_nut = object_model.get_part("right_nut")
    spout_nut = object_model.get_part("spout_nut")
    left_joint = object_model.get_articulation("left_handle_spindle")
    right_joint = object_model.get_articulation("right_handle_spindle")

    # ---- joint plan: two independent revolute handles, vertical axis, +-180 deg ----
    for joint in (left_joint, right_joint):
        ctx.check(
            f"{joint.name}_is_revolute",
            str(joint.joint_type).lower().endswith("revolute"),
            f"type={joint.joint_type}",
        )
        ax = joint.axis
        ctx.check(
            f"{joint.name}_axis_is_vertical",
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

    # ---- deck plate is grounded at z = 0 ----
    deck_aabb = ctx.part_world_aabb(deck)
    assert deck_aabb is not None
    ctx.check(
        "deck_plate_grounded_at_z0",
        abs(deck_aabb[0][2]) < 1e-6,
        f"deck zmin={deck_aabb[0][2]:.4f}",
    )

    # ---- deck plate oval width about 0.32 m ----
    deck_w = deck_aabb[1][0] - deck_aabb[0][0]
    ctx.check(
        "deck_plate_oval_width_about_0p32",
        0.28 <= deck_w <= 0.34,
        f"deck width={deck_w:.3f}",
    )

    # ---- stem collars visible (collar radius > stem radius) ----
    for side, stem_p in (("left", left_stem), ("right", right_stem)):
        collar_aabb = ctx.part_element_world_aabb(stem_p, elem="collar")
        shaft_aabb = ctx.part_element_world_aabb(stem_p, elem="shaft")
        assert collar_aabb is not None and shaft_aabb is not None
        collar_dx = collar_aabb[1][0] - collar_aabb[0][0]
        shaft_dx = shaft_aabb[1][0] - shaft_aabb[0][0]
        ctx.check(
            f"{side}_collar_wider_than_shaft",
            collar_dx > shaft_dx + 0.005,
            f"collar_dx={collar_dx:.4f}, shaft_dx={shaft_dx:.4f}",
        )

    # ---- underside nuts below deck plate (z < 0) ----
    for side, nut_p in (("left", left_nut), ("right", right_nut)):
        nut_aabb = ctx.part_world_aabb(nut_p)
        assert nut_aabb is not None
        ctx.check(
            f"{side}_nut_below_deck",
            nut_aabb[0][2] < -0.002,
            f"nut zmin={nut_aabb[0][2]:.4f}",
        )
    spout_nut_aabb = ctx.part_world_aabb(spout_nut)
    assert spout_nut_aabb is not None
    ctx.check(
        "spout_nut_below_deck",
        spout_nut_aabb[0][2] < -0.002,
        f"spout_nut zmin={spout_nut_aabb[0][2]:.4f}",
    )

    # ---- handle stem inserts seat inside stem collars (intentional overlap) ----
    ctx.allow_overlap(
        left_handle,
        left_stem,
        elem_a=left_handle.get_visual("stem_insert"),
        elem_b=left_stem.get_visual("collar"),
        reason="handle stem insert is seated inside the collar bore and turns with the handle",
    )
    ctx.allow_overlap(
        right_handle,
        right_stem,
        elem_a=right_handle.get_visual("stem_insert"),
        elem_b=right_stem.get_visual("collar"),
        reason="handle stem insert is seated inside the collar bore and turns with the handle",
    )

    # ---- handles stay on stem axis while rotating ----
    with ctx.pose({left_joint: math.pi / 4.0}):
        rot_aabb = ctx.part_world_aabb(left_handle)
        assert rot_aabb is not None
        # At 45 deg the cross AABB shrinks on X (spokes become diagonal)
        rest_aabb = ctx.part_world_aabb(left_handle)  # inside pose, so this is rotated
        rot_dx = rot_aabb[1][0] - rot_aabb[0][0]
        ctx.check(
            "left_handle_spokes_rotate_off_axis",
            rot_dx < 0.075,
            f"x extent at q=45deg is {rot_dx:.3f} (cross shrinks from ~0.080)",
        )
        # Handle stays centred on its stem
        cen = ctx.part_world_position(left_handle)
        assert cen is not None
        expected_x = -HANDLE_SPACING
        ctx.check(
            "left_handle_stays_on_stem_axis",
            abs(cen[0] - expected_x) < 0.002,
            f"handle x={cen[0]:.4f}, expected={expected_x}",
        )

    with ctx.pose({right_joint: -math.pi / 2.0}):
        # Quarter turn: cross maps onto itself, handle must remain on stem.
        ctx.expect_overlap(right_handle, right_stem, axes="xy", min_overlap=0.005)

    # ---- spout geometry: hollow bore, rises, curves, reaches forward ----
    ctx.check(
        "spout_tube_is_hollow",
        0.0 < SPOUT_SOLID_VOLUME < 0.95 * SPOUT_UNBORED_VOLUME,
        f"solid={SPOUT_SOLID_VOLUME:.3e} vs unbored={SPOUT_UNBORED_VOLUME:.3e}",
    )
    spout_aabb = ctx.part_world_aabb(spout)
    assert spout_aabb is not None
    (sx0, sy0, sz0), (sx1, sy1, sz1) = spout_aabb
    spout_top = sz1 - DECK_TOP
    ctx.check(
        "spout_rises_about_0p17_above_deck",
        0.15 <= spout_top <= 0.21,
        f"spout peak above deck={spout_top:.3f}",
    )
    ctx.check(
        "spout_reaches_forward_in_y",
        sy1 > 0.06,
        f"spout ymax={sy1:.3f}",
    )
    ctx.check(
        "spout_outlet_drops_toward_deck",
        sz0 < DECK_TOP + 0.10,
        f"spout zmin={sz0:.3f}",
    )

    # ---- handle cross size about 0.08 m tip to tip ----
    lh_aabb = ctx.part_world_aabb(left_handle)
    assert lh_aabb is not None
    lh_dx = lh_aabb[1][0] - lh_aabb[0][0]
    ctx.check(
        "left_handle_cross_about_0p08_tip_to_tip",
        0.070 <= lh_dx <= 0.100,
        f"handle x extent={lh_dx:.3f}",
    )

    # ---- overall width about 0.30 m ----
    rh_aabb = ctx.part_world_aabb(right_handle)
    assert rh_aabb is not None
    total_w = max(rh_aabb[1][0], deck_aabb[1][0]) - min(lh_aabb[0][0], deck_aabb[0][0])
    ctx.check(
        "overall_width_about_0p30",
        0.28 <= total_w <= 0.36,
        f"total width={total_w:.3f}",
    )

    # ---- handles overlap their stems in XY (mounted, not floating) ----
    ctx.expect_overlap(left_handle, left_stem, axes="xy", min_overlap=0.005)
    ctx.expect_overlap(right_handle, right_stem, axes="xy", min_overlap=0.005)

    return ctx.report()


object_model = build_object_model()
