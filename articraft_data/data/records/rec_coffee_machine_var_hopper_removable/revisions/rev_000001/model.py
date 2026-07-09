from __future__ import annotations

"""DeLonghi Magnifica-style super-automatic bean-to-cup espresso machine.

True scale: ~0.43 m tall, 0.24 m wide, 0.35 m deep, grounded at z=0.
Frame: +X = front, +Y = machine left, +Z = up.

Articulations (per brief):
  1. selection dial      - CONTINUOUS, axis normal to the 15-degree tilted fascia
  2. spout block         - PRISMATIC, vertical, 0.06 m travel (q=0 at top)
  3. drip tray           - PRISMATIC, slides forward +X, 0.12 m travel
  4. steam wand          - REVOLUTE about a vertical axis at its upper mount, ~60 deg
  5. hopper canister     - PRISMATIC, vertical lift +Z, 0.10 m travel (removable)
"""

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobIndicator,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------- dimensions
WIDTH = 0.24           # Y extent
DEPTH = 0.35           # X extent of the main body
HEIGHT = 0.43          # Z extent including hopper lid
X_FRONT = DEPTH / 2.0  # +0.175
X_REAR = -DEPTH / 2.0  # -0.175

TILT = math.radians(15.0)  # fascia lean-back angle
FASCIA_TH = 0.025
FASCIA_H = 0.155
# Fascia outer bottom edge sits at the front face, z = 0.262.
_N = (math.cos(TILT), 0.0, math.sin(TILT))        # fascia outward normal
_H = (-math.sin(TILT), 0.0, math.cos(TILT))       # fascia "up the panel" dir
FASCIA_C = (
    X_FRONT - (FASCIA_H / 2.0) * math.sin(TILT) - (FASCIA_TH / 2.0) * math.cos(TILT),
    0.0,
    0.262 + (FASCIA_H / 2.0) * math.cos(TILT) - (FASCIA_TH / 2.0) * math.sin(TILT),
)

SPOUT_TRAVEL = 0.06
TRAY_TRAVEL = 0.12
WAND_SWING = math.radians(60.0)

# Hopper canister (lift-out, replaces hinged lid)
HOPPER_W = 0.100        # X extent of canister body
HOPPER_D = 0.160        # Y extent of canister body
HOPPER_H = 0.080        # Z extent of canister walls
HOPPER_WALL = 0.004     # wall thickness
HOPPER_CENTER_X = -0.050  # center X on top deck (rear portion)
DECK_TOP_Z = 0.418      # top surface of top_deck
GUIDE_H = 0.030         # guide wall height above deck
GUIDE_T = 0.005         # guide wall thickness
HOPPER_TRAVEL = 0.10    # vertical lift travel


def fascia_point(u: float, y: float, proud: float) -> tuple[float, float, float]:
    """Point on (or proud of) the fascia outer surface; u runs up the panel."""
    return (
        FASCIA_C[0] + (FASCIA_TH / 2.0 + proud) * _N[0] + u * _H[0],
        y,
        FASCIA_C[2] + (FASCIA_TH / 2.0 + proud) * _N[2] + u * _H[2],
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="super_automatic_espresso_machine")

    model.material("gloss_black", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("graphite", rgba=(0.17, 0.17, 0.185, 1.0))
    model.material("fascia_gray", rgba=(0.55, 0.56, 0.575, 1.0))
    model.material("trim_black", rgba=(0.065, 0.065, 0.07, 1.0))
    model.material("stainless", rgba=(0.72, 0.73, 0.75, 1.0))
    model.material("silver", rgba=(0.80, 0.81, 0.83, 1.0))
    model.material("grid_dark", rgba=(0.24, 0.24, 0.255, 1.0))
    model.material("button_dark", rgba=(0.30, 0.30, 0.315, 1.0))

    # ------------------------------------------------------------- body root
    body = model.part("body")

    # Rear base block (the drip tray docks in front of it).
    body.visual(
        Box((0.195, WIDTH, 0.080)),
        origin=Origin(xyz=(-0.0775, 0.0, 0.040)),
        material="graphite",
        name="base_block",
    )
    # Thin side skirts flanking the drip tray bay.
    for sgn, tag in ((1.0, "left"), (-1.0, "right")):
        body.visual(
            Box((0.165, 0.010, 0.080)),
            origin=Origin(xyz=(0.0925, sgn * 0.115, 0.040)),
            material="graphite",
            name=f"{tag}_base_skirt",
        )
    # Main glossy black core / side panels.
    body.visual(
        Box((0.275, WIDTH, 0.325)),
        origin=Origin(xyz=(-0.0375, 0.0, 0.2375)),
        material="gloss_black",
        name="core_shell",
    )
    # Gray front cheeks flanking the dispensing recess.
    for sgn, tag in ((1.0, "left"), (-1.0, "right")):
        body.visual(
            Box((0.080, 0.055, 0.190)),
            origin=Origin(xyz=(0.135, sgn * 0.0925, 0.170)),
            material="fascia_gray",
            name=f"{tag}_front_cheek",
        )
    # Recessed black channel back wall between the cheeks.
    body.visual(
        Box((0.018, 0.130, 0.190)),
        origin=Origin(xyz=(0.107, 0.0, 0.170)),
        material="trim_black",
        name="channel_back_wall",
    )
    # Vertical rail the spout carriage rides on.
    body.visual(
        Box((0.014, 0.032, 0.170)),
        origin=Origin(xyz=(0.119, 0.0, 0.165)),
        material="trim_black",
        name="spout_rail",
    )
    # Tilted gray control fascia across the upper front.
    body.visual(
        Box((FASCIA_TH, WIDTH, FASCIA_H)),
        origin=Origin(xyz=FASCIA_C, rpy=(0.0, -TILT, 0.0)),
        material="fascia_gray",
        name="fascia_panel",
    )
    # Flat black top deck.
    body.visual(
        Box((0.293, WIDTH, 0.020)),
        origin=Origin(xyz=(-0.0285, 0.0, 0.408)),
        material="trim_black",
        name="top_deck",
    )
    # Perforated cup-warming grid inset on the front of the deck.
    cup_grid = PerforatedPanelGeometry(
        (0.150, 0.180),
        0.004,
        hole_diameter=0.005,
        pitch=0.011,
        frame=0.008,
        stagger=True,
    )
    body.visual(
        mesh_from_geometry(cup_grid, "cup_warming_grid"),
        origin=Origin(xyz=(0.030, 0.0, 0.419)),
        material="grid_dark",
        name="cup_warming_grid",
    )
    # Square fascia buttons (3 per side) plus a small round power button.
    for sgn, tag in ((1.0, "left"), (-1.0, "right")):
        for k, u in enumerate((0.034, 0.0, -0.034)):
            body.visual(
                Box((0.007, 0.016, 0.016)),
                origin=Origin(xyz=fascia_point(u, sgn * 0.078, 0.003), rpy=(0.0, -TILT, 0.0)),
                material="button_dark",
                name=f"{tag}_button_{k}",
            )
    body.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(
            xyz=fascia_point(0.058, 0.085, 0.002),
            rpy=(0.0, math.pi / 2.0 - TILT, 0.0),
        ),
        material="button_dark",
        name="power_button",
    )
    # Brand badge plate below the dial.
    body.visual(
        Box((0.004, 0.055, 0.014)),
        origin=Origin(xyz=fascia_point(-0.060, 0.0, 0.001), rpy=(0.0, -TILT, 0.0)),
        material="silver",
        name="brand_badge",
    )
    # Steam-wand pivot boss on the left flank.
    body.visual(
        Cylinder(radius=0.015, length=0.035),
        origin=Origin(xyz=(0.085, 0.125, 0.285), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="trim_black",
        name="wand_boss",
    )

    # --- Hopper canister guide bay on top deck ---
    # Four thin guide walls form a rectangular bay for the lift-out canister.
    guide_base_z = DECK_TOP_Z + GUIDE_H / 2.0
    half_w = HOPPER_W / 2.0 + HOPPER_WALL + GUIDE_T / 2.0 + 0.002
    half_d = HOPPER_D / 2.0 + HOPPER_WALL + GUIDE_T / 2.0 + 0.002
    for sgn, tag in ((1.0, "front"), (-1.0, "rear")):
        body.visual(
            Box((GUIDE_T, HOPPER_D + 2 * HOPPER_WALL + GUIDE_T, GUIDE_H)),
            origin=Origin(xyz=(HOPPER_CENTER_X + sgn * half_w, 0.0, guide_base_z)),
            material="trim_black",
            name=f"hopper_guide_{tag}",
        )
    for sgn, tag in ((1.0, "left"), (-1.0, "right")):
        body.visual(
            Box((HOPPER_W + 2 * HOPPER_WALL + GUIDE_T, GUIDE_T, GUIDE_H)),
            origin=Origin(xyz=(HOPPER_CENTER_X, sgn * half_d, guide_base_z)),
            material="trim_black",
            name=f"hopper_guide_{tag}",
        )
    # Recessed floor inside bay (slightly below deck top to seat canister).
    body.visual(
        Box((HOPPER_W + 2 * HOPPER_WALL, HOPPER_D + 2 * HOPPER_WALL, 0.004)),
        origin=Origin(xyz=(HOPPER_CENTER_X, 0.0, DECK_TOP_Z - 0.002)),
        material="trim_black",
        name="hopper_bay_floor",
    )

    # --------------------------------------------------------- selection dial
    dial = model.part("selection_dial")
    dial.visual(
        Cylinder(radius=0.008, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material="trim_black",
        name="dial_stem",
    )
    dial.visual(
        Cylinder(radius=0.027, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.0035)),
        material="silver",
        name="dial_ring",
    )
    knob = KnobGeometry(
        0.040,
        0.022,
        body_style="cylindrical",
        edge_radius=0.002,
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=90.0),
        center=False,
    )
    dial.visual(
        mesh_from_geometry(knob, "dial_knob"),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material="silver",
        name="dial_knob",
    )
    dial.visual(
        Box((0.003, 0.003, 0.0024)),
        origin=Origin(xyz=(0.0, 0.014, 0.0287)),
        material="trim_black",
        name="dial_pointer",
    )
    model.articulation(
        "fascia_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(
            xyz=fascia_point(0.005, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0 - TILT, 0.0),
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0),
    )

    # ----------------------------------------------------------- spout block
    spout = model.part("spout_block")
    spout.visual(
        Box((0.058, 0.092, 0.075)),
        origin=Origin(xyz=(0.0, 0.0, -0.0375)),
        material="trim_black",
        name="spout_housing",
    )
    for sgn, tag in ((1.0, "left"), (-1.0, "right")):
        spout.visual(
            Cylinder(radius=0.0065, length=0.030),
            origin=Origin(xyz=(0.012, sgn * 0.020, -0.090)),
            material="stainless",
            name=f"{tag}_nozzle",
        )
    model.articulation(
        "body_to_spout",
        ArticulationType.PRISMATIC,
        parent=body,
        child=spout,
        origin=Origin(xyz=(0.146, 0.0, 0.250)),
        # Positive q lowers the spout toward a short cup.
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.1, lower=0.0, upper=SPOUT_TRAVEL),
    )

    # ------------------------------------------------------------- drip tray
    tray = model.part("drip_tray")
    tray.visual(
        Box((0.165, 0.215, 0.008)),
        origin=Origin(xyz=(0.1025, 0.0, 0.004)),
        material="trim_black",
        name="tray_floor",
    )
    tray.visual(
        Box((0.010, 0.215, 0.064)),
        origin=Origin(xyz=(0.174, 0.0, 0.038)),
        material="trim_black",
        name="tray_front_wall",
    )
    # Rear wall butts against the body base block so the docked tray reads seated.
    tray.visual(
        Box((0.010, 0.215, 0.064)),
        origin=Origin(xyz=(0.025, 0.0, 0.038)),
        material="trim_black",
        name="tray_rear_wall",
    )
    for sgn, tag in ((1.0, "left"), (-1.0, "right")):
        tray.visual(
            Box((0.160, 0.008, 0.064)),
            origin=Origin(xyz=(0.101, sgn * 0.1035, 0.038)),
            material="trim_black",
            name=f"tray_{tag}_wall",
        )
    drip_plate = PerforatedPanelGeometry(
        (0.150, 0.200),
        0.004,
        hole_diameter=0.007,
        pitch=0.016,
        frame=0.010,
        stagger=True,
    )
    tray.visual(
        mesh_from_geometry(drip_plate, "drip_plate"),
        origin=Origin(xyz=(0.100, 0.0, 0.070)),
        material="stainless",
        name="drip_plate",
    )
    # Glossy front lip standing slightly proud of the body front.
    tray.visual(
        Box((0.008, 0.215, 0.052)),
        origin=Origin(xyz=(0.182, 0.0, 0.034)),
        material="gloss_black",
        name="tray_front_lip",
    )
    model.articulation(
        "body_to_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.2, lower=0.0, upper=TRAY_TRAVEL),
    )

    # ------------------------------------------------------------ steam wand
    wand = model.part("steam_wand")
    wand.visual(
        Cylinder(radius=0.012, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material="trim_black",
        name="pivot_knuckle",
    )
    lean = 0.28  # outward lean of the wand tube (rad)
    d = (0.0, math.sin(lean), -math.cos(lean))
    top = (0.0, 0.003, -0.020)
    tube_rpy = (math.pi + lean, 0.0, 0.0)
    wand.visual(
        Cylinder(radius=0.0055, length=0.130),
        origin=Origin(
            xyz=(top[0] + 0.065 * d[0], top[1] + 0.065 * d[1], top[2] + 0.065 * d[2]),
            rpy=tube_rpy,
        ),
        material="stainless",
        name="wand_tube",
    )
    wand.visual(
        Cylinder(radius=0.0075, length=0.040),
        origin=Origin(
            xyz=(top[0] + 0.145 * d[0], top[1] + 0.145 * d[1], top[2] + 0.145 * d[2]),
            rpy=tube_rpy,
        ),
        material="trim_black",
        name="frother_sleeve",
    )
    model.articulation(
        "body_to_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=wand,
        origin=Origin(xyz=(0.085, 0.138, 0.285)),
        # -Z so positive q swings the wand forward (toward +X).
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=0.0, upper=WAND_SWING),
    )

    # ---------------------------------------------------------- hopper canister
    # Lift-out bean hopper canister (PRISMATIC along +Z) replacing hinged lid.
    canister = model.part("hopper_canister")

    # Main canister shell: hollow rounded-rectangle container (open top).
    canister_body = (
        cq.Workplane("XY")
        .box(HOPPER_W, HOPPER_D, HOPPER_H)
        .edges("|Z")
        .fillet(0.008)
        .faces(">Z")
        .shell(-HOPPER_WALL)
    )
    canister.visual(
        mesh_from_cadquery(canister_body, "canister_shell"),
        origin=Origin(xyz=(0.0, 0.0, HOPPER_H / 2.0)),
        material="graphite",
        name="canister_shell",
    )

    # Flat lid plate on top of the canister.
    canister.visual(
        Box((HOPPER_W - 0.002, HOPPER_D - 0.002, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, HOPPER_H + 0.003)),
        material="trim_black",
        name="canister_lid_plate",
    )

    # Grip handle on top of the lid for lifting.
    canister.visual(
        Box((0.040, 0.015, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, HOPPER_H + 0.012)),
        material="trim_black",
        name="canister_grip",
    )

    # Small alignment rib on the bottom (fits into bay floor groove).
    canister.visual(
        Box((HOPPER_W - 0.010, 0.008, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material="trim_black",
        name="canister_align_rib",
    )

    model.articulation(
        "body_to_hopper",
        ArticulationType.PRISMATIC,
        parent=body,
        child=canister,
        origin=Origin(xyz=(HOPPER_CENTER_X, 0.0, DECK_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=15.0, velocity=0.15, lower=0.0, upper=HOPPER_TRAVEL
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    dial = object_model.get_part("selection_dial")
    spout = object_model.get_part("spout_block")
    tray = object_model.get_part("drip_tray")
    wand = object_model.get_part("steam_wand")
    canister = object_model.get_part("hopper_canister")

    j_dial = object_model.get_articulation("fascia_to_dial")
    j_spout = object_model.get_articulation("body_to_spout")
    j_tray = object_model.get_articulation("body_to_tray")
    j_wand = object_model.get_articulation("body_to_wand")
    j_hopper = object_model.get_articulation("body_to_hopper")

    # Intentional local embeddings that represent real mechanical capture.
    ctx.allow_overlap(
        dial,
        body,
        elem_a="dial_stem",
        elem_b="fascia_panel",
        reason="Dial shaft is intentionally inserted through the fascia panel.",
    )
    ctx.allow_overlap(
        spout,
        body,
        elem_a="spout_housing",
        elem_b="spout_rail",
        reason="Spout carriage is intentionally captured on the vertical guide rail.",
    )
    ctx.allow_overlap(
        wand,
        body,
        elem_a="pivot_knuckle",
        elem_b="wand_boss",
        reason="Wand knuckle is intentionally captured on the side pivot boss.",
    )
    ctx.allow_overlap(
        canister,
        body,
        elem_a="canister_align_rib",
        elem_b="hopper_bay_floor",
        reason="Canister alignment rib is intentionally seated in the bay floor recess.",
    )

    # ---- overall scale and grounding -------------------------------------
    body_aabb = ctx.part_world_aabb(body)
    canister_aabb = ctx.part_world_aabb(canister)
    tray_aabb = ctx.part_world_aabb(tray)
    core_aabb = ctx.part_element_world_aabb(body, elem="core_shell")
    ctx.check(
        "body footprint matches real machine scale",
        body_aabb is not None
        and core_aabb is not None
        and abs((core_aabb[1][1] - core_aabb[0][1]) - WIDTH) < 0.01
        and abs((body_aabb[1][0] - body_aabb[0][0]) - DEPTH) < 0.02,
        details=f"body aabb={body_aabb}, core aabb={core_aabb}",
    )
    ctx.check(
        "machine grounded at z=0",
        body_aabb is not None
        and tray_aabb is not None
        and abs(body_aabb[0][2]) < 1e-6
        and abs(tray_aabb[0][2]) < 1e-6,
        details=f"body zmin={body_aabb}, tray zmin={tray_aabb}",
    )
    ctx.check(
        "overall height about 0.43 m including hopper canister",
        canister_aabb is not None and 0.42 < canister_aabb[1][2] < 0.52,
        details=f"canister aabb={canister_aabb}",
    )

    # ---- joint plan metadata ----------------------------------------------
    ctx.check(
        "dial joint is continuous and unbounded",
        j_dial.articulation_type == ArticulationType.CONTINUOUS
        and (j_dial.motion_limits is None or j_dial.motion_limits.upper is None),
    )
    ctx.check(
        "spout joint is a 0.06 m vertical prismatic slide",
        j_spout.articulation_type == ArticulationType.PRISMATIC
        and j_spout.motion_limits is not None
        and abs(j_spout.motion_limits.upper - SPOUT_TRAVEL) < 1e-9
        and abs(j_spout.axis[2]) > 0.99,
        details=f"axis={j_spout.axis}",
    )
    ctx.check(
        "tray joint is a 0.12 m forward prismatic slide",
        j_tray.articulation_type == ArticulationType.PRISMATIC
        and j_tray.motion_limits is not None
        and abs(j_tray.motion_limits.upper - TRAY_TRAVEL) < 1e-9
        and j_tray.axis[0] > 0.99,
        details=f"axis={j_tray.axis}",
    )
    ctx.check(
        "wand joint is a ~60 deg revolute about a vertical axis",
        j_wand.articulation_type == ArticulationType.REVOLUTE
        and j_wand.motion_limits is not None
        and abs(j_wand.motion_limits.upper - WAND_SWING) < 1e-6
        and abs(j_wand.axis[2]) > 0.99,
        details=f"axis={j_wand.axis}",
    )
    ctx.check(
        "hopper canister joint is a vertical prismatic lift with 0.10 m travel",
        j_hopper.articulation_type == ArticulationType.PRISMATIC
        and j_hopper.motion_limits is not None
        and abs(j_hopper.motion_limits.upper - HOPPER_TRAVEL) < 1e-9
        and abs(j_hopper.axis[2]) > 0.99,
        details=f"axis={j_hopper.axis}",
    )

    # ---- seating / fit ------------------------------------------------------
    # Canister sits within the guide bay at rest (xy centering).
    ctx.expect_within(
        canister,
        body,
        axes="xy",
        inner_elem="canister_shell",
        outer_elem="hopper_bay_floor",
        margin=0.008,
        name="hopper canister fits within the guide bay footprint",
    )
    # Canister bottom seats near the bay floor at rest.
    ctx.expect_gap(
        canister,
        body,
        axis="z",
        positive_elem="canister_shell",
        negative_elem="hopper_bay_floor",
        max_penetration=0.006,
        name="hopper canister seats on the bay floor at rest",
    )
    ctx.expect_within(
        spout,
        body,
        axes="y",
        name="spout block stays within the front recess width",
    )
    ctx.expect_within(
        tray,
        body,
        axes="y",
        name="drip tray stays within the body width",
    )
    ctx.expect_overlap(
        spout,
        body,
        axes="z",
        elem_a="spout_housing",
        elem_b="spout_rail",
        min_overlap=0.05,
        name="spout carriage engages the guide rail at rest",
    )
    ctx.expect_overlap(
        dial,
        body,
        axes="x",
        elem_a="dial_stem",
        elem_b="fascia_panel",
        min_overlap=0.003,
        name="dial shaft remains inserted in the fascia",
    )
    ctx.expect_overlap(
        wand,
        body,
        axes="y",
        elem_a="pivot_knuckle",
        elem_b="wand_boss",
        min_overlap=0.002,
        name="wand knuckle remains captured on the pivot boss",
    )

    # ---- articulated poses ---------------------------------------------------
    spout_rest = ctx.part_world_aabb(spout)
    with ctx.pose({j_spout: SPOUT_TRAVEL}):
        spout_low = ctx.part_world_aabb(spout)
        ctx.expect_overlap(
            spout,
            body,
            axes="z",
            elem_a="spout_housing",
            elem_b="spout_rail",
            min_overlap=0.03,
            name="lowered spout carriage retains rail engagement",
        )
        ctx.expect_gap(
            spout,
            tray,
            axis="z",
            min_gap=0.01,
            name="lowered nozzles clear the drip plate",
        )
    ctx.check(
        "positive spout q lowers the dispenser by 0.06 m",
        spout_rest is not None
        and spout_low is not None
        and abs((spout_rest[0][2] - spout_low[0][2]) - SPOUT_TRAVEL) < 1e-6,
        details=f"rest={spout_rest}, lowered={spout_low}",
    )

    tray_rest = ctx.part_world_aabb(tray)
    with ctx.pose({j_tray: TRAY_TRAVEL}):
        tray_out = ctx.part_world_aabb(tray)
    ctx.check(
        "positive tray q slides the tray 0.12 m forward",
        tray_rest is not None
        and tray_out is not None
        and abs((tray_out[1][0] - tray_rest[1][0]) - TRAY_TRAVEL) < 1e-6,
        details=f"rest={tray_rest}, out={tray_out}",
    )

    # Hopper canister lifts vertically out of the bay.
    hopper_rest = ctx.part_world_aabb(canister)
    with ctx.pose({j_hopper: HOPPER_TRAVEL}):
        hopper_lifted = ctx.part_world_aabb(canister)
    ctx.check(
        "positive hopper q lifts the canister 0.10 m upward",
        hopper_rest is not None
        and hopper_lifted is not None
        and abs((hopper_lifted[0][2] - hopper_rest[0][2]) - HOPPER_TRAVEL) < 1e-6,
        details=f"rest={hopper_rest}, lifted={hopper_lifted}",
    )
    # At max lift, canister should clear the guide walls.
    with ctx.pose({j_hopper: HOPPER_TRAVEL}):
        ctx.expect_gap(
            canister,
            body,
            axis="z",
            positive_elem="canister_shell",
            negative_elem="hopper_guide_front",
            min_gap=0.01,
            name="fully lifted canister clears the guide walls",
        )

    wand_rest = ctx.part_world_aabb(wand)
    with ctx.pose({j_wand: WAND_SWING}):
        wand_swung = ctx.part_world_aabb(wand)
    ctx.check(
        "positive wand q swings the wand tip forward",
        wand_rest is not None
        and wand_swung is not None
        and wand_swung[1][0] > wand_rest[1][0] + 0.03,
        details=f"rest={wand_rest}, swung={wand_swung}",
    )

    # Off-axis pointer proves the dial actually spins about the fascia normal.
    ptr_rest = ctx.part_element_world_aabb(dial, elem="dial_pointer")
    with ctx.pose({j_dial: math.pi}):
        ptr_half = ctx.part_element_world_aabb(dial, elem="dial_pointer")
    ctx.check(
        "dial pointer sweeps to the opposite side after a half turn",
        ptr_rest is not None
        and ptr_half is not None
        and (ptr_rest[0][1] + ptr_rest[1][1]) / 2.0
        - (ptr_half[0][1] + ptr_half[1][1]) / 2.0
        > 0.02,
        details=f"rest={ptr_rest}, half_turn={ptr_half}",
    )

    # ---- canister visible geometry checks ------------------------------------
    # Confirm the canister shell is a CadQuery mesh (hollow container shape).
    shell_vis = canister.get_visual("canister_shell")
    ctx.check(
        "hopper canister has a mesh-backed shell visual",
        shell_vis is not None and shell_vis.geometry is not None,
    )
    # Confirm grip handle exists on top.
    grip_vis = canister.get_visual("canister_grip")
    ctx.check(
        "hopper canister has a visible grip handle",
        grip_vis is not None,
    )
    # Confirm guide walls are present on the body.
    guide_front = body.get_visual("hopper_guide_front")
    guide_rear = body.get_visual("hopper_guide_rear")
    ctx.check(
        "hopper guide bay walls exist on the body",
        guide_front is not None and guide_rear is not None,
    )

    return ctx.report()


object_model = build_object_model()
