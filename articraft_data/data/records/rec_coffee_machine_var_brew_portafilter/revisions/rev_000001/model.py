from __future__ import annotations

"""Espresso portafilter machine (portafilter-style brew mechanism).

Derived from super-automatic base.  Replaced bean hopper and dual dispenser
with a cylindrical brew group and removable portafilter.

True scale: ~0.43 m tall, 0.24 m wide, 0.35 m deep, grounded at z=0.
Frame: +X = front, +Y = machine left, +Z = up.

Articulations:
  1. selection dial   - CONTINUOUS, axis normal to the 15-degree tilted fascia
  2. drip tray        - PRISMATIC, slides forward +X, 0.12 m travel
  3. steam wand       - REVOLUTE about vertical axis at upper mount, ~60 deg
  4. group head       - FIXED to body (structural brew group mount)
  5. portafilter      - FIXED, docked under the group head
"""

import math

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
    mesh_from_geometry,
)

# ---------------------------------------------------------------- dimensions
WIDTH = 0.24           # Y extent
DEPTH = 0.35           # X extent of the main body
HEIGHT = 0.43          # Z extent
X_FRONT = DEPTH / 2.0  # +0.175
X_REAR = -DEPTH / 2.0  # -0.175

TILT = math.radians(15.0)  # fascia lean-back angle
FASCIA_TH = 0.025
FASCIA_H = 0.155
_N = (math.cos(TILT), 0.0, math.sin(TILT))
_H = (-math.sin(TILT), 0.0, math.cos(TILT))
FASCIA_C = (
    X_FRONT - (FASCIA_H / 2.0) * math.sin(TILT) - (FASCIA_TH / 2.0) * math.cos(TILT),
    0.0,
    0.262 + (FASCIA_H / 2.0) * math.cos(TILT) - (FASCIA_TH / 2.0) * math.sin(TILT),
)

TRAY_TRAVEL = 0.12
WAND_SWING = math.radians(60.0)

# Group head placement (part origin = bottom of main cylinder body)
GH_X = 0.155
GH_Y = 0.0
GH_Z = 0.185          # z of group-head part origin
GH_BODY_H = 0.055
GH_FLANGE_H = 0.010
GH_LOCK_H = 0.006

# Portafilter placement (part origin = top of rim, docked under group head)
PF_Z = 0.176


def fascia_point(u: float, y: float, proud: float) -> tuple[float, float, float]:
    """Point on (or proud of) the fascia outer surface; u runs up the panel."""
    return (
        FASCIA_C[0] + (FASCIA_TH / 2.0 + proud) * _N[0] + u * _H[0],
        y,
        FASCIA_C[2] + (FASCIA_TH / 2.0 + proud) * _N[2] + u * _H[2],
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portafilter_espresso_machine")

    model.material("gloss_black", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("graphite", rgba=(0.17, 0.17, 0.185, 1.0))
    model.material("fascia_gray", rgba=(0.55, 0.56, 0.575, 1.0))
    model.material("trim_black", rgba=(0.065, 0.065, 0.07, 1.0))
    model.material("stainless", rgba=(0.72, 0.73, 0.75, 1.0))
    model.material("silver", rgba=(0.80, 0.81, 0.83, 1.0))
    model.material("grid_dark", rgba=(0.24, 0.24, 0.255, 1.0))
    model.material("button_dark", rgba=(0.30, 0.30, 0.315, 1.0))
    model.material("brass", rgba=(0.72, 0.58, 0.28, 1.0))
    model.material("handle_black", rgba=(0.08, 0.08, 0.09, 1.0))

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
    # Gray front cheeks flanking the brew-group recess (shorter to leave the
    # portafilter clearance area open below the group head).
    for sgn, tag in ((1.0, "left"), (-1.0, "right")):
        body.visual(
            Box((0.080, 0.055, 0.085)),
            origin=Origin(xyz=(0.135, sgn * 0.0925, 0.2225)),
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
        (0.150, 0.180), 0.004,
        hole_diameter=0.005, pitch=0.011, frame=0.008, stagger=True,
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

    # Group-head mounting boss (bridges from channel wall to brew group).
    body.visual(
        Cylinder(radius=0.022, length=0.012),
        origin=Origin(xyz=(0.122, 0.0, GH_Z + 0.028), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="stainless",
        name="group_mount_boss",
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
        0.040, 0.022,
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

    # ----------------------------------------------------------- group head
    gh = model.part("group_head")
    # Bottom lock ring (portafilter tabs engage here).
    gh.visual(
        Cylinder(radius=0.034, length=GH_LOCK_H),
        origin=Origin(xyz=(0.0, 0.0, -GH_LOCK_H / 2.0)),
        material="brass",
        name="gh_lock_ring",
    )
    # Main cylindrical brew body.
    gh.visual(
        Cylinder(radius=0.030, length=GH_BODY_H),
        origin=Origin(xyz=(0.0, 0.0, GH_BODY_H / 2.0)),
        material="brass",
        name="gh_body",
    )
    # Top mounting flange (wider ring).
    gh.visual(
        Cylinder(radius=0.038, length=GH_FLANGE_H),
        origin=Origin(xyz=(0.0, 0.0, GH_BODY_H + GH_FLANGE_H / 2.0)),
        material="brass",
        name="gh_flange",
    )
    # FIXED joint: group head is structural, mounted in the front recess.
    model.articulation(
        "body_to_group_head",
        ArticulationType.FIXED,
        parent=body,
        child=gh,
        origin=Origin(xyz=(GH_X, GH_Y, GH_Z)),
    )

    # --------------------------------------------------------- portafilter
    pf = model.part("portafilter")
    # Filter basket (cylindrical, hangs below the rim).
    pf.visual(
        Cylinder(radius=0.029, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material="stainless",
        name="pf_basket",
    )
    # Rim collar at top of basket (docks into group head).
    pf.visual(
        Cylinder(radius=0.035, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material="stainless",
        name="pf_rim",
    )
    # Locking ears (two tabs on opposite sides of rim, engage group slots).
    for i in range(2):
        y_sign = 1.0 if i == 0 else -1.0
        pf.visual(
            Box((0.012, 0.010, 0.005)),
            origin=Origin(xyz=(0.0, y_sign * 0.034, 0.004)),
            material="stainless",
            name=f"pf_ear_{i}",
        )
    # Handle lug (transition block from basket to handle shaft).
    pf.visual(
        Box((0.016, 0.024, 0.018)),
        origin=Origin(xyz=(0.0, -0.035, -0.009)),
        material="stainless",
        name="pf_handle_lug",
    )
    # Handle shaft (cylindrical rod extending outward along -Y).
    pf.visual(
        Cylinder(radius=0.006, length=0.090),
        origin=Origin(xyz=(0.0, -0.080, -0.012), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="stainless",
        name="pf_handle_shaft",
    )
    # Handle grip (wider black sleeve at the end).
    pf.visual(
        Cylinder(radius=0.010, length=0.045),
        origin=Origin(xyz=(0.0, -0.130, -0.016), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="handle_black",
        name="pf_handle_grip",
    )
    # FIXED joint: portafilter docked under the group head.
    model.articulation(
        "body_to_portafilter",
        ArticulationType.FIXED,
        parent=body,
        child=pf,
        origin=Origin(xyz=(GH_X, GH_Y, PF_Z)),
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
        (0.150, 0.200), 0.004,
        hole_diameter=0.007, pitch=0.016, frame=0.010, stagger=True,
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

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    dial = object_model.get_part("selection_dial")
    gh = object_model.get_part("group_head")
    pf = object_model.get_part("portafilter")
    tray = object_model.get_part("drip_tray")
    wand = object_model.get_part("steam_wand")

    j_dial = object_model.get_articulation("fascia_to_dial")
    j_gh = object_model.get_articulation("body_to_group_head")
    j_pf = object_model.get_articulation("body_to_portafilter")
    j_tray = object_model.get_articulation("body_to_tray")
    j_wand = object_model.get_articulation("body_to_wand")

    # ---- verify conversion: removed super-auto parts --------------------
    all_part_names = {p.name for p in object_model.parts}
    ctx.check(
        "bean hopper lid removed (portafilter conversion)",
        "hopper_lid" not in all_part_names,
        details=f"parts: {sorted(all_part_names)}",
    )
    ctx.check(
        "spout block removed (portafilter conversion)",
        "spout_block" not in all_part_names,
        details=f"parts: {sorted(all_part_names)}",
    )
    ctx.check(
        "group head present (portafilter conversion)",
        "group_head" in all_part_names,
    )
    ctx.check(
        "portafilter present (portafilter conversion)",
        "portafilter" in all_part_names,
    )

    # ---- intentional overlap allowances ---------------------------------
    ctx.allow_overlap(
        dial, body,
        elem_a="dial_stem", elem_b="fascia_panel",
        reason="Dial shaft is intentionally inserted through the fascia panel.",
    )
    ctx.allow_overlap(
        wand, body,
        elem_a="pivot_knuckle", elem_b="wand_boss",
        reason="Wand knuckle is intentionally captured on the side pivot boss.",
    )
    ctx.allow_overlap(
        gh, body,
        elem_a="gh_body", elem_b="group_mount_boss",
        reason="Group head cylinder seats against the body mounting boss.",
    )
    # Portafilter docking interface: rim and ears engage the group lock ring.
    ctx.allow_overlap(
        pf, gh,
        elem_a="pf_rim", elem_b="gh_lock_ring",
        reason="Portafilter rim collar docks into the group head lock ring.",
    )
    for ear_name in ("pf_ear_0", "pf_ear_1"):
        ctx.allow_overlap(
            pf, gh,
            elem_a=ear_name, elem_b="gh_lock_ring",
            reason="Portafilter locking ear engages the group head lock ring slot.",
        )

    # ---- overall scale and grounding ------------------------------------
    body_aabb = ctx.part_world_aabb(body)
    core_aabb = ctx.part_element_world_aabb(body, elem="core_shell")
    ctx.check(
        "body footprint matches real machine scale",
        body_aabb is not None
        and core_aabb is not None
        and abs((core_aabb[1][1] - core_aabb[0][1]) - WIDTH) < 0.01
        and abs((body_aabb[1][0] - body_aabb[0][0]) - DEPTH) < 0.02,
        details=f"body aabb={body_aabb}, core aabb={core_aabb}",
    )
    tray_aabb = ctx.part_world_aabb(tray)
    ctx.check(
        "machine grounded at z=0",
        body_aabb is not None
        and tray_aabb is not None
        and abs(body_aabb[0][2]) < 1e-6
        and abs(tray_aabb[0][2]) < 1e-6,
        details=f"body zmin={body_aabb}, tray zmin={tray_aabb}",
    )
    ctx.check(
        "overall height about 0.43 m",
        body_aabb is not None and 0.41 < body_aabb[1][2] < 0.44,
        details=f"body aabb={body_aabb}",
    )

    # ---- joint plan metadata ---------------------------------------------
    ctx.check(
        "dial joint is continuous and unbounded",
        j_dial.articulation_type == ArticulationType.CONTINUOUS
        and (j_dial.motion_limits is None or j_dial.motion_limits.upper is None),
    )
    ctx.check(
        "group head joint is FIXED",
        j_gh.articulation_type == ArticulationType.FIXED,
    )
    ctx.check(
        "portafilter joint is FIXED",
        j_pf.articulation_type == ArticulationType.FIXED,
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

    # ---- group head and portafilter visible geometry --------------------
    gh_aabb = ctx.part_world_aabb(gh)
    pf_aabb = ctx.part_world_aabb(pf)

    ctx.check(
        "group head positioned in front recess area",
        gh_aabb is not None
        and gh_aabb[0][0] > 0.10   # forward of the channel back wall
        and gh_aabb[1][0] < 0.20   # not far past the front cheeks
        and abs(gh_aabb[0][1] + gh_aabb[1][1]) < 0.02,  # centered in Y
        details=f"gh aabb={gh_aabb}",
    )
    ctx.check(
        "portafilter docked under group head",
        gh_aabb is not None and pf_aabb is not None
        and abs(gh_aabb[0][2] - pf_aabb[1][2]) < 0.015,
        details=f"gh bottom z={gh_aabb[0][2] if gh_aabb else None}, "
                f"pf top z={pf_aabb[1][2] if pf_aabb else None}",
    )
    ctx.check(
        "portafilter handle extends visibly to one side",
        pf_aabb is not None
        and (pf_aabb[1][1] - pf_aabb[0][1]) > 0.08,
        details=f"pf Y extent={pf_aabb[1][1] - pf_aabb[0][1] if pf_aabb else None}",
    )
    ctx.check(
        "group head is vertically cylindrical (taller than wide in XZ)",
        gh_aabb is not None
        and (gh_aabb[1][2] - gh_aabb[0][2]) > 0.04
        and (gh_aabb[1][0] - gh_aabb[0][0]) < 0.09,
        details=f"gh dims x={gh_aabb[1][0] - gh_aabb[0][0]}, "
                f"z={gh_aabb[1][2] - gh_aabb[0][2]}",
    )

    # ---- seating / fit ---------------------------------------------------
    ctx.expect_within(
        tray, body,
        axes="y",
        name="drip tray stays within the body width",
    )
    ctx.expect_overlap(
        dial, body,
        axes="x",
        elem_a="dial_stem", elem_b="fascia_panel",
        min_overlap=0.003,
        name="dial shaft remains inserted in the fascia",
    )
    ctx.expect_overlap(
        wand, body,
        axes="y",
        elem_a="pivot_knuckle", elem_b="wand_boss",
        min_overlap=0.002,
        name="wand knuckle remains captured on the pivot boss",
    )
    ctx.expect_overlap(
        gh, body,
        axes="x",
        elem_a="gh_body", elem_b="group_mount_boss",
        min_overlap=0.002,
        name="group head engaged with mounting boss in X",
    )
    ctx.expect_overlap(
        pf, gh,
        axes="z",
        elem_a="pf_rim", elem_b="gh_lock_ring",
        min_overlap=0.002,
        name="portafilter rim engaged with group head lock ring in Z",
    )
    ctx.expect_overlap(
        pf, gh,
        axes="xy",
        elem_a="pf_basket", elem_b="gh_lock_ring",
        min_overlap=0.020,
        name="portafilter basket aligned under group head in XY",
    )

    # ---- articulated poses -----------------------------------------------
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

    return ctx.report()


object_model = build_object_model()
