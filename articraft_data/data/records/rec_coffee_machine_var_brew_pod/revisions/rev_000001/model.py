from __future__ import annotations

"""Pod/capsule coffee machine (Nespresso-style).

True scale: ~0.31 m tall, 0.14 m wide, 0.28 m deep, grounded at z=0.
Frame: +X = front, +Y = machine left, +Z = up.

Articulations:
  1. capsule_flap   - REVOLUTE, rear-edge hinge on top, opens ~100 deg
  2. drip_tray      - PRISMATIC, slides forward +X, 0.08 m travel
  3. control_dial   - CONTINUOUS, rotary dial on front face
"""

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    KnobGeometry,
    KnobIndicator,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

# ---------------------------------------------------------------- dimensions
WIDTH = 0.14            # Y extent
DEPTH = 0.28            # X extent of the main body
HEIGHT = 0.30           # Z extent of the body shell
X_FRONT = DEPTH / 2.0   # +0.14
X_REAR = -DEPTH / 2.0   # -0.14

# Top deck sits on the shell top, adding 6mm proud surface
DECK_TH = 0.012
DECK_TOP = HEIGHT + DECK_TH / 2.0  # 0.306

TRAY_TRAVEL = 0.08
FLAP_OPEN = math.radians(100.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pod_coffee_machine")

    # Materials
    model.material("body_dark", rgba=(0.12, 0.12, 0.14, 1.0))
    model.material("body_accent", rgba=(0.18, 0.18, 0.20, 1.0))
    model.material("chrome", rgba=(0.78, 0.80, 0.82, 1.0))
    model.material("silver_ring", rgba=(0.82, 0.83, 0.85, 1.0))
    model.material("trim_black", rgba=(0.06, 0.06, 0.07, 1.0))
    model.material("stainless", rgba=(0.72, 0.73, 0.75, 1.0))
    model.material("water_tank", rgba=(0.35, 0.45, 0.55, 0.6))

    # ------------------------------------------------------------- body root
    body = model.part("body")

    # Main body shell - rounded rectangle extruded vertically
    body_profile = rounded_rect_profile(DEPTH, WIDTH, 0.025, corner_segments=8)
    body_shell = ExtrudeGeometry.from_z0(body_profile, HEIGHT, cap=True)
    body.visual(
        mesh_from_geometry(body_shell, "body_shell"),
        material="body_dark",
        name="body_shell",
    )

    # Top deck - sits on the shell top surface, extends slightly proud
    body.visual(
        Box((DEPTH - 0.04, WIDTH - 0.02, DECK_TH)),
        origin=Origin(xyz=(0.0, 0.0, HEIGHT)),
        material="body_accent",
        name="top_deck",
    )

    # Front fascia panel - slightly proud of the body shell front face
    body.visual(
        Box((0.008, WIDTH - 0.02, 0.18)),
        origin=Origin(xyz=(X_FRONT - 0.001, 0.0, 0.16)),
        material="body_accent",
        name="front_fascia",
    )

    # Dispensing recess - dark alcove, touches body shell front face
    body.visual(
        Box((0.05, 0.08, 0.10)),
        origin=Origin(xyz=(X_FRONT - 0.025, 0.0, 0.13)),
        material="trim_black",
        name="dispensing_recess",
    )

    # Single chrome spout nozzle - horizontal cylinder protruding from recess
    body.visual(
        Cylinder(radius=0.008, length=0.030),
        origin=Origin(xyz=(X_FRONT - 0.01, 0.0, 0.10), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="chrome",
        name="single_spout",
    )
    # Spout tip - narrower, extends past the front face
    body.visual(
        Cylinder(radius=0.005, length=0.016),
        origin=Origin(xyz=(X_FRONT + 0.01, 0.0, 0.10), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="chrome",
        name="spout_tip",
    )

    # Capsule slot opening on top (dark recessed slot, under the flap when closed)
    body.visual(
        Box((0.05, 0.04, 0.006)),
        origin=Origin(xyz=(-0.06, 0.0, DECK_TOP + 0.001)),
        material="trim_black",
        name="capsule_slot",
    )

    # Water tank at rear - extends to touch the body shell rear face
    body.visual(
        Box((0.07, WIDTH - 0.03, 0.20)),
        origin=Origin(xyz=(X_REAR + 0.035, 0.0, 0.14)),
        material="water_tank",
        name="water_tank",
    )

    # Base platform - slightly wider for stability look
    body.visual(
        Box((DEPTH + 0.01, WIDTH + 0.01, 0.015)),
        origin=Origin(xyz=(0.0, 0.0, 0.0075)),
        material="body_accent",
        name="base_platform",
    )

    # Tray guide rails on the base platform
    for i in range(2):
        sgn = 1.0 if i == 0 else -1.0
        tag = "left" if i == 0 else "right"
        body.visual(
            Box((0.12, 0.005, 0.010)),
            origin=Origin(xyz=(0.05, sgn * (WIDTH / 2.0 - 0.015), 0.020)),
            material="body_accent",
            name=f"tray_guide_{i}",
        )

    # Flap hinge boss at rear top edge - protrudes from the body shell
    body.visual(
        Cylinder(radius=0.007, length=WIDTH - 0.02),
        origin=Origin(xyz=(X_REAR, 0.0, DECK_TOP), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="body_dark",
        name="flap_hinge_boss",
    )

    # --------------------------------------------------------- control dial
    dial = model.part("control_dial")
    # Dial stem inserts through the fascia and body shell
    dial.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material="trim_black",
        name="dial_stem",
    )
    # Metallic ring around the dial
    dial.visual(
        Cylinder(radius=0.020, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material="silver_ring",
        name="dial_ring",
    )
    # Knob cap
    knob = KnobGeometry(
        0.030,
        0.015,
        body_style="cylindrical",
        edge_radius=0.002,
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=90.0),
        center=False,
    )
    dial.visual(
        mesh_from_geometry(knob, "dial_knob"),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material="chrome",
        name="dial_knob",
    )
    # Pointer mark on knob face
    dial.visual(
        Box((0.003, 0.002, 0.002)),
        origin=Origin(xyz=(0.0, 0.010, 0.021)),
        material="trim_black",
        name="dial_pointer",
    )

    # Dial articulation: continuous rotation about the front face normal (+X)
    # rpy=(0, pi/2, 0) maps dial local Z to world +X (outward from front face)
    # axis=(0,0,1) in the rotated frame = rotation around world +X
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(X_FRONT + 0.001, 0.0, 0.20), rpy=(0.0, math.pi / 2.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0),
    )

    # ----------------------------------------------------------- capsule flap
    flap = model.part("capsule_flap")

    # Flap plate - covers the capsule slot, hinged at rear edge
    # Plate bottom sits on the top deck surface when closed
    flap.visual(
        Box((0.14, WIDTH - 0.04, 0.008)),
        origin=Origin(xyz=(0.07, 0.0, 0.004)),
        material="body_accent",
        name="flap_plate",
    )
    # Grip ridge on front edge of flap
    flap.visual(
        Box((0.012, 0.05, 0.006)),
        origin=Origin(xyz=(0.135, 0.0, 0.008)),
        material="chrome",
        name="flap_grip",
    )
    # Hinge pin - at the articulation origin (local 0,0,0)
    flap.visual(
        Cylinder(radius=0.004, length=WIDTH - 0.04),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="chrome",
        name="flap_hinge_pin",
    )

    # Flap articulation: REVOLUTE at rear top edge
    # Plate extends along +X from the hinge line.
    # axis = -Y so positive q opens the flap upward (rotates +X edge toward +Z)
    model.articulation(
        "body_to_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flap,
        origin=Origin(xyz=(X_REAR, 0.0, DECK_TOP)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=FLAP_OPEN),
    )

    # ------------------------------------------------------------- drip tray
    tray = model.part("drip_tray")

    # Tray floor - sits on the base platform
    tray.visual(
        Box((0.11, WIDTH - 0.04, 0.006)),
        origin=Origin(xyz=(0.055, 0.0, 0.003)),
        material="trim_black",
        name="tray_floor",
    )
    # Front wall
    tray.visual(
        Box((0.008, WIDTH - 0.04, 0.035)),
        origin=Origin(xyz=(0.106, 0.0, 0.020)),
        material="trim_black",
        name="tray_front_wall",
    )
    # Rear wall
    tray.visual(
        Box((0.008, WIDTH - 0.04, 0.035)),
        origin=Origin(xyz=(0.004, 0.0, 0.020)),
        material="trim_black",
        name="tray_rear_wall",
    )
    # Side walls
    for i in range(2):
        sgn = 1.0 if i == 0 else -1.0
        tag = "left" if i == 0 else "right"
        tray.visual(
            Box((0.105, 0.006, 0.035)),
            origin=Origin(xyz=(0.054, sgn * (WIDTH / 2.0 - 0.023), 0.020)),
            material="trim_black",
            name=f"tray_{tag}_wall_{i}",
        )
    # Perforated drip plate on top
    drip_plate = PerforatedPanelGeometry(
        (0.100, WIDTH - 0.06),
        0.003,
        hole_diameter=0.005,
        pitch=0.012,
        frame=0.006,
        stagger=True,
    )
    tray.visual(
        mesh_from_geometry(drip_plate, "drip_plate"),
        origin=Origin(xyz=(0.054, 0.0, 0.038)),
        material="stainless",
        name="drip_plate",
    )
    # Chrome front lip/pull handle
    tray.visual(
        Box((0.006, 0.06, 0.020)),
        origin=Origin(xyz=(0.113, 0.0, 0.028)),
        material="chrome",
        name="tray_pull_handle",
    )

    # Tray articulation: PRISMATIC, slides forward (+X) for emptying
    # Tray origin at the base platform top (z=0.015)
    model.articulation(
        "body_to_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(X_FRONT - 0.12, 0.0, 0.015)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=0.15, lower=0.0, upper=TRAY_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    dial = object_model.get_part("control_dial")
    flap = object_model.get_part("capsule_flap")
    tray = object_model.get_part("drip_tray")

    j_dial = object_model.get_articulation("body_to_dial")
    j_flap = object_model.get_articulation("body_to_flap")
    j_tray = object_model.get_articulation("body_to_tray")

    # Intentional local embeddings
    ctx.allow_overlap(
        dial,
        body,
        elem_a="dial_stem",
        elem_b="front_fascia",
        reason="Dial stem is intentionally inserted through the front fascia panel.",
    )
    ctx.allow_overlap(
        dial,
        body,
        elem_a="dial_stem",
        elem_b="body_shell",
        reason="Dial stem is intentionally inserted through the body shell wall.",
    )
    ctx.allow_overlap(
        flap,
        body,
        elem_a="flap_hinge_pin",
        elem_b="flap_hinge_boss",
        reason="Flap hinge pin is intentionally captured in the body hinge boss.",
    )
    ctx.allow_overlap(
        flap,
        body,
        elem_a="flap_plate",
        elem_b="flap_hinge_boss",
        reason="Flap plate wraps around the hinge boss at the rear edge, representing a captured hinge barrel.",
    )

    # ---- overall scale and grounding -------------------------------------
    body_aabb = ctx.part_world_aabb(body)
    tray_aabb = ctx.part_world_aabb(tray)
    shell_aabb = ctx.part_element_world_aabb(body, elem="body_shell")

    ctx.check(
        "machine grounded at z=0",
        body_aabb is not None
        and tray_aabb is not None
        and abs(body_aabb[0][2]) < 1e-6
        and tray_aabb[0][2] >= -1e-6,
        details=f"body zmin={body_aabb}, tray zmin={tray_aabb}",
    )
    ctx.check(
        "overall height approximately 0.30 m (pod machine body shell)",
        shell_aabb is not None
        and abs(shell_aabb[1][2] - HEIGHT) < 0.01,
        details=f"shell aabb={shell_aabb}",
    )
    ctx.check(
        "body width approximately 0.14 m (compact pod machine)",
        shell_aabb is not None
        and abs((shell_aabb[1][1] - shell_aabb[0][1]) - WIDTH) < 0.015,
        details=f"shell aabb={shell_aabb}",
    )
    ctx.check(
        "body depth approximately 0.28 m",
        shell_aabb is not None
        and abs((shell_aabb[1][0] - shell_aabb[0][0]) - DEPTH) < 0.015,
        details=f"shell aabb={shell_aabb}",
    )

    # ---- joint plan metadata ----------------------------------------------
    ctx.check(
        "dial joint is continuous and unbounded",
        j_dial.articulation_type == ArticulationType.CONTINUOUS
        and (j_dial.motion_limits is None or j_dial.motion_limits.upper is None),
    )
    ctx.check(
        "flap joint is a ~100 deg revolute rear hinge",
        j_flap.articulation_type == ArticulationType.REVOLUTE
        and j_flap.motion_limits is not None
        and abs(j_flap.motion_limits.upper - FLAP_OPEN) < 1e-6
        and abs(j_flap.axis[1]) > 0.99,
        details=f"axis={j_flap.axis}",
    )
    ctx.check(
        "tray joint is a 0.08 m forward prismatic slide",
        j_tray.articulation_type == ArticulationType.PRISMATIC
        and j_tray.motion_limits is not None
        and abs(j_tray.motion_limits.upper - TRAY_TRAVEL) < 1e-9
        and j_tray.axis[0] > 0.99,
        details=f"axis={j_tray.axis}",
    )

    # ---- seating / fit ------------------------------------------------------
    ctx.expect_contact(
        flap,
        body,
        elem_a="flap_plate",
        elem_b="top_deck",
        contact_tol=2e-3,
        name="closed capsule flap seats on the top deck",
    )
    ctx.expect_within(
        tray,
        body,
        axes="y",
        name="drip tray stays within the body width",
    )
    ctx.expect_overlap(
        dial,
        body,
        axes="x",
        elem_a="dial_stem",
        elem_b="front_fascia",
        min_overlap=0.002,
        name="dial shaft remains inserted in the fascia",
    )
    ctx.expect_overlap(
        flap,
        body,
        axes="y",
        elem_a="flap_hinge_pin",
        elem_b="flap_hinge_boss",
        min_overlap=0.002,
        name="flap hinge pin remains captured in the boss",
    )
    ctx.expect_contact(
        flap,
        body,
        elem_a="flap_plate",
        elem_b="flap_hinge_boss",
        contact_tol=0.010,
        name="flap plate rear edge contacts hinge boss at the pivot",
    )

    # ---- visible geometry claims --------------------------------------------
    spout_vis = body.get_visual("single_spout")
    ctx.check(
        "single spout nozzle present on body",
        spout_vis is not None,
        details="Expected 'single_spout' visual on body part",
    )

    slot_vis = body.get_visual("capsule_slot")
    ctx.check(
        "capsule slot opening present on top",
        slot_vis is not None,
        details="Expected 'capsule_slot' visual on body part",
    )

    tank_vis = body.get_visual("water_tank")
    ctx.check(
        "water tank present at rear",
        tank_vis is not None,
        details="Expected 'water_tank' visual on body part",
    )

    # ---- articulated poses ---------------------------------------------------
    # Flap opens upward
    flap_rest = ctx.part_world_aabb(flap)
    with ctx.pose({j_flap: 1.4}):
        flap_open = ctx.part_world_aabb(flap)
    ctx.check(
        "opening the capsule flap raises its free edge",
        flap_rest is not None
        and flap_open is not None
        and flap_open[1][2] > flap_rest[1][2] + 0.03,
        details=f"rest={flap_rest}, open={flap_open}",
    )

    # Tray slides forward
    tray_rest = ctx.part_world_aabb(tray)
    with ctx.pose({j_tray: TRAY_TRAVEL}):
        tray_out = ctx.part_world_aabb(tray)
    ctx.check(
        "positive tray q slides the tray 0.08 m forward",
        tray_rest is not None
        and tray_out is not None
        and abs((tray_out[1][0] - tray_rest[1][0]) - TRAY_TRAVEL) < 1e-6,
        details=f"rest={tray_rest}, out={tray_out}",
    )

    # Dial pointer sweeps
    ptr_rest = ctx.part_element_world_aabb(dial, elem="dial_pointer")
    with ctx.pose({j_dial: math.pi}):
        ptr_half = ctx.part_element_world_aabb(dial, elem="dial_pointer")
    ctx.check(
        "dial pointer sweeps to the opposite side after a half turn",
        ptr_rest is not None
        and ptr_half is not None
        and abs(
            (ptr_rest[0][1] + ptr_rest[1][1]) / 2.0
            - (ptr_half[0][1] + ptr_half[1][1]) / 2.0
        )
        > 0.01,
        details=f"rest={ptr_rest}, half_turn={ptr_half}",
    )

    return ctx.report()


object_model = build_object_model()
