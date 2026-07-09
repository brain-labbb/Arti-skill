from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Single-hole basin faucet — tall straight tower, ~0.17 m, mirror chrome.
# World frame: +Z up, deck at z = 0, spout points toward +X (front).
# Body is strictly VERTICAL (no tilt).
# ---------------------------------------------------------------------------

# Base flange (sits flat on the deck).
FLANGE_R = 0.030
FLANGE_H = 0.006

# Main body barrel — straight vertical tower.
BODY_R = 0.022
BODY_Z0 = 0.006
BODY_Z1 = 0.130

# Decorative groove ring around the upper third.
GROOVE_R = 0.019
GROOVE_Z0 = 0.092
GROOVE_Z1 = 0.097

# Stepped-in upper neck above the groove.
NECK_R = 0.020
NECK_Z0 = 0.097
NECK_Z1 = 0.150

# Spout swivel collar dimensions (ring that wraps around the body).
COLLAR_R_OUT = 0.026
COLLAR_R_IN = 0.0225
COLLAR_H = 0.014
COLLAR_Z_CENTER = 0.105

# Spout tube.
SPOUT_R = 0.011
SPOUT_ARM_START_X = 0.024  # start inside collar wall for clean union
SPOUT_ARM_STRAIGHT = 0.025
SPOUT_BEND_R = 0.018

# Spout end position in spout-local frame.
SPOUT_END_X = SPOUT_ARM_START_X + SPOUT_ARM_STRAIGHT + SPOUT_BEND_R
SPOUT_END_Z = -SPOUT_BEND_R

# Valve stem (prismatic press carrier nested in the neck bore).
STEM_R = 0.010
STEM_Z0 = 0.140
STEM_Z1 = 0.160
STEM_LEN = STEM_Z1 - STEM_Z0

# Push cap (press-to-run button, also rotates for temperature).
CAP_R = 0.028
CAP_DISC_H = 0.012
CAP_FLARE_H = 0.004
CAP_FLARE_R0 = 0.014
CAP_Z = 0.158  # cap frame origin: bottom of the flat cap disc

# Aerator insert.
AERATOR_R = 0.009
AERATOR_H = 0.003

# Joint limits.
PRESS_TRAVEL = 0.008
TURN_LIMIT = math.radians(60.0)
SWIVEL_LIMIT = math.radians(90.0)


def _build_spout_shape() -> cq.Workplane:
    """Spout with swivel collar, short forward arm, downward curve, and
    hollow outlet bore.  Local origin at the swivel joint centre (body axis
    at collar height).  +X is forward, +Z is up."""
    # --- swivel collar (annular ring) ---
    collar = (
        cq.Workplane("XY")
        .circle(COLLAR_R_OUT)
        .circle(COLLAR_R_IN)
        .extrude(COLLAR_H)
        .translate((0.0, 0.0, -COLLAR_H / 2.0))
    )

    # --- spout arm path in XZ plane ---
    path = (
        cq.Workplane("XZ")
        .moveTo(SPOUT_ARM_START_X, 0.0)
        .lineTo(SPOUT_ARM_START_X + SPOUT_ARM_STRAIGHT, 0.0)
        .tangentArcPoint((SPOUT_BEND_R, -SPOUT_BEND_R), relative=True)
    )

    # Sweep tube along path.
    tube = (
        cq.Workplane("YZ", origin=(SPOUT_ARM_START_X, 0.0, 0.0))
        .circle(SPOUT_R)
        .sweep(path)
    )

    spout = collar.union(tube)

    # Small fillet ring at collar/arm junction for visual continuity.
    junction_ring = (
        cq.Workplane("XY", origin=(SPOUT_ARM_START_X + 0.004, 0.0, 0.0))
        .circle(SPOUT_R + 0.003)
        .extrude(COLLAR_H * 0.6)
        .translate((0.0, 0.0, -COLLAR_H * 0.3))
    )
    spout = spout.union(junction_ring)

    end_x = SPOUT_END_X
    end_z = SPOUT_END_Z

    # Flared outlet rim: extends well above the tube end face so the loft
    # overlaps solidly with the swept tube, producing one connected mesh.
    flare = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z + 0.010))
        .circle(SPOUT_R + 0.003)
        .workplane(offset=-0.020)
        .circle(SPOUT_R + 0.006)
        .loft()
    )
    spout = spout.union(flare)

    # --- hollow outlet bore through the spout end (cuts tube + flare) ---
    bore = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z - 0.010))
        .circle(0.0085)
        .workplane(offset=0.028)
        .circle(0.006)
        .loft()
    )
    spout = spout.cut(bore)

    return spout


def _build_cap_shape() -> cq.Workplane:
    """Push cap shell with under-flare and flat disc.
    Cap-local z=0 is the disc bottom."""
    flare = (
        cq.Workplane("XY", origin=(0.0, 0.0, -CAP_FLARE_H))
        .circle(CAP_FLARE_R0)
        .workplane(offset=CAP_FLARE_H)
        .circle(CAP_R)
        .loft()
    )
    disc = cq.Workplane("XY").circle(CAP_R).extrude(CAP_DISC_H)
    cap = disc.union(flare)
    cap = cap.edges(">Z").fillet(0.0015)
    return cap


def _build_aerator_shape() -> cq.Workplane:
    """Circular aerator insert: thin disc with recessed screen and through-holes.
    Built with z=0 at the disc bottom, extruded to z=AERATOR_H."""
    disc = cq.Workplane("XY").circle(AERATOR_R).extrude(AERATOR_H)

    # Recessed top face (screen cavity).
    recess = (
        cq.Workplane("XY", origin=(0.0, 0.0, AERATOR_H - 0.001))
        .circle(AERATOR_R - 0.001)
        .extrude(0.002)
    )
    disc = disc.cut(recess)

    # Ring of small through-holes (aerator screen pattern).
    n_holes = 10
    hole_r = 0.0008
    hole_circle_r = 0.005
    for i in range(n_holes):
        angle = 2.0 * math.pi * i / n_holes
        hx = hole_circle_r * math.cos(angle)
        hy = hole_circle_r * math.sin(angle)
        hole = (
            cq.Workplane("XY", origin=(hx, hy, -0.001))
            .circle(hole_r)
            .extrude(AERATOR_H + 0.002)
        )
        disc = disc.cut(hole)

    # Center through-hole.
    center_hole = (
        cq.Workplane("XY", origin=(0.0, 0.0, -0.001))
        .circle(0.0012)
        .extrude(AERATOR_H + 0.002)
    )
    disc = disc.cut(center_hole)

    return disc


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet_tower")

    model.material("chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    model.material("chrome_dark", rgba=(0.42, 0.44, 0.47, 1.0))
    model.material("chrome_brushed", rgba=(0.70, 0.72, 0.74, 1.0))
    model.material("index_mark", rgba=(0.30, 0.32, 0.35, 1.0))
    model.material("aerator_screen", rgba=(0.55, 0.58, 0.60, 1.0))

    # ---------------- body (root): flange + barrel + groove + neck ---------
    body = model.part("body")
    body.visual(
        Cylinder(radius=FLANGE_R, length=FLANGE_H),
        origin=Origin(xyz=(0.0, 0.0, FLANGE_H / 2.0)),
        material="chrome",
        name="base_flange",
    )
    body.visual(
        Cylinder(radius=BODY_R, length=BODY_Z1 - BODY_Z0),
        origin=Origin(xyz=(0.0, 0.0, (BODY_Z0 + BODY_Z1) / 2.0)),
        material="chrome",
        name="body_barrel",
    )
    body.visual(
        Cylinder(radius=GROOVE_R, length=GROOVE_Z1 - GROOVE_Z0),
        origin=Origin(xyz=(0.0, 0.0, (GROOVE_Z0 + GROOVE_Z1) / 2.0)),
        material="chrome_dark",
        name="groove_ring",
    )
    body.visual(
        Cylinder(radius=NECK_R, length=NECK_Z1 - NECK_Z0),
        origin=Origin(xyz=(0.0, 0.0, (NECK_Z0 + NECK_Z1) / 2.0)),
        material="chrome",
        name="body_neck",
    )

    # ---------------- spout (swivels around vertical body axis) -----------
    spout = model.part("spout")
    spout.visual(
        mesh_from_cadquery(_build_spout_shape(), "spout", tolerance=0.0003),
        material="chrome",
        name="spout_body",
    )
    # Revolute joint: spout swivels around the vertical body axis (Z).
    # Origin at the collar center on the body axis.
    # axis = (0, 0, 1), positive q swings spout toward +Y.
    model.articulation(
        "spout_swivel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, COLLAR_Z_CENTER)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=1.5, lower=-SWIVEL_LIMIT, upper=SWIVEL_LIMIT
        ),
    )

    # ---------------- aerator (fixed insert in spout mouth) ---------------
    aerator = model.part("aerator")
    aerator.visual(
        mesh_from_cadquery(_build_aerator_shape(), "aerator", tolerance=0.0003),
        material="aerator_screen",
        name="aerator_disc",
    )
    # Fixed joint: aerator seated inside the spout mouth bore.
    # Positioned above the tube end face so the disc rim overlaps with
    # the remaining tube wall (intentional press-fit).
    model.articulation(
        "aerator_seat",
        ArticulationType.FIXED,
        parent=spout,
        child=aerator,
        origin=Origin(xyz=(SPOUT_END_X, 0.0, SPOUT_END_Z + 0.002)),
    )

    # ---------------- valve stem (prismatic press carrier) ----------------
    stem = model.part("valve_stem")
    stem.visual(
        Cylinder(radius=STEM_R, length=STEM_LEN),
        origin=Origin(),
        material="chrome",
        name="stem_shaft",
    )
    # Joint frame on the body axis at the neck top; local +z is up,
    # axis -z makes positive q press DOWN toward the body.
    model.articulation(
        "cap_press",
        ArticulationType.PRISMATIC,
        parent=body,
        child=stem,
        origin=Origin(xyz=(0.0, 0.0, NECK_Z1)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=30.0, velocity=0.05, lower=0.0, upper=PRESS_TRAVEL
        ),
    )

    # ---------------- push cap (revolute temperature ring) ----------------
    cap = model.part("push_cap")
    cap.visual(
        mesh_from_cadquery(_build_cap_shape(), "push_cap", tolerance=0.0003),
        material="chrome",
        name="cap_shell",
    )
    cap.visual(
        Cylinder(radius=0.026, length=0.0025),
        origin=Origin(xyz=(0.0, 0.0, 0.01125)),
        material="chrome_brushed",
        name="cap_top_brushed",
    )
    # Temperature index mark on cap rim.
    cap.visual(
        Cylinder(radius=0.0025, length=0.0025),
        origin=Origin(xyz=(0.0275, 0.0, 0.006), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="index_mark",
        name="temp_indicator_dot",
    )
    model.articulation(
        "cap_turn",
        ArticulationType.REVOLUTE,
        parent=stem,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, CAP_Z - NECK_Z1)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=2.0, lower=-TURN_LIMIT, upper=TURN_LIMIT
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    spout = object_model.get_part("spout")
    aerator = object_model.get_part("aerator")
    stem = object_model.get_part("valve_stem")
    cap = object_model.get_part("push_cap")
    swivel = object_model.get_articulation("spout_swivel")
    press = object_model.get_articulation("cap_press")
    turn = object_model.get_articulation("cap_turn")

    # ---- intentional seated overlaps (scoped per element) ----------------
    ctx.allow_overlap(
        spout,
        body,
        elem_a="spout_body",
        elem_b="body_barrel",
        reason="Spout collar wraps around the body barrel at the swivel joint.",
    )
    ctx.allow_overlap(
        spout,
        body,
        elem_a="spout_body",
        elem_b="body_neck",
        reason="Spout arm emerges through the neck zone where the collar straddles the barrel/neck transition.",
    )
    ctx.allow_overlap(
        aerator,
        spout,
        elem_a="aerator_disc",
        elem_b="spout_body",
        reason="Aerator disc is press-fit seated inside the spout mouth bore.",
    )
    ctx.allow_overlap(
        stem,
        body,
        elem_a="stem_shaft",
        elem_b="body_neck",
        reason="Valve stem nests inside the solid neck bore proxy and slides deeper when pressed.",
    )
    ctx.allow_overlap(
        cap,
        stem,
        elem_a="cap_shell",
        elem_b="stem_shaft",
        reason="Stem top is press-fit into the cap's under-flare boss.",
    )

    # ---- hero geometry: tall straight vertical tower ---------------------
    flange_aabb = ctx.part_element_world_aabb(body, elem="base_flange")
    ctx.check(
        "base flange sits flat on the deck",
        flange_aabb is not None and abs(flange_aabb[0][2]) <= 0.0005,
        details=f"flange aabb={flange_aabb}",
    )

    neck_aabb = ctx.part_element_world_aabb(body, elem="body_neck")
    barrel_aabb = ctx.part_element_world_aabb(body, elem="body_barrel")
    ctx.check(
        "body is a straight vertical tower (neck centered over barrel in XY)",
        neck_aabb is not None
        and barrel_aabb is not None
        and abs((neck_aabb[0][0] + neck_aabb[1][0]) / 2.0 - (barrel_aabb[0][0] + barrel_aabb[1][0]) / 2.0) < 0.002
        and abs((neck_aabb[0][1] + neck_aabb[1][1]) / 2.0 - (barrel_aabb[0][1] + barrel_aabb[1][1]) / 2.0) < 0.002,
        details=f"neck={neck_aabb}, barrel={barrel_aabb}",
    )

    # ---- overall height taller than parent (0.13 m) ----------------------
    cap_aabb = ctx.part_world_aabb(cap)
    ctx.check(
        "overall faucet height is taller than parent (~0.16 to 0.18 m)",
        cap_aabb is not None and 0.155 <= cap_aabb[1][2] <= 0.180,
        details=f"cap aabb={cap_aabb}",
    )

    # ---- spout: short forward projection, curves down --------------------
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "spout projects forward from the body and reaches below collar height",
        spout_aabb is not None
        and spout_aabb[1][0] > 0.050
        and spout_aabb[0][2] < 0.095,
        details=f"spout aabb={spout_aabb}",
    )

    # ---- aerator: separate part seated in spout mouth --------------------
    aerator_aabb = ctx.part_world_aabb(aerator)
    ctx.check(
        "aerator is positioned at the spout mouth (low and forward)",
        aerator_aabb is not None
        and aerator_aabb[1][0] > 0.040
        and aerator_aabb[0][2] < 0.095,
        details=f"aerator aabb={aerator_aabb}",
    )
    ctx.expect_within(
        aerator,
        spout,
        axes="xy",
        margin=0.005,
        name="aerator sits within the spout mouth footprint",
    )

    # ---- spout swivel: revolute joint around vertical axis ---------------
    sl = swivel.motion_limits
    ctx.check(
        "spout swivel limits are -90 to +90 degrees",
        sl is not None
        and sl.lower is not None
        and sl.upper is not None
        and abs(sl.lower + SWIVEL_LIMIT) < 1e-6
        and abs(sl.upper - SWIVEL_LIMIT) < 1e-6,
        details=f"limits={sl}",
    )
    ctx.check(
        "spout swivel axis is vertical (Z)",
        swivel.axis is not None
        and abs(swivel.axis[0]) < 1e-9
        and abs(swivel.axis[1]) < 1e-9
        and abs(abs(swivel.axis[2]) - 1.0) < 1e-9,
        details=f"axis={swivel.axis}",
    )

    # Decisive pose: swiveling the spout swings the aerator/outlet sideways.
    aerator_rest = ctx.part_world_position(aerator)
    with ctx.pose({swivel: SWIVEL_LIMIT}):
        aerator_swung = ctx.part_world_position(aerator)
    ctx.check(
        "swiveling spout +90 deg moves the aerator toward +Y",
        aerator_rest is not None
        and aerator_swung is not None
        and aerator_swung[1] > aerator_rest[1] + 0.020
        and abs(aerator_swung[2] - aerator_rest[2]) < 0.005,
        details=f"rest={aerator_rest}, swung={aerator_swung}",
    )

    # ---- stem/cap stack: retained insertion and press/turn ---------------
    ctx.expect_overlap(
        stem,
        body,
        axes="z",
        elem_a="stem_shaft",
        elem_b="body_neck",
        min_overlap=0.008,
        name="valve stem retained inside the neck bore",
    )
    ctx.expect_overlap(
        cap,
        stem,
        axes="z",
        elem_a="cap_shell",
        elem_b="stem_shaft",
        min_overlap=0.002,
        name="cap boss retains the stem top",
    )
    ctx.expect_gap(
        cap,
        body,
        axis="z",
        min_gap=0.002,
        max_gap=0.015,
        name="push cap hovers just above the neck",
    )

    # Press pose: cap moves down.
    rest_pos = ctx.part_world_position(cap)
    with ctx.pose({press: PRESS_TRAVEL}):
        pressed_pos = ctx.part_world_position(cap)
    ctx.check(
        "pressing the cap moves it down along the vertical body axis",
        rest_pos is not None
        and pressed_pos is not None
        and 0.006 <= (rest_pos[2] - pressed_pos[2]) <= 0.010,
        details=f"rest={rest_pos}, pressed={pressed_pos}",
    )

    # Turn limits.
    tl = turn.motion_limits
    ctx.check(
        "temperature turn limits are -60 to +60 degrees",
        tl is not None
        and tl.lower is not None
        and tl.upper is not None
        and abs(tl.lower + TURN_LIMIT) < 1e-6
        and abs(tl.upper - TURN_LIMIT) < 1e-6,
        details=f"limits={tl}",
    )

    return ctx.report()


object_model = build_object_model()
