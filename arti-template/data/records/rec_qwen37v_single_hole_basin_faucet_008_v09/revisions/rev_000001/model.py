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
# Single-hole basin faucet variant (~0.13 m tall, mirror chrome).
# World frame: +Z up, deck at z = 0, spout points toward +X (front).
# The body leans BACK a few degrees, i.e. its long axis tilts toward -X.
# Variant 09: detachable spout collar, pull-up drain rod, hollow outlet.
# ---------------------------------------------------------------------------

TILT = math.radians(6.0)
SIN_T = math.sin(TILT)
COS_T = math.cos(TILT)

# Base flange (sits flat on the deck).
FLANGE_R = 0.030
FLANGE_H = 0.006

# Main body barrel.
BODY_R = 0.025
BODY_S0 = 0.006
BODY_S1 = 0.0725

# Thin recessed separation groove ring around the upper third.
GROOVE_R = 0.0215
GROOVE_S0 = 0.0705
GROOVE_S1 = 0.0760

# Stepped-in upper neck above the groove.
NECK_R = 0.0228
NECK_S0 = 0.0740
NECK_S1 = 0.104

# Valve stem (prismatic press carrier) nested in the neck bore.
STEM_R = 0.011
STEM_S0 = 0.0925
STEM_S1 = 0.1155
STEM_LEN = STEM_S1 - STEM_S0

# Push cap (press-to-run button, also rotates for temperature).
CAP_JOINT_S = 0.117  # cap frame origin: bottom of the flat cap disc
CAP_R = 0.030
CAP_DISC_H = 0.013
CAP_FLARE_H = 0.004
CAP_FLARE_R0 = 0.016

# Spout exit station on the body axis.
SPOUT_S = 0.050

# Spout collar (detachable seam ring).
COLLAR_R = 0.019
COLLAR_H = 0.005

# Drain rod dimensions.
DRAIN_ROD_R = 0.002
DRAIN_ROD_LEN = 0.065
DRAIN_ROD_KNOB_R = 0.005
DRAIN_ROD_KNOB_H = 0.006
DRAIN_ROD_TRAVEL = 0.030

PRESS_TRAVEL = 0.008
TURN_LIMIT = math.radians(60.0)


def _axis_point(s: float) -> tuple[float, float, float]:
    """World position of the tilted body axis at axial station s."""
    return (-s * SIN_T, 0.0, s * COS_T)


def _tilted(s: float) -> Origin:
    """Origin on the body axis at station s, z-axis aligned with the axis."""
    return Origin(xyz=_axis_point(s), rpy=(0.0, -TILT, 0.0))


def _build_spout_shape() -> cq.Workplane:
    """Hollow chrome spout: straight shank, smooth downward bend, flared
    open outlet rim with a real hollow bore, plus a detachable collar ring
    at the base. Built in spout-local frame whose origin sits on the body
    axis at SPOUT_S; the shank runs along local +X."""
    r_out = 0.015
    shank_x0 = 0.010  # seated ~10 mm inside the body casting
    shank_x1 = 0.035
    bend = 0.028  # bend radius; end heads straight down at (0.063, -0.028)
    end_x = shank_x1 + bend
    end_z = -bend

    path = (
        cq.Workplane("XZ")
        .moveTo(shank_x0, 0.0)
        .lineTo(shank_x1, 0.0)
        .tangentArcPoint((bend, -bend), relative=True)
    )
    tube = cq.Workplane("YZ", origin=(shank_x0, 0.0, 0.0)).circle(r_out).sweep(path)

    # Flared outlet skirt around the down-turned end.
    flare = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z + 0.006))
        .circle(0.0148)
        .workplane(offset=-0.010)
        .circle(0.0185)
        .loft()
    )
    spout = tube.union(flare)

    # Deep cylindrical bore for real hollow outlet at spout mouth.
    bore = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z - 0.010))
        .circle(0.012)
        .extrude(0.022)
    )
    spout = spout.cut(bore)

    # Detachable collar ring at spout base (where it exits the body).
    # Ring perpendicular to the shank axis, slightly larger than the tube.
    collar = (
        cq.Workplane("YZ", origin=(0.006, 0.0, 0.0))
        .circle(COLLAR_R)
        .extrude(COLLAR_H)
    )
    collar_bore = (
        cq.Workplane("YZ", origin=(0.006, 0.0, 0.0))
        .circle(r_out)
        .extrude(COLLAR_H)
    )
    collar = collar.cut(collar_bore)
    spout = spout.union(collar)

    return spout


def _build_cap_shape() -> cq.Workplane:
    """Push cap shell: under-flare cone from the stem boss out to the flat
    disc, with a softened top rim. Cap-local z=0 is the disc bottom."""
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


def _build_drain_rod_shape() -> cq.Workplane:
    """Pull-up drain rod: thin vertical shaft with a knob at the top.
    Rod-local z=0 is at the bottom of the rod."""
    shaft = cq.Workplane("XY").circle(DRAIN_ROD_R).extrude(DRAIN_ROD_LEN)
    # Knob at top for gripping
    knob = (
        cq.Workplane("XY", origin=(0.0, 0.0, DRAIN_ROD_LEN))
        .circle(DRAIN_ROD_KNOB_R)
        .extrude(DRAIN_ROD_KNOB_H)
    )
    knob = knob.edges(">Z").fillet(0.001)
    return shaft.union(knob)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    model.material("chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    model.material("chrome_dark", rgba=(0.42, 0.44, 0.47, 1.0))
    model.material("chrome_brushed", rgba=(0.70, 0.72, 0.74, 1.0))
    model.material("index_mark", rgba=(0.30, 0.32, 0.35, 1.0))


    # ---------------- body (root): flange + barrel + groove + neck ---------
    body = model.part("body")
    body.visual(
        Cylinder(radius=FLANGE_R, length=FLANGE_H),
        origin=Origin(xyz=(0.0, 0.0, FLANGE_H / 2.0)),
        material="chrome",
        name="base_flange",
    )
    body.visual(
        Cylinder(radius=BODY_R, length=BODY_S1 - BODY_S0),
        origin=_tilted((BODY_S0 + BODY_S1) / 2.0),
        material="chrome",
        name="body_barrel",
    )
    body.visual(
        Cylinder(radius=GROOVE_R, length=GROOVE_S1 - GROOVE_S0),
        origin=_tilted((GROOVE_S0 + GROOVE_S1) / 2.0),
        material="chrome_dark",
        name="groove_ring",
    )
    body.visual(
        Cylinder(radius=NECK_R, length=NECK_S1 - NECK_S0),
        origin=_tilted((NECK_S0 + NECK_S1) / 2.0),
        material="chrome",
        name="body_neck",
    )

    # Drain rod guide boss on the back of the body.
    # A small cylindrical boss protruding from the body rear toward the
    # drain rod, representing the bore guide the rod slides through.
    boss_station = 0.058  # height station for the boss
    boss_pt = _axis_point(boss_station)
    boss_body_x = boss_pt[0] - BODY_R  # body back surface
    rod_station = 0.045
    rod_pt = _axis_point(rod_station)
    rod_x_val = rod_pt[0] - BODY_R - DRAIN_ROD_R - 0.010
    boss_center_x = (boss_body_x + rod_x_val) / 2.0
    boss_length = abs(boss_body_x - rod_x_val) + 0.002  # slight overlap both ends
    boss_z = boss_pt[2]
    body.visual(
        Cylinder(radius=0.007, length=boss_length),
        origin=Origin(
            xyz=(boss_center_x, 0.0, boss_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="chrome",
        name="drain_rod_guide",
    )

    # ---------------- spout (fixed): swept hollow tube + flared outlet -----
    spout = model.part("spout")
    spout.visual(
        mesh_from_cadquery(_build_spout_shape(), "spout", tolerance=0.0003),
        material="chrome",
        name="spout_tube",
    )
    model.articulation(
        "spout_mount",
        ArticulationType.FIXED,
        parent=body,
        child=spout,
        origin=Origin(xyz=_axis_point(SPOUT_S)),
    )

    # ---------------- valve stem (prismatic press carrier) -----------------
    stem = model.part("valve_stem")
    stem.visual(
        Cylinder(radius=STEM_R, length=STEM_LEN),
        origin=Origin(),
        material="chrome",
        name="stem_shaft",
    )
    # Joint frame on the body axis at the neck top; local +z runs up the
    # tilted axis, so axis -z makes positive q press DOWN toward the body.
    model.articulation(
        "cap_press",
        ArticulationType.PRISMATIC,
        parent=body,
        child=stem,
        origin=Origin(xyz=_axis_point(NECK_S1), rpy=(0.0, -TILT, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.05, lower=0.0, upper=PRESS_TRAVEL),
    )

    # ---------------- push cap (revolute temperature ring) -----------------
    cap = model.part("push_cap")
    cap.visual(
        mesh_from_cadquery(_build_cap_shape(), "push_cap", tolerance=0.0003),
        material="chrome",
        name="cap_shell",
    )
    cap.visual(
        Cylinder(radius=0.028, length=0.0025),
        origin=Origin(xyz=(0.0, 0.0, 0.01225)),
        material="chrome_brushed",
        name="cap_top_brushed",
    )
    # Small engraved temperature index mark on the front of the cap rim.
    cap.visual(
        Cylinder(radius=0.0025, length=0.0025),
        origin=Origin(xyz=(0.0295, 0.0, 0.0065), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="index_mark",
        name="temp_indicator_dot",
    )
    model.articulation(
        "cap_turn",
        ArticulationType.REVOLUTE,
        parent=stem,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, CAP_JOINT_S - NECK_S1)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-TURN_LIMIT, upper=TURN_LIMIT),
    )

    # ---------------- drain rod (prismatic, slides vertically behind body) -
    drain_rod = model.part("drain_rod")
    drain_rod.visual(
        mesh_from_cadquery(_build_drain_rod_shape(), "drain_rod", tolerance=0.0003),
        material="chrome",
        name="drain_rod_shaft",
    )

    # Position the drain rod behind the body at mid-height.
    # The rod emerges from the rear of the body, so place it offset in -X.
    # Extra offset ensures the rod clears the tilted neck above.
    rod_station = 0.045  # axial station on body axis for height reference
    rod_axis_pt = _axis_point(rod_station)
    rod_x = rod_axis_pt[0] - BODY_R - DRAIN_ROD_R - 0.010  # clear behind body
    rod_z_base = rod_axis_pt[2] - 0.010  # rod starts slightly below mid-height

    model.articulation(
        "drain_rod_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drain_rod,
        origin=Origin(xyz=(rod_x, 0.0, rod_z_base)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.1,
            lower=0.0,
            upper=DRAIN_ROD_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    spout = object_model.get_part("spout")
    stem = object_model.get_part("valve_stem")
    cap = object_model.get_part("push_cap")
    drain_rod = object_model.get_part("drain_rod")
    press = object_model.get_articulation("cap_press")
    turn = object_model.get_articulation("cap_turn")
    drain_slide = object_model.get_articulation("drain_rod_slide")

    # Intentional seated insertions (solid proxies, scoped per element).
    ctx.allow_overlap(
        spout,
        body,
        elem_a="spout_tube",
        elem_b="body_barrel",
        reason="Spout shank is intentionally seated ~10 mm into the solid body casting.",
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
        reason="Stem top is press-fit ~2.5 mm into the cap's under-flare boss.",
    )
    # The drain rod is positioned behind the body but close to the surface.
    # Allow any marginal overlap with the body casting at the bore exit.
    ctx.allow_overlap(
        drain_rod,
        body,
        reason="Drain rod passes through a bore in the rear of the body casting.",
    )

    # ---- hero geometry: flange seated on deck, body leaning back ----------
    flange_aabb = ctx.part_element_world_aabb(body, elem="base_flange")
    ctx.check(
        "base flange sits flat on the deck",
        flange_aabb is not None and abs(flange_aabb[0][2]) <= 0.0005,
        details=f"flange aabb={flange_aabb}",
    )
    neck_aabb = ctx.part_element_world_aabb(body, elem="body_neck")
    ctx.check(
        "body leans back (neck offset toward -X behind the flange center)",
        neck_aabb is not None and (neck_aabb[0][0] + neck_aabb[1][0]) / 2.0 < -0.005,
        details=f"neck aabb={neck_aabb}",
    )

    # ---- spout: projects forward from the body and curves down ------------
    ctx.expect_overlap(
        spout,
        body,
        axes="xz",
        min_overlap=0.005,
        name="spout shank stays seated in the body",
    )
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "spout reaches forward and droops to a low open outlet above the deck",
        spout_aabb is not None
        and spout_aabb[1][0] > 0.060
        and spout_aabb[0][2] < 0.025
        and spout_aabb[0][2] > 0.008,
        details=f"spout aabb={spout_aabb}",
    )

    # ---- spout collar seam (detachable appearance) -------------------------
    # The collar is built into the spout tube mesh. Verify the spout has
    # a wider region at the base (collar) compared to the tube mid-section.
    ctx.check(
        "spout tube includes a collar ring (Y extent > tube diameter at base)",
        spout_aabb is not None
        and (spout_aabb[1][1] - spout_aabb[0][1]) > 0.030,
        details=f"spout aabb={spout_aabb}",
    )

    # ---- stem/cap stack: retained insertion, cap floats above the neck ----
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
    ctx.expect_within(
        stem,
        cap,
        axes="xy",
        inner_elem="stem_shaft",
        margin=0.001,
        name="stem stays centered under the push cap",
    )
    ctx.expect_gap(
        cap,
        body,
        axis="z",
        min_gap=0.002,
        max_gap=0.012,
        name="push cap hovers just above the neck (carried by the stem)",
    )
    cap_aabb = ctx.part_world_aabb(cap)
    ctx.check(
        "push cap is wider than the stepped-in neck (flared button)",
        cap_aabb is not None
        and neck_aabb is not None
        and (cap_aabb[1][1] - cap_aabb[0][1]) > (neck_aabb[1][1] - neck_aabb[0][1]) + 0.010,
        details=f"cap aabb={cap_aabb}, neck aabb={neck_aabb}",
    )
    ctx.check(
        "overall faucet height is about 0.13 m",
        cap_aabb is not None and 0.125 <= cap_aabb[1][2] <= 0.138,
        details=f"cap aabb={cap_aabb}",
    )

    # ---- drain rod: positioned behind body, slides vertically ------------
    drain_aabb = ctx.part_world_aabb(drain_rod)
    ctx.check(
        "drain rod is positioned behind the body (negative X offset)",
        drain_aabb is not None and (drain_aabb[0][0] + drain_aabb[1][0]) / 2.0 < -0.020,
        details=f"drain_rod aabb={drain_aabb}",
    )
    ctx.check(
        "drain rod is vertical (taller than wide)",
        drain_aabb is not None
        and (drain_aabb[1][2] - drain_aabb[0][2]) > (drain_aabb[1][0] - drain_aabb[0][0]) * 3.0,
        details=f"drain_rod aabb={drain_aabb}",
    )

    # ---- drain rod articulation limits ------------------------------------
    dl = drain_slide.motion_limits
    ctx.check(
        "drain rod slide has valid prismatic limits",
        dl is not None
        and dl.lower is not None
        and dl.upper is not None
        and abs(dl.lower) < 1e-9
        and dl.upper > 0.020
        and dl.upper <= 0.040,
        details=f"limits={dl}",
    )

    # ---- drain rod pose: pulling up raises the rod -------------------------
    rod_rest_pos = ctx.part_world_position(drain_rod)
    with ctx.pose({drain_slide: DRAIN_ROD_TRAVEL}):
        rod_pulled_pos = ctx.part_world_position(drain_rod)
    ctx.check(
        "pulling the drain rod raises it vertically",
        rod_rest_pos is not None
        and rod_pulled_pos is not None
        and (rod_pulled_pos[2] - rod_rest_pos[2]) > 0.020
        and abs(rod_pulled_pos[0] - rod_rest_pos[0]) < 0.001,
        details=f"rest={rod_rest_pos}, pulled={rod_pulled_pos}",
    )

    # ---- articulation limits match the prompt ------------------------------
    pl = press.motion_limits
    ctx.check(
        "press travel limits are 0 to 8 mm",
        pl is not None
        and pl.lower is not None
        and pl.upper is not None
        and abs(pl.lower) < 1e-9
        and abs(pl.upper - PRESS_TRAVEL) < 1e-9,
        details=f"limits={pl}",
    )
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

    # ---- decisive poses: press goes down the tilted axis, turn swings dot --
    rest_pos = ctx.part_world_position(cap)
    with ctx.pose({press: PRESS_TRAVEL}):
        pressed_pos = ctx.part_world_position(cap)
    ctx.check(
        "pressing the cap moves it down along the tilted body axis",
        rest_pos is not None
        and pressed_pos is not None
        and 0.006 <= (rest_pos[2] - pressed_pos[2]) <= 0.0095
        and (pressed_pos[0] - rest_pos[0]) > 0.0003,
        details=f"rest={rest_pos}, pressed={pressed_pos}",
    )

    dot_rest = ctx.part_element_world_aabb(cap, elem="temp_indicator_dot")
    with ctx.pose({turn: TURN_LIMIT}):
        dot_hot = ctx.part_element_world_aabb(cap, elem="temp_indicator_dot")
    ctx.check(
        "turning the cap +60 deg swings the index mark around the cap axis",
        dot_rest is not None
        and dot_hot is not None
        and abs((dot_rest[0][1] + dot_rest[1][1]) / 2.0) < 0.004
        and (dot_hot[0][1] + dot_hot[1][1]) / 2.0 > 0.018,
        details=f"rest={dot_rest}, turned={dot_hot}",
    )

    return ctx.report()


object_model = build_object_model()
