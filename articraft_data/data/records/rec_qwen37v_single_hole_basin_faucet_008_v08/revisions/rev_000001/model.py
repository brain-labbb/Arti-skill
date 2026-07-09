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
# Self-closing (timed-flow) basin pillar tap, ~0.13 m tall, mirror chrome.
# World frame: +Z up, deck at z = 0, spout points toward +X (front).
# The body leans BACK a few degrees, i.e. its long axis tilts toward -X.
# All "s" values below are axial stations in meters measured along that
# tilted body axis from the deck.
# ---------------------------------------------------------------------------

TILT = math.radians(6.0)
SIN_T = math.sin(TILT)
COS_T = math.cos(TILT)

# Base flange (sits flat on the deck).
FLANGE_R = 0.030
FLANGE_H = 0.006

# Raised circular collar around the base (above flange).
COLLAR_R = 0.029
COLLAR_H = 0.008
COLLAR_S0 = FLANGE_H
COLLAR_S1 = COLLAR_S0 + COLLAR_H

# Main body barrel.
BODY_R = 0.025
BODY_S0 = COLLAR_S1
BODY_S1 = 0.0725

# Grip grooves: subtle rings on the body barrel for tactile grip.
GRIP_GROOVE_R = 0.0235  # slightly recessed from body radius
GRIP_GROOVE_H = 0.0015
GRIP_GROOVE_SPACING = 0.004
GRIP_GROOVE_S0 = 0.054  # above spout exit to avoid interference
GRIP_GROOVE_COUNT = 4

# Cartridge cap seam: thin ring below the push cap.
SEAM_R = 0.0238
SEAM_H = 0.002

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

PRESS_TRAVEL = 0.008
TURN_LIMIT = math.radians(60.0)
SPOUT_SWIVEL_LIMIT = math.radians(90.0)


def _axis_point(s: float) -> tuple[float, float, float]:
    """World position of the tilted body axis at axial station s."""
    return (-s * SIN_T, 0.0, s * COS_T)


def _tilted(s: float) -> Origin:
    """Origin on the body axis at station s, z-axis aligned with the axis."""
    return Origin(xyz=_axis_point(s), rpy=(0.0, -TILT, 0.0))


def _build_spout_shape() -> cq.Workplane:
    """Hollow chrome spout: straight shank, smooth downward bend, flared
    open outlet rim. Built in spout-local frame whose origin sits on the
    body axis at SPOUT_S; the shank runs along local +X."""
    r_out = 0.015
    shank_x0 = 0.010  # seated ~15 mm inside the body casting
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

    # Tapered bore opening the outlet mouth (real hollow outlet rim).
    bore = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z - 0.006))
        .circle(0.0155)
        .workplane(offset=0.018)
        .circle(0.011)
        .loft()
    )
    return spout.cut(bore)


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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="self_closing_timed_flow_pillar_tap")

    model.material("chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    model.material("chrome_dark", rgba=(0.42, 0.44, 0.47, 1.0))
    model.material("chrome_brushed", rgba=(0.70, 0.72, 0.74, 1.0))
    model.material("index_mark", rgba=(0.30, 0.32, 0.35, 1.0))

    # ---------------- body (root): flange + collar + barrel + grooves + neck ---------
    body = model.part("body")
    body.visual(
        Cylinder(radius=FLANGE_R, length=FLANGE_H),
        origin=Origin(xyz=(0.0, 0.0, FLANGE_H / 2.0)),
        material="chrome",
        name="base_flange",
    )
    # Raised circular collar around the base.
    body.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_H),
        origin=_tilted((COLLAR_S0 + COLLAR_S1) / 2.0),
        material="chrome",
        name="base_collar",
    )
    body.visual(
        Cylinder(radius=BODY_R, length=BODY_S1 - BODY_S0),
        origin=_tilted((BODY_S0 + BODY_S1) / 2.0),
        material="chrome",
        name="body_barrel",
    )
    # Subtle grip grooves on the body barrel surface.
    for i in range(GRIP_GROOVE_COUNT):
        groove_s = GRIP_GROOVE_S0 + i * GRIP_GROOVE_SPACING
        body.visual(
            Cylinder(radius=GRIP_GROOVE_R, length=GRIP_GROOVE_H),
            origin=_tilted(groove_s + GRIP_GROOVE_H / 2.0),
            material="chrome_dark",
            name=f"grip_groove_{i}",
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
    # Thin cartridge cap seam ring below the push cap.
    body.visual(
        Cylinder(radius=SEAM_R, length=SEAM_H),
        origin=_tilted(NECK_S1 - SEAM_H / 2.0),
        material="chrome_dark",
        name="cartridge_seam",
    )

    # ---------------- spout (swivel): swept hollow tube + flared outlet -----
    spout = model.part("spout")
    spout.visual(
        mesh_from_cadquery(_build_spout_shape(), "spout", tolerance=0.0003),
        material="chrome",
        name="spout_tube",
    )
    model.articulation(
        "spout_swivel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=spout,
        origin=_tilted(SPOUT_S),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=1.0,
            lower=-SPOUT_SWIVEL_LIMIT, upper=SPOUT_SWIVEL_LIMIT,
        ),
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

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    spout = object_model.get_part("spout")
    stem = object_model.get_part("valve_stem")
    cap = object_model.get_part("push_cap")
    press = object_model.get_articulation("cap_press")
    turn = object_model.get_articulation("cap_turn")
    swivel = object_model.get_articulation("spout_swivel")

    # Intentional seated insertions (solid proxies, scoped per element).
    ctx.allow_overlap(
        spout,
        body,
        elem_a="spout_tube",
        elem_b="body_barrel",
        reason="Spout shank is intentionally seated ~15 mm into the solid body casting.",
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
    ctx.allow_overlap(
        body,
        stem,
        elem_a="cartridge_seam",
        elem_b="stem_shaft",
        reason="Thin cartridge seam ring sits at the neck top where the stem passes through.",
    )
    for i in range(GRIP_GROOVE_COUNT):
        ctx.allow_overlap(
            body,
            spout,
            elem_a=f"grip_groove_{i}",
            elem_b="spout_tube",
            reason="Thin grip groove rings are cosmetic surface features on the body near the spout exit point.",
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

    # ---- variant 08: raised circular collar around the base ---------------
    collar_aabb = ctx.part_element_world_aabb(body, elem="base_collar")
    barrel_aabb = ctx.part_element_world_aabb(body, elem="body_barrel")
    ctx.check(
        "raised circular collar exists above the flange",
        collar_aabb is not None
        and collar_aabb[0][2] > 0.002
        and collar_aabb[1][2] < 0.022,
        details=f"collar aabb={collar_aabb}",
    )
    ctx.check(
        "collar is wider than the body barrel",
        collar_aabb is not None
        and barrel_aabb is not None
        and (collar_aabb[1][1] - collar_aabb[0][1]) > (barrel_aabb[1][1] - barrel_aabb[0][1]) + 0.005,
        details=f"collar aabb={collar_aabb}, barrel aabb={barrel_aabb}",
    )

    # ---- variant 08: grip grooves on the body surface ---------------------
    groove_aabb = ctx.part_element_world_aabb(body, elem="grip_groove_0")
    ctx.check(
        "grip grooves exist on the body barrel surface",
        groove_aabb is not None,
        details=f"grip_groove_0 aabb={groove_aabb}",
    )

    # ---- variant 08: cartridge cap seam below the lever -------------------
    seam_aabb = ctx.part_element_world_aabb(body, elem="cartridge_seam")
    ctx.check(
        "thin cartridge cap seam exists below the push cap",
        seam_aabb is not None,
        details=f"seam aabb={seam_aabb}",
    )
    ctx.check(
        "cartridge seam is positioned near the neck top",
        seam_aabb is not None
        and neck_aabb is not None
        and abs(seam_aabb[1][2] - neck_aabb[1][2]) < 0.008,
        details=f"seam aabb={seam_aabb}, neck aabb={neck_aabb}",
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

    # ---- variant 08: spout swivel articulation ----------------------------
    sw = swivel.motion_limits
    ctx.check(
        "spout swivel is a revolute joint with ±90 degree limits",
        sw is not None
        and sw.lower is not None
        and sw.upper is not None
        and abs(sw.lower + SPOUT_SWIVEL_LIMIT) < 1e-6
        and abs(sw.upper - SPOUT_SWIVEL_LIMIT) < 1e-6,
        details=f"limits={sw}",
    )
    # Decisive pose: swiveling the spout moves its outlet sideways.
    spout_rest = ctx.part_world_aabb(spout)
    with ctx.pose({swivel: SPOUT_SWIVEL_LIMIT}):
        spout_swung = ctx.part_world_aabb(spout)
    ctx.check(
        "spout swivel rotates the spout outlet laterally",
        spout_rest is not None
        and spout_swung is not None
        and abs(
            (spout_swung[0][1] + spout_swung[1][1]) / 2.0
            - (spout_rest[0][1] + spout_rest[1][1]) / 2.0
        ) > 0.02,
        details=f"rest_y={(spout_rest[0][1] + spout_rest[1][1]) / 2.0:.4f}, "
                f"swung_y={(spout_swung[0][1] + spout_swung[1][1]) / 2.0:.4f}",
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
        "overall tap height is about 0.13 m",
        cap_aabb is not None and 0.125 <= cap_aabb[1][2] <= 0.138,
        details=f"cap aabb={cap_aabb}",
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
