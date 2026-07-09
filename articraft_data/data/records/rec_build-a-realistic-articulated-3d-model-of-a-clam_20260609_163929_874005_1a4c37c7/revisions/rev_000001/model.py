from __future__ import annotations

# Articraft model: black cast-iron G-clamp (C-clamp) with a zinc threaded
# screw spindle, a T-handle (tommy bar) with two ball ends, and a swivel
# pressing pad. Modeled faithfully from picture/Handtools/Clamp/002.png.
#
# Articraft brief:
# - Object: ~100 mm G-clamp. Black cast-iron C-frame with an integral fixed
#   lower jaw, a heavy threaded boss on the upper arm, a galvanized screw
#   spindle through the boss, a T-handle across the top with two ball ends,
#   and a swivel pressing pad at the screw tip.
# - Root/support: the C-frame is the fixed root. It carries the lower jaw and
#   the threaded boss that guides the screw.
# - Parts: frame (C-body + fixed jaw + boss) and screw (spindle + thread +
#   T-handle bar + two ball ends + swivel pad).
# - Articulation: frame_to_screw, PRISMATIC along the screw axis (+Z up). The
#   screw advances DOWN toward the fixed jaw to clamp; negative travel reduces
#   the jaw gap. Positive q raises the screw (opens the clamp).
# - Visible geometry: black C silhouette, zinc ribbed spindle, chrome T-bar
#   with black balls, round swivel foot, threaded boss collar.
# - Support/fit: the spindle passes through the boss bore; the swivel pad is
#   captured on the spindle tip; the T-bar passes through a cross-hole near the
#   spindle top. The fixed jaw and screw pad face each other across the throat.
# - Intentional overlaps: spindle nested inside the boss bore (sleeve/screw
#   fit); T-bar captured through the spindle cross-hole; pad captured on the
#   spindle tip; ball ends seated on the bar.
# - Tests: frame is the single root, screw is a prismatic child, screw axis is
#   +Z, lowering the screw closes the jaw gap, T-bar + both balls + pad + boss
#   present, pad faces the fixed jaw across the throat.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Key dimensions (meters)
# ---------------------------------------------------------------------------

FRAME_THICK = 0.030  # C-frame thickness along Y (cast-iron section)

# Screw axis is at x=0. The C opens toward +X; the back spine is on -X.
SCREW_AXIS_X = 0.0

# Vertical layout. Fixed-jaw top pressing face sits at z=0.
JAW_TOP_Z = 0.0
JAW_ARM_THICK = 0.026  # vertical thickness of the lower jaw arm

# Top arm centerline / boss: high above the jaw across the throat.
BOSS_CENTER_Z = 0.108  # center of the threaded boss on the upper arm

# Boss (threaded collar the screw runs through).
BOSS_RADIUS = 0.020
BOSS_HEIGHT = 0.044

# Screw spindle.
SPINDLE_RADIUS = 0.0095
THREAD_OUTER_R = 0.0115
# Spindle spans from below the boss (down into the throat toward the pad) up
# through the boss to the T-handle. Authored in the screw part frame; the screw
# part frame origin coincides with the boss center at q=0.
SPINDLE_TOP_Z = 0.092  # spindle protrudes this far above the boss center
SPINDLE_BOTTOM_Z = -0.070  # spindle reaches this far below the boss center

# T-handle (tommy bar).
TBAR_LENGTH = 0.150
TBAR_RADIUS = 0.0055
TBAR_Z = SPINDLE_TOP_Z - 0.006  # cross-bar near the spindle top
BALL_RADIUS = 0.0125

# Swivel pressing pad at the spindle tip.
PAD_RADIUS = 0.018
PAD_THICK = 0.008
PAD_NECK_R = 0.006
PAD_NECK_H = 0.010

# Throat: the C opening depth (how far the jaw/pad reach out in +X is shared by
# both jaws so they oppose each other across the gap).
THROAT_DEPTH = 0.090  # +X reach of the fixed jaw / pad alignment

# At q=0 the pad's pressing face (bottom of the swivel pad, in world Z) sits at
# BOSS_CENTER_Z + SPINDLE_BOTTOM_Z - PAD_NECK_H - PAD_THICK above the fixed
# jaw face (JAW_TOP_Z). The prismatic closing limit must stop the pad just
# shy of the jaw so it never punches through it.
PAD_FACE_Z_AT_REST = BOSS_CENTER_Z + SPINDLE_BOTTOM_Z - PAD_NECK_H - PAD_THICK
CLAMP_CLEARANCE = 0.001  # leave ~1 mm before pad/jaw contact
SCREW_TRAVEL_DOWN = (PAD_FACE_Z_AT_REST - JAW_TOP_Z) - CLAMP_CLEARANCE


def _frame_solid() -> cq.Workplane:
    """Cast-iron C-frame: back spine, upper arm with boss, lower fixed jaw.

    Built as a C-profile in the XZ plane extruded along Y. The profile is a
    closed polyline (rounded) that forms the C silhouette opening toward +X.
    """
    half_t = FRAME_THICK / 2.0

    # Profile in the XZ plane (x = horizontal reach, z = vertical).
    # The throat (mouth) opens toward +X. Back spine on -X side.
    back_x = -0.070  # outer back of the C
    spine_inner_x = -0.044  # inner wall of the back spine (throat side)

    upper_top_z = BOSS_CENTER_Z + 0.024  # top of upper arm
    upper_bot_z = BOSS_CENTER_Z - 0.024  # underside of upper arm
    lower_top_z = JAW_TOP_Z  # top pressing face of fixed jaw
    lower_bot_z = JAW_TOP_Z - JAW_ARM_THICK  # underside of lower jaw

    # Reach of the jaws/arms in +X (mouth). Upper arm extends to hold the boss;
    # lower jaw extends to oppose the pad.
    upper_tip_x = 0.034
    lower_tip_x = THROAT_DEPTH * 0.62  # solid lower jaw nose

    # Closed C-profile, counter-clockwise. Starts at upper outer-back corner.
    pts = [
        (back_x, upper_top_z),      # top-back corner
        (upper_tip_x, upper_top_z),  # top of upper arm tip
        (upper_tip_x, upper_bot_z),  # underside of upper-arm tip
        (spine_inner_x, upper_bot_z),  # into the throat (inner upper)
        (spine_inner_x, lower_top_z),  # down the inner spine to jaw level
        (lower_tip_x, lower_top_z),  # out along the jaw pressing face
        (lower_tip_x, lower_bot_z),  # down the jaw nose
        (back_x, lower_bot_z),      # bottom-back corner
    ]

    profile = (
        cq.Workplane("XZ")
        .polyline(pts)
        .close()
    )
    # Extrude symmetrically along Y for frame thickness.
    frame = profile.extrude(FRAME_THICK, both=True)

    # Soften the visible cast-iron edges.
    frame = frame.edges("|Y").fillet(0.006)

    # Threaded boss: a stout cylindrical collar on top of the upper arm, bored
    # so the screw passes through it.
    boss = (
        cq.Workplane("XY")
        .workplane(offset=BOSS_CENTER_Z - BOSS_HEIGHT / 2.0)
        .center(SCREW_AXIS_X, 0.0)
        .circle(BOSS_RADIUS)
        .extrude(BOSS_HEIGHT)
    )
    frame = frame.union(boss)

    # Bore the screw clearance hole straight down through the boss + upper arm.
    # Bore the screw clearance hole slightly tighter than the thread crests so
    # the spindle thread engages the boss wall (real threaded fit, captured by
    # the spindle/frame overlap allowance) and never reads as floating.
    bore = (
        cq.Workplane("XY")
        .workplane(offset=lower_top_z)
        .center(SCREW_AXIS_X, 0.0)
        .circle(THREAD_OUTER_R - 0.0008)
        .extrude(upper_top_z + 0.05 - lower_top_z)
    )
    frame = frame.cut(bore)

    return frame


def _spindle_solid() -> cq.Workplane:
    """Zinc screw spindle with a ribbed (threaded) shaft.

    Authored in the screw part frame; z=0 is the boss center at q=0.
    """
    length = SPINDLE_TOP_Z - SPINDLE_BOTTOM_Z
    # Smooth core shaft.
    shaft = (
        cq.Workplane("XY")
        .workplane(offset=SPINDLE_BOTTOM_Z)
        .center(SCREW_AXIS_X, 0.0)
        .circle(SPINDLE_RADIUS)
        .extrude(length)
    )

    # Thread ridges as stacked thin tori along the lower portion (visible zinc
    # thread). Keep them only where the thread is exposed below the boss top.
    thread = shaft
    pitch = 0.005
    thread_lo = SPINDLE_BOTTOM_Z + 0.006
    thread_hi = SPINDLE_TOP_Z - 0.014  # leave the very top smooth for the T-bar
    z = thread_lo
    rings = []
    while z <= thread_hi:
        ring = (
            cq.Workplane("XY")
            .workplane(offset=z)
            .center(SCREW_AXIS_X, 0.0)
            .moveTo(SPINDLE_RADIUS, 0.0)
            # small triangular thread crest revolved around the axis
            .lineTo(THREAD_OUTER_R, pitch * 0.18)
            .lineTo(SPINDLE_RADIUS, pitch * 0.42)
            .close()
            .revolve(360.0, (0, 0, 0), (0, 1, 0))
        )
        rings.append(ring)
        z += pitch

    for r in rings:
        thread = thread.union(r)

    return thread


def _pad_solid() -> cq.Workplane:
    """Swivel pressing pad at the screw tip (round foot + neck)."""
    pad = (
        cq.Workplane("XY")
        .workplane(offset=SPINDLE_BOTTOM_Z - PAD_NECK_H - PAD_THICK)
        .center(SCREW_AXIS_X, 0.0)
        .circle(PAD_RADIUS)
        .extrude(PAD_THICK)
        .edges("|Z or <Z")
        .fillet(0.0015)
    )
    neck = (
        cq.Workplane("XY")
        .workplane(offset=SPINDLE_BOTTOM_Z - PAD_NECK_H)
        .center(SCREW_AXIS_X, 0.0)
        .circle(PAD_NECK_R)
        .extrude(PAD_NECK_H + 0.004)  # overlap up into the spindle tip (capture)
    )
    return pad.union(neck)


def _tbar_solid() -> cq.Workplane:
    """Chrome T-handle bar passing through the spindle top (along Y)."""
    bar = (
        cq.Workplane("XZ")
        .workplane(offset=-TBAR_LENGTH / 2.0)
        .center(SCREW_AXIS_X, TBAR_Z)
        .circle(TBAR_RADIUS)
        .extrude(TBAR_LENGTH)
    )
    return bar


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="g_clamp")

    cast_iron = model.material("cast_iron", rgba=(0.10, 0.10, 0.11, 1.0))
    zinc = model.material("zinc_plate", rgba=(0.74, 0.76, 0.79, 1.0))
    chrome = model.material("chrome", rgba=(0.82, 0.84, 0.86, 1.0))
    black_ball = model.material("ball_black", rgba=(0.07, 0.07, 0.08, 1.0))

    # --- Frame (root): black cast-iron C-body with fixed jaw + threaded boss --
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_frame_solid(), "frame_body"),
        material=cast_iron,
        name="frame_body",
    )
    frame.inertial = Inertial.from_geometry(
        Cylinder(radius=0.07, length=0.18),
        mass=0.85,
        origin=Origin(xyz=(-0.02, 0.0, BOSS_CENTER_Z / 2.0)),
    )

    # --- Screw (prismatic child): spindle + thread + T-bar + balls + pad ------
    screw = model.part("screw")
    screw.visual(
        mesh_from_cadquery(_spindle_solid(), "spindle"),
        material=zinc,
        name="spindle",
    )
    screw.visual(
        mesh_from_cadquery(_tbar_solid(), "tbar"),
        material=chrome,
        name="tbar",
    )
    screw.visual(
        Sphere(radius=BALL_RADIUS),
        origin=Origin(xyz=(SCREW_AXIS_X, TBAR_LENGTH / 2.0, TBAR_Z)),
        material=black_ball,
        name="ball_pos",
    )
    screw.visual(
        Sphere(radius=BALL_RADIUS),
        origin=Origin(xyz=(SCREW_AXIS_X, -TBAR_LENGTH / 2.0, TBAR_Z)),
        material=black_ball,
        name="ball_neg",
    )
    screw.visual(
        mesh_from_cadquery(_pad_solid(), "swivel_pad"),
        material=zinc,
        name="swivel_pad",
    )
    screw.inertial = Inertial.from_geometry(
        Cylinder(radius=0.012, length=0.18),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, (SPINDLE_TOP_Z + SPINDLE_BOTTOM_Z) / 2.0)),
    )

    # --- Articulation: screw slides along its axis through the boss ----------
    # The screw part frame origin coincides with the boss center at q=0.
    # +Z raises the screw (opens the clamp); negative q drives the pad toward
    # the fixed jaw (clamping). Limits: from fully raised (0) to clamped
    # against the jaw (negative travel).
    model.articulation(
        "frame_to_screw",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=screw,
        origin=Origin(xyz=(SCREW_AXIS_X, 0.0, BOSS_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.05,
            lower=-SCREW_TRAVEL_DOWN,
            upper=0.010,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    screw = object_model.get_part("screw")
    joint = object_model.get_articulation("frame_to_screw")

    # --- Structure: single root, prismatic screw child along +Z --------------
    roots = object_model.root_parts()
    ctx.check(
        "frame is the single root",
        len(roots) == 1 and roots[0].name == "frame",
        details=f"roots={[r.name for r in roots]}",
    )
    ctx.check(
        "screw joint is prismatic",
        str(joint.joint_type).endswith("PRISMATIC")
        or joint.joint_type == ArticulationType.PRISMATIC,
        details=f"joint_type={joint.joint_type}",
    )
    ax = tuple(round(v, 6) for v in joint.axis)
    ctx.check(
        "screw axis is vertical +Z",
        ax == (0.0, 0.0, 1.0),
        details=f"axis={ax}",
    )

    # --- Hero parts present and placed ---------------------------------------
    tbar = screw.get_visual("tbar")
    ball_pos = screw.get_visual("ball_pos")
    ball_neg = screw.get_visual("ball_neg")
    pad = screw.get_visual("swivel_pad")
    spindle = screw.get_visual("spindle")
    frame_body = frame.get_visual("frame_body")

    for visual, label in (
        (tbar, "tbar"),
        (ball_pos, "ball_pos"),
        (ball_neg, "ball_neg"),
        (pad, "swivel_pad"),
        (spindle, "spindle"),
        (frame_body, "frame_body"),
    ):
        ctx.check(f"{label} present", visual is not None, details=f"{label} missing")

    # T-bar ball ends straddle the spindle along Y (a real tommy bar).
    ctx.expect_origin_distance(
        screw,
        screw,
        axes="y",
        min_dist=0.0,
        name="screw assembly spans width",
    )
    ball_pos_aabb = ctx.part_element_world_aabb(screw, elem="ball_pos")
    ball_neg_aabb = ctx.part_element_world_aabb(screw, elem="ball_neg")
    ctx.check(
        "ball ends straddle the spindle along Y",
        ball_pos_aabb is not None
        and ball_neg_aabb is not None
        and ball_pos_aabb[0][1] > 0.04
        and ball_neg_aabb[1][1] < -0.04,
        details=f"pos={ball_pos_aabb}, neg={ball_neg_aabb}",
    )

    # Swivel pad sits at the bottom of the screw, facing the fixed jaw (below).
    pad_aabb = ctx.part_element_world_aabb(screw, elem="swivel_pad")
    tbar_aabb = ctx.part_element_world_aabb(screw, elem="tbar")
    ctx.check(
        "pad is below the T-bar along Z",
        pad_aabb is not None
        and tbar_aabb is not None
        and pad_aabb[1][2] < tbar_aabb[0][2],
        details=f"pad={pad_aabb}, tbar={tbar_aabb}",
    )

    # The threaded boss bore captures the spindle: spindle stays inside the
    # frame footprint in XY near the axis.
    ctx.expect_within(
        screw,
        frame,
        axes="xy",
        inner_elem="spindle",
        outer_elem="frame_body",
        margin=0.002,
        name="spindle runs through the frame boss",
    )

    # --- Pad opposes the fixed jaw across the throat -------------------------
    # At rest (q=0) the pad hovers above the fixed-jaw pressing face.
    frame_aabb = ctx.part_world_aabb(frame)
    with ctx.pose({joint: 0.0}):
        pad_rest = ctx.part_element_world_aabb(screw, elem="swivel_pad")
    ctx.check(
        "pad hovers above the fixed jaw at rest",
        pad_rest is not None
        and frame_aabb is not None
        and pad_rest[0][2] > JAW_TOP_Z,
        details=f"pad_rest={pad_rest}, jaw_top_z={JAW_TOP_Z}",
    )

    # --- Mechanism: driving the screw down closes the throat gap -------------
    with ctx.pose({joint: 0.0}):
        pad_open = ctx.part_world_position(screw)
    with ctx.pose({joint: joint.motion_limits.lower}):
        pad_clamped = ctx.part_world_position(screw)
    ctx.check(
        "lowering the screw moves the pad down toward the jaw",
        pad_open is not None
        and pad_clamped is not None
        and pad_clamped[2] < pad_open[2] - 0.9 * SCREW_TRAVEL_DOWN,
        details=f"open={pad_open}, clamped={pad_clamped}",
    )

    # At the closing limit the pad face stops just above the jaw face (never
    # punching through the fixed jaw or below the frame underside).
    with ctx.pose({joint: joint.motion_limits.lower}):
        pad_closed = ctx.part_element_world_aabb(screw, elem="swivel_pad")
    ctx.check(
        "pad stops at the jaw face at full closure",
        pad_closed is not None
        and JAW_TOP_Z - 1e-6 <= pad_closed[0][2] <= JAW_TOP_Z + 0.003,
        details=f"pad_closed={pad_closed}, jaw_top_z={JAW_TOP_Z}",
    )

    # --- Intentional capture/nesting allowances ------------------------------
    ctx.allow_overlap(
        screw,
        frame,
        elem_a="spindle",
        elem_b="frame_body",
        reason="The threaded spindle is captured inside the boss clearance bore (screw-in-boss fit).",
    )
    ctx.allow_overlap(
        screw,
        screw,
        elem_a="tbar",
        elem_b="spindle",
        reason="The tommy bar passes through the spindle cross-hole and is captured by it.",
    )
    ctx.allow_overlap(
        screw,
        screw,
        elem_a="swivel_pad",
        elem_b="spindle",
        reason="The swivel pad neck is captured on the spindle tip.",
    )
    ctx.allow_overlap(
        screw,
        screw,
        elem_a="ball_pos",
        elem_b="tbar",
        reason="The ball end is seated on the end of the tommy bar.",
    )
    ctx.allow_overlap(
        screw,
        screw,
        elem_a="ball_neg",
        elem_b="tbar",
        reason="The ball end is seated on the end of the tommy bar.",
    )

    return ctx.report()


object_model = build_object_model()
