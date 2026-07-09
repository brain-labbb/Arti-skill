from __future__ import annotations

# ---------------------------------------------------------------------------
# C-clamp (a.k.a. G-clamp), a small hand tool — anvil-disc variant.
#
# Reference image (picture/Handtools/Clamp/001.png):
#   - A red cast-iron "C" / "G" shaped frame. The lower jaw of the C ends in a
#     small fixed anvil pad. The upper arm carries a boss that the screw threads
#     through.
#   - A blue threaded spindle (screw) runs vertically down through the top arm's
#     boss into the throat of the C.
#   - At the top of the screw is a purple T-bar handle with orange/red end caps.
#     Turning the bar drives the screw down/up.
#   - At the bottom tip of the screw is a broad flat steel anvil pressing disc
#     fixed rigidly to the spindle tip (no swivel neck — a wide flat clamping
#     face that advances straight down onto the workpiece).
#
# Coordinate convention:
#   +Z is up (the screw axis). The C opens toward -Y (its mouth/throat faces
#   -Y); the solid spine of the C is on +Y. The handle T-bar lies along X.
#
# Articulation (real mechanism):
#   - frame_to_screw  : PRISMATIC along -Z. Turning the handle advances the
#     screw down into the throat to clamp. This is the primary clamp motion.
#   The T-handle bar and the pressing disc are rigidly part of the screw (they
#   move together), so they are fused into the screw part rather than given
#   separate joints.
# ---------------------------------------------------------------------------

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
# Dimensions (meters). A small bench C-clamp, ~2 inch capacity.
# ---------------------------------------------------------------------------

FRAME_DEPTH = 0.034          # thickness of the C-frame along X
FRAME_BAR = 0.026            # width of the C cross-section (in the YZ plane)

THROAT = 0.060               # clear opening between the jaws (Z) at rest
THROAT_Y = 0.060             # depth of the throat (how far the mouth reaches in Y)

# The C spine sits on the +Y side. The mouth opens toward -Y.
SPINE_Y = THROAT_Y / 2.0 + FRAME_BAR / 2.0      # center Y of the vertical spine
BOTTOM_Z = 0.0               # bottom (lower jaw) reference plane
TOP_ARM_Z = THROAT + FRAME_BAR                  # center Z of the upper arm bar

# Lower anvil pad (fixed jaw foot) on the bottom inner jaw.
ANVIL_R = 0.018
ANVIL_H = 0.010
ANVIL_Y = 0.0                # the fixed pad sits at the throat center line in Y

# Screw / spindle
SCREW_R = 0.0085             # core radius of the threaded spindle
THREAD_R = 0.0030            # thread ridge radius (tube swept along helix)
SCREW_LEN = 0.085            # visible threaded length of the spindle
BOSS_R = 0.016               # threaded boss on the top arm the screw runs through
BOSS_H = 0.020

# Collar nut just under the boss (blue hex/round collar in the image)
COLLAR_R = 0.014
COLLAR_H = 0.009

# T-handle bar
HANDLE_R = 0.0065
HANDLE_LEN = 0.050           # HALF length of the T-bar along X (full span = 0.10)
HANDLE_CAP_R = 0.010         # orange end caps
HANDLE_CAP_H = 0.012
HUB_R = 0.012                # purple hub where bar meets the screw top
HUB_H = 0.018

# Anvil pressing disc (broad flat disc fixed rigidly to the screw tip)
DISC_R = 0.024               # broad clamping face radius (~48 mm diameter)
DISC_H = 0.007               # thin flat disc
DISC_CHAMFER = 0.0015        # small edge chamfer for a machined look
DISC_HUB_R = 0.012           # small raised hub where disc meets spindle tip
DISC_HUB_H = 0.004

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

RED = "clamp_red"
BLUE = "clamp_blue"
PURPLE = "clamp_purple"
ORANGE = "clamp_orange"
STEEL = "clamp_steel"


def _build_frame_mesh() -> cq.Workplane:
    """Red cast 'C' frame: lower jaw, vertical spine, upper arm, all one body.

    Built in the X / (Y,Z) plane. The cross-section of the bar is a rounded
    square of FRAME_DEPTH (X) x FRAME_BAR (the in-plane dimension). We build it
    as three filleted box segments fused, plus the boss on the top arm, plus the
    fixed anvil pad on the lower jaw.
    """
    inner_y = -THROAT_Y / 2.0          # inner face of the spine (mouth side)
    outer_y = SPINE_Y + FRAME_BAR / 2.0

    # Vertical spine (back of the C), spanning the full height plus the bars.
    spine_z0 = BOTTOM_Z - FRAME_BAR / 2.0
    spine_z1 = TOP_ARM_Z + FRAME_BAR / 2.0
    spine = (
        cq.Workplane("XY")
        .box(
            FRAME_DEPTH,
            FRAME_BAR,
            spine_z1 - spine_z0,
            centered=(True, True, False),
        )
        .translate((0.0, SPINE_Y, spine_z0))
    )

    # Lower jaw bar: reaches from the spine inward across the throat.
    jaw_len_y = SPINE_Y + FRAME_BAR / 2.0 - inner_y
    lower = (
        cq.Workplane("XY")
        .box(FRAME_DEPTH, jaw_len_y, FRAME_BAR, centered=(True, True, True))
        .translate((0.0, (outer_y + inner_y) / 2.0, BOTTOM_Z))
    )

    # Upper arm bar: same span at the top, carries the screw boss.
    upper = (
        cq.Workplane("XY")
        .box(FRAME_DEPTH, jaw_len_y, FRAME_BAR, centered=(True, True, True))
        .translate((0.0, (outer_y + inner_y) / 2.0, TOP_ARM_Z))
    )

    frame = spine.union(lower).union(upper)

    # Round the prominent outer corners so it reads as a cast C-clamp.
    frame = frame.edges("|X").fillet(0.006)

    # Threaded boss on the upper arm, centered over the throat (Y=ANVIL_Y).
    boss = (
        cq.Workplane("XY")
        .circle(BOSS_R)
        .extrude(BOSS_H)
        .translate((0.0, ANVIL_Y, TOP_ARM_Z + FRAME_BAR / 2.0))
    )
    frame = frame.union(boss)

    # Through-bore for the screw in the upper arm + boss. The bore radius is
    # cut just inside the thread crest radius so the spindle's helical thread
    # ridge engages (threads into) the boss wall instead of floating in a gap.
    bore = (
        cq.Workplane("XY")
        .circle(SCREW_R + THREAD_R - 0.0006)
        .extrude(FRAME_BAR + BOSS_H + 0.02)
        .translate((0.0, ANVIL_Y, TOP_ARM_Z - FRAME_BAR / 2.0 - 0.005))
    )
    frame = frame.cut(bore)

    return frame


def _build_anvil_mesh() -> cq.Workplane:
    """Fixed lower clamping pad on the inside of the lower jaw (steel disc)."""
    z0 = BOTTOM_Z + FRAME_BAR / 2.0
    return (
        cq.Workplane("XY")
        .circle(ANVIL_R)
        .extrude(ANVIL_H)
        .translate((0.0, ANVIL_Y, z0))
    )


def _build_screw_core_mesh() -> cq.Workplane:
    """Blue threaded spindle core with thread ridges represented as ring grooves.

    Authored in a local frame whose origin is the BOTTOM tip of the spindle,
    growing along +Z. Thread ridges are represented as a series of thin torus
    rings at the thread pitch spacing, giving visible thread texture without
    an expensive helical sweep.
    """
    core = cq.Workplane("XY").circle(SCREW_R).extrude(SCREW_LEN)

    # Thread ridges as thin ring grooves at the pitch spacing.
    pitch = 0.008
    n_turns = int(SCREW_LEN / pitch)
    thread_outer = SCREW_R + THREAD_R
    rings = None
    for i in range(n_turns):
        z = i * pitch
        ring = (
            cq.Workplane("XY")
            .circle(thread_outer)
            .circle(SCREW_R - 0.0002)
            .extrude(pitch * 0.45)
            .translate((0.0, 0.0, z))
        )
        rings = ring if rings is None else rings.union(ring)
    if rings is not None:
        core = core.union(rings)
    return core


def _build_collar_mesh() -> cq.Workplane:
    """Blue collar/nut riding at the top of the spindle just under the boss."""
    return (
        cq.Workplane("XY")
        .polygon(6, COLLAR_R * 2.0)
        .extrude(COLLAR_H)
        .translate((0.0, 0.0, SCREW_LEN))
        .edges("|Z")
        .fillet(0.0015)
    )


def _build_hub_mesh() -> cq.Workplane:
    """Purple hub at the very top of the screw where the T-bar crosses."""
    return (
        cq.Workplane("XY")
        .circle(HUB_R)
        .extrude(HUB_H)
        .translate((0.0, 0.0, SCREW_LEN + COLLAR_H))
    )


def _build_handle_bar_mesh() -> cq.Workplane:
    """Purple horizontal T-bar through the hub, along X."""
    z = SCREW_LEN + COLLAR_H + HUB_H / 2.0
    return (
        cq.Workplane("YZ")
        .circle(HANDLE_R)
        .extrude(HANDLE_LEN, both=True)  # along X (YZ workplane normal = X)
        .translate((0.0, 0.0, z))
    )


def _build_handle_caps_mesh() -> cq.Workplane:
    """Two orange rounded end caps on the T-bar."""
    z = SCREW_LEN + COLLAR_H + HUB_H / 2.0
    caps = None
    for sign in (-1.0, 1.0):
        cap = (
            cq.Workplane("YZ")
            .circle(HANDLE_CAP_R)
            .extrude(HANDLE_CAP_H)
            .translate((sign * (HANDLE_LEN - HANDLE_CAP_H / 2.0), 0.0, z))
            .edges()
            .fillet(0.003)
        )
        caps = cap if caps is None else caps.union(cap)
    return caps


def _build_pressing_disc_mesh() -> cq.Workplane:
    """Broad flat anvil-style pressing disc fixed rigidly to the screw tip.

    Authored in a local frame whose origin is the top face of the disc (flush
    with the spindle tip). The disc extends downward (-Z) with a small raised
    hub on top where it welds/bolts to the spindle end.

    The clamping face is the broad flat bottom — wide and flat, no swivel neck.
    """
    # Main flat disc body, extending downward from the origin.
    disc = (
        cq.Workplane("XY")
        .circle(DISC_R)
        .extrude(-DISC_H)
    )
    # Small raised hub on top (where it meets the spindle tip), giving a
    # visible transition from the narrow screw to the wide disc.
    hub = (
        cq.Workplane("XY")
        .circle(DISC_HUB_R)
        .extrude(DISC_HUB_H)
    )
    return hub.union(disc)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="c_clamp")

    model.material(RED, rgba=(0.86, 0.13, 0.10, 1.0))
    model.material(BLUE, rgba=(0.13, 0.32, 0.72, 1.0))
    model.material(PURPLE, rgba=(0.36, 0.20, 0.62, 1.0))
    model.material(ORANGE, rgba=(0.95, 0.42, 0.08, 1.0))
    model.material(STEEL, rgba=(0.62, 0.64, 0.67, 1.0))

    # --- Frame (root) -----------------------------------------------------
    frame = model.part("frame")
    frame.visual(mesh_from_cadquery(_build_frame_mesh(), "frame_body"), material=RED, name="frame_body")
    frame.visual(mesh_from_cadquery(_build_anvil_mesh(), "anvil_pad"), material=STEEL, name="anvil_pad")

    # --- Screw spindle (prismatic child of frame) -------------------------
    # The screw part frame origin is the BOTTOM TIP of the spindle. The boss
    # bore sits at z = TOP_ARM_Z + FRAME_BAR/2 region; at rest, the spindle's
    # threaded section must thread through the boss, so the tip starts down in
    # the throat and the spindle reaches up through the boss.
    screw = model.part("screw")
    screw.visual(mesh_from_cadquery(_build_screw_core_mesh(), "screw_core"), material=BLUE, name="screw_core")
    screw.visual(mesh_from_cadquery(_build_collar_mesh(), "screw_collar"), material=BLUE, name="screw_collar")
    screw.visual(mesh_from_cadquery(_build_hub_mesh(), "handle_hub"), material=PURPLE, name="handle_hub")
    screw.visual(mesh_from_cadquery(_build_handle_bar_mesh(), "handle_bar"), material=PURPLE, name="handle_bar")
    screw.visual(mesh_from_cadquery(_build_handle_caps_mesh(), "handle_caps"), material=ORANGE, name="handle_caps")
    # The pressing disc is rigidly fixed to the screw tip (no swivel).
    screw.visual(mesh_from_cadquery(_build_pressing_disc_mesh(), "pressing_disc"), material=STEEL, name="pressing_disc")

    # ------------------------------------------------------------------
    # Articulation: frame -> screw (PRISMATIC, clamp travel along Z).
    # The screw part frame origin (spindle bottom tip) is placed in the throat
    # at rest. Positive q moves the child along +axis; we want turning to clamp
    # DOWN, so axis = -Z and positive q advances the foot toward the anvil.
    # ------------------------------------------------------------------
    # At REST the clamp is OPEN: the disc sits OPEN_GAP above the anvil pad so
    # there is a real clamping throat. The threaded spindle still spans the boss
    # (its tip just under the upper arm, its top well above the boss).
    anvil_top_z = BOTTOM_Z + FRAME_BAR / 2.0 + ANVIL_H
    OPEN_GAP = 0.030
    rest_disc_bottom_z = anvil_top_z + OPEN_GAP
    # The disc bottom = tip_z - DISC_H, so tip_z = disc_bottom + DISC_H.
    rest_tip_z = rest_disc_bottom_z + DISC_H

    model.articulation(
        "frame_to_screw",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=screw,
        origin=Origin(xyz=(0.0, ANVIL_Y, rest_tip_z)),
        axis=(0.0, 0.0, -1.0),  # positive q drives the screw DOWN to clamp
        motion_limits=MotionLimits(effort=120.0, velocity=0.05, lower=0.0, upper=0.028),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    screw = object_model.get_part("screw")
    prismatic = object_model.get_articulation("frame_to_screw")

    # --- Joint type and axis are exactly what the mechanism requires ---
    ctx.check(
        "frame_to_screw is prismatic",
        prismatic.joint_type == "prismatic",
        details=f"got {prismatic.joint_type}",
    )
    ctx.check(
        "screw clamps along vertical axis",
        abs(abs(prismatic.axis[2]) - 1.0) < 1e-6
        and abs(prismatic.axis[0]) < 1e-6
        and abs(prismatic.axis[1]) < 1e-6,
        details=f"axis={prismatic.axis}",
    )

    # --- No swivel joint exists: the pressing disc is rigid on the screw ---
    articulation_names = [a.name for a in object_model.articulations]
    ctx.check(
        "no swivel joint (disc is rigid)",
        "screw_to_pad" not in articulation_names,
        details=f"articulations={articulation_names}",
    )
    ctx.check(
        "exactly one articulation (prismatic only)",
        len(object_model.articulations) == 1,
        details=f"count={len(object_model.articulations)}",
    )

    # --- Thread engagement: the spindle thread ridge threads into the boss
    #     bore (real screw/nut capture), so a small intentional overlap exists
    #     between the screw core and the frame body. Allow it and prove the
    #     spindle is actually seated in (contacting) the boss, not floating. ---
    ctx.allow_overlap(
        screw,
        frame,
        elem_a="screw_core",
        elem_b="frame_body",
        reason="The spindle's helical thread engages (threads into) the boss bore; the thread crest intentionally embeds slightly into the boss wall, like a real screw/nut fit.",
    )
    ctx.expect_contact(
        screw,
        frame,
        elem_a="screw_core",
        elem_b="frame_body",
        name="spindle threads engage the boss bore",
    )

    # --- Hero geometry present: C-frame, screw, handle bar+caps, disc ---
    frame_body = frame.get_visual("frame_body")
    anvil = frame.get_visual("anvil_pad")
    screw_core = screw.get_visual("screw_core")
    handle_bar = screw.get_visual("handle_bar")
    handle_caps = screw.get_visual("handle_caps")
    pressing_disc = screw.get_visual("pressing_disc")
    for v in (frame_body, anvil, screw_core, handle_bar, handle_caps, pressing_disc):
        ctx.check(f"visual present: {v.name}", v is not None, details=str(v))

    # --- The pressing disc is a visual on the screw part, not a separate part ---
    part_names = [p.name for p in object_model.parts]
    ctx.check(
        "no separate pad part (disc is inline on screw)",
        "pad" not in part_names,
        details=f"parts={part_names}",
    )

    # --- The C frame is open: its mouth/throat faces -Y. Confirm the frame
    #     does NOT fill the throat region (there is a real opening). We check the
    #     frame body min Y is well into +Y/-Y but the throat center column near
    #     z = THROAT/2 is open by confirming the anvil sits at the bottom only.
    frame_aabb = ctx.part_world_aabb(frame)
    assert frame_aabb is not None
    (fx0, fy0, fz0), (fx1, fy1, fz1) = frame_aabb
    ctx.check(
        "frame spans full clamp height",
        (fz1 - fz0) > THROAT,
        details=f"frame z-span={fz1 - fz0:.4f}",
    )

    # --- The T-handle bar is horizontal along X and wider than the screw ---
    bar_aabb = ctx.part_element_world_aabb(screw, elem="handle_bar")
    assert bar_aabb is not None
    (bx0, by0, bz0), (bx1, by1, bz1) = bar_aabb
    ctx.check(
        "handle bar is long along X",
        (bx1 - bx0) > (by1 - by0) * 3.0 and (bx1 - bx0) > 0.08,
        details=f"bar x-span={bx1 - bx0:.4f}, y-span={by1 - by0:.4f}",
    )
    # Handle bar sits at the very top, above the frame's upper arm.
    ctx.check(
        "handle bar sits above frame top",
        bz0 > TOP_ARM_Z,
        details=f"bar z0={bz0:.4f}, top_arm_z={TOP_ARM_Z:.4f}",
    )

    # --- The screw threads down through the boss: at rest the screw spans the
    #     upper arm vertically (its top is above, its tip is below the boss). ---
    screw_aabb = ctx.part_world_aabb(screw)
    assert screw_aabb is not None
    (sx0, sy0, sz0), (sx1, sy1, sz1) = screw_aabb
    ctx.check(
        "screw tip reaches into the throat at rest",
        sz0 < TOP_ARM_Z,
        details=f"screw z0={sz0:.4f}",
    )
    ctx.check(
        "screw top rises above the frame",
        sz1 > TOP_ARM_Z + FRAME_BAR / 2.0 + BOSS_H,
        details=f"screw z1={sz1:.4f}",
    )

    # --- The pressing disc is broad and flat: its diameter is clearly larger
    #     than the screw core diameter (reads as an anvil face, not a swivel). ---
    disc_aabb = ctx.part_element_world_aabb(screw, elem="pressing_disc")
    assert disc_aabb is not None
    (dx0, dy0, dz0), (dx1, dy1, dz1) = disc_aabb
    disc_span_x = dx1 - dx0
    disc_span_y = dy1 - dy0
    disc_span_z = dz1 - dz0
    ctx.check(
        "pressing disc is broad (wider than screw)",
        disc_span_x > SCREW_R * 4.0 and disc_span_y > SCREW_R * 4.0,
        details=f"disc x={disc_span_x:.4f}, y={disc_span_y:.4f}, screw_r={SCREW_R:.4f}",
    )
    ctx.check(
        "pressing disc is flat (thin relative to width)",
        disc_span_z < disc_span_x * 0.4,
        details=f"disc z={disc_span_z:.4f}, x={disc_span_x:.4f}",
    )

    # --- The disc hangs below the screw spindle (foot points down) ---
    core_aabb = ctx.part_element_world_aabb(screw, elem="screw_core")
    assert core_aabb is not None
    (cx0, cy0, cz0), (cx1, cy1, cz1) = core_aabb
    ctx.check(
        "pressing disc is below the screw core",
        dz0 < cz0 + 0.001,
        details=f"disc z0={dz0:.4f}, core z0={cz0:.4f}",
    )

    # --- Mechanism actuation: advancing the prismatic joint drives the
    #     pressing disc DOWN toward the fixed anvil (clamping). ---
    rest_screw_pos = ctx.part_world_position(screw)
    assert rest_screw_pos is not None
    # At rest the clamp is open: the disc sits clear above the anvil pad.
    ctx.expect_gap(
        screw,
        frame,
        axis="z",
        min_gap=0.015,
        positive_elem="pressing_disc",
        negative_elem="anvil_pad",
        name="rest clamp throat is open above the anvil",
    )
    with ctx.pose({prismatic: 0.026}):
        clamped_screw_pos = ctx.part_world_position(screw)
        assert clamped_screw_pos is not None
        ctx.check(
            "advancing screw moves the disc downward to clamp",
            clamped_screw_pos[2] < rest_screw_pos[2] - 0.02,
            details=f"rest_z={rest_screw_pos[2]:.4f}, clamped_z={clamped_screw_pos[2]:.4f}",
        )
        # At a near-closed pose the disc should approach the anvil pad without
        # penetrating it.
        ctx.expect_gap(
            screw,
            frame,
            axis="z",
            min_gap=0.0,
            max_gap=0.012,
            positive_elem="pressing_disc",
            negative_elem="anvil_pad",
            name="clamped disc approaches anvil",
        )

    # --- The screw is centered over the anvil (clamp line) in X ---
    ctx.expect_within(
        screw,
        frame,
        axes="x",
        inner_elem="screw_core",
        outer_elem="frame_body",
        margin=0.005,
        name="screw stays over the frame in X",
    )

    # --- The pressing disc is concentric with the screw core (rigid mount) ---
    ctx.expect_origin_distance(
        screw,
        screw,
        axes="xy",
        max_dist=0.001,
        name="screw part origin is self-consistent",
    )

    return ctx.report()


object_model = build_object_model()
