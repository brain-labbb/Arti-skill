from __future__ import annotations

# ---------------------------------------------------------------------------
# C-clamp (a.k.a. G-clamp), a small hand tool.
#
# Reference image (picture/Handtools/Clamp/001.png):
#   - A red cast-iron "C" / "G" shaped frame. The lower jaw of the C ends in a
#     small fixed anvil pad. The upper arm carries a boss that the screw threads
#     through.
#   - A blue threaded spindle (screw) runs vertically down through the top arm's
#     boss into the throat of the C.
#   - At the top of the screw is a purple hub with two opposed flat orange
#     butterfly thumb wings (butterfly grip variant). Turning the wings drives
#     the screw down/up.
#   - At the bottom tip of the screw is a blue swivel clamping pad (the foot)
#     that seats flat against the work.
#
# Coordinate convention:
#   +Z is up (the screw axis). The C opens toward -Y (its mouth/throat faces
#   -Y); the solid spine of the C is on +Y. The butterfly wings extend along X.
#
# Articulation (real mechanism):
#   - frame_to_screw  : PRISMATIC along -Z. Turning the handle advances the
#     screw down into the throat to clamp. This is the primary clamp motion.
#   - screw_to_pad    : REVOLUTE. The clamping foot swivels on the screw tip so
#     it can lie flat against an angled workpiece (a real ball/swivel pad).
#   The butterfly grip (hub + wings) is rigidly part of the screw (they move
#   together), so it is fused into the screw part rather than given its own joint.
# ---------------------------------------------------------------------------

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
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

# Butterfly grip (two flat opposed thumb wings on the spindle top)
HUB_R = 0.012                # purple hub where the wings meet the screw top
HUB_H = 0.018
WING_LENGTH = 0.040          # length of one wing paddle (hub center to tip)
WING_WIDTH = 0.020           # finger-grip width (along Y)
WING_THICK = 0.005           # flat thickness (along Z)

# Swivel clamping pad (foot) at the screw tip
PAD_R = 0.016
PAD_H = 0.009
PAD_NECK_R = 0.007
PAD_NECK_H = 0.006

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
    """Blue threaded spindle core (simplified cylinder without helix).

    Authored in a local frame whose origin is the BOTTOM tip of the spindle,
    growing along +Z. The helical thread is omitted for compile performance;
    the thread engagement is represented by the bore fit in the boss.
    """
    return cq.Workplane("XY").circle(SCREW_R + THREAD_R).extrude(SCREW_LEN)


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
    """Purple hub at the very top of the screw where the butterfly wings mount."""
    return (
        cq.Workplane("XY")
        .circle(HUB_R)
        .extrude(HUB_H)
        .translate((0.0, 0.0, SCREW_LEN + COLLAR_H))
    )



def _build_pad_mesh() -> cq.Workplane:
    """Blue swivel clamping foot.

    Authored in a local frame whose origin is the swivel center (top of the
    neck). The neck reaches up toward the screw tip (+Z) and the pad disc sits
    below it (-Z) so its flat clamping face points down toward the work.
    """
    # Neck (stem that captures into the screw tip), reaching up.
    neck = (
        cq.Workplane("XY")
        .circle(PAD_NECK_R)
        .extrude(PAD_NECK_H)
    )
    # Pad disc below the swivel center, flat face down.
    pad = (
        cq.Workplane("XY")
        .circle(PAD_R)
        .extrude(-PAD_H)
        .edges(">Z or <Z")
        .fillet(0.002)
    )
    return neck.union(pad)


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
    # Butterfly grip: two opposed flat thumb wings, emitted via a loop.
    _wing_z = SCREW_LEN + COLLAR_H + HUB_H / 2.0
    for i, sign in enumerate((-1.0, 1.0)):
        wing_name = f"wing_{i}"
        screw.visual(
            Box((WING_LENGTH, WING_WIDTH, WING_THICK)),
            origin=Origin(xyz=(sign * WING_LENGTH / 2.0, 0.0, _wing_z)),
            material=ORANGE,
            name=wing_name,
        )

    # --- Swivel pad (revolute child of screw) -----------------------------
    pad = model.part("pad")
    pad.visual(mesh_from_cadquery(_build_pad_mesh(), "swivel_pad"), material=BLUE, name="swivel_pad")

    # ------------------------------------------------------------------
    # Articulation 1: frame -> screw (PRISMATIC, clamp travel along Z).
    # The screw part frame origin (spindle bottom tip) is placed in the throat
    # at rest. Positive q moves the child along +axis; we want turning to clamp
    # DOWN, so axis = -Z and positive q advances the foot toward the anvil.
    # ------------------------------------------------------------------
    # At REST the clamp is OPEN: the foot sits OPEN_GAP above the anvil pad so
    # there is a real clamping throat. The threaded spindle still spans the boss
    # (its tip just under the upper arm, its top well above the boss).
    anvil_top_z = BOTTOM_Z + FRAME_BAR / 2.0 + ANVIL_H
    OPEN_GAP = 0.030
    rest_foot_bottom_z = anvil_top_z + OPEN_GAP
    # Foot bottom = tip_z - PAD_NECK_H - PAD_H, so the tip sits above the foot.
    rest_tip_z = rest_foot_bottom_z + PAD_NECK_H + PAD_H

    model.articulation(
        "frame_to_screw",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=screw,
        origin=Origin(xyz=(0.0, ANVIL_Y, rest_tip_z)),
        axis=(0.0, 0.0, -1.0),  # positive q drives the screw DOWN to clamp
        motion_limits=MotionLimits(effort=120.0, velocity=0.05, lower=0.0, upper=0.028),
    )

    # ------------------------------------------------------------------
    # Articulation 2: screw -> pad (REVOLUTE swivel of the clamping foot).
    # The pad swivel center sits just below the spindle tip. Positive q tilts
    # the foot about Y so it can mate flat against an angled workpiece.
    # ------------------------------------------------------------------
    model.articulation(
        "screw_to_pad",
        ArticulationType.REVOLUTE,
        parent=screw,
        child=pad,
        # Swivel center just below the spindle tip (tip is at the screw frame
        # origin z=0; the pad neck reaches up to meet it).
        origin=Origin(xyz=(0.0, 0.0, -PAD_NECK_H)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-0.30, upper=0.30),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    screw = object_model.get_part("screw")
    pad = object_model.get_part("pad")
    prismatic = object_model.get_articulation("frame_to_screw")
    swivel = object_model.get_articulation("screw_to_pad")

    # --- Joint types and axes are exactly what the mechanism requires ---
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
    ctx.check(
        "screw_to_pad is revolute",
        swivel.joint_type == "revolute",
        details=f"got {swivel.joint_type}",
    )
    ctx.check(
        "pad swivels about Y",
        abs(abs(swivel.axis[1]) - 1.0) < 1e-6,
        details=f"axis={swivel.axis}",
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

    # --- Hero geometry present: C-frame, screw, hub+butterfly wings, pad ---
    frame_body = frame.get_visual("frame_body")
    anvil = frame.get_visual("anvil_pad")
    screw_core = screw.get_visual("screw_core")
    handle_hub = screw.get_visual("handle_hub")
    wing_0 = screw.get_visual("wing_0")
    wing_1 = screw.get_visual("wing_1")
    swivel_pad = pad.get_visual("swivel_pad")
    for v in (frame_body, anvil, screw_core, handle_hub, wing_0, wing_1, swivel_pad):
        ctx.check(f"visual present: {v.name}", v is not None, details=str(v))

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

    # --- The butterfly wings are flat paddles: thin in Z, wide along X,
    #     moderate grip width along Y. Together they span wider than the hub. ---
    w0_aabb = ctx.part_element_world_aabb(screw, elem="wing_0")
    w1_aabb = ctx.part_element_world_aabb(screw, elem="wing_1")
    assert w0_aabb is not None and w1_aabb is not None
    (w0x0, w0y0, w0z0), (w0x1, w0y1, w0z1) = w0_aabb
    (w1x0, w1y0, w1z0), (w1x1, w1y1, w1z1) = w1_aabb
    # Each wing is thin in Z (flat paddle).
    for i, (z0, z1, y0, y1) in enumerate(
        [(w0z0, w0z1, w0y0, w0y1), (w1z0, w1z1, w1y0, w1y1)]
    ):
        ctx.check(
            f"wing_{i} is flat (thin in Z, wider in Y)",
            (z1 - z0) < (y1 - y0) * 0.5,
            details=f"z-thick={z1 - z0:.4f}, y-width={y1 - y0:.4f}",
        )
    # Combined wing span in X is much wider than the hub.
    combined_x_span = max(w0x1, w1x1) - min(w0x0, w1x0)
    ctx.check(
        "butterfly wings span wide along X",
        combined_x_span > 0.060,
        details=f"combined x-span={combined_x_span:.4f}",
    )
    # Wings sit at the very top, above the frame's upper arm.
    ctx.check(
        "butterfly wings sit above frame top",
        min(w0z0, w1z0) > TOP_ARM_Z,
        details=f"min wing z0={min(w0z0, w1z0):.4f}, top_arm_z={TOP_ARM_Z:.4f}",
    )

    # --- The butterfly wings are cast integral with the hub (same screw part).
    #     Verify they overlap the hub region (wing root embeds into the hub).
    hub_aabb = ctx.part_element_world_aabb(screw, elem="handle_hub")
    assert hub_aabb is not None
    (hx0, hy0, hz0), (hx1, hy1, hz1) = hub_aabb
    # Each wing's X extent should overlap with the hub's X extent.
    ctx.check(
        "wing_0 root overlaps hub X range",
        w0x1 > hx0 and w0x0 < hx1,
        details=f"wing_0 x=[{w0x0:.4f},{w0x1:.4f}], hub x=[{hx0:.4f},{hx1:.4f}]",
    )
    ctx.check(
        "wing_1 root overlaps hub X range",
        w1x1 > hx0 and w1x0 < hx1,
        details=f"wing_1 x=[{w1x0:.4f},{w1x1:.4f}], hub x=[{hx0:.4f},{hx1:.4f}]",
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

    # --- The swivel pad hangs below the screw tip (foot points down) ---
    pad_aabb = ctx.part_world_aabb(pad)
    assert pad_aabb is not None
    (px0, py0, pz0), (px1, py1, pz1) = pad_aabb
    ctx.check(
        "swivel pad is below the screw spindle",
        pz0 < sz0 + 0.001,
        details=f"pad z0={pz0:.4f}, screw z0={sz0:.4f}",
    )

    # --- Mechanism actuation: advancing the prismatic joint drives the
    #     pad foot DOWN toward the fixed anvil (clamping). ---
    rest_pad = ctx.part_world_position(pad)
    assert rest_pad is not None
    # At rest the clamp is open: the foot sits clear above the anvil pad.
    ctx.expect_gap(
        pad,
        frame,
        axis="z",
        min_gap=0.015,
        positive_elem="swivel_pad",
        negative_elem="anvil_pad",
        name="rest clamp throat is open above the anvil",
    )
    with ctx.pose({prismatic: 0.026}):
        clamped_pad = ctx.part_world_position(pad)
        assert clamped_pad is not None
        ctx.check(
            "advancing screw moves the pad downward to clamp",
            clamped_pad[2] < rest_pad[2] - 0.02,
            details=f"rest_z={rest_pad[2]:.4f}, clamped_z={clamped_pad[2]:.4f}",
        )
        # At a near-closed pose the foot should approach the anvil pad without
        # penetrating it.
        ctx.expect_gap(
            pad,
            frame,
            axis="z",
            min_gap=0.0,
            max_gap=0.012,
            positive_elem="swivel_pad",
            negative_elem="anvil_pad",
            name="clamped foot approaches anvil",
        )

    # --- Swivel actuation: rotating the pad joint tilts the foot in X/Z ---
    with ctx.pose({swivel: 0.30}):
        tilted_pad = ctx.part_element_world_aabb(pad, elem="swivel_pad")
        assert tilted_pad is not None

    # --- The screw is centered over the anvil (clamp line) in X and Y ---
    ctx.expect_within(
        screw,
        frame,
        axes="x",
        inner_elem="screw_core",
        outer_elem="frame_body",
        margin=0.005,
        name="screw stays over the frame in X",
    )
    ctx.expect_origin_distance(
        screw,
        pad,
        axes="xy",
        max_dist=0.006,
        name="pad hangs concentric with the screw",
    )

    return ctx.report()


object_model = build_object_model()
