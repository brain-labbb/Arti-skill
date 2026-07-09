from __future__ import annotations

# ---------------------------------------------------------------------------
# C-clamp (a.k.a. G-clamp), a small hand tool — LEVER variant.
#
# Parent: rec_build-a-realistic-articulated-3d-model-of-a-clam_...
# Variant change: the symmetric T-bar handle is replaced by a single
# side-mounted lever arm that drives the screw, like a quick-action clamp.
#
# Reference image (picture/Handtools/Clamp/001.png):
#   - A red cast-iron "C" / "G" shaped frame. The lower jaw ends in a small
#     fixed anvil pad. The upper arm carries a boss that the screw threads
#     through.
#   - A blue threaded spindle (screw) runs vertically through the top arm's
#     boss into the throat of the C.
#   - At the top of the screw is a hub carrying a SINGLE lever arm extending
#     to one side (replacing the original symmetric T-bar).
#   - At the bottom tip of the screw is a blue swivel clamping pad.
#
# Coordinate convention:
#   +Z is up (the screw axis). The C opens toward -Y (its mouth/throat faces
#   -Y); the solid spine of the C is on +Y. The lever arm extends along +X.
#
# Articulation (real mechanism):
#   - frame_to_lever : REVOLUTE about Z. The lever pivots on the boss top;
#     swinging it drives the screw down via a cam/ratchet mechanism (mimic).
#   - frame_to_screw : PRISMATIC along -Z. Mimic follower of frame_to_lever.
#     Advancing the lever clamps the screw down into the throat.
#   - screw_to_pad   : REVOLUTE. The clamping foot swivels on the screw tip.
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

# ---------------------------------------------------------------------------
# Lever handle (replaces the old symmetric T-bar)
# ---------------------------------------------------------------------------
LEVER_HUB_R = 0.014          # hub disc radius at the pivot
LEVER_HUB_H = 0.016          # hub disc height
LEVER_ARM_R = 0.006          # arm bar radius
LEVER_ARM_LEN = 0.072        # arm reach from hub center to grip start
LEVER_GRIP_R = 0.010         # grip section radius
LEVER_GRIP_LEN = 0.030       # grip section length along X
GRIP_RIDGE_R = 0.0125        # outer radius of each grip ridge ring
GRIP_RIDGE_W = 0.0025        # axial width of each ridge
GRIP_RIDGE_COUNT = 5         # number of grip ridges

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
RUBBER = "clamp_rubber"


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _build_frame_mesh() -> cq.Workplane:
    """Red cast 'C' frame: lower jaw, vertical spine, upper arm, all one body."""
    inner_y = -THROAT_Y / 2.0
    outer_y = SPINE_Y + FRAME_BAR / 2.0

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

    jaw_len_y = SPINE_Y + FRAME_BAR / 2.0 - inner_y
    lower = (
        cq.Workplane("XY")
        .box(FRAME_DEPTH, jaw_len_y, FRAME_BAR, centered=(True, True, True))
        .translate((0.0, (outer_y + inner_y) / 2.0, BOTTOM_Z))
    )

    upper = (
        cq.Workplane("XY")
        .box(FRAME_DEPTH, jaw_len_y, FRAME_BAR, centered=(True, True, True))
        .translate((0.0, (outer_y + inner_y) / 2.0, TOP_ARM_Z))
    )

    frame = spine.union(lower).union(upper)
    frame = frame.edges("|X").fillet(0.006)

    boss = (
        cq.Workplane("XY")
        .circle(BOSS_R)
        .extrude(BOSS_H)
        .translate((0.0, ANVIL_Y, TOP_ARM_Z + FRAME_BAR / 2.0))
    )
    frame = frame.union(boss)

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
    """Blue threaded spindle core.

    Uses the thread crest radius so the thread ridges engage the boss bore
    wall like a real screw/nut fit.
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


# ---------------------------------------------------------------------------
# Lever handle geometry (replaces the old T-bar)
# ---------------------------------------------------------------------------

def _build_lever_hub_mesh() -> cq.Workplane:
    """Purple hub disc at the pivot point (boss top).

    Built in the lever's local frame where the origin is at the hub center.
    The hub is a thick disc (bore is represented by overlap with screw).
    """
    return (
        cq.Workplane("XY")
        .circle(LEVER_HUB_R)
        .extrude(LEVER_HUB_H / 2.0, both=True)
    )


def _build_lever_arm_mesh() -> cq.Workplane:
    """Purple lever arm extending along +X from the hub edge to the grip."""
    # Start at hub edge (not center) to avoid overlapping the screw axis
    arm_start_x = LEVER_HUB_R * 0.85  # slightly inside hub for visual fusion
    arm_length = LEVER_ARM_LEN - arm_start_x
    return (
        cq.Workplane("YZ")
        .circle(LEVER_ARM_R)
        .extrude(arm_length)
        .translate((arm_start_x, 0.0, 0.0))
    )


def _build_lever_grip_mesh() -> cq.Workplane:
    """Orange ergonomic grip section at the end of the lever arm."""
    grip_start_x = LEVER_ARM_LEN
    # Main grip cylinder along X with filleted edges
    return (
        cq.Workplane("YZ")
        .circle(LEVER_GRIP_R)
        .extrude(LEVER_GRIP_LEN)
        .translate((grip_start_x, 0.0, 0.0))
        .edges()
        .fillet(0.003)
    )


def _build_grip_ridge_mesh(index: int) -> cq.Workplane:
    """One annular grip ridge ring at the given index position.

    Shared geometry helper for the repeated grip ridges. Each ridge is a
    short raised ring around the grip section (solid cylinder that protrudes
    beyond the grip radius).
    """
    # Space ridges evenly along the grip, with margins at the ends
    margin = 0.004
    usable = LEVER_GRIP_LEN - 2.0 * margin
    if GRIP_RIDGE_COUNT > 1:
        spacing = usable / (GRIP_RIDGE_COUNT - 1)
    else:
        spacing = 0.0
    x_center = LEVER_ARM_LEN + margin + index * spacing

    return (
        cq.Workplane("YZ")
        .circle(GRIP_RIDGE_R)
        .extrude(GRIP_RIDGE_W)
        .translate((x_center - GRIP_RIDGE_W / 2.0, 0.0, 0.0))
    )


def _build_pad_mesh() -> cq.Workplane:
    """Blue swivel clamping foot.

    Authored with origin at the swivel center (top of the neck).
    """
    neck = (
        cq.Workplane("XY")
        .circle(PAD_NECK_R)
        .extrude(PAD_NECK_H)
    )
    pad = (
        cq.Workplane("XY")
        .circle(PAD_R)
        .extrude(-PAD_H)
        .edges(">Z or <Z")
        .fillet(0.002)
    )
    return neck.union(pad)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="c_clamp_lever")

    model.material(RED, rgba=(0.86, 0.13, 0.10, 1.0))
    model.material(BLUE, rgba=(0.13, 0.32, 0.72, 1.0))
    model.material(PURPLE, rgba=(0.36, 0.20, 0.62, 1.0))
    model.material(ORANGE, rgba=(0.95, 0.42, 0.08, 1.0))
    model.material(STEEL, rgba=(0.62, 0.64, 0.67, 1.0))
    model.material(RUBBER, rgba=(0.18, 0.18, 0.20, 1.0))

    # --- Frame (root) -----------------------------------------------------
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_mesh(), "frame_body"),
        material=RED, name="frame_body",
    )
    frame.visual(
        mesh_from_cadquery(_build_anvil_mesh(), "anvil_pad"),
        material=STEEL, name="anvil_pad",
    )

    # --- Lever (revolute child of frame, pivots on the boss top) ----------
    lever = model.part("lever")
    lever.visual(
        mesh_from_cadquery(_build_lever_hub_mesh(), "lever_hub"),
        material=PURPLE, name="lever_hub",
    )
    lever.visual(
        mesh_from_cadquery(_build_lever_arm_mesh(), "lever_arm"),
        material=PURPLE, name="lever_arm",
    )
    lever.visual(
        mesh_from_cadquery(_build_lever_grip_mesh(), "lever_grip"),
        material=ORANGE, name="lever_grip",
    )
    # Grip ridges: repeated sub-parts emitted via a for loop
    for i in range(GRIP_RIDGE_COUNT):
        lever.visual(
            mesh_from_cadquery(_build_grip_ridge_mesh(i), f"grip_ridge_{i}"),
            material=RUBBER, name=f"grip_ridge_{i}",
        )

    # --- Screw spindle (prismatic child of frame, mimic of lever) ---------
    screw = model.part("screw")
    screw.visual(
        mesh_from_cadquery(_build_screw_core_mesh(), "screw_core"),
        material=BLUE, name="screw_core",
    )
    screw.visual(
        mesh_from_cadquery(_build_collar_mesh(), "screw_collar"),
        material=BLUE, name="screw_collar",
    )

    # --- Swivel pad (revolute child of screw) -----------------------------
    pad = model.part("pad")
    pad.visual(
        mesh_from_cadquery(_build_pad_mesh(), "swivel_pad"),
        material=BLUE, name="swivel_pad",
    )

    # ------------------------------------------------------------------
    # Articulation 1: frame -> lever (REVOLUTE about Z, pivot on boss top).
    # The lever hub sits on the boss top. Positive q swings the arm from
    # rest (along +X) toward +Y (counterclockwise when viewed from above).
    # ------------------------------------------------------------------
    boss_top_z = TOP_ARM_Z + FRAME_BAR / 2.0 + BOSS_H
    # Hub center sits on top of the boss: bottom face of hub touches boss top
    lever_pivot_z = boss_top_z + LEVER_HUB_H / 2.0

    model.articulation(
        "frame_to_lever",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=lever,
        origin=Origin(xyz=(0.0, ANVIL_Y, lever_pivot_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0, lower=0.0, upper=1.5),
    )

    # ------------------------------------------------------------------
    # Articulation 2: frame -> screw (PRISMATIC, clamp travel along -Z).
    # Mimic follower of frame_to_lever: swinging the lever drives the
    # screw down. multiplier maps lever angle to screw travel.
    # ------------------------------------------------------------------
    anvil_top_z = BOTTOM_Z + FRAME_BAR / 2.0 + ANVIL_H
    OPEN_GAP = 0.030
    rest_foot_bottom_z = anvil_top_z + OPEN_GAP
    rest_tip_z = rest_foot_bottom_z + PAD_NECK_H + PAD_H

    screw_travel_max = 0.028

    model.articulation(
        "frame_to_screw",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=screw,
        origin=Origin(xyz=(0.0, ANVIL_Y, rest_tip_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=120.0, velocity=0.05, lower=0.0, upper=screw_travel_max,
        ),
    )

    # ------------------------------------------------------------------
    # Articulation 3: screw -> pad (REVOLUTE swivel of the clamping foot).
    # ------------------------------------------------------------------
    model.articulation(
        "screw_to_pad",
        ArticulationType.REVOLUTE,
        parent=screw,
        child=pad,
        origin=Origin(xyz=(0.0, 0.0, -PAD_NECK_H)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-0.30, upper=0.30),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    lever = object_model.get_part("lever")
    screw = object_model.get_part("screw")
    pad = object_model.get_part("pad")
    lever_joint = object_model.get_articulation("frame_to_lever")
    prismatic = object_model.get_articulation("frame_to_screw")
    swivel = object_model.get_articulation("screw_to_pad")

    # --- Joint types and axes are exactly what the mechanism requires ---
    ctx.check(
        "frame_to_lever is revolute",
        lever_joint.joint_type == "revolute",
        details=f"got {lever_joint.joint_type}",
    )
    ctx.check(
        "lever pivots about vertical axis",
        abs(abs(lever_joint.axis[2]) - 1.0) < 1e-6
        and abs(lever_joint.axis[0]) < 1e-6
        and abs(lever_joint.axis[1]) < 1e-6,
        details=f"axis={lever_joint.axis}",
    )
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

    # --- The lever is mounted on the frame (not on the screw) ---
    ctx.check(
        "lever parent is frame",
        lever_joint.parent == "frame",
        details=f"parent={lever_joint.parent}",
    )
    ctx.check(
        "screw parent is frame",
        prismatic.parent == "frame",
        details=f"parent={prismatic.parent}",
    )

    # --- Hero geometry present: frame, lever, screw, pad ---
    frame_body = frame.get_visual("frame_body")
    anvil = frame.get_visual("anvil_pad")
    lever_hub = lever.get_visual("lever_hub")
    lever_arm = lever.get_visual("lever_arm")
    lever_grip = lever.get_visual("lever_grip")
    screw_core = screw.get_visual("screw_core")
    swivel_pad = pad.get_visual("swivel_pad")
    for v in (frame_body, anvil, lever_hub, lever_arm, lever_grip, screw_core, swivel_pad):
        ctx.check(f"visual present: {v.name}", v is not None, details=str(v))

    # --- Grip ridges present (repeated sub-parts) ---
    for i in range(GRIP_RIDGE_COUNT):
        ridge = lever.get_visual(f"grip_ridge_{i}")
        ctx.check(f"grip ridge {i} present", ridge is not None)

    # --- Lever arm is ASYMMETRIC: extends to one side only (+X) ---
    arm_aabb = ctx.part_element_world_aabb(lever, elem="lever_arm")
    assert arm_aabb is not None
    (ax0, ay0, az0), (ax1, ay1, az1) = arm_aabb
    arm_x_span = ax1 - ax0
    # The arm should extend significantly in X (at least 50mm)
    ctx.check(
        "lever arm is long along X",
        arm_x_span > 0.050,
        details=f"arm x-span={arm_x_span:.4f}",
    )
    # The arm should NOT extend symmetrically: min X should be near 0
    # (the hub center), not far into -X like a T-bar
    lever_pos = ctx.part_world_position(lever)
    assert lever_pos is not None
    ctx.check(
        "lever arm extends to one side only (not symmetric T-bar)",
        ax0 > lever_pos[0] - 0.020,  # arm start is near the pivot, not far in -X
        details=f"arm min_x={ax0:.4f}, lever_x={lever_pos[0]:.4f}",
    )

    # --- Lever sits at the top of the frame, above the boss ---
    hub_aabb = ctx.part_element_world_aabb(lever, elem="lever_hub")
    assert hub_aabb is not None
    (_, _, hz0), (_, _, hz1) = hub_aabb
    boss_top_z = TOP_ARM_Z + FRAME_BAR / 2.0 + BOSS_H
    ctx.check(
        "lever hub sits at boss top",
        hz0 >= boss_top_z - 0.005 and hz1 <= boss_top_z + LEVER_HUB_H + 0.005,
        details=f"hub z=[{hz0:.4f}, {hz1:.4f}], boss_top={boss_top_z:.4f}",
    )

    # --- The old T-bar visuals should NOT exist on the screw ---
    screw_visual_names = [v.name for v in screw.visuals]
    ctx.check(
        "old T-bar handle_bar removed from screw",
        "handle_bar" not in screw_visual_names,
        details=f"screw visuals: {screw_visual_names}",
    )
    ctx.check(
        "old T-bar handle_caps removed from screw",
        "handle_caps" not in screw_visual_names,
        details=f"screw visuals: {screw_visual_names}",
    )

    # --- Thread engagement: spindle threads into the boss bore ---
    ctx.allow_overlap(
        screw,
        frame,
        elem_a="screw_core",
        elem_b="frame_body",
        reason="The spindle's helical thread engages the boss bore like a real screw/nut fit.",
    )
    ctx.expect_contact(
        screw,
        frame,
        elem_a="screw_core",
        elem_b="frame_body",
        name="spindle threads engage the boss bore",
    )

    # --- The screw passes through the lever hub bore (intentional overlap) ---
    ctx.allow_overlap(
        screw,
        lever,
        elem_a="screw_core",
        elem_b="lever_hub",
        reason="The screw spindle passes through the lever hub bore; the hub captures the screw like a real bearing seat.",
    )
    ctx.expect_contact(
        screw,
        lever,
        elem_a="screw_core",
        elem_b="lever_hub",
        name="screw passes through lever hub bore",
    )

    # --- Frame spans full clamp height ---
    frame_aabb = ctx.part_world_aabb(frame)
    assert frame_aabb is not None
    (fx0, fy0, fz0), (fx1, fy1, fz1) = frame_aabb
    ctx.check(
        "frame spans full clamp height",
        (fz1 - fz0) > THROAT,
        details=f"frame z-span={fz1 - fz0:.4f}",
    )

    # --- Screw tip reaches into the throat at rest ---
    screw_aabb = ctx.part_world_aabb(screw)
    assert screw_aabb is not None
    (sx0, sy0, sz0), (sx1, sy1, sz1) = screw_aabb
    ctx.check(
        "screw tip reaches into the throat at rest",
        sz0 < TOP_ARM_Z,
        details=f"screw z0={sz0:.4f}",
    )

    # --- Swivel pad hangs below the screw tip ---
    pad_aabb = ctx.part_world_aabb(pad)
    assert pad_aabb is not None
    (px0, py0, pz0), (px1, py1, pz1) = pad_aabb
    ctx.check(
        "swivel pad is below the screw spindle",
        pz0 < sz0 + 0.001,
        details=f"pad z0={pz0:.4f}, screw z0={sz0:.4f}",
    )

    # --- Mechanism: swinging the lever drives the screw down to clamp ---
    rest_pad = ctx.part_world_position(pad)
    assert rest_pad is not None

    # At rest (lever=0), clamp is open
    ctx.expect_gap(
        pad,
        frame,
        axis="z",
        min_gap=0.015,
        positive_elem="swivel_pad",
        negative_elem="anvil_pad",
        name="rest clamp throat is open above the anvil",
    )

    # Advance the screw directly: pad should drive down to clamp
    with ctx.pose({prismatic: 0.026}):
        clamped_pad = ctx.part_world_position(pad)
        assert clamped_pad is not None
        ctx.check(
            "advancing screw moves the pad downward to clamp",
            clamped_pad[2] < rest_pad[2] - 0.02,
            details=f"rest_z={rest_pad[2]:.4f}, clamped_z={clamped_pad[2]:.4f}",
        )
        ctx.expect_gap(
            pad,
            frame,
            axis="z",
            min_gap=0.0,
            max_gap=0.012,
            positive_elem="swivel_pad",
            negative_elem="anvil_pad",
            name="clamped foot approaches anvil after screw advance",
        )

    # --- Lever swing actually rotates the arm (not just translating) ---
    rest_arm_center = ctx.part_element_world_aabb(lever, elem="lever_arm")
    assert rest_arm_center is not None
    (_, _, raz0), (_, _, raz1) = rest_arm_center
    with ctx.pose({lever_joint: 1.0}):
        swung_arm_center = ctx.part_element_world_aabb(lever, elem="lever_arm")
        assert swung_arm_center is not None
        (_, say0, _), (_, say1, _) = swung_arm_center
        # The arm's Y extent should change when swung (it was along X, now partially along Y)
        rest_arm_y = ay1 - ay0
        swung_arm_y = say1 - say0
        ctx.check(
            "lever arm Y-extent changes when swung",
            abs(swung_arm_y - rest_arm_y) > 0.010,
            details=f"rest_y={rest_arm_y:.4f}, swung_y={swung_arm_y:.4f}",
        )

    # --- Screw stays over the frame in X ---
    ctx.expect_within(
        screw,
        frame,
        axes="x",
        inner_elem="screw_core",
        outer_elem="frame_body",
        margin=0.005,
        name="screw stays over the frame in X",
    )

    # --- Pad hangs concentric with the screw ---
    ctx.expect_origin_distance(
        screw,
        pad,
        axes="xy",
        max_dist=0.006,
        name="pad hangs concentric with the screw",
    )

    # --- Swivel actuation ---
    with ctx.pose({swivel: 0.30}):
        tilted_pad = ctx.part_element_world_aabb(pad, elem="swivel_pad")
        assert tilted_pad is not None

    return ctx.report()


object_model = build_object_model()
