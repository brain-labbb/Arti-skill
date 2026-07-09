from __future__ import annotations

# Realistic articulated clipboard with a foldback (butterfly) clip fastener.
#
# Object identity:
#   A letter/A4-size blue plastic clipboard with a foldback clip fastener
#   mounted at the top edge. The foldback clip replaces the parent torsion-spring
#   jaw: a folded sheet-metal channel body grips the board top edge, and two flat
#   wire handle arms pivot on the channel rims. Each arm swings from upright
#   (loading / squeezing position) to flat (stowed against the channel body).
#
# Layout:
#   Board lies flat in the XY plane, top surface facing +Z. The channel sits at
#   the -X (top) edge. Channel width runs along Y. Wire arms are mirrored left
#   and right (±Y), each pivoting about a local X axis at the channel rim so
#   positive q folds the arm inward onto the channel top surface.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
    WirePath,
)

# ---------------------------------------------------------------------------
# Real-world dimensions (meters)
# ---------------------------------------------------------------------------
BOARD_LEN = 0.330       # along +X (long dimension)
BOARD_WID = 0.232       # along Y  (short dimension)
BOARD_THK = 0.0032      # plastic panel thickness
BOARD_CORNER_R = 0.012

# Board occupies x in [0, BOARD_LEN], centered on Y, top face at z = BOARD_THK.

# -- Foldback clip channel (sheet-metal body) --
CHANNEL_WID = 0.070     # width along Y
CHANNEL_DEPTH = 0.028   # front-to-back along X
CHANNEL_RIDGE_H = 0.015 # height of the ridge above board top
CHANNEL_WALL_THK = 0.0015
CHANNEL_CENTER_X = 0.034
CHANNEL_TOP_HD = CHANNEL_DEPTH / 2.0 * 0.40  # half-depth at the narrower ridge

# -- Wire handle arms --
WIRE_DIAM = 0.002
ARM_HEIGHT = 0.032      # grip bar height above pivot
ARM_PIN_HALF = 0.010    # half-spacing between the two wire legs along X

# Pivot sits slightly above the channel ridge so the wire clears the surface
# when folded flat.
PIVOT_Z = BOARD_THK + CHANNEL_RIDGE_H + WIRE_DIAM / 2.0 + 0.0005

# Materials
MAT_BOARD = "clipboard_blue"
MAT_CHANNEL = "spring_steel"
MAT_WIRE = "zinc_wire"


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def _board_solid() -> cq.Workplane:
    """Thin rounded-corner plastic panel, top face at z = BOARD_THK.
    
    With centered=(False, True, False), the box already extends from z=0 to
    z=BOARD_THK, so no Z translation is needed.
    """
    board = (
        cq.Workplane("XY")
        .box(BOARD_LEN, BOARD_WID, BOARD_THK, centered=(False, True, False))
        .edges("|Z")
        .fillet(BOARD_CORNER_R)
    )
    return board


def _channel_body() -> cq.Workplane:
    """Folded sheet-metal channel of the foldback clip, in world coordinates.

    A hollow trapezoidal arch extruded along Y with pivot-pin bosses at the
    channel rims where the wire arms attach.
    """
    cx = CHANNEL_CENTER_X
    hw = CHANNEL_WID / 2.0
    hd = CHANNEL_DEPTH / 2.0
    thd = CHANNEL_TOP_HD
    t = CHANNEL_WALL_THK
    base_z = BOARD_THK
    top_z = BOARD_THK + CHANNEL_RIDGE_H

    # Outer trapezoidal shell (wider base, narrower ridge).
    outer = (
        cq.Workplane("XZ")
        .moveTo(cx - hd, base_z)
        .lineTo(cx + hd, base_z)
        .lineTo(cx + thd, top_z)
        .lineTo(cx - thd, top_z)
        .close()
        .extrude(hw, both=True)
    )

    # Inner cavity — open at the bottom so the channel straddles the board edge.
    inner = (
        cq.Workplane("XZ")
        .moveTo(cx - hd + t * 1.8, base_z - 0.0005)
        .lineTo(cx + hd - t * 1.8, base_z - 0.0005)
        .lineTo(cx + thd - t * 0.9, top_z - t)
        .lineTo(cx - thd + t * 0.9, top_z - t)
        .close()
        .extrude(hw - t, both=True)
    )

    channel = outer.cut(inner)

    # Pivot-ear plates and pin bosses at each rim.  Each ear is a vertical
    # plate that rises from inside the channel body up to the pivot height,
    # ensuring a connected solid from the channel shell to the pivot pin.
    pin_r = 0.0022
    pin_len = 0.006
    ear_thk_y = 0.003        # ear plate thickness along Y
    ear_span_x = 0.008       # ear plate width along X
    ear_dip = 0.005          # how far the ear extends below the channel top

    for sign_y in (-1.0, 1.0):
        for sign_x in (-1.0, 1.0):
            ear_cx = cx + sign_x * ARM_PIN_HALF
            ear_bot = top_z - ear_dip          # inside the channel body
            ear_top = PIVOT_Z + pin_r + 0.001  # slightly above the pin
            ear_h = ear_top - ear_bot
            ear_cz = (ear_bot + ear_top) / 2.0

            # Vertical ear plate overlapping the channel side wall in Y and
            # extending from inside the body up past the ridge.
            ear_plate = (
                cq.Workplane("XY")
                .box(ear_span_x, ear_thk_y, ear_h)
                .translate((ear_cx, sign_y * (hw - ear_thk_y / 2.0), ear_cz))
            )
            channel = channel.union(ear_plate)

            # Pin cylinder along X protruding outward from the ear.
            pin = (
                cq.Workplane("YZ")
                .workplane(offset=ear_cx - pin_len / 2.0)
                .center(sign_y * (hw + pin_len * 0.15), PIVOT_Z)
                .circle(pin_r)
                .extrude(pin_len)
            )
            channel = channel.union(pin)

    # Gripping lips at the channel feet — thin ridges that press on the paper.
    # They extend slightly into the channel body so the mesh is connected.
    lip_thk = 0.0012
    lip_w = 0.004
    for sign_x in (-1.0, 1.0):
        lip = (
            cq.Workplane("XY")
            .box(lip_w, CHANNEL_WID * 0.92, lip_thk * 2.5)
            .translate((cx + sign_x * (hd - lip_w / 2.0), 0.0, base_z + lip_thk * 0.5))
        )
        channel = channel.union(lip)

    return channel


def _wire_arm_points() -> list[tuple[float, float, float]]:
    """U-shaped spline points for one wire handle arm, in joint-local frame.

    Origin is the pivot point.  +Z is up (arm extends upward at q = 0).
    The wire path traces from one leg bottom, up to the grip bar, and back
    down to the other leg bottom.
    """
    ph = ARM_PIN_HALF
    h = ARM_HEIGHT
    # Dense intermediate points keep the catmull-rom spline close to the
    # intended rectangular-U shape with gentle corner rounding.
    pts: list[tuple[float, float, float]] = []
    n_leg = 6  # points per vertical leg
    n_bar = 5  # points across the top grip bar
    for k in range(n_leg + 1):
        frac = k / n_leg
        pts.append((-ph, 0.0, 0.002 + (h - 0.002) * frac))
    for k in range(1, n_bar):
        frac = k / n_bar
        pts.append((-ph + 2.0 * ph * frac, 0.0, h))
    for k in range(1, n_leg + 1):
        frac = 1.0 - k / n_leg
        pts.append((ph, 0.0, 0.002 + (h - 0.002) * frac))
    return pts


def _wire_arm_mesh() -> "MeshGeometry":  # noqa: F821  (type is imported at runtime)
    """Build the managed mesh for one wire handle arm."""
    return tube_from_spline_points(
        _wire_arm_points(),
        radius=WIRE_DIAM / 2.0,
        samples_per_segment=10,
        radial_segments=12,
        cap_ends=True,
        spline="catmull_rom",
        up_hint=(0.0, 1.0, 0.0),
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clipboard_foldback_clip")

    model.material(MAT_BOARD, rgba=(0.13, 0.45, 0.92, 1.0))
    model.material(MAT_CHANNEL, rgba=(0.22, 0.22, 0.26, 1.0))
    model.material(MAT_WIRE, rgba=(0.62, 0.64, 0.68, 1.0))

    # --- Board (root) ---
    board = model.part("board")
    board.visual(
        mesh_from_cadquery(_board_solid(), "board_panel"),
        material=MAT_BOARD,
        name="board_panel",
    )

    # --- Channel body (fixed to board top edge) ---
    channel = model.part("channel")
    channel.visual(
        mesh_from_cadquery(_channel_body(), "channel_body"),
        material=MAT_CHANNEL,
        name="channel_body",
    )

    # --- Wire arms (two revolute children of channel, emitted via loop) ---
    arm_mesh_geom = _wire_arm_mesh()
    arm_parts = []
    for i in range(2):
        arm = model.part(f"arm_{i}")
        arm.visual(
            mesh_from_geometry(arm_mesh_geom.clone(), f"wire_arm_{i}"),
            material=MAT_WIRE,
            name=f"wire_arm_{i}",
        )
        arm_parts.append(arm)

    # Board → channel (fixed mount at board top edge).
    model.articulation(
        "board_to_channel",
        ArticulationType.FIXED,
        parent=board,
        child=channel,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # Channel → each wire arm (revolute on the channel rim).
    # Left arm (i=0, sign=-1): axis (-1,0,0) so positive q folds inward (+Y).
    # Right arm (i=1, sign=+1): axis (+1,0,0) so positive q folds inward (-Y).
    for i in range(2):
        sign = -1.0 if i == 0 else 1.0
        model.articulation(
            f"channel_to_arm_{i}",
            ArticulationType.REVOLUTE,
            parent=channel,
            child=arm_parts[i],
            origin=Origin(xyz=(CHANNEL_CENTER_X, sign * CHANNEL_WID / 2.0, PIVOT_Z)),
            axis=(sign, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0, velocity=4.0, lower=0.0, upper=1.40,
            ),
        )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    board = object_model.get_part("board")
    channel = object_model.get_part("channel")
    arm_0 = object_model.get_part("arm_0")
    arm_1 = object_model.get_part("arm_1")
    joint_0 = object_model.get_articulation("channel_to_arm_0")
    joint_1 = object_model.get_articulation("channel_to_arm_1")

    # ---- Board hero geometry: large thin flat panel ----
    bmin, bmax = ctx.part_world_aabb(board)
    board_len = bmax[0] - bmin[0]
    board_wid = bmax[1] - bmin[1]
    board_thk = bmax[2] - bmin[2]
    ctx.check(
        "board reads as a wide thin clipboard panel",
        board_len > 0.30 and board_wid > 0.21 and board_thk < 0.006,
        details=f"len={board_len:.3f} wid={board_wid:.3f} thk={board_thk:.4f}",
    )

    # ---- Channel sits at the top edge of the board ----
    cmin, cmax = ctx.part_world_aabb(channel)
    ctx.check(
        "channel is near the top (-X) edge of the board",
        cmax[0] < 0.10 and cmin[0] >= bmin[0] - 1e-4,
        details=f"ch x=[{cmin[0]:.3f},{cmax[0]:.3f}] board x0={bmin[0]:.3f}",
    )
    ctx.check(
        "channel rises above the board top face (arch shape)",
        cmax[2] > BOARD_THK + 0.008,
        details=f"ch top z={cmax[2]:.4f} board top z={bmax[2]:.4f}",
    )

    # Channel is mounted on the board (overlap in XY, seats on top face in Z).
    ctx.expect_overlap(channel, board, axes="xy", min_overlap=0.015)
    ctx.expect_gap(
        channel, board, axis="z",
        max_gap=0.0005, max_penetration=0.002,
        name="channel seats on the board top face",
    )

    # ---- Channel width spans a meaningful fraction of the board ----
    ch_wid = cmax[1] - cmin[1]
    ctx.check(
        "channel width is at least 50 mm",
        ch_wid > 0.050,
        details=f"ch_wid={ch_wid:.3f}",
    )

    # ---- Two wire arms exist and are revolute ----
    for i, (arm, joint) in enumerate([(arm_0, joint_0), (arm_1, joint_1)]):
        ctx.check(
            f"arm_{i} joint is revolute",
            joint.articulation_type == ArticulationType.REVOLUTE,
            details=f"type={joint.articulation_type}",
        )
        # Axis runs along X (perpendicular to channel width).
        ax = tuple(joint.axis)
        ctx.check(
            f"arm_{i} pivot axis runs along X",
            abs(ax[0]) > 0.99 and abs(ax[1]) < 1e-6 and abs(ax[2]) < 1e-6,
            details=f"axis={ax}",
        )

    # ---- Arms are mirrored across the board center (Y = 0) ----
    j0_origin = joint_0.origin
    j1_origin = joint_1.origin
    ctx.check(
        "arms are mirrored across Y = 0",
        abs(j0_origin.xyz[1] + j1_origin.xyz[1]) < 1e-6
        and abs(j0_origin.xyz[0] - j1_origin.xyz[0]) < 1e-6
        and abs(j0_origin.xyz[2] - j1_origin.xyz[2]) < 1e-6,
        details=f"j0_y={j0_origin.xyz[1]:.4f} j1_y={j1_origin.xyz[1]:.4f}",
    )

    # Joint axes are opposite (mirrored): one is +X, the other -X.
    ax0 = tuple(joint_0.axis)
    ax1 = tuple(joint_1.axis)
    ctx.check(
        "arm axes are mirrored (opposite X signs)",
        abs(ax0[0] + ax1[0]) < 1e-6,
        details=f"ax0={ax0} ax1={ax1}",
    )

    # ---- Arm pivot origins sit at the channel rims (not floating) ----
    for i, arm in enumerate([arm_0, arm_1]):
        ctx.expect_contact(
            arm, channel,
            contact_tol=0.004,
            name=f"arm_{i} pivot is anchored on the channel rim",
        )

    # ---- Wire arm geometry reads as bent wire, not a box ----
    for i, arm in enumerate([arm_0, arm_1]):
        amin, amax = ctx.part_world_aabb(arm)
        arm_dx = amax[0] - amin[0]
        arm_dy = amax[1] - amin[1]
        arm_dz = amax[2] - amin[2]
        # Wire arm is tall (extends upward) and thin (wire diameter scale).
        ctx.check(
            f"arm_{i} reads as a tall thin wire handle (not a box)",
            arm_dz > 0.020 and (arm_dx < 0.030 or arm_dy < 0.010),
            details=f"dx={arm_dx:.4f} dy={arm_dy:.4f} dz={arm_dz:.4f}",
        )

    # ---- Closed (rest, q=0) pose: arms extend upright ----
    with ctx.pose({joint_0: 0.0, joint_1: 0.0}):
        a0_min0, a0_max0 = ctx.part_world_aabb(arm_0)
        a1_min0, a1_max0 = ctx.part_world_aabb(arm_1)
        z_top_rest = max(a0_max0[2], a1_max0[2])
        ctx.check(
            "arms extend well above the channel at rest (upright)",
            z_top_rest > PIVOT_Z + ARM_HEIGHT * 0.8,
            details=f"z_top_rest={z_top_rest:.4f} pivot_z={PIVOT_Z:.4f}",
        )

    # ---- Open (folded, q=upper) pose: arms lie flat ----
    upper_0 = joint_0.motion_limits.upper
    upper_1 = joint_1.motion_limits.upper
    with ctx.pose({joint_0: upper_0, joint_1: upper_1}):
        a0_min1, a0_max1 = ctx.part_world_aabb(arm_0)
        a1_min1, a1_max1 = ctx.part_world_aabb(arm_1)
        z_top_fold = max(a0_max1[2], a1_max1[2])
        ctx.check(
            "folding brings arms down near the channel top (stowed)",
            z_top_fold < z_top_rest - 0.015,
            details=f"z_top_rest={z_top_rest:.4f} z_top_fold={z_top_fold:.4f}",
        )
        # Arms should not penetrate below the board.
        z_bot_fold = min(a0_min1[2], a1_min1[2])
        ctx.check(
            "folded arms stay above the board bottom",
            z_bot_fold > -0.001,
            details=f"z_bot_fold={z_bot_fold:.4f}",
        )

    # ---- Gripping zone: channel overlaps the board paper area in XY ----
    ctx.expect_overlap(
        channel, board, axes="xy", min_overlap=0.015,
        name="channel grips the paper area on the board",
    )

    return ctx.report()


object_model = build_object_model()
