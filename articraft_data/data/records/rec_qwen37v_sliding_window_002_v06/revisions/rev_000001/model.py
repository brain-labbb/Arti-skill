from __future__ import annotations

# Two-panel horizontal sliding window variant: slim vinyl frame with bevelled
# outer corners, one fixed sash (left) + one sliding sash (right), an
# independent insect screen on a shallow prismatic track, and two small roller
# blocks at the bottom of the sliding sash.
#
# Coordinate convention:
#   +Z is up. Window stands vertically.
#     - width  -> X
#     - height -> Z   (sill near z=0)
#     - frame depth / glazing thickness / slide-normal -> Y
#   Glass plane is the X-Z plane. q=0 reads SHUT for sash and screen.
#   Positive sash q slides right sash toward -X (opens).
#   Positive screen q slides screen toward +X (opens to the other side).

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
# Absolute dimensions (meters)
# ---------------------------------------------------------------------------

TOTAL_W = 1.52            # overall window width along X
TOTAL_H = 1.72            # overall height along Z (sill at z=0, head at z=TOTAL_H)

FRAME_FACE = 0.055        # slim vinyl frame member face width
FRAME_DEPTH = 0.110       # frame depth along Y (slimmer profile)
FRAME_BEVEL = 0.006       # outer corner chamfer size

MEETING_OVERLAP = 0.035   # the two sash stiles overlap by this much at center

SASH_FACE = 0.048         # sash perimeter rail/stile face width (slim)
SASH_DEPTH = 0.048        # sash depth along Y
GLASS_T = 0.006           # glazing thickness along Y

# Y layout: frame box centered on y=0. Fixed sash in the rear glazing plane;
# sliding sash sits proud toward +Y so it can pass in front of the fixed sash.
FIXED_SASH_Y = -0.020     # rear glazing plane center (Y)
SLIDE_SASH_Y = 0.034      # sliding sash proud toward +Y (front track)

REBATE = 0.004            # glass tucks under the sash lip by this much

# Roller blocks (two, at bottom of sliding sash)
ROLLER_W = 0.018          # roller block width (X)
ROLLER_T = 0.012          # roller block thickness (Y, stands off sash rear)
ROLLER_H = 0.010          # roller block height (Z)

# Insect screen
SCREEN_FRAME_W = 0.025    # screen frame member width
SCREEN_DEPTH = 0.008      # screen frame depth (Y, very shallow)
SCREEN_MESH_T = 0.002     # mesh panel thickness
SCREEN_Y = -0.051         # near interior face, behind fixed sash rear face

# Latch (cam lock) hardware
LATCH_PLATE_W = 0.025     # keeper plate face width (X)
LATCH_PLATE_H = 0.065     # keeper plate height (Z)
LATCH_PLATE_T = 0.008     # keeper plate thickness (Y)
LATCH_LEVER_LEN = 0.040   # lever arm length
LATCH_LEVER_R = 0.005     # lever arm radius

# ---------------------------------------------------------------------------
# Derived layout
# ---------------------------------------------------------------------------

HALF_W = TOTAL_W / 2.0
INNER_X0 = -HALF_W + FRAME_FACE
INNER_X1 = HALF_W - FRAME_FACE
INNER_Z0 = FRAME_FACE
INNER_Z1 = TOTAL_H - FRAME_FACE
INNER_W = INNER_X1 - INNER_X0
INNER_H = INNER_Z1 - INNER_Z0

SASH_OPENING_W = (INNER_W + MEETING_OVERLAP) / 2.0
SASH_OPENING_H = INNER_H

FIXED_OPEN_CX = INNER_X0 + SASH_OPENING_W / 2.0
SLIDE_OPEN_CX = INNER_X1 - SASH_OPENING_W / 2.0
MID_CZ = (INNER_Z0 + INNER_Z1) / 2.0

# Screen opening matches one sash opening width, slightly shorter in height
SCREEN_OPENING_W = SASH_OPENING_W - 0.010
SCREEN_OPENING_H = INNER_H - 0.020
SCREEN_OPEN_CX_CLOSED = INNER_X1 - SCREEN_OPENING_W / 2.0 - 0.020

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

VINYL_RGBA = (0.92, 0.93, 0.94, 1.0)     # white vinyl
GLASS_RGBA = (0.52, 0.60, 0.66, 0.30)    # cool grey-blue, semi-transparent
METAL_RGBA = (0.70, 0.72, 0.75, 1.0)     # brushed metal
SCREEN_MESH_RGBA = (0.20, 0.22, 0.20, 0.45)  # dark grey mesh, semi-transparent
ROLLER_RGBA = (0.25, 0.25, 0.28, 1.0)    # dark nylon/plastic rollers


# ---------------------------------------------------------------------------
# Geometry helpers (CadQuery), authored directly in meters.
# ---------------------------------------------------------------------------

def _slab(x0: float, x1: float, z0: float, z1: float, y_center: float, depth: float) -> cq.Workplane:
    """Axis-aligned box spanning [x0,x1] x [z0,z1] in the X-Z plane, centered on
    y_center with the given Y depth."""
    return (
        cq.Workplane("XY")
        .transformed(offset=((x0 + x1) / 2.0, y_center, (z0 + z1) / 2.0))
        .box(x1 - x0, depth, z1 - z0)
    )


def _build_frame_shape() -> cq.Workplane:
    """Static outer frame: slim slab with bevelled outer corners, cut by the
    sash opening to leave a true hollow perimeter."""
    # Build outer box first, chamfer vertical outer edges, then cut opening.
    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, TOTAL_H / 2.0))
        .box(TOTAL_W, FRAME_DEPTH, TOTAL_H)
        .edges("|Z")
        .chamfer(FRAME_BEVEL)
    )
    cut_depth = FRAME_DEPTH + 0.02
    opening = _slab(INNER_X0, INNER_X1, INNER_Z0, INNER_Z1, 0.0, cut_depth)
    return outer.cut(opening)


def _build_sash_shape() -> cq.Workplane:
    """One sash ring in its OWN local frame, centered on local origin."""
    ow = SASH_OPENING_W
    oh = SASH_OPENING_H
    out_w = ow + 2 * SASH_FACE
    out_h = oh + 2 * SASH_FACE
    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, SASH_DEPTH)
    opening = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SASH_DEPTH + 0.02)
    return outer.cut(opening)


def _build_sash_glass_shape() -> cq.Workplane:
    """Single clear pane filling the sash opening, rebated under the sash lip."""
    ow = SASH_OPENING_W + 2 * REBATE
    oh = SASH_OPENING_H + 2 * REBATE
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, GLASS_T)


def _build_screen_frame_shape() -> cq.Workplane:
    """Insect screen frame ring in its own local frame, centered on origin."""
    ow = SCREEN_OPENING_W
    oh = SCREEN_OPENING_H
    out_w = ow + 2 * SCREEN_FRAME_W
    out_h = oh + 2 * SCREEN_FRAME_W
    outer = _slab(-out_w / 2.0, out_w / 2.0, -out_h / 2.0, out_h / 2.0, 0.0, SCREEN_DEPTH)
    opening = _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SCREEN_DEPTH + 0.01)
    return outer.cut(opening)


def _build_screen_mesh_shape() -> cq.Workplane:
    """Thin mesh panel filling the screen opening, slightly oversized so it
    overlaps with the screen frame ring (captured in the spline groove)."""
    ow = SCREEN_OPENING_W + 0.006  # extends into frame groove
    oh = SCREEN_OPENING_H + 0.006
    return _slab(-ow / 2.0, ow / 2.0, -oh / 2.0, oh / 2.0, 0.0, SCREEN_MESH_T)


# ---------------------------------------------------------------------------
# Part builders
# ---------------------------------------------------------------------------

def _add_sash(model: ArticulatedObject, name: str) -> None:
    """Add a sash part (vinyl ring + clear glass) in its own local frame."""
    sash = model.part(name)
    sash.visual(
        mesh_from_cadquery(_build_sash_shape(), f"{name}_vinyl"),
        material="vinyl",
        name=f"{name}_vinyl",
    )
    sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(), f"{name}_glass"),
        material="glass",
        name=f"{name}_glass",
    )


def _add_rollers(model: ArticulatedObject, sash_name: str) -> None:
    """Add two small roller blocks at the bottom of the sliding sash.
    Positioned at left and right ends of the bottom rail, protruding slightly
    below the sash bottom edge (into the sill track)."""
    sash = model.get_part(sash_name)
    # In sash-local frame, bottom rail is at z = -SASH_OPENING_H/2 - SASH_FACE/2
    bottom_z = -SASH_OPENING_H / 2.0 - SASH_FACE / 2.0
    # Rollers sit at the very bottom, partially below the sash rail
    roller_z = bottom_z - SASH_FACE / 2.0 + ROLLER_H / 2.0 - 0.002
    # Y: rear face of sash (where rollers contact the track)
    roller_y = -SASH_DEPTH / 2.0 - ROLLER_T / 2.0 + 0.003
    # X positions: near left and right ends of the bottom rail
    inset = SASH_FACE + 0.040
    half_ow = SASH_OPENING_W / 2.0
    roller_x_positions = [-(half_ow - inset), (half_ow - inset)]

    for i, rx in enumerate(roller_x_positions):
        sash.visual(
            Box((ROLLER_W, ROLLER_T, ROLLER_H)),
            origin=Origin(xyz=(rx, roller_y, roller_z)),
            material="roller",
            name=f"{sash_name}_roller_{i}",
        )


def _add_latch(model: ArticulatedObject, sash_name: str) -> None:
    """Add the cam-latch hardware on the sliding sash's meeting stile."""
    sash = model.get_part(sash_name)

    stile_x = -SASH_OPENING_W / 2.0 - SASH_FACE / 2.0
    face_y = SASH_DEPTH / 2.0
    plate_y = face_y + LATCH_PLATE_T / 2.0

    sash.visual(
        Box((LATCH_PLATE_W, LATCH_PLATE_T, LATCH_PLATE_H)),
        origin=Origin(xyz=(stile_x, plate_y, 0.0)),
        material="metal",
        name=f"{sash_name}_latch_plate",
    )
    lever_y = face_y + LATCH_PLATE_T + LATCH_LEVER_LEN / 2.0
    sash.visual(
        Cylinder(radius=LATCH_LEVER_R, length=LATCH_LEVER_LEN),
        origin=Origin(xyz=(stile_x, lever_y, -0.008), rpy=(1.5707963, 0.0, 0.0)),
        material="metal",
        name=f"{sash_name}_latch_lever",
    )


def _add_screen(model: ArticulatedObject) -> None:
    """Add the insect screen part (frame ring + mesh panel)."""
    screen = model.part("insect_screen")
    screen.visual(
        mesh_from_cadquery(_build_screen_frame_shape(), "screen_frame"),
        material="vinyl",
        name="screen_frame",
    )
    screen.visual(
        mesh_from_cadquery(_build_screen_mesh_shape(), "screen_mesh"),
        material="screen_mesh",
        name="screen_mesh",
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_window_slim_screen")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("metal", rgba=METAL_RGBA)
    model.material("screen_mesh", rgba=SCREEN_MESH_RGBA)
    model.material("roller", rgba=ROLLER_RGBA)

    # --- Static outer frame (root) with slim bevelled profile ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="vinyl",
        name="frame_shell",
    )

    # --- Fixed (left) + sliding (right) sashes ---
    _add_sash(model, "fixed_sash")
    _add_sash(model, "sliding_sash")
    _add_latch(model, "sliding_sash")
    _add_rollers(model, "sliding_sash")

    # --- Insect screen ---
    _add_screen(model)

    # FIXED left sash seated in the rear glazing plane.
    model.articulation(
        "frame_to_fixed_sash",
        ArticulationType.FIXED,
        parent="frame",
        child="fixed_sash",
        origin=Origin(xyz=(FIXED_OPEN_CX, FIXED_SASH_Y, MID_CZ)),
    )

    # SLIDING right sash: PRISMATIC along X.
    slide_travel = SASH_OPENING_W * 0.88
    model.articulation(
        "frame_to_sliding_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="sliding_sash",
        origin=Origin(xyz=(SLIDE_OPEN_CX, SLIDE_SASH_Y, MID_CZ)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.5, lower=0.0, upper=slide_travel),
    )

    # INSECT SCREEN: PRISMATIC along X, independent shallow track.
    # Screen slides toward +X (positive q opens screen to the opposite side).
    screen_travel = SCREEN_OPENING_W * 0.80
    model.articulation(
        "frame_to_screen",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="insect_screen",
        origin=Origin(xyz=(SCREEN_OPEN_CX_CLOSED, SCREEN_Y, MID_CZ)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.3, lower=0.0, upper=screen_travel),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    fixed_sash = object_model.get_part("fixed_sash")
    sliding_sash = object_model.get_part("sliding_sash")
    insect_screen = object_model.get_part("insect_screen")
    slide = object_model.get_articulation("frame_to_sliding_sash")
    screen_slide = object_model.get_articulation("frame_to_screen")

    # --- Intentional overlaps ---
    # Glass tucks under the vinyl sash lip on each sash (captured glass).
    for nm in ("fixed_sash", "sliding_sash"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass",
            elem_b=f"{nm}_vinyl",
            reason="Clear pane is rebated under the sash lip so it reads captured, not floating.",
        )
    # Each sash ring laps the frame opening edge (seated in track).
    for nm in ("fixed_sash", "sliding_sash"):
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_vinyl",
            reason=f"{nm} ring is rebated into the frame opening / head-sill track (seated capture).",
        )
        ctx.allow_overlap(
            "frame", nm,
            elem_a="frame_shell",
            elem_b=f"{nm}_glass",
            reason=f"{nm} glass is rebated under the frame opening lip (captured glazing).",
        )
    # Latch keeper plate seated onto sliding sash stile face.
    ctx.allow_overlap(
        "sliding_sash", "sliding_sash",
        elem_a="sliding_sash_latch_plate",
        elem_b="sliding_sash_vinyl",
        reason="Latch keeper plate is seated onto the sliding-sash meeting-stile face.",
    )
    # Roller blocks seated against the bottom rail rear face.
    for i in range(2):
        ctx.allow_overlap(
            "sliding_sash", "sliding_sash",
            elem_a=f"sliding_sash_roller_{i}",
            elem_b="sliding_sash_vinyl",
            reason=f"Roller block {i} is seated against the bottom rail for track engagement.",
        )
    # Screen frame and mesh overlap (mesh captured in screen frame).
    ctx.allow_overlap(
        "insect_screen", "insect_screen",
        elem_a="screen_mesh",
        elem_b="screen_frame",
        reason="Screen mesh is captured inside the screen frame ring.",
    )
    # Screen frame laps the frame opening (seated in its own shallow track).
    ctx.allow_overlap(
        "frame", "insect_screen",
        elem_a="frame_shell",
        elem_b="screen_frame",
        reason="Insect screen frame sits in a shallow track in the frame interior.",
    )
    ctx.allow_overlap(
        "frame", "insect_screen",
        elem_a="frame_shell",
        elem_b="screen_mesh",
        reason="Screen mesh passes through the frame opening region in its track.",
    )

    # --- Slim frame verification ---
    with ctx.pose({slide: 0.0, screen_slide: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        frame_w = frame_aabb[1][0] - frame_aabb[0][0]
        frame_d = frame_aabb[1][1] - frame_aabb[0][1]

        # Frame is slim: depth should be less than the original chunky 0.140.
        ctx.check(
            "frame depth is slim profile",
            frame_d < 0.130,
            details=f"frame_depth={frame_d:.3f}",
        )

        # Sill near floor, head at full height.
        ctx.check(
            "sill sits near z=0",
            abs(frame_aabb[0][2]) < 0.02,
            details=f"frame zmin={frame_aabb[0][2]:.4f}",
        )
        ctx.check(
            "head reaches full height",
            abs(frame_aabb[1][2] - TOTAL_H) < 0.02,
            details=f"frame zmax={frame_aabb[1][2]:.4f}",
        )

        f_aabb = ctx.part_world_aabb(fixed_sash)
        s_aabb = ctx.part_world_aabb(sliding_sash)
        scr_aabb = ctx.part_world_aabb(insect_screen)

        # Two sashes side by side: fixed on left, sliding on right.
        fx = (f_aabb[0][0] + f_aabb[1][0]) / 2.0
        sx = (s_aabb[0][0] + s_aabb[1][0]) / 2.0
        ctx.check(
            "fixed sash left of sliding sash",
            fx < sx,
            details=f"fixed_x={fx:.3f}, sliding_x={sx:.3f}",
        )

        # Sliding sash sits proud (+Y) of the fixed sash.
        fy = (f_aabb[0][1] + f_aabb[1][1]) / 2.0
        sy = (s_aabb[0][1] + s_aabb[1][1]) / 2.0
        ctx.check(
            "sliding sash proud of fixed sash",
            sy > fy + 0.01,
            details=f"sliding_y={sy:.3f}, fixed_y={fy:.3f}",
        )

        # Both sashes seated within frame opening.
        ctx.expect_overlap(
            fixed_sash, frame, axes="xz", min_overlap=0.03,
            name="fixed sash seated in frame opening",
        )
        ctx.expect_overlap(
            sliding_sash, frame, axes="xz", min_overlap=0.03,
            name="sliding sash seated in frame opening",
        )

        # --- Roller blocks verification ---
        # Rollers exist at the bottom of the sliding sash.
        for i in range(2):
            roller_aabb = ctx.part_element_world_aabb(sliding_sash, elem=f"sliding_sash_roller_{i}")
            sash_bottom = s_aabb[0][2]
            roller_bottom = roller_aabb[0][2]
            ctx.check(
                f"roller_{i} near bottom of sliding sash",
                roller_bottom < sash_bottom + 0.020,
                details=f"roller_zmin={roller_bottom:.3f}, sash_zmin={sash_bottom:.3f}",
            )

        # The two rollers should be separated in X (at opposite ends).
        r0_aabb = ctx.part_element_world_aabb(sliding_sash, elem="sliding_sash_roller_0")
        r1_aabb = ctx.part_element_world_aabb(sliding_sash, elem="sliding_sash_roller_1")
        r0_cx = (r0_aabb[0][0] + r0_aabb[1][0]) / 2.0
        r1_cx = (r1_aabb[0][0] + r1_aabb[1][0]) / 2.0
        ctx.check(
            "rollers separated in X at opposite ends of sash bottom",
            abs(r1_cx - r0_cx) > 0.30,
            details=f"roller0_x={r0_cx:.3f}, roller1_x={r1_cx:.3f}",
        )

        # --- Insect screen verification ---
        scr_cy = (scr_aabb[0][1] + scr_aabb[1][1]) / 2.0
        # Screen is on the interior side (-Y) of the sashes.
        ctx.check(
            "insect screen on interior side of window",
            scr_cy < fy - 0.005,
            details=f"screen_y={scr_cy:.3f}, fixed_sash_y={fy:.3f}",
        )
        # Screen is shallow (thin in Y).
        scr_depth = scr_aabb[1][1] - scr_aabb[0][1]
        ctx.check(
            "screen is shallow depth",
            scr_depth < 0.040,
            details=f"screen_depth={scr_depth:.3f}",
        )
        # Screen overlaps frame in XZ (seated in opening).
        ctx.expect_overlap(
            insect_screen, frame, axes="xz", min_overlap=0.02,
            name="insect screen seated in frame opening region",
        )

        rest_sx = sx
        rest_sz = (s_aabb[0][2] + s_aabb[1][2]) / 2.0
        rest_scr_cx = (scr_aabb[0][0] + scr_aabb[1][0]) / 2.0

    # --- Sash open pose: sliding sash slides toward fixed sash (-X) ---
    travel = slide.motion_limits.upper
    with ctx.pose({slide: travel, screen_slide: 0.0}):
        s_open = ctx.part_world_aabb(sliding_sash)
        open_sx = (s_open[0][0] + s_open[1][0]) / 2.0
        ctx.check(
            "sliding sash opens toward fixed sash (-X)",
            open_sx < rest_sx - 0.20,
            details=f"rest_x={rest_sx:.3f}, open_x={open_sx:.3f}, travel={travel:.3f}",
        )
        # Pure horizontal slide (no Z change).
        open_sz = (s_open[0][2] + s_open[1][2]) / 2.0
        ctx.check(
            "sash slide is purely horizontal",
            abs(open_sz - rest_sz) < 0.02,
            details=f"open_z={open_sz:.3f}, rest_z={rest_sz:.3f}",
        )
        # Retained insertion: sash stays within frame X span.
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "sash retained within frame X span at full travel",
            s_open[0][0] > f_aabb[0][0] - 1e-4 and s_open[1][0] < f_aabb[1][0] + 1e-4,
            details=f"sash x=[{s_open[0][0]:.3f},{s_open[1][0]:.3f}] frame x=[{f_aabb[0][0]:.3f},{f_aabb[1][0]:.3f}]",
        )
        ctx.expect_overlap(
            sliding_sash, frame, axes="z", min_overlap=0.10,
            name="sash retains vertical engagement with head/sill track",
        )

    # --- Screen open pose: screen slides independently toward +X ---
    screen_travel = screen_slide.motion_limits.upper
    with ctx.pose({slide: 0.0, screen_slide: screen_travel}):
        scr_open = ctx.part_world_aabb(insect_screen)
        open_scr_cx = (scr_open[0][0] + scr_open[1][0]) / 2.0
        ctx.check(
            "insect screen slides independently toward +X",
            open_scr_cx > rest_scr_cx + 0.15,
            details=f"rest_screen_x={rest_scr_cx:.3f}, open_screen_x={open_scr_cx:.3f}",
        )
        # Screen stays within frame height.
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "screen retained within frame height at full travel",
            scr_open[0][2] > f_aabb[0][2] - 0.01 and scr_open[1][2] < f_aabb[1][2] + 0.01,
            details=f"screen z=[{scr_open[0][2]:.3f},{scr_open[1][2]:.3f}]",
        )

    return ctx.report()


object_model = build_object_model()
