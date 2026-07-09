from __future__ import annotations

# Single-hung window with a sage-grey frame: a fixed upper sash and a lower
# sash that slides up, with a meeting-rail sash lock.
#
# Reference (005.png): a modern single-hung window with a warm pale grey-green
# (sage / greige) aluminium-or-vinyl frame. Clear glass, no grille. The TOP
# sash is fixed; the BOTTOM sash slides vertically. A small cam-lock / sash
# latch sits at the meeting rail, centered at the top of the lower sash. In the
# photo the lower sash is raised (partly open); q=0 here is fully shut and the
# driven pose reproduces the raised/open photo state.
#
# Coordinate convention:
#   +Z is up. Window stands vertically: height along +Z, width along X, frame
#   depth / glazing thickness along Y (glass plane is the X-Z plane). The sill
#   sits at z=0; the head is at z=WIN_H.
#
# Articulation (single-hung):
#   - UPPER sash is FIXED to the frame (it does not move).
#   - LOWER sash is PRISMATIC, axis (0,0,1): positive q slides it UP (opens),
#     riding in the side tracks and staying retained at full travel.

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

WIN_W = 0.86          # overall window width (X)
WIN_H = 1.36          # overall window height (Z), sill at z=0
FRAME_FACE = 0.052    # outer frame member face width (X/Z)
FRAME_DEPTH = 0.095   # outer frame jamb depth (Y)

# Clear opening inside the outer frame
OPEN_W = WIN_W - 2 * FRAME_FACE
OPEN_H = WIN_H - 2 * FRAME_FACE
OPEN_X0 = -WIN_W / 2.0 + FRAME_FACE
OPEN_X1 = WIN_W / 2.0 - FRAME_FACE
OPEN_Z0 = FRAME_FACE
OPEN_Z1 = WIN_H - FRAME_FACE

# Sashes. The upper (fixed) sash is a bit taller than the lower in the photo.
SASH_RAIL = 0.046                  # sash perimeter member width
SASH_DEPTH = 0.032                 # sash thickness (Y)
GLASS_T = 0.006                    # glazing thickness (Y)
# Sash stiles tuck a few mm into the jamb rebates/tracks on each side, so each
# sash overlaps the frame (retained insertion) and reads as captured, not
# floating. The sash is therefore slightly WIDER than the clear opening.
SASH_TUCK = 0.007                  # stile tuck into each jamb rebate
SASH_W = OPEN_W + 2 * SASH_TUCK

UPPER_FRAC = 0.52                  # upper sash takes ~52% of the clear height
UPPER_SASH_H = OPEN_H * UPPER_FRAC
LOWER_SASH_H = OPEN_H * 0.50       # lower sash height (overlaps at meeting rail)

# Y planes: lower sash rides interior (-Y), upper sash exterior (+Y) so the
# lower sash can pass behind the meeting rail as it rises.
SASH_Y_GAP = 0.014
LOWER_SASH_Y = -SASH_Y_GAP
UPPER_SASH_Y = +SASH_Y_GAP

# Closed-pose seating (world Z of each sash's bottom rail).
LOWER_BOTTOM_Z = OPEN_Z0 + 0.003                 # lower sash seated on the sill
MEETING_OVERLAP = SASH_RAIL                       # one rail of overlap at meeting rail
UPPER_BOTTOM_Z = LOWER_BOTTOM_Z + LOWER_SASH_H - MEETING_OVERLAP

# Side track channels in the jambs.
TRACK_DEPTH = 0.028

# Cam lock at the meeting rail (top center of the lower sash).
LOCK_BASE = (0.072, 0.022, 0.020)   # (X, Y, Z) mounting base plate
CAM_R = 0.013                        # rotary cam thumb-turn radius
CAM_LEN = 0.020                      # cam body length (along Y, standing off the sash)
LEVER = (0.040, 0.011, 0.010)        # small lever arm

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

FRAME_RGBA = (0.74, 0.74, 0.68, 1.0)    # sage / greige warm pale grey-green
SASH_RGBA = (0.76, 0.76, 0.70, 1.0)     # matching sash, slightly lighter
GLASS_RGBA = (0.62, 0.70, 0.74, 0.26)   # clear glass, faint cool tint, transparent
LOCK_RGBA = (0.80, 0.81, 0.83, 1.0)     # brushed metal cam lock


# ---------------------------------------------------------------------------
# Static outer frame geometry (CadQuery)
# ---------------------------------------------------------------------------

def _build_frame_shape() -> cq.Workplane:
    """Sage-grey outer frame: a perimeter slab with the central opening cut
    out, plus two side-track channels notched into the jamb inner edges where
    the lower sash stiles ride.

    World frame: opening centered on X=0, Z from 0 (sill) to WIN_H (head).
    """
    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, WIN_H / 2.0))
        .box(WIN_W, FRAME_DEPTH, WIN_H)
    )
    opening = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, (OPEN_Z0 + OPEN_Z1) / 2.0))
        .box(OPEN_W, FRAME_DEPTH + 0.02, OPEN_H)
    )
    frame = outer.cut(opening)

    # Track grooves notched into the opening edge of each jamb where the sash
    # stiles tuck in. One groove per sash Y plane on each jamb. The groove is
    # anchored at the opening edge and reaches outward into the jamb just past
    # the stile tuck, so the frame stays one connected solid while the stiles
    # ride inside the grooves (retained insertion).
    groove_x = SASH_TUCK + 0.012
    for sign, edge_x in ((+1.0, OPEN_X0), (-1.0, OPEN_X1)):
        cx = edge_x - sign * groove_x / 2.0
        for track_y in (LOWER_SASH_Y, UPPER_SASH_Y):
            groove = (
                cq.Workplane("XY")
                .transformed(offset=(cx, track_y, (OPEN_Z0 + OPEN_Z1) / 2.0))
                .box(groove_x, TRACK_DEPTH, OPEN_H)
            )
            frame = frame.cut(groove)

    return frame


# ---------------------------------------------------------------------------
# Sash frame geometry (CadQuery): single-pane perimeter ring + glass
# ---------------------------------------------------------------------------

def _build_sash_frame_shape(sash_h: float) -> cq.Workplane:
    """One single-pane sash: a perimeter ring (no muntins) built as a slab with
    the central lite opening cut out. Authored sash-local: X centered, Z from 0
    (bottom rail) to sash_h, Y centered."""
    w = SASH_W
    h = sash_h
    r = SASH_RAIL
    d = SASH_DEPTH
    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, h / 2.0))
        .box(w, d, h)
    )
    lite = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, h / 2.0))
        .box(w - 2 * r, d + 0.02, h - 2 * r)
    )
    return outer.cut(lite)


def _build_sash_glass_shape(sash_h: float) -> cq.Workplane:
    """Single clear glass pane filling the lite, rebated under the sash lip."""
    w = SASH_W
    h = sash_h
    r = SASH_RAIL
    rebate = 0.005
    return (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, h / 2.0))
        .box(w - 2 * r + 2 * rebate, GLASS_T, h - 2 * r + 2 * rebate)
    )


def _add_sash(model: ArticulatedObject, name: str, sash_h: float) -> None:
    sash = model.part(name)
    sash.visual(
        mesh_from_cadquery(_build_sash_frame_shape(sash_h), f"{name}_frame"),
        material="sash",
        name=f"{name}_frame",
    )
    sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(sash_h), f"{name}_glass"),
        material="glass",
        name=f"{name}_glass",
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hung_sage_window")

    model.material("frame", rgba=FRAME_RGBA)
    model.material("sash", rgba=SASH_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("lock", rgba=LOCK_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="frame",
        name="frame_shell",
    )

    # --- Sashes ---
    _add_sash(model, "upper_sash", UPPER_SASH_H)   # fixed
    _add_sash(model, "lower_sash", LOWER_SASH_H)   # slides up

    # --- Cam lock on the lower sash top (meeting) rail, centered ---
    lower = model.get_part("lower_sash")
    lock_z = LOWER_SASH_H - SASH_RAIL / 2.0          # on the top rail
    base_y = -(SASH_DEPTH / 2.0 + LOCK_BASE[1] / 2.0 - 0.003)  # interior face (-Y)
    lower.visual(
        Box(LOCK_BASE),
        origin=Origin(xyz=(0.0, base_y, lock_z)),
        material="lock",
        name="lower_sash_lock_base",
    )
    # Rotary cam thumb-turn standing off the base into -Y.
    cam_y = base_y - (LOCK_BASE[1] / 2.0 + CAM_LEN / 2.0 - 0.002)
    lower.visual(
        Cylinder(radius=CAM_R, length=CAM_LEN),
        origin=Origin(xyz=(0.0, cam_y, lock_z), rpy=(1.5707963, 0.0, 0.0)),
        material="lock",
        name="lower_sash_cam",
    )
    # Small lever arm on the cam.
    lower.visual(
        Box(LEVER),
        origin=Origin(xyz=(LEVER[0] / 2.0 - 0.004, cam_y, lock_z)),
        material="lock",
        name="lower_sash_cam_lever",
    )

    # ----- Articulations (single-hung) -----
    # UPPER sash: FIXED to the frame (does not move). Placed at its seated world
    # position. FIXED joints take no axis or motion limits.
    model.articulation(
        "frame_to_upper_sash",
        ArticulationType.FIXED,
        parent="frame",
        child="upper_sash",
        origin=Origin(xyz=(0.0, UPPER_SASH_Y, UPPER_BOTTOM_Z)),
    )

    # LOWER sash: PRISMATIC, slides UP. axis (0,0,1), positive q opens upward.
    # Origin at the seated (closed) world position so q=0 reads as fully shut.
    model.articulation(
        "frame_to_lower_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="lower_sash",
        origin=Origin(xyz=(0.0, LOWER_SASH_Y, LOWER_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=0.25, lower=0.0, upper=LOWER_SASH_H * 0.45
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    upper = object_model.get_part("upper_sash")
    lower = object_model.get_part("lower_sash")
    j_lower = object_model.get_articulation("frame_to_lower_sash")

    # --- Intentional overlaps ---
    for sash_name in ("upper_sash", "lower_sash"):
        ctx.allow_overlap(
            sash_name, sash_name,
            elem_a=f"{sash_name}_glass",
            elem_b=f"{sash_name}_frame",
            reason="Glass pane is rebated under the sash lip so it reads as captured, not floating.",
        )
    ctx.allow_overlap(
        "frame", "lower_sash",
        reason="Lower sash stiles ride in the jamb side-track grooves (retained insertion).",
    )
    ctx.allow_overlap(
        "frame", "upper_sash",
        reason="Fixed upper sash stiles seat into the jamb rebates/tracks (captured, not floating).",
    )
    ctx.allow_overlap(
        "lower_sash", "upper_sash",
        reason="Sashes overlap by one rail at the meeting rail; they ride in offset Y planes.",
    )
    # Cam lock parts are seated onto / into the lower sash meeting rail.
    ctx.allow_overlap(
        "lower_sash", "lower_sash",
        elem_a="lower_sash_lock_base",
        elem_b="lower_sash_frame",
        reason="Cam-lock base plate is mounted (seated) onto the lower sash meeting rail.",
    )
    ctx.allow_overlap(
        "lower_sash", "lower_sash",
        elem_a="lower_sash_cam_lever",
        elem_b="lower_sash_cam",
        reason="Lever arm is attached to the rotary cam body.",
    )

    # --- Closed pose (q=0): both sashes seated, window reads shut ---
    with ctx.pose({j_lower: 0.0}):
        f_aabb = ctx.part_world_aabb(frame)
        up_aabb = ctx.part_world_aabb(upper)
        lo_aabb = ctx.part_world_aabb(lower)

        frame_w = f_aabb[1][0] - f_aabb[0][0]
        sash_w = lo_aabb[1][0] - lo_aabb[0][0]
        ctx.check(
            "frame spans wider than a sash",
            frame_w > sash_w + 0.04,
            details=f"frame_w={frame_w:.3f}, sash_w={sash_w:.3f}",
        )
        ctx.check(
            "frame sill near z=0",
            abs(f_aabb[0][2]) < 0.01 and f_aabb[1][2] > 1.0,
            details=f"frame z range=({f_aabb[0][2]:.3f}, {f_aabb[1][2]:.3f})",
        )
        ctx.check(
            "sashes within frame width",
            lo_aabb[0][0] > f_aabb[0][0] and lo_aabb[1][0] < f_aabb[1][0]
            and up_aabb[0][0] > f_aabb[0][0] and up_aabb[1][0] < f_aabb[1][0],
            details=f"lower x=({lo_aabb[0][0]:.3f},{lo_aabb[1][0]:.3f})",
        )
        lo_cz = (lo_aabb[0][2] + lo_aabb[1][2]) / 2.0
        up_cz = (up_aabb[0][2] + up_aabb[1][2]) / 2.0
        ctx.check(
            "lower sash below upper sash at closed pose",
            lo_cz < up_cz - 0.25,
            details=f"lower_cz={lo_cz:.3f}, upper_cz={up_cz:.3f}",
        )
        ctx.check(
            "sashes overlap at meeting rail (shut)",
            lo_aabb[1][2] >= up_aabb[0][2] - 1e-4,
            details=f"lower_top={lo_aabb[1][2]:.3f}, upper_bottom={up_aabb[0][2]:.3f}",
        )
        # Lower sash sits low (near the sill) when shut.
        ctx.check(
            "lower sash seated near sill when shut",
            lo_aabb[0][2] < OPEN_Z0 + 0.02,
            details=f"lower_bottom={lo_aabb[0][2]:.3f}, sill_top={OPEN_Z0:.3f}",
        )

        rest_lo_cz = lo_cz
        rest_lo_bot = lo_aabb[0][2]
        rest_up_cz = up_cz

    # --- HERO: lower sash slides UP (opens), matching the raised photo state ---
    travel = LOWER_SASH_H * 0.42
    with ctx.pose({j_lower: travel}):
        op = ctx.part_world_aabb(lower)
        op_cz = (op[0][2] + op[1][2]) / 2.0
        ctx.check(
            "lower sash slides up when opened",
            op_cz > rest_lo_cz + travel * 0.8,
            details=f"rest_cz={rest_lo_cz:.3f}, opened_cz={op_cz:.3f}, travel={travel:.3f}",
        )
        # A clear opening band appears below the raised sash (open daylight gap).
        ctx.check(
            "open pose lifts lower sash bottom above the sill",
            op[0][2] > rest_lo_bot + travel * 0.8,
            details=f"rest_bottom={rest_lo_bot:.3f}, opened_bottom={op[0][2]:.3f}",
        )
        # Stays retained in the side tracks (X footprint still overlaps frame).
        ctx.expect_overlap(
            lower, frame, axes="x", min_overlap=0.05,
            name="lower sash retained in frame when open",
        )

    # --- Upper sash is fixed: it does not move when the lower sash is driven ---
    with ctx.pose({j_lower: travel}):
        up_now = ctx.part_world_aabb(upper)
        up_now_cz = (up_now[0][2] + up_now[1][2]) / 2.0
        ctx.check(
            "upper sash is fixed (does not move)",
            abs(up_now_cz - rest_up_cz) < 1e-4,
            details=f"rest_cz={rest_up_cz:.3f}, posed_cz={up_now_cz:.3f}",
        )

    # --- Cam lock sits at the meeting rail, centered in X ---
    lock_aabb = ctx.part_element_world_aabb(lower, elem="lower_sash_lock_base")
    if lock_aabb is not None:
        lock_cx = (lock_aabb[0][0] + lock_aabb[1][0]) / 2.0
        ctx.check(
            "cam lock centered on the meeting rail",
            abs(lock_cx) < 0.06,
            details=f"lock world X center={lock_cx:.3f}",
        )
        # Lock stands on the interior face of the sash.
        lock_cy = (lock_aabb[0][1] + lock_aabb[1][1]) / 2.0
        ctx.check(
            "cam lock mounted on the interior sash face",
            lock_cy < LOWER_SASH_Y,
            details=f"lock world Y center={lock_cy:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
