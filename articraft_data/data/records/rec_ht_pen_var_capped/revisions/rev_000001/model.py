from __future__ import annotations

# Realistic articulated capped ballpoint pen (Handtools/Pen fork variant).
#
# Articraft brief:
# - Object: a metallic capped ballpoint pen, ~0.146 m long, barrel diameter
#   ~0.011 m. Modeled upright along +Z: writing tip at the bottom (-Z),
#   barrel crown at the top (+Z). No click mechanism; no push-button.
# - Root/support: the barrel (body + nose cone + permanently exposed tip +
#   closed crown) is the fixed root. The pocket clip is rigidly mounted.
# - Parts:
#     * barrel - root: lathed tapered body, conical nose, writing ball-tip,
#                closed crown top, hinge lug, and repeated grip rings.
#     * clip   - spring-steel pocket clip with oval window (FIXED).
#     * cap    - friction-fit cap covering the nose cone and tip, connected
#                via a revolute snap hinge that swings the cap clear.
# - Articulations:
#     * barrel_to_cap, REVOLUTE, hinge at barrel surface near nose cone top
#       on the +X side, axis = -Y so positive q swings cap away from the tip.
#       Limits 0 to ~2.8 rad (~160°).
#     * barrel_to_clip, FIXED (structural trim).
# - Visible geometry: lathed barrel with closed dome crown, conical nose,
#   ball tip, hollow cap shell with hinge knuckle, flat spring clip with oval
#   window, chrome grip rings, restrained metal/cap-color materials.
# - Support/fit: cap friction-fits over the nose cone when closed; hinge
#   knuckle and lug share the pivot axis.
# - Intentional overlaps: cap shell over nose cone (friction fit, scoped);
#   cap knuckle and barrel lug at the pivot (scoped).
# - Tests: no plunger; cap is REVOLUTE; positive pose swings cap clear;
#   cap covers tip when closed; tip is lowest; clip intact; grip rings exist.

import math
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# --- Key dimensions (meters) ---------------------------------------------------
BARREL_TOP_Z = 0.118
COLLAR_TOP_Z = 0.118
COLLAR_BOT_Z = 0.100
BODY_TOP_Z = 0.100
NOSE_TOP_Z = 0.030
NOSE_TIP_Z = 0.006
TIP_END_Z = 0.000

R_COLLAR = 0.0062
R_BODY_TOP = 0.0058
R_BODY_MID = 0.0056
R_NOSE_TOP = 0.0050
R_TIP_SEAT = 0.0011
R_BALL = 0.0006

# Crown (closed dome top, replaces the plunger bore)
CROWN_HEIGHT = 0.008
CROWN_TOP_Z = BARREL_TOP_Z + CROWN_HEIGHT

# Cap dimensions
CAP_LENGTH = 0.036
CAP_OUTER_R = 0.0066
CAP_INNER_R = 0.0054
CAP_WALL = CAP_OUTER_R - CAP_INNER_R

# Hinge location: barrel surface on +X side, at nose cone top
HINGE_Z = NOSE_TOP_Z
HINGE_X = R_BODY_TOP

# Grip ring layout
N_GRIP_RINGS = 5
RING_START_Z = 0.042
RING_SPACING = 0.005

# Materials
CHROME = (0.83, 0.84, 0.86, 1.0)
ALUM = (0.74, 0.75, 0.77, 1.0)
TIP_METAL = (0.62, 0.63, 0.66, 1.0)
CAP_COLOR = (0.14, 0.18, 0.42, 1.0)  # deep navy blue cap


# ---------------------------------------------------------------------------
# Geometry builders
# ---------------------------------------------------------------------------

def _build_barrel_geometry() -> LatheGeometry:
    """Lathed barrel body + nose cone + writing tip + closed dome crown.

    Same outer profile as parent except the top is a closed dome instead of
    an open bore. Profile is (radius, z), lathe axis +Z.
    """
    profile = [
        (0.0, TIP_END_Z),
        (R_BALL, NOSE_TIP_Z * 0.45),
        (R_TIP_SEAT, NOSE_TIP_Z),
        (R_NOSE_TOP, NOSE_TOP_Z),
        (R_BODY_MID, 0.060),
        (R_BODY_TOP, BODY_TOP_Z),
        (R_BODY_TOP, COLLAR_BOT_Z),
        (R_COLLAR, COLLAR_BOT_Z + 0.001),
        (R_COLLAR, COLLAR_TOP_Z),
        # Closed dome crown
        (R_COLLAR, CROWN_TOP_Z - 0.003),
        (R_COLLAR * 0.82, CROWN_TOP_Z - 0.001),
        (R_COLLAR * 0.45, CROWN_TOP_Z),
        (0.0, CROWN_TOP_Z + 0.001),
    ]
    return LatheGeometry(profile, segments=64, closed=True)


def _build_grip_ring_geometry() -> LatheGeometry:
    """Single raised grip ring band, revolved around Z at local origin."""
    r_inner = R_BODY_MID - 0.0001
    r_outer = R_BODY_MID + 0.0005
    h = 0.0015
    profile = [
        (0.0, 0.0),
        (r_inner, 0.0),
        (r_outer, 0.0003),
        (r_outer, h - 0.0003),
        (r_inner, h),
        (0.0, h),
    ]
    return LatheGeometry(profile, segments=32, closed=True)


def _build_clip_shape() -> cq.Workplane:
    """Spring-steel pocket clip with oval window (identical to parent)."""
    clip_len = 0.052
    clip_w = 0.0072
    clip_t = 0.0009
    top_z = COLLAR_TOP_Z - 0.002

    blade = (
        cq.Workplane("YZ")
        .workplane(offset=R_COLLAR + clip_t / 2.0 + 0.0006)
        .center(0.0, top_z - clip_len / 2.0)
        .rect(clip_w, clip_len)
        .extrude(clip_t)
    )
    win_l = 0.020
    win_w = 0.0034
    window = (
        cq.Workplane("YZ")
        .workplane(offset=R_COLLAR + clip_t / 2.0 + 0.0006)
        .center(0.0, top_z - 0.020)
        .ellipse(win_w / 2.0, win_l / 2.0)
        .extrude(clip_t + 0.002)
    )
    blade = blade.cut(window)

    bridge = (
        cq.Workplane("XZ")
        .workplane(offset=clip_w / 2.0)
        .moveTo(R_COLLAR - 0.0008, top_z + 0.0008)
        .lineTo(R_COLLAR + clip_t / 2.0 + 0.0016, top_z + 0.0008)
        .lineTo(R_COLLAR + clip_t / 2.0 + 0.0016, top_z - 0.002)
        .lineTo(R_COLLAR - 0.0008, top_z - 0.002)
        .close()
        .extrude(-clip_w)
    )
    foot = (
        cq.Workplane("YZ")
        .workplane(offset=R_COLLAR + 0.0006)
        .center(0.0, top_z - clip_len + 0.004)
        .circle(0.0028)
        .extrude(0.0016)
    )
    return blade.union(bridge).union(foot)


def _build_cap_shape() -> cq.Workplane:
    """Friction-fit cap shell with hinge knuckle, in the cap part frame.

    Cap part origin = hinge point. At q=0 the cap hangs straight down
    covering the nose cone; the cap cylinder axis is offset in -X so it
    aligns with the pen Z-axis when closed.
    """
    cx = -HINGE_X  # cap axis offset so it centers on pen axis at q=0
    cap_top_z = -0.0005  # just below hinge
    cap_bot_z = cap_top_z - CAP_LENGTH

    # Outer cylindrical shell
    outer = (
        cq.Workplane("XY")
        .workplane(offset=cap_top_z)
        .center(cx, 0.0)
        .circle(CAP_OUTER_R)
        .extrude(-CAP_LENGTH)
    )

    # Hollow interior (open at the top for the nose cone to enter)
    inner_depth = CAP_LENGTH - CAP_WALL
    inner = (
        cq.Workplane("XY")
        .workplane(offset=cap_top_z)
        .center(cx, 0.0)
        .circle(CAP_INNER_R)
        .extrude(-inner_depth)
    )
    cap_body = outer.cut(inner)

    # Closed bottom end cap
    end_cap = (
        cq.Workplane("XY")
        .workplane(offset=cap_bot_z)
        .center(cx, 0.0)
        .circle(CAP_OUTER_R)
        .extrude(-CAP_WALL)
    )
    cap_body = cap_body.union(end_cap)

    # Hinge knuckle: small cylinder along Y at the origin (hinge pivot)
    knuckle_r = 0.0015
    knuckle_len = 0.008
    knuckle = (
        cq.Workplane("XZ")
        .workplane(offset=-knuckle_len / 2.0)
        .center(0.0, 0.0)
        .circle(knuckle_r)
        .extrude(knuckle_len)
    )
    cap_body = cap_body.union(knuckle)

    # Neck plate bridging the knuckle down to the cap body top edge
    neck = (
        cq.Workplane("XY")
        .workplane(offset=cap_top_z)
        .center(cx * 0.5, 0.0)
        .rect(abs(cx) + 0.004, 0.006)
        .extrude(abs(cap_top_z) + knuckle_r)
    )
    cap_body = cap_body.union(neck)

    return cap_body


def _build_barrel_hinge_lug() -> cq.Workplane:
    """Barrel-side hinge lug at the nose cone top on the +X side.

    A small cylindrical pin along Y with a mounting boss that overlaps
    the barrel body for physical connectivity.
    """
    lug_r = 0.0017
    lug_len = 0.010

    # Pivot pin along Y
    pin = (
        cq.Workplane("XZ")
        .workplane(offset=-lug_len / 2.0)
        .center(HINGE_X, HINGE_Z)
        .circle(lug_r)
        .extrude(lug_len)
    )

    # Mounting boss from barrel surface inward (overlaps barrel body)
    boss_inner = R_NOSE_TOP - 0.002
    boss_outer = HINGE_X + lug_r + 0.001
    boss = (
        cq.Workplane("YZ")
        .workplane(offset=boss_inner)
        .center(0.0, HINGE_Z)
        .rect(0.005, 0.005)
        .extrude(boss_outer - boss_inner)
    )
    return pin.union(boss)


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="capped_ballpoint_pen")
    model.material("pen_alum", rgba=ALUM)
    model.material("pen_chrome", rgba=CHROME)
    model.material("pen_tip", rgba=TIP_METAL)
    model.material("pen_cap", rgba=CAP_COLOR)

    # --- Barrel (root) ---
    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_geometry(_build_barrel_geometry(), "barrel_body"),
        material="pen_alum",
        name="barrel_body",
    )
    barrel.visual(
        mesh_from_cadquery(_build_barrel_hinge_lug(), "hinge_lug"),
        material="pen_chrome",
        name="hinge_lug",
    )

    # Repeated grip rings (inline parent visuals, for-i-in-range pattern)
    grip_ring_geom = _build_grip_ring_geometry()
    for i in range(N_GRIP_RINGS):
        z = RING_START_Z + i * RING_SPACING
        barrel.visual(
            mesh_from_geometry(grip_ring_geom, f"grip_ring_{i}"),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material="pen_chrome",
            name=f"grip_ring_{i}",
        )

    # --- Pocket clip (FIXED structural trim, same as parent) ---
    clip = model.part("clip")
    clip.visual(
        mesh_from_cadquery(_build_clip_shape(), "clip_blade"),
        material="pen_chrome",
        name="clip_blade",
    )

    # --- Cap (friction-fit, revolute snap hinge) ---
    cap = model.part("cap")
    cap.visual(
        mesh_from_cadquery(_build_cap_shape(), "cap_shell"),
        material="pen_cap",
        name="cap_shell",
    )

    # --- Articulations ---

    # Clip is rigidly mounted on the collar (same as parent)
    model.articulation(
        "barrel_to_clip",
        ArticulationType.FIXED,
        parent=barrel,
        child=clip,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # Cap hinge: revolute snap hinge at barrel surface near nose cone top.
    # Axis = -Y so positive q swings the cap bottom toward +X (away from tip).
    model.articulation(
        "barrel_to_cap",
        ArticulationType.REVOLUTE,
        parent=barrel,
        child=cap,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=2.8,
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel = object_model.get_part("barrel")
    clip = object_model.get_part("clip")
    cap = object_model.get_part("cap")
    cap_joint = object_model.get_articulation("barrel_to_cap")
    clip_joint = object_model.get_articulation("barrel_to_clip")

    # --- Structural variant claims ---
    part_names = [p.name for p in object_model.parts]
    ctx.check(
        "no plunger part (capped variant)",
        "plunger" not in part_names,
        details=f"parts={part_names}",
    )

    # --- Joint type / axis claims ---
    ctx.check(
        "cap joint is revolute",
        cap_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"got {cap_joint.articulation_type}",
    )
    ax = tuple(round(a, 6) for a in cap_joint.axis)
    ctx.check(
        "cap hinge axis is perpendicular to pen axis (Z)",
        abs(ax[2]) < 0.1 and (abs(ax[0]) > 0.5 or abs(ax[1]) > 0.5),
        details=f"axis={ax}",
    )
    ctx.check(
        "clip joint is fixed",
        clip_joint.articulation_type == ArticulationType.FIXED,
        details=f"got {clip_joint.articulation_type}",
    )

    # --- Clip geometry (same as parent) ---
    clip_aabb = ctx.part_world_aabb(clip)
    barrel_aabb = ctx.part_world_aabb(barrel)
    if clip_aabb is not None and barrel_aabb is not None:
        clip_len = clip_aabb[1][2] - clip_aabb[0][2]
        ctx.check(
            "clip is a long blade",
            clip_len > 0.035,
            details=f"clip z-length={clip_len}",
        )
        ctx.check(
            "clip stands off on +X side",
            clip_aabb[1][0] > barrel_aabb[1][0] - 0.001,
            details=f"clip max x={clip_aabb[1][0]}, barrel max x={barrel_aabb[1][0]}",
        )
    ctx.expect_contact(
        clip, barrel,
        contact_tol=0.0015,
        name="clip is anchored to barrel collar",
    )

    # --- Writing tip is the lowest geometry ---
    if barrel_aabb is not None:
        ctx.check(
            "writing tip reaches the bottom",
            barrel_aabb[0][2] <= 0.001,
            details=f"barrel min z={barrel_aabb[0][2]}",
        )

    # --- Cap covers tip when closed (q=0) ---
    cap_aabb_closed = ctx.part_world_aabb(cap)
    if cap_aabb_closed is not None:
        ctx.check(
            "cap extends below nose tip when closed",
            cap_aabb_closed[0][2] < NOSE_TIP_Z,
            details=f"cap min z={cap_aabb_closed[0][2]}",
        )
    if cap_aabb_closed is not None and barrel_aabb is not None:
        ctx.expect_overlap(
            cap, barrel,
            axes="z",
            min_overlap=0.020,
            name="cap overlaps barrel zone when closed",
        )

    # --- Cap swings clear of tip when opened ---
    with ctx.pose({cap_joint: 2.0}):
        open_aabb = ctx.part_world_aabb(cap)
        if open_aabb is not None:
            ctx.check(
                "cap swings clear of nose tip when opened",
                open_aabb[0][2] > NOSE_TIP_Z,
                details=f"open cap min z={open_aabb[0][2]}, nose_tip_z={NOSE_TIP_Z}",
            )
    with ctx.pose({cap_joint: cap_joint.motion_limits.upper}):
        full_open_aabb = ctx.part_world_aabb(cap)
        if full_open_aabb is not None and barrel_aabb is not None:
            ctx.check(
                "cap fully open clears the writing tip",
                full_open_aabb[0][2] > NOSE_TIP_Z + 0.005,
                details=f"full open cap min z={full_open_aabb[0][2]}",
            )

    # --- Grip rings present ---
    barrel_visual_names = [v.name for v in barrel.visuals]
    ring_names = [n for n in barrel_visual_names if n.startswith("grip_ring_")]
    ctx.check(
        "grip rings present on barrel",
        len(ring_names) >= 3,
        details=f"found: {ring_names}",
    )

    # --- Crown top exists (no bore) ---
    if barrel_aabb is not None:
        ctx.check(
            "barrel has closed crown top",
            barrel_aabb[1][2] > CROWN_TOP_Z - 0.003,
            details=f"barrel max z={barrel_aabb[1][2]}",
        )

    # --- Intentional overlaps ---
    # Cap friction-fits over the nose cone when closed
    ctx.allow_overlap(
        cap, barrel,
        elem_a="cap_shell",
        elem_b="barrel_body",
        reason="The cap friction-fits over the nose cone when closed; this is the intended friction-fit seating.",
    )
    # Cap knuckle and barrel lug share the hinge pivot
    ctx.allow_overlap(
        cap, barrel,
        elem_a="cap_shell",
        elem_b="hinge_lug",
        reason="The cap knuckle and barrel lug share the hinge pivot axis; small overlap at the pivot is the snap-hinge mount.",
    )

    # --- Proof checks for the allowed overlaps ---
    ctx.expect_contact(
        cap, barrel,
        elem_a="cap_shell",
        elem_b="barrel_body",
        contact_tol=0.003,
        name="cap seats on nose cone when closed",
    )

    return ctx.report()


object_model = build_object_model()
