from __future__ import annotations

# Hip flask with a crimped metal crown cap.
# Profile-fork of the long-neck beer bottle: the glass body is reshaped into a
# flattened kidney/D cross-section with a convex outer face (+Y) and a gently
# concave inner face (-Y), rising to a short narrow neck and rolled lip.
# The crown cap and prismatic pop-off joint are unchanged.
# Frame: bottle axis along +Z (base at z=0, mouth/lip at the top).

import math

import cadquery as cq

GLASS_RGBA = (0.18, 0.12, 0.07, 0.4)
CAP_RGBA = (0.62, 0.60, 0.58, 1.0)

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- Flask profile heights (z, metres) -------------------------------------
BASE_Z = 0.0
BODY_START_Z = 0.015
BODY_TOP_Z = 0.105
SHOULDER_TOP_Z = 0.130
NECK_TOP_Z = 0.160
LIP_TOP_Z = 0.168

# Body cross-section: wide and flat (D/kidney shaped)
BODY_HW = 0.040          # half-width in X  (80 mm wide)
BODY_R_POS = 0.014       # convex face (+Y) extent
BODY_R_NEG = 0.010       # concave face (-Y) max extent
CONCAVE_SAG = 0.006      # concave-face inward pull at center

# Neck and lip
NECK_R = 0.0105          # slightly narrower neck so the body reads as deepest
LIP_R = 0.0135           # lip unchanged (cap compatibility)
WALL = 0.0025
BORE_R = 0.0085

N_PTS = 48  # polygon points per cross-section


# ---------------------------------------------------------------------------
# Profile helpers
# ---------------------------------------------------------------------------

def _flask_profile_pts(hw, r_pos, r_neg, sag, cb, n=N_PTS):
    """Generate 2D profile points for a D/kidney cross-section.

    Convex face on +Y, concave face on -Y.
    ``cb`` (circle_blend) 0 = full flask shape, 1 = circle of radius *hw*.
    """
    pts = []
    for i in range(n):
        t = 2.0 * math.pi * i / n
        ct, st = math.cos(t), math.sin(t)
        x = hw * ct
        if st >= 0.0:
            # Convex (+Y) face: interpolate between flask and circle
            y = (1.0 - cb) * r_pos * st + cb * hw * st
        else:
            s = -st  # 0 at sides, 1 at center
            # Flask concave face:  -r_neg·s·(2-s) + sag·s²
            # deepest at intermediate s, shallower at center → concave
            y_flask = -r_neg * s * (2.0 - s) + sag * s * s
            # Circle
            y_circle = -hw * s
            y = (1.0 - cb) * y_flask + cb * y_circle
        pts.append((x, y))
    return pts


def _flask_loft(sections, n=N_PTS, ruled=True):
    """Loft through D-shaped flask cross-sections.

    Each section: (z, hw, r_pos, r_neg, sag, circle_blend)
    """
    wp = cq.Workplane("XY")
    prev_z = 0.0
    for i, (z, hw, rp, rn, sag, cb) in enumerate(sections):
        off = z if i == 0 else z - prev_z
        if i > 0 or abs(off) > 1e-12:
            wp = wp.workplane(offset=off)
        pts = _flask_profile_pts(hw, rp, rn, sag, cb, n)
        wp = wp.moveTo(pts[0][0], pts[0][1])
        for x, y in pts[1:]:
            wp = wp.lineTo(x, y)
        wp = wp.close()
        prev_z = z
    return wp.loft(ruled=ruled)


def _bottle_solid():
    """Build the hollow flask body: outer shell minus inner cavity."""
    # ---- outer silhouette ------------------------------------------------
    outer = _flask_loft([
        # (z,       hw,     r_pos,   r_neg,   sag,    cb)
        (0.000,    0.020,  0.008,   0.006,   0.002,  0.0),   # small base
        (0.005,    0.028,  0.010,   0.008,   0.003,  0.0),   # base flare
        (0.010,    0.034,  0.012,   0.009,   0.004,  0.0),   # heel
        (BODY_START_Z, 0.038, 0.013, 0.009,  0.005,  0.0),   # body start
        (0.040,    BODY_HW, BODY_R_POS, BODY_R_NEG, CONCAVE_SAG, 0.0),  # body
        (0.075,    BODY_HW, BODY_R_POS, BODY_R_NEG, CONCAVE_SAG, 0.0),  # body
        (BODY_TOP_Z, 0.038, 0.013,  0.009,   0.004,  0.0),   # body top
        # Shoulder: cb=0, taper in all directions (D-shape preserved)
        (0.110,    0.028,  0.012,   0.008,   0.003,  0.0),
        (0.118,    0.020,  0.011,   0.008,   0.002,  0.0),
        (SHOULDER_TOP_Z, 0.013, 0.010, 0.009, 0.0,  0.0),
        # Transition to circular neck (snap to circle once hw≈neck_r)
        (0.135,    NECK_R, NECK_R,  NECK_R,  0.0,    1.0),
        (NECK_TOP_Z, NECK_R, NECK_R, NECK_R, 0.0,   1.0),
        # Lip (circular)
        (0.162,    0.0118, 0.0118,  0.0118,  0.0,    1.0),
        (0.165,    LIP_R,  LIP_R,   LIP_R,   0.0,    1.0),
        (LIP_TOP_Z, LIP_R, LIP_R,   LIP_R,   0.0,    1.0),
    ])

    # ---- inner cavity (hollow glass shell) --------------------------------
    w = WALL
    inner = _flask_loft([
        (0.010,    0.016,   0.005,  0.004,  0.001,  0.0),   # cavity floor
        (BODY_START_Z + w, BODY_HW - w, BODY_R_POS - w, BODY_R_NEG - w, CONCAVE_SAG * 0.8, 0.0),
        (0.040,    BODY_HW - w, BODY_R_POS - w, BODY_R_NEG - w, CONCAVE_SAG * 0.8, 0.0),
        (0.075,    BODY_HW - w, BODY_R_POS - w, BODY_R_NEG - w, CONCAVE_SAG * 0.8, 0.0),
        (BODY_TOP_Z - 0.004, 0.036 - w, 0.011, 0.007, 0.003, 0.0),
        (0.110,    0.026 - w, 0.010,  0.006,  0.002,  0.0),
        (0.118,    0.018 - w, 0.009,  0.006,  0.001,  0.0),
        (SHOULDER_TOP_Z, 0.011, 0.008, 0.007, 0.0,  0.0),
        (0.135,    NECK_R - w, NECK_R - w, NECK_R - w, 0.0, 1.0),
        (NECK_TOP_Z, BORE_R, BORE_R, BORE_R, 0.0,    1.0),
        (LIP_TOP_Z + 0.004, BORE_R, BORE_R, BORE_R, 0.0, 1.0),  # poke through
    ])

    return outer.cut(inner)


# ---------------------------------------------------------------------------
# Crown cap (unchanged from parent)
# ---------------------------------------------------------------------------

def _cap_mesh():
    cap_top_z = 0.0
    skirt_h = 0.0080
    top_h = 0.0035
    top_r = 0.0152
    skirt_r = 0.0150

    top = (
        cq.Workplane("XY")
        .workplane(offset=cap_top_z)
        .circle(top_r)
        .workplane(offset=top_h)
        .circle(top_r * 0.86)
        .loft(ruled=True)
    )

    skirt = (
        cq.Workplane("XY")
        .workplane(offset=cap_top_z - skirt_h)
        .circle(skirt_r)
        .workplane(offset=skirt_h)
        .circle(top_r)
        .loft(ruled=True)
    )
    skirt_bore = (
        cq.Workplane("XY")
        .workplane(offset=cap_top_z - skirt_h - 0.001)
        .circle(skirt_r - 0.0016)
        .workplane(offset=skirt_h + 0.0005)
        .circle(top_r - 0.0016)
        .loft(ruled=True)
    )
    skirt = skirt.cut(skirt_bore)
    cap = top.union(skirt)

    n_flutes = 21
    flute_z = cap_top_z - skirt_h + 0.0030
    for i in range(n_flutes):
        ang = 2.0 * math.pi * i / n_flutes
        fx = (skirt_r + 0.0004) * math.cos(ang)
        fy = (skirt_r + 0.0004) * math.sin(ang)
        tab = (
            cq.Workplane("XY")
            .workplane(offset=flute_z)
            .center(fx, fy)
            .rect(0.0026, 0.0026)
            .extrude(0.0050)
        )
        cap = cap.union(tab)

    return mesh_from_cadquery(cap, "crown_cap")


# ---------------------------------------------------------------------------
# Object model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hip_flask")

    glass = model.material("dark_glass", rgba=GLASS_RGBA)
    cap_metal = model.material("cap_metal", rgba=CAP_RGBA)

    # ---- flask body (root): hollow dark-glass D-shaped shell ----
    bottle = model.part("bottle")
    bottle.visual(
        mesh_from_cadquery(_bottle_solid(), "bottle_glass"),
        material=glass,
        name="bottle_glass",
    )
    bottle.inertial = Inertial.from_geometry(
        Cylinder(radius=BODY_HW * 0.6, length=LIP_TOP_Z),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, LIP_TOP_Z / 2.0)),
    )

    # ---- crown cap (unchanged) ----
    cap = model.part("crown_cap")
    cap.visual(_cap_mesh(), material=cap_metal, name="crown_cap")
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0152, length=0.012),
        mass=0.004,
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
    )

    # Prismatic pop-off joint (unchanged mechanism, origin at new lip height)
    model.articulation(
        "bottle_to_cap",
        ArticulationType.PRISMATIC,
        parent=bottle,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, LIP_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.2, lower=0.0, upper=0.03),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bottle = object_model.get_part("bottle")
    cap = object_model.get_part("crown_cap")
    cap_joint = object_model.get_articulation("bottle_to_cap")

    # --- Dark glass: translucent body material ---
    ctx.check(
        "bottle glass is translucent (alpha < 1)",
        GLASS_RGBA[3] < 1.0,
        details=f"glass rgba={GLASS_RGBA}",
    )

    # --- Flask profile: body is wider (X) than deep (Y) — D-shaped section ---
    body_ext = _ext(ctx.part_world_aabb(bottle))
    ctx.check(
        "body is wider than deep (flask D-section)",
        body_ext[0] > 1.6 * body_ext[1],
        details=f"bottle extents dx={body_ext[0]:.4f} dy={body_ext[1]:.4f}",
    )

    # --- Flask profile: total height is shorter than a long-neck bottle ---
    ctx.check(
        "flask is compact (shorter than long-neck)",
        body_ext[2] < 0.20,
        details=f"height={body_ext[2]:.4f}",
    )

    # --- Flask profile: body width is substantial (> 60 mm) ---
    ctx.check(
        "body is wide (> 60 mm)",
        body_ext[0] > 0.060,
        details=f"width={body_ext[0]:.4f}",
    )

    # --- Flask profile: body depth is thin (< 35 mm) ---
    ctx.check(
        "body is flat/thin (< 35 mm deep)",
        body_ext[1] < 0.035,
        details=f"depth={body_ext[1]:.4f}",
    )

    # --- Concavity proof: the -Y face center is shallower than the max-depth
    #     point, confirming a genuine concave curve on the inner face. ---
    _rn = BODY_R_NEG
    _sag = CONCAVE_SAG
    _s_peak = _rn / (_rn + _sag)           # where max depth occurs
    _d_max = _rn * _s_peak * (2.0 - _s_peak) - _sag * _s_peak * _s_peak
    _d_center = _rn - _sag                  # depth at the exact center
    ctx.check(
        "concave face: center shallower than intermediate region",
        _d_center < _d_max,
        details=f"d_center={_d_center:.4f} d_max={_d_max:.4f}",
    )

    # --- Cap seated on lip when down ---
    ctx.allow_overlap(
        cap,
        bottle,
        elem_a="crown_cap",
        elem_b="bottle_glass",
        reason="Crown cap skirt is crimped down over the bottle lip when seated.",
    )
    ctx.expect_overlap(
        cap, bottle, axes="z", min_overlap=0.002,
        name="cap seated over the bottle lip",
    )

    # Cap is at the top (mouth)
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "cap mounted at the bottle mouth (top)",
        cap_pos is not None and cap_pos[2] > 0.14,
        details=f"cap origin={cap_pos}",
    )

    # --- Cap pops straight up (prismatic pop-off unchanged) ---
    rest_z = ctx.part_world_position(cap)[2]
    with ctx.pose({cap_joint: 0.03}):
        lifted_z = ctx.part_world_position(cap)[2]
        ctx.expect_gap(cap, bottle, axis="z", max_gap=0.05, min_gap=0.001)
    ctx.check(
        "cap pops straight up off the mouth",
        lifted_z > rest_z + 0.025,
        details=f"rest_z={rest_z}, lifted_z={lifted_z}",
    )

    return ctx.report()


object_model = build_object_model()
