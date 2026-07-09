from __future__ import annotations

# Flip-cap lotion/soap bottle with molded volume bands and a revolute flip hinge.
# Frame: vertical axis +Z, bottle standing on z=0, centerline on x=y=0.
#   - rounded base -> cylindrical body with 3 raised bands -> tapered shoulder
#     -> narrow neck -> collar with hinge boss -> flip-top cap.
# Articulation:
#   - flip_hinge: REVOLUTE about +X at rear of collar; positive q opens the cap
#     upward/backward.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- key heights (m) along +Z ----
BODY_TOP_Z = 0.115       # end of straight body, start of shoulder
SHOULDER_TOP_Z = 0.148   # end of taper, base of neck
NECK_TOP_Z = 0.163       # top of neck
COLLAR_H = 0.010
COLLAR_TOP_Z = NECK_TOP_Z + COLLAR_H  # 0.173

# ---- radii (m) ----
BODY_R = 0.030           # body outer radius (60 mm dia)
BAND_R = 0.033           # bands protrude 3 mm beyond body
NECK_R = 0.013           # neck outer radius
NECK_BORE_R = 0.010      # neck bore
COLLAR_R = 0.017         # collar outer radius

# ---- flip cap ----
CAP_R = 0.019            # cap disc radius (slightly larger than collar)
CAP_THICK = 0.005        # cap disc thickness
HINGE_Y = -0.015         # hinge axis Y in body frame (near rear of collar)
CAP_OFFSET_Y = -HINGE_Y  # disc center offset in cap local frame (0.015)


# ---------------------------------------------------------------------------
# Body geometry
# ---------------------------------------------------------------------------

def _body_profile():
    """Outer-wall profile as (z, radius) from base to neck top."""
    return [
        (0.000, 0.016),    # tucked-in heel
        (0.005, 0.026),    # base rounding
        (0.010, BODY_R),   # full body radius
        # ---- Band 1 (z ~0.033 – 0.038) ----
        (0.032, BODY_R),
        (0.033, BAND_R),
        (0.038, BAND_R),
        (0.039, BODY_R),
        # ---- Band 2 (z ~0.061 – 0.066) ----
        (0.060, BODY_R),
        (0.061, BAND_R),
        (0.066, BAND_R),
        (0.067, BODY_R),
        # ---- Band 3 (z ~0.089 – 0.094) ----
        (0.088, BODY_R),
        (0.089, BAND_R),
        (0.094, BAND_R),
        (0.095, BODY_R),
        # ---- body top ----
        (BODY_TOP_Z, BODY_R),
        # ---- shoulder taper ----
        (0.130, 0.025),
        (0.142, 0.018),
        (SHOULDER_TOP_Z, NECK_R + 0.001),
        # ---- neck ----
        (NECK_TOP_Z, NECK_R),
    ]


def _bottle_solid() -> cq.Workplane:
    """Complete body solid: shell + collar + hinge boss, hollowed inside."""
    pts = _body_profile()

    # Outer revolve (profile in XZ plane, revolve around Z)
    wp = cq.Workplane("XZ").moveTo(0.0, pts[0][0])
    for r, z in [(r, z) for (z, r) in pts]:
        wp = wp.lineTo(r, z)
    wp = wp.lineTo(0.0, pts[-1][0]).close()
    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))

    # Collar ring on top of neck
    collar = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, NECK_TOP_Z))
        .circle(COLLAR_R)
        .extrude(COLLAR_H)
    )
    outer = outer.union(collar)

    # Hinge-mount boss at rear of collar (small rectangular lug, flush with collar top)
    boss_h = COLLAR_H
    boss = (
        cq.Workplane("XY")
        .transformed(offset=(0, HINGE_Y, NECK_TOP_Z + boss_h / 2.0))
        .box(0.010, 0.007, boss_h)
    )
    outer = outer.union(boss)

    # Hollow interior (open through collar top)
    wall = 0.0015
    inner_pts = [
        (0.008, 0.006),
        (BODY_R - wall, 0.012),
        (BODY_R - wall, BODY_TOP_Z),
        (0.024, 0.130),
        (0.016, 0.142),
        (NECK_BORE_R, SHOULDER_TOP_Z + 0.002),
        (NECK_BORE_R, COLLAR_TOP_Z + 0.005),   # open through rim
    ]
    iwp = cq.Workplane("XZ").moveTo(0.0, inner_pts[0][1])
    for r, z in inner_pts:
        iwp = iwp.lineTo(r, z)
    iwp = iwp.lineTo(0.0, inner_pts[-1][1]).close()
    cavity = iwp.revolve(360.0, (0, 0, 0), (0, 1, 0))
    return outer.cut(cavity)


# ---------------------------------------------------------------------------
# Flip-cap geometry
# ---------------------------------------------------------------------------

def _flip_cap_solid() -> cq.Workplane:
    """Flip cap: disc + hinge tab + front lip, centred on collar when closed."""
    # Main disc, offset so its centre lands on the collar centre at q=0.
    disc = (
        cq.Workplane("XY")
        .transformed(offset=(0, CAP_OFFSET_Y, 0))
        .circle(CAP_R)
        .extrude(CAP_THICK)
    )

    # Hinge tab at the rear (small tongue reaching toward the boss)
    tab = (
        cq.Workplane("XY")
        .transformed(offset=(0, -0.001, CAP_THICK / 2.0))
        .box(0.008, 0.005, 0.004)
    )
    cap = disc.union(tab)

    # Front lip / flip tab (raised grip at the front edge, sits on top of disc)
    lip_z = (CAP_THICK + 0.003) / 2.0
    lip = (
        cq.Workplane("XY")
        .transformed(offset=(0, CAP_OFFSET_Y + CAP_R - 0.004, lip_z))
        .box(0.014, 0.006, CAP_THICK + 0.003)
    )
    cap = cap.union(lip)

    return cap


# ---------------------------------------------------------------------------
# Model assembly
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flip_cap_bottle")

    # Materials
    body_mat = model.material("body_plastic", rgba=(0.94, 0.92, 0.87, 0.95))
    cap_mat = model.material("cap_plastic", rgba=(0.96, 0.96, 0.96, 1.0))
    hinge_mat = model.material("hinge_accent", rgba=(0.55, 0.55, 0.58, 1.0))

    # ---- Bottle body (root) ----
    body = model.part("bottle_body")
    body.visual(
        mesh_from_cadquery(_bottle_solid(), "body_shell"),
        material=body_mat,
        name="body_shell",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, COLLAR_TOP_Z),
        mass=0.045,
        origin=Origin(xyz=(0.0, 0.0, COLLAR_TOP_Z / 2.0)),
    )

    # ---- Flip cap ----
    cap = model.part("flip_cap")
    cap.visual(
        mesh_from_cadquery(_flip_cap_solid(), "cap_disc"),
        material=cap_mat,
        name="cap_disc",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(CAP_R, CAP_THICK),
        mass=0.005,
        origin=Origin(xyz=(0.0, CAP_OFFSET_Y, CAP_THICK / 2.0)),
    )

    # ---- Flip hinge articulation ----
    # Origin at the rear of the collar top; axis +X so positive q lifts the
    # front of the cap upward (right-hand rule: +Y → +Z).
    model.articulation(
        "flip_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cap,
        origin=Origin(xyz=(0.0, HINGE_Y, COLLAR_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.0,
            lower=0.0,
            upper=2.1,       # ~120° open
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle_body")
    cap = object_model.get_part("flip_cap")
    hinge = object_model.get_articulation("flip_hinge")

    # --- Body proportions: tall bottle, narrower at the top ---
    body_aabb = ctx.part_world_aabb(body)
    mn, mx = body_aabb
    dx = mx[0] - mn[0]
    dz = mx[2] - mn[2]
    ctx.check(
        "bottle is tall (taller than wide)",
        dz > 2.5 * dx,
        details=f"dx={dx:.4f}, dz={dz:.4f}",
    )

    # --- Body is opaque/translucent plastic ---
    body_material = next(m for m in object_model.materials if m.name == "body_plastic")
    ctx.check(
        "body is near-opaque plastic",
        body_material.rgba[3] > 0.85,
        details=f"alpha={body_material.rgba[3]}",
    )

    # --- Cap is mounted on top of the bottle ---
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "flip cap is at the top of the bottle",
        cap_pos is not None and cap_pos[2] > 0.16,
        details=f"cap origin z={cap_pos[2] if cap_pos else None}",
    )

    # --- Allow intentional overlap at the hinge tab/boss interface ---
    ctx.allow_overlap(
        cap,
        body,
        elem_a="cap_disc",
        elem_b="body_shell",
        reason="The cap hinge tab nests against the collar hinge boss at the pivot point.",
    )

    # --- Cap covers the collar when closed ---
    ctx.expect_overlap(
        cap, body,
        axes="xy",
        min_overlap=0.010,
        name="closed cap overlaps collar footprint in XY",
    )
    ctx.expect_gap(
        cap, body,
        axis="z",
        min_gap=-0.003,
        max_gap=0.008,
        name="closed cap seats near the collar top",
    )

    # --- Hinge opens the cap upward ---
    cap_front_z_closed = ctx.part_world_aabb(cap)[1][2]  # max Z
    with ctx.pose({hinge: 1.2}):
        cap_aabb_open = ctx.part_world_aabb(cap)
        cap_max_z_open = cap_aabb_open[1][2]
        cap_center_open = ctx.part_world_position(cap)
    ctx.check(
        "hinge opens cap upward (max Z rises)",
        cap_max_z_open > cap_front_z_closed + 0.005,
        details=f"closed max_z={cap_front_z_closed:.4f}, open max_z={cap_max_z_open:.4f}",
    )

    # --- At full open, the bulk of the cap is well above the collar ---
    with ctx.pose({hinge: 2.1}):
        cap_aabb_full = ctx.part_world_aabb(cap)
        cap_mid_z = (cap_aabb_full[0][2] + cap_aabb_full[1][2]) / 2.0
        cap_max_z_full = cap_aabb_full[1][2]
    ctx.check(
        "fully open cap bulk is above collar",
        cap_mid_z > COLLAR_TOP_Z + 0.003,
        details=f"mid_z={cap_mid_z:.4f}, collar={COLLAR_TOP_Z}",
    )
    ctx.check(
        "fully open cap extends above collar",
        cap_max_z_full > COLLAR_TOP_Z + 0.010,
        details=f"max_z={cap_max_z_full:.4f}, collar={COLLAR_TOP_Z}",
    )

    # --- Revolute joint exists and is non-fixed ---
    ctx.check(
        "flip_hinge is a revolute joint",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={hinge.articulation_type}",
    )
    ctx.check(
        "flip_hinge has non-trivial range",
        hinge.motion_limits.upper - hinge.motion_limits.lower > 1.0,
        details=f"range={hinge.motion_limits.upper - hinge.motion_limits.lower:.2f} rad",
    )

    return ctx.report()


object_model = build_object_model()
