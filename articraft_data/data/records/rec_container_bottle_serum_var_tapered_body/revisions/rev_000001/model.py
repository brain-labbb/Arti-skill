from __future__ import annotations

# Small amber-glass serum bottle with a white dropper cap — TAPERED variant.
# The body is a smooth conical taper: wide at the base, narrowing toward the
# shoulder/neck. Dropper closure, hollow shell, neck, and label band are
# identical in function to the straight-cylinder parent.
#
# Frame: bottle axis along +Z, bottle base at z=0, dropper rising at +Z.
# Construction:
#   - body (root): tapered amber-glass body + shoulder + neck (hollow shell),
#     wrapped by a white paper label band around the middle.
#   - dropper assembly (ONE rigid part): white collar that grips the neck, a
#     rounded white rubber squeeze bulb on top, and a thin clear-glass pipette
#     that runs down through the neck into the bottle.
# Articulation:
#   - dropper assembly: PRISMATIC, pulls straight UP out of the neck, lifting
#     the pipette clear of the bottle mouth.

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

# ---- bottle body dimensions (tapered) ----
BODY_R_BASE = 0.017       # body outer radius at base (~0.034 m diameter)
BODY_R_TOP = 0.011        # body outer radius at top (~0.022 m diameter)
BODY_Z0 = 0.0             # base
BODY_TOP = 0.060          # top of the tapered cylinder wall
SHOULDER_TOP = 0.070      # top of the rounded shoulder
NECK_R = 0.0080           # neck outer radius
NECK_TOP = 0.0820         # top of the neck (bottle mouth)
WALL = 0.0018             # glass wall thickness
BORE_R = NECK_R - WALL    # inner bore radius at the neck

# ---- label band ----
LABEL_Z0 = 0.012
LABEL_Z1 = 0.044

# ---- dropper assembly dimensions ----
COLLAR_R = 0.0105         # white collar grips over the neck
COLLAR_Z0 = 0.066         # collar bottom (overlaps the shoulder/neck slightly)
COLLAR_TOP = 0.090        # collar top
BULB_R = 0.0095           # squeeze bulb radius
BULB_CZ = 0.1075          # bulb center height
PIPETTE_R = 0.0024        # thin glass pipette radius
PIPETTE_BOTTOM = 0.018    # pipette tip height at rest (deep in the bottle)

DROPPER_TRAVEL = 0.068    # prismatic pull-up distance


def _body_radius(z: float) -> float:
    """Outer body radius at height z (linear taper from base to top)."""
    t = max(0.0, min(1.0, (z - BODY_Z0) / (BODY_TOP - BODY_Z0)))
    return BODY_R_BASE + (BODY_R_TOP - BODY_R_BASE) * t


def _body_glass_mesh():
    # Hollow amber-glass shell: tapered body, rounded shoulder, short neck,
    # with an internal bore cut so the bottle reads as a real open container.
    outer = (
        cq.Workplane("XY")
        .workplane(offset=BODY_Z0)
        .circle(BODY_R_BASE)
        .workplane(offset=BODY_TOP - BODY_Z0)
        .circle(BODY_R_TOP)
        .loft()
    )
    # rounded shoulder lofted from body top radius to neck radius
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP)
        .circle(BODY_R_TOP)
        .workplane(offset=(SHOULDER_TOP - BODY_TOP))
        .circle(NECK_R + 0.0015)
        .loft(ruled=False)
    )
    neck = (
        cq.Workplane("XY")
        .workplane(offset=SHOULDER_TOP)
        .circle(NECK_R)
        .extrude(NECK_TOP - SHOULDER_TOP)
    )
    solid = outer.union(shoulder).union(neck)
    # interior cavity: follows the taper, staying a wall-thickness inside
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=WALL)
        .circle(_body_radius(WALL) - WALL)          # z = WALL
        .workplane(offset=(BODY_TOP - WALL))
        .circle(BODY_R_TOP - WALL)                   # z = BODY_TOP
        .workplane(offset=(SHOULDER_TOP - BODY_TOP))
        .circle(BORE_R)                              # z = SHOULDER_TOP
        .workplane(offset=(NECK_TOP - SHOULDER_TOP + 0.001))
        .circle(BORE_R)                              # through the mouth
        .loft(ruled=False)
    )
    solid = solid.cut(cavity)
    return mesh_from_cadquery(solid, "body_glass")


def _label_mesh():
    # White label band that follows the body taper.
    r_outer_lo = _body_radius(LABEL_Z0) + 0.0006
    r_outer_hi = _body_radius(LABEL_Z1) + 0.0006
    r_inner_lo = _body_radius(LABEL_Z0) - 0.0002
    r_inner_hi = _body_radius(LABEL_Z1) - 0.0002
    outer = (
        cq.Workplane("XY")
        .workplane(offset=LABEL_Z0)
        .circle(r_outer_lo)
        .workplane(offset=LABEL_Z1 - LABEL_Z0)
        .circle(r_outer_hi)
        .loft()
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=LABEL_Z0 - 0.001)
        .circle(r_inner_lo)
        .workplane(offset=LABEL_Z1 - LABEL_Z0 + 0.002)
        .circle(r_inner_hi)
        .loft()
    )
    label = outer.cut(inner)
    return mesh_from_cadquery(label, "label_band")


def _collar_mesh():
    # White collar: an outer ring that grips the neck, with a small bore that
    # the pipette passes through, plus a thin top cap the bulb sits on.
    outer = (
        cq.Workplane("XY")
        .workplane(offset=COLLAR_Z0)
        .circle(COLLAR_R)
        .extrude(COLLAR_TOP - COLLAR_Z0)
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=COLLAR_Z0 - 0.001)
        .circle(NECK_R + 0.0006)
        .extrude((COLLAR_TOP - 0.002) - (COLLAR_Z0 - 0.001))
    )
    collar = outer.cut(inner)
    return mesh_from_cadquery(collar, "collar")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="serum_bottle_tapered")

    amber = model.material("amber_glass", rgba=(0.45, 0.24, 0.06, 0.5))
    white = model.material("white_plastic", rgba=(0.95, 0.95, 0.94, 1.0))
    label_white = model.material("label_white", rgba=(0.97, 0.97, 0.96, 1.0))
    clear = model.material("clear_glass", rgba=(0.85, 0.90, 0.92, 0.35))

    # ---- body (root) ----
    body = model.part("body")
    body.visual(_body_glass_mesh(), material=amber, name="body_glass")
    body.visual(_label_mesh(), material=label_white, name="label_band")

    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R_BASE, BODY_TOP),
        mass=0.040,
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP / 2.0)),
    )

    # ---- dropper assembly (collar + bulb + pipette as ONE rigid part) ----
    dropper = model.part("dropper")

    dropper.visual(_collar_mesh(), material=white, name="collar")

    dropper.visual(
        Sphere(BULB_R),
        origin=Origin(xyz=(0.0, 0.0, BULB_CZ)),
        material=white,
        name="bulb",
    )
    dropper.visual(
        Cylinder(0.0045, (BULB_CZ - BULB_R) - COLLAR_TOP + 0.004),
        origin=Origin(xyz=(0.0, 0.0, (COLLAR_TOP + (BULB_CZ - BULB_R)) / 2.0)),
        material=white,
        name="bulb_stem",
    )

    pip_len = COLLAR_TOP - PIPETTE_BOTTOM
    dropper.visual(
        Cylinder(PIPETTE_R, pip_len),
        origin=Origin(xyz=(0.0, 0.0, PIPETTE_BOTTOM + pip_len / 2.0)),
        material=clear,
        name="pipette",
    )

    dropper.inertial = Inertial.from_geometry(
        Cylinder(COLLAR_R, COLLAR_TOP - PIPETTE_BOTTOM),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, (PIPETTE_BOTTOM + BULB_CZ) / 2.0)),
    )

    model.articulation(
        "body_to_dropper",
        ArticulationType.PRISMATIC,
        parent=body,
        child=dropper,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=DROPPER_TRAVEL,
            effort=5.0,
            velocity=0.1,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    dropper = object_model.get_part("dropper")
    pull = object_model.get_articulation("body_to_dropper")

    # --- bottle body is tapered: wider at the base than at the shoulder ---
    body_aabb = ctx.part_world_aabb(body)
    bmn, bmx = body_aabb
    body_h = bmx[2] - bmn[2]
    body_dia = max(bmx[0] - bmn[0], bmx[1] - bmn[1])
    ctx.check(
        "bottle body is short — height under ~0.09 m",
        body_h < 0.090,
        details=f"body height={body_h:.4f}",
    )
    ctx.check(
        "bottle body base is wide (~0.034 m dia)",
        0.025 < body_dia < 0.045,
        details=f"body dia={body_dia:.4f}",
    )

    # Verify taper: body glass XY span is dominated by the wide base
    glass_aabb = ctx.part_element_world_aabb(body, elem="body_glass")
    g_mn, g_mx = glass_aabb
    xy_span = max(g_mx[0] - g_mn[0], g_mx[1] - g_mn[1])
    ctx.check(
        "body glass XY span matches tapered base radius",
        xy_span > 2.0 * BODY_R_BASE - 0.002,
        details=f"xy_span={xy_span:.4f}, expected ~{2*BODY_R_BASE:.4f}",
    )
    ctx.check(
        "body base is clearly wider than neck (visible taper)",
        body_dia > 2.0 * NECK_R + 0.010,
        details=f"body_dia={body_dia:.4f}, neck_dia={2*NECK_R:.4f}",
    )

    body_glass = body.get_visual("body_glass")
    mat = body_glass.material
    rgba = getattr(mat, "rgba", None)
    ctx.check(
        "body glass is amber-tinted and translucent",
        rgba is not None and rgba[0] > rgba[2] and rgba[3] < 0.95,
        details=f"amber_glass rgba={rgba}",
    )

    # --- white label band wraps the tapered body ---
    label_aabb = ctx.part_element_world_aabb(body, elem="label_band")
    lmn, lmx = label_aabb
    ctx.check(
        "label band is on the body wall, around the middle",
        lmn[2] > 0.005 and lmx[2] < SHOULDER_TOP,
        details=f"label z=[{lmn[2]:.4f},{lmx[2]:.4f}]",
    )
    label_xy_span = max(lmx[0] - lmn[0], lmx[1] - lmn[1])
    ctx.check(
        "label band follows body taper (XY span > neck diameter)",
        label_xy_span > 2.0 * NECK_R + 0.004,
        details=f"label xy_span={label_xy_span:.4f}",
    )

    # --- dropper seated in the neck at rest ---
    pip_rest = ctx.part_element_world_aabb(dropper, elem="pipette")
    ctx.check(
        "pipette tip is seated deep inside the bottle at rest",
        pip_rest[0][2] < SHOULDER_TOP - 0.010,
        details=f"pipette bottom z(rest)={pip_rest[0][2]:.4f}",
    )
    ctx.allow_overlap(
        dropper,
        body,
        elem_a="collar",
        elem_b="body_glass",
        reason="The white collar intentionally slips over the bottle neck (seated grip).",
    )
    ctx.allow_overlap(
        dropper,
        body,
        elem_a="pipette",
        elem_b="body_glass",
        reason="The thin glass pipette is intentionally inserted down through the neck into the bottle.",
    )
    ctx.expect_overlap(
        dropper,
        body,
        axes="z",
        elem_a="collar",
        elem_b="body_glass",
        min_overlap=0.002,
        name="collar seated over the neck at rest",
    )

    # --- dropper pulls straight UP ---
    rest_pos = ctx.part_world_position(dropper)
    with ctx.pose({pull: DROPPER_TRAVEL}):
        up_pos = ctx.part_world_position(dropper)
        pip_up = ctx.part_element_world_aabb(dropper, elem="pipette")
    ctx.check(
        "dropper translates straight up when pulled",
        up_pos[2] > rest_pos[2] + 0.05
        and abs(up_pos[0] - rest_pos[0]) < 1e-4
        and abs(up_pos[1] - rest_pos[1]) < 1e-4,
        details=f"rest={rest_pos}, up={up_pos}",
    )
    ctx.check(
        "pipette tip clears the bottle mouth at full extension",
        pip_up[0][2] >= NECK_TOP - 0.001,
        details=f"pipette bottom z(extended)={pip_up[0][2]:.4f}, mouth={NECK_TOP:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
