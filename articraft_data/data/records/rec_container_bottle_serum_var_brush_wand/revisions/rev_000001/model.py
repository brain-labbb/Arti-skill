from __future__ import annotations

# Small amber-glass serum bottle with a gloss-style brush/wand applicator cap.
# Frame: bottle axis along +Z, bottle base at z=0, wand rising at +Z.
# Construction:
#   - body (root): short amber-glass cylinder body + shoulder + neck (hollow
#     shell), wrapped by a white paper label band around the middle.
#   - wand applicator (ONE rigid part): white twist-off cap shell with a gloss
#     dome top that grips the neck, a long thin stem hanging straight down
#     through the neck bore into the bottle, and a doe-foot applicator tip at
#     the stem bottom.
# Articulation:
#   - wand: PRISMATIC, pulls straight UP out of the neck as one rigid piece,
#     lifting the applicator tip clear of the bottle mouth.

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

# ---- bottle body dimensions (IDENTICAL to parent) ----
BODY_R = 0.015          # body outer radius (~0.03 m diameter)
BODY_Z0 = 0.0           # base
BODY_TOP = 0.060        # top of the straight cylinder wall
SHOULDER_TOP = 0.070    # top of the rounded shoulder
NECK_R = 0.0080         # neck outer radius
NECK_TOP = 0.0820       # top of the neck (bottle mouth)
WALL = 0.0018           # glass wall thickness
BORE_R = NECK_R - WALL  # inner bore radius at the neck

# ---- label band (IDENTICAL to parent) ----
LABEL_Z0 = 0.012
LABEL_Z1 = 0.044
LABEL_R = BODY_R + 0.0006

# ---- wand applicator cap dimensions ----
CAP_R = 0.0108          # cap outer radius (~21.6 mm dia)
CAP_Z0 = 0.066          # cap bottom (grips over neck/shoulder junction)
CAP_TOP = 0.096         # cap top (~30 mm tall cap)
CAP_CEIL = 0.003        # cap ceiling (top disk) thickness

STEM_R = 0.0012         # thin wand stem radius (~2.4 mm dia)
APPLICATOR_R = 0.0035   # doe-foot max radius (~7 mm at widest)
APPLICATOR_H = 0.010    # doe-foot height/length (~10 mm)
STEM_TIP_Z = 0.016      # applicator bottom at rest (deep in bottle)

WAND_TRAVEL = 0.072     # prismatic pull-up distance


def _body_glass_mesh():
    """Hollow amber-glass shell: straight body, rounded shoulder, short neck,
    with an internal bore — IDENTICAL to the parent dropper bottle."""
    outer = (
        cq.Workplane("XY")
        .circle(BODY_R)
        .extrude(BODY_TOP)
    )
    # rounded shoulder lofted from body radius to neck radius
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP)
        .circle(BODY_R)
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
    # interior cavity: a single tapered bore that stays a wall-thickness inside
    # the outer profile at every height, so the shell never breaks apart.
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=WALL)
        .circle(BODY_R - WALL)              # z = WALL, body interior
        .workplane(offset=(BODY_TOP - WALL))
        .circle(BODY_R - WALL)              # z = BODY_TOP, body interior
        .workplane(offset=(SHOULDER_TOP - BODY_TOP))
        .circle(BORE_R)                     # z = SHOULDER_TOP, shrinking into neck
        .workplane(offset=(NECK_TOP - SHOULDER_TOP + 0.001))
        .circle(BORE_R)                     # through the mouth (open top)
        .loft(ruled=False)
    )
    solid = solid.cut(cavity)
    return mesh_from_cadquery(solid, "body_glass")


def _cap_shell_mesh():
    """White twist-off cap shell: hollow cylinder with a gloss-dome top,
    open at the bottom to slip over the neck."""
    # Outer cylinder
    outer = (
        cq.Workplane("XY")
        .workplane(offset=CAP_Z0)
        .circle(CAP_R)
        .extrude(CAP_TOP - CAP_Z0)
    )
    # Bore out the inside (open at bottom, stops at ceiling thickness)
    bore_r = NECK_R + 0.0006  # slip fit over the neck
    bore_start = CAP_Z0 - 0.001
    bore_end = CAP_TOP - CAP_CEIL
    inner = (
        cq.Workplane("XY")
        .workplane(offset=bore_start)
        .circle(bore_r)
        .extrude(bore_end - bore_start)
    )
    shell = outer.cut(inner)
    # Gloss-style dome on top: loft from full radius to a smaller crown
    dome = (
        cq.Workplane("XY")
        .workplane(offset=CAP_TOP)
        .circle(CAP_R)
        .workplane(offset=0.004)
        .circle(CAP_R * 0.35)
        .loft(ruled=False)
    )
    shell = shell.union(dome)
    # Small grip ring at the cap bottom edge for twist-off feel
    grip_ring = (
        cq.Workplane("XY")
        .workplane(offset=CAP_Z0)
        .circle(CAP_R + 0.0008)
        .circle(CAP_R - 0.0002)
        .extrude(0.003)
    )
    shell = shell.union(grip_ring)
    return mesh_from_cadquery(shell, "cap_shell")


def _applicator_tip_mesh():
    """Doe-foot applicator tip: smooth lathe-profile shape, narrow at the
    bottom, widest below center, tapering back to the stem radius at top."""
    base_z = STEM_TIP_Z
    # Loft sections from bottom point to stem junction
    solid = (
        cq.Workplane("XY")
        .workplane(offset=base_z)
        .circle(0.0005)                       # rounded bottom tip
        .workplane(offset=APPLICATOR_H * 0.30)
        .circle(APPLICATOR_R * 0.80)          # swelling toward max width
        .workplane(offset=APPLICATOR_H * 0.20)
        .circle(APPLICATOR_R)                 # widest point (doe-foot bulge)
        .workplane(offset=APPLICATOR_H * 0.25)
        .circle(APPLICATOR_R * 0.60)          # narrowing above center
        .workplane(offset=APPLICATOR_H * 0.25)
        .circle(STEM_R + 0.0002)              # transition to stem
        .loft(ruled=False)
    )
    return mesh_from_cadquery(solid, "applicator_tip")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="serum_bottle_wand")

    amber = model.material("amber_glass", rgba=(0.45, 0.24, 0.06, 0.5))
    white = model.material("white_plastic", rgba=(0.95, 0.95, 0.94, 1.0))
    label_white = model.material("label_white", rgba=(0.97, 0.97, 0.96, 1.0))
    flock = model.material("flocked_tip", rgba=(0.90, 0.88, 0.85, 1.0))

    # ---- body (root) — IDENTICAL to parent ----
    body = model.part("body")
    body.visual(_body_glass_mesh(), material=amber, name="body_glass")

    # white paper label band wrapping the middle of the body (thin shell ring)
    label = (
        cq.Workplane("XY")
        .workplane(offset=LABEL_Z0)
        .circle(LABEL_R)
        .circle(BODY_R - 0.0002)
        .extrude(LABEL_Z1 - LABEL_Z0)
    )
    body.visual(mesh_from_cadquery(label, "label_band"), material=label_white, name="label_band")

    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, BODY_TOP),
        mass=0.040,
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP / 2.0)),
    )

    # ---- wand applicator assembly (cap + stem + doe-foot, ONE rigid part) ----
    wand = model.part("wand")

    # white twist-off cap shell with gloss dome
    wand.visual(_cap_shell_mesh(), material=white, name="cap_shell")

    # thin stem rod from inside cap ceiling down to applicator junction
    stem_top_z = CAP_TOP - CAP_CEIL       # 0.093
    stem_bottom_z = STEM_TIP_Z + APPLICATOR_H  # 0.026
    stem_len = stem_top_z - stem_bottom_z       # 0.067
    stem_cz = (stem_top_z + stem_bottom_z) / 2.0
    wand.visual(
        Cylinder(STEM_R, stem_len),
        origin=Origin(xyz=(0.0, 0.0, stem_cz)),
        material=white,
        name="stem",
    )

    # doe-foot applicator tip at the bottom of the stem
    wand.visual(
        _applicator_tip_mesh(),
        material=flock,
        name="applicator_tip",
    )

    wand.inertial = Inertial.from_geometry(
        Cylinder(CAP_R, CAP_TOP - STEM_TIP_Z),
        mass=0.010,
        origin=Origin(xyz=(0.0, 0.0, (STEM_TIP_Z + CAP_TOP) / 2.0)),
    )

    model.articulation(
        "body_to_wand",
        ArticulationType.PRISMATIC,
        parent=body,
        child=wand,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=WAND_TRAVEL,
            effort=5.0,
            velocity=0.1,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    wand = object_model.get_part("wand")
    pull = object_model.get_articulation("body_to_wand")

    # --- bottle body is short and amber (same checks as parent) ---
    body_aabb = ctx.part_world_aabb(body)
    bmn, bmx = body_aabb
    body_h = bmx[2] - bmn[2]
    body_dia = max(bmx[0] - bmn[0], bmx[1] - bmn[1])
    ctx.check(
        "bottle body is short (squat) — height under ~0.09 m",
        body_h < 0.090,
        details=f"body height={body_h:.4f}",
    )
    ctx.check(
        "bottle body is narrow (~0.03 m dia)",
        0.020 < body_dia < 0.040,
        details=f"body dia={body_dia:.4f}",
    )
    body_glass = body.get_visual("body_glass")
    mat = body_glass.material
    rgba = getattr(mat, "rgba", None)
    ctx.check(
        "body glass is amber-tinted and translucent",
        rgba is not None and rgba[0] > rgba[2] and rgba[3] < 0.95,
        details=f"amber_glass rgba={rgba}",
    )

    # --- white label band wraps the body ---
    label_aabb = ctx.part_element_world_aabb(body, elem="label_band")
    lmn, lmx = label_aabb
    ctx.check(
        "label band is on the body wall, around the middle",
        lmn[2] > 0.005 and lmx[2] < SHOULDER_TOP,
        details=f"label z=[{lmn[2]:.4f},{lmx[2]:.4f}]",
    )

    # --- wand: cap grips the neck, applicator inserted into the bottle ---
    tip_rest = ctx.part_element_world_aabb(wand, elem="applicator_tip")
    ctx.check(
        "applicator tip is seated deep inside the bottle at rest",
        tip_rest[0][2] < SHOULDER_TOP - 0.010,
        details=f"applicator bottom z(rest)={tip_rest[0][2]:.4f}",
    )

    # Cap shell overlaps the neck (seated twist grip)
    ctx.allow_overlap(
        wand,
        body,
        elem_a="cap_shell",
        elem_b="body_glass",
        reason="The twist-off cap shell intentionally slips over the bottle neck for a seated grip.",
    )
    # Stem passes through the neck bore
    ctx.allow_overlap(
        wand,
        body,
        elem_a="stem",
        elem_b="body_glass",
        reason="The thin wand stem passes through the neck bore into the bottle interior.",
    )
    # Applicator tip inside the bottle body
    ctx.allow_overlap(
        wand,
        body,
        elem_a="applicator_tip",
        elem_b="body_glass",
        reason="The doe-foot applicator tip is intentionally inserted down through the neck into the bottle.",
    )

    ctx.expect_overlap(
        wand,
        body,
        axes="z",
        elem_a="cap_shell",
        elem_b="body_glass",
        min_overlap=0.002,
        name="cap shell seated over the neck at rest",
    )

    # --- wand pulls straight UP and applicator clears the mouth ---
    rest_pos = ctx.part_world_position(wand)
    with ctx.pose({pull: WAND_TRAVEL}):
        up_pos = ctx.part_world_position(wand)
        tip_up = ctx.part_element_world_aabb(wand, elem="applicator_tip")
    ctx.check(
        "wand translates straight up when pulled (no lateral shift)",
        up_pos[2] > rest_pos[2] + 0.05
        and abs(up_pos[0] - rest_pos[0]) < 1e-4
        and abs(up_pos[1] - rest_pos[1]) < 1e-4,
        details=f"rest={rest_pos}, up={up_pos}",
    )
    ctx.check(
        "applicator tip clears the bottle mouth at full extension",
        tip_up[0][2] >= NECK_TOP - 0.001,
        details=f"applicator bottom z(extended)={tip_up[0][2]:.4f}, mouth={NECK_TOP:.4f}",
    )

    # --- verify the doe-foot applicator is wider than the stem ---
    tip_aabb = ctx.part_element_world_aabb(wand, elem="applicator_tip")
    t_mn, t_mx = tip_aabb
    tip_dx = t_mx[0] - t_mn[0]
    tip_dz = t_mx[2] - t_mn[2]
    ctx.check(
        "applicator tip is wider than the stem (doe-foot profile)",
        tip_dx > STEM_R * 2.5,
        details=f"tip width={tip_dx:.4f}, stem dia={STEM_R*2:.4f}",
    )
    ctx.check(
        "applicator tip has reasonable doe-foot height (~8-12 mm)",
        0.006 < tip_dz < 0.015,
        details=f"tip height={tip_dz:.4f}",
    )

    # --- cap shell reads as a tall twist-off cap (not a thin collar) ---
    cap_aabb = ctx.part_element_world_aabb(wand, elem="cap_shell")
    c_mn, c_mx = cap_aabb
    cap_h = c_mx[2] - c_mn[2]
    ctx.check(
        "cap shell is tall enough to read as a twist-off cap (~25-38 mm)",
        0.020 < cap_h < 0.042,
        details=f"cap height={cap_h:.4f}",
    )

    # --- stem is visibly thinner than the neck bore ---
    stem_aabb = ctx.part_element_world_aabb(wand, elem="stem")
    s_mn, s_mx = stem_aabb
    stem_dx = s_mx[0] - s_mn[0]
    ctx.check(
        "stem is thin enough to pass through the neck bore",
        stem_dx < BORE_R * 2.0,
        details=f"stem width={stem_dx:.4f}, bore dia={BORE_R*2:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
