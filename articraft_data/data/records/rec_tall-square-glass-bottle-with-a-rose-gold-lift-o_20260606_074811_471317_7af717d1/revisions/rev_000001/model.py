from __future__ import annotations

# Tall square glass bottle with a rose-gold lift-off cap.
# Frame: vertical axis +Z, bottle centered on the world Z axis, base on z=0.
#   - body: a clear, tall, square-section (rounded-corner) hollow glass bottle.
#           The body runs from z=0 up to a slightly inset top rim near z=0.150.
#   - cap : a tall rose-gold rectangular block that slips down OVER the top of
#           the bottle. It is hollow underneath so it sleeves over the bottle
#           top. It is a friction / lift-off cap (PRISMATIC, +Z), NO thread.
#
# Articulation:
#   - body_to_cap: PRISMATIC along +Z. At q=0 the cap is seated down over the
#     bottle top; positive q lifts the cap straight up off the bottle (~0.05 m).

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Inertial,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ----- key dimensions (meters) -----
SECT = 0.060  # outer square section width of the glass body
CORNER_R = 0.006  # rounded corner radius
GLASS_WALL = 0.004  # glass wall thickness

BODY_BOTTOM_Z = 0.0
BODY_TOP_Z = 0.150  # top rim of the glass body
BODY_H = BODY_TOP_Z - BODY_BOTTOM_Z

# The cap sleeves down over the upper portion of the bottle.
CAP_OUTER = 0.064  # outer square of the rose-gold cap (slightly proud of glass)
CAP_CORNER_R = 0.006
CAP_WALL = 0.005  # rose-gold wall thickness
CAP_H = 0.050  # total cap height
CAP_OVERLAP = 0.034  # how far the cap skirt slides down over the bottle top

# At rest, the cap top is above the bottle top; its skirt embraces the upper body.
CAP_BOTTOM_Z = BODY_TOP_Z - CAP_OVERLAP  # z where the open underside of the cap is
CAP_TOP_Z = CAP_BOTTOM_Z + CAP_H


def _body_solid() -> cq.Workplane:
    # Hollow rounded-square glass bottle: outer shell minus an inner cavity.
    # The cavity is open at the top (poke the cutter up past the rim) and leaves
    # a solid glass base.
    outer = (
        cq.Workplane("XY")
        .rect(SECT, SECT)
        .extrude(BODY_H)
        .edges("|Z")
        .fillet(CORNER_R)
    )
    inner_w = SECT - 2.0 * GLASS_WALL
    inner = (
        cq.Workplane("XY")
        .workplane(offset=GLASS_WALL)  # solid glass floor of thickness GLASS_WALL
        .rect(inner_w, inner_w)
        .extrude(BODY_H)  # over-extrude so the cavity opens through the top rim
        .edges("|Z")
        .fillet(max(CORNER_R - GLASS_WALL, 0.001))
    )
    return outer.cut(inner)


def _cap_solid() -> cq.Workplane:
    # Tall rose-gold rectangular cap, hollow underside so it slips over the
    # bottle top. Built in cap-local frame with the open underside at z=0 and
    # the closed top at z=CAP_H.
    outer = (
        cq.Workplane("XY")
        .rect(CAP_OUTER, CAP_OUTER)
        .extrude(CAP_H)
        .edges("|Z")
        .fillet(CAP_CORNER_R)
    )
    bore_w = CAP_OUTER - 2.0 * CAP_WALL
    # Hollow bore open at the bottom, capped by the top plate (CAP_WALL thick).
    bore_depth = CAP_H - CAP_WALL
    inner = (
        cq.Workplane("XY")
        .rect(bore_w, bore_w)
        .extrude(bore_depth)
        .edges("|Z")
        .fillet(max(CAP_CORNER_R - CAP_WALL, 0.001))
    )
    return outer.cut(inner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="square_glass_bottle")

    glass = model.material("clear_glass", rgba=(0.82, 0.86, 0.88, 0.25))
    rose_gold = model.material("rose_gold", rgba=(0.86, 0.58, 0.50, 1.0))

    # ---- body (root): clear tall square glass bottle ----
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_solid(), "glass_body"),
        material=glass,
        name="glass_body",
    )
    body.inertial = Inertial.from_geometry(
        Box((SECT, SECT, BODY_H)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
    )

    # ---- cap: rose-gold rectangular lift-off cap (slips over the top) ----
    cap = model.part("cap")
    cap.visual(
        mesh_from_cadquery(_cap_solid(), "rose_gold_cap"),
        material=rose_gold,
        name="rose_gold_cap",
    )
    cap.inertial = Inertial.from_geometry(
        Box((CAP_OUTER, CAP_OUTER, CAP_H)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, CAP_H / 2.0)),
    )

    # Cap lifts straight up off the bottle top. At q=0 the cap is seated down,
    # its open underside at CAP_BOTTOM_Z, skirt sleeved over the upper body.
    model.articulation(
        "body_to_cap",
        ArticulationType.PRISMATIC,
        parent=body,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, CAP_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.2, lower=0.0, upper=0.05),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    cap = object_model.get_part("cap")
    lift = object_model.get_articulation("body_to_cap")

    # The cap skirt is intentionally slipped down over the bottle top (friction
    # lift-off fit), so a thin local overlap of the cap wall and the glass top
    # is expected and intended.
    ctx.allow_overlap(
        cap,
        body,
        elem_a="rose_gold_cap",
        elem_b="glass_body",
        reason="Rose-gold cap skirt is intentionally seated down over the bottle top (lift-off friction fit).",
    )

    # ---- bottle is tall and square-section ----
    body_ext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "bottle body is square in section",
        abs(body_ext[0] - body_ext[1]) < 0.004,
        details=f"body extents={body_ext}",
    )
    ctx.check(
        "bottle body is tall (height >> section)",
        body_ext[2] > 2.0 * max(body_ext[0], body_ext[1]),
        details=f"body extents={body_ext}",
    )

    # ---- cap seated over the top when down (q=0): skirt overlaps the upper body ----
    ctx.expect_overlap(
        cap,
        body,
        axes="xy",
        min_overlap=0.030,
        name="cap footprint sits over the bottle top",
    )
    ctx.expect_overlap(
        cap,
        body,
        axes="z",
        min_overlap=0.010,
        name="cap skirt slides down over the bottle top",
    )
    # The cap is at the top of the bottle, not the bottom.
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "cap sits at the top of the bottle",
        cap_pos is not None and cap_pos[2] > BODY_TOP_Z - CAP_OVERLAP - 1e-6,
        details=f"cap origin={cap_pos}",
    )

    # ---- cap lifts STRAIGHT UP off the bottle top (prismatic, no rotation) ----
    rest_z = ctx.part_world_position(cap)[2]
    rest_xy = ctx.part_world_position(cap)[:2]
    with ctx.pose({lift: 0.05}):
        up_pos = ctx.part_world_position(cap)
        # Once fully lifted, the cap clears the bottle top entirely.
        ctx.expect_gap(
            cap,
            body,
            axis="z",
            min_gap=0.0,
            name="lifted cap clears the bottle top",
        )
    ctx.check(
        "cap lifts straight up (Z increases)",
        up_pos is not None and up_pos[2] > rest_z + 0.045,
        details=f"rest_z={rest_z}, lifted_z={up_pos[2] if up_pos else None}",
    )
    ctx.check(
        "cap does not translate sideways while lifting",
        up_pos is not None
        and abs(up_pos[0] - rest_xy[0]) < 1e-6
        and abs(up_pos[1] - rest_xy[1]) < 1e-6,
        details=f"rest_xy={rest_xy}, lifted_xy={up_pos[:2] if up_pos else None}",
    )

    # ---- materials: cap is rose-gold, body is clear glass (distinct) ----
    cap_mat = cap.get_visual("rose_gold_cap").material
    body_mat = body.get_visual("glass_body").material
    ctx.check(
        "cap material is rose-gold and distinct from glass",
        cap_mat is not None
        and body_mat is not None
        and getattr(cap_mat, "name", None) == "rose_gold"
        and getattr(body_mat, "name", None) == "clear_glass",
        details=f"cap_mat={getattr(cap_mat, 'name', None)}, body_mat={getattr(body_mat, 'name', None)}",
    )

    return ctx.report()


object_model = build_object_model()
