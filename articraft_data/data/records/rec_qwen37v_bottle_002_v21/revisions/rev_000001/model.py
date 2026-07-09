from __future__ import annotations

# Tall cylindrical bottle with a narrow neck and a flip cap on a revolute hinge.
# Frame: vertical axis along +Z, bottle standing on z=0, centerline on x=y=0.
#   - rounded base -> tall straight cylindrical body -> short shoulder taper
#     -> narrow neck -> open mouth rim -> flip cap hinged at rear.
# Articulation:
#   - cap_hinge: REVOLUTE at rear of neck rim, axis along +X so positive q
#     flips the cap upward and backward (open).

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
BASE_Z = 0.0
BODY_TOP_Z = 0.190       # end of straight cylindrical body
SHOULDER_TOP_Z = 0.220   # end of shoulder taper, start of neck
NECK_TOP_Z = 0.248       # top of neck rim (mouth opening)

BODY_R = 0.032           # body radius (~64mm dia, typical water bottle)
NECK_R = 0.013           # neck outer radius (~26mm dia)
NECK_BORE_R = 0.010      # mouth bore radius (~20mm opening)
WALL = 0.0015            # wall thickness

CAP_R = 0.015            # cap disc radius (slightly larger than neck)
CAP_THICK = 0.005        # cap disc thickness
TAB_LENGTH = 0.008       # small flip tab extending from front of cap
TAB_WIDTH = 0.006
TAB_THICK = 0.003

# Hinge at rear (-Y side) of neck rim
HINGE_Y = -(NECK_R + 0.001)  # just behind the neck outer wall


def _bottle_solid() -> cq.Workplane:
    """Hollow bottle shell: tall cylinder -> shoulder taper -> narrow neck with open mouth."""
    # Outer profile points (z, r) for revolution about Z axis.
    outer_pts = [
        (0.000, 0.018),    # rounded base heel (tucked in)
        (0.006, 0.030),    # base flare out
        (0.012, BODY_R),   # full body radius
        (BODY_TOP_Z, BODY_R),            # straight body up to shoulder
        (BODY_TOP_Z + 0.008, BODY_R - 0.002),  # shoulder start
        (SHOULDER_TOP_Z, NECK_R + 0.002),       # shoulder end
        (SHOULDER_TOP_Z + 0.003, NECK_R),       # neck start
        (NECK_TOP_Z, NECK_R),                   # straight neck to rim
    ]

    # Build outer revolved profile
    wp = cq.Workplane("XZ").moveTo(0.0, outer_pts[0][0])
    for r, z in [(r, z) for (z, r) in outer_pts]:
        wp = wp.lineTo(r, z)
    wp = wp.lineTo(0.0, outer_pts[-1][0]).close()
    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))

    # Inner cavity (hollow) - opens through the neck mouth.
    # Flat bottom at z=0.014 ensures a clean mesh connection through the wall.
    inner_pts = [
        (0.014, 0.008),              # cavity floor center (near axis)
        (0.014, BODY_R - WALL),     # cavity floor at inner body radius
        (BODY_TOP_Z, BODY_R - WALL),
        (BODY_TOP_Z + 0.008, BODY_R - WALL - 0.002),
        (SHOULDER_TOP_Z, NECK_BORE_R + 0.002),
        (SHOULDER_TOP_Z + 0.003, NECK_BORE_R),
        (NECK_TOP_Z + 0.005, NECK_BORE_R),  # open through the rim
    ]
    iwp = cq.Workplane("XZ").moveTo(0.0, inner_pts[0][1])
    for r, z in inner_pts:
        iwp = iwp.lineTo(r, z)
    iwp = iwp.lineTo(0.0, inner_pts[-1][1]).close()
    cavity = iwp.revolve(360.0, (0, 0, 0), (0, 1, 0))
    bottle = outer.cut(cavity)

    return bottle


def _bottle_mesh():
    return mesh_from_cadquery(_bottle_solid(), "bottle_shell")


def _cap_solid() -> cq.Workplane:
    """Flip cap: disc that covers the mouth, with a front flip tab and rear hinge bracket.

    The cap part frame origin is at the hinge pin location (rear of neck rim).
    In local frame: cap disc extends along +Y toward the front, centered at z=0.
    The hinge bracket extends downward (-Z) from the origin alongside the neck.
    """
    # Main cap disc - centered at (0, NECK_R, 0) relative to hinge origin
    cap = (
        cq.Workplane("XY")
        .center(0.0, NECK_R)
        .circle(CAP_R)
        .extrude(CAP_THICK)
    )

    # Hinge bracket: extends downward from the hinge area alongside the neck rim.
    # In local frame, this goes from z=-0.010 to z=CAP_THICK at the rear (near origin).
    bracket_height = 0.010
    bracket = (
        cq.Workplane("XY")
        .workplane(offset=-bracket_height)
        .center(0.0, 0.001)
        .box(0.006, 0.005, bracket_height + CAP_THICK, centered=(True, True, False))
    )
    cap = cap.union(bracket)

    # Front flip tab for easy gripping
    tab = (
        cq.Workplane("XY")
        .center(0.0, NECK_R + CAP_R + TAB_LENGTH / 2.0 - 0.002)
        .box(TAB_WIDTH, TAB_LENGTH, TAB_THICK, centered=(True, True, False))
    )
    cap = cap.union(tab)

    return cap


def _cap_mesh():
    return mesh_from_cadquery(_cap_solid(), "cap_disc")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flip_cap_bottle")

    clear = model.material("clear_pet", rgba=(0.80, 0.88, 0.90, 0.30))
    cap_mat = model.material("cap_solid", rgba=(0.15, 0.55, 0.85, 1.0))  # blue cap
    tab_mat = model.material("tab_accent", rgba=(0.10, 0.40, 0.70, 1.0))

    # ---- bottle body (root): clear hollow PET shell ----
    body = model.part("bottle_body")
    body.visual(_bottle_mesh(), material=clear, name="bottle_shell")
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, NECK_TOP_Z),
        mass=0.028,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    # ---- flip cap: hinged disc that covers the mouth ----
    cap = model.part("flip_cap")
    cap.visual(_cap_mesh(), material=cap_mat, name="cap_disc")
    cap.inertial = Inertial.from_geometry(
        Cylinder(CAP_R, CAP_THICK),
        mass=0.004,
        origin=Origin(xyz=(0.0, NECK_R, CAP_THICK / 2.0)),
    )

    # cap_hinge: REVOLUTE at rear of neck rim.
    # Origin at the hinge pin: (0, -NECK_R, NECK_TOP_Z)
    # Axis along +X: positive rotation lifts the +Y end (front of cap) upward.
    # Cap part frame sits at the hinge; cap disc extends along +Y.
    model.articulation(
        "cap_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cap,
        origin=Origin(xyz=(0.0, HINGE_Y, NECK_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,       # closed: cap covers mouth
            upper=2.2,       # ~126 degrees open (flipped back)
            effort=2.0,
            velocity=4.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle_body")
    cap = object_model.get_part("flip_cap")
    hinge = object_model.get_articulation("cap_hinge")

    # --- bottle is tall and cylindrical (height >> width) ---
    body_aabb = ctx.part_world_aabb(body)
    mn, mx = body_aabb
    dx = mx[0] - mn[0]
    dy = mx[1] - mn[1]
    dz = mx[2] - mn[2]
    ctx.check(
        "bottle is tall (height > 3x width)",
        dz > 3.0 * max(dx, dy),
        details=f"body extents dx={dx:.4f} dy={dy:.4f} dz={dz:.4f}",
    )

    # --- narrow neck: body is much wider than neck ---
    ctx.check(
        "neck is narrow relative to body",
        NECK_R < BODY_R * 0.5,
        details=f"neck_r={NECK_R}, body_r={BODY_R}",
    )

    # --- flip cap is at the top of the bottle ---
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "flip cap mounted at top of bottle",
        cap_pos is not None and cap_pos[2] > NECK_TOP_Z - 0.01,
        details=f"cap origin z={cap_pos}",
    )

    # --- revolute hinge exists and is non-fixed ---
    ctx.check(
        "cap hinge is revolute",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={hinge.articulation_type}",
    )
    ctx.check(
        "hinge has non-trivial range",
        hinge.motion_limits.upper > 0.5,
        details=f"upper={hinge.motion_limits.upper}",
    )

    # --- flip cap opens: at positive pose, cap geometry rises above closed position ---
    def _cap_aabb_center():
        mn, mx = ctx.part_world_aabb(cap)
        return ((mn[0] + mx[0]) / 2, (mn[1] + mx[1]) / 2, (mn[2] + mx[2]) / 2)

    cap_center_closed = _cap_aabb_center()
    with ctx.pose({hinge: 1.5}):
        cap_center_open = _cap_aabb_center()
    ctx.check(
        "flip cap opens upward at positive angle",
        cap_center_open[2] > cap_center_closed[2] + 0.003,
        details=f"closed center z={cap_center_closed[2]:.4f}, open center z={cap_center_open[2]:.4f}",
    )

    # --- at open pose, cap AABB moves away from the mouth position ---
    with ctx.pose({hinge: 2.0}):
        cap_center_wide = _cap_aabb_center()
    z_rise = cap_center_wide[2] - cap_center_closed[2]
    y_shift = abs(cap_center_wide[1] - cap_center_closed[1])
    ctx.check(
        "flip cap swings away from mouth when open",
        z_rise > 0.005 or y_shift > 0.005,
        details=f"closed={cap_center_closed}, wide_open={cap_center_wide}",
    )

    # --- clear plastic body (alpha < 1) ---
    clear_mat = next(m for m in object_model.materials if m.name == "clear_pet")
    a = clear_mat.rgba[3] if clear_mat.rgba is not None else 1.0
    ctx.check(
        "bottle shell is transparent",
        a < 1.0,
        details=f"clear_pet alpha={a}",
    )

    # --- cap seated on neck rim at rest (small contact/overlap is intentional) ---
    ctx.allow_overlap(
        cap,
        body,
        elem_a="cap_disc",
        elem_b="bottle_shell",
        reason="The flip cap disc seats on the neck rim when closed; small contact overlap at the seating surface is intentional.",
    )

    return ctx.report()


object_model = build_object_model()
