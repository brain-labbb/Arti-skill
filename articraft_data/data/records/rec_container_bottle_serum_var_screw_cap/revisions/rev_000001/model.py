from __future__ import annotations

# Small amber-glass serum bottle with a white screw-on twist cap.
# Frame: bottle axis along +Z, bottle base at z=0, cap on top.
# Construction:
#   - body (root): short amber-glass cylinder body + shoulder + neck (hollow
#     shell), wrapped by a white paper label band around the middle.
#   - cap: white cylindrical twist cap with ribbed grip that screws onto the
#     bottle neck and rotates about the bottle axis.
# Articulation:
#   - body_to_cap: REVOLUTE, rotates about +Z at the neck top, representing
#     the screw-on twist motion.

import math

import cadquery as cq
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

# ---- twist cap dimensions ----
CAP_DIA = 0.022         # cap outer diameter
CAP_R = CAP_DIA / 2     # cap outer radius
SKIRT_DROP = 0.010      # skirt extends 10mm below neck top (covers upper neck)
CAP_ABOVE = 0.006       # cap extends 6mm above neck top
CAP_TOTAL_H = SKIRT_DROP + CAP_ABOVE  # 0.016 total height
CAP_BORE_R = NECK_R - 0.0003  # interference fit (cap compresses onto neck)

# ---- grip ribs (repeated via for-loop) ----
N_RIBS = 24
RIB_BOTTOM = -SKIRT_DROP + 0.002   # ribs start 2mm above skirt bottom
RIB_TOP = CAP_ABOVE - 0.002        # ribs end 2mm below cap top
RIB_H = RIB_TOP - RIB_BOTTOM       # rib height
RIB_W = 0.0010          # rib tangential width
RIB_D = 0.0005          # rib radial protrusion


def _body_glass_mesh():
    # IDENTICAL to parent: hollow amber-glass shell with body, shoulder, neck,
    # and internal bore cut so the bottle reads as a real open container.
    outer = (
        cq.Workplane("XY")
        .circle(BODY_R)
        .extrude(BODY_TOP)
    )
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
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=WALL)
        .circle(BODY_R - WALL)
        .workplane(offset=(BODY_TOP - WALL))
        .circle(BODY_R - WALL)
        .workplane(offset=(SHOULDER_TOP - BODY_TOP))
        .circle(BORE_R)
        .workplane(offset=(NECK_TOP - SHOULDER_TOP + 0.001))
        .circle(BORE_R)
        .loft(ruled=False)
    )
    solid = solid.cut(cavity)
    return mesh_from_cadquery(solid, "body_glass")


def _cap_shell_mesh():
    """Hollow cylindrical cap shell with a skirt that drops over the neck.

    Built from z=0 (mesh bottom) to z=CAP_TOTAL_H (mesh top).
    The visual origin offset shifts it so z=0 in mesh = z=-SKIRT_DROP in cap frame.
    """
    # Outer cylinder: full cap height
    outer = (
        cq.Workplane("XY")
        .circle(CAP_R)
        .extrude(CAP_TOTAL_H)
    )
    # Bore: from bottom up to SKIRT_DROP level (the neck top plane in mesh coords)
    # Above SKIRT_DROP the cap is solid (the closed top).
    bore = (
        cq.Workplane("XY")
        .workplane(offset=-0.001)
        .circle(CAP_BORE_R)
        .extrude(SKIRT_DROP + 0.001)
    )
    cap = outer.cut(bore)
    return mesh_from_cadquery(cap, "cap_shell")


def _cap_rib_mesh():
    """Single thin vertical grip rib for the cap side."""
    # X = radial (rib_d), Y = tangential (rib_w), Z = height (rib_h)
    rib = (
        cq.Workplane("XY")
        .rect(RIB_D, RIB_W)
        .extrude(RIB_H)
    )
    return mesh_from_cadquery(rib, "cap_rib")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="serum_bottle")

    amber = model.material("amber_glass", rgba=(0.45, 0.24, 0.06, 0.5))
    white = model.material("white_plastic", rgba=(0.95, 0.95, 0.94, 1.0))
    label_white = model.material("label_white", rgba=(0.97, 0.97, 0.96, 1.0))

    # ---- body (root) — IDENTICAL to parent ----
    body = model.part("body")
    body.visual(_body_glass_mesh(), material=amber, name="body_glass")

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

    # ---- twist cap ----
    cap = model.part("cap")

    # Main cap shell — visual origin shifts mesh so bottom is at -SKIRT_DROP in cap frame
    cap.visual(
        _cap_shell_mesh(),
        origin=Origin(xyz=(0.0, 0.0, -SKIRT_DROP)),
        material=white,
        name="cap_shell",
    )

    # Grip ribs placed around the cap circumference via for-loop
    rib_mesh = _cap_rib_mesh()
    for i in range(N_RIBS):
        angle = 2.0 * math.pi * i / N_RIBS
        x = CAP_R * math.cos(angle)
        y = CAP_R * math.sin(angle)
        cap.visual(
            rib_mesh,
            origin=Origin(xyz=(x, y, RIB_BOTTOM), rpy=(0.0, 0.0, angle)),
            material=white,
            name=f"rib_{i}",
        )

    cap.inertial = Inertial.from_geometry(
        Cylinder(CAP_R, CAP_TOTAL_H),
        mass=0.005,
        origin=Origin(xyz=(0.0, 0.0, (CAP_ABOVE - SKIRT_DROP) / 2.0)),
    )

    # Revolute joint: cap rotates about the bottle axis at the neck top.
    model.articulation(
        "body_to_cap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=2.0 * math.pi,
            effort=2.0,
            velocity=1.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    cap = object_model.get_part("cap")
    twist = object_model.get_articulation("body_to_cap")

    # --- bottle body is short and amber-glass, same as parent ---
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

    # --- twist cap geometry ---
    cap_aabb = ctx.part_world_aabb(cap)
    cmn, cmx = cap_aabb
    ctx.check(
        "cap skirt drops below neck top (wraps the neck)",
        cmn[2] < NECK_TOP - 0.005,
        details=f"cap base z={cmn[2]:.4f}, neck_top={NECK_TOP:.4f}",
    )
    ctx.check(
        "cap extends above neck top",
        cmx[2] > NECK_TOP + 0.003,
        details=f"cap top z={cmx[2]:.4f}, neck_top={NECK_TOP:.4f}",
    )
    ctx.check(
        "cap diameter is wider than the neck",
        (cmx[0] - cmn[0]) > NECK_R * 2,
        details=f"cap width={cmx[0]-cmn[0]:.4f}, neck dia={NECK_R*2:.4f}",
    )

    # --- cap bore interference fit with neck (intentional overlap) ---
    ctx.allow_overlap(
        cap,
        body,
        elem_a="cap_shell",
        elem_b="body_glass",
        reason="The cap bore has a slight interference fit over the bottle neck (seated screw-cap grip).",
    )
    ctx.expect_overlap(
        cap,
        body,
        axes="z",
        elem_a="cap_shell",
        elem_b="body_glass",
        min_overlap=0.005,
        name="cap skirt overlaps the neck along Z (retained on the neck)",
    )

    # --- twist cap rotates about the bottle axis (not prismatic pull) ---
    rest_pos = ctx.part_world_position(cap)
    with ctx.pose({twist: math.pi}):
        half_pos = ctx.part_world_position(cap)

    ctx.check(
        "cap rotates in place (stays at same height, no lateral shift)",
        abs(half_pos[2] - rest_pos[2]) < 1e-4
        and abs(half_pos[0] - rest_pos[0]) < 1e-4
        and abs(half_pos[1] - rest_pos[1]) < 1e-4,
        details=f"rest={rest_pos}, half_turn={half_pos}",
    )

    # --- joint is revolute, not prismatic ---
    ctx.check(
        "cap articulation is revolute (twist), not prismatic (pull)",
        twist.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={twist.articulation_type}",
    )

    # --- grip ribs are present around the cap ---
    rib_count = sum(1 for v in cap.visuals if v.name.startswith("rib_"))
    ctx.check(
        f"cap has {N_RIBS} grip ribs for tactile twist grip",
        rib_count == N_RIBS,
        details=f"found {rib_count} ribs",
    )

    return ctx.report()


object_model = build_object_model()
