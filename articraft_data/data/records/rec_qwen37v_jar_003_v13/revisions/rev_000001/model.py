from __future__ import annotations

# Tall cylindrical storage jar with a clamp bail lid.
# Frame: vertical axis +Z, jar centered on the world Z axis, base on z=0.
#
# Parts:
#   - jar_body (root): tall hollow glass cylinder with a wider rim at the top
#     and two pivot ear lugs on opposite sides of the neck.
#   - lid: flat glass disk that sits on top of the gasket/rim.
#   - bail: U-shaped wire clamp that pivots on the body ears. When closed
#     (q=0), the cross-bar presses down on the lid top. When open (positive q),
#     the bail swings away from the lid toward -Y.
#
# The gasket ring is a visual on jar_body, seated on the rim.
#
# Articulations:
#   - body_to_lid: FIXED — lid is clamped onto the rim by the bail.
#   - body_to_bail: REVOLUTE around X axis. At q=0 bail is closed (cross-bar
#     over lid); positive q swings bail open (cross-bar moves toward -Y and down).

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

# ----- key dimensions (meters) -----
# Jar body
BODY_OUTER_R = 0.042       # outer radius of jar body
BODY_INNER_R = 0.038       # inner radius (4mm glass wall)
BODY_H = 0.220             # total jar body height
GLASS_FLOOR = 0.005        # solid glass floor thickness

# Rim (wider lip at mouth)
RIM_OUTER_R = 0.047        # wider rim outer radius
RIM_INNER_R = BODY_INNER_R # same bore as body
RIM_H = 0.012              # rim height
RIM_BOTTOM_Z = BODY_H - RIM_H  # rim starts here

# Pivot ears (small lugs on the jar neck for bail pivot)
EAR_WIDTH = 0.010          # ear width (along Y)
EAR_THICKNESS = 0.005      # ear thickness (along X, protrusion from body)
EAR_HEIGHT = 0.014         # ear height (along Z)
EAR_BOTTOM_Z = 0.175       # ear bottom z position
EAR_OUTER_X = BODY_OUTER_R + EAR_THICKNESS  # outer X of ear
EAR_CENTER_Z = EAR_BOTTOM_Z + EAR_HEIGHT / 2.0

# Bail pivot height (center of pivot pin)
PIVOT_Z = EAR_CENTER_Z     # 0.182

# Gasket (rubber ring on rim)
GASKET_OUTER_R = 0.046
GASKET_INNER_R = 0.039
GASKET_H = 0.003
GASKET_BOTTOM_Z = BODY_H   # sits on top of rim

# Lid (glass disk on gasket)
LID_R = 0.045
LID_H = 0.006
LID_BOTTOM_Z = GASKET_BOTTOM_Z + GASKET_H  # sits on top of gasket
LID_CENTER_Z = LID_BOTTOM_Z + LID_H / 2.0
LID_TOP_Z = LID_BOTTOM_Z + LID_H

# Bail wire
WIRE_R = 0.0018            # wire radius (3.6mm diameter wire)
BAIL_ARM_X = EAR_OUTER_X   # bail arms are at the ear X positions
BAIL_ARM_H = LID_TOP_Z - PIVOT_Z  # arm height from pivot to lid top


def _jar_body_solid() -> cq.Workplane:
    """Hollow glass cylinder with wider rim and two pivot ear lugs."""
    # Outer shell
    outer = (
        cq.Workplane("XY")
        .circle(BODY_OUTER_R)
        .extrude(BODY_H)
    )
    # Wider rim on top portion
    rim = (
        cq.Workplane("XY")
        .workplane(offset=RIM_BOTTOM_Z)
        .circle(RIM_OUTER_R)
        .extrude(RIM_H)
    )
    body = outer.union(rim)

    # Hollow interior (open at top, solid floor at bottom)
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=GLASS_FLOOR)
        .circle(BODY_INNER_R)
        .extrude(BODY_H + RIM_H)  # over-extrude to open through top
    )
    body = body.cut(cavity)

    # Pivot ear lugs (two rectangular protrusions on opposite sides)
    for sign in (1, -1):
        ear_center_x = sign * (BODY_OUTER_R + EAR_THICKNESS / 2.0)
        ear = (
            cq.Workplane("XY")
            .workplane(offset=EAR_BOTTOM_Z)
            .center(ear_center_x, 0.0)
            .rect(EAR_THICKNESS, EAR_WIDTH)
            .extrude(EAR_HEIGHT)
        )
        body = body.union(ear)

    return body


def _gasket_solid() -> cq.Workplane:
    """Rubber gasket ring that sits on the jar rim."""
    outer = (
        cq.Workplane("XY")
        .circle(GASKET_OUTER_R)
        .extrude(GASKET_H)
    )
    inner = (
        cq.Workplane("XY")
        .circle(GASKET_INNER_R)
        .extrude(GASKET_H)
    )
    return outer.cut(inner)


def _lid_solid() -> cq.Workplane:
    """Flat glass lid disk with a small center knob."""
    # Main disk
    disk = (
        cq.Workplane("XY")
        .circle(LID_R)
        .extrude(LID_H)
    )
    # Small center knob/handle on top
    knob = (
        cq.Workplane("XY")
        .workplane(offset=LID_H)
        .circle(0.008)
        .extrude(0.005)
    )
    return disk.union(knob)


def _bail_solid() -> cq.Workplane:
    """U-shaped wire bail clamp. Built in bail-local frame with pivot at origin.

    At q=0, bail frame coincides with articulation frame at the pivot center.
    The arms extend upward (+Z) from the pivot, and the cross-bar connects
    them at the top.
    """
    arm_h = BAIL_ARM_H
    arm_x = BAIL_ARM_X

    # Left arm (cylinder along Z at x = -arm_x, from z=0 to z=arm_h+WIRE_R)
    # Slight over-extrude so the arm top overlaps with the cross-bar
    left_arm = (
        cq.Workplane("XY")
        .center(-arm_x, 0.0)
        .circle(WIRE_R)
        .extrude(arm_h + WIRE_R)
    )
    # Right arm
    right_arm = (
        cq.Workplane("XY")
        .center(arm_x, 0.0)
        .circle(WIRE_R)
        .extrude(arm_h + WIRE_R)
    )

    # Cross-bar: cylinder along X connecting arm tops at z = arm_h
    # Use YZ workplane (normal along +X), start at x = -arm_x
    cross_bar = (
        cq.Workplane("YZ")
        .workplane(offset=-arm_x)
        .center(0.0, arm_h)
        .circle(WIRE_R)
        .extrude(2.0 * arm_x)
    )

    bail = left_arm.union(right_arm).union(cross_bar)
    return bail


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clamp_jar")

    glass = model.material("clear_glass", rgba=(0.80, 0.85, 0.88, 0.30))
    rubber = model.material("rubber_gasket", rgba=(0.85, 0.35, 0.15, 1.0))
    steel = model.material("stainless_steel", rgba=(0.72, 0.73, 0.74, 1.0))

    # ---- jar_body (root): hollow glass cylinder with rim and ears ----
    jar_body = model.part("jar_body")
    jar_body.visual(
        mesh_from_cadquery(_jar_body_solid(), "glass_jar"),
        material=glass,
        name="glass_jar",
    )
    # Gasket ring visual on the body (seated on the rim)
    jar_body.visual(
        mesh_from_cadquery(_gasket_solid(), "gasket_ring"),
        material=rubber,
        origin=Origin(xyz=(0.0, 0.0, GASKET_BOTTOM_Z)),
        name="gasket_ring",
    )
    jar_body.inertial = Inertial.from_geometry(
        Cylinder(radius=BODY_OUTER_R, length=BODY_H),
        mass=0.32,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
    )

    # ---- lid: flat glass disk on the gasket ----
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_solid(), "glass_lid"),
        material=glass,
        name="glass_lid",
    )
    lid.inertial = Inertial.from_geometry(
        Cylinder(radius=LID_R, length=LID_H),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.0, LID_H / 2.0)),
    )

    # ---- bail: U-shaped wire clamp ----
    bail = model.part("bail")
    bail.visual(
        mesh_from_cadquery(_bail_solid(), "wire_bail"),
        material=steel,
        name="wire_bail",
    )
    bail.inertial = Inertial.from_geometry(
        Cylinder(radius=0.002, length=BAIL_ARM_H),
        mass=0.025,
        origin=Origin(xyz=(0.0, 0.0, BAIL_ARM_H / 2.0)),
    )

    # ---- Articulation: body_to_lid (FIXED — lid clamped on rim) ----
    model.articulation(
        "body_to_lid",
        ArticulationType.FIXED,
        parent=jar_body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, LID_BOTTOM_Z)),
    )

    # ---- Articulation: body_to_bail (REVOLUTE) ----
    # Pivot axis along X, through both ear pivot points.
    # At q=0, bail arms extend straight up (+Z) from pivot, cross-bar over lid.
    # Positive q rotates around +X (RHR): +Z toward -Y, so cross-bar swings
    # from above (closed) toward -Y and down (open).
    model.articulation(
        "body_to_bail",
        ArticulationType.REVOLUTE,
        parent=jar_body,
        child=bail,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.0,
            lower=0.0,              # closed: cross-bar over lid
            upper=math.pi * 0.75,   # open: ~135 degrees swing
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    jar_body = object_model.get_part("jar_body")
    lid = object_model.get_part("lid")
    bail = object_model.get_part("bail")
    bail_joint = object_model.get_articulation("body_to_bail")
    lid_joint = object_model.get_articulation("body_to_lid")

    # ---- Structural checks: jar is cylindrical and tall ----
    body_aabb = ctx.part_world_aabb(jar_body)
    mn, mx = body_aabb
    dx = mx[0] - mn[0]
    dy = mx[1] - mn[1]
    dz = mx[2] - mn[2]
    ctx.check(
        "jar body is cylindrical (roughly circular section)",
        abs(dx - dy) < 0.015,
        details=f"x_extent={dx:.4f}, y_extent={dy:.4f}",
    )
    ctx.check(
        "jar body is tall (height > 1.8x section width)",
        dz > 1.8 * max(dx, dy),
        details=f"height={dz:.4f}, max_section={max(dx, dy):.4f}",
    )

    # ---- Wide mouth: lid covers the jar mouth opening ----
    ctx.expect_overlap(
        lid,
        jar_body,
        axes="xy",
        min_overlap=0.030,
        name="lid covers the jar mouth opening",
    )

    # ---- Gasket ring exists under the lid ----
    gasket_vis = jar_body.get_visual("gasket_ring")
    ctx.check(
        "gasket ring visual exists on jar body",
        gasket_vis is not None,
        details="gasket_ring visual not found",
    )
    if gasket_vis is not None:
        gasket_mat = gasket_vis.material
        ctx.check(
            "gasket has rubber material",
            gasket_mat is not None and getattr(gasket_mat, "name", None) == "rubber_gasket",
            details=f"gasket material={getattr(gasket_mat, 'name', None)}",
        )

    # ---- Lid sits on top of body (Z position check) ----
    lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid bottom is near the jar body top",
        lid_aabb is not None and lid_aabb[0][2] > BODY_H - 0.005,
        details=f"lid_bottom_z={lid_aabb[0][2]:.4f}, body_top={BODY_H}",
    )

    # ---- Bail joint is revolute with valid limits ----
    ctx.check(
        "bail joint is revolute",
        bail_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={bail_joint.articulation_type}",
    )
    bail_limits = bail_joint.motion_limits
    ctx.check(
        "bail joint has motion limits",
        bail_limits is not None
        and bail_limits.lower is not None
        and bail_limits.upper is not None
        and bail_limits.upper > bail_limits.lower,
        details=f"limits={bail_limits}",
    )

    # ---- Bail extends above the pivot when closed (AABB check) ----
    bail_aabb_rest = ctx.part_world_aabb(bail)
    bail_top_rest = bail_aabb_rest[1][2] if bail_aabb_rest else 0.0
    ctx.check(
        "bail extends above the pivot height when closed",
        bail_top_rest > PIVOT_Z + 0.010,
        details=f"bail_top_z={bail_top_rest:.4f}, pivot_z={PIVOT_Z}",
    )

    # ---- Bail opens: positive q swings bail cross-bar down ----
    with ctx.pose({bail_joint: 1.0}):  # ~57 degrees open
        bail_aabb_open = ctx.part_world_aabb(bail)
        bail_top_open = bail_aabb_open[1][2] if bail_aabb_open else 0.0
        ctx.check(
            "bail top moves lower when opened (swings away from lid)",
            bail_top_open < bail_top_rest - 0.005,
            details=f"rest_top={bail_top_rest:.4f}, open_top={bail_top_open:.4f}",
        )

    # ---- Bail cross-bar overlaps lid footprint when closed (XY) ----
    # The cross-bar is a thin wire, so Y overlap is just the wire diameter.
    # X overlap should be substantial (cross-bar spans the jar width).
    ctx.expect_overlap(
        bail,
        lid,
        axes="x",
        min_overlap=0.040,
        name="bail cross-bar spans the lid width when closed",
    )
    ctx.expect_overlap(
        bail,
        lid,
        axes="y",
        min_overlap=0.003,
        name="bail cross-bar contacts lid surface when closed",
    )

    # ---- Lid joint is FIXED (clamped by bail) ----
    ctx.check(
        "lid joint is fixed",
        lid_joint.articulation_type == ArticulationType.FIXED,
        details=f"type={lid_joint.articulation_type}",
    )

    # ---- Materials: glass, rubber, steel are distinct ----
    jar_mat = jar_body.get_visual("glass_jar").material
    bail_mat = bail.get_visual("wire_bail").material
    ctx.check(
        "jar body is clear glass",
        jar_mat is not None and getattr(jar_mat, "name", None) == "clear_glass",
        details=f"jar_mat={getattr(jar_mat, 'name', None)}",
    )
    ctx.check(
        "bail is stainless steel",
        bail_mat is not None and getattr(bail_mat, "name", None) == "stainless_steel",
        details=f"bail_mat={getattr(bail_mat, 'name', None)}",
    )

    return ctx.report()


object_model = build_object_model()
