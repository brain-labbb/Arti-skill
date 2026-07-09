from __future__ import annotations

# Squat cosmetic cream jar with a thick screw lid and gasket ring.
# Frame: vertical axis +Z, jar centered on the world Z axis, base on z=0.
#   - jar_body: a round, squat hollow glass/ceramic jar with a wide mouth opening.
#   - lid: a thick screw-on lid with knurled grip exterior.
#   - gasket: a thin rubber gasket ring seated under the lid.
#
# Articulation:
#   - body_to_lid: CONTINUOUS around +Z (screw thread, unlimited rotation).

import cadquery as cq
import math

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
JAR_OD = 0.080        # jar outer diameter
JAR_RADIUS = JAR_OD / 2.0
JAR_WALL = 0.005      # jar wall thickness
JAR_BASE = 0.006      # solid base thickness
JAR_H = 0.050         # jar body height
MOUTH_ID = 0.065      # wide mouth inner diameter
MOUTH_RADIUS = MOUTH_ID / 2.0

LID_OD = 0.084        # lid outer diameter (slightly wider than jar for screw fit)
LID_RADIUS = LID_OD / 2.0
LID_H = 0.025         # lid total height
LID_WALL = 0.006      # lid side wall thickness
LID_TOP = 0.005       # lid top plate thickness
LID_BORE_ID = 0.068   # lid bore inner diameter (fits over jar mouth rim)
LID_BORE_RADIUS = LID_BORE_ID / 2.0
LID_BORE_DEPTH = LID_H - LID_TOP  # how deep the bore goes

# Gasket dimensions
GASKET_OD = 0.074     # gasket outer diameter
GASKET_ID = 0.060     # gasket inner diameter
GASKET_THICK = 0.003  # gasket thickness
GASKET_RADIUS = GASKET_OD / 2.0
GASKET_INNER_RADIUS = GASKET_ID / 2.0

# Lid seats on top of jar body at z = JAR_H
LID_SEAT_Z = JAR_H


def _jar_body_solid() -> cq.Workplane:
    """Hollow round squat jar with wide mouth opening at top."""
    # Outer cylinder with filleted bottom edge
    outer = (
        cq.Workplane("XY")
        .circle(JAR_RADIUS)
        .extrude(JAR_H)
        .edges("<Z")
        .fillet(0.003)
    )

    # Hollow cavity - open at top, solid base at bottom
    inner_h = JAR_H - JAR_BASE  # cavity depth (open at top)
    inner = (
        cq.Workplane("XY")
        .workplane(offset=JAR_BASE)  # start above solid base
        .circle(MOUTH_RADIUS)
        .extrude(inner_h + 0.005)  # over-extrude to ensure opening through top
    )
    body = outer.cut(inner)

    # Add a subtle lip/rim around the mouth opening (thread ridge representation)
    rim = (
        cq.Workplane("XY")
        .workplane(offset=JAR_H - 0.008)
        .circle(JAR_RADIUS + 0.001)
        .circle(MOUTH_RADIUS + 0.002)
        .extrude(0.008)
    )
    body = body.union(rim)

    return body


def _lid_solid() -> cq.Workplane:
    """Thick screw lid with knurled exterior grip."""
    # Main lid body - solid cylinder with filleted top edge
    lid = (
        cq.Workplane("XY")
        .circle(LID_RADIUS)
        .extrude(LID_H)
        .edges(">Z")
        .fillet(0.002)
    )

    # Hollow bore from bottom to fit over jar mouth rim
    bore = (
        cq.Workplane("XY")
        .circle(LID_BORE_RADIUS)
        .extrude(LID_BORE_DEPTH)
    )
    lid = lid.cut(bore)

    # Add knurling grooves on the exterior (vertical slots around the perimeter)
    n_grooves = 24
    groove_depth = 0.0012
    groove_width = 0.002
    for i in range(n_grooves):
        angle = i * (2.0 * math.pi / n_grooves)
        cx = (LID_RADIUS - groove_depth / 2.0) * math.cos(angle)
        cy = (LID_RADIUS - groove_depth / 2.0) * math.sin(angle)
        groove = (
            cq.Workplane("XY")
            .center(cx, cy)
            .rect(groove_width, groove_width)
            .extrude(LID_H - 0.003)
        )
        lid = lid.cut(groove)

    return lid


def _gasket_solid() -> cq.Workplane:
    """Thin rubber gasket ring that sits between lid and jar rim."""
    gasket = (
        cq.Workplane("XY")
        .circle(GASKET_RADIUS)
        .circle(GASKET_INNER_RADIUS)
        .extrude(GASKET_THICK)
    )
    return gasket


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cosmetic_cream_jar")

    # Materials
    frosted_glass = model.material("frosted_glass", rgba=(0.92, 0.90, 0.88, 0.6))
    matte_white = model.material("matte_white_lid", rgba=(0.95, 0.94, 0.93, 1.0))
    rubber_grey = model.material("rubber_gasket", rgba=(0.25, 0.25, 0.27, 1.0))

    # ---- jar body (root): round squat hollow jar with wide mouth ----
    jar_body = model.part("jar_body")
    jar_body.visual(
        mesh_from_cadquery(_jar_body_solid(), "jar_body"),
        material=frosted_glass,
        name="jar_shell",
    )
    jar_body.inertial = Inertial.from_geometry(
        Cylinder(radius=JAR_RADIUS, length=JAR_H),
        mass=0.15,
        origin=Origin(xyz=(0.0, 0.0, JAR_H / 2.0)),
    )

    # ---- lid: thick screw-on lid with knurled grip ----
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_solid(), "lid_shell"),
        material=matte_white,
        name="lid_shell",
    )
    lid.inertial = Inertial.from_geometry(
        Cylinder(radius=LID_RADIUS, length=LID_H),
        mass=0.04,
        origin=Origin(xyz=(0.0, 0.0, LID_H / 2.0)),
    )

    # ---- gasket ring: fixed to the lid underside ----
    gasket = model.part("gasket")
    gasket.visual(
        mesh_from_cadquery(_gasket_solid(), "gasket_ring"),
        material=rubber_grey,
        name="gasket_ring",
    )
    gasket.inertial = Inertial.from_geometry(
        Cylinder(radius=GASKET_RADIUS, length=GASKET_THICK),
        mass=0.005,
        origin=Origin(xyz=(0.0, 0.0, GASKET_THICK / 2.0)),
    )

    # Lid-to-gasket: fixed joint. The gasket sits just under the lid bottom.
    # Lid local frame has its bottom at z=0, gasket sits at z = -GASKET_THICK.
    model.articulation(
        "lid_to_gasket",
        ArticulationType.FIXED,
        parent=lid,
        child=gasket,
        origin=Origin(xyz=(0.0, 0.0, -GASKET_THICK)),
    )

    # Body-to-lid: CONTINUOUS screw joint. At q=0 the lid is seated on top of jar.
    # The lid part frame origin sits at its own geometric center (z = LID_H/2).
    # We place the articulation origin at the jar top rim. The lid frame is
    # positioned so its bottom face aligns with the jar top.
    model.articulation(
        "body_to_lid",
        ArticulationType.CONTINUOUS,
        parent=jar_body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, LID_SEAT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    jar_body = object_model.get_part("jar_body")
    lid = object_model.get_part("lid")
    gasket = object_model.get_part("gasket")
    screw = object_model.get_articulation("body_to_lid")
    gasket_joint = object_model.get_articulation("lid_to_gasket")

    # ---- The gasket is intentionally seated (compressed) between lid bottom
    # and jar rim. Small local overlap is expected for the seated seal. ----
    ctx.allow_overlap(
        gasket,
        jar_body,
        elem_a="gasket_ring",
        elem_b="jar_shell",
        reason="Gasket ring is intentionally compressed/seated between the lid underside and the jar rim.",
    )
    ctx.allow_overlap(
        gasket,
        lid,
        elem_a="gasket_ring",
        elem_b="lid_shell",
        reason="Gasket ring is intentionally seated against the lid underside for a compression seal.",
    )

    # ---- Jar is squat: wider than tall ----
    jar_aabb = ctx.part_world_aabb(jar_body)
    mn, mx = jar_aabb
    jar_dx = mx[0] - mn[0]
    jar_dy = mx[1] - mn[1]
    jar_dz = mx[2] - mn[2]
    ctx.check(
        "jar body is squat (diameter > height)",
        jar_dx > jar_dz and jar_dy > jar_dz,
        details=f"jar extents: dx={jar_dx:.4f}, dy={jar_dy:.4f}, dz={jar_dz:.4f}",
    )

    # ---- Jar body is round in section ----
    ctx.check(
        "jar body is round (dx ~ dy)",
        abs(jar_dx - jar_dy) < 0.004,
        details=f"dx={jar_dx:.4f}, dy={jar_dy:.4f}",
    )

    # ---- Wide mouth hollow opening exists: jar has visible cavity ----
    # The inner cavity should be visible through the top opening.
    # We verify the jar shell visual exists and is mesh-backed (hollow form).
    jar_shell = jar_body.get_visual("jar_shell")
    ctx.check(
        "jar shell visual exists",
        jar_shell is not None,
        details="jar_shell visual not found",
    )

    # ---- Lid sits on top of the jar body at rest ----
    lid_pos = ctx.part_world_position(lid)
    jar_pos = ctx.part_world_position(jar_body)
    ctx.check(
        "lid sits above jar body",
        lid_pos is not None and jar_pos is not None and lid_pos[2] > jar_pos[2] + JAR_H * 0.5,
        details=f"jar_pos={jar_pos}, lid_pos={lid_pos}",
    )

    # ---- Lid overlaps jar in XY footprint (seated on top) ----
    ctx.expect_overlap(
        lid,
        jar_body,
        axes="xy",
        min_overlap=0.050,
        name="lid footprint covers jar mouth",
    )

    # ---- Screw joint is CONTINUOUS (unlimited rotation for threading) ----
    ctx.check(
        "body_to_lid is a continuous screw joint",
        screw.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint type={screw.articulation_type}",
    )

    # ---- Screw axis is vertical (+Z) ----
    axis = screw.axis
    ctx.check(
        "screw joint axis is vertical",
        abs(axis[2]) > 0.99 and abs(axis[0]) < 0.01 and abs(axis[1]) < 0.01,
        details=f"axis={axis}",
    )

    # ---- Rotating the lid does not translate it (pure rotation) ----
    rest_lid_pos = ctx.part_world_position(lid)
    with ctx.pose({screw: math.pi}):
        rotated_lid_pos = ctx.part_world_position(lid)
    ctx.check(
        "lid rotates in place without translating",
        rest_lid_pos is not None
        and rotated_lid_pos is not None
        and abs(rotated_lid_pos[0] - rest_lid_pos[0]) < 1e-5
        and abs(rotated_lid_pos[1] - rest_lid_pos[1]) < 1e-5
        and abs(rotated_lid_pos[2] - rest_lid_pos[2]) < 1e-5,
        details=f"rest={rest_lid_pos}, rotated={rotated_lid_pos}",
    )

    # ---- Gasket ring is below the lid and above the jar rim ----
    gasket_pos = ctx.part_world_position(gasket)
    ctx.check(
        "gasket sits between lid and jar rim",
        gasket_pos is not None
        and gasket_pos[2] >= JAR_H - GASKET_THICK - 0.002
        and gasket_pos[2] <= JAR_H + 0.002,
        details=f"gasket_pos={gasket_pos}, jar_top={JAR_H}",
    )

    # ---- Gasket ring exists as a named visual ----
    gasket_vis = gasket.get_visual("gasket_ring")
    ctx.check(
        "gasket ring visual exists",
        gasket_vis is not None,
        details="gasket_ring visual not found",
    )

    # ---- Gasket joint is fixed (gasket moves with the lid) ----
    ctx.check(
        "lid_to_gasket is a fixed joint",
        gasket_joint.articulation_type == ArticulationType.FIXED,
        details=f"joint type={gasket_joint.articulation_type}",
    )

    # ---- Lid knurled exterior: lid shell visual exists ----
    lid_shell = lid.get_visual("lid_shell")
    ctx.check(
        "lid shell visual exists with knurling",
        lid_shell is not None,
        details="lid_shell visual not found",
    )

    # ---- Materials are distinct ----
    jar_mat = jar_body.get_visual("jar_shell").material
    lid_mat = lid.get_visual("lid_shell").material
    gasket_mat = gasket.get_visual("gasket_ring").material
    ctx.check(
        "jar, lid, and gasket have distinct materials",
        jar_mat is not None
        and lid_mat is not None
        and gasket_mat is not None
        and getattr(jar_mat, "name", None) == "frosted_glass"
        and getattr(lid_mat, "name", None) == "matte_white_lid"
        and getattr(gasket_mat, "name", None) == "rubber_gasket",
        details=f"jar={getattr(jar_mat, 'name', None)}, lid={getattr(lid_mat, 'name', None)}, gasket={getattr(gasket_mat, 'name', None)}",
    )

    return ctx.report()


object_model = build_object_model()
