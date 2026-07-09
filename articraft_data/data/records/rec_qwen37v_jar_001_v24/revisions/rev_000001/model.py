from __future__ import annotations

# Faceted glass jar with metal swing-top lid and clamp bail.
# Octagonal (8-facet) glass body with thick walls visible at the mouth,
# a flat metal disc lid, and a wire bail clamp that pivots on two side
# revolute hinges at the neck lugs.
#
# Frame: jar axis along +Z, base on z=0, centered at origin.
# Pivot axis along Y through both side lugs at ±Y sides of the neck.
#
# Articulations:
#   bail_pivot: REVOLUTE around Y at lug height, 0 = closed over lid, positive = open

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
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---- key dimensions (meters) ----
SIDES = 8
BODY_R = 0.038                # outer radius (center-to-vertex) of octagon
BODY_H = 0.052                # body height
WALL = 0.004                  # glass wall thickness
BASE_THICK = 0.006            # thick glass base
NECK_R = 0.030                # neck outer radius
NECK_H = 0.012                # neck height above body shoulder
MOUTH_WALL = 0.003            # glass wall thickness at the mouth rim
RIM_TOP_Z = BODY_H + NECK_H  # top of the rim

LID_R = 0.033                 # metal lid radius (slightly over neck)
LID_H = 0.003                 # lid disc thickness

# Bail pivot geometry
LUG_Z = BODY_H + NECK_H * 0.45   # pivot lug center height
LUG_HALF_SPAN = NECK_R + 0.005   # half-distance between lugs (along Y)
PIVOT_PIN_R = 0.0025             # pivot pin radius
BAIL_WIRE_R = 0.0018             # bail wire radius


def _octagon_profile(radius: float) -> list[tuple[float, float]]:
    """Vertices of a regular octagon in the XY plane."""
    pts = []
    for i in range(SIDES):
        a = 2.0 * math.pi * i / SIDES + math.pi / SIDES
        pts.append((radius * math.cos(a), radius * math.sin(a)))
    return pts


def _jar_glass_solid() -> cq.Workplane:
    """Faceted octagonal glass jar with hollow interior, neck, and rim."""
    # Outer octagonal shell
    outer_pts = _octagon_profile(BODY_R)
    outer = (
        cq.Workplane("XY")
        .polyline(outer_pts)
        .close()
        .extrude(BODY_H)
    )

    # Inner cavity (smaller octagon, offset up by base thickness)
    inner_r = BODY_R - WALL
    inner_pts = _octagon_profile(inner_r)
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=BASE_THICK)
        .polyline(inner_pts)
        .close()
        .extrude(BODY_H - BASE_THICK + 0.001)  # slight overshoot to cut cleanly
    )
    jar = outer.cut(cavity)

    # Shoulder transition: small chamfer ring where body meets neck
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_H - 0.003)
        .circle(BODY_R * 0.92)
        .circle(NECK_R)
        .extrude(0.003)
    )
    jar = jar.union(shoulder)

    # Neck cylinder (slightly inset from body facets)
    neck_outer = (
        cq.Workplane("XY")
        .workplane(offset=BODY_H)
        .circle(NECK_R)
        .extrude(NECK_H)
    )
    # Neck inner bore (wall thickness at mouth)
    neck_bore = (
        cq.Workplane("XY")
        .workplane(offset=BODY_H)
        .circle(NECK_R - MOUTH_WALL)
        .extrude(NECK_H + 0.001)
    )
    neck = neck_outer.cut(neck_bore)
    jar = jar.union(neck)

    # Thickened rim at mouth top: visible glass wall thickness
    rim_outer = (
        cq.Workplane("XY")
        .workplane(offset=RIM_TOP_Z - 0.003)
        .circle(NECK_R + 0.0015)
        .extrude(0.003)
    )
    rim_bore = (
        cq.Workplane("XY")
        .workplane(offset=RIM_TOP_Z - 0.004)
        .circle(NECK_R - MOUTH_WALL)
        .extrude(0.005)
    )
    rim = rim_outer.cut(rim_bore)
    jar = jar.union(rim)

    # Pivot lugs on ±Y sides of the neck - built as small boxes that clearly
    # intersect the neck cylinder to avoid disconnected mesh islands.
    lug_half_h = PIVOT_PIN_R * 1.5
    lug_y_depth = 0.007
    for sign in (-1, 1):
        lug_center_y = sign * (NECK_R + lug_y_depth * 0.5 - 0.002)
        lug = (
            cq.Workplane("XY")
            .workplane(offset=LUG_Z - lug_half_h)
            .center(0, lug_center_y)
            .rect(0.008, lug_y_depth)
            .extrude(lug_half_h * 2)
        )
        jar = jar.union(lug)

    return jar


def _lid_solid() -> cq.Workplane:
    """Metal lid disc with a raised rim edge and central emboss."""
    disc = (
        cq.Workplane("XY")
        .circle(LID_R)
        .extrude(LID_H)
    )
    # Raised outer rim
    edge = (
        cq.Workplane("XY")
        .workplane(offset=LID_H)
        .circle(LID_R)
        .circle(LID_R - 0.0025)
        .extrude(0.0012)
    )
    # Small central emboss
    emboss = (
        cq.Workplane("XY")
        .workplane(offset=LID_H)
        .circle(0.008)
        .extrude(0.001)
    )
    return disc.union(edge).union(emboss)


def _bail_mesh():
    """Wire bail clamp: U-shaped wire from one pivot lug over the lid to the other."""
    # Bail part frame origin is at (0, 0, LUG_Z) in world (center of jar at lug height).
    # Pivot axis is along Y.
    # Bail-local: Y axis connects the two lugs, Z is up, X is perpendicular.
    top_z = RIM_TOP_Z + LID_H + 0.006 - LUG_Z  # bail arch top in local Z
    hs = LUG_HALF_SPAN  # half-span to each lug

    # Wire path: from one lug up, across the top, down to the other lug
    points = [
        (0.0, -hs, 0.0),                      # left lug pivot
        (0.0, -hs * 0.85, top_z * 0.35),      # left arm rising
        (0.0, -hs * 0.55, top_z * 0.75),      # left arm approaching top
        (0.0, -hs * 0.25, top_z * 0.95),      # near top left
        (0.0, 0.0, top_z),                     # top center (over lid)
        (0.0, hs * 0.25, top_z * 0.95),       # near top right
        (0.0, hs * 0.55, top_z * 0.75),       # right arm leaving top
        (0.0, hs * 0.85, top_z * 0.35),       # right arm descending
        (0.0, hs, 0.0),                        # right lug pivot
    ]

    geom = tube_from_spline_points(
        points,
        radius=BAIL_WIRE_R,
        samples_per_segment=14,
        radial_segments=12,
        cap_ends=True,
    )
    return mesh_from_geometry(geom, "bail_wire")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="faceted_bail_jar")

    # Materials
    glass_clear = model.material("glass_clear", rgba=(0.82, 0.90, 0.86, 0.42))
    metal_brushed = model.material("metal_brushed", rgba=(0.76, 0.78, 0.80, 1.0))
    metal_bail = model.material("metal_bail", rgba=(0.40, 0.42, 0.44, 1.0))
    gasket_orange = model.material("gasket_orange", rgba=(0.85, 0.40, 0.15, 1.0))

    # ---- jar body (root): faceted glass + neck + lugs ----
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_jar_glass_solid(), "jar_glass"),
        material=glass_clear,
        name="jar_glass",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, BODY_H + NECK_H),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, (BODY_H + NECK_H) * 0.5)),
    )

    # ---- lid: metal disc sitting on the mouth rim ----
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_solid(), "lid_disc"),
        material=metal_brushed,
        name="lid_disc",
    )
    # Rubber gasket ring on the underside of the lid
    gasket = (
        cq.Workplane("XY")
        .circle(LID_R - 0.001)
        .circle(NECK_R - MOUTH_WALL + 0.001)
        .extrude(0.0018)
    )
    lid.visual(
        mesh_from_cadquery(gasket, "lid_gasket"),
        material=gasket_orange,
        name="lid_gasket",
        origin=Origin(xyz=(0.0, 0.0, -0.0018)),
    )
    lid.inertial = Inertial.from_geometry(
        Cylinder(LID_R, LID_H + 0.002),
        mass=0.025,
        origin=Origin(xyz=(0.0, 0.0, LID_H * 0.5)),
    )

    # Fixed joint: lid sits on the rim
    model.articulation(
        "lid_seat",
        ArticulationType.FIXED,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, RIM_TOP_Z)),
    )

    # ---- bail: wire clamp ----
    bail = model.part("bail")
    bail.visual(_bail_mesh(), material=metal_bail, name="bail_wire")
    bail.inertial = Inertial.from_geometry(
        Box((0.006, 2 * LUG_HALF_SPAN, 0.025)),
        mass=0.012,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )

    # Bail pivots on revolute joint around Y axis at lug height
    # At q=0: bail arch is above the lid (closed)
    # At q>0: bail swings open (arch rotates away from +Z toward -X)
    model.articulation(
        "bail_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=bail,
        origin=Origin(xyz=(0.0, 0.0, LUG_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=0.0,
            upper=2.6,  # ~149° open
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    bail = object_model.get_part("bail")
    bail_pivot = object_model.get_articulation("bail_pivot")
    lid_seat = object_model.get_articulation("lid_seat")

    # Bail wire pivots inside the lugs - small intentional overlap
    ctx.allow_overlap(
        bail, body,
        elem_a="bail_wire", elem_b="jar_glass",
        reason="The bail wire ends pivot inside the side lugs on the jar neck.",
    )

    # ---- faceted jar is roughly as wide as it is tall (squat-to-square proportion) ----
    body_aabb = ctx.part_world_aabb(body)
    dx = body_aabb[1][0] - body_aabb[0][0]
    dy = body_aabb[1][1] - body_aabb[0][1]
    dz = body_aabb[1][2] - body_aabb[0][2]
    ctx.check(
        "jar body width is at least 70% of height",
        min(dx, dy) > dz * 0.7,
        details=f"body extents: dx={dx:.4f}, dy={dy:.4f}, dz={dz:.4f}",
    )

    # ---- lid sits at mouth level (on the rim) ----
    lid_pos = ctx.part_world_position(lid)
    ctx.check(
        "lid sits at rim top",
        lid_pos is not None and abs(lid_pos[2] - RIM_TOP_Z) < 0.005,
        details=f"lid_z={lid_pos}, rim_top_z={RIM_TOP_Z}",
    )

    # ---- lid overlaps body footprint in XY (covers the mouth) ----
    ctx.expect_overlap(
        lid, body, axes="xy", min_overlap=0.04,
        name="lid covers the mouth opening",
    )

    # ---- bail is mounted near the neck ----
    bail_pos = ctx.part_world_position(bail)
    ctx.check(
        "bail is above jar midline",
        bail_pos is not None and bail_pos[2] > BODY_H * 0.5,
        details=f"bail_z={bail_pos}",
    )

    # ---- bail_pivot is a non-fixed revolute joint with limits ----
    ctx.check(
        "bail_pivot is revolute",
        bail_pivot.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={bail_pivot.articulation_type}",
    )
    ctx.check(
        "bail_pivot has finite motion range",
        (bail_pivot.motion_limits.upper is not None
         and bail_pivot.motion_limits.lower is not None
         and bail_pivot.motion_limits.upper > bail_pivot.motion_limits.lower),
        details=f"limits={bail_pivot.motion_limits}",
    )

    # ---- bail swings open: arch position changes visibly ----
    bail_rest_aabb = ctx.part_element_world_aabb(bail, elem="bail_wire")
    bail_rest_top = bail_rest_aabb[1][2] if bail_rest_aabb else 0
    with ctx.pose({bail_pivot: 2.2}):
        bail_open_aabb = ctx.part_element_world_aabb(bail, elem="bail_wire")
    bail_open_top = bail_open_aabb[1][2] if bail_open_aabb else 0
    ctx.check(
        "bail arch visibly moves when opened",
        bail_rest_aabb is not None and bail_open_aabb is not None,
        details=f"rest_top={bail_rest_top}, open_top={bail_open_top}",
    )
    if bail_rest_aabb is not None and bail_open_aabb is not None:
        # The arch top should drop when the bail swings open
        arch_moved = abs(bail_open_top - bail_rest_top)
        ctx.check(
            "bail arch top moves by >8mm between closed and open",
            arch_moved > 0.008,
            details=f"rest_top={bail_rest_top:.4f}, open_top={bail_open_top:.4f}, moved={arch_moved:.4f}",
        )

    # ---- at closed pose, bail arch is above the lid ----
    lid_top_z = RIM_TOP_Z + LID_H
    ctx.check(
        "closed bail arch is at or above lid level",
        bail_rest_top >= lid_top_z - 0.005,
        details=f"bail_rest_top={bail_rest_top:.4f}, lid_top_z={lid_top_z:.4f}",
    )

    # ---- lid_seat is a fixed joint ----
    ctx.check(
        "lid_seat is fixed",
        lid_seat.articulation_type == ArticulationType.FIXED,
        details=f"type={lid_seat.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
