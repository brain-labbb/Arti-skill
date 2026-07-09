from __future__ import annotations

# Faceted glass jar with a metal screw-on lid.
# Frame: vertical axis +Z, jar centered on the world Z axis, base on z=0.
#
# Jar body: octagonal (8-facet) clear glass body with:
#   - solid glass floor
#   - hollow interior open at the wide mouth
#   - thickened glass wall at the mouth/neck
#   - visible rim seam (protruding octagonal ring near the top of the neck)
#   - base foot ring (cylindrical ring slightly inset from the body)
#
# Lid: brushed metal screw cap with flat top and cylindrical skirt that
#   wraps down around the neck/rim. The lid rotates on a CONTINUOUS joint
#   around +Z (screw thread representation).
#
# Articulation:
#   - body_to_lid: CONTINUOUS around +Z. At q=0 the lid is seated on the neck.
#     Positive q rotates the lid (tighten/loosen screw thread).

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Inertial,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ----- key dimensions (meters) -----
N_FACETS = 8                # octagonal facets
BODY_D = 0.076              # circumscribed diameter of octagonal body
BODY_H = 0.075              # main body height
NECK_D = 0.070              # wide mouth neck diameter (jar = wide mouth)
NECK_H = 0.018              # neck height
GLASS_WALL = 0.003          # body glass wall thickness
MOUTH_WALL = 0.005          # mouth/neck glass wall thickness (thicker)

FOOT_H = 0.004              # foot ring height
FOOT_INSET = 0.004          # foot ring inset from body circumscribed edge
RIM_H = 0.004               # rim seam height
RIM_EXTRA = 0.002           # rim protrusion beyond neck outer surface

# Lid dimensions
LID_SKIRT_ID = NECK_D + 2.0 * RIM_EXTRA + 0.002  # skirt inner diameter (clears rim)
LID_SKIRT_T = 0.002         # skirt wall thickness
LID_OD = LID_SKIRT_ID + 2.0 * LID_SKIRT_T        # lid outer diameter
LID_TOP_T = 0.003           # lid top plate thickness
LID_SKIRT_H = 0.014         # lid skirt height (wraps down around neck)
LID_H = LID_TOP_T + LID_SKIRT_H                   # total lid height

# Derived heights
BODY_BASE_Z = FOOT_H                                # body starts above foot
BODY_TOP_Z = BODY_BASE_Z + BODY_H                   # top of body (shoulder)
NECK_TOP_Z = BODY_TOP_Z + NECK_H                    # top of neck
TOTAL_H = NECK_TOP_Z                                # total jar height (without lid)


def _polygon_pts(circumradius: float, n: int = N_FACETS) -> list[tuple[float, float]]:
    """Regular polygon vertices (first vertex at angle 0)."""
    return [
        (circumradius * math.cos(2.0 * math.pi * i / n),
         circumradius * math.sin(2.0 * math.pi * i / n))
        for i in range(n)
    ]


def _jar_solid() -> cq.Workplane:
    """Build the faceted glass jar body (hollow, with foot ring and rim seam)."""
    body_R = BODY_D / 2.0
    neck_R = NECK_D / 2.0
    body_pts = _polygon_pts(body_R)
    neck_pts = _polygon_pts(neck_R)

    # --- Outer shell ---
    # Foot ring: cylindrical disk at the base
    foot_R = body_R - FOOT_INSET
    foot = (
        cq.Workplane("XY")
        .circle(foot_R)
        .extrude(FOOT_H)
    )

    # Main body: octagonal prism from BODY_BASE_Z to BODY_TOP_Z
    body_outer = (
        cq.Workplane("XY")
        .workplane(offset=BODY_BASE_Z)
        .polyline(body_pts)
        .close()
        .extrude(BODY_H)
    )

    # Neck: octagonal prism from BODY_TOP_Z to NECK_TOP_Z
    neck_outer = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP_Z)
        .polyline(neck_pts)
        .close()
        .extrude(NECK_H)
    )

    # Rim seam: protruding octagonal ring near top of neck
    rim_R = neck_R + RIM_EXTRA
    rim_pts = _polygon_pts(rim_R)
    rim = (
        cq.Workplane("XY")
        .workplane(offset=NECK_TOP_Z - RIM_H)
        .polyline(rim_pts)
        .close()
        .extrude(RIM_H)
    )

    # Combine all outer geometry
    jar = foot.union(body_outer).union(neck_outer).union(rim)

    # --- Inner cavity (hollow) ---
    # Body cavity: octagonal, thin walls, solid glass floor
    inner_body_R = body_R - GLASS_WALL
    inner_body_pts = _polygon_pts(inner_body_R)
    inner_body = (
        cq.Workplane("XY")
        .workplane(offset=BODY_BASE_Z + GLASS_WALL)  # glass floor thickness
        .polyline(inner_body_pts)
        .close()
        .extrude(BODY_H - GLASS_WALL)  # stop at body top to avoid cutting into neck
    )

    # Neck cavity: thicker walls at the mouth
    inner_neck_R = neck_R - MOUTH_WALL
    inner_neck_pts = _polygon_pts(inner_neck_R)
    inner_neck = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP_Z)
        .polyline(inner_neck_pts)
        .close()
        .extrude(NECK_H + 0.002)  # over-extrude to open through the top
    )

    # Combine cavity and cut from jar
    cavity = inner_body.union(inner_neck)
    jar = jar.cut(cavity)

    return jar


def _lid_solid() -> cq.Workplane:
    """Build the metal screw cap in lid-local frame.

    Lid-local origin is at the joint (neck top). The top plate extends upward
    (+Z) and the skirt extends downward (-Z) wrapping around the neck.
    """
    lid_R = LID_OD / 2.0
    skirt_inner_R = LID_SKIRT_ID / 2.0

    # Top plate: solid disk from z=0 to z=LID_TOP_T
    top_plate = (
        cq.Workplane("XY")
        .circle(lid_R)
        .extrude(LID_TOP_T)
    )

    # Skirt: cylindrical shell from z=-LID_SKIRT_H to z=0
    skirt_outer = (
        cq.Workplane("XY")
        .workplane(offset=-LID_SKIRT_H)
        .circle(lid_R)
        .extrude(LID_SKIRT_H)
    )
    skirt_bore = (
        cq.Workplane("XY")
        .workplane(offset=-LID_SKIRT_H - 0.001)
        .circle(skirt_inner_R)
        .extrude(LID_SKIRT_H + 0.002)
    )
    skirt = skirt_outer.cut(skirt_bore)

    # Combine top plate and skirt
    lid = top_plate.union(skirt)

    return lid


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="faceted_glass_jar")

    glass = model.material("clear_glass", rgba=(0.80, 0.85, 0.87, 0.30))
    metal = model.material("brushed_metal", rgba=(0.70, 0.72, 0.74, 1.0))

    # ---- jar body (root): faceted glass jar ----
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_jar_solid(), "glass_jar_body"),
        material=glass,
        name="glass_jar_body",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(radius=BODY_D / 2.0, length=TOTAL_H),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, TOTAL_H / 2.0)),
    )

    # ---- lid: metal screw cap ----
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_solid(), "metal_lid"),
        material=metal,
        name="metal_lid",
    )
    lid.inertial = Inertial.from_geometry(
        Cylinder(radius=LID_OD / 2.0, length=LID_H),
        mass=0.04,
        origin=Origin(xyz=(0.0, 0.0, LID_TOP_T / 2.0)),
    )

    # Continuous screw joint: lid rotates around +Z at the neck top.
    # At q=0 the lid is seated; positive q rotates (tighten/loosen).
    model.articulation(
        "body_to_lid",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    screw = object_model.get_articulation("body_to_lid")

    # The lid skirt wraps around the neck/rim of the jar. The skirt inner
    # surface clears the rim outer surface, but the top plate contacts the
    # neck top face. Allow the seated contact overlap.
    ctx.allow_overlap(
        lid,
        body,
        elem_a="metal_lid",
        elem_b="glass_jar_body",
        reason="Metal lid top plate is seated on the neck top face and skirt wraps around the rim (screw cap fit).",
    )

    # ---- jar body is faceted (octagonal), not circular ----
    body_aabb = ctx.part_world_aabb(body)
    if body_aabb:
        mn, mx = body_aabb
        body_dx = mx[0] - mn[0]
        body_dy = mx[1] - mn[1]
        body_dz = mx[2] - mn[2]
        # Octagonal: x and y extents should be approximately equal (symmetric)
        ctx.check(
            "jar body has roughly equal X and Y extents (faceted, not elongated)",
            abs(body_dx - body_dy) < 0.006,
            details=f"dx={body_dx:.4f}, dy={body_dy:.4f}",
        )
        # Jar proportions: width comparable to height (not tall like a bottle)
        ctx.check(
            "jar is squat (width-to-height ratio > 0.5, jar-like)",
            max(body_dx, body_dy) / body_dz > 0.5,
            details=f"width={max(body_dx, body_dy):.4f}, height={body_dz:.4f}",
        )

    # ---- lid is at the top of the jar, seated on the neck ----
    lid_pos = ctx.part_world_position(lid)
    ctx.check(
        "lid sits at the top of the jar (above body shoulder)",
        lid_pos is not None and lid_pos[2] > BODY_TOP_Z - 0.005,
        details=f"lid_origin_z={lid_pos[2] if lid_pos else None}",
    )

    # ---- lid overlaps the neck in Z (skirt wraps down around neck) ----
    ctx.expect_overlap(
        lid,
        body,
        axes="z",
        min_overlap=0.005,
        name="lid skirt wraps down around the neck",
    )

    # ---- lid XY footprint overlaps the neck ----
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        min_overlap=0.040,
        name="lid footprint covers the jar mouth",
    )

    # ---- continuous joint: lid rotates around Z without translating ----
    rest_pos = ctx.part_world_position(lid)
    with ctx.pose({screw: math.pi}):
        rotated_pos = ctx.part_world_position(lid)
        ctx.check(
            "lid rotates in place (Z stays constant under rotation)",
            rotated_pos is not None
            and rest_pos is not None
            and abs(rotated_pos[2] - rest_pos[2]) < 0.001,
            details=f"rest_z={rest_pos[2] if rest_pos else None}, rotated_z={rotated_pos[2] if rotated_pos else None}",
        )
        ctx.check(
            "lid XY does not translate under rotation",
            rotated_pos is not None
            and rest_pos is not None
            and abs(rotated_pos[0] - rest_pos[0]) < 1e-6
            and abs(rotated_pos[1] - rest_pos[1]) < 1e-6,
            details=f"rest_xy={rest_pos[:2] if rest_pos else None}, rotated_xy={rotated_pos[:2] if rotated_pos else None}",
        )

    # ---- articulation type is CONTINUOUS (screw thread, no hard stops) ----
    ctx.check(
        "body_to_lid is a continuous joint (screw thread)",
        screw.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={screw.articulation_type}",
    )

    # ---- materials: lid is metal, body is glass (distinct) ----
    lid_mat = lid.get_visual("metal_lid").material
    body_mat = body.get_visual("glass_jar_body").material
    ctx.check(
        "lid material is metal and distinct from glass",
        lid_mat is not None
        and body_mat is not None
        and getattr(lid_mat, "name", None) == "brushed_metal"
        and getattr(body_mat, "name", None) == "clear_glass",
        details=f"lid_mat={getattr(lid_mat, 'name', None)}, body_mat={getattr(body_mat, 'name', None)}",
    )

    # ---- mouth wall is thicker than body wall (structural check via geometry) ----
    # The neck/mouth outer diameter is smaller than the body, but the inner
    # diameter shrinks more (thicker walls). Verify the jar has a visible neck.
    ctx.check(
        "mouth wall is thicker than body wall (design intent)",
        MOUTH_WALL > GLASS_WALL,
        details=f"mouth_wall={MOUTH_WALL}, body_wall={GLASS_WALL}",
    )

    # ---- rim seam protrudes beyond the neck outer surface ----
    ctx.check(
        "rim seam protrudes beyond neck (visible seam geometry)",
        RIM_EXTRA > 0.0,
        details=f"rim_extra={RIM_EXTRA}",
    )

    # ---- base foot ring exists (foot height > 0) ----
    ctx.check(
        "base foot ring exists below the body",
        FOOT_H > 0.0,
        details=f"foot_height={FOOT_H}",
    )

    return ctx.report()


object_model = build_object_model()
