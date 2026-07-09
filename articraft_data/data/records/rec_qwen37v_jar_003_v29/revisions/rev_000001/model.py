from __future__ import annotations

# Spice jar variant: wide-mouth glass jar with rose-gold screw lid and
# rotating perforated shaker insert.
#
# Frame: vertical axis +Z, jar centered on the world Z axis, base on z=0.
#   - body  : clear glass cylindrical jar, hollow with wide mouth opening.
#   - lid   : rose-gold screw cap that threads onto the jar neck.
#             Small clamp hooks on the skirt engage the jar neck lip.
#   - shaker: perforated disk insert seated inside the lid that rotates
#             to open/close the shaker holes (REVOLUTE, limited range).
#
# Articulations:
#   - body_to_lid  : CONTINUOUS around +Z (screw thread, lid rotates freely).
#   - lid_to_shaker: REVOLUTE around +Z, limited ±π/2 rad (shaker alignment).

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
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ----- key dimensions (meters) -----
# Jar body: wide cylindrical glass jar
JAR_OD = 0.065          # outer diameter of jar body
JAR_ID = 0.055          # inner diameter (wide mouth bore)
JAR_WALL = (JAR_OD - JAR_ID) / 2.0  # glass wall thickness ~0.005
JAR_BASE_H = 0.005      # solid glass base thickness
JAR_BODY_H = 0.085      # total jar body height (base to top of neck)

# Neck section: slightly narrower threaded section at top
NECK_OD = 0.058         # outer diameter of threaded neck
NECK_H = 0.015          # neck height above main body shoulder
NECK_LIP_H = 0.003      # lip ring at top of neck for clamp engagement
NECK_LIP_OD = 0.062     # lip outer diameter (slightly wider for hooks)

# Shoulder transition height
SHOULDER_Z = JAR_BODY_H - NECK_H  # z where shoulder begins

# Lid: rose-gold screw cap
LID_OD = 0.064          # outer diameter (slightly wider than neck lip)
LID_WALL = 0.004        # lid wall thickness
LID_TOP_H = 0.005       # top plate thickness
LID_SKIRT_H = 0.018     # skirt depth that threads onto neck
LID_TOTAL_H = LID_TOP_H + LID_SKIRT_H

# Shaker insert: perforated disk inside the lid
SHAKER_OD = 0.048       # fits inside the lid bore
SHAKER_THICK = 0.003    # disk thickness
SHAKER_HOLE_D = 0.003   # hole diameter
SHAKER_PITCH = 0.007    # hole spacing

# Clamp hooks: small protrusions on lid skirt inner face
HOOK_W = 0.006
HOOK_H = 0.004
HOOK_D = 0.003

# Lid seated position: skirt engages the neck, top plate above neck lip
LID_SEATED_Z = JAR_BODY_H + NECK_LIP_H - LID_SKIRT_H


def _jar_body_solid() -> cq.Workplane:
    """Hollow cylindrical glass jar with wide mouth, shoulder, and neck lip."""
    # Main body cylinder
    outer = (
        cq.Workplane("XY")
        .circle(JAR_OD / 2.0)
        .extrude(JAR_BODY_H)
    )
    # Neck section (slightly narrower)
    neck = (
        cq.Workplane("XY")
        .workplane(offset=SHOULDER_Z)
        .circle(NECK_OD / 2.0)
        .extrude(NECK_H)
    )
    # Lip ring at top of neck
    lip = (
        cq.Workplane("XY")
        .workplane(offset=JAR_BODY_H)
        .circle(NECK_LIP_OD / 2.0)
        .extrude(NECK_LIP_H)
    )
    body = outer.union(neck).union(lip)

    # Hollow bore: wide mouth cavity from top through to just above base
    bore_depth = JAR_BODY_H + NECK_H + NECK_LIP_H - JAR_BASE_H
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=JAR_BASE_H)
        .circle(JAR_ID / 2.0)
        .extrude(bore_depth + 0.002)  # over-extrude to ensure open top
    )
    return body.cut(cavity)


def _lid_solid() -> cq.Workplane:
    """Rose-gold screw cap with hollow bore and clamp hooks on inner skirt."""
    # Outer cylinder
    outer = (
        cq.Workplane("XY")
        .circle(LID_OD / 2.0)
        .extrude(LID_TOTAL_H)
    )
    # Knurled grip ring at top (slight chamfer)
    outer = outer.edges(">Z").chamfer(0.001)

    # Hollow bore open at bottom, capped by top plate
    bore_id = LID_OD - 2.0 * LID_WALL
    bore = (
        cq.Workplane("XY")
        .circle(bore_id / 2.0)
        .extrude(LID_SKIRT_H)  # bore goes up to underside of top plate
    )
    lid = outer.cut(bore)

    # Add clamp hooks: 4 small protrusions on inner skirt wall
    # These are box-shaped hooks that engage the jar neck lip
    hook_radius = bore_id / 2.0 - HOOK_D * 0.5
    for i in range(4):
        angle = i * math.pi / 2.0
        hx = hook_radius * math.cos(angle)
        hy = hook_radius * math.sin(angle)
        hook = (
            cq.Workplane("XY")
            .workplane(offset=LID_SKIRT_H - HOOK_H)
            .center(hx, hy)
            .rect(HOOK_W, HOOK_D)
            .extrude(HOOK_H)
        )
        lid = lid.union(hook)

    return lid


def _shaker_disk() -> PerforatedPanelGeometry:
    """Perforated shaker insert disk (round approximation via square panel
    clipped to circle will look fine at this scale)."""
    return PerforatedPanelGeometry(
        (SHAKER_OD, SHAKER_OD),
        SHAKER_THICK,
        hole_diameter=SHAKER_HOLE_D,
        pitch=(SHAKER_PITCH, SHAKER_PITCH),
        frame=0.004,
        corner_radius=SHAKER_OD / 2.0 - 0.004,
        stagger=True,
        center=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="spice_jar_shaker")

    glass = model.material("clear_glass", rgba=(0.80, 0.85, 0.88, 0.30))
    rose_gold = model.material("rose_gold", rgba=(0.86, 0.58, 0.50, 1.0))
    steel = model.material("brushed_steel", rgba=(0.70, 0.70, 0.72, 1.0))

    # ---- body (root): glass jar with wide mouth ----
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_jar_body_solid(), "glass_jar"),
        material=glass,
        name="glass_jar",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(radius=JAR_OD / 2.0, length=JAR_BODY_H + NECK_H + NECK_LIP_H),
        mass=0.15,
        origin=Origin(xyz=(0.0, 0.0, (JAR_BODY_H + NECK_H + NECK_LIP_H) / 2.0)),
    )

    # ---- lid: rose-gold screw cap with clamp hooks ----
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_solid(), "screw_cap"),
        material=rose_gold,
        # CadQuery solid starts at z=0 (skirt bottom) in lid local frame.
        # No extra offset needed; the articulation places the part frame.
        origin=Origin(),
        name="screw_cap",
    )
    lid.inertial = Inertial.from_geometry(
        Cylinder(radius=LID_OD / 2.0, length=LID_TOTAL_H),
        mass=0.04,
        origin=Origin(xyz=(0.0, 0.0, LID_TOTAL_H / 2.0)),
    )

    # ---- shaker: perforated insert disk ----
    # Seated inside the lid bore, just below the top plate.
    # PerforatedPanelGeometry with center=True is centered at z=0.
    shaker = model.part("shaker")
    shaker_z_in_lid = LID_SKIRT_H - SHAKER_THICK  # just under the top plate
    shaker.visual(
        mesh_from_geometry(_shaker_disk(), "shaker_insert"),
        material=steel,
        origin=Origin(),
        name="shaker_insert",
    )
    shaker.inertial = Inertial.from_geometry(
        Cylinder(radius=SHAKER_OD / 2.0, length=SHAKER_THICK),
        mass=0.01,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # ---- articulation: body_to_lid (CONTINUOUS, screw thread) ----
    # The lid rotates around Z axis on the jar neck.
    # Place the lid part frame at the jar shoulder so the skirt (z=0..SKIRT_H
    # in lid local) covers the neck (z=SHOULDER_Z..JAR_BODY_H+NECK_LIP_H in body).
    model.articulation(
        "body_to_lid",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0),
    )

    # ---- articulation: lid_to_shaker (REVOLUTE, ±π/2) ----
    # The shaker rotates within the lid to align holes.
    # Place the shaker frame at shaker_z_in_lid in the lid local frame.
    model.articulation(
        "lid_to_shaker",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=shaker,
        origin=Origin(xyz=(0.0, 0.0, shaker_z_in_lid)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=-math.pi / 2.0,
            upper=math.pi / 2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    shaker = object_model.get_part("shaker")
    screw = object_model.get_articulation("body_to_lid")
    shaker_joint = object_model.get_articulation("lid_to_shaker")

    # ---- lid skirt intentionally overlaps the jar neck (screw thread fit) ----
    ctx.allow_overlap(
        lid,
        body,
        elem_a="screw_cap",
        elem_b="glass_jar",
        reason="Rose-gold screw cap skirt threads onto the jar neck (intentional screw fit).",
    )

    # ---- shaker disk is intentionally inside the lid bore ----
    ctx.allow_overlap(
        shaker,
        lid,
        elem_a="shaker_insert",
        elem_b="screw_cap",
        reason="Perforated shaker insert is seated inside the lid bore (captured disk).",
    )

    # ---- jar identity: body is roughly cylindrical, wider than tall ----
    body_aabb = ctx.part_world_aabb(body)
    body_ext = (
        body_aabb[1][0] - body_aabb[0][0],
        body_aabb[1][1] - body_aabb[0][1],
        body_aabb[1][2] - body_aabb[0][2],
    )
    ctx.check(
        "jar body is roughly circular in section",
        abs(body_ext[0] - body_ext[1]) < 0.005,
        details=f"body XY extents: dx={body_ext[0]:.4f}, dy={body_ext[1]:.4f}",
    )
    ctx.check(
        "jar is wider than it is tall (jar proportions)",
        max(body_ext[0], body_ext[1]) > body_ext[2] * 0.5,
        details=f"body extents={body_ext}",
    )

    # ---- wide-mouth hollow opening: jar inner bore is at least 70% of outer ----
    mouth_ratio = JAR_ID / JAR_OD
    ctx.check(
        "jar has wide mouth (inner > 70% of outer diameter)",
        mouth_ratio > 0.70,
        details=f"JAR_ID/JAR_OD={mouth_ratio:.2f}",
    )

    # ---- lid sits on the jar neck at rest ----
    lid_pos = ctx.part_world_position(lid)
    ctx.check(
        "lid is seated at the jar top",
        lid_pos is not None and lid_pos[2] > SHOULDER_Z - 0.005,
        details=f"lid origin z={lid_pos[2] if lid_pos else None}",
    )

    # ---- lid XY footprint overlaps jar neck (screw engagement) ----
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        min_overlap=0.030,
        name="lid footprint overlaps jar neck",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="z",
        min_overlap=0.005,
        name="lid skirt engages jar neck in Z",
    )

    # ---- screw joint is CONTINUOUS (no angular limits) ----
    ctx.check(
        "body_to_lid is a continuous joint",
        screw.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={screw.articulation_type}",
    )

    # ---- shaker joint is REVOLUTE with bounded limits ----
    shaker_limits = shaker_joint.motion_limits
    ctx.check(
        "lid_to_shaker is revolute with finite limits",
        shaker_joint.articulation_type == ArticulationType.REVOLUTE
        and shaker_limits is not None
        and shaker_limits.lower is not None
        and shaker_limits.upper is not None
        and shaker_limits.upper - shaker_limits.lower > 0.5,
        details=f"type={shaker_joint.articulation_type}, "
        f"lower={getattr(shaker_limits, 'lower', None)}, "
        f"upper={getattr(shaker_limits, 'upper', None)}",
    )

    # ---- shaker insert is captured inside the lid ----
    ctx.expect_within(
        shaker,
        lid,
        axes="xy",
        margin=0.002,
        name="shaker insert stays within lid bore (XY)",
    )

    # ---- shaker rotates when posed ----
    rest_pos = ctx.part_world_position(shaker)
    with ctx.pose({shaker_joint: math.pi / 4.0}):
        rotated_pos = ctx.part_world_position(shaker)
    ctx.check(
        "shaker joint allows rotation (part moves with pose)",
        rest_pos is not None and rotated_pos is not None,
        details=f"rest={rest_pos}, rotated={rotated_pos}",
    )

    # ---- lid rotates on screw joint (continuous pose) ----
    with ctx.pose({screw: math.pi}):
        lid_rotated = ctx.part_world_position(lid)
    ctx.check(
        "lid stays on jar during rotation",
        lid_rotated is not None and lid_rotated[2] > SHOULDER_Z - 0.005,
        details=f"rotated lid z={lid_rotated[2] if lid_rotated else None}",
    )

    # ---- materials: distinct glass and rose-gold ----
    jar_mat = body.get_visual("glass_jar").material
    cap_mat = lid.get_visual("screw_cap").material
    ctx.check(
        "jar is clear glass, lid is rose-gold (distinct materials)",
        jar_mat is not None
        and cap_mat is not None
        and getattr(jar_mat, "name", None) == "clear_glass"
        and getattr(cap_mat, "name", None) == "rose_gold",
        details=f"jar={getattr(jar_mat, 'name', None)}, cap={getattr(cap_mat, 'name', None)}",
    )

    # ---- shaker has perforated geometry (named visual exists) ----
    shaker_vis = shaker.get_visual("shaker_insert")
    ctx.check(
        "shaker insert visual exists with perforated geometry",
        shaker_vis is not None,
        details="shaker_insert visual not found",
    )

    return ctx.report()


object_model = build_object_model()
