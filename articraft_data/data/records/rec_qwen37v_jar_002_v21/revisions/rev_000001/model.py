from __future__ import annotations

# Squat cosmetic cream jar with thick screw lid and clamp bail.
# Frame: jar stands on +XY ground plane, central axis along +Z (up).
#   - jar_body: squat round clear glass shell, hollow inside with wide mouth
#     opening at the top. Two small pivot bosses on the sides for the bail. (root)
#   - lid: thick round knurled cap that screws onto the wide mouth.
#   - bail: U-shaped wire clamp bail that pivots on two side revolute hinges.
#     Swings from hanging down (open) up and over the lid (clamped).
# Articulations:
#   - lid_rotate (CONTINUOUS, body->lid): lid spins about +Z to screw on/off
#   - bail_hinge (REVOLUTE, body->bail): bail pivots about Y axis through
#     both side pivot bosses, limits 0 to 3.0 rad

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
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
BODY_R = 0.035            # outer radius of the round jar body (70mm diameter)
BODY_H = 0.042            # height of the jar body (squat)
WALL = 0.003              # glass wall thickness
BASE_THICK = 0.004        # thick base floor
MOUTH_R = 0.030           # wide mouth inner radius (60mm diameter opening)
LIP_H = 0.005             # short lip/rim above body top
LIP_R = 0.031             # lip outer radius (slightly wider than mouth)

LID_R = 0.033             # lid outer radius (wraps over the lip)
LID_H = 0.016             # thick lid height
LID_TOP_THICK = 0.004     # lid top plate thickness
SCALLOP_N = 24            # knurling scallops on lid edge

# Bail dimensions
PIVOT_Z = BODY_H + LIP_H - 0.002  # pivot height near top of jar
PIVOT_HALF_W = BODY_R + 0.004     # half-width between pivot points (on jar sides)
BAIL_ARM_LEN = 0.028              # bail arm length from pivot down
BAIL_WIRE_R = 0.0015              # wire radius
BOSS_R = 0.004                    # pivot boss radius
BOSS_H = 0.005                    # pivot boss height (protrusion from jar side)


def _body_solid() -> cq.Workplane:
    """Squat round glass jar with wide mouth, hollow interior, and pivot bosses."""
    # Main cylindrical body
    body = (
        cq.Workplane("XY")
        .circle(BODY_R)
        .extrude(BODY_H)
    )

    # Lip/rim at the top (slightly wider ring for the mouth)
    lip = (
        cq.Workplane("XY")
        .workplane(offset=BODY_H)
        .circle(LIP_R)
        .circle(MOUTH_R)
        .extrude(LIP_H)
    )

    # Pivot bosses on sides (small cylinders protruding from jar wall)
    boss_y_pos = (
        cq.Workplane("XZ")
        .workplane(offset=BODY_R)
        .center(0.0, PIVOT_Z)
        .circle(BOSS_R)
        .extrude(BOSS_H)
    )
    boss_y_neg = (
        cq.Workplane("XZ")
        .workplane(offset=-BODY_R)
        .center(0.0, PIVOT_Z)
        .circle(BOSS_R)
        .extrude(-BOSS_H)
    )

    solid = body.union(lip).union(boss_y_pos).union(boss_y_neg)

    # Hollow cavity: wide mouth opening down into the jar
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=BASE_THICK)
        .circle(MOUTH_R)
        .extrude(BODY_H + LIP_H - BASE_THICK + 0.001)
    )

    return solid.cut(cavity)


def _body_mesh():
    return mesh_from_cadquery(_body_solid(), "jar_glass")


def _lid_solid() -> cq.Workplane:
    """Thick round lid with knurled/scalloped edge and hollow bore."""
    # Outer cylinder
    outer = (
        cq.Workplane("XY")
        .circle(LID_R)
        .extrude(LID_H)
    )

    # Hollow bore so it fits over the lip (open at bottom)
    bore = (
        cq.Workplane("XY")
        .circle(LIP_R - 0.0005)
        .extrude(LID_H - LID_TOP_THICK)
    )
    lid = outer.cut(bore)

    # Knurling scallops around the rim
    for k in range(SCALLOP_N):
        ang = 2.0 * math.pi * k / SCALLOP_N
        fx = LID_R * math.cos(ang)
        fy = LID_R * math.sin(ang)
        flute = (
            cq.Workplane("XY")
            .center(fx, fy)
            .circle(0.0022)
            .extrude(LID_H)
        )
        lid = lid.cut(flute)

    # Slight chamfer on top edge
    try:
        lid = lid.faces(">Z").edges().chamfer(0.001)
    except Exception:
        pass

    return lid


def _lid_mesh():
    return mesh_from_cadquery(_lid_solid(), "lid_brass")


def _bail_mesh():
    """U-shaped clamp bail wire."""
    # The bail is in the bail part frame (origin at the pivot axis center).
    # At q=0, the bail hangs down from the pivots.
    # Points define the U-shape: left pivot -> left arm bottom -> cross bar -> right arm bottom -> right pivot
    half_w = PIVOT_HALF_W
    arm = BAIL_ARM_LEN

    points = [
        (0.0, -half_w, 0.0),          # left pivot
        (0.0, -half_w, -arm * 0.3),   # left arm upper
        (0.0, -half_w, -arm),         # left arm bottom
        (0.0, -half_w * 0.3, -arm),   # cross bar left
        (0.0, 0.0, -arm),             # cross bar center
        (0.0, half_w * 0.3, -arm),    # cross bar right
        (0.0, half_w, -arm),          # right arm bottom
        (0.0, half_w, -arm * 0.3),    # right arm upper
        (0.0, half_w, 0.0),           # right pivot
    ]

    bail_geom = tube_from_spline_points(
        points,
        radius=BAIL_WIRE_R,
        samples_per_segment=14,
        radial_segments=12,
        cap_ends=True,
    )
    return mesh_from_geometry(bail_geom, "bail_wire")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="squat_cosmetic_cream_jar")

    glass = model.material("clear_glass", rgba=(0.80, 0.85, 0.88, 0.30))
    brass = model.material("brass", rgba=(0.72, 0.55, 0.20, 1.0))
    steel = model.material("steel_wire", rgba=(0.65, 0.65, 0.68, 1.0))
    brass_dark = model.material("brass_dark", rgba=(0.52, 0.38, 0.12, 1.0))

    # ---- jar body (root): squat round hollow glass jar with wide mouth ----
    body = model.part("jar_body")
    body.visual(_body_mesh(), material=glass, name="jar_glass")
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, BODY_H + LIP_H),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, (BODY_H + LIP_H) / 2.0)),
    )

    # ---- thick brass screw lid ----
    lid = model.part("lid")
    lid.visual(_lid_mesh(), material=brass, name="lid_brass")
    # Off-axis marker to observe rotation
    marker = CylinderGeometry(0.002, 0.003).translate(LID_R - 0.004, 0.0, LID_H)
    lid.visual(
        mesh_from_geometry(marker, "lid_marker"),
        material=brass_dark,
        name="lid_marker",
    )
    lid.inertial = Inertial.from_geometry(
        Cylinder(LID_R, LID_H),
        mass=0.035,
        origin=Origin(xyz=(0.0, 0.0, LID_H / 2.0)),
    )

    # ---- bail wire (U-shaped clamp) ----
    bail = model.part("bail")
    bail.visual(_bail_mesh(), material=steel, name="bail_wire")
    bail.inertial = Inertial.from_geometry(
        Box((0.004, 2 * PIVOT_HALF_W, BAIL_ARM_LEN)),
        mass=0.012,
        origin=Origin(xyz=(0.0, 0.0, -BAIL_ARM_LEN / 2.0)),
    )

    # ---- lid_rotate: CONTINUOUS, body -> lid, axis +Z ----
    # Lid sits on the lip at BODY_H. Local lid origin is at bottom of lid.
    lid_mount_z = BODY_H + LIP_H - (LID_H - LID_TOP_THICK)
    model.articulation(
        "lid_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, lid_mount_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )

    # ---- bail_hinge: REVOLUTE, body -> bail, axis along Y through both pivots ----
    # The bail pivots at (0, 0, PIVOT_Z), axis (0,1,0).
    # At q=0, bail hangs down. Positive q swings bail up and over the lid.
    model.articulation(
        "bail_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=bail,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=3.0,
            effort=2.0,
            velocity=2.0,
        ),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("jar_body")
    lid = object_model.get_part("lid")
    bail = object_model.get_part("bail")
    lid_rotate = object_model.get_articulation("lid_rotate")
    bail_hinge = object_model.get_articulation("bail_hinge")

    # --- Lid is intentionally seated over the lip/mouth region ---
    ctx.allow_overlap(
        lid,
        body,
        elem_a="lid_brass",
        elem_b="jar_glass",
        reason="The thick brass lid skirt is intentionally screwed down over the jar lip.",
    )

    # --- Bail pivot bosses are part of the jar body; bail wire ends near them ---
    ctx.allow_overlap(
        bail,
        body,
        elem_a="bail_wire",
        elem_b="jar_glass",
        reason="The bail wire ends are intentionally represented at the pivot boss locations on the jar body.",
    )

    # === Jar geometry checks ===

    # Jar is squat: wider than tall
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "jar body is round (nearly equal X and Y extents)",
        abs(bext[0] - bext[1]) < 0.012,
        details=f"x={bext[0]:.4f}, y={bext[1]:.4f}",
    )
    ctx.check(
        "jar is squat (wider than tall)",
        bext[0] > bext[2] and bext[1] > bext[2],
        details=f"extents=({bext[0]:.4f}, {bext[1]:.4f}, {bext[2]:.4f})",
    )

    # === Lid checks ===

    # Lid is round and sits on top of the jar
    lext = _ext(ctx.part_world_aabb(lid))
    ctx.check(
        "lid is round (nearly equal X and Y extents)",
        abs(lext[0] - lext[1]) < 0.004,
        details=f"lid x={lext[0]:.4f}, y={lext[1]:.4f}",
    )
    lid_aabb = ctx.part_world_aabb(lid)
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "lid sits on top of the jar body (lid top above body top)",
        lid_aabb is not None
        and body_aabb is not None
        and lid_aabb[1][2] > body_aabb[1][2] - 0.002,
        details=f"lid top z={lid_aabb[1][2] if lid_aabb else None}, "
                f"body top z={body_aabb[1][2] if body_aabb else None}",
    )
    # Lid overlaps the jar mouth region (seated on lip)
    ctx.expect_overlap(
        lid, body, axes="xy", min_overlap=0.02,
        name="lid seated over mouth footprint",
    )

    # === Lid rotation check ===
    m0 = ctx.part_element_world_aabb(lid, elem="lid_marker")
    m0c = ((m0[0][0] + m0[1][0]) / 2.0, (m0[0][1] + m0[1][1]) / 2.0)
    with ctx.pose({lid_rotate: math.pi / 2.0}):
        m1 = ctx.part_element_world_aabb(lid, elem="lid_marker")
        m1c = ((m1[0][0] + m1[1][0]) / 2.0, (m1[0][1] + m1[1][1]) / 2.0)
    marker_shift = math.hypot(m1c[0] - m0c[0], m1c[1] - m0c[1])
    ctx.check(
        "lid_rotate spins the lid (marker moves)",
        marker_shift > 0.008,
        details=f"marker moved {marker_shift:.4f} m on a quarter turn",
    )

    # === Bail hinge checks ===

    # Bail hinge is revolute about Y axis
    ctx.check(
        "bail_hinge is revolute about Y axis",
        bail_hinge.axis == (0.0, 1.0, 0.0)
        and bail_hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"axis={bail_hinge.axis}, type={bail_hinge.articulation_type}",
    )

    # Bail hinge has proper limits
    bail_limits = bail_hinge.motion_limits
    ctx.check(
        "bail_hinge has swing limits from 0 to ~3.0 rad",
        bail_limits is not None
        and bail_limits.lower == 0.0
        and bail_limits.upper >= 2.5,
        details=f"limits=({bail_limits.lower}, {bail_limits.upper})" if bail_limits else "no limits",
    )

    # Bail swings: at q=0 bail hangs down, at q~pi bail is up over the lid
    bail_rest_aabb = ctx.part_world_aabb(bail)
    with ctx.pose({bail_hinge: math.pi}):
        bail_swung_aabb = ctx.part_world_aabb(bail)
    rest_center_z = (bail_rest_aabb[0][2] + bail_rest_aabb[1][2]) / 2.0 if bail_rest_aabb else 0
    swung_center_z = (bail_swung_aabb[0][2] + bail_swung_aabb[1][2]) / 2.0 if bail_swung_aabb else 0
    ctx.check(
        "bail swings upward when hinge is at pi",
        bail_rest_aabb is not None
        and bail_swung_aabb is not None
        and swung_center_z > rest_center_z + 0.01,
        details=f"rest center z={rest_center_z:.4f}, swung center z={swung_center_z:.4f}",
    )

    # Bail exists and has reasonable extent (wider than tall at rest, arms hang down)
    bail_ext = _ext(ctx.part_world_aabb(bail))
    ctx.check(
        "bail spans between the two pivot points (Y extent)",
        bail_ext[1] > 0.04,
        details=f"bail extents=({bail_ext[0]:.4f}, {bail_ext[1]:.4f}, {bail_ext[2]:.4f})",
    )

    # === Wide mouth check ===
    # The jar body has a wide mouth - the cavity opening should be visible.
    # The mouth radius is MOUTH_R = 0.030, which is >80% of body radius.
    ctx.check(
        "wide mouth opening (mouth radius > 80% of body radius)",
        MOUTH_R > 0.8 * BODY_R,
        details=f"mouth_r={MOUTH_R}, body_r={BODY_R}, ratio={MOUTH_R/BODY_R:.2f}",
    )

    return ctx.report()


object_model = build_object_model()
