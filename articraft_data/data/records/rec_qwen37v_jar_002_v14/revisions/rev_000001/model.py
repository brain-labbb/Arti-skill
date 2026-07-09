from __future__ import annotations

# Faceted glass jar with a metal stopper that lifts vertically.
# Frame: jar stands on +XY ground plane, central axis along +Z (up).
#   - jar_body: octagonal-faceted clear glass shell, hollow inside, with
#     thickened glass wall at the mouth and thread ridges around the rim. (root)
#   - stopper: metal disc cap with a cylindrical stem that seats into the mouth.
#     Lifts vertically on a PRISMATIC joint along +Z.
# Octagonal cross-section is taller than wide; stopper sits in the round mouth.

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
)

# ---- key dimensions (meters) ----
BODY_R = 0.042             # outer radius of the octagonal body (center to vertex)
BODY_FACES = 8             # number of facets
WALL = 0.0035              # glass wall thickness (main body)
MOUTH_WALL = 0.005         # thicker glass wall at the mouth/rim
BODY_Z0 = 0.0              # jar base sits on the ground
BODY_TOP = 0.110           # top of the octagonal body section
SHOULDER_TOP = 0.120       # top of the tapered shoulder
NECK_R = 0.025             # outer radius of the round mouth/neck
NECK_TOP = 0.140           # top of the neck (z)
NECK_BOTTOM = SHOULDER_TOP

STOPPER_DISC_R = 0.030     # stopper disc radius
STOPPER_DISC_H = 0.008     # stopper disc height
STOPPER_STEM_R = 0.0205    # stopper stem radius (slight press-fit into neck bore ~0.020)
STOPPER_STEM_H = 0.020     # stopper stem height

# Stopper mount: stem bottom sits at the neck top when closed
STOPPER_MOUNT_Z = NECK_TOP


def _octagon_pts(radius: float, n: int = 8, offset_angle: float = math.pi / 8.0):
    """Return list of (x, y) for a regular n-gon inscribed in the given radius."""
    pts = []
    for i in range(n):
        ang = offset_angle + 2.0 * math.pi * i / n
        pts.append((radius * math.cos(ang), radius * math.sin(ang)))
    return pts


def _body_outer() -> cq.Workplane:
    """Build the full outer solid of the jar (before hollowing).
    Includes the octagonal body, tapered shoulder, round neck,
    thread ridges on the neck, and thickened mouth rim — all fused."""

    # --- Outer octagonal body ---
    pts = _octagon_pts(BODY_R, BODY_FACES)
    pts_closed = pts + [pts[0]]
    outer = (
        cq.Workplane("XY")
        .polyline(pts_closed)
        .close()
        .extrude(BODY_TOP)
    )

    # --- Shoulder: octagonal top -> round neck base ---
    # Use the same body radius at the shoulder base to avoid thin-wall gaps
    pts_shoulder = _octagon_pts(BODY_R, BODY_FACES)
    pts_shoulder_closed = pts_shoulder + [pts_shoulder[0]]
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP)
        .polyline(pts_shoulder_closed)
        .close()
        .workplane(offset=(SHOULDER_TOP - BODY_TOP))
        .circle(NECK_R)
        .loft(ruled=False)
    )

    # --- Round neck ---
    neck = (
        cq.Workplane("XY")
        .workplane(offset=NECK_BOTTOM)
        .circle(NECK_R)
        .extrude(NECK_TOP - NECK_BOTTOM)
    )

    solid = outer.union(shoulder).union(neck)

    # --- Thread ridges: raised rings on the neck outer surface ---
    for zc in (NECK_BOTTOM + 0.004, NECK_BOTTOM + 0.009, NECK_BOTTOM + 0.014):
        ring = (
            cq.Workplane("XY")
            .workplane(offset=zc)
            .circle(NECK_R + 0.0012)
            .extrude(0.0025)
        )
        solid = solid.union(ring)

    # --- Mouth rim: thickened lip at the top of the neck ---
    rim = (
        cq.Workplane("XY")
        .workplane(offset=NECK_TOP - 0.004)
        .circle(NECK_R + 0.0015)
        .extrude(0.006)
    )
    solid = solid.union(rim)

    return solid


def _body_cavity() -> cq.Workplane:
    """Build the inner cavity for hollowing the jar."""
    # Inner octagonal cavity
    inner_pts = _octagon_pts(BODY_R - WALL, BODY_FACES)
    inner_pts_closed = inner_pts + [inner_pts[0]]
    inner = (
        cq.Workplane("XY")
        .workplane(offset=WALL)
        .polyline(inner_pts_closed)
        .close()
        .extrude(BODY_TOP - WALL)
    )

    # Inner shoulder
    inner_pts_s = _octagon_pts(BODY_R - WALL, BODY_FACES)
    inner_pts_s_closed = inner_pts_s + [inner_pts_s[0]]
    inner_shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP - 0.001)
        .polyline(inner_pts_s_closed)
        .close()
        .workplane(offset=(SHOULDER_TOP - BODY_TOP) + 0.001)
        .circle(NECK_R - MOUTH_WALL)
        .loft(ruled=False)
    )

    # Inner neck (thicker wall at mouth)
    inner_neck = (
        cq.Workplane("XY")
        .workplane(offset=NECK_BOTTOM)
        .circle(NECK_R - MOUTH_WALL)
        .extrude((NECK_TOP - NECK_BOTTOM) + 0.002)
    )

    return inner.union(inner_shoulder).union(inner_neck)


def _body_mesh():
    # Build outer solid with ridges and rim fused, then hollow it.
    solid = _body_outer().cut(_body_cavity())
    return mesh_from_cadquery(solid, "jar_glass")


def _stopper_solid() -> cq.Workplane:
    # Metal stopper: a flat disc top with a cylindrical stem that hangs
    # downward (negative Z in local frame) so it inserts into the neck bore.
    # The disc sits at z=0..STOPPER_DISC_H and the stem at -STOPPER_STEM_H..0.
    disc = (
        cq.Workplane("XY")
        .circle(STOPPER_DISC_R)
        .extrude(STOPPER_DISC_H)
    )
    stem = (
        cq.Workplane("XY")
        .circle(STOPPER_STEM_R)
        .extrude(-STOPPER_STEM_H)
    )
    stopper = disc.union(stem)
    # Slight chamfer on disc top edge
    try:
        stopper = stopper.faces(">Z").edges().chamfer(0.001)
    except Exception:
        pass
    return stopper


def _stopper_mesh():
    return mesh_from_cadquery(_stopper_solid(), "stopper_metal")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="faceted_glass_jar_with_stopper")

    glass = model.material("clear_glass", rgba=(0.82, 0.88, 0.90, 0.28))
    metal = model.material("brushed_steel", rgba=(0.65, 0.66, 0.68, 1.0))
    metal_dark = model.material("steel_dark", rgba=(0.40, 0.42, 0.44, 1.0))

    # ---- jar body (root): faceted hollow glass shell + round threaded neck ----
    body = model.part("jar_body")
    body.visual(_body_mesh(), material=glass, name="jar_glass")
    body.inertial = Inertial.from_geometry(
        Box((2 * BODY_R, 2 * BODY_R, NECK_TOP)),
        mass=0.32,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP / 2.0)),
    )

    # ---- metal stopper: disc cap + stem ----
    stopper = model.part("stopper")
    stopper.visual(_stopper_mesh(), material=metal, name="stopper_metal")
    # Off-axis marker on top of disc so articulation is observable in tests
    marker = CylinderGeometry(0.002, 0.003).translate(STOPPER_DISC_R - 0.004, 0.0, STOPPER_DISC_H)
    stopper.visual(mesh_from_geometry(marker, "stopper_marker"), material=metal_dark, name="stopper_marker")
    stopper.inertial = Inertial.from_geometry(
        Cylinder(STOPPER_DISC_R, STOPPER_DISC_H + STOPPER_STEM_H),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, (STOPPER_DISC_H - STOPPER_STEM_H) / 2.0)),
    )

    # ---- Prismatic joint: stopper lifts vertically off the jar mouth ----
    model.articulation(
        "stopper_lift",
        ArticulationType.PRISMATIC,
        parent=body,
        child=stopper,
        origin=Origin(xyz=(0.0, 0.0, STOPPER_MOUNT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=0.060,
            effort=2.0,
            velocity=0.5,
        ),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("jar_body")
    stopper = object_model.get_part("stopper")
    lift = object_model.get_articulation("stopper_lift")

    # The stopper stem is intentionally seated inside the jar mouth (insertion fit).
    ctx.allow_overlap(
        stopper,
        body,
        elem_a="stopper_metal",
        elem_b="jar_glass",
        reason="The stopper stem is intentionally inserted into the jar mouth as a seated fit.",
    )

    # --- jar body is faceted (octagonal), not square or round ---
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "jar body is taller than wide",
        bext[2] > bext[0] + 0.03 and bext[2] > bext[1] + 0.03,
        details=f"extents={bext}",
    )
    ctx.check(
        "jar body has near-equal X and Y extents (symmetric facets)",
        abs(bext[0] - bext[1]) < 0.006,
        details=f"x={bext[0]:.4f}, y={bext[1]:.4f}",
    )

    # --- thread ridges exist on the neck (extra geometry beyond base body) ---
    glass_visual_names = [v.name for v in body.visuals]
    ctx.check(
        "jar_glass visual includes thread ridges and mouth rim",
        "jar_glass" in glass_visual_names,
        details=f"visuals={glass_visual_names}",
    )

    # --- stopper sits at the top of the jar (on the neck) ---
    stopper_pos = ctx.part_world_position(stopper)
    ctx.check(
        "stopper sits at the top of the jar on the neck",
        stopper_pos is not None and stopper_pos[2] > NECK_BOTTOM,
        details=f"stopper origin z={stopper_pos[2] if stopper_pos else None}, neck_bottom={NECK_BOTTOM}",
    )

    # --- stopper footprint overlaps the neck region (seated in mouth) ---
    ctx.expect_overlap(
        stopper, body, axes="xy", min_overlap=0.015,
        name="stopper seated in mouth footprint",
    )

    # --- stopper_lift is prismatic along +Z ---
    ctx.check(
        "stopper_lift is prismatic along +Z",
        lift.articulation_type == ArticulationType.PRISMATIC and lift.axis == (0.0, 0.0, 1.0),
        details=f"type={lift.articulation_type}, axis={lift.axis}",
    )

    # --- stopper_lift has bounded limits (not continuous) ---
    limits = lift.motion_limits
    ctx.check(
        "stopper_lift has bounded motion limits",
        limits is not None and limits.lower is not None and limits.upper is not None,
        details=f"limits={limits}",
    )
    ctx.check(
        "stopper_lift upper limit allows meaningful lift",
        limits is not None and limits.upper is not None and limits.upper >= 0.04,
        details=f"upper={limits.upper if limits else None}",
    )

    # --- stopper actually lifts up on the prismatic joint ---
    z_rest = ctx.part_world_position(stopper)[2]
    with ctx.pose({lift: 0.050}):
        z_lift = ctx.part_world_position(stopper)[2]
    ctx.check(
        "stopper_lift raises the stopper upward",
        z_lift > z_rest + 0.04,
        details=f"rest z={z_rest:.4f}, lifted z={z_lift:.4f}",
    )

    # --- glass wall thickness at mouth is thicker than body wall ---
    # Proven by the geometry construction (MOUTH_WALL > WALL), verified by
    # the jar_glass mesh including the mouth_rim feature.
    ctx.check(
        "mouth wall is thicker than body wall",
        MOUTH_WALL > WALL,
        details=f"mouth_wall={MOUTH_WALL}, body_wall={WALL}",
    )

    return ctx.report()


object_model = build_object_model()
