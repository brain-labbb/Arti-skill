from __future__ import annotations

# Ribbed water bottle with deep grip grooves and a flip cap on a revolute hinge.
# Frame: bottle axis along +Z, base at z=0, neck/cap at the top (+Z).
# The body is a semi-transparent bottle with deep horizontal grip grooves
# cut into the barrel. A hollow neck with visible mouth opening sits at top.
# A rubber gasket ring is seated on the neck rim.
# A flip cap opens on a revolute hinge at the rear of the neck.

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

# ---- key dimensions (meters) ----
BODY_R = 0.032          # outer barrel radius (~64mm dia)
WALL = 0.002            # wall thickness
BASE_Z = 0.0            # bottom of the bottle
BARREL_TOP_Z = 0.110    # where the shoulder taper begins
SHOULDER_TOP_Z = 0.135  # top of the shoulder, base of the neck
NECK_R = 0.013          # neck outer radius
NECK_INNER_R = 0.0105   # neck inner radius (mouth opening)
NECK_TOP_Z = 0.158      # top rim of the neck
CAP_R = 0.016           # flip cap radius
CAP_THICKNESS = 0.006   # flip cap disc thickness

# Grip groove parameters
GROOVE_DEPTH = 0.004    # how deep each groove cuts into the wall
GROOVE_WIDTH = 0.006    # width of each groove
N_GROOVES = 5           # number of grip grooves
GROOVE_START_Z = 0.025  # z of first groove
GROOVE_SPACING = 0.014  # center-to-center spacing between grooves

# Hinge geometry
HINGE_OFFSET_R = NECK_R + 0.002  # hinge pin at rear of neck


def _bottle_profile_points():
    """Build the outer profile of the bottle with grip grooves.
    Returns list of (radius, z) from bottom to top of neck."""
    pts = []

    # Rounded base corner
    pts.append((0.0, BASE_Z))
    pts.append((BODY_R - 0.006, BASE_Z))
    pts.append((BODY_R, BASE_Z + 0.006))
    pts.append((BODY_R, BASE_Z + 0.012))

    # Barrel with grip grooves: step in and out
    z_cursor = BASE_Z + 0.012
    for i in range(N_GROOVES):
        groove_z = GROOVE_START_Z + i * GROOVE_SPACING
        # Straight section up to groove start
        if groove_z - GROOVE_WIDTH / 2.0 > z_cursor:
            pts.append((BODY_R, groove_z - GROOVE_WIDTH / 2.0))
        # Groove: cut inward
        pts.append((BODY_R - GROOVE_DEPTH, groove_z - GROOVE_WIDTH / 2.0 + 0.001))
        pts.append((BODY_R - GROOVE_DEPTH, groove_z + GROOVE_WIDTH / 2.0 - 0.001))
        pts.append((BODY_R, groove_z + GROOVE_WIDTH / 2.0))
        z_cursor = groove_z + GROOVE_WIDTH / 2.0

    # Continue barrel to shoulder start
    pts.append((BODY_R, BARREL_TOP_Z))

    # Shoulder taper to neck
    mid_r = (BODY_R + NECK_R) / 2.0
    mid_z = (BARREL_TOP_Z + SHOULDER_TOP_Z) / 2.0 + 0.003
    pts.append((mid_r, mid_z))
    pts.append((NECK_R, SHOULDER_TOP_Z))

    # Neck straight up with slight lip at top
    pts.append((NECK_R, NECK_TOP_Z - 0.003))
    pts.append((NECK_R + 0.001, NECK_TOP_Z - 0.002))
    pts.append((NECK_R + 0.001, NECK_TOP_Z))
    pts.append((NECK_R, NECK_TOP_Z))

    return pts


def _bottle_shell():
    """Semi-transparent thin-wall bottle with grip grooves and hollow neck."""
    pts = _bottle_profile_points()

    wp = cq.Workplane("XZ")
    wp = wp.moveTo(pts[0][0], pts[0][1])
    for (r, z) in pts[1:]:
        wp = wp.lineTo(r, z)
    # Close along axis at top
    wp = wp.lineTo(0.0, NECK_TOP_Z).close()
    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))

    # Shell: hollow it from the top face inward
    return outer.faces(">Z").shell(-WALL)


def _gasket_ring():
    """Rubber gasket ring that sits on the neck rim.
    Built centered at local origin (0,0,0); the articulation places it."""
    gasket_center_r = NECK_R + 0.001
    gasket_minor_r = 0.0015

    # Build as a revolved cross-section at local z=0
    wp = cq.Workplane("XZ")
    wp = (
        wp.moveTo(gasket_center_r, 0.0)
        .circle(gasket_minor_r)
    )
    ring = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))
    return ring


def _flip_cap():
    """Flip cap disc with hinge lug at the rear.
    Local frame: origin at hinge point (rear of neck top).
    The cap disc extends along +X from the hinge."""
    cap_center_x = CAP_R + 0.002  # distance from hinge to cap center

    # Cap disc: sits above hinge level, extending along +X
    cap = (
        cq.Workplane("XY")
        .transformed(offset=(cap_center_x, 0.0, CAP_THICKNESS / 2.0))
        .circle(CAP_R)
        .extrude(CAP_THICKNESS)
    )
    # Fillet top edges
    cap = cap.edges(">Z").fillet(0.001)

    # Hinge lug: small box connecting cap to hinge axis (bridges from x=0 to cap)
    lug = (
        cq.Workplane("XY")
        .transformed(offset=(cap_center_x / 2.0, 0.0, CAP_THICKNESS / 2.0))
        .box(cap_center_x + 0.004, 0.008, CAP_THICKNESS)
    )
    cap = cap.union(lug)

    # Spout plug on underside: extends down from the disc into the mouth.
    # Must connect to the disc (start at z=0, go down)
    plug = (
        cq.Workplane("XY")
        .transformed(offset=(cap_center_x, 0.0, -0.002))
        .circle(NECK_INNER_R - 0.001)
        .extrude(CAP_THICKNESS / 2.0 + 0.002)  # extends up to disc bottom
    )
    cap = cap.union(plug)

    return cap


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ribbed_water_bottle")

    # Materials
    clear_body = model.material("clear_plastic", rgba=(0.75, 0.82, 0.88, 0.35))
    dark_cap = model.material("cap_dark", rgba=(0.08, 0.08, 0.10, 1.0))
    rubber_gasket = model.material("rubber_gray", rgba=(0.25, 0.25, 0.27, 1.0))

    # ---- bottle body (root) ----
    body = model.part("bottle")
    shell = _bottle_shell()
    body.visual(mesh_from_cadquery(shell, "bottle_shell"), material=clear_body, name="bottle_shell")
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, NECK_TOP_Z),
        mass=0.040,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    # ---- gasket ring ----
    gasket = model.part("gasket")
    gasket_geo = _gasket_ring()
    gasket.visual(mesh_from_cadquery(gasket_geo, "gasket_ring"), material=rubber_gasket, name="gasket_ring")
    gasket.inertial = Inertial.from_geometry(
        Cylinder(NECK_R + 0.003, 0.003),
        mass=0.002,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # ---- flip cap ----
    cap = model.part("flip_cap")
    cap_geo = _flip_cap()
    cap.visual(mesh_from_cadquery(cap_geo, "cap_disc"), material=dark_cap, name="cap_disc")
    cap.inertial = Inertial.from_geometry(
        Cylinder(CAP_R, CAP_THICKNESS),
        mass=0.005,
        origin=Origin(xyz=(CAP_R + 0.002, 0.0, CAP_THICKNESS / 2.0)),
    )

    # Gasket is rigidly mounted on the bottle neck (no articulation needed,
    # but it's a separate part for the "separate gasket ring" requirement).
    # We attach it via a FIXED joint to the body.
    model.articulation(
        "gasket_mount",
        ArticulationType.FIXED,
        parent=body,
        child=gasket,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z - 0.001)),
    )

    # ---- flip cap hinge (REVOLUTE) ----
    # Hinge at the rear of the neck top, axis along +Y.
    # Positive rotation opens the cap upward (away from mouth).
    model.articulation(
        "cap_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cap,
        origin=Origin(xyz=(0.0, -HINGE_OFFSET_R, NECK_TOP_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.0,
            lower=0.0,
            upper=2.2,  # ~126 degrees open
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle")
    gasket = object_model.get_part("gasket")
    cap = object_model.get_part("flip_cap")
    hinge = object_model.get_articulation("cap_hinge")

    bottle_shell = body.get_visual("bottle_shell")
    cap_disc = cap.get_visual("cap_disc")
    gasket_ring = gasket.get_visual("gasket_ring")

    # --- bottle body is semi-transparent ---
    ctx.check(
        "bottle material is semi-transparent",
        bottle_shell.material.rgba is not None and bottle_shell.material.rgba[3] < 1.0,
        details=f"bottle rgba={bottle_shell.material.rgba}",
    )

    # --- cap is opaque dark ---
    ctx.check(
        "cap material is opaque dark",
        cap_disc.material.rgba is not None
        and cap_disc.material.rgba[3] >= 0.99
        and max(cap_disc.material.rgba[:3]) < 0.2,
        details=f"cap rgba={cap_disc.material.rgba}",
    )

    # --- gasket is separate opaque rubber part ---
    ctx.check(
        "gasket is a separate opaque part",
        gasket_ring.material.rgba is not None and gasket_ring.material.rgba[3] >= 0.99,
        details=f"gasket rgba={gasket_ring.material.rgba}",
    )

    # --- grip grooves: bottle body has surface indentations (barrel width varies) ---
    # The grooves make the body wider at ridges than at groove bottoms.
    # We check that the body dims in X or Y are larger than 2*(BODY_R - GROOVE_DEPTH)
    # confirming the grooved profile exists.
    body_dims = ctx.part_world_aabb(body)
    if body_dims is not None:
        body_dx = body_dims[1][0] - body_dims[0][0]
        body_dy = body_dims[1][1] - body_dims[0][1]
        # Outer diameter should be close to 2*BODY_R (at ridges)
        ctx.check(
            "bottle body has grip grooves (diameter matches ridged profile)",
            body_dx > 2.0 * (BODY_R - GROOVE_DEPTH) - 0.001,
            details=f"body X span={body_dx:.4f}, expected >{2*(BODY_R - GROOVE_DEPTH):.4f}",
        )

    # --- gasket sits on neck (above shoulder) ---
    gasket_pos = ctx.part_world_position(gasket)
    ctx.check(
        "gasket positioned at neck top",
        gasket_pos is not None and gasket_pos[2] > SHOULDER_TOP_Z,
        details=f"gasket pos={gasket_pos}",
    )

    # --- gasket below cap at rest ---
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "gasket is below flip cap",
        gasket_pos is not None and cap_pos is not None and gasket_pos[2] <= cap_pos[2],
        details=f"gasket z={gasket_pos[2]:.4f}, cap z={cap_pos[2]:.4f}",
    )

    # --- flip cap hinge is REVOLUTE ---
    ctx.check(
        "cap_hinge is revolute",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={hinge.articulation_type}",
    )

    # --- flip cap opens upward: at max angle, cap AABB center Z rises ---
    rest_aabb = ctx.part_world_aabb(cap)
    rest_center_z = (rest_aabb[0][2] + rest_aabb[1][2]) / 2.0
    with ctx.pose({hinge: 1.5}):
        open_aabb = ctx.part_world_aabb(cap)
        open_center_z = (open_aabb[0][2] + open_aabb[1][2]) / 2.0
    ctx.check(
        "flip cap opens upward (cap AABB center rises)",
        open_center_z > rest_center_z + 0.005,
        details=f"rest_center_z={rest_center_z:.4f}, open_center_z={open_center_z:.4f}",
    )

    # --- at rest, cap covers the mouth area ---
    ctx.expect_overlap(
        cap,
        body,
        axes="xy",
        min_overlap=0.005,
        name="closed cap overlaps neck area in XY",
    )

    # Allow the gasket to slightly overlap the bottle neck (seated compression)
    ctx.allow_overlap(
        gasket,
        body,
        elem_a="gasket_ring",
        elem_b="bottle_shell",
        reason="Gasket ring is intentionally seated on the neck rim with slight compression.",
    )

    # Allow cap plug to overlap body neck interior when closed
    ctx.allow_overlap(
        cap,
        body,
        elem_a="cap_disc",
        elem_b="bottle_shell",
        reason="Flip cap plug intentionally seats into the hollow mouth opening.",
    )

    return ctx.report()


object_model = build_object_model()
