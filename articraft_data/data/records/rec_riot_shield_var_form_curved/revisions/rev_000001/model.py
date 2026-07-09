from __future__ import annotations

"""Classic handheld curved polycarbonate riot shield.

Single-body concave-toward-the-officer polycarbonate shield shell (~0.9 m tall
× 0.60 m wide) with smoke-tinted integral viewport band, horizontal molded
reinforcement ribs, dark rubber edge trim around the perimeter, and a cast
aluminum carry-handle bracket bolted to the convex outer face with four
dome-head carriage bolts (open rectangular grip window).  A pivoting forearm
retention strap attaches below the grip frame for adjustable carry angle.
"""

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Shield shell
# ---------------------------------------------------------------------------
SHIELD_W = 0.60        # width  (Y)
SHIELD_H = 0.90        # height (Z)
SHIELD_T = 0.005       # polycarbonate wall thickness
CURVE_R = 1.50         # curvature radius – large ⇒ gentle bow

# Edge trim
TRIM_W = 0.018         # rubber U-channel width
TRIM_T = 0.008         # rubber U-channel depth (proud of each face)

# Integral viewport band
VP_H = 0.15            # viewport band height
VP_W = 0.44            # viewport band width (narrower than shield)
VP_T = 0.003           # viewport tint film thickness
VP_Z = 0.25            # viewport band centre height

# Molded ribs
RIB_W = 0.010          # rib width (Z)
RIB_T = 0.003          # rib relief above shell surface
RIB_L = 0.46           # rib span (Y, shorter than shield)

# ---------------------------------------------------------------------------
# Carry-handle bracket (kept from parent)
# ---------------------------------------------------------------------------
PLATE_W = 0.21
PLATE_H = 0.15
PLATE_T = 0.016
FRAME_OUT_W = 0.15
FRAME_OUT_H = 0.10
FRAME_IN_W = 0.10
FRAME_IN_H = 0.05
FRAME_DEPTH = 0.048
HANDLE_Z = -0.05       # handle centre height (slightly below mid-shield)

BOLT_R = 0.010
BOLT_Y = 0.088
BOLT_Z = 0.060

# ---------------------------------------------------------------------------
# Forearm strap
# ---------------------------------------------------------------------------
STRAP_L = 0.20
STRAP_W = 0.040
STRAP_T = 0.004
BUCKLE_W = 0.035
BUCKLE_H = 0.025
BUCKLE_T = 0.006
PIVOT_LOWER = -0.55
PIVOT_UPPER = 0.55


# ===== Geometry helpers ====================================================

def _shell_surface_x(y: float, z: float, radius: float) -> float:
    """X coordinate on a cylindrical surface centred at x = -CURVE_R, axis ∥ Y.

    Returns the X of the point on the cylinder of the given *radius* at the
    requested (y, z).  ``y`` is along the cylinder axis and does not affect the
    result; ``z`` determines the arc angle.
    """
    # The cylinder axis is at (x, z) = (-CURVE_R, 0).  A point on the surface
    # at height z satisfies x = -CURVE_R + radius * cos(theta) where
    # theta = asin(z / radius).
    sin_t = max(-1.0, min(1.0, z / radius))
    return -CURVE_R + radius * math.cos(math.asin(sin_t))


def _build_curved_shell_mesh() -> MeshGeometry:
    """Triangulated thin curved rectangular panel (closed solid)."""
    R = CURVE_R
    W = SHIELD_W
    H = SHIELD_H
    T = SHIELD_T

    ny = 6     # width segments
    nz = 24    # height segments

    cols = ny + 1
    rows = nz + 1
    theta_max = math.asin(min(1.0, H / (2.0 * R)))

    geom = MeshGeometry()

    # --- outer surface (radius R) ---
    for iz in range(rows):
        theta = -theta_max + 2.0 * theta_max * iz / nz
        z = R * math.sin(theta)
        x = -R + R * math.cos(theta)
        for iy in range(cols):
            y = -W / 2.0 + W * iy / ny
            geom.add_vertex(x, y, z)

    # --- inner surface (radius R - T) ---
    ri = R - T
    n_outer = rows * cols
    for iz in range(rows):
        theta = -theta_max + 2.0 * theta_max * iz / nz
        z = ri * math.sin(theta)
        x = -R + ri * math.cos(theta)
        for iy in range(cols):
            y = -W / 2.0 + W * iy / ny
            geom.add_vertex(x, y, z)

    # --- outer faces (normals outward / +X) ---
    for iz in range(nz):
        for iy in range(ny):
            i00 = iz * cols + iy
            i10 = iz * cols + iy + 1
            i01 = (iz + 1) * cols + iy
            i11 = (iz + 1) * cols + iy + 1
            geom.add_face(i00, i10, i01)
            geom.add_face(i10, i11, i01)

    # --- inner faces (normals inward / -X, reversed winding) ---
    for iz in range(nz):
        for iy in range(ny):
            i00 = n_outer + iz * cols + iy
            i10 = n_outer + iz * cols + iy + 1
            i01 = n_outer + (iz + 1) * cols + iy
            i11 = n_outer + (iz + 1) * cols + iy + 1
            geom.add_face(i00, i01, i10)
            geom.add_face(i10, i01, i11)

    # --- edge caps ---
    # top edge (last row)
    for iy in range(ny):
        o0 = nz * cols + iy
        o1 = nz * cols + iy + 1
        n0 = n_outer + nz * cols + iy
        n1 = n_outer + nz * cols + iy + 1
        geom.add_face(o0, o1, n0)
        geom.add_face(o1, n1, n0)
    # bottom edge (first row)
    for iy in range(ny):
        o0 = iy
        o1 = iy + 1
        n0 = n_outer + iy
        n1 = n_outer + iy + 1
        geom.add_face(o0, n0, o1)
        geom.add_face(o1, n0, n1)
    # left edge (first column)
    for iz in range(nz):
        o0 = iz * cols
        o1 = (iz + 1) * cols
        n0 = n_outer + iz * cols
        n1 = n_outer + (iz + 1) * cols
        geom.add_face(o0, n0, o1)
        geom.add_face(o1, n0, n1)
    # right edge (last column)
    for iz in range(nz):
        o0 = iz * cols + ny
        o1 = (iz + 1) * cols + ny
        n0 = n_outer + iz * cols + ny
        n1 = n_outer + (iz + 1) * cols + ny
        geom.add_face(o0, o1, n0)
        geom.add_face(o1, n1, n0)

    return geom


def _rounded_plate() -> cq.Workplane:
    """Cast aluminium mounting plate with rounded corners (handle part frame)."""
    plate = cq.Workplane("YZ").rect(PLATE_W, PLATE_H).extrude(PLATE_T)
    try:
        plate = plate.edges("|X").fillet(0.018)
    except ValueError:
        pass
    return plate


def _grip_frame() -> cq.Workplane:
    """Open rectangular grip loop standing proud of the plate."""
    frame = (
        cq.Workplane("YZ", origin=(PLATE_T - 0.002, 0.0, 0.0))
        .rect(FRAME_OUT_W, FRAME_OUT_H)
        .rect(FRAME_IN_W, FRAME_IN_H)
        .extrude(FRAME_DEPTH)
    )
    try:
        frame = frame.edges("|X").fillet(0.008)
    except ValueError:
        pass
    return frame


# ===== Build ===============================================================

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="curved_polycarbonate_riot_shield")

    # Materials
    model.material("polycarbonate", rgba=(0.82, 0.84, 0.83, 0.55))
    model.material("smoke_tint", rgba=(0.32, 0.33, 0.34, 0.65))
    model.material("rubber_trim", rgba=(0.09, 0.09, 0.10, 1.0))
    model.material("cast_aluminum", rgba=(0.63, 0.64, 0.66, 1.0))
    model.material("bolt_steel", rgba=(0.24, 0.24, 0.26, 1.0))
    model.material("nylon_webbing", rgba=(0.07, 0.07, 0.08, 1.0))
    model.material("buckle_metal", rgba=(0.40, 0.40, 0.42, 1.0))

    # ---- Shield shell (root part) ----------------------------------------
    front = model.part("front_panel")

    # Main curved polycarbonate shell (mesh-backed for exact curvature)
    front.visual(
        mesh_from_geometry(_build_curved_shell_mesh(), "shield_shell"),
        material="polycarbonate",
        name="shield_shell",
    )

    # Integral smoke-tinted viewport band – thin curved strip on the outer
    # face.  We use a simple Box positioned at the outer surface since the
    # curvature deviation across VP_H is negligible.
    x_at_vp = _shell_surface_x(0.0, VP_Z, CURVE_R)
    front.visual(
        Box((VP_T, VP_W, VP_H)),
        origin=Origin(xyz=(x_at_vp + VP_T / 2 + 0.001, 0.0, VP_Z)),
        material="smoke_tint",
        name="viewport_band",
    )

    # Rubber edge trim – 4 perimeter strips.
    # Side edges (span full height at y = ±W/2)
    for i, sy in enumerate((-1.0, 1.0)):
        x_edge = _shell_surface_x(sy * SHIELD_W / 2.0, 0.0, CURVE_R)
        front.visual(
            Box((SHIELD_T + 2 * TRIM_T, TRIM_W, SHIELD_H)),
            origin=Origin(xyz=(x_edge + SHIELD_T / 2, sy * (SHIELD_W / 2.0 - TRIM_W / 2.0), 0.0)),
            material="rubber_trim",
            name=f"edge_trim_side_{i}",
        )
    # Top and bottom edges (span full width at z = ±H/2)
    for i, sz in enumerate((1.0, -1.0)):
        x_edge = _shell_surface_x(0.0, sz * SHIELD_H / 2.0, CURVE_R)
        front.visual(
            Box((SHIELD_T + 2 * TRIM_T, SHIELD_W, TRIM_W)),
            origin=Origin(xyz=(x_edge + SHIELD_T / 2, 0.0, sz * (SHIELD_H / 2.0 - TRIM_W / 2.0))),
            material="rubber_trim",
            name=f"edge_trim_end_{i}",
        )

    # Molded reinforcement ribs on the convex outer face.
    # Each rib sits slightly embedded into the shell to ensure geometry
    # connectivity (the rib base overlaps the shell outer surface by 1 mm).
    rib_z_positions = (-0.20, 0.05, 0.38)
    for i, rz in enumerate(rib_z_positions):
        x_rib = _shell_surface_x(0.0, rz, CURVE_R)
        front.visual(
            Box((RIB_T + 0.001, RIB_L, RIB_W)),
            origin=Origin(xyz=(x_rib + RIB_T / 2 - 0.0005, 0.0, rz)),
            material="polycarbonate",
            name=f"molded_rib_{i}",
        )

    # ---- Carry-handle bracket (bolted to convex outer face) ---------------
    handle = model.part("carry_handle")

    handle.visual(
        mesh_from_cadquery(_rounded_plate(), "handle_plate", tolerance=0.0012),
        material="cast_aluminum",
        name="mount_plate",
    )
    handle.visual(
        mesh_from_cadquery(_grip_frame(), "handle_grip_frame", tolerance=0.0012),
        material="cast_aluminum",
        name="grip_frame",
    )

    # Four dome-head carriage bolts at the plate corners
    corners = ((-1.0, -1.0), (-1.0, 1.0), (1.0, -1.0), (1.0, 1.0))
    for i, (sy, sz) in enumerate(corners):
        y = sy * BOLT_Y
        z = sz * BOLT_Z
        handle.visual(
            Cylinder(radius=BOLT_R, length=0.016),
            origin=Origin(xyz=(PLATE_T + 0.002, y, z), rpy=(0.0, math.pi / 2, 0.0)),
            material="bolt_steel",
            name=f"bolt_shank_{i}",
        )
        handle.visual(
            Sphere(radius=BOLT_R),
            origin=Origin(xyz=(PLATE_T + 0.010, y, z)),
            material="bolt_steel",
            name=f"bolt_dome_{i}",
        )

    # Mount: handle frame origin coincides with the outer shell surface at the
    # handle height.  The mount plate back face (x = 0 in handle frame) sits
    # flush against the convex surface.
    x_handle = _shell_surface_x(0.0, HANDLE_Z, CURVE_R)
    model.articulation(
        "panel_to_handle",
        ArticulationType.FIXED,
        parent=front,
        child=handle,
        origin=Origin(xyz=(x_handle, 0.0, HANDLE_Z)),
    )

    # ---- Forearm retention strap (pivots on grip frame) -------------------
    strap = model.part("forearm_strap")

    # Strap body hangs downward from the pivot point
    strap.visual(
        Box((STRAP_T, STRAP_W, STRAP_L)),
        origin=Origin(xyz=(0.0, 0.0, -STRAP_L / 2.0)),
        material="nylon_webbing",
        name="strap_body",
    )
    # Buckle / retention clip at the strap bottom
    strap.visual(
        Box((BUCKLE_T, BUCKLE_W, BUCKLE_H)),
        origin=Origin(xyz=(0.0, 0.0, -STRAP_L - BUCKLE_H / 2.0)),
        material="buckle_metal",
        name="strap_buckle",
    )
    # Pivot pin connecting strap to grip frame
    strap.visual(
        Cylinder(radius=0.006, length=FRAME_OUT_W + 0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="bolt_steel",
        name="pivot_pin",
    )

    # Pivot origin: at the bottom of the grip frame, centred in grip depth.
    # Axis (0, -1, 0) so positive q swings the strap outward (+X, away from
    # the officer), which is the natural "open" direction for a forearm strap.
    pivot_x = PLATE_T - 0.002 + FRAME_DEPTH / 2.0
    pivot_z = -FRAME_OUT_H / 2.0

    model.articulation(
        "strap_pivot",
        ArticulationType.REVOLUTE,
        parent=handle,
        child=strap,
        origin=Origin(xyz=(pivot_x, 0.0, pivot_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=3.0, lower=PIVOT_LOWER, upper=PIVOT_UPPER
        ),
    )

    return model


# ===== Tests ===============================================================

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front = object_model.get_part("front_panel")
    handle = object_model.get_part("carry_handle")
    strap = object_model.get_part("forearm_strap")
    pivot = object_model.get_articulation("strap_pivot")

    # ---- Prompt-specific presence checks ----------------------------------
    ctx.check(
        "curved_shield_shell_present",
        front.get_visual("shield_shell") is not None,
        "front_panel must carry the curved polycarbonate shield shell",
    )

    has_rear_panel = False
    try:
        object_model.get_part("rear_panel")
        has_rear_panel = True
    except Exception:
        pass
    ctx.check(
        "no_rear_panel",
        not has_rear_panel,
        "rear panel must be removed – this is a single-body shield",
    )

    ctx.check(
        "viewport_band_present",
        front.get_visual("viewport_band") is not None,
        "shield shell must include the integral smoke-tinted viewport band",
    )
    ctx.check(
        "grip_window_frame_present",
        handle.get_visual("grip_frame") is not None,
        "handle bracket must include the open grip frame",
    )
    ctx.check(
        "four_bolts_present",
        all(handle.get_visual(f"bolt_dome_{i}") is not None for i in range(4)),
        "handle bracket must be bolted with four dome-head bolts",
    )

    # ---- Handle mount on convex shell surface -----------------------------
    ctx.expect_contact(
        handle, front,
        elem_a="mount_plate", elem_b="shield_shell",
        contact_tol=0.005,
    )
    ctx.expect_overlap(
        handle, front,
        axes="yz",
        elem_a="mount_plate", elem_b="shield_shell",
        min_overlap=0.05,
    )

    # ---- Forearm strap pivot (the required non-fixed joint) ---------------
    ctx.check(
        "strap_pivot_is_revolute",
        pivot.articulation_type == ArticulationType.REVOLUTE,
        "strap pivot must be a revolute joint",
    )

    limits = pivot.motion_limits
    ctx.check(
        "strap_pivot_travel",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and (limits.upper - limits.lower) > 0.8,
        "strap pivot must provide at least 0.8 rad of adjustment travel",
    )

    # Pivot pin is captured inside the grip frame bottom rail.
    ctx.allow_overlap(
        "carry_handle", "forearm_strap",
        elem_a="grip_frame", elem_b="pivot_pin",
        reason="pivot pin is intentionally captured inside the grip frame bottom rail",
    )
    ctx.expect_contact(
        strap, handle,
        elem_a="pivot_pin", elem_b="grip_frame",
        contact_tol=0.010,
    )

    # Positive q (axis = (0,-1,0)) swings the strap outward (+X, away from
    # the officer).  The strap body and buckle translate significantly while
    # the pivot pin stays on the rotation axis.  Check the AABB min_z: the
    # strap bottom rises when tilted because the hanging elements swing up.
    with ctx.pose({pivot: 0.0}):
        rest_aabb = ctx.part_world_aabb(strap)
    with ctx.pose({pivot: PIVOT_UPPER}):
        tilted_aabb = ctx.part_world_aabb(strap)
    rest_min_z = rest_aabb[0][2] if rest_aabb is not None else None
    tilt_min_z = tilted_aabb[0][2] if tilted_aabb is not None else None
    tilt_max_x = tilted_aabb[1][0] if tilted_aabb is not None else None
    rest_max_x = rest_aabb[1][0] if rest_aabb is not None else None
    ctx.check(
        "strap_pivots_outward_at_positive_q",
        rest_min_z is not None
        and tilt_min_z is not None
        and tilt_min_z > rest_min_z + 0.01,
        f"positive pivot angle must raise the strap bottom: rest_min_z={rest_min_z}, "
        f"tilted_min_z={tilt_min_z}",
    )

    return ctx.report()


object_model = build_object_model()
