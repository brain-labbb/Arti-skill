from __future__ import annotations

# Hinged swing-top bottle with wire bail geometry.
# Variant of clear plastic juice bottle with safety collar and flip-top stopper.
# Frame: bottle axis along +Z, base at z=0, neck/mouth at the top (+Z).
#
# Part hierarchy:
#   bottle (root) -> collar via collar_rotate (CONTINUOUS, +Z)
#   collar -> bail via bail_swing (REVOLUTE, +Y horizontal at collar pivot)
#
# The bail carries wire arms + stopper as one rigid assembly that swings
# open/closed. At q=0 the stopper plugs the mouth; positive q swings it away.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    WirePath,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---- key dimensions (meters) ----
BODY_R = 0.030          # outer barrel radius (~0.06 m dia)
WALL = 0.0018           # thin wall thickness
BASE_Z = 0.0            # bottom of the bottle
BARREL_TOP_Z = 0.105    # where the shoulder taper begins
SHOULDER_TOP_Z = 0.130  # top of the shoulder, base of the neck
NECK_R = 0.013          # neck outer radius
NECK_TOP_Z = 0.155      # top rim of the neck (mouth)
MOUTH_R = 0.010         # inner mouth opening radius
LIP_R = 0.015           # lip/ridge radius at neck top for stopper seating
LIP_H = 0.003           # lip height

# Collar dimensions
COLLAR_Z = 0.136        # collar center height (mid-neck, below lip)
COLLAR_TUBE_R = 0.003   # collar ring tube radius
COLLAR_MAJOR_R = NECK_R + 0.003  # collar ring center radius
PIVOT_R = COLLAR_MAJOR_R + COLLAR_TUBE_R + 0.002  # bail pivot radius (outer collar + gap)

# Bail dimensions
ARM_H = 0.032           # height of bail arch above pivot
WIRE_R = 0.0015         # wire radius
STOPPER_R = 0.011       # stopper disc radius (plugs into mouth)
STOPPER_H = 0.010       # stopper height

# Stopper disc bottom in bail frame should sit on the mouth rim at NECK_TOP_Z.
# Disc bottom in local = -STOPPER_H/2, so:
# COLLAR_Z + STOPPER_Z_BAIL - STOPPER_H/2 = NECK_TOP_Z
STOPPER_Z_BAIL = NECK_TOP_Z - COLLAR_Z + STOPPER_H / 2.0  # = 0.024


def _bottle_shell():
    """Transparent thin-wall bottle as one solid revolve, shelled open at top.
    Includes the neck with a lip ridge at the top for stopper seating."""
    wp = (
        cq.Workplane("XZ")
        .moveTo(0.0, BASE_Z)
        # rounded base corner
        .lineTo(BODY_R - 0.006, BASE_Z)
        .threePointArc((BODY_R, BASE_Z + 0.006), (BODY_R, BASE_Z + 0.012))
        # straight cylindrical barrel
        .lineTo(BODY_R, BARREL_TOP_Z)
        # shoulder taper up to the neck
        .threePointArc(
            ((BODY_R + NECK_R) / 2.0, (BARREL_TOP_Z + SHOULDER_TOP_Z) / 2.0 + 0.004),
            (NECK_R, SHOULDER_TOP_Z),
        )
    )
    # neck up to the lip
    wp = wp.lineTo(NECK_R, NECK_TOP_Z - LIP_H)
    # lip/ridge (wider than neck for stopper seating)
    wp = wp.lineTo(LIP_R, NECK_TOP_Z - LIP_H)
    wp = wp.lineTo(LIP_R, NECK_TOP_Z)
    # close back along axis
    wp = wp.lineTo(0.0, NECK_TOP_Z).close()
    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))
    # Shell open at the top face so the mouth is hollow and visible
    return outer.faces(">Z").shell(-WALL)


def _collar_mesh():
    """Safety collar ring: a torus around the neck with two pivot ears."""
    # Main ring (torus)
    ring = TorusGeometry(
        radius=COLLAR_MAJOR_R,
        tube=COLLAR_TUBE_R,
        radial_segments=12,
        tubular_segments=32,
    )
    # Add two pivot ear posts (small boxes on opposite sides)
    ear_w = 0.004
    ear_h = 0.006
    ear_d = 0.003
    ear_offset = COLLAR_MAJOR_R + COLLAR_TUBE_R + ear_d / 2.0
    # Left ear (+Y side)
    left_ear = BoxGeometry((ear_d, ear_w, ear_h))
    left_ear.translate(0.0, ear_offset, 0.0)
    # Right ear (-Y side)
    right_ear = BoxGeometry((ear_d, ear_w, ear_h))
    right_ear.translate(0.0, -ear_offset, 0.0)
    # Tab for tamper-evidence (small protrusion on +X side)
    tab = BoxGeometry((0.005, 0.003, 0.004))
    tab.translate(COLLAR_MAJOR_R + COLLAR_TUBE_R + 0.002, 0.0, 0.0)
    # Merge all
    geom = MeshGeometry()
    geom.merge(ring)
    geom.merge(left_ear)
    geom.merge(right_ear)
    geom.merge(tab)
    return geom


def _bail_wire_mesh():
    """Wire bail arms as a continuous tube from pivot to arch peak to pivot."""
    # Continuous path: left pivot -> up -> arch peak -> down -> right pivot
    # In bail frame: origin at pivot center (world COLLAR_Z)
    pivot_r = PIVOT_R
    mid_h = ARM_H * 0.55
    top_h = ARM_H
    # Use bezier for smooth arch shape
    wp = (
        WirePath((0.0, -pivot_r, 0.0))
        .bezier_to(
            (0.0, -pivot_r * 0.8, mid_h),
            (0.0, -0.005, top_h * 0.95),
            (0.0, 0.0, top_h),
            samples=16,
        )
        .bezier_to(
            (0.0, 0.005, top_h * 0.95),
            (0.0, pivot_r * 0.8, mid_h),
            (0.0, pivot_r, 0.0),
            samples=16,
        )
    )
    return tube_from_spline_points(
        wp.to_points(),
        radius=WIRE_R,
        samples_per_segment=8,
        radial_segments=10,
        cap_ends=True,
    )


def _stopper_mesh():
    """Ceramic/plastic stopper disc that plugs the mouth."""
    # Main disc
    disc = CylinderGeometry(STOPPER_R, STOPPER_H, radial_segments=24, closed=True)
    # Add a small plug/stem that goes into the mouth
    stem = CylinderGeometry(MOUTH_R * 0.85, 0.005, radial_segments=16, closed=True)
    stem.translate(0.0, 0.0, -STOPPER_H / 2.0 - 0.0025)
    geom = MeshGeometry()
    geom.merge(disc)
    geom.merge(stem)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swing_top_bottle")

    # Materials
    clear = model.material("clear_glass", rgba=(0.82, 0.88, 0.86, 0.30))
    metal = model.material("bail_metal", rgba=(0.72, 0.73, 0.74, 1.0))
    collar_mat = model.material("collar_plastic", rgba=(0.85, 0.15, 0.12, 1.0))
    stopper_mat = model.material("stopper_ceramic", rgba=(0.92, 0.90, 0.85, 1.0))

    # ---- bottle body (root) ----
    bottle = model.part("bottle")
    shell = _bottle_shell()
    bottle.visual(
        mesh_from_cadquery(shell, "bottle_shell"),
        material=clear,
        name="bottle_shell",
    )
    bottle.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, NECK_TOP_Z),
        mass=0.045,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    # ---- safety collar (rotates around neck) ----
    collar = model.part("collar")
    collar_mesh = _collar_mesh()
    collar.visual(
        mesh_from_geometry(collar_mesh, "collar_ring"),
        material=collar_mat,
        name="collar_ring",
    )
    collar.inertial = Inertial.from_geometry(
        Cylinder(COLLAR_MAJOR_R + COLLAR_TUBE_R, 0.008),
        mass=0.004,
    )

    # ---- bail + stopper assembly (swings on collar pivot) ----
    bail = model.part("bail")
    bail_wire = _bail_wire_mesh()
    bail.visual(
        mesh_from_geometry(bail_wire, "bail_wire"),
        material=metal,
        name="bail_wire",
    )
    # Stopper hangs from the arch peak
    stopper_mesh = _stopper_mesh()
    bail.visual(
        mesh_from_geometry(stopper_mesh, "stopper_disc"),
        material=stopper_mat,
        name="stopper_disc",
        origin=Origin(xyz=(0.0, 0.0, STOPPER_Z_BAIL)),
    )
    bail.inertial = Inertial.from_geometry(
        Cylinder(PIVOT_R, ARM_H),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, ARM_H / 2.0)),
    )

    # ---- articulations ----
    # collar_rotate: safety collar spins around the neck axis
    model.articulation(
        "collar_rotate",
        ArticulationType.CONTINUOUS,
        parent=bottle,
        child=collar,
        origin=Origin(xyz=(0.0, 0.0, COLLAR_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0),
    )

    # bail_swing: bail+stopper swings on horizontal Y axis at collar pivot
    # At q=0: stopper is closed (on mouth). Positive q swings bail open.
    model.articulation(
        "bail_swing",
        ArticulationType.REVOLUTE,
        parent=collar,
        child=bail,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=2.3,  # ~132 degrees open
            effort=2.0,
            velocity=3.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bottle = object_model.get_part("bottle")
    collar = object_model.get_part("collar")
    bail = object_model.get_part("bail")
    collar_rotate = object_model.get_articulation("collar_rotate")
    bail_swing = object_model.get_articulation("bail_swing")

    # --- bottle is clear/transparent ---
    bottle_shell = bottle.get_visual("bottle_shell")
    ctx.check(
        "bottle material is tinted-transparent (alpha < 1)",
        bottle_shell.material.rgba is not None and bottle_shell.material.rgba[3] < 1.0,
        details=f"bottle rgba={bottle_shell.material.rgba}",
    )

    # --- collar ring exists and is mounted around the neck ---
    collar_ring = collar.get_visual("collar_ring")
    ctx.check(
        "collar ring exists",
        collar_ring is not None,
    )
    collar_pos = ctx.part_world_position(collar)
    ctx.check(
        "collar is mounted at neck height",
        collar_pos is not None and SHOULDER_TOP_Z < collar_pos[2] < NECK_TOP_Z,
        details=f"collar_pos={collar_pos}",
    )

    # --- bail has wire and stopper visuals ---
    bail_wire = bail.get_visual("bail_wire")
    stopper_disc = bail.get_visual("stopper_disc")
    ctx.check("bail wire visual exists", bail_wire is not None)
    ctx.check("stopper disc visual exists", stopper_disc is not None)

    # --- bail_swing is revolute with proper limits ---
    ctx.check(
        "bail_swing is revolute",
        bail_swing.articulation_type == ArticulationType.REVOLUTE,
    )
    limits = bail_swing.motion_limits
    ctx.check(
        "bail_swing has non-trivial range",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.upper - limits.lower > 1.0,
        details=f"limits={limits}",
    )

    # --- collar_rotate is continuous ---
    ctx.check(
        "collar_rotate is continuous",
        collar_rotate.articulation_type == ArticulationType.CONTINUOUS,
    )

    # --- collar ring wraps around neck (intentional overlap) ---
    ctx.allow_overlap(
        collar,
        bottle,
        elem_a="collar_ring",
        elem_b="bottle_shell",
        reason="Collar ring clips around the bottle neck as a safety ring.",
    )
    ctx.expect_within(
        collar,
        bottle,
        axes="xy",
        inner_elem="collar_ring",
        outer_elem="bottle_shell",
        margin=0.005,
        name="collar ring stays within bottle neck footprint",
    )

    # --- stopper sits near mouth at rest (q=0) ---
    with ctx.pose({bail_swing: 0.0}):
        closed_aabb = ctx.part_world_aabb(bail)
    # --- bail swing opens: AABB shifts in X when bail rotates around Y ---
    with ctx.pose({bail_swing: 2.0}):
        open_aabb = ctx.part_world_aabb(bail)

    if closed_aabb is not None and open_aabb is not None:
        closed_center_x = (closed_aabb[0][0] + closed_aabb[1][0]) / 2.0
        open_center_x = (open_aabb[0][0] + open_aabb[1][0]) / 2.0
        ctx.check(
            "bail swing moves the bail assembly away from closed position",
            abs(open_center_x - closed_center_x) > 0.008,
            details=f"closed_center_x={closed_center_x}, open_center_x={open_center_x}",
        )
    else:
        ctx.fail("bail AABB available", f"closed={closed_aabb}, open={open_aabb}")

    # --- intentional overlap: stopper plug inserts into mouth ---
    ctx.allow_overlap(
        bail,
        bottle,
        elem_a="stopper_disc",
        elem_b="bottle_shell",
        reason="Stopper plug stem is intentionally seated inside the bottle mouth.",
    )

    # Prove the seating: stopper stem inserts into the mouth, disc sits on rim
    ctx.expect_gap(
        bail,
        bottle,
        axis="z",
        positive_elem="stopper_disc",
        negative_elem="bottle_shell",
        max_penetration=0.006,
        name="stopper seats into mouth with small stem insertion",
    )

    return ctx.report()


object_model = build_object_model()
