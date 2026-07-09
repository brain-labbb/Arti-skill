from __future__ import annotations

# Wide apothecary jar with a domed rose-gold stopper on a continuous screw joint.
# Frame: vertical axis +Z, jar centered on the world origin, base on z=0.
#
# Parts:
#   - jar_body: a wide cylindrical amber-glass jar with hollow interior,
#               a raised neck, and a wide open mouth at the top.
#   - stopper : a domed rose-gold stopper with a ground-glass plug that inserts
#               into the jar mouth. Rotates on a CONTINUOUS screw joint.
#
# Articulation:
#   - body_to_stopper: CONTINUOUS around Z. The stopper screws into the jar
#     mouth. At q=0 the stopper is fully seated; positive q rotates the
#     stopper (unscrewing direction).

import math
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ----- key dimensions (meters) -----
# Jar body
JAR_OUTER_R = 0.055       # outer radius of the jar cylinder
JAR_BODY_H = 0.085        # height of the main cylindrical body
GLASS_WALL = 0.004        # glass wall thickness
GLASS_BASE = 0.008        # solid glass floor thickness

# Neck / mouth
NECK_OUTER_R = 0.044      # outer radius of the raised neck
NECK_INNER_R = 0.038      # inner radius of the mouth (wide opening)
NECK_H = 0.016            # height of the raised neck above the body top

JAR_TOP_Z = JAR_BODY_H + NECK_H  # top of the neck / mouth rim

# Stopper
PLUG_R = NECK_INNER_R     # plug radius (ground-glass fit, contacts the mouth wall)
PLUG_H = 0.016            # plug insertion depth into mouth
FLANGE_R = 0.048          # stopper flange radius (rests on neck rim)
FLANGE_H = 0.005          # flange thickness
DOME_R = 0.034            # dome base radius
DOME_H = 0.028            # dome height (squished sphere)
KNOB_R = 0.009            # finial knob radius

# Stopper stacking (in stopper-local frame, plug bottom at z=0):
# plug: 0 .. PLUG_H
# flange: PLUG_H .. PLUG_H + FLANGE_H
# dome: PLUG_H + FLANGE_H .. PLUG_H + FLANGE_H + DOME_H
# knob center: PLUG_H + FLANGE_H + DOME_H + KNOB_R * 0.5

STOPPER_LOCAL_TOP = PLUG_H + FLANGE_H + DOME_H


def _jar_body_solid() -> cq.Workplane:
    """Wide cylindrical amber-glass jar with hollow interior and raised neck.

    The jar has:
    - A thick glass base
    - Cylindrical walls
    - A shoulder that transitions to a narrower raised neck
    - A wide open mouth at the top (hollow through the neck)
    """
    # Main cylindrical body (solid for now)
    outer_body = (
        cq.Workplane("XY")
        .circle(JAR_OUTER_R)
        .extrude(JAR_BODY_H)
    )

    # Raised neck on top of the body
    outer_neck = (
        cq.Workplane("XY")
        .workplane(offset=JAR_BODY_H)
        .circle(NECK_OUTER_R)
        .extrude(NECK_H)
    )

    # Combine body + neck outer shell
    jar = outer_body.union(outer_neck)

    # Hollow interior: cylindrical cavity within the body, stopping below the
    # body top to leave a solid shoulder that connects the neck to the body wall.
    inner_r = JAR_OUTER_R - GLASS_WALL
    SHOULDER_H = 0.006  # solid shoulder thickness between cavity top and body top
    cavity_h = JAR_BODY_H - GLASS_BASE - SHOULDER_H
    inner_cavity = (
        cq.Workplane("XY")
        .workplane(offset=GLASS_BASE)
        .circle(inner_r)
        .extrude(cavity_h)
    )
    jar = jar.cut(inner_cavity)

    # Mouth bore: wide opening through the shoulder and neck, creating the
    # visible wide-mouth hollow opening that connects the body cavity to the top.
    bore_start_z = JAR_BODY_H - SHOULDER_H  # starts inside the shoulder
    mouth_bore = (
        cq.Workplane("XY")
        .workplane(offset=bore_start_z)
        .circle(NECK_INNER_R)
        .extrude(SHOULDER_H + NECK_H + 0.001)
    )
    jar = jar.cut(mouth_bore)

    # Add a slight chamfer/rim at the mouth top for realism
    try:
        jar = jar.edges(">Z").edges(cq.selectors.RadiusNthSelector(0)).chamfer(0.001)
    except Exception:
        pass  # chamfer is cosmetic, skip if selection fails

    return jar


def _stopper_solid() -> cq.Workplane:
    """Domed rose-gold stopper with ground-glass plug and flange.

    Local frame: plug bottom at z=0, everything grows upward.
    """
    # Plug cylinder (ground-glass joint portion)
    plug = (
        cq.Workplane("XY")
        .circle(PLUG_R)
        .extrude(PLUG_H)
    )

    # Flange / shoulder that rests on the jar rim
    flange = (
        cq.Workplane("XY")
        .workplane(offset=PLUG_H)
        .circle(FLANGE_R)
        .extrude(FLANGE_H)
    )

    # Dome: use a sphere squished to be a dome shape
    # Create a sphere at the right height, then cut below the flange top
    dome_center_z = PLUG_H + FLANGE_H
    dome_sphere = (
        cq.Workplane("XY")
        .workplane(offset=dome_center_z)
        .sphere(DOME_R)
    )
    # Scale the sphere to make it a flatter dome (squish in Z)
    # Instead, use a half-sphere approach: create sphere and cut below
    # For a dome that's DOME_H tall with base radius DOME_R:
    # We use a sphere of radius R where R = (DOME_R^2 + DOME_H^2) / (2 * DOME_H)
    dome_sphere_r = (DOME_R**2 + DOME_H**2) / (2.0 * DOME_H)
    # The sphere center is at z = dome_center_z - (dome_sphere_r - DOME_H) relative
    sphere_z_offset = dome_center_z - (dome_sphere_r - DOME_H)
    dome = (
        cq.Workplane("XY")
        .workplane(offset=sphere_z_offset)
        .sphere(dome_sphere_r)
    )
    # Cut below the flange top to make it a dome
    cutter_below = (
        cq.Workplane("XY")
        .rect(FLANGE_R * 3, FLANGE_R * 3)
        .extrude(dome_center_z)
    )
    dome = dome.cut(cutter_below)

    # Finial knob on top
    knob_z = dome_center_z + DOME_H + KNOB_R * 0.6
    knob = (
        cq.Workplane("XY")
        .workplane(offset=knob_z)
        .sphere(KNOB_R)
    )

    # Combine all stopper pieces
    stopper = plug.union(flange).union(dome).union(knob)
    return stopper


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="apothecary_jar")

    amber_glass = model.material("amber_glass", rgba=(0.55, 0.35, 0.15, 0.40))
    rose_gold = model.material("rose_gold", rgba=(0.86, 0.58, 0.50, 1.0))

    # ---- jar_body (root): wide amber glass apothecary jar ----
    jar_body = model.part("jar_body")
    jar_body.visual(
        mesh_from_cadquery(_jar_body_solid(), "jar_glass_body"),
        material=amber_glass,
        name="jar_glass_body",
    )
    jar_body.inertial = Inertial.from_geometry(
        Cylinder(radius=JAR_OUTER_R, length=JAR_TOP_Z),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, JAR_TOP_Z / 2.0)),
    )

    # ---- stopper: domed rose-gold stopper with plug ----
    stopper = model.part("stopper")
    stopper.visual(
        mesh_from_cadquery(_stopper_solid(), "stopper_dome"),
        material=rose_gold,
        name="stopper_dome",
    )
    stopper.inertial = Inertial.from_geometry(
        Cylinder(radius=FLANGE_R, length=STOPPER_LOCAL_TOP),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, STOPPER_LOCAL_TOP / 2.0)),
    )

    # The stopper sits in the jar mouth. At q=0, the plug is inserted into the
    # mouth (plug bottom at z = JAR_BODY_H - PLUG_H + small offset so the
    # flange rests on the neck rim).
    # Plug bottom world Z when seated:
    plug_bottom_z = JAR_TOP_Z - PLUG_H
    # The stopper local origin is at plug bottom, so the articulation origin
    # is at plug_bottom_z in world (parent = jar_body frame at world origin).
    model.articulation(
        "body_to_stopper",
        ArticulationType.CONTINUOUS,
        parent=jar_body,
        child=stopper,
        origin=Origin(xyz=(0.0, 0.0, plug_bottom_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    jar_body = object_model.get_part("jar_body")
    stopper = object_model.get_part("stopper")
    screw = object_model.get_articulation("body_to_stopper")

    # The stopper plug is intentionally inserted into the jar mouth (screw
    # thread engagement). Allow that local overlap.
    ctx.allow_overlap(
        stopper,
        jar_body,
        elem_a="stopper_dome",
        elem_b="jar_glass_body",
        reason="Stopper plug is intentionally inserted into the jar mouth for screw-thread engagement.",
    )

    # ---- jar is wide, not tall (apothecary proportions) ----
    body_aabb = ctx.part_world_aabb(jar_body)
    if body_aabb is not None:
        mn, mx = body_aabb
        dx = mx[0] - mn[0]
        dy = mx[1] - mn[1]
        dz = mx[2] - mn[2]
        ctx.check(
            "jar body is wide (diameter comparable to or greater than height)",
            max(dx, dy) >= dz * 0.7,
            details=f"body extents: dx={dx:.4f}, dy={dy:.4f}, dz={dz:.4f}",
        )
        ctx.check(
            "jar body is roughly circular in section",
            abs(dx - dy) < 0.008,
            details=f"body XY extents: dx={dx:.4f}, dy={dy:.4f}",
        )

    # ---- wide mouth opening exists: the neck inner radius is wide ----
    ctx.check(
        "wide mouth opening (NECK_INNER_R > 0.03m)",
        NECK_INNER_R > 0.030,
        details=f"NECK_INNER_R={NECK_INNER_R}",
    )

    # ---- stopper sits at the top of the jar ----
    stopper_pos = ctx.part_world_position(stopper)
    ctx.check(
        "stopper sits above the jar body midpoint",
        stopper_pos is not None and stopper_pos[2] > JAR_BODY_H * 0.5,
        details=f"stopper_pos={stopper_pos}",
    )

    # ---- stopper footprint overlaps the jar mouth in XY ----
    ctx.expect_overlap(
        stopper,
        jar_body,
        axes="xy",
        min_overlap=0.020,
        name="stopper plug is centered in the jar mouth (XY overlap)",
    )

    # ---- stopper plug is inserted into the mouth (Z overlap at rest) ----
    ctx.expect_overlap(
        stopper,
        jar_body,
        axes="z",
        min_overlap=0.005,
        name="stopper plug engages the jar mouth at rest",
    )

    # ---- joint type is CONTINUOUS (screw) ----
    ctx.check(
        "body_to_stopper is a CONTINUOUS joint (screw rotation)",
        screw.articulation_type == ArticulationType.CONTINUOUS,
        details=f"articulation_type={screw.articulation_type}",
    )

    # ---- rotating the stopper does not translate it (screw in place) ----
    rest_pos = ctx.part_world_position(stopper)
    with ctx.pose({screw: math.pi}):
        rotated_pos = ctx.part_world_position(stopper)
    ctx.check(
        "stopper rotates in place (no lateral drift under screw rotation)",
        rest_pos is not None
        and rotated_pos is not None
        and abs(rotated_pos[0] - rest_pos[0]) < 1e-5
        and abs(rotated_pos[1] - rest_pos[1]) < 1e-5,
        details=f"rest={rest_pos}, rotated={rotated_pos}",
    )

    # ---- materials: jar is amber glass, stopper is rose-gold ----
    jar_mat = jar_body.get_visual("jar_glass_body").material
    stopper_mat = stopper.get_visual("stopper_dome").material
    ctx.check(
        "jar is amber glass and stopper is rose-gold (distinct materials)",
        jar_mat is not None
        and stopper_mat is not None
        and getattr(jar_mat, "name", None) == "amber_glass"
        and getattr(stopper_mat, "name", None) == "rose_gold",
        details=f"jar_mat={getattr(jar_mat, 'name', None)}, stopper_mat={getattr(stopper_mat, 'name', None)}",
    )

    return ctx.report()


object_model = build_object_model()
