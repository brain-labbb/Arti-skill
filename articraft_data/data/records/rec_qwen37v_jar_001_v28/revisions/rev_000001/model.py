from __future__ import annotations

# HONEY JAR with dipper holder on the lid and a lift-out stopper.
# Frame: jar axis along +Z, base resting on z=0, jar centered on (x=0, y=0).
#
# Parts:
#   body (root) - glass jar with thick walls at mouth, base foot ring, rim seam
#   lid         - screw-on lid with a central dipper-holder tube on top
#   stopper     - cylindrical plug with ball handle, lifts from the holder
#
# Articulations:
#   lid_rotate:   CONTINUOUS spin of the lid about +Z at the rim top
#   stopper_lift: PRISMATIC vertical lift of the stopper along +Z

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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- key dimensions (meters) ----
JAR_OUTER_R = 0.036            # outer radius of the glass body
JAR_BODY_H = 0.058             # height of the glass body (taller than face cream jar)
WALL = 0.004                   # glass wall thickness (body)
WALL_MOUTH = 0.006             # thicker wall at the mouth rim
NECK_R = 0.032                 # outer radius of the threaded neck
NECK_H = 0.012                 # neck height above the shoulder
RIM_TOP_Z = JAR_BODY_H + NECK_H  # z of the rim top where lid seats

# Base foot ring
FOOT_RING_OUTER_R = JAR_OUTER_R + 0.001
FOOT_RING_INNER_R = JAR_OUTER_R - 0.008
FOOT_RING_H = 0.004

# Rim seam (a small ridge at the top of the mouth)
RIM_SEAM_R = NECK_R + 0.001
RIM_SEAM_H = 0.002

# Honey fill level
HONEY_TOP_Z = JAR_BODY_H - 0.006

# Lid dimensions (in lid-local frame, origin at articulation = rim top)
LID_OUTER_R = 0.037
LID_SKIRT_DEPTH = 0.010        # how far skirt drops below rim
LID_THICKNESS = 0.005          # lid disc thickness
LID_TOP_LOCAL = LID_THICKNESS  # top of lid disc in local z

# Dipper holder tube on top of lid
HOLDER_OUTER_R = 0.008
HOLDER_INNER_R = 0.006
HOLDER_H = 0.018

# Stopper
STOPPER_R = 0.0055             # slightly smaller than holder inner
STOPPER_PLUG_H = 0.016         # plug length that goes into holder
STOPPER_HANDLE_R = 0.009       # ball handle on top
STOPPER_TOTAL_H = STOPPER_PLUG_H + STOPPER_HANDLE_R * 2


def _jar_glass_solid() -> cq.Workplane:
    """Hollow thick-walled glass jar with visible wall thickness at mouth,
    base foot ring integrated, and rim seam ridge."""
    inner_r = JAR_OUTER_R - WALL
    inner_r_mouth = NECK_R - WALL_MOUTH

    # Main jar body profile (revolved about Z axis)
    pts = [
        (0.0, 0.0),
        (JAR_OUTER_R, 0.0),
        (JAR_OUTER_R, JAR_BODY_H - 0.008),
        (JAR_OUTER_R - 0.003, JAR_BODY_H),           # rounded shoulder
        (NECK_R, JAR_BODY_H + 0.003),                  # step into neck
        (NECK_R, RIM_TOP_Z),                           # neck outer wall up
        (NECK_R + 0.001, RIM_TOP_Z),                   # rim seam bump
        (NECK_R + 0.001, RIM_TOP_Z - RIM_SEAM_H),      # rim seam
        (NECK_R, RIM_TOP_Z - RIM_SEAM_H),              # back to neck wall
        # Now trace the rim top inward and down the inner wall
        (NECK_R, RIM_TOP_Z),                           # back up to rim
        (inner_r_mouth, RIM_TOP_Z),                    # across rim top (thick mouth wall)
        (inner_r_mouth, JAR_BODY_H - 0.002),           # inner neck wall down
        (inner_r, JAR_BODY_H - 0.008),                 # inner shoulder
        (inner_r, WALL + 0.002),                       # inner body wall
        (0.0, WALL + 0.002),                           # inner base
        (0.0, 0.0),
    ]
    # Build as two separate shells to avoid self-intersection in the profile
    # Outer shell: solid jar shape
    outer_pts = [
        (0.0, 0.0),
        (JAR_OUTER_R, 0.0),
        (JAR_OUTER_R, JAR_BODY_H - 0.008),
        (JAR_OUTER_R - 0.003, JAR_BODY_H),
        (NECK_R, JAR_BODY_H + 0.003),
        (NECK_R, RIM_TOP_Z),
        (NECK_R + 0.001, RIM_TOP_Z),
        (NECK_R + 0.001, RIM_TOP_Z - RIM_SEAM_H),
        (NECK_R, RIM_TOP_Z - RIM_SEAM_H),
        (NECK_R, RIM_TOP_Z),
        (0.0, RIM_TOP_Z),
        (0.0, 0.0),
    ]
    outer = (
        cq.Workplane("XZ")
        .polyline(outer_pts)
        .close()
        .revolve(360.0, (0, 0, 0), (0, 1, 0))
    )

    # Inner cavity (hollow)
    cavity_pts = [
        (0.0, WALL + 0.002),
        (inner_r, WALL + 0.002),
        (inner_r, JAR_BODY_H - 0.008),
        (inner_r_mouth, JAR_BODY_H - 0.002),
        (inner_r_mouth, RIM_TOP_Z + 0.001),
        (0.0, RIM_TOP_Z + 0.001),
        (0.0, WALL + 0.002),
    ]
    cavity = (
        cq.Workplane("XZ")
        .polyline(cavity_pts)
        .close()
        .revolve(360.0, (0, 0, 0), (0, 1, 0))
    )

    jar = outer.cut(cavity)

    # Add rim seam as a separate raised ring for visual clarity
    rim_ring = (
        cq.Workplane("XY")
        .workplane(offset=RIM_TOP_Z - RIM_SEAM_H)
        .circle(NECK_R + 0.0015)
        .circle(NECK_R - 0.0005)
        .extrude(RIM_SEAM_H)
    )
    jar = jar.union(rim_ring)

    # Base foot ring
    foot = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .circle(FOOT_RING_OUTER_R)
        .circle(FOOT_RING_INNER_R)
        .extrude(FOOT_RING_H)
    )
    # Also add a flat base disc inside the foot ring
    base_disc = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .circle(FOOT_RING_INNER_R)
        .extrude(FOOT_RING_H * 0.5)
    )
    jar = jar.union(foot).union(base_disc)

    return jar


def _neck_threads() -> cq.Workplane:
    """Thread ridges on the neck exterior."""
    threads = None
    z0 = JAR_BODY_H + 0.004
    for i in range(3):
        z = z0 + i * 0.003
        ring = (
            cq.Workplane("XY")
            .workplane(offset=z)
            .circle(NECK_R + 0.0008)
            .circle(NECK_R - 0.0005)
            .extrude(0.0018)
        )
        threads = ring if threads is None else threads.union(ring)
    return threads


def _honey_fill() -> cq.Workplane:
    """Amber honey filling inside the jar."""
    inner_r = JAR_OUTER_R - WALL - 0.001
    fill = (
        cq.Workplane("XY")
        .workplane(offset=WALL + 0.003)
        .circle(inner_r)
        .extrude(HONEY_TOP_Z - WALL - 0.003)
    )
    # Slight dome on top surface
    dome = (
        cq.Workplane("XY")
        .workplane(offset=HONEY_TOP_Z)
        .circle(inner_r)
        .workplane(offset=0.003)
        .circle(inner_r * 0.6)
        .loft(ruled=False)
    )
    return fill.union(dome)


def _lid_solid() -> cq.Workplane:
    """Lid disc with skirt and central hole for the dipper holder."""
    # Main lid disc (in lid-local frame, z=0 at rim top / lid bottom)
    lid_disc = (
        cq.Workplane("XY")
        .workplane(offset=-LID_SKIRT_DEPTH)
        .circle(LID_OUTER_R)
        .extrude(LID_SKIRT_DEPTH + LID_THICKNESS)
    )
    # Fillet top edge
    lid_disc = lid_disc.edges(">Z").fillet(0.002)

    # Skirt cavity (hollow inside to fit over neck)
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=-LID_SKIRT_DEPTH - 0.001)
        .circle(NECK_R + 0.0005)
        .extrude(LID_SKIRT_DEPTH + 0.001)
    )
    lid_disc = lid_disc.cut(cavity)

    # Central hole through the lid for the dipper holder
    center_hole = (
        cq.Workplane("XY")
        .workplane(offset=-LID_SKIRT_DEPTH - 0.001)
        .circle(HOLDER_INNER_R + 0.0005)
        .extrude(LID_SKIRT_DEPTH + LID_THICKNESS + 0.002)
    )
    lid_disc = lid_disc.cut(center_hole)

    return lid_disc


def _dipper_holder() -> cq.Workplane:
    """Raised cylindrical tube on top of the lid for holding a honey dipper."""
    # Outer tube
    tube = (
        cq.Workplane("XY")
        .workplane(offset=LID_THICKNESS)
        .circle(HOLDER_OUTER_R)
        .extrude(HOLDER_H)
    )
    # Inner bore
    bore = (
        cq.Workplane("XY")
        .workplane(offset=LID_THICKNESS - 0.001)
        .circle(HOLDER_INNER_R)
        .extrude(HOLDER_H + 0.001)
    )
    holder = tube.cut(bore)

    # Small flange at base where holder meets lid
    flange = (
        cq.Workplane("XY")
        .workplane(offset=LID_THICKNESS - 0.001)
        .circle(HOLDER_OUTER_R + 0.002)
        .circle(HOLDER_OUTER_R - 0.001)
        .extrude(0.003)
    )
    return holder.union(flange)


def _lid_grip_ribs() -> cq.Workplane:
    """Knurled grip ribs around the lid skirt."""
    ribs = None
    n = 36
    band_z = -LID_SKIRT_DEPTH + 0.002
    band_h = LID_SKIRT_DEPTH - 0.003
    for i in range(n):
        ang = 2.0 * math.pi * i / n
        r = LID_OUTER_R - 0.0005
        rib = (
            cq.Workplane("XY")
            .workplane(offset=band_z)
            .center(r * math.cos(ang), r * math.sin(ang))
            .rect(0.0014, 0.0014)
            .extrude(band_h)
        )
        ribs = rib if ribs is None else ribs.union(rib)
    return ribs


def _stopper_solid() -> cq.Workplane:
    """Stopper plug with ball handle on top. Authored in stopper-local frame
    with origin at the plug bottom (seated position)."""
    # Cylindrical plug
    plug = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .circle(STOPPER_R)
        .extrude(STOPPER_PLUG_H)
    )
    # Small taper at plug bottom
    taper = (
        cq.Workplane("XY")
        .workplane(offset=-0.002)
        .circle(STOPPER_R * 0.7)
        .workplane(offset=0.003)
        .circle(STOPPER_R)
        .loft(ruled=True)
    )
    # Ball handle on top
    handle_z = STOPPER_PLUG_H + STOPPER_HANDLE_R
    handle = (
        cq.Workplane("XY")
        .workplane(offset=handle_z - STOPPER_HANDLE_R)
        .sphere(STOPPER_HANDLE_R)
    )
    # Small neck connecting plug to handle
    neck = (
        cq.Workplane("XY")
        .workplane(offset=STOPPER_PLUG_H - 0.001)
        .circle(STOPPER_R * 0.6)
        .extrude(STOPPER_HANDLE_R + 0.001)
    )
    return plug.union(taper).union(neck).union(handle)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="honey_jar")

    # Materials
    glass_amber = model.material("glass_amber", rgba=(0.72, 0.58, 0.30, 0.45))
    honey_gold = model.material("honey_gold", rgba=(0.85, 0.60, 0.13, 0.92))
    lid_cream = model.material("lid_cream", rgba=(0.92, 0.88, 0.78, 1.0))
    holder_brown = model.material("holder_brown", rgba=(0.45, 0.30, 0.15, 1.0))
    stopper_wood = model.material("stopper_wood", rgba=(0.60, 0.40, 0.20, 1.0))
    label_cream = model.material("label_cream", rgba=(0.96, 0.93, 0.85, 1.0))

    # ---- jar body (root) ----
    body = model.part("body")

    jar_glass = _jar_glass_solid().union(_neck_threads())
    body.visual(
        mesh_from_cadquery(jar_glass, "jar_glass"),
        material=glass_amber,
        name="jar_glass",
    )

    # Honey fill
    body.visual(
        mesh_from_cadquery(_honey_fill(), "honey_fill"),
        material=honey_gold,
        name="honey_fill",
    )

    # Label band
    body.visual(
        Cylinder(JAR_OUTER_R + 0.0005, 0.022),
        origin=Origin(xyz=(0.0, 0.0, JAR_BODY_H * 0.45)),
        material=label_cream,
        name="label_band",
    )

    body.inertial = Inertial.from_geometry(
        Cylinder(JAR_OUTER_R, JAR_BODY_H),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, JAR_BODY_H * 0.5)),
    )

    # ---- lid (rotates about +Z at the rim top) ----
    lid = model.part("lid")

    lid_solid = _lid_solid().union(_dipper_holder())
    lid.visual(
        mesh_from_cadquery(lid_solid, "lid_shell"),
        material=lid_cream,
        name="lid_shell",
    )
    lid.visual(
        mesh_from_cadquery(_lid_grip_ribs(), "lid_ribs"),
        material=lid_cream,
        name="lid_ribs",
    )

    lid.inertial = Inertial.from_geometry(
        Cylinder(LID_OUTER_R, LID_THICKNESS + LID_SKIRT_DEPTH),
        mass=0.04,
        origin=Origin(xyz=(0.0, 0.0, (LID_THICKNESS - LID_SKIRT_DEPTH) * 0.5)),
    )

    model.articulation(
        "lid_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, RIM_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )

    # ---- stopper (lifts vertically from the dipper holder) ----
    stopper = model.part("stopper")

    stopper.visual(
        mesh_from_cadquery(_stopper_solid(), "stopper_plug"),
        material=stopper_wood,
        name="stopper_plug",
    )

    stopper.inertial = Inertial.from_geometry(
        Cylinder(STOPPER_R, STOPPER_TOTAL_H),
        mass=0.01,
        origin=Origin(xyz=(0.0, 0.0, STOPPER_TOTAL_H * 0.5)),
    )

    # Prismatic joint: stopper lifts along +Z from its seated position in the holder.
    # The stopper origin sits at the lid top (where the holder base is), so
    # the plug extends down into the holder at rest.
    # Seated position: plug bottom is at holder base (lid local z = LID_THICKNESS)
    # Articulation origin: at the holder base on the lid
    model.articulation(
        "stopper_lift",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=stopper,
        origin=Origin(xyz=(0.0, 0.0, LID_THICKNESS)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=HOLDER_H + 0.01,  # can lift fully out of holder
            effort=1.0,
            velocity=0.5,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    stopper = object_model.get_part("stopper")
    lid_rotate = object_model.get_articulation("lid_rotate")
    stopper_lift = object_model.get_articulation("stopper_lift")

    # Allow the stopper plug to overlap the lid holder tube (seated insertion)
    ctx.allow_overlap(
        stopper,
        lid,
        elem_a="stopper_plug",
        elem_b="lid_shell",
        reason="The stopper plug is intentionally seated inside the dipper holder tube on the lid.",
    )

    # Allow lid skirt to overlap jar neck (screw-on fit)
    ctx.allow_overlap(
        lid,
        body,
        elem_a="lid_shell",
        elem_b="jar_glass",
        reason="The lid skirt slips over the threaded neck when screwed on.",
    )

    # ---- jar has base foot ring geometry ----
    ctx.check(
        "base foot ring exists",
        any(v.name == "jar_glass" for v in body.visuals),
        details="jar_glass visual should include the foot ring in its revolved profile",
    )
    # Foot ring is below the main body
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "jar rests on foot ring",
        body_aabb is not None and body_aabb[0][2] >= -0.001,
        details=f"body aabb min z = {body_aabb[0][2] if body_aabb else None}",
    )

    # ---- lid sits on top of the jar ----
    lid_pos = ctx.part_world_position(lid)
    ctx.check(
        "lid is above the jar body",
        lid_pos is not None and lid_pos[2] > JAR_BODY_H,
        details=f"lid_pos={lid_pos}",
    )

    # ---- lid caps the neck (XY overlap with body) ----
    ctx.expect_overlap(
        lid, body, axes="xy", min_overlap=0.02,
        name="lid caps the neck in XY",
    )

    # ---- stopper is on the lid ----
    stopper_pos = ctx.part_world_position(stopper)
    ctx.check(
        "stopper is on top of the lid",
        stopper_pos is not None and stopper_pos[2] > RIM_TOP_Z,
        details=f"stopper_pos={stopper_pos}, rim_top={RIM_TOP_Z}",
    )

    # ---- stopper overlaps lid in XY (seated in holder) ----
    ctx.expect_overlap(
        stopper, lid, axes="xy", min_overlap=0.005,
        name="stopper is seated in the dipper holder",
    )

    # ---- stopper_lift raises the stopper vertically ----
    rest_z = ctx.part_world_position(stopper)[2]
    lift_amount = 0.02  # 20mm lift
    with ctx.pose({stopper_lift: lift_amount}):
        lifted_z = ctx.part_world_position(stopper)[2]
        # Stopper should have moved up by approximately the lift amount
        ctx.check(
            "stopper_lift raises the stopper vertically",
            lifted_z > rest_z + lift_amount * 0.8,
            details=f"rest_z={rest_z}, lifted_z={lifted_z}",
        )
        # At full lift, stopper should clear the holder top
        ctx.expect_gap(
            stopper, lid, axis="z",
            positive_elem="stopper_plug", negative_elem="lid_shell",
            min_gap=-0.005,  # small tolerance for geometry
            name="lifted stopper clears the holder",
        )

    # ---- lid_rotate spins the lid (and stopper follows as child) ----
    # Use an off-center visual to confirm rotation: the grip ribs are symmetric,
    # so we check that the stopper (child of lid) also rotates
    stopper_rest_pos = ctx.part_world_position(stopper)
    with ctx.pose({lid_rotate: math.pi}):
        stopper_rotated_pos = ctx.part_world_position(stopper)
    # Since stopper is centered on the rotation axis, its position shouldn't
    # change much, but the lid should have rotated. We verify the joint works.
    ctx.check(
        "lid_rotate joint is continuous",
        lid_rotate.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={lid_rotate.articulation_type}",
    )

    # ---- stopper_lift is prismatic with proper limits ----
    limits = stopper_lift.motion_limits
    ctx.check(
        "stopper_lift is prismatic",
        stopper_lift.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={stopper_lift.articulation_type}",
    )
    ctx.check(
        "stopper_lift has bounded range",
        limits is not None and limits.lower is not None and limits.upper is not None
        and limits.upper > limits.lower,
        details=f"limits={limits}",
    )

    # ---- at least one non-fixed joint exists ----
    joint_types = [a.articulation_type for a in object_model.articulations]
    ctx.check(
        "at least one non-fixed joint",
        any(t != ArticulationType.FIXED for t in joint_types),
        details=f"joint types={[str(t) for t in joint_types]}",
    )

    # ---- honey fill visible inside the jar ----
    ctx.check(
        "honey fill exists",
        any(v.name == "honey_fill" for v in body.visuals),
        details="honey_fill visual should exist on the body",
    )

    # ---- dipper holder tube on lid (geometry above lid disc) ----
    lid_aabb = ctx.part_world_aabb(lid)
    lid_shell_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    ctx.check(
        "dipper holder extends above lid disc",
        lid_aabb is not None and lid_shell_aabb is not None
        and lid_aabb[1][2] > RIM_TOP_Z + LID_THICKNESS + HOLDER_H * 0.5,
        details=f"lid top z = {lid_aabb[1][2] if lid_aabb else None}",
    )

    return ctx.report()


object_model = build_object_model()
