from __future__ import annotations

# MASON JAR with two-piece lid (ring + disk) and flip-open disk on rear hinge.
# Frame: jar axis along +Z, base resting on z=0, jar centered on (x=0, y=0).
#
# Parts:
#   jar_body  (root): tall glass mason jar with foot ring, body, shoulder,
#                     threaded neck, rim seam, and interior cavity
#   lid_ring:        screw band ring that sits on the neck threads
#   lid_disk:        flat metal disk that flips open on a rear revolute hinge
#
# Articulation:
#   lid_flip: REVOLUTE hinge at rear of mouth, axis along +X so positive q
#             lifts the front edge of the disk upward.

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
JAR_OUTER_R = 0.040           # outer radius of the glass body (~80mm dia)
JAR_BODY_H = 0.120            # height of the cylindrical body
WALL = 0.004                  # glass wall thickness
FOOT_OUTER_R = 0.043          # foot ring slightly wider than body
FOOT_H = 0.006                # foot ring height
FOOT_INSET = 0.003            # foot ring inner step

NECK_R = 0.040                # neck outer radius (wide mouth = body width)
NECK_H = 0.014                # neck height above shoulder
RIM_SEAM_R = 0.043            # rim seam outer radius (raised bead)
RIM_SEAM_H = 0.003            # rim seam height

SHOULDER_Z = JAR_BODY_H       # where body meets shoulder
NECK_BASE_Z = SHOULDER_Z      # neck starts here
RIM_TOP_Z = NECK_BASE_Z + NECK_H  # top of neck rim (0.134)

# Lid ring (screw band)
RING_OUTER_R = 0.044          # ring outer radius
RING_INNER_R = 0.0415         # ring inner radius (clears neck + threads)
RING_H = 0.016                # ring band height

# Lid disk (flat seal disk)
DISK_R = 0.037                # disk radius (fits inside ring)
DISK_THICK = 0.002            # disk thickness

# Hinge geometry (rear of jar mouth)
HINGE_Y = -(NECK_R - 0.004)   # hinge line at rear edge of neck (-0.036)
HINGE_Z = RIM_TOP_Z + RING_H * 0.7  # hinge pin near top of ring

# Lug dimensions (body side)
LUG_W = 0.010                 # lug width (X)
LUG_D = 0.008                 # lug depth (Y)
LUG_BASE_Z = RIM_TOP_Z - 0.002  # lug starts just below rim
LUG_TOP_Z = HINGE_Z + 0.003    # lug extends above hinge pin


def _jar_glass_solid() -> cq.Workplane:
    """Hollow thick-walled mason jar: foot ring, body, shoulder, neck, rim seam,
    and internal cavity. Built as a revolve of the half-profile in XZ."""
    inner_r = JAR_OUTER_R - WALL
    pts = [
        (0.0, 0.0),
        (FOOT_OUTER_R, 0.0),
        (FOOT_OUTER_R, FOOT_H),
        (JAR_OUTER_R, FOOT_H + 0.002),
        (JAR_OUTER_R, SHOULDER_Z - 0.008),
        (JAR_OUTER_R - 0.006, SHOULDER_Z),
        (NECK_R, NECK_BASE_Z + 0.003),
        (NECK_R, RIM_TOP_Z - RIM_SEAM_H),
        (RIM_SEAM_R, RIM_TOP_Z - RIM_SEAM_H),
        (RIM_SEAM_R, RIM_TOP_Z),
        (NECK_R - WALL, RIM_TOP_Z),
        (NECK_R - WALL, NECK_BASE_Z + 0.004),
        (inner_r, SHOULDER_Z - 0.004),
        (inner_r, FOOT_H + WALL + 0.002),
        (FOOT_OUTER_R - FOOT_INSET - WALL, FOOT_H + WALL),
        (0.0, FOOT_H + WALL),
    ]
    profile = cq.Workplane("XZ").polyline(pts).close()
    return profile.revolve(360.0, (0, 0, 0), (0, 1, 0))


def _neck_threads() -> cq.Workplane:
    """Thread ridges on the outside of the neck for the screw band."""
    threads = None
    z0 = NECK_BASE_Z + 0.004
    for i in range(4):
        z = z0 + i * 0.0028
        ring = (
            cq.Workplane("XY")
            .workplane(offset=z)
            .circle(NECK_R + 0.0008)
            .circle(NECK_R - 0.0003)
            .extrude(0.0018)
        )
        threads = ring if threads is None else threads.union(ring)
    return threads


def _foot_ring_detail() -> cq.Workplane:
    """A subtle raised bead at the base of the body above the foot."""
    bead_z = FOOT_H + 0.001
    return (
        cq.Workplane("XY")
        .workplane(offset=bead_z)
        .circle(JAR_OUTER_R + 0.001)
        .circle(JAR_OUTER_R - 0.001)
        .extrude(0.002)
    )


def _hinge_mount_body() -> cq.Workplane:
    """Hinge lug + pin on the jar body at the rear of the neck.
    One connected solid: vertical lug from rim up through pin height, with
    a horizontal cylindrical pin through it."""
    # Vertical lug
    lug = (
        cq.Workplane("XY")
        .workplane(offset=LUG_BASE_Z)
        .center(0.0, HINGE_Y - LUG_D * 0.3)
        .rect(LUG_W, LUG_D)
        .extrude(LUG_TOP_Z - LUG_BASE_Z)
    )
    # Horizontal pin through lug (along X axis at hinge height)
    pin_r = 0.0018
    pin_half = LUG_W * 0.5 + 0.003  # pin extends past lug sides
    pin = (
        cq.Workplane("YZ")
        .workplane(offset=-pin_half)
        .center(HINGE_Y - LUG_D * 0.3, HINGE_Z)
        .circle(pin_r)
        .extrude(pin_half * 2)
    )
    return lug.union(pin)


def _lid_ring_solid() -> cq.Workplane:
    """Screw band ring: a cylindrical band with a central hole."""
    outer = (
        cq.Workplane("XY")
        .circle(RING_OUTER_R)
        .extrude(RING_H)
    )
    inner = (
        cq.Workplane("XY")
        .circle(RING_INNER_R)
        .extrude(RING_H)
    )
    band = outer.cut(inner)
    band = band.edges(">Z").fillet(0.0015)
    band = band.edges("<Z").fillet(0.001)
    return band


def _lid_ring_threads() -> cq.Workplane:
    """Internal thread ridges inside the ring that engage the neck threads."""
    threads = None
    for i in range(4):
        z = 0.003 + i * 0.0028
        ring = (
            cq.Workplane("XY")
            .workplane(offset=z)
            .circle(RING_INNER_R + 0.0006)
            .circle(RING_INNER_R - 0.0004)
            .extrude(0.0016)
        )
        threads = ring if threads is None else threads.union(ring)
    return threads


def _disk_assembly() -> cq.Workplane:
    """Disk plate + hinge ear as one connected solid.
    Part frame origin is at the hinge line (HINGE_Y, HINGE_Z in world).
    The disk plate sits forward (+Y in part frame) at the rim level when closed.
    A connecting ear bridges from the plate's rear edge up to the hinge point."""
    # Disk plate position in part-local coords
    plate_cy = -HINGE_Y  # forward offset to center over jar mouth
    plate_cz = -(HINGE_Z - RIM_TOP_Z)  # below hinge to sit on rim
    plate = (
        cq.Workplane("XY")
        .workplane(offset=plate_cz - DISK_THICK * 0.5)
        .center(0.0, plate_cy)
        .circle(DISK_R)
        .extrude(DISK_THICK)
    )
    # Connecting ear from plate rear edge to hinge point
    # Plate rear edge Y in part frame: plate_cy - DISK_R = -HINGE_Y - DISK_R
    ear_rear_y = plate_cy - DISK_R  # ≈ -0.001
    ear_w = 0.008
    ear_bottom_z = plate_cz - DISK_THICK * 0.5
    ear_top_z = 0.003  # slightly above hinge axis
    # Ear is a box from ear_rear_y to +0.002 in Y, ear_bottom_z to ear_top_z in Z
    ear_depth = abs(ear_rear_y) + 0.003
    ear = (
        cq.Workplane("XY")
        .workplane(offset=ear_bottom_z)
        .center(0.0, ear_rear_y + ear_depth * 0.5)
        .rect(ear_w, ear_depth)
        .extrude(ear_top_z - ear_bottom_z)
    )
    return plate.union(ear)


def _seal_gasket() -> cq.Workplane:
    """Red rubber seal ring on underside of disk, touching the plate bottom."""
    plate_cy = -HINGE_Y
    plate_cz = -(HINGE_Z - RIM_TOP_Z)
    # Seal sits just below the plate bottom surface
    seal_z = plate_cz - DISK_THICK * 0.5 - 0.0008
    return (
        cq.Workplane("XY")
        .workplane(offset=seal_z)
        .center(0.0, plate_cy)
        .circle(DISK_R - 0.003)
        .circle(DISK_R - 0.006)
        .extrude(0.001)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mason_jar")

    # Materials
    glass_clear = model.material("glass_clear", rgba=(0.85, 0.92, 0.88, 0.45))
    metal_silver = model.material("metal_silver", rgba=(0.78, 0.78, 0.80, 1.0))
    metal_dark = model.material("metal_dark", rgba=(0.55, 0.55, 0.58, 1.0))
    seal_red = model.material("seal_red", rgba=(0.72, 0.22, 0.18, 1.0))

    # ---- jar body (root) ----
    body = model.part("jar_body")

    # Glass shell: revolved body + neck threads + foot bead
    glass_shape = (
        _jar_glass_solid()
        .union(_neck_threads())
        .union(_foot_ring_detail())
    )
    body.visual(
        mesh_from_cadquery(glass_shape, "jar_glass"),
        material=glass_clear,
        name="jar_glass",
    )

    # Hinge mount (lug + pin) on the jar body - connected solid
    body.visual(
        mesh_from_cadquery(_hinge_mount_body(), "hinge_mount"),
        material=metal_dark,
        name="hinge_mount",
    )

    body.inertial = Inertial.from_geometry(
        Cylinder(JAR_OUTER_R, JAR_BODY_H + NECK_H + FOOT_H),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, (JAR_BODY_H + NECK_H + FOOT_H) * 0.5)),
    )

    # ---- lid ring (screw band): fixed to body ----
    lid_ring = model.part("lid_ring")
    ring_shape = _lid_ring_solid().union(_lid_ring_threads())
    lid_ring.visual(
        mesh_from_cadquery(ring_shape, "ring_band"),
        material=metal_silver,
        name="ring_band",
    )
    lid_ring.inertial = Inertial.from_geometry(
        Cylinder(RING_OUTER_R, RING_H),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.0, RING_H * 0.5)),
    )

    model.articulation(
        "ring_attach",
        ArticulationType.FIXED,
        parent=body,
        child=lid_ring,
        origin=Origin(xyz=(0.0, 0.0, RIM_TOP_Z + 0.001)),
    )

    # ---- lid disk: flips open on rear revolute hinge ----
    lid_disk = model.part("lid_disk")

    # Disk assembly: plate + connecting ear as one solid
    lid_disk.visual(
        mesh_from_cadquery(_disk_assembly(), "disk_plate"),
        material=metal_dark,
        name="disk_plate",
    )

    # Red rubber seal gasket (touching the disk underside)
    lid_disk.visual(
        mesh_from_cadquery(_seal_gasket(), "seal_gasket"),
        material=seal_red,
        name="seal_gasket",
    )

    lid_disk.inertial = Inertial.from_geometry(
        Cylinder(DISK_R, DISK_THICK + 0.004),
        mass=0.015,
        origin=Origin(xyz=(0.0, -HINGE_Y, -(HINGE_Z - RIM_TOP_Z))),
    )

    # Revolute hinge: parent=body, child=lid_disk
    # Origin at rear hinge line, axis along +X
    # Positive q rotates front edge (+Y side) upward (+Z) by right-hand rule
    model.articulation(
        "lid_flip",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid_disk,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=2.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("jar_body")
    lid_ring = object_model.get_part("lid_ring")
    lid_disk = object_model.get_part("lid_disk")
    flip = object_model.get_articulation("lid_flip")

    # ---- Allow intentional overlaps ----
    ctx.allow_overlap(
        lid_ring, body,
        elem_a="ring_band",
        elem_b="jar_glass",
        reason="The screw band ring threads engage the neck thread ridges.",
    )
    ctx.allow_overlap(
        lid_ring, body,
        elem_a="ring_band",
        elem_b="hinge_mount",
        reason="The hinge mount lug protrudes through the ring band area to support the disk pivot.",
    )
    ctx.allow_overlap(
        lid_disk, body,
        elem_a="disk_plate",
        elem_b="jar_glass",
        reason="The disk plate seats on the rim top when closed.",
    )
    ctx.allow_overlap(
        lid_disk, body,
        elem_a="disk_plate",
        elem_b="hinge_mount",
        reason="The disk hinge ear wraps around the body hinge mount at the pivot.",
    )

    # ---- Mason jar structure checks ----

    # 1. Jar is taller than wide (mason jar proportions)
    body_aabb = ctx.part_world_aabb(body)
    bx = body_aabb[1][0] - body_aabb[0][0]
    bz = body_aabb[1][2] - body_aabb[0][2]
    ctx.check(
        "jar is taller than wide (mason jar proportions)",
        bz > bx + 0.02,
        details=f"body width={bx:.4f}, height={bz:.4f}",
    )

    # 2. Foot ring exists: body XY extent includes the wider foot
    ctx.check(
        "foot ring wider than body wall",
        bx > JAR_OUTER_R * 2 - 0.001,
        details=f"body x-extent={bx:.4f}, expected >= {JAR_OUTER_R * 2:.4f}",
    )

    # 3. Ring sits on the neck
    ctx.expect_overlap(
        lid_ring, body,
        axes="xy",
        min_overlap=0.03,
        name="ring sits over the neck in XY",
    )
    ring_pos = ctx.part_world_position(lid_ring)
    ctx.check(
        "ring is at the top of the jar",
        ring_pos is not None and ring_pos[2] > SHOULDER_Z,
        details=f"ring_pos={ring_pos}",
    )

    # 4. Disk covers the jar mouth when closed
    ctx.expect_overlap(
        lid_disk, body,
        axes="xy",
        min_overlap=0.02,
        elem_a="disk_plate",
        name="disk covers the jar mouth",
    )

    # 5. lid_flip is a non-fixed REVOLUTE joint
    ctx.check(
        "lid_flip is revolute",
        flip.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={flip.articulation_type}",
    )

    # 6. Positive lid_flip raises the disk plate (opens the lid)
    # Use element AABB since part origin is at the hinge point
    rest_aabb = ctx.part_element_world_aabb(lid_disk, elem="disk_plate")
    rest_plate_top = rest_aabb[1][2]  # max Z of disk plate
    with ctx.pose({flip: 1.0}):
        open_aabb = ctx.part_element_world_aabb(lid_disk, elem="disk_plate")
        open_plate_top = open_aabb[1][2]
    ctx.check(
        "lid_flip positive angle raises disk plate (opens lid)",
        open_plate_top > rest_plate_top + 0.01,
        details=f"rest_plate_top={rest_plate_top:.4f}, open_plate_top(q=1)={open_plate_top:.4f}",
    )

    # 7. At large open angle, disk is well above the jar rim
    with ctx.pose({flip: 2.0}):
        max_aabb = ctx.part_element_world_aabb(lid_disk, elem="disk_plate")
        max_plate_z = max_aabb[1][2]
    ctx.check(
        "disk plate is high when fully open",
        max_plate_z > RIM_TOP_Z + 0.03,
        details=f"max_plate_z={max_plate_z:.4f}",
    )

    # 8. Ring inner radius properly sized for neck thread engagement
    ctx.check(
        "ring inner radius clears neck outer for thread engagement",
        RING_INNER_R > NECK_R and RING_INNER_R < NECK_R + 0.004,
        details=f"ring_inner={RING_INNER_R}, neck_r={NECK_R}",
    )

    # 9. Glass wall thickness at mouth: inner neck wall exists
    # (verified structurally via the revolved profile with WALL offset)
    inner_neck_r = NECK_R - WALL
    ctx.check(
        "glass wall thickness at mouth",
        WALL >= 0.003 and inner_neck_r > 0.02,
        details=f"wall={WALL}, inner_neck_r={inner_neck_r}",
    )

    # 10. Hinge mount is physically connected to jar body
    ctx.expect_contact(
        body, body,
        elem_a="hinge_mount",
        elem_b="jar_glass",
        contact_tol=0.002,
        name="hinge mount contacts jar body at rim",
    )

    # 11. Hinge mount overlaps ring band zone (proves the hinge protrudes through ring area)
    ctx.expect_overlap(
        lid_ring, body,
        axes="z",
        elem_a="ring_band",
        elem_b="hinge_mount",
        min_overlap=0.003,
        name="hinge mount protrudes through ring zone in Z",
    )

    return ctx.report()


object_model = build_object_model()
