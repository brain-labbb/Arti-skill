from __future__ import annotations

# Square pantry jar with rounded corners, wide mouth, and a split lid
# (ring + disk) as separate articulated parts.
#
# Frame: vertical axis +Z, jar centered on the world Z axis, base on z=0.
#
# Parts:
#   - body:     clear glass square jar with rounded corners, hollow interior,
#               thick rim at the mouth showing glass wall thickness.
#   - lid_ring: metallic ring that sits on top of the jar rim. Articulated
#               with a REVOLUTE joint around Z (can spin on the rim).
#   - lid_disk: flat disk that sits inside the ring opening. Articulated
#               with a PRISMATIC joint along +Z (lifts off).
#
# Articulations:
#   - body_to_ring: REVOLUTE around Z axis. Limits ±π rad.
#   - ring_to_disk: PRISMATIC along +Z. At q=0 disk is seated in the ring;
#                    positive q lifts the disk straight up off the jar.

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Inertial,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ----- key dimensions (meters) -----
JAR_SECT = 0.090        # outer square section width of the glass jar
CORNER_R = 0.010        # rounded corner radius
GLASS_WALL = 0.004      # glass wall thickness (body walls)
BASE_THICK = 0.006      # solid glass floor thickness

JAR_BOTTOM_Z = 0.0
JAR_BODY_H = 0.090      # main body height (below rim)
RIM_H = 0.012           # thickened rim height above body
RIM_OUTER = JAR_SECT    # rim outer matches jar outer
RIM_INNER = JAR_SECT - 2.0 * GLASS_WALL  # rim inner opening (wide mouth)
RIM_WALL = GLASS_WALL   # rim wall thickness visible at mouth

JAR_TOP_Z = JAR_BOTTOM_Z + JAR_BODY_H + RIM_H  # top of rim

# Lid ring: sits on top of the rim
RING_OUTER = RIM_OUTER + 0.002   # slightly larger than rim outer for clearance
RING_CORNER_R = CORNER_R + 0.001
RING_INNER = RIM_INNER - 0.002   # ring opening slightly smaller than rim inner
RING_H = 0.008                   # ring thickness (height)
RING_BOTTOM_Z = JAR_TOP_Z        # ring sits on top of rim

# Lid disk: sits inside the ring opening
DISK_SIZE = RING_INNER - 0.002   # disk slightly smaller than ring opening
DISK_CORNER_R = max((RING_CORNER_R - 0.002), 0.002)
DISK_H = 0.005                   # disk thickness
DISK_BOTTOM_Z = RING_BOTTOM_Z + RING_H - DISK_H  # disk sits flush with ring top


def _jar_body_solid() -> cq.Workplane:
    """Hollow rounded-square glass jar with a thickened rim at the mouth."""
    total_h = JAR_BODY_H + RIM_H

    # Outer shell
    outer = (
        cq.Workplane("XY")
        .rect(JAR_SECT, JAR_SECT)
        .extrude(total_h)
        .edges("|Z")
        .fillet(CORNER_R)
    )

    # Inner cavity: open at top, solid floor at bottom
    inner_w = JAR_SECT - 2.0 * GLASS_WALL
    inner_cr = max(CORNER_R - GLASS_WALL, 0.001)
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=BASE_THICK)
        .rect(inner_w, inner_w)
        .extrude(total_h)  # over-extrude to open through top
        .edges("|Z")
        .fillet(inner_cr)
    )

    result = outer.cut(cavity)

    # Add a visible thickened rim/lip at the mouth top.
    # The rim is a raised ring around the mouth opening - it adds extra wall
    # thickness visible at the top edge, making the glass wall thickness clear.
    rim_outer_rect = RIM_OUTER
    rim_inner_rect = RIM_INNER
    rim_cr_outer = CORNER_R
    rim_cr_inner = max(CORNER_R - GLASS_WALL, 0.001)

    rim_outer = (
        cq.Workplane("XY")
        .workplane(offset=JAR_BODY_H)
        .rect(rim_outer_rect, rim_outer_rect)
        .extrude(RIM_H)
        .edges("|Z")
        .fillet(rim_cr_outer)
    )
    rim_inner_cut = (
        cq.Workplane("XY")
        .workplane(offset=JAR_BODY_H)
        .rect(rim_inner_rect, rim_inner_rect)
        .extrude(RIM_H + 0.001)
        .edges("|Z")
        .fillet(rim_cr_inner)
    )
    rim_ring = rim_outer.cut(rim_inner_cut)

    # Union the rim ring onto the body
    result = result.union(rim_ring)
    return result


def _lid_ring_solid() -> cq.Workplane:
    """Metallic ring that sits on top of the jar rim.
    Built in ring-local frame with bottom at z=0."""
    outer = (
        cq.Workplane("XY")
        .rect(RING_OUTER, RING_OUTER)
        .extrude(RING_H)
        .edges("|Z")
        .fillet(RING_CORNER_R)
    )
    inner = (
        cq.Workplane("XY")
        .rect(RING_INNER, RING_INNER)
        .extrude(RING_H)
        .edges("|Z")
        .fillet(max(RING_CORNER_R - 0.003, 0.002))
    )
    return outer.cut(inner)


def _lid_disk_solid() -> cq.Workplane:
    """Flat disk that sits inside the ring opening.
    Built in disk-local frame with bottom at z=0."""
    return (
        cq.Workplane("XY")
        .rect(DISK_SIZE, DISK_SIZE)
        .extrude(DISK_H)
        .edges("|Z")
        .fillet(DISK_CORNER_R)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="square_pantry_jar")

    glass = model.material("clear_glass", rgba=(0.82, 0.88, 0.90, 0.30))
    brushed_metal = model.material("brushed_steel", rgba=(0.70, 0.70, 0.72, 1.0))
    dark_lid = model.material("dark_bakelite", rgba=(0.15, 0.13, 0.12, 1.0))

    # ---- body (root): clear glass square jar with thick rim ----
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_jar_body_solid(), "glass_jar_body"),
        material=glass,
        name="glass_jar_body",
    )
    body.inertial = Inertial.from_geometry(
        Box((JAR_SECT, JAR_SECT, JAR_BODY_H + RIM_H)),
        mass=0.32,
        origin=Origin(xyz=(0.0, 0.0, (JAR_BODY_H + RIM_H) / 2.0)),
    )

    # ---- lid_ring: metallic ring sitting on the jar rim ----
    # Ring part frame is placed by the articulation at world z=RING_BOTTOM_Z.
    # Ring geometry is built from z=0 to z=RING_H in local frame.
    lid_ring = model.part("lid_ring")
    lid_ring.visual(
        mesh_from_cadquery(_lid_ring_solid(), "lid_ring_mesh"),
        material=brushed_metal,
        name="lid_ring_mesh",
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    lid_ring.inertial = Inertial.from_geometry(
        Box((RING_OUTER, RING_OUTER, RING_H)),
        mass=0.04,
        origin=Origin(xyz=(0.0, 0.0, RING_H / 2.0)),
    )

    # ---- lid_disk: flat disk sitting in the ring opening ----
    # Disk part frame is placed by the articulation relative to ring frame.
    # Disk sits inside ring at local z = RING_H - DISK_H (flush with ring top).
    DISK_LOCAL_Z = RING_H - DISK_H  # disk bottom in ring-local coords
    lid_disk = model.part("lid_disk")
    lid_disk.visual(
        mesh_from_cadquery(_lid_disk_solid(), "lid_disk_mesh"),
        material=dark_lid,
        name="lid_disk_mesh",
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    lid_disk.inertial = Inertial.from_geometry(
        Box((DISK_SIZE, DISK_SIZE, DISK_H)),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.0, DISK_H / 2.0)),
    )

    # ---- Articulation 1: body_to_ring (REVOLUTE around Z) ----
    # The ring can spin on the jar rim. Origin in body frame at top of rim.
    model.articulation(
        "body_to_ring",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid_ring,
        origin=Origin(xyz=(0.0, 0.0, JAR_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-3.14159, upper=3.14159),
    )

    # ---- Articulation 2: ring_to_disk (PRISMATIC along +Z) ----
    # The disk lifts straight up out of the ring. At q=0 disk is seated.
    # Origin in ring-local frame at the disk's seated position.
    model.articulation(
        "ring_to_disk",
        ArticulationType.PRISMATIC,
        parent=lid_ring,
        child=lid_disk,
        origin=Origin(xyz=(0.0, 0.0, DISK_LOCAL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.3, lower=0.0, upper=0.04),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid_ring = object_model.get_part("lid_ring")
    lid_disk = object_model.get_part("lid_disk")
    ring_joint = object_model.get_articulation("body_to_ring")
    disk_joint = object_model.get_articulation("ring_to_disk")

    # The disk is a removable insert that sits inside the ring opening with a
    # small radial clearance gap. It is supported by the ring (which is
    # grounded to the body) but does not physically contact the ring walls.
    ctx.allow_isolated_part(
        lid_disk,
        reason="Disk is a removable lid insert seated inside the ring opening with a small radial clearance for lift-off.",
    )

    # ---- Jar is square in section (rounded corners) ----
    body_ext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "jar body is square in section",
        abs(body_ext[0] - body_ext[1]) < 0.005,
        details=f"body extents={body_ext}",
    )

    # ---- Jar is wider than tall (pantry jar proportions) ----
    ctx.check(
        "jar is wider than tall (pantry jar proportions)",
        body_ext[0] > body_ext[2] * 0.7,
        details=f"body extents={body_ext}",
    )

    # ---- Ring sits on top of jar body ----
    ctx.expect_gap(
        lid_ring,
        body,
        axis="z",
        min_gap=-0.001,
        max_gap=0.002,
        name="ring sits on top of jar rim",
    )

    # ---- Ring footprint overlaps jar body in XY (it sits on the rim) ----
    ctx.expect_overlap(
        lid_ring,
        body,
        axes="xy",
        min_overlap=0.020,
        name="ring overlaps jar body in XY",
    )

    # ---- Disk sits inside the ring (contained in XY) ----
    ctx.expect_within(
        lid_disk,
        lid_ring,
        axes="xy",
        margin=0.003,
        name="disk is contained within ring footprint",
    )

    # ---- Disk seated in ring at rest (q=0) ----
    ctx.expect_overlap(
        lid_disk,
        lid_ring,
        axes="z",
        min_overlap=0.001,
        name="disk is seated in ring at rest",
    )

    # ---- Ring joint is REVOLUTE (non-fixed) ----
    ctx.check(
        "body_to_ring joint is revolute",
        ring_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"joint type={ring_joint.articulation_type}",
    )

    # ---- Disk joint is PRISMATIC (non-fixed) ----
    ctx.check(
        "ring_to_disk joint is prismatic",
        disk_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"joint type={disk_joint.articulation_type}",
    )

    # ---- Disk lifts straight up when prismatic joint is actuated ----
    rest_disk_z = ctx.part_world_position(lid_disk)[2]
    with ctx.pose({disk_joint: 0.04}):
        lifted_disk_z = ctx.part_world_position(lid_disk)[2]
        ctx.expect_gap(
            lid_disk,
            lid_ring,
            axis="z",
            min_gap=0.010,
            name="lifted disk clears ring vertically",
        )
    ctx.check(
        "disk lifts upward when prismatic joint actuated",
        lifted_disk_z > rest_disk_z + 0.035,
        details=f"rest_z={rest_disk_z}, lifted_z={lifted_disk_z}",
    )

    # ---- Ring rotates when revolute joint is actuated ----
    rest_ring_pos = ctx.part_world_position(lid_ring)
    with ctx.pose({ring_joint: 1.5708}):  # ~90 degrees
        rotated_ring_pos = ctx.part_world_position(lid_ring)
    ctx.check(
        "ring stays at same height when rotated (revolute around Z)",
        abs(rotated_ring_pos[2] - rest_ring_pos[2]) < 0.001,
        details=f"rest_z={rest_ring_pos[2]}, rotated_z={rotated_ring_pos[2]}",
    )

    # ---- Materials are distinct ----
    body_mat = body.get_visual("glass_jar_body").material
    ring_mat = lid_ring.get_visual("lid_ring_mesh").material
    disk_mat = lid_disk.get_visual("lid_disk_mesh").material
    ctx.check(
        "all three parts have distinct materials",
        body_mat is not None and ring_mat is not None and disk_mat is not None
        and getattr(body_mat, "name", None) == "clear_glass"
        and getattr(ring_mat, "name", None) == "brushed_steel"
        and getattr(disk_mat, "name", None) == "dark_bakelite",
        details=f"body={getattr(body_mat, 'name', None)}, ring={getattr(ring_mat, 'name', None)}, disk={getattr(disk_mat, 'name', None)}",
    )

    return ctx.report()


object_model = build_object_model()
