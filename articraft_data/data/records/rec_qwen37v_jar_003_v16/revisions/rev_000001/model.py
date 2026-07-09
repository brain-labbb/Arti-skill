from __future__ import annotations

# Mason jar with two-piece flip lid (ring + disk).
# Frame: vertical axis +Z, jar centered on world origin, base on z=0.
#
# Parts:
#   jar_body (root): hollow glass jar with foot ring, rim seam/thread band,
#                    wide mouth; plus a fixed metal lid ring on the mouth.
#   lid_disk: flat metal disk hinged at rear of ring, flips open.
#
# Articulation:
#   ring_to_disk: REVOLUTE at rear of ring top, axis +X.
#     q=0  → disk closed (flat, covering mouth)
#     q>0  → front edge lifts upward (opens)
#     limits 0 … 2.0 rad (~115°)

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ──────────────────── key dimensions (metres) ────────────────────
JAR_OD = 0.085          # jar body outer diameter
JAR_R = JAR_OD / 2.0
GLASS_WALL = 0.004
JAR_ID = JAR_OD - 2.0 * GLASS_WALL   # mouth inner diameter
JAR_IR = JAR_ID / 2.0
JAR_H = 0.120          # total jar height (base to mouth top)

# Foot ring – wider ring at the base
FOOT_OD = 0.091
FOOT_R = FOOT_OD / 2.0
FOOT_H = 0.006

# Rim seam – wider thread band near the top
RIM_OD = 0.093
RIM_R = RIM_OD / 2.0
RIM_H = 0.020
RIM_BOTTOM_Z = JAR_H - RIM_H         # 0.100

# Two-piece lid: ring (fixed on jar) + disk (flips)
RING_OD = 0.092
RING_R = RING_OD / 2.0
RING_ID = JAR_ID + 0.002             # slightly larger than mouth
RING_IR = RING_ID / 2.0
RING_H = 0.010

DISK_OD = JAR_ID - 0.002             # fits inside mouth with 1 mm clearance
DISK_R = DISK_OD / 2.0
DISK_H = 0.002

# Hinge location – rear edge of disk, top of ring
HINGE_Y = -DISK_R                    # rear of disk
HINGE_Z = JAR_H + RING_H            # top of ring


# ──────────────────── CadQuery geometry builders ─────────────────

def _jar_glass() -> cq.Workplane:
    """Hollow glass mason jar with foot ring and rim seam."""
    # 1. Foot ring – solid wider base
    foot = (
        cq.Workplane("XY")
        .circle(FOOT_R)
        .extrude(FOOT_H)
    )
    # 2. Main cylindrical body (from foot top to rim bottom)
    body_cyl = (
        cq.Workplane("XY")
        .workplane(offset=FOOT_H)
        .circle(JAR_R)
        .extrude(RIM_BOTTOM_Z - FOOT_H)
    )
    # 3. Rim seam / thread band (wider at top)
    rim = (
        cq.Workplane("XY")
        .workplane(offset=RIM_BOTTOM_Z)
        .circle(RIM_R)
        .extrude(RIM_H)
    )
    jar = foot.union(body_cyl).union(rim)

    # 4. Cut interior cavity – open at top, solid glass base
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=GLASS_WALL)   # leave a solid glass floor
        .circle(JAR_IR)
        .extrude(JAR_H + 0.005)         # over-extrude to guarantee open mouth
    )
    jar = jar.cut(cavity)

    # 5. Small sealing ledge inside the mouth (makes wall thickness visible)
    #    A thin inward-facing ring near the top of the cavity.
    ledge = (
        cq.Workplane("XY")
        .workplane(offset=JAR_H - 0.005)
        .circle(JAR_IR)
        .circle(JAR_IR - 0.002)         # 2 mm inward step
        .extrude(0.003)
    )
    jar = jar.union(ledge)

    return jar


def _lid_ring() -> cq.Workplane:
    """Annular metal ring that sits on the jar mouth."""
    return (
        cq.Workplane("XY")
        .circle(RING_R)
        .circle(RING_IR)
        .extrude(RING_H)
    )


def _lid_disk() -> cq.Workplane:
    """Flat metal disk, centred at local origin for easy hinge offset."""
    return (
        cq.Workplane("XY")
        .circle(DISK_R)
        .extrude(DISK_H)
        .translate((0.0, 0.0, -DISK_H / 2.0))   # centre at z = 0
    )


# ──────────────────── model assembly ─────────────────────────────

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mason_jar_flip_lid")

    glass = model.material("clear_glass", rgba=(0.80, 0.85, 0.88, 0.30))
    metal = model.material("brushed_aluminum", rgba=(0.72, 0.72, 0.76, 1.0))

    # ── jar_body (root) ──────────────────────────────────────────
    jar_body = model.part("jar_body")

    jar_body.visual(
        mesh_from_cadquery(_jar_glass(), "jar_glass"),
        material=glass,
        name="jar_glass",
    )
    # Metal ring fixed on the mouth
    jar_body.visual(
        mesh_from_cadquery(_lid_ring(), "lid_ring"),
        origin=Origin(xyz=(0.0, 0.0, JAR_H)),
        material=metal,
        name="lid_ring",
    )

    jar_body.inertial = Inertial.from_geometry(
        Cylinder(radius=JAR_R, length=JAR_H),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, JAR_H / 2.0)),
    )

    # ── lid_disk ─────────────────────────────────────────────────
    lid_disk = model.part("lid_disk")

    # In the disk part frame (which coincides with the articulation
    # frame at q = 0), the hinge is at the origin.  The disk mesh is
    # centred at local z = 0, so we offset it forward (+Y) by DISK_R
    # and down so it lands on the jar mouth.
    disk_y_off = DISK_R
    # Seat the disk on the sealing ledge (ledge top at JAR_H - 0.002).
    # Disk bottom lands at JAR_H - 0.002, top flush with mouth.
    disk_z_off = (JAR_H - 0.002 + DISK_H / 2.0) - HINGE_Z

    lid_disk.visual(
        mesh_from_cadquery(_lid_disk(), "disk_panel"),
        origin=Origin(xyz=(0.0, disk_y_off, disk_z_off)),
        material=metal,
        name="disk_panel",
    )
    lid_disk.inertial = Inertial.from_geometry(
        Cylinder(radius=DISK_R, length=DISK_H),
        mass=0.012,
        origin=Origin(xyz=(0.0, disk_y_off, disk_z_off)),
    )

    # ── articulation: rear revolute hinge ────────────────────────
    model.articulation(
        "ring_to_disk",
        ArticulationType.REVOLUTE,
        parent=jar_body,
        child=lid_disk,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),          # +X → positive q lifts front edge
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0, lower=0.0, upper=2.0,
        ),
    )

    return model


# ──────────────────── tests ──────────────────────────────────────

def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    jar_body = object_model.get_part("jar_body")
    lid_disk = object_model.get_part("lid_disk")
    hinge = object_model.get_articulation("ring_to_disk")

    # ── jar proportions: roughly cylindrical, similar X/Y extents ──
    jar_ext = _ext(ctx.part_world_aabb(jar_body))
    ctx.check(
        "jar section is roughly circular",
        abs(jar_ext[0] - jar_ext[1]) < 0.010,
        details=f"jar extents XY = ({jar_ext[0]:.4f}, {jar_ext[1]:.4f})",
    )

    # ── two-piece lid: ring visual on jar_body, disk on lid_disk ──
    ctx.check(
        "jar_body carries lid_ring visual (two-piece lid ring)",
        jar_body.get_visual("lid_ring") is not None,
    )
    ctx.check(
        "lid_disk carries disk_panel visual",
        lid_disk.get_visual("disk_panel") is not None,
    )

    # ── hinge is revolute (non-fixed joint) ─────────────────────
    ctx.check(
        "ring_to_disk is REVOLUTE",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type = {hinge.articulation_type}",
    )

    # ── foot ring is wider than body ────────────────────────────
    # jar_glass AABB should be wider at the bottom (foot ring)
    # Check that the XY extent exceeds the body diameter by ≥ foot_proud
    foot_proud = (FOOT_OD - JAR_OD) / 2.0   # 3 mm
    ctx.check(
        "foot ring extends beyond jar body",
        max(jar_ext[0], jar_ext[1]) >= JAR_OD + foot_proud - 0.001,
        details=f"jar max XY = {max(jar_ext[0], jar_ext[1]):.4f}, expected >= {JAR_OD + foot_proud:.4f}",
    )

    # ── rim seam wider than body ────────────────────────────────
    rim_proud = (RIM_OD - JAR_OD) / 2.0
    ctx.check(
        "rim seam extends beyond jar body",
        max(jar_ext[0], jar_ext[1]) >= JAR_OD + rim_proud - 0.001,
        details=f"jar max XY = {max(jar_ext[0], jar_ext[1]):.4f}",
    )

    # ── disk overlaps jar mouth when closed (q = 0) ─────────────
    ctx.expect_overlap(
        lid_disk, jar_body,
        axes="xy",
        min_overlap=0.020,
        name="closed disk covers jar mouth in XY",
    )

    # ── disk flips open: front edge rises at positive angle ─────
    # Part origin is on the hinge axis and does not move with
    # revolute rotation, so use the AABB centre instead.
    rest_aabb = ctx.part_world_aabb(lid_disk)
    rest_z = (rest_aabb[0][2] + rest_aabb[1][2]) / 2.0
    with ctx.pose({hinge: 1.5}):
        open_aabb = ctx.part_world_aabb(lid_disk)
        open_z = (open_aabb[0][2] + open_aabb[1][2]) / 2.0
    ctx.check(
        "disk flips open (AABB center rises at q=1.5 rad)",
        open_z > rest_z + 0.01,
        details=f"rest_z={rest_z:.4f}, open_z={open_z:.4f}",
    )

    # ── materials: glass and metal are distinct ─────────────────
    glass_mat = jar_body.get_visual("jar_glass").material
    metal_mat = jar_body.get_visual("lid_ring").material
    ctx.check(
        "glass and metal materials are distinct",
        glass_mat is not None
        and metal_mat is not None
        and glass_mat.name != metal_mat.name,
        details=f"glass={getattr(glass_mat, 'name', None)}, metal={getattr(metal_mat, 'name', None)}",
    )

    # ── hinge motion limits are sensible ────────────────────────
    ctx.check(
        "hinge lower limit is 0 (closed)",
        hinge.motion_limits.lower == 0.0,
    )
    ctx.check(
        "hinge upper limit allows full open (>1.5 rad)",
        hinge.motion_limits.upper >= 1.5,
    )

    return ctx.report()


object_model = build_object_model()
