from __future__ import annotations

# Square pantry jar with rounded corners, split lid (ring + disk).
# Frame: jar stands on +XY ground plane, central axis along +Z (up).
#   - jar_body: square-section clear glass shell with generously rounded vertical
#     edges, hollow inside, topped by a thick-walled mouth rim. (root)
#   - lid_ring: brass annular retaining ring that sits on the mouth rim.
#   - lid_disk: flat brass disk that sits inside the ring bore; can spin.
# Articulations:
#   ring_lift   (PRISMATIC, body->ring): lifts ring off the mouth vertically
#   disk_rotate (CONTINUOUS, ring->disk): disk spins inside the ring

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
BODY_HALF = 0.042          # half-width of the square section (0.084 square)
BODY_FILLET = 0.016        # rounded vertical-edge radius (generously rounded)
WALL = 0.0035              # glass wall thickness
BODY_Z0 = 0.0              # jar base sits on the ground
BODY_TOP = 0.100           # top of the main square body section
MOUTH_HEIGHT = 0.018       # height of the thickened mouth rim above body top
MOUTH_TOP = BODY_TOP + MOUTH_HEIGHT  # top of mouth rim
MOUTH_OUTER_HALF = BODY_HALF        # mouth outer profile matches body
MOUTH_INNER_HALF = BODY_HALF - WALL - 0.004  # wider inner bore for pantry access
MOUTH_RIM_THICK = 0.005    # extra radial thickness of the mouth rim

# Lid ring dimensions
RING_OD = 2.0 * (MOUTH_OUTER_HALF + MOUTH_RIM_THICK)  # ring outer spans mouth + rim
RING_OR = RING_OD / 2.0
RING_ID = 2.0 * MOUTH_INNER_HALF  # ring inner bore
RING_IR = RING_ID / 2.0
RING_HEIGHT = 0.010         # ring thickness

# Lid disk dimensions
DISK_R = RING_IR + 0.0002   # disk press-fits into ring bore (seated contact)
DISK_HEIGHT = 0.004         # disk thickness
DISK_KNOB_R = 0.005         # small central knob on the disk
DISK_KNOB_H = 0.008         # knob height

# Ring mount height: ring sits on top of mouth rim
RING_MOUNT_Z = MOUTH_TOP


def _body_solid() -> cq.Workplane:
    # Hollow square glass jar with rounded vertical edges and a thickened mouth.
    # The mouth is a wider-walled section at the top that shows glass thickness.
    outer = (
        cq.Workplane("XY")
        .box(2 * BODY_HALF, 2 * BODY_HALF, BODY_TOP, centered=(True, True, False))
        .edges("|Z")
        .fillet(BODY_FILLET)
    )

    # Mouth rim: a thick-walled square section at the top with visible wall thickness.
    # Outer profile matches body; the bore is narrower to create a thick rim.
    mouth_outer = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP)
        .box(2 * MOUTH_OUTER_HALF, 2 * MOUTH_OUTER_HALF, MOUTH_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(BODY_FILLET)
    )

    solid = outer.union(mouth_outer)

    # Hollow cavity: the main body is hollow, and the mouth opens with a thick rim.
    inner = (
        cq.Workplane("XY")
        .workplane(offset=WALL)
        .box(
            2 * (BODY_HALF - WALL),
            2 * (BODY_HALF - WALL),
            BODY_TOP - WALL,
            centered=(True, True, False),
        )
        .edges("|Z")
        .fillet(max(BODY_FILLET - WALL, 0.001))
    )
    # Mouth bore: narrower opening through the rim (shows wall thickness)
    mouth_bore = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP - 0.001)
        .box(
            2 * MOUTH_INNER_HALF,
            2 * MOUTH_INNER_HALF,
            MOUTH_HEIGHT + 0.002,
            centered=(True, True, False),
        )
        .edges("|Z")
        .fillet(max(BODY_FILLET - WALL - 0.004, 0.002))
    )
    cavity = inner.union(mouth_bore)
    return solid.cut(cavity)


def _body_mesh():
    return mesh_from_cadquery(_body_solid(), "jar_glass")


def _ring_solid() -> cq.Workplane:
    # Brass annular ring: outer circle minus inner bore, extruded.
    ring = (
        cq.Workplane("XY")
        .circle(RING_OR)
        .circle(RING_IR)
        .extrude(RING_HEIGHT)
    )
    # Add a subtle lip/groove on the outer edge for grip
    grip = (
        cq.Workplane("XY")
        .workplane(offset=RING_HEIGHT * 0.3)
        .circle(RING_OR + 0.001)
        .circle(RING_OR - 0.0005)
        .extrude(RING_HEIGHT * 0.4)
    )
    ring = ring.union(grip)
    # Internal seating ledge: thin annular lip at the ring bottom that supports
    # the disk. The ledge narrows the bore slightly at the bottom.
    ledge_width = 0.003  # ledge protrudes inward from ring bore
    ledge_ir = RING_IR - ledge_width
    ledge = (
        cq.Workplane("XY")
        .circle(RING_IR + 0.0001)
        .circle(ledge_ir)
        .extrude(0.002)
    )
    ring = ring.union(ledge)
    return ring


def _ring_mesh():
    return mesh_from_cadquery(_ring_solid(), "ring_brass")


def _disk_solid() -> cq.Workplane:
    # Flat brass disk with a small central knob for gripping.
    disk = (
        cq.Workplane("XY")
        .circle(DISK_R)
        .extrude(DISK_HEIGHT)
    )
    # Central knob
    knob = (
        cq.Workplane("XY")
        .workplane(offset=DISK_HEIGHT)
        .circle(DISK_KNOB_R)
        .extrude(DISK_KNOB_H)
    )
    # Slight dome on knob top
    try:
        knob = knob.faces(">Z").fillet(DISK_KNOB_R * 0.5)
    except Exception:
        pass
    disk = disk.union(knob)
    return disk


def _disk_mesh():
    return mesh_from_cadquery(_disk_solid(), "disk_brass")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="square_pantry_jar_split_lid")

    glass = model.material("clear_glass", rgba=(0.82, 0.86, 0.88, 0.25))
    brass = model.material("brass", rgba=(0.72, 0.55, 0.20, 1.0))
    brass_dark = model.material("brass_dark", rgba=(0.52, 0.38, 0.12, 1.0))

    # ---- jar body (root): square hollow glass shell with thick mouth rim ----
    body = model.part("jar_body")
    body.visual(_body_mesh(), material=glass, name="jar_glass")
    body.inertial = Inertial.from_geometry(
        Box((2 * BODY_HALF, 2 * BODY_HALF, MOUTH_TOP)),
        mass=0.32,
        origin=Origin(xyz=(0.0, 0.0, MOUTH_TOP / 2.0)),
    )

    # ---- lid ring: annular brass ring that sits on the mouth ----
    ring = model.part("lid_ring")
    ring.visual(_ring_mesh(), material=brass, name="ring_brass")
    ring.inertial = Inertial.from_geometry(
        Cylinder(RING_OR, RING_HEIGHT),
        mass=0.025,
        origin=Origin(xyz=(0.0, 0.0, RING_HEIGHT / 2.0)),
    )

    # ---- lid disk: flat brass disk inside the ring bore ----
    disk = model.part("lid_disk")
    disk.visual(_disk_mesh(), material=brass, name="disk_brass")
    # Off-axis marker on the disk so rotation is observable.
    marker = CylinderGeometry(0.002, 0.003).translate(DISK_R - 0.006, 0.0, DISK_HEIGHT)
    disk.visual(mesh_from_geometry(marker, "disk_marker"), material=brass_dark, name="disk_marker")
    disk.inertial = Inertial.from_geometry(
        Cylinder(DISK_R, DISK_HEIGHT + DISK_KNOB_H),
        mass=0.015,
        origin=Origin(xyz=(0.0, 0.0, (DISK_HEIGHT + DISK_KNOB_H) / 2.0)),
    )

    # ---- ring_lift: prismatic, body -> ring, lifts ring off mouth ----
    model.articulation(
        "ring_lift",
        ArticulationType.PRISMATIC,
        parent=body,
        child=ring,
        origin=Origin(xyz=(0.0, 0.0, RING_MOUNT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0, upper=RING_HEIGHT + 0.030, effort=2.0, velocity=1.0
        ),
    )

    # ---- disk_rotate: continuous, ring -> disk, spins disk inside ring ----
    # Disk sits on the internal ledge (2mm above ring bottom), contacts ring bore wall.
    model.articulation(
        "disk_rotate",
        ArticulationType.CONTINUOUS,
        parent=ring,
        child=disk,
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("jar_body")
    ring = object_model.get_part("lid_ring")
    disk = object_model.get_part("lid_disk")
    ring_lift = object_model.get_articulation("ring_lift")
    disk_rotate = object_model.get_articulation("disk_rotate")

    # The ring is seated on the mouth rim (capture fit, small overlap expected).
    ctx.allow_overlap(
        ring,
        body,
        elem_a="ring_brass",
        elem_b="jar_glass",
        reason="The brass ring sits on the thickened mouth rim as a seated capture fit.",
    )
    # The disk sits inside the ring bore (nested fit).
    ctx.allow_overlap(
        disk,
        ring,
        elem_a="disk_brass",
        elem_b="ring_brass",
        reason="The disk is intentionally nested inside the ring bore.",
    )

    # --- jar body is square with rounded corners, taller than wide ---
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "jar body is square in cross-section",
        abs(bext[0] - bext[1]) < 0.008,
        details=f"x={bext[0]:.4f}, y={bext[1]:.4f}",
    )
    ctx.check(
        "jar is taller than wide",
        bext[2] > bext[0] + 0.03,
        details=f"extents={bext}",
    )

    # --- mouth rim is present above the body (glass wall thickness visible) ---
    ctx.check(
        "mouth rim extends above body top",
        MOUTH_TOP > BODY_TOP + 0.010,
        details=f"mouth_top={MOUTH_TOP:.4f}, body_top={BODY_TOP:.4f}",
    )

    # --- ring is annular and seated on the mouth ---
    rext = _ext(ctx.part_world_aabb(ring))
    ctx.check(
        "ring is round in footprint",
        abs(rext[0] - rext[1]) < 0.004,
        details=f"ring x={rext[0]:.4f}, y={rext[1]:.4f}",
    )
    ring_pos = ctx.part_world_position(ring)
    ctx.check(
        "ring sits at the top of the jar mouth",
        ring_pos is not None and ring_pos[2] > BODY_TOP,
        details=f"ring z={ring_pos[2] if ring_pos else None}, body_top={BODY_TOP}",
    )

    # --- disk is inside the ring bore ---
    dext = _ext(ctx.part_world_aabb(disk))
    ctx.check(
        "disk fits inside ring (smaller footprint)",
        dext[0] < rext[0] - 0.004 and dext[1] < rext[1] - 0.004,
        details=f"disk x={dext[0]:.4f}, ring x={rext[0]:.4f}",
    )
    ctx.expect_within(
        disk, ring, axes="xy",
        inner_elem="disk_brass", outer_elem="ring_brass",
        margin=0.003,
        name="disk is within ring bore in XY",
    )

    # --- ring_lift: prismatic, lifts ring off mouth ---
    ctx.check(
        "ring_lift is prismatic along +Z",
        ring_lift.articulation_type == ArticulationType.PRISMATIC
        and ring_lift.axis == (0.0, 0.0, 1.0),
        details=f"type={ring_lift.articulation_type}, axis={ring_lift.axis}",
    )
    z_rest = ctx.part_world_position(ring)[2]
    with ctx.pose({ring_lift: 0.025}):
        z_lifted = ctx.part_world_position(ring)[2]
    ctx.check(
        "ring_lift lifts the ring upward",
        z_lifted > z_rest + 0.020,
        details=f"rest z={z_rest:.4f}, lifted z={z_lifted:.4f}",
    )

    # --- disk_rotate: continuous, spins disk inside ring ---
    ctx.check(
        "disk_rotate is continuous about +Z",
        disk_rotate.articulation_type == ArticulationType.CONTINUOUS
        and disk_rotate.axis == (0.0, 0.0, 1.0),
        details=f"type={disk_rotate.articulation_type}, axis={disk_rotate.axis}",
    )
    # Marker should move when disk rotates
    m0 = ctx.part_element_world_aabb(disk, elem="disk_marker")
    m0c = ((m0[0][0] + m0[1][0]) / 2.0, (m0[0][1] + m0[1][1]) / 2.0)
    with ctx.pose({disk_rotate: math.pi / 2.0}):
        m1 = ctx.part_element_world_aabb(disk, elem="disk_marker")
        m1c = ((m1[0][0] + m1[1][0]) / 2.0, (m1[0][1] + m1[1][1]) / 2.0)
    marker_shift = math.hypot(m1c[0] - m0c[0], m1c[1] - m0c[1])
    ctx.check(
        "disk_rotate spins the disk (marker moves)",
        marker_shift > 0.005,
        details=f"marker moved {marker_shift:.4f} m on a quarter turn",
    )

    # --- ring and disk are separate articulated parts ---
    ctx.check(
        "ring and disk are distinct parts",
        ring.name != disk.name,
        details=f"ring={ring.name}, disk={disk.name}",
    )

    return ctx.report()


object_model = build_object_model()
