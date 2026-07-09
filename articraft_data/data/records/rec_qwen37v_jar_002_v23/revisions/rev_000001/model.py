from __future__ import annotations

# Tall cylindrical glass storage jar with a clamp lid.
# Frame: jar stands on +XY ground plane, central axis along +Z (up).
#   - jar_body: tall hollow cylindrical glass shell with a wide mouth,
#     thickened rim at top, and small hinge knuckle bosses at the rear. (root)
#   - lid_ring: metal annular clamp ring with an inward capture lip that fits
#     over the jar rim. Hinged open/closed via ring_hinge (REVOLUTE).
#   - lid_disk: flat circular disk captured by the ring lip.
#     Spins freely via disk_spin (CONTINUOUS) relative to the ring.
#
# Articulation chain: body -> ring_hinge -> lid_ring -> disk_spin -> lid_disk

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
JAR_R = 0.040              # jar body outer radius (80 mm diameter)
WALL = 0.003               # glass wall thickness
JAR_BOTTOM = 0.0           # base on ground
JAR_BODY_TOP = 0.160       # top of the main cylinder body
RIM_R = 0.044              # rim outer radius (wider than body for flange)
RIM_INNER_R = JAR_R - WALL # rim inner opening (0.037)
RIM_BOTTOM = JAR_BODY_TOP  # rim starts at body top
RIM_TOP = 0.175            # top of the rim (mouth of jar)
RIM_HEIGHT = RIM_TOP - RIM_BOTTOM  # 0.015

# Hinge location: rear of jar (+Y side), at rim top
HINGE_Y = RIM_R            # at the outer edge of the rim
HINGE_Z = RIM_TOP          # at the mouth level
HINGE_KNUCKLE_R = 0.003    # hinge knuckle radius
HINGE_KNUCKLE_LEN = 0.014  # hinge knuckle length (along pin axis = X)

# Ring dimensions (in ring local frame, origin at hinge pin center)
RING_INNER_R = RIM_R + 0.001  # clears the rim (0.045)
RING_OUTER_R = 0.049          # ring outer radius
RING_BAND_H = 0.014           # ring band height
RING_LIP_H = 0.002            # inward capture lip thickness at bottom
RING_LIP_INNER = 0.034        # lip inner radius (captures the disk)
RING_LUG_W = 0.014            # hinge lug width (X direction)
RING_LUG_EXT = 0.006          # lug extension beyond ring outer (Y)

# Disk dimensions
DISK_R = 0.042               # disk radius (sits on lip, inside band)
DISK_HEIGHT = 0.003          # disk thickness
DISK_KNOB_R = 0.006          # small center knob radius
DISK_KNOB_H = 0.005          # knob height


def _jar_body_solid() -> cq.Workplane:
    """Tall hollow cylindrical glass jar with wide mouth, rim, and hinge bosses."""
    # Main cylinder body (solid, will be shelled)
    body = (
        cq.Workplane("XY")
        .circle(JAR_R)
        .extrude(JAR_BODY_TOP)
    )

    # Thickened rim/flange at the top
    rim = (
        cq.Workplane("XY")
        .workplane(offset=RIM_BOTTOM)
        .circle(RIM_R)
        .extrude(RIM_HEIGHT)
    )
    solid = body.union(rim)

    # Hollow interior: cut a cylinder from inside, open at top
    inner_r = JAR_R - WALL
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=WALL)  # leave bottom wall
        .circle(inner_r)
        .extrude(RIM_TOP - WALL + 0.002)  # cut through rim top (open mouth)
    )
    result = solid.cut(cavity)

    # Hinge knuckle bosses: two small cylinders bridging from the rim to outside
    # These are the fixed knuckle halves on the jar body at the rear (+Y)
    # Pin axis is along X (tangent to rim at the back)
    for x_offset in (-0.005, 0.005):
        knuckle = (
            cq.Workplane("YZ")
            .workplane(offset=x_offset)
            .center(HINGE_Y, HINGE_Z)
            .circle(HINGE_KNUCKLE_R)
            .extrude(HINGE_KNUCKLE_LEN * 0.5)
        )
        result = result.union(knuckle)

    return result


def _jar_body_mesh():
    return mesh_from_cadquery(_jar_body_solid(), "jar_glass")


def _lid_ring_solid() -> cq.Workplane:
    """Annular clamp ring with capture lip and hinge lug.
    
    Ring local origin is at the hinge pin center.
    Ring band is offset so it centers on the jar mouth when closed.
    """
    # Ring band centered at (0, -HINGE_Y, 0) from hinge pin (local origin)
    cx, cy = 0.0, -HINGE_Y

    # Annular band
    band_outer = (
        cq.Workplane("XY")
        .center(cx, cy)
        .circle(RING_OUTER_R)
        .extrude(RING_BAND_H)
    )
    band_bore = (
        cq.Workplane("XY")
        .center(cx, cy)
        .circle(RING_INNER_R)
        .extrude(RING_BAND_H)
    )
    band = band_outer.cut(band_bore)

    # Inward capture lip at bottom: annular shelf from lip_inner to band inner
    lip_outer = (
        cq.Workplane("XY")
        .center(cx, cy)
        .circle(RING_INNER_R)
        .extrude(RING_LIP_H)
    )
    lip_bore = (
        cq.Workplane("XY")
        .center(cx, cy)
        .circle(RING_LIP_INNER)
        .extrude(RING_LIP_H)
    )
    lip = lip_outer.cut(lip_bore)
    ring = band.union(lip)

    # Hinge lug: tab extending from ring outer toward hinge pin location (+Y side)
    lug = (
        cq.Workplane("XY")
        .center(cx, cy + RING_OUTER_R + RING_LUG_EXT * 0.5 - 0.001)
        .rect(RING_LUG_W, RING_LUG_EXT + 0.002)
        .extrude(RING_BAND_H)
    )
    # Lug hole for hinge pin (cylindrical cutout along X)
    lug_hole = (
        cq.Workplane("YZ")
        .workplane(offset=cx - RING_LUG_W * 0.5 - 0.001)
        .center(cy + RING_OUTER_R + RING_LUG_EXT - 0.001, RING_BAND_H * 0.5)
        .circle(HINGE_KNUCKLE_R * 0.8)
        .extrude(RING_LUG_W + 0.002)
    )
    ring = ring.union(lug).cut(lug_hole)

    return ring


def _lid_ring_mesh():
    return mesh_from_cadquery(_lid_ring_solid(), "ring_band")


def _lid_disk_solid() -> cq.Workplane:
    """Flat circular disk with a small center knob.
    
    Disk local origin at center bottom.
    """
    disk = (
        cq.Workplane("XY")
        .circle(DISK_R)
        .extrude(DISK_HEIGHT)
    )
    # Small raised knob in center for grip
    knob = (
        cq.Workplane("XY")
        .workplane(offset=DISK_HEIGHT)
        .circle(DISK_KNOB_R)
        .extrude(DISK_KNOB_H)
    )
    try:
        knob = knob.faces(">Z").edges().fillet(DISK_KNOB_R * 0.3)
    except Exception:
        pass
    return disk.union(knob)


def _lid_disk_mesh():
    return mesh_from_cadquery(_lid_disk_solid(), "disk_plate")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tall_cylindrical_clamp_jar")

    # Materials
    glass = model.material("clear_glass", rgba=(0.78, 0.85, 0.88, 0.30))
    steel = model.material("brushed_steel", rgba=(0.60, 0.63, 0.66, 1.0))
    disk_mat = model.material("frosted_glass", rgba=(0.88, 0.90, 0.92, 0.55))

    # ---- jar body (root): tall hollow cylinder with rim and hinge bosses ----
    body = model.part("jar_body")
    body.visual(_jar_body_mesh(), material=glass, name="jar_glass")
    body.inertial = Inertial.from_geometry(
        Cylinder(JAR_R, RIM_TOP),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, RIM_TOP / 2.0)),
    )

    # ---- lid_ring: clamp ring that hinges open/closed ----
    lid_ring = model.part("lid_ring")
    lid_ring.visual(_lid_ring_mesh(), material=steel, name="ring_band")
    lid_ring.inertial = Inertial.from_geometry(
        Cylinder(RING_OUTER_R, RING_BAND_H),
        mass=0.03,
        origin=Origin(xyz=(0.0, -HINGE_Y, RING_BAND_H / 2.0)),
    )

    # ---- lid_disk: flat disk captured in the ring, can spin ----
    lid_disk = model.part("lid_disk")
    lid_disk.visual(_lid_disk_mesh(), material=disk_mat, name="disk_plate")
    # Off-center marker so disk rotation is observable
    marker = CylinderGeometry(0.003, 0.004).translate(DISK_R * 0.65, 0.0, DISK_HEIGHT)
    lid_disk.visual(
        mesh_from_geometry(marker, "disk_marker"),
        material=steel,
        name="disk_marker",
    )
    lid_disk.inertial = Inertial.from_geometry(
        Cylinder(DISK_R, DISK_HEIGHT + DISK_KNOB_H),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.0, (DISK_HEIGHT + DISK_KNOB_H) / 2.0)),
    )

    # ---- ring_hinge: REVOLUTE, body -> lid_ring ----
    # Hinge pin at rear of jar rim (+Y), axis tangent to rim (along X).
    # At q=0 the ring is closed (flat on rim). Positive q opens the ring upward.
    # Ring part origin is at hinge point; ring geometry is offset -Y from there.
    model.articulation(
        "ring_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid_ring,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=2.2),
    )

    # ---- disk_spin: CONTINUOUS, lid_ring -> lid_disk ----
    # Disk sits on the ring capture lip, centered on the jar mouth.
    # In ring local frame at q=0, the jar center is at (0, -HINGE_Y, 0).
    # Disk sits on the lip: z = RING_LIP_H (on top of the lip shelf).
    model.articulation(
        "disk_spin",
        ArticulationType.CONTINUOUS,
        parent=lid_ring,
        child=lid_disk,
        origin=Origin(xyz=(0.0, -HINGE_Y, RING_LIP_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=3.0),
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
    hinge = object_model.get_articulation("ring_hinge")
    spin = object_model.get_articulation("disk_spin")

    # Allow ring to sit on/around the rim (capture fit)
    ctx.allow_overlap(
        ring,
        body,
        elem_a="ring_band",
        elem_b="jar_glass",
        reason="The clamp ring wraps around the jar rim as a capture fit.",
    )

    # Allow disk to sit on ring capture lip (intentional nesting)
    ctx.allow_overlap(
        ring,
        disk,
        elem_a="ring_band",
        elem_b="disk_plate",
        reason="The disk sits on the ring capture lip, intentionally nested inside the band.",
    )

    # --- jar body is cylindrical (round cross-section) and tall ---
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "jar body is round (x and y extents similar)",
        abs(bext[0] - bext[1]) < 0.020,
        details=f"x={bext[0]:.4f}, y={bext[1]:.4f}",
    )
    ctx.check(
        "jar is tall (height > 1.8x width)",
        bext[2] > 1.8 * min(bext[0], bext[1]),
        details=f"height={bext[2]:.4f}, min_width={min(bext[0], bext[1]):.4f}",
    )

    # --- wide mouth: opening is present (rim inner radius is substantial) ---
    ctx.check(
        "wide mouth opening exists (rim inner radius > 0.025m)",
        RIM_INNER_R > 0.025,
        details=f"rim_inner_r={RIM_INNER_R:.4f}",
    )

    # --- ring sits at the top of the jar in closed pose ---
    ring_aabb = ctx.part_world_aabb(ring)
    ctx.check(
        "ring sits near the top of the jar",
        ring_aabb is not None and ring_aabb[0][2] > JAR_BODY_TOP - 0.005,
        details=f"ring min z={ring_aabb[0][2] if ring_aabb else None}",
    )

    # --- ring overlaps the jar mouth footprint (seated on rim) ---
    ctx.expect_overlap(
        ring, body, axes="xy", min_overlap=0.02,
        name="ring seated over jar mouth footprint",
    )

    # --- disk sits on the ring lip inside the ring ---
    ctx.expect_within(
        disk, ring, axes="xy", margin=0.002,
        name="disk stays within ring footprint",
    )

    # --- ring_hinge is REVOLUTE and opens the ring upward ---
    ctx.check(
        "ring_hinge is revolute",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={hinge.articulation_type}",
    )

    # Check ring opening by measuring the ring_band element z-extent
    ring_rest_aabb = ctx.part_element_world_aabb(ring, elem="ring_band")
    ring_rest_max_z = ring_rest_aabb[1][2]
    with ctx.pose({hinge: 1.5}):
        ring_open_aabb = ctx.part_element_world_aabb(ring, elem="ring_band")
        ring_open_max_z = ring_open_aabb[1][2]
    ctx.check(
        "ring_hinge opens ring upward (positive q raises ring geometry)",
        ring_open_max_z > ring_rest_max_z + 0.02,
        details=f"rest max_z={ring_rest_max_z:.4f}, open max_z={ring_open_max_z:.4f}",
    )

    # --- disk_spin is CONTINUOUS about +Z ---
    ctx.check(
        "disk_spin is continuous about +Z",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and spin.axis == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )

    # --- disk rotation moves the off-center marker ---
    m0 = ctx.part_element_world_aabb(disk, elem="disk_marker")
    m0c = ((m0[0][0] + m0[1][0]) / 2.0, (m0[0][1] + m0[1][1]) / 2.0)
    with ctx.pose({spin: math.pi / 2.0}):
        m1 = ctx.part_element_world_aabb(disk, elem="disk_marker")
        m1c = ((m1[0][0] + m1[1][0]) / 2.0, (m1[0][1] + m1[1][1]) / 2.0)
    marker_shift = math.hypot(m1c[0] - m0c[0], m1c[1] - m0c[1])
    ctx.check(
        "disk_spin rotates the disk (marker moves)",
        marker_shift > 0.005,
        details=f"marker moved {marker_shift:.4f} m on quarter turn",
    )

    # --- disk moves with ring when ring opens (tilts, so check max_z) ---
    disk_rest_aabb = ctx.part_element_world_aabb(disk, elem="disk_plate")
    disk_rest_max_z = disk_rest_aabb[1][2]
    with ctx.pose({hinge: 1.5}):
        disk_open_aabb = ctx.part_element_world_aabb(disk, elem="disk_plate")
        disk_open_max_z = disk_open_aabb[1][2]
    ctx.check(
        "disk follows ring when hinge opens",
        disk_open_max_z > disk_rest_max_z + 0.02,
        details=f"disk rest max_z={disk_rest_max_z:.4f}, open max_z={disk_open_max_z:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
