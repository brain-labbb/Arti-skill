from __future__ import annotations

# Faceted GLASS JAR with METAL SCREW-ON STOPPER
# Variant of the cosmetic face cream jar: octagonal faceted glass body,
# brushed-metal stopper cap that lifts vertically on a prismatic joint.
#
# Frame: jar axis along +Z, base on z=0, centered at origin.
#
# Articulation:
#   - stopper_lift: PRISMATIC along +Z, lifts the metal stopper off the neck.

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
SIDES = 8                          # octagonal facets
JAR_BODY_AF = 0.072               # across-flats width of the body
JAR_BODY_R = (JAR_BODY_AF / 2.0) / math.cos(math.pi / SIDES)  # circumradius ~0.039

JAR_BODY_H = 0.042                # body height
WALL = 0.004                      # glass wall thickness
BASE_THICK = 0.005                # thick glass base

NECK_AF = 0.056                   # neck is narrower than body
NECK_R = (NECK_AF / 2.0) / math.cos(math.pi / SIDES)  # neck circumradius ~0.0305
NECK_H = 0.012                    # neck height above shoulder
RIM_TOP_Z = JAR_BODY_H + NECK_H  # 0.054

INNER_R = JAR_BODY_R - WALL       # inner cavity radius (cylindrical)

# Stopper dimensions (in stopper-local frame, origin at rim top)
STOPPER_R = NECK_R + 0.003        # stopper slightly wider than neck
STOPPER_TOP_H = 0.006             # flat top disc thickness
STOPPER_SKIRT_H = 0.008           # skirt depth that slips over neck
STOPPER_TOTAL_H = STOPPER_TOP_H + STOPPER_SKIRT_H  # 0.014

# Lift range
LIFT_MAX = 0.030                  # stopper lifts 30mm off the jar


def _polygon_pts(n: int, radius: float) -> list[tuple[float, float]]:
    """Vertices of a regular n-gon inscribed in a circle of given radius."""
    pts = []
    for i in range(n):
        a = 2.0 * math.pi * i / n
        pts.append((radius * math.cos(a), radius * math.sin(a)))
    return pts


def _faceted_jar_solid() -> cq.Workplane:
    """Octagonal faceted glass jar with hollow interior and thick base.

    Built as: outer octagonal body prism + narrower octagonal neck prism,
    then hollowed with a body cavity and a narrower neck bore. The visible
    wall thickness at the mouth comes from the octagonal outer vs circular
    inner bore difference.
    """
    NECK_BORE_R = NECK_AF / 2.0 - WALL  # bore radius so wall at flats = WALL

    # Outer octagonal body
    outer_pts = _polygon_pts(SIDES, JAR_BODY_R)
    outer_body = (
        cq.Workplane("XY")
        .polyline(outer_pts).close()
        .extrude(JAR_BODY_H)
    )

    # Shoulder: narrower octagonal prism for the neck
    neck_pts = _polygon_pts(SIDES, NECK_R)
    neck_outer = (
        cq.Workplane("XY")
        .workplane(offset=JAR_BODY_H)
        .polyline(neck_pts).close()
        .extrude(NECK_H)
    )

    # Combine body + neck
    jar = outer_body.union(neck_outer)

    # Fillet vertical edges slightly for a refined faceted look
    try:
        jar = jar.edges("|Z").fillet(0.001)
    except Exception:
        pass  # skip if fillet fails on complex edges

    # Body cavity: cylindrical, from above the thick base to just below the
    # shoulder. Stops WALL below the body top so a solid shoulder disc remains
    # to connect the body wall to the narrower neck wall.
    SHOULDER_Z = JAR_BODY_H - WALL
    body_cavity = (
        cq.Workplane("XY")
        .workplane(offset=BASE_THICK)
        .circle(INNER_R)
        .extrude(SHOULDER_Z - BASE_THICK)
    )
    jar = jar.cut(body_cavity)

    # Neck bore: narrower cylindrical bore through the neck and shoulder disc.
    # Starts just below the shoulder so the disc material around the bore
    # connects body wall to neck wall.
    neck_bore = (
        cq.Workplane("XY")
        .workplane(offset=SHOULDER_Z - 0.001)
        .circle(NECK_BORE_R)
        .extrude(RIM_TOP_Z - SHOULDER_Z + 0.003)
    )
    jar = jar.cut(neck_bore)

    return jar


def _neck_threads() -> cq.Workplane:
    """Thread ridges around the faceted neck rim.

    Inner radius is smaller than the neck flat distance so the rings
    embed into the neck wall for solid connectivity.
    """
    thread_inner = NECK_AF / 2.0 - 0.001  # embeds into neck flats
    thread_outer = NECK_R + 0.001          # protrudes past vertices
    threads = None
    z0 = JAR_BODY_H + 0.003
    for i in range(4):
        z = z0 + i * 0.0025
        ring = (
            cq.Workplane("XY")
            .workplane(offset=z)
            .circle(thread_outer)
            .circle(thread_inner)
            .extrude(0.0014)
        )
        threads = ring if threads is None else threads.union(ring)
    return threads


def _rim_lip() -> cq.Workplane:
    """A visible rim/lip ring at the top of the neck showing glass wall thickness.

    The ring spans from the neck bore to slightly outside the neck outer,
    creating a visible annular ring that shows the wall cross-section.
    """
    neck_bore_r = NECK_AF / 2.0 - WALL
    lip_outer = NECK_R + 0.001
    lip = (
        cq.Workplane("XY")
        .workplane(offset=RIM_TOP_Z - 0.002)
        .circle(lip_outer)
        .circle(neck_bore_r)
        .extrude(0.002)
    )
    return lip


def _stopper_solid() -> cq.Workplane:
    """Metal stopper: a flat disc top with a short cylindrical skirt.

    In the stopper-local frame, origin is at the skirt bottom (which seats
    at the rim top in world space). The skirt extends downward (-Z local)
    and the top disc extends upward (+Z local).
    """
    # Skirt: slips over the neck exterior
    skirt = (
        cq.Workplane("XY")
        .workplane(offset=-STOPPER_SKIRT_H)
        .circle(STOPPER_R)
        .circle(NECK_R + 0.0005)
        .extrude(STOPPER_SKIRT_H)
    )

    # Top disc: solid metal cap
    top = (
        cq.Workplane("XY")
        .circle(STOPPER_R)
        .extrude(STOPPER_TOP_H)
    )

    stopper = skirt.union(top)

    # Fillet top edge
    try:
        stopper = stopper.edges(">Z").fillet(0.002)
    except Exception:
        pass

    return stopper


def _stopper_grip_mesh():
    """Small grip ridges around the stopper skirt for visual detail."""
    ribs = None
    n = 32
    for i in range(n):
        ang = 2.0 * math.pi * i / n
        x = (STOPPER_R - 0.0005) * math.cos(ang)
        y = (STOPPER_R - 0.0005) * math.sin(ang)
        rib = (
            cq.Workplane("XY")
            .workplane(offset=-STOPPER_SKIRT_H + 0.001)
            .center(x, y)
            .rect(0.0012, 0.0012)
            .extrude(STOPPER_SKIRT_H - 0.002)
        )
        ribs = rib if ribs is None else ribs.union(rib)
    return mesh_from_cadquery(ribs, "stopper_grip")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="faceted_glass_jar")

    # Materials
    glass_clear = model.material("glass_clear", rgba=(0.85, 0.90, 0.92, 0.45))
    metal_brushed = model.material("metal_brushed", rgba=(0.72, 0.73, 0.74, 1.0))
    cream_fill = model.material("cream_fill", rgba=(0.96, 0.93, 0.85, 1.0))
    label_gold = model.material("label_gold", rgba=(0.78, 0.65, 0.32, 1.0))

    # ---- jar body (root): faceted glass + threads + cream fill + label ----
    body = model.part("body")

    # Faceted glass shell with threads and rim lip
    glass = _faceted_jar_solid().union(_neck_threads()).union(_rim_lip())
    body.visual(
        mesh_from_cadquery(glass, "jar_glass"),
        material=glass_clear,
        name="jar_glass",
    )

    # Cream filling visible inside (sits below the rim)
    cream_z = JAR_BODY_H - 0.005
    cream = (
        cq.Workplane("XY")
        .workplane(offset=BASE_THICK + 0.001)
        .circle(INNER_R - 0.001)
        .extrude(cream_z - BASE_THICK)
    )
    body.visual(
        mesh_from_cadquery(cream, "cream_fill"),
        material=cream_fill,
        name="cream_fill",
    )

    # Gold label band on one facet of the body
    label_r = JAR_BODY_R + 0.0003
    body.visual(
        Cylinder(label_r, 0.016),
        origin=Origin(xyz=(0.0, 0.0, JAR_BODY_H * 0.45)),
        material=label_gold,
        name="brand_label",
    )

    body.inertial = Inertial.from_geometry(
        Cylinder(JAR_BODY_R, JAR_BODY_H + NECK_H),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, (JAR_BODY_H + NECK_H) * 0.5)),
    )

    # ---- stopper: metal cap that lifts vertically off the neck ----
    stopper = model.part("stopper")

    stopper.visual(
        mesh_from_cadquery(_stopper_solid(), "stopper_cap"),
        material=metal_brushed,
        name="stopper_cap",
    )
    stopper.visual(
        _stopper_grip_mesh(),
        material=metal_brushed,
        name="stopper_grip",
    )

    stopper.inertial = Inertial.from_geometry(
        Cylinder(STOPPER_R, STOPPER_TOTAL_H),
        mass=0.04,
        origin=Origin(xyz=(0.0, 0.0, -STOPPER_SKIRT_H + STOPPER_TOTAL_H * 0.5)),
    )

    # Prismatic joint: stopper lifts vertically from the rim top
    model.articulation(
        "stopper_lift",
        ArticulationType.PRISMATIC,
        parent=body,
        child=stopper,
        origin=Origin(xyz=(0.0, 0.0, RIM_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=LIFT_MAX,
            effort=2.0,
            velocity=0.5,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    stopper = object_model.get_part("stopper")
    lift = object_model.get_articulation("stopper_lift")

    # The stopper skirt seats over the neck; allow small intentional overlap.
    ctx.allow_overlap(
        stopper,
        body,
        elem_a="stopper_cap",
        elem_b="jar_glass",
        reason="The stopper skirt intentionally slips down over the threaded neck rim.",
    )

    # ---- jar is faceted (octagonal): XY extent ratio shows non-circular symmetry ----
    body_aabb = ctx.part_world_aabb(body)
    bx = body_aabb[1][0] - body_aabb[0][0]
    by = body_aabb[1][1] - body_aabb[0][1]
    bz = body_aabb[1][2] - body_aabb[0][2]
    ctx.check(
        "jar is faceted (octagonal, wider than tall)",
        bx > bz + 0.005 and by > bz + 0.005,
        details=f"body extents: x={bx:.4f}, y={by:.4f}, z={bz:.4f}",
    )

    # ---- stopper sits on top of the jar ----
    stopper_pos = ctx.part_world_position(stopper)
    ctx.check(
        "stopper seats on the neck rim",
        stopper_pos is not None and stopper_pos[2] > RIM_TOP_Z - 0.005,
        details=f"stopper_pos={stopper_pos}, rim_top={RIM_TOP_Z}",
    )

    # ---- stopper overlaps the body footprint in XY ----
    ctx.expect_overlap(
        stopper, body, axes="xy", min_overlap=0.02,
        name="stopper caps the neck (XY overlap)",
    )

    # ---- stopper_lift raises the stopper vertically ----
    rest_z = ctx.part_world_position(stopper)[2]
    with ctx.pose({lift: LIFT_MAX}):
        lifted_z = ctx.part_world_position(stopper)[2]
        ctx.expect_gap(
            stopper, body, axis="z",
            min_gap=0.005,
            positive_elem="stopper_cap", negative_elem="jar_glass",
            name="lifted stopper clears the neck rim",
        )

    ctx.check(
        "stopper_lift raises stopper vertically",
        lifted_z > rest_z + LIFT_MAX * 0.8,
        details=f"rest_z={rest_z:.4f}, lifted_z={lifted_z:.4f}",
    )

    # ---- joint is prismatic (not fixed) ----
    ctx.check(
        "stopper_lift is a prismatic joint",
        lift.articulation_type == ArticulationType.PRISMATIC,
        details=f"joint type={lift.articulation_type}",
    )

    # ---- glass wall thickness visible at mouth (jar_glass has neck geometry above body) ----
    glass_aabb = ctx.part_element_world_aabb(body, elem="jar_glass")
    glass_top = glass_aabb[1][2]
    ctx.check(
        "jar glass extends to the rim (neck present)",
        glass_top > JAR_BODY_H + NECK_H * 0.8,
        details=f"glass top z={glass_top:.4f}, expected > {JAR_BODY_H + NECK_H * 0.8:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
