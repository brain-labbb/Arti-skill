from __future__ import annotations

# Tall cylindrical storage jar with clamp lid and vertical-lift stopper.
# Frame: jar axis along +Z, base on z=0, centered on (x=0, y=0).
#
# Parts:
#   jar_body (root): tall hollow glass cylinder with wide-mouth neck rim,
#                    clamp ring, rubber gasket, brand label
#   lid:             clamp-style disk lid hinged at the back of the rim,
#                    central hole for the stopper pull tab, offset knob
#   stopper:         rubber stopper plugging the wide mouth, lifts vertically
#
# Articulations:
#   lid_hinge:    REVOLUTE at back of rim, axis +X, positive opens lid upward
#   stopper_lift: PRISMATIC along +Z, lifts stopper out of the mouth

import math

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

# ---- key dimensions (meters) ----
JAR_OUTER_R = 0.040           # outer radius of glass body (~80 mm dia)
JAR_BODY_H = 0.150            # tall cylindrical body height
WALL = 0.004                  # glass wall thickness
BASE_THICK = 0.006            # thick glass base

# Neck / wide mouth
NECK_OUTER_R = 0.037          # outer radius of the neck rim
MOUTH_R = 0.032               # inner mouth opening radius (~64 mm wide mouth)
NECK_H = 0.014                # neck height above body shoulder
RIM_TOP_Z = JAR_BODY_H + NECK_H  # z of rim top (0.164)

# Lid
LID_R = 0.043                 # lid radius (slightly wider than jar body)
LID_H = 0.007                 # lid thickness
TAB_HOLE_R = 0.010            # central hole for stopper pull tab

# Stopper
STOPPER_R = 0.034             # plug radius (compression fit in mouth, contacts wall)
STOPPER_H = 0.020             # tapered plug height
TAB_R = 0.008                 # pull-tab radius
TAB_H = 0.004                 # pull-tab height
STOPPER_LIFT = 0.055          # max prismatic travel
STOPPER_Z_OFFSET = -0.022     # child-frame Z offset seating plug in mouth

# Hinge location
HINGE_Y = -NECK_OUTER_R       # hinge pin at the back of the rim


# ---------------------------------------------------------------------------
# CadQuery geometry builders
# ---------------------------------------------------------------------------

def _jar_glass_solid() -> cq.Workplane:
    """Hollow thick-walled tall glass jar via revolve of an XZ half-profile.
    The profile traces the outer wall up, across the shoulder into the neck,
    then back down the inner wall to form a real open-topped cavity with a
    visible wide-mouth hollow opening."""
    pts = [
        (0.0, 0.0),                            # center of base
        (JAR_OUTER_R, 0.0),                    # outer base edge
        (JAR_OUTER_R, JAR_BODY_H - 0.008),     # outer wall up
        (JAR_OUTER_R - 0.002, JAR_BODY_H),     # rounded shoulder
        (NECK_OUTER_R, JAR_BODY_H + 0.003),    # step into the neck
        (NECK_OUTER_R, RIM_TOP_Z),             # neck outer to rim top
        (MOUTH_R, RIM_TOP_Z),                  # across rim top to mouth edge
        (MOUTH_R, JAR_BODY_H - 0.004),         # inner neck wall down
        (JAR_OUTER_R - WALL, JAR_BODY_H - 0.008),
        (JAR_OUTER_R - WALL, BASE_THICK),      # inner wall down to thick base
        (0.0, BASE_THICK),                     # across inner base
        (0.0, 0.0),                            # close back to center
    ]
    profile = cq.Workplane("XZ").polyline(pts).close()
    return profile.revolve(360.0, (0, 0, 0), (0, 1, 0))


def _neck_threads() -> cq.Workplane:
    """Thread ridges on the neck for the clamp mechanism grip."""
    threads = None
    z0 = JAR_BODY_H + 0.007
    for i in range(3):
        z = z0 + i * 0.003
        ring = (
            cq.Workplane("XY")
            .workplane(offset=z)
            .circle(NECK_OUTER_R + 0.0008)
            .circle(NECK_OUTER_R - 0.0005)
            .extrude(0.0018)
        )
        threads = ring if threads is None else threads.union(ring)
    return threads


def _clamp_ring_cq() -> cq.Workplane:
    """Metal clamp band around the neck with two hinge lugs."""
    band_z = JAR_BODY_H + 0.001
    band = (
        cq.Workplane("XY")
        .workplane(offset=band_z)
        .circle(NECK_OUTER_R + 0.003)
        .circle(NECK_OUTER_R - 0.0005)
        .extrude(0.005)
    )
    for sign in (1, -1):
        lug = (
            cq.Workplane("XY")
            .workplane(offset=band_z + 0.001)
            .center(sign * (NECK_OUTER_R + 0.004), 0)
            .rect(0.006, 0.008)
            .extrude(0.008)
        )
        band = band.union(lug)
    return band


def _gasket_cq() -> cq.Workplane:
    """Rubber gasket ring seated on the rim, flush with the rim top."""
    return (
        cq.Workplane("XY")
        .workplane(offset=RIM_TOP_Z - 0.003)
        .circle(NECK_OUTER_R - 0.001)
        .circle(MOUTH_R + 0.001)
        .extrude(0.003)
    )


def _lid_cq() -> cq.Workplane:
    """Clamp lid: flat disk with central hole for the stopper tab, plus an
    offset knob on top."""
    # Solid disk
    disk = cq.Workplane("XY").circle(LID_R).extrude(LID_H)
    # Cut central hole for the stopper pull tab
    hole = (
        cq.Workplane("XY")
        .workplane(offset=-0.002)
        .circle(TAB_HOLE_R)
        .extrude(LID_H + 0.004)
    )
    disk = disk.cut(hole)
    # Offset knob for gripping the lid (avoids the central hole)
    knob = (
        cq.Workplane("XY")
        .workplane(offset=LID_H)
        .center(LID_R * 0.4, 0)
        .circle(0.005)
        .extrude(0.007)
    )
    knob = knob.edges(">Z").fillet(0.002)
    return disk.union(knob)


def _stopper_cq() -> cq.Workplane:
    """Rubber stopper: tapered plug body with a pull-tab knob on top."""
    # Tapered plug from z=0 (wide base) to z=STOPPER_H (narrower top)
    body = (
        cq.Workplane("XY")
        .circle(STOPPER_R)
        .workplane(offset=STOPPER_H)
        .circle(STOPPER_R - 0.003)
        .loft()
    )
    # Pull tab on top
    tab = (
        cq.Workplane("XY")
        .workplane(offset=STOPPER_H)
        .circle(TAB_R)
        .extrude(TAB_H)
    )
    tab = tab.edges(">Z").fillet(0.002)
    return body.union(tab)


def _label_cq() -> cq.Workplane:
    """Thin label band wrapped around the jar body exterior."""
    label_z = JAR_BODY_H * 0.45 - 0.0175
    return (
        cq.Workplane("XY")
        .workplane(offset=label_z)
        .circle(JAR_OUTER_R + 0.0005)
        .circle(JAR_OUTER_R - 0.0005)
        .extrude(0.035)
    )


# ---------------------------------------------------------------------------
# Model assembly
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tall_storage_jar")

    # Materials
    glass_clear = model.material("glass_clear", rgba=(0.82, 0.88, 0.90, 0.45))
    metal_silver = model.material("metal_silver", rgba=(0.72, 0.73, 0.74, 1.0))
    lid_cream = model.material("lid_cream", rgba=(0.92, 0.89, 0.82, 1.0))
    rubber_dark = model.material("rubber_dark", rgba=(0.22, 0.20, 0.18, 1.0))
    label_beige = model.material("label_beige", rgba=(0.95, 0.92, 0.85, 1.0))

    # ---- jar body (root): tall hollow glass cylinder ----
    jar = model.part("jar_body")

    glass = _jar_glass_solid().union(_neck_threads())
    jar.visual(
        mesh_from_cadquery(glass, "jar_glass"),
        material=glass_clear, name="jar_glass",
    )
    jar.visual(
        mesh_from_cadquery(_clamp_ring_cq(), "clamp_ring"),
        material=metal_silver, name="clamp_ring",
    )
    jar.visual(
        mesh_from_cadquery(_gasket_cq(), "gasket"),
        material=rubber_dark, name="gasket",
    )
    jar.visual(
        mesh_from_cadquery(_label_cq(), "brand_label"),
        material=label_beige, name="brand_label",
    )

    jar.inertial = Inertial.from_geometry(
        Cylinder(JAR_OUTER_R, JAR_BODY_H),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, JAR_BODY_H * 0.5)),
    )

    # ---- lid: clamp-style disk hinged at the back of the rim ----
    # At q=0, the child frame coincides with the hinge frame at
    # (0, HINGE_Y, RIM_TOP_Z) in parent.  The lid visual is offset
    # by +NECK_OUTER_R in local Y so the disk centres over the mouth.
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_cq(), "lid_disk"),
        origin=Origin(xyz=(0.0, NECK_OUTER_R, 0.0)),
        material=lid_cream, name="lid_disk",
    )

    lid.inertial = Inertial.from_geometry(
        Cylinder(LID_R, LID_H),
        mass=0.05,
        origin=Origin(xyz=(0.0, NECK_OUTER_R, LID_H * 0.5)),
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=jar,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, RIM_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0, lower=0.0, upper=2.4,
        ),
    )

    # ---- stopper: rubber plug seated in the mouth, lifts vertically ----
    # Prismatic joint origin at the rim top.  At q=0 the child frame is
    # at (0, 0, RIM_TOP_Z); the visual offset seats the plug inside the
    # mouth cavity with the pull tab protruding through the lid hole.
    stopper = model.part("stopper")
    stopper.visual(
        mesh_from_cadquery(_stopper_cq(), "stopper_plug"),
        origin=Origin(xyz=(0.0, 0.0, STOPPER_Z_OFFSET)),
        material=rubber_dark, name="stopper_plug",
    )

    stopper.inertial = Inertial.from_geometry(
        Cylinder(STOPPER_R, STOPPER_H + TAB_H),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.0, STOPPER_Z_OFFSET + (STOPPER_H + TAB_H) * 0.5)),
    )

    model.articulation(
        "stopper_lift",
        ArticulationType.PRISMATIC,
        parent=jar,
        child=stopper,
        origin=Origin(xyz=(0.0, 0.0, RIM_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=0.5, lower=0.0, upper=STOPPER_LIFT,
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    jar = object_model.get_part("jar_body")
    lid = object_model.get_part("lid")
    stopper = object_model.get_part("stopper")
    hinge = object_model.get_articulation("lid_hinge")
    lift = object_model.get_articulation("stopper_lift")

    # The stopper plug is intentionally seated inside the jar mouth cavity;
    # small mesh-boundary overlap may occur at the curved inner wall.
    ctx.allow_overlap(
        stopper, jar,
        elem_a="stopper_plug", elem_b="jar_glass",
        reason="Stopper plug is intentionally seated inside the hollow jar mouth.",
    )
    ctx.expect_overlap(
        stopper, jar, axes="xy", min_overlap=0.01,
        elem_a="stopper_plug", elem_b="jar_glass",
        name="stopper centered in the mouth cavity",
    )

    # ---- jar is tall: taller than wide ----
    jar_aabb = ctx.part_world_aabb(jar)
    ext = (
        jar_aabb[1][0] - jar_aabb[0][0],
        jar_aabb[1][1] - jar_aabb[0][1],
        jar_aabb[1][2] - jar_aabb[0][2],
    )
    ctx.check(
        "jar is tall (taller than wide)",
        ext[2] > ext[0] + 0.02 and ext[2] > ext[1] + 0.02,
        details=f"body_extents={ext}",
    )

    # ---- lid covers the mouth at rest ----
    ctx.expect_overlap(
        lid, jar, axes="xy", min_overlap=0.02,
        name="lid covers the jar mouth at rest",
    )

    # ---- lid hinge opens the lid upward ----
    # Use the lid disk element AABB center since the part origin sits at the
    # rotation center (hinge pin) and does not translate during rotation.
    rest_aabb = ctx.part_element_world_aabb(lid, elem="lid_disk")
    lid_rest_z = (rest_aabb[0][2] + rest_aabb[1][2]) * 0.5
    with ctx.pose({hinge: 1.5}):
        open_aabb = ctx.part_element_world_aabb(lid, elem="lid_disk")
        lid_open_z = (open_aabb[0][2] + open_aabb[1][2]) * 0.5
    ctx.check(
        "lid hinge opens the lid upward",
        lid_open_z > lid_rest_z + 0.02,
        details=f"rest_z={lid_rest_z:.4f}, open_z={lid_open_z:.4f}",
    )

    # ---- stopper is seated in the mouth at rest (XY overlap) ----
    ctx.expect_overlap(
        stopper, jar, axes="xy", min_overlap=0.01,
        elem_a="stopper_plug", elem_b="jar_glass",
        name="stopper is seated in the jar mouth at rest",
    )

    # ---- stopper lifts vertically ----
    s_rest_z = ctx.part_world_position(stopper)[2]
    with ctx.pose({lift: STOPPER_LIFT}):
        s_lifted_z = ctx.part_world_position(stopper)[2]
    ctx.check(
        "stopper lifts vertically out of the mouth",
        s_lifted_z > s_rest_z + STOPPER_LIFT * 0.8,
        details=f"rest_z={s_rest_z:.4f}, lifted_z={s_lifted_z:.4f}",
    )

    # ---- at max lift, stopper clears the rim along Z ----
    with ctx.pose({lift: STOPPER_LIFT}):
        ctx.expect_gap(
            stopper, jar, axis="z",
            min_gap=0.0,
            positive_elem="stopper_plug", negative_elem="jar_glass",
            name="lifted stopper clears the jar rim",
        )

    # ---- wide mouth opening ----
    ctx.check(
        "jar has a wide mouth opening",
        MOUTH_R > 0.025,
        details=f"mouth_radius={MOUTH_R:.4f}",
    )

    # ---- at least one non-fixed joint exists ----
    ctx.check(
        "model has non-fixed joints",
        hinge.articulation_type != ArticulationType.FIXED
        and lift.articulation_type != ArticulationType.FIXED,
        details=f"hinge={hinge.articulation_type}, lift={lift.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
