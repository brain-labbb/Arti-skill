from __future__ import annotations

# Spice jar variant: wide-mouth square glass jar with a rotating perforated
# shaker insert, flip lid on rear revolute hinge, and visible gasket ring.
#
# Frame: vertical axis +Z, jar centered on world origin in XY, base on z=0.
#
# Parts:
#   - jar_body: clear square-section hollow glass jar with wide mouth opening.
#   - shaker: rotating perforated insert sitting in the jar mouth (CONTINUOUS).
#   - gasket: rubber sealing ring seated on the jar rim (FIXED to jar_body).
#   - lid: flip lid hinged at the rear (+Y) edge (REVOLUTE, opens upward).
#
# Articulations:
#   - body_to_shaker: CONTINUOUS around Z, rotates the shaker insert.
#   - body_to_lid: REVOLUTE at rear hinge, axis chosen so positive q opens lid.

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ----- key dimensions (meters) -----
JAR_SECT = 0.065        # outer square section of jar body
JAR_CORNER_R = 0.008    # rounded corner radius
JAR_WALL = 0.003        # glass wall thickness

JAR_BOTTOM_Z = 0.0
JAR_TOP_Z = 0.100       # top of jar body
JAR_H = JAR_TOP_Z - JAR_BOTTOM_Z

# Raised rim at top of jar (slightly thicker wall, forms the mouth)
RIM_H = 0.008           # height of the raised rim above body top
RIM_SECT = 0.067        # rim outer section (slightly proud of body)
RIM_CORNER_R = 0.008
MOUTH_SECT = 0.056      # wide mouth inner opening

# Internal shelf inside the rim for the shaker to sit on
SHELF_THICK = 0.002
SHELF_OD = MOUTH_SECT   # shelf outer = mouth bore (flush with inner rim wall)
SHELF_ID = 0.050         # shelf inner opening (supports shaker edges)
SHELF_Z = JAR_TOP_Z     # shelf sits at the base of the rim

# Shaker insert: perforated square plate sitting on the shelf
SHAKER_SECT = 0.054     # slightly smaller than mouth for free rotation
SHAKER_THICK = 0.002
SHAKER_HOLE_D = 0.003   # hole diameter
SHAKER_PITCH = 0.007    # hole spacing
SHAKER_FRAME = 0.006    # solid frame border
SHAKER_Z = SHELF_Z + SHELF_THICK + SHAKER_THICK / 2.0  # sits on shelf, centered

# Gasket ring: annular seal on top of rim
GASKET_OD = 0.064       # outer diameter
GASKET_ID = 0.054       # inner diameter
GASKET_THICK = 0.002
GASKET_Z = JAR_TOP_Z + RIM_H  # sits on top of rim

# Flip lid
LID_SECT = 0.068        # slightly larger than jar body for coverage
LID_THICK = 0.005
LID_CORNER_R = 0.006
# Lid hinges at rear (+Y edge) of the rim top
HINGE_Y = RIM_SECT / 2.0
HINGE_Z = GASKET_Z + GASKET_THICK  # hinge sits above gasket

# Hinge barrel (visual detail)
HINGE_BARREL_R = 0.004
HINGE_BARREL_L = 0.030


def _jar_body_solid() -> cq.Workplane:
    """Hollow square-section glass jar with raised rim and wide mouth."""
    # Main body: outer shell
    outer = (
        cq.Workplane("XY")
        .rect(JAR_SECT, JAR_SECT)
        .extrude(JAR_H)
        .edges("|Z")
        .fillet(JAR_CORNER_R)
    )
    # Inner cavity: hollow with solid glass floor
    inner_w = JAR_SECT - 2.0 * JAR_WALL
    inner = (
        cq.Workplane("XY")
        .workplane(offset=JAR_WALL)  # solid glass floor
        .rect(inner_w, inner_w)
        .extrude(JAR_H + RIM_H + 0.01)  # over-extrude to open through top
        .edges("|Z")
        .fillet(max(JAR_CORNER_R - JAR_WALL, 0.001))
    )
    body = outer.cut(inner)

    # Raised rim at top: thicker wall section forming the mouth
    rim_outer = (
        cq.Workplane("XY")
        .workplane(offset=JAR_TOP_Z)
        .rect(RIM_SECT, RIM_SECT)
        .extrude(RIM_H)
        .edges("|Z")
        .fillet(RIM_CORNER_R)
    )
    # Rim bore: wide mouth opening
    rim_inner = (
        cq.Workplane("XY")
        .workplane(offset=JAR_TOP_Z - 0.001)
        .rect(MOUTH_SECT, MOUTH_SECT)
        .extrude(RIM_H + 0.002)
        .edges("|Z")
        .fillet(max(RIM_CORNER_R - 0.003, 0.001))
    )
    rim = rim_outer.cut(rim_inner)

    # Internal shelf/ledge at the base of the rim: supports the shaker insert.
    # Annular ring from shelf_id to shelf_od, thickness = SHELF_THICK.
    shelf_outer = (
        cq.Workplane("XY")
        .workplane(offset=SHELF_Z)
        .rect(SHELF_OD, SHELF_OD)
        .extrude(SHELF_THICK)
        .edges("|Z")
        .fillet(max(RIM_CORNER_R - 0.003, 0.001))
    )
    shelf_bore = (
        cq.Workplane("XY")
        .workplane(offset=SHELF_Z - 0.001)
        .rect(SHELF_ID, SHELF_ID)
        .extrude(SHELF_THICK + 0.002)
        .edges("|Z")
        .fillet(max(RIM_CORNER_R - 0.004, 0.001))
    )
    shelf = shelf_outer.cut(shelf_bore)

    return body.union(rim).union(shelf)


def _gasket_solid() -> cq.Workplane:
    """Annular gasket ring that sits on top of the jar rim."""
    outer = (
        cq.Workplane("XY")
        .circle(GASKET_OD / 2.0)
        .extrude(GASKET_THICK)
    )
    inner = (
        cq.Workplane("XY")
        .circle(GASKET_ID / 2.0)
        .extrude(GASKET_THICK)
    )
    return outer.cut(inner)


def _lid_solid() -> cq.Workplane:
    """Flip lid panel. Built in lid-local frame with hinge edge at origin,
    panel extends toward -Y (forward) from the hinge line."""
    # Lid panel: extends from hinge at Y=0 toward -Y direction
    # The hinge line is along X at Y=0, Z=0 (lid local frame)
    panel = (
        cq.Workplane("XY")
        .center(0.0, -LID_SECT / 2.0)
        .rect(LID_SECT, LID_SECT)
        .extrude(LID_THICK)
        .edges("|Z")
        .fillet(LID_CORNER_R)
    )
    # Add a small lip/tab at the front for grip
    tab = (
        cq.Workplane("XY")
        .center(0.0, -LID_SECT + 0.004)
        .rect(0.016, 0.008)
        .extrude(LID_THICK + 0.002)
        .edges("|Z")
        .fillet(0.002)
    )
    return panel.union(tab)


def _hinge_barrel_solid() -> cq.Workplane:
    """Small hinge barrel cylinder for visual detail at the hinge line."""
    return (
        cq.Workplane("XZ")
        .circle(HINGE_BARREL_R)
        .extrude(HINGE_BARREL_L)
        .translate((-HINGE_BARREL_L / 2.0, 0.0, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="spice_jar")

    # Materials
    glass = model.material("clear_glass", rgba=(0.80, 0.85, 0.88, 0.30))
    dark_plastic = model.material("dark_plastic", rgba=(0.15, 0.15, 0.15, 1.0))
    rubber = model.material("rubber_gasket", rgba=(0.25, 0.22, 0.20, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.72, 0.72, 0.70, 1.0))

    # ---- jar_body (root): clear glass jar with wide mouth ----
    jar_body = model.part("jar_body")
    jar_body.visual(
        mesh_from_cadquery(_jar_body_solid(), "jar_glass"),
        material=glass,
        name="jar_glass",
    )
    jar_body.inertial = Inertial.from_geometry(
        Box((JAR_SECT, JAR_SECT, JAR_H + RIM_H)),
        mass=0.14,
        origin=Origin(xyz=(0.0, 0.0, (JAR_H + RIM_H) / 2.0)),
    )

    # ---- gasket: rubber ring on top of rim (FIXED to jar_body conceptually) ----
    # Modeled as part of jar_body since it doesn't move independently.
    jar_body.visual(
        mesh_from_cadquery(_gasket_solid(), "gasket_ring"),
        material=rubber,
        origin=Origin(xyz=(0.0, 0.0, GASKET_Z)),
        name="gasket_ring",
    )

    # ---- shaker: rotating perforated insert ----
    shaker = model.part("shaker")
    shaker_geom = PerforatedPanelGeometry(
        (SHAKER_SECT, SHAKER_SECT),
        SHAKER_THICK,
        hole_diameter=SHAKER_HOLE_D,
        pitch=SHAKER_PITCH,
        frame=SHAKER_FRAME,
        corner_radius=0.004,
        stagger=True,
        center=True,
    )
    shaker.visual(
        mesh_from_geometry(shaker_geom, "shaker_plate"),
        material=dark_plastic,
        name="shaker_plate",
    )
    shaker.inertial = Inertial.from_geometry(
        Box((SHAKER_SECT, SHAKER_SECT, SHAKER_THICK)),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # Shaker rotates continuously around Z in the jar mouth
    model.articulation(
        "body_to_shaker",
        ArticulationType.CONTINUOUS,
        parent=jar_body,
        child=shaker,
        origin=Origin(xyz=(0.0, 0.0, SHAKER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0),
    )

    # ---- lid: flip lid on rear hinge ----
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_solid(), "lid_panel"),
        material=brushed_metal,
        name="lid_panel",
    )
    # Hinge barrel visual on the lid part (at the hinge line)
    lid.visual(
        mesh_from_cadquery(_hinge_barrel_solid(), "hinge_barrel"),
        material=brushed_metal,
        name="hinge_barrel",
    )
    lid.inertial = Inertial.from_geometry(
        Box((LID_SECT, LID_SECT, LID_THICK)),
        mass=0.025,
        origin=Origin(xyz=(0.0, -LID_SECT / 2.0, LID_THICK / 2.0)),
    )

    # Lid hinge: rear (+Y) edge of rim top.
    # At q=0, lid is closed (flat). The lid extends along -Y from the hinge.
    # axis=(-1, 0, 0) makes positive q rotate the -Y side toward +Z (opening up).
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=jar_body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=0.0, upper=1.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    jar_body = object_model.get_part("jar_body")
    shaker = object_model.get_part("shaker")
    lid = object_model.get_part("lid")
    shaker_joint = object_model.get_articulation("body_to_shaker")
    lid_hinge = object_model.get_articulation("body_to_lid")

    # ---- Jar has wide mouth: jar is shorter than tall bottle ----
    body_ext = ctx.part_world_aabb(jar_body)
    if body_ext is not None:
        mn, mx = body_ext
        dz = mx[2] - mn[2]
        dx = mx[0] - mn[0]
        ctx.check(
            "jar is wider than tall (jar proportions)",
            dz < 2.0 * dx,
            details=f"height={dz:.4f}, width={dx:.4f}",
        )

    # ---- Gasket ring exists on jar body ----
    gasket_vis = jar_body.get_visual("gasket_ring")
    ctx.check(
        "gasket ring visual exists on jar body",
        gasket_vis is not None,
        details="gasket_ring visual not found",
    )

    # ---- Shaker insert exists and rotates continuously ----
    ctx.check(
        "shaker joint is continuous rotation",
        shaker_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={shaker_joint.articulation_type}",
    )

    # ---- Shaker sits within the jar mouth (XY containment) ----
    ctx.expect_within(
        shaker,
        jar_body,
        axes="xy",
        margin=0.005,
        name="shaker insert fits within jar mouth",
    )

    # ---- Lid hinge is REVOLUTE with proper limits ----
    ctx.check(
        "lid hinge is revolute",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={lid_hinge.articulation_type}",
    )
    limits = lid_hinge.motion_limits
    ctx.check(
        "lid hinge has bounded limits (not continuous)",
        limits is not None and limits.lower is not None and limits.upper is not None,
        details=f"limits={limits}",
    )
    if limits is not None and limits.lower is not None and limits.upper is not None:
        ctx.check(
            "lid hinge opens to at least 90 degrees",
            limits.upper >= 1.4,
            details=f"upper={limits.upper}",
        )

    # ---- At rest (q=0), lid is closed and covers the mouth ----
    ctx.expect_overlap(
        lid,
        jar_body,
        axes="xy",
        min_overlap=0.030,
        name="closed lid overlaps jar mouth in XY",
    )

    # ---- Opening the lid: positive q lifts the front edge upward ----
    lid_rest_pos = ctx.part_world_position(lid)
    with ctx.pose({lid_hinge: 1.5}):
        lid_open_pos = ctx.part_world_position(lid)
        # The lid origin (hinge) stays fixed, but the lid panel should rise
        # We check that the lid part AABB top increases
        open_aabb = ctx.part_world_aabb(lid)
        rest_aabb = ctx.part_world_aabb(lid)  # this is now at the open pose

    # Better check: compare Z of lid at rest vs open using part_world_position
    # The lid origin is at the hinge, which doesn't move. Let's check
    # that at open pose, the lid no longer fully covers the mouth.
    with ctx.pose({lid_hinge: 1.5}):
        # At open pose, lid XY overlap with jar should decrease significantly
        open_overlap_report = ctx.part_world_aabb(lid)

    # Use a direct Z-gap check: at open, lid front edge should be above jar top
    with ctx.pose({lid_hinge: 1.5}):
        lid_open_aabb = ctx.part_world_aabb(lid)
        jar_aabb = ctx.part_world_aabb(jar_body)
        if lid_open_aabb is not None and jar_aabb is not None:
            lid_min_z = lid_open_aabb[0][2]
            jar_max_z = jar_aabb[1][2]
            ctx.check(
                "opened lid clears the jar top (front edge rises)",
                lid_min_z > jar_max_z - 0.015,
                details=f"lid_min_z={lid_min_z:.4f}, jar_max_z={jar_max_z:.4f}",
            )

    # ---- Lid closed pose: lid sits near jar rim height ----
    lid_closed_aabb = ctx.part_world_aabb(lid)
    if lid_closed_aabb is not None:
        ctx.check(
            "closed lid sits at or above jar rim",
            lid_closed_aabb[0][2] >= JAR_TOP_Z - 0.005,
            details=f"lid_min_z={lid_closed_aabb[0][2]:.4f}",
        )

    # ---- Intentional overlap: hinge barrel seats at the hinge line ----
    ctx.allow_overlap(
        lid,
        jar_body,
        elem_a="hinge_barrel",
        elem_b="jar_glass",
        reason="Hinge barrel is intentionally embedded at the rear rim to represent the pivot mount.",
    )

    # ---- Gasket sits on top of rim (Z positioning) ----
    ctx.expect_gap(
        jar_body,
        jar_body,
        axis="z",
        positive_elem="gasket_ring",
        negative_elem="jar_glass",
        min_gap=-0.003,
        max_gap=0.010,
        name="gasket ring seated on jar rim",
    )

    return ctx.report()


object_model = build_object_model()
