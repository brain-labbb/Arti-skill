from __future__ import annotations

# Tall cylindrical STORAGE JAR with clamp bail lid.
# Frame: jar axis along +Z, base resting on z=0, jar centered on (x=0, y=0).
#
# A tall clear glass jar with a wide mouth, flat glass lid, rubber gasket ring,
# and a wire bail clamp that pivots on two side lugs to secure the lid.
#
# Articulations:
#   - lid_hinge: REVOLUTE at rear rim edge, lid flips open backward
#   - bail_pivot: REVOLUTE on Y-axis through the two side pivot lugs

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
JAR_OUTER_R = 0.040          # outer radius of glass body (~80mm dia)
JAR_BODY_H = 0.180           # height of the glass body
WALL = 0.004                 # glass wall thickness
BASE_THICK = 0.006           # thick glass base

# Rim (wider collar at top for sealing)
RIM_OUTER_R = 0.044          # rim outer radius
RIM_INNER_R = JAR_OUTER_R - WALL  # jar inner radius (~0.036)
RIM_H = 0.014                # rim/collar height
RIM_TOP_Z = JAR_BODY_H + RIM_H  # top of the rim (0.194)

# Lid
LID_R = RIM_OUTER_R - 0.002  # lid fits just inside rim outer edge (~0.042)
LID_H = 0.005                # lid thickness

# Gasket
GASKET_CENTER_R = RIM_OUTER_R - 0.006  # gasket sits on rim flat (~0.038)
GASKET_SECTION_R = 0.0025              # gasket cross-section radius

# Bail pivot geometry
PIVOT_Z = JAR_BODY_H + RIM_H * 0.6    # pivot height (upper part of rim)
PIVOT_LUG_W = 0.010                    # lug width
PIVOT_LUG_H = 0.012                    # lug height
PIVOT_LUG_D = 0.006                    # lug protrusion from rim outer
PIVOT_Y = RIM_OUTER_R + PIVOT_LUG_D * 0.5  # pivot center Y offset (~0.047)

# Bail wire
WIRE_R = 0.0018               # wire cross-section radius (~3.6mm dia)
BAIL_LEG_H = 0.055           # height of bail legs above pivot



def _jar_body_solid() -> cq.Workplane:
    """Hollow thick-walled tall glass jar built as a revolve profile in XZ."""
    inner_r = JAR_OUTER_R - WALL
    pts = [
        (0.0, 0.0),
        (JAR_OUTER_R, 0.0),
        (JAR_OUTER_R, JAR_BODY_H - 0.008),
        (JAR_OUTER_R + 0.001, JAR_BODY_H - 0.004),
        (RIM_OUTER_R, JAR_BODY_H),
        (RIM_OUTER_R, RIM_TOP_Z),
        (RIM_OUTER_R - 0.003, RIM_TOP_Z),
        (RIM_OUTER_R - 0.003, RIM_TOP_Z - 0.003),
        (inner_r, RIM_TOP_Z - 0.003),
        (inner_r, BASE_THICK),
        (0.0, BASE_THICK),
    ]
    profile = cq.Workplane("XZ").polyline(pts).close()
    return profile.revolve(360.0, (0, 0, 0), (0, 1, 0))


def _pivot_lugs() -> cq.Workplane:
    """Two pivot mount lugs on opposite sides of the rim."""
    result = None
    for sign in (+1, -1):
        y_center = sign * (RIM_OUTER_R + PIVOT_LUG_D * 0.5)
        lug = (
            cq.Workplane("XY")
            .workplane(offset=PIVOT_Z - PIVOT_LUG_H * 0.5)
            .center(0.0, y_center)
            .rect(PIVOT_LUG_W, PIVOT_LUG_D)
            .extrude(PIVOT_LUG_H)
        )
        result = lug if result is None else result.union(lug)
    return result


def _lid_solid() -> cq.Workplane:
    """Flat glass lid disk.
    In lid-local frame: origin at hinge point (rear edge of rim),
    lid disk extends in +Y toward the front, centered on the jar mouth."""
    # Disk center at y = distance from hinge to jar center in local frame
    hinge_to_center = RIM_OUTER_R - 0.004  # ~0.040
    disk = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .center(0.0, hinge_to_center)
        .circle(LID_R)
        .extrude(LID_H)
    )
    # Small rear flange at the hinge point for visual pivot connection
    flange = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .center(0.0, 0.003)
        .rect(0.012, 0.010)
        .extrude(LID_H)
    )
    return disk.union(flange)


def _gasket_solid() -> cq.Workplane:
    """Rubber gasket O-ring that sits on the rim top.
    Built as a revolved circle (torus) in world coordinates on the jar body."""
    gasket_z = RIM_TOP_Z - GASKET_SECTION_R
    # Draw circle in XZ plane at (GASKET_CENTER_R, gasket_z), revolve around Z axis
    # Use moveTo to place the circle without shifting the workplane origin
    torus = (
        cq.Workplane("XZ")
        .moveTo(GASKET_CENTER_R, gasket_z)
        .circle(GASKET_SECTION_R)
        .revolve(360.0, (0, 0, 0), (0, 1, 0))
    )
    return torus


def _bail_solid() -> cq.Workplane:
    """Wire bail clamp in local frame (origin at pivot center between lugs).
    Built from unioned cylinders: two legs + crossbar arch."""
    # Left leg: from (0, -PIVOT_Y, -0.008) to (0, -PIVOT_Y, BAIL_LEG_H)
    # Right leg: from (0, +PIVOT_Y, -0.008) to (0, +PIVOT_Y, BAIL_LEG_H)
    # Top bar: connecting the tops of the legs

    leg_len = BAIL_LEG_H + 0.008  # total leg length including stub below pivot

    # Left leg (cylinder along Z, centered at y=-PIVOT_Y)
    left_leg = (
        cq.Workplane("XY")
        .workplane(offset=-0.008)
        .center(0.0, -PIVOT_Y)
        .circle(WIRE_R)
        .extrude(leg_len)
    )

    # Right leg
    right_leg = (
        cq.Workplane("XY")
        .workplane(offset=-0.008)
        .center(0.0, PIVOT_Y)
        .circle(WIRE_R)
        .extrude(leg_len)
    )

    # Top crossbar: cylinder along Y connecting leg tops
    # From (0, -PIVOT_Y, BAIL_LEG_H) to (0, +PIVOT_Y, BAIL_LEG_H)
    crossbar_len = PIVOT_Y * 2
    crossbar = (
        cq.Workplane("XZ")
        .workplane(offset=-PIVOT_Y)
        .center(0.0, BAIL_LEG_H)
        .circle(WIRE_R)
        .extrude(crossbar_len)
    )

    # The two legs and crossbar form one connected solid:
    # leg tops overlap with the crossbar cylinder at the T-junctions.
    bail = left_leg.union(right_leg).union(crossbar)
    return bail


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="storage_jar_clamp_lid")

    # Materials
    glass_clear = model.material("glass_clear", rgba=(0.85, 0.90, 0.92, 0.40))
    lid_glass = model.material("lid_glass", rgba=(0.80, 0.88, 0.90, 0.50))
    rubber_dark = model.material("rubber_dark", rgba=(0.15, 0.12, 0.10, 1.0))
    wire_metal = model.material("wire_metal", rgba=(0.65, 0.67, 0.70, 1.0))

    # ---- jar body (root): hollow glass cylinder + rim + pivot lugs ----
    body = model.part("body")
    jar_glass = _jar_body_solid().union(_pivot_lugs())
    body.visual(
        mesh_from_cadquery(jar_glass, "jar_shell"),
        material=glass_clear,
        name="jar_shell",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(JAR_OUTER_R, JAR_BODY_H + RIM_H),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, (JAR_BODY_H + RIM_H) * 0.5)),
    )

    # ---- gasket: rubber ring on rim (fixed to body) ----
    gasket = model.part("gasket")
    gasket.visual(
        mesh_from_cadquery(_gasket_solid(), "gasket_ring"),
        material=rubber_dark,
        name="gasket_ring",
    )
    gasket.inertial = Inertial.from_geometry(
        Cylinder(GASKET_CENTER_R + GASKET_SECTION_R, GASKET_SECTION_R * 2),
        mass=0.01,
        origin=Origin(xyz=(0.0, 0.0, RIM_TOP_Z - GASKET_SECTION_R)),
    )
    model.articulation(
        "gasket_fixed",
        ArticulationType.FIXED,
        parent=body,
        child=gasket,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # ---- lid: hinged at rear rim edge ----
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_solid(), "lid_disk"),
        material=lid_glass,
        name="lid_disk",
    )
    lid.inertial = Inertial.from_geometry(
        Cylinder(LID_R, LID_H),
        mass=0.06,
        origin=Origin(xyz=(0.0, LID_R, LID_H * 0.5)),
    )
    # Lid hinge at rear rim edge: hinge axis along X, lid extends in +Y (front)
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -(RIM_OUTER_R - 0.004), RIM_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0, lower=0.0, upper=2.2
        ),
    )

    # ---- bail: pivots on Y axis through side lugs ----
    bail = model.part("bail")
    bail.visual(
        mesh_from_cadquery(_bail_solid(), "bail_wire"),
        material=wire_metal,
        name="bail_wire",
    )
    bail.inertial = Inertial.from_geometry(
        Box((WIRE_R * 4, PIVOT_Y * 2, BAIL_LEG_H)),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0, BAIL_LEG_H * 0.5)),
    )
    # Bail pivot at center between lugs, axis along Y
    # q=0: bail arch is up (over lid), positive q swings bail down
    model.articulation(
        "bail_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=bail,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=3.0, lower=0.0, upper=math.pi
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    bail = object_model.get_part("bail")
    gasket = object_model.get_part("gasket")
    lid_hinge = object_model.get_articulation("lid_hinge")
    bail_pivot = object_model.get_articulation("bail_pivot")

    # Allow gasket to overlap body rim (seated on rim)
    ctx.allow_overlap(
        gasket, body,
        elem_a="gasket_ring", elem_b="jar_shell",
        reason="The gasket ring is seated on the jar rim surface.",
    )
    # Allow gasket to overlap lid (sandwiched under lid when closed)
    ctx.allow_overlap(
        gasket, lid,
        elem_a="gasket_ring", elem_b="lid_disk",
        reason="The gasket ring contacts the underside of the lid when closed.",
    )
    # Allow bail to overlap body at pivot lugs (stubs in lug holes)
    ctx.allow_overlap(
        bail, body,
        elem_a="bail_wire", elem_b="jar_shell",
        reason="The bail pivot stubs are intentionally nested inside the side lugs.",
    )

    # ---- jar is tall: height > diameter ----
    body_aabb = ctx.part_world_aabb(body)
    bext = (body_aabb[1][0] - body_aabb[0][0],
            body_aabb[1][1] - body_aabb[0][1],
            body_aabb[1][2] - body_aabb[0][2])
    ctx.check(
        "jar is tall (height > diameter)",
        bext[2] > bext[0] + 0.02 and bext[2] > bext[1] + 0.02,
        details=f"body extents (x,y,z)={bext}",
    )

    # ---- wide mouth: inner opening is substantial ----
    mouth_dia = 2.0 * RIM_INNER_R
    ctx.check(
        "jar has wide mouth opening",
        mouth_dia > 0.050,
        details=f"mouth diameter={mouth_dia:.4f}m",
    )

    # ---- gasket exists and sits at rim height (check visual element) ----
    gasket_aabb = ctx.part_element_world_aabb(gasket, elem="gasket_ring")
    gasket_z_center = (gasket_aabb[0][2] + gasket_aabb[1][2]) * 0.5
    ctx.check(
        "gasket is at rim height",
        abs(gasket_z_center - (RIM_TOP_Z - GASKET_SECTION_R)) < 0.010,
        details=f"gasket_z_center={gasket_z_center}, expected~{RIM_TOP_Z - GASKET_SECTION_R}",
    )

    # ---- lid sits at rim level when closed (check visual disk) ----
    lid_closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_disk")
    lid_closed_z = (lid_closed_aabb[0][2] + lid_closed_aabb[1][2]) * 0.5
    ctx.check(
        "lid is at rim level when closed",
        abs(lid_closed_z - (RIM_TOP_Z + LID_H * 0.5)) < 0.015,
        details=f"lid_closed_z={lid_closed_z}, expected~{RIM_TOP_Z + LID_H * 0.5}",
    )

    # ---- lid hinge opens the lid upward ----
    with ctx.pose({lid_hinge: 1.5}):
        lid_open_aabb = ctx.part_element_world_aabb(lid, elem="lid_disk")
        lid_open_z = (lid_open_aabb[0][2] + lid_open_aabb[1][2]) * 0.5
        ctx.check(
            "lid hinge lifts lid when opened",
            lid_open_z > lid_closed_z + 0.02,
            details=f"closed_z={lid_closed_z}, open_z={lid_open_z}",
        )

    # ---- bail pivot: at q=0 arch is up, at q=pi arch swings down ----
    bail_closed_aabb = ctx.part_element_world_aabb(bail, elem="bail_wire")
    bail_closed_z = (bail_closed_aabb[0][2] + bail_closed_aabb[1][2]) * 0.5
    with ctx.pose({bail_pivot: math.pi}):
        bail_open_aabb = ctx.part_element_world_aabb(bail, elem="bail_wire")
        bail_open_z = (bail_open_aabb[0][2] + bail_open_aabb[1][2]) * 0.5
        ctx.check(
            "bail swings from closed (up) to open (down)",
            bail_open_z < bail_closed_z - 0.02,
            details=f"closed_z={bail_closed_z}, open_z={bail_open_z}",
        )

    # ---- bail pivots on side hinges (joint axis is along Y) ----
    bail_joint = object_model.get_articulation("bail_pivot")
    ctx.check(
        "bail pivot axis is along Y (side hinges)",
        abs(bail_joint.axis[1]) > 0.9,
        details=f"bail axis={bail_joint.axis}",
    )

    # ---- lid hinge is a non-fixed revolute joint with finite limits ----
    ctx.check(
        "lid_hinge is revolute with finite limits",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE
        and lid_hinge.motion_limits is not None
        and lid_hinge.motion_limits.lower is not None
        and lid_hinge.motion_limits.upper is not None
        and lid_hinge.motion_limits.upper > lid_hinge.motion_limits.lower,
        details=f"type={lid_hinge.articulation_type}, limits={lid_hinge.motion_limits}",
    )

    # ---- bail_pivot is a non-fixed revolute joint ----
    ctx.check(
        "bail_pivot is revolute with limits",
        bail_pivot.articulation_type == ArticulationType.REVOLUTE
        and bail_pivot.motion_limits is not None
        and bail_pivot.motion_limits.upper is not None
        and bail_pivot.motion_limits.upper > bail_pivot.motion_limits.lower,
        details=f"type={bail_pivot.articulation_type}, limits={bail_pivot.motion_limits}",
    )

    return ctx.report()


object_model = build_object_model()
