from __future__ import annotations

# Red LPG gas cylinder with a self-closing clip-on lever valve.
# Frame: vertical cylinder axis along +Z. The flat floor of the foot ring sits
# at z=0; the domed top and collar guard ring are at the top (+Z). The brass
# valve sits on the domed shoulder, centered on the +Z axis. A horizontal
# on/off lever pivots about a horizontal axis at the top of the valve body
# (push down to open, spring back to close).
# Articulations:
#   - valve lever: REVOLUTE about a horizontal axis at the valve stem top
#     (push lever down to open gas flow, spring return to horizontal/closed).

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- key dimensions (meters) ----
BODY_R = 0.150               # cylinder outer radius (~0.30 m dia)
FOOT_TOP_Z = 0.030           # top of the foot ring
WALL_TOP_Z = 0.330           # where the cylindrical wall ends and shoulder begins
SHOULDER_TOP_Z = 0.430       # top of the domed shoulder (where the neck starts)
NECK_TOP_Z = 0.470           # top of the brass valve neck boss
COLLAR_RING_Z = 0.500        # height of the protective collar guard ring (~0.50 m tall)

VALVE_AXIS_Z = NECK_TOP_Z    # valve body starts on top of the neck boss


def _body_mesh():
    # Weathered steel cylinder: foot transition, cylindrical wall, rounded
    # shoulder, and a domed top, built as a lathed shell of revolution.
    # Profile is a list of (radius, z) points revolved about +Z.
    profile = [
        (0.000, FOOT_TOP_Z - 0.005),   # bottom center (closed base just above foot)
        (BODY_R - 0.004, FOOT_TOP_Z),  # base edge
        (BODY_R, 0.060),               # lower wall flares out a touch
        (BODY_R, WALL_TOP_Z),          # straight cylindrical wall
        (BODY_R - 0.012, 0.360),       # shoulder starts to round in
        (BODY_R - 0.045, 0.392),
        (BODY_R - 0.095, SHOULDER_TOP_Z - 0.012),
        (0.052, SHOULDER_TOP_Z),       # top of dome near the neck
        (0.034, NECK_TOP_Z),           # short brass neck boss base radius (steel collar shoulder)
        (0.000, NECK_TOP_Z),           # close the top center
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=64), "body_shell")


def _valve_mesh():
    # Brass valve block: a stout hex-ish body on the neck, a rising stem that the
    # handwheel caps, and a side outlet spigot. Built in CadQuery in valve-local
    # frame (origin at the base of the valve where it seats on the neck boss).
    # Valve seat boss (cylindrical) sitting on the neck.
    body = cq.Workplane("XY").circle(0.026).extrude(0.022)
    # Main valve block (slightly tapered cap).
    body = body.union(
        cq.Workplane("XY").workplane(offset=0.022).circle(0.022).extrude(0.024)
    )
    # Outlet spigot pointing out along +X (the brass nozzle the hose connects to).
    # Built on the YZ plane (normal +X) so it extrudes radially outward; the
    # workplane offset places the spigot center at z=0.034 on the valve block,
    # and it starts inside the block (x>=0.010) so it stays welded to the body.
    spigot = (
        cq.Workplane("YZ")
        .workplane(offset=0.010)
        .center(0.0, 0.034)  # (in-plane) -> world (y=0, z=0.034)
        .circle(0.010)
        .extrude(0.034)
    )
    body = body.union(spigot)
    # A small cap ring at the spigot tip.
    tip = (
        cq.Workplane("YZ")
        .workplane(offset=0.040)
        .center(0.0, 0.034)
        .circle(0.012)
        .extrude(0.006)
    )
    body = body.union(tip)
    # Valve stem rising up the axis (the handwheel will cap this).
    body = body.union(
        cq.Workplane("XY").workplane(offset=0.046).circle(0.007).extrude(0.020)
    )
    return mesh_from_cadquery(body, "valve_body")


def _lever_mesh():
    # Self-closing clip-on lever valve: a pivot hub that clips over the valve
    # stem top, a horizontal arm extending along +X, and a wider paddle grip
    # at the tip for finger operation. Origin at the pivot center; when the
    # joint is at q=0 the arm is horizontal. Push down (positive q about +Y)
    # swings the +X tip toward -Z to open the valve.
    #
    # Pivot hub — cylindrical clip that snaps over the valve stem top.
    # Extends below the pivot origin so the bore wraps around the stem
    # (realistic clip-on seating; small intentional overlap with stem).
    hub = cq.Workplane("XY").workplane(offset=-0.008).circle(0.012).extrude(0.020)
    # Lever arm — a flat bar extending along +X from the hub.
    arm = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .center(0.036, 0.0)
        .rect(0.056, 0.010)
        .extrude(0.008)
    )
    hub = hub.union(arm)
    # Paddle grip — wider, slightly thicker end for finger leverage.
    grip = (
        cq.Workplane("XY")
        .workplane(offset=-0.001)
        .center(0.070, 0.0)
        .rect(0.024, 0.020)
        .extrude(0.010)
    )
    hub = hub.union(grip)
    # Spring-return boss on the opposite side of the arm (behind the pivot)
    # — a short stub along -X that the return spring wraps around. Placed
    # so it overlaps with the hub body for a connected mesh.
    spring_boss = (
        cq.Workplane("XY")
        .workplane(offset=0.001)
        .center(-0.014, 0.0)
        .circle(0.005)
        .extrude(0.006)
    )
    hub = hub.union(spring_boss)
    return mesh_from_cadquery(hub, "lever")


def _collar_mesh():
    # Circular protective collar guard ring: a top ring supported by four short
    # struts rising from the steel shoulder. Modeled as one connected mesh in the
    # body part frame (centered on +Z axis).
    ring_r = 0.080
    geom = TorusGeometry(ring_r, 0.009, radial_segments=12, tubular_segments=48)
    geom.translate(0.0, 0.0, COLLAR_RING_Z)
    # Four struts down to the shoulder.
    strut_bottom_z = SHOULDER_TOP_Z - 0.020
    strut_h = COLLAR_RING_Z - strut_bottom_z
    for i in range(4):
        ang = i * math.pi / 2.0 + math.pi / 4.0
        strut = CylinderGeometry(0.006, strut_h, radial_segments=8)
        strut.translate(ring_r, 0.0, strut_bottom_z + strut_h / 2.0)
        strut.rotate_z(ang)
        geom.merge(strut)
    return mesh_from_geometry(geom, "collar_guard")


def _foot_mesh():
    # Dark foot ring base: a stout ring skirt at the bottom of the cylinder.
    profile = [
        (BODY_R - 0.018, 0.0),
        (BODY_R + 0.006, 0.0),
        (BODY_R + 0.006, FOOT_TOP_Z),
        (BODY_R - 0.004, FOOT_TOP_Z + 0.004),
        (BODY_R - 0.018, FOOT_TOP_Z - 0.004),
        (BODY_R - 0.018, 0.0),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=64), "foot_ring")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lpg_gas_cylinder")

    red_steel = model.material("weathered_red_steel", rgba=(0.72, 0.16, 0.13, 1.0))
    brass = model.material("brass_valve", rgba=(0.78, 0.62, 0.22, 1.0))
    dark = model.material("dark_foot", rgba=(0.12, 0.12, 0.13, 1.0))
    steel = model.material("bare_steel", rgba=(0.62, 0.62, 0.64, 1.0))
    hazard = model.material("hazard_label", rgba=(0.92, 0.80, 0.20, 1.0))
    lever_mat = model.material("lever_handle", rgba=(0.18, 0.18, 0.20, 1.0))

    # ---- body (root): red steel cylinder + domed top + collar guard + label ----
    body = model.part("body")
    body.visual(_body_mesh(), material=red_steel, name="body_shell")

    # Protective collar guard ring (bare steel) on top, struts to the shoulder.
    body.visual(_collar_mesh(), material=steel, name="collar_guard")

    # Hazard diamond label patch on the cylindrical wall (front, +X face).
    # A thin rotated box hugging the curved wall surface.
    body.visual(
        Box((0.006, 0.075, 0.075)),
        origin=Origin(xyz=(BODY_R - 0.001, 0.0, 0.165), rpy=(math.pi / 4.0, 0.0, 0.0)),
        material=hazard,
        name="hazard_label",
    )

    body.inertial = Inertial.from_geometry(
        Cylinder(radius=BODY_R, length=WALL_TOP_Z),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, WALL_TOP_Z / 2.0)),
    )

    # ---- dark foot ring base ----
    foot = model.part("foot_ring")
    foot.visual(_foot_mesh(), material=dark, name="foot_ring")
    foot.inertial = Inertial.from_geometry(
        Cylinder(radius=BODY_R, length=FOOT_TOP_Z), mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, FOOT_TOP_Z / 2.0)),
    )
    model.articulation(
        "body_to_foot",
        ArticulationType.FIXED,
        parent=body,
        child=foot,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # ---- brass valve body, seated on the neck boss (fixed to the body) ----
    valve = model.part("valve")
    valve.visual(_valve_mesh(), material=brass, name="valve_body")
    valve.inertial = Inertial.from_geometry(
        Cylinder(radius=0.026, length=0.046), mass=0.4,
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
    )
    model.articulation(
        "body_to_valve",
        ArticulationType.FIXED,
        parent=body,
        child=valve,
        origin=Origin(xyz=(0.0, 0.0, VALVE_AXIS_Z - 0.006)),
    )

    # ---- lever valve: REVOLUTE about horizontal axis at the valve stem top ----
    # Push the lever tip (along +X) down to open; spring returns to horizontal.
    lever = model.part("lever")
    lever.visual(_lever_mesh(), material=lever_mat, name="lever")
    lever.inertial = Inertial.from_geometry(
        Box((0.090, 0.020, 0.012)), mass=0.04,
        origin=Origin(xyz=(0.035, 0.0, 0.004)),
    )
    # Joint origin at the top of the valve stem in valve-local frame.
    # Axis along +Y: positive rotation swings the +X lever tip toward -Z (down).
    model.articulation(
        "valve_to_lever",
        ArticulationType.REVOLUTE,
        parent=valve,
        child=lever,
        origin=Origin(xyz=(0.0, 0.0, 0.072)),  # valve-local: top of the stem
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=0.0, upper=1.05),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    foot = object_model.get_part("foot_ring")
    valve = object_model.get_part("valve")
    lever = object_model.get_part("lever")
    lever_joint = object_model.get_articulation("valve_to_lever")

    # ---- cylinder is tall with a domed top ----
    body_aabb = ctx.part_world_aabb(body)
    bext = _ext(body_aabb)
    ctx.check(
        "cylinder body is tall (height > diameter)",
        bext[2] > 0.45 and bext[2] > max(bext[0], bext[1]),
        details=f"body extents={bext}",
    )
    # Domed top: the shell narrows toward the top (top section radius < body radius).
    top_band = ctx.part_element_world_aabb(body, elem="body_shell")
    ctx.check(
        "body shell reaches near full target height",
        top_band is not None and top_band[1][2] > 0.46,
        details=f"shell top={top_band[1] if top_band else None}",
    )

    # ---- foot ring at the base ----
    foot_aabb = ctx.part_world_aabb(foot)
    ctx.check(
        "foot ring is at the base",
        foot_aabb is not None and foot_aabb[0][2] < 0.005 and foot_aabb[1][2] < 0.045,
        details=f"foot aabb z=({foot_aabb[0][2]:.3f},{foot_aabb[1][2]:.3f})",
    )
    ctx.allow_overlap(
        foot,
        body,
        elem_a="foot_ring",
        elem_b="body_shell",
        reason="Steel foot ring skirt is intentionally seated up around the cylinder base edge.",
    )
    ctx.expect_contact(foot, body, name="foot ring attached to cylinder base")

    # ---- valve sits on top, on the axis, above the body wall ----
    valve_pos = ctx.part_world_position(valve)
    ctx.check(
        "valve mounted on top of the cylinder, on the axis",
        valve_pos is not None
        and valve_pos[2] > WALL_TOP_Z
        and abs(valve_pos[0]) < 0.02
        and abs(valve_pos[1]) < 0.02,
        details=f"valve origin={valve_pos}",
    )
    # Valve seats on the neck boss of the domed top (intentional local embed).
    ctx.allow_overlap(
        valve,
        body,
        elem_a="valve_body",
        elem_b="body_shell",
        reason="Brass valve seat boss is intentionally threaded into the steel neck of the domed top.",
    )

    # ---- collar guard ring surrounds/guards the valve on top ----
    collar_aabb = ctx.part_element_world_aabb(body, elem="collar_guard")
    ctx.check(
        "collar guard ring is above the valve neck (guards the valve)",
        collar_aabb is not None and collar_aabb[1][2] >= valve_pos[2],
        details=f"collar top z={collar_aabb[1][2] if collar_aabb else None}, valve z={valve_pos[2]:.3f}",
    )
    # The collar ring radius is wider than the valve, encircling it.
    ctx.check(
        "collar ring encircles the valve",
        collar_aabb is not None
        and (collar_aabb[1][0] - collar_aabb[0][0]) > 0.12,
        details=f"collar x-span={collar_aabb[1][0]-collar_aabb[0][0] if collar_aabb else None}",
    )

    # ---- lever sits on the valve stem top, above the valve body ----
    # The clip-on hub wraps around the valve stem (small intentional overlap).
    ctx.allow_overlap(
        lever,
        valve,
        elem_a="lever",
        elem_b="valve_body",
        reason="Clip-on lever hub bore intentionally wraps around the valve stem top for a seated pivot.",
    )
    ctx.expect_contact(lever, valve, name="lever pivot hub seated on valve stem top")
    lever_pos = ctx.part_world_position(lever)
    ctx.check(
        "lever pivot is on the valve axis, above the valve body",
        lever_pos is not None
        and abs(lever_pos[0]) < 0.02
        and abs(lever_pos[1]) < 0.02
        and lever_pos[2] > valve_pos[2],
        details=f"lever origin={lever_pos}",
    )

    # ---- lever pivots about a horizontal axis: push down to open ----
    # At rest (q=0), the lever arm is horizontal: the tip is at roughly the same
    # height as the pivot. Verify the rest pose has the tip extending along +X
    # and staying near the pivot height.
    rest_aabb = ctx.part_world_aabb(lever)
    rest_tip_z = (rest_aabb[0][2] + rest_aabb[1][2]) / 2.0
    rest_xspan = rest_aabb[1][0] - rest_aabb[0][0]
    ctx.check(
        "lever at rest is horizontal (arm spans wider than tall)",
        rest_xspan > 0.05 and (rest_aabb[1][2] - rest_aabb[0][2]) < 0.030,
        details=f"rest xspan={rest_xspan:.3f}, z-height={rest_aabb[1][2]-rest_aabb[0][2]:.3f}",
    )

    # At max open (positive q), the lever tip drops below the rest height.
    with ctx.pose({lever_joint: 0.9}):
        open_aabb = ctx.part_world_aabb(lever)
        open_tip_z_min = open_aabb[0][2]
    ctx.check(
        "lever pushed down: tip drops below rest position (opens valve)",
        open_tip_z_min < rest_tip_z - 0.015,
        details=f"rest_tip_z={rest_tip_z:.3f}, open_tip_z_min={open_tip_z_min:.3f}",
    )

    # The pivot axis is horizontal: joint axis should have zero Z component.
    joint_axis = lever_joint.axis
    ctx.check(
        "lever joint axis is horizontal (no vertical component)",
        joint_axis is not None and abs(joint_axis[2]) < 0.01,
        details=f"joint axis={joint_axis}",
    )

    return ctx.report()


object_model = build_object_model()
