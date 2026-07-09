from __future__ import annotations

# Red LPG gas cylinder with a threaded protective screw-cap bonnet.
# Frame: vertical cylinder axis along +Z. The flat floor of the foot ring sits
# at z=0; the domed top and collar guard ring are at the top (+Z). The brass
# valve sits on the domed shoulder, centered on the +Z axis. A domed steel
# screw-cap bonnet threads down over the valve stem on a REVOLUTE joint about
# the vertical valve axis (coupling motion that lowers the cap as it spins to
# seal the valve).
# Articulations:
#   - valve_to_bonnet: REVOLUTE about the vertical valve axis

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
    # bonnet cap covers, and a side outlet spigot. Built in CadQuery in valve-local
    # frame (origin at the base of the valve where it seats on the neck boss).
    # Valve seat boss (cylindrical) sitting on the neck.
    body = cq.Workplane("XY").circle(0.026).extrude(0.022)
    # Main valve block (slightly tapered cap).
    body = body.union(
        cq.Workplane("XY").workplane(offset=0.022).circle(0.022).extrude(0.024)
    )
    # Outlet spigot pointing out along +X (the brass nozzle the hose connects to).
    spigot = (
        cq.Workplane("YZ")
        .workplane(offset=0.010)
        .center(0.0, 0.034)
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
    # Valve stem rising up the axis (the bonnet cap covers this).
    body = body.union(
        cq.Workplane("XY").workplane(offset=0.046).circle(0.007).extrude(0.020)
    )
    return mesh_from_cadquery(body, "valve_body")


def _bonnet_mesh():
    """Domed steel screw-cap bonnet with ribbed grip and rotation indicator.

    A compact protective cap that threads over the valve stem. Built entirely
    in CadQuery for a single connected mesh: cylindrical skirt, hemisphere
    dome (revolved quarter-circle), blind bore from below, six vertical grip
    ribs on the skirt exterior, and a horizontal indicator lug on the +X side
    that breaks rotational symmetry for detectability.
    """
    outer_r = 0.022   # cap outer radius (44 mm diameter)
    inner_r = 0.009   # bore radius (18 mm, fits over 14 mm stem with clearance)
    skirt_h = 0.024   # cylindrical skirt height
    dome_r = outer_r  # hemisphere dome (same radius as skirt)

    # 1. Cylindrical skirt
    cap = cq.Workplane("XY").circle(outer_r).extrude(skirt_h)

    # 2. Hemisphere dome on top: revolved quarter-circle profile on XZ plane.
    #    Profile traces from the dome equator (outer_r, skirt_h) through a
    #    quarter-circle arc to the apex (0, skirt_h + dome_r), then closes
    #    back along the Z axis.
    dome = (
        cq.Workplane("XZ")
        .moveTo(0.0, skirt_h)
        .lineTo(outer_r, skirt_h)
        .threePointArc(
            (outer_r * 0.707, skirt_h + dome_r * 0.707),
            (0.0, skirt_h + dome_r),
        )
        .close()
        .revolve(360, (0.0, skirt_h), (0.0, skirt_h + 1.0))
    )
    cap = cap.union(dome)

    # 3. Blind bore from the bottom (accepts the valve stem).
    #    Stops 5 mm below the dome peak to leave a solid dome shell.
    bore_depth = skirt_h + dome_r - 0.005
    bore = cq.Workplane("XY").circle(inner_r).extrude(bore_depth)
    cap = cap.cut(bore)

    # 4. Grip ribs: 6 vertical ribs on the skirt exterior, offset 30° from
    #    the indicator lug to avoid overlapping features.
    for i in range(6):
        ang = i * math.pi / 3.0 + math.pi / 6.0
        x = (outer_r + 0.001) * math.cos(ang)
        y = (outer_r + 0.001) * math.sin(ang)
        rib = (
            cq.Workplane("XY")
            .workplane(offset=0.004)
            .center(x, y)
            .circle(0.0015)
            .extrude(skirt_h - 0.008)
        )
        cap = cap.union(rib)

    # 5. Indicator lug: a horizontal pin on the +X side of the skirt.
    #    This breaks rotational symmetry so the rotation is visually detectable.
    lug = (
        cq.Workplane("YZ")
        .workplane(offset=outer_r - 0.003)
        .center(0.0, skirt_h * 0.55)
        .circle(0.003)
        .extrude(0.012)
    )
    cap = cap.union(lug)

    return mesh_from_cadquery(cap, "bonnet_cap")


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
    steel = model.material("bare_ste", rgba=(0.62, 0.62, 0.64, 1.0))
    zinc_cap = model.material("zinc_plated_cap", rgba=(0.52, 0.55, 0.58, 1.0))
    hazard = model.material("hazard_label", rgba=(0.92, 0.80, 0.20, 1.0))

    # ---- body (root): red steel cylinder + domed top + collar guard + label ----
    body = model.part("body")
    body.visual(_body_mesh(), material=red_steel, name="body_shell")

    # Protective collar guard ring (bare steel) on top, struts to the shoulder.
    body.visual(_collar_mesh(), material=steel, name="collar_guard")

    # Hazard diamond label patch on the cylindrical wall (front, +X face).
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

    # ---- screw-cap bonnet: REVOLUTE about the vertical valve axis ----
    # The bonnet is a domed steel cap that threads down over the valve stem.
    # Joint origin at the top of the main valve block (valve-local z=0.046),
    # which is the seating surface where the cap rests when fully tightened.
    bonnet = model.part("bonnet")
    bonnet.visual(_bonnet_mesh(), material=zinc_cap, name="bonnet_cap")
    bonnet.inertial = Inertial.from_geometry(
        Cylinder(radius=0.022, length=0.034), mass=0.06,
    )
    # Positive q = counterclockwise when viewed from above = unscrewing direction.
    # At q=0 the cap is fully seated (tightened); multiple turns to fully remove.
    model.articulation(
        "valve_to_bonnet",
        ArticulationType.REVOLUTE,
        parent=valve,
        child=bonnet,
        origin=Origin(xyz=(0.0, 0.0, 0.046)),  # valve-local: top of main block
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=3.0,
            lower=0.0,           # fully seated (tightened)
            upper=6.0 * math.pi, # ~3 full turns to unscrew
        ),
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
    bonnet = object_model.get_part("bonnet")
    bonnet_joint = object_model.get_articulation("valve_to_bonnet")

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

    # ---- bonnet cap sits on the valve, covering the stem ----
    bonnet_pos = ctx.part_world_position(bonnet)
    ctx.check(
        "bonnet on the valve axis, above the valve base",
        bonnet_pos is not None
        and abs(bonnet_pos[0]) < 0.02
        and abs(bonnet_pos[1]) < 0.02
        and bonnet_pos[2] > valve_pos[2],
        details=f"bonnet origin={bonnet_pos}",
    )
    # Bonnet covers the valve stem in XY footprint.
    ctx.expect_overlap(
        bonnet, valve, axes="xy", min_overlap=0.008,
        name="bonnet cap overlaps valve stem in XY",
    )
    # Bonnet cap sits above the main valve block (seated on the block top).
    ctx.expect_contact(
        bonnet, valve,
        name="bonnet cap seated on valve body",
    )
    # The valve stem tip may nest slightly into the dome ceiling of the cap.
    ctx.allow_overlap(
        bonnet,
        valve,
        elem_a="bonnet_cap",
        elem_b="valve_body",
        reason="Valve stem tip is intentionally nested inside the bonnet dome ceiling when fully seated.",
    )

    # ---- bonnet rotates about the valve axis (indicator dot moves) ----
    rest_aabb = ctx.part_world_aabb(bonnet)
    rest_xspan = rest_aabb[1][0] - rest_aabb[0][0]
    rest_yspan = rest_aabb[1][1] - rest_aabb[0][1]
    with ctx.pose({bonnet_joint: math.pi / 2.0}):
        turned_aabb = ctx.part_world_aabb(bonnet)
        turn_xspan = turned_aabb[1][0] - turned_aabb[0][0]
        turn_yspan = turned_aabb[1][1] - turned_aabb[0][1]
    # At rest the raised indicator dot biases the AABB toward +X; after a
    # quarter-turn the bias swaps to +Y.
    ctx.check(
        "bonnet quarter-turn rotates the indicator dot",
        (rest_xspan > rest_yspan + 0.001) and (turn_yspan > turn_xspan + 0.001),
        details=f"rest=({rest_xspan:.4f},{rest_yspan:.4f}) turned=({turn_xspan:.4f},{turn_yspan:.4f})",
    )

    # ---- bonnet cap is a domed shape (taller than wide from dome) ----
    bonnet_aabb = ctx.part_world_aabb(bonnet)
    cap_ext = _ext(bonnet_aabb)
    ctx.check(
        "bonnet cap is compact (diameter > 30mm, height > 20mm)",
        cap_ext[0] > 0.030 and cap_ext[1] > 0.030 and cap_ext[2] > 0.020,
        details=f"cap extents=({cap_ext[0]:.4f},{cap_ext[1]:.4f},{cap_ext[2]:.4f})",
    )

    return ctx.report()


object_model = build_object_model()
