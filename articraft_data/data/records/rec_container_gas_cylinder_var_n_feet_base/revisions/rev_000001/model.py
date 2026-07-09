from __future__ import annotations

# Red LPG gas cylinder (3-stub-foot base variant).
# Frame: vertical cylinder axis along +Z. Three discrete stub feet stand the
# cylinder on the floor (z=0); the domed top and collar guard ring are at the
# top (+Z). The brass valve sits on the domed shoulder, centered on the +Z axis.
# The valve handwheel turns about the vertical valve axis.
# Articulations:
#   - 3 stub feet: FIXED to the body base, evenly spaced at 120 deg.
#   - valve handwheel: REVOLUTE about the vertical valve axis (turn to open/close),
#     with an off-axis spoke so the rotation is visually detectable.

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

# ---- stub foot dimensions (meters) ----
N_FEET = 3                   # number of discrete stub feet around the base
FOOT_H = 0.030               # stub foot height (floor z=0 to body contact z=FOOT_H)
FOOT_WIDTH = 0.042           # tangential width of each foot
FOOT_DEPTH = 0.024           # radial depth of each foot
FOOT_R = BODY_R - 0.008      # radial placement of foot centers from cylinder axis


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


def _handwheel_mesh():
    # Star/cross handwheel: a hub, a rim torus, and four radial spokes; one spoke
    # extends past the rim into an off-axis handle lug so the rotation is clearly
    # detectable (breaks rotational symmetry of the wheel).
    geom = CylinderGeometry(0.009, 0.012, radial_segments=20)  # hub
    rim = TorusGeometry(0.028, 0.005, radial_segments=12, tubular_segments=36)
    geom.merge(rim)
    # Four spokes (cross/star) joining hub to rim.
    for i in range(4):
        ang = i * math.pi / 2.0
        spoke = CylinderGeometry(0.0035, 0.046, radial_segments=8).rotate_y(math.pi / 2.0)
        spoke.rotate_z(ang)
        geom.merge(spoke)
    # Off-axis handle lug: one spoke extends well past the rim along +X, so the
    # wheel's footprint is clearly biased toward +X and a quarter-turn swaps it
    # to +Y. This makes the rotation unambiguously detectable.
    lug = CylinderGeometry(0.0045, 0.034, radial_segments=10).rotate_y(math.pi / 2.0)
    lug.translate(0.038, 0.0, 0.0)  # centered out at +X, past the 0.028 rim
    geom.merge(lug)
    knob = CylinderGeometry(0.007, 0.016, radial_segments=12)
    knob.translate(0.052, 0.0, 0.0)  # ball at the tip of the handle lug
    geom.merge(knob)
    return mesh_from_geometry(geom, "handwheel")


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


def _stub_foot_mesh():
    # Short rounded stub foot: a compact block with filleted vertical edges.
    # Built in foot-local frame with top face at z=0 (body contact surface)
    # and bottom face at z=-FOOT_H (floor contact surface).
    # X axis = radial (depth), Y axis = tangential (width).
    foot = (
        cq.Workplane("XY")
        .workplane(offset=-FOOT_H)
        .rect(FOOT_DEPTH, FOOT_WIDTH)
        .extrude(FOOT_H)
    )
    # Fillet the vertical edges for a molded look.
    foot = foot.edges("|Z").fillet(0.005)
    return mesh_from_cadquery(foot, "stub_foot")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lpg_gas_cylinder")

    red_steel = model.material("weathered_red_steel", rgba=(0.72, 0.16, 0.13, 1.0))
    brass = model.material("brass_valve", rgba=(0.78, 0.62, 0.22, 1.0))
    dark = model.material("dark_foot", rgba=(0.12, 0.12, 0.13, 1.0))
    steel = model.material("bare_steel", rgba=(0.62, 0.62, 0.64, 1.0))
    hazard = model.material("hazard_label", rgba=(0.92, 0.80, 0.20, 1.0))

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

    # ---- N stub feet around the base, evenly spaced at 120 deg ----
    shared_foot_mesh = _stub_foot_mesh()
    for i in range(N_FEET):
        theta = i * (2.0 * math.pi / N_FEET)
        foot_part = model.part(f"foot_{i}")
        foot_part.visual(shared_foot_mesh, material=dark, name="stub_foot")
        foot_part.inertial = Inertial.from_geometry(
            Box((FOOT_DEPTH, FOOT_WIDTH, FOOT_H)),
            mass=0.3,
            origin=Origin(xyz=(0.0, 0.0, -FOOT_H / 2.0)),
        )
        # Joint origin at the body-foot contact surface (body bottom at FOOT_TOP_Z).
        # Foot-local frame: top at z=0, bottom at z=-FOOT_H.
        # rpy rotates the foot so local X points radially outward.
        model.articulation(
            f"body_to_foot_{i}",
            ArticulationType.FIXED,
            parent=body,
            child=foot_part,
            origin=Origin(
                xyz=(FOOT_R * math.cos(theta), FOOT_R * math.sin(theta), FOOT_TOP_Z),
                rpy=(0.0, 0.0, theta),
            ),
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

    # ---- valve handwheel: REVOLUTE about the vertical valve axis ----
    handwheel = model.part("handwheel")
    handwheel.visual(_handwheel_mesh(), material=brass, name="handwheel")
    handwheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.030, length=0.014), mass=0.05,
    )
    # Joint origin at the top of the valve stem; child handwheel sits there.
    # Stem top in body frame: (VALVE_AXIS_Z - 0.006) + 0.066 = stem cap top.
    wheel_z = (VALVE_AXIS_Z - 0.006) + 0.066
    model.articulation(
        "valve_to_handwheel",
        ArticulationType.REVOLUTE,
        parent=valve,
        child=handwheel,
        origin=Origin(xyz=(0.0, 0.0, 0.072)),  # valve-local: top of the stem
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=-4.0 * math.pi, upper=4.0 * math.pi),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    feet = [object_model.get_part(f"foot_{i}") for i in range(N_FEET)]
    valve = object_model.get_part("valve")
    handwheel = object_model.get_part("handwheel")
    wheel_joint = object_model.get_articulation("valve_to_handwheel")

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

    # ---- N stub feet at the base, evenly spaced ----
    for i, foot in enumerate(feet):
        foot_aabb = ctx.part_world_aabb(foot)
        ctx.check(
            f"foot_{i} is at the base (floor contact)",
            foot_aabb is not None and foot_aabb[0][2] < 0.005 and foot_aabb[1][2] < 0.045,
            details=f"foot_{i} aabb z=({foot_aabb[0][2]:.3f},{foot_aabb[1][2]:.3f})",
        )
        ctx.allow_overlap(
            foot,
            body,
            elem_a="stub_foot",
            elem_b="body_shell",
            reason=f"Stub foot_{i} top face is intentionally embedded into the cylinder base at the contact surface.",
        )
        ctx.expect_contact(foot, body, name=f"foot_{i} attached to cylinder base")
    
    # Check feet are evenly spaced at 120 deg around the cylinder axis.
    foot_centers = [ctx.part_world_position(f) for f in feet]
    foot_angles = [math.atan2(c[1], c[0]) for c in foot_centers]
    # Normalize angles to [0, 2pi) and check spacing.
    foot_angles_norm = [(a + 2*math.pi) % (2*math.pi) for a in foot_angles]
    foot_angles_sorted = sorted(foot_angles_norm)
    angle_diffs = [
        (foot_angles_sorted[(j+1) % N_FEET] - foot_angles_sorted[j]) % (2*math.pi)
        for j in range(N_FEET)
    ]
    expected_spacing = 2*math.pi / N_FEET
    ctx.check(
        f"{N_FEET} feet are evenly spaced at 120 deg",
        all(abs(diff - expected_spacing) < 0.1 for diff in angle_diffs),
        details=f"angle diffs={[f'{d:.2f}' for d in angle_diffs]}, expected={expected_spacing:.2f}",
    )

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

    # ---- handwheel sits on the valve stem, on the valve axis ----
    ctx.expect_contact(handwheel, valve, name="handwheel seated on valve stem")
    wheel_pos = ctx.part_world_position(handwheel)
    ctx.check(
        "handwheel on the valve axis, above the valve",
        wheel_pos is not None
        and abs(wheel_pos[0]) < 0.02
        and abs(wheel_pos[1]) < 0.02
        and wheel_pos[2] > valve_pos[2],
        details=f"handwheel origin={wheel_pos}",
    )

    # ---- handwheel rotates about the valve axis: off-axis spoke knob moves ----
    knob_rest = ctx.part_world_aabb(handwheel)
    # At rest, the off-axis knob biases the AABB along +X (knob at +x).
    rest_xspan = knob_rest[1][0] - knob_rest[0][0]
    rest_yspan = knob_rest[1][1] - knob_rest[0][1]
    with ctx.pose({wheel_joint: math.pi / 2.0}):
        knob_turn = ctx.part_world_aabb(handwheel)
        turn_xspan = knob_turn[1][0] - knob_turn[0][0]
        turn_yspan = knob_turn[1][1] - knob_turn[0][1]
    # A quarter turn swaps which axis the off-axis knob extends along.
    ctx.check(
        "handwheel quarter-turn rotates the off-axis spoke knob",
        (rest_xspan > rest_yspan + 0.004) and (turn_yspan > turn_xspan + 0.004),
        details=f"rest=({rest_xspan:.3f},{rest_yspan:.3f}) turned=({turn_xspan:.3f},{turn_yspan:.3f})",
    )

    return ctx.report()


object_model = build_object_model()
