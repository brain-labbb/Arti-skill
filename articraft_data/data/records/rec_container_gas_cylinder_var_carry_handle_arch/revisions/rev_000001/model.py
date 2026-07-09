from __future__ import annotations

# Red LPG gas cylinder with arched carry-handle bow guard.
# Frame: vertical cylinder axis along +Z. The flat floor of the foot ring sits
# at z=0; the domed top and bow guard arch are at the top (+Z). The brass valve
# sits on the domed shoulder, centered on the +Z axis. A tall inverted-U steel
# bar rises from two anchor pads on the shoulder, arches over the valve as a
# carry handle, and protects it from above. The valve handwheel turns about the
# vertical valve axis.
# Articulations:
#   - bow guard: REVOLUTE about the shoulder-mounted pivot line so the arched
#     carry handle can swing forward/back while staying seated in the anchor pads.
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
    tube_from_spline_points,
)

# ---- key dimensions (meters) ----
BODY_R = 0.150               # cylinder outer radius (~0.30 m dia)
FOOT_TOP_Z = 0.030           # top of the foot ring
WALL_TOP_Z = 0.330           # where the cylindrical wall ends and shoulder begins
SHOULDER_TOP_Z = 0.430       # top of the domed shoulder (where the neck starts)
NECK_TOP_Z = 0.470           # top of the brass valve neck boss
VALVE_AXIS_Z = NECK_TOP_Z    # valve body starts on top of the neck boss
BOW_ANCHOR_Y = 0.105         # wider half-span of the carry handle pivots
BOW_ANCHOR_Z = SHOULDER_TOP_Z - 0.023
BOW_RISE = 0.185             # taller clearance above the shoulder
BOW_BAR_R = 0.006


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


def _bow_guard_mesh():
    # Tall arched carry-handle bow guard: a single rounded inverted-U steel bar
    # that rises from two anchor pads on the domed shoulder, spans over the valve
    # as a carry handle, and protects it from above. Modeled in the guard's
    # local pivot frame: the revolute joint origin is halfway between the two
    # anchor pads, the pivot axis is +Y, and the tube ends sit at z=0.
    anchor_y = BOW_ANCHOR_Y
    peak_z = BOW_RISE
    # Spline points for the inverted-U arch (symmetric about Y=0).
    # The legs stay wide enough to clear the handwheel rim (±0.033 m).
    arch_points = [
        (0.0, -anchor_y, 0.0),
        (0.0, -anchor_y + 0.002, 0.050),
        (0.0, -0.082, 0.102),
        (0.0, -0.048, peak_z - 0.020),
        (0.0, 0.000, peak_z),
        (0.0, 0.040, peak_z - 0.020),
        (0.0, 0.082, 0.102),
        (0.0, anchor_y - 0.002, 0.050),
        (0.0, anchor_y, 0.0),
    ]
    bar = tube_from_spline_points(
        arch_points,
        radius=BOW_BAR_R,
        samples_per_segment=16,
        radial_segments=16,
        cap_ends=True,
    )
    return mesh_from_geometry(bar, "bow_guard_tube")


def _bow_anchor_pad_mesh(name: str):
    # Fixed shoulder boss that captures each rotating handle end.
    pad = CylinderGeometry(0.0135, 0.024, radial_segments=20)
    return mesh_from_geometry(pad, name)


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

    # ---- body (root): red steel cylinder + domed top + bow guard + label ----
    body = model.part("body")
    body.visual(_body_mesh(), material=red_steel, name="body_shell")

    # Fixed anchor pads on the domed shoulder. The rotating bow guard's pivot
    # line runs through the two pad centers.
    for i in range(2):
        sign = -1 + 2 * i
        body.visual(
            _bow_anchor_pad_mesh(f"bow_anchor_pad_{i}"),
            origin=Origin(xyz=(0.0, sign * BOW_ANCHOR_Y, BOW_ANCHOR_Z - 0.005)),
            material=steel,
            name=f"bow_anchor_pad_{i}",
        )

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

    # ---- rotating arched carry-handle bow guard ----
    bow_guard = model.part("bow_guard")
    bow_guard.visual(_bow_guard_mesh(), material=steel, name="bow_guard_tube")
    bow_guard.inertial = Inertial.from_geometry(
        Cylinder(radius=BOW_BAR_R, length=2.0 * BOW_ANCHOR_Y), mass=0.18
    )
    model.articulation(
        "body_to_bow_guard",
        ArticulationType.REVOLUTE,
        parent=body,
        child=bow_guard,
        origin=Origin(xyz=(0.0, 0.0, BOW_ANCHOR_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.55 * math.pi,
            upper=0.55 * math.pi,
            effort=4.0,
            velocity=2.0,
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
    bow_guard = object_model.get_part("bow_guard")
    foot = object_model.get_part("foot_ring")
    valve = object_model.get_part("valve")
    handwheel = object_model.get_part("handwheel")
    bow_joint = object_model.get_articulation("body_to_bow_guard")
    wheel_joint = object_model.get_articulation("valve_to_handwheel")

    # ---- cylinder is tall with a domed top ----
    body_aabb = ctx.part_world_aabb(body)
    bext = _ext(body_aabb)
    ctx.check(
        "cylinder body is tall (height > diameter)",
        bext[2] > 0.43 and bext[2] > max(bext[0], bext[1]),
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

    # ---- bow guard arch spans over the valve as a carry handle ----
    bow_aabb = ctx.part_world_aabb(bow_guard)
    hw_aabb = ctx.part_world_aabb(handwheel)
    ctx.check(
        "bow guard arch rises above the handwheel (protects from above)",
        bow_aabb is not None and hw_aabb is not None and bow_aabb[1][2] > hw_aabb[1][2] + 0.035,
        details=f"bow top z={bow_aabb[1][2] if bow_aabb else None}, handwheel top z={hw_aabb[1][2] if hw_aabb else None}",
    )
    # The bow guard spans laterally (along Y) wider than the valve body.
    bow_yspan = bow_aabb[1][1] - bow_aabb[0][1] if bow_aabb else 0.0
    ctx.check(
        "bow guard arch spans wide across the shoulder",
        bow_aabb is not None and bow_yspan > 0.20,
        details=f"bow y-span={bow_yspan:.3f}",
    )
    # The arch is taller than it is wide (inverted-U shape, not a flat ring).
    bow_zspan = bow_aabb[1][2] - bow_aabb[0][2] if bow_aabb else 0.0
    bow_xspan = bow_aabb[1][0] - bow_aabb[0][0] if bow_aabb else 0.0
    ctx.check(
        "bow guard reads as a tall arch (taller than wide in X)",
        bow_zspan > bow_xspan + 0.02,
        details=f"bow z-span={bow_zspan:.3f}, x-span={bow_xspan:.3f}",
    )
    for i in range(2):
        pad_aabb = ctx.part_element_world_aabb(body, elem=f"bow_anchor_pad_{i}")
        ctx.check(
            f"bow anchor pad_{i} is fixed on the domed shoulder",
            pad_aabb is not None
            and abs(((pad_aabb[0][1] + pad_aabb[1][1]) / 2.0) - (-1 + 2 * i) * BOW_ANCHOR_Y) < 0.010
            and SHOULDER_TOP_Z - 0.050 < pad_aabb[0][2] < SHOULDER_TOP_Z,
            details=f"pad_aabb={pad_aabb}",
        )
    for i in range(2):
        ctx.allow_overlap(
            body,
            bow_guard,
            elem_a=f"bow_anchor_pad_{i}",
            elem_b="bow_guard_tube",
            reason="Rotating carry-handle tube end is intentionally captured inside this shoulder anchor pad.",
        )
    ctx.expect_contact(bow_guard, body, name="carry handle ends seated in shoulder anchor pads")

    with ctx.pose({bow_joint: math.pi / 6.0}):
        bow_tilted = ctx.part_world_aabb(bow_guard)
    rest_x_center = (bow_aabb[0][0] + bow_aabb[1][0]) / 2.0
    tilted_x_center = (bow_tilted[0][0] + bow_tilted[1][0]) / 2.0
    ctx.check(
        "bow guard revolute joint swings the handle forward/back",
        abs(tilted_x_center - rest_x_center) > 0.035 and bow_tilted[1][2] > hw_aabb[1][2],
        details=f"rest_x={rest_x_center:.3f}, tilted_x={tilted_x_center:.3f}, tilted_top={bow_tilted[1][2]:.3f}",
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
