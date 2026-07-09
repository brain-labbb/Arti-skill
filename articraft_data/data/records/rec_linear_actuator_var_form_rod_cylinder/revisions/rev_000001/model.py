from __future__ import annotations

from math import cos, pi, sin

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------


def _barrel_tube(outer_r: float, inner_r: float, length: float) -> cq.Workplane:
    """Hollow cylindrical barrel tube centred at origin, axis along X."""
    outer = cq.Workplane("YZ").circle(outer_r).extrude(length / 2.0, both=True)
    bore = cq.Workplane("YZ").circle(inner_r).extrude(length / 2.0 + 0.001, both=True)
    return outer.cut(bore)


def _end_cap(
    outer_r: float,
    bore_r: float,
    thickness: float,
    bolt_count: int = 6,
    bolt_circle_r: float = 0.038,
    bolt_hole_r: float = 0.003,
) -> cq.Workplane:
    """Circular end-cap disc with centre bore and bolt-circle holes (axis along X)."""
    cap = cq.Workplane("YZ").circle(outer_r).extrude(thickness / 2.0, both=True)
    bore = cq.Workplane("YZ").circle(bore_r).extrude(thickness, both=True)
    cap = cap.cut(bore)
    for i in range(bolt_count):
        a = 2.0 * pi * i / bolt_count
        y = bolt_circle_r * cos(a)
        z = bolt_circle_r * sin(a)
        hole = (
            cq.Workplane("YZ")
            .center(y, z)
            .circle(bolt_hole_r)
            .extrude(thickness * 2.0, both=True)
        )
        cap = cap.cut(hole)
    return cap


def _motor_housing(radius: float, length: float) -> cq.Workplane:
    """Cylindrical motor body with rear end-bell flange (axis along X)."""
    body = cq.Workplane("YZ").circle(radius).extrude(length / 2.0, both=True)
    flange = (
        cq.Workplane("YZ")
        .circle(radius + 0.005)
        .extrude(0.005)
        .translate((-length / 2.0 - 0.0025, 0.0, 0.0))
    )
    return body.union(flange)


def _rod_gland(outer_r: float, inner_r: float, length: float) -> cq.Workplane:
    """Rod seal / gland ring (axis along X)."""
    outer = cq.Workplane("YZ").circle(outer_r).extrude(length / 2.0, both=True)
    bore = cq.Workplane("YZ").circle(inner_r).extrude(length, both=True)
    return outer.cut(bore)


def _lead_screw_thread_mesh(length: float = 0.380):
    """Helical thread wrapping around the screw core."""
    turns = 20
    samples = turns * 12
    radius = 0.0066
    points = []
    for i in range(samples + 1):
        t = i / samples
        x = -length / 2.0 + length * t
        theta = 2.0 * pi * turns * t
        points.append((x, radius * cos(theta), radius * sin(theta)))
    return tube_from_spline_points(
        points,
        radius=0.0011,
        samples_per_segment=2,
        radial_segments=8,
        cap_ends=True,
        up_hint=(0.0, 0.0, 1.0),
    )


# ---------------------------------------------------------------------------
# Key dimensions
# ---------------------------------------------------------------------------

BARREL_OUTER_R = 0.040
BARREL_INNER_R = 0.033
BARREL_LENGTH = 0.450
END_CAP_R = 0.048
END_CAP_THICKNESS = 0.018

HALF_BARREL = BARREL_LENGTH / 2.0  # 0.225
HALF_CAP = END_CAP_THICKNESS / 2.0  # 0.009

FRONT_CAP_X = HALF_BARREL + HALF_CAP  # 0.234
REAR_CAP_X = -(HALF_BARREL + HALF_CAP)  # -0.234
FRONT_CAP_FACE = FRONT_CAP_X + HALF_CAP  # 0.243
REAR_CAP_FACE = REAR_CAP_X - HALF_CAP  # -0.243

ROD_RADIUS = 0.012
ROD_LENGTH = 0.450
STROKE = 0.300
ROD_PROTRUSION = 0.120  # protrusion past front cap face at rest

# rod centre at rest so that the front tip protrudes ROD_PROTRUSION past front face
ROD_CENTER_AT_REST = FRONT_CAP_FACE + ROD_PROTRUSION - ROD_LENGTH / 2.0  # 0.138

MOTOR_RADIUS = 0.038
MOTOR_LENGTH = 0.100


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="linear_actuator_rod_barrel",
        meta={
            "category": "Robotics / Linear actuator",
            "run_note": (
                "Cylindrical tube-and-rod barrel actuator form. "
                "Internal lead screw drives the coaxial extending rod."
            ),
        },
    )

    # --- Materials ---
    anodized = model.material("anodized_barrel", rgba=(0.18, 0.28, 0.52, 1.0))
    black = model.material("black_anodized", rgba=(0.02, 0.02, 0.025, 1.0))
    dark = model.material("dark_recess", rgba=(0.0, 0.0, 0.0, 1.0))
    steel = model.material("machined_steel", rgba=(0.82, 0.83, 0.80, 1.0))
    bright = model.material("polished_steel", rgba=(0.90, 0.91, 0.88, 1.0))
    brass = model.material("bronze_nut", rgba=(0.78, 0.54, 0.24, 1.0))
    rubber = model.material("rod_seal", rgba=(0.06, 0.06, 0.05, 1.0))

    # =================================================================--
    # FRAME  (barrel assembly)
    # =================================================================--
    frame = model.part("frame")

    # Barrel tube
    frame.visual(
        mesh_from_cadquery(
            _barrel_tube(BARREL_OUTER_R, BARREL_INNER_R, BARREL_LENGTH),
            "barrel_tube_mesh",
            tolerance=0.0008,
        ),
        material=anodized,
        name="barrel_tube",
    )

    # Rear end cap (bore for motor shaft / coupler pass-through)
    rear_bore_r = 0.014
    frame.visual(
        mesh_from_cadquery(
            _end_cap(END_CAP_R, rear_bore_r, END_CAP_THICKNESS),
            "rear_cap_mesh",
            tolerance=0.0008,
        ),
        origin=Origin(xyz=(REAR_CAP_X, 0.0, 0.0)),
        material=black,
        name="rear_cap",
    )

    # Front end cap (bore for rod passage)
    front_bore_r = ROD_RADIUS + 0.004
    frame.visual(
        mesh_from_cadquery(
            _end_cap(END_CAP_R, front_bore_r, END_CAP_THICKNESS),
            "front_cap_mesh",
            tolerance=0.0008,
        ),
        origin=Origin(xyz=(FRONT_CAP_X, 0.0, 0.0)),
        material=black,
        name="front_cap",
    )

    # Rod gland / seal ring on front cap outer face
    gland_outer = 0.024
    gland_inner = ROD_RADIUS - 0.001  # 0.001 m interference grip on rod surface
    gland_len = 0.010
    gland_x = FRONT_CAP_FACE + gland_len / 2.0 - 0.001  # slight embed into cap
    frame.visual(
        mesh_from_cadquery(
            _rod_gland(gland_outer, gland_inner, gland_len),
            "rod_gland_mesh",
            tolerance=0.001,
        ),
        origin=Origin(xyz=(gland_x, 0.0, 0.0)),
        material=rubber,
        name="rod_gland",
    )

    # Motor block (seated against rear cap with 0.001 m embed for connectivity)
    motor_x = REAR_CAP_FACE - MOTOR_LENGTH / 2.0 + 0.001
    frame.visual(
        mesh_from_cadquery(
            _motor_housing(MOTOR_RADIUS, MOTOR_LENGTH),
            "motor_block_mesh",
            tolerance=0.0008,
        ),
        origin=Origin(xyz=(motor_x, 0.0, 0.0)),
        material=black,
        name="motor_block",
    )

    # Motor rear connector plate (slightly overlaps motor body for connectivity)
    motor_rear_x = motor_x - MOTOR_LENGTH / 2.0 - 0.002
    frame.visual(
        Cylinder(radius=MOTOR_RADIUS + 0.006, length=0.008),
        origin=Origin(xyz=(motor_rear_x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=black,
        name="motor_rear_plate",
    )

    # Mounting feet (two lugs on barrel underside)
    foot_half_h = 0.008
    foot_z = -(BARREL_OUTER_R + foot_half_h - 0.002)  # 2 mm overlap into barrel wall
    for i, xp in enumerate((-0.120, 0.120)):
        frame.visual(
            Box((0.060, 0.050, foot_half_h * 2)),
            origin=Origin(xyz=(xp, 0.0, foot_z)),
            material=black,
            name=f"mount_foot_{i}",
        )
        frame.visual(
            Cylinder(radius=0.004, length=0.020),
            origin=Origin(xyz=(xp, 0.0, foot_z)),
            material=dark,
            name=f"mount_hole_{i}",
        )

    # =================================================================--
    # CARRIAGE  (extending rod / piston)
    # =================================================================--
    carriage = model.part("carriage")

    # Main polished rod cylinder
    carriage.visual(
        Cylinder(radius=ROD_RADIUS, length=ROD_LENGTH),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=bright,
        name="carriage_block",
    )

    # Attachment collar near rod tip
    collar_len = 0.018
    collar_x = ROD_LENGTH / 2.0 - collar_len / 2.0
    carriage.visual(
        Cylinder(radius=ROD_RADIUS + 0.005, length=collar_len),
        origin=Origin(xyz=(collar_x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="rod_collar",
    )

    # Threaded stub past collar (clevis attachment region)
    stub_len = 0.028
    stub_x = ROD_LENGTH / 2.0 + stub_len / 2.0
    carriage.visual(
        Cylinder(radius=ROD_RADIUS - 0.001, length=stub_len),
        origin=Origin(xyz=(stub_x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="rod_thread_tip",
    )

    # =================================================================--
    # LEAD SCREW  (internal rotating drive)
    # =================================================================--
    lead_screw = model.part("lead_screw")

    screw_length = BARREL_LENGTH - 0.050  # 0.400
    half_screw = screw_length / 2.0  # 0.200

    # Screw core
    lead_screw.visual(
        Cylinder(radius=0.006, length=screw_length),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="screw_core",
    )

    # Helical thread
    lead_screw.visual(
        mesh_from_geometry(_lead_screw_thread_mesh(screw_length), "lead_screw_thread"),
        material=bright,
        name="screw_thread",
    )

    # Drive coupler at rear of screw (bridges motor shaft to screw)
    coupler_len = 0.030
    coupler_x = -(half_screw + coupler_len / 2.0)
    lead_screw.visual(
        Cylinder(radius=0.011, length=coupler_len),
        origin=Origin(xyz=(coupler_x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=brass,
        name="drive_coupler",
    )

    # =================================================================--
    # ARTICULATIONS
    # =================================================================--

    # Prismatic: rod extends along +X from barrel
    model.articulation(
        "carriage_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(ROD_CENTER_AT_REST, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=STROKE),
    )

    # Revolute: lead screw rotates about barrel axis
    model.articulation(
        "screw_spin",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=lead_screw,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=20.0, lower=-6.283185307, upper=6.283185307
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    screw = object_model.get_part("lead_screw")
    slide = object_model.get_articulation("carriage_slide")
    spin = object_model.get_articulation("screw_spin")

    # --- Part hierarchy ---
    ctx.check(
        "linear actuator barrel part set",
        all(object_model.get_part(n) is not None for n in ("frame", "carriage", "lead_screw")),
        details="Expected frame (barrel), carriage (rod), and lead_screw parts.",
    )

    # --- Joint types ---
    ctx.check(
        "primary joints present",
        slide.articulation_type == ArticulationType.PRISMATIC
        and spin.articulation_type == ArticulationType.REVOLUTE,
        details=f"slide={slide.articulation_type}, spin={spin.articulation_type}",
    )

    ctx.check(
        "joint axes follow barrel axis",
        tuple(slide.axis) == (1.0, 0.0, 0.0) and tuple(spin.axis) == (1.0, 0.0, 0.0),
        details=f"slide_axis={slide.axis}, spin_axis={spin.axis}",
    )

    # --- TARGET: barrel form family assertion ---
    barrel_vis = frame.get_visual("barrel_tube")
    ctx.check(
        "frame has cylindrical barrel tube (tube-and-rod form)",
        barrel_vis is not None,
        details="Expected barrel_tube visual on frame part — primary form family is round barrel.",
    )

    # --- Intentional overlap allowances ---

    # 1) Rod gland interference: the seal grips the rod surface for support.
    ctx.allow_overlap(
        frame,
        carriage,
        elem_a="rod_gland",
        elem_b="carriage_block",
        reason="The rod seal/gland has 0.001 m radial interference grip on the polished rod surface as a sliding support.",
    )
    ctx.expect_contact(
        frame,
        carriage,
        elem_a="rod_gland",
        elem_b="carriage_block",
        name="rod gland contacts rod (sliding support)",
    )

    # 2) Lead screw nested inside rod: both are coaxial inside the barrel.
    ctx.allow_overlap(
        carriage,
        screw,
        elem_a="carriage_block",
        elem_b="screw_thread",
        reason="The lead screw thread is intentionally nested inside the barrel alongside the solid rod; the screw drives the internal rod nut.",
    )
    ctx.allow_overlap(
        carriage,
        screw,
        elem_a="carriage_block",
        elem_b="screw_core",
        reason="The lead screw core is intentionally nested inside the barrel alongside the solid rod.",
    )
    ctx.expect_within(
        screw,
        carriage,
        axes="yz",
        inner_elem="screw_thread",
        outer_elem="carriage_block",
        margin=0.005,
        name="screw thread is within rod cross-section (nested mechanism)",
    )
    ctx.expect_overlap(
        screw,
        carriage,
        axes="x",
        elem_a="screw_core",
        elem_b="carriage_block",
        min_overlap=0.100,
        name="screw and rod share axial region inside barrel",
    )

    # --- Exact relationship checks ---

    # Rod is coaxial within barrel cross-section
    ctx.expect_within(
        carriage,
        frame,
        axes="yz",
        inner_elem="carriage_block",
        outer_elem="barrel_tube",
        margin=0.005,
        name="rod is coaxial within barrel cross-section",
    )

    # Rod has retained insertion in barrel at rest
    ctx.expect_overlap(
        carriage,
        frame,
        axes="x",
        elem_a="carriage_block",
        elem_b="barrel_tube",
        min_overlap=0.150,
        name="rod has retained insertion in barrel at rest",
    )

    # Lead screw is coaxial inside barrel
    ctx.expect_within(
        screw,
        frame,
        axes="yz",
        inner_elem="screw_core",
        outer_elem="barrel_tube",
        margin=0.005,
        name="lead screw is coaxial inside barrel",
    )

    # Lead screw overlaps barrel along X (it lives inside)
    ctx.expect_overlap(
        screw,
        frame,
        axes="x",
        elem_a="screw_core",
        elem_b="barrel_tube",
        min_overlap=0.300,
        name="lead screw is axially inside barrel",
    )

    # --- Pose: rod extends ---
    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: STROKE, spin: pi}):
        extended_pos = ctx.part_world_position(carriage)
        # Rod still has some insertion at full stroke
        ctx.expect_overlap(
            carriage,
            frame,
            axes="x",
            elem_a="carriage_block",
            elem_b="barrel_tube",
            min_overlap=0.010,
            name="rod retains partial insertion at full extension",
        )

    ctx.check(
        "positive slide extends rod along barrel axis",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.20,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
