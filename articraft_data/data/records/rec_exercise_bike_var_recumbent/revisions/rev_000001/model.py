from __future__ import annotations

# Recumbent stationary exercise bike.
# Frame: +X = front (flywheel housing), -X = rear (seat); +Z = up; +Y = left side.
#   - A long low horizontal beam (white molded) runs front-to-back.
#   - Front flywheel disc (gray center, RED accent ring) spins about Y axis, low.
#   - Crank with two pedal arms 180 deg apart, just behind the flywheel.
#   - Reclined bucket seat with upright padded backrest at the rear on prismatic slide.
#   - RED curved side grips beside the seat on beam sides.
#   - Chrome stabilizer cross-tubes (front + rear) with black end caps.
# Articulations:
#   - flywheel: CONTINUOUS about Y.
#   - crank: CONTINUOUS about Y.
#   - left/right pedal: CONTINUOUS about Y spindle.
#   - seat_carriage: PRISMATIC along X (~0.12 m travel).

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---- Key reference coordinates (meters) ----
BEAM_FRONT_X = 0.50
BEAM_REAR_X = -0.56
BEAM_LENGTH = BEAM_FRONT_X - BEAM_REAR_X  # ~1.06m
BEAM_Z_BOT = 0.10
BEAM_Z_TOP = 0.22
BEAM_HALF_W = 0.046  # half-width along Y

FLYWHEEL_X = 0.38
FLYWHEEL_Z = 0.22
FLYWHEEL_R = 0.150
FLYWHEEL_HALF_W = 0.020
FLYWHEEL_Y = 0.055  # flywheel center Y (just outside beam surface)

CRANK_X = 0.14
CRANK_Z = 0.19
CRANK_ARM_LEN = 0.085
CRANK_ARM_Y = 0.090
PEDAL_Y = 0.155

SEAT_X = -0.36
SEAT_RAIL_Z_TOP = BEAM_Z_TOP + 0.010  # top of seat rail
CARRIAGE_PLATE_Z = SEAT_RAIL_Z_TOP + 0.008  # carriage plate center above rail

GRIP_X = -0.28
GRIP_Z = BEAM_Z_TOP


def _beam_solid() -> cq.Workplane:
    """Long low horizontal beam: extruded side profile in XZ, thickness along Y."""
    profile_pts = [
        (BEAM_FRONT_X, BEAM_Z_BOT + 0.02),
        (BEAM_FRONT_X + 0.01, BEAM_Z_BOT + 0.06),
        (BEAM_FRONT_X - 0.02, BEAM_Z_TOP + 0.02),
        (BEAM_FRONT_X - 0.10, BEAM_Z_TOP + 0.01),
        (0.20, BEAM_Z_TOP),
        (0.0, BEAM_Z_TOP),
        (-0.20, BEAM_Z_TOP),
        (-0.40, BEAM_Z_TOP + 0.005),
        (BEAM_REAR_X + 0.06, BEAM_Z_TOP + 0.01),
        (BEAM_REAR_X + 0.02, BEAM_Z_TOP),
        (BEAM_REAR_X, BEAM_Z_TOP - 0.02),
        (BEAM_REAR_X, BEAM_Z_BOT + 0.02),
        (BEAM_REAR_X + 0.04, BEAM_Z_BOT),
        (-0.40, BEAM_Z_BOT - 0.005),
        (-0.20, BEAM_Z_BOT),
        (0.0, BEAM_Z_BOT),
        (0.20, BEAM_Z_BOT),
        (BEAM_FRONT_X - 0.10, BEAM_Z_BOT),
        (BEAM_FRONT_X - 0.02, BEAM_Z_BOT + 0.01),
    ]
    wire = cq.Workplane("XZ").moveTo(*profile_pts[0])
    for p in profile_pts[1:]:
        wire = wire.lineTo(*p)
    base = wire.close().extrude(BEAM_HALF_W, both=True)
    try:
        base = base.edges("|Y").fillet(0.014)
    except Exception:
        pass
    return base


def _flywheel_mesh():
    disc = CylinderGeometry(FLYWHEEL_R * 0.86, FLYWHEEL_HALF_W * 2.0 * 0.7).rotate_x(
        math.pi / 2.0
    )
    hub = CylinderGeometry(0.036, FLYWHEEL_HALF_W * 2.0 * 1.1).rotate_x(math.pi / 2.0)
    disc.merge(hub)
    return mesh_from_geometry(disc, "flywheel_disc")


def _flywheel_ring_mesh():
    ring = TorusGeometry(
        FLYWHEEL_R * 0.92, 0.011, radial_segments=18, tubular_segments=64
    ).rotate_x(math.pi / 2.0)
    return mesh_from_geometry(ring, "flywheel_red_ring")


def _bucket_seat_mesh():
    """Reclined bucket seat pan: loft of YZ cross-sections along X."""
    wp = cq.Workplane("YZ")
    sections = [
        (0.12, 0.14, 0.014),
        (0.08, 0.15, 0.018),
        (0.04, 0.16, 0.022),
        (0.0, 0.17, 0.025),
        (-0.04, 0.17, 0.028),
        (-0.08, 0.16, 0.030),
        (-0.12, 0.15, 0.026),
    ]
    prev_x = sections[0][0]
    for i, (x, hw, hh) in enumerate(sections):
        dx = x - prev_x
        if i > 0:
            wp = wp.workplane(offset=dx)
        wp = wp.rect(2.0 * hw, 2.0 * hh)
        prev_x = x
    seat_pan = wp.loft(ruled=False)
    try:
        seat_pan = seat_pan.edges("|X").fillet(0.008)
    except Exception:
        pass
    return seat_pan


def _backrest_mesh():
    """Upright padded backrest: loft of XY cross-sections along Z.
    Slight recline: top is offset rearward (-X)."""
    wp = cq.Workplane("XY")
    sections = [
        (0.0, 0.13, 0.018, 0.0),
        (0.05, 0.12, 0.020, -0.015),
        (0.10, 0.11, 0.022, -0.030),
        (0.16, 0.10, 0.023, -0.045),
        (0.22, 0.09, 0.022, -0.055),
        (0.26, 0.085, 0.020, -0.060),
    ]
    prev_z = sections[0][0]
    for i, (z, hw, hh, xc) in enumerate(sections):
        dz = z - prev_z
        if i > 0:
            wp = wp.workplane(offset=dz)
        wp = wp.rect(2.0 * hw, 2.0 * hh)
        prev_z = z
    backrest = wp.loft(ruled=False)
    try:
        backrest = backrest.edges("|Z").fillet(0.007)
    except Exception:
        pass
    return backrest


def _side_grip_mesh(side: float):
    """RED curved side grip bar. Starts at beam surface, curves out and up."""
    pts = [
        (0.0, 0.0, 0.0),
        (0.02, side * 0.04, 0.03),
        (0.0, side * 0.07, 0.08),
        (-0.04, side * 0.08, 0.13),
        (-0.06, side * 0.08, 0.17),
    ]
    bar = tube_from_spline_points(
        pts, radius=0.013, samples_per_segment=12, radial_segments=12
    )
    return mesh_from_geometry(bar, f"side_grip_bar")


def _foot_mesh(length_y: float):
    tube = CylinderGeometry(0.018, length_y).rotate_x(math.pi / 2.0)
    return mesh_from_geometry(tube, "stabilizer_tube")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="recumbent_exercise_bike")

    white = model.material("body_white", rgba=(0.93, 0.93, 0.94, 1.0))
    red = model.material("accent_red", rgba=(0.82, 0.10, 0.12, 1.0))
    gray = model.material("part_gray", rgba=(0.45, 0.45, 0.48, 1.0))
    dark = model.material("dark_gray", rgba=(0.20, 0.20, 0.22, 1.0))
    chrome = model.material("chrome", rgba=(0.78, 0.80, 0.83, 1.0))
    black = model.material("cap_black", rgba=(0.10, 0.10, 0.11, 1.0))
    pad_gray = model.material("seat_pad", rgba=(0.35, 0.35, 0.38, 1.0))

    # ========================= BODY (root) =========================
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_beam_solid(), "body_beam"),
        material=white,
        name="body_beam",
    )

    # Flywheel housing: dark cylindrical boss on the beam at the front.
    # Positioned to overlap with the beam in Z so it's connected.
    fw_housing = CylinderGeometry(FLYWHEEL_R * 0.40, BEAM_HALF_W * 2.2).rotate_x(
        math.pi / 2.0
    )
    fw_housing.translate(FLYWHEEL_X, 0.0, FLYWHEEL_Z)
    body.visual(
        mesh_from_geometry(fw_housing, "flywheel_housing"),
        material=dark,
        name="flywheel_housing",
    )

    # Flywheel axle stub: a short chrome cylinder along Y from beam surface
    # out to the flywheel mounting point.
    axle_len = FLYWHEEL_Y - BEAM_HALF_W + 0.01
    axle = CylinderGeometry(0.012, axle_len).rotate_x(math.pi / 2.0)
    axle.translate(FLYWHEEL_X, BEAM_HALF_W + axle_len / 2.0 - 0.01, FLYWHEEL_Z)
    body.visual(
        mesh_from_geometry(axle, "flywheel_axle"),
        material=chrome,
        name="flywheel_axle",
    )

    # Crank axle boss: dark hub band on the beam where the crank seats.
    boss = CylinderGeometry(0.026, BEAM_HALF_W * 2.4).rotate_x(math.pi / 2.0)
    boss.translate(CRANK_X, 0.0, CRANK_Z)
    body.visual(
        mesh_from_geometry(boss, "crank_boss"),
        material=dark,
        name="crank_boss",
    )

    # Seat rail: dark track on beam top for the carriage to slide on.
    rail_len = 0.28
    rail = BoxGeometry((rail_len, 0.044, 0.010))
    rail.translate(SEAT_X + 0.02, 0.0, BEAM_Z_TOP + 0.005)
    body.visual(
        mesh_from_geometry(rail, "seat_rail"),
        material=dark,
        name="seat_rail",
    )

    body.inertial = Inertial.from_geometry(
        Box((BEAM_LENGTH, BEAM_HALF_W * 2.0, BEAM_Z_TOP - BEAM_Z_BOT + 0.02)),
        mass=16.0,
        origin=Origin(xyz=((BEAM_FRONT_X + BEAM_REAR_X) / 2.0, 0.0, (BEAM_Z_BOT + BEAM_Z_TOP) / 2.0)),
    )

    # ========================= STABILIZER FEET =========================
    # Front + rear chrome cross-tubes. Each foot part frame is at its
    # articulation origin (on the beam bottom), visuals are local offsets.
    foot_specs = (
        ("front_foot", 0.36, 0.48),
        ("rear_foot", -0.44, 0.48),
    )
    for fname, fx, flen in foot_specs:
        foot = model.part(fname)
        # Cross tube on the ground (local z=0.022 - beam_bottom_z).
        ground_z = 0.022 - BEAM_Z_BOT  # in foot's local frame (origin at beam bottom)
        foot.visual(
            _foot_mesh(flen),
            origin=Origin(xyz=(0.0, 0.0, ground_z)),
            material=chrome,
            name="stabilizer_tube",
        )
        # Vertical leg from ground tube up to beam bottom.
        leg_h = BEAM_Z_BOT - 0.022 + 0.04  # from ground to beam bottom + insertion
        leg = CylinderGeometry(0.018, leg_h)
        leg.translate(0.0, 0.0, ground_z + leg_h / 2.0)
        foot.visual(
            mesh_from_geometry(leg, "foot_leg"),
            material=chrome,
            name="foot_leg",
        )
        # Black end caps + ground pads at each tube end.
        for i, cy in enumerate((flen / 2.0, -flen / 2.0)):
            cap = CylinderGeometry(0.024, 0.028).rotate_x(math.pi / 2.0)
            cap.translate(0.0, cy - math.copysign(0.012, cy), ground_z)
            foot.visual(
                mesh_from_geometry(cap, f"foot_cap_{i}"),
                material=black,
                name=f"foot_cap_{i}",
            )
            pad = BoxGeometry((0.038, 0.048, 0.020))
            pad.translate(0.0, cy - math.copysign(0.006, cy), ground_z - 0.010)
            foot.visual(
                mesh_from_geometry(pad, f"foot_pad_{i}"),
                material=black,
                name=f"foot_pad_{i}",
            )
        foot.inertial = Inertial.from_geometry(
            Box((0.05, flen, BEAM_Z_BOT)),
            mass=1.5,
            origin=Origin(xyz=(0.0, 0.0, ground_z + BEAM_Z_BOT / 2.0)),
        )
        model.articulation(
            f"body_to_{fname}",
            ArticulationType.FIXED,
            parent=body,
            child=foot,
            origin=Origin(xyz=(fx, 0.0, BEAM_Z_BOT)),
        )

    # ========================= FLYWHEEL =========================
    flywheel = model.part("flywheel")
    flywheel.visual(_flywheel_mesh(), material=gray, name="flywheel_disc")
    flywheel.visual(_flywheel_ring_mesh(), material=red, name="flywheel_red_ring")
    # Off-axis marker bolt.
    marker = CylinderGeometry(0.010, FLYWHEEL_HALF_W * 2.0 * 1.2).rotate_x(
        math.pi / 2.0
    )
    marker.translate(FLYWHEEL_R * 0.55, 0.0, 0.0)
    flywheel.visual(
        mesh_from_geometry(marker, "flywheel_marker"),
        material=dark,
        name="flywheel_marker",
    )
    flywheel.inertial = Inertial.from_geometry(
        Cylinder(FLYWHEEL_R, 2.0 * FLYWHEEL_HALF_W), mass=4.0,
    )
    model.articulation(
        "body_to_flywheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=flywheel,
        origin=Origin(xyz=(FLYWHEEL_X, FLYWHEEL_Y, FLYWHEEL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=20.0),
    )

    # ========================= CRANK + PEDALS =========================
    crank = model.part("crank")
    hub = CylinderGeometry(0.015, 2.0 * CRANK_ARM_Y).rotate_x(math.pi / 2.0)
    crank_geo = hub
    for sgn, ay in ((1.0, CRANK_ARM_Y), (-1.0, -CRANK_ARM_Y)):
        arm = BoxGeometry((0.024, 0.020, CRANK_ARM_LEN))
        arm.translate(0.0, ay, sgn * CRANK_ARM_LEN / 2.0)
        crank_geo = crank_geo.merge(arm)
    crank.visual(
        mesh_from_geometry(crank_geo, "crank_body"),
        material=dark,
        name="crank_body",
    )
    crank.inertial = Inertial.from_geometry(
        Box((0.06, 0.20, 0.18)), mass=1.2
    )
    model.articulation(
        "body_to_crank",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=crank,
        origin=Origin(xyz=(CRANK_X, 0.0, CRANK_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=12.0),
    )

    # Pedals.
    pedal_specs = (
        ("left_pedal", CRANK_ARM_Y, CRANK_ARM_LEN, PEDAL_Y),
        ("right_pedal", -CRANK_ARM_Y, -CRANK_ARM_LEN, -PEDAL_Y),
    )
    for pname, tip_y, tip_z, plat_y in pedal_specs:
        pedal = model.part(pname)
        out = plat_y - tip_y
        plat = BoxGeometry((0.088, 0.058, 0.014))
        plat.translate(0.0, out, 0.0)
        spindle = CylinderGeometry(0.008, abs(out) + 0.02).rotate_x(math.pi / 2.0)
        spindle.translate(0.0, out / 2.0, 0.0)
        plat = plat.merge(spindle)
        ridge = BoxGeometry((0.012, 0.058, 0.018))
        ridge.translate(0.033, out, 0.006)
        plat = plat.merge(ridge)
        pedal.visual(
            mesh_from_geometry(plat, "pedal_tread"),
            material=dark,
            name="pedal_tread",
        )
        pedal.inertial = Inertial.from_geometry(
            Box((0.09, 0.06, 0.03)), mass=0.25, origin=Origin(xyz=(0.0, out, 0.0))
        )
        model.articulation(
            f"crank_to_{pname}",
            ArticulationType.CONTINUOUS,
            parent=crank,
            child=pedal,
            origin=Origin(xyz=(0.0, tip_y, tip_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=8.0),
        )

    # ========================= SEAT CARRIAGE (prismatic) =========================
    seat_carriage = model.part("seat_carriage")

    # Carriage plate rides on top of the seat rail.
    carriage_plate = BoxGeometry((0.16, 0.060, 0.014))
    carriage_plate.translate(0.0, 0.0, 0.007)
    seat_carriage.visual(
        mesh_from_geometry(carriage_plate, "carriage_plate"),
        material=dark,
        name="carriage_plate",
    )

    # Seat post: short chrome riser from carriage to bucket seat.
    seat_post_h = 0.08
    seat_post = CylinderGeometry(0.020, seat_post_h)
    seat_post.translate(0.02, 0.0, 0.014 + seat_post_h / 2.0)
    seat_carriage.visual(
        mesh_from_geometry(seat_post, "seat_post"),
        material=chrome,
        name="seat_post",
    )

    # Bucket seat cushion (gray padded).
    seat_carriage.visual(
        mesh_from_cadquery(_bucket_seat_mesh(), "bucket_seat"),
        origin=Origin(xyz=(0.0, 0.0, 0.014 + seat_post_h + 0.01)),
        material=pad_gray,
        name="bucket_seat",
    )

    # Backrest support bracket: chrome tube from carriage up behind the seat.
    bracket_h = 0.26
    bracket = CylinderGeometry(0.012, bracket_h)
    bracket.translate(-0.10, 0.0, 0.014 + bracket_h / 2.0)
    seat_carriage.visual(
        mesh_from_geometry(bracket, "backrest_bracket"),
        material=chrome,
        name="backrest_bracket",
    )

    # Backrest pad mounted at top of bracket, behind the seat.
    seat_carriage.visual(
        mesh_from_cadquery(_backrest_mesh(), "backrest_pad"),
        origin=Origin(xyz=(-0.10, 0.0, 0.014 + 0.04)),
        material=pad_gray,
        name="backrest_pad",
    )

    seat_carriage.inertial = Inertial.from_geometry(
        Box((0.22, 0.20, 0.30)),
        mass=3.5,
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
    )

    model.articulation(
        "body_to_seat_carriage",
        ArticulationType.PRISMATIC,
        parent=body,
        child=seat_carriage,
        origin=Origin(xyz=(SEAT_X, 0.0, SEAT_RAIL_Z_TOP)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.1, lower=-0.06, upper=0.06),
    )

    # ========================= SIDE GRIPS =========================
    # Two red curved side grips mounted on beam sides beside the seat.
    for i, side in enumerate((1.0, -1.0)):
        grip_name = f"side_grip_{i}"
        grip = model.part(grip_name)
        # Mount stub: short chrome cylinder from beam surface outward.
        stub_len = 0.04
        stub = CylinderGeometry(0.011, stub_len).rotate_x(math.pi / 2.0)
        stub.translate(0.0, side * stub_len / 2.0, 0.0)
        grip.visual(
            mesh_from_geometry(stub, "grip_stub"),
            material=chrome,
            name="grip_stub",
        )
        # Curved red grip bar starting at the stub tip.
        grip.visual(
            _side_grip_mesh(side),
            origin=Origin(xyz=(0.0, side * stub_len, 0.0)),
            material=red,
            name=f"grip_bar_{i}",
        )
        grip.inertial = Inertial.from_geometry(
            Box((0.12, 0.10, 0.18)), mass=0.4, origin=Origin(xyz=(0.0, side * 0.05, 0.06))
        )
        model.articulation(
            f"body_to_{grip_name}",
            ArticulationType.FIXED,
            parent=body,
            child=grip,
            origin=Origin(xyz=(GRIP_X, side * BEAM_HALF_W, GRIP_Z)),
        )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    flywheel = object_model.get_part("flywheel")
    crank = object_model.get_part("crank")
    left_pedal = object_model.get_part("left_pedal")
    right_pedal = object_model.get_part("right_pedal")
    seat_carriage = object_model.get_part("seat_carriage")
    front_foot = object_model.get_part("front_foot")
    rear_foot = object_model.get_part("rear_foot")

    fly_joint = object_model.get_articulation("body_to_flywheel")
    crank_joint = object_model.get_articulation("body_to_crank")
    lpedal_joint = object_model.get_articulation("crank_to_left_pedal")
    seat_joint = object_model.get_articulation("body_to_seat_carriage")

    # ---- Intentional seated overlaps ----
    ctx.allow_overlap(
        crank, body,
        reason="Crank hub passes through the beam and seats on the dark axle boss; intentional mounting.",
    )
    ctx.allow_overlap(
        flywheel, body,
        reason="Flywheel disc is seated on the axle stub protruding from the beam flywheel housing; intentional mounting.",
    )
    ctx.allow_overlap(
        left_pedal, crank,
        reason="Left pedal spindle is intentionally captured on the crank arm tip.",
    )
    ctx.allow_overlap(
        right_pedal, crank,
        reason="Right pedal spindle is intentionally captured on the crank arm tip.",
    )
    ctx.allow_overlap(
        seat_carriage, body,
        elem_a="carriage_plate", elem_b="seat_rail",
        reason="Seat carriage plate rides on top of the seat rail; intentional sliding contact.",
    )
    ctx.allow_overlap(
        front_foot, body,
        elem_a="foot_leg", elem_b="body_beam",
        reason="Front stabilizer down-leg intentionally enters the beam bottom to mount.",
    )
    ctx.allow_overlap(
        rear_foot, body,
        elem_a="foot_leg", elem_b="body_beam",
        reason="Rear stabilizer down-leg intentionally enters the beam bottom to mount.",
    )

    # ---- Frame is a long horizontal beam (recumbent posture) ----
    body_aabb = ctx.part_world_aabb(body)
    body_dx = body_aabb[1][0] - body_aabb[0][0]
    body_dz = body_aabb[1][2] - body_aabb[0][2]
    ctx.check(
        "frame is a long horizontal beam (recumbent posture)",
        body_dx > 0.80 and body_dx > body_dz * 4.0,
        details=f"beam_length_x={body_dx:.3f}, beam_height_z={body_dz:.3f}",
    )

    # ---- Flywheel is low at the front ----
    fly_aabb = ctx.part_world_aabb(flywheel)
    fly_cx = (fly_aabb[0][0] + fly_aabb[1][0]) / 2.0
    fly_cz = (fly_aabb[0][2] + fly_aabb[1][2]) / 2.0
    ctx.check(
        "flywheel is at the front of the beam",
        fly_cx > 0.20,
        details=f"flywheel_center_x={fly_cx:.3f}",
    )
    ctx.check(
        "flywheel is low (recumbent, not elevated)",
        fly_cz < 0.40,
        details=f"flywheel_center_z={fly_cz:.3f}",
    )

    # ---- Seat carriage is at the rear ----
    seat_aabb = ctx.part_world_aabb(seat_carriage)
    seat_cx = (seat_aabb[0][0] + seat_aabb[1][0]) / 2.0
    ctx.check(
        "seat carriage is at the rear of the beam",
        seat_cx < -0.10,
        details=f"seat_carriage_center_x={seat_cx:.3f}",
    )

    # ---- Seat has a backrest (tall vertical extent) ----
    seat_dz = seat_aabb[1][2] - seat_aabb[0][2]
    ctx.check(
        "seat carriage includes a tall backrest",
        seat_dz > 0.18,
        details=f"seat_carriage_height_z={seat_dz:.3f}",
    )

    # ---- Flywheel spins about its side (Y) axis ----
    m0 = ctx.part_element_world_aabb(flywheel, elem="flywheel_marker")
    mc0 = ((m0[0][0] + m0[1][0]) / 2.0, (m0[0][2] + m0[1][2]) / 2.0)
    with ctx.pose({fly_joint: math.pi / 2.0}):
        m1 = ctx.part_element_world_aabb(flywheel, elem="flywheel_marker")
        mc1 = ((m1[0][0] + m1[1][0]) / 2.0, (m1[0][2] + m1[1][2]) / 2.0)
    ctx.check(
        "flywheel marker revolves about the side axis",
        abs(mc1[1] - mc0[1]) > 0.03 and abs(mc1[0] - mc0[0]) > 0.03,
        details=f"marker XZ rest={mc0}, quarter-turn={mc1}",
    )

    # ---- Crank rotates: pedal revolves ----
    p0 = ctx.part_world_position(left_pedal)
    with ctx.pose({crank_joint: math.pi / 2.0}):
        p1 = ctx.part_world_position(left_pedal)
    ctx.check(
        "crank rotation revolves the pedal about the crank axis",
        p0 is not None and p1 is not None
        and (abs(p1[0] - p0[0]) > 0.03 or abs(p1[2] - p0[2]) > 0.03),
        details=f"left pedal rest={p0}, crank quarter-turn={p1}",
    )

    # ---- Pedal spins on its spindle ----
    e0 = _ext(ctx.part_world_aabb(left_pedal))
    with ctx.pose({lpedal_joint: math.pi / 2.0}):
        e1 = _ext(ctx.part_world_aabb(left_pedal))
    ctx.check(
        "left pedal spins on its spindle",
        abs(e1[0] - e0[0]) > 0.01 or abs(e1[2] - e0[2]) > 0.01,
        details=f"pedal extents rest={e0}, spun={e1}",
    )

    # ---- Seat carriage slides along X ----
    s0 = ctx.part_world_position(seat_carriage)
    with ctx.pose({seat_joint: 0.06}):
        s1 = ctx.part_world_position(seat_carriage)
    ctx.check(
        "seat carriage slides forward along X",
        s0 is not None and s1 is not None and s1[0] > s0[0] + 0.04,
        details=f"seat rest_x={s0}, slid_x={s1}",
    )
    with ctx.pose({seat_joint: -0.06}):
        s2 = ctx.part_world_position(seat_carriage)
    ctx.check(
        "seat carriage slides rearward along X",
        s0 is not None and s2 is not None and s2[0] < s0[0] - 0.04,
        details=f"seat rest_x={s0}, rear_x={s2}",
    )

    # ---- Seat carriage stays on the beam at rest ----
    ctx.expect_overlap(
        seat_carriage, body, axes="x",
        elem_a="carriage_plate", elem_b="body_beam",
        min_overlap=0.04,
        name="seat carriage retained on beam at rest",
    )

    # ---- Feet on ground, widest footprint ----
    ff = ctx.part_world_aabb(front_foot)
    rf = ctx.part_world_aabb(rear_foot)
    foot_min_z = min(ff[0][2], rf[0][2])
    ctx.check(
        "stabilizer feet rest near the ground plane",
        foot_min_z < 0.06,
        details=f"foot_min_z={foot_min_z:.3f}",
    )
    foot_span_y = max(ff[1][1], rf[1][1]) - min(ff[0][1], rf[0][1])
    body_span_y = _ext(ctx.part_world_aabb(body))[1]
    ctx.check(
        "feet are the widest footprint",
        foot_span_y >= body_span_y - 0.01,
        details=f"foot_span_y={foot_span_y:.3f}, body_span_y={body_span_y:.3f}",
    )
    ctx.check(
        "front foot is forward of rear foot",
        (ff[0][0] + ff[1][0]) / 2.0 > (rf[0][0] + rf[1][0]) / 2.0,
        details=f"front_cx={(ff[0][0]+ff[1][0])/2.0:.3f}, rear_cx={(rf[0][0]+rf[1][0])/2.0:.3f}",
    )

    # ---- Recumbent layout: flywheel forward of crank, crank forward of seat ----
    crank_aabb = ctx.part_world_aabb(crank)
    crank_cx = (crank_aabb[0][0] + crank_aabb[1][0]) / 2.0
    ctx.check(
        "flywheel is forward of the crank",
        fly_cx > crank_cx + 0.05,
        details=f"flywheel_x={fly_cx:.3f}, crank_x={crank_cx:.3f}",
    )
    ctx.check(
        "crank is forward of the seat carriage",
        crank_cx > seat_cx + 0.10,
        details=f"crank_x={crank_cx:.3f}, seat_x={seat_cx:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
