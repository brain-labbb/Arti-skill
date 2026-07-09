from __future__ import annotations

# Racing go-kart.
# Frame convention:
#   +Y = forward (front of the kart), -Y = rear.
#   +X = kart's left, -X = kart's right.
#   +Z = up; ground plane at z=0, wheel centers at z = tire_radius.
# Hero forms: low tubular steel frame, pink/red side pods + front fairing,
# low flat sling seat with short back panel, small steering wheel on an angled
# column, four fat slick tires (smaller/narrower front, larger/wider rear) on
# exposed axles, number "5" decals.
# Articulations:
#   - steering wheel: CONTINUOUS spin about its angled column axis.
#   - front-left knuckle: REVOLUTE steer (vertical); its wheel: CONTINUOUS roll (child).
#   - front-right knuckle: REVOLUTE steer (vertical); its wheel: CONTINUOUS roll (child).
#   - rear-left wheel: CONTINUOUS roll.
#   - rear-right wheel: CONTINUOUS roll.

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    superellipse_side_loft,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---- key dimensions ----
FRONT_AXLE_Y = 0.74
REAR_AXLE_Y = -0.74
FRONT_TRACK_X = 0.46  # half-track to each front wheel center
REAR_TRACK_X = 0.50  # half-track to each rear wheel center (wider rear)

FRONT_TIRE_R = 0.115
FRONT_TIRE_W = 0.140
REAR_TIRE_R = 0.140
REAR_TIRE_W = 0.230

AXLE_Z_FRONT = FRONT_TIRE_R
AXLE_Z_REAR = REAR_TIRE_R

FRAME_Z = 0.085  # low tubular frame height off the ground
COLUMN_TILT = 0.55  # steering column lean from vertical (top leans back toward driver, shaft runs down-forward to the front axle)


def _mesh(name, geom):
    return mesh_from_geometry(geom, name)


def _mirror_x(points):
    return [(-x, y, z) for x, y, z in points]


def _tire_mesh(name, tire_radius, tire_width):
    # Fat slick tire as a lathed cross-section revolved about the lateral axle.
    # The profile is built in (radius, x_along_axle) and the lathe is rotated so
    # its axis lies along local X (the wheel's spin/axle axis).
    hw = tire_width * 0.5
    profile = [
        (tire_radius * 0.46, -hw * 0.98),
        (tire_radius * 0.74, -hw * 1.00),
        (tire_radius * 0.92, -hw * 0.86),
        (tire_radius * 0.99, -hw * 0.50),
        (tire_radius, -hw * 0.14),
        (tire_radius, hw * 0.14),
        (tire_radius * 0.99, hw * 0.50),
        (tire_radius * 0.92, hw * 0.86),
        (tire_radius * 0.74, hw * 1.00),
        (tire_radius * 0.46, hw * 0.98),
        (tire_radius * 0.40, hw * 0.30),
        (tire_radius * 0.38, 0.0),
        (tire_radius * 0.40, -hw * 0.30),
        (tire_radius * 0.46, -hw * 0.98),
    ]
    return _mesh(name, LatheGeometry(profile, segments=56).rotate_y(pi / 2.0))


def _wheel_visuals(part, prefix, tire_radius, tire_width, rubber, rim_mat, hub_mat, marker_mat):
    # Tire, dished rim, hub, exposed axle stub, and a small off-axis marker bolt
    # so the wheel's spin is visually detectable.
    spin_rpy = Origin(rpy=(0.0, pi / 2.0, 0.0))  # cylinder axis -> local X
    part.visual(_tire_mesh(f"{prefix}_tire.obj", tire_radius, tire_width), material=rubber)

    # Rim barrel + outer dished face.
    rim_r = tire_radius * 0.58
    part.visual(
        Cylinder(radius=rim_r, length=tire_width * 0.80),
        origin=spin_rpy,
        material=rim_mat,
        name=f"{prefix}_rim",
    )
    # Hub center.
    part.visual(
        Cylinder(radius=tire_radius * 0.30, length=tire_width * 0.92),
        origin=spin_rpy,
        material=hub_mat,
        name=f"{prefix}_hub",
    )
    # Exposed axle stub poking inward toward the chassis center (-X for left side
    # handled by caller mirroring through marker; keep symmetric stub both ways).
    part.visual(
        Cylinder(radius=tire_radius * 0.13, length=tire_width * 1.6),
        origin=spin_rpy,
        material=hub_mat,
        name=f"{prefix}_axle",
    )
    # Off-axis marker lug near the rim so rolling is detectable.
    mr = tire_radius * 0.62
    part.visual(
        Box((tire_width * 0.55, tire_radius * 0.12, tire_radius * 0.12)),
        origin=Origin(xyz=(0.0, mr, 0.0)),
        material=marker_mat,
        name=f"{prefix}_marker",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="racing_go_kart")

    steel = model.material("frame_steel", rgba=(0.72, 0.73, 0.76, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.26, 0.27, 0.29, 1.0))
    pink = model.material("body_pink", rgba=(0.92, 0.28, 0.42, 1.0))
    red = model.material("body_red", rgba=(0.80, 0.14, 0.16, 1.0))
    rubber = model.material("tire_rubber", rgba=(0.06, 0.06, 0.07, 1.0))
    rim_mat = model.material("rim_silver", rgba=(0.60, 0.61, 0.64, 1.0))
    seat_mat = model.material("seat_gray", rgba=(0.82, 0.82, 0.84, 1.0))
    decal = model.material("decal_white", rgba=(0.96, 0.96, 0.97, 1.0))
    marker_mat = model.material("marker_yellow", rgba=(0.95, 0.80, 0.15, 1.0))

    # ================= chassis (root) =================
    chassis = model.part("chassis")
    chassis.inertial = Inertial.from_geometry(
        Box((1.00, 1.70, 0.40)),
        mass=85.0,
        origin=Origin(xyz=(0.0, -0.05, 0.18)),
    )

    frame_z = FRAME_Z

    # --- main side rails running front-to-rear ---
    for nm, pts in (
        (
            "left_main_rail",
            [
                (FRONT_TRACK_X - 0.10, FRONT_AXLE_Y, frame_z),
                (0.30, 0.45, frame_z),
                (0.30, -0.10, frame_z),
                (0.32, -0.50, frame_z),
                (REAR_TRACK_X - 0.06, REAR_AXLE_Y, frame_z + 0.02),
            ],
        ),
        (
            "right_main_rail",
            _mirror_x(
                [
                    (FRONT_TRACK_X - 0.10, FRONT_AXLE_Y, frame_z),
                    (0.30, 0.45, frame_z),
                    (0.30, -0.10, frame_z),
                    (0.32, -0.50, frame_z),
                    (REAR_TRACK_X - 0.06, REAR_AXLE_Y, frame_z + 0.02),
                ]
            ),
        ),
    ):
        rail = tube_from_spline_points(pts, radius=0.022, samples_per_segment=12, radial_segments=14)
        chassis.visual(_mesh(f"{nm}.obj", rail), material=steel)

    # --- center spine tube ---
    spine = tube_from_spline_points(
        [(0.0, 0.70, frame_z + 0.01), (0.0, 0.20, frame_z), (0.0, -0.30, frame_z), (0.0, REAR_AXLE_Y, frame_z + 0.02)],
        radius=0.020,
        samples_per_segment=14,
        radial_segments=14,
    )
    chassis.visual(_mesh("center_spine.obj", spine), material=steel)

    # --- front cross tube (steering axle support) ---
    front_cross = tube_from_spline_points(
        [(-FRONT_TRACK_X, FRONT_AXLE_Y, frame_z + 0.01), (0.0, FRONT_AXLE_Y + 0.02, frame_z + 0.03), (FRONT_TRACK_X, FRONT_AXLE_Y, frame_z + 0.01)],
        radius=0.020,
        samples_per_segment=12,
        radial_segments=14,
    )
    chassis.visual(_mesh("front_cross_tube.obj", front_cross), material=steel)

    # --- rear cross tube + solid rear axle line ---
    rear_cross = tube_from_spline_points(
        [(-REAR_TRACK_X, REAR_AXLE_Y, frame_z + 0.02), (0.0, REAR_AXLE_Y - 0.02, frame_z + 0.02), (REAR_TRACK_X, REAR_AXLE_Y, frame_z + 0.02)],
        radius=0.022,
        samples_per_segment=12,
        radial_segments=14,
    )
    chassis.visual(_mesh("rear_cross_tube.obj", rear_cross), material=steel)
    # Live rear axle bar between the rear wheels (exposed axle).
    chassis.visual(
        Cylinder(radius=0.018, length=REAR_TRACK_X * 2.0 - 0.02),
        origin=Origin(xyz=(0.0, REAR_AXLE_Y, AXLE_Z_REAR), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="rear_axle_bar",
    )

    # --- a couple of diagonal braces (rail to center spine) ---
    for nm, pts in (
        ("left_brace", [(0.30, 0.20, frame_z), (0.16, 0.05, frame_z), (0.0, -0.05, frame_z)]),
        ("right_brace", [(-0.30, 0.20, frame_z), (-0.16, 0.05, frame_z), (0.0, -0.05, frame_z)]),
    ):
        br = tube_from_spline_points(pts, radius=0.014, samples_per_segment=8, radial_segments=12)
        chassis.visual(_mesh(f"{nm}.obj", br), material=steel)

    # --- side pods (pink bodywork) ---
    # superellipse_side_loft sweeps along +Y; sections are (y, z_min, z_max, width_x)
    # centered at x=0. Build one pod centered, then translate to each side.
    pod_sections = [
        (0.30, frame_z - 0.01, frame_z + 0.12, 0.22),
        (0.02, frame_z - 0.02, frame_z + 0.15, 0.26),
        (-0.26, frame_z - 0.01, frame_z + 0.12, 0.22),
    ]
    left_pod = superellipse_side_loft(pod_sections, exponents=2.6, segments=40).translate(0.40, 0.0, 0.0)
    right_pod = superellipse_side_loft(pod_sections, exponents=2.6, segments=40).translate(-0.40, 0.0, 0.0)
    chassis.visual(_mesh("left_side_pod.obj", left_pod), material=pink)
    chassis.visual(_mesh("right_side_pod.obj", right_pod), material=pink)

    # number "5" decals on the side pods (white block standing in for the decal).
    chassis.visual(
        Box((0.02, 0.10, 0.12)),
        origin=Origin(xyz=(0.515, 0.02, frame_z + 0.06)),
        material=decal,
        name="left_number_5",
    )
    chassis.visual(
        Box((0.02, 0.10, 0.12)),
        origin=Origin(xyz=(-0.515, 0.02, frame_z + 0.06)),
        material=decal,
        name="right_number_5",
    )

    # --- front fairing / nose cone (red), wide and low across the front ---
    nose = superellipse_side_loft(
        [
            (FRONT_AXLE_Y - 0.02, frame_z - 0.01, frame_z + 0.10, 0.60),
            (FRONT_AXLE_Y + 0.16, frame_z - 0.01, frame_z + 0.11, 0.70),
            (FRONT_AXLE_Y + 0.32, frame_z + 0.00, frame_z + 0.09, 0.42),
        ],
        exponents=2.8,
        segments=44,
    )
    chassis.visual(_mesh("front_fairing.obj", nose), material=red)

    # --- rear fairing / bumper (red), wide across the back ---
    rear_fairing = superellipse_side_loft(
        [
            (REAR_AXLE_Y - 0.04, frame_z + 0.00, frame_z + 0.10, 0.58),
            (REAR_AXLE_Y - 0.20, frame_z + 0.00, frame_z + 0.10, 0.70),
            (REAR_AXLE_Y - 0.34, frame_z + 0.01, frame_z + 0.08, 0.52),
        ],
        exponents=2.6,
        segments=40,
    )
    chassis.visual(_mesh("rear_fairing.obj", rear_fairing), material=red)

    # --- pedal box / floor pan up front (sits on the rails) ---
    chassis.visual(
        Box((0.46, 0.52, 0.03)),
        origin=Origin(xyz=(0.0, 0.34, frame_z + 0.01)),
        material=dark_steel,
        name="floor_pan",
    )

    # --- seat support tray spanning the rails under the seat pan ---
    seat_tray_top = frame_z + 0.05
    chassis.visual(
        Box((0.60, 0.40, 0.03)),
        origin=Origin(xyz=(0.0, -0.06, seat_tray_top - 0.015)),
        material=dark_steel,
        name="seat_tray",
    )

    # --- steering column lower mount: a tube running along the tilted column
    #     line, rising from the floor pan up to where the steering shaft starts,
    #     so the steering column plugs straight into it. ---
    chassis.visual(
        Cylinder(radius=0.024, length=0.40),
        origin=Origin(xyz=(0.0, 0.277, frame_z + 0.15), rpy=(COLUMN_TILT, 0.0, 0.0)),
        material=dark_steel,
        name="column_lower_mount",
    )

    # ================= seat (fixed) =================
    seat = model.part("seat")
    seat.inertial = Inertial.from_geometry(
        Box((0.40, 0.44, 0.16)),
        mass=2.5,
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
    )
    # Low flat sling seat: a thin shallow pan lofted along +Y with minimal
    # thickness, plus a short low back panel at the rear. No tall wraparound
    # bolsters. Sections are (y, z_min, z_max, width_x), centered at x=0,
    # swept along +Y. seat-local z=0 is the seat floor.
    seat_pan = superellipse_side_loft(
        [
            (0.18, 0.000, 0.022, 0.30),
            (0.02, 0.000, 0.018, 0.38),
            (-0.16, 0.000, 0.022, 0.34),
        ],
        exponents=2.4,
        segments=40,
    )
    seat.visual(_mesh("seat_pan.obj", seat_pan), material=seat_mat, name="seat_pan")
    # Short low back panel at the rear — no taller than ~0.14 m.
    backrest = superellipse_side_loft(
        [
            (-0.15, 0.015, 0.13, 0.34),
            (-0.19, 0.010, 0.14, 0.32),
            (-0.22, 0.010, 0.12, 0.28),
        ],
        exponents=2.6,
        segments=40,
    )
    seat.visual(_mesh("seat_back.obj", backrest), material=seat_mat, name="seat_back")

    model.articulation(
        "seat_mount",
        ArticulationType.FIXED,
        parent=chassis,
        child=seat,
        origin=Origin(xyz=(0.0, -0.06, frame_z + 0.05)),
    )

    # ================= steering wheel (continuous about column) =================
    # Angled column: the wheel (top) leans back toward the driver while the shaft
    # runs down-and-forward (+Y) into the front-mounted lower column sleeve.
    # Column axis in the YZ plane. Tilt angle from vertical.
    tilt = COLUMN_TILT
    steering = model.part("steering_wheel")
    steering.inertial = Inertial.from_geometry(
        Box((0.24, 0.06, 0.24)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    # Column shaft (lies along the spin axis, local +Z of the steering frame is
    # the column axis because we author the steering geometry in a frame whose Z
    # is the column). We model geometry in the articulation frame: spin axis = Z.
    steering.visual(
        Cylinder(radius=0.015, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, -0.10)),
        material=dark_steel,
        name="steering_column",
    )
    # Wheel rim torus modeled as a thin lathed ring in the XY plane (perp to Z).
    ring_profile = [
        (0.105 - 0.014, -0.014),
        (0.105 + 0.014, -0.014),
        (0.105 + 0.014, 0.014),
        (0.105 - 0.014, 0.014),
        (0.105 - 0.014, -0.014),
    ]
    steering.visual(_mesh("steering_rim.obj", LatheGeometry(ring_profile, segments=48)), material=dark_steel)
    # Spokes + hub.
    steering.visual(Cylinder(radius=0.026, length=0.03), material=dark_steel, name="steering_hub")
    for ang in (0.0, 2.0 * pi / 3.0, 4.0 * pi / 3.0):
        steering.visual(
            Box((0.10, 0.018, 0.012)),
            origin=Origin(xyz=(0.05 * cos(ang), 0.05 * sin(ang), 0.0), rpy=(0.0, 0.0, ang)),
            material=dark_steel,
            name=f"steering_spoke_{int(round(ang * 100))}",
        )
    # Off-axis marker so rotation is detectable.
    steering.visual(
        Box((0.02, 0.02, 0.04)),
        origin=Origin(xyz=(0.0, 0.105, 0.0)),
        material=marker_mat,
        name="steering_marker",
    )

    # Place the steering articulation frame so its Z aligns with the angled
    # column. The wheel sits just in front of the driver; +tilt leans the wheel
    # top back toward the seat (-Y) so the shaft runs down-and-forward (+Y) into
    # the lower column sleeve. Local Z = col_axis; spin axis is -Z.
    col_top = (0.0, 0.12, 0.49)
    model.articulation(
        "steering_spin",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=steering,
        origin=Origin(xyz=col_top, rpy=(tilt, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )

    # ================= front steering knuckles + wheels =================
    # Each knuckle is a child of the chassis (REVOLUTE steer about vertical Z).
    # Each front wheel is a child of its knuckle (CONTINUOUS roll about X).
    def _knuckle(part, sign):
        # Upright + stub axle. side sign: +1 = left, -1 = right.
        part.visual(
            Box((0.05, 0.05, 0.14)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=dark_steel,
            name="knuckle_upright",
        )
        # Steering arm pointing rearward/inward.
        part.visual(
            Box((0.10, 0.04, 0.03)),
            origin=Origin(xyz=(-0.05 * sign, -0.04, 0.02)),
            material=dark_steel,
            name="knuckle_arm",
        )

    front_left_knuckle = model.part("front_left_knuckle")
    front_left_knuckle.inertial = Inertial.from_geometry(
        Box((0.10, 0.10, 0.16)), mass=1.2, origin=Origin(xyz=(0.0, 0.0, 0.0))
    )
    _knuckle(front_left_knuckle, sign=1.0)

    front_right_knuckle = model.part("front_right_knuckle")
    front_right_knuckle.inertial = Inertial.from_geometry(
        Box((0.10, 0.10, 0.16)), mass=1.2, origin=Origin(xyz=(0.0, 0.0, 0.0))
    )
    _knuckle(front_right_knuckle, sign=-1.0)

    # Knuckle steer joints: vertical axis at each front wheel center.
    model.articulation(
        "front_left_steer",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=front_left_knuckle,
        origin=Origin(xyz=(FRONT_TRACK_X, FRONT_AXLE_Y, AXLE_Z_FRONT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=2.0, lower=-0.52, upper=0.52),
    )
    model.articulation(
        "front_right_steer",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=front_right_knuckle,
        origin=Origin(xyz=(-FRONT_TRACK_X, FRONT_AXLE_Y, AXLE_Z_FRONT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=2.0, lower=-0.52, upper=0.52),
    )

    front_left_wheel = model.part("front_left_wheel")
    front_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=FRONT_TIRE_R, length=FRONT_TIRE_W),
        mass=4.0,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _wheel_visuals(front_left_wheel, "fl_wheel", FRONT_TIRE_R, FRONT_TIRE_W, rubber, rim_mat, dark_steel, marker_mat)

    front_right_wheel = model.part("front_right_wheel")
    front_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=FRONT_TIRE_R, length=FRONT_TIRE_W),
        mass=4.0,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _wheel_visuals(front_right_wheel, "fr_wheel", FRONT_TIRE_R, FRONT_TIRE_W, rubber, rim_mat, dark_steel, marker_mat)

    # Front wheels roll about local X, mounted at the knuckle center (origin 0).
    model.articulation(
        "front_left_roll",
        ArticulationType.CONTINUOUS,
        parent=front_left_knuckle,
        child=front_left_wheel,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=30.0),
    )
    model.articulation(
        "front_right_roll",
        ArticulationType.CONTINUOUS,
        parent=front_right_knuckle,
        child=front_right_wheel,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=30.0),
    )

    # ================= rear wheels (continuous roll) =================
    rear_left_wheel = model.part("rear_left_wheel")
    rear_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=REAR_TIRE_R, length=REAR_TIRE_W),
        mass=6.0,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _wheel_visuals(rear_left_wheel, "rl_wheel", REAR_TIRE_R, REAR_TIRE_W, rubber, rim_mat, dark_steel, marker_mat)

    rear_right_wheel = model.part("rear_right_wheel")
    rear_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=REAR_TIRE_R, length=REAR_TIRE_W),
        mass=6.0,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _wheel_visuals(rear_right_wheel, "rr_wheel", REAR_TIRE_R, REAR_TIRE_W, rubber, rim_mat, dark_steel, marker_mat)

    model.articulation(
        "rear_left_roll",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=rear_left_wheel,
        origin=Origin(xyz=(REAR_TRACK_X, REAR_AXLE_Y, AXLE_Z_REAR)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=30.0),
    )
    model.articulation(
        "rear_right_roll",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=rear_right_wheel,
        origin=Origin(xyz=(-REAR_TRACK_X, REAR_AXLE_Y, AXLE_Z_REAR)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=30.0),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests():
    from sdk import TestContext

    ctx = TestContext(object_model)

    chassis = object_model.get_part("chassis")
    seat = object_model.get_part("seat")
    steering = object_model.get_part("steering_wheel")
    fl_knuckle = object_model.get_part("front_left_knuckle")
    fr_knuckle = object_model.get_part("front_right_knuckle")
    fl_wheel = object_model.get_part("front_left_wheel")
    fr_wheel = object_model.get_part("front_right_wheel")
    rl_wheel = object_model.get_part("rear_left_wheel")
    rr_wheel = object_model.get_part("rear_right_wheel")

    steering_spin = object_model.get_articulation("steering_spin")
    fl_steer = object_model.get_articulation("front_left_steer")
    fr_steer = object_model.get_articulation("front_right_steer")
    fl_roll = object_model.get_articulation("front_left_roll")
    fr_roll = object_model.get_articulation("front_right_roll")
    rl_roll = object_model.get_articulation("rear_left_roll")
    rr_roll = object_model.get_articulation("rear_right_roll")

    # ---- intentional mounting overlaps ----
    # Each front wheel hub/axle stub is intentionally captured inside its
    # steering knuckle upright (the wheel rides on the knuckle stub axle).
    ctx.allow_overlap(
        fl_knuckle, fl_wheel,
        reason="Front-left wheel hub/axle is captured on the knuckle stub axle.",
    )
    ctx.allow_overlap(
        fr_knuckle, fr_wheel,
        reason="Front-right wheel hub/axle is captured on the knuckle stub axle.",
    )
    # Wheels and knuckles ride on axles that pass through the tubular chassis
    # frame at the spindle/axle lines (exposed-axle mounting).
    for w in (fl_wheel, fr_wheel, rl_wheel, rr_wheel):
        ctx.allow_overlap(
            chassis, w,
            reason="Wheel hub/axle passes through the chassis axle line / frame tube at its mount.",
        )
    ctx.allow_overlap(
        chassis, fl_knuckle,
        reason="Front-left knuckle straddles the front cross tube at its kingpin mount.",
    )
    ctx.allow_overlap(
        chassis, fr_knuckle,
        reason="Front-right knuckle straddles the front cross tube at its kingpin mount.",
    )
    # Steering shaft plugs into the chassis column-lower-mount sleeve.
    ctx.allow_overlap(
        chassis, steering,
        elem_a="column_lower_mount", elem_b="steering_column",
        reason="Steering shaft is intentionally inserted into the column lower-mount sleeve.",
    )

    # ---- kart rests on all four wheels (bottoms near ground z=0) ----
    for nm, w in (("FL", fl_wheel), ("FR", fr_wheel), ("RL", rl_wheel), ("RR", rr_wheel)):
        mn, _ = ctx.part_world_aabb(w)
        ctx.check(
            f"{nm} wheel sits on the ground",
            mn[2] <= 0.02,
            details=f"{nm} wheel min z={mn[2]:.4f}",
        )

    # ---- rear wheels are larger AND wider than the front wheels ----
    fl_ext = _ext(ctx.part_world_aabb(fl_wheel))
    rl_ext = _ext(ctx.part_world_aabb(rl_wheel))
    ctx.check(
        "rear tire is larger diameter than front",
        rl_ext[2] > fl_ext[2] + 0.01,
        details=f"front dia(z)={fl_ext[2]:.3f}, rear dia(z)={rl_ext[2]:.3f}",
    )
    ctx.check(
        "rear tire is wider than front",
        rl_ext[0] > fl_ext[0] + 0.02,
        details=f"front width(x)={fl_ext[0]:.3f}, rear width(x)={rl_ext[0]:.3f}",
    )

    # ---- steering wheel spins about its angled column ----
    rest = ctx.part_element_world_aabb(steering, elem="steering_marker")
    rmid = [(rest[0][i] + rest[1][i]) * 0.5 for i in range(3)]
    with ctx.pose({steering_spin: pi}):
        half = ctx.part_element_world_aabb(steering, elem="steering_marker")
        hmid = [(half[0][i] + half[1][i]) * 0.5 for i in range(3)]
    moved = max(abs(hmid[i] - rmid[i]) for i in range(3))
    ctx.check(
        "steering wheel marker moves when column spins",
        moved > 0.05,
        details=f"rest marker={rmid}, half-turn marker={hmid}, moved={moved:.4f}",
    )
    # Steering shaft is seated in the chassis column mount (no floating wheel).
    ctx.expect_overlap(
        steering, chassis,
        axes="z",
        elem_a="steering_column", elem_b="column_lower_mount",
        min_overlap=0.02,
        name="steering shaft seated in column mount",
    )

    # ---- front knuckles steer: front wheel heading changes left/right ----
    def heading_x_extent(wheel, steer_joint, q):
        with ctx.pose({steer_joint: q}):
            return _ext(ctx.part_world_aabb(wheel))

    for nm, wheel, steer in (("front_left", fl_wheel, fl_steer), ("front_right", fr_wheel, fr_steer)):
        straight = _ext(ctx.part_world_aabb(wheel))
        turned = heading_x_extent(wheel, steer, 0.5)
        # When steered, the wheel rotates about vertical: its Y-extent (fore-aft
        # footprint) grows as the tread turns to face sideways.
        ctx.check(
            f"{nm} knuckle steers the wheel (heading changes)",
            turned[1] > straight[1] + 0.02,
            details=f"{nm} straight Y-ext={straight[1]:.3f}, steered Y-ext={turned[1]:.3f}",
        )

    # ---- all four wheels roll about their axles (marker swings) ----
    def marker_swing(wheel, prefix, roll_joint):
        # Marker sits at +Y in the wheel frame; rolling pi about X moves it to -Y
        # (and any z offset flips too). Measure total in-plane (Y,Z) displacement.
        elem = f"{prefix}_marker"
        rest_a = ctx.part_element_world_aabb(wheel, elem=elem)
        rmid = [(rest_a[0][i] + rest_a[1][i]) * 0.5 for i in range(3)]
        with ctx.pose({roll_joint: pi}):
            roll_a = ctx.part_element_world_aabb(wheel, elem=elem)
            rmid2 = [(roll_a[0][i] + roll_a[1][i]) * 0.5 for i in range(3)]
        return max(abs(rmid2[1] - rmid[1]), abs(rmid2[2] - rmid[2]))

    for nm, wheel, prefix, roll in (
        ("front_left", fl_wheel, "fl_wheel", fl_roll),
        ("front_right", fr_wheel, "fr_wheel", fr_roll),
        ("rear_left", rl_wheel, "rl_wheel", rl_roll),
        ("rear_right", rr_wheel, "rr_wheel", rr_roll),
    ):
        swing = marker_swing(wheel, prefix, roll)
        ctx.check(
            f"{nm} wheel rolls about its axle",
            swing > 0.05,
            details=f"{nm} marker z-swing on half turn={swing:.4f}",
        )

    # ---- seat is mounted on the chassis, centrally ----
    sp = ctx.part_world_position(seat)
    ctx.check(
        "seat mounted near kart center",
        sp is not None and abs(sp[0]) < 0.05 and -0.3 < sp[1] < 0.2,
        details=f"seat origin={sp}",
    )
    ctx.expect_contact(seat, chassis, name="seat attached to chassis")

    # ---- sling seat is low and flat (no deep bucket, no tall bolsters) ----
    seat_aabb = ctx.part_world_aabb(seat)
    seat_height = seat_aabb[1][2] - seat_aabb[0][2]
    ctx.check(
        "sling seat is low (total height < 0.20 m)",
        seat_height < 0.20,
        details=f"seat z-extent={seat_height:.4f} m (flat sling, not bucket)",
    )
    # Seat pan itself is very thin (sling fabric / thin pad, not a deep molded shell).
    pan_aabb = ctx.part_element_world_aabb(seat, elem="seat_pan")
    pan_height = pan_aabb[1][2] - pan_aabb[0][2]
    ctx.check(
        "seat pan is thin (sling, not deep bucket)",
        pan_height < 0.06,
        details=f"seat_pan z-extent={pan_height:.4f} m",
    )
    # Seat back is short — no taller than about 0.14 m above the seat floor.
    back_aabb = ctx.part_element_world_aabb(seat, elem="seat_back")
    back_height = back_aabb[1][2] - back_aabb[0][2]
    ctx.check(
        "seat back is short (no tall wraparound bolster)",
        back_height < 0.16,
        details=f"seat_back z-extent={back_height:.4f} m",
    )
    # No side bolsters present on the sling seat.
    seat_visual_names = [v.name for v in seat.visuals]
    ctx.check(
        "no side bolsters on sling seat",
        "left_bolster" not in seat_visual_names and "right_bolster" not in seat_visual_names,
        details=f"seat visuals={seat_visual_names}",
    )

    # ---- front wheel is carried by its knuckle (no floating) ----
    ctx.expect_contact(fl_wheel, fl_knuckle, name="front-left wheel on its knuckle")
    ctx.expect_contact(fr_wheel, fr_knuckle, name="front-right wheel on its knuckle")

    return ctx.report()


object_model = build_object_model()
