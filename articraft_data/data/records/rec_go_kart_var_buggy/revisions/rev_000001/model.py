from __future__ import annotations

# Off-road tube-frame buggy (fork of racing go-kart).
# Frame convention:
#   +Y = forward (front of the buggy), -Y = rear.
#   +X = buggy's left, -X = buggy's right.
#   +Z = up; ground plane at z=0, wheel centers at z = tire_radius.
# Hero forms: raised tubular steel frame with roll cage hoop arching over the
# seat, nerf-bar side protection, tube front/rear bumpers, exposed welded tube
# construction, single bucket seat, small steering wheel on an angled column,
# four slick tires (smaller/narrower front, larger/wider rear) on exposed axles.
# Articulations (unchanged from parent):
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
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---- key dimensions ----
FRONT_AXLE_Y = 0.74
REAR_AXLE_Y = -0.74
FRONT_TRACK_X = 0.46
REAR_TRACK_X = 0.50

FRONT_TIRE_R = 0.115
FRONT_TIRE_W = 0.140
REAR_TIRE_R = 0.140
REAR_TIRE_W = 0.230

AXLE_Z_FRONT = FRONT_TIRE_R
AXLE_Z_REAR = REAR_TIRE_R

# Raised ride height: frame sits higher than a flat racing kart.
FRAME_Z = 0.20
COLUMN_TILT = 0.55

# Roll cage dimensions (relative to frame).
CAGE_PEAK_Z = FRAME_Z + 0.62  # hoop peak above ground
CAGE_HOOP_Y = -0.10  # main hoop is just behind seat center
CAGE_FRONT_Y = 0.50  # where front down-bars tie into the frame
CAGE_HALF_WIDTH = 0.32  # half-width at hoop top
CAGE_RAIL_X = 0.30  # side rail X at frame level


def _mesh(name, geom):
    return mesh_from_geometry(geom, name)


def _mirror_x(points):
    return [(-x, y, z) for x, y, z in points]


def _tube(name, pts, radius=0.022, samples=12, radial=14):
    return _mesh(name, tube_from_spline_points(
        pts, radius=radius, samples_per_segment=samples, radial_segments=radial,
    ))


def _tire_mesh(name, tire_radius, tire_width):
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
    spin_rpy = Origin(rpy=(0.0, pi / 2.0, 0.0))
    part.visual(_tire_mesh(f"{prefix}_tire.obj", tire_radius, tire_width), material=rubber)
    rim_r = tire_radius * 0.58
    part.visual(
        Cylinder(radius=rim_r, length=tire_width * 0.80),
        origin=spin_rpy, material=rim_mat, name=f"{prefix}_rim",
    )
    part.visual(
        Cylinder(radius=tire_radius * 0.30, length=tire_width * 0.92),
        origin=spin_rpy, material=hub_mat, name=f"{prefix}_hub",
    )
    part.visual(
        Cylinder(radius=tire_radius * 0.13, length=tire_width * 1.6),
        origin=spin_rpy, material=hub_mat, name=f"{prefix}_axle",
    )
    mr = tire_radius * 0.62
    part.visual(
        Box((tire_width * 0.55, tire_radius * 0.12, tire_radius * 0.12)),
        origin=Origin(xyz=(0.0, mr, 0.0)), material=marker_mat, name=f"{prefix}_marker",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offroad_buggy")

    steel = model.material("frame_steel", rgba=(0.72, 0.73, 0.76, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.26, 0.27, 0.29, 1.0))
    cage_green = model.material("cage_green", rgba=(0.22, 0.52, 0.28, 1.0))
    rubber = model.material("tire_rubber", rgba=(0.06, 0.06, 0.07, 1.0))
    rim_mat = model.material("rim_silver", rgba=(0.60, 0.61, 0.64, 1.0))
    seat_mat = model.material("seat_gray", rgba=(0.82, 0.82, 0.84, 1.0))
    marker_mat = model.material("marker_yellow", rgba=(0.95, 0.80, 0.15, 1.0))
    bumper_mat = model.material("bumper_black", rgba=(0.12, 0.12, 0.13, 1.0))

    fz = FRAME_Z

    # ================= chassis (root) =================
    chassis = model.part("chassis")
    chassis.inertial = Inertial.from_geometry(
        Box((1.10, 1.80, 0.90)),
        mass=110.0,
        origin=Origin(xyz=(0.0, -0.05, 0.35)),
    )

    # --- main side rails (raised) running front-to-rear ---
    rail_pts = [
        (CAGE_RAIL_X, FRONT_AXLE_Y, fz),
        (0.32, 0.45, fz),
        (0.32, -0.10, fz),
        (0.34, -0.50, fz),
        (REAR_TRACK_X - 0.06, REAR_AXLE_Y, fz + 0.02),
    ]
    for i, (nm, pts) in enumerate((
        ("left_main_rail", rail_pts),
        ("right_main_rail", _mirror_x(rail_pts)),
    )):
        chassis.visual(_tube(f"{nm}.obj", pts, radius=0.024), name=nm)

    # --- center spine tube (raised) ---
    chassis.visual(_tube("center_spine.obj", [
        (0.0, 0.70, fz + 0.01), (0.0, 0.20, fz), (0.0, -0.30, fz),
        (0.0, REAR_AXLE_Y, fz + 0.02),
    ], radius=0.022), name="center_spine")

    # --- front cross tube (steering axle support) ---
    chassis.visual(_tube("front_cross_tube.obj", [
        (-FRONT_TRACK_X, FRONT_AXLE_Y, fz + 0.01),
        (0.0, FRONT_AXLE_Y + 0.02, fz + 0.03),
        (FRONT_TRACK_X, FRONT_AXLE_Y, fz + 0.01),
    ], radius=0.022), name="front_cross_tube")

    # --- rear cross tube ---
    chassis.visual(_tube("rear_cross_tube.obj", [
        (-REAR_TRACK_X, REAR_AXLE_Y, fz + 0.02),
        (0.0, REAR_AXLE_Y - 0.02, fz + 0.02),
        (REAR_TRACK_X, REAR_AXLE_Y, fz + 0.02),
    ], radius=0.024), name="rear_cross_tube")

    # Live rear axle bar.
    chassis.visual(
        Cylinder(radius=0.018, length=REAR_TRACK_X * 2.0 - 0.02),
        origin=Origin(xyz=(0.0, REAR_AXLE_Y, AXLE_Z_REAR), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel, name="rear_axle_bar",
    )
    # Axle hanger brackets: short vertical tubes from the rear cross tube down
    # to the axle bar, bridging the mesh connectivity gap.
    for nm, x in (("left_axle_hanger", 0.18), ("right_axle_hanger", -0.18)):
        chassis.visual(_tube(f"{nm}.obj", [
            (x, REAR_AXLE_Y, AXLE_Z_REAR + 0.018),
            (x, REAR_AXLE_Y, fz + 0.01),
        ], radius=0.012, samples=4, radial=10), material=dark_steel, name=nm)

    # --- diagonal braces (rail to center spine) ---
    for nm, pts in (
        ("left_brace", [(0.30, 0.20, fz), (0.16, 0.05, fz), (0.0, -0.05, fz)]),
        ("right_brace", [(-0.30, 0.20, fz), (-0.16, 0.05, fz), (0.0, -0.05, fz)]),
    ):
        chassis.visual(_tube(f"{nm}.obj", pts, radius=0.016, samples=8, radial=12), name=nm)

    # --- vertical frame risers from axle level to frame level ---
    # These connect the frame to the axle lines, giving the raised ride height
    # visible structural support.
    for i, (nm, x, y, z_bot, z_top) in enumerate((
        ("left_front_riser", FRONT_TRACK_X - 0.06, FRONT_AXLE_Y, AXLE_Z_FRONT + 0.04, fz),
        ("right_front_riser", -(FRONT_TRACK_X - 0.06), FRONT_AXLE_Y, AXLE_Z_FRONT + 0.04, fz),
        ("left_rear_riser", REAR_TRACK_X - 0.06, REAR_AXLE_Y, AXLE_Z_REAR + 0.04, fz + 0.02),
        ("right_rear_riser", -(REAR_TRACK_X - 0.06), REAR_AXLE_Y, AXLE_Z_REAR + 0.04, fz + 0.02),
    )):
        chassis.visual(_tube(f"{nm}.obj", [
            (x, y, z_bot), (x, y, (z_bot + z_top) * 0.5), (x, y, z_top),
        ], radius=0.018, samples=6, radial=12), name=nm)

    # ================= ROLL CAGE =================
    # Main hoop: left rail → up → across top → down → right rail.
    cage_r = 0.026
    hoop_left_base = (CAGE_RAIL_X, CAGE_HOOP_Y, fz)
    hoop_left_top = (CAGE_HALF_WIDTH, CAGE_HOOP_Y, CAGE_PEAK_Z)
    hoop_center = (0.0, CAGE_HOOP_Y, CAGE_PEAK_Z + 0.02)
    hoop_right_top = (-CAGE_HALF_WIDTH, CAGE_HOOP_Y, CAGE_PEAK_Z)
    hoop_right_base = (-CAGE_RAIL_X, CAGE_HOOP_Y, fz)

    chassis.visual(_tube("roll_cage_hoop.obj", [
        hoop_left_base,
        (CAGE_RAIL_X, CAGE_HOOP_Y, fz + 0.30),
        hoop_left_top,
        hoop_center,
        hoop_right_top,
        (-CAGE_RAIL_X, CAGE_HOOP_Y, fz + 0.30),
        hoop_right_base,
    ], radius=cage_r, samples=16, radial=16), material=cage_green, name="roll_cage_hoop")

    # Front down-bars (A-pillars): from hoop top corners forward and down to
    # the front frame rail area.
    for nm, pts in (
        ("cage_front_left_bar", [
            hoop_left_top,
            (CAGE_HALF_WIDTH + 0.02, 0.20, CAGE_PEAK_Z - 0.18),
            (CAGE_RAIL_X + 0.02, CAGE_FRONT_Y, fz + 0.04),
        ]),
        ("cage_front_right_bar", [
            hoop_right_top,
            (-CAGE_HALF_WIDTH - 0.02, 0.20, CAGE_PEAK_Z - 0.18),
            (-CAGE_RAIL_X - 0.02, CAGE_FRONT_Y, fz + 0.04),
        ]),
    ):
        chassis.visual(_tube(f"{nm}.obj", pts, radius=cage_r, samples=14, radial=16), material=cage_green, name=nm)

    # Rear down-bars: from hoop top corners backward and down to rear frame.
    for nm, pts in (
        ("cage_rear_left_bar", [
            hoop_left_top,
            (CAGE_HALF_WIDTH + 0.01, CAGE_HOOP_Y - 0.20, CAGE_PEAK_Z - 0.20),
            (CAGE_RAIL_X, -0.52, fz + 0.04),
        ]),
        ("cage_rear_right_bar", [
            hoop_right_top,
            (-CAGE_HALF_WIDTH - 0.01, CAGE_HOOP_Y - 0.20, CAGE_PEAK_Z - 0.20),
            (-CAGE_RAIL_X, -0.52, fz + 0.04),
        ]),
    ):
        chassis.visual(_tube(f"{nm}.obj", pts, radius=cage_r, samples=14, radial=16), material=cage_green, name=nm)

    # Cross bar at the top of the cage (lateral brace between front down-bars).
    # Placed forward (y=0.38) and lower to clear the steering wheel sweep.
    chassis.visual(_tube("cage_top_cross.obj", [
        (CAGE_HALF_WIDTH + 0.02, 0.38, CAGE_PEAK_Z - 0.36),
        (0.0, 0.38, CAGE_PEAK_Z - 0.34),
        (-CAGE_HALF_WIDTH - 0.02, 0.38, CAGE_PEAK_Z - 0.36),
    ], radius=0.018, samples=10, radial=12), material=cage_green, name="cage_top_cross")

    # Rear cross bar between rear down-bars.
    chassis.visual(_tube("cage_rear_cross.obj", [
        (CAGE_HALF_WIDTH, CAGE_HOOP_Y - 0.20, CAGE_PEAK_Z - 0.18),
        (0.0, CAGE_HOOP_Y - 0.20, CAGE_PEAK_Z - 0.16),
        (-CAGE_HALF_WIDTH, CAGE_HOOP_Y - 0.20, CAGE_PEAK_Z - 0.18),
    ], radius=0.018, samples=10, radial=12), material=cage_green, name="cage_rear_cross")

    # ================= NERF BARS (side protection tubes) =================
    # Horizontal tubes along each side, above the rails, providing side impact
    # protection typical of off-road buggies.
    nerf_z = fz + 0.12
    for nm, pts in (
        ("left_nerf_bar", [
            (CAGE_RAIL_X + 0.04, 0.45, nerf_z),
            (CAGE_RAIL_X + 0.06, 0.10, nerf_z),
            (CAGE_RAIL_X + 0.04, -0.30, nerf_z),
        ]),
        ("right_nerf_bar", [
            (-CAGE_RAIL_X - 0.04, 0.45, nerf_z),
            (-CAGE_RAIL_X - 0.06, 0.10, nerf_z),
            (-CAGE_RAIL_X - 0.04, -0.30, nerf_z),
        ]),
    ):
        chassis.visual(_tube(f"{nm}.obj", pts, radius=0.016, samples=10, radial=12), material=dark_steel, name=nm)

    # Nerf bar vertical supports (connecting nerf bars down to the frame rails).
    for i, (nm, x, y) in enumerate((
        ("left_nerf_support_front", CAGE_RAIL_X + 0.04, 0.35),
        ("left_nerf_support_mid", CAGE_RAIL_X + 0.05, 0.0),
        ("left_nerf_support_rear", CAGE_RAIL_X + 0.04, -0.25),
        ("right_nerf_support_front", -(CAGE_RAIL_X + 0.04), 0.35),
        ("right_nerf_support_mid", -(CAGE_RAIL_X + 0.05), 0.0),
        ("right_nerf_support_rear", -(CAGE_RAIL_X + 0.04), -0.25),
    )):
        chassis.visual(_tube(f"{nm}.obj", [
            (x, y, fz + 0.02), (x, y, nerf_z - 0.01),
        ], radius=0.012, samples=4, radial=10), material=dark_steel, name=nm)

    # ================= FRONT BUMPER (tube brush guard) =================
    # Single continuous tube from left frame rail, out through the bumper
    # curve, and back to the right frame rail — ensures mesh connectivity.
    bumper_z = fz + 0.04
    bumper_fwd = FRONT_AXLE_Y + 0.20
    chassis.visual(_tube("front_bumper.obj", [
        (CAGE_RAIL_X, FRONT_AXLE_Y, fz + 0.01),
        (CAGE_RAIL_X + 0.06, FRONT_AXLE_Y + 0.08, bumper_z),
        (0.38, bumper_fwd, bumper_z + 0.04),
        (0.18, bumper_fwd + 0.06, bumper_z + 0.08),
        (0.0, bumper_fwd + 0.08, bumper_z + 0.10),
        (-0.18, bumper_fwd + 0.06, bumper_z + 0.08),
        (-0.38, bumper_fwd, bumper_z + 0.04),
        (-CAGE_RAIL_X - 0.06, FRONT_AXLE_Y + 0.08, bumper_z),
        (-CAGE_RAIL_X, FRONT_AXLE_Y, fz + 0.01),
    ], radius=0.020, samples=14, radial=14), material=bumper_mat, name="front_bumper")

    # ================= REAR BUMPER (tube) =================
    # Single continuous tube from left rear rail to right rear rail through
    # the rear bumper curve — ensures mesh connectivity.
    rear_bumper_y = REAR_AXLE_Y - 0.18
    chassis.visual(_tube("rear_bumper.obj", [
        (REAR_TRACK_X - 0.06, REAR_AXLE_Y, fz + 0.02),
        (0.42, REAR_AXLE_Y - 0.06, fz + 0.03),
        (0.40, rear_bumper_y, fz + 0.04),
        (0.0, rear_bumper_y - 0.04, fz + 0.06),
        (-0.40, rear_bumper_y, fz + 0.04),
        (-0.42, REAR_AXLE_Y - 0.06, fz + 0.03),
        (-(REAR_TRACK_X - 0.06), REAR_AXLE_Y, fz + 0.02),
    ], radius=0.020, samples=12, radial=14), material=bumper_mat, name="rear_bumper")

    # ================= FLOOR PAN / SEAT TRAY / COLUMN MOUNT =================
    # Pedal box / floor pan (sits on the raised rails).
    chassis.visual(
        Box((0.46, 0.52, 0.03)),
        origin=Origin(xyz=(0.0, 0.34, fz + 0.01)),
        material=dark_steel, name="floor_pan",
    )

    # Seat support tray.
    chassis.visual(
        Box((0.60, 0.40, 0.03)),
        origin=Origin(xyz=(0.0, -0.06, fz + 0.035)),
        material=dark_steel, name="seat_tray",
    )

    # Steering column lower mount tube.
    chassis.visual(
        Cylinder(radius=0.024, length=0.40),
        origin=Origin(xyz=(0.0, 0.277, fz + 0.15), rpy=(COLUMN_TILT, 0.0, 0.0)),
        material=dark_steel, name="column_lower_mount",
    )

    # ================= seat (fixed, geometry unchanged) =================
    seat = model.part("seat")
    seat.inertial = Inertial.from_geometry(
        Box((0.42, 0.46, 0.40)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
    )
    from sdk import superellipse_side_loft
    seat_pan = superellipse_side_loft(
        [
            (0.20, 0.02, 0.07, 0.30),
            (0.02, 0.00, 0.06, 0.40),
            (-0.14, 0.01, 0.07, 0.36),
        ],
        exponents=2.4, segments=40,
    )
    seat.visual(_mesh("seat_pan.obj", seat_pan), material=seat_mat)
    backrest = superellipse_side_loft(
        [
            (-0.14, 0.02, 0.34, 0.36),
            (-0.20, 0.02, 0.38, 0.34),
            (-0.25, 0.02, 0.32, 0.30),
        ],
        exponents=2.6, segments=40,
    )
    seat.visual(_mesh("seat_back.obj", backrest), material=seat_mat)
    seat.visual(Box((0.04, 0.34, 0.14)), origin=Origin(xyz=(0.19, 0.0, 0.10)), material=seat_mat, name="left_bolster")
    seat.visual(Box((0.04, 0.34, 0.14)), origin=Origin(xyz=(-0.19, 0.0, 0.10)), material=seat_mat, name="right_bolster")

    model.articulation(
        "seat_mount",
        ArticulationType.FIXED,
        parent=chassis, child=seat,
        origin=Origin(xyz=(0.0, -0.06, fz + 0.05)),
    )

    # ================= steering wheel (continuous about column) =================
    tilt = COLUMN_TILT
    steering = model.part("steering_wheel")
    steering.inertial = Inertial.from_geometry(
        Box((0.24, 0.06, 0.24)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    steering.visual(
        Cylinder(radius=0.015, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, -0.10)),
        material=dark_steel, name="steering_column",
    )
    ring_profile = [
        (0.105 - 0.014, -0.014),
        (0.105 + 0.014, -0.014),
        (0.105 + 0.014, 0.014),
        (0.105 - 0.014, 0.014),
        (0.105 - 0.014, -0.014),
    ]
    steering.visual(_mesh("steering_rim.obj", LatheGeometry(ring_profile, segments=48)), material=dark_steel, name="steering_rim")
    steering.visual(Cylinder(radius=0.026, length=0.03), material=dark_steel, name="steering_hub")
    for ang in (0.0, 2.0 * pi / 3.0, 4.0 * pi / 3.0):
        steering.visual(
            Box((0.10, 0.018, 0.012)),
            origin=Origin(xyz=(0.05 * cos(ang), 0.05 * sin(ang), 0.0), rpy=(0.0, 0.0, ang)),
            material=dark_steel,
            name=f"steering_spoke_{int(round(ang * 100))}",
        )
    steering.visual(
        Box((0.02, 0.02, 0.04)),
        origin=Origin(xyz=(0.0, 0.105, 0.0)),
        material=marker_mat, name="steering_marker",
    )

    # Steering articulation origin shifted up with raised frame.
    col_top = (0.0, 0.12, fz + 0.405)
    model.articulation(
        "steering_spin",
        ArticulationType.CONTINUOUS,
        parent=chassis, child=steering,
        origin=Origin(xyz=col_top, rpy=(tilt, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )

    # ================= front steering knuckles + wheels =================
    def _knuckle(part, sign):
        part.visual(
            Box((0.05, 0.05, 0.14)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=dark_steel, name="knuckle_upright",
        )
        part.visual(
            Box((0.10, 0.04, 0.03)),
            origin=Origin(xyz=(-0.05 * sign, -0.04, 0.02)),
            material=dark_steel, name="knuckle_arm",
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

    model.articulation(
        "front_left_steer",
        ArticulationType.REVOLUTE,
        parent=chassis, child=front_left_knuckle,
        origin=Origin(xyz=(FRONT_TRACK_X, FRONT_AXLE_Y, AXLE_Z_FRONT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=2.0, lower=-0.52, upper=0.52),
    )
    model.articulation(
        "front_right_steer",
        ArticulationType.REVOLUTE,
        parent=chassis, child=front_right_knuckle,
        origin=Origin(xyz=(-FRONT_TRACK_X, FRONT_AXLE_Y, AXLE_Z_FRONT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=2.0, lower=-0.52, upper=0.52),
    )

    front_left_wheel = model.part("front_left_wheel")
    front_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=FRONT_TIRE_R, length=FRONT_TIRE_W),
        mass=4.0, origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _wheel_visuals(front_left_wheel, "fl_wheel", FRONT_TIRE_R, FRONT_TIRE_W, rubber, rim_mat, dark_steel, marker_mat)

    front_right_wheel = model.part("front_right_wheel")
    front_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=FRONT_TIRE_R, length=FRONT_TIRE_W),
        mass=4.0, origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _wheel_visuals(front_right_wheel, "fr_wheel", FRONT_TIRE_R, FRONT_TIRE_W, rubber, rim_mat, dark_steel, marker_mat)

    model.articulation(
        "front_left_roll",
        ArticulationType.CONTINUOUS,
        parent=front_left_knuckle, child=front_left_wheel,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=30.0),
    )
    model.articulation(
        "front_right_roll",
        ArticulationType.CONTINUOUS,
        parent=front_right_knuckle, child=front_right_wheel,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=30.0),
    )

    # ================= rear wheels (continuous roll) =================
    rear_left_wheel = model.part("rear_left_wheel")
    rear_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=REAR_TIRE_R, length=REAR_TIRE_W),
        mass=6.0, origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _wheel_visuals(rear_left_wheel, "rl_wheel", REAR_TIRE_R, REAR_TIRE_W, rubber, rim_mat, dark_steel, marker_mat)

    rear_right_wheel = model.part("rear_right_wheel")
    rear_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=REAR_TIRE_R, length=REAR_TIRE_W),
        mass=6.0, origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _wheel_visuals(rear_right_wheel, "rr_wheel", REAR_TIRE_R, REAR_TIRE_W, rubber, rim_mat, dark_steel, marker_mat)

    model.articulation(
        "rear_left_roll",
        ArticulationType.CONTINUOUS,
        parent=chassis, child=rear_left_wheel,
        origin=Origin(xyz=(REAR_TRACK_X, REAR_AXLE_Y, AXLE_Z_REAR)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=30.0),
    )
    model.articulation(
        "rear_right_roll",
        ArticulationType.CONTINUOUS,
        parent=chassis, child=rear_right_wheel,
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

    # ---- intentional mounting overlaps (same as parent) ----
    ctx.allow_overlap(
        fl_knuckle, fl_wheel,
        reason="Front-left wheel hub/axle is captured on the knuckle stub axle.",
    )
    ctx.allow_overlap(
        fr_knuckle, fr_wheel,
        reason="Front-right wheel hub/axle is captured on the knuckle stub axle.",
    )
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
    ctx.allow_overlap(
        chassis, steering,
        elem_a="column_lower_mount", elem_b="steering_column",
        reason="Steering shaft is intentionally inserted into the column lower-mount sleeve.",
    )

    # ---- buggy rests on all four wheels ----
    for nm, w in (("FL", fl_wheel), ("FR", fr_wheel), ("RL", rl_wheel), ("RR", rr_wheel)):
        mn, _ = ctx.part_world_aabb(w)
        ctx.check(
            f"{nm} wheel sits on the ground",
            mn[2] <= 0.02,
            details=f"{nm} wheel min z={mn[2]:.4f}",
        )

    # ---- PROMPT-SPECIFIC: raised ride height ----
    # The frame floor pan should sit well above the wheel axle line.
    # (The rear axle bar sits at axle height by design, so we check the floor pan.)
    floor_aabb = ctx.part_element_world_aabb(chassis, elem="floor_pan")
    floor_min_z = floor_aabb[0][2]
    ctx.check(
        "frame floor sits above axle line (raised ride height)",
        floor_min_z > AXLE_Z_REAR + 0.04,
        details=f"floor pan min z={floor_min_z:.4f}, rear axle z={AXLE_Z_REAR:.4f}",
    )

    # ---- PROMPT-SPECIFIC: roll cage hoop extends well above the seat ----
    chassis_aabb = ctx.part_world_aabb(chassis)
    chassis_max_z = chassis_aabb[1][2]
    seat_aabb = ctx.part_world_aabb(seat)
    seat_max_z = seat_aabb[1][2]
    ctx.check(
        "roll cage extends above seat top",
        chassis_max_z > seat_max_z + 0.15,
        details=f"chassis max z={chassis_max_z:.4f}, seat max z={seat_max_z:.4f}",
    )

    # ---- PROMPT-SPECIFIC: roll cage peak is high (buggy proportions) ----
    ctx.check(
        "roll cage peak reaches buggy height",
        chassis_max_z > 0.70,
        details=f"chassis max z={chassis_max_z:.4f}",
    )

    # ---- PROMPT-SPECIFIC: frame has roll_cage_hoop visual (tube cage identity) ----
    cage_visual_names = [v.name for v in chassis.visuals if v.name and ("cage" in v.name or "roll" in v.name)]
    ctx.check(
        "roll cage tube visuals present on chassis",
        len(cage_visual_names) >= 4,
        details=f"cage visuals found: {cage_visual_names}",
    )

    # ---- PROMPT-SPECIFIC: nerf bars present (exposed tube side protection) ----
    nerf_names = [v.name for v in chassis.visuals if v.name and "nerf" in v.name]
    ctx.check(
        "nerf bar visuals present on chassis",
        len(nerf_names) >= 2,
        details=f"nerf visuals found: {nerf_names}",
    )

    # ---- PROMPT-SPECIFIC: no smooth bodywork (no side pods, no fairings) ----
    bodywork_names = [v.name for v in chassis.visuals if v.name and ("pod" in v.name or "fairing" in v.name)]
    ctx.check(
        "no smooth bodywork panels (exposed tube frame)",
        len(bodywork_names) == 0,
        details=f"bodywork visuals found: {bodywork_names}",
    )

    # ---- rear wheels larger and wider than front ----
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
    ctx.expect_overlap(
        steering, chassis,
        axes="z",
        elem_a="steering_column", elem_b="column_lower_mount",
        min_overlap=0.02,
        name="steering shaft seated in column mount",
    )

    # ---- front knuckles steer ----
    def heading_x_extent(wheel, steer_joint, q):
        with ctx.pose({steer_joint: q}):
            return _ext(ctx.part_world_aabb(wheel))

    for nm, wheel, steer in (("front_left", fl_wheel, fl_steer), ("front_right", fr_wheel, fr_steer)):
        straight = _ext(ctx.part_world_aabb(wheel))
        turned = heading_x_extent(wheel, steer, 0.5)
        ctx.check(
            f"{nm} knuckle steers the wheel (heading changes)",
            turned[1] > straight[1] + 0.02,
            details=f"{nm} straight Y-ext={straight[1]:.3f}, steered Y-ext={turned[1]:.3f}",
        )

    # ---- all four wheels roll ----
    def marker_swing(wheel, prefix, roll_joint):
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

    # ---- seat mounted on chassis centrally ----
    sp = ctx.part_world_position(seat)
    ctx.check(
        "seat mounted near buggy center",
        sp is not None and abs(sp[0]) < 0.05 and -0.3 < sp[1] < 0.2,
        details=f"seat origin={sp}",
    )
    ctx.expect_contact(seat, chassis, name="seat attached to chassis")

    # ---- front wheels on knuckles ----
    ctx.expect_contact(fl_wheel, fl_knuckle, name="front-left wheel on its knuckle")
    ctx.expect_contact(fr_wheel, fr_knuckle, name="front-right wheel on its knuckle")

    return ctx.report()


object_model = build_object_model()
