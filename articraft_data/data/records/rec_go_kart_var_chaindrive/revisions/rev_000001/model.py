from __future__ import annotations

# Racing go-kart.
# Frame convention:
#   +Y = forward (front of the kart), -Y = rear.
#   +X = kart's left, -X = kart's right.
#   +Z = up; ground plane at z=0, wheel centers at z = tire_radius.
# Hero forms: low tubular steel frame, pink/red side pods + front fairing,
# single molded bucket seat, small steering wheel on an angled column,
# four fat slick tires (smaller/narrower front, larger/wider rear) on exposed
# axles, number "5" decals.
# Articulations:
#   - steering wheel: CONTINUOUS spin about its angled column axis.
#   - front-left knuckle: REVOLUTE steer (vertical); its wheel: CONTINUOUS roll (child).
#   - front-right knuckle: REVOLUTE steer (vertical); its wheel: CONTINUOUS roll (child).
#   - rear-left wheel: CONTINUOUS roll.
#   - rear-right wheel: CONTINUOUS roll.

from math import atan2, cos, pi, sin, sqrt

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

# ---- drivetrain (chain-and-sprocket) dimensions ----
# Both sprockets share the same X plane (right side of kart, inboard).
SPROCKET_X = -0.25
REAR_SPROCKET_R = 0.074       # rear sprocket outer (tooth tip) radius
REAR_SPROCKET_BORE = 0.019    # keyed onto the live rear axle
REAR_SPROCKET_W = 0.012       # face width along axle
ENGINE_SPROCKET_R = 0.026     # small engine-output sprocket
ENGINE_SPROCKET_BORE = 0.010
ENGINE_SPROCKET_W = 0.009
CLUTCH_DRUM_R = 0.040         # centrifugal clutch drum outboard of engine
CLUTCH_DRUM_W = 0.032
CLUTCH_X = -0.20              # clutch drum center X
SHAFT_X_MIN = -0.18           # output shaft exit from engine block
SHAFT_X_MAX = -0.26           # shaft end past sprocket
# Engine block sits on the right side, forward of the rear axle.
ENGINE_BLOCK_XYZ = (-0.12, -0.54, 0.22)
CRANKSHAFT_Z = 0.19           # engine output shaft / sprocket axis height
ENGINE_SPROCKET_Y = -0.54
REAR_SPROCKET_Y = REAR_AXLE_Y
REAR_SPROCKET_Z = AXLE_Z_REAR


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


def _sprocket_mesh(name, outer_r, bore_r, width, n_segments=48):
    """Lathe a sprocket disc: bore → hub → thin web → tooth ring at outer radius.
    Axis is revolved around Z then rotated so it lies along local X."""
    hw = width * 0.5
    hub_r = bore_r * 2.2
    web_r = (hub_r + outer_r * 0.82) * 0.5
    root_r = outer_r * 0.88
    tip_hw = hw * 0.58  # teeth narrower than the disc body
    profile = [
        (bore_r, -hw),
        (bore_r, hw),
        (hub_r, hw),
        (hub_r, hw * 0.65),
        (web_r, hw * 0.38),
        (root_r, hw * 0.55),
        (root_r, tip_hw),
        (outer_r, tip_hw * 0.72),
        (outer_r, -tip_hw * 0.72),
        (root_r, -tip_hw),
        (root_r, -hw * 0.55),
        (web_r, -hw * 0.38),
        (hub_r, -hw * 0.65),
        (hub_r, -hw),
    ]
    return _mesh(name, LatheGeometry(profile, segments=n_segments).rotate_y(pi / 2.0))


def _chain_loop_points():
    """Generate a closed chain path wrapping the rear and engine sprockets.
    The chain lies in the YZ plane at SPROCKET_X."""
    x = SPROCKET_X
    rcy, rcz = REAR_SPROCKET_Y, REAR_SPROCKET_Z
    rr = REAR_SPROCKET_R * 0.93  # chain rides at the tooth root of the sprocket
    ecy, ecz = ENGINE_SPROCKET_Y, CRANKSHAFT_Z
    er = ENGINE_SPROCKET_R * 0.93

    dy = ecy - rcy
    dz = ecz - rcz
    base_ang = atan2(dz, dy)
    d = (dy * dy + dz * dz) ** 0.5
    delta = atan2(rr - er, d) if d > abs(rr - er) else 0.0

    u_ang = base_ang + pi / 2.0 + delta   # upper tangent angle
    l_ang = base_ang - pi / 2.0 - delta   # lower tangent angle

    pts = []
    n_arc = 16

    # Arc around rear sprocket (upper tangent → away side → lower tangent)
    rear_sweep = u_ang - l_ang
    if rear_sweep < 0.0:
        rear_sweep += 2.0 * pi
    for i in range(n_arc + 1):
        t = i / n_arc
        ang = u_ang - t * rear_sweep
        pts.append((x, rcy + rr * cos(ang), rcz + rr * sin(ang)))

    # Straight upper run to engine sprocket upper tangent
    pts.append((x, ecy + er * cos(u_ang), ecz + er * sin(u_ang)))

    # Arc around engine sprocket (upper tangent → away side → lower tangent)
    eng_sweep = 2.0 * pi - rear_sweep
    for i in range(n_arc + 1):
        t = i / n_arc
        ang = u_ang + t * eng_sweep
        pts.append((x, ecy + er * cos(ang), ecz + er * sin(ang)))

    # Close the loop back to the first point
    pts.append(pts[0])
    return pts


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
    sprocket_mat = model.material("sprocket_brass", rgba=(0.72, 0.56, 0.22, 1.0))
    engine_mat = model.material("engine_aluminum", rgba=(0.62, 0.63, 0.66, 1.0))
    clutch_mat = model.material("clutch_dark", rgba=(0.30, 0.30, 0.32, 1.0))
    chain_mat = model.material("chain_oiled_steel", rgba=(0.22, 0.22, 0.24, 1.0))

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

    # ================= chain-and-sprocket drivetrain =================
    # All drivetrain visuals are chassis-inline (non-moving decorations):
    # engine block, clutch drum, output shaft, two sprockets, and chain loop.
    ex, ey, ez = ENGINE_BLOCK_XYZ

    # --- engine mount plate (welded to frame rails under the engine) ---
    chassis.visual(
        Box((0.22, 0.24, 0.04)),
        origin=Origin(xyz=(ex - 0.02, ey, frame_z)),
        material=steel,
        name="engine_mount_plate",
    )
    # Vertical engine mount brackets connecting the plate to the engine block
    for i, bx in enumerate((-0.06, -0.18)):
        bracket_h = ez - 0.07 - (frame_z + 0.02)
        chassis.visual(
            Box((0.03, 0.03, bracket_h)),
            origin=Origin(xyz=(bx, ey, (frame_z + 0.02 + ez - 0.07) * 0.5)),
            material=steel,
            name=f"engine_mount_bracket_{i}",
        )

    # --- engine block (compact single-cylinder, crankcase + cylinder) ---
    chassis.visual(
        Box((0.13, 0.15, 0.14)),
        origin=Origin(xyz=(ex, ey, ez)),
        material=engine_mat,
        name="engine_block",
    )
    # Cylinder barrel rising from the crankcase
    chassis.visual(
        Cylinder(radius=0.042, length=0.08),
        origin=Origin(xyz=(ex + 0.01, ey + 0.02, ez + 0.11)),
        material=engine_mat,
        name="cylinder_barrel",
    )
    # Cooling fins around the cylinder (repeated thin discs)
    for i in range(5):
        z_fin = ez + 0.08 + i * 0.014
        chassis.visual(
            Cylinder(radius=0.052, length=0.003),
            origin=Origin(xyz=(ex + 0.01, ey + 0.02, z_fin)),
            material=engine_mat,
            name=f"cooling_fin_{i}",
        )
    # Cylinder head cap
    chassis.visual(
        Cylinder(radius=0.046, length=0.022),
        origin=Origin(xyz=(ex + 0.01, ey + 0.02, ez + 0.15)),
        material=engine_mat,
        name="cylinder_head",
    )
    # Air filter housing on top
    chassis.visual(
        Cylinder(radius=0.032, length=0.045),
        origin=Origin(xyz=(ex + 0.01, ey + 0.04, ez + 0.18)),
        material=dark_steel,
        name="air_filter",
    )
    # Exhaust header pipe curving rearward and down from the cylinder barrel
    exhaust_pts = [
        (ex + 0.01, ey - 0.01, ez + 0.08),   # starts inside the cylinder barrel
        (ex - 0.02, ey - 0.12, ez + 0.05),
        (ex - 0.06, ey - 0.26, ez + 0.025),
        (ex - 0.09, ey - 0.40, ez + 0.015),
    ]
    exhaust_tube = tube_from_spline_points(
        exhaust_pts, radius=0.014, samples_per_segment=10, radial_segments=10
    )
    chassis.visual(_mesh("exhaust_pipe.obj", exhaust_tube), material=dark_steel)

    # --- engine output shaft (horizontal, along X, from block side to sprocket) ---
    shaft_len = abs(SHAFT_X_MAX - SHAFT_X_MIN)
    shaft_cx = (SHAFT_X_MIN + SHAFT_X_MAX) * 0.5
    chassis.visual(
        Cylinder(radius=0.012, length=shaft_len),
        origin=Origin(xyz=(shaft_cx, ENGINE_SPROCKET_Y, CRANKSHAFT_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="output_shaft",
    )

    # --- clutch drum (centrifugal clutch around the shaft, inboard of sprocket) ---
    chassis.visual(
        Cylinder(radius=CLUTCH_DRUM_R, length=CLUTCH_DRUM_W),
        origin=Origin(xyz=(CLUTCH_X, ENGINE_SPROCKET_Y, CRANKSHAFT_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=clutch_mat,
        name="clutch_drum",
    )

    # --- engine (drive) sprocket ---
    chassis.visual(
        _sprocket_mesh("engine_sprocket.obj", ENGINE_SPROCKET_R, ENGINE_SPROCKET_BORE, ENGINE_SPROCKET_W),
        origin=Origin(xyz=(SPROCKET_X, ENGINE_SPROCKET_Y, CRANKSHAFT_Z)),
        material=sprocket_mat,
        name="engine_sprocket",
    )

    # --- rear axle sprocket (keyed onto the live rear axle) ---
    chassis.visual(
        _sprocket_mesh("rear_sprocket.obj", REAR_SPROCKET_R, REAR_SPROCKET_BORE, REAR_SPROCKET_W),
        origin=Origin(xyz=(SPROCKET_X, REAR_SPROCKET_Y, REAR_SPROCKET_Z)),
        material=sprocket_mat,
        name="rear_sprocket",
    )

    # --- drive chain (continuous loop wrapping both sprockets) ---
    chain_pts = _chain_loop_points()
    chain_tube = tube_from_spline_points(
        chain_pts,
        radius=0.005,
        samples_per_segment=6,
        radial_segments=8,
        closed_spline=False,
        cap_ends=False,
    )
    chassis.visual(_mesh("drive_chain.obj", chain_tube), material=chain_mat, name="drive_chain")

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
        Box((0.42, 0.46, 0.40)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
    )
    # Bucket seat: a low seat pan lofted along +Y, then a backrest panel that
    # rises in z toward the rear. Sections are (y, z_min, z_max, width_x),
    # centered at x=0, swept along +Y. seat-local z=0 is the seat floor.
    seat_pan = superellipse_side_loft(
        [
            (0.20, 0.02, 0.07, 0.30),
            (0.02, 0.00, 0.06, 0.40),
            (-0.14, 0.01, 0.07, 0.36),
        ],
        exponents=2.4,
        segments=40,
    )
    seat.visual(_mesh("seat_pan.obj", seat_pan), material=seat_mat)
    # Backrest: tall, thin in Y, rising at the rear of the pan.
    backrest = superellipse_side_loft(
        [
            (-0.14, 0.02, 0.34, 0.36),
            (-0.20, 0.02, 0.38, 0.34),
            (-0.25, 0.02, 0.32, 0.30),
        ],
        exponents=2.6,
        segments=40,
    )
    seat.visual(_mesh("seat_back.obj", backrest), material=seat_mat)
    # Side bolsters along the seat pan edges.
    seat.visual(Box((0.04, 0.34, 0.14)), origin=Origin(xyz=(0.19, 0.0, 0.10)), material=seat_mat, name="left_bolster")
    seat.visual(Box((0.04, 0.34, 0.14)), origin=Origin(xyz=(-0.19, 0.0, 0.10)), material=seat_mat, name="right_bolster")

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

    # ---- front wheel is carried by its knuckle (no floating) ----
    ctx.expect_contact(fl_wheel, fl_knuckle, name="front-left wheel on its knuckle")
    ctx.expect_contact(fr_wheel, fr_knuckle, name="front-right wheel on its knuckle")

    # ================= drivetrain (chain-and-sprocket) checks =================
    rear_sp_aabb = ctx.part_element_world_aabb(chassis, elem="rear_sprocket")
    eng_sp_aabb = ctx.part_element_world_aabb(chassis, elem="engine_sprocket")
    chain_aabb = ctx.part_element_world_aabb(chassis, elem="drive_chain")
    clutch_aabb = ctx.part_element_world_aabb(chassis, elem="clutch_drum")

    # Rear sprocket sits at axle height on the live rear axle
    rear_sp_cz = (rear_sp_aabb[0][2] + rear_sp_aabb[1][2]) * 0.5
    ctx.check(
        "rear sprocket centered on rear axle height",
        abs(rear_sp_cz - AXLE_Z_REAR) < 0.02,
        details=f"rear sprocket center z={rear_sp_cz:.4f}, axle z={AXLE_Z_REAR}",
    )

    # Engine sprocket is above the rear axle (chain runs upward to it)
    eng_sp_cz = (eng_sp_aabb[0][2] + eng_sp_aabb[1][2]) * 0.5
    ctx.check(
        "engine sprocket above rear axle line",
        eng_sp_cz > AXLE_Z_REAR + 0.01,
        details=f"engine sprocket center z={eng_sp_cz:.4f}",
    )

    # Both sprockets on the same side of the kart (right = -X)
    rear_sp_cx = (rear_sp_aabb[0][0] + rear_sp_aabb[1][0]) * 0.5
    eng_sp_cx = (eng_sp_aabb[0][0] + eng_sp_aabb[1][0]) * 0.5
    ctx.check(
        "drivetrain on right side of kart",
        rear_sp_cx < -0.05 and eng_sp_cx < -0.05,
        details=f"rear sprocket x={rear_sp_cx:.3f}, engine sprocket x={eng_sp_cx:.3f}",
    )

    # Chain spans both sprockets in Y (fore-aft run)
    ctx.check(
        "chain spans rear to engine sprocket in Y",
        chain_aabb[0][1] < REAR_SPROCKET_Y + 0.02 and chain_aabb[1][1] > ENGINE_SPROCKET_Y - 0.02,
        details=f"chain Y=[{chain_aabb[0][1]:.3f}, {chain_aabb[1][1]:.3f}]",
    )

    # Chain spans the vertical range between both sprocket heights
    ctx.check(
        "chain spans both sprocket heights",
        chain_aabb[0][2] < REAR_SPROCKET_Z + 0.01 and chain_aabb[1][2] > CRANKSHAFT_Z - 0.01,
        details=f"chain Z=[{chain_aabb[0][2]:.3f}, {chain_aabb[1][2]:.3f}]",
    )

    # Rear sprocket is larger than engine sprocket (diameter comparison)
    rear_sp_dz = rear_sp_aabb[1][2] - rear_sp_aabb[0][2]
    eng_sp_dz = eng_sp_aabb[1][2] - eng_sp_aabb[0][2]
    ctx.check(
        "rear sprocket larger than engine sprocket",
        rear_sp_dz > eng_sp_dz + 0.02,
        details=f"rear dz={rear_sp_dz:.4f}, engine dz={eng_sp_dz:.4f}",
    )

    # Clutch drum is between the engine and the sprocket in X
    clutch_cx = (clutch_aabb[0][0] + clutch_aabb[1][0]) * 0.5
    ctx.check(
        "clutch drum between engine and sprocket",
        eng_sp_cx < clutch_cx < -0.05,
        details=f"clutch x={clutch_cx:.3f}, engine sprocket x={eng_sp_cx:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
