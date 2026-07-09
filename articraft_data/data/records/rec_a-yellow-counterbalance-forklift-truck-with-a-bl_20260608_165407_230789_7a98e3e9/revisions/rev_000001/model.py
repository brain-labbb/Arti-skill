from __future__ import annotations

# Yellow counterbalance forklift truck.
# Convention (real meters, Z-up): truck FORWARD = +X (forks point +X),
# width along Y, up = Z. Wheel axles along Y. Tires touch z = 0.
# Overall ~2.7 m long incl forks, ~1.1 m wide, ~2.1 m tall to the guard.
#
# Articulation:
#   - fork_carriage LIFT along the mast  -> PRISMATIC, axis +Z, 0 .. ~1.0 m
#   - rear_steer_l / rear_steer_r steer  -> REVOLUTE about Z (chassis → steer knuckle)
#   - rear_wheel_l / rear_wheel_r spin   -> CONTINUOUS about Y (steer knuckle → wheel)
#   - front_wheel_l / front_wheel_r spin -> CONTINUOUS about Y
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Key dimensions
# ---------------------------------------------------------------------------
FRONT_TIRE_R = 0.275            # front drive tire radius (~0.55 m dia)
FRONT_TIRE_W = 0.22
REAR_TIRE_R = 0.21             # rear steer tire radius (~0.42 m dia)
REAR_TIRE_W = 0.16

# Axle heights so tires touch z = 0.
FRONT_AXLE_Z = FRONT_TIRE_R
REAR_AXLE_Z = REAR_TIRE_R

# Track / wheelbase
FRONT_AXLE_X = 0.42            # front drive axle forward of chassis origin
REAR_AXLE_X = -0.78           # rear steer axle behind chassis origin
FRONT_TRACK_Y = 0.45          # half-track of front wheels
REAR_TRACK_Y = 0.42           # half-track of rear wheels

# Chassis body block (the yellow hood / lower body).
BODY_L = 0.80
BODY_W = 0.64
BODY_H = 0.40
BODY_CX = -0.10               # body centered slightly to the rear
BODY_FLOOR_Z = 0.20          # underside of the main hood block

# Mast
MAST_FRONT_X = 0.50          # mast plane at the very front of the chassis
MAST_RAIL_H = 1.85
MAST_RAIL_GAP = 0.34         # clear inner spacing between the two channels
MAST_RAIL_W = 0.07           # channel width (Y)
MAST_RAIL_D = 0.10           # channel depth (X)
MAST_BASE_Z = 0.05

# Carriage
CARRIAGE_W = MAST_RAIL_GAP + 2.0 * MAST_RAIL_W + 0.04   # embraces both rails
CARRIAGE_PLATE_T = 0.05      # plate thickness (X)
CARRIAGE_H = 0.42
CARRIAGE_REST_Z = 0.30       # carriage center z when lowered (forks near ground)
LIFT_TRAVEL = 1.0            # prismatic travel upper limit

# Forks
FORK_LEN = 1.00
FORK_W = 0.12
FORK_BLADE_T = 0.04
FORK_HEEL_H = 0.36
FORK_GAUGE_Y = 0.22          # half-spacing between the two forks (clears the tires)


def _front_wheel_visuals(part, name_prefix, wheel_steel, dark_rubber):
    """Larger smooth-shouldered drive wheel; axle along local Y."""
    wheel = WheelGeometry(
        FRONT_TIRE_R * 0.66,
        FRONT_TIRE_W * 0.70,
        rim=WheelRim(inner_radius=FRONT_TIRE_R * 0.44, flange_height=0.012, flange_thickness=0.008),
        hub=WheelHub(
            radius=FRONT_TIRE_R * 0.22,
            width=FRONT_TIRE_W * 0.78,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=6, circle_diameter=FRONT_TIRE_R * 0.32, hole_diameter=0.014),
        ),
        face=WheelFace(dish_depth=0.008, front_inset=0.004),
        spokes=WheelSpokes(style="disc", thickness=0.014),
        bore=WheelBore(style="round", diameter=0.04),
    )
    tire = TireGeometry(
        FRONT_TIRE_R,
        FRONT_TIRE_W,
        inner_radius=FRONT_TIRE_R * 0.66,
        carcass=TireCarcass(belt_width_ratio=0.74, sidewall_bulge=0.05),
        tread=TireTread(style="block", depth=0.008, count=20, land_ratio=0.7),
        sidewall=TireSidewall(style="bulged", bulge=0.04),
        shoulder=TireShoulder(style="square", width=0.012, radius=0.006),
    )
    # Geometry is built about local X; orient so the axle lies along local Y.
    axle_y = Origin(rpy=(0.0, 0.0, pi / 2.0))
    part.visual(
        mesh_from_geometry(tire, f"{name_prefix}_tire"),
        origin=axle_y,
        material=dark_rubber,
        name=f"{name_prefix}_tire",
    )
    part.visual(
        mesh_from_geometry(wheel, f"{name_prefix}_wheel"),
        origin=axle_y,
        material=wheel_steel,
        name=f"{name_prefix}_wheel",
    )


def _rear_wheel_visuals(part, name_prefix, wheel_steel, dark_rubber):
    """Smaller steer wheel; axle along local Y."""
    wheel = WheelGeometry(
        REAR_TIRE_R * 0.64,
        REAR_TIRE_W * 0.70,
        rim=WheelRim(inner_radius=REAR_TIRE_R * 0.42, flange_height=0.010, flange_thickness=0.006),
        hub=WheelHub(
            radius=REAR_TIRE_R * 0.24,
            width=REAR_TIRE_W * 0.78,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=5, circle_diameter=REAR_TIRE_R * 0.32, hole_diameter=0.012),
        ),
        face=WheelFace(dish_depth=0.006, front_inset=0.003),
        spokes=WheelSpokes(style="disc", thickness=0.012),
        bore=WheelBore(style="round", diameter=0.03),
    )
    tire = TireGeometry(
        REAR_TIRE_R,
        REAR_TIRE_W,
        inner_radius=REAR_TIRE_R * 0.66,
        carcass=TireCarcass(belt_width_ratio=0.74, sidewall_bulge=0.05),
        tread=TireTread(style="smooth", depth=0.0, count=0, land_ratio=1.0),
        sidewall=TireSidewall(style="bulged", bulge=0.04),
        shoulder=TireShoulder(style="square", width=0.010, radius=0.005),
    )
    axle_y = Origin(rpy=(0.0, 0.0, pi / 2.0))
    part.visual(
        mesh_from_geometry(tire, f"{name_prefix}_tire"),
        origin=axle_y,
        material=dark_rubber,
        name=f"{name_prefix}_tire",
    )
    part.visual(
        mesh_from_geometry(wheel, f"{name_prefix}_wheel"),
        origin=axle_y,
        material=wheel_steel,
        name=f"{name_prefix}_wheel",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="counterbalance_forklift")

    yellow = model.material("forklift_yellow", rgba=(0.92, 0.74, 0.10, 1.0))
    dark_yellow = model.material("hood_yellow", rgba=(0.80, 0.64, 0.08, 1.0))
    black_steel = model.material("black_steel", rgba=(0.11, 0.11, 0.12, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.16, 0.16, 0.18, 1.0))
    grey_steel = model.material("grey_steel", rgba=(0.55, 0.56, 0.58, 1.0))
    counterweight_grey = model.material("counterweight_grey", rgba=(0.30, 0.31, 0.33, 1.0))
    seat_blue = model.material("seat_blue", rgba=(0.22, 0.40, 0.58, 1.0))
    wheel_steel = model.material("wheel_steel", rgba=(0.50, 0.51, 0.54, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.07, 0.07, 0.08, 1.0))
    fork_steel = model.material("fork_steel", rgba=(0.56, 0.57, 0.59, 1.0))
    backrest_brown = model.material("backrest_brown", rgba=(0.46, 0.26, 0.18, 1.0))

    # -------------------------------------------------------------------
    # CHASSIS (root): lower body, counterweight, floor, seat, controls,
    #                 overhead guard.
    # -------------------------------------------------------------------
    chassis = model.part("chassis")

    # Main hood / lower body block (yellow).
    chassis.visual(
        Box((BODY_L, BODY_W, BODY_H)),
        origin=Origin(xyz=(BODY_CX, 0.0, BODY_FLOOR_Z + BODY_H / 2.0)),
        material=yellow,
        name="lower_body",
    )
    # Sloped/raised engine hood behind the operator (a second, taller block).
    chassis.visual(
        Box((0.50, 0.62, 0.26)),
        origin=Origin(xyz=(-0.44, 0.0, BODY_FLOOR_Z + 0.40)),
        material=dark_yellow,
        name="engine_hood",
    )
    # Front frame nose under the mast (kept clear of the mast plane at x=0.50).
    chassis.visual(
        Box((0.26, 0.62, 0.28)),
        origin=Origin(xyz=(0.22, 0.0, FRONT_AXLE_Z + 0.02)),
        material=yellow,
        name="front_frame",
    )
    # Mast mounting brackets: bridge the front frame forward to the mast base
    # plane so the mast is physically carried by the chassis (no gap).
    # Their front face stops just at the mast base, behind the carriage hooks.
    for sign, idx in ((1.0, 1), (-1.0, 0)):
        chassis.visual(
            Box((0.22, 0.12, 0.20)),
            origin=Origin(xyz=(0.36, sign * 0.22, 0.18)),
            material=dark_steel,
            name=f"mast_mount_{idx}",
        )
    # Front axle housing (connects the two front wheels visibly).
    # Sits just behind the wheel centerline so it clears the mast/lift plane.
    chassis.visual(
        Cylinder(radius=0.06, length=2.0 * FRONT_TRACK_Y - 0.06),
        origin=Origin(xyz=(FRONT_AXLE_X, 0.0, FRONT_AXLE_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="front_axle",
    )
    # Rear axle / steer beam.
    chassis.visual(
        Box((0.14, 2.0 * REAR_TRACK_Y - 0.04, 0.12)),
        origin=Origin(xyz=(REAR_AXLE_X, 0.0, REAR_AXLE_Z)),
        material=dark_steel,
        name="rear_axle_beam",
    )

    # Heavy rear COUNTERWEIGHT block (touches the body so the rear assembly
    # reads as one connected mass).
    chassis.visual(
        Box((0.40, 0.62, 0.50)),
        origin=Origin(xyz=(-0.70, 0.0, REAR_AXLE_Z + 0.27)),
        material=counterweight_grey,
        name="counterweight",
    )

    # Operator floor / step.
    chassis.visual(
        Box((0.34, 0.62, 0.04)),
        origin=Origin(xyz=(0.06, 0.0, BODY_FLOOR_Z + 0.02)),
        material=black_steel,
        name="operator_floor",
    )

    # Blue seat: cushion + backrest.
    seat_x = -0.24
    seat_base_z = BODY_FLOOR_Z + BODY_H  # ~0.60
    chassis.visual(
        Box((0.34, 0.42, 0.10)),
        origin=Origin(xyz=(seat_x, 0.0, seat_base_z + 0.05)),
        material=seat_blue,
        name="seat_cushion",
    )
    chassis.visual(
        Box((0.10, 0.42, 0.34)),
        origin=Origin(xyz=(seat_x - 0.16, 0.0, seat_base_z + 0.22)),
        material=seat_blue,
        name="seat_back",
    )

    # Dashboard / cowl in front of the operator.
    dash_x = 0.16
    chassis.visual(
        Box((0.14, 0.50, 0.18)),
        origin=Origin(xyz=(dash_x, 0.0, BODY_FLOOR_Z + 0.30)),
        material=dark_yellow,
        name="dashboard",
    )
    # Steering column (tilted) + small steering wheel.
    chassis.visual(
        Cylinder(radius=0.022, length=0.30),
        origin=Origin(xyz=(dash_x - 0.04, 0.0, BODY_FLOOR_Z + 0.48), rpy=(0.0, -0.5, 0.0)),
        material=dark_steel,
        name="steering_column",
    )
    chassis.visual(
        Cylinder(radius=0.12, length=0.025),
        origin=Origin(xyz=(dash_x - 0.12, 0.0, BODY_FLOOR_Z + 0.60), rpy=(0.0, -0.5 + pi / 2.0, 0.0)),
        material=black_steel,
        name="steering_wheel_rim",
    )
    chassis.visual(
        Cylinder(radius=0.03, length=0.03),
        origin=Origin(xyz=(dash_x - 0.12, 0.0, BODY_FLOOR_Z + 0.60), rpy=(0.0, -0.5 + pi / 2.0, 0.0)),
        material=dark_steel,
        name="steering_hub",
    )

    # OVERHEAD GUARD (ROPS): 4 black posts + flat roof cage.
    # Posts rise from the body corners (kept within the body top footprint so
    # each post is physically connected to the chassis, not floating).
    guard_top_z = 2.05
    post_half = 0.05
    post_x_front = 0.24
    post_x_rear = -0.48
    post_y = 0.29
    post_corners = [
        (post_x_front, post_y),
        (post_x_front, -post_y),
        (post_x_rear, post_y),
        (post_x_rear, -post_y),
    ]
    # Posts overlap into the body top so they read as connected.
    post_base_z = BODY_FLOOR_Z + BODY_H - 0.06  # ~0.54
    post_h = guard_top_z - post_base_z
    for i, (px, py) in enumerate(post_corners):
        chassis.visual(
            Box((2.0 * post_half, 2.0 * post_half, post_h)),
            origin=Origin(xyz=(px, py, post_base_z + post_h / 2.0)),
            material=black_steel,
            name=f"guard_post_{i}",
        )
    # Roof cage frame: perimeter rails + a couple of cross bars.
    roof_frame_t = 0.05
    roof_z = guard_top_z
    cage_len = post_x_front - post_x_rear
    cage_cx = (post_x_front + post_x_rear) / 2.0
    # Side rails (run X).
    for py in (post_y, -post_y):
        chassis.visual(
            Box((cage_len + 2.0 * post_half, roof_frame_t, roof_frame_t)),
            origin=Origin(xyz=(cage_cx, py, roof_z)),
            material=black_steel,
            name=f"roof_rail_x_{1 if py > 0 else 0}",
        )
    # End rails (run Y).
    for px in (post_x_front, post_x_rear):
        chassis.visual(
            Box((roof_frame_t, 2.0 * post_y + roof_frame_t, roof_frame_t)),
            origin=Origin(xyz=(px, 0.0, roof_z)),
            material=black_steel,
            name=f"roof_rail_y_{1 if px > 0 else 0}",
        )
    # Cross bars across the roof.
    for cx in (cage_cx - 0.22, cage_cx + 0.22):
        chassis.visual(
            Box((0.035, 2.0 * post_y, 0.035)),
            origin=Origin(xyz=(cx, 0.0, roof_z)),
            material=dark_steel,
            name=f"roof_cross_{0 if cx < cage_cx else 1}",
        )

    chassis.inertial = Inertial.from_geometry(
        Box((2.0, BODY_W, 2.1)),
        mass=2600.0,
        origin=Origin(xyz=(-0.25, 0.0, 0.55)),
    )

    # -------------------------------------------------------------------
    # WHEELS
    # -------------------------------------------------------------------
    front_wheel_l = model.part("front_wheel_l")
    _front_wheel_visuals(front_wheel_l, "front_l", wheel_steel, dark_rubber)
    front_wheel_l.inertial = Inertial.from_geometry(
        Cylinder(radius=FRONT_TIRE_R, length=FRONT_TIRE_W),
        mass=80.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    front_wheel_r = model.part("front_wheel_r")
    _front_wheel_visuals(front_wheel_r, "front_r", wheel_steel, dark_rubber)
    front_wheel_r.inertial = Inertial.from_geometry(
        Cylinder(radius=FRONT_TIRE_R, length=FRONT_TIRE_W),
        mass=80.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    rear_wheel_l = model.part("rear_wheel_l")
    _rear_wheel_visuals(rear_wheel_l, "rear_l", wheel_steel, dark_rubber)
    rear_wheel_l.inertial = Inertial.from_geometry(
        Cylinder(radius=REAR_TIRE_R, length=REAR_TIRE_W),
        mass=45.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    rear_wheel_r = model.part("rear_wheel_r")
    _rear_wheel_visuals(rear_wheel_r, "rear_r", wheel_steel, dark_rubber)
    rear_wheel_r.inertial = Inertial.from_geometry(
        Cylinder(radius=REAR_TIRE_R, length=REAR_TIRE_W),
        mass=45.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    # Steer knuckles: pivot parts that carry the rear wheel spin joint.
    # Each has a vertical kingpin (steering axis Z) and a horizontal spindle (wheel spin axis Y).
    rear_steer_l = model.part("rear_steer_l")
    rear_steer_l.visual(
        Cylinder(radius=0.026, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_steel,
        name="rear_l_kingpin",
    )
    rear_steer_l.visual(
        Cylinder(radius=0.030, length=REAR_TIRE_W + 0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grey_steel,
        name="rear_l_spindle",
    )
    rear_steer_l.inertial = Inertial.from_geometry(
        Cylinder(radius=0.04, length=REAR_TIRE_W),
        mass=4.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    rear_steer_r = model.part("rear_steer_r")
    rear_steer_r.visual(
        Cylinder(radius=0.026, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_steel,
        name="rear_r_kingpin",
    )
    rear_steer_r.visual(
        Cylinder(radius=0.030, length=REAR_TIRE_W + 0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grey_steel,
        name="rear_r_spindle",
    )
    rear_steer_r.inertial = Inertial.from_geometry(
        Cylinder(radius=0.04, length=REAR_TIRE_W),
        mass=4.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    # -------------------------------------------------------------------
    # MAST: two vertical channel uprights + top cross-tie.
    # The mast part frame sits at the chassis front, base on the floor.
    # -------------------------------------------------------------------
    mast = model.part("mast")
    rail_cy = MAST_RAIL_GAP / 2.0 + MAST_RAIL_W / 2.0  # center of each channel
    for sign, idx in ((1.0, 1), (-1.0, 0)):
        # Outer channel web.
        mast.visual(
            Box((MAST_RAIL_D, MAST_RAIL_W, MAST_RAIL_H)),
            origin=Origin(xyz=(0.0, sign * rail_cy, MAST_BASE_Z + MAST_RAIL_H / 2.0)),
            material=dark_steel,
            name=f"mast_rail_{idx}",
        )
        # Inner flange lip facing the gap (gives the channel a C-section read).
        mast.visual(
            Box((MAST_RAIL_D * 0.45, 0.02, MAST_RAIL_H)),
            origin=Origin(
                xyz=(MAST_RAIL_D * 0.20, sign * (rail_cy - MAST_RAIL_W / 2.0 - 0.01),
                     MAST_BASE_Z + MAST_RAIL_H / 2.0)
            ),
            material=grey_steel,
            name=f"mast_lip_{idx}",
        )
    # Top cross-tie joining the two uprights.
    mast.visual(
        Box((MAST_RAIL_D, MAST_RAIL_GAP + 2.0 * MAST_RAIL_W, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, MAST_BASE_Z + MAST_RAIL_H - 0.04)),
        material=dark_steel,
        name="mast_top_tie",
    )
    # Lower cross-tie / tilt cylinder anchor near the base.
    mast.visual(
        Box((MAST_RAIL_D, MAST_RAIL_GAP + 2.0 * MAST_RAIL_W, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, MAST_BASE_Z + 0.18)),
        material=dark_steel,
        name="mast_base_tie",
    )
    # Hydraulic lift cylinder running up the center of the mast.
    mast.visual(
        Cylinder(radius=0.05, length=MAST_RAIL_H - 0.2),
        origin=Origin(xyz=(-0.02, 0.0, MAST_BASE_Z + (MAST_RAIL_H - 0.2) / 2.0 + 0.1)),
        material=grey_steel,
        name="lift_cylinder",
    )
    mast.inertial = Inertial.from_geometry(
        Box((MAST_RAIL_D, MAST_RAIL_GAP + 2.0 * MAST_RAIL_W, MAST_RAIL_H)),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, MAST_BASE_Z + MAST_RAIL_H / 2.0)),
    )

    # -------------------------------------------------------------------
    # FORK CARRIAGE: plate riding up the mast + backrest cage + 2 forks.
    # The carriage part frame is centered at the carriage plate.
    # -------------------------------------------------------------------
    carriage = model.part("fork_carriage")
    # Main carriage plate (faces the mast; thin in X).
    carriage.visual(
        Box((CARRIAGE_PLATE_T, CARRIAGE_W, CARRIAGE_H)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=fork_steel,
        name="carriage_plate",
    )
    # Side hooks that embrace the two mast rails (front of the plate).
    for sign, idx in ((1.0, 1), (-1.0, 0)):
        carriage.visual(
            Box((MAST_RAIL_D + 0.05, 0.05, 0.14)),
            origin=Origin(xyz=(-0.02, sign * rail_cy, CARRIAGE_H / 2.0 - 0.09)),
            material=dark_steel,
            name=f"carriage_hook_top_{idx}",
        )
        carriage.visual(
            Box((MAST_RAIL_D + 0.05, 0.05, 0.14)),
            origin=Origin(xyz=(-0.02, sign * rail_cy, -CARRIAGE_H / 2.0 + 0.09)),
            material=dark_steel,
            name=f"carriage_hook_bot_{idx}",
        )
    # Reddish-brown fork BACKREST / load guard cage on the front face.
    bx = CARRIAGE_PLATE_T / 2.0 + 0.025
    backrest_top = CARRIAGE_H / 2.0 + 0.20
    # Frame: 2 vertical posts + 2 horizontal rails (open cage look).
    for sign, idx in ((1.0, 1), (-1.0, 0)):
        carriage.visual(
            Box((0.05, 0.05, backrest_top + CARRIAGE_H / 2.0)),
            origin=Origin(xyz=(bx, sign * 0.26, 0.10)),
            material=backrest_brown,
            name=f"backrest_post_{idx}",
        )
    for rz, idx in ((CARRIAGE_H / 2.0 + 0.02, 0), (backrest_top, 1)):
        carriage.visual(
            Box((0.05, 0.60, 0.05)),
            origin=Origin(xyz=(bx, 0.0, rz)),
            material=backrest_brown,
            name=f"backrest_rail_{idx}",
        )
    # Lower backrest panel against the load.
    carriage.visual(
        Box((0.04, 0.58, CARRIAGE_H * 0.7)),
        origin=Origin(xyz=(bx - 0.005, 0.0, -0.02)),
        material=backrest_brown,
        name="backrest_panel",
    )

    # Two L-shaped FORKS pointing +X. Each = vertical heel (on the carriage)
    # + horizontal blade along +X near the ground.
    fork_root_x = bx + 0.02
    blade_z = -CARRIAGE_H / 2.0 - 0.06   # blade just below the carriage plate
    for sign, idx in ((1.0, 1), (-1.0, 0)):
        fy = sign * FORK_GAUGE_Y
        # Vertical heel section riding the carriage front.
        carriage.visual(
            Box((FORK_BLADE_T, FORK_W, FORK_HEEL_H)),
            origin=Origin(xyz=(fork_root_x, fy, blade_z + FORK_HEEL_H / 2.0)),
            material=fork_steel,
            name=f"fork_heel_{idx}",
        )
        # Horizontal blade.
        carriage.visual(
            Box((FORK_LEN, FORK_W, FORK_BLADE_T)),
            origin=Origin(xyz=(fork_root_x + FORK_LEN / 2.0, fy, blade_z + FORK_BLADE_T / 2.0)),
            material=fork_steel,
            name=f"fork_blade_{idx}",
        )
        # Tapered tip plate (reads as a pointed fork tip).
        carriage.visual(
            Box((0.16, FORK_W * 0.6, FORK_BLADE_T * 0.6)),
            origin=Origin(xyz=(fork_root_x + FORK_LEN + 0.06, fy, blade_z + FORK_BLADE_T * 0.3)),
            material=fork_steel,
            name=f"fork_tip_{idx}",
        )

    carriage.inertial = Inertial.from_geometry(
        Box((FORK_LEN, CARRIAGE_W, CARRIAGE_H + 0.5)),
        mass=120.0,
        origin=Origin(xyz=(0.4, 0.0, 0.0)),
    )

    # -------------------------------------------------------------------
    # ARTICULATIONS
    # -------------------------------------------------------------------
    # Front drive wheels: CONTINUOUS spin about Y.
    model.articulation(
        "front_wheel_l_spin",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=front_wheel_l,
        origin=Origin(xyz=(FRONT_AXLE_X, FRONT_TRACK_Y, FRONT_AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=400.0, velocity=20.0),
    )
    model.articulation(
        "front_wheel_r_spin",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=front_wheel_r,
        origin=Origin(xyz=(FRONT_AXLE_X, -FRONT_TRACK_Y, FRONT_AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=400.0, velocity=20.0),
    )
    # Rear steer joints: chassis → steer knuckle (REVOLUTE about Z).
    model.articulation(
        "rear_wheel_l_steer",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=rear_steer_l,
        origin=Origin(xyz=(REAR_AXLE_X, REAR_TRACK_Y, REAR_AXLE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=2.0, lower=-1.2, upper=1.2),
    )
    model.articulation(
        "rear_wheel_r_steer",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=rear_steer_r,
        origin=Origin(xyz=(REAR_AXLE_X, -REAR_TRACK_Y, REAR_AXLE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=2.0, lower=-1.2, upper=1.2),
    )
    # Rear wheel spin joints: steer knuckle → wheel (CONTINUOUS about Y).
    model.articulation(
        "rear_wheel_l_spin",
        ArticulationType.CONTINUOUS,
        parent=rear_steer_l,
        child=rear_wheel_l,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=20.0),
    )
    model.articulation(
        "rear_wheel_r_spin",
        ArticulationType.CONTINUOUS,
        parent=rear_steer_r,
        child=rear_wheel_r,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=20.0),
    )
    # Mast bolted to the chassis front.
    model.articulation(
        "chassis_to_mast",
        ArticulationType.FIXED,
        parent=chassis,
        child=mast,
        origin=Origin(xyz=(MAST_FRONT_X, 0.0, 0.0)),
    )
    # PRIMARY ARTICULATION: carriage LIFT along the mast (PRISMATIC, +Z).
    # Joint frame at the carriage rest center; child translates up +Z.
    model.articulation(
        "carriage_lift",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(MAST_RAIL_D / 2.0 + CARRIAGE_PLATE_T / 2.0, 0.0, CARRIAGE_REST_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4000.0, velocity=0.4, lower=0.0, upper=LIFT_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    chassis = object_model.get_part("chassis")
    mast = object_model.get_part("mast")
    carriage = object_model.get_part("fork_carriage")
    fwl = object_model.get_part("front_wheel_l")
    fwr = object_model.get_part("front_wheel_r")
    rwl = object_model.get_part("rear_wheel_l")
    rwr = object_model.get_part("rear_wheel_r")
    rsl = object_model.get_part("rear_steer_l")
    rsr = object_model.get_part("rear_steer_r")
    lift = object_model.get_articulation("carriage_lift")

    # The carriage embraces the mast rails -> intentional local overlap.
    ctx.allow_overlap(
        carriage,
        mast,
        reason="Carriage hooks intentionally wrap the mast rails to ride the uprights.",
    )
    # Front axle stub enters the drive wheel hub.
    for wheel_part in (fwl, fwr):
        ctx.allow_overlap(
            chassis,
            wheel_part,
            reason="The axle housing enters the front wheel hub at the mounting bore.",
        )
    # Rear axle beam enters the steer knuckle bore at the pivot.
    for steer_part in (rsl, rsr):
        ctx.allow_overlap(
            chassis,
            steer_part,
            reason="The rear axle beam enters the steer knuckle bore at the pivot.",
        )
    # Wheel hub bearing seats into the steer knuckle at the spin axis.
    for steer_part, wheel_part in ((rsl, rwl), (rsr, rwr)):
        ctx.allow_overlap(
            steer_part,
            wheel_part,
            reason="Wheel hub bearing seats into the steer knuckle at the spin axis.",
        )
    # The chassis mast-mount brackets bolt into the mast base -> the bolted
    # joint is modeled as a small intentional embedding.
    ctx.allow_overlap(
        chassis,
        mast,
        reason="Mast-mount brackets bolt the mast base to the chassis front (bolted joint embedding).",
    )

    # --- All four tires touch z = 0 (truck sits on its wheels). ---
    for name, part, r in (
        ("front_l", fwl, FRONT_TIRE_R),
        ("front_r", fwr, FRONT_TIRE_R),
        ("rear_l", rwl, REAR_TIRE_R),
        ("rear_r", rwr, REAR_TIRE_R),
    ):
        aabb = ctx.part_world_aabb(part)
        zmin = aabb[0][2]
        ctx.check(
            f"{name}_tire_on_ground",
            abs(zmin) <= 0.02,
            details=f"{name} tire zmin={zmin:.4f} (expected ~0)",
        )

    # --- Front tires are larger than rear tires. ---
    f_aabb = ctx.part_world_aabb(fwl)
    r_aabb = ctx.part_world_aabb(rwl)
    f_dia = f_aabb[1][2] - f_aabb[0][2]
    r_dia = r_aabb[1][2] - r_aabb[0][2]
    ctx.check(
        "front_tires_larger_than_rear",
        f_dia > r_dia + 0.05,
        details=f"front_dia={f_dia:.3f}, rear_dia={r_dia:.3f}",
    )

    # --- Mast is at the FRONT (+X) of the chassis. ---
    mast_aabb = ctx.part_world_aabb(mast)
    ch_aabb = ctx.part_world_aabb(chassis)
    ctx.check(
        "mast_at_front",
        mast_aabb[0][0] >= ch_aabb[1][0] - 0.25,
        details=f"mast xmin={mast_aabb[0][0]:.3f}, chassis xmax={ch_aabb[1][0]:.3f}",
    )
    # --- Mast rises ~1.85 m. ---
    mast_h = mast_aabb[1][2] - mast_aabb[0][2]
    ctx.check(
        "mast_tall_enough",
        mast_h >= 1.7,
        details=f"mast height={mast_h:.3f}",
    )
    # --- Mast is physically carried by the chassis (mount bracket contacts it). ---
    mount = chassis.get_visual("mast_mount_0")
    rail0 = mast.get_visual("mast_rail_0")
    ctx.expect_contact(
        chassis,
        mast,
        elem_a=mount,
        elem_b=rail0,
        contact_tol=1e-4,
        name="mast_mount_contacts_mast_rail",
    )

    # --- Overhead guard is above the seat (~2.1 m up). ---
    guard_post = chassis.get_visual("guard_post_0")
    guard_aabb = ctx.part_element_world_aabb(chassis, elem=guard_post)
    ctx.check(
        "guard_above_seat",
        guard_aabb[1][2] >= 1.9,
        details=f"guard top z={guard_aabb[1][2]:.3f}",
    )

    # --- Counterweight is at the rear (-X). ---
    cw = chassis.get_visual("counterweight")
    cw_aabb = ctx.part_element_world_aabb(chassis, elem=cw)
    cw_cx = (cw_aabb[0][0] + cw_aabb[1][0]) / 2.0
    ctx.check(
        "counterweight_at_rear",
        cw_cx < -0.5,
        details=f"counterweight center x={cw_cx:.3f}",
    )

    # --- Forks point +X (forward) and reach well past the mast. ---
    blade = carriage.get_visual("fork_blade_0")
    blade_aabb = ctx.part_element_world_aabb(carriage, elem=blade)
    ctx.check(
        "forks_point_forward",
        blade_aabb[1][0] > mast_aabb[1][0] + 0.5,
        details=f"fork blade xmax={blade_aabb[1][0]:.3f}, mast xmax={mast_aabb[1][0]:.3f}",
    )

    # --- Carriage stays captured on the mast: hooks overlap the rails in
    #     X and Z, and the carriage plate hugs the mast plane in X. ---
    carriage_plate = carriage.get_visual("carriage_plate")
    mast_rail_0 = mast.get_visual("mast_rail_0")
    hook_top_0 = carriage.get_visual("carriage_hook_top_0")
    ctx.expect_overlap(
        carriage,
        mast,
        axes="xz",
        elem_a=hook_top_0,
        elem_b=mast_rail_0,
        min_overlap=0.02,
        name="carriage_hook_wraps_mast_rail",
    )

    def carriage_plate_to_mast_gap() -> float:
        plate_aabb = ctx.part_element_world_aabb(carriage, elem=carriage_plate)
        m_aabb = ctx.part_world_aabb(mast)
        # Plate front face should sit just ahead of the mast front face.
        return plate_aabb[0][0] - m_aabb[1][0]

    # --- PRIMARY: carriage lift. Forks near ground when down, raised when up. ---
    with ctx.pose({lift: 0.0}):
        down_blade = ctx.part_element_world_aabb(carriage, elem=blade)
        down_z = down_blade[0][2]
        down_gap = carriage_plate_to_mast_gap()
    ctx.check(
        "forks_near_ground_when_lowered",
        down_z <= 0.18,
        details=f"lowered fork blade zmin={down_z:.3f}",
    )

    with ctx.pose({lift: LIFT_TRAVEL}):
        up_blade = ctx.part_element_world_aabb(carriage, elem=blade)
        up_z = up_blade[0][2]
        up_gap = carriage_plate_to_mast_gap()
        # Carriage stays vertically within the mast rails when fully raised.
        ctx.expect_within(
            carriage,
            mast,
            axes="z",
            elem_a=hook_top_0,
            elem_b=mast_rail_0,
            margin=0.02,
            name="raised_carriage_hook_within_mast_z",
        )
    ctx.check(
        "forks_raise_with_carriage",
        up_z > down_z + 0.8,
        details=f"lowered zmin={down_z:.3f}, raised zmin={up_z:.3f}",
    )
    # --- Carriage rides the mast in pure +Z: its X offset to the mast does
    #     not change between lowered and raised. ---
    ctx.check(
        "carriage_tracks_mast_in_x",
        abs(up_gap - down_gap) <= 0.02 and abs(down_gap) <= 0.08,
        details=f"down_gap={down_gap:.3f}, up_gap={up_gap:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
