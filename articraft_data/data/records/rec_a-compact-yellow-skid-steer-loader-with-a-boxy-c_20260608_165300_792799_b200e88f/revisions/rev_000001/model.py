from __future__ import annotations

# Compact yellow skid-steer loader (Bobcat style).
#
# Convention (real meters, Z-up):
#   +X = machine forward (toward the bucket), +Y = left, +Z = up.
#   Tires touch z=0; wheel axles run along Y.
#   Overall ~2.6 m long incl. bucket, ~1.5 m wide, ~2.0 m tall.
#
# Parts:
#   chassis   - boxy yellow body on four chunky tires (root), engine hood at the
#               rear, rear lift-arm towers, hydraulic barrels.
#   cab       - enclosed roll-cage operator cab with tinted front glass, a seat
#               hint, and a red beacon on the roof (fixed to chassis).
#   wheel_fl/fr/rl/rr - four wheel+tire assemblies with yellow rims.
#   lift_arms - one assembly: a stout arm down each side + a front cross-member,
#               pivoting up/down (REVOLUTE about Y) from the rear towers; carries
#               the lift-cylinder rods.
#   bucket    - wide flat-bottomed loader bucket with a cutting edge, pivoting
#               (REVOLUTE about Y) at the arm fronts to curl/dump.
#
# Primary articulations:
#   lift_arm_raise : revolute about Y at the rear towers (frac 0 down -> 1 raised)
#   bucket_tilt    : revolute about Y at the arm fronts (frac 0 dumped-down ->
#                    1 curled back); default/closed pose matches the reference
#                    (arms down + bucket tilted to the ground, digging).
#   wheel spin     : continuous about Y on each wheel.

from math import atan2, cos, pi, sin

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
    TireGeometry,
    TireGroove,
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
    rounded_rect_profile,
    superellipse_side_loft,
)

# --- key dimensions ------------------------------------------------------
TIRE_R = 0.30  # outer rolling radius (0.60 m diameter)
TIRE_W = 0.26  # tire width along the axle (Y)
AXLE_Z = TIRE_R  # axle height so tires touch z=0
FRONT_AXLE_X = 0.42
REAR_AXLE_X = -0.42
TRACK_Y = 0.62  # half-track: wheel center offset from machine centerline

REAR_TOWER_X = -0.58
PIVOT_Z = 0.92  # lift-arm pivot height at the rear towers
ARM_RUN = 1.46  # horizontal length of the side arms from pivot to bucket pin
ARM_SIDE_Y = 0.585  # arm centerline offset from machine centerline


def _wheel_geom(prefix: str):
    wheel = WheelGeometry(
        TIRE_R * 0.74,
        TIRE_W * 0.86,
        rim=WheelRim(inner_radius=TIRE_R * 0.50, flange_height=0.012, flange_thickness=0.006),
        hub=WheelHub(
            radius=0.072,
            width=0.060,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=6, circle_diameter=0.092, hole_diameter=0.011),
        ),
        face=WheelFace(dish_depth=0.020, front_inset=0.010),
        spokes=WheelSpokes(style="straight", count=6, thickness=0.018, window_radius=0.030),
        bore=WheelBore(style="round", diameter=0.040),
    )
    tire = TireGeometry(
        TIRE_R,
        TIRE_W,
        inner_radius=TIRE_R * 0.74,
        tread=TireTread(style="block", depth=0.020, count=22, land_ratio=0.55),
        grooves=(TireGroove(center_offset=0.0, width=0.010, depth=0.008),),
        sidewall=TireSidewall(style="square", bulge=0.02),
        shoulder=TireShoulder(width=0.014, radius=0.004),
    )
    # WheelGeometry/TireGeometry spin about local X; rotate so they spin about Y.
    wheel_mesh = mesh_from_geometry(wheel, f"{prefix}_rim")
    tire_mesh = mesh_from_geometry(tire, f"{prefix}_tire")
    return wheel_mesh, tire_mesh


def _build_wheel(model, name: str, *, axle_x: float, side: float, rim_mat, tire_mat):
    part = model.part(name)
    wheel_mesh, tire_mesh = _wheel_geom(name)
    # rotate_y(pi/2) turns the X-spin geometry into a Y-spin wheel.
    spin = Origin(rpy=(0.0, 0.0, pi / 2.0))
    part.visual(tire_mesh, origin=spin, material=tire_mat, name=f"{name}_tire")
    part.visual(wheel_mesh, origin=spin, material=rim_mat, name=f"{name}_rim")
    part.inertial = Inertial.from_geometry(
        Cylinder(radius=TIRE_R, length=TIRE_W),
        mass=28.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    return part, axle_x, side


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="skid_steer_loader")

    body_yellow = model.material("body_yellow", rgba=(0.95, 0.72, 0.08, 1.0))
    deep_yellow = model.material("deep_yellow", rgba=(0.86, 0.60, 0.05, 1.0))
    rim_orange = model.material("rim_orange", rgba=(0.95, 0.58, 0.10, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))
    cage_black = model.material("cage_black", rgba=(0.11, 0.11, 0.12, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.23, 0.25, 1.0))
    bucket_steel = model.material("bucket_steel", rgba=(0.16, 0.16, 0.18, 1.0))
    edge_steel = model.material("edge_steel", rgba=(0.30, 0.31, 0.33, 1.0))
    glass = model.material("glass", rgba=(0.10, 0.12, 0.16, 0.85))
    seat_vinyl = model.material("seat_vinyl", rgba=(0.13, 0.13, 0.14, 1.0))
    chrome = model.material("chrome", rgba=(0.62, 0.64, 0.67, 1.0))
    beacon_red = model.material("beacon_red", rgba=(0.85, 0.10, 0.10, 1.0))

    # =====================================================================
    # CHASSIS (root) - boxy yellow body that sits between the four tires.
    # =====================================================================
    chassis = model.part("chassis")
    chassis.inertial = Inertial.from_geometry(
        Box((1.05, 1.05, 0.55)),
        mass=1800.0,
        origin=Origin(xyz=(-0.05, 0.0, 0.45)),
    )

    # Lower body tub between the wheels (low slung).
    chassis.visual(
        Box((0.96, 0.88, 0.30)),
        origin=Origin(xyz=(-0.04, 0.0, 0.40)),
        material=body_yellow,
        name="lower_tub",
    )
    # Belly skid plate / frame underside.
    chassis.visual(
        Box((1.00, 0.80, 0.10)),
        origin=Origin(xyz=(-0.04, 0.0, 0.26)),
        material=dark_steel,
        name="belly_plate",
    )
    # Side panels flanking the tub (read as the body skin over the tracks).
    for sy, tag in ((1.0, "left"), (-1.0, "right")):
        chassis.visual(
            Box((0.86, 0.06, 0.34)),
            origin=Origin(xyz=(-0.04, sy * 0.44, 0.42)),
            material=deep_yellow,
            name=f"side_panel_{tag}",
        )

    # Rounded rear engine hood. Built as a side-loft along +Y (the loft axis):
    # each section is (y, z_min, z_max, width_x). This gives a rounded hood that
    # tapers toward the sides, sitting over the rear of the tub.
    hood = superellipse_side_loft(
        [
            (-0.30, 0.48, 0.74, 0.42),
            (-0.16, 0.48, 0.82, 0.48),
            (0.16, 0.48, 0.82, 0.48),
            (0.30, 0.48, 0.74, 0.42),
        ],
        exponents=2.6,
        segments=44,
    )
    # Shift the hood to the rear of the machine (centered near x=-0.64).
    hood.translate(-0.64, 0.0, 0.0)
    chassis.visual(mesh_from_geometry(hood, "engine_hood"), material=body_yellow, name="engine_hood")
    # Rear grille / louvers on the hood tail (set into the hood rear face).
    for i in range(4):
        chassis.visual(
            Box((0.06, 0.42, 0.040)),
            origin=Origin(xyz=(-0.80, 0.0, 0.58 + i * 0.055)),
            material=cage_black,
            name=f"rear_louver_{i}",
        )
    # Tail light pods.
    for sy, tag in ((1.0, "left"), (-1.0, "right")):
        chassis.visual(
            Box((0.05, 0.10, 0.10)),
            origin=Origin(xyz=(-0.82, sy * 0.28, 0.66)),
            material=rim_orange,
            name=f"tail_light_{tag}",
        )

    # Front frame plate (where the lift arms reach forward to and the chassis
    # nose sits between the front wheels).
    chassis.visual(
        Box((0.10, 0.74, 0.40)),
        origin=Origin(xyz=(0.44, 0.0, 0.42)),
        material=deep_yellow,
        name="front_frame",
    )
    # Headlights on the front frame.
    for sy, tag in ((1.0, "left"), (-1.0, "right")):
        chassis.visual(
            Cylinder(radius=0.045, length=0.04),
            origin=Origin(xyz=(0.50, sy * 0.26, 0.55), rpy=(0.0, pi / 2.0, 0.0)),
            material=chrome,
            name=f"headlight_{tag}",
        )

    # Axle stubs reaching out from the body to each wheel hub, so the wheels are
    # physically carried by the chassis (no floating wheels) and the side bodywork
    # reads as covering the drivetrain.
    for ax, sy, tag in (
        (FRONT_AXLE_X, 1.0, "fl"),
        (FRONT_AXLE_X, -1.0, "fr"),
        (REAR_AXLE_X, 1.0, "rl"),
        (REAR_AXLE_X, -1.0, "rr"),
    ):
        chassis.visual(
            Cylinder(radius=0.055, length=0.28),
            origin=Origin(xyz=(ax, sy * 0.50, AXLE_Z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"axle_stub_{tag}",
        )

    # Rear lift-arm towers: stout uprights that carry the lift pivot. Towers sit
    # inboard of the rear wheels (TOWER_Y < wheel inner face) so they clear the
    # tires; the side arms pivot outboard of them.
    tower_y = 0.40
    for sy, tag in ((1.0, "left"), (-1.0, "right")):
        chassis.visual(
            Box((0.16, 0.12, 0.74)),
            origin=Origin(xyz=(REAR_TOWER_X, sy * tower_y, 0.58)),
            material=deep_yellow,
            name=f"lift_tower_{tag}",
        )
        # pivot boss at the top of each tower
        chassis.visual(
            Cylinder(radius=0.06, length=0.12),
            origin=Origin(xyz=(REAR_TOWER_X, sy * tower_y, PIVOT_Z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"lift_pivot_boss_{tag}",
        )

    # Lift hydraulic cylinder BARRELS (chassis ends). Each runs from a low
    # mount near the rear wheel up toward the arm; the rod (on lift_arms)
    # telescopes inside.
    # Lift cylinders sit inboard of the rear wheels (BARREL_Y < wheel inner face)
    # so they do not collide with the tires; they sit behind the cab, up the
    # tower line.
    barrel_y = 0.40
    lift_barrel_lo = (-0.58, 0.0, 0.55)
    lift_barrel_hi = (-0.34, 0.0, 0.78)
    bx = (lift_barrel_lo[0] + lift_barrel_hi[0]) / 2.0
    bz = (lift_barrel_lo[2] + lift_barrel_hi[2]) / 2.0
    barrel_len = (
        (lift_barrel_hi[0] - lift_barrel_lo[0]) ** 2 + (lift_barrel_hi[2] - lift_barrel_lo[2]) ** 2
    ) ** 0.5
    # angle of the barrel in the XZ plane (about Y)
    barrel_ang = atan2(lift_barrel_hi[2] - lift_barrel_lo[2], lift_barrel_hi[0] - lift_barrel_lo[0])
    for sy, tag in ((1.0, "left"), (-1.0, "right")):
        chassis.visual(
            Cylinder(radius=0.045, length=barrel_len),
            origin=Origin(xyz=(bx, sy * barrel_y, bz), rpy=(0.0, pi / 2.0 - barrel_ang, 0.0)),
            material=dark_steel,
            name=f"lift_barrel_{tag}",
        )
        # lower clevis pin mount of the barrel onto the chassis
        chassis.visual(
            Cylinder(radius=0.035, length=0.10),
            origin=Origin(
                xyz=(lift_barrel_lo[0], sy * barrel_y, lift_barrel_lo[2]), rpy=(pi / 2.0, 0.0, 0.0)
            ),
            material=cage_black,
            name=f"lift_barrel_mount_{tag}",
        )

    # =====================================================================
    # WHEELS - four chunky tires with yellow/orange rims.
    # =====================================================================
    wheels = []
    for name, axle_x, sy in (
        ("wheel_fl", FRONT_AXLE_X, 1.0),
        ("wheel_fr", FRONT_AXLE_X, -1.0),
        ("wheel_rl", REAR_AXLE_X, 1.0),
        ("wheel_rr", REAR_AXLE_X, -1.0),
    ):
        part, ax, side = _build_wheel(
            model, name, axle_x=axle_x, side=sy, rim_mat=rim_orange, tire_mat=rubber
        )
        wheels.append((part, ax, side))

    # =====================================================================
    # CAB - enclosed roll-cage operator cab, mounted on the chassis.
    # =====================================================================
    cab = model.part("cab")
    cab.inertial = Inertial.from_geometry(
        Box((0.66, 0.78, 0.86)),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, 0.46)),
    )
    cab_h = 1.15  # cage height
    cab_w = 0.80  # cage width (Y)
    cab_d = 0.60  # cage depth (X)
    post_t = 0.045
    base_t = 0.05  # cab base/mount plate thickness; seats onto the chassis tub
    # Cab base/mount plate; bottom sits on the tub top, posts rise from it.
    cab.visual(
        Box((cab_d, cab_w, base_t)),
        origin=Origin(xyz=(0.0, 0.0, base_t / 2.0)),
        material=cage_black,
        name="cab_base_plate",
    )
    # Four corner posts of the roll cage (rise from the base plate top).
    post_z0 = base_t
    post_h = cab_h - post_z0
    for sx, sxt in ((1.0, "f"), (-1.0, "r")):
        for sy, syt in ((1.0, "l"), (-1.0, "r")):
            cab.visual(
                Box((post_t, post_t, post_h)),
                origin=Origin(xyz=(sx * (cab_d / 2.0 - post_t / 2.0),
                                   sy * (cab_w / 2.0 - post_t / 2.0),
                                   post_z0 + post_h / 2.0)),
                material=cage_black,
                name=f"cage_post_{sxt}{syt}",
            )
    # Top rails (front, rear, left, right) of the cage.
    cab.visual(
        Box((cab_d, post_t, post_t)),
        origin=Origin(xyz=(0.0, cab_w / 2.0 - post_t / 2.0, cab_h - post_t / 2.0)),
        material=cage_black,
        name="cage_rail_top_left",
    )
    cab.visual(
        Box((cab_d, post_t, post_t)),
        origin=Origin(xyz=(0.0, -(cab_w / 2.0 - post_t / 2.0), cab_h - post_t / 2.0)),
        material=cage_black,
        name="cage_rail_top_right",
    )
    cab.visual(
        Box((post_t, cab_w, post_t)),
        origin=Origin(xyz=(cab_d / 2.0 - post_t / 2.0, 0.0, cab_h - post_t / 2.0)),
        material=cage_black,
        name="cage_rail_top_front",
    )
    cab.visual(
        Box((post_t, cab_w, post_t)),
        origin=Origin(xyz=(-(cab_d / 2.0 - post_t / 2.0), 0.0, cab_h - post_t / 2.0)),
        material=cage_black,
        name="cage_rail_top_rear",
    )
    # Roof panel of the cage.
    cab.visual(
        Box((cab_d - 0.04, cab_w - 0.04, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, cab_h - post_t)),
        material=cage_black,
        name="cab_roof",
    )
    # Lower bulkhead rails to enclose the cab between the lift towers/arms.
    cab.visual(
        Box((cab_d, cab_w, post_t)),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=cage_black,
        name="cab_floor_rail",
    )
    cab.visual(
        Box((post_t, cab_w, 0.34)),
        origin=Origin(xyz=(-(cab_d / 2.0 - post_t / 2.0), 0.0, 0.32)),
        material=cage_black,
        name="cab_rear_panel",
    )
    # Dark tinted front glass; sized to meet the front posts and top rail so it
    # reads as glazing held in the cage frame.
    cab.visual(
        Box((0.025, cab_w - 2.0 * post_t + 0.01, cab_h - 0.30)),
        origin=Origin(xyz=(cab_d / 2.0 - post_t / 2.0, 0.0, cab_h - post_t - (cab_h - 0.30) / 2.0)),
        material=glass,
        name="front_glass",
    )
    # Side screen panels cover only the lower bay; the upper side bays stay open
    # so the roll cage reads as a cage rather than a solid box. A mid rail and a
    # diagonal-free upper opening keep the silhouette light.
    for sy, tag in ((1.0, "left"), (-1.0, "right")):
        cab.visual(
            Box((cab_d - 2.0 * post_t + 0.01, 0.010, 0.40)),
            origin=Origin(xyz=(0.0, sy * (cab_w / 2.0 - post_t / 2.0), 0.42)),
            material=cage_black,
            name=f"side_screen_{tag}",
        )
        # mid side rail across the open upper bay
        cab.visual(
            Box((cab_d - 2.0 * post_t + 0.01, post_t, post_t)),
            origin=Origin(xyz=(0.0, sy * (cab_w / 2.0 - post_t / 2.0), cab_h - 0.34)),
            material=cage_black,
            name=f"side_mid_rail_{tag}",
        )
    # Seat hint inside the cab, on a pedestal rising from the floor rail.
    cab.visual(
        Box((0.16, 0.22, 0.16)),
        origin=Origin(xyz=(-0.02, 0.0, 0.22)),
        material=cage_black,
        name="seat_pedestal",
    )
    cab.visual(
        Box((0.30, 0.34, 0.10)),
        origin=Origin(xyz=(-0.02, 0.0, 0.33)),
        material=seat_vinyl,
        name="seat_base",
    )
    cab.visual(
        Box((0.08, 0.34, 0.34)),
        origin=Origin(xyz=(-0.16, 0.0, 0.50)),
        material=seat_vinyl,
        name="seat_back",
    )
    # Red beacon on the cab roof: a small post + a red dome cap (seated on roof).
    cab.visual(
        Cylinder(radius=0.022, length=0.07),
        origin=Origin(xyz=(0.0, 0.28, cab_h - post_t + 0.02)),
        material=cage_black,
        name="beacon_post",
    )
    cab.visual(
        Cylinder(radius=0.045, length=0.08),
        origin=Origin(xyz=(0.0, 0.28, cab_h - post_t + 0.095)),
        material=beacon_red,
        name="beacon_lamp",
    )

    # =====================================================================
    # LIFT ARMS (one assembly) - side arms + front cross member.
    # Part frame at the rear pivot line (so the joint origin coincides with the
    # arm root). Geometry extends along +X toward the front bucket pin.
    # =====================================================================
    lift_arms = model.part("lift_arms")
    arm_z0 = 0.0  # local z of the arm centerline at the pivot
    # Side arms: long boxes from pivot (x=0) forward to the bucket pin.
    arm_len = ARM_RUN
    arm_cx = arm_len / 2.0
    for sy, tag in ((1.0, "left"), (-1.0, "right")):
        lift_arms.visual(
            Box((arm_len, 0.10, 0.14)),
            origin=Origin(xyz=(arm_cx, sy * ARM_SIDE_Y, arm_z0)),
            material=deep_yellow,
            name=f"side_arm_{tag}",
        )
        # gusset where the arm meets the rear pivot (bridges side arm to the
        # pivot tube; sits outboard of the tower so it does not collide with it)
        lift_arms.visual(
            Box((0.20, 0.09, 0.22)),
            origin=Origin(xyz=(0.09, sy * ARM_SIDE_Y, arm_z0 - 0.02)),
            material=deep_yellow,
            name=f"arm_root_gusset_{tag}",
        )
    # Front cross-member tying the two arms together just behind the bucket pin.
    lift_arms.visual(
        Box((0.12, 2.0 * ARM_SIDE_Y + 0.10, 0.12)),
        origin=Origin(xyz=(arm_len - 0.14, 0.0, arm_z0)),
        material=deep_yellow,
        name="front_cross_member",
    )
    # Rear pivot tube spanning both arms (reads as the lift cylinder line / axle).
    lift_arms.visual(
        Cylinder(radius=0.05, length=2.0 * ARM_SIDE_Y + 0.06),
        origin=Origin(xyz=(0.0, 0.0, arm_z0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="rear_pivot_tube",
    )
    # Lift cylinder RODS (arm ends) that slide into the chassis barrels. Each rod
    # reaches back-and-down from the arm toward the barrel; modeled as a chrome
    # rod + clevis. The barrel(chassis) and rod(arm) overlap intentionally where
    # they telescope. Rods are inboard (rod_y) to line up with the inboard
    # barrels and clear the rear wheels.
    rod_y = 0.40
    # Arm-side rod attach in the arm frame (q=0). The barrel upper end is at
    # world ~ (-0.34, +-0.40, 0.78); the arm pivot is at world (-0.58, 0, 0.92),
    # so that attach point maps to arm-local ~ (0.24, 0.40, -0.14). The rod
    # extends from there back-down into the barrel.
    rod_attach = (0.24, 0.0, -0.14)
    for sy, tag in ((1.0, "left"), (-1.0, "right")):
        lift_arms.visual(
            Cylinder(radius=0.028, length=0.30),
            origin=Origin(xyz=(rod_attach[0] - 0.10, sy * rod_y, rod_attach[2] - 0.05),
                          rpy=(0.0, pi / 2.0 - barrel_ang, 0.0)),
            material=chrome,
            name=f"lift_rod_{tag}",
        )
        # rod-to-arm clevis pin
        lift_arms.visual(
            Cylinder(radius=0.030, length=0.10),
            origin=Origin(xyz=(rod_attach[0], sy * rod_y, rod_attach[2]), rpy=(pi / 2.0, 0.0, 0.0)),
            material=cage_black,
            name=f"lift_rod_clevis_{tag}",
        )
        # bracket tying the rod clevis up-and-out to the side arm so the rod is
        # carried by the arm (no floating cylinder).
        lift_arms.visual(
            Box((0.06, abs(ARM_SIDE_Y - rod_y) + 0.06, 0.16)),
            origin=Origin(
                xyz=(rod_attach[0], sy * (rod_y + ARM_SIDE_Y) / 2.0, (rod_attach[2] + arm_z0) / 2.0)
            ),
            material=deep_yellow,
            name=f"lift_rod_bracket_{tag}",
        )

    # =====================================================================
    # BUCKET - wide curved loader bucket with a cutting edge.
    # Part frame at the bucket pivot pin (top-rear lip of the bucket). Bucket
    # geometry hangs forward+down from the pin so positive tilt curls it back up.
    # =====================================================================
    bucket = model.part("bucket")
    bucket_w = 1.40
    # Curved back-plate of the bucket built as a lathe-like profile using a
    # side-loft along Y (constant cross section) would be flat; instead build
    # the curved scoop from a swept set of plates. Use an extruded curved
    # profile in the XZ plane, extruded along Y.
    # Profile (local, pin at origin): back wall down, curved floor forward.
    # We approximate the curl with a back panel, a curved bottom, and a front lip.
    back_h = 0.50
    floor_run = 0.52
    # back wall (vertical-ish, just below/forward of the pin)
    bucket.visual(
        Box((0.04, bucket_w, back_h)),
        origin=Origin(xyz=(0.02, 0.0, -back_h / 2.0)),
        material=bucket_steel,
        name="bucket_back",
    )
    # curved floor: a series of stepped/rotated plates forming the scoop curl.
    n_floor = 5
    px, pz, ang = 0.04, -back_h, 0.0
    seg = floor_run / n_floor
    for i in range(n_floor):
        ang -= 0.18  # progressively tilt the floor plates up toward the lip
        cx = px + cos(ang) * seg / 2.0
        cz = pz + sin(ang) * seg / 2.0
        bucket.visual(
            Box((seg, bucket_w, 0.05)),
            origin=Origin(xyz=(cx, 0.0, cz), rpy=(0.0, -ang, 0.0)),
            material=bucket_steel,
            name=f"bucket_floor_{i}",
        )
        px += cos(ang) * seg
        pz += sin(ang) * seg
    # Cutting edge along the front bottom lip.
    bucket.visual(
        Box((0.06, bucket_w, 0.04)),
        origin=Origin(xyz=(px + 0.02, 0.0, pz), rpy=(0.0, -ang, 0.0)),
        material=edge_steel,
        name="cutting_edge",
    )
    # Side plates closing the bucket ends.
    for sy, tag in ((1.0, "left"), (-1.0, "right")):
        bucket.visual(
            Box((floor_run + 0.10, 0.03, back_h + 0.10)),
            origin=Origin(xyz=(0.24, sy * (bucket_w / 2.0 - 0.015), -back_h / 2.0 - 0.02)),
            material=bucket_steel,
            name=f"bucket_side_{tag}",
        )
    # Top reinforcement rail (forward of the pivot pin so it clears the arm
    # cross-member; the narrow lugs make the actual pinned connection).
    bucket.visual(
        Box((0.10, bucket_w, 0.06)),
        origin=Origin(xyz=(0.06, 0.0, 0.0)),
        material=bucket_steel,
        name="bucket_top_rail",
    )
    # Pivot lugs reach rearward from the bucket back to the arm fronts so the
    # bucket is physically carried by (pinned to) the lift arms.
    for sy, tag in ((1.0, "left"), (-1.0, "right")):
        bucket.visual(
            Box((0.24, 0.06, 0.12)),
            origin=Origin(xyz=(-0.10, sy * ARM_SIDE_Y, 0.02)),
            material=dark_steel,
            name=f"bucket_lug_{tag}",
        )

    # Tilt cylinder: barrel on the lift arm, rod to the bucket. (Both halves are
    # authored on lift_arms / bucket so they move with the right link.)
    for sy, tag in ((1.0, "left"), (-1.0, "right")):
        # barrel on the arm, near the front, angled down to the bucket top
        lift_arms.visual(
            Cylinder(radius=0.034, length=0.34),
            origin=Origin(xyz=(arm_len - 0.34, sy * ARM_SIDE_Y, arm_z0 + 0.02),
                          rpy=(0.0, pi / 2.0 - 0.55, 0.0)),
            material=dark_steel,
            name=f"tilt_barrel_{tag}",
        )

    # =====================================================================
    # ARTICULATIONS
    # =====================================================================
    # Wheel spin (continuous about Y).
    for part, ax, sy in wheels:
        model.articulation(
            f"{part.name}_spin",
            ArticulationType.CONTINUOUS,
            parent="chassis",
            child=part,
            origin=Origin(xyz=(ax, sy * TRACK_Y, AXLE_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=400.0, velocity=12.0),
        )

    # Cab fixed to the chassis (mounted above the tub).
    model.articulation(
        "cab_mount",
        ArticulationType.FIXED,
        parent="chassis",
        child="cab",
        origin=Origin(xyz=(0.06, 0.0, 0.55)),
    )

    # Lift-arm raise: pivot at the rear towers. The arm extends along +X, so
    # axis (0,-1,0) makes positive q lift the front (and bucket) upward.
    model.articulation(
        "lift_arm_raise",
        ArticulationType.REVOLUTE,
        parent="chassis",
        child="lift_arms",
        origin=Origin(xyz=(REAR_TOWER_X, 0.0, PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4000.0, velocity=0.8, lower=0.0, upper=1.05),
    )

    # Bucket tilt: pivot at the front cross-member / bucket lugs. The bucket
    # back extends along local -Z (and floor along +X) from the pin. Positive q
    # about +Y curls the cutting edge upward (dump<->curl). frac=0 leaves the
    # bucket tilted down to the ground (digging, matching the reference).
    model.articulation(
        "bucket_tilt",
        ArticulationType.REVOLUTE,
        parent="lift_arms",
        child="bucket",
        origin=Origin(xyz=(arm_len + 0.10, 0.0, arm_z0 + 0.02)),
        # Bucket floor extends along +X and the back wall down -Z from the pin.
        # axis (0,-1,0) makes positive q curl the front cutting edge upward
        # (dump -> curl). frac=0 leaves the bucket tilted down (digging pose).
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3000.0, velocity=1.0, lower=0.0, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    chassis = object_model.get_part("chassis")
    cab = object_model.get_part("cab")
    lift_arms = object_model.get_part("lift_arms")
    bucket = object_model.get_part("bucket")
    w_fl = object_model.get_part("wheel_fl")
    w_fr = object_model.get_part("wheel_fr")
    w_rl = object_model.get_part("wheel_rl")
    w_rr = object_model.get_part("wheel_rr")

    lift = object_model.get_articulation("lift_arm_raise")
    tilt = object_model.get_articulation("bucket_tilt")

    # --- ground contact: all four tires touch z ~= 0 -------------------
    for w in (w_fl, w_fr, w_rl, w_rr):
        aabb = ctx.part_world_aabb(w)
        zmin = aabb[0][2]
        ctx.check(
            f"{w.name}_touches_ground",
            -0.02 <= zmin <= 0.02,
            f"{w.name} zmin={zmin:.3f} (expected ~0)",
        )

    # --- overall scale sanity -----------------------------------------
    with ctx.pose({lift: 0.0, tilt: 0.0}):
        full = ctx.part_world_aabb(chassis)
    # full machine bbox at default pose
    allp = [chassis, cab, lift_arms, bucket, w_fl, w_fr, w_rl, w_rr]
    with ctx.pose({lift: 0.0, tilt: 0.0}):
        mins = [1e9, 1e9, 1e9]
        maxs = [-1e9, -1e9, -1e9]
        for p in allp:
            aabb = ctx.part_world_aabb(p)
            for i in range(3):
                mins[i] = min(mins[i], aabb[0][i])
                maxs[i] = max(maxs[i], aabb[1][i])
    length = maxs[0] - mins[0]
    width = maxs[1] - mins[1]
    height = maxs[2] - mins[2]
    ctx.check("machine_length_reasonable", 2.2 <= length <= 3.2, f"length={length:.3f}")
    ctx.check("machine_width_reasonable", 1.3 <= width <= 1.7, f"width={width:.3f}")
    ctx.check("machine_height_reasonable", 1.7 <= height <= 2.3, f"height={height:.3f}")
    ctx.check("machine_sits_on_ground", -0.02 <= mins[2] <= 0.02, f"zmin={mins[2]:.3f}")

    # --- cab sits between the arms, above the chassis ------------------
    cab_aabb = ctx.part_world_aabb(cab)
    ctx.check(
        "cab_above_chassis",
        cab_aabb[0][2] > 0.45,
        f"cab zmin={cab_aabb[0][2]:.3f}",
    )
    ctx.check(
        "cab_within_track",
        cab_aabb[0][1] > -TRACK_Y and cab_aabb[1][1] < TRACK_Y + 0.05,
        f"cab y-extent=({cab_aabb[0][1]:.3f},{cab_aabb[1][1]:.3f})",
    )

    # --- lift arms raise the bucket -----------------------------------
    with ctx.pose({lift: 0.0, tilt: 0.0}):
        bucket_down = ctx.part_world_position(bucket)
        bucket_down_aabb = ctx.part_world_aabb(bucket)
    with ctx.pose({lift: 1.05, tilt: 0.0}):
        bucket_up = ctx.part_world_position(bucket)
        bucket_up_aabb = ctx.part_world_aabb(bucket)
    ctx.check(
        "lift_raises_bucket",
        bucket_up[2] > bucket_down[2] + 0.5,
        f"down_z={bucket_down[2]:.3f} up_z={bucket_up[2]:.3f}",
    )
    # bucket rests low (near ground) in the default digging pose
    ctx.check(
        "bucket_starts_low",
        bucket_down_aabb[0][2] < 0.35,
        f"bucket zmin (down) = {bucket_down_aabb[0][2]:.3f}",
    )

    # --- bucket tilts (curls) at the arm front -------------------------
    with ctx.pose({lift: 0.0, tilt: 0.0}):
        edge_lo = ctx.part_element_world_aabb(bucket, elem="cutting_edge")
    with ctx.pose({lift: 0.0, tilt: 1.15}):
        edge_hi = ctx.part_element_world_aabb(bucket, elem="cutting_edge")
    ctx.check(
        "bucket_tilt_lifts_edge",
        edge_hi[0][2] > edge_lo[0][2] + 0.10,
        f"edge zmin: dump={edge_lo[0][2]:.3f} curl={edge_hi[0][2]:.3f}",
    )

    # --- bucket is at the FRONT (forward of the chassis) ---------------
    ctx.check(
        "bucket_in_front",
        bucket_down_aabb[1][0] > 0.9,
        f"bucket xmax={bucket_down_aabb[1][0]:.3f}",
    )
    # --- bucket is wide -----------------------------------------------
    ctx.check(
        "bucket_is_wide",
        (bucket_down_aabb[1][1] - bucket_down_aabb[0][1]) > 1.2,
        f"bucket width={bucket_down_aabb[1][1] - bucket_down_aabb[0][1]:.3f}",
    )

    # --- beacon present on the cab roof --------------------------------
    beacon = ctx.part_element_world_aabb(cab, elem="beacon_lamp")
    ctx.check(
        "beacon_on_roof",
        beacon[0][2] > 1.3,
        f"beacon zmin={beacon[0][2]:.3f}",
    )

    # --- lift arms pivot from the REAR ---------------------------------
    arms_aabb = ctx.part_world_aabb(lift_arms)
    ctx.check(
        "arms_run_front_to_rear",
        arms_aabb[0][0] < -0.4 and arms_aabb[1][0] > 0.8,
        f"arms x-extent=({arms_aabb[0][0]:.3f},{arms_aabb[1][0]:.3f})",
    )

    # The lift rod + its clevis/bracket (on the arm) telescope into the barrel.
    for tag in ("left", "right"):
        for arm_elem in (
            f"lift_rod_{tag}",
            f"lift_rod_clevis_{tag}",
            f"lift_rod_bracket_{tag}",
        ):
            ctx.allow_overlap(
                chassis,
                lift_arms,
                elem_a=f"lift_barrel_{tag}",
                elem_b=arm_elem,
                reason=f"{tag.title()} lift cylinder rod assembly telescopes into the chassis barrel.",
            )
    # The lift-arm rear pivot tube is captured by the tower pivot bosses: this is
    # the physical revolute pin nested in its bearing.
    ctx.allow_overlap(
        chassis,
        lift_arms,
        elem_a="lift_pivot_boss_left",
        elem_b="rear_pivot_tube",
        reason="Lift-arm pivot tube is captured by the left tower pivot boss (revolute pin in bearing).",
    )
    ctx.allow_overlap(
        chassis,
        lift_arms,
        elem_a="lift_pivot_boss_right",
        elem_b="rear_pivot_tube",
        reason="Lift-arm pivot tube is captured by the right tower pivot boss (revolute pin in bearing).",
    )
    # The lift-arm rear pivot tube passes through the tower tops at the pivot,
    # and the lift rod runs up the tower line into the barrel.
    ctx.allow_overlap(
        chassis,
        lift_arms,
        elem_a="lift_tower_left",
        elem_b="rear_pivot_tube",
        reason="Lift-arm pivot tube passes through the left tower at the revolute pivot.",
    )
    ctx.allow_overlap(
        chassis,
        lift_arms,
        elem_a="lift_tower_right",
        elem_b="rear_pivot_tube",
        reason="Lift-arm pivot tube passes through the right tower at the revolute pivot.",
    )
    ctx.allow_overlap(
        chassis,
        lift_arms,
        elem_a="lift_tower_left",
        elem_b="lift_rod_left",
        reason="Left lift rod runs up the tower line toward its barrel at the pivot.",
    )
    ctx.allow_overlap(
        chassis,
        lift_arms,
        elem_a="lift_tower_right",
        elem_b="lift_rod_right",
        reason="Right lift rod runs up the tower line toward its barrel at the pivot.",
    )
    # Bucket pivot lugs pin onto the lift-arm fronts (side arms + cross member).
    for le in ("side_arm_left", "front_cross_member"):
        ctx.allow_overlap(
            bucket,
            lift_arms,
            elem_a="bucket_lug_left",
            elem_b=le,
            reason="Left bucket pivot lug pins onto the lift-arm front (revolute pin contact).",
        )
    for le in ("side_arm_right", "front_cross_member"):
        ctx.allow_overlap(
            bucket,
            lift_arms,
            elem_a="bucket_lug_right",
            elem_b=le,
            reason="Right bucket pivot lug pins onto the lift-arm front (revolute pin contact).",
        )
    # The wheel hubs mount on the chassis axle stubs (stub nests in the rim hub).
    for w, tag in ((w_fl, "fl"), (w_fr, "fr"), (w_rl, "rl"), (w_rr, "rr")):
        ctx.allow_overlap(
            chassis,
            w,
            elem_a=f"axle_stub_{tag}",
            elem_b=f"{w.name}_rim",
            reason=f"Wheel {tag} hub mounts on its chassis axle stub.",
        )

    return ctx.report()


object_model = build_object_model()
