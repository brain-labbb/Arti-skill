from __future__ import annotations

# Heavy-duty plastic tilt truck (dump cart), four-wheel variant.
#
# Real object: a large tapered polyethylene hopper (~1 cubic yard) that pivots
# forward on a steel axle to dump its load. This variant rides on FOUR wheels:
# two large fixed roll wheels on a rear axle plus two swivel-caster roll wheels
# under the front, so the cart is fully self-standing and steerable.
#
# Coordinate convention (Z-up world):
#   +Z up; floor at z = 0.
#   +X forward travel / dump direction.
#   +Y lateral (left); centerline y = 0.
#
# Root: steel BASE FRAME carries everything.
# All four wheels are emitted from a single for-loop over WHEEL_LAYOUT, using
# shared _wheel_mesh() / _tire_mesh() helpers and uniform CONTINUOUS roll joints.
# Front pair additionally gets REVOLUTE swivel (yaw about Z) via yoke parts.
# Hopper tilts forward (REVOLUTE about Y) on the rear axle line.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    section_loft,
    tube_from_spline_points,
)

# ---- key dimensions (meters) -------------------------------------------------
BODY_LEN = 1.62          # hopper fore-aft length at rim
BODY_WID = 0.92          # hopper width at rim
RIM_Z = 1.08             # top rim height (at rest)
BOTTOM_Z = 0.40          # hopper bottom above floor
WALL_T = 0.022           # plastic wall thickness

# All four wheels share the same geometry:
WHEEL_R = 0.15           # wheel radius
WHEEL_W = 0.065          # wheel width
WHEEL_Y = 0.34           # |y| of each wheel center (outside frame rails)

AXLE_X = 0.0             # rear axle x-position
FRONT_AXLE_X = 0.55      # front axle x-position
AXLE_Z = WHEEL_R         # axle/wheel center height = radius (wheels touch z=0)

# Steel frame sits above the wheels so front casters can swivel freely:
FRAME_CZ = 0.33          # frame member center z
FRAME_H = 0.04           # frame member height
FRAME_TOP = FRAME_CZ + FRAME_H / 2.0  # 0.35

# Swivel mount plate overlaps into the outrigger arm for connectivity:
MOUNT_PLATE_H = 0.030    # total mount plate height
MOUNT_PLATE_EMBED = 0.010  # plate bottom embeds below arm top
YOKE_MOUNT_Z = FRAME_TOP - MOUNT_PLATE_EMBED + MOUNT_PLATE_H  # 0.37
YOKE_DROP = YOKE_MOUNT_Z - AXLE_Z  # 0.22, yoke fork length

CASTER_OFFSET = 0.08     # front wheel trails forward of the swivel axis

# Uniform wheel layout: (index, axle_x, y_sign, has_swivel)
WHEEL_LAYOUT = [
    (0, AXLE_X,        -1, False),   # rear left
    (1, AXLE_X,         1, False),   # rear right
    (2, FRONT_AXLE_X,  -1, True),    # front left (swivel)
    (3, FRONT_AXLE_X,   1, True),    # front right (swivel)
]


# ---------------------------------------------------------------------------
# Hopper shell: tapered ribbed plastic tub (unchanged from parent design).
# ---------------------------------------------------------------------------
def _rrect_loop(hx: float, hy: float, r: float, z: float, n_corner: int = 5):
    """Rounded-rectangle loop in the XY plane at height z."""
    r = min(r, hx - 1e-4, hy - 1e-4)
    pts: list[tuple[float, float, float]] = []
    corners = [
        (hx - r, hy - r),
        (-(hx - r), hy - r),
        (-(hx - r), -(hy - r)),
        (hx - r, -(hy - r)),
    ]
    start_angles = [0.0, math.pi / 2.0, math.pi, 1.5 * math.pi]
    for (cx, cy), a0 in zip(corners, start_angles):
        for i in range(n_corner + 1):
            a = a0 + (math.pi / 2.0) * (i / n_corner)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a), z))
    return pts


def _hopper_section(scale_x: float, scale_y: float, front_shift: float, z: float):
    hx = (BODY_LEN / 2.0) * scale_x
    hy = (BODY_WID / 2.0) * scale_y
    r = 0.10 * min(scale_x, scale_y) + 0.02
    loop = _rrect_loop(hx, hy, r, z)
    return [(x + front_shift, y, zz) for (x, y, zz) in loop]


_OUTER_SECS = [
    (0.34, 0.46, 0.16, BOTTOM_Z),
    (0.58, 0.68, 0.11, BOTTOM_Z + 0.22 * (RIM_Z - BOTTOM_Z)),
    (0.80, 0.87, 0.06, BOTTOM_Z + 0.55 * (RIM_Z - BOTTOM_Z)),
    (0.96, 0.98, 0.015, BOTTOM_Z + 0.85 * (RIM_Z - BOTTOM_Z)),
    (1.00, 1.00, 0.0, RIM_Z),
]


def _hopper_shell_mesh() -> MeshGeometry:
    return section_loft([_hopper_section(*s) for s in _OUTER_SECS])


def _hopper_inner_mesh() -> MeshGeometry:
    inset = WALL_T
    bottom_z = BOTTOM_Z + WALL_T
    inner_secs = [
        (0.34, 0.46, 0.16, bottom_z),
        (0.58, 0.68, 0.11, bottom_z + 0.22 * (RIM_Z - bottom_z)),
        (0.80, 0.87, 0.06, bottom_z + 0.55 * (RIM_Z - bottom_z)),
        (0.96, 0.98, 0.015, bottom_z + 0.85 * (RIM_Z - bottom_z)),
        (1.00, 1.00, 0.0, RIM_Z - 0.018),
    ]

    def _inner_loop(sx, sy, fs, z):
        hx = (BODY_LEN / 2.0) * sx - inset
        hy = (BODY_WID / 2.0) * sy - inset
        r = 0.10 * min(sx, sy) + 0.02
        loop = _rrect_loop(hx, hy, r, z)
        return [(x + fs, y, zz) for (x, y, zz) in loop]

    return section_loft([_inner_loop(*s) for s in inner_secs])


def _rim_ring_mesh() -> MeshGeometry:
    hx = BODY_LEN / 2.0
    hy = BODY_WID / 2.0
    tube_r = 0.028
    loop = _rrect_loop(hx - tube_r * 0.4, hy - tube_r * 0.4, 0.12, RIM_Z, n_corner=10)
    return tube_from_spline_points(
        loop, radius=tube_r, samples_per_segment=2, radial_segments=14,
        closed_spline=True, cap_ends=False,
    )


def _hopper_ribs_mesh() -> MeshGeometry:
    geo = MeshGeometry()
    for (frac, sx, sy, fs) in (
        (0.30, 0.64, 0.73, 0.095),
        (0.50, 0.78, 0.85, 0.06),
        (0.68, 0.90, 0.94, 0.03),
    ):
        z = BOTTOM_Z + frac * (RIM_Z - BOTTOM_Z)
        loop = _hopper_section(sx, sy, fs, z)
        rib = tube_from_spline_points(
            loop, radius=0.012, samples_per_segment=1, radial_segments=8,
            closed_spline=True, cap_ends=False,
        )
        geo.merge(rib)
    return geo


def _hopper_saddle_mesh() -> MeshGeometry:
    """Steel trunnion saddle under the hopper: descends from the body bottom
    to wrap the rear axle line so the hopper pivots on the axle."""
    geo = MeshGeometry()
    plate_top_z = BOTTOM_Z + 0.03
    plate_bot_z = AXLE_Z - 0.02
    for sy in (1, -1):
        plate = BoxGeometry((0.20, 0.028, plate_top_z - plate_bot_z))
        plate.translate(AXLE_X, sy * 0.16, (plate_top_z + plate_bot_z) / 2.0)
        geo.merge(plate)
    sleeve = CylinderGeometry(0.036, 0.40, radial_segments=18)
    sleeve.rotate_x(math.pi / 2.0)
    sleeve.translate(AXLE_X, 0.0, AXLE_Z)
    geo.merge(sleeve)
    strap = BoxGeometry((0.22, 0.36, 0.03))
    strap.translate(AXLE_X, 0.0, BOTTOM_Z + 0.012)
    geo.merge(strap)
    return geo


# ---------------------------------------------------------------------------
# Steel base frame (root): carries the rear axle + wheels and front swivel
# mount plates. Two side rails span from rear to front axle positions.
# ---------------------------------------------------------------------------
def _base_frame_mesh() -> MeshGeometry:
    geo = MeshGeometry()
    rail_x_front = FRONT_AXLE_X + 0.05
    rail_x_rear = AXLE_X - 0.08
    rail_len = rail_x_front - rail_x_rear
    rail_cx = (rail_x_front + rail_x_rear) / 2.0

    # Two fore-aft side rails (inboard of the wheels)
    for sy in (1, -1):
        rail = BoxGeometry((rail_len, 0.05, FRAME_H))
        rail.translate(rail_cx, sy * 0.235, FRAME_CZ)
        geo.merge(rail)

    # Rear cross member at frame level (spans between rails for connectivity)
    rear_cross = BoxGeometry((0.07, 2 * 0.235, FRAME_H))
    rear_cross.translate(AXLE_X, 0.0, FRAME_CZ)
    geo.merge(rear_cross)

    # Front cross member at frame level (spans between rails for connectivity)
    front_cross = BoxGeometry((0.07, 2 * 0.235, FRAME_H))
    front_cross.translate(FRONT_AXLE_X, 0.0, FRAME_CZ)
    geo.merge(front_cross)

    # Steel axle tube (rear, below frame, spanning to wheels)
    axle = CylinderGeometry(0.022, 2 * WHEEL_Y + 0.02, radial_segments=18)
    axle.rotate_x(math.pi / 2.0)
    axle.translate(AXLE_X, 0.0, AXLE_Z)
    geo.merge(axle)

    # Bearing/upright blocks: overlap axle tube (bottom at AXLE_Z) and rails
    for sy in (1, -1):
        block_bot = AXLE_Z           # overlaps axle cylinder
        block_top = FRAME_CZ - FRAME_H / 2.0 + 0.005  # overlaps rail bottom
        block_h = block_top - block_bot
        if block_h > 0.01:
            block = BoxGeometry((0.06, 0.04, block_h))
            block.translate(AXLE_X, sy * 0.235, (block_top + block_bot) / 2.0)
            geo.merge(block)

    # Front outrigger arms: overlap front cross member and mount plates
    for sy in (1, -1):
        arm_len = WHEEL_Y - 0.10
        arm = BoxGeometry((0.06, arm_len, FRAME_H))
        arm.translate(FRONT_AXLE_X, sy * (0.10 + arm_len / 2.0), FRAME_CZ)
        geo.merge(arm)

    # Front swivel mount plates (embed into arm top for connectivity)
    plate_cz = FRAME_TOP - MOUNT_PLATE_EMBED + MOUNT_PLATE_H / 2.0
    for sy in (1, -1):
        plate = BoxGeometry((0.08, 0.08, MOUNT_PLATE_H))
        plate.translate(FRONT_AXLE_X, sy * WHEEL_Y, plate_cz)
        geo.merge(plate)

    return geo


# ---------------------------------------------------------------------------
# Shared wheel/tire geometry helpers (used for all four wheels).
# WheelGeometry spins about local X; rotate so spin axis becomes local Y.
# ---------------------------------------------------------------------------
def _wheel_mesh() -> MeshGeometry:
    return WheelGeometry(
        WHEEL_R - 0.040,
        WHEEL_W - 0.016,
        rim=WheelRim(inner_radius=0.072, flange_height=0.010, flange_thickness=0.004),
        hub=WheelHub(
            radius=0.030,
            width=WHEEL_W - 0.012,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=5, circle_diameter=0.044, hole_diameter=0.005),
        ),
        face=WheelFace(dish_depth=0.008, front_inset=0.003),
        spokes=WheelSpokes(style="split_y", count=5, thickness=0.005, window_radius=0.016),
        bore=WheelBore(style="round", diameter=0.018),
    )


def _tire_mesh() -> MeshGeometry:
    return TireGeometry(
        WHEEL_R,
        WHEEL_W,
        inner_radius=WHEEL_R - 0.045,
        tread=TireTread(style="block", depth=0.007, count=20, land_ratio=0.6),
        sidewall=TireSidewall(style="square", bulge=0.02),
    )


# ---------------------------------------------------------------------------
# Swivel yoke fork (for front caster wheels). Authored in yoke local frame
# with origin at the kingpin swivel axis. Wheel axle at (0, 0, -YOKE_DROP).
# ---------------------------------------------------------------------------
def _yoke_mesh() -> MeshGeometry:
    geo = MeshGeometry()
    # Kingpin boss (box for reliable mesh connectivity)
    boss = BoxGeometry((0.042, 0.042, 0.030))
    boss.translate(0.0, 0.0, -0.005)
    geo.merge(boss)
    # Narrow arm from kingpin toward fork
    arm = BoxGeometry((CASTER_OFFSET, 0.042, 0.020))
    arm.translate(CASTER_OFFSET / 2.0, 0.0, -0.020)
    geo.merge(arm)
    # Bridge plate at fork position (spans cheek-to-cheek width)
    bridge = BoxGeometry((0.042, WHEEL_W + 0.028, 0.020))
    bridge.translate(CASTER_OFFSET, 0.0, -0.024)
    geo.merge(bridge)
    # Fork cheeks at the wheel position
    cheek_top_z = -0.024
    cheek_bot_z = -YOKE_DROP + 0.005
    cheek_h = cheek_top_z - cheek_bot_z
    for sy in (1, -1):
        cheek = BoxGeometry((0.042, 0.012, cheek_h))
        cheek.translate(CASTER_OFFSET, sy * (WHEEL_W / 2.0 + 0.008),
                        (cheek_top_z + cheek_bot_z) / 2.0)
        geo.merge(cheek)
    # Axle pin through wheel center (box, extends past cheeks)
    pin = BoxGeometry((0.020, WHEEL_W + 0.040, 0.020))
    pin.translate(CASTER_OFFSET, 0.0, -YOKE_DROP)
    geo.merge(pin)
    return geo


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="plastic_tilt_truck")

    gray_plastic = model.material("gray_plastic", rgba=(0.62, 0.63, 0.64, 1.0))
    plastic_dark = model.material("plastic_dark", rgba=(0.50, 0.51, 0.52, 1.0))
    steel = model.material("steel_gray", rgba=(0.34, 0.35, 0.37, 1.0))
    wheel_gray = model.material("wheel_gray", rgba=(0.55, 0.56, 0.58, 1.0))
    rubber = model.material("rubber_dark", rgba=(0.18, 0.18, 0.19, 1.0))

    # ---- base frame (root) --------------------------------------------------
    frame = model.part("base_frame")
    frame.visual(mesh_from_geometry(_base_frame_mesh(), "frame"), material=steel, name="frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.70, 2 * 0.235, FRAME_H + 0.18)),
        mass=22.0,
        origin=Origin(xyz=(0.25, 0.0, (FRAME_CZ + AXLE_Z) / 2.0)),
    )

    # ---- all four wheels from a single for-loop ----------------------------
    for i, axle_x, sy, has_swivel in WHEEL_LAYOUT:
        # Create wheel part with shared geometry
        wheel = model.part(f"wheel_{i}")
        disc = _wheel_mesh()
        tire = _tire_mesh()
        disc.rotate_z(math.pi / 2.0)  # local X spin -> local Y
        tire.rotate_z(math.pi / 2.0)
        wheel.visual(
            mesh_from_geometry(disc, f"wheel_{i}_disc"),
            material=wheel_gray, name=f"wheel_{i}_disc",
        )
        wheel.visual(
            mesh_from_geometry(tire, f"wheel_{i}_tire"),
            material=rubber, name=f"wheel_{i}_tire",
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=WHEEL_R, length=WHEEL_W), mass=1.8,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        )

        if has_swivel:
            # Front wheel: swivel yoke intermediate part
            yoke = model.part(f"yoke_{i}")
            yoke.visual(
                mesh_from_geometry(_yoke_mesh(), f"yoke_{i}_body"),
                material=steel, name=f"yoke_{i}_body",
            )
            yoke.inertial = Inertial.from_geometry(
                Box((CASTER_OFFSET + 0.06, WHEEL_W + 0.04, YOKE_DROP)), mass=0.8,
                origin=Origin(xyz=(CASTER_OFFSET / 2.0, 0.0, -YOKE_DROP / 2.0)),
            )
            # Swivel joint: frame -> yoke (revolute about Z)
            model.articulation(
                f"frame_to_yoke_{i}",
                ArticulationType.REVOLUTE,
                parent=frame,
                child=yoke,
                origin=Origin(xyz=(axle_x, sy * WHEEL_Y, YOKE_MOUNT_Z)),
                axis=(0.0, 0.0, 1.0),
                motion_limits=MotionLimits(
                    effort=20.0, velocity=10.0,
                    lower=-math.pi, upper=math.pi,
                ),
            )
            # Roll joint: yoke -> wheel (continuous about Y)
            model.articulation(
                f"yoke_{i}_to_wheel_{i}",
                ArticulationType.CONTINUOUS,
                parent=yoke,
                child=wheel,
                origin=Origin(xyz=(CASTER_OFFSET, 0.0, -YOKE_DROP)),
                axis=(0.0, 1.0, 0.0),
                motion_limits=MotionLimits(effort=40.0, velocity=30.0),
            )
        else:
            # Rear wheel: direct roll joint from frame (continuous about Y)
            model.articulation(
                f"frame_to_wheel_{i}",
                ArticulationType.CONTINUOUS,
                parent=frame,
                child=wheel,
                origin=Origin(xyz=(axle_x, sy * WHEEL_Y, AXLE_Z)),
                axis=(0.0, 1.0, 0.0),
                motion_limits=MotionLimits(effort=60.0, velocity=30.0),
            )

    # ---- plastic hopper (tilts forward about the rear axle line) ------------
    pivot = (-AXLE_X, 0.0, AXLE_Z)

    def _seat(geo: MeshGeometry) -> MeshGeometry:
        return geo.translate(-pivot[0], -pivot[1], -pivot[2])

    hopper = model.part("hopper")
    hopper.visual(mesh_from_geometry(_seat(_hopper_shell_mesh()), "hopper_outer"),
                  material=gray_plastic, name="hopper_outer")
    hopper.visual(mesh_from_geometry(_seat(_hopper_inner_mesh()), "hopper_inner"),
                  material=plastic_dark, name="hopper_inner")
    hopper.visual(mesh_from_geometry(_seat(_rim_ring_mesh()), "hopper_rim"),
                  material=gray_plastic, name="hopper_rim")
    hopper.visual(mesh_from_geometry(_seat(_hopper_ribs_mesh()), "hopper_ribs"),
                  material=gray_plastic, name="hopper_ribs")
    hopper.visual(mesh_from_geometry(_seat(_hopper_saddle_mesh()), "hopper_saddle"),
                  material=steel, name="hopper_saddle")
    hopper.inertial = Inertial.from_geometry(
        Box((BODY_LEN, BODY_WID, RIM_Z - BOTTOM_Z)),
        mass=40.0,
        origin=Origin(xyz=(AXLE_X, 0.0, (RIM_Z + BOTTOM_Z) / 2.0 - AXLE_Z)),
    )
    model.articulation(
        "frame_to_hopper",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=hopper,
        origin=Origin(xyz=pivot),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=200.0, velocity=1.0, lower=0.0, upper=1.4),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("base_frame")
    hopper = object_model.get_part("hopper")
    dump = object_model.get_articulation("frame_to_hopper")

    # Gather all wheels and their joints from the layout
    wheels = {}
    roll_joints = {}
    yoke_parts = {}
    swivel_joints = {}
    for i, axle_x, sy, has_swivel in WHEEL_LAYOUT:
        wheels[i] = object_model.get_part(f"wheel_{i}")
        if has_swivel:
            yoke_parts[i] = object_model.get_part(f"yoke_{i}")
            swivel_joints[i] = object_model.get_articulation(f"frame_to_yoke_{i}")
            roll_joints[i] = object_model.get_articulation(f"yoke_{i}_to_wheel_{i}")
        else:
            roll_joints[i] = object_model.get_articulation(f"frame_to_wheel_{i}")

    # --- all four wheels are CONTINUOUS rollers about Y ---------------------
    for i in range(4):
        j = roll_joints[i]
        ctx.check(
            f"wheel_{i} has continuous roll joint about Y",
            j.joint_type == ArticulationType.CONTINUOUS
            and tuple(j.axis) == (0.0, 1.0, 0.0),
            details=f"type={j.joint_type}, axis={j.axis}",
        )

    # --- front pair have swivel joints (revolute about Z) -------------------
    for i in (2, 3):
        sj = swivel_joints[i]
        ctx.check(
            f"yoke_{i} swivels (revolute) about Z",
            sj.joint_type == ArticulationType.REVOLUTE
            and tuple(sj.axis) == (0.0, 0.0, 1.0),
            details=f"type={sj.joint_type}, axis={sj.axis}",
        )

    # --- hopper dump is revolute about Y ------------------------------------
    ctx.check(
        "hopper dump is revolute about Y",
        dump.joint_type == ArticulationType.REVOLUTE
        and tuple(dump.axis) == (0.0, 1.0, 0.0),
        details=f"type={dump.joint_type}, axis={dump.axis}",
    )

    # --- all four wheels touch the floor (z_min ~ 0) -----------------------
    for i in range(4):
        aabb = ctx.part_world_aabb(wheels[i])
        ctx.check(
            f"wheel_{i} touches floor (z~0)",
            aabb is not None and -0.01 <= aabb[0][2] <= 0.02,
            details=f"wheel_{i} z_min={aabb[0][2] if aabb else None}",
        )

    # --- frame above the floor -----------------------------------------------
    fr = ctx.part_world_aabb(frame)
    ctx.check(
        "frame bottom above floor (carried on wheels)",
        fr is not None and fr[0][2] > 0.05,
        details=f"frame z_min={fr[0][2] if fr else None}",
    )

    # --- hopper is the large tapered body ------------------------------------
    ha = ctx.part_world_aabb(hopper)
    he = _ext(ha)
    ctx.check(
        "hopper is the large tapered body (tall, wide, long)",
        he[2] > 0.55 and he[1] > 0.7 and he[0] > 1.2,
        details=f"hopper_ext={he}",
    )
    ctx.check(
        "hopper top rim near expected rim height",
        abs(ha[1][2] - RIM_Z) < 0.07,
        details=f"hopper_top_z={ha[1][2]}, expected~{RIM_Z}",
    )
    body_aabb = ctx.part_element_world_aabb(hopper, elem="hopper_outer")
    ctx.check(
        "hopper body sits above the frame",
        body_aabb is not None and body_aabb[0][2] > 0.25,
        details=f"hopper body z_min={body_aabb[0][2] if body_aabb else None}",
    )

    # --- all four wheels identical size (shared geometry) --------------------
    exts = [_ext(ctx.part_world_aabb(wheels[i])) for i in range(4)]
    for i in range(1, 4):
        ctx.check(
            f"wheel_{i} identical size to wheel_0 (shared geometry)",
            all(abs(exts[i][k] - exts[0][k]) < 0.004 for k in range(3)),
            details=f"wheel_0_ext={exts[0]}, wheel_{i}_ext={exts[i]}",
        )

    # --- wheel diameter matches WHEEL_R --------------------------------------
    ctx.check(
        "wheel diameter ~ 2*WHEEL_R",
        abs(exts[0][0] - 2 * WHEEL_R) < 0.05 and abs(exts[0][2] - 2 * WHEEL_R) < 0.05
        and exts[0][1] < exts[0][0],
        details=f"wheel_ext={exts[0]}, 2R={2*WHEEL_R}",
    )

    # --- mirror pairs: rear (0,1) and front (2,3) symmetric across y=0 ------
    for a, b in ((0, 1), (2, 3)):
        pa = ctx.part_world_position(wheels[a])
        pb = ctx.part_world_position(wheels[b])
        ctx.check(
            f"wheel_{a}/wheel_{b} mirror across centerline",
            pa is not None and pb is not None
            and abs(pa[1] + pb[1]) < 0.006
            and abs(pa[2] - pb[2]) < 0.006,
            details=f"pos_{a}={pa}, pos_{b}={pb}",
        )

    # --- rear pair at rear axle x, front pair at front axle x ---------------
    for i in (0, 1):
        p = ctx.part_world_position(wheels[i])
        ctx.check(
            f"wheel_{i} at rear axle x",
            p is not None and abs(p[0] - AXLE_X) < 0.02,
            details=f"pos={p}, expected_x={AXLE_X}",
        )
    for i in (2, 3):
        p = ctx.part_world_position(wheels[i])
        expected_x = FRONT_AXLE_X + CASTER_OFFSET
        ctx.check(
            f"wheel_{i} at front caster position (trailing swivel axis)",
            p is not None and abs(p[0] - expected_x) < 0.02,
            details=f"pos={p}, expected_x={expected_x}",
        )

    # --- wheel roll actually rotates the disc (spoke shift) -----------------
    spin0 = roll_joints[0]
    e0 = ctx.part_element_world_aabb(wheels[0], elem="wheel_0_disc")
    with ctx.pose({spin0: math.radians(36.0)}):
        e1 = ctx.part_element_world_aabb(wheels[0], elem="wheel_0_disc")
    ctx.check(
        "wheel_0 disc spins (spoke extent shifts under rotation about Y)",
        e0 is not None and e1 is not None
        and (abs(e1[0][0] - e0[0][0]) > 0.001 or abs(e1[0][2] - e0[0][2]) > 0.001
             or abs(e1[1][0] - e0[1][0]) > 0.001 or abs(e1[1][2] - e0[1][2]) > 0.001),
        details=f"rest={e0}, spun={e1}",
    )

    # --- front caster swivel actually yaws the wheel about Z ----------------
    sw2 = swivel_joints[2]
    c2_rest = ctx.part_world_position(wheels[2])
    with ctx.pose({sw2: math.pi / 2.0}):
        c2_yaw = ctx.part_world_position(wheels[2])
    ctx.check(
        "front wheel_2 yaws about its kingpin (swings in XY)",
        c2_rest is not None and c2_yaw is not None
        and (abs(c2_yaw[0] - c2_rest[0]) > 0.01 or abs(c2_yaw[1] - c2_rest[1]) > 0.01),
        details=f"rest={c2_rest}, yawed={c2_yaw}",
    )

    # --- dump tilt tips the hopper forward ----------------------------------
    rest_aabb = ctx.part_world_aabb(hopper)
    with ctx.pose({dump: 1.4}):
        tip_aabb = ctx.part_world_aabb(hopper)
    ctx.check(
        "dump tilt moves the hopper (forward tip)",
        abs(tip_aabb[1][0] - rest_aabb[1][0]) > 0.1
        or abs(tip_aabb[0][2] - rest_aabb[0][2]) > 0.05,
        details=f"rest_aabb={rest_aabb}, tipped_aabb={tip_aabb}",
    )

    # --- front wheels captured between yoke forks ---------------------------
    for i in (2, 3):
        ctx.allow_overlap(
            wheels[i], yoke_parts[i],
            reason=f"The wheel_{i} is captured between the yoke_{i} fork cheeks.",
        )
        ctx.expect_contact(
            wheels[i], yoke_parts[i],
            name=f"wheel_{i} held in yoke_{i} fork",
        )

    # --- yoke kingpin seats into frame mount plate --------------------------
    for i in (2, 3):
        ctx.allow_overlap(
            frame, yoke_parts[i],
            reason=f"The yoke_{i} kingpin boss seats into the frame swivel mount plate.",
        )
        ctx.expect_contact(
            frame, yoke_parts[i],
            name=f"yoke_{i} kingpin seated in frame mount",
        )

    # --- rear axle passes through rear wheel hubs --------------------------
    for i in (0, 1):
        ctx.allow_overlap(
            frame, wheels[i],
            reason=f"The frame axle passes through wheel_{i} hub bore.",
        )

    # --- hopper saddle wraps the frame axle ---------------------------------
    ctx.allow_overlap(
        hopper, frame,
        reason="The hopper trunnion saddle wraps the frame axle it pivots on.",
    )
    ctx.expect_contact(
        hopper, frame, name="hopper saddle carried on the frame axle",
    )

    return ctx.report()


object_model = build_object_model()
