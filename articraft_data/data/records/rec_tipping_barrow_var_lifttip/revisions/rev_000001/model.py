from __future__ import annotations

# Heavy-duty plastic tilt truck (dump cart) – LIFT-AND-TIP variant.
#
# Real object: a large tapered polyethylene hopper (~1 cubic yard) carried on
# a hinged lift linkage so that as it dumps it both rises and rotates forward.
# A rocker-style lift arm raises the tub off its rest cradle and tips it over
# the front lip, clearing the frame for a higher, cleaner discharge.
#
# Coordinate convention (Z-up world):
#   - +Z is up; the floor is z = 0 (all wheels/casters touch z ~ 0).
#   - +X is the FORWARD travel / dump direction.
#   - +Y is the lateral axle direction; centerline is y = 0.
#
# Root structure: the steel BASE FRAME (chassis) carries everything.
#
# Kinematic chain (lift-and-tip):
#   frame → lift_arm (REVOLUTE about -Y, raises the arm)
#   lift_arm → hopper (REVOLUTE about +Y, tips the tub forward)
#
# Other articulations (unchanged from parent):
#   - frame_to_wheel_l / frame_to_wheel_r : CONTINUOUS big-wheel roll
#   - frame_to_caster_yoke_0/1            : REVOLUTE swivel-caster yaw
#   - caster_yoke_*_to_wheel              : CONTINUOUS small-caster roll

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
BODY_LEN = 1.62          # overall fore-aft length of the hopper at the rim
BODY_WID = 0.92          # overall width of the hopper at the rim
RIM_Z = 1.04             # top rim height (at rest, before tilting)
BOTTOM_Z = 0.30          # height of the hopper bottom above the floor
WALL_T = 0.022           # plastic wall thickness

WHEEL_R = 0.205          # big rear wheel radius
WHEEL_W = 0.075          # big rear wheel width
WHEEL_Y = 0.345          # |y| of each big wheel center (just outside the body)
AXLE_X = 0.0             # axle sits under the body center-rear
AXLE_Z = WHEEL_R         # axle center height = wheel radius

CASTER_WHEEL_R = 0.055   # small front swivel-caster wheel radius
CASTER_WHEEL_W = 0.042
CASTER_X = 0.64          # forward position of the front caster king-pins
CASTER_Y = 0.27          # |y| of each front caster
CASTER_OFFSET = 0.105    # trailing offset of caster wheel ahead of its swivel axis

FRAME_Z = 0.055          # steel sub-frame member height

# ---- lift-and-tip mechanism dimensions ---------------------------------------
LIFT_PX = -0.30          # lift arm pivot X (rear of frame, behind axle)
LIFT_PZ = 0.250          # lift arm pivot Z (arm center height at rest)
LIFT_ARM_Y = 0.235       # |y| of each lift arm (aligned with frame rails)
ARM_LENGTH = 0.70        # lift arm length (pivot to past tub pivot)
ARM_WIDTH = 0.045        # lift arm cross-section width (Y)
ARM_HEIGHT = 0.040       # lift arm cross-section height (Z)

TUB_TIP_DX = 0.65        # tub tip pivot X offset in lift arm frame
TUB_TIP_DZ = ARM_HEIGHT / 2.0  # tub tip pivot Z in lift arm frame (arm top)

# World position of the tub tip pivot at rest:
_TUB_TIP_WORLD = (LIFT_PX + TUB_TIP_DX, 0.0, LIFT_PZ + TUB_TIP_DZ)
# = (0.35, 0.0, 0.270)


# ---------------------------------------------------------------------------
# Hopper shell: a tapered, ribbed plastic tub (same as parent).
# ---------------------------------------------------------------------------
def _rrect_loop(hx: float, hy: float, r: float, z: float, n_corner: int = 5):
    """Rounded-rectangle loop in the XY plane at height z (counter-clockwise)."""
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


# ---------------------------------------------------------------------------
# Hopper cradle pads: moulded pads under the hopper bottom that sit on the
# lift arms. Four pads total (two per side, at two X positions along each arm).
# Also includes a pivot bracket at the front-bottom where the tub tips.
# Authored in world coordinates (same frame as the hopper shell).
# ---------------------------------------------------------------------------
def _hopper_cradle_mesh() -> MeshGeometry:
    geo = MeshGeometry()
    # The lift arm top is at LIFT_PZ + ARM_HEIGHT/2 = arm_top_z.
    # The hopper bottom is at BOTTOM_Z. Cradle pads bridge this gap.
    arm_top_z = LIFT_PZ + ARM_HEIGHT / 2.0
    pad_top = BOTTOM_Z + 0.005        # slightly into the hopper body
    pad_bot = arm_top_z - 0.002       # slightly below arm top for contact
    pad_h = pad_top - pad_bot
    pad_cz = (pad_top + pad_bot) / 2.0
    pad_len = 0.10
    pad_w = ARM_WIDTH + 0.012
    # Pad X positions in world coords (under the hopper body, along the arms)
    pad_xs = (0.0, 0.20)
    for sy in (1, -1):
        for px in pad_xs:
            pad = BoxGeometry((pad_len, pad_w, pad_h))
            pad.translate(px, sy * LIFT_ARM_Y, pad_cz)
            geo.merge(pad)
    # Pivot bracket: a reinforced lug at the front-bottom of the hopper where
    # the tub tips on the lift arm pin. Two side plates + a transverse sleeve.
    bracket_x = _TUB_TIP_WORLD[0]
    bracket_z = _TUB_TIP_WORLD[2]
    bracket_top = BOTTOM_Z + 0.005    # connects into hopper body
    bracket_bot = bracket_z - 0.018   # wraps below the pivot pin
    bracket_h = bracket_top - bracket_bot
    bracket_cz = (bracket_top + bracket_bot) / 2.0
    for sy in (1, -1):
        plate = BoxGeometry((0.10, 0.028, bracket_h))
        plate.translate(bracket_x, sy * 0.10, bracket_cz)
        geo.merge(plate)
    # Transverse sleeve wrapping the lift arm tub-end pin
    sleeve = CylinderGeometry(0.024, 2 * 0.10 + 0.028, radial_segments=16)
    sleeve.rotate_x(math.pi / 2.0)  # Z -> Y
    sleeve.translate(bracket_x, 0.0, bracket_z)
    geo.merge(sleeve)
    # Connecting strap tying the bracket to the hopper body bottom
    strap = BoxGeometry((0.12, 0.22, 0.018))
    strap.translate(bracket_x, 0.0, BOTTOM_Z + 0.005)
    geo.merge(strap)
    return geo


# ---------------------------------------------------------------------------
# Steel base frame (root): carries the axle, casters, and lift-arm pivot.
# ---------------------------------------------------------------------------
def _base_frame_mesh() -> MeshGeometry:
    geo = MeshGeometry()
    rail_x0 = CASTER_X + 0.04
    rail_x1 = LIFT_PX - 0.05  # extend rails behind the lift pivot
    rail_len = rail_x0 - rail_x1
    rail_cx = (rail_x0 + rail_x1) / 2.0
    # Two fore-aft side rails
    for sy in (1, -1):
        rail = BoxGeometry((rail_len, 0.05, FRAME_Z))
        rail.translate(rail_cx, sy * 0.235, FRAME_Z / 2.0 + 0.012)
        geo.merge(rail)
    # Rear cross member at the axle
    rear_cross = BoxGeometry((0.09, 0.24, 0.05))
    rear_cross.translate(AXLE_X, 0.0, AXLE_Z)
    geo.merge(rear_cross)
    # Steel axle tube spanning between the two big wheels
    axle = CylinderGeometry(0.022, 2 * WHEEL_Y + 0.02, radial_segments=18)
    axle.rotate_x(math.pi / 2.0)  # Z -> Y
    axle.translate(AXLE_X, 0.0, AXLE_Z)
    geo.merge(axle)
    # Bearing/upright blocks tying the axle to the rails
    for sy in (1, -1):
        block = BoxGeometry((0.06, 0.04, AXLE_Z - 0.012))
        block.translate(AXLE_X, sy * 0.235, (AXLE_Z + 0.012) / 2.0)
        geo.merge(block)
    # Front caster swivel mounting plates
    for sy in (1, -1):
        plate = BoxGeometry((0.09, 0.09, 0.018))
        plate.translate(CASTER_X, sy * CASTER_Y, 0.012 + 0.009)
        geo.merge(plate)

    # ---- lift-arm pivot brackets -------------------------------------------
    # Upright brackets rising from the rails to the lift pivot height
    bracket_bot = FRAME_Z + 0.012
    bracket_h = LIFT_PZ - bracket_bot
    for sy in (1, -1):
        upright = BoxGeometry((0.06, 0.05, bracket_h))
        upright.translate(LIFT_PX, sy * LIFT_ARM_Y, bracket_bot + bracket_h / 2.0)
        geo.merge(upright)
    # Cross member tying the two uprights together at the pivot height
    lift_cross = BoxGeometry((0.06, 2 * LIFT_ARM_Y - 0.05, 0.04))
    lift_cross.translate(LIFT_PX, 0.0, LIFT_PZ)
    geo.merge(lift_cross)
    # Pivot pin spanning between the uprights (the lift arm wraps this)
    pin = CylinderGeometry(0.016, 2 * LIFT_ARM_Y + 0.04, radial_segments=14)
    pin.rotate_x(math.pi / 2.0)  # Z -> Y
    pin.translate(LIFT_PX, 0.0, LIFT_PZ)
    geo.merge(pin)
    # Lower cross brace at the lift pivot base
    base_cross = BoxGeometry((0.06, 2 * LIFT_ARM_Y - 0.05, 0.04))
    base_cross.translate(LIFT_PX, 0.0, bracket_bot + 0.02)
    geo.merge(base_cross)
    return geo


# ---------------------------------------------------------------------------
# Lift arm: a U-shaped steel lift frame with two side arms connected by
# crossbars. Pivots on the frame at the rear, carries the tub at the front.
# Authored in the lift arm local frame (origin at the pivot).
# ---------------------------------------------------------------------------
def _one_lift_arm_mesh(sy: int) -> MeshGeometry:
    """One side arm of the lift frame, authored in the lift arm local frame."""
    geo = MeshGeometry()
    # Main arm bar: extends from pivot (X=0) forward to ARM_LENGTH
    arm = BoxGeometry((ARM_LENGTH, ARM_WIDTH, ARM_HEIGHT))
    arm.translate(ARM_LENGTH / 2.0, sy * LIFT_ARM_Y, 0.0)
    geo.merge(arm)
    # Pivot eye (sleeve wrapping the frame pivot pin) at the rear
    eye = CylinderGeometry(0.026, 0.05, radial_segments=14)
    eye.rotate_x(math.pi / 2.0)  # Z -> Y
    eye.translate(0.0, sy * LIFT_ARM_Y, 0.0)
    geo.merge(eye)
    # Tub-end pivot eye (carries the tub pivot pin)
    tub_eye = CylinderGeometry(0.024, 0.05, radial_segments=14)
    tub_eye.rotate_x(math.pi / 2.0)
    tub_eye.translate(TUB_TIP_DX, sy * LIFT_ARM_Y, TUB_TIP_DZ)
    geo.merge(tub_eye)
    # Gusset plate reinforcing the tub-end connection
    gusset = BoxGeometry((0.08, ARM_WIDTH, 0.06))
    gusset.translate(TUB_TIP_DX - 0.04, sy * LIFT_ARM_Y, TUB_TIP_DZ / 2.0)
    geo.merge(gusset)
    return geo


def _lift_arm_assembly_mesh() -> MeshGeometry:
    """Full lift arm assembly: two mirrored arms + crossbars + tub pivot pin."""
    geo = MeshGeometry()
    # Two side arms via mirrored for-loop over the shared helper
    for i in range(2):
        sy = 1 - 2 * i  # +1, -1
        arm_geo = _one_lift_arm_mesh(sy)
        geo.merge(arm_geo)
    # Rear crossbar connecting both arms near the pivot
    rear_bar = BoxGeometry((0.06, 2 * LIFT_ARM_Y - ARM_WIDTH, ARM_HEIGHT))
    rear_bar.translate(0.03, 0.0, 0.0)
    geo.merge(rear_bar)
    # Mid crossbar for rigidity
    mid_x = ARM_LENGTH * 0.45
    mid_bar = BoxGeometry((0.05, 2 * LIFT_ARM_Y - ARM_WIDTH, ARM_HEIGHT * 0.8))
    mid_bar.translate(mid_x, 0.0, 0.0)
    geo.merge(mid_bar)
    # Tub-end pivot pin spanning between the arms
    pin = CylinderGeometry(0.014, 2 * LIFT_ARM_Y - 0.02, radial_segments=14)
    pin.rotate_x(math.pi / 2.0)  # Z -> Y
    pin.translate(TUB_TIP_DX, 0.0, TUB_TIP_DZ)
    geo.merge(pin)
    return geo


# ---------------------------------------------------------------------------
# Wheels (same as parent)
# ---------------------------------------------------------------------------
def _big_wheel_mesh() -> MeshGeometry:
    return WheelGeometry(
        WHEEL_R - 0.045,
        WHEEL_W - 0.018,
        rim=WheelRim(inner_radius=0.085, flange_height=0.012, flange_thickness=0.005),
        hub=WheelHub(
            radius=0.034,
            width=WHEEL_W - 0.012,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=5, circle_diameter=0.05, hole_diameter=0.006),
        ),
        face=WheelFace(dish_depth=0.010, front_inset=0.004),
        spokes=WheelSpokes(style="split_y", count=5, thickness=0.006, window_radius=0.018),
        bore=WheelBore(style="round", diameter=0.020),
    )


def _big_tire_mesh() -> MeshGeometry:
    return TireGeometry(
        WHEEL_R,
        WHEEL_W,
        inner_radius=WHEEL_R - 0.05,
        tread=TireTread(style="block", depth=0.008, count=22, land_ratio=0.6),
        sidewall=TireSidewall(style="square", bulge=0.02),
    )


CASTER_WHEEL_CTR_LOCAL = CASTER_WHEEL_R - 0.012


def _caster_yoke_mesh() -> MeshGeometry:
    geo = MeshGeometry()
    boss = CylinderGeometry(0.022, 0.034, radial_segments=18)
    boss.translate(0.0, 0.0, -0.005)
    geo.merge(boss)
    leg = BoxGeometry((CASTER_OFFSET + 0.02, 0.055, 0.02))
    leg.translate(CASTER_OFFSET / 2.0, 0.0, 0.0)
    geo.merge(leg)
    for sy in (1, -1):
        cheek = BoxGeometry((0.045, 0.012, abs(CASTER_WHEEL_CTR_LOCAL) + 0.03))
        cheek.translate(
            CASTER_OFFSET, sy * (CASTER_WHEEL_W / 2.0 + 0.008),
            (CASTER_WHEEL_CTR_LOCAL) / 2.0,
        )
        geo.merge(cheek)
    return geo


def _caster_wheel_mesh() -> MeshGeometry:
    return TireGeometry(
        CASTER_WHEEL_R,
        CASTER_WHEEL_W,
        inner_radius=CASTER_WHEEL_R - 0.028,
        sidewall=TireSidewall(style="rounded", bulge=0.05),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="plastic_tilt_truck_lift_tip")

    gray_plastic = model.material("gray_plastic", rgba=(0.62, 0.63, 0.64, 1.0))
    plastic_dark = model.material("plastic_dark", rgba=(0.50, 0.51, 0.52, 1.0))
    steel = model.material("steel_gray", rgba=(0.34, 0.35, 0.37, 1.0))
    steel_arm = model.material("steel_arm", rgba=(0.38, 0.39, 0.41, 1.0))
    wheel_gray = model.material("wheel_gray", rgba=(0.55, 0.56, 0.58, 1.0))
    rubber = model.material("rubber_dark", rgba=(0.18, 0.18, 0.19, 1.0))

    # ---- base frame (root) --------------------------------------------------
    frame = model.part("base_frame")
    frame.visual(mesh_from_geometry(_base_frame_mesh(), "frame"), material=steel, name="frame")
    frame.inertial = Inertial.from_geometry(
        Box((BODY_LEN, 2 * WHEEL_Y, 0.12)),
        mass=22.0,
        origin=Origin(xyz=(0.1, 0.0, AXLE_Z * 0.6)),
    )

    # ---- big rear wheels (continuous spin about Y) --------------------------
    for sy, wheel_name, joint_name in (
        (1, "wheel_l", "frame_to_wheel_l"),
        (-1, "wheel_r", "frame_to_wheel_r"),
    ):
        wheel = model.part(wheel_name)
        disc = _big_wheel_mesh()
        tire = _big_tire_mesh()
        disc.rotate_z(math.pi / 2.0)
        tire.rotate_z(math.pi / 2.0)
        wheel.visual(mesh_from_geometry(disc, f"{wheel_name}_disc"),
                     material=wheel_gray, name=f"{wheel_name}_disc")
        wheel.visual(mesh_from_geometry(tire, f"{wheel_name}_tire"),
                     material=rubber, name=f"{wheel_name}_tire")
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=WHEEL_R, length=WHEEL_W), mass=2.2,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        )
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(-AXLE_X, sy * WHEEL_Y, AXLE_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=60.0, velocity=30.0),
        )

    # ---- front swivel casters (yaw about Z) + roll (about Y) ----------------
    for i, sy in ((0, 1), (1, -1)):
        yoke = model.part(f"caster_yoke_{i}")
        yoke.visual(mesh_from_geometry(_caster_yoke_mesh(), f"caster_yoke_{i}_body"),
                    material=steel, name=f"caster_yoke_{i}_body")
        yoke.inertial = Inertial.from_geometry(
            Box((0.14, 0.08, 0.10)), mass=0.6,
            origin=Origin(xyz=(CASTER_OFFSET / 2.0, 0.0, 0.0)),
        )
        model.articulation(
            f"frame_to_caster_yoke_{i}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=yoke,
            origin=Origin(xyz=(CASTER_X, sy * CASTER_Y, 0.012)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=20.0, velocity=10.0, lower=-math.pi, upper=math.pi),
        )

        wheel = model.part(f"caster_wheel_{i}")
        cw = _caster_wheel_mesh()
        cw.rotate_z(math.pi / 2.0)
        wheel.visual(mesh_from_geometry(cw, f"caster_wheel_{i}_tire"),
                     material=rubber, name=f"caster_wheel_{i}_tire")
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=CASTER_WHEEL_R, length=CASTER_WHEEL_W), mass=0.3,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        )
        model.articulation(
            f"caster_yoke_{i}_to_wheel",
            ArticulationType.CONTINUOUS,
            parent=yoke,
            child=wheel,
            origin=Origin(xyz=(CASTER_OFFSET, 0.0, CASTER_WHEEL_R - 0.012)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=20.0, velocity=30.0),
        )

    # ---- lift arm (raises the tub off the frame) ----------------------------
    # The lift arm local frame has its origin at the pivot on the frame.
    # At q=0 the arm is horizontal, extending forward (+X).
    lift_arm = model.part("lift_arm")
    lift_arm.visual(
        mesh_from_geometry(_lift_arm_assembly_mesh(), "lift_arm_body"),
        material=steel_arm, name="lift_arm_body",
    )
    lift_arm.inertial = Inertial.from_geometry(
        Box((ARM_LENGTH, 2 * LIFT_ARM_Y, ARM_HEIGHT + 0.04)),
        mass=8.0,
        origin=Origin(xyz=(ARM_LENGTH / 2.0, 0.0, 0.0)),
    )
    # Lift arm pivots on the frame: axis (0,-1,0) so positive q raises the arm
    model.articulation(
        "frame_to_lift_arm",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=lift_arm,
        origin=Origin(xyz=(LIFT_PX, 0.0, LIFT_PZ)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=300.0, velocity=0.8, lower=0.0, upper=1.0),
    )

    # ---- plastic hopper (tips forward on the lift arm) ----------------------
    # The hopper meshes are authored in world coordinates. Its part frame sits
    # at the tub tip pivot on the lift arm. At q=0 for both joints, the hopper
    # frame coincides with the lift arm's tub-end articulation frame.
    # We translate the world-coord geometry by -_TUB_TIP_WORLD so it lands at
    # its authored world pose while still rotating correctly.
    hopper_offset = _TUB_TIP_WORLD

    def _seat(geo: MeshGeometry) -> MeshGeometry:
        return geo.translate(-hopper_offset[0], -hopper_offset[1], -hopper_offset[2])

    hopper = model.part("hopper")
    hopper.visual(mesh_from_geometry(_seat(_hopper_shell_mesh()), "hopper_outer"),
                  material=gray_plastic, name="hopper_outer")
    hopper.visual(mesh_from_geometry(_seat(_hopper_inner_mesh()), "hopper_inner"),
                  material=plastic_dark, name="hopper_inner")
    hopper.visual(mesh_from_geometry(_seat(_rim_ring_mesh()), "hopper_rim"),
                  material=gray_plastic, name="hopper_rim")
    hopper.visual(mesh_from_geometry(_seat(_hopper_ribs_mesh()), "hopper_ribs"),
                  material=gray_plastic, name="hopper_ribs")
    hopper.visual(mesh_from_geometry(_seat(_hopper_cradle_mesh()), "hopper_cradle"),
                  material=steel, name="hopper_cradle")
    hopper.inertial = Inertial.from_geometry(
        Box((BODY_LEN, BODY_WID, RIM_Z - BOTTOM_Z)),
        mass=40.0,
        origin=Origin(xyz=(
            0.0 - hopper_offset[0],
            0.0,
            (RIM_Z + BOTTOM_Z) / 2.0 - hopper_offset[2],
        )),
    )
    # Tub tips forward on the lift arm: axis (0,1,0) so positive q tips the
    # front down (dumping contents forward over the lip).
    model.articulation(
        "lift_arm_to_hopper",
        ArticulationType.REVOLUTE,
        parent=lift_arm,
        child=hopper,
        origin=Origin(xyz=(TUB_TIP_DX, 0.0, TUB_TIP_DZ)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=200.0, velocity=1.0, lower=0.0, upper=1.2),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("base_frame")
    wheel_l = object_model.get_part("wheel_l")
    wheel_r = object_model.get_part("wheel_r")
    lift_arm = object_model.get_part("lift_arm")
    hopper = object_model.get_part("hopper")
    caster_w0 = object_model.get_part("caster_wheel_0")
    caster_w1 = object_model.get_part("caster_wheel_1")

    spin_l = object_model.get_articulation("frame_to_wheel_l")
    swivel0 = object_model.get_articulation("frame_to_caster_yoke_0")
    lift_joint = object_model.get_articulation("frame_to_lift_arm")
    tip_joint = object_model.get_articulation("lift_arm_to_hopper")

    # --- joint types/axes match the lift-and-tip mechanism ------------------
    ctx.check(
        "big wheels are continuous rollers about Y",
        spin_l.joint_type == ArticulationType.CONTINUOUS
        and tuple(spin_l.axis) == (0.0, 1.0, 0.0),
        details=f"type={spin_l.joint_type}, axis={spin_l.axis}",
    )
    ctx.check(
        "front casters swivel (revolute) about Z",
        swivel0.joint_type == ArticulationType.REVOLUTE
        and tuple(swivel0.axis) == (0.0, 0.0, 1.0),
        details=f"type={swivel0.joint_type}, axis={swivel0.axis}",
    )
    ctx.check(
        "lift arm is revolute (raises the tub)",
        lift_joint.joint_type == ArticulationType.REVOLUTE,
        details=f"type={lift_joint.joint_type}, axis={lift_joint.axis}",
    )
    ctx.check(
        "tub tip is revolute (tips the tub forward)",
        tip_joint.joint_type == ArticulationType.REVOLUTE,
        details=f"type={tip_joint.joint_type}, axis={tip_joint.axis}",
    )

    # --- everything sits on the floor: wheels & casters touch z ~ 0 ---------
    for part, name in (
        (wheel_l, "wheel_l"), (wheel_r, "wheel_r"),
        (caster_w0, "caster_wheel_0"), (caster_w1, "caster_wheel_1"),
    ):
        aabb = ctx.part_world_aabb(part)
        ctx.check(
            f"{name} touches the floor (z~0)",
            aabb is not None and -0.01 <= aabb[0][2] <= 0.02,
            details=f"{name} z_min={aabb[0][2] if aabb else None}",
        )
    fr = ctx.part_world_aabb(frame)
    ctx.check(
        "frame bottom at/near the floor",
        fr is not None and fr[0][2] <= 0.04,
        details=f"frame z_min={fr[0][2] if fr else None}",
    )

    # --- hopper is the big body, standing above the frame -------------------
    ha = ctx.part_world_aabb(hopper)
    he = _ext(ha)
    ctx.check(
        "hopper is the large tapered body (tall, wide, long)",
        he[2] > 0.6 and he[1] > 0.7 and he[0] > 1.2,
        details=f"hopper_ext={he}",
    )
    ctx.check(
        "hopper top rim near expected rim height",
        abs(ha[1][2] - RIM_Z) < 0.07,
        details=f"hopper_top_z={ha[1][2]}, expected~{RIM_Z}",
    )
    body_aabb = ctx.part_element_world_aabb(hopper, elem="hopper_outer")
    ctx.check(
        "hopper body sits above the floor (rides on the lift arm)",
        body_aabb is not None and body_aabb[0][2] > 0.20,
        details=f"hopper body z_min={body_aabb[0][2] if body_aabb else None}",
    )

    # --- wheels mirror-identical, opposite Y sides --------------------------
    wl = ctx.part_world_position(wheel_l)
    wr = ctx.part_world_position(wheel_r)
    ctx.check(
        "big wheels mirror across centerline",
        wl is not None and wr is not None and abs(wl[1] + wr[1]) < 0.006
        and abs(wl[2] - wr[2]) < 0.006,
        details=f"left={wl}, right={wr}",
    )
    el = _ext(ctx.part_world_aabb(wheel_l))
    er = _ext(ctx.part_world_aabb(wheel_r))
    ctx.check(
        "big wheels identical size",
        all(abs(el[i] - er[i]) < 0.004 for i in range(3)),
        details=f"left_ext={el}, right_ext={er}",
    )

    # --- big wheel actually rolls -------------------------------------------
    e0 = ctx.part_element_world_aabb(wheel_l, elem="wheel_l_disc")
    with ctx.pose({spin_l: math.radians(36.0)}):
        e1 = ctx.part_element_world_aabb(wheel_l, elem="wheel_l_disc")
    ctx.check(
        "big wheel disc spins (spoke extent shifts under rotation about Y)",
        e0 is not None and e1 is not None
        and (abs(e1[0][0] - e0[0][0]) > 0.001 or abs(e1[0][2] - e0[0][2]) > 0.001
             or abs(e1[1][0] - e0[1][0]) > 0.001 or abs(e1[1][2] - e0[1][2]) > 0.001),
        details=f"rest_disc_aabb={e0}, spun_disc_aabb={e1}",
    )

    # --- caster swivel actually yaws the wheel about Z ----------------------
    c0_rest = ctx.part_world_position(caster_w0)
    with ctx.pose({swivel0: math.pi / 2.0}):
        c0_yaw = ctx.part_world_position(caster_w0)
    ctx.check(
        "front caster yaws about its king-pin (wheel swings in XY)",
        c0_rest is not None and c0_yaw is not None
        and (abs(c0_yaw[0] - c0_rest[0]) > 0.01 or abs(c0_yaw[1] - c0_rest[1]) > 0.01),
        details=f"rest={c0_rest}, yawed={c0_yaw}",
    )

    # --- lift arm raises the hopper (Z increases) ---------------------------
    rest_hopper_pos = ctx.part_world_position(hopper)
    with ctx.pose({lift_joint: 0.8}):
        lifted_hopper_pos = ctx.part_world_position(hopper)
    ctx.check(
        "lift arm raises the hopper (Z increases significantly)",
        rest_hopper_pos is not None and lifted_hopper_pos is not None
        and lifted_hopper_pos[2] > rest_hopper_pos[2] + 0.15,
        details=f"rest_z={rest_hopper_pos[2]}, lifted_z={lifted_hopper_pos[2]}",
    )

    # --- tub tip actually tips the hopper forward ---------------------------
    rest_hopper_aabb = ctx.part_world_aabb(hopper)
    with ctx.pose({tip_joint: 1.0}):
        tipped_aabb = ctx.part_world_aabb(hopper)
    ctx.check(
        "tub tip moves the hopper front downward (forward tip)",
        abs(tipped_aabb[0][2] - rest_hopper_aabb[0][2]) > 0.05
        or abs(tipped_aabb[1][0] - rest_hopper_aabb[1][0]) > 0.05,
        details=f"rest_aabb={rest_hopper_aabb}, tipped_aabb={tipped_aabb}",
    )

    # --- combined lift+tip shows rise-and-tip behavior ----------------------
    with ctx.pose({lift_joint: 0.8, tip_joint: 1.0}):
        combined_aabb = ctx.part_world_aabb(hopper)
    ctx.check(
        "combined lift+tip: hopper rises above rest AND tips forward",
        combined_aabb[1][2] > rest_hopper_aabb[1][2] + 0.10  # rose
        and combined_aabb[0][0] < rest_hopper_aabb[0][0] - 0.05,  # front moved back/down
        details=f"rest={rest_hopper_aabb}, combined={combined_aabb}",
    )

    # --- lift arm sits between the wheels at rest (correct lateral position)
    la = ctx.part_world_aabb(lift_arm)
    ctx.check(
        "lift arm lateral extent inside the big wheels",
        la is not None and la[0][1] > -WHEEL_Y and la[1][1] < WHEEL_Y,
        details=f"lift_arm_y=[{la[0][1]:.3f}, {la[1][1]:.3f}], wheels at ±{WHEEL_Y}",
    )

    # --- caster wheels seated in their forks (captured) ---------------------
    ctx.allow_overlap(
        caster_w0, object_model.get_part("caster_yoke_0"),
        reason="The caster wheel is captured between the swivel-fork cheeks.",
    )
    ctx.allow_overlap(
        caster_w1, object_model.get_part("caster_yoke_1"),
        reason="The caster wheel is captured between the swivel-fork cheeks.",
    )
    ctx.expect_contact(
        caster_w0, object_model.get_part("caster_yoke_0"),
        name="caster wheel 0 held in its fork",
    )

    # --- each swivel caster's king-pin boss seats up into its frame mount plate
    yoke0 = object_model.get_part("caster_yoke_0")
    yoke1 = object_model.get_part("caster_yoke_1")
    ctx.allow_overlap(
        frame, yoke0,
        reason="The caster king-pin boss is captured up into the frame swivel "
               "mount plate it swivels on.",
    )
    ctx.allow_overlap(
        frame, yoke1,
        reason="The caster king-pin boss is captured up into the frame swivel "
               "mount plate it swivels on.",
    )
    ctx.expect_contact(frame, yoke0, name="caster 0 king-pin seated in frame plate")
    ctx.expect_contact(frame, yoke1, name="caster 1 king-pin seated in frame plate")

    # --- the steel axle passes through the big-wheel hubs (genuine) ----------
    ctx.allow_overlap(
        frame, wheel_l,
        reason="The frame axle passes through the left wheel hub bore.",
    )
    ctx.allow_overlap(
        frame, wheel_r,
        reason="The frame axle passes through the right wheel hub bore.",
    )

    # --- lift arm pivot wraps the frame pivot pin (captured pin) ------------
    ctx.allow_overlap(
        frame, lift_arm,
        elem_a="frame",
        elem_b="lift_arm_body",
        reason="The lift arm pivot eyes wrap the frame pivot pin at the rear "
               "bracket, forming the lift hinge.",
    )
    ctx.expect_contact(
        frame, lift_arm,
        name="lift arm pivot seated on frame brackets",
    )

    # --- hopper pivot bracket wraps the lift arm tub-end pin (captured pin) --
    ctx.allow_overlap(
        lift_arm, hopper,
        elem_a="lift_arm_body",
        elem_b="hopper_cradle",
        reason="The hopper pivot bracket sleeve wraps the lift arm tub-end "
               "pin, forming the tip hinge.",
    )
    ctx.expect_contact(
        lift_arm, hopper,
        name="hopper pivot bracket seated on lift arm",
    )

    return ctx.report()


object_model = build_object_model()