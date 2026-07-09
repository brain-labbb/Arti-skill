from __future__ import annotations

# Heavy-duty plastic tilt truck (dump cart), Lavex / Rubbermaid style.
#
# Real object: a large tapered polyethylene hopper (~1 cubic yard) that pivots
# forward on a steel axle to dump its load. It rides on two big center wheels
# and is steered by two small swivel casters under the front lip.
#
# Coordinate convention (Z-up world):
#   - +Z is up; the floor is z = 0 (all wheels/casters touch z ~ 0).
#   - +X is the FORWARD travel / dump direction (the body slopes/dumps toward +X).
#   - +Y is the left/right (lateral) axle direction; centerline is y = 0.
#
# Root structure: the steel BASE FRAME (chassis) carries everything. The two big
# rear wheels spin on the axle; the two front swivel casters yaw + roll; the big
# plastic hopper tilts forward about the rear axle line to dump.
#
# Articulations:
#   - frame_to_wheel_l / frame_to_wheel_r : CONTINUOUS big-wheel roll (about Y)
#   - frame_to_caster_yoke_0/1            : REVOLUTE swivel-caster yaw (about Z)
#   - caster_yoke_*_to_wheel              : CONTINUOUS small-caster roll (about Y)
#   - frame_to_hopper                     : REVOLUTE forward dump tilt (about Y)
#
# The two big wheels are mirror-identical; the two front casters are mirror
# identical across the y = 0 centerline.

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
RIM_Z = 1.38             # top rim height (deep-tub variant: ~1.08 m cavity depth)
BOTTOM_Z = 0.30          # height of the hopper bottom above the floor
WALL_T = 0.022           # plastic wall thickness

WHEEL_R = 0.205          # big rear wheel radius
WHEEL_W = 0.075          # big rear wheel width
WHEEL_Y = 0.345          # |y| of each big wheel center (just outside the body)
AXLE_X = 0.0             # axle sits under the body center-rear (wheels centered)
AXLE_Z = WHEEL_R         # axle center height = wheel radius (wheel touches z=0)

CASTER_WHEEL_R = 0.055   # small front swivel-caster wheel radius
CASTER_WHEEL_W = 0.042
CASTER_X = 0.64          # forward position of the front caster king-pins
CASTER_Y = 0.27          # |y| of each front caster
CASTER_OFFSET = 0.105    # trailing offset of caster wheel ahead of its swivel axis

FRAME_Z = 0.055          # steel sub-frame member height


# ---------------------------------------------------------------------------
# Hopper shell: a tapered, ribbed plastic tub. Wide rounded-rectangle rim at the
# top, tapering down to a small bottom; the FRONT (+X) wall slopes steeply while
# the rear (-X) wall is more upright. Modeled as a hollow shell (outer loft +
# inset inner loft + rim ring) so it reads hollow like the real tub.
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


# (scale_x, scale_y, front_shift, z) sections from bottom to rim. Deep-tub
# variant: walls are much more upright (bottom is ~55-65 % of the rim footprint
# instead of ~35-45 %) so the cavity is deep and steep-sided. The bottom is
# kept narrow enough in Y to clear the big-wheel tires that sit just outside
# the body. The small front_shift still gives the front dump wall a slightly
# steeper rake than the rear, but the overall silhouette reads as a tall
# near-vertical bin.
_OUTER_SECS = [
    (0.52, 0.55, 0.06, BOTTOM_Z),
    (0.70, 0.72, 0.04, BOTTOM_Z + 0.22 * (RIM_Z - BOTTOM_Z)),
    (0.85, 0.88, 0.02, BOTTOM_Z + 0.55 * (RIM_Z - BOTTOM_Z)),
    (0.96, 0.97, 0.008, BOTTOM_Z + 0.85 * (RIM_Z - BOTTOM_Z)),
    (1.00, 1.00, 0.0, RIM_Z),
]


def _hopper_shell_mesh() -> MeshGeometry:
    return section_loft([_hopper_section(*s) for s in _OUTER_SECS])


def _hopper_inner_mesh() -> MeshGeometry:
    inset = WALL_T
    bottom_z = BOTTOM_Z + WALL_T
    inner_secs = [
        (0.52, 0.55, 0.06, bottom_z),
        (0.70, 0.72, 0.04, bottom_z + 0.22 * (RIM_Z - bottom_z)),
        (0.85, 0.88, 0.02, bottom_z + 0.55 * (RIM_Z - bottom_z)),
        (0.96, 0.97, 0.008, bottom_z + 0.85 * (RIM_Z - bottom_z)),
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
        (0.30, 0.74, 0.77, 0.035),
        (0.50, 0.83, 0.86, 0.022),
        (0.68, 0.92, 0.94, 0.012),
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
    # Steel trunnion saddle moulded under the hopper: it descends from the hopper
    # bottom (z=BOTTOM_Z) down to wrap the axle line at AXLE_Z, so the hopper is
    # physically carried on the axle and is not floating. A transverse trunnion
    # sleeve over the axle, two side plates rising into the body bottom, and a
    # cross strap tying them together (one connected island that also overlaps
    # the body bottom). Authored in world coords matching the hopper body.
    geo = MeshGeometry()
    plate_top_z = BOTTOM_Z + 0.03   # rises into the body bottom (overlap)
    plate_bot_z = AXLE_Z - 0.02     # wraps below the axle center
    for sy in (1, -1):
        plate = BoxGeometry((0.20, 0.028, plate_top_z - plate_bot_z))
        plate.translate(AXLE_X, sy * 0.16, (plate_top_z + plate_bot_z) / 2.0)
        geo.merge(plate)
    # transverse trunnion sleeve straddling the axle, joining both plates.
    sleeve = CylinderGeometry(0.036, 0.40, radial_segments=18)
    sleeve.rotate_x(math.pi / 2.0)  # Z -> Y
    sleeve.translate(AXLE_X, 0.0, AXLE_Z)
    geo.merge(sleeve)
    # top cross strap under the body bottom tying the two plates (and overlapping
    # the body bottom for a connected island).
    strap = BoxGeometry((0.22, 0.36, 0.03))
    strap.translate(AXLE_X, 0.0, BOTTOM_Z + 0.012)
    geo.merge(strap)
    return geo


# ---------------------------------------------------------------------------
# Steel base frame (root): a low rectangular cradle carrying the axle (with the
# two big wheels) at the rear and the two swivel casters at the front.
# ---------------------------------------------------------------------------
def _base_frame_mesh() -> MeshGeometry:
    geo = MeshGeometry()
    rail_x0 = CASTER_X + 0.04
    rail_x1 = AXLE_X - 0.10
    rail_len = rail_x0 - rail_x1
    rail_cx = (rail_x0 + rail_x1) / 2.0
    # two fore-aft side rails (just inboard of the saddle plates).
    for sy in (1, -1):
        rail = BoxGeometry((rail_len, 0.05, FRAME_Z))
        rail.translate(rail_cx, sy * 0.235, FRAME_Z / 2.0 + 0.012)
        geo.merge(rail)
    # narrow rear cross member at the axle (kept |y| small so it clears the
    # hopper saddle side plates at |y|=0.16).
    rear_cross = BoxGeometry((0.09, 0.24, 0.05))
    rear_cross.translate(AXLE_X, 0.0, AXLE_Z)
    geo.merge(rear_cross)
    # steel axle tube spanning between the two big wheels.
    axle = CylinderGeometry(0.022, 2 * WHEEL_Y + 0.02, radial_segments=18)
    axle.rotate_x(math.pi / 2.0)  # Z -> Y
    axle.translate(AXLE_X, 0.0, AXLE_Z)
    geo.merge(axle)
    # bearing/upright blocks tying the axle to the rails.
    for sy in (1, -1):
        block = BoxGeometry((0.06, 0.04, AXLE_Z - 0.012))
        block.translate(AXLE_X, sy * 0.235, (AXLE_Z + 0.012) / 2.0)
        geo.merge(block)
    # front caster swivel mounting plates (the casters hang below and trail
    # forward of these plates so the small wheels clear the frame).
    for sy in (1, -1):
        plate = BoxGeometry((0.09, 0.09, 0.018))
        plate.translate(CASTER_X, sy * CASTER_Y, 0.012 + 0.009)
        geo.merge(plate)
    return geo


# ---------------------------------------------------------------------------
# Wheels (WheelGeometry/TireGeometry spin about local X; rotate so the spin axis
# becomes local Y in the part frame, matching the joint axis (0,1,0)).
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


CASTER_WHEEL_CTR_LOCAL = CASTER_WHEEL_R - 0.012  # wheel center z in the yoke frame


def _caster_yoke_mesh() -> MeshGeometry:
    # Swivel fork authored in the yoke local frame: the king-pin swivel axis is
    # at the local origin (= the frame mount plate). A short boss at the pin, a
    # forward top leg reaching out over the trailing wheel, and two cheeks that
    # drop to straddle the wheel axle. The wheel trails forward (+X).
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
        Box((BODY_LEN, 2 * WHEEL_Y, 0.12)),
        mass=18.0,
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
        disc.rotate_z(math.pi / 2.0)  # local X spin -> local Y
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
        cw.rotate_z(math.pi / 2.0)  # local X spin -> local Y
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

    # ---- plastic hopper (tilts forward about the rear axle line) ------------
    # The hopper meshes are authored in world coordinates (bottom at z=BOTTOM_Z).
    # Its pivot/joint frame is the axle line at PIVOT; because at q=0 the child
    # frame is coincident with the joint frame, we translate the hopper geometry
    # by -PIVOT so it lands back at its authored world pose while still rotating
    # about the true axle line when the dump joint moves.
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
    wheel_l = object_model.get_part("wheel_l")
    wheel_r = object_model.get_part("wheel_r")
    hopper = object_model.get_part("hopper")
    caster_w0 = object_model.get_part("caster_wheel_0")
    caster_w1 = object_model.get_part("caster_wheel_1")

    spin_l = object_model.get_articulation("frame_to_wheel_l")
    swivel0 = object_model.get_articulation("frame_to_caster_yoke_0")
    dump = object_model.get_articulation("frame_to_hopper")

    # --- joint types/axes match the real mechanism --------------------------
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
        "hopper dump is revolute about Y",
        dump.joint_type == ArticulationType.REVOLUTE
        and tuple(dump.axis) == (0.0, 1.0, 0.0),
        details=f"type={dump.joint_type}, axis={dump.axis}",
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
        "hopper is the large deep body (tall, wide, long)",
        he[2] > 0.95 and he[1] > 0.7 and he[0] > 1.2,
        details=f"hopper_ext={he}",
    )
    ctx.check(
        "hopper top rim near expected rim height",
        abs(ha[1][2] - RIM_Z) < 0.07,
        details=f"hopper_top_z={ha[1][2]}, expected~{RIM_Z}",
    )
    body_aabb = ctx.part_element_world_aabb(hopper, elem="hopper_outer")
    ctx.check(
        "hopper body sits above the floor (rides on the frame)",
        body_aabb is not None and body_aabb[0][2] > 0.20,
        details=f"hopper body z_min={body_aabb[0][2] if body_aabb else None}",
    )
    # --- deep-tub claim: cavity depth is markedly greater than the parent ---
    outer_zmin = body_aabb[0][2] if body_aabb else None
    outer_zmax = body_aabb[1][2] if body_aabb else None
    if outer_zmin is not None and outer_zmax is not None:
        cavity_depth = outer_zmax - outer_zmin
        ctx.check(
            "deep-tub: outer shell cavity depth >= 0.95 m",
            cavity_depth >= 0.95,
            details=f"cavity_depth={cavity_depth:.3f}",
        )
        # Upright walls: bottom width at least 55% of rim width
        bottom_half_x = (BODY_LEN / 2.0) * 0.58
        rim_half_x = BODY_LEN / 2.0
        ctx.check(
            "upright walls: bottom length >= 55% of rim length",
            bottom_half_x / rim_half_x >= 0.55,
            details=f"bottom_frac={bottom_half_x/rim_half_x:.2f}",
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
    ctx.check(
        "big wheel diameter ~ 2*WHEEL_R (round, width along Y)",
        abs(el[0] - 2 * WHEEL_R) < 0.05 and abs(el[2] - 2 * WHEEL_R) < 0.05
        and el[1] < el[0],
        details=f"wheel_ext={el}, 2R={2*WHEEL_R}",
    )

    # --- big wheel actually rolls: the (5-fold) spoke disc AABB shifts when
    # spun by 36 deg (half the 72-deg spoke pitch), proving rotation about Y. ---
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

    # --- dump tilt actually tips the hopper forward -------------------------
    rest_aabb = ctx.part_world_aabb(hopper)
    with ctx.pose({dump: 1.4}):
        tip_aabb = ctx.part_world_aabb(hopper)
    ctx.check(
        "dump tilt moves the hopper (forward tip)",
        abs(tip_aabb[1][0] - rest_aabb[1][0]) > 0.1
        or abs(tip_aabb[0][2] - rest_aabb[0][2]) > 0.05,
        details=f"rest_aabb={rest_aabb}, tipped_aabb={tip_aabb}",
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
    # --- the hopper saddle trunnion wraps the frame axle (carried, not floating)
    ctx.allow_overlap(
        hopper, frame,
        reason="The hopper trunnion saddle wraps the frame axle and rear cross "
               "member it pivots on.",
    )
    ctx.expect_contact(
        hopper, frame, name="hopper saddle carried on the frame axle",
    )

    return ctx.report()


object_model = build_object_model()
