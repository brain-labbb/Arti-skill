from __future__ import annotations

# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)

# Conventions: +Z up, lamp arms reach toward +X at rest.

# ---- Base (weighted disk + on/off switch) ----------------------------------
BASE_DISK_RADIUS = 0.115
BASE_DISK_HEIGHT = 0.020
BASE_CAP_RADIUS = 0.092
BASE_CAP_HEIGHT = 0.010
BASE_FELT_RADIUS = 0.108        # rubber/felt pad under the disk
BASE_FELT_HEIGHT = 0.004
SWITCH_RADIUS = 0.008
SWITCH_HEIGHT = 0.014
SWITCH_OFFSET = 0.062           # X offset of switch from base center

BASE_TOP_Z = BASE_FELT_HEIGHT + BASE_DISK_HEIGHT + BASE_CAP_HEIGHT  # 0.034

# ---- Swivel column (rotates about Z on the base) ---------------------------
SWIVEL_COLUMN_RADIUS = 0.018
SWIVEL_COLUMN_HEIGHT = 0.070
SWIVEL_COLLAR_RADIUS = 0.024
SWIVEL_COLLAR_HEIGHT = 0.008
SWIVEL_HUB_Z = SWIVEL_COLUMN_HEIGHT          # top of swivel column (relative to swivel frame)
# The swivel joint origin in base frame: just above the base cap.
SWIVEL_JOINT_Z = BASE_TOP_Z

# Pivot lugs at the top of the swivel column (where the lower arm hinges).
PIVOT_LUG_RADIUS = 0.011
PIVOT_LUG_LENGTH = 0.006
PIVOT_LUG_Y = 0.013                          # ± Y offset of the two lugs

# ---- Arms (Anglepoise-style: spine + parallel rails + spring) --------------
ARM_HUB_RADIUS = 0.011
ARM_HUB_LENGTH = 0.020                       # proximal hub spans the two yokes
ARM_YOKE_RADIUS = 0.010
ARM_YOKE_LENGTH = 0.006
ARM_YOKE_Y = 0.013                           # ± Y offset of the distal yokes
ARM_SPINE_W = 0.008                          # spine thickness perpendicular to arm
ARM_SPINE_H = 0.006                          # spine height (along Z in arm-local before tilt)
ARM_RAIL_Y = 0.011                           # ± Y offset of the two side rails (chosen to bridge hub & yokes)
ARM_RAIL_W = 0.005
ARM_RAIL_H = 0.004
ARM_END_INSET = 0.036                        # rail / spring inset from each end hub (also keeps spine clear of swivel)
ARM_CROSSBAR_W = 0.022                       # length of crossbar between rails (Y direction)
ARM_CROSSBAR_H = 0.006                       # crossbar thickness along arm-local Z
ARM_CROSSBAR_T = 0.005                       # crossbar thickness along arm axis
ARM_SPRING_RADIUS = 0.005

# Geometry of each arm (vector from start hub to end hub, arm-local).
LOWER_DX = 0.230
LOWER_DZ = 0.175
LOWER_LENGTH = math.hypot(LOWER_DX, LOWER_DZ)
LOWER_ANGLE = math.atan2(LOWER_DZ, LOWER_DX)

UPPER_DX = 0.250
UPPER_DZ = -0.150
UPPER_LENGTH = math.hypot(UPPER_DX, UPPER_DZ)
UPPER_ANGLE = math.atan2(UPPER_DZ, UPPER_DX)

# ---- Head (conical shade + bulb on a small bracket) ------------------------
HEAD_HUB_RADIUS = 0.011
HEAD_HUB_LENGTH = 0.020
HEAD_NECK_DX = 0.022                         # offset from head hub to shade base
HEAD_NECK_DZ = -0.014
HEAD_PITCH = -0.45                           # built-in shade pitch (rad, downward)

SHADE_BASE_RADIUS = 0.018
SHADE_OPEN_RADIUS = 0.066
SHADE_DEPTH = 0.108
SHADE_WALL = 0.004
BULB_RADIUS = 0.018
SOCKET_LENGTH = 0.018
SOCKET_RADIUS = 0.014


def _save_mesh(name, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _make_shade_mesh():
    # Hollow conical-frustum shade with a subtle parabolic outer profile.
    outer_profile = [
        (SHADE_BASE_RADIUS, 0.000),
        (SHADE_BASE_RADIUS + 0.006, 0.012),
        (SHADE_BASE_RADIUS + 0.024, 0.036),
        (SHADE_BASE_RADIUS + 0.038, 0.066),
        (SHADE_OPEN_RADIUS, SHADE_DEPTH),
    ]
    inner_profile = [
        (max(SHADE_BASE_RADIUS - SHADE_WALL, 0.0), 0.000),
        (SHADE_BASE_RADIUS + 0.002, 0.016),
        (SHADE_BASE_RADIUS + 0.018, 0.038),
        (SHADE_BASE_RADIUS + 0.032, 0.066),
        (SHADE_OPEN_RADIUS - SHADE_WALL, SHADE_DEPTH - 0.002),
    ]
    shade = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )
    # Lathe default axis is Z; rotate so shade opens along +X (the head's forward direction).
    shade.rotate_y(math.pi / 2.0)
    return _save_mesh("desk_lamp_shade.obj", shade)


def _add_arm_segment(
    part,
    *,
    prefix: str,
    dx: float,
    dz: float,
    arm_material,
    hardware_material,
    spring_material,
):
    """Build an Anglepoise-style arm segment from start hub (origin) to end hub at (dx, 0, dz)."""

    length = math.hypot(dx, dz)
    angle = math.atan2(dz, dx)
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)

    def axial(t):
        return (cos_a * t, 0.0, sin_a * t)

    # Proximal hub at the arm's origin (axle that fits inside the parent yokes).
    part.visual(
        Cylinder(radius=ARM_HUB_RADIUS, length=ARM_HUB_LENGTH),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_material,
        name=f"{prefix}_hub",
    )
    # Distal yokes at the arm's tip — receive the next arm's proximal hub.
    for side, sign in (("left", -1.0), ("right", 1.0)):
        part.visual(
            Cylinder(radius=ARM_YOKE_RADIUS, length=ARM_YOKE_LENGTH),
            origin=Origin(
                xyz=(dx, sign * ARM_YOKE_Y, dz),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=hardware_material,
            name=f"{prefix}_yoke_{side}",
        )

    # Central spine connecting the two hubs.
    spine_start = ARM_END_INSET * 0.6
    spine_end = length - ARM_END_INSET * 0.6
    spine_len = spine_end - spine_start
    spine_center_t = (spine_start + spine_end) * 0.5
    part.visual(
        Box((spine_len, ARM_SPINE_W, ARM_SPINE_H)),
        origin=Origin(xyz=axial(spine_center_t), rpy=(0.0, -angle, 0.0)),
        material=arm_material,
        name=f"{prefix}_spine",
    )

    # Two parallel side rails (one on each Y side). They span the full arm so
    # they bridge the proximal hub and the distal yokes.
    rail_start = 0.0
    rail_end = length
    rail_len = rail_end - rail_start
    rail_center_t = (rail_start + rail_end) * 0.5
    for side, sign in (("left", -1.0), ("right", 1.0)):
        part.visual(
            Box((rail_len, ARM_RAIL_W, ARM_RAIL_H)),
            origin=Origin(
                xyz=(
                    cos_a * rail_center_t,
                    sign * ARM_RAIL_Y,
                    sin_a * rail_center_t,
                ),
                rpy=(0.0, -angle, 0.0),
            ),
            material=arm_material,
            name=f"{prefix}_rail_{side}",
        )

    # Crossbars near each end, joining the two rails through the spine.
    crossbar_axial_offsets = (
        ARM_END_INSET + 0.012,
        length - ARM_END_INSET - 0.012,
    )
    for label, t in (("near", crossbar_axial_offsets[0]), ("far", crossbar_axial_offsets[1])):
        part.visual(
            Box((ARM_CROSSBAR_T, ARM_CROSSBAR_W, ARM_CROSSBAR_H)),
            origin=Origin(
                xyz=(cos_a * t, 0.0, sin_a * t),
                rpy=(0.0, -angle, 0.0),
            ),
            material=arm_material,
            name=f"{prefix}_crossbar_{label}",
        )

    # Tension spring running between the two crossbars (cylinder along arm axis).
    spring_axial_start = crossbar_axial_offsets[0]
    spring_axial_end = crossbar_axial_offsets[1]
    spring_center_t = (spring_axial_start + spring_axial_end) * 0.5
    spring_len = spring_axial_end - spring_axial_start
    part.visual(
        Cylinder(radius=ARM_SPRING_RADIUS, length=spring_len),
        origin=Origin(
            xyz=(cos_a * spring_center_t, 0.0, sin_a * spring_center_t),
            rpy=(0.0, math.pi / 2.0 - angle, 0.0),
        ),
        material=spring_material,
        name=f"{prefix}_spring",
    )

    return length, angle


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="architect_desk_lamp", assets=ASSETS)

    iron = model.material("cast_iron", rgba=(0.18, 0.20, 0.23, 1.0))
    felt = model.material("base_felt", rgba=(0.10, 0.10, 0.12, 1.0))
    enamel = model.material("lamp_enamel", rgba=(0.94, 0.84, 0.32, 1.0))
    chrome = model.material("brushed_chrome", rgba=(0.62, 0.65, 0.68, 1.0))
    spring_steel = model.material("spring_steel", rgba=(0.72, 0.74, 0.78, 1.0))
    switch_red = model.material("switch_red", rgba=(0.72, 0.18, 0.20, 1.0))
    bulb_glow = model.material("warm_bulb", rgba=(0.98, 0.92, 0.72, 0.85))
    socket_dark = model.material("socket_dark", rgba=(0.18, 0.18, 0.22, 1.0))

    shade_mesh = _make_shade_mesh()

    # ============================ Base (root) ============================
    base = model.part("base")

    base.visual(
        Cylinder(radius=BASE_FELT_RADIUS, length=BASE_FELT_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_FELT_HEIGHT * 0.5)),
        material=felt,
        name="base_felt",
    )
    base.visual(
        Cylinder(radius=BASE_DISK_RADIUS, length=BASE_DISK_HEIGHT),
        origin=Origin(
            xyz=(0.0, 0.0, BASE_FELT_HEIGHT + BASE_DISK_HEIGHT * 0.5)
        ),
        material=iron,
        name="base_disk",
    )
    base.visual(
        Cylinder(radius=BASE_CAP_RADIUS, length=BASE_CAP_HEIGHT),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                BASE_FELT_HEIGHT + BASE_DISK_HEIGHT + BASE_CAP_HEIGHT * 0.5,
            )
        ),
        material=iron,
        name="base_cap",
    )
    # On/off switch knob on top of the cap.
    base.visual(
        Cylinder(radius=SWITCH_RADIUS, length=SWITCH_HEIGHT),
        origin=Origin(
            xyz=(SWITCH_OFFSET, 0.0, BASE_TOP_Z + SWITCH_HEIGHT * 0.5)
        ),
        material=switch_red,
        name="switch_knob",
    )

    base.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_DISK_RADIUS, length=BASE_TOP_Z),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z * 0.5)),
    )

    # ============================ Swivel column ============================
    swivel = model.part("swivel_column")
    swivel.visual(
        Cylinder(radius=SWIVEL_COLLAR_RADIUS, length=SWIVEL_COLLAR_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_COLLAR_HEIGHT * 0.5)),
        material=chrome,
        name="swivel_collar",
    )
    swivel.visual(
        Cylinder(radius=SWIVEL_COLUMN_RADIUS, length=SWIVEL_COLUMN_HEIGHT),
        origin=Origin(
            xyz=(0.0, 0.0, SWIVEL_COLLAR_HEIGHT + SWIVEL_COLUMN_HEIGHT * 0.5)
        ),
        material=chrome,
        name="swivel_post",
    )
    # Pivot lugs at the top, oriented along Y to receive the lower arm hub.
    pivot_z_in_swivel = SWIVEL_COLLAR_HEIGHT + SWIVEL_COLUMN_HEIGHT
    for side, sign in (("left", -1.0), ("right", 1.0)):
        swivel.visual(
            Cylinder(radius=PIVOT_LUG_RADIUS, length=PIVOT_LUG_LENGTH),
            origin=Origin(
                xyz=(0.0, sign * PIVOT_LUG_Y, pivot_z_in_swivel),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=chrome,
            name=f"pivot_lug_{side}",
        )
    swivel.inertial = Inertial.from_geometry(
        Cylinder(
            radius=SWIVEL_COLLAR_RADIUS,
            length=SWIVEL_COLLAR_HEIGHT + SWIVEL_COLUMN_HEIGHT,
        ),
        mass=0.45,
        origin=Origin(
            xyz=(0.0, 0.0, (SWIVEL_COLLAR_HEIGHT + SWIVEL_COLUMN_HEIGHT) * 0.5)
        ),
    )

    # ============================ Lower arm ============================
    lower_arm = model.part("lower_arm")
    _add_arm_segment(
        lower_arm,
        prefix="lower",
        dx=LOWER_DX,
        dz=LOWER_DZ,
        arm_material=enamel,
        hardware_material=chrome,
        spring_material=spring_steel,
    )
    lower_arm.inertial = Inertial.from_geometry(
        Box((LOWER_LENGTH, 0.040, 0.040)),
        mass=0.32,
        origin=Origin(xyz=(LOWER_DX * 0.5, 0.0, LOWER_DZ * 0.5)),
    )

    # ============================ Upper arm ============================
    upper_arm = model.part("upper_arm")
    _add_arm_segment(
        upper_arm,
        prefix="upper",
        dx=UPPER_DX,
        dz=UPPER_DZ,
        arm_material=enamel,
        hardware_material=chrome,
        spring_material=spring_steel,
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((UPPER_LENGTH, 0.040, 0.040)),
        mass=0.28,
        origin=Origin(xyz=(UPPER_DX * 0.5, 0.0, UPPER_DZ * 0.5)),
    )

    # ============================ Head ============================
    head = model.part("head")
    # Head hub (where the head pivots on the upper arm).
    head.visual(
        Cylinder(radius=HEAD_HUB_RADIUS, length=HEAD_HUB_LENGTH),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="head_hub",
    )
    # Short neck bracket linking the hub to the shade base, with a pre-built tilt.
    neck_len = math.hypot(HEAD_NECK_DX, HEAD_NECK_DZ)
    neck_angle = math.atan2(HEAD_NECK_DZ, HEAD_NECK_DX)
    head.visual(
        Box((neck_len, 0.012, 0.012)),
        origin=Origin(
            xyz=(HEAD_NECK_DX * 0.5, 0.0, HEAD_NECK_DZ * 0.5),
            rpy=(0.0, -neck_angle, 0.0),
        ),
        material=enamel,
        name="head_neck",
    )
    # Lamp socket (where the bulb screws in), tilted along the shade axis.
    socket_pitch_origin = (HEAD_NECK_DX, 0.0, HEAD_NECK_DZ)
    cos_p = math.cos(HEAD_PITCH)
    sin_p = math.sin(HEAD_PITCH)
    socket_center_x = socket_pitch_origin[0] + cos_p * (SOCKET_LENGTH * 0.5)
    socket_center_z = socket_pitch_origin[2] + sin_p * (SOCKET_LENGTH * 0.5)
    head.visual(
        Cylinder(radius=SOCKET_RADIUS, length=SOCKET_LENGTH),
        origin=Origin(
            xyz=(socket_center_x, 0.0, socket_center_z),
            rpy=(0.0, math.pi / 2.0 - HEAD_PITCH, 0.0),
        ),
        material=socket_dark,
        name="socket",
    )
    # Shade origin = socket far end (start of cone in lathe local frame).
    shade_origin_x = socket_pitch_origin[0] + cos_p * SOCKET_LENGTH
    shade_origin_z = socket_pitch_origin[2] + sin_p * SOCKET_LENGTH
    head.visual(
        shade_mesh,
        origin=Origin(
            xyz=(shade_origin_x, 0.0, shade_origin_z),
            rpy=(0.0, -HEAD_PITCH, 0.0),
        ),
        material=enamel,
        name="shade_shell",
    )
    # Bulb sphere, screwed into the socket far end and sheltered inside the shade.
    bulb_center_t = SOCKET_LENGTH + BULB_RADIUS - 0.006
    bulb_x = socket_pitch_origin[0] + cos_p * bulb_center_t
    bulb_z = socket_pitch_origin[2] + sin_p * bulb_center_t
    head.visual(
        Sphere(radius=BULB_RADIUS),
        origin=Origin(xyz=(bulb_x, 0.0, bulb_z)),
        material=bulb_glow,
        name="bulb",
    )

    head.inertial = Inertial.from_geometry(
        Box((SHADE_DEPTH + 0.04, SHADE_OPEN_RADIUS * 2.0, SHADE_OPEN_RADIUS * 2.0)),
        mass=0.30,
        origin=Origin(xyz=(HEAD_NECK_DX + cos_p * SHADE_DEPTH * 0.5, 0.0, HEAD_NECK_DZ + sin_p * SHADE_DEPTH * 0.5)),
    )

    # ============================ Articulations ============================

    # Swivel pan around vertical axis on the base cap.
    model.articulation(
        "base_to_swivel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=swivel,
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0),
    )
    # Lower-arm tilt joint between swivel column top and lower arm root.
    model.articulation(
        "swivel_to_lower",
        ArticulationType.REVOLUTE,
        parent=swivel,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, pivot_z_in_swivel)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.5, lower=-0.85, upper=0.50),
    )
    # Elbow between lower and upper arms.
    model.articulation(
        "lower_to_upper",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(LOWER_DX, 0.0, LOWER_DZ)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.8, lower=-1.40, upper=0.40),
    )
    # Head tilt at upper arm tip.
    model.articulation(
        "upper_to_head",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=head,
        origin=Origin(xyz=(UPPER_DX, 0.0, UPPER_DZ)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=-1.05, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    swivel = object_model.get_part("swivel_column")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    head = object_model.get_part("head")

    base_to_swivel = object_model.get_articulation("base_to_swivel")
    swivel_to_lower = object_model.get_articulation("swivel_to_lower")
    lower_to_upper = object_model.get_articulation("lower_to_upper")
    upper_to_head = object_model.get_articulation("upper_to_head")

    base_disk = base.get_visual("base_disk")
    base_cap = base.get_visual("base_cap")
    swivel_collar = swivel.get_visual("swivel_collar")
    swivel_post = swivel.get_visual("swivel_post")
    pivot_lug_left = swivel.get_visual("pivot_lug_left")
    pivot_lug_right = swivel.get_visual("pivot_lug_right")
    lower_hub = lower_arm.get_visual("lower_hub")
    lower_yoke_left = lower_arm.get_visual("lower_yoke_left")
    lower_yoke_right = lower_arm.get_visual("lower_yoke_right")
    upper_hub = upper_arm.get_visual("upper_hub")
    upper_yoke_left = upper_arm.get_visual("upper_yoke_left")
    upper_yoke_right = upper_arm.get_visual("upper_yoke_right")
    head_hub = head.get_visual("head_hub")
    shade_shell = head.get_visual("shade_shell")
    bulb = head.get_visual("bulb")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Proximal hub of the lower arm sits directly above (and slightly embeds
    # into) the top of the swivel column.
    ctx.allow_overlap(
        lower_arm,
        swivel,
        reason="The lower arm proximal hub seats on top of the swivel column.",
        elem_a=lower_hub,
        elem_b=swivel_post,
    )

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=96)

    # ---- Realistic size + grounding ----
    base_aabb = ctx.part_world_aabb(base)
    if base_aabb is not None:
        bmin, bmax = base_aabb
        ctx.check(
            "base_grounded_on_desk",
            abs(bmin[2]) <= 1e-5,
            f"base felt should rest on z=0, got {bmin[2]:.6f}",
        )
        diameter = max(bmax[0] - bmin[0], bmax[1] - bmin[1])
        ctx.check(
            "base_diameter_realistic",
            0.18 <= diameter <= 0.28,
            f"weighted base diameter {diameter:.3f} m out of realistic range",
        )

    # ---- Articulation contract ----
    ctx.check(
        "base_swivel_is_continuous_vertical",
        base_to_swivel.articulation_type == ArticulationType.CONTINUOUS
        and tuple(base_to_swivel.axis) == (0.0, 0.0, 1.0),
        "base_to_swivel must be a CONTINUOUS joint about the +Z vertical axis",
    )
    for joint, name in (
        (swivel_to_lower, "swivel_to_lower"),
        (lower_to_upper, "lower_to_upper"),
        (upper_to_head, "upper_to_head"),
    ):
        ctx.check(
            f"{name}_is_revolute_about_y",
            joint.articulation_type == ArticulationType.REVOLUTE
            and tuple(joint.axis) == (0.0, 1.0, 0.0),
            f"{name} must be a REVOLUTE joint about the +Y axis",
        )

    # ---- Mounting: swivel collar seats on the base cap ----
    ctx.expect_contact(
        swivel,
        base,
        elem_a=swivel_collar,
        elem_b=base_cap,
        name="swivel_collar_rests_on_base_cap",
    )
    ctx.expect_origin_distance(
        swivel,
        base,
        axes="xy",
        max_dist=0.001,
        name="swivel_axis_centered_on_base",
    )
    ctx.expect_within(
        swivel,
        base,
        axes="xy",
        inner_elem=swivel_collar,
        outer_elem=base_disk,
        name="swivel_footprint_within_base_disk",
    )

    # ---- Lower arm hub seats between the two pivot lugs at the top of the swivel ----
    ctx.expect_contact(
        lower_arm,
        swivel,
        elem_a=lower_hub,
        elem_b=pivot_lug_left,
        name="lower_arm_hinges_on_left_pivot_lug",
    )
    ctx.expect_contact(
        lower_arm,
        swivel,
        elem_a=lower_hub,
        elem_b=pivot_lug_right,
        name="lower_arm_hinges_on_right_pivot_lug",
    )

    # ---- Elbow and head hubs sit between the previous arm's yokes ----
    ctx.expect_contact(
        upper_arm,
        lower_arm,
        elem_a=upper_hub,
        elem_b=lower_yoke_left,
        name="elbow_hub_meets_lower_left_yoke",
    )
    ctx.expect_contact(
        upper_arm,
        lower_arm,
        elem_a=upper_hub,
        elem_b=lower_yoke_right,
        name="elbow_hub_meets_lower_right_yoke",
    )
    ctx.expect_contact(
        head,
        upper_arm,
        elem_a=head_hub,
        elem_b=upper_yoke_left,
        name="head_hub_meets_upper_left_yoke",
    )
    ctx.expect_contact(
        head,
        upper_arm,
        elem_a=head_hub,
        elem_b=upper_yoke_right,
        name="head_hub_meets_upper_right_yoke",
    )

    # ---- Shade encloses the bulb ----
    ctx.expect_overlap(
        head,
        head,
        axes="yz",
        min_overlap=0.020,
        elem_a=bulb,
        elem_b=shade_shell,
        name="bulb_sheltered_by_shade",
    )
    # Shade projects forward of the head pivot.
    ctx.expect_gap(
        head,
        head,
        axis="x",
        min_gap=0.015,
        positive_elem=shade_shell,
        negative_elem=head_hub,
        name="conical_shade_projects_forward_of_head_pivot",
    )

    # ---- Task pose: arms fold to a working position above the desk ----
    task_pose = {
        swivel_to_lower: 0.50,
        lower_to_upper: -0.80,
        upper_to_head: -0.20,
    }
    with ctx.pose(task_pose):
        ctx.expect_contact(
            head,
            upper_arm,
            elem_a=head_hub,
            elem_b=upper_yoke_left,
            name="task_pose_head_keeps_left_yoke_contact",
        )
        ctx.expect_contact(
            head,
            upper_arm,
            elem_a=head_hub,
            elem_b=upper_yoke_right,
            name="task_pose_head_keeps_right_yoke_contact",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="task_pose_no_overlap")

    # ---- Raised pose: arms reach high, head tilted back ----
    raised_pose = {
        swivel_to_lower: -0.70,
        lower_to_upper: -1.20,
        upper_to_head: -0.50,
    }
    with ctx.pose(raised_pose):
        ctx.expect_contact(
            upper_arm,
            lower_arm,
            elem_a=upper_hub,
            elem_b=lower_yoke_left,
            name="raised_pose_elbow_keeps_left_yoke_contact",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="raised_pose_no_overlap")

    # ---- Swivel pan moves the head sideways ----
    rest_shade_center = _aabb_center(ctx.part_element_world_aabb(head, elem=shade_shell))
    with ctx.pose({base_to_swivel: math.pi * 0.5}):
        rotated_shade_center = _aabb_center(ctx.part_element_world_aabb(head, elem=shade_shell))
        if rest_shade_center is None or rotated_shade_center is None:
            ctx.fail("swivel_pan_measurable", "could not measure shade center in rest or panned pose")
        else:
            dx = rotated_shade_center[0] - rest_shade_center[0]
            dy = rotated_shade_center[1] - rest_shade_center[1]
            ctx.check(
                "swivel_pan_moves_shade_to_side",
                abs(dy) > 0.10 and abs(dx) > 0.05,
                f"shade should move sideways after swivel; got dx={dx:.3f}, dy={dy:.3f}",
            )

    return ctx.report()


def _aabb_center(aabb):
    if aabb is None:
        return None
    low, high = aabb
    return (
        (low[0] + high[0]) * 0.5,
        (low[1] + high[1]) * 0.5,
        (low[2] + high[2]) * 0.5,
    )


# >>> USER_CODE_END

object_model = build_object_model()
