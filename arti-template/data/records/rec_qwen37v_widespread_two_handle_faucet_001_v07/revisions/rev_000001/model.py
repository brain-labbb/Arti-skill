from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Widespread two-handle bathroom faucet in polished gold brass.
# Variant of wall-mounted faucet with:
#   - High curved swan neck spout
#   - Continuous vertical swivel joint on the spout
#   - Visible stem collars under each handle
#   - Separate hot and cold cap disks
#
# Frame conventions:
#   - The wall is the vertical XZ plane at y = 0 (wall slab occupies y > 0).
#   - The faucet projects out of the wall along -Y (toward the viewer).
#   - The spout rises toward +Z then arches outward and down.
#   - Valve assemblies are centered at z = VALVE_Z.
# ---------------------------------------------------------------------------

# Layout
VALVE_Z = 0.18  # height of valve centers
VALVE_PITCH_X = 0.11  # valve centers at x = +/- 0.11

# Wall panel (mounting substrate)
WALL_W = 0.40
WALL_T = 0.012
WALL_H = 0.34

# Swan neck spout
SPOUT_TUBE_R = 0.014  # outer radius
SPOUT_BORE_R = 0.010  # inner bore radius
SPOUT_RISE_Z = 0.16  # height the neck rises above mount point
SPOUT_REACH_Y = 0.16  # horizontal reach from wall
SPOUT_DROP_Z = 0.06  # outlet drops below the peak
SPOUT_BASE_R = 0.020  # base collar radius
SPOUT_BASE_LEN = 0.018  # base collar length

# Valve assemblies
VALVE_ESC_R1, VALVE_ESC_T1 = 0.030, 0.008
VALVE_ESC_R2, VALVE_ESC_T2 = 0.024, 0.008
VALVE_BODY_R = 0.013
VALVE_BODY_FRONT_Y = 0.048

# Stem collar
COLLAR_R = 0.016
COLLAR_T = 0.008

# Cross handle
HANDLE_ROD_R = 0.004
HANDLE_ROD_LEN = 0.095
HANDLE_ROD_PLANE_Y = 0.012
HUB_R = 0.012
HUB_LEN = 0.022
KNURL_R = 0.0135
STEM_R = 0.006

# Cap disks
CAP_R = 0.009
CAP_T = 0.003

# Computed for hollow-bore verification
SPOUT_SOLID_VOLUME: float = 0.0
SPOUT_UNBORED_VOLUME: float = 0.0


def _build_swan_neck_solid() -> cq.Workplane:
    """Swan neck spout in its local frame: base at origin, axis along +Y
    (out from wall), rising in +Z then arching over and dropping down."""
    # Path points in YZ plane (local: y->worldY after mount transform, z->worldZ)
    # The spout rises steeply, arches over, then drops to outlet
    pts = [
        (0.0, 0.0),       # start at mount
        (0.02, 0.06),     # initial rise
        (0.06, 0.12),     # rising and moving outward
        (0.10, SPOUT_RISE_Z),  # peak of arch
        (0.14, SPOUT_RISE_Z - 0.02),  # past peak, starting to drop
        (SPOUT_REACH_Y, SPOUT_RISE_Z - SPOUT_DROP_Z),  # outlet position
    ]

    # Build smooth spline path
    path = cq.Workplane("YZ").spline(pts)

    # Bore path (slightly extended for clean cuts)
    bore_pts = [
        (-0.003, -0.003),
        (0.02, 0.058),
        (0.06, 0.118),
        (0.10, SPOUT_RISE_Z - 0.001),
        (0.14, SPOUT_RISE_Z - 0.022),
        (SPOUT_REACH_Y, SPOUT_RISE_Z - SPOUT_DROP_Z - 0.004),
    ]

    bore_path = cq.Workplane("YZ").spline(bore_pts)

    # Sweep tube along path
    tube = cq.Workplane("ZX").circle(SPOUT_TUBE_R).sweep(path)
    bore = (
        cq.Workplane("ZX")
        .workplane(offset=-0.003)
        .circle(SPOUT_BORE_R)
        .sweep(bore_path)
    )

    solid = tube.cut(bore)

    global SPOUT_SOLID_VOLUME, SPOUT_UNBORED_VOLUME
    SPOUT_SOLID_VOLUME = solid.val().Volume()
    SPOUT_UNBORED_VOLUME = tube.val().Volume()
    return solid


def _build_hub_solid() -> cq.Workplane:
    """Cross-handle central hub: axis +Y, back face at y=0, with knurled
    middle band and a domed front cap."""
    hub = cq.Workplane("ZX").circle(HUB_R).extrude(HUB_LEN)
    knurl = (
        cq.Workplane("ZX")
        .workplane(offset=0.006)
        .polygon(16, 2.0 * KNURL_R)
        .extrude(0.010)
    )
    dome = cq.Workplane("ZX").workplane(offset=0.015).sphere(0.011)
    return hub.union(knurl).union(dome)


def _add_valve_visuals(valve, gold) -> None:
    """Stepped escutcheon + projecting cylindrical valve body, axis +Y from
    the wall plane."""
    valve.visual(
        Cylinder(radius=VALVE_ESC_R1, length=VALVE_ESC_T1),
        origin=Origin(xyz=(0.0, VALVE_ESC_T1 / 2.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="escutcheon_base",
    )
    valve.visual(
        Cylinder(radius=VALVE_ESC_R2, length=VALVE_ESC_T2),
        origin=Origin(
            xyz=(0.0, VALVE_ESC_T1 + VALVE_ESC_T2 / 2.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=gold,
        name="escutcheon_step",
    )
    body_len = VALVE_BODY_FRONT_Y - (VALVE_ESC_T1 + VALVE_ESC_T2)
    valve.visual(
        Cylinder(radius=VALVE_BODY_R, length=body_len),
        origin=Origin(
            xyz=(0.0, VALVE_ESC_T1 + VALVE_ESC_T2 + body_len / 2.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=gold,
        name="valve_body",
    )


def _add_stem_collar(valve, gold) -> None:
    """Visible stem collar ring at the front of the valve body."""
    collar_y = VALVE_BODY_FRONT_Y + COLLAR_T / 2.0
    valve.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_T),
        origin=Origin(
            xyz=(0.0, collar_y, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=gold,
        name="stem_collar",
    )


def _add_handle_visuals(handle, hub_mesh, gold) -> None:
    """Four-arm cross handle with stem, hub, spokes, and rounded tips."""
    handle.visual(
        Cylinder(radius=STEM_R, length=0.014),
        origin=Origin(xyz=(0.0, -0.005, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="stem",
    )
    handle.visual(hub_mesh, material=gold, name="hub")
    # Vertical spoke rod
    handle.visual(
        Cylinder(radius=HANDLE_ROD_R, length=HANDLE_ROD_LEN),
        origin=Origin(xyz=(0.0, HANDLE_ROD_PLANE_Y, 0.0)),
        material=gold,
        name="vertical_spokes",
    )
    # Horizontal spoke rod
    handle.visual(
        Cylinder(radius=HANDLE_ROD_R, length=HANDLE_ROD_LEN),
        origin=Origin(xyz=(0.0, HANDLE_ROD_PLANE_Y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gold,
        name="horizontal_spokes",
    )
    # Rounded spoke tips
    half = HANDLE_ROD_LEN / 2.0
    for name, (dx, dz) in (
        ("tip_top", (0.0, half)),
        ("tip_bottom", (0.0, -half)),
        ("tip_outer", (half, 0.0)),
        ("tip_inner", (-half, 0.0)),
    ):
        handle.visual(
            Sphere(radius=HANDLE_ROD_R),
            origin=Origin(xyz=(dx, HANDLE_ROD_PLANE_Y, dz)),
            material=gold,
            name=name,
        )


def _add_cap_disk(handle, material, name) -> None:
    """Small indicator cap disk on the front face of the handle hub."""
    cap_y = HUB_LEN + CAP_T / 2.0
    handle.visual(
        Cylinder(radius=CAP_R, length=CAP_T),
        origin=Origin(
            xyz=(0.0, cap_y, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_swan_neck_faucet")

    gold = model.material("polished_gold_brass", rgba=(0.85, 0.66, 0.20, 1.0))
    wall_white = model.material("wall_white", rgba=(0.93, 0.93, 0.90, 1.0))
    hot_red = model.material("hot_indicator_red", rgba=(0.80, 0.15, 0.15, 1.0))
    cold_blue = model.material("cold_indicator_blue", rgba=(0.15, 0.30, 0.80, 1.0))

    # --- wall panel (root, mounting substrate) ---
    wall = model.part("wall_panel")
    wall.visual(
        Box((WALL_W, WALL_T, WALL_H)),
        origin=Origin(xyz=(0.0, WALL_T / 2.0, WALL_H / 2.0)),
        material=wall_white,
        name="panel",
    )

    # --- spout base collar (fixed to wall) ---
    spout_base = model.part("spout_base")
    spout_base.visual(
        Cylinder(radius=SPOUT_BASE_R, length=SPOUT_BASE_LEN),
        origin=Origin(
            xyz=(0.0, SPOUT_BASE_LEN / 2.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=gold,
        name="base_collar",
    )
    model.articulation(
        "wall_to_spout_base",
        ArticulationType.FIXED,
        parent=wall,
        child=spout_base,
        origin=Origin(xyz=(0.0, 0.0, VALVE_Z), rpy=(0.0, 0.0, math.pi)),
    )

    # --- swan neck spout (continuous swivel on vertical axis) ---
    spout_neck = model.part("spout_neck")
    spout_neck.visual(
        mesh_from_cadquery(_build_swan_neck_solid(), "swan_neck"),
        material=gold,
        name="neck_tube",
    )
    model.articulation(
        "spout_swivel",
        ArticulationType.CONTINUOUS,
        parent=spout_base,
        child=spout_neck,
        # Joint at front of base collar; vertical axis (Z in local frame)
        origin=Origin(xyz=(0.0, SPOUT_BASE_LEN, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=3.0),
    )

    # --- valve assemblies (fixed) and cross handles (revolute) ---
    hub_mesh = mesh_from_cadquery(_build_hub_solid(), "handle_hub")
    for side, sx in (("left", -1.0), ("right", 1.0)):
        valve = model.part(f"{side}_valve")
        _add_valve_visuals(valve, gold)
        _add_stem_collar(valve, gold)
        model.articulation(
            f"wall_to_{side}_valve",
            ArticulationType.FIXED,
            parent=wall,
            child=valve,
            origin=Origin(
                xyz=(sx * VALVE_PITCH_X, 0.0, VALVE_Z),
                rpy=(0.0, 0.0, math.pi),
            ),
        )

        handle = model.part(f"{side}_cross_handle")
        _add_handle_visuals(handle, hub_mesh, gold)
        # Add hot/cold cap disk
        cap_mat = hot_red if side == "left" else cold_blue
        cap_name = f"{side}_cap_disk"
        _add_cap_disk(handle, cap_mat, cap_name)

        model.articulation(
            f"{side}_handle_spindle",
            ArticulationType.REVOLUTE,
            parent=valve,
            child=handle,
            origin=Origin(xyz=(0.0, VALVE_BODY_FRONT_Y + COLLAR_T, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=5.0, velocity=3.0, lower=-math.pi, upper=math.pi
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wall = object_model.get_part("wall_panel")
    spout_base = object_model.get_part("spout_base")
    spout_neck = object_model.get_part("spout_neck")
    left_valve = object_model.get_part("left_valve")
    right_valve = object_model.get_part("right_valve")
    left_handle = object_model.get_part("left_cross_handle")
    right_handle = object_model.get_part("right_cross_handle")
    left_joint = object_model.get_articulation("left_handle_spindle")
    right_joint = object_model.get_articulation("right_handle_spindle")
    swivel_joint = object_model.get_articulation("spout_swivel")

    # --- spout swivel is a continuous joint with vertical axis ---
    ctx.check(
        "spout_swivel_is_continuous",
        str(swivel_joint.joint_type).lower().endswith("continuous"),
        f"type={swivel_joint.joint_type}",
    )
    swivel_ax = swivel_joint.axis
    ctx.check(
        "spout_swivel_axis_is_vertical",
        abs(swivel_ax[0]) < 1e-9 and abs(swivel_ax[1]) < 1e-9 and abs(swivel_ax[2] - 1.0) < 1e-9,
        f"axis={swivel_ax}",
    )

    # --- swan neck rises high above mount point ---
    neck_aabb = ctx.part_world_aabb(spout_neck)
    assert neck_aabb is not None
    (nx0, ny0, nz0), (nx1, ny1, nz1) = neck_aabb
    peak_height = nz1 - VALVE_Z
    ctx.check(
        "swan_neck_rises_at_least_0p10_above_mount",
        peak_height >= 0.10,
        f"peak above mount={peak_height:.3f}m, mount z={VALVE_Z}",
    )
    ctx.check(
        "swan_neck_outlet_below_peak",
        nz0 < nz1 - 0.03,
        f"zmin={nz0:.3f}, zmax={nz1:.3f}",
    )

    # --- spout is hollow (bore volume less than solid) ---
    ctx.check(
        "spout_tube_is_hollow",
        0.0 < SPOUT_SOLID_VOLUME < 0.95 * SPOUT_UNBORED_VOLUME,
        f"solid={SPOUT_SOLID_VOLUME:.3e} m^3 vs unbored={SPOUT_UNBORED_VOLUME:.3e} m^3",
    )

    # --- spout swivel motion: rotate the neck and confirm AABB changes ---
    rest_aabb = ctx.part_world_aabb(spout_neck)
    assert rest_aabb is not None
    rest_x_span = rest_aabb[1][0] - rest_aabb[0][0]
    with ctx.pose({swivel_joint: math.pi / 2.0}):
        rotated_aabb = ctx.part_world_aabb(spout_neck)
        assert rotated_aabb is not None
        rotated_x_span = rotated_aabb[1][0] - rotated_aabb[0][0]
        # At 90 degrees, the spout reach (originally in Y) projects onto X
        ctx.check(
            "spout_swivels_sideways_on_rotation",
            rotated_x_span > rest_x_span + 0.05,
            f"rest_x_span={rest_x_span:.3f}, rotated_x_span={rotated_x_span:.3f}",
        )

    # --- stem collars exist on both valves ---
    for side, valve in (("left", left_valve), ("right", right_valve)):
        collar = valve.get_visual("stem_collar")
        ctx.check(
            f"{side}_stem_collar_exists",
            collar is not None,
            f"stem_collar visual not found on {side}_valve",
        )

    # --- hot and cold cap disks exist ---
    hot_cap = left_handle.get_visual("left_cap_disk")
    cold_cap = right_handle.get_visual("right_cap_disk")
    ctx.check("hot_cap_disk_exists", hot_cap is not None, "left_cap_disk not found")
    ctx.check("cold_cap_disk_exists", cold_cap is not None, "right_cap_disk not found")

    # --- handle joints: two independent revolute, axis out of wall ---
    for joint in (left_joint, right_joint):
        ctx.check(
            f"{joint.name}_revolute",
            str(joint.joint_type).lower().endswith("revolute"),
            f"type={joint.joint_type}",
        )
        ax = joint.axis
        ctx.check(
            f"{joint.name}_axis_perpendicular_to_wall",
            abs(ax[0]) < 1e-9 and abs(ax[1] - 1.0) < 1e-9 and abs(ax[2]) < 1e-9,
            f"axis={ax}",
        )
        lim = joint.motion_limits
        ctx.check(
            f"{joint.name}_full_turn_range",
            lim is not None
            and abs(lim.lower + math.pi) < 1e-6
            and abs(lim.upper - math.pi) < 1e-6,
            f"limits=({lim.lower}, {lim.upper})",
        )

    # --- intentional overlap: handle stems seat into valve body bores ---
    ctx.allow_overlap(
        left_handle,
        left_valve,
        elem_a=left_handle.get_visual("stem"),
        elem_b=left_valve.get_visual("valve_body"),
        reason="valve stem is seated inside the valve body bore and turns with the handle",
    )
    ctx.allow_overlap(
        right_handle,
        right_valve,
        elem_a=right_handle.get_visual("stem"),
        elem_b=right_valve.get_visual("valve_body"),
        reason="valve stem is seated inside the valve body bore and turns with the handle",
    )
    # Stem passes through the stem collar ring
    ctx.allow_overlap(
        left_handle,
        left_valve,
        elem_a=left_handle.get_visual("stem"),
        elem_b=left_valve.get_visual("stem_collar"),
        reason="handle stem passes through the visible stem collar ring",
    )
    ctx.allow_overlap(
        right_handle,
        right_valve,
        elem_a=right_handle.get_visual("stem"),
        elem_b=right_valve.get_visual("stem_collar"),
        reason="handle stem passes through the visible stem collar ring",
    )

    # --- proof: stem-collar fit stays seated ---
    for handle, valve, side in (
        (left_handle, left_valve, "left"),
        (right_handle, right_valve, "right"),
    ):
        ctx.expect_overlap(
            handle,
            valve,
            axes="xz",
            min_overlap=0.005,
            name=f"{side}_handle_stays_seated_on_valve",
        )

    # --- valve placement flanking the spout ---
    lv = ctx.part_world_position(left_valve)
    rv = ctx.part_world_position(right_valve)
    assert lv is not None and rv is not None
    ctx.check(
        "valves_flank_spout_symmetrically",
        abs(lv[0] + VALVE_PITCH_X) < 1e-6
        and abs(rv[0] - VALVE_PITCH_X) < 1e-6
        and abs(lv[2] - VALVE_Z) < 1e-6
        and abs(rv[2] - VALVE_Z) < 1e-6,
        f"left={lv}, right={rv}",
    )

    # --- handle stays on valve while rotating ---
    with ctx.pose({left_joint: math.pi / 4.0}):
        rot_aabb = ctx.part_world_aabb(left_handle)
        assert rot_aabb is not None
        rot_z = rot_aabb[1][2] - rot_aabb[0][2]
        ctx.check(
            "left_handle_spokes_rotate_off_axis",
            rot_z < 0.085,
            f"z extent at q=45deg is {rot_z:.3f}",
        )

    with ctx.pose({right_joint: -math.pi / 2.0}):
        ctx.expect_overlap(right_handle, right_valve, axes="xz", min_overlap=0.01)

    # --- overall width about 0.30 m ---
    lh_aabb = ctx.part_world_aabb(left_handle)
    rh_aabb = ctx.part_world_aabb(right_handle)
    assert lh_aabb is not None and rh_aabb is not None
    total_w = rh_aabb[1][0] - lh_aabb[0][0]
    ctx.check(
        "overall_width_about_0p30",
        0.27 <= total_w <= 0.34,
        f"handle-tip to handle-tip width={total_w:.3f}",
    )

    # --- wall grounded ---
    wall_aabb = ctx.part_world_aabb(wall)
    assert wall_aabb is not None
    ctx.check(
        "wall_panel_grounded",
        abs(wall_aabb[0][2]) < 1e-6,
        f"wall zmin={wall_aabb[0][2]:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
