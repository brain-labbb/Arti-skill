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
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Widespread two-handle wall-mounted faucet in polished gold brass.
# Variant 25: lever handles with stem collars and hot/cold cap disks.
#
# Frame conventions:
#   - The wall is the vertical XZ plane at y = 0 (wall slab occupies y > 0).
#   - The faucet projects out of the wall along -Y (toward the viewer).
#     Sub-assemblies are authored in a local "+Y out of wall" frame and
#     mounted with a yaw of pi, so viewer-left is world -X.
#   - The spout drops toward -Z; the wall panel base sits on the floor (z = 0).
#   - Spout flange and both valve assemblies are centered at z = SPOUT_AXIS_Z.
# ---------------------------------------------------------------------------

# Layout
SPOUT_AXIS_Z = 0.20  # height of the spout / valve axes
VALVE_PITCH_X = 0.12  # valve centers at x = +/- 0.12

# Wall panel (mounting substrate)
WALL_W = 0.38
WALL_T = 0.012
WALL_H = 0.32

# Spout
SPOUT_TUBE_R = 0.015  # outer radius (0.03 m diameter per prompt)
SPOUT_BORE_R = 0.0105  # inner bore radius (visible at the outlet)
SPOUT_STRAIGHT = 0.16  # straight horizontal run before the bend
SPOUT_BEND_R = 0.06  # bend radius of the downward curve
SPOUT_DROP_Z = -0.105  # outlet end below the spout axis

# Spout escutcheon flange (stepped, ~0.07 m diameter per prompt)
FLANGE_R1, FLANGE_T1 = 0.035, 0.010
FLANGE_R2, FLANGE_T2 = 0.026, 0.010

# Valve assemblies
VALVE_ESC_R1, VALVE_ESC_T1 = 0.033, 0.010
VALVE_ESC_R2, VALVE_ESC_T2 = 0.026, 0.010
VALVE_BODY_R = 0.0145
VALVE_BODY_FRONT_Y = 0.052  # front face of the valve body (joint plane)

# Stem collar (visible ring under each handle)
COLLAR_R = 0.018
COLLAR_LEN = 0.008

# Lever handle
HUB_R = 0.0130
HUB_LEN = 0.022
LEVER_ARM_LEN = 0.065  # lever arm length from hub center
LEVER_ARM_W = 0.012  # arm width
LEVER_ARM_H = 0.009  # arm height (thickness)
LEVER_TIP_R = 0.008  # rounded tip sphere radius
STEM_R = 0.007

# Hot/cold cap disks
CAP_R = 0.008
CAP_THICKNESS = 0.003

# Computed by build_object_model() for hollow-bore verification in run_tests().
SPOUT_SOLID_VOLUME: float = 0.0
SPOUT_UNBORED_VOLUME: float = 0.0


def _build_spout_solid() -> cq.Workplane:
    """Spout in its local frame: flange back face on the wall plane (y=0),
    tube axis along +Y, curving down to an open, hollow outlet."""
    mid = (
        SPOUT_STRAIGHT + SPOUT_BEND_R * math.sin(math.pi / 4.0),
        -SPOUT_BEND_R + SPOUT_BEND_R * math.cos(math.pi / 4.0),
    )
    end = (SPOUT_STRAIGHT + SPOUT_BEND_R, -SPOUT_BEND_R)

    # Path in the YZ plane (local x -> world Y, local y -> world Z).
    path = (
        cq.Workplane("YZ")
        .moveTo(0.0, 0.0)
        .lineTo(SPOUT_STRAIGHT, 0.0)
        .threePointArc(mid, end)
        .lineTo(end[0], SPOUT_DROP_Z)
    )
    # Bore path slightly longer on both ends for clean open cuts.
    bore_path = (
        cq.Workplane("YZ")
        .moveTo(-0.004, 0.0)
        .lineTo(SPOUT_STRAIGHT, 0.0)
        .threePointArc(mid, end)
        .lineTo(end[0], SPOUT_DROP_Z - 0.003)
    )

    # "ZX" workplanes have +Y normals, matching the path start tangent.
    tube = cq.Workplane("ZX").circle(SPOUT_TUBE_R).sweep(path)
    bore = (
        cq.Workplane("ZX")
        .workplane(offset=-0.004)
        .circle(SPOUT_BORE_R)
        .sweep(bore_path)
    )
    flange_outer = cq.Workplane("ZX").circle(FLANGE_R1).extrude(FLANGE_T1)
    flange_step = (
        cq.Workplane("ZX")
        .workplane(offset=FLANGE_T1)
        .circle(FLANGE_R2)
        .extrude(FLANGE_T2)
    )

    unbored = tube.union(flange_outer).union(flange_step)
    solid = unbored.cut(bore)

    global SPOUT_SOLID_VOLUME, SPOUT_UNBORED_VOLUME
    SPOUT_SOLID_VOLUME = solid.val().Volume()
    SPOUT_UNBORED_VOLUME = unbored.val().Volume()
    return solid


def _build_lever_handle_solid() -> cq.Workplane:
    """Lever handle in the handle frame: axis +Y, back face at y=0.
    A cylindrical hub with a single lever arm extending along +Z (upward)
    in the neutral position. The arm sweeps in the XZ plane when rotated."""
    hub_mid_y = HUB_LEN / 2.0

    # Hub body (cylinder along Y axis, built on ZX workplane so circle is in XZ)
    hub = cq.Workplane("ZX").circle(HUB_R).extrude(HUB_LEN)

    # Lever arm: extends upward (+Z) from inside the hub to above it.
    # Start the arm 4mm below the hub top surface to ensure solid connectivity.
    arm_start_z = 0.0  # arm starts at hub center (z=0), well inside the hub
    arm_total_len = LEVER_ARM_LEN + HUB_R  # extends from z=0 to z=HUB_R+LEVER_ARM_LEN
    arm_center_z = arm_start_z + arm_total_len / 2.0
    arm_solid = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, hub_mid_y, arm_center_z))
        .box(LEVER_ARM_H, LEVER_ARM_W, arm_total_len)
    )

    # Tip sphere at the end of the arm
    tip_z = HUB_R + LEVER_ARM_LEN
    tip = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, hub_mid_y, tip_z))
        .sphere(LEVER_TIP_R)
    )

    result = hub.union(arm_solid).union(tip)
    return result


def _build_stem_collar_solid() -> cq.Workplane:
    """Stem collar: a wider ring that sits between valve body and handle.
    Axis along +Y, starts at y=0."""
    collar = cq.Workplane("ZX").circle(COLLAR_R).extrude(COLLAR_LEN)
    # Add a small chamfer ring for visual detail
    ring = (
        cq.Workplane("ZX")
        .workplane(offset=COLLAR_LEN * 0.4)
        .circle(COLLAR_R + 0.002)
        .extrude(COLLAR_LEN * 0.2)
    )
    return collar.union(ring)


def _add_valve_visuals(valve, gold) -> None:
    """Stepped escutcheon + projecting cylindrical valve body, axis +Y from
    the wall plane (valve frame origin sits on the wall face)."""
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


def _add_lever_handle_visuals(handle, lever_mesh, collar_mesh, gold, hot_mat, cold_mat, side: str) -> None:
    """Lever handle with stem, lever arm, and hot/cold cap disk.
    Joint frame at the valve body front face, axis along valve stem."""
    # Stem turns with the handle; it seats back into the valve body bore.
    handle.visual(
        Cylinder(radius=STEM_R, length=0.016),
        origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="stem",
    )
    # Lever handle body (hub + arm + tip)
    handle.visual(lever_mesh, material=gold, name="lever_body")

    # Hot/cold cap disk on top of the hub, slightly embedded for connectivity
    cap_y = HUB_LEN - 0.001  # embed 1mm into hub face for physical contact
    cap_mat = hot_mat if side == "left" else cold_mat
    handle.visual(
        Cylinder(radius=CAP_R, length=CAP_THICKNESS),
        origin=Origin(
            xyz=(0.0, cap_y + CAP_THICKNESS / 2.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=cap_mat,
        name="cap_disk",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_lever_faucet")

    gold = model.material("polished_gold_brass", rgba=(0.85, 0.66, 0.20, 1.0))
    wall_white = model.material("wall_white", rgba=(0.93, 0.93, 0.90, 1.0))
    hot_red = model.material("hot_indicator_red", rgba=(0.80, 0.12, 0.12, 1.0))
    cold_blue = model.material("cold_indicator_blue", rgba=(0.12, 0.25, 0.75, 1.0))

    # --- wall panel (root, mounting substrate) ---
    wall = model.part("wall_panel")
    wall.visual(
        Box((WALL_W, WALL_T, WALL_H)),
        origin=Origin(xyz=(0.0, WALL_T / 2.0, WALL_H / 2.0)),
        material=wall_white,
        name="panel",
    )

    # --- central spout (fixed) ---
    spout = model.part("spout")
    spout.visual(mesh_from_cadquery(_build_spout_solid(), "spout"), material=gold, name="tube")
    model.articulation(
        "wall_to_spout",
        ArticulationType.FIXED,
        parent=wall,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, SPOUT_AXIS_Z), rpy=(0.0, 0.0, math.pi)),
    )

    # --- valve assemblies (fixed) and lever handles (revolute) ---
    lever_mesh = mesh_from_cadquery(_build_lever_handle_solid(), "lever_handle")
    collar_mesh = mesh_from_cadquery(_build_stem_collar_solid(), "stem_collar")

    for side, sx in (("left", -1.0), ("right", 1.0)):
        valve = model.part(f"{side}_valve")
        _add_valve_visuals(valve, gold)

        # Stem collar: fixed trim ring on the valve, visible between body and handle.
        # The CadQuery mesh is already axis-aligned along +Y; just translate to front of body.
        valve.visual(
            collar_mesh,
            origin=Origin(xyz=(0.0, VALVE_BODY_FRONT_Y - COLLAR_LEN, 0.0)),
            material=gold,
            name="stem_collar",
        )

        model.articulation(
            f"wall_to_{side}_valve",
            ArticulationType.FIXED,
            parent=wall,
            child=valve,
            origin=Origin(
                xyz=(sx * VALVE_PITCH_X, 0.0, SPOUT_AXIS_Z),
                rpy=(0.0, 0.0, math.pi),
            ),
        )

        handle = model.part(f"{side}_lever_handle")
        _add_lever_handle_visuals(handle, lever_mesh, collar_mesh, gold, hot_red, cold_blue, side)
        # Left: axis (0,+1,0) → positive q sweeps arm leftward (away from spout).
        # Right: axis (0,-1,0) → positive q sweeps arm rightward (away from spout).
        # Both represent "forward lever opens" symmetrically.
        handle_axis = (0.0, 1.0, 0.0) if side == "left" else (0.0, -1.0, 0.0)
        model.articulation(
            f"{side}_handle_spindle",
            ArticulationType.REVOLUTE,
            parent=valve,
            child=handle,
            # Joint frame at the valve body front face; axis projects out of
            # the wall (horizontal, perpendicular to the wall plane).
            origin=Origin(xyz=(0.0, VALVE_BODY_FRONT_Y, 0.0)),
            axis=handle_axis,
            motion_limits=MotionLimits(
                effort=5.0, velocity=3.0, lower=-math.pi / 4.0, upper=math.pi / 2.0
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wall = object_model.get_part("wall_panel")
    spout = object_model.get_part("spout")
    left_valve = object_model.get_part("left_valve")
    right_valve = object_model.get_part("right_valve")
    left_handle = object_model.get_part("left_lever_handle")
    right_handle = object_model.get_part("right_lever_handle")
    left_joint = object_model.get_articulation("left_handle_spindle")
    right_joint = object_model.get_articulation("right_handle_spindle")

    # --- joint plan: two independent revolute lever handles, axis perpendicular
    # to wall, range -45..+90 deg (forward-back lever rotation) ---
    for joint, expected_axis in (
        (left_joint, (0.0, 1.0, 0.0)),
        (right_joint, (0.0, -1.0, 0.0)),
    ):
        ctx.check(
            f"{joint.name}_revolute",
            str(joint.joint_type).lower().endswith("revolute"),
            f"type={joint.joint_type}",
        )
        ax = joint.axis
        ctx.check(
            f"{joint.name}_axis_perpendicular_to_wall",
            all(abs(ax[i] - expected_axis[i]) < 1e-9 for i in range(3)),
            f"axis={ax}, expected={expected_axis}",
        )
        lim = joint.motion_limits
        ctx.check(
            f"{joint.name}_lever_range",
            lim is not None
            and abs(lim.lower + math.pi / 4.0) < 1e-6
            and abs(lim.upper - math.pi / 2.0) < 1e-6,
            f"limits=({lim.lower}, {lim.upper})",
        )

    # --- intentional embedding: handle stems seat into the valve body bores ---
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
    # Stem collar wraps around the stem at the valve body front face.
    ctx.allow_overlap(
        left_handle,
        left_valve,
        elem_a=left_handle.get_visual("stem"),
        elem_b=left_valve.get_visual("stem_collar"),
        reason="stem collar is a trim ring that wraps around the handle stem at the valve front face",
    )
    ctx.allow_overlap(
        right_handle,
        right_valve,
        elem_a=right_handle.get_visual("stem"),
        elem_b=right_valve.get_visual("stem_collar"),
        reason="stem collar is a trim ring that wraps around the handle stem at the valve front face",
    )

    # --- spout geometry: hollow bore, reach, downward outlet ---
    ctx.check(
        "spout_tube_is_hollow",
        0.0 < SPOUT_SOLID_VOLUME < 0.97 * SPOUT_UNBORED_VOLUME,
        f"solid={SPOUT_SOLID_VOLUME:.3e} m^3 vs unbored={SPOUT_UNBORED_VOLUME:.3e} m^3",
    )
    spout_aabb = ctx.part_world_aabb(spout)
    assert spout_aabb is not None
    (sx0, sy0, sz0), (sx1, sy1, sz1) = spout_aabb
    ctx.check(
        "spout_projects_about_0p22_from_wall",
        -0.26 <= sy0 <= -0.20 and abs(sy1) < 0.005,
        f"spout y extent=({sy0:.3f}, {sy1:.3f})",
    )
    ctx.check(
        "spout_outlet_drops_about_0p10_below_axis",
        0.08 <= (SPOUT_AXIS_Z - sz0) <= 0.13,
        f"spout zmin={sz0:.3f}, axis z={SPOUT_AXIS_Z}",
    )
    # Flange seats against the wall plane (wall slab is on the +Y side).
    ctx.expect_gap(wall, spout, axis="y", max_gap=0.0005, max_penetration=0.0005)

    # --- valve placement: flanking the spout at x = +/-0.10, same height ---
    lv = ctx.part_world_position(left_valve)
    rv = ctx.part_world_position(right_valve)
    assert lv is not None and rv is not None
    ctx.check(
        "valves_flank_spout_symmetrically",
        abs(lv[0] + VALVE_PITCH_X) < 1e-6
        and abs(rv[0] - VALVE_PITCH_X) < 1e-6
        and abs(lv[2] - SPOUT_AXIS_Z) < 1e-6
        and abs(rv[2] - SPOUT_AXIS_Z) < 1e-6,
        f"left={lv}, right={rv}",
    )
    ctx.expect_gap(wall, left_valve, axis="y", max_gap=0.0005, max_penetration=0.0005)
    ctx.expect_gap(wall, right_valve, axis="y", max_gap=0.0005, max_penetration=0.0005)

    # --- stem collars present on each valve ---
    for valve_part, valve_name in ((left_valve, "left"), (right_valve, "right")):
        collar_vis = valve_part.get_visual("stem_collar")
        ctx.check(
            f"{valve_name}_stem_collar_exists",
            collar_vis is not None,
            f"stem_collar visual not found on {valve_name}_valve",
        )

    # --- hot/cold cap disks present as geometry ---
    left_cap = left_handle.get_visual("cap_disk")
    right_cap = right_handle.get_visual("cap_disk")
    ctx.check("left_hot_cap_disk_exists", left_cap is not None, "cap_disk not found on left handle")
    ctx.check("right_cold_cap_disk_exists", right_cap is not None, "cap_disk not found on right handle")

    # Cap disks should be distinct (separate geometry on each handle)
    ctx.check(
        "cap_disks_are_separate_geometry",
        left_cap is not right_cap,
        "hot and cold cap disks should be separate visual instances",
    )

    # --- lever handle mounted on valve body, not floating ---
    ctx.expect_overlap(left_handle, left_valve, axes="xz", min_overlap=0.01)
    ctx.expect_overlap(right_handle, right_valve, axes="xz", min_overlap=0.01)

    # --- lever handle has a distinct arm extending upward ---
    lh_aabb = ctx.part_world_aabb(left_handle)
    assert lh_aabb is not None
    (hx0, hy0, hz0), (hx1, hy1, hz1) = lh_aabb
    handle_x_span = hx1 - hx0
    handle_z_span = hz1 - hz0
    ctx.check(
        "lever_handle_arm_extends_upward",
        handle_z_span > 0.06,
        f"lever arm z span={handle_z_span:.3f} (should be > 0.06 for upward arm)",
    )

    # --- overall width: lever handles flanking spout ---
    rh_aabb = ctx.part_world_aabb(right_handle)
    assert rh_aabb is not None
    total_w = rh_aabb[1][0] - lh_aabb[0][0]
    ctx.check(
        "overall_width_widespread_layout",
        0.25 <= total_w <= 0.40,
        f"handle-to-handle width={total_w:.3f}",
    )

    # --- lever rotation: prove the handle actually rotates forward-back ---
    rest_pos = ctx.part_world_position(left_handle)
    assert rest_pos is not None

    with ctx.pose({left_joint: math.pi / 4.0}):
        rot_pos = ctx.part_world_position(left_handle)
        assert rot_pos is not None
        # At +45° rotation the handle origin stays fixed (revolute joint)
        ctx.check(
            "left_lever_origin_fixed_during_rotation",
            abs(rot_pos[0] - rest_pos[0]) < 1e-6
            and abs(rot_pos[1] - rest_pos[1]) < 1e-6
            and abs(rot_pos[2] - rest_pos[2]) < 1e-6,
            f"rest={rest_pos}, rotated={rot_pos}",
        )
        # The lever arm should have rotated: z extent shrinks as arm tilts
        rot_aabb = ctx.part_world_aabb(left_handle)
        assert rot_aabb is not None
        rot_z_span = rot_aabb[1][2] - rot_aabb[0][2]
        ctx.check(
            "left_lever_arm_tilts_from_vertical",
            rot_z_span < handle_z_span - 0.005,
            f"rest_z_span={handle_z_span:.3f}, rotated_z_span={rot_z_span:.3f}",
        )
        # Handle stays on its valve
        ctx.expect_overlap(left_handle, left_valve, axes="xz", min_overlap=0.005)

    with ctx.pose({right_joint: math.pi / 3.0}):
        ctx.expect_overlap(right_handle, right_valve, axes="xz", min_overlap=0.005)
        ctx.expect_gap(right_handle, spout, axis="x", min_gap=0.01)

    # --- wall panel grounded; faucet hardware at realistic mounting height ---
    wall_aabb = ctx.part_world_aabb(wall)
    assert wall_aabb is not None
    ctx.check(
        "wall_panel_grounded",
        abs(wall_aabb[0][2]) < 1e-6,
        f"wall zmin={wall_aabb[0][2]:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
