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
# Widespread two-handle wall-mounted bathroom faucet in polished gold brass.
#
# Frame conventions:
#   - The wall is the vertical XZ plane at y = 0 (wall slab occupies y > 0).
#   - The faucet projects out of the wall along -Y (toward the viewer).
#     Sub-assemblies are authored in a local "+Y out of wall" frame and
#     mounted with a yaw of pi, so viewer-left is world -X.
#   - The spout drops toward -Z; the wall panel base sits on the floor (z = 0).
#   - Spout base and both valve assemblies are centered at z = SPOUT_AXIS_Z.
# ---------------------------------------------------------------------------

# Layout – wider spread for widespread faucet
SPOUT_AXIS_Z = 0.20  # height of the valve axes / spout base mount
VALVE_PITCH_X = 0.15  # valve centers at x = +/- 0.15 (widespread)

# Wall panel (mounting substrate)
WALL_W = 0.44
WALL_T = 0.012
WALL_H = 0.48

# Spout base (vertical riser column)
RISER_R = 0.014
RISER_H = 0.10  # riser height above mount point

# Spout arm (gooseneck / high-arc tube)
SPOUT_TUBE_R = 0.013  # outer radius
SPOUT_BORE_R = 0.009  # inner bore radius (visible at outlet)

# Spout escutcheon flange (stepped, ~0.07 m diameter)
FLANGE_R1, FLANGE_T1 = 0.035, 0.008
FLANGE_R2, FLANGE_T2 = 0.025, 0.008

# Valve assemblies
VALVE_ESC_R1, VALVE_ESC_T1 = 0.033, 0.008
VALVE_ESC_R2, VALVE_ESC_T2 = 0.025, 0.008
VALVE_BODY_R = 0.014
VALVE_BODY_FRONT_Y = 0.048  # front face of the valve body (joint plane)

# Stem collar (visible trim ring under each handle)
COLLAR_R1, COLLAR_T1 = 0.018, 0.006
COLLAR_R2, COLLAR_T2 = 0.015, 0.005

# Cross handle
HANDLE_ROD_R = 0.004
HANDLE_ROD_LEN = 0.095  # tip-to-tip ~0.095 m
HANDLE_ROD_PLANE_Y = 0.012  # rod plane in the handle frame
HUB_R = 0.012
HUB_LEN = 0.024
KNURL_R = 0.0135
STEM_R = 0.006

# Computed by build_object_model() for hollow-bore verification in run_tests().
SPOUT_SOLID_VOLUME: float = 0.0
SPOUT_UNBORED_VOLUME: float = 0.0


def _build_spout_arm_solid() -> cq.Workplane:
    """High-arc gooseneck spout arm in its local frame.
    Origin at the swivel point (top of riser). Arm extends along +Y
    (outward from wall after mount rotation), arcing up and over to
    a downward-facing outlet."""
    # Path in YZ plane using arcs for robustness.
    # The gooseneck: short horizontal lead, arc up, arc over, drop to outlet.
    # On a "YZ" workplane, the 2D coords are (y, z).
    path = (
        cq.Workplane("YZ")
        .moveTo(0.0, 0.0)
        .lineTo(0.02, 0.0)
        # Arc upward: from (0.02, 0) through midpoint to near-vertical
        .threePointArc((0.04, 0.06), (0.05, 0.12))
        # Arc over the peak
        .threePointArc((0.08, 0.155), (0.12, 0.13))
        # Descend to outlet
        .lineTo(0.14, 0.06)
        .lineTo(0.14, -0.02)
    )

    bore_path = (
        cq.Workplane("YZ")
        .moveTo(-0.003, 0.0)
        .lineTo(0.02, 0.0)
        .threePointArc((0.04, 0.06), (0.05, 0.12))
        .threePointArc((0.08, 0.155), (0.12, 0.13))
        .lineTo(0.14, 0.06)
        .lineTo(0.14, -0.025)
    )

    tube = cq.Workplane("ZX").circle(SPOUT_TUBE_R).sweep(path)
    bore = (
        cq.Workplane("ZX")
        .workplane(offset=-0.003)
        .circle(SPOUT_BORE_R)
        .sweep(bore_path)
    )

    unbored = tube
    solid = unbored.cut(bore)

    global SPOUT_SOLID_VOLUME, SPOUT_UNBORED_VOLUME
    SPOUT_SOLID_VOLUME = solid.val().Volume()
    SPOUT_UNBORED_VOLUME = unbored.val().Volume()
    return solid


def _build_hub_solid() -> cq.Workplane:
    """Cross-handle central hub in the handle frame: axis +Y, back face at
    y=0, with a knurled (faceted) middle band and a domed front cap."""
    hub = cq.Workplane("ZX").circle(HUB_R).extrude(HUB_LEN)
    knurl = (
        cq.Workplane("ZX")
        .workplane(offset=0.006)
        .polygon(16, 2.0 * KNURL_R)
        .extrude(0.010)
    )
    dome = cq.Workplane("ZX").workplane(offset=0.016).sphere(0.011)
    return hub.union(knurl).union(dome)


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
    # Stem collar – visible trim ring between valve body and handle
    collar_y_start = VALVE_BODY_FRONT_Y
    valve.visual(
        Cylinder(radius=COLLAR_R1, length=COLLAR_T1),
        origin=Origin(
            xyz=(0.0, collar_y_start + COLLAR_T1 / 2.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=gold,
        name="stem_collar_outer",
    )
    valve.visual(
        Cylinder(radius=COLLAR_R2, length=COLLAR_T2),
        origin=Origin(
            xyz=(0.0, collar_y_start + COLLAR_T1 + COLLAR_T2 / 2.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=gold,
        name="stem_collar_inner",
    )


def _add_handle_visuals(handle, hub_mesh, gold) -> None:
    """Four-arm cross handle in the handle frame (joint frame at the valve
    body front face + collar, axis +Y): knurled hub, two crossing spoke rods
    with rounded sphere tips, and a hidden stem seated into the valve body."""
    # Stem turns with the handle; seats back into the valve body bore.
    handle.visual(
        Cylinder(radius=STEM_R, length=0.014),
        origin=Origin(xyz=(0.0, -0.005, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="stem",
    )
    handle.visual(hub_mesh, material=gold, name="hub")
    # Vertical spoke rod (cylinder default long axis is Z).
    handle.visual(
        Cylinder(radius=HANDLE_ROD_R, length=HANDLE_ROD_LEN),
        origin=Origin(xyz=(0.0, HANDLE_ROD_PLANE_Y, 0.0)),
        material=gold,
        name="vertical_spokes",
    )
    # Horizontal spoke rod: rotate the Z-axis cylinder onto X.
    handle.visual(
        Cylinder(radius=HANDLE_ROD_R, length=HANDLE_ROD_LEN),
        origin=Origin(xyz=(0.0, HANDLE_ROD_PLANE_Y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gold,
        name="horizontal_spokes",
    )
    # Rounded spoke tips.
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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_two_handle_faucet")

    gold = model.material("polished_gold_brass", rgba=(0.85, 0.66, 0.20, 1.0))
    wall_white = model.material("wall_white", rgba=(0.93, 0.93, 0.90, 1.0))

    # --- wall panel (root, mounting substrate) ---
    wall = model.part("wall_panel")
    wall.visual(
        Box((WALL_W, WALL_T, WALL_H)),
        origin=Origin(xyz=(0.0, WALL_T / 2.0, WALL_H / 2.0)),
        material=wall_white,
        name="panel",
    )

    # --- spout base (fixed to wall, contains riser column) ---
    spout_base = model.part("spout_base")
    # Escutcheon flange against wall
    spout_base.visual(
        Cylinder(radius=FLANGE_R1, length=FLANGE_T1),
        origin=Origin(xyz=(0.0, FLANGE_T1 / 2.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="flange_base",
    )
    spout_base.visual(
        Cylinder(radius=FLANGE_R2, length=FLANGE_T2),
        origin=Origin(
            xyz=(0.0, FLANGE_T1 + FLANGE_T2 / 2.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=gold,
        name="flange_step",
    )
    # Vertical riser column
    riser_y = FLANGE_T1 + FLANGE_T2
    spout_base.visual(
        Cylinder(radius=RISER_R, length=RISER_H),
        origin=Origin(
            xyz=(0.0, riser_y, RISER_H / 2.0),
        ),
        material=gold,
        name="riser",
    )
    model.articulation(
        "wall_to_spout_base",
        ArticulationType.FIXED,
        parent=wall,
        child=spout_base,
        origin=Origin(xyz=(0.0, 0.0, SPOUT_AXIS_Z), rpy=(0.0, 0.0, math.pi)),
    )

    # --- spout arm (continuous swivel about vertical axis) ---
    spout = model.part("spout_arm")
    spout.visual(
        mesh_from_cadquery(_build_spout_arm_solid(), "spout_arm"),
        material=gold,
        name="tube",
    )
    # The swivel joint is at the top of the riser. In spout_base local frame:
    # +Y is outward from wall, +Z is up. Top of riser is at (0, riser_y, RISER_H).
    model.articulation(
        "spout_swivel",
        ArticulationType.CONTINUOUS,
        parent=spout_base,
        child=spout,
        origin=Origin(xyz=(0.0, riser_y, RISER_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=3.0),
    )

    # --- valve assemblies (fixed) and cross handles (revolute) ---
    hub_mesh = mesh_from_cadquery(_build_hub_solid(), "handle_hub")
    # Handle joint Y position: past the valve body + stem collars
    handle_joint_y = VALVE_BODY_FRONT_Y + COLLAR_T1 + COLLAR_T2
    for side, sx in (("left", -1.0), ("right", 1.0)):
        valve = model.part(f"{side}_valve")
        _add_valve_visuals(valve, gold)
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

        handle = model.part(f"{side}_cross_handle")
        _add_handle_visuals(handle, hub_mesh, gold)
        model.articulation(
            f"{side}_handle_spindle",
            ArticulationType.REVOLUTE,
            parent=valve,
            child=handle,
            # Joint frame at the front of the stem collar; axis projects out
            # of the wall (horizontal, perpendicular to the wall plane).
            origin=Origin(xyz=(0.0, handle_joint_y, 0.0)),
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
    spout_arm = object_model.get_part("spout_arm")
    left_valve = object_model.get_part("left_valve")
    right_valve = object_model.get_part("right_valve")
    left_handle = object_model.get_part("left_cross_handle")
    right_handle = object_model.get_part("right_cross_handle")
    left_joint = object_model.get_articulation("left_handle_spindle")
    right_joint = object_model.get_articulation("right_handle_spindle")
    swivel_joint = object_model.get_articulation("spout_swivel")

    # --- spout swivel: continuous joint about vertical axis ---
    ctx.check(
        "spout_swivel_is_continuous",
        str(swivel_joint.joint_type).lower().endswith("continuous"),
        f"type={swivel_joint.joint_type}",
    )
    swivel_axis = swivel_joint.axis
    ctx.check(
        "spout_swivel_axis_is_vertical",
        abs(swivel_axis[0]) < 1e-9 and abs(swivel_axis[1]) < 1e-9 and abs(swivel_axis[2] - 1.0) < 1e-9,
        f"axis={swivel_axis}",
    )

    # --- spout arm is taller than parent (high-arc gooseneck) ---
    spout_aabb = ctx.part_world_aabb(spout_arm)
    assert spout_aabb is not None
    (sx0, sy0, sz0), (sx1, sy1, sz1) = spout_aabb
    spout_height = sz1 - sz0
    ctx.check(
        "spout_arm_is_tall_high_arc",
        spout_height >= 0.12,
        f"spout arm height={spout_height:.3f} (expected >= 0.12 for high-arc)",
    )
    # Spout arm projects outward from wall
    ctx.check(
        "spout_arm_projects_from_wall",
        sy0 < -0.10,
        f"spout arm min y={sy0:.3f} (should project at least 0.10 from wall)",
    )

    # --- spout hollow bore ---
    ctx.check(
        "spout_tube_is_hollow",
        0.0 < SPOUT_SOLID_VOLUME < 0.95 * SPOUT_UNBORED_VOLUME,
        f"solid={SPOUT_SOLID_VOLUME:.3e} m^3 vs unbored={SPOUT_UNBORED_VOLUME:.3e} m^3",
    )

    # --- spout base is fixed and mounted on wall ---
    ctx.expect_gap(wall, spout_base, axis="y", max_gap=0.001, max_penetration=0.001)

    # --- spout swivel pose: arm rotates about vertical axis ---
    rest_aabb = ctx.part_world_aabb(spout_arm)
    assert rest_aabb is not None
    rest_cx = (rest_aabb[0][0] + rest_aabb[1][0]) / 2.0
    rest_cy = (rest_aabb[0][1] + rest_aabb[1][1]) / 2.0
    rest_cz = (rest_aabb[0][2] + rest_aabb[1][2]) / 2.0
    with ctx.pose({swivel_joint: math.pi / 2.0}):
        rot_aabb = ctx.part_world_aabb(spout_arm)
        assert rot_aabb is not None
        rot_cx = (rot_aabb[0][0] + rot_aabb[1][0]) / 2.0
        rot_cy = (rot_aabb[0][1] + rot_aabb[1][1]) / 2.0
        rot_cz = (rot_aabb[0][2] + rot_aabb[1][2]) / 2.0
        # After 90 degree rotation about Z, the arm center should shift in XY
        ctx.check(
            "spout_swivel_rotates_arm_horizontally",
            abs(rot_cx - rest_cx) > 0.01 or abs(rot_cy - rest_cy) > 0.01,
            f"rest_center=({rest_cx:.3f},{rest_cy:.3f}), rot_center=({rot_cx:.3f},{rot_cy:.3f})",
        )
        # Z center should remain approximately unchanged (rotation about vertical axis)
        ctx.check(
            "spout_swivel_preserves_height",
            abs(rot_cz - rest_cz) < 0.01,
            f"rest_cz={rest_cz:.4f}, rot_cz={rot_cz:.4f}",
        )

    # --- handle joints: two independent revolute cross handles ---
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

    # --- widespread layout: handles spread wider than parent (±0.15) ---
    lv = ctx.part_world_position(left_valve)
    rv = ctx.part_world_position(right_valve)
    assert lv is not None and rv is not None
    ctx.check(
        "valves_widespread_layout",
        abs(lv[0] + VALVE_PITCH_X) < 1e-6
        and abs(rv[0] - VALVE_PITCH_X) < 1e-6
        and abs(lv[2] - SPOUT_AXIS_Z) < 1e-6
        and abs(rv[2] - SPOUT_AXIS_Z) < 1e-6,
        f"left={lv}, right={rv}",
    )
    ctx.check(
        "valves_wider_than_parent",
        (rv[0] - lv[0]) > 0.25,
        f"valve spread={rv[0] - lv[0]:.3f} (parent was 0.20)",
    )

    # --- valves mounted on wall ---
    ctx.expect_gap(wall, left_valve, axis="y", max_gap=0.001, max_penetration=0.001)
    ctx.expect_gap(wall, right_valve, axis="y", max_gap=0.001, max_penetration=0.001)

    # --- stem collars exist on each valve ---
    for side in ("left", "right"):
        valve = object_model.get_part(f"{side}_valve")
        collar_outer = valve.get_visual("stem_collar_outer")
        collar_inner = valve.get_visual("stem_collar_inner")
        ctx.check(
            f"{side}_stem_collar_outer_exists",
            collar_outer is not None,
            "stem_collar_outer visual missing",
        )
        ctx.check(
            f"{side}_stem_collar_inner_exists",
            collar_inner is not None,
            "stem_collar_inner visual missing",
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
    # Stem passes through the collar trim ring (intentional embedding)
    ctx.allow_overlap(
        left_handle,
        left_valve,
        elem_a=left_handle.get_visual("stem"),
        elem_b=left_valve.get_visual("stem_collar_outer"),
        reason="handle stem passes through the fixed stem collar trim ring",
    )
    ctx.allow_overlap(
        right_handle,
        right_valve,
        elem_a=right_handle.get_visual("stem"),
        elem_b=right_valve.get_visual("stem_collar_outer"),
        reason="handle stem passes through the fixed stem collar trim ring",
    )
    ctx.allow_overlap(
        left_handle,
        left_valve,
        elem_a=left_handle.get_visual("stem"),
        elem_b=left_valve.get_visual("stem_collar_inner"),
        reason="handle stem passes through the fixed stem collar inner ring",
    )
    ctx.allow_overlap(
        right_handle,
        right_valve,
        elem_a=right_handle.get_visual("stem"),
        elem_b=right_valve.get_visual("stem_collar_inner"),
        reason="handle stem passes through the fixed stem collar inner ring",
    )
    # Spout arm tube wraps around the riser top at the swivel joint
    ctx.allow_overlap(
        spout_arm,
        spout_base,
        elem_a=spout_arm.get_visual("tube"),
        elem_b=spout_base.get_visual("riser"),
        reason="spout arm tube wraps around the riser top at the swivel pivot connection",
    )
    ctx.expect_contact(spout_arm, spout_base, elem_a="tube", elem_b="riser",
                       name="spout arm contacts riser at swivel")

    # --- cross handle size ---
    lh_aabb = ctx.part_world_aabb(left_handle)
    assert lh_aabb is not None
    (hx0, hy0, hz0), (hx1, hy1, hz1) = lh_aabb
    ctx.check(
        "cross_handle_size_reasonable",
        0.085 <= (hz1 - hz0) <= 0.115 and 0.085 <= (hx1 - hx0) <= 0.115,
        f"handle extents x={hx1 - hx0:.3f}, z={hz1 - hz0:.3f}",
    )
    # Handle mounted on valve (not floating)
    ctx.expect_overlap(left_handle, left_valve, axes="xz", min_overlap=0.008)
    ctx.expect_overlap(right_handle, right_valve, axes="xz", min_overlap=0.008)

    # --- overall width: widespread faucet wider than parent ---
    rh_aabb = ctx.part_world_aabb(right_handle)
    assert rh_aabb is not None
    total_w = rh_aabb[1][0] - lh_aabb[0][0]
    ctx.check(
        "widespread_overall_width",
        0.35 <= total_w <= 0.45,
        f"handle-tip to handle-tip width={total_w:.3f}",
    )

    # --- handle rotation proves real revolute motion ---
    with ctx.pose({left_joint: math.pi / 4.0}):
        rot_aabb = ctx.part_world_aabb(left_handle)
        assert rot_aabb is not None
        rot_z = rot_aabb[1][2] - rot_aabb[0][2]
        ctx.check(
            "left_handle_spokes_rotate_off_axis",
            rot_z < 0.090,
            f"z extent at q=45deg is {rot_z:.3f}",
        )

    with ctx.pose({right_joint: -math.pi / 2.0}):
        ctx.expect_overlap(right_handle, right_valve, axes="xz", min_overlap=0.008)

    # --- wall panel grounded ---
    wall_aabb = ctx.part_world_aabb(wall)
    assert wall_aabb is not None
    ctx.check(
        "wall_panel_grounded",
        abs(wall_aabb[0][2]) < 1e-6,
        f"wall zmin={wall_aabb[0][2]:.4f}",
    )

    # --- at least one non-fixed joint exists ---
    all_joints = [
        object_model.get_articulation(n)
        for n in ["spout_swivel", "left_handle_spindle", "right_handle_spindle"]
    ]
    non_fixed = [j for j in all_joints if not str(j.joint_type).lower().endswith("fixed")]
    ctx.check(
        "has_non_fixed_joints",
        len(non_fixed) >= 1,
        f"found {len(non_fixed)} non-fixed joints",
    )

    return ctx.report()


object_model = build_object_model()
