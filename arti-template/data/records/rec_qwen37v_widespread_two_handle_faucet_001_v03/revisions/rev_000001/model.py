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
# Variant of the wall-mounted faucet with:
#   - cross-shaped handles on separate round bases
#   - visible stem collars under each handle
#   - outlet aerator that pivots downward on a small hinge
#
# Frame conventions:
#   - The wall is the vertical XZ plane at y = 0 (wall slab occupies y > 0).
#   - The faucet projects out of the wall along -Y (toward the viewer).
#   - The spout drops toward -Z; the wall panel base sits on the floor (z = 0).
# ---------------------------------------------------------------------------

# Layout
SPOUT_AXIS_Z = 0.20
VALVE_PITCH_X = 0.10

# Wall panel (mounting substrate)
WALL_W = 0.38
WALL_T = 0.012
WALL_H = 0.32

# Spout
SPOUT_TUBE_R = 0.015
SPOUT_BORE_R = 0.0105
SPOUT_STRAIGHT = 0.16
SPOUT_BEND_R = 0.06
SPOUT_DROP_Z = -0.105
SPOUT_REACH = SPOUT_STRAIGHT + SPOUT_BEND_R

# Spout escutcheon flange
FLANGE_R1, FLANGE_T1 = 0.035, 0.010
FLANGE_R2, FLANGE_T2 = 0.026, 0.010

# Valve assemblies
VALVE_ESC_R1, VALVE_ESC_T1 = 0.033, 0.010
VALVE_ESC_R2, VALVE_ESC_T2 = 0.026, 0.010
VALVE_BODY_R = 0.0145
VALVE_BODY_FRONT_Y = 0.052

# Round bases (separate visible platforms under each handle)
BASE_R = 0.028
BASE_T = 0.008

# Stem collars (visible rings under each handle cross)
COLLAR_R = 0.011
COLLAR_LEN = 0.007

# Cross handle
HANDLE_ROD_R = 0.0045
HANDLE_ROD_LEN = 0.100
HANDLE_ROD_PLANE_Y = 0.013
HUB_R = 0.0130
HUB_LEN = 0.026
KNURL_R = 0.0145
STEM_R = 0.007

# Aerator
AERATOR_R = 0.013
AERATOR_LEN = 0.020
AERATOR_SCREEN_R = 0.011
HINGE_BARREL_R = 0.004
HINGE_BARREL_LEN = 0.010

# Computed by build_object_model() for hollow-bore verification.
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

    path = (
        cq.Workplane("YZ")
        .moveTo(0.0, 0.0)
        .lineTo(SPOUT_STRAIGHT, 0.0)
        .threePointArc(mid, end)
        .lineTo(end[0], SPOUT_DROP_Z)
    )
    bore_path = (
        cq.Workplane("YZ")
        .moveTo(-0.004, 0.0)
        .lineTo(SPOUT_STRAIGHT, 0.0)
        .threePointArc(mid, end)
        .lineTo(end[0], SPOUT_DROP_Z - 0.003)
    )

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


def _build_hub_solid() -> cq.Workplane:
    """Cross-handle central hub: axis +Y, back face at y=0, knurled band
    and domed front cap."""
    hub = cq.Workplane("ZX").circle(HUB_R).extrude(HUB_LEN)
    knurl = (
        cq.Workplane("ZX")
        .workplane(offset=0.007)
        .polygon(16, 2.0 * KNURL_R)
        .extrude(0.012)
    )
    dome = cq.Workplane("ZX").workplane(offset=0.018).sphere(0.0125)
    return hub.union(knurl).union(dome)


def _build_aerator_solid() -> cq.Workplane:
    """Aerator body: hollow cylinder with a screen ring at the bottom,
    axis along +Z (will be mounted pointing downward)."""
    shell = cq.Workplane("XY").circle(AERATOR_R).extrude(AERATOR_LEN)
    bore = cq.Workplane("XY").circle(AERATOR_SCREEN_R).extrude(AERATOR_LEN)
    # Screen ring at the bottom: a thin disk with smaller holes represented
    # as a ring at the outlet face.
    screen_ring = (
        cq.Workplane("XY")
        .workplane(offset=AERATOR_LEN - 0.002)
        .circle(AERATOR_R)
        .circle(AERATOR_SCREEN_R - 0.002)
        .extrude(0.002)
    )
    # Collar lip at top for hinge connection.
    collar_lip = (
        cq.Workplane("XY")
        .circle(AERATOR_R + 0.002)
        .extrude(0.004)
    )
    body = shell.cut(bore).union(screen_ring).union(collar_lip)
    return body


def _add_valve_visuals(valve, gold) -> None:
    """Stepped escutcheon + valve body + round base, axis +Y from wall."""
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
    # Round base: visible platform in front of valve body for the handle.
    valve.visual(
        Cylinder(radius=BASE_R, length=BASE_T),
        origin=Origin(
            xyz=(0.0, VALVE_BODY_FRONT_Y + BASE_T / 2.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=gold,
        name="round_base",
    )


def _add_handle_visuals(handle, hub_mesh, gold) -> None:
    """Four-arm cross handle with stem, stem collar, hub, spoke rods, and tips."""
    # Stem turns with the handle; seated into the valve body bore.
    handle.visual(
        Cylinder(radius=STEM_R, length=0.016),
        origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="stem",
    )
    # Visible stem collar: ring around the stem, between the base and the cross.
    handle.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_LEN),
        origin=Origin(
            xyz=(0.0, BASE_T + COLLAR_LEN / 2.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=gold,
        name="stem_collar",
    )
    handle.visual(hub_mesh, material=gold, name="hub")
    # Vertical spoke rod.
    handle.visual(
        Cylinder(radius=HANDLE_ROD_R, length=HANDLE_ROD_LEN),
        origin=Origin(xyz=(0.0, HANDLE_ROD_PLANE_Y, 0.0)),
        material=gold,
        name="vertical_spokes",
    )
    # Horizontal spoke rod.
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
    chrome_accent = model.material("chrome_accent", rgba=(0.75, 0.75, 0.78, 1.0))

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

    # --- aerator (child of spout, revolute on small hinge) ---
    aerator = model.part("aerator")
    aerator.visual(
        mesh_from_cadquery(_build_aerator_solid(), "aerator"),
        material=gold,
        name="aerator_body",
        # Aerator axis is +Z in its build frame; mount pointing downward (-Z).
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi, 0.0, 0.0)),
    )
    # Hinge pivot barrels (part of the aerator, on either side).
    for side_name, sx in (("pivot_left", -1.0), ("pivot_right", 1.0)):
        aerator.visual(
            Cylinder(radius=HINGE_BARREL_R, length=HINGE_BARREL_LEN),
            origin=Origin(
                xyz=(sx * (AERATOR_R + HINGE_BARREL_LEN / 2.0 + 0.001), 0.0, 0.002),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=gold,
            name=side_name,
        )
    # Aerator joint: pivots about the X axis in the spout frame.
    # At q=0 the aerator hangs straight down from the spout outlet.
    # Positive q tilts the aerator forward (toward the viewer).
    model.articulation(
        "spout_to_aerator",
        ArticulationType.REVOLUTE,
        parent=spout,
        child=aerator,
        origin=Origin(
            xyz=(0.0, SPOUT_REACH, SPOUT_DROP_Z - 0.003),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=1.5, lower=-0.15, upper=0.45,
        ),
    )

    # --- valve assemblies (fixed) and cross handles (revolute) ---
    hub_mesh = mesh_from_cadquery(_build_hub_solid(), "handle_hub")
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
            origin=Origin(xyz=(0.0, VALVE_BODY_FRONT_Y, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=5.0, velocity=3.0, lower=-math.pi, upper=math.pi
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wall = object_model.get_part("wall_panel")
    spout = object_model.get_part("spout")
    aerator = object_model.get_part("aerator")
    left_valve = object_model.get_part("left_valve")
    right_valve = object_model.get_part("right_valve")
    left_handle = object_model.get_part("left_cross_handle")
    right_handle = object_model.get_part("right_cross_handle")
    left_joint = object_model.get_articulation("left_handle_spindle")
    right_joint = object_model.get_articulation("right_handle_spindle")
    aerator_joint = object_model.get_articulation("spout_to_aerator")

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

    # --- aerator joint: revolute, pivots downward on hinge ---
    ctx.check(
        "aerator_joint_is_revolute",
        str(aerator_joint.joint_type).lower().endswith("revolute"),
        f"type={aerator_joint.joint_type}",
    )
    aer_ax = aerator_joint.axis
    ctx.check(
        "aerator_hinge_axis_horizontal",
        abs(aer_ax[2]) < 1e-9 and (abs(aer_ax[0]) > 0.5 or abs(aer_ax[1]) > 0.5),
        f"axis={aer_ax}",
    )
    aer_lim = aerator_joint.motion_limits
    ctx.check(
        "aerator_pivot_range_reasonable",
        aer_lim is not None and aer_lim.upper > 0.1 and aer_lim.lower < 0.0,
        f"limits=({aer_lim.lower}, {aer_lim.upper})",
    )

    # --- intentional embedding: handle stems seat into valve body bores ---
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
    # Hub sits flush on the round base (seated trim contact).
    ctx.allow_overlap(
        left_handle,
        left_valve,
        elem_a=left_handle.get_visual("hub"),
        elem_b=left_valve.get_visual("round_base"),
        reason="cross-handle hub rests on the round base platform as seated trim",
    )
    ctx.allow_overlap(
        right_handle,
        right_valve,
        elem_a=right_handle.get_visual("hub"),
        elem_b=right_valve.get_visual("round_base"),
        reason="cross-handle hub rests on the round base platform as seated trim",
    )
    # Prove the hub is centered on the base.
    for side, handle, valve in (
        ("left", left_handle, left_valve),
        ("right", right_handle, right_valve),
    ):
        ctx.expect_overlap(
            handle, valve,
            axes="xz",
            elem_a=handle.get_visual("hub"),
            elem_b=valve.get_visual("round_base"),
            min_overlap=0.01,
            name=f"{side}_hub_seated_on_round_base",
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
        "spout_projects_from_wall",
        sy0 < -0.18 and abs(sy1) < 0.005,
        f"spout y extent=({sy0:.3f}, {sy1:.3f})",
    )
    ctx.check(
        "spout_outlet_drops_below_axis",
        (SPOUT_AXIS_Z - sz0) > 0.06,
        f"spout zmin={sz0:.3f}, axis z={SPOUT_AXIS_Z}",
    )
    ctx.expect_gap(wall, spout, axis="y", max_gap=0.0005, max_penetration=0.0005)

    # --- round bases: visible on each valve, wider than valve body ---
    for side, valve in (("left", left_valve), ("right", right_valve)):
        base_vis = valve.get_visual("round_base")
        ctx.check(
            f"{side}_valve_has_round_base",
            base_vis is not None,
            f"round_base visual missing on {side}_valve",
        )

    # --- stem collars: visible on each handle ---
    for side, handle in (("left", left_handle), ("right", right_handle)):
        collar_vis = handle.get_visual("stem_collar")
        ctx.check(
            f"{side}_handle_has_stem_collar",
            collar_vis is not None,
            f"stem_collar visual missing on {side}_handle",
        )

    # --- aerator exists and hangs below spout outlet ---
    aer_aabb = ctx.part_world_aabb(aerator)
    assert aer_aabb is not None
    ctx.check(
        "aerator_below_spout_outlet",
        aer_aabb[0][2] < SPOUT_AXIS_Z + SPOUT_DROP_Z,
        f"aerator zmin={aer_aabb[0][2]:.4f}, outlet z~{SPOUT_AXIS_Z + SPOUT_DROP_Z:.4f}",
    )

    # --- aerator pivot proof: positive pose tilts the aerator body ---
    rest_aabb = ctx.part_world_aabb(aerator)
    assert rest_aabb is not None
    rest_zmin = rest_aabb[0][2]
    rest_ymin = rest_aabb[0][1]
    with ctx.pose({aerator_joint: 0.35}):
        tilted_aabb = ctx.part_world_aabb(aerator)
        assert tilted_aabb is not None
        tilt_zmin = tilted_aabb[0][2]
        tilt_ymin = tilted_aabb[0][1]
        ctx.check(
            "aerator_pivots_under_positive_pose",
            abs(tilt_zmin - rest_zmin) > 0.001 or abs(tilt_ymin - rest_ymin) > 0.001,
            f"rest_zmin={rest_zmin:.4f} tilt_zmin={tilt_zmin:.4f}, "
            f"rest_ymin={rest_ymin:.4f} tilt_ymin={tilt_ymin:.4f}",
        )

    # --- valve placement: flanking spout symmetrically ---
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

    # --- cross handle size: about 0.10 m tip to tip ---
    lh_aabb = ctx.part_world_aabb(left_handle)
    assert lh_aabb is not None
    (hx0, hy0, hz0), (hx1, hy1, hz1) = lh_aabb
    ctx.check(
        "cross_handle_about_0p10_tip_to_tip",
        0.095 <= (hz1 - hz0) <= 0.115 and 0.095 <= (hx1 - hx0) <= 0.115,
        f"handle extents x={hx1 - hx0:.3f}, z={hz1 - hz0:.3f}",
    )
    ctx.expect_overlap(left_handle, left_valve, axes="xz", min_overlap=0.01)
    ctx.expect_overlap(right_handle, right_valve, axes="xz", min_overlap=0.01)

    # --- overall width about 0.30 m across the handle tips ---
    rh_aabb = ctx.part_world_aabb(right_handle)
    assert rh_aabb is not None
    total_w = rh_aabb[1][0] - lh_aabb[0][0]
    ctx.check(
        "overall_width_about_0p30",
        0.28 <= total_w <= 0.33,
        f"handle-tip to handle-tip width={total_w:.3f}",
    )

    # --- handle rotation proof ---
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
        ctx.expect_overlap(right_handle, right_valve, axes="xz", min_overlap=0.01)
        ctx.expect_gap(right_handle, spout, axis="x", min_gap=0.01)

    # --- wall panel grounded ---
    wall_aabb = ctx.part_world_aabb(wall)
    assert wall_aabb is not None
    ctx.check(
        "wall_panel_grounded",
        abs(wall_aabb[0][2]) < 1e-6,
        f"wall zmin={wall_aabb[0][2]:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
