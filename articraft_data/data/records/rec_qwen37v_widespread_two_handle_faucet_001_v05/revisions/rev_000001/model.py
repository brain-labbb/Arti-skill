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
# Widespread two-handle wall-mounted faucet in polished gold brass.
# Variant 05: cross handles rotate on short vertical axles, visible stem
# collars under each handle, small hex nuts below the escutcheon bases.
#
# Frame conventions:
#   - Wall is the XZ plane at y=0 (slab occupies y>0).
#   - Faucet projects along -Y (toward viewer).
#   - Sub-assemblies use local "+Y out of wall" frame, mounted with yaw pi.
#   - Spout drops toward -Z; wall panel base at z=0.
# ---------------------------------------------------------------------------

# Layout
SPOUT_AXIS_Z = 0.20
VALVE_PITCH_X = 0.10

# Wall panel
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

# Spout escutcheon flange (~0.07 m diameter)
FLANGE_R1, FLANGE_T1 = 0.035, 0.010
FLANGE_R2, FLANGE_T2 = 0.026, 0.010

# Valve assemblies
VALVE_ESC_R1, VALVE_ESC_T1 = 0.033, 0.010
VALVE_ESC_R2, VALVE_ESC_T2 = 0.026, 0.010
VALVE_BODY_R = 0.0145
VALVE_BODY_FRONT_Y = 0.090

# Cross handle (vertical-axis rotation)
HANDLE_ROD_R = 0.0045
HANDLE_ROD_LEN = 0.100
HUB_R = 0.0130
HUB_LEN = 0.026
KNURL_R = 0.0145
STEM_R = 0.007
STEM_LEN = 0.018
HANDLE_ROD_PLANE_Z = 0.020  # spoke plane height above joint origin

# Stem collar — visible ring on top of valve body under each handle
COLLAR_R = 0.016
COLLAR_H = 0.005

# Underside hex nut below each escutcheon base
NUT_DIAM = 0.010  # circumscribed circle diameter
NUT_H = 0.004

# Volume tracking for hollow-bore verification
SPOUT_SOLID_VOLUME: float = 0.0
SPOUT_UNBORED_VOLUME: float = 0.0


def _build_spout_solid() -> cq.Workplane:
    """Spout in local frame: flange back at y=0, tube along +Y, curving down
    to an open hollow outlet."""
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
    """Cross-handle hub: axis +Z (vertical), base at z=0, knurled band and
    domed cap."""
    hub = cq.Workplane("XY").circle(HUB_R).extrude(HUB_LEN)
    knurl = (
        cq.Workplane("XY")
        .workplane(offset=0.007)
        .polygon(16, 2.0 * KNURL_R)
        .extrude(0.012)
    )
    dome = cq.Workplane("XY").workplane(offset=HUB_LEN).sphere(0.0125)
    return hub.union(knurl).union(dome)


def _build_collar_solid() -> cq.Workplane:
    """Stem collar: short ring along +Z, base at z=0."""
    return cq.Workplane("XY").circle(COLLAR_R).extrude(COLLAR_H)


def _build_nut_solid() -> cq.Workplane:
    """Hex nut: along +Z, base at z=0."""
    return cq.Workplane("XY").polygon(6, NUT_DIAM).extrude(NUT_H)


def _add_valve_visuals(valve, gold, collar_mesh, nut_mesh) -> None:
    """Stepped escutcheon + valve body + stem collar + underside nut.
    Valve frame: axis +Y from wall plane, origin on the wall face."""
    # Escutcheon steps
    valve.visual(
        Cylinder(radius=VALVE_ESC_R1, length=VALVE_ESC_T1),
        origin=Origin(
            xyz=(0.0, VALVE_ESC_T1 / 2.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)
        ),
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
    # Valve body (horizontal cylinder along Y)
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
    # Stem collar on top of valve body
    y_body = VALVE_ESC_T1 + VALVE_ESC_T2 + body_len / 2.0
    valve.visual(
        collar_mesh,
        origin=Origin(xyz=(0.0, y_body, VALVE_BODY_R)),
        material=gold,
        name="stem_collar",
    )
    # Underside hex nut below escutcheon base
    valve.visual(
        nut_mesh,
        origin=Origin(
            xyz=(0.0, VALVE_ESC_T1 / 2.0, -VALVE_ESC_R1 - NUT_H)
        ),
        material=gold,
        name="underside_nut",
    )


def _add_handle_visuals(handle, hub_mesh, gold) -> None:
    """Four-arm cross handle on a vertical axle. Handle frame origin at the
    joint (top of valve body). Hub rises along +Z; spokes are horizontal."""
    # Stem seats down into the valve body bore
    handle.visual(
        Cylinder(radius=STEM_R, length=STEM_LEN),
        origin=Origin(xyz=(0.0, 0.0, -STEM_LEN / 2.0)),
        material=gold,
        name="stem",
    )
    # Hub (vertical, knurled, domed)
    handle.visual(hub_mesh, material=gold, name="hub")
    # X-axis spoke rod (horizontal, rotated from Z to X)
    handle.visual(
        Cylinder(radius=HANDLE_ROD_R, length=HANDLE_ROD_LEN),
        origin=Origin(
            xyz=(0.0, 0.0, HANDLE_ROD_PLANE_Z), rpy=(0.0, math.pi / 2.0, 0.0)
        ),
        material=gold,
        name="x_spokes",
    )
    # Y-axis spoke rod (horizontal, rotated from Z to Y)
    handle.visual(
        Cylinder(radius=HANDLE_ROD_R, length=HANDLE_ROD_LEN),
        origin=Origin(
            xyz=(0.0, 0.0, HANDLE_ROD_PLANE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)
        ),
        material=gold,
        name="y_spokes",
    )
    # Rounded spoke tips
    half = HANDLE_ROD_LEN / 2.0
    for name, (dx, dy) in (
        ("tip_x_pos", (half, 0.0)),
        ("tip_x_neg", (-half, 0.0)),
        ("tip_y_pos", (0.0, half)),
        ("tip_y_neg", (0.0, -half)),
    ):
        handle.visual(
            Sphere(radius=HANDLE_ROD_R),
            origin=Origin(xyz=(dx, dy, HANDLE_ROD_PLANE_Z)),
            material=gold,
            name=name,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_wall_faucet")

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

    # --- central spout (fixed, wall-mounted) ---
    spout = model.part("spout")
    spout.visual(
        mesh_from_cadquery(_build_spout_solid(), "spout"),
        material=gold,
        name="tube",
    )
    model.articulation(
        "wall_to_spout",
        ArticulationType.FIXED,
        parent=wall,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, SPOUT_AXIS_Z), rpy=(0.0, 0.0, math.pi)),
    )

    # --- valve assemblies and cross handles ---
    hub_mesh = mesh_from_cadquery(_build_hub_solid(), "handle_hub")
    collar_mesh = mesh_from_cadquery(_build_collar_solid(), "stem_collar")
    nut_mesh = mesh_from_cadquery(_build_nut_solid(), "underside_nut")

    body_len = VALVE_BODY_FRONT_Y - (VALVE_ESC_T1 + VALVE_ESC_T2)
    y_body = VALVE_ESC_T1 + VALVE_ESC_T2 + body_len / 2.0

    for side, sx in (("left", -1.0), ("right", 1.0)):
        valve = model.part(f"{side}_valve")
        _add_valve_visuals(valve, gold, collar_mesh, nut_mesh)
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
            # Joint on top of valve body; axis vertical (+Z in valve
            # frame = world +Z). Positive q rotates the cross in the
            # horizontal plane.
            origin=Origin(xyz=(0.0, y_body, VALVE_BODY_R)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=5.0, velocity=3.0, lower=-math.pi, upper=math.pi
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wall = object_model.get_part("wall_panel")
    spout = object_model.get_part("spout")
    left_valve = object_model.get_part("left_valve")
    right_valve = object_model.get_part("right_valve")
    left_handle = object_model.get_part("left_cross_handle")
    right_handle = object_model.get_part("right_cross_handle")
    left_joint = object_model.get_articulation("left_handle_spindle")
    right_joint = object_model.get_articulation("right_handle_spindle")

    # --- joint plan: two independent revolute cross handles, vertical axis,
    # range -180..+180 deg ---
    for joint in (left_joint, right_joint):
        ctx.check(
            f"{joint.name}_revolute",
            str(joint.joint_type).lower().endswith("revolute"),
            f"type={joint.joint_type}",
        )
        ax = joint.axis
        ctx.check(
            f"{joint.name}_vertical_axis",
            abs(ax[0]) < 1e-9 and abs(ax[1]) < 1e-9 and abs(ax[2] - 1.0) < 1e-9,
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

    # --- variant geometry: stem collars on each valve ---
    for vn in ("left_valve", "right_valve"):
        v = object_model.get_part(vn)
        ctx.check(
            f"{vn}_has_stem_collar",
            v.get_visual("stem_collar") is not None,
        )

    # --- variant geometry: underside nuts below each escutcheon ---
    for vn in ("left_valve", "right_valve"):
        v = object_model.get_part(vn)
        ctx.check(
            f"{vn}_has_underside_nut",
            v.get_visual("underside_nut") is not None,
        )

    # --- intentional embeddings ---
    # Stems seat through the collar into the valve body
    ctx.allow_overlap(
        left_handle,
        left_valve,
        elem_a=left_handle.get_visual("stem"),
        elem_b=left_valve.get_visual("valve_body"),
        reason="stem seats through the collar into the valve body bore",
    )
    ctx.allow_overlap(
        right_handle,
        right_valve,
        elem_a=right_handle.get_visual("stem"),
        elem_b=right_valve.get_visual("valve_body"),
        reason="stem seats through the collar into the valve body bore",
    )
    # Collar rings the base of the handle hub
    ctx.allow_overlap(
        left_handle,
        left_valve,
        elem_a=left_handle.get_visual("hub"),
        elem_b=left_valve.get_visual("stem_collar"),
        reason="stem collar encircles the hub base as a visible mounting ring",
    )
    ctx.allow_overlap(
        right_handle,
        right_valve,
        elem_a=right_handle.get_visual("hub"),
        elem_b=right_valve.get_visual("stem_collar"),
        reason="stem collar encircles the hub base as a visible mounting ring",
    )

    # Prove the stem-collar-valve seating
    ctx.expect_contact(
        left_handle,
        left_valve,
        elem_a=left_handle.get_visual("stem"),
        elem_b=left_valve.get_visual("stem_collar"),
        contact_tol=0.002,
        name="left_stem_contacts_collar",
    )
    ctx.expect_contact(
        right_handle,
        right_valve,
        elem_a=right_handle.get_visual("stem"),
        elem_b=right_valve.get_visual("stem_collar"),
        contact_tol=0.002,
        name="right_stem_contacts_collar",
    )

    # --- spout: hollow bore, reach, downward outlet ---
    ctx.check(
        "spout_tube_is_hollow",
        0.0 < SPOUT_SOLID_VOLUME < 0.97 * SPOUT_UNBORED_VOLUME,
        f"solid={SPOUT_SOLID_VOLUME:.3e} vs unbored={SPOUT_UNBORED_VOLUME:.3e}",
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
    ctx.check(
        "spout_flange_about_0p07_diameter",
        0.065 <= (sx1 - sx0) <= 0.075,
        f"spout x extent={sx1 - sx0:.3f}",
    )
    ctx.expect_gap(wall, spout, axis="y", max_gap=0.0005, max_penetration=0.0005)

    # --- valve placement: flanking the spout ---
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
    ctx.expect_gap(
        wall, right_valve, axis="y", max_gap=0.0005, max_penetration=0.0005
    )

    # --- cross handle size: about 0.10 m tip to tip (horizontal plane) ---
    lh_aabb = ctx.part_world_aabb(left_handle)
    assert lh_aabb is not None
    (hx0, hy0, hz0), (hx1, hy1, hz1) = lh_aabb
    ctx.check(
        "cross_handle_about_0p10_tip_to_tip_horizontal",
        0.090 <= (hx1 - hx0) <= 0.120,
        f"handle x extent={hx1 - hx0:.3f}",
    )

    # Handle mounted on top of valve (overlap in XY footprint)
    ctx.expect_overlap(left_handle, left_valve, axes="xy", min_overlap=0.005)
    ctx.expect_overlap(right_handle, right_valve, axes="xy", min_overlap=0.005)

    # Handle hub rises above the valve body top surface
    ctx.expect_gap(
        left_handle,
        left_valve,
        axis="z",
        positive_elem="hub",
        negative_elem="valve_body",
        min_gap=-0.002,
        max_gap=0.030,
        name="left_hub_rises_above_valve_body",
    )
    ctx.expect_gap(
        right_handle,
        right_valve,
        axis="z",
        positive_elem="hub",
        negative_elem="valve_body",
        min_gap=-0.002,
        max_gap=0.030,
        name="right_hub_rises_above_valve_body",
    )

    # --- overall width about 0.30 m ---
    rh_aabb = ctx.part_world_aabb(right_handle)
    assert rh_aabb is not None
    total_w = rh_aabb[1][0] - lh_aabb[0][0]
    ctx.check(
        "overall_width_about_0p30",
        0.28 <= total_w <= 0.33,
        f"handle-tip to handle-tip width={total_w:.3f}",
    )

    # --- rotation proof: spokes rotate in horizontal plane ---
    rest_pos = ctx.part_world_position(left_handle)
    with ctx.pose({left_joint: math.pi / 4.0}):
        rot_aabb = ctx.part_world_aabb(left_handle)
        assert rot_aabb is not None
        rot_x = rot_aabb[1][0] - rot_aabb[0][0]
        ctx.check(
            "left_handle_x_extent_shrinks_at_45deg",
            rot_x < 0.095,
            f"x extent at q=45deg is {rot_x:.3f} (cross shrinks from ~0.10)",
        )
        rot_pos = ctx.part_world_position(left_handle)
        assert rest_pos is not None and rot_pos is not None
        ctx.check(
            "left_handle_origin_stays_fixed_during_rotation",
            abs(rot_pos[0] - rest_pos[0]) < 1e-6
            and abs(rot_pos[1] - rest_pos[1]) < 1e-6,
            f"rest={rest_pos}, rotated={rot_pos}",
        )

    with ctx.pose({right_joint: -math.pi / 2.0}):
        # Quarter turn: 4-fold cross symmetry, handle stays on valve
        ctx.expect_overlap(right_handle, right_valve, axes="xy", min_overlap=0.005)
        ctx.expect_gap(right_handle, spout, axis="x", min_gap=0.01)

    # --- underside nut below escutcheon ---
    for vn, valve_part in (("left_valve", left_valve), ("right_valve", right_valve)):
        nut_aabb = ctx.part_element_world_aabb(valve_part, elem="underside_nut")
        esc_aabb = ctx.part_element_world_aabb(valve_part, elem="escutcheon_base")
        assert nut_aabb is not None and esc_aabb is not None
        ctx.check(
            f"{vn}_nut_below_escutcheon",
            nut_aabb[1][2] <= esc_aabb[0][2] + 0.001,
            f"nut zmax={nut_aabb[1][2]:.4f}, esc zmin={esc_aabb[0][2]:.4f}",
        )

    # --- stem collar above valve body ---
    for vn, valve_part in (("left_valve", left_valve), ("right_valve", right_valve)):
        collar_aabb = ctx.part_element_world_aabb(valve_part, elem="stem_collar")
        body_aabb = ctx.part_element_world_aabb(valve_part, elem="valve_body")
        assert collar_aabb is not None and body_aabb is not None
        ctx.check(
            f"{vn}_collar_above_valve_body",
            collar_aabb[0][2] >= body_aabb[1][2] - 0.001,
            f"collar zmin={collar_aabb[0][2]:.4f}, body zmax={body_aabb[1][2]:.4f}",
        )

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
