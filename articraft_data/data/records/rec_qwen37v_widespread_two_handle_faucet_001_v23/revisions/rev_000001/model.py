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
# Wall-mounted bathroom faucet in polished gold brass.
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
VALVE_PITCH_X = 0.10  # valve centers at x = +/- 0.10

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
SPOUT_REACH = SPOUT_STRAIGHT + SPOUT_BEND_R  # 0.22 m projection from wall

# Spout escutcheon flange (stepped, ~0.07 m diameter per prompt)
FLANGE_R1, FLANGE_T1 = 0.035, 0.010
FLANGE_R2, FLANGE_T2 = 0.026, 0.010

# Valve assemblies
VALVE_ESC_R1, VALVE_ESC_T1 = 0.033, 0.010
VALVE_ESC_R2, VALVE_ESC_T2 = 0.026, 0.010
VALVE_BODY_R = 0.0145
VALVE_BODY_FRONT_Y = 0.052  # front face of the valve body (joint plane)

# Cross handle
HANDLE_ROD_R = 0.0045
HANDLE_ROD_LEN = 0.100  # tip-to-tip ~0.10 m (plus rounded sphere tips)
HANDLE_ROD_PLANE_Y = 0.013  # rod plane in the handle frame
HUB_R = 0.0130
HUB_LEN = 0.026
KNURL_R = 0.0145
STEM_R = 0.007

# Stem collar (visible ring between handle and valve body)
COLLAR_R = 0.016
COLLAR_LEN = 0.008

# Underside mounting nut (hexagonal, below each escutcheon)
NUT_AF = 0.012  # hex across-flats
NUT_H = 0.008

# Diverter knob (behind spout, prismatic up-down)
DIVERTER_X = 0.0
DIVERTER_Z = SPOUT_AXIS_Z + 0.10  # above spout axis
DIVERTER_KNOB_R = 0.012
DIVERTER_KNOB_H = 0.020
DIVERTER_STEM_R = 0.005
DIVERTER_STEM_LEN = 0.018
DIVERTER_TRAVEL = 0.025  # prismatic travel

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


def _build_hex_nut_solid() -> cq.Workplane:
    """Small hexagonal mounting nut in local frame: axis +Z, centered at origin."""
    return (
        cq.Workplane("XY")
        .polygon(6, NUT_AF / math.cos(math.pi / 6.0))
        .extrude(NUT_H)
    )


def _build_diverter_solid() -> cq.Workplane:
    """Diverter knob body: cylindrical knob with a small grip stem, axis +Y
    from the wall plane. Knob front face at y=0."""
    # Knob cylinder
    knob = cq.Workplane("ZX").circle(DIVERTER_KNOB_R).extrude(DIVERTER_KNOB_H)
    # Grip ring around the knob
    ring = (
        cq.Workplane("ZX")
        .workplane(offset=DIVERTER_KNOB_H * 0.4)
        .circle(DIVERTER_KNOB_R + 0.002)
        .extrude(DIVERTER_KNOB_H * 0.2)
    )
    return knob.union(ring)


def _build_hub_solid() -> cq.Workplane:
    """Cross-handle central hub in the handle frame: axis +Y, back face at
    y=0, with a knurled (faceted) middle band and a domed front cap."""
    hub = cq.Workplane("ZX").circle(HUB_R).extrude(HUB_LEN)
    knurl = (
        cq.Workplane("ZX")
        .workplane(offset=0.007)
        .polygon(16, 2.0 * KNURL_R)
        .extrude(0.012)
    )
    dome = cq.Workplane("ZX").workplane(offset=0.018).sphere(0.0125)
    return hub.union(knurl).union(dome)


def _add_valve_visuals(valve, nut_mesh, gold) -> None:
    """Stepped escutcheon + projecting cylindrical valve body + underside hex
    nut, axis +Y from the wall plane (valve frame origin sits on the wall face)."""
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
    # Underside mounting hex nut behind the escutcheon (wall-side).
    # CadQuery nut is extruded along +Z from z=0 to z=NUT_H.
    # After Rx(90°) rotation, it extends from y=-NUT_H to y=0.
    # Small +Y offset (0.002) embeds front face slightly into escutcheon
    # for geometric connectivity.
    valve.visual(
        nut_mesh,
        origin=Origin(
            xyz=(0.0, 0.002, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=gold,
        name="mounting_nut",
    )


def _add_handle_visuals(handle, hub_mesh, gold) -> None:
    """Four-arm cross handle in the handle frame (joint frame at the valve
    body front face, axis +Y): stem collar, knurled hub, two crossing spoke
    rods with rounded sphere tips, and a hidden stem seated into the valve body."""
    # Stem turns with the handle; it seats back into the valve body bore.
    handle.visual(
        Cylinder(radius=STEM_R, length=0.016),
        origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="stem",
    )
    # Visible stem collar: wider ring at the base of the handle
    handle.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_LEN),
        origin=Origin(xyz=(0.0, COLLAR_LEN / 2.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="stem_collar",
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

    # --- valve assemblies (fixed) and cross handles (revolute) ---
    hub_mesh = mesh_from_cadquery(_build_hub_solid(), "handle_hub")
    nut_mesh = mesh_from_cadquery(_build_hex_nut_solid(), "mounting_nut")
    for side, sx in (("left", -1.0), ("right", 1.0)):
        valve = model.part(f"{side}_valve")
        _add_valve_visuals(valve, nut_mesh, gold)
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
            # Joint frame at the valve body front face; axis projects out of
            # the wall (horizontal, perpendicular to the wall plane).
            origin=Origin(xyz=(0.0, VALVE_BODY_FRONT_Y, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=5.0, velocity=3.0, lower=-math.pi, upper=math.pi
            ),
        )

    # --- diverter knob (prismatic, slides up-down behind/above spout) ---
    diverter = model.part("diverter_knob")
    div_mesh = mesh_from_cadquery(_build_diverter_solid(), "diverter_knob")
    # Diverter stem (slides inside wall bore)
    diverter.visual(
        Cylinder(radius=DIVERTER_STEM_R, length=DIVERTER_STEM_LEN),
        origin=Origin(xyz=(0.0, -DIVERTER_STEM_LEN / 2.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="diverter_stem",
    )
    # Diverter knob body (CadQuery mesh already extruded along +Y)
    diverter.visual(
        div_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=gold,
        name="diverter_body",
    )
    model.articulation(
        "wall_to_diverter",
        ArticulationType.PRISMATIC,
        parent=wall,
        child=diverter,
        # Mounted on wall above spout, axis vertical (world Z)
        origin=Origin(
            xyz=(DIVERTER_X, 0.0, DIVERTER_Z),
            rpy=(0.0, 0.0, math.pi),
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0, velocity=0.5, lower=0.0, upper=DIVERTER_TRAVEL
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
    diverter = object_model.get_part("diverter_knob")
    left_joint = object_model.get_articulation("left_handle_spindle")
    right_joint = object_model.get_articulation("right_handle_spindle")
    div_joint = object_model.get_articulation("wall_to_diverter")

    # --- joint plan: two independent revolute cross handles, axis out of the
    # wall (+Y), range -180..+180 deg ---
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

    # --- diverter prismatic joint: vertical slide, 0 to DIVERTER_TRAVEL ---
    ctx.check(
        "diverter_is_prismatic",
        str(div_joint.joint_type).lower().endswith("prismatic"),
        f"type={div_joint.joint_type}",
    )
    div_ax = div_joint.axis
    ctx.check(
        "diverter_axis_vertical",
        abs(div_ax[0]) < 1e-9 and abs(div_ax[1]) < 1e-9 and abs(div_ax[2] - 1.0) < 1e-9,
        f"axis={div_ax}",
    )
    div_lim = div_joint.motion_limits
    ctx.check(
        "diverter_travel_limits",
        div_lim is not None
        and abs(div_lim.lower) < 1e-6
        and abs(div_lim.upper - DIVERTER_TRAVEL) < 1e-6,
        f"limits=({div_lim.lower}, {div_lim.upper})",
    )

    # --- diverter slide motion proof ---
    rest_pos = ctx.part_world_position(diverter)
    with ctx.pose({div_joint: DIVERTER_TRAVEL}):
        ext_pos = ctx.part_world_position(diverter)
    assert rest_pos is not None and ext_pos is not None
    ctx.check(
        "diverter_moves_upward_at_max_travel",
        ext_pos[2] > rest_pos[2] + DIVERTER_TRAVEL * 0.9,
        f"rest_z={rest_pos[2]:.4f}, extended_z={ext_pos[2]:.4f}",
    )

    # --- stem collars exist on each handle ---
    for handle in (left_handle, right_handle):
        collar = handle.get_visual("stem_collar")
        ctx.check(
            f"{handle.name}_has_stem_collar",
            collar is not None,
            f"stem_collar visual missing on {handle.name}",
        )

    # --- underside mounting nuts exist on each valve ---
    for valve in (left_valve, right_valve):
        nut = valve.get_visual("mounting_nut")
        ctx.check(
            f"{valve.name}_has_mounting_nut",
            nut is not None,
            f"mounting_nut visual missing on {valve.name}",
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

    # --- diverter stem embedded in wall (prismatic slide bore) ---
    ctx.allow_overlap(
        wall,
        diverter,
        elem_a="panel",
        elem_b="diverter_stem",
        reason="diverter stem slides inside a bore in the wall panel",
    )

    # --- mounting nuts clamp behind the wall surface (intentional penetration) ---
    ctx.allow_overlap(
        left_valve,
        wall,
        elem_a="mounting_nut",
        elem_b="panel",
        reason="mounting nut sits behind the escutcheon and clamps into the wall thickness",
    )
    ctx.allow_overlap(
        right_valve,
        wall,
        elem_a="mounting_nut",
        elem_b="panel",
        reason="mounting nut sits behind the escutcheon and clamps into the wall thickness",
    )
    # Prove the nut stays near the escutcheon base
    for valve in (left_valve, right_valve):
        ctx.expect_contact(
            valve,
            valve,
            elem_a="mounting_nut",
            elem_b="escutcheon_base",
            name=f"{valve.name}_nut_contacts_escutcheon",
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
    ctx.check(
        "spout_flange_about_0p07_diameter",
        0.065 <= (sx1 - sx0) <= 0.075,
        f"spout x extent={sx1 - sx0:.3f} (flange sets the width)",
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
    ctx.expect_gap(
        wall, left_valve, axis="y", max_gap=0.0005, max_penetration=0.0005,
        positive_elem="panel", negative_elem="escutcheon_base",
    )
    ctx.expect_gap(
        wall, right_valve, axis="y", max_gap=0.0005, max_penetration=0.0005,
        positive_elem="panel", negative_elem="escutcheon_base",
    )

    # --- cross handle size: about 0.10 m tip to tip ---
    lh_aabb = ctx.part_world_aabb(left_handle)
    assert lh_aabb is not None
    (hx0, hy0, hz0), (hx1, hy1, hz1) = lh_aabb
    ctx.check(
        "cross_handle_about_0p10_tip_to_tip",
        0.095 <= (hz1 - hz0) <= 0.115 and 0.095 <= (hx1 - hx0) <= 0.115,
        f"handle extents x={hx1 - hx0:.3f}, z={hz1 - hz0:.3f}",
    )
    # Handle sits in front of the valve body, mounted on it (not floating).
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

    # --- off-axis spokes prove real rotation about the wall-normal axis ---
    with ctx.pose({left_joint: math.pi / 4.0}):
        rot_aabb = ctx.part_world_aabb(left_handle)
        assert rot_aabb is not None
        rot_z = rot_aabb[1][2] - rot_aabb[0][2]
        ctx.check(
            "left_handle_spokes_rotate_off_axis",
            rot_z < 0.090,
            f"z extent at q=45deg is {rot_z:.3f} (cross at 45 deg shrinks from ~0.105)",
        )
        cen = ctx.part_world_position(left_handle)
        assert cen is not None
        ctx.check(
            "left_handle_stays_on_valve_axis_while_rotating",
            abs(cen[0] + VALVE_PITCH_X) < 1e-6 and abs(cen[2] - SPOUT_AXIS_Z) < 1e-6,
            f"handle origin={cen}",
        )

    with ctx.pose({right_joint: -math.pi / 2.0}):
        # Quarter turn maps the cross onto itself; the handle must remain
        # seated on its valve and clear of the spout.
        ctx.expect_overlap(right_handle, right_valve, axes="xz", min_overlap=0.01)
        ctx.expect_gap(right_handle, spout, axis="x", min_gap=0.01)

    # --- wall panel grounded; faucet hardware at realistic mounting height ---
    wall_aabb = ctx.part_world_aabb(wall)
    assert wall_aabb is not None
    ctx.check(
        "wall_panel_grounded",
        abs(wall_aabb[0][2]) < 1e-6,
        f"wall zmin={wall_aabb[0][2]:.4f}",
    )

    # --- diverter knob is above spout on the wall ---
    div_pos = ctx.part_world_position(diverter)
    assert div_pos is not None
    ctx.check(
        "diverter_above_spout_axis",
        div_pos[2] > SPOUT_AXIS_Z + 0.05,
        f"diverter z={div_pos[2]:.4f}, spout axis z={SPOUT_AXIS_Z}",
    )
    ctx.check(
        "diverter_centered_on_spout_x",
        abs(div_pos[0]) < 0.01,
        f"diverter x={div_pos[0]:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
