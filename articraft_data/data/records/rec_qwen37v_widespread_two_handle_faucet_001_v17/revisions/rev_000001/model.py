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
# Widespread two-handle bathroom faucet with high swan-neck spout.
# Polished gold brass finish.
#
# Frame conventions:
#   - The wall is the vertical XZ plane at y = 0 (wall slab occupies y > 0).
#   - The faucet projects out of the wall along -Y (toward the viewer).
#   - Sub-assemblies are authored in a local "+Y out of wall" frame and
#     mounted with a yaw of pi, so viewer-left is world -X.
#   - The spout rises in a swan-neck arch then curves down to an aerator.
# ---------------------------------------------------------------------------

# Layout
SPOUT_AXIS_Z = 0.18          # height of spout/valve mounting axis
VALVE_PITCH_X = 0.10         # valve centres at x = +/- 0.10

# Wall panel (mounting substrate)
WALL_W = 0.38
WALL_T = 0.012
WALL_H = 0.40

# Spout tube
SPOUT_TUBE_R = 0.015          # outer radius (~0.03 m diameter)
SPOUT_BORE_R = 0.0105         # inner bore radius

# Swan-neck path control points (local YZ plane).
# Local +Y = out from wall, +Z = up.
# Built with threePointArc chain for robust sweep + boolean.

# Spout outlet in spout local frame
OUTLET_Y = 0.12
OUTLET_Z = -0.03

# Spout escutcheon flange (stepped)
FLANGE_R1, FLANGE_T1 = 0.035, 0.010
FLANGE_R2, FLANGE_T2 = 0.026, 0.010

# Valve assemblies
VALVE_ESC_R1, VALVE_ESC_T1 = 0.033, 0.010
VALVE_ESC_R2, VALVE_ESC_T2 = 0.026, 0.010
VALVE_BODY_R = 0.0145
VALVE_BODY_FRONT_Y = 0.052

# Cross handle
HANDLE_ROD_R = 0.0045
HANDLE_ROD_LEN = 0.100
HANDLE_ROD_PLANE_Y = 0.013
HUB_R = 0.0130
HUB_LEN = 0.026
KNURL_R = 0.0145
STEM_R = 0.007

# Stem collar (visible ring between valve body and handle)
COLLAR_R_OUTER = 0.018
COLLAR_R_INNER = 0.0145
COLLAR_T = 0.006

# Underside hex nut
NUT_AF = 0.014         # across-flats diameter
NUT_H = 0.005
NUT_HOLE_R = 0.005

# Aerator
AERATOR_R = 0.011
AERATOR_LEN = 0.016
AERATOR_RIM_R = 0.013

# Computed volumes for hollow-bore verification
SPOUT_SOLID_VOLUME: float = 0.0
SPOUT_UNBORED_VOLUME: float = 0.0


def _build_swan_neck_solid() -> cq.Workplane:
    """Swan-neck spout: flange at wall, tube rises in an arch, descends to an
    open downward-pointing outlet with visible bore."""
    spline_pts = [
        (0.00, 0.00), (0.03, 0.01), (0.05, 0.08), (0.07, 0.16),
        (0.09, 0.19), (0.10, 0.16), (0.11, 0.08), (0.12, 0.00), (0.12, -0.04),
    ]
    path = cq.Workplane("YZ").spline(spline_pts, tangents=[(1, 0), (0, -1)])

    # Sweep tube profile along spline path
    tube = cq.Workplane("ZX").circle(SPOUT_TUBE_R).sweep(path)

    # Stepped escutcheon flange at the wall
    flange_outer = cq.Workplane("ZX").circle(FLANGE_R1).extrude(FLANGE_T1)
    flange_step = (
        cq.Workplane("ZX")
        .workplane(offset=FLANGE_T1)
        .circle(FLANGE_R2)
        .extrude(FLANGE_T2)
    )
    unbored = tube.union(flange_outer).union(flange_step)

    # Inlet bore: straight cylinder through the flange along +Y
    inlet_bore = (
        cq.Workplane("ZX")
        .workplane(offset=-0.005)
        .circle(SPOUT_BORE_R)
        .extrude(FLANGE_T1 + FLANGE_T2 + 0.04)
    )

    # Outlet bore: short vertical cylinder at the spout tip to open the end
    outlet_bore = (
        cq.Workplane("XY")
        .circle(SPOUT_BORE_R)
        .extrude(0.06)
        .translate((0.0, 0.12, -0.06))
    )

    solid = unbored.cut(inlet_bore).cut(outlet_bore)

    global SPOUT_SOLID_VOLUME, SPOUT_UNBORED_VOLUME
    SPOUT_SOLID_VOLUME = solid.val().Volume()
    SPOUT_UNBORED_VOLUME = unbored.val().Volume()
    return solid


def _build_aerator_solid() -> cq.Workplane:
    """Aerator housing extending downward (-Z) from origin: short cylinder
    with a wider rim at the top and a screen ring at the bottom."""
    h = AERATOR_LEN
    r = AERATOR_R
    body = cq.Workplane("XY").workplane(offset=-h).circle(r).extrude(h)
    rim = (
        cq.Workplane("XY")
        .workplane(offset=-0.003)
        .circle(AERATOR_RIM_R)
        .extrude(0.003)
    )
    screen = (
        cq.Workplane("XY")
        .workplane(offset=-h)
        .circle(r - 0.001)
        .extrude(0.002)
    )
    return body.union(rim).union(screen)


def _build_hex_nut() -> cq.Workplane:
    """Hex nut: hexagonal prism with centre bore, axis +Z."""
    nut = cq.Workplane("XY").polygon(6, NUT_AF).extrude(NUT_H)
    hole = cq.Workplane("XY").circle(NUT_HOLE_R).extrude(NUT_H)
    return nut.cut(hole)


def _build_stem_collar() -> cq.Workplane:
    """Stem collar ring: annular disc on ZX plane extruded along +Y."""
    outer = cq.Workplane("ZX").circle(COLLAR_R_OUTER).extrude(COLLAR_T)
    inner = cq.Workplane("ZX").circle(COLLAR_R_INNER).extrude(COLLAR_T)
    return outer.cut(inner)


def _build_hub_solid() -> cq.Workplane:
    """Cross-handle central hub: axis +Y, back face at y=0."""
    hub = cq.Workplane("ZX").circle(HUB_R).extrude(HUB_LEN)
    knurl = (
        cq.Workplane("ZX")
        .workplane(offset=0.007)
        .polygon(16, 2.0 * KNURL_R)
        .extrude(0.012)
    )
    dome = cq.Workplane("ZX").workplane(offset=0.018).sphere(0.0125)
    return hub.union(knurl).union(dome)


def _add_valve_visuals(valve, gold, collar_mesh, nut_mesh) -> None:
    """Stepped escutcheon + valve body + stem collar + underside nut."""
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
    # Stem collar ring at the front of the valve body
    collar_y = VALVE_BODY_FRONT_Y - COLLAR_T
    valve.visual(
        collar_mesh,
        origin=Origin(xyz=(0.0, collar_y, 0.0)),
        material=gold,
        name="stem_collar",
    )
    # Underside hex nut below the escutcheon base (clear of wall plane)
    valve.visual(
        nut_mesh,
        origin=Origin(
            xyz=(0.0, 0.012, -VALVE_ESC_R1 - NUT_H),
        ),
        material=gold,
        name="mounting_nut",
    )


def _add_handle_visuals(handle, hub_mesh, gold) -> None:
    """Four-arm cross handle: stem, hub, two crossing rods, four sphere tips."""
    handle.visual(
        Cylinder(radius=STEM_R, length=0.016),
        origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="stem",
    )
    handle.visual(hub_mesh, material=gold, name="hub")
    handle.visual(
        Cylinder(radius=HANDLE_ROD_R, length=HANDLE_ROD_LEN),
        origin=Origin(xyz=(0.0, HANDLE_ROD_PLANE_Y, 0.0)),
        material=gold,
        name="vertical_spokes",
    )
    handle.visual(
        Cylinder(radius=HANDLE_ROD_R, length=HANDLE_ROD_LEN),
        origin=Origin(
            xyz=(0.0, HANDLE_ROD_PLANE_Y, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=gold,
        name="horizontal_spokes",
    )
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
    model = ArticulatedObject(name="widespread_swan_neck_faucet")

    gold = model.material("polished_gold_brass", rgba=(0.85, 0.66, 0.20, 1.0))
    chrome = model.material("aerator_chrome", rgba=(0.75, 0.75, 0.78, 1.0))
    wall_white = model.material("wall_white", rgba=(0.93, 0.93, 0.90, 1.0))

    # --- Wall panel (root, mounting substrate) ---
    wall = model.part("wall_panel")
    wall.visual(
        Box((WALL_W, WALL_T, WALL_H)),
        origin=Origin(xyz=(0.0, WALL_T / 2.0, WALL_H / 2.0)),
        material=wall_white,
        name="panel",
    )

    # --- Central swan-neck spout (fixed) ---
    spout = model.part("spout")
    spout.visual(
        mesh_from_cadquery(_build_swan_neck_solid(), "spout"),
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

    # --- Aerator with revolute hinge at spout outlet ---
    aerator = model.part("aerator")
    aerator.visual(
        mesh_from_cadquery(_build_aerator_solid(), "aerator"),
        material=chrome,
        name="housing",
    )
    model.articulation(
        "spout_to_aerator",
        ArticulationType.REVOLUTE,
        parent=spout,
        child=aerator,
        origin=Origin(xyz=(0.0, OUTLET_Y, OUTLET_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=1.5, lower=-0.35, upper=0.35
        ),
    )

    # --- Valve assemblies and cross handles ---
    hub_mesh = mesh_from_cadquery(_build_hub_solid(), "handle_hub")
    collar_mesh = mesh_from_cadquery(_build_stem_collar(), "stem_collar")
    nut_mesh = mesh_from_cadquery(_build_hex_nut(), "mounting_nut")

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

    # --- Swan neck arch: spout rises well above the mounting axis ---
    spout_aabb = ctx.part_world_aabb(spout)
    assert spout_aabb is not None
    (sx0, sy0, sz0), (sx1, sy1, sz1) = spout_aabb
    ctx.check(
        "swan_neck_arch_rises_above_axis",
        sz1 > SPOUT_AXIS_Z + 0.10,
        f"spout zmax={sz1:.3f}, axis z={SPOUT_AXIS_Z}, need >0.10m arch",
    )

    # --- Spout outlet drops back toward axis level ---
    ctx.check(
        "spout_outlet_descends",
        sz0 < SPOUT_AXIS_Z + 0.05,
        f"spout zmin={sz0:.3f}, outlet should descend near or below axis",
    )

    # --- Spout is hollow (bore visible at outlet) ---
    ctx.check(
        "spout_tube_is_hollow",
        0.0 < SPOUT_SOLID_VOLUME < 0.97 * SPOUT_UNBORED_VOLUME,
        f"solid={SPOUT_SOLID_VOLUME:.3e} vs unbored={SPOUT_UNBORED_VOLUME:.3e}",
    )

    # --- Flange seats against wall ---
    ctx.expect_gap(wall, spout, axis="y", max_gap=0.001, max_penetration=0.001)

    # --- Aerator hinge: revolute with real range ---
    ctx.check(
        "aerator_joint_is_revolute",
        str(aerator_joint.joint_type).lower().endswith("revolute"),
        f"type={aerator_joint.joint_type}",
    )
    alim = aerator_joint.motion_limits
    ctx.check(
        "aerator_hinge_has_real_range",
        alim is not None and alim.lower < -0.1 and alim.upper > 0.1,
        f"limits=({alim.lower}, {alim.upper})",
    )

    # --- Aerator tilts when posed (use AABB since part origin is the pivot) ---
    rest_aabb = ctx.part_world_aabb(aerator)
    assert rest_aabb is not None
    with ctx.pose({aerator_joint: 0.30}):
        tilted_aabb = ctx.part_world_aabb(aerator)
        assert tilted_aabb is not None
        rest_cy = (rest_aabb[0][1] + rest_aabb[1][1]) / 2.0
        tilt_cy = (tilted_aabb[0][1] + tilted_aabb[1][1]) / 2.0
        rest_cz = (rest_aabb[0][2] + rest_aabb[1][2]) / 2.0
        tilt_cz = (tilted_aabb[0][2] + tilted_aabb[1][2]) / 2.0
        ctx.check(
            "aerator_tilts_on_hinge",
            abs(tilt_cy - rest_cy) > 0.0005 or abs(tilt_cz - rest_cz) > 0.0005,
            f"rest_center=({rest_cy:.4f},{rest_cz:.4f}), "
            f"tilted_center=({tilt_cy:.4f},{tilt_cz:.4f})",
        )

    # --- Aerator overlaps spout at outlet (intentional seating) ---
    ctx.allow_overlap(
        aerator, spout,
        elem_a=aerator.get_visual("housing"),
        elem_b=spout.get_visual("tube"),
        reason="aerator rim seats inside the spout outlet bore",
    )
    ctx.expect_contact(spout, aerator, contact_tol=0.008,
                       name="aerator_seated_at_spout_outlet")

    # --- Handle joints: two independent revolute handles ---
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
        hlim = joint.motion_limits
        ctx.check(
            f"{joint.name}_full_turn_range",
            hlim is not None
            and abs(hlim.lower + math.pi) < 1e-6
            and abs(hlim.upper - math.pi) < 1e-6,
            f"limits=({hlim.lower}, {hlim.upper})",
        )

    # --- Stem collars present on each valve ---
    for valve in (left_valve, right_valve):
        ctx.check(
            f"{valve.name}_has_stem_collar",
            valve.get_visual("stem_collar") is not None,
            f"missing stem_collar on {valve.name}",
        )

    # --- Underside nuts present on each valve ---
    for valve in (left_valve, right_valve):
        ctx.check(
            f"{valve.name}_has_mounting_nut",
            valve.get_visual("mounting_nut") is not None,
            f"missing mounting_nut on {valve.name}",
        )

    # --- Intentional overlap: handle stems in valve body bores ---
    ctx.allow_overlap(
        left_handle, left_valve,
        elem_a=left_handle.get_visual("stem"),
        elem_b=left_valve.get_visual("valve_body"),
        reason="valve stem seated inside the valve body bore",
    )
    ctx.allow_overlap(
        right_handle, right_valve,
        elem_a=right_handle.get_visual("stem"),
        elem_b=right_valve.get_visual("valve_body"),
        reason="valve stem seated inside the valve body bore",
    )

    # --- Valve placement flanking the spout ---
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
    ctx.expect_gap(wall, left_valve, axis="y", max_gap=0.001, max_penetration=0.001)
    ctx.expect_gap(wall, right_valve, axis="y", max_gap=0.001, max_penetration=0.001)

    # --- Cross handle size ~0.10 m tip-to-tip ---
    lh_aabb = ctx.part_world_aabb(left_handle)
    assert lh_aabb is not None
    (hx0, hy0, hz0), (hx1, hy1, hz1) = lh_aabb
    ctx.check(
        "cross_handle_about_0p10_tip_to_tip",
        0.095 <= (hz1 - hz0) <= 0.115 and 0.095 <= (hx1 - hx0) <= 0.115,
        f"handle x={hx1 - hx0:.3f}, z={hz1 - hz0:.3f}",
    )
    ctx.expect_overlap(left_handle, left_valve, axes="xz", min_overlap=0.01)
    ctx.expect_overlap(right_handle, right_valve, axes="xz", min_overlap=0.01)

    # --- Overall width ~0.30 m ---
    rh_aabb = ctx.part_world_aabb(right_handle)
    assert rh_aabb is not None
    total_w = rh_aabb[1][0] - lh_aabb[0][0]
    ctx.check(
        "overall_width_about_0p30",
        0.28 <= total_w <= 0.33,
        f"width={total_w:.3f}",
    )

    # --- Handle rotation proof ---
    with ctx.pose({left_joint: math.pi / 4.0}):
        rot_aabb = ctx.part_world_aabb(left_handle)
        assert rot_aabb is not None
        rot_z = rot_aabb[1][2] - rot_aabb[0][2]
        ctx.check(
            "left_handle_spokes_rotate_off_axis",
            rot_z < 0.090,
            f"z extent at 45deg={rot_z:.3f}",
        )

    with ctx.pose({right_joint: -math.pi / 2.0}):
        ctx.expect_overlap(right_handle, right_valve, axes="xz", min_overlap=0.01)

    # --- Wall panel grounded ---
    wall_aabb = ctx.part_world_aabb(wall)
    assert wall_aabb is not None
    ctx.check(
        "wall_panel_grounded",
        abs(wall_aabb[0][2]) < 1e-6,
        f"wall zmin={wall_aabb[0][2]:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
