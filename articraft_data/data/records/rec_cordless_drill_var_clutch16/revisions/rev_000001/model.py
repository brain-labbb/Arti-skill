from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    ExtrudeGeometry,
    KnobGeometry,
    KnobGrip,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Handheld cordless drill/driver, ~0.23 m long x ~0.19 m tall x ~0.066 m wide.
# World frame: +X = forward (toward chuck), +Z = up, +Y = left.
# Barrel centerline at z = 0.155; battery foot at the bottom rear.
# ---------------------------------------------------------------------------

BARREL_Z = 0.155
BARREL_R = 0.032
BARREL_X0, BARREL_X1 = -0.075, 0.055

COLLAR_X = 0.055  # clutch collar joint plane (housing front face)
CHUCK_X = 0.082  # chuck joint plane (front face of clutch collar)

TRIGGER_PIVOT = (-0.0215, 0.0, 0.122)
SELECTOR_POS = (-0.023, 0.0, 0.133)
BATTERY_SEAT = (-0.057, 0.0, 0.064)


def _housing_solids():
    """Build the molded housing and split it into mirrored shells + seam."""
    barrel = (
        cq.Workplane("YZ")
        .workplane(offset=BARREL_X0)
        .center(0.0, BARREL_Z)
        .circle(BARREL_R)
        .extrude(BARREL_X1 - BARREL_X0)
    )
    try:
        barrel = barrel.edges("<X").fillet(0.010)
    except Exception:
        pass

    # Pistol grip, raked slightly rearward toward the bottom.
    grip_pts = [(-0.018, 0.130), (-0.065, 0.130), (-0.085, 0.070), (-0.041, 0.070)]
    grip = cq.Workplane("XZ").polyline(grip_pts).close().extrude(0.023, both=True)
    try:
        grip = grip.edges("|Y").fillet(0.008)
    except Exception:
        pass

    # Foot rail plate the battery pack slides onto.
    foot = (
        cq.Workplane("XY")
        .workplane(offset=0.064)
        .center(-0.057, 0.0)
        .rect(0.078, 0.052)
        .extrude(0.014)
    )
    try:
        foot = foot.edges("|Z").fillet(0.006)
    except Exception:
        pass

    solid = barrel.union(grip).union(foot)

    right_box = cq.Workplane("XY").transformed(offset=(0.0, 0.1504, 0.1)).box(0.8, 0.3, 0.6)
    left_box = cq.Workplane("XY").transformed(offset=(0.0, -0.1504, 0.1)).box(0.8, 0.3, 0.6)
    seam_box = cq.Workplane("XY").transformed(offset=(0.0, 0.0, 0.1)).box(0.8, 0.0016, 0.6)

    right_shell = solid.intersect(right_box)
    left_shell = solid.intersect(left_box)
    seam = solid.intersect(seam_box)

    # Black rubber overmold saddle along the top of the motor housing.
    overmold = (
        cq.Workplane("YZ")
        .workplane(offset=-0.060)
        .center(0.0, BARREL_Z)
        .circle(BARREL_R + 0.0015)
        .extrude(0.080)
        .intersect(
            cq.Workplane("XY").transformed(offset=(0.0, 0.0, 0.172 + 0.05)).box(0.8, 0.8, 0.1)
        )
    )
    return right_shell, left_shell, seam, overmold


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cordless_drill_driver")

    lime = model.material("lime_plastic", rgba=(0.72, 0.80, 0.20, 1.0))
    rubber = model.material("rubber_black", rgba=(0.09, 0.09, 0.10, 1.0))
    charcoal = model.material("charcoal_plastic", rgba=(0.15, 0.15, 0.17, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.78, 1.0))
    seam_dark = model.material("seam_dark", rgba=(0.05, 0.05, 0.05, 1.0))
    label_yellow = model.material("label_yellow", rgba=(0.95, 0.83, 0.10, 1.0))
    tick_white = model.material("tick_white", rgba=(0.92, 0.92, 0.90, 1.0))

    # ------------------------------------------------------------------ housing
    housing = model.part("housing")
    right_shell, left_shell, seam, overmold = _housing_solids()
    housing.visual(mesh_from_cadquery(right_shell, "housing_right"), material=lime, name="right_shell")
    housing.visual(mesh_from_cadquery(left_shell, "housing_left"), material=lime, name="left_shell")
    housing.visual(mesh_from_cadquery(seam, "housing_seam"), material=seam_dark, name="parting_seam")
    housing.visual(
        mesh_from_cadquery(overmold, "housing_overmold"), material=rubber, name="top_overmold"
    )

    # Rubber overmold pad on the rear face of the grip.
    housing.visual(
        Box((0.005, 0.030, 0.052)),
        origin=Origin(xyz=(-0.075, 0.0, 0.100), rpy=(0.0, 0.322, 0.0)),
        material=rubber,
        name="grip_overmold",
    )

    # Vent slots recessed into both housing sides near the front.
    vent_rows = ((0.160, 0.0316), (0.1465, 0.0309))
    vent_idx = 0
    for vz, half_w in vent_rows:
        for vx in (0.012, 0.030, 0.048):
            for side in (1.0, -1.0):
                housing.visual(
                    Box((0.012, 0.003, 0.005)),
                    origin=Origin(xyz=(vx, side * half_w, vz)),
                    material=seam_dark,
                    name=f"vent_{vent_idx}",
                )
                vent_idx += 1

    # Torque index mark on top of the barrel, in front of the clutch collar.
    housing.visual(
        Box((0.006, 0.003, 0.0025)),
        origin=Origin(xyz=(0.050, 0.0, 0.1870)),
        material=tick_white,
        name="torque_index",
    )

    # ------------------------------------------------------------- clutch collar
    collar = model.part("clutch_collar")
    collar_geo = LatheGeometry.from_shell_profiles(
        [(0.0275, -0.0015), (0.0285, 0.004), (0.0285, 0.022), (0.026, 0.027)],
        [(0.0105, -0.0015), (0.0105, 0.027)],
        segments=48,
    )
    for i in range(24):
        rib = BoxGeometry((0.0024, 0.0022, 0.020))
        rib.translate(0.0288, 0.0, 0.012)
        rib.rotate_z(i * 2.0 * math.pi / 24.0)
        collar_geo.merge(rib)
    collar_geo.rotate_y(math.pi / 2.0)
    collar.visual(
        mesh_from_geometry(collar_geo, "clutch_collar_body"),
        material=charcoal,
        name="collar_body",
    )

    ticks_geo = None
    for i in range(16):
        tick = BoxGeometry((0.0018, 0.0018, 0.0030))
        tick.translate(0.0284, 0.0, 0.0235)
        tick.rotate_z(i * 2.0 * math.pi / 16.0)
        ticks_geo = tick if ticks_geo is None else ticks_geo.merge(tick)
    ticks_geo.rotate_y(math.pi / 2.0)
    collar.visual(
        mesh_from_geometry(ticks_geo, "collar_ticks"), material=tick_white, name="setting_ticks"
    )

    pointer_geo = BoxGeometry((0.0020, 0.0036, 0.0050))
    pointer_geo.translate(-0.0288, 0.0, 0.0240)  # lands on top after rotate_y
    pointer_geo.rotate_y(math.pi / 2.0)
    collar.visual(
        mesh_from_geometry(pointer_geo, "collar_pointer"),
        material=label_yellow,
        name="torque_pointer",
    )

    model.articulation(
        "housing_to_clutch_collar",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=collar,
        origin=Origin(xyz=(COLLAR_X, 0.0, BARREL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=-1.047, upper=1.047),
    )

    # ----------------------------------------------------------------- chuck
    chuck = model.part("chuck")
    chuck.visual(
        Cylinder(radius=0.0085, length=0.036),
        origin=Origin(xyz=(-0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="spindle",
    )
    sleeve_geo = KnobGeometry(
        0.048,
        0.028,
        body_style="cylindrical",
        grip=KnobGrip(style="knurled", count=32, depth=0.0010, helix_angle_deg=25.0),
        center=False,
    )
    chuck.visual(
        mesh_from_geometry(sleeve_geo, "chuck_sleeve_mesh"),
        origin=Origin(xyz=(0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="chuck_sleeve",
    )
    nose_geo = LatheGeometry(
        [(0.0, 0.0), (0.0235, 0.0), (0.021, 0.005), (0.012, 0.0105), (0.0, 0.0105)],
        segments=40,
    )
    nose_geo.rotate_y(math.pi / 2.0)
    nose_geo.translate(0.0315, 0.0, 0.0)
    chuck.visual(mesh_from_geometry(nose_geo, "chuck_nose"), material=charcoal, name="chuck_nose")

    for i in range(3):
        jaw = BoxGeometry((0.012, 0.004, 0.004))
        jaw.rotate_y(0.46)
        jaw.translate(0.0455, 0.0, 0.0075)
        jaw.rotate_x(i * 2.0 * math.pi / 3.0)
        chuck.visual(mesh_from_geometry(jaw, f"jaw_{i}"), material=steel, name=f"jaw_{i}")

    model.articulation(
        "housing_to_chuck",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=chuck,
        origin=Origin(xyz=(CHUCK_X, 0.0, BARREL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=50.0),
    )

    # ---------------------------------------------------------------- trigger
    trigger = model.part("trigger")
    trigger.visual(
        Cylinder(radius=0.0045, length=0.015),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="pivot_boss",
    )
    blade_pts = [
        (-0.0045, 0.0005),
        (0.0080, 0.0005),
        (0.0125, -0.0060),
        (0.0108, -0.0130),
        (0.0088, -0.0190),
        (0.0098, -0.0250),
        (0.0040, -0.0285),
        (-0.0045, -0.0275),
    ]
    blade_geo = ExtrudeGeometry(blade_pts, 0.016)
    blade_geo.rotate_x(math.pi / 2.0)
    trigger.visual(mesh_from_geometry(blade_geo, "trigger_blade"), material=rubber, name="blade")

    model.articulation(
        "housing_to_trigger",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=trigger,
        origin=Origin(xyz=TRIGGER_PIVOT),
        # Blade hangs below the pivot; +Y swings the blade rearward into the grip.
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=0.0, upper=0.26),
    )

    # --------------------------------------------------------------- selector
    selector = model.part("selector")
    selector.visual(
        Cylinder(radius=0.0048, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="pin",
    )
    for i, side in enumerate((1.0, -1.0)):
        selector.visual(
            Cylinder(radius=0.0066, length=0.0050),
            origin=Origin(xyz=(0.0, side * 0.0240, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=charcoal,
            name=f"cap_{i}",
        )

    model.articulation(
        "housing_to_selector",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=selector,
        origin=Origin(xyz=SELECTOR_POS),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=0.05, lower=-0.006, upper=0.006),
    )

    # ----------------------------------------------------------- battery pack
    battery = model.part("battery_pack")
    battery.visual(
        Box((0.080, 0.064, 0.0635)),
        origin=Origin(xyz=(0.001, 0.0, -0.03225)),
        material=rubber,
        name="shell",
    )
    for i, side in enumerate((1.0, -1.0)):
        battery.visual(
            Box((0.062, 0.005, 0.008)),
            origin=Origin(xyz=(0.0, side * 0.020, 0.003)),
            material=charcoal,
            name=f"rail_{i}",
        )
    for i, side in enumerate((1.0, -1.0)):
        battery.visual(
            Box((0.040, 0.0012, 0.013)),
            origin=Origin(xyz=(0.0, side * 0.0322, -0.028)),
            material=label_yellow,
            name=f"label_18v_{i}",
        )
        battery.visual(
            Box((0.046, 0.0012, 0.006)),
            origin=Origin(xyz=(0.0, side * 0.0322, -0.044)),
            material=label_yellow,
            name=f"label_lithium_{i}",
        )
    battery.visual(
        Box((0.007, 0.018, 0.009)),
        origin=Origin(xyz=(0.0415, 0.0, -0.010)),
        material=charcoal,
        name="release_latch",
    )

    model.articulation(
        "housing_to_battery_pack",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=battery,
        origin=Origin(xyz=BATTERY_SEAT),
        # Positive travel slides the pack rearward off the foot rail.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.2, lower=0.0, upper=0.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    chuck = object_model.get_part("chuck")
    collar = object_model.get_part("clutch_collar")
    trigger = object_model.get_part("trigger")
    selector = object_model.get_part("selector")
    battery = object_model.get_part("battery_pack")

    spin_j = object_model.get_articulation("housing_to_chuck")
    clutch_j = object_model.get_articulation("housing_to_clutch_collar")
    trigger_j = object_model.get_articulation("housing_to_trigger")
    selector_j = object_model.get_articulation("housing_to_selector")
    battery_j = object_model.get_articulation("housing_to_battery_pack")

    # ---- intentional, scoped overlaps -------------------------------------
    ctx.allow_overlap(
        housing,
        chuck,
        elem_b="spindle",
        reason="The steel spindle is captured in the gearbox bore at the housing front face.",
    )
    ctx.allow_overlap(
        housing,
        collar,
        elem_b="collar_body",
        reason="The clutch collar rear lip seats 1.5 mm onto the housing nose so it reads mounted.",
    )
    for shell in ("right_shell", "left_shell", "parting_seam"):
        ctx.allow_overlap(
            housing,
            trigger,
            elem_a=shell,
            elem_b="pivot_boss",
            reason="The trigger pivot boss is captured inside the grip recess.",
        )
        ctx.allow_overlap(
            housing,
            trigger,
            elem_a=shell,
            elem_b="blade",
            reason="The trigger blade top edge seats into the grip recess slot at rest.",
        )
        for elem in ("pin", "cap_0", "cap_1"):
            ctx.allow_overlap(
                housing,
                selector,
                elem_a=shell,
                elem_b=elem,
                reason="The forward/reverse selector pin passes through a bore in the housing.",
            )
    for elem in ("rail_0", "rail_1"):
        ctx.allow_overlap(
            housing,
            battery,
            elem_b=elem,
            reason="The battery slide rails are engaged inside the foot rail channel.",
        )

    # ---- overall envelope matches the prompt scale ------------------------
    parts = (housing, chuck, collar, trigger, selector, battery)
    mins = [1e9, 1e9, 1e9]
    maxs = [-1e9, -1e9, -1e9]
    for p in parts:
        bb = ctx.part_world_aabb(p)
        if bb is None:
            ctx.fail("part aabb available", f"no aabb for {p.name}")
            continue
        for k in range(3):
            mins[k] = min(mins[k], bb[0][k])
            maxs[k] = max(maxs[k], bb[1][k])
    length = maxs[0] - mins[0]
    width = maxs[1] - mins[1]
    height = maxs[2] - mins[2]
    ctx.check("overall length ~0.22 m", 0.20 <= length <= 0.25, f"length={length:.3f}")
    ctx.check("overall height ~0.19 m", 0.17 <= height <= 0.21, f"height={height:.3f}")
    ctx.check("overall width ~0.07 m", 0.055 <= width <= 0.080, f"width={width:.3f}")

    # ---- front-end stackup: housing -> collar -> chuck ---------------------
    ctx.expect_origin_gap(chuck, collar, axis="x", min_gap=0.02, max_gap=0.04,
                          name="chuck sits in front of the clutch collar")
    ctx.expect_gap(chuck, housing, axis="x", positive_elem="chuck_sleeve", min_gap=0.01,
                   name="knurled chuck sleeve is fully in front of the housing")
    ctx.expect_gap(chuck, collar, axis="x", positive_elem="chuck_sleeve",
                   negative_elem="collar_body", min_gap=0.001, max_gap=0.010,
                   name="visible spindle gap between collar and chuck sleeve")
    ctx.expect_within(chuck, collar, axes="yz", inner_elem="spindle", outer_elem="collar_body",
                      margin=0.0, name="spindle centered inside the collar bore")
    ctx.expect_overlap(chuck, collar, axes="x", elem_a="spindle", elem_b="collar_body",
                       min_overlap=0.02, name="spindle passes through the clutch collar")

    # ---- three exposed steel jaw tips at the nose --------------------------
    for i in range(3):
        bb = ctx.part_element_world_aabb(chuck, elem=f"jaw_{i}")
        ctx.check(
            f"jaw_{i} exposed beyond the chuck nose",
            bb is not None and bb[1][0] > 0.124,
            f"aabb={bb}",
        )

    # ---- chuck spins about the horizontal drill axis (X) --------------------
    bb_jaw_rest = ctx.part_element_world_aabb(chuck, elem="jaw_0")
    with ctx.pose({spin_j: math.pi}):
        bb_jaw_flip = ctx.part_element_world_aabb(chuck, elem="jaw_0")
    cz_rest = 0.5 * (bb_jaw_rest[0][2] + bb_jaw_rest[1][2]) if bb_jaw_rest else 0.0
    cz_flip = 0.5 * (bb_jaw_flip[0][2] + bb_jaw_flip[1][2]) if bb_jaw_flip else 0.0
    ctx.check(
        "chuck jaw swings around the X axis when the spindle spins",
        bb_jaw_rest is not None and bb_jaw_flip is not None
        and cz_rest > BARREL_Z + 0.003 and cz_flip < BARREL_Z - 0.003,
        f"jaw0 z center rest={cz_rest:.4f}, half turn={cz_flip:.4f}",
    )

    # ---- clutch collar twists between torque settings ----------------------
    bb_ptr_rest = ctx.part_element_world_aabb(collar, elem="torque_pointer")
    with ctx.pose({clutch_j: 1.0}):
        bb_ptr_turn = ctx.part_element_world_aabb(collar, elem="torque_pointer")
    py_rest = 0.5 * (bb_ptr_rest[0][1] + bb_ptr_rest[1][1]) if bb_ptr_rest else 0.0
    py_turn = 0.5 * (bb_ptr_turn[0][1] + bb_ptr_turn[1][1]) if bb_ptr_turn else 0.0
    ctx.check(
        "torque pointer rotates with the clutch collar",
        bb_ptr_rest is not None and bb_ptr_turn is not None and abs(py_turn - py_rest) > 0.015,
        f"pointer y rest={py_rest:.4f}, turned={py_turn:.4f}",
    )

    # ---- 16-detent tick spacing (22.5 deg per step) -----------------------
    detent_step = 2.0 * math.pi / 16.0  # 22.5 degrees
    bb_ptr_step = None
    with ctx.pose({clutch_j: detent_step}):
        bb_ptr_step = ctx.part_element_world_aabb(collar, elem="torque_pointer")
    py_step = 0.5 * (bb_ptr_step[0][1] + bb_ptr_step[1][1]) if bb_ptr_step else 0.0
    ctx.check(
        "one 16-detent step moves the pointer by a visible amount",
        bb_ptr_rest is not None and bb_ptr_step is not None and abs(py_step - py_rest) > 0.005,
        f"pointer y rest={py_rest:.4f}, one step={py_step:.4f}",
    )
    # Verify 16 equal steps span 360 deg: 8 steps = half turn, pointer should
    # be on the opposite side from rest.
    bb_ptr_half = None
    with ctx.pose({clutch_j: 8 * detent_step}):
        bb_ptr_half = ctx.part_element_world_aabb(collar, elem="torque_pointer")
    pz_rest = 0.5 * (bb_ptr_rest[0][2] + bb_ptr_rest[1][2]) if bb_ptr_rest else 0.0
    pz_half = 0.5 * (bb_ptr_half[0][2] + bb_ptr_half[1][2]) if bb_ptr_half else 0.0
    ctx.check(
        "8 of 16 detent steps flip the pointer to the opposite side (half turn)",
        bb_ptr_rest is not None and bb_ptr_half is not None
        and pz_rest > BARREL_Z and pz_half < BARREL_Z,
        f"pointer z rest={pz_rest:.4f}, 8 steps={pz_half:.4f}, barrel_z={BARREL_Z:.4f}",
    )

    # ---- collar has exactly 3 visuals: body, ticks, pointer ----------------
    collar_visuals = [v for v in collar.visuals]
    ctx.check(
        "clutch collar has body, setting_ticks, and torque_pointer visuals",
        len(collar_visuals) == 3,
        f"collar visual count={len(collar_visuals)}, names={[v.name for v in collar_visuals]}",
    )
    bb_ticks = ctx.part_element_world_aabb(collar, elem="setting_ticks")
    ctx.check(
        "setting_ticks visual is present on the clutch collar",
        bb_ticks is not None and bb_ticks[1][2] > BARREL_Z + 0.020,
        f"ticks aabb={bb_ticks}",
    )

    # ---- trigger squeezes rearward into the grip ---------------------------
    ctx.expect_contact(trigger, housing, name="trigger is seated against the grip")
    bb_blade_rest = ctx.part_element_world_aabb(trigger, elem="blade")
    with ctx.pose({trigger_j: 0.26}):
        bb_blade_sq = ctx.part_element_world_aabb(trigger, elem="blade")
    ctx.check(
        "squeezed trigger blade swings rearward into the grip",
        bb_blade_rest is not None and bb_blade_sq is not None
        and bb_blade_sq[0][0] < bb_blade_rest[0][0] - 0.003,
        f"blade min x rest={bb_blade_rest}, squeezed={bb_blade_sq}",
    )

    # ---- forward/reverse selector slides through the housing ---------------
    ctx.expect_origin_gap(selector, trigger, axis="z", min_gap=0.005,
                          name="selector sits above the trigger")
    bb_sel = ctx.part_world_aabb(selector)
    ctx.check(
        "selector protrudes from both sides of the housing",
        bb_sel is not None and bb_sel[1][1] > 0.025 and bb_sel[0][1] < -0.025,
        f"selector aabb={bb_sel}",
    )
    p_sel_rest = ctx.part_world_position(selector)
    with ctx.pose({selector_j: 0.006}):
        p_sel_push = ctx.part_world_position(selector)
    ctx.check(
        "selector slides 6 mm sideways through the housing",
        p_sel_rest is not None and p_sel_push is not None
        and abs((p_sel_push[1] - p_sel_rest[1]) - 0.006) < 1e-6,
        f"selector y rest={p_sel_rest}, pushed={p_sel_push}",
    )

    # ---- battery pack seats under the grip and slides rearward -------------
    ctx.expect_contact(battery, housing, name="battery rails engage the foot rail")
    ctx.expect_gap(housing, battery, axis="z", max_penetration=0.008, max_gap=0.001,
                   name="battery pack is seated flush under the housing foot")
    ctx.expect_overlap(battery, housing, axes="x", min_overlap=0.05,
                       name="battery pack sits under the rear foot")
    p_bat_rest = ctx.part_world_position(battery)
    with ctx.pose({battery_j: 0.05}):
        p_bat_out = ctx.part_world_position(battery)
        ctx.expect_overlap(battery, housing, axes="x", min_overlap=0.01,
                           name="released battery still engages the foot rail")
    ctx.check(
        "battery pack slides 50 mm rearward to detach",
        p_bat_rest is not None and p_bat_out is not None
        and abs((p_bat_rest[0] - p_bat_out[0]) - 0.05) < 1e-6,
        f"battery x rest={p_bat_rest}, released={p_bat_out}",
    )

    # ---- mirrored shells, seam, overmold, vents, labels --------------------
    bb_seam = ctx.part_element_world_aabb(housing, elem="parting_seam")
    ctx.check(
        "visible parting seam runs along the housing midplane",
        bb_seam is not None
        and (bb_seam[1][1] - bb_seam[0][1]) < 0.002
        and (bb_seam[1][2] - bb_seam[0][2]) > 0.10,
        f"seam aabb={bb_seam}",
    )
    for elem in ("right_shell", "left_shell"):
        bb = ctx.part_element_world_aabb(housing, elem=elem)
        ctx.check(f"{elem} present as a mirrored half", bb is not None, f"aabb={bb}")

    bb_om = ctx.part_element_world_aabb(housing, elem="top_overmold")
    ctx.check(
        "rubber overmold strip caps the motor housing top",
        bb_om is not None and bb_om[1][2] > BARREL_Z + BARREL_R,
        f"overmold aabb={bb_om}",
    )
    for elem in ("vent_0", "vent_1"):
        bb = ctx.part_element_world_aabb(housing, elem=elem)
        ctx.check(
            f"{elem} sits proud on a housing side wall",
            bb is not None and max(abs(bb[0][1]), abs(bb[1][1])) > 0.029,
            f"aabb={bb}",
        )
    bb_label = ctx.part_element_world_aabb(battery, elem="label_18v_0")
    ctx.check(
        "yellow 18V label shows on the battery side",
        bb_label is not None and max(abs(bb_label[0][1]), abs(bb_label[1][1])) > 0.0315,
        f"label aabb={bb_label}",
    )

    return ctx.report()


object_model = build_object_model()
