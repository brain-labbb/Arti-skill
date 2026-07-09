"""Apollo Lunar Roving Vehicle (LRV) seed asset.

Built from the side-view reference photo: an open silvery tubular chassis
riding on four dark wire-mesh wheels with orange-tan fenders, an
umbrella-shaped high-gain dish antenna on a swiveling front mast, gold-foil
instrument boxes at the front, two fold-up seats with white webbing straps,
a T-handle hand controller console between the seats, a thin low-gain
antenna rod, and a rear equipment rack with stowed boxes.

Articulation: all four wire-mesh wheels spin on their hubs (CONTINUOUS),
the high-gain antenna mast swivels about its base (REVOLUTE), and the
T-handle hand controller tilts fore/aft (REVOLUTE).
"""

from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------- layout ----
# Real LRV proportions: ~3.1 m long, 2.29 m wheelbase, 1.83 m track,
# 0.82 m diameter wheels, floor pan ~0.45 m above the surface.
WHEEL_RADIUS = 0.408
WHEEL_WIDTH = 0.22
WHEEL_X = 1.145  # half wheelbase
WHEEL_Y = 0.915  # half track
FLOOR_TOP = 0.45
FLOOR_THICK = 0.03
FLOOR_LEN = 2.5
FLOOR_HALF_W = 0.75
RAIL_Y = 0.74
RAIL_Z = 0.465
FENDER_R = 0.48  # mid radius of the fender arc over each wheel
FENDER_SEGS = 7
DISH_TILT = 0.30  # umbrella dish pitched toward the front of the rover

# (name, wheel center x, lateral sign)
WHEELS = (
    ("front_left", WHEEL_X, 1.0),
    ("front_right", WHEEL_X, -1.0),
    ("rear_left", -WHEEL_X, 1.0),
    ("rear_right", -WHEEL_X, -1.0),
)
# (side, seat center y)
SEATS = (("left", 0.33), ("right", -0.33))
STRAP_ZS = (0.70, 0.79, 0.88, 0.97)
BACK_LEAN = 0.17  # backrest lean-back angle in radians


# -------------------------------------------------------------- geometry ----
def _wheel_disc_mesh():
    """Silvery hub/rim/spoke disc that carries the wire-mesh tire."""
    wheel = WheelGeometry(
        0.330,
        0.16,
        rim=WheelRim(
            inner_radius=0.270,
            flange_height=0.014,
            flange_thickness=0.008,
            bead_seat_depth=0.004,
        ),
        hub=WheelHub(radius=0.070, width=0.18, cap_style="domed"),
        face=WheelFace(dish_depth=0.015, front_inset=0.008, rear_inset=0.006),
        spokes=WheelSpokes(style="split_y", count=8, thickness=0.010, window_radius=0.045),
    )
    return mesh_from_geometry(wheel, "lrv_wheel_disc")


def _tire_mesh():
    """Dark woven wire-mesh tire; chevron tread stands in for the mesh weave."""
    tire = TireGeometry(
        WHEEL_RADIUS,
        WHEEL_WIDTH,
        inner_radius=0.310,
        carcass=TireCarcass(belt_width_ratio=0.68, sidewall_bulge=0.05),
        tread=TireTread(style="chevron", depth=0.010, count=30, angle_deg=28.0, land_ratio=0.55),
        sidewall=TireSidewall(style="rounded", bulge=0.05),
        shoulder=TireShoulder(width=0.008, radius=0.004),
    )
    return mesh_from_geometry(tire, "lrv_tire")


def _dish_mesh():
    """Umbrella high-gain dish: thin shell, 8 ribs, hub, feed spike; tilted."""
    outer = [(0.035, 0.000), (0.18, 0.020), (0.32, 0.055), (0.46, 0.100), (0.50, 0.118)]
    inner = [(0.035, 0.010), (0.18, 0.030), (0.32, 0.065), (0.46, 0.110), (0.495, 0.118)]
    dish = LatheGeometry.from_shell_profiles(outer, inner, segments=48)
    # Umbrella ribs follow the concave top surface so they stay embedded.
    surface = [(0.040, 0.014), (0.18, 0.032), (0.32, 0.067), (0.46, 0.112), (0.495, 0.120)]
    for k in range(8):
        phi = k * math.pi / 4.0
        pts = [(r * math.cos(phi), r * math.sin(phi), z + 0.003) for r, z in surface]
        rib = tube_from_spline_points(pts, radius=0.006, samples_per_segment=5, radial_segments=10)
        dish.merge(rib)
    dish.merge(CylinderGeometry(0.045, 0.14).translate(0.0, 0.0, -0.01))  # center hub
    dish.merge(CylinderGeometry(0.008, 0.36).translate(0.0, 0.0, 0.23))  # feed spike
    dish.merge(CylinderGeometry(0.028, 0.05).translate(0.0, 0.0, 0.43))  # feed head
    dish.rotate_y(DISH_TILT).translate(0.0, 0.0, 0.90)
    return mesh_from_geometry(dish, "hga_dish")


def _suspension_arm_mesh(label: str, p0, p1):
    mid = ((p0[0] + p1[0]) / 2.0, (p0[1] + p1[1]) / 2.0, (p0[2] + p1[2]) / 2.0)
    tube = tube_from_spline_points(
        [p0, mid, p1], radius=0.02, samples_per_segment=4, radial_segments=12
    )
    return mesh_from_geometry(tube, label)


def _fender_strut_mesh(name: str, wx: float, s: float):
    """Bent tube from the side rail up and over the tire to the fender top."""
    pts = [
        (wx, s * RAIL_Y, 0.46),
        (wx, s * RAIL_Y, 0.70),
        (wx, s * RAIL_Y, 0.86),
        (wx, s * 0.76, 0.90),
        (wx, s * 0.82, 0.915),
        (wx, s * WHEEL_Y, 0.915),
    ]
    tube = tube_from_spline_points(pts, radius=0.015, samples_per_segment=6, radial_segments=12)
    return mesh_from_geometry(tube, f"{name}_fender_strut")


# ----------------------------------------------------------------- model ----
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="apollo_lunar_roving_vehicle")

    model.material("chassis_alloy", rgba=(0.72, 0.73, 0.76, 1.0))
    model.material("floor_grey", rgba=(0.42, 0.43, 0.45, 1.0))
    model.material("hub_silver", rgba=(0.80, 0.81, 0.83, 1.0))
    model.material("wire_mesh", rgba=(0.15, 0.15, 0.16, 1.0))
    model.material("fender_orange", rgba=(0.80, 0.44, 0.16, 1.0))
    model.material("gold_foil", rgba=(0.83, 0.60, 0.13, 1.0))
    model.material("seat_white", rgba=(0.90, 0.90, 0.87, 1.0))
    model.material("console_dark", rgba=(0.20, 0.20, 0.22, 1.0))
    model.material("dish_white", rgba=(0.85, 0.85, 0.83, 1.0))

    chassis = model.part("chassis")

    # ---- floor pan and tubular frame rails ----
    chassis.visual(
        Box((FLOOR_LEN, 2.0 * FLOOR_HALF_W, FLOOR_THICK)),
        origin=Origin(xyz=(0.0, 0.0, FLOOR_TOP - FLOOR_THICK / 2.0)),
        name="floor_pan",
        material="floor_grey",
    )
    for s, side in ((1.0, "left"), (-1.0, "right")):
        chassis.visual(
            Cylinder(0.025, FLOOR_LEN),
            origin=Origin(xyz=(0.0, s * RAIL_Y, RAIL_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            name=f"side_rail_{side}",
            material="chassis_alloy",
        )
    for xx, tag in ((FLOOR_LEN / 2.0 - 0.01, "front"), (-FLOOR_LEN / 2.0 + 0.01, "rear")):
        chassis.visual(
            Cylinder(0.025, 2.0 * RAIL_Y),
            origin=Origin(xyz=(xx, 0.0, RAIL_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            name=f"cross_rail_{tag}",
            material="chassis_alloy",
        )

    # ---- suspension, stub axles, and orange fenders (per wheel) ----
    for name, wx, s in WHEELS:
        knuckle_y = s * 0.79
        anchors = (
            max(min(wx - 0.28, 1.20), -1.20),
            max(min(wx + 0.28, 1.20), -1.20),
        )
        for j, ax in enumerate(anchors):
            chassis.visual(
                _suspension_arm_mesh(
                    f"{name}_susp_arm{j}",
                    (ax, s * 0.73, 0.455),
                    (wx, knuckle_y, 0.41),
                ),
                name=f"{name}_susp_arm{j}",
                material="chassis_alloy",
            )
        chassis.visual(
            Box((0.10, 0.06, 0.08)),
            origin=Origin(xyz=(wx, knuckle_y, 0.41)),
            name=f"{name}_knuckle",
            material="chassis_alloy",
        )
        # Stub axle rides into the wheel hub bore (declared bearing overlap);
        # the hub bore radius is ~0.0297, so a 0.034 axle seats into its wall.
        chassis.visual(
            Cylinder(0.034, 0.19),
            origin=Origin(xyz=(wx, s * 0.885, WHEEL_RADIUS), rpy=(math.pi / 2.0, 0.0, 0.0)),
            name=f"{name}_axle",
            material="hub_silver",
        )
        # Segmented orange-tan fender arc over the top of the wheel.
        for seg in range(FENDER_SEGS):
            theta = math.radians(40.0 + (seg + 0.5) * 100.0 / FENDER_SEGS)
            cx = wx + FENDER_R * math.cos(theta)
            cz = WHEEL_RADIUS + FENDER_R * math.sin(theta)
            chassis.visual(
                Box((0.128, 0.30, 0.04)),
                origin=Origin(xyz=(cx, s * WHEEL_Y, cz), rpy=(0.0, math.pi / 2.0 - theta, 0.0)),
                name=f"{name}_fender_seg{seg}",
                material="fender_orange",
            )
        chassis.visual(
            _fender_strut_mesh(name, wx, s),
            name=f"{name}_fender_strut",
            material="chassis_alloy",
        )

    # ---- gold-foil instrument boxes and camera unit at the front ----
    gold_boxes = (
        ("gold_instrument_box_a", (0.35, 0.45, 0.30), (1.00, 0.30, 0.595)),
        ("gold_instrument_box_b", (0.30, 0.35, 0.25), (1.05, -0.35, 0.570)),
    )
    for nm, size, pos in gold_boxes:
        chassis.visual(Box(size), origin=Origin(xyz=pos), name=nm, material="gold_foil")
    chassis.visual(
        Box((0.20, 0.25, 0.12)),
        origin=Origin(xyz=(1.00, 0.30, 0.805)),
        name="front_camera_unit",
        material="console_dark",
    )

    # ---- high-gain antenna swivel base (mast itself is articulated) ----
    chassis.visual(
        Cylinder(0.05, 0.08),
        origin=Origin(xyz=(1.15, 0.0, 0.47)),
        name="hga_mast_base",
        material="chassis_alloy",
    )

    # ---- thin low-gain antenna rod ----
    chassis.visual(
        Cylinder(0.03, 0.09),
        origin=Origin(xyz=(0.62, 0.30, 0.49)),
        name="low_gain_antenna_base",
        material="chassis_alloy",
    )
    chassis.visual(
        Cylinder(0.012, 0.85),
        origin=Origin(xyz=(0.62, 0.30, 0.90)),
        name="low_gain_antenna_mast",
        material="hub_silver",
    )
    chassis.visual(
        Cylinder(0.022, 0.12),
        origin=Origin(xyz=(0.62, 0.30, 1.30)),
        name="low_gain_antenna_tip",
        material="console_dark",
    )

    # ---- control console between the seats ----
    chassis.visual(
        Box((0.25, 0.22, 0.20)),
        origin=Origin(xyz=(0.28, 0.0, 0.548)),
        name="control_console",
        material="console_dark",
    )

    # ---- two fold-up seats with white webbing straps ----
    lean = math.tan(BACK_LEAN)
    for side, sy in SEATS:
        chassis.visual(
            Box((0.45, 0.42, 0.03)),
            origin=Origin(xyz=(-0.10, sy, 0.62)),
            name=f"seat_{side}_pan",
            material="seat_white",
        )
        for li, (lx, ly) in enumerate(((-0.27, -0.16), (-0.27, 0.16), (0.07, -0.16), (0.07, 0.16))):
            chassis.visual(
                Cylinder(0.015, 0.175),
                origin=Origin(xyz=(lx, sy + ly, 0.5275)),
                name=f"seat_{side}_leg{li}",
                material="chassis_alloy",
            )
        for ti, ty in enumerate((-0.19, 0.19)):
            chassis.visual(
                Cylinder(0.015, 0.47),
                origin=Origin(xyz=(-0.348, sy + ty, 0.835), rpy=(0.0, -BACK_LEAN, 0.0)),
                name=f"seat_{side}_back_tube{ti}",
                material="chassis_alloy",
            )
        for si, sz in enumerate(STRAP_ZS):
            sx = -0.348 - lean * (sz - 0.835)
            chassis.visual(
                Box((0.025, 0.42, 0.055)),
                origin=Origin(xyz=(sx, sy, sz)),
                name=f"seat_{side}_strap{si}",
                material="seat_white",
            )
        top_x = -0.348 - lean * (1.05 - 0.835)
        chassis.visual(
            Cylinder(0.016, 0.40),
            origin=Origin(xyz=(top_x, sy, 1.05), rpy=(math.pi / 2.0, 0.0, 0.0)),
            name=f"seat_{side}_top_rail",
            material="chassis_alloy",
        )

    # ---- rear equipment rack with stowed boxes ----
    chassis.visual(
        Box((0.50, 1.20, 0.03)),
        origin=Origin(xyz=(-1.00, 0.0, 0.545)),
        name="rear_rack_platform",
        material="chassis_alloy",
    )
    for pi, (px, py) in enumerate(((-0.80, -0.50), (-0.80, 0.50), (-1.20, -0.50), (-1.20, 0.50))):
        chassis.visual(
            Cylinder(0.015, 0.12),
            origin=Origin(xyz=(px, py, 0.50)),
            name=f"rear_rack_post{pi}",
            material="chassis_alloy",
        )
    chassis.visual(
        Box((0.40, 0.50, 0.28)),
        origin=Origin(xyz=(-1.00, -0.28, 0.695)),
        name="rear_tool_box_a",
        material="floor_grey",
    )
    chassis.visual(
        Box((0.35, 0.45, 0.22)),
        origin=Origin(xyz=(-1.00, 0.30, 0.665)),
        name="rear_tool_box_b",
        material="seat_white",
    )

    # ---- four spinning wire-mesh wheels ----
    wheel_disc = _wheel_disc_mesh()
    tire = _tire_mesh()
    for name, wx, s in WHEELS:
        wheel = model.part(f"{name}_wheel")
        # WheelGeometry spins about local X; yaw it so the axle runs along Y,
        # keeping the domed hub cap on the outboard side.
        yaw = math.pi / 2.0 if s > 0 else -math.pi / 2.0
        wheel.visual(
            wheel_disc, origin=Origin(rpy=(0.0, 0.0, yaw)), name="wheel_disc", material="hub_silver"
        )
        wheel.visual(tire, origin=Origin(rpy=(0.0, 0.0, yaw)), name="tire", material="wire_mesh")
        model.articulation(
            f"{name}_wheel_spin",
            ArticulationType.CONTINUOUS,
            parent=chassis,
            child=wheel,
            origin=Origin(xyz=(wx, s * WHEEL_Y, WHEEL_RADIUS)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=80.0, velocity=12.0),
        )

    # ---- swiveling high-gain umbrella antenna ----
    antenna = model.part("high_gain_antenna")
    antenna.visual(
        Cylinder(0.022, 0.92),
        origin=Origin(xyz=(0.0, 0.0, 0.44)),
        name="hga_mast_tube",
        material="chassis_alloy",
    )
    antenna.visual(_dish_mesh(), name="hga_dish", material="dish_white")
    model.articulation(
        "antenna_mast_swivel",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=antenna,
        origin=Origin(xyz=(1.15, 0.0, 0.51)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.5, lower=-2.62, upper=2.62),
    )

    # ---- tilting T-handle hand controller ----
    controller = model.part("hand_controller")
    controller.visual(
        Cylinder(0.012, 0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        name="controller_shaft",
        material="console_dark",
    )
    controller.visual(
        Box((0.035, 0.16, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        name="controller_grip",
        material="console_dark",
    )
    model.articulation(
        "hand_controller_tilt",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=controller,
        origin=Origin(xyz=(0.28, 0.0, 0.66)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.35, upper=0.35),
    )

    return model


# ----------------------------------------------------------------- tests ----
def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")

    # Intentional bearing embeds.
    for name, _, _ in WHEELS:
        ctx.allow_overlap(
            "chassis",
            f"{name}_wheel",
            elem_a=f"{name}_axle",
            elem_b="wheel_disc",
            reason="Stub axle is captured inside the spinning wire-mesh wheel hub bearing.",
        )
    ctx.allow_overlap(
        "chassis",
        "high_gain_antenna",
        elem_a="hga_mast_base",
        elem_b="hga_mast_tube",
        reason="Antenna mast is seated in its swivel base bearing.",
    )
    ctx.allow_overlap(
        "chassis",
        "hand_controller",
        elem_a="control_console",
        elem_b="controller_shaft",
        reason="T-handle shaft passes into the console boot.",
    )

    # Four wire-mesh wheels: continuous spin about the lateral axis, on the ground.
    for name, wx, s in WHEELS:
        joint = object_model.get_articulation(f"{name}_wheel_spin")
        ctx.check(
            f"{name} wheel spin joint is continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"type={joint.articulation_type}",
        )
        ctx.check(
            f"{name} wheel spins about the lateral axis",
            abs(joint.axis[1]) > 0.99,
            details=f"axis={joint.axis}",
        )
        aabb = ctx.part_world_aabb(f"{name}_wheel")
        ctx.check(
            f"{name} tire rests on the lunar surface",
            aabb is not None and abs(aabb[0][2]) <= 0.02,
            details=f"aabb={aabb}",
        )
        ctx.expect_gap(
            "chassis",
            f"{name}_wheel",
            axis="z",
            positive_elem=f"{name}_fender_seg3",
            negative_elem="tire",
            min_gap=0.01,
            max_gap=0.15,
            name=f"{name} fender clears the tire",
        )
        ctx.expect_overlap(
            "chassis",
            f"{name}_wheel",
            axes="xy",
            elem_a=f"{name}_fender_seg3",
            elem_b="tire",
            min_overlap=0.05,
            name=f"{name} fender sits over the tire",
        )

    # High-gain umbrella antenna swivels on its front mast.
    hga = object_model.get_articulation("antenna_mast_swivel")
    ctx.check(
        "antenna mast swivel is revolute about z",
        hga.articulation_type == ArticulationType.REVOLUTE and abs(hga.axis[2]) > 0.99,
        details=f"type={hga.articulation_type} axis={hga.axis}",
    )
    dish_aabb = ctx.part_element_world_aabb("high_gain_antenna", elem="hga_dish")
    ctx.check(
        "umbrella dish present and close to 1 m wide",
        dish_aabb is not None and (dish_aabb[1][1] - dish_aabb[0][1]) > 0.85,
        details=f"dish aabb={dish_aabb}",
    )
    rest_cx = (dish_aabb[0][0] + dish_aabb[1][0]) / 2.0 if dish_aabb else 0.0
    with ctx.pose({hga: math.pi}):
        posed = ctx.part_element_world_aabb("high_gain_antenna", elem="hga_dish")
        posed_cx = (posed[0][0] + posed[1][0]) / 2.0 if posed else rest_cx
    ctx.check(
        "tilted dish sweeps around when the mast swivels",
        abs(posed_cx - rest_cx) > 0.04,
        details=f"rest_cx={rest_cx:.3f} posed_cx={posed_cx:.3f}",
    )

    # T-handle controller tilts forward.
    tilt = object_model.get_articulation("hand_controller_tilt")
    ctx.check(
        "hand controller tilt is revolute",
        tilt.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={tilt.articulation_type}",
    )
    grip_rest = ctx.part_element_world_aabb("hand_controller", elem="controller_grip")
    with ctx.pose({tilt: 0.3}):
        grip_posed = ctx.part_element_world_aabb("hand_controller", elem="controller_grip")
    rest_x = (grip_rest[0][0] + grip_rest[1][0]) / 2.0 if grip_rest else 0.0
    posed_x = (grip_posed[0][0] + grip_posed[1][0]) / 2.0 if grip_posed else rest_x
    ctx.check(
        "T-handle grip pitches forward with positive tilt",
        posed_x - rest_x > 0.02,
        details=f"rest_x={rest_x:.3f} posed_x={posed_x:.3f}",
    )

    # Seats with webbing straps, gold boxes up front.
    visual_names = {v.name for v in chassis.visuals if v.name}
    for side, _ in SEATS:
        ctx.check(f"seat {side} pan present", f"seat_{side}_pan" in visual_names)
        straps = [n for n in visual_names if n.startswith(f"seat_{side}_strap")]
        ctx.check(
            f"seat {side} backrest has webbing straps",
            len(straps) >= 4,
            details=f"straps={sorted(straps)}",
        )
    gold = [v for v in chassis.visuals if v.name and v.name.startswith("gold_instrument_box")]
    ctx.check(
        "gold-foil instrument boxes cluster at the front",
        len(gold) >= 2 and all(v.origin.xyz[0] > 0.7 for v in gold),
        details=f"gold boxes={[v.name for v in gold]}",
    )
    fenders = [n for n in visual_names if "_fender_seg" in n]
    ctx.check(
        "each wheel carries a segmented fender arc",
        len(fenders) == 4 * FENDER_SEGS,
        details=f"count={len(fenders)}",
    )

    return ctx.report()


object_model = build_object_model()
