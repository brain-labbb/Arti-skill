from __future__ import annotations

"""Industrial single-needle lockstitch flatbed sewing machine on a steel K-leg power table.

Layout (world frame):
- +X is the operator's right (handwheel / motor side), needle end is -X.
- -Y is the front (operator side), +Y is the back.
- +Z is up, floor at z = 0.

Articulated mechanisms:
- spoked nickel balance handwheel (continuous about +X)
- needle bar (prismatic, positive drives the needle down to the stitch plate)
- motor pulley (continuous about +X, drives the handwheel via V-belt)
"""

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    WheelBore,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)

# ---------------------------------------------------------------- dimensions
FLOOR = 0.0
TABLE_TOP_Z = 0.790                       # table and bedplate top surface
TABLE_THICK = 0.038
TABLE_SIZE = (1.08, 0.55, TABLE_THICK)
BED_XC = 0.05                             # bed center X (toward handwheel)
BED_SIZE = (0.40, 0.19, TABLE_THICK)      # industrial flat bedplate
ARM_Z = 0.972                             # arm axis height
HANDWHEEL_X = 0.250                       # balance wheel centre X
NEEDLE_X = -0.085
NEEDLE_JOINT_Z = 0.867
NEEDLE_TRAVEL = 0.011
STITCH_PLATE_Z = 0.791

# K-frame
LEG_TUBE = 0.040                          # square-tube side
FRAME_HX = 0.46                           # half-width of leg frame
FRAME_HY = 0.22                           # half-depth of leg frame
LEG_H = TABLE_TOP_Z - TABLE_THICK         # legs from floor to table underside

# Motor
MOTOR_CX = 0.30                           # motor body centre X
MOTOR_CY = 0.06                           # slightly behind centre
MOTOR_CZ = 0.60                           # below the table
MOTOR_SIZE = (0.20, 0.14, 0.14)
PULLEY_CX = HANDWHEEL_X                   # pulley aligned with handwheel
PULLEY_CY = MOTOR_CY
PULLEY_CZ = 0.52                          # motor shaft centre
PULLEY_R = 0.030
PULLEY_W = 0.022


def _shift(profile: list[tuple[float, float]], dx: float, dy: float) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _circle(cx: float, cy: float, r: float, n: int = 24) -> list[tuple[float, float]]:
    return [
        (cx + r * math.cos(2.0 * math.pi * i / n), cy + r * math.sin(2.0 * math.pi * i / n))
        for i in range(n)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_power_table_sewing_machine")

    # ---------------------------------------------------------------- materials
    steel_grey = model.material("steel_frame", rgba=(0.28, 0.28, 0.30, 1.0))
    table_top = model.material("laminate_white", rgba=(0.82, 0.82, 0.80, 1.0))
    head_paint = model.material("head_black_japanned", rgba=(0.06, 0.06, 0.07, 1.0))
    motor_green = model.material("motor_enamel", rgba=(0.30, 0.40, 0.32, 1.0))
    nickel = model.material("nickel_plate", rgba=(0.75, 0.76, 0.78, 1.0))
    rubber = model.material("belt_rubber", rgba=(0.10, 0.10, 0.10, 1.0))
    foot_rubber = model.material("foot_pad", rgba=(0.15, 0.15, 0.15, 1.0))

    # ============================================================ K-FRAME (root)
    k_frame = model.part("k_frame")

    for side, sx in enumerate((-FRAME_HX, FRAME_HX)):
        # front and rear vertical legs
        for end, ey in enumerate((-FRAME_HY, FRAME_HY)):
            k_frame.visual(
                Box((LEG_TUBE, LEG_TUBE, LEG_H)),
                origin=Origin(xyz=(sx, ey, LEG_H / 2.0)),
                material=steel_grey,
                name=f"leg_{side}_{end}",
            )
            # rubber foot pad
            k_frame.visual(
                Cylinder(radius=0.022, length=0.008),
                origin=Origin(xyz=(sx, ey, 0.004)),
                material=foot_rubber,
                name=f"foot_{side}_{end}",
            )
        # top rail (connects front and rear legs at table underside)
        k_frame.visual(
            Box((LEG_TUBE, 2.0 * FRAME_HY, LEG_TUBE)),
            origin=Origin(xyz=(sx, 0.0, LEG_H - LEG_TUBE / 2.0)),
            material=steel_grey,
            name=f"top_rail_{side}",
        )
        # lower stretcher at mid-height
        k_frame.visual(
            Box((LEG_TUBE, 2.0 * FRAME_HY, LEG_TUBE)),
            origin=Origin(xyz=(sx, 0.0, 0.18)),
            material=steel_grey,
            name=f"lower_stretcher_{side}",
        )
        # diagonal K-brace (front-bottom to rear-top)
        brace_len = math.sqrt((2.0 * FRAME_HY) ** 2 + (LEG_H - 0.18 - LEG_TUBE) ** 2)
        brace_angle = math.atan2(LEG_H - 0.18 - LEG_TUBE, 2.0 * FRAME_HY)
        k_frame.visual(
            Box((LEG_TUBE * 0.75, brace_len, LEG_TUBE * 0.75)),
            origin=Origin(
                xyz=(sx, 0.0, 0.5 * (0.18 + LEG_H)),
                rpy=(brace_angle, 0.0, 0.0),
            ),
            material=steel_grey,
            name=f"k_brace_{side}",
        )

    # cross stretchers connecting the two sides at mid-height
    for i, cz in enumerate((0.18, 0.38)):
        k_frame.visual(
            Box((2.0 * FRAME_HX, LEG_TUBE, LEG_TUBE)),
            origin=Origin(xyz=(0.0, -FRAME_HY, cz)),
            material=steel_grey,
            name=f"front_cross_{i}",
        )
        k_frame.visual(
            Box((2.0 * FRAME_HX, LEG_TUBE, LEG_TUBE)),
            origin=Origin(xyz=(0.0, FRAME_HY, cz)),
            material=steel_grey,
            name=f"rear_cross_{i}",
        )

    # ============================================================ TABLE
    table = model.part("table")

    # table top with rectangular cutout for the machine bed
    table_profile = rounded_rect_profile(TABLE_SIZE[0], TABLE_SIZE[1], 0.015)
    cutout = _shift(
        rounded_rect_profile(BED_SIZE[0] + 0.004, BED_SIZE[1] + 0.004, 0.006),
        BED_XC, 0.0,
    )
    table_top_geom = ExtrudeWithHolesGeometry(
        table_profile, [cutout], TABLE_THICK, center=True,
    )
    table.visual(
        mesh_from_geometry(table_top_geom, "tabletop"),
        origin=Origin(xyz=(0.0, 0.0, TABLE_TOP_Z - TABLE_THICK / 2.0)),
        material=table_top,
        name="tabletop",
    )

    model.articulation(
        "frame_to_table",
        ArticulationType.FIXED,
        parent=k_frame,
        child=table,
    )

    # ============================================================ MACHINE HEAD
    head = model.part("machine_head")

    # industrial flat bedplate (recessed flush in the table cutout)
    head.visual(
        Box(BED_SIZE),
        origin=Origin(xyz=(BED_XC, 0.0, TABLE_TOP_Z - TABLE_THICK / 2.0)),
        material=head_paint,
        name="bed",
    )
    # pillar rising from the bed
    pillar_geom = ExtrudeGeometry.from_z0(
        rounded_rect_profile(0.080, 0.110, 0.02), 0.200,
    )
    head.visual(
        mesh_from_geometry(pillar_geom, "pillar"),
        origin=Origin(xyz=(0.19, 0.0, TABLE_TOP_Z)),
        material=head_paint,
        name="pillar",
    )
    # horizontal arm
    arm_geom = CapsuleGeometry(0.034, 0.24).rotate_y(math.pi / 2.0)
    head.visual(
        mesh_from_geometry(arm_geom, "head_arm"),
        origin=Origin(xyz=(0.05, 0.0, ARM_Z)),
        material=head_paint,
        name="arm",
    )
    # nose casting at the needle end
    head.visual(
        Box((0.06, 0.062, 0.13)),
        origin=Origin(xyz=(NEEDLE_X, 0.0, 0.925)),
        material=head_paint,
        name="nose",
    )
    # front faceplate
    head.visual(
        Box((0.005, 0.05, 0.11)),
        origin=Origin(xyz=(-0.116, 0.0, 0.925)),
        material=nickel,
        name="faceplate",
    )
    # thread tension assembly
    head.visual(
        Cylinder(radius=0.011, length=0.012),
        origin=Origin(xyz=(NEEDLE_X, -0.036, 0.94), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=nickel,
        name="tension_knob",
    )
    # stitch plate with needle hole
    plate_geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.07, 0.05, 0.008),
        [_circle(-0.005, 0.0, 0.004)],
        0.003,
        center=True,
    )
    head.visual(
        mesh_from_geometry(plate_geom, "stitch_plate"),
        origin=Origin(xyz=(-0.08, 0.0, STITCH_PLATE_Z)),
        material=nickel,
        name="stitch_plate",
    )
    # slide plate
    head.visual(
        Box((0.04, 0.05, 0.0025)),
        origin=Origin(xyz=(-0.025, 0.0, STITCH_PLATE_Z)),
        material=nickel,
        name="slide_plate",
    )
    # spool pin
    head.visual(
        Cylinder(radius=0.003, length=0.05),
        origin=Origin(xyz=(0.14, 0.0, 1.025)),
        material=nickel,
        name="spool_pin",
    )
    # presser bar and foot
    head.visual(
        Cylinder(radius=0.0032, length=0.08),
        origin=Origin(xyz=(-0.065, 0.012, 0.836)),
        material=nickel,
        name="presser_bar",
    )
    head.visual(
        Box((0.028, 0.018, 0.004)),
        origin=Origin(xyz=(-0.065, 0.012, STITCH_PLATE_Z + 0.002)),
        material=nickel,
        name="presser_foot",
    )

    model.articulation(
        "table_to_head",
        ArticulationType.FIXED,
        parent=table,
        child=head,
    )

    # ============================================================ BALANCE HANDWHEEL
    handwheel = model.part("handwheel")
    hw_geom = WheelGeometry(
        0.088,
        0.024,
        rim=WheelRim(inner_radius=0.072),
        hub=WheelHub(radius=0.015, width=0.028, cap_style="domed"),
        spokes=WheelSpokes(style="straight", count=6, thickness=0.005, window_radius=0.008),
        bore=WheelBore(style="round", diameter=0.008),
    )
    handwheel.visual(
        mesh_from_geometry(hw_geom, "handwheel"),
        material=nickel,
        name="wheel",
    )
    handwheel.visual(
        Cylinder(radius=0.007, length=0.038),
        origin=Origin(xyz=(-0.019, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=nickel,
        name="axle",
    )
    model.articulation(
        "head_to_handwheel",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=handwheel,
        origin=Origin(xyz=(HANDWHEEL_X, 0.0, ARM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=20.0),
    )

    # ============================================================ NEEDLE BAR
    needle_bar = model.part("needle_bar")
    needle_bar.visual(
        Cylinder(radius=0.0035, length=0.11),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=nickel,
        name="bar",
    )
    needle_bar.visual(
        Cylinder(radius=0.006, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
        material=nickel,
        name="clamp",
    )
    needle_bar.visual(
        Cylinder(radius=0.0012, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.0525)),
        material=nickel,
        name="needle",
    )
    model.articulation(
        "head_to_needle_bar",
        ArticulationType.PRISMATIC,
        parent=head,
        child=needle_bar,
        origin=Origin(xyz=(NEEDLE_X, 0.0, NEEDLE_JOINT_Z)),
        # Positive q drives the needle down toward the stitch plate.
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.5, lower=0.0, upper=NEEDLE_TRAVEL),
    )

    # ============================================================ MOTOR BOX
    motor = model.part("motor_box")
    # motor body
    motor.visual(
        Box(MOTOR_SIZE),
        origin=Origin(xyz=(MOTOR_CX, MOTOR_CY, MOTOR_CZ)),
        material=motor_green,
        name="motor_body",
    )
    # motor end-bell (cylindrical cap on the shaft side)
    _motor_left = MOTOR_CX - MOTOR_SIZE[0] / 2.0
    motor.visual(
        Cylinder(radius=0.055, length=0.03),
        origin=Origin(
            xyz=(_motor_left - 0.015, MOTOR_CY, MOTOR_CZ),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=motor_green,
        name="end_bell",
    )
    # mounting bracket plate bolted to underside of table
    motor.visual(
        Box((0.24, 0.16, 0.006)),
        origin=Origin(xyz=(MOTOR_CX, MOTOR_CY, MOTOR_CZ + MOTOR_SIZE[2] / 2.0 + 0.003)),
        material=steel_grey,
        name="mount_plate",
    )
    # motor shaft stub extending toward the handwheel pulley
    _shaft_left = _motor_left - 0.030
    _shaft_len = PULLEY_CX - _shaft_left
    motor.visual(
        Cylinder(radius=0.008, length=_shaft_len),
        origin=Origin(
            xyz=(_shaft_left + _shaft_len / 2.0, MOTOR_CY, MOTOR_CZ),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=nickel,
        name="shaft",
    )

    model.articulation(
        "table_to_motor",
        ArticulationType.FIXED,
        parent=table,
        child=motor,
    )

    # ============================================================ MOTOR PULLEY
    pulley = model.part("motor_pulley")
    # pulley body
    pulley.visual(
        Cylinder(radius=PULLEY_R, length=PULLEY_W),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=nickel,
        name="pulley_body",
    )
    # V-groove (torus inset)
    pulley.visual(
        Cylinder(radius=PULLEY_R - 0.005, length=PULLEY_W + 0.004),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel_grey,
        name="pulley_groove",
    )
    model.articulation(
        "motor_to_pulley",
        ArticulationType.CONTINUOUS,
        parent=motor,
        child=pulley,
        origin=Origin(xyz=(PULLEY_CX, PULLEY_CY, PULLEY_CZ)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=30.0),
    )

    # ============================================================ V-BELT
    belt = model.part("v_belt")
    # Belt loop from motor pulley up to the handwheel.
    # Approximate the path as a closed spline in the YZ plane at x ≈ HANDWHEEL_X.
    hw_r = 0.088   # handwheel effective belt radius
    pr = PULLEY_R   # motor pulley radius
    belt_points = [
        # tight side (front) - motor pulley up to handwheel
        (HANDWHEEL_X, PULLEY_CY + pr, PULLEY_CZ),
        (HANDWHEEL_X, PULLEY_CY + 0.5 * (pr + hw_r), 0.5 * (PULLEY_CZ + ARM_Z)),
        (HANDWHEEL_X, hw_r, ARM_Z),
        # arc over handwheel top
        (HANDWHEEL_X, 0.65 * hw_r, ARM_Z + 0.75 * hw_r),
        (HANDWHEEL_X, 0.0, ARM_Z + hw_r),
        (HANDWHEEL_X, -0.65 * hw_r, ARM_Z + 0.75 * hw_r),
        (HANDWHEEL_X, -hw_r, ARM_Z),
        # slack side (back) - handwheel down to motor pulley
        (HANDWHEEL_X, -0.5 * (pr + hw_r), 0.5 * (PULLEY_CZ + ARM_Z)),
        (HANDWHEEL_X, -(PULLEY_CY + pr), PULLEY_CZ),
        # arc under motor pulley bottom
        (HANDWHEEL_X, -0.5 * pr, PULLEY_CZ - 0.85 * pr),
        (HANDWHEEL_X, 0.0, PULLEY_CZ - pr),
        (HANDWHEEL_X, 0.5 * pr, PULLEY_CZ - 0.85 * pr),
    ]
    belt_geom = tube_from_spline_points(
        belt_points,
        radius=0.004,
        closed_spline=True,
        up_hint=(1.0, 0.0, 0.0),
    )
    belt.visual(
        mesh_from_geometry(belt_geom, "belt_loop"),
        material=rubber,
        name="belt_loop",
    )
    model.articulation(
        "table_to_belt",
        ArticulationType.FIXED,
        parent=table,
        child=belt,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    k_frame = object_model.get_part("k_frame")
    table = object_model.get_part("table")
    head = object_model.get_part("machine_head")
    handwheel = object_model.get_part("handwheel")
    needle_bar = object_model.get_part("needle_bar")
    motor = object_model.get_part("motor_box")
    pulley = object_model.get_part("motor_pulley")
    belt = object_model.get_part("v_belt")

    hw_joint = object_model.get_articulation("head_to_handwheel")
    needle_joint = object_model.get_articulation("head_to_needle_bar")
    pulley_joint = object_model.get_articulation("motor_to_pulley")

    # ---- intentional overlap allowances ----
    ctx.allow_overlap(
        handwheel, head, elem_a="axle", elem_b="pillar",
        reason="Handwheel axle is intentionally seated in the pillar bearing bore.",
    )
    ctx.allow_overlap(
        needle_bar, head, elem_a="bar", elem_b="nose",
        reason="Needle bar intentionally slides inside the head nose bore.",
    )
    ctx.allow_overlap(
        head, table, elem_a="bed", elem_b="tabletop",
        reason="Industrial bedplate is intentionally recessed flush into the table cutout.",
    )
    ctx.allow_overlap(
        handwheel, belt, elem_a="wheel", elem_b="belt_loop",
        reason="V-belt intentionally wraps around the handwheel rim groove.",
    )
    ctx.allow_overlap(
        pulley, belt, elem_a="pulley_body", elem_b="belt_loop",
        reason="V-belt intentionally wraps around the motor pulley.",
    )
    ctx.allow_overlap(
        motor, pulley, elem_a="motor_body", elem_b="pulley_groove",
        reason="Motor pulley groove is intentionally recessed into the motor shaft area.",
    )
    ctx.allow_overlap(
        motor, pulley, elem_a="motor_body", elem_b="pulley_body",
        reason="Motor pulley is intentionally mounted directly on the motor shaft stub.",
    )
    ctx.allow_overlap(
        motor, belt, elem_a="mount_plate", elem_b="belt_loop",
        reason="V-belt passes close to the motor mounting bracket on its way to the handwheel.",
    )
    ctx.allow_overlap(
        motor, belt, elem_a="motor_body", elem_b="belt_loop",
        reason="V-belt run passes near the motor body in the drive assembly.",
    )
    ctx.allow_overlap(
        table, belt, elem_a="tabletop", elem_b="belt_loop",
        reason="V-belt passes through the table bed cutout area from motor to handwheel.",
    )
    ctx.allow_overlap(
        pulley, belt, elem_a="pulley_groove", elem_b="belt_loop",
        reason="V-belt is intentionally seated in the motor pulley V-groove.",
    )

    # ---- K-frame reaches the floor and supports the table ----
    frame_aabb = ctx.part_world_aabb(k_frame)
    ctx.check(
        "k_frame legs reach the floor",
        frame_aabb is not None and -0.002 <= frame_aabb[0][2] <= 0.006,
        details=f"k_frame_aabb={frame_aabb!r}",
    )
    ctx.expect_contact(table, k_frame, contact_tol=0.002,
                       name="table sits on the K-frame top rails")

    # ---- machine head bedplate is flush with the table surface ----
    bed_aabb = ctx.part_element_world_aabb(head, elem="bed")
    tabletop_aabb = ctx.part_element_world_aabb(table, elem="tabletop")
    ctx.check(
        "bed plate is recessed flush in the table cutout",
        bed_aabb is not None and tabletop_aabb is not None
        and abs(bed_aabb[1][2] - tabletop_aabb[1][2]) < 0.002,
        details=f"bed_top={bed_aabb[1][2]:.4f} table_top={tabletop_aabb[1][2]:.4f}"
        if bed_aabb and tabletop_aabb else "missing aabb",
    )

    # ---- overall real-world scale ----
    full_min = [math.inf] * 3
    full_max = [-math.inf] * 3
    for p in (k_frame, table, head, handwheel, motor, belt):
        aabb = ctx.part_world_aabb(p)
        if aabb is None:
            continue
        for i in range(3):
            full_min[i] = min(full_min[i], aabb[0][i])
            full_max[i] = max(full_max[i], aabb[1][i])
    width = full_max[0] - full_min[0]
    depth = full_max[1] - full_min[1]
    height = full_max[2] - full_min[2]
    ctx.check("table width ~1.08 m", 1.04 <= width <= 1.12, details=f"width={width:.3f}")
    ctx.check("depth ~0.55 m", 0.50 <= depth <= 0.60, details=f"depth={depth:.3f}")
    ctx.check("height ~1.06 m", 1.00 <= height <= 1.12, details=f"height={height:.3f}")

    # ---- balance handwheel (preserved) ----
    ctx.check(
        "handwheel spins continuously about X",
        str(hw_joint.articulation_type).lower().endswith("continuous")
        and abs(hw_joint.axis[0]) == 1.0,
        details=f"type={hw_joint.articulation_type!r} axis={hw_joint.axis!r}",
    )
    hw_aabb = ctx.part_element_world_aabb(handwheel, elem="wheel")
    if ctx.check("handwheel aabb present", hw_aabb is not None):
        hw_d = hw_aabb[1][2] - hw_aabb[0][2]
        ctx.check("handwheel diameter ~0.176 m", 0.168 <= hw_d <= 0.186,
                  details=f"diameter={hw_d:.3f}")

    # ---- needle bar (preserved) ----
    needle_rest = ctx.part_element_world_aabb(needle_bar, elem="needle")
    if ctx.check("needle present", needle_rest is not None):
        tip = needle_rest[0][2]
        ctx.check("needle tip rests above the stitch plate",
                  0.798 <= tip <= 0.810,
                  details=f"tip_z={tip:.4f}")
    ctx.expect_within(
        needle_bar, head, axes="xy", inner_elem="needle", outer_elem="stitch_plate",
        margin=0.0, name="needle is aligned over the stitch plate hole",
    )
    with ctx.pose({needle_joint: NEEDLE_TRAVEL}):
        needle_down = ctx.part_element_world_aabb(needle_bar, elem="needle")
        if ctx.check("needle aabb at full stroke", needle_down is not None):
            tip = needle_down[0][2]
            ctx.check(
                "needle descends toward the stitch plate",
                0.787 <= tip <= 0.800,
                details=f"tip_z={tip:.4f}",
            )
        ctx.expect_overlap(
            needle_bar, head, axes="z", elem_a="bar", elem_b="nose",
            min_overlap=0.03, name="needle bar stays inserted in the head at full stroke",
        )

    # ---- motor box under the table ----
    motor_aabb = ctx.part_world_aabb(motor)
    ctx.check(
        "motor box hangs below the table",
        motor_aabb is not None and motor_aabb[1][2] < TABLE_TOP_Z - 0.01,
        details=f"motor_top_z={motor_aabb[1][2]:.3f}" if motor_aabb else "no aabb",
    )
    ctx.expect_gap(
        table, motor, axis="z",
        positive_elem="tabletop", negative_elem="motor_body",
        max_penetration=0.008,
        name="motor body is mounted under the table surface",
    )

    # ---- motor pulley continuous joint ----
    ctx.check(
        "motor pulley spins continuously about X",
        str(pulley_joint.articulation_type).lower().endswith("continuous")
        and abs(pulley_joint.axis[0]) == 1.0,
        details=f"type={pulley_joint.articulation_type!r} axis={pulley_joint.axis!r}",
    )
    pulley_aabb = ctx.part_element_world_aabb(pulley, elem="pulley_body")
    if ctx.check("motor pulley aabb present", pulley_aabb is not None):
        pd = pulley_aabb[1][2] - pulley_aabb[0][2]
        ctx.check("motor pulley diameter ~0.060 m", 0.050 <= pd <= 0.070,
                  details=f"diameter={pd:.3f}")
    # pulley sits below the table
    if pulley_aabb is not None:
        ctx.check(
            "motor pulley is below the table underside",
            pulley_aabb[1][2] < TABLE_TOP_Z - TABLE_THICK,
            details=f"pulley_top_z={pulley_aabb[1][2]:.3f}",
        )

    # ---- V-belt spans motor pulley to handwheel ----
    belt_aabb = ctx.part_element_world_aabb(belt, elem="belt_loop")
    if ctx.check("v-belt aabb present", belt_aabb is not None):
        belt_span = belt_aabb[1][2] - belt_aabb[0][2]
        ctx.check(
            "v-belt spans vertically between pulleys (~0.45 m)",
            0.35 <= belt_span <= 0.60,
            details=f"belt_z_span={belt_span:.3f}",
        )
        ctx.check(
            "v-belt top reaches the handwheel height",
            belt_aabb[1][2] >= ARM_Z + 0.04,
            details=f"belt_top_z={belt_aabb[1][2]:.3f}",
        )
        ctx.check(
            "v-belt bottom reaches the motor pulley",
            belt_aabb[0][2] <= PULLEY_CZ + 0.01,
            details=f"belt_bot_z={belt_aabb[0][2]:.3f}",
        )

    # ---- proof checks for intentional overlaps ----
    # Belt wraps around the handwheel in XY
    ctx.expect_overlap(
        belt, handwheel, axes="yz", elem_a="belt_loop", elem_b="wheel",
        min_overlap=0.05,
        name="v-belt wraps around the handwheel in the drive plane",
    )
    # Belt wraps around the motor pulley
    ctx.expect_overlap(
        belt, pulley, axes="yz", elem_a="belt_loop", elem_b="pulley_body",
        min_overlap=0.02,
        name="v-belt wraps around the motor pulley",
    )
    # Motor pulley is near the motor shaft area
    ctx.expect_within(
        pulley, motor, axes="z", inner_elem="pulley_body", outer_elem="motor_body",
        margin=0.10,
        name="motor pulley is vertically near the motor shaft axis",
    )

    return ctx.report()


object_model = build_object_model()
