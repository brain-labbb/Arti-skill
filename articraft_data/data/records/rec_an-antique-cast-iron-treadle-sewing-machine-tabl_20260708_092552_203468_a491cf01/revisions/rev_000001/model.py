from __future__ import annotations

"""Antique cast-iron treadle sewing machine table.

Layout (world frame):
- +X is the operator's right (handwheel / band-wheel side), needle end is -X.
- -Y is the front (operator side, drawer fronts), +Y is the back.
- +Z is up, floor at z = 0.

Articulated mechanisms:
- rocking foot treadle (revolute about X, pin-in-boss pivots)
- large spoked band drive wheel (continuous about X, with crank pin)
- pitman rod on the treadle with an eye ring around the crank pin
- spoked nickel balance handwheel on the head (continuous about X)
- needle bar (prismatic, positive drives the needle down to the stitch plate)
- three dovetailed wooden drawers (prismatic, pull out toward the front)
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
    TorusGeometry,
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
TOP_Z0, TOP_Z1 = 0.725, 0.760          # oak tabletop slab
TOP_SIZE = (0.92, 0.46, 0.035)
CAB_XC = 0.38                          # cabinet centers at +/- CAB_XC
CAB_Z0, CAB_Z1 = 0.527, 0.727          # cabinet shells (embed 2 mm into top)
RAIL_TOP = 0.527                       # iron top rails carry the cabinets
BED_TOP = 0.784                        # machine bed top surface
ARM_Z = 0.972                          # horizontal arm axis height
WHEEL_C = (0.245, 0.02, 0.38)          # band drive wheel center
HANDWHEEL_X = 0.2475                   # balance wheel center
PIVOT = (0.0, 0.04, 0.11)              # treadle pivot line (axis along X)
NEEDLE_X, NEEDLE_JOINT_Z = -0.085, 0.86
NEEDLE_TRAVEL = 0.011
DRAWER_TRAVEL = 0.20


def _shift(profile: list[tuple[float, float]], dx: float, dy: float) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _circle(cx: float, cy: float, r: float, n: int = 24) -> list[tuple[float, float]]:
    return [
        (cx + r * math.cos(2.0 * math.pi * i / n), cy + r * math.sin(2.0 * math.pi * i / n))
        for i in range(n)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="treadle_sewing_machine_table")

    iron = model.material("cast_iron_black", rgba=(0.10, 0.10, 0.11, 1.0))
    head_paint = model.material("head_black_japanned", rgba=(0.05, 0.05, 0.06, 1.0))
    oak = model.material("oak_wood", rgba=(0.45, 0.28, 0.14, 1.0))
    dark_wood = model.material("dark_stained_wood", rgba=(0.30, 0.17, 0.09, 1.0))
    nickel = model.material("nickel_plate", rgba=(0.75, 0.76, 0.78, 1.0))
    gold = model.material("gold_decal", rgba=(0.80, 0.62, 0.20, 1.0))

    # ------------------------------------------------------------ treadle stand (root)
    stand = model.part("treadle_stand")

    for side, sx in enumerate((-0.31, 0.31)):
        legs = tube_from_spline_points(
            [
                (sx, -0.19, 0.505),
                (sx, -0.15, 0.43),
                (sx, -0.088, 0.30),
                (sx, -0.10, 0.175),
                (sx, -0.155, 0.06),
                (sx, -0.175, 0.028),
            ],
            radius=0.014,
            up_hint=(1.0, 0.0, 0.0),
        )
        legs.merge(
            tube_from_spline_points(
                [
                    (sx, 0.19, 0.505),
                    (sx, 0.15, 0.43),
                    (sx, 0.088, 0.30),
                    (sx, 0.10, 0.175),
                    (sx, 0.155, 0.06),
                    (sx, 0.175, 0.028),
                ],
                radius=0.014,
                up_hint=(1.0, 0.0, 0.0),
            )
        )
        # casters under the splayed feet
        for fy in (-0.175, 0.175):
            legs.merge(
                CylinderGeometry(0.018, 0.02).rotate_y(math.pi / 2.0).translate(sx, fy, 0.018)
            )
        stand.visual(
            mesh_from_geometry(legs, f"side_frame_{side}"),
            material=iron,
            name=f"side_frame_{side}",
        )
        stand.visual(
            Box((0.035, 0.42, 0.03)),
            origin=Origin(xyz=(sx, 0.0, 0.512)),
            material=iron,
            name=f"top_rail_{side}",
        )
        stand.visual(
            Box((0.03, 0.36, 0.025)),
            origin=Origin(xyz=(sx, 0.0, 0.10)),
            material=iron,
            name=f"bottom_rail_{side}",
        )
        # treadle pivot bearing boss on the bottom rail inner face
        stand.visual(
            Cylinder(radius=0.012, length=0.025),
            origin=Origin(xyz=(math.copysign(0.2875, sx), PIVOT[1], PIVOT[2]),
                          rpy=(0.0, math.pi / 2.0, 0.0)),
            material=iron,
            name=f"pivot_boss_{side}",
        )

    # cross braces tying the two side frames together
    stand.visual(
        Cylinder(radius=0.011, length=0.63),
        origin=Origin(xyz=(0.0, 0.16, 0.495), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="rear_top_brace",
    )
    for i, by in enumerate((-0.0965, 0.0965)):
        stand.visual(
            Cylinder(radius=0.011, length=0.64),
            origin=Origin(xyz=(0.0, by, 0.20), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=iron,
            name=f"lower_brace_{i}",
        )

    # band-wheel support bar and bearing boss on the right side frame
    stand.visual(
        Box((0.022, 0.30, 0.028)),
        origin=Origin(xyz=(0.31, 0.02, 0.38)),
        material=iron,
        name="wheel_support_bar",
    )
    stand.visual(
        Cylinder(radius=0.014, length=0.03),
        origin=Origin(xyz=(0.29, WHEEL_C[1], WHEEL_C[2]), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="bearing_boss",
    )

    # ------------------------------------------------------------ oak table
    table = model.part("table")
    table.visual(
        Box(TOP_SIZE),
        origin=Origin(xyz=(0.0, 0.0, 0.5 * (TOP_Z0 + TOP_Z1))),
        material=oak,
        name="tabletop",
    )
    table.visual(
        Box((0.61, 0.018, 0.10)),
        origin=Origin(xyz=(0.0, 0.195, 0.677)),
        material=dark_wood,
        name="rear_apron",
    )
    table.visual(
        Box((0.61, 0.018, 0.047)),
        origin=Origin(xyz=(0.0, -0.195, 0.7035)),
        material=dark_wood,
        name="front_rail",
    )

    def add_cabinet(prefix: str, xc: float) -> None:
        zc = 0.5 * (CAB_Z0 + CAB_Z1)
        for i, wx in enumerate((xc - 0.074, xc + 0.074)):
            table.visual(
                Box((0.012, 0.42, CAB_Z1 - CAB_Z0)),
                origin=Origin(xyz=(wx, 0.0, zc)),
                material=dark_wood,
                name=f"{prefix}_wall_{i}",
            )
        table.visual(
            Box((0.144, 0.012, CAB_Z1 - CAB_Z0)),
            origin=Origin(xyz=(xc, 0.204, zc)),
            material=dark_wood,
            name=f"{prefix}_back",
        )
        table.visual(
            Box((0.144, 0.414, 0.012)),
            origin=Origin(xyz=(xc, -0.003, CAB_Z0 + 0.006)),
            material=dark_wood,
            name=f"{prefix}_floor",
        )
        table.visual(
            Box((0.144, 0.414, 0.012)),
            origin=Origin(xyz=(xc, -0.003, CAB_Z1 - 0.006)),
            material=dark_wood,
            name=f"{prefix}_ceiling",
        )
        table.visual(
            Box((0.144, 0.414, 0.010)),
            origin=Origin(xyz=(xc, -0.003, zc)),
            material=dark_wood,
            name=f"{prefix}_divider",
        )

    add_cabinet("left_cabinet", -CAB_XC)
    add_cabinet("right_cabinet", CAB_XC)

    model.articulation(
        "stand_to_table",
        ArticulationType.FIXED,
        parent=stand,
        child=table,
    )

    # ------------------------------------------------------------ machine head
    head = model.part("machine_head")
    head.visual(
        Box((0.36, 0.17, 0.024)),
        origin=Origin(xyz=(0.05, 0.0, 0.772)),
        material=head_paint,
        name="bed",
    )
    pillar_geom = ExtrudeGeometry.from_z0(rounded_rect_profile(0.075, 0.105, 0.02), 0.223)
    head.visual(
        mesh_from_geometry(pillar_geom, "pillar"),
        origin=Origin(xyz=(0.19, 0.0, 0.782)),
        material=head_paint,
        name="pillar",
    )
    arm_geom = CapsuleGeometry(0.033, 0.24).rotate_y(math.pi / 2.0)
    head.visual(
        mesh_from_geometry(arm_geom, "head_arm"),
        origin=Origin(xyz=(0.05, 0.0, ARM_Z)),
        material=head_paint,
        name="arm",
    )
    head.visual(
        Box((0.06, 0.062, 0.13)),
        origin=Origin(xyz=(NEEDLE_X, 0.0, 0.925)),
        material=head_paint,
        name="nose",
    )
    head.visual(
        Box((0.005, 0.05, 0.11)),
        origin=Origin(xyz=(-0.116, 0.0, 0.925)),
        material=nickel,
        name="faceplate",
    )
    head.visual(
        Cylinder(radius=0.011, length=0.012),
        origin=Origin(xyz=(NEEDLE_X, -0.036, 0.94), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=nickel,
        name="tension_knob",
    )
    # gold decal accents on the japanned surfaces
    head.visual(
        Box((0.14, 0.003, 0.028)),
        origin=Origin(xyz=(0.04, -0.0325, ARM_Z)),
        material=gold,
        name="arm_decal",
    )
    head.visual(
        Box((0.10, 0.05, 0.002)),
        origin=Origin(xyz=(0.05, -0.05, 0.7845)),
        material=gold,
        name="bed_decal",
    )
    # stitch plate with a needle hole at the needle drop point
    plate_geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.07, 0.05, 0.008),
        [_circle(-0.005, 0.0, 0.004)],
        0.003,
        center=True,
    )
    head.visual(
        mesh_from_geometry(plate_geom, "stitch_plate"),
        origin=Origin(xyz=(-0.08, 0.0, 0.785)),
        material=nickel,
        name="stitch_plate",
    )
    head.visual(
        Box((0.04, 0.05, 0.0025)),
        origin=Origin(xyz=(-0.025, 0.0, 0.785)),
        material=nickel,
        name="slide_plate",
    )
    head.visual(
        Cylinder(radius=0.003, length=0.05),
        origin=Origin(xyz=(0.14, 0.0, 1.025)),
        material=nickel,
        name="spool_pin",
    )
    head.visual(
        Cylinder(radius=0.0032, length=0.08),
        origin=Origin(xyz=(-0.065, 0.012, 0.836)),
        material=nickel,
        name="presser_bar",
    )
    head.visual(
        Box((0.028, 0.018, 0.004)),
        origin=Origin(xyz=(-0.065, 0.012, 0.795)),
        material=nickel,
        name="presser_foot",
    )

    model.articulation(
        "table_to_head",
        ArticulationType.FIXED,
        parent=table,
        child=head,
    )

    # ------------------------------------------------------------ balance handwheel
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

    # ------------------------------------------------------------ needle bar
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

    # ------------------------------------------------------------ band drive wheel
    band_wheel = model.part("band_wheel")
    bw_geom = WheelGeometry(
        0.15,
        0.026,
        rim=WheelRim(inner_radius=0.132),
        hub=WheelHub(radius=0.018, width=0.032, cap_style="flat"),
        spokes=WheelSpokes(style="straight", count=6, thickness=0.006, window_radius=0.010),
        bore=WheelBore(style="round", diameter=0.012),
    )
    band_wheel.visual(
        mesh_from_geometry(bw_geom, "band_wheel"),
        material=iron,
        name="wheel",
    )
    band_wheel.visual(
        Cylinder(radius=0.008, length=0.052),
        origin=Origin(xyz=(0.026, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="axle",
    )
    band_wheel.visual(
        Box((0.008, 0.018, 0.072)),
        origin=Origin(xyz=(-0.009, 0.0, -0.0275)),
        material=iron,
        name="crank_arm",
    )
    band_wheel.visual(
        Cylinder(radius=0.0055, length=0.03),
        origin=Origin(xyz=(-0.026, 0.0, -0.055), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="crank_pin",
    )
    model.articulation(
        "stand_to_band_wheel",
        ArticulationType.CONTINUOUS,
        parent=stand,
        child=band_wheel,
        origin=Origin(xyz=WHEEL_C),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=25.0),
    )

    # ------------------------------------------------------------ foot treadle
    treadle = model.part("treadle_pedal")
    lattice = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.50, 0.24, 0.02),
        [
            _shift(rounded_rect_profile(0.12, 0.055, 0.015), dx, dy)
            for dx in (-0.16, 0.0, 0.16)
            for dy in (-0.0625, 0.0625)
        ],
        0.010,
        center=True,
    )
    treadle.visual(
        mesh_from_geometry(lattice, "pedal_lattice"),
        origin=Origin(xyz=(0.0, -0.06, 0.0)),
        material=iron,
        name="pedal_plate",
    )
    treadle.visual(
        Cylinder(radius=0.008, length=0.50),
        origin=Origin(xyz=(0.0, -0.17, 0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="toe_bar",
    )
    for i, px in enumerate((-0.2625, 0.2625)):
        treadle.visual(
            Cylinder(radius=0.008, length=0.055),
            origin=Origin(xyz=(px, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=iron,
            name=f"pivot_pin_{i}",
        )
    rod = tube_from_spline_points(
        [
            (0.21, 0.045, 0.004),
            (0.214, 0.03, 0.06),
            (0.219, -0.005, 0.15),
            (0.221, -0.02, 0.202),
        ],
        radius=0.006,
        up_hint=(1.0, 0.0, 0.0),
    )
    treadle.visual(
        mesh_from_geometry(rod, "pitman_rod"),
        material=iron,
        name="pitman_rod",
    )
    eye = TorusGeometry(0.0115, 0.0035).rotate_y(math.pi / 2.0)
    treadle.visual(
        mesh_from_geometry(eye, "pitman_eye"),
        origin=Origin(xyz=(0.221, -0.02, 0.215)),
        material=iron,
        name="pitman_eye",
    )
    model.articulation(
        "stand_to_treadle",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=treadle,
        origin=Origin(xyz=PIVOT),
        # Positive q rocks the toe (front edge) down.
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=3.0, lower=-0.22, upper=0.22),
    )

    # ------------------------------------------------------------ drawers
    def add_drawer(name: str, xc: float, z_base: float) -> None:
        drawer = model.part(name)
        drawer.visual(
            Box((0.14, 0.014, 0.082)),
            origin=Origin(xyz=(0.0, -0.007, 0.035)),
            material=dark_wood,
            name="front_panel",
        )
        drawer.visual(
            Cylinder(radius=0.030, length=0.008),
            origin=Origin(xyz=(0.0, -0.017, 0.035), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_wood,
            name="medallion",
        )
        drawer.visual(
            Cylinder(radius=0.005, length=0.012),
            origin=Origin(xyz=(0.0, -0.025, 0.035), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_wood,
            name="knob_stem",
        )
        drawer.visual(
            Cylinder(radius=0.011, length=0.010),
            origin=Origin(xyz=(0.0, -0.034, 0.035), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_wood,
            name="knob",
        )
        drawer.visual(
            Box((0.126, 0.344, 0.008)),
            origin=Origin(xyz=(0.0, 0.170, 0.004)),
            material=oak,
            name="bottom",
        )
        for i, wx in enumerate((-0.058, 0.058)):
            drawer.visual(
                Box((0.010, 0.344, 0.058)),
                origin=Origin(xyz=(wx, 0.170, 0.033)),
                material=oak,
                name=f"side_wall_{i}",
            )
        drawer.visual(
            Box((0.126, 0.010, 0.058)),
            origin=Origin(xyz=(0.0, 0.337, 0.033)),
            material=oak,
            name="back_wall",
        )
        model.articulation(
            f"table_to_{name}",
            ArticulationType.PRISMATIC,
            parent=table,
            child=drawer,
            origin=Origin(xyz=(xc, -0.21, z_base)),
            # Positive q pulls the drawer out toward the front.
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=30.0, velocity=0.3, lower=0.0, upper=DRAWER_TRAVEL),
        )

    add_drawer("left_drawer_upper", -CAB_XC, 0.636)
    add_drawer("left_drawer_lower", -CAB_XC, 0.543)
    add_drawer("right_drawer", CAB_XC, 0.636)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("treadle_stand")
    table = object_model.get_part("table")
    head = object_model.get_part("machine_head")
    handwheel = object_model.get_part("handwheel")
    needle_bar = object_model.get_part("needle_bar")
    band_wheel = object_model.get_part("band_wheel")
    treadle = object_model.get_part("treadle_pedal")
    drawers = [
        object_model.get_part("left_drawer_upper"),
        object_model.get_part("left_drawer_lower"),
        object_model.get_part("right_drawer"),
    ]

    hw_joint = object_model.get_articulation("head_to_handwheel")
    needle_joint = object_model.get_articulation("head_to_needle_bar")
    bw_joint = object_model.get_articulation("stand_to_band_wheel")
    treadle_joint = object_model.get_articulation("stand_to_treadle")

    # Intentional captured-shaft embeddings.
    ctx.allow_overlap(
        handwheel, head, elem_a="axle", elem_b="pillar",
        reason="Handwheel axle is intentionally seated in the pillar bearing bore.",
    )
    ctx.allow_overlap(
        needle_bar, head, elem_a="bar", elem_b="nose",
        reason="Needle bar intentionally slides inside the head nose bore.",
    )
    ctx.allow_overlap(
        band_wheel, stand, elem_a="axle", elem_b="bearing_boss",
        reason="Band-wheel axle is intentionally captured by the stand bearing boss.",
    )
    for i in range(2):
        ctx.allow_overlap(
            treadle, stand, elem_a=f"pivot_pin_{i}", elem_b=f"pivot_boss_{i}",
            reason="Treadle pivot pin is intentionally seated in the frame bearing boss.",
        )

    # --- assembly seating -------------------------------------------------
    ctx.expect_contact(head, table, contact_tol=0.0005,
                       name="machine bed rests on the tabletop")
    ctx.expect_contact(table, stand, contact_tol=0.0005,
                       name="cabinets rest on the iron top rails")
    stand_aabb = ctx.part_world_aabb(stand)
    ctx.check(
        "casters reach the floor",
        stand_aabb is not None and -0.001 <= stand_aabb[0][2] <= 0.004,
        details=f"stand_aabb={stand_aabb!r}",
    )

    # --- overall real-world scale -----------------------------------------
    full_min = [math.inf] * 3
    full_max = [-math.inf] * 3
    for part in (stand, table, head, handwheel, band_wheel, treadle):
        aabb = ctx.part_world_aabb(part)
        if aabb is None:
            continue
        for i in range(3):
            full_min[i] = min(full_min[i], aabb[0][i])
            full_max[i] = max(full_max[i], aabb[1][i])
    width = full_max[0] - full_min[0]
    depth = full_max[1] - full_min[1]
    height = full_max[2] - full_min[2]
    ctx.check("table width ~0.92 m", 0.90 <= width <= 0.94, details=f"width={width:.3f}")
    ctx.check("depth ~0.46 m", 0.42 <= depth <= 0.52, details=f"depth={depth:.3f}")
    ctx.check("height ~1.06 m", 1.02 <= height <= 1.10, details=f"height={height:.3f}")

    # --- balance handwheel --------------------------------------------------
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
    ctx.expect_gap(
        handwheel, head, axis="x", min_gap=0.002, max_gap=0.02,
        positive_elem="wheel", negative_elem="pillar",
        name="handwheel clears the pillar face",
    )

    # --- needle bar ---------------------------------------------------------
    needle_rest = ctx.part_element_world_aabb(needle_bar, elem="needle")
    if ctx.check("needle present", needle_rest is not None):
        tip = needle_rest[0][2]
        ctx.check("needle tip rests above the stitch plate", 0.794 <= tip <= 0.799,
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
                "needle descends into the stitch plate hole, above the bed",
                0.7842 <= tip <= 0.788,
                details=f"tip_z={tip:.4f}",
            )
        ctx.expect_overlap(
            needle_bar, head, axes="z", elem_a="bar", elem_b="nose",
            min_overlap=0.03, name="needle bar stays inserted in the head at full stroke",
        )

    # --- band drive wheel and treadle ---------------------------------------
    bw_aabb = ctx.part_element_world_aabb(band_wheel, elem="wheel")
    if ctx.check("band wheel aabb present", bw_aabb is not None):
        bw_d = bw_aabb[1][2] - bw_aabb[0][2]
        ctx.check("band wheel diameter ~0.30 m", 0.29 <= bw_d <= 0.31,
                  details=f"diameter={bw_d:.3f}")
        ctx.check("band wheel sits below the tabletop", bw_aabb[1][2] <= 0.60,
                  details=f"top_z={bw_aabb[1][2]:.3f}")
    ctx.check(
        "band wheel spins continuously about X",
        str(bw_joint.articulation_type).lower().endswith("continuous")
        and abs(bw_joint.axis[0]) == 1.0,
        details=f"type={bw_joint.articulation_type!r} axis={bw_joint.axis!r}",
    )
    pin_rest = ctx.part_element_world_aabb(band_wheel, elem="crank_pin")
    with ctx.pose({bw_joint: math.pi}):
        pin_up = ctx.part_element_world_aabb(band_wheel, elem="crank_pin")
        ctx.check(
            "crank pin orbits the wheel axle",
            pin_rest is not None and pin_up is not None
            and (pin_up[0][2] - pin_rest[0][2]) > 0.06,
            details=f"rest={pin_rest!r} up={pin_up!r}",
        )

    # pitman eye ring encircles the crank pin with clearance at rest
    ctx.expect_within(
        treadle, band_wheel, axes="x", inner_elem="pitman_eye", outer_elem="crank_pin",
        margin=0.0005, name="pitman eye is on the crank pin along the axle",
    )
    ctx.expect_overlap(
        treadle, band_wheel, axes="yz", elem_a="pitman_eye", elem_b="crank_pin",
        min_overlap=0.004, name="pitman eye ring surrounds the crank pin",
    )
    ctx.expect_gap(
        treadle, band_wheel, axis="x", elem_a="pitman_eye", elem_b="crank_pin",
        min_gap=None, max_penetration=0.02,
        name="pitman eye and crank pin interleave without collision",
    )

    plate_rest = ctx.part_element_world_aabb(treadle, elem="pedal_plate")
    with ctx.pose({treadle_joint: 0.18}):
        plate_down = ctx.part_element_world_aabb(treadle, elem="pedal_plate")
        ctx.check(
            "treadle toe rocks downward under positive pedal motion",
            plate_rest is not None and plate_down is not None
            and (plate_rest[0][2] - plate_down[0][2]) > 0.015,
            details=f"rest={plate_rest!r} down={plate_down!r}",
        )
        ctx.check(
            "rocked treadle stays above the floor",
            plate_down is not None and plate_down[0][2] > 0.01,
            details=f"down={plate_down!r}",
        )

    # --- drawers -------------------------------------------------------------
    for drawer in drawers:
        joint = object_model.get_articulation(f"table_to_{drawer.name}")
        ctx.expect_contact(
            drawer, table, contact_tol=0.001,
            name=f"{drawer.name} front sits flush against the cabinet",
        )
        rest_aabb = ctx.part_world_aabb(drawer)
        with ctx.pose({joint: DRAWER_TRAVEL}):
            open_aabb = ctx.part_world_aabb(drawer)
            ctx.check(
                f"{drawer.name} pulls out toward the front",
                rest_aabb is not None and open_aabb is not None
                and (rest_aabb[0][1] - open_aabb[0][1]) > 0.18,
                details=f"rest={rest_aabb!r} open={open_aabb!r}",
            )
            side = "left_cabinet" if drawer.name.startswith("left") else "right_cabinet"
            ctx.expect_overlap(
                drawer, table, axes="y", elem_b=f"{side}_wall_0",
                min_overlap=0.10,
                name=f"{drawer.name} keeps retained insertion when open",
            )
            ctx.expect_within(
                drawer, table, axes="x", inner_elem="back_wall", outer_elem=f"{side}_floor",
                margin=0.001,
                name=f"{drawer.name} stays centered in the cabinet cavity",
            )

    return ctx.report()


object_model = build_object_model()
