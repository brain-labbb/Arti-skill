from __future__ import annotations

# Tall ergonomic office chair with cantilever sled base (Eames-soft-pad / visitor-chair style).
# Variant: replaces the 5-spoke caster base and gas-lift column with a
# continuous bent-tubular-steel sled frame.
#
# Kinematic tree:
#   sled_base (root: bent tubular steel frame with floor runners and risers)
#     -> seat           CONTINUOUS swivel about Z (seat pan, armrest loops, mech)
#        -> backrest    REVOLUTE about Y (recline, 0..-0.25 rad)
#        -> tilt_lever  REVOLUTE paddle lever on the mechanism side

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)

# ---------------------------------------------------------------- dimensions
SEAT_MOUNT_Z = 0.40  # height of the seat swivel plate above floor

RECLINE_PIVOT = (-0.21, 0.0, -0.02)  # in the seat frame
RECLINE_RANGE = 0.25

# Sled base tube dimensions
TUBE_R = 0.015
RUNNER_Y = 0.22  # half-width of floor runners


def _sled_side_tube(sign: float, name: str):
    """One side of the cantilever sled: floor runner → heel curve → riser → top rail."""
    pts = [
        (0.24, sign * RUNNER_Y, 0.015),
        (-0.20, sign * RUNNER_Y, 0.015),
        (-0.26, sign * (RUNNER_Y - 0.01), 0.08),
        (-0.24, sign * 0.18, 0.22),
        (-0.16, sign * 0.14, 0.34),
        (-0.02, sign * 0.06, 0.39),
    ]
    geom = tube_from_spline_points(
        pts,
        radius=TUBE_R,
        samples_per_segment=16,
        radial_segments=14,
        cap_ends=True,
    )
    return mesh_from_geometry(geom, name)


def _crossbar(x: float, y_half: float, z: float, name: str):
    """Horizontal crossbar connecting left and right frame sides."""
    geom = tube_from_spline_points(
        [(x, -y_half, z), (x, y_half, z)],
        radius=TUBE_R,
        samples_per_segment=8,
        radial_segments=14,
        cap_ends=True,
    )
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="markus_sled_chair")

    plastic_black = model.material("plastic_black", rgba=(0.10, 0.10, 0.11, 1.0))
    mesh_black = model.material("mesh_black", rgba=(0.05, 0.05, 0.055, 1.0))
    fabric_black = model.material("fabric_black", rgba=(0.085, 0.085, 0.09, 1.0))
    chrome = model.material("chrome", rgba=(0.72, 0.73, 0.75, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.28, 0.29, 0.31, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.06, 0.06, 0.065, 1.0))

    # -------------------------------------------------------- sled base (root)
    sled_base = model.part("sled_base")

    # Two bent-tubular side frames (left and right)
    for i, sign in enumerate((1.0, -1.0)):
        sled_base.visual(
            _sled_side_tube(sign, f"sled_side_{i}"),
            material=chrome,
            name=f"sled_side_{i}",
        )

    # Crossbars: front feet, rear runners, mid-riser brace
    sled_base.visual(
        _crossbar(0.24, RUNNER_Y, 0.015, "front_crossbar"),
        material=chrome,
        name="front_crossbar",
    )
    sled_base.visual(
        _crossbar(-0.20, RUNNER_Y, 0.015, "rear_crossbar"),
        material=chrome,
        name="rear_crossbar",
    )
    sled_base.visual(
        _crossbar(-0.24, 0.18, 0.20, "mid_crossbar"),
        material=chrome,
        name="mid_crossbar",
    )

    # Swivel bearing plate at the top of the sled
    sled_base.visual(
        Cylinder(radius=0.10, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, SEAT_MOUNT_Z - 0.006)),
        material=steel_dark,
        name="swivel_plate",
    )

    # Rubber foot glides at the four floor contact points
    for sx, lx in ((0.24, "front"), (-0.20, "rear")):
        for sy, ly in ((RUNNER_Y, "left"), (-RUNNER_Y, "right")):
            sled_base.visual(
                Cylinder(radius=0.018, length=0.006),
                origin=Origin(xyz=(sx, sy, 0.003)),
                material=rubber_black,
                name=f"{lx}_{ly}_foot",
            )

    # ------------------------------------------------------------------ seat
    seat = model.part("seat")
    seat.visual(
        Box((0.26, 0.20, 0.075)),
        origin=Origin(xyz=(0.01, 0.0, -0.020)),
        material=plastic_black,
        name="mech_housing",
    )
    seat.visual(
        Box((0.090, 0.12, 0.050)),
        origin=Origin(xyz=(-0.150, 0.0, -0.020)),
        material=plastic_black,
        name="mech_rear_bracket",
    )

    pan_geom = ExtrudeGeometry(rounded_rect_profile(0.50, 0.46, 0.10), 0.07, center=True)
    pan_geom.translate(0.01, 0.0, 0.043)
    seat.visual(
        mesh_from_geometry(pan_geom, "seat_pan"),
        material=plastic_black,
        name="seat_pan",
    )
    cushion_geom = ExtrudeGeometry(rounded_rect_profile(0.44, 0.42, 0.09), 0.03, center=True)
    cushion_geom.translate(0.02, 0.0, 0.088)
    seat.visual(
        mesh_from_geometry(cushion_geom, "seat_cushion"),
        material=fabric_black,
        name="seat_cushion",
    )

    # Closed-loop armrests: flattened black plastic rings on angled stems
    for label, sign in (("left", 1.0), ("right", -1.0)):
        stem_geom = tube_from_spline_points(
            [
                (0.05, sign * 0.205, 0.015),
                (0.03, sign * 0.245, 0.070),
                (0.01, sign * 0.252, 0.130),
            ],
            radius=0.014,
            samples_per_segment=14,
            radial_segments=14,
            cap_ends=True,
        )
        seat.visual(
            mesh_from_geometry(stem_geom, f"armrest_stem_{label}"),
            material=plastic_black,
            name=f"{label}_armrest_stem",
        )
        ring_geom = TorusGeometry(radius=0.085, tube=0.016, radial_segments=14, tubular_segments=40)
        ring_geom.rotate_x(math.pi / 2.0)
        ring_geom.scale(1.0, 0.7, 1.0)
        ring_geom.translate(0.01, sign * 0.255, 0.215)
        seat.visual(
            mesh_from_geometry(ring_geom, f"armrest_loop_{label}"),
            material=plastic_black,
            name=f"{label}_armrest_loop",
        )

    model.articulation(
        "sled_to_seat",
        ArticulationType.CONTINUOUS,
        parent=sled_base,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, SEAT_MOUNT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=3.0),
    )

    # ------------------------------------------------------------- tilt lever
    lever = model.part("tilt_lever")
    lever.visual(
        Cylinder(radius=0.006, length=0.070),
        origin=Origin(xyz=(0.0, -0.020, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=plastic_black,
        name="lever_shaft",
    )
    lever.visual(
        Box((0.055, 0.035, 0.010)),
        origin=Origin(xyz=(0.0, -0.065, 0.0)),
        material=plastic_black,
        name="lever_paddle",
    )
    model.articulation(
        "seat_to_tilt_lever",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=lever,
        origin=Origin(xyz=(0.06, -0.10, -0.03)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=4.0, lower=0.0, upper=0.25),
    )

    # -------------------------------------------------------------- backrest
    backrest = model.part("backrest")
    backrest.visual(
        Cylinder(radius=0.022, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=plastic_black,
        name="pivot_barrel",
    )
    for label, sign in (("left", 1.0), ("right", -1.0)):
        backrest.visual(
            Box((0.090, 0.030, 0.040)),
            origin=Origin(xyz=(-0.045, sign * 0.045, 0.0)),
            material=plastic_black,
            name=f"{label}_pivot_arm",
        )
    backrest.visual(
        Box((0.050, 0.100, 0.180)),
        origin=Origin(xyz=(-0.060, 0.0, 0.070)),
        material=plastic_black,
        name="spine",
    )
    backrest.visual(
        Box((0.050, 0.420, 0.050)),
        origin=Origin(xyz=(-0.020, 0.0, 0.135)),
        material=plastic_black,
        name="bottom_rail",
    )

    # Curved side rails following the lumbar S-curve of the frame
    for label, sign in (("left", 1.0), ("right", -1.0)):
        rail_geom = tube_from_spline_points(
            [
                (-0.020, sign * 0.210, 0.130),
                (0.005, sign * 0.210, 0.320),
                (-0.030, sign * 0.205, 0.560),
                (-0.090, sign * 0.195, 0.840),
            ],
            radius=0.017,
            samples_per_segment=14,
            radial_segments=14,
            cap_ends=True,
        )
        backrest.visual(
            mesh_from_geometry(rail_geom, f"side_rail_{label}"),
            material=plastic_black,
            name=f"{label}_side_rail",
        )

    backrest.visual(
        Box((0.045, 0.400, 0.050)),
        origin=Origin(xyz=(-0.085, 0.0, 0.845), rpy=(0.0, -0.21, 0.0)),
        material=plastic_black,
        name="top_rail",
    )

    # Dark mesh centre panel in three curved segments inside the frame
    panel_specs = (
        ("mesh_panel_lower", (-0.016, 0.0, 0.245), 0.13, 0.24),
        ("mesh_panel_mid", (-0.0205, 0.0, 0.440), -0.145, 0.27),
        ("mesh_panel_upper", (-0.064, 0.0, 0.700), -0.21, 0.30),
    )
    for name, center, pitch, height in panel_specs:
        backrest.visual(
            Box((0.016, 0.400, height)),
            origin=Origin(xyz=center, rpy=(0.0, pitch, 0.0)),
            material=mesh_black,
            name=name,
        )

    # Padded pillow-like headrest capping the top of the frame
    headrest_geom = CapsuleGeometry(radius=0.075, length=0.26, radial_segments=20)
    headrest_geom.rotate_x(math.pi / 2.0)
    headrest_geom.scale(0.60, 1.0, 0.93)
    headrest_geom.rotate_y(-0.21)
    headrest_geom.translate(-0.105, 0.0, 0.900)
    backrest.visual(
        mesh_from_geometry(headrest_geom, "headrest_pillow"),
        material=fabric_black,
        name="headrest_pillow",
    )

    model.articulation(
        "seat_to_backrest",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=backrest,
        origin=Origin(xyz=RECLINE_PIVOT),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=200.0, velocity=1.0, lower=-RECLINE_RANGE, upper=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    sled_base = object_model.get_part("sled_base")
    seat = object_model.get_part("seat")
    backrest = object_model.get_part("backrest")
    lever = object_model.get_part("tilt_lever")
    swivel_joint = object_model.get_articulation("sled_to_seat")
    recline_joint = object_model.get_articulation("seat_to_backrest")
    lever_joint = object_model.get_articulation("seat_to_tilt_lever")

    # ----------------------------------------------------- intentional fits
    ctx.allow_overlap(
        sled_base,
        seat,
        elem_a="swivel_plate",
        elem_b="mech_housing",
        reason="Seat mechanism housing is seated on the sled swivel-plate bearing.",
    )
    ctx.allow_overlap(
        sled_base,
        seat,
        elem_a="sled_side_0",
        elem_b="mech_housing",
        reason="Left sled side tube rises into the mechanism housing to reach the swivel bearing.",
    )
    ctx.allow_overlap(
        sled_base,
        seat,
        elem_a="sled_side_1",
        elem_b="mech_housing",
        reason="Right sled side tube rises into the mechanism housing to reach the swivel bearing.",
    )
    ctx.allow_overlap(
        seat,
        backrest,
        elem_a="mech_rear_bracket",
        elem_b="pivot_barrel",
        reason="Backrest pivot barrel is captured in the mechanism rear bracket clevis.",
    )
    ctx.allow_overlap(
        seat,
        lever,
        elem_a="mech_housing",
        elem_b="lever_shaft",
        reason="Lever shaft passes into its bore in the mechanism housing.",
    )

    # ------------------------------------------------------- hero geometry
    cushion_aabb = ctx.part_element_world_aabb(seat, elem="seat_cushion")
    ctx.check(
        "seat top sits at office-chair height (0.44-0.55 m)",
        cushion_aabb is not None and 0.44 <= cushion_aabb[1][2] <= 0.55,
        details=f"cushion aabb={cushion_aabb}",
    )
    pan_aabb = ctx.part_element_world_aabb(seat, elem="seat_pan")
    ctx.check(
        "seat pan is about 0.5 x 0.45 m",
        pan_aabb is not None
        and 0.44 <= (pan_aabb[1][0] - pan_aabb[0][0]) <= 0.56
        and 0.40 <= (pan_aabb[1][1] - pan_aabb[0][1]) <= 0.52,
        details=f"pan aabb={pan_aabb}",
    )

    head_aabb = ctx.part_element_world_aabb(backrest, elem="headrest_pillow")
    ctx.check(
        "headrest pillow caps the backrest at 1.20-1.42 m",
        head_aabb is not None and 1.20 <= head_aabb[1][2] <= 1.42,
        details=f"headrest aabb={head_aabb}",
    )
    back_aabb = ctx.part_world_aabb(backrest)
    ctx.check(
        "slim backrest is about 0.45 m wide",
        back_aabb is not None and 0.40 <= (back_aabb[1][1] - back_aabb[0][1]) <= 0.50,
        details=f"backrest aabb={back_aabb}",
    )
    panel_aabb = ctx.part_element_world_aabb(backrest, elem="mesh_panel_mid")
    rail_aabb = ctx.part_element_world_aabb(backrest, elem="left_side_rail")
    ctx.check(
        "mesh center panel is framed inside the side rails",
        panel_aabb is not None
        and rail_aabb is not None
        and panel_aabb[1][1] <= rail_aabb[1][1] + 0.001,
        details=f"panel={panel_aabb}, rail={rail_aabb}",
    )

    for label, low, high in (("left", 0.22, 0.30), ("right", -0.30, -0.22)):
        loop_aabb = ctx.part_element_world_aabb(seat, elem=f"{label}_armrest_loop")
        cy = None if loop_aabb is None else 0.5 * (loop_aabb[0][1] + loop_aabb[1][1])
        ctx.check(
            f"{label} closed-loop armrest sits at the seat side",
            loop_aabb is not None
            and low <= cy <= high
            and 0.58 <= loop_aabb[1][2] <= 0.82,
            details=f"{label} loop aabb={loop_aabb}",
        )

    # --------------------------------------------------- sled base structure
    sled_aabb = ctx.part_world_aabb(sled_base)
    ctx.check(
        "sled base floor runners span a stable footprint",
        sled_aabb is not None
        and (sled_aabb[1][0] - sled_aabb[0][0]) >= 0.40
        and (sled_aabb[1][1] - sled_aabb[0][1]) >= 0.36,
        details=f"sled aabb={sled_aabb}",
    )
    ctx.check(
        "sled base rests on the floor",
        sled_aabb is not None and sled_aabb[0][2] <= 0.020,
        details=f"sled bottom z={sled_aabb[0][2] if sled_aabb else None}",
    )

    plate_aabb = ctx.part_element_world_aabb(sled_base, elem="swivel_plate")
    ctx.check(
        "swivel plate at the top of the sled supports the seat",
        plate_aabb is not None and 0.35 <= plate_aabb[1][2] <= 0.45,
        details=f"plate aabb={plate_aabb}",
    )

    # Verify the sled has tubular chrome construction (two side tubes visible)
    side0_aabb = ctx.part_element_world_aabb(sled_base, elem="sled_side_0")
    side1_aabb = ctx.part_element_world_aabb(sled_base, elem="sled_side_1")
    ctx.check(
        "sled has two distinct side tube frames",
        side0_aabb is not None
        and side1_aabb is not None
        and (side0_aabb[1][2] - side0_aabb[0][2]) >= 0.30
        and (side1_aabb[1][2] - side1_aabb[0][2]) >= 0.30,
        details=f"side0={side0_aabb}, side1={side1_aabb}",
    )

    # ------------------------------------------------------------- swivel
    with ctx.pose({swivel_joint: math.pi / 2.0}):
        loop_aabb = ctx.part_element_world_aabb(seat, elem="right_armrest_loop")
        cx = None if loop_aabb is None else 0.5 * (loop_aabb[0][0] + loop_aabb[1][0])
        ctx.check(
            "seat assembly swivels about the vertical Z axis on the sled",
            cx is not None and cx > 0.15,
            details=f"right loop center x after 90deg swivel: {cx}",
        )
        head_swivel = ctx.part_element_world_aabb(backrest, elem="headrest_pillow")
        hy = None if head_swivel is None else 0.5 * (head_swivel[0][1] + head_swivel[1][1])
        ctx.check(
            "backrest swivels together with the seat",
            hy is not None and hy < -0.20,
            details=f"headrest center y after 90deg swivel: {hy}",
        )

    # ------------------------------------------------------------- recline
    head_rest_cx = 0.5 * (head_aabb[0][0] + head_aabb[1][0]) if head_aabb else None
    with ctx.pose({recline_joint: -RECLINE_RANGE}):
        head_back = ctx.part_element_world_aabb(backrest, elem="headrest_pillow")
        hbx = None if head_back is None else 0.5 * (head_back[0][0] + head_back[1][0])
        ctx.check(
            "backrest reclines backward about the horizontal mechanism axis",
            head_rest_cx is not None and hbx is not None and hbx < head_rest_cx - 0.15,
            details=f"headrest center x rest={head_rest_cx}, reclined={hbx}",
        )
        ctx.expect_overlap(
            backrest,
            seat,
            axes="xz",
            elem_a="pivot_barrel",
            elem_b="mech_rear_bracket",
            min_overlap=0.002,
            name="pivot barrel stays captured in the bracket while reclined",
        )
    ctx.expect_overlap(
        backrest,
        seat,
        axes="xz",
        elem_a="pivot_barrel",
        elem_b="mech_rear_bracket",
        min_overlap=0.002,
        name="pivot barrel is captured in the mechanism rear bracket",
    )

    # ---------------------------------------------------------- tilt lever
    paddle_rest = ctx.part_element_world_aabb(lever, elem="lever_paddle")
    with ctx.pose({lever_joint: 0.25}):
        paddle_up = ctx.part_element_world_aabb(lever, elem="lever_paddle")
        ctx.check(
            "mechanism paddle lever flips upward when actuated",
            paddle_rest is not None
            and paddle_up is not None
            and paddle_up[1][2] > paddle_rest[1][2] + 0.010,
            details=f"paddle top rest={paddle_rest}, up={paddle_up}",
        )

    return ctx.report()


object_model = build_object_model()
