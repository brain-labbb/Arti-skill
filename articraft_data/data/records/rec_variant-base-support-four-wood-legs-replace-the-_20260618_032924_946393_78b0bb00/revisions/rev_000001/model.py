from __future__ import annotations

# Tall ergonomic office chair (IKEA Markus style), four-leg wood-base variant.
#
# Variant fork: replaces the five-spoke caster base, gas-lift column, and
# swivel joint with four splayed solid-wood legs fixed directly under the seat.
#
# Kinematic tree:
#   seat (root: seat pan, cushion, armrest loops, mechanism housing, 4 wood legs)
#     -> backrest      REVOLUTE about Y (recline, 0..-0.25 rad)
#     -> tilt_lever    REVOLUTE paddle lever on the mechanism side

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
# The seat part is root, so its origin is world (0,0,0).  All z values in
# seat visuals are world-space heights.
SH = 0.440  # seat reference height – the height that was the seat-frame
#             origin in the parent model.  Every seat-visual z from the parent
#             gets +SH to convert from seat-local to world.

# Leg geometry – four splayed legs from the seat pan underside to the floor.
# Leg tops extend 2 mm into the seat pan for hidden connectivity.
LEG_ATTACH_Z = SH + 0.043 - 0.035 + 0.002  # world z of leg top (inside seat pan)
LEG_FLOOR_Z = 0.004             # world z of leg bottom (tiny clearance for glide)
LEG_SPLAY = 0.072               # horizontal offset per leg direction
LEG_RADIUS = 0.019              # leg tube radius (~38 mm diameter)
LEG_SEAT_XY = 0.160             # leg attach distance from centre in X / Y

LEG_CORNERS = (
    ("front_left",   1.0,  1.0),
    ("front_right",  1.0, -1.0),
    ("rear_left",   -1.0,  1.0),
    ("rear_right",  -1.0, -1.0),
)

# Backrest recline pivot (world coords: parent-model local + SH)
RECLINE_PIVOT = (-0.21, 0.0, SH - 0.02)
RECLINE_RANGE = 0.25


# ----------------------------------------------------------- geometry helpers
def _leg_mesh(label: str, sx: float, sy: float):
    """One splayed wooden leg from the seat underside down to the floor."""
    tx = sx * LEG_SEAT_XY
    ty = sy * LEG_SEAT_XY
    bx = tx + sx * LEG_SPLAY
    by = ty + sy * LEG_SPLAY
    geom = tube_from_spline_points(
        [
            (tx, ty, LEG_ATTACH_Z),
            (0.5 * (tx + bx), 0.5 * (ty + by), 0.5 * (LEG_ATTACH_Z + LEG_FLOOR_Z)),
            (bx, by, LEG_FLOOR_Z),
        ],
        radius=LEG_RADIUS,
        samples_per_segment=12,
        radial_segments=14,
        cap_ends=True,
    )
    return mesh_from_geometry(geom, f"leg_{label}")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="markus_wood_leg_chair")

    # --------------------------------------------------------- materials
    plastic_black = model.material("plastic_black", rgba=(0.10, 0.10, 0.11, 1.0))
    mesh_black = model.material("mesh_black", rgba=(0.05, 0.05, 0.055, 1.0))
    fabric_black = model.material("fabric_black", rgba=(0.085, 0.085, 0.09, 1.0))
    wood = model.material("wood_oak", rgba=(0.54, 0.34, 0.17, 1.0))
    wood_dark = model.material("wood_dark", rgba=(0.36, 0.22, 0.10, 1.0))
    glide_black = model.material("glide_black", rgba=(0.06, 0.06, 0.065, 1.0))

    # ===================================================================== seat
    seat = model.part("seat")

    # -- Tilt-mechanism housing (still needed for the backrest pivot mount)
    seat.visual(
        Box((0.24, 0.18, 0.060)),
        origin=Origin(xyz=(0.01, 0.0, SH - 0.020)),
        material=plastic_black,
        name="mech_housing",
    )
    seat.visual(
        Box((0.090, 0.12, 0.050)),
        origin=Origin(xyz=(-0.150, 0.0, SH - 0.020)),
        material=plastic_black,
        name="mech_rear_bracket",
    )

    # -- Seat pan and cushion  (parent local z + SH)
    pan_geom = ExtrudeGeometry(rounded_rect_profile(0.50, 0.46, 0.10), 0.07, center=True)
    pan_geom.translate(0.01, 0.0, SH + 0.043)
    seat.visual(
        mesh_from_geometry(pan_geom, "seat_pan"),
        material=plastic_black,
        name="seat_pan",
    )
    cushion_geom = ExtrudeGeometry(rounded_rect_profile(0.44, 0.42, 0.09), 0.03, center=True)
    cushion_geom.translate(0.02, 0.0, SH + 0.088)
    seat.visual(
        mesh_from_geometry(cushion_geom, "seat_cushion"),
        material=fabric_black,
        name="seat_cushion",
    )

    # -- Closed-loop armrests (flattened plastic rings on angled stems)
    for label, sign in (("left", 1.0), ("right", -1.0)):
        stem_geom = tube_from_spline_points(
            [
                (0.05, sign * 0.205, SH + 0.015),
                (0.03, sign * 0.245, SH + 0.070),
                (0.01, sign * 0.252, SH + 0.130),
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
        ring_geom.translate(0.01, sign * 0.255, SH + 0.215)
        seat.visual(
            mesh_from_geometry(ring_geom, f"armrest_loop_{label}"),
            material=plastic_black,
            name=f"{label}_armrest_loop",
        )

    # -- Four splayed solid-wood legs (inline visuals on the root seat part)
    for label, sx, sy in LEG_CORNERS:
        # Leg tube
        seat.visual(
            _leg_mesh(label, sx, sy),
            material=wood,
            name=f"leg_{label}",
        )
        # Tenon mounting block at seat underside (overlaps leg top and seat pan)
        tx = sx * LEG_SEAT_XY
        ty = sy * LEG_SEAT_XY
        seat.visual(
            Box((0.052, 0.052, 0.028)),
            origin=Origin(xyz=(tx, ty, LEG_ATTACH_Z)),
            material=wood_dark,
            name=f"mount_block_{label}",
        )
        # Floor glide disc at leg bottom
        bx = tx + sx * LEG_SPLAY
        by = ty + sy * LEG_SPLAY
        seat.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(xyz=(bx, by, LEG_FLOOR_Z - 0.001)),
            material=glide_black,
            name=f"foot_glide_{label}",
        )

    # ============================================================= tilt lever
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
        origin=Origin(xyz=(0.06, -0.10, SH - 0.03)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=4.0, lower=0.0, upper=0.25),
    )

    # =============================================================== backrest
    backrest = model.part("backrest")

    # Pivot barrel captured in the mechanism rear bracket
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

    # Spine and bottom rail
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

    # Curved side rails following the lumbar S-curve
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

    # Top rail
    backrest.visual(
        Box((0.045, 0.400, 0.050)),
        origin=Origin(xyz=(-0.085, 0.0, 0.845), rpy=(0.0, -0.21, 0.0)),
        material=plastic_black,
        name="top_rail",
    )

    # Dark mesh center panel in three curved segments
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

    # Padded pillow headrest capping the top
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

    seat = object_model.get_part("seat")
    backrest = object_model.get_part("backrest")
    lever = object_model.get_part("tilt_lever")
    recline_joint = object_model.get_articulation("seat_to_backrest")
    lever_joint = object_model.get_articulation("seat_to_tilt_lever")

    # ---------------------------------------------- intentional overlaps
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

    # ---------------------------------------------- seat height and pan size
    cushion_aabb = ctx.part_element_world_aabb(seat, elem="seat_cushion")
    ctx.check(
        "seat cushion top sits at chair height (0.45-0.57 m)",
        cushion_aabb is not None and 0.45 <= cushion_aabb[1][2] <= 0.57,
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

    # ---------------------------------------------- backrest dimensions
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

    # ---------------------------------------------- armrests
    for label, low, high in (("left", 0.22, 0.30), ("right", -0.30, -0.22)):
        loop_aabb = ctx.part_element_world_aabb(seat, elem=f"{label}_armrest_loop")
        cy = None if loop_aabb is None else 0.5 * (loop_aabb[0][1] + loop_aabb[1][1])
        ctx.check(
            f"{label} closed-loop armrest sits at the seat side",
            loop_aabb is not None
            and low <= cy <= high
            and 0.58 <= loop_aabb[1][2] <= 0.76,
            details=f"{label} loop aabb={loop_aabb}",
        )

    # ---------------------------------------------- four wood legs on the floor
    for label, sx, sy in LEG_CORNERS:
        leg_aabb = ctx.part_element_world_aabb(seat, elem=f"leg_{label}")
        ctx.check(
            f"leg_{label} reaches down to the floor",
            leg_aabb is not None and leg_aabb[0][2] <= 0.012,
            details=f"leg_{label} aabb={leg_aabb}",
        )
        glide_aabb = ctx.part_element_world_aabb(seat, elem=f"foot_glide_{label}")
        ctx.check(
            f"foot_glide_{label} rests near the floor",
            glide_aabb is not None and glide_aabb[0][2] <= 0.008,
            details=f"glide_{label} aabb={glide_aabb}",
        )

    # Legs splay outward: front-left leg bottom should be further +X,+Y than top
    fl_leg_aabb = ctx.part_element_world_aabb(seat, elem="leg_front_left")
    ctx.check(
        "front-left leg splays outward (wider footprint than seat corner)",
        fl_leg_aabb is not None
        and fl_leg_aabb[1][0] > LEG_SEAT_XY + 0.03
        and fl_leg_aabb[1][1] > LEG_SEAT_XY + 0.03,
        details=f"front_left leg aabb={fl_leg_aabb}",
    )

    # ---------------------------------------------- backrest recline
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

    # ---------------------------------------------- tilt lever
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
