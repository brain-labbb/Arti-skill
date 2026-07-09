from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    CylinderGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ----------------------------------------------------------------------------
# Variant 29: Two-seat playground seesaw with central A-frame support.
#
# Layout (world frame, Z up, base centered on the origin):
# - Sky-blue A-frame base: two angled tube legs meeting at an apex (~0.70 m),
#   joined by a cross brace, with visible axle bracket plates at the top.
# - Single yellow rocking beam (~2.6 m), a triangulated tube truss.
# - Molded seats with raised lips at each end of the beam.
# - Rounded handle grips (capsule-shaped) at each end, just inboard of seats.
# - Articulation: beam connects to the A-frame apex via a revolute joint,
#   horizontal axis perpendicular to beam length, +/- 18 degrees.
# ----------------------------------------------------------------------------

TUBE_R = 0.020        # ~40 mm diameter main tubing
BRACE_R = 0.016       # diagonal brace tubing
SUPPORT_R = 0.018     # seat support tubing
HANDLE_R = 0.014      # handle grip tubing
GRIP_R = 0.018        # rounded grip capsule radius

AFRAME_HEIGHT = 0.70  # apex height of A-frame
AFRAME_HALF_SPREAD = 0.35  # ground half-spread of A-frame legs
AFRAME_DEPTH = 0.30   # front-to-back depth of A-frame legs at ground
CROSS_BRACE_Z = 0.32  # height of cross brace

BEAM_LEN = 2.60
MAIN_Z = 0.08         # main top tube height above the pivot axis
SLEEVE_R = 0.032
SLEEVE_LEN = 0.13
SEAT_X = 1.15         # seat center along beam from pivot
SEAT_Z = 0.038        # seat center height above pivot axis
HANDLE_X = 0.82       # handle grip position along beam from pivot

TILT = math.radians(18.0)  # rocking range

# Materials
SKY_BLUE = Material("sky_blue_paint", rgba=(0.33, 0.62, 0.84, 1.0))
WORN_YELLOW = Material("worn_yellow_paint", rgba=(0.87, 0.74, 0.12, 1.0))
RUST_BROWN = Material("rust_brown_steel", rgba=(0.42, 0.21, 0.13, 1.0))
DARK_GRAY = Material("dark_gray_metal", rgba=(0.25, 0.25, 0.27, 1.0))
SEAT_GREEN = Material("molded_green_plastic", rgba=(0.18, 0.45, 0.22, 1.0))
GRIP_BLACK = Material("rubber_grip", rgba=(0.12, 0.12, 0.12, 1.0))


def _tube_between(
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    radius: float,
    *,
    radial_segments: int = 16,
) -> MeshGeometry:
    """Straight capped tube between two 3D points."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    geom = CylinderGeometry(radius, length, radial_segments=radial_segments)
    ux, uy, uz = dx / length, dy / length, dz / length
    ax, ay, az = -uy, ux, 0.0
    s = math.sqrt(ax * ax + ay * ay + az * az)
    if s > 1e-9:
        geom.rotate((ax / s, ay / s, az / s), math.atan2(s, uz))
    elif uz < 0.0:
        geom.rotate_x(math.pi)
    geom.translate(
        (p0[0] + p1[0]) / 2.0,
        (p0[1] + p1[1]) / 2.0,
        (p0[2] + p1[2]) / 2.0,
    )
    return geom


def _build_aframe_base(model: ArticulatedObject):
    """Build the A-frame base part with two angled legs, cross brace, and axle brackets."""
    base = model.part("aframe_base")

    # Legs converge to a junction below the axle so they don't overlap the beam sleeve.
    # The bracket plates and apex tube bridge from the leg junction to the axle.
    LEG_TOP_Z = 0.62  # leg convergence point, well below sleeve bottom (~0.668)
    junction_front = (0.0, 0.035, LEG_TOP_Z)
    junction_back = (0.0, -0.035, LEG_TOP_Z)

    # Front-left leg
    base.visual(
        mesh_from_geometry(
            _tube_between((-0.08, AFRAME_DEPTH, 0.01), junction_front, TUBE_R),
            "leg_front_left",
        ),
        material=SKY_BLUE,
        name="leg_front_left",
    )
    # Front-right leg
    base.visual(
        mesh_from_geometry(
            _tube_between((0.08, AFRAME_DEPTH, 0.01), junction_front, TUBE_R),
            "leg_front_right",
        ),
        material=SKY_BLUE,
        name="leg_front_right",
    )
    # Back-left leg
    base.visual(
        mesh_from_geometry(
            _tube_between((-0.08, -AFRAME_DEPTH, 0.01), junction_back, TUBE_R),
            "leg_back_left",
        ),
        material=SKY_BLUE,
        name="leg_back_left",
    )
    # Back-right leg
    base.visual(
        mesh_from_geometry(
            _tube_between((0.08, -AFRAME_DEPTH, 0.01), junction_back, TUBE_R),
            "leg_back_right",
        ),
        material=SKY_BLUE,
        name="leg_back_right",
    )

    # Cross braces at mid-height (between legs at CROSS_BRACE_Z)
    t = CROSS_BRACE_Z / LEG_TOP_Z
    fl_brace = (-0.08 * (1 - t), AFRAME_DEPTH * (1 - t), CROSS_BRACE_Z)
    fr_brace = (0.08 * (1 - t), AFRAME_DEPTH * (1 - t), CROSS_BRACE_Z)
    bl_brace = (-0.08 * (1 - t), -AFRAME_DEPTH * (1 - t), CROSS_BRACE_Z)
    br_brace = (0.08 * (1 - t), -AFRAME_DEPTH * (1 - t), CROSS_BRACE_Z)

    base.visual(
        mesh_from_geometry(_tube_between(fl_brace, fr_brace, SUPPORT_R), "cross_brace_front"),
        material=SKY_BLUE,
        name="cross_brace_front",
    )
    base.visual(
        mesh_from_geometry(_tube_between(bl_brace, br_brace, SUPPORT_R), "cross_brace_back"),
        material=SKY_BLUE,
        name="cross_brace_back",
    )
    base.visual(
        mesh_from_geometry(_tube_between(fl_brace, bl_brace, SUPPORT_R), "cross_brace_left"),
        material=SKY_BLUE,
        name="cross_brace_left",
    )
    base.visual(
        mesh_from_geometry(_tube_between(fr_brace, br_brace, SUPPORT_R), "cross_brace_right"),
        material=SKY_BLUE,
        name="cross_brace_right",
    )

    # Vertical gusset tubes from leg junctions up toward the axle (stop below sleeve)
    GUSSET_TOP_Z = 0.64  # stays below sleeve bottom (~0.668)
    for junction, name_suffix in ((junction_front, "front"), (junction_back, "back")):
        gusset = _tube_between(
            junction,
            (junction[0], junction[1], GUSSET_TOP_Z),
            SUPPORT_R,
        )
        base.visual(
            mesh_from_geometry(gusset, f"gusset_{name_suffix}"),
            material=SKY_BLUE,
            name=f"gusset_{name_suffix}",
        )

    # Axle bracket plates at apex - two flat plates on each side of the axle
    # Each side has two narrow plates flanking the cylinder (at x=±0.04, outside the 0.032 radius)
    bracket_thickness = 0.008
    bracket_width = 0.04  # narrower to avoid cylinder
    bracket_height = 0.10  # from z=0.64 to z=0.74
    bracket_idx = 0
    for sy, side_name in ((1.0, "left"), (-1.0, "right")):
        for sx, x_name in ((1.0, "front"), (-1.0, "back")):
            bracket = cq.Workplane("XY").box(bracket_width, bracket_thickness, bracket_height)
            bracket_mesh = mesh_from_cadquery(bracket, f"axle_bracket_{side_name}_{x_name}")
            # Position bracket at x=sx*0.06 (well outside cylinder radius 0.032)
            # and y=sy*0.055 (between gusset and axle region)
            base.visual(
                bracket_mesh,
                origin=Origin(xyz=(sx * 0.06, sy * 0.055, 0.69)),
                material=DARK_GRAY,
                name=f"axle_bracket_{side_name}_{x_name}",
            )
            bracket_idx += 1

    # Top axle tube: the structural axle that the beam sleeve rotates around.
    # This is longer than the sleeve so its ends protrude past the sleeve.
    axle_len = 0.16  # longer than sleeve (0.13) so axle ends protrude
    axle = CylinderGeometry(TUBE_R, axle_len, radial_segments=16).rotate_x(math.pi / 2.0)
    base.visual(
        mesh_from_geometry(axle, "apex_axle"),
        origin=Origin(xyz=(0.0, 0.0, AFRAME_HEIGHT)),
        material=DARK_GRAY,
        name="apex_axle",
    )

    return base


def _build_molded_seat() -> cq.Workplane:
    """Build a molded seat with raised lips using CadQuery.

    A shallow rectangular dish: flat base plate with four raised lip walls.
    The seat is centered at origin, sitting on z=0 plane.
    """
    seat_w = 0.26    # width (along beam axis, X)
    seat_d = 0.28    # depth (perpendicular, Y)
    base_h = 0.010   # base plate thickness
    lip_h = 0.030    # lip height above base
    lip_t = 0.012    # lip wall thickness

    # Start with a solid base plate
    seat = cq.Workplane("XY").box(seat_w, seat_d, base_h).translate((0, 0, base_h / 2))

    # Add raised lips as four walls on the edges
    # Front lip (along X, at +Y edge)
    front_lip = (
        cq.Workplane("XY")
        .box(seat_w, lip_t, lip_h)
        .translate((0, seat_d / 2 - lip_t / 2, base_h + lip_h / 2))
    )
    # Back lip
    back_lip = (
        cq.Workplane("XY")
        .box(seat_w, lip_t, lip_h)
        .translate((0, -(seat_d / 2 - lip_t / 2), base_h + lip_h / 2))
    )
    # Left lip (along Y, at +X edge)
    left_lip = (
        cq.Workplane("XY")
        .box(lip_t, seat_d - 2 * lip_t, lip_h)
        .translate((seat_w / 2 - lip_t / 2, 0, base_h + lip_h / 2))
    )
    # Right lip
    right_lip = (
        cq.Workplane("XY")
        .box(lip_t, seat_d - 2 * lip_t, lip_h)
        .translate((-(seat_w / 2 - lip_t / 2), 0, base_h + lip_h / 2))
    )

    seat = seat.union(front_lip).union(back_lip).union(left_lip).union(right_lip)

    # Fillet the top outer edges of the lips for a molded look
    seat = seat.edges(">Z").fillet(0.004)

    return seat


def _build_beam(model: ArticulatedObject):
    """Build the single rocking beam with truss, molded seats, and rounded grips."""
    beam = model.part("beam")

    # --- Main truss tube (yellow) ---
    truss = (
        CylinderGeometry(TUBE_R, BEAM_LEN, radial_segments=18)
        .rotate_y(math.pi / 2.0)
        .translate(0.0, 0.0, MAIN_Z)
    )
    # Diagonal braces from center sleeve to main tube at each side
    for sx in (1.0, -1.0):
        truss.merge(
            _tube_between(
                (sx * 0.04, 0.0, 0.005),
                (sx * 0.55, 0.0, MAIN_Z),
                BRACE_R,
            )
        )
        # Short bent seat support dropping from main tube to under the seat
        truss.merge(
            tube_from_spline_points(
                [
                    (sx * 1.00, 0.0, MAIN_Z),
                    (sx * 1.08, 0.0, 0.055),
                    (sx * 1.13, 0.0, 0.025),
                    (sx * 1.18, 0.0, 0.012),
                ],
                radius=SUPPORT_R,
                samples_per_segment=10,
                radial_segments=14,
                cap_ends=True,
            )
        )

    beam.visual(
        mesh_from_geometry(truss, "beam_truss"),
        material=WORN_YELLOW,
        name="truss_tube",
    )

    # --- Axle sleeve (wraps the apex axle) ---
    sleeve = (
        CylinderGeometry(SLEEVE_R, SLEEVE_LEN, radial_segments=20)
        .rotate_x(math.pi / 2.0)
    )
    # Weld post tying sleeve to main top tube
    weld_post = CylinderGeometry(0.014, MAIN_Z - 0.024, radial_segments=14).translate(
        0.0, 0.0, (MAIN_Z + 0.024) / 2.0
    )
    sleeve.merge(weld_post)

    beam.visual(
        mesh_from_geometry(sleeve, "beam_sleeve"),
        material=WORN_YELLOW,
        name="axle_sleeve",
    )

    # --- Molded seats with raised lips ---
    seat_shape = _build_molded_seat()
    seat_mesh_pos = mesh_from_cadquery(seat_shape, "seat_positive")
    seat_mesh_neg = mesh_from_cadquery(seat_shape, "seat_negative")

    # Rotate seat so that X of seat aligns with beam axis (already correct)
    # Seat sits on top of the support tube end
    beam.visual(
        seat_mesh_pos,
        origin=Origin(xyz=(SEAT_X, 0.0, SEAT_Z)),
        material=SEAT_GREEN,
        name="seat_0",
    )
    beam.visual(
        seat_mesh_neg,
        origin=Origin(xyz=(-SEAT_X, 0.0, SEAT_Z)),
        material=SEAT_GREEN,
        name="seat_1",
    )

    # --- Rounded handle grips (capsule-shaped) at each end ---
    # Each grip: an upright post (yellow tube) + a horizontal capsule grip (black rubber)
    for sx, end_idx in ((1.0, 0), (-1.0, 1)):
        # Upright post from main tube
        post = CylinderGeometry(HANDLE_R, 0.26, radial_segments=14).translate(
            sx * HANDLE_X, 0.0, MAIN_Z + 0.12
        )
        # Short connecting bar at top
        bar = (
            CylinderGeometry(HANDLE_R, 0.06, radial_segments=14)
            .rotate_x(math.pi / 2.0)
            .translate(sx * HANDLE_X, 0.0, MAIN_Z + 0.25)
        )
        post.merge(bar)

        beam.visual(
            mesh_from_geometry(post, f"handle_post_{end_idx}"),
            material=WORN_YELLOW,
            name=f"handle_post_{end_idx}",
        )

        # Rounded capsule grip (horizontal, along Y axis)
        grip = CapsuleGeometry(GRIP_R, 0.22, radial_segments=16, height_segments=6)
        grip.rotate_x(math.pi / 2.0)

        beam.visual(
            mesh_from_geometry(grip, f"handle_grip_{end_idx}"),
            origin=Origin(xyz=(sx * HANDLE_X, 0.0, MAIN_Z + 0.25)),
            material=GRIP_BLACK,
            name=f"handle_grip_{end_idx}",
        )

    return beam


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="aframe_playground_seesaw")

    # --- A-frame base ---
    base = _build_aframe_base(model)

    # --- Single rocking beam ---
    beam = _build_beam(model)

    # --- Articulation: beam pivots on the A-frame apex ---
    limits = MotionLimits(effort=150.0, velocity=2.5, lower=-TILT, upper=TILT)
    model.articulation(
        "beam_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, AFRAME_HEIGHT)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("aframe_base")
    beam = object_model.get_part("beam")
    pivot = object_model.get_articulation("beam_pivot")

    # --- A-frame base geometry ---
    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "A-frame base is approximately 0.7 m tall",
        base_aabb is not None and 0.65 <= base_aabb[1][2] <= 0.80,
        details=f"base aabb={base_aabb}",
    )
    ctx.check(
        "A-frame feet rest on the ground plane",
        base_aabb is not None and -0.01 <= base_aabb[0][2] <= 0.02,
        details=f"base aabb={base_aabb}",
    )

    # Axle brackets exist at the apex (now 4 brackets: left-front, left-back, right-front, right-back)
    bracket_lf = ctx.part_element_world_aabb(base, elem="axle_bracket_left_front")
    bracket_rb = ctx.part_element_world_aabb(base, elem="axle_bracket_right_back")
    ctx.check(
        "visible axle brackets exist at the A-frame apex",
        bracket_lf is not None and bracket_rb is not None,
        details=f"left_front={bracket_lf}, right_back={bracket_rb}",
    )
    if bracket_lf is not None and bracket_rb is not None:
        bracket_center_z = (
            (bracket_lf[0][2] + bracket_lf[1][2]) / 2.0
            + (bracket_rb[0][2] + bracket_rb[1][2]) / 2.0
        ) / 2.0
        ctx.check(
            "axle brackets are positioned at the A-frame apex",
            abs(bracket_center_z - AFRAME_HEIGHT) < 0.05,
            details=f"bracket_center_z={bracket_center_z:.3f}, apex={AFRAME_HEIGHT}",
        )

    # --- Molded seats with raised lips ---
    for end in (0, 1):
        seat_aabb = ctx.part_element_world_aabb(beam, elem=f"seat_{end}")
        ctx.check(
            f"molded seat {end} exists on the beam",
            seat_aabb is not None,
            details=f"seat_{end} aabb={seat_aabb}",
        )
        if seat_aabb is not None:
            seat_height = seat_aabb[1][2] - seat_aabb[0][2]
            ctx.check(
                f"seat {end} has raised lips (total height > base plate thickness)",
                seat_height > 0.025,
                details=f"seat height={seat_height:.4f} m (should be >0.025 for base+lip)",
            )

    # --- Rounded handle grips ---
    for end in (0, 1):
        grip_aabb = ctx.part_element_world_aabb(beam, elem=f"handle_grip_{end}")
        ctx.check(
            f"rounded handle grip {end} exists",
            grip_aabb is not None,
            details=f"handle_grip_{end} aabb={grip_aabb}",
        )
        if grip_aabb is not None:
            grip_dy = grip_aabb[1][1] - grip_aabb[0][1]
            ctx.check(
                f"handle grip {end} is capsule-shaped (elongated along Y)",
                grip_dy > 0.15,
                details=f"grip Y extent={grip_dy:.4f} m",
            )

    # --- Revolute joint on central pivot ---
    ctx.check(
        "beam_pivot is a revolute joint",
        pivot.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={pivot.articulation_type}",
    )
    lim = pivot.motion_limits
    ctx.check(
        "beam pivot rocks +/- 18 degrees",
        lim is not None
        and abs(lim.lower + TILT) < 1e-6
        and abs(lim.upper - TILT) < 1e-6,
        details=f"limits=({lim.lower if lim else None}, {lim.upper if lim else None})",
    )

    # --- Captured axle fit: sleeve wraps the apex axle ---
    ctx.allow_overlap(
        beam,
        base,
        elem_a="axle_sleeve",
        elem_b="apex_axle",
        reason="Beam axle sleeve intentionally wraps the A-frame apex axle as the pivot bearing.",
    )
    # Axle is nested inside sleeve: prove centering and retained insertion
    ctx.expect_within(
        base,
        beam,
        axes="xz",
        inner_elem="apex_axle",
        outer_elem="axle_sleeve",
        margin=0.005,
        name="apex axle stays centered inside the sleeve bore",
    )
    ctx.expect_overlap(
        base,
        beam,
        axes="y",
        elem_a="apex_axle",
        elem_b="axle_sleeve",
        min_overlap=0.10,
        name="apex axle is retained inside the sleeve along the pivot axis",
    )

    # --- Seesaw motion: one end rises, other drops ---
    rest_seat0 = ctx.part_element_world_aabb(beam, elem="seat_0")
    rest_seat1 = ctx.part_element_world_aabb(beam, elem="seat_1")

    with ctx.pose({pivot: TILT}):
        tilt_seat0 = ctx.part_element_world_aabb(beam, elem="seat_0")
        tilt_seat1 = ctx.part_element_world_aabb(beam, elem="seat_1")
        beam_aabb = ctx.part_world_aabb(beam)

        ctx.check(
            "beam seesaws: seat_0 drops when tilted positive",
            rest_seat0 is not None
            and tilt_seat0 is not None
            and tilt_seat0[0][2] < rest_seat0[0][2] - 0.25,
            details=f"seat0 rest={rest_seat0}, tilted={tilt_seat0}",
        )
        ctx.check(
            "beam seesaws: seat_1 rises when tilted positive",
            rest_seat1 is not None
            and tilt_seat1 is not None
            and tilt_seat1[0][2] > rest_seat1[0][2] + 0.25,
            details=f"seat1 rest={rest_seat1}, tilted={tilt_seat1}",
        )
        ctx.check(
            "fully tilted beam stays clear of the ground",
            beam_aabb is not None and beam_aabb[0][2] > 0.02,
            details=f"beam aabb={beam_aabb}",
        )
        ctx.expect_within(
            base,
            beam,
            axes="xz",
            inner_elem="apex_axle",
            outer_elem="axle_sleeve",
            margin=0.005,
            name="tilted beam: apex axle stays centered in sleeve",
        )

    return ctx.report()


object_model = build_object_model()
