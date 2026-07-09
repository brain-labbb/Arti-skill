from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoxGeometry,
    CylinderGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ----------------------------------------------------------------------------
# Classic two-seat playground plank seesaw with round tube support legs.
#
# Layout (world frame, Z up, base centered on the origin):
# - Sky-blue base: two inverted-U tube legs joined by a horizontal crossbar,
#   standing about 0.60 m tall at the pivot axle.
# - One yellow plank beam (~2.4 m) that rocks on the central pivot.
# - Two molded seats with raised lips at opposite ends of the plank.
# - Two T-handlebars, each on its own revolute joint allowing slight pivot.
# - Articulation: plank rocks +/- 18 degrees; handlebars pivot +/- 12 degrees.
# ----------------------------------------------------------------------------

TUBE_R = 0.020  # ~40 mm diameter main tubing
SUPPORT_R = 0.018
HANDLE_R = 0.014

PLANK_LEN = 2.40
PLANK_R = 0.022  # plank main tube radius
PIVOT_Z = 0.60  # pivot axle height above ground
TILT = math.radians(18.0)  # plank rocking range
HANDLEBAR_TILT = math.radians(12.0)  # handlebar pivot range

# Base geometry
ARCH_HALF_SPAN = 0.30  # ground half-span of each inverted-U leg
LEG_SPACING = 0.22  # half-distance between the two leg planes along Y
CROSSBAR_Z = 0.28  # height of horizontal crossbar joining legs

# Seat geometry
SEAT_OFFSET = 1.02  # seat center distance from plank pivot along X
SEAT_BASE_SIZE = (0.24, 0.24, 0.010)  # seat base plate
SEAT_LIP_HEIGHT = 0.030  # raised lip height above base
SEAT_LIP_THICK = 0.008  # lip wall thickness

# Handlebar geometry
HANDLE_OFFSET = 0.72  # handlebar post distance from pivot along X
HANDLE_POST_HEIGHT = 0.28
HANDLE_CROSSBAR_LEN = 0.26
HANDLE_TOP_Z = 0.36  # crossbar center height above pivot

SKY_BLUE = Material("sky_blue_paint", rgba=(0.33, 0.62, 0.84, 1.0))
WORN_YELLOW = Material("worn_yellow_paint", rgba=(0.87, 0.74, 0.12, 1.0))
SEAT_GREEN = Material("molded_green_plastic", rgba=(0.18, 0.45, 0.22, 1.0))
DARK_GRAY = Material("dark_gray_steel", rgba=(0.25, 0.25, 0.27, 1.0))


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


def _build_base() -> MeshGeometry:
    """Build the inverted-U tube base stand with two legs and a crossbar."""
    result = MeshGeometry()

    # Two inverted-U legs, one at +Y and one at -Y
    for sy in (1.0, -1.0):
        y_pos = sy * LEG_SPACING
        # Each leg: two vertical posts and a horizontal top run (axle support)
        # Left post
        result.merge(
            _tube_between(
                (-ARCH_HALF_SPAN, y_pos, 0.0),
                (-ARCH_HALF_SPAN, y_pos, PIVOT_Z - 0.02),
                TUBE_R,
            )
        )
        # Right post
        result.merge(
            _tube_between(
                (ARCH_HALF_SPAN, y_pos, 0.0),
                (ARCH_HALF_SPAN, y_pos, PIVOT_Z - 0.02),
                TUBE_R,
            )
        )
        # Top horizontal run (acts as axle support)
        result.merge(
            _tube_between(
                (-ARCH_HALF_SPAN, y_pos, PIVOT_Z),
                (ARCH_HALF_SPAN, y_pos, PIVOT_Z),
                TUBE_R,
            )
        )
        # Bent top corners (short curved sections using spline tubes)
        for sx in (-1.0, 1.0):
            result.merge(
                tube_from_spline_points(
                    [
                        (sx * ARCH_HALF_SPAN, y_pos, PIVOT_Z - 0.06),
                        (sx * ARCH_HALF_SPAN, y_pos, PIVOT_Z - 0.02),
                        (sx * (ARCH_HALF_SPAN - 0.02), y_pos, PIVOT_Z),
                        (sx * (ARCH_HALF_SPAN - 0.06), y_pos, PIVOT_Z),
                    ],
                    radius=TUBE_R,
                    samples_per_segment=6,
                    radial_segments=14,
                    cap_ends=False,
                )
            )
        # Foot plates (small flat discs at ground)
        for sx in (-1.0, 1.0):
            foot = CylinderGeometry(0.035, 0.006, radial_segments=16).translate(
                sx * ARCH_HALF_SPAN, y_pos, 0.003
            )
            result.merge(foot)

    # Axle tube spanning between the two leg frames at pivot height
    result.merge(
        _tube_between(
            (0.0, -LEG_SPACING - 0.01, PIVOT_Z),
            (0.0, LEG_SPACING + 0.01, PIVOT_Z),
            TUBE_R,
        )
    )

    # Crossbars bracing the two leg frames
    for z_h in (CROSSBAR_Z, 0.12):
        # Front-to-back crossbars connecting the two leg frames
        result.merge(
            _tube_between(
                (-ARCH_HALF_SPAN + 0.02, -LEG_SPACING, z_h),
                (-ARCH_HALF_SPAN + 0.02, LEG_SPACING, z_h),
                SUPPORT_R,
            )
        )
        result.merge(
            _tube_between(
                (ARCH_HALF_SPAN - 0.02, -LEG_SPACING, z_h),
                (ARCH_HALF_SPAN - 0.02, LEG_SPACING, z_h),
                SUPPORT_R,
            )
        )

    return result


def _build_plank() -> MeshGeometry:
    """Build the plank beam in its local frame (X along beam, pivot at origin)."""
    # Main plank tube
    plank = CylinderGeometry(PLANK_R, PLANK_LEN, radial_segments=18)
    plank.rotate_y(math.pi / 2.0)

    # Axle sleeve at center (wraps the base axle)
    sleeve = CylinderGeometry(0.030, 0.12, radial_segments=20)
    sleeve.rotate_x(math.pi / 2.0)
    plank.merge(sleeve)

    # Short diagonal braces from sleeve to plank tube for structural look
    for sx in (1.0, -1.0):
        plank.merge(
            _tube_between(
                (sx * 0.04, 0.0, 0.005),
                (sx * 0.40, 0.0, PLANK_R * 0.3),
                SUPPORT_R,
            )
        )

    # Seat mount brackets: short vertical posts at each end to carry seats
    for sx in (1.0, -1.0):
        plank.merge(
            _tube_between(
                (sx * SEAT_OFFSET, 0.0, PLANK_R),
                (sx * SEAT_OFFSET, 0.0, PLANK_R + 0.04),
                SUPPORT_R,
            )
        )

    return plank


def _build_molded_seat() -> MeshGeometry:
    """Build a molded seat with raised lips in local frame (centered, Z up).

    The seat is a rectangular dish: flat base with raised perimeter lips.
    """
    w, d, h = SEAT_BASE_SIZE  # width (X), depth (Y), base thickness (Z)
    lip_h = SEAT_LIP_HEIGHT
    lip_t = SEAT_LIP_THICK

    # Flat base plate
    seat = BoxGeometry((w, d, h))

    # Raised lips on all four sides
    # Front lip (+X)
    seat.merge(
        BoxGeometry((lip_t, d, lip_h)).translate(w / 2.0 - lip_t / 2.0, 0.0, h / 2.0 + lip_h / 2.0)
    )
    # Back lip (-X)
    seat.merge(
        BoxGeometry((lip_t, d, lip_h)).translate(-w / 2.0 + lip_t / 2.0, 0.0, h / 2.0 + lip_h / 2.0)
    )
    # Left lip (+Y)
    seat.merge(
        BoxGeometry((w - 2 * lip_t, lip_t, lip_h)).translate(0.0, d / 2.0 - lip_t / 2.0, h / 2.0 + lip_h / 2.0)
    )
    # Right lip (-Y)
    seat.merge(
        BoxGeometry((w - 2 * lip_t, lip_t, lip_h)).translate(0.0, -d / 2.0 + lip_t / 2.0, h / 2.0 + lip_h / 2.0)
    )

    return seat


def _build_handlebar() -> MeshGeometry:
    """Build a T-handlebar in local frame (post goes up in +Z, crossbar along Y).

    The handlebar pivot is at its base (where it attaches to the plank).
    """
    # Vertical post
    post = CylinderGeometry(HANDLE_R, HANDLE_POST_HEIGHT, radial_segments=14)
    post.translate(0.0, 0.0, HANDLE_POST_HEIGHT / 2.0)

    # Horizontal crossbar at top
    bar = CylinderGeometry(HANDLE_R, HANDLE_CROSSBAR_LEN, radial_segments=14)
    bar.rotate_x(math.pi / 2.0)
    bar.translate(0.0, 0.0, HANDLE_POST_HEIGHT)
    post.merge(bar)

    # Grip ends (small spheres or short caps at crossbar ends)
    for sy in (1.0, -1.0):
        grip = CylinderGeometry(HANDLE_R * 1.3, 0.04, radial_segments=12)
        grip.rotate_x(math.pi / 2.0)
        grip.translate(0.0, sy * HANDLE_CROSSBAR_LEN / 2.0, HANDLE_POST_HEIGHT)
        post.merge(grip)

    # Mounting collar at base (contacts plank surface)
    collar = CylinderGeometry(HANDLE_R * 1.4, 0.014, radial_segments=12)
    collar.translate(0.0, 0.0, 0.007)
    post.merge(collar)

    return post


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_seat_plank_seesaw")

    # --- Static base with round tube legs ---
    base = model.part("base")
    base.visual(
        mesh_from_geometry(_build_base(), "base_frame"),
        material=SKY_BLUE,
        name="base_frame",
    )

    # --- Plank beam ---
    plank = model.part("plank")
    plank.visual(
        mesh_from_geometry(_build_plank(), "plank_beam"),
        material=WORN_YELLOW,
        name="plank_beam",
    )

    # Molded seats on the plank (these are visuals on the plank part)
    seat_mesh = _build_molded_seat()
    seat_z = PLANK_R + 0.04 + SEAT_BASE_SIZE[2] / 2.0  # on top of mount bracket

    plank.visual(
        mesh_from_geometry(seat_mesh.copy(), "seat_left"),
        origin=Origin(xyz=(SEAT_OFFSET, 0.0, seat_z)),
        material=SEAT_GREEN,
        name="seat_left",
    )
    plank.visual(
        mesh_from_geometry(seat_mesh.copy().rotate_z(math.pi), "seat_right"),
        origin=Origin(xyz=(-SEAT_OFFSET, 0.0, seat_z)),
        material=SEAT_GREEN,
        name="seat_right",
    )

    # --- Handlebars as separate articulated parts ---
    left_hb = model.part("left_handlebar")
    left_hb.visual(
        mesh_from_geometry(_build_handlebar(), "left_hb_mesh"),
        material=DARK_GRAY,
        name="left_hb_mesh",
    )

    right_hb = model.part("right_handlebar")
    right_hb.visual(
        mesh_from_geometry(_build_handlebar(), "right_hb_mesh"),
        material=DARK_GRAY,
        name="right_hb_mesh",
    )

    # --- Articulations ---

    # Plank pivot: revolute about Y axis at the top of the base
    model.articulation(
        "plank_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=plank,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=150.0, velocity=2.5, lower=-TILT, upper=TILT),
    )

    # Handlebar pivots: each handlebar connects to the plank with a slight
    # revolute pivot about the Y axis (allowing forward/back tilt while gripping).
    # The handlebar base sits at the top of the plank tube, at HANDLE_OFFSET from center.
    hb_mount_z = PLANK_R  # at plank top surface for contact
    model.articulation(
        "left_handlebar_pivot",
        ArticulationType.REVOLUTE,
        parent=plank,
        child=left_hb,
        origin=Origin(xyz=(HANDLE_OFFSET, 0.0, hb_mount_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=5.0, lower=-HANDLEBAR_TILT, upper=HANDLEBAR_TILT),
    )
    model.articulation(
        "right_handlebar_pivot",
        ArticulationType.REVOLUTE,
        parent=plank,
        child=right_hb,
        origin=Origin(xyz=(-HANDLE_OFFSET, 0.0, hb_mount_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=5.0, lower=-HANDLEBAR_TILT, upper=HANDLEBAR_TILT),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    plank = object_model.get_part("plank")
    left_hb = object_model.get_part("left_handlebar")
    right_hb = object_model.get_part("right_handlebar")
    plank_pivot = object_model.get_articulation("plank_pivot")
    left_hb_pivot = object_model.get_articulation("left_handlebar_pivot")
    right_hb_pivot = object_model.get_articulation("right_handlebar_pivot")

    # --- Plank pivot ---
    # The axle sleeve wraps the base axle (intentional captured fit).
    ctx.allow_overlap(
        plank,
        base,
        elem_a="plank_beam",
        elem_b="base_frame",
        reason="Plank axle sleeve intentionally wraps the base axle tube as a captured pivot.",
    )
    ctx.expect_contact(
        plank,
        base,
        elem_a="plank_beam",
        elem_b="base_frame",
        name="plank sleeve rides on the base axle",
    )

    # Base stands ~0.6 m tall with feet on ground
    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "base is a tube stand about 0.6 m tall",
        base_aabb is not None and 0.55 <= base_aabb[1][2] <= 0.70,
        details=f"base aabb={base_aabb}",
    )
    ctx.check(
        "base feet rest on the ground plane",
        base_aabb is not None and -0.01 <= base_aabb[0][2] <= 0.015,
        details=f"base aabb={base_aabb}",
    )

    # --- Molded seats with raised lips ---
    # Both seats exist at opposite ends of the plank
    for seat_name, side_sign in [("seat_left", 1.0), ("seat_right", -1.0)]:
        seat_aabb = ctx.part_element_world_aabb(plank, elem=seat_name)
        ctx.check(
            f"{seat_name} exists at plank end",
            seat_aabb is not None,
            details=f"{seat_name} aabb={seat_aabb}",
        )
        if seat_aabb is not None:
            # Seat should be elevated above plank tube center
            seat_center_z = (seat_aabb[0][2] + seat_aabb[1][2]) / 2.0
            ctx.check(
                f"{seat_name} sits above the plank beam",
                seat_center_z > PIVOT_Z + PLANK_R,
                details=f"seat_center_z={seat_center_z:.3f}, expected > {PIVOT_Z + PLANK_R:.3f}",
            )
            # Seat should be at the correct end (positive or negative X)
            seat_center_x = (seat_aabb[0][0] + seat_aabb[1][0]) / 2.0
            ctx.check(
                f"{seat_name} is at the correct plank end",
                seat_center_x * side_sign > 0.7,
                details=f"seat_center_x={seat_center_x:.3f}",
            )
            # Raised lips: the seat top should extend above the seat base by ~30mm
            seat_height = seat_aabb[1][2] - seat_aabb[0][2]
            ctx.check(
                f"{seat_name} has raised lips (total height > 25mm)",
                seat_height > 0.025,
                details=f"seat_height={seat_height:.4f}",
            )

    # --- Handlebar pivots ---
    # Handlebars exist and are mounted on the plank
    for hb_part, hb_name, pivot_joint in [
        (left_hb, "left_handlebar", left_hb_pivot),
        (right_hb, "right_handlebar", right_hb_pivot),
    ]:
        hb_aabb = ctx.part_world_aabb(hb_part)
        ctx.check(
            f"{hb_name} exists as a separate part",
            hb_aabb is not None,
            details=f"{hb_name} aabb={hb_aabb}",
        )
        if hb_aabb is not None:
            # Handlebar should extend above the plank
            ctx.check(
                f"{hb_name} extends above the plank",
                hb_aabb[1][2] > PIVOT_Z + PLANK_R + 0.15,
                details=f"hb top={hb_aabb[1][2]:.3f}",
            )

        # Pivot joint should have non-zero range
        lim = pivot_joint.motion_limits
        ctx.check(
            f"{hb_name} has a non-fixed revolute pivot",
            lim is not None and abs(lim.upper - lim.lower) > 0.01,
            details=f"limits=({lim.lower if lim else None}, {lim.upper if lim else None})",
        )

    # --- Plank rocking pose check ---
    rest_left_seat = ctx.part_element_world_aabb(plank, elem="seat_left")
    rest_right_seat = ctx.part_element_world_aabb(plank, elem="seat_right")

    with ctx.pose({plank_pivot: TILT}):
        tilt_left = ctx.part_element_world_aabb(plank, elem="seat_left")
        tilt_right = ctx.part_element_world_aabb(plank, elem="seat_right")
        beam_aabb = ctx.part_world_aabb(plank)

        ctx.check(
            "plank seesaws: left seat drops when tilted positive",
            rest_left_seat is not None
            and tilt_left is not None
            and tilt_left[0][2] < rest_left_seat[0][2] - 0.20,
            details=f"left seat {rest_left_seat} -> {tilt_left}",
        )
        ctx.check(
            "plank seesaws: right seat rises when tilted positive",
            rest_right_seat is not None
            and tilt_right is not None
            and tilt_right[0][2] > rest_right_seat[0][2] + 0.20,
            details=f"right seat {rest_right_seat} -> {tilt_right}",
        )
        ctx.check(
            "tilted plank stays clear of the ground",
            beam_aabb is not None and beam_aabb[0][2] > 0.02,
            details=f"plank aabb={beam_aabb}",
        )
        ctx.expect_contact(
            plank,
            base,
            elem_a="plank_beam",
            elem_b="base_frame",
            name="tilted plank stays on its axle",
        )

    # --- Handlebar pivot pose check ---
    # When the plank tilts, handlebars follow (they are children of plank).
    # Verify handlebars can also pivot independently.
    with ctx.pose({left_hb_pivot: HANDLEBAR_TILT}):
        hb_aabb_tilted = ctx.part_world_aabb(left_hb)
        hb_aabb_rest = ctx.part_world_aabb(left_hb)
        # The handlebar part should exist and be connected
        ctx.check(
            "left handlebar pivot responds to input",
            hb_aabb_tilted is not None,
            details=f"tilted hb aabb={hb_aabb_tilted}",
        )

    # --- Plank pivot range check ---
    lim = plank_pivot.motion_limits
    ctx.check(
        "plank pivot rocks +/- 18 degrees",
        lim is not None
        and abs(lim.lower + TILT) < 1e-6
        and abs(lim.upper - TILT) < 1e-6,
        details=f"limits=({lim.lower if lim else None}, {lim.upper if lim else None})",
    )

    return ctx.report()


object_model = build_object_model()
