from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
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
# Heavy commercial steel beam seesaw with rubber bumpers and molded seats.
#
# Layout (world frame, Z up, base centered on the origin):
# - Sky-blue base: two arched inverted-U tube legs joined by cross members,
#   forming a central pedestal ~0.65 m tall with a pivot axle at the top.
# - Single heavy yellow steel box beam (~2.8 m long, 100×80 mm section)
#   pivoting on the central axle.
# - Molded seats with raised lips at each end of the beam.
# - Rubber bumpers underneath the beam near each end.
# - T-shaped handlebars just inboard of each seat.
# - Articulation: REVOLUTE, horizontal axis perpendicular to beam, ±18 deg.
# ----------------------------------------------------------------------------

TUBE_R = 0.020       # ~40 mm diameter base tubing
HANDLE_R = 0.014
TILT = math.radians(18.0)

# Base dimensions
ARCH_TOP = 0.65      # pivot height
ARCH_HALF_SPAN = 0.32
ARCH_LEG_FLARE = 0.06

# Beam dimensions
BEAM_LEN = 2.80
BEAM_W = 0.10        # beam width (Y direction)
BEAM_H = 0.08        # beam height (Z direction)
BEAM_CENTER_Z = 0.0  # beam center at pivot height in local frame

# Seat dimensions
SEAT_X = 1.22        # seat center distance from pivot along beam
SEAT_W = 0.28        # seat width
SEAT_D = 0.30        # seat depth
SEAT_THICK = 0.012   # seat base plate thickness
LIP_H = 0.035        # raised lip height
LIP_T = 0.008        # lip wall thickness

# Handlebar dimensions
HANDLE_X = 0.88      # handlebar post position along beam
HANDLE_POST_H = 0.28
HANDLE_BAR_W = 0.30
HANDLE_TOP_Z = BEAM_H / 2.0 + HANDLE_POST_H

# Bumper dimensions
BUMPER_X = 0.95      # bumper position from center
BUMPER_R = 0.035
BUMPER_H = 0.06

# Axle/bearing
AXLE_R = 0.028       # pivot axle radius
AXLE_LEN = 0.26      # axle length (along Y), must span between arches
BEARING_R = 0.042    # bearing sleeve radius on beam

SKY_BLUE = Material("sky_blue_paint", rgba=(0.33, 0.62, 0.84, 1.0))
WORN_YELLOW = Material("worn_yellow_paint", rgba=(0.85, 0.72, 0.10, 1.0))
RUBBER_BLACK = Material("rubber_black", rgba=(0.12, 0.12, 0.12, 1.0))
SEAT_GREEN = Material("molded_green_plastic", rgba=(0.18, 0.45, 0.22, 1.0))
STEEL_GRAY = Material("steel_gray", rgba=(0.45, 0.45, 0.48, 1.0))


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


ARCH_Y_OFFSET = 0.10  # half-distance between two parallel arches along Y (front/back)

# Approximate arch |X| at given Z heights, from the profile spline.
# These are used for placing cross braces between the two arches.
_ARCH_X_AT_Z: list[tuple[float, float]] = [
    (0.15, 0.300),
    (0.30, 0.244),
]


def _arch_profile() -> list[tuple[float, float]]:
    """Return the arch profile (x, z) for one inverted-U leg."""
    return [
        (-ARCH_HALF_SPAN - ARCH_LEG_FLARE, 0.02),
        (-ARCH_HALF_SPAN - ARCH_LEG_FLARE + 0.02, 0.03),
        (-ARCH_HALF_SPAN, 0.08),
        (-0.28, 0.22),
        (-0.20, 0.40),
        (-0.10, 0.55),
        (-0.04, ARCH_TOP - 0.02),
        (0.0, ARCH_TOP),
        (0.04, ARCH_TOP - 0.02),
        (0.10, 0.55),
        (0.20, 0.40),
        (0.28, 0.22),
        (ARCH_HALF_SPAN, 0.08),
        (ARCH_HALF_SPAN + ARCH_LEG_FLARE - 0.02, 0.03),
        (ARCH_HALF_SPAN + ARCH_LEG_FLARE, 0.02),
    ]


def _base_mesh() -> MeshGeometry:
    """Build the entire base as one connected mesh: two arches + cross braces + axle.

    The two arches are in XZ planes at y = ±ARCH_Y_OFFSET (front and back).
    Cross braces run along Y between the arches.
    The pivot axle runs along Y at the arch tops.
    """
    profile = _arch_profile()

    # Build both arches (in XZ planes, offset along Y)
    combined = MeshGeometry()
    for sy in (1.0, -1.0):
        arch_pts = [(u, sy * ARCH_Y_OFFSET, z) for (u, z) in profile]
        arch_tube = tube_from_spline_points(
            arch_pts,
            radius=TUBE_R,
            samples_per_segment=10,
            radial_segments=16,
            cap_ends=True,
        )
        combined.merge(arch_tube)

    # Cross braces along Y connecting the two arch legs at various heights.
    for cz, arch_x in _ARCH_X_AT_Z:
        for sx in (1.0, -1.0):
            brace = _tube_between(
                (sx * arch_x, -ARCH_Y_OFFSET, cz),
                (sx * arch_x, ARCH_Y_OFFSET, cz),
                0.016,
            )
            combined.merge(brace)

    # Pivot axle tube along Y at the top of both arches.
    axle = CylinderGeometry(AXLE_R, AXLE_LEN, radial_segments=20)
    axle.rotate_x(math.pi / 2.0)  # align along Y
    axle.translate(0.0, 0.0, ARCH_TOP)
    combined.merge(axle)

    return combined


def _build_molded_seat() -> cq.Workplane:
    """Build a molded seat with raised lips on three sides (back and two sides).

    The seat is centered at origin, with the seat surface at z=0 and lips
    extending upward. Front is +X direction (open, no lip).
    """
    hw = SEAT_W / 2.0
    hd = SEAT_D / 2.0

    # Base plate
    seat = cq.Workplane("XY").box(SEAT_W, SEAT_D, SEAT_THICK)

    # Slightly dished seat surface - raised lip around back and sides
    # Back lip (at -Y edge)
    back_lip = (
        cq.Workplane("XY")
        .transformed(offset=(0, -hd + LIP_T / 2.0, SEAT_THICK / 2.0 + LIP_H / 2.0))
        .box(SEAT_W, LIP_T, LIP_H)
    )
    # Left side lip (at -X edge)
    left_lip = (
        cq.Workplane("XY")
        .transformed(offset=(-hw + LIP_T / 2.0, 0, SEAT_THICK / 2.0 + LIP_H / 2.0))
        .box(LIP_T, SEAT_D, LIP_H)
    )
    # Right side lip (at +X edge)
    right_lip = (
        cq.Workplane("XY")
        .transformed(offset=(hw - LIP_T / 2.0, 0, SEAT_THICK / 2.0 + LIP_H / 2.0))
        .box(LIP_T, SEAT_D, LIP_H)
    )

    result = seat.union(back_lip).union(left_lip).union(right_lip)
    return result


def _build_beam_body() -> cq.Workplane:
    """Build the heavy rectangular box beam with a central axle bore.

    Beam is centered at origin, extends along X. A cylindrical hole along Y
    at the center allows the pivot axle to pass through.
    """
    beam = (
        cq.Workplane("XY")
        .box(BEAM_LEN, BEAM_W, BEAM_H)
        .edges("|X").fillet(0.006)
    )
    # Cut a bore along Y for the pivot axle to pass through
    bore_r = AXLE_R + 0.004  # clearance around axle
    beam = (
        beam
        .faces(">Y").workplane()
        .circle(bore_r)
        .cutThruAll()
    )
    return beam


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_beam_seesaw")

    # --- Static base ---------------------------------------------------------
    base = model.part("base")

    # Single connected base mesh: two arches + cross braces + pivot axle
    base.visual(
        mesh_from_geometry(_base_mesh(), "base_frame"),
        material=SKY_BLUE,
        name="base_frame",
    )

    # --- Beam assembly -------------------------------------------------------
    beam_part = model.part("beam")

    # Heavy box beam body
    beam_body = _build_beam_body()
    beam_part.visual(
        mesh_from_cadquery(beam_body, "beam_body"),
        material=WORN_YELLOW,
        name="beam_body",
    )

    # Central bearing sleeve (wraps the pivot axle)
    bearing_geom = CylinderGeometry(BEARING_R, BEAM_W + 0.02, radial_segments=20)
    bearing_geom.rotate_x(math.pi / 2.0)
    beam_part.visual(
        mesh_from_geometry(bearing_geom, "bearing_sleeve"),
        material=STEEL_GRAY,
        name="bearing_sleeve",
    )

    # Molded seats at each end
    seat_shape = _build_molded_seat()
    for sx in (1.0, -1.0):
        seat_mesh = mesh_from_cadquery(
            seat_shape,
            f"molded_seat_{int(sx > 0)}",
        )
        beam_part.visual(
            seat_mesh,
            origin=Origin(xyz=(sx * SEAT_X, 0.0, BEAM_H / 2.0 + SEAT_THICK / 2.0)),
            material=SEAT_GREEN,
            name=f"molded_seat_{int(sx > 0)}",
        )

    # Rubber bumpers underneath the beam
    for sx in (1.0, -1.0):
        bumper_geom = CylinderGeometry(BUMPER_R, BUMPER_H, radial_segments=16)
        bumper_geom.translate(sx * BUMPER_X, 0.0, -BEAM_H / 2.0 - BUMPER_H / 2.0)
        beam_part.visual(
            mesh_from_geometry(bumper_geom, f"bumper_{int(sx > 0)}"),
            material=RUBBER_BLACK,
            name=f"bumper_{int(sx > 0)}",
        )

    # T-handlebars just inboard of each seat
    for sx in (1.0, -1.0):
        post = CylinderGeometry(HANDLE_R, HANDLE_POST_H, radial_segments=14)
        post.translate(sx * HANDLE_X, 0.0, BEAM_H / 2.0 + HANDLE_POST_H / 2.0)
        bar = CylinderGeometry(HANDLE_R, HANDLE_BAR_W, radial_segments=14)
        bar.rotate_x(math.pi / 2.0)
        bar.translate(sx * HANDLE_X, 0.0, HANDLE_TOP_Z)
        handle_geom = post.merge(bar)
        beam_part.visual(
            mesh_from_geometry(handle_geom, f"handlebar_{int(sx > 0)}"),
            material=WORN_YELLOW,
            name=f"handlebar_{int(sx > 0)}",
        )

    # --- Articulation --------------------------------------------------------
    limits = MotionLimits(effort=200.0, velocity=2.0, lower=-TILT, upper=TILT)
    model.articulation(
        "beam_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=beam_part,
        origin=Origin(xyz=(0.0, 0.0, ARCH_TOP)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    beam = object_model.get_part("beam")
    pivot = object_model.get_articulation("beam_pivot")

    # Bearing sleeve wraps the pivot axle area of the base frame (intentional overlap)
    ctx.allow_overlap(
        beam,
        base,
        elem_a="bearing_sleeve",
        elem_b="base_frame",
        reason="Beam bearing sleeve intentionally wraps the pivot axle portion of the base frame as a rotational bearing.",
    )
    ctx.allow_overlap(
        beam,
        base,
        elem_a="beam_body",
        elem_b="base_frame",
        reason="Beam body passes through the arch top region at the pivot; the beam rotates around the axle between the two arches.",
    )
    ctx.expect_contact(
        beam,
        base,
        elem_a="bearing_sleeve",
        elem_b="base_frame",
        name="beam bearing rides on the base frame axle",
    )

    # Base is about 0.65 m tall
    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "base is a pedestal stand about 0.65 m tall",
        base_aabb is not None and 0.62 <= base_aabb[1][2] <= 0.75,
        details=f"base aabb={base_aabb}",
    )
    ctx.check(
        "base feet rest on the ground plane",
        base_aabb is not None and -0.01 <= base_aabb[0][2] <= 0.015,
        details=f"base aabb={base_aabb}",
    )

    # Molded seats exist at both ends of the beam with raised lips
    for end_idx in (0, 1):
        seat_name = f"molded_seat_{end_idx}"
        seat_aabb = ctx.part_element_world_aabb(beam, elem=seat_name)
        ctx.check(
            f"molded seat {end_idx} exists at beam end",
            seat_aabb is not None,
            details=f"seat aabb={seat_aabb}",
        )
        if seat_aabb is not None:
            seat_cx = (seat_aabb[0][0] + seat_aabb[1][0]) / 2.0
            seat_cz = (seat_aabb[0][2] + seat_aabb[1][2]) / 2.0
            # Seat should be near beam end and at sit height
            ctx.check(
                f"molded seat {end_idx} is near a beam end at sit height",
                abs(seat_cx) > 1.0 and 0.60 <= seat_cz <= 0.78,
                details=f"seat center=({seat_cx:.3f}, {seat_cz:.3f})",
            )
            # Raised lips: seat height should be more than just the base plate
            seat_h = seat_aabb[1][2] - seat_aabb[0][2]
            ctx.check(
                f"molded seat {end_idx} has raised lips (height > base plate)",
                seat_h > SEAT_THICK + LIP_H * 0.7,
                details=f"seat height={seat_h:.4f}, expected > {SEAT_THICK + LIP_H * 0.7:.4f}",
            )

    # Rubber bumpers exist underneath the beam
    for end_idx in (0, 1):
        bumper_name = f"bumper_{end_idx}"
        bumper_aabb = ctx.part_element_world_aabb(beam, elem=bumper_name)
        ctx.check(
            f"rubber bumper {end_idx} exists under beam",
            bumper_aabb is not None,
            details=f"bumper aabb={bumper_aabb}",
        )
        if bumper_aabb is not None:
            bumper_cz = (bumper_aabb[0][2] + bumper_aabb[1][2]) / 2.0
            beam_aabb = ctx.part_element_world_aabb(beam, elem="beam_body")
            if beam_aabb is not None:
                beam_bottom = beam_aabb[0][2]
                ctx.check(
                    f"bumper {end_idx} hangs below the beam bottom",
                    bumper_cz < beam_bottom,
                    details=f"bumper center z={bumper_cz:.4f}, beam bottom={beam_bottom:.4f}",
                )

    # Handlebars exist and stand upright
    for end_idx in (0, 1):
        handle_name = f"handlebar_{end_idx}"
        handle_aabb = ctx.part_element_world_aabb(beam, elem=handle_name)
        ctx.check(
            f"handlebar {end_idx} exists just inboard of seat",
            handle_aabb is not None,
            details=f"handle aabb={handle_aabb}",
        )
        if handle_aabb is not None:
            handle_top = handle_aabb[1][2]
            beam_aabb = ctx.part_element_world_aabb(beam, elem="beam_body")
            if beam_aabb is not None:
                ctx.check(
                    f"handlebar {end_idx} extends above the beam",
                    handle_top > beam_aabb[1][2] + 0.15,
                    details=f"handle top={handle_top:.4f}, beam top={beam_aabb[1][2]:.4f}",
                )

    # Beam pivot rocks ±18 degrees
    lim = pivot.motion_limits
    ctx.check(
        "beam pivot rocks +/- 18 degrees",
        lim is not None
        and abs(lim.lower + TILT) < 1e-6
        and abs(lim.upper - TILT) < 1e-6,
        details=f"limits=({lim.lower if lim else None}, {lim.upper if lim else None})",
    )

    # Decisive pose check: beam seesaws, one end drops and the other rises
    rest_seat0 = ctx.part_element_world_aabb(beam, elem="molded_seat_0")
    rest_seat1 = ctx.part_element_world_aabb(beam, elem="molded_seat_1")

    with ctx.pose({pivot: TILT}):
        tilt_seat0 = ctx.part_element_world_aabb(beam, elem="molded_seat_0")
        tilt_seat1 = ctx.part_element_world_aabb(beam, elem="molded_seat_1")
        beam_aabb = ctx.part_world_aabb(beam)

        ctx.check(
            "beam seesaws: one end drops, the opposite end rises",
            rest_seat0 is not None
            and tilt_seat0 is not None
            and rest_seat1 is not None
            and tilt_seat1 is not None
            and tilt_seat0[0][2] > rest_seat0[0][2] + 0.30
            and tilt_seat1[0][2] < rest_seat1[0][2] - 0.30,
            details=f"seat0 {rest_seat0} -> {tilt_seat0}, seat1 {rest_seat1} -> {tilt_seat1}",
        )
        ctx.check(
            "fully tilted beam stays clear of the ground",
            beam_aabb is not None and beam_aabb[0][2] > 0.01,
            details=f"beam aabb={beam_aabb}",
        )
        ctx.expect_contact(
            beam,
            base,
            elem_a="bearing_sleeve",
            elem_b="base_frame",
            name="tilted beam bearing stays on the base frame axle",
        )

    return ctx.report()


object_model = build_object_model()
