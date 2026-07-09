from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Spring-assisted modern playground seesaw
#
# World frame: beam runs along X, pivot axis along Y, Z up.
# - A-frame base with two angled tubular steel legs, a central pivot bracket,
#   a horizontal axle, and visible axle caps at each end.
# - Coil spring around the pivot provides damping / return force.
# - Modern beam: painted tubular steel bar with molded seats (raised lips),
#   rubber bumpers, and gusset plates at the pivot.
# - Each handlebar is a separate part on its own revolute joint (±10° tilt).
# - Main beam pivot: revolute, axis (0,1,0), ±20°.
# ---------------------------------------------------------------------------

BEAM_HALF = 1.50
BEAM_W = 0.080
BEAM_T = 0.040
PIVOT_Z = 0.76

# Base geometry
LEG_SPREAD_X = 0.60
LEG_SPREAD_Y = 0.38
LEG_TUBE_R = 0.028
BRACKET_W = 0.12
BRACKET_H = 0.08
BRACKET_T = 0.012

# Axle
AXLE_R = 0.016
AXLE_LEN = 0.24
AXLE_CAP_R = 0.030
AXLE_CAP_T = 0.010

# Spring
SPRING_R = 0.024  # coil mean radius
SPRING_WIRE_R = 0.005
SPRING_HEIGHT = 0.080
SPRING_COILS = 5

# Beam-local: origin at axle center
BAR_BOT = 0.050
BAR_CTR = BAR_BOT + BEAM_T / 2.0
BAR_TOP = BAR_BOT + BEAM_T

SEAT_X = 1.28
HANDLE_X = 1.02
BUMPER_X = 1.42
TILT = math.radians(20.0)
HANDLE_TILT = math.radians(10.0)


def _leg_points(side: float) -> list[tuple[float, float, float]]:
    """One A-frame leg: rises from two feet to the central apex bracket."""
    rise = PIVOT_Z - 0.02
    pts: list[tuple[float, float, float]] = []
    for i in range(11):
        t = -1.0 + 0.2 * i
        s = 1.0 - t * t  # parabolic rise
        x = LEG_SPREAD_X * t
        z = 0.02 + rise * s
        y = side * LEG_SPREAD_Y * (1.0 - 0.85 * s)  # converges to near center
        pts.append((x, y, z))
    return pts


def _spring_points() -> list[tuple[float, float, float]]:
    """Helical centerline for the coil spring around the pivot axle."""
    pts: list[tuple[float, float, float]] = []
    n_per_coil = 24
    total = SPRING_COILS * n_per_coil
    for i in range(total + 1):
        frac = i / total
        angle = 2.0 * math.pi * SPRING_COILS * frac
        y = (frac - 0.5) * SPRING_HEIGHT
        x = SPRING_R * math.cos(angle)
        z = SPRING_R * math.sin(angle)
        pts.append((x, y, z))
    return pts


def _handle_points(x_base: float) -> list[tuple[float, float, float]]:
    """U-shaped handlebar rod centerline, plane across beam (YZ).

    The leg bottoms embed into the stem top region (z ~ 0.040) for connectivity.
    """
    half_w = 0.040
    leg_bot = 0.040  # rod tip embedded in stem top
    arc_z = 0.260
    pts: list[tuple[float, float, float]] = [
        (x_base, -half_w, leg_bot),
        (x_base, -half_w, 0.180),
    ]
    for k in range(7):
        a = math.pi * k / 6.0
        pts.append((x_base, -half_w * math.cos(a), arc_z + half_w * math.sin(a)))
    pts.append((x_base, half_w, 0.180))
    pts.append((x_base, half_w, leg_bot))
    return pts


def _bumper_geometry(x: float, index: int):
    """Rubber bumper: half-annulus shell under beam tip."""
    r_out = 0.055
    r_in = 0.040
    profile: list[tuple[float, float]] = []
    n = 20
    for k in range(n + 1):
        a = math.pi + math.pi * k / n
        profile.append((r_out * math.cos(a), r_out * math.sin(a)))
    for k in range(n + 1):
        a = 2.0 * math.pi - math.pi * k / n
        profile.append((r_in * math.cos(a), r_in * math.sin(a)))
    geom = ExtrudeGeometry(profile, 0.10, cap=True, center=True)
    geom.rotate_x(math.pi / 2.0)
    geom.translate(x, 0.0, BAR_BOT + 0.002)
    return mesh_from_geometry(geom, f"bumper_{index}")


def _gusset_geometry(index: int):
    """Triangular gusset plate connecting pivot post to beam bar."""
    z_lo = 0.040  # above spring extent (0.029), within pivot_post range
    z_hi = BAR_BOT + 0.005  # inside beam bar for connectivity
    profile = [(-0.08, z_hi), (0.08, z_hi), (0.0, z_lo)]
    geom = ExtrudeGeometry(profile, 0.010, cap=True, center=True)
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, f"gusset_{index}")


def _molded_seat_geometry(index: int):
    """Molded plastic seat: rounded rectangle plate with raised lip walls."""
    # Build a rounded-rect base plate
    plate_w = 0.28
    plate_d = 0.22
    plate_t = 0.012
    lip_h = 0.018
    lip_t = 0.008

    profile = rounded_rect_profile(plate_w, plate_d, 0.03, corner_segments=6)
    plate = ExtrudeGeometry(profile, plate_t, cap=True, center=True)
    # Plate centered in XY, spanning z in [-plate_t/2, plate_t/2]

    # Build lip walls as four thin box strips around the seat perimeter
    # Front lip (along +X edge)
    front = ExtrudeGeometry(
        [(plate_w / 2 - lip_t, -plate_d / 2 + 0.02),
         (plate_w / 2, -plate_d / 2 + 0.02),
         (plate_w / 2, plate_d / 2 - 0.02),
         (plate_w / 2 - lip_t, plate_d / 2 - 0.02)],
        lip_h, cap=True, center=True,
    )
    front.translate(0.0, 0.0, plate_t / 2 + lip_h / 2)

    # Back lip (along -X edge)
    back = ExtrudeGeometry(
        [(-plate_w / 2, -plate_d / 2 + 0.02),
         (-plate_w / 2 + lip_t, -plate_d / 2 + 0.02),
         (-plate_w / 2 + lip_t, plate_d / 2 - 0.02),
         (-plate_w / 2, plate_d / 2 - 0.02)],
        lip_h, cap=True, center=True,
    )
    back.translate(0.0, 0.0, plate_t / 2 + lip_h / 2)

    # Left lip (along +Y edge)
    left = ExtrudeGeometry(
        [(-plate_w / 2 + 0.02, plate_d / 2 - lip_t),
         (plate_w / 2 - 0.02, plate_d / 2 - lip_t),
         (plate_w / 2 - 0.02, plate_d / 2),
         (-plate_w / 2 + 0.02, plate_d / 2)],
        lip_h, cap=True, center=True,
    )
    left.translate(0.0, 0.0, plate_t / 2 + lip_h / 2)

    # Right lip (along -Y edge)
    right = ExtrudeGeometry(
        [(-plate_w / 2 + 0.02, -plate_d / 2),
         (plate_w / 2 - 0.02, -plate_d / 2),
         (plate_w / 2 - 0.02, -plate_d / 2 + lip_t),
         (-plate_w / 2 + 0.02, -plate_d / 2 + lip_t)],
        lip_h, cap=True, center=True,
    )
    right.translate(0.0, 0.0, plate_t / 2 + lip_h / 2)

    # Merge all into one mesh
    merged = plate.copy()
    merged = merged.merge(front)
    merged = merged.merge(back)
    merged = merged.merge(left)
    merged = merged.merge(right)
    return mesh_from_geometry(merged, f"molded_seat_{index}")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="spring_modern_seesaw")

    # Materials
    steel_gray = model.material("powder_coat_gray", rgba=(0.38, 0.40, 0.42, 1.0))
    beam_blue = model.material("powder_coat_blue", rgba=(0.12, 0.42, 0.68, 1.0))
    seat_red = model.material("molded_red_plastic", rgba=(0.78, 0.15, 0.12, 1.0))
    rubber = model.material("black_rubber", rgba=(0.06, 0.06, 0.06, 1.0))
    chrome = model.material("chrome_cap", rgba=(0.82, 0.82, 0.84, 1.0))
    spring_steel = model.material("spring_steel", rgba=(0.52, 0.54, 0.50, 1.0))
    handle_grip = model.material("rubber_grip", rgba=(0.18, 0.18, 0.20, 1.0))

    # ================================================================ base ===
    base = model.part("base")

    # A-frame legs
    for i, side in enumerate((1.0, -1.0)):
        base.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _leg_points(side),
                    radius=LEG_TUBE_R,
                    samples_per_segment=8,
                    radial_segments=18,
                    cap_ends=True,
                ),
                f"leg_{i}",
            ),
            material=steel_gray,
            name=f"leg_{i}",
        )

    # Central pivot bracket plate (bottom plate of the clevis, below the sleeve)
    base.visual(
        Box((BRACKET_W, BRACKET_W, BRACKET_T)),
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z - BRACKET_H / 2 + BRACKET_T / 2)),
        material=steel_gray,
        name="pivot_bracket",
    )
    # Vertical side plates of the bracket (clevis-style)
    for i, side in enumerate((1.0, -1.0)):
        base.visual(
            Box((BRACKET_W, BRACKET_T, BRACKET_H)),
            origin=Origin(xyz=(0.0, side * (BRACKET_W / 2 - BRACKET_T / 2), PIVOT_Z)),
            material=steel_gray,
            name=f"bracket_side_{i}",
        )

    # Axle bolt through bracket
    base.visual(
        Cylinder(radius=AXLE_R, length=AXLE_LEN),
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="pivot_axle",
    )

    # Visible axle caps at each end of the support bracket
    for i, side in enumerate((1.0, -1.0)):
        base.visual(
            Cylinder(radius=AXLE_CAP_R, length=AXLE_CAP_T),
            origin=Origin(
                xyz=(0.0, side * (AXLE_LEN / 2.0 + AXLE_CAP_T / 2), PIVOT_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=chrome,
            name=f"axle_cap_{i}",
        )

    # Coil spring around pivot (mounted on base, visually encircling axle area)
    base.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                _spring_points(),
                radius=SPRING_WIRE_R,
                samples_per_segment=4,
                radial_segments=10,
                cap_ends=True,
            ),
            "coil_spring",
        ),
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        material=spring_steel,
        name="coil_spring",
    )

    # ================================================================ beam ===
    beam = model.part("beam")

    # Pivot sleeve (bushing around axle, along Y axis)
    beam.visual(
        Cylinder(radius=0.024, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_gray,
        name="pivot_sleeve",
    )

    # Vertical connecting post from sleeve top to beam bar bottom
    post_h = BAR_BOT  # from z=0 to z=BAR_BOT
    beam.visual(
        Box((0.048, 0.048, post_h)),
        origin=Origin(xyz=(0.0, 0.0, post_h / 2.0)),
        material=steel_gray,
        name="pivot_post",
    )

    # Gusset plates (one each side of beam)
    for i, y_off in enumerate((0.020, -0.020)):
        beam.visual(
            _gusset_geometry(i),
            origin=Origin(xyz=(0.0, y_off, 0.0)),
            material=beam_blue,
            name=f"gusset_{i}",
        )

    # Main beam bar
    beam.visual(
        Box((2.0 * BEAM_HALF, BEAM_W, BEAM_T)),
        origin=Origin(xyz=(0.0, 0.0, BAR_CTR)),
        material=beam_blue,
        name="beam_bar",
    )

    # Molded seats with raised lips
    for i, side in enumerate((1.0, -1.0)):
        beam.visual(
            _molded_seat_geometry(i),
            origin=Origin(xyz=(side * SEAT_X, 0.0, BAR_TOP + 0.006)),
            material=seat_red,
            name=f"seat_{i}",
        )

    # Rubber bumpers under beam tips
    for i, side in enumerate((1.0, -1.0)):
        beam.visual(
            _bumper_geometry(side * BUMPER_X, i),
            material=rubber,
            name=f"bumper_{i}",
        )

    # ========================================================= handlebars ===
    for i, side in enumerate((1.0, -1.0)):
        hb = model.part(f"handlebar_{i}")
        # Handle stem: flat mounting bracket on beam top that reaches both grip legs
        hb.visual(
            Box((0.024, 0.100, 0.060)),
            origin=Origin(xyz=(0.0, 0.0, 0.030)),
            material=steel_gray,
            name=f"stem_{i}",
        )
        # U-shaped grip handle
        hb.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _handle_points(0.0),
                    radius=0.010,
                    samples_per_segment=8,
                    radial_segments=14,
                    cap_ends=True,
                ),
                f"handle_grip_{i}",
            ),
            material=handle_grip,
            name=f"grip_{i}",
        )

        # Articulation: handlebar pivots on beam (about Y axis, ±10°)
        model.articulation(
            f"handlebar_pivot_{i}",
            ArticulationType.REVOLUTE,
            parent=beam,
            child=hb,
            origin=Origin(xyz=(side * HANDLE_X, 0.0, BAR_TOP)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0, velocity=2.0, lower=-HANDLE_TILT, upper=HANDLE_TILT
            ),
        )

    # ========================================================== main pivot ===
    model.articulation(
        "beam_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=200.0, velocity=2.5, lower=-TILT, upper=TILT),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    beam = object_model.get_part("beam")
    pivot = object_model.get_articulation("beam_pivot")
    hb0 = object_model.get_part("handlebar_0")
    hb1 = object_model.get_part("handlebar_1")
    hb0_pivot = object_model.get_articulation("handlebar_pivot_0")
    hb1_pivot = object_model.get_articulation("handlebar_pivot_1")

    # --- Pivot sleeve / axle overlap (intentional bushing) ---
    ctx.allow_overlap(
        beam, base,
        elem_a="pivot_sleeve", elem_b="pivot_axle",
        reason="Pivot sleeve is a bushing intentionally nested around the axle bolt.",
    )
    ctx.expect_contact(
        beam, base,
        elem_a="pivot_sleeve", elem_b="pivot_axle",
        name="pivot sleeve is seated on the axle bolt",
    )
    ctx.expect_within(
        beam, base,
        axes="y",
        inner_elem="pivot_sleeve", outer_elem="pivot_axle",
        margin=0.001,
        name="pivot sleeve stays inside the axle span",
    )

    # --- Spring encircles the pivot sleeve and post (intentional nesting) ---
    ctx.allow_overlap(
        base, beam,
        elem_a="coil_spring", elem_b="pivot_sleeve",
        reason="Coil spring encircles the pivot bushing area as a return mechanism.",
    )
    ctx.allow_overlap(
        base, beam,
        elem_a="coil_spring", elem_b="pivot_post",
        reason="Coil spring wraps around the pivot post that connects sleeve to beam bar.",
    )
    # Axle passes through the lower portion of the pivot post
    ctx.allow_overlap(
        base, beam,
        elem_a="pivot_axle", elem_b="pivot_post",
        reason="Axle bolt passes through the pivot post that structurally bridges sleeve to beam bar.",
    )
    ctx.expect_overlap(
        base, beam,
        axes="xy",
        elem_a="coil_spring", elem_b="pivot_sleeve",
        min_overlap=0.020,
        name="coil spring overlaps the pivot sleeve footprint in XY",
    )
    ctx.expect_within(
        beam, base,
        axes="y",
        inner_elem="pivot_post", outer_elem="pivot_axle",
        margin=0.001,
        name="pivot post stays within the axle span in Y",
    )

    # --- Axle caps visible at the support bracket ---
    for i in range(2):
        cap_box = ctx.part_element_world_aabb(base, elem=f"axle_cap_{i}")
        axle_box = ctx.part_element_world_aabb(base, elem="pivot_axle")
        ctx.check(
            f"axle_cap_{i} is visible at the bracket end",
            cap_box is not None and axle_box is not None
            and (cap_box[1][1] - cap_box[0][1]) > 0.005
            and cap_box[1][1] >= axle_box[1][1] - 0.005
            if i == 0 else cap_box[0][1] <= axle_box[0][1] + 0.005,
            details=f"cap={cap_box}, axle={axle_box}",
        )

    # --- Coil spring visible at pivot ---
    spring_box = ctx.part_element_world_aabb(base, elem="coil_spring")
    ctx.check(
        "coil spring is present near the pivot",
        spring_box is not None
        and abs((spring_box[0][2] + spring_box[1][2]) / 2.0 - PIVOT_Z) < 0.06,
        details=f"spring aabb={spring_box}",
    )

    # --- Molded seats with raised lips ---
    bar_box = ctx.part_element_world_aabb(beam, elem="beam_bar")
    for i in range(2):
        seat_box = ctx.part_element_world_aabb(beam, elem=f"seat_{i}")
        ctx.check(
            f"molded_seat_{i} sits on the beam bar with raised lips",
            seat_box is not None and bar_box is not None
            and seat_box[0][2] >= bar_box[1][2] - 0.005
            and (seat_box[1][2] - seat_box[0][2]) > 0.020,  # lip raises total height above thin plate
            details=f"seat aabb={seat_box}",
        )

    # --- Handlebar pivot joints exist and have correct limits ---
    for j, (hb, hb_piv, hb_name) in enumerate([
        (hb0, hb0_pivot, "handlebar_0"),
        (hb1, hb1_pivot, "handlebar_1"),
    ]):
        ax = hb_piv.axis
        ctx.check(
            f"{hb_name} pivot axis is horizontal Y",
            abs(ax[0]) < 1e-9 and abs(ax[1] - 1.0) < 1e-9 and abs(ax[2]) < 1e-9,
            details=f"axis={ax}",
        )
        lim = hb_piv.motion_limits
        ctx.check(
            f"{hb_name} pivot limits are about ±10 degrees",
            lim is not None and lim.lower is not None and lim.upper is not None
            and abs(lim.lower + HANDLE_TILT) < 1e-6
            and abs(lim.upper - HANDLE_TILT) < 1e-6,
            details=f"limits=({lim.lower}, {lim.upper})",
        )
        # Handlebar should move when pivoted (check top of grip via max Z corner)
        rest_pos = ctx.part_element_world_aabb(hb, elem=f"grip_{j}")
        with ctx.pose({hb_piv: HANDLE_TILT}):
            tilted_pos = ctx.part_element_world_aabb(hb, elem=f"grip_{j}")
            ctx.check(
                f"{hb_name} grip moves when pivoted",
                rest_pos is not None and tilted_pos is not None
                and (abs(tilted_pos[1][0] - rest_pos[1][0]) + abs(tilted_pos[1][2] - rest_pos[1][2]) > 0.010),
                details=f"rest_max={rest_pos[1]}, tilted_max={tilted_pos[1]}",
            )

    # --- Main beam pivot ---
    ax = pivot.axis
    ctx.check(
        "main pivot axis is horizontal Y",
        abs(ax[0]) < 1e-9 and abs(ax[1] - 1.0) < 1e-9 and abs(ax[2]) < 1e-9,
        details=f"axis={ax}",
    )
    lim = pivot.motion_limits
    ctx.check(
        "main rocking limits are about ±20 degrees",
        lim is not None and lim.lower is not None and lim.upper is not None
        and abs(lim.lower + TILT) < 1e-6 and abs(lim.upper - TILT) < 1e-6,
        details=f"limits=({lim.lower}, {lim.upper})",
    )

    # --- Scale checks ---
    bar_aabb = ctx.part_element_world_aabb(beam, elem="beam_bar")
    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "beam is about 3.0 m long",
        bar_aabb is not None and abs((bar_aabb[1][0] - bar_aabb[0][0]) - 3.0) < 0.04,
        details=f"bar aabb={bar_aabb}",
    )
    ctx.check(
        "base rests on the ground",
        base_aabb is not None and -0.01 <= base_aabb[0][2] <= 0.03,
        details=f"base aabb={base_aabb}",
    )

    # --- Beam rocking pose ---
    rest_b0 = ctx.part_element_world_aabb(beam, elem="bumper_0")
    with ctx.pose({pivot: TILT}):
        down_b0 = ctx.part_element_world_aabb(beam, elem="bumper_0")
        up_b1 = ctx.part_element_world_aabb(beam, elem="bumper_1")
        ctx.check(
            "positive rock lowers +X end",
            rest_b0 is not None and down_b0 is not None
            and down_b0[0][2] < rest_b0[0][2] - 0.30,
            details=f"rest={rest_b0}, tilted={down_b0}",
        )
        ctx.check(
            "positive rock raises -X end",
            up_b1 is not None and up_b1[0][2] > 1.0,
            details=f"raised bumper={up_b1}",
        )

    return ctx.report()


object_model = build_object_model()
