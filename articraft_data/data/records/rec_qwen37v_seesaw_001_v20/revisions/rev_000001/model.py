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
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Vintage playground seesaw – variant 20
#
# World frame: beam runs along X, pivot axis along Y, Z up.
# - Two bent galvanized-steel tube arches (~50 mm dia) cross side by side and
#   form an A-shaped saddle; the apex carries a horizontal pivot axle bolt.
#   Rubber ground pads sit under each arch foot.
# - The rocking beam is a 3.0 m mustard-yellow steel bar (80 x 40 mm) with a
#   pivot sleeve + triangular gusset at center, asymmetric wooden seats
#   (one raised, one low), an inverted-U grab handle per end, textured
#   footrests near each seat.
# - Rubber tire-section bumpers are separate parts on short prismatic joints
#   allowing vertical compression travel under each beam tip.
# - Main pivot: REVOLUTE, axis (0, 1, 0), +/- 20 degrees.
#   Positive q lowers the +X end (right-hand rule about +Y).
# - Bumper compression: PRISMATIC, axis (0, 0, 1), 0 to 0.025 m.
# ---------------------------------------------------------------------------

BEAM_HALF = 1.50  # 3.0 m beam
BEAM_W = 0.08
BEAM_T = 0.04
PIVOT_Z = 0.76  # axle height (about 0.8 m tall at the pivot)

ARCH_FOOT_X = 0.66
ARCH_FOOT_Y = 0.34
ARCH_APEX_Y = 0.05  # each arch crosses past center so the pair reads as an A
ARCH_FOOT_Z = 0.028
TUBE_R = 0.025  # ~50 mm diameter bent tube

AXLE_R = 0.016
AXLE_LEN = 0.22

# Beam-local frame: origin at the axle center; the bar bottom sits 50 mm above.
BAR_BOT = 0.05
BAR_CTR = BAR_BOT + BEAM_T / 2.0  # 0.07
BAR_TOP = BAR_BOT + BEAM_T  # 0.09

SEAT_X = 1.30
HANDLE_X = 1.04
BUMPER_X = 1.42
FOOTREST_X = 1.18  # between handle and seat
TILT = math.radians(20.0)

# Asymmetric seat heights
SEAT_HIGH_THICK = 0.035  # thick wooden seat on the +X end
SEAT_HIGH_RISER = 0.040  # steel riser block under the high seat
SEAT_LOW_THICK = 0.022   # thin wooden seat on the -X end

# Bumper compression travel
BUMPER_COMPRESS = 0.025  # 25 mm vertical compression

# Ground pad dimensions
PAD_L = 0.14
PAD_W = 0.10
PAD_T = 0.012


def _arch_points(side: float) -> list[tuple[float, float, float]]:
    """Centerline of one bent-tube arch.

    The arch spans X, rises as a parabola to the apex, and leans inward in Y
    from its feet (y = side * ARCH_FOOT_Y) past the centerline to the apex
    (y = -side * ARCH_APEX_Y) so the two arches cross below the saddle.
    """
    pts: list[tuple[float, float, float]] = []
    rise = PIVOT_Z - ARCH_FOOT_Z
    for i in range(11):
        t = -1.0 + 0.2 * i
        s = 1.0 - t * t
        x = ARCH_FOOT_X * t
        z = ARCH_FOOT_Z + rise * s
        y = side * ARCH_FOOT_Y + (-side * ARCH_APEX_Y - side * ARCH_FOOT_Y) * s
        pts.append((x, y, z))
    return pts


def _handle_points(x: float) -> list[tuple[float, float, float]]:
    """Inverted-U grab-handle rod centerline, plane across the beam (YZ)."""
    half_w = 0.035
    leg_bot = BAR_TOP - 0.010  # rod tip embedded in the beam bar
    arc_z = 0.275
    pts: list[tuple[float, float, float]] = [
        (x, -half_w, leg_bot),
        (x, -half_w, 0.190),
    ]
    for k in range(7):  # semicircular top bend
        a = math.pi * k / 6.0
        pts.append((x, -half_w * math.cos(a), arc_z + half_w * math.sin(a)))
    pts.append((x, half_w, 0.190))
    pts.append((x, half_w, leg_bot))
    return pts


def _bumper_geometry(index: int):
    """Curved tire-section bumper centered at origin for use as a separate part.

    The half-annulus shell spans across the beam (Y axis).
    """
    r_out = 0.065
    r_in = 0.048
    profile: list[tuple[float, float]] = []
    n = 20
    for k in range(n + 1):  # outer arc, bottom half (pi .. 2*pi)
        a = math.pi + math.pi * k / n
        profile.append((r_out * math.cos(a), r_out * math.sin(a)))
    for k in range(n + 1):  # inner arc back (2*pi .. pi)
        a = 2.0 * math.pi - math.pi * k / n
        profile.append((r_in * math.cos(a), r_in * math.sin(a)))
    geom = ExtrudeGeometry(profile, 0.10, cap=True, center=True)
    # Profile was authored in the XZ plane; rotate the extrusion onto Y.
    geom.rotate_x(math.pi / 2.0)
    # Geometry is now centered at origin; articulation will position it.
    return mesh_from_geometry(geom, f"bumper_shell_{index}")


def _gusset_geometry():
    """Triangular gusset plate joining the beam bar to the pivot sleeve."""
    profile = [(-0.10, 0.055), (0.10, 0.055), (0.0, 0.020)]
    geom = ExtrudeGeometry(profile, 0.020, cap=True, center=True)
    geom.rotate_x(math.pi / 2.0)  # plate thickness across Y
    return mesh_from_geometry(geom, "seesaw_gusset_plate")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_playground_seesaw")

    galvanized = model.material("weathered_galvanized", rgba=(0.55, 0.58, 0.56, 1.0))
    rust = model.material("rust_steel", rgba=(0.42, 0.25, 0.13, 1.0))
    mustard = model.material("rusty_mustard_paint", rgba=(0.74, 0.53, 0.12, 1.0))
    pale_steel = model.material("pale_weathered_steel", rgba=(0.70, 0.66, 0.58, 1.0))
    wood = model.material("worn_wood", rgba=(0.60, 0.45, 0.28, 1.0))
    rubber = model.material("black_rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    rubber_pad = model.material("ground_pad_rubber", rgba=(0.12, 0.12, 0.10, 1.0))
    footrest_mat = model.material("textured_steel", rgba=(0.45, 0.42, 0.38, 1.0))

    # --------------------------------------------------------------- base ---
    base = model.part("arched_base")
    for i, side in enumerate((1.0, -1.0)):
        base.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _arch_points(side),
                    radius=TUBE_R,
                    samples_per_segment=8,
                    radial_segments=18,
                    cap_ends=True,
                ),
                f"seesaw_arch_{i}",
            ),
            material=galvanized,
            name=f"arch_{i}",
        )

    # Pivot axle bolt through both flattened arch apexes, axis along Y.
    base.visual(
        Cylinder(radius=AXLE_R, length=AXLE_LEN),
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rust,
        name="pivot_axle",
    )
    for i, side in enumerate((1.0, -1.0)):
        base.visual(
            Cylinder(radius=0.024, length=0.014),
            origin=Origin(
                xyz=(0.0, side * (AXLE_LEN / 2.0 - 0.006), PIVOT_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=rust,
            name=f"axle_nut_{i}",
        )

    # Rubber ground pads under each arch foot (4 pads total).
    pad_idx = 0
    for side_y in (1.0, -1.0):
        for side_x in (1.0, -1.0):
            foot_x = side_x * ARCH_FOOT_X
            foot_y = side_y * ARCH_FOOT_Y
            base.visual(
                Box((PAD_L, PAD_W, PAD_T)),
                origin=Origin(xyz=(foot_x, foot_y, ARCH_FOOT_Z - PAD_T / 2.0 + 0.002)),
                material=rubber_pad,
                name=f"ground_pad_{pad_idx}",
            )
            pad_idx += 1

    # --------------------------------------------------------------- beam ---
    # Beam part frame sits at the axle center so the joint is at its origin.
    beam = model.part("beam")

    beam.visual(
        Cylinder(radius=0.026, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rust,
        name="pivot_sleeve",
    )
    beam.visual(_gusset_geometry(), material=mustard, name="gusset_plate")

    beam.visual(
        Box((2.0 * BEAM_HALF, BEAM_W, BEAM_T)),
        origin=Origin(xyz=(0.0, 0.0, BAR_CTR)),
        material=mustard,
        name="beam_bar",
    )

    # Rust streak patches wrapping the painted bar (cosmetic weathering).
    for i, px in enumerate((-0.85, -0.30, 0.45, 0.95)):
        beam.visual(
            Box((0.16, BEAM_W + 0.004, 0.012)),
            origin=Origin(xyz=(px, 0.0, BAR_TOP - 0.004)),
            material=rust,
            name=f"rust_patch_{i}",
        )

    # --- Asymmetric seats ---
    # +X end (index 0): raised seat with steel riser block + thick wood plate
    beam.visual(
        Box((0.28, 0.22, SEAT_HIGH_RISER)),
        origin=Origin(xyz=(SEAT_X, 0.0, BAR_TOP + SEAT_HIGH_RISER / 2.0)),
        material=pale_steel,
        name="seat_riser_0",
    )
    beam.visual(
        Box((0.30, 0.24, SEAT_HIGH_THICK)),
        origin=Origin(xyz=(SEAT_X, 0.0, BAR_TOP + SEAT_HIGH_RISER + SEAT_HIGH_THICK / 2.0)),
        material=wood,
        name="seat_0",
    )

    # -X end (index 1): low seat directly on bar
    beam.visual(
        Box((0.30, 0.24, SEAT_LOW_THICK)),
        origin=Origin(xyz=(-SEAT_X, 0.0, BAR_TOP + SEAT_LOW_THICK / 2.0)),
        material=wood,
        name="seat_1",
    )

    # --- Grab handles (symmetric) ---
    for i, side in enumerate((1.0, -1.0)):
        beam.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _handle_points(side * HANDLE_X),
                    radius=0.009,
                    samples_per_segment=8,
                    radial_segments=16,
                    cap_ends=True,
                ),
                f"seesaw_handle_{i}",
            ),
            material=pale_steel,
            name=f"handle_{i}",
        )

    # --- Textured footrests near each seat ---
    for i, side in enumerate((1.0, -1.0)):
        fx = side * FOOTREST_X
        # Base plate
        beam.visual(
            Box((0.18, 0.16, 0.006)),
            origin=Origin(xyz=(fx, 0.0, BAR_TOP + 0.003)),
            material=footrest_mat,
            name=f"footrest_plate_{i}",
        )
        # Raised grip ridges (4 bars across the plate)
        for r in range(4):
            ry = -0.054 + r * 0.036
            beam.visual(
                Box((0.16, 0.012, 0.005)),
                origin=Origin(xyz=(fx, ry, BAR_TOP + 0.006 + 0.0025)),
                material=rust,
                name=f"footrest_ridge_{i}_{r}",
            )

    # ------------------------------------------------------- bumper parts ---
    # Each bumper is a separate part on a short prismatic joint allowing
    # vertical compression when the beam tip nears the ground.
    bumpers = []
    for i, side in enumerate((1.0, -1.0)):
        bp = model.part(f"bumper_{i}")
        bp.visual(
            _bumper_geometry(i),
            material=rubber,
            name=f"bumper_shell_{i}",
        )
        bumpers.append(bp)

    # -------------------------------------------------------------- joints ---
    # Main beam pivot
    model.articulation(
        "beam_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),  # positive q lowers the +X end
        motion_limits=MotionLimits(effort=200.0, velocity=2.5, lower=-TILT, upper=TILT),
    )

    # Bumper compression prismatic joints
    # At q=0 (rest), the bumper hangs at its normal position below the beam tip.
    # Positive q (axis=(0,0,1)) compresses the bumper upward toward the bar.
    for i, side in enumerate((1.0, -1.0)):
        bumper_rest_z = BAR_BOT + 0.002  # rest position in beam frame
        model.articulation(
            f"bumper_{i}_compress",
            ArticulationType.PRISMATIC,
            parent=beam,
            child=bumpers[i],
            origin=Origin(xyz=(side * BUMPER_X, 0.0, bumper_rest_z)),
            axis=(0.0, 0.0, 1.0),  # positive q = upward compression
            motion_limits=MotionLimits(
                effort=500.0, velocity=0.5, lower=0.0, upper=BUMPER_COMPRESS
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("arched_base")
    beam = object_model.get_part("beam")
    bumper_0 = object_model.get_part("bumper_0")
    bumper_1 = object_model.get_part("bumper_1")
    pivot = object_model.get_articulation("beam_pivot")
    bump0_joint = object_model.get_articulation("bumper_0_compress")
    bump1_joint = object_model.get_articulation("bumper_1_compress")

    # ---- Main pivot checks ----
    # The beam's pivot sleeve intentionally captures the base axle bolt.
    ctx.allow_overlap(
        beam,
        base,
        elem_a="pivot_sleeve",
        elem_b="pivot_axle",
        reason="Pivot sleeve is a bushing intentionally nested around the axle bolt.",
    )
    ctx.expect_contact(
        beam,
        base,
        elem_a="pivot_sleeve",
        elem_b="pivot_axle",
        name="pivot sleeve is seated on the axle bolt",
    )
    ctx.expect_within(
        beam,
        base,
        axes="y",
        inner_elem="pivot_sleeve",
        outer_elem="pivot_axle",
        margin=0.001,
        name="pivot sleeve stays inside the axle span",
    )

    # Beam bar rides just above the arch saddle, not embedded in it.
    ctx.expect_gap(
        beam,
        base,
        axis="z",
        positive_elem="beam_bar",
        negative_elem="arch_0",
        min_gap=0.005,
        max_gap=0.06,
        name="beam bar clears the arch saddle",
    )

    # Joint configuration: horizontal Y axis, +/- 20 degree rocking limits.
    ax = pivot.axis
    ctx.check(
        "pivot axis is horizontal and perpendicular to the beam",
        abs(ax[0]) < 1e-9 and abs(ax[1] - 1.0) < 1e-9 and abs(ax[2]) < 1e-9,
        details=f"axis={ax}",
    )
    lim = pivot.motion_limits
    ctx.check(
        "rocking limits are about +/- 20 degrees",
        lim is not None
        and lim.lower is not None
        and lim.upper is not None
        and abs(lim.lower + TILT) < 1e-6
        and abs(lim.upper - TILT) < 1e-6,
        details=f"limits=({lim.lower}, {lim.upper})",
    )

    # Hero geometry: scale, saddle height, grounded feet.
    bar_box = ctx.part_element_world_aabb(beam, elem="beam_bar")
    base_box = ctx.part_world_aabb(base)
    axle_box = ctx.part_element_world_aabb(base, elem="pivot_axle")
    ctx.check(
        "beam is about 3.0 m long",
        bar_box is not None and abs((bar_box[1][0] - bar_box[0][0]) - 3.0) < 0.02,
        details=f"bar aabb={bar_box}",
    )
    ctx.check(
        "arched base feet rest on the ground",
        base_box is not None and -0.02 <= base_box[0][2] <= 0.02,
        details=f"base aabb={base_box}",
    )
    ctx.check(
        "pivot axle sits about 0.8 m high",
        axle_box is not None and 0.70 <= (axle_box[0][2] + axle_box[1][2]) / 2.0 <= 0.82,
        details=f"axle aabb={axle_box}",
    )

    # ---- Variant 20: asymmetric seat heights ----
    seat0_box = ctx.part_element_world_aabb(beam, elem="seat_0")
    seat1_box = ctx.part_element_world_aabb(beam, elem="seat_1")
    ctx.check(
        "seat_0 (high) top is higher above bar than seat_1 (low) top",
        seat0_box is not None
        and seat1_box is not None
        and bar_box is not None
        and (seat0_box[1][2] - bar_box[1][2]) > (seat1_box[1][2] - bar_box[1][2]) + 0.03,
        details=f"seat0 top={seat0_box[1][2]}, seat1 top={seat1_box[1][2]}, bar top={bar_box[1][2]}",
    )
    ctx.check(
        "seat_0 sits on a riser block above the bar",
        seat0_box is not None
        and bar_box is not None
        and seat0_box[0][2] > bar_box[1][2] + 0.02,
        details=f"seat0 bottom={seat0_box[0][2]}, bar top={bar_box[1][2]}",
    )
    ctx.check(
        "seat_1 sits directly on the bar",
        seat1_box is not None
        and bar_box is not None
        and abs(seat1_box[0][2] - bar_box[1][2]) < 0.005,
        details=f"seat1 bottom={seat1_box[0][2]}, bar top={bar_box[1][2]}",
    )

    # ---- Variant 20: rubber ground pads under support legs ----
    for p in range(4):
        pad_box = ctx.part_element_world_aabb(base, elem=f"ground_pad_{p}")
        ctx.check(
            f"ground_pad_{p} is near ground level",
            pad_box is not None and pad_box[0][2] < 0.04 and pad_box[1][2] > -0.01,
            details=f"pad aabb={pad_box}",
        )

    # ---- Variant 20: textured footrests near each seat ----
    for i in range(2):
        fp_box = ctx.part_element_world_aabb(beam, elem=f"footrest_plate_{i}")
        ctx.check(
            f"footrest_plate_{i} is on the beam near seat_{i}",
            fp_box is not None
            and bar_box is not None
            and fp_box[0][2] > bar_box[1][2] - 0.002
            and fp_box[1][2] > bar_box[1][2],
            details=f"footrest aabb={fp_box}",
        )
        # At least one ridge visible
        ridge_box = ctx.part_element_world_aabb(beam, elem=f"footrest_ridge_{i}_0")
        ctx.check(
            f"footrest_ridge_{i}_0 stands above the footrest plate",
            ridge_box is not None
            and fp_box is not None
            and ridge_box[1][2] > fp_box[1][2] - 0.001,
            details=f"ridge aabb={ridge_box}",
        )

    # ---- Variant 20: bumper compression prismatic joints ----
    for i, joint in enumerate((bump0_joint, bump1_joint)):
        jlim = joint.motion_limits
        ctx.check(
            f"bumper_{i}_compress is prismatic with vertical axis",
            joint.articulation_type == ArticulationType.PRISMATIC
            and abs(joint.axis[2] - 1.0) < 1e-6,
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )
        ctx.check(
            f"bumper_{i}_compress has compression travel of about {BUMPER_COMPRESS} m",
            jlim is not None
            and jlim.lower is not None
            and abs(jlim.lower) < 1e-6
            and abs(jlim.upper - BUMPER_COMPRESS) < 1e-4,
            details=f"limits=({jlim.lower}, {jlim.upper})",
        )

    # Bumper rest pose: hanging below the beam bar tip
    for i, side in enumerate((1.0, -1.0)):
        bumper_box = ctx.part_element_world_aabb(object_model.get_part(f"bumper_{i}"), elem=f"bumper_shell_{i}")
        ctx.check(
            f"bumper_{i} hangs below the beam at rest",
            bumper_box is not None
            and bar_box is not None
            and bumper_box[0][2] < bar_box[0][2],
            details=f"bumper aabb={bumper_box}",
        )

    # Bumper compression pose: positive q moves bumper upward
    rest_b0_z = ctx.part_element_world_aabb(bumper_0, elem="bumper_shell_0")
    with ctx.pose({bump0_joint: BUMPER_COMPRESS}):
        comp_b0_z = ctx.part_element_world_aabb(bumper_0, elem="bumper_shell_0")
        ctx.check(
            "bumper_0 compresses upward at max travel",
            rest_b0_z is not None
            and comp_b0_z is not None
            and comp_b0_z[0][2] > rest_b0_z[0][2] + 0.01,
            details=f"rest={rest_b0_z}, compressed={comp_b0_z}",
        )

    # ---- Decisive rocking pose checks ----
    rest_b0 = ctx.part_element_world_aabb(bumper_0, elem="bumper_shell_0")
    with ctx.pose({pivot: TILT}):
        down_b0 = ctx.part_element_world_aabb(bumper_0, elem="bumper_shell_0")
        up_b1 = ctx.part_element_world_aabb(bumper_1, elem="bumper_shell_1")
        ctx.check(
            "positive rock lowers the +X end near the ground",
            rest_b0 is not None
            and down_b0 is not None
            and down_b0[0][2] < rest_b0[0][2] - 0.40
            and down_b0[0][2] > 0.0,
            details=f"rest={rest_b0}, tilted={down_b0}",
        )
        ctx.check(
            "positive rock raises the -X end",
            up_b1 is not None and up_b1[0][2] > 1.0,
            details=f"raised bumper aabb={up_b1}",
        )
    with ctx.pose({pivot: -TILT}):
        down_b1 = ctx.part_element_world_aabb(bumper_1, elem="bumper_shell_1")
        ctx.check(
            "negative rock lowers the -X end near the ground",
            down_b1 is not None and 0.0 < down_b1[0][2] < 0.32,
            details=f"tilted bumper aabb={down_b1}",
        )

    # Handle checks
    for i in range(2):
        handle = ctx.part_element_world_aabb(beam, elem=f"handle_{i}")
        ctx.check(
            f"handle_{i} stands about 0.25 m above the beam",
            handle is not None
            and bar_box is not None
            and handle[1][2] > bar_box[1][2] + 0.18
            and handle[0][2] < bar_box[1][2],
            details=f"handle aabb={handle}",
        )

    return ctx.report()


object_model = build_object_model()
