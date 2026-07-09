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
    sweep_profile_along_spline,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Vintage playground seesaw — variant 06: curved beam with raised ends,
# rubber compression bumpers on prismatic joints, rubber ground pads.
#
# World frame: beam runs along X, pivot axis along Y, Z up.
# - Two bent galvanized-steel tube arches (~50 mm dia) cross side by side and
#   form an A-shaped saddle; the apex carries a horizontal pivot axle bolt.
# - Rubber ground pads sit under each arch foot.
# - The rocking beam is a 3.0 m curved rectangular bar (80 x 40 mm) that
#   rises at both ends, painted mustard yellow with rust streaks.
# - Each end carries a wooden seat plate, an inverted-U grab handle, and a
#   rubber compression bumper on a short prismatic joint.
# - Revolute joint at the apex, axis (0, 1, 0), +/- 20 degrees.
# ---------------------------------------------------------------------------

BEAM_HALF = 1.50  # 3.0 m beam
BEAM_W = 0.08
BEAM_T = 0.04
PIVOT_Z = 0.77  # axle height (about 0.8 m tall at the pivot)

# Curved beam: ends rise above center
CURVE_RISE = 0.14  # how much the beam tips rise above the center

ARCH_FOOT_X = 0.66
ARCH_FOOT_Y = 0.34
ARCH_APEX_Y = 0.05
TUBE_R = 0.025  # ~50 mm diameter bent tube

# Rubber ground pads under each arch foot
PAD_HEIGHT = 0.026
PAD_RADIUS = 0.055
# The spline tube mesh bottom sits slightly above the nominal (foot_z - tube_r)
# due to spline fitting, so we lower ARCH_FOOT_Z to make the arches contact the pads.
ARCH_FOOT_Z = 0.040  # adjusted so arch tube bottom contacts pad top

AXLE_R = 0.016
AXLE_LEN = 0.22

# Beam-local frame: origin at the axle center; the bar centerline at x=0
# sits at BAR_CTR above the pivot.
BAR_CTR = 0.07

SEAT_X = 1.30
HANDLE_X = 1.04
BUMPER_X = 1.42
TILT = math.radians(20.0)

# Bumper prismatic joint
BUMPER_TRAVEL = 0.030  # 30 mm compression travel


def beam_curve_z(x: float) -> float:
    """Beam centerline Z in beam-local frame at a given X position."""
    t = x / BEAM_HALF
    return BAR_CTR + CURVE_RISE * t * t


def _arch_points(side: float) -> list[tuple[float, float, float]]:
    """Centerline of one bent-tube arch."""
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


def _beam_path_points() -> list[tuple[float, float, float]]:
    """Centerline path for the curved beam (beam-local frame)."""
    pts: list[tuple[float, float, float]] = []
    n = 24
    for i in range(n + 1):
        t = -1.0 + 2.0 * i / n
        x = BEAM_HALF * t
        z = beam_curve_z(x)
        pts.append((x, 0.0, z))
    return pts


def _handle_points(x: float) -> list[tuple[float, float, float]]:
    """Inverted-U grab-handle rod centerline, plane across the beam (YZ)."""
    half_w = 0.035
    bar_top_local = beam_curve_z(x) + BEAM_T / 2.0
    leg_bot = bar_top_local - 0.010  # rod tip embedded in the beam bar
    arc_z = bar_top_local + 0.235
    pts: list[tuple[float, float, float]] = [
        (x, -half_w, leg_bot),
        (x, -half_w, bar_top_local + 0.120),
    ]
    for k in range(7):  # semicircular top bend
        a = math.pi * k / 6.0
        pts.append((x, -half_w * math.cos(a), arc_z + half_w * math.sin(a)))
    pts.append((x, half_w, bar_top_local + 0.120))
    pts.append((x, half_w, leg_bot))
    return pts


def _curved_beam_mesh():
    """Sweep a rectangular profile along the curved beam path."""
    profile = rounded_rect_profile(BEAM_W, BEAM_T, radius=0.005)
    geom = sweep_profile_along_spline(
        _beam_path_points(),
        profile=profile,
        samples_per_segment=6,
        cap_profile=True,
        up_hint=(0.0, 0.0, 1.0),
    )
    return mesh_from_geometry(geom, "curved_beam_bar")


def _gusset_geometry():
    """Triangular gusset plate joining the beam bar to the pivot sleeve."""
    profile = [(-0.10, 0.055), (0.10, 0.055), (0.0, 0.020)]
    geom = ExtrudeGeometry(profile, 0.020, cap=True, center=True)
    geom.rotate_x(math.pi / 2.0)  # plate thickness across Y
    return mesh_from_geometry(geom, "seesaw_gusset_plate")


BUMPER_THICK = 0.045  # bumper block thickness (Z)
BUMPER_TOP_OFFSET = 0.004  # top of bumper block embeds slightly into beam bottom (bolted contact)

def _bumper_mesh(index: int):
    """Rubber compression bumper pad (short rounded block)."""
    profile = rounded_rect_profile(0.09, BUMPER_THICK, radius=0.006)
    geom = ExtrudeGeometry(profile, 0.09, cap=True, center=True)
    # Profile is in XY; extrusion along Z. Rotate so the block hangs
    # with its thickness along Z (vertical compression direction).
    geom.rotate_x(math.pi / 2.0)  # extrusion onto Y
    # Now block: 0.09 wide (X), 0.09 deep (Y extrusion), BUMPER_THICK tall (Z)
    # Offset downward so top surface is below the part origin
    geom.translate(0.0, 0.0, BUMPER_TOP_OFFSET - BUMPER_THICK / 2.0)
    return mesh_from_geometry(geom, f"seesaw_bumper_{index}")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_playground_seesaw")

    galvanized = model.material("weathered_galvanized", rgba=(0.55, 0.58, 0.56, 1.0))
    rust = model.material("rust_steel", rgba=(0.42, 0.25, 0.13, 1.0))
    mustard = model.material("rusty_mustard_paint", rgba=(0.74, 0.53, 0.12, 1.0))
    pale_steel = model.material("pale_weathered_steel", rgba=(0.70, 0.66, 0.58, 1.0))
    wood = model.material("worn_wood", rgba=(0.60, 0.45, 0.28, 1.0))
    rubber = model.material("black_rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    pad_rubber = model.material("ground_pad_rubber", rgba=(0.12, 0.12, 0.10, 1.0))

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

    # Rubber ground pads under each arch foot (4 feet total).
    pad_idx = 0
    for side in (1.0, -1.0):
        for foot_x_sign in (1.0, -1.0):
            base.visual(
                Cylinder(radius=PAD_RADIUS, length=PAD_HEIGHT),
                origin=Origin(
                    xyz=(
                        foot_x_sign * ARCH_FOOT_X,
                        side * ARCH_FOOT_Y,
                        PAD_HEIGHT / 2.0,
                    )
                ),
                material=pad_rubber,
                name=f"ground_pad_{pad_idx}",
            )
            pad_idx += 1

    # --------------------------------------------------------------- beam ---
    beam = model.part("beam")

    beam.visual(
        Cylinder(radius=0.026, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rust,
        name="pivot_sleeve",
    )
    beam.visual(_gusset_geometry(), material=mustard, name="gusset_plate")

    # Curved beam bar (swept rectangular profile with raised ends).
    beam.visual(
        _curved_beam_mesh(),
        material=mustard,
        name="beam_bar",
    )

    # Rust streak patches wrapping the painted bar (cosmetic weathering).
    for i, px in enumerate((-0.85, -0.30, 0.45, 0.95)):
        bar_z = beam_curve_z(px)
        beam.visual(
            Box((0.16, BEAM_W + 0.004, 0.012)),
            origin=Origin(xyz=(px, 0.0, bar_z + BEAM_T / 2.0 - 0.004)),
            material=rust,
            name=f"rust_patch_{i}",
        )

    # Seats and handles on the beam (follow the curve).
    for i, side in enumerate((1.0, -1.0)):
        seat_z = beam_curve_z(side * SEAT_X) + BEAM_T / 2.0 + 0.008
        beam.visual(
            Box((0.30, 0.24, 0.022)),
            origin=Origin(xyz=(side * SEAT_X, 0.0, seat_z)),
            material=wood,
            name=f"seat_{i}",
        )
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

    # ---------------------------------------------------- bumper parts ---
    # Each bumper is a separate part on a short prismatic joint (vertical
    # compression). The bumper part frame sits at the beam bottom surface.
    for i, side in enumerate((1.0, -1.0)):
        bumper_part = model.part(f"bumper_{i}")
        bumper_part.visual(
            _bumper_mesh(i),
            material=rubber,
            name=f"bumper_pad_{i}",
        )

        # Prismatic joint: parent=beam, child=bumper, axis=(0,0,1)
        # At q=0 the bumper hangs at rest below the beam.
        # At q=BUMPER_TRAVEL the bumper is fully compressed upward.
        bumper_z = beam_curve_z(side * BUMPER_X) - BEAM_T / 2.0
        model.articulation(
            f"bumper_{i}_slide",
            ArticulationType.PRISMATIC,
            parent=beam,
            child=bumper_part,
            origin=Origin(xyz=(side * BUMPER_X, 0.0, bumper_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=800.0,
                velocity=0.5,
                lower=0.0,
                upper=BUMPER_TRAVEL,
            ),
        )

    # -------------------------------------------------------------- joint ---
    model.articulation(
        "beam_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),  # positive q lowers the +X end
        motion_limits=MotionLimits(effort=200.0, velocity=2.5, lower=-TILT, upper=TILT),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("arched_base")
    beam = object_model.get_part("beam")
    bumper_0 = object_model.get_part("bumper_0")
    bumper_1 = object_model.get_part("bumper_1")
    pivot = object_model.get_articulation("beam_pivot")
    bumper_slide_0 = object_model.get_articulation("bumper_0_slide")
    bumper_slide_1 = object_model.get_articulation("bumper_1_slide")

    # --- Pivot sleeve / axle overlap allowance ---
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

    # --- Beam bar clears the arch saddle ---
    ctx.expect_gap(
        beam,
        base,
        axis="z",
        positive_elem="beam_bar",
        negative_elem="arch_0",
        min_gap=0.005,
        max_gap=0.12,
        name="beam bar clears the arch saddle",
    )

    # --- Joint configuration ---
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

    # --- Curved beam: ends are raised above center ---
    bar_box = ctx.part_element_world_aabb(beam, elem="beam_bar")
    ctx.check(
        "curved beam bar exists and spans about 3.0 m",
        bar_box is not None and abs((bar_box[1][0] - bar_box[0][0]) - 3.0) < 0.05,
        details=f"bar aabb={bar_box}",
    )
    # The beam ends should be visibly higher than the beam center.
    # Check that the beam Z extent is taller than just BEAM_T (straight bar).
    ctx.check(
        "curved beam has raised ends (Z span exceeds bar thickness)",
        bar_box is not None
        and (bar_box[1][2] - bar_box[0][2]) > BEAM_T + CURVE_RISE * 0.6,
        details=f"bar aabb={bar_box}",
    )

    # --- Ground pads exist under the arch feet ---
    base_box = ctx.part_world_aabb(base)
    ctx.check(
        "base with ground pads rests near the ground",
        base_box is not None and -0.005 <= base_box[0][2] <= 0.02,
        details=f"base aabb={base_box}",
    )
    # Check that ground pads exist as named visuals
    pad_names = [v.name for v in base.visuals if v.name and v.name.startswith("ground_pad_")]
    ctx.check(
        "four rubber ground pads exist on the base",
        len(pad_names) == 4,
        details=f"pads={pad_names}",
    )

    # --- Pivot axle height ---
    axle_box = ctx.part_element_world_aabb(base, elem="pivot_axle")
    ctx.check(
        "pivot axle sits about 0.8 m high",
        axle_box is not None and 0.70 <= (axle_box[0][2] + axle_box[1][2]) / 2.0 <= 0.84,
        details=f"axle aabb={axle_box}",
    )

    # --- Per-end fittings: seat and handle ---
    for i in range(2):
        seat = ctx.part_element_world_aabb(beam, elem=f"seat_{i}")
        handle = ctx.part_element_world_aabb(beam, elem=f"handle_{i}")
        ctx.check(
            f"seat_{i} is seated on the curved beam top",
            seat is not None
            and bar_box is not None
            and seat[0][2] > bar_box[0][2]
            and seat[1][2] > seat[0][2],
            details=f"seat aabb={seat}",
        )
        ctx.check(
            f"handle_{i} stands above the beam",
            handle is not None
            and bar_box is not None
            and handle[1][2] > bar_box[1][2] + 0.10,
            details=f"handle aabb={handle}",
        )

    # --- Bumper prismatic joints ---
    for j_idx, (slide_joint, bumper_part) in enumerate(
        [(bumper_slide_0, bumper_0), (bumper_slide_1, bumper_1)]
    ):
        j_type = slide_joint.articulation_type
        ctx.check(
            f"bumper_{j_idx}_slide is a prismatic joint",
            str(j_type) == "ArticulationType.PRISMATIC" or j_type == ArticulationType.PRISMATIC,
            details=f"type={j_type}",
        )
        j_ax = slide_joint.axis
        ctx.check(
            f"bumper_{j_idx}_slide axis is vertical (Z)",
            abs(j_ax[0]) < 1e-9 and abs(j_ax[1]) < 1e-9 and abs(j_ax[2] - 1.0) < 1e-9,
            details=f"axis={j_ax}",
        )
        j_lim = slide_joint.motion_limits
        ctx.check(
            f"bumper_{j_idx}_slide has compression travel",
            j_lim is not None
            and j_lim.lower is not None
            and j_lim.upper is not None
            and abs(j_lim.lower) < 1e-6
            and abs(j_lim.upper - BUMPER_TRAVEL) < 1e-6,
            details=f"limits=({j_lim.lower}, {j_lim.upper})",
        )

    # Bumper pads are bolted to the underside of the curved beam ends.
    # They intentionally overlap at the mounting interface.
    for i in range(2):
        ctx.allow_overlap(
            beam,
            object_model.get_part(f"bumper_{i}"),
            elem_a="beam_bar",
            elem_b=f"bumper_pad_{i}",
            reason=f"Bumper {i} is bolted against the beam underside with small seated overlap.",
        )
        ctx.expect_contact(
            beam,
            object_model.get_part(f"bumper_{i}"),
            elem_a="beam_bar",
            elem_b=f"bumper_pad_{i}",
            name=f"bumper_{i} is seated against the beam underside",
        )
        bumper_box = ctx.part_world_aabb(object_model.get_part(f"bumper_{i}"))
        ctx.check(
            f"bumper_{i} is near the beam end",
            bumper_box is not None
            and min(abs(bumper_box[0][0]), abs(bumper_box[1][0])) > 1.2
            and bumper_box[0][2] > 0.40,
            details=f"bumper aabb={bumper_box}",
        )

    # --- Bumper compression: positive q raises the bumper ---
    rest_b0 = ctx.part_world_aabb(bumper_0)
    with ctx.pose({bumper_slide_0: BUMPER_TRAVEL}):
        compressed_b0 = ctx.part_world_aabb(bumper_0)
        ctx.check(
            "bumper_0 compresses upward when prismatic q is positive",
            rest_b0 is not None
            and compressed_b0 is not None
            and compressed_b0[0][2] > rest_b0[0][2] + 0.010,
            details=f"rest={rest_b0}, compressed={compressed_b0}",
        )

    # --- Decisive rocking pose checks ---
    rest_b0_bumper = ctx.part_world_aabb(bumper_0)
    with ctx.pose({pivot: TILT}):
        down_b0 = ctx.part_world_aabb(bumper_0)
        up_b1 = ctx.part_world_aabb(bumper_1)
        ctx.check(
            "positive rock lowers the +X end near the ground",
            rest_b0_bumper is not None
            and down_b0 is not None
            and down_b0[0][2] < rest_b0_bumper[0][2] - 0.30
            and down_b0[0][2] > -0.05,
            details=f"rest={rest_b0_bumper}, tilted={down_b0}",
        )
        ctx.check(
            "positive rock raises the -X end",
            up_b1 is not None and up_b1[0][2] > 0.9,
            details=f"raised bumper aabb={up_b1}",
        )
    with ctx.pose({pivot: -TILT}):
        down_b1 = ctx.part_world_aabb(bumper_1)
        ctx.check(
            "negative rock lowers the -X end near the ground",
            down_b1 is not None and down_b1[0][2] < 0.5,
            details=f"tilted bumper aabb={down_b1}",
        )

    return ctx.report()


object_model = build_object_model()
