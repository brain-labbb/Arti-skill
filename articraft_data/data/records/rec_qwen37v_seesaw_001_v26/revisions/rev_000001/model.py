from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Variant 26 – curved-beam playground seesaw with locking pin
#
# World frame: beam runs along X, pivot axis along Y, Z up.
# Structural changes vs parent:
#   1. Curved beam with raised ends (parabolic loft, ~120 mm rise at tips)
#   2. Locking pin that slides (PRISMATIC) near the central bracket
#   3. Rubber ground pads under the four arch feet
#   4. Textured footrests near each seat
# ---------------------------------------------------------------------------

BEAM_HALF = 1.50  # 3.0 m beam
BEAM_W = 0.08
BEAM_T = 0.04
PIVOT_Z = 0.76
CURVE_RISE = 0.12  # ends rise 120 mm above center

ARCH_FOOT_X = 0.66
ARCH_FOOT_Y = 0.34
ARCH_APEX_Y = 0.05
ARCH_FOOT_Z = 0.028
TUBE_R = 0.025

AXLE_R = 0.016
AXLE_LEN = 0.22

BAR_BOT = 0.05
BAR_CTR = BAR_BOT + BEAM_T / 2.0  # 0.07
BAR_TOP = BAR_BOT + BEAM_T  # 0.09

SEAT_X = 1.30
HANDLE_X = 1.04
BUMPER_X = 1.42
FOOTREST_X = 1.15
TILT = math.radians(20.0)

# Ground-pad dimensions
PAD_W = 0.12
PAD_D = 0.12
PAD_T = 0.008

# Footrest dimensions
FR_W = 0.18  # along beam (X)
FR_D = 0.16  # across beam (Y)
FR_T = 0.008
RIDGE_N = 4
RIDGE_W = FR_W - 0.02
RIDGE_H = 0.005
RIDGE_T = 0.006

# Locking-pin dimensions (vertical drop pin through the pivot bracket)
PIN_R = 0.006
PIN_LEN = 0.10
PIN_HEAD_R = 0.012
PIN_HEAD_LEN = 0.010
PIN_Z_LOCAL = -0.020  # beam-local z of pin origin (below beam center)
PIN_TRAVEL = 0.08


def _curve_z(x: float) -> float:
    """Parabolic rise of the beam centerline at position x."""
    t = x / BEAM_HALF
    return CURVE_RISE * t * t


def _arch_points(side: float) -> list[tuple[float, float, float]]:
    """Centerline of one bent-tube arch (same as parent)."""
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
    """Inverted-U grab-handle rod centerline, adjusted for curved beam."""
    half_w = 0.035
    z_off = _curve_z(x)
    leg_bot = BAR_TOP + z_off - 0.010
    arc_z = 0.275 + z_off
    pts: list[tuple[float, float, float]] = [
        (x, -half_w, leg_bot),
        (x, -half_w, 0.190 + z_off),
    ]
    for k in range(7):
        a = math.pi * k / 6.0
        pts.append((x, -half_w * math.cos(a), arc_z + half_w * math.sin(a)))
    pts.append((x, half_w, 0.190 + z_off))
    pts.append((x, half_w, leg_bot))
    return pts


def _bumper_geometry(x: float, index: int):
    """Curved tire-section bumper, adjusted for curved beam height."""
    r_out = 0.065
    r_in = 0.048
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
    z_off = _curve_z(x)
    geom.translate(x, 0.0, BAR_BOT + z_off + 0.002)
    return mesh_from_geometry(geom, f"seesaw_bumper_{index}")


def _gusset_geometry():
    """Triangular gusset plate joining the beam bar to the pivot sleeve."""
    profile = [(-0.10, 0.055), (0.10, 0.055), (0.0, 0.020)]
    geom = ExtrudeGeometry(profile, 0.020, cap=True, center=True)
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, "seesaw_gusset_plate")


def _curved_beam_geometry():
    """Curved beam loft: rectangular sections along a parabolic path.

    LoftGeometry requires profiles in the XY plane at different Z heights.
    We author in a temp frame where Z = beam length, X = -beam_Z, Y = beam_Y,
    then rotate_y(pi/2) to align the beam along world X.
    """
    n_sections = 13
    hw = BEAM_W / 2.0
    ht = BEAM_T / 2.0
    profiles: list[list[tuple[float, float, float]]] = []
    for i in range(n_sections):
        t = -1.0 + 2.0 * i / (n_sections - 1)
        beam_x = BEAM_HALF * t
        z_rise = CURVE_RISE * t * t
        cz = BAR_CTR + z_rise  # beam-local Z center at this section
        # Map: loft_x = -beam_z, loft_y = beam_y, loft_z = beam_x
        profiles.append([
            (-(cz + ht), -hw, beam_x),
            (-(cz + ht), hw, beam_x),
            (-(cz - ht), hw, beam_x),
            (-(cz - ht), -hw, beam_x),
        ])
    geom = LoftGeometry(profiles, cap=True, closed=True)
    # Rotate so loft Z → beam X: rotate_y(+pi/2) maps (x,y,z)→(z,y,-x)
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, "curved_beam_bar")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_playground_seesaw_v26")

    galvanized = model.material("weathered_galvanized", rgba=(0.55, 0.58, 0.56, 1.0))
    rust = model.material("rust_steel", rgba=(0.42, 0.25, 0.13, 1.0))
    mustard = model.material("rusty_mustard_paint", rgba=(0.74, 0.53, 0.12, 1.0))
    pale_steel = model.material("pale_weathered_steel", rgba=(0.70, 0.66, 0.58, 1.0))
    wood = model.material("worn_wood", rgba=(0.60, 0.45, 0.28, 1.0))
    rubber = model.material("black_rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.12, 0.12, 0.10, 1.0))
    pin_steel = model.material("pin_steel", rgba=(0.50, 0.48, 0.44, 1.0))

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

    # Pivot axle bolt
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

    # --- Rubber ground pads under the four arch feet ---
    foot_positions = [
        (ARCH_FOOT_X, ARCH_FOOT_Y),
        (-ARCH_FOOT_X, ARCH_FOOT_Y),
        (ARCH_FOOT_X, -ARCH_FOOT_Y),
        (-ARCH_FOOT_X, -ARCH_FOOT_Y),
    ]
    for i, (fx, fy) in enumerate(foot_positions):
        base.visual(
            Box((PAD_W, PAD_D, PAD_T)),
            origin=Origin(xyz=(fx, fy, PAD_T / 2.0)),
            material=rubber,
            name=f"ground_pad_{i}",
        )

    # --------------------------------------------------------------- beam ---
    beam = model.part("beam")

    beam.visual(
        Cylinder(radius=0.026, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rust,
        name="pivot_sleeve",
    )
    beam.visual(_gusset_geometry(), material=mustard, name="gusset_plate")

    # Curved beam bar (replaces straight Box)
    beam.visual(_curved_beam_geometry(), material=mustard, name="beam_bar")

    # Pin guide bracket on the gusset (small sleeve for the locking pin)
    beam.visual(
        Cylinder(radius=0.014, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, PIN_Z_LOCAL)),
        material=rust,
        name="pin_bracket",
    )

    # Rust streak patches (cosmetic weathering on the curved beam)
    for i, px in enumerate((-0.85, -0.30, 0.45, 0.95)):
        z_off = _curve_z(px)
        beam.visual(
            Box((0.16, BEAM_W + 0.004, 0.012)),
            origin=Origin(xyz=(px, 0.0, BAR_TOP + z_off - 0.004)),
            material=rust,
            name=f"rust_patch_{i}",
        )

    # End fittings: seat, handle, bumper, footrest per side
    for i, side in enumerate((1.0, -1.0)):
        sx = side * SEAT_X
        z_seat = _curve_z(sx)
        beam.visual(
            Box((0.30, 0.24, 0.022)),
            origin=Origin(xyz=(sx, 0.0, BAR_TOP + z_seat + 0.008)),
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
        beam.visual(
            _bumper_geometry(side * BUMPER_X, i),
            material=rubber,
            name=f"bumper_{i}",
        )

        # --- Textured footrest near each seat ---
        frx = side * FOOTREST_X
        z_fr = _curve_z(frx)
        fr_z_center = BAR_TOP + z_fr + FR_T / 2.0
        beam.visual(
            Box((FR_W, FR_D, FR_T)),
            origin=Origin(xyz=(frx, 0.0, fr_z_center)),
            material=grip_rubber,
            name=f"footrest_{i}",
        )
        # Grip ridges on top of each footrest
        for r in range(RIDGE_N):
            ry = -FR_D / 2.0 + FR_D * (r + 0.5) / RIDGE_N
            beam.visual(
                Box((RIDGE_W, RIDGE_T, RIDGE_H)),
                origin=Origin(
                    xyz=(frx, ry, fr_z_center + FR_T / 2.0 + RIDGE_H / 2.0)
                ),
                material=grip_rubber,
                name=f"footrest_ridge_{i}_{r}",
            )

    # -------------------------------------------------------- locking pin ---
    locking_pin = model.part("locking_pin")
    locking_pin.visual(
        Cylinder(radius=PIN_R, length=PIN_LEN),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=pin_steel,
        name="pin_rod",
    )
    locking_pin.visual(
        Cylinder(radius=PIN_HEAD_R, length=PIN_HEAD_LEN),
        origin=Origin(xyz=(0.0, 0.0, PIN_LEN / 2.0 + PIN_HEAD_LEN / 2.0)),
        material=rust,
        name="pin_head",
    )

    # -------------------------------------------------------------- joints ---
    model.articulation(
        "beam_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=200.0, velocity=2.5, lower=-TILT, upper=TILT
        ),
    )

    model.articulation(
        "pin_slide",
        ArticulationType.PRISMATIC,
        parent=beam,
        child=locking_pin,
        origin=Origin(xyz=(0.0, 0.0, PIN_Z_LOCAL)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=0.5, lower=0.0, upper=PIN_TRAVEL
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("arched_base")
    beam = object_model.get_part("beam")
    locking_pin = object_model.get_part("locking_pin")
    pivot = object_model.get_articulation("beam_pivot")
    pin_joint = object_model.get_articulation("pin_slide")

    # ---- pivot sleeve / axle overlap (carried from parent) ----
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

    # ---- pin bracket overlaps pin rod (intentional capture) ----
    ctx.allow_overlap(
        beam,
        locking_pin,
        elem_a="pin_bracket",
        elem_b="pin_rod",
        reason="Pin rod slides through the guide bracket; small intentional capture overlap.",
    )
    # Pin passes through the gusset plate vertically
    ctx.allow_overlap(
        beam,
        locking_pin,
        elem_a="gusset_plate",
        elem_b="pin_rod",
        reason="Vertical locking pin passes through a hole in the gusset plate.",
    )
    # Pin head sits within the gusset zone when locked
    ctx.allow_overlap(
        beam,
        locking_pin,
        elem_a="gusset_plate",
        elem_b="pin_head",
        reason="Pin head nests near the gusset top when the pin is in the locked (down) position.",
    )
    # Pin passes through the pivot sleeve (drop-pin locks the pivot)
    ctx.allow_overlap(
        beam,
        locking_pin,
        elem_a="pivot_sleeve",
        elem_b="pin_rod",
        reason="Locking pin drops through the pivot sleeve to engage the lock.",
    )
    # Pin bracket and axle share the pivot zone
    ctx.allow_overlap(
        base,
        beam,
        elem_a="pivot_axle",
        elem_b="pin_bracket",
        reason="Pin guide bracket is mounted at the pivot axle zone on the beam.",
    )
    # Pin rod passes through the base axle when locked
    ctx.allow_overlap(
        base,
        locking_pin,
        elem_a="pivot_axle",
        elem_b="pin_rod",
        reason="Locking pin passes through a cross-hole in the axle when engaged.",
    )

    # ---- ground pads overlap arch feet (rubber compression) ----
    for i in range(4):
        ctx.allow_overlap(
            base,
            base,
            elem_a=f"ground_pad_{i}",
            elem_b=f"arch_{i // 2}",
            reason="Rubber ground pad compressed under the steel arch tube foot.",
        )

    # ---- curved beam: raised ends ----
    bar_box = ctx.part_element_world_aabb(beam, elem="beam_bar")
    ctx.check(
        "curved beam has raised ends (top above straight-beam height)",
        bar_box is not None
        and bar_box[1][2] > PIVOT_Z + BAR_TOP + CURVE_RISE * 0.5,
        details=f"beam_bar aabb top={bar_box[1][2] if bar_box else None},"
        f" expected > {PIVOT_Z + BAR_TOP + CURVE_RISE * 0.5:.3f}",
    )
    ctx.check(
        "curved beam spans about 3.0 m",
        bar_box is not None
        and abs((bar_box[1][0] - bar_box[0][0]) - 3.0) < 0.05,
        details=f"beam_bar dx={bar_box[1][0] - bar_box[0][0] if bar_box else None}",
    )

    # ---- ground pads exist and sit at ground level ----
    base_box = ctx.part_world_aabb(base)
    for i in range(4):
        pad_box = ctx.part_element_world_aabb(base, elem=f"ground_pad_{i}")
        ctx.check(
            f"ground_pad_{i} exists near ground level",
            pad_box is not None and pad_box[0][2] < 0.005,
            details=f"pad aabb={pad_box}",
        )
    ctx.check(
        "arched base feet rest on the ground (with pads)",
        base_box is not None and -0.01 <= base_box[0][2] <= 0.02,
        details=f"base aabb={base_box}",
    )

    # ---- footrests near each seat ----
    for i in range(2):
        fr_box = ctx.part_element_world_aabb(beam, elem=f"footrest_{i}")
        seat_box = ctx.part_element_world_aabb(beam, elem=f"seat_{i}")
        ctx.check(
            f"footrest_{i} exists on top of the curved beam near seat_{i}",
            fr_box is not None
            and bar_box is not None
            and fr_box[0][2] > bar_box[0][2] + 0.05
            and fr_box[1][2] > bar_box[0][2] + 0.08,
            details=f"footrest aabb={fr_box}, bar aabb={bar_box}",
        )
        ctx.check(
            f"footrest_{i} is inboard of seat_{i}",
            fr_box is not None
            and seat_box is not None
            and abs(fr_box[0][0] + fr_box[1][0]) / 2.0
            < abs(seat_box[0][0] + seat_box[1][0]) / 2.0,
            details=f"footrest center x={(fr_box[0][0] + fr_box[1][0]) / 2.0 if fr_box else None},"
            f" seat center x={(seat_box[0][0] + seat_box[1][0]) / 2.0 if seat_box else None}",
        )
        # At least one ridge per footrest
        ridge_box = ctx.part_element_world_aabb(beam, elem=f"footrest_ridge_{i}_0")
        ctx.check(
            f"footrest_{i} has textured ridges",
            ridge_box is not None
            and fr_box is not None
            and ridge_box[0][2] >= fr_box[1][2] - 0.002,
            details=f"ridge aabb={ridge_box}",
        )

    # ---- locking pin: prismatic joint ----
    ctx.check(
        "locking pin part exists",
        locking_pin is not None,
    )
    ctx.check(
        "pin_slide is a prismatic joint",
        pin_joint.joint_type == ArticulationType.PRISMATIC,
        details=f"type={pin_joint.joint_type}",
    )
    pin_axis = pin_joint.axis
    ctx.check(
        "pin_slide axis is vertical (Z)",
        abs(pin_axis[0]) < 1e-9
        and abs(pin_axis[1]) < 1e-9
        and abs(pin_axis[2] - 1.0) < 1e-9,
        details=f"axis={pin_axis}",
    )
    pin_lim = pin_joint.motion_limits
    ctx.check(
        "pin_slide has positive travel (0 to ~80 mm)",
        pin_lim is not None
        and pin_lim.lower is not None
        and pin_lim.upper is not None
        and abs(pin_lim.lower) < 1e-6
        and pin_lim.upper > 0.04,
        details=f"limits=({pin_lim.lower}, {pin_lim.upper})",
    )

    # Pin pose: slides along Z when actuated
    pin_rest = ctx.part_world_position(locking_pin)
    with ctx.pose({pin_joint: PIN_TRAVEL}):
        pin_ext = ctx.part_world_position(locking_pin)
    ctx.check(
        "pin slides upward (+Z) when extended",
        pin_rest is not None
        and pin_ext is not None
        and pin_ext[2] > pin_rest[2] + 0.03,
        details=f"rest={pin_rest}, extended={pin_ext}",
    )

    # Proof checks for intentional pin/pivot overlaps
    ctx.expect_within(
        locking_pin,
        base,
        axes="xy",
        inner_elem="pin_rod",
        outer_elem="pivot_axle",
        margin=0.005,
        name="pin rod stays within axle cross-section at the pivot",
    )
    ctx.expect_overlap(
        locking_pin,
        beam,
        axes="z",
        elem_a="pin_rod",
        elem_b="pivot_sleeve",
        min_overlap=0.010,
        name="pin rod overlaps the pivot sleeve zone vertically",
    )
    # When pin is raised (unlocked), pin_head clears above the gusset
    with ctx.pose({pin_joint: PIN_TRAVEL}):
        ctx.expect_gap(
            locking_pin,
            beam,
            axis="z",
            positive_elem="pin_head",
            negative_elem="gusset_plate",
            min_gap=-0.005,
            name="pin head clears or nearly clears the gusset when unlocked",
        )

    # ---- pivot joint (carried from parent) ----
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

    # ---- hero geometry ----
    axle_box = ctx.part_element_world_aabb(base, elem="pivot_axle")
    ctx.check(
        "pivot axle sits about 0.8 m high",
        axle_box is not None and 0.70 <= (axle_box[0][2] + axle_box[1][2]) / 2.0 <= 0.82,
        details=f"axle aabb={axle_box}",
    )

    # Seats, handles, bumpers
    for i in range(2):
        seat = ctx.part_element_world_aabb(beam, elem=f"seat_{i}")
        handle = ctx.part_element_world_aabb(beam, elem=f"handle_{i}")
        bumper = ctx.part_element_world_aabb(beam, elem=f"bumper_{i}")
        ctx.check(
            f"seat_{i} is seated on the curved beam",
            seat is not None
            and bar_box is not None
            and seat[1][2] > bar_box[0][2],
            details=f"seat aabb={seat}",
        )
        ctx.check(
            f"handle_{i} stands above the beam",
            handle is not None
            and bar_box is not None
            and handle[1][2] > bar_box[0][2] + 0.18,
            details=f"handle aabb={handle}",
        )
        ctx.check(
            f"bumper_{i} hangs below the curved beam at the end",
            bumper is not None
            and bar_box is not None
            and bumper[0][2] < bar_box[1][2] - 0.05
            and min(abs(bumper[0][0]), abs(bumper[1][0])) > 1.3,
            details=f"bumper aabb={bumper}",
        )

    # ---- decisive pose: rocking ----
    rest_b0 = ctx.part_element_world_aabb(beam, elem="bumper_0")
    with ctx.pose({pivot: TILT}):
        down_b0 = ctx.part_element_world_aabb(beam, elem="bumper_0")
        up_b1 = ctx.part_element_world_aabb(beam, elem="bumper_1")
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
        down_b1 = ctx.part_element_world_aabb(beam, elem="bumper_1")
        ctx.check(
            "negative rock lowers the -X end near the ground",
            down_b1 is not None and 0.0 < down_b1[0][2] < 0.45,
            details=f"tilted bumper aabb={down_b1}",
        )

    return ctx.report()


object_model = build_object_model()
