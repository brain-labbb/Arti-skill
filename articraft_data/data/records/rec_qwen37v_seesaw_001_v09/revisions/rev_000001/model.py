from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Vintage playground seesaw – variant 09
#
# World frame: beam runs along X, pivot axis along Y, Z up.
# - Central A-frame support: four bent-tube legs (two per side) converging at
#   the apex, upper crossbar, foot bars, and visible axle bracket plates.
# - Rocking beam: 3.0 m mustard-yellow bar (80×40 mm) with pivot sleeve,
#   gusset plate, molded seats with raised lips, grab handles, and rubber
#   tire-section bumpers.
# - Locking pin: cylindrical pin on the +Y bracket that slides along Y to
#   engage or retract from the beam pivot area.
# - Articulation 1: beam_pivot (REVOLUTE, axis Y, ±20°).
# - Articulation 2: lock_slide (PRISMATIC, axis Y, 0–60 mm travel).
# ---------------------------------------------------------------------------

BEAM_HALF = 1.50  # 3.0 m beam
BEAM_W = 0.08
BEAM_T = 0.04
PIVOT_Z = 0.76  # axle height

# A-frame dimensions
FRAME_FOOT_Y = 0.34  # half-width of base feet
FRAME_APEX_Y = 0.050  # half-width at apex (bracket spacing)
FOOT_X = 0.48  # half-spread of feet along beam direction
TUBE_R = 0.025  # ~50 mm diameter tube
FOOT_Z = 0.025  # foot pad height

# Axle
AXLE_R = 0.016
BRACKET_T = 0.006  # bracket plate thickness
AXLE_LEN = 2.0 * FRAME_APEX_Y + 2.0 * BRACKET_T + 0.02  # spans + protrudes

# Beam-local frame: origin at the axle center.
BAR_BOT = 0.050
BAR_CTR = BAR_BOT + BEAM_T / 2.0
BAR_TOP = BAR_BOT + BEAM_T

SEAT_X = 1.30
HANDLE_X = 1.04
BUMPER_X = 1.42
TILT = math.radians(20.0)

# Locking pin
PIN_Z_OFFSET = -0.030  # below axle center
PIN_R = 0.005
PIN_TRAVEL = 0.058
PIN_Y_ORIG = FRAME_APEX_Y + 0.032  # outer face, clear of leg tubes

# Molded seat
SEAT_W = 0.26
SEAT_D = 0.22
SEAT_BASE_T = 0.014
SEAT_LIP_H = 0.026
SEAT_LIP_T = 0.016


# ────────────────────────────────────── geometry helpers ──────────────────


def _aleg_points(foot_x: float, side: float) -> list[tuple[float, float, float]]:
    """Centerline of one A-frame leg (foot to apex)."""
    mx = foot_x * 0.5
    my = side * (FRAME_FOOT_Y + FRAME_APEX_Y) * 0.5
    mz = (FOOT_Z + PIVOT_Z) * 0.5 + 0.01
    return [
        (foot_x, side * FRAME_FOOT_Y, FOOT_Z),
        (mx, my, mz),
        (0.0, side * FRAME_APEX_Y, PIVOT_Z),
    ]


def _handle_points(x: float) -> list[tuple[float, float, float]]:
    """Inverted-U grab-handle rod centerline, plane across the beam (YZ)."""
    half_w = 0.035
    leg_bot = BAR_TOP - 0.010
    arc_z = 0.275
    pts: list[tuple[float, float, float]] = [
        (x, -half_w, leg_bot),
        (x, -half_w, 0.190),
    ]
    for k in range(7):
        a = math.pi * k / 6.0
        pts.append((x, -half_w * math.cos(a), arc_z + half_w * math.sin(a)))
    pts.append((x, half_w, 0.190))
    pts.append((x, half_w, leg_bot))
    return pts


def _bumper_geometry(x: float, index: int):
    """Curved tire-section bumper: half-annulus shell extruded across beam."""
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
    geom.translate(x, 0.0, BAR_BOT + 0.002)
    return mesh_from_geometry(geom, f"seesaw_bumper_{index}")


def _gusset_geometry():
    """Triangular gusset plate joining the beam bar to the pivot sleeve."""
    profile = [(-0.10, 0.055), (0.10, 0.055), (0.0, 0.020)]
    geom = ExtrudeGeometry(profile, 0.020, cap=True, center=True)
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, "seesaw_gusset_plate")


def _molded_seat_cq():
    """Molded plastic seat pan with raised perimeter lips (CadQuery).

    Bottom at z = 0 so the visual origin places it on the beam bar top.
    """
    base_t = SEAT_BASE_T
    lip_h = SEAT_LIP_H
    lip_t = SEAT_LIP_T
    sw = SEAT_W
    sd = SEAT_D

    base = cq.Workplane("XY").box(sw, sd, base_t).translate((0, 0, base_t / 2))
    z_lip = base_t + lip_h / 2
    front = (
        cq.Workplane("XY")
        .box(sw, lip_t, lip_h)
        .translate((0, sd / 2 - lip_t / 2, z_lip))
    )
    back = (
        cq.Workplane("XY")
        .box(sw, lip_t, lip_h)
        .translate((0, -(sd / 2 - lip_t / 2), z_lip))
    )
    left = (
        cq.Workplane("XY")
        .box(lip_t, sd - 2 * lip_t, lip_h)
        .translate((sw / 2 - lip_t / 2, 0, z_lip))
    )
    right = (
        cq.Workplane("XY")
        .box(lip_t, sd - 2 * lip_t, lip_h)
        .translate((-(sw / 2 - lip_t / 2), 0, z_lip))
    )
    seat = base.union(front).union(back).union(left).union(right)
    # Fillet the outer vertical edges for a molded look.
    seat = seat.edges("|Z and (not <Z)").fillet(0.006)
    return seat


def _bracket_plate_cq(y_pos: float, name: str):
    """Flat steel bracket plate at the A-frame apex (CadQuery).

    A vertical plate in the XZ plane with an axle bore at PIVOT_Z and a
    smaller locking-pin bore below.
    """
    plate_w = 0.080
    plate_h = 0.100
    plate = (
        cq.Workplane("XY")
        .box(plate_w, BRACKET_T, plate_h)
        .translate((0, y_pos, PIVOT_Z))
    )
    # Axle bore (centered at PIVOT_Z).
    axle_bore = (
        cq.Workplane("XZ")
        .center(0, PIVOT_Z)
        .circle(AXLE_R + 0.002)
        .extrude(BRACKET_T)
        .translate((0, y_pos - BRACKET_T / 2, 0))
    )
    # Locking pin bore (below axle, only on +Y bracket).
    pin_bore = (
        cq.Workplane("XZ")
        .center(0, PIVOT_Z + PIN_Z_OFFSET)
        .circle(PIN_R + 0.001)
        .extrude(BRACKET_T)
        .translate((0, y_pos - BRACKET_T / 2, 0))
    )
    plate = plate.cut(axle_bore).cut(pin_bore)
    return plate


# ────────────────────────────────────── build ─────────────────────────────


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_playground_seesaw")

    galvanized = model.material("weathered_galvanized", rgba=(0.55, 0.58, 0.56, 1.0))
    rust = model.material("rust_steel", rgba=(0.42, 0.25, 0.13, 1.0))
    mustard = model.material("rusty_mustard_paint", rgba=(0.74, 0.53, 0.12, 1.0))
    pale_steel = model.material("pale_weathered_steel", rgba=(0.70, 0.66, 0.58, 1.0))
    bracket_steel = model.material("dark_bracket_steel", rgba=(0.35, 0.34, 0.32, 1.0))
    seat_plastic = model.material("molded_seat_green", rgba=(0.16, 0.32, 0.18, 1.0))
    rubber = model.material("black_rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    pin_steel = model.material("zinc_pin", rgba=(0.60, 0.62, 0.60, 1.0))

    # ───────────────────────────── A-frame base ────────────────────────────
    base = model.part("a_frame_base")

    # Four tube legs: two per side forming inverted-V when viewed from end.
    leg_idx = 0
    for side in (1.0, -1.0):
        for foot_x in (FOOT_X, -FOOT_X):
            base.visual(
                mesh_from_geometry(
                    tube_from_spline_points(
                        _aleg_points(foot_x, side),
                        radius=TUBE_R,
                        samples_per_segment=8,
                        radial_segments=18,
                        cap_ends=True,
                    ),
                    f"aframe_leg_{leg_idx}",
                ),
                material=galvanized,
                name=f"leg_{leg_idx}",
            )
            leg_idx += 1

    # Upper crossbar connecting the two apexes, routed below the pivot line
    # so it does not collide with the pivot sleeve.
    cross_z = PIVOT_Z - 0.065
    base.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [(0.0, -FRAME_APEX_Y, cross_z), (0.0, FRAME_APEX_Y, cross_z)],
                radius=TUBE_R * 0.8,
                samples_per_segment=4,
                radial_segments=16,
                cap_ends=True,
            ),
            "aframe_crossbar_upper",
        ),
        material=galvanized,
        name="crossbar_upper",
    )

    # Foot bars connecting left/right feet at ground level.
    for foot_x in (FOOT_X, -FOOT_X):
        base.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    [
                        (foot_x, -FRAME_FOOT_Y, FOOT_Z),
                        (foot_x, FRAME_FOOT_Y, FOOT_Z),
                    ],
                    radius=TUBE_R * 0.8,
                    samples_per_segment=4,
                    radial_segments=16,
                    cap_ends=True,
                ),
                f"aframe_foot_bar_{foot_x:.0f}",
            ),
            material=galvanized,
            name=f"foot_bar_{'front' if foot_x > 0 else 'rear'}",
        )

    # Axle bracket plates (CadQuery with bore holes).
    for i, side in enumerate((1.0, -1.0)):
        y_pos = side * FRAME_APEX_Y
        base.visual(
            mesh_from_cadquery(
                _bracket_plate_cq(y_pos, f"bracket_{i}"),
                f"bracket_plate_{i}",
            ),
            material=bracket_steel,
            name=f"bracket_{i}",
        )

    # Small gusset triangles reinforcing each bracket-to-leg junction.
    for i, side in enumerate((1.0, -1.0)):
        y_pos = side * FRAME_APEX_Y
        for dx in (-0.025, 0.025):
            base.visual(
                Box((0.012, BRACKET_T, 0.050)),
                origin=Origin(xyz=(dx, y_pos, PIVOT_Z - 0.045)),
                material=bracket_steel,
                name=f"bracket_gusset_{i}_{'f' if dx > 0 else 'r'}",
            )

    # Pivot axle bolt through both bracket plates, axis along Y.
    base.visual(
        Cylinder(radius=AXLE_R, length=AXLE_LEN),
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rust,
        name="pivot_axle",
    )
    for i, side in enumerate((1.0, -1.0)):
        base.visual(
            Cylinder(radius=0.022, length=0.012),
            origin=Origin(
                xyz=(0.0, side * (AXLE_LEN / 2.0 - 0.004), PIVOT_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=rust,
            name=f"axle_nut_{i}",
        )

    # ───────────────────────────── beam ────────────────────────────────────
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

    # Rust streak patches.
    for i, px in enumerate((-0.85, -0.30, 0.45, 0.95)):
        beam.visual(
            Box((0.16, BEAM_W + 0.004, 0.012)),
            origin=Origin(xyz=(px, 0.0, BAR_TOP - 0.004)),
            material=rust,
            name=f"rust_patch_{i}",
        )

    # Molded seats with raised lips (CadQuery), handles, and bumpers.
    seat_mesh = mesh_from_cadquery(_molded_seat_cq(), "molded_seat")
    for i, side in enumerate((1.0, -1.0)):
        beam.visual(
            seat_mesh,
            origin=Origin(xyz=(side * SEAT_X, 0.0, BAR_TOP - 0.004)),
            material=seat_plastic,
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

    # ───────────────────────────── locking pin ─────────────────────────────
    locking_pin = model.part("locking_pin")

    # Pin shaft: cylinder along Y, extends inward (-Y) from just outside
    # the grip through the bracket hole.
    shaft_len = 0.044
    locking_pin.visual(
        Cylinder(radius=PIN_R, length=shaft_len),
        origin=Origin(
            xyz=(0.0, -shaft_len / 2.0 + 0.006, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=pin_steel,
        name="pin_shaft",
    )
    # Grip knob at the outer end.
    locking_pin.visual(
        Sphere(radius=0.011),
        origin=Origin(xyz=(0.0, 0.016, 0.0)),
        material=pin_steel,
        name="pin_grip",
    )
    # Small retaining ring collar near the grip.
    locking_pin.visual(
        Cylinder(radius=PIN_R + 0.003, length=0.005),
        origin=Origin(
            xyz=(0.0, 0.008, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=bracket_steel,
        name="pin_collar",
    )

    # ───────────────────────────── joints ──────────────────────────────────
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
        "lock_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=locking_pin,
        origin=Origin(xyz=(0.0, PIN_Y_ORIG, PIVOT_Z + PIN_Z_OFFSET)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=0.5, lower=0.0, upper=PIN_TRAVEL
        ),
    )

    return model


# ────────────────────────────────────── tests ─────────────────────────────


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("a_frame_base")
    beam = object_model.get_part("beam")
    locking_pin = object_model.get_part("locking_pin")
    pivot = object_model.get_articulation("beam_pivot")
    lock = object_model.get_articulation("lock_slide")

    # ── pivot sleeve captures the axle bolt ──
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

    # ── locking pin shaft passes through bracket bore and near apex legs ──
    for leg_name in ("leg_0", "leg_1", "leg_2", "leg_3"):
        ctx.allow_overlap(
            base,
            locking_pin,
            elem_a=leg_name,
            elem_b="pin_shaft",
            reason="Locking pin shaft passes through the bracket bore hole; small local overlap with apex leg weld zone is intentional.",
        )
    for brk_name in ("bracket_0", "bracket_1"):
        ctx.allow_overlap(
            base,
            locking_pin,
            elem_a=brk_name,
            elem_b="pin_shaft",
            reason="Locking pin shaft passes through the bore hole cut into the bracket plate.",
        )
    # Proof: at retracted pose the pin moves outward in +Y.
    pin_rest = ctx.part_element_world_aabb(locking_pin, elem="pin_shaft")
    with ctx.pose({lock: PIN_TRAVEL}):
        pin_out = ctx.part_element_world_aabb(locking_pin, elem="pin_shaft")
        ctx.check(
            "retracted pin moves outward in +Y",
            pin_rest is not None
            and pin_out is not None
            and pin_out[1][1] > pin_rest[1][1] + PIN_TRAVEL * 0.8,
            details=f"rest={pin_rest}, retracted={pin_out}",
        )

    # ── beam bar clears the A-frame ──
    ctx.expect_gap(
        beam,
        base,
        axis="z",
        positive_elem="beam_bar",
        negative_elem="leg_0",
        min_gap=0.005,
        max_gap=0.08,
        name="beam bar clears the A-frame legs",
    )

    # ── pivot joint configuration ──
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

    # ── locking pin joint configuration ──
    lock_ax = lock.axis
    ctx.check(
        "locking pin slides along Y axis",
        abs(lock_ax[0]) < 1e-9 and abs(lock_ax[1] - 1.0) < 1e-9 and abs(lock_ax[2]) < 1e-9,
        details=f"lock axis={lock_ax}",
    )
    lock_lim = lock.motion_limits
    ctx.check(
        "locking pin has prismatic travel 0 to ~58 mm",
        lock_lim is not None
        and lock_lim.lower is not None
        and lock_lim.upper is not None
        and abs(lock_lim.lower) < 1e-6
        and 0.04 <= lock_lim.upper <= 0.08,
        details=f"lock limits=({lock_lim.lower}, {lock_lim.upper})",
    )

    # ── A-frame geometry checks ──
    base_box = ctx.part_world_aabb(base)
    axle_box = ctx.part_element_world_aabb(base, elem="pivot_axle")
    ctx.check(
        "A-frame base feet rest on the ground",
        base_box is not None and -0.01 <= base_box[0][2] <= 0.04,
        details=f"base aabb={base_box}",
    )
    ctx.check(
        "pivot axle sits about 0.8 m high",
        axle_box is not None and 0.70 <= (axle_box[0][2] + axle_box[1][2]) / 2.0 <= 0.82,
        details=f"axle aabb={axle_box}",
    )

    # Visible axle bracket plates exist and are near the apex.
    for i in range(2):
        brk = ctx.part_element_world_aabb(base, elem=f"bracket_{i}")
        ctx.check(
            f"bracket_{i} is a visible plate near the apex",
            brk is not None
            and abs((brk[0][2] + brk[1][2]) / 2.0 - PIVOT_Z) < 0.06
            and (brk[1][2] - brk[0][2]) > 0.06,
            details=f"bracket aabb={brk}",
        )

    # ── beam hero geometry ──
    bar_box = ctx.part_element_world_aabb(beam, elem="beam_bar")
    ctx.check(
        "beam is about 3.0 m long",
        bar_box is not None and abs((bar_box[1][0] - bar_box[0][0]) - 3.0) < 0.02,
        details=f"bar aabb={bar_box}",
    )

    # ── molded seats with raised lips ──
    for i in range(2):
        seat = ctx.part_element_world_aabb(beam, elem=f"seat_{i}")
        ctx.check(
            f"seat_{i} sits on the beam bar top",
            seat is not None
            and bar_box is not None
            and bar_box[0][2] < seat[0][2] < bar_box[1][2]
            and seat[1][2] > bar_box[1][2],
            details=f"seat aabb={seat}",
        )
        seat_height = seat[1][2] - seat[0][2] if seat else 0.0
        ctx.check(
            f"seat_{i} has raised lips (total height > base thickness)",
            seat is not None and seat_height > SEAT_BASE_T + SEAT_LIP_H * 0.5,
            details=f"seat height={seat_height:.4f}",
        )

    # ── handles and bumpers ──
    for i in range(2):
        handle = ctx.part_element_world_aabb(beam, elem=f"handle_{i}")
        bumper = ctx.part_element_world_aabb(beam, elem=f"bumper_{i}")
        ctx.check(
            f"handle_{i} stands about 0.25 m above the beam",
            handle is not None
            and bar_box is not None
            and handle[1][2] > bar_box[1][2] + 0.18
            and handle[0][2] < bar_box[1][2],
            details=f"handle aabb={handle}",
        )
        ctx.check(
            f"bumper_{i} hangs below the beam tip",
            bumper is not None
            and bar_box is not None
            and bumper[0][2] < bar_box[0][2]
            and min(abs(bumper[0][0]), abs(bumper[1][0])) > 1.3,
            details=f"bumper aabb={bumper}",
        )

    # ── locking pin position checks ──
    pin_box_rest = ctx.part_element_world_aabb(locking_pin, elem="pin_shaft")
    ctx.check(
        "locking pin shaft exists near the bracket",
        pin_box_rest is not None
        and abs((pin_box_rest[0][2] + pin_box_rest[1][2]) / 2.0 - (PIVOT_Z + PIN_Z_OFFSET)) < 0.02,
        details=f"pin aabb={pin_box_rest}",
    )

    # ── decisive rocking pose checks ──
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
            down_b1 is not None and 0.0 < down_b1[0][2] < 0.32,
            details=f"tilted bumper aabb={down_b1}",
        )

    return ctx.report()


object_model = build_object_model()
