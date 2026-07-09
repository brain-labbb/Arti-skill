from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Low inclusive playground seesaw with backrest seats and pivoting handlebars
#
# World frame: beam runs along X, pivot axis along Y, Z up.
# - Two bent galvanized-steel tube arches (~50 mm dia) form an A-shaped saddle;
#   the apex carries a horizontal pivot axle bolt.
# - Lower pivot height (~0.50 m) for inclusive/accessible use.
# - The rocking beam is a 3.0 m mustard-yellow steel bar (80 x 40 mm).
# - Each end has a molded bucket seat with raised lips, a backrest panel,
#   a pivoting handlebar, and a curved rubber bumper underneath.
# - Handlebars are separate parts with revolute joints for slight pivot.
# ---------------------------------------------------------------------------

BEAM_HALF = 1.50
BEAM_W = 0.08
BEAM_T = 0.04
PIVOT_Z = 0.50

ARCH_FOOT_X = 0.50
ARCH_FOOT_Y = 0.30
ARCH_APEX_Y = 0.04
ARCH_FOOT_Z = 0.025
TUBE_R = 0.025

AXLE_R = 0.016
AXLE_LEN = 0.22

BAR_BOT = 0.04
BAR_CTR = BAR_BOT + BEAM_T / 2.0
BAR_TOP = BAR_BOT + BEAM_T  # 0.08

SEAT_X = 1.20
HANDLE_X = 0.95
BUMPER_X = 1.38
TILT = math.radians(20.0)
HANDLE_TILT = math.radians(12.0)


def _arch_points(side: float) -> list[tuple[float, float, float]]:
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


def _handle_rod_points() -> list[tuple[float, float, float]]:
    """Inverted-U grab-handle rod centerline in handlebar-local frame.
    Origin at the mount top surface; rod legs go down into the mount."""
    half_w = 0.04
    pts: list[tuple[float, float, float]] = [
        (0.0, -half_w, -0.005),  # leg base inside mount
        (0.0, -half_w, 0.14),
    ]
    arc_z = 0.20
    for k in range(7):
        a = math.pi * k / 6.0
        pts.append((0.0, -half_w * math.cos(a), arc_z + half_w * math.sin(a)))
    pts.append((0.0, half_w, 0.14))
    pts.append((0.0, half_w, -0.005))
    return pts


def _molded_seat_geometry(index: int):
    """Molded bucket seat: flat base + raised side and front lips."""
    seat_w = 0.24
    seat_d = 0.22
    lip_h = 0.040
    lip_t = 0.014
    base_t = 0.014

    merged = MeshGeometry()

    # Base plate
    base_profile = [
        (-seat_d / 2, -seat_w / 2),
        (seat_d / 2, -seat_w / 2),
        (seat_d / 2, seat_w / 2),
        (-seat_d / 2, seat_w / 2),
    ]
    merged.merge(ExtrudeGeometry(base_profile, base_t, cap=True, center=False))

    # Left lip (-Y edge)
    merged.merge(ExtrudeGeometry(
        [(-seat_d / 2 + lip_t, -seat_w / 2),
         (seat_d / 2 - lip_t, -seat_w / 2),
         (seat_d / 2 - lip_t, -seat_w / 2 + lip_t),
         (-seat_d / 2 + lip_t, -seat_w / 2 + lip_t)],
        lip_h, cap=True, center=False))

    # Right lip (+Y edge)
    merged.merge(ExtrudeGeometry(
        [(-seat_d / 2 + lip_t, seat_w / 2 - lip_t),
         (seat_d / 2 - lip_t, seat_w / 2 - lip_t),
         (seat_d / 2 - lip_t, seat_w / 2),
         (-seat_d / 2 + lip_t, seat_w / 2)],
        lip_h, cap=True, center=False))

    # Front lip (+X edge, outward from beam center)
    merged.merge(ExtrudeGeometry(
        [(seat_d / 2 - lip_t, -seat_w / 2 + lip_t),
         (seat_d / 2, -seat_w / 2 + lip_t),
         (seat_d / 2, seat_w / 2 - lip_t),
         (seat_d / 2 - lip_t, seat_w / 2 - lip_t)],
        lip_h, cap=True, center=False))

    return mesh_from_geometry(merged, f"molded_seat_{index}")


def _backrest_geometry(index: int):
    """Backrest panel: slightly curved plate, 0.22 wide (Y), 0.22 tall (Z), 0.012 thick (X).
    Profile in XY (X=depth, Y=width), extruded along Z (height)."""
    w = 0.22  # width along Y
    h = 0.22  # height (extrusion along Z)
    t = 0.012
    # Profile in XY: X = depth/thickness, Y = width
    profile = []
    n = 12
    for i in range(n + 1):
        frac = i / n
        yy = -w / 2 + w * frac
        x_off = -0.012 * (1.0 - (2.0 * frac - 1.0) ** 2)
        profile.append((x_off, yy))
    for i in range(n, -1, -1):
        frac = i / n
        yy = -w / 2 + w * frac
        x_off = -0.012 * (1.0 - (2.0 * frac - 1.0) ** 2) + t
        profile.append((x_off, yy))

    # Extrude along Z (height direction), bottom at z=0
    geom = ExtrudeGeometry(profile, h, cap=True, center=False)
    return mesh_from_geometry(geom, f"backrest_{index}")


def _bumper_geometry(x: float, index: int):
    r_out = 0.060
    r_in = 0.044
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


def _gusset_geometry():
    profile = [(-0.08, 0.045), (0.08, 0.045), (0.0, 0.015)]
    geom = ExtrudeGeometry(profile, 0.018, cap=True, center=True)
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, "gusset_plate")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inclusive_playground_seesaw")

    galvanized = model.material("weathered_galvanized", rgba=(0.55, 0.58, 0.56, 1.0))
    rust = model.material("rust_steel", rgba=(0.42, 0.25, 0.13, 1.0))
    mustard = model.material("rusty_mustard_paint", rgba=(0.74, 0.53, 0.12, 1.0))
    pale_steel = model.material("pale_weathered_steel", rgba=(0.70, 0.66, 0.58, 1.0))
    molded_green = model.material("molded_green_plastic", rgba=(0.18, 0.42, 0.22, 1.0))
    rubber = model.material("black_rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    dark_blue = model.material("dark_blue_plastic", rgba=(0.10, 0.15, 0.45, 1.0))

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
                f"arch_{i}",
            ),
            material=galvanized,
            name=f"arch_{i}",
        )

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
                xyz=(0.0, side * (AXLE_LEN / 2.0 - 0.005), PIVOT_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=rust,
            name=f"axle_nut_{i}",
        )

    # --------------------------------------------------------------- beam ---
    beam = model.part("beam")

    beam.visual(
        Cylinder(radius=0.024, length=0.040),
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

    for i, px in enumerate((-0.70, -0.20, 0.35, 0.80)):
        beam.visual(
            Box((0.14, BEAM_W + 0.004, 0.010)),
            origin=Origin(xyz=(px, 0.0, BAR_TOP - 0.003)),
            material=rust,
            name=f"rust_patch_{i}",
        )

    # Per-end: molded seats, backrests, bumpers
    for i, side in enumerate((1.0, -1.0)):
        seat_x = side * SEAT_X

        # Molded bucket seat on top of beam
        beam.visual(
            _molded_seat_geometry(i),
            origin=Origin(xyz=(seat_x, 0.0, BAR_TOP)),
            material=molded_green,
            name=f"seat_{i}",
        )

        # Backrest: upright panel behind seat (inboard toward center)
        backrest_x = seat_x - side * 0.13
        beam.visual(
            _backrest_geometry(i),
            origin=Origin(xyz=(backrest_x, 0.0, BAR_TOP)),
            material=dark_blue,
            name=f"backrest_{i}",
        )

        # Bumper under each end
        beam.visual(
            _bumper_geometry(side * BUMPER_X, i),
            material=rubber,
            name=f"bumper_{i}",
        )

    # -------------------------------------------------------- handlebars ---
    for i, side in enumerate((1.0, -1.0)):
        hb = model.part(f"handlebar_{i}")

        # Mounting bracket: box spanning between rod legs, connects to beam
        hb.visual(
            Box((0.036, 0.090, 0.022)),
            origin=Origin(xyz=(0.0, 0.0, -0.011)),
            material=rust,
            name=f"handle_mount_{i}",
        )

        # Rod: inverted U shape
        hb.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _handle_rod_points(),
                    radius=0.010,
                    samples_per_segment=8,
                    radial_segments=16,
                    cap_ends=True,
                ),
                f"handlebar_rod_{i}",
            ),
            material=pale_steel,
            name=f"handlebar_rod_{i}",
        )

        # Rubber grip on the top arc
        hb.visual(
            Cylinder(radius=0.015, length=0.08),
            origin=Origin(xyz=(0.0, 0.0, 0.20), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"grip_{i}",
        )

    # -------------------------------------------------------------- joints ---
    model.articulation(
        "beam_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=200.0, velocity=2.5, lower=-TILT, upper=TILT),
    )

    for i, side in enumerate((1.0, -1.0)):
        hb_part = model.get_part(f"handlebar_{i}")
        model.articulation(
            f"handlebar_pivot_{i}",
            ArticulationType.REVOLUTE,
            parent=beam,
            child=hb_part,
            origin=Origin(xyz=(side * HANDLE_X, 0.0, BAR_TOP + 0.005)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=10.0, velocity=1.5,
                lower=-HANDLE_TILT, upper=HANDLE_TILT,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("arched_base")
    beam = object_model.get_part("beam")
    pivot = object_model.get_articulation("beam_pivot")
    hb0 = object_model.get_part("handlebar_0")
    hb1 = object_model.get_part("handlebar_1")
    hb_pivot_0 = object_model.get_articulation("handlebar_pivot_0")
    hb_pivot_1 = object_model.get_articulation("handlebar_pivot_1")

    # --- Pivot sleeve/axle intentional nesting ---
    ctx.allow_overlap(
        beam, base,
        elem_a="pivot_sleeve", elem_b="pivot_axle",
        reason="Pivot sleeve is a bushing intentionally nested around the axle bolt.",
    )
    # The arch tubes pass through the pivot sleeve region at the apex
    for arch_name in ("arch_0", "arch_1"):
        ctx.allow_overlap(
            base, beam,
            elem_a=arch_name, elem_b="pivot_sleeve",
            reason=f"Arch tube passes through the pivot sleeve region at the saddle apex.",
        )

    ctx.expect_contact(
        beam, base,
        elem_a="pivot_sleeve", elem_b="pivot_axle",
        name="pivot sleeve is seated on the axle bolt",
    )
    ctx.expect_within(
        beam, base,
        axes="y",
        inner_elem="pivot_sleeve",
        outer_elem="pivot_axle",
        margin=0.001,
        name="pivot sleeve stays inside the axle span",
    )

    # Handlebar mounts intentionally embed slightly into the beam bar
    for i in range(2):
        ctx.allow_overlap(
            beam, f"handlebar_{i}",
            elem_a="beam_bar", elem_b=f"handle_mount_{i}",
            reason="Handle mount bracket is bolted into the beam bar top surface.",
        )

    # --- Beam bar clears the arch saddle ---
    ctx.expect_gap(
        beam, base,
        axis="z",
        positive_elem="beam_bar",
        negative_elem="arch_0",
        min_gap=0.002,
        max_gap=0.10,
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

    # --- Low inclusive height ---
    axle_box = ctx.part_element_world_aabb(base, elem="pivot_axle")
    ctx.check(
        "pivot axle sits at low inclusive height (~0.45-0.55 m)",
        axle_box is not None and 0.40 <= (axle_box[0][2] + axle_box[1][2]) / 2.0 <= 0.58,
        details=f"axle aabb={axle_box}",
    )

    # --- Scale ---
    bar_box = ctx.part_element_world_aabb(beam, elem="beam_bar")
    base_box = ctx.part_world_aabb(base)
    ctx.check(
        "beam is about 3.0 m long",
        bar_box is not None and abs((bar_box[1][0] - bar_box[0][0]) - 3.0) < 0.02,
        details=f"bar aabb={bar_box}",
    )
    ctx.check(
        "arched base feet rest on the ground",
        base_box is not None and -0.01 <= base_box[0][2] <= 0.02,
        details=f"base aabb={base_box}",
    )

    # --- Molded seats with raised lips ---
    for i in range(2):
        seat = ctx.part_element_world_aabb(beam, elem=f"seat_{i}")
        ctx.check(
            f"seat_{i} is mounted on the beam bar top",
            seat is not None
            and bar_box is not None
            and seat[0][2] >= bar_box[1][2] - 0.005
            and seat[1][2] > bar_box[1][2] + 0.02,
            details=f"seat aabb={seat}",
        )
        ctx.check(
            f"seat_{i} has raised lips (height exceeds flat plate)",
            seat is not None and (seat[1][2] - seat[0][2]) > 0.030,
            details=f"seat height={seat[1][2] - seat[0][2] if seat else None}",
        )

    # --- Backrests extend above seats ---
    for i in range(2):
        backrest = ctx.part_element_world_aabb(beam, elem=f"backrest_{i}")
        ctx.check(
            f"backrest_{i} extends above the beam bar",
            backrest is not None
            and bar_box is not None
            and backrest[1][2] > bar_box[1][2] + 0.15,
            details=f"backrest aabb={backrest}",
        )

    # --- Handlebar pivots are non-trivial revolute joints ---
    for i in range(2):
        hb_pivot = object_model.get_articulation(f"handlebar_pivot_{i}")
        hb_lim = hb_pivot.motion_limits
        ctx.check(
            f"handlebar_pivot_{i} has non-trivial revolute limits",
            hb_lim is not None
            and hb_lim.lower is not None
            and hb_lim.upper is not None
            and hb_lim.lower < 0.0
            and hb_lim.upper > 0.0
            and abs(hb_lim.upper - hb_lim.lower) > math.radians(5.0),
            details=f"limits=({hb_lim.lower}, {hb_lim.upper})",
        )

    # --- Handlebars extend above the beam ---
    for i in range(2):
        hb_part = object_model.get_part(f"handlebar_{i}")
        hb_box = ctx.part_world_aabb(hb_part)
        ctx.check(
            f"handlebar_{i} extends above the beam",
            hb_box is not None
            and bar_box is not None
            and hb_box[1][2] > bar_box[1][2] + 0.15,
            details=f"handlebar aabb={hb_box}",
        )

    # --- Handlebar pivot proof: positive pose tilts handlebar ---
    hb0_rest = ctx.part_world_aabb(hb0)
    with ctx.pose({hb_pivot_0: HANDLE_TILT}):
        hb0_tilted = ctx.part_world_aabb(hb0)
        ctx.check(
            "handlebar_0 pivots when actuated",
            hb0_rest is not None
            and hb0_tilted is not None
            and (abs(hb0_tilted[0][0] - hb0_rest[0][0]) > 0.001
                 or abs(hb0_tilted[0][2] - hb0_rest[0][2]) > 0.001),
            details=f"rest={hb0_rest}, tilted={hb0_tilted}",
        )

    # --- Decisive pose: rocking alternately lowers each end ---
    rest_b0 = ctx.part_element_world_aabb(beam, elem="bumper_0")
    with ctx.pose({pivot: TILT}):
        down_b0 = ctx.part_element_world_aabb(beam, elem="bumper_0")
        up_b1 = ctx.part_element_world_aabb(beam, elem="bumper_1")
        ctx.check(
            "positive rock lowers the +X end near the ground",
            rest_b0 is not None
            and down_b0 is not None
            and down_b0[0][2] < rest_b0[0][2] - 0.15,
            details=f"rest={rest_b0}, tilted={down_b0}",
        )
        ctx.check(
            "positive rock raises the -X end",
            up_b1 is not None and up_b1[0][2] > 0.7,
            details=f"raised bumper aabb={up_b1}",
        )
    with ctx.pose({pivot: -TILT}):
        down_b1 = ctx.part_element_world_aabb(beam, elem="bumper_1")
        ctx.check(
            "negative rock lowers the -X end toward the ground",
            down_b1 is not None and down_b1[0][2] < rest_b0[0][2] - 0.15,
            details=f"tilted bumper aabb={down_b1}",
        )

    return ctx.report()


object_model = build_object_model()
