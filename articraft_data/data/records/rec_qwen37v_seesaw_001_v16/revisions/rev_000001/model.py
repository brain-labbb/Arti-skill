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
# Curved-beam spring-loaded playground seesaw (Variant 16)
#
# World frame: beam runs along X, pivot axis along Y, Z up.
# - Two bent galvanized-steel tube arches (~50 mm dia) form an A-shaped
#   saddle; rubber ground pads sit under each foot.
# - A flat saddle plate bridges the two arch apexes.
# - A helical compression spring sits on the saddle plate, carrying the
#   pivot pin at its top via a PRISMATIC joint (axis -Z, 40 mm travel).
# - The curved beam (banana-shaped, raised ends) rocks on a REVOLUTE joint
#   about the pivot pin (axis +Y, +/- 20 deg).
# - Each beam end carries a wooden seat, an inverted-U grab handle, and a
#   rubber bump stop block underneath.
# ---------------------------------------------------------------------------

BEAM_HALF = 1.50  # 3.0 m beam
BEAM_W = 0.08
BEAM_T = 0.04

SADDLE_Z = 0.70  # arch apex / saddle plate height
PIVOT_HEIGHT = 0.80  # beam pivot height at rest

ARCH_FOOT_X = 0.66
ARCH_FOOT_Y = 0.34
ARCH_APEX_Y = 0.065  # arches cross past center for A-shape
ARCH_FOOT_Z = 0.028
TUBE_R = 0.025  # ~50 mm diameter bent tube

AXLE_R = 0.016
AXLE_LEN = 0.14

BAR_BOT = 0.05
BAR_CTR = BAR_BOT + BEAM_T / 2.0
BAR_TOP = BAR_BOT + BEAM_T

SEAT_X = 1.30
HANDLE_X = 1.04
BUMP_STOP_X = 1.42
TILT = math.radians(20.0)

CURVE_RISE = 0.15  # beam ends rise above center

# Spring geometry
SPRING_COIL_R = 0.028
SPRING_WIRE_R = 0.005
SPRING_TURNS = 5
SPRING_HEIGHT = 0.08
SPRING_TOP_OFFSET = 0.036  # spring top below pivot sleeve bottom
SPRING_TRAVEL = 0.04

# Ground pads
PAD_R = 0.055
PAD_T = 0.014

# Bump stops
BUMP_W = 0.10
BUMP_D = 0.08
BUMP_H = 0.05


def beam_curve_z(x: float) -> float:
    """Height of curved beam centerline at position x (in beam-local frame)."""
    return BAR_CTR + CURVE_RISE * (x / BEAM_HALF) ** 2


def _arch_points(side: float) -> list[tuple[float, float, float]]:
    """Centerline of one bent-tube arch."""
    pts: list[tuple[float, float, float]] = []
    rise = SADDLE_Z - ARCH_FOOT_Z
    for i in range(11):
        t = -1.0 + 0.2 * i
        s = 1.0 - t * t
        x = ARCH_FOOT_X * t
        z = ARCH_FOOT_Z + rise * s
        y = side * ARCH_FOOT_Y + (-side * ARCH_APEX_Y - side * ARCH_FOOT_Y) * s
        pts.append((x, y, z))
    return pts


def _curved_beam_geometry():
    """Curved beam bar: extruded rectangular profile following a parabolic arc."""
    n = 28
    half_t = BEAM_T / 2.0
    top: list[tuple[float, float]] = []
    bot: list[tuple[float, float]] = []
    for i in range(n + 1):
        x = -BEAM_HALF + (2.0 * BEAM_HALF) * i / n
        z = beam_curve_z(x)
        top.append((x, z + half_t))
        bot.append((x, z - half_t))
    profile = top + list(reversed(bot))
    geom = ExtrudeGeometry(profile, BEAM_W, cap=True, center=True)
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, "curved_beam_bar")


def _handle_points(x_pos: float) -> list[tuple[float, float, float]]:
    """Inverted-U grab-handle rod centerline adjusted for curved beam."""
    half_w = 0.035
    btop = beam_curve_z(x_pos) + BEAM_T / 2.0
    leg_bot = btop - 0.010
    stem_top = btop + 0.170
    arc_z = btop + 0.250
    pts: list[tuple[float, float, float]] = [
        (x_pos, -half_w, leg_bot),
        (x_pos, -half_w, stem_top),
    ]
    for k in range(7):
        a = math.pi * k / 6.0
        pts.append((x_pos, -half_w * math.cos(a), arc_z + half_w * math.sin(a)))
    pts.append((x_pos, half_w, stem_top))
    pts.append((x_pos, half_w, leg_bot))
    return pts


def _spring_points() -> list[tuple[float, float, float]]:
    """Helical centerline for the compression spring coil."""
    pts: list[tuple[float, float, float]] = []
    n_total = SPRING_TURNS * 18
    z_top = -SPRING_TOP_OFFSET
    z_bot = z_top - SPRING_HEIGHT
    for i in range(n_total + 1):
        t = i / n_total
        angle = 2.0 * math.pi * SPRING_TURNS * t
        px = SPRING_COIL_R * math.cos(angle)
        py = SPRING_COIL_R * math.sin(angle)
        pz = z_top + (z_bot - z_top) * t
        pts.append((px, py, pz))
    return pts


def _gusset_geometry():
    """Triangular gusset plate joining the beam to the pivot area."""
    profile = [(-0.10, 0.055), (0.10, 0.055), (0.0, 0.020)]
    geom = ExtrudeGeometry(profile, 0.020, cap=True, center=True)
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, "seesaw_gusset_plate")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="curved_spring_seesaw")

    # -- materials --
    galvanized = model.material("weathered_galvanized", rgba=(0.55, 0.58, 0.56, 1.0))
    rust = model.material("rust_steel", rgba=(0.42, 0.25, 0.13, 1.0))
    mustard = model.material("rusty_mustard_paint", rgba=(0.74, 0.53, 0.12, 1.0))
    pale_steel = model.material("pale_weathered_steel", rgba=(0.70, 0.66, 0.58, 1.0))
    wood = model.material("worn_wood", rgba=(0.60, 0.45, 0.28, 1.0))
    rubber = model.material("black_rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    spring_steel = model.material("spring_steel", rgba=(0.45, 0.48, 0.50, 1.0))
    pad_rubber = model.material("ground_pad_rubber", rgba=(0.14, 0.14, 0.14, 1.0))

    # ============================================================ BASE ====
    base = model.part("arched_base")

    # Arched tube legs
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

    # Saddle plate bridging the two arch apexes (connects the arches)
    base.visual(
        Cylinder(radius=0.070, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, SADDLE_Z - 0.006)),
        material=galvanized,
        name="saddle_plate",
    )

    # Rubber ground pads (4 pads, one under each arch foot)
    pad_idx = 0
    for side_y in (1.0, -1.0):
        for side_x in (1.0, -1.0):
            base.visual(
                Cylinder(radius=PAD_R, length=PAD_T),
                origin=Origin(
                    xyz=(side_x * ARCH_FOOT_X, side_y * ARCH_FOOT_Y, PAD_T / 2.0),
                ),
                material=pad_rubber,
                name=f"ground_pad_{pad_idx}",
            )
            pad_idx += 1

    # =================================================== SPRING_POST ====
    spring_post = model.part("spring_post")

    # Helical compression spring coil
    spring_post.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                _spring_points(),
                radius=SPRING_WIRE_R,
                samples_per_segment=4,
                radial_segments=10,
                cap_ends=True,
            ),
            "spring_coil_mesh",
        ),
        material=spring_steel,
        name="spring_coil",
    )

    # Top bearing plate (flat disk between spring top and pivot)
    spring_post.visual(
        Cylinder(radius=0.042, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -SPRING_TOP_OFFSET + 0.005)),
        material=pale_steel,
        name="top_plate",
    )

    # Pin boss bridging from the top plate to the pivot pin
    spring_post.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.021)),
        material=pale_steel,
        name="pin_boss",
    )

    # Pivot pin (horizontal cylinder along Y for the beam sleeve)
    spring_post.visual(
        Cylinder(radius=AXLE_R, length=AXLE_LEN),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rust,
        name="pivot_pin",
    )

    # ============================================================= BEAM ====
    beam = model.part("beam")

    # Pivot sleeve (captures the pivot pin)
    beam.visual(
        Cylinder(radius=0.026, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rust,
        name="pivot_sleeve",
    )

    # Gusset plate
    beam.visual(_gusset_geometry(), material=mustard, name="gusset_plate")

    # Curved beam bar (parabolic arc, raised ends)
    beam.visual(_curved_beam_geometry(), material=mustard, name="beam_bar")

    # Rust streak patches following the curve
    for i, px in enumerate((-0.85, -0.30, 0.45, 0.95)):
        z_top = beam_curve_z(px) + BEAM_T / 2.0
        beam.visual(
            Box((0.16, BEAM_W + 0.004, 0.012)),
            origin=Origin(xyz=(px, 0.0, z_top - 0.004)),
            material=rust,
            name=f"rust_patch_{i}",
        )

    # End fittings: seats, handles, bump stops
    for i, side in enumerate((1.0, -1.0)):
        # Wooden seat plate on top of the curved beam
        seat_z = beam_curve_z(side * SEAT_X) + BEAM_T / 2.0 + 0.011
        beam.visual(
            Box((0.30, 0.24, 0.022)),
            origin=Origin(xyz=(side * SEAT_X, 0.0, seat_z)),
            material=wood,
            name=f"seat_{i}",
        )

        # Inverted-U grab handle
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

        # Rubber bump stop block below beam end
        bump_z = beam_curve_z(side * BUMP_STOP_X) - BEAM_T / 2.0 - BUMP_H / 2.0
        beam.visual(
            Box((BUMP_W, BUMP_D, BUMP_H)),
            origin=Origin(xyz=(side * BUMP_STOP_X, 0.0, bump_z)),
            material=rubber,
            name=f"bump_stop_{i}",
        )

    # ========================================================== JOINTS ====

    # Prismatic: base to spring_post (vertical spring compression)
    model.articulation(
        "spring_compress",
        ArticulationType.PRISMATIC,
        parent=base,
        child=spring_post,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_HEIGHT)),
        axis=(0.0, 0.0, -1.0),  # positive q compresses spring downward
        motion_limits=MotionLimits(
            effort=500.0, velocity=0.5, lower=0.0, upper=SPRING_TRAVEL
        ),
    )

    # Revolute: spring_post to beam (rocking pivot)
    model.articulation(
        "beam_pivot",
        ArticulationType.REVOLUTE,
        parent=spring_post,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),  # positive q lowers the +X end
        motion_limits=MotionLimits(
            effort=200.0, velocity=2.5, lower=-TILT, upper=TILT
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("arched_base")
    spring_post = object_model.get_part("spring_post")
    beam = object_model.get_part("beam")
    pivot = object_model.get_articulation("beam_pivot")
    spring = object_model.get_articulation("spring_compress")

    # --- Overlap allowances ---

    # Pivot sleeve captures the pivot pin (bushing-on-pin)
    ctx.allow_overlap(
        beam,
        spring_post,
        elem_a="pivot_sleeve",
        elem_b="pivot_pin",
        reason="Pivot sleeve is a bushing intentionally capturing the pivot pin.",
    )
    ctx.expect_contact(
        beam,
        spring_post,
        elem_a="pivot_sleeve",
        elem_b="pivot_pin",
        name="pivot sleeve is seated on the pivot pin",
    )

    # Top plate sits against the pivot sleeve bottom (bearing surface)
    ctx.allow_overlap(
        beam,
        spring_post,
        elem_a="pivot_sleeve",
        elem_b="top_plate",
        reason="Pivot sleeve seats onto the spring top plate bearing surface.",
    )

    # Pin boss is part of the pivot pin assembly captured by the sleeve
    ctx.allow_overlap(
        beam,
        spring_post,
        elem_a="pivot_sleeve",
        elem_b="pin_boss",
        reason="Pin boss supports the pivot pin inside the captured sleeve bushing.",
    )

    # Spring coil sits on the saddle plate; at max compression the coil
    # bottom nestles into the plate recess (small intentional embed).
    ctx.allow_overlap(
        spring_post,
        base,
        elem_a="spring_coil",
        elem_b="saddle_plate",
        reason="Spring coil seats onto the saddle plate; minor embed at max compression.",
    )

    # --- Curved beam geometry ---
    beam_bar = ctx.part_element_world_aabb(beam, elem="beam_bar")
    ctx.check(
        "curved beam spans about 3.0 m",
        beam_bar is not None
        and abs((beam_bar[1][0] - beam_bar[0][0]) - 3.0) < 0.05,
        details=f"beam_bar aabb={beam_bar}",
    )
    ctx.check(
        "beam has raised ends (curved beam top above center height)",
        beam_bar is not None
        and beam_bar[1][2] > PIVOT_HEIGHT + BAR_CTR + 0.08,
        details=f"beam_bar max_z={beam_bar[1][2] if beam_bar else None}",
    )

    # --- Spring mechanism ---
    ctx.check(
        "spring joint is prismatic with vertical axis",
        spring.articulation_type == ArticulationType.PRISMATIC
        and abs(spring.axis[2]) > 0.9,
        details=f"type={spring.articulation_type}, axis={spring.axis}",
    )
    spring_lim = spring.motion_limits
    ctx.check(
        "spring has compression travel > 20 mm",
        spring_lim is not None
        and spring_lim.upper is not None
        and spring_lim.upper > 0.02,
        details=f"limits=({spring_lim.lower}, {spring_lim.upper})",
    )
    spring_aabb = ctx.part_element_world_aabb(spring_post, elem="spring_coil")
    ctx.check(
        "spring coil visual exists below the pivot",
        spring_aabb is not None and spring_aabb[0][2] < PIVOT_HEIGHT - 0.04,
        details=f"spring_coil aabb={spring_aabb}",
    )

    # --- Ground pads ---
    for i in range(4):
        pad = ctx.part_element_world_aabb(base, elem=f"ground_pad_{i}")
        ctx.check(
            f"ground_pad_{i} sits near ground level",
            pad is not None and pad[0][2] < 0.02 and pad[1][2] < 0.04,
            details=f"pad aabb={pad}",
        )

    # --- Bump stops ---
    for i in range(2):
        bump = ctx.part_element_world_aabb(beam, elem=f"bump_stop_{i}")
        ctx.check(
            f"bump_stop_{i} hangs below the curved beam at its end",
            bump is not None
            and beam_bar is not None
            and bump[0][2] < beam_bar[1][2] - 0.04,
            details=f"bump_stop aabb={bump}",
        )

    # --- Pivot joint configuration ---
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

    # --- Base grounded ---
    base_box = ctx.part_world_aabb(base)
    ctx.check(
        "arched base feet rest on the ground",
        base_box is not None and -0.02 <= base_box[0][2] <= 0.02,
        details=f"base aabb={base_box}",
    )

    # --- Beam clearance above arch saddle ---
    ctx.expect_gap(
        beam,
        base,
        axis="z",
        positive_elem="beam_bar",
        negative_elem="arch_0",
        min_gap=0.005,
        max_gap=0.20,
        name="curved beam bar clears the arch saddle",
    )

    # --- Decisive pose: beam rocking ---
    rest_b0 = ctx.part_element_world_aabb(beam, elem="bump_stop_0")
    with ctx.pose({pivot: TILT}):
        down_b0 = ctx.part_element_world_aabb(beam, elem="bump_stop_0")
        up_b1 = ctx.part_element_world_aabb(beam, elem="bump_stop_1")
        ctx.check(
            "positive rock lowers the +X bump stop",
            rest_b0 is not None
            and down_b0 is not None
            and down_b0[0][2] < rest_b0[0][2] - 0.25,
            details=f"rest={rest_b0}, tilted={down_b0}",
        )
        ctx.check(
            "positive rock raises the -X bump stop",
            up_b1 is not None and up_b1[1][2] > PIVOT_HEIGHT + 0.2,
            details=f"raised bump_stop aabb={up_b1}",
        )

    # --- Spring compression pose ---
    rest_plate = ctx.part_element_world_aabb(spring_post, elem="top_plate")
    with ctx.pose({spring: SPRING_TRAVEL}):
        compressed_plate = ctx.part_element_world_aabb(spring_post, elem="top_plate")
        ctx.check(
            "spring compression lowers the top plate",
            rest_plate is not None
            and compressed_plate is not None
            and compressed_plate[1][2] < rest_plate[1][2] - 0.02,
            details=f"rest={rest_plate}, compressed={compressed_plate}",
        )

    return ctx.report()


object_model = build_object_model()
