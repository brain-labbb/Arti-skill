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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Variant 17 — Low inclusive playground seesaw with backrest seats
#
# World frame: beam runs along X, pivot axis along Y, Z up.
# - Two bent-tube arches (~50 mm dia) form an A-shaped saddle at reduced
#   height (~0.50 m) for inclusive accessibility.
# - Beam: 3.0 m mustard-yellow steel bar (80 x 40 mm) with pivot sleeve
#   and triangular gusset at center.
# - Each end carries: a molded seat with raised lips + backrest, an inverted-U
#   grab handle with rounded grip, and a rubber bumper on a prismatic joint.
# - Articulations:
#   beam_pivot: revolute, Y axis, +/- 20 degrees
#   bumper_0_compress / bumper_1_compress: prismatic, Z axis, 0..30 mm
# ---------------------------------------------------------------------------

BEAM_HALF = 1.50
BEAM_W = 0.08
BEAM_T = 0.04
PIVOT_Z = 0.50  # low inclusive pivot height

ARCH_FOOT_X = 0.50
ARCH_FOOT_Y = 0.38
ARCH_APEX_Y = 0.06
ARCH_FOOT_Z = 0.025
TUBE_R = 0.025

AXLE_R = 0.016
AXLE_LEN = 0.22

BAR_BOT = 0.04
BAR_CTR = BAR_BOT + BEAM_T / 2.0  # 0.06
BAR_TOP = BAR_BOT + BEAM_T  # 0.08

SEAT_X = 1.20
HANDLE_X = 0.96
BUMPER_X = 1.42
TILT = math.radians(20.0)
BUMPER_COMPRESS = 0.030

# Seat parameters (in beam-local frame)
SEAT_SX = 0.14  # half-length along beam (X)
SEAT_SY = 0.12  # half-width across beam (Y)
PAN_T = 0.012  # seat pan thickness
LIP_H = 0.018  # lip height above pan
LIP_T = 0.010  # lip wall thickness

# Backrest
BR_HW = 0.11  # half-width
BR_H = 0.20  # height
BR_T = 0.008  # thickness
BR_TILT = math.radians(5.0)


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


def _handle_points(x: float) -> list[tuple[float, float, float]]:
    """Inverted-U grab-handle rod centerline."""
    half_w = 0.040
    leg_bot = BAR_TOP - 0.008
    arc_z = 0.260
    pts: list[tuple[float, float, float]] = [
        (x, -half_w, leg_bot),
        (x, -half_w, 0.180),
    ]
    for k in range(7):
        a = math.pi * k / 6.0
        pts.append((x, -half_w * math.cos(a), arc_z + half_w * math.sin(a)))
    pts.append((x, half_w, 0.180))
    pts.append((x, half_w, leg_bot))
    return pts


def _gusset_geometry():
    """Triangular gusset plate joining the beam bar to the pivot sleeve."""
    profile = [(-0.10, 0.055), (0.10, 0.055), (0.0, 0.020)]
    geom = ExtrudeGeometry(profile, 0.020, cap=True, center=True)
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, "seesaw_gusset_plate")


def _bumper_shell_mesh(index: int):
    """Solid half-cylinder rubber bumper pad (D-shaped cross-section).

    Flat top face sits against the mounting plate; curved bottom faces
    the ground.  Solid profile ensures the center bolt/mount overlaps
    with the rubber body for connectivity.
    """
    r = 0.055
    profile: list[tuple[float, float]] = []
    n = 24
    for k in range(n + 1):
        a = math.pi + math.pi * k / n  # bottom semicircle
        profile.append((r * math.cos(a), r * math.sin(a)))
    # ExtrudeGeometry auto-closes the profile along the diameter line (y=0)
    geom = ExtrudeGeometry(profile, 0.10, cap=True, center=True)
    # Profile in XY (semicircle below Y=0), extrude along Z.
    # After rotate_x(pi/2): flat top at Z=0, curved bottom at Z=-r,
    # extrusion becomes along Y (±0.05).
    geom.rotate_x(math.pi / 2.0)
    # Shift slightly up so the flat top embeds into the mount plate
    geom.translate(0.0, 0.0, 0.003)
    return mesh_from_geometry(geom, f"bumper_shell_{index}")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inclusive_playground_seesaw")

    galvanized = model.material("weathered_galvanized", rgba=(0.55, 0.58, 0.56, 1.0))
    rust = model.material("rust_steel", rgba=(0.42, 0.25, 0.13, 1.0))
    mustard = model.material("mustard_paint", rgba=(0.74, 0.53, 0.12, 1.0))
    pale_steel = model.material("pale_steel", rgba=(0.70, 0.68, 0.64, 1.0))
    molded_green = model.material("molded_green_plastic", rgba=(0.18, 0.42, 0.22, 1.0))
    backrest_red = model.material("molded_red_plastic", rgba=(0.55, 0.15, 0.12, 1.0))
    rubber = model.material("black_rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.12, 0.12, 0.14, 1.0))

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

    # Pivot axle bolt through both arch apexes
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

    # Pivot sleeve (bushing around axle)
    beam.visual(
        Cylinder(radius=0.024, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rust,
        name="pivot_sleeve",
    )
    beam.visual(_gusset_geometry(), material=mustard, name="gusset_plate")

    # Main beam bar
    beam.visual(
        Box((2.0 * BEAM_HALF, BEAM_W, BEAM_T)),
        origin=Origin(xyz=(0.0, 0.0, BAR_CTR)),
        material=mustard,
        name="beam_bar",
    )

    # ------------------------------------------------- per-end fittings ---
    for i, side in enumerate((1.0, -1.0)):
        seat_cx = side * SEAT_X  # seat center X in beam frame

        # --- Molded seat pan (horizontal plate, extrude along Z) ---
        pan_profile = [
            (-SEAT_SX, -SEAT_SY), (SEAT_SX, -SEAT_SY),
            (SEAT_SX, SEAT_SY), (-SEAT_SX, SEAT_SY),
        ]
        pan_geom = ExtrudeGeometry(pan_profile, PAN_T, cap=True, center=True)
        pan_geom.translate(seat_cx, 0.0, BAR_TOP + PAN_T / 2.0)
        beam.visual(
            mesh_from_geometry(pan_geom, f"seat_pan_{i}"),
            material=molded_green,
            name=f"seat_pan_{i}",
        )

        # Lip Z center (above pan top)
        lip_z = BAR_TOP + PAN_T + LIP_H / 2.0

        # --- Front lip (at outer end of seat, away from pivot) ---
        fl_profile = [
            (-LIP_T / 2.0, -SEAT_SY), (LIP_T / 2.0, -SEAT_SY),
            (LIP_T / 2.0, SEAT_SY), (-LIP_T / 2.0, SEAT_SY),
        ]
        fl_geom = ExtrudeGeometry(fl_profile, LIP_H, cap=True, center=True)
        fl_geom.translate(seat_cx + side * (SEAT_SX - LIP_T / 2.0), 0.0, lip_z)
        beam.visual(
            mesh_from_geometry(fl_geom, f"seat_lip_front_{i}"),
            material=molded_green,
            name=f"seat_lip_front_{i}",
        )

        # --- Back lip (at inner end of seat, toward pivot) ---
        bl_geom = ExtrudeGeometry(fl_profile, LIP_H, cap=True, center=True)
        bl_geom.translate(seat_cx - side * (SEAT_SX - LIP_T / 2.0), 0.0, lip_z)
        beam.visual(
            mesh_from_geometry(bl_geom, f"seat_lip_back_{i}"),
            material=molded_green,
            name=f"seat_lip_back_{i}",
        )

        # --- Left lip (+Y side) ---
        sl_profile = [
            (-SEAT_SX, -LIP_T / 2.0), (SEAT_SX, -LIP_T / 2.0),
            (SEAT_SX, LIP_T / 2.0), (-SEAT_SX, LIP_T / 2.0),
        ]
        ll_geom = ExtrudeGeometry(sl_profile, LIP_H, cap=True, center=True)
        ll_geom.translate(seat_cx, SEAT_SY - LIP_T / 2.0, lip_z)
        beam.visual(
            mesh_from_geometry(ll_geom, f"seat_lip_left_{i}"),
            material=molded_green,
            name=f"seat_lip_left_{i}",
        )

        # --- Right lip (-Y side) ---
        rl_geom = ExtrudeGeometry(sl_profile, LIP_H, cap=True, center=True)
        rl_geom.translate(seat_cx, -(SEAT_SY - LIP_T / 2.0), lip_z)
        beam.visual(
            mesh_from_geometry(rl_geom, f"seat_lip_right_{i}"),
            material=molded_green,
            name=f"seat_lip_right_{i}",
        )

        # --- Backrest: vertical plate at rear of seat, slight recline ---
        # Build profile: 2*BR_HW wide (X), BR_T thick (Y), extrude BR_H (Z)
        br_profile = [
            (-BR_HW, -BR_T / 2.0), (BR_HW, -BR_T / 2.0),
            (BR_HW, BR_T / 2.0), (-BR_HW, BR_T / 2.0),
        ]
        br_geom = ExtrudeGeometry(br_profile, BR_H, cap=True, center=True)
        # Now: 0.22 (X) x 0.008 (Y) x 0.20 (Z)
        # Rotate 90° around Z to swap X↔Y: thin in X, wide in Y
        br_geom.rotate_z(math.pi / 2.0)
        # Now: 0.008 (X) x 0.22 (Y) x 0.20 (Z)
        # Tilt back (top leans away from center)
        br_geom.rotate_y(side * BR_TILT)
        # Position: rear of seat, bottom overlapping seat pan for connectivity
        br_cx = seat_cx - side * (SEAT_SX - 0.020)
        br_cz = BAR_TOP + PAN_T + BR_H / 2.0 - 0.010
        br_geom.translate(br_cx, 0.0, br_cz)
        beam.visual(
            mesh_from_geometry(br_geom, f"backrest_{i}"),
            material=backrest_red,
            name=f"backrest_{i}",
        )

        # --- Handle: inverted-U rod ---
        beam.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _handle_points(side * HANDLE_X),
                    radius=0.008,
                    samples_per_segment=8,
                    radial_segments=16,
                    cap_ends=True,
                ),
                f"handle_rod_{i}",
            ),
            material=pale_steel,
            name=f"handle_{i}",
        )

        # --- Rounded handle grip (sphere at top of arc) ---
        grip_z = 0.260 + 0.040  # arc_z + half_w from handle_points
        beam.visual(
            Sphere(radius=0.022),
            origin=Origin(xyz=(side * HANDLE_X, 0.0, grip_z)),
            material=grip_rubber,
            name=f"handle_grip_{i}",
        )

    # --------------------------------------------------------- bumpers ---
    for i, side in enumerate((1.0, -1.0)):
        bumper = model.part(f"bumper_{i}")

        # Rubber shell (half-annulus, hangs below mount)
        bumper.visual(
            _bumper_shell_mesh(i),
            material=rubber,
            name=f"bumper_shell_{i}",
        )
        # Mounting plate (connects to beam underside)
        bumper.visual(
            Box((0.06, 0.08, 0.008)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=rust,
            name=f"bumper_mount_{i}",
        )
        # Connecting bolt (bridges mount plate to shell for connectivity)
        # Bolt spans from z=-0.020 to z=0, overlapping both mount and shell
        bumper.visual(
            Cylinder(radius=0.012, length=0.020),
            origin=Origin(xyz=(0.0, 0.0, -0.010)),
            material=rust,
            name=f"bumper_bolt_{i}",
        )

        # Prismatic joint: bumper compresses vertically
        model.articulation(
            f"bumper_{i}_compress",
            ArticulationType.PRISMATIC,
            parent=beam,
            child=bumper,
            origin=Origin(xyz=(side * BUMPER_X, 0.0, BAR_BOT - 0.004)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=500.0, velocity=0.5,
                lower=0.0, upper=BUMPER_COMPRESS,
            ),
        )

    # -------------------------------------------------------------- pivot ---
    model.articulation(
        "beam_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=200.0, velocity=2.5,
            lower=-TILT, upper=TILT,
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
    bump_j0 = object_model.get_articulation("bumper_0_compress")
    bump_j1 = object_model.get_articulation("bumper_1_compress")

    # --- Pivot sleeve captures axle bolt ---
    ctx.allow_overlap(
        beam, base,
        elem_a="pivot_sleeve", elem_b="pivot_axle",
        reason="Pivot sleeve is a bushing intentionally nested around the axle bolt.",
    )
    ctx.expect_contact(
        beam, base,
        elem_a="pivot_sleeve", elem_b="pivot_axle",
        name="pivot sleeve seated on axle",
    )
    ctx.expect_within(
        beam, base, axes="y",
        inner_elem="pivot_sleeve", outer_elem="pivot_axle",
        margin=0.001,
        name="pivot sleeve inside axle span",
    )

    # --- Arch apex surrounds pivot region (saddle overlap is intentional) ---
    for arch_name in ("arch_0", "arch_1"):
        ctx.allow_overlap(
            base, beam,
            elem_a=arch_name, elem_b="pivot_sleeve",
            reason="Arch apex saddle intentionally surrounds the pivot region; "
                   "the pivot bolt passes through both arches at the crossing.",
        )

    # --- Beam clears arch saddle (non-apex region) ---
    ctx.expect_gap(
        beam, base, axis="z",
        positive_elem="beam_bar", negative_elem="arch_0",
        min_gap=0.002, max_gap=0.10,
        name="beam bar clears arch saddle",
    )

    # --- Revolute pivot: horizontal Y axis, +/-20 deg ---
    ax = pivot.axis
    ctx.check(
        "pivot axis is horizontal Y",
        abs(ax[0]) < 1e-9 and abs(ax[1] - 1.0) < 1e-9 and abs(ax[2]) < 1e-9,
        details=f"axis={ax}",
    )
    lim = pivot.motion_limits
    ctx.check(
        "rocking limits are +/- 20 degrees",
        lim is not None and lim.lower is not None and lim.upper is not None
        and abs(lim.lower + TILT) < 1e-6 and abs(lim.upper - TILT) < 1e-6,
        details=f"limits=({lim.lower}, {lim.upper})",
    )

    # --- Prismatic bumper joints: Z axis, 0..30mm ---
    for j, jname in [(bump_j0, "bumper_0"), (bump_j1, "bumper_1")]:
        jax = j.axis
        ctx.check(
            f"{jname}_compress axis is vertical Z",
            abs(jax[0]) < 1e-9 and abs(jax[1]) < 1e-9 and abs(jax[2] - 1.0) < 1e-9,
            details=f"axis={jax}",
        )
        jlim = j.motion_limits
        ctx.check(
            f"{jname}_compress has short travel (~30 mm)",
            jlim is not None and jlim.lower is not None and jlim.upper is not None
            and abs(jlim.lower) < 1e-6 and 0.020 <= jlim.upper <= 0.040,
            details=f"limits=({jlim.lower}, {jlim.upper})",
        )

    # --- Low inclusive pivot height ---
    axle_box = ctx.part_element_world_aabb(base, elem="pivot_axle")
    ctx.check(
        "pivot is low for inclusive access (~0.5 m)",
        axle_box is not None and 0.40 <= (axle_box[0][2] + axle_box[1][2]) / 2.0 <= 0.58,
        details=f"axle aabb={axle_box}",
    )

    # --- Beam length ---
    bar_box = ctx.part_element_world_aabb(beam, elem="beam_bar")
    ctx.check(
        "beam is about 3.0 m long",
        bar_box is not None and abs((bar_box[1][0] - bar_box[0][0]) - 3.0) < 0.02,
        details=f"bar aabb={bar_box}",
    )

    # --- Base grounded ---
    base_box = ctx.part_world_aabb(base)
    ctx.check(
        "arched base feet on ground",
        base_box is not None and -0.01 <= base_box[0][2] <= 0.03,
        details=f"base aabb={base_box}",
    )

    # --- Molded seats with raised lips ---
    for idx in range(2):
        seat_pan = ctx.part_element_world_aabb(beam, elem=f"seat_pan_{idx}")
        lip_front = ctx.part_element_world_aabb(beam, elem=f"seat_lip_front_{idx}")
        ctx.check(
            f"seat_pan_{idx} sits on beam bar",
            seat_pan is not None and bar_box is not None
            and seat_pan[0][2] >= bar_box[1][2] - 0.005
            and seat_pan[1][2] > bar_box[1][2],
            details=f"pan aabb={seat_pan}",
        )
        ctx.check(
            f"seat_lip_front_{idx} rises above seat pan top",
            lip_front is not None and seat_pan is not None
            and lip_front[1][2] > seat_pan[1][2] - 0.002,
            details=f"lip aabb={lip_front}, pan aabb={seat_pan}",
        )

    # --- Backrests rise above seats ---
    for idx in range(2):
        br = ctx.part_element_world_aabb(beam, elem=f"backrest_{idx}")
        seat_pan = ctx.part_element_world_aabb(beam, elem=f"seat_pan_{idx}")
        ctx.check(
            f"backrest_{idx} rises above seat",
            br is not None and seat_pan is not None
            and br[1][2] > seat_pan[1][2] + 0.08,
            details=f"backrest aabb={br}, pan aabb={seat_pan}",
        )

    # --- Handle grips (rounded spheres at top) ---
    for idx in range(2):
        grip = ctx.part_element_world_aabb(beam, elem=f"handle_grip_{idx}")
        handle = ctx.part_element_world_aabb(beam, elem=f"handle_{idx}")
        ctx.check(
            f"handle_grip_{idx} at top of handle",
            grip is not None and handle is not None
            and grip[1][2] >= handle[1][2] - 0.01
            and grip[0][2] > handle[0][2] + 0.10,
            details=f"grip aabb={grip}, handle aabb={handle}",
        )

    # --- Bumpers hang below beam ---
    for idx in range(2):
        bp = object_model.get_part(f"bumper_{idx}")
        bshell = ctx.part_element_world_aabb(bp, elem=f"bumper_shell_{idx}")
        ctx.check(
            f"bumper_{idx} hangs below beam",
            bshell is not None and bar_box is not None
            and bshell[0][2] < bar_box[0][2],
            details=f"bumper aabb={bshell}",
        )

    # --- Decisive rocking pose ---
    rest_b0 = ctx.part_world_aabb(bumper_0)
    with ctx.pose({pivot: TILT}):
        down_b0 = ctx.part_world_aabb(bumper_0)
        up_b1 = ctx.part_world_aabb(bumper_1)
        ctx.check(
            "positive rock lowers +X end",
            rest_b0 is not None and down_b0 is not None
            and down_b0[0][2] < rest_b0[0][2] - 0.15,
            details=f"rest={rest_b0}, tilted={down_b0}",
        )
        ctx.check(
            "positive rock raises -X end",
            up_b1 is not None and up_b1[0][2] > 0.60,
            details=f"raised bumper aabb={up_b1}",
        )

    # --- Bumper compression pose ---
    rest_bump0 = ctx.part_world_aabb(bumper_0)
    with ctx.pose({bump_j0: BUMPER_COMPRESS}):
        compressed = ctx.part_world_aabb(bumper_0)
        ctx.check(
            "bumper_0 compresses upward on prismatic joint",
            compressed is not None and rest_bump0 is not None
            and compressed[0][2] > rest_bump0[0][2] + 0.010,
            details=f"rest={rest_bump0}, compressed={compressed}",
        )

    return ctx.report()


object_model = build_object_model()
