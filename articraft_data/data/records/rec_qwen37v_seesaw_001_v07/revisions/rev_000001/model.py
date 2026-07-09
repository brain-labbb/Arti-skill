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
# Low inclusive playground seesaw with backrest seats
#
# World frame: beam runs along X, pivot axis along Y, Z up.
# - Two bent galvanized-steel tube arches (~50 mm dia) cross side by side and
#   form an A-shaped saddle; the apex carries a horizontal pivot axle bolt.
#   Lower pivot height (~0.50 m) for inclusive access.
# - The rocking beam is a 3.0 m mustard-yellow steel bar (80 x 40 mm) with a
#   pivot sleeve + triangular gusset at center.
# - Each end has a molded bucket seat with raised lips and an integrated
#   backrest, plus a pivoting handlebar on a revolute joint.
# - Curved rubber tire-section bumpers under each tip.
# - Visible axle caps at the support bracket.
# - Main revolute joint at the apex, axis (0, 1, 0), +/- 20 degrees.
# - Each handlebar has its own revolute joint (slight forward/back pivot).
# ---------------------------------------------------------------------------

BEAM_HALF = 1.50  # 3.0 m beam
BEAM_W = 0.08
BEAM_T = 0.04
PIVOT_Z = 0.50  # lower inclusive pivot height

ARCH_FOOT_X = 0.58
ARCH_FOOT_Y = 0.34
ARCH_APEX_Y = 0.05
ARCH_FOOT_Z = 0.025
TUBE_R = 0.025  # ~50 mm diameter bent tube

AXLE_R = 0.016
AXLE_LEN = 0.24

# Beam-local frame: origin at the axle center; the bar bottom sits above.
BAR_BOT = 0.05
BAR_CTR = BAR_BOT + BEAM_T / 2.0
BAR_TOP = BAR_BOT + BEAM_T

SEAT_X = 1.28
HANDLE_X = 1.02
BUMPER_X = 1.42
TILT = math.radians(20.0)
HANDLE_TILT = math.radians(12.0)  # handlebar pivot limit


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
    """Inverted-U grab-handle rod centerline with a cross-bar at the base.

    The cross-bar connects both legs so the mesh is one connected component.
    Handle origin is at beam top surface center; rod extends upward from there.
    """
    half_w = 0.035
    leg_bot = -0.006  # legs start slightly below origin for boss embedding
    arc_z = 0.22
    pts: list[tuple[float, float, float]] = [
        # Left leg bottom -> cross-bar start
        (x, -half_w, leg_bot),
        (x, -half_w, 0.0),
        # Cross-bar: connect to right leg at base
        (x, 0.0, 0.0),
        (x, half_w, 0.0),
        # Right leg up
        (x, half_w, 0.150),
    ]
    for k in range(7):  # semicircular top bend
        a = math.pi * k / 6.0
        pts.append((x, half_w * math.cos(a), arc_z + half_w * math.sin(a)))
    # Left leg down from arc
    pts.append((x, -half_w, 0.150))
    pts.append((x, -half_w, leg_bot))
    return pts


def _bumper_geometry(x: float, index: int):
    """Curved tire-section bumper: half-annulus shell extruded across the beam."""
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


def _molded_seat_geometry(index: int):
    """Molded bucket seat with raised lips on 3 sides (left, right, back).
    
    The seat pan is a shallow dish. The profile is extruded along Y (across beam).
    Returns a mesh positioned at the seat location on the beam.
    """
    # Seat dimensions
    seat_dx = 0.28  # length along beam
    seat_dy = 0.22  # width across beam
    pan_thick = 0.012  # seat pan thickness
    lip_h = 0.030  # raised lip height
    lip_w = 0.014  # lip wall thickness

    # Build seat pan as a flat extrusion
    # Profile in XZ plane (seat cross-section along beam length)
    # Bottom surface with raised lips on both ends
    profile = [
        (-seat_dx / 2, 0.0),
        (-seat_dx / 2, lip_h),
        (-seat_dx / 2 + lip_w, lip_h),
        (-seat_dx / 2 + lip_w, pan_thick),
        (seat_dx / 2 - lip_w, pan_thick),
        (seat_dx / 2 - lip_w, lip_h),
        (seat_dx / 2, lip_h),
        (seat_dx / 2, 0.0),
    ]
    geom = ExtrudeGeometry(profile, seat_dy, cap=True, center=True)
    # ExtrudeGeometry extrudes along local Z by default; rotate to extrude along Y
    geom.rotate_x(math.pi / 2.0)
    # Position on beam
    side = 1.0 if index == 0 else -1.0
    geom.translate(side * SEAT_X, 0.0, BAR_TOP)
    return mesh_from_geometry(geom, f"molded_seat_{index}")


def _backrest_geometry(index: int):
    """Backrest plate behind the molded seat, slightly curved.
    
    A vertical plate at the outer end of the seat (away from beam center),
    angled slightly backward.
    """
    # Backrest profile in XZ: rises from seat top, angles back slightly
    back_h = 0.28  # backrest height
    back_thick = 0.012
    back_w = 0.20  # width across beam

    profile = [
        (0.0, 0.0),
        (0.0, back_h),
        (back_thick, back_h),
        (back_thick, 0.0),
    ]
    geom = ExtrudeGeometry(profile, back_w, cap=True, center=True)
    geom.rotate_x(math.pi / 2.0)
    # Position: at outer edge of seat, rising from lip top
    side = 1.0 if index == 0 else -1.0
    seat_outer_x = side * (SEAT_X + 0.14 - 0.007)
    geom.translate(seat_outer_x, 0.0, BAR_TOP + 0.030)
    return mesh_from_geometry(geom, f"backrest_{index}")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inclusive_playground_seesaw")

    galvanized = model.material("weathered_galvanized", rgba=(0.55, 0.58, 0.56, 1.0))
    rust = model.material("rust_steel", rgba=(0.42, 0.25, 0.13, 1.0))
    mustard = model.material("rusty_mustard_paint", rgba=(0.74, 0.53, 0.12, 1.0))
    pale_steel = model.material("pale_weathered_steel", rgba=(0.70, 0.66, 0.58, 1.0))
    molded_plastic = model.material("molded_hdpe", rgba=(0.18, 0.42, 0.55, 1.0))
    rubber = model.material("black_rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    axle_cap_mat = model.material("zinc_cap", rgba=(0.62, 0.64, 0.60, 1.0))

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

    # Visible axle caps at the support bracket (larger discs at each arch face)
    for i, side in enumerate((1.0, -1.0)):
        base.visual(
            Cylinder(radius=0.032, length=0.010),
            origin=Origin(
                xyz=(0.0, side * (AXLE_LEN / 2.0 + 0.002), PIVOT_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=axle_cap_mat,
            name=f"axle_cap_{i}",
        )

    # Small hex nuts behind the caps
    for i, side in enumerate((1.0, -1.0)):
        base.visual(
            Cylinder(radius=0.022, length=0.008),
            origin=Origin(
                xyz=(0.0, side * (AXLE_LEN / 2.0 - 0.008), PIVOT_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=rust,
            name=f"axle_nut_{i}",
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

    beam.visual(
        Box((2.0 * BEAM_HALF, BEAM_W, BEAM_T)),
        origin=Origin(xyz=(0.0, 0.0, BAR_CTR)),
        material=mustard,
        name="beam_bar",
    )

    # Rust streak patches wrapping the painted bar
    for i, px in enumerate((-0.85, -0.30, 0.45, 0.75)):
        beam.visual(
            Box((0.16, BEAM_W + 0.004, 0.012)),
            origin=Origin(xyz=(px, 0.0, BAR_TOP - 0.004)),
            material=rust,
            name=f"rust_patch_{i}",
        )

    # Molded seats with raised lips + backrests (on the beam, not separate parts)
    for i in range(2):
        beam.visual(
            _molded_seat_geometry(i),
            material=molded_plastic,
            name=f"molded_seat_{i}",
        )
        beam.visual(
            _backrest_geometry(i),
            material=molded_plastic,
            name=f"backrest_{i}",
        )

    # Bumpers under each tip
    for i, side in enumerate((1.0, -1.0)):
        beam.visual(
            _bumper_geometry(side * BUMPER_X, i),
            material=rubber,
            name=f"bumper_{i}",
        )

    # -------------------------------------------------------- handlebars ---
    # Each handlebar is a separate part with a revolute joint on the beam.
    for i, side in enumerate((1.0, -1.0)):
        handle = model.part(f"handlebar_{i}")
        # Handle geometry: inverted-U rod, origin at base where it meets beam top
        handle_x_local = 0.0  # handle part frame is at the joint origin
        handle.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _handle_points(handle_x_local),
                    radius=0.009,
                    samples_per_segment=8,
                    radial_segments=16,
                    cap_ends=True,
                ),
                f"handlebar_rod_{i}",
            ),
            material=pale_steel,
            name=f"handle_rod_{i}",
        )
        # Mounting bracket at the base (box encompassing both leg bases)
        handle.visual(
            Box((0.030, 0.090, 0.014)),
            origin=Origin(xyz=(0.0, 0.0, 0.007)),
            material=pale_steel,
            name=f"handle_bracket_{i}",
        )

        # Revolute joint: handlebar pivots slightly forward/backward
        # Joint origin is in beam-local frame at the handle mount position
        # The handle part frame will be at the joint origin at q=0
        model.articulation(
            f"handle_pivot_{i}",
            ArticulationType.REVOLUTE,
            parent=beam,
            child=handle,
            # Joint frame in beam-local coords: at beam top, at handle X position
            origin=Origin(xyz=(side * HANDLE_X, 0.0, BAR_TOP)),
            axis=(0.0, 1.0, 0.0),  # pivot around Y (forward/back lean)
            motion_limits=MotionLimits(
                effort=10.0,
                velocity=2.0,
                lower=-HANDLE_TILT,
                upper=HANDLE_TILT,
            ),
        )

    # -------------------------------------------------------------- joint ---
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
    base = object_model.get_part("arched_base")
    beam = object_model.get_part("beam")
    pivot = object_model.get_articulation("beam_pivot")
    handlebar_0 = object_model.get_part("handlebar_0")
    handlebar_1 = object_model.get_part("handlebar_1")
    handle_pivot_0 = object_model.get_articulation("handle_pivot_0")
    handle_pivot_1 = object_model.get_articulation("handle_pivot_1")

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

    # --- Handlebar rod legs embed into beam bar (intentional mounting) ---
    for i in range(2):
        hb = object_model.get_part(f"handlebar_{i}")
        ctx.allow_overlap(
            beam,
            hb,
            elem_a="beam_bar",
            elem_b=f"handle_rod_{i}",
            reason=f"Handlebar {i} rod legs are intentionally embedded into the beam bar top for mounting.",
        )
        ctx.expect_contact(
            beam,
            hb,
            elem_a="beam_bar",
            elem_b=f"handle_bracket_{i}",
            name=f"handlebar_{i} bracket contacts beam bar top",
        )

    # --- Beam bar clearance over arch saddle ---
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

    # --- Main pivot joint: horizontal Y axis, +/- 20 degree limits ---
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
        "pivot axle sits at low inclusive height (~0.50 m)",
        axle_box is not None and 0.42 <= (axle_box[0][2] + axle_box[1][2]) / 2.0 <= 0.56,
        details=f"axle aabb={axle_box}",
    )

    # --- Base feet on ground ---
    base_box = ctx.part_world_aabb(base)
    ctx.check(
        "arched base feet rest on the ground",
        base_box is not None and -0.01 <= base_box[0][2] <= 0.02,
        details=f"base aabb={base_box}",
    )

    # --- Beam is about 3.0 m long ---
    bar_box = ctx.part_element_world_aabb(beam, elem="beam_bar")
    ctx.check(
        "beam is about 3.0 m long",
        bar_box is not None and abs((bar_box[1][0] - bar_box[0][0]) - 3.0) < 0.02,
        details=f"bar aabb={bar_box}",
    )

    # --- Molded seats with raised lips exist and sit on beam ---
    for i in range(2):
        seat_box = ctx.part_element_world_aabb(beam, elem=f"molded_seat_{i}")
        ctx.check(
            f"molded_seat_{i} sits on top of beam bar",
            seat_box is not None
            and bar_box is not None
            and seat_box[0][2] >= bar_box[1][2] - 0.005
            and seat_box[1][2] > bar_box[1][2] + 0.015,
            details=f"seat aabb={seat_box}",
        )

    # --- Backrests exist and rise above seats ---
    for i in range(2):
        back_box = ctx.part_element_world_aabb(beam, elem=f"backrest_{i}")
        seat_box = ctx.part_element_world_aabb(beam, elem=f"molded_seat_{i}")
        ctx.check(
            f"backrest_{i} rises above the seat",
            back_box is not None
            and seat_box is not None
            and back_box[1][2] > seat_box[1][2] + 0.10,
            details=f"backrest aabb={back_box}, seat aabb={seat_box}",
        )

    # --- Axle caps visible at the support bracket ---
    for i in range(2):
        cap_box = ctx.part_element_world_aabb(base, elem=f"axle_cap_{i}")
        ctx.check(
            f"axle_cap_{i} is present at the bracket",
            cap_box is not None
            and abs((cap_box[0][2] + cap_box[1][2]) / 2.0 - PIVOT_Z) < 0.02,
            details=f"cap aabb={cap_box}",
        )

    # --- Handlebar joints: revolute, non-fixed, with correct limits ---
    for i, hp in enumerate((handle_pivot_0, handle_pivot_1)):
        ctx.check(
            f"handle_pivot_{i} is revolute",
            hp.articulation_type == ArticulationType.REVOLUTE,
            details=f"type={hp.articulation_type}",
        )
        hlim = hp.motion_limits
        ctx.check(
            f"handle_pivot_{i} has non-trivial motion limits",
            hlim is not None
            and hlim.lower is not None
            and hlim.upper is not None
            and hlim.upper > hlim.lower,
            details=f"limits=({hlim.lower if hlim else None}, {hlim.upper if hlim else None})",
        )

    # --- Handlebar geometry: stands above beam ---
    for i in range(2):
        handle_box = ctx.part_world_aabb(object_model.get_part(f"handlebar_{i}"))
        ctx.check(
            f"handlebar_{i} extends above beam",
            handle_box is not None
            and bar_box is not None
            and handle_box[1][2] > bar_box[1][2] + 0.15
            and handle_box[0][2] < bar_box[1][2] + 0.02,
            details=f"handle aabb={handle_box}",
        )

    # --- Handle pivot test: handlebar actually moves when posed ---
    rest_h0 = ctx.part_world_aabb(handlebar_0)
    with ctx.pose({handle_pivot_0: HANDLE_TILT}):
        tilted_h0 = ctx.part_world_aabb(handlebar_0)
        ctx.check(
            "handlebar_0 moves when handle_pivot_0 is posed",
            rest_h0 is not None
            and tilted_h0 is not None
            and abs(tilted_h0[1][0] - rest_h0[1][0]) > 0.005,
            details=f"rest={rest_h0}, tilted={tilted_h0}",
        )

    # --- Bumpers hang below beam tips ---
    for i in range(2):
        bumper = ctx.part_element_world_aabb(beam, elem=f"bumper_{i}")
        ctx.check(
            f"bumper_{i} hangs below the beam tip",
            bumper is not None
            and bar_box is not None
            and bumper[0][2] < bar_box[0][2]
            and min(abs(bumper[0][0]), abs(bumper[1][0])) > 1.2,
            details=f"bumper aabb={bumper}",
        )

    # --- Decisive pose checks: rocking alternately lowers each end ---
    rest_b0 = ctx.part_element_world_aabb(beam, elem="bumper_0")
    with ctx.pose({pivot: TILT}):
        down_b0 = ctx.part_element_world_aabb(beam, elem="bumper_0")
        up_b1 = ctx.part_element_world_aabb(beam, elem="bumper_1")
        ctx.check(
            "positive rock lowers the +X end near the ground",
            rest_b0 is not None
            and down_b0 is not None
            and down_b0[0][2] < rest_b0[0][2] - 0.25
            and down_b0[1][2] < 0.15,
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
            "negative rock lowers the -X end near the ground",
            down_b1 is not None and down_b1[1][2] < 0.15 and down_b1[0][2] > -0.05,
            details=f"tilted bumper aabb={down_b1}",
        )

    return ctx.report()


object_model = build_object_model()
