from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    CylinderGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ----------------------------------------------------------------------------
# Heavy commercial steel beam playground seesaw with rubber bumpers.
#
# Layout (world frame, Z up, base centered on the origin):
# - Steel pedestal base (~0.70 m tall) with two side bearing plates bracketing
#   the beam, cross braces, wide base plate, and foot pads.
# - Single heavy box-section steel beam (~2.60 m long, 100 x 80 mm section)
#   pivoting between the bearing plates on a horizontal axle.
# - Two molded bucket seats with raised lip rings on the beam ends.
# - Two rubber bumpers under the beam near each end.
# - Two backrest panels, each on its own small revolute tilt joint.
# - Visible zinc axle caps on the outside of each bearing plate.
# - Articulation: beam_pivot REVOLUTE +/-18 deg;
#   backrest_0_tilt, backrest_1_tilt REVOLUTE 0..15 deg.
# ----------------------------------------------------------------------------

# --- Dimensions (meters) ---
BEAM_LEN = 2.60
BEAM_W = 0.100
BEAM_H = 0.080

PIVOT_Z = 0.70

# Side bearing plates
PLATE_T = 0.012
PLATE_H = PIVOT_Z + 0.040
PLATE_W = 0.140
PLATE_GAP = 0.006  # clearance between beam side and plate inner face
PLATE_Y = BEAM_W / 2.0 + PLATE_GAP + PLATE_T / 2.0  # plate center Y

SEAT_R = 0.15
SEAT_H = 0.025
LIP_H = 0.012
LIP_W = 0.012

BUMPER_R = 0.040
BUMPER_L = 0.060

CAP_R = 0.038
CAP_T = 0.016

BACK_W = 0.26
BACK_H = 0.25
BACK_T = 0.012

SEAT_X = 1.05
SLEEVE_R = 0.035
SLEEVE_LEN = 0.100  # fits between the bearing plates

TILT = math.radians(18.0)
BACK_TILT = math.radians(15.0)

# --- Materials ---
STEEL = Material("structural_steel", rgba=(0.45, 0.47, 0.50, 1.0))
BEAM_PAINT = Material("beam_paint", rgba=(0.28, 0.31, 0.36, 1.0))
RUBBER = Material("rubber_bumper", rgba=(0.10, 0.10, 0.12, 1.0))
SEAT_MAT = Material("molded_seat", rgba=(0.20, 0.55, 0.28, 1.0))
BACK_MAT = Material("backrest_panel", rgba=(0.18, 0.48, 0.24, 1.0))
CAP_MAT = Material("zinc_cap", rgba=(0.62, 0.64, 0.60, 1.0))
AXLE_MAT = Material("axle_steel", rgba=(0.55, 0.55, 0.58, 1.0))


# ---------------------------------------------------------------------------
# CadQuery geometry builders
# ---------------------------------------------------------------------------


def _build_base_cq() -> cq.Workplane:
    """Pedestal base: two side bearing plates, cross braces, base plate, feet."""
    inner_y = PLATE_Y - PLATE_T / 2.0  # inner face of plate

    # Side bearing plates
    p1 = (
        cq.Workplane("XY")
        .box(PLATE_W, PLATE_T, PLATE_H)
        .edges("|Z")
        .fillet(0.003)
        .translate((0.0, PLATE_Y, PLATE_H / 2.0))
    )
    p2 = (
        cq.Workplane("XY")
        .box(PLATE_W, PLATE_T, PLATE_H)
        .edges("|Z")
        .fillet(0.003)
        .translate((0.0, -PLATE_Y, PLATE_H / 2.0))
    )

    # Upper cross brace (between plates)
    b1 = (
        cq.Workplane("XY")
        .box(PLATE_W - 0.02, 2.0 * inner_y, 0.022)
        .translate((0.0, 0.0, 0.38))
    )
    # Lower cross brace
    b2 = (
        cq.Workplane("XY")
        .box(PLATE_W - 0.02, 2.0 * inner_y, 0.022)
        .translate((0.0, 0.0, 0.16))
    )

    # Wide base plate for stability
    bp_w = PLATE_W + 0.18
    bp_d = 2.0 * PLATE_Y + PLATE_T + 0.22
    bp = (
        cq.Workplane("XY")
        .box(bp_w, bp_d, 0.014)
        .edges("|Z")
        .fillet(0.006)
        .translate((0.0, 0.0, 0.007))
    )

    # Foot pads at four corners
    foot_size = (0.08, 0.08, 0.008)
    fx = PLATE_W / 2.0 + 0.05
    fy = PLATE_Y + 0.07
    result = p1.union(p2).union(b1).union(b2).union(bp)
    for sx in (1.0, -1.0):
        for sy in (1.0, -1.0):
            foot = (
                cq.Workplane("XY")
                .box(*foot_size)
                .translate((sx * fx, sy * fy, 0.004))
            )
            result = result.union(foot)

    return result


def _build_beam_cq() -> cq.Workplane:
    """Heavy box-section beam with slightly rounded vertical edges."""
    return (
        cq.Workplane("XY")
        .box(BEAM_LEN, BEAM_W, BEAM_H)
        .edges("|Z")
        .fillet(0.004)
    )


def _build_seat_cq() -> cq.Workplane:
    """Molded seat: flat disc pan with a raised annular lip ring on top."""
    seat = cq.Workplane("XY").circle(SEAT_R).extrude(SEAT_H)
    lip = (
        cq.Workplane("XY")
        .circle(SEAT_R + 0.003)
        .circle(SEAT_R - LIP_W)
        .extrude(LIP_H)
        .translate((0.0, 0.0, SEAT_H))
    )
    return seat.union(lip)


def _build_backrest_cq() -> cq.Workplane:
    """Backrest panel with bottom edge at origin (hinge line)."""
    return (
        cq.Workplane("XY")
        .box(BACK_W, BACK_T, BACK_H)
        .edges("|Z")
        .fillet(0.003)
        .translate((0.0, 0.0, BACK_H / 2.0))
    )


# ---------------------------------------------------------------------------
# Model assembly
# ---------------------------------------------------------------------------


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_beam_seesaw")

    # ---- Base (root, static) ------------------------------------------------
    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_cq(), "base_pedestal"),
        material=STEEL,
        name="pedestal",
    )

    # Axle tube through both bearing plates (along Y)
    axle_len = 2.0 * (PLATE_Y + PLATE_T / 2.0 + 0.022)
    axle_geom = CylinderGeometry(0.025, axle_len, radial_segments=20)
    axle_geom.rotate_x(math.pi / 2.0)
    axle_geom.translate(0.0, 0.0, PIVOT_Z)
    base.visual(
        mesh_from_geometry(axle_geom, "axle_tube"),
        material=AXLE_MAT,
        name="axle",
    )

    # Visible axle caps on the outside of each bearing plate
    cap_y = PLATE_Y + PLATE_T / 2.0 + 0.003 + CAP_T / 2.0
    for idx, sy in enumerate((-1.0, 1.0)):
        cap_geom = CylinderGeometry(CAP_R, CAP_T, radial_segments=24)
        cap_geom.rotate_x(math.pi / 2.0)
        cap_geom.translate(0.0, sy * cap_y, PIVOT_Z)
        base.visual(
            mesh_from_geometry(cap_geom, f"axle_cap_{idx}"),
            material=CAP_MAT,
            name=f"axle_cap_{idx}",
        )

    # ---- Beam ---------------------------------------------------------------
    beam = model.part("beam")
    beam.visual(
        mesh_from_cadquery(_build_beam_cq(), "beam_box"),
        material=BEAM_PAINT,
        name="beam_body",
    )

    # Pivot sleeve bearing at center (shorter than plate gap to avoid overlap)
    sleeve_geom = CylinderGeometry(SLEEVE_R, SLEEVE_LEN, radial_segments=22)
    sleeve_geom.rotate_x(math.pi / 2.0)
    beam.visual(
        mesh_from_geometry(sleeve_geom, "pivot_sleeve"),
        material=BEAM_PAINT,
        name="sleeve",
    )

    # Rubber bumpers under beam near each end
    for idx, sx in enumerate((1.0, -1.0)):
        bgeom = CylinderGeometry(BUMPER_R, BUMPER_L, radial_segments=16)
        bgeom.translate(
            sx * (BEAM_LEN / 2.0 - 0.18),
            0.0,
            -BEAM_H / 2.0 - BUMPER_L / 2.0,
        )
        beam.visual(
            mesh_from_geometry(bgeom, f"bumper_{idx}"),
            material=RUBBER,
            name=f"bumper_{idx}",
        )

    # Molded seats on top of beam at each end
    for idx, sx in enumerate((1.0, -1.0)):
        beam.visual(
            mesh_from_cadquery(_build_seat_cq(), f"seat_{idx}"),
            origin=Origin(xyz=(sx * SEAT_X, 0.0, BEAM_H / 2.0)),
            material=SEAT_MAT,
            name=f"seat_{idx}",
        )

    # ---- Backrests ----------------------------------------------------------
    backrest_0 = model.part("backrest_0")
    backrest_0.visual(
        mesh_from_cadquery(_build_backrest_cq(), "backrest_panel_0"),
        material=BACK_MAT,
        name="panel",
    )

    backrest_1 = model.part("backrest_1")
    backrest_1.visual(
        mesh_from_cadquery(_build_backrest_cq(), "backrest_panel_1"),
        material=BACK_MAT,
        name="panel",
    )

    # ---- Articulations ------------------------------------------------------

    # Beam pivot: REVOLUTE, horizontal axis along Y, +/-18 deg
    model.articulation(
        "beam_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=200.0, velocity=2.0, lower=-TILT, upper=TILT
        ),
    )

    # Backrest hinge Z: at seat disc surface so backrest contacts the seat
    hinge_z = BEAM_H / 2.0 + SEAT_H

    # Backrest 0 at +X end: hinge at outboard edge of seat
    hinge_x_0 = SEAT_X + SEAT_R * 0.70
    model.articulation(
        "backrest_0_tilt",
        ArticulationType.REVOLUTE,
        parent=beam,
        child=backrest_0,
        origin=Origin(xyz=(hinge_x_0, 0.0, hinge_z)),
        axis=(0.0, 1.0, 0.0),  # positive q -> top tilts toward +X (backward)
        motion_limits=MotionLimits(
            effort=10.0, velocity=1.0, lower=0.0, upper=BACK_TILT
        ),
    )

    # Backrest 1 at -X end
    hinge_x_1 = -(SEAT_X + SEAT_R * 0.70)
    model.articulation(
        "backrest_1_tilt",
        ArticulationType.REVOLUTE,
        parent=beam,
        child=backrest_1,
        origin=Origin(xyz=(hinge_x_1, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),  # positive q -> top tilts toward -X (backward)
        motion_limits=MotionLimits(
            effort=10.0, velocity=1.0, lower=0.0, upper=BACK_TILT
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    beam = object_model.get_part("beam")
    backrest_0 = object_model.get_part("backrest_0")
    backrest_1 = object_model.get_part("backrest_1")
    beam_pivot = object_model.get_articulation("beam_pivot")
    br_0_tilt = object_model.get_articulation("backrest_0_tilt")
    br_1_tilt = object_model.get_articulation("backrest_1_tilt")

    # --- Allowances: axle bearing interface ---
    ctx.allow_overlap(
        beam,
        base,
        elem_a="sleeve",
        elem_b="axle",
        reason="Beam pivot sleeve intentionally wraps the axle tube as a bearing.",
    )
    ctx.allow_overlap(
        base,
        beam,
        elem_a="axle",
        elem_b="beam_body",
        reason="Axle passes through the beam body center to reach the pivot sleeve bearing.",
    )
    ctx.expect_contact(
        beam,
        base,
        elem_a="sleeve",
        elem_b="axle",
        name="beam sleeve rides on the axle",
    )

    # --- Allowances: backrest hinge seating ---
    ctx.allow_overlap(
        backrest_0,
        beam,
        elem_a="panel",
        elem_b="seat_0",
        reason="Backrest panel bottom seats against the molded seat lip at the hinge point.",
    )
    ctx.allow_overlap(
        backrest_1,
        beam,
        elem_a="panel",
        elem_b="seat_1",
        reason="Backrest panel bottom seats against the molded seat lip at the hinge point.",
    )
    ctx.expect_contact(
        backrest_0,
        beam,
        elem_a="panel",
        elem_b="seat_0",
        name="backrest 0 seats on the molded seat",
    )
    ctx.expect_contact(
        backrest_1,
        beam,
        elem_a="panel",
        elem_b="seat_1",
        name="backrest 1 seats on the molded seat",
    )

    # --- Base geometry ---
    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "base is a pedestal about 0.7 m tall",
        base_aabb is not None and 0.65 <= base_aabb[1][2] <= 0.82,
        details=f"base aabb={base_aabb}",
    )
    ctx.check(
        "base feet rest on the ground",
        base_aabb is not None and -0.01 <= base_aabb[0][2] <= 0.015,
        details=f"base min z={base_aabb[0][2] if base_aabb else None}",
    )

    # --- Axle caps visible at bracket ---
    for idx in (0, 1):
        cap_aabb = ctx.part_element_world_aabb(base, elem=f"axle_cap_{idx}")
        ctx.check(
            f"axle cap {idx} is visible at the support bracket",
            cap_aabb is not None,
            details=f"cap aabb={cap_aabb}",
        )
        if cap_aabb is not None:
            cap_cz = (cap_aabb[0][2] + cap_aabb[1][2]) / 2.0
            ctx.check(
                f"axle cap {idx} is near pivot height",
                abs(cap_cz - PIVOT_Z) < 0.02,
                details=f"cap center z={cap_cz:.4f}",
            )

    # --- Molded seats with raised lips ---
    for idx in (0, 1):
        seat_aabb = ctx.part_element_world_aabb(beam, elem=f"seat_{idx}")
        ctx.check(
            f"molded seat {idx} exists on beam",
            seat_aabb is not None,
            details=f"seat aabb={seat_aabb}",
        )
        if seat_aabb is not None:
            seat_h = seat_aabb[1][2] - seat_aabb[0][2]
            ctx.check(
                f"seat {idx} has raised lip (total height > base thickness)",
                seat_h > SEAT_H + LIP_H * 0.5,
                details=f"seat height={seat_h:.4f}, expected > {SEAT_H + LIP_H * 0.5:.4f}",
            )

    # --- Rubber bumpers ---
    for idx in (0, 1):
        bump_aabb = ctx.part_element_world_aabb(beam, elem=f"bumper_{idx}")
        ctx.check(
            f"rubber bumper {idx} exists under the beam end",
            bump_aabb is not None,
            details=f"bumper aabb={bump_aabb}",
        )

    # --- Backrests and their revolute tilt joints ---
    for br, tilt in [
        (backrest_0, br_0_tilt),
        (backrest_1, br_1_tilt),
    ]:
        br_aabb = ctx.part_world_aabb(br)
        ctx.check(
            f"{br.name} backrest panel exists",
            br_aabb is not None,
            details=f"backrest aabb={br_aabb}",
        )
        lim = tilt.motion_limits
        ctx.check(
            f"{tilt.name} is revolute with a tilt range",
            tilt.articulation_type == ArticulationType.REVOLUTE
            and lim is not None
            and lim.upper is not None
            and lim.upper > 0.0,
            details=f"type={tilt.articulation_type}, upper={lim.upper if lim else None}",
        )

    # --- Beam pivot is non-fixed ---
    pivot_lim = beam_pivot.motion_limits
    ctx.check(
        "beam pivot is a non-fixed revolute joint",
        beam_pivot.articulation_type == ArticulationType.REVOLUTE
        and pivot_lim is not None
        and pivot_lim.upper is not None
        and pivot_lim.upper > 0.0,
        details=f"type={beam_pivot.articulation_type}, "
        f"limits=({pivot_lim.lower if pivot_lim else None}, {pivot_lim.upper if pivot_lim else None})",
    )

    # --- Beam rocking pose check ---
    rest_seat0 = ctx.part_element_world_aabb(beam, elem="seat_0")
    rest_seat1 = ctx.part_element_world_aabb(beam, elem="seat_1")
    with ctx.pose({beam_pivot: TILT}):
        tilt_seat0 = ctx.part_element_world_aabb(beam, elem="seat_0")
        tilt_seat1 = ctx.part_element_world_aabb(beam, elem="seat_1")
        beam_aabb = ctx.part_world_aabb(beam)
        ctx.check(
            "beam rocks: seat 0 drops when tilted positive",
            rest_seat0 is not None
            and tilt_seat0 is not None
            and tilt_seat0[0][2] < rest_seat0[0][2] - 0.20,
            details=f"seat0 rest min_z={rest_seat0[0][2] if rest_seat0 else None}, "
            f"tilt min_z={tilt_seat0[0][2] if tilt_seat0 else None}",
        )
        ctx.check(
            "beam rocks: seat 1 rises when tilted positive",
            rest_seat1 is not None
            and tilt_seat1 is not None
            and tilt_seat1[0][2] > rest_seat1[0][2] + 0.20,
            details=f"seat1 rest min_z={rest_seat1[0][2] if rest_seat1 else None}, "
            f"tilt min_z={tilt_seat1[0][2] if tilt_seat1 else None}",
        )
        ctx.check(
            "tilted beam stays above ground",
            beam_aabb is not None and beam_aabb[0][2] > -0.05,
            details=f"beam aabb={beam_aabb}",
        )
        ctx.expect_contact(
            beam,
            base,
            elem_a="sleeve",
            elem_b="axle",
            name="tilted beam sleeve stays on the axle",
        )

    # --- Backrest tilt pose checks ---
    # Backrest 0 (at +X end): positive tilt moves top toward +X
    rest_br0 = ctx.part_world_aabb(backrest_0)
    with ctx.pose({br_0_tilt: BACK_TILT}):
        tilt_br0 = ctx.part_world_aabb(backrest_0)
        ctx.check(
            "backrest 0 tilts backward: top moves in +X",
            rest_br0 is not None
            and tilt_br0 is not None
            and tilt_br0[1][0] > rest_br0[1][0] + 0.01,
            details=f"rest max_x={rest_br0[1][0]:.4f}, tilt max_x={tilt_br0[1][0]:.4f}",
        )

    # Backrest 1 (at -X end): positive tilt moves top toward -X
    rest_br1 = ctx.part_world_aabb(backrest_1)
    with ctx.pose({br_1_tilt: BACK_TILT}):
        tilt_br1 = ctx.part_world_aabb(backrest_1)
        ctx.check(
            "backrest 1 tilts backward: top moves in -X",
            rest_br1 is not None
            and tilt_br1 is not None
            and tilt_br1[0][0] < rest_br1[0][0] - 0.01,
            details=f"rest min_x={rest_br1[0][0]:.4f}, tilt min_x={tilt_br1[0][0]:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
