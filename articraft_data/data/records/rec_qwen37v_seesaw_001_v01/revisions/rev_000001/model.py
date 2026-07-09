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
# Classic playground seesaw variant
#
# World frame: beam runs along X, pivot axis along Y, Z up.
# - Round cylindrical support legs form an A-frame pedestal; two angled legs
#   spread outward in Y from the apex bracket down to ground-level feet, tied
#   by a horizontal cross-bar.
# - The rocking beam is a 3.0 m plank (~80 mm wide, 40 mm thick) painted in
#   a bright playground color. Each end carries a molded seat with raised
#   perimeter lips, an inverted-U grab handle, and a curved rubber bumper
#   underneath.
# - A central pivot bracket at the apex carries the horizontal axle with
#   visible axle caps on the outside.
# - Single revolute joint, axis (0, 1, 0), +/- 20 degrees.
#   Positive q lowers the +X end (right-hand rule about +Y).
# ---------------------------------------------------------------------------

BEAM_HALF = 1.50  # 3.0 m beam
BEAM_W = 0.08
BEAM_T = 0.04
PIVOT_Z = 0.76  # axle height (~0.8 m at the pivot)

# Support leg geometry
LEG_FOOT_Y = 0.40  # feet spread outward in Y
LEG_TUBE_R = 0.025  # ~50 mm diameter round tube legs
CROSSBAR_Y = 0.36  # cross-bar position along legs (near ground)

AXLE_R = 0.016
AXLE_LEN = 0.24
AXLE_CAP_R = 0.030  # visible axle cap radius
AXLE_CAP_LEN = 0.018  # axle cap thickness

# Beam-local frame: origin at the axle center; the bar bottom sits above.
BAR_BOT = 0.05
BAR_CTR = BAR_BOT + BEAM_T / 2.0  # 0.07
BAR_TOP = BAR_BOT + BEAM_T  # 0.09

SEAT_X = 1.28
HANDLE_X = 1.02
BUMPER_X = 1.42
TILT = math.radians(20.0)

# Molded seat dimensions
SEAT_PAN_W = 0.26  # seat width (along beam Y)
SEAT_PAN_L = 0.30  # seat length (along beam X)
SEAT_PAN_T = 0.012  # seat pan thickness
SEAT_LIP_H = 0.022  # lip wall height above seat pan top
SEAT_LIP_T = 0.008  # lip wall thickness


def _leg_points(side: float) -> list[tuple[float, float, float]]:
    """Centerline of one angled round-tube support leg.

    Each leg runs from ground at y = side * LEG_FOOT_Y up to the bracket plate
    at y = side * BRACKET_Y, just outside the pivot sleeve zone.
    """
    bracket_y = 0.055  # terminate at the bracket plate, not at center
    return [
        (0.0, side * LEG_FOOT_Y, 0.025),
        (0.0, side * LEG_FOOT_Y * 0.5, PIVOT_Z * 0.5),
        (0.0, side * bracket_y, PIVOT_Z),
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


def _molded_seat_geometry(index: int, seat_x: float):
    """Molded seat pan with raised perimeter lips, positioned at seat_x on beam."""
    from sdk import MeshGeometry

    mesh = MeshGeometry()

    pw = SEAT_PAN_L / 2.0
    pd = SEAT_PAN_W / 2.0
    pt = SEAT_PAN_T

    # Pan vertices (8 corners of a box)
    v = []
    for z in (0.0, pt):
        for y in (-pd, pd):
            for x in (-pw, pw):
                v.append(mesh.add_vertex(x, y, z))

    # Pan faces (12 triangles for 6 faces of the box)
    # Bottom (z=0): v[0..3]
    mesh.add_face(v[0], v[2], v[1])
    mesh.add_face(v[1], v[2], v[3])
    # Top (z=pt): v[4..7]
    mesh.add_face(v[4], v[5], v[6])
    mesh.add_face(v[5], v[7], v[6])
    # Front (x=-pw): v[0],v[1],v[4],v[5]
    mesh.add_face(v[0], v[1], v[5])
    mesh.add_face(v[0], v[5], v[4])
    # Back (x=+pw): v[2],v[3],v[6],v[7]
    mesh.add_face(v[2], v[6], v[3])
    mesh.add_face(v[3], v[6], v[7])
    # Left (y=-pd): v[0],v[2],v[4],v[6]
    mesh.add_face(v[0], v[4], v[2])
    mesh.add_face(v[2], v[4], v[6])
    # Right (y=+pd): v[1],v[3],v[5],v[7]
    mesh.add_face(v[1], v[3], v[5])
    mesh.add_face(v[3], v[7], v[5])

    # Raised lip walls
    lh = SEAT_LIP_H
    lt = SEAT_LIP_T

    def _add_lip_wall(x0: float, x1: float, y0: float, y1: float):
        base_z = pt
        top_z = pt + lh
        bv = []
        for z in (base_z, top_z):
            for y in (y0, y1):
                for x in (x0, x1):
                    bv.append(mesh.add_vertex(x, y, z))
        mesh.add_face(bv[0], bv[2], bv[1])
        mesh.add_face(bv[1], bv[2], bv[3])
        mesh.add_face(bv[4], bv[5], bv[6])
        mesh.add_face(bv[5], bv[7], bv[6])
        mesh.add_face(bv[0], bv[1], bv[5])
        mesh.add_face(bv[0], bv[5], bv[4])
        mesh.add_face(bv[2], bv[6], bv[3])
        mesh.add_face(bv[3], bv[6], bv[7])
        mesh.add_face(bv[0], bv[4], bv[2])
        mesh.add_face(bv[2], bv[4], bv[6])
        mesh.add_face(bv[1], bv[3], bv[5])
        mesh.add_face(bv[3], bv[7], bv[5])

    _add_lip_wall(-pw, -pw + lt, -pd, pd)
    _add_lip_wall(pw - lt, pw, -pd, pd)
    _add_lip_wall(-pw + lt, pw - lt, -pd, -pd + lt)
    _add_lip_wall(-pw + lt, pw - lt, pd - lt, pd)

    mesh.translate(seat_x, 0.0, BAR_TOP)
    return mesh_from_geometry(mesh, f"molded_seat_{index}")


def _gusset_geometry():
    """Triangular gusset plate joining the beam bar to the pivot sleeve."""
    profile = [(-0.10, 0.055), (0.10, 0.055), (0.0, 0.020)]
    geom = ExtrudeGeometry(profile, 0.020, cap=True, center=True)
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, "seesaw_gusset_plate")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="classic_playground_seesaw")

    # Materials
    steel_gray = model.material("galvanized_steel", rgba=(0.62, 0.64, 0.63, 1.0))
    bracket_paint = model.material("painted_bracket", rgba=(0.25, 0.45, 0.65, 1.0))
    plank_color = model.material("playground_plank", rgba=(0.72, 0.20, 0.15, 1.0))
    seat_color = model.material("molded_seat_plastic", rgba=(0.18, 0.55, 0.30, 1.0))
    handle_metal = model.material("chrome_handle", rgba=(0.75, 0.75, 0.78, 1.0))
    rubber = model.material("black_rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    cap_metal = model.material("zinc_cap", rgba=(0.70, 0.72, 0.70, 1.0))
    rust = model.material("rust_accent", rgba=(0.42, 0.25, 0.13, 1.0))

    # --------------------------------------------------------------- base ---
    base = model.part("support_base")

    # Two angled round-tube legs forming an A-frame in the YZ plane
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
                f"support_leg_{i}",
            ),
            material=steel_gray,
            name=f"leg_{i}",
        )

    # Horizontal cross-bar connecting the two feet for stability
    base.visual(
        Cylinder(radius=LEG_TUBE_R, length=2.0 * LEG_FOOT_Y - 0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.04), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_gray,
        name="crossbar",
    )

    # Pivot bracket / saddle at the apex - two vertical plates flanking the beam
    for i, side in enumerate((1.0, -1.0)):
        base.visual(
            Box((0.06, 0.008, 0.10)),
            origin=Origin(xyz=(0.0, side * 0.055, PIVOT_Z - 0.02)),
            material=bracket_paint,
            name=f"bracket_plate_{i}",
        )

    # Pivot axle bolt through the bracket, axis along Y
    base.visual(
        Cylinder(radius=AXLE_R, length=AXLE_LEN),
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rust,
        name="pivot_axle",
    )

    # Visible axle caps on the outside of each bracket plate
    for i, side in enumerate((1.0, -1.0)):
        base.visual(
            Cylinder(radius=AXLE_CAP_R, length=AXLE_CAP_LEN),
            origin=Origin(
                xyz=(0.0, side * (AXLE_LEN / 2.0 + AXLE_CAP_LEN / 2.0 - 0.004), PIVOT_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=cap_metal,
            name=f"axle_cap_{i}",
        )

    # --------------------------------------------------------------- beam ---
    # Beam part frame sits at the axle center so the joint is at its origin.
    beam = model.part("beam")

    # Pivot sleeve (bushing around axle)
    beam.visual(
        Cylinder(radius=0.026, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rust,
        name="pivot_sleeve",
    )
    beam.visual(_gusset_geometry(), material=plank_color, name="gusset_plate")

    # Main plank beam bar
    beam.visual(
        Box((2.0 * BEAM_HALF, BEAM_W, BEAM_T)),
        origin=Origin(xyz=(0.0, 0.0, BAR_CTR)),
        material=plank_color,
        name="beam_bar",
    )

    # End fittings: index 0 = +X end, index 1 = -X end (symmetric ends).
    for i, side in enumerate((1.0, -1.0)):
        # Molded seat with raised lips
        beam.visual(
            _molded_seat_geometry(i, side * SEAT_X),
            material=seat_color,
            name=f"seat_{i}",
        )

        # Grab handle
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
            material=handle_metal,
            name=f"handle_{i}",
        )

        # Tire-section bumper
        beam.visual(
            _bumper_geometry(side * BUMPER_X, i),
            material=rubber,
            name=f"bumper_{i}",
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
    base = object_model.get_part("support_base")
    beam = object_model.get_part("beam")
    pivot = object_model.get_articulation("beam_pivot")

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

    # Beam bar rides above the support bracket, not embedded in it.
    ctx.expect_gap(
        beam,
        base,
        axis="z",
        positive_elem="beam_bar",
        negative_elem="leg_0",
        min_gap=0.005,
        max_gap=0.08,
        name="beam bar clears the support legs",
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

    # Hero geometry: scale, support height, grounded feet.
    bar_box = ctx.part_element_world_aabb(beam, elem="beam_bar")
    base_box = ctx.part_world_aabb(base)
    axle_box = ctx.part_element_world_aabb(base, elem="pivot_axle")
    ctx.check(
        "beam is about 3.0 m long",
        bar_box is not None and abs((bar_box[1][0] - bar_box[0][0]) - 3.0) < 0.02,
        details=f"bar aabb={bar_box}",
    )
    ctx.check(
        "support base feet rest on the ground",
        base_box is not None and -0.01 <= base_box[0][2] <= 0.03,
        details=f"base aabb={base_box}",
    )
    ctx.check(
        "pivot axle sits about 0.8 m high",
        axle_box is not None and 0.70 <= (axle_box[0][2] + axle_box[1][2]) / 2.0 <= 0.82,
        details=f"axle aabb={axle_box}",
    )

    # Variant-specific: round support legs exist and are grounded.
    for i in range(2):
        leg_box = ctx.part_element_world_aabb(base, elem=f"leg_{i}")
        ctx.check(
            f"round support leg_{i} is present and grounded",
            leg_box is not None and leg_box[0][2] < 0.05 and leg_box[1][2] > 0.60,
            details=f"leg_{i} aabb={leg_box}",
        )

    # Variant-specific: visible axle caps outside the bracket.
    for i in range(2):
        cap_box = ctx.part_element_world_aabb(base, elem=f"axle_cap_{i}")
        ctx.check(
            f"axle_cap_{i} is visible at the bracket",
            cap_box is not None
            and (cap_box[1][1] - cap_box[0][1]) > 0.010
            and 0.70 <= (cap_box[0][2] + cap_box[1][2]) / 2.0 <= 0.82,
            details=f"axle_cap_{i} aabb={cap_box}",
        )

    # Variant-specific: molded seats with raised lips extend above the beam bar.
    for i in range(2):
        seat_box = ctx.part_element_world_aabb(beam, elem=f"seat_{i}")
        ctx.check(
            f"molded seat_{i} sits on beam and has raised lip height",
            seat_box is not None
            and bar_box is not None
            and seat_box[0][2] > bar_box[1][2] - 0.005  # seat bottom at or above bar top
            and (seat_box[1][2] - seat_box[0][2]) > 0.020,  # total height includes lips
            details=f"seat_{i} aabb={seat_box}",
        )

    # Per-end fittings: handle upright and bumper hanging below.
    for i in range(2):
        handle = ctx.part_element_world_aabb(beam, elem=f"handle_{i}")
        bumper = ctx.part_element_world_aabb(beam, elem=f"bumper_{i}")
        ctx.check(
            f"handle_{i} stands above the beam",
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
            and min(abs(bumper[0][0]), abs(bumper[1][0])) > 1.2,
            details=f"bumper aabb={bumper}",
        )

    # Decisive pose checks: rocking alternately lowers each end.
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
