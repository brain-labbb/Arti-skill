from __future__ import annotations

# Decorative two-pan balance scale (scales-of-justice style), matte black cast iron.
# Variant: flat_plate_single_rod — flat circular weighing plates on single central rods.
# Frame: +Z up, ground at z=0, beam along X, all pivot axes along +Y.
#
# Assembly (root = pedestal):
#   pedestal (root): stepped square plinth, fluted transition ring, turned
#                    baluster column with ring moldings, pivot clevis head and
#                    dome finial. Static.
#   beam:            straight rectangular cross beam (~0.30 m) with center hub,
#                    Y pivot axle through the clevis, end knobs and drop pins
#                    with hang balls. REVOLUTE about +Y at the column top,
#                    range -15..+15 deg (weighing tilt).
#   pan_0 / pan_1:   flat-plate weighing assemblies. Each is a top ring hung on
#                    the beam-tip hang ball, a single central hanging rod, and
#                    a flat circular plate (~0.11 m dia) with a thin raised rim.
#                    Each REVOLUTE about +Y at its beam tip, range -15..+15 deg
#                    (independent swing, lets plates stay level as beam tilts).

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------- layout constants (meters) ----------------
PLINTH1_W, PLINTH1_H = 0.130, 0.018          # bottom square step
PLINTH2_W, PLINTH2_H = 0.114, 0.014          # second square step
CAP_BASE_W, CAP_TOP_R = 0.100, 0.040         # square -> round molding loft
CAP_Z0, CAP_Z1 = 0.032, 0.052

COLUMN_TOP = 0.342                           # top of the turned column capital
PIVOT_Z = 0.357                              # beam pivot height (clevis center)

BEAM_LEN = 0.300
BEAM_SEC_Y = 0.012
BEAM_SEC_Z = 0.014
TIP_X = BEAM_LEN / 2.0                       # 0.150
KNOB_R = 0.008
PIN_R = 0.0025
BALL_R = 0.0050
HANG_Z = -0.0215                             # hang-ball center in beam frame

TILT = math.radians(15.0)                    # +/- joint range for all three joints

# Pan geometry (flat plate on single rod variant)
RING_R = 0.006                               # hanger top ring major radius
RING_TUBE = 0.0017                           # hanger ring tube radius
ROD_R = 0.0020                               # single hanging rod radius
PLATE_R = 0.055                              # plate radius (0.11 m diameter)
PLATE_T = 0.003                              # plate thickness
RIM_H = 0.004                                # rim height above plate top
RIM_W = 0.002                                # rim radial width
ROD_LEN = 0.168                              # rod length (hub to plate top)
PLATE_TOP_Z = -0.008 - ROD_LEN               # = -0.176 (plate top in pan frame)
PLATE_BOT_Z = PLATE_TOP_Z - PLATE_T          # = -0.179 (plate bottom in pan frame)

FLUTE_COUNT = 14


# ---------------- cadquery helpers ----------------
def _rod_solid(p0: tuple, p1: tuple, radius: float) -> cq.Solid:
    """Cylinder of given radius from point p0 to point p1."""
    v = (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2])
    length = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
    direction = cq.Vector(v[0] / length, v[1] / length, v[2] / length)
    return cq.Solid.makeCylinder(radius, length, cq.Vector(*p0), direction)


def _build_plinth() -> cq.Workplane:
    """Stepped square plinth with a square-to-round molding cap."""
    step1 = cq.Workplane("XY").box(PLINTH1_W, PLINTH1_W, PLINTH1_H, centered=(True, True, False))
    step2 = (
        cq.Workplane("XY")
        .workplane(offset=PLINTH1_H)
        .box(PLINTH2_W, PLINTH2_W, PLINTH2_H, centered=(True, True, False))
    )
    cap = (
        cq.Workplane("XY")
        .workplane(offset=CAP_Z0)
        .rect(CAP_BASE_W, CAP_BASE_W)
        .workplane(offset=CAP_Z1 - CAP_Z0)
        .circle(CAP_TOP_R)
        .loft(ruled=True)
    )
    return step1.union(step2).union(cap)


def _build_column() -> cq.Workplane:
    """Turned baluster column: bell flare, vase bulge, ring moldings, capital."""
    pts = [
        (0.0, 0.046),
        (0.042, 0.046),
        (0.040, 0.052),
        (0.033, 0.062),
        (0.025, 0.076),
        (0.021, 0.090),
        (0.020, 0.104),
        (0.0245, 0.110),   # ring molding
        (0.0245, 0.118),
        (0.020, 0.124),
        (0.027, 0.142),    # vase bulge
        (0.0285, 0.162),
        (0.024, 0.186),
        (0.019, 0.206),
        (0.023, 0.212),    # ring molding
        (0.023, 0.220),
        (0.018, 0.226),
        (0.016, 0.250),
        (0.014, 0.280),
        (0.017, 0.286),    # ring molding
        (0.017, 0.292),
        (0.013, 0.297),
        (0.012, 0.320),
        (0.016, 0.328),    # capital flare
        (0.016, 0.338),
        (0.011, COLUMN_TOP),
        (0.0, COLUMN_TOP),
    ]
    return cq.Workplane("XZ").polyline(pts).close().revolve(360.0, (0, 0), (0, 1))


def _build_flute_ring() -> cq.Workplane:
    """Ring of slanted ribs decorating the bell flare above the plinth."""
    solids = []
    for k in range(FLUTE_COUNT):
        ang = math.radians(k * 360.0 / FLUTE_COUNT)
        p0 = (0.0405 * math.cos(ang), 0.0405 * math.sin(ang), 0.050)
        p1 = (0.026 * math.cos(ang), 0.026 * math.sin(ang), 0.078)
        solids.append(_rod_solid(p0, p1, 0.0035))
    return cq.Workplane(obj=cq.Compound.makeCompound(solids))


def _build_pivot_head() -> cq.Workplane:
    """Clevis fork at the column top, bridge cap, and dome finial."""
    plate_a = (
        cq.Workplane("XY")
        .box(0.024, 0.006, 0.034, centered=(True, True, False))
        .translate((0.0, 0.011, 0.340))
    )
    plate_b = plate_a.mirror("XZ")
    bridge = (
        cq.Workplane("XY")
        .box(0.024, 0.028, 0.006, centered=(True, True, False))
        .translate((0.0, 0.0, 0.374))
    )
    neck = (
        cq.Workplane("XY")
        .cylinder(0.009, 0.011, centered=(True, True, False))
        .translate((0.0, 0.0, 0.379))
    )
    dome = cq.Workplane("XY").sphere(0.0115).translate((0.0, 0.0, 0.389))
    return plate_a.union(plate_b).union(bridge).union(neck).union(dome)


def _build_single_rod() -> cq.Workplane:
    """Top ring hook, hub sphere, and single central hanging rod."""
    ring = cq.Solid.makeTorus(RING_R, RING_TUBE, cq.Vector(0, 0, 0), cq.Vector(0, 1, 0))
    hub = cq.Solid.makeSphere(0.004, cq.Vector(0, 0, -0.008),
                              angleDegrees1=-90, angleDegrees2=90)
    # Rod: cylinder from z=PLATE_TOP_Z up to z=-0.008
    rod = (
        cq.Workplane("XY")
        .cylinder(ROD_LEN, ROD_R, centered=(True, True, False))
        .translate((0.0, 0.0, PLATE_TOP_Z))
    )
    return cq.Workplane(obj=ring).union(cq.Workplane(obj=hub)).union(rod)


def _build_flat_plate() -> cq.Workplane:
    """Flat circular weighing plate with raised rim around the perimeter."""
    # Main disk: from z=PLATE_BOT_Z to z=PLATE_TOP_Z
    plate = (
        cq.Workplane("XY")
        .cylinder(PLATE_T, PLATE_R, centered=(True, True, False))
        .translate((0.0, 0.0, PLATE_BOT_Z))
    )
    # Rim: thin ring from z=PLATE_TOP_Z to z=PLATE_TOP_Z+RIM_H
    rim_outer = (
        cq.Workplane("XY")
        .cylinder(RIM_H, PLATE_R, centered=(True, True, False))
        .translate((0.0, 0.0, PLATE_TOP_Z))
    )
    rim_inner = (
        cq.Workplane("XY")
        .cylinder(RIM_H, PLATE_R - RIM_W, centered=(True, True, False))
        .translate((0.0, 0.0, PLATE_TOP_Z))
    )
    rim = rim_outer.cut(rim_inner)
    return plate.union(rim)


# ---------------- model ----------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_pan_balance_scale")

    iron_base = model.material("cast_iron_base", rgba=(0.075, 0.075, 0.082, 1.0))
    iron_beam = model.material("cast_iron_beam", rgba=(0.105, 0.105, 0.115, 1.0))
    iron_rod = model.material("cast_iron_rod", rgba=(0.135, 0.135, 0.145, 1.0))
    iron_plate = model.material("cast_iron_plate", rgba=(0.060, 0.060, 0.067, 1.0))

    # ----- pedestal (root): plinth + flute ring + column + pivot head -----
    pedestal = model.part("pedestal")
    pedestal.visual(
        mesh_from_cadquery(_build_plinth(), "pedestal_plinth"),
        material=iron_base,
        name="plinth",
    )
    pedestal.visual(
        mesh_from_cadquery(_build_flute_ring(), "pedestal_flute_ring"),
        material=iron_base,
        name="flute_ring",
    )
    pedestal.visual(
        mesh_from_cadquery(_build_column(), "pedestal_column"),
        material=iron_base,
        name="column",
    )
    pedestal.visual(
        mesh_from_cadquery(_build_pivot_head(), "pedestal_pivot_head"),
        material=iron_base,
        name="pivot_head",
    )

    # ----- cross beam: REVOLUTE about +Y at the column top -----
    beam = model.part("beam")
    beam.visual(
        Box((BEAM_LEN, BEAM_SEC_Y, BEAM_SEC_Z)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=iron_beam,
        name="bar",
    )
    beam.visual(
        Cylinder(radius=0.0095, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron_beam,
        name="pivot_hub",
    )
    beam.visual(
        Cylinder(radius=0.0035, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron_beam,
        name="pivot_axle",
    )
    for idx, sx in ((0, 1.0), (1, -1.0)):
        beam.visual(
            Sphere(radius=KNOB_R),
            origin=Origin(xyz=(sx * TIP_X, 0.0, 0.0)),
            material=iron_beam,
            name=f"end_knob_{idx}",
        )
        beam.visual(
            Cylinder(radius=PIN_R, length=0.016),
            origin=Origin(xyz=(sx * TIP_X, 0.0, -0.012)),
            material=iron_beam,
            name=f"hang_pin_{idx}",
        )
        beam.visual(
            Sphere(radius=BALL_R),
            origin=Origin(xyz=(sx * TIP_X, 0.0, HANG_Z)),
            material=iron_beam,
            name=f"hang_ball_{idx}",
        )

    model.articulation(
        "pedestal_to_beam",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.0, lower=-TILT, upper=TILT),
    )

    # ----- pan plates: single rod + flat plate, each REVOLUTE about +Y -----
    for idx, sx in ((0, 1.0), (1, -1.0)):
        pan = model.part(f"pan_{idx}")
        pan.visual(
            mesh_from_cadquery(_build_single_rod(), f"pan_rod_{idx}"),
            material=iron_rod,
            name="hanger_rod",
        )
        pan.visual(
            mesh_from_cadquery(_build_flat_plate(), f"pan_plate_{idx}"),
            material=iron_plate,
            name="plate",
        )
        model.articulation(
            f"beam_to_pan_{idx}",
            ArticulationType.REVOLUTE,
            parent=beam,
            child=pan,
            origin=Origin(xyz=(sx * TIP_X, 0.0, HANG_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=1.5, lower=-TILT, upper=TILT),
        )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pedestal = object_model.get_part("pedestal")
    beam = object_model.get_part("beam")
    pan_0 = object_model.get_part("pan_0")
    pan_1 = object_model.get_part("pan_1")

    pivot = object_model.get_articulation("pedestal_to_beam")
    swing_0 = object_model.get_articulation("beam_to_pan_0")
    swing_1 = object_model.get_articulation("beam_to_pan_1")

    # ---- intentional mechanical embeddings ----
    ctx.allow_overlap(
        beam,
        pedestal,
        elem_a="pivot_axle",
        elem_b="pivot_head",
        reason="Beam pivot axle passes through the clevis fork plates at the column top.",
    )
    for pan, ball, pin in ((pan_0, "hang_ball_0", "hang_pin_0"), (pan_1, "hang_ball_1", "hang_pin_1")):
        ctx.allow_overlap(
            pan,
            beam,
            elem_a="hanger_rod",
            elem_b=ball,
            reason="Hanger top ring is threaded around the beam-tip hang ball (hook-on-ring joint).",
        )
        ctx.allow_overlap(
            pan,
            beam,
            elem_a="hanger_rod",
            elem_b=pin,
            reason="Hanger top ring encircles the drop pin above the hang ball.",
        )

    # ---- pedestal: grounded, ~0.40 m tall, square plinth ~0.13 m wide ----
    ped_aabb = ctx.part_world_aabb(pedestal)
    ctx.check(
        "pedestal sits on the ground plane",
        abs(ped_aabb[0][2]) < 1e-4,
        details=f"pedestal zmin={ped_aabb[0][2]}",
    )
    ctx.check(
        "scale stands about 0.40 m tall (dome finial at top)",
        0.39 < ped_aabb[1][2] < 0.41,
        details=f"pedestal zmax={ped_aabb[1][2]}",
    )
    ped_ext = _ext(ped_aabb)
    ctx.check(
        "square plinth is about 0.13 m wide",
        abs(ped_ext[0] - PLINTH1_W) < 0.005 and abs(ped_ext[1] - PLINTH1_W) < 0.005,
        details=f"pedestal extents={ped_ext}",
    )

    # ---- beam: ~0.30 m long slender bar pivoting at the column top ----
    beam_ext = _ext(ctx.part_world_aabb(beam))
    ctx.check(
        "cross beam spans about 0.30 m along X with small end knobs",
        0.300 <= beam_ext[0] < 0.330 and beam_ext[1] < 0.05,
        details=f"beam extents={beam_ext}",
    )
    ctx.check(
        "beam pivot is a revolute joint about +Y at the column top",
        pivot.articulation_type == ArticulationType.REVOLUTE
        and tuple(pivot.axis) == (0.0, 1.0, 0.0)
        and abs(pivot.origin.xyz[2] - PIVOT_Z) < 1e-9,
        details=f"axis={pivot.axis}, origin={pivot.origin.xyz}",
    )
    for joint in (pivot, swing_0, swing_1):
        lim = joint.motion_limits
        ctx.check(
            f"{joint.name} has a -15..+15 degree range about +Y",
            lim is not None
            and abs(lim.lower + TILT) < 1e-9
            and abs(lim.upper - TILT) < 1e-9
            and tuple(joint.axis) == (0.0, 1.0, 0.0),
            details=f"{joint.name}: lower={lim.lower}, upper={lim.upper}, axis={joint.axis}",
        )

    # ---- pans: round 0.11 m flat plates on single rods below the beam tips ----
    for name, pan, sx in (("pan_0", pan_0, 1.0), ("pan_1", pan_1, -1.0)):
        aabb = ctx.part_world_aabb(pan)
        ext = _ext(aabb)
        ctx.check(
            f"{name} plate is round and about 0.11 m in diameter",
            abs(ext[0] - 2 * PLATE_R) < 0.005 and abs(ext[1] - 2 * PLATE_R) < 0.005,
            details=f"{name} extents={ext}",
        )
        ctx.check(
            f"{name} assembly hangs below its beam tip (rod + flat plate)",
            0.17 < ext[2] < 0.21,
            details=f"{name} extents={ext}",
        )
        cx = 0.5 * (aabb[0][0] + aabb[1][0])
        ctx.check(
            f"{name} is centered under its beam tip at x={sx * TIP_X:+.3f}",
            abs(cx - sx * TIP_X) < 0.004,
            details=f"{name} center x={cx}",
        )
        ctx.check(
            f"{name} clears the pedestal column and base at rest",
            (aabb[0][0] > ped_ext[0] / 2.0 if sx > 0 else aabb[1][0] < -ped_ext[0] / 2.0)
            and aabb[0][2] > CAP_Z1,
            details=f"{name} aabb={aabb}",
        )

    # ---- flat plate is thin in Z (distinguishes from a deep dish) ----
    for name, pan in (("pan_0", pan_0), ("pan_1", pan_1)):
        plate_aabb = ctx.part_element_world_aabb(pan, elem="plate")
        if plate_aabb is not None:
            plate_z = plate_aabb[1][2] - plate_aabb[0][2]
            ctx.check(
                f"{name} plate is a thin flat (not a deep dished pan)",
                plate_z < 0.012,
                details=f"{name} plate z-extent={plate_z:.4f}",
            )

    # ---- single rod: each pan has exactly one central hanging rod ----
    for name, pan in (("pan_0", pan_0), ("pan_1", pan_1)):
        rod_aabb = ctx.part_element_world_aabb(pan, elem="hanger_rod")
        if rod_aabb is not None:
            rod_xy = (rod_aabb[1][0] - rod_aabb[0][0], rod_aabb[1][1] - rod_aabb[0][1])
            ctx.check(
                f"{name} hanger is a single narrow rod (not a wide three-chain spread)",
                rod_xy[0] < 0.020 and rod_xy[1] < 0.020,
                details=f"{name} rod XY extents=({rod_xy[0]:.4f}, {rod_xy[1]:.4f})",
            )

    # ---- pans hang on the beam tips (ring threaded over the hang ball) ----
    ctx.expect_overlap(pan_0, beam, axes="z", min_overlap=0.005, name="pan_0 ring hangs on beam tip")
    ctx.expect_overlap(pan_1, beam, axes="z", min_overlap=0.005, name="pan_1 ring hangs on beam tip")

    # ---- weighing tilt: +q drops the +X tip and raises the -X tip ----
    rest_z0 = ctx.part_world_position(pan_0)[2]
    rest_z1 = ctx.part_world_position(pan_1)[2]
    with ctx.pose({pivot: TILT}):
        tilt_z0 = ctx.part_world_position(pan_0)[2]
        tilt_z1 = ctx.part_world_position(pan_1)[2]
    ctx.check(
        "tilting the beam +15 deg lowers one pan hook and raises the other",
        tilt_z0 < rest_z0 - 0.030 and tilt_z1 > rest_z1 + 0.030,
        details=f"rest=({rest_z0:.4f},{rest_z1:.4f}) tilted=({tilt_z0:.4f},{tilt_z1:.4f})",
    )

    # ---- pan swing: rotating a pan joint swings the plate off-axis in X ----
    rest_aabb = ctx.part_world_aabb(pan_0)
    rest_cx = 0.5 * (rest_aabb[0][0] + rest_aabb[1][0])
    with ctx.pose({swing_0: TILT}):
        swung_aabb = ctx.part_world_aabb(pan_0)
    swung_cx = 0.5 * (swung_aabb[0][0] + swung_aabb[1][0])
    ctx.check(
        "swinging pan_0 +15 deg displaces the plate sideways (off-axis proof)",
        abs(swung_cx - rest_cx) > 0.020,
        details=f"rest cx={rest_cx:.4f}, swung cx={swung_cx:.4f}",
    )

    # ---- full tilt with level pans: nothing strikes the pedestal ----
    with ctx.pose({pivot: TILT, swing_0: -TILT, swing_1: -TILT}):
        low_aabb = ctx.part_world_aabb(pan_0)
        ctx.check(
            "lowered pan stays above the plinth and clear of the column",
            low_aabb[0][2] > CAP_Z1 and low_aabb[0][0] > ped_ext[0] / 2.0,
            details=f"lowered pan_0 aabb={low_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
