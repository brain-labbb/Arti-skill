from __future__ import annotations

# Decorative two-pan balance scale — ornate scroll beam variant.
# Matte black cast iron, scales-of-justice style.
# Frame: +Z up, ground at z=0, beam along X, all pivot axes along +Y.
#
# Assembly (root = pedestal):
#   pedestal (root): stepped square plinth, fluted transition ring, turned
#                    baluster column with ring moldings, pivot clevis head and
#                    dome finial. Static.
#   beam:            ornate S-curve scroll arms (~0.30 m span) with decorative
#                    scroll volutes at each tip, center rosette, pivot hub and
#                    axle through the clevis, end knobs and drop pins with hang
#                    balls. REVOLUTE about +Y at the column top, range -15..+15
#                    deg (weighing tilt).
#   pan_0 / pan_1:   pan-and-hanger slings. Each is a top ring hung on the beam
#                    tip ball, three thin rod "chains" converging from the dish
#                    rim, and a shallow concave dished pan (~0.11 m dia).
#                    Each REVOLUTE about +Y at its beam tip, range -15..+15 deg
#                    (independent swing, lets pans stay level as the beam tilts).

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
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
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

RING_R = 0.006                               # hanger top ring (torus) major radius
RING_TUBE = 0.0017
ROD_R = 0.0017
DISH_R = 0.055                               # pan radius (0.11 m diameter)
DISH_SPHERE_R = 0.135                        # dishing sphere radius (shallow concave)
DISH_BOT = -0.192                            # dish underside in pan frame
DISH_RIM_TOP = -0.1795
ATTACH_R = 0.050                             # rod attachment radius on the rim
ATTACH_Z = -0.1805
ATTACH_ANGLES = (90.0, 210.0, 330.0)

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


def _build_hanger_set() -> cq.Workplane:
    """Top ring, converging hub, and three thin rod 'chains' plus rim balls."""
    ring = cq.Solid.makeTorus(RING_R, RING_TUBE, cq.Vector(0, 0, 0), cq.Vector(0, 1, 0))
    hub = cq.Solid.makeSphere(0.0035, cq.Vector(0, 0, -0.008), angleDegrees1=-90, angleDegrees2=90)
    result = cq.Workplane(obj=ring).union(cq.Workplane(obj=hub))
    for ang_deg in ATTACH_ANGLES:
        ang = math.radians(ang_deg)
        tip = (ATTACH_R * math.cos(ang), ATTACH_R * math.sin(ang), ATTACH_Z)
        rod = _rod_solid((0.0, 0.0, -0.008), tip, ROD_R)
        ball = cq.Solid.makeSphere(
            0.0028, cq.Vector(*tip), angleDegrees1=-90, angleDegrees2=90
        )
        result = result.union(cq.Workplane(obj=rod)).union(cq.Workplane(obj=ball))
    return result


def _build_dish() -> cq.Workplane:
    """Shallow concave dished pan with a thin rim (spherical-cap shell)."""
    zc = DISH_BOT + DISH_SPHERE_R
    thickness = 0.0028
    outer = cq.Workplane("XY").sphere(DISH_SPHERE_R).translate((0.0, 0.0, zc))
    inner = cq.Workplane("XY").sphere(DISH_SPHERE_R).translate((0.0, 0.0, zc + thickness))
    slab = (
        cq.Workplane("XY")
        .cylinder(DISH_RIM_TOP - (DISH_BOT - 0.001), DISH_R, centered=(True, True, False))
        .translate((0.0, 0.0, DISH_BOT - 0.001))
    )
    return outer.cut(inner).intersect(slab)


# ---------------- ornate scroll beam ----------------
def _scroll_arm_path(sign: float) -> list[tuple[float, float, float]]:
    """S-curve (ogee) path for one scroll arm from center hub to beam tip.

    sign=+1 for the +X arm, sign=-1 for the -X arm.
    The arm rises gently above the pivot axis then returns level at the tip.
    """
    s = sign
    return [
        (s * 0.005, 0.0, 0.0),       # start inside the pivot hub for connectivity
        (s * 0.030, 0.0, 0.006),
        (s * 0.055, 0.0, 0.013),
        (s * 0.080, 0.0, 0.015),
        (s * 0.105, 0.0, 0.010),
        (s * 0.130, 0.0, 0.003),
        (s * TIP_X, 0.0, 0.0),
    ]


def _build_scroll_arm_mesh(sign: float):
    """Swept ornate arm with a decorative rounded-rect cross section."""
    profile = rounded_rect_profile(0.010, 0.016, radius=0.003, corner_segments=6)
    return sweep_profile_along_spline(
        _scroll_arm_path(sign),
        profile=profile,
        samples_per_segment=18,
        cap_profile=True,
        up_hint=(0.0, 0.0, 1.0),
    )


def _scroll_volute_path(sign: float) -> list[tuple[float, float, float]]:
    """Small decorative scroll curl at the beam tip.

    The volute curls upward and outward above the arm tip, reading as a
    classic cast-iron scroll motif, clear of the hanger ring below.
    """
    s = sign
    cx = s * TIP_X
    return [
        (cx - s * 0.003, 0.0, 0.005),   # start inside the arm tip for connectivity
        (cx - s * 0.001, 0.0, 0.014),
        (cx + s * 0.005, 0.0, 0.022),
        (cx + s * 0.011, 0.0, 0.020),
        (cx + s * 0.013, 0.0, 0.014),
        (cx + s * 0.009, 0.0, 0.010),
        (cx + s * 0.004, 0.0, 0.013),
    ]


def _build_scroll_volute_mesh(sign: float):
    """Thin tubular scroll volute at one beam tip."""
    return tube_from_spline_points(
        _scroll_volute_path(sign),
        radius=0.003,
        samples_per_segment=14,
        radial_segments=14,
        cap_ends=True,
        up_hint=(0.0, 1.0, 0.0),
    )


def _build_center_rosette_mesh():
    """Small decorative rosette ring around the pivot hub on the beam.

    Sized to fit between the clevis fork plates without touching the bridge.
    """
    torus = cq.Solid.makeTorus(0.011, 0.0025, cq.Vector(0, 0, 0), cq.Vector(0, 1, 0))
    return mesh_from_cadquery(
        cq.Workplane(obj=torus),
        "beam_center_rosette",
    )


# ---------------- model ----------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_pan_balance_scale_scroll_beam")

    iron_base = model.material("cast_iron_base", rgba=(0.075, 0.075, 0.082, 1.0))
    iron_beam = model.material("cast_iron_beam", rgba=(0.105, 0.105, 0.115, 1.0))
    iron_chain = model.material("cast_iron_chain", rgba=(0.135, 0.135, 0.145, 1.0))
    iron_pan = model.material("cast_iron_pan", rgba=(0.060, 0.060, 0.067, 1.0))

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

    # ----- ornate scroll beam: S-curve arms with scroll volutes -----
    beam = model.part("beam")
    for idx, sx in ((0, 1.0), (1, -1.0)):
        beam.visual(
            mesh_from_geometry(_build_scroll_arm_mesh(sx), f"scroll_arm_{idx}"),
            material=iron_beam,
            name=f"scroll_arm_{idx}",
        )
        beam.visual(
            mesh_from_geometry(_build_scroll_volute_mesh(sx), f"scroll_volute_{idx}"),
            material=iron_beam,
            name=f"scroll_volute_{idx}",
        )
    beam.visual(
        _build_center_rosette_mesh(),
        material=iron_beam,
        name="center_rosette",
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

    # ----- pan slings: each hangs on its beam tip ball, REVOLUTE about +Y -----
    for idx, sx in ((0, 1.0), (1, -1.0)):
        pan = model.part(f"pan_{idx}")
        pan.visual(
            mesh_from_cadquery(_build_hanger_set(), f"pan_hangers_{idx}"),
            material=iron_chain,
            name="hanger_rods",
        )
        pan.visual(
            mesh_from_cadquery(_build_dish(), f"pan_dish_{idx}"),
            material=iron_pan,
            name="dish",
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
            elem_a="hanger_rods",
            elem_b=ball,
            reason="Hanger top ring is threaded around the beam-tip hang ball (hook-on-ring joint).",
        )
        ctx.allow_overlap(
            pan,
            beam,
            elem_a="hanger_rods",
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

    # ---- ornate scroll beam: S-curve arms spanning ~0.30 m ----
    beam_ext = _ext(ctx.part_world_aabb(beam))
    ctx.check(
        "scroll beam spans about 0.30 m along X",
        0.300 <= beam_ext[0] < 0.340 and beam_ext[1] < 0.05,
        details=f"beam extents={beam_ext}",
    )
    # The scroll arms rise above the pivot axis; the beam z-extent should exceed
    # the old straight bar (0.014 m) because the S-curve peaks at ~0.014 m above center.
    ctx.check(
        "scroll beam has curved arms (z-extent exceeds a straight bar)",
        beam_ext[2] > 0.020,
        details=f"beam z-extent={beam_ext[2]:.4f}",
    )
    # Each scroll arm rises above the pivot axis at its midpoint
    for arm_name in ("scroll_arm_0", "scroll_arm_1"):
        arm_aabb = ctx.part_element_world_aabb(beam, elem=arm_name)
        ctx.check(
            f"{arm_name} rises above the pivot axis (curved S-shape)",
            arm_aabb is not None and arm_aabb[1][2] > PIVOT_Z + 0.005,
            details=f"{arm_name} top z={arm_aabb[1][2] if arm_aabb else None}",
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

    # ---- pans: round 0.11 m dishes hanging level below the beam tips ----
    for name, pan, sx in (("pan_0", pan_0, 1.0), ("pan_1", pan_1, -1.0)):
        aabb = ctx.part_world_aabb(pan)
        ext = _ext(aabb)
        ctx.check(
            f"{name} dish is round and about 0.11 m in diameter",
            abs(ext[0] - 2 * DISH_R) < 0.004 and abs(ext[1] - 2 * DISH_R) < 0.004,
            details=f"{name} extents={ext}",
        )
        ctx.check(
            f"{name} hangs ~0.19 m below its beam tip (ring + rods + dish)",
            0.18 < ext[2] < 0.21,
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

    # ---- pan swing: rotating a pan joint swings the dish off-axis in X ----
    rest_aabb = ctx.part_world_aabb(pan_0)
    rest_cx = 0.5 * (rest_aabb[0][0] + rest_aabb[1][0])
    with ctx.pose({swing_0: TILT}):
        swung_aabb = ctx.part_world_aabb(pan_0)
    swung_cx = 0.5 * (swung_aabb[0][0] + swung_aabb[1][0])
    ctx.check(
        "swinging pan_0 +15 deg displaces the dish sideways (off-axis proof)",
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
