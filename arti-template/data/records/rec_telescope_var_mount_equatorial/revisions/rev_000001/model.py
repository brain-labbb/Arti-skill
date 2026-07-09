from __future__ import annotations

# Small refractor telescope on an adjustable tripod with a German Equatorial Mount.
# Frame:
#   - World Z is up. The tripod stands on the ground (feet near z=0).
#   - The EQ head sits on top of the tripod. The RA (azimuth_rotation) joint
#     tilts the head frame so local Z aligns with the polar axis (tilted at
#     LAT_ANGLE from horizontal toward +Y). RA rotation is CONTINUOUS about
#     this tilted polar axis.
#   - The DEC (tube_altitude) axis is perpendicular to the polar axis, at the
#     top of the polar housing. The optical tube points along +X at rest; the
#     black dew-shield end is at +X (front), the brass focuser draw-tube and
#     eyepiece are at -X (rear).
#   - A counterweight shaft + ball extend from the DEC bar on the opposite
#     side from the tube, balancing the assembly.
# Articulations:
#   - azimuth_rotation: CONTINUOUS about the tilted polar axis (RA tracking).
#   - tube_altitude:    REVOLUTE about the DEC axis (declination adjustment).
#   - focuser_slide:    PRISMATIC along the tube axis (draw-tube slides in/out).

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

# --- key dimensions (meters) ---
TRIPOD_TOP_Z = 0.320  # head pivot height above ground
TUBE_LEN = 0.300  # optical tube body length
TUBE_R = 0.030  # optical tube outer radius
# Tube local X span: rear of tube at -0.135, front (dew shield mouth) at +0.165.
TUBE_REAR_X = -0.135
TUBE_FRONT_X = 0.165

# German Equatorial Mount parameters.
LAT_ANGLE = math.radians(40.0)  # latitude angle for polar axis tilt
POLAR_LEN = 0.085  # polar-axis housing length (along head local Z)


def _tube_shell() -> cq.Workplane:
    # Hollow optical tube: outer cylinder minus a bore so the objective end reads
    # as an open dew shield.  Built along local +X (lofted YZ sections).
    outer = (
        cq.Workplane("YZ")
        .workplane(offset=TUBE_REAR_X)
        .circle(TUBE_R * 0.92)
        .workplane(offset=0.020)
        .circle(TUBE_R)
        .workplane(offset=TUBE_FRONT_X - TUBE_REAR_X - 0.020)
        .circle(TUBE_R)
        .loft(ruled=True)
    )
    bore = (
        cq.Workplane("YZ")
        .workplane(offset=TUBE_FRONT_X - 0.090)
        .circle(TUBE_R - 0.004)
        .workplane(offset=0.120)
        .circle(TUBE_R - 0.004)
        .loft(ruled=True)
    )
    return outer.cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="refractor_telescope")

    blue = model.material("tube_blue", rgba=(0.30, 0.42, 0.62, 1.0))
    white = model.material("tube_white", rgba=(0.90, 0.92, 0.95, 1.0))
    black = model.material("matte_black", rgba=(0.10, 0.10, 0.12, 1.0))
    brass = model.material("brass", rgba=(0.78, 0.62, 0.28, 1.0))
    metal = model.material("tripod_metal", rgba=(0.70, 0.72, 0.76, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.28, 0.30, 0.33, 1.0))
    foot_rubber = model.material("foot_rubber", rgba=(0.12, 0.14, 0.20, 1.0))

    # =====================================================================
    # TRIPOD (root part) — hub on top, three splayed legs to pointed feet,
    # plus a spreader ring tying the legs together.
    # =====================================================================
    tripod = model.part("tripod")

    HUB_Z = TRIPOD_TOP_Z - 0.020
    LEG_TOP_R = 0.026  # leg anchor radius at the hub
    FOOT_R = 0.150  # foot footprint radius
    SPREADER_Z = 0.110

    # Central hub the head bolts onto.
    tripod.visual(
        Cylinder(radius=0.030, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, HUB_Z)),
        material=dark_metal,
        name="hub",
    )
    tripod.visual(
        Cylinder(radius=0.034, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, TRIPOD_TOP_Z - 0.044)),
        material=brass,
        name="hub_collar",
    )

    leg_angles = (math.pi / 2.0, math.pi / 2.0 + 2.0 * math.pi / 3.0, math.pi / 2.0 + 4.0 * math.pi / 3.0)
    for i, ang in enumerate(leg_angles):
        c, s = math.cos(ang), math.sin(ang)
        # Splayed leg: from just under the hub out and down to a pointed foot.
        leg_mesh = tube_from_spline_points(
            [
                (LEG_TOP_R * c, LEG_TOP_R * s, HUB_Z - 0.010),
                (0.5 * (LEG_TOP_R + FOOT_R) * c, 0.5 * (LEG_TOP_R + FOOT_R) * s, SPREADER_Z),
                (FOOT_R * c, FOOT_R * s, 0.022),
            ],
            radius=0.0075,
            samples_per_segment=16,
            radial_segments=14,
            cap_ends=True,
        )
        tripod.visual(mesh_from_geometry(leg_mesh, f"tripod_leg_{i}"), material=metal, name=f"leg_{i}")

        # Pointed metal foot (cone tip down).
        foot_mesh = tube_from_spline_points(
            [
                (FOOT_R * c, FOOT_R * s, 0.024),
                (FOOT_R * c, FOOT_R * s, 0.004),
            ],
            radius=0.0095,
            samples_per_segment=4,
            radial_segments=12,
            cap_ends=True,
        )
        tripod.visual(mesh_from_geometry(foot_mesh, f"tripod_foot_{i}"), material=dark_metal, name=f"foot_collar_{i}")
        tripod.visual(
            Sphere(radius=0.006),
            origin=Origin(xyz=(FOOT_R * c, FOOT_R * s, 0.004)),
            material=foot_rubber,
            name=f"foot_tip_{i}",
        )

        # Spreader bar joining this leg to the next (brace ring).
        ang2 = leg_angles[(i + 1) % 3]
        c2, s2 = math.cos(ang2), math.sin(ang2)
        mid_r = 0.5 * (LEG_TOP_R + FOOT_R)
        p0 = (mid_r * c, mid_r * s, SPREADER_Z)
        p1 = (mid_r * c2, mid_r * s2, SPREADER_Z)
        brace_mesh = tube_from_spline_points(
            [p0, (0.5 * (p0[0] + p1[0]), 0.5 * (p0[1] + p1[1]), SPREADER_Z), p1],
            radius=0.0055,
            samples_per_segment=10,
            radial_segments=12,
            cap_ends=True,
        )
        tripod.visual(mesh_from_geometry(brace_mesh, f"tripod_spreader_{i}"), material=brass, name=f"spreader_{i}")

    tripod.inertial = Inertial.from_geometry(
        Box((2.0 * FOOT_R, 2.0 * FOOT_R, TRIPOD_TOP_Z)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
    )

    # =====================================================================
    # EQUATORIAL HEAD — German Equatorial Mount (GEM) on top of the tripod.
    # The RA joint tilts the head frame so local Z = polar axis (tilted at
    # LAT_ANGLE from horizontal). The polar housing, DEC cross-bar, and
    # counterweight are all defined in this tilted frame.
    # =====================================================================
    head = model.part("azimuth_head")

    # Wedge base: a block transitioning from the tripod top to the tilted
    # polar axis. In head frame, Z = polar axis, so this is along polar axis.
    head.visual(
        Box((0.060, 0.058, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=dark_metal,
        name="polar_wedge",
    )
    # Polar-axis housing: the main RA bearing cylinder along head Z.
    # Kept short so the top clears the tube's blue_band wrap.
    head.visual(
        Cylinder(radius=0.022, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=metal,
        name="polar_housing",
    )
    # Upper polar shaft connecting the housing to the DEC bar.
    head.visual(
        Cylinder(radius=0.012, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        material=metal,
        name="polar_shaft",
    )
    # RA bearing cap at the top of the polar housing.
    head.visual(
        Cylinder(radius=0.028, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        material=brass,
        name="ra_bearing",
    )
    # DEC axis cross-bar: horizontal cylinder along head X at the polar top.
    head.visual(
        Cylinder(radius=0.016, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, POLAR_LEN), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="dec_bar",
    )
    # Counterweight shaft: rod from the -X end of the DEC bar, offset in +Y
    # so the CW hangs below the tube on the opposite side.
    cw_shaft_mesh = tube_from_spline_points(
        [
            (-0.045, 0.0, POLAR_LEN),
            (-0.105, 0.060, POLAR_LEN),
        ],
        radius=0.005,
        samples_per_segment=4,
        radial_segments=10,
        cap_ends=True,
    )
    head.visual(
        mesh_from_geometry(cw_shaft_mesh, "cw_shaft_mesh"),
        material=dark_metal,
        name="cw_shaft",
    )
    # Counterweight ball at the end of the shaft.
    head.visual(
        Sphere(radius=0.020),
        origin=Origin(xyz=(-0.105, 0.060, POLAR_LEN)),
        material=dark_metal,
        name="cw_ball",
    )
    # Off-axis RA marker on the side of the polar housing so RA spin is
    # visually detectable.  Placed low enough to clear the tube shell.
    head.visual(
        Box((0.014, 0.010, 0.014)),
        origin=Origin(xyz=(0.028, 0.0, 0.030)),
        material=brass,
        name="ra_marker",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.14, 0.08, 0.10)),
        mass=0.50,
        origin=Origin(xyz=(-0.03, 0.0, 0.045)),
    )

    # RA (azimuth_rotation): CONTINUOUS about the tilted polar axis.
    # The rpy tilts the head frame so its local Z aligns with the polar axis
    # direction (0, cos(LAT), sin(LAT)) in the tripod/world frame.
    model.articulation(
        "azimuth_rotation",
        ArticulationType.CONTINUOUS,
        parent=tripod,
        child=head,
        origin=Origin(
            xyz=(0.0, 0.0, TRIPOD_TOP_Z),
            rpy=(LAT_ANGLE - math.pi / 2.0, 0.0, 0.0),
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0),
    )

    # =====================================================================
    # OPTICAL TUBE — declination (DEC) rotation about an axis perpendicular
    # to the polar axis, at the top of the polar housing. The tube extends
    # along local +X (objective at +X, focuser at -X). Positive DEC raises
    # the objective end toward the celestial pole.
    # =====================================================================
    tube = model.part("optical_tube")

    tube.visual(mesh_from_cadquery(_tube_shell(), "tube_shell"), material=white, name="tube_shell")

    # Blue band wrapping the mid/rear of the tube (slightly proud of the shell).
    tube.visual(
        Cylinder(radius=TUBE_R + 0.0015, length=0.150),
        origin=Origin(xyz=(-0.040, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blue,
        name="blue_band",
    )
    # Black dew shield / objective ring at the front mouth.
    tube.visual(
        Cylinder(radius=TUBE_R + 0.003, length=0.030),
        origin=Origin(xyz=(TUBE_FRONT_X - 0.013, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="dew_shield",
    )
    # Brass trim ring just behind the dew shield.
    tube.visual(
        Cylinder(radius=TUBE_R + 0.0035, length=0.008),
        origin=Origin(xyz=(TUBE_FRONT_X - 0.034, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="objective_ring",
    )
    # Brass cradle ring where the tube sits in the yoke (carries the tilt axis).
    tube.visual(
        Cylinder(radius=TUBE_R + 0.004, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="cradle_ring",
    )
    # Brass focuser housing block at the rear of the tube.
    tube.visual(
        Cylinder(radius=0.016, length=0.030),
        origin=Origin(xyz=(TUBE_REAR_X + 0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="focuser_housing",
    )
    # Focuser knob on the side.
    tube.visual(
        Cylinder(radius=0.007, length=0.016),
        origin=Origin(xyz=(TUBE_REAR_X + 0.010, TUBE_R + 0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="focus_knob",
    )

    tube.inertial = Inertial.from_geometry(
        Cylinder(radius=TUBE_R, length=TUBE_LEN),
        mass=0.50,
        origin=Origin(xyz=(0.5 * (TUBE_FRONT_X + TUBE_REAR_X), 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    # DEC (tube_altitude): REVOLUTE about an axis perpendicular to the polar
    # axis. The rpy orients the articulation frame so its Z = head -Y (the
    # DEC axis direction in the head frame). At q=0 the tube extends along
    # head X (= world X at RA=0). Positive DEC raises the objective end.
    model.articulation(
        "tube_altitude",
        ArticulationType.REVOLUTE,
        parent=head,
        child=tube,
        origin=Origin(
            xyz=(0.0, 0.0, POLAR_LEN),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.0,
            lower=-math.radians(15.0),
            upper=math.radians(75.0),
        ),
    )

    # =====================================================================
    # FOCUSER DRAW-TUBE — prismatic slide out the back of the tube, carrying
    # a small eyepiece.  At rest it is mostly inserted into the focuser
    # housing; extending it grows the overall tube length.
    # =====================================================================
    draw = model.part("focuser_drawtube")
    # Draw-tube barrel reaching forward (+X) into the focuser housing so it stays
    # captured at full rearward extension.  Local +X is forward (into the tube).
    draw.visual(
        Cylinder(radius=0.010, length=0.090),
        origin=Origin(xyz=(0.030, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="drawtube_barrel",
    )
    # Eyepiece at the rear (-X) end.
    draw.visual(
        Cylinder(radius=0.013, length=0.022),
        origin=Origin(xyz=(-0.026, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="eyepiece_body",
    )
    draw.visual(
        Cylinder(radius=0.009, length=0.010),
        origin=Origin(xyz=(-0.042, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="eyepiece_cup",
    )
    draw.inertial = Inertial.from_geometry(
        Box((0.130, 0.026, 0.026)),
        mass=0.04,
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
    )

    # Joint frame at the rear of the tube; +axis points -X so positive travel
    # slides the draw-tube rearward (out of the tube).
    model.articulation(
        "focuser_slide",
        ArticulationType.PRISMATIC,
        parent=tube,
        child=draw,
        origin=Origin(xyz=(TUBE_REAR_X + 0.010, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.05, lower=0.0, upper=0.030),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tripod = object_model.get_part("tripod")
    head = object_model.get_part("azimuth_head")
    tube = object_model.get_part("optical_tube")
    draw = object_model.get_part("focuser_drawtube")

    az = object_model.get_articulation("azimuth_rotation")
    dec = object_model.get_articulation("tube_altitude")
    slide = object_model.get_articulation("focuser_slide")

    # ---- Tripod: three feet on the ground, widest footprint at the bottom ----
    foot_zs = []
    foot_xy = []
    for i in range(3):
        ab = ctx.part_element_world_aabb(tripod, elem=f"foot_tip_{i}")
        mn, mx = ab
        foot_zs.append(0.5 * (mn[2] + mx[2]))
        foot_xy.append((0.5 * (mn[0] + mx[0]), 0.5 * (mn[1] + mx[1])))
    ctx.check(
        "three feet rest near the ground",
        all(z < 0.03 for z in foot_zs),
        details=f"foot center z={foot_zs}",
    )
    foot_spread = max(math.hypot(x, y) for x, y in foot_xy)
    hub_ab = ctx.part_element_world_aabb(tripod, elem="hub")
    hub_r = max(abs(hub_ab[0][0]), abs(hub_ab[1][0]))
    ctx.check(
        "feet splay wider than the hub (widest footprint at the bottom)",
        foot_spread > hub_r + 0.05,
        details=f"foot_spread={foot_spread}, hub_r={hub_r}",
    )

    # ---- EQ head sits on the tripod top; tube at the DEC axis ----
    # The tilted polar wedge seats onto the hub with a small local embed.
    ctx.allow_overlap(
        head,
        tripod,
        elem_a="polar_wedge",
        elem_b="hub",
        reason="The tilted polar wedge seats onto the tripod hub; the wedge bottom face is perpendicular to the polar axis, creating a small local embed into the horizontal hub top.",
    )
    ctx.allow_overlap(
        head,
        tripod,
        elem_a="polar_housing",
        elem_b="hub",
        reason="The polar housing cylinder extends to the base of the head; its tilted bottom face creates a small local embed into the hub top at the seating interface.",
    )
    # Proof: head is seated on tripod (contact or small gap at the seating face).
    ctx.expect_overlap(
        head,
        tripod,
        axes="z",
        elem_a="polar_wedge",
        elem_b="hub",
        min_overlap=0.001,
        name="polar wedge seated on the hub (Z overlap at contact)",
    )
    ctx.expect_contact(head, tripod, name="EQ head seated on tripod")
    head_z = ctx.part_world_position(head)[2]
    ctx.check(
        "EQ head sits at the top of the tripod",
        head_z > 0.28,
        details=f"head_z={head_z}",
    )

    # ---- EQ-specific: polar housing and counterweight exist on the head ----
    polar_ab = ctx.part_element_world_aabb(head, elem="polar_housing")
    cw_ab = ctx.part_element_world_aabb(head, elem="cw_ball")
    ctx.check(
        "polar_housing exists on the EQ head",
        polar_ab is not None and (polar_ab[1][2] - polar_ab[0][2]) > 0.04,
        details=f"polar_housing z-extent={polar_ab[1][2] - polar_ab[0][2] if polar_ab else None}",
    )
    ctx.check(
        "cw_ball is on the opposite side of the DEC axis from the tube",
        cw_ab is not None and 0.5 * (cw_ab[0][0] + cw_ab[1][0]) < -0.05,
        details=f"cw_ball center x={0.5 * (cw_ab[0][0] + cw_ab[1][0]) if cw_ab else None}",
    )

    # ---- Polar axis is tilted (not vertical): the ra_bearing top is offset
    #      in Y from the polar_wedge bottom, proving the head frame is tilted.
    wedge_ab = ctx.part_element_world_aabb(head, elem="polar_wedge")
    bearing_ab = ctx.part_element_world_aabb(head, elem="ra_bearing")
    wedge_y = 0.5 * (wedge_ab[0][1] + wedge_ab[1][1])
    bearing_y = 0.5 * (bearing_ab[0][1] + bearing_ab[1][1])
    ctx.check(
        "polar axis is tilted from vertical (ra_bearing Y offset from polar_wedge)",
        abs(bearing_y - wedge_y) > 0.02,
        details=f"wedge_y={wedge_y}, bearing_y={bearing_y}",
    )

    # ---- RA rotation: the off-axis ra_marker orbits around the tilted polar
    #      axis, so its XY position changes with a quarter turn. ----
    marker0 = ctx.part_element_world_aabb(head, elem="ra_marker")
    m0 = (0.5 * (marker0[0][0] + marker0[1][0]), 0.5 * (marker0[0][1] + marker0[1][1]))
    with ctx.pose({az: math.pi / 2.0}):
        marker1 = ctx.part_element_world_aabb(head, elem="ra_marker")
        m1 = (0.5 * (marker1[0][0] + marker1[1][0]), 0.5 * (marker1[0][1] + marker1[1][1]))
    ctx.check(
        "RA rotation moves the off-axis marker around the tilted polar axis",
        math.hypot(m1[0] - m0[0], m1[1] - m0[1]) > 0.02,
        details=f"marker rest={m0}, quarter-turn={m1}",
    )

    # ---- DEC tilt: objective (front) end rises when DEC increases ----
    # Allow tube/dec_bar overlap: the tube shell wraps around the DEC axis.
    ctx.allow_overlap(
        tube,
        head,
        elem_a="tube_shell",
        elem_b="dec_bar",
        reason="The optical tube shell wraps around the DEC bar at the cradle-ring mount point.",
    )
    ctx.allow_overlap(
        tube,
        head,
        elem_a="cradle_ring",
        elem_b="dec_bar",
        reason="The brass cradle ring surrounds the DEC bar as the tube mounting clamp.",
    )
    ctx.allow_overlap(
        tube,
        head,
        elem_a="tube_shell",
        elem_b="cw_shaft",
        reason="The CW shaft emerges from the DEC bar through the tube shell proxy at the mount point.",
    )
    ctx.allow_overlap(
        tube,
        head,
        elem_a="tube_shell",
        elem_b="ra_bearing",
        reason="The RA bearing cap sits just below the tube shell at the DEC axis intersection.",
    )
    ctx.allow_overlap(
        tube,
        head,
        elem_a="cradle_ring",
        elem_b="ra_bearing",
        reason="The cradle ring surrounds the DEC axis and embeds around the RA bearing cap at the mount intersection.",
    )
    ctx.allow_overlap(
        tube,
        head,
        elem_a="blue_band",
        elem_b="ra_bearing",
        reason="The blue band wrap passes over the RA bearing at the DEC axis mount point.",
    )
    ctx.allow_overlap(
        tube,
        head,
        elem_a="blue_band",
        elem_b="dec_bar",
        reason="The blue band wrap passes over the DEC bar at the tube mount point.",
    )
    ctx.allow_overlap(
        tube,
        head,
        elem_a="blue_band",
        elem_b="cw_shaft",
        reason="The CW shaft root passes through the blue band wrap near the DEC axis mount.",
    )
    ctx.allow_overlap(
        tube,
        head,
        elem_a="blue_band",
        elem_b="polar_shaft",
        reason="The upper polar shaft passes through the blue band at the DEC axis intersection.",
    )
    ctx.allow_overlap(
        tube,
        head,
        elem_a="tube_shell",
        elem_b="polar_shaft",
        reason="The upper polar shaft passes through the tube shell at the DEC axis intersection.",
    )
    ctx.allow_overlap(
        tube,
        head,
        elem_a="cradle_ring",
        elem_b="polar_shaft",
        reason="The cradle ring surrounds the DEC axis and passes near the upper polar shaft at the mount intersection.",
    )
    # Proof: tube is mounted at the DEC axis (overlap in X at the mount point).
    ctx.expect_overlap(
        tube,
        head,
        axes="x",
        elem_a="cradle_ring",
        elem_b="dec_bar",
        min_overlap=0.010,
        name="tube cradle ring engaged with the DEC bar",
    )
    # The CW shaft and drawtube barrel converge near the DEC axis mount point.
    ctx.allow_overlap(
        head,
        draw,
        elem_a="cw_shaft",
        elem_b="drawtube_barrel",
        reason="The CW shaft and drawtube barrel both pass near the DEC axis mount; small local embedding at the shaft root.",
    )
    ctx.allow_overlap(
        head,
        draw,
        elem_a="cw_ball",
        elem_b="drawtube_barrel",
        reason="The CW ball hangs near the drawtube barrel at the rear of the assembly.",
    )

    front0 = ctx.part_element_world_aabb(tube, elem="dew_shield")
    front_z0 = 0.5 * (front0[0][2] + front0[1][2])
    with ctx.pose({dec: math.radians(55.0)}):
        front_up = ctx.part_element_world_aabb(tube, elem="dew_shield")
        front_z_up = 0.5 * (front_up[0][2] + front_up[1][2])
    with ctx.pose({dec: math.radians(-10.0)}):
        front_dn = ctx.part_element_world_aabb(tube, elem="dew_shield")
        front_z_dn = 0.5 * (front_dn[0][2] + front_dn[1][2])
    ctx.check(
        "positive DEC raises the objective end",
        front_z_up > front_z0 + 0.03,
        details=f"rest={front_z0}, up={front_z_up}",
    )
    ctx.check(
        "negative DEC lowers the objective end",
        front_z_dn < front_z0 - 0.005,
        details=f"rest={front_z0}, down={front_z_dn}",
    )

    # ---- Focuser draw-tube: stays captured at rest, slides out rearward ----
    ctx.allow_overlap(
        draw,
        tube,
        elem_a="drawtube_barrel",
        elem_b="focuser_housing",
        reason="The draw-tube barrel is intentionally inserted into the brass focuser housing proxy.",
    )
    ctx.allow_overlap(
        draw,
        tube,
        elem_a="drawtube_barrel",
        elem_b="tube_shell",
        reason="The draw-tube barrel slides inside the solid rear of the optical-tube shell proxy.",
    )
    ctx.allow_overlap(
        draw,
        tube,
        elem_a="drawtube_barrel",
        elem_b="blue_band",
        reason="The draw-tube barrel slides inside the optical tube, passing under the blue band wrap.",
    )
    ctx.expect_within(
        draw,
        tube,
        axes="yz",
        inner_elem="drawtube_barrel",
        outer_elem="focuser_housing",
        margin=0.004,
        name="draw-tube centered in the focuser housing",
    )
    ctx.expect_overlap(
        draw,
        tube,
        axes="x",
        elem_a="drawtube_barrel",
        elem_b="focuser_housing",
        min_overlap=0.010,
        name="draw-tube inserted in housing at rest",
    )

    len0 = _ext(ctx.part_world_aabb(tube))[0] + 0.0  # rest reference along X
    tube_rear0 = ctx.part_world_aabb(tube)[0][0]
    draw_rear0 = ctx.part_world_aabb(draw)[0][0]
    with ctx.pose({slide: 0.030}):
        draw_rear1 = ctx.part_world_aabb(draw)[0][0]
        ctx.expect_overlap(
            draw,
            tube,
            axes="x",
            elem_a="drawtube_barrel",
            elem_b="focuser_housing",
            min_overlap=0.004,
            name="draw-tube stays captured when extended",
        )
    ctx.check(
        "focuser draw-tube slides out the back, extending the assembly",
        draw_rear1 < draw_rear0 - 0.02,
        details=f"draw rear rest={draw_rear0}, extended={draw_rear1}",
    )
    ctx.check(
        "extended draw-tube reaches behind the tube body",
        draw_rear1 < tube_rear0,
        details=f"draw_rear_extended={draw_rear1}, tube_rear={tube_rear0}, len0={len0}",
    )

    return ctx.report()


object_model = build_object_model()
