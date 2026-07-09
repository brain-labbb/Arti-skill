from __future__ import annotations

# Maksutov-Cassegrain catadioptric telescope on an adjustable tripod.
# Frame:
#   - World Z is up. The tripod stands on the ground (feet near z=0).
#   - The alt-az head sits on top of the tripod and swings in azimuth about +Z.
#   - The optical tube (short stubby Mak OTA) points along +X at rest; the
#     front corrector lens is at +X (front), the focuser drawtube and eyepiece
#     are at -X (rear). The tube tilts about the horizontal -Y axis.
# Articulations:
#   - azimuth_rotation: CONTINUOUS about +Z at the mount (head swings).
#   - tube_altitude:    REVOLUTE about -Y at the yoke (objective rises/falls).
#   - focuser_slide:    PRISMATIC along -X (draw-tube slides in/out the back).

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
TRIPOD_TOP_Z = 0.320

# Maksutov-Cassegrain OTA: short stubby closed cylinder, roughly as wide as long.
TUBE_LEN = 0.105          # main body length along X
TUBE_R = 0.044            # outer radius (diameter ~0.088, D/L ~ 0.84)
TUBE_REAR_X = -0.0525
TUBE_FRONT_X = 0.0525

# Rear visual-back cell extends behind the main tube body.
REAR_CELL_LEN = 0.022
REAR_CELL_R = TUBE_R * 0.82

# Focuser housing centre on the rear-cell back face.
FOCUSER_X = TUBE_REAR_X - REAR_CELL_LEN - 0.004  # ~ -0.0785


# ---------------------------------------------------------------------------
# CadQuery helpers for catadioptric OTA geometry
# ---------------------------------------------------------------------------

def _mak_tube_shell() -> cq.Workplane:
    """Short stubby hollow cylinder for the Maksutov OTA body."""
    outer = (
        cq.Workplane("YZ")
        .workplane(offset=TUBE_REAR_X)
        .circle(TUBE_R)
        .workplane(offset=TUBE_LEN)
        .circle(TUBE_R)
        .loft(ruled=True)
    )
    bore = (
        cq.Workplane("YZ")
        .workplane(offset=TUBE_REAR_X + 0.004)
        .circle(TUBE_R - 0.003)
        .workplane(offset=TUBE_LEN - 0.008)
        .circle(TUBE_R - 0.003)
        .loft(ruled=True)
    )
    return outer.cut(bore)


def _corrector_lens() -> cq.Workplane:
    """Front corrector meniscus — thin glass disc closing the aperture."""
    return (
        cq.Workplane("YZ")
        .workplane(offset=TUBE_FRONT_X - 0.005)
        .circle(TUBE_R - 0.003)
        .workplane(offset=0.005)
        .circle(TUBE_R - 0.004)
        .loft()
    )


def _rear_cell_body() -> cq.Workplane:
    """Rear cylindrical visual back, slightly narrower than the main tube."""
    return (
        cq.Workplane("YZ")
        .workplane(offset=TUBE_REAR_X - REAR_CELL_LEN)
        .circle(REAR_CELL_R)
        .workplane(offset=REAR_CELL_LEN + 0.003)
        .circle(REAR_CELL_R * 1.03)
        .loft(ruled=True)
    )


def _front_ring() -> cq.Workplane:
    """Annular retaining ring around the corrector at the front mouth."""
    outer_r = TUBE_R + 0.004
    inner_r = TUBE_R - 0.001
    x0 = TUBE_FRONT_X - 0.009
    length = 0.011
    outer = (
        cq.Workplane("YZ")
        .workplane(offset=x0)
        .circle(outer_r)
        .workplane(offset=length)
        .circle(outer_r)
        .loft(ruled=True)
    )
    bore = (
        cq.Workplane("YZ")
        .workplane(offset=x0 - 0.001)
        .circle(inner_r)
        .workplane(offset=length + 0.002)
        .circle(inner_r)
        .loft(ruled=True)
    )
    return outer.cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mak_telescope")

    # --- materials (pearl-white / orange catadioptric livery) ---
    pearl = model.material("pearl_white", rgba=(0.92, 0.90, 0.86, 1.0))
    orange = model.material("tube_orange", rgba=(0.85, 0.45, 0.12, 1.0))
    black = model.material("matte_black", rgba=(0.10, 0.10, 0.12, 1.0))
    brass = model.material("brass", rgba=(0.78, 0.62, 0.28, 1.0))
    metal = model.material("tripod_metal", rgba=(0.70, 0.72, 0.76, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.28, 0.30, 0.33, 1.0))
    foot_rubber = model.material("foot_rubber", rgba=(0.12, 0.14, 0.20, 1.0))
    glass = model.material("corrector_glass", rgba=(0.72, 0.80, 0.88, 0.55))

    # =====================================================================
    # TRIPOD (root part) — identical to parent baseline
    # =====================================================================
    tripod = model.part("tripod")

    HUB_Z = TRIPOD_TOP_Z - 0.020
    LEG_TOP_R = 0.026
    FOOT_R = 0.150
    SPREADER_Z = 0.110

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

    leg_angles = (
        math.pi / 2.0,
        math.pi / 2.0 + 2.0 * math.pi / 3.0,
        math.pi / 2.0 + 4.0 * math.pi / 3.0,
    )
    for i, ang in enumerate(leg_angles):
        c, s = math.cos(ang), math.sin(ang)
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
    # AZIMUTH HEAD — wider yoke cheeks to accommodate the stubby Mak OTA
    # =====================================================================
    head = model.part("azimuth_head")
    YOKE_HALF = 0.054   # half-spacing between cheeks (along Y)
    TILT_Z = 0.102       # tilt-axis height above head origin

    head.visual(
        Cylinder(radius=0.030, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_metal,
        name="az_turntable",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=metal,
        name="az_post",
    )
    head.visual(
        Box((0.020, 2.0 * YOKE_HALF + 0.010, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=metal,
        name="yoke_base",
    )
    for side, yy in (("a", YOKE_HALF), ("b", -YOKE_HALF)):
        head.visual(
            Box((0.020, 0.010, 0.068)),
            origin=Origin(xyz=(0.0, yy, 0.068)),
            material=metal,
            name=f"yoke_cheek_{side}",
        )
        head.visual(
            Cylinder(radius=0.010, length=0.012),
            origin=Origin(xyz=(0.0, yy, TILT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brass,
            name=f"tilt_boss_{side}",
        )
    head.visual(
        Box((0.012, 0.010, 0.014)),
        origin=Origin(xyz=(0.026, 0.0, 0.020)),
        material=brass,
        name="azimuth_marker",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.06, 0.12, 0.08)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
    )

    model.articulation(
        "azimuth_rotation",
        ArticulationType.CONTINUOUS,
        parent=tripod,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, TRIPOD_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0),
    )

    # =====================================================================
    # OPTICAL TUBE — Maksutov-Cassegrain catadioptric OTA
    # Short stubby wide closed cylinder with front corrector and rear cell.
    # =====================================================================
    tube = model.part("optical_tube")

    # Main tube shell — pearl-white hollow cylinder
    tube.visual(
        mesh_from_cadquery(_mak_tube_shell(), "tube_shell"),
        material=pearl,
        name="tube_shell",
    )

    # Front corrector meniscus lens — glass disc closing the aperture
    tube.visual(
        mesh_from_cadquery(_corrector_lens(), "corrector_lens"),
        material=glass,
        name="corrector_lens",
    )

    # Orange retaining ring around the corrector
    tube.visual(
        mesh_from_cadquery(_front_ring(), "front_ring"),
        material=orange,
        name="front_ring",
    )

    # Secondary mirror spot on the corrector (characteristic Mak feature)
    tube.visual(
        Cylinder(radius=0.011, length=0.003),
        origin=Origin(xyz=(TUBE_FRONT_X - 0.001, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="secondary_mirror",
    )

    # Rear cylindrical visual back — black cell behind the main tube
    tube.visual(
        mesh_from_cadquery(_rear_cell_body(), "rear_cell"),
        material=black,
        name="rear_cell",
    )

    # Orange accent band wrapping the front third of the tube
    tube.visual(
        Cylinder(radius=TUBE_R + 0.002, length=0.022),
        origin=Origin(xyz=(0.024, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=orange,
        name="orange_band",
    )

    # Brass cradle ring — yoke tilt interface
    tube.visual(
        Cylinder(radius=TUBE_R + 0.004, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="cradle_ring",
    )

    # Brass focuser housing on the rear cell back face
    tube.visual(
        Cylinder(radius=0.018, length=0.024),
        origin=Origin(xyz=(FOCUSER_X, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="focuser_housing",
    )

    # Focus knob protruding from the housing side
    tube.visual(
        Cylinder(radius=0.007, length=0.022),
        origin=Origin(xyz=(FOCUSER_X, 0.018 + 0.011, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="focus_knob",
    )

    tube.inertial = Inertial.from_geometry(
        Cylinder(radius=TUBE_R, length=TUBE_LEN + REAR_CELL_LEN),
        mass=0.65,
        origin=Origin(
            xyz=(0.5 * (TUBE_FRONT_X + TUBE_REAR_X - REAR_CELL_LEN), 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
    )

    model.articulation(
        "tube_altitude",
        ArticulationType.REVOLUTE,
        parent=head,
        child=tube,
        origin=Origin(xyz=(0.0, 0.0, TILT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.0,
            lower=-math.radians(30.0),
            upper=math.radians(60.0),
        ),
    )

    # =====================================================================
    # FOCUSER DRAW-TUBE — prismatic slide out the back
    # =====================================================================
    draw = model.part("focuser_drawtube")

    # Draw-tube barrel reaching forward into the focuser housing
    draw.visual(
        Cylinder(radius=0.010, length=0.070),
        origin=Origin(xyz=(0.025, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="drawtube_barrel",
    )
    # Eyepiece body at the rear end
    draw.visual(
        Cylinder(radius=0.013, length=0.022),
        origin=Origin(xyz=(-0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="eyepiece_body",
    )
    # Eyepiece rubber cup
    draw.visual(
        Cylinder(radius=0.009, length=0.010),
        origin=Origin(xyz=(-0.036, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="eyepiece_cup",
    )
    draw.inertial = Inertial.from_geometry(
        Box((0.110, 0.026, 0.026)),
        mass=0.04,
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
    )

    # Joint frame at the focuser; +axis is -X so positive travel pushes
    # the draw-tube rearward (out of the OTA).
    model.articulation(
        "focuser_slide",
        ArticulationType.PRISMATIC,
        parent=tube,
        child=draw,
        origin=Origin(xyz=(FOCUSER_X, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.05, lower=0.0, upper=0.030),
    )

    return model


# ---------------------------------------------------------------------------
# Test helpers
# ---------------------------------------------------------------------------

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
    tilt = object_model.get_articulation("tube_altitude")
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

    # ---- Head sits on the tripod top; tube held in the mount ----
    ctx.expect_contact(head, tripod, name="head seated on tripod")
    ctx.expect_contact(tube, head, name="tube held in the mount yoke")
    head_z = ctx.part_world_position(head)[2]
    ctx.check(
        "azimuth head sits at the top of the tripod",
        head_z > 0.28,
        details=f"head_z={head_z}",
    )

    # ---- Azimuth: head + tube swing about vertical ----
    marker0 = ctx.part_element_world_aabb(head, elem="azimuth_marker")
    m0 = (0.5 * (marker0[0][0] + marker0[1][0]), 0.5 * (marker0[0][1] + marker0[1][1]))
    with ctx.pose({az: math.pi / 2.0}):
        marker1 = ctx.part_element_world_aabb(head, elem="azimuth_marker")
        m1 = (0.5 * (marker1[0][0] + marker1[1][0]), 0.5 * (marker1[0][1] + marker1[1][1]))
    ctx.check(
        "azimuth rotation swings the off-axis marker around vertical",
        math.hypot(m1[0] - m0[0], m1[1] - m0[1]) > 0.02,
        details=f"marker rest={m0}, quarter-turn={m1}",
    )

    # ---- Altitude tilt: corrector (front) end rises when tilted up ----
    front0 = ctx.part_element_world_aabb(tube, elem="front_ring")
    front_z0 = 0.5 * (front0[0][2] + front0[1][2])
    with ctx.pose({tilt: math.radians(55.0)}):
        front_up = ctx.part_element_world_aabb(tube, elem="front_ring")
        front_z_up = 0.5 * (front_up[0][2] + front_up[1][2])
    with ctx.pose({tilt: math.radians(-25.0)}):
        front_dn = ctx.part_element_world_aabb(tube, elem="front_ring")
        front_z_dn = 0.5 * (front_dn[0][2] + front_dn[1][2])
    ctx.check(
        "tilting up raises the corrector end",
        front_z_up > front_z0 + 0.02,
        details=f"rest={front_z0}, up={front_z_up}",
    )
    ctx.check(
        "tilting down lowers the corrector end",
        front_z_dn < front_z0 - 0.005,
        details=f"rest={front_z0}, down={front_z_dn}",
    )

    # =====================================================================
    # CATADIOPTRIC OTA FORM CHECKS (variant-specific)
    # =====================================================================

    # Stubby tube: diameter should be at least 70% of length
    tube_shell_ab = ctx.part_element_world_aabb(tube, elem="tube_shell")
    tdx = tube_shell_ab[1][0] - tube_shell_ab[0][0]
    tdy = tube_shell_ab[1][1] - tube_shell_ab[0][1]
    tdz = tube_shell_ab[1][2] - tube_shell_ab[0][2]
    tube_diameter = max(tdy, tdz)
    ctx.check(
        "catadioptric OTA is stubby (diameter >= 70% of tube length)",
        tube_diameter >= 0.70 * tdx,
        details=f"length={tdx:.4f}, diameter={tube_diameter:.4f}, ratio={tube_diameter / max(tdx, 1e-9):.2f}",
    )

    # Corrector lens closes the front aperture (sits near the front face)
    corrector_ab = ctx.part_element_world_aabb(tube, elem="corrector_lens")
    ctx.check(
        "corrector lens present at front of tube (closes the aperture)",
        corrector_ab[1][0] >= tube_shell_ab[1][0] - 0.012,
        details=f"tube_front_x={tube_shell_ab[1][0]:.4f}, corrector_front_x={corrector_ab[1][0]:.4f}",
    )

    # Rear cell extends behind the main tube body
    rear_cell_ab = ctx.part_element_world_aabb(tube, elem="rear_cell")
    ctx.check(
        "rear cell extends behind main tube body",
        rear_cell_ab[0][0] < tube_shell_ab[0][0] - 0.005,
        details=f"tube_rear_x={tube_shell_ab[0][0]:.4f}, rear_cell_rear_x={rear_cell_ab[0][0]:.4f}",
    )

    # ---- Focuser draw-tube: captured at rest, slides out rearward ----
    ctx.allow_overlap(
        draw, tube,
        elem_a="drawtube_barrel", elem_b="focuser_housing",
        reason="The draw-tube barrel is intentionally inserted into the brass focuser housing proxy.",
    )
    ctx.allow_overlap(
        draw, tube,
        elem_a="drawtube_barrel", elem_b="tube_shell",
        reason="The draw-tube barrel slides inside the solid rear of the optical-tube shell proxy.",
    )
    ctx.allow_overlap(
        draw, tube,
        elem_a="drawtube_barrel", elem_b="orange_band",
        reason="The draw-tube barrel slides inside the optical tube, passing under the orange band wrap.",
    )
    ctx.allow_overlap(
        draw, tube,
        elem_a="drawtube_barrel", elem_b="rear_cell",
        reason="The draw-tube barrel passes through the rear cell visual back on its way into the focuser housing.",
    )
    ctx.allow_overlap(
        draw, tube,
        elem_a="drawtube_barrel", elem_b="cradle_ring",
        reason="The draw-tube barrel extends forward inside the tube past the cradle ring.",
    )

    ctx.expect_within(
        draw, tube,
        axes="yz",
        inner_elem="drawtube_barrel",
        outer_elem="focuser_housing",
        margin=0.004,
        name="draw-tube centered in the focuser housing",
    )
    ctx.expect_overlap(
        draw, tube,
        axes="x",
        elem_a="drawtube_barrel",
        elem_b="focuser_housing",
        min_overlap=0.008,
        name="draw-tube inserted in housing at rest",
    )

    tube_rear0 = ctx.part_world_aabb(tube)[0][0]
    draw_rear0 = ctx.part_world_aabb(draw)[0][0]
    with ctx.pose({slide: 0.030}):
        draw_rear1 = ctx.part_world_aabb(draw)[0][0]
        ctx.expect_overlap(
            draw, tube,
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
        details=f"draw_rear_extended={draw_rear1}, tube_rear={tube_rear0}",
    )

    return ctx.report()


object_model = build_object_model()
