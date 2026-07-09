from __future__ import annotations

# Small Newtonian reflector telescope on an adjustable tripod.
# Frame:
#   - World Z is up. The tripod stands on the ground (feet near z=0).
#   - The alt-az head sits on top of the tripod and swings in azimuth about +Z.
#   - The optical tube points along the head's local +X at rest; the open
#     front mouth is at +X, the rear mirror cell is at -X.  The side-mounted
#     focuser with eyepiece protrudes radially upward (+Z) near the front.
#   - The tube tilts about the horizontal +Y axis.
# Articulations:
#   - azimuth_rotation: CONTINUOUS about +Z at the mount (head swings left/right).
#   - tube_altitude:    REVOLUTE about -Y at the yoke (front rises on positive q).
#   - focuser_slide:    PRISMATIC along +Z (draw-tube slides radially outward
#                       from the side-mounted focuser housing).

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
TUBE_R = 0.032  # reflector tube outer radius (fat for a small scope)
TUBE_LEN = 0.190  # short tube length
TUBE_REAR_X = -0.085
TUBE_FRONT_X = TUBE_REAR_X + TUBE_LEN  # 0.105
SPIDER_X = TUBE_FRONT_X - 0.025  # 0.080 — spider vane station near front
FOCUSER_X = TUBE_FRONT_X - 0.040  # 0.065 — side focuser station


def _reflector_tube_shell() -> cq.Workplane:
    """Fat short hollow cylinder along +X with open front and focuser port."""
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
        .workplane(offset=TUBE_REAR_X - 0.010)
        .circle(TUBE_R - 0.003)
        .workplane(offset=TUBE_LEN + 0.020)
        .circle(TUBE_R - 0.003)
        .loft(ruled=True)
    )
    tube = outer.cut(bore)

    # Cut a focuser port hole on the top (+Z side) of the tube near the front.
    # Build the cutter from a fixed global workplane to avoid drift.
    port = (
        cq.Workplane("XY")
        .workplane(offset=TUBE_R - 0.005)
        .transformed(offset=(FOCUSER_X, 0.0, 0.0))
        .circle(0.011)
        .extrude(0.012)
    )
    tube = tube.cut(port)

    return tube


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="newtonian_telescope")

    # --- materials: matte-white / gloss-black reflector livery ---
    white = model.material("tube_white", rgba=(0.90, 0.92, 0.95, 1.0))
    gloss_black = model.material("gloss_black", rgba=(0.05, 0.05, 0.07, 1.0))
    black = model.material("matte_black", rgba=(0.10, 0.10, 0.12, 1.0))
    brass = model.material("brass", rgba=(0.78, 0.62, 0.28, 1.0))
    metal = model.material("tripod_metal", rgba=(0.70, 0.72, 0.76, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.28, 0.30, 0.33, 1.0))
    foot_rubber = model.material("foot_rubber", rgba=(0.12, 0.14, 0.20, 1.0))
    mirror_surf = model.material("mirror_surface", rgba=(0.82, 0.84, 0.88, 1.0))

    # =====================================================================
    # TRIPOD (root part) — hub on top, three splayed legs to pointed feet,
    # plus a spreader ring tying the legs together.
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
    # AZIMUTH HEAD — turntable plate + U-shaped yoke carrying the tube tilt
    # axis.  Swings about +Z on top of the tripod.
    # =====================================================================
    head = model.part("azimuth_head")
    YOKE_HALF = 0.038
    TILT_Z = 0.090

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
            Box((0.020, 0.010, 0.072)),
            origin=Origin(xyz=(0.0, yy, 0.054)),
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
        Box((0.06, 0.08, 0.07)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
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
    # OPTICAL TUBE — Newtonian reflector OTA
    # Fat short hollow tube with open front mouth, rear mirror cell, spider
    # vane assembly near the front, and a side-mounted focuser near the
    # front top aimed radially outward (+Z).
    # =====================================================================
    tube = model.part("optical_tube")

    # Hollow tube shell (CadQuery: open cylinder with focuser port)
    tube.visual(mesh_from_cadquery(_reflector_tube_shell(), "tube_shell"), material=white, name="tube_shell")

    # Rear mirror cell cap
    tube.visual(
        Cylinder(radius=TUBE_R - 0.002, length=0.006),
        origin=Origin(xyz=(TUBE_REAR_X + 0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gloss_black,
        name="mirror_cell",
    )
    # Primary mirror face (visible inside the rear bore)
    tube.visual(
        Cylinder(radius=TUBE_R - 0.006, length=0.004),
        origin=Origin(xyz=(TUBE_REAR_X + 0.008, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mirror_surf,
        name="primary_mirror",
    )

    # --- Spider vane assembly: 4 thin vanes + central secondary hub ---
    HUB_R = 0.007
    VANE_LEN = 0.024
    VANE_CENTER_R = 0.018  # radial distance from tube axis to vane midpoint
    # Vane specs: (axis_along, sign) — 4 vanes at 90° intervals
    vane_specs = [("y", 1.0), ("y", -1.0), ("z", 1.0), ("z", -1.0)]
    for i, (ax, sign) in enumerate(vane_specs):
        if ax == "y":
            dims = (0.006, VANE_LEN, 0.0015)
            cy = sign * VANE_CENTER_R
            cz = 0.0
        else:
            dims = (0.006, 0.0015, VANE_LEN)
            cy = 0.0
            cz = sign * VANE_CENTER_R
        tube.visual(
            Box(dims),
            origin=Origin(xyz=(SPIDER_X, cy, cz)),
            material=gloss_black,
            name=f"spider_vane_{i}",
        )

    # Central secondary mirror hub
    tube.visual(
        Cylinder(radius=HUB_R, length=0.012),
        origin=Origin(xyz=(SPIDER_X, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gloss_black,
        name="secondary_mirror_hub",
    )
    # Secondary mirror face (small flat, tilted 45°)
    tube.visual(
        Box((0.002, 0.010, 0.010)),
        origin=Origin(xyz=(SPIDER_X + 0.005, 0.0, 0.0), rpy=(0.0, 0.0, math.pi / 4.0)),
        material=mirror_surf,
        name="secondary_mirror",
    )

    # Brass cradle ring (yoke tilt interface)
    tube.visual(
        Cylinder(radius=TUBE_R + 0.004, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="cradle_ring",
    )

    # --- Side-mounted focuser housing on top of tube near front ---
    tube.visual(
        Cylinder(radius=0.014, length=0.026),
        origin=Origin(xyz=(FOCUSER_X, 0.0, TUBE_R + 0.013)),
        material=gloss_black,
        name="focuser_housing",
    )
    # Focus knob on the side of the housing (+Y direction)
    tube.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(FOCUSER_X, 0.019, TUBE_R + 0.013), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="focus_knob",
    )

    tube.inertial = Inertial.from_geometry(
        Cylinder(radius=TUBE_R, length=TUBE_LEN),
        mass=0.55,
        origin=Origin(xyz=(0.5 * (TUBE_FRONT_X + TUBE_REAR_X), 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
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
    # FOCUSER DRAW-TUBE — prismatic slide radially outward from the
    # side-mounted focuser housing.  At rest the barrel is mostly inserted;
    # extending pushes the eyepiece outward (+Z in tube frame).
    # =====================================================================
    draw = model.part("focuser_drawtube")

    # Draw-tube barrel (reaches into the housing and tube bore)
    draw.visual(
        Cylinder(radius=0.009, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=brass,
        name="drawtube_barrel",
    )
    # Eyepiece body at the outer end
    draw.visual(
        Cylinder(radius=0.012, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=dark_metal,
        name="eyepiece_body",
    )
    # Eyepiece eye-lens cup
    draw.visual(
        Cylinder(radius=0.008, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=black,
        name="eyepiece_cup",
    )
    draw.inertial = Inertial.from_geometry(
        Box((0.026, 0.026, 0.100)),
        mass=0.04,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    # Joint frame at the tube outer surface where the housing mounts.
    # +Z axis so positive travel pushes the draw-tube radially outward.
    model.articulation(
        "focuser_slide",
        ArticulationType.PRISMATIC,
        parent=tube,
        child=draw,
        origin=Origin(xyz=(FOCUSER_X, 0.0, TUBE_R)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.05, lower=0.0, upper=0.025),
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

    # ---- Azimuth: head + tube swing about vertical; off-axis marker moves ----
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

    # ---- Altitude tilt: front (open mouth / spider) end rises when tilted up ----
    front0 = ctx.part_element_world_aabb(tube, elem="secondary_mirror_hub")
    front_z0 = 0.5 * (front0[0][2] + front0[1][2])
    with ctx.pose({tilt: math.radians(55.0)}):
        front_up = ctx.part_element_world_aabb(tube, elem="secondary_mirror_hub")
        front_z_up = 0.5 * (front_up[0][2] + front_up[1][2])
    with ctx.pose({tilt: math.radians(-25.0)}):
        front_dn = ctx.part_element_world_aabb(tube, elem="secondary_mirror_hub")
        front_z_dn = 0.5 * (front_dn[0][2] + front_dn[1][2])
    ctx.check(
        "tilting up raises the open-mouth (front) end",
        front_z_up > front_z0 + 0.03,
        details=f"rest={front_z0}, up={front_z_up}",
    )
    ctx.check(
        "tilting down lowers the open-mouth (front) end",
        front_z_dn < front_z0 - 0.01,
        details=f"rest={front_z0}, down={front_z_dn}",
    )

    # ---- Newtonian reflector form: focuser on the side, spider near front ----
    housing_ab = ctx.part_element_world_aabb(tube, elem="focuser_housing")
    cradle_ab = ctx.part_element_world_aabb(tube, elem="cradle_ring")
    cradle_z = 0.5 * (cradle_ab[0][2] + cradle_ab[1][2])
    ctx.check(
        "optical_tube: focuser_housing is side-mounted above the tube centerline",
        housing_ab[0][2] > cradle_z + 0.015,
        details=f"housing_bottom_z={housing_ab[0][2]}, cradle_center_z={cradle_z}",
    )

    spider_ab = ctx.part_element_world_aabb(tube, elem="secondary_mirror_hub")
    mirror_cell_ab = ctx.part_element_world_aabb(tube, elem="mirror_cell")
    ctx.check(
        "optical_tube: secondary_mirror_hub is near the front, ahead of the mirror_cell",
        spider_ab[0][0] > mirror_cell_ab[0][0] + 0.05,
        details=f"spider_min_x={spider_ab[0][0]}, mirror_cell_min_x={mirror_cell_ab[0][0]}",
    )

    # ---- Focuser draw-tube: side-mounted prismatic slide along +Z ----
    ctx.allow_overlap(
        draw,
        tube,
        elem_a="drawtube_barrel",
        elem_b="focuser_housing",
        reason="The draw-tube barrel is intentionally inserted into the side-mounted focuser housing proxy.",
    )
    ctx.allow_overlap(
        draw,
        tube,
        elem_a="drawtube_barrel",
        elem_b="tube_shell",
        reason="The draw-tube barrel passes through the focuser port in the tube wall into the bore.",
    )
    ctx.allow_overlap(
        draw,
        tube,
        elem_a="eyepiece_body",
        elem_b="focuser_housing",
        reason="The eyepiece body sits partially inside the focuser housing at rest as part of the nested draw-tube assembly.",
    )

    ctx.expect_within(
        draw,
        tube,
        axes="xy",
        inner_elem="drawtube_barrel",
        outer_elem="focuser_housing",
        margin=0.004,
        name="draw-tube centered in the focuser housing (xy)",
    )
    ctx.expect_within(
        draw,
        tube,
        axes="xy",
        inner_elem="eyepiece_body",
        outer_elem="focuser_housing",
        margin=0.004,
        name="eyepiece body stays within housing bore (xy)",
    )
    ctx.expect_overlap(
        draw,
        tube,
        axes="z",
        elem_a="drawtube_barrel",
        elem_b="focuser_housing",
        min_overlap=0.008,
        name="draw-tube inserted in housing at rest",
    )

    draw_top0 = ctx.part_world_aabb(draw)[1][2]
    with ctx.pose({slide: 0.025}):
        draw_top1 = ctx.part_world_aabb(draw)[1][2]
        ctx.expect_overlap(
            draw,
            tube,
            axes="z",
            elem_a="drawtube_barrel",
            elem_b="focuser_housing",
            min_overlap=0.004,
            name="draw-tube stays captured when extended",
        )
    ctx.check(
        "focuser_slide pushes draw-tube radially outward (upward at rest pose)",
        draw_top1 > draw_top0 + 0.015,
        details=f"draw top rest={draw_top0}, extended={draw_top1}",
    )

    return ctx.report()


object_model = build_object_model()
