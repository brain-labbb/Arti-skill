"""Large satellite ground-station antenna dish on a tripod ground stand.

Reference: white parabolic reflector on a three-leg splayed tripod stand
with a central hub carrying the azimuth turret. A quad-strut feed support
holds a subreflector over a central feed horn. The dish assembly rotates
in azimuth on top of the hub and tilts in elevation between the yoke cheeks.

Frames:
- Tripod root frame: z up, ground at z=0.
- Azimuth turret frame: at the hub top, spins about +Z.
- Dish frame: at the trunnion (elevation) axis. Boresight is local +Z at
  q_elev = 0 (dish pointing at zenith); positive elevation tilts the
  boresight toward the +X horizon.
"""

from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    CylinderGeometry,
    DomeGeometry,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# ---------------------------------------------------------------- dimensions
# Tripod ground stand (three-leg splayed support).
PED_H = 5.6
HUB_R = 1.25
HUB_H = 0.40
HUB_TOP_Z = PED_H + 0.16
HUB_BOTTOM_Z = HUB_TOP_Z - HUB_H
LEG_FOOT_R = 3.80
LEG_R = 0.18
FOOT_PAD_R = 0.50
FOOT_PAD_H = 0.12
BRACE_R = 0.07

# Azimuth turret.
TURRET_R = 1.05
TURRET_H = 0.55
YOKE_H = 1.15  # cheek height above turret top
TRUNNION_Z = TURRET_H + YOKE_H  # elevation axis height in turret frame

# Parabolic reflector.
DISH_R = 4.5  # 9 m aperture
FOCAL = 2.7
DISH_T = 0.07  # shell thickness
VERTEX_Z = 0.55  # dish vertex height above the trunnion axis
STRUT_MOUNT_R = 3.1  # radius where the feed struts leave the dish
STRUT_R = 0.085
APEX_Z = VERTEX_Z + FOCAL + 0.15  # apex hub center (just past the focus)

HORN_R = 0.16
HORN_LEN = 1.35

WHITE = Material(name="panel_white", color=(0.92, 0.92, 0.90, 1.0))
LIGHT_GRAY = Material(name="cladding_gray", color=(0.80, 0.81, 0.83, 1.0))
STEEL = Material(name="steel_gray", color=(0.58, 0.60, 0.63, 1.0))
DARK = Material(name="dark_gray", color=(0.30, 0.31, 0.33, 1.0))


def _parabola_z(r: float) -> float:
    return r * r / (4.0 * FOCAL)


def _strut_origin(mount_angle: float) -> tuple[Origin, float]:
    """Origin + length for a strut from the dish surface to the apex hub."""
    ax = STRUT_MOUNT_R * math.cos(mount_angle)
    ay = STRUT_MOUNT_R * math.sin(mount_angle)
    az = VERTEX_Z + _parabola_z(STRUT_MOUNT_R) - 0.04  # embedded in the shell
    bx, by, bz = 0.0, 0.0, APEX_Z
    dx, dy, dz = bx - ax, by - ay, bz - az
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    yaw = math.atan2(dy, dx)
    mid = ((ax + bx) / 2.0, (ay + by) / 2.0, (az + bz) / 2.0)
    return Origin(xyz=mid, rpy=(0.0, pitch, yaw)), length


def _leg_endpoint(ang: float, z: float) -> tuple[float, float, float]:
    """World position on a tripod leg at height z."""
    t = (z - FOOT_PAD_H) / (HUB_BOTTOM_Z - FOOT_PAD_H)
    r = LEG_FOOT_R + (HUB_R - LEG_FOOT_R) * t
    return (r * math.cos(ang), r * math.sin(ang), z)


def _tube_origin_and_length(
    pa: tuple[float, float, float], pb: tuple[float, float, float]
) -> tuple[Origin, float]:
    """Origin + length for a cylindrical member aligned from pa to pb."""
    dx, dy, dz = pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    mid = ((pa[0] + pb[0]) / 2.0, (pa[1] + pb[1]) / 2.0, (pa[2] + pb[2]) / 2.0)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    return Origin(xyz=mid, rpy=(0.0, pitch, yaw)), length


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ground_station_antenna_dish")

    # ----------------------------------------------------------- tripod stand
    pedestal = model.part("pedestal_building")

    # Central hub cylinder at the top of the tripod.
    pedestal.visual(
        mesh_from_geometry(
            CylinderGeometry(HUB_R, HUB_H, radial_segments=48),
            "tripod_hub",
        ),
        origin=Origin(xyz=(0.0, 0.0, HUB_BOTTOM_Z + HUB_H / 2.0)),
        material=LIGHT_GRAY,
        name="tripod_hub",
    )
    # Hub bearing plate for the azimuth turntable.
    pedestal.visual(
        mesh_from_geometry(
            CylinderGeometry(HUB_R + 0.06, 0.05, radial_segments=48),
            "hub_bearing_plate",
        ),
        origin=Origin(xyz=(0.0, 0.0, HUB_TOP_Z + 0.025)),
        material=STEEL,
        name="hub_bearing_plate",
    )

    # Three splayed legs with foot pads, gussets, and cross-braces.
    for i in range(3):
        ang = 2.0 * math.pi * i / 3.0
        foot = (
            LEG_FOOT_R * math.cos(ang),
            LEG_FOOT_R * math.sin(ang),
            FOOT_PAD_H,
        )
        top = (
            HUB_R * math.cos(ang),
            HUB_R * math.sin(ang),
            HUB_BOTTOM_Z,
        )
        leg_origin, leg_length = _tube_origin_and_length(foot, top)

        # Leg tube.
        pedestal.visual(
            mesh_from_geometry(
                CylinderGeometry(LEG_R, leg_length, radial_segments=16),
                f"leg_tube_{i}",
            ),
            origin=leg_origin,
            material=WHITE,
            name=f"leg_{i}",
        )
        # Ground foot pad (anchor plate).
        pedestal.visual(
            mesh_from_geometry(
                CylinderGeometry(FOOT_PAD_R, FOOT_PAD_H, radial_segments=24),
                f"foot_pad_{i}",
            ),
            origin=Origin(
                xyz=(
                    LEG_FOOT_R * math.cos(ang),
                    LEG_FOOT_R * math.sin(ang),
                    FOOT_PAD_H / 2.0,
                )
            ),
            material=STEEL,
            name=f"foot_pad_{i}",
        )
        # Gusset plate at leg-to-hub junction.
        pedestal.visual(
            Box((0.35, 0.08, 0.55)),
            origin=Origin(
                xyz=(
                    (HUB_R + 0.12) * math.cos(ang),
                    (HUB_R + 0.12) * math.sin(ang),
                    HUB_BOTTOM_Z + 0.20,
                ),
                rpy=(0.0, 0.0, ang),
            ),
            material=STEEL,
            name=f"gusset_{i}",
        )

    # Cross-braces between adjacent legs at two heights.
    for level, z_brace in enumerate((PED_H * 0.28, PED_H * 0.56)):
        for i in range(3):
            j = (i + 1) % 3
            ang_i = 2.0 * math.pi * i / 3.0
            ang_j = 2.0 * math.pi * j / 3.0
            pa = _leg_endpoint(ang_i, z_brace)
            pb = _leg_endpoint(ang_j, z_brace)
            brace_origin, brace_length = _tube_origin_and_length(pa, pb)
            pedestal.visual(
                mesh_from_geometry(
                    CylinderGeometry(BRACE_R, brace_length, radial_segments=12),
                    f"brace_{level}_{i}",
                ),
                origin=brace_origin,
                material=LIGHT_GRAY,
                name=f"cross_brace_{level}_{i}",
            )

    # ------------------------------------------------------- azimuth turret
    turret = model.part("azimuth_turret")
    turret.visual(
        mesh_from_geometry(
            CylinderGeometry(TURRET_R, TURRET_H, radial_segments=48), "turret_drum"
        ),
        origin=Origin(xyz=(0.0, 0.0, TURRET_H / 2.0)),
        material=LIGHT_GRAY,
        name="turntable_drum",
    )
    # Two yoke cheeks carrying the elevation trunnion (at +/-Y).
    for s, sy in (("left", 1.0), ("right", -1.0)):
        turret.visual(
            Box((0.85, 0.24, YOKE_H + 0.30)),
            origin=Origin(
                xyz=(0.0, sy * 0.72, TURRET_H + (YOKE_H + 0.30) / 2.0 - 0.15)
            ),
            material=WHITE,
            name=f"yoke_cheek_{s}",
        )
        # Trunnion bearing boss on each cheek.
        turret.visual(
            mesh_from_geometry(
                CylinderGeometry(0.24, 0.14, radial_segments=32), f"bearing_{s}"
            ),
            origin=Origin(
                xyz=(0.0, sy * 0.86, TRUNNION_Z), rpy=(math.pi / 2.0, 0.0, 0.0)
            ),
            material=STEEL,
            name=f"trunnion_bearing_{s}",
        )
    # Elevation drive housing behind the yoke.
    turret.visual(
        Box((0.30, 0.55, 0.75)),
        origin=Origin(xyz=(-0.62, 0.0, TURRET_H + 0.30)),
        material=LIGHT_GRAY,
        name="elevation_drive_housing",
    )

    model.articulation(
        "pedestal_to_turret",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=turret,
        origin=Origin(xyz=(0.0, 0.0, HUB_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=800.0, velocity=0.3, lower=-math.pi, upper=math.pi
        ),
    )

    # -------------------------------------------------------- dish assembly
    dish = model.part("dish_assembly")

    # Parabolic reflector shell (revolved thin wall, concave side up at q=0).
    n = 22
    outer = [
        (DISH_R * i / (n - 1), VERTEX_Z + _parabola_z(DISH_R * i / (n - 1)))
        for i in range(n)
    ]
    inner = [(r, z + DISH_T) for (r, z) in outer]
    dish.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(outer, inner, segments=64),
            "reflector_shell",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=WHITE,
        name="parabolic_reflector",
    )
    # Rim stiffening ring.
    rim_z = VERTEX_Z + _parabola_z(DISH_R)
    dish.visual(
        mesh_from_geometry(
            LatheGeometry(
                [
                    (DISH_R - 0.10, rim_z - 0.02),
                    (DISH_R + 0.06, rim_z + 0.02),
                    (DISH_R + 0.06, rim_z + 0.10),
                    (DISH_R - 0.10, rim_z + 0.14),
                ],
                segments=64,
            ),
            "rim_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=WHITE,
        name="rim_stiffener_ring",
    )
    # Backing hub between the trunnion axis and the dish vertex.
    dish.visual(
        mesh_from_geometry(
            CylinderGeometry(0.62, VERTEX_Z + 0.25, radial_segments=40), "hub"
        ),
        origin=Origin(xyz=(0.0, 0.0, (VERTEX_Z + 0.25) / 2.0 - 0.12)),
        material=STEEL,
        name="backing_hub",
    )
    # Trunnion shaft through the hub into both yoke bearings.
    dish.visual(
        mesh_from_geometry(
            CylinderGeometry(0.13, 1.9, radial_segments=28), "trunnion_shaft"
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=STEEL,
        name="trunnion_shaft",
    )
    # Radial backing ribs hugging the convex rear face (chord from the hub
    # outward, held just below the back surface).
    rib_r1, rib_r2 = 0.5, 3.6
    rib_drop = 0.25
    rib_z1 = VERTEX_Z + _parabola_z(rib_r1) - rib_drop
    rib_z2 = VERTEX_Z + _parabola_z(rib_r2) - rib_drop
    rib_len = math.hypot(rib_r2 - rib_r1, rib_z2 - rib_z1)
    rib_pitch = -math.atan2(rib_z2 - rib_z1, rib_r2 - rib_r1)
    rib_r_mid = (rib_r1 + rib_r2) / 2.0
    rib_z_mid = (rib_z1 + rib_z2) / 2.0
    for i in range(8):
        ang = 2.0 * math.pi * i / 8.0
        dish.visual(
            Box((rib_len, 0.07, 0.10)),
            origin=Origin(
                xyz=(
                    rib_r_mid * math.cos(ang),
                    rib_r_mid * math.sin(ang),
                    rib_z_mid,
                ),
                rpy=(0.0, rib_pitch, ang),
            ),
            material=LIGHT_GRAY,
            name=f"backing_rib_{i}",
        )
    # Counterweight box hanging behind the hub (balances the reflector).
    dish.visual(
        Box((0.9, 1.1, 0.5)),
        origin=Origin(xyz=(-0.70, 0.0, -0.28), rpy=(0.0, -0.35, 0.0)),
        material=LIGHT_GRAY,
        name="counterweight_box",
    )

    # Quad feed-support struts converging on the apex hub.
    for i, ang_deg in enumerate((45.0, 135.0, 225.0, 315.0)):
        origin, length = _strut_origin(math.radians(ang_deg))
        dish.visual(
            mesh_from_geometry(
                CylinderGeometry(STRUT_R, length, radial_segments=16),
                f"feed_strut_{i}",
            ),
            origin=origin,
            material=WHITE,
            name=f"feed_strut_{i}",
        )
    # Apex hub + subreflector facing back down at the dish.
    dish.visual(
        mesh_from_geometry(CylinderGeometry(0.20, 0.34, radial_segments=24), "apex_hub"),
        origin=Origin(xyz=(0.0, 0.0, APEX_Z)),
        material=STEEL,
        name="apex_hub",
    )
    dish.visual(
        mesh_from_geometry(DomeGeometry(0.42, radial_segments=32), "subreflector"),
        origin=Origin(xyz=(0.0, 0.0, APEX_Z - 0.14), rpy=(math.pi, 0.0, 0.0)),
        material=WHITE,
        name="subreflector",
    )
    # Central feed horn rising from the vertex toward the subreflector.
    dish.visual(
        mesh_from_geometry(
            CylinderGeometry(HORN_R, HORN_LEN, radial_segments=24), "feed_horn_tube"
        ),
        origin=Origin(xyz=(0.0, 0.0, VERTEX_Z + HORN_LEN / 2.0)),
        material=LIGHT_GRAY,
        name="feed_horn_tube",
    )
    dish.visual(
        mesh_from_geometry(
            ConeGeometry(HORN_R + 0.10, 0.30, radial_segments=24), "feed_horn_mouth"
        ),
        origin=Origin(
            xyz=(0.0, 0.0, VERTEX_Z + HORN_LEN + 0.15), rpy=(math.pi, 0.0, 0.0)
        ),
        material=DARK,
        name="feed_horn_mouth",
    )

    model.articulation(
        "turret_to_dish",
        ArticulationType.REVOLUTE,
        parent=turret,
        child=dish,
        origin=Origin(xyz=(0.0, 0.0, TRUNNION_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1500.0, velocity=0.2, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal_building")
    turret = object_model.get_part("azimuth_turret")
    dish = object_model.get_part("dish_assembly")
    az = object_model.get_articulation("pedestal_to_turret")
    el = object_model.get_articulation("turret_to_dish")

    # --- tripod-specific structural assertions ---
    for i in range(3):
        ctx.check(
            f"tripod_leg_{i}_exists",
            pedestal.get_visual(f"leg_{i}") is not None,
            f"leg_{i} missing from pedestal_building",
        )
    ctx.check(
        "tripod_hub_exists",
        pedestal.get_visual("tripod_hub") is not None,
        "tripod_hub missing from pedestal_building",
    )
    # Tripod legs splay well beyond the hub diameter.
    ped_lo, ped_hi = ctx.part_world_aabb(pedestal)
    ped_span_x = ped_hi[0] - ped_lo[0]
    ctx.check(
        "tripod_legs_splayed",
        ped_span_x > 2.0 * HUB_R + 2.0,
        f"pedestal x-span {ped_span_x:.2f} should exceed {2.0 * HUB_R + 2.0:.2f}",
    )

    # Intentional embeddings.
    ctx.allow_overlap(
        pedestal,
        turret,
        elem_a="hub_bearing_plate",
        elem_b="turntable_drum",
        reason="hub bearing plate seats under the azimuth turntable drum",
    )
    ctx.allow_overlap(
        turret,
        dish,
        reason="elevation trunnion shaft is captured inside both yoke bearings",
    )

    # Turret is centered on and supported by the tripod hub.
    with ctx.pose({az: 0.0, el: 0.0}):
        ctx.expect_within(
            turret, pedestal, axes="xy", name="turret_centered_on_tripod_hub"
        )
        ctx.expect_overlap(turret, pedestal, axes="xy", min_overlap=1.0)

        # Zenith pose: dish AABB is laterally centered and the feed apex
        # (struts + subreflector) stands well above the trunnion axis.
        lo, hi = ctx.part_world_aabb(dish)
        cx = (lo[0] + hi[0]) / 2.0
        ctx.check(
            "dish_centered_at_zenith", abs(cx) < 0.35, f"dish aabb x-center {cx:.3f}"
        )
        trunnion_world_z = HUB_TOP_Z + TRUNNION_Z
        ctx.check(
            "feed_apex_above_dish",
            hi[2] > trunnion_world_z + FOCAL + 0.4,
            f"dish top {hi[2]:.2f} vs trunnion {trunnion_world_z:.2f}",
        )
        ctx.check(
            "aperture_span",
            (hi[0] - lo[0]) > 2.0 * DISH_R * 0.95,
            f"dish x-span {(hi[0] - lo[0]):.2f}",
        )
        zen_center_x = cx
        sub_lo, sub_hi = ctx.part_element_world_aabb(
            dish, elem=dish.get_visual("subreflector")
        )
        zen_sub_z = (sub_lo[2] + sub_hi[2]) / 2.0

    # Elevation: tilting toward the horizon swings the dish mass toward +X
    # and lowers the feed apex.
    with ctx.pose({az: 0.0, el: 1.1}):
        lo, hi = ctx.part_world_aabb(dish)
        tilt_center_x = (lo[0] + hi[0]) / 2.0
        ctx.check(
            "elevation_tilts_dish_forward",
            tilt_center_x > zen_center_x + 0.8,
            f"x-center moved {zen_center_x:.2f} -> {tilt_center_x:.2f}",
        )
        sub_lo, sub_hi = ctx.part_element_world_aabb(
            dish, elem=dish.get_visual("subreflector")
        )
        sub_cz = (sub_lo[2] + sub_hi[2]) / 2.0
        sub_cx = (sub_lo[0] + sub_hi[0]) / 2.0
        ctx.check(
            "elevation_swings_feed_apex",
            sub_cz < zen_sub_z - 1.0 and sub_cx > 1.5,
            f"subreflector center z {zen_sub_z:.2f} -> {sub_cz:.2f}, x {sub_cx:.2f}",
        )

    # Azimuth: with the dish tilted, slewing 180 deg mirrors it to -X.
    with ctx.pose({az: math.pi, el: 1.1}):
        lo, hi = ctx.part_world_aabb(dish)
        slew_center_x = (lo[0] + hi[0]) / 2.0
        ctx.check(
            "azimuth_slews_dish",
            slew_center_x < -0.8,
            f"x-center after 180deg slew {slew_center_x:.3f}",
        )

    # Dish never dips into the pedestal ground zone across the motion range.
    for q in (0.0, 0.7, 1.35):
        with ctx.pose({az: 0.0, el: q}):
            lo, hi = ctx.part_world_aabb(dish)
            ctx.check(
                f"dish_clears_ground_el_{q:.2f}",
                lo[2] > 0.5,
                f"dish min z {lo[2]:.2f} at el={q:.2f}",
            )

    return ctx.report()


object_model = build_object_model()
