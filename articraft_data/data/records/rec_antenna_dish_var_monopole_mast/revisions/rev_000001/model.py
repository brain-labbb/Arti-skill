"""Large satellite ground-station antenna dish on a monopole mast mount.

Reference: white parabolic reflector on a slender vertical tubular steel
monopole mast standing on a welded square flat base plate footing. A
quad-strut feed support holds a subreflector over a central feed horn. The
dish assembly rotates in azimuth on a flange at the mast top and tilts in
elevation between the yoke cheeks.

Frames:
- Monopole mast root frame: z up, ground at z=0.
- Azimuth turret frame: at the mast-top flange, spins about +Z.
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
# Monopole mast support (replaces the conical pedestal building).
BASE_PLATE_SIZE = 2.4   # square steel base plate side length
BASE_PLATE_T = 0.10     # base plate thickness
MAST_R = 0.20           # mast tube outer radius (400 mm OD pipe)
MAST_H = 5.60           # mast free height above the base plate
FLANGE_R = 0.55         # top mounting flange radius
FLANGE_T = 0.06         # top mounting flange thickness
PED_H = MAST_H          # alias kept so the joint origin formula is unchanged
GUSSET_H = 0.55         # base gusset plate height
GUSSET_W = 0.38         # gusset radial extent from mast surface
GUSSET_T = 0.018        # gusset plate thickness
JOINT_Z = BASE_PLATE_T + MAST_H + FLANGE_T  # flange-top z = turret seat height

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ground_station_antenna_dish")

    # -------------------------------------------------------- monopole mast
    pedestal = model.part("monopole_mast")

    # Square flat base plate footing at ground level.
    pedestal.visual(
        Box((BASE_PLATE_SIZE, BASE_PLATE_SIZE, BASE_PLATE_T)),
        origin=Origin(xyz=(0.0, 0.0, BASE_PLATE_T / 2.0)),
        material=STEEL,
        name="base_plate",
    )

    # Slender vertical tubular steel mast.
    mast_cz = BASE_PLATE_T + MAST_H / 2.0
    pedestal.visual(
        mesh_from_geometry(
            CylinderGeometry(MAST_R, MAST_H, radial_segments=36), "mast_tube"
        ),
        origin=Origin(xyz=(0.0, 0.0, mast_cz)),
        material=LIGHT_GRAY,
        name="mast_tube",
    )

    # Top mounting flange where the azimuth turret seats.
    flange_cz = BASE_PLATE_T + MAST_H + FLANGE_T / 2.0
    pedestal.visual(
        mesh_from_geometry(
            CylinderGeometry(FLANGE_R, FLANGE_T, radial_segments=40), "top_flange"
        ),
        origin=Origin(xyz=(0.0, 0.0, flange_cz)),
        material=STEEL,
        name="top_flange",
    )

    # Four radial gusset plates welded between the mast and the base plate.
    gusset_cz = BASE_PLATE_T + GUSSET_H / 2.0
    gusset_r = MAST_R + GUSSET_W / 2.0
    for i in range(4):
        ang = math.radians(90.0 * i)
        pedestal.visual(
            Box((GUSSET_W, GUSSET_T, GUSSET_H)),
            origin=Origin(
                xyz=(gusset_r * math.cos(ang), gusset_r * math.sin(ang), gusset_cz),
                rpy=(0.0, 0.0, ang),
            ),
            material=STEEL,
            name=f"gusset_{i}",
        )

    # Cable conduit running up the mast exterior (feed/rotary cables).
    conduit_h = MAST_H * 0.85
    pedestal.visual(
        mesh_from_geometry(
            CylinderGeometry(0.04, conduit_h, radial_segments=12), "cable_conduit"
        ),
        origin=Origin(
            xyz=(MAST_R + 0.04, 0.0, BASE_PLATE_T + conduit_h / 2.0 + 0.10)
        ),
        material=DARK,
        name="cable_conduit",
    )

    # Anchor bolt heads at the four base plate corners.
    bolt_inset = BASE_PLATE_SIZE / 2.0 - 0.18
    for i, (sx, sy) in enumerate(
        ((1.0, 1.0), (-1.0, 1.0), (-1.0, -1.0), (1.0, -1.0))
    ):
        pedestal.visual(
            mesh_from_geometry(
                CylinderGeometry(0.04, 0.04, radial_segments=12), f"anchor_bolt_{i}"
            ),
            origin=Origin(
                xyz=(sx * bolt_inset, sy * bolt_inset, BASE_PLATE_T + 0.02)
            ),
            material=DARK,
            name=f"anchor_bolt_{i}",
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
        origin=Origin(xyz=(0.0, 0.0, JOINT_Z)),
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
    pedestal = object_model.get_part("monopole_mast")
    turret = object_model.get_part("azimuth_turret")
    dish = object_model.get_part("dish_assembly")
    az = object_model.get_articulation("pedestal_to_turret")
    el = object_model.get_articulation("turret_to_dish")

    # ---------- variant-specific: monopole mast structure ----------
    mast_vis = pedestal.get_visual("mast_tube")
    ctx.check(
        "monopole_mast_tube_exists",
        mast_vis is not None,
        "monopole mast should have a mast_tube visual (slender vertical tube)",
    )

    # The mast tube must be tall and slender — proves this is a monopole
    # tube mast, not the parent's truncated conical building.
    with ctx.pose({az: 0.0, el: 0.0}):
        mast_lo, mast_hi = ctx.part_element_world_aabb(pedestal, elem=mast_vis)
        mast_height = mast_hi[2] - mast_lo[2]
        mast_width = max(mast_hi[0] - mast_lo[0], mast_hi[1] - mast_lo[1])
        ctx.check(
            "mast_is_slender_vertical_tube",
            mast_height > 4.0 and mast_width < 0.60,
            f"mast height={mast_height:.2f} width={mast_width:.2f}",
        )

        # Base plate is a square footing wider than the mast.
        bp_lo, bp_hi = ctx.part_element_world_aabb(
            pedestal, elem=pedestal.get_visual("base_plate")
        )
        bp_sx = bp_hi[0] - bp_lo[0]
        bp_sy = bp_hi[1] - bp_lo[1]
        ctx.check(
            "square_base_plate_footprint",
            bp_sx > 1.8 and bp_sy > 1.8 and abs(bp_sx - bp_sy) < 0.05,
            f"base plate sx={bp_sx:.2f} sy={bp_sy:.2f}",
        )

    # Intentional embeddings.
    ctx.allow_overlap(
        pedestal,
        turret,
        elem_a="top_flange",
        elem_b="turntable_drum",
        reason="azimuth turntable drum seats on the mast top flange",
    )
    ctx.allow_overlap(
        turret,
        dish,
        reason="elevation trunnion shaft is captured inside both yoke bearings",
    )

    # Turret is supported by the mast flange from below.
    with ctx.pose({az: 0.0, el: 0.0}):
        ctx.expect_gap(
            turret,
            pedestal,
            axis="z",
            min_gap=-0.005,
            max_gap=0.02,
            positive_elem="turntable_drum",
            negative_elem="top_flange",
            name="turret_drum_seated_on_mast_flange",
        )
        ctx.expect_overlap(
            turret, pedestal, axes="xy", min_overlap=0.80,
            name="turret_overlaps_mast_flange_in_xy",
        )

        # Zenith pose: dish AABB is laterally centered and the feed apex
        # (struts + subreflector) stands well above the trunnion axis.
        lo, hi = ctx.part_world_aabb(dish)
        cx = (lo[0] + hi[0]) / 2.0
        ctx.check(
            "dish_centered_at_zenith", abs(cx) < 0.35, f"dish aabb x-center {cx:.3f}"
        )
        trunnion_world_z = JOINT_Z + TRUNNION_Z
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

    # Dish never dips into the ground zone across the motion range.
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
