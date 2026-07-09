"""Large satellite ground-station antenna dish.

Reference: white parabolic reflector on a conical pedestal building with
paneled cladding, louver vents and a blue access door. A quad-strut feed
support holds a subreflector over a central feed horn. The dish assembly
rotates in azimuth on top of the pedestal and tilts in elevation between
the yoke cheeks.

Frames:
- Pedestal root frame: z up, ground at z=0.
- Azimuth turret frame: at the pedestal top, spins about +Z.
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
# Pedestal building (truncated cone).
PED_BASE_R = 3.2
PED_TOP_R = 1.15
PED_H = 5.6

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
BLUE = Material(name="door_blue", color=(0.13, 0.22, 0.55, 1.0))


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

    # ------------------------------------------------------------- pedestal
    pedestal = model.part("pedestal_building")

    cone_profile = [
        (0.0, 0.0),
        (PED_BASE_R, 0.0),
        (PED_TOP_R, PED_H),
        (0.0, PED_H),
    ]
    pedestal.visual(
        mesh_from_geometry(LatheGeometry(cone_profile, segments=48), "pedestal_cone"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=WHITE,
        name="pedestal_cone_shell",
    )
    # Flat top platform the turret sits on.
    pedestal.visual(
        mesh_from_geometry(
            CylinderGeometry(PED_TOP_R + 0.08, 0.16, radial_segments=48),
            "pedestal_platform",
        ),
        origin=Origin(xyz=(0.0, 0.0, PED_H + 0.08)),
        material=LIGHT_GRAY,
        name="pedestal_top_platform",
    )
    # Blue access door, straddling the sloped wall at the base (+X side).
    door_r = PED_BASE_R - (PED_BASE_R - PED_TOP_R) * (1.1 / PED_H) * 0.5
    pedestal.visual(
        Box((0.28, 1.0, 2.1)),
        origin=Origin(xyz=(door_r - 0.05, 0.0, 1.05)),
        material=BLUE,
        name="entrance_door",
    )
    # Door surround / porch frame.
    pedestal.visual(
        Box((0.34, 1.3, 2.35)),
        origin=Origin(xyz=(door_r - 0.14, 0.0, 1.175)),
        material=LIGHT_GRAY,
        name="entrance_surround",
    )
    # Louver vent panels around the base (like the photo's slatted vents).
    for i in range(3):
        ang = math.radians(55.0 + 70.0 * i)
        vr = PED_BASE_R - (PED_BASE_R - PED_TOP_R) * (1.0 / PED_H) * 0.5
        pedestal.visual(
            Box((0.22, 0.9, 1.1)),
            origin=Origin(
                xyz=(vr * math.cos(ang) * 0.985, vr * math.sin(ang) * 0.985, 1.35),
                rpy=(0.0, 0.0, ang),
            ),
            material=DARK,
            name=f"louver_vent_{i}",
        )
    # Horizontal panel seam rings on the cladding.
    for i in range(3):
        z = PED_H * (0.28 + 0.24 * i)
        r_at = PED_BASE_R + (PED_TOP_R - PED_BASE_R) * (z / PED_H)
        pedestal.visual(
            mesh_from_geometry(
                CylinderGeometry(r_at + 0.015, 0.06, radial_segments=48),
                f"seam_ring_{i}",
            ),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=LIGHT_GRAY,
            name=f"panel_seam_ring_{i}",
        )
    # Side access ladder flush against the sloped wall (-Y side): rails + rungs.
    def _wall_r(z: float) -> float:
        return PED_BASE_R - (PED_BASE_R - PED_TOP_R) * (z / PED_H)

    lad_z0 = PED_H * 0.60
    wall_slope = (PED_BASE_R - PED_TOP_R) / PED_H
    tilt = -math.atan(wall_slope)  # rail leans in, following the cone wall
    for s, off in (("left", -0.22), ("right", 0.22)):
        pedestal.visual(
            Box((0.06, 0.08, PED_H * 0.62)),
            origin=Origin(xyz=(off, -(_wall_r(lad_z0)), lad_z0), rpy=(tilt, 0.0, 0.0)),
            material=STEEL,
            name=f"ladder_rail_{s}",
        )
    for i in range(7):
        t = i / 6.0
        z = PED_H * (0.34 + 0.52 * t)
        pedestal.visual(
            Box((0.50, 0.06, 0.05)),
            origin=Origin(xyz=(0.0, -(_wall_r(z)), z)),
            material=STEEL,
            name=f"ladder_rung_{i}",
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
        origin=Origin(xyz=(0.0, 0.0, PED_H + 0.16)),
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
    # ---- Open triangulated space-frame truss behind reflector membrane ----
    # Radial spar members, concentric ring hoops, and diagonal bracing.
    N_SPARS = 12
    RING_RADII = [1.5, 2.6, 3.7]
    SPAR_R_IN = 0.70       # spar inner radius (just outside hub)
    SPAR_R_OUT = 4.15      # spar outer radius (near rim stiffener)
    TRUSS_DROP = 0.16      # truss plane offset below reflector back surface
    SPAR_MEM_R = 0.042     # spar/ring tubular member radius
    DIAG_MEM_R = 0.032     # diagonal brace radius

    member_idx = 0

    def _truss_z(r: float) -> float:
        """Z of the truss node plane at radial distance r."""
        return VERTEX_Z + _parabola_z(r) - TRUSS_DROP

    def _truss_member(p1, p2, radius, name):
        """Emit a thin cylindrical truss member between two 3D nodes."""
        nonlocal member_idx
        dx, dy, dz = p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]
        length = math.sqrt(dx * dx + dy * dy + dz * dz)
        mid = ((p1[0] + p2[0]) / 2.0, (p1[1] + p2[1]) / 2.0, (p1[2] + p2[2]) / 2.0)
        pitch = math.atan2(math.hypot(dx, dy), dz)
        yaw = math.atan2(dy, dx)
        dish.visual(
            mesh_from_geometry(
                CylinderGeometry(radius, length, radial_segments=8), name
            ),
            origin=Origin(xyz=mid, rpy=(0.0, pitch, yaw)),
            material=STEEL,
            name=name,
        )

    # --- Radial spar members ---
    for i in range(N_SPARS):
        ang = 2.0 * math.pi * i / N_SPARS
        ca, sa = math.cos(ang), math.sin(ang)
        p_in = (SPAR_R_IN * ca, SPAR_R_IN * sa, _truss_z(SPAR_R_IN))
        p_out = (SPAR_R_OUT * ca, SPAR_R_OUT * sa, _truss_z(SPAR_R_OUT))
        _truss_member(p_in, p_out, SPAR_MEM_R, f"member_{member_idx}")
        member_idx += 1

    # --- Concentric ring hoops (chord segments between adjacent spars) ---
    for ring_r in RING_RADII:
        z_ring = _truss_z(ring_r)
        for i in range(N_SPARS):
            ang0 = 2.0 * math.pi * i / N_SPARS
            ang1 = 2.0 * math.pi * ((i + 1) % N_SPARS) / N_SPARS
            p0 = (ring_r * math.cos(ang0), ring_r * math.sin(ang0), z_ring)
            p1 = (ring_r * math.cos(ang1), ring_r * math.sin(ang1), z_ring)
            _truss_member(p0, p1, SPAR_MEM_R, f"member_{member_idx}")
            member_idx += 1

    # --- Diagonal bracing (triangulation between spar/ring node layers) ---
    # Zones: hub_edge -> ring_0 -> ring_1 -> ring_2 -> spar_tip
    zone_radii = [SPAR_R_IN] + list(RING_RADII) + [SPAR_R_OUT]
    for zone_idx in range(len(zone_radii) - 1):
        r_lo = zone_radii[zone_idx]
        r_hi = zone_radii[zone_idx + 1]
        for i in range(N_SPARS):
            ang_i = 2.0 * math.pi * i / N_SPARS
            ang_n = 2.0 * math.pi * ((i + 1) % N_SPARS) / N_SPARS
            # Alternate diagonal direction per cell for triangulation
            if (zone_idx + i) % 2 == 0:
                pa = (r_lo * math.cos(ang_i), r_lo * math.sin(ang_i), _truss_z(r_lo))
                pb = (r_hi * math.cos(ang_n), r_hi * math.sin(ang_n), _truss_z(r_hi))
            else:
                pa = (r_lo * math.cos(ang_n), r_lo * math.sin(ang_n), _truss_z(r_lo))
                pb = (r_hi * math.cos(ang_i), r_hi * math.sin(ang_i), _truss_z(r_hi))
            _truss_member(pa, pb, DIAG_MEM_R, f"member_{member_idx}")
            member_idx += 1
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

    # Intentional embeddings.
    ctx.allow_overlap(
        pedestal,
        turret,
        reason="azimuth turntable drum seats on the pedestal top platform",
    )
    ctx.allow_overlap(
        turret,
        dish,
        reason="elevation trunnion shaft is captured inside both yoke bearings",
    )

    # Turret is centered on and supported by the pedestal.
    with ctx.pose({az: 0.0, el: 0.0}):
        ctx.expect_within(
            turret, pedestal, axes="xy", name="turret_centered_on_pedestal"
        )
        ctx.expect_overlap(turret, pedestal, axes="xy", min_overlap=1.0)

        # Zenith pose: dish AABB is laterally centered and the feed apex
        # (struts + subreflector) stands well above the trunnion axis.
        lo, hi = ctx.part_world_aabb(dish)
        cx = (lo[0] + hi[0]) / 2.0
        ctx.check(
            "dish_centered_at_zenith", abs(cx) < 0.35, f"dish aabb x-center {cx:.3f}"
        )
        trunnion_world_z = PED_H + 0.16 + TRUNNION_Z
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

    # ---- Open space-frame truss verification ----
    # The dish_assembly backing structure is an open triangulated truss
    # (radial spars, ring hoops, diagonal braces) behind the reflector membrane,
    # not a closed shell with backing ribs.
    spar = dish.get_visual("member_0")
    ctx.check(
        "truss_radial_spar_exists",
        spar is not None,
        "first radial spar member (member_0) missing from dish_assembly truss",
    )
    if spar is not None:
        with ctx.pose({az: 0.0, el: 0.0}):
            spar_lo, spar_hi = ctx.part_element_world_aabb(dish, elem=spar)
            ref_lo, ref_hi = ctx.part_element_world_aabb(
                dish, elem=dish.get_visual("parabolic_reflector")
            )
            ctx.check(
                "truss_members_behind_reflector",
                spar_hi[2] < ref_hi[2] - 0.02,
                f"spar top {spar_hi[2]:.3f} must be below reflector top {ref_hi[2]:.3f}",
            )
    # Confirm multiple truss member types exist (spars + rings + diagonals)
    ring_seg = dish.get_visual("member_12")
    diag_seg = dish.get_visual("member_48")
    ctx.check(
        "truss_ring_hoop_exists",
        ring_seg is not None,
        "first ring hoop segment (member_12) missing from truss",
    )
    ctx.check(
        "truss_diagonal_brace_exists",
        diag_seg is not None,
        "first diagonal brace (member_48) missing from truss",
    )

    return ctx.report()


object_model = build_object_model()
