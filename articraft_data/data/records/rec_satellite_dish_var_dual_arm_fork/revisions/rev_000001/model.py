from __future__ import annotations

# Sci-fi satellite-dish comm unit — dual-arm trunnion fork variant.
#
# Coordinate convention:
#   - up is +Z; the equipment base box rests on the ground at z = 0.
#   - the box "front" (DATA LINK PANEL label + glowing slat-grille vent) faces -Y.
#   - the parabolic dish opens up-and-forward toward +X (aimed at the sky).
#
# Structure / articulation:
#   - pedestal_base (root, static): a dark matte rectangular equipment enclosure
#     with a glowing teal slat-grille vent, illuminated teal edge accents, etched
#     warning-triangle greebles, a DATA LINK PANEL label plate, a rack of amber
#     port lights, and a wider pedestal plinth on top that carries the mount.
#   - azimuth_yoke (REVOLUTE about +Z, SECONDARY azimuth): a symmetric dual-arm
#     trunnion fork — two parallel upright arms rise from the azimuth bearing on
#     either side of the dish and capture the reflector between them on a
#     horizontal trunnion shaft, like a heavy observatory mount.  The whole fork
#     slews left/right in azimuth.
#   - dish_assembly (REVOLUTE about -Y, PRIMARY elevation): the true concave
#     parabolic reflector (lathed shell) — kept smooth/clean to match the
#     reference photo — with a thin glowing rim outline, a back hub, and a
#     center-fed feed horn on an axial boom near the focus.  It tilts the dish
#     up and down to aim in elevation on the shared cross-shaft between the two
#     fork arms.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    TorusGeometry,
    VentGrilleGeometry,
    VentGrilleSlats,
    VentGrilleSleeve,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="scifi_satellite_dish")

    matte_black = model.material("matte_black", rgba=(0.09, 0.10, 0.11, 1.0))
    dark_panel = model.material("dark_panel", rgba=(0.14, 0.15, 0.17, 1.0))
    gun_metal = model.material("gun_metal", rgba=(0.22, 0.24, 0.27, 1.0))
    dish_face = model.material("dish_face", rgba=(0.17, 0.19, 0.22, 1.0))
    glow_teal = model.material("glow_teal", rgba=(0.10, 0.95, 0.80, 1.0))
    glow_lime = model.material("glow_lime", rgba=(0.62, 0.92, 0.20, 1.0))
    glow_amber = model.material("glow_amber", rgba=(0.98, 0.62, 0.10, 1.0))
    accent_red = model.material("accent_red", rgba=(0.85, 0.12, 0.10, 1.0))

    # --------------------------------------------------------------- box scale
    bx, by, bz = 0.62, 0.48, 0.34   # equipment enclosure (W x D x H), tabletop scale
    box_cz = bz / 2.0

    # ===================================================================== BASE
    pedestal_base = model.part("pedestal_base")

    pedestal_base.visual(
        Box((bx, by, bz)),
        origin=Origin(xyz=(0.0, 0.0, box_cz)),
        material=matte_black,
        name="enclosure_body",
    )
    pedestal_base.visual(
        Box((bx * 0.98, by * 0.98, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, bz + 0.001)),
        material=dark_panel,
        name="top_deck_plate",
    )

    # ---- glowing teal slat-grille vent on the -Y front face ------------------
    grille = mesh_from_geometry(
        VentGrilleGeometry(
            (0.20, 0.16),
            frame=0.012,
            face_thickness=0.004,
            duct_depth=0.022,
            slat_pitch=0.018,
            slat_width=0.010,
            slat_angle_deg=0.0,
            corner_radius=0.006,
            slats=VentGrilleSlats(profile="flat", divider_count=2, divider_width=0.004),
            sleeve=VentGrilleSleeve(style="short"),
        ),
        "side_grille",
    )
    pedestal_base.visual(
        grille,
        origin=Origin(xyz=(-0.13, -by / 2.0 + 0.002, box_cz), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=glow_teal,
        name="side_grille",
    )
    pedestal_base.visual(
        Box((0.228, 0.024, 0.188)),
        origin=Origin(xyz=(-0.13, -by / 2.0 - 0.006, box_cz)),
        material=gun_metal,
        name="grille_bezel",
    )

    # ---- glowing teal edge accents along the bottom front/side ---------------
    pedestal_base.visual(
        Box((bx * 0.94, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -by / 2.0 - 0.003, 0.020)),
        material=glow_teal,
        name="edge_accent_front",
    )
    pedestal_base.visual(
        Box((0.010, by * 0.94, 0.010)),
        origin=Origin(xyz=(-bx / 2.0 - 0.003, 0.0, 0.020)),
        material=glow_teal,
        name="edge_accent_side",
    )
    pedestal_base.visual(
        Box((bx * 0.92, 0.008, 0.006)),
        origin=Origin(xyz=(0.0, -by / 2.0 - 0.002, bz - 0.020)),
        material=glow_teal,
        name="edge_accent_top",
    )

    # ---- DATA LINK PANEL label plate on the -Y face (three lime label bars) --
    for i in range(3):
        pedestal_base.visual(
            Box((0.110, 0.010, 0.013)),
            origin=Origin(xyz=(0.165, -by / 2.0 + 0.002, box_cz + 0.045 - i * 0.034)),
            material=glow_lime,
            name=f"label_bar_{i}",
        )

    # ---- etched warning-triangle greebles (top + front) ---------------------
    def _triangle(name, edge):
        h = edge * math.sqrt(3.0) / 2.0
        a = (-edge / 2.0, -h / 3.0)
        b = (edge / 2.0, -h / 3.0)
        c = (0.0, 2.0 * h / 3.0)
        pts = [a, b, c, a]
        path = []
        for (x0, y0), (x1, y1) in zip(pts[:-1], pts[1:]):
            path += [(x0, y0, 0.0), ((x0 + x1) / 2.0, (y0 + y1) / 2.0, 0.0), (x1, y1, 0.0)]
        return mesh_from_geometry(
            tube_from_spline_points(path, radius=0.0022, samples_per_segment=2,
                                    radial_segments=6, cap_ends=True),
            name,
        )

    pedestal_base.visual(
        _triangle("warn_triangle_top", 0.060),
        origin=Origin(xyz=(-0.21, 0.11, bz + 0.006)),
        material=glow_lime,
        name="warn_triangle_top",
    )
    pedestal_base.visual(
        _triangle("warn_triangle_front", 0.055),
        origin=Origin(xyz=(0.10, -by / 2.0 + 0.001, box_cz + 0.10), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=glow_lime,
        name="warn_triangle_front",
    )

    # ---- amber port rack on the +Y back face --------------------------------
    for i in range(5):
        pedestal_base.visual(
            Box((0.014, 0.006, 0.014)),
            origin=Origin(xyz=(-0.13 + i * 0.055, by / 2.0 + 0.003, box_cz - 0.02)),
            material=glow_amber,
            name=f"port_light_{i}",
        )
    for i in range(3):
        pedestal_base.visual(
            Box((0.34, 0.008, 0.006)),
            origin=Origin(xyz=(0.06, by / 2.0 + 0.002, box_cz + 0.05 - i * 0.04)),
            material=gun_metal,
            name=f"rack_slat_{i}",
        )

    # ---- pedestal plinth + azimuth bearing rising off the top deck ------------
    # Wider plinth supports the dual-arm fork base
    post_h = 0.10
    post_cz = bz + 0.018 + post_h / 2.0
    pedestal_base.visual(
        Box((0.14, 0.50, post_h)),
        origin=Origin(xyz=(0.07, 0.0, post_cz)),
        material=gun_metal,
        name="pedestal_post",
    )
    pedestal_base.visual(
        Cylinder(radius=0.18, length=0.030),
        origin=Origin(xyz=(0.07, 0.0, bz + 0.018 + post_h + 0.015)),
        material=dark_panel,
        name="azimuth_bearing",
    )
    # two curved cable conduits running up the back of the post (sci-fi detail)
    for s, ys in enumerate((-0.08, 0.08)):
        conduit = mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.115, ys, bz - 0.04),
                    (0.10, ys, bz + 0.02),
                    (0.085, ys, bz + 0.06),
                    (0.07, ys, bz + 0.10),
                ],
                radius=0.010, samples_per_segment=8, radial_segments=12, cap_ends=True,
            ),
            f"cable_conduit_{s}",
        )
        pedestal_base.visual(conduit, material=matte_black, name=f"cable_conduit_{s}")

    pedestal_base.inertial = Inertial.from_geometry(
        Box((bx, by, bz + post_h)),
        mass=15.0,
        origin=Origin(xyz=(0.0, 0.0, box_cz)),
    )

    azimuth_pivot_z = bz + 0.018 + post_h + 0.030  # top of azimuth bearing
    azimuth_pivot_x = 0.07

    # ============================================================= AZIMUTH YOKE
    # Symmetric dual-arm trunnion fork: two parallel upright arms rise from the
    # azimuth bearing on either side of the dish and capture the reflector
    # between them on a horizontal trunnion shaft that spans both arm tops.
    # Frame origin = azimuth pivot (top of bearing, center of fork base).
    azimuth_yoke = model.part("azimuth_yoke")

    arm_y = 0.36          # arm center Y distance from fork centerline
    arm_wx = 0.06         # arm width (X)
    arm_wy = 0.05         # arm depth (Y)
    bearing_r = 0.040     # bearing housing radius
    arm_bottom = 0.045    # arm bottom Z (slight overlap into base plate)
    elevation_pivot_z = 0.345  # elevation pivot at bearing centers
    arm_top = elevation_pivot_z - bearing_r + 0.008  # arms seat into bearing bottom
    arm_h = arm_top - arm_bottom  # arm height (stops below trunnion centerline)

    # yoke collar sits on the azimuth bearing
    azimuth_yoke.visual(
        Cylinder(radius=0.16, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=gun_metal,
        name="yoke_collar",
    )
    # base plate spans both arm bases
    azimuth_yoke.visual(
        Box((0.12, 2.0 * arm_y + arm_wy + 0.02, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=dark_panel,
        name="yoke_base_plate",
    )

    # loop-emit: two symmetric upright arms, bearing housings, and cap bolts
    for i in range(2):
        y_sign = -1.0 if i == 0 else 1.0
        cy = y_sign * arm_y

        # upright arm
        azimuth_yoke.visual(
            Box((arm_wx, arm_wy, arm_h)),
            origin=Origin(xyz=(0.0, cy, arm_bottom + arm_h / 2.0)),
            material=gun_metal,
            name=f"fork_arm_{i}",
        )
        # bearing housing at arm top (cylinder along Y)
        azimuth_yoke.visual(
            Cylinder(radius=0.040, length=0.065),
            origin=Origin(xyz=(0.0, cy, elevation_pivot_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_panel,
            name=f"bearing_housing_{i}",
        )
        # cap bolt on the outside face of each bearing
        azimuth_yoke.visual(
            Cylinder(radius=0.024, length=0.018),
            origin=Origin(xyz=(0.0, cy + y_sign * 0.040, elevation_pivot_z),
                          rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=gun_metal,
            name=f"cap_bolt_{i}",
        )

    azimuth_yoke.inertial = Inertial.from_geometry(
        Box((0.14, 2.0 * arm_y + arm_wy, arm_h + 0.06)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, 0.06 + arm_h / 2.0)),
    )

    # =============================================================== DISH ASSEMBLY
    # Local frame origin = the elevation pivot (on the trunnion axis, along +/-Y).
    # At rest (q=0) the dish is baked tilted UP by `rest_tilt` so it perches above
    # the fork and points up-forward like the reference. Positive elevation
    # tilts further up; negative brings it toward the horizon.
    dish_assembly = model.part("dish_assembly")

    rest_tilt = -0.62   # baked up-forward tilt about the elevation (-Y) axis (~36 deg);
    #                     negative so the local +X bowl axis points up-and-forward
    #                     (the concave mouth faces the sky, feed at the prime focus).
    mount_lift = 0.0    # dish hub sits at the pivot (between fork arms, no lift needed)

    def _tilt(xyz, rpy=(0.0, 0.0, 0.0)):
        x, y, z = xyz
        c, s = math.cos(rest_tilt), math.sin(rest_tilt)
        xr = c * x + s * z
        zr = -s * x + c * z + mount_lift
        r, p, yaw = rpy
        return Origin(xyz=(xr, y, zr), rpy=(r, p + rest_tilt, yaw))

    def _tilt_pt(xyz):
        x, y, z = xyz
        c, s = math.cos(rest_tilt), math.sin(rest_tilt)
        return (c * x + s * z, y, -s * x + c * z + mount_lift)

    dish_radius = 0.30
    focal = 0.165
    rim_depth_full = (dish_radius * dish_radius) / (4.0 * focal)

    # central spar through the vertex region so shell, hub, and feed base share
    # connected geometry (no floating islands).
    dish_assembly.visual(
        Cylinder(radius=0.034, length=rim_depth_full + 0.12),
        origin=_tilt((rim_depth_full / 2.0 - 0.06, 0.0, 0.0), (0.0, math.pi / 2.0, 0.0)),
        material=gun_metal,
        name="dish_spar",
    )

    # parabolic shell profiles in a +Z-axial lathe frame: radius -> axial depth.
    n_prof = 14
    wall = 0.010
    outer = []
    inner = []
    for k in range(n_prof + 1):
        r = dish_radius * k / n_prof
        depth = (r * r) / (4.0 * focal)
        outer.append((r, depth))
        inner.append((max(r - 0.004, 0.0), depth + wall))
    dish_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer, inner, segments=72, start_cap="flat", end_cap="flat",
        ),
        "reflector_shell",
    )
    # bowl axis is local +Z of the lathe; rotate +90 about Y so the concave mouth
    # opens toward local +X (the same axis the feed/spar/hub/rim live on).
    dish_assembly.visual(
        dish_shell,
        origin=_tilt((0.0, 0.0, 0.0), (0.0, math.pi / 2.0, 0.0)),
        material=dish_face,
        name="reflector_shell",
    )
    # The inner bowl is kept smooth and clean to match the reference photo (a
    # plain dark reflector). The only dish-mouth accent is a single thin glowing
    # rim outline; no concentric panel rings, no radial ribs.
    #
    # glowing lime rim outline hugging the dish mouth (thin, sits on the shell
    # edge so it stays connected — reads as a lit rim line, not a fat ring).
    dish_assembly.visual(
        mesh_from_geometry(
            TorusGeometry(radius=dish_radius - 0.003, tube=0.006, radial_segments=12,
                          tubular_segments=96),
            "dish_rim",
        ),
        origin=_tilt((rim_depth_full, 0.0, 0.0), (0.0, math.pi / 2.0, 0.0)),
        material=glow_lime,
        name="dish_rim",
    )
    # back hub at the dish vertex — sits between the fork arms on the trunnion
    dish_assembly.visual(
        Box((0.085, 0.12, 0.12)),
        origin=_tilt((-0.048, 0.0, 0.0)),
        material=gun_metal,
        name="dish_hub",
    )
    # trunnion shaft along the (untilted) Y pivot axis, spanning between the two
    # fork arm bearings. Captured inside both bearing housings.
    trunnion_span = 2.0 * arm_y  # from -arm_y to +arm_y
    dish_assembly.visual(
        Cylinder(radius=0.018, length=trunnion_span),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_panel,
        name="trunnion_shaft",
    )

    # ---- feed horn on the dish axis near the focus, on a central boom -------
    feed_x = focal + 0.02
    feed_horn = mesh_from_geometry(
        ConeGeometry(radius=0.028, height=0.12, radial_segments=22),
        "feed_horn",
    )
    # cone axis is local +Z; aim apex/tip forward (+X), wide mouth toward the dish
    dish_assembly.visual(
        feed_horn,
        origin=_tilt((feed_x, 0.0, 0.0), (0.0, math.pi / 2.0, 0.0)),
        material=gun_metal,
        name="feed_horn",
    )
    dish_assembly.visual(
        mesh_from_geometry(SphereGeometry(0.020), "feed_tip"),
        origin=_tilt((feed_x + 0.075, 0.0, 0.0)),
        material=accent_red,
        name="feed_tip",
    )
    # central feed boom: a slender mast on the dish axis running from the vertex
    # out to the feed horn (center-fed, like the reference photo).
    feed_boom = mesh_from_geometry(
        tube_from_spline_points(
            [_tilt_pt((0.02, 0.0, 0.0)),
             _tilt_pt(((0.02 + feed_x) / 2.0, 0.0, 0.0)),
             _tilt_pt((feed_x, 0.0, 0.0))],
            radius=0.012, samples_per_segment=10, radial_segments=14, cap_ends=True,
        ),
        "feed_boom",
    )
    dish_assembly.visual(feed_boom, material=gun_metal, name="feed_boom")

    dish_assembly.inertial = Inertial.from_geometry(
        Box((0.65, 0.65, 0.40)),
        mass=3.4,
        origin=Origin(xyz=(0.06, 0.0, 0.05)),
    )

    # ============================================================ ARTICULATIONS
    # SECONDARY: azimuth rotation of the whole fork about the vertical axis.
    model.articulation(
        "azimuth_rotation",
        ArticulationType.REVOLUTE,
        parent=pedestal_base,
        child=azimuth_yoke,
        origin=Origin(xyz=(azimuth_pivot_x, 0.0, azimuth_pivot_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.8, lower=-math.pi, upper=math.pi),
    )
    # PRIMARY: elevation tilt of the dish about the horizontal -Y axis at the
    # shared cross-shaft between the two fork arms. Dish opens up-forward (+X);
    # -Y axis raises the mouth further.
    model.articulation(
        "elevation_tilt",
        ArticulationType.REVOLUTE,
        parent=azimuth_yoke,
        child=dish_assembly,
        origin=Origin(xyz=(0.0, 0.0, elevation_pivot_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.7, lower=-0.5, upper=0.5),
    )

    return model


def _aabb_center(aabb):
    lo, hi = aabb
    return tuple((lo[i] + hi[i]) / 2.0 for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("pedestal_base")
    yoke = object_model.get_part("azimuth_yoke")
    dish = object_model.get_part("dish_assembly")
    azimuth = object_model.get_articulation("azimuth_rotation")
    elevation = object_model.get_articulation("elevation_tilt")

    # intentional joint-fit overlaps at the trunnion bearings
    for i in range(2):
        ctx.allow_overlap(
            dish, yoke,
            elem_a="trunnion_shaft", elem_b=f"bearing_housing_{i}",
            reason=f"trunnion shaft captured inside fork arm {i} bearing housing",
        )

    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "base box rests on the ground at z~0",
        base_aabb is not None and abs(base_aabb[0][2]) < 0.01,
        details=f"base min z = {None if base_aabb is None else base_aabb[0][2]}",
    )

    # --- dual-arm fork structure ---
    arm_0_aabb = ctx.part_element_world_aabb(yoke, elem="fork_arm_0")
    arm_1_aabb = ctx.part_element_world_aabb(yoke, elem="fork_arm_1")
    ctx.check(
        "fork has two symmetric upright arms",
        arm_0_aabb is not None and arm_1_aabb is not None,
        details="missing fork arm visuals",
    )
    if arm_0_aabb and arm_1_aabb:
        arm_0_cy = (arm_0_aabb[0][1] + arm_0_aabb[1][1]) / 2.0
        arm_1_cy = (arm_1_aabb[0][1] + arm_1_aabb[1][1]) / 2.0
        ctx.check(
            "fork arms are symmetric about the Y=0 centerline",
            abs(arm_0_cy + arm_1_cy) < 0.02,
            details=f"arm_0 cy={arm_0_cy:.4f}, arm_1 cy={arm_1_cy:.4f}",
        )
        # arms are on opposite sides of center
        ctx.check(
            "fork arms straddle the dish (one on each side)",
            arm_0_cy < -0.10 and arm_1_cy > 0.10,
            details=f"arm_0 cy={arm_0_cy:.4f}, arm_1 cy={arm_1_cy:.4f}",
        )

    # cap bolts on both arms
    bolt_0_aabb = ctx.part_element_world_aabb(yoke, elem="cap_bolt_0")
    bolt_1_aabb = ctx.part_element_world_aabb(yoke, elem="cap_bolt_1")
    ctx.check(
        "cap bolts present on outside of both fork bearings",
        bolt_0_aabb is not None and bolt_1_aabb is not None,
        details="missing cap bolt visuals",
    )

    # --- elevation joint ---
    ctx.check(
        "elevation joint is revolute about the horizontal Y axis",
        elevation.articulation_type == ArticulationType.REVOLUTE
        and tuple(elevation.axis) in ((0.0, 1.0, 0.0), (0.0, -1.0, 0.0)),
        details=f"type={elevation.articulation_type}, axis={elevation.axis}",
    )
    tip_rest = _aabb_center(ctx.part_element_world_aabb(dish, elem="feed_tip"))
    with ctx.pose({elevation: 0.5}):
        tip_up = _aabb_center(ctx.part_element_world_aabb(dish, elem="feed_tip"))
    with ctx.pose({elevation: -0.5}):
        tip_down = _aabb_center(ctx.part_element_world_aabb(dish, elem="feed_tip"))
    ctx.check(
        "elevation tilt raises the dish/feed upward",
        tip_up[2] > tip_rest[2] + 0.02 and tip_down[2] < tip_rest[2] - 0.02,
        details=f"down z={tip_down[2]}, rest z={tip_rest[2]}, up z={tip_up[2]}",
    )

    # --- azimuth joint ---
    ctx.check(
        "azimuth joint is revolute about +Z",
        azimuth.articulation_type == ArticulationType.REVOLUTE
        and tuple(azimuth.axis) == (0.0, 0.0, 1.0),
        details=f"type={azimuth.articulation_type}, axis={azimuth.axis}",
    )
    rim_rest = _aabb_center(ctx.part_element_world_aabb(dish, elem="dish_rim"))
    with ctx.pose({azimuth: math.pi / 2.0}):
        rim_spun = _aabb_center(ctx.part_element_world_aabb(dish, elem="dish_rim"))
    ctx.check(
        "azimuth rotation swings the dish horizontally",
        abs(rim_spun[0] - rim_rest[0]) > 0.03 or abs(rim_spun[1] - rim_rest[1]) > 0.03,
        details=f"rest={rim_rest}, spun={rim_spun}",
    )

    # --- dish geometry ---
    rim_aabb = ctx.part_element_world_aabb(dish, elem="dish_rim")
    hub_aabb = ctx.part_element_world_aabb(dish, elem="dish_hub")
    ctx.check(
        "parabolic dish opens forward (rim ahead of the back hub)",
        _aabb_center(rim_aabb)[0] > _aabb_center(hub_aabb)[0] + 0.05,
        details=f"rim x={_aabb_center(rim_aabb)[0]}, hub x={_aabb_center(hub_aabb)[0]}",
    )

    feed_aabb = ctx.part_element_world_aabb(dish, elem="feed_horn")
    ctx.check(
        "feed horn is mounted ahead of the reflector near the focus",
        _aabb_center(feed_aabb)[0] > _aabb_center(rim_aabb)[0] - 0.12,
        details=f"feed x={_aabb_center(feed_aabb)[0]}, rim x={_aabb_center(rim_aabb)[0]}",
    )

    dish_w = rim_aabb[1][1] - rim_aabb[0][1]
    ctx.check(
        "reflector dish is wide (> 0.5 m across)",
        dish_w > 0.50,
        details=f"dish width = {dish_w}",
    )

    # --- fork captures dish between the arms ---
    trunnion_aabb = ctx.part_element_world_aabb(dish, elem="trunnion_shaft")
    if arm_0_aabb and arm_1_aabb and trunnion_aabb:
        ctx.check(
            "trunnion shaft spans between both fork arms",
            trunnion_aabb[0][1] < arm_0_aabb[1][1] and trunnion_aabb[1][1] > arm_1_aabb[0][1],
            details=(
                f"trunnion y=[{trunnion_aabb[0][1]:.3f}, {trunnion_aabb[1][1]:.3f}], "
                f"arm_0 y_max={arm_0_aabb[1][1]:.3f}, arm_1 y_min={arm_1_aabb[0][1]:.3f}"
            ),
        )

    if arm_0_aabb and arm_1_aabb and rim_aabb:
        ctx.check(
            "fork arms are outside the dish rim (dish captured between arms)",
            arm_0_aabb[0][1] < rim_aabb[0][1] and arm_1_aabb[1][1] > rim_aabb[1][1],
            details=(
                f"rim y=[{rim_aabb[0][1]:.3f}, {rim_aabb[1][1]:.3f}], "
                f"arm_0 y=[{arm_0_aabb[0][1]:.3f}, {arm_0_aabb[1][1]:.3f}], "
                f"arm_1 y=[{arm_1_aabb[0][1]:.3f}, {arm_1_aabb[1][1]:.3f}]"
            ),
        )

    # --- base details ---
    grille_aabb = ctx.part_element_world_aabb(base, elem="side_grille")
    ctx.check(
        "glowing slat-grille is on the box front face",
        grille_aabb is not None and _aabb_center(grille_aabb)[1] < -0.10,
        details=f"grille y={None if grille_aabb is None else _aabb_center(grille_aabb)[1]}",
    )

    yoke_pos = ctx.part_world_position(yoke)
    ctx.check(
        "yoke mounted above the base box",
        yoke_pos is not None and yoke_pos[2] > 0.30,
        details=f"yoke z={None if yoke_pos is None else yoke_pos[2]}",
    )

    return ctx.report()


object_model = build_object_model()
