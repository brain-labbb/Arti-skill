from __future__ import annotations

# Sci-fi satellite-dish comm unit.
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
#     port lights, and a short pedestal post on top that carries the mount.
#   - azimuth_yoke (REVOLUTE about +Z, SECONDARY azimuth): a stout rear pedestal
#     arm that rises off the azimuth bearing and leans forward into a pivot
#     knuckle carrying the dish; it swings the whole dish left/right.
#   - dish_assembly (REVOLUTE about -Y, PRIMARY elevation): the true concave
#     parabolic reflector (lathed shell) — kept smooth/clean to match the
#     reference photo — with a thin glowing rim outline, a back hub, and an
#     offset-fed configuration: a curved support arm sweeps from the lower rim
#     forward to a feed horn positioned off the central axis (asymmetric focus,
#     like a modern Ku-band dish), with loop-emitted mounting clamps along the
#     arm. It tilts the dish up and down to aim in elevation.

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

    # ---- pedestal post + azimuth bearing rising off the top deck ------------
    post_h = 0.10
    post_cz = bz + 0.018 + post_h / 2.0
    pedestal_base.visual(
        Box((0.13, 0.13, post_h)),
        origin=Origin(xyz=(0.07, 0.0, post_cz)),
        material=gun_metal,
        name="pedestal_post",
    )
    pedestal_base.visual(
        Cylinder(radius=0.062, length=0.030),
        origin=Origin(xyz=(0.07, 0.0, bz + 0.018 + post_h + 0.014)),
        material=dark_panel,
        name="azimuth_bearing",
    )
    # two curved cable conduits running up the back of the post (sci-fi detail)
    for s, ys in enumerate((-0.05, 0.05)):
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

    azimuth_pivot_z = bz + 0.018 + post_h + 0.029
    azimuth_pivot_x = 0.07

    # ============================================================= AZIMUTH YOKE
    # A single rear pedestal arm: a stout post rises off the azimuth bearing and
    # leans forward at the top into a round pivot knuckle that carries the dish
    # at its back center. The post stays BEHIND the dish so the dish can tilt up
    # without its rim hitting the mount. Frame origin = azimuth pivot.
    azimuth_yoke = model.part("azimuth_yoke")

    arm_h = 0.20
    knuckle_dx = 0.05
    elevation_pivot_z = arm_h + 0.055

    azimuth_yoke.visual(
        Cylinder(radius=0.052, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=gun_metal,
        name="yoke_collar",
    )
    post_arm = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.0, 0.0, 0.030),
                (0.004, 0.0, 0.085),
                (0.020, 0.0, 0.140),
                (0.038, 0.0, elevation_pivot_z - 0.022),
                (knuckle_dx, 0.0, elevation_pivot_z),
            ],
            radius=0.034, samples_per_segment=10, radial_segments=18, cap_ends=True,
        ),
        "yoke_post",
    )
    azimuth_yoke.visual(post_arm, material=gun_metal, name="yoke_post")
    azimuth_yoke.visual(
        Cylinder(radius=0.046, length=0.060),
        origin=Origin(xyz=(knuckle_dx, 0.0, elevation_pivot_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_panel,
        name="yoke_knuckle",
    )
    azimuth_yoke.inertial = Inertial.from_geometry(
        Box((0.18, 0.11, 0.32)),
        mass=2.4,
        origin=Origin(xyz=(0.04, 0.0, 0.16)),
    )

    # =============================================================== DISH ASSEMBLY
    # Local frame origin = the elevation pivot (on the trunnion axis, along +/-Y).
    # At rest (q=0) the dish is baked tilted UP by `rest_tilt` so it perches above
    # the rear post and points up-forward like the reference. Positive elevation
    # tilts further up; negative brings it toward the horizon.
    dish_assembly = model.part("dish_assembly")

    rest_tilt = -0.62   # baked up-forward tilt about the elevation (-Y) axis (~36 deg);
    #                     negative so the local +X bowl axis points up-and-forward
    #                     (the concave mouth faces the sky, feed at the prime focus).
    mount_lift = 0.34   # raise the dish above the pivot so its lower rim clears
    #                     the rear post; a neck drops back down to the knuckle.

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
    # rim outline; no concentric panel rings, no radial ribs (those read as a
    # floating orbital cage and do not exist in the reference).
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
    # back hub at the lifted dish vertex
    dish_assembly.visual(
        Box((0.085, 0.12, 0.12)),
        origin=_tilt((-0.048, 0.0, 0.0)),
        material=gun_metal,
        name="dish_hub",
    )
    # neck dropping from the lifted vertex down to the pivot knuckle (real support)
    vert = _tilt_pt((-0.048, 0.0, 0.0))
    neck = mesh_from_geometry(
        tube_from_spline_points(
            [(vert[0], 0.0, vert[2]),
             (vert[0] * 0.5, 0.0, vert[2] * 0.5 + 0.01),
             (0.0, 0.0, 0.014)],
            radius=0.032, samples_per_segment=10, radial_segments=16, cap_ends=True,
        ),
        "dish_neck",
    )
    dish_assembly.visual(neck, material=gun_metal, name="dish_neck")
    # trunnion pin along the (untilted) Y pivot axis, seating into the knuckle
    dish_assembly.visual(
        Cylinder(radius=0.018, length=0.17),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_panel,
        name="trunnion_shaft",
    )

    # ---- offset-fed support arm and feed horn (modern Ku-band style) ---------
    # A single curved arm sweeps from the lower rim of the dish forward and to
    # the +Y side, carrying a feed horn at an offset focus position so the feed
    # and arm sit clear of the beam path rather than dead-center.
    offset_y = 0.22  # lateral offset from dish axis
    feed_fwd = focal + 0.10  # feed sits further forward than prime focus

    # Arm spline in the untilted dish frame (opens toward +X), then transformed.
    arm_pts_untilted = [
        (rim_depth_full, 0.02, -dish_radius + 0.02),          # lower rim attachment
        (rim_depth_full + 0.05, 0.08, -dish_radius * 0.65),   # leaving the rim
        (focal * 0.75, 0.15, -dish_radius * 0.25),            # mid-sweep forward
        (focal * 0.95, offset_y * 0.85, -0.04),               # approaching feed
        (feed_fwd, offset_y, 0.03),                           # offset feed position
    ]
    arm_pts = [_tilt_pt(p) for p in arm_pts_untilted]

    offset_arm = mesh_from_geometry(
        tube_from_spline_points(
            arm_pts,
            radius=0.015, samples_per_segment=14, radial_segments=14, cap_ends=True,
        ),
        "offset_arm",
    )
    dish_assembly.visual(offset_arm, material=gun_metal, name="offset_arm")

    # rim bracket: a small plate where the arm attaches to the dish lower rim
    bracket_pt = arm_pts[0]
    dish_assembly.visual(
        Box((0.05, 0.06, 0.04)),
        origin=Origin(xyz=bracket_pt),
        material=dark_panel,
        name="arm_rim_bracket",
    )

    # mounting clamps/bolts loop-emitted along the offset arm
    n_clamps = 4
    for i in range(n_clamps):
        t = (i + 1) / (n_clamps + 1)
        # linear interpolation along the control-point polyline for clamp placement
        idx_f = t * (len(arm_pts) - 1)
        idx = min(int(idx_f), len(arm_pts) - 2)
        frac = idx_f - idx
        p0, p1 = arm_pts[idx], arm_pts[idx + 1]
        cx = p0[0] + frac * (p1[0] - p0[0])
        cy = p0[1] + frac * (p1[1] - p0[1])
        cz = p0[2] + frac * (p1[2] - p0[2])
        dish_assembly.visual(
            Cylinder(radius=0.022, length=0.018),
            origin=Origin(xyz=(cx, cy, cz)),
            material=dark_panel,
            name=f"arm_clamp_{i}",
        )

    # offset feed horn at the arm tip, pointing back toward the dish reflector
    offset_feed_pos = arm_pts[-1]
    offset_feed = mesh_from_geometry(
        ConeGeometry(radius=0.026, height=0.11, radial_segments=22),
        "offset_feed_horn",
    )
    # cone axis is local +Z; rotate so apex points forward (+X) and mouth faces dish
    dish_assembly.visual(
        offset_feed,
        origin=Origin(xyz=offset_feed_pos, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gun_metal,
        name="offset_feed_horn",
    )
    # glowing tip sphere at the horn apex (overlaps cone tip for connectivity)
    # ConeGeometry is centered at origin: apex at +height/2 along the axis.
    # After (0, pi/2, 0) rotation, apex is at offset_feed_pos + (height/2, 0, 0).
    tip_pos = (offset_feed_pos[0] + 0.045, offset_feed_pos[1], offset_feed_pos[2])
    dish_assembly.visual(
        mesh_from_geometry(SphereGeometry(0.018), "offset_feed_tip"),
        origin=Origin(xyz=tip_pos),
        material=accent_red,
        name="offset_feed_tip",
    )

    dish_assembly.inertial = Inertial.from_geometry(
        Box((0.65, 0.65, 0.65)),
        mass=3.4,
        origin=Origin(xyz=(0.06, 0.0, 0.12)),
    )

    # ============================================================ ARTICULATIONS
    # SECONDARY: azimuth rotation of the whole yoke about the vertical axis.
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
    # rear knuckle. Dish opens up-forward (+X); -Y axis raises the mouth further.
    model.articulation(
        "elevation_tilt",
        ArticulationType.REVOLUTE,
        parent=azimuth_yoke,
        child=dish_assembly,
        origin=Origin(xyz=(knuckle_dx, 0.0, elevation_pivot_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.7, lower=-0.9, upper=0.5),
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

    # intentional joint-fit overlaps at the elevation bearing
    for ea, eb, reason in [
        ("trunnion_shaft", "yoke_knuckle", "dish trunnion pin captured inside the rear pivot knuckle"),
        ("dish_hub", "yoke_knuckle", "dish back hub seats against the pivot knuckle"),
        ("trunnion_shaft", "yoke_post", "trunnion pin passes through the post top into the knuckle"),
        ("dish_neck", "yoke_knuckle", "dish neck drops into and is captured by the pivot knuckle"),
        ("dish_neck", "yoke_post", "dish neck meets the post top at the bearing"),
    ]:
        ctx.allow_overlap(dish, yoke, elem_a=ea, elem_b=eb, reason=reason)

    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "base box rests on the ground at z~0",
        base_aabb is not None and abs(base_aabb[0][2]) < 0.01,
        details=f"base min z = {None if base_aabb is None else base_aabb[0][2]}",
    )

    ctx.check(
        "elevation joint is revolute about the horizontal Y axis",
        elevation.articulation_type == ArticulationType.REVOLUTE
        and tuple(elevation.axis) in ((0.0, 1.0, 0.0), (0.0, -1.0, 0.0)),
        details=f"type={elevation.articulation_type}, axis={elevation.axis}",
    )
    tip_rest = _aabb_center(ctx.part_element_world_aabb(dish, elem="offset_feed_tip"))
    with ctx.pose({elevation: 0.5}):
        tip_up = _aabb_center(ctx.part_element_world_aabb(dish, elem="offset_feed_tip"))
    with ctx.pose({elevation: -0.9}):
        tip_down = _aabb_center(ctx.part_element_world_aabb(dish, elem="offset_feed_tip"))
    ctx.check(
        "elevation tilt raises the dish/feed upward",
        tip_up[2] > tip_rest[2] + 0.02 and tip_down[2] < tip_rest[2] - 0.02,
        details=f"down z={tip_down[2]}, rest z={tip_rest[2]}, up z={tip_up[2]}",
    )

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

    rim_aabb = ctx.part_element_world_aabb(dish, elem="dish_rim")
    hub_aabb = ctx.part_element_world_aabb(dish, elem="dish_hub")
    ctx.check(
        "parabolic dish opens forward (rim ahead of the back hub)",
        _aabb_center(rim_aabb)[0] > _aabb_center(hub_aabb)[0] + 0.05,
        details=f"rim x={_aabb_center(rim_aabb)[0]}, hub x={_aabb_center(hub_aabb)[0]}",
    )

    feed_aabb = ctx.part_element_world_aabb(dish, elem="offset_feed_horn")
    feed_center = _aabb_center(feed_aabb)
    rim_center = _aabb_center(rim_aabb)
    ctx.check(
        "offset feed horn is mounted ahead of the reflector",
        feed_center[0] > rim_center[0] - 0.12,
        details=f"feed x={feed_center[0]}, rim x={rim_center[0]}",
    )
    ctx.check(
        "feed horn is offset from the dish central axis (asymmetric focus)",
        abs(feed_center[1] - rim_center[1]) > 0.04,
        details=f"feed y={feed_center[1]}, rim center y={rim_center[1]}",
    )

    # offset arm exists and connects rim to feed
    arm_aabb = ctx.part_element_world_aabb(dish, elem="offset_arm")
    ctx.check(
        "curved offset arm spans from rim to feed horn",
        arm_aabb is not None
        and (arm_aabb[1][1] - arm_aabb[0][1]) > 0.08  # Y span shows offset sweep
        and (arm_aabb[1][0] - arm_aabb[0][0]) > 0.05,  # X span shows forward reach
        details=f"arm aabb={arm_aabb}",
    )

    # mounting clamps present along the arm (loop-emitted)
    clamp_count = sum(
        1 for v in object_model.get_part("dish_assembly").visuals
        if v.name.startswith("arm_clamp_")
    )
    ctx.check(
        "mounting clamps are loop-emitted along the offset arm",
        clamp_count >= 3,
        details=f"found {clamp_count} arm clamps",
    )

    dish_w = rim_aabb[1][1] - rim_aabb[0][1]
    ctx.check(
        "reflector dish is wide (> 0.5 m across)",
        dish_w > 0.50,
        details=f"dish width = {dish_w}",
    )

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
