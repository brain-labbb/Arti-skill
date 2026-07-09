from __future__ import annotations

# Sci-fi phased-array antenna comm unit (fork variant of satellite dish).
#
# Coordinate convention:
#   - up is +Z; the equipment base box rests on the ground at z = 0.
#   - the box "front" (DATA LINK PANEL label + glowing slat-grille vent) faces -Y.
#   - the flat phased-array panel faces up-and-forward toward +X (aimed at sky).
#
# Structure / articulation:
#   - pedestal_base (root, static): dark matte rectangular equipment enclosure
#     with glowing teal slat-grille vent, illuminated teal edge accents, etched
#     warning-triangle greebles, DATA LINK PANEL label plate, amber port rack,
#     and a short pedestal post on top carrying the mount.
#   - azimuth_yoke (REVOLUTE about +Z, SECONDARY azimuth): stout rear pedestal
#     arm rising off the azimuth bearing, leaning forward into a pivot knuckle
#     that carries the panel; swings the whole array left/right.
#   - array_panel (REVOLUTE about -Y, PRIMARY elevation): flat thick rectangular
#     phased-array plate tiled with a grid of small square radiator elements
#     (panel_0..panel_N), glowing edge trim, and a shallow back housing. No
#     concave reflector, no feed horn. Tilts up/down to aim in elevation.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    VentGrilleGeometry,
    VentGrilleSlats,
    VentGrilleSleeve,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="scifi_phased_array_comm")

    matte_black = model.material("matte_black", rgba=(0.09, 0.10, 0.11, 1.0))
    dark_panel = model.material("dark_panel", rgba=(0.14, 0.15, 0.17, 1.0))
    gun_metal = model.material("gun_metal", rgba=(0.22, 0.24, 0.27, 1.0))
    array_face = model.material("array_face", rgba=(0.13, 0.15, 0.18, 1.0))
    radiator_mat = model.material("radiator", rgba=(0.18, 0.22, 0.26, 1.0))
    glow_teal = model.material("glow_teal", rgba=(0.10, 0.95, 0.80, 1.0))
    glow_lime = model.material("glow_lime", rgba=(0.62, 0.92, 0.20, 1.0))
    glow_amber = model.material("glow_amber", rgba=(0.98, 0.62, 0.10, 1.0))

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
    # leans forward at the top into a round pivot knuckle that carries the panel
    # at its back center. The post stays BEHIND the panel so the panel can tilt
    # up without its edge hitting the mount. Frame origin = azimuth pivot.
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

    # ========================================================== ARRAY PANEL
    # Local frame origin = the elevation pivot (on the trunnion axis, +/-Y).
    # At rest (q=0) the panel is baked tilted UP by `rest_tilt` so it perches
    # above the rear post and aims up-forward. Positive elevation tilts further
    # up; negative brings it toward the horizon.
    array_panel = model.part("array_panel")

    rest_tilt = -0.62   # baked up-forward tilt about the elevation (-Y) axis (~36 deg)
    mount_lift = 0.34   # raise the panel above the pivot so lower edge clears post

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

    # ---- panel dimensions ----
    panel_thick = 0.028   # plate thickness (X in local untilted frame)
    panel_w = 0.52        # panel width (Y)
    panel_h = 0.42        # panel height (Z)

    # ---- main flat array plate ----
    array_panel.visual(
        Box((panel_thick, panel_w, panel_h)),
        origin=_tilt((0.0, 0.0, 0.0)),
        material=array_face,
        name="array_plate",
    )

    # ---- shallow back housing on the -X rear of the plate ----
    housing_thick = 0.022
    housing_w = 0.44
    housing_h = 0.36
    array_panel.visual(
        Box((housing_thick, housing_w, housing_h)),
        origin=_tilt((-panel_thick / 2.0 - housing_thick / 2.0, 0.0, 0.0)),
        material=gun_metal,
        name="back_housing",
    )

    # ---- glowing teal edge trim around the panel perimeter (front face) ----
    trim_thick = 0.006
    trim_w = 0.010
    front_x = panel_thick / 2.0 + trim_thick / 2.0

    # top trim bar
    array_panel.visual(
        Box((trim_thick, panel_w - 0.02, trim_w)),
        origin=_tilt((front_x, 0.0, panel_h / 2.0 - trim_w / 2.0)),
        material=glow_teal,
        name="trim_top",
    )
    # bottom trim bar
    array_panel.visual(
        Box((trim_thick, panel_w - 0.02, trim_w)),
        origin=_tilt((front_x, 0.0, -panel_h / 2.0 + trim_w / 2.0)),
        material=glow_teal,
        name="trim_bottom",
    )
    # left trim bar
    array_panel.visual(
        Box((trim_thick, trim_w, panel_h - 0.02)),
        origin=_tilt((front_x, -panel_w / 2.0 + trim_w / 2.0, 0.0)),
        material=glow_teal,
        name="trim_left",
    )
    # right trim bar
    array_panel.visual(
        Box((trim_thick, trim_w, panel_h - 0.02)),
        origin=_tilt((front_x, panel_w / 2.0 - trim_w / 2.0, 0.0)),
        material=glow_teal,
        name="trim_right",
    )

    # ---- phased-array radiator element grid on the front face ----
    # Shared geometry helper: one small square radiator pad
    elem_size = 0.042
    elem_thick = 0.004

    def _radiator_element():
        """Return a mesh for one square radiator element (thin raised pad)."""
        return Box((elem_thick, elem_size, elem_size))

    n_cols = 8   # elements along Y (panel width)
    n_rows = 6   # elements along Z (panel height)
    pitch_y = 0.056
    pitch_z = 0.058
    elem_front_x = panel_thick / 2.0 + elem_thick / 2.0  # protrude from front face

    idx = 0
    for row in range(n_rows):
        for col in range(n_cols):
            y = -(n_cols - 1) * pitch_y / 2.0 + col * pitch_y
            z = -(n_rows - 1) * pitch_z / 2.0 + row * pitch_z
            array_panel.visual(
                _radiator_element(),
                origin=_tilt((elem_front_x, y, z)),
                material=radiator_mat,
                name=f"panel_{idx}",
            )
            idx += 1

    # ---- support neck from back housing down to the pivot knuckle ----
    back_center = _tilt_pt((-panel_thick / 2.0 - housing_thick / 2.0, 0.0, 0.0))
    neck = mesh_from_geometry(
        tube_from_spline_points(
            [(back_center[0], 0.0, back_center[2]),
             (back_center[0] * 0.5, 0.0, back_center[2] * 0.5 + 0.01),
             (0.0, 0.0, 0.014)],
            radius=0.032, samples_per_segment=10, radial_segments=16, cap_ends=True,
        ),
        "panel_neck",
    )
    array_panel.visual(neck, material=gun_metal, name="panel_neck")

    # ---- trunnion pin along the (untilted) Y pivot axis, seating into knuckle --
    array_panel.visual(
        Cylinder(radius=0.018, length=0.17),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_panel,
        name="trunnion_shaft",
    )

    array_panel.inertial = Inertial.from_geometry(
        Box((0.12, panel_w, panel_h)),
        mass=3.0,
        origin=Origin(xyz=(0.04, 0.0, 0.12)),
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
    # PRIMARY: elevation tilt of the array panel about the horizontal -Y axis
    # at the rear knuckle. Panel faces up-forward (+X); -Y axis raises further.
    model.articulation(
        "elevation_tilt",
        ArticulationType.REVOLUTE,
        parent=azimuth_yoke,
        child=array_panel,
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
    panel = object_model.get_part("array_panel")
    azimuth = object_model.get_articulation("azimuth_rotation")
    elevation = object_model.get_articulation("elevation_tilt")

    # intentional joint-fit overlaps at the elevation bearing
    for ea, eb, reason in [
        ("trunnion_shaft", "yoke_knuckle", "panel trunnion pin captured inside the rear pivot knuckle"),
        ("back_housing", "yoke_knuckle", "panel back housing seats against the pivot knuckle"),
        ("trunnion_shaft", "yoke_post", "trunnion pin passes through the post top into the knuckle"),
        ("panel_neck", "yoke_knuckle", "panel neck drops into and is captured by the pivot knuckle"),
        ("panel_neck", "yoke_post", "panel neck meets the post top at the bearing"),
    ]:
        ctx.allow_overlap(panel, yoke, elem_a=ea, elem_b=eb, reason=reason)

    # ---- base grounding ----
    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "base box rests on the ground at z~0",
        base_aabb is not None and abs(base_aabb[0][2]) < 0.01,
        details=f"base min z = {None if base_aabb is None else base_aabb[0][2]}",
    )

    # ---- elevation joint is revolute about horizontal Y ----
    ctx.check(
        "elevation joint is revolute about the horizontal Y axis",
        elevation.articulation_type == ArticulationType.REVOLUTE
        and tuple(elevation.axis) in ((0.0, 1.0, 0.0), (0.0, -1.0, 0.0)),
        details=f"type={elevation.articulation_type}, axis={elevation.axis}",
    )

    # elevation tilt aims the panel face up (bottom edge rises) or down
    bot_rest = _aabb_center(ctx.part_element_world_aabb(panel, elem="trim_bottom"))
    with ctx.pose({elevation: 0.5}):
        bot_up = _aabb_center(ctx.part_element_world_aabb(panel, elem="trim_bottom"))
    with ctx.pose({elevation: -0.9}):
        bot_down = _aabb_center(ctx.part_element_world_aabb(panel, elem="trim_bottom"))
    ctx.check(
        "elevation tilt aims the panel face upward (bottom edge rises) and downward",
        bot_up[2] > bot_rest[2] + 0.01 and bot_down[2] < bot_rest[2] - 0.02,
        details=f"down z={bot_down[2]}, rest z={bot_rest[2]}, up z={bot_up[2]}",
    )

    # ---- azimuth joint is revolute about +Z ----
    ctx.check(
        "azimuth joint is revolute about +Z",
        azimuth.articulation_type == ArticulationType.REVOLUTE
        and tuple(azimuth.axis) == (0.0, 0.0, 1.0),
        details=f"type={azimuth.articulation_type}, axis={azimuth.axis}",
    )
    trim_rest = _aabb_center(ctx.part_element_world_aabb(panel, elem="trim_top"))
    with ctx.pose({azimuth: math.pi / 2.0}):
        trim_spun = _aabb_center(ctx.part_element_world_aabb(panel, elem="trim_top"))
    ctx.check(
        "azimuth rotation swings the panel horizontally",
        abs(trim_spun[0] - trim_rest[0]) > 0.03 or abs(trim_spun[1] - trim_rest[1]) > 0.03,
        details=f"rest={trim_rest}, spun={trim_spun}",
    )

    # ---- flat panel geometry: no concave dish, no feed horn ----
    plate_aabb = ctx.part_element_world_aabb(panel, elem="array_plate")
    housing_aabb = ctx.part_element_world_aabb(panel, elem="back_housing")
    ctx.check(
        "flat panel front face is ahead of back housing",
        _aabb_center(plate_aabb)[0] > _aabb_center(housing_aabb)[0] + 0.01,
        details=f"plate x={_aabb_center(plate_aabb)[0]}, housing x={_aabb_center(housing_aabb)[0]}",
    )

    # panel is wide (> 0.40 m across Y)
    plate_w = plate_aabb[1][1] - plate_aabb[0][1]
    ctx.check(
        "array panel is wide (> 0.40 m across)",
        plate_w > 0.40,
        details=f"panel width = {plate_w}",
    )

    # panel is flat: width (Y) significantly exceeds projected depth (X, inflated by tilt)
    plate_x = plate_aabb[1][0] - plate_aabb[0][0]
    plate_z = plate_aabb[1][2] - plate_aabb[0][2]
    ctx.check(
        "array panel is flat (width >> projected depth)",
        plate_w > plate_x * 1.5 and plate_w > plate_z * 1.2,
        details=f"proj_x={plate_x:.3f}, width_y={plate_w:.3f}, proj_z={plate_z:.3f}",
    )

    # radiator grid exists and is on the front face
    panel_0_aabb = ctx.part_element_world_aabb(panel, elem="panel_0")
    panel_last_aabb = ctx.part_element_world_aabb(panel, elem="panel_47")
    ctx.check(
        "radiator grid elements exist on the panel front face",
        panel_0_aabb is not None and panel_last_aabb is not None,
        details="panel_0 or panel_47 missing",
    )

    # radiator elements are forward of the plate center
    plate_center_x = _aabb_center(plate_aabb)[0]
    ctx.check(
        "radiator elements protrude from the front face",
        _aabb_center(panel_0_aabb)[0] > plate_center_x - 0.01,
        details=f"panel_0 x={_aabb_center(panel_0_aabb)[0]}, plate center x={plate_center_x}",
    )

    # glowing edge trim present
    trim_aabb = ctx.part_element_world_aabb(panel, elem="trim_top")
    ctx.check(
        "glowing edge trim present around panel perimeter",
        trim_aabb is not None,
        details="trim_top missing",
    )

    # ---- base decorations ----
    grille_aabb = ctx.part_element_world_aabb(base, elem="side_grille")
    ctx.check(
        "glowing slat-grille is on the box front face",
        grille_aabb is not None and _aabb_center(grille_aabb)[1] < -0.10,
        details=f"grille y={None if grille_aabb is None else _aabb_center(grille_aabb)[1]}",
    )

    # ---- yoke mounted above the base box ----
    yoke_pos = ctx.part_world_position(yoke)
    ctx.check(
        "yoke mounted above the base box",
        yoke_pos is not None and yoke_pos[2] > 0.30,
        details=f"yoke z={None if yoke_pos is None else yoke_pos[2]}",
    )

    return ctx.report()


object_model = build_object_model()
