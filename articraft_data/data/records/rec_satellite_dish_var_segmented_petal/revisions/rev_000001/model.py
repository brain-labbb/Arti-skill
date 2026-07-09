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
#   - dish_assembly (REVOLUTE about -Y, PRIMARY elevation): a true concave
#     parabolic reflector built from 8 radial petal segments (panel_0..panel_7)
#     radiating from the vertex to the rim with thin dark gap lines between
#     adjacent petals and mount bolts at the hub. A thin glowing rim outline,
#     a back hub, and a center-fed feed horn on an axial boom complete the
#     assembly. It tilts the dish up and down to aim in elevation.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
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

    # central spar through the vertex region so hub, petals, and feed base share
    # connected geometry (no floating islands). Radius exceeds petal_inner_r so
    # the spar surface contacts the inner petal edges, anchoring them.
    spar_radius = 0.052
    dish_assembly.visual(
        Cylinder(radius=spar_radius, length=rim_depth_full + 0.12),
        origin=_tilt((rim_depth_full / 2.0 - 0.06, 0.0, 0.0), (0.0, math.pi / 2.0, 0.0)),
        material=gun_metal,
        name="dish_spar",
    )

    # --------------------------------------------------------------------------
    # SEGMENTED-PETAL PARABOLIC REFLECTOR
    # The dish surface is split into N_PETALS long curved pie-wedge petals
    # radiating from the vertex to the rim, like a deployable space-antenna
    # unfurled from folded segments. Thin dark radial gap lines separate
    # adjacent petals. Small mount bolts anchor each petal to the back hub.
    # Every petal shares the same parabolic profile so the assembled dish still
    # focuses to the feed horn at the prime focus.
    # --------------------------------------------------------------------------
    N_PETALS = 8
    petal_wall = 0.008
    petal_inner_r = 0.045   # inner radius near hub vertex
    gap_angle = 0.025       # angular gap between petals (~1.4 deg, visible dark line)
    petal_arc = (2.0 * math.pi / N_PETALS) - gap_angle  # angular span per petal
    n_radial = 12           # radial subdivisions per petal
    n_angular = 6           # angular subdivisions per petal

    def _build_petal_mesh(theta_offset: float) -> MeshGeometry:
        """Build one parabolic petal wedge in the +Z-axial lathe frame.

        The petal spans theta in [theta_offset, theta_offset + petal_arc] and
        r in [petal_inner_r, dish_radius]. Outer face follows z = r^2/(4f);
        inner face is offset by petal_wall. Edge strips close the shell.
        """
        geom = MeshGeometry()
        # outer and inner vertex grids
        outer_start = 0
        inner_start = (n_radial + 1) * (n_angular + 1)
        for ri in range(n_radial + 1):
            r = petal_inner_r + (dish_radius - petal_inner_r) * ri / n_radial
            depth = (r * r) / (4.0 * focal)
            for ai in range(n_angular + 1):
                theta = theta_offset + petal_arc * ai / n_angular
                # outer surface
                geom.add_vertex(r * math.cos(theta), r * math.sin(theta), depth)
                # inner surface (offset outward by wall thickness)
                geom.add_vertex(r * math.cos(theta), r * math.sin(theta), depth + petal_wall)
        # outer face triangles (normals pointing inward toward focus, i.e. -Z direction)
        for ri in range(n_radial):
            for ai in range(n_angular):
                v00 = outer_start + ri * (n_angular + 1) + ai
                v10 = outer_start + (ri + 1) * (n_angular + 1) + ai
                v01 = outer_start + ri * (n_angular + 1) + ai + 1
                v11 = outer_start + (ri + 1) * (n_angular + 1) + ai + 1
                geom.add_face(v00, v01, v10)
                geom.add_face(v10, v01, v11)
        # inner face triangles (normals pointing outward, +Z)
        for ri in range(n_radial):
            for ai in range(n_angular):
                v00 = inner_start + ri * (n_angular + 1) + ai
                v10 = inner_start + (ri + 1) * (n_angular + 1) + ai
                v01 = inner_start + ri * (n_angular + 1) + ai + 1
                v11 = inner_start + (ri + 1) * (n_angular + 1) + ai + 1
                geom.add_face(v00, v10, v01)
                geom.add_face(v10, v11, v01)
        # edge strips: outer rim (r = dish_radius)
        rim_ri = n_radial
        for ai in range(n_angular):
            o0 = outer_start + rim_ri * (n_angular + 1) + ai
            o1 = outer_start + rim_ri * (n_angular + 1) + ai + 1
            i0 = inner_start + rim_ri * (n_angular + 1) + ai
            i1 = inner_start + rim_ri * (n_angular + 1) + ai + 1
            geom.add_face(o0, i0, o1)
            geom.add_face(o1, i0, i1)
        # edge strip: inner edge (r = petal_inner_r) near hub
        hub_ri = 0
        for ai in range(n_angular):
            o0 = outer_start + hub_ri * (n_angular + 1) + ai
            o1 = outer_start + hub_ri * (n_angular + 1) + ai + 1
            i0 = inner_start + hub_ri * (n_angular + 1) + ai
            i1 = inner_start + hub_ri * (n_angular + 1) + ai + 1
            geom.add_face(o0, o1, i0)
            geom.add_face(o1, i1, i0)
        # edge strips: angular sides (theta = theta_offset and theta = theta_offset + petal_arc)
        for side_ai in (0, n_angular):
            for ri in range(n_radial):
                o0 = outer_start + ri * (n_angular + 1) + side_ai
                o1 = outer_start + (ri + 1) * (n_angular + 1) + side_ai
                i0 = inner_start + ri * (n_angular + 1) + side_ai
                i1 = inner_start + (ri + 1) * (n_angular + 1) + side_ai
                if side_ai == 0:
                    geom.add_face(o0, o1, i0)
                    geom.add_face(o1, i1, i0)
                else:
                    geom.add_face(o0, i0, o1)
                    geom.add_face(o1, i0, i1)
        return geom

    # bolt geometry: small cylinders where each petal meets the back hub
    bolt_radius = 0.008
    bolt_length = 0.016
    bolt_mesh = mesh_from_geometry(
        CylinderGeometry(bolt_radius, bolt_length, radial_segments=8),
        "petal_bolt",
    )

    for i in range(N_PETALS):
        theta_off = i * (2.0 * math.pi / N_PETALS) + gap_angle / 2.0
        petal_mesh = mesh_from_geometry(_build_petal_mesh(theta_off), f"panel_{i}")
        dish_assembly.visual(
            petal_mesh,
            origin=_tilt((0.0, 0.0, 0.0), (0.0, math.pi / 2.0, 0.0)),
            material=dish_face,
            name=f"panel_{i}",
        )
        # mount bolt at inner edge of petal, on the back face near the hub
        bolt_r = petal_inner_r + 0.010
        bolt_theta = theta_off + petal_arc / 2.0  # center of petal arc
        bolt_depth = (bolt_r * bolt_r) / (4.0 * focal) + petal_wall + bolt_length / 2.0
        # bolt position in the lathe +Z frame, then tilt to dish frame
        bolt_x_lathe = bolt_r * math.cos(bolt_theta)
        bolt_y_lathe = bolt_r * math.sin(bolt_theta)
        bolt_z_lathe = bolt_depth
        # lathe +Z maps to dish +X after the pi/2 Y rotation:
        # (x_lathe, y_lathe, z_lathe) -> (z_lathe, -y_lathe, x_lathe) in dish frame? No.
        # The _tilt applies rest_tilt; the origin rotation (0, pi/2, 0) rotates the lathe
        # frame so lathe +Z -> dish +X, lathe +X -> dish -Z, lathe +Y -> dish +Y.
        # In dish local frame (before tilt):
        dish_bx = bolt_z_lathe
        dish_by = bolt_y_lathe
        dish_bz = -bolt_x_lathe
        bolt_origin = _tilt((dish_bx, dish_by, dish_bz), (0.0, math.pi / 2.0, 0.0))
        dish_assembly.visual(
            bolt_mesh,
            origin=bolt_origin,
            material=gun_metal,
            name=f"bolt_{i}",
        )
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

    # ---- feed horn on the dish axis near the focus, on three struts ---------
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
    # out to the feed horn (center-fed, like the reference photo — no rim-mounted
    # tripod struts). Overlaps the dish spar at the vertex so it stays connected.
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
    tip_rest = _aabb_center(ctx.part_element_world_aabb(dish, elem="feed_tip"))
    with ctx.pose({elevation: 0.5}):
        tip_up = _aabb_center(ctx.part_element_world_aabb(dish, elem="feed_tip"))
    with ctx.pose({elevation: -0.9}):
        tip_down = _aabb_center(ctx.part_element_world_aabb(dish, elem="feed_tip"))
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

    # --- segmented-petal dish checks -----------------------------------------
    # verify all 8 petal panels exist as named visuals
    petal_names = [f"panel_{i}" for i in range(8)]
    bolt_names = [f"bolt_{i}" for i in range(8)]
    all_visual_names = {v.name for v in dish.visuals if v.name}
    ctx.check(
        "all 8 petal panels are present on the dish",
        all(n in all_visual_names for n in petal_names),
        details=f"missing: {[n for n in petal_names if n not in all_visual_names]}",
    )
    ctx.check(
        "all 8 petal mount bolts are present on the dish",
        all(n in all_visual_names for n in bolt_names),
        details=f"missing: {[n for n in bolt_names if n not in all_visual_names]}",
    )

    # verify petal panels span most of the dish diameter (concave reflector shape)
    # panels at ±90° (panel_2, panel_6) span the Y axis most fully
    panel_2_aabb = ctx.part_element_world_aabb(dish, elem="panel_2")
    panel_6_aabb = ctx.part_element_world_aabb(dish, elem="panel_6")
    if panel_2_aabb is not None and panel_6_aabb is not None:
        combined_y_span = max(panel_2_aabb[1][1], panel_6_aabb[1][1]) - min(panel_2_aabb[0][1], panel_6_aabb[0][1])
        ctx.check(
            "opposing petals span the dish diameter",
            combined_y_span > 0.45,
            details=f"combined Y span = {combined_y_span}",
        )

    # verify petals are concave (panel center is forward of the back hub)
    panel_0_aabb = ctx.part_element_world_aabb(dish, elem="panel_0")
    panel_0_center = _aabb_center(panel_0_aabb) if panel_0_aabb is not None else None
    if panel_0_center is not None:
        ctx.check(
            "petal panels are positioned in front of the hub (concave dish)",
            panel_0_center[0] > _aabb_center(hub_aabb)[0] + 0.01,
            details=f"panel_0 x={panel_0_center[0]}, hub x={_aabb_center(hub_aabb)[0]}",
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
