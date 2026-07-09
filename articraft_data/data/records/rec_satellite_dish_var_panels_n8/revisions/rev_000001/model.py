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
#     reference photo — with a thin glowing rim outline, a back hub, and a
#     center-fed feed horn on an axial boom near the focus. It tilts the dish
#     up and down to aim in elevation.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
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

    # central spar through the vertex region so shell, hub, and feed base share
    # connected geometry (no floating islands).
    dish_assembly.visual(
        Cylinder(radius=0.034, length=rim_depth_full + 0.12),
        origin=_tilt((rim_depth_full / 2.0 - 0.06, 0.0, 0.0), (0.0, math.pi / 2.0, 0.0)),
        material=gun_metal,
        name="dish_spar",
    )

    # ------------------------------------------------------------------ 8-PANEL
    # Coarse few-panel sci-fi reflector: 8 wedge-shaped parabolic segments that
    # together form the same concave paraboloid focusing at the feed horn.
    # Each panel is a closed shell (outer concave face + inner back face + edge
    # walls at the radial seams and outer rim). Seam lines and mount bolts are
    # loop-emitted alongside.
    n_panels = 8
    panel_r_inner = 0.028   # panels start inside the spar radius for connectivity
    panel_wall = 0.010
    n_r = 8                 # radial sample rings
    n_t = 6                 # angular samples per panel wedge

    def _build_panel_segment(index):
        """Build one parabolic panel wedge as a closed shell MeshGeometry."""
        g = MeshGeometry()
        t0 = 2.0 * math.pi * index / n_panels
        t1 = 2.0 * math.pi * (index + 1) / n_panels

        def _paraboloid_z(r):
            return (r * r) / (4.0 * focal)

        # --- outer (concave reflecting) surface ---
        ov = []
        for ir in range(n_r + 1):
            r = panel_r_inner + (dish_radius - panel_r_inner) * ir / n_r
            z = _paraboloid_z(r)
            for it in range(n_t + 1):
                th = t0 + (t1 - t0) * it / n_t
                ov.append(g.add_vertex(r * math.cos(th), r * math.sin(th), z))

        # --- inner (back) surface, offset by wall thickness along the axis ---
        iv = []
        for ir in range(n_r + 1):
            r = panel_r_inner + (dish_radius - panel_r_inner) * ir / n_r
            z = _paraboloid_z(r) + panel_wall
            for it in range(n_t + 1):
                th = t0 + (t1 - t0) * it / n_t
                iv.append(g.add_vertex(r * math.cos(th), r * math.sin(th), z))

        # --- triangulate outer face (normal toward dish axis / focus, -Z) ---
        for ir in range(n_r):
            for it in range(n_t):
                a = ir * (n_t + 1) + it
                b = ir * (n_t + 1) + it + 1
                c = (ir + 1) * (n_t + 1) + it
                d = (ir + 1) * (n_t + 1) + it + 1
                g.add_face(ov[a], ov[c], ov[b])
                g.add_face(ov[b], ov[c], ov[d])

        # --- triangulate inner face (normal away from focus, +Z) ---
        for ir in range(n_r):
            for it in range(n_t):
                a = ir * (n_t + 1) + it
                b = ir * (n_t + 1) + it + 1
                c = (ir + 1) * (n_t + 1) + it
                d = (ir + 1) * (n_t + 1) + it + 1
                g.add_face(iv[a], iv[b], iv[c])
                g.add_face(iv[b], iv[d], iv[c])

        # --- edge wall at theta_start radial seam ---
        for ir in range(n_r):
            a_o = ir * (n_t + 1)
            b_o = (ir + 1) * (n_t + 1)
            a_i = ir * (n_t + 1)
            b_i = (ir + 1) * (n_t + 1)
            g.add_face(ov[a_o], ov[b_o], iv[a_i])
            g.add_face(ov[b_o], iv[b_i], iv[a_i])

        # --- edge wall at theta_end radial seam ---
        for ir in range(n_r):
            a_o = ir * (n_t + 1) + n_t
            b_o = (ir + 1) * (n_t + 1) + n_t
            a_i = ir * (n_t + 1) + n_t
            b_i = (ir + 1) * (n_t + 1) + n_t
            g.add_face(ov[a_o], iv[a_i], ov[b_o])
            g.add_face(ov[b_o], iv[a_i], iv[b_i])

        # --- outer rim wall (at r = dish_radius) ---
        for it in range(n_t):
            a_o = n_r * (n_t + 1) + it
            b_o = n_r * (n_t + 1) + it + 1
            a_i = n_r * (n_t + 1) + it
            b_i = n_r * (n_t + 1) + it + 1
            g.add_face(ov[a_o], ov[b_o], iv[a_i])
            g.add_face(ov[b_o], iv[b_i], iv[a_i])

        # --- inner rim wall (at r = panel_r_inner) ---
        for it in range(n_t):
            a_o = 0 * (n_t + 1) + it
            b_o = 0 * (n_t + 1) + it + 1
            a_i = 0 * (n_t + 1) + it
            b_i = 0 * (n_t + 1) + it + 1
            g.add_face(ov[a_o], iv[a_i], ov[b_o])
            g.add_face(ov[b_o], iv[a_i], iv[b_i])

        return g

    # bowl axis is local +Z of the paraboloid frame; rotate +90 about Y so the
    # concave mouth opens toward local +X (same axis the feed/spar/hub/rim use).
    _bowl_tilt = _tilt((0.0, 0.0, 0.0), (0.0, math.pi / 2.0, 0.0))

    # --- loop-emit: 8 panel segments ---
    for i in range(n_panels):
        dish_assembly.visual(
            mesh_from_geometry(_build_panel_segment(i), f"panel_{i}"),
            origin=_bowl_tilt,
            material=dish_face,
            name=f"panel_{i}",
        )

    # --- loop-emit: 8 seam lines (thin raised ridges at each panel boundary) ---
    for i in range(n_panels):
        theta_seam = 2.0 * math.pi * i / n_panels
        # sample 4 points along the radial seam from hub to rim on the paraboloid
        seam_pts = []
        for sr in range(5):
            r = panel_r_inner + (dish_radius - panel_r_inner) * sr / 4
            z = (r * r) / (4.0 * focal) + panel_wall + 0.002
            seam_pts.append(
                (r * math.cos(theta_seam), r * math.sin(theta_seam), z)
            )
        seam_mesh = mesh_from_geometry(
            tube_from_spline_points(
                seam_pts, radius=0.004, samples_per_segment=4,
                radial_segments=6, cap_ends=True,
            ),
            f"seam_{i}",
        )
        dish_assembly.visual(
            seam_mesh,
            origin=_bowl_tilt,
            material=gun_metal,
            name=f"seam_{i}",
        )

    # --- loop-emit: 16 mount bolts (2 per panel at the outer rim) ---
    # Bolts are placed in the panel's local frame (lathe frame, axis +Z) and
    # transformed by the same rotation as the panel visuals.
    _panel_theta = math.pi / 2.0 + rest_tilt
    _pc, _ps = math.cos(_panel_theta), math.sin(_panel_theta)

    def _panel_pt(xyz):
        """Transform a point from the lathe frame to dish_assembly frame."""
        x, y, z = xyz
        return (_pc * x + _ps * z, y, -_ps * x + _pc * z + mount_lift)

    for i in range(n_panels):
        for j in range(2):
            theta_bolt = 2.0 * math.pi * (i + 0.3 + 0.4 * j) / n_panels
            r_bolt = dish_radius - 0.012
            z_bolt = (r_bolt * r_bolt) / (4.0 * focal) + panel_wall + 0.003
            bolt_x = r_bolt * math.cos(theta_bolt)
            bolt_y = r_bolt * math.sin(theta_bolt)
            bx, by, bz = _panel_pt((bolt_x, bolt_y, z_bolt - 0.002))
            # small cylinder bolt head embedded into the back face near the rim
            # bolt axis aligns with the panel's local +Z (paraboloid axis)
            dish_assembly.visual(
                Cylinder(radius=0.010, length=0.014),
                origin=Origin(xyz=(bx, by, bz), rpy=(0.0, _panel_theta, 0.0)),
                material=dark_panel,
                name=f"bolt_{i}_{j}",
            )

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

    # --- panel structure: 8 segments with seams and bolts ---
    panel_names = [f"panel_{i}" for i in range(8)]
    seam_names = [f"seam_{i}" for i in range(8)]
    bolt_names = [f"bolt_{i}_{j}" for i in range(8) for j in range(2)]
    dish_visual_names = set(v.name for v in dish.visuals)

    ctx.check(
        "reflector has exactly 8 panel segments",
        all(n in dish_visual_names for n in panel_names) and len(panel_names) == 8,
        details=f"missing: {[n for n in panel_names if n not in dish_visual_names]}",
    )
    ctx.check(
        "reflector has exactly 8 seam lines",
        all(n in dish_visual_names for n in seam_names),
        details=f"missing: {[n for n in seam_names if n not in dish_visual_names]}",
    )
    ctx.check(
        "reflector has 16 mount bolts (2 per panel)",
        all(n in dish_visual_names for n in bolt_names),
        details=f"missing: {[n for n in bolt_names if n not in dish_visual_names]}",
    )

    # panels together span the full dish width (concave paraboloid reaching the rim)
    # panel_2 covers θ≈π/2 (max +Y) and panel_6 covers θ≈3π/2 (max -Y)
    p2_aabb = ctx.part_element_world_aabb(dish, elem="panel_2")
    p6_aabb = ctx.part_element_world_aabb(dish, elem="panel_6")
    if p2_aabb is not None and p6_aabb is not None:
        span_y = max(p2_aabb[1][1], p6_aabb[1][1]) - min(p2_aabb[0][1], p6_aabb[0][1])
        ctx.check(
            "panels span the full dish diameter",
            span_y > 0.50,
            details=f"panel span y = {span_y}",
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
