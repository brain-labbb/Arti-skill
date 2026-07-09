from __future__ import annotations

# Sci-fi satellite-dish comm unit — hex-faceted phased-mirror variant.
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
#   - dish_assembly (REVOLUTE about -Y, PRIMARY elevation): a hex-faceted
#     segmented parabolic reflector — flat hexagonal mirror facets tiled in
#     concentric rings across the parabolic curve over a thin dark backing bowl,
#     with a thin glowing rim outline, a back hub, and a center-fed feed horn on
#     an axial boom near the focus. It tilts the dish up and down in elevation.

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


# ---- shared geometry helpers ------------------------------------------------

def _hex_facet_mesh(center_v, center_w, hex_radius, thickness, focal,
                    outward_offset=0.0):
    """Build a flat hexagonal mirror-facet mesh in the dish body frame.

    The dish body frame has the bowl axis along +X; the parabolic surface is
    x = (y² + z²) / (4 f).  ``center_v`` and ``center_w`` are the aperture-plane
    coordinates (dish-local Y and Z) of the facet centre.  The facet is a thin
    flat hexagonal plate oriented so its outward face normal matches the local
    parabola normal, then shifted outward along that normal by
    ``outward_offset`` so it sits proud of the backing shell.
    """
    r_sq = center_v * center_v + center_w * center_w
    depth = r_sq / (4.0 * focal)

    # Surface normal at the facet centre (pointing toward feed / +X).
    nx, ny, nz = 1.0, -center_v / (2.0 * focal), -center_w / (2.0 * focal)
    n_len = math.sqrt(nx * nx + ny * ny + nz * nz)
    nx, ny, nz = nx / n_len, ny / n_len, nz / n_len

    # Facet centre, offset outward along the normal.
    cx = depth + outward_offset * nx
    cy = center_v + outward_offset * ny
    cz = center_w + outward_offset * nz

    # Build a flat-top hex plate at origin with face normal = +Z.
    g = MeshGeometry()
    top_idx = []
    bot_idx = []
    for i in range(6):
        angle = math.pi / 6.0 + i * math.pi / 3.0
        hx = hex_radius * math.cos(angle)
        hy = hex_radius * math.sin(angle)
        top_idx.append(g.add_vertex(hx, hy, thickness / 2.0))
        bot_idx.append(g.add_vertex(hx, hy, -thickness / 2.0))

    tc = g.add_vertex(0.0, 0.0, thickness / 2.0)
    bc = g.add_vertex(0.0, 0.0, -thickness / 2.0)

    # Top face (CCW from +Z — the reflective face).
    for i in range(6):
        g.add_face(tc, top_idx[i], top_idx[(i + 1) % 6])
    # Bottom face (CCW from -Z).
    for i in range(6):
        g.add_face(bc, bot_idx[(i + 1) % 6], bot_idx[i])
    # Side walls.
    for i in range(6):
        j = (i + 1) % 6
        g.add_face(top_idx[i], bot_idx[i], bot_idx[j])
        g.add_face(top_idx[i], bot_idx[j], top_idx[j])

    # Rotate from default +Z normal to the parabola normal N.
    cos_a = nz  # dot((0,0,1), N)
    if cos_a < -0.9999:
        g.rotate((0.0, 1.0, 0.0), math.pi)
    elif cos_a < 0.9999:
        ax, ay = -ny, nx
        a_len = math.sqrt(ax * ax + ay * ay)
        if a_len > 1e-10:
            g.rotate((ax / a_len, ay / a_len, 0.0),
                     math.acos(max(-1.0, min(1.0, cos_a))))

    g.translate(cx, cy, cz)
    return g


def _hex_grid_centers(hex_size, n_rings, max_radius):
    """Return (v, w) aperture coords for a hex grid filtered to a circle.

    Uses axial-coordinate hex rings (flat-top orientation) so the facets tile
    in concentric hexagonal rings around the dish centre.
    """
    dirs = [(1, 0), (0, 1), (-1, 1), (-1, 0), (0, -1), (1, -1)]
    axial = [(0, 0)]
    for ring in range(1, n_rings + 1):
        q, r = dirs[4][0] * ring, dirs[4][1] * ring
        for d in range(6):
            for _ in range(ring):
                axial.append((q, r))
                q += dirs[d][0]
                r += dirs[d][1]

    centers = []
    for q, r in axial:
        v = hex_size * 1.5 * q
        w = hex_size * math.sqrt(3.0) * (r + q / 2.0)
        if math.sqrt(v * v + w * w) <= max_radius:
            centers.append((v, w))
    return centers


# ---- model ------------------------------------------------------------------

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
    mirror_facet = model.material("mirror_facet", rgba=(0.50, 0.58, 0.68, 1.0))
    seam_dark = model.material("seam_dark", rgba=(0.04, 0.04, 0.06, 1.0))

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

    rest_tilt = -0.62   # baked up-forward tilt about the elevation (-Y) axis (~36 deg)
    mount_lift = 0.34   # raise the dish above the pivot so its lower rim clears

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

    # ---- thin dark backing bowl behind the hex facets -----------------------
    # This solid parabolic shell sits just behind the facets and provides the
    # dark seam colour visible through the gaps between hex panels.
    n_back = 14
    back_outer = []
    back_inner = []
    wall_back = 0.004
    for k in range(n_back + 1):
        r = dish_radius * k / n_back
        depth = (r * r) / (4.0 * focal)
        back_outer.append((r, depth))
        back_inner.append((max(r - 0.003, 0.0), depth + wall_back))
    backing_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            back_outer, back_inner, segments=72,
            start_cap="flat", end_cap="flat",
        ),
        "backing_shell",
    )
    dish_assembly.visual(
        backing_shell,
        origin=_tilt((0.0, 0.0, 0.0), (0.0, math.pi / 2.0, 0.0)),
        material=seam_dark,
        name="backing_shell",
    )

    # ---- hex-faceted mirror panels tiled in concentric rings ----------------
    hex_grid_size = 0.040      # hex-grid spacing parameter (centre-to-vertex)
    facet_radius = 0.036       # actual facet vertex radius (~10 % smaller → seams)
    facet_thickness = 0.005   # plate thickness
    facet_outward = 0.006      # offset along normal so facets sit proud of backing
    n_hex_rings = 5            # concentric hex rings (centre + 5 rings)
    max_aperture_r = dish_radius - facet_radius  # keep facet edges inside the rim

    facet_centers = _hex_grid_centers(hex_grid_size, n_hex_rings, max_aperture_r)

    facet_origin = _tilt((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    # facet meshes are built directly in the dish body frame (bowl axis = +X),
    # so the placement Origin applies only rest_tilt + mount_lift (no extra
    # lathe-to-dish rotation).

    for i, (cv, cw) in enumerate(facet_centers):
        facet_mesh = mesh_from_geometry(
            _hex_facet_mesh(cv, cw, facet_radius, facet_thickness, focal,
                           outward_offset=facet_outward),
            f"panel_{i}",
        )
        dish_assembly.visual(
            facet_mesh,
            origin=facet_origin,
            material=mirror_facet,
            name=f"panel_{i}",
        )

    n_panels = len(facet_centers)

    # glowing lime rim outline hugging the dish mouth
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

    # ---- feed horn on the dish axis near the focus, on a central boom -------
    feed_x = focal + 0.02
    feed_horn = mesh_from_geometry(
        ConeGeometry(radius=0.028, height=0.12, radial_segments=22),
        "feed_horn",
    )
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
    model.articulation(
        "azimuth_rotation",
        ArticulationType.REVOLUTE,
        parent=pedestal_base,
        child=azimuth_yoke,
        origin=Origin(xyz=(azimuth_pivot_x, 0.0, azimuth_pivot_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.8, lower=-math.pi, upper=math.pi),
    )
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

    # ---- hex-facet specific checks ------------------------------------------
    # Count hex panels (should be a substantial number forming the honeycomb).
    panel_names = [v.name for v in dish.visuals if v.name.startswith("panel_")]
    ctx.check(
        "hex-faceted reflector has many distinct mirror panels",
        len(panel_names) >= 30,
        details=f"panel count = {len(panel_names)}",
    )

    # Panels should span a wide area across the dish (Y and Z).
    panel_aabbs = [ctx.part_element_world_aabb(dish, elem=n) for n in panel_names]
    panel_aabbs = [a for a in panel_aabbs if a is not None]
    if panel_aabbs:
        all_y_min = min(a[0][1] for a in panel_aabbs)
        all_y_max = max(a[1][1] for a in panel_aabbs)
        panel_span_y = all_y_max - all_y_min
    else:
        panel_span_y = 0.0
    ctx.check(
        "hex panels span most of the dish width",
        panel_span_y > 0.40,
        details=f"panel Y span = {panel_span_y}",
    )

    # Outer-ring panels should be further from the axis than inner panels,
    # confirming the concentric-ring tiling covers the bowl.
    p0_aabb = ctx.part_element_world_aabb(dish, elem="panel_0")
    if p0_aabb and panel_aabbs:
        p0_cz = _aabb_center(p0_aabb)[2]
        farthest_dz = max(abs(_aabb_center(a)[2] - p0_cz) for a in panel_aabbs)
        ctx.check(
            "outer-ring panels extend well beyond the centre panel",
            farthest_dz > 0.12,
            details=f"farthest panel dz = {farthest_dz}",
        )

    # The backing shell should exist behind the facets.
    backing_aabb = ctx.part_element_world_aabb(dish, elem="backing_shell")
    ctx.check(
        "dark backing shell present behind hex facets",
        backing_aabb is not None,
        details="backing_shell AABB is None",
    )

    return ctx.report()


object_model = build_object_model()
