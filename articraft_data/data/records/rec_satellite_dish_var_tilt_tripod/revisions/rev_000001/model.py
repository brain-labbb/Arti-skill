from __future__ import annotations

# Sci-fi satellite-dish comm unit — tripod gimbal variant.
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
#     port lights, and a central tripod hub on the top deck.
#   - tripod_leg_0/1/2 (FIXED to base): three splayed angled struts descending
#     from the hub perimeter to footpads on the deck, forming a stable tripod.
#   - azimuth_yoke (REVOLUTE about +Z, azimuth): a short central mast rising
#     from the hub to a tilt knuckle; swings the dish left/right.
#   - dish_assembly (REVOLUTE about -Y, elevation): the true concave parabolic
#     reflector (lathed shell) with a thin glowing rim outline, a back hub, and
#     a center-fed feed horn on an axial boom near the focus. Tilts up/down.

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

    # ========================================================= TRIPOD HUB PARAMS
    hub_r = 0.070          # hub disk radius
    hub_h = 0.100          # hub height (from deck to top)
    hub_cx = 0.0           # centered on deck
    hub_top_z = bz + hub_h  # top of hub = azimuth bearing plane

    leg_attach_z = bz + hub_h - 0.028  # leg attach height (below bearing ring)
    deck_top_z = bz + 0.010  # top surface of the deck plate
    footpad_r = 0.20       # footpad radial distance from hub center
    footpad_thickness = 0.014
    footpad_radius = 0.036
    strut_radius = 0.015

    # mast
    mast_h = 0.20          # mast height from hub top to knuckle center
    mast_r = 0.032

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

    # ---- tripod hub on the top deck -----------------------------------------
    pedestal_base.visual(
        Cylinder(radius=hub_r, length=hub_h),
        origin=Origin(xyz=(hub_cx, 0.0, bz + hub_h / 2.0)),
        material=gun_metal,
        name="tripod_hub",
    )
    # bearing ring at hub top (azimuth bearing seat)
    pedestal_base.visual(
        Cylinder(radius=hub_r + 0.010, length=0.008),
        origin=Origin(xyz=(hub_cx, 0.0, hub_top_z - 0.004)),
        material=dark_panel,
        name="hub_bearing_ring",
    )
    # teal accent ring around hub mid-height
    pedestal_base.visual(
        Cylinder(radius=hub_r + 0.004, length=0.008),
        origin=Origin(xyz=(hub_cx, 0.0, bz + hub_h * 0.45)),
        material=glow_teal,
        name="hub_accent_ring",
    )
    # hub bolts around the bearing ring perimeter
    for i in range(6):
        a = i * math.pi / 3.0
        pedestal_base.visual(
            Cylinder(radius=0.005, length=0.008),
            origin=Origin(xyz=(hub_cx + (hub_r + 0.006) * math.cos(a),
                               (hub_r + 0.006) * math.sin(a),
                               hub_top_z - 0.003)),
            material=accent_red,
            name=f"hub_bolt_{i}",
        )

    pedestal_base.inertial = Inertial.from_geometry(
        Box((bx, by, bz + hub_h)),
        mass=15.0,
        origin=Origin(xyz=(0.0, 0.0, box_cz)),
    )

    # ========================================================= TRIPOD LEGS (×3)
    # Shared geometry helper: each leg is a splayed strut from the hub perimeter
    # down to a footpad on the deck, with bolts securing the pad.
    def _build_tripod_leg(leg_index: int, angle_rad: float):
        name = f"tripod_leg_{leg_index}"
        leg = model.part(name)

        ca, sa = math.cos(angle_rad), math.sin(angle_rad)

        # Leg part frame origin = hub attachment point (world coords).
        # Strut and footpad geometry expressed relative to this origin.
        fp_dx = (footpad_r - leg_attach_r) * ca
        fp_dy = (footpad_r - leg_attach_r) * sa
        fp_dz = (deck_top_z + footpad_thickness) - leg_attach_z  # strut ends at footpad top

        leg_attach_r_local = hub_r  # same as leg_attach_r

        # Strut: angled tube from hub perimeter down to footpad
        mid_x = fp_dx * 0.50
        mid_y = fp_dy * 0.50
        mid_z = fp_dz * 0.50 + 0.008  # slight upward bow for structural realism
        strut = mesh_from_geometry(
            tube_from_spline_points(
                [(0.0, 0.0, 0.0), (mid_x, mid_y, mid_z), (fp_dx, fp_dy, fp_dz)],
                radius=strut_radius, samples_per_segment=10,
                radial_segments=14, cap_ends=True,
            ),
            f"{name}_strut",
        )
        leg.visual(strut, material=gun_metal, name=f"{name}_strut")

        # Gusset collar at the hub attachment end
        leg.visual(
            Cylinder(radius=strut_radius + 0.008, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, -0.004)),
            material=dark_panel,
            name=f"{name}_gusset",
        )

        # Footpad: flat disk on the deck
        leg.visual(
            Cylinder(radius=footpad_radius, length=footpad_thickness),
            origin=Origin(xyz=(fp_dx, fp_dy, fp_dz - footpad_thickness / 2.0)),
            material=dark_panel,
            name=f"{name}_footpad",
        )

        # Bolts on footpad (3 hex bolts in a triangle pattern)
        for j in range(3):
            ba = j * 2.0 * math.pi / 3.0 + angle_rad
            br = 0.020
            bolt_x = fp_dx + br * math.cos(ba)
            bolt_y = fp_dy + br * math.sin(ba)
            bolt_z = fp_dz - footpad_thickness + 0.003
            leg.visual(
                Cylinder(radius=0.004, length=0.006),
                origin=Origin(xyz=(bolt_x, bolt_y, bolt_z)),
                material=accent_red,
                name=f"{name}_bolt_{j}",
            )

        # FIXED joint: leg attaches to base at the hub perimeter
        joint_x = hub_cx + hub_r * ca
        joint_y = hub_r * sa
        model.articulation(
            f"base_to_{name}",
            ArticulationType.FIXED,
            parent=pedestal_base,
            child=leg,
            origin=Origin(xyz=(joint_x, joint_y, leg_attach_z)),
        )

        leg.inertial = Inertial.from_geometry(
            Box((0.06, 0.06, 0.12)),
            mass=0.6,
            origin=Origin(xyz=(fp_dx * 0.5, fp_dy * 0.5, fp_dz * 0.5)),
        )
        return leg

    leg_attach_r = hub_r  # legs attach at hub perimeter
    tripod_legs = []
    for i in range(3):
        angle = i * 2.0 * math.pi / 3.0
        tripod_legs.append(_build_tripod_leg(i, angle))

    # ============================================================= AZIMUTH MAST
    # A short central mast rises from the hub to a tilt knuckle that carries the
    # dish. Frame origin = azimuth pivot at hub top center.
    azimuth_yoke = model.part("azimuth_yoke")

    # Mast collar at the base (where mast enters the hub bearing)
    azimuth_yoke.visual(
        Cylinder(radius=mast_r + 0.012, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=dark_panel,
        name="mast_collar",
    )
    # Mast shaft: central tube from hub top to knuckle
    azimuth_yoke.visual(
        Cylinder(radius=mast_r, length=mast_h),
        origin=Origin(xyz=(0.0, 0.0, mast_h / 2.0)),
        material=gun_metal,
        name="mast_shaft",
    )
    # Teal accent stripe on the mast
    azimuth_yoke.visual(
        Cylinder(radius=mast_r + 0.003, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, mast_h * 0.55)),
        material=glow_teal,
        name="mast_accent",
    )
    # Cable conduit running up the mast (sci-fi detail)
    conduit_path = [
        (mast_r + 0.010, 0.0, 0.020),
        (mast_r + 0.008, 0.0, mast_h * 0.35),
        (mast_r + 0.006, 0.0, mast_h * 0.70),
        (mast_r + 0.004, 0.0, mast_h - 0.010),
    ]
    mast_conduit = mesh_from_geometry(
        tube_from_spline_points(
            conduit_path,
            radius=0.008, samples_per_segment=8, radial_segments=10, cap_ends=True,
        ),
        "mast_conduit",
    )
    azimuth_yoke.visual(mast_conduit, material=matte_black, name="mast_conduit")

    # Knuckle: horizontal cylinder at the mast top (elevation pivot bearing)
    knuckle_z = mast_h
    azimuth_yoke.visual(
        Cylinder(radius=0.046, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, knuckle_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_panel,
        name="yoke_knuckle",
    )

    azimuth_yoke.inertial = Inertial.from_geometry(
        Box((0.10, 0.10, mast_h + 0.06)),
        mass=2.0,
        origin=Origin(xyz=(0.0, 0.0, mast_h / 2.0)),
    )

    # =============================================================== DISH ASSEMBLY
    # Local frame origin = the elevation pivot (on the trunnion axis, along +/-Y).
    # At rest (q=0) the dish is baked tilted UP by `rest_tilt` so it perches above
    # the mast and points up-forward like the reference. Positive elevation
    # tilts further up; negative brings it toward the horizon.
    dish_assembly = model.part("dish_assembly")

    rest_tilt = -0.62   # baked up-forward tilt about the elevation (-Y) axis (~36 deg)
    mount_lift = 0.34   # raise the dish above the pivot so its lower rim clears
                        # the mast; a neck drops back down to the knuckle.

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

    # ---- feed horn on the dish axis near the focus, on a central boom --------
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
        Box((0.65, 0.65, 0.65)),
        mass=3.4,
        origin=Origin(xyz=(0.06, 0.0, 0.12)),
    )

    # ============================================================ ARTICULATIONS
    # SECONDARY: azimuth rotation of the mast+dish about the vertical axis at hub top.
    model.articulation(
        "azimuth_rotation",
        ArticulationType.REVOLUTE,
        parent=pedestal_base,
        child=azimuth_yoke,
        origin=Origin(xyz=(hub_cx, 0.0, hub_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.8, lower=-math.pi, upper=math.pi),
    )
    # PRIMARY: elevation tilt of the dish about the horizontal -Y axis at the
    # knuckle. Dish opens up-forward (+X); -Y axis raises the mouth further.
    model.articulation(
        "elevation_tilt",
        ArticulationType.REVOLUTE,
        parent=azimuth_yoke,
        child=dish_assembly,
        origin=Origin(xyz=(0.0, 0.0, knuckle_z)),
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

    # ---- intentional joint-fit overlaps at the elevation bearing ---------------
    for ea, eb, reason in [
        ("trunnion_shaft", "yoke_knuckle", "dish trunnion pin captured inside the pivot knuckle"),
        ("trunnion_shaft", "mast_shaft", "trunnion pin half-embedded in mast top at the bearing"),
        ("dish_neck", "yoke_knuckle", "dish neck drops into and is captured by the pivot knuckle"),
    ]:
        ctx.allow_overlap(dish, yoke, elem_a=ea, elem_b=eb, reason=reason)

    # ---- intentional overlaps at the hub-leg attachment points ----------------
    # The leg struts and gussets emerge from the hub surface, so they slightly
    # penetrate the hub cylinder at the attachment point. This is the real
    # mechanical connection (welded/bolted strut into a hub boss).
    for i in range(3):
        leg_part = object_model.get_part(f"tripod_leg_{i}")
        strut_elem = f"tripod_leg_{i}_strut"
        gusset_elem = f"tripod_leg_{i}_gusset"
        ctx.allow_overlap(
            base, leg_part,
            elem_a="tripod_hub", elem_b=strut_elem,
            reason=f"tripod_leg_{i} strut embeds into the hub at the attachment point",
        )
        ctx.allow_overlap(
            base, leg_part,
            elem_a="tripod_hub", elem_b=gusset_elem,
            reason=f"tripod_leg_{i} gusset collar seats against the hub surface",
        )
        # Proof: leg strut contacts the hub (connected, not floating)
        ctx.expect_contact(
            base, leg_part,
            elem_a="tripod_hub", elem_b=strut_elem,
            contact_tol=0.020,
            name=f"tripod_leg_{i} strut contacts the hub",
        )

    # ---- base rests on the ground at z~0 ----
    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "base box rests on the ground at z~0",
        base_aabb is not None and abs(base_aabb[0][2]) < 0.01,
        details=f"base min z = {None if base_aabb is None else base_aabb[0][2]}",
    )

    # ---- tripod legs exist and are splayed outward ----
    hub_center = ctx.part_element_world_aabb(base, elem="tripod_hub")
    for i in range(3):
        leg = object_model.get_part(f"tripod_leg_{i}")
        footpad_aabb = ctx.part_element_world_aabb(leg, elem=f"tripod_leg_{i}_footpad")
        ctx.check(
            f"tripod_leg_{i} footpad is on the deck",
            footpad_aabb is not None and footpad_aabb[0][2] > 0.30,
            details=f"footpad min z = {None if footpad_aabb is None else footpad_aabb[0][2]}",
        )
        # footpad should be further from center than the hub (splayed)
        if hub_center is not None and footpad_aabb is not None:
            hub_xy = _aabb_center(hub_center)
            fp_xy = _aabb_center(footpad_aabb)
            hub_r = math.hypot(hub_xy[0], hub_xy[1])
            fp_r = math.hypot(fp_xy[0], fp_xy[1])
            ctx.check(
                f"tripod_leg_{i} footpad splayed outward from hub",
                fp_r > hub_r + 0.05,
                details=f"hub_r={hub_r:.3f}, footpad_r={fp_r:.3f}",
            )

    # ---- elevation joint is revolute about the horizontal Y axis ----
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

    # ---- azimuth joint is revolute about +Z ----
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

    # ---- parabolic dish opens forward (rim ahead of the back hub) ----
    rim_aabb = ctx.part_element_world_aabb(dish, elem="dish_rim")
    hub_aabb = ctx.part_element_world_aabb(dish, elem="dish_hub")
    ctx.check(
        "parabolic dish opens forward (rim ahead of the back hub)",
        _aabb_center(rim_aabb)[0] > _aabb_center(hub_aabb)[0] + 0.05,
        details=f"rim x={_aabb_center(rim_aabb)[0]}, hub x={_aabb_center(hub_aabb)[0]}",
    )

    # ---- feed horn is mounted ahead of the reflector near the focus ----
    feed_aabb = ctx.part_element_world_aabb(dish, elem="feed_horn")
    ctx.check(
        "feed horn is mounted ahead of the reflector near the focus",
        _aabb_center(feed_aabb)[0] > _aabb_center(rim_aabb)[0] - 0.12,
        details=f"feed x={_aabb_center(feed_aabb)[0]}, rim x={_aabb_center(rim_aabb)[0]}",
    )

    # ---- reflector dish is wide (> 0.5 m across) ----
    dish_w = rim_aabb[1][1] - rim_aabb[0][1]
    ctx.check(
        "reflector dish is wide (> 0.5 m across)",
        dish_w > 0.50,
        details=f"dish width = {dish_w}",
    )

    # ---- glowing slat-grille is on the box front face ----
    grille_aabb = ctx.part_element_world_aabb(base, elem="side_grille")
    ctx.check(
        "glowing slat-grille is on the box front face",
        grille_aabb is not None and _aabb_center(grille_aabb)[1] < -0.10,
        details=f"grille y={None if grille_aabb is None else _aabb_center(grille_aabb)[1]}",
    )

    # ---- mast mounted above the base box on the tripod hub ----
    yoke_pos = ctx.part_world_position(yoke)
    ctx.check(
        "mast assembly mounted above the base box",
        yoke_pos is not None and yoke_pos[2] > 0.40,
        details=f"yoke z={None if yoke_pos is None else yoke_pos[2]}",
    )

    # ---- tripod hub is visible on the deck ----
    ctx.check(
        "tripod hub is on the base deck",
        hub_center is not None and _aabb_center(hub_center)[2] > bz + 0.02,
        details=f"hub z={None if hub_center is None else _aabb_center(hub_center)[2]}",
    )

    return ctx.report()


# module-level constant for test reference
bz = 0.34

object_model = build_object_model()
