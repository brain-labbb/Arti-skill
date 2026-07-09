from __future__ import annotations

"""Six-wheeled armored personnel carrier (6x6 BTR-style APC).

Reference image: picture/Military/Tank/001.png

- Faceted olive-drab welded hull, ~7.6 m long x ~2.9 m wide, turret roof ~2.6 m.
- Wedge nose with angled glacis plates, slab sides with tumblehome facets,
  fender strips over cut wheel arches, stowage boxes, grab rails, side door
  panels, red/white tactical marking.
- Low cylindrical (conic-frustum) turret with tan camo patches, hatch ring,
  long thin autocannon with muzzle device plus coaxial MG.
- Articulations: continuous turret yaw, revolute gun elevation (-5..+35 deg),
  6 continuous road wheels (~1.1 m diameter), 3 evenly spaced axle pairs.

Frame: +X forward, +Y left, +Z up. Ground plane at z = 0.
"""

from math import cos, pi, sin

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------- dimensions
HULL_LEN_FRONT = 3.80
HULL_LEN_REAR = -3.80
HULL_HALF_W = 1.42
HULL_BOTTOM = 0.50
HULL_ROOF = 2.20

WHEEL_RADIUS = 0.55  # ~1.1 m diameter (carcass + tread blocks)
TIRE_CARCASS_R = 0.525  # tread blocks add ~0.025 m
WHEEL_WIDTH = 0.35
AXLE_Z = WHEEL_RADIUS
AXLE_X = (2.50, 0.00, -2.50)
WHEEL_Y = 1.22
ARCH_RADIUS = 0.62

TURRET_X = 0.40
TURRET_BASE_R = 0.88
TURRET_TOP_R = 0.66
TURRET_H = 0.44  # shell roof at 2.20 + 0.44 = 2.64 m

GUN_TRUNNION = (0.88, 0.0, 0.23)  # in turret frame
GUN_TUBE_LEN = 3.35
GUN_TUBE_R = 0.052

ELEV_LOWER = -5.0 * pi / 180.0
ELEV_UPPER = 35.0 * pi / 180.0


# ---------------------------------------------------------------- hull solid
def _build_hull_solid() -> cq.Workplane:
    # Side profile (x, z): flat belly, sloped lower/upper glacis nose,
    # flat roof, sloped rear plates.
    profile = [
        (-3.60, HULL_BOTTOM),
        (2.55, HULL_BOTTOM),
        (HULL_LEN_FRONT, 1.30),  # nose ridge
        (1.55, HULL_ROOF),
        (-3.55, HULL_ROOF),
        (HULL_LEN_REAR, 1.40),
    ]
    body = cq.Workplane("XZ").polyline(profile).close().extrude(HULL_HALF_W, both=True)

    def yz_prism(points: list[tuple[float, float]]) -> cq.Workplane:
        return cq.Workplane("YZ").polyline(points).close().extrude(4.2, both=True)

    # Lower tumblehome facets (boat-hull slope from fender line to belly).
    lower_cut = [(1.03, 0.45), (HULL_HALF_W, 1.10), (1.80, 1.10), (1.80, 0.45)]
    body = body.cut(yz_prism(lower_cut))
    body = body.cut(yz_prism([(-y, z) for y, z in lower_cut]))

    # Upper tumblehome facets (sides lean inward toward the roof).
    upper_cut = [(HULL_HALF_W, 1.80), (1.18, HULL_ROOF), (1.18, 2.50), (1.80, 2.50), (1.80, 1.80)]
    body = body.cut(yz_prism(upper_cut))
    body = body.cut(yz_prism([(-y, z) for y, z in upper_cut]))

    # Wedge nose: plan-view side facets narrowing the bow.
    nose_cut = [(2.70, HULL_HALF_W), (3.90, 0.92), (4.30, 2.20), (2.70, 2.20)]
    nose = cq.Workplane("XY").polyline(nose_cut).close().extrude(3.0)
    body = body.cut(nose)
    nose_m = (
        cq.Workplane("XY")
        .polyline([(x, -y) for x, y in nose_cut])
        .close()
        .extrude(3.0)
    )
    body = body.cut(nose_m)

    # Wheel arches: cylindrical wells cut into both lower sides.
    left_arches = (
        cq.Workplane("XZ")
        .workplane(offset=-1.60)
        .pushPoints([(x, AXLE_Z) for x in AXLE_X])
        .circle(ARCH_RADIUS)
        .extrude(0.68)
    )
    right_arches = (
        cq.Workplane("XZ")
        .workplane(offset=0.92)
        .pushPoints([(x, AXLE_Z) for x in AXLE_X])
        .circle(ARCH_RADIUS)
        .extrude(0.68)
    )
    body = body.cut(left_arches).cut(right_arches)
    return body


# --------------------------------------------------------------- turret cq
def _frustum(r0: float, r1: float, z0: float, z1: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .workplane(offset=z0)
        .circle(r0)
        .workplane(offset=z1 - z0)
        .circle(r1)
        .loft()
    )


def _turret_shell() -> cq.Workplane:
    return _frustum(TURRET_BASE_R, TURRET_TOP_R, 0.0, TURRET_H)


def _camo_patches() -> cq.Workplane:
    # Conformal tan patches hugging the turret cone (slightly embedded).
    outer = _frustum(TURRET_BASE_R + 0.022, TURRET_TOP_R + 0.022, 0.05, 0.36)
    inner = _frustum(TURRET_BASE_R - 0.005, TURRET_TOP_R - 0.005, 0.0, TURRET_H)
    shell = outer.cut(inner)

    def sector(center_deg: float, span_deg: float) -> cq.Workplane:
        a0 = (center_deg - span_deg / 2.0) * pi / 180.0
        a1 = (center_deg + span_deg / 2.0) * pi / 180.0
        am = (a0 + a1) / 2.0
        pts = [
            (0.0, 0.0),
            (1.2 * cos(a0), 1.2 * sin(a0)),
            (1.2 * cos(am), 1.2 * sin(am)),
            (1.2 * cos(a1), 1.2 * sin(a1)),
        ]
        return cq.Workplane("XY").polyline(pts).close().extrude(0.45)

    patches = shell.intersect(sector(40.0, 55.0))
    patches = patches.union(shell.intersect(sector(150.0, 45.0)))
    patches = patches.union(shell.intersect(sector(255.0, 60.0)))
    return patches


def _hatch_ring() -> cq.Workplane:
    ring = (
        cq.Workplane("XY")
        .workplane(offset=0.43)
        .circle(0.25)
        .circle(0.185)
        .extrude(0.07)
    )
    return ring.translate((-0.22, 0.18, 0.0))


# ----------------------------------------------------------------- model
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="six_wheeled_apc")

    olive = model.material("olive_drab", rgba=(0.345, 0.375, 0.255, 1.0))
    olive_dark = model.material("olive_dark", rgba=(0.245, 0.270, 0.180, 1.0))
    hub_green = model.material("hub_green", rgba=(0.310, 0.355, 0.225, 1.0))
    rubber = model.material("tire_rubber", rgba=(0.115, 0.115, 0.125, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.160, 0.170, 0.180, 1.0))
    camo_tan = model.material("camo_tan", rgba=(0.620, 0.540, 0.380, 1.0))
    marking_white = model.material("marking_white", rgba=(0.900, 0.900, 0.880, 1.0))
    marking_red = model.material("marking_red", rgba=(0.720, 0.110, 0.100, 1.0))
    glass_dark = model.material("glass_dark", rgba=(0.080, 0.095, 0.110, 1.0))

    # ------------------------------------------------------------- hull
    hull = model.part("hull")
    hull.visual(mesh_from_cadquery(_build_hull_solid(), "apc_hull"), material=olive, name="hull_shell")

    # Fender strips above the wheel arches (overall width ~2.9 m).
    for sgn, nm in ((1.0, "left_fender_strip"), (-1.0, "right_fender_strip")):
        hull.visual(
            Box((5.30, 0.07, 0.06)),
            origin=Origin(xyz=(0.0, sgn * 1.445, 1.16)),
            material=olive_dark,
            name=nm,
        )

    # Side door panels (upper + lower leaves) between axles 1 and 2.
    for sgn in (1.0, -1.0):
        side = "left" if sgn > 0 else "right"
        hull.visual(
            Box((0.72, 0.05, 0.40)),
            origin=Origin(xyz=(0.05, sgn * 1.43, 1.59)),
            material=olive_dark,
            name=f"{side}_door_upper_panel",
        )
        hull.visual(
            Box((0.72, 0.05, 0.26)),
            origin=Origin(xyz=(0.05, sgn * 1.43, 1.26)),
            material=olive_dark,
            name=f"{side}_door_lower_panel",
        )
        hull.visual(
            Box((0.14, 0.03, 0.04)),
            origin=Origin(xyz=(0.30, sgn * 1.465, 1.55)),
            material=gunmetal,
            name=f"{side}_door_handle",
        )

    # Stowage boxes on the hull sides.
    for sgn in (1.0, -1.0):
        side = "left" if sgn > 0 else "right"
        hull.visual(
            Box((0.85, 0.10, 0.42)),
            origin=Origin(xyz=(-1.70, sgn * 1.445, 1.44)),
            material=olive_dark,
            name=f"{side}_stowage_box_mid",
        )
        hull.visual(
            Box((0.62, 0.10, 0.38)),
            origin=Origin(xyz=(-3.00, sgn * 1.445, 1.44)),
            material=olive_dark,
            name=f"{side}_stowage_box_rear",
        )
        hull.visual(
            Box((0.55, 0.10, 0.34)),
            origin=Origin(xyz=(1.65, sgn * 1.445, 1.42)),
            material=olive_dark,
            name=f"{side}_stowage_box_front",
        )

    # Grab rails with standoff blocks on the upper side facets.
    for sgn in (1.0, -1.0):
        side = "left" if sgn > 0 else "right"
        for rx, rn in ((-1.70, "rear"), (0.30, "front")):
            hull.visual(
                Cylinder(radius=0.016, length=0.95),
                origin=Origin(xyz=(rx, sgn * 1.41, 1.95), rpy=(0.0, pi / 2.0, 0.0)),
                material=olive_dark,
                name=f"{side}_{rn}_grab_rail",
            )
            for ex in (rx - 0.40, rx + 0.40):
                hull.visual(
                    Box((0.04, 0.12, 0.04)),
                    origin=Origin(xyz=(ex, sgn * 1.37, 1.95)),
                    material=olive_dark,
                )

    # Red-and-white tactical marking on the right hull side.
    hull.visual(
        Box((0.30, 0.030, 0.24)),
        origin=Origin(xyz=(1.30, -1.437, 1.52)),
        material=marking_white,
        name="tactical_marking_plate",
    )
    hull.visual(
        Box((0.14, 0.036, 0.12)),
        origin=Origin(xyz=(1.30, -1.439, 1.52)),
        material=marking_red,
        name="tactical_marking_emblem",
    )

    # Driver vision blocks on the upper glacis (slope ~21.8 deg).
    glacis_pitch = 0.38
    for sgn in (1.0, -1.0):
        hull.visual(
            Box((0.22, 0.30, 0.10)),
            origin=Origin(xyz=(1.90, sgn * 0.45, 2.08), rpy=(0.0, glacis_pitch, 0.0)),
            material=glass_dark,
        )
    # Headlights with brush guards near the nose ridge.
    for sgn in (1.0, -1.0):
        hull.visual(
            Cylinder(radius=0.060, length=0.10),
            origin=Origin(xyz=(3.20, sgn * 0.55, 1.56), rpy=(0.0, glacis_pitch, 0.0)),
            material=gunmetal,
        )
        hull.visual(
            Box((0.05, 0.18, 0.18)),
            origin=Origin(xyz=(3.14, sgn * 0.55, 1.55), rpy=(0.0, glacis_pitch, 0.0)),
            material=olive_dark,
        )
    # Trim vane plate on the lower glacis.
    hull.visual(
        Box((1.30, 1.70, 0.05)),
        origin=Origin(xyz=(3.15, 0.0, 0.895), rpy=(0.0, -0.57, 0.0)),
        material=olive_dark,
        name="trim_vane",
    )

    # Crew hatches on the forward roof.
    for sgn in (1.0, -1.0):
        side = "left" if sgn > 0 else "right"
        hull.visual(
            Cylinder(radius=0.21, length=0.05),
            origin=Origin(xyz=(1.28, sgn * 0.52, 2.21)),
            material=olive_dark,
            name=f"{side}_crew_hatch",
        )
    # Engine deck grilles on the rear roof.
    hull.visual(Box((0.85, 1.05, 0.06)), origin=Origin(xyz=(-2.55, 0.0, 2.21)), material=olive_dark, name="engine_deck_grille")
    hull.visual(Box((0.45, 0.85, 0.05)), origin=Origin(xyz=(-3.25, 0.0, 2.20)), material=olive_dark)
    # Tail lights on the upper rear plate.
    for sgn in (1.0, -1.0):
        hull.visual(
            Box((0.06, 0.16, 0.10)),
            origin=Origin(xyz=(-3.70, sgn * 0.85, 1.70)),
            material=marking_red,
        )

    # Axle stubs reaching from the wheel wells into each hub.
    for idx, ax in enumerate(AXLE_X):
        for sgn, side in ((1.0, "left"), (-1.0, "right")):
            hull.visual(
                Cylinder(radius=0.07, length=0.22),
                origin=Origin(xyz=(ax, sgn * 1.01, AXLE_Z), rpy=(pi / 2.0, 0.0, 0.0)),
                material=gunmetal,
                name=f"{side}_axle_stub_{idx}",
            )

    # ------------------------------------------------------------- wheels
    def _tire_geom():
        return TireGeometry(
            TIRE_CARCASS_R,
            WHEEL_WIDTH,
            inner_radius=0.31,
            carcass=TireCarcass(belt_width_ratio=0.62, sidewall_bulge=0.06),
            tread=TireTread(style="block", depth=0.030, count=24, land_ratio=0.55),
            grooves=(TireGroove(center_offset=0.0, width=0.020, depth=0.012),),
            sidewall=TireSidewall(style="rounded", bulge=0.05),
            shoulder=TireShoulder(width=0.018, radius=0.008),
        )

    def _wheel_geom():
        return WheelGeometry(
            0.315,
            0.30,
            rim=WheelRim(inner_radius=0.245, flange_height=0.020, flange_thickness=0.014, bead_seat_depth=0.005),
            hub=WheelHub(
                radius=0.105,
                width=0.32,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=8, circle_diameter=0.150, hole_diameter=0.016),
            ),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.030, window_radius=0.040),
            bore=WheelBore(style="round", diameter=0.040),
        )

    # The wheel face/hub cap is one-sided, so mirror the meshes per side
    # (outboard face points +Y on the left, -Y on the right).
    tire_meshes = {
        "left": mesh_from_geometry(_tire_geom().rotate_z(pi / 2.0), "apc_road_tire_left"),
        "right": mesh_from_geometry(_tire_geom().rotate_z(-pi / 2.0), "apc_road_tire_right"),
    }
    wheel_meshes = {
        "left": mesh_from_geometry(_wheel_geom().rotate_z(pi / 2.0), "apc_wheel_rim_left"),
        "right": mesh_from_geometry(_wheel_geom().rotate_z(-pi / 2.0), "apc_wheel_rim_right"),
    }

    wheel_parts = []
    for side_sgn, side in ((1.0, "left"), (-1.0, "right")):
        for idx, ax in enumerate(AXLE_X):
            wp = model.part(f"{side}_wheel_{idx}")
            wp.visual(tire_meshes[side], material=rubber, name="tire")
            wp.visual(wheel_meshes[side], material=hub_green, name="rim")
            # Off-axis valve stem on the outboard wheel face.
            wp.visual(
                Cylinder(radius=0.012, length=0.08),
                origin=Origin(xyz=(0.30, side_sgn * 0.14, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
                material=gunmetal,
                name="valve_stem",
            )
            model.articulation(
                f"{side}_wheel_{idx}_spin",
                ArticulationType.CONTINUOUS,
                parent=hull,
                child=wp,
                origin=Origin(xyz=(ax, side_sgn * WHEEL_Y, AXLE_Z)),
                axis=(0.0, 1.0, 0.0),
                motion_limits=MotionLimits(effort=900.0, velocity=30.0),
            )
            wheel_parts.append(wp)

    # ------------------------------------------------------------- turret
    turret = model.part("turret")
    turret.visual(mesh_from_cadquery(_turret_shell(), "apc_turret_shell"), material=olive, name="turret_shell")
    turret.visual(mesh_from_cadquery(_camo_patches(), "apc_turret_camo"), material=camo_tan, name="camo_patches")
    turret.visual(
        Cylinder(radius=TURRET_BASE_R + 0.05, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=olive_dark,
        name="turret_bearing_ring",
    )
    turret.visual(
        Box((0.95, 1.28, 0.28)),
        origin=Origin(xyz=(-0.55, 0.0, 0.29)),
        material=olive_dark,
        name="turret_rear_bustle_box",
    )
    for post_i, px in enumerate((-0.98, -0.68, -0.38)):
        for sgn, side in ((1.0, "left"), (-1.0, "right")):
            turret.visual(
                Box((0.055, 0.055, 0.34)),
                origin=Origin(xyz=(px, sgn * 0.69, 0.36)),
                material=gunmetal,
                name=f"{side}_basket_post_{post_i}",
            )
    turret.visual(
        Box((0.82, 0.05, 0.07)),
        origin=Origin(xyz=(-0.62, 0.69, 0.48)),
        material=gunmetal,
        name="left_bustle_basket_rail",
    )
    turret.visual(
        Box((0.82, 0.05, 0.07)),
        origin=Origin(xyz=(-0.62, -0.69, 0.48)),
        material=gunmetal,
        name="right_bustle_basket_rail",
    )
    turret.visual(
        Box((0.08, 1.42, 0.07)),
        origin=Origin(xyz=(-1.01, 0.0, 0.48)),
        material=gunmetal,
        name="rear_bustle_basket_rail",
    )
    for sgn, side in ((1.0, "left"), (-1.0, "right")):
        turret.visual(
            Box((0.34, 0.045, 0.29)),
            origin=Origin(xyz=(0.05, sgn * 0.715, 0.25)),
            material=camo_tan,
            name=f"{side}_turret_side_camo_plate",
        )
        turret.visual(
            Box((0.20, 0.05, 0.20)),
            origin=Origin(xyz=(-0.55, sgn * 0.70, 0.35)),
            material=gunmetal,
            name=f"{side}_bustle_grille",
        )
        turret.visual(
            Box((0.48, 0.08, 0.34)),
            origin=Origin(xyz=(0.31, sgn * 0.615, 0.27)),
            material=olive_dark,
            name=f"{side}_smoke_launcher_backing_plate",
        )
        for j, z in enumerate((0.18, 0.27, 0.36)):
            turret.visual(
                Cylinder(radius=0.038, length=0.34),
                origin=Origin(xyz=(0.32, sgn * 0.67, z), rpy=(0.0, pi / 2.0, 0.0)),
                material=gunmetal,
                name=f"{side}_smoke_launcher_{j}",
            )
            turret.visual(
                Box((0.08, 0.035, 0.095)),
                origin=Origin(xyz=(0.16, sgn * 0.635, z)),
                material=olive,
                name=f"{side}_smoke_launcher_rear_clamp_{j}",
            )
            turret.visual(
                Box((0.08, 0.035, 0.095)),
                origin=Origin(xyz=(0.48, sgn * 0.635, z)),
                material=olive,
                name=f"{side}_smoke_launcher_front_clamp_{j}",
            )
    turret.visual(mesh_from_cadquery(_hatch_ring(), "apc_hatch_ring"), material=olive_dark, name="hatch_ring")
    turret.visual(
        Cylinder(radius=0.18, length=0.035),
        origin=Origin(xyz=(-0.22, 0.18, 0.475)),
        material=olive_dark,
        name="hatch_lid",
    )
    turret.visual(
        Cylinder(radius=0.40, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.455)),
        material=olive,
        name="turret_roof_disc",
    )
    # Oversized mantlet and gun trunnion housing at the turret front.
    turret.visual(
        Box((0.62, 0.70, 0.42)),
        origin=Origin(xyz=(0.82, 0.0, 0.24)),
        material=olive,
        name="trunnion_housing",
    )
    turret.visual(
        Box((0.34, 0.76, 0.16)),
        origin=Origin(xyz=(0.95, 0.0, 0.41), rpy=(0.0, -0.18, 0.0)),
        material=olive_dark,
        name="upper_mantlet_brow",
    )
    turret.visual(
        Box((0.30, 0.70, 0.11)),
        origin=Origin(xyz=(0.98, 0.0, 0.04), rpy=(0.0, 0.16, 0.0)),
        material=olive_dark,
        name="lower_mantlet_lip",
    )
    turret.visual(
        Cylinder(radius=0.27, length=0.24),
        origin=Origin(xyz=(1.10, 0.0, 0.24), rpy=(0.0, pi / 2.0, 0.0)),
        material=olive_dark,
        name="mantlet_collaring_ring",
    )
    # Coaxial machine gun in a bracket outboard of the mantlet. The barrel is
    # held clear of the trunnion-housing wall so it stands off the turret face
    # instead of being buried in it.
    coax_y, coax_z = -0.45, 0.205
    # Mount bracket bolted to the front-right turret face, beside the mantlet.
    turret.visual(
        Box((0.20, 0.19, 0.18)),
        origin=Origin(xyz=(0.585, coax_y + 0.02, coax_z)),
        material=olive_dark,
        name="coax_mount_bracket",
    )
    # Cradle clamp capturing the gun jacket.
    turret.visual(
        Cylinder(radius=0.052, length=0.12),
        origin=Origin(xyz=(0.70, coax_y, coax_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=gunmetal,
        name="coax_cradle_clamp",
    )
    # Cooling jacket / receiver sleeve near the breech.
    turret.visual(
        Cylinder(radius=0.038, length=0.42),
        origin=Origin(xyz=(0.93, coax_y, coax_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=gunmetal,
        name="coax_barrel_jacket",
    )
    # Slim exposed barrel reaching forward, well clear of the turret wall.
    turret.visual(
        Cylinder(radius=0.019, length=0.62),
        origin=Origin(xyz=(1.35, coax_y, coax_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=gunmetal,
        name="coax_barrel",
    )
    # Flash hider at the muzzle.
    turret.visual(
        Cylinder(radius=0.032, length=0.11),
        origin=Origin(xyz=(1.64, coax_y, coax_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=gunmetal,
        name="coax_flash_hider",
    )
    # Compact ammo can clipped to the outboard side of the mount.
    turret.visual(
        Box((0.13, 0.11, 0.13)),
        origin=Origin(xyz=(0.73, coax_y - 0.10, coax_z - 0.015)),
        material=olive_dark,
        name="coax_ammo_can",
    )
    # Gunner sight and raised commander / AA machine-gun station.
    turret.visual(
        Box((0.24, 0.18, 0.13)),
        origin=Origin(xyz=(0.32, 0.33, 0.51)),
        material=olive_dark,
        name="gunner_sight",
    )
    turret.visual(
        Box((0.035, 0.12, 0.065)),
        origin=Origin(xyz=(0.45, 0.33, 0.51)),
        material=glass_dark,
        name="gunner_sight_glass",
    )
    # Brow hood shading the sight glass (small refinement).
    turret.visual(
        Box((0.10, 0.20, 0.028)),
        origin=Origin(xyz=(0.455, 0.33, 0.578), rpy=(0.0, -0.34, 0.0)),
        material=olive_dark,
        name="gunner_sight_hood",
    )
    turret.visual(
        Box((0.30, 0.24, 0.045)),
        origin=Origin(xyz=(0.32, 0.33, 0.455)),
        material=olive,
        name="gunner_sight_base_plate",
    )
    turret.visual(
        Cylinder(radius=0.13, length=0.055),
        origin=Origin(xyz=(-0.18, -0.38, 0.49)),
        material=olive_dark,
        name="aa_mg_mount_base",
    )
    turret.visual(
        Cylinder(radius=0.045, length=0.14),
        origin=Origin(xyz=(-0.18, -0.38, 0.55)),
        material=gunmetal,
        name="aa_mg_pintle",
    )
    turret.visual(
        Box((0.08, 0.035, 0.22)),
        origin=Origin(xyz=(-0.02, -0.315, 0.62)),
        material=gunmetal,
        name="aa_mg_left_yoke",
    )
    turret.visual(
        Box((0.08, 0.035, 0.22)),
        origin=Origin(xyz=(-0.02, -0.445, 0.62)),
        material=gunmetal,
        name="aa_mg_right_yoke",
    )
    turret.visual(
        Box((0.24, 0.11, 0.10)),
        origin=Origin(xyz=(0.02, -0.38, 0.64)),
        material=gunmetal,
        name="aa_mg_receiver",
    )
    turret.visual(
        Box((0.16, 0.11, 0.16)),
        origin=Origin(xyz=(-0.13, -0.52, 0.61)),
        material=olive_dark,
        name="aa_mg_ammo_box",
    )
    turret.visual(
        Box((0.12, 0.12, 0.055)),
        origin=Origin(xyz=(-0.06, -0.455, 0.63)),
        material=gunmetal,
        name="aa_mg_feed_bridge",
    )
    turret.visual(
        Cylinder(radius=0.018, length=1.00),
        origin=Origin(xyz=(0.42, -0.38, 0.70), rpy=(0.0, pi / 2.0 - 0.18, 0.0)),
        material=gunmetal,
        name="aa_mg_barrel",
    )

    model.articulation(
        "turret_traverse",
        ArticulationType.CONTINUOUS,
        parent=hull,
        child=turret,
        origin=Origin(xyz=(TURRET_X, 0.0, HULL_ROOF)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=600.0, velocity=1.5),
    )

    # ------------------------------------------------------------- gun
    gun = model.part("autocannon")
    gun.visual(
        Box((0.50, 0.30, 0.30)),
        origin=Origin(xyz=(-0.18, 0.0, 0.0)),
        material=gunmetal,
        name="breech_block",
    )
    gun.visual(
        Cylinder(radius=0.090, length=0.52),
        origin=Origin(xyz=(0.18, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=olive_dark,
        name="recoil_sleeve",
    )
    for sgn, side in ((1.0, "left"), (-1.0, "right")):
        gun.visual(
            Cylinder(radius=0.030, length=0.64),
            origin=Origin(xyz=(0.22, sgn * 0.115, -0.055), rpy=(0.0, pi / 2.0, 0.0)),
            material=gunmetal,
            name=f"{side}_recoil_rod",
        )
    gun.visual(
        Cylinder(radius=GUN_TUBE_R, length=GUN_TUBE_LEN),
        origin=Origin(xyz=(GUN_TUBE_LEN / 2.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=gunmetal,
        name="barrel_tube",
    )
    gun.visual(
        Cylinder(radius=0.080, length=1.05),
        origin=Origin(xyz=(0.70, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=olive_dark,
        name="thermal_sleeve",
    )
    gun.visual(
        Cylinder(radius=0.058, length=0.34),
        origin=Origin(xyz=(1.46, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=gunmetal,
        name="fume_extractor",
    )
    gun.visual(
        Cylinder(radius=0.092, length=0.40),
        origin=Origin(xyz=(GUN_TUBE_LEN + 0.14, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=gunmetal,
        name="muzzle_device",
    )
    gun.visual(
        Cylinder(radius=0.064, length=0.060),
        origin=Origin(xyz=(GUN_TUBE_LEN + 0.35, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=gunmetal,
        name="muzzle_end_cap",
    )
    for sgn, name in ((1.0, "left"), (-1.0, "right")):
        gun.visual(
            Box((0.15, 0.022, 0.070)),
            origin=Origin(xyz=(GUN_TUBE_LEN + 0.14, sgn * 0.098, 0.0)),
            material=glass_dark,
            name=f"{name}_muzzle_brake_port",
        )

    model.articulation(
        "gun_elevation",
        ArticulationType.REVOLUTE,
        parent=turret,
        child=gun,
        origin=Origin(xyz=GUN_TRUNNION),
        # -Y so positive q elevates the muzzle (barrel extends along +X).
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=300.0, velocity=0.8, lower=ELEV_LOWER, upper=ELEV_UPPER),
    )

    return model


# ----------------------------------------------------------------- tests
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hull = object_model.get_part("hull")
    turret = object_model.get_part("turret")
    gun = object_model.get_part("autocannon")
    traverse = object_model.get_articulation("turret_traverse")
    elevation = object_model.get_articulation("gun_elevation")

    # The gun cradle/barrel is intentionally seated through the trunnion
    # housing gun port; the breech recesses into the turret shell.
    ctx.allow_overlap(
        turret,
        gun,
        elem_a="trunnion_housing",
        elem_b="barrel_tube",
        reason="The barrel passes through the gun port in the trunnion housing.",
    )
    ctx.allow_overlap(
        turret,
        gun,
        elem_a="trunnion_housing",
        elem_b="breech_block",
        reason="The gun cradle/breech is captured inside the trunnion housing.",
    )
    # The breech, cradle and barrel are seated through the whole mantlet stack
    # (trunnion housing, collaring ring, upper brow, lower lip) and recess into
    # the turret shell -- all intentional embedments around the gun port.
    mantlet_elems = (
        "trunnion_housing",
        "mantlet_collaring_ring",
        "upper_mantlet_brow",
        "lower_mantlet_lip",
        "turret_shell",
    )
    gun_thru_elems = (
        "breech_block",
        "recoil_sleeve",
        "thermal_sleeve",
        "barrel_tube",
        "left_recoil_rod",
        "right_recoil_rod",
    )
    for m_elem in mantlet_elems:
        for g_elem in gun_thru_elems:
            ctx.allow_overlap(
                turret,
                gun,
                elem_a=m_elem,
                elem_b=g_elem,
                reason=f"The {g_elem} is seated through the {m_elem} at the gun port.",
            )
    # The turret base furniture (bearing race and the forward mantlet lip) seats
    # down onto the hull roof and overhangs the forward crew hatches at the
    # turret ring -- intentional embedment at the turret/hull interface.
    for base_elem in (
        "turret_shell",
        "turret_bearing_ring",
        "lower_mantlet_lip",
        "mantlet_collaring_ring",
    ):
        for roof_elem in ("hull_shell", "left_crew_hatch", "right_crew_hatch"):
            ctx.allow_overlap(
                hull,
                turret,
                elem_a=roof_elem,
                elem_b=base_elem,
                reason=f"The {base_elem} seats onto the {roof_elem} at the turret ring.",
            )
    # Each axle stub is intentionally captured inside its wheel hub.
    for side in ("left", "right"):
        for i in range(3):
            ctx.allow_overlap(
                hull,
                object_model.get_part(f"{side}_wheel_{i}"),
                elem_a=f"{side}_axle_stub_{i}",
                elem_b="rim",
                reason="The fixed axle stub is captured inside the spinning wheel hub.",
            )

    # ---- overall scale ----
    hull_bb = ctx.part_world_aabb(hull)
    ctx.check(
        "hull length ~7.6 m",
        hull_bb is not None and abs((hull_bb[1][0] - hull_bb[0][0]) - 7.6) < 0.15,
        details=f"hull aabb={hull_bb}",
    )
    ctx.check(
        "overall width ~2.9 m",
        hull_bb is not None and 2.75 <= (hull_bb[1][1] - hull_bb[0][1]) <= 3.05,
        details=f"hull aabb={hull_bb}",
    )
    turret_bb = ctx.part_element_world_aabb(turret, elem="turret_shell")
    ctx.check(
        "turret roof height ~2.6 m",
        turret_bb is not None and abs(turret_bb[1][2] - 2.60) < 0.05,
        details=f"turret shell aabb={turret_bb}",
    )

    # ---- wheels: count, grounding, diameter, joint type ----
    wheel_names = [f"{side}_wheel_{i}" for side in ("left", "right") for i in range(3)]
    ctx.check("six road wheels exist", len(wheel_names) == 6)
    for wn in wheel_names:
        wp = object_model.get_part(wn)
        bb = ctx.part_world_aabb(wp)
        ctx.check(
            f"{wn} grounded at z=0",
            bb is not None and abs(bb[0][2]) < 0.012,
            details=f"{wn} aabb={bb}",
        )
        ctx.check(
            f"{wn} diameter ~1.1 m",
            bb is not None and abs((bb[1][2] - bb[0][2]) - 1.10) < 0.05,
            details=f"{wn} aabb={bb}",
        )
        art = object_model.get_articulation(f"{wn}_spin")
        ctx.check(
            f"{wn} continuous lateral axle",
            art.articulation_type == ArticulationType.CONTINUOUS and abs(art.axis[1]) == 1.0,
            details=f"type={art.articulation_type}, axis={art.axis}",
        )

    # Fender strip clears the wheel top (mounted above, not intersecting).
    front_left_wheel = object_model.get_part("left_wheel_0")
    ctx.expect_gap(
        hull,
        front_left_wheel,
        axis="z",
        positive_elem="left_fender_strip",
        min_gap=0.005,
        max_gap=0.10,
        name="fender strip sits just above wheel",
    )
    # Axle stub retains insertion into the wheel hub.
    ctx.expect_overlap(
        hull,
        front_left_wheel,
        axes="y",
        elem_a="left_axle_stub_0",
        elem_b="rim",
        min_overlap=0.02,
        name="axle stub engages the wheel hub",
    )

    # ---- 6x6 layout: 3 evenly spaced axle pairs ----
    axle_positions = [
        ctx.part_world_position(object_model.get_part(f"left_wheel_{i}"))[0]
        for i in range(3)
    ]
    span = axle_positions[0] - axle_positions[2]
    half_span = span / 2.0
    ctx.check(
        "three axle pairs evenly spaced fore-aft",
        span > 4.0
        and abs(axle_positions[1] - (axle_positions[0] + axle_positions[2]) / 2.0) < 0.15,
        details=f"axle x positions={axle_positions}, span={span:.2f}",
    )

    # ---- turret traverse ----
    ctx.check(
        "turret traverse is continuous about Z",
        traverse.articulation_type == ArticulationType.CONTINUOUS and traverse.axis == (0.0, 0.0, 1.0),
        details=f"type={traverse.articulation_type}, axis={traverse.axis}",
    )
    gun_bb_rest = ctx.part_world_aabb(gun)
    with ctx.pose({traverse: pi / 2.0}):
        gun_bb_yaw = ctx.part_world_aabb(gun)
    ctx.check(
        "turret yaw swings the gun to +Y",
        gun_bb_rest is not None
        and gun_bb_yaw is not None
        and gun_bb_rest[1][0] > 3.0
        and gun_bb_yaw[1][1] > 2.5
        and gun_bb_yaw[1][0] < 1.5,
        details=f"rest={gun_bb_rest}, yawed={gun_bb_yaw}",
    )

    # ---- gun elevation ----
    ctx.check(
        "gun elevation range -5..+35 deg",
        elevation.articulation_type == ArticulationType.REVOLUTE
        and elevation.motion_limits is not None
        and abs(elevation.motion_limits.lower - ELEV_LOWER) < 1e-6
        and abs(elevation.motion_limits.upper - ELEV_UPPER) < 1e-6,
        details=f"limits={elevation.motion_limits}",
    )
    muzzle_rest = ctx.part_element_world_aabb(gun, elem="muzzle_device")
    with ctx.pose({elevation: ELEV_UPPER}):
        muzzle_up = ctx.part_element_world_aabb(gun, elem="muzzle_device")
    ctx.check(
        "positive elevation raises the muzzle",
        muzzle_rest is not None
        and muzzle_up is not None
        and (muzzle_up[1][2] - muzzle_rest[1][2]) > 1.0,
        details=f"rest={muzzle_rest}, elevated={muzzle_up}",
    )

    # ---- wheel spin proven by off-axis valve stem ----
    valve_rest = ctx.part_element_world_aabb(front_left_wheel, elem="valve_stem")
    spin = object_model.get_articulation("left_wheel_0_spin")
    with ctx.pose({spin: pi}):
        valve_half = ctx.part_element_world_aabb(front_left_wheel, elem="valve_stem")
    ctx.check(
        "wheel rotation carries the valve stem around the axle",
        valve_rest is not None
        and valve_half is not None
        and abs(valve_rest[0][0] - valve_half[0][0]) > 0.5,
        details=f"rest={valve_rest}, half-turn={valve_half}",
    )

    # ---- key placements ----
    marking_bb = ctx.part_element_world_aabb(hull, elem="tactical_marking_plate")
    ctx.check(
        "tactical marking proud of the right hull side",
        marking_bb is not None and marking_bb[0][1] < -1.42,
        details=f"marking aabb={marking_bb}",
    )
    coax_bb = ctx.part_element_world_aabb(turret, elem="coax_barrel")
    trunnion_bb = ctx.part_element_world_aabb(turret, elem="trunnion_housing")
    ctx.check(
        "coaxial MG barrel stands off outboard of the mantlet wall",
        coax_bb is not None
        and trunnion_bb is not None
        and coax_bb[1][1] < -0.15
        and coax_bb[1][1] < trunnion_bb[0][1],
        details=f"coax aabb={coax_bb}, trunnion aabb={trunnion_bb}",
    )
    # Turret base seats on the hull roof (overlap in plan, contact in z).
    ctx.expect_overlap(turret, hull, axes="xy", min_overlap=0.4, name="turret seated over hull roof")
    ctx.expect_gap(
        turret,
        hull,
        axis="z",
        positive_elem="turret_shell",
        negative_elem="hull_shell",
        max_gap=0.004,
        max_penetration=0.004,
        name="turret base flush on hull roof",
    )

    return ctx.report()


object_model = build_object_model()
