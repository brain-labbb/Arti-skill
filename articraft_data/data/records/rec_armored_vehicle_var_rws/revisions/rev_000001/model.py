from __future__ import annotations

"""Eight-wheeled armored personnel carrier (BTR-style APC) with RWS.

Reference image: picture/Military/Tank/001.png

- Faceted olive-drab welded hull, ~7.6 m long x ~2.9 m wide, roof ~2.2 m.
- Wedge nose with angled glacis plates, slab sides with tumblehome facets,
  fender strips over cut wheel arches, stowage boxes, grab rails, side door
  panels, red/white tactical marking.
- Small remote weapon station (RWS) cupola on the roof: compact armored
  sensor-and-gun pod with one slim machine gun.
- Articulations: continuous RWS pod yaw, revolute gun elevation (-5..+35 deg),
  8 continuous road wheels (~1.1 m diameter).

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
AXLE_X = (2.60, 1.25, -1.10, -2.45)
WHEEL_Y = 1.22
ARCH_RADIUS = 0.62

# Remote weapon station dimensions
RWS_X = 0.40  # center position on hull roof
RWS_BASE_R = 0.24  # pedestal ring radius
RWS_BASE_H = 0.10  # pedestal height
RWS_POD_R = 0.22  # pod body radius
RWS_POD_H = 0.26  # pod body height
RWS_POD_LEN = 0.42  # pod length (front-to-back)

GUN_TRUNNION = (0.22, 0.0, 0.14)  # in rws_pod frame
GUN_BARREL_LEN = 1.15
GUN_BARREL_R = 0.022

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


# --------------------------------------------------------------- RWS geometry
def _rws_pedestal() -> cq.Workplane:
    """Low armored pedestal ring bolted to the hull roof."""
    return (
        cq.Workplane("XY")
        .circle(RWS_BASE_R + 0.04)
        .circle(RWS_BASE_R - 0.02)
        .extrude(RWS_BASE_H)
    )


def _rws_pod_shell() -> cq.Workplane:
    """Compact armored pod body: rounded box with integrated cradle arm."""
    # Main body: rounded-corner box, starts slightly below z=0 so it overlaps
    # the pedestal ring when mounted.
    body = (
        cq.Workplane("XY")
        .workplane(offset=-0.01)
        .rect(RWS_POD_LEN, RWS_POD_R * 2.0)
        .extrude(RWS_POD_H + 0.01)
    )
    # Round the vertical edges to make it look armored
    body = body.edges("|Z").fillet(0.04)
    # Chamfer the top edges
    body = body.edges(">Z").chamfer(0.02)
    body = body.translate((-RWS_POD_LEN / 2.0, -RWS_POD_R, 0.0))

    # Integrated gun cradle arm extending forward from the pod front face.
    cradle_arm = (
        cq.Workplane("XY")
        .workplane(offset=0.03)
        .center(RWS_POD_LEN / 2.0 + 0.06, 0.0)
        .rect(0.18, 0.16)
        .extrude(0.18)
    )
    body = body.union(cradle_arm)
    return body


def _rws_sensor_window() -> cq.Workplane:
    """Sensor aperture cutout shape (slightly proud of pod surface)."""
    window = (
        cq.Workplane("XY")
        .rect(0.14, 0.10)
        .extrude(0.025)
    )
    return window.translate((-0.07, -0.05, RWS_POD_H - 0.005))


# ----------------------------------------------------------------- model
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="eight_wheeled_apc_rws")

    olive = model.material("olive_drab", rgba=(0.345, 0.375, 0.255, 1.0))
    olive_dark = model.material("olive_dark", rgba=(0.245, 0.270, 0.180, 1.0))
    hub_green = model.material("hub_green", rgba=(0.310, 0.355, 0.225, 1.0))
    rubber = model.material("tire_rubber", rgba=(0.115, 0.115, 0.125, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.160, 0.170, 0.180, 1.0))
    camo_tan = model.material("camo_tan", rgba=(0.620, 0.540, 0.380, 1.0))
    marking_white = model.material("marking_white", rgba=(0.900, 0.900, 0.880, 1.0))
    marking_red = model.material("marking_red", rgba=(0.720, 0.110, 0.100, 1.0))
    glass_dark = model.material("glass_dark", rgba=(0.080, 0.095, 0.110, 1.0))
    sensor_glass = model.material("sensor_glass", rgba=(0.060, 0.070, 0.120, 1.0))

    # ------------------------------------------------------------- hull
    hull = model.part("hull")
    hull.visual(mesh_from_cadquery(_build_hull_solid(), "apc_hull"), material=olive, name="hull_shell")

    # Fender strips above the wheel arches (overall width ~2.9 m).
    for sgn, nm in ((1.0, "left_fender_strip"), (-1.0, "right_fender_strip")):
        hull.visual(
            Box((5.70, 0.07, 0.06)),
            origin=Origin(xyz=(-0.25, sgn * 1.445, 1.16)),
            material=olive_dark,
            name=nm,
        )

    # Side door panels (upper + lower leaves) between axles 2 and 3.
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

    # RWS pedestal base ring (fixed to hull roof, slightly embedded for contact).
    hull.visual(
        mesh_from_cadquery(_rws_pedestal(), "apc_rws_pedestal"),
        origin=Origin(xyz=(RWS_X, 0.0, HULL_ROOF - 0.01)),
        material=olive_dark,
        name="rws_pedestal",
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

    # ------------------------------------------------------------- RWS pod
    rws_pod = model.part("rws_pod")
    rws_pod.visual(
        mesh_from_cadquery(_rws_pod_shell(), "apc_rws_pod_shell"),
        material=olive,
        name="pod_shell",
    )
    # Bearing race ring at the pod base (extends below z=0 to seat into pedestal)
    rws_pod.visual(
        Cylinder(radius=RWS_BASE_R, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, -0.04)),
        material=olive_dark,
        name="pod_bearing_ring",
    )
    # Sensor window (dark glass aperture on top of pod)
    rws_pod.visual(
        mesh_from_cadquery(_rws_sensor_window(), "apc_rws_sensor_window"),
        material=sensor_glass,
        name="sensor_window",
    )
    # Sensor housing bump on the top-rear of the pod
    rws_pod.visual(
        Box((0.16, 0.18, 0.06)),
        origin=Origin(xyz=(-0.08, 0.0, RWS_POD_H + 0.02)),
        material=olive_dark,
        name="sensor_housing",
    )
    # Day camera lens (small cylinder on front of sensor housing)
    rws_pod.visual(
        Cylinder(radius=0.025, length=0.04),
        origin=Origin(xyz=(0.02, 0.0, RWS_POD_H - 0.01), rpy=(0.0, pi / 2.0, 0.0)),
        material=glass_dark,
        name="day_camera",
    )
    # Thermal imager lens (offset to one side, embedded in pod shell)
    rws_pod.visual(
        Cylinder(radius=0.020, length=0.035),
        origin=Origin(xyz=(0.02, 0.04, RWS_POD_H - 0.06), rpy=(0.0, pi / 2.0, 0.0)),
        material=glass_dark,
        name="thermal_camera",
    )
    # Laser rangefinder window (small rectangle on the other side, embedded)
    rws_pod.visual(
        Box((0.03, 0.04, 0.03)),
        origin=Origin(xyz=(0.02, -0.04, RWS_POD_H - 0.06)),
        material=sensor_glass,
        name="lrf_window",
    )
    # Gun cradle mount arm is now integrated into the pod_shell CadQuery geometry.
    # Add cradle pivot bearing housings as separate visuals on the cradle arm.
    # Cradle pivot bearings (left and right, embedded in cradle mount)
    for sgn, side in ((1.0, "left"), (-1.0, "right")):
        rws_pod.visual(
            Cylinder(radius=0.040, length=0.06),
            origin=Origin(xyz=(RWS_POD_LEN / 2.0 + 0.08, sgn * 0.08, 0.14), rpy=(pi / 2.0, 0.0, 0.0)),
            material=gunmetal,
            name=f"{side}_cradle_bearing",
        )
    # Small camo tan patch on pod side (military marking, deeply embedded in shell)
    rws_pod.visual(
        Box((0.12, 0.03, 0.10)),
        origin=Origin(xyz=(0.0, RWS_POD_R - 0.02, 0.13)),
        material=camo_tan,
        name="pod_side_marking",
    )
    # Ammo box mounted on the side of the pod (partially embedded for contact)
    rws_pod.visual(
        Box((0.18, 0.14, 0.14)),
        origin=Origin(xyz=(-0.05, -(RWS_POD_R + 0.02), 0.10)),
        material=olive_dark,
        name="ammo_box",
    )
    # Ammo feed chute from box to gun (embedded in pod shell)
    rws_pod.visual(
        Box((0.10, 0.06, 0.05)),
        origin=Origin(xyz=(0.06, -(RWS_POD_R - 0.01), 0.14)),
        material=gunmetal,
        name="ammo_chute",
    )

    model.articulation(
        "rws_traverse",
        ArticulationType.CONTINUOUS,
        parent=hull,
        child=rws_pod,
        origin=Origin(xyz=(RWS_X, 0.0, HULL_ROOF + RWS_BASE_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=200.0, velocity=2.0),
    )

    # ------------------------------------------------------------- RWS gun
    rws_gun = model.part("rws_gun")
    # Receiver/action body
    rws_gun.visual(
        Box((0.26, 0.10, 0.10)),
        origin=Origin(xyz=(-0.05, 0.0, 0.0)),
        material=gunmetal,
        name="gun_receiver",
    )
    # Barrel jacket (cooling sleeve)
    rws_gun.visual(
        Cylinder(radius=0.032, length=0.40),
        origin=Origin(xyz=(0.28, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=gunmetal,
        name="barrel_jacket",
    )
    # Exposed barrel section
    rws_gun.visual(
        Cylinder(radius=GUN_BARREL_R, length=GUN_BARREL_LEN - 0.40),
        origin=Origin(xyz=(0.48 + (GUN_BARREL_LEN - 0.40) / 2.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=gunmetal,
        name="barrel_tube",
    )
    # Flash hider / muzzle brake
    rws_gun.visual(
        Cylinder(radius=0.035, length=0.10),
        origin=Origin(xyz=(0.48 + GUN_BARREL_LEN - 0.40 + 0.03, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=gunmetal,
        name="flash_hider",
    )
    # Carrying handle / heat shield on top of jacket
    rws_gun.visual(
        Box((0.22, 0.04, 0.03)),
        origin=Origin(xyz=(0.28, 0.0, 0.045)),
        material=olive_dark,
        name="carrying_handle",
    )
    # Front sight post
    rws_gun.visual(
        Box((0.02, 0.02, 0.05)),
        origin=Origin(xyz=(0.46, 0.0, 0.055)),
        material=gunmetal,
        name="front_sight",
    )

    model.articulation(
        "rws_gun_elevation",
        ArticulationType.REVOLUTE,
        parent=rws_pod,
        child=rws_gun,
        origin=Origin(xyz=GUN_TRUNNION),
        # -Y so positive q elevates the muzzle (barrel extends along +X).
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=100.0, velocity=1.0, lower=ELEV_LOWER, upper=ELEV_UPPER),
    )

    return model


# ----------------------------------------------------------------- tests
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hull = object_model.get_part("hull")
    rws_pod = object_model.get_part("rws_pod")
    rws_gun = object_model.get_part("rws_gun")
    traverse = object_model.get_articulation("rws_traverse")
    elevation = object_model.get_articulation("rws_gun_elevation")

    # The gun receiver/jacket is seated inside the pod gun cradle mount.
    ctx.allow_overlap(
        rws_pod,
        rws_gun,
        elem_a="pod_shell",
        elem_b="gun_receiver",
        reason="The gun receiver is captured in the pod cradle arm (integrated into pod_shell).",
    )
    ctx.allow_overlap(
        rws_pod,
        rws_gun,
        elem_a="pod_shell",
        elem_b="barrel_jacket",
        reason="The barrel jacket passes through the cradle arm opening (integrated into pod_shell).",
    )
    # The RWS pod bearing ring seats onto the hull pedestal.
    ctx.allow_overlap(
        hull,
        rws_pod,
        elem_a="rws_pedestal",
        elem_b="pod_bearing_ring",
        reason="The pod bearing ring seats inside the hull pedestal ring.",
    )
    # Each axle stub is intentionally captured inside its wheel hub.
    for side in ("left", "right"):
        for i in range(4):
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

    # RWS pod body is compact (the cradle arm extends forward to support the gun)
    pod_bb = ctx.part_world_aabb(rws_pod)
    ctx.check(
        "RWS pod exists on roof",
        pod_bb is not None and pod_bb[0][2] > 2.2,
        details=f"pod aabb={pod_bb}",
    )
    pod_dy = (pod_bb[1][1] - pod_bb[0][1]) if pod_bb else 0.0
    pod_dz = (pod_bb[1][2] - pod_bb[0][2]) if pod_bb else 0.0
    ctx.check(
        "RWS pod width and height are reasonable",
        pod_dy < 0.7 and pod_dz < 0.5,
        details=f"pod size y={pod_dy:.3f}, z={pod_dz:.3f}",
    )

    # ---- wheels: count, grounding, diameter, joint type ----
    wheel_names = [f"{side}_wheel_{i}" for side in ("left", "right") for i in range(4)]
    ctx.check("eight road wheels exist", len(wheel_names) == 8)
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

    # ---- RWS traverse ----
    ctx.check(
        "RWS traverse is continuous about Z",
        traverse.articulation_type == ArticulationType.CONTINUOUS and traverse.axis == (0.0, 0.0, 1.0),
        details=f"type={traverse.articulation_type}, axis={traverse.axis}",
    )
    gun_bb_rest = ctx.part_world_aabb(rws_gun)
    with ctx.pose({traverse: pi / 2.0}):
        gun_bb_yaw = ctx.part_world_aabb(rws_gun)
    ctx.check(
        "RWS yaw swings the gun to +Y",
        gun_bb_rest is not None
        and gun_bb_yaw is not None
        and gun_bb_rest[1][0] > 0.5
        and gun_bb_yaw[1][1] > 0.5
        and gun_bb_yaw[1][0] < 1.5,
        details=f"rest={gun_bb_rest}, yawed={gun_bb_yaw}",
    )

    # ---- gun elevation ----
    ctx.check(
        "RWS gun elevation range -5..+35 deg",
        elevation.articulation_type == ArticulationType.REVOLUTE
        and elevation.motion_limits is not None
        and abs(elevation.motion_limits.lower - ELEV_LOWER) < 1e-6
        and abs(elevation.motion_limits.upper - ELEV_UPPER) < 1e-6,
        details=f"limits={elevation.motion_limits}",
    )
    muzzle_rest = ctx.part_element_world_aabb(rws_gun, elem="flash_hider")
    with ctx.pose({elevation: ELEV_UPPER}):
        muzzle_up = ctx.part_element_world_aabb(rws_gun, elem="flash_hider")
    ctx.check(
        "positive elevation raises the muzzle",
        muzzle_rest is not None
        and muzzle_up is not None
        and (muzzle_up[1][2] - muzzle_rest[1][2]) > 0.3,
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
    # RWS pod seated on hull roof (overlap in plan, contact in z).
    ctx.expect_overlap(rws_pod, hull, axes="xy", min_overlap=0.1, name="RWS pod seated over hull roof")
    ctx.expect_gap(
        rws_pod,
        hull,
        axis="z",
        positive_elem="pod_shell",
        negative_elem="hull_shell",
        max_gap=0.14,
        max_penetration=0.005,
        name="RWS pod base sits on hull roof via pedestal",
    )

    # ---- RWS has sensor optics (not a plain turret) ----
    sensor_bb = ctx.part_element_world_aabb(rws_pod, elem="sensor_window")
    ctx.check(
        "RWS sensor window exists on pod",
        sensor_bb is not None and sensor_bb[0][2] > 2.3,
        details=f"sensor aabb={sensor_bb}",
    )

    # ---- No manned turret exists ----
    has_turret = True
    try:
        object_model.get_part("turret")
    except Exception:
        has_turret = False
    ctx.check(
        "no manned turret part exists",
        not has_turret,
    )

    return ctx.report()


object_model = build_object_model()
