from __future__ import annotations

"""Eight-wheeled armored personnel carrier (BTR-style APC) — open pintle variant.

Reference image: picture/Military/Tank/001.png

- Faceted olive-drab welded hull, ~7.6 m long x ~2.9 m wide, roof ~2.2 m.
- Wedge nose with angled glacis plates, slab sides with tumblehome facets,
  fender strips over cut wheel arches, stowage boxes, grab rails, side door
  panels, red/white tactical marking.
- Open troop deck roof with a pintle-mounted machine gun on a circular ring
  race over the forward crew hatch area.
- Four large rubber-tired road wheels per side (eight total, ~1.1 m diameter,
  dark gray with chunky tread and green hubs) mount low on the hull.
- Articulations: continuous pintle yaw about vertical, 8 continuous road wheels.

Frame: +X forward, +Y left, +Z up. Ground plane at z = 0.
"""

from math import pi

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
    TrunnionYokeGeometry,
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

# Pintle mount position: forward roof, centerline, over the crew hatch area.
PINTLE_X = 0.80
PINTLE_Z = HULL_ROOF
RING_OUTER_R = 0.40
RING_INNER_R = 0.28
RING_H = 0.050

# Pintle post and yoke
POST_R = 0.035
POST_H = 0.42
YOKE_SIZE = (0.20, 0.30, 0.22)  # (width_x, depth_y, height_z)
YOKE_SPAN = 0.14  # clear opening between cheeks (must be < overall_size[0])
YOKE_TRUNNION_D = 0.030
YOKE_TRUNNION_Z = 0.16  # trunnion center height from yoke base
YOKE_BASE_T = 0.020

# Machine gun dimensions (generic heavy MG like NSVT/M2HB)
MG_RECEIVER_LEN = 0.34
MG_RECEIVER_W = 0.12
MG_RECEIVER_H = 0.14
MG_BARREL_R = 0.022
MG_BARREL_LEN = 1.05
MG_JACKET_R = 0.038
MG_JACKET_LEN = 0.55


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


# ----------------------------------------------------------- pintle helpers
# All CQ shapes in pintle-local frame: origin at the pintle ring center,
# +X forward, +Z up. CQ extrudes start at local 0 and extend along +axis.

RING_TOP_Z = RING_H - 0.008  # rotating ring top surface
POST_TOP_Z = RING_TOP_Z + POST_H  # post top surface
PLATE_THICK = 0.018
YOKE_BASE_Z = POST_TOP_Z  # yoke base sits on post top
# Spider plate bridges ring inner wall to post (sits below ring top for overlap).
SPIDER_Z = RING_TOP_Z - 0.012
SPIDER_R = RING_INNER_R + 0.020  # overlaps ring inner wall at 0.295


def _ring_race() -> cq.Workplane:
    """Fixed ring race mounted on the hull roof."""
    return (
        cq.Workplane("XY")
        .circle(RING_OUTER_R)
        .circle(RING_INNER_R)
        .extrude(RING_H)
    )


def _rotating_ring() -> cq.Workplane:
    """Rotating ring that sits inside the race."""
    return (
        cq.Workplane("XY")
        .circle(RING_OUTER_R - 0.015)
        .circle(RING_INNER_R + 0.015)
        .extrude(RING_TOP_Z)
    )


def _spider_plate() -> cq.Workplane:
    """Solid disk bridging the rotating ring inner wall to the pintle post."""
    return (
        cq.Workplane("XY")
        .workplane(offset=SPIDER_Z)
        .circle(SPIDER_R)
        .extrude(0.020)
    )


def _pintle_post() -> cq.Workplane:
    """Vertical post rising from the spider plate."""
    return (
        cq.Workplane("XY")
        .workplane(offset=SPIDER_Z)
        .circle(POST_R)
        .extrude(POST_H + 0.012)  # extends from spider plate through ring top
    )


def _yoke_base_plate() -> cq.Workplane:
    """Horizontal plate connecting post top to yoke."""
    return (
        cq.Workplane("XY")
        .workplane(offset=POST_TOP_Z)
        .rect(0.12, 0.24)
        .extrude(PLATE_THICK)
    )


def _mg_receiver_body() -> cq.Workplane:
    """Machine gun receiver body (centered box)."""
    return cq.Workplane("XY").box(MG_RECEIVER_LEN, MG_RECEIVER_W, MG_RECEIVER_H)


def _mg_barrel_jacket() -> cq.Workplane:
    """Barrel jacket extending along +X from local origin."""
    return cq.Workplane("YZ").circle(MG_JACKET_R).extrude(MG_JACKET_LEN)


def _mg_barrel() -> cq.Workplane:
    """Exposed barrel extending along +X from local origin."""
    return cq.Workplane("YZ").circle(MG_BARREL_R).extrude(MG_BARREL_LEN)


def _mg_flash_hider() -> cq.Workplane:
    """Flash hider extending along +X from local origin."""
    return cq.Workplane("YZ").circle(0.035).extrude(0.10)


# ----------------------------------------------------------------- model
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="eight_wheeled_apc_pintle")

    olive = model.material("olive_drab", rgba=(0.345, 0.375, 0.255, 1.0))
    olive_dark = model.material("olive_dark", rgba=(0.245, 0.270, 0.180, 1.0))
    hub_green = model.material("hub_green", rgba=(0.310, 0.355, 0.225, 1.0))
    rubber = model.material("tire_rubber", rgba=(0.115, 0.115, 0.125, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.160, 0.170, 0.180, 1.0))
    marking_white = model.material("marking_white", rgba=(0.900, 0.900, 0.880, 1.0))
    marking_red = model.material("marking_red", rgba=(0.720, 0.110, 0.100, 1.0))
    glass_dark = model.material("glass_dark", rgba=(0.080, 0.095, 0.110, 1.0))

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

    # Fixed ring race on the hull roof over the forward crew hatch area.
    hull.visual(
        mesh_from_cadquery(_ring_race(), "pintle_ring_race"),
        origin=Origin(xyz=(PINTLE_X, 0.0, PINTLE_Z)),
        material=olive_dark,
        name="pintle_ring_race",
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

    # ------------------------------------------------------------- pintle mount
    pintle = model.part("pintle_mount")

    # Rotating ring that sits in the fixed race.
    pintle.visual(
        mesh_from_cadquery(_rotating_ring(), "pintle_rotating_ring"),
        material=olive_dark,
        name="rotating_ring",
    )

    # Spider plate: solid disk bridging the ring inner wall to the post.
    # This is the structural platform the post sits on.
    pintle.visual(
        mesh_from_cadquery(_spider_plate(), "pintle_spider_plate"),
        material=olive_dark,
        name="spider_plate",
    )

    # Vertical pintle post rising from the spider plate.
    pintle.visual(
        mesh_from_cadquery(_pintle_post(), "pintle_post"),
        material=olive_dark,
        name="pintle_post",
    )

    # Horizontal base plate connecting post top to the yoke.
    pintle.visual(
        mesh_from_cadquery(_yoke_base_plate(), "pintle_yoke_base"),
        material=olive_dark,
        name="yoke_base_plate",
    )

    # Trunnion yoke (two upright cheeks with trunnion bores), base on plate top.
    yoke_geom = TrunnionYokeGeometry(
        YOKE_SIZE,
        span_width=YOKE_SPAN,
        trunnion_diameter=YOKE_TRUNNION_D,
        trunnion_center_z=YOKE_TRUNNION_Z,
        base_thickness=YOKE_BASE_T,
        center=False,
    )
    pintle.visual(
        mesh_from_geometry(yoke_geom, "pintle_yoke"),
        origin=Origin(xyz=(0.0, 0.0, YOKE_BASE_Z + PLATE_THICK)),
        material=gunmetal,
        name="yoke",
    )

    # ---- Machine gun mounted in the yoke ----
    # Trunnion height in pintle frame (yoke base + trunnion offset).
    trunnion_z = YOKE_BASE_Z + PLATE_THICK + YOKE_TRUNNION_Z

    # MG receiver body (centered box), slightly forward of trunnion center.
    recv_cx = 0.04  # receiver center x in pintle frame
    pintle.visual(
        mesh_from_cadquery(_mg_receiver_body(), "mg_receiver"),
        origin=Origin(xyz=(recv_cx, 0.0, trunnion_z)),
        material=gunmetal,
        name="mg_receiver",
    )

    # Barrel chain: CQ extrudes start at local x=0 and extend along +X.
    # Each piece starts where the previous one ends.
    recv_front_x = recv_cx + MG_RECEIVER_LEN / 2.0  # receiver front face

    # Barrel jacket extending forward from receiver front face.
    pintle.visual(
        mesh_from_cadquery(_mg_barrel_jacket(), "mg_jacket"),
        origin=Origin(xyz=(recv_front_x, 0.0, trunnion_z)),
        material=olive_dark,
        name="mg_barrel_jacket",
    )
    jacket_end_x = recv_front_x + MG_JACKET_LEN

    # Exposed barrel extending forward from jacket end.
    pintle.visual(
        mesh_from_cadquery(_mg_barrel(), "mg_barrel"),
        origin=Origin(xyz=(jacket_end_x, 0.0, trunnion_z)),
        material=gunmetal,
        name="mg_barrel",
    )
    barrel_end_x = jacket_end_x + MG_BARREL_LEN

    # Flash hider at the muzzle.
    pintle.visual(
        mesh_from_cadquery(_mg_flash_hider(), "mg_flash_hider"),
        origin=Origin(xyz=(barrel_end_x, 0.0, trunnion_z)),
        material=gunmetal,
        name="mg_flash_hider",
    )

    # Rear backplate seated onto the receiver back face.
    recv_back_x = recv_cx - MG_RECEIVER_LEN / 2.0  # receiver back face
    pintle.visual(
        Box((0.03, MG_RECEIVER_W - 0.02, MG_RECEIVER_H - 0.02)),
        origin=Origin(xyz=(recv_back_x - 0.005, 0.0, trunnion_z)),
        material=gunmetal,
        name="mg_backplate",
    )

    # Spade grip handles at the rear of the receiver (horizontal along X,
    # extending into the receiver back face for a solid mount).
    for sgn, side in ((1.0, "left"), (-1.0, "right")):
        pintle.visual(
            Cylinder(radius=0.014, length=0.14),
            origin=Origin(
                xyz=(recv_back_x - 0.05, sgn * 0.05, trunnion_z - 0.03),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=gunmetal,
            name=f"mg_{side}_spade_grip",
        )

    # Ammo box on the left side of the receiver, bridging into the receiver wall.
    pintle.visual(
        Box((0.16, 0.14, 0.14)),
        origin=Origin(xyz=(-0.02, 0.10, trunnion_z)),
        material=olive_dark,
        name="mg_ammo_box",
    )

    # Ammo feed chute bridge between the ammo box and receiver top.
    pintle.visual(
        Box((0.08, 0.05, 0.06)),
        origin=Origin(xyz=(0.02, 0.065, trunnion_z + 0.06)),
        material=gunmetal,
        name="mg_feed_chute",
    )

    # Traverse handle / T-bar spanning across the ring for manual traverse.
    # Extends to the ring inner wall so the gunner can push/pull to rotate.
    handle_half = SPIDER_R + 0.010  # slightly past the spider plate edge
    handle_z = SPIDER_Z + 0.010  # just above the spider plate
    pintle.visual(
        Cylinder(radius=0.014, length=2.0 * handle_half),
        origin=Origin(xyz=(0.0, 0.0, handle_z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="traverse_handle",
    )
    # Rubber grips at the handle ends, just past the ring inner wall.
    for sgn in (1.0, -1.0):
        side = "left" if sgn > 0 else "right"
        pintle.visual(
            Cylinder(radius=0.020, length=0.06),
            origin=Origin(xyz=(0.0, sgn * handle_half, handle_z)),
            material=olive_dark,
            name=f"traverse_grip_{side}",
        )

    model.articulation(
        "pintle_traverse",
        ArticulationType.CONTINUOUS,
        parent=hull,
        child=pintle,
        origin=Origin(xyz=(PINTLE_X, 0.0, PINTLE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=200.0, velocity=1.5),
    )

    return model


# ----------------------------------------------------------------- tests
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hull = object_model.get_part("hull")
    pintle = object_model.get_part("pintle_mount")
    traverse = object_model.get_articulation("pintle_traverse")

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

    # The pintle rotating ring sits inside the fixed ring race on the hull.
    ctx.allow_overlap(
        hull,
        pintle,
        elem_a="pintle_ring_race",
        elem_b="rotating_ring",
        reason="The rotating ring sits inside the fixed ring race as a bearing interface.",
    )
    # The spider plate overlaps the ring race at the inner wall.
    ctx.allow_overlap(
        hull,
        pintle,
        elem_a="pintle_ring_race",
        elem_b="spider_plate",
        reason="The spider plate seats onto the ring race inner wall.",
    )
    # Traverse grips at the ring inner wall overlap the fixed ring race.
    for grip in ("traverse_grip_left", "traverse_grip_right"):
        ctx.allow_overlap(
            hull,
            pintle,
            elem_a="pintle_ring_race",
            elem_b=grip,
            reason=f"The {grip} sits at the ring race inner wall for the gunner to reach.",
        )
    # The traverse handle bar crosses through the ring race opening.
    ctx.allow_overlap(
        hull,
        pintle,
        elem_a="pintle_ring_race",
        elem_b="traverse_handle",
        reason="The traverse handle bar passes through the ring race center opening.",
    )

    # Spider plate bridges the ring inner wall to the post (structural platform).
    ctx.allow_overlap(
        pintle,
        pintle,
        elem_a="rotating_ring",
        elem_b="spider_plate",
        reason="The spider plate overlaps the rotating ring inner wall to bridge to the post.",
    )
    ctx.allow_overlap(
        pintle,
        pintle,
        elem_a="spider_plate",
        elem_b="pintle_post",
        reason="The pintle post rises from the spider plate.",
    )

    # The traverse handle sits on the spider plate.
    ctx.allow_overlap(
        pintle,
        pintle,
        elem_a="spider_plate",
        elem_b="traverse_handle",
        reason="The traverse handle bar rests on the spider plate.",
    )

    # MG receiver is cradled between the yoke cheeks.
    ctx.allow_overlap(
        pintle,
        pintle,
        elem_a="yoke",
        elem_b="mg_receiver",
        reason="The MG receiver is cradled between the yoke cheeks at the trunnion.",
    )

    # MG backplate seats into the receiver back face.
    ctx.allow_overlap(
        pintle,
        pintle,
        elem_a="mg_receiver",
        elem_b="mg_backplate",
        reason="The backplate is seated onto the receiver rear face.",
    )

    # Spade grips bridge into the receiver back.
    for grip_elem in ("mg_left_spade_grip", "mg_right_spade_grip"):
        ctx.allow_overlap(
            pintle,
            pintle,
            elem_a="mg_receiver",
            elem_b=grip_elem,
            reason=f"The {grip_elem} is anchored into the receiver back.",
        )

    # Ammo box and feed chute seat against the receiver side/top.
    ctx.allow_overlap(
        pintle,
        pintle,
        elem_a="mg_receiver",
        elem_b="mg_ammo_box",
        reason="The ammo box is bolted against the receiver left side.",
    )
    ctx.allow_overlap(
        pintle,
        pintle,
        elem_a="mg_receiver",
        elem_b="mg_feed_chute",
        reason="The feed chute bridges from the ammo box into the receiver top.",
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

    # No turret — roof height should be the hull roof (~2.2 m), not the old 2.6 m.
    ctx.check(
        "hull roof height ~2.2 m (no enclosed turret)",
        hull_bb is not None and 2.15 <= hull_bb[1][2] <= 2.35,
        details=f"hull aabb={hull_bb}",
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

    # Fender strip clears the wheel top.
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

    # ---- pintle traverse ----
    ctx.check(
        "pintle traverse is continuous about Z",
        traverse.articulation_type == ArticulationType.CONTINUOUS and traverse.axis == (0.0, 0.0, 1.0),
        details=f"type={traverse.articulation_type}, axis={traverse.axis}",
    )

    # Pintle ring race is on the hull roof in the forward area.
    race_bb = ctx.part_element_world_aabb(hull, elem="pintle_ring_race")
    ctx.check(
        "pintle ring race on hull roof forward area",
        race_bb is not None
        and race_bb[0][0] > 0.0
        and abs(race_bb[0][2] - PINTLE_Z) < 0.02,
        details=f"ring race aabb={race_bb}",
    )

    # The pintle mount sits on the hull roof.
    ctx.expect_overlap(pintle, hull, axes="xy", min_overlap=0.2, name="pintle seated over hull roof")
    ctx.expect_gap(
        pintle,
        hull,
        axis="z",
        positive_elem="rotating_ring",
        negative_elem="hull_shell",
        max_gap=0.06,
        max_penetration=0.005,
        name="pintle ring sits on hull roof",
    )

    # Pintle yaw swings the MG barrel around.
    barrel_rest = ctx.part_element_world_aabb(pintle, elem="mg_barrel")
    with ctx.pose({traverse: pi / 2.0}):
        barrel_yawed = ctx.part_element_world_aabb(pintle, elem="mg_barrel")
    ctx.check(
        "pintle yaw swings the barrel to +Y",
        barrel_rest is not None
        and barrel_yawed is not None
        and barrel_rest[1][0] > PINTLE_X + 0.5
        and barrel_yawed[1][1] > PINTLE_X + 0.3
        and barrel_yawed[1][0] < PINTLE_X + 0.3,
        details=f"rest={barrel_rest}, yawed={barrel_yawed}",
    )

    # MG barrel extends well forward of the pintle center.
    ctx.check(
        "MG barrel extends forward of the pintle",
        barrel_rest is not None and barrel_rest[1][0] > PINTLE_X + 1.0,
        details=f"barrel aabb={barrel_rest}",
    )

    # Yoke is visible above the post.
    yoke_bb = ctx.part_element_world_aabb(pintle, elem="yoke")
    post_bb = ctx.part_element_world_aabb(pintle, elem="pintle_post")
    ctx.check(
        "yoke sits above the pintle post",
        yoke_bb is not None
        and post_bb is not None
        and yoke_bb[0][2] > post_bb[0][2] + 0.3,
        details=f"yoke aabb={yoke_bb}, post aabb={post_bb}",
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

    return ctx.report()


object_model = build_object_model()
