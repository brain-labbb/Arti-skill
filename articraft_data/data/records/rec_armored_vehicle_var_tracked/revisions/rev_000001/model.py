from __future__ import annotations

"""Tracked armored personnel carrier (BMP-style tracked APC).

Fork of the 8-wheeled BTR-style APC converted to tracked running gear.

- Faceted olive-drab welded hull, ~7.6 m long x ~2.9 m wide, turret roof ~2.6 m.
- Wedge nose with angled glacis plates, slab sides with upper tumblehome facets,
  stowage boxes, grab rails, side door panels, red/white tactical marking.
- Two track assemblies (left/right) each with a continuous track band, five
  evenly spaced road-wheel bogies, a rear drive sprocket and a front idler.
- Low cylindrical (conic-frustum) turret with tan camo patches, hatch ring,
  long thin autocannon with muzzle device plus coaxial MG.
- Articulations: continuous turret yaw, revolute gun elevation (-5..+35 deg),
  10 continuous bogie road-wheel spin joints.

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
HULL_BOTTOM = 0.58
HULL_ROOF = 2.20

# Track running gear
N_BOGEYS = 5
BOGEY_R = 0.26          # road wheel radius (~0.52 m diameter)
BOGEY_W = 0.14           # road wheel width
BOGEY_CZ = 0.30          # bogey centre height (ground + radius + clearance)
BOGEY_XS = tuple(2.20 - i * 1.10 for i in range(N_BOGEYS))  # (2.20, 1.10, 0.00, -1.10, -2.20)

SPROCKET_R = 0.30        # drive sprocket radius
SPROCKET_X = -2.85       # rear position
SPROCKET_CZ = 0.55       # elevated rear

IDLER_R = 0.28           # front idler radius
IDLER_X = 2.85           # front position
IDLER_CZ = 0.42          # slightly elevated front

TRACK_W = 0.40           # track shoe width (Y)
TRACK_Y = 1.30           # track centre Y offset from hull centreline
TRACK_PAD = 0.03         # track shoe pad thickness

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

    # No wheel arches — running gear is tracked.
    return body


# ---------------------------------------------------------------- track band
def _build_track_band() -> cq.Workplane:
    """Track band as a thin extruded XZ loop, centred at y=0."""
    # Outer profile of the track loop (clockwise from front-bottom)
    outer = [
        (3.18, -0.02),
        (-3.20, -0.02),
        (-3.28, 0.22),
        (-3.18, 0.58),
        (-2.98, 0.85),
        (-2.55, 0.90),
        (2.45, 0.72),
        (2.88, 0.62),
        (3.16, 0.30),
    ]
    # Inner profile (~0.06 m inset)
    inner = [
        (3.10, 0.04),
        (-3.12, 0.04),
        (-3.18, 0.24),
        (-3.08, 0.55),
        (-2.90, 0.79),
        (-2.51, 0.84),
        (2.41, 0.66),
        (2.82, 0.57),
        (3.08, 0.28),
    ]
    outer_solid = cq.Workplane("XZ").polyline(outer).close().extrude(TRACK_W / 2.0, both=True)
    inner_void = cq.Workplane("XZ").polyline(inner).close().extrude(TRACK_W / 2.0 + 0.10, both=True)
    return outer_solid.cut(inner_void)


# ----------------------------------------------------------- sprocket/idler
def _build_sprocket() -> cq.Workplane:
    """Drive sprocket disc (face along Z, rotate for lateral mount)."""
    disc = cq.Workplane("XY").circle(SPROCKET_R).extrude(0.10)
    # Hub boss
    hub = cq.Workplane("XY").circle(0.08).extrude(0.14).translate((0, 0, -0.02))
    disc = disc.union(hub)
    # Lightening holes
    for i in range(5):
        a = 2.0 * pi * i / 5.0
        cx, cy = 0.16 * cos(a), 0.16 * sin(a)
        hole = cq.Workplane("XY").circle(0.045).extrude(0.12).translate((cx, cy, -0.01))
        disc = disc.cut(hole)
    # Tooth ring (raised ring on the outer edge suggesting sprocket teeth)
    ring = (
        cq.Workplane("XY")
        .circle(SPROCKET_R + 0.025)
        .circle(SPROCKET_R - 0.015)
        .extrude(0.10)
    )
    return disc.union(ring)


def _build_idler() -> cq.Workplane:
    """Idler disc (face along Z, rotate for lateral mount)."""
    disc = cq.Workplane("XY").circle(IDLER_R).extrude(0.08)
    hub = cq.Workplane("XY").circle(0.06).extrude(0.12).translate((0, 0, -0.02))
    disc = disc.union(hub)
    # Lightening holes
    for i in range(4):
        a = 2.0 * pi * i / 4.0
        cx, cy = 0.13 * cos(a), 0.13 * sin(a)
        hole = cq.Workplane("XY").circle(0.04).extrude(0.10).translate((cx, cy, -0.01))
        disc = disc.cut(hole)
    return disc


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
    model = ArticulatedObject(name="tracked_apc")

    olive = model.material("olive_drab", rgba=(0.345, 0.375, 0.255, 1.0))
    olive_dark = model.material("olive_dark", rgba=(0.245, 0.270, 0.180, 1.0))
    hub_green = model.material("hub_green", rgba=(0.310, 0.355, 0.225, 1.0))
    rubber = model.material("tire_rubber", rgba=(0.115, 0.115, 0.125, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.160, 0.170, 0.180, 1.0))
    camo_tan = model.material("camo_tan", rgba=(0.620, 0.540, 0.380, 1.0))
    marking_white = model.material("marking_white", rgba=(0.900, 0.900, 0.880, 1.0))
    marking_red = model.material("marking_red", rgba=(0.720, 0.110, 0.100, 1.0))
    glass_dark = model.material("glass_dark", rgba=(0.080, 0.095, 0.110, 1.0))
    track_steel = model.material("track_steel", rgba=(0.220, 0.220, 0.200, 1.0))

    # ------------------------------------------------------------- hull
    hull = model.part("hull")
    hull.visual(mesh_from_cadquery(_build_hull_solid(), "apc_hull"), material=olive, name="hull_shell")

    # Track skirt panels covering the upper track run on each side.
    for sgn, nm in ((1.0, "left_track_skirt"), (-1.0, "right_track_skirt")):
        hull.visual(
            Box((5.40, 0.025, 0.55)),
            origin=Origin(xyz=(0.0, sgn * 1.445, 0.88)),
            material=olive_dark,
            name=nm,
        )

    # Track guard rails along the top edge of each track skirt.
    for sgn, nm in ((1.0, "left_track_guard_rail"), (-1.0, "right_track_guard_rail")):
        hull.visual(
            Box((5.50, 0.04, 0.04)),
            origin=Origin(xyz=(0.0, sgn * 1.46, 1.10)),
            material=olive_dark,
            name=nm,
        )

    # Side door panels (upper + lower leaves) between bogies 2 and 3.
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

    # -------------------------------------------------------- track assemblies
    # Track bands, sprockets, idlers and return rollers are fixed hull visuals.
    track_band_mesh = mesh_from_cadquery(_build_track_band(), "track_band")
    sprocket_mesh = mesh_from_cadquery(_build_sprocket(), "drive_sprocket")
    idler_mesh = mesh_from_cadquery(_build_idler(), "idler_disc")

    for sgn, side in ((1.0, "left"), (-1.0, "right")):
        # Track band
        hull.visual(
            track_band_mesh,
            origin=Origin(xyz=(0.0, sgn * TRACK_Y, 0.0)),
            material=track_steel,
            name=f"{side}_track_band",
        )
        # Drive sprocket (rear) — disc oriented along Y via roll rotation
        hull.visual(
            sprocket_mesh,
            origin=Origin(xyz=(SPROCKET_X, sgn * TRACK_Y, SPROCKET_CZ), rpy=(pi / 2.0, 0.0, 0.0)),
            material=gunmetal,
            name=f"{side}_drive_sprocket",
        )
        # Idler disc (front)
        hull.visual(
            idler_mesh,
            origin=Origin(xyz=(IDLER_X, sgn * TRACK_Y, IDLER_CZ), rpy=(pi / 2.0, 0.0, 0.0)),
            material=gunmetal,
            name=f"{side}_idler_disc",
        )
        # Final drive housing bulge near sprocket
        hull.visual(
            Box((0.45, 0.18, 0.40)),
            origin=Origin(xyz=(SPROCKET_X + 0.30, sgn * 1.30, 0.82)),
            material=olive_dark,
            name=f"{side}_final_drive_housing",
        )
        # Track tensioner cylinder near idler
        hull.visual(
            Cylinder(radius=0.04, length=0.35),
            origin=Origin(xyz=(IDLER_X - 0.20, sgn * TRACK_Y, IDLER_CZ), rpy=(0.0, pi / 2.0, 0.0)),
            material=gunmetal,
            name=f"{side}_track_tensioner",
        )
        # Return rollers (3 per side, supporting the return track run)
        # Each has a bracket arm connecting it to the hull side.
        for ri, rx in enumerate((1.30, 0.0, -1.30)):
            rz = 0.66 + (SPROCKET_CZ - IDLER_CZ) * (1.30 - rx) / 5.10
            # Bracket arm from hull side out to the return roller.
            # Extends past the hull outer surface (y≈1.42) so it contacts
            # the hull shell mesh for connectivity.
            hull.visual(
                Box((0.08, 0.62, 0.06)),
                origin=Origin(xyz=(rx, sgn * (TRACK_Y - 0.18), rz)),
                material=olive_dark,
                name=f"{side}_return_roller_bracket_{ri}",
            )
            hull.visual(
                Cylinder(radius=0.06, length=0.12),
                origin=Origin(xyz=(rx, sgn * TRACK_Y, rz), rpy=(pi / 2.0, 0.0, 0.0)),
                material=gunmetal,
                name=f"{side}_return_roller_{ri}",
            )
        # Suspension arms from hull belly to each bogie axle.
        # Arms overlap with the hull shell at the mounting face (z >= 0.58).
        for idx, bx in enumerate(BOGEY_XS):
            hull.visual(
                Box((0.28, 0.06, 0.28)),
                origin=Origin(xyz=(bx + 0.08, sgn * 1.18, 0.48)),
                material=olive_dark,
                name=f"{side}_suspension_arm_{idx}",
            )
            # Bump stop above each bogie (overlaps hull shell at z=0.58)
            hull.visual(
                Cylinder(radius=0.035, length=0.08),
                origin=Origin(xyz=(bx, sgn * 1.16, 0.56)),
                material=rubber,
                name=f"{side}_bump_stop_{idx}",
            )

    # -------------------------------------------------------- bogie road wheels
    def _road_wheel_geom():
        return WheelGeometry(
            0.19,
            BOGEY_W - 0.02,
            rim=WheelRim(inner_radius=0.145, flange_height=0.014, flange_thickness=0.010, bead_seat_depth=0.004),
            hub=WheelHub(
                radius=0.060,
                width=BOGEY_W,
                cap_style="flat",
                bolt_pattern=BoltPattern(count=6, circle_diameter=0.085, hole_diameter=0.011),
            ),
            spokes=WheelSpokes(style="straight", count=5, thickness=0.018, window_radius=0.022),
            bore=WheelBore(style="round", diameter=0.028),
        )

    def _road_tire_geom():
        return TireGeometry(
            BOGEY_R,
            BOGEY_W,
            inner_radius=0.19,
            carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.03),
            tread=TireTread(style="block", depth=0.012, count=20, land_ratio=0.60),
            sidewall=TireSidewall(style="square", bulge=0.02),
            shoulder=TireShoulder(width=0.010, radius=0.004),
        )

    # Rotate wheel/tire geometry so spin axis is Y (lateral) instead of X.
    wheel_meshes = {
        "left": mesh_from_geometry(_road_wheel_geom().rotate_z(pi / 2.0), "road_wheel_left"),
        "right": mesh_from_geometry(_road_wheel_geom().rotate_z(-pi / 2.0), "road_wheel_right"),
    }
    tire_meshes = {
        "left": mesh_from_geometry(_road_tire_geom().rotate_z(pi / 2.0), "road_tire_left"),
        "right": mesh_from_geometry(_road_tire_geom().rotate_z(-pi / 2.0), "road_tire_right"),
    }

    for side_sgn, side in ((1.0, "left"), (-1.0, "right")):
        for i in range(N_BOGEYS):
            bp = model.part(f"{side}_bogie_{i}")
            bp.visual(tire_meshes[side], material=rubber, name="tire")
            bp.visual(wheel_meshes[side], material=hub_green, name="rim")
            # Off-axis marker on the wheel face (proves spin)
            bp.visual(
                Cylinder(radius=0.010, length=0.06),
                origin=Origin(xyz=(0.18, side_sgn * 0.06, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
                material=gunmetal,
                name="axle_marker",
            )
            model.articulation(
                f"{side}_bogie_{i}_spin",
                ArticulationType.CONTINUOUS,
                parent=hull,
                child=bp,
                origin=Origin(xyz=(BOGEY_XS[i], side_sgn * TRACK_Y, BOGEY_CZ)),
                axis=(0.0, 1.0, 0.0),
                motion_limits=MotionLimits(effort=400.0, velocity=25.0),
            )

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
    # Coaxial machine gun in a bracket outboard of the mantlet.
    coax_y, coax_z = -0.45, 0.205
    turret.visual(
        Box((0.20, 0.19, 0.18)),
        origin=Origin(xyz=(0.585, coax_y + 0.02, coax_z)),
        material=olive_dark,
        name="coax_mount_bracket",
    )
    turret.visual(
        Cylinder(radius=0.052, length=0.12),
        origin=Origin(xyz=(0.70, coax_y, coax_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=gunmetal,
        name="coax_cradle_clamp",
    )
    turret.visual(
        Cylinder(radius=0.038, length=0.42),
        origin=Origin(xyz=(0.93, coax_y, coax_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=gunmetal,
        name="coax_barrel_jacket",
    )
    turret.visual(
        Cylinder(radius=0.019, length=0.62),
        origin=Origin(xyz=(1.35, coax_y, coax_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=gunmetal,
        name="coax_barrel",
    )
    turret.visual(
        Cylinder(radius=0.032, length=0.11),
        origin=Origin(xyz=(1.64, coax_y, coax_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=gunmetal,
        name="coax_flash_hider",
    )
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
        turret, gun,
        elem_a="trunnion_housing", elem_b="barrel_tube",
        reason="The barrel passes through the gun port in the trunnion housing.",
    )
    ctx.allow_overlap(
        turret, gun,
        elem_a="trunnion_housing", elem_b="breech_block",
        reason="The gun cradle/breech is captured inside the trunnion housing.",
    )
    mantlet_elems = (
        "trunnion_housing", "mantlet_collaring_ring",
        "upper_mantlet_brow", "lower_mantlet_lip", "turret_shell",
    )
    gun_thru_elems = (
        "breech_block", "recoil_sleeve", "thermal_sleeve",
        "barrel_tube", "left_recoil_rod", "right_recoil_rod",
    )
    for m_elem in mantlet_elems:
        for g_elem in gun_thru_elems:
            ctx.allow_overlap(
                turret, gun,
                elem_a=m_elem, elem_b=g_elem,
                reason=f"The {g_elem} is seated through the {m_elem} at the gun port.",
            )
    for base_elem in (
        "turret_shell", "turret_bearing_ring",
        "lower_mantlet_lip", "mantlet_collaring_ring",
    ):
        for roof_elem in ("hull_shell", "left_crew_hatch", "right_crew_hatch"):
            ctx.allow_overlap(
                hull, turret,
                elem_a=roof_elem, elem_b=base_elem,
                reason=f"The {base_elem} seats onto the {roof_elem} at the turret ring.",
            )

    # Track band intentionally wraps around the bogie road wheels, sprockets
    # and idlers — the bogies ride inside the track loop.
    for side in ("left", "right"):
        sgn = 1.0 if side == "left" else -1.0
        for i in range(N_BOGEYS):
            bogie = object_model.get_part(f"{side}_bogie_{i}")
            ctx.allow_overlap(
                hull, bogie,
                elem_a=f"{side}_track_band", elem_b="tire",
                reason="The bogie road wheel rides inside the track band loop.",
            )
            ctx.allow_overlap(
                hull, bogie,
                elem_a=f"{side}_track_band", elem_b="rim",
                reason="The bogie road wheel hub rides inside the track band loop.",
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

    # ---- tracked running gear ----
    bogie_names = [f"{side}_bogie_{i}" for side in ("left", "right") for i in range(N_BOGEYS)]
    ctx.check("ten bogie road wheels exist", len(bogie_names) == 10)

    for bn in bogie_names:
        bp = object_model.get_part(bn)
        bb = ctx.part_world_aabb(bp)
        ctx.check(
            f"{bn} grounded near z=0",
            bb is not None and bb[0][2] < 0.08,
            details=f"{bn} aabb={bb}",
        )
        ctx.check(
            f"{bn} road wheel diameter ~0.52 m",
            bb is not None and abs((bb[1][2] - bb[0][2]) - 0.52) < 0.06,
            details=f"{bn} aabb={bb}",
        )
        art = object_model.get_articulation(f"{bn}_spin")
        ctx.check(
            f"{bn} continuous lateral axle",
            art.articulation_type == ArticulationType.CONTINUOUS and abs(art.axis[1]) == 1.0,
            details=f"type={art.articulation_type}, axis={art.axis}",
        )

    # Bogies evenly spaced along X on each side
    for side in ("left", "right"):
        xs = []
        for i in range(N_BOGEYS):
            pos = ctx.part_world_position(object_model.get_part(f"{side}_bogie_{i}"))
            if pos is not None:
                xs.append(pos[0])
        if len(xs) == N_BOGEYS:
            spacings = [abs(xs[i] - xs[i + 1]) for i in range(len(xs) - 1)]
            avg_spacing = sum(spacings) / len(spacings)
            ctx.check(
                f"{side} bogies evenly spaced (~1.1 m apart)",
                all(abs(s - avg_spacing) < 0.15 for s in spacings) and avg_spacing > 0.90,
                details=f"spacings={spacings}",
            )

    # Track bands present on both sides
    for side in ("left", "right"):
        tb_bb = ctx.part_element_world_aabb(hull, elem=f"{side}_track_band")
        ctx.check(
            f"{side} track band visible",
            tb_bb is not None and (tb_bb[1][0] - tb_bb[0][0]) > 5.5,
            details=f"{side}_track_band aabb={tb_bb}",
        )

    # Drive sprockets at rear, idlers at front
    for side in ("left", "right"):
        spr_bb = ctx.part_element_world_aabb(hull, elem=f"{side}_drive_sprocket")
        idl_bb = ctx.part_element_world_aabb(hull, elem=f"{side}_idler_disc")
        ctx.check(
            f"{side} sprocket behind idler",
            spr_bb is not None and idl_bb is not None and spr_bb[0][0] < idl_bb[0][0] - 4.0,
            details=f"sprocket={spr_bb}, idler={idl_bb}",
        )

    # Track skirt clears the bogie tops (skirt sits above road wheels)
    front_left_bogie = object_model.get_part("left_bogie_0")
    ctx.expect_gap(
        hull, front_left_bogie,
        axis="z",
        positive_elem="left_track_skirt",
        min_gap=0.001,
        max_gap=0.35,
        name="track skirt sits above bogie road wheels",
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

    # ---- bogie spin proven by off-axis marker ----
    marker_rest = ctx.part_element_world_aabb(front_left_bogie, elem="axle_marker")
    spin = object_model.get_articulation("left_bogie_0_spin")
    with ctx.pose({spin: pi}):
        marker_half = ctx.part_element_world_aabb(front_left_bogie, elem="axle_marker")
    ctx.check(
        "bogie rotation carries the axle marker around the axle",
        marker_rest is not None
        and marker_half is not None
        and abs(marker_rest[0][0] - marker_half[0][0]) > 0.30,
        details=f"rest={marker_rest}, half-turn={marker_half}",
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
    ctx.expect_overlap(turret, hull, axes="xy", min_overlap=0.4, name="turret seated over hull roof")
    ctx.expect_gap(
        turret, hull,
        axis="z",
        positive_elem="turret_shell",
        negative_elem="hull_shell",
        max_gap=0.004,
        max_penetration=0.004,
        name="turret base flush on hull roof",
    )

    return ctx.report()


object_model = build_object_model()
