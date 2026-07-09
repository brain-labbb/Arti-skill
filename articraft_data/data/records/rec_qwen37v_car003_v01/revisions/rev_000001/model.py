from __future__ import annotations

# White Volkswagen Santana style three-box compact sedan (variant 01).
# Z-up world. Long axis along +Y (nose at +Y), width along X (driver/left at +X),
# up along +Z. Wheels touch z = 0. ~4.5 m long, ~1.7 m wide, ~1.43 m tall.
#
# Articulation:
#   - FOUR conventional passenger doors, each on a near-vertical hinge at its
#     FRONT edge, swinging OUTWARD (revolute about Z).
#   - Front hood and rear trunk lid each hinge UP (revolute about lateral X).
#   - Both front wheels STEER (revolute king-pin) and all four wheels SPIN.
# The cabin is HOLLOW: carved out so opening any door reveals the interior.
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TireCarcass,
    TireGeometry,
    TireSidewall,
    TireTread,
    TireShoulder,
    TorusGeometry,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    boolean_difference,
    mesh_from_geometry,
    superellipse_side_loft,
)

# ---------------------------------------------------------------- proportions
WHEEL_R = 0.30
WHEEL_W = 0.195
HALF_TRACK = 0.73
FRONT_AXLE_Y = 1.28
REAR_AXLE_Y = -1.27
BELT_Z = 0.85
FLANK_X = 0.85
AXLE_ROD_R = 0.040
AXLE_CHANNEL_R = 0.070

# Lower-body 3-box side rails: boxy upright Santana hull
LOWER_SECTIONS = [
    (2.22, 0.26, 0.80, 1.56),
    (2.04, 0.20, 0.82, 1.66),
    (1.74, 0.17, 0.83, 1.70),
    (1.28, 0.16, 0.84, 1.70),
    (0.92, 0.16, 0.85, 1.70),
    (0.30, 0.16, 0.85, 1.72),
    (-0.30, 0.16, 0.85, 1.72),
    (-0.92, 0.16, 0.85, 1.70),
    (-1.27, 0.16, 0.85, 1.70),
    (-1.74, 0.17, 0.86, 1.68),
    (-2.04, 0.20, 0.85, 1.62),
    (-2.22, 0.26, 0.82, 1.50),
]

# Greenhouse glasshouse (white shell) -- roof + A/B/C pillars + window frames
GREENHOUSE_SECTIONS = [
    (1.00, 0.80, 1.00, 1.66),
    (0.62, 0.80, 1.34, 1.64),
    (0.28, 0.80, 1.45, 1.62),
    (-0.34, 0.80, 1.46, 1.62),
    (-0.62, 0.80, 1.40, 1.62),
    (-1.00, 0.80, 1.10, 1.62),
]

# Cabin hollow + door apertures
CABIN_HALF_X = 0.70
CABIN_Y = (-0.96, 0.96)
CABIN_Z = (0.42, 0.90)
DOOR_APERTURE_X = (0.55, 1.02)
DOOR_APERTURE_Z = (0.44, 0.86)
DOOR_SPANS = (("front", 0.05, 0.89), ("rear", -0.89, -0.04))


def _save(name, geom):
    return mesh_from_geometry(geom, name)


def _box_cutter(x0, x1, y0, y1, z0, z1):
    box = BoxGeometry((x1 - x0, y1 - y0, z1 - z0)).translate(
        (x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0
    )
    return MeshGeometry(vertices=list(box.vertices), faces=[(f[0], f[2], f[1]) for f in box.faces])


def _raked_box_cutter(size, rx, cx, cy, cz):
    g = BoxGeometry(size).rotate_x(rx).translate(cx, cy, cz)
    return MeshGeometry(vertices=list(g.vertices), faces=[(f[0], f[2], f[1]) for f in g.faces])


def _drop_small_islands(geom, min_faces=8):
    faces = [tuple(f) for f in geom.faces]
    parent = list(range(len(faces)))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    edge_map = {}
    for fi, f in enumerate(faces):
        for a, b in ((f[0], f[1]), (f[1], f[2]), (f[2], f[0])):
            e = (a, b) if a < b else (b, a)
            if e in edge_map:
                ra, rb = find(fi), find(edge_map[e])
                if ra != rb:
                    parent[ra] = rb
            else:
                edge_map[e] = fi

    from collections import Counter

    sizes = Counter(find(fi) for fi in range(len(faces)))
    keep = {r for r, c in sizes.items() if c >= min_faces}
    new_faces = [faces[fi] for fi in range(len(faces)) if find(fi) in keep]
    return MeshGeometry(vertices=list(geom.vertices), faces=new_faces)


_LOWER_BODY_CACHE = None


def _lower_body_mesh():
    global _LOWER_BODY_CACHE
    if _LOWER_BODY_CACHE is None:
        body = superellipse_side_loft(LOWER_SECTIONS, exponents=4.2, segments=64)
        # Wheel wells
        for ax, ay in (
            (HALF_TRACK, FRONT_AXLE_Y),
            (-HALF_TRACK, FRONT_AXLE_Y),
            (HALF_TRACK, REAR_AXLE_Y),
            (-HALF_TRACK, REAR_AXLE_Y),
        ):
            sign = 1.0 if ax > 0 else -1.0
            well = (
                CylinderGeometry(radius=WHEEL_R + 0.04, height=0.56, radial_segments=32)
                .rotate_y(pi / 2.0)
                .translate(sign * 0.85, ay, WHEEL_R)
            )
            body = boolean_difference(body, well)
        # Door apertures + sill carves
        for sgn in (1.0, -1.0):
            xa, xb = sorted((sgn * DOOR_APERTURE_X[0], sgn * DOOR_APERTURE_X[1]))
            for _which, y0, y1 in DOOR_SPANS:
                body = boolean_difference(
                    body, _box_cutter(xa, xb, y0, y1, DOOR_APERTURE_Z[0], DOOR_APERTURE_Z[1])
                )
                body = boolean_difference(body, _box_cutter(xa, xb, y0, y1, 0.24, DOOR_APERTURE_Z[0]))
        # Engine bay + trunk cavities
        body = boolean_difference(body, _box_cutter(-0.48, 0.48, 0.98, 1.92, 0.46, 0.88))
        body = boolean_difference(body, _box_cutter(-0.48, 0.48, -1.92, -0.98, 0.46, 0.88))
        # Axle channels
        for ay in (FRONT_AXLE_Y, REAR_AXLE_Y):
            chan = (
                CylinderGeometry(radius=AXLE_CHANNEL_R, height=1.52, radial_segments=24)
                .rotate_y(pi / 2.0)
                .translate(0.0, ay, WHEEL_R)
            )
            body = boolean_difference(body, chan)
        # Hollow cabin (last to scrub stray facets)
        body = boolean_difference(
            body, _box_cutter(-CABIN_HALF_X, CABIN_HALF_X, CABIN_Y[0], CABIN_Y[1], CABIN_Z[0], CABIN_Z[1])
        )
        _LOWER_BODY_CACHE = body
    return _LOWER_BODY_CACHE.clone()


_GREENHOUSE_CACHE = None


def _greenhouse_parts():
    global _GREENHOUSE_CACHE
    if _GREENHOUSE_CACHE is None:
        t = 0.06
        outer = superellipse_side_loft(GREENHOUSE_SECTIONS, exponents=6.0, segments=72)
        inner_secs = [
            (y, zmin - 0.25, zmax - t, max(w - 2.0 * t, 0.04)) for (y, zmin, zmax, w) in GREENHOUSE_SECTIONS
        ]
        inner = superellipse_side_loft(inner_secs, exponents=6.0, segments=72)
        inner = MeshGeometry(vertices=list(inner.vertices), faces=[(f[0], f[2], f[1]) for f in inner.faces])
        shell = boolean_difference(outer, inner)
        windshield_box = _raked_box_cutter((1.00, 0.80, 0.46), -0.80, 0.0, 0.58, 1.09)
        rear_box = _raked_box_cutter((1.00, 0.76, 0.46), 0.66, 0.0, -0.73, 1.12)
        side_boxes = []
        for sgn in (1.0, -1.0):
            xa, xb = sorted((sgn * 0.62, sgn * 1.14))
            side_boxes.append(_box_cutter(xa, xb, 0.14, 0.70, 0.78, 1.16))
            side_boxes.append(_box_cutter(xa, xb, -0.70, -0.08, 0.78, 1.16))
        frame = shell
        for b in (windshield_box, rear_box, *side_boxes):
            frame = boolean_difference(frame, b)
        _GREENHOUSE_CACHE = _drop_small_islands(frame)
    return _GREENHOUSE_CACHE.clone()


# Modest 5-spoke steel wheel with hubcap look
_TIRE_GEOM = TireGeometry(
    WHEEL_R,
    WHEEL_W,
    inner_radius=0.20,
    carcass=TireCarcass(belt_width_ratio=0.74, sidewall_bulge=0.025),
    tread=TireTread(style="ribbed", depth=0.004, count=24, angle_deg=0.0, land_ratio=0.70),
    sidewall=TireSidewall(style="rounded", bulge=0.025),
    shoulder=TireShoulder(width=0.005, radius=0.003),
)
_WHEEL_GEOM = WheelGeometry(
    0.195,
    0.14,
    rim=WheelRim(inner_radius=0.155, flange_height=0.010, flange_thickness=0.005),
    hub=WheelHub(
        radius=0.045,
        width=0.07,
        cap_style="domed",
        bolt_pattern=BoltPattern(count=5, circle_diameter=0.060, hole_diameter=0.008),
    ),
    face=WheelFace(dish_depth=0.015, front_inset=0.005),
    spokes=WheelSpokes(style="straight", count=5, thickness=0.028, window_radius=0.042),
    bore=WheelBore(style="round", diameter=0.035),
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vw_santana_sedan")

    white = model.material("body_white", rgba=(0.93, 0.93, 0.94, 1.0))
    black_trim = model.material("black_trim", rgba=(0.05, 0.05, 0.055, 1.0))
    glass = model.material("glass_tint", rgba=(0.11, 0.12, 0.15, 0.66))
    chrome = model.material("chrome", rgba=(0.80, 0.81, 0.84, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    amber = model.material("amber", rgba=(0.86, 0.52, 0.07, 1.0))
    red_tail = model.material("tail_red", rgba=(0.62, 0.05, 0.06, 1.0))
    lens_clear = model.material("lens_clear", rgba=(0.88, 0.90, 0.92, 1.0))
    interior_grey = model.material("interior_grey", rgba=(0.22, 0.22, 0.24, 1.0))
    grille_dk = model.material("grille_dark", rgba=(0.08, 0.08, 0.09, 1.0))
    reflector = model.material("reflector", rgba=(0.90, 0.90, 0.88, 1.0))

    # ------------------------------------------------------------------ body
    body = model.part("body")
    body.visual(_save("lower_body.obj", _lower_body_mesh()), material=white, name="lower_body")
    body.visual(_save("greenhouse.obj", _greenhouse_parts()), material=white, name="roof")

    # Windshield - raked glass pane
    body.visual(
        Box((1.08, 0.90, 0.02)),
        origin=Origin(xyz=(0.0, 0.58, 1.09), rpy=(-0.80, 0.0, 0.0)),
        material=glass,
        name="windshield",
    )
    # Rear window - raked glass pane
    body.visual(
        Box((1.08, 0.72, 0.02)),
        origin=Origin(xyz=(0.0, -0.76, 1.10), rpy=(0.66, 0.0, 0.0)),
        material=glass,
        name="rear_window",
    )
    # Side windows - thin panes positioned INSIDE the greenhouse shell,
    # well inboard of door skins (door skins are at x=±0.85 ± 0.025)
    for sgn, side in ((1.0, "left"), (-1.0, "right")):
        body.visual(
            Box((0.02, 0.60, 0.36)),
            origin=Origin(xyz=(sgn * 0.79, 0.42, 1.02)),
            material=glass,
            name=f"side_window_front_{side}",
        )
        body.visual(
            Box((0.02, 0.66, 0.36)),
            origin=Origin(xyz=(sgn * 0.79, -0.38, 1.02)),
            material=glass,
            name=f"side_window_rear_{side}",
        )

    # ---- Interior ----
    # Floor pan caps door openings at bottom
    body.visual(
        Box((1.72, 1.96, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.41)),
        material=interior_grey,
        name="cabin_floor",
    )
    # Dashboard - narrower to avoid tire overlap
    body.visual(
        Box((1.10, 0.36, 0.24)),
        origin=Origin(xyz=(0.0, 0.86, 0.60)),
        material=interior_grey,
        name="dashboard",
    )
    for sx, side in ((0.34, "left"), (-0.34, "right")):
        body.visual(
            Box((0.46, 0.48, 0.18)),
            origin=Origin(xyz=(sx, 0.34, 0.55)),
            material=interior_grey,
            name=f"seat_base_front_{side}",
        )
        body.visual(
            Box((0.46, 0.10, 0.40)),
            origin=Origin(xyz=(sx, 0.12, 0.72)),
            material=interior_grey,
            name=f"seat_back_front_{side}",
        )
    body.visual(
        Box((1.20, 0.42, 0.18)),
        origin=Origin(xyz=(0.0, -0.55, 0.55)),
        material=interior_grey,
        name="seat_base_rear",
    )
    body.visual(
        Box((1.20, 0.10, 0.42)),
        origin=Origin(xyz=(0.0, -0.78, 0.73)),
        material=interior_grey,
        name="seat_back_rear",
    )
    body.visual(
        Box((1.34, 0.34, 0.06)),
        origin=Origin(xyz=(0.0, -0.84, 0.89)),
        material=interior_grey,
        name="parcel_shelf",
    )
    _SW_HUB = (0.34, 0.60, 0.74)
    _SW_RAKE = (0.395, 0.0, 0.0)
    # Column bridges from under-dashboard to the steering wheel hub
    body.visual(
        Cylinder(radius=0.022, length=0.30),
        origin=Origin(xyz=(0.34, 0.65, 0.62), rpy=_SW_RAKE),
        material=black_trim,
        name="steering_column",
    )

    # ---- Front fascia: sharper grille + detailed headlights ----
    NOSE_Y = 2.22

    # Grille: dark background with chrome horizontal bars
    body.visual(
        Box((0.62, 0.04, 0.12)),
        origin=Origin(xyz=(0.0, NOSE_Y + 0.003, 0.58)),
        material=grille_dk,
        name="grille",
    )
    # Chrome horizontal slats
    for k in range(4):
        body.visual(
            Box((0.60, 0.02, 0.012)),
            origin=Origin(xyz=(0.0, NOSE_Y + 0.025, 0.535 + 0.03 * k)),
            material=chrome,
            name=f"grille_slat_{k}",
        )
    # Chrome brow connecting headlights
    body.visual(
        Box((1.40, 0.03, 0.025)),
        origin=Origin(xyz=(0.0, NOSE_Y + 0.015, 0.65)),
        material=chrome,
        name="grille_brow",
    )

    # Headlight clusters: housing + reflector + lens + indicator
    for sx, side in ((0.56, "left"), (-0.56, "right")):
        # Dark housing recess
        body.visual(
            Box((0.38, 0.06, 0.20)),
            origin=Origin(xyz=(sx, NOSE_Y - 0.005, 0.62)),
            material=black_trim,
            name=f"headlight_housing_{side}",
        )
        # Reflector bowl
        body.visual(
            Box((0.30, 0.03, 0.14)),
            origin=Origin(xyz=(sx, NOSE_Y + 0.01, 0.64)),
            material=reflector,
            name=f"headlight_reflector_{side}",
        )
        # Clear lens proud of housing
        body.visual(
            Box((0.34, 0.02, 0.17)),
            origin=Origin(xyz=(sx, NOSE_Y + 0.03, 0.63)),
            material=lens_clear,
            name=f"headlight_{side}",
        )
        # Amber indicator strip at bottom
        body.visual(
            Box((0.30, 0.02, 0.028)),
            origin=Origin(xyz=(sx, NOSE_Y + 0.03, 0.555)),
            material=amber,
            name=f"front_indicator_{side}",
        )

    # Lower bumper
    body.visual(
        Box((1.60, 0.10, 0.22)),
        origin=Origin(xyz=(0.0, NOSE_Y + 0.0, 0.34)),
        material=white,
        name="front_bumper",
    )
    # Central intake
    body.visual(
        Box((0.90, 0.04, 0.12)),
        origin=Origin(xyz=(0.0, NOSE_Y + 0.04, 0.33)),
        material=grille_dk,
        name="front_intake",
    )
    # Fog lamps
    for sx, side in ((0.58, "left"), (-0.58, "right")):
        body.visual(
            Cylinder(radius=0.045, length=0.04),
            origin=Origin(xyz=(sx, NOSE_Y + 0.04, 0.32), rpy=(pi / 2.0, 0.0, 0.0)),
            material=lens_clear,
            name=f"fog_{side}",
        )

    # Rear bumper
    body.visual(
        Box((1.54, 0.10, 0.20)),
        origin=Origin(xyz=(0.0, -2.20, 0.40)),
        material=black_trim,
        name="rear_bumper",
    )
    # Taillights - embedded slightly into body rear face for connectivity
    for sx, side in ((0.55, "left"), (-0.55, "right")):
        body.visual(
            Box((0.30, 0.06, 0.16)),
            origin=Origin(xyz=(sx, -2.20, 0.60)),
            material=red_tail,
            name=f"taillight_{side}",
        )

    # Rocker / sill panels
    for sx, side in ((0.845, "left"), (-0.845, "right")):
        body.visual(
            Box((0.08, 1.90, 0.22)),
            origin=Origin(xyz=(sx, 0.0, 0.32)),
            material=black_trim,
            name=f"rocker_{side}",
        )

    # Axle rods - extend through bored channels, embed into body sides
    for ay, side in ((FRONT_AXLE_Y, "front"), (REAR_AXLE_Y, "rear")):
        body.visual(
            Cylinder(radius=AXLE_ROD_R, length=2.0 * HALF_TRACK + 0.14),
            origin=Origin(xyz=(0.0, ay, WHEEL_R), rpy=(0.0, pi / 2.0, 0.0)),
            material=chrome,
            name=f"{side}_axle_rod",
        )

    body.inertial = Inertial.from_geometry(
        Box((1.7, 4.5, 1.1)), mass=1150.0, origin=Origin(xyz=(0.0, 0.0, 0.6))
    )

    # ---------------------------------------------------------------- doors
    def make_door(side, which, hinge_y, rear_y):
        name = f"door_{which}_{side}"
        d = model.part(name)
        span = hinge_y - rear_y
        midy = -span / 2.0
        # Door panel - slightly inset for sharper panel gaps
        d.visual(
            Box((0.04, span - 0.06, 0.38)),
            origin=Origin(xyz=(0.0, midy, 0.09)),
            material=white,
            name="door_skin",
        )
        d.visual(
            Box((0.04, span - 0.06, 0.025)),
            origin=Origin(xyz=(0.0, midy, 0.275)),
            material=black_trim,
            name="door_beltline",
        )
        d.visual(
            Box((0.04, 0.10, 0.025)),
            origin=Origin(xyz=(0.005, -span + 0.16, 0.12)),
            material=chrome,
            name="door_handle",
        )
        d.inertial = Inertial.from_geometry(
            Box((0.06, span, 0.85)), mass=24.0, origin=Origin(xyz=(0.0, midy, 0.25))
        )
        return d

    door_specs = []
    for sgn, side in ((1.0, "left"), (-1.0, "right")):
        for which, y0, y1 in (("front", 0.05, 0.89), ("rear", -0.89, -0.04)):
            d = make_door(side, which, y1, y0)
            door_specs.append((d, sgn, y1))

    # ----------------------------------------------------------- hood + trunk
    hood = model.part("hood")
    hood.visual(
        Box((1.00, 0.92, 0.05)),
        origin=Origin(xyz=(0.0, 0.44, 0.02), rpy=(-0.06, 0.0, 0.0)),
        material=white,
        name="hood_skin",
    )
    hood.inertial = Inertial.from_geometry(Box((1.00, 0.92, 0.06)), mass=18.0, origin=Origin(xyz=(0.0, 0.44, 0.0)))

    trunk = model.part("trunk")
    trunk.visual(
        Box((0.92, 0.84, 0.04)),
        origin=Origin(xyz=(0.0, -0.44, -0.02), rpy=(0.10, 0.0, 0.0)),
        material=white,
        name="trunk_skin",
    )
    trunk.inertial = Inertial.from_geometry(Box((0.92, 0.84, 0.06)), mass=14.0, origin=Origin(xyz=(0.0, -0.44, 0.0)))

    # ---------------------------------------------------------------- wheels
    def make_wheel(name, outboard_sign):
        w = model.part(name)
        face_rpy = (0.0, 0.0, 0.0) if outboard_sign > 0 else (0.0, 0.0, pi)
        w.visual(_save(f"{name}_tire.obj", _TIRE_GEOM.clone()), origin=Origin(rpy=face_rpy), material=rubber, name="tire")
        w.visual(
            _save(f"{name}_wheel.obj", _WHEEL_GEOM.clone()),
            origin=Origin(xyz=(outboard_sign * 0.03, 0.0, 0.0), rpy=face_rpy),
            material=chrome,
            name="rim",
        )
        w.inertial = Inertial.from_geometry(
            Cylinder(radius=WHEEL_R, length=WHEEL_W), mass=18.0, origin=Origin(rpy=(0.0, pi / 2.0, 0.0))
        )
        return w

    make_wheel("wheel_front_left", 1.0)
    make_wheel("wheel_front_right", -1.0)
    make_wheel("wheel_rear_left", 1.0)
    make_wheel("wheel_rear_right", -1.0)

    def make_knuckle(name):
        k = model.part(name)
        k.inertial = Inertial.from_geometry(Box((0.10, 0.10, 0.20)), mass=5.0)
        return k

    knuckle_fl = make_knuckle("steer_knuckle_front_left")
    knuckle_fr = make_knuckle("steer_knuckle_front_right")

    # Steering wheel
    steer_wheel = model.part("steering_wheel")
    steer_wheel.visual(
        _save("sw_rim.obj", TorusGeometry(radius=0.14, tube=0.015, radial_segments=12, tubular_segments=44)),
        material=black_trim,
        name="sw_rim",
    )
    steer_wheel.visual(Box((0.26, 0.020, 0.016)), material=black_trim, name="sw_spoke_a")
    steer_wheel.visual(Box((0.020, 0.26, 0.016)), material=black_trim, name="sw_spoke_b")
    steer_wheel.visual(Cylinder(radius=0.032, length=0.04), material=chrome, name="sw_hub")
    steer_wheel.inertial = Inertial.from_geometry(Cylinder(radius=0.14, length=0.04), mass=1.5)

    # ----------------------------------------------------------- articulations
    for d, sgn, hinge_y in door_specs:
        axis = (0.0, 0.0, 1.0) if sgn > 0 else (0.0, 0.0, -1.0)
        model.articulation(
            f"{d.name}_hinge",
            ArticulationType.REVOLUTE,
            parent=body,
            child=d,
            origin=Origin(xyz=(sgn * FLANK_X, hinge_y, 0.55)),
            axis=axis,
            motion_limits=MotionLimits(effort=40.0, velocity=2.0, lower=0.0, upper=1.2),
        )

    model.articulation(
        "hood_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=hood,
        origin=Origin(xyz=(0.0, 0.98, 0.85)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=1.5, lower=0.0, upper=1.0),
    )
    model.articulation(
        "trunk_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=trunk,
        origin=Origin(xyz=(0.0, -0.98, 0.85)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=1.5, lower=0.0, upper=1.0),
    )

    STEER_LOCK = 0.42
    for knuckle, sx in ((knuckle_fl, HALF_TRACK), (knuckle_fr, -HALF_TRACK)):
        model.articulation(
            f"{knuckle.name}_steer",
            ArticulationType.REVOLUTE,
            parent=body,
            child=knuckle,
            origin=Origin(xyz=(sx, FRONT_AXLE_Y, WHEEL_R)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=100.0, velocity=4.0, lower=-STEER_LOCK, upper=STEER_LOCK),
        )

    for name, spin_parent, origin_xyz in (
        ("wheel_front_left", knuckle_fl, (0.0, 0.0, 0.0)),
        ("wheel_front_right", knuckle_fr, (0.0, 0.0, 0.0)),
        ("wheel_rear_left", body, (HALF_TRACK, REAR_AXLE_Y, WHEEL_R)),
        ("wheel_rear_right", body, (-HALF_TRACK, REAR_AXLE_Y, WHEEL_R)),
    ):
        model.articulation(
            f"{name}_spin",
            ArticulationType.CONTINUOUS,
            parent=spin_parent,
            child=name,
            origin=Origin(xyz=origin_xyz),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=160.0, velocity=60.0),
        )

    model.articulation(
        "steering_wheel_turn",
        ArticulationType.REVOLUTE,
        parent=body,
        child=steer_wheel,
        origin=Origin(xyz=_SW_HUB, rpy=_SW_RAKE),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=8.0, lower=-3.14, upper=3.14),
    )

    return model


object_model = build_object_model()


def run_tests():
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    doors = {
        f"door_{w}_{s}": object_model.get_part(f"door_{w}_{s}")
        for s in ("left", "right")
        for w in ("front", "rear")
    }
    wheels = {n: object_model.get_part(n) for n in (
        "wheel_front_left", "wheel_front_right", "wheel_rear_left", "wheel_rear_right")}

    # --- Intentional overlaps -------------------------------------------------
    # Wheels in wheel arches
    for n, w in wheels.items():
        for elem in ("tire", "rim"):
            ctx.allow_overlap(body, w, elem_a="lower_body", elem_b=elem,
                              reason="Wheel seated in the fender wheel arch of the body shell.")
            rod = "front_axle_rod" if "front" in n else "rear_axle_rod"
            ctx.allow_overlap(body, w, elem_a=rod, elem_b=elem,
                              reason="Axle rod end reaches into the wheel hub by design.")

    # Doors seat flush in body apertures
    for dn, d in doors.items():
        side = dn.split("_")[-1]
        for shell in ("lower_body", "roof", "windshield", f"rocker_{side}", "cabin_floor"):
            for delem in ("door_skin", "door_beltline"):
                ctx.allow_overlap(body, d, elem_a=shell, elem_b=delem,
                                  reason="Door seats flush in the body aperture; thin embed is intentional.")

    # Hood and trunk lids seat flush on body openings
    for part_name, elem in (("hood", "hood_skin"), ("trunk", "trunk_skin")):
        p = object_model.get_part(part_name)
        ctx.allow_overlap(body, p, elem_a="lower_body", elem_b=elem,
                          reason="Hood/trunk lid seats flush on the body opening.")
    # Trunk lid may contact the rear window at the closed-pose interface
    trunk = object_model.get_part("trunk")
    ctx.allow_overlap(body, trunk, elem_a="rear_window", elem_b="trunk_skin",
                      reason="Rear window glass meets trunk lid at the rear opening edge.")

    # Rear wheels may lightly contact the cabin floor pan at the wheel well edge
    for wn in ("wheel_rear_left", "wheel_rear_right"):
        w = object_model.get_part(wn)
        ctx.allow_overlap(body, w, elem_a="cabin_floor", elem_b="tire",
                          reason="Floor pan edge near rear wheel well; thin contact is intentional.")

    # Steering column reaches into the steering wheel hub for connectivity
    sw = object_model.get_part("steering_wheel")
    ctx.allow_overlap(body, sw, elem_a="steering_column", elem_b="sw_hub",
                      reason="Steering column shaft inserts into the wheel hub boss.")
    ctx.allow_overlap(body, sw, elem_a="steering_column", elem_b="sw_spoke_a",
                      reason="Steering column shaft passes through the spoke cross near the hub.")
    ctx.allow_overlap(body, sw, elem_a="steering_column", elem_b="sw_spoke_b",
                      reason="Steering column shaft passes through the spoke cross near the hub.")
    # Parcel shelf seals the cabin from the trunk; trunk lid front edge meets it
    ctx.allow_overlap(body, trunk, elem_a="parcel_shelf", elem_b="trunk_skin",
                      reason="Trunk lid front edge meets the parcel shelf at the cabin/trunk boundary.")

    # Taillights embedded into body rear face
    for side in ("left", "right"):
        ctx.allow_overlap(body, body, elem_a="lower_body", elem_b=f"taillight_{side}",
                          reason="Taillight lens embedded into rear body panel for connectivity.")

    # Axle rods pass through body channels
    for rod_name in ("front_axle_rod", "rear_axle_rod"):
        ctx.allow_overlap(body, body, elem_a="lower_body", elem_b=rod_name,
                          reason="Axle rod passes through bored channel in body shell.")

    # --- Hero features present ------------------------------------------------
    vis = {v.name for v in body.visuals}
    ctx.check("3-box body shell present", {"lower_body", "roof"} <= vis)
    ctx.check("greenhouse glass present (windshield + backlight)", {"windshield", "rear_window"} <= vis)
    ctx.check("grille + two headlights + two taillights",
              {"grille", "headlight_left", "headlight_right", "taillight_left", "taillight_right"} <= vis)
    ctx.check("interior present (floor, seats, dash, wheel)",
              {"cabin_floor", "dashboard", "seat_base_rear", "steering_column"} <= vis)
    sw = object_model.get_part("steering_wheel")
    ctx.check("rotating steering wheel present (rim + spokes + hub)",
              {"sw_rim", "sw_spoke_a", "sw_hub"} <= {v.name for v in sw.visuals})
    sw_turn = object_model.get_articulation("steering_wheel_turn")
    ctx.check("steering wheel TURNS (revolute about the column axis)",
              sw_turn.articulation_type == ArticulationType.REVOLUTE
              and tuple(sw_turn.axis) == (0.0, 0.0, 1.0))
    ctx.check("front + rear axle rods present", {"front_axle_rod", "rear_axle_rod"} <= vis)
    for side, ay in (("front", FRONT_AXLE_Y), ("rear", REAR_AXLE_Y)):
        rod = next(v for v in body.visuals if v.name == f"{side}_axle_rod")
        rz = rod.origin.xyz[2]
        ctx.check(f"{side} axle rod is at wheel-center height", abs(rz - WHEEL_R) < 1e-6,
                  details=f"rod z={rz:.3f}, wheel center z={WHEEL_R:.3f}")
    ctx.check("four doors exist",
              set(doors) == {"door_front_left", "door_rear_left", "door_front_right", "door_rear_right"})

    # --- Scale ----------------------------------------------------------------
    bb = ctx.part_world_aabb(body)
    assert bb is not None
    lo, hi = bb
    ctx.check("sedan length ~4.5 m", 4.3 <= hi[1] - lo[1] <= 4.7, details=f"L={hi[1] - lo[1]:.2f}")
    ctx.check("sedan width ~1.7 m", 1.6 <= hi[0] - lo[0] <= 1.85, details=f"W={hi[0] - lo[0]:.2f}")
    ctx.check("sedan height ~1.4 m", 1.3 <= hi[2] <= 1.55, details=f"H={hi[2]:.2f}")

    # --- White body, dark glass ----------------------------------------------
    mats = {m.name: m for m in object_model.materials}
    ctx.check("body is white/light", sum(mats["body_white"].rgba[:3]) > 2.5)
    ctx.check("glass darker than body",
              sum(mats["glass_tint"].rgba[:3]) < sum(mats["body_white"].rgba[:3]) - 1.5)

    # --- Doors: vertical hinge, swing OUTWARD, reveal the cabin ---------------
    for dn, d in doors.items():
        side = dn.split("_")[-1]
        hinge = object_model.get_articulation(f"{dn}_hinge")
        ax = tuple(hinge.axis)
        ctx.check(f"{dn} hinge is near-vertical (Z) revolute",
                  abs(ax[2]) > 0.9 and hinge.articulation_type == ArticulationType.REVOLUTE,
                  details=f"axis={ax}")
        rest = ctx.part_world_aabb(d)
        with ctx.pose({hinge: 1.0}):
            opened = ctx.part_world_aabb(d)
        assert rest is not None and opened is not None
        if side == "left":
            moved_out = opened[1][0] > rest[1][0] + 0.15
        else:
            moved_out = opened[0][0] < rest[0][0] - 0.15
        ctx.check(f"{dn} swings OUTWARD when opened", moved_out,
                  details=f"rest x=[{rest[0][0]:.2f},{rest[1][0]:.2f}] open x=[{opened[0][0]:.2f},{opened[1][0]:.2f}]")

    # --- Hood + trunk hinge open ---------------------------------------------
    for part_name, jn, q in (("hood", "hood_hinge", 0.85), ("trunk", "trunk_hinge", 0.85)):
        p = object_model.get_part(part_name)
        j = object_model.get_articulation(jn)
        ctx.check(f"{part_name} hinge is lateral (X) revolute",
                  abs(tuple(j.axis)[0]) > 0.9 and j.articulation_type == ArticulationType.REVOLUTE)
        rest = ctx.part_world_aabb(p)
        with ctx.pose({j: q}):
            lifted = ctx.part_world_aabb(p)
        assert rest is not None and lifted is not None
        ctx.check(f"{part_name} lifts up when opened", lifted[1][2] > rest[1][2] + 0.2,
                  details=f"rest top z={rest[1][2]:.2f}, open top z={lifted[1][2]:.2f}")

    # --- Wheels steer + spin, grounded ---------------------------------------
    for name, w in wheels.items():
        sp = object_model.get_articulation(f"{name}_spin")
        ctx.check(f"{name} spins about lateral X (continuous)",
                  tuple(sp.axis) == (1.0, 0.0, 0.0) and sp.articulation_type == ArticulationType.CONTINUOUS)
        wbb = ctx.part_world_aabb(w)
        assert wbb is not None
        ctx.check(f"{name} touches the ground", abs(wbb[0][2]) <= 0.03, details=f"min z={wbb[0][2]:.3f}")
    for side, kn in (("left", "steer_knuckle_front_left_steer"), ("right", "steer_knuckle_front_right_steer")):
        st = object_model.get_articulation(kn)
        ctx.check(f"front-{side} steering is vertical (Z) revolute",
                  abs(tuple(st.axis)[2]) > 0.9 and st.articulation_type == ArticulationType.REVOLUTE)

    return ctx.report()
