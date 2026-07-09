from __future__ import annotations

# Ferrari F40 (1987) style mid-engine supercar, Rosso Corsa red.
# Z-up world. Long axis of the car runs along +Y (nose at +Y), width along X
# (driver/left side at +X), up along +Z. Wheels touch z = 0.
#
# Forked from the Diablo wedge supercar: the proven STRUCTURE is reused -- the
# cabin is hollowed out of the solid body by boolean_difference (cabin cavity +
# door apertures cut clean THROUGH the flanks), each front wheel hangs off a
# steering knuckle so it both STEERS about a vertical king-pin AND spins off the
# knuckle, rear wheels spin off the body, and straight axle rods run hub-to-hub
# through bored channels. The body is re-skinned from the Diablo wedge into the
# F40's low, sharp, pointier wedge, and given the F40's defining details: a low
# full-width slatted nose intake + round driving lights, pop-up-style headlight
# blisters flush on the wing tops, NACA-duct flank scoops ahead of the rear
# wheels, a LARGE fixed rear wing on twin end-plate pylons, a black louvered
# engine cover over the deck, four round taillights (two per side), dual central
# round exhausts, and a yellow Ferrari shield on the nose.
#
# Primary articulation: BOTH doors swing UP-and-forward on revolute hinges about
# a lateral-dominant axis at each door's front-top edge (the F40 has conventional
# front-hinged doors, but the inherited scissor-ish upward swing is kept as the
# demo articulation). Secondary: all four wheels spin (continuous about lateral
# X); the two FRONT wheels additionally steer about a vertical king-pin.
# >>> USER_CODE_START
from math import pi, sqrt

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
    TireCarcass,
    TireGeometry,
    TireSidewall,
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

# ----------------------------------------------------------------------------
# Global proportions (meters). Real F40: ~4.36 L x 1.98 W x 1.13 H,
# wheelbase 2.45, wheel radius ~0.33. We hold the inherited ~4.46 m envelope so
# the cabin/door/wheel structure stays valid, but skin it as the F40 wedge.
# ----------------------------------------------------------------------------
WHEEL_R = 0.33
WHEEL_W = 0.305
HALF_TRACK = 0.89
FRONT_AXLE_Y = 1.32
REAR_AXLE_Y = -1.33

# Wheel-arch cavities are carved out of the solid lower body at each wheel so
# the wheels sit in tight open wells. Each cutter is a lateral (X) cylinder
# hugging the wheel that only opens the OUTBOARD flank.
ARCH_RADIUS = 0.36
# (x_center_sign*track, y_center, inboard_wall_abs_x, outboard_wall_abs_x)
WHEEL_ARCHES = (
    (HALF_TRACK, FRONT_AXLE_Y, 0.58, 1.16),
    (-HALF_TRACK, FRONT_AXLE_Y, 0.58, 1.16),
    (HALF_TRACK, REAR_AXLE_Y, 0.70, 1.18),
    (-HALF_TRACK, REAR_AXLE_Y, 0.70, 1.18),
)

# Straight axle rods run hub-to-hub at wheel-center height. A transverse channel
# is bored clean through the lower body on each axle line.
AXLE_BAR_RADIUS = 0.05
AXLE_CHANNEL_RADIUS = 0.085

# The cabin is hollowed out of the solid body so the seats + steering wheel sit
# in real open space, and the two door apertures are cut clean THROUGH the
# flanks so opening a door reveals the cabin, not a solid mesh cross-section.
CABIN_HALF_X = 0.60
CABIN_Y = (-0.80, 0.88)
CABIN_Z = (0.50, 0.78)  # hollow up toward the low glass base so seats are open
DOOR_APERTURE_X = (0.52, 1.05)  # inboard (into cabin) .. outboard (through flank)
DOOR_APERTURE_Y = (-0.26, 0.80)
# Door-opening bottom raised to the CABIN FLOOR level (0.50) so the door sill is
# FLUSH with the floor.
DOOR_APERTURE_Z = (0.50, 0.74)

# Door hinge point (left side; right side mirrors x), body frame. Front-hinged
# doors that swing up-and-forward (inherited lateral-dominant scissor axis).
DOOR_HINGE = (0.88, 0.84, 0.70)
_AX_TILT = 0.25
_AX_NORM = sqrt(1.0 + _AX_TILT * _AX_TILT)
DOOR_AXIS_LEFT = (-1.0 / _AX_NORM, 0.0, _AX_TILT / _AX_NORM)
DOOR_AXIS_RIGHT = (-1.0 / _AX_NORM, 0.0, -_AX_TILT / _AX_NORM)
DOOR_OPEN_MAX = 1.4

# Lower wedge body side-profile rails: (y, z_min, z_max, width).
# F40: VERY low pointy nose, a long flat-ish wedge hood, a low cab, then a high
# muscular rear deck + Kamm tail. Lower and pointier at the nose than the Diablo.
LOWER_SECTIONS = [
    (2.28, 0.13, 0.27, 0.84),   # sharp low pointed nose tip
    (2.12, 0.11, 0.34, 1.34),   # nose blends out fast (low full-width intake area)
    (1.86, 0.11, 0.56, 1.78),   # front wing shoulder rising
    (1.60, 0.11, 0.74, 1.94),   # front fender crown (drapes over the wheel)
    (1.32, 0.11, 0.78, 1.98),   # over front axle -- fender caps the well
    (1.10, 0.12, 0.74, 1.96),   # hood dips just behind the fender crown
    (0.88, 0.12, 0.68, 1.94),   # windshield base
    (0.40, 0.13, 0.66, 1.88),   # door front / low beltline
    (-0.12, 0.13, 0.67, 1.88),  # door mid
    (-0.62, 0.12, 0.70, 1.98),  # rear haunch swelling
    (-1.08, 0.11, 0.74, 2.04),  # muscular rear haunch (widest), deck flatter
    (-1.45, 0.11, 0.75, 2.04),  # over rear axle
    (-1.86, 0.12, 0.74, 1.94),  # flat rear deck
    (-2.12, 0.16, 0.72, 1.76),
    (-2.26, 0.20, 0.70, 1.58),  # flat Kamm tail
]

# Cab-forward greenhouse, built as THREE thin shells that share seam sections so
# the windshield / roof / rear window meet exactly (seamless). Front + rear are
# tinted glass; the middle is the body-color roof. The F40 has a fairly upright
# windshield and a low fastback roof flowing into the engine deck.
# F40 greenhouse: LOW, fairly flat roof, then a long shallow fastback that drops
# gently to the engine deck (not a tall round bubble). Roof crown held near
# ~1.03 m and nearly level so the canopy reads flat, not a dome.
_SEAM_FRONT = (0.34, 0.66, 1.00, 1.14)  # windshield top == roof leading edge
_SEAM_REAR = (-0.66, 0.66, 0.90, 1.02)  # roof trailing edge == rear window top
WINDSHIELD_SECTIONS = [
    (0.94, 0.64, 0.70, 1.40),
    (0.64, 0.65, 0.86, 1.28),
    _SEAM_FRONT,
]
ROOF_SECTIONS = [
    _SEAM_FRONT,
    (0.02, 0.66, 1.02, 1.10),   # low nearly-level roof over the seats
    (-0.36, 0.66, 1.00, 1.06),
    _SEAM_REAR,
]
REAR_WINDOW_SECTIONS = [
    _SEAM_REAR,
    (-0.96, 0.66, 0.76, 1.02),  # long shallow rear screen into the deck
]
GLASS_SHELL_T = 0.016


def _save(name: str, geom):
    return mesh_from_geometry(geom, name)


def _box_cutter(x0, x1, y0, y1, z0, z1):
    # Axis-aligned box for boolean carving. BoxGeometry ships inward-wound
    # (negative volume) -> manifold3d reads it as empty and the subtraction is a
    # silent no-op; flip the winding so it is a real solid cutter.
    box = BoxGeometry((x1 - x0, y1 - y0, z1 - z0)).translate(
        (x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0
    )
    return MeshGeometry(
        vertices=list(box.vertices),
        faces=[(f[0], f[2], f[1]) for f in box.faces],
    )


_LOWER_BODY_CACHE = None


def _lower_body_mesh():
    # High exponent -> flat angular panels with hard shoulders (sharp F40 wedge,
    # not a blob). Then subtract a lateral cylinder at each wheel to carve open
    # wheel arches, bore axle channels, hollow the cabin + door apertures, and
    # carve a shallow flush blister pocket into each front wing for the pop-up
    # headlight. Cached because the boolean carve is reused by export + QC.
    global _LOWER_BODY_CACHE
    if _LOWER_BODY_CACHE is None:
        body = superellipse_side_loft(LOWER_SECTIONS, exponents=4.0, segments=64)
        for ax, ay, inboard, outboard in WHEEL_ARCHES:
            sign = 1.0 if ax > 0 else -1.0
            arch = (
                CylinderGeometry(
                    radius=ARCH_RADIUS, height=outboard - inboard, radial_segments=32
                )
                .rotate_y(pi / 2.0)  # long axis Z -> X (lateral wheel-arch tunnel)
                .translate(sign * (inboard + outboard) / 2.0, ay, WHEEL_R)
            )
            body = boolean_difference(body, arch)
        # Bore a straight transverse channel through each axle line.
        for ay in (FRONT_AXLE_Y, REAR_AXLE_Y):
            channel = (
                CylinderGeometry(
                    radius=AXLE_CHANNEL_RADIUS, height=2.0 * HALF_TRACK + 0.2, radial_segments=24
                )
                .rotate_y(pi / 2.0)
                .translate(0.0, ay, WHEEL_R)
            )
            body = boolean_difference(body, channel)
        # Hollow the cabin, then cut a door aperture clean through each flank.
        body = boolean_difference(
            body,
            _box_cutter(-CABIN_HALF_X, CABIN_HALF_X, CABIN_Y[0], CABIN_Y[1], CABIN_Z[0], CABIN_Z[1]),
        )
        for sgn in (1.0, -1.0):
            xa, xb = sorted((sgn * DOOR_APERTURE_X[0], sgn * DOOR_APERTURE_X[1]))
            body = boolean_difference(
                body,
                _box_cutter(xa, xb, DOOR_APERTURE_Y[0], DOOR_APERTURE_Y[1], DOOR_APERTURE_Z[0], DOOR_APERTURE_Z[1]),
            )
        # Carve a shallow flush pocket into each front-wing top so the F40
        # pop-up headlight blister sits flush/recessed (lamps down), not proud.
        # Raked -0.18 about X to follow the gentle wing slope.
        for hx_sign in (1.0, -1.0):
            g = (
                BoxGeometry((0.42, 0.30, 0.10))
                .rotate_x(-0.18)
                .translate(hx_sign * 0.56, 1.84, 0.53)
            )
            pocket = MeshGeometry(
                vertices=list(g.vertices),
                faces=[(f[0], f[2], f[1]) for f in g.faces],
            )
            body = boolean_difference(body, pocket)
        _LOWER_BODY_CACHE = body
    return _LOWER_BODY_CACHE.clone()


def _glass_shell(sections, t=GLASS_SHELL_T):
    # Thin uniform-thickness shell of a side-loft: subtract an inset copy (top/
    # sides pulled in by t, bottom dropped below so the underside opens into the
    # cabin). Adjacent shells share seam rails so they meet seamlessly.
    outer = superellipse_side_loft(sections, exponents=2.8, segments=56)
    inner = superellipse_side_loft(
        [(y, zmin - 0.15, zmax - t, max(w - 2.0 * t, 0.04)) for (y, zmin, zmax, w) in sections],
        exponents=2.8,
        segments=56,
    )
    inner = MeshGeometry(
        vertices=list(inner.vertices),
        faces=[(f[0], f[2], f[1]) for f in inner.faces],
    )
    return boolean_difference(outer, inner)


# Shared wheel/tire geometry: black tire + multi-spoke (F40 5x2 = 10-look)
# light alloy. The real F40 wears multi-spoke modular wheels; we read that with
# a many-thin-spoke silver alloy turbine.
_TIRE_GEOM = TireGeometry(
    WHEEL_R,
    WHEEL_W,
    inner_radius=0.215,
    carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.04),
    sidewall=TireSidewall(style="rounded", bulge=0.04),
)
_WHEEL_GEOM = WheelGeometry(
    0.220,
    0.205,
    rim=WheelRim(inner_radius=0.184, flange_height=0.012, flange_thickness=0.006),
    hub=WheelHub(
        radius=0.062,
        width=0.09,
        cap_style="domed",
        bolt_pattern=BoltPattern(count=5, circle_diameter=0.076, hole_diameter=0.009),
    ),
    face=WheelFace(dish_depth=0.018, front_inset=0.006, window_depth=0.028),
    # Many thin radial spokes -> the F40's busy multi-spoke modular wheel look.
    spokes=WheelSpokes(style="straight", count=10, thickness=0.026, window_radius=0.048),
    bore=WheelBore(style="round", diameter=0.04),
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ferrari_f40")

    rosso = model.material("rosso_corsa", rgba=(0.78, 0.04, 0.04, 1.0))  # Ferrari red
    black_trim = model.material("black_trim", rgba=(0.05, 0.05, 0.055, 1.0))
    model.material("glass_dark", rgba=(0.08, 0.09, 0.11, 1.0))  # used by QC by name
    glass_tint = model.material("glass_tint", rgba=(0.12, 0.13, 0.15, 0.34))
    silver = model.material("silver_alloy", rgba=(0.76, 0.77, 0.80, 1.0))
    rubber = model.material("rubber", rgba=(0.04, 0.04, 0.045, 1.0))
    amber = model.material("amber", rgba=(0.85, 0.50, 0.06, 1.0))
    red_tail = model.material("tail_red", rgba=(0.62, 0.03, 0.04, 1.0))
    lens_pale = model.material("lens_pale", rgba=(0.82, 0.85, 0.88, 1.0))
    interior_dk = model.material("interior_dark", rgba=(0.10, 0.10, 0.11, 1.0))
    axle_steel = model.material("axle_steel", rgba=(0.66, 0.67, 0.70, 1.0))
    chrome = model.material("chrome", rgba=(0.80, 0.81, 0.84, 1.0))
    ferrari_yellow = model.material("ferrari_yellow", rgba=(0.95, 0.80, 0.05, 1.0))

    # ------------------------------------------------------------------ body
    body = model.part("body")
    body.visual(_save("lower_body.obj", _lower_body_mesh()), material=rosso, name="lower_body")
    body.visual(_save("roof.obj", _glass_shell(ROOF_SECTIONS)), material=rosso, name="greenhouse")
    body.visual(
        _save("windshield.obj", _glass_shell(WINDSHIELD_SECTIONS)),
        material=glass_tint,
        name="windshield",
    )
    body.visual(
        _save("rear_window.obj", _glass_shell(REAR_WINDOW_SECTIONS)),
        material=glass_tint,
        name="rear_window",
    )

    # Cabin interior (floor + spartan seat backs). The F40 is famously bare.
    body.visual(
        Box((1.10, 1.30, 0.10)),
        origin=Origin(xyz=(0.0, 0.05, 0.47)),
        material=interior_dk,
        name="cabin_floor",
    )
    for sx, side in ((0.32, "left"), (-0.32, "right")):
        body.visual(
            Box((0.42, 0.18, 0.30)),
            origin=Origin(xyz=(sx, -0.42, 0.62)),
            material=interior_dk,
            name=f"seat_back_{side}",
        )

    # Steering wheel: raked column fixed on the body; the round wheel is a
    # SEPARATE part that SPINS about the raked column axis.
    _SW_X = 0.34
    _SW_FWD = 0.40
    _SW_RAKE = (0.6, 0.0, 0.0)
    _SW_HUB = (_SW_X, -0.02 + _SW_FWD, 0.69)
    body.visual(
        Cylinder(radius=0.022, length=0.34),
        origin=Origin(xyz=(_SW_X, 0.08 + _SW_FWD, 0.55), rpy=_SW_RAKE),
        material=black_trim,
        name="steering_column",
    )
    # ---- Round 3-spoke steering wheel (separate part, spins on the column) ----
    sw_accent = model.material("sw_accent", rgba=(0.85, 0.06, 0.06, 1.0))
    steer_wheel = model.part("steering_wheel")
    _RR = 0.150  # rim radius
    _RT = 0.026  # rim bar cross-section
    _RD = 0.045  # rim depth along the column axis (Z)
    # Round rim made of a ring of short arc segments (a circle of small boxes).
    _N_RIM = 16
    import math as _m
    for _k in range(_N_RIM):
        _a = 2.0 * _m.pi * _k / _N_RIM
        _rx = _RR * _m.cos(_a)
        _ry = _RR * _m.sin(_a)
        steer_wheel.visual(
            Box((0.064, _RT, _RD)),
            origin=Origin(xyz=(_rx, _ry, 0.0), rpy=(0.0, 0.0, _a + _m.pi / 2.0)),
            material=black_trim,
            name=f"sw_rim_top" if _k == 0 else (f"sw_rim_bot" if _k == _N_RIM // 2 else f"sw_rim_{_k}"),
        )
    # Three spokes (classic Ferrari 3-spoke).
    for _ang, _tag in ((0.0, "left"), (2.0 * _m.pi / 3.0, "up"), (-2.0 * _m.pi / 3.0, "right")):
        _sx = (_RR * 0.55) * _m.cos(_ang)
        _sy = (_RR * 0.55) * _m.sin(_ang)
        steer_wheel.visual(
            Box((0.026, _RR, 0.020)),
            origin=Origin(xyz=(_sx, _sy, 0.0), rpy=(0.0, 0.0, _ang + _m.pi / 2.0)),
            material=black_trim,
            name=f"sw_grip_{_tag}" if _tag in ("left", "right") else f"sw_spoke_{_tag}",
        )
    steer_wheel.visual(
        Cylinder(radius=0.048, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, -0.02)),
        material=black_trim,
        name="sw_hub",
    )
    steer_wheel.visual(
        Box((0.150, 0.080, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=interior_dk,
        name="sw_display",
    )
    steer_wheel.visual(
        Box((0.030, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        material=ferrari_yellow,
        name="sw_prancing_horse",
    )
    # Bright top-centre alignment marker (off the spin axis -> sweeps round).
    steer_wheel.visual(
        Box((0.034, 0.020, 0.020)),
        origin=Origin(xyz=(0.0, _RR, 0.020)),
        material=sw_accent,
        name="sw_top_marker",
    )
    steer_wheel.inertial = Inertial.from_geometry(Box((0.31, 0.31, 0.06)), mass=2.0)
    model.articulation(
        "steering_wheel_turn",
        ArticulationType.REVOLUTE,
        parent=body,
        child=steer_wheel,
        origin=Origin(xyz=_SW_HUB, rpy=_SW_RAKE),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=8.0, lower=-3.14, upper=3.14),
    )

    # --- Front: low full-width slatted intake + round driving lights -----------
    # Black front splitter under the nose.
    body.visual(
        Box((1.40, 0.40, 0.07)),
        origin=Origin(xyz=(0.0, 2.06, 0.10)),
        material=black_trim,
        name="front_splitter",
    )
    # Low full-width slatted intake mouth across the nose (the F40 signature
    # front grille opening): a dark recessed mouth + a row of horizontal slats.
    body.visual(
        Box((1.14, 0.08, 0.18)),
        origin=Origin(xyz=(0.0, 2.14, 0.27)),
        material=black_trim,
        name="front_intake",
    )
    for k in range(4):
        body.visual(
            Box((1.10, 0.05, 0.018)),
            origin=Origin(xyz=(0.0, 2.165, 0.215 + 0.038 * k)),
            material=black_trim,
            name=f"front_intake_slat_{k}",
        )
    # Yellow Ferrari shield badge pressed into the nose sheetmetal, above the
    # intake. The nose top at x~0 sits around z~0.33 (y=2.12..2.14), so the badge
    # is seated low and its rear half is buried in the body -> connected, flush.
    body.visual(
        Box((0.075, 0.07, 0.11)),
        origin=Origin(xyz=(0.0, 2.125, 0.345)),
        material=ferrari_yellow,
        name="ferrari_badge",
    )
    # Black prancing-horse plate proud of the shield face (its rear edge is buried
    # in the yellow shield, so it stays part of the connected nose piece).
    body.visual(
        Box((0.040, 0.045, 0.060)),
        origin=Origin(xyz=(0.0, 2.150, 0.345)),
        material=black_trim,
        name="badge_horse",
    )
    # Small round driving lights low in the intake corners.
    for sx, side in ((0.42, "left"), (-0.42, "right")):
        body.visual(
            Cylinder(radius=0.045, length=0.05),
            origin=Origin(xyz=(sx, 2.165, 0.26), rpy=(pi / 2.0, 0.0, 0.0)),
            material=lens_pale,
            name=f"driving_light_{side}",
        )
    # Amber turn indicators outboard of the driving lights.
    for sx, side in ((0.60, "left"), (-0.60, "right")):
        body.visual(
            Box((0.12, 0.05, 0.06)),
            origin=Origin(xyz=(sx, 2.15, 0.30)),
            material=amber,
            name=f"front_indicator_{side}",
        )

    # --- Pop-up-style headlight blisters flush on the wing tops ----------------
    # The F40 has rounded-rectangular pop-up headlamps that, when down, sit FLUSH
    # as low blisters on the front wing tops. Built as a low body-color blister
    # lid with a thin lamp lens peeking at its leading edge, set into the carved
    # wing pocket. Raked -0.18 about X to lie on the wing slope.
    _HL_RAKE = (-0.18, 0.0, 0.0)
    for sx, side in ((0.56, "left"), (-0.56, "right")):
        # Body-color blister lid (the closed pop-up panel), flush with the wing.
        body.visual(
            Box((0.40, 0.27, 0.045)),
            origin=Origin(xyz=(sx, 1.84, 0.565), rpy=_HL_RAKE),
            material=rosso,
            name=f"headlight_blister_{side}",
        )
        # Thin dark lamp lens peeking at the blister's leading (front) edge.
        body.visual(
            Box((0.36, 0.06, 0.045)),
            origin=Origin(xyz=(sx, 1.975, 0.583), rpy=_HL_RAKE),
            material=glass_tint,
            name=f"headlight_{side}",
        )
        # Pale reflector behind the lens (reads as the lamp through the glaze).
        body.visual(
            Box((0.34, 0.035, 0.032)),
            origin=Origin(xyz=(sx, 1.972, 0.588), rpy=_HL_RAKE),
            material=lens_pale,
            name=f"headlight_reflector_{side}",
        )

    # Black rocker sills between the wheel arches.
    for sx, side in ((0.835, "left"), (-0.835, "right")):
        body.visual(
            Box((0.12, 1.72, 0.13)),
            origin=Origin(xyz=(sx, 0.10, 0.175)),
            material=black_trim,
            name=f"rocker_sill_{side}",
        )

    # Front and rear axle rods, hub to hub THROUGH the bored channels.
    for ay, axle_name in ((FRONT_AXLE_Y, "front_axle_bar"), (REAR_AXLE_Y, "rear_axle_bar")):
        body.visual(
            Cylinder(radius=AXLE_BAR_RADIUS, length=2.0 * HALF_TRACK),
            origin=Origin(xyz=(0.0, ay, WHEEL_R), rpy=(0.0, pi / 2.0, 0.0)),
            material=axle_steel,
            name=axle_name,
        )

    # --- Large NACA duct flank scoops ahead of the rear wheels -----------------
    # The F40 signature NACA duct: a flush triangular scoop let into the rear
    # flank that DEEPENS toward the rear (the duct floor ramps down to a throat
    # at the back). Built as a dark recessed throat plus a body-color ramp wall
    # so it reads as a let-in NACA duct, not a bolt-on box.
    for sx, side in ((0.92, "left"), (-0.92, "right")):
        # Dark recessed duct throat (widening, deepening toward the rear).
        body.visual(
            Box((0.10, 0.46, 0.16)),
            origin=Origin(xyz=(sx, -0.72, 0.52), rpy=(0.0, 0.10, 0.0)),
            material=black_trim,
            name=f"naca_duct_{side}",
        )
        # Body-color ramp lip along the duct's lower/leading edge (the NACA ramp).
        body.visual(
            Box((0.085, 0.50, 0.035)),
            origin=Origin(xyz=(sx * 1.01, -0.70, 0.44)),
            material=rosso,
            name=f"naca_ramp_{side}",
        )

    # --- Black louvered engine cover over the deck -----------------------------
    # The F40's defining rear deck: a black slatted/louvered engine cover (a
    # grille of slats over the engine, showing the V8 beneath).
    body.visual(
        Box((1.34, 0.86, 0.045)),
        origin=Origin(xyz=(0.0, -1.30, 0.755)),
        material=black_trim,
        name="engine_deck",
    )
    for k in range(8):
        body.visual(
            Box((1.28, 0.072, 0.03)),
            origin=Origin(xyz=(0.0, -0.96 - 0.095 * k, 0.79), rpy=(0.18, 0.0, 0.0)),
            material=black_trim,
            name=f"deck_louver_{k}",
        )
    # Two raised body-color buttress strakes flanking the louvers (the F40's
    # engine-cover side rails that frame the louvered panel).
    for sx, side in ((0.62, "left"), (-0.62, "right")):
        body.visual(
            Box((0.10, 0.86, 0.06)),
            origin=Origin(xyz=(sx, -1.30, 0.785)),
            material=rosso,
            name=f"deck_rail_{side}",
        )
    # Crisp body-color frame lips closing the front and rear edges of the engine-
    # cover opening, so the deck reads as a sharply let-in panel (not a soft slot).
    # They sit on the deck and overlap both the lower_body and the side rails ->
    # connected, and they sharpen the haunch-to-deck transition.
    for ly, ltag in ((-0.86, "front"), (-1.74, "rear")):
        body.visual(
            Box((1.30, 0.05, 0.05)),
            origin=Origin(xyz=(0.0, ly, 0.78)),
            material=rosso,
            name=f"deck_frame_{ltag}",
        )

    # --- LARGE fixed rear wing on twin end-plate pylons ------------------------
    # The F40 signature: a big fixed rear wing standing tall off the tail on two
    # end-plate pylons, integrated with the rear deck. Wide flat blade, tall
    # pylons rooting DOWN INTO the rear deck sheetmetal so the whole wing is one
    # connected piece with the body (no floating wing). The deck top at the pylon
    # foot (x=+/-0.66, y=-2.06) is z~0.70-0.72, so the pylon bottom is driven to
    # z~0.62 (clearly inside the body) and the top reaches the blade underside.
    _PYLON_TOP = 1.24
    _PYLON_BOT = 0.62  # buried ~0.10 into the deck sheetmetal -> connected
    _PYLON_H = _PYLON_TOP - _PYLON_BOT
    _PYLON_ZC = (_PYLON_TOP + _PYLON_BOT) / 2.0
    for sx, side in ((0.66, "left"), (-0.66, "right")):
        body.visual(
            Box((0.10, 0.26, _PYLON_H)),
            origin=Origin(xyz=(sx, -2.06, _PYLON_ZC)),
            material=rosso,
            name=f"wing_pylon_{side}",
        )
    # Wide flat blade; its underside dips to ~1.20 so it bites onto the pylon tops.
    body.visual(
        Box((1.86, 0.44, 0.055)),
        origin=Origin(xyz=(0.0, -2.07, 1.245), rpy=(0.12, 0.0, 0.0)),
        material=rosso,
        name="wing_blade",
    )
    # Upturned trailing gurney lip on the wing's rear edge (overlaps the blade).
    body.visual(
        Box((1.86, 0.045, 0.075)),
        origin=Origin(xyz=(0.0, -2.270, 1.273), rpy=(0.12, 0.0, 0.0)),
        material=rosso,
        name="wing_gurney",
    )
    for sx, side in ((0.92, "left"), (-0.92, "right")):
        body.visual(
            Box((0.03, 0.46, 0.20)),
            origin=Origin(xyz=(sx, -2.07, 1.245)),
            material=black_trim,
            name=f"wing_endplate_{side}",
        )

    # --- Tail: Kamm tail, four round taillights, dual central exhausts, diffuser
    # Black rear tail panel.
    body.visual(
        Box((1.54, 0.06, 0.30)),
        origin=Origin(xyz=(0.0, -2.27, 0.50)),
        material=black_trim,
        name="tail_panel",
    )
    # FOUR round red taillights -- two per side (the F40 signature round lamps).
    for sx, side in ((0.62, "left"), (-0.62, "right")):
        for k, ix in enumerate((0.0, 0.20)):
            body.visual(
                Cylinder(radius=0.075, length=0.05),
                origin=Origin(xyz=(sx - (ix if sx > 0 else -ix), -2.295, 0.55), rpy=(pi / 2.0, 0.0, 0.0)),
                material=red_tail,
                name=f"taillight_{side}_{k}",
            )
            # Chrome ring around each round lamp.
            body.visual(
                Cylinder(radius=0.085, length=0.035),
                origin=Origin(xyz=(sx - (ix if sx > 0 else -ix), -2.285, 0.55), rpy=(pi / 2.0, 0.0, 0.0)),
                material=chrome,
                name=f"taillight_ring_{side}_{k}",
            )
    # Dual central round exhaust tips (the F40 runs a center pair up high) ...
    for sx, side in ((0.085, "left"), (-0.085, "right")):
        body.visual(
            Cylinder(radius=0.066, length=0.10),
            origin=Origin(xyz=(sx, -2.30, 0.34), rpy=(pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name=f"exhaust_ring_{side}",
        )
        body.visual(
            Cylinder(radius=0.048, length=0.12),
            origin=Origin(xyz=(sx, -2.31, 0.34), rpy=(pi / 2.0, 0.0, 0.0)),
            material=black_trim,
            name=f"exhaust_{side}",
        )
    # ... plus the F40's distinctive THIRD, lower central exhaust pipe sitting on
    # the centerline just beneath the twin tips. Its back end is buried into the
    # tail panel / twin-tip cluster so it stays a connected piece, not a float.
    body.visual(
        Cylinder(radius=0.060, length=0.10),
        origin=Origin(xyz=(0.0, -2.295, 0.235), rpy=(pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="exhaust_ring_center",
    )
    body.visual(
        Cylinder(radius=0.044, length=0.14),
        origin=Origin(xyz=(0.0, -2.305, 0.235), rpy=(pi / 2.0, 0.0, 0.0)),
        material=black_trim,
        name="exhaust_center",
    )
    # Black diffuser with vertical strakes across the lower tail.
    body.visual(
        Box((1.36, 0.18, 0.13)),
        origin=Origin(xyz=(0.0, -2.20, 0.16)),
        material=black_trim,
        name="rear_diffuser",
    )
    for k, fx in enumerate((-0.50, -0.28, 0.28, 0.50)):
        body.visual(
            Box((0.022, 0.16, 0.10)),
            origin=Origin(xyz=(fx, -2.20, 0.12)),
            material=black_trim,
            name=f"diffuser_fin_{k}",
        )

    body.inertial = Inertial.from_geometry(
        Box((1.98, 4.46, 1.13)),
        mass=1100.0,
        origin=Origin(xyz=(0.0, 0.0, 0.55)),
    )

    # --------------------------------------------------------------- doors
    hx, hy, hz = DOOR_HINGE

    def make_door(side: str):
        s = 1.0 if side == "left" else -1.0
        door = model.part(f"door_{side}")

        skin_sections = [
            (0.84, 0.20, 0.70, 0.18),
            (0.55, 0.17, 0.72, 0.22),
            (0.15, 0.17, 0.72, 0.22),
            (-0.28, 0.19, 0.70, 0.18),
        ]
        skin = superellipse_side_loft(skin_sections, exponents=3.2, segments=40)
        door.visual(
            _save(f"door_{side}_skin.obj", skin.translate(0.0, -hy, -hz)),
            material=rosso,
            name="door_skin",
        )

        glass_sections = [
            (0.76, 0.71, 0.80, 0.08),
            (0.48, 0.72, 0.95, 0.10),
            (0.10, 0.72, 1.00, 0.10),
            (-0.24, 0.72, 0.90, 0.08),
        ]
        glass_loft = superellipse_side_loft(glass_sections, exponents=2.6, segments=36)
        door.visual(
            _save(f"door_{side}_glass.obj", glass_loft.translate(-s * 0.26, -hy, -hz)),
            material=glass_tint,
            name="door_glass",
        )

        door.visual(
            Box((0.30, 1.04, 0.025)),
            origin=Origin(xyz=(-s * 0.125, -0.57, 0.015)),
            material=black_trim,
            name="door_beltline",
        )
        door.visual(
            Box((0.04, 0.14, 0.03)),
            origin=Origin(xyz=(s * 0.105, -0.92, -0.10)),
            material=black_trim,
            name="door_handle",
        )
        # Black side mirror on the door's front-top corner (F40 mirrors are black).
        door.visual(
            Box((0.12, 0.05, 0.04)),
            origin=Origin(xyz=(s * 0.11, -0.16, 0.0), rpy=(0.0, -s * 0.5, 0.0)),
            material=black_trim,
            name="mirror_stalk",
        )
        door.visual(
            Box((0.06, 0.15, 0.10)),
            origin=Origin(xyz=(s * 0.18, -0.18, 0.06)),
            material=black_trim,
            name="mirror_head",
        )

        door.inertial = Inertial.from_geometry(
            Box((0.20, 1.10, 0.85)),
            mass=28.0,
            origin=Origin(xyz=(0.0, -0.45, -0.15)),
        )
        return door

    door_left = make_door("left")
    door_right = make_door("right")

    # ---------------------------------------------------------------- wheels
    def make_wheel(name: str, outboard_sign: float):
        w = model.part(name)
        face_rpy = (0.0, 0.0, 0.0) if outboard_sign > 0 else (0.0, 0.0, pi)
        w.visual(
            _save(f"{name}_tire.obj", _TIRE_GEOM.clone()),
            origin=Origin(rpy=face_rpy),
            material=rubber,
            name="tire",
        )
        w.visual(
            _save(f"{name}_alloy.obj", _WHEEL_GEOM.clone()),
            origin=Origin(xyz=(outboard_sign * 0.04, 0.0, 0.0), rpy=face_rpy),
            material=silver,
            name="rim",
        )
        w.inertial = Inertial.from_geometry(
            Cylinder(radius=WHEEL_R, length=WHEEL_W),
            mass=22.0,
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        )
        return w

    make_wheel("wheel_front_left", 1.0)
    make_wheel("wheel_front_right", -1.0)
    make_wheel("wheel_rear_left", 1.0)
    make_wheel("wheel_rear_right", -1.0)

    # ----------------------------------------------- front steering knuckles
    def make_knuckle(name: str):
        k = model.part(name)
        k.inertial = Inertial.from_geometry(Box((0.10, 0.10, 0.22)), mass=6.0)
        return k

    knuckle_fl = make_knuckle("steer_knuckle_front_left")
    knuckle_fr = make_knuckle("steer_knuckle_front_right")

    # ----------------------------------------------------------- articulations
    model.articulation(
        "door_left_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door_left,
        origin=Origin(xyz=(hx, hy, hz)),
        axis=DOOR_AXIS_LEFT,
        motion_limits=MotionLimits(effort=60.0, velocity=2.0, lower=0.0, upper=DOOR_OPEN_MAX),
    )
    model.articulation(
        "door_right_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door_right,
        origin=Origin(xyz=(-hx, hy, hz)),
        axis=DOOR_AXIS_RIGHT,
        motion_limits=MotionLimits(effort=60.0, velocity=2.0, lower=0.0, upper=DOOR_OPEN_MAX),
    )

    STEER_LOCK = 0.40
    for knuckle, sx in ((knuckle_fl, HALF_TRACK), (knuckle_fr, -HALF_TRACK)):
        model.articulation(
            f"{knuckle.name}_steer",
            ArticulationType.REVOLUTE,
            parent=body,
            child=knuckle,
            origin=Origin(xyz=(sx, FRONT_AXLE_Y, WHEEL_R)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=120.0, velocity=4.0, lower=-STEER_LOCK, upper=STEER_LOCK
            ),
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
            motion_limits=MotionLimits(effort=200.0, velocity=60.0),
        )

    return model


# >>> USER_CODE_END

object_model = build_object_model()


def run_tests():
    from sdk import TestContext

    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door_l = object_model.get_part("door_left")
    door_r = object_model.get_part("door_right")
    wheels = {
        name: object_model.get_part(name)
        for name in (
            "wheel_front_left",
            "wheel_front_right",
            "wheel_rear_left",
            "wheel_rear_right",
        )
    }
    hinge_l = object_model.get_articulation("door_left_hinge")
    hinge_r = object_model.get_articulation("door_right_hinge")

    # --- Intentional overlap allowances --------------------------------------
    _sw_part = object_model.get_part("steering_wheel")
    for _selem in ("sw_hub", "sw_spoke_up", "sw_grip_left", "sw_grip_right"):
        ctx.allow_overlap(
            body,
            _sw_part,
            elem_a="steering_column",
            elem_b=_selem,
            reason="The fixed steering column meets the wheel hub/spokes at the center, on the spin axis.",
        )
    axle_of = {
        "wheel_front_left": "front_axle_bar",
        "wheel_front_right": "front_axle_bar",
        "wheel_rear_left": "rear_axle_bar",
        "wheel_rear_right": "rear_axle_bar",
    }
    for wname, w in wheels.items():
        for elem in ("tire", "rim"):
            ctx.allow_overlap(
                body,
                w,
                elem_a="lower_body",
                elem_b=elem,
                reason="Wheel seated flush inside the fender wheel arch of the solid body shell.",
            )
            ctx.allow_overlap(
                body,
                w,
                elem_a=axle_of[wname],
                elem_b=elem,
                reason="Straight axle rod runs out to the wheel hub on the spin/steer axis.",
            )
    # Door panels seat flush into the body door aperture; the inner half of the
    # skin / glass / beltline shelf is intentionally embedded in the flank.
    for door, side in ((door_l, "left"), (door_r, "right")):
        for shell in ("lower_body", "greenhouse", f"rocker_sill_{side}", "windshield"):
            for delem in ("door_skin", "door_glass", "door_beltline", "mirror_stalk"):
                ctx.allow_overlap(
                    body,
                    door,
                    elem_a=shell,
                    elem_b=delem,
                    reason="Front-hinged door seats flush in the body door aperture; thin embed is intentional.",
                )

    # --- Hero F40 features present and legible --------------------------------
    vis_names = {v.name for v in body.visuals}
    ctx.check(
        "lofted wedge body + greenhouse present (not a box)",
        {"lower_body", "greenhouse"} <= vis_names,
        details=f"body visuals={sorted(vis_names)}",
    )
    ctx.check(
        "windshield and rear window glass present",
        {"windshield", "rear_window"} <= vis_names,
    )
    ctx.check(
        "two pop-up-style headlight blisters flush on the front wings",
        {
            "headlight_blister_left",
            "headlight_blister_right",
            "headlight_left",
            "headlight_right",
        }
        <= vis_names,
    )
    ctx.check(
        "low full-width slatted front intake (>=3 slats) + splitter",
        {"front_intake", "front_splitter"} <= vis_names
        and sum(1 for v in body.visuals if v.name.startswith("front_intake_slat_")) >= 3,
    )
    ctx.check(
        "round driving lights + amber indicators in the nose, both sides",
        {
            "driving_light_left",
            "driving_light_right",
            "front_indicator_left",
            "front_indicator_right",
        }
        <= vis_names,
    )
    ctx.check(
        "yellow Ferrari shield badge on the nose",
        {"ferrari_badge", "badge_horse"} <= vis_names,
    )
    ctx.check(
        "large NACA duct flank scoops ahead of the rear wheels (both sides)",
        {"naca_duct_left", "naca_duct_right", "naca_ramp_left", "naca_ramp_right"} <= vis_names,
    )
    ctx.check(
        "black louvered engine cover over the deck (>=6 louvers)",
        "engine_deck" in vis_names
        and sum(1 for v in body.visuals if v.name.startswith("deck_louver_")) >= 6,
    )
    ctx.check(
        "large fixed rear wing on twin end-plate pylons",
        {"wing_blade", "wing_pylon_left", "wing_pylon_right", "wing_endplate_left", "wing_endplate_right"}
        <= vis_names,
    )
    ctx.check(
        "four round taillights (two per side)",
        sum(1 for v in body.visuals if v.name.startswith("taillight_") and "ring" not in v.name) == 4,
    )
    ctx.check(
        "dual central round exhaust tips",
        {"exhaust_left", "exhaust_right"} <= vis_names,
    )
    ctx.check(
        "black rocker sills both sides",
        {"rocker_sill_left", "rocker_sill_right"} <= vis_names,
    )
    ctx.check(
        "front and rear axle shafts present",
        {"front_axle_bar", "rear_axle_bar"} <= vis_names,
    )
    for door, side in ((door_l, "left"), (door_r, "right")):
        dnames = {v.name for v in door.visuals}
        ctx.check(
            f"door_{side} has skin, tinted glass, handle, mirror",
            {"door_skin", "door_glass", "door_handle", "mirror_head"} <= dnames,
            details=f"door_{side} visuals={sorted(dnames)}",
        )

    # --- F40 wedge profile: low pointy nose, high rear deck -------------------
    nose_top = max(z for (_x, y, z) in _lower_body_mesh().vertices if y > 2.0)
    deck_top = max(z for (_x, y, z) in _lower_body_mesh().vertices if -1.6 < y < -0.9)
    ctx.check(
        "wedge: low pointy nose sheetmetal is clearly lower than the rear deck",
        nose_top + 0.20 < deck_top,
        details=f"nose_top={nose_top:.3f}, deck_top={deck_top:.3f}",
    )
    # Nose tip is narrow and pointed (the F40 nose tapers to a point).
    nose_w = max(
        (abs(x) for (x, y, z) in _lower_body_mesh().vertices if y > 2.22),
        default=0.0,
    )
    ctx.check(
        "F40 nose tapers to a narrow point at the tip",
        nose_w < 0.55,
        details=f"nose half-width at tip={nose_w:.3f}",
    )
    # Front fender sheetmetal drapes over the front wheel (well capped from above).
    fender_cover = max(
        (
            z
            for (x, y, z) in _lower_body_mesh().vertices
            if abs(abs(x) - HALF_TRACK) < 0.18 and abs(y - FRONT_AXLE_Y) < 0.30
        ),
        default=0.0,
    )
    ctx.check(
        "front fender drapes above the front wheel top (caps the well from above)",
        fender_cover > 2.0 * WHEEL_R + 0.03,
        details=f"front fender top z={fender_cover:.3f}, wheel top z={2.0 * WHEEL_R:.3f}",
    )

    # --- Scale sanity ---------------------------------------------------------
    bb = ctx.part_world_aabb(body)
    assert bb is not None
    lo, hi = bb
    ctx.check("car length ~4.4-4.5 m", 4.3 <= hi[1] - lo[1] <= 4.7, details=f"L={hi[1] - lo[1]:.3f}")
    ctx.check("car width ~2.0 m", 1.9 <= hi[0] - lo[0] <= 2.2, details=f"W={hi[0] - lo[0]:.3f}")
    ctx.check("car height ~1.1-1.2 m (incl. wing)", 1.0 <= hi[2] <= 1.4, details=f"H={hi[2]:.3f}")

    # --- Large rear wing rides high above the deck on pylons -----------------
    wing = ctx.part_element_world_aabb(body, elem="wing_blade")
    pylon = ctx.part_element_world_aabb(body, elem="wing_pylon_left")
    deck = ctx.part_element_world_aabb(body, elem="engine_deck")
    assert wing is not None and pylon is not None and deck is not None
    ctx.check(
        "wing rides high above the engine deck",
        wing[0][2] > deck[1][2] + 0.15,
        details=f"wing bottom z={wing[0][2]:.3f}, deck top z={deck[1][2]:.3f}",
    )
    ctx.check(
        "wing is wide (full-width F40 spoiler)",
        (wing[1][0] - wing[0][0]) > 1.6,
        details=f"wing width={(wing[1][0] - wing[0][0]):.3f}",
    )
    ctx.check(
        "wing seats on the pylons; pylons root into the tail deck",
        pylon[1][2] >= wing[0][2] - 0.05 and pylon[0][2] <= 0.80,
        details=f"pylon z=[{pylon[0][2]:.3f},{pylon[1][2]:.3f}], wing bottom={wing[0][2]:.3f}",
    )

    # --- Glass reads darker than the Rosso Corsa paint -----------------------
    mats = {m.name: m for m in object_model.materials}
    glass_rgb = sum(mats["glass_dark"].rgba[:3])
    body_rgb = sum(mats["rosso_corsa"].rgba[:3])
    ctx.check(
        "glass is much darker than the red body",
        glass_rgb < body_rgb - 0.5,
        details=f"glass={glass_rgb:.2f}, rosso={body_rgb:.2f}",
    )
    ctx.check(
        "body paint is recognizably RED (R dominates G and B)",
        mats["rosso_corsa"].rgba[0] > 0.5
        and mats["rosso_corsa"].rgba[0] > mats["rosso_corsa"].rgba[1] + 0.4
        and mats["rosso_corsa"].rgba[0] > mats["rosso_corsa"].rgba[2] + 0.4,
        details=f"rosso rgba={mats['rosso_corsa'].rgba}",
    )

    # --- Doors: lateral-dominant revolute hinges, upward swing ---------------
    for hinge, door, side in ((hinge_l, door_l, "left"), (hinge_r, door_r, "right")):
        ax = tuple(hinge.axis)
        ctx.check(
            f"door_{side} hinge axis is lateral-dominant (upward swing)",
            abs(ax[0]) > 0.9 and abs(ax[1]) < 1e-6,
            details=f"axis={ax}",
        )
        ml = hinge.motion_limits
        ctx.check(
            f"door_{side} hinge limits ~[0, 1.4] rad",
            ml is not None and abs(ml.lower) < 1e-6 and 1.2 <= ml.upper <= 1.6,
            details=f"limits=({ml.lower}, {ml.upper})" if ml else "no limits",
        )
        ctx.expect_contact(body, door, contact_tol=0.05, name=f"door_{side} seated on body")

        rest = ctx.part_world_aabb(door)
        assert rest is not None
        with ctx.pose({hinge: 1.2}):
            opened = ctx.part_world_aabb(door)
            assert opened is not None
        ctx.check(
            f"door_{side} swings UP: top rises clearly when opened",
            opened[1][2] > rest[1][2] + 0.45,
            details=f"rest top z={rest[1][2]:.3f}, open top z={opened[1][2]:.3f}",
        )
        ctx.check(
            f"door_{side} bottom edge lifts clear of the rocker sill when open",
            opened[0][2] > rest[0][2] + 0.25,
            details=f"rest bottom z={rest[0][2]:.3f}, open bottom z={opened[0][2]:.3f}",
        )
        if side == "left":
            ctx.check(
                "door_left stays outboard of the cabin while opening",
                opened[0][0] >= rest[0][0] - 0.02,
                details=f"rest min x={rest[0][0]:.3f}, open min x={opened[0][0]:.3f}",
            )
        else:
            ctx.check(
                "door_right stays outboard of the cabin while opening",
                opened[1][0] <= rest[1][0] + 0.02,
                details=f"rest max x={rest[1][0]:.3f}, open max x={opened[1][0]:.3f}",
            )

    # --- Round steering wheel: present, and TURNS about the column -----------
    sw = object_model.get_part("steering_wheel")
    sw_names = {v.name for v in sw.visuals}
    ctx.check(
        "round 3-spoke steering wheel: rim, grips, hub, prancing-horse, marker",
        {
            "sw_rim_top",
            "sw_rim_bot",
            "sw_grip_left",
            "sw_grip_right",
            "sw_hub",
            "sw_prancing_horse",
            "sw_top_marker",
        }
        <= sw_names
        and "steering_column" in vis_names,
        details=f"sw visuals={sorted(sw_names)}",
    )
    sw_turn = object_model.get_articulation("steering_wheel_turn")
    ctx.check(
        "steering wheel joint is revolute about its column axis",
        tuple(sw_turn.axis) == (0.0, 0.0, 1.0)
        and sw_turn.articulation_type == ArticulationType.REVOLUTE,
        details=f"axis={sw_turn.axis}",
    )
    hub_rest = ctx.part_world_position(sw)
    mark_rest = ctx.part_element_world_aabb(sw, elem="sw_top_marker")
    assert hub_rest is not None and mark_rest is not None
    with ctx.pose({sw_turn: 1.2}):
        hub_turn = ctx.part_world_position(sw)
        mark_turn = ctx.part_element_world_aabb(sw, elem="sw_top_marker")
    assert hub_turn is not None and mark_turn is not None
    ctx.check(
        "steering wheel spins in place (hub center fixed, the top marker sweeps)",
        all(abs(hub_rest[i] - hub_turn[i]) < 1e-3 for i in range(3))
        and max(abs(mark_rest[0][i] - mark_turn[0][i]) for i in range(3)) > 0.05,
        details=f"hub rest={hub_rest}, turned={hub_turn}",
    )

    # --- Wheels: four continuous lateral spins, grounded and mirrored --------
    ground_zs = []
    for name, w in wheels.items():
        j = object_model.get_articulation(f"{name}_spin")
        ctx.check(
            f"{name} spin axis is lateral (X) and continuous",
            tuple(j.axis) == (1.0, 0.0, 0.0) and j.articulation_type == ArticulationType.CONTINUOUS,
            details=f"axis={j.axis}, type={j.articulation_type}",
        )
        wbb = ctx.part_world_aabb(w)
        assert wbb is not None
        ground_zs.append(wbb[0][2])
        ctx.check(
            f"{name} touches the ground plane",
            abs(wbb[0][2]) <= 0.02,
            details=f"min z={wbb[0][2]:.4f}",
        )
    ctx.check(
        "all four wheels touch the ground consistently",
        max(ground_zs) - min(ground_zs) <= 0.01,
        details=f"ground zs={['%.4f' % z for z in ground_zs]}",
    )
    flp = ctx.part_world_position(wheels["wheel_front_left"])
    frp = ctx.part_world_position(wheels["wheel_front_right"])
    assert flp is not None and frp is not None
    ctx.check(
        "front wheels mirror across the centerline",
        abs(flp[0] + frp[0]) < 0.02 and abs(flp[1] - frp[1]) < 0.02,
        details=f"fl={flp}, fr={frp}",
    )
    fl_spin = object_model.get_articulation("wheel_front_left_spin")
    rest_center = ctx.part_world_position(wheels["wheel_front_left"])
    with ctx.pose({fl_spin: 0.9}):
        spun_center = ctx.part_world_position(wheels["wheel_front_left"])
    assert rest_center is not None and spun_center is not None
    ctx.check(
        "front-left wheel rolls in place (center fixed under spin)",
        all(abs(rest_center[i] - spun_center[i]) < 1e-4 for i in range(3)),
        details=f"rest={rest_center}, spun={spun_center}",
    )
    ctx.check(
        "wheel center sits one radius above the ground",
        abs(rest_center[2] - WHEEL_R) < 1e-4,
        details=f"center z={rest_center[2]:.4f}, R={WHEEL_R}",
    )

    # --- Front steering: vertical king-pin pivots, wheels steer in place ------
    for kname, wname, side in (
        ("steer_knuckle_front_left", "wheel_front_left", "left"),
        ("steer_knuckle_front_right", "wheel_front_right", "right"),
    ):
        steer = object_model.get_articulation(f"{kname}_steer")
        ax = tuple(steer.axis)
        ctx.check(
            f"front-{side} steering axis is vertical (Z) and revolute",
            abs(ax[2]) > 0.9
            and abs(ax[0]) < 1e-6
            and abs(ax[1]) < 1e-6
            and steer.articulation_type == ArticulationType.REVOLUTE,
            details=f"axis={ax}, type={steer.articulation_type}",
        )
        sml = steer.motion_limits
        ctx.check(
            f"front-{side} steering has a symmetric lock (~+/-0.4 rad)",
            sml is not None
            and sml.lower < -0.25
            and sml.upper > 0.25
            and abs(sml.lower + sml.upper) < 1e-6,
            details=f"limits=({sml.lower}, {sml.upper})" if sml else "no limits",
        )

        w = wheels[wname]
        rest_c = ctx.part_world_position(w)
        rest_bb = ctx.part_world_aabb(w)
        assert rest_c is not None and rest_bb is not None
        with ctx.pose({steer: 0.35}):
            steer_c = ctx.part_world_position(w)
            steer_bb = ctx.part_world_aabb(w)
        assert steer_c is not None and steer_bb is not None
        ctx.check(
            f"front-{side} wheel steers in place (king-pin runs through its center)",
            all(abs(rest_c[i] - steer_c[i]) < 1e-3 for i in range(3)),
            details=f"rest={rest_c}, steered={steer_c}",
        )
        rest_xw = rest_bb[1][0] - rest_bb[0][0]
        steer_xw = steer_bb[1][0] - steer_bb[0][0]
        ctx.check(
            f"front-{side} wheel actually turns about the vertical axis",
            steer_xw > rest_xw + 0.10,
            details=f"rest x-extent={rest_xw:.3f}, steered x-extent={steer_xw:.3f}",
        )

    return ctx.report()
