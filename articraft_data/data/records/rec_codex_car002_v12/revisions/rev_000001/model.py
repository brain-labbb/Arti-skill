from __future__ import annotations

# Bright yellow Lamborghini Diablo style wedge supercar.
# Z-up world. Long axis of the car runs along +Y (nose at +Y), width along X
# (driver/left side at +X), up along +Z. Wheels touch z = 0.
#
# Primary articulation: BOTH scissor doors swing UP-and-forward on revolute
# hinges about a lateral-dominant axis located at each door's front-top edge
# (real Diablo hinges add a small outward tilt so the panel kicks clear of the
# flank as it lifts). Secondary: all four wheels spin (continuous about the
# lateral X axle axis). The TWO FRONT wheels additionally STEER: each hangs off
# a small steering knuckle that pivots about a near-vertical king-pin axis
# (revolute about Z through the wheel center), so the front wheels both turn
# left/right AND spin, with the spin axle swinging along with the steer angle.
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
# Global proportions (meters). Real Diablo: ~4.46 L x 2.04 W x 1.105 H,
# wheelbase 2.65, wheel radius ~0.33.
# ----------------------------------------------------------------------------
WHEEL_R = 0.33
WHEEL_W = 0.295
HALF_TRACK = 0.89
FRONT_AXLE_Y = 1.32
REAR_AXLE_Y = -1.33

# Wheel-arch cavities are carved out of the solid lower body at each wheel so
# the wheels sit in tight open wells instead of buried in solid sheetmetal.
# Each cutter is a lateral (X-axis) cylinder hugging the wheel (radius = wheel
# radius + a small clearance) that only opens the OUTBOARD flank: it spans |x|
# from an inboard wall (just past the tire's inner face) out through the body
# side, so the body stays solid behind the wheel and we don't gouge a big scoop.
# Front wells reach a touch deeper inboard to clear the steered front tire.
ARCH_RADIUS = 0.36
# (x_center_sign*track, y_center, inboard_wall_abs_x, outboard_wall_abs_x)
WHEEL_ARCHES = (
    (HALF_TRACK, FRONT_AXLE_Y, 0.58, 1.16),
    (-HALF_TRACK, FRONT_AXLE_Y, 0.58, 1.16),
    (HALF_TRACK, REAR_AXLE_Y, 0.70, 1.16),
    (-HALF_TRACK, REAR_AXLE_Y, 0.70, 1.16),
)

# Straight axle rods run hub-to-hub at wheel-center height. A transverse channel
# is bored clean through the lower body on each axle line so the rod passes
# THROUGH the body (visible in the open bore) instead of being buried in solid.
AXLE_BAR_RADIUS = 0.05
AXLE_CHANNEL_RADIUS = 0.085

# The cabin is hollowed out of the solid body so the seats + steering wheel sit
# in real open space, and the two door apertures are cut clean THROUGH the
# flanks so opening a door reveals the cabin, not a solid mesh cross-section.
# (x is half-width; y/z are (min, max) ranges in the body frame.)
CABIN_HALF_X = 0.60
CABIN_Y = (-0.80, 0.88)
CABIN_Z = (0.50, 0.71)  # lower-body interior: above the floor pan, up to body top
DOOR_APERTURE_X = (0.52, 1.05)  # inboard (into cabin) .. outboard (through flank)
DOOR_APERTURE_Y = (-0.26, 0.80)
DOOR_APERTURE_Z = (0.30, 0.70)

# Scissor door hinge point (left side; right side mirrors x), body frame.
DOOR_HINGE = (0.88, 0.84, 0.70)
# Lateral-dominant hinge axis with a small vertical tilt so the door kicks
# slightly OUTWARD while it swings up (real Diablo scissor kinematics).
_AX_TILT = 0.25
_AX_NORM = sqrt(1.0 + _AX_TILT * _AX_TILT)
DOOR_AXIS_LEFT = (-1.0 / _AX_NORM, 0.0, _AX_TILT / _AX_NORM)
DOOR_AXIS_RIGHT = (-1.0 / _AX_NORM, 0.0, -_AX_TILT / _AX_NORM)
DOOR_OPEN_MAX = 1.4

# Lower wedge body side-profile rails: (y, z_min, z_max, width).
# Sharp low nose, rising fender line, high rear deck, kamm tail.
LOWER_SECTIONS = [
    (2.23, 0.16, 0.35, 1.18),
    (2.06, 0.13, 0.42, 1.56),
    (1.80, 0.12, 0.55, 1.84),
    (1.50, 0.12, 0.70, 1.92),
    (1.32, 0.12, 0.76, 1.96),
    (1.15, 0.12, 0.73, 1.96),
    (0.92, 0.12, 0.70, 1.94),
    (0.45, 0.13, 0.68, 1.86),
    (-0.10, 0.13, 0.68, 1.86),
    (-0.65, 0.12, 0.72, 1.96),
    (-1.10, 0.11, 0.76, 2.02),
    (-1.45, 0.11, 0.77, 2.02),
    (-1.85, 0.13, 0.75, 1.92),
    (-2.10, 0.16, 0.72, 1.76),
    (-2.23, 0.20, 0.70, 1.60),
]

# Cab-forward greenhouse, built as THREE thin shells that share seam sections so
# the windshield / roof / rear window meet exactly (seamless), and each reads as
# a thin skin rather than a thick dome. Front + rear shells are tinted glass; the
# middle is the body-color roof. The seam rails are reused verbatim by the
# adjacent shells, which is what guarantees the panels line up perfectly.
_SEAM_FRONT = (0.20, 0.66, 1.10, 1.22)  # windshield top == roof leading edge
_SEAM_REAR = (-0.55, 0.66, 1.08, 1.14)  # roof trailing edge == rear window top
WINDSHIELD_SECTIONS = [
    (0.92, 0.62, 0.72, 1.46),
    (0.55, 0.64, 0.92, 1.34),
    _SEAM_FRONT,
]
# Roof raised and kept high (gently domed) across the WHOLE cabin so there is
# real headroom above the seats, instead of a low dome pressing down on them.
ROOF_SECTIONS = [
    _SEAM_FRONT,
    (-0.05, 0.66, 1.18, 1.20),
    (-0.42, 0.66, 1.17, 1.16),  # held high right over the seats
    _SEAM_REAR,
]
REAR_WINDOW_SECTIONS = [
    _SEAM_REAR,
    (-0.88, 0.66, 0.82, 1.10),
]
# Thin uniform shell thickness for roof + glass panes. Kept at 0.016 rather than
# 0.015 because the 4-section curved roof boolean degenerates at *exactly* 0.015
# (a coincident-edge fluke); 0.016 is robust and visually identical.
GLASS_SHELL_T = 0.016


def _save(name: str, geom):
    return mesh_from_geometry(geom, name)


def _box_cutter(x0, x1, y0, y1, z0, z1):
    # Axis-aligned box spanning the given world ranges, for boolean carving.
    # NOTE: BoxGeometry ships with inward-facing winding (negative volume), which
    # manifold3d reads as empty space -> the subtraction is a silent no-op. Flip
    # the face winding so it is a real solid cutter (CylinderGeometry is already
    # wound correctly, which is why the wheel wells carved but boxes did not).
    box = BoxGeometry((x1 - x0, y1 - y0, z1 - z0)).translate(
        (x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0
    )
    return MeshGeometry(
        vertices=list(box.vertices),
        faces=[(f[0], f[2], f[1]) for f in box.faces],
    )


_LOWER_BODY_CACHE = None


def _lower_body_mesh():
    # High exponent -> flat angular panels with hard shoulders (wedge, not blob).
    # Then subtract a lateral cylinder at each wheel to carve open wheel arches
    # so the wheels are no longer buried in solid sheetmetal. Cached because the
    # boolean carve is reused by the visual export and the QC checks.
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
        # Bore a straight transverse channel through each axle line so the rigid
        # axle rod passes cleanly THROUGH the body, connecting the two wheel wells.
        for ay in (FRONT_AXLE_Y, REAR_AXLE_Y):
            channel = (
                CylinderGeometry(
                    radius=AXLE_CHANNEL_RADIUS, height=2.0 * HALF_TRACK + 0.2, radial_segments=24
                )
                .rotate_y(pi / 2.0)
                .translate(0.0, ay, WHEEL_R)
            )
            body = boolean_difference(body, channel)
        # Hollow the cabin out of the solid lower body, then cut a door aperture
        # clean through each flank so opening a door reveals the open cabin.
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
        # Carve a shallow, wide-shallow raked pocket into each front-fender top so
        # the headlight cluster sits FLUSH/recessed (pop-up lamps shown down), not
        # proud on the sheetmetal. Raked -0.30 about X to follow the fender slope.
        for hx_sign in (1.0, -1.0):
            g = (
                BoxGeometry((0.40, 0.235, 0.10))
                .rotate_x(-0.30)
                .translate(hx_sign * 0.54, 1.925, 0.50)
            )
            pocket = MeshGeometry(
                vertices=list(g.vertices),
                faces=[(f[0], f[2], f[1]) for f in g.faces],
            )
            body = boolean_difference(body, pocket)
        _LOWER_BODY_CACHE = body
    return _LOWER_BODY_CACHE.clone()


def _glass_shell(sections, t=GLASS_SHELL_T):
    # A thin, uniform-thickness shell of a side-loft: subtract an inset copy of
    # the same loft (top/sides pulled in by t, bottom dropped well below so the
    # underside opens into the cabin). Adjacent shells built from shared seam
    # sections meet seamlessly because they share the seam rail exactly.
    outer = superellipse_side_loft(sections, exponents=2.8, segments=56)
    inner = superellipse_side_loft(
        [(y, zmin - 0.15, zmax - t, max(w - 2.0 * t, 0.04)) for (y, zmin, zmax, w) in sections],
        exponents=2.8,
        segments=56,
    )
    # superellipse_side_loft (like BoxGeometry) is wound INWARD (negative volume);
    # as a boolean subtrahend manifold3d reads it as empty -> a silent no-op that
    # leaves the shell SOLID. Flip the inner loft's faces so it actually carves
    # the shell hollow.
    inner = MeshGeometry(
        vertices=list(inner.vertices),
        faces=[(f[0], f[2], f[1]) for f in inner.faces],
    )
    return boolean_difference(outer, inner)


# Shared wheel/tire geometry: low-profile black tire + silver 5-spoke alloy.
_TIRE_GEOM = TireGeometry(
    WHEEL_R,
    WHEEL_W,
    inner_radius=0.215,
    carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.04),
    sidewall=TireSidewall(style="rounded", bulge=0.04),
)
_WHEEL_GEOM = WheelGeometry(
    0.218,
    0.20,
    rim=WheelRim(inner_radius=0.176, flange_height=0.012, flange_thickness=0.006),
    hub=WheelHub(
        radius=0.055,
        width=0.09,
        cap_style="domed",
        bolt_pattern=BoltPattern(count=5, circle_diameter=0.072, hole_diameter=0.009),
    ),
    face=WheelFace(dish_depth=0.025, front_inset=0.008),
    spokes=WheelSpokes(style="straight", count=5, thickness=0.028, window_radius=0.070),
    bore=WheelBore(style="round", diameter=0.04),
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dark_blue_low_wing_road_car")

    yellow = model.material("gloss_yellow", rgba=(0.03, 0.14, 0.42, 1.0))
    black_trim = model.material("black_trim", rgba=(0.05, 0.05, 0.055, 1.0))
    model.material("glass_dark", rgba=(0.08, 0.09, 0.11, 1.0))  # used by QC by name
    # Gray-black but see-through glazing for the windshield + side/rear windows.
    glass_tint = model.material("glass_tint", rgba=(0.12, 0.13, 0.15, 0.34))
    silver = model.material("silver_alloy", rgba=(0.74, 0.75, 0.78, 1.0))
    rubber = model.material("rubber", rgba=(0.04, 0.04, 0.045, 1.0))
    amber = model.material("amber", rgba=(0.85, 0.50, 0.06, 1.0))
    red_tail = model.material("tail_red", rgba=(0.45, 0.04, 0.05, 1.0))
    lens_pale = model.material("lens_pale", rgba=(0.72, 0.75, 0.78, 1.0))
    interior_dk = model.material("interior_dark", rgba=(0.10, 0.10, 0.11, 1.0))
    axle_steel = model.material("axle_steel", rgba=(0.66, 0.67, 0.70, 1.0))

    # ------------------------------------------------------------------ body
    body = model.part("body")
    body.visual(_save("lower_body.obj", _lower_body_mesh()), material=yellow, name="lower_body")
    # Glasshouse = three thin shells sharing seam rails, so the windshield and
    # rear window meet the roof seamlessly (and the roof reads as a thin skin).
    body.visual(_save("roof.obj", _glass_shell(ROOF_SECTIONS)), material=yellow, name="greenhouse")
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

    # Cabin interior (floor + seat backs) so the cabin is not an empty shell.
    body.visual(
        Box((1.10, 1.30, 0.10)),
        origin=Origin(xyz=(0.0, 0.05, 0.47)),
        material=interior_dk,
        name="cabin_floor",
    )
    # Seat backs sit low so the cabin keeps clear headroom under the roofline
    # (top ~0.76 m, well below the greenhouse roof, no roof-on-seat contact).
    for sx, side in ((0.32, "left"), (-0.32, "right")):
        body.visual(
            Box((0.42, 0.18, 0.30)),
            origin=Origin(xyz=(sx, -0.42, 0.61)),
            material=interior_dk,
            name=f"seat_back_{side}",
        )

    # Steering wheel on the driver (left, +X) side: a raked column rising out of
    # the cabin floor up to the wheel hub. The column is FIXED on the body; the
    # squared F1-style wheel itself is a SEPARATE articulated part that SPINS
    # about the raked column axis (see "steering_wheel" below), so it actually
    # turns. The column plunges into the cabin_floor so the assembly is connected
    # (no floating island); ~35 deg back-rake (rpy x = 0.6) faces the driver.
    _SW_X = 0.34
    _SW_FWD = 0.40  # steering assembly shifted forward (+Y, toward the windshield)
    _SW_RAKE = (0.6, 0.0, 0.0)
    _SW_HUB = (_SW_X, -0.02 + _SW_FWD, 0.69)  # wheel/hub center, top of the column
    body.visual(
        Cylinder(radius=0.022, length=0.34),
        origin=Origin(xyz=(_SW_X, 0.08 + _SW_FWD, 0.55), rpy=_SW_RAKE),
        material=black_trim,
        name="steering_column",
    )
    # ---- Squared F1-style steering wheel (separate part, spins on the column) --
    # Authored in its OWN un-raked frame: the rectangular rim, grips and spokes
    # lie in the local XY plane and the hub runs along +Z. The joint origin
    # applies the column rake and turns the whole wheel about local +Z (the
    # column line), so the square rim, padded grips, dash and the bright
    # top-centre marker all visibly sweep round.
    sw_accent = model.material("sw_accent", rgba=(0.85, 0.06, 0.06, 1.0))
    steer_wheel = model.part("steering_wheel")
    _RW = 0.150  # rim half-width (X) -- wider than tall, like an F1 wheel
    _RH = 0.115  # rim half-height (Y)
    _RT = 0.028  # rim bar cross-section
    _RD = 0.045  # rim depth along the column axis (Z)
    # Flat top and bottom bars (full width so the squared corners join solid).
    for _sy, _tag in ((_RH, "top"), (-_RH, "bot")):
        steer_wheel.visual(
            Box((2.0 * _RW + _RT, _RT, _RD)),
            origin=Origin(xyz=(0.0, _sy, 0.0)),
            material=black_trim,
            name=f"sw_rim_{_tag}",
        )
    # Left/right padded grips: thicker uprights where the driver holds the wheel.
    for _sx, _tag in ((_RW, "left"), (-_RW, "right")):
        steer_wheel.visual(
            Box((_RT + 0.018, 2.0 * _RH - 0.010, _RD + 0.014)),
            origin=Origin(xyz=(_sx, 0.0, 0.0)),
            material=black_trim,
            name=f"sw_grip_{_tag}",
        )
    # Central spokes: a cross-bar across X through the hub + a lower spoke down.
    steer_wheel.visual(
        Box((2.0 * _RW, 0.030, 0.020)),
        material=black_trim,
        name="sw_spoke_cross",
    )
    steer_wheel.visual(
        Box((0.030, _RH, 0.020)),
        origin=Origin(xyz=(0.0, -_RH / 2.0, 0.0)),
        material=black_trim,
        name="sw_spoke_low",
    )
    # Hub barrel running back down the column (overlaps the fixed column top so
    # the wheel reads as connected, not floating).
    steer_wheel.visual(
        Cylinder(radius=0.045, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, -0.02)),
        material=black_trim,
        name="sw_hub",
    )
    # Flat rectangular F1 "dash" display on the front face + two control buttons.
    steer_wheel.visual(
        Box((0.150, 0.080, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=interior_dk,
        name="sw_display",
    )
    for _sx, _bmat, _tag in ((0.050, amber, "r"), (-0.050, lens_pale, "l")):
        steer_wheel.visual(
            Cylinder(radius=0.014, length=0.014),
            origin=Origin(xyz=(_sx, -0.024, 0.040)),
            material=_bmat,
            name=f"sw_button_{_tag}",
        )
    # Bright top-centre alignment marker (the F1 "12 o'clock" stripe). Off the
    # spin axis, so it sweeps round and proves the wheel is turning.
    steer_wheel.visual(
        Box((0.034, 0.020, 0.020)),
        origin=Origin(xyz=(0.0, _RH, 0.020)),
        material=sw_accent,
        name="sw_top_marker",
    )
    steer_wheel.inertial = Inertial.from_geometry(Box((0.33, 0.27, 0.06)), mass=2.0)
    model.articulation(
        "steering_wheel_turn",
        ArticulationType.REVOLUTE,
        parent=body,
        child=steer_wheel,
        origin=Origin(xyz=_SW_HUB, rpy=_SW_RAKE),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=8.0, lower=-3.14, upper=3.14),
    )

    # Black front splitter under the nose.
    body.visual(
        Box((1.46, 0.42, 0.075)),
        origin=Origin(xyz=(0.0, 2.04, 0.105)),
        material=black_trim,
        name="front_splitter",
    )
    # Black bumper strip across the nose with amber indicators + driving lights.
    body.visual(
        Box((1.18, 0.06, 0.14)),
        origin=Origin(xyz=(0.0, 2.225, 0.26)),
        material=black_trim,
        name="front_bumper_strip",
    )
    for sx, side in ((0.50, "left"), (-0.50, "right")):
        body.visual(
            Box((0.14, 0.05, 0.08)),
            origin=Origin(xyz=(sx, 2.245, 0.26)),
            material=amber,
            name=f"front_indicator_{side}",
        )
    for sx, side in ((0.24, "left"), (-0.24, "right")):
        body.visual(
            Box((0.14, 0.05, 0.08)),
            origin=Origin(xyz=(sx, 2.245, 0.26)),
            material=lens_pale,
            name=f"driving_light_{side}",
        )

    # Headlight clusters flush in the carved fender pockets (Diablo pop-ups shown
    # DOWN/closed): a black recess surround, a pale reflector backing split by a
    # thin bar, and a dark glazed lens flush with the fender -- clean and
    # integrated, no busy round projectors. Amber turn markers live on the
    # bumper strip below. Raked -0.30 about X to lie on the fender slope.
    _HL_RAKE = (-0.30, 0.0, 0.0)
    for sx, side in ((0.54, "left"), (-0.54, "right")):
        # Black surround lining the carved recess (thin dark frame).
        body.visual(
            Box((0.37, 0.20, 0.05)),
            origin=Origin(xyz=(sx, 1.925, 0.455), rpy=_HL_RAKE),
            material=black_trim,
            name=f"headlight_surround_{side}",
        )
        # Pale reflector backing -- reads as the lit lens through the dark glaze.
        body.visual(
            Box((0.34, 0.17, 0.014)),
            origin=Origin(xyz=(sx, 1.925, 0.475), rpy=_HL_RAKE),
            material=lens_pale,
            name=f"headlight_reflector_{side}",
        )
        # Thin dark bar splitting the lens into two horizontal elements.
        body.visual(
            Box((0.34, 0.014, 0.02)),
            origin=Origin(xyz=(sx, 1.925, 0.483), rpy=_HL_RAKE),
            material=black_trim,
            name=f"headlight_bar_{side}",
        )
        # Dark glazed lens, flush with the fender surface (the face you see).
        body.visual(
            Box((0.35, 0.18, 0.012)),
            origin=Origin(xyz=(sx, 1.926, 0.490), rpy=_HL_RAKE),
            material=glass_tint,
            name=f"headlight_{side}",
        )
    body.visual(
        Box((0.020, 0.92, 0.016)),
        origin=Origin(xyz=(0.0, 1.53, 0.59), rpy=(-0.10, 0.0, 0.0)),
        material=black_trim,
        name="hood_center_panel_gap",
    )
    body.visual(
        Box((1.02, 0.018, 0.016)),
        origin=Origin(xyz=(0.0, 0.88, 0.61)),
        material=black_trim,
        name="hood_rear_panel_gap",
    )

    # (Hood vents removed; replaced by the windshield wipers added below.)

    # Black rocker sills between the wheel arches.
    for sx, side in ((0.835, "left"), (-0.835, "right")):
        body.visual(
            Box((0.12, 1.72, 0.13)),
            origin=Origin(xyz=(sx, 0.10, 0.175)),
            material=black_trim,
            name=f"rocker_sill_{side}",
        )

    # Front and rear axle rods: a straight steel shaft on each axle line at
    # wheel-center height, running hub to hub THROUGH the bored channel in the
    # body (see _lower_body_mesh). Rigid with the body. The REAR rod is the rear
    # wheels' spin axis; the FRONT rod ends on the steering king-pins
    # (x = +/-HALF_TRACK), so each front wheel pivots about its rod end and the
    # rod never blocks the steering.
    for ay, axle_name in ((FRONT_AXLE_Y, "front_axle_bar"), (REAR_AXLE_Y, "rear_axle_bar")):
        body.visual(
            Cylinder(radius=AXLE_BAR_RADIUS, length=2.0 * HALF_TRACK),
            origin=Origin(xyz=(0.0, ay, WHEEL_R), rpy=(0.0, pi / 2.0, 0.0)),
            material=axle_steel,
            name=axle_name,
        )

    # Deep black side intakes ahead of the rear wheels, with a body-color slat.
    for sx, side in ((0.94, "left"), (-0.94, "right")):
        body.visual(
            Box((0.10, 0.52, 0.26)),
            origin=Origin(xyz=(sx, -0.70, 0.49)),
            material=black_trim,
            name=f"side_intake_{side}",
        )
        body.visual(
            Box((0.105, 0.50, 0.04)),
            origin=Origin(xyz=(sx * (0.945 / 0.94), -0.70, 0.49)),
            material=yellow,
            name=f"intake_slat_{side}",
        )

    # Black louvered engine deck behind the cabin.
    body.visual(
        Box((1.30, 0.85, 0.05)),
        origin=Origin(xyz=(0.0, -1.30, 0.74)),
        material=black_trim,
        name="engine_deck",
    )
    for k in range(6):
        body.visual(
            Box((1.24, 0.085, 0.02)),
            origin=Origin(xyz=(0.0, -0.96 - 0.13 * k, 0.778), rpy=(0.12, 0.0, 0.0)),
            material=black_trim,
            name=f"deck_louver_{k}",
        )

    # Tall rear wing on twin pylons, firmly seated into the rear deck.
    for sx, side in ((0.52, "left"), (-0.52, "right")):
        body.visual(
            Box((0.09, 0.24, 0.30)),
            origin=Origin(xyz=(sx, -1.92, 0.82)),
            material=yellow,
            name=f"wing_pylon_{side}",
        )
    body.visual(
        Box((1.62, 0.28, 0.040)),
        origin=Origin(xyz=(0.0, -1.96, 0.970), rpy=(0.08, 0.0, 0.0)),
        material=yellow,
        name="wing_blade",
    )
    for sx, side in ((0.87, "left"), (-0.87, "right")):
        body.visual(
            Box((0.025, 0.24, 0.080)),
            origin=Origin(xyz=(sx, -1.96, 0.970)),
            material=black_trim,
            name=f"wing_endplate_{side}",
        )

    # Tail: black rear panel, taillights, exhausts, diffuser with fins.
    body.visual(
        Box((1.50, 0.06, 0.30)),
        origin=Origin(xyz=(0.0, -2.225, 0.50)),
        material=black_trim,
        name="tail_panel",
    )
    for sx, side in ((0.50, "left"), (-0.50, "right")):
        body.visual(
            Box((0.30, 0.04, 0.12)),
            origin=Origin(xyz=(sx, -2.26, 0.55)),
            material=red_tail,
            name=f"taillight_{side}",
        )
    for sx, side in ((0.18, "left"), (-0.18, "right")):
        body.visual(
            Cylinder(radius=0.045, length=0.10),
            origin=Origin(xyz=(sx, -2.26, 0.30), rpy=(pi / 2.0, 0.0, 0.0)),
            material=black_trim,
            name=f"exhaust_{side}",
        )
    body.visual(
        Box((1.30, 0.16, 0.12)),
        origin=Origin(xyz=(0.0, -2.17, 0.16)),
        material=black_trim,
        name="rear_diffuser",
    )
    for k, fx in enumerate((-0.45, -0.15, 0.15, 0.45)):
        body.visual(
            Box((0.02, 0.14, 0.08)),
            origin=Origin(xyz=(fx, -2.17, 0.10)),
            material=black_trim,
            name=f"diffuser_fin_{k}",
        )

    body.inertial = Inertial.from_geometry(
        Box((2.0, 4.46, 1.10)),
        mass=1530.0,
        origin=Origin(xyz=(0.0, 0.0, 0.55)),
    )

    # ----------------------------------------------------------- scissor doors
    hx, hy, hz = DOOR_HINGE

    def make_door(side: str):
        s = 1.0 if side == "left" else -1.0
        door = model.part(f"door_{side}")

        # Door skin loft authored in body frame, shifted into the local hinge
        # frame (hinge sits at the door's front-top edge).
        skin_sections = [
            (0.84, 0.20, 0.70, 0.18),
            (0.55, 0.17, 0.72, 0.22),
            (0.15, 0.17, 0.72, 0.22),
            (-0.28, 0.19, 0.70, 0.18),
        ]
        skin = superellipse_side_loft(skin_sections, exponents=3.2, segments=40)
        door.visual(
            _save(f"door_{side}_skin.obj", skin.translate(0.0, -hy, -hz)),
            material=yellow,
            name="door_skin",
        )

        # Tinted side window carried by the door (tumblehome: inboard of skin).
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

        # Beltline shelf bridging skin top to the inboard window glass.
        door.visual(
            Box((0.30, 1.04, 0.025)),
            origin=Origin(xyz=(-s * 0.125, -0.57, 0.015)),
            material=black_trim,
            name="door_beltline",
        )
        # Flush black door handle near the rear edge.
        door.visual(
            Box((0.04, 0.14, 0.03)),
            origin=Origin(xyz=(s * 0.105, -0.92, -0.10)),
            material=black_trim,
            name="door_handle",
        )
        # Body-color side mirror on the door's front-top corner.
        door.visual(
            Box((0.12, 0.05, 0.04)),
            origin=Origin(xyz=(s * 0.11, -0.16, 0.0), rpy=(0.0, -s * 0.5, 0.0)),
            material=yellow,
            name="mirror_stalk",
        )
        door.visual(
            Box((0.06, 0.15, 0.10)),
            origin=Origin(xyz=(s * 0.18, -0.18, 0.06)),
            material=yellow,
            name="mirror_head",
        )

        door.inertial = Inertial.from_geometry(
            Box((0.20, 1.10, 0.85)),
            mass=32.0,
            origin=Origin(xyz=(0.0, -0.45, -0.15)),
        )
        return door

    door_left = make_door("left")
    door_right = make_door("right")

    # ---------------------------------------------------------------- wheels
    def make_wheel(name: str, outboard_sign: float):
        w = model.part(name)
        # Flip right-side wheels so the dished alloy face points outboard.
        face_rpy = (0.0, 0.0, 0.0) if outboard_sign > 0 else (0.0, 0.0, pi)
        w.visual(
            _save(f"{name}_tire.obj", _TIRE_GEOM.clone()),
            origin=Origin(rpy=face_rpy),
            material=rubber,
            name="tire",
        )
        # Alloy shifted slightly outboard so the 5-spoke face reads outside
        # the tire shoulder (still captured inside the tire bead).
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
    # Tiny carrier links (inertial only, no visual) seated at each front wheel
    # center. The body steers the knuckle about the vertical king-pin axis; the
    # wheel then spins off the knuckle, so steering and spin compose cleanly.
    def make_knuckle(name: str):
        k = model.part(name)
        k.inertial = Inertial.from_geometry(Box((0.10, 0.10, 0.22)), mass=6.0)
        return k

    knuckle_fl = make_knuckle("steer_knuckle_front_left")
    knuckle_fr = make_knuckle("steer_knuckle_front_right")

    # ----------------------------------------------------------- articulations
    # Scissor doors: revolute about a lateral-dominant axis at the front-top
    # door edge; positive q swings the trailing edge UP and forward.
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

    # Front steering: revolute about the vertical (Z) king-pin axis, pivoting
    # the knuckle (and the wheel it carries) in place. ~+/-0.40 rad (~23 deg)
    # lock — a visible steering throw that the tight front wheel wells still
    # clear without the wheel sweeping into the bodywork.
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

    # Wheel spin: continuous about the lateral X axle. Front wheels spin off
    # their steering knuckle (origin at the knuckle = wheel center, so the spin
    # axle swings with the steer angle); rear wheels spin directly off the body.
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
    # The fixed steering column meets the rotating wheel's hub/spokes at the
    # center, on the spin axis.
    _sw_part = object_model.get_part("steering_wheel")
    for _selem in ("sw_hub", "sw_spoke_cross", "sw_spoke_low"):
        ctx.allow_overlap(
            body,
            _sw_part,
            elem_a="steering_column",
            elem_b=_selem,
            reason="The fixed steering column meets the wheel hub/spokes at the center, on the spin axis.",
        )
    # Wheels are seated inside the fender wheel arches of the solid wedge shell,
    # and each straight axle rod runs hub to hub through its wheels.
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
                    reason="Scissor door seats flush in the body door aperture; thin embed is intentional.",
                )

    # --- Hero features present and legible -----------------------------------
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
        "two flush headlight clusters in the nose",
        {"headlight_left", "headlight_right"} <= vis_names,
    )
    ctx.check(
        "amber indicators + driving lights + splitter + bumper strip",
        {
            "front_indicator_left",
            "front_indicator_right",
            "driving_light_left",
            "driving_light_right",
            "front_splitter",
            "front_bumper_strip",
        }
        <= vis_names,
    )
    ctx.check(
        "black rocker sills both sides",
        {"rocker_sill_left", "rocker_sill_right"} <= vis_names,
    )
    ctx.check(
        "deep side intakes ahead of rear wheels (both sides)",
        {"side_intake_left", "side_intake_right"} <= vis_names,
    )
    ctx.check(
        "louvered engine deck (>=5 louvers)",
        sum(1 for v in body.visuals if v.name.startswith("deck_louver_")) >= 5,
    )
    ctx.check(
        "rear wing on twin pylons",
        {"wing_blade", "wing_pylon_left", "wing_pylon_right"} <= vis_names,
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

    # --- Wedge profile: nose drops low, rear deck rides high -----------------
    nose_top = max(z for (_x, y, z) in _lower_body_mesh().vertices if y > 1.9)
    deck_top = max(z for (_x, y, z) in _lower_body_mesh().vertices if -1.6 < y < -0.9)
    ctx.check(
        "wedge: nose sheetmetal is far lower than the rear deck",
        nose_top + 0.20 < deck_top,
        details=f"nose_top={nose_top:.3f}, deck_top={deck_top:.3f}",
    )
    # Front fender sheetmetal is raised so the body drapes over the front wheel
    # and the wheel-well gap is capped from above (not left open to the sky).
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
    hl = ctx.part_element_world_aabb(body, elem="headlight_left")
    louver = ctx.part_element_world_aabb(body, elem="deck_louver_2")
    assert hl is not None and louver is not None
    ctx.check(
        "headlights sit low on the wedge nose, below the engine deck line",
        hl[1][2] < louver[0][2],
        details=f"headlight top z={hl[1][2]:.3f}, louver bottom z={louver[0][2]:.3f}",
    )

    # --- Scale sanity ---------------------------------------------------------
    bb = ctx.part_world_aabb(body)
    assert bb is not None
    lo, hi = bb
    ctx.check("car length ~4.5 m", 4.3 <= hi[1] - lo[1] <= 4.7, details=f"L={hi[1] - lo[1]:.3f}")
    ctx.check("car width ~2.0 m", 1.9 <= hi[0] - lo[0] <= 2.2, details=f"W={hi[0] - lo[0]:.3f}")
    ctx.check("car height ~1.1 m", 1.0 <= hi[2] <= 1.2, details=f"H={hi[2]:.3f}")

    # --- Rear wing mounted above the deck on pylons (not floating) -----------
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
        "wing seats on the pylons; pylons root into the tail deck",
        pylon[1][2] >= wing[0][2] - 0.01 and pylon[0][2] <= 0.745,
        details=f"pylon z=[{pylon[0][2]:.3f},{pylon[1][2]:.3f}], wing bottom={wing[0][2]:.3f}",
    )

    # --- Glass reads darker than the yellow paint ----------------------------
    mats = {m.name: m for m in object_model.materials}
    glass_rgb = sum(mats["glass_dark"].rgba[:3])
    body_rgb = sum(mats["gloss_yellow"].rgba[:3])
    ctx.check(
        "glass is darker than the dark blue body paint",
        glass_rgb < body_rgb,
        details=f"glass={glass_rgb:.2f}, body={body_rgb:.2f}",
    )

    # --- Scissor doors: lateral-dominant revolute hinges, upward swing -------
    for hinge, door, side in ((hinge_l, door_l, "left"), (hinge_r, door_r, "right")):
        ax = tuple(hinge.axis)
        ctx.check(
            f"door_{side} hinge axis is lateral-dominant (scissor, not gullwing/normal)",
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
            f"door_{side} swings UP (scissor): top rises ~hinge height + door length",
            opened[1][2] > rest[1][2] + 0.45,
            details=f"rest top z={rest[1][2]:.3f}, open top z={opened[1][2]:.3f}",
        )
        ctx.check(
            f"door_{side} bottom edge lifts clear of the rocker sill when open",
            opened[0][2] > rest[0][2] + 0.25,
            details=f"rest bottom z={rest[0][2]:.3f}, open bottom z={opened[0][2]:.3f}",
        )
        # The tilted hinge kicks the panel slightly outboard, never inboard.
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

    # --- Squared F1 steering wheel: present, and TURNS about the column -------
    sw = object_model.get_part("steering_wheel")
    sw_names = {v.name for v in sw.visuals}
    ctx.check(
        "squared F1 steering wheel: flat top/bottom rim, side grips, dash + marker",
        {
            "sw_rim_top",
            "sw_rim_bot",
            "sw_grip_left",
            "sw_grip_right",
            "sw_hub",
            "sw_display",
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
    # The wheel spins in place: the hub center stays put (the column line runs
    # through it) while the off-axis top marker sweeps round.
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
