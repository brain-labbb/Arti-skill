from __future__ import annotations

# Koenigsegg Regera style mid-engine hypercar, icy light blue.
# Z-up world. Long axis of the car runs along +Y (nose at +Y), width along X
# (driver/left side at +X), up along +Z. Wheels touch z = 0.
#
# Forked from the Diablo wedge supercar: the proven STRUCTURE is reused -- the
# cabin is hollowed out of the solid body by boolean_difference (cabin cavity +
# door apertures cut clean THROUGH the flanks), each front wheel hangs off a
# steering knuckle so it both STEERS about a vertical king-pin AND spins off the
# knuckle, rear wheels spin off the body, and straight axle rods run hub-to-hub
# through bored channels. The body is re-skinned from a sharp wedge into the
# Regera's smooth ROUNDED organic form.
#
# Primary articulation: BOTH DIHEDRAL synchro-helix doors swing OUT-and-UP on a
# revolute hinge about a forward/outward-canted near-vertical axis at the front
# of the door (not scissor, not gullwing). Secondary: all four wheels spin
# (continuous about lateral X); the two FRONT wheels additionally steer.
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
# Global proportions (meters). Real Regera: ~4.56 L x 2.05 W x 1.11 H,
# wheelbase 2.66, wheel radius ~0.34.
# ----------------------------------------------------------------------------
WHEEL_R = 0.335
WHEEL_W = 0.30
# Track narrowed so the wheels tuck UNDER the fender crests (outer ~flush with
# the body) instead of floating proud of the flank.
HALF_TRACK = 0.81
FRONT_AXLE_Y = 1.34
REAR_AXLE_Y = -1.32

# Wheel-arch cavities carved out of the solid lower body at each wheel so the
# wheels sit in tight open wells. Each cutter is a lateral (X) cylinder hugging
# the wheel that only opens the OUTBOARD flank.
ARCH_RADIUS = 0.37
# (x_center_sign*track, y_center, inboard_wall_abs_x, outboard_wall_abs_x)
WHEEL_ARCHES = (
    (HALF_TRACK, FRONT_AXLE_Y, 0.50, 1.04),
    (-HALF_TRACK, FRONT_AXLE_Y, 0.50, 1.04),
    (HALF_TRACK, REAR_AXLE_Y, 0.56, 1.06),
    (-HALF_TRACK, REAR_AXLE_Y, 0.56, 1.06),
)

# Straight axle rods run hub-to-hub at wheel-center height; a transverse channel
# is bored clean through the lower body on each axle line so the rod passes
# THROUGH the body instead of being buried in solid.
AXLE_BAR_RADIUS = 0.05
AXLE_CHANNEL_RADIUS = 0.085

# The cabin is hollowed out of the solid body so the seats + steering wheel sit
# in real open space, and the two door apertures are cut clean THROUGH the
# flanks so opening a door reveals the cabin, not a solid mesh cross-section.
# The cabin must be hollowed ALL THE WAY UP to where the glass canopy takes over
# (z ~0.82), otherwise a solid lid of lower body (~0.72-0.80) sits over the seats
# and buries them. Front edge stops at y~0.70 so the cowl/scuttle under the
# windshield base stays solid.
CABIN_HALF_X = 0.62
CABIN_Y = (-0.80, 0.70)
CABIN_Z = (0.45, 0.83)
DOOR_APERTURE_X = (0.52, 1.06)  # inboard (into cabin) .. outboard (through flank)
DOOR_APERTURE_Y = (-0.30, 0.66)
# Door-opening bottom raised to the CABIN FLOOR level (~0.45) so the door sill is
# FLUSH with the floor -- no 15 cm step between the black cabin floor and the door.
DOOR_APERTURE_Z = (0.45, 0.74)

# Dihedral synchro-helix door hinge (left; right mirrors x), at the FRONT-LOWER
# corner of the door. The axis is a near-vertical king-pin canted FORWARD and a
# touch OUTWARD so one rotation swings the door OUT and UP (Koenigsegg dihedral),
# unlike the Diablo's lateral-dominant scissor axis.
DOOR_HINGE = (0.86, 0.88, 0.44)
# Near-vertical king-pin canted FORE-AFT: rotating the left door about it swings
# it OUT (+X) and UP; the right door uses the fully negated axis so it mirrors
# (OUT -X and UP) under the same positive opening angle.
_DH = (0.0, -0.45, 0.89)
_DN = sqrt(_DH[0] * _DH[0] + _DH[1] * _DH[1] + _DH[2] * _DH[2])
DOOR_AXIS_LEFT = (_DH[0] / _DN, _DH[1] / _DN, _DH[2] / _DN)
DOOR_AXIS_RIGHT = (-_DH[0] / _DN, -_DH[1] / _DN, -_DH[2] / _DN)
DOOR_OPEN_MAX = 1.45

# Lower body side-profile rails: (y, z_min, z_max, width). Smooth ROUNDED form:
# soft low nose, gently domed hood, rounded front + rear haunches, tapered tail.
LOWER_SECTIONS = [
    (2.30, 0.20, 0.35, 0.92),   # very low rounded nose tip
    (2.14, 0.14, 0.46, 1.36),
    (1.90, 0.12, 0.60, 1.78),   # fender shoulder rising
    (1.60, 0.12, 0.79, 1.95),   # peaked front fender crown (drapes over wheel)
    (1.34, 0.12, 0.83, 1.98),   # over front axle -- fender caps the well
    (1.06, 0.13, 0.77, 1.92),   # hood dips just below the fender crown
    (0.76, 0.13, 0.76, 1.85),   # cowl
    (0.36, 0.14, 0.76, 1.81),   # door front / low beltline
    (-0.12, 0.14, 0.77, 1.84),  # door mid
    (-0.58, 0.12, 0.80, 1.97),  # rear haunch swelling
    (-1.02, 0.11, 0.84, 2.07),  # muscular rear haunch crown (widest, tallest)
    (-1.44, 0.11, 0.82, 2.03),  # over rear axle
    (-1.84, 0.13, 0.75, 1.84),  # rear deck taper
    (-2.12, 0.18, 0.68, 1.56),
    (-2.30, 0.24, 0.60, 1.30),  # rounded kamm tail
]

# Rounded bubble canopy, built as THREE thin shells that share seam sections so
# windshield / roof / rear glass meet exactly. Front + rear are tinted glass; the
# middle is the body-color roof. Short and domed (Regera bubble greenhouse).
# Low, long, narrow teardrop canopy (Regera is sleek, not a tall bubble).
_SEAM_FRONT = (0.34, 0.60, 1.01, 1.12)  # windshield top == roof leading edge
_SEAM_REAR = (-0.70, 0.58, 0.96, 0.94)  # roof trailing edge == rear window top
WINDSHIELD_SECTIONS = [
    (0.88, 0.58, 0.72, 1.40),
    (0.60, 0.60, 0.90, 1.24),
    _SEAM_FRONT,
]
ROOF_SECTIONS = [
    _SEAM_FRONT,
    (0.02, 0.62, 1.05, 1.06),  # low gentle dome over the seats (~1.05 m roof)
    (-0.36, 0.62, 1.02, 0.98),
    _SEAM_REAR,
]
REAR_WINDOW_SECTIONS = [
    _SEAM_REAR,
    (-0.96, 0.60, 0.82, 0.88),
]
# Thin uniform shell thickness for roof + glass panes (0.016 is robust against a
# coincident-edge degeneracy the curved-roof boolean hits at exactly 0.015).
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


# Hood centre-channel carve params (big fore-aft cylinder dipping the hood
# centre so the flanks read as raised fender crests).
HOOD_CHAN_R = 0.95      # cylinder radius -> sets channel width
HOOD_CHAN_CROWN = 0.80  # hood crown z at the centre (where the carve starts)
HOOD_CHAN_DEPTH = 0.10  # how far the centre is dropped

_LOWER_BODY_CACHE = None


def _lower_body_mesh():
    # Low exponent -> rounded, soft superellipse cross-sections (organic Regera
    # body, not the angular wedge). Then carve open wheel arches, axle channels,
    # the hollow cabin and the two door apertures. Cached (reused by visual
    # export + QC).
    global _LOWER_BODY_CACHE
    if _LOWER_BODY_CACHE is None:
        body = superellipse_side_loft(LOWER_SECTIONS, exponents=2.3, segments=64)
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
        # Hood centre channel: the Regera hood DIPS in the middle between two
        # raised fender crests (two-up, centre-down streamline). The single-peak
        # superellipse loft is convex (highest at centre), so carve a wide shallow
        # valley down the hood centre with a big cylinder laid fore-aft -- the
        # centre drops and the flanks read as raised fenders.
        hood_chan = (
            CylinderGeometry(radius=HOOD_CHAN_R, height=1.15, radial_segments=56)
            .rotate_x(pi / 2.0)  # axis Z -> Y (fore-aft)
            .translate(0.0, 1.52, HOOD_CHAN_CROWN + HOOD_CHAN_R - HOOD_CHAN_DEPTH)
        )
        body = boolean_difference(body, hood_chan)
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
    # superellipse_side_loft is wound INWARD; flip the inner loft's faces so the
    # subtraction actually carves the shell hollow instead of a silent no-op.
    inner = MeshGeometry(
        vertices=list(inner.vertices),
        faces=[(f[0], f[2], f[1]) for f in inner.faces],
    )
    return boolean_difference(outer, inner)


# Shared wheel/tire geometry: low-profile black tire + dark multi-spoke turbine.
_TIRE_GEOM = TireGeometry(
    WHEEL_R,
    WHEEL_W,
    inner_radius=0.218,
    carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.04),
    sidewall=TireSidewall(style="rounded", bulge=0.04),
)
_WHEEL_GEOM = WheelGeometry(
    0.222,
    0.205,
    rim=WheelRim(inner_radius=0.196, flange_height=0.012, flange_thickness=0.006),
    hub=WheelHub(
        radius=0.090,
        width=0.10,
        cap_style="protruding",
        bolt_pattern=BoltPattern(count=5, circle_diameter=0.082, hole_diameter=0.008),
    ),
    # Pull the spoke disc forward (low dish) and flush with the hub cap so the
    # face is one plane: hub + spoke roots + rim read as a single connected
    # turbine, with no recessed shadow moat ringing the centre.
    face=WheelFace(dish_depth=0.004, front_inset=0.004, window_depth=0.030),
    # Many thin radial spokes; the slots stay narrow (small window_radius) so the
    # solid spoke wedges between them are wide and visibly root into the hub cap.
    spokes=WheelSpokes(style="straight", count=15, thickness=0.020, window_radius=0.034),
    bore=WheelBore(style="round", diameter=0.030),
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="koenigsegg_regera")

    ice_blue = model.material("ice_blue", rgba=(0.61, 0.73, 0.82, 1.0))
    carbon = model.material("carbon", rgba=(0.06, 0.06, 0.07, 1.0))
    model.material("glass_dark", rgba=(0.07, 0.08, 0.10, 1.0))  # used by QC by name
    # Smoked but see-through glazing for the windshield + side/rear windows.
    glass_tint = model.material("glass_tint", rgba=(0.10, 0.12, 0.15, 0.36))
    dark_alloy = model.material("dark_alloy", rgba=(0.14, 0.14, 0.16, 1.0))
    rubber = model.material("rubber", rgba=(0.04, 0.04, 0.045, 1.0))
    amber = model.material("amber", rgba=(0.85, 0.50, 0.06, 1.0))
    red_tail = model.material("tail_red", rgba=(0.62, 0.04, 0.05, 1.0))
    lens_pale = model.material("lens_pale", rgba=(0.84, 0.88, 0.92, 1.0))
    interior_dk = model.material("interior_dark", rgba=(0.10, 0.10, 0.11, 1.0))
    axle_steel = model.material("axle_steel", rgba=(0.66, 0.67, 0.70, 1.0))

    # ------------------------------------------------------------------ body
    body = model.part("body")
    body.visual(_save("lower_body.obj", _lower_body_mesh()), material=ice_blue, name="lower_body")
    # Glasshouse = three thin shells sharing seam rails (seamless windshield /
    # roof / rear window); the roof reads as a thin body-color skin.
    body.visual(_save("roof.obj", _glass_shell(ROOF_SECTIONS)), material=ice_blue, name="greenhouse")
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

    # Cabin interior: a single FLAT thin floor panel flush with the carved body
    # floor pan (top ~z=0.46), spanning the whole cabin so the footwell reads as
    # one flat floor (no raised slab / step on the outer side of the seats).
    body.visual(
        Box((1.26, 1.46, 0.05)),
        origin=Origin(xyz=(0.0, -0.02, 0.435)),
        material=interior_dk,
        name="cabin_floor",
    )
    # Seats sit ON the flat floor (base at the floor top ~0.46).
    for sx, side in ((0.30, "left"), (-0.30, "right")):
        body.visual(
            Box((0.40, 0.18, 0.30)),
            origin=Origin(xyz=(sx, -0.40, 0.61)),
            material=interior_dk,
            name=f"seat_back_{side}",
        )
        body.visual(
            Box((0.40, 0.36, 0.05)),
            origin=Origin(xyz=(sx, -0.18, 0.475)),
            material=interior_dk,
            name=f"seat_base_{side}",
        )
    # Firewall/bulkhead sealing the BACK of the cabin so the
    # hollow interior is closed off from the engine bay and not seen through.
    body.visual(
        Box((1.16, 0.05, 0.34)),
        origin=Origin(xyz=(0.0, -0.78, 0.63)),
        material=interior_dk,
        name="cabin_bulkhead",
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
        material=carbon,
        name="steering_column",
    )
    sw_accent = model.material("sw_accent", rgba=(0.80, 0.82, 0.86, 1.0))
    steer_wheel = model.part("steering_wheel")
    _RW = 0.150
    _RH = 0.120
    _RT = 0.026
    _RD = 0.045
    for _sy, _tag in ((_RH, "top"), (-_RH, "bot")):
        steer_wheel.visual(
            Box((2.0 * _RW + _RT, _RT, _RD)),
            origin=Origin(xyz=(0.0, _sy, 0.0)),
            material=carbon,
            name=f"sw_rim_{_tag}",
        )
    for _sx, _tag in ((_RW, "left"), (-_RW, "right")):
        steer_wheel.visual(
            Box((_RT + 0.018, 2.0 * _RH - 0.010, _RD + 0.014)),
            origin=Origin(xyz=(_sx, 0.0, 0.0)),
            material=carbon,
            name=f"sw_grip_{_tag}",
        )
    steer_wheel.visual(
        Box((2.0 * _RW, 0.030, 0.020)),
        material=carbon,
        name="sw_spoke_cross",
    )
    steer_wheel.visual(
        Box((0.030, _RH, 0.020)),
        origin=Origin(xyz=(0.0, -_RH / 2.0, 0.0)),
        material=carbon,
        name="sw_spoke_low",
    )
    steer_wheel.visual(
        Cylinder(radius=0.045, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, -0.02)),
        material=carbon,
        name="sw_hub",
    )
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

    # --- Front splitter + lower intake (carbon) --------------------------------
    # Low carbon front lip/splitter wrapping the bottom of the nose.
    body.visual(
        Box((1.58, 0.40, 0.055)),
        origin=Origin(xyz=(0.0, 2.04, 0.115), rpy=(0.12, 0.0, 0.0)),
        material=carbon,
        name="front_splitter",
    )
    # Central lower intake mouth (dark opening in the nose).
    body.visual(
        Box((0.74, 0.12, 0.17)),
        origin=Origin(xyz=(0.0, 2.17, 0.30)),
        material=carbon,
        name="front_intake",
    )
    # Carbon air-curtain blades raked into the lower front corners.
    for s in (1.0, -1.0):
        side = "left" if s > 0 else "right"
        body.visual(
            Box((0.10, 0.22, 0.30)),
            origin=Origin(xyz=(s * 0.66, 2.06, 0.30), rpy=(0.0, 0.0, s * 0.20)),
            material=carbon,
            name=f"air_curtain_{side}",
        )

    # --- Hook / comma LED headlights -------------------------------------------
    # The Regera headlight is a CURVED 'comma/hook': a bright LED strip starting
    # high near the hood edge that sweeps DOWN-and-OUTBOARD around the front
    # fender corner. Built as a row of small lens segments following that curve,
    # set into a dark carbon housing, with an amber turn marker at the tail.
    # APPROACH B -- CONTINUOUS LIGHT-GUIDE COMMA.
    # The Regera headlight reads as ONE smooth glowing comma stroke: a rounded
    # bright HEAD high & inboard, sweeping DOWN-and-OUTBOARD into a thinning TAIL.
    # We trace that single hooked curve with a DENSE chain of many small,
    # heavily overlapping bright cylinders (lens_pale) so it reads as one
    # continuous luminous light-guide line -- NOT a stack of discrete lamps.
    # The line sits on a dark recessed teardrop housing; a couple of round
    # projector accents anchor the head, and an amber lamp closes the tail.
    #
    # Smooth hook curve in the fender-corner box (x in [0.49,0.72], z in
    # [0.45,0.66]). Parameter u in [0,1] runs HEAD(inboard,high) -> TAIL
    # (outboard,low). The light-guide rake matches the fender corner.
    import math as _m

    def _comma_point(u: float) -> tuple[float, float]:
        # x sweeps inboard->outboard; the fuller HEAD (round lamps) sits inboard
        # and LOW near the nose, and the tail RISES up-and-outboard toward the
        # wheel arch -- the Regera headlight slants OUTWARD-up, it must NOT dive
        # down-inboard. z rises with a gentle convex crown.
        x = 0.498 + 0.220 * (u ** 0.85)
        z = 0.486 + 0.176 * u + 0.028 * _m.sin(_m.pi * u)
        return x, z

    _N_GUIDE = 24  # dense overlapping segments -> continuous glowing line
    _PROJECTOR_FACE = 1.928  # round head accents, recessed flush into the teardrop
    _LINE_FACE = 1.934  # light-guide line, sits proud of the dark recess

    for s in (1.0, -1.0):
        side = "left" if s > 0 else "right"

        # Dark recessed teardrop housing, raked onto the fender corner -- a slim
        # carbon trough that hugs the comma curve so the glowing stroke (not a
        # big black field) is the hero. A dense chain of small carbon segments
        # tracing just BELOW the light-guide gives the recessed teardrop a clean
        # tapering silhouette without swallowing the lit line.
        # Small dark backing disc tucked BEHIND the recess segment-chain (which is
        # the visible dark teardrop frame). No big crossing box -- it sits behind
        # the cluster, hidden, and only exists to satisfy the housing part name.
        body.visual(
            Cylinder(radius=0.055, length=0.035),
            origin=Origin(xyz=(s * 0.598, 1.900, 0.583), rpy=(pi / 2.0, 0.0, 0.0)),
            material=carbon,
            name=f"headlight_housing_{side}",
        )
        for k in range(_N_GUIDE):
            u = k / (_N_GUIDE - 1)
            cx, cz = _comma_point(u)
            br = 0.060 - 0.034 * u  # dark recess slightly fatter than the line
            body.visual(
                Cylinder(radius=br, length=0.045),
                origin=Origin(xyz=(s * cx, 1.915, cz), rpy=(pi / 2.0, 0.0, 0.0)),
                material=carbon,
                name=f"headlight_housing_seg_{side}_{k}",
            )

        # Continuous luminous light-guide line: dense chain of small overlapping
        # bright cylinders tracing the hook. Radius tapers head->tail so the
        # stroke thins as it flicks outboard, like a real comma.
        n = 0
        for k in range(_N_GUIDE):
            u = k / (_N_GUIDE - 1)
            cx, cz = _comma_point(u)
            r = 0.043 - 0.021 * u  # fuller at head -> thin at tail (stays joined)
            body.visual(
                Cylinder(radius=r, length=0.085),
                origin=Origin(xyz=(s * cx, _LINE_FACE, cz), rpy=(pi / 2.0, 0.0, 0.0)),
                material=lens_pale,
                name=f"headlight_{side}_{n}",
            )
            n += 1

        # Round LED projector accents anchoring the bright HEAD of the comma,
        # each seated as a RECESSED JEWEL BOWL: a dark interior cup behind a
        # smaller, pulled-back lens so the head reads as a recessed lamp in the
        # dark teardrop -- not a proud bright ball.
        _cup = 0
        for px, pz, pr in ((0.514, 0.500, 0.042), (0.548, 0.534, 0.034)):
            body.visual(
                Cylinder(radius=pr + 0.011, length=0.050),
                origin=Origin(xyz=(s * px, 1.908, pz), rpy=(pi / 2.0, 0.0, 0.0)),
                material=interior_dk,
                name=f"headlight_cup_{side}_{_cup}",
            )
            _cup += 1
            body.visual(
                Cylinder(radius=pr, length=0.078),
                origin=Origin(xyz=(s * px, _PROJECTOR_FACE, pz), rpy=(pi / 2.0, 0.0, 0.0)),
                material=lens_pale,
                name=f"headlight_{side}_{n}",
            )
            n += 1

        # Thin dark bezel ring around each projector head for a crisp jewel edge.
        for px, pz, pr in ((0.514, 0.500, 0.052), (0.548, 0.534, 0.043)):
            body.visual(
                Cylinder(radius=pr, length=0.066),
                origin=Origin(xyz=(s * px, 1.926, pz), rpy=(pi / 2.0, 0.0, 0.0)),
                material=carbon,
                name=f"headlight_{side}_{n}",
            )
            n += 1

        # Amber turn-signal lamp closing the outboard TAIL of the comma.
        _tx, _tz = _comma_point(1.0)
        body.visual(
            Cylinder(radius=0.020, length=0.080),
            origin=Origin(xyz=(s * (_tx + 0.004), _LINE_FACE, _tz - 0.002), rpy=(pi / 2.0, 0.0, 0.0)),
            material=amber,
            name=f"headlight_drl_{side}",
        )

    # --- Three 'ghost' hood vents (clean short slits) --------------------------
    for k, hx in enumerate((-0.10, 0.0, 0.10)):
        body.visual(
            Box((0.045, 0.12, 0.04)),
            origin=Origin(xyz=(hx, 1.67, 0.74)),
            material=carbon,
            name=f"ghost_vent_{k}",
        )

    # Carbon rocker sills between the wheel arches.
    for sx, side in ((0.84, "left"), (-0.84, "right")):
        body.visual(
            Box((0.11, 1.74, 0.12)),
            origin=Origin(xyz=(sx, 0.06, 0.17)),
            material=carbon,
            name=f"rocker_sill_{side}",
        )

    # Front and rear axle rods, hub to hub THROUGH the bored channels. The rear
    # rod is the rear wheels' spin axis; the front rod ends on the king-pins.
    for ay, axle_name in ((FRONT_AXLE_Y, "front_axle_bar"), (REAR_AXLE_Y, "rear_axle_bar")):
        body.visual(
            Cylinder(radius=AXLE_BAR_RADIUS, length=2.0 * HALF_TRACK),
            origin=Origin(xyz=(0.0, ay, WHEEL_R), rpy=(0.0, pi / 2.0, 0.0)),
            material=axle_steel,
            name=axle_name,
        )

    # Large Regera-style side scoop ahead of each rear wheel: a deep recessed
    # carbon throat with a body-color leading spear splitting the mouth, so the
    # flank is sculpted (not a flat slab).
    for sx, side in ((0.90, "left"), (-0.90, "right")):
        s = 1.0 if side == "left" else -1.0
        body.visual(
            Box((0.11, 0.58, 0.40)),
            origin=Origin(xyz=(sx, -0.64, 0.50), rpy=(0.0, 0.0, s * 0.12)),
            material=carbon,
            name=f"side_intake_{side}",
        )
        body.visual(
            Box((0.13, 0.05, 0.34)),
            origin=Origin(xyz=(s * 0.93, -0.62, 0.52), rpy=(0.0, 0.0, s * 0.18)),
            material=ice_blue,
            name=f"intake_spear_{side}",
        )

    # Small roof intake snorkel/scoop tucked behind the canopy (Regera scoop).
    body.visual(
        Box((0.16, 0.26, 0.07)),
        origin=Origin(xyz=(0.0, -0.84, 0.92), rpy=(0.22, 0.0, 0.0)),
        material=carbon,
        name="roof_scoop",
    )

    # Body-color rear clamshell sealing the engine bay top FLUSH with the body
    # (no dark recessed trough), with a slim CENTRAL carbon vent grille for the
    # engine cooling -- Regera-style, not a big black louvered panel.
    body.visual(
        Box((1.46, 0.92, 0.06)),
        origin=Origin(xyz=(0.0, -1.30, 0.80)),
        material=ice_blue,
        name="engine_deck",
    )
    for k in range(6):
        body.visual(
            Box((0.74, 0.075, 0.02)),
            origin=Origin(xyz=(0.0, -1.04 - 0.11 * k, 0.835), rpy=(0.10, 0.0, 0.0)),
            material=carbon,
            name=f"deck_louver_{k}",
        )

    # Slim TOP-MOUNTED active rear wing on short carbon supports near the deck
    # (Regera's thin top wing, not a tall twin-pylon spoiler).
    # Supports run from DOWN INTO the rear deck (z~0.67, embedded in the body) up
    # to the blade (z~1.0), so the wing is firmly rooted, not floating.
    for sx, side in ((0.56, "left"), (-0.56, "right")):
        body.visual(
            Box((0.05, 0.14, 0.36)),
            origin=Origin(xyz=(sx, -1.95, 0.84)),
            material=carbon,
            name=f"wing_support_{side}",
        )
    body.visual(
        Box((1.62, 0.30, 0.035)),
        origin=Origin(xyz=(0.0, -2.00, 0.99), rpy=(0.08, 0.0, 0.0)),
        material=carbon,
        name="wing_blade",
    )
    for sx, side in ((0.81, "left"), (-0.81, "right")):
        body.visual(
            Box((0.022, 0.26, 0.07)),
            origin=Origin(xyz=(sx, -2.00, 0.99)),
            material=carbon,
            name=f"wing_endplate_{side}",
        )

    # --- Tail: dark light-bar backing + full-width LED bar + badges + oval exhaust
    # Slim dark backing strip directly behind the LED bar (its top tucks just
    # under the bar so no dark stripe shows above it) to seat the light cleanly.
    body.visual(
        Box((1.26, 0.025, 0.085)),
        origin=Origin(xyz=(0.0, -2.248, 0.578)),
        material=interior_dk,
        name="tail_panel",
    )
    # FULL-WIDTH slim red LED light bar (Regera signature). One continuous slim
    # bar sitting PROUD of the rounded tail face so it reads as a single clean
    # line spanning the whole width, with short angled tips that follow the
    # haunches and die into the bodywork at each end.
    BAR_Z = 0.600
    BAR_Y = -2.300
    body.visual(
        Box((1.04, 0.045, 0.050)),
        origin=Origin(xyz=(0.0, BAR_Y, BAR_Z)),
        material=red_tail,
        name="tail_light_bar",
    )
    for sx, side in ((1.0, "left"), (-1.0, "right")):
        body.visual(
            Box((0.36, 0.045, 0.048)),
            origin=Origin(xyz=(sx * 0.66, BAR_Y + 0.012, BAR_Z + 0.004), rpy=(0.0, 0.0, sx * 0.30)),
            material=red_tail,
            name=f"tail_light_wing_{side}",
        )
    # Pale "Koenigsegg" wordmark strip above the bar + "Regera" badge below.
    body.visual(
        Box((0.48, 0.018, 0.024)),
        origin=Origin(xyz=(0.0, BAR_Y + 0.006, BAR_Z + 0.070)),
        material=lens_pale,
        name="tail_wordmark",
    )
    body.visual(
        Box((0.30, 0.020, 0.036)),
        origin=Origin(xyz=(0.10, BAR_Y + 0.006, BAR_Z - 0.080)),
        material=lens_pale,
        name="rear_badge",
    )
    # Single large central round exhaust outlet with a bright chrome ring.
    # One chrome disc + one dark opening proud of it -> reads as ONE clean round
    # tailpipe (the earlier box+rounded-cap oval broke into a "twin ball" look).
    EXH_Z = 0.300
    EXH_Y = -2.400
    body.visual(
        Cylinder(radius=0.100, length=0.05),
        origin=Origin(xyz=(0.0, EXH_Y, EXH_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=axle_steel,
        name="exhaust_ring",
    )
    body.visual(
        Cylinder(radius=0.077, length=0.10),
        origin=Origin(xyz=(0.0, EXH_Y + 0.010, EXH_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=carbon,
        name="exhaust",
    )
    # Wide carbon diffuser with vertical strakes across the full lower tail.
    body.visual(
        Box((1.48, 0.26, 0.18)),
        origin=Origin(xyz=(0.0, -2.20, 0.145)),
        material=carbon,
        name="rear_diffuser",
    )
    for k, fx in enumerate((-0.62, -0.42, -0.21, 0.0, 0.21, 0.42, 0.62)):
        # skip the central strakes that would clash with the exhaust opening
        if abs(fx) < 0.18:
            continue
        body.visual(
            Box((0.026, 0.30, 0.165)),
            origin=Origin(xyz=(fx, -2.215, 0.118)),
            material=carbon,
            name=f"diffuser_fin_{k}",
        )

    body.inertial = Inertial.from_geometry(
        Box((2.05, 4.56, 1.11)),
        mass=1590.0,
        origin=Origin(xyz=(0.0, 0.0, 0.55)),
    )

    # -------------------------------------------------------- dihedral doors
    hx, hy, hz = DOOR_HINGE

    def make_door(side: str):
        s = 1.0 if side == "left" else -1.0
        door = model.part(f"door_{side}")

        # Door skin loft authored in body frame, shifted into the local hinge
        # frame (hinge sits at the door's front-lower corner).
        # Thin door skin that sits FLUSH against the rounded flank (not a fat
        # bulging slab) -- narrow width so its outer face hugs the body surface.
        skin_sections = [
            (0.84, 0.21, 0.72, 0.07),
            (0.55, 0.18, 0.74, 0.09),
            (0.15, 0.18, 0.74, 0.09),
            (-0.30, 0.20, 0.72, 0.07),
        ]
        skin = superellipse_side_loft(skin_sections, exponents=2.6, segments=40)
        door.visual(
            _save(f"door_{side}_skin.obj", skin.translate(0.0, -hy, -hz)),
            material=ice_blue,
            name="door_skin",
        )

        glass_sections = [
            (0.74, 0.73, 0.82, 0.08),
            (0.46, 0.74, 0.97, 0.10),
            (0.08, 0.74, 1.00, 0.10),
            (-0.26, 0.74, 0.90, 0.08),
        ]
        glass_loft = superellipse_side_loft(glass_sections, exponents=2.6, segments=36)
        door.visual(
            _save(f"door_{side}_glass.obj", glass_loft.translate(-s * 0.26, -hy, -hz)),
            material=glass_tint,
            name="door_glass",
        )

        # Beltline trim at the window sill, bridging the outer skin to the inboard
        # window glass so the door + window read as ONE connected panel (not a
        # floating window). Authored at link z~0.27 -> world z~0.71.
        door.visual(
            Box((0.32, 1.02, 0.04)),
            origin=Origin(xyz=(-s * 0.10, -0.55, 0.27)),
            material=carbon,
            name="door_beltline",
        )
        door.visual(
            Box((0.04, 0.14, 0.03)),
            origin=Origin(xyz=(s * 0.10, -0.92, -0.08)),
            material=carbon,
            name="door_handle",
        )
        # Body-color side mirror on the door's front-top corner.
        door.visual(
            Box((0.12, 0.05, 0.04)),
            origin=Origin(xyz=(s * 0.11, -0.18, 0.06), rpy=(0.0, -s * 0.5, 0.0)),
            material=ice_blue,
            name="mirror_stalk",
        )
        door.visual(
            Box((0.06, 0.15, 0.10)),
            origin=Origin(xyz=(s * 0.18, -0.20, 0.10)),
            material=ice_blue,
            name="mirror_head",
        )

        door.inertial = Inertial.from_geometry(
            Box((0.20, 1.10, 0.85)),
            mass=32.0,
            origin=Origin(xyz=(0.0, -0.45, -0.12)),
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
            material=dark_alloy,
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
    # Inertial-only carrier links at each front wheel center. The body steers the
    # knuckle about the vertical king-pin; the wheel spins off the knuckle, so
    # steering and spin compose cleanly.
    def make_knuckle(name: str):
        k = model.part(name)
        k.inertial = Inertial.from_geometry(Box((0.10, 0.10, 0.22)), mass=6.0)
        return k

    knuckle_fl = make_knuckle("steer_knuckle_front_left")
    knuckle_fr = make_knuckle("steer_knuckle_front_right")

    # ----------------------------------------------------------- articulations
    # Dihedral synchro-helix doors: revolute about a forward/outward-canted
    # near-vertical axis at the front-lower door corner; +q swings the door OUT
    # and UP.
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

    # Front steering: revolute about the vertical (Z) king-pin, pivoting the
    # knuckle (and the wheel it carries) in place.
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

    # Wheel spin: continuous about lateral X. Front wheels spin off their
    # steering knuckle (so the spin axle swings with the steer angle); rear
    # wheels spin directly off the body.
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
    for _selem in ("sw_hub", "sw_spoke_cross", "sw_spoke_low"):
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
                    reason="Dihedral door seats flush in the body door aperture; thin embed is intentional.",
                )

    # --- Hero features present and legible -----------------------------------
    vis_names = {v.name for v in body.visuals}
    ctx.check(
        "lofted rounded body + bubble greenhouse present (not a box)",
        {"lower_body", "greenhouse"} <= vis_names,
        details=f"body visuals={sorted(vis_names)}",
    )
    ctx.check(
        "windshield and rear window glass present",
        {"windshield", "rear_window"} <= vis_names,
    )
    ctx.check(
        "hook/comma LED headlights: multi-segment sweep + housing + amber, both sides",
        {
            "headlight_housing_left",
            "headlight_housing_right",
            "headlight_drl_left",
            "headlight_drl_right",
        }
        <= vis_names
        and sum(1 for v in body.visuals if v.name.startswith("headlight_left_")) >= 4
        and sum(1 for v in body.visuals if v.name.startswith("headlight_right_")) >= 4,
    )
    ctx.check(
        "three ghost hood vents on the centerline",
        sum(1 for v in body.visuals if v.name.startswith("ghost_vent_")) == 3,
    )
    ctx.check(
        "carbon front splitter + intake + air curtains",
        {"front_splitter", "front_intake", "air_curtain_left", "air_curtain_right"} <= vis_names,
    )
    ctx.check(
        "carbon rocker sills both sides",
        {"rocker_sill_left", "rocker_sill_right"} <= vis_names,
    )
    ctx.check(
        "scalloped side intakes ahead of rear wheels (both sides)",
        {"side_intake_left", "side_intake_right"} <= vis_names,
    )
    ctx.check(
        "roof intake scoop behind the canopy",
        "roof_scoop" in vis_names,
    )
    ctx.check(
        "louvered engine deck (>=5 louvers)",
        sum(1 for v in body.visuals if v.name.startswith("deck_louver_")) >= 5,
    )
    ctx.check(
        "slim top-mounted wing on supports",
        {"wing_blade", "wing_support_left", "wing_support_right"} <= vis_names,
    )
    ctx.check(
        "full-width tail light bar + single central exhaust + diffuser",
        {"tail_light_bar", "exhaust", "rear_diffuser"} <= vis_names,
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

    # --- Rounded body: front fender drapes over the front wheel ---------------
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
    # Rounded, not a sharp wedge: the hood line is fairly level (nose not far
    # below the rear deck the way the Diablo wedge was).
    nose_top = max(z for (_x, y, z) in _lower_body_mesh().vertices if y > 1.95)
    deck_top = max(z for (_x, y, z) in _lower_body_mesh().vertices if -1.6 < y < -0.9)
    ctx.check(
        "rounded profile: nose sheetmetal is only gently below the rear deck",
        0.0 < deck_top - nose_top < 0.45,
        details=f"nose_top={nose_top:.3f}, deck_top={deck_top:.3f}",
    )

    # --- Scale sanity ---------------------------------------------------------
    bb = ctx.part_world_aabb(body)
    assert bb is not None
    lo, hi = bb
    ctx.check("car length ~4.56 m", 4.3 <= hi[1] - lo[1] <= 4.8, details=f"L={hi[1] - lo[1]:.3f}")
    ctx.check("car width ~2.05 m", 1.9 <= hi[0] - lo[0] <= 2.2, details=f"W={hi[0] - lo[0]:.3f}")
    ctx.check("car height ~1.11 m", 1.0 <= hi[2] <= 1.22, details=f"H={hi[2]:.3f}")

    # --- Slim wing rides above the deck on supports --------------------------
    wing = ctx.part_element_world_aabb(body, elem="wing_blade")
    support = ctx.part_element_world_aabb(body, elem="wing_support_left")
    deck = ctx.part_element_world_aabb(body, elem="engine_deck")
    assert wing is not None and support is not None and deck is not None
    ctx.check(
        "wing rides above the engine deck on its supports",
        wing[0][2] > deck[1][2] + 0.02 and support[1][2] >= wing[0][2] - 0.02,
        details=f"wing bottom z={wing[0][2]:.3f}, deck top z={deck[1][2]:.3f}",
    )

    # --- Glass reads darker than the ice-blue paint --------------------------
    mats = {m.name: m for m in object_model.materials}
    glass_rgb = sum(mats["glass_dark"].rgba[:3])
    body_rgb = sum(mats["ice_blue"].rgba[:3])
    ctx.check(
        "glass is much darker than the ice-blue body",
        glass_rgb < body_rgb - 0.8,
        details=f"glass={glass_rgb:.2f}, ice_blue={body_rgb:.2f}",
    )

    # --- Dihedral doors: canted near-vertical revolute, swing OUT and UP ------
    for hinge, door, side in ((hinge_l, door_l, "left"), (hinge_r, door_r, "right")):
        ax = tuple(hinge.axis)
        ctx.check(
            f"door_{side} hinge axis is dihedral (near-vertical canted, not scissor/gullwing)",
            abs(ax[2]) > 0.8 and abs(ax[0]) < 0.6,
            details=f"axis={ax}",
        )
        ml = hinge.motion_limits
        ctx.check(
            f"door_{side} hinge opens from closed",
            ml is not None and abs(ml.lower) < 1e-6 and 1.0 <= ml.upper <= 1.6,
            details=f"limits=({ml.lower}, {ml.upper})" if ml else "no limits",
        )
        ctx.expect_contact(body, door, contact_tol=0.05, name=f"door_{side} seated on body")

        rest = ctx.part_world_aabb(door)
        assert rest is not None
        with ctx.pose({hinge: DOOR_OPEN_MAX}):
            opened = ctx.part_world_aabb(door)
            assert opened is not None
        ctx.check(
            f"door_{side} swings UP (dihedral): top rises clearly when opened",
            opened[1][2] > rest[1][2] + 0.20,
            details=f"rest top z={rest[1][2]:.3f}, open top z={opened[1][2]:.3f}",
        )
        if side == "left":
            ctx.check(
                "door_left swings OUTBOARD (+X) as it opens",
                opened[1][0] > rest[1][0] + 0.10,
                details=f"rest max x={rest[1][0]:.3f}, open max x={opened[1][0]:.3f}",
            )
        else:
            ctx.check(
                "door_right swings OUTBOARD (-X) as it opens",
                opened[0][0] < rest[0][0] - 0.10,
                details=f"rest min x={rest[0][0]:.3f}, open min x={opened[0][0]:.3f}",
            )

    # --- Steering wheel: present, and TURNS about the column -----------------
    sw = object_model.get_part("steering_wheel")
    sw_names = {v.name for v in sw.visuals}
    ctx.check(
        "round steering wheel: rim, grips, hub, display + marker",
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
