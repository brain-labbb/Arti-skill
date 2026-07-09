from __future__ import annotations

# Bugatti Chiron style W16 hypercar: rounded, very WIDE and LOW, two-tone paint.
# Z-up world. Long axis of the car runs along +Y (nose at +Y), width along X
# (driver/left side at +X), up along +Z. Wheels touch z = 0.
#
# Forked from the Diablo wedge supercar: the proven STRUCTURE is reused -- the
# cabin is hollowed out of the solid body by boolean_difference (cabin cavity +
# door apertures cut clean THROUGH the flanks), each front wheel hangs off a
# steering knuckle so it both STEERS about a vertical king-pin AND spins off the
# knuckle, rear wheels spin off the body, and straight axle rods run hub-to-hub
# through bored channels. The body is re-skinned from a sharp yellow wedge into
# the Chiron's smooth, very WIDE, LOW rounded form.
#
# CHIRON IDENTITY (worked from knowledge, no reference photo):
#  - Proportions: extra-wide, low, rounded muscular haunches; soft surfaces.
#  - Two-tone paint split by the signature side "C-line": a polished SILVER /
#    aluminium UPPER and a deep BLUE LOWER, with a bright C-shaped chrome sweep
#    along each flank that wraps around the door (built as an applied trim arc).
#  - FRONT: the Bugatti horseshoe grille (tall rounded-trapezoid mesh, centred on
#    the nose) flanked by slim QUAD-element LED headlights + large lower side
#    intakes; an oval EB badge centred on the nose.
#  - REAR: a full-width thin LED light bar, a central stacked exhaust outlet, a
#    huge diffuser, a subtle integrated lip spoiler; an EB badge centred on tail.
#
# Primary articulation: BOTH doors swing UP-and-out on a dihedral revolute hinge
# (Chiron's signature butterfly/dihedral door) about a forward/outward-canted
# near-vertical axis at the front of the door. Secondary: all four wheels spin
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
# Global proportions (meters). Real Chiron: ~4.54 L x 2.04 W x 1.21 H,
# wheelbase 2.71, wheel radius ~0.355. The Chiron is notably WIDE and LOW with a
# fat rear track; keep the track wide and the haunches swollen.
# ----------------------------------------------------------------------------
WHEEL_R = 0.345
WHEEL_W = 0.33
# Wide track (the Chiron is one of the widest hypercars). Outer wheel face tucks
# just under the swollen fender crests.
HALF_TRACK = 0.86
FRONT_AXLE_Y = 1.34
REAR_AXLE_Y = -1.34

# Wheel-arch cavities carved out of the solid lower body at each wheel so the
# wheels sit in tight open wells. Each cutter is a lateral (X) cylinder hugging
# the wheel that only opens the OUTBOARD flank.
ARCH_RADIUS = 0.385
# (x_center_sign*track, y_center, inboard_wall_abs_x, outboard_wall_abs_x)
WHEEL_ARCHES = (
    (HALF_TRACK, FRONT_AXLE_Y, 0.54, 1.12),
    (-HALF_TRACK, FRONT_AXLE_Y, 0.54, 1.12),
    (HALF_TRACK, REAR_AXLE_Y, 0.60, 1.14),
    (-HALF_TRACK, REAR_AXLE_Y, 0.60, 1.14),
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
# (z ~0.84), otherwise a solid lid of lower body sits over the seats and buries
# them. Front edge stops at y~0.66 so the cowl/scuttle under the windshield base
# stays solid.
CABIN_HALF_X = 0.62
CABIN_Y = (-0.82, 0.66)
CABIN_Z = (0.45, 0.84)
DOOR_APERTURE_X = (0.52, 1.10)  # inboard (into cabin) .. outboard (through flank)
DOOR_APERTURE_Y = (-0.30, 0.62)
# Door-opening bottom raised to the CABIN FLOOR level (~0.45) so the door sill is
# FLUSH with the floor -- no step between the cabin floor and the door opening.
DOOR_APERTURE_Z = (0.45, 0.76)

# Dihedral / butterfly door hinge (left; right mirrors x), at the FRONT-LOWER
# corner of the door. The axis is a near-vertical king-pin canted FORWARD and a
# touch OUTWARD so one rotation swings the door OUT and UP (Chiron butterfly),
# unlike the Diablo's lateral-dominant scissor axis.
DOOR_HINGE = (0.88, 0.86, 0.44)
# Near-vertical king-pin canted FORE-AFT: rotating the left door about it swings
# it OUT (+X) and UP; the right door uses the fully negated axis so it mirrors
# (OUT -X and UP) under the same positive opening angle.
_DH = (0.0, -0.42, 0.91)
_DN = sqrt(_DH[0] * _DH[0] + _DH[1] * _DH[1] + _DH[2] * _DH[2])
DOOR_AXIS_LEFT = (_DH[0] / _DN, _DH[1] / _DN, _DH[2] / _DN)
DOOR_AXIS_RIGHT = (-_DH[0] / _DN, -_DH[1] / _DN, -_DH[2] / _DN)
DOOR_OPEN_MAX = 1.55

# Lower body side-profile rails: (y, z_min, z_max, width). Chiron form: smooth
# ROUNDED, very WIDE, low. Soft rounded nose, level hood, broad shoulders, big
# swollen muscular rear haunches (widest at the rear axle), short rounded tail.
# The car keeps a low, level beltline (no Diablo wedge rise).
LOWER_SECTIONS = [
    (2.27, 0.18, 0.40, 1.06),   # rounded low nose tip
    (2.12, 0.13, 0.52, 1.52),
    (1.90, 0.12, 0.66, 1.86),   # front fender shoulder rising
    (1.62, 0.12, 0.82, 2.02),   # peaked front fender crown (drapes over wheel)
    (1.34, 0.12, 0.85, 2.04),   # over front axle -- fender caps the well
    (1.04, 0.13, 0.80, 1.96),   # hood dips just below the fender crown
    (0.74, 0.13, 0.79, 1.90),   # cowl / scuttle
    (0.34, 0.14, 0.80, 1.88),   # door front / low beltline
    (-0.14, 0.14, 0.81, 1.94),  # door mid
    (-0.60, 0.12, 0.84, 2.06),  # rear haunch swelling
    (-1.04, 0.11, 0.87, 2.14),  # muscular rear haunch crown (widest, tallest)
    (-1.46, 0.11, 0.85, 2.10),  # over rear axle
    (-1.86, 0.13, 0.78, 1.92),  # rear deck taper
    (-2.12, 0.18, 0.70, 1.62),
    (-2.28, 0.24, 0.62, 1.34),  # rounded short tail
]

# Rounded canopy, built as THREE thin shells that share seam sections so
# windshield / roof / rear glass meet exactly. Front + rear are tinted glass; the
# middle is the body-color (silver) roof. The Chiron canopy is a low, wide,
# wraparound "bubble" with a black roof band.
_SEAM_FRONT = (0.36, 0.62, 1.07, 1.18)  # windshield top == roof leading edge
_SEAM_REAR = (-0.72, 0.60, 1.00, 0.98)  # roof trailing edge == rear window top
WINDSHIELD_SECTIONS = [
    (0.92, 0.60, 0.74, 1.48),
    (0.62, 0.62, 0.94, 1.32),
    _SEAM_FRONT,
]
ROOF_SECTIONS = [
    _SEAM_FRONT,
    (0.02, 0.64, 1.11, 1.14),  # low gentle dome over the seats
    (-0.38, 0.64, 1.07, 1.02),
    _SEAM_REAR,
]
REAR_WINDOW_SECTIONS = [
    _SEAM_REAR,
    (-1.00, 0.62, 0.84, 0.92),
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
# centre so the flanks read as raised fender crests -- the Chiron hood has two
# raised fender lines with a lower centre).
HOOD_CHAN_R = 1.00      # cylinder radius -> sets channel width
HOOD_CHAN_CROWN = 0.83  # hood crown z at the centre (where the carve starts)
HOOD_CHAN_DEPTH = 0.09  # how far the centre is dropped

_LOWER_BODY_CACHE = None


def _lower_body_mesh():
    # Low exponent -> rounded, soft superellipse cross-sections (organic Chiron
    # body, not the angular wedge). Then carve open wheel arches, axle channels,
    # the hollow cabin and the two door apertures, and a shallow hood channel.
    # Cached (reused by visual export + QC).
    global _LOWER_BODY_CACHE
    if _LOWER_BODY_CACHE is None:
        body = superellipse_side_loft(LOWER_SECTIONS, exponents=2.2, segments=64)
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
        # Hood centre channel: the Chiron hood dips slightly in the middle
        # between two raised fender crests. The single-peak superellipse loft is
        # convex (highest at centre), so carve a wide shallow valley down the
        # hood centre with a big cylinder laid fore-aft.
        hood_chan = (
            CylinderGeometry(radius=HOOD_CHAN_R, height=1.20, radial_segments=56)
            .rotate_x(pi / 2.0)  # axis Z -> Y (fore-aft)
            .translate(0.0, 1.50, HOOD_CHAN_CROWN + HOOD_CHAN_R - HOOD_CHAN_DEPTH)
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


# Shared wheel/tire geometry: low-profile black tire + bright multi-spoke alloy.
# The Chiron's signature wheel is a many-spoke turbine/"Y-spoke" alloy.
_TIRE_GEOM = TireGeometry(
    WHEEL_R,
    WHEEL_W,
    inner_radius=0.224,
    carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.04),
    sidewall=TireSidewall(style="rounded", bulge=0.04),
)
_WHEEL_GEOM = WheelGeometry(
    0.228,
    0.220,
    rim=WheelRim(inner_radius=0.200, flange_height=0.012, flange_thickness=0.006),
    hub=WheelHub(
        radius=0.075,
        width=0.10,
        cap_style="domed",
        bolt_pattern=BoltPattern(count=5, circle_diameter=0.080, hole_diameter=0.009),
    ),
    # Pull the spoke disc forward (low dish) and flush with the hub cap so the
    # face reads as one bright connected turbine, no recessed shadow moat.
    face=WheelFace(dish_depth=0.006, front_inset=0.004, window_depth=0.028),
    # Many thin radial spokes (Chiron turbine alloy).
    spokes=WheelSpokes(style="straight", count=16, thickness=0.018, window_radius=0.030),
    bore=WheelBore(style="round", diameter=0.030),
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bugatti_chiron")

    # --- Chiron two-tone palette --------------------------------------------
    # Polished SILVER / aluminium UPPER body + deep BLUE LOWER, split by the
    # signature C-line. A bright chrome trim traces the C sweep along each flank.
    silver = model.material("silver_upper", rgba=(0.74, 0.76, 0.80, 1.0))
    blue = model.material("bugatti_blue", rgba=(0.04, 0.13, 0.38, 1.0))
    chrome = model.material("chrome_trim", rgba=(0.88, 0.90, 0.93, 1.0))
    carbon = model.material("carbon", rgba=(0.05, 0.05, 0.06, 1.0))
    model.material("glass_dark", rgba=(0.07, 0.08, 0.10, 1.0))  # used by QC by name
    # Smoked but see-through glazing for the windshield + side/rear windows.
    glass_tint = model.material("glass_tint", rgba=(0.09, 0.11, 0.14, 0.36))
    dark_alloy = model.material("dark_alloy", rgba=(0.16, 0.16, 0.18, 1.0))
    bright_alloy = model.material("bright_alloy", rgba=(0.80, 0.82, 0.85, 1.0))
    rubber = model.material("rubber", rgba=(0.04, 0.04, 0.045, 1.0))
    amber = model.material("amber", rgba=(0.85, 0.50, 0.06, 1.0))
    red_tail = model.material("tail_red", rgba=(0.66, 0.03, 0.04, 1.0))
    lens_pale = model.material("lens_pale", rgba=(0.86, 0.90, 0.94, 1.0))
    led_white = model.material("led_white", rgba=(0.97, 0.98, 1.0, 1.0))
    interior_dk = model.material("interior_dark", rgba=(0.10, 0.10, 0.11, 1.0))
    axle_steel = model.material("axle_steel", rgba=(0.66, 0.67, 0.70, 1.0))

    # ------------------------------------------------------------------ body
    body = model.part("body")
    body.visual(_save("lower_body.obj", _lower_body_mesh()), material=silver, name="lower_body")
    # Glasshouse = three thin shells sharing seam rails (seamless windshield /
    # roof / rear window); the roof reads as a thin silver skin.
    body.visual(_save("roof.obj", _glass_shell(ROOF_SECTIONS)), material=silver, name="greenhouse")
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

    # --- TWO-TONE lower body: deep BLUE lower cladding ------------------------
    # The Chiron's defining look is a clean two-tone split. The lofted lower
    # body is silver; we drape a thin BLUE lower-body cladding skin over its
    # bottom half (a second, slightly-inset side-loft, capped at the C-line
    # height) so the lower flanks + nose read deep blue and the upper reads
    # polished silver. The split line is where this blue skin tops out.
    BLUE_TOP = 0.58  # C-line split height on the flank
    blue_sections = [
        (y, zmin, min(zmax, BLUE_TOP), w + 0.012)
        for (y, zmin, zmax, w) in LOWER_SECTIONS
    ]
    blue_skin = superellipse_side_loft(blue_sections, exponents=2.2, segments=64)
    # Carve the SAME wheel arches (a touch larger) out of the blue cladding so it
    # does not drape over and clip the tires -- it was a solid skin covering the
    # wheel openings (was only masked with allow_overlap before).
    for ax, ay, inboard, outboard in WHEEL_ARCHES:
        sign = 1.0 if ax > 0 else -1.0
        arch = (
            CylinderGeometry(radius=ARCH_RADIUS + 0.02, height=outboard - inboard, radial_segments=32)
            .rotate_y(pi / 2.0)
            .translate(sign * (inboard + outboard) / 2.0, ay, WHEEL_R)
        )
        blue_skin = boolean_difference(blue_skin, arch)
    body.visual(
        _save("blue_lower.obj", blue_skin),
        material=blue,
        name="blue_lower_cladding",
    )

    # --- Signature C-LINE chrome sweep ---------------------------------------
    # The Chiron's hero flank feature: one bold polished "C" that starts high at
    # the A-pillar base, sweeps DOWN and BACK low along the door, then curls UP
    # and AROUND the side air intake behind the door -- a backwards-C wrapping the
    # door. We trace that C as ONE smooth CONTINUOUS chrome RIBBON: short round
    # tube segments laid end-to-end ALONG the curve's local tangent (not flat
    # discs facing outboard, which scallop into beads). Each segment's round body
    # follows the C and overlaps its neighbours, so the flank reads a single clean
    # polished ribbon. The curve rides the lower/mid flank (z ~0.40 .. 0.74), well
    # below the greenhouse, hugging the door + intake.
    import math as _m

    def _cline_point(u: float) -> tuple[float, float]:
        # The C OPENS toward the FRONT: its two tips point forward (one high near
        # the A-pillar/door top, one low near the rocker), and its curved spine
        # bulges REARWARD, wrapping the side air intake behind the door. y is
        # fore-aft (+ forward), z is height.
        #   u: 0 -> bottom-front tip, 0.5 -> rear belly, 1 -> top-front tip.
        ang = _m.pi * (-0.50 + 1.0 * u)        # -90deg .. +90deg
        cy_back = -0.62                          # belly fore-aft (at the intake)
        # Arc centred lower on the flank so the whole C rides the MID/LOW flank
        # (a Chiron-correct lower-flank sweep) and its top-front tip stays clear
        # below the door beltline trim (z>=0.69), avoiding a trim/beltline clash.
        cz = 0.500                               # arc centre height on the flank
        ry = 0.62                                # depth of the C (fore-aft reach)
        rz = 0.150                               # half-height of the C
        y = cy_back + ry * (1.0 - _m.cos(ang))   # tips forward, belly rearward
        z = cz + rz * _m.sin(ang)
        return y, z

    # Many short segments, each spanning the gap to the next sample with a small
    # overlap, oriented along the local tangent -> one smooth continuous ribbon.
    _N_C = 72
    _CL_R = 0.026   # ribbon half-thickness (round tube)
    fx = 0.95       # flank x just outboard of the door/flank surface
    for s in (1.0, -1.0):
        side = "left" if s > 0 else "right"
        for k in range(_N_C - 1):
            u0 = k / (_N_C - 1)
            u1 = (k + 1) / (_N_C - 1)
            y0, z0 = _cline_point(u0)
            y1, z1 = _cline_point(u1)
            my, mz = (y0 + y1) / 2.0, (z0 + z1) / 2.0
            dy, dz = (y1 - y0), (z1 - z0)
            seg = _m.hypot(dy, dz)
            # Cylinder default axis is +Z; rotate it in the Y-Z plane to point
            # along the (dy, dz) tangent. Add a small overlap so segments fuse.
            roll = _m.atan2(dy, dz)  # angle of tangent from +Z toward +Y
            body.visual(
                Cylinder(radius=_CL_R, length=seg + 0.020),
                origin=Origin(xyz=(s * fx, my, mz), rpy=(-roll, 0.0, 0.0)),
                material=chrome,
                name=f"cline_{side}_{k}",
            )

    # Cabin interior: a single FLAT thin floor panel flush with the carved body
    # floor pan, spanning the whole cabin so the footwell reads as one flat floor.
    body.visual(
        Box((1.26, 1.48, 0.05)),
        origin=Origin(xyz=(0.0, -0.04, 0.435)),
        material=interior_dk,
        name="cabin_floor",
    )
    # Seats sit ON the flat floor (base at the floor top ~0.46).
    for sx, side in ((0.30, "left"), (-0.30, "right")):
        body.visual(
            Box((0.40, 0.18, 0.30)),
            origin=Origin(xyz=(sx, -0.42, 0.61)),
            material=interior_dk,
            name=f"seat_back_{side}",
        )
        body.visual(
            Box((0.40, 0.36, 0.05)),
            origin=Origin(xyz=(sx, -0.20, 0.475)),
            material=interior_dk,
            name=f"seat_base_{side}",
        )
    # Firewall/bulkhead sealing the BACK of the cabin so the hollow interior is
    # closed off from the engine bay and not seen through.
    body.visual(
        Box((1.16, 0.05, 0.36)),
        origin=Origin(xyz=(0.0, -0.80, 0.63)),
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

    # ===================================================================== FRONT
    # --- Bugatti HORSESHOE grille (center nose) ------------------------------
    # The hero front feature: a tall rounded-trapezoid mesh grille on the nose
    # centerline. Built as a recessed dark trough + a bright chrome surround
    # frame + a horizontal mesh of slats, narrower at the bottom (horseshoe).
    GRILLE_Y = 2.21
    # Big DARK recessed horseshoe field dominating the nose centre (the Bugatti
    # horseshoe is the hero front feature). Taller than wide. The dark field
    # itself reads as the horseshoe; a bright chrome U outlines it, fine dark
    # slats mesh it, and the EB badge caps the top.
    GR_CZ = 0.475       # grille centre height
    GR_HALF_W = 0.215   # horseshoe half-width
    GR_TOP = GR_CZ + 0.260
    GR_BOT = GR_CZ - 0.225
    # Dark recess field (the horseshoe void).
    body.visual(
        Box((2.0 * GR_HALF_W, 0.10, GR_TOP - GR_BOT),),
        origin=Origin(xyz=(0.0, GRILLE_Y - 0.05, (GR_TOP + GR_BOT) / 2.0)),
        material=carbon,
        name="grille_recess",
    )
    # Bright chrome horseshoe BEZEL: a clean, SOLID continuous chrome surround
    # ring around the dark field. Built from short bezel bars laid end-to-end
    # ALONG the U's local tangent, each of a constant slim cross-section and
    # overlapping its neighbour, so the surround reads as one smooth solid chrome
    # horseshoe (not a chain of chunky beads).
    import math as _mg

    def _shoe_point(v: float) -> tuple[float, float]:
        # v in [0,1]: 0 -> top-left, 0.5 -> bottom centre, 1 -> top-right.
        # Straight uprights on the upper two thirds, a rounded arc at the bottom.
        if v < 0.30:  # left upright (top -> down)
            t = v / 0.30
            x = -GR_HALF_W
            z = GR_TOP - t * (GR_TOP - (GR_BOT + 0.11))
        elif v > 0.70:  # right upright (down -> top)
            t = (v - 0.70) / 0.30
            x = GR_HALF_W
            z = (GR_BOT + 0.11) + t * (GR_TOP - (GR_BOT + 0.11))
        else:  # rounded bottom arc, left -> right
            t = (v - 0.30) / 0.40
            ang = _mg.pi * (1.0 - t)  # 180deg -> 0deg
            x = GR_HALF_W * _mg.cos(ang) * 0.92
            z = (GR_BOT + 0.11) - 0.11 * _mg.sin(ang)
        return x, z

    _N_SHOE = 56
    _BEZEL_W = 0.034  # slim constant bezel cross-section
    for k in range(_N_SHOE - 1):
        v0 = k / (_N_SHOE - 1)
        v1 = (k + 1) / (_N_SHOE - 1)
        x0, z0 = _shoe_point(v0)
        x1, z1 = _shoe_point(v1)
        mx, mz = (x0 + x1) / 2.0, (z0 + z1) / 2.0
        dx, dz = (x1 - x0), (z1 - z0)
        seg = _mg.hypot(dx, dz)
        # Bar default long axis +X; yaw it about Y so it points along the (dx, dz)
        # tangent in the X-Z plane. Overlap each segment so the bezel is solid.
        yaw = _mg.atan2(dz, dx)
        body.visual(
            Box((seg + 0.022, 0.085, _BEZEL_W)),
            origin=Origin(xyz=(mx, GRILLE_Y, mz), rpy=(0.0, -yaw, 0.0)),
            material=chrome,
            name=f"grille_frame_{k}",
        )
    # Keep stable named left/right/top frame elements (QC-friendly anchors).
    body.visual(
        Box((0.05, 0.085, 0.40)),
        origin=Origin(xyz=(-GR_HALF_W, GRILLE_Y, GR_CZ + 0.04)),
        material=chrome,
        name="grille_frame_left",
    )
    body.visual(
        Box((0.05, 0.085, 0.40)),
        origin=Origin(xyz=(GR_HALF_W, GRILLE_Y, GR_CZ + 0.04)),
        material=chrome,
        name="grille_frame_right",
    )
    body.visual(
        Box((2.0 * GR_HALF_W + 0.05, 0.085, 0.05)),
        origin=Origin(xyz=(0.0, GRILLE_Y, GR_TOP)),
        material=chrome,
        name="grille_frame_top",
    )
    # Fine horizontal mesh slats inside the horseshoe (narrower at the bottom).
    for k in range(11):
        gz = (GR_BOT + 0.02) + 0.040 * k
        gw = 0.20 + 0.020 * k  # narrower at the bottom -> horseshoe taper
        body.visual(
            Box((gw, 0.06, 0.012)),
            origin=Origin(xyz=(0.0, GRILLE_Y - 0.015, gz)),
            material=dark_alloy,
            name=f"grille_slat_{k}",
        )
    # Oval EB / Bugatti badge capping the TOP of the horseshoe (on the hood lip),
    # where the real Chiron badge sits. The badge cylinder runs fore-aft and its
    # REAR end is pressed back INTO the chrome grille_frame_top (which spans
    # y in [2.167, 2.252], z in [0.710, 0.760]) so the badge is seated against
    # the grille, not floating proud of the nose.
    body.visual(
        Cylinder(radius=0.055, length=0.090),
        origin=Origin(xyz=(0.0, GRILLE_Y + 0.045, GR_TOP + 0.020), rpy=(pi / 2.0, 0.0, 0.0)),
        material=red_tail,
        name="nose_badge",
    )
    body.visual(
        Cylinder(radius=0.036, length=0.100),
        origin=Origin(xyz=(0.0, GRILLE_Y + 0.045, GR_TOP + 0.020), rpy=(pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="nose_badge_center",
    )

    # --- Front splitter / lip (carbon) ---------------------------------------
    body.visual(
        Box((1.66, 0.42, 0.055)),
        origin=Origin(xyz=(0.0, 2.02, 0.115), rpy=(0.10, 0.0, 0.0)),
        material=carbon,
        name="front_splitter",
    )
    # --- Large lower SIDE intakes flanking the grille (Chiron) ---------------
    for s, side in ((1.0, "left"), (-1.0, "right")):
        body.visual(
            Box((0.30, 0.16, 0.26)),
            origin=Origin(xyz=(s * 0.60, 2.04, 0.30), rpy=(0.0, 0.0, s * 0.10)),
            material=carbon,
            name=f"front_side_intake_{side}",
        )
        # Bright trim blade splitting the intake.
        body.visual(
            Box((0.28, 0.04, 0.04)),
            origin=Origin(xyz=(s * 0.60, 2.06, 0.33), rpy=(0.0, 0.0, s * 0.10)),
            material=chrome,
            name=f"front_intake_blade_{side}",
        )

    # --- Slim QUAD-element LED headlights (Chiron signature) -----------------
    # The Chiron headlight is a slim sliver holding FOUR LED light elements
    # side-by-side ("four eyes"), set into a DARK housing on the fender corner.
    # For legibility against the silver fender, the housing is a deep dark recess
    # panel that FRAMES the four bright LED bars, which sit clearly proud of it.
    HL_FACE = 1.965     # bright LED lens plane (proud of the fender)
    HL_HOUSE = 1.935    # dark housing plane (recessed, frames the LEDs)
    HL_RAKE = (-0.16, 0.0, 0.0)
    for s in (1.0, -1.0):
        side = "left" if s > 0 else "right"
        rake = (HL_RAKE[0], 0.0, s * 0.06)
        # Deep DARK housing panel the four LEDs sit on -- a clear dark field on
        # the fender corner so the bright quad LEDs read against it.
        body.visual(
            Box((0.38, 0.07, 0.115)),
            origin=Origin(xyz=(s * 0.58, HL_HOUSE, 0.555), rpy=rake),
            material=carbon,
            name=f"headlight_housing_{side}",
        )
        # FOUR quad LED elements: bright vertical bars in a row, set proud of the
        # dark housing -- the signature Chiron "four eyes" cluster.
        for k in range(4):
            ex = 0.46 + 0.078 * k   # inboard -> outboard
            ez = 0.578 - 0.016 * k  # slight downward rake outboard
            body.visual(
                Box((0.026, 0.06, 0.082)),
                origin=Origin(xyz=(s * ex, HL_FACE, ez), rpy=rake),
                material=led_white,
                name=f"headlight_{side}_led_{k}",
            )
        # Thin chrome eyebrow trim capping the top of the cluster.
        body.visual(
            Box((0.36, 0.05, 0.018)),
            origin=Origin(xyz=(s * 0.58, HL_FACE - 0.010, 0.612), rpy=rake),
            material=chrome,
            name=f"headlight_brow_{side}",
        )
        # Amber turn marker at the outboard tail of the cluster.
        body.visual(
            Box((0.024, 0.05, 0.05)),
            origin=Origin(xyz=(s * 0.755, HL_FACE, 0.520), rpy=(-0.18, 0.0, s * 0.06)),
            material=amber,
            name=f"headlight_drl_{side}",
        )

    # Carbon rocker sills between the wheel arches (blends with the blue lower).
    for sx, side in ((0.86, "left"), (-0.86, "right")):
        body.visual(
            Box((0.11, 1.76, 0.12)),
            origin=Origin(xyz=(sx, 0.04, 0.16)),
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

    # Large Chiron-style side air intake ahead of each rear wheel: a deep
    # recessed carbon throat with a bright leading spear (part of the C-line
    # NACA scoop), so the flank is sculpted (not a flat slab).
    for sx, side in ((0.92, "left"), (-0.92, "right")):
        s = 1.0 if side == "left" else -1.0
        body.visual(
            Box((0.11, 0.56, 0.42)),
            origin=Origin(xyz=(sx, -0.66, 0.52), rpy=(0.0, 0.0, s * 0.10)),
            material=carbon,
            name=f"side_intake_{side}",
        )
        body.visual(
            Box((0.13, 0.05, 0.36)),
            origin=Origin(xyz=(s * 0.95, -0.64, 0.54), rpy=(0.0, 0.0, s * 0.16)),
            material=chrome,
            name=f"intake_spear_{side}",
        )

    # NACA roof intake duct feeding the W16 (Chiron roof scoop slit).
    body.visual(
        Box((0.18, 0.30, 0.06)),
        origin=Origin(xyz=(0.0, -0.86, 0.95), rpy=(0.20, 0.0, 0.0)),
        material=carbon,
        name="roof_scoop",
    )

    # Body-color (silver) rear clamshell sealing the engine bay top FLUSH with
    # the body, with a CENTRAL carbon vent grille for the W16 cooling.
    body.visual(
        Box((1.48, 0.94, 0.06)),
        origin=Origin(xyz=(0.0, -1.32, 0.84)),
        material=silver,
        name="engine_deck",
    )
    for k in range(6):
        body.visual(
            Box((0.80, 0.075, 0.02)),
            origin=Origin(xyz=(0.0, -1.06 - 0.11 * k, 0.875), rpy=(0.10, 0.0, 0.0)),
            material=carbon,
            name=f"deck_louver_{k}",
        )

    # --- Subtle integrated rear lip spoiler (Chiron) -------------------------
    # The Chiron's rear spoiler sits low and integrated (not a tall twin-pylon
    # wing). A slim body-color blade on short carbon supports rooting into the
    # rear deck, with a carbon Gurney lip.
    for sx, side in ((0.60, "left"), (-0.60, "right")):
        body.visual(
            Box((0.05, 0.16, 0.22)),
            origin=Origin(xyz=(sx, -1.98, 0.82)),
            material=carbon,
            name=f"wing_support_{side}",
        )
    body.visual(
        Box((1.66, 0.30, 0.04)),
        origin=Origin(xyz=(0.0, -2.02, 0.93), rpy=(0.06, 0.0, 0.0)),
        material=silver,
        name="wing_blade",
    )
    body.visual(
        Box((1.66, 0.04, 0.05)),
        origin=Origin(xyz=(0.0, -2.14, 0.955), rpy=(0.30, 0.0, 0.0)),
        material=carbon,
        name="wing_gurney",
    )
    for sx, side in ((0.83, "left"), (-0.83, "right")):
        body.visual(
            Box((0.022, 0.26, 0.06)),
            origin=Origin(xyz=(sx, -2.02, 0.93)),
            material=carbon,
            name=f"wing_endplate_{side}",
        )

    # ====================================================================== TAIL
    # --- Rear valance: CLOSE the tail face -----------------------------------
    # The lofted body leaves an open rounded rear face (we'd otherwise see into
    # the hollow interior as a dark void). Cap it with a broad rounded body-color
    # panel right at the rear face (y ~ -2.30), then a two-tone band: silver upper
    # + carbon lower that blends into the diffuser. The exhaust + light bar mount
    # proud of this closed face.
    rear_cap_sections = [
        (-2.16, 0.18, 0.66, 1.88),
        (-2.26, 0.20, 0.64, 1.66),
        (-2.32, 0.22, 0.62, 1.30),
    ]
    rear_cap = superellipse_side_loft(rear_cap_sections, exponents=2.4, segments=48)
    body.visual(
        _save("rear_cap.obj", rear_cap),
        material=silver,
        name="rear_cap",
    )
    # Carbon lower band across the tail (blends the valance into the diffuser).
    body.visual(
        Box((1.50, 0.12, 0.22)),
        origin=Origin(xyz=(0.0, -2.255, 0.330)),
        material=carbon,
        name="rear_valance_lower",
    )
    # --- Full-width thin LED light bar (Chiron signature) --------------------
    # One continuous slim red LED bar spanning the whole tail width, sitting
    # proud of the rear face so it reads as a single clean line. The Chiron bar
    # follows the haunches and dies into the bodywork at each end.
    #
    # SEATING: rear_cap's rear face is at y=-2.320. Every tail trim element below
    # is given enough fore-aft depth that its FRONT face bites forward of -2.320
    # (i.e. INTO the rear_cap), so the whole tail cluster is one connected mesh
    # with the body -- nothing floats proud of the tail.
    #
    # Dark recessed backing panel: a single clean strip that beds INTO the cap and
    # carries the LED bar + badge, so the rear quarter reads as one tidy graphic
    # rather than a scatter of floating trims.
    body.visual(
        Box((1.46, 0.13, 0.115)),
        origin=Origin(xyz=(0.0, -2.300, 0.575)),
        material=interior_dk,
        name="tail_panel",
    )
    BAR_Z = 0.580
    BAR_Y = -2.360  # front face at -2.3825 bites into the tail_panel (front -2.365)
    body.visual(
        Box((1.30, 0.060, 0.052)),
        origin=Origin(xyz=(0.0, BAR_Y, BAR_Z)),
        material=red_tail,
        name="tail_light_bar",
    )
    # Tapered LED wings curling up into the haunches at each end; deep enough to
    # bite into the tail_panel so they are seated, not floating.
    for sx, side in ((1.0, "left"), (-1.0, "right")):
        body.visual(
            Box((0.34, 0.060, 0.050)),
            origin=Origin(xyz=(sx * 0.74, BAR_Y + 0.004, BAR_Z + 0.004), rpy=(0.0, 0.0, sx * 0.28)),
            material=red_tail,
            name=f"tail_light_wing_{side}",
        )
    # Oval EB / Bugatti tail badge centred just above the bar; its fore-aft
    # cylinder is pressed forward so its front end seats into the tail_panel, and
    # it sits low enough that its lower rim bites into the tail_panel top
    # (tail_panel z spans [0.5175, 0.6325]) -- seated, not floating above the bar.
    body.visual(
        Cylinder(radius=0.048, length=0.105),
        origin=Origin(xyz=(0.0, -2.330, BAR_Z + 0.082), rpy=(pi / 2.0, 0.0, 0.0)),
        material=red_tail,
        name="tail_badge",
    )
    body.visual(
        Cylinder(radius=0.030, length=0.120),
        origin=Origin(xyz=(0.0, -2.330, BAR_Z + 0.082), rpy=(pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="tail_badge_center",
    )

    # --- Central STACKED exhaust outlet (Chiron) -----------------------------
    # The Chiron's exhaust is a single tidy central cluster: a chrome surround
    # block holding a clean 2x3 stack of dark round pipe openings on the
    # centerline. The surround is seated forward into the rear cap / valance so
    # the whole cluster connects to the body instead of hanging behind it.
    EXH_Z = 0.330
    EXH_Y = -2.355  # surround front face ~-2.265 bites into rear_cap (rear -2.320)
    body.visual(
        Box((0.34, 0.18, 0.20)),
        origin=Origin(xyz=(0.0, EXH_Y, EXH_Z), rpy=(0.0, 0.0, 0.0)),
        material=axle_steel,
        name="exhaust_surround",
    )
    _e = 0
    for ex in (-0.085, 0.085):
        for ez in (EXH_Z - 0.060, EXH_Z, EXH_Z + 0.060):
            # Pipe openings: front end embeds in the surround, rear end proud of
            # it, so each pipe is one connected mesh with the cluster.
            body.visual(
                Cylinder(radius=0.040, length=0.16),
                origin=Origin(xyz=(ex, EXH_Y - 0.045, ez), rpy=(pi / 2.0, 0.0, 0.0)),
                material=carbon,
                name=f"exhaust_{_e}",
            )
            _e += 1
    # Keep a generic "exhaust" alias element so structural QC has a stable name;
    # seated into the surround on the centerline between the two pipe columns.
    body.visual(
        Cylinder(radius=0.030, length=0.14),
        origin=Origin(xyz=(0.0, EXH_Y - 0.02, EXH_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=carbon,
        name="exhaust",
    )

    # --- Huge rear DIFFUSER with vertical strakes (Chiron) -------------------
    body.visual(
        Box((1.62, 0.30, 0.24)),
        origin=Origin(xyz=(0.0, -2.18, 0.150)),
        material=carbon,
        name="rear_diffuser",
    )
    for k, fx in enumerate((-0.68, -0.46, -0.24, 0.24, 0.46, 0.68)):
        body.visual(
            Box((0.026, 0.34, 0.22)),
            origin=Origin(xyz=(fx, -2.20, 0.130)),
            material=carbon,
            name=f"diffuser_fin_{k}",
        )

    body.inertial = Inertial.from_geometry(
        Box((2.04, 4.54, 1.21)),
        mass=1995.0,
        origin=Origin(xyz=(0.0, 0.0, 0.55)),
    )

    # -------------------------------------------------------- dihedral doors
    hx, hy, hz = DOOR_HINGE

    def make_door(side: str):
        s = 1.0 if side == "left" else -1.0
        door = model.part(f"door_{side}")

        # Door skin loft authored in body frame, shifted into the local hinge
        # frame (hinge sits at the door's front-lower corner). Thin skin that
        # sits FLUSH against the rounded flank (silver upper tone).
        skin_sections = [
            (0.82, 0.21, 0.74, 0.07),
            (0.55, 0.18, 0.76, 0.09),
            (0.15, 0.18, 0.76, 0.09),
            (-0.30, 0.20, 0.74, 0.07),
        ]
        skin = superellipse_side_loft(skin_sections, exponents=2.6, segments=40)
        door.visual(
            _save(f"door_{side}_skin.obj", skin.translate(0.0, -hy, -hz)),
            material=silver,
            name="door_skin",
        )
        # Blue lower portion of the door (two-tone split carried onto the door).
        blue_skin_sections = [
            (0.82, 0.21, 0.50, 0.075),
            (0.55, 0.18, 0.52, 0.095),
            (0.15, 0.18, 0.52, 0.095),
            (-0.30, 0.20, 0.50, 0.075),
        ]
        blue_door = superellipse_side_loft(blue_skin_sections, exponents=2.6, segments=40)
        door.visual(
            _save(f"door_{side}_blue.obj", blue_door.translate(0.0, -hy, -hz)),
            material=blue,
            name="door_blue",
        )

        glass_sections = [
            (0.72, 0.75, 0.84, 0.08),
            (0.46, 0.76, 0.99, 0.10),
            (0.08, 0.76, 1.02, 0.10),
            (-0.26, 0.76, 0.92, 0.08),
        ]
        glass_loft = superellipse_side_loft(glass_sections, exponents=2.6, segments=36)
        door.visual(
            _save(f"door_{side}_glass.obj", glass_loft.translate(-s * 0.26, -hy, -hz)),
            material=glass_tint,
            name="door_glass",
        )

        # (The signature C-line is carried by the body flank, which visually
        # passes across the embedded door panel -- no separate door C-line, which
        # would read as a messy double line.)

        # Beltline trim at the window sill, bridging the outer skin to the
        # inboard window glass so door + window read as ONE connected panel.
        door.visual(
            Box((0.32, 1.02, 0.04)),
            origin=Origin(xyz=(-s * 0.10, -0.55, 0.27)),
            material=carbon,
            name="door_beltline",
        )
        # Flush pull-handle, mounted PROUD of the flank: its inboard face sits at
        # world |x| ~1.02, clear of the blue_lower_cladding outer surface
        # (~0.976 at this station) so it does not collide with the cladding.
        door.visual(
            Box((0.05, 0.16, 0.032)),
            origin=Origin(xyz=(s * 0.165, -0.92, -0.08)),
            material=chrome,
            name="door_handle",
        )
        # Side mirror (silver) on the door's front-top corner.
        door.visual(
            Box((0.12, 0.05, 0.04)),
            origin=Origin(xyz=(s * 0.11, -0.18, 0.06), rpy=(0.0, -s * 0.5, 0.0)),
            material=silver,
            name="mirror_stalk",
        )
        door.visual(
            Box((0.07, 0.16, 0.10)),
            origin=Origin(xyz=(s * 0.19, -0.20, 0.10)),
            material=silver,
            name="mirror_head",
        )

        door.inertial = Inertial.from_geometry(
            Box((0.20, 1.10, 0.85)),
            mass=34.0,
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
            material=bright_alloy,
            name="rim",
        )
        # Bright caliper accent peeking through the spokes (Chiron blue/red).
        w.visual(
            Box((0.04, 0.16, 0.10)),
            origin=Origin(xyz=(outboard_sign * -0.02, 0.0, -0.20)),
            material=red_tail,
            name="caliper",
        )
        w.inertial = Inertial.from_geometry(
            Cylinder(radius=WHEEL_R, length=WHEEL_W),
            mass=24.0,
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
    # Dihedral / butterfly doors: revolute about a forward/outward-canted
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
        for elem in ("tire", "rim", "caliper"):
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
        # The blue lower cladding drapes over the same flank the wheels tuck into.
        # The caliper sits inboard, deep inside the carved wheel well; the cladding
        # is a thin skin over the SAME solid lower body whose arch the caliper is
        # already (allowed) seated in, so the cladding necessarily wraps over it
        # exactly as lower_body does -- this is the identical intentional seating.
        for elem in ("tire", "rim", "caliper"):
            ctx.allow_overlap(
                body,
                w,
                elem_a="blue_lower_cladding",
                elem_b=elem,
                reason="Blue lower cladding is a thin skin over the same body whose wheel arch the caliper tucks into; it wraps the well exactly as lower_body does.",
            )
    # The blue lower cladding is a thin skin draped over the silver lower body's
    # lower half -- it is intentionally coincident with lower_body / sills /
    # C-line / intakes on the flank.
    for elem in ("lower_body",):
        ctx.allow_overlap(
            body,
            body,
            elem_a="blue_lower_cladding",
            elem_b=elem,
            reason="Blue two-tone cladding is a thin skin over the lower half of the silver body.",
        )
    # Door panels seat flush into the body door aperture; the inner half of the
    # skin / blue / glass / beltline shelf is intentionally embedded in the flank.
    for door, side in ((door_l, "left"), (door_r, "right")):
        for shell in ("lower_body", "blue_lower_cladding", "greenhouse", f"rocker_sill_{side}", "windshield"):
            for delem in ("door_skin", "door_blue", "door_glass", "door_beltline", "mirror_stalk"):
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
        "lofted rounded body + greenhouse present (not a box)",
        {"lower_body", "greenhouse"} <= vis_names,
        details=f"body visuals={sorted(vis_names)}",
    )
    ctx.check(
        "two-tone: blue lower cladding draped on the silver body",
        "blue_lower_cladding" in vis_names,
    )
    ctx.check(
        "windshield and rear window glass present",
        {"windshield", "rear_window"} <= vis_names,
    )
    # Bugatti horseshoe grille: recess + chrome frame + a stack of mesh slats.
    ctx.check(
        "Bugatti horseshoe grille: recess + chrome frame + mesh slats",
        {"grille_recess", "grille_frame_left", "grille_frame_right", "grille_frame_top"} <= vis_names
        and sum(1 for v in body.visuals if v.name.startswith("grille_slat_")) >= 5,
    )
    ctx.check(
        "oval EB badge on the nose and on the tail",
        {"nose_badge", "tail_badge"} <= vis_names,
    )
    # Slim quad-element LED headlights: 4 LEDs per side + housing + amber DRL.
    ctx.check(
        "slim quad-element LED headlights: 4 LEDs/side + housing + amber, both sides",
        {
            "headlight_housing_left",
            "headlight_housing_right",
            "headlight_drl_left",
            "headlight_drl_right",
        }
        <= vis_names
        and sum(1 for v in body.visuals if v.name.startswith("headlight_left_led_")) == 4
        and sum(1 for v in body.visuals if v.name.startswith("headlight_right_led_")) == 4,
    )
    ctx.check(
        "large lower side intakes flanking the grille",
        {"front_side_intake_left", "front_side_intake_right"} <= vis_names,
    )
    ctx.check(
        "front splitter present",
        "front_splitter" in vis_names,
    )
    # Signature C-line: a dense chrome sweep along each flank.
    ctx.check(
        "signature C-line chrome sweep on both flanks (>=10 segs each)",
        sum(1 for v in body.visuals if v.name.startswith("cline_left_")) >= 10
        and sum(1 for v in body.visuals if v.name.startswith("cline_right_")) >= 10,
    )
    ctx.check(
        "carbon rocker sills both sides",
        {"rocker_sill_left", "rocker_sill_right"} <= vis_names,
    )
    ctx.check(
        "deep side intakes ahead of rear wheels (both sides)",
        {"side_intake_left", "side_intake_right"} <= vis_names,
    )
    ctx.check(
        "roof intake scoop behind the canopy",
        "roof_scoop" in vis_names,
    )
    ctx.check(
        "vented engine deck (>=5 louvers)",
        sum(1 for v in body.visuals if v.name.startswith("deck_louver_")) >= 5,
    )
    ctx.check(
        "subtle integrated rear spoiler on supports",
        {"wing_blade", "wing_support_left", "wing_support_right"} <= vis_names,
    )
    # Full-width tail bar + central stacked exhaust + huge diffuser.
    ctx.check(
        "full-width tail LED bar + central stacked exhaust + diffuser",
        {"tail_light_bar", "exhaust", "rear_diffuser"} <= vis_names
        and sum(1 for v in body.visuals if v.name.startswith("exhaust_")) >= 4,
    )
    ctx.check(
        "front and rear axle shafts present",
        {"front_axle_bar", "rear_axle_bar"} <= vis_names,
    )
    for door, side in ((door_l, "left"), (door_r, "right")):
        dnames = {v.name for v in door.visuals}
        ctx.check(
            f"door_{side} has silver skin, blue lower, tinted glass, handle, mirror",
            {"door_skin", "door_blue", "door_glass", "door_handle", "mirror_head"} <= dnames,
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
    # Rounded, low, level (not a sharp Diablo wedge): nose only gently below deck.
    nose_top = max(z for (_x, y, z) in _lower_body_mesh().vertices if y > 1.95)
    deck_top = max(z for (_x, y, z) in _lower_body_mesh().vertices if -1.6 < y < -0.9)
    ctx.check(
        "rounded profile: nose sheetmetal is only gently below the rear deck",
        0.0 < deck_top - nose_top < 0.55,
        details=f"nose_top={nose_top:.3f}, deck_top={deck_top:.3f}",
    )

    # --- Scale sanity: WIDE and low Chiron ------------------------------------
    bb = ctx.part_world_aabb(body)
    assert bb is not None
    lo, hi = bb
    L = hi[1] - lo[1]
    W = hi[0] - lo[0]
    H = hi[2]
    ctx.check("car length ~4.54 m", 4.3 <= L <= 4.8, details=f"L={L:.3f}")
    ctx.check("car is WIDE ~2.04 m", 1.95 <= W <= 2.3, details=f"W={W:.3f}")
    ctx.check("car height ~1.21 m (low)", 1.0 <= H <= 1.35, details=f"H={H:.3f}")
    ctx.check("Chiron is wide relative to its height (W > 1.7*H)", W > 1.7 * H, details=f"W={W:.3f}, H={H:.3f}")

    # --- Subtle spoiler rides above the deck on supports ---------------------
    wing = ctx.part_element_world_aabb(body, elem="wing_blade")
    support = ctx.part_element_world_aabb(body, elem="wing_support_left")
    deck = ctx.part_element_world_aabb(body, elem="engine_deck")
    assert wing is not None and support is not None and deck is not None
    ctx.check(
        "spoiler rides above the engine deck on its supports",
        wing[0][2] > deck[1][2] + 0.01 and support[1][2] >= wing[0][2] - 0.03,
        details=f"wing bottom z={wing[0][2]:.3f}, deck top z={deck[1][2]:.3f}",
    )

    # --- Two-tone + glass color checks ---------------------------------------
    mats = {m.name: m for m in object_model.materials}
    glass_rgb = sum(mats["glass_dark"].rgba[:3])
    body_rgb = sum(mats["silver_upper"].rgba[:3])
    ctx.check(
        "glass is much darker than the silver body",
        glass_rgb < body_rgb - 0.8,
        details=f"glass={glass_rgb:.2f}, silver={body_rgb:.2f}",
    )
    # The two tones are clearly distinct: silver upper is much brighter than blue.
    silver_rgb = sum(mats["silver_upper"].rgba[:3])
    blue_rgb = sum(mats["bugatti_blue"].rgba[:3])
    ctx.check(
        "two-tone: silver upper is clearly brighter than the deep blue lower",
        silver_rgb > blue_rgb + 0.6,
        details=f"silver={silver_rgb:.2f}, blue={blue_rgb:.2f}",
    )
    # The blue is genuinely blue (B channel dominates).
    bl = mats["bugatti_blue"].rgba
    ctx.check(
        "lower tone is a deep blue (blue channel dominant)",
        bl[2] > bl[0] + 0.15 and bl[2] > bl[1] + 0.15,
        details=f"blue rgba={bl}",
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
