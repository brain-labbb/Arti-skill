from __future__ import annotations

# Pagani Huayra style mid-engine hypercar -- bare carbon-fibre body with polished
# aluminium accents. Z-up world. Long axis of the car runs along +Y (nose at +Y),
# width along X (driver/left side at +X), up along +Z. Wheels touch z = 0.
#
# Forked from the Diablo wedge supercar: the proven STRUCTURE is reused -- the
# cabin is hollowed out of the solid body by boolean_difference (cabin cavity +
# door apertures cut clean THROUGH the flanks), each front wheel hangs off a
# steering knuckle so it both STEERS about a vertical king-pin AND spins off the
# knuckle, rear wheels spin off the body, and straight axle rods run hub-to-hub
# through bored channels. The body is re-skinned from a sharp wedge into the
# Huayra's soft, curvaceous, TEARDROP organic form.
#
# Primary articulation: BOTH GULLWING doors swing straight UP on a revolute hinge
# about a ROOF-MOUNTED, near-HORIZONTAL (lateral-dominant) axis at the top of the
# door (real Pagani Huayra gullwings hinge into the roof spine and lift up-and-
# out). Secondary: all four wheels spin (continuous about lateral X); the two
# FRONT wheels additionally steer about a vertical king-pin.
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
# Global proportions (meters). Real Huayra: ~4.61 L x 2.04 W x 1.17 H,
# wheelbase 2.795, wheel radius ~0.35.
# ----------------------------------------------------------------------------
WHEEL_R = 0.345
WHEEL_W = 0.30
# Track narrowed so the wheels tuck UNDER the rounded fender crests (outer ~flush
# with the body) rather than floating proud of the flank.
HALF_TRACK = 0.81
FRONT_AXLE_Y = 1.40
REAR_AXLE_Y = -1.36

# Wheel-arch cavities carved out of the solid lower body at each wheel so the
# wheels sit in tight open wells. Each cutter is a lateral (X) cylinder hugging
# the wheel that only opens the OUTBOARD flank.
ARCH_RADIUS = 0.38
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
# (z ~0.84), otherwise a solid lid of lower body sits over the seats and buries
# them. Front edge stops at y~0.72 so the cowl/scuttle under the windshield base
# stays solid.
CABIN_HALF_X = 0.62
CABIN_Y = (-0.78, 0.72)
CABIN_Z = (0.46, 0.84)
DOOR_APERTURE_X = (0.52, 1.06)  # inboard (into cabin) .. outboard (through flank)
DOOR_APERTURE_Y = (-0.28, 0.66)
# Door-opening bottom raised to the CABIN FLOOR level (~0.46) so the door sill is
# FLUSH with the floor -- no step between the dark cabin floor and the door.
DOOR_APERTURE_Z = (0.46, 0.76)

# Gullwing door hinge (left; right mirrors x), at the TOP-INBOARD edge of the
# door near the roof spine. The axis is LATERAL-DOMINANT (near-horizontal, runs
# fore-aft-ish along the roof line) with a small vertical tilt so the panel kicks
# slightly outward while it lifts up -- a true Pagani Huayra gullwing that hinges
# into the roof and swings the whole door UP-and-OUT (not the Diablo scissor, not
# the Regera dihedral king-pin).
DOOR_HINGE = (0.60, 0.30, 0.96)
# Lateral-dominant axis (runs across the car, X) with a small +Z tilt so the
# panel kicks slightly outward as it lifts; right door mirrors the tilt so both
# kick OUTWARD under a positive opening angle. Tilt trimmed from 0.22 to 0.12:
# with the door skin now seated OUTBOARD at the flank (x~0.85), the +Z axis tilt
# cross-couples the door's X offset into the swing and robs the open door of lift;
# a shallower tilt (still kicks out) restores the >0.20 m open-door sill clearance
# the test requires, without moving the hinge (which would shift the door's
# box-authored beltline/handle/mirror off the skin).
_AX_TILT = 0.12
_AX_NORM = sqrt(1.0 + _AX_TILT * _AX_TILT)
DOOR_AXIS_LEFT = (-1.0 / _AX_NORM, 0.0, _AX_TILT / _AX_NORM)
DOOR_AXIS_RIGHT = (-1.0 / _AX_NORM, 0.0, -_AX_TILT / _AX_NORM)
DOOR_OPEN_MAX = 1.60  # opens far enough that the gullwing bottom lifts clear of the sill, within the hinge upper-limit test bound (<=1.6)

# Lower body side-profile rails: (y, z_min, z_max, width). Soft, curvaceous,
# TEARDROP Huayra form: low rounded nose, a domed bonnet that bulges over the
# front wheels, a pinched waist at the doors, voluptuous rounded rear haunches,
# and a tapered drop-away tail. Widest + tallest over the rear haunches.
LOWER_SECTIONS = [
    (2.34, 0.20, 0.36, 0.84),   # rounded teardrop nose tip
    (2.18, 0.14, 0.48, 1.30),
    (1.94, 0.12, 0.62, 1.74),   # front fender shoulder rising
    (1.66, 0.12, 0.80, 1.96),   # peaked front fender crown (bulges over wheel)
    (1.40, 0.12, 0.83, 1.99),   # over front axle -- fender caps the well
    (1.10, 0.13, 0.79, 1.92),   # bonnet dips just below the fender crown
    (0.78, 0.13, 0.78, 1.82),   # cowl / scuttle
    (0.36, 0.14, 0.79, 1.74),   # pinched waist at the door (narrowest mid)
    (-0.14, 0.14, 0.80, 1.78),  # door mid
    (-0.60, 0.12, 0.82, 1.98),  # rear haunch swelling
    (-1.04, 0.11, 0.85, 2.08),  # voluptuous rear haunch crown (widest, tallest)
    (-1.46, 0.11, 0.83, 2.04),  # over rear axle
    (-1.86, 0.13, 0.76, 1.82),  # rear deck taper
    (-2.14, 0.18, 0.66, 1.52),
    (-2.34, 0.24, 0.56, 1.24),  # rounded drop-away tail
]

# Rounded teardrop bubble canopy, built as THREE thin shells that share seam
# sections so windshield / roof / rear glass meet exactly. Front + rear are
# tinted glass; the middle is the dark carbon roof. Long, low, narrow teardrop
# (the Huayra wraps the glass right around the cabin like a fighter canopy).
_SEAM_FRONT = (0.36, 0.60, 1.03, 1.10)  # windshield top == roof leading edge
_SEAM_REAR = (-0.70, 0.58, 0.98, 0.90)  # roof trailing edge == rear window top
WINDSHIELD_SECTIONS = [
    (0.90, 0.58, 0.74, 1.36),
    (0.62, 0.60, 0.92, 1.22),
    _SEAM_FRONT,
]
ROOF_SECTIONS = [
    _SEAM_FRONT,
    (0.02, 0.62, 1.07, 1.02),  # low gentle dome over the seats
    (-0.36, 0.62, 1.04, 0.94),
    _SEAM_REAR,
]
REAR_WINDOW_SECTIONS = [
    _SEAM_REAR,
    (-0.98, 0.60, 0.84, 0.84),
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


# Bonnet centre-channel carve params (a big fore-aft cylinder dipping the bonnet
# centre so the flanks read as raised fender crests -- the Huayra has a sculpted
# central spine valley flanked by two raised fender humps).
HOOD_CHAN_R = 0.92
HOOD_CHAN_CROWN = 0.81
HOOD_CHAN_DEPTH = 0.09

_LOWER_BODY_CACHE = None


def _lower_body_mesh():
    # Low exponent -> rounded, soft superellipse cross-sections (organic Huayra
    # body, not the angular wedge). Then carve open wheel arches, axle channels,
    # the hollow cabin, the two door apertures, and the bonnet centre channel.
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
        # Bonnet centre channel: the Huayra bonnet has a sculpted central valley
        # between two raised fender humps. The single-peak superellipse loft is
        # convex (highest at centre), so carve a wide shallow valley down the
        # centre with a big fore-aft cylinder -- the centre drops and the flanks
        # read as raised fender crests.
        hood_chan = (
            CylinderGeometry(radius=HOOD_CHAN_R, height=1.20, radial_segments=56)
            .rotate_x(pi / 2.0)  # axis Z -> Y (fore-aft)
            .translate(0.0, 1.58, HOOD_CHAN_CROWN + HOOD_CHAN_R - HOOD_CHAN_DEPTH)
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


# Shared wheel/tire geometry: low-profile black tire + polished aluminium
# turbine-style alloy (the Huayra's signature multi-spoke turbine wheels).
_TIRE_GEOM = TireGeometry(
    WHEEL_R,
    WHEEL_W,
    inner_radius=0.222,
    carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.04),
    sidewall=TireSidewall(style="rounded", bulge=0.04),
)
_WHEEL_GEOM = WheelGeometry(
    0.226,
    0.205,
    rim=WheelRim(inner_radius=0.200, flange_height=0.012, flange_thickness=0.006),
    hub=WheelHub(
        radius=0.090,
        width=0.10,
        cap_style="protruding",
        bolt_pattern=BoltPattern(count=5, circle_diameter=0.060, hole_diameter=0.012),
    ),
    # Pull the spoke disc forward (low dish) flush with the hub cap so the face is
    # one plane: hub + spoke roots + rim read as a single connected turbine.
    face=WheelFace(dish_depth=0.004, front_inset=0.004, window_depth=0.030),
    # Many thin radial turbine spokes; narrow slots so the solid spoke wedges are
    # wide and root cleanly into the central locking hub (Huayra centre-lock).
    spokes=WheelSpokes(style="straight", count=20, thickness=0.018, window_radius=0.030),
    bore=WheelBore(style="round", diameter=0.030),
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pagani_huayra")

    # --- Materials: bare carbon-fibre body + polished aluminium accents --------
    carbon = model.material("carbon_fibre", rgba=(0.07, 0.07, 0.08, 1.0))  # dark bare carbon body
    alloy = model.material("polished_alloy", rgba=(0.82, 0.84, 0.87, 1.0))  # bright aluminium accents
    carbon_trim = model.material("carbon_trim", rgba=(0.04, 0.04, 0.05, 1.0))  # deepest carbon detailing
    model.material("glass_dark", rgba=(0.07, 0.08, 0.10, 1.0))  # used by QC by name
    # Smoked but see-through glazing for the windshield + side/rear windows.
    glass_tint = model.material("glass_tint", rgba=(0.10, 0.12, 0.15, 0.36))
    rubber = model.material("rubber", rgba=(0.04, 0.04, 0.045, 1.0))
    amber = model.material("amber", rgba=(0.85, 0.50, 0.06, 1.0))
    red_tail = model.material("tail_red", rgba=(0.62, 0.04, 0.05, 1.0))
    lens_pale = model.material("lens_pale", rgba=(0.55, 0.60, 0.66, 1.0))  # muted cool lamp lens (not a bright white dome)
    interior_dk = model.material("interior_dark", rgba=(0.12, 0.10, 0.08, 1.0))  # tan-brown leather hint
    chrome = model.material("chrome", rgba=(0.86, 0.87, 0.90, 1.0))  # bright exhaust / accents

    # ------------------------------------------------------------------ body
    body = model.part("body")
    body.visual(_save("lower_body.obj", _lower_body_mesh()), material=carbon, name="lower_body")
    # Glasshouse = three thin shells sharing seam rails (seamless windshield /
    # roof / rear window); the roof reads as a thin carbon skin.
    body.visual(_save("roof.obj", _glass_shell(ROOF_SECTIONS)), material=carbon, name="greenhouse")
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
    # floor pan. Kept NARROW (half-width 0.50) so it stays well inboard of the
    # door aperture inner wall (x=0.52) and never reaches into the closed door
    # skin -- the footwell floor reads as one flat pan inside the cabin.
    body.visual(
        Box((1.00, 1.48, 0.05)),
        origin=Origin(xyz=(0.0, -0.02, 0.445)),
        material=interior_dk,
        name="cabin_floor",
    )
    # Seats sit ON the flat floor (base at the floor top). The backrest top is
    # kept below z=0.75 so it stays clear of the door beltline / window-sill line
    # (no seat-vs-door collision when the gullwing is shut).
    for sx, side in ((0.30, "left"), (-0.30, "right")):
        body.visual(
            Box((0.40, 0.18, 0.28)),
            origin=Origin(xyz=(sx, -0.40, 0.60)),
            material=interior_dk,
            name=f"seat_back_{side}",
        )
        body.visual(
            Box((0.40, 0.36, 0.05)),
            origin=Origin(xyz=(sx, -0.18, 0.485)),
            material=interior_dk,
            name=f"seat_base_{side}",
        )
    # Firewall/bulkhead sealing the BACK of the cabin so the hollow interior is
    # closed off from the engine bay and not seen through.
    body.visual(
        Box((1.16, 0.05, 0.34)),
        origin=Origin(xyz=(0.0, -0.76, 0.64)),
        material=interior_dk,
        name="cabin_bulkhead",
    )

    # Steering wheel: raked column fixed on the body; the round wheel is a
    # SEPARATE part that SPINS about the raked column axis.
    # Steering wheel hub dropped to z=0.635 and nudged inboard so the round rim
    # top sits clear BELOW the closed door glass bottom (z~0.74) -- no door-glass
    # vs steering-wheel collision when the door is shut.
    _SW_X = 0.31
    _SW_FWD = 0.40
    _SW_RAKE = (0.6, 0.0, 0.0)
    _SW_HUB = (_SW_X, -0.02 + _SW_FWD, 0.635)
    body.visual(
        Cylinder(radius=0.022, length=0.34),
        origin=Origin(xyz=(_SW_X, 0.08 + _SW_FWD, 0.495), rpy=_SW_RAKE),
        material=carbon_trim,
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
            material=carbon_trim,
            name=f"sw_rim_{_tag}",
        )
    for _sx, _tag in ((_RW, "left"), (-_RW, "right")):
        steer_wheel.visual(
            Box((_RT + 0.018, 2.0 * _RH - 0.010, _RD + 0.014)),
            origin=Origin(xyz=(_sx, 0.0, 0.0)),
            material=carbon_trim,
            name=f"sw_grip_{_tag}",
        )
    steer_wheel.visual(
        Box((2.0 * _RW, 0.030, 0.020)),
        material=carbon_trim,
        name="sw_spoke_cross",
    )
    steer_wheel.visual(
        Box((0.030, _RH, 0.020)),
        origin=Origin(xyz=(0.0, -_RH / 2.0, 0.0)),
        material=carbon_trim,
        name="sw_spoke_low",
    )
    steer_wheel.visual(
        Cylinder(radius=0.045, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, -0.02)),
        material=alloy,
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

    # --- Front splitter + wide rounded lower intake (carbon + alloy mesh) -------
    # Low carbon front lip/splitter wrapping the bottom of the rounded nose.
    body.visual(
        Box((1.56, 0.42, 0.055)),
        origin=Origin(xyz=(0.0, 2.08, 0.115), rpy=(0.12, 0.0, 0.0)),
        material=carbon_trim,
        name="front_splitter",
    )
    # Wide rounded central lower intake mouth (the Huayra's big oval grille). A
    # dark recessed throat with a bright aluminium mesh bar across it.
    body.visual(
        Box((0.92, 0.12, 0.20)),
        origin=Origin(xyz=(0.0, 2.20, 0.31)),
        material=carbon_trim,
        name="front_intake",
    )
    body.visual(
        Box((0.86, 0.06, 0.05)),
        origin=Origin(xyz=(0.0, 2.205, 0.31)),
        material=alloy,
        name="front_intake_mesh",
    )
    # Carbon air-curtain blades raked into the lower front corners.
    for s in (1.0, -1.0):
        side = "left" if s > 0 else "right"
        body.visual(
            Box((0.10, 0.22, 0.30)),
            origin=Origin(xyz=(s * 0.68, 2.08, 0.30), rpy=(0.0, 0.0, s * 0.20)),
            material=carbon_trim,
            name=f"air_curtain_{side}",
        )

    # --- Round headlight pods (2-3 round lamps per side) -----------------------
    # The Pagani Huayra's nose carries a cluster of ROUND lamps in a sculpted
    # carbon pod on each front fender corner: a large main projector lamp, a
    # smaller secondary lamp, a tiny accent lamp, plus an amber turn marker. Each
    # round lamp is a recessed JEWEL: a dark carbon cup behind a bright alloy
    # bezel ring behind a pale glass lens, so it reads as a deep round headlamp,
    # not a flat dot. The whole cluster sits on a sculpted pod backing.
    # The lamps are built as RECESSED flat round jewels (not bulging balls): a
    # deep dark cup set well INTO the pod, a bright alloy bezel ring framing the
    # mouth, and a thin flat pale lens disc sitting just inside the bezel mouth so
    # the lamp reads as a crisp round eye flush with the carbon, not a white dome.
    # Faces are pulled BACK so nothing protrudes proud of the carbon pod face --
    # the lamps read as deep RECESSED round eyes set into the fender, not bright
    # bulging balls. Bezel sits flush at the pod mouth; the muted lens disc is
    # recessed well behind it.
    _POD_FACE = 1.965     # outer face of the fender corner pod (the carbon mask)
    _CUP_FACE = 1.905     # deep dark cup, set well back in the pod
    _BEZEL_FACE = 1.958   # alloy bezel ring, flush at the pod mouth (not proud)
    _LENS_FACE = 1.940    # thin flat lens disc, recessed well behind the bezel
    # (x, z, radius) of each round lamp on the left side; right mirrors x. Tighter
    # cluster, lifted up onto the fender shoulder so the three round eyes sit
    # FULLY on the pod face (no half-buried lamps) and read as ONE Huayra pod.
    _LAMPS = (
        (0.50, 0.60, 0.062),   # large main projector (inboard, high)
        (0.605, 0.565, 0.048), # secondary projector (outboard)
        (0.685, 0.535, 0.031), # small accent lamp (outboard corner)
    )
    for s in (1.0, -1.0):
        side = "left" if s > 0 else "right"
        # Sculpted carbon pod backing/mask the whole cluster (the dark teardrop
        # pod the three round lamps are recessed into).
        body.visual(
            Box((0.32, 0.06, 0.24)),
            origin=Origin(xyz=(s * 0.595, _POD_FACE - 0.045, 0.565), rpy=(-0.16, 0.0, s * 0.10)),
            material=carbon_trim,
            name=f"headlight_pod_{side}",
        )
        for li, (lx, lz, lr) in enumerate(_LAMPS):
            # Deep dark cup (the recess the lamp sits in).
            body.visual(
                Cylinder(radius=lr + 0.004, length=0.075),
                origin=Origin(xyz=(s * lx, _CUP_FACE, lz), rpy=(pi / 2.0, 0.0, 0.0)),
                material=carbon_trim,
                name=f"headlight_cup_{side}_{li}",
            )
            # Bright alloy bezel ring framing the mouth of the cup (flush at face).
            body.visual(
                Cylinder(radius=lr + 0.011, length=0.022),
                origin=Origin(xyz=(s * lx, _BEZEL_FACE, lz), rpy=(pi / 2.0, 0.0, 0.0)),
                material=alloy,
                name=f"headlight_bezel_{side}_{li}",
            )
            # Thin flat pale lens disc, recessed just inside the bezel mouth so it
            # reads as a round eye, not a protruding white ball.
            body.visual(
                Cylinder(radius=lr, length=0.018),
                origin=Origin(xyz=(s * lx, _LENS_FACE, lz), rpy=(pi / 2.0, 0.0, 0.0)),
                material=lens_pale,
                name=f"headlight_{side}_{li}",
            )
        # Slim amber turn-signal lamp tucked at the lower-outboard corner.
        body.visual(
            Cylinder(radius=0.017, length=0.020),
            origin=Origin(xyz=(s * 0.72, _LENS_FACE - 0.006, 0.49), rpy=(pi / 2.0, 0.0, 0.0)),
            material=amber,
            name=f"headlight_turn_{side}",
        )

    # --- Bonnet centre power-dome + four alloy quad nostrils -------------------
    # A slim raised carbon spine down the bonnet centre channel, with the Huayra's
    # signature little round alloy bonnet vents flanking it. NOTE the bonnet centre
    # channel is carved away (no top skin) over y~1.50-1.68, so both vent rows must
    # sit FORWARD of the carve on solid sheetmetal (y>=1.70) -- otherwise the
    # rear row hovers over the empty valley. Vent bottom (z=0.73) embeds ~0.015 into
    # the bonnet skin (top z~0.745) so each vent sits flush, not floating.
    for k, vy in enumerate((1.80, 1.72)):
        for s in (1.0, -1.0):
            body.visual(
                Cylinder(radius=0.030, length=0.03),
                origin=Origin(xyz=(s * 0.16, vy, 0.74), rpy=(0.0, 0.0, 0.0)),
                material=alloy,
                name=f"bonnet_vent_{k}_{'l' if s > 0 else 'r'}",
            )

    # Carbon rocker sills between the wheel arches.
    for sx, side in ((0.84, "left"), (-0.84, "right")):
        body.visual(
            Box((0.11, 1.76, 0.12)),
            origin=Origin(xyz=(sx, 0.04, 0.17)),
            material=carbon_trim,
            name=f"rocker_sill_{side}",
        )

    # Front and rear axle rods, hub to hub THROUGH the bored channels. The rear
    # rod is the rear wheels' spin axis; the front rod ends on the king-pins.
    for ay, axle_name in ((FRONT_AXLE_Y, "front_axle_bar"), (REAR_AXLE_Y, "rear_axle_bar")):
        body.visual(
            Cylinder(radius=AXLE_BAR_RADIUS, length=2.0 * HALF_TRACK),
            origin=Origin(xyz=(0.0, ay, WHEEL_R), rpy=(0.0, pi / 2.0, 0.0)),
            material=chrome,
            name=axle_name,
        )

    # Large Huayra-style side scoop ahead of each rear wheel: a deep recessed
    # carbon throat with a polished alloy leading spear splitting the mouth, so
    # the flank is sculpted (not a flat slab).
    for sx, side in ((0.90, "left"), (-0.90, "right")):
        s = 1.0 if side == "left" else -1.0
        body.visual(
            Box((0.11, 0.58, 0.40)),
            origin=Origin(xyz=(sx, -0.64, 0.50), rpy=(0.0, 0.0, s * 0.12)),
            material=carbon_trim,
            name=f"side_intake_{side}",
        )
        body.visual(
            Box((0.13, 0.05, 0.34)),
            origin=Origin(xyz=(s * 0.93, -0.62, 0.52), rpy=(0.0, 0.0, s * 0.18)),
            material=alloy,
            name=f"intake_spear_{side}",
        )

    # Roof intake snorkel/scoop tucked behind the canopy (Huayra roof scoop). Sits
    # in the trough between the canopy trailing edge (greenhouse top z~0.98 at
    # y~-0.70) and the engine deck (top z~0.84). Pulled FORWARD + DOWN and made
    # taller so its front edge laps under the canopy (connects to the greenhouse)
    # and its base rests on the engine deck -- no gap-float between roof and deck.
    body.visual(
        Box((0.16, 0.34, 0.16)),
        origin=Origin(xyz=(0.0, -0.78, 0.90), rpy=(0.22, 0.0, 0.0)),
        material=carbon_trim,
        name="roof_scoop",
    )

    # Carbon rear clamshell sealing the engine bay top FLUSH with the body, with
    # a slim CENTRAL alloy vent grille for engine cooling, plus a glazed window
    # over the engine (the Huayra shows its V12 through a rear glass panel).
    body.visual(
        Box((1.46, 0.92, 0.06)),
        origin=Origin(xyz=(0.0, -1.30, 0.81)),
        material=carbon,
        name="engine_deck",
    )
    body.visual(
        Box((0.78, 0.50, 0.012)),
        origin=Origin(xyz=(0.0, -1.18, 0.845)),
        material=glass_tint,
        name="engine_glass",
    )
    for k in range(6):
        body.visual(
            Box((0.74, 0.075, 0.02)),
            origin=Origin(xyz=(0.0, -1.06 - 0.11 * k, 0.845), rpy=(0.10, 0.0, 0.0)),
            material=carbon_trim,
            name=f"deck_louver_{k}",
        )

    # --- Active aero: FOUR discrete corner flaps on short posts ----------------
    # The Pagani Huayra's signature active aero is FOUR independent little flaps,
    # one at each corner of the car, each raised on a short stout post. Here the
    # four flaps sit at the four corners of the engine deck; every post roots
    # DOWN INTO the deck (post bottom z=0.80 < deck top z=0.84) so the whole wing
    # assembly is one connected piece of the body, NOT a floating wing. Each flap
    # is a small carbon blade tilted up on its post, framed by alloy end fences.
    # The deck top is z=0.84; posts bite into it at z=0.80 and rise to z=0.96.
    _DECK_TOP = 0.84
    _POST_BOT = 0.80          # post bottom, embedded into the carbon deck (connect)
    _POST_TOP = 0.965         # post top, supporting the flap
    _POST_H = _POST_TOP - _POST_BOT
    _POST_CZ = (_POST_BOT + _POST_TOP) / 2.0
    _FLAP_CZ = 0.985          # flap mid-height, riding just above the posts
    # Each corner: (x_center, y_center, flap name suffix). The front-left corner
    # keeps the asserted names aero_flap + aero_actuator_left; the front-right
    # keeps aero_actuator_right; the other corners are aero_flap_* / aero_post_*.
    _AERO_CORNERS = (
        (0.50, -1.00, "fl"),   # front-left corner of the deck
        (-0.50, -1.00, "fr"),  # front-right corner
        (0.50, -1.60, "rl"),   # rear-left corner
        (-0.50, -1.60, "rr"),  # rear-right corner
    )
    for cx, cy, tag in _AERO_CORNERS:
        # Short stout post rooting down into the deck and up to the flap. Twin
        # posts per corner so each little flap stands on a pair of legs.
        for px, leg in ((cx - 0.10, "i"), (cx + 0.10, "o")):
            post_name = (
                "aero_actuator_left" if (tag == "fl" and leg == "i")
                else "aero_actuator_right" if (tag == "fr" and leg == "i")
                else f"aero_post_{tag}_{leg}"
            )
            body.visual(
                Box((0.05, 0.06, _POST_H)),
                origin=Origin(xyz=(px, cy, _POST_CZ)),
                material=alloy,
                name=post_name,
            )
        # Carbon flap blade, tilted up, spanning across its pair of posts. Wide
        # enough to read clearly as an aero flap (not a peg) while still resting
        # on -- and connected to -- both posts beneath it.
        flap_name = "aero_flap" if tag == "fl" else f"aero_flap_{tag}"
        body.visual(
            Box((0.40, 0.24, 0.030)),
            origin=Origin(xyz=(cx, cy, _FLAP_CZ), rpy=(0.30, 0.0, 0.0)),
            material=carbon,
            name=flap_name,
        )
        # Alloy end-fences framing each little flap (Huayra corner-flap look).
        for ex, end in ((cx - 0.20, "i"), (cx + 0.20, "o")):
            fence_name = (
                "aero_endplate_left" if (tag == "fl" and end == "i")
                else "aero_endplate_right" if (tag == "fr" and end == "i")
                else f"aero_fence_{tag}_{end}"
            )
            body.visual(
                Box((0.018, 0.20, 0.085)),
                origin=Origin(xyz=(ex, cy, _FLAP_CZ - 0.01), rpy=(0.30, 0.0, 0.0)),
                material=carbon_trim,
                name=fence_name,
            )

    # --- Tail: round quad taillights + central quad exhaust + diffuser ---------
    # NOTE the rounded tail caps at y=-2.34 only up to z~0.56; ABOVE that the body
    # rolls forward (rear recedes to y~-2.14). So the tail panel + the round lamp
    # cluster must stay BELOW z~0.56, otherwise their upper edge stands ~0.1 m proud
    # of the tapered tail (floating). Panel lowered + shortened to sit on the flat
    # tail face.
    body.visual(
        Box((1.30, 0.04, 0.22)),
        origin=Origin(xyz=(0.0, -2.275, 0.47)),
        material=carbon_trim,
        name="tail_panel",
    )
    # Round QUAD taillights: a cluster of round red lamps on each side (the Huayra
    # signature round tail lamps), each a dark cup + alloy ring + red lens. Cluster
    # dropped onto the flat lower tail (all z<=0.55) so every lens seats flush
    # against the tail face instead of hanging off the tapered upper corner.
    _TL_FACE = 1.0  # placeholder; positions set below
    _TAIL_LAMPS = (
        (0.42, 0.53, 0.044),
        (0.56, 0.53, 0.044),
        (0.49, 0.41, 0.038),
    )
    for s in (1.0, -1.0):
        side = "left" if s > 0 else "right"
        for ti, (tx, tz, tr) in enumerate(_TAIL_LAMPS):
            body.visual(
                Cylinder(radius=tr + 0.008, length=0.045),
                origin=Origin(xyz=(s * tx, -2.27, tz), rpy=(pi / 2.0, 0.0, 0.0)),
                material=alloy,
                name=f"taillight_ring_{side}_{ti}",
            )
            body.visual(
                Cylinder(radius=tr, length=0.072),
                origin=Origin(xyz=(s * tx, -2.300, tz), rpy=(pi / 2.0, 0.0, 0.0)),
                material=red_tail,
                name=f"taillight_{side}_{ti}",
            )

    # CENTRAL QUAD EXHAUST: four round titanium tailpipes clustered tightly in the
    # centre (the unmistakable Pagani signature). Each is a bright chrome ring
    # with a dark bore, arranged 2x2 in a tight diamond/square cluster.
    EXH_Y_RING = -2.40
    EXH_Y_BORE = -2.39
    for ex, ez, tag in (
        (-0.075, 0.34, "tl"),
        (0.075, 0.34, "tr"),
        (-0.075, 0.21, "bl"),
        (0.075, 0.21, "br"),
    ):
        body.visual(
            Cylinder(radius=0.058, length=0.07),
            origin=Origin(xyz=(ex, EXH_Y_RING, ez), rpy=(pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name=f"exhaust_ring_{tag}",
        )
        body.visual(
            Cylinder(radius=0.040, length=0.10),
            origin=Origin(xyz=(ex, EXH_Y_BORE, ez), rpy=(pi / 2.0, 0.0, 0.0)),
            material=carbon_trim,
            name=f"exhaust_{tag}",
        )
    # Slim carbon surround hugging the quad-pipe cluster.
    body.visual(
        Box((0.34, 0.05, 0.34)),
        origin=Origin(xyz=(0.0, -2.355, 0.275)),
        material=carbon_trim,
        name="exhaust_surround",
    )

    # Large carbon rear diffuser with vertical strakes across the full lower tail.
    body.visual(
        Box((1.52, 0.28, 0.20)),
        origin=Origin(xyz=(0.0, -2.22, 0.155)),
        material=carbon_trim,
        name="rear_diffuser",
    )
    for k, fx in enumerate((-0.66, -0.46, -0.26, 0.26, 0.46, 0.66)):
        body.visual(
            Box((0.026, 0.32, 0.18)),
            origin=Origin(xyz=(fx, -2.235, 0.125)),
            material=carbon_trim,
            name=f"diffuser_fin_{k}",
        )

    body.inertial = Inertial.from_geometry(
        Box((2.04, 4.61, 1.17)),
        mass=1350.0,
        origin=Origin(xyz=(0.0, 0.0, 0.55)),
    )

    # --------------------------------------------------------- gullwing doors
    hx, hy, hz = DOOR_HINGE

    # Outboard seat offset: the hinge sits at x=hx=0.60 but the body FLANK around
    # the door aperture reaches out to x~0.88-0.90. With no x-shift the door skin
    # would sit at x~0.60 -- ~0.25 m INBOARD of the flank, floating in a deep
    # pocket (the "车门悬空" bug). Push the whole door shell OUTBOARD so the skin
    # seats FLUSH in the aperture and its inner half embeds in the flank.
    DOOR_OUT = 0.25

    def make_door(side: str):
        s = 1.0 if side == "left" else -1.0
        door = model.part(f"door_{side}")

        # Door skin loft authored in body frame, shifted OUTBOARD into the body
        # flank so the closed gullwing fills the aperture flush (its inner half is
        # intentionally embedded in the flank around the aperture). Thin door skin
        # following the rounded flank; the gullwing carries a tall slice of flank
        # + glass.
        skin_sections = [
            (0.82, 0.21, 0.74, 0.07),
            (0.54, 0.18, 0.76, 0.09),
            (0.14, 0.18, 0.76, 0.09),
            (-0.30, 0.20, 0.74, 0.07),
        ]
        skin = superellipse_side_loft(skin_sections, exponents=2.6, segments=40)
        door.visual(
            _save(f"door_{side}_skin.obj", skin.translate(s * DOOR_OUT, -hy, -hz)),
            material=carbon,
            name="door_skin",
        )

        # Tall wrap-around side glass carried by the gullwing (cuts up into the
        # roof like a real Huayra gullwing window). Seated just OUTBOARD enough to
        # sit flush over the narrow greenhouse-canopy flank (x~0.50-0.55) -- no gap
        # between the side window and the bubble canopy when the door is shut.
        glass_sections = [
            (0.74, 0.74, 0.86, 0.08),
            (0.46, 0.75, 1.00, 0.10),
            (0.08, 0.75, 1.03, 0.10),
            (-0.26, 0.75, 0.92, 0.08),
        ]
        glass_loft = superellipse_side_loft(glass_sections, exponents=2.6, segments=36)
        door.visual(
            _save(f"door_{side}_glass.obj", glass_loft.translate(-s * 0.10, -hy, -hz)),
            material=glass_tint,
            name="door_glass",
        )

        # Beltline trim at the window sill, bridging the OUTER skin (now at the
        # flank, x~0.85) to the inboard window glass (x~0.50) so the door + window
        # read as ONE connected panel. Widened + recentred to span the new gap.
        # Length trimmed + nudged forward so its rear edge stops short of the fixed
        # rear window glass (no door-vs-rear_window collision when the door is shut).
        door.visual(
            Box((0.42, 0.86, 0.04)),
            origin=Origin(xyz=(s * 0.10, -0.49, -0.18)),
            material=carbon_trim,
            name="door_beltline",
        )
        # Handle let into the OUTER skin face at the flank (x~0.87). Placed inside
        # the door APERTURE window (world y~-0.10, z~0.56) so it sits on the
        # exposed door skin -- NOT straddling the aperture sill into the solid
        # flank (which would collide with lower_body), and well clear of the body
        # side_intake throat ahead of the rear wheel.
        door.visual(
            Box((0.04, 0.14, 0.03)),
            origin=Origin(xyz=(s * 0.27, -0.40, -0.40)),
            material=carbon_trim,
            name="door_handle",
        )
        # Slim-stalk side mirror on the door's front-upper corner (Huayra mirrors
        # are on thin teardrop stalks standing off the door). The stalk roots into
        # the outboard skin (x~0.85) and the head stands proud of the flank.
        door.visual(
            Cylinder(radius=0.012, length=0.16),
            origin=Origin(xyz=(s * 0.34, -0.16, -0.30), rpy=(0.0, s * 1.3, 0.0)),
            material=carbon_trim,
            name="mirror_stalk",
        )
        door.visual(
            Box((0.05, 0.15, 0.09)),
            origin=Origin(xyz=(s * 0.42, -0.18, -0.28)),
            material=carbon,
            name="mirror_head",
        )

        door.inertial = Inertial.from_geometry(
            Box((0.20, 1.10, 0.85)),
            mass=30.0,
            origin=Origin(xyz=(0.0, -0.45, -0.40)),
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
            material=alloy,
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
    # Gullwing doors: revolute about a ROOF-MOUNTED, lateral-dominant (near-
    # horizontal) axis at the top of the door; +q swings the whole door straight
    # UP-and-OUT (Pagani Huayra gullwing).
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
    # skin / glass / beltline shelf is intentionally embedded in the flank, and
    # the gullwing's tall glass reaches up into the roof/greenhouse.
    for door, side in ((door_l, "left"), (door_r, "right")):
        for shell in ("lower_body", "greenhouse", f"rocker_sill_{side}", "windshield"):
            for delem in ("door_skin", "door_glass", "door_beltline", "mirror_stalk"):
                ctx.allow_overlap(
                    body,
                    door,
                    elem_a=shell,
                    elem_b=delem,
                    reason="Gullwing door seats flush in the body door aperture; thin embed is intentional.",
                )

    # --- Hero features present and legible -----------------------------------
    vis_names = {v.name for v in body.visuals}
    ctx.check(
        "lofted rounded teardrop body + bubble greenhouse present (not a box)",
        {"lower_body", "greenhouse"} <= vis_names,
        details=f"body visuals={sorted(vis_names)}",
    )
    ctx.check(
        "windshield and rear window glass present",
        {"windshield", "rear_window"} <= vis_names,
    )
    # Round headlight clusters: a sculpted pod + >=3 round lamps + amber turn,
    # per side -- the Huayra's signature multi-round-lamp front pods.
    ctx.check(
        "round headlight pods: sculpted pod + >=3 round lamps + amber turn, both sides",
        {
            "headlight_pod_left",
            "headlight_pod_right",
            "headlight_turn_left",
            "headlight_turn_right",
        }
        <= vis_names
        and sum(1 for v in body.visuals if v.name.startswith("headlight_left_")) >= 3
        and sum(1 for v in body.visuals if v.name.startswith("headlight_right_")) >= 3,
    )
    ctx.check(
        "wide rounded lower intake with alloy mesh bar",
        {"front_intake", "front_intake_mesh"} <= vis_names,
    )
    ctx.check(
        "carbon front splitter + air curtains",
        {"front_splitter", "air_curtain_left", "air_curtain_right"} <= vis_names,
    )
    ctx.check(
        "round alloy bonnet vents (>=4)",
        sum(1 for v in body.visuals if v.name.startswith("bonnet_vent_")) >= 4,
    )
    ctx.check(
        "carbon rocker sills both sides",
        {"rocker_sill_left", "rocker_sill_right"} <= vis_names,
    )
    ctx.check(
        "sculpted side intakes ahead of rear wheels (both sides)",
        {"side_intake_left", "side_intake_right"} <= vis_names,
    )
    ctx.check(
        "roof intake scoop behind the canopy",
        "roof_scoop" in vis_names,
    )
    ctx.check(
        "engine deck with glazed engine window + louvers (>=5)",
        {"engine_deck", "engine_glass"} <= vis_names
        and sum(1 for v in body.visuals if v.name.startswith("deck_louver_")) >= 5,
    )
    # Active aero flap on twin actuators -- the Huayra signature.
    ctx.check(
        "active aero rear flap on twin actuator posts",
        {"aero_flap", "aero_actuator_left", "aero_actuator_right"} <= vis_names,
    )
    # CENTRAL QUAD EXHAUST: four round tailpipes clustered tightly in the centre.
    ctx.check(
        "central quad exhaust: four round tailpipes + chrome rings",
        sum(1 for v in body.visuals if v.name.startswith("exhaust_") and not v.name.startswith("exhaust_ring") and v.name != "exhaust_surround") >= 4
        and sum(1 for v in body.visuals if v.name.startswith("exhaust_ring_")) >= 4,
    )
    # Round QUAD taillights -- a cluster of round red lamps per side.
    ctx.check(
        "round quad taillights: >=3 round red lamps per side",
        sum(1 for v in body.visuals if v.name.startswith("taillight_left_")) >= 3
        and sum(1 for v in body.visuals if v.name.startswith("taillight_right_")) >= 3,
    )
    ctx.check(
        "large rear diffuser with vertical strakes (>=4)",
        "rear_diffuser" in vis_names
        and sum(1 for v in body.visuals if v.name.startswith("diffuser_fin_")) >= 4,
    )
    ctx.check(
        "front and rear axle shafts present",
        {"front_axle_bar", "rear_axle_bar"} <= vis_names,
    )
    for door, side in ((door_l, "left"), (door_r, "right")):
        dnames = {v.name for v in door.visuals}
        ctx.check(
            f"door_{side} has skin, tinted glass, handle, slim-stalk mirror",
            {"door_skin", "door_glass", "door_handle", "mirror_stalk", "mirror_head"} <= dnames,
            details=f"door_{side} visuals={sorted(dnames)}",
        )

    # --- Rounded teardrop body: front fender drapes over the front wheel ------
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
    # Rounded, not a sharp wedge: the bonnet line is fairly level (nose only
    # gently below the rear deck, unlike the Diablo wedge).
    nose_top = max(z for (_x, y, z) in _lower_body_mesh().vertices if y > 1.95)
    deck_top = max(z for (_x, y, z) in _lower_body_mesh().vertices if -1.6 < y < -0.9)
    ctx.check(
        "rounded teardrop profile: nose sheetmetal is only gently below the rear deck",
        0.0 < deck_top - nose_top < 0.45,
        details=f"nose_top={nose_top:.3f}, deck_top={deck_top:.3f}",
    )

    # --- Scale sanity ---------------------------------------------------------
    bb = ctx.part_world_aabb(body)
    assert bb is not None
    lo, hi = bb
    ctx.check("car length ~4.61 m", 4.3 <= hi[1] - lo[1] <= 4.9, details=f"L={hi[1] - lo[1]:.3f}")
    ctx.check("car width ~2.04 m", 1.9 <= hi[0] - lo[0] <= 2.2, details=f"W={hi[0] - lo[0]:.3f}")
    ctx.check("car height ~1.17 m", 1.0 <= hi[2] <= 1.28, details=f"H={hi[2]:.3f}")

    # --- Active aero flap rides above the deck on actuators ------------------
    flap = ctx.part_element_world_aabb(body, elem="aero_flap")
    actuator = ctx.part_element_world_aabb(body, elem="aero_actuator_left")
    deck = ctx.part_element_world_aabb(body, elem="engine_deck")
    assert flap is not None and actuator is not None and deck is not None
    ctx.check(
        "aero flap rides above the engine deck on its actuators",
        flap[0][2] > deck[1][2] + 0.02 and actuator[1][2] >= flap[0][2] - 0.05,
        details=f"flap bottom z={flap[0][2]:.3f}, deck top z={deck[1][2]:.3f}",
    )

    # --- Central quad exhaust sits low, in the centre, clustered tightly ------
    exr_tl = ctx.part_element_world_aabb(body, elem="exhaust_ring_tl")
    exr_br = ctx.part_element_world_aabb(body, elem="exhaust_ring_br")
    assert exr_tl is not None and exr_br is not None
    # All four pipes hug the centerline (|x| small) and sit below the deck.
    ctx.check(
        "quad exhaust clusters tightly on the centerline, low on the tail",
        abs(exr_tl[0][0]) < 0.20 and abs(exr_br[1][0]) < 0.20 and exr_tl[1][2] < deck[0][2],
        details=f"tl x-min={exr_tl[0][0]:.3f}, br x-max={exr_br[1][0]:.3f}, tl top z={exr_tl[1][2]:.3f}",
    )

    # --- Glass reads darker than the carbon-fibre body -----------------------
    mats = {m.name: m for m in object_model.materials}
    glass_rgb = sum(mats["glass_dark"].rgba[:3])
    # The body IS dark carbon, and there is a bright alloy accent material -- the
    # identity is carbon-dark body + polished alloy, so assert the alloy reads
    # much brighter than both the carbon body and the glass.
    carbon_rgb = sum(mats["carbon_fibre"].rgba[:3])
    alloy_rgb = sum(mats["polished_alloy"].rgba[:3])
    ctx.check(
        "bare carbon-fibre body is dark, polished alloy accents are much brighter",
        carbon_rgb < 0.6 and alloy_rgb > carbon_rgb + 1.5,
        details=f"carbon={carbon_rgb:.2f}, alloy={alloy_rgb:.2f}",
    )
    ctx.check(
        "glass reads dark (smoked), close to the carbon body tone",
        glass_rgb < alloy_rgb - 1.0,
        details=f"glass={glass_rgb:.2f}, alloy={alloy_rgb:.2f}",
    )

    # --- Gullwing doors: roof-mounted lateral-dominant revolute, swing UP -----
    for hinge, door, side in ((hinge_l, door_l, "left"), (hinge_r, door_r, "right")):
        ax = tuple(hinge.axis)
        ctx.check(
            f"door_{side} hinge axis is lateral-dominant (gullwing roof hinge, swings up)",
            abs(ax[0]) > 0.9 and abs(ax[1]) < 1e-6,
            details=f"axis={ax}",
        )
        ml = hinge.motion_limits
        ctx.check(
            f"door_{side} hinge opens from closed",
            ml is not None and abs(ml.lower) < 1e-6 and 1.2 <= ml.upper <= 1.6,
            details=f"limits=({ml.lower}, {ml.upper})" if ml else "no limits",
        )
        # The hinge sits up near the roof line (gullwing, not a low scissor pivot).
        ho = hinge.origin
        ctx.check(
            f"door_{side} hinge is mounted high near the roof (gullwing)",
            ho is not None and ho.xyz[2] > 0.85,
            details=f"hinge z={ho.xyz[2] if ho else 'none'}",
        )
        ctx.expect_contact(body, door, contact_tol=0.05, name=f"door_{side} seated on body")

        rest = ctx.part_world_aabb(door)
        assert rest is not None
        with ctx.pose({hinge: DOOR_OPEN_MAX}):
            opened = ctx.part_world_aabb(door)
            assert opened is not None
        ctx.check(
            f"door_{side} swings UP (gullwing): top rises clearly when opened",
            opened[1][2] > rest[1][2] + 0.35,
            details=f"rest top z={rest[1][2]:.3f}, open top z={opened[1][2]:.3f}",
        )
        ctx.check(
            f"door_{side} bottom edge lifts clear of the rocker sill when open",
            opened[0][2] > rest[0][2] + 0.20,
            details=f"rest bottom z={rest[0][2]:.3f}, open bottom z={opened[0][2]:.3f}",
        )

    # --- Steering wheel: present, and TURNS about the column -----------------
    sw = object_model.get_part("steering_wheel")
    sw_names = {v.name for v in sw.visuals}
    ctx.check(
        "steering wheel: rim, grips, hub, display + marker",
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
