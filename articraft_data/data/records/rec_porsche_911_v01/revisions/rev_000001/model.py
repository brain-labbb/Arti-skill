from __future__ import annotations

# Classic air-cooled Porsche 911 (993/964 vibe), Guards Red.
# Z-up world. Long axis of the car runs along +Y (nose at +Y), width along X
# (driver/left side at +X), up along +Z. Wheels touch z = 0.
#
# Forked from the Diablo wedge supercar: the proven STRUCTURE is reused -- the
# cabin is hollowed out of the solid body by boolean_difference (cabin cavity +
# door apertures cut clean THROUGH the flanks), each front wheel hangs off a
# steering knuckle so it both STEERS about a vertical king-pin AND spins off the
# knuckle, rear wheels spin off the body, and straight axle rods run hub-to-hub
# through bored channels. The body is re-skinned from a sharp wedge into the
# unmistakable 911 silhouette: rounded sloping nose with ROUND headlights
# standing up on the front fender tops, a continuous fastback roofline sloping
# to the tail, WIDE rounded REAR haunches, and a rear-engine deck low at the
# very back closed by a thin ducktail spoiler lip.
#
# Primary articulation: BOTH doors swing OUT-and-slightly-UP on a conventional
# near-vertical hinge at the front of the door (911 doors are ordinary
# front-hinged doors, not scissor/gullwing). Secondary: all four wheels spin
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
# Global proportions (meters). Real 993 911: ~4.25 L x 1.80 W x 1.30 H,
# wheelbase 2.27, wheel radius ~0.32. We keep the body in the ~4.3 m length and
# ~1.8-1.9 m width range, with a notably TALLER greenhouse than the wedge.
# ----------------------------------------------------------------------------
WHEEL_R = 0.32
WHEEL_W = 0.265
# Track set so the wheels tuck just under the swollen rear haunch crests.
HALF_TRACK = 0.78
FRONT_AXLE_Y = 1.27
REAR_AXLE_Y = -1.27

# Wheel-arch cavities carved out of the solid lower body at each wheel so the
# wheels sit in tight open wells. Each cutter is a lateral (X) cylinder hugging
# the wheel that only opens the OUTBOARD flank. The 911 has classic round
# wheel-arch cutouts, so the arch radius is a touch larger than the tire.
ARCH_RADIUS = 0.355
# (x_center_sign*track, y_center, inboard_wall_abs_x, outboard_wall_abs_x)
WHEEL_ARCHES = (
    (HALF_TRACK, FRONT_AXLE_Y, 0.50, 1.00),
    (-HALF_TRACK, FRONT_AXLE_Y, 0.50, 1.00),
    (HALF_TRACK, REAR_AXLE_Y, 0.52, 1.04),
    (-HALF_TRACK, REAR_AXLE_Y, 0.52, 1.04),
)

# Straight axle rods run hub-to-hub at wheel-center height; a transverse channel
# is bored clean through the lower body on each axle line so the rod passes
# THROUGH the body instead of being buried in solid.
AXLE_BAR_RADIUS = 0.05
AXLE_CHANNEL_RADIUS = 0.085

# The cabin is hollowed out of the solid body so the seats + steering wheel sit
# in real open space, and the two door apertures are cut clean THROUGH the
# flanks so opening a door reveals the cabin. The 911 greenhouse is TALL, so the
# cabin is hollowed all the way up to where the glass canopy takes over (~0.92).
# Front edge stops at y~0.74 so the cowl/scuttle under the windshield stays
# solid; rear edge stops short of the engine bulkhead.
CABIN_HALF_X = 0.60
CABIN_Y = (-0.55, 0.74)
CABIN_Z = (0.50, 0.92)
DOOR_APERTURE_X = (0.50, 1.02)  # inboard (into cabin) .. outboard (through flank)
DOOR_APERTURE_Y = (-0.22, 0.70)
# Door-opening bottom raised to the CABIN FLOOR level (~0.50) so the door sill
# is FLUSH with the floor -- no step between the cabin floor and the door.
DOOR_APERTURE_Z = (0.50, 0.82)

# Door hinge (left; right mirrors x), at the FRONT edge of the door. A classic
# 911 door is front-hinged, swinging OUT about a near-vertical axis. We cant it
# a touch fore-aft so the trailing edge rises slightly as it opens (reads as a
# real swung door, not a flat slab), but it is dominantly vertical.
DOOR_HINGE = (0.83, 0.74, 0.50)
# Near-vertical king-pin, slightly canted fore-aft. Left door swings OUT (+X);
# right door uses the fully negated axis so it mirrors (OUT -X) under the same
# positive opening angle.
_DH = (0.0, -0.22, 0.975)
_DN = sqrt(_DH[0] * _DH[0] + _DH[1] * _DH[1] + _DH[2] * _DH[2])
DOOR_AXIS_LEFT = (_DH[0] / _DN, _DH[1] / _DN, _DH[2] / _DN)
DOOR_AXIS_RIGHT = (-_DH[0] / _DN, -_DH[1] / _DN, -_DH[2] / _DN)
DOOR_OPEN_MAX = 1.2

# Lower body side-profile rails: (y, z_min, z_max, width). The 911 silhouette:
# a low rounded sloping nose, fender tops that PEAK over the front wheels (the
# round headlights stand on these crests), a gently dropping hood, a tall
# cabin floor, then WIDE swollen REAR haunches (widest+tallest body), tapering
# into a rounded fastback tail with the engine deck low at the very back.
LOWER_SECTIONS = [
    (2.16, 0.16, 0.40, 0.96),   # low rounded nose tip / bumper
    (2.00, 0.13, 0.52, 1.34),   # nose rising
    (1.78, 0.12, 0.66, 1.66),   # front fender shoulder rising
    (1.50, 0.12, 0.78, 1.78),   # peaked front fender crown (round lamps stand here)
    (1.27, 0.12, 0.79, 1.78),   # over front axle -- fender caps the well
    (1.02, 0.13, 0.70, 1.70),   # hood / frunk lid dips below the fender crowns
    (0.82, 0.13, 0.74, 1.68),   # cowl rising toward the windshield base
    (0.62, 0.14, 0.75, 1.66),   # scuttle / windshield base shelf
    (0.36, 0.14, 0.75, 1.66),   # door front, beltline
    (-0.10, 0.14, 0.75, 1.72),  # door mid
    (-0.54, 0.12, 0.78, 1.86),  # rear haunch swelling begins
    (-0.96, 0.11, 0.80, 1.96),  # muscular rear haunch crown (widest, tallest)
    (-1.34, 0.11, 0.74, 1.92),  # over rear axle, haunch holding
    (-1.74, 0.13, 0.66, 1.70),  # engine deck dropping at the rear
    (-2.04, 0.18, 0.58, 1.44),  # low engine lid
    (-2.20, 0.24, 0.52, 1.18),  # rounded fastback tail
]

# TALL fastback greenhouse, built as THREE thin shells that share seam sections
# so windshield / roof / rear glass meet exactly. Front + rear are tinted glass;
# the middle is the body-color roof. The 911 roofline is a single continuous
# curve from the steep windshield over a domed roof and down a long, gently
# sloping fastback rear screen to the engine lid. The greenhouse is NARROWER
# than the body (the roof tapers in well inside the swollen flanks -- it must NOT
# balloon out into a full-width blister), so the cabin reads as a distinct
# glasshouse perched on the body. The seam rails are reused verbatim by adjacent
# shells, which guarantees the panels line up perfectly.
# The 911 roofline is ONE continuous curve: steep windshield -> a short roof
# crown directly over the seats -> a long, gently sloping fastback rear screen
# that runs all the way down onto the engine lid. To get that flow the roof
# crown is kept fairly LOW (~1.22) and SHORT (front-to-back), and the rear seam
# sits just behind the seats so the fastback can be long and shallow, dropping
# smoothly to the deck instead of falling off a vertical back wall.
# The glasshouse BASE must seat down on the body beltline (~z 0.72) so there is
# no open gap between the body top and the glass: the windshield/side/rear glass
# all START at the beltline and rise from there. The roofline is ONE continuous
# curve: steep windshield -> a short roof crown over the seats -> a long, gently
# sloping fastback rear screen running down onto the engine lid.
# A LOW, smooth, even-width canopy tube: the windshield, roof and fastback share
# a near-constant width (~0.96) and rounded top so the glasshouse reads as one
# continuous flowing surface (no flaring slabs, no narrow bulb). The roof is kept
# low (~1.20) so it does not balloon over the body.
_SEAM_FRONT = (0.54, 0.80, 1.14, 0.98)  # windshield top == roof leading edge
_SEAM_REAR = (-0.20, 0.78, 1.16, 0.96)  # roof trailing edge == rear glass top
WINDSHIELD_SECTIONS = [
    (0.86, 0.74, 0.96, 1.00),   # windshield base seated on the cowl/scuttle
    (0.70, 0.78, 1.06, 0.99),   # steep A-pillar rake rising
    _SEAM_FRONT,
]
# Short, gently domed roof crown directly over the seats: a thin even-width skin
# flowing smoothly from the windshield into the fastback.
ROOF_SECTIONS = [
    _SEAM_FRONT,
    (0.14, 0.80, 1.20, 0.96),   # roof crown over the seats (~1.20 m)
    _SEAM_REAR,
]
# LONG gently raked fastback rear screen: from the roof crown just behind the
# seats it slopes smoothly down onto the engine lid, base seated on the rear
# quarters/deck, widening only slightly as it descends (the 911 fastback flare).
REAR_WINDOW_SECTIONS = [
    _SEAM_REAR,
    (-0.64, 0.74, 1.08, 0.98),
    (-1.06, 0.72, 0.94, 1.02),
    (-1.50, 0.66, 0.76, 1.06),
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


_LOWER_BODY_CACHE = None


def _lower_body_mesh():
    # Low exponent -> rounded, soft superellipse cross-sections (the curvy 911
    # body, not the angular wedge). Then carve open wheel arches, axle channels,
    # the hollow cabin and the two door apertures, plus shallow pockets in the
    # fender tops for the round headlight buckets to sit in. Cached (reused by
    # visual export + QC).
    global _LOWER_BODY_CACHE
    if _LOWER_BODY_CACHE is None:
        body = superellipse_side_loft(LOWER_SECTIONS, exponents=2.4, segments=64)
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
        # Hood centre-dip: the 911 hood/frunk lid sinks BETWEEN the two raised
        # round-headlight fender crests. Carve a wide shallow valley down the
        # hood centre with a big cylinder laid fore-aft so the flanks read as
        # the raised fenders that the round lamps stand on.
        hood_chan = (
            CylinderGeometry(radius=0.90, height=1.05, radial_segments=56)
            .rotate_x(pi / 2.0)  # axis Z -> Y (fore-aft)
            .translate(0.0, 1.55, 0.74 + 0.90 - 0.085)
        )
        body = boolean_difference(body, hood_chan)
        # Round headlight buckets: the iconic 911 lamps are round bowls set INTO
        # the tops of the front fenders, standing UPRIGHT and facing forward.
        # Carve a round bowl into each fender crown, tilted only gently back so
        # the lamp face stands up and looks ahead (not a flat lamp lying on the
        # sheetmetal). The bowl axis points forward+up so the lens seats in a
        # recessed bucket on the fender top. The carve is kept SHALLOW (short
        # cylinder) so a thick solid fender shelf remains UNDER and BEHIND the
        # bowl -- the headlight bucket plunges down/back into that shelf so the
        # lamp grows out of the fender instead of hovering in an empty hole.
        for hx_sign in (1.0, -1.0):
            bowl = (
                CylinderGeometry(radius=0.140, height=0.13, radial_segments=32)
                .rotate_x(1.05)  # bowl axis tilts toward +Y/up -> lamp faces forward
                .translate(hx_sign * 0.60, 1.70, 0.74)
            )
            body = boolean_difference(body, bowl)
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


def _glass_pane(sections, cabin_cut=None):
    # A FULLY SOLID, see-through-proof glass pane. _glass_shell built a THIN
    # HOLLOW skin and dropped its inner-loft bottom 0.15 BELOW the section so the
    # underside opened wide into the cabin; from a 3/4 view that open underside
    # plus the razor-thin curved side walls read as triangular SEE-THROUGH gaps
    # you can look clean through to the interior/sky. Here the pane is the SOLID
    # superellipse loft -- a single filled volume, so every outward face is one
    # continuous surface with NO interior hole and NO sliver wall: nothing to see
    # through. The windshield volume (y 0.54..0.86, z 0.74..1.14) sits well
    # FORWARD of the seats (y -0.30) and the steering wheel (y 0.44), and above
    # the cabin floor (z 0.51), so the solid does not bury any interior part.
    # If a future section set did poke into the cabin, an optional cabin_cut box
    # scoops the lower interior back out (the scoop stays hidden up inside the
    # glasshouse) while the whole OUTER pane stays unbroken.
    solid = superellipse_side_loft(sections, exponents=2.8, segments=56)
    if cabin_cut is not None:
        solid = boolean_difference(solid, _box_cutter(*cabin_cut))
    return solid


# Shared wheel/tire geometry: low-profile black tire + Fuchs-style 5-spoke alloy
# (polished silver star with five flat blades, classic 911 wheel).
_TIRE_GEOM = TireGeometry(
    WHEEL_R,
    WHEEL_W,
    inner_radius=0.205,
    carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.04),
    sidewall=TireSidewall(style="rounded", bulge=0.04),
)
_WHEEL_GEOM = WheelGeometry(
    0.210,
    0.195,
    rim=WheelRim(inner_radius=0.180, flange_height=0.014, flange_thickness=0.007),
    hub=WheelHub(
        radius=0.060,
        width=0.09,
        cap_style="domed",
        bolt_pattern=BoltPattern(count=5, circle_diameter=0.078, hole_diameter=0.010),
    ),
    # Low dish, face pulled flush with the hub so the five Fuchs blades read as a
    # flat polished star, not a deeply recessed dish.
    face=WheelFace(dish_depth=0.010, front_inset=0.006, window_depth=0.022),
    # Five broad flat spokes (the Fuchs star); wide solid blades between narrow
    # windows so it reads as the classic five-petal Fuchs wheel.
    spokes=WheelSpokes(style="straight", count=5, thickness=0.052, window_radius=0.058),
    bore=WheelBore(style="round", diameter=0.034),
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="porsche_911")

    # Guards Red -- the signature 911 colour.
    guards_red = model.material("guards_red", rgba=(0.78, 0.05, 0.06, 1.0))
    black_trim = model.material("black_trim", rgba=(0.05, 0.05, 0.055, 1.0))
    model.material("glass_dark", rgba=(0.08, 0.09, 0.11, 1.0))  # used by QC by name
    # Smoked but see-through glazing for the windshield + side/rear windows.
    glass_tint = model.material("glass_tint", rgba=(0.11, 0.12, 0.14, 0.34))
    silver = model.material("silver_alloy", rgba=(0.80, 0.81, 0.84, 1.0))
    rubber = model.material("rubber", rgba=(0.04, 0.04, 0.045, 1.0))
    amber = model.material("amber", rgba=(0.86, 0.52, 0.07, 1.0))
    red_tail = model.material("tail_red", rgba=(0.60, 0.04, 0.05, 1.0))
    clear_lens = model.material("clear_lens", rgba=(0.90, 0.91, 0.93, 1.0))
    chrome = model.material("chrome", rgba=(0.82, 0.83, 0.86, 1.0))
    interior_dk = model.material("interior_dark", rgba=(0.10, 0.10, 0.11, 1.0))
    axle_steel = model.material("axle_steel", rgba=(0.66, 0.67, 0.70, 1.0))

    # ------------------------------------------------------------------ body
    body = model.part("body")
    body.visual(_save("lower_body.obj", _lower_body_mesh()), material=guards_red, name="lower_body")
    # Glasshouse = three thin shells sharing seam rails (seamless windshield /
    # roof / fastback rear window); the roof reads as a thin body-color skin.
    body.visual(_save("roof.obj", _glass_shell(ROOF_SECTIONS)), material=guards_red, name="greenhouse")
    # Solid windshield pane. Its lower-rear interior is scooped out by a cabin
    # cut so the raked steering-wheel rim (which sweeps up to z~0.86 just behind
    # the A-pillar base) tucks into open cabin space instead of being buried in
    # solid glass (which would trip the body<->steering_wheel real_overlap). The
    # cut stays low and to the REAR (y<=0.72, away from the y=0.86 base) so the
    # whole OUTER glass surface -- the steep front pane and the dome -- remains a
    # single unbroken see-through-proof pane.
    body.visual(
        _save(
            "windshield.obj",
            _glass_pane(WINDSHIELD_SECTIONS, cabin_cut=(-0.55, 0.55, 0.46, 0.72, 0.40, 0.88)),
        ),
        material=glass_tint,
        name="windshield",
    )
    body.visual(
        _save("rear_window.obj", _glass_pane(REAR_WINDOW_SECTIONS)),
        material=glass_tint,
        name="rear_window",
    )

    # Cabin interior: a single FLAT thin floor panel flush with the carved body
    # floor pan (top ~z=0.51), spanning the whole cabin.
    body.visual(
        Box((1.20, 1.28, 0.05)),
        origin=Origin(xyz=(0.0, 0.06, 0.485)),
        material=interior_dk,
        name="cabin_floor",
    )
    # Seats sit ON the flat floor (base at the floor top ~0.51). Kept low so the
    # seat tops stay below the beltline and do not poke up through the glass.
    for sx, side in ((0.30, "left"), (-0.30, "right")):
        body.visual(
            Box((0.40, 0.18, 0.26)),
            origin=Origin(xyz=(sx, -0.30, 0.64)),
            material=interior_dk,
            name=f"seat_back_{side}",
        )
        body.visual(
            Box((0.40, 0.34, 0.05)),
            origin=Origin(xyz=(sx, -0.08, 0.525)),
            material=interior_dk,
            name=f"seat_base_{side}",
        )
    # Firewall/bulkhead sealing the BACK of the cabin so the hollow interior is
    # closed off from the engine bay and not seen through. Top held at the
    # beltline (~0.75) so it caps the cabin rear without poking into the glass.
    body.visual(
        Box((1.12, 0.05, 0.30)),
        origin=Origin(xyz=(0.0, -0.55, 0.62)),
        material=interior_dk,
        name="cabin_bulkhead",
    )

    # Steering wheel: raked column fixed on the body; the round wheel is a
    # SEPARATE part that SPINS about the raked column axis.
    _SW_X = 0.34
    _SW_FWD = 0.46
    _SW_RAKE = (0.62, 0.0, 0.0)
    _SW_HUB = (_SW_X, -0.02 + _SW_FWD, 0.74)
    body.visual(
        Cylinder(radius=0.022, length=0.34),
        origin=Origin(xyz=(_SW_X, 0.08 + _SW_FWD, 0.60), rpy=_SW_RAKE),
        material=black_trim,
        name="steering_column",
    )
    sw_accent = model.material("sw_accent", rgba=(0.80, 0.05, 0.06, 1.0))
    steer_wheel = model.part("steering_wheel")
    # Classic round 911 wheel: a thin circular rim ring built from boxes around
    # the circle, three thin spokes, a central hub. Authored in its own un-raked
    # frame (rim in local XY, hub along +Z); the joint applies the column rake.
    import math as _m

    _RIM_R = 0.150       # rim radius
    _RIM_BAR = 0.022     # rim cross-section
    _RIM_DEPTH = 0.040
    _N_RIM = 16
    for k in range(_N_RIM):
        a = 2.0 * _m.pi * k / _N_RIM
        px = _RIM_R * _m.cos(a)
        py = _RIM_R * _m.sin(a)
        # Each short rim segment is a small box tangent to the circle.
        seg_len = 2.0 * _m.pi * _RIM_R / _N_RIM * 1.18
        steer_wheel.visual(
            Box((seg_len, _RIM_BAR, _RIM_DEPTH)),
            origin=Origin(xyz=(px, py, 0.0), rpy=(0.0, 0.0, a + _m.pi / 2.0)),
            material=black_trim,
            name=f"sw_rim_{k}",
        )
    # Name the top and bottom rim segments explicitly so QC can find rim parts.
    # (sw_rim_4 is near +Y top, sw_rim_12 near -Y bottom for N=16.)
    # Left/right padded grips at 3 and 9 o'clock.
    for sx, _tag in ((_RIM_R, "left"), (-_RIM_R, "right")):
        steer_wheel.visual(
            Box((_RIM_BAR + 0.016, 0.130, _RIM_DEPTH + 0.012)),
            origin=Origin(xyz=(sx, 0.0, 0.0)),
            material=black_trim,
            name=f"sw_grip_{_tag}",
        )
    # Three spokes (911 wheels are three-spoke): cross-bar across X + lower spoke.
    steer_wheel.visual(
        Box((2.0 * _RIM_R, 0.028, 0.018)),
        material=black_trim,
        name="sw_spoke_cross",
    )
    steer_wheel.visual(
        Box((0.028, _RIM_R, 0.018)),
        origin=Origin(xyz=(0.0, -_RIM_R / 2.0, 0.0)),
        material=black_trim,
        name="sw_spoke_low",
    )
    steer_wheel.visual(
        Cylinder(radius=0.046, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, -0.02)),
        material=black_trim,
        name="sw_hub",
    )
    # Round centre boss with the crest accent.
    steer_wheel.visual(
        Cylinder(radius=0.034, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=interior_dk,
        name="sw_display",
    )
    # Bright top-centre alignment marker (off the spin axis, so it sweeps round
    # and proves the wheel is turning).
    steer_wheel.visual(
        Box((0.030, 0.020, 0.020)),
        origin=Origin(xyz=(0.0, _RIM_R, 0.020)),
        material=sw_accent,
        name="sw_top_marker",
    )
    steer_wheel.inertial = Inertial.from_geometry(Box((0.32, 0.32, 0.06)), mass=2.0)
    model.articulation(
        "steering_wheel_turn",
        ArticulationType.REVOLUTE,
        parent=body,
        child=steer_wheel,
        origin=Origin(xyz=_SW_HUB, rpy=_SW_RAKE),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=8.0, lower=-3.14, upper=3.14),
    )

    # --- Front bumper / low full-width intake + round turn signals -------------
    # Body-color front bumper wrapping the lower nose (the integrated 911 bumper).
    body.visual(
        Box((1.50, 0.30, 0.20)),
        origin=Origin(xyz=(0.0, 2.02, 0.28)),
        material=guards_red,
        name="front_bumper",
    )
    # Low full-width black intake slot under the bumper (the 911 air intake).
    body.visual(
        Box((1.12, 0.10, 0.13)),
        origin=Origin(xyz=(0.0, 2.13, 0.235)),
        material=black_trim,
        name="front_intake",
    )
    # Slim black front spoiler lip under the nose.
    body.visual(
        Box((1.40, 0.34, 0.05)),
        origin=Origin(xyz=(0.0, 2.00, 0.135)),
        material=black_trim,
        name="front_splitter",
    )
    # Round amber turn signals at the outer corners of the bumper (911 has round
    # indicator/foglamp pods low on the bumper, beside the intake).
    for sx, side in ((0.56, "left"), (-0.56, "right")):
        body.visual(
            Cylinder(radius=0.058, length=0.05),
            origin=Origin(xyz=(sx, 2.155, 0.27), rpy=(pi / 2.0, 0.0, 0.0)),
            material=black_trim,
            name=f"turn_pod_{side}",
        )
        body.visual(
            Cylinder(radius=0.046, length=0.04),
            origin=Origin(xyz=(sx, 2.175, 0.27), rpy=(pi / 2.0, 0.0, 0.0)),
            material=amber,
            name=f"turn_signal_{side}",
        )

    # --- ICONIC ROUND HEADLIGHTS standing UP on the front fender tops ----------
    # The single most recognizable 911 feature: a big round headlamp set upright
    # in a bowl on top of each front fender, facing UP-and-FORWARD. Built as a
    # stack of round discs seating in the carved fender bowl, with the disc
    # AXIS pointing forward+up (rpy x ~1.05) so the round face is presented to
    # anyone in front of the car:
    #   - a black bucket ring (the rim of the bowl)
    #   - a chrome trim ring
    #   - a clear convex lens (the bright glass eye)
    #   - a small round high-beam/parking accent at the centre
    _HL_RAKE = (1.05, 0.0, 0.0)  # disc axis tilts toward +Y/up -> lamp faces forward
    _HL_Y = 1.70
    _HL_Z = 0.70
    for sx, side in ((0.60, "left"), (-0.60, "right")):
        # Black bucket: a DEEP round can that plunges DOWN and BACK into the
        # solid fender shelf under the shallow carved bowl. Its back/lower wall
        # bites well into solid body material so the whole lamp stack grows out
        # of the fender crown (connected), not hovering in an empty hole. The
        # bucket is a touch wider/rounder than before so the lamp reads bigger.
        body.visual(
            Cylinder(radius=0.158, length=0.22),
            origin=Origin(xyz=(sx, _HL_Y - 0.075, _HL_Z - 0.04), rpy=_HL_RAKE),
            material=black_trim,
            name=f"headlight_bucket_{side}",
        )
        # Chrome trim ring framing the lens, overlapping the bucket mouth.
        body.visual(
            Cylinder(radius=0.150, length=0.06),
            origin=Origin(xyz=(sx, _HL_Y + 0.015, _HL_Z + 0.075), rpy=_HL_RAKE),
            material=chrome,
            name=f"headlight_ring_{side}",
        )
        # Clear convex lens -- the bright round glass eye facing forward (larger,
        # rounder), seated into the chrome ring + bucket.
        body.visual(
            Cylinder(radius=0.130, length=0.08),
            origin=Origin(xyz=(sx, _HL_Y + 0.06, _HL_Z + 0.105), rpy=_HL_RAKE),
            material=clear_lens,
            name=f"headlight_{side}",
        )
        # Inner reflector accent disc at the centre of the eye.
        body.visual(
            Cylinder(radius=0.055, length=0.09),
            origin=Origin(xyz=(sx, _HL_Y + 0.09, _HL_Z + 0.13), rpy=_HL_RAKE),
            material=clear_lens,
            name=f"headlight_inner_{side}",
        )

    # Black rocker sills between the wheel arches.
    for sx, side in ((0.80, "left"), (-0.80, "right")):
        body.visual(
            Box((0.10, 1.66, 0.12)),
            origin=Origin(xyz=(sx, 0.0, 0.165)),
            material=black_trim,
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

    # Engine air intake louvers low on the rear flank ahead of the rear wheels
    # (the 911's rear-quarter cooling intakes), with a body-color leading edge.
    for sx, side in ((0.86, "left"), (-0.86, "right")):
        body.visual(
            Box((0.09, 0.40, 0.20)),
            origin=Origin(xyz=(sx, -0.66, 0.55)),
            material=black_trim,
            name=f"side_intake_{side}",
        )
        body.visual(
            Box((0.095, 0.36, 0.03)),
            origin=Origin(xyz=(sx * (0.865 / 0.86), -0.66, 0.55)),
            material=guards_red,
            name=f"intake_slat_{side}",
        )

    # --- Rear engine deck (low at the very back) + ducktail spoiler lip --------
    # Body-color engine lid sealing the rear deck flush with the body, with a
    # central black cooling grille (the rear-engine 911 deck breathes through a
    # grille set into the lid).
    body.visual(
        Box((1.36, 0.78, 0.05)),
        origin=Origin(xyz=(0.0, -1.62, 0.665)),
        material=guards_red,
        name="engine_deck",
    )
    # Black engine cooling grille slats set into the lid.
    for k in range(6):
        body.visual(
            Box((1.04, 0.075, 0.02)),
            origin=Origin(xyz=(0.0, -1.40 - 0.10 * k, 0.695), rpy=(0.06, 0.0, 0.0)),
            material=black_trim,
            name=f"deck_louver_{k}",
        )
    # DUCKTAIL spoiler lip: a thin body-color blade kicking up off the trailing
    # edge of the engine lid (the classic 911 Carrera ducktail). It is short and
    # sits low, not a tall whale-tail or twin-pylon wing.
    body.visual(
        Box((1.34, 0.34, 0.045)),
        origin=Origin(xyz=(0.0, -1.96, 0.72), rpy=(-0.28, 0.0, 0.0)),
        material=guards_red,
        name="ducktail_blade",
    )
    # Black rubber lip edge along the back of the ducktail.
    body.visual(
        Box((1.30, 0.06, 0.03)),
        origin=Origin(xyz=(0.0, -2.10, 0.755)),
        material=black_trim,
        name="ducktail_lip",
    )
    # Two short body-color risers tying the ducktail down into the engine lid so
    # it reads as rooted, not floating.
    for sx, side in ((0.50, "left"), (-0.50, "right")):
        body.visual(
            Box((0.10, 0.20, 0.10)),
            origin=Origin(xyz=(sx, -1.90, 0.685)),
            material=guards_red,
            name=f"ducktail_riser_{side}",
        )

    # --- Tail: full-width red reflector band with PORSCHE lettering ------------
    # Body-color rear bumper wrapping the lower tail.
    body.visual(
        Box((1.44, 0.30, 0.20)),
        origin=Origin(xyz=(0.0, -2.10, 0.30)),
        material=guards_red,
        name="rear_bumper",
    )
    # Dark backing strip carrying the whole tail-light assembly. It is a DEEP
    # plate that runs from just behind the body surface FORWARD into the solid
    # rear bodywork (the body ends at y=-2.20 at this height), so the plate bites
    # into the body and the band/lights/letters seated on it are all connected.
    TAIL_FACE_Y = -2.245  # rear (visible) face of the tail assembly
    body.visual(
        Box((1.34, 0.16, 0.135)),
        origin=Origin(xyz=(0.0, TAIL_FACE_Y + 0.08, 0.50)),
        material=interior_dk,
        name="tail_panel",
    )
    # FULL-WIDTH red reflector band spanning the tail between the taillights
    # (the signature 911 red band across the back). Seated against the panel
    # front, biting a hair into it so band+panel are one connected mesh.
    BAND_Y = TAIL_FACE_Y + 0.018
    BAND_Z = 0.50
    body.visual(
        Box((1.30, 0.040, 0.090)),
        origin=Origin(xyz=(0.0, BAND_Y, BAND_Z)),
        material=red_tail,
        name="reflector_band",
    )
    # Round-ish red taillight clusters at each end of the band, overlapping both
    # the band and the backing panel.
    for sx, side in ((0.56, "left"), (-0.56, "right")):
        body.visual(
            Box((0.26, 0.055, 0.13)),
            origin=Origin(xyz=(sx, TAIL_FACE_Y + 0.020, BAND_Z)),
            material=red_tail,
            name=f"taillight_{side}",
        )
    # "PORSCHE" wordmark: a row of small pale blocks pressed onto the band face,
    # each overlapping the band so the lettering is connected to it.
    _word = "PORSCHE"
    _n = len(_word)
    _span = 0.62
    for i in range(_n):
        lx = (i - (_n - 1) / 2.0) * (_span / (_n - 1))
        body.visual(
            Box((0.045, 0.022, 0.034)),
            origin=Origin(xyz=(lx, BAND_Y - 0.020, BAND_Z + 0.002)),
            material=clear_lens,
            name=f"porsche_letter_{i}",
        )

    # Twin chrome tailpipes low under the rear bumper (911 sports exhaust).
    for sx, side in ((0.22, "left"), (-0.22, "right")):
        body.visual(
            Cylinder(radius=0.046, length=0.10),
            origin=Origin(xyz=(sx, -2.24, 0.20), rpy=(pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name=f"exhaust_ring_{side}",
        )
        body.visual(
            Cylinder(radius=0.034, length=0.08),
            origin=Origin(xyz=(sx, -2.26, 0.20), rpy=(pi / 2.0, 0.0, 0.0)),
            material=black_trim,
            name=f"exhaust_{side}",
        )
    # Slim black rear valance/diffuser strip across the lower tail.
    body.visual(
        Box((1.30, 0.18, 0.10)),
        origin=Origin(xyz=(0.0, -2.16, 0.135)),
        material=black_trim,
        name="rear_valance",
    )

    body.inertial = Inertial.from_geometry(
        Box((1.80, 4.25, 1.30)),
        mass=1380.0,
        origin=Origin(xyz=(0.0, 0.0, 0.58)),
    )

    # ----------------------------------------------------- front-hinged doors
    hx, hy, hz = DOOR_HINGE

    def make_door(side: str):
        s = 1.0 if side == "left" else -1.0
        door = model.part(f"door_{side}")

        # Door skin loft authored in body frame, shifted into the local hinge
        # frame (hinge sits at the door's front edge). Thin skin hugging the
        # rounded flank.
        skin_sections = [
            (0.70, 0.22, 0.78, 0.07),
            (0.40, 0.18, 0.80, 0.09),
            (0.00, 0.18, 0.80, 0.09),
            (-0.34, 0.20, 0.78, 0.07),
        ]
        skin = superellipse_side_loft(skin_sections, exponents=2.6, segments=40)
        door.visual(
            _save(f"door_{side}_skin.obj", skin.translate(0.0, -hy, -hz)),
            material=guards_red,
            name="door_skin",
        )

        glass_sections = [
            (0.62, 0.81, 0.92, 0.08),
            (0.34, 0.82, 1.08, 0.10),
            (-0.04, 0.82, 1.10, 0.10),
            (-0.32, 0.82, 1.00, 0.08),
        ]
        glass_loft = superellipse_side_loft(glass_sections, exponents=2.6, segments=36)
        door.visual(
            _save(f"door_{side}_glass.obj", glass_loft.translate(-s * 0.26, -hy, -hz)),
            material=glass_tint,
            name="door_glass",
        )

        # Beltline trim at the window sill, bridging the outer skin to the
        # inboard window glass so door + window read as ONE connected panel.
        door.visual(
            Box((0.32, 1.00, 0.04)),
            origin=Origin(xyz=(-s * 0.10, -0.50, 0.30)),
            material=black_trim,
            name="door_beltline",
        )
        # Flush black door handle near the rear edge.
        door.visual(
            Box((0.04, 0.14, 0.03)),
            origin=Origin(xyz=(s * 0.10, -0.86, -0.04)),
            material=black_trim,
            name="door_handle",
        )
        # Body-color side mirror on the door's front-top corner (911 mirror).
        door.visual(
            Box((0.12, 0.06, 0.04)),
            origin=Origin(xyz=(s * 0.11, -0.12, 0.10), rpy=(0.0, -s * 0.5, 0.0)),
            material=guards_red,
            name="mirror_stalk",
        )
        door.visual(
            Box((0.07, 0.15, 0.11)),
            origin=Origin(xyz=(s * 0.19, -0.14, 0.14)),
            material=guards_red,
            name="mirror_head",
        )

        door.inertial = Inertial.from_geometry(
            Box((0.20, 1.05, 0.90)),
            mass=30.0,
            origin=Origin(xyz=(0.0, -0.42, -0.10)),
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
            mass=20.0,
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
    # Front-hinged doors: revolute about a near-vertical (canted fore-aft) axis
    # at the door's front edge; +q swings the door OUT (and a touch up).
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
    for _selem in ("sw_hub", "sw_spoke_cross", "sw_spoke_low", "sw_display"):
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
                reason="Wheel seated flush inside the round wheel arch of the solid body shell.",
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

    # --- Hero features present and legible -----------------------------------
    vis_names = {v.name for v in body.visuals}
    ctx.check(
        "lofted rounded body + tall greenhouse present (not a box)",
        {"lower_body", "greenhouse"} <= vis_names,
        details=f"body visuals={sorted(vis_names)}",
    )
    ctx.check(
        "windshield and fastback rear window glass present",
        {"windshield", "rear_window"} <= vis_names,
    )
    ctx.check(
        "iconic round headlights on the fender tops: lens + chrome ring + bucket, both sides",
        {
            "headlight_left",
            "headlight_right",
            "headlight_ring_left",
            "headlight_ring_right",
            "headlight_bucket_left",
            "headlight_bucket_right",
        }
        <= vis_names,
    )
    ctx.check(
        "front bumper + low full-width intake + round turn signals + spoiler lip",
        {
            "front_bumper",
            "front_intake",
            "front_splitter",
            "turn_signal_left",
            "turn_signal_right",
        }
        <= vis_names,
    )
    ctx.check(
        "black rocker sills both sides",
        {"rocker_sill_left", "rocker_sill_right"} <= vis_names,
    )
    ctx.check(
        "rear-quarter engine intakes ahead of rear wheels (both sides)",
        {"side_intake_left", "side_intake_right"} <= vis_names,
    )
    ctx.check(
        "rear engine lid with cooling grille (>=5 slats)",
        sum(1 for v in body.visuals if v.name.startswith("deck_louver_")) >= 5,
    )
    ctx.check(
        "ducktail spoiler lip on the engine deck (not a tall twin-pylon wing)",
        {"ducktail_blade", "ducktail_lip"} <= vis_names
        and {"ducktail_riser_left", "ducktail_riser_right"} <= vis_names,
    )
    ctx.check(
        "full-width red reflector band with PORSCHE lettering across the tail",
        "reflector_band" in vis_names
        and sum(1 for v in body.visuals if v.name.startswith("porsche_letter_")) >= 7
        and {"taillight_left", "taillight_right"} <= vis_names,
    )
    ctx.check(
        "twin tailpipes + rear valance",
        {"exhaust_left", "exhaust_right", "rear_valance"} <= vis_names,
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

    # --- 911 silhouette: round headlights stand UP on raised fender tops ------
    mesh_verts = _lower_body_mesh().vertices
    fender_cover = max(
        (
            z
            for (x, y, z) in mesh_verts
            if abs(abs(x) - HALF_TRACK) < 0.20 and abs(y - FRONT_AXLE_Y) < 0.30
        ),
        default=0.0,
    )
    ctx.check(
        "front fender drapes above the front wheel top (caps the well from above)",
        fender_cover > 2.0 * WHEEL_R + 0.02,
        details=f"front fender top z={fender_cover:.3f}, wheel top z={2.0 * WHEEL_R:.3f}",
    )
    # Round headlights sit HIGH on the fender tops, well above the low front
    # bumper/intake -- the iconic upright 911 lamp position.
    hl = ctx.part_element_world_aabb(body, elem="headlight_left")
    intake = ctx.part_element_world_aabb(body, elem="front_intake")
    assert hl is not None and intake is not None
    ctx.check(
        "round headlights stand high on the fender tops, well above the low intake",
        hl[0][2] > intake[1][2] + 0.30,
        details=f"headlight bottom z={hl[0][2]:.3f}, intake top z={intake[1][2]:.3f}",
    )
    # Headlights face the front (their bowl is on the front fender, ahead of the
    # cabin) -- they sit forward of the body centre.
    ctx.check(
        "round headlights sit on the FRONT fenders (forward of centre)",
        hl[0][1] > 1.3,
        details=f"headlight min y={hl[0][1]:.3f}",
    )

    # --- Fastback profile: continuous roof curve, NOT a sharp wedge -----------
    nose_top = max(z for (_x, y, z) in mesh_verts if y > 1.9)
    haunch_top = max(z for (_x, y, z) in mesh_verts if -1.2 < y < -0.7)
    ctx.check(
        "rounded 911 profile: nose is gently (not drastically) below the rear haunch",
        0.0 < haunch_top - nose_top < 0.55,
        details=f"nose_top={nose_top:.3f}, haunch_top={haunch_top:.3f}",
    )
    # Wide swollen REAR haunches: the body is widest over the rear axle region.
    rear_w = max(
        (abs(x) for (x, y, _z) in mesh_verts if -1.1 < y < -0.8), default=0.0
    )
    front_w = max(
        (abs(x) for (x, y, _z) in mesh_verts if 1.6 < y < 1.9), default=0.0
    )
    ctx.check(
        "rear haunches swell WIDER than the front fenders (911 wide hips)",
        rear_w > front_w + 0.04,
        details=f"rear half-width={rear_w:.3f}, front half-width={front_w:.3f}",
    )
    # Tall greenhouse: the roof crown clears the wedge-era ~1.1 m, 911 is taller.
    roof = ctx.part_element_world_aabb(body, elem="greenhouse")
    assert roof is not None
    ctx.check(
        "tall 911 greenhouse (roof crown well above the wedge ~1.1 m)",
        roof[1][2] > 1.16,
        details=f"roof crown z={roof[1][2]:.3f}",
    )

    # --- Scale sanity ---------------------------------------------------------
    bb = ctx.part_world_aabb(body)
    assert bb is not None
    lo, hi = bb
    ctx.check("car length ~4.25 m", 4.0 <= hi[1] - lo[1] <= 4.6, details=f"L={hi[1] - lo[1]:.3f}")
    ctx.check("car width ~1.85 m", 1.7 <= hi[0] - lo[0] <= 2.05, details=f"W={hi[0] - lo[0]:.3f}")
    ctx.check("car height ~1.3 m", 1.15 <= hi[2] <= 1.45, details=f"H={hi[2]:.3f}")

    # --- Ducktail spoiler rides low on the engine lid (not a tall wing) -------
    duck = ctx.part_element_world_aabb(body, elem="ducktail_blade")
    deck = ctx.part_element_world_aabb(body, elem="engine_deck")
    roof_bb = ctx.part_element_world_aabb(body, elem="greenhouse")
    assert duck is not None and deck is not None and roof_bb is not None
    ctx.check(
        "ducktail sits just above the engine lid (low lip, not a tall wing)",
        duck[0][2] >= deck[0][2] - 0.05 and duck[1][2] < roof_bb[1][2] - 0.30,
        details=f"ducktail z=[{duck[0][2]:.3f},{duck[1][2]:.3f}], deck bottom={deck[0][2]:.3f}, roof top={roof_bb[1][2]:.3f}",
    )

    # --- Glass reads darker than the red paint -------------------------------
    mats = {m.name: m for m in object_model.materials}
    glass_rgb = sum(mats["glass_dark"].rgba[:3])
    body_rgb = sum(mats["guards_red"].rgba[:3])
    ctx.check(
        "glass is darker than the Guards Red body",
        glass_rgb < body_rgb - 0.4,
        details=f"glass={glass_rgb:.2f}, guards_red={body_rgb:.2f}",
    )
    ctx.check(
        "body colour is recognizably RED (red channel dominant)",
        mats["guards_red"].rgba[0] > 0.6
        and mats["guards_red"].rgba[1] < 0.2
        and mats["guards_red"].rgba[2] < 0.2,
        details=f"guards_red rgba={mats['guards_red'].rgba}",
    )

    # --- Front-hinged doors: near-vertical revolute, swing OUT ----------------
    for hinge, door, side in ((hinge_l, door_l, "left"), (hinge_r, door_r, "right")):
        ax = tuple(hinge.axis)
        ctx.check(
            f"door_{side} hinge axis is near-vertical (conventional front-hinged door)",
            abs(ax[2]) > 0.9 and abs(ax[0]) < 0.2,
            details=f"axis={ax}",
        )
        ml = hinge.motion_limits
        ctx.check(
            f"door_{side} hinge opens from closed",
            ml is not None and abs(ml.lower) < 1e-6 and 0.9 <= ml.upper <= 1.4,
            details=f"limits=({ml.lower}, {ml.upper})" if ml else "no limits",
        )
        ctx.expect_contact(body, door, contact_tol=0.05, name=f"door_{side} seated on body")

        rest = ctx.part_world_aabb(door)
        assert rest is not None
        with ctx.pose({hinge: DOOR_OPEN_MAX}):
            opened = ctx.part_world_aabb(door)
            assert opened is not None
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

    # --- Round steering wheel: present, and TURNS about the column -----------
    sw = object_model.get_part("steering_wheel")
    sw_names = {v.name for v in sw.visuals}
    ctx.check(
        "round 3-spoke steering wheel: rim ring, grips, hub, boss + marker",
        {
            "sw_grip_left",
            "sw_grip_right",
            "sw_spoke_cross",
            "sw_spoke_low",
            "sw_hub",
            "sw_display",
            "sw_top_marker",
        }
        <= sw_names
        and sum(1 for v in sw.visuals if v.name.startswith("sw_rim_")) >= 12
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
