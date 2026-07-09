from __future__ import annotations

# WWII twin-engine attack bomber (Douglas A-26 Invader style) in bare-metal
# silver with a yellow band on the tall fin, US star-and-bar insignia, "RG-A"
# fuselage codes, tail number 322369, red "Stinky" nose art, gun nose, and
# olive-drab anti-glare panels on top of both engine nacelles.
#
# Frame conventions:
# - +X is the nose (flight) direction, +Y is the port (left) side, +Z is up.
# - Fuselage centerline sits at z = CL_Z. At rest each propeller holds one
#   blade straight up and two at +-120 deg, so the lowest blade tips graze the
#   ground plane z ~= 0.
# - Articulations: left and right propellers are independent CONTINUOUS spins
#   about each nacelle's +X axis; the dorsal gun turret is a CONTINUOUS
#   traverse about a vertical +Z axis on the fuselage spine.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoxGeometry,
    CylinderGeometry,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    sample_catmull_rom_spline_2d,
    section_loft,
)

# ---------------------------------------------------------------------------
# Global layout constants (meters)
# ---------------------------------------------------------------------------
CL_Z = 1.28  # fuselage centerline height above ground
TAIL_X = -8.0  # fuselage tail tip
NOSE_X = 7.5  # fuselage nose tip (hull length 15.5 m)
HALF_SPAN = 10.75  # wing half-span (21.5 m total)
WING_DZ = 0.15  # wing mid-plane above fuselage centerline

NAC_Y = 3.5  # nacelle lateral offset
NAC_DZ = -0.30  # nacelle axis below wing plane reference (rel. centerline)
NAC_Z = CL_Z + NAC_DZ  # world nacelle axis height = 0.98
NAC_FRONT_X = 3.4  # nacelle front (cowl) plane, world x
NAC_LEN = 5.1

PROP_TIP_R = 1.90  # propeller tip radius (3.8 m disc)
PROP_JOINT_DX = 0.10  # prop joint ahead of the nacelle front face

# Landing-gear layout (tricycle, fixed)
MAIN_GEAR_STATION = 3.50  # nacelle station (from front face) for main legs
MAIN_WHEEL_R = 0.32  # main tire radius
MAIN_WHEEL_W = 0.14  # main tire width
MAIN_STRUT_R = 0.040  # main oleo-strut outer radius

NOSE_GEAR_X = 5.50  # world x of nose-gear attachment
NOSE_WHEEL_R = 0.24  # nose tire radius
NOSE_WHEEL_W = 0.10  # nose tire width
NOSE_STRUT_R = 0.032  # nose oleo-strut outer radius

TURRET_X = 2.0
TURRET_DZ = 0.70  # turret ring base above centerline

FIN_TIP_Z = 3.2  # fin tip height above centerline
BAND_Z0, BAND_Z1 = 2.0, 2.5  # yellow fin band (rel. centerline)

# Fuselage hull lathe control profile: (radius, station) with the station
# measured from the tail tip (0) to the nose (15.5).
HULL_PROFILE = [
    (0.03, 0.00),
    (0.12, 0.90),
    (0.25, 2.40),
    (0.40, 4.40),
    (0.55, 6.60),
    (0.66, 8.60),
    (0.73, 10.60),
    (0.755, 12.00),
    (0.74, 13.20),
    (0.65, 14.30),
    (0.48, 15.05),
    (0.26, 15.38),
    (0.05, 15.50),
]


def _hull_radius_at_x(x: float) -> float:
    """Linear-interp hull radius at fuselage station x (centerline coords)."""
    s = x - TAIL_X
    pts = HULL_PROFILE
    if s <= pts[0][1]:
        return pts[0][0]
    for (r0, s0), (r1, s1) in zip(pts, pts[1:]):
        if s0 <= s <= s1:
            t = (s - s0) / (s1 - s0)
            return r0 + t * (r1 - r0)
    return pts[-1][0]


def _nacelle_radius_at(station_from_front: float) -> float:
    """Linear-interp nacelle radius at nacelle station measured from the front face."""
    s = NAC_LEN - station_from_front  # convert to lathe station
    pts = NACELLE_PROFILE
    if s <= pts[0][1]:
        return pts[0][0]
    for (r0, s0), (r1, s1) in zip(pts, pts[1:]):
        if s0 <= s <= s1:
            t = (s - s0) / (s1 - s0)
            return r0 + t * (r1 - r0)
    return pts[-1][0]


# ---------------------------------------------------------------------------
# Airfoil section helper
# ---------------------------------------------------------------------------
def _foil_pairs(front: float, back: float, ht: float) -> list[tuple[float, float]]:
    """Cambered airfoil loop (rounded LE, near-sharp TE) as (x, t) pairs."""
    c = front - back
    upper = [
        (0.00, 0.00),
        (0.03, 0.40),
        (0.08, 0.65),
        (0.18, 0.88),
        (0.32, 1.00),
        (0.50, 0.95),
        (0.70, 0.72),
        (0.88, 0.40),
        (1.00, 0.06),
    ]
    lower = [
        (1.00, -0.05),
        (0.88, -0.34),
        (0.70, -0.61),
        (0.50, -0.81),
        (0.32, -0.85),
        (0.18, -0.75),
        (0.08, -0.55),
        (0.03, -0.34),
    ]
    return [(front - u * c, f * ht) for u, f in upper + lower]


# ---------------------------------------------------------------------------
# Fuselage component meshes
# ---------------------------------------------------------------------------
def _hull_mesh() -> MeshGeometry:
    profile = sample_catmull_rom_spline_2d(HULL_PROFILE, samples_per_segment=8)
    profile = [(max(r, 0.0), s) for r, s in profile]
    hull = LatheGeometry(profile, segments=56)
    hull.rotate_y(math.pi / 2.0)  # lathe +Z axis -> +X (nose forward)
    hull.translate(TAIL_X, 0.0, 0.0)
    return hull


def _nose_cap_mesh() -> MeshGeometry:
    """Dark gun-nose cap shell over the front of the hull."""
    profile = [
        (0.58, 14.70),
        (0.50, 15.05),
        (0.29, 15.38),
        (0.11, 15.48),
        (0.01, 15.55),
    ]
    profile = sample_catmull_rom_spline_2d(profile, samples_per_segment=6)
    profile = [(max(r, 0.0), s) for r, s in profile]
    cap = LatheGeometry(profile, segments=48)
    cap.rotate_y(math.pi / 2.0)
    cap.translate(TAIL_X, 0.0, 0.0)
    return cap


def _nose_guns_mesh() -> MeshGeometry:
    """Six small black gun barrels protruding from the gun nose."""
    merged = MeshGeometry()
    for y, dz in ((0.12, 0.16), (-0.12, 0.16), (0.21, 0.0), (-0.21, 0.0), (0.12, -0.16), (-0.12, -0.16)):
        barrel = CylinderGeometry(0.028, 0.55, radial_segments=12)
        barrel.rotate_y(math.pi / 2.0)
        barrel.translate(7.45, y, dz)
        merged.merge(barrel)
    return merged


def _canopy_glass_mesh() -> MeshGeometry:
    canopy = SphereGeometry(1.0, width_segments=28, height_segments=18)
    canopy.scale(1.05, 0.46, 0.42)
    canopy.translate(4.30, 0.0, 0.62)
    return canopy


def _canopy_frames_mesh() -> MeshGeometry:
    """Silver framing ribs that make the canopy read as multi-pane glazing."""
    merged = MeshGeometry()
    for dx, sy, sz in ((-0.52, 0.41, 0.375), (0.0, 0.47, 0.43), (0.52, 0.41, 0.375)):
        rib = SphereGeometry(1.0, width_segments=20, height_segments=14)
        rib.scale(0.06, sy, sz)
        rib.translate(4.30 + dx, 0.0, 0.62)
        merged.merge(rib)
    spine = SphereGeometry(1.0, width_segments=20, height_segments=14)
    spine.scale(1.06, 0.06, 0.43)
    spine.translate(4.30, 0.0, 0.62)
    merged.merge(spine)
    return merged


def _dorsal_fairing_mesh() -> MeshGeometry:
    """Dorsal fin fillet blending the tall fin into the rear spine."""

    def loop(z: float, xf: float, xb: float, w: float):
        xm = (2.0 * xf + xb) / 3.0
        return [
            (xf + 0.05, 0.0, z),
            (xf, -0.55 * w, z),
            (xm, -w, z),
            (xb + 0.08, -0.7 * w, z),
            (xb, 0.0, z),
            (xb + 0.08, 0.7 * w, z),
            (xm, w, z),
            (xf, 0.55 * w, z),
        ]

    return section_loft(
        [
            loop(0.02, -4.60, -7.60, 0.07),
            loop(0.55, -5.60, -7.80, 0.05),
        ]
    )


# ---------------------------------------------------------------------------
# Wing / tail lofts
# ---------------------------------------------------------------------------
def _wing_le(y: float) -> float:
    return 2.0 - 0.04 * abs(y)


def _wing_chord(y: float) -> float:
    return 2.9 - 0.158 * abs(y)


def _wing_ht(y: float) -> float:
    return 0.145 - 0.0105 * abs(y)


def _wing_section(y: float, cs: float = 1.0, hs: float = 1.0) -> list[tuple[float, float, float]]:
    chord = _wing_chord(y)
    xc = _wing_le(y) - chord / 2.0
    c_eff = chord * cs
    pairs = _foil_pairs(xc + c_eff / 2.0, xc - c_eff / 2.0, _wing_ht(y) * hs)
    return [(x, y, WING_DZ + t) for x, t in pairs]


def _wing_mesh() -> MeshGeometry:
    specs = [
        (-HALF_SPAN, 0.45, 0.50),
        (-10.45, 0.85, 0.85),
        (-9.50, 1.0, 1.0),
        (-7.00, 1.0, 1.0),
        (-3.50, 1.0, 1.0),
        (0.00, 1.0, 1.0),
        (3.50, 1.0, 1.0),
        (7.00, 1.0, 1.0),
        (9.50, 1.0, 1.0),
        (10.45, 0.85, 0.85),
        (HALF_SPAN, 0.45, 0.50),
    ]
    return section_loft([_wing_section(y, cs, hs) for y, cs, hs in specs])


def _fin_le(z: float) -> float:
    return -5.8 - 0.45 * z


def _fin_te(z: float) -> float:
    return -8.0 + 0.05 * z


def _fin_ht(z: float) -> float:
    return max(0.025, 0.10 - 0.022 * z)


def _fin_section(z: float, cs: float = 1.0) -> list[tuple[float, float, float]]:
    chord = _fin_le(z) - _fin_te(z)
    xc = _fin_te(z) + chord / 2.0
    c_eff = chord * cs
    pairs = _foil_pairs(xc + c_eff / 2.0, xc - c_eff / 2.0, _fin_ht(z))
    return [(x, t, z) for x, t in pairs]


def _fin_lower_mesh() -> MeshGeometry:
    return section_loft([_fin_section(z) for z in (0.0, 0.8, 1.5, BAND_Z0)])


def _fin_band_mesh() -> MeshGeometry:
    return section_loft([_fin_section(z) for z in (BAND_Z0, 0.5 * (BAND_Z0 + BAND_Z1), BAND_Z1)])


def _fin_upper_mesh() -> MeshGeometry:
    return section_loft(
        [
            _fin_section(BAND_Z1),
            _fin_section(2.85),
            _fin_section(3.08, 0.85),
            _fin_section(FIN_TIP_Z, 0.5),
        ]
    )


def _stab_le(y: float) -> float:
    return -5.9 - 0.25 * abs(y)


def _stab_chord(y: float) -> float:
    return 1.9 - 0.26 * abs(y)


def _stab_ht(y: float) -> float:
    return max(0.022, 0.07 - 0.012 * abs(y))


def _stab_section(y: float, cs: float = 1.0) -> list[tuple[float, float, float]]:
    chord = _stab_chord(y)
    xc = _stab_le(y) - chord / 2.0
    c_eff = chord * cs
    pairs = _foil_pairs(xc + c_eff / 2.0, xc - c_eff / 2.0, _stab_ht(y))
    return [(x, y, 0.25 + t) for x, t in pairs]


def _stab_mesh() -> MeshGeometry:
    specs = [
        (-3.40, 0.50),
        (-3.28, 0.85),
        (-2.50, 1.0),
        (-1.20, 1.0),
        (0.00, 1.0),
        (1.20, 1.0),
        (2.50, 1.0),
        (3.28, 0.85),
        (3.40, 0.50),
    ]
    return section_loft([_stab_section(y, cs) for y, cs in specs])


# ---------------------------------------------------------------------------
# Nacelle / propeller meshes (shared construction, instantiated per side)
# ---------------------------------------------------------------------------
# Nacelle local frame: x = 0 at the cowl front face (+X forward), z = 0 on the
# nacelle axis.
NACELLE_PROFILE = [
    (0.04, 0.00),
    (0.15, 0.70),
    (0.30, 1.60),
    (0.46, 2.70),
    (0.58, 3.70),
    (0.635, 4.30),
    (0.635, 4.85),
    (0.60, NAC_LEN),
]


def _nacelle_hull_mesh() -> MeshGeometry:
    profile = sample_catmull_rom_spline_2d(NACELLE_PROFILE, samples_per_segment=8)
    profile = [(max(r, 0.0), s) for r, s in profile]
    body = LatheGeometry(profile, segments=48)
    body.rotate_y(math.pi / 2.0)  # +Z -> +X
    body.translate(-NAC_LEN, 0.0, 0.0)  # front face to local x = 0
    return body


def _cowl_ring_mesh() -> MeshGeometry:
    ring = CylinderGeometry(0.665, 0.24, radial_segments=40)
    ring.rotate_y(math.pi / 2.0)
    ring.translate(-0.12, 0.0, 0.0)
    return ring


def _engine_face_mesh() -> MeshGeometry:
    # Spans the nacelle front cap plane (x = 0) so it stays surface-connected
    # to the nacelle hull.
    face = CylinderGeometry(0.50, 0.12, radial_segments=32)
    face.rotate_y(math.pi / 2.0)
    face.translate(-0.04, 0.0, 0.0)
    return face


def _antiglare_panel_mesh() -> MeshGeometry:
    panel = BoxGeometry((1.90, 0.60, 0.05))
    panel.translate(-1.15, 0.0, 0.545)
    return panel


def _spinner_mesh() -> MeshGeometry:
    profile = [
        (0.010, 0.00),  # closed backplate so the shaft stays connected
        (0.298, 0.005),
        (0.300, 0.02),
        (0.295, 0.10),
        (0.260, 0.26),
        (0.195, 0.42),
        (0.100, 0.55),
        (0.010, 0.62),
    ]
    spinner = LatheGeometry(profile, segments=40)
    spinner.rotate_y(math.pi / 2.0)
    spinner.translate(0.02, 0.0, 0.0)
    return spinner


def _prop_shaft_mesh() -> MeshGeometry:
    shaft = CylinderGeometry(0.07, 0.30, radial_segments=20)
    shaft.rotate_y(math.pi / 2.0)
    shaft.translate(-0.05, 0.0, 0.0)  # spans x in [-0.20, 0.10]
    return shaft


# Blade sections: (radius, chord, half_thickness, twist_rad)
_BLADE_SPECS = [
    (0.18, 0.16, 0.085, 1.05),
    (0.40, 0.30, 0.060, 0.80),
    (0.80, 0.34, 0.045, 0.55),
    (1.20, 0.30, 0.035, 0.42),
    (1.60, 0.24, 0.028, 0.34),
    (1.82, 0.16, 0.020, 0.30),
    (PROP_TIP_R, 0.06, 0.010, 0.28),
]


def _blade_loft() -> MeshGeometry:
    sections = []
    for r, c, ht, tw in _BLADE_SPECS:
        pairs = _foil_pairs(c / 2.0, -c / 2.0, ht)
        loop = []
        for a, t in pairs:
            # chord coordinate a (in the rotation plane), thickness t; twist tw
            # tilts the chord toward the prop axis (+X) about the radial axis.
            x = a * math.sin(tw) + t * math.cos(tw)
            y = a * math.cos(tw) - t * math.sin(tw)
            loop.append((x, y, r))
        sections.append(loop)
    return section_loft(sections)


def _propeller_blades_mesh() -> MeshGeometry:
    """Three black blades; at rest one points straight up (+Z)."""
    base = _blade_loft()
    base.translate(0.20, 0.0, 0.0)  # blade plane just behind spinner mid-body
    merged = MeshGeometry()
    for k in range(3):
        blade = base.copy()
        blade.rotate_x(k * 2.0 * math.pi / 3.0)
        merged.merge(blade)
    return merged


# ---------------------------------------------------------------------------
# Landing-gear meshes (origin at attachment point, strut extends -Z)
# ---------------------------------------------------------------------------
def _gear_strut_mesh(strut_length: float, strut_r: float) -> MeshGeometry:
    """Oleo strut: outer tube + inner slider. Origin at attachment, -Z down."""
    merged = MeshGeometry()
    outer_len = strut_length * 0.55
    outer = CylinderGeometry(strut_r * 1.40, outer_len, radial_segments=16)
    outer.translate(0.0, 0.0, -outer_len / 2.0)
    merged.merge(outer)
    inner_len = strut_length - outer_len
    inner = CylinderGeometry(strut_r, inner_len, radial_segments=16)
    inner.translate(0.0, 0.0, -outer_len - inner_len / 2.0)
    merged.merge(inner)
    # Small collar where the strut meets the parent skin
    collar = CylinderGeometry(strut_r * 2.0, strut_r * 1.6, radial_segments=16)
    collar.translate(0.0, 0.0, -strut_r * 0.8)
    merged.merge(collar)
    return merged


def _gear_wheel_mesh(strut_length: float, wheel_r: float, wheel_w: float) -> MeshGeometry:
    """Tire + hub at the bottom of the strut (centered at z = -strut_length)."""
    merged = MeshGeometry()
    # Axle stub (lateral, along Y)
    axle = CylinderGeometry(wheel_r * 0.12, wheel_w * 1.40, radial_segments=12)
    axle.rotate_x(math.pi / 2.0)
    axle.translate(0.0, 0.0, -strut_length)
    merged.merge(axle)
    # Tire
    tire = CylinderGeometry(wheel_r, wheel_w, radial_segments=28)
    tire.rotate_x(math.pi / 2.0)
    tire.translate(0.0, 0.0, -strut_length)
    merged.merge(tire)
    # Hub cap (slightly proud of the tire sidewall)
    hub = CylinderGeometry(wheel_r * 0.38, wheel_w * 0.30, radial_segments=20)
    hub.rotate_x(math.pi / 2.0)
    hub.translate(0.0, 0.0, -strut_length)
    merged.merge(hub)
    return merged


# ---------------------------------------------------------------------------
# Turret meshes (turret local frame: z = 0 at ring base on the spine)
# ---------------------------------------------------------------------------
def _turret_ring_mesh() -> MeshGeometry:
    # Wider than the dome so the base ring stays surface-connected to it.
    ring = CylinderGeometry(0.50, 0.14, radial_segments=32)
    ring.translate(0.0, 0.0, -0.03)
    return ring


def _turret_dome_mesh() -> MeshGeometry:
    dome = SphereGeometry(1.0, width_segments=28, height_segments=18)
    dome.scale(0.46, 0.46, 0.34)
    dome.translate(0.0, 0.0, 0.02)
    return dome


def _turret_guns_mesh() -> MeshGeometry:
    merged = MeshGeometry()
    for y in (0.10, -0.10):
        gun = CylinderGeometry(0.024, 0.85, radial_segments=12)
        gun.rotate_y(math.pi / 2.0)
        gun.translate(-0.55, y, 0.14)  # stowed pointing aft (-X)
        merged.merge(gun)
    return merged


# ---------------------------------------------------------------------------
# Markings: stroke-font glyphs, star-and-bar insignia
# ---------------------------------------------------------------------------
# Glyph strokes in a unit cell (x in [0, 0.66], y in [0, 1]).
_SEG = {
    "A": ((0.0, 1.0), (0.66, 1.0)),
    "B": ((0.66, 0.5), (0.66, 1.0)),
    "C": ((0.66, 0.0), (0.66, 0.5)),
    "D": ((0.0, 0.0), (0.66, 0.0)),
    "E": ((0.0, 0.0), (0.0, 0.5)),
    "F": ((0.0, 0.5), (0.0, 1.0)),
    "G": ((0.0, 0.5), (0.66, 0.5)),
}

_GLYPHS: dict[str, list[tuple[tuple[float, float], tuple[float, float]]]] = {
    "2": [_SEG["A"], _SEG["B"], _SEG["G"], _SEG["E"], _SEG["D"]],
    "3": [_SEG["A"], _SEG["B"], _SEG["G"], _SEG["C"], _SEG["D"]],
    "6": [_SEG["A"], _SEG["F"], _SEG["E"], _SEG["D"], _SEG["C"], _SEG["G"]],
    "9": [_SEG["A"], _SEG["F"], _SEG["B"], _SEG["G"], _SEG["C"], _SEG["D"]],
    "R": [
        _SEG["F"],
        _SEG["E"],
        _SEG["A"],
        _SEG["B"],
        _SEG["G"],
        ((0.30, 0.5), (0.66, 0.0)),
    ],
    "G": [
        _SEG["A"],
        _SEG["F"],
        _SEG["E"],
        _SEG["D"],
        ((0.66, 0.0), (0.66, 0.42)),
        ((0.36, 0.42), (0.66, 0.42)),
    ],
    "A": [_SEG["F"], _SEG["E"], _SEG["B"], _SEG["C"], _SEG["A"], _SEG["G"]],
    "S": [_SEG["A"], _SEG["F"], _SEG["G"], _SEG["C"], _SEG["D"]],
    "T": [_SEG["A"], ((0.33, 0.0), (0.33, 1.0))],
    "I": [((0.33, 0.0), (0.33, 1.0))],
    "N": [
        _SEG["F"],
        _SEG["E"],
        _SEG["B"],
        _SEG["C"],
        ((0.0, 1.0), (0.66, 0.0)),
    ],
    "K": [
        _SEG["F"],
        _SEG["E"],
        ((0.0, 0.5), (0.66, 1.0)),
        ((0.0, 0.5), (0.66, 0.0)),
    ],
    "Y": [
        ((0.0, 1.0), (0.33, 0.55)),
        ((0.66, 1.0), (0.33, 0.55)),
        ((0.33, 0.55), (0.33, 0.0)),
    ],
}


def _glyph_mesh(ch: str, h: float, t: float) -> MeshGeometry:
    """One glyph in the XY plane, centered at the origin, thickness +Z."""
    s = 0.16 * h
    mesh = MeshGeometry()
    for (x0, y0), (x1, y1) in _GLYPHS[ch]:
        x0, y0, x1, y1 = x0 * h, y0 * h, x1 * h, y1 * h
        dx, dy = x1 - x0, y1 - y0
        length = math.hypot(dx, dy)
        box = BoxGeometry((length + s, s, t))
        box.rotate_z(math.atan2(dy, dx))
        box.translate((x0 + x1) / 2.0 - 0.33 * h, (y0 + y1) / 2.0 - 0.5 * h, t / 2.0)
        mesh.merge(box)
    return mesh


def _flank_text_mesh(text: str, h: float, t: float, pitch: float, x_start: float, z_rel: float, side: float) -> MeshGeometry:
    """Text wrapped onto one fuselage flank (side=+1 port, -1 starboard)."""
    mesh = MeshGeometry()
    for i, ch in enumerate(text):
        x_i = x_start - i * pitch
        r_loc = _hull_radius_at_x(x_i)
        g = _glyph_mesh(ch, h, t)
        g.rotate_x(math.pi / 2.0)  # height -> +Z, thickness -> [-t, 0] in Y
        dy = (r_loc + 0.02) if side > 0 else -(r_loc - 0.03)
        g.translate(x_i, dy, z_rel)
        mesh.merge(g)
    return mesh


def _fin_text_mesh(text: str, h: float, t: float, pitch: float, x_start: float, z_rel: float, side: float) -> MeshGeometry:
    """Digits embedded deep into the fin so they stay connected near the thin TE."""
    mesh = MeshGeometry()
    ht = _fin_ht(z_rel)
    proud = 0.025
    for i, ch in enumerate(text):
        x_i = x_start - i * pitch
        g = _glyph_mesh(ch, h, t)
        g.rotate_x(math.pi / 2.0)  # thickness occupies [-t, 0] in Y
        dy = (ht + proud) if side > 0 else -(ht + proud - t)
        g.translate(x_i, dy, z_rel)
        mesh.merge(g)
    return mesh


def _star_loop(r_out: float, z: float) -> list[tuple[float, float, float]]:
    pts = []
    for k in range(10):
        ang = math.pi / 2.0 + k * math.pi / 5.0
        r = r_out if k % 2 == 0 else 0.382 * r_out
        pts.append((r * math.cos(ang), r * math.sin(ang), z))
    return pts


def _star_prism(r_out: float, t: float) -> MeshGeometry:
    return section_loft([_star_loop(r_out, 0.0), _star_loop(r_out, t)])


def _insignia_blue_mesh(r: float, t: float) -> MeshGeometry:
    """Blue disc + blue bar field, in the XY plane, star point toward +Y."""
    mesh = MeshGeometry()
    disc = CylinderGeometry(r, t, radial_segments=36)
    disc.translate(0.0, 0.0, t / 2.0)
    mesh.merge(disc)
    bar = BoxGeometry((3.7 * r, 0.92 * r, t))
    bar.translate(0.0, 0.0, t / 2.0)
    mesh.merge(bar)
    return mesh


def _insignia_white_mesh(r: float, t: float) -> MeshGeometry:
    """White star + white side bars, slightly proud of the blue field."""
    mesh = MeshGeometry()
    mesh.merge(_star_prism(0.88 * r, t))
    for sx in (1.0, -1.0):
        wbar = BoxGeometry((1.0 * r, 0.50 * r, t))
        wbar.translate(sx * 1.30 * r, 0.0, t / 2.0)
        mesh.merge(wbar)
    return mesh


T_BLUE = 0.030
T_WHITE = 0.042


def _fuselage_insignia(side: float, x: float, z_rel: float, r: float) -> tuple[MeshGeometry, MeshGeometry]:
    r_loc = _hull_radius_at_x(x)
    out = []
    for builder, t in ((_insignia_blue_mesh, T_BLUE), (_insignia_white_mesh, T_WHITE)):
        m = builder(r, t)
        m.rotate_x(math.pi / 2.0)  # star point -> +Z, thickness -> [-t, 0] in Y
        dy = (r_loc + 0.02) if side > 0 else -(r_loc - 0.03)
        m.translate(x, dy, z_rel)
        out.append(m)
    return out[0], out[1]


def _wing_top_insignia(x: float, y: float, z_base: float) -> tuple[MeshGeometry, MeshGeometry]:
    out = []
    for builder, t in ((_insignia_blue_mesh, T_BLUE), (_insignia_white_mesh, T_WHITE)):
        m = builder(0.62, t)
        m.rotate_z(-math.pi / 2.0)  # star point toward +X (leading edge)
        m.translate(x, y, z_base)
        out.append(m)
    return out[0], out[1]


def _wing_bottom_insignia(x: float, y: float, z_low: float) -> tuple[MeshGeometry, MeshGeometry]:
    out = []
    for builder, t in ((_insignia_blue_mesh, T_BLUE), (_insignia_white_mesh, T_WHITE)):
        m = builder(0.62, t)
        m.rotate_z(math.pi / 2.0)  # point -X first ...
        m.rotate_y(math.pi)  # ... flipped to face down, point back to +X
        m.translate(x, y, z_low)
        out.append(m)
    return out[0], out[1]


# ---------------------------------------------------------------------------
# Model assembly
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="a26_invader_attack_bomber")

    silver = model.material("bare_metal_silver", rgba=(0.76, 0.77, 0.79, 1.0))
    chrome = model.material("chrome_cowl", rgba=(0.86, 0.88, 0.91, 1.0))
    olive = model.material("olive_drab", rgba=(0.30, 0.31, 0.18, 1.0))
    yellow = model.material("tail_yellow", rgba=(0.94, 0.78, 0.15, 1.0))
    black = model.material("gun_black", rgba=(0.06, 0.06, 0.07, 1.0))
    dark = model.material("dark_grey", rgba=(0.16, 0.16, 0.17, 1.0))
    glass = model.material("canopy_glass", rgba=(0.18, 0.26, 0.36, 1.0))
    blue = model.material("insignia_blue", rgba=(0.10, 0.16, 0.38, 1.0))
    white = model.material("insignia_white", rgba=(0.94, 0.94, 0.94, 1.0))
    red = model.material("nose_art_red", rgba=(0.75, 0.10, 0.10, 1.0))

    cl = Origin(xyz=(0.0, 0.0, CL_Z))

    # --- fuselage (root) ---
    fuselage = model.part("fuselage")
    fuselage.visual(mesh_from_geometry(_hull_mesh(), "fuselage_hull"), origin=cl, material=silver, name="hull")
    fuselage.visual(
        mesh_from_geometry(_nose_cap_mesh(), "gun_nose_cap"), origin=cl, material=dark, name="nose_cap"
    )
    fuselage.visual(
        mesh_from_geometry(_nose_guns_mesh(), "nose_gun_barrels"), origin=cl, material=black, name="nose_guns"
    )
    fuselage.visual(
        mesh_from_geometry(_canopy_glass_mesh(), "canopy_glass"), origin=cl, material=glass, name="canopy"
    )
    fuselage.visual(
        mesh_from_geometry(_canopy_frames_mesh(), "canopy_frames"), origin=cl, material=silver, name="canopy_frames"
    )
    fuselage.visual(
        mesh_from_geometry(_dorsal_fairing_mesh(), "dorsal_fairing"), origin=cl, material=silver, name="dorsal_fairing"
    )

    # fuselage codes "RG (star) A" on both flanks, aft of the wing trailing edge
    for side, tag in ((1.0, "port"), (-1.0, "starboard")):
        codes_fwd = _flank_text_mesh("RG", 0.44, 0.05, 0.56, -1.45, 0.0, side)
        fuselage.visual(
            mesh_from_geometry(codes_fwd, f"code_rg_{tag}"), origin=cl, material=black, name=f"code_rg_{tag}"
        )
        code_a = _flank_text_mesh("A", 0.44, 0.05, 0.56, -4.25, 0.0, side)
        fuselage.visual(
            mesh_from_geometry(code_a, f"code_a_{tag}"), origin=cl, material=black, name=f"code_a_{tag}"
        )
        ins_b, ins_w = _fuselage_insignia(side, -3.0, 0.0, 0.34)
        fuselage.visual(
            mesh_from_geometry(ins_b, f"fus_insignia_blue_{tag}"),
            origin=cl,
            material=blue,
            name=f"fuselage_star_blue_{tag}",
        )
        fuselage.visual(
            mesh_from_geometry(ins_w, f"fus_insignia_white_{tag}"),
            origin=cl,
            material=white,
            name=f"fuselage_star_white_{tag}",
        )

    # red "STINKY" nose art on the port nose flank
    stinky = _flank_text_mesh("STINKY", 0.26, 0.045, 0.24, 6.35, 0.10, 1.0)
    fuselage.visual(mesh_from_geometry(stinky, "nose_art_stinky"), origin=cl, material=red, name="nose_art")

    # --- wing (one-piece, fixed) ---
    wing = model.part("wing")
    wing.visual(mesh_from_geometry(_wing_mesh(), "wing_loft"), material=silver, name="wing_loft")
    wt_b, wt_w = _wing_top_insignia(0.80, 6.60, WING_DZ + 0.035)
    wing.visual(mesh_from_geometry(wt_b, "wing_star_blue_top"), material=blue, name="wing_star_blue_top")
    wing.visual(mesh_from_geometry(wt_w, "wing_star_white_top"), material=white, name="wing_star_white_top")
    wb_b, wb_w = _wing_bottom_insignia(0.80, -6.60, WING_DZ - 0.035)
    wing.visual(mesh_from_geometry(wb_b, "wing_star_blue_bottom"), material=blue, name="wing_star_blue_bottom")
    wing.visual(mesh_from_geometry(wb_w, "wing_star_white_bottom"), material=white, name="wing_star_white_bottom")
    model.articulation("fuselage_to_wing", ArticulationType.FIXED, parent=fuselage, child=wing, origin=cl)

    # --- nacelles (fixed under each wing) and independent propellers ---
    def add_nacelle_and_prop(side: float, tag: str):
        nacelle = model.part(f"{tag}_nacelle")
        nacelle.visual(
            mesh_from_geometry(_nacelle_hull_mesh(), f"{tag}_nacelle_hull"),
            material=silver,
            name="nacelle_hull",
        )
        nacelle.visual(
            mesh_from_geometry(_cowl_ring_mesh(), f"{tag}_cowl_ring"), material=chrome, name="cowl_ring"
        )
        nacelle.visual(
            mesh_from_geometry(_engine_face_mesh(), f"{tag}_engine_face"), material=dark, name="engine_face"
        )
        nacelle.visual(
            mesh_from_geometry(_antiglare_panel_mesh(), f"{tag}_antiglare"), material=olive, name="antiglare_panel"
        )
        model.articulation(
            f"wing_to_{tag}_nacelle",
            ArticulationType.FIXED,
            parent=wing,
            child=nacelle,
            origin=Origin(xyz=(NAC_FRONT_X, side * NAC_Y, NAC_DZ)),
        )

        prop = model.part(f"{tag}_propeller")
        prop.visual(
            mesh_from_geometry(_spinner_mesh(), f"{tag}_spinner"), material=dark, name="spinner"
        )
        prop.visual(
            mesh_from_geometry(_prop_shaft_mesh(), f"{tag}_prop_shaft"), material=dark, name="prop_shaft"
        )
        prop.visual(
            mesh_from_geometry(_propeller_blades_mesh(), f"{tag}_prop_blades"), material=black, name="blades"
        )
        model.articulation(
            f"{tag}_prop_spin",
            ArticulationType.CONTINUOUS,
            parent=nacelle,
            child=prop,
            origin=Origin(xyz=(PROP_JOINT_DX, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=600.0, velocity=120.0),
        )
        return nacelle

    left_nac = add_nacelle_and_prop(+1.0, "left")
    right_nac = add_nacelle_and_prop(-1.0, "right")

    # --- tricycle landing gear (fixed) ---
    gear_steel = model.material("gear_steel", rgba=(0.42, 0.43, 0.45, 1.0))
    tire_rubber = model.material("tire_rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    nac_r = _nacelle_radius_at(MAIN_GEAR_STATION)
    main_attach_z = NAC_Z - nac_r
    main_strut_len = main_attach_z - MAIN_WHEEL_R

    main_gear_specs = [
        (+1.0, "left", left_nac),
        (-1.0, "right", right_nac),
    ]
    for i, (side, tag, nac_parent) in enumerate(main_gear_specs):
        gear = model.part(f"gear_{i}")
        gear.visual(
            mesh_from_geometry(_gear_strut_mesh(main_strut_len, MAIN_STRUT_R), f"gear_{i}_strut"),
            material=gear_steel,
            name=f"gear_{i}_strut",
        )
        gear.visual(
            mesh_from_geometry(_gear_wheel_mesh(main_strut_len, MAIN_WHEEL_R, MAIN_WHEEL_W), f"gear_{i}_wheel"),
            material=tire_rubber,
            name=f"gear_{i}_wheel",
        )
        model.articulation(
            f"nacelle_to_gear_{i}",
            ArticulationType.FIXED,
            parent=nac_parent,
            child=gear,
            origin=Origin(xyz=(-MAIN_GEAR_STATION, 0.0, -nac_r)),
        )

    # Nose gear
    nose_r = _hull_radius_at_x(NOSE_GEAR_X)
    nose_attach_z = CL_Z - nose_r
    nose_strut_len = nose_attach_z - NOSE_WHEEL_R

    nose_gear = model.part("nose_gear")
    nose_gear.visual(
        mesh_from_geometry(_gear_strut_mesh(nose_strut_len, NOSE_STRUT_R), "nose_gear_strut"),
        material=gear_steel,
        name="nose_gear_strut",
    )
    nose_gear.visual(
        mesh_from_geometry(_gear_wheel_mesh(nose_strut_len, NOSE_WHEEL_R, NOSE_WHEEL_W), "nose_gear_wheel"),
        material=tire_rubber,
        name="nose_gear_wheel",
    )
    model.articulation(
        "fuselage_to_nose_gear",
        ArticulationType.FIXED,
        parent=fuselage,
        child=nose_gear,
        origin=Origin(xyz=(NOSE_GEAR_X, 0.0, CL_Z - nose_r)),
    )

    # --- dorsal gun turret (continuous traverse about vertical axis) ---
    turret = model.part("gun_turret")
    turret.visual(mesh_from_geometry(_turret_ring_mesh(), "turret_ring"), material=dark, name="ring")
    turret.visual(mesh_from_geometry(_turret_dome_mesh(), "turret_dome"), material=glass, name="dome")
    turret.visual(mesh_from_geometry(_turret_guns_mesh(), "turret_guns"), material=black, name="guns")
    model.articulation(
        "turret_traverse",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=turret,
        origin=Origin(xyz=(TURRET_X, 0.0, CL_Z + TURRET_DZ)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=4.0),
    )

    # --- vertical fin with yellow band and tail number ---
    tail_fin = model.part("tail_fin")
    tail_fin.visual(mesh_from_geometry(_fin_lower_mesh(), "fin_lower"), material=silver, name="fin_lower")
    tail_fin.visual(mesh_from_geometry(_fin_band_mesh(), "fin_band"), material=yellow, name="fin_band")
    tail_fin.visual(mesh_from_geometry(_fin_upper_mesh(), "fin_upper"), material=silver, name="fin_upper")
    for side, tag in ((1.0, "port"), (-1.0, "starboard")):
        num = _fin_text_mesh("322369", 0.20, 0.10, 0.17, -6.70, 1.55, side)
        tail_fin.visual(
            mesh_from_geometry(num, f"tail_number_{tag}"), material=black, name=f"tail_number_{tag}"
        )
    model.articulation("fuselage_to_tail_fin", ArticulationType.FIXED, parent=fuselage, child=tail_fin, origin=cl)

    # --- horizontal stabilizer ---
    stabilizer = model.part("horizontal_stabilizer")
    stabilizer.visual(mesh_from_geometry(_stab_mesh(), "stabilizer_loft"), material=silver, name="stabilizer_loft")
    model.articulation(
        "fuselage_to_stabilizer", ArticulationType.FIXED, parent=fuselage, child=stabilizer, origin=cl
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    fuselage = object_model.get_part("fuselage")
    wing = object_model.get_part("wing")
    left_nacelle = object_model.get_part("left_nacelle")
    right_nacelle = object_model.get_part("right_nacelle")
    left_prop = object_model.get_part("left_propeller")
    right_prop = object_model.get_part("right_propeller")
    turret = object_model.get_part("gun_turret")
    tail_fin = object_model.get_part("tail_fin")
    stabilizer = object_model.get_part("horizontal_stabilizer")
    gear_0 = object_model.get_part("gear_0")
    gear_1 = object_model.get_part("gear_1")
    nose_gear = object_model.get_part("nose_gear")

    left_spin = object_model.get_articulation("left_prop_spin")
    right_spin = object_model.get_articulation("right_prop_spin")
    traverse = object_model.get_articulation("turret_traverse")

    # --- intentional, scoped overlaps (real construction junctions) ---
    ctx.allow_overlap(
        wing,
        fuselage,
        elem_a="wing_loft",
        elem_b="hull",
        reason="One-piece mid wing carries through the fuselage, as on the real A-26.",
    )
    ctx.allow_overlap(
        tail_fin,
        fuselage,
        elem_a="fin_lower",
        elem_b="hull",
        reason="Fin root spar is buried in the tail cone.",
    )
    ctx.allow_overlap(
        tail_fin,
        fuselage,
        elem_a="fin_lower",
        elem_b="dorsal_fairing",
        reason="Fin root seats into the dorsal fillet.",
    )
    ctx.allow_overlap(
        stabilizer,
        fuselage,
        elem_a="stabilizer_loft",
        elem_b="hull",
        reason="One-piece stabilizer carries through the tail cone.",
    )
    ctx.allow_overlap(
        stabilizer,
        fuselage,
        elem_a="stabilizer_loft",
        elem_b="dorsal_fairing",
        reason="Stabilizer root blends into the dorsal fillet.",
    )
    ctx.allow_overlap(
        stabilizer,
        tail_fin,
        elem_a="stabilizer_loft",
        elem_b="fin_lower",
        reason="Stabilizer and fin roots share the tail-cone junction.",
    )
    for nacelle in (left_nacelle, right_nacelle):
        ctx.allow_overlap(
            nacelle,
            wing,
            elem_a="nacelle_hull",
            elem_b="wing_loft",
            reason="Engine nacelle is slung through the wing structure.",
        )
        ctx.allow_overlap(
            nacelle,
            wing,
            elem_a="antiglare_panel",
            elem_b="wing_loft",
            reason="Anti-glare panel fairs into the wing leading-edge root.",
        )
    for prop, nacelle in ((left_prop, left_nacelle), (right_prop, right_nacelle)):
        for elem_b in ("engine_face", "cowl_ring", "nacelle_hull"):
            ctx.allow_overlap(
                prop,
                nacelle,
                elem_a="prop_shaft",
                elem_b=elem_b,
                reason="Propeller shaft is seated in the engine gearbox bore.",
            )
        ctx.allow_overlap(
            prop,
            nacelle,
            elem_a="spinner",
            elem_b="cowl_ring",
            reason="Spinner base nests just inside the cowl ring lip.",
        )
    for elem_a in ("ring", "dome"):
        ctx.allow_overlap(
            turret,
            fuselage,
            elem_a=elem_a,
            elem_b="hull",
            reason="Turret ring and dome base are seated in the fuselage spine cutout.",
        )

    # Landing gear: strut passes through the nacelle/fuselage gear-bay opening
    for i, nac_parent in enumerate((left_nacelle, right_nacelle)):
        ctx.allow_overlap(
            nac_parent,
            object_model.get_part(f"gear_{i}"),
            elem_a="nacelle_hull",
            elem_b=f"gear_{i}_strut",
            reason="Main gear oleo strut passes through the nacelle gear-bay opening.",
        )
    ctx.allow_overlap(
        fuselage,
        nose_gear,
        elem_a="hull",
        elem_b="nose_gear_strut",
        reason="Nose gear strut passes through the fuselage gear-bay opening.",
    )

    # --- joint plan: two independent continuous props + continuous turret ---
    for spin, tag in ((left_spin, "left"), (right_spin, "right")):
        ctx.check(
            f"{tag} propeller joint is continuous about the nacelle +X axis",
            spin.articulation_type == ArticulationType.CONTINUOUS
            and abs(spin.axis[0] - 1.0) < 1e-6
            and abs(spin.axis[1]) < 1e-6
            and abs(spin.axis[2]) < 1e-6
            and spin.motion_limits is not None
            and spin.motion_limits.lower is None
            and spin.motion_limits.upper is None
            and spin.mimic is None,
            details=f"type={spin.articulation_type}, axis={spin.axis}, mimic={spin.mimic}",
        )
    ctx.check(
        "propellers are independent links on separate joints",
        left_spin.child != right_spin.child and left_spin.parent != right_spin.parent,
        details=f"left={left_spin.parent}->{left_spin.child}, right={right_spin.parent}->{right_spin.child}",
    )
    ctx.check(
        "turret traverse is continuous about the vertical +Z axis on the fuselage",
        traverse.articulation_type == ArticulationType.CONTINUOUS
        and abs(traverse.axis[2] - 1.0) < 1e-6
        and abs(traverse.axis[0]) < 1e-6
        and abs(traverse.axis[1]) < 1e-6
        and traverse.motion_limits is not None
        and traverse.motion_limits.lower is None
        and traverse.motion_limits.upper is None,
        details=f"type={traverse.articulation_type}, axis={traverse.axis}",
    )

    # --- real-world scale ---
    hull_aabb = ctx.part_element_world_aabb(fuselage, elem="hull")
    length = hull_aabb[1][0] - hull_aabb[0][0]
    ctx.check("fuselage length about 15.5 m", 15.2 < length < 15.8, details=f"length={length:.3f}")
    wing_aabb = ctx.part_world_aabb(wing)
    span = wing_aabb[1][1] - wing_aabb[0][1]
    ctx.check("wingspan about 21.5 m", 21.2 < span < 21.8, details=f"span={span:.3f}")

    # --- grounding: lowest geometry (wheels + blade tips at rest) grazes z ~ 0 ---
    all_parts = (
        fuselage,
        wing,
        left_nacelle,
        right_nacelle,
        left_prop,
        right_prop,
        turret,
        tail_fin,
        stabilizer,
        gear_0,
        gear_1,
        nose_gear,
    )
    zmin = min(ctx.part_world_aabb(p)[0][2] for p in all_parts)
    ctx.check("rest pose grazes the ground plane", -0.02 <= zmin <= 0.12, details=f"zmin={zmin:.3f}")

    # --- propeller geometry: 3.8 m disc, spinner ahead of the chrome cowl ---
    lp_aabb = ctx.part_world_aabb(left_prop)
    tip_reach = lp_aabb[1][2] - NAC_Z
    ctx.check(
        "blade tip radius about 1.9 m (one blade straight up at rest)",
        1.82 < tip_reach < 1.98,
        details=f"tip_reach={tip_reach:.3f}",
    )
    spinner_aabb = ctx.part_element_world_aabb(left_prop, elem="spinner")
    ctx.check(
        "spinner projects ahead of the nacelle front",
        spinner_aabb is not None and spinner_aabb[1][0] > NAC_FRONT_X + 0.5,
        details=f"spinner_xmax={spinner_aabb[1][0]:.3f}" if spinner_aabb else "missing",
    )
    ctx.check(
        "left prop disc clears the fuselage flank",
        lp_aabb[0][1] > 0.9,
        details=f"prop_ymin={lp_aabb[0][1]:.3f}",
    )

    # --- off-axis proof: spinning one prop swings its blades, not the other ---
    rp_rest = ctx.part_world_aabb(right_prop)
    with ctx.pose({left_spin: math.pi / 3.0}):
        lp_spun = ctx.part_world_aabb(left_prop)
        rp_same = ctx.part_world_aabb(right_prop)
        ctx.check(
            "left prop at 60 deg swings a blade tip well below the rest envelope",
            lp_spun[0][2] < lp_aabb[0][2] - 0.5,
            details=f"rest_zmin={lp_aabb[0][2]:.3f}, spun_zmin={lp_spun[0][2]:.3f}",
        )
        ctx.check(
            "right prop is unaffected by the left prop joint (independent)",
            abs(rp_same[0][2] - rp_rest[0][2]) < 1e-9 and abs(rp_same[1][1] - rp_rest[1][1]) < 1e-9,
            details=f"rest={rp_rest[0][2]:.4f}, posed={rp_same[0][2]:.4f}",
        )

    # --- turret: dome bump on the spine; traverse swings the stowed guns ---
    dome_aabb = ctx.part_element_world_aabb(turret, elem="dome")
    ctx.check(
        "turret dome bulges above the fuselage spine",
        dome_aabb is not None and dome_aabb[1][2] > CL_Z + 0.95,
        details=f"dome_zmax={dome_aabb[1][2]:.3f}" if dome_aabb else "missing",
    )
    tur_rest = ctx.part_world_aabb(turret)
    with ctx.pose({traverse: math.pi / 2.0}):
        tur_yaw = ctx.part_world_aabb(turret)
        lateral_growth = max(tur_rest[0][1] - tur_yaw[0][1], tur_yaw[1][1] - tur_rest[1][1])
        ctx.check(
            "90 deg traverse swings the stowed gun barrels from -X to lateral",
            lateral_growth > 0.4 and (tur_yaw[0][0] - tur_rest[0][0]) > 0.4,
            details=f"rest_x=({tur_rest[0][0]:.2f},{tur_rest[1][0]:.2f}) y=({tur_rest[0][1]:.2f},{tur_rest[1][1]:.2f}); "
            f"yaw_x=({tur_yaw[0][0]:.2f},{tur_yaw[1][0]:.2f}) y=({tur_yaw[0][1]:.2f},{tur_yaw[1][1]:.2f})",
        )

    # --- gun nose ---
    guns_aabb = ctx.part_element_world_aabb(fuselage, elem="nose_guns")
    ctx.check(
        "gun barrels protrude past the nose tip",
        guns_aabb is not None and guns_aabb[1][0] > NOSE_X + 0.1,
        details=f"guns_xmax={guns_aabb[1][0]:.3f}" if guns_aabb else "missing",
    )

    # --- canopy ---
    canopy_aabb = ctx.part_element_world_aabb(fuselage, elem="canopy")
    ctx.check(
        "glazed canopy rises above the fuselage spine",
        canopy_aabb is not None and canopy_aabb[1][2] > CL_Z + 0.90,
        details=f"canopy_zmax={canopy_aabb[1][2]:.3f}" if canopy_aabb else "missing",
    )

    # --- tall fin with yellow band ---
    band_aabb = ctx.part_element_world_aabb(tail_fin, elem="fin_band")
    ctx.check(
        "yellow band sits high on the fin",
        band_aabb is not None
        and band_aabb[0][2] > CL_Z + BAND_Z0 - 0.15
        and band_aabb[1][2] < CL_Z + BAND_Z1 + 0.15,
        details=f"band_z=({band_aabb[0][2]:.2f},{band_aabb[1][2]:.2f})" if band_aabb else "missing",
    )
    fin_aabb = ctx.part_world_aabb(tail_fin)
    ctx.check(
        "fin tip is the tallest fixed structure (~4.5 m up)",
        fin_aabb[1][2] > CL_Z + 3.0,
        details=f"fin_zmax={fin_aabb[1][2]:.3f}",
    )

    # --- nacelles: chrome cowl ring at the front, olive panel on top ---
    for nacelle, tag, side in ((left_nacelle, "left", 1.0), (right_nacelle, "right", -1.0)):
        ring_aabb = ctx.part_element_world_aabb(nacelle, elem="cowl_ring")
        ctx.check(
            f"{tag} chrome cowl ring caps the nacelle front",
            ring_aabb is not None and abs(ring_aabb[1][0] - NAC_FRONT_X) < 0.05,
            details=f"ring_xmax={ring_aabb[1][0]:.3f}" if ring_aabb else "missing",
        )
        panel_aabb = ctx.part_element_world_aabb(nacelle, elem="antiglare_panel")
        ctx.check(
            f"{tag} olive anti-glare panel rides on top of the nacelle",
            panel_aabb is not None
            and panel_aabb[0][2] > NAC_Z + 0.4
            and abs((panel_aabb[0][1] + panel_aabb[1][1]) / 2.0 - side * NAC_Y) < 0.05,
            details=f"panel_z=({panel_aabb[0][2]:.2f},{panel_aabb[1][2]:.2f})" if panel_aabb else "missing",
        )
        ctx.expect_overlap(
            nacelle,
            wing,
            axes="x",
            elem_a="nacelle_hull",
            elem_b="wing_loft",
            min_overlap=1.0,
            name=f"{tag} nacelle is carried by the wing (chordwise engagement)",
        )

    # --- markings present ---
    for elem in (
        "fuselage_star_blue_port",
        "fuselage_star_white_port",
        "code_rg_port",
        "code_a_port",
        "nose_art",
    ):
        aabb = ctx.part_element_world_aabb(fuselage, elem=elem)
        ctx.check(f"marking '{elem}' present on the fuselage", aabb is not None, details=str(aabb))
    for elem in ("wing_star_blue_top", "wing_star_white_top", "wing_star_blue_bottom"):
        aabb = ctx.part_element_world_aabb(wing, elem=elem)
        ctx.check(f"marking '{elem}' present on the wing", aabb is not None, details=str(aabb))
    num_aabb = ctx.part_element_world_aabb(tail_fin, elem="tail_number_port")
    ctx.check(
        "tail number 322369 sits below the yellow band",
        num_aabb is not None and num_aabb[1][2] < CL_Z + BAND_Z0 + 0.05,
        details=str(num_aabb),
    )

    # --- tricycle landing gear ---
    nac_r = _nacelle_radius_at(MAIN_GEAR_STATION)
    main_attach_world_z = NAC_Z - nac_r

    for i, (side, tag, nac_parent) in enumerate(
        [(+1.0, "left", left_nacelle), (-1.0, "right", right_nacelle)]
    ):
        gear = object_model.get_part(f"gear_{i}")
        strut_aabb = ctx.part_element_world_aabb(gear, elem=f"gear_{i}_strut")
        wheel_aabb = ctx.part_element_world_aabb(gear, elem=f"gear_{i}_wheel")
        ctx.check(
            f"gear_{i} strut present under {tag} nacelle",
            strut_aabb is not None and strut_aabb[1][2] < main_attach_world_z + 0.05,
            details=f"strut_zmax={strut_aabb[1][2]:.3f}" if strut_aabb else "missing",
        )
        ctx.check(
            f"gear_{i} main wheel touches the ground plane",
            wheel_aabb is not None and abs(wheel_aabb[0][2]) < 0.04,
            details=f"wheel_zmin={wheel_aabb[0][2]:.3f}" if wheel_aabb else "missing",
        )
        # Main gear is laterally centered under its nacelle
        gear_center_y = (wheel_aabb[0][1] + wheel_aabb[1][1]) / 2.0 if wheel_aabb else 0.0
        ctx.check(
            f"gear_{i} is centered laterally under the {tag} nacelle",
            abs(gear_center_y - side * NAC_Y) < 0.20,
            details=f"gear_y={gear_center_y:.3f}, nac_y={side * NAC_Y:.3f}",
        )
        # Main gear is forward of the nacelle rear (anchored on the nacelle body)
        ctx.check(
            f"gear_{i} sits within the nacelle fore-aft extent",
            wheel_aabb is not None
            and wheel_aabb[1][0] < NAC_FRONT_X
            and wheel_aabb[0][0] > NAC_FRONT_X - NAC_LEN,
            details=f"wheel_x=({wheel_aabb[0][0]:.2f},{wheel_aabb[1][0]:.2f})" if wheel_aabb else "missing",
        )

    # Nose gear
    nose_strut_aabb = ctx.part_element_world_aabb(nose_gear, elem="nose_gear_strut")
    nose_wheel_aabb = ctx.part_element_world_aabb(nose_gear, elem="nose_gear_wheel")
    ctx.check(
        "nose gear strut present under forward fuselage",
        nose_strut_aabb is not None and nose_strut_aabb[1][2] < CL_Z - 0.20,
        details=f"strut_zmax={nose_strut_aabb[1][2]:.3f}" if nose_strut_aabb else "missing",
    )
    ctx.check(
        "nose gear wheel touches the ground plane",
        nose_wheel_aabb is not None and abs(nose_wheel_aabb[0][2]) < 0.04,
        details=f"nose_wheel_zmin={nose_wheel_aabb[0][2]:.3f}" if nose_wheel_aabb else "missing",
    )
    ctx.check(
        "nose gear is on the fuselage centerline",
        nose_wheel_aabb is not None
        and abs((nose_wheel_aabb[0][1] + nose_wheel_aabb[1][1]) / 2.0) < 0.10,
        details=str(nose_wheel_aabb),
    )
    ctx.check(
        "nose gear is forward of the main gear (tricycle stance)",
        nose_wheel_aabb is not None and nose_wheel_aabb[0][0] > NAC_FRONT_X - MAIN_GEAR_STATION + 1.0,
        details=f"nose_x={nose_wheel_aabb[0][0]:.2f}" if nose_wheel_aabb else "missing",
    )

    # Level stance: all three wheel contact points at the same ground level
    w0_zmin = ctx.part_element_world_aabb(gear_0, elem="gear_0_wheel")[0][2]
    w1_zmin = ctx.part_element_world_aabb(gear_1, elem="gear_1_wheel")[0][2]
    wn_zmin = ctx.part_element_world_aabb(nose_gear, elem="nose_gear_wheel")[0][2]
    ctx.check(
        "tricycle gear sits level (all wheels within 4 cm of ground)",
        max(abs(w0_zmin), abs(w1_zmin), abs(wn_zmin)) < 0.04,
        details=f"left={w0_zmin:.3f}, right={w1_zmin:.3f}, nose={wn_zmin:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
